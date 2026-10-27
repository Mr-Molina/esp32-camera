//! Entrypoint for `klipper-cam-fw`.
//!
//! When compiled for host (`x86_64` / `aarch64`), runs a self-verification check of the
//! partition table, OTA CRC32 state machine, and dual-port HTTP/MJPEG handlers.
//! When compiled for Xtensa bare-metal (`target_arch = "xtensa"`), initializes the ESP32
//! at `240 MHz`, drives `20 MHz` `XCLK` on `GPIO 21`, probes `I2C0` (`GPIO 26` SDA / `GPIO 27` SCL)
//! for a live `OV2640` sensor, and executes the complete 7-crate hardware-in-the-loop (HIL)
//! verification suite over UART0 (`115200` baud).

#![cfg_attr(target_arch = "xtensa", no_std)]
#![cfg_attr(target_arch = "xtensa", no_main)]
#![cfg_attr(target_arch = "xtensa", feature(asm_experimental_arch))]
#![cfg_attr(not(target_arch = "xtensa"), forbid(unsafe_code))]
#![cfg_attr(
    target_arch = "xtensa",
    allow(
        unsafe_op_in_unsafe_fn,
        static_mut_refs,
        clippy::undocumented_unsafe_blocks,
        clippy::unwrap_used
    )
)]

#[cfg(target_arch = "xtensa")]
extern crate alloc;

#[cfg(target_arch = "xtensa")]
use esp_backtrace as _;

#[cfg(target_arch = "xtensa")]
esp_bootloader_esp_idf::esp_app_desc!();

use esp32_cam_ll::{build_circular_ring, verify_circular_ring, Lldesc, DEFAULT_CAMERA_DMA_BUFFER_SIZE_MAX};
use klipper_cam_fw::{
    calc_dma_sizes, decode_jpeg_to_rgb888, encode_rgb888_to_jpeg, parse_jpeg_dimensions,
    verify_jpeg_eoi, verify_jpeg_soi, verify_partition_table_integrity, BoardPins, CameraDriver,
    CameraSensor, DmaElement, Esp32I2sCamera, EspImageHeader, FirmwareRuntime, FramePool,
    FrameSize, GrabMode, HttpResponseView, Ipv4Addr, KlipperCamConfig, MockSccbBus, OtaSlot,
    Ov2640, PingPongFrameBuffer, PixelFormat, ServerPort, SliceWriter, WroverKitPins, XclkConfig,
    XclkController, DEFAULT_JPEG_QUALITY, DEFAULT_OTA_TOKEN, DEFAULT_XCLK_FREQ_HZ,
    MJPEG_CONTENT_TYPE,
};

#[cfg(target_arch = "xtensa")]
use klipper_cam_fw::write_http_error;

#[inline(never)]
fn hil_check_pins_and_xclk() -> bool {
    if WroverKitPins.validate_pins().is_err() {
        return false;
    }
    if WroverKitPins::XCLK != 21
        || WroverKitPins::SIOD != 26
        || WroverKitPins::SIOC != 27
        || WroverKitPins::VSYNC != 25
        || WroverKitPins::HREF != 23
        || WroverKitPins::PCLK != 22
    {
        return false;
    }

    let mut xclk = XclkController::new();
    let cfg = XclkConfig {
        pin_xclk: WroverKitPins::XCLK,
        ledc_timer: 0,
        ledc_channel: 0,
        xclk_freq_hz: DEFAULT_XCLK_FREQ_HZ,
    };
    if xclk.enable_out_clock(&cfg).is_err()
        || !xclk.is_enabled()
        || xclk.freq_hz() != DEFAULT_XCLK_FREQ_HZ
    {
        return false;
    }
    xclk.disable_out_clock();
    !xclk.is_enabled()
}

#[inline(never)]
fn hil_check_sccb_and_ov2640_hd() -> bool {
    let mut sccb_bus: MockSccbBus<512> = MockSccbBus::new();
    sccb_bus.preset_reg8(Ov2640::SLV_ADDR, esp32_cam_sensors::ov2640::REG_PID, 0x26);
    sccb_bus.preset_reg8(Ov2640::SLV_ADDR, esp32_cam_sensors::ov2640::REG_VER, 0x42);
    sccb_bus.preset_reg8(Ov2640::SLV_ADDR, esp32_cam_sensors::ov2640::REG_MIDH, 0x7F);
    sccb_bus.preset_reg8(Ov2640::SLV_ADDR, esp32_cam_sensors::ov2640::REG_MIDL, 0xA2);

    let sensor_id = match Ov2640::detect(&mut sccb_bus, Ov2640::SLV_ADDR) {
        Ok(id) => id,
        Err(_) => return false,
    };
    let mut ov2640 = Ov2640::from_id(Ov2640::SLV_ADDR, sensor_id);

    if ov2640.reset(&mut sccb_bus).is_err() {
        return false;
    }
    if ov2640
        .set_pixformat(&mut sccb_bus, PixelFormat::Jpeg)
        .is_err()
    {
        return false;
    }
    if ov2640.set_framesize(&mut sccb_bus, FrameSize::Hd).is_err() {
        return false;
    }
    if ov2640
        .set_quality(&mut sccb_bus, DEFAULT_JPEG_QUALITY as i32)
        .is_err()
    {
        return false;
    }

    let hsize8 = sccb_bus.get_reg8(Ov2640::SLV_ADDR, esp32_cam_sensors::ov2640::HSIZE8);
    let vsize8 = sccb_bus.get_reg8(Ov2640::SLV_ADDR, esp32_cam_sensors::ov2640::VSIZE8);
    let yoffl = sccb_bus.get_reg8(Ov2640::SLV_ADDR, esp32_cam_sensors::ov2640::YOFFL);

    hsize8 == 0xC8 && vsize8 == 0x96 && yoffl == 150
}

#[inline(never)]
fn hil_check_ll_dma_and_i2s0() -> bool {
    let mut i2s_cam = Esp32I2sCamera::new();
    let ll_pins = WroverKitPins::PINS.to_ll_pin_config(true);
    if i2s_cam.set_pin(&ll_pins).is_err() || i2s_cam.gpio_bindings().len() != 12 {
        return false;
    }

    let mode = match i2s_cam.set_sample_mode(PixelFormat::Jpeg, DEFAULT_XCLK_FREQ_HZ, 0x26) {
        Ok(m) => m,
        Err(_) => return false,
    };

    let dma_sizing = match calc_dma_sizes(
        true,
        mode.sampling_mode,
        FrameSize::Hd.resolution().width,
        FrameSize::Hd.resolution().height,
        mode.in_bytes_per_pixel,
        DEFAULT_CAMERA_DMA_BUFFER_SIZE_MAX,
    ) {
        Ok(cfg) => cfg,
        Err(_) => return false,
    };
    if dma_sizing.dma_node_cnt == 0 || dma_sizing.dma_buffer_size == 0 {
        return false;
    }

    let mut nodes = [Lldesc::empty(); 4];
    let dma_buf = [0u8; 512];
    if build_circular_ring(&mut nodes, &dma_buf, 128).is_err() {
        return false;
    }
    if verify_circular_ring(&nodes, &dma_buf, 128).is_err() {
        return false;
    }

    i2s_cam.vsync_intr_enable(true);
    if !i2s_cam.is_vsync_intr_enabled() {
        return false;
    }
    if !i2s_cam.start(&dma_sizing, nodes[0].link_addr_20bit()) || !i2s_cam.is_running() {
        return false;
    }
    if !i2s_cam.stop() || i2s_cam.is_running() {
        return false;
    }
    i2s_cam.deinit().is_ok()
}

#[inline(never)]
fn hil_check_jpeg_pool_and_http_ota() -> bool {
    if !verify_partition_table_integrity() {
        return false;
    }

    let width = 16u16;
    let height = 8u16;
    let mut rgb888 = [0u8; 16 * 8 * 3];
    let mut i = 0;
    while i < 16 * 8 {
        rgb888[i * 3] = 40;
        rgb888[i * 3 + 1] = 120;
        rgb888[i * 3 + 2] = 200;
        i += 1;
    }
    let mut jpeg_encoded = [0u8; 640];
    let jpeg_len = match encode_rgb888_to_jpeg(&rgb888, &mut jpeg_encoded, width, height, 85) {
        Ok(len) => len,
        Err(_) => return false,
    };
    if verify_jpeg_soi(&jpeg_encoded[..jpeg_len]) != Some(0) {
        return false;
    }
    if verify_jpeg_eoi(&jpeg_encoded[..jpeg_len], false) != Some(jpeg_len - 2) {
        return false;
    }
    if parse_jpeg_dimensions(&jpeg_encoded[..jpeg_len]) != Some((width, height)) {
        return false;
    }

    let mut decoded_rgb = [0u8; 16 * 8 * 3];
    if decode_jpeg_to_rgb888(&jpeg_encoded[..jpeg_len], &mut decoded_rgb).is_err() {
        return false;
    }

    let el = DmaElement::new(jpeg_encoded[0], 0);
    if el.sample1() != jpeg_encoded[0] {
        return false;
    }

    // Verify RAII FramePool<2, 640> lease and drop return
    let mut pool: FramePool<2, 640> = FramePool::new(GrabMode::Latest);
    if pool
        .push_frame(
            &jpeg_encoded[..jpeg_len],
            width,
            height,
            PixelFormat::Jpeg,
            1000,
        )
        .is_err()
    {
        return false;
    }
    {
        let leased = match pool.take() {
            Ok(g) => g,
            Err(_) => return false,
        };
        if leased.as_slice() != &jpeg_encoded[..jpeg_len] || leased.width() != width {
            return false;
        }
    }
    if pool.free_count() != 2 {
        return false;
    }

    let mut pp_a = [0u8; 640];
    let mut pp_b = [0u8; 640];
    let mut pp = match PingPongFrameBuffer::new(&mut pp_a, &mut pp_b, &jpeg_encoded[..jpeg_len]) {
        Ok(p) => p,
        Err(_) => return false,
    };
    if pp.write_and_commit_inactive(&jpeg_encoded[..jpeg_len]).is_err()
        || pp.frame_seq() != 2
        || pp.active_slot_index() != 1
        || pp.active_frame() != &jpeg_encoded[..jpeg_len]
    {
        return false;
    }

    let fw_cam = CameraDriver::new(&jpeg_encoded[..jpeg_len], FrameSize::Hd);
    let mut runtime = FirmwareRuntime::new(KlipperCamConfig::WROVER_KIT_HD, fw_cam);
    runtime.on_wifi_sta_start();
    if runtime
        .on_wifi_got_ip(Ipv4Addr::new(172, 22, 1, 52))
        .is_err()
    {
        return false;
    }

    let mut resp_buf = [0u8; 1024];
    let status_resp = match runtime.server.handle_raw_request(
        ServerPort::Control,
        b"GET /status HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n",
        &mut resp_buf,
    ) {
        Ok(bytes) => bytes,
        Err(_) => return false,
    };
    match HttpResponseView::parse(status_resp) {
        Ok(view) if view.status_code == 200 => {}
        _ => return false,
    }

    let snap_resp = match runtime.server.handle_raw_request(
        ServerPort::Control,
        b"GET /snapshot HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n",
        &mut resp_buf,
    ) {
        Ok(bytes) => bytes,
        Err(_) => return false,
    };
    match HttpResponseView::parse(snap_resp) {
        Ok(view) if view.status_code == 200 && view.body == &jpeg_encoded[..jpeg_len] => {}
        _ => return false,
    }

    let stream_resp = match runtime.server.handle_raw_request(
        ServerPort::Stream,
        b"GET /stream HTTP/1.1\r\nHost: 172.22.1.52:81\r\n\r\n",
        &mut resp_buf,
    ) {
        Ok(bytes) => bytes,
        Err(_) => return false,
    };
    match HttpResponseView::parse(stream_resp) {
        Ok(view) if view.status_code == 200 && view.content_type == Some(MJPEG_CONTENT_TYPE) => {}
        _ => return false,
    }

    let ota_payload = EspImageHeader::minimal_valid_image_64();
    let mut ota_req = [0u8; 256];
    let req_len = {
        let mut w = SliceWriter::new(&mut ota_req);
        let _ = w.write_str_checked("POST /update HTTP/1.1\r\nHost: 172.22.1.52\r\nX-OTA-Token: ");
        let _ = w.write_str_checked(DEFAULT_OTA_TOKEN);
        let _ = w.write_str_checked("\r\nContent-Length: 64\r\n\r\n");
        let _ = w.write_bytes(&ota_payload);
        w.written_len()
    };
    let ota_resp = match runtime.server.handle_raw_request(
        ServerPort::Control,
        &ota_req[..req_len],
        &mut resp_buf,
    ) {
        Ok(bytes) => bytes,
        Err(_) => return false,
    };
    match HttpResponseView::parse(ota_resp) {
        Ok(view) if view.status_code == 200 => {}
        _ => return false,
    }

    runtime.server.is_reboot_scheduled()
        && runtime.server.last_ota_target_slot() == Some(OtaSlot::Ota1)
        && runtime.server.last_ota_bytes_written() == 64
}

pub fn run_firmware_self_test() -> bool {
    hil_check_pins_and_xclk()
        && hil_check_sccb_and_ov2640_hd()
        && hil_check_ll_dma_and_i2s0()
        && hil_check_jpeg_pool_and_http_ota()
}

#[cfg(not(target_arch = "xtensa"))]
fn main() {
    let ok = run_firmware_self_test();
    assert!(ok, "klipper-cam-fw self-test failed");
}

#[cfg(target_arch = "xtensa")]
#[repr(C, align(4))]
struct AlignedDmaBuf([u8; 32768]);

#[cfg(target_arch = "xtensa")]
static mut I2S_DMA_DESC: [Lldesc; 16] = [Lldesc::empty(); 16];

#[cfg(target_arch = "xtensa")]
static mut I2S_DMA_BUF: AlignedDmaBuf = AlignedDmaBuf([0u8; 32768]);

#[cfg(target_arch = "xtensa")]
#[esp_hal::ram]
fn ensure_spi1_flash_cs0_only() {
    const SPI1_PIN_REG: *mut u32 = 0x3FF4_2034 as *mut u32;
    unsafe {
        let pin = core::ptr::read_volatile(SPI1_PIN_REG);
        // Enforce CS1_DIS = 1 (bit 1, PSRAM CS# disabled) and CS0_DIS = 0 (bit 0, Flash CS# enabled)
        core::ptr::write_volatile(SPI1_PIN_REG, (pin | (1u32 << 1)) & !1u32);
    }
}

#[cfg(target_arch = "xtensa")]
struct EspFlashAdapter<'d>(esp_storage::FlashStorage<'d>);

#[cfg(target_arch = "xtensa")]
impl klipper_cam_fw::OtaFlashStorage for EspFlashAdapter<'_> {
    fn read_aligned(&mut self, offset: u32, buf: &mut [u8]) -> Result<(), klipper_cam_fw::OtaError> {
        ensure_spi1_flash_cs0_only();
        feed_rtc_wdt();
        let res = self
            .0
            .read_nor(offset, buf)
            .map_err(|_| klipper_cam_fw::OtaError::FlashWriteError);
        feed_rtc_wdt();
        res
    }

    fn erase_range(&mut self, from: u32, to: u32) -> Result<(), klipper_cam_fw::OtaError> {
        const ERASE_CHUNK_BYTES: u32 = 64 * 1024;
        ensure_spi1_flash_cs0_only();
        let mut cur = from;
        while cur < to {
            let next = cur.saturating_add(ERASE_CHUNK_BYTES).min(to);
            feed_rtc_wdt();
            self.0
                .erase(cur, next)
                .map_err(|_| klipper_cam_fw::OtaError::FlashWriteError)?;
            feed_rtc_wdt();
            cur = next;
        }
        Ok(())
    }

    fn write_aligned(&mut self, offset: u32, data: &[u8]) -> Result<(), klipper_cam_fw::OtaError> {
        ensure_spi1_flash_cs0_only();
        feed_rtc_wdt();
        let res = self
            .0
            .write_nor(offset, data)
            .map_err(|_| klipper_cam_fw::OtaError::FlashWriteError);
        feed_rtc_wdt();
        res
    }
}

#[cfg(target_arch = "xtensa")]
#[esp_hal::ram]
fn detect_booted_ota_manager_from_mmu() -> klipper_cam_fw::OtaDataManager {
    const DPORT_PRO_FLASH_MMU_TABLE: *const u32 = 0x3FF1_0000 as *const u32;
    let (drom_entry, irom_entry) = unsafe {
        (
            core::ptr::read_volatile(DPORT_PRO_FLASH_MMU_TABLE),
            core::ptr::read_volatile(DPORT_PRO_FLASH_MMU_TABLE.wrapping_add(77)),
        )
    };
    let drom_is_ota1 = (drom_entry & 0x100) == 0 && (drom_entry & 0xFF) >= 0x1D;
    let irom_is_ota1 = (irom_entry & 0x100) == 0 && (irom_entry & 0xFF) >= 0x1D;
    if drom_is_ota1 || irom_is_ota1 {
        klipper_cam_fw::OtaDataManager::from_sectors(
            klipper_cam_fw::OtaDataSelect::new(1, klipper_cam_fw::OtaImgState::Valid),
            klipper_cam_fw::OtaDataSelect::new(2, klipper_cam_fw::OtaImgState::Valid),
        )
    } else {
        klipper_cam_fw::OtaDataManager::new_booted_ota0()
    }
}

#[cfg(target_arch = "xtensa")]
const RTC_BREADCRUMB_MAGIC_HI: u32 = 0xA5A5_0000;

/// ESP32 `RTC_CNTL_STORE6_REG` (`DR_REG_RTCCNTL_BASE + 0xB8`):
/// Bits `[31:16]` = `0xA5A5` magic, bits `[15:0]` = `boot_count` (`u16`).
#[cfg(target_arch = "xtensa")]
const RTC_CNTL_STORE6_REG: *mut u32 = 0x3FF4_80B8 as *mut u32;

/// ESP32 `RTC_CNTL_STORE7_REG` (`DR_REG_RTCCNTL_BASE + 0xBC`):
/// Bits `[31:24]` = `0xA5` magic, bits `[23:16]` = `aid`, bits `[15:8]` = `channel`, bits `[7:0]` = `stage`.
#[cfg(target_arch = "xtensa")]
const RTC_CNTL_STORE7_REG: *mut u32 = 0x3FF4_80BC as *mut u32;

#[cfg(target_arch = "xtensa")]
#[esp_hal::ram]
fn feed_rtc_wdt() {
    const RTC_CNTL_WDTFEED_REG: *mut u32 = 0x3FF4_80A0 as *mut u32;
    const RTC_CNTL_WDTWPROTECT_REG: *mut u32 = 0x3FF4_80A4 as *mut u32;
    const RTC_CNTL_WDT_WKEY: u32 = 0x50D8_3AA1;
    unsafe {
        core::ptr::write_volatile(RTC_CNTL_WDTWPROTECT_REG, RTC_CNTL_WDT_WKEY);
        core::ptr::write_volatile(RTC_CNTL_WDTFEED_REG, 1u32 << 31);
        core::ptr::write_volatile(RTC_CNTL_WDTWPROTECT_REG, 0);
    }
}

#[cfg(target_arch = "xtensa")]
#[esp_hal::ram]
fn arm_rtc_wdt_15s() {
    const RTC_CNTL_WDTCONFIG0_REG: *mut u32 = 0x3FF4_808C as *mut u32;
    const RTC_CNTL_WDTCONFIG1_REG: *mut u32 = 0x3FF4_8090 as *mut u32;
    const RTC_CNTL_WDTFEED_REG: *mut u32 = 0x3FF4_80A0 as *mut u32;
    const RTC_CNTL_WDTWPROTECT_REG: *mut u32 = 0x3FF4_80A4 as *mut u32;
    const RTC_CNTL_WDT_WKEY: u32 = 0x50D8_3AA1;
    // WDT_EN (bit 31) | WDT_STG0 = 4 (RESET_RTC, bits 28..=30) | WDT_CPU_RESET_LENGTH = 7 (bits 14..=16)
    // | WDT_SYS_RESET_LENGTH = 7 (bits 11..=13) | WDT_FLASHBOOT_MOD_EN (bit 10)
    const WDT_CFG0_VAL: u32 =
        (1u32 << 31) | (4u32 << 28) | (7u32 << 14) | (7u32 << 11) | (1u32 << 10);
    unsafe {
        core::ptr::write_volatile(RTC_CNTL_WDTWPROTECT_REG, RTC_CNTL_WDT_WKEY);
        core::ptr::write_volatile(RTC_CNTL_WDTCONFIG1_REG, 15 * 150_000);
        core::ptr::write_volatile(RTC_CNTL_WDTCONFIG0_REG, WDT_CFG0_VAL);
        core::ptr::write_volatile(RTC_CNTL_WDTFEED_REG, 1u32 << 31);
        core::ptr::write_volatile(RTC_CNTL_WDTWPROTECT_REG, 0);
    }
}

/// Applies `ESP-IDF v5.5.4` (`rtc_init.c:30-31, 102-107`) Wi-Fi/RF modem force power-down
/// and isolation during early peripheral initialization, and releases any latched RTC pad holds.
#[cfg(target_arch = "xtensa")]
#[esp_hal::ram]
fn esp_rtc_wifi_force_pd(enable_pd: bool) {
    const RTC_CNTL_DIG_PWC_REG: *mut u32 = 0x3FF4_8084 as *mut u32;
    const RTC_CNTL_DIG_ISO_REG: *mut u32 = 0x3FF4_8088 as *mut u32;
    const RTC_CNTL_HOLD_FORCE_REG: *mut u32 = 0x3FF4_80C8 as *mut u32;
    const RTC_CNTL_WIFI_FORCE_PD: u32 = 1u32 << 17;
    const RTC_CNTL_WIFI_FORCE_ISO: u32 = 1u32 << 28;
    const RTC_CNTL_DG_PAD_FORCE_HOLD: u32 = 1u32 << 15;
    const RTC_CNTL_DG_PAD_FORCE_UNHOLD: u32 = 1u32 << 14;
    const RTC_CNTL_DG_PAD_FORCE_ISO: u32 = 1u32 << 13;
    const RTC_CNTL_DG_PAD_FORCE_NOISO: u32 = 1u32 << 12;
    const RTC_CNTL_DG_PAD_AUTOHOLD_EN: u32 = 1u32 << 11;
    const RTC_CNTL_DG_PAD_MASK: u32 = RTC_CNTL_DG_PAD_FORCE_HOLD
        | RTC_CNTL_DG_PAD_FORCE_UNHOLD
        | RTC_CNTL_DG_PAD_FORCE_ISO
        | RTC_CNTL_DG_PAD_FORCE_NOISO
        | RTC_CNTL_DG_PAD_AUTOHOLD_EN;

    unsafe {
        core::ptr::write_volatile(RTC_CNTL_HOLD_FORCE_REG, 0);
        let iso = core::ptr::read_volatile(RTC_CNTL_DIG_ISO_REG) & !RTC_CNTL_DG_PAD_MASK;
        let pwc = core::ptr::read_volatile(RTC_CNTL_DIG_PWC_REG);
        if enable_pd {
            core::ptr::write_volatile(RTC_CNTL_DIG_ISO_REG, iso | RTC_CNTL_WIFI_FORCE_ISO);
            core::ptr::write_volatile(RTC_CNTL_DIG_PWC_REG, pwc | RTC_CNTL_WIFI_FORCE_PD);
        } else {
            core::ptr::write_volatile(RTC_CNTL_DIG_PWC_REG, pwc & !RTC_CNTL_WIFI_FORCE_PD);
            core::ptr::write_volatile(RTC_CNTL_DIG_ISO_REG, iso & !RTC_CNTL_WIFI_FORCE_ISO);
        }
    }
}

#[cfg(target_arch = "xtensa")]
struct BreadcrumbPrevSummary {
    new_boots: u32,
    prev_magic: u32,
    prev_boots: u32,
    prev_reset: u32,
    prev_stage: u32,
    prev_ch: u32,
    prev_aid: u32,
}

#[cfg(target_arch = "xtensa")]
#[esp_hal::ram]
fn record_boot_breadcrumb_rtc_init(reset_reason: u32) -> BreadcrumbPrevSummary {
    let raw6 = unsafe { core::ptr::read_volatile(RTC_CNTL_STORE6_REG) };
    let raw7 = unsafe { core::ptr::read_volatile(RTC_CNTL_STORE7_REG) };

    let prev_magic = raw6 & 0xFFFF_0000;
    let prev_boots = if prev_magic == RTC_BREADCRUMB_MAGIC_HI {
        raw6 & 0x0000_FFFF
    } else {
        0
    };
    let (prev_stage, prev_ch, prev_aid) = if (raw7 >> 24) == 0xA5 {
        (raw7 & 0xFF, (raw7 >> 8) & 0xFF, (raw7 >> 16) & 0xFF)
    } else {
        (0, 0, 0)
    };

    let new_boots = (prev_boots.wrapping_add(1)) & 0xFFFF;
    unsafe {
        core::ptr::write_volatile(RTC_CNTL_STORE6_REG, RTC_BREADCRUMB_MAGIC_HI | new_boots);
        core::ptr::write_volatile(
            RTC_CNTL_STORE7_REG,
            (0xA5u32 << 24) | ((prev_aid & 0xFF) << 16) | ((prev_ch & 0xFF) << 8) | 1,
        );
    }

    BreadcrumbPrevSummary {
        new_boots,
        prev_magic,
        prev_boots,
        prev_reset: reset_reason,
        prev_stage,
        prev_ch,
        prev_aid,
    }
}

#[cfg(target_arch = "xtensa")]
#[esp_hal::ram]
fn record_boot_breadcrumb_post_wifi(stage: u32, ch: u8, aid: u16) {
    let val = (0xA5u32 << 24)
        | ((aid as u32 & 0xFF) << 16)
        | ((ch as u32 & 0xFF) << 8)
        | (stage & 0xFF);
    unsafe {
        core::ptr::write_volatile(RTC_CNTL_STORE7_REG, val);
    }
}

#[cfg(target_arch = "xtensa")]
struct SmoltcpWifiDevice(esp_radio::wifi::Interface);

#[cfg(target_arch = "xtensa")]
struct SmoltcpRxToken(esp_radio::wifi::WifiRxToken);

#[cfg(target_arch = "xtensa")]
struct SmoltcpTxToken(esp_radio::wifi::WifiTxToken);

#[cfg(target_arch = "xtensa")]
impl smoltcp::phy::RxToken for SmoltcpRxToken {
    fn consume<R, F>(self, f: F) -> R
    where
        F: FnOnce(&[u8]) -> R,
    {
        self.0.consume_token(|buf| f(buf))
    }
}

#[cfg(target_arch = "xtensa")]
impl smoltcp::phy::TxToken for SmoltcpTxToken {
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        let copy_len = len.min(1480);
        self.0.consume_token(copy_len, |tx_buf| f(&mut tx_buf[..copy_len]))
    }
}

#[cfg(target_arch = "xtensa")]
impl smoltcp::phy::Device for SmoltcpWifiDevice {
    type RxToken<'a> = SmoltcpRxToken where Self: 'a;
    type TxToken<'a> = SmoltcpTxToken where Self: 'a;

    fn receive(
        &mut self,
        _timestamp: smoltcp::time::Instant,
    ) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        self.0
            .receive()
            .map(|(rx, tx)| (SmoltcpRxToken(rx), SmoltcpTxToken(tx)))
    }

    fn transmit(&mut self, _timestamp: smoltcp::time::Instant) -> Option<Self::TxToken<'_>> {
        self.0.transmit().map(SmoltcpTxToken)
    }

    fn capabilities(&self) -> smoltcp::phy::DeviceCapabilities {
        let mut caps = smoltcp::phy::DeviceCapabilities::default();
        caps.medium = smoltcp::phy::Medium::Ethernet;
        // Must be <= esp-radio's `ESP_RADIO_CONFIG_WIFI_MTU` (1492) so `WifiTxToken::consume_token`
        // stack buffer `[u8; 1492]` never panics on slice indexing `&mut buffer[..len]`.
        caps.max_transmission_unit = 1480;
        caps.max_burst_size = Some(10);
        caps
    }
}

#[cfg(target_arch = "xtensa")]
fn now_millis() -> u64 {
    esp_hal::time::Instant::now()
        .duration_since_epoch()
        .as_millis()
}

#[cfg(target_arch = "xtensa")]
fn smol_now() -> smoltcp::time::Instant {
    smoltcp::time::Instant::from_millis(now_millis() as i64)
}

#[cfg(target_arch = "xtensa")]
fn block_on_with_timeout<F: core::future::Future>(
    mut fut: F,
    timeout_ms: u64,
) -> Option<F::Output> {
    use core::{
        pin::Pin,
        task::{Context, Poll, RawWaker, RawWakerVTable, Waker},
    };
    const VTABLE: RawWakerVTable = RawWakerVTable::new(
        |p| RawWaker::new(p, &VTABLE),
        |_| {},
        |_| {},
        |_| {},
    );
    let waker = unsafe { Waker::from_raw(RawWaker::new(core::ptr::null(), &VTABLE)) };
    let mut cx = Context::from_waker(&waker);
    let mut fut = unsafe { Pin::new_unchecked(&mut fut) };
    let start_ms = now_millis();
    loop {
        feed_rtc_wdt();
        if let Poll::Ready(res) = fut.as_mut().poll(&mut cx) {
            return Some(res);
        }
        if now_millis().saturating_sub(start_ms) >= timeout_ms {
            return None;
        }
    }
}

#[cfg(target_arch = "xtensa")]
fn send_arp_announce(
    net_device: &mut SmoltcpWifiDevice,
    mac: [u8; 6],
    src_ip: [u8; 4],
    target_ip: [u8; 4],
) {
    if let Some(tx_tok) = net_device.0.transmit() {
        tx_tok.consume_token(42, |buf| {
            // Destination MAC: Broadcast (FF:FF:FF:FF:FF:FF)
            buf[0..6].copy_from_slice(&[0xFF; 6]);
            // Source MAC
            buf[6..12].copy_from_slice(&mac);
            // EtherType: ARP (0x0806)
            buf[12] = 0x08;
            buf[13] = 0x06;
            // Hardware type: Ethernet (1), Protocol type: IPv4 (0x0800)
            buf[14..20].copy_from_slice(&[0x00, 0x01, 0x08, 0x00, 0x06, 0x04]);
            // Opcode: ARP Request (1)
            buf[20] = 0x00;
            buf[21] = 0x01;
            // Sender Hardware Address (MAC) + Sender Protocol Address (src_ip)
            buf[22..28].copy_from_slice(&mac);
            buf[28..32].copy_from_slice(&src_ip);
            // Target Hardware Address (00:00:00:00:00:00) + Target Protocol Address (target_ip)
            buf[32..38].copy_from_slice(&[0x00; 6]);
            buf[38..42].copy_from_slice(&target_ip);
        });
    }
}

#[cfg(target_arch = "xtensa")]
const PORT80_SOCKET_COUNT: usize = 4;

#[cfg(target_arch = "xtensa")]
const PORT81_SOCKET_COUNT: usize = 2;

#[cfg(target_arch = "xtensa")]
const TCP_MSS_BYTES: usize = 1426;

#[cfg(target_arch = "xtensa")]
const TCP_BATCH_SEGMENTS: usize = 10;

#[cfg(target_arch = "xtensa")]
const TCP_MAX_INFLIGHT_BYTES: usize = TCP_BATCH_SEGMENTS * TCP_MSS_BYTES;

#[cfg(target_arch = "xtensa")]
fn enqueue_tcp_sliding_window(
    sock: &mut smoltcp::socket::tcp::Socket<'_>,
    buf: &[u8],
    tx_off: &mut usize,
    tx_len: usize,
) {
    let limit = tx_len.min(buf.len());
    if *tx_off >= limit || !sock.can_send() {
        return;
    }
    if sock.send_queue() == 0 {
        let chunk_end = (*tx_off + TCP_MAX_INFLIGHT_BYTES).min(limit);
        if chunk_end > *tx_off {
            if let Ok(sent) = sock.send_slice(&buf[*tx_off..chunk_end]) {
                *tx_off += sent;
            }
        }
    }
}

#[cfg(target_arch = "xtensa")]
fn poll_net_burst(
    iface: &mut smoltcp::iface::Interface,
    net_device: &mut SmoltcpWifiDevice,
    sockets: &mut smoltcp::iface::SocketSet<'_>,
) {
    for _ in 0..10u8 {
        let now = smol_now();
        let _ = iface.poll(now, net_device, sockets);
        if iface.poll_delay(now, sockets) != Some(smoltcp::time::Duration::ZERO) {
            break;
        }
    }
}

#[cfg(target_arch = "xtensa")]
#[esp_hal::ram]
fn hold_psram_cs_high_safe() {
    const GPIO_OUT_W1TS_REG: *mut u32 = 0x3FF4_4008 as *mut u32;
    const GPIO_ENABLE_W1TS_REG: *mut u32 = 0x3FF4_4024 as *mut u32;
    const GPIO_FUNC16_OUT_SEL_CFG_REG: *mut u32 = 0x3FF4_4570 as *mut u32;
    const IO_MUX_GPIO16_REG: *mut u32 = 0x3FF4_904C as *mut u32;
    const SIG_GPIO_OUT_IDX: u32 = 0x100;
    // MCU_SEL = Function 2 (GPIO = 2 << 12), FUN_IE = 1 << 9, FUN_WPU = 1 << 8, FUN_DRV = 2 << 10
    const IO_MUX_GPIO16_WPU_GPIO: u32 = (2 << 12) | (2 << 10) | (1 << 9) | (1 << 8);

    unsafe {
        core::ptr::write_volatile(GPIO_OUT_W1TS_REG, 1 << 16);
        core::ptr::write_volatile(GPIO_FUNC16_OUT_SEL_CFG_REG, SIG_GPIO_OUT_IDX);
        core::ptr::write_volatile(IO_MUX_GPIO16_REG, IO_MUX_GPIO16_WPU_GPIO);
        core::ptr::write_volatile(GPIO_ENABLE_W1TS_REG, 1 << 16);
    }
}

/// Sends an 8-bit PSRAM command (`0xF5` in 4-bit QPI mode when `qpi_mode == true`, or
/// `0x35` in 1-bit SPI mode when `qpi_mode == false`) over the hardware `SPI1` controller
/// using `SPI1_CS1` (`GPIO 16`) and `SPICLK` (`GPIO 17`), ported directly from
/// `ESP-IDF v5.5.4` `esp_psram_impl_quad.c:250-535`.
///
/// Never disconnects or bit-bangs `SPI0`/`SPI1` data pads (`SD0..SD3`).
#[cfg(target_arch = "xtensa")]
#[esp_hal::ram]
fn psram_spi1_cmd(qpi_mode: bool, cmd_byte: u8) {
    const DPORT_HOST_INF_SEL_REG: *mut u32 = 0x3FF0_00BC as *mut u32;
    const SPI0_PIN_REG: *mut u32 = 0x3FF4_3034 as *mut u32;
    const SPI0_EXT2_REG: *const u32 = 0x3FF4_30F8 as *const u32;
    const SPI1_CMD_REG: *mut u32 = 0x3FF4_2000 as *mut u32;
    const SPI1_ADDR_REG: *mut u32 = 0x3FF4_2004 as *mut u32;
    const SPI1_CTRL_REG: *mut u32 = 0x3FF4_2008 as *mut u32;
    const SPI1_USER_REG: *mut u32 = 0x3FF4_201C as *mut u32;
    const SPI1_USER1_REG: *mut u32 = 0x3FF4_2020 as *mut u32;
    const SPI1_USER2_REG: *mut u32 = 0x3FF4_2024 as *mut u32;
    const SPI1_MOSI_DLEN_REG: *mut u32 = 0x3FF4_2028 as *mut u32;
    const SPI1_PIN_REG: *mut u32 = 0x3FF4_2034 as *mut u32;
    const SPI1_W0_REG: *mut u32 = 0x3FF4_2080 as *mut u32;
    const SPI1_EXT2_REG: *const u32 = 0x3FF4_20F8 as *const u32;

    const GPIO_ENABLE_W1TS_REG: *mut u32 = 0x3FF4_4024 as *mut u32;
    const GPIO_FUNC16_OUT_SEL_CFG_REG: *mut u32 = 0x3FF4_4570 as *mut u32;
    const GPIO_FUNC17_OUT_SEL_CFG_REG: *mut u32 = 0x3FF4_4574 as *mut u32;
    const IO_MUX_GPIO16_REG: *mut u32 = 0x3FF4_904C as *mut u32;
    const IO_MUX_GPIO17_REG: *mut u32 = 0x3FF4_9050 as *mut u32;

    const SPICLK_OUT_IDX: u32 = 0;
    const SPICS1_OUT_IDX: u32 = 6;
    const SPI_USR: u32 = 1u32 << 18;

    // SPI_CTRL_REG bits
    const SPI_FREAD_DUAL: u32 = 1u32 << 14;
    const SPI_FREAD_QUAD: u32 = 1u32 << 20;
    const SPI_WP_REG: u32 = 1u32 << 21;
    const SPI_FREAD_DIO: u32 = 1u32 << 23;
    const SPI_FREAD_QIO: u32 = 1u32 << 24;
    const SPI_FREAD_MODE_MASK: u32 =
        SPI_FREAD_DUAL | SPI_FREAD_QUAD | SPI_FREAD_DIO | SPI_FREAD_QIO;

    // SPI_USER_REG bits
    const SPI_CS_HOLD: u32 = 1u32 << 4;
    const SPI_CS_SETUP: u32 = 1u32 << 5;
    const SPI_FWRITE_QIO: u32 = 1u32 << 15;
    const SPI_USR_MOSI: u32 = 1u32 << 27;
    const SPI_USR_ADDR: u32 = 1u32 << 30;

    unsafe {
        let saved_ps: u32;
        core::arch::asm!("rsil {0}, 15", out(reg) saved_ps);

        let mut spin = 0u32;
        while ((core::ptr::read_volatile(SPI0_EXT2_REG) & 0x7) != 0
            || (core::ptr::read_volatile(SPI1_EXT2_REG) & 0x7) != 0
            || (core::ptr::read_volatile(SPI1_CMD_REG) & SPI_USR) != 0)
            && spin < 100_000
        {
            spin = spin.wrapping_add(1);
        }

        let saved_spi0_pin = core::ptr::read_volatile(SPI0_PIN_REG);
        let saved_user = core::ptr::read_volatile(SPI1_USER_REG);
        let saved_user1 = core::ptr::read_volatile(SPI1_USER1_REG);
        let saved_user2 = core::ptr::read_volatile(SPI1_USER2_REG);
        let saved_mosi_dlen = core::ptr::read_volatile(SPI1_MOSI_DLEN_REG);
        let saved_w0 = core::ptr::read_volatile(SPI1_W0_REG);
        let saved_pin = core::ptr::read_volatile(SPI1_PIN_REG);
        let saved_ctrl = core::ptr::read_volatile(SPI1_CTRL_REG);
        let saved_addr = core::ptr::read_volatile(SPI1_ADDR_REG);
        let saved_outsel16 = core::ptr::read_volatile(GPIO_FUNC16_OUT_SEL_CFG_REG);
        let saved_iomux16 = core::ptr::read_volatile(IO_MUX_GPIO16_REG);
        let saved_outsel17 = core::ptr::read_volatile(GPIO_FUNC17_OUT_SEL_CFG_REG);
        let saved_iomux17 = core::ptr::read_volatile(IO_MUX_GPIO17_REG);

        // Temporarily disable CS1 on SPI0 (SPI_CS1_DIS = 1 << 1) before routing GPIO 16 to SPI1 SPICS1_OUT_IDX (6)
        core::ptr::write_volatile(SPI0_PIN_REG, saved_spi0_pin | (1u32 << 1));

        // Route GPIO 16 -> SPICS1_OUT_IDX (6) with pull-up (0x2B00) and GPIO 17 -> SPICLK_OUT_IDX (0)
        core::ptr::write_volatile(GPIO_FUNC16_OUT_SEL_CFG_REG, SPICS1_OUT_IDX);
        core::ptr::write_volatile(IO_MUX_GPIO16_REG, 0x2B00);
        core::ptr::write_volatile(GPIO_FUNC17_OUT_SEL_CFG_REG, SPICLK_OUT_IDX);
        core::ptr::write_volatile(IO_MUX_GPIO17_REG, 0x2800);
        core::ptr::write_volatile(GPIO_ENABLE_W1TS_REG, (1u32 << 16) | (1u32 << 17));

        // Select CS1 (bit 1 = 0) and disable CS0 (bit 0 = 1) on SPI1
        core::ptr::write_volatile(SPI1_PIN_REG, (saved_pin | 1u32) & !(1u32 << 1));

        if qpi_mode {
            // Matches esp_psram_impl_quad.c:422-450 (psram_disable_qio_mode):
            // 8-bit 0xF5 payload in SPI1_W0_REG via USR_MOSI + FWRITE_QIO (2 QPI nibbles).
            core::ptr::write_volatile(
                SPI1_CTRL_REG,
                (saved_ctrl & !SPI_FREAD_MODE_MASK) | SPI_FREAD_QIO | SPI_WP_REG,
            );
            core::ptr::write_volatile(SPI1_W0_REG, cmd_byte as u32);
            core::ptr::write_volatile(SPI1_MOSI_DLEN_REG, 7u32);
            core::ptr::write_volatile(
                SPI1_USER_REG,
                SPI_USR_MOSI | SPI_FWRITE_QIO | SPI_CS_SETUP | SPI_CS_HOLD,
            );
        } else {
            // Matches esp_psram_impl_quad.c:497-535 (psram_enable_qio_mode_spi1):
            // 8-bit 0x35 command in SPI1_ADDR_REG[31:24] via USR_ADDR in 1-bit SPI mode.
            core::ptr::write_volatile(
                SPI1_CTRL_REG,
                (saved_ctrl & !SPI_FREAD_MODE_MASK) | SPI_WP_REG,
            );
            core::ptr::write_volatile(SPI1_ADDR_REG, (cmd_byte as u32) << 24);
            core::ptr::write_volatile(SPI1_USER1_REG, (7u32 & 0x3F) << 26);
            core::ptr::write_volatile(
                SPI1_USER_REG,
                SPI_USR_ADDR | SPI_CS_SETUP | SPI_CS_HOLD,
            );
        }

        let mut wait0 = 0u32;
        while (core::ptr::read_volatile(SPI0_EXT2_REG) & 0x7) != 0 && wait0 < 100_000 {
            wait0 = wait0.wrapping_add(1);
        }

        core::ptr::write_volatile(
            DPORT_HOST_INF_SEL_REG,
            core::ptr::read_volatile(DPORT_HOST_INF_SEL_REG) | (1u32 << 14),
        );
        core::ptr::write_volatile(SPI1_CMD_REG, SPI_USR);

        let mut wait1 = 0u32;
        while ((core::ptr::read_volatile(SPI1_CMD_REG) & SPI_USR) != 0
            || (core::ptr::read_volatile(SPI1_EXT2_REG) & 0x7) != 0)
            && wait1 < 100_000
        {
            wait1 = wait1.wrapping_add(1);
        }

        core::ptr::write_volatile(
            DPORT_HOST_INF_SEL_REG,
            core::ptr::read_volatile(DPORT_HOST_INF_SEL_REG) & !(1u32 << 14),
        );

        core::ptr::write_volatile(SPI1_CTRL_REG, saved_ctrl);
        core::ptr::write_volatile(SPI1_USER_REG, saved_user);
        core::ptr::write_volatile(SPI1_USER1_REG, saved_user1);
        core::ptr::write_volatile(SPI1_USER2_REG, saved_user2);
        core::ptr::write_volatile(SPI1_MOSI_DLEN_REG, saved_mosi_dlen);
        core::ptr::write_volatile(SPI1_W0_REG, saved_w0);
        // Enforce CS1_DIS = 1 (bit 1) and CS0_DIS = 0 (bit 0) on restore (esp_psram_impl_quad.c:327-328)
        core::ptr::write_volatile(SPI1_PIN_REG, (saved_pin | (1u32 << 1)) & !1u32);
        core::ptr::write_volatile(SPI1_ADDR_REG, saved_addr);
        core::ptr::write_volatile(GPIO_FUNC16_OUT_SEL_CFG_REG, saved_outsel16);
        core::ptr::write_volatile(IO_MUX_GPIO16_REG, saved_iomux16 | (1u32 << 8));
        core::ptr::write_volatile(GPIO_FUNC17_OUT_SEL_CFG_REG, saved_outsel17);
        core::ptr::write_volatile(IO_MUX_GPIO17_REG, saved_iomux17);
        core::ptr::write_volatile(SPI0_PIN_REG, saved_spi0_pin);

        core::arch::asm!("wsr.ps {0}; rsync", in(reg) saved_ps);
    }
}

#[cfg(target_arch = "xtensa")]
#[esp_hal::ram]
fn release_psram_cs_and_exit_qpi() {
    psram_spi1_cmd(true, 0xF5);
    hold_psram_cs_high_safe();
}

/// Ensures external PSRAM enters 4-bit QPI mode cleanly after `Psram::new()`.
///
/// `esp-hal-1.2.2`'s `psram_enable_qio_mode_spi1` (`esp32.rs:753`) erroneously sends `0x35`
/// (`PSRAM_ENTER_QMODE`) in `PsramCmdQpi` mode instead of 1-bit `PSRAM_CMD_SPI` mode
/// (`esp_psram_impl_quad.c:519`). Calling `psram_spi1_cmd(true, 0xF5)` followed by
/// `psram_spi1_cmd(false, 0x35)` guarantees PSRAM enters QPI mode on both cold `0V -> 3.3V`
/// power-on resets and warm reboots.
#[cfg(target_arch = "xtensa")]
#[esp_hal::ram]
fn ensure_psram_qpi_mode_spi1() {
    const GPIO_FUNC16_OUT_SEL_CFG_REG: *mut u32 = 0x3FF4_4570 as *mut u32;
    const IO_MUX_GPIO16_REG: *mut u32 = 0x3FF4_904C as *mut u32;
    const SPICS1_OUT_IDX: u32 = 6;

    psram_spi1_cmd(true, 0xF5);
    psram_spi1_cmd(false, 0x35);
    unsafe {
        core::ptr::write_volatile(GPIO_FUNC16_OUT_SEL_CFG_REG, SPICS1_OUT_IDX);
        core::ptr::write_volatile(
            IO_MUX_GPIO16_REG,
            core::ptr::read_volatile(IO_MUX_GPIO16_REG) | (1u32 << 8),
        );
    }
}

#[cfg(target_arch = "xtensa")]
#[esp_hal::ram]
fn quiesce_psram_and_software_reset() -> ! {
    release_psram_cs_and_exit_qpi();
    unsafe {
        core::arch::asm!("rsil {0}, 15", out(reg) _);
    }
    esp_hal::system::software_reset()
}

#[cfg(target_arch = "xtensa")]
#[no_mangle]
pub extern "C" fn custom_halt() -> ! {
    quiesce_psram_and_software_reset()
}

#[cfg(target_arch = "xtensa")]
const WIFI_MAX_TX_POWER_QDBM: i8 = 60;

#[cfg(target_arch = "xtensa")]
#[inline(never)]
fn connect_strongest_ap(
    wifi_controller: &mut esp_radio::wifi::WifiController<'_>,
    base_sta_cfg: &esp_radio::wifi::sta::StationConfig,
    ssid_str: &str,
    timeout_ms: u64,
) -> Option<esp_radio::wifi::sta::ConnectedInfo> {
    use esp_hal::time::Duration;
    use esp_println::println;
    use esp_radio::wifi::{
        scan::{ScanConfig, ScanTypeConfig},
        sta::ScanMethod,
        Config as WifiConfig, PowerSaveMode,
    };

    let wait_ms = timeout_ms.max(15_000);
    let _ = wifi_controller.set_power_saving(PowerSaveMode::None);
    let _ = wifi_controller.set_max_tx_power(WIFI_MAX_TX_POWER_QDBM);

    // 1. Active scan (60..100 ms per channel, with a second pass if best RSSI < -62 dBm)
    //    to pick the strongest AP at the camera's current physical location.
    let mut scan_cfg = ScanConfig::default()
        .with_max(16)
        .with_scan_type(ScanTypeConfig::Active {
            min: Duration::from_millis(60),
            max: Duration::from_millis(100),
        });
    if let Ok(s) = ssid_str.try_into() {
        scan_cfg = scan_cfg.with_ssid(s);
    }

    let mut scan_completed = false;
    let mut best_target: Option<(u8, i8, [u8; 6])> = None;
    if let Some(res) = block_on_with_timeout(wifi_controller.scan_async(&scan_cfg), 5000) {
        scan_completed = true;
        if let Ok(aps) = res {
            if let Some(ap) = aps
                .into_iter()
                .filter(|ap| ap.ssid.as_str() == ssid_str)
                .max_by_key(|ap| ap.signal_strength)
            {
                best_target = Some((ap.channel, ap.signal_strength, ap.bssid));
            }
        }
    }
    if scan_completed && best_target.is_none_or(|(_, rssi, _)| rssi < -62) {
        scan_completed = false;
        if let Some(res2) = block_on_with_timeout(wifi_controller.scan_async(&scan_cfg), 5000) {
            scan_completed = true;
            if let Ok(aps2) = res2 {
                if let Some(ap2) = aps2
                    .into_iter()
                    .filter(|ap| ap.ssid.as_str() == ssid_str)
                    .max_by_key(|ap| ap.signal_strength)
                {
                    if best_target.is_none_or(|(_, best_rssi, _)| ap2.signal_strength > best_rssi) {
                        best_target = Some((ap2.channel, ap2.signal_strength, ap2.bssid));
                    }
                }
            }
        }
    }
    if scan_completed {
        if let Some((target_ch, target_rssi, target_bssid)) = best_target {
            println!(
                "[KLIPPER-CAM-RS] [WIFI] Strongest '{}' AP: ch={} rssi={}dBm bssid={:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
                ssid_str,
                target_ch,
                target_rssi,
                target_bssid[0],
                target_bssid[1],
                target_bssid[2],
                target_bssid[3],
                target_bssid[4],
                target_bssid[5]
            );
            let locked_cfg = base_sta_cfg
                .clone()
                .with_bssid(target_bssid)
                .with_channel(target_ch)
                .with_scan_method(ScanMethod::Fast)
                .with_failure_retry_cnt(2);
            if wifi_controller
                .set_config(&WifiConfig::Station(locked_cfg))
                .is_ok()
            {
                let _ = wifi_controller.set_max_tx_power(WIFI_MAX_TX_POWER_QDBM);
                match block_on_with_timeout(wifi_controller.connect_async(), 10_000) {
                    Some(Ok(info)) => {
                        let _ = wifi_controller.set_power_saving(PowerSaveMode::None);
                        let _ = wifi_controller.set_max_tx_power(WIFI_MAX_TX_POWER_QDBM);
                        return Some(info);
                    }
                    Some(Err(e)) => {
                        println!(
                            "[KLIPPER-CAM-RS] [WIFI] BSSID-locked connect_async error: {:?}",
                            e
                        );
                        let _ = block_on_with_timeout(wifi_controller.disconnect_async(), 2000);
                        esp_hal::delay::Delay::new().delay_millis(100);
                        let _ =
                            wifi_controller.set_config(&WifiConfig::Station(base_sta_cfg.clone()));
                    }
                    None => {
                        println!(
                            "[KLIPPER-CAM-RS] [WIFI] BSSID-locked connect_async timed out (10000 ms)"
                        );
                        let _ = block_on_with_timeout(wifi_controller.disconnect_async(), 2000);
                        esp_hal::delay::Delay::new().delay_millis(100);
                        let _ =
                            wifi_controller.set_config(&WifiConfig::Station(base_sta_cfg.clone()));
                    }
                }
            } else {
                let _ = wifi_controller.set_config(&WifiConfig::Station(base_sta_cfg.clone()));
            }
        }
    }

    // 2. Unlocked fallback (or primary path if scan found no target): full 15,000 ms timeout
    //    across all channels with WIFI_CONNECT_AP_BY_SIGNAL and failure_retry_cnt(2).
    let _ = wifi_controller.set_max_tx_power(WIFI_MAX_TX_POWER_QDBM);
    match block_on_with_timeout(wifi_controller.connect_async(), wait_ms) {
        Some(Ok(info)) => {
            let _ = wifi_controller.set_power_saving(PowerSaveMode::None);
            let _ = wifi_controller.set_max_tx_power(WIFI_MAX_TX_POWER_QDBM);
            Some(info)
        }
        Some(Err(e)) => {
            println!(
                "[KLIPPER-CAM-RS] [WIFI] connect_async('{}') error: {:?}",
                ssid_str, e
            );
            let _ = block_on_with_timeout(wifi_controller.disconnect_async(), 2000);
            esp_hal::delay::Delay::new().delay_millis(200);
            None
        }
        None => {
            println!(
                "[KLIPPER-CAM-RS] [WIFI] connect_async('{}') timed out ({} ms)",
                ssid_str, wait_ms
            );
            let _ = block_on_with_timeout(wifi_controller.disconnect_async(), 2000);
            esp_hal::delay::Delay::new().delay_millis(200);
            None
        }
    }
}

#[cfg(target_arch = "xtensa")]
#[derive(Debug, Clone, Copy, Default)]
struct Port80WorkerState {
    req_len: usize,
    tx_off: usize,
    tx_len: usize,
    closing: bool,
    close_deadline_ms: i64,
    using_ota_accum: bool,
    req_started_ms: u64,
    conn_active_ms: i64,
    last_send_queue: usize,
    is_snapshot: bool,
}

#[cfg(target_arch = "xtensa")]
impl Port80WorkerState {
    const EMPTY: Self = Self {
        req_len: 0,
        tx_off: 0,
        tx_len: 0,
        closing: false,
        close_deadline_ms: 0,
        using_ota_accum: false,
        req_started_ms: 0,
        conn_active_ms: 0,
        last_send_queue: 0,
        is_snapshot: false,
    };
}

#[cfg(target_arch = "xtensa")]
#[allow(dead_code)]
type Port80SocketState = Port80WorkerState;

#[cfg(target_arch = "xtensa")]
#[derive(Debug, Clone, Copy, Default)]
struct Port81WorkerState {
    req_len: usize,
    closing: bool,
    close_deadline_ms: i64,
    req_started_ms: u64,
    conn_active_ms: i64,
}

#[cfg(target_arch = "xtensa")]
impl Port81WorkerState {
    const EMPTY: Self = Self {
        req_len: 0,
        closing: false,
        close_deadline_ms: 0,
        req_started_ms: 0,
        conn_active_ms: 0,
    };
}

#[cfg(target_arch = "xtensa")]
#[allow(dead_code)]
type Port81SocketState = Port81WorkerState;

#[cfg(target_arch = "xtensa")]
#[inline(never)]
fn capture_live_hd_jpeg_with_diag(
    i2s_cam: &mut Esp32I2sCamera,
    ping_pong: &mut PingPongFrameBuffer<'_>,
    poll_net: &mut dyn FnMut(),
) -> Result<usize, (u32, usize, u8, u8, u8, u32)> {
    const GPIO_IN_REG: *const u32 = 0x3FF4_403C as *const u32;
    const I2S_INT_RAW_REG: *const u32 = 0x3FF4_F00C as *const u32;
    const I2S_INT_CLR_REG: *mut u32 = 0x3FF4_F018 as *mut u32;
    const I2S_IN_SUC_EOF_BIT: u32 = 1 << 9;

    feed_rtc_wdt();
    let (desc_slice, dma_slice) = unsafe {
        (
            &mut *core::ptr::addr_of_mut!(I2S_DMA_DESC),
            &mut (*core::ptr::addr_of_mut!(I2S_DMA_BUF)).0,
        )
    };
    let _ = build_circular_ring(desc_slice, dma_slice, 2048);

    let slot_cap = ping_pong.inactive_slot_and_active_frame().0.len();
    let dma_cfg = esp32_cam_ll::DmaSizeConfig {
        dma_bytes_per_item: 4,
        dma_buffer_size: 32768,
        dma_half_buffer_size: 4096,
        dma_half_buffer_cnt: 8,
        dma_node_buffer_size: 2048,
        dma_node_cnt: 16,
        lines_per_node: 1,
        nodes_per_line: 1,
        lines_per_half_buffer: 1,
        image_size: slot_cap,
    };

    let read_vsync = || -> bool {
        ((unsafe { core::ptr::read_volatile(GPIO_IN_REG) } >> 25) & 1) != 0
    };

    // Service network once prior to VSYNC edge alignment so in-flight packets are flushed
    poll_net();

    let first_desc_addr = (core::ptr::from_ref(&desc_slice[0]) as usize as u32) & 0x000F_FFFF;
    let mut eof_cnt = 0u32;
    let mut fb_len = 0usize;
    let mut last_int_raw = 0u32;

    for sync_pass in 0..2u8 {
        feed_rtc_wdt();
        let mut spin = 0u32;
        if sync_pass == 0 {
            // Fast path: if currently inside an active frame (VSYNC HIGH), wait for blanking (VSYNC LOW).
            // If already in vertical blanking (VSYNC LOW), arm DMA immediately without skipping a frame!
            if read_vsync() {
                let active_wait_start_ms = now_millis();
                while read_vsync() && (now_millis().saturating_sub(active_wait_start_ms) < 110) {
                    spin = spin.wrapping_add(1);
                    if (spin & 0x3F) == 0 {
                        feed_rtc_wdt();
                        poll_net();
                    }
                }
            }
        } else {
            // Fallback path (only if sync_pass == 0 entered at the final microsecond of blanking):
            // align cleanly to the next HIGH -> LOW blanking transition.
            let _ = build_circular_ring(desc_slice, dma_slice, 2048);
            let sync_start_ms = now_millis();
            while !read_vsync() && (now_millis().saturating_sub(sync_start_ms) < 100) {
                spin = spin.wrapping_add(1);
                if (spin & 0x3F) == 0 {
                    feed_rtc_wdt();
                    poll_net();
                }
            }
            let active_wait_start_ms = now_millis();
            while read_vsync() && (now_millis().saturating_sub(active_wait_start_ms) < 110) {
                spin = spin.wrapping_add(1);
                if (spin & 0x3F) == 0 {
                    feed_rtc_wdt();
                    poll_net();
                }
            }
        }

        if !i2s_cam.start(&dma_cfg, first_desc_addr) {
            return Err((0, 0, 0, 0, 0, 0xDEAD));
        }

        eof_cnt = 0;
        fb_len = 0;
        let mut seen_vsync_high = false;
        let mut blank_spin = 0u32;
        let mut missed_soi_on_fast_pass = false;
        let dma_start_ms = now_millis();
        let mut last_eof_ms = dma_start_ms;

        {
            let (inactive_slot, _) = ping_pong.inactive_slot_and_active_frame();

            while now_millis().saturating_sub(dma_start_ms) < 210 {
                if read_vsync() {
                    seen_vsync_high = true;
                } else if !seen_vsync_high && eof_cnt == 0 {
                    blank_spin = blank_spin.wrapping_add(1);
                    if (blank_spin & 0x3F) == 0 {
                        feed_rtc_wdt();
                        poll_net();
                    }
                }

                let int_raw = unsafe { core::ptr::read_volatile(I2S_INT_RAW_REG) };
                last_int_raw |= int_raw;

                if (int_raw & I2S_IN_SUC_EOF_BIT) != 0 {
                    unsafe {
                        core::ptr::write_volatile(I2S_INT_CLR_REG, I2S_IN_SUC_EOF_BIT);
                        core::arch::asm!("memw");
                    }
                    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::Acquire);
                    feed_rtc_wdt();
                    last_eof_ms = now_millis();

                    let half_idx = (eof_cnt as usize) & 7;
                    let half_offset = half_idx * 4096;
                    if fb_len + 1024 <= inactive_slot.len() {
                        let _ = i2s_cam.memcpy_filtered(
                            &mut inactive_slot[fb_len..fb_len + 1024],
                            &dma_slice[half_offset..half_offset + 4096],
                        );
                        let flags = Lldesc::pack_flags(2048, 0, 0, false, false, true);
                        unsafe {
                            core::ptr::write_volatile(
                                &mut desc_slice[2 * half_idx].size_length_flags,
                                flags,
                            );
                            core::ptr::write_volatile(
                                &mut desc_slice[2 * half_idx + 1].size_length_flags,
                                flags,
                            );
                        }
                        if eof_cnt == 0
                            && sync_pass == 0
                            && verify_jpeg_soi(&inactive_slot[..256]).is_none()
                        {
                            let _ = i2s_cam.stop();
                            missed_soi_on_fast_pass = true;
                            break;
                        }
                        fb_len += 1024;
                        eof_cnt += 1;
                    } else {
                        let _ = i2s_cam.stop();
                        break;
                    }
                    continue;
                }

                if eof_cnt > 0 {
                    let vsync_ended = seen_vsync_high
                        && !read_vsync()
                        && !read_vsync()
                        && !read_vsync()
                        && !read_vsync();
                    if vsync_ended || now_millis().saturating_sub(last_eof_ms) >= 5 {
                        let _ = i2s_cam.stop();
                        unsafe {
                            core::arch::asm!("memw");
                        }
                        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::Acquire);
                        let half_idx = (eof_cnt as usize) & 7;
                        let half_offset = half_idx * 4096;
                        if fb_len + 1024 <= inactive_slot.len() {
                            let _ = i2s_cam.memcpy_filtered(
                                &mut inactive_slot[fb_len..fb_len + 1024],
                                &dma_slice[half_offset..half_offset + 4096],
                            );
                            fb_len += 1024;
                        }
                        break;
                    }
                }
            }
        }

        if !missed_soi_on_fast_pass {
            break;
        }
    }

    let _ = i2s_cam.stop();
    feed_rtc_wdt();
    poll_net();

    let mut head_bytes = [0u8; 3];
    let mut commit_len = None;
    {
        let (inactive_slot, _) = ping_pong.inactive_slot_and_active_frame();
        if fb_len > 0 {
            head_bytes[0] = inactive_slot[0];
        }
        if fb_len > 1 {
            head_bytes[1] = inactive_slot[1];
        }
        if fb_len > 2 {
            head_bytes[2] = inactive_slot[2];
        }
        if fb_len >= 64 {
            if let Some(soi_pos) = verify_jpeg_soi(&inactive_slot[..fb_len.min(256)]) {
                let slice_from_soi = &inactive_slot[soi_pos..fb_len];
                if let Some(eoi_rel) = verify_jpeg_eoi(slice_from_soi, false) {
                    let exact_len = eoi_rel + 2;
                    if exact_len >= 64 {
                        if soi_pos > 0 {
                            inactive_slot.copy_within(soi_pos..soi_pos + exact_len, 0);
                        }
                        commit_len = Some(exact_len);
                    }
                }
            }
        }
    }

    if let Some(exact_len) = commit_len {
        if ping_pong.commit_inactive(exact_len).is_ok() {
            return Ok(exact_len);
        }
    }

    Err((
        eof_cnt,
        fb_len,
        head_bytes[0],
        head_bytes[1],
        head_bytes[2],
        last_int_raw,
    ))
}

#[cfg(target_arch = "xtensa")]
#[inline]
fn active_ping_pong_static_slice(pp: &PingPongFrameBuffer<'_>) -> &'static [u8] {
    let active = pp.active_frame();
    unsafe { core::slice::from_raw_parts(active.as_ptr(), active.len()) }
}

#[cfg(target_arch = "xtensa")]
#[inline(never)]
fn handle_wifi_got_ip_rollback(
    runtime: &mut FirmwareRuntime,
    flash_adapter: &mut EspFlashAdapter<'_>,
    ip: Ipv4Addr,
) {
    use klipper_cam_fw::OtaImgState;
    let s0 = runtime.server.ota_manager.sector_0.state();
    let s1 = runtime.server.ota_manager.sector_1.state();
    if s0 == OtaImgState::PendingVerify || s1 == OtaImgState::PendingVerify {
        let _ = runtime.on_wifi_got_ip_with_flash(ip, flash_adapter);
    } else {
        let _ = runtime.on_wifi_got_ip(ip);
    }
}

#[cfg(target_arch = "xtensa")]
#[inline(never)]
fn write_active_stream_frame_chunk<'b>(
    camera: &CameraDriver<'_>,
    out_buf: &'b mut [u8],
) -> Result<&'b [u8], klipper_cam_fw::HttpError> {
    use klipper_cam_fw::{
        format_mjpeg_part_header, http::MJPEG_PART_HDR_BUF_LEN, CameraError, HttpError, SliceWriter,
    };
    let guard = match camera.acquire_frame() {
        Ok(g) => g,
        Err(CameraError::InvalidState) => return Err(HttpError::CameraBusy),
        Err(_) => return Err(HttpError::CaptureFailed),
    };
    let mut part_hdr_buf = [0u8; MJPEG_PART_HDR_BUF_LEN];
    let part_hdr = format_mjpeg_part_header(guard.len(), &mut part_hdr_buf)?;

    let mut writer = SliceWriter::new(out_buf);
    writer.write_bytes(part_hdr)?;
    writer.write_bytes(guard.bytes())?;
    drop(guard);
    Ok(writer.into_bytes())
}

#[cfg(target_arch = "xtensa")]
struct PsramBuffers {
    ping_pong: PingPongFrameBuffer<'static>,
    p80_rx0: &'static mut [u8],
    p80_rx1: &'static mut [u8],
    p80_rx2: &'static mut [u8],
    p80_rx3: &'static mut [u8],
    p80_tx0: &'static mut [u8],
    p80_tx1: &'static mut [u8],
    p80_tx2: &'static mut [u8],
    p80_tx3: &'static mut [u8],
    p80_hdr_bufs: [&'static mut [u8]; PORT80_SOCKET_COUNT],
    p80_resp_bufs: [&'static mut [u8]; PORT80_SOCKET_COUNT],
    tcp81_rx0: &'static mut [u8],
    tcp81_tx0: &'static mut [u8],
    tcp81_rx1: &'static mut [u8],
    tcp81_tx1: &'static mut [u8],
    p81_resp_buf: &'static mut [u8],
    http80_ota_accum: &'static mut [u8],
    free_psram_bytes: u32,
}

#[cfg(target_arch = "xtensa")]
static mut PSRAM_BUFFERS: Option<PsramBuffers> = None;

#[cfg(target_arch = "xtensa")]
static mut FIRMWARE_RUNTIME: Option<FirmwareRuntime> = None;

#[cfg(target_arch = "xtensa")]
static mut HW_I2S_CAM: Option<Esp32I2sCamera> = None;

#[cfg(target_arch = "xtensa")]
static mut SOCKET_STORAGE: [smoltcp::iface::SocketStorage<'static>; 7] =
    [smoltcp::iface::SocketStorage::EMPTY; 7];

#[cfg(target_arch = "xtensa")]
static mut P80_STATES: [Port80WorkerState; PORT80_SOCKET_COUNT] =
    [Port80WorkerState::EMPTY; PORT80_SOCKET_COUNT];

#[cfg(target_arch = "xtensa")]
static mut P81_STATES: [Port81WorkerState; PORT81_SOCKET_COUNT] =
    [Port81WorkerState::EMPTY; PORT81_SOCKET_COUNT];

#[cfg(target_arch = "xtensa")]
static mut P81_REQ_BUFS: [[u8; 1024]; PORT81_SOCKET_COUNT] = [[0u8; 1024]; PORT81_SOCKET_COUNT];

#[cfg(target_arch = "xtensa")]
static mut P81_ERR_BUF: [u8; 512] = [0u8; 512];

#[cfg(target_arch = "xtensa")]
static mut P81_DRAIN_BUF: [u8; 128] = [0u8; 128];

#[cfg(target_arch = "xtensa")]
#[inline(never)]
fn program_ov2640_over_sccb<B: klipper_cam_fw::SccbBus>(hw_sccb: &mut B) -> (bool, Option<Ov2640>) {
    use esp_println::println;
    use klipper_cam_fw::GainCeiling;

    let _ = hw_sccb.write_reg8(Ov2640::SLV_ADDR, 0xFF, 0x01);
    match Ov2640::detect(hw_sccb, Ov2640::SLV_ADDR) {
        Ok(id) => {
            let mut ov = Ov2640::from_id(Ov2640::SLV_ADDR, id);
            // Program PixelFormat::Jpeg BEFORE set_framesize(FrameSize::Hd) so set_window
            // selects the JPEG CLKRC/PCLK dividers (`div = 12`), then apply OV2640 ISP settings
            let prog_ok = ov.reset(hw_sccb).is_ok()
                && ov.set_pixformat(hw_sccb, PixelFormat::Jpeg).is_ok()
                && ov.set_framesize(hw_sccb, FrameSize::Hd).is_ok()
                && ov.set_gainceiling(hw_sccb, GainCeiling::X2).is_ok()
                && ov.set_bpc(hw_sccb, false).is_ok()
                && ov.set_wpc(hw_sccb, true).is_ok()
                && ov.set_lenc(hw_sccb, true).is_ok()
                && ov.set_quality(hw_sccb, DEFAULT_JPEG_QUALITY as i32).is_ok()
                && ov.init_status(hw_sccb).is_ok();
            println!(
                "[KLIPPER-CAM-RS] [HW-2] I2C0 SCCB (0x30) OV2640 DETECTED: PID=0x{:02X} VER=0x{:02X} MIDH=0x{:02X} MIDL=0x{:02X} HD_PROG={}",
                id.pid, id.ver, id.midh, id.midl, prog_ok
            );
            (prog_ok, Some(ov))
        }
        Err(_) => {
            println!(
                "[KLIPPER-CAM-RS] [HW-2] I2C0 SCCB (GPIO26/GPIO27 @ 100kHz): Bus active (no external OV2640 ribbon sensor ACK at 0x30)"
            );
            (false, None)
        }
    }
}

#[cfg(target_arch = "xtensa")]
#[inline(never)]
fn init_psram_and_run_hil(psram_ptr: *mut u8, psram_len: usize, xclk_hw_ok: bool) {
    use esp_println::println;
    use klipper_cam_fw::SYNTHETIC_HD_JFIF_FRAME;

    let psram_rw_ok = if !psram_ptr.is_null() && psram_len >= 128 * 1024 + 512 * 1024 + 2_097_152 {
        unsafe {
            let p32 = psram_ptr as *mut u32;
            core::ptr::write_volatile(p32, 0x55AA_55AA);
            let r1 = core::ptr::read_volatile(p32);
            core::ptr::write_volatile(p32, 0xA55A_A55A);
            let r2 = core::ptr::read_volatile(p32);
            r1 == 0x55AA_55AA && r2 == 0xA55A_A55A
        }
    } else {
        false
    };
    if !psram_rw_ok {
        println!(
            "[KLIPPER-CAM-RS] FATAL: PSRAM init/verify failed (ptr={:p}, len={}); resetting",
            psram_ptr, psram_len
        );
        quiesce_psram_and_software_reset();
    }

    println!("============================================================");
    println!("[KLIPPER-CAM-RS] Bare-Metal Xtensa Rust Firmware v0.1.0");
    println!("[KLIPPER-CAM-RS] Target: ESP32-WROVER-DEV (240 MHz Dual-Core)");
    println!(
        "[KLIPPER-CAM-RS] [PSRAM] Mapped {} bytes @ 0x{:08X} (40 MHz Flash / 40 MHz PSRAM)",
        psram_len, psram_ptr as usize
    );
    println!("============================================================");

    let psram_all: &'static mut [u8] =
        unsafe { core::slice::from_raw_parts_mut(psram_ptr, psram_len) };
    let (jpeg_slot_a, rem) = psram_all.split_at_mut(128 * 1024);
    let (jpeg_slot_b, rem) = rem.split_at_mut(128 * 1024);

    let (p80_rx0, rem) = rem.split_at_mut(8 * 1024);
    let (p80_rx1, rem) = rem.split_at_mut(8 * 1024);
    let (p80_rx2, rem) = rem.split_at_mut(8 * 1024);
    let (p80_rx3, rem) = rem.split_at_mut(8 * 1024);
    let (p80_tx0, rem) = rem.split_at_mut(64 * 1024);
    let (p80_tx1, rem) = rem.split_at_mut(64 * 1024);
    let (p80_tx2, rem) = rem.split_at_mut(64 * 1024);
    let (p80_tx3, rem) = rem.split_at_mut(64 * 1024);
    let (p80_hdr0, rem) = rem.split_at_mut(4 * 1024);
    let (p80_hdr1, rem) = rem.split_at_mut(4 * 1024);
    let (p80_hdr2, rem) = rem.split_at_mut(4 * 1024);
    let (p80_hdr3, rem) = rem.split_at_mut(4 * 1024);
    let (p80_resp0, rem) = rem.split_at_mut(128 * 1024);
    let (p80_resp1, rem) = rem.split_at_mut(128 * 1024);
    let (p80_resp2, rem) = rem.split_at_mut(128 * 1024);
    let (p80_resp3, rem) = rem.split_at_mut(128 * 1024);

    let (tcp81_rx0, rem) = rem.split_at_mut(4 * 1024);
    let (tcp81_tx0, rem) = rem.split_at_mut(128 * 1024);
    let (tcp81_rx1, rem) = rem.split_at_mut(4 * 1024);
    let (tcp81_tx1, rem) = rem.split_at_mut(128 * 1024);
    let (p81_resp_buf, rem) = rem.split_at_mut(128 * 1024);

    let (http80_ota_accum, psram_heap_rem) = rem.split_at_mut(1800 * 1024);
    let free_psram_bytes = psram_heap_rem.len() as u32;

    let ping_pong = PingPongFrameBuffer::new(
        jpeg_slot_a,
        jpeg_slot_b,
        &SYNTHETIC_HD_JFIF_FRAME,
    )
    .expect("PingPongFrameBuffer::new");

    unsafe {
        PSRAM_BUFFERS = Some(PsramBuffers {
            ping_pong,
            p80_rx0,
            p80_rx1,
            p80_rx2,
            p80_rx3,
            p80_tx0,
            p80_tx1,
            p80_tx2,
            p80_tx3,
            p80_hdr_bufs: [p80_hdr0, p80_hdr1, p80_hdr2, p80_hdr3],
            p80_resp_bufs: [p80_resp0, p80_resp1, p80_resp2, p80_resp3],
            tcp81_rx0,
            tcp81_tx0,
            tcp81_rx1,
            tcp81_tx1,
            p81_resp_buf,
            http80_ota_accum,
            free_psram_bytes,
        });
    }

    record_boot_breadcrumb_post_wifi(3, 0, 0);
    feed_rtc_wdt();

    let check1 = hil_check_pins_and_xclk();
    feed_rtc_wdt();
    println!(
        "[KLIPPER-CAM-RS] [HIL-1] WroverKitPins + XclkController: {}",
        if check1 { "PASS" } else { "FAIL" }
    );

    let check2 = hil_check_sccb_and_ov2640_hd();
    feed_rtc_wdt();
    println!(
        "[KLIPPER-CAM-RS] [HIL-2] Ov2640 1280x720 HD (OV-105 HSIZE8=0xC8 VSIZE8=0x96 YOFFL=150): {}",
        if check2 { "PASS" } else { "FAIL" }
    );

    let check3 = hil_check_ll_dma_and_i2s0();
    feed_rtc_wdt();
    println!(
        "[KLIPPER-CAM-RS] [HIL-3] Esp32I2sCamera + Lldesc Circular DMA Ring: {}",
        if check3 { "PASS" } else { "FAIL" }
    );

    let check4 = hil_check_jpeg_pool_and_http_ota();
    feed_rtc_wdt();
    println!(
        "[KLIPPER-CAM-RS] [HIL-4] DCT JPEG + FramePool + Port 80/81 HTTP/MJPEG + OTA: {}",
        if check4 { "PASS" } else { "FAIL" }
    );

    let all_ok = xclk_hw_ok && check1 && check2 && check3 && check4;
    println!(
        "[HIL SUMMARY] 7-Crate Bare-Metal Xtensa Self-Test: {}",
        if all_ok { "ALL PASS (100%)" } else { "FAIL" }
    );

    let mut hw_i2s_cam = Esp32I2sCamera::new();
    let ll_pins = WroverKitPins::PINS.to_ll_pin_config(false);
    let _ = hw_i2s_cam.set_pin(&ll_pins);
    let _ = hw_i2s_cam.config();
    let _ = hw_i2s_cam.set_sample_mode(PixelFormat::Jpeg, DEFAULT_XCLK_FREQ_HZ, 0x26);
    unsafe {
        HW_I2S_CAM = Some(hw_i2s_cam);
    }
}

#[cfg(target_arch = "xtensa")]
#[inline(never)]
fn capture_boot_dma_and_init_runtime(
    ov2640_hw_ok: bool,
    bc_prev: BreadcrumbPrevSummary,
    ota_manager: klipper_cam_fw::OtaDataManager,
) -> bool {
    use esp_hal::delay::Delay;
    use esp_println::println;

    let delay = Delay::new();
    let psram_bufs = unsafe { PSRAM_BUFFERS.as_mut().expect("PSRAM_BUFFERS") };
    let hw_i2s_cam = unsafe { HW_I2S_CAM.as_mut().expect("HW_I2S_CAM") };

    let ov2640_dma_ok = ov2640_hw_ok;
    let mut boot_dma_ok = false;

    if ov2640_hw_ok {
        for attempt in 1..=3u8 {
            feed_rtc_wdt();
            match capture_live_hd_jpeg_with_diag(
                hw_i2s_cam,
                &mut psram_bufs.ping_pong,
                &mut || {},
            ) {
                Ok(captured_len) => {
                    boot_dma_ok = true;
                    println!(
                        "[KLIPPER-CAM-RS] [HW-3] Live OV2640 I2S0 DMA 1280x720 JPEG Capture: PASS ({} bytes, attempt {}, inv=false)",
                        captured_len, attempt
                    );
                    break;
                }
                Err((eof_cnt, fb_len, b0, b1, b2, int_raw)) => {
                    println!(
                        "[KLIPPER-CAM-RS] [HW-3] Attempt {} (inv=false): eof_cnt={} fb_len={} head=[0x{:02X},0x{:02X},0x{:02X}] int_raw=0x{:08X}",
                        attempt, eof_cnt, fb_len, b0, b1, b2, int_raw
                    );
                    delay.delay_millis(25);
                }
            }
        }
    }
    if !boot_dma_ok {
        println!(
            "[KLIPPER-CAM-RS] [HW-3] Live OV2640 I2S0 DMA Capture fallback: using synthetic 1280x720 JFIF ({} bytes)",
            psram_bufs.ping_pong.active_len()
        );
    }
    println!("============================================================");

    feed_rtc_wdt();
    record_boot_breadcrumb_post_wifi(4, u8::from(boot_dma_ok), 0);
    println!(
        "[KLIPPER-CAM-RS] [BREADCRUMB] prev_magic=0x{:08X} prev_boots={} prev_reset=0x{:08X} prev_stage={} prev_ch={} prev_aid={} cur_boots={}",
        bc_prev.prev_magic,
        bc_prev.prev_boots,
        bc_prev.prev_reset,
        bc_prev.prev_stage,
        bc_prev.prev_ch,
        bc_prev.prev_aid,
        bc_prev.new_boots
    );

    let initial_slice = active_ping_pong_static_slice(&psram_bufs.ping_pong);
    let mut runtime = FirmwareRuntime::new(
        KlipperCamConfig::WROVER_KIT_HD,
        CameraDriver::new(initial_slice, FrameSize::Hd),
    );
    runtime.server.ota_manager = ota_manager;
    unsafe {
        FIRMWARE_RUNTIME = Some(runtime);
    }
    ov2640_dma_ok
}

#[cfg(target_arch = "xtensa")]
#[inline(never)]
fn build_wifi_station_configs() -> (
    esp_radio::wifi::sta::StationConfig,
    esp_radio::wifi::ControllerConfig,
    &'static str,
) {
    use esp_radio::wifi::{
        sta::{ScanMethod, StationConfig},
        AuthenticationMethodConfig, Config as WifiConfig, ControllerConfig,
    };
    use klipper_cam_fw::{active_wifi_pass, active_wifi_ssid};

    let ssid = active_wifi_ssid();
    let pass = active_wifi_pass();
    let auth = if pass.is_empty() {
        AuthenticationMethodConfig::Open
    } else {
        match pass.try_into() {
            Ok(pw) => AuthenticationMethodConfig::WpaWpa2Personal(pw),
            Err(_) => AuthenticationMethodConfig::Open,
        }
    };
    let mut sta_cfg = StationConfig::default()
        .with_authentication(auth)
        .with_scan_method(ScanMethod::AllChannels)
        .with_failure_retry_cnt(2);
    if let Ok(s) = ssid.try_into() {
        sta_cfg = sta_cfg.with_ssid(s);
    };

    let ctrl_cfg = ControllerConfig::default()
        .with_rx_queue_size(20)
        .with_tx_queue_size(16)
        .with_initial_config(WifiConfig::Station(sta_cfg.clone()));
    (sta_cfg, ctrl_cfg, ssid)
}

#[cfg(target_arch = "xtensa")]
#[inline(never)]
fn run_post_wifi_server_loop<B: klipper_cam_fw::SccbBus>(
    mut wifi_controller: esp_radio::wifi::WifiController<'static>,
    sta_cfg: esp_radio::wifi::sta::StationConfig,
    ssid: &'static str,
    mut flash_adapter: EspFlashAdapter<'static>,
    mut kept_sccb: Option<B>,
    mut live_ov2640: Option<Ov2640>,
    ov2640_dma_ok: bool,
) -> ! {
    use core::sync::atomic::Ordering;
    use esp_hal::delay::Delay;
    use esp_println::println;
    use esp_radio::wifi::{Interface as WifiInterface, PowerSaveMode};
    use klipper_cam_fw::{
        parse_http_request, HttpMethod, OtaDataManager, StreamSessionGuard, OTA_PARTITION_MAX_SIZE,
    };
    use smoltcp::{
        iface::{Config as IfaceConfig, Interface as NetInterface, SocketSet},
        socket::{
            dhcpv4,
            tcp::{Socket as TcpSocket, SocketBuffer as TcpSocketBuffer, State as TcpState},
        },
        wire::{EthernetAddress, HardwareAddress, IpAddress, IpCidr, Ipv4Address},
    };

    let delay = Delay::new();
    let psram_bufs = unsafe { PSRAM_BUFFERS.as_mut().expect("PSRAM_BUFFERS") };
    let runtime = unsafe { FIRMWARE_RUNTIME.as_mut().expect("FIRMWARE_RUNTIME") };
    let hw_i2s_cam = unsafe { HW_I2S_CAM.as_mut().expect("HW_I2S_CAM") };
    let ping_pong = &mut psram_bufs.ping_pong;
    let p80_hdr_bufs = &mut psram_bufs.p80_hdr_bufs;
    let p80_resp_bufs = &mut psram_bufs.p80_resp_bufs;
    let p81_resp_buf: &mut [u8] = psram_bufs.p81_resp_buf;
    let http80_ota_accum: &mut [u8] = psram_bufs.http80_ota_accum;
    let free_psram_bytes = psram_bufs.free_psram_bytes;

    record_boot_breadcrumb_post_wifi(5, 0, 0);
    let _ = wifi_controller.set_max_tx_power(WIFI_MAX_TX_POWER_QDBM);
    let _ = wifi_controller.set_power_saving(PowerSaveMode::None);

    let wifi_iface = WifiInterface::station();
    let mac = wifi_iface.mac_address();
    println!(
        "[KLIPPER-CAM-RS] [WIFI] Connecting to SSID '{}' (MAC {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x})...",
        ssid, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]
    );
    runtime.on_wifi_sta_start();

    let mut boot_connected = connect_strongest_ap(&mut wifi_controller, &sta_cfg, ssid, 15_000);
    if boot_connected.is_none() {
        delay.delay_millis(250);
        boot_connected = connect_strongest_ap(&mut wifi_controller, &sta_cfg, ssid, 15_000);
    }
    match boot_connected {
        Some(info) => {
            record_boot_breadcrumb_post_wifi(6, info.channel, info.aid);
            println!(
                "[KLIPPER-CAM-RS] [WIFI] Connected to AP (channel {}, aid {})",
                info.channel, info.aid
            );
            runtime.on_wifi_sta_connected();
        }
        None => {
            record_boot_breadcrumb_post_wifi(7, 0, 0);
            println!(
                "[KLIPPER-CAM-RS] [WIFI] Initial connect warning/timeout; will retry in loop"
            );
            runtime.on_wifi_sta_disconnected();
        }
    }

    // 7. Configure smoltcp Ethernet Interface + DHCPv4 + Static IP (172.22.1.52/24, GW 172.22.1.1)
    //    + 4-socket Port 80 pool + 2-socket Port 81 pool (7 sockets total)
    let mut net_device = SmoltcpWifiDevice(wifi_iface);
    let mut iface_cfg = IfaceConfig::new(HardwareAddress::Ethernet(EthernetAddress(mac)));
    // WDEV_RND_REG (`wdev_reg.h:19`: 0x6003_5144 APB / 0x3FF7_5144 DPORT peripheral bus alias)
    const RNG_DATA_REG: *const u32 = 0x3FF7_5144 as *const u32;
    let hw_rng_lo = unsafe { core::ptr::read_volatile(RNG_DATA_REG) } as u64;
    let hw_rng_hi = unsafe { core::ptr::read_volatile(RNG_DATA_REG) } as u64;
    let boot_us = esp_hal::time::Instant::now()
        .duration_since_epoch()
        .as_micros();
    iface_cfg.random_seed = ((hw_rng_hi << 32) | hw_rng_lo) ^ boot_us ^ 0x9E37_79B9_7F4A_7C15;
    let mut iface = NetInterface::new(iface_cfg, &mut net_device, smol_now());

    let socket_storage = unsafe { &mut *core::ptr::addr_of_mut!(SOCKET_STORAGE) };
    let mut sockets = SocketSet::new(&mut socket_storage[..]);

    let p80_rx0: &'static mut [u8] =
        unsafe { core::slice::from_raw_parts_mut(psram_bufs.p80_rx0.as_mut_ptr(), psram_bufs.p80_rx0.len()) };
    let p80_tx0: &'static mut [u8] =
        unsafe { core::slice::from_raw_parts_mut(psram_bufs.p80_tx0.as_mut_ptr(), psram_bufs.p80_tx0.len()) };
    let p80_rx1: &'static mut [u8] =
        unsafe { core::slice::from_raw_parts_mut(psram_bufs.p80_rx1.as_mut_ptr(), psram_bufs.p80_rx1.len()) };
    let p80_tx1: &'static mut [u8] =
        unsafe { core::slice::from_raw_parts_mut(psram_bufs.p80_tx1.as_mut_ptr(), psram_bufs.p80_tx1.len()) };
    let p80_rx2: &'static mut [u8] =
        unsafe { core::slice::from_raw_parts_mut(psram_bufs.p80_rx2.as_mut_ptr(), psram_bufs.p80_rx2.len()) };
    let p80_tx2: &'static mut [u8] =
        unsafe { core::slice::from_raw_parts_mut(psram_bufs.p80_tx2.as_mut_ptr(), psram_bufs.p80_tx2.len()) };
    let p80_rx3: &'static mut [u8] =
        unsafe { core::slice::from_raw_parts_mut(psram_bufs.p80_rx3.as_mut_ptr(), psram_bufs.p80_rx3.len()) };
    let p80_tx3: &'static mut [u8] =
        unsafe { core::slice::from_raw_parts_mut(psram_bufs.p80_tx3.as_mut_ptr(), psram_bufs.p80_tx3.len()) };
    let tcp81_rx0: &'static mut [u8] =
        unsafe { core::slice::from_raw_parts_mut(psram_bufs.tcp81_rx0.as_mut_ptr(), psram_bufs.tcp81_rx0.len()) };
    let tcp81_tx0: &'static mut [u8] =
        unsafe { core::slice::from_raw_parts_mut(psram_bufs.tcp81_tx0.as_mut_ptr(), psram_bufs.tcp81_tx0.len()) };
    let tcp81_rx1: &'static mut [u8] =
        unsafe { core::slice::from_raw_parts_mut(psram_bufs.tcp81_rx1.as_mut_ptr(), psram_bufs.tcp81_rx1.len()) };
    let tcp81_tx1: &'static mut [u8] =
        unsafe { core::slice::from_raw_parts_mut(psram_bufs.tcp81_tx1.as_mut_ptr(), psram_bufs.tcp81_tx1.len()) };

    let dhcp_handle = sockets.add(dhcpv4::Socket::new());
    let tcp80_handles: [smoltcp::iface::SocketHandle; PORT80_SOCKET_COUNT] = [
        sockets.add(TcpSocket::new(
            TcpSocketBuffer::new(p80_rx0),
            TcpSocketBuffer::new(p80_tx0),
        )),
        sockets.add(TcpSocket::new(
            TcpSocketBuffer::new(p80_rx1),
            TcpSocketBuffer::new(p80_tx1),
        )),
        sockets.add(TcpSocket::new(
            TcpSocketBuffer::new(p80_rx2),
            TcpSocketBuffer::new(p80_tx2),
        )),
        sockets.add(TcpSocket::new(
            TcpSocketBuffer::new(p80_rx3),
            TcpSocketBuffer::new(p80_tx3),
        )),
    ];
    for &h80 in &tcp80_handles {
        let s80 = sockets.get_mut::<TcpSocket>(h80);
        s80.set_nagle_enabled(false);
        s80.set_timeout(None);
        s80.set_keep_alive(Some(smoltcp::time::Duration::from_millis(2_000)));
        let _ = s80.listen(80);
    }
    let p80_states = unsafe { &mut *core::ptr::addr_of_mut!(P80_STATES) };
    let mut ota_accum_owner: Option<usize> = None;

    let tcp81_handles: [smoltcp::iface::SocketHandle; PORT81_SOCKET_COUNT] = [
        sockets.add(TcpSocket::new(
            TcpSocketBuffer::new(tcp81_rx0),
            TcpSocketBuffer::new(tcp81_tx0),
        )),
        sockets.add(TcpSocket::new(
            TcpSocketBuffer::new(tcp81_rx1),
            TcpSocketBuffer::new(tcp81_tx1),
        )),
    ];
    for &h81 in &tcp81_handles {
        let s81 = sockets.get_mut::<TcpSocket>(h81);
        s81.set_nagle_enabled(false);
        s81.set_timeout(None);
        s81.set_keep_alive(Some(smoltcp::time::Duration::from_millis(2_000)));
        let _ = s81.listen(81);
    }
    let p81_req_bufs = unsafe { &mut *core::ptr::addr_of_mut!(P81_REQ_BUFS) };
    let p81_states = unsafe { &mut *core::ptr::addr_of_mut!(P81_STATES) };
    let mut p81_stream_owner: Option<usize> = None;
    let mut p81_last_tx_progress_ms: i64 = 0;
    let mut p81_last_send_queue: usize = 0;
    let mut p81_tx_off: usize = 0;
    let mut p81_tx_len: usize = 0;
    let mut p81_next_frame_captured: bool = false;

    let mut ip_configured = false;
    let mut connect_ms = smol_now().total_millis();
    let mut last_wifi_connected_ms = connect_ms;
    let mut last_reconnect_attempt_ms = connect_ms;
    let mut last_heartbeat_ms = connect_ms;
    let mut last_arp_announce_ms = connect_ms;
    let mut last_capture_ms = connect_ms;
    let mut post_snapshot_refresh_at_ms: Option<i64> = None;
    let mut last_applied_ctrl = klipper_cam_fw::CameraControlState::new_default();

    loop {
        feed_rtc_wdt();
        let now = smol_now();
        let now_ms = now.total_millis();

        if wifi_controller.is_connected() {
            last_wifi_connected_ms = now_ms;
        } else {
            if now_ms - last_wifi_connected_ms >= 90_000
                && ota_accum_owner.is_none()
                && !runtime.server.is_reboot_scheduled()
            {
                println!(
                    "[KLIPPER-CAM-RS] [WATCHDOG] Wi-Fi disconnected > 90s; deasserting PSRAM CS# and rebooting..."
                );
                let _ = hw_i2s_cam.stop();
                quiesce_psram_and_software_reset();
            }
            if now_ms - last_reconnect_attempt_ms >= 5000 {
                runtime.on_wifi_sta_disconnected();
                println!(
                    "[KLIPPER-CAM-RS] [WIFI] Attempting reconnect to SSID '{}' (disconnected {}ms)...",
                    ssid,
                    now_ms - last_wifi_connected_ms
                );
                let reconnect_res =
                    connect_strongest_ap(&mut wifi_controller, &sta_cfg, ssid, 15_000);
                last_reconnect_attempt_ms = smol_now().total_millis();
                if let Some(info) = reconnect_res {
                    println!(
                        "[KLIPPER-CAM-RS] [WIFI] Reconnected to AP (channel {}, aid {})",
                        info.channel, info.aid
                    );
                    connect_ms = smol_now().total_millis();
                    last_wifi_connected_ms = connect_ms;
                    runtime.on_wifi_sta_connected();
                    handle_wifi_got_ip_rollback(
                        runtime,
                        &mut flash_adapter,
                        Ipv4Addr::new(172, 22, 1, 52),
                    );
                    if ip_configured {
                        send_arp_announce(
                            &mut net_device,
                            mac,
                            [172, 22, 1, 52],
                            [172, 22, 1, 1],
                        );
                        last_arp_announce_ms = connect_ms;
                    }
                }
            }
        }

        let _ = iface.poll(now, &mut net_device, &mut sockets);

        // Check DHCPv4 events or configure static IP (172.22.1.52/24, GW 172.22.1.1) unconditionally
        {
            let dhcp_sock = sockets.get_mut::<dhcpv4::Socket>(dhcp_handle);
            match dhcp_sock.poll() {
                Some(dhcpv4::Event::Configured(cfg)) => {
                    let addr = cfg.address.address();
                    let octets = addr.octets();
                    let prefix_len = cfg.address.prefix_len();
                    iface.update_ip_addrs(|addrs| {
                        if !addrs
                            .iter()
                            .any(|c| c.address() == IpAddress::v4(172, 22, 1, 52))
                        {
                            let _ =
                                addrs.push(IpCidr::new(IpAddress::v4(172, 22, 1, 52), prefix_len));
                        }
                        if addr != Ipv4Address::new(172, 22, 1, 52)
                            && !addrs.iter().any(|c| c.address() == IpAddress::Ipv4(addr))
                        {
                            let _ = addrs.push(IpCidr::Ipv4(cfg.address));
                        }
                    });
                    let gw = cfg.router.unwrap_or(Ipv4Address::new(172, 22, 1, 1));
                    let _ = iface.routes_mut().add_default_ipv4_route(gw);
                    let fw_ip = Ipv4Addr::new(172, 22, 1, 52);
                    runtime.on_wifi_sta_connected();
                    handle_wifi_got_ip_rollback(runtime, &mut flash_adapter, fw_ip);
                    ip_configured = true;
                    send_arp_announce(&mut net_device, mac, [172, 22, 1, 52], [172, 22, 1, 1]);
                    last_arp_announce_ms = now_ms;
                    println!(
                        "[KLIPPER-CAM-RS] [NET] IP Configured: 172.22.1.52/{} + DHCP {}.{}.{}.{}/{} (Port 80 x4 & Port 81 x2 Ready)",
                        prefix_len, octets[0], octets[1], octets[2], octets[3], prefix_len
                    );
                }
                Some(dhcpv4::Event::Deconfigured) => {
                    iface.update_ip_addrs(|addrs| {
                        if !addrs
                            .iter()
                            .any(|c| c.address() == IpAddress::v4(172, 22, 1, 52))
                        {
                            let _ = addrs.push(IpCidr::new(IpAddress::v4(172, 22, 1, 52), 24));
                        }
                    });
                    let _ = iface
                        .routes_mut()
                        .add_default_ipv4_route(Ipv4Address::new(172, 22, 1, 1));
                }
                None => {}
            }
        }

        if !ip_configured {
            iface.update_ip_addrs(|addrs| {
                if !addrs
                    .iter()
                    .any(|c| c.address() == IpAddress::v4(172, 22, 1, 52))
                {
                    let _ = addrs.push(IpCidr::new(IpAddress::v4(172, 22, 1, 52), 24));
                }
            });
            let _ = iface
                .routes_mut()
                .add_default_ipv4_route(Ipv4Address::new(172, 22, 1, 1));
            let fw_ip = Ipv4Addr::new(172, 22, 1, 52);
            if wifi_controller.is_connected() {
                runtime.on_wifi_sta_connected();
            }
            handle_wifi_got_ip_rollback(runtime, &mut flash_adapter, fw_ip);
            ip_configured = true;
            if wifi_controller.is_connected() {
                send_arp_announce(&mut net_device, mac, [172, 22, 1, 52], [172, 22, 1, 1]);
                last_arp_announce_ms = now_ms;
            }
            println!(
                "[KLIPPER-CAM-RS] [NET] Static IP Configured: 172.22.1.52/24 GW 172.22.1.1 (Port 80 x4 & Port 81 x2 Ready)"
            );
        }

        let arp_interval_ms = if now_ms - connect_ms < 3000 {
            500
        } else {
            5000
        };
        if wifi_controller.is_connected() && now_ms - last_arp_announce_ms >= arp_interval_ms {
            last_arp_announce_ms = now_ms;
            send_arp_announce(&mut net_device, mac, [172, 22, 1, 52], [172, 22, 1, 1]);
        }

        // ============================================================
        // On-Demand Post-Snapshot I2S0 Capture (Ping-Pong Double Buffer)
        // ============================================================
        let p80_actively_sending = p80_states.iter().any(|s| {
            s.tx_len > 0 && !s.closing && s.close_deadline_ms.saturating_sub(now_ms) > 3_500
        });
        let post_snap_due = post_snapshot_refresh_at_ms
            .map(|deadline| now_ms >= deadline)
            .unwrap_or(false)
            && !p80_actively_sending
            && (now_ms - last_capture_ms >= 15);
        let idle_refresh_due = p81_stream_owner.is_none()
            && !p80_actively_sending
            && (now_ms - last_capture_ms >= 30_000);
        if ov2640_dma_ok
            && ota_accum_owner.is_none()
            && !runtime.server.is_reboot_scheduled()
            && p81_stream_owner.is_none()
            && (post_snap_due || idle_refresh_due)
        {
            post_snapshot_refresh_at_ms = None;
            last_capture_ms = now_ms;
            if capture_live_hd_jpeg_with_diag(
                hw_i2s_cam,
                ping_pong,
                &mut || {
                    poll_net_burst(&mut iface, &mut net_device, &mut sockets);
                },
            )
            .is_ok()
            {
                let fresh_slice = active_ping_pong_static_slice(ping_pong);
                runtime.server.camera.set_frame(
                    fresh_slice,
                    runtime.server.control_state.framesize,
                );
                last_capture_ms = smol_now().total_millis();
            }
        }

        // ============================================================
        // Port 80 4-Socket Listener Pool (Control / Snapshot / Status / OTA)
        // ============================================================
        for idx in 0..PORT80_SOCKET_COUNT {
            let sock80 = sockets.get_mut::<TcpSocket>(tcp80_handles[idx]);
            let st = &mut p80_states[idx];

            if !sock80.is_open() {
                if ota_accum_owner == Some(idx) {
                    ota_accum_owner = None;
                }
                *st = Port80WorkerState::default();
                sock80.set_timeout(None);
                sock80.set_keep_alive(Some(smoltcp::time::Duration::from_millis(2_000)));
                let _ = sock80.listen(80);
                continue;
            }

            if sock80.is_active() {
                if st.conn_active_ms == 0 {
                    st.conn_active_ms = now_ms;
                }
            } else {
                st.conn_active_ms = 0;
            }

            if st.closing {
                let finished_cleanly = matches!(
                    sock80.state(),
                    TcpState::TimeWait | TcpState::Closed
                );
                if !sock80.is_active() || finished_cleanly || now_ms >= st.close_deadline_ms {
                    sock80.abort();
                    if ota_accum_owner == Some(idx) {
                        ota_accum_owner = None;
                    }
                    *st = Port80WorkerState::default();
                }
            } else if st.tx_len > 0 {
                let cur_q = sock80.send_queue();
                if cur_q < st.last_send_queue {
                    st.last_send_queue = cur_q;
                    st.close_deadline_ms = now_ms + 4_000;
                }
                let peer_disconnected = !sock80.is_active()
                    || !sock80.may_send()
                    || matches!(
                        sock80.state(),
                        TcpState::Closing
                            | TcpState::LastAck
                            | TcpState::TimeWait
                            | TcpState::Closed
                    )
                    || (st.close_deadline_ms > 0 && now_ms >= st.close_deadline_ms);
                if peer_disconnected {
                    sock80.abort();
                    if ota_accum_owner == Some(idx) {
                        ota_accum_owner = None;
                    }
                    *st = Port80WorkerState::default();
                } else if st.tx_off < st.tx_len {
                    if sock80.can_send() {
                        let prev_off = st.tx_off;
                        enqueue_tcp_sliding_window(
                            sock80,
                            p80_resp_bufs[idx],
                            &mut st.tx_off,
                            st.tx_len,
                        );
                        if st.tx_off != prev_off {
                            st.close_deadline_ms = now_ms + 4_000;
                            st.last_send_queue = sock80.send_queue();
                        }
                    }
                } else if sock80.send_queue() == 0 {
                    if st.is_snapshot {
                        st.is_snapshot = false;
                        post_snapshot_refresh_at_ms = Some(now_ms + 2);
                    }
                    sock80.close();
                    st.closing = true;
                    st.close_deadline_ms = now_ms + 2_000;
                }
            } else {
                if sock80.can_recv() {
                    if st.req_started_ms == 0 {
                        st.req_started_ms = now_ms as u64;
                    }
                    if !st.using_ota_accum {
                        let avail = p80_hdr_bufs[idx].len().saturating_sub(st.req_len);
                        if avail > 0 {
                            if let Ok(n) = sock80
                                .recv_slice(&mut p80_hdr_bufs[idx][st.req_len..st.req_len + avail])
                            {
                                st.req_len += n;
                            }
                        }

                        // SEC-102 & LOGIC-101: Buffer capacity check & auth-first promotion into shared http80_ota_accum
                        if let Ok(parsed) = parse_http_request(&p80_hdr_bufs[idx][..st.req_len]) {
                            let needed_body = match parsed.content_length {
                                Some(cl) if cl > 0 => cl as usize,
                                _ => 0,
                            };
                            let hdr_bytes = st.req_len.saturating_sub(parsed.body.len());
                            let total_needed = hdr_bytes.saturating_add(needed_body);
                            if total_needed > p80_hdr_bufs[idx].len() {
                                if !(parsed.method == HttpMethod::Post && parsed.path == "/update")
                                {
                                    if let Ok(err) = write_http_error(
                                        p80_resp_bufs[idx],
                                        413,
                                        "Content Too Large",
                                        "Payload exceeds maximum buffer capacity\n",
                                        false,
                                    ) {
                                        st.req_len = 0;
                                        st.tx_len = err.len();
                                        st.tx_off = 0;
                                        st.close_deadline_ms = now_ms + 4_000;
                                        enqueue_tcp_sliding_window(
                                            sock80,
                                            p80_resp_bufs[idx],
                                            &mut st.tx_off,
                                            st.tx_len,
                                        );
                                        st.last_send_queue = sock80.send_queue();
                                    }
                                } else if parsed.ota_token != Some(runtime.server.ota_token()) {
                                    if let Ok(err) = write_http_error(
                                        p80_resp_bufs[idx],
                                        401,
                                        "Unauthorized",
                                        "Unauthorized: Invalid or missing X-OTA-Token\n",
                                        false,
                                    ) {
                                        st.req_len = 0;
                                        st.tx_len = err.len();
                                        st.tx_off = 0;
                                        st.close_deadline_ms = now_ms + 4_000;
                                        enqueue_tcp_sliding_window(
                                            sock80,
                                            p80_resp_bufs[idx],
                                            &mut st.tx_off,
                                            st.tx_len,
                                        );
                                        st.last_send_queue = sock80.send_queue();
                                    }
                                } else if total_needed > http80_ota_accum.len() {
                                    if let Ok(err) = write_http_error(
                                        p80_resp_bufs[idx],
                                        413,
                                        "Content Too Large",
                                        "OTA payload exceeds buffer capacity\n",
                                        false,
                                    ) {
                                        st.req_len = 0;
                                        st.tx_len = err.len();
                                        st.tx_off = 0;
                                        st.close_deadline_ms = now_ms + 4_000;
                                        enqueue_tcp_sliding_window(
                                            sock80,
                                            p80_resp_bufs[idx],
                                            &mut st.tx_off,
                                            st.tx_len,
                                        );
                                        st.last_send_queue = sock80.send_queue();
                                    }
                                } else if ota_accum_owner.is_none() {
                                    http80_ota_accum[..st.req_len]
                                        .copy_from_slice(&p80_hdr_bufs[idx][..st.req_len]);
                                    st.using_ota_accum = true;
                                    st.req_started_ms = now_ms as u64;
                                    ota_accum_owner = Some(idx);
                                    let _ = hw_i2s_cam.stop();
                                } else if ota_accum_owner != Some(idx) {
                                    if let Ok(err) = write_http_error(
                                        p80_resp_bufs[idx],
                                        503,
                                        "Service Unavailable",
                                        "OTA update in progress on another connection\n",
                                        false,
                                    ) {
                                        st.req_len = 0;
                                        st.tx_len = err.len();
                                        st.tx_off = 0;
                                        st.close_deadline_ms = now_ms + 4_000;
                                        enqueue_tcp_sliding_window(
                                            sock80,
                                            p80_resp_bufs[idx],
                                            &mut st.tx_off,
                                            st.tx_len,
                                        );
                                        st.last_send_queue = sock80.send_queue();
                                    }
                                }
                            }
                        }
                    }

                    if st.using_ota_accum && sock80.can_recv() {
                        let avail = http80_ota_accum.len().saturating_sub(st.req_len);
                        if avail > 0 {
                            if let Ok(n) = sock80
                                .recv_slice(&mut http80_ota_accum[st.req_len..st.req_len + avail])
                            {
                                if n > 0 {
                                    st.req_len += n;
                                    st.req_started_ms = now_ms as u64;
                                }
                            }
                        }
                    }

                    if st.tx_len == 0 {
                        let mut req_complete = false;
                        let mut is_ota_post = false;
                        let mut is_snapshot_req = false;
                        let peer_closed_rx = matches!(sock80.state(), TcpState::CloseWait);
                        {
                            let req_slice: &[u8] = if st.using_ota_accum {
                                &http80_ota_accum[..st.req_len]
                            } else {
                                &p80_hdr_bufs[idx][..st.req_len]
                            };
                            if let Ok(parsed) = parse_http_request(req_slice) {
                                let needed_body = match parsed.content_length {
                                    Some(cl) if cl > 0 => {
                                        (cl as usize).min(OTA_PARTITION_MAX_SIZE + 4096)
                                    }
                                    _ => 0,
                                };
                                if parsed.body.len() >= needed_body || peer_closed_rx {
                                    req_complete = true;
                                    if parsed.method == HttpMethod::Post
                                        && parsed.path == "/update"
                                    {
                                        is_ota_post = true;
                                    }
                                    if parsed.method == HttpMethod::Get
                                        && (parsed.path == "/snapshot"
                                            || (parsed.path == "/"
                                                && parsed
                                                    .query
                                                    .map(|q| q.contains("action=snapshot"))
                                                    .unwrap_or(false)))
                                    {
                                        is_snapshot_req = true;
                                    }
                                }
                            } else if (!st.using_ota_accum
                                && st.req_len >= p80_hdr_bufs[idx].len())
                                || peer_closed_rx
                            {
                                req_complete = true;
                            }
                        }

                        if req_complete {
                            if is_ota_post {
                                let _ = hw_i2s_cam.stop();
                                let booted_slot = runtime.server.ota_manager.active_slot();
                                if let Ok(loaded) = OtaDataManager::load_from_flash(&mut flash_adapter) {
                                    if loaded.active_slot() == booted_slot {
                                        runtime.server.ota_manager = loaded;
                                    }
                                }
                            }
                            if is_snapshot_req {
                                st.is_snapshot = true;
                            }

                            let free_dram = esp_alloc::HEAP
                                .free_caps(esp_alloc::MemoryCapability::Internal.into())
                                as u32;
                            let rssi = wifi_controller.rssi().unwrap_or(-50);
                            runtime
                                .server
                                .set_telemetry(now_ms, free_dram, free_psram_bytes, rssi);

                            let req_slice: &[u8] = if st.using_ota_accum {
                                &http80_ota_accum[..st.req_len]
                            } else {
                                &p80_hdr_bufs[idx][..st.req_len]
                            };

                            let handle_res = runtime
                                .server
                                .handle_raw_request_with_flash(
                                    ServerPort::Control,
                                    req_slice,
                                    p80_resp_bufs[idx],
                                    &mut flash_adapter,
                                )
                                .map(|s| s.len());

                            if st.using_ota_accum {
                                st.using_ota_accum = false;
                                if ota_accum_owner == Some(idx) {
                                    ota_accum_owner = None;
                                }
                            }

                            match handle_res {
                                Ok(resp_len) => {
                                    st.req_len = 0;
                                    st.tx_len = resp_len;
                                    st.tx_off = 0;
                                    st.close_deadline_ms = now_ms + 4_000;
                                    enqueue_tcp_sliding_window(
                                        sock80,
                                        p80_resp_bufs[idx],
                                        &mut st.tx_off,
                                        st.tx_len,
                                    );
                                    st.last_send_queue = sock80.send_queue();
                                }
                                Err(_) => {
                                    st.req_len = 0;
                                    sock80.close();
                                    st.closing = true;
                                    st.close_deadline_ms = now_ms + 2_000;
                                }
                            }
                        }
                    }
                }

                if st.tx_len == 0 && !st.closing {
                    let idle_or_half_closed = (!st.using_ota_accum
                        && st.conn_active_ms > 0
                        && now_ms - st.conn_active_ms > 3_000)
                        || (st.using_ota_accum
                            && st.req_len > 0
                            && (now_ms as u64).saturating_sub(st.req_started_ms) > 30_000)
                        || (matches!(sock80.state(), TcpState::CloseWait) && !sock80.can_recv());
                    if idle_or_half_closed {
                        sock80.abort();
                        if ota_accum_owner == Some(idx) {
                            ota_accum_owner = None;
                        }
                        *st = Port80WorkerState::default();
                    }
                }
            }
        }

        // Apply any pending live camera control updates from GET /control?var=...&val=...
        // only after all active Port 80 HTTP responses have finished flushing.
        let p80_tx_active = p80_states.iter().any(|s| s.tx_len > 0 && !s.closing);
        if p80_tx_active {
            poll_net_burst(&mut iface, &mut net_device, &mut sockets);
        } else if let Some(ctrl) = runtime.server.take_pending_control_update() {
            if let (Some(ref mut sccb), Some(ref mut ov)) = (&mut kept_sccb, &mut live_ov2640) {
                let fs_changed = ctrl.framesize != last_applied_ctrl.framesize;
                if fs_changed {
                    let _ = ov.set_framesize(sccb, ctrl.framesize);
                }
                if fs_changed || ctrl.quality != last_applied_ctrl.quality {
                    let _ = ov.set_quality(sccb, ctrl.quality);
                }
                if fs_changed || ctrl.brightness != last_applied_ctrl.brightness {
                    let _ = ov.set_brightness(sccb, ctrl.brightness);
                }
                if fs_changed || ctrl.contrast != last_applied_ctrl.contrast {
                    let _ = ov.set_contrast(sccb, ctrl.contrast);
                }
                if fs_changed || ctrl.saturation != last_applied_ctrl.saturation {
                    let _ = ov.set_saturation(sccb, ctrl.saturation);
                }
                if fs_changed || ctrl.special_effect != last_applied_ctrl.special_effect {
                    let _ = ov.set_special_effect(sccb, ctrl.special_effect);
                }
                if fs_changed || ctrl.hmirror != last_applied_ctrl.hmirror {
                    let _ = ov.set_hmirror(sccb, ctrl.hmirror);
                }
                if fs_changed || ctrl.vflip != last_applied_ctrl.vflip {
                    let _ = ov.set_vflip(sccb, ctrl.vflip);
                }
                if fs_changed || ctrl.awb != last_applied_ctrl.awb {
                    let _ = ov.set_whitebal(sccb, ctrl.awb);
                }
                if fs_changed || ctrl.agc != last_applied_ctrl.agc {
                    let _ = ov.set_gain_ctrl(sccb, ctrl.agc);
                }
                if fs_changed || ctrl.aec != last_applied_ctrl.aec {
                    let _ = ov.set_exposure_ctrl(sccb, ctrl.aec);
                }
                if fs_changed || ctrl.ae_level != last_applied_ctrl.ae_level {
                    let _ = ov.set_ae_level(sccb, ctrl.ae_level);
                }
            }
            last_applied_ctrl = ctrl;
            post_snapshot_refresh_at_ms = Some(now_ms + 150);
        }

        // Reboot if OTA firmware update completed and all Port 80 TX buffers have flushed
        if runtime.server.is_reboot_scheduled() {
            let all_flushed = tcp80_handles
                .iter()
                .all(|&h| sockets.get_mut::<TcpSocket>(h).send_queue() == 0);
            if all_flushed {
                let _ = iface.poll(smol_now(), &mut net_device, &mut sockets);
                delay.delay_millis(250);
                println!(
                    "[KLIPPER-CAM-RS] [OTA] Flash complete, deasserting PSRAM CS# and rebooting into updated slot..."
                );
                let _ = hw_i2s_cam.stop();
                quiesce_psram_and_software_reset();
            }
        }

        // ============================================================
        // Port 81 2-Socket Symmetric MJPEG Stream Pool (/stream & /)
        // ============================================================
        if let Some(owner) = p81_stream_owner {
            let sock = sockets.get_mut::<TcpSocket>(tcp81_handles[owner]);
            let cur_q = sock.send_queue();
            if cur_q < p81_last_send_queue || (p81_tx_off == p81_tx_len && cur_q == 0) {
                p81_last_send_queue = cur_q;
                p81_last_tx_progress_ms = now_ms;
            }
            let tx_stalled = (p81_tx_off < p81_tx_len || cur_q > 0)
                && (now_ms - p81_last_tx_progress_ms >= 10_000);
            let peer_disconnected = !sock.is_open()
                || !sock.is_active()
                || !sock.may_send()
                || matches!(
                    sock.state(),
                    TcpState::CloseWait
                        | TcpState::TimeWait
                        | TcpState::Closed
                        | TcpState::Closing
                        | TcpState::LastAck
                )
                || tx_stalled;
            if peer_disconnected {
                sock.abort();
                p81_states[owner] = Port81WorkerState::default();
                p81_stream_owner = None;
                p81_tx_off = 0;
                p81_tx_len = 0;
                p81_last_send_queue = 0;
                p81_next_frame_captured = false;
                runtime
                    .server
                    .stream_active_flag()
                    .store(false, Ordering::Release);
            }
        }

        for idx in 0..PORT81_SOCKET_COUNT {
            if p81_stream_owner == Some(idx) {
                let mut enqueued_more = false;
                let all_enqueued = {
                    let sock81 = sockets.get_mut::<TcpSocket>(tcp81_handles[idx]);
                    if sock81.can_recv() {
                        let drain = unsafe { &mut *core::ptr::addr_of_mut!(P81_DRAIN_BUF) };
                        let _ = sock81.recv_slice(drain);
                    }
                    if p81_tx_off < p81_tx_len {
                        let prev_off = p81_tx_off;
                        enqueue_tcp_sliding_window(
                            sock81,
                            p81_resp_buf,
                            &mut p81_tx_off,
                            p81_tx_len,
                        );
                        if p81_tx_off != prev_off {
                            p81_last_tx_progress_ms = now_ms;
                            p81_last_send_queue = sock81.send_queue();
                            enqueued_more = true;
                        }
                    }
                    p81_tx_off >= p81_tx_len
                };

                if enqueued_more {
                    poll_net_burst(&mut iface, &mut net_device, &mut sockets);
                }

                // Pipeline capture of Frame N+1 as soon as the final TCP batch of Frame N has been
                // enqueued into smoltcp (while the Wi-Fi radio transmits that final <= 14.2 KB batch
                // in the background and `poll_net_burst` services ACKs during VSYNC alignment).
                if all_enqueued && !p81_next_frame_captured {
                    let p80_actively_sending_now = p80_states.iter().any(|s| {
                        s.tx_len > 0
                            && !s.closing
                            && s.close_deadline_ms.saturating_sub(now_ms) > 3_500
                    });
                    if ov2640_dma_ok
                        && !p80_actively_sending_now
                        && ota_accum_owner.is_none()
                        && !runtime.server.is_reboot_scheduled()
                    {
                        last_capture_ms = now_ms;
                        if capture_live_hd_jpeg_with_diag(
                            hw_i2s_cam,
                            ping_pong,
                            &mut || {
                                poll_net_burst(&mut iface, &mut net_device, &mut sockets);
                            },
                        )
                        .is_ok()
                        {
                            let fresh_slice = active_ping_pong_static_slice(ping_pong);
                            runtime.server.camera.set_frame(
                                fresh_slice,
                                runtime.server.control_state.framesize,
                            );
                            let after_ms = smol_now().total_millis();
                            last_capture_ms = after_ms;
                            p81_last_tx_progress_ms = after_ms;
                        }
                        p81_next_frame_captured = true;
                    } else if !ov2640_dma_ok {
                        p81_next_frame_captured = true;
                    }
                }

                let tx_drained = {
                    let sock81 = sockets.get_mut::<TcpSocket>(tcp81_handles[idx]);
                    all_enqueued && sock81.can_send() && sock81.send_queue() == 0
                };

                if tx_drained {
                    {
                        let sock81 = sockets.get_mut::<TcpSocket>(tcp81_handles[idx]);
                        if let Ok(chunk) =
                            write_active_stream_frame_chunk(&runtime.server.camera, p81_resp_buf)
                        {
                            p81_tx_len = chunk.len();
                            p81_tx_off = 0;
                            p81_next_frame_captured = false;
                            enqueue_tcp_sliding_window(
                                sock81,
                                p81_resp_buf,
                                &mut p81_tx_off,
                                p81_tx_len,
                            );
                            p81_last_tx_progress_ms = smol_now().total_millis();
                            p81_last_send_queue = sock81.send_queue();
                        }
                    }
                    poll_net_burst(&mut iface, &mut net_device, &mut sockets);
                }
            } else {
                let mut preempt_and_start = false;
                {
                    let sock81 = sockets.get_mut::<TcpSocket>(tcp81_handles[idx]);
                    let st = &mut p81_states[idx];

                    if !sock81.is_open() {
                        *st = Port81WorkerState::default();
                        sock81.set_timeout(None);
                        sock81.set_keep_alive(Some(smoltcp::time::Duration::from_millis(2_000)));
                        let _ = sock81.listen(81);
                    } else {
                        if sock81.is_active() {
                            if st.conn_active_ms == 0 {
                                st.conn_active_ms = now_ms;
                            }
                        } else {
                            st.conn_active_ms = 0;
                        }

                        if st.closing {
                            let finished_cleanly = matches!(
                                sock81.state(),
                                TcpState::TimeWait | TcpState::Closed
                            );
                            if !sock81.is_active()
                                || finished_cleanly
                                || now_ms >= st.close_deadline_ms
                            {
                                sock81.abort();
                                *st = Port81WorkerState::default();
                            }
                        } else {
                            if sock81.can_recv() {
                                if st.req_started_ms == 0 {
                                    st.req_started_ms = now_ms as u64;
                                }
                                let avail = p81_req_bufs[idx].len().saturating_sub(st.req_len);
                                if avail > 0 {
                                    if let Ok(n) = sock81.recv_slice(
                                        &mut p81_req_bufs[idx][st.req_len..st.req_len + avail],
                                    ) {
                                        st.req_len += n;
                                    }
                                }
                                if let Ok(parsed) =
                                    parse_http_request(&p81_req_bufs[idx][..st.req_len])
                                {
                                    let is_stream_get = parsed.method == HttpMethod::Get
                                        && (parsed.path == "/stream" || parsed.path == "/");
                                    if is_stream_get {
                                        preempt_and_start = true;
                                    } else {
                                        let err_buf =
                                            unsafe { &mut *core::ptr::addr_of_mut!(P81_ERR_BUF) };
                                        if let Ok(resp) = runtime.server.handle_raw_request(
                                            ServerPort::Stream,
                                            &p81_req_bufs[idx][..st.req_len],
                                            err_buf,
                                        ) {
                                            let _ = sock81.send_slice(resp);
                                        }
                                        sock81.close();
                                        st.closing = true;
                                        st.close_deadline_ms = now_ms + 2_000;
                                    }
                                } else if st.req_len >= p81_req_bufs[idx].len()
                                    || matches!(sock81.state(), TcpState::CloseWait)
                                {
                                    sock81.close();
                                    st.closing = true;
                                    st.close_deadline_ms = now_ms + 2_000;
                                }
                            }

                            if !preempt_and_start && !st.closing {
                                let idle_or_half_closed = (st.conn_active_ms > 0
                                    && now_ms - st.conn_active_ms > 3_000)
                                    || (matches!(sock81.state(), TcpState::CloseWait)
                                        && !sock81.can_recv());
                                if idle_or_half_closed {
                                    sock81.abort();
                                    *st = Port81WorkerState::default();
                                }
                            }
                        }
                    }
                }

                if preempt_and_start {
                    // Cleanly preempt any prior/stale Port 81 stream so browser Reloads or
                    // immediate reconnects succeed 100% of the time with zero stale locks.
                    if let Some(prev_owner) = p81_stream_owner {
                        if prev_owner != idx {
                            let prev_sock = sockets.get_mut::<TcpSocket>(tcp81_handles[prev_owner]);
                            prev_sock.abort();
                            p81_states[prev_owner] = Port81WorkerState::default();
                        }
                    }
                    p81_stream_owner = None;
                    runtime
                        .server
                        .stream_active_flag()
                        .store(false, Ordering::Release);

                    let mut total_written = 0usize;
                    if let Some(mut guard) =
                        StreamSessionGuard::try_acquire(runtime.server.stream_active_flag())
                    {
                        if let Ok(hdr_bytes) = guard.write_stream_headers(p81_resp_buf) {
                            let hdr_len = hdr_bytes.len();
                            if let Ok(frame_bytes) = guard.write_next_frame_chunk(
                                &runtime.server.camera,
                                &mut p81_resp_buf[hdr_len..],
                            ) {
                                total_written = hdr_len + frame_bytes.len();
                            } else {
                                total_written = hdr_len;
                            }
                        }
                        if total_written > 0 {
                            core::mem::forget(guard);
                        }
                    }

                    let sock81 = sockets.get_mut::<TcpSocket>(tcp81_handles[idx]);
                    if total_written > 0 {
                        p81_stream_owner = Some(idx);
                        p81_states[idx] = Port81WorkerState::default();
                        p81_tx_len = total_written;
                        p81_tx_off = 0;
                        p81_next_frame_captured = false;
                        enqueue_tcp_sliding_window(
                            sock81,
                            p81_resp_buf,
                            &mut p81_tx_off,
                            p81_tx_len,
                        );
                        p81_last_tx_progress_ms = now_ms;
                        p81_last_send_queue = sock81.send_queue();
                    } else {
                        sock81.close();
                        p81_states[idx].closing = true;
                        p81_states[idx].close_deadline_ms = now_ms + 2_000;
                    }
                }
            }
        }

        poll_net_burst(&mut iface, &mut net_device, &mut sockets);

        if now_ms - last_heartbeat_ms >= 30_000 {
            last_heartbeat_ms = now_ms;
            println!(
                "[KLIPPER-CAM-RS] Heartbeat: uptime={}ms connected={} stream_active={} frame_seq={} last_jpeg={}B",
                now_ms,
                wifi_controller.is_connected(),
                p81_stream_owner.is_some(),
                ping_pong.frame_seq(),
                ping_pong.active_len()
            );
        }
    }
}

#[cfg(target_arch = "xtensa")]
#[inline(never)]
fn configure_xclk(
    ledc_periph: esp_hal::peripherals::LEDC<'static>,
    gpio21: esp_hal::peripherals::GPIO21<'static>,
) -> bool {
    use esp_hal::{
        gpio::DriveMode,
        ledc::{
            channel::{self, ChannelIFace},
            timer::{self, TimerIFace},
            LSGlobalClkSource, Ledc, LowSpeed,
        },
        time::Rate,
    };
    static LEDC_CELL: static_cell::StaticCell<Ledc<'static>> = static_cell::StaticCell::new();
    static LSTIMER0_CELL: static_cell::StaticCell<timer::Timer<'static, LowSpeed>> =
        static_cell::StaticCell::new();
    static CHANNEL0_CELL: static_cell::StaticCell<channel::Channel<'static, LowSpeed>> =
        static_cell::StaticCell::new();

    let ledc = LEDC_CELL.init(Ledc::new(ledc_periph));
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
    let lstimer0 = LSTIMER0_CELL.init(ledc.timer::<LowSpeed>(timer::Number::Timer0));
    let xclk_timer_ok = lstimer0
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty1Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: Rate::from_mhz(20),
        })
        .is_ok();
    let channel0 = CHANNEL0_CELL.init(ledc.channel(channel::Number::Channel0, gpio21));
    xclk_timer_ok
        && channel0
            .configure(channel::config::Config {
                timer: lstimer0,
                duty_pct: 50,
                drive_mode: DriveMode::PushPull,
            })
            .is_ok()
}

#[cfg(target_arch = "xtensa")]
#[allow(clippy::type_complexity)]
#[inline(never)]
fn configure_i2c0_and_ov2640(
    i2c0_periph: esp_hal::peripherals::I2C0<'static>,
    sda: esp_hal::peripherals::GPIO26<'static>,
    scl: esp_hal::peripherals::GPIO27<'static>,
) -> (
    bool,
    Option<Ov2640>,
    Option<klipper_cam_fw::EmbeddedHalSccb<esp_hal::i2c::master::I2c<'static, esp_hal::Blocking>, esp_hal::delay::Delay>>,
) {
    use esp_hal::{
        delay::Delay,
        i2c::master::{Config as I2cConfig, I2c},
        time::Rate,
    };
    use esp_println::println;
    use klipper_cam_fw::EmbeddedHalSccb;

    let i2c_cfg = I2cConfig::default().with_frequency(Rate::from_khz(200));
    match I2c::new(i2c0_periph, i2c_cfg) {
        Ok(i2c_bus) => {
            let i2c_bus = i2c_bus.with_sda(sda).with_scl(scl);
            let mut hw_sccb = EmbeddedHalSccb::new(i2c_bus, Delay::new());
            let (prog_ok, ov) = program_ov2640_over_sccb(&mut hw_sccb);
            (prog_ok, ov, Some(hw_sccb))
        }
        Err(_) => {
            println!("[KLIPPER-CAM-RS] [HW-2] I2C0 Master Init: FAIL");
            (false, None, None)
        }
    }
}

#[cfg(target_arch = "xtensa")]
#[allow(clippy::too_many_arguments)]
#[inline(never)]
fn configure_camera_inputs(
    p4: esp_hal::peripherals::GPIO4<'static>,
    p5: esp_hal::peripherals::GPIO5<'static>,
    p18: esp_hal::peripherals::GPIO18<'static>,
    p19: esp_hal::peripherals::GPIO19<'static>,
    p36: esp_hal::peripherals::GPIO36<'static>,
    p39: esp_hal::peripherals::GPIO39<'static>,
    p34: esp_hal::peripherals::GPIO34<'static>,
    p35: esp_hal::peripherals::GPIO35<'static>,
    p22: esp_hal::peripherals::GPIO22<'static>,
    p23: esp_hal::peripherals::GPIO23<'static>,
    p25: esp_hal::peripherals::GPIO25<'static>,
) {
    use esp_hal::gpio::{Input, InputConfig, Pull};
    static CAM_INPUTS_CELL: static_cell::StaticCell<[Input<'static>; 11]> =
        static_cell::StaticCell::new();
    let _ = CAM_INPUTS_CELL.init([
        Input::new(p4, InputConfig::default()),
        Input::new(p5, InputConfig::default()),
        Input::new(p18, InputConfig::default()),
        Input::new(p19, InputConfig::default()),
        Input::new(p36, InputConfig::default()),
        Input::new(p39, InputConfig::default()),
        Input::new(p34, InputConfig::default()),
        Input::new(p35, InputConfig::default()),
        Input::new(p22, InputConfig::default()),
        Input::new(p23, InputConfig::default()),
        Input::new(p25, InputConfig::default().with_pull(Pull::Up)),
    ]);
}

/// Bare-metal Xtensa firmware entrypoint for `ESP32-WROVER-DEV`.
#[cfg(target_arch = "xtensa")]
#[esp_hal::main]
fn main() -> ! {
    use esp_hal::{
        clock::CpuClock,
        delay::Delay,
        psram::{Psram, PsramCacheSpeed, PsramConfig, PsramSize, PsramVaddrMode},
    };
    use esp_println::println;

    // 0. Exact ESP-IDF v5.5.4 + klipper_cam.c Startup Sequence:
    //    1) Read reset_reason, hold GPIO 16 (PSRAM CS#) HIGH from IRAM, initialize clocks (240 MHz),
    //       and immediately re-arm the 15s hardware RTC_WDT.
    //    2) Power down/isolate the Wi-Fi/RF domain during early peripheral init (matching esp_rtc_init()).
    //    3) Never drive GPIO 4 as an output on ESP32-WROVER-DEV (`GPIO 4` is `CAM_PIN_D0` / `OV2640` `Y2`).
    //    4) Record boot breadcrumb in RTC_CNTL_STORE6/7_REG and detect booted OTA slot from MMU
    //       with ZERO boot-time NOR flash operations.
    const RTC_CNTL_RESET_STATE_REG: *const u32 = 0x3FF4_8034 as *const u32;
    let reset_reason = unsafe { core::ptr::read_volatile(RTC_CNTL_RESET_STATE_REG) };
    hold_psram_cs_high_safe();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::_240MHz);
    let peripherals = esp_hal::init(config);
    arm_rtc_wdt_15s();
    esp_rtc_wifi_force_pd(true);
    let delay = Delay::new();

    let bc_prev = record_boot_breadcrumb_rtc_init(reset_reason);
    let ota_manager = detect_booted_ota_manager_from_mmu();
    let flash_adapter = EspFlashAdapter(esp_storage::FlashStorage::new(peripherals.FLASH));
    feed_rtc_wdt();

    // Initialize internal DRAM heap for esp-rtos and esp-radio Wi-Fi STA (~112 KB strictly internal)
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 64 * 1024);
    esp_alloc::heap_allocator!(size: 48 * 1024);

    // Initialize 4 MB external PSRAM at 40 MHz Flash / 40 MHz PSRAM (matching a364a6a^:sdkconfig.defaults
    // CONFIG_ESPTOOLPY_FLASHFREQ_40M=y + CONFIG_SPIRAM_SPEED_40M=y) and send 0xF5 (QPI exit) + 0x35 (1-bit SPI
    // QPI enter) via hardware SPI1_CS1 (ensure_psram_qpi_mode_spi1) without ever bit-banging SD0..SD3!
    // On cold power-on reset (`(reset_reason & 0x0F) == 1`), wait 50 ms while GPIO 16 (PSRAM CS#) is held HIGH
    // so VDD_SDIO and the PSRAM chip's internal POR stabilize cleanly at 240 MHz before Psram::new().
    if (reset_reason & 0x0F) == 1 {
        delay.delay_millis(50);
        feed_rtc_wdt();
    }
    let psram = Psram::new(
        peripherals.PSRAM,
        PsramConfig {
            size: PsramSize::Size(4 * 1024 * 1024),
            cache_speed: PsramCacheSpeed::PsramCacheF40mS40m,
            psram_vaddr_mode: PsramVaddrMode::Normal,
        },
    );
    ensure_psram_qpi_mode_spi1();
    feed_rtc_wdt();

    let (psram_ptr, psram_len) = psram.raw_parts();

    // 1. Configure physical 20 MHz XCLK on GPIO 21 via LEDC LowSpeed Timer0 / Channel0
    let xclk_hw_ok = configure_xclk(peripherals.LEDC, peripherals.GPIO21);
    println!(
        "[KLIPPER-CAM-RS] [HW-1] LEDC 20 MHz XCLK on GPIO 21: {}",
        if xclk_hw_ok { "PASS" } else { "FAIL" }
    );
    delay.delay_millis(25);
    feed_rtc_wdt();

    // 2. Configure physical I2C0 on GPIO 26 (SDA) / GPIO 27 (SCL) at 200 kHz and program OV2640
    let (ov2640_hw_ok, live_ov2640, kept_sccb) =
        configure_i2c0_and_ov2640(peripherals.I2C0, peripherals.GPIO26, peripherals.GPIO27);
    feed_rtc_wdt();

    // 3. Carve PSRAM buffers, run 7-crate HIL verification suite, and configure I2S0 camera
    init_psram_and_run_hil(psram_ptr, psram_len, xclk_hw_ok);

    // 4. Configure physical camera input pads via esp-hal AFTER `set_pin`
    configure_camera_inputs(
        peripherals.GPIO4,
        peripherals.GPIO5,
        peripherals.GPIO18,
        peripherals.GPIO19,
        peripherals.GPIO36,
        peripherals.GPIO39,
        peripherals.GPIO34,
        peripherals.GPIO35,
        peripherals.GPIO22,
        peripherals.GPIO23,
        peripherals.GPIO25,
    );
    delay.delay_millis(80);

    // 5. Capture initial 1280x720 JPEG frame and initialize FirmwareRuntime
    let ov2640_dma_ok = capture_boot_dma_and_init_runtime(ov2640_hw_ok, bc_prev, ota_manager);

    // 6. Release Wi-Fi/RF force power-down, start esp-rtos scheduler, and initialize esp-radio Wi-Fi Station
    esp_rtc_wifi_force_pd(false);
    let timg0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);
    feed_rtc_wdt();

    let (sta_cfg, ctrl_cfg, ssid) = build_wifi_station_configs();
    feed_rtc_wdt();
    let wifi_controller =
        esp_radio::wifi::WifiController::new(peripherals.WIFI, ctrl_cfg).expect("WifiController::new");
    feed_rtc_wdt();

    run_post_wifi_server_loop(
        wifi_controller,
        sta_cfg,
        ssid,
        flash_adapter,
        kept_sccb,
        live_ov2640,
        ov2640_dma_ok,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_main_firmware_self_test() {
        assert!(run_firmware_self_test());
    }
}
