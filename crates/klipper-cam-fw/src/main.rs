//! Entrypoint for `klipper-cam-fw`.
//!
//! When compiled for host (`x86_64` / `aarch64`), runs a self-verification
//! check of the partition table, OTA CRC32 state machine, and dual-port
//! HTTP/MJPEG handlers.
//! When compiled for Xtensa bare-metal (`target_arch = "xtensa"`), initializes
//! the ESP32 at `240 MHz`, drives `20 MHz` `XCLK` on `GPIO 21`, probes `I2C0`
//! (`GPIO 26` SDA / `GPIO 27` SCL) for a live `OV2640` sensor, and executes
//! the complete 7-crate hardware-in-the-loop (HIL) verification suite over
//! UART0 (`115200` baud).

#![cfg_attr(target_arch = "xtensa", no_std)]
#![cfg_attr(target_arch = "xtensa", no_main)]
#![cfg_attr(target_arch = "xtensa", feature(asm_experimental_arch))]
#![cfg_attr(not(target_arch = "xtensa"), forbid(unsafe_code))]
#![cfg_attr(
    target_arch = "xtensa",
    allow(
        unsafe_op_in_unsafe_fn,
        clippy::undocumented_unsafe_blocks,
        clippy::unwrap_used
    )
)]
#![deny(static_mut_refs)]

#[cfg(target_arch = "xtensa")]
extern crate alloc;

#[cfg(target_arch = "xtensa")]
use esp_backtrace as _;

#[cfg(target_arch = "xtensa")]
esp_bootloader_esp_idf::esp_app_desc!();

use esp32_cam_ll::{
    build_circular_ring, verify_circular_ring, Lldesc, DEFAULT_CAMERA_DMA_BUFFER_SIZE_MAX,
};
use klipper_cam_fw::{
    calc_dma_sizes, decode_jpeg_to_rgb888, encode_rgb888_to_jpeg, parse_jpeg_dimensions,
    verify_jpeg_eoi, verify_jpeg_soi, verify_partition_table_integrity, BoardPins, CameraDriver,
    CameraSensor, DmaElement, Esp32I2sCamera, EspImageHeader, FirmwareRuntime, FramePool,
    FrameSize, GrabMode, HttpResponseView, Ipv4Addr, KlipperCamConfig, MockSccbBus, OtaSlot,
    Ov2640, PingPongFrameBuffer, PixelFormat, ServerPort, SliceWriter, WroverKitPins, XclkConfig,
    XclkController, DEFAULT_JPEG_QUALITY, DEFAULT_XCLK_FREQ_HZ, MJPEG_CONTENT_TYPE,
};

#[cfg(target_arch = "xtensa")]
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
#[cfg(target_arch = "xtensa")]
use esp_hal::system::{Cpu, CpuControl, Stack};
#[cfg(target_arch = "xtensa")]
use klipper_cam_fw::telemetry::{
    ccount, format_sha256_hex, measure_stack_hwm_volatile, paint_stack_sentinel,
    reset_reason_label, FwStats,
};
#[cfg(target_arch = "xtensa")]
use klipper_cam_fw::{
    write_http_error, Core1JpegSpsc, TripleFrameBuffer, JPEG_SPSC_CHUNK_BYTES, SPSC_FLAG_ABORT,
    SPSC_FLAG_EOF, SPSC_FLAG_SOF,
};

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

    let hsize8 = sccb_bus.reg8(Ov2640::SLV_ADDR, esp32_cam_sensors::ov2640::HSIZE8);
    let vsize8 = sccb_bus.reg8(Ov2640::SLV_ADDR, esp32_cam_sensors::ov2640::VSIZE8);
    let yoffl = sccb_bus.reg8(Ov2640::SLV_ADDR, esp32_cam_sensors::ov2640::YOFFL);

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
    let started = i2s_cam.start(&dma_sizing, nodes[0].link_addr_20bit()) && i2s_cam.is_running();
    let stopped = i2s_cam.stop() && !i2s_cam.is_running();
    let deinit_ok = i2s_cam.deinit().is_ok();
    started && stopped && deinit_ok
}

/// Private token for the in-memory HTTP/OTA self-test (never a device secret).
const HIL_SELF_TEST_OTA_TOKEN: &str = "hil-self-test-ota-token";

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
    if pp
        .write_and_commit_inactive(&jpeg_encoded[..jpeg_len])
        .is_err()
        || pp.frame_seq() != 2
        || pp.active_slot_index() != 1
        || pp.active_frame() != &jpeg_encoded[..jpeg_len]
    {
        return false;
    }

    let fw_cam = CameraDriver::new(&jpeg_encoded[..jpeg_len], FrameSize::Hd);
    // The public default OTA token is refused with 403 (FW-NET-04), so the
    // in-memory OTA self-test uses its own private token.
    let mut self_test_cfg = KlipperCamConfig::WROVER_KIT_HD;
    self_test_cfg.ota_token = HIL_SELF_TEST_OTA_TOKEN;
    let mut runtime = FirmwareRuntime::new(self_test_cfg, fw_cam);
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
        let _ = w.write_str_checked(HIL_SELF_TEST_OTA_TOKEN);
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
static mut APP_CORE_STACK: Stack<8192> = Stack::new();

#[cfg(target_arch = "xtensa")]
static FW_STATS: FwStats = FwStats::new();

#[cfg(target_arch = "xtensa")]
static CORE1_SPSC: Core1JpegSpsc<32> = Core1JpegSpsc::new();

#[cfg(target_arch = "xtensa")]
static CORE1_PAUSE_REQ: AtomicBool = AtomicBool::new(false);

#[cfg(target_arch = "xtensa")]
static CORE1_IS_PAUSED: AtomicBool = AtomicBool::new(false);

#[cfg(target_arch = "xtensa")]
static CORE1_ACTIVE: AtomicBool = AtomicBool::new(false);

/// Requests Core 1 to stop I2S0 capture and waits for it to acknowledge.
///
/// Returns `true` once Core 1 is parked in its IRAM-resident pause loop (or
/// when Core 1 was never started). Returns `false` on timeout, in which case
/// Core 1 may still be executing flash-resident code: callers that are about
/// to disable the flash cache must treat that as a failure, not proceed.
#[cfg(target_arch = "xtensa")]
#[esp_hal::ram]
fn pause_core1_camera_producer() -> bool {
    if Cpu::current() == Cpu::AppCpu {
        return true;
    }
    if !CORE1_ACTIVE.load(Ordering::Acquire) {
        return true;
    }
    CORE1_PAUSE_REQ.store(true, Ordering::Release);
    for spin in 0..2_000_000u32 {
        if CORE1_IS_PAUSED.load(Ordering::Acquire) {
            return true;
        }
        if (spin & 0xFFF) == 0 {
            feed_rtc_wdt();
        }
        core::hint::spin_loop();
    }
    false
}

/// Releases Core 1 from its pause loop and waits until it has actually left
/// it, so a subsequent `pause_core1_camera_producer` can never observe a stale
/// `CORE1_IS_PAUSED == true` from this pause cycle.
///
/// Returns `true` once Core 1 has exited its pause loop (or when Core 1 was
/// never started). Returns `false` on timeout.
#[cfg(target_arch = "xtensa")]
#[esp_hal::ram]
fn resume_core1_camera_producer() -> bool {
    CORE1_PAUSE_REQ.store(false, Ordering::Release);
    if !CORE1_ACTIVE.load(Ordering::Acquire) {
        return true;
    }
    for spin in 0..2_000_000u32 {
        if !CORE1_IS_PAUSED.load(Ordering::Acquire) {
            return true;
        }
        if (spin & 0xFFF) == 0 {
            feed_rtc_wdt();
        }
        core::hint::spin_loop();
    }
    false
}

#[cfg(target_arch = "xtensa")]
#[esp_hal::ram]
fn ensure_spi1_flash_cs0_only() {
    const SPI1_PIN_REG: *mut u32 = 0x3FF4_2034 as *mut u32;
    // JUSTIFICATION: Hardware MMIO register access for SPI1 CS pin config.
    // SAFETY: SPI1_PIN_REG is a valid MMIO register mapped in peripheral space.
    unsafe {
        let pin = core::ptr::read_volatile(SPI1_PIN_REG);
        // Enforce CS1_DIS = 1 (bit 1, PSRAM CS# disabled) and CS0_DIS = 0
        // (bit 0, Flash CS# enabled)
        core::ptr::write_volatile(SPI1_PIN_REG, (pin | (1u32 << 1)) & !1u32);
    }
}

#[cfg(target_arch = "xtensa")]
struct XtensaCore1Park;

#[cfg(target_arch = "xtensa")]
impl klipper_cam_fw::Core1ParkControl for XtensaCore1Park {
    fn pause_core1(&self) -> bool {
        pause_core1_camera_producer()
    }

    fn resume_core1(&self) -> bool {
        let ok = resume_core1_camera_producer();
        if !ok {
            FW_STATS.inc_sccb_errors();
            esp_println::println!("camera: Core 1 resume timed out after flash op");
        }
        ok
    }
}

#[cfg(target_arch = "xtensa")]
struct EspFlashAdapter<'d>(esp_storage::FlashStorage<'d>);

#[cfg(target_arch = "xtensa")]
impl klipper_cam_fw::OtaFlashStorage for EspFlashAdapter<'_> {
    fn read_aligned(
        &mut self,
        offset: u32,
        buf: &mut [u8],
    ) -> Result<(), klipper_cam_fw::OtaError> {
        let _guard = klipper_cam_fw::Core1PauseGuard::new(&XtensaCore1Park)?;
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
        let _guard = klipper_cam_fw::Core1PauseGuard::new(&XtensaCore1Park)?;
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
        let _guard = klipper_cam_fw::Core1PauseGuard::new(&XtensaCore1Park)?;
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
static BOOT_RESET_REASON: AtomicU32 = AtomicU32::new(0);

#[cfg(target_arch = "xtensa")]
static mut APP_SHA256_HEX: [u8; 64] = [0u8; 64];

#[cfg(target_arch = "xtensa")]
static APP_SHA256_VALID: AtomicBool = AtomicBool::new(false);

#[cfg(target_arch = "xtensa")]
fn init_boot_app_sha256(flash: &mut EspFlashAdapter<'_>, slot: OtaSlot) {
    use klipper_cam_fw::OtaFlashStorage as _;

    #[repr(C, align(4))]
    struct AlignedFlashBuf32([u8; 32]);

    let base_off: u32 = match slot {
        OtaSlot::Ota0 => 0x0002_0000,
        OtaSlot::Ota1 => 0x001D_0000,
    };

    let mut hdr_buf = AlignedFlashBuf32([0u8; 32]);
    if flash.read_aligned(base_off, &mut hdr_buf.0[..24]).is_err() {
        return;
    }
    let hdr = &hdr_buf.0[..24];
    let seg_count = hdr[1];
    if hdr[0] != 0xE9 || seg_count == 0 || seg_count > 16 || hdr[23] != 1 {
        return;
    }

    let mut pos: u32 = 24;
    let mut seg_idx: u8 = 0;
    while seg_idx < seg_count {
        let Some(abs_off) = base_off.checked_add(pos) else {
            return;
        };
        let aligned_off = abs_off & !3;
        let shift = (abs_off & 3) as usize;
        let read_len = if shift == 0 { 8usize } else { 12usize };
        let mut seg_buf = AlignedFlashBuf32([0u8; 32]);
        if flash
            .read_aligned(aligned_off, &mut seg_buf.0[..read_len])
            .is_err()
        {
            return;
        }
        let seg_hdr = &seg_buf.0[shift..shift + 8];
        let _load_addr = u32::from_le_bytes([seg_hdr[0], seg_hdr[1], seg_hdr[2], seg_hdr[3]]);
        let data_len = u32::from_le_bytes([seg_hdr[4], seg_hdr[5], seg_hdr[6], seg_hdr[7]]);
        if data_len > klipper_cam_fw::OTA_0_SIZE {
            return;
        }
        let Some(after_hdr) = pos.checked_add(8) else {
            return;
        };
        let Some(after_seg) = after_hdr.checked_add(data_len) else {
            return;
        };
        if after_seg > klipper_cam_fw::OTA_0_SIZE {
            return;
        }
        pos = after_seg;
        seg_idx += 1;
    }

    let sha_off = (pos + 1 + 15) & !15;
    let Some(sha_end) = sha_off.checked_add(32) else {
        return;
    };
    if sha_end > klipper_cam_fw::OTA_0_SIZE {
        return;
    }
    let Some(abs_sha_off) = base_off.checked_add(sha_off) else {
        return;
    };
    let mut digest_buf = AlignedFlashBuf32([0u8; 32]);
    if flash.read_aligned(abs_sha_off, &mut digest_buf.0).is_err() {
        return;
    }

    // JUSTIFICATION: Populate static 64-byte SHA-256 hex buffer once at
    // single-threaded boot before starting Core 1 or the HTTP server loop.
    // SAFETY:
    // 1. Preconditions: Executed once during early single-core boot init.
    // 2. Alignment: `[u8; 64]` has alignment 1 and is statically allocated.
    // 3. Provenance & Lifetime: `APP_SHA256_HEX` lives in `.bss` for `'static`.
    // 4. Aliasing: No shared references exist before `APP_SHA256_VALID` is set.
    // 5. Volatile Semantics: Published via `Ordering::Release` on `AtomicBool`.
    // 6. Panic Safety: `format_sha256_hex` writes bounded indices `0..64`.
    let ok = unsafe {
        let hex_out = &mut *core::ptr::addr_of_mut!(APP_SHA256_HEX);
        format_sha256_hex(&digest_buf.0, hex_out).len() == 64
    };
    if ok {
        APP_SHA256_VALID.store(true, Ordering::Release);
    }
}

#[cfg(target_arch = "xtensa")]
fn boot_app_sha256_str() -> &'static str {
    if !APP_SHA256_VALID.load(Ordering::Acquire) {
        return "unknown";
    }
    // JUSTIFICATION: Borrow immutable `'static` SHA-256 hex buffer after
    // one-time boot initialization ordered by `APP_SHA256_VALID`.
    // SAFETY:
    // 1. Preconditions: `APP_SHA256_VALID` is `true` via `Acquire` load.
    // 2. Alignment: `[u8; 64]` has 1-byte alignment.
    // 3. Provenance & Lifetime: Static `.bss` buffer lives for `'static`.
    // 4. Aliasing: Never mutated after `APP_SHA256_VALID` Release store.
    // 5. Volatile Semantics: Synchronized via Acquire/Release atomic flag.
    // 6. Panic Safety: Fallible `from_utf8` match cannot panic or unwind.
    let bytes: &'static [u8; 64] = unsafe { &*core::ptr::addr_of!(APP_SHA256_HEX) };
    core::str::from_utf8(bytes).unwrap_or("unknown")
}

#[cfg(target_arch = "xtensa")]
#[esp_hal::ram]
fn detect_booted_ota_manager_from_mmu() -> OtaSlot {
    const DPORT_PRO_FLASH_MMU_TABLE: *const u32 = 0x3FF1_0000 as *const u32;
    // JUSTIFICATION: Read hardware MMU table entries for DROM and IROM to
    // identify which OTA flash partition (`ota_0` vs `ota_1`) is mapped, with
    // zero SPI NOR flash commands during early cold boot.
    // SAFETY:
    // 1. Preconditions: PRO_CPU MMU table is active while executing this code.
    // 2. Alignment: `0x3FF1_0000` and `+77 * 4` are 4-byte aligned `u32` MMIO.
    // 3. Provenance & Lifetime: Fixed ESP32 DPORT MMIO aperture.
    // 4. Aliasing: Read-only volatile MMIO inspection forms no Rust references.
    // 5. Synchronization: Single-core read of immutable boot MMU mapping.
    // 6. Unwind safety: Volatile register reads cannot panic or unwind.
    let (drom_entry, irom_entry) = unsafe {
        (
            core::ptr::read_volatile(DPORT_PRO_FLASH_MMU_TABLE),
            core::ptr::read_volatile(DPORT_PRO_FLASH_MMU_TABLE.wrapping_add(77)),
        )
    };
    let drom_is_ota1 = (drom_entry & 0x100) == 0 && (drom_entry & 0xFF) >= 0x1D;
    let irom_is_ota1 = (irom_entry & 0x100) == 0 && (irom_entry & 0xFF) >= 0x1D;
    if drom_is_ota1 || irom_is_ota1 {
        OtaSlot::Ota1
    } else {
        OtaSlot::Ota0
    }
}

#[cfg(target_arch = "xtensa")]
#[inline(never)]
fn load_boot_otadata_and_sha256(flash_adapter: &mut EspFlashAdapter<'_>, mmu_booted_slot: OtaSlot) {
    use esp_println::println;

    let active_ota_slot = match klipper_cam_fw::OtaDataManager::load_from_flash(flash_adapter) {
        Ok(loaded) => {
            let flash_slot = loaded.active_slot();
            if flash_slot != mmu_booted_slot {
                println!(
                    "[KLIPPER-CAM-RS] [OTA] Warning: otadata slot {:?} != MMU {:?}; flash wins",
                    flash_slot, mmu_booted_slot
                );
            }
            // JUSTIFICATION: Install flash-loaded `OtaDataManager` into static
            // `FIRMWARE_RUNTIME` on Core 0 before Core 1 or HTTP loop starts.
            // SAFETY:
            // 1. Alignment & Non-null: `addr_of_mut!(FIRMWARE_RUNTIME)` is a
            //    valid aligned static in `.bss`.
            // 2. Layout: `Option<FirmwareRuntime>` was initialized to `Some` in
            //    `capture_boot_dma_and_init_runtime`.
            // 3. Provenance & Lifetime: Static lives for `'static`.
            // 4. Aliasing: No other references to `FIRMWARE_RUNTIME` exist at
            //    this single-core boot point before `run_post_wifi_server_loop`.
            // 5. Synchronization: Single-threaded execution on Core 0 prior to
            //    spawning Core 1.
            // 6. Unwind safety: Bare-metal firmware builds with `panic = abort`.
            if let Some(rt) = unsafe { (*core::ptr::addr_of_mut!(FIRMWARE_RUNTIME)).as_mut() } {
                rt.server.ota_manager = loaded;
            }
            flash_slot
        }
        Err(err) => {
            FW_STATS.inc_sccb_errors();
            println!(
                "[KLIPPER-CAM-RS] [OTA] Failed to load otadata ({:?}); using MMU {:?}",
                err, mmu_booted_slot
            );
            mmu_booted_slot
        }
    };
    init_boot_app_sha256(flash_adapter, active_ota_slot);
}

#[cfg(target_arch = "xtensa")]
const RTC_BREADCRUMB_MAGIC_HI: u32 = 0xA5A5_0000;

/// ESP32 `RTC_CNTL_STORE6_REG` (`DR_REG_RTCCNTL_BASE + 0xB8`):
/// Bits `[31:16]` = `0xA5A5` magic, bits `[15:0]` = `boot_count` (`u16`).
#[cfg(target_arch = "xtensa")]
const RTC_CNTL_STORE6_REG: *mut u32 = 0x3FF4_80B8 as *mut u32;

/// ESP32 `RTC_CNTL_STORE7_REG` (`DR_REG_RTCCNTL_BASE + 0xBC`):
/// Bits `[31:24]` = `0xA5` magic, bits `[23:16]` = `aid`,
/// bits `[15:8]` = `channel`, bits `[7:0]` = `stage`.
#[cfg(target_arch = "xtensa")]
const RTC_CNTL_STORE7_REG: *mut u32 = 0x3FF4_80BC as *mut u32;

#[cfg(target_arch = "xtensa")]
#[esp_hal::ram]
fn feed_rtc_wdt() {
    const RTC_CNTL_WDTFEED_REG: *mut u32 = 0x3FF4_80A0 as *mut u32;
    const RTC_CNTL_WDTWPROTECT_REG: *mut u32 = 0x3FF4_80A4 as *mut u32;
    const RTC_CNTL_WDT_WKEY: u32 = 0x50D8_3AA1;
    // JUSTIFICATION: Hardware RTC watchdog timer register feed operation.
    // SAFETY: RTC_CNTL_WDT registers are valid MMIO addresses.
    unsafe {
        core::ptr::write_volatile(RTC_CNTL_WDTWPROTECT_REG, RTC_CNTL_WDT_WKEY);
        core::ptr::write_volatile(RTC_CNTL_WDTFEED_REG, 1u32 << 31);
        core::ptr::write_volatile(RTC_CNTL_WDTWPROTECT_REG, 0);
    }
}

#[cfg(target_arch = "xtensa")]
#[esp_hal::ram]
fn arm_rtc_wdt_15s() {
    const RTC_CNTL_INT_ENA_REG: *mut u32 = 0x3FF4_8040 as *mut u32;
    const RTC_CNTL_INT_CLR_REG: *mut u32 = 0x3FF4_8048 as *mut u32;
    const RTC_CNTL_WDTCONFIG0_REG: *mut u32 = 0x3FF4_808C as *mut u32;
    const RTC_CNTL_WDTCONFIG1_REG: *mut u32 = 0x3FF4_8090 as *mut u32;
    const RTC_CNTL_WDTFEED_REG: *mut u32 = 0x3FF4_80A0 as *mut u32;
    const RTC_CNTL_WDTWPROTECT_REG: *mut u32 = 0x3FF4_80A4 as *mut u32;
    const RTC_CNTL_WDT_WKEY: u32 = 0x50D8_3AA1;
    // WDT_EN (bit 31) | WDT_STG0 = 4 (RESET_RTC, bits 28..=30) |
    // WDT_CPU_RESET_LENGTH = 7 (bits 14..=16) | WDT_SYS_RESET_LENGTH = 7
    // (bits 11..=13) | WDT_FLASHBOOT_MOD_EN (bit 10) |
    // PROCPU_RESET_EN (bit 9) | APPCPU_RESET_EN (bit 8) | PAUSE_IN_SLP (bit 7)
    const WDT_CFG0_VAL: u32 = (1u32 << 31)
        | (4u32 << 28)
        | (7u32 << 14)
        | (7u32 << 11)
        | (1u32 << 10)
        | (1u32 << 9)
        | (1u32 << 8)
        | (1u32 << 7);
    // JUSTIFICATION: Hardware RTC watchdog timer 15-second arming configuration.
    // SAFETY: RTC_CNTL_WDT registers are valid MMIO addresses.
    unsafe {
        core::ptr::write_volatile(RTC_CNTL_INT_ENA_REG, 0);
        core::ptr::write_volatile(RTC_CNTL_INT_CLR_REG, 0xFFFF_FFFF);
        core::ptr::write_volatile(RTC_CNTL_WDTWPROTECT_REG, RTC_CNTL_WDT_WKEY);
        core::ptr::write_volatile(RTC_CNTL_WDTCONFIG1_REG, 15 * 150_000);
        core::ptr::write_volatile(RTC_CNTL_WDTCONFIG0_REG, WDT_CFG0_VAL);
        core::ptr::write_volatile(RTC_CNTL_WDTFEED_REG, 1u32 << 31);
        core::ptr::write_volatile(RTC_CNTL_WDTWPROTECT_REG, 0);
    }
}

/// Applies `ESP-IDF v5.5.4` (`rtc_init.c:30-31, 102-107` and
/// `phy_init.c:434-448`) Wi-Fi/RF modem force power-down and isolation during
/// early peripheral initialization, releases any latched RTC/digital pad holds,
/// and pulses `MODEM_RESET_FIELD_WHEN_PU` when powering the Wi-Fi domain on.
#[cfg(target_arch = "xtensa")]
#[esp_hal::ram]
fn esp_rtc_wifi_force_pd(enable_pd: bool) {
    const DPORT_CORE_RST_EN_REG: *mut u32 = 0x3FF0_00D0 as *mut u32;
    const RTC_CNTL_DIG_PWC_REG: *mut u32 = 0x3FF4_8084 as *mut u32;
    const RTC_CNTL_DIG_ISO_REG: *mut u32 = 0x3FF4_8088 as *mut u32;
    const RTC_CNTL_HOLD_FORCE_REG: *mut u32 = 0x3FF4_80C8 as *mut u32;
    const RTC_IO_DIG_PAD_HOLD_REG: *mut u32 = 0x3FF4_8474 as *mut u32;
    const MODEM_RESET_FIELD_WHEN_PU: u32 = 0x21D;
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

    // JUSTIFICATION: Hardware RTC power control and Wi-Fi modem isolation.
    // SAFETY: RTC_CNTL, RTC_IO, and DPORT registers are valid MMIO addresses.
    unsafe {
        core::ptr::write_volatile(RTC_CNTL_HOLD_FORCE_REG, 0);
        core::ptr::write_volatile(RTC_IO_DIG_PAD_HOLD_REG, 0);
        let iso = core::ptr::read_volatile(RTC_CNTL_DIG_ISO_REG) & !RTC_CNTL_DG_PAD_MASK;
        let pwc = core::ptr::read_volatile(RTC_CNTL_DIG_PWC_REG);
        if enable_pd {
            core::ptr::write_volatile(RTC_CNTL_DIG_ISO_REG, iso | RTC_CNTL_WIFI_FORCE_ISO);
            core::ptr::write_volatile(RTC_CNTL_DIG_PWC_REG, pwc | RTC_CNTL_WIFI_FORCE_PD);
        } else {
            core::ptr::write_volatile(RTC_CNTL_DIG_PWC_REG, pwc & !RTC_CNTL_WIFI_FORCE_PD);
            for _ in 0..3600u32 {
                core::hint::spin_loop();
            }
            let rst = core::ptr::read_volatile(DPORT_CORE_RST_EN_REG);
            core::ptr::write_volatile(DPORT_CORE_RST_EN_REG, rst | MODEM_RESET_FIELD_WHEN_PU);
            let _ = core::ptr::read_volatile(DPORT_CORE_RST_EN_REG);
            core::ptr::write_volatile(DPORT_CORE_RST_EN_REG, rst & !MODEM_RESET_FIELD_WHEN_PU);
            let _ = core::ptr::read_volatile(DPORT_CORE_RST_EN_REG);
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
    // JUSTIFICATION: Hardware RTC store register read for boot breadcrumb.
    // SAFETY: RTC_CNTL_STORE6_REG is a valid MMIO register address.
    let raw6 = unsafe { core::ptr::read_volatile(RTC_CNTL_STORE6_REG) };
    // JUSTIFICATION: Hardware RTC store register read for boot breadcrumb.
    // SAFETY: RTC_CNTL_STORE7_REG is a valid MMIO register address.
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
    // JUSTIFICATION: Hardware RTC store register write for boot breadcrumb.
    // SAFETY: RTC_CNTL_STORE6/7 registers are valid MMIO addresses.
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
    let val =
        (0xA5u32 << 24) | ((aid as u32 & 0xFF) << 16) | ((ch as u32 & 0xFF) << 8) | (stage & 0xFF);
    // JUSTIFICATION: Hardware RTC store register write for Wi-Fi breadcrumb.
    // SAFETY: RTC_CNTL_STORE7_REG is a valid MMIO register address.
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
        self.0
            .consume_token(copy_len, |tx_buf| f(&mut tx_buf[..copy_len]))
    }
}

#[cfg(target_arch = "xtensa")]
impl smoltcp::phy::Device for SmoltcpWifiDevice {
    type RxToken<'a>
        = SmoltcpRxToken
    where
        Self: 'a;
    type TxToken<'a>
        = SmoltcpTxToken
    where
        Self: 'a;

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
        // Must be <= esp-radio's `ESP_RADIO_CONFIG_WIFI_MTU` (1492) so
        // `WifiTxToken::consume_token` stack buffer `[u8; 1492]` never
        // panics on slice indexing `&mut buffer[..len]`.
        caps.max_transmission_unit = 1480;
        caps.max_burst_size = Some(12);
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
    use core::pin::Pin;
    use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};
    const VTABLE: RawWakerVTable =
        RawWakerVTable::new(|p| RawWaker::new(p, &VTABLE), |_| {}, |_| {}, |_| {});
    // JUSTIFICATION: Constructing synchronous no-op waker for block_on polling.
    // SAFETY: VTABLE is a valid static RawWakerVTable.
    let waker = unsafe { Waker::from_raw(RawWaker::new(core::ptr::null(), &VTABLE)) };
    let mut cx = Context::from_waker(&waker);
    // JUSTIFICATION: Stack pinning future for synchronous polling loop.
    // SAFETY: `fut` is not moved before it is dropped.
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
        core::hint::spin_loop();
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
            // Target Hardware Address (00:00:00:00:00:00) +
            // Target Protocol Address (target_ip)
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
const TCP_BATCH_SEGMENTS: usize = 64;

#[cfg(target_arch = "xtensa")]
const TCP_MAX_INFLIGHT_BYTES: usize = TCP_BATCH_SEGMENTS * TCP_MSS_BYTES;

#[cfg(target_arch = "xtensa")]
const P81_PIPELINE_LOW_WATER_BYTES: usize = 45_632;

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
    let queued = sock.send_queue();
    if queued < TCP_MAX_INFLIGHT_BYTES {
        let max_send = TCP_MAX_INFLIGHT_BYTES - queued;
        let chunk_end = (*tx_off + max_send).min(limit);
        if chunk_end > *tx_off {
            let seg = &buf[*tx_off..chunk_end];
            if let Ok(sent) = sock.send_slice(seg) {
                if sent > 0 && sent < seg.len() {
                    FW_STATS.inc_send_partial();
                }
                *tx_off += sent;
            }
        }
    }
}

#[cfg(target_arch = "xtensa")]
fn enqueue_p81_zero_copy(
    sock: &mut smoltcp::socket::tcp::Socket<'_>,
    hdr: &[u8],
    jpeg: &[u8],
    tx_off: &mut usize,
) {
    let hdr_end = hdr.len();
    let total_len = hdr_end.saturating_add(jpeg.len());
    while *tx_off < total_len && sock.can_send() {
        let queued = sock.send_queue();
        if queued >= TCP_MAX_INFLIGHT_BYTES {
            break;
        }
        let budget = TCP_MAX_INFLIGHT_BYTES - queued;
        let seg = if *tx_off < hdr_end {
            let rem = &hdr[*tx_off..hdr_end];
            &rem[..rem.len().min(budget)]
        } else {
            let off = *tx_off - hdr_end;
            let rem = &jpeg[off..];
            &rem[..rem.len().min(budget)]
        };
        if seg.is_empty() {
            break;
        }
        match sock.send_slice(seg) {
            Ok(0) | Err(_) => break,
            Ok(sent) => {
                if sent < seg.len() {
                    FW_STATS.inc_send_partial();
                }
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
    let t0 = ccount();
    for _ in 0..16u8 {
        let now = smol_now();
        let _ = iface.poll(now, net_device, sockets);
        if iface.poll_delay(now, sockets) != Some(smoltcp::time::Duration::ZERO) {
            break;
        }
    }
    FW_STATS.add_c0_net_cycles(ccount().wrapping_sub(t0));
}

#[cfg(target_arch = "xtensa")]
#[esp_hal::ram]
fn hold_psram_cs_high_safe() {
    const GPIO_OUT_W1TS_REG: *mut u32 = 0x3FF4_4008 as *mut u32;
    const GPIO_ENABLE_W1TS_REG: *mut u32 = 0x3FF4_4024 as *mut u32;
    const GPIO_FUNC16_OUT_SEL_CFG_REG: *mut u32 = 0x3FF4_4570 as *mut u32;
    const IO_MUX_GPIO16_REG: *mut u32 = 0x3FF4_904C as *mut u32;
    const SIG_GPIO_OUT_IDX: u32 = 0x100;
    // MCU_SEL = Function 2 (GPIO = 2 << 12), FUN_IE = 1 << 9,
    // FUN_WPU = 1 << 8, FUN_DRV = 2 << 10
    const IO_MUX_GPIO16_WPU_GPIO: u32 = (2 << 12) | (2 << 10) | (1 << 9) | (1 << 8);

    // JUSTIFICATION: Direct GPIO MMIO configuration to hold PSRAM CS# HIGH.
    // SAFETY: GPIO MMIO registers are valid addresses in peripheral space.
    unsafe {
        core::ptr::write_volatile(GPIO_OUT_W1TS_REG, 1 << 16);
        core::ptr::write_volatile(GPIO_FUNC16_OUT_SEL_CFG_REG, SIG_GPIO_OUT_IDX);
        core::ptr::write_volatile(IO_MUX_GPIO16_REG, IO_MUX_GPIO16_WPU_GPIO);
        core::ptr::write_volatile(GPIO_ENABLE_W1TS_REG, 1 << 16);
    }
}

/// Sends an 8-bit PSRAM command (`0xF5` in 4-bit QPI mode when
/// `qpi_mode == true`, or `0x35` in 1-bit SPI mode when `qpi_mode == false`)
/// over the hardware `SPI1` controller
/// using `SPI1_CS1` (`GPIO 16`) and `SPICLK` (`GPIO 17`), ported directly from
/// `ESP-IDF v5.5.4` `esp_psram_impl_quad.c:250-535`.
///
/// Never disconnects or bit-bangs `SPI0`/`SPI1` data pads (`SD0..SD3`).
#[cfg(target_arch = "xtensa")]
#[esp_hal::ram]
fn psram_spi1_cmd(qpi_mode: bool, cmd_byte: u8) -> bool {
    const DPORT_HOST_INF_SEL_REG: *mut u32 = 0x3FF0_00BC as *mut u32;
    const SPI0_PIN_REG: *mut u32 = 0x3FF4_3034 as *mut u32;
    const SPI0_EXT2_REG: *const u32 = 0x3FF4_30F8 as *const u32;
    const SPI1_CMD_REG: *mut u32 = 0x3FF4_2000 as *mut u32;
    const SPI1_ADDR_REG: *mut u32 = 0x3FF4_2004 as *mut u32;
    const SPI1_CTRL_REG: *mut u32 = 0x3FF4_2008 as *mut u32;
    const SPI1_CLOCK_REG: *mut u32 = 0x3FF4_2018 as *mut u32;
    const SPI1_USER_REG: *mut u32 = 0x3FF4_201C as *mut u32;
    const SPI1_USER1_REG: *mut u32 = 0x3FF4_2020 as *mut u32;
    const SPI1_USER2_REG: *mut u32 = 0x3FF4_2024 as *mut u32;
    const SPI1_MOSI_DLEN_REG: *mut u32 = 0x3FF4_2028 as *mut u32;
    const SPI1_MISO_DLEN_REG: *mut u32 = 0x3FF4_202C as *mut u32;
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

    // JUSTIFICATION: Hardware SPI1 peripheral registers for PSRAM QPI command.
    // SAFETY: Disables interrupts during SPI1 bus transaction and restores PS.
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
        let saved_clock = core::ptr::read_volatile(SPI1_CLOCK_REG);
        let saved_user = core::ptr::read_volatile(SPI1_USER_REG);
        let saved_user1 = core::ptr::read_volatile(SPI1_USER1_REG);
        let saved_user2 = core::ptr::read_volatile(SPI1_USER2_REG);
        let saved_mosi_dlen = core::ptr::read_volatile(SPI1_MOSI_DLEN_REG);
        let saved_miso_dlen = core::ptr::read_volatile(SPI1_MISO_DLEN_REG);
        let saved_w0 = core::ptr::read_volatile(SPI1_W0_REG);
        let saved_pin = core::ptr::read_volatile(SPI1_PIN_REG);
        let saved_ctrl = core::ptr::read_volatile(SPI1_CTRL_REG);
        let saved_addr = core::ptr::read_volatile(SPI1_ADDR_REG);
        let saved_outsel16 = core::ptr::read_volatile(GPIO_FUNC16_OUT_SEL_CFG_REG);
        let saved_iomux16 = core::ptr::read_volatile(IO_MUX_GPIO16_REG);
        let saved_outsel17 = core::ptr::read_volatile(GPIO_FUNC17_OUT_SEL_CFG_REG);
        let saved_iomux17 = core::ptr::read_volatile(IO_MUX_GPIO17_REG);

        if (saved_clock & (1u32 << 31)) != 0 {
            core::ptr::write_volatile(SPI1_CLOCK_REG, 0x0000_1001);
        }

        // Temporarily disable CS1 on SPI0 (SPI_CS1_DIS = 1 << 1) before
        // routing GPIO 16 to SPI1 SPICS1_OUT_IDX (6)
        core::ptr::write_volatile(SPI0_PIN_REG, saved_spi0_pin | (1u32 << 1));

        // Route GPIO 16 -> SPICS1_OUT_IDX (6) with pull-up (0x2B00)
        // and GPIO 17 -> SPICLK_OUT_IDX (0)
        core::ptr::write_volatile(GPIO_FUNC16_OUT_SEL_CFG_REG, SPICS1_OUT_IDX);
        core::ptr::write_volatile(IO_MUX_GPIO16_REG, 0x2B00);
        core::ptr::write_volatile(GPIO_FUNC17_OUT_SEL_CFG_REG, SPICLK_OUT_IDX);
        core::ptr::write_volatile(IO_MUX_GPIO17_REG, 0x2800);
        core::ptr::write_volatile(GPIO_ENABLE_W1TS_REG, (1u32 << 16) | (1u32 << 17));

        // Select CS1 (bit 1 = 0) and disable CS0 (bit 0 = 1) on SPI1
        core::ptr::write_volatile(SPI1_PIN_REG, (saved_pin | 1u32) & !(1u32 << 1));

        if qpi_mode {
            // Matches PSRAM_CLK_MODE_NORM 8-bit command transmission:
            // 8-bit cmd_byte payload in SPI1_W0_REG via USR_MOSI +
            // FWRITE_QIO (2 QPI clocks).
            core::ptr::write_volatile(
                SPI1_CTRL_REG,
                (saved_ctrl & !SPI_FREAD_MODE_MASK) | SPI_FREAD_QIO | SPI_WP_REG,
            );
            core::ptr::write_volatile(SPI1_USER1_REG, 0);
            core::ptr::write_volatile(SPI1_USER2_REG, 0);
            core::ptr::write_volatile(SPI1_W0_REG, cmd_byte as u32);
            core::ptr::write_volatile(SPI1_MOSI_DLEN_REG, 7u32);
            core::ptr::write_volatile(SPI1_MISO_DLEN_REG, 0);
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
            core::ptr::write_volatile(SPI1_USER2_REG, 0);
            core::ptr::write_volatile(SPI1_MOSI_DLEN_REG, 0);
            core::ptr::write_volatile(SPI1_MISO_DLEN_REG, 0);
            core::ptr::write_volatile(SPI1_ADDR_REG, (cmd_byte as u32) << 24);
            core::ptr::write_volatile(SPI1_USER1_REG, (7u32 & 0x3F) << 26);
            core::ptr::write_volatile(SPI1_USER_REG, SPI_USR_ADDR | SPI_CS_SETUP | SPI_CS_HOLD);
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
        core::ptr::write_volatile(SPI1_CLOCK_REG, saved_clock);
        core::ptr::write_volatile(SPI1_USER_REG, saved_user);
        core::ptr::write_volatile(SPI1_USER1_REG, saved_user1);
        core::ptr::write_volatile(SPI1_USER2_REG, saved_user2);
        core::ptr::write_volatile(SPI1_MOSI_DLEN_REG, saved_mosi_dlen);
        core::ptr::write_volatile(SPI1_MISO_DLEN_REG, saved_miso_dlen);
        core::ptr::write_volatile(SPI1_W0_REG, saved_w0);
        // Enforce CS1_DIS = 1 (bit 1) and CS0_DIS = 0 (bit 0) on
        // restore (esp_psram_impl_quad.c:327-328)
        core::ptr::write_volatile(SPI1_PIN_REG, (saved_pin | (1u32 << 1)) & !1u32);
        core::ptr::write_volatile(SPI1_ADDR_REG, saved_addr);
        core::ptr::write_volatile(GPIO_FUNC16_OUT_SEL_CFG_REG, saved_outsel16);
        core::ptr::write_volatile(IO_MUX_GPIO16_REG, saved_iomux16 | (1u32 << 8));
        core::ptr::write_volatile(GPIO_FUNC17_OUT_SEL_CFG_REG, saved_outsel17);
        core::ptr::write_volatile(IO_MUX_GPIO17_REG, saved_iomux17);
        core::ptr::write_volatile(SPI0_PIN_REG, saved_spi0_pin);

        core::arch::asm!("wsr.ps {0}; rsync", in(reg) saved_ps);
        wait1 < 100_000
    }
}

/// Issues a 1-bit SPI `0x9F` (`PSRAM_DEVICE_ID`) dummy read over `SPI1`
/// (`esp_psram_impl_quad.c:467-495, 983-988`:
/// `/* 16Mbit psram needs a dummy read to get id when cold start */`).
#[cfg(target_arch = "xtensa")]
#[esp_hal::ram]
fn psram_spi1_dummy_read_id() {
    const DPORT_HOST_INF_SEL_REG: *mut u32 = 0x3FF0_00BC as *mut u32;
    const SPI0_PIN_REG: *mut u32 = 0x3FF4_3034 as *mut u32;
    const SPI0_EXT2_REG: *const u32 = 0x3FF4_30F8 as *const u32;
    const SPI1_CMD_REG: *mut u32 = 0x3FF4_2000 as *mut u32;
    const SPI1_ADDR_REG: *mut u32 = 0x3FF4_2004 as *mut u32;
    const SPI1_CTRL_REG: *mut u32 = 0x3FF4_2008 as *mut u32;
    const SPI1_CLOCK_REG: *mut u32 = 0x3FF4_2018 as *mut u32;
    const SPI1_USER_REG: *mut u32 = 0x3FF4_201C as *mut u32;
    const SPI1_USER1_REG: *mut u32 = 0x3FF4_2020 as *mut u32;
    const SPI1_USER2_REG: *mut u32 = 0x3FF4_2024 as *mut u32;
    const SPI1_MOSI_DLEN_REG: *mut u32 = 0x3FF4_2028 as *mut u32;
    const SPI1_MISO_DLEN_REG: *mut u32 = 0x3FF4_202C as *mut u32;
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

    const SPI_FREAD_DUAL: u32 = 1u32 << 14;
    const SPI_FREAD_QUAD: u32 = 1u32 << 20;
    const SPI_WP_REG: u32 = 1u32 << 21;
    const SPI_FREAD_DIO: u32 = 1u32 << 23;
    const SPI_FREAD_QIO: u32 = 1u32 << 24;
    const SPI_FREAD_MODE_MASK: u32 =
        SPI_FREAD_DUAL | SPI_FREAD_QUAD | SPI_FREAD_DIO | SPI_FREAD_QIO;

    const SPI_CS_HOLD: u32 = 1u32 << 4;
    const SPI_CS_SETUP: u32 = 1u32 << 5;
    const SPI_USR_MISO: u32 = 1u32 << 28;
    const SPI_USR_ADDR: u32 = 1u32 << 30;

    // JUSTIFICATION: Hardware SPI1 peripheral registers for PSRAM 0x9F ID read.
    // SAFETY: Disables interrupts during SPI1 bus transaction and restores PS.
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
        let saved_clock = core::ptr::read_volatile(SPI1_CLOCK_REG);
        let saved_user = core::ptr::read_volatile(SPI1_USER_REG);
        let saved_user1 = core::ptr::read_volatile(SPI1_USER1_REG);
        let saved_user2 = core::ptr::read_volatile(SPI1_USER2_REG);
        let saved_mosi_dlen = core::ptr::read_volatile(SPI1_MOSI_DLEN_REG);
        let saved_miso_dlen = core::ptr::read_volatile(SPI1_MISO_DLEN_REG);
        let saved_w0 = core::ptr::read_volatile(SPI1_W0_REG);
        let saved_w1 = core::ptr::read_volatile(SPI1_W0_REG.wrapping_add(1));
        let saved_pin = core::ptr::read_volatile(SPI1_PIN_REG);
        let saved_ctrl = core::ptr::read_volatile(SPI1_CTRL_REG);
        let saved_addr = core::ptr::read_volatile(SPI1_ADDR_REG);
        let saved_outsel16 = core::ptr::read_volatile(GPIO_FUNC16_OUT_SEL_CFG_REG);
        let saved_iomux16 = core::ptr::read_volatile(IO_MUX_GPIO16_REG);
        let saved_outsel17 = core::ptr::read_volatile(GPIO_FUNC17_OUT_SEL_CFG_REG);
        let saved_iomux17 = core::ptr::read_volatile(IO_MUX_GPIO17_REG);

        if (saved_clock & (1u32 << 31)) != 0 {
            core::ptr::write_volatile(SPI1_CLOCK_REG, 0x0000_1001);
        }

        core::ptr::write_volatile(SPI0_PIN_REG, saved_spi0_pin | (1u32 << 1));

        core::ptr::write_volatile(GPIO_FUNC16_OUT_SEL_CFG_REG, SPICS1_OUT_IDX);
        core::ptr::write_volatile(IO_MUX_GPIO16_REG, 0x2B00);
        core::ptr::write_volatile(GPIO_FUNC17_OUT_SEL_CFG_REG, SPICLK_OUT_IDX);
        core::ptr::write_volatile(IO_MUX_GPIO17_REG, 0x2800);
        core::ptr::write_volatile(GPIO_ENABLE_W1TS_REG, (1u32 << 16) | (1u32 << 17));

        core::ptr::write_volatile(SPI1_PIN_REG, (saved_pin | 1u32) & !(1u32 << 1));

        core::ptr::write_volatile(
            SPI1_CTRL_REG,
            (saved_ctrl & !SPI_FREAD_MODE_MASK) | SPI_WP_REG,
        );
        core::ptr::write_volatile(SPI1_USER2_REG, 0);
        core::ptr::write_volatile(SPI1_ADDR_REG, 0x9Fu32 << 24);
        core::ptr::write_volatile(SPI1_USER1_REG, (31u32 & 0x3F) << 26);
        core::ptr::write_volatile(SPI1_MOSI_DLEN_REG, 0);
        core::ptr::write_volatile(SPI1_MISO_DLEN_REG, 63);
        core::ptr::write_volatile(
            SPI1_USER_REG,
            SPI_USR_ADDR | SPI_USR_MISO | SPI_CS_SETUP | SPI_CS_HOLD,
        );

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
        core::ptr::write_volatile(SPI1_CLOCK_REG, saved_clock);
        core::ptr::write_volatile(SPI1_USER_REG, saved_user);
        core::ptr::write_volatile(SPI1_USER1_REG, saved_user1);
        core::ptr::write_volatile(SPI1_USER2_REG, saved_user2);
        core::ptr::write_volatile(SPI1_MOSI_DLEN_REG, saved_mosi_dlen);
        core::ptr::write_volatile(SPI1_MISO_DLEN_REG, saved_miso_dlen);
        core::ptr::write_volatile(SPI1_W0_REG, saved_w0);
        core::ptr::write_volatile(SPI1_W0_REG.wrapping_add(1), saved_w1);
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
/// `esp-hal-1.2.2`'s `psram_enable_qio_mode_spi1` (`esp32.rs:753`) erroneously
/// sends `0x35` (`PSRAM_ENTER_QMODE`) in `PsramCmdQpi` mode instead of 1-bit
/// `PSRAM_CMD_SPI` mode (`esp_psram_impl_quad.c:519`). Calling
/// `psram_spi1_cmd(true, 0xF5)`, two `psram_spi1_dummy_read_id()` cycles, and
/// `psram_spi1_cmd(false, 0x35)` guarantees PSRAM enters QPI mode on both cold
/// `0V -> 3.3V` power-on resets and warm reboots.
#[cfg(target_arch = "xtensa")]
#[esp_hal::ram]
fn ensure_psram_qpi_mode_spi1() {
    const GPIO_FUNC16_OUT_SEL_CFG_REG: *mut u32 = 0x3FF4_4570 as *mut u32;
    const IO_MUX_GPIO16_REG: *mut u32 = 0x3FF4_904C as *mut u32;
    const SPICS1_OUT_IDX: u32 = 6;

    psram_spi1_cmd(true, 0xF5);
    psram_spi1_dummy_read_id();
    psram_spi1_dummy_read_id();
    psram_spi1_cmd(false, 0x35);
    // JUSTIFICATION: Route GPIO 16 to hardware SPI1 CS1 after mode transition.
    // SAFETY: GPIO_FUNC16_OUT_SEL_CFG_REG and IO_MUX_GPIO16_REG are valid MMIO.
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
    let _ = pause_core1_camera_producer();
    // JUSTIFICATION: Halt I2S0 DMA engine immediately before PSRAM pin release.
    // SAFETY:
    // 1. Alignment & Non-null: Direct MMIO register write via validated HAL.
    // 2. Layout: Target I2S0 peripheral registers exist on ESP32 silicon.
    // 3. Provenance: MMIO base address is mapped and valid for SoC lifecycle.
    // 4. Aliasing: Producer is parked; exclusive hardware access during reset.
    // 5. Volatile/Atomics: Peripheral registers are volatile MMIO.
    // 6. Unwind safety: C-ABI HAL function does not unwind or panic.
    unsafe {
        esp32_cam_ll::emergency_stop_i2s0_hw();
    }
    release_psram_cs_and_exit_qpi();
    // JUSTIFICATION: Mask interrupts at CPU level 15 prior to software reset.
    // SAFETY: Inline assembly modifies only processor interrupt priority.
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

/// Maximum Wi-Fi transmit power in quarter-dBm units (78 qdBm = 19.5 dBm,
/// ~89 mW).
///
/// Sets the ESP32 Wi-Fi PHY transmit power to 78 qdBm (19.5 dBm), providing
/// optimal link budget and packet throughput for MJPEG streaming under
/// noisy RF conditions.
#[cfg(target_arch = "xtensa")]
pub const WIFI_MAX_TX_POWER_QDBM: i8 = 78;

/// Builds the active, SSID-filtered scan configuration shared by boot AP
/// selection and runtime roaming scans.
///
/// Dwell time per channel is `SCAN_ACTIVE_MIN_MS..SCAN_ACTIVE_MAX_MS`.
#[cfg(target_arch = "xtensa")]
fn active_scan_config(ssid_str: &str) -> esp_radio::wifi::scan::ScanConfig {
    use esp_hal::time::Duration;
    use esp_radio::wifi::scan::{ScanConfig, ScanTypeConfig};
    use klipper_cam_fw::{SCAN_ACTIVE_MAX_MS, SCAN_ACTIVE_MIN_MS};

    let mut scan_cfg = ScanConfig::default()
        .with_max(16)
        .with_scan_type(ScanTypeConfig::Active {
            min: Duration::from_millis(SCAN_ACTIVE_MIN_MS),
            max: Duration::from_millis(SCAN_ACTIVE_MAX_MS),
        });
    if let Ok(s) = ssid_str.try_into() {
        scan_cfg = scan_cfg.with_ssid(s);
    }
    scan_cfg
}

/// Runs one active scan pass (5000 ms timeout) and records every AP whose
/// SSID equals `ssid_str` into `tracker`.
///
/// Returns `true` if the scan completed before the timeout (even if it
/// reported an error or no matching AP).
#[cfg(target_arch = "xtensa")]
fn scan_pass_into_tracker(
    wifi_controller: &mut esp_radio::wifi::WifiController<'_>,
    scan_cfg: &esp_radio::wifi::scan::ScanConfig,
    ssid_str: &str,
    tracker: &mut klipper_cam_fw::ApTracker,
) -> bool {
    let Some(res) = block_on_with_timeout(wifi_controller.scan_async(scan_cfg), 5000) else {
        return false;
    };
    if let Ok(aps) = res {
        for ap in aps {
            if ap.ssid.as_str() == ssid_str {
                tracker.record(ap.bssid, ap.channel, ap.signal_strength);
            }
        }
    }
    true
}

/// Prints one `[WIFI] Candidate` line per BSSID recorded in `tracker`.
///
/// `passes` is the number of scan passes fed into `tracker`, printed as the
/// denominator of the `seen=` hit count.
#[cfg(target_arch = "xtensa")]
fn log_ap_candidates(tracker: &klipper_cam_fw::ApTracker, passes: u8) {
    use esp_println::println;

    let max_hits = tracker.max_hits();
    for c in tracker.candidates() {
        let b = c.bssid();
        println!(
            concat!(
                "[KLIPPER-CAM-RS] [WIFI] Candidate ",
                "bssid={:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x} ",
                "ch={} eff={}dBm avg={}dBm max={}dBm seen={}/{}"
            ),
            b[0],
            b[1],
            b[2],
            b[3],
            b[4],
            b[5],
            c.channel(),
            c.effective_rssi(max_hits),
            c.rssi_avg(),
            c.rssi_max(),
            c.hits(),
            passes
        );
    }
}

/// Returns `base_sta_cfg` locked to `bssid` on `channel` (fast scan, 2
/// association retries), as used by boot selection and runtime roaming.
#[cfg(target_arch = "xtensa")]
fn locked_station_config(
    base_sta_cfg: &esp_radio::wifi::sta::StationConfig,
    bssid: [u8; 6],
    channel: u8,
) -> esp_radio::wifi::sta::StationConfig {
    use esp_radio::wifi::sta::ScanMethod;

    base_sta_cfg
        .clone()
        .with_bssid(bssid)
        .with_channel(channel)
        .with_scan_method(ScanMethod::Fast)
        .with_failure_retry_cnt(2)
}

/// Runs one active roam scan and decides whether to leave `current_bssid`.
///
/// Candidates are logged; the decision applies `ROAM_HYSTERESIS_DB` to
/// `current_rssi`.
#[cfg(target_arch = "xtensa")]
fn roam_scan_decision(
    wifi_controller: &mut esp_radio::wifi::WifiController<'_>,
    ssid_str: &str,
    current_bssid: [u8; 6],
    current_rssi: i8,
) -> klipper_cam_fw::RoamDecision {
    use klipper_cam_fw::{roam_decision, ApTracker, RoamDecision, ROAM_HYSTERESIS_DB};

    let scan_cfg = active_scan_config(ssid_str);
    let mut tracker = ApTracker::new();
    if !scan_pass_into_tracker(wifi_controller, &scan_cfg, ssid_str, &mut tracker) {
        return RoamDecision::Stay;
    }
    log_ap_candidates(&tracker, 1);
    roam_decision(current_bssid, current_rssi, &tracker, ROAM_HYSTERESIS_DB)
}

/// Re-associates with `target`: disconnect (2 s), BSSID+channel locked
/// config, `connect_async` (10 s).
///
/// On failure the unlocked `base_sta_cfg` is restored so the regular
/// reconnect path in the server loop can recover the link.
#[cfg(target_arch = "xtensa")]
fn roam_to_candidate(
    wifi_controller: &mut esp_radio::wifi::WifiController<'_>,
    base_sta_cfg: &esp_radio::wifi::sta::StationConfig,
    target: &klipper_cam_fw::ApCandidate,
) -> Option<esp_radio::wifi::sta::ConnectedInfo> {
    use esp_println::println;
    use esp_radio::wifi::Config as WifiConfig;

    let _ = block_on_with_timeout(wifi_controller.disconnect_async(), 2000);
    let locked_cfg = locked_station_config(base_sta_cfg, target.bssid(), target.channel());
    if wifi_controller
        .set_config(&WifiConfig::Station(locked_cfg))
        .is_err()
    {
        println!("[KLIPPER-CAM-RS] [ROAM] set_config(locked) failed; restoring base config");
        let _ = wifi_controller.set_config(&WifiConfig::Station(base_sta_cfg.clone()));
        return None;
    }
    let _ = wifi_controller.set_max_tx_power(WIFI_MAX_TX_POWER_QDBM);
    match block_on_with_timeout(wifi_controller.connect_async(), 10_000) {
        Some(Ok(info)) => Some(info),
        Some(Err(e)) => {
            println!("[KLIPPER-CAM-RS] [ROAM] connect_async error: {:?}", e);
            let _ = block_on_with_timeout(wifi_controller.disconnect_async(), 2000);
            let _ = wifi_controller.set_config(&WifiConfig::Station(base_sta_cfg.clone()));
            None
        }
        None => {
            println!("[KLIPPER-CAM-RS] [ROAM] connect_async timed out (10000 ms)");
            let _ = block_on_with_timeout(wifi_controller.disconnect_async(), 2000);
            let _ = wifi_controller.set_config(&WifiConfig::Station(base_sta_cfg.clone()));
            None
        }
    }
}

/// Post-association steps shared by the reconnect and roam paths: mark the
/// STA connected, disable power save, restore max TX power, arm static-IP ARP
/// confirmation if IP is not yet confirmed, and (once the IP is configured)
/// send a gratuitous ARP announce.
///
/// `announce_ip` is `Some(ip)` once the static IP is configured; `None`
/// skips the ARP announce.
#[cfg(target_arch = "xtensa")]
fn after_sta_associated(
    wifi_controller: &mut esp_radio::wifi::WifiController<'_>,
    runtime: &mut FirmwareRuntime,
    static_ip_arp_pending: &mut bool,
    net_device: &mut SmoltcpWifiDevice,
    mac: [u8; 6],
    announce_ip: Option<[u8; 4]>,
) {
    runtime.on_wifi_sta_connected();
    let _ = wifi_controller.set_power_saving(esp_radio::wifi::PowerSaveMode::None);
    let _ = wifi_controller.set_max_tx_power(WIFI_MAX_TX_POWER_QDBM);
    if let Some(ip) = announce_ip {
        if !runtime.ip_confirmed {
            *static_ip_arp_pending = true;
        }
        runtime.wifi_state = klipper_cam_fw::WifiStaState::ConnectedGotIp {
            ip: Ipv4Addr::new(ip[0], ip[1], ip[2], ip[3]),
        };
        runtime.webservers_started = true;
        send_arp_announce(net_device, mac, ip, [172, 22, 1, 1]);
    }
}

#[cfg(target_arch = "xtensa")]
#[inline(never)]
fn connect_strongest_ap(
    wifi_controller: &mut esp_radio::wifi::WifiController<'_>,
    base_sta_cfg: &esp_radio::wifi::sta::StationConfig,
    ssid_str: &str,
    timeout_ms: u64,
) -> Option<esp_radio::wifi::sta::ConnectedInfo> {
    use esp_println::println;
    use esp_radio::wifi::{Config as WifiConfig, PowerSaveMode};
    use klipper_cam_fw::{ApTracker, SCAN_PASSES};

    let wait_ms = timeout_ms.max(15_000);
    let _ = wifi_controller.set_power_saving(PowerSaveMode::None);
    let _ = wifi_controller.set_max_tx_power(WIFI_MAX_TX_POWER_QDBM);

    // 1. SCAN_PASSES active scans (SCAN_ACTIVE_MIN_MS..SCAN_ACTIVE_MAX_MS
    //    dwell per channel, SSID-filtered) aggregated per BSSID, so the AP
    //    choice uses averaged RSSI rather than one noisy sample.
    let scan_cfg = active_scan_config(ssid_str);
    let mut tracker = ApTracker::new();
    let mut scan_completed = false;
    for _ in 0..SCAN_PASSES {
        if scan_pass_into_tracker(wifi_controller, &scan_cfg, ssid_str, &mut tracker) {
            scan_completed = true;
        }
    }
    log_ap_candidates(&tracker, SCAN_PASSES);
    if scan_completed {
        if let Some(best) = tracker.best() {
            let target_ch = best.channel();
            let target_bssid = best.bssid();
            println!(
                concat!(
                    "[KLIPPER-CAM-RS] [WIFI] Strongest '{}' AP: ch={} ",
                    "rssi={}dBm bssid={:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}"
                ),
                ssid_str,
                target_ch,
                best.rssi_avg(),
                target_bssid[0],
                target_bssid[1],
                target_bssid[2],
                target_bssid[3],
                target_bssid[4],
                target_bssid[5]
            );
            let locked_cfg = locked_station_config(base_sta_cfg, target_bssid, target_ch);
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
                        println!(concat!(
                            "[KLIPPER-CAM-RS] [WIFI] BSSID-locked ",
                            "connect_async timed out (10000 ms)"
                        ));
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

    // 2. Unlocked fallback (or primary path if scan found no target): full
    //    15,000 ms timeout across all channels with WIFI_CONNECT_AP_BY_SIGNAL
    //    and failure_retry_cnt(2).
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
    triple_fb: &mut TripleFrameBuffer<'_>,
    camera: &mut CameraDriver<'static>,
    current_fs: FrameSize,
    poll_net: &mut dyn FnMut(),
) -> Result<usize, (u32, usize, u8, u8, u8, u32)> {
    const GPIO_IN_REG: *const u32 = 0x3FF4_403C as *const u32;
    const I2S_INT_RAW_REG: *const u32 = 0x3FF4_F00C as *const u32;
    const I2S_INT_CLR_REG: *mut u32 = 0x3FF4_F018 as *mut u32;
    const I2S_IN_SUC_EOF_BIT: u32 = 1 << 9;

    feed_rtc_wdt();
    // JUSTIFICATION: Static DMA descriptor ring and buffer for I2S0 DMA.
    // SAFETY: Exclusively accessed from single-threaded capture routine when
    // Core 1 producer is inactive.
    let (desc_slice, dma_slice) = unsafe {
        (
            &mut *core::ptr::addr_of_mut!(I2S_DMA_DESC),
            &mut (*core::ptr::addr_of_mut!(I2S_DMA_BUF)).0,
        )
    };
    if build_circular_ring(desc_slice, dma_slice, 2048).is_err() {
        FW_STATS.inc_ring_build_failures();
    }

    let slot_cap = triple_fb.write_slot_mut().len();
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
        // JUSTIFICATION: Direct GPIO register read to sample camera VSYNC pin.
        // SAFETY: GPIO_IN_REG is a valid MMIO register address.
        ((unsafe { core::ptr::read_volatile(GPIO_IN_REG) } >> 25) & 1) != 0
    };

    // Service network once prior to VSYNC edge alignment so in-flight
    // packets are flushed
    poll_net();

    let first_desc_addr = (core::ptr::from_ref(&desc_slice[0]) as usize as u32) & 0x000F_FFFF;
    let mut eof_cnt = 0u32;
    let mut fb_len = 0usize;
    let mut last_int_raw = 0u32;

    for sync_pass in 0..2u8 {
        feed_rtc_wdt();
        let mut spin = 0u32;
        if sync_pass == 0 {
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
            if build_circular_ring(desc_slice, dma_slice, 2048).is_err() {
                FW_STATS.inc_ring_build_failures();
            }
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
            let write_slot = triple_fb.write_slot_mut();

            'dma_loop: while now_millis().saturating_sub(dma_start_ms) < 210 {
                if read_vsync() {
                    seen_vsync_high = true;
                } else if !seen_vsync_high && eof_cnt == 0 {
                    blank_spin = blank_spin.wrapping_add(1);
                    if (blank_spin & 0x3F) == 0 {
                        feed_rtc_wdt();
                        poll_net();
                    }
                }

                // JUSTIFICATION: Direct I2S0 interrupt register poll for DMA completion.
                // SAFETY: I2S_INT_RAW_REG is a valid MMIO register address.
                let int_raw = unsafe { core::ptr::read_volatile(I2S_INT_RAW_REG) };
                last_int_raw |= int_raw;

                if (int_raw & I2S_IN_SUC_EOF_BIT) != 0 {
                    // JUSTIFICATION: Clear I2S0 DMA EOF interrupt status bit in hardware.
                    // SAFETY: I2S_INT_CLR_REG is a valid MMIO register address.
                    unsafe {
                        core::ptr::write_volatile(I2S_INT_CLR_REG, I2S_IN_SUC_EOF_BIT);
                        core::arch::asm!("memw");
                    }
                    core::sync::atomic::compiler_fence(Ordering::Acquire);
                    feed_rtc_wdt();
                    last_eof_ms = now_millis();

                    for drained in 0..8u8 {
                        let half_idx = (eof_cnt as usize) & 7;
                        if drained > 0 && desc_slice[2 * half_idx + 1].owner() {
                            break;
                        }

                        let half_offset = half_idx * 4096;
                        if fb_len + 1024 <= write_slot.len() {
                            let _ = i2s_cam.memcpy_filtered(
                                &mut write_slot[fb_len..fb_len + 1024],
                                &dma_slice[half_offset..half_offset + 4096],
                            );
                            let flags = Lldesc::pack_flags(2048, 0, 0, false, false, true);
                            // JUSTIFICATION: Write DMA descriptor ownership flags in ring.
                            // SAFETY: half_idx is masked & 7, indexing within desc_slice.
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
                                && verify_jpeg_soi(&write_slot[..256]).is_none()
                            {
                                let _ = i2s_cam.stop();
                                missed_soi_on_fast_pass = true;
                                break 'dma_loop;
                            }
                            fb_len += 1024;
                            eof_cnt += 1;
                            if (eof_cnt & 7) == 0 {
                                feed_rtc_wdt();
                                poll_net();
                            }
                        } else {
                            let _ = i2s_cam.stop();
                            break 'dma_loop;
                        }
                    }
                    continue;
                }

                if eof_cnt > 0 {
                    let vsync_ended = seen_vsync_high
                        && !read_vsync()
                        && !read_vsync()
                        && !read_vsync()
                        && !read_vsync();
                    if vsync_ended || now_millis().saturating_sub(last_eof_ms) >= 25 {
                        let _ = i2s_cam.stop();
                        // JUSTIFICATION: Ensure memory writes complete after stopping camera.
                        // SAFETY: memw instruction enforces write ordering on Xtensa.
                        unsafe {
                            core::arch::asm!("memw");
                        }
                        core::sync::atomic::compiler_fence(Ordering::Acquire);
                        for _ in 0..8u8 {
                            let half_idx = (eof_cnt as usize) & 7;
                            if !desc_slice[half_idx * 2].owner()
                                && fb_len + 1024 <= write_slot.len()
                            {
                                let half_offset = half_idx * 4096;
                                let _ = i2s_cam.memcpy_filtered(
                                    &mut write_slot[fb_len..fb_len + 1024],
                                    &dma_slice[half_offset..half_offset + 4096],
                                );
                                fb_len += 1024;
                                eof_cnt += 1;
                            } else {
                                break;
                            }
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
        let write_slot = triple_fb.write_slot_mut();
        if fb_len > 0 {
            head_bytes[0] = write_slot[0];
        }
        if fb_len > 1 {
            head_bytes[1] = write_slot[1];
        }
        if fb_len > 2 {
            head_bytes[2] = write_slot[2];
        }
        if fb_len >= 64 {
            if let Some(soi_pos) = verify_jpeg_soi(&write_slot[..fb_len.min(256)]) {
                let slice_from_soi = &write_slot[soi_pos..fb_len];
                if let Some(eoi_rel) = verify_jpeg_eoi(slice_from_soi, false) {
                    let exact_len = eoi_rel + 2;
                    if exact_len >= 64 {
                        if soi_pos > 0 {
                            write_slot.copy_within(soi_pos..soi_pos + exact_len, 0);
                        }
                        commit_len = Some(exact_len);
                    }
                }
            }
        }
    }

    if let Some(exact_len) = commit_len {
        if triple_fb.commit_write_slot(exact_len).is_ok() {
            let _ = triple_fb.promote_ready_to_active();
            let fresh_slice = active_triple_static_slice(triple_fb);
            camera.set_frame(fresh_slice, current_fs);
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
fn active_triple_static_slice(tfb: &TripleFrameBuffer<'_>) -> &'static [u8] {
    let active = tfb.active_frame();
    // JUSTIFICATION: Extend active read-slot slice lifetime to 'static for
    // zero-copy HTTP response framing while TripleFrameBuffer pins read_idx.
    // SAFETY:
    // 1. Alignment & Non-null: `active.as_ptr()` comes from a valid PSRAM `&[u8]`.
    // 2. Layout: `[u8]` has 1-byte alignment and initialized byte contents.
    // 3. Provenance & Lifetime: Backing PSRAM slots are carved once at boot and
    //    live for `'static`.
    // 4. Aliasing: `read_idx` is disjoint from `ready_idx` and `write_idx`, so
    //    `commit_write_slot` never mutates `slots[read_idx]`.
    // 5. Synchronization: Accessed exclusively on Core 0.
    // 6. Unwind safety: Pure slice reconstruction cannot panic or unwind.
    unsafe { core::slice::from_raw_parts(active.as_ptr(), active.len()) }
}

#[cfg(target_arch = "xtensa")]
#[inline]
fn latest_snapshot_static_slice(tfb: &TripleFrameBuffer<'_>) -> &'static [u8] {
    let snap = tfb.latest_snapshot_frame();
    // JUSTIFICATION: Extend latest committed snapshot slice lifetime to 'static
    // so Port 80 `/snapshot` copies the newest frame synchronously.
    // SAFETY:
    // 1. Alignment & Non-null: `snap.as_ptr()` comes from a valid PSRAM `&[u8]`.
    // 2. Layout: `[u8]` has 1-byte alignment and initialized JPEG bytes.
    // 3. Provenance & Lifetime: Backing PSRAM slots live for `'static`, and
    //    `/snapshot` copies all bytes into `p80_resp_bufs` before returning.
    // 4. Aliasing: `ready_idx` and `read_idx` are disjoint from `write_idx`,
    //    and Core 0 does not run SPSC drain during `handle_raw_request`.
    // 5. Synchronization: Accessed exclusively on Core 0.
    // 6. Unwind safety: Pure slice reconstruction cannot panic or unwind.
    unsafe { core::slice::from_raw_parts(snap.as_ptr(), snap.len()) }
}

#[cfg(target_arch = "xtensa")]
#[esp_hal::ram]
fn core1_camera_producer_loop() -> ! {
    const GPIO_IN_REG: *const u32 = 0x3FF4_403C as *const u32;
    const I2S_INT_RAW_REG: *const u32 = 0x3FF4_F00C as *const u32;
    const I2S_INT_CLR_REG: *mut u32 = 0x3FF4_F018 as *mut u32;
    const I2S_IN_SUC_EOF_BIT: u32 = 1 << 9;

    // JUSTIFICATION: Core 1 (`APP_CPU`) takes exclusive ownership of `HW_I2S_CAM`
    // in internal DRAM once started by Core 0.
    // SAFETY:
    // 1. Alignment & Non-null: `HW_I2S_CAM` is a valid aligned static in DRAM.
    // 2. Layout: Initialized to `Some(hw_i2s_cam)` prior to starting Core 1.
    // 3. Provenance: Static lives for `'static`.
    // 4. Aliasing: Core 0 never accesses `HW_I2S_CAM` while `CORE1_ACTIVE` is true.
    // 5. Synchronization: `CORE1_ACTIVE` Release/Acquire orders handoff.
    // 6. Unwind safety: Diverging loop in `no_std` firmware.
    let Some(i2s_cam) = (unsafe { (*core::ptr::addr_of_mut!(HW_I2S_CAM)).as_mut() }) else {
        loop {
            core::hint::spin_loop();
        }
    };

    // JUSTIFICATION: Core 1 exclusively owns static I2S0 DMA descriptor ring
    // and 32 KB internal SRAM DMA buffer.
    // SAFETY:
    // 1. Alignment & Non-null: `I2S_DMA_DESC` and `I2S_DMA_BUF` are 4-byte
    //    aligned internal DRAM statics.
    // 2. Layout: `Lldesc` is `#[repr(C, align(4))]`; `AlignedDmaBuf` is `align(4)`.
    // 3. Provenance: Statics remain valid for `'static`.
    // 4. Aliasing: Core 0 never touches `I2S_DMA_DESC` or `I2S_DMA_BUF` while
    //    `CORE1_ACTIVE` is true.
    // 5. Synchronization: Handoff ordered before `start_app_core`.
    // 6. Unwind safety: No panicking paths.
    let (desc_slice, dma_slice) = unsafe {
        (
            &mut *core::ptr::addr_of_mut!(I2S_DMA_DESC),
            &mut (*core::ptr::addr_of_mut!(I2S_DMA_BUF)).0,
        )
    };

    dma_slice.fill(0);
    if build_circular_ring(desc_slice, dma_slice, 2048).is_err() {
        FW_STATS.inc_ring_build_failures();
    }

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
        image_size: 128 * 1024,
    };
    let first_desc_addr = (core::ptr::from_ref(&desc_slice[0]) as usize as u32) & 0x000F_FFFF;

    let read_vsync = || -> bool {
        // JUSTIFICATION: Poll camera VSYNC level directly from GPIO_IN_REG MMIO.
        // SAFETY:
        // 1. Alignment & Non-null: `0x3FF4_403C` is 4-byte aligned MMIO.
        // 2. Layout: 32-bit GPIO input status register.
        // 3. Provenance: ESP32 DPORT/GPIO peripheral address space.
        // 4. Aliasing: Read-only volatile MMIO load.
        // 5. Volatile: `read_volatile` prevents loop hoisting.
        // 6. Unwind safety: Cannot panic.
        ((unsafe { core::ptr::read_volatile(GPIO_IN_REG) } >> 25) & 1) != 0
    };

    let push_spsc_bounded = |chunk: &[u8], flags: u8, max_spins: u32| -> bool {
        for _ in 0..max_spins {
            // JUSTIFICATION: Core 1 is the sole SPSC producer on `CORE1_SPSC`.
            // SAFETY:
            // 1. Alignment & Non-null: `CORE1_SPSC` slots are 4-byte aligned in `.bss`.
            // 2. Layout: `chunk.len() <= JPEG_SPSC_CHUNK_BYTES` checked inside.
            // 3. Provenance: Static `CORE1_SPSC` lives in internal DRAM for `'static`.
            // 4. Aliasing: Single producer (Core 1) / single consumer (Core 0).
            // 5. Synchronization: Acquire/Release atomics on `head` and `tail`.
            // 6. Unwind safety: Bounded slice copy, zero panics.
            if unsafe { CORE1_SPSC.push_chunk(chunk, flags) } {
                return true;
            }
            if CORE1_PAUSE_REQ.load(Ordering::Relaxed) {
                return false;
            }
            core::hint::spin_loop();
        }
        false
    };

    let mut unpack_buf = [0u8; JPEG_SPSC_CHUNK_BYTES];
    let mut aligned_from_eof = false;
    let mut last_vsync_cycles = 0u32;
    let rearm_flags = Lldesc::pack_flags(2048, 0, 0, false, false, true);

    loop {
        if CORE1_PAUSE_REQ.load(Ordering::Acquire) {
            if !i2s_cam.stop() {
                FW_STATS.inc_i2s_stop_timeouts();
            }
            aligned_from_eof = false;
            CORE1_IS_PAUSED.store(true, Ordering::Release);
            while CORE1_PAUSE_REQ.load(Ordering::Acquire) {
                core::hint::spin_loop();
            }
            CORE1_IS_PAUSED.store(false, Ordering::Release);
            continue;
        }

        if !aligned_from_eof || read_vsync() {
            let mut paused = false;
            if !read_vsync() {
                for spin in 0..3_000_000u32 {
                    if (spin & 0x3FF) == 0 && CORE1_PAUSE_REQ.load(Ordering::Acquire) {
                        paused = true;
                        break;
                    }
                    if read_vsync() {
                        break;
                    }
                    core::hint::spin_loop();
                }
            }
            if !paused {
                for spin in 0..4_000_000u32 {
                    if (spin & 0x3FF) == 0 && CORE1_PAUSE_REQ.load(Ordering::Acquire) {
                        paused = true;
                        break;
                    }
                    if !read_vsync() {
                        break;
                    }
                    core::hint::spin_loop();
                }
            }
            if paused {
                continue;
            }
        }

        let vsync_cycles = ccount();
        if last_vsync_cycles != 0 {
            let delta = vsync_cycles.wrapping_sub(last_vsync_cycles);
            FW_STATS.record_sensor_period_us(delta / 240);
        }
        last_vsync_cycles = vsync_cycles;

        if build_circular_ring(desc_slice, dma_slice, 2048).is_err() {
            FW_STATS.inc_ring_build_failures();
        }
        // JUSTIFICATION: Clear stale I2S0 EOF interrupt flag before arming DMA.
        // SAFETY:
        // 1. Alignment & Non-null: `0x3FF4_F018` is 4-byte aligned I2S0 MMIO.
        // 2. Layout: 32-bit write-1-to-clear interrupt register.
        // 3. Provenance: ESP32 I2S0 peripheral register block.
        // 4. Aliasing: Core 1 exclusively controls I2S0 while unpaused.
        // 5. Volatile: `write_volatile` commits hardware clear immediately.
        // 6. Unwind safety: Cannot panic.
        unsafe {
            core::ptr::write_volatile(I2S_INT_CLR_REG, I2S_IN_SUC_EOF_BIT);
        }
        if !i2s_cam.start(&dma_cfg, first_desc_addr) {
            aligned_from_eof = false;
            continue;
        }

        let mut half_idx = 0usize;
        let mut is_first_chunk = true;
        let mut total_bytes = 0usize;
        let mut aborted = false;
        let mut seen_vsync_high = false;
        let mut idle_spins = 0u32;
        let mut c1_drop_counted = false;

        'frame_poll: loop {
            if (idle_spins & 0x3FF) == 0 && CORE1_PAUSE_REQ.load(Ordering::Relaxed) {
                aborted = true;
                aligned_from_eof = false;
                break 'frame_poll;
            }

            let vsync_now = read_vsync();
            if vsync_now {
                seen_vsync_high = true;
            }

            // JUSTIFICATION: Poll raw I2S0 DMA `IN_SUC_EOF` status bit.
            // SAFETY:
            // 1. Alignment & Non-null: `0x3FF4_F00C` is 4-byte aligned MMIO.
            // 2. Layout: 32-bit status register.
            // 3. Provenance: ESP32 I2S0 peripheral register block.
            // 4. Aliasing: Read-only volatile MMIO load.
            // 5. Volatile: `read_volatile` prevents caching across poll spins.
            // 6. Unwind safety: Cannot panic.
            let int_raw = unsafe { core::ptr::read_volatile(I2S_INT_RAW_REG) };
            if (int_raw & I2S_IN_SUC_EOF_BIT) != 0 {
                // JUSTIFICATION: Clear I2S0 EOF flag and order SRAM DMA reads.
                // SAFETY:
                // 1. Alignment & Non-null: `0x3FF4_F018` is 4-byte aligned MMIO.
                // 2. Layout: 32-bit interrupt clear register.
                // 3. Provenance: ESP32 I2S0 peripheral register block.
                // 4. Aliasing: Core 1 exclusively owns I2S0.
                // 5. Volatile: `write_volatile` + `memw` orders SRAM reads.
                // 6. Unwind safety: Inline `memw` cannot fault or unwind.
                unsafe {
                    core::ptr::write_volatile(I2S_INT_CLR_REG, I2S_IN_SUC_EOF_BIT);
                    core::arch::asm!("memw");
                }
                core::sync::atomic::compiler_fence(Ordering::Acquire);
                idle_spins = 0;

                for drained in 0..8u8 {
                    if drained > 0 && desc_slice[2 * half_idx + 1].owner() {
                        break;
                    }
                    let half_off = half_idx * 4096;
                    let written = i2s_cam
                        .memcpy_filtered(&mut unpack_buf, &dma_slice[half_off..half_off + 4096]);
                    dma_slice[half_off..half_off + 4096].fill(0);

                    // JUSTIFICATION: Re-arm both 2048 B descriptors of the
                    // drained 4096 B half-buffer back to DMA ownership.
                    // SAFETY:
                    // 1. Alignment & Non-null: `desc_slice` is 4-byte aligned.
                    // 2. Layout: `half_idx < 8`, so `2 * half_idx + 1 < 16`.
                    // 3. Provenance: Static `I2S_DMA_DESC` array.
                    // 4. Aliasing: Core 1 exclusively accesses `desc_slice`.
                    // 5. Volatile: `write_volatile` ensures DMA sees `owner=1`.
                    // 6. Unwind safety: Bounds proven by `& 7` mask.
                    unsafe {
                        core::ptr::write_volatile(
                            &mut desc_slice[2 * half_idx].size_length_flags,
                            rearm_flags,
                        );
                        core::ptr::write_volatile(
                            &mut desc_slice[2 * half_idx + 1].size_length_flags,
                            rearm_flags,
                        );
                    }
                    half_idx = (half_idx + 1) & 7;

                    if is_first_chunk {
                        let scan_end = written.min(256);
                        if verify_jpeg_soi(&unpack_buf[..scan_end]).is_none() {
                            FW_STATS.inc_soi_miss();
                            FW_STATS.inc_c1_dropped();
                            c1_drop_counted = true;
                            aborted = true;
                            aligned_from_eof = false;
                            break 'frame_poll;
                        }
                    }

                    if total_bytes + written > 128 * 1024 {
                        FW_STATS.inc_overflow_aborts();
                        FW_STATS.inc_c1_dropped();
                        c1_drop_counted = true;
                        aborted = true;
                        aligned_from_eof = false;
                        break 'frame_poll;
                    }

                    let flags = if is_first_chunk {
                        is_first_chunk = false;
                        SPSC_FLAG_SOF
                    } else {
                        0
                    };

                    if !push_spsc_bounded(&unpack_buf[..written], flags, 8_000) {
                        FW_STATS.inc_ring_full_events();
                        FW_STATS.inc_c1_dropped();
                        c1_drop_counted = true;
                        aborted = true;
                        aligned_from_eof = false;
                        break 'frame_poll;
                    }
                    total_bytes += written;

                    if CORE1_PAUSE_REQ.load(Ordering::Relaxed) {
                        aborted = true;
                        aligned_from_eof = false;
                        break 'frame_poll;
                    }
                }
                continue 'frame_poll;
            }

            if !is_first_chunk {
                let vsync_ended = seen_vsync_high
                    && !vsync_now
                    && !read_vsync()
                    && !read_vsync()
                    && !read_vsync();
                if vsync_ended || idle_spins >= 600_000 {
                    let _ = i2s_cam.stop();
                    // JUSTIFICATION: Flush DMA writes to internal SRAM after
                    // stopping I2S0 at end of frame.
                    // SAFETY:
                    // 1. Alignment & Non-null: No memory operand.
                    // 2. Layout: Architectural memory barrier instruction.
                    // 3. Provenance: Valid Xtensa ISA instruction.
                    // 4. Aliasing: Synchronizes hardware DMA with Core 1 reads.
                    // 5. Volatile: `memw` drains processor/bus write buffers.
                    // 6. Unwind safety: Cannot trap or unwind.
                    unsafe {
                        core::arch::asm!("memw");
                    }
                    core::sync::atomic::compiler_fence(Ordering::Acquire);
                    aligned_from_eof = vsync_ended && !read_vsync();

                    for _ in 0..8u8 {
                        if desc_slice[2 * half_idx + 1].owner() {
                            break;
                        }
                        let half_off = half_idx * 4096;
                        let written = i2s_cam.memcpy_filtered(
                            &mut unpack_buf,
                            &dma_slice[half_off..half_off + 4096],
                        );
                        dma_slice[half_off..half_off + 4096].fill(0);
                        half_idx = (half_idx + 1) & 7;
                        if total_bytes + written > 128 * 1024 {
                            FW_STATS.inc_overflow_aborts();
                            FW_STATS.inc_c1_dropped();
                            c1_drop_counted = true;
                            aborted = true;
                            aligned_from_eof = false;
                            break;
                        }
                        if !push_spsc_bounded(&unpack_buf[..written], 0, 8_000) {
                            FW_STATS.inc_ring_full_events();
                            FW_STATS.inc_c1_dropped();
                            c1_drop_counted = true;
                            aborted = true;
                            aligned_from_eof = false;
                            break;
                        }
                        total_bytes += written;
                    }

                    if !aborted && total_bytes + JPEG_SPSC_CHUNK_BYTES <= 128 * 1024 {
                        let tail_off = half_idx * 4096;
                        let written = i2s_cam.memcpy_filtered(
                            &mut unpack_buf,
                            &dma_slice[tail_off..tail_off + 4096],
                        );
                        dma_slice[tail_off..tail_off + 4096].fill(0);
                        if unpack_buf[..written].iter().any(|&b| b != 0) {
                            if !push_spsc_bounded(&unpack_buf[..written], 0, 8_000) {
                                FW_STATS.inc_ring_full_events();
                                FW_STATS.inc_c1_dropped();
                                c1_drop_counted = true;
                                aborted = true;
                                aligned_from_eof = false;
                            } else {
                                total_bytes += written;
                            }
                        }
                    }
                    break 'frame_poll;
                }
            }

            idle_spins = idle_spins.wrapping_add(1);
            if idle_spins >= 4_000_000 {
                FW_STATS.inc_c1_dropped();
                c1_drop_counted = true;
                aborted = true;
                aligned_from_eof = false;
                break 'frame_poll;
            }
            core::hint::spin_loop();
        }

        if !i2s_cam.stop() {
            FW_STATS.inc_i2s_stop_timeouts();
        }

        if aborted || is_first_chunk || total_bytes < 64 {
            if !c1_drop_counted {
                FW_STATS.inc_c1_dropped();
            }
            dma_slice.fill(0);
            if !is_first_chunk {
                let _ = push_spsc_bounded(&[], SPSC_FLAG_ABORT, 8_000);
            }
            CORE1_SPSC.record_frame_dropped();
        } else if push_spsc_bounded(&[], SPSC_FLAG_EOF, 32_000) {
            FW_STATS.inc_c1_pushed();
            FW_STATS.record_frame_bytes(total_bytes as u32);
            CORE1_SPSC.record_frame_pushed();
        } else {
            FW_STATS.inc_ring_full_events();
            FW_STATS.inc_c1_dropped();
            let _ = push_spsc_bounded(&[], SPSC_FLAG_ABORT, 8_000);
            CORE1_SPSC.record_frame_dropped();
        }
    }
}

#[cfg(target_arch = "xtensa")]
#[inline(never)]
fn poll_deferred_rollback_cancel(
    runtime: &mut FirmwareRuntime,
    flash_adapter: &mut EspFlashAdapter<'_>,
    now_ms: u64,
    last_retry_ms: &mut u64,
) {
    use esp_println::println;

    if runtime.server.is_reboot_scheduled() {
        return;
    }
    if runtime.ip_confirmed
        && runtime.first_http_served
        && !runtime.rollback_cancelled
        && (*last_retry_ms == 0 || now_ms.saturating_sub(*last_retry_ms) >= 2_000)
    {
        *last_retry_ms = now_ms;
        match runtime.try_cancel_rollback_with_flash(flash_adapter) {
            Ok(true) => {
                println!(
                    "[KLIPPER-CAM-RS] [OTA] Rollback cancelled; slot {:?} marked Valid",
                    runtime.server.ota_manager.active_slot()
                );
            }
            Ok(false) => {}
            Err(err) => {
                FW_STATS.inc_sccb_errors();
                println!(
                    "[KLIPPER-CAM-RS] [OTA] Rollback cancel failed ({:?}); will retry",
                    err
                );
            }
        }
    }
}

#[cfg(target_arch = "xtensa")]
struct PsramBuffers {
    triple_fb: TripleFrameBuffer<'static>,
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
static mut P81_HDR_BUF: [u8; 512] = [0u8; 512];

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
            // Program PixelFormat::Jpeg BEFORE set_framesize(FrameSize::Hd) so
            // set_window selects the JPEG CLKRC/PCLK dividers (`div = 12`),
            // then apply OV2640 ISP settings
            let prog_ok = ov.reset(hw_sccb).is_ok()
                && ov.select_pixformat(PixelFormat::Jpeg).is_ok()
                && ov.set_framesize(hw_sccb, FrameSize::Hd).is_ok()
                && ov.set_gainceiling(hw_sccb, GainCeiling::X2).is_ok()
                && ov.set_bpc(hw_sccb, false).is_ok()
                && ov.set_wpc(hw_sccb, true).is_ok()
                && ov.set_lenc(hw_sccb, true).is_ok()
                && ov.set_quality(hw_sccb, DEFAULT_JPEG_QUALITY as i32).is_ok()
                && ov.init_status(hw_sccb).is_ok();
            println!(
                concat!(
                    "[KLIPPER-CAM-RS] [HW-2] I2C0 SCCB (0x30) OV2640 DETECTED: ",
                    "PID=0x{:02X} VER=0x{:02X} MIDH=0x{:02X} MIDL=0x{:02X} HD_PROG={}"
                ),
                id.pid, id.ver, id.midh, id.midl, prog_ok
            );
            (prog_ok, Some(ov))
        }
        Err(_) => {
            println!(concat!(
                "[KLIPPER-CAM-RS] [HW-2] I2C0 SCCB (GPIO26/GPIO27 @ 200kHz): ",
                "Bus active (no external OV2640 ribbon sensor ACK at 0x30)"
            ));
            (false, None)
        }
    }
}

#[cfg(target_arch = "xtensa")]
#[inline(never)]
fn init_psram_and_run_hil(psram_ptr: *mut u8, psram_len: usize, xclk_hw_ok: bool) {
    use esp_println::println;
    use klipper_cam_fw::SYNTHETIC_HD_JFIF_FRAME;

    const REQUIRED_PSRAM_BYTES: usize = 3_264 * 1024;
    let mut psram_rw_ok = false;
    if !psram_ptr.is_null() && psram_len >= REQUIRED_PSRAM_BYTES {
        for attempt in 0..3u8 {
            // JUSTIFICATION: Volatile write/read to verify PSRAM memory bus.
            // SAFETY: psram_ptr is verified non-null and within PSRAM bounds.
            let ok = unsafe {
                let p32 = psram_ptr as *mut u32;
                core::ptr::write_volatile(p32, 0x55AA_55AA);
                let r1 = core::ptr::read_volatile(p32);
                core::ptr::write_volatile(p32, 0xA55A_A55A);
                let r2 = core::ptr::read_volatile(p32);
                r1 == 0x55AA_55AA && r2 == 0xA55A_A55A
            };
            if ok {
                psram_rw_ok = true;
                break;
            }
            if attempt < 2 {
                ensure_psram_qpi_mode_spi1();
                esp_hal::delay::Delay::new().delay_millis(10);
                feed_rtc_wdt();
            }
        }
    }
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

    let psram_all: &'static mut [u8];
    // JUSTIFICATION: Obtain mutable slice of mapped PSRAM memory region.
    // SAFETY: psram_ptr points to valid, allocated PSRAM heap memory.
    unsafe {
        psram_all = core::slice::from_raw_parts_mut(psram_ptr, psram_len);
    }
    let (jpeg_slot_a, rem) = psram_all.split_at_mut(128 * 1024);
    let (jpeg_slot_b, rem) = rem.split_at_mut(128 * 1024);
    let (jpeg_slot_c, rem) = rem.split_at_mut(128 * 1024);

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

    let (http80_ota_accum, psram_heap_rem) = rem.split_at_mut(1800 * 1024);
    let free_psram_bytes = psram_heap_rem.len() as u32;

    let Ok(triple_fb) = TripleFrameBuffer::new(
        jpeg_slot_a,
        jpeg_slot_b,
        jpeg_slot_c,
        &SYNTHETIC_HD_JFIF_FRAME,
    ) else {
        quiesce_psram_and_software_reset();
    };

    // JUSTIFICATION: Initialize static PSRAM_BUFFERS singleton.
    // SAFETY:
    // 1. Alignment & Non-null: addr_of_mut!(PSRAM_BUFFERS) points to static.
    // 2. Layout: Valid Option<PsramBuffers> slot in BSS.
    // 3. Provenance: PSRAM_BUFFERS lives for 'static.
    // 4. Aliasing: Unique single-threaded access during boot initialization.
    // 5. Synchronization: Called sequentially before WiFi or server starts.
    // 6. Unwind safety: Bare-metal environment with panic = abort.
    unsafe {
        *core::ptr::addr_of_mut!(PSRAM_BUFFERS) = Some(PsramBuffers {
            triple_fb,
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
        concat!(
            "[KLIPPER-CAM-RS] [HIL-2] Ov2640 1280x720 HD ",
            "(OV-105 HSIZE8=0xC8 VSIZE8=0x96 YOFFL=150): {}"
        ),
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
    // JUSTIFICATION: Initialize static HW_I2S_CAM global singleton.
    // SAFETY:
    // 1. Alignment & Non-null: addr_of_mut!(HW_I2S_CAM) points to static.
    // 2. Layout: Valid Option<Esp32I2sCamera> slot in BSS.
    // 3. Provenance: HW_I2S_CAM lives for 'static.
    // 4. Aliasing: Unique single-threaded access during boot initialization.
    // 5. Synchronization: Called sequentially before server loop starts.
    // 6. Unwind safety: Bare-metal environment with panic = abort.
    unsafe {
        *core::ptr::addr_of_mut!(HW_I2S_CAM) = Some(hw_i2s_cam);
    }
}

#[cfg(target_arch = "xtensa")]
#[inline(never)]
fn capture_boot_dma_and_init_runtime(ov2640_hw_ok: bool, bc_prev: BreadcrumbPrevSummary) -> bool {
    use esp_hal::delay::Delay;
    use esp_println::println;

    let delay = Delay::new();
    // JUSTIFICATION: Access mutable static PSRAM_BUFFERS singleton.
    // SAFETY:
    // 1. Alignment & Non-null: addr_of_mut!(PSRAM_BUFFERS) points to static.
    // 2. Layout: Valid Option<PsramBuffers> initialized prior to this call.
    // 3. Provenance: Lives for 'static.
    // 4. Aliasing: Exclusively accessed from single-threaded initialization.
    // 5. Synchronization: Single-threaded sequence on Core 0.
    // 6. Unwind safety: Bare-metal environment with panic = abort.
    let Some(psram_bufs) = (unsafe { (*core::ptr::addr_of_mut!(PSRAM_BUFFERS)).as_mut() }) else {
        quiesce_psram_and_software_reset();
    };
    // JUSTIFICATION: Access mutable static HW_I2S_CAM singleton.
    // SAFETY:
    // 1. Alignment & Non-null: addr_of_mut!(HW_I2S_CAM) points to static.
    // 2. Layout: Valid Option<Esp32I2sCamera> initialized prior to this call.
    // 3. Provenance: Lives for 'static.
    // 4. Aliasing: Exclusively accessed from single-threaded initialization.
    // 5. Synchronization: Single-threaded sequence on Core 0.
    // 6. Unwind safety: Bare-metal environment with panic = abort.
    let Some(hw_i2s_cam) = (unsafe { (*core::ptr::addr_of_mut!(HW_I2S_CAM)).as_mut() }) else {
        quiesce_psram_and_software_reset();
    };

    let ov2640_dma_ok = ov2640_hw_ok;
    let mut boot_dma_ok = false;

    let initial_slice = active_triple_static_slice(&psram_bufs.triple_fb);
    let mut camera = CameraDriver::new(initial_slice, FrameSize::Hd);

    if ov2640_hw_ok {
        for attempt in 1..=3u8 {
            feed_rtc_wdt();
            match capture_live_hd_jpeg_with_diag(
                hw_i2s_cam,
                &mut psram_bufs.triple_fb,
                &mut camera,
                FrameSize::Hd,
                &mut || {},
            ) {
                Ok(captured_len) => {
                    boot_dma_ok = true;
                    println!(
                        concat!(
                            "[KLIPPER-CAM-RS] [HW-3] Live OV2640 I2S0 DMA 1280x720 ",
                            "JPEG Capture: PASS ({} bytes, attempt {}, inv=false)"
                        ),
                        captured_len, attempt
                    );
                    break;
                }
                Err((eof_cnt, fb_len, b0, b1, b2, int_raw)) => {
                    println!(
                        concat!(
                            "[KLIPPER-CAM-RS] [HW-3] Attempt {} (inv=false): ",
                            "eof_cnt={} fb_len={} ",
                            "head=[0x{:02X},0x{:02X},0x{:02X}] int_raw=0x{:08X}"
                        ),
                        attempt, eof_cnt, fb_len, b0, b1, b2, int_raw
                    );
                    delay.delay_millis(25);
                }
            }
        }
    }
    if !boot_dma_ok {
        println!(
            concat!(
                "[KLIPPER-CAM-RS] [HW-3] Live OV2640 I2S0 DMA Capture fallback: ",
                "using synthetic 1280x720 JFIF ({} bytes)"
            ),
            psram_bufs.triple_fb.active_len()
        );
    }
    println!("============================================================");

    feed_rtc_wdt();
    record_boot_breadcrumb_post_wifi(4, u8::from(boot_dma_ok), 0);
    println!(
        concat!(
            "[KLIPPER-CAM-RS] [BREADCRUMB] prev_magic=0x{:08X} prev_boots={} ",
            "prev_reset=0x{:08X} prev_stage={} prev_ch={} prev_aid={} cur_boots={}"
        ),
        bc_prev.prev_magic,
        bc_prev.prev_boots,
        bc_prev.prev_reset,
        bc_prev.prev_stage,
        bc_prev.prev_ch,
        bc_prev.prev_aid,
        bc_prev.new_boots
    );

    let mut runtime = FirmwareRuntime::new(KlipperCamConfig::WROVER_KIT_HD, camera);
    runtime.server.ota_manager = klipper_cam_fw::OtaDataManager::new();
    // JUSTIFICATION: Initialize static FIRMWARE_RUNTIME global singleton.
    // SAFETY:
    // 1. Alignment & Non-null: addr_of_mut!(FIRMWARE_RUNTIME) points to static.
    // 2. Layout: Valid Option<FirmwareRuntime> slot in BSS.
    // 3. Provenance: FIRMWARE_RUNTIME lives for 'static.
    // 4. Aliasing: Unique single-threaded access during boot initialization.
    // 5. Synchronization: Called sequentially before server loop starts.
    // 6. Unwind safety: Bare-metal environment with panic = abort.
    unsafe {
        *core::ptr::addr_of_mut!(FIRMWARE_RUNTIME) = Some(runtime);
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
    use esp_radio::wifi::sta::{ScanMethod, StationConfig};
    use esp_radio::wifi::{AuthenticationMethodConfig, Config as WifiConfig, ControllerConfig};
    use klipper_cam_fw::config::{WifiAuthMode, ALLOW_OPEN_WIFI};

    // FW-NET-03: validate the boot configuration and fail closed. A bad
    // configuration or passphrase stops boot; it never falls back to Open.
    let boot_cfg = KlipperCamConfig::from_build_env();
    let auth_mode = match boot_cfg.boot_wifi_auth(ALLOW_OPEN_WIFI) {
        Ok(mode) => mode,
        Err(e) => panic!("[KLIPPER-CAM-RS] [CONFIG] refusing to start Wi-Fi: {e}"),
    };
    // FW-NET-04: the public default token disables OTA (POST /update -> 403).
    if boot_cfg.is_default_ota_token() {
        println!(
            "[KLIPPER-CAM-RS] [SECURITY] WARNING: default OTA token in use; \
             POST /update is disabled (403). Rebuild with WIFI_OTA_TOKEN set."
        );
    }

    let ssid = boot_cfg.network.wifi_ssid;
    let pass = boot_cfg.network.wifi_password;
    let auth = match auth_mode {
        WifiAuthMode::Open => {
            println!("[KLIPPER-CAM-RS] [SECURITY] WARNING: joining an Open Wi-Fi network (allow-open-wifi)");
            AuthenticationMethodConfig::Open
        }
        WifiAuthMode::WpaWpa2Personal => match pass.try_into() {
            Ok(pw) => AuthenticationMethodConfig::WpaWpa2Personal(pw),
            Err(_) => panic!(
                "[KLIPPER-CAM-RS] [CONFIG] WIFI_PASS rejected by the Wi-Fi driver; refusing to start Wi-Fi"
            ),
        },
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
struct Core0SpscAssembler {
    write_len: usize,
    in_frame: bool,
}

#[cfg(target_arch = "xtensa")]
impl Core0SpscAssembler {
    const fn new() -> Self {
        Self {
            write_len: 0,
            in_frame: false,
        }
    }
}

#[cfg(target_arch = "xtensa")]
fn drain_core1_spsc_to_triple(
    asm: &mut Core0SpscAssembler,
    triple_fb: &mut TripleFrameBuffer<'static>,
    camera: &mut CameraDriver<'static>,
    current_fs: FrameSize,
    p81_active: bool,
) {
    let t0 = ccount();
    for _ in 0..64u8 {
        let mut frame_done_len: Option<usize> = None;
        // JUSTIFICATION: Core 0 is the sole SPSC consumer on `CORE1_SPSC`.
        // SAFETY:
        // 1. Alignment & Non-null: `CORE1_SPSC` slots are 4-byte aligned in `.bss`.
        // 2. Layout: `clamped_len <= JPEG_SPSC_CHUNK_BYTES` verified inside.
        // 3. Provenance: Static `CORE1_SPSC` lives in internal DRAM for `'static`.
        // 4. Aliasing: Single producer (Core 1) / single consumer (Core 0).
        // 5. Synchronization: Acquire load of `head`, Release store of `tail`.
        // 6. Unwind safety: Bounded slice copy into `write_slot_mut()`.
        let popped = unsafe {
            CORE1_SPSC.pop_with(|chunk, flags| {
                if (flags & SPSC_FLAG_ABORT) != 0 {
                    asm.write_len = 0;
                    asm.in_frame = false;
                }
                if (flags & SPSC_FLAG_SOF) != 0 {
                    asm.write_len = 0;
                    asm.in_frame = true;
                }
                if asm.in_frame && !chunk.is_empty() {
                    let dst = triple_fb.write_slot_mut();
                    if asm.write_len + chunk.len() <= dst.len() {
                        dst[asm.write_len..asm.write_len + chunk.len()].copy_from_slice(chunk);
                        asm.write_len += chunk.len();
                    } else {
                        asm.in_frame = false;
                        asm.write_len = 0;
                    }
                }
                if (flags & SPSC_FLAG_EOF) != 0 && asm.in_frame {
                    asm.in_frame = false;
                    frame_done_len = Some(asm.write_len);
                    asm.write_len = 0;
                }
            })
        };
        if popped.is_none() {
            break;
        }
        if let Some(written_len) = frame_done_len {
            if written_len >= 64 {
                let mut valid_bounds: Option<(usize, usize)> = None;
                {
                    let dst = triple_fb.write_slot_mut();
                    let soi_window = written_len.min(256);
                    if let Some(soi_pos) = verify_jpeg_soi(&dst[..soi_window]) {
                        if let Some(eoi_rel) = verify_jpeg_eoi(&dst[soi_pos..written_len], false) {
                            valid_bounds = Some((soi_pos, soi_pos + eoi_rel + 2));
                        }
                    }
                }
                if let Some((soi_pos, eoi_end)) = valid_bounds {
                    let exact_len = eoi_end - soi_pos;
                    if exact_len >= 64 {
                        if soi_pos > 0 {
                            triple_fb.write_slot_mut().copy_within(soi_pos..eoi_end, 0);
                        }
                        if triple_fb.has_new_ready() {
                            FW_STATS.inc_p81_frames_superseded();
                        }
                        if triple_fb.commit_write_slot(exact_len).is_ok()
                            && !p81_active
                            && triple_fb.promote_ready_to_active()
                        {
                            camera.set_frame(active_triple_static_slice(triple_fb), current_fs);
                        }
                    } else {
                        FW_STATS.inc_c0_validation_failures();
                    }
                } else {
                    FW_STATS.inc_c0_validation_failures();
                }
            } else {
                FW_STATS.inc_c0_validation_failures();
            }
        }
    }
    FW_STATS.add_c0_spsc_cycles(ccount().wrapping_sub(t0));
}

#[cfg(target_arch = "xtensa")]
#[allow(clippy::too_many_arguments)]
#[inline(never)]
fn run_post_wifi_server_loop<B: klipper_cam_fw::SccbBus>(
    mut wifi_controller: esp_radio::wifi::WifiController<'static>,
    sta_cfg: esp_radio::wifi::sta::StationConfig,
    ssid: &'static str,
    mut flash_adapter: EspFlashAdapter<'static>,
    mut kept_sccb: Option<B>,
    mut live_ov2640: Option<Ov2640>,
    ov2640_dma_ok: bool,
    cpu_ctrl_peri: esp_hal::peripherals::CPU_CTRL<'static>,
) -> ! {
    use esp_hal::delay::Delay;
    use esp_println::println;
    use esp_radio::wifi::{Interface as WifiInterface, PowerSaveMode};
    use klipper_cam_fw::{
        parse_http_request, write_mjpeg_part_header, HttpMethod, RoamBackoff, RoamDecision,
        StreamSessionGuard, WeakLinkTimer, OTA_PARTITION_MAX_SIZE, ROAM_CHECK_INTERVAL_MS,
        ROAM_TRIGGER_RSSI_DBM, ROAM_WEAK_DURATION_MS,
    };
    use smoltcp::iface::{Config as IfaceConfig, Interface as NetInterface, SocketSet};
    use smoltcp::socket::dhcpv4;
    use smoltcp::socket::tcp::{
        Socket as TcpSocket, SocketBuffer as TcpSocketBuffer, State as TcpState,
    };
    use smoltcp::wire::{EthernetAddress, HardwareAddress, IpAddress, IpCidr, Ipv4Address};

    let delay = Delay::new();
    // JUSTIFICATION: Take ownership of static PSRAM_BUFFERS singleton on Core 0.
    // SAFETY:
    // 1. Alignment & Non-null: addr_of_mut!(PSRAM_BUFFERS) points to static.
    // 2. Layout: Valid Option<PsramBuffers> initialized prior to server loop.
    // 3. Provenance: PSRAM_BUFFERS lives for 'static.
    // 4. Aliasing: Exclusively taken once on Core 0; never accessed again.
    // 5. Synchronization: Core 1 is not running yet during this phase.
    // 6. Unwind safety: Bare-metal environment with panic = abort.
    let psram_buffers_opt = unsafe { (*core::ptr::addr_of_mut!(PSRAM_BUFFERS)).take() };
    let Some(PsramBuffers {
        triple_fb: mut triple_fb_owned,
        p80_rx0,
        p80_rx1,
        p80_rx2,
        p80_rx3,
        p80_tx0,
        p80_tx1,
        p80_tx2,
        p80_tx3,
        p80_hdr_bufs: mut p80_hdr_bufs_owned,
        p80_resp_bufs: mut p80_resp_bufs_owned,
        tcp81_rx0,
        tcp81_tx0,
        tcp81_rx1,
        tcp81_tx1,
        http80_ota_accum,
        free_psram_bytes,
    }) = psram_buffers_opt
    else {
        quiesce_psram_and_software_reset();
    };
    let triple_fb = &mut triple_fb_owned;
    let p80_hdr_bufs = &mut p80_hdr_bufs_owned;
    let p80_resp_bufs = &mut p80_resp_bufs_owned;

    // JUSTIFICATION: Access mutable static FIRMWARE_RUNTIME singleton on Core 0.
    // SAFETY:
    // 1. Alignment & Non-null: addr_of_mut!(FIRMWARE_RUNTIME) points to static.
    // 2. Layout: Valid Option<FirmwareRuntime> initialized during boot.
    // 3. Provenance: FIRMWARE_RUNTIME lives for 'static.
    // 4. Aliasing: Exclusively accessed from Core 0 server execution.
    // 5. Synchronization: Core 1 is not yet active during initialization.
    // 6. Unwind safety: Bare-metal environment with panic = abort.
    let Some(runtime) = (unsafe { (*core::ptr::addr_of_mut!(FIRMWARE_RUNTIME)).as_mut() }) else {
        quiesce_psram_and_software_reset();
    };

    record_boot_breadcrumb_post_wifi(5, 0, 0);
    let _ = wifi_controller.set_max_tx_power(WIFI_MAX_TX_POWER_QDBM);
    let _ = wifi_controller.set_power_saving(PowerSaveMode::None);

    let wifi_iface = WifiInterface::station();
    let mac = wifi_iface.mac_address();
    println!(
        concat!(
            "[KLIPPER-CAM-RS] [WIFI] Connecting to SSID '{}' ",
            "(MAC {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x})..."
        ),
        ssid, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]
    );
    runtime.on_wifi_sta_start();

    let mut boot_connected = connect_strongest_ap(&mut wifi_controller, &sta_cfg, ssid, 15_000);
    if boot_connected.is_none() {
        delay.delay_millis(250);
        boot_connected = connect_strongest_ap(&mut wifi_controller, &sta_cfg, ssid, 15_000);
    }
    // BSSID of the current association; drives the runtime roam decision.
    let mut current_bssid: Option<[u8; 6]> = boot_connected.as_ref().map(|info| info.bssid);
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
            println!("[KLIPPER-CAM-RS] [WIFI] Initial connect warning/timeout; will retry in loop");
            runtime.on_wifi_sta_disconnected();
        }
    }

    // 7. Configure smoltcp Ethernet Interface + DHCPv4 + Static IP
    //    (172.22.1.52/24, GW 172.22.1.1) + 4-socket Port 80 pool +
    //    2-socket Port 81 pool (7 sockets total)
    let mut net_device = SmoltcpWifiDevice(wifi_iface);
    let mut iface_cfg = IfaceConfig::new(HardwareAddress::Ethernet(EthernetAddress(mac)));
    // WDEV_RND_REG (`wdev_reg.h:19`: 0x6003_5144 APB /
    // 0x3FF7_5144 DPORT peripheral bus alias)
    const RNG_DATA_REG: *const u32 = 0x3FF7_5144 as *const u32;
    // JUSTIFICATION: Read hardware random number generator for smoltcp seed.
    // SAFETY: RNG_DATA_REG is a valid MMIO register mapped to APB bus.
    let hw_rng_lo = unsafe { core::ptr::read_volatile(RNG_DATA_REG) } as u64;
    // JUSTIFICATION: Read hardware random number generator for smoltcp seed.
    // SAFETY: RNG_DATA_REG is a valid MMIO register mapped to APB bus.
    let hw_rng_hi = unsafe { core::ptr::read_volatile(RNG_DATA_REG) } as u64;
    let boot_us = esp_hal::time::Instant::now()
        .duration_since_epoch()
        .as_micros();
    iface_cfg.random_seed = ((hw_rng_hi << 32) | hw_rng_lo) ^ boot_us ^ 0x9E37_79B9_7F4A_7C15;
    let mut iface = NetInterface::new(iface_cfg, &mut net_device, smol_now());

    // JUSTIFICATION: Mutable borrow of static SOCKET_STORAGE array.
    // SAFETY:
    // 1. Alignment & Non-null: addr_of_mut!(SOCKET_STORAGE) points to static.
    // 2. Layout: Valid [Option<SocketStorage>; 7] array in BSS.
    // 3. Provenance: SOCKET_STORAGE lives for 'static.
    // 4. Aliasing: Exclusively borrowed once during network init on Core 0.
    // 5. Synchronization: Executed on Core 0 before network polling starts.
    // 6. Unwind safety: Bare-metal environment with panic = abort.
    let socket_storage = unsafe { &mut *core::ptr::addr_of_mut!(SOCKET_STORAGE) };
    let mut sockets = SocketSet::new(&mut socket_storage[..]);

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
        s80.set_ack_delay(None);
        s80.set_timeout(None);
        s80.set_keep_alive(Some(smoltcp::time::Duration::from_millis(2_000)));
        let _ = s80.listen(80);
    }
    // JUSTIFICATION: Mutable borrow of static P80_STATES worker state array.
    // SAFETY: Single-threaded network loop owns exclusive access to P80_STATES.
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
        s81.set_ack_delay(None);
        s81.set_timeout(None);
        s81.set_keep_alive(Some(smoltcp::time::Duration::from_millis(2_000)));
        let _ = s81.listen(81);
    }
    // JUSTIFICATION: Mutable borrow of static P81_REQ_BUFS request array.
    // SAFETY: Single-threaded network loop owns exclusive access to P81_REQ_BUFS.
    let p81_req_bufs = unsafe { &mut *core::ptr::addr_of_mut!(P81_REQ_BUFS) };
    // JUSTIFICATION: Mutable borrow of static P81_STATES worker state array.
    // SAFETY: Single-threaded network loop owns exclusive access to P81_STATES.
    let p81_states = unsafe { &mut *core::ptr::addr_of_mut!(P81_STATES) };
    // JUSTIFICATION: Mutable borrow of static 512 B internal DRAM header buffer
    // for zero-copy Port 81 multipart header framing on Core 0.
    // SAFETY:
    // 1. Alignment & Non-null: `P81_HDR_BUF` is a valid `[u8; 512]` in `.bss`.
    // 2. Layout: 1-byte aligned initialized `u8` array.
    // 3. Provenance: Static lives for `'static`.
    // 4. Aliasing: Exclusively accessed by `run_post_wifi_server_loop` on Core 0.
    // 5. Synchronization: Single-core ownership on Core 0.
    // 6. Unwind safety: Diverging loop with `custom_halt` panic handler.
    let p81_hdr_buf = unsafe { &mut *core::ptr::addr_of_mut!(P81_HDR_BUF) };
    let mut p81_stream_owner: Option<usize> = None;
    let mut p81_last_tx_progress_ms: i64 = 0;
    let mut p81_last_send_queue: usize = 0;
    let mut p81_hdr_len: usize = 0;
    let mut p81_jpeg_slice: &'static [u8] = &[];
    let mut p81_tx_off: usize = 0;
    let mut p81_tx_len: usize = 0;

    let mut ip_configured = false;
    let mut static_ip_arp_pending = false;
    let mut last_rollback_retry_ms: u64 = 0;
    let mut connect_ms = smol_now().total_millis();
    let mut last_wifi_connected_ms = connect_ms;
    let mut last_reconnect_attempt_ms = connect_ms;
    let mut last_heartbeat_ms = connect_ms;
    let mut last_arp_announce_ms = connect_ms;
    let mut last_capture_ms = connect_ms;
    let mut last_p81_send_ms = connect_ms.saturating_sub(250);
    let mut post_snapshot_refresh_at_ms: Option<i64> = None;
    let mut last_applied_ctrl = klipper_cam_fw::CameraControlState::new_default();
    // Runtime roaming state (see `ROAM_*` in `klipper_cam_fw::ap_select`).
    let mut weak_link_timer = WeakLinkTimer::new();
    let mut last_roam_check_ms: u64 = u64::try_from(connect_ms).unwrap_or(0);
    let mut last_roam_scan_ms: Option<u64> = None;
    // Spacing between roam scans; grows while scans find no better AP.
    let mut roam_backoff = RoamBackoff::new();
    let mut spsc_asm = Core0SpscAssembler::new();
    let mut cached_wifi_rssi: Option<i32> = if wifi_controller.is_connected() {
        wifi_controller.rssi().ok()
    } else {
        None
    };
    let mut core1_stack_base: *const u8 = core::ptr::null();
    let mut core1_stack_len: usize = 0;

    if ov2640_dma_ok {
        let mut cpu_ctrl = CpuControl::new(cpu_ctrl_peri);
        CORE1_ACTIVE.store(true, Ordering::Release);
        let stack_raw_mut = core::ptr::addr_of_mut!(APP_CORE_STACK).cast::<u8>();
        let stack_bytes = core::mem::size_of::<Stack<8192>>();
        // JUSTIFICATION: Paint 0xA5 canary across the 8 KB Core 1 stack prior
        // to booting APP_CPU so `/status` can measure live stack HWM.
        // SAFETY:
        // 1. Preconditions: `stack_raw_mut` points to 8192-byte static stack.
        // 2. Alignment: `Stack<8192>` is 16-byte aligned; `u8` requires 1.
        // 3. Provenance & Lifetime: Static `.bss` storage valid for `'static`.
        // 4. Aliasing: Core 1 is still parked in reset before `start_app_core`.
        // 5. Volatile Semantics: `paint_stack_sentinel` uses `write_volatile`.
        // 6. Panic Safety: Bounded byte loop cannot panic or unwind.
        unsafe {
            paint_stack_sentinel(stack_raw_mut, stack_bytes);
        }
        // JUSTIFICATION: Hand exclusive 16-byte aligned 8 KB internal DRAM
        // stack to APP_CPU (`Core 1`).
        // SAFETY:
        // 1. Alignment & Non-null: `APP_CORE_STACK` is `Stack<8192>` (`align(16)`).
        // 2. Layout: Static 8 KB `MaybeUninit` stack in `.bss`.
        // 3. Provenance: Lives for `'static` and `AppCoreGuard` is forgotten.
        // 4. Aliasing: Core 0 never forms references to `APP_CORE_STACK` again.
        // 5. Synchronization: `start_app_core` orders stack pointers via Release.
        // 6. Unwind safety: Core 1 entry diverges (`-> !`).
        let app_stack = unsafe { &mut *core::ptr::addr_of_mut!(APP_CORE_STACK) };
        if let Ok(guard) = cpu_ctrl.start_app_core(app_stack, || core1_camera_producer_loop()) {
            // CRITICAL: Never drop AppCoreGuard or esp-hal parks APP_CPU!
            core::mem::forget(guard);
            core1_stack_base = core::ptr::addr_of!(APP_CORE_STACK).cast::<u8>();
            core1_stack_len = stack_bytes;
            println!("[KLIPPER-CAM-RS] [CORE1] APP_CPU continuous I2S0 camera producer started");
        } else {
            CORE1_ACTIVE.store(false, Ordering::Release);
        }
    }

    loop {
        let loop_t0 = ccount();
        feed_rtc_wdt();
        drain_core1_spsc_to_triple(
            &mut spsc_asm,
            triple_fb,
            &mut runtime.server.camera,
            runtime.server.control_state.framesize,
            p81_stream_owner.is_some(),
        );
        let now = smol_now();
        let now_ms = now.total_millis();

        if wifi_controller.is_connected() {
            last_wifi_connected_ms = now_ms;
            if cached_wifi_rssi.is_none() {
                cached_wifi_rssi = wifi_controller.rssi().ok();
            }
            let now_u64 = u64::try_from(now_ms).unwrap_or(0);
            if now_u64.saturating_sub(last_roam_check_ms) >= ROAM_CHECK_INTERVAL_MS {
                last_roam_check_ms = now_u64;
                let raw_rssi = wifi_controller.rssi().ok();
                cached_wifi_rssi = raw_rssi;
                let rssi = raw_rssi.and_then(|raw| i8::try_from(raw).ok());
                let weak_ms = match rssi {
                    Some(r) => weak_link_timer.update(now_u64, r, ROAM_TRIGGER_RSSI_DBM),
                    None => 0,
                };
                let scan_allowed = last_roam_scan_ms
                    .is_none_or(|t| now_u64.saturating_sub(t) >= roam_backoff.interval_ms());
                if let (Some(r), Some(bssid)) = (rssi, current_bssid) {
                    if weak_ms >= ROAM_WEAK_DURATION_MS && scan_allowed {
                        println!(
                            concat!(
                                "[KLIPPER-CAM-RS] [ROAM] Link weak for {} ms ",
                                "(rssi={}dBm < {}dBm); scanning..."
                            ),
                            weak_ms, r, ROAM_TRIGGER_RSSI_DBM
                        );
                        last_roam_scan_ms = Some(now_u64);
                        let decision = roam_scan_decision(&mut wifi_controller, ssid, bssid, r);
                        if let RoamDecision::Roam(target) = decision {
                            let tb = target.bssid();
                            println!(
                                concat!(
                                    "[KLIPPER-CAM-RS] [ROAM] Roaming {}dBm -> ",
                                    "bssid={:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x} ",
                                    "ch={} avg={}dBm"
                                ),
                                r,
                                tb[0],
                                tb[1],
                                tb[2],
                                tb[3],
                                tb[4],
                                tb[5],
                                target.channel(),
                                target.rssi_avg()
                            );
                            match roam_to_candidate(&mut wifi_controller, &sta_cfg, &target) {
                                Some(info) => {
                                    println!(
                                        concat!(
                                            "[KLIPPER-CAM-RS] [ROAM] Roamed to AP ",
                                            "(channel {}, aid {})"
                                        ),
                                        info.channel, info.aid
                                    );
                                    current_bssid = Some(info.bssid);
                                    cached_wifi_rssi = wifi_controller.rssi().ok();
                                    connect_ms = smol_now().total_millis();
                                    last_wifi_connected_ms = connect_ms;
                                    // The roam blocked the loop for seconds;
                                    // give the port 81 stream a fresh 10 s
                                    // window instead of aborting it as stalled.
                                    p81_last_tx_progress_ms = connect_ms;
                                    let announce_ip = ip_configured.then_some([172, 22, 1, 52]);
                                    after_sta_associated(
                                        &mut wifi_controller,
                                        runtime,
                                        &mut static_ip_arp_pending,
                                        &mut net_device,
                                        mac,
                                        announce_ip,
                                    );
                                    if ip_configured {
                                        last_arp_announce_ms = connect_ms;
                                    }
                                }
                                None => {
                                    println!(concat!(
                                        "[KLIPPER-CAM-RS] [ROAM] Roam failed; ",
                                        "base config restored, reconnect path will recover"
                                    ));
                                    current_bssid = None;
                                    cached_wifi_rssi = None;
                                }
                            }
                            roam_backoff.reset();
                        } else {
                            roam_backoff.on_no_better_ap();
                            println!(
                                concat!(
                                    "[KLIPPER-CAM-RS] [ROAM] No better AP; staying ",
                                    "(next scan in {} s)"
                                ),
                                roam_backoff.interval_ms() / 1000
                            );
                        }
                        weak_link_timer.reset();
                    }
                }
            }
        } else {
            cached_wifi_rssi = None;
            if now_ms - last_wifi_connected_ms >= 90_000
                && ota_accum_owner.is_none()
                && !runtime.server.is_reboot_scheduled()
            {
                println!(concat!(
                    "[KLIPPER-CAM-RS] [WATCHDOG] Wi-Fi disconnected > 90s; ",
                    "deasserting PSRAM CS# and rebooting..."
                ));
                quiesce_psram_and_software_reset();
            }
            if now_ms - last_reconnect_attempt_ms >= 5000 {
                runtime.on_wifi_sta_disconnected();
                println!(
                    concat!(
                        "[KLIPPER-CAM-RS] [WIFI] Attempting reconnect to SSID ",
                        "'{}' (disconnected {}ms)..."
                    ),
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
                    current_bssid = Some(info.bssid);
                    cached_wifi_rssi = wifi_controller.rssi().ok();
                    weak_link_timer.reset();
                    roam_backoff.reset();
                    let announce_ip = ip_configured.then_some([172, 22, 1, 52]);
                    after_sta_associated(
                        &mut wifi_controller,
                        runtime,
                        &mut static_ip_arp_pending,
                        &mut net_device,
                        mac,
                        announce_ip,
                    );
                    if ip_configured {
                        last_arp_announce_ms = connect_ms;
                    }
                }
            }
        }

        let _ = iface.poll(now, &mut net_device, &mut sockets);

        // Check DHCPv4 events or configure static IP (172.22.1.52/24,
        // GW 172.22.1.1) unconditionally
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
                    runtime.on_wifi_ip_confirmed(fw_ip);
                    static_ip_arp_pending = false;
                    ip_configured = true;
                    let _ = wifi_controller.set_power_saving(esp_radio::wifi::PowerSaveMode::None);
                    let _ = wifi_controller.set_max_tx_power(WIFI_MAX_TX_POWER_QDBM);
                    send_arp_announce(&mut net_device, mac, [172, 22, 1, 52], [172, 22, 1, 1]);
                    last_arp_announce_ms = now_ms;
                    println!(
                        concat!(
                            "[KLIPPER-CAM-RS] [NET] IP Configured: 172.22.1.52/{} + ",
                            "DHCP {}.{}.{}.{}/{} (Port 80 x4 & Port 81 x2 Ready)"
                        ),
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
            static_ip_arp_pending = true;
            runtime.wifi_state = klipper_cam_fw::WifiStaState::ConnectedGotIp { ip: fw_ip };
            runtime.webservers_started = true;
            ip_configured = true;
            let _ = wifi_controller.set_power_saving(esp_radio::wifi::PowerSaveMode::None);
            let _ = wifi_controller.set_max_tx_power(WIFI_MAX_TX_POWER_QDBM);
            if wifi_controller.is_connected() {
                send_arp_announce(&mut net_device, mac, [172, 22, 1, 52], [172, 22, 1, 1]);
                last_arp_announce_ms = now_ms;
            }
            println!(concat!(
                "[KLIPPER-CAM-RS] [NET] Static IP Configured: 172.22.1.52/24 ",
                "GW 172.22.1.1 (Port 80 x4 & Port 81 x2 Ready)"
            ));
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
        // Fallback Single-Core I2S0 Capture (only when Core 1 inactive)
        // ============================================================
        if !CORE1_ACTIVE.load(Ordering::Relaxed) {
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
                && (now_ms - last_capture_ms >= 2_000);
            if ov2640_dma_ok
                && ota_accum_owner.is_none()
                && !runtime.server.is_reboot_scheduled()
                && p81_stream_owner.is_none()
                && (post_snap_due || idle_refresh_due)
            {
                post_snapshot_refresh_at_ms = None;
                last_capture_ms = now_ms;
                // JUSTIFICATION: Fallback single-core access to `HW_I2S_CAM`
                // strictly when `CORE1_ACTIVE` is false.
                // SAFETY:
                // 1. Alignment & Non-null: Static `HW_I2S_CAM` in internal DRAM.
                // 2. Layout: Initialized `Option<Esp32I2sCamera>`.
                // 3. Provenance: Lives for `'static`.
                // 4. Aliasing: `CORE1_ACTIVE` is false, so Core 1 is parked.
                // 5. Synchronization: Checked before borrow.
                // 6. Unwind safety: Bounded fallback capture.
                if let Some(hw_i2s_cam) = unsafe { (*core::ptr::addr_of_mut!(HW_I2S_CAM)).as_mut() }
                {
                    if capture_live_hd_jpeg_with_diag(
                        hw_i2s_cam,
                        triple_fb,
                        &mut runtime.server.camera,
                        runtime.server.control_state.framesize,
                        &mut || {
                            poll_net_burst(&mut iface, &mut net_device, &mut sockets);
                        },
                    )
                    .is_ok()
                    {
                        last_capture_ms = smol_now().total_millis();
                    }
                }
            }
        }

        // ============================================================
        // Port 80 4-Socket Listener Pool (Control / Snapshot / Status / OTA)
        // ============================================================
        let p80_t0 = ccount();
        for idx in 0..PORT80_SOCKET_COUNT {
            let sock80 = sockets.get_mut::<TcpSocket>(tcp80_handles[idx]);
            let st = &mut p80_states[idx];

            if !sock80.is_open() {
                if ota_accum_owner == Some(idx) {
                    ota_accum_owner = None;
                }
                *st = Port80WorkerState::default();
                sock80.set_ack_delay(None);
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
                let finished_cleanly =
                    matches!(sock80.state(), TcpState::TimeWait | TcpState::Closed);
                if !sock80.is_active() || finished_cleanly || now_ms >= st.close_deadline_ms {
                    let was_snapshot = st.is_snapshot;
                    if was_snapshot && post_snapshot_refresh_at_ms.is_none() {
                        post_snapshot_refresh_at_ms = Some(now_ms.saturating_add(50));
                    }
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
                    let was_snapshot = st.is_snapshot;
                    if was_snapshot && post_snapshot_refresh_at_ms.is_none() {
                        post_snapshot_refresh_at_ms = Some(now_ms.saturating_add(50));
                    }
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

                        // SEC-102 & LOGIC-101: Buffer capacity check &
                        // auth-first promotion into shared http80_ota_accum
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
                                } else if let Err(rej) = runtime
                                    .server
                                    .check_ota_auth(parsed.ota_token.unwrap_or(b""))
                                {
                                    // FW-NET-04: 403 while the default token is
                                    // configured, else 401 on a token mismatch.
                                    if let Ok(err) = write_http_error(
                                        p80_resp_bufs[idx],
                                        rej.status,
                                        rej.reason,
                                        rej.body,
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
                                    if parsed.method == HttpMethod::Post && parsed.path == "/update"
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
                            } else if (!st.using_ota_accum && st.req_len >= p80_hdr_bufs[idx].len())
                                || peer_closed_rx
                            {
                                req_complete = true;
                            }
                        }

                        if req_complete {
                            // Core 1 must be parked in IRAM before any flash
                            // access disables the cache; a failed pause means
                            // the OTA request is refused rather than risked.
                            let mut core1_parked = true;
                            if is_ota_post {
                                core1_parked = pause_core1_camera_producer();
                                if core1_parked {
                                    let booted_slot = runtime.server.ota_manager.active_slot();
                                    if let Ok(loaded) =
                                        klipper_cam_fw::OtaDataManager::load_from_flash(
                                            &mut flash_adapter,
                                        )
                                    {
                                        if loaded.active_slot() == booted_slot {
                                            runtime.server.ota_manager = loaded;
                                        }
                                    }
                                } else {
                                    println!(
                                        "[KLIPPER-CAM-RS] [CORE1] pause timeout; refusing OTA POST"
                                    );
                                }
                            }
                            if is_snapshot_req {
                                st.is_snapshot = true;
                                runtime.server.camera.set_frame(
                                    latest_snapshot_static_slice(triple_fb),
                                    runtime.server.control_state.framesize,
                                );
                            }

                            let free_dram = esp_alloc::HEAP
                                .free_caps(esp_alloc::MemoryCapability::Internal.into())
                                as u32;
                            let core1_stack_hwm = if !core1_stack_base.is_null()
                                && core1_stack_len > 0
                            {
                                // JUSTIFICATION: Volatile byte-level scan of Core 1's
                                // downward-growing stack canary without forming `&[u8]`
                                // references that would alias `&mut Stack<8192>`.
                                // SAFETY:
                                // 1. Preconditions: `core1_stack_base` and `core1_stack_len`
                                //    were captured from `APP_CORE_STACK` (`8192` bytes).
                                // 2. Alignment: `u8` has alignment 1.
                                // 3. Provenance & Lifetime: `'static` `.bss` stack memory.
                                // 4. Aliasing: Raw volatile byte reads form zero references.
                                // 5. Volatile Semantics: `read_volatile` reads SRAM directly.
                                // 6. Panic Safety: Bounded scan cannot panic or unwind.
                                unsafe {
                                    measure_stack_hwm_volatile(core1_stack_base, core1_stack_len)
                                }
                            } else {
                                0
                            };
                            runtime.server.set_extended_telemetry(
                                now_ms,
                                free_dram,
                                free_psram_bytes,
                                cached_wifi_rssi,
                            );
                            runtime.server.set_diagnostics(
                                reset_reason_label(BOOT_RESET_REASON.load(Ordering::Relaxed)),
                                boot_app_sha256_str(),
                                free_dram,
                                core1_stack_hwm,
                                FW_STATS.snapshot(),
                            );

                            let req_slice: &[u8] = if st.using_ota_accum {
                                &http80_ota_accum[..st.req_len]
                            } else {
                                &p80_hdr_bufs[idx][..st.req_len]
                            };

                            let handle_res = if core1_parked {
                                runtime
                                    .server
                                    .handle_raw_request_with_flash(
                                        ServerPort::Control,
                                        req_slice,
                                        p80_resp_bufs[idx],
                                        &mut flash_adapter,
                                    )
                                    .map(|s| s.len())
                            } else {
                                write_http_error(
                                    p80_resp_bufs[idx],
                                    503,
                                    "Service Unavailable",
                                    "503 Camera core busy; retry OTA\n",
                                    false,
                                )
                                .map(|s| s.len())
                            };

                            if is_snapshot_req && p81_stream_owner.is_some() {
                                runtime.server.camera.set_frame(
                                    active_triple_static_slice(triple_fb),
                                    runtime.server.control_state.framesize,
                                );
                            }
                            if is_ota_post
                                && !runtime.server.is_reboot_scheduled()
                                && !resume_core1_camera_producer()
                            {
                                FW_STATS.inc_sccb_errors();
                                println!("camera: Core 1 resume timed out after aborted OTA");
                            }

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
        FW_STATS.add_c0_p80_cycles(ccount().wrapping_sub(p80_t0));
        poll_deferred_rollback_cancel(
            runtime,
            &mut flash_adapter,
            u64::try_from(now_ms).unwrap_or(0),
            &mut last_rollback_retry_ms,
        );

        // Apply any pending live camera control updates from
        // GET /control?var=...&val=...
        // only after all active Port 80 HTTP responses have finished flushing.
        let p80_tx_active = p80_states.iter().any(|s| s.tx_len > 0 && !s.closing);
        if p80_tx_active {
            poll_net_burst(&mut iface, &mut net_device, &mut sockets);
        } else if runtime.server.has_pending_control_update() {
            if !pause_core1_camera_producer() {
                FW_STATS.inc_sccb_errors();
                println!("camera: Core 1 pause timed out; deferring /control update");
            } else if let Some(ctrl) = runtime.server.take_pending_control_update() {
                if let (Some(ref mut sccb), Some(ref mut ov)) = (&mut kept_sccb, &mut live_ov2640) {
                    let mut applier = Ov2640ControlApplier(ov, sccb);
                    let errs = klipper_cam_fw::http::apply_camera_control_diff(
                        &mut applier,
                        &ctrl,
                        &mut last_applied_ctrl,
                    );
                    for _ in 0..errs {
                        FW_STATS.inc_sccb_errors();
                    }
                } else {
                    last_applied_ctrl = ctrl;
                }
                CORE1_SPSC.reset_consumer();
                spsc_asm = Core0SpscAssembler::new();
                if !resume_core1_camera_producer() {
                    FW_STATS.inc_sccb_errors();
                    println!("camera: Core 1 resume timed out after /control update");
                }
                post_snapshot_refresh_at_ms = Some(now_ms + 150);
            } else if !resume_core1_camera_producer() {
                FW_STATS.inc_sccb_errors();
            }
        }

        // Reboot if OTA firmware update completed and all Port 80
        // TX buffers have flushed
        if runtime.server.is_reboot_scheduled() {
            let all_flushed = tcp80_handles
                .iter()
                .all(|&h| sockets.get_mut::<TcpSocket>(h).send_queue() == 0);
            if all_flushed {
                let _ = iface.poll(smol_now(), &mut net_device, &mut sockets);
                delay.delay_millis(250);
                println!(concat!(
                    "[KLIPPER-CAM-RS] [OTA] Flash complete, deasserting ",
                    "PSRAM CS# and rebooting into updated slot..."
                ));
                quiesce_psram_and_software_reset();
            }
        }

        // ============================================================
        // Port 81 2-Socket Symmetric MJPEG Stream Pool (/stream & /)
        // ============================================================
        let p81_t0 = ccount();
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
                p81_hdr_len = 0;
                p81_jpeg_slice = &[];
                p81_tx_off = 0;
                p81_tx_len = 0;
                p81_last_send_queue = 0;
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
                        // JUSTIFICATION: Mutable borrow of static drain buffer.
                        // SAFETY: Single-threaded loop has exclusive access.
                        let drain = unsafe { &mut *core::ptr::addr_of_mut!(P81_DRAIN_BUF) };
                        let _ = sock81.recv_slice(drain);
                    }
                    if p81_tx_off < p81_tx_len {
                        let prev_off = p81_tx_off;
                        enqueue_p81_zero_copy(
                            sock81,
                            &p81_hdr_buf[..p81_hdr_len],
                            p81_jpeg_slice,
                            &mut p81_tx_off,
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
                    drain_core1_spsc_to_triple(
                        &mut spsc_asm,
                        triple_fb,
                        &mut runtime.server.camera,
                        runtime.server.control_state.framesize,
                        true,
                    );
                }

                let tx_drained = {
                    let sock81 = sockets.get_mut::<TcpSocket>(tcp81_handles[idx]);
                    all_enqueued
                        && sock81.can_send()
                        && sock81.send_queue() <= P81_PIPELINE_LOW_WATER_BYTES
                };

                if tx_drained {
                    // `capture_live_hd_jpeg_with_diag` commits *and* promotes
                    // internally, so a frame_seq change must also count as fresh.
                    let seq_before_capture = triple_fb.frame_seq();
                    if !CORE1_ACTIVE.load(Ordering::Relaxed)
                        && ov2640_dma_ok
                        && !triple_fb.has_new_ready()
                        && ota_accum_owner.is_none()
                        && !runtime.server.is_reboot_scheduled()
                    {
                        // JUSTIFICATION: Fallback single-core access to `HW_I2S_CAM`
                        // strictly when `CORE1_ACTIVE` is false.
                        // SAFETY:
                        // 1. Alignment & Non-null: Static `HW_I2S_CAM` in DRAM.
                        // 2. Layout: Initialized `Option<Esp32I2sCamera>`.
                        // 3. Provenance: Lives for `'static`.
                        // 4. Aliasing: `CORE1_ACTIVE` is false.
                        // 5. Synchronization: Checked before borrow.
                        // 6. Unwind safety: Bounded fallback capture.
                        let hw = unsafe { (*core::ptr::addr_of_mut!(HW_I2S_CAM)).as_mut() };
                        if let Some(hw_i2s_cam) = hw {
                            let _ = capture_live_hd_jpeg_with_diag(
                                hw_i2s_cam,
                                triple_fb,
                                &mut runtime.server.camera,
                                runtime.server.control_state.framesize,
                                &mut || {
                                    poll_net_burst(&mut iface, &mut net_device, &mut sockets);
                                },
                            );
                        }
                    }

                    let got_fresh = triple_fb.promote_ready_to_active()
                        || triple_fb.frame_seq() != seq_before_capture;
                    if got_fresh || (!ov2640_dma_ok && now_ms - last_p81_send_ms >= 200) {
                        let fresh_slice = active_triple_static_slice(triple_fb);
                        runtime
                            .server
                            .camera
                            .set_frame(fresh_slice, runtime.server.control_state.framesize);
                        if let Ok(part_hdr_len) =
                            write_mjpeg_part_header(fresh_slice.len(), p81_hdr_buf).map(|h| h.len())
                        {
                            FW_STATS.inc_p81_frames_sent();
                            p81_hdr_len = part_hdr_len;
                            p81_jpeg_slice = fresh_slice;
                            p81_tx_len = p81_hdr_len + p81_jpeg_slice.len();
                            p81_tx_off = 0;
                            {
                                let sock81 = sockets.get_mut::<TcpSocket>(tcp81_handles[idx]);
                                enqueue_p81_zero_copy(
                                    sock81,
                                    &p81_hdr_buf[..p81_hdr_len],
                                    p81_jpeg_slice,
                                    &mut p81_tx_off,
                                );
                                let sent_ms = smol_now().total_millis();
                                last_p81_send_ms = sent_ms;
                                p81_last_tx_progress_ms = sent_ms;
                                p81_last_send_queue = sock81.send_queue();
                            }
                            poll_net_burst(&mut iface, &mut net_device, &mut sockets);
                            drain_core1_spsc_to_triple(
                                &mut spsc_asm,
                                triple_fb,
                                &mut runtime.server.camera,
                                runtime.server.control_state.framesize,
                                true,
                            );
                        }
                    }
                }
            } else {
                let mut preempt_and_start = false;
                {
                    let sock81 = sockets.get_mut::<TcpSocket>(tcp81_handles[idx]);
                    let st = &mut p81_states[idx];

                    if !sock81.is_open() {
                        *st = Port81WorkerState::default();
                        sock81.set_ack_delay(None);
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
                            let finished_cleanly =
                                matches!(sock81.state(), TcpState::TimeWait | TcpState::Closed);
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
                                        let err_buf: &mut [u8];
                                        // JUSTIFICATION: Mutable borrow of static error buffer.
                                        // SAFETY: Single-threaded loop has exclusive access.
                                        unsafe {
                                            err_buf = &mut *core::ptr::addr_of_mut!(P81_ERR_BUF);
                                        }
                                        if let Ok(resp) = runtime.server.handle_raw_request(
                                            ServerPort::Stream,
                                            &p81_req_bufs[idx][..st.req_len],
                                            err_buf,
                                        ) {
                                            match sock81.send_slice(resp) {
                                                Ok(sent) if sent == resp.len() => {}
                                                Ok(_) | Err(_) => FW_STATS.inc_send_partial(),
                                            }
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
                    // Cleanly preempt any prior/stale Port 81 stream so browser
                    // reloads or immediate reconnects succeed 100% of the time.
                    if let Some(prev_owner) = p81_stream_owner {
                        if prev_owner != idx {
                            let prev_sock = sockets.get_mut::<TcpSocket>(tcp81_handles[prev_owner]);
                            prev_sock.abort();
                            p81_states[prev_owner] = Port81WorkerState::default();
                        }
                    }
                    p81_stream_owner = None;
                    p81_hdr_len = 0;
                    p81_jpeg_slice = &[];
                    p81_tx_off = 0;
                    p81_tx_len = 0;
                    p81_last_send_queue = 0;
                    runtime
                        .server
                        .stream_active_flag()
                        .store(false, Ordering::Release);

                    let _ = triple_fb.promote_ready_to_active();
                    let initial_jpeg = active_triple_static_slice(triple_fb);
                    runtime
                        .server
                        .camera
                        .set_frame(initial_jpeg, runtime.server.control_state.framesize);

                    let mut started_ok = false;
                    if let Some(guard) =
                        StreamSessionGuard::try_acquire(runtime.server.stream_active_flag())
                    {
                        let stream_hdr_res =
                            guard.write_stream_headers(p81_hdr_buf).map(|b| b.len());
                        if let Ok(stream_hdr_len) = stream_hdr_res {
                            let part_hdr_res = write_mjpeg_part_header(
                                initial_jpeg.len(),
                                &mut p81_hdr_buf[stream_hdr_len..],
                            )
                            .map(|h| h.len());
                            if let Ok(part_hdr_len) = part_hdr_res {
                                p81_hdr_len = stream_hdr_len + part_hdr_len;
                                p81_jpeg_slice = initial_jpeg;
                                p81_tx_len = p81_hdr_len + p81_jpeg_slice.len();
                                p81_tx_off = 0;
                                started_ok = true;
                            }
                        }
                        if started_ok {
                            core::mem::forget(guard);
                        }
                    }

                    let sock81 = sockets.get_mut::<TcpSocket>(tcp81_handles[idx]);
                    if started_ok {
                        FW_STATS.inc_p81_frames_sent();
                        p81_stream_owner = Some(idx);
                        p81_states[idx] = Port81WorkerState::default();
                        enqueue_p81_zero_copy(
                            sock81,
                            &p81_hdr_buf[..p81_hdr_len],
                            p81_jpeg_slice,
                            &mut p81_tx_off,
                        );
                        last_p81_send_ms = now_ms;
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
        FW_STATS.add_c0_p81_cycles(ccount().wrapping_sub(p81_t0));

        poll_net_burst(&mut iface, &mut net_device, &mut sockets);

        if now_ms - last_heartbeat_ms >= 30_000 {
            last_heartbeat_ms = now_ms;
            println!(
                concat!(
                    "[KLIPPER-CAM-RS] Heartbeat: uptime={}ms connected={} ",
                    "stream_active={} frame_seq={} last_jpeg={}B ",
                    "c1_pushed={} c1_dropped={}"
                ),
                now_ms,
                wifi_controller.is_connected(),
                p81_stream_owner.is_some(),
                triple_fb.frame_seq(),
                triple_fb.active_len(),
                CORE1_SPSC.frames_pushed(),
                CORE1_SPSC.frames_dropped()
            );
        }

        let loop_delta = ccount().wrapping_sub(loop_t0);
        FW_STATS.update_max_loop_latency_us(loop_delta / 240);
    }
}

#[cfg(target_arch = "xtensa")]
#[inline(never)]
fn configure_xclk(
    ledc_periph: esp_hal::peripherals::LEDC<'static>,
    gpio21: esp_hal::peripherals::GPIO21<'static>,
) -> bool {
    use esp_hal::gpio::DriveMode;
    use esp_hal::ledc::channel::{self, ChannelIFace};
    use esp_hal::ledc::timer::{self, TimerIFace};
    use esp_hal::ledc::{LSGlobalClkSource, Ledc, LowSpeed};
    use esp_hal::time::Rate;
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
    Option<
        klipper_cam_fw::EmbeddedHalSccb<
            esp_hal::i2c::master::I2c<'static, esp_hal::Blocking>,
            esp_hal::delay::Delay,
        >,
    >,
) {
    use esp_hal::delay::Delay;
    use esp_hal::i2c::master::{Config as I2cConfig, I2c};
    use esp_hal::time::Rate;
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

#[cfg(target_arch = "xtensa")]
struct Ov2640ControlApplier<'a, B>(&'a mut Ov2640, &'a mut B);

#[cfg(target_arch = "xtensa")]
impl<'a, B: esp32_cam_sccb::SccbBus> klipper_cam_fw::http::CameraControlApplier
    for Ov2640ControlApplier<'a, B>
{
    fn apply_framesize(&mut self, val: FrameSize) -> Result<(), klipper_cam_fw::CameraError> {
        self.0.set_framesize(self.1, val)
    }

    fn apply_quality(&mut self, val: i32) -> Result<(), klipper_cam_fw::CameraError> {
        self.0.set_quality(self.1, val)
    }

    fn apply_brightness(&mut self, val: i32) -> Result<(), klipper_cam_fw::CameraError> {
        self.0.set_brightness(self.1, val)
    }

    fn apply_contrast(&mut self, val: i32) -> Result<(), klipper_cam_fw::CameraError> {
        self.0.set_contrast(self.1, val)
    }

    fn apply_saturation(&mut self, val: i32) -> Result<(), klipper_cam_fw::CameraError> {
        self.0.set_saturation(self.1, val)
    }

    fn apply_special_effect(&mut self, val: i32) -> Result<(), klipper_cam_fw::CameraError> {
        self.0.set_special_effect(self.1, val)
    }

    fn apply_hmirror(&mut self, val: bool) -> Result<(), klipper_cam_fw::CameraError> {
        self.0.set_hmirror(self.1, val)
    }

    fn apply_vflip(&mut self, val: bool) -> Result<(), klipper_cam_fw::CameraError> {
        self.0.set_vflip(self.1, val)
    }

    fn apply_awb(&mut self, val: bool) -> Result<(), klipper_cam_fw::CameraError> {
        self.0.set_whitebal(self.1, val)
    }

    fn apply_agc(&mut self, val: bool) -> Result<(), klipper_cam_fw::CameraError> {
        self.0.set_gain_ctrl(self.1, val)
    }

    fn apply_aec(&mut self, val: bool) -> Result<(), klipper_cam_fw::CameraError> {
        self.0.set_exposure_ctrl(self.1, val)
    }

    fn apply_ae_level(&mut self, val: i32) -> Result<(), klipper_cam_fw::CameraError> {
        self.0.set_ae_level(self.1, val)
    }
}

/// Bare-metal Xtensa firmware entrypoint for `ESP32-WROVER-DEV`.
#[cfg(target_arch = "xtensa")]
#[esp_hal::main]
fn main() -> ! {
    use esp_hal::clock::CpuClock;
    use esp_hal::delay::Delay;
    use esp_hal::psram::{Psram, PsramCacheSpeed, PsramConfig, PsramSize, PsramVaddrMode};
    use esp_println::println;

    // 0. Exact ESP-IDF v5.5.4 + klipper_cam.c Startup Sequence:
    //    1) Read reset_reason, hold GPIO 16 (PSRAM CS#) HIGH from IRAM,
    //       initialize clocks (240 MHz), and immediately re-arm the 15s RTC_WDT.
    //    2) Power down/isolate the Wi-Fi/RF domain during early peripheral init
    //       (matching esp_rtc_init()).
    //    3) Never drive GPIO 4 as an output on ESP32-WROVER-DEV (`GPIO 4` is
    //       `CAM_PIN_D0` / `OV2640` `Y2`).
    //    4) Record boot breadcrumb in RTC_CNTL_STORE6/7_REG and detect booted
    //       OTA slot from MMU with ZERO boot-time NOR flash operations.
    const RTC_CNTL_RESET_STATE_REG: *const u32 = 0x3FF4_8034 as *const u32;
    // JUSTIFICATION: Read RTC controller reset state register during cold boot.
    // SAFETY: RTC_CNTL_RESET_STATE_REG is a valid MMIO register address.
    let reset_reason = unsafe { core::ptr::read_volatile(RTC_CNTL_RESET_STATE_REG) };
    BOOT_RESET_REASON.store(reset_reason, Ordering::Relaxed);
    hold_psram_cs_high_safe();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::_240MHz);
    let peripherals = esp_hal::init(config);
    arm_rtc_wdt_15s();
    esp_rtc_wifi_force_pd(true);
    let delay = Delay::new();

    let bc_prev = record_boot_breadcrumb_rtc_init(reset_reason);
    let mmu_booted_slot = detect_booted_ota_manager_from_mmu();
    feed_rtc_wdt();

    // Initialize internal DRAM heap for esp-rtos and esp-radio Wi-Fi STA
    // (~112 KB strictly internal)
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 64 * 1024);
    esp_alloc::heap_allocator!(size: 48 * 1024);

    // Initialize 4 MB external PSRAM at 40 MHz Flash / 40 MHz PSRAM
    // (matching a364a6a^:sdkconfig.defaults CONFIG_ESPTOOLPY_FLASHFREQ_40M=y
    // + CONFIG_SPIRAM_SPEED_40M=y) and send 0xF5 (QPI exit) + 0x35 (1-bit SPI
    // QPI enter) via hardware SPI1_CS1 (ensure_psram_qpi_mode_spi1) without
    // ever bit-banging SD0..SD3!
    // On cold power-on reset (`(reset_reason & 0x3F) == 1`), wait 50 ms while
    // GPIO 16 (PSRAM CS#) is held HIGH so VDD_SDIO and the PSRAM chip's internal
    // POR stabilize cleanly at 240 MHz before Psram::new().
    if (reset_reason & 0x3F) == 1 {
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

    // 1. Configure physical 20 MHz XCLK on GPIO 21 via LEDC LowSpeed
    //    Timer0 / Channel0
    let xclk_hw_ok = configure_xclk(peripherals.LEDC, peripherals.GPIO21);
    println!(
        "[KLIPPER-CAM-RS] [HW-1] LEDC 20 MHz XCLK on GPIO 21: {}",
        if xclk_hw_ok { "PASS" } else { "FAIL" }
    );
    delay.delay_millis(25);
    feed_rtc_wdt();

    // 2. Configure physical I2C0 on GPIO 26 (SDA) / GPIO 27 (SCL) at 200 kHz
    //    and program OV2640
    let (ov2640_hw_ok, live_ov2640, kept_sccb) =
        configure_i2c0_and_ov2640(peripherals.I2C0, peripherals.GPIO26, peripherals.GPIO27);
    feed_rtc_wdt();

    // 3. Carve PSRAM buffers, run 7-crate HIL verification suite, and
    //    configure I2S0 camera
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
    let ov2640_dma_ok = capture_boot_dma_and_init_runtime(ov2640_hw_ok, bc_prev);
    // 6. Release Wi-Fi/RF force power-down, start esp-rtos scheduler,
    //    and initialize esp-radio Wi-Fi Station
    esp_rtc_wifi_force_pd(false);
    let timg0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);
    feed_rtc_wdt();

    let (sta_cfg, ctrl_cfg, ssid) = build_wifi_station_configs();
    feed_rtc_wdt();
    let Ok(wifi_controller) = esp_radio::wifi::WifiController::new(peripherals.WIFI, ctrl_cfg)
    else {
        quiesce_psram_and_software_reset();
    };
    feed_rtc_wdt();
    // JUSTIFICATION: Wrap FlashStorage with multicore_ignore; Core1PauseGuard
    // parks APP_CPU in IRAM before every flash cache disable.
    // SAFETY:
    // 1. Preconditions: Runs after WifiController::new before APP_CPU starts.
    // 2. Alignment: FlashStorage takes ownership of peripheral singleton.
    // 3. Provenance & Lifetime: `peripherals.FLASH` has `'static` lifetime.
    // 4. Aliasing: Exclusively owned by `EspFlashAdapter` on Core 0.
    // 5. Synchronization: `Core1PauseGuard` parks Core 1 on all flash ops.
    // 6. Unwind safety: Bare-metal firmware builds with `panic = "abort"`.
    let mut flash_adapter = EspFlashAdapter(unsafe {
        esp_storage::FlashStorage::new(peripherals.FLASH).multicore_ignore()
    });
    load_boot_otadata_and_sha256(&mut flash_adapter, mmu_booted_slot);
    feed_rtc_wdt();
    run_post_wifi_server_loop(
        wifi_controller,
        sta_cfg,
        ssid,
        flash_adapter,
        kept_sccb,
        live_ov2640,
        ov2640_dma_ok,
        peripherals.CPU_CTRL,
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
