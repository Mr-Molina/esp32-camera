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

#[cfg(target_arch = "xtensa")]
use esp32_cam_ll::{build_circular_ring, Lldesc};
use esp32_cam_ll::{verify_circular_ring, DmaRingStorage, DEFAULT_CAMERA_DMA_BUFFER_SIZE_MAX};
use klipper_cam_fw::{
    calc_dma_sizes, decode_jpeg_to_rgb888, encode_rgb888_to_jpeg, handle_captive_dns_query,
    parse_jpeg_dimensions, verify_jpeg_eoi, verify_jpeg_soi, verify_partition_table_integrity,
    BoardPins, CameraDriver, CameraSensor, DhcpServer, DmaElement, Esp32I2sCamera, EspImageHeader,
    FirmwareRuntime, FramePool, FrameSize, GrabMode, HttpResponseView, Ipv4Addr, KlipperCamConfig,
    MockSccbBus, OtaSlot, Ov2640, PingPongFrameBuffer, PixelFormat, ServerPort, SliceWriter,
    WroverKitPins, XclkConfig, XclkController, DEFAULT_JPEG_QUALITY, DEFAULT_XCLK_FREQ_HZ,
    DHCP_CLIENT_PORT, DHCP_SERVER_PORT, DNS_PORT, MJPEG_CONTENT_TYPE,
};

#[cfg(target_arch = "xtensa")]
use core::cell::UnsafeCell;
#[cfg(target_arch = "xtensa")]
use core::sync::atomic::AtomicU8;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
#[cfg(target_arch = "xtensa")]
use esp32_cam_wifi_mac::phy_bridge::phy_run_tx_pwctrl_bg;
#[cfg(target_arch = "xtensa")]
use esp32_cam_wifi_mac::phy_bridge::phy_set_max_tx_power_qdbm;
#[cfg(target_arch = "xtensa")]
use esp32_cam_wifi_mac::phy_bridge::XtensaPhySession;
#[cfg(target_arch = "xtensa")]
use esp32_cam_wifi_mac::ApConfig;
#[cfg(target_arch = "xtensa")]
use esp32_cam_wifi_mac::Esp32StaWifiDriver;
#[cfg(target_arch = "xtensa")]
use esp32_cam_wifi_mac::HardwareWifiMmioBus;
#[cfg(target_arch = "xtensa")]
use esp32_cam_wifi_mac::StaConnectConfig;
#[cfg(target_arch = "xtensa")]
use esp32_cam_wifi_mac::StaState;
#[cfg(target_arch = "xtensa")]
use esp_hal::system::{Cpu, CpuControl, Stack};
#[cfg(target_arch = "xtensa")]
use klipper_cam_fw::frame_buffer::PinnedFrameSlot;
#[cfg(target_arch = "xtensa")]
use klipper_cam_fw::http::handle_setup_http_request;
#[cfg(target_arch = "xtensa")]
use klipper_cam_fw::http::SetupHttpOutcome;
#[cfg(target_arch = "xtensa")]
use klipper_cam_fw::telemetry::{
    ccount, format_sha256_hex, measure_stack_hwm_volatile, paint_stack_sentinel,
    reset_reason_label, FwStats,
};
#[cfg(any(target_arch = "xtensa", test))]
use klipper_cam_fw::AP_IP;
#[cfg(target_arch = "xtensa")]
use klipper_cam_fw::{
    write_http_error, Core1JpegSpsc, IpMode, ProvStore, ProvisionedConfig, TripleFrameBuffer,
    UartProvAction, UartProvSession, JPEG_SPSC_CHUNK_BYTES, OTA_0_OFFSET, OTA_0_SIZE, OTA_1_OFFSET,
    OTA_1_SIZE, SPSC_FLAG_ABORT, SPSC_FLAG_EOF, SPSC_FLAG_SOF,
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

/// DMA ring for the I2S0 self-test: 4 nodes of 128 bytes. A `static`, so it
/// lies in internal SRAM (DMA-capable) and is 4-byte aligned (WP-08 step 7).
static SELF_TEST_RING: DmaRingStorage<4, 512> = DmaRingStorage::new();

#[inline(never)]
fn hil_check_ll_dma_and_i2s0() -> bool {
    let Some(mut i2s_cam) = Esp32I2sCamera::take() else {
        return false;
    };
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

    // WP-08 step 7: a checked, writable, aligned `'static` ring (it used to be
    // an immutable, 1-byte-aligned stack array) and a geometry that matches
    // it: 2 nodes per half-buffer.
    let Ok(mut ring) = SELF_TEST_RING.take_ring(128) else {
        return false;
    };
    match ring.parts_mut() {
        Ok((nodes, dma_buf)) if verify_circular_ring(nodes, dma_buf, 128).is_ok() => {}
        _ => return false,
    }
    let ring_cfg = esp32_cam_ll::DmaSizeConfig {
        dma_bytes_per_item: 4,
        dma_buffer_size: 512,
        dma_half_buffer_size: 256,
        dma_half_buffer_cnt: 2,
        dma_node_buffer_size: 128,
        dma_node_cnt: 4,
        lines_per_node: 1,
        nodes_per_line: 1,
        lines_per_half_buffer: 1,
        image_size: 512,
    };

    i2s_cam.vsync_intr_enable(true);
    if !i2s_cam.is_vsync_intr_enabled() {
        return false;
    }
    let started = match i2s_cam.start(&ring_cfg, &mut ring) {
        Ok(running) => {
            let ok = running.is_running();
            // Dropping the guard would stop the DMA too; stop explicitly.
            running.stop();
            ok
        }
        Err(_) => false,
    };
    let stopped = !i2s_cam.is_running() && ring.parts_mut().is_ok();
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
        let _ = w.write_str_checked("\r\nContent-Length: 96\r\n\r\n");
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
        && runtime.server.last_ota_bytes_written() == ota_payload.len()
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

/// Nesting-safe, epoch-checked Core 1 pause/resume handshake state machine
/// (`M-019` / `C3-003`).
///
/// Uses an `AtomicU32` nesting depth and monotonic non-zero handshake epochs
/// (`req_epoch` / `ack_epoch`) plus a sticky fault latch on resume timeout to
/// prevent premature unpark on nested guards, ABA races, and sticky requests.
#[cfg_attr(not(target_arch = "xtensa"), allow(dead_code))]
struct Core1PauseState {
    depth: AtomicU32,
    req_epoch: AtomicU32,
    ack_epoch: AtomicU32,
    faulted: AtomicBool,
}

#[cfg_attr(not(target_arch = "xtensa"), allow(dead_code))]
impl Core1PauseState {
    const fn new() -> Self {
        Self {
            depth: AtomicU32::new(0),
            req_epoch: AtomicU32::new(0),
            ack_epoch: AtomicU32::new(0),
            faulted: AtomicBool::new(false),
        }
    }

    #[cfg_attr(target_arch = "xtensa", esp_hal::ram)]
    fn is_pause_requested(&self) -> bool {
        self.depth.load(Ordering::Acquire) > 0
    }

    #[cfg_attr(target_arch = "xtensa", esp_hal::ram)]
    fn req_epoch(&self) -> u32 {
        self.req_epoch.load(Ordering::Acquire)
    }

    #[cfg(test)]
    #[inline]
    fn ack_epoch(&self) -> u32 {
        self.ack_epoch.load(Ordering::Acquire)
    }

    #[cfg_attr(target_arch = "xtensa", esp_hal::ram)]
    fn is_acknowledged(&self) -> bool {
        self.ack_epoch.load(Ordering::Acquire) != 0
    }

    #[cfg(test)]
    #[inline]
    fn is_faulted(&self) -> bool {
        self.faulted.load(Ordering::Acquire)
    }

    #[cfg_attr(target_arch = "xtensa", esp_hal::ram)]
    fn ack_paused(&self, epoch: u32) {
        self.ack_epoch.store(epoch, Ordering::Release);
    }

    #[cfg_attr(target_arch = "xtensa", esp_hal::ram)]
    fn clear_ack(&self) {
        self.ack_epoch.store(0, Ordering::Release);
    }

    /// Acquires one pause nesting level. On the `0 -> 1` transition when
    /// `core1_active` is true, verifies `ack_epoch == 0` and bumps `req_epoch`
    /// BEFORE publishing `depth: 0 -> 1` (`Release`) so Core 1's `Acquire`
    /// load of `depth` always observes the new epoch (`M-019`). Rolls back
    /// `depth` on timeout so a failed pause never strands Core 1 deadlocked.
    #[cfg_attr(target_arch = "xtensa", esp_hal::ram)]
    fn begin_pause(&self, core1_active: bool, max_spins: u32, mut feed_wdt: impl FnMut()) -> bool {
        if self.faulted.load(Ordering::Acquire) {
            return false;
        }
        loop {
            let cur = self.depth.load(Ordering::Acquire);
            if cur == u32::MAX {
                return false;
            }
            if !core1_active {
                if self
                    .depth
                    .compare_exchange_weak(cur, cur + 1, Ordering::AcqRel, Ordering::Acquire)
                    .is_ok()
                {
                    return true;
                }
                continue;
            }

            if cur > 0 {
                let req = self.req_epoch.load(Ordering::Acquire);
                let ack = self.ack_epoch.load(Ordering::Acquire);
                if req == 0 || ack != req {
                    return false;
                }
                if self
                    .depth
                    .compare_exchange_weak(cur, cur + 1, Ordering::AcqRel, Ordering::Acquire)
                    .is_ok()
                {
                    return true;
                }
                continue;
            }

            // Outermost 0 -> 1 transition: verify no stale ACK remains from an
            // earlier cycle and bump `req_epoch` BEFORE publishing `depth = 1`
            // so Core 1 can never observe `depth > 0` with a stale epoch.
            if self.ack_epoch.load(Ordering::Acquire) != 0 {
                self.faulted.store(true, Ordering::Release);
                return false;
            }

            let mut next_epoch = self
                .req_epoch
                .fetch_add(1, Ordering::AcqRel)
                .wrapping_add(1);
            if next_epoch == 0 {
                next_epoch = self
                    .req_epoch
                    .fetch_add(1, Ordering::AcqRel)
                    .wrapping_add(1);
            }

            if self
                .depth
                .compare_exchange(0, 1, Ordering::AcqRel, Ordering::Acquire)
                .is_err()
            {
                return false;
            }

            for spin in 0..max_spins {
                if self.ack_epoch.load(Ordering::Acquire) == next_epoch {
                    return true;
                }
                if (spin & 0xFFF) == 0 {
                    feed_wdt();
                }
                core::hint::spin_loop();
            }

            // Pause timed out: roll back depth so Core 1 is never left stranded.
            self.depth.fetch_sub(1, Ordering::AcqRel);
            return false;
        }
    }

    /// Releases one pause nesting level. Only when depth drops `1 -> 0` does
    /// Core 1 unpark; waits up to `max_spins` for `ack_epoch == 0` and latches
    /// `faulted = true` on timeout so subsequent flash ops are refused.
    #[cfg_attr(target_arch = "xtensa", esp_hal::ram)]
    fn end_pause(&self, core1_active: bool, max_spins: u32, mut feed_wdt: impl FnMut()) -> bool {
        let prev_depth = loop {
            let cur = self.depth.load(Ordering::Acquire);
            if cur == 0 {
                // Already unpaused (e.g. cleanup resume after timeout rollback).
                return true;
            }
            if self
                .depth
                .compare_exchange_weak(cur, cur - 1, Ordering::AcqRel, Ordering::Acquire)
                .is_ok()
            {
                break cur;
            }
        };

        if prev_depth > 1 || !core1_active {
            return true;
        }

        for spin in 0..max_spins {
            if self.ack_epoch.load(Ordering::Acquire) == 0 {
                return true;
            }
            if (spin & 0xFFF) == 0 {
                feed_wdt();
            }
            core::hint::spin_loop();
        }

        self.faulted.store(true, Ordering::Release);
        false
    }
}

/// Port 80 wall-clock timeout constants (`M-082` / `C3-016`).
#[cfg_attr(not(target_arch = "xtensa"), allow(dead_code))]
const P80_TX_IDLE_TIMEOUT_MS: u64 = 4_000;
#[cfg_attr(not(target_arch = "xtensa"), allow(dead_code))]
const P80_TX_MAX_WALL_MS: u64 = 15_000;
#[cfg_attr(not(target_arch = "xtensa"), allow(dead_code))]
const P80_OTA_RX_IDLE_TIMEOUT_MS: u64 = 30_000;
#[cfg_attr(not(target_arch = "xtensa"), allow(dead_code))]
const P80_OTA_RX_MAX_WALL_MS: u64 = 120_000;
#[cfg_attr(not(target_arch = "xtensa"), allow(dead_code))]
const OTA_REBOOT_FORCE_TIMEOUT_MS: u64 = 5_000;

/// Returns `true` if a Port 80 TX response has exceeded either its sliding
/// idle deadline or the hard total wall-clock cap (`M-082` / `C3-016`).
#[cfg_attr(not(target_arch = "xtensa"), allow(dead_code))]
#[inline]
const fn p80_tx_deadline_exceeded(now_ms: u64, tx_started_ms: u64, close_deadline_ms: u64) -> bool {
    let idle_expired = close_deadline_ms > 0 && now_ms >= close_deadline_ms;
    let wall_expired =
        tx_started_ms > 0 && now_ms.saturating_sub(tx_started_ms) >= P80_TX_MAX_WALL_MS;
    idle_expired || wall_expired
}

/// Returns `true` if an OTA upload RX stream has exceeded either its sliding
/// idle timeout or the hard total wall-clock cap (`M-082` / `C3-016`).
#[cfg_attr(not(target_arch = "xtensa"), allow(dead_code))]
#[inline]
const fn p80_ota_rx_expired(now_ms: u64, last_rx_ms: u64, ota_started_ms: u64) -> bool {
    let idle_expired =
        last_rx_ms > 0 && now_ms.saturating_sub(last_rx_ms) > P80_OTA_RX_IDLE_TIMEOUT_MS;
    let wall_expired =
        ota_started_ms > 0 && now_ms.saturating_sub(ota_started_ms) >= P80_OTA_RX_MAX_WALL_MS;
    idle_expired || wall_expired
}

/// Maps ESP32 DROM/IROM MMU flash page indices to an [`OtaSlot`] using the
/// shared partition constants (`M-224` / `C3-063`).
#[cfg_attr(not(target_arch = "xtensa"), allow(dead_code))]
#[inline]
const fn decode_ota_slot_from_mmu_entries(drom_entry: u32, irom_entry: u32) -> Option<OtaSlot> {
    const OTA0_PAGE_START: u32 = klipper_cam_fw::OTA_0_OFFSET >> 16;
    const OTA0_PAGE_END: u32 = (klipper_cam_fw::OTA_0_OFFSET + klipper_cam_fw::OTA_0_SIZE) >> 16;
    const OTA1_PAGE_START: u32 = klipper_cam_fw::OTA_1_OFFSET >> 16;
    const OTA1_PAGE_END: u32 = (klipper_cam_fw::OTA_1_OFFSET + klipper_cam_fw::OTA_1_SIZE) >> 16;

    const fn classify(entry: u32) -> Option<OtaSlot> {
        if (entry & 0x100) != 0 {
            return None;
        }
        let page = entry & 0xFF;
        if page >= OTA1_PAGE_START && page < OTA1_PAGE_END {
            Some(OtaSlot::Ota1)
        } else if page >= OTA0_PAGE_START && page < OTA0_PAGE_END {
            Some(OtaSlot::Ota0)
        } else {
            None
        }
    }

    match (classify(drom_entry), classify(irom_entry)) {
        (Some(OtaSlot::Ota1), _) | (_, Some(OtaSlot::Ota1)) => Some(OtaSlot::Ota1),
        (Some(OtaSlot::Ota0), _) | (_, Some(OtaSlot::Ota0)) => Some(OtaSlot::Ota0),
        (None, None) => None,
    }
}

/// Idle fallback timeout in SoftAP Setup Mode when saved STA credentials exist
/// (`180_000 ms` = 3 minutes) so the board reboots and retries STA after a
/// power outage once the router recovers.
#[cfg_attr(not(target_arch = "xtensa"), allow(dead_code))]
const AP_FALLBACK_IDLE_RETRY_MS: u64 = 180_000;

/// Returns `true` when firmware should enter SoftAP Setup Mode immediately on
/// boot (before attempting STA association or starting the Core 1 camera).
#[cfg_attr(not(target_arch = "xtensa"), allow(dead_code))]
#[inline]
const fn should_enter_setup_on_boot(provisioning_ap: bool, has_sta_creds: bool) -> bool {
    provisioning_ap && !has_sta_creds
}

/// Returns `true` when SoftAP Setup Mode (entered as a STA fallback while
/// valid saved credentials exist) has been idle with zero associated clients
/// for at least [`AP_FALLBACK_IDLE_RETRY_MS`] (`180_000 ms`) and should reboot
/// to retry STA.
#[cfg_attr(not(target_arch = "xtensa"), allow(dead_code))]
#[inline]
const fn should_retry_sta_after_ap_idle(
    had_saved_creds: bool,
    associated_clients: usize,
    now_ms: u64,
    last_activity_ms: u64,
) -> bool {
    had_saved_creds
        && associated_clients == 0
        && now_ms.saturating_sub(last_activity_ms) >= AP_FALLBACK_IDLE_RETRY_MS
}

/// Outcome of inspecting an incoming Ethernet-II frame in SoftAP Setup Mode for
/// UDP DHCPv4 (port 67) or captive-portal DNS (port 53).
#[cfg_attr(not(target_arch = "xtensa"), allow(dead_code))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SetupL2UdpAction {
    /// Not a Setup Mode UDP DHCP/DNS packet; forward unchanged to `smoltcp`.
    PassToStack,
    /// Matched Setup Mode UDP port 67/53 but malformed or ignored; drop without
    /// forwarding to `smoltcp`.
    Drop,
    /// Constructed a complete Ethernet-II + IPv4 + UDP reply frame of `usize`
    /// bytes in `eth_tx_out`.
    Reply(usize),
}

/// Accumulates big-endian 16-bit words into a 32-bit one's-complement sum.
#[cfg_attr(not(target_arch = "xtensa"), allow(dead_code))]
#[inline]
fn internet_checksum_add_slice(mut sum: u32, data: &[u8]) -> u32 {
    let mut i = 0usize;
    while i + 1 < data.len() {
        sum += u32::from(u16::from_be_bytes([data[i], data[i + 1]]));
        i += 2;
    }
    if i < data.len() {
        sum += u32::from(u16::from_be_bytes([data[i], 0]));
    }
    sum
}

/// Folds a 32-bit one's-complement sum into a 16-bit RFC 791/768 checksum.
#[cfg_attr(not(target_arch = "xtensa"), allow(dead_code))]
#[inline]
const fn internet_checksum_fold(mut sum: u32) -> u16 {
    while (sum >> 16) != 0 {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }
    !(sum as u16)
}

/// Writes an Ethernet-II (14 B) + IPv4 (20 B) + UDP (8 B) header prefix into
/// `eth_tx_out[..42]` around a UDP payload already written at
/// `eth_tx_out[42..42 + payload_len]`.
#[cfg_attr(not(target_arch = "xtensa"), allow(dead_code))]
#[allow(clippy::too_many_arguments)]
fn finalize_udp_eth_reply(
    eth_tx_out: &mut [u8],
    dst_mac: [u8; 6],
    src_mac: [u8; 6],
    src_ip: [u8; 4],
    dst_ip: [u8; 4],
    src_port: u16,
    dst_port: u16,
    payload_len: usize,
    ip_id: u16,
) -> Option<usize> {
    const HDR_LEN: usize = 14 + 20 + 8;
    let total_frame_len = HDR_LEN.checked_add(payload_len)?;
    let ipv4_total_len = u16::try_from(20usize.checked_add(8)?.checked_add(payload_len)?).ok()?;
    let udp_len = u16::try_from(8usize.checked_add(payload_len)?).ok()?;
    if total_frame_len > eth_tx_out.len() {
        return None;
    }

    // Ethernet II header (14 B)
    eth_tx_out[0..6].copy_from_slice(&dst_mac);
    eth_tx_out[6..12].copy_from_slice(&src_mac);
    eth_tx_out[12] = 0x08;
    eth_tx_out[13] = 0x00;

    // IPv4 header (20 B)
    let ip = &mut eth_tx_out[14..34];
    ip[0] = 0x45; // Version=4, IHL=5
    ip[1] = 0x00; // DSCP/ECN
    ip[2..4].copy_from_slice(&ipv4_total_len.to_be_bytes());
    ip[4..6].copy_from_slice(&ip_id.to_be_bytes());
    ip[6..8].copy_from_slice(&[0x00, 0x00]); // Flags/FragOffset
    ip[8] = 64; // TTL
    ip[9] = 17; // Protocol = UDP
    ip[10..12].copy_from_slice(&[0x00, 0x00]); // Checksum placeholder
    ip[12..16].copy_from_slice(&src_ip);
    ip[16..20].copy_from_slice(&dst_ip);
    let ip_csum = internet_checksum_fold(internet_checksum_add_slice(0, ip));
    ip[10..12].copy_from_slice(&ip_csum.to_be_bytes());

    // UDP header (8 B)
    eth_tx_out[34..36].copy_from_slice(&src_port.to_be_bytes());
    eth_tx_out[36..38].copy_from_slice(&dst_port.to_be_bytes());
    eth_tx_out[38..40].copy_from_slice(&udp_len.to_be_bytes());
    eth_tx_out[40..42].copy_from_slice(&[0x00, 0x00]);

    // RFC 768 UDP checksum (pseudo-header + UDP header + payload)
    let mut pseudo = [0u8; 12];
    pseudo[0..4].copy_from_slice(&src_ip);
    pseudo[4..8].copy_from_slice(&dst_ip);
    pseudo[8] = 0;
    pseudo[9] = 17;
    pseudo[10..12].copy_from_slice(&udp_len.to_be_bytes());
    let mut udp_sum = internet_checksum_add_slice(0, &pseudo);
    udp_sum = internet_checksum_add_slice(udp_sum, &eth_tx_out[34..total_frame_len]);
    let raw_udp_csum = internet_checksum_fold(udp_sum);
    let udp_csum = if raw_udp_csum == 0 {
        0xFFFF
    } else {
        raw_udp_csum
    };
    eth_tx_out[40..42].copy_from_slice(&udp_csum.to_be_bytes());

    Some(total_frame_len)
}

/// Intercepts SoftAP Setup Mode UDP DHCPv4 (port 67) and captive-portal DNS
/// (port 53) frames at Layer 2 so `smoltcp` needs no extra `socket-udp`
/// feature and broadcast `0.0.0.0 -> 255.255.255.255` DHCP replies work
/// reliably without ARP state.
#[cfg_attr(not(target_arch = "xtensa"), allow(dead_code))]
fn process_setup_l2_udp_frame(
    eth_rx: &[u8],
    ap_mac: [u8; 6],
    ap_ip: [u8; 4],
    dhcp: &mut DhcpServer,
    ip_id: u16,
    eth_tx_out: &mut [u8],
) -> SetupL2UdpAction {
    const L2_UDP_HDR_LEN: usize = 14 + 20 + 8;
    if eth_rx.len() < L2_UDP_HDR_LEN || eth_tx_out.len() < L2_UDP_HDR_LEN {
        return SetupL2UdpAction::PassToStack;
    }
    if eth_rx[12] != 0x08 || eth_rx[13] != 0x00 {
        return SetupL2UdpAction::PassToStack;
    }

    let client_mac = [
        eth_rx[6], eth_rx[7], eth_rx[8], eth_rx[9], eth_rx[10], eth_rx[11],
    ];
    let ip = &eth_rx[14..];
    let ver = ip[0] >> 4;
    let ihl = usize::from(ip[0] & 0x0F) * 4;
    if ver != 4 || ihl < 20 || ip.len() < ihl + 8 {
        return SetupL2UdpAction::PassToStack;
    }
    let total_len = usize::from(u16::from_be_bytes([ip[2], ip[3]]));
    if total_len < ihl + 8 || total_len > ip.len() {
        return SetupL2UdpAction::PassToStack;
    }
    let flags_frag = u16::from_be_bytes([ip[6], ip[7]]);
    if (flags_frag & 0x3FFF) != 0 || ip[9] != 17 {
        return SetupL2UdpAction::PassToStack;
    }

    let src_ip = [ip[12], ip[13], ip[14], ip[15]];
    let dst_ip = [ip[16], ip[17], ip[18], ip[19]];
    let udp = &ip[ihl..total_len];
    let src_port = u16::from_be_bytes([udp[0], udp[1]]);
    let dst_port = u16::from_be_bytes([udp[2], udp[3]]);
    let udp_len = usize::from(u16::from_be_bytes([udp[4], udp[5]]));
    if udp_len < 8 || udp_len > udp.len() {
        return SetupL2UdpAction::PassToStack;
    }
    let udp_payload = &udp[8..udp_len];

    if dst_port == DHCP_SERVER_PORT {
        if dst_ip != ap_ip && dst_ip != [255, 255, 255, 255] {
            return SetupL2UdpAction::PassToStack;
        }
        let Some(resp_len) = dhcp.handle_packet(udp_payload, &mut eth_tx_out[L2_UDP_HDR_LEN..])
        else {
            return SetupL2UdpAction::Drop;
        };
        let req_bcast = udp_payload.len() >= 12 && (udp_payload[10] & 0x80) != 0;
        let resp_bcast = resp_len >= 12 && (eth_tx_out[L2_UDP_HDR_LEN + 10] & 0x80) != 0;
        let (reply_dst_mac, reply_dst_ip) = if src_ip == [0, 0, 0, 0] || req_bcast || resp_bcast {
            ([0xFF; 6], [255, 255, 255, 255])
        } else {
            (client_mac, src_ip)
        };
        match finalize_udp_eth_reply(
            eth_tx_out,
            reply_dst_mac,
            ap_mac,
            ap_ip,
            reply_dst_ip,
            DHCP_SERVER_PORT,
            DHCP_CLIENT_PORT,
            resp_len,
            ip_id,
        ) {
            Some(frame_len) => SetupL2UdpAction::Reply(frame_len),
            None => SetupL2UdpAction::Drop,
        }
    } else if dst_port == DNS_PORT {
        if dst_ip != ap_ip && dst_ip != [255, 255, 255, 255] {
            return SetupL2UdpAction::PassToStack;
        }
        if src_ip == [0, 0, 0, 0] || src_port == 0 {
            return SetupL2UdpAction::Drop;
        }
        let Some(resp_len) =
            handle_captive_dns_query(udp_payload, ap_ip, &mut eth_tx_out[L2_UDP_HDR_LEN..])
        else {
            return SetupL2UdpAction::Drop;
        };
        match finalize_udp_eth_reply(
            eth_tx_out, client_mac, ap_mac, ap_ip, src_ip, DNS_PORT, src_port, resp_len, ip_id,
        ) {
            Some(frame_len) => SetupL2UdpAction::Reply(frame_len),
            None => SetupL2UdpAction::Drop,
        }
    } else {
        SetupL2UdpAction::PassToStack
    }
}

#[cfg(target_arch = "xtensa")]
#[repr(C, align(4))]
struct AlignedDmaBuf([u8; 32768]);

#[cfg(target_arch = "xtensa")]
struct I2sDmaBuffers {
    desc: [Lldesc; 16],
    buf: AlignedDmaBuf,
}

/// Atomic lease-guarded static storage for I2S0 DMA descriptors and buffer
/// (`M-077`, `M-080`).
#[cfg(target_arch = "xtensa")]
struct I2sDmaSlot {
    borrowed: AtomicBool,
    inner: UnsafeCell<I2sDmaBuffers>,
}

// JUSTIFICATION: Synchronize static I2S0 DMA descriptor ring across cores via
// an atomic single-owner lease guard.
// SAFETY:
// 1. Preconditions: Only one core may hold an `I2sDmaLease` at a time.
// 2. Alignment & Layout: `I2sDmaBuffers` is 4-byte aligned in internal DRAM.
// 3. Provenance & Lifetime: Static `.bss` storage lives for `'static`.
// 4. Aliasing: `borrowed` CAS guarantees unique `&mut` access per lease.
// 5. Synchronization: `AcqRel` CAS on acquire paired with `Release` on drop.
// 6. Panic Safety: Bare-metal firmware uses `panic = "abort"`.
#[cfg(target_arch = "xtensa")]
unsafe impl Sync for I2sDmaSlot {}

#[cfg(target_arch = "xtensa")]
struct I2sDmaLease<'a> {
    slot: &'a I2sDmaSlot,
}

#[cfg(target_arch = "xtensa")]
impl I2sDmaSlot {
    const fn new() -> Self {
        Self {
            borrowed: AtomicBool::new(false),
            inner: UnsafeCell::new(I2sDmaBuffers {
                desc: [Lldesc::EMPTY; 16],
                buf: AlignedDmaBuf([0u8; 32768]),
            }),
        }
    }

    #[esp_hal::ram]
    fn try_borrow(&self) -> Option<I2sDmaLease<'_>> {
        if self
            .borrowed
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
            .is_ok()
        {
            Some(I2sDmaLease { slot: self })
        } else {
            None
        }
    }
}

#[cfg(target_arch = "xtensa")]
impl I2sDmaLease<'_> {
    #[esp_hal::ram]
    fn parts_mut(&mut self) -> (&mut [Lldesc; 16], &mut [u8; 32768]) {
        // JUSTIFICATION: Provide unique mutable slices to leased DMA storage.
        // SAFETY:
        // 1. Preconditions: `I2sDmaLease` holds the atomic `borrowed` flag.
        // 2. Alignment: `desc` and `buf` are 4-byte aligned in internal DRAM.
        // 3. Provenance: `self.slot.inner` lives in static `.bss`.
        // 4. Aliasing: `&mut self` prevents overlapping borrows of this lease.
        // 5. Synchronization: Acquire CAS in `try_borrow`, Release in `Drop`.
        // 6. Panic Safety: Pure field projection cannot panic.
        let inner = unsafe { &mut *self.slot.inner.get() };
        (&mut inner.desc, &mut inner.buf.0)
    }
}

#[cfg(target_arch = "xtensa")]
impl Drop for I2sDmaLease<'_> {
    #[esp_hal::ram]
    fn drop(&mut self) {
        self.slot.borrowed.store(false, Ordering::Release);
    }
}

/// Single-shot Core 0 static cell eliminating unsynchronized `static mut`
/// globals (`M-080` / `C3-014`).
#[cfg(target_arch = "xtensa")]
struct Core0TakeCell<T> {
    taken: AtomicBool,
    value: UnsafeCell<T>,
}

// JUSTIFICATION: Static Core 0 singleton cell synchronized by atomic CAS.
// SAFETY:
// 1. Preconditions: `take()` enforces `Cpu::current() == Cpu::ProCpu`.
// 2. Alignment & Layout: Inline `UnsafeCell<T>` preserves alignment of `T`.
// 3. Provenance & Lifetime: Static `.bss` storage lives for `'static`.
// 4. Aliasing: Single-shot CAS grants at most one `&'static mut T`.
// 5. Synchronization: `AcqRel` CAS guarantees exclusive handoff.
// 6. Panic Safety: Bare-metal firmware uses `panic = "abort"`.
#[cfg(target_arch = "xtensa")]
unsafe impl<T> Sync for Core0TakeCell<T> {}

#[cfg(target_arch = "xtensa")]
impl<T> Core0TakeCell<T> {
    const fn new(val: T) -> Self {
        Self {
            taken: AtomicBool::new(false),
            value: UnsafeCell::new(val),
        }
    }

    #[allow(clippy::mut_from_ref)]
    fn take(&'static self) -> Option<&'static mut T> {
        if Cpu::current() != Cpu::ProCpu {
            return None;
        }
        if self
            .taken
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
            .is_err()
        {
            return None;
        }
        // JUSTIFICATION: Claim sole mutable reference after atomic CAS.
        // SAFETY:
        // 1. Preconditions: Running on Core 0 and `taken` transitioned 0 -> 1.
        // 2. Alignment: `self.value` is validly aligned for `T`.
        // 3. Provenance: `&'static self` backs `'static` storage.
        // 4. Aliasing: No other reference can ever be handed out.
        // 5. Synchronization: `AcqRel` CAS orders prior initialization.
        // 6. Panic Safety: No panic occurs between CAS and return.
        Some(unsafe { &mut *self.value.get() })
    }

    fn raw_ptr(&self) -> *const T {
        self.value.get().cast_const()
    }
}

/// Atomic state-guarded `Option<T>` singleton supporting boot-time `put`,
/// exclusive Core 0 `with_mut`, and single-shot cross-core ownership `take`
/// (`M-080` / `C3-014`).
#[cfg(target_arch = "xtensa")]
struct OnceOptionSlot<T> {
    // 0 = Empty, 1 = Locked/Busy, 2 = Ready
    state: AtomicU8,
    slot: UnsafeCell<Option<T>>,
}

// JUSTIFICATION: Synchronized singleton slot for owned cross-core handoff.
// SAFETY:
// 1. Preconditions: Every access transitions `state` via `AcqRel` CAS.
// 2. Alignment & Layout: `UnsafeCell<Option<T>>` preserves `Option<T>` layout.
// 3. Provenance & Lifetime: Static `.bss` storage lives for `'static`.
// 4. Aliasing: Only the holder of `state == 1` accesses `self.slot`.
// 5. Synchronization: `AcqRel` CAS + `Release` store orders `T` transfer.
// 6. Panic Safety: Bare-metal firmware uses `panic = "abort"`.
#[cfg(target_arch = "xtensa")]
unsafe impl<T: Send> Sync for OnceOptionSlot<T> {}

#[cfg(target_arch = "xtensa")]
impl<T: Send> OnceOptionSlot<T> {
    const fn new() -> Self {
        Self {
            state: AtomicU8::new(0),
            slot: UnsafeCell::new(None),
        }
    }

    fn put(&self, val: T) -> bool {
        if self
            .state
            .compare_exchange(0, 1, Ordering::AcqRel, Ordering::Acquire)
            .is_err()
        {
            return false;
        }
        // JUSTIFICATION: Store initial value while holding exclusive state=1.
        // SAFETY: CAS transitioned state 0 -> 1; no concurrent access exists.
        unsafe {
            *self.slot.get() = Some(val);
        }
        self.state.store(2, Ordering::Release);
        true
    }

    fn with_mut<R>(&self, f: impl FnOnce(&mut T) -> R) -> Option<R> {
        if self
            .state
            .compare_exchange(2, 1, Ordering::AcqRel, Ordering::Acquire)
            .is_err()
        {
            return None;
        }
        // JUSTIFICATION: Borrow stored value exclusively while state == 1.
        // SAFETY: CAS transitioned state 2 -> 1; restored to 2 before return.
        let res = unsafe { (*self.slot.get()).as_mut().map(f) };
        self.state.store(2, Ordering::Release);
        res
    }

    #[esp_hal::ram]
    fn take(&self) -> Option<T> {
        if self
            .state
            .compare_exchange(2, 1, Ordering::AcqRel, Ordering::Acquire)
            .is_err()
        {
            return None;
        }
        // JUSTIFICATION: Move owned value out while holding exclusive state=1.
        // SAFETY: CAS transitioned state 2 -> 1; slot becomes Empty (state=0).
        let val = unsafe { (*self.slot.get()).take() };
        self.state.store(0, Ordering::Release);
        val
    }
}

#[cfg(target_arch = "xtensa")]
static I2S_DMA_STORAGE: I2sDmaSlot = I2sDmaSlot::new();

#[cfg(target_arch = "xtensa")]
static APP_CORE_STACK: Core0TakeCell<Stack<8192>> = Core0TakeCell::new(Stack::new());

#[cfg(target_arch = "xtensa")]
static FW_STATS: FwStats = FwStats::new();

#[cfg(target_arch = "xtensa")]
static OTA_FLASH_ERRORS: AtomicU32 = AtomicU32::new(0);

#[cfg(target_arch = "xtensa")]
static CORE1_SYNC_TIMEOUTS: AtomicU32 = AtomicU32::new(0);

#[cfg(target_arch = "xtensa")]
static VSYNC_TIMEOUTS: AtomicU32 = AtomicU32::new(0);

#[cfg(target_arch = "xtensa")]
static CORE1_SPSC: Core1JpegSpsc<32> = Core1JpegSpsc::new();

#[cfg(target_arch = "xtensa")]
static CORE1_PAUSE_STATE: Core1PauseState = Core1PauseState::new();

#[cfg(target_arch = "xtensa")]
static CORE1_ACTIVE: AtomicBool = AtomicBool::new(false);

#[cfg(target_arch = "xtensa")]
static HALT_IN_PROGRESS: AtomicBool = AtomicBool::new(false);

/// Requests Core 1 to stop I2S0 capture and waits for epoch acknowledgement.
///
/// Nesting-safe (`AtomicU32` depth) and ABA-safe (monotonic epoch counter).
/// Refuses calls from `AppCpu` (`M-020`) and rolls back depth on timeout so
/// Core 1 is never stranded paused (`M-019`).
#[cfg(target_arch = "xtensa")]
#[esp_hal::ram]
fn pause_core1_camera_producer() -> bool {
    if Cpu::current() != Cpu::ProCpu {
        return false;
    }
    let ok = CORE1_PAUSE_STATE.begin_pause(
        CORE1_ACTIVE.load(Ordering::Acquire),
        2_000_000,
        feed_rtc_wdt,
    );
    if !ok {
        CORE1_SYNC_TIMEOUTS.fetch_add(1, Ordering::Relaxed);
    }
    ok
}

/// Decrements Core 1 pause depth and, on `1 -> 0`, waits for Core 1 to clear
/// its epoch acknowledgement (`M-019`). Latches a fault on timeout so future
/// flash operations are refused.
#[cfg(target_arch = "xtensa")]
#[esp_hal::ram]
fn resume_core1_camera_producer() -> bool {
    if Cpu::current() != Cpu::ProCpu {
        return false;
    }
    let ok = CORE1_PAUSE_STATE.end_pause(
        CORE1_ACTIVE.load(Ordering::Acquire),
        2_000_000,
        feed_rtc_wdt,
    );
    if !ok {
        CORE1_SYNC_TIMEOUTS.fetch_add(1, Ordering::Relaxed);
    }
    ok
}

/// Synchronously enforces `CS1_DIS = 1` / `CS0_DIS = 0` on `SPI1_PIN_REG`
/// with interrupts masked and SPI0/SPI1 idle polling (`M-172` / `C3-043`).
#[cfg(target_arch = "xtensa")]
#[esp_hal::ram]
fn ensure_spi1_flash_cs0_only() -> bool {
    const SPI0_EXT2_REG: *const u32 = 0x3FF4_30F8 as *const u32;
    const SPI1_CMD_REG: *const u32 = 0x3FF4_2000 as *const u32;
    const SPI1_EXT2_REG: *const u32 = 0x3FF4_20F8 as *const u32;
    const SPI1_PIN_REG: *mut u32 = 0x3FF4_2034 as *mut u32;
    const SPI_USR: u32 = 1u32 << 18;
    // JUSTIFICATION: Mask IRQs and wait for SPI0/SPI1 idle before RMW on
    // `SPI1_PIN_REG` so concurrent ISRs cannot race on SPI1 CS selection.
    // SAFETY:
    // 1. Preconditions: Called on Core 0 with Core 1 parked before flash I/O.
    // 2. Alignment: All SPI0/SPI1 addresses are 4-byte aligned MMIO.
    // 3. Provenance: Fixed ESP32 peripheral MMIO aperture.
    // 4. Aliasing: Volatile MMIO reads/writes form no Rust references.
    // 5. Synchronization: `rsil 15` masks interrupts; `wsr.ps` restores them.
    // 6. Panic Safety: Bounded spin loop cannot panic or unwind.
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
        let idle_ok = spin < 100_000;
        if idle_ok {
            let pin = core::ptr::read_volatile(SPI1_PIN_REG);
            // Enforce CS1_DIS = 1 (bit 1, PSRAM CS# disabled) and
            // CS0_DIS = 0 (bit 0, Flash CS# enabled).
            core::ptr::write_volatile(SPI1_PIN_REG, (pin | (1u32 << 1)) & !1u32);
        }
        core::arch::asm!("wsr.ps {0}; rsync", in(reg) saved_ps);
        idle_ok
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
        klipper_cam_fw::Core1PauseGuard::with_core1_paused(&XtensaCore1Park, || {
            if !ensure_spi1_flash_cs0_only() {
                OTA_FLASH_ERRORS.fetch_add(1, Ordering::Relaxed);
                return Err(klipper_cam_fw::OtaError::FlashWriteError);
            }
            feed_rtc_wdt();
            // Note: `OtaError` has a single flash-I/O error variant
            // (`FlashWriteError`); log and count read errors distinctly.
            let res = self.0.read_nor(offset, buf).map_err(|_| {
                OTA_FLASH_ERRORS.fetch_add(1, Ordering::Relaxed);
                esp_println::println!(
                    "[KLIPPER-CAM-RS] [FLASH] read_nor failed @ 0x{:08X}",
                    offset
                );
                klipper_cam_fw::OtaError::FlashWriteError
            });
            feed_rtc_wdt();
            res
        })
    }

    fn erase_range(&mut self, from: u32, to: u32) -> Result<(), klipper_cam_fw::OtaError> {
        const ERASE_CHUNK_BYTES: u32 = 64 * 1024;
        klipper_cam_fw::Core1PauseGuard::with_core1_paused(&XtensaCore1Park, || {
            let mut cur = from;
            while cur < to {
                if !ensure_spi1_flash_cs0_only() {
                    OTA_FLASH_ERRORS.fetch_add(1, Ordering::Relaxed);
                    return Err(klipper_cam_fw::OtaError::FlashWriteError);
                }
                let next = cur.saturating_add(ERASE_CHUNK_BYTES).min(to);
                feed_rtc_wdt();
                self.0.erase(cur, next).map_err(|_| {
                    OTA_FLASH_ERRORS.fetch_add(1, Ordering::Relaxed);
                    klipper_cam_fw::OtaError::FlashWriteError
                })?;
                feed_rtc_wdt();
                cur = next;
            }
            Ok(())
        })
    }

    fn write_aligned(&mut self, offset: u32, data: &[u8]) -> Result<(), klipper_cam_fw::OtaError> {
        klipper_cam_fw::Core1PauseGuard::with_core1_paused(&XtensaCore1Park, || {
            if !ensure_spi1_flash_cs0_only() {
                OTA_FLASH_ERRORS.fetch_add(1, Ordering::Relaxed);
                return Err(klipper_cam_fw::OtaError::FlashWriteError);
            }
            feed_rtc_wdt();
            let res = self.0.write_nor(offset, data).map_err(|_| {
                OTA_FLASH_ERRORS.fetch_add(1, Ordering::Relaxed);
                klipper_cam_fw::OtaError::FlashWriteError
            });
            feed_rtc_wdt();
            res
        })
    }
}

#[cfg(target_arch = "xtensa")]
static BOOT_RESET_REASON: AtomicU32 = AtomicU32::new(0);

/// Atomic once-guarded storage for the booted image's 64-byte SHA-256 hex
/// digest (`M-084` / `C3-018`).
#[cfg(target_arch = "xtensa")]
struct BootSha256Cell {
    // 0 = uninitialized, 1 = initializing, 2 = published immutable
    state: AtomicU8,
    buf: UnsafeCell<[u8; 64]>,
}

// JUSTIFICATION: Share one-time boot SHA-256 hex digest across static storage.
// SAFETY:
// 1. Preconditions: `state` transitions `0 -> 1 -> 2` via `AcqRel` CAS.
// 2. Alignment & Layout: `[u8; 64]` has 1-byte alignment in `.bss`.
// 3. Provenance & Lifetime: Static storage lives for `'static`.
// 4. Aliasing: Shared borrows form only after `state == 2` (`Acquire`);
//    no mutation is ever possible once `state != 0`.
// 5. Synchronization: Release store of `state = 2` pairs with Acquire load.
// 6. Panic Safety: Bare-metal firmware uses `panic = "abort"`.
#[cfg(target_arch = "xtensa")]
unsafe impl Sync for BootSha256Cell {}

#[cfg(target_arch = "xtensa")]
static APP_SHA256_CELL: BootSha256Cell = BootSha256Cell {
    state: AtomicU8::new(0),
    buf: UnsafeCell::new([0u8; 64]),
};

#[cfg(target_arch = "xtensa")]
fn init_boot_app_sha256(flash: &mut EspFlashAdapter<'_>, slot: OtaSlot) {
    use klipper_cam_fw::OtaFlashStorage as _;

    if APP_SHA256_CELL.state.load(Ordering::Acquire) != 0 {
        return;
    }

    #[repr(C, align(4))]
    struct AlignedFlashBuf32([u8; 32]);

    let (base_off, part_size): (u32, u32) = match slot {
        OtaSlot::Ota0 => (OTA_0_OFFSET, OTA_0_SIZE),
        OtaSlot::Ota1 => (OTA_1_OFFSET, OTA_1_SIZE),
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
        if data_len > part_size {
            return;
        }
        let Some(after_hdr) = pos.checked_add(8) else {
            return;
        };
        let Some(after_seg) = after_hdr.checked_add(data_len) else {
            return;
        };
        if after_seg > part_size {
            return;
        }
        pos = after_seg;
        seg_idx += 1;
    }

    let Some(sha_pre) = pos.checked_add(16) else {
        return;
    };
    let sha_off = sha_pre & !15;
    let Some(sha_end) = sha_off.checked_add(32) else {
        return;
    };
    if sha_end > part_size {
        return;
    }
    let Some(abs_sha_off) = base_off.checked_add(sha_off) else {
        return;
    };
    let mut digest_buf = AlignedFlashBuf32([0u8; 32]);
    if flash.read_aligned(abs_sha_off, &mut digest_buf.0).is_err() {
        return;
    }

    if APP_SHA256_CELL
        .state
        .compare_exchange(0, 1, Ordering::AcqRel, Ordering::Acquire)
        .is_err()
    {
        return;
    }
    // JUSTIFICATION: Populate static 64-byte SHA-256 hex buffer exactly once
    // while holding exclusive `state == 1` via `compare_exchange`.
    // SAFETY:
    // 1. Preconditions: `APP_SHA256_CELL.state` transitioned `0 -> 1` once.
    // 2. Alignment: `[u8; 64]` has alignment 1 and is statically allocated.
    // 3. Provenance & Lifetime: `APP_SHA256_CELL` lives in `.bss` for `'static`.
    // 4. Aliasing: No shared references can be formed until `state == 2`.
    // 5. Volatile Semantics: Published via `Ordering::Release` store of `2`.
    // 6. Panic Safety: `format_sha256_hex` writes bounded indices `0..64`.
    let ok = unsafe {
        let hex_out = &mut *APP_SHA256_CELL.buf.get();
        format_sha256_hex(&digest_buf.0, hex_out).len() == 64
    };
    APP_SHA256_CELL
        .state
        .store(if ok { 2 } else { 0 }, Ordering::Release);
}

#[cfg(target_arch = "xtensa")]
fn boot_app_sha256_str() -> &'static str {
    if APP_SHA256_CELL.state.load(Ordering::Acquire) != 2 {
        return "unknown";
    }
    // JUSTIFICATION: Borrow immutable `'static` SHA-256 hex buffer after
    // atomic `state == 2` publication.
    // SAFETY:
    // 1. Preconditions: `APP_SHA256_CELL.state` observed as `2` via `Acquire`.
    // 2. Alignment: `[u8; 64]` has 1-byte alignment.
    // 3. Provenance & Lifetime: Static `.bss` buffer lives for `'static`.
    // 4. Aliasing: Never mutated again once `state == 2`.
    // 5. Volatile Semantics: Synchronized via Acquire/Release `AtomicU8`.
    // 6. Panic Safety: Fallible `from_utf8` match cannot panic or unwind.
    let bytes: &'static [u8; 64] = unsafe { &*APP_SHA256_CELL.buf.get() };
    core::str::from_utf8(bytes).unwrap_or("unknown")
}

#[cfg(target_arch = "xtensa")]
#[esp_hal::ram]
fn detect_booted_ota_manager_from_mmu() -> Option<OtaSlot> {
    const DPORT_PRO_FLASH_MMU_TABLE: *const u32 = 0x3FF1_0000 as *const u32;
    // ESP32 Errata 3.10: DPORT reads require single-core execution or peer
    // stall and masked interrupts.
    if Cpu::current() != Cpu::ProCpu || CORE1_ACTIVE.load(Ordering::Acquire) {
        return None;
    }
    // JUSTIFICATION: Read hardware MMU table entries for DROM and IROM to
    // identify which OTA flash partition (`ota_0` vs `ota_1`) is mapped, with
    // zero SPI NOR flash commands during early cold boot.
    // SAFETY:
    // 1. Preconditions: Verified single-core `ProCpu` execution with IRQs off.
    // 2. Alignment: `0x3FF1_0000` and `+77 * 4` are 4-byte aligned `u32` MMIO.
    // 3. Provenance & Lifetime: Fixed ESP32 DPORT MMIO aperture.
    // 4. Aliasing: Read-only volatile MMIO inspection forms no Rust references.
    // 5. Synchronization: Single-core read with interrupts masked (`rsil 15`).
    // 6. Unwind safety: Volatile register reads cannot panic or unwind.
    let (drom_entry, irom_entry) = unsafe {
        let saved_ps: u32;
        core::arch::asm!("rsil {0}, 15", out(reg) saved_ps);
        let drom = core::ptr::read_volatile(DPORT_PRO_FLASH_MMU_TABLE);
        let irom = core::ptr::read_volatile(DPORT_PRO_FLASH_MMU_TABLE.wrapping_add(77));
        core::arch::asm!("wsr.ps {0}; rsync", in(reg) saved_ps);
        (drom, irom)
    };
    decode_ota_slot_from_mmu_entries(drom_entry, irom_entry)
}

#[cfg(target_arch = "xtensa")]
#[inline(never)]
fn load_boot_otadata_and_sha256(
    flash_adapter: &mut EspFlashAdapter<'_>,
    mmu_booted_slot: Option<OtaSlot>,
) {
    use esp_println::println;

    let active_ota_slot = match klipper_cam_fw::OtaDataManager::load_from_flash(flash_adapter) {
        Ok(loaded) => {
            let flash_slot = loaded.active_slot();
            if mmu_booted_slot.is_some_and(|s| s != flash_slot) {
                println!(
                    "[KLIPPER-CAM-RS] [OTA] Warning: otadata slot {:?} != MMU {:?}; flash wins",
                    flash_slot, mmu_booted_slot
                );
            }
            let _ = FIRMWARE_RUNTIME.with_mut(|rt| {
                rt.server.ota_manager = loaded;
            });
            flash_slot
        }
        Err(err) => {
            OTA_FLASH_ERRORS.fetch_add(1, Ordering::Relaxed);
            let fallback = mmu_booted_slot.unwrap_or_else(|| {
                println!("[KLIPPER-CAM-RS] [OTA] Warning: MMU slot unknown; defaulting to Ota0");
                OtaSlot::Ota0
            });
            println!(
                "[KLIPPER-CAM-RS] [OTA] Failed to load otadata ({:?}); using MMU {:?}",
                err, fallback
            );
            fallback
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
    const DPORT_WIFI_CLK_EN_REG: *mut u32 = 0x3FF0_00CC as *mut u32;
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
            core::ptr::write_volatile(DPORT_WIFI_CLK_EN_REG, u32::MAX);
            let _ = core::ptr::read_volatile(DPORT_WIFI_CLK_EN_REG);
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
fn send_arp_announce(
    net_device: &mut Esp32StaWifiDriver<HardwareWifiMmioBus>,
    mac: [u8; 6],
    src_ip: [u8; 4],
    target_ip: [u8; 4],
) {
    use smoltcp::phy::{Device as _, TxToken as _};

    if let Some(tx_tok) = net_device.transmit(smol_now()) {
        tx_tok.consume(42, |buf| {
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
    net_device: &mut Esp32StaWifiDriver<HardwareWifiMmioBus>,
    sockets: &mut smoltcp::iface::SocketSet<'_>,
) {
    let t0 = ccount();
    for _ in 0..16u8 {
        let now = smol_now();
        let _ = net_device.poll(u64::try_from(now.total_millis()).unwrap_or(0));
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

/// Suspends PRO CPU SPI0 flash/PSRAM cache if enabled, returning prior enable
/// state (`M-083` / `C3-017`). Caller must execute from IRAM with IRQs masked.
#[cfg(target_arch = "xtensa")]
#[esp_hal::ram]
fn suspend_pro_spi_cache() -> Option<bool> {
    const DPORT_PRO_CACHE_CTRL_REG: *mut u32 = 0x3FF0_0040 as *mut u32;
    const SPI0_EXT2_REG: *const u32 = 0x3FF4_30F8 as *const u32;
    const PRO_CACHE_ENABLE: u32 = 1u32 << 3;
    // JUSTIFICATION: Suspend PRO CPU SPI0 cache before SPI1 PSRAM commands so
    // cache fills cannot contend with SPI1 QPI mode transitions.
    // SAFETY: Runs from IRAM with interrupts masked and single-core execution.
    unsafe {
        let mut spin = 0u32;
        while (core::ptr::read_volatile(SPI0_EXT2_REG) & 0x7) != 0 && spin < 100_000 {
            spin = spin.wrapping_add(1);
        }
        if spin >= 100_000 {
            return None;
        }
        let ctrl = core::ptr::read_volatile(DPORT_PRO_CACHE_CTRL_REG);
        let was_enabled = (ctrl & PRO_CACHE_ENABLE) != 0;
        if was_enabled {
            core::ptr::write_volatile(DPORT_PRO_CACHE_CTRL_REG, ctrl & !PRO_CACHE_ENABLE);
            let _ = core::ptr::read_volatile(DPORT_PRO_CACHE_CTRL_REG);
        }
        Some(was_enabled)
    }
}

/// Re-enables PRO CPU SPI0 cache (with flush) if `was_enabled` is true.
#[cfg(target_arch = "xtensa")]
#[esp_hal::ram]
fn resume_pro_spi_cache(was_enabled: bool) {
    if !was_enabled {
        return;
    }
    const DPORT_PRO_CACHE_CTRL_REG: *mut u32 = 0x3FF0_0040 as *mut u32;
    const PRO_CACHE_ENABLE: u32 = 1u32 << 3;
    const PRO_CACHE_FLUSH_ENA: u32 = 1u32 << 4;
    const PRO_CACHE_FLUSH_DONE: u32 = 1u32 << 5;
    // JUSTIFICATION: Flush and re-enable PRO CPU SPI0 cache after SPI1 PSRAM
    // transition completes.
    // SAFETY: Runs from IRAM with interrupts masked; MMIO address is valid.
    unsafe {
        let ctrl = core::ptr::read_volatile(DPORT_PRO_CACHE_CTRL_REG);
        core::ptr::write_volatile(DPORT_PRO_CACHE_CTRL_REG, ctrl & !PRO_CACHE_FLUSH_ENA);
        core::ptr::write_volatile(DPORT_PRO_CACHE_CTRL_REG, ctrl | PRO_CACHE_FLUSH_ENA);
        let mut spin = 0u32;
        while (core::ptr::read_volatile(DPORT_PRO_CACHE_CTRL_REG) & PRO_CACHE_FLUSH_DONE) == 0
            && spin < 100_000
        {
            spin = spin.wrapping_add(1);
        }
        let after = core::ptr::read_volatile(DPORT_PRO_CACHE_CTRL_REG);
        core::ptr::write_volatile(
            DPORT_PRO_CACHE_CTRL_REG,
            (after & !PRO_CACHE_FLUSH_ENA) | PRO_CACHE_ENABLE,
        );
        let _ = core::ptr::read_volatile(DPORT_PRO_CACHE_CTRL_REG);
    }
}

/// Restores `GPIO_ENABLE` bits 16 and 17 to their saved pre-transaction values
/// (`M-085` / `C3-019`) via the `esp-hal` GPIO PAC.
#[cfg(target_arch = "xtensa")]
#[esp_hal::ram]
fn restore_gpio16_17_enable(saved_gpio_en: u32) {
    let mask = (1u32 << 16) | (1u32 << 17);
    let to_clear = mask & !saved_gpio_en;
    let to_set = mask & saved_gpio_en;
    let gpio = esp_hal::peripherals::GPIO::regs();
    if to_clear != 0 {
        // JUSTIFICATION: Restore GPIO16/17 output-enable bits after SPI1 op.
        // SAFETY: Writing only bits 16-17 to W1TC affects only PSRAM CS/CLK.
        gpio.enable_w1tc().write(|w| unsafe { w.bits(to_clear) });
    }
    if to_set != 0 {
        // JUSTIFICATION: Restore GPIO16/17 output-enable bits after SPI1 op.
        // SAFETY: Writing only bits 16-17 to W1TS affects only PSRAM CS/CLK.
        gpio.enable_w1ts().write(|w| unsafe { w.bits(to_set) });
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

    // Require single-core execution, parked Core 1, or active halt guard
    // (ESP32 DPORT Errata 3.10 + `M-083`).
    if CORE1_ACTIVE.load(Ordering::Acquire)
        && !CORE1_PAUSE_STATE.is_acknowledged()
        && !HALT_IN_PROGRESS.load(Ordering::Acquire)
    {
        return false;
    }

    // JUSTIFICATION: Hardware SPI1 peripheral registers for PSRAM QPI command.
    // SAFETY: Disables interrupts and SPI0 cache during SPI1 bus transaction,
    // aborts cleanly on busy timeout, and restores saved registers and PS.
    unsafe {
        let saved_ps: u32;
        core::arch::asm!("rsil {0}, 15", out(reg) saved_ps);

        let Some(cache_was_enabled) = suspend_pro_spi_cache() else {
            core::arch::asm!("wsr.ps {0}; rsync", in(reg) saved_ps);
            return false;
        };

        let mut spin = 0u32;
        while ((core::ptr::read_volatile(SPI0_EXT2_REG) & 0x7) != 0
            || (core::ptr::read_volatile(SPI1_EXT2_REG) & 0x7) != 0
            || (core::ptr::read_volatile(SPI1_CMD_REG) & SPI_USR) != 0)
            && spin < 100_000
        {
            spin = spin.wrapping_add(1);
        }
        if spin >= 100_000 {
            resume_pro_spi_cache(cache_was_enabled);
            core::arch::asm!("wsr.ps {0}; rsync", in(reg) saved_ps);
            return false;
        }

        let saved_host_inf = core::ptr::read_volatile(DPORT_HOST_INF_SEL_REG);
        let saved_gpio_en = esp_hal::peripherals::GPIO::regs().enable().read().bits();
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
            // Matches esp_psram_impl_quad.c:497-535:
            // 8-bit 0x35 command in SPI1_ADDR_REG[31:24] via USR_ADDR (1-bit).
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

        let completed = if wait0 < 100_000 {
            core::ptr::write_volatile(DPORT_HOST_INF_SEL_REG, saved_host_inf | (1u32 << 14));
            core::ptr::write_volatile(SPI1_CMD_REG, SPI_USR);

            let mut wait1 = 0u32;
            while ((core::ptr::read_volatile(SPI1_CMD_REG) & SPI_USR) != 0
                || (core::ptr::read_volatile(SPI1_EXT2_REG) & 0x7) != 0)
                && wait1 < 100_000
            {
                wait1 = wait1.wrapping_add(1);
            }

            core::ptr::write_volatile(DPORT_HOST_INF_SEL_REG, saved_host_inf);
            wait1 < 100_000
        } else {
            false
        };

        core::ptr::write_volatile(SPI1_CTRL_REG, saved_ctrl);
        core::ptr::write_volatile(SPI1_CLOCK_REG, saved_clock);
        core::ptr::write_volatile(SPI1_USER_REG, saved_user);
        core::ptr::write_volatile(SPI1_USER1_REG, saved_user1);
        core::ptr::write_volatile(SPI1_USER2_REG, saved_user2);
        core::ptr::write_volatile(SPI1_MOSI_DLEN_REG, saved_mosi_dlen);
        core::ptr::write_volatile(SPI1_MISO_DLEN_REG, saved_miso_dlen);
        core::ptr::write_volatile(SPI1_W0_REG, saved_w0);
        // Enforce CS1_DIS = 1 (bit 1) and CS0_DIS = 0 (bit 0) on restore.
        core::ptr::write_volatile(SPI1_PIN_REG, (saved_pin | (1u32 << 1)) & !1u32);
        core::ptr::write_volatile(SPI1_ADDR_REG, saved_addr);
        core::ptr::write_volatile(GPIO_FUNC16_OUT_SEL_CFG_REG, saved_outsel16);
        core::ptr::write_volatile(IO_MUX_GPIO16_REG, saved_iomux16);
        core::ptr::write_volatile(GPIO_FUNC17_OUT_SEL_CFG_REG, saved_outsel17);
        core::ptr::write_volatile(IO_MUX_GPIO17_REG, saved_iomux17);
        restore_gpio16_17_enable(saved_gpio_en);
        core::ptr::write_volatile(SPI0_PIN_REG, saved_spi0_pin);

        resume_pro_spi_cache(cache_was_enabled);
        core::arch::asm!("wsr.ps {0}; rsync", in(reg) saved_ps);
        completed
    }
}

/// Issues a 1-bit SPI `0x9F` (`PSRAM_DEVICE_ID`) dummy read over `SPI1`
/// (`esp_psram_impl_quad.c:467-495, 983-988`), returning `true` on completion.
#[cfg(target_arch = "xtensa")]
#[esp_hal::ram]
fn psram_spi1_dummy_read_id() -> bool {
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

    if CORE1_ACTIVE.load(Ordering::Acquire)
        && !CORE1_PAUSE_STATE.is_acknowledged()
        && !HALT_IN_PROGRESS.load(Ordering::Acquire)
    {
        return false;
    }

    // JUSTIFICATION: Hardware SPI1 peripheral registers for PSRAM 0x9F ID read.
    // SAFETY: Disables interrupts and SPI0 cache during SPI1 bus transaction,
    // aborts cleanly on busy timeout, and restores saved registers and PS.
    unsafe {
        let saved_ps: u32;
        core::arch::asm!("rsil {0}, 15", out(reg) saved_ps);

        let Some(cache_was_enabled) = suspend_pro_spi_cache() else {
            core::arch::asm!("wsr.ps {0}; rsync", in(reg) saved_ps);
            return false;
        };

        let mut spin = 0u32;
        while ((core::ptr::read_volatile(SPI0_EXT2_REG) & 0x7) != 0
            || (core::ptr::read_volatile(SPI1_EXT2_REG) & 0x7) != 0
            || (core::ptr::read_volatile(SPI1_CMD_REG) & SPI_USR) != 0)
            && spin < 100_000
        {
            spin = spin.wrapping_add(1);
        }
        if spin >= 100_000 {
            resume_pro_spi_cache(cache_was_enabled);
            core::arch::asm!("wsr.ps {0}; rsync", in(reg) saved_ps);
            return false;
        }

        let saved_host_inf = core::ptr::read_volatile(DPORT_HOST_INF_SEL_REG);
        let saved_gpio_en = esp_hal::peripherals::GPIO::regs().enable().read().bits();
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

        let completed = if wait0 < 100_000 {
            core::ptr::write_volatile(DPORT_HOST_INF_SEL_REG, saved_host_inf | (1u32 << 14));
            core::ptr::write_volatile(SPI1_CMD_REG, SPI_USR);

            let mut wait1 = 0u32;
            while ((core::ptr::read_volatile(SPI1_CMD_REG) & SPI_USR) != 0
                || (core::ptr::read_volatile(SPI1_EXT2_REG) & 0x7) != 0)
                && wait1 < 100_000
            {
                wait1 = wait1.wrapping_add(1);
            }

            core::ptr::write_volatile(DPORT_HOST_INF_SEL_REG, saved_host_inf);
            wait1 < 100_000
        } else {
            false
        };

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
        core::ptr::write_volatile(IO_MUX_GPIO16_REG, saved_iomux16);
        core::ptr::write_volatile(GPIO_FUNC17_OUT_SEL_CFG_REG, saved_outsel17);
        core::ptr::write_volatile(IO_MUX_GPIO17_REG, saved_iomux17);
        restore_gpio16_17_enable(saved_gpio_en);
        core::ptr::write_volatile(SPI0_PIN_REG, saved_spi0_pin);

        resume_pro_spi_cache(cache_was_enabled);
        core::arch::asm!("wsr.ps {0}; rsync", in(reg) saved_ps);
        completed
    }
}

/// Atomically exits PSRAM QPI mode (`0xF5`) and holds PSRAM CS# high with
/// interrupts masked and SPI0 CS1 permanently disabled until reset (`M-022`).
#[cfg(target_arch = "xtensa")]
#[esp_hal::ram]
fn release_psram_cs_and_exit_qpi() {
    const SPI0_PIN_REG: *mut u32 = 0x3FF4_3034 as *mut u32;
    // JUSTIFICATION: Mask IRQs, suspend SPI0 cache, and keep SPI0 CS1 disabled
    // across `0xF5` QPI exit and GPIO16 CS# park without unmasking before reset.
    // SAFETY: Caller has halted/stalled the peer core and never returns to
    // flash-resident code after exiting QPI mode.
    unsafe {
        core::arch::asm!("rsil {0}, 15", out(reg) _);
        let _ = suspend_pro_spi_cache();
        let pin = core::ptr::read_volatile(SPI0_PIN_REG);
        core::ptr::write_volatile(SPI0_PIN_REG, pin | (1u32 << 1));
    }
    let _ = psram_spi1_cmd(true, 0xF5);
    hold_psram_cs_high_safe();
}

/// Ensures external PSRAM enters 4-bit QPI mode cleanly after `Psram::new()`,
/// keeping IRQs masked, SPI0 cache suspended, and SPI0 CS1 disabled across all
/// four SPI1 transactions and final GPIO16 `0x2B00` mux setup (`M-022`,
/// `M-083`, `M-085`).
#[cfg(target_arch = "xtensa")]
#[esp_hal::ram]
fn ensure_psram_qpi_mode_spi1() -> bool {
    const SPI0_PIN_REG: *mut u32 = 0x3FF4_3034 as *mut u32;
    const GPIO_ENABLE_W1TS_REG: *mut u32 = 0x3FF4_4024 as *mut u32;
    const GPIO_FUNC16_OUT_SEL_CFG_REG: *mut u32 = 0x3FF4_4570 as *mut u32;
    const IO_MUX_GPIO16_REG: *mut u32 = 0x3FF4_904C as *mut u32;
    const SPICS1_OUT_IDX: u32 = 6;
    // MCU_SEL=2 (GPIO matrix), FUN_DRV=2, FUN_IE=1, FUN_WPU=1, FUN_WPD=0.
    const IO_MUX_GPIO16_CS1_WPU: u32 = 0x2B00;

    // JUSTIFICATION: Hold IRQs masked, SPI0 cache suspended, and SPI0 CS1
    // disabled across all 4 PSRAM QPI transition commands and final mux write.
    // SAFETY: Executed from IRAM during single-core boot; all saved states are
    // restored before returning.
    unsafe {
        let saved_ps: u32;
        core::arch::asm!("rsil {0}, 15", out(reg) saved_ps);

        let Some(cache_was_enabled) = suspend_pro_spi_cache() else {
            core::arch::asm!("wsr.ps {0}; rsync", in(reg) saved_ps);
            return false;
        };
        let outer_spi0_pin = core::ptr::read_volatile(SPI0_PIN_REG);
        core::ptr::write_volatile(SPI0_PIN_REG, outer_spi0_pin | (1u32 << 1));

        let ok = psram_spi1_cmd(true, 0xF5)
            && psram_spi1_dummy_read_id()
            && psram_spi1_dummy_read_id()
            && psram_spi1_cmd(false, 0x35);

        core::ptr::write_volatile(GPIO_FUNC16_OUT_SEL_CFG_REG, SPICS1_OUT_IDX);
        core::ptr::write_volatile(IO_MUX_GPIO16_REG, IO_MUX_GPIO16_CS1_WPU);
        core::ptr::write_volatile(GPIO_ENABLE_W1TS_REG, (1u32 << 16) | (1u32 << 17));
        core::ptr::write_volatile(SPI0_PIN_REG, outer_spi0_pin);

        resume_pro_spi_cache(cache_was_enabled);
        core::arch::asm!("wsr.ps {0}; rsync", in(reg) saved_ps);
        ok
    }
}

/// Hardware-stalls the peer Xtensa CPU core via `RTC_CNTL` SW stall registers
/// (`M-020` / `C3-004`) so PSRAM QPI exit and reset never race with a running
/// peer core regardless of which CPU entered halt.
#[cfg(target_arch = "xtensa")]
#[esp_hal::ram]
fn stall_peer_cpu_hw() {
    let lpwr = esp_hal::peripherals::LPWR::regs();
    match Cpu::current() {
        Cpu::ProCpu => {
            lpwr.options0().modify(|_, w| {
                // JUSTIFICATION: Hardware-stall APP_CPU (Core 1) C0 field.
                // SAFETY: ESP32 TRM §31.3.13 stall sequence (c0=0x2) affects
                // only APP_CPU prior to full system software reset.
                unsafe { w.sw_stall_appcpu_c0().bits(0x2) }
            });
            lpwr.sw_cpu_stall().modify(|_, w| {
                // JUSTIFICATION: Hardware-stall APP_CPU (Core 1) C1 field.
                // SAFETY: ESP32 TRM §31.3.13 stall sequence (c1=0x21) affects
                // only APP_CPU prior to full system software reset.
                unsafe { w.sw_stall_appcpu_c1().bits(0x21) }
            });
        }
        Cpu::AppCpu => {
            lpwr.options0().modify(|_, w| {
                // JUSTIFICATION: Hardware-stall PRO_CPU (Core 0) C0 field.
                // SAFETY: ESP32 TRM §31.3.13 stall sequence (c0=0x2) affects
                // only PRO_CPU prior to full system software reset.
                unsafe { w.sw_stall_procpu_c0().bits(0x2) }
            });
            lpwr.sw_cpu_stall().modify(|_, w| {
                // JUSTIFICATION: Hardware-stall PRO_CPU (Core 0) C1 field.
                // SAFETY: ESP32 TRM §31.3.13 stall sequence (c1=0x21) affects
                // only PRO_CPU prior to full system software reset.
                unsafe { w.sw_stall_procpu_c1().bits(0x21) }
            });
        }
    }
    for _ in 0..256u32 {
        core::hint::spin_loop();
    }
}

#[cfg(target_arch = "xtensa")]
#[esp_hal::ram]
fn quiesce_psram_and_software_reset() -> ! {
    // JUSTIFICATION: Mask interrupts immediately at level 15 before single-entry
    // halt claim so no ISR can run during PSRAM quiesce or reset (`M-022`).
    // SAFETY: Inline assembly modifies only processor interrupt priority.
    unsafe {
        core::arch::asm!("rsil {0}, 15", out(reg) _);
    }

    // Single-entry atomic halt guard (`M-020` / `C3-004`): if the peer core
    // already claimed halt, spin quietly in IRAM with IRQs masked.
    if HALT_IN_PROGRESS.swap(true, Ordering::AcqRel) {
        loop {
            core::hint::spin_loop();
        }
    }

    // Symmetrically quiesce or hardware-stall the peer CPU before touching
    // I2S0 or PSRAM QPI state (`M-020`).
    match Cpu::current() {
        Cpu::ProCpu => {
            if CORE1_ACTIVE.load(Ordering::Acquire) && !pause_core1_camera_producer() {
                stall_peer_cpu_hw();
            }
        }
        Cpu::AppCpu => {
            stall_peer_cpu_hw();
        }
    }

    // JUSTIFICATION: Halt I2S0 DMA engine immediately before PSRAM pin release.
    // SAFETY:
    // 1. Alignment & Non-null: Direct MMIO register write via validated HAL.
    // 2. Layout: Target I2S0 peripheral registers exist on ESP32 silicon.
    // 3. Provenance: MMIO base address is mapped and valid for SoC lifecycle.
    // 4. Aliasing: Peer core is parked or stalled; exclusive hardware access.
    // 5. Volatile/Atomics: Peripheral registers are volatile MMIO.
    // 6. Unwind safety: C-ABI HAL function does not unwind or panic.
    unsafe {
        esp32_cam_ll::emergency_stop_i2s0_hw();
    }
    release_psram_cs_and_exit_qpi();
    esp_hal::system::software_reset()
}

#[cfg(target_arch = "xtensa")]
#[esp_hal::ram]
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

#[cfg(target_arch = "xtensa")]
#[derive(Debug, Clone, Copy)]
struct StaConnectedSummary {
    bssid: [u8; 6],
    channel: u8,
    aid: u16,
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

/// Evaluates runtime roam decision against cached scan candidates.
///
/// Candidates are logged; the decision applies `ROAM_HYSTERESIS_DB` to
/// `current_rssi`.
#[cfg(target_arch = "xtensa")]
fn roam_scan_decision(
    net_device: &Esp32StaWifiDriver<HardwareWifiMmioBus>,
    current_bssid: [u8; 6],
    current_rssi: i8,
) -> klipper_cam_fw::RoamDecision {
    use klipper_cam_fw::{roam_decision, ApTracker, ROAM_HYSTERESIS_DB};

    let mut tracker = ApTracker::new();
    for c in net_device.scan_candidates() {
        tracker.record(c.bssid, c.channel, c.rssi);
    }
    if tracker.candidates().is_empty() {
        tracker.record(current_bssid, net_device.channel(), current_rssi);
    }
    log_ap_candidates(&tracker, 1);
    roam_decision(current_bssid, current_rssi, &tracker, ROAM_HYSTERESIS_DB)
}

/// Re-associates with `target`: disconnect, BSSID-locked `StaConnectConfig`,
/// and bounded `poll()` loop (10 s).
///
/// On failure the unlocked base `StaConnectConfig` is restored so the regular
/// reconnect path in the server loop can recover the link.
#[cfg(target_arch = "xtensa")]
fn roam_to_candidate(
    net_device: &mut Esp32StaWifiDriver<HardwareWifiMmioBus>,
    ssid_str: &str,
    pass_str: &str,
    target: &klipper_cam_fw::ApCandidate,
) -> Option<StaConnectedSummary> {
    use esp_hal::delay::Delay;
    use esp_println::println;

    let delay = Delay::new();
    let _ = net_device.disconnect(3, now_millis());
    phy_set_max_tx_power_qdbm(WIFI_MAX_TX_POWER_QDBM);
    let locked_cfg = build_sta_config(ssid_str, pass_str, Some(target.bssid()));
    if net_device.connect(locked_cfg, now_millis()).is_err() {
        println!("[KLIPPER-CAM-RS] [ROAM] connect(locked) rejected; restoring base config");
        let _ = net_device.connect(build_sta_config(ssid_str, pass_str, None), now_millis());
        return None;
    }
    let start_ms = now_millis();
    loop {
        feed_rtc_wdt();
        let cur_ms = now_millis();
        let _ = net_device.poll(cur_ms);
        if net_device.is_connected() {
            phy_set_max_tx_power_qdbm(WIFI_MAX_TX_POWER_QDBM);
            return Some(StaConnectedSummary {
                bssid: net_device.bssid().unwrap_or(target.bssid()),
                channel: net_device.channel(),
                aid: net_device.aid(),
            });
        }
        if matches!(net_device.state(), StaState::Idle) {
            println!("[KLIPPER-CAM-RS] [ROAM] connect stopped idle; restoring base config");
            let _ = net_device.connect(build_sta_config(ssid_str, pass_str, None), cur_ms);
            return None;
        }
        if cur_ms.saturating_sub(start_ms) >= 10_000 {
            println!("[KLIPPER-CAM-RS] [ROAM] connect timed out (10000 ms)");
            let _ = net_device.disconnect(3, cur_ms);
            let _ = net_device.connect(build_sta_config(ssid_str, pass_str, None), cur_ms);
            return None;
        }
        delay.delay_millis(2);
    }
}

/// Post-association steps shared by the reconnect and roam paths: mark the
/// STA connected, restore max TX power, arm static-IP ARP confirmation if IP
/// is not yet confirmed, and (once the IP is configured) send a gratuitous
/// ARP announce.
///
/// `announce_ip` is `Some((ip, gateway))` once the station IP is configured;
/// `None` skips the ARP announce.
#[cfg(target_arch = "xtensa")]
fn after_sta_associated(
    runtime: &mut FirmwareRuntime,
    static_ip_arp_pending: &mut bool,
    net_device: &mut Esp32StaWifiDriver<HardwareWifiMmioBus>,
    mac: [u8; 6],
    announce_ip: Option<([u8; 4], [u8; 4])>,
    is_static_mode: bool,
) {
    runtime.on_wifi_sta_connected();
    phy_set_max_tx_power_qdbm(WIFI_MAX_TX_POWER_QDBM);
    if let Some((ip, gw)) = announce_ip {
        if is_static_mode && !runtime.ip_confirmed {
            *static_ip_arp_pending = true;
        }
        runtime.wifi_state = klipper_cam_fw::WifiStaState::ConnectedGotIp {
            ip: Ipv4Addr::new(ip[0], ip[1], ip[2], ip[3]),
        };
        runtime.webservers_started = true;
        send_arp_announce(net_device, mac, ip, gw);
    }
}

#[cfg(target_arch = "xtensa")]
#[inline(never)]
fn connect_strongest_ap(
    net_device: &mut Esp32StaWifiDriver<HardwareWifiMmioBus>,
    ssid_str: &str,
    pass_str: &str,
    timeout_ms: u64,
) -> Option<StaConnectedSummary> {
    use esp_hal::delay::Delay;
    use esp_println::println;
    use klipper_cam_fw::ApTracker;

    let wait_ms = timeout_ms.max(15_000);
    let delay = Delay::new();
    phy_set_max_tx_power_qdbm(WIFI_MAX_TX_POWER_QDBM);

    let sta_cfg = build_sta_config(ssid_str, pass_str, None);
    if let Err(e) = net_device.connect(sta_cfg, now_millis()) {
        println!(
            "[KLIPPER-CAM-RS] [WIFI] connect('{}') start error: {:?}",
            ssid_str, e
        );
        return None;
    }

    let start_ms = now_millis();
    let mut candidates_logged = false;
    loop {
        feed_rtc_wdt();
        let cur_ms = now_millis();
        let _ = net_device.poll(cur_ms);
        if !candidates_logged
            && !net_device.scan_candidates().is_empty()
            && net_device.state() != StaState::Scanning
        {
            candidates_logged = true;
            let mut tracker = ApTracker::new();
            for c in net_device.scan_candidates() {
                tracker.record(c.bssid, c.channel, c.rssi);
            }
            log_ap_candidates(&tracker, 1);
            if let Some(best) = tracker.best() {
                let target_bssid = best.bssid();
                println!(
                    concat!(
                        "[KLIPPER-CAM-RS] [WIFI] Strongest '{}' AP: ch={} ",
                        "rssi={}dBm bssid={:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}"
                    ),
                    ssid_str,
                    best.channel(),
                    best.rssi_avg(),
                    target_bssid[0],
                    target_bssid[1],
                    target_bssid[2],
                    target_bssid[3],
                    target_bssid[4],
                    target_bssid[5]
                );
            }
        }
        if net_device.is_connected() {
            phy_set_max_tx_power_qdbm(WIFI_MAX_TX_POWER_QDBM);
            let fallback_bssid = net_device
                .scan_candidates()
                .first()
                .map_or([0; 6], |c| c.bssid);
            return Some(StaConnectedSummary {
                bssid: net_device.bssid().unwrap_or(fallback_bssid),
                channel: net_device.channel(),
                aid: net_device.aid(),
            });
        }
        if matches!(net_device.state(), StaState::Idle) {
            println!(
                "[KLIPPER-CAM-RS] [WIFI] connect('{}') stopped idle",
                ssid_str
            );
            delay.delay_millis(200);
            return None;
        }
        if cur_ms.saturating_sub(start_ms) >= wait_ms {
            println!(
                "[KLIPPER-CAM-RS] [WIFI] connect('{}') timed out ({} ms)",
                ssid_str, wait_ms
            );
            let _ = net_device.disconnect(3, cur_ms);
            delay.delay_millis(200);
            return None;
        }
        delay.delay_millis(2);
    }
}

#[cfg(target_arch = "xtensa")]
#[derive(Debug, Clone, Copy, Default)]
struct Port80WorkerState {
    req_len: usize,
    tx_off: usize,
    tx_len: usize,
    closing: bool,
    close_deadline_ms: u64,
    tx_started_ms: u64,
    using_ota_accum: bool,
    req_started_ms: u64,
    ota_rx_started_ms: u64,
    conn_active_ms: u64,
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
        tx_started_ms: 0,
        using_ota_accum: false,
        req_started_ms: 0,
        ota_rx_started_ms: 0,
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
    close_deadline_ms: u64,
    req_started_ms: u64,
    conn_active_ms: u64,
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

    feed_rtc_wdt();
    let Some(mut dma_lease) = I2S_DMA_STORAGE.try_borrow() else {
        return Err((0, 0, 0, 0, 0, 0xDEAD));
    };
    let (desc_slice, dma_slice) = dma_lease.parts_mut();
    if build_circular_ring(desc_slice, dma_slice, 2048).is_err() {
        FW_STATS.inc_ring_build_failures();
        return Err((0, 0, 0, 0, 0, 0xDEAD));
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

    // WP-06: completed half-buffers are consumed only through the shared ll
    // drain (EOF position + owner write-back gate, poison, single-store
    // re-arm, ring-overrun detection).
    let Ok(mut drain) = esp32_cam_ll::HalfDrain::new(
        8,
        2048,
        i2s_cam.filter(),
        esp32_cam_ll::DrainGate::OwnerWriteback,
    ) else {
        return Err((0, 0, 0, 0, 0, 0xDEAD));
    };
    let mut chunk = [0u8; esp32_cam_ll::JPEG_UNPACKED_HALF_BUFFER_SIZE];

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
    let mut dma_failed = false;

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
                return Err((0, 0, 0, 0, 0, 0xDEAD));
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

        // JUSTIFICATION: Starts I2S0 DMA on the static ring (WP-08 migration
        // entry point; this loop keeps its WP-06 drain over raw slices).
        // SAFETY:
        // 1. Alignment & Non-null: `first_desc_addr` is `I2S_DMA_DESC[0]`, a
        //    4-byte aligned static in internal DRAM.
        // 2. Layout: `build_circular_ring` linked 16 x 2048-byte nodes over
        //    the 4-byte aligned 32 KiB `I2S_DMA_BUF` in internal DRAM.
        // 3. Provenance: Both statics live for `'static`.
        // 4. Aliasing: While the DMA runs, the ring is touched only through
        //    the ll `HalfDrain`; every exit below calls `i2s_cam.stop()`.
        // 5. Synchronization: `start_raw` fences (`memw`) before `in_link`.
        // 6. Unwind safety: Bare-metal environment with panic = abort.
        if !unsafe { i2s_cam.start_raw(&dma_cfg, first_desc_addr) } {
            return Err((0, 0, 0, 0, 0, 0xDEAD));
        }
        drain.reset();

        eof_cnt = 0;
        fb_len = 0;
        dma_failed = false;
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

                // FW-CORE-02: consume only halves the DMA has finished.
                let mut slot_full = false;
                let mut missed_soi = false;
                let drained = drain.drain_ready_halves(
                    desc_slice,
                    dma_slice,
                    &mut i2s_cam.dma_status(),
                    &mut chunk,
                    |half: &[u8]| {
                        if fb_len + half.len() > write_slot.len() {
                            slot_full = true;
                            return core::ops::ControlFlow::Break(());
                        }
                        write_slot[fb_len..fb_len + half.len()].copy_from_slice(half);
                        if eof_cnt == 0
                            && sync_pass == 0
                            && verify_jpeg_soi(&write_slot[..256]).is_none()
                        {
                            missed_soi = true;
                            return core::ops::ControlFlow::Break(());
                        }
                        fb_len += half.len();
                        eof_cnt += 1;
                        if (eof_cnt & 7) == 0 {
                            feed_rtc_wdt();
                            poll_net();
                        }
                        core::ops::ControlFlow::Continue(())
                    },
                );
                match drained {
                    Err(_) => {
                        // LL-07: ring overrun or descriptor error; the
                        // frame is corrupt and must not be committed.
                        FW_STATS.inc_overflow_aborts();
                        dma_failed = true;
                        let _ = i2s_cam.stop();
                        break 'dma_loop;
                    }
                    Ok(_) if missed_soi => {
                        let _ = i2s_cam.stop();
                        missed_soi_on_fast_pass = true;
                        break 'dma_loop;
                    }
                    Ok(_) if slot_full => {
                        let _ = i2s_cam.stop();
                        break 'dma_loop;
                    }
                    Ok(n) if n > 0 => {
                        feed_rtc_wdt();
                        last_eof_ms = now_millis();
                        continue;
                    }
                    Ok(_) => {}
                }

                if eof_cnt > 0 {
                    let vsync_ended = seen_vsync_high
                        && !read_vsync()
                        && !read_vsync()
                        && !read_vsync()
                        && !read_vsync();
                    if vsync_ended || now_millis().saturating_sub(last_eof_ms) >= 25 {
                        // `stop()` halts and resets the inlink DMA and
                        // issues `memw` (LL-21).
                        let _ = i2s_cam.stop();
                        // LL-05: take halves the owner gate still held back,
                        // then the partially written tail half,
                        // unconditionally. Poisoning (LL-06) makes unwritten
                        // bytes read as `DMA_POISON_BYTE`, never as stale
                        // markers.
                        loop {
                            let more = drain.pending() > 0;
                            let Ok(n) = drain.drain_tail_half(
                                desc_slice,
                                dma_slice,
                                &mut i2s_cam.dma_status(),
                                &mut chunk,
                            ) else {
                                break;
                            };
                            let tail = &chunk[..n];
                            if tail.iter().any(|&b| b != esp32_cam_ll::DMA_POISON_BYTE) {
                                if fb_len + n > write_slot.len() {
                                    break;
                                }
                                write_slot[fb_len..fb_len + n].copy_from_slice(tail);
                                fb_len += n;
                                eof_cnt += 1;
                            }
                            if !more {
                                break;
                            }
                        }
                        break;
                    }
                }
            }
        }
        last_int_raw |= drain.int_raw_seen();

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
        if fb_len >= 64 && !dma_failed {
            if let Some(soi_pos) = verify_jpeg_soi(&write_slot[..fb_len.min(256)]) {
                let slice_from_soi = &write_slot[soi_pos..fb_len];
                // CONV-10: probe only the last chunk plus a margin.
                if let Some(eoi_rel) = esp32_cam_ll::find_jpeg_eoi_tail(
                    slice_from_soi,
                    esp32_cam_ll::JPEG_UNPACKED_HALF_BUFFER_SIZE,
                ) {
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
            camera.set_frame_size(current_fs);
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

/// Formats an HTTP 200 `/snapshot` response directly from a scoped
/// [`TripleFrameBuffer`] slice into `out_buf` without `'static` lifetime
/// laundering (`M-018` / `C3-002`).
#[cfg(target_arch = "xtensa")]
fn write_snapshot_http_response(
    jpeg: &[u8],
    out_buf: &mut [u8],
) -> Result<usize, klipper_cam_fw::HttpError> {
    use core::fmt::Write as _;

    let mut writer = SliceWriter::new(out_buf);
    write!(
        writer,
        "HTTP/1.1 200 OK\r\nContent-Type: image/jpeg\r\n\
         Content-Disposition: inline; filename=capture.jpg\r\n\
         Access-Control-Allow-Origin: *\r\nConnection: close\r\n\
         Cache-Control: no-store\r\n\
         Content-Length: {}\r\n\r\n",
        jpeg.len()
    )
    .map_err(|_| klipper_cam_fw::HttpError::BufferTooSmall)?;
    writer.write_bytes(jpeg)?;
    Ok(writer.written_len())
}

/// Why Core 1 abandoned the frame it was streaming into the SPSC ring.
#[cfg(target_arch = "xtensa")]
#[derive(Clone, Copy)]
enum Core1Abort {
    /// The first chunk carried no JPEG SOI.
    SoiMiss,
    /// The frame exceeded the 128 KiB SPSC frame budget.
    Overflow,
    /// The DMA ring lapped or reported a descriptor error (LL-07).
    DmaFault,
    /// The SPSC ring stayed full.
    RingFull,
    /// Core 0 requested a pause.
    Paused,
}

/// Validates one filtered half-buffer chunk and pushes it into the SPSC ring.
#[cfg(target_arch = "xtensa")]
#[inline]
fn core1_accept_chunk(
    chunk: &[u8],
    is_first_chunk: &mut bool,
    total_bytes: &mut usize,
    push: &impl Fn(&[u8], u8, u32) -> Result<(), Core1Abort>,
) -> Result<(), Core1Abort> {
    if *is_first_chunk {
        let scan_end = chunk.len().min(256);
        if verify_jpeg_soi(&chunk[..scan_end]).is_none() {
            return Err(Core1Abort::SoiMiss);
        }
    }
    if *total_bytes + chunk.len() > 128 * 1024 {
        return Err(Core1Abort::Overflow);
    }
    let flags = if *is_first_chunk { SPSC_FLAG_SOF } else { 0 };
    push(chunk, flags, 8_000)?;
    *is_first_chunk = false;
    *total_bytes += chunk.len();
    if CORE1_PAUSE_STATE.is_pause_requested() {
        return Err(Core1Abort::Paused);
    }
    Ok(())
}

/// Records a Core 1 frame abort in `FW_STATS`. Returns whether the frame was
/// already counted as dropped.
#[cfg(target_arch = "xtensa")]
#[inline]
fn core1_count_abort(reason: Core1Abort) -> bool {
    match reason {
        Core1Abort::SoiMiss => {
            FW_STATS.inc_soi_miss();
            FW_STATS.inc_c1_dropped();
            true
        }
        Core1Abort::Overflow | Core1Abort::DmaFault => {
            FW_STATS.inc_overflow_aborts();
            FW_STATS.inc_c1_dropped();
            true
        }
        Core1Abort::RingFull => {
            FW_STATS.inc_ring_full_events();
            FW_STATS.inc_c1_dropped();
            true
        }
        Core1Abort::Paused => false,
    }
}

/// Parks Core 1 inside IRAM after stopping I2S0 DMA and confirming no active
/// `&mut` DMA references exist (`M-019`, `M-077`). Never acknowledges a pause
/// if I2S0 DMA cannot be stopped.
#[cfg(target_arch = "xtensa")]
#[esp_hal::ram]
fn core1_handle_pause_if_requested(i2s_cam: &mut Esp32I2sCamera, aligned_from_eof: &mut bool) {
    if !CORE1_PAUSE_STATE.is_pause_requested() {
        return;
    }
    let mut stopped = i2s_cam.stop();
    if !stopped {
        FW_STATS.inc_i2s_stop_timeouts();
        // JUSTIFICATION: Hard-reset I2S0 DMA hardware when soft stop times out
        // so DMA cannot write while Core 0 disables SPI flash cache (`M-077`).
        // SAFETY: Core 1 owns `i2s_cam`; MMIO registers are valid ESP32 I2S0.
        unsafe {
            esp32_cam_ll::emergency_stop_i2s0_hw();
        }
        stopped = i2s_cam.stop();
    }
    *aligned_from_eof = false;
    if !stopped {
        // Refuse to acknowledge pause while DMA stop is unconfirmed (`M-077`).
        return;
    }
    let epoch = CORE1_PAUSE_STATE.req_epoch();
    if epoch != 0 && CORE1_PAUSE_STATE.is_pause_requested() {
        CORE1_PAUSE_STATE.ack_paused(epoch);
        while CORE1_PAUSE_STATE.is_pause_requested() {
            core::hint::spin_loop();
        }
        CORE1_PAUSE_STATE.clear_ack();
    }
}

#[cfg(target_arch = "xtensa")]
#[esp_hal::ram]
fn core1_camera_producer_loop() -> ! {
    const GPIO_IN_REG: *const u32 = 0x3FF4_403C as *const u32;

    // Synchronize with Core 0 startup handoff via `Acquire` barrier (`M-077`).
    while !CORE1_ACTIVE.load(Ordering::Acquire) {
        core::hint::spin_loop();
    }

    // Take sole owned `Esp32I2sCamera` from atomic `OnceOptionSlot` (`M-080`).
    let Some(mut i2s_cam) = HW_I2S_CAM.take() else {
        loop {
            if CORE1_PAUSE_STATE.is_pause_requested() {
                let epoch = CORE1_PAUSE_STATE.req_epoch();
                if epoch != 0 {
                    CORE1_PAUSE_STATE.ack_paused(epoch);
                    while CORE1_PAUSE_STATE.is_pause_requested() {
                        core::hint::spin_loop();
                    }
                    CORE1_PAUSE_STATE.clear_ack();
                }
            }
            core::hint::spin_loop();
        }
    };

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

    // WP-06: completed half-buffers are consumed only through the shared ll
    // drain (EOF position + owner write-back gate, poison, single-store
    // re-arm, ring-overrun detection).
    let Ok(mut drain) = esp32_cam_ll::HalfDrain::new(
        8,
        2048,
        i2s_cam.filter(),
        esp32_cam_ll::DrainGate::OwnerWriteback,
    ) else {
        let mut dummy_aligned = false;
        loop {
            core1_handle_pause_if_requested(&mut i2s_cam, &mut dummy_aligned);
            core::hint::spin_loop();
        }
    };

    let read_vsync = || -> bool {
        // JUSTIFICATION: Poll camera VSYNC level directly from GPIO_IN_REG.
        // SAFETY:
        // 1. Alignment & Non-null: `0x3FF4_403C` is 4-byte aligned MMIO.
        // 2. Layout: 32-bit GPIO input status register.
        // 3. Provenance: ESP32 DPORT/GPIO peripheral address space.
        // 4. Aliasing: Read-only volatile MMIO load.
        // 5. Volatile: `read_volatile` prevents loop hoisting.
        // 6. Unwind safety: Cannot panic.
        ((unsafe { core::ptr::read_volatile(GPIO_IN_REG) } >> 25) & 1) != 0
    };

    let push_spsc_bounded = |chunk: &[u8], flags: u8, max_spins: u32| -> Result<(), Core1Abort> {
        for _ in 0..max_spins {
            if CORE1_PAUSE_STATE.is_pause_requested() {
                return Err(Core1Abort::Paused);
            }
            // JUSTIFICATION: Core 1 is sole SPSC producer on `CORE1_SPSC`.
            // SAFETY:
            // 1. Alignment: `CORE1_SPSC` slots are 4-byte aligned in `.bss`.
            // 2. Layout: `chunk.len() <= JPEG_SPSC_CHUNK_BYTES` checked.
            // 3. Provenance: Static `CORE1_SPSC` lives in internal DRAM.
            // 4. Aliasing: Single producer (Core 1) / consumer (Core 0).
            // 5. Synchronization: Acquire/Release atomics on head/tail.
            // 6. Unwind safety: Bounded slice copy, zero panics.
            if unsafe { CORE1_SPSC.push_chunk(chunk, flags) } {
                return Ok(());
            }
            if CORE1_PAUSE_STATE.is_pause_requested() {
                return Err(Core1Abort::Paused);
            }
            core::hint::spin_loop();
        }
        if CORE1_PAUSE_STATE.is_pause_requested() {
            Err(Core1Abort::Paused)
        } else {
            Err(Core1Abort::RingFull)
        }
    };

    // Startup DMA ring build check (`M-078`): never fall through on error.
    let startup_ring_ok = if let Some(mut lease) = I2S_DMA_STORAGE.try_borrow() {
        let (desc_slice, dma_slice) = lease.parts_mut();
        dma_slice.fill(esp32_cam_ll::DMA_POISON_BYTE);
        build_circular_ring(desc_slice, dma_slice, 2048).is_ok()
    } else {
        false
    };
    if !startup_ring_ok {
        FW_STATS.inc_ring_build_failures();
        let mut dummy_aligned = false;
        loop {
            core1_handle_pause_if_requested(&mut i2s_cam, &mut dummy_aligned);
            core::hint::spin_loop();
        }
    }

    let mut unpack_buf = [0u8; JPEG_SPSC_CHUNK_BYTES];
    let mut aligned_from_eof = false;
    let mut last_vsync_cycles = 0u32;

    loop {
        if CORE1_PAUSE_STATE.is_pause_requested() {
            // No `I2sDmaLease` / `&mut` DMA references exist while parked.
            core1_handle_pause_if_requested(&mut i2s_cam, &mut aligned_from_eof);
            continue;
        }

        if !aligned_from_eof || read_vsync() {
            let mut paused = false;
            let mut saw_high = read_vsync();
            if !saw_high {
                for spin in 0..3_000_000u32 {
                    if (spin & 0x3FF) == 0 && CORE1_PAUSE_STATE.is_pause_requested() {
                        paused = true;
                        break;
                    }
                    if read_vsync() {
                        saw_high = true;
                        break;
                    }
                    core::hint::spin_loop();
                }
            }
            if paused {
                continue;
            }
            if !saw_high {
                // VSYNC stuck low (`M-221` / `C3-060`): do not record bogus
                // period or arm unaligned DMA.
                VSYNC_TIMEOUTS.fetch_add(1, Ordering::Relaxed);
                FW_STATS.inc_c1_dropped();
                last_vsync_cycles = 0;
                aligned_from_eof = false;
                continue;
            }
            let mut saw_low = false;
            for spin in 0..4_000_000u32 {
                if (spin & 0x3FF) == 0 && CORE1_PAUSE_STATE.is_pause_requested() {
                    paused = true;
                    break;
                }
                if !read_vsync() {
                    saw_low = true;
                    break;
                }
                core::hint::spin_loop();
            }
            if paused {
                continue;
            }
            if !saw_low {
                // VSYNC stuck high (`M-221` / `C3-060`): skip bogus period.
                VSYNC_TIMEOUTS.fetch_add(1, Ordering::Relaxed);
                FW_STATS.inc_c1_dropped();
                last_vsync_cycles = 0;
                aligned_from_eof = false;
                continue;
            }
        }

        let vsync_cycles = ccount();
        if last_vsync_cycles != 0 {
            let delta = vsync_cycles.wrapping_sub(last_vsync_cycles);
            FW_STATS.record_sensor_period_us(delta / 240);
        }
        last_vsync_cycles = vsync_cycles;

        // Scope DMA buffer lease strictly to a single capture iteration so zero
        // `&'static mut` DMA references exist across pause windows (`M-077`).
        let Some(mut dma_lease) = I2S_DMA_STORAGE.try_borrow() else {
            aligned_from_eof = false;
            continue;
        };
        let (desc_slice, dma_slice) = dma_lease.parts_mut();

        if build_circular_ring(desc_slice, dma_slice, 2048).is_err() {
            FW_STATS.inc_ring_build_failures();
            aligned_from_eof = false;
            continue;
        }
        let first_desc_addr = (core::ptr::from_ref(&desc_slice[0]) as usize as u32) & 0x000F_FFFF;

        // `start()` clears the stale `IN_SUC_EOF` / `IN_DSCR_ERR` /
        // `IN_DSCR_EMPTY` latches before arming the DMA.
        // JUSTIFICATION: Starts I2S0 DMA on the leased static ring.
        // SAFETY:
        // 1. Alignment & Non-null: `first_desc_addr` is `desc_slice[0]`, a
        //    4-byte aligned static in internal DRAM.
        // 2. Layout: `build_circular_ring` linked 16 x 2048-byte nodes over
        //    the 4-byte aligned 32 KiB buffer in internal DRAM.
        // 3. Provenance: `I2S_DMA_STORAGE` lives for `'static`.
        // 4. Aliasing: Core 1 holds `I2sDmaLease`; while DMA runs it is touched
        //    only through `HalfDrain`, and every frame ends with `stop()`.
        // 5. Synchronization: `start_raw` fences (`memw`) before `in_link`.
        // 6. Unwind safety: Diverging loop in `no_std` firmware.
        if !unsafe { i2s_cam.start_raw(&dma_cfg, first_desc_addr) } {
            aligned_from_eof = false;
            continue;
        }
        drain.reset();

        let mut is_first_chunk = true;
        let mut total_bytes = 0usize;
        let mut aborted = false;
        let mut seen_vsync_high = false;
        let mut idle_spins = 0u32;
        let mut c1_drop_counted = false;

        'frame_poll: loop {
            if (idle_spins & 0x3FF) == 0 && CORE1_PAUSE_STATE.is_pause_requested() {
                aborted = true;
                c1_drop_counted = true;
                aligned_from_eof = false;
                break 'frame_poll;
            }

            let vsync_now = read_vsync();
            if vsync_now {
                seen_vsync_high = true;
            }

            // FW-CORE-02: consume only halves the DMA has finished; the EOF
            // bit is only a wake-up hint inside the drain.
            let mut sink_abort: Option<Core1Abort> = None;
            let drained = drain.drain_ready_halves(
                desc_slice,
                dma_slice,
                &mut i2s_cam.dma_status(),
                &mut unpack_buf,
                |chunk: &[u8]| match core1_accept_chunk(
                    chunk,
                    &mut is_first_chunk,
                    &mut total_bytes,
                    &push_spsc_bounded,
                ) {
                    Ok(()) => core::ops::ControlFlow::Continue(()),
                    Err(reason) => {
                        sink_abort = Some(reason);
                        core::ops::ControlFlow::Break(())
                    }
                },
            );
            let (drained_halves, abort) = match drained {
                Ok(n) => (n, sink_abort),
                Err(_) => (0, Some(Core1Abort::DmaFault)),
            };
            if let Some(reason) = abort {
                if matches!(reason, Core1Abort::Paused) {
                    c1_drop_counted = true;
                } else {
                    c1_drop_counted |= core1_count_abort(reason);
                }
                aborted = true;
                aligned_from_eof = false;
                break 'frame_poll;
            }
            if drained_halves > 0 {
                idle_spins = 0;
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
                    aligned_from_eof = vsync_ended && !read_vsync();

                    loop {
                        let more = drain.pending() > 0;
                        let Ok(n) = drain.drain_tail_half(
                            desc_slice,
                            dma_slice,
                            &mut i2s_cam.dma_status(),
                            &mut unpack_buf,
                        ) else {
                            break;
                        };
                        let tail = &unpack_buf[..n];
                        let has_data = tail.iter().any(|&b| b != esp32_cam_ll::DMA_POISON_BYTE);
                        let accept = has_data && (more || total_bytes + n <= 128 * 1024);
                        let accepted = if accept {
                            core1_accept_chunk(
                                tail,
                                &mut is_first_chunk,
                                &mut total_bytes,
                                &push_spsc_bounded,
                            )
                        } else {
                            Ok(())
                        };
                        if let Err(reason) = accepted {
                            if matches!(reason, Core1Abort::Paused) {
                                c1_drop_counted = true;
                            } else {
                                c1_drop_counted |= core1_count_abort(reason);
                            }
                            aborted = true;
                            aligned_from_eof = false;
                            break;
                        }
                        if !more {
                            break;
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
            dma_slice.fill(esp32_cam_ll::DMA_POISON_BYTE);
            if !is_first_chunk {
                let _ = push_spsc_bounded(&[], SPSC_FLAG_ABORT, 8_000);
            }
            if !CORE1_PAUSE_STATE.is_pause_requested() {
                CORE1_SPSC.record_frame_dropped();
            }
        } else {
            match push_spsc_bounded(&[], SPSC_FLAG_EOF, 32_000) {
                Ok(()) => {
                    FW_STATS.inc_c1_pushed();
                    FW_STATS.record_frame_bytes(total_bytes as u32);
                    CORE1_SPSC.record_frame_pushed();
                }
                Err(Core1Abort::Paused) => {
                    let _ = push_spsc_bounded(&[], SPSC_FLAG_ABORT, 1_000);
                }
                Err(_) => {
                    FW_STATS.inc_ring_full_events();
                    FW_STATS.inc_c1_dropped();
                    let _ = push_spsc_bounded(&[], SPSC_FLAG_ABORT, 8_000);
                    CORE1_SPSC.record_frame_dropped();
                }
            }
        }
        // `dma_lease` drops here before the next loop iteration / pause window.
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
                OTA_FLASH_ERRORS.fetch_add(1, Ordering::Relaxed);
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
struct WifiDriverSlot {
    taken: AtomicBool,
    driver: UnsafeCell<Esp32StaWifiDriver<HardwareWifiMmioBus>>,
}

// JUSTIFICATION: Share single static internal-DRAM Wi-Fi MAC driver across
// `Sync` static storage in `#![no_std]` bare-metal firmware.
// SAFETY:
// 1. Preconditions: `WifiDriverSlot::take()` enforces `Cpu::ProCpu` affinity
//    and atomically grants at most one `&'static mut` via `compare_exchange`.
// 2. Alignment & Layout: `Esp32StaWifiDriver` is 4-byte aligned in `.bss`.
// 3. Provenance & Lifetime: Static storage lives for `'static`.
// 4. Aliasing: Only Core 0 initialization takes and owns the `&'static mut`.
// 5. Synchronization: `AtomicBool` `AcqRel` CAS + Core 0 affinity check.
// 6. Panic Safety: Bare-metal firmware uses `panic = "abort"`.
#[cfg(target_arch = "xtensa")]
unsafe impl Sync for WifiDriverSlot {}

#[cfg(target_arch = "xtensa")]
impl WifiDriverSlot {
    const fn new() -> Self {
        // JUSTIFICATION: Construct the single static Wi-Fi MMIO bus inside
        // `WIFI_DRIVER_SLOT` during `const fn` static initialization (`M-016`).
        // SAFETY:
        // 1. Preconditions: Only one `WifiDriverSlot` static exists and
        //    `HardwareWifiMmioBus::take()` is never called in this binary.
        // 2. Alignment & Layout: Zero-sized bus handle stored inline in `.bss`.
        // 3. Provenance & Lifetime: Static storage lives for `'static`.
        // 4. Aliasing: `WifiDriverSlot::take()` enforces single-shot Core 0
        //    mutable ownership via atomic CAS.
        // 5. Synchronization: Exclusive access granted before driver init.
        // 6. Panic Safety: Pure `const fn` construction cannot panic.
        let bus = unsafe { HardwareWifiMmioBus::new() };
        Self {
            taken: AtomicBool::new(false),
            driver: UnsafeCell::new(Esp32StaWifiDriver::new(bus)),
        }
    }

    // Single-shot CAS on `taken` + Core 0 check grants at most one `&mut`.
    #[allow(clippy::mut_from_ref)]
    fn take(&'static self) -> Option<&'static mut Esp32StaWifiDriver<HardwareWifiMmioBus>> {
        if Cpu::current() != Cpu::ProCpu {
            return None;
        }
        if self
            .taken
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
            .is_err()
        {
            return None;
        }
        // JUSTIFICATION: Claim sole mutable reference to static internal-DRAM
        // `Esp32StaWifiDriver` after atomic `taken` CAS transition on Core 0.
        // SAFETY:
        // 1. Preconditions: `Cpu::ProCpu` verified and CAS succeeded once.
        // 2. Alignment & Layout: `self.driver` is properly aligned and valid.
        // 3. Provenance & Lifetime: `&'static self` backs `'static` storage.
        // 4. Aliasing: No other reference can be formed after `taken == true`.
        // 5. Synchronization: `AcqRel` CAS publishes initialization state.
        // 6. Panic Safety: No panic occurs between CAS and return.
        Some(unsafe { &mut *self.driver.get() })
    }
}

#[cfg(target_arch = "xtensa")]
const _: fn() = || {
    fn assert_send<T: Send>() {}
    assert_send::<Esp32StaWifiDriver<HardwareWifiMmioBus>>();
};

#[cfg(target_arch = "xtensa")]
static WIFI_DRIVER_SLOT: WifiDriverSlot = WifiDriverSlot::new();

#[cfg(target_arch = "xtensa")]
static PSRAM_BUFFERS: OnceOptionSlot<PsramBuffers> = OnceOptionSlot::new();

#[cfg(target_arch = "xtensa")]
static FIRMWARE_RUNTIME: OnceOptionSlot<FirmwareRuntime<'static>> = OnceOptionSlot::new();

#[cfg(target_arch = "xtensa")]
static HW_I2S_CAM: OnceOptionSlot<Esp32I2sCamera> = OnceOptionSlot::new();

#[cfg(target_arch = "xtensa")]
static SOCKET_STORAGE: Core0TakeCell<[smoltcp::iface::SocketStorage<'static>; 7]> =
    Core0TakeCell::new([smoltcp::iface::SocketStorage::EMPTY; 7]);

#[cfg(target_arch = "xtensa")]
static P80_STATES: Core0TakeCell<[Port80WorkerState; PORT80_SOCKET_COUNT]> =
    Core0TakeCell::new([Port80WorkerState::EMPTY; PORT80_SOCKET_COUNT]);

#[cfg(target_arch = "xtensa")]
static P81_STATES: Core0TakeCell<[Port81WorkerState; PORT81_SOCKET_COUNT]> =
    Core0TakeCell::new([Port81WorkerState::EMPTY; PORT81_SOCKET_COUNT]);

#[cfg(target_arch = "xtensa")]
static P81_REQ_BUFS: Core0TakeCell<[[u8; 1024]; PORT81_SOCKET_COUNT]> =
    Core0TakeCell::new([[0u8; 1024]; PORT81_SOCKET_COUNT]);

#[cfg(target_arch = "xtensa")]
static P81_HDR_BUF: Core0TakeCell<[u8; 512]> = Core0TakeCell::new([0u8; 512]);

#[cfg(target_arch = "xtensa")]
static P81_ERR_BUF: Core0TakeCell<[u8; 512]> = Core0TakeCell::new([0u8; 512]);

#[cfg(target_arch = "xtensa")]
static P81_DRAIN_BUF: Core0TakeCell<[u8; 128]> = Core0TakeCell::new([0u8; 128]);

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
                if !ensure_psram_qpi_mode_spi1() {
                    println!("[KLIPPER-CAM-RS] [PSRAM] QPI retry command timed out");
                }
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

    if !PSRAM_BUFFERS.put(PsramBuffers {
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
    }) {
        quiesce_psram_and_software_reset();
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

    // LL-13: the single I2S0 owner (the self-test above dropped its claim).
    let Some(mut hw_i2s_cam) = Esp32I2sCamera::take() else {
        println!("[KLIPPER-CAM-RS] I2S0 is already claimed; camera disabled");
        return;
    };
    let ll_pins = WroverKitPins::PINS.to_ll_pin_config(false);
    let _ = hw_i2s_cam.set_pin(&ll_pins);
    let _ = hw_i2s_cam.config();
    let _ = hw_i2s_cam.set_sample_mode(PixelFormat::Jpeg, DEFAULT_XCLK_FREQ_HZ, 0x26);
    if !HW_I2S_CAM.put(hw_i2s_cam) {
        println!("[KLIPPER-CAM-RS] HW_I2S_CAM slot already initialized");
    }
}

#[cfg(target_arch = "xtensa")]
#[inline(never)]
fn capture_boot_dma_and_init_runtime(ov2640_hw_ok: bool, bc_prev: BreadcrumbPrevSummary) -> bool {
    use esp_hal::delay::Delay;
    use esp_println::println;

    let delay = Delay::new();
    let ov2640_dma_ok = ov2640_hw_ok;
    let mut boot_dma_ok = false;
    let mut camera = CameraDriver::new_hd_default();

    let Some(active_len_after) = PSRAM_BUFFERS.with_mut(|psram_bufs| {
        if ov2640_hw_ok {
            let _ = HW_I2S_CAM.with_mut(|hw_i2s_cam| {
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
            });
        }
        psram_bufs.triple_fb.active_len()
    }) else {
        quiesce_psram_and_software_reset();
    };
    if !boot_dma_ok {
        println!(
            concat!(
                "[KLIPPER-CAM-RS] [HW-3] Live OV2640 I2S0 DMA Capture fallback: ",
                "using synthetic 1280x720 JFIF ({} bytes)"
            ),
            active_len_after
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
    if !FIRMWARE_RUNTIME.put(runtime) {
        quiesce_psram_and_software_reset();
    }
    ov2640_dma_ok
}

#[cfg(target_arch = "xtensa")]
fn build_sta_config(ssid: &str, pass: &str, target_bssid: Option<[u8; 6]>) -> StaConnectConfig {
    match StaConnectConfig::from_slices(ssid.as_bytes(), pass.as_bytes(), target_bssid, -127) {
        Ok(cfg) => cfg,
        Err(_) => panic!(
            "[KLIPPER-CAM-RS] [CONFIG] WIFI_PASS rejected by the Wi-Fi driver; \
             refusing to start Wi-Fi"
        ),
    }
}

#[cfg(target_arch = "xtensa")]
static PROV_CONFIG_CELL: static_cell::StaticCell<ProvisionedConfig> =
    static_cell::StaticCell::new();

#[cfg(target_arch = "xtensa")]
#[inline(never)]
fn validate_wifi_station_config(boot_cfg: &KlipperCamConfig) -> (&'static str, &'static str) {
    use esp_println::println;
    use klipper_cam_fw::config::WifiAuthMode;
    use klipper_cam_fw::config::ALLOW_OPEN_WIFI;

    // FW-NET-03: validate the boot configuration and fail closed. A bad
    // configuration or passphrase stops boot; it never falls back to Open.
    let auth_mode = match boot_cfg.boot_wifi_auth(ALLOW_OPEN_WIFI) {
        Ok(mode) => mode,
        Err(e) => panic!("[KLIPPER-CAM-RS] [CONFIG] refusing to start Wi-Fi: {e}"),
    };
    // FW-NET-04: default or blank OTA token disables OTA/setup writes (403).
    if !boot_cfg.has_valid_ota_token() || boot_cfg.is_default_ota_token() {
        println!(
            "[KLIPPER-CAM-RS] [SECURITY] WARNING: no valid private OTA token configured; \
             POST /update and STA POST /setup are disabled (403)."
        );
    }

    let ssid = boot_cfg.network.wifi_ssid;
    let pass = boot_cfg.network.wifi_password;
    let effective_pass = match auth_mode {
        WifiAuthMode::Open => {
            println!(
                "[KLIPPER-CAM-RS] [SECURITY] WARNING: joining an Open Wi-Fi \
                 network (allow-open-wifi)"
            );
            ""
        }
        WifiAuthMode::WpaWpa2Personal => pass,
    };
    let _ = build_sta_config(ssid, effective_pass, None);
    (ssid, effective_pass)
}

/// Non-blocking single-byte read from the hardware UART0 RX FIFO via the
/// `esp-hal` PAC (active strictly inside Setup Mode).
#[cfg(target_arch = "xtensa")]
#[inline]
fn poll_uart0_rx_byte() -> Option<u8> {
    let uart0 = esp_hal::peripherals::UART0::regs();
    if uart0.status().read().rxfifo_cnt().bits() > 0 {
        Some(uart0.fifo().read().rxfifo_rd_byte().bits())
    } else {
        None
    }
}

/// Borrowed Ethernet RX token used by [`SetupApNetDevice`] when forwarding
/// non-DHCP/DNS frames (ARP, TCP Port 80, etc.) to `smoltcp`.
#[cfg(target_arch = "xtensa")]
struct SetupRxToken<'a> {
    data: &'a [u8],
}

#[cfg(target_arch = "xtensa")]
impl smoltcp::phy::RxToken for SetupRxToken<'_> {
    fn consume<R, F: FnOnce(&[u8]) -> R>(self, f: F) -> R {
        f(self.data)
    }
}

/// Layer-2 `smoltcp::phy::Device` wrapper active only in SoftAP Setup Mode.
///
/// Intercepts UDP DHCPv4 (port 67) and captive-portal DNS (port 53) packets
/// directly over the shared [`Esp32StaWifiDriver`] DMA ring while forwarding
/// ARP and TCP Port 80 traffic to the local Setup Mode `smoltcp::Interface`.
#[cfg(target_arch = "xtensa")]
struct SetupApNetDevice<'a> {
    driver: &'a mut Esp32StaWifiDriver<HardwareWifiMmioBus>,
    dhcp: DhcpServer,
    ap_mac: [u8; 6],
    ap_ip: [u8; 4],
    rx_buf: &'a mut [u8],
    tx_buf: &'a mut [u8],
    ip_id: u16,
    saw_activity: bool,
}

#[cfg(target_arch = "xtensa")]
impl SetupApNetDevice<'_> {
    fn take_activity(&mut self) -> bool {
        let had = self.saw_activity;
        self.saw_activity = false;
        had
    }
}

#[cfg(target_arch = "xtensa")]
impl smoltcp::phy::Device for SetupApNetDevice<'_> {
    type RxToken<'b>
        = SetupRxToken<'b>
    where
        Self: 'b;
    type TxToken<'b>
        = esp32_cam_wifi_mac::StaTxToken<'b, HardwareWifiMmioBus>
    where
        Self: 'b;

    fn receive(
        &mut self,
        timestamp: smoltcp::time::Instant,
    ) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        use smoltcp::phy::RxToken as _;
        use smoltcp::phy::TxToken as _;

        loop {
            let mut rx_len = 0usize;
            {
                let (rx_tok, _tx_tok) = self.driver.receive(timestamp)?;
                rx_tok.consume(|frame| {
                    let copy_len = frame.len().min(self.rx_buf.len());
                    self.rx_buf[..copy_len].copy_from_slice(&frame[..copy_len]);
                    rx_len = copy_len;
                });
            }
            if rx_len == 0 {
                continue;
            }
            self.ip_id = self.ip_id.wrapping_add(1);
            match process_setup_l2_udp_frame(
                &self.rx_buf[..rx_len],
                self.ap_mac,
                self.ap_ip,
                &mut self.dhcp,
                self.ip_id,
                self.tx_buf,
            ) {
                SetupL2UdpAction::Reply(tx_len) => {
                    self.saw_activity = true;
                    if let Some(tx_tok) = self.driver.transmit(timestamp) {
                        tx_tok.consume(tx_len, |buf| {
                            buf.copy_from_slice(&self.tx_buf[..tx_len]);
                        });
                    }
                }
                SetupL2UdpAction::Drop => {}
                SetupL2UdpAction::PassToStack => {
                    let tx_tok = self.driver.transmit(timestamp)?;
                    return Some((
                        SetupRxToken {
                            data: &self.rx_buf[..rx_len],
                        },
                        tx_tok,
                    ));
                }
            }
        }
    }

    fn transmit(&mut self, timestamp: smoltcp::time::Instant) -> Option<Self::TxToken<'_>> {
        self.driver.transmit(timestamp)
    }

    fn capabilities(&self) -> smoltcp::phy::DeviceCapabilities {
        self.driver.capabilities()
    }
}

#[cfg(target_arch = "xtensa")]
#[derive(Debug, Clone, Copy, Default)]
struct SetupTcpWorkerState {
    req_len: usize,
    tx_off: usize,
    tx_len: usize,
    closing: bool,
    close_deadline_ms: u64,
    conn_active_ms: u64,
}

#[cfg(target_arch = "xtensa")]
#[inline(never)]
fn run_provisioning_ap_loop(
    net_device: &'static mut Esp32StaWifiDriver<HardwareWifiMmioBus>,
    hw_mac: [u8; 6],
    current_ssid: &str,
    had_saved_creds: bool,
    mut flash_adapter: EspFlashAdapter<'static>,
) -> ! {
    use esp_hal::delay::Delay;
    use esp_println::print;
    use esp_println::println;
    use klipper_cam_fw::parse_http_request;
    use smoltcp::iface::Config as IfaceConfig;
    use smoltcp::iface::Interface as NetInterface;
    use smoltcp::iface::SocketSet;
    use smoltcp::iface::SocketStorage;
    use smoltcp::socket::tcp::Socket as TcpSocket;
    use smoltcp::socket::tcp::SocketBuffer as TcpSocketBuffer;
    use smoltcp::socket::tcp::State as TcpState;
    use smoltcp::wire::EthernetAddress;
    use smoltcp::wire::HardwareAddress;
    use smoltcp::wire::IpAddress;
    use smoltcp::wire::IpCidr;

    const SETUP_TCP_COUNT: usize = 2;
    const SETUP_REBOOT_FLUSH_CAP_MS: u64 = 3_000;
    const RNG_DATA_REG: *const u32 = 0x3FF7_5144 as *const u32;

    let delay = Delay::new();
    phy_set_max_tx_power_qdbm(WIFI_MAX_TX_POWER_QDBM);

    let ap_cfg = ApConfig::for_setup_mac(hw_mac, 1);
    let ap_ssid = core::str::from_utf8(ap_cfg.ssid_bytes()).unwrap_or("KlipperCam-Setup");
    if let Err(err) = net_device.enter_ap_mode(ap_cfg) {
        println!(
            "[KLIPPER-CAM-RS] [SETUP] enter_ap_mode failed ({:?}); rebooting...",
            err
        );
        delay.delay_millis(20);
        quiesce_psram_and_software_reset();
    }

    println!(
        "[SETUP] SoftAP SSID: {} | IP: 192.168.4.1 | Web: http://192.168.4.1/setup | UART0 CLI ready (type 'help')",
        ap_ssid
    );

    let Some(PsramBuffers {
        p80_rx0,
        p80_rx1,
        p80_tx0,
        p80_tx1,
        p80_hdr_bufs,
        p80_resp_bufs,
        ..
    }) = PSRAM_BUFFERS.take()
    else {
        quiesce_psram_and_software_reset();
    };
    let [req_buf0, req_buf1, l2_rx_buf, l2_tx_buf] = p80_hdr_bufs;
    let [resp_buf0, resp_buf1, _, _] = p80_resp_bufs;
    let req_bufs: [&mut [u8]; SETUP_TCP_COUNT] = [req_buf0, req_buf1];
    let resp_bufs: [&mut [u8]; SETUP_TCP_COUNT] = [resp_buf0, resp_buf1];

    let mut setup_net = SetupApNetDevice {
        driver: net_device,
        dhcp: DhcpServer::new(),
        ap_mac: hw_mac,
        ap_ip: AP_IP,
        rx_buf: &mut l2_rx_buf[..1536],
        tx_buf: &mut l2_tx_buf[..1536],
        ip_id: 1,
        saw_activity: false,
    };

    let mut iface_cfg = IfaceConfig::new(HardwareAddress::Ethernet(EthernetAddress(hw_mac)));
    // JUSTIFICATION: Read ESP32 hardware RNG register for Setup Mode TCP ISN seed.
    // SAFETY: RNG_DATA_REG (`0x3FF7_5144`) is a valid read-only APB MMIO register.
    let hw_rng_lo = unsafe { core::ptr::read_volatile(RNG_DATA_REG) } as u64;
    // JUSTIFICATION: Read second 32-bit hardware RNG word for 64-bit smoltcp seed.
    // SAFETY: RNG_DATA_REG (`0x3FF7_5144`) is a valid read-only APB MMIO register.
    let hw_rng_hi = unsafe { core::ptr::read_volatile(RNG_DATA_REG) } as u64;
    let boot_us = esp_hal::time::Instant::now()
        .duration_since_epoch()
        .as_micros();
    iface_cfg.random_seed = ((hw_rng_hi << 32) | hw_rng_lo) ^ boot_us ^ 0x9E37_79B9_7F4A_7C15;

    let mut iface = NetInterface::new(iface_cfg, &mut setup_net, smol_now());
    iface.update_ip_addrs(|addrs| {
        let _ = addrs.push(IpCidr::new(
            IpAddress::v4(AP_IP[0], AP_IP[1], AP_IP[2], AP_IP[3]),
            24,
        ));
    });

    let mut socket_storage = [SocketStorage::EMPTY; SETUP_TCP_COUNT];
    let mut sockets = SocketSet::new(&mut socket_storage[..]);
    let tcp_handles: [smoltcp::iface::SocketHandle; SETUP_TCP_COUNT] = [
        sockets.add(TcpSocket::new(
            TcpSocketBuffer::new(p80_rx0),
            TcpSocketBuffer::new(p80_tx0),
        )),
        sockets.add(TcpSocket::new(
            TcpSocketBuffer::new(p80_rx1),
            TcpSocketBuffer::new(p80_tx1),
        )),
    ];
    for &h in &tcp_handles {
        let sock = sockets.get_mut::<TcpSocket>(h);
        sock.set_nagle_enabled(false);
        sock.set_ack_delay(None);
        sock.set_timeout(None);
        let _ = sock.listen(80);
    }

    let mut tcp_states = [SetupTcpWorkerState::default(); SETUP_TCP_COUNT];
    let mut uart_session = UartProvSession::new();
    let mut last_activity_ms = now_millis();
    let mut last_pwctrl_ms = last_activity_ms;
    let mut last_rf_diag_ms = last_activity_ms;
    let mut pending_reboot_at_ms: Option<u64> = None;

    loop {
        feed_rtc_wdt();
        let now_u64 = now_millis();
        let now = smoltcp::time::Instant::from_millis(now_u64 as i64);

        let _ = setup_net.driver.poll(now_u64);
        let _ = iface.poll(now, &mut setup_net, &mut sockets);
        if setup_net.take_activity() {
            last_activity_ms = now_u64;
        }
        let ap_act = setup_net.driver.ap_last_activity_ms();
        if ap_act > last_activity_ms {
            last_activity_ms = ap_act;
        }
        if now_u64.saturating_sub(last_pwctrl_ms) >= 1_000 {
            last_pwctrl_ms = now_u64;
            phy_run_tx_pwctrl_bg();
        }

        for idx in 0..SETUP_TCP_COUNT {
            let sock = sockets.get_mut::<TcpSocket>(tcp_handles[idx]);
            let st = &mut tcp_states[idx];

            if !sock.is_open() {
                *st = SetupTcpWorkerState::default();
                sock.set_ack_delay(None);
                sock.set_timeout(None);
                let _ = sock.listen(80);
                continue;
            }

            if sock.is_active() {
                if st.conn_active_ms == 0 {
                    st.conn_active_ms = now_u64;
                }
            } else {
                st.conn_active_ms = 0;
            }

            if st.closing {
                let done = matches!(sock.state(), TcpState::TimeWait | TcpState::Closed);
                if !sock.is_active() || done || now_u64 >= st.close_deadline_ms {
                    sock.abort();
                    *st = SetupTcpWorkerState::default();
                }
            } else if st.tx_len > 0 {
                let peer_gone = !sock.is_active()
                    || !sock.may_send()
                    || matches!(
                        sock.state(),
                        TcpState::Closing
                            | TcpState::LastAck
                            | TcpState::TimeWait
                            | TcpState::Closed
                    )
                    || now_u64 >= st.close_deadline_ms;
                if peer_gone {
                    sock.abort();
                    *st = SetupTcpWorkerState::default();
                } else if st.tx_off < st.tx_len {
                    if sock.can_send() {
                        if let Ok(sent) = sock.send_slice(&resp_bufs[idx][st.tx_off..st.tx_len]) {
                            if sent > 0 {
                                st.tx_off += sent;
                                st.close_deadline_ms = now_u64.saturating_add(3_000);
                            }
                        }
                    }
                } else if sock.send_queue() == 0 {
                    sock.close();
                    st.closing = true;
                    st.close_deadline_ms = now_u64.saturating_add(1_500);
                }
            } else {
                if sock.can_recv() {
                    last_activity_ms = now_u64;
                    let avail = req_bufs[idx].len().saturating_sub(st.req_len);
                    if avail > 0 {
                        if let Ok(n) =
                            sock.recv_slice(&mut req_bufs[idx][st.req_len..st.req_len + avail])
                        {
                            st.req_len += n;
                        }
                    }
                    let peer_closed_rx = matches!(sock.state(), TcpState::CloseWait);
                    let mut req_complete = false;
                    let mut too_large = false;
                    if let Ok(parsed) = parse_http_request(&req_bufs[idx][..st.req_len]) {
                        let needed_body = parsed.content_length.unwrap_or(0);
                        let hdr_bytes = st.req_len.saturating_sub(parsed.body.len());
                        if hdr_bytes.saturating_add(needed_body) > req_bufs[idx].len() {
                            too_large = true;
                            req_complete = true;
                        } else if parsed.body.len() >= needed_body || peer_closed_rx {
                            req_complete = true;
                        }
                    } else if st.req_len >= req_bufs[idx].len() || peer_closed_rx {
                        req_complete = true;
                    }

                    if req_complete {
                        let resp_len = if too_large {
                            write_http_error(
                                resp_bufs[idx],
                                413,
                                "Content Too Large",
                                "Setup request exceeds buffer capacity\n",
                                false,
                            )
                            .map(|b| b.len())
                            .unwrap_or(0)
                        } else {
                            match handle_setup_http_request(
                                &req_bufs[idx][..st.req_len],
                                &hw_mac,
                                ap_ssid,
                                current_ssid,
                                resp_bufs[idx],
                            ) {
                                SetupHttpOutcome::Responded(n) => n,
                                SetupHttpOutcome::SavedAndReboot { resp_len, config } => {
                                    match ProvStore::save(&mut flash_adapter, &config) {
                                        Ok(seq) => {
                                            println!(
                                                "[SETUP] Provisioned config saved to flash \
                                                 (seq={}); rebooting after HTTP flush...",
                                                seq
                                            );
                                            if pending_reboot_at_ms.is_none() {
                                                pending_reboot_at_ms = Some(now_u64);
                                            }
                                            resp_len
                                        }
                                        Err(err) => {
                                            println!(
                                                "[SETUP] Flash save failed ({:?}); returning 500",
                                                err
                                            );
                                            write_http_error(
                                                resp_bufs[idx],
                                                500,
                                                "Internal Server Error",
                                                "Failed to save provisioning config to flash\n",
                                                false,
                                            )
                                            .map(|b| b.len())
                                            .unwrap_or(0)
                                        }
                                    }
                                }
                            }
                        };
                        st.req_len = 0;
                        if resp_len > 0 {
                            st.tx_len = resp_len;
                            st.tx_off = 0;
                            st.close_deadline_ms = now_u64.saturating_add(3_000);
                            if sock.can_send() {
                                if let Ok(sent) = sock.send_slice(&resp_bufs[idx][..st.tx_len]) {
                                    st.tx_off += sent;
                                }
                            }
                        } else {
                            sock.close();
                            st.closing = true;
                            st.close_deadline_ms = now_u64.saturating_add(1_500);
                        }
                    }
                }

                if st.tx_len == 0 && !st.closing {
                    let idle_or_half_closed = (st.conn_active_ms > 0
                        && now_u64.saturating_sub(st.conn_active_ms) > 5_000)
                        || (matches!(sock.state(), TcpState::CloseWait) && !sock.can_recv());
                    if idle_or_half_closed {
                        sock.abort();
                        *st = SetupTcpWorkerState::default();
                    }
                }
            }
        }

        if let Some(sched_ms) = pending_reboot_at_ms {
            let all_flushed = (0..SETUP_TCP_COUNT).all(|i| {
                tcp_states[i].tx_off >= tcp_states[i].tx_len
                    && sockets.get_mut::<TcpSocket>(tcp_handles[i]).send_queue() == 0
            });
            if all_flushed || now_u64.saturating_sub(sched_ms) >= SETUP_REBOOT_FLUSH_CAP_MS {
                let _ = setup_net.driver.poll(now_millis());
                let _ = iface.poll(smol_now(), &mut setup_net, &mut sockets);
                feed_rtc_wdt();
                delay.delay_millis(150);
                setup_net.driver.exit_ap_mode();
                println!("[SETUP] Rebooting into Wi-Fi STA mode...");
                quiesce_psram_and_software_reset();
            }
        }

        // Setup-Mode-only non-blocking UART0 CLI fallback.
        let mut drained = 0usize;
        while drained < 64 {
            let Some(byte) = poll_uart0_rx_byte() else {
                break;
            };
            drained += 1;
            last_activity_ms = now_u64;
            match uart_session.feed_byte(byte, &hw_mac, ap_ssid) {
                UartProvAction::None => {}
                UartProvAction::Reply(reply) => {
                    print!("{}", reply.as_str());
                }
                UartProvAction::SaveAndReboot(cfg) => {
                    match ProvStore::save(&mut flash_adapter, &cfg) {
                        Ok(seq) => {
                            println!(
                                "[SETUP] Provisioned config saved via UART0 (seq={}); rebooting...",
                                seq
                            );
                            feed_rtc_wdt();
                            delay.delay_millis(100);
                            setup_net.driver.exit_ap_mode();
                            quiesce_psram_and_software_reset();
                        }
                        Err(err) => {
                            println!("ERR: flash save failed ({:?})", err);
                        }
                    }
                }
                UartProvAction::Reboot => {
                    println!("[SETUP] Rebooting via UART0 command...");
                    feed_rtc_wdt();
                    delay.delay_millis(100);
                    setup_net.driver.exit_ap_mode();
                    quiesce_psram_and_software_reset();
                }
            }
        }

        let assoc_clients = setup_net.driver.ap_associated_clients();
        if now_u64.saturating_sub(last_rf_diag_ms) >= 15_000 {
            last_rf_diag_ms = now_u64;
            if let Some(st) = setup_net.driver.ap_stats() {
                println!(
                    "[SETUP][RF] bcn_ok={} bcn_err={} prb_rx={} prb_tx_ok={} \
                     prb_tx_err={} rx={} clients={} last_err={:?}",
                    st.beacons_ok,
                    st.beacons_err,
                    st.probe_req_rx,
                    st.probe_resp_ok,
                    st.probe_resp_err,
                    st.rx_frames,
                    assoc_clients,
                    st.last_tx_err
                );
            }
        }
        if pending_reboot_at_ms.is_none()
            && should_retry_sta_after_ap_idle(
                had_saved_creds,
                assoc_clients,
                now_u64,
                last_activity_ms,
            )
        {
            println!(
                "[SETUP] SoftAP fallback idle for 180s with 0 clients; rebooting to retry STA..."
            );
            feed_rtc_wdt();
            delay.delay_millis(100);
            setup_net.driver.exit_ap_mode();
            quiesce_psram_and_software_reset();
        }

        if iface.poll_delay(smol_now(), &sockets) != Some(smoltcp::time::Duration::ZERO) {
            delay.delay_millis(1);
        }
    }
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
                        // CONV-10: probe only the last SPSC chunk plus a margin.
                        if let Some(eoi_rel) = esp32_cam_ll::find_jpeg_eoi_tail(
                            &dst[soi_pos..written_len],
                            JPEG_SPSC_CHUNK_BYTES,
                        ) {
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
                            camera.set_frame_size(current_fs);
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
    net_device: &'static mut Esp32StaWifiDriver<HardwareWifiMmioBus>,
    mac: [u8; 6],
    ssid: &'static str,
    pass: &'static str,
    mut flash_adapter: EspFlashAdapter<'static>,
    mut kept_sccb: Option<B>,
    mut live_ov2640: Option<Ov2640>,
    ov2640_dma_ok: bool,
    cpu_ctrl_peri: esp_hal::peripherals::CPU_CTRL<'static>,
) -> ! {
    use esp_hal::delay::Delay;
    use esp_println::println;
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
    record_boot_breadcrumb_post_wifi(5, 0, 0);
    phy_set_max_tx_power_qdbm(WIFI_MAX_TX_POWER_QDBM);

    println!(
        concat!(
            "[KLIPPER-CAM-RS] [WIFI] Connecting to SSID '{}' ",
            "(MAC {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x})..."
        ),
        ssid, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]
    );
    let _ = FIRMWARE_RUNTIME.with_mut(FirmwareRuntime::on_wifi_sta_start);

    let mut boot_connected = connect_strongest_ap(&mut *net_device, ssid, pass, 15_000);
    if boot_connected.is_none() {
        delay.delay_millis(250);
        boot_connected = connect_strongest_ap(&mut *net_device, ssid, pass, 15_000);
    }
    if boot_connected.is_none() && cfg!(feature = "provisioning-ap") {
        record_boot_breadcrumb_post_wifi(7, 0, 0);
        println!(
            "[KLIPPER-CAM-RS] [SETUP] STA association failed after ~30s; \
             falling back to SoftAP Setup Mode (180s idle retry)"
        );
        run_provisioning_ap_loop(net_device, mac, ssid, true, flash_adapter);
    }

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
    }) = PSRAM_BUFFERS.take()
    else {
        quiesce_psram_and_software_reset();
    };
    let triple_fb = &mut triple_fb_owned;
    let p80_hdr_bufs = &mut p80_hdr_bufs_owned;
    let p80_resp_bufs = &mut p80_resp_bufs_owned;

    let Some(mut runtime_owned) = FIRMWARE_RUNTIME.take() else {
        quiesce_psram_and_software_reset();
    };
    let runtime = &mut runtime_owned;

    // BSSID of the current association; drives the runtime roam decision.
    let mut current_bssid: Option<[u8; 6]> = boot_connected.map(|info| info.bssid);
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

    // 7. Configure smoltcp Ethernet Interface according to IpMode (pure Static
    //    disables DHCPv4; pure DHCP omits any hardcoded static IP) + 4-socket
    //    Port 80 pool + 2-socket Port 81 pool (up to 7 sockets total).
    let ip_mode = runtime.config.ip_mode();
    let is_static_mode = matches!(ip_mode, IpMode::Static(_));
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
    let mut iface = NetInterface::new(iface_cfg, &mut *net_device, smol_now());

    let Some(socket_storage) = SOCKET_STORAGE.take() else {
        quiesce_psram_and_software_reset();
    };
    let mut sockets = SocketSet::new(&mut socket_storage[..]);

    let dhcp_handle = match ip_mode {
        IpMode::Dhcp => Some(sockets.add(dhcpv4::Socket::new())),
        IpMode::Static(_) => None,
    };
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
    let Some(p80_states) = P80_STATES.take() else {
        quiesce_psram_and_software_reset();
    };
    let mut ota_accum_owner: Option<usize> = None;
    let mut reboot_scheduled_at_ms: Option<u64> = None;

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
    let Some(p81_req_bufs) = P81_REQ_BUFS.take() else {
        quiesce_psram_and_software_reset();
    };
    let Some(p81_states) = P81_STATES.take() else {
        quiesce_psram_and_software_reset();
    };
    let Some(p81_hdr_buf) = P81_HDR_BUF.take() else {
        quiesce_psram_and_software_reset();
    };
    let Some(p81_err_buf) = P81_ERR_BUF.take() else {
        quiesce_psram_and_software_reset();
    };
    let Some(p81_drain_buf) = P81_DRAIN_BUF.take() else {
        quiesce_psram_and_software_reset();
    };
    let mut p81_stream_owner: Option<usize> = None;
    let mut p81_last_tx_progress_ms: u64 = 0;
    let mut p81_last_send_queue: usize = 0;
    let mut p81_hdr_len: usize = 0;
    let mut p81_pinned_slot: Option<PinnedFrameSlot> = None;
    let mut p81_tx_off: usize = 0;
    let mut p81_tx_len: usize = 0;

    let mut ip_configured = false;
    let mut current_ip_octets: Option<[u8; 4]> = None;
    let mut current_gw_octets: Option<[u8; 4]> = None;
    let mut static_ip_arp_pending = false;
    let mut last_rollback_retry_ms: u64 = 0;
    let mut connect_ms: u64 = now_millis();
    let mut last_ip_acquired_ms: u64 = connect_ms;
    let mut last_wifi_connected_ms: u64 = connect_ms;
    let mut last_reconnect_attempt_ms: u64 = connect_ms;
    let mut last_heartbeat_ms: u64 = connect_ms;
    let mut last_arp_announce_ms: u64 = connect_ms;
    let mut last_capture_ms: u64 = connect_ms;
    let mut last_p81_send_ms: u64 = connect_ms.saturating_sub(250);
    let mut post_snapshot_refresh_at_ms: Option<u64> = None;
    let mut last_applied_ctrl = klipper_cam_fw::CameraControlState::new_default();
    // Runtime roaming state (see `ROAM_*` in `klipper_cam_fw::ap_select`).
    let mut weak_link_timer = WeakLinkTimer::new();
    let mut last_roam_check_ms: u64 = connect_ms;
    let mut last_roam_scan_ms: Option<u64> = None;
    // Spacing between roam scans; grows while scans find no better AP.
    let mut roam_backoff = RoamBackoff::new();
    let mut spsc_asm = Core0SpscAssembler::new();
    let mut was_connected = net_device.is_connected();
    let mut last_pwctrl_ms: u64 = connect_ms;
    let mut cached_wifi_rssi: Option<i32>;
    let mut core1_stack_base: *const u8 = core::ptr::null();
    let mut core1_stack_len: usize = 0;

    if ov2640_dma_ok {
        let mut cpu_ctrl = CpuControl::new(cpu_ctrl_peri);
        if let Some(app_stack) = APP_CORE_STACK.take() {
            let stack_bytes = core::mem::size_of::<Stack<8192>>();
            let stack_raw_mut = core::ptr::from_mut(app_stack).cast::<u8>();
            // JUSTIFICATION: Paint 0xA5 canary across the 8 KB Core 1 stack
            // prior to booting APP_CPU so `/status` can measure live stack HWM.
            // SAFETY:
            // 1. Preconditions: `stack_raw_mut` points to 8192-byte static stack.
            // 2. Alignment: `Stack<8192>` is 16-byte aligned; `u8` requires 1.
            // 3. Provenance & Lifetime: Static `.bss` storage valid for `'static`.
            // 4. Aliasing: Core 1 is still parked in reset before start_app_core.
            // 5. Volatile Semantics: `paint_stack_sentinel` uses `write_volatile`.
            // 6. Panic Safety: Bounded byte loop cannot panic or unwind.
            unsafe {
                paint_stack_sentinel(stack_raw_mut, stack_bytes);
            }
            CORE1_ACTIVE.store(true, Ordering::Release);
            if let Ok(guard) = cpu_ctrl.start_app_core(app_stack, || core1_camera_producer_loop()) {
                // CRITICAL: Never drop AppCoreGuard or esp-hal parks APP_CPU!
                core::mem::forget(guard);
                core1_stack_base = APP_CORE_STACK.raw_ptr().cast::<u8>();
                core1_stack_len = stack_bytes;
                println!(
                    "[KLIPPER-CAM-RS] [CORE1] APP_CPU continuous I2S0 camera producer started"
                );
            } else {
                CORE1_ACTIVE.store(false, Ordering::Release);
            }
        }
    }
    // Own fallback `Esp32I2sCamera` locally only when Core 1 was not started.
    let mut fallback_i2s_cam = if !CORE1_ACTIVE.load(Ordering::Acquire) {
        HW_I2S_CAM.take()
    } else {
        None
    };

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
        let now_u64 = now_millis();
        let now = smoltcp::time::Instant::from_millis(now_u64 as i64);
        let _ = net_device.poll(now_u64);

        if net_device.is_connected() {
            if !was_connected {
                was_connected = true;
                connect_ms = now_u64;
                current_bssid = net_device.bssid();
                weak_link_timer.reset();
                roam_backoff.reset();
                p81_last_tx_progress_ms = connect_ms;
                println!(
                    "[KLIPPER-CAM-RS] [WIFI] Reconnected to AP (channel {}, aid {})",
                    net_device.channel(),
                    net_device.aid()
                );
                let announce_ip = current_ip_octets.zip(current_gw_octets);
                after_sta_associated(
                    runtime,
                    &mut static_ip_arp_pending,
                    &mut *net_device,
                    mac,
                    announce_ip,
                    is_static_mode,
                );
                if announce_ip.is_some() {
                    last_arp_announce_ms = connect_ms;
                }
            }
            last_wifi_connected_ms = now_u64;
            cached_wifi_rssi = Some(i32::from(net_device.rssi()));
            if now_u64.saturating_sub(last_pwctrl_ms) >= 1_000 {
                last_pwctrl_ms = now_u64;
                phy_run_tx_pwctrl_bg();
            }
            if now_u64.saturating_sub(last_roam_check_ms) >= ROAM_CHECK_INTERVAL_MS {
                last_roam_check_ms = now_u64;
                let r = net_device.rssi();
                cached_wifi_rssi = Some(i32::from(r));
                let weak_ms = weak_link_timer.update(now_u64, r, ROAM_TRIGGER_RSSI_DBM);
                let scan_allowed = last_roam_scan_ms
                    .is_none_or(|t| now_u64.saturating_sub(t) >= roam_backoff.interval_ms());
                if let Some(bssid) = current_bssid {
                    if weak_ms >= ROAM_WEAK_DURATION_MS && scan_allowed {
                        println!(
                            concat!(
                                "[KLIPPER-CAM-RS] [ROAM] Link weak for {} ms ",
                                "(rssi={}dBm < {}dBm); scanning..."
                            ),
                            weak_ms, r, ROAM_TRIGGER_RSSI_DBM
                        );
                        last_roam_scan_ms = Some(now_u64);
                        let decision = roam_scan_decision(&*net_device, bssid, r);
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
                            match roam_to_candidate(&mut *net_device, ssid, pass, &target) {
                                Some(info) => {
                                    println!(
                                        concat!(
                                            "[KLIPPER-CAM-RS] [ROAM] Roamed to AP ",
                                            "(channel {}, aid {})"
                                        ),
                                        info.channel, info.aid
                                    );
                                    current_bssid = Some(info.bssid);
                                    cached_wifi_rssi = Some(i32::from(net_device.rssi()));
                                    connect_ms = now_millis();
                                    last_wifi_connected_ms = connect_ms;
                                    // The roam blocked the loop for seconds;
                                    // give the port 81 stream a fresh 10 s
                                    // window instead of aborting it as stalled.
                                    p81_last_tx_progress_ms = connect_ms;
                                    let announce_ip = current_ip_octets.zip(current_gw_octets);
                                    after_sta_associated(
                                        runtime,
                                        &mut static_ip_arp_pending,
                                        &mut *net_device,
                                        mac,
                                        announce_ip,
                                        is_static_mode,
                                    );
                                    if announce_ip.is_some() {
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
                                    was_connected = false;
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
            if was_connected {
                was_connected = false;
                current_bssid = None;
                runtime.on_wifi_sta_disconnected();
            }
            cached_wifi_rssi = None;
            if now_u64.saturating_sub(last_wifi_connected_ms) >= 90_000
                && ota_accum_owner.is_none()
                && !runtime.server.is_reboot_scheduled()
            {
                println!(concat!(
                    "[KLIPPER-CAM-RS] [WATCHDOG] Wi-Fi disconnected > 90s; ",
                    "deasserting PSRAM CS# and rebooting..."
                ));
                quiesce_psram_and_software_reset();
            }
            if now_u64.saturating_sub(last_reconnect_attempt_ms) >= 5000 {
                last_reconnect_attempt_ms = now_u64;
                if matches!(net_device.state(), StaState::Idle) {
                    println!(
                        concat!(
                            "[KLIPPER-CAM-RS] [WIFI] Attempting reconnect to SSID ",
                            "'{}' (disconnected {}ms)..."
                        ),
                        ssid,
                        now_u64.saturating_sub(last_wifi_connected_ms)
                    );
                    let _ = net_device.connect(build_sta_config(ssid, pass, None), now_u64);
                }
            }
        }

        let _ = iface.poll(now, &mut *net_device, &mut sockets);

        // Apply configured IP mode: pure Static (DHCPv4 disabled) vs pure DHCP
        // (omits hardcoded 172.22.1.52/24, updates active_ip + ARP on lease).
        if let Some(dh) = dhcp_handle {
            let dhcp_sock = sockets.get_mut::<dhcpv4::Socket>(dh);
            match dhcp_sock.poll() {
                Some(dhcpv4::Event::Configured(cfg)) => {
                    let addr = cfg.address.address();
                    let octets = addr.octets();
                    let prefix_len = cfg.address.prefix_len();
                    iface.update_ip_addrs(|addrs| {
                        addrs.clear();
                        let _ = addrs.push(IpCidr::Ipv4(cfg.address));
                    });
                    let gw_octets = if let Some(router) = cfg.router {
                        let _ = iface.routes_mut().add_default_ipv4_route(router);
                        router.octets()
                    } else {
                        iface.routes_mut().remove_default_ipv4_route();
                        [octets[0], octets[1], octets[2], 1]
                    };
                    let dhcp_ip = Ipv4Addr::new(octets[0], octets[1], octets[2], octets[3]);
                    current_ip_octets = Some(octets);
                    current_gw_octets = Some(gw_octets);
                    runtime.on_wifi_sta_connected();
                    runtime.on_wifi_ip_confirmed(dhcp_ip);
                    static_ip_arp_pending = false;
                    ip_configured = true;
                    last_ip_acquired_ms = now_u64;
                    phy_set_max_tx_power_qdbm(WIFI_MAX_TX_POWER_QDBM);
                    send_arp_announce(&mut *net_device, mac, octets, gw_octets);
                    last_arp_announce_ms = now_u64;
                    println!(
                        concat!(
                            "[KLIPPER-CAM-RS] [NET] DHCP IP Configured: ",
                            "{}.{}.{}.{}/{} GW {}.{}.{}.{} (Port 80 x4 & Port 81 x2 Ready)"
                        ),
                        octets[0],
                        octets[1],
                        octets[2],
                        octets[3],
                        prefix_len,
                        gw_octets[0],
                        gw_octets[1],
                        gw_octets[2],
                        gw_octets[3]
                    );
                }
                Some(dhcpv4::Event::Deconfigured) => {
                    iface.update_ip_addrs(|addrs| addrs.clear());
                    iface.routes_mut().remove_default_ipv4_route();
                    current_ip_octets = None;
                    current_gw_octets = None;
                    ip_configured = false;
                    runtime.ip_confirmed = false;
                    println!("[KLIPPER-CAM-RS] [NET] DHCP lease lost; awaiting renewal");
                }
                None => {}
            }
            if ip_configured {
                last_ip_acquired_ms = now_u64;
            } else if net_device.is_connected()
                && now_u64.saturating_sub(last_ip_acquired_ms) >= 90_000
                && ota_accum_owner.is_none()
                && !runtime.server.is_reboot_scheduled()
            {
                println!(concat!(
                    "[KLIPPER-CAM-RS] [WATCHDOG] DHCP lease not acquired within 90s; ",
                    "deasserting PSRAM CS# and rebooting..."
                ));
                quiesce_psram_and_software_reset();
            }
        } else if !ip_configured {
            if let IpMode::Static(static_cfg) = ip_mode {
                let ip_octets = static_cfg.ip.octets();
                let gw_octets = static_cfg.gateway.octets();
                iface.update_ip_addrs(|addrs| {
                    addrs.clear();
                    let _ = addrs.push(IpCidr::new(
                        IpAddress::v4(ip_octets[0], ip_octets[1], ip_octets[2], ip_octets[3]),
                        static_cfg.prefix_len,
                    ));
                });
                let _ = iface.routes_mut().add_default_ipv4_route(Ipv4Address::new(
                    gw_octets[0],
                    gw_octets[1],
                    gw_octets[2],
                    gw_octets[3],
                ));
                current_ip_octets = Some(ip_octets);
                current_gw_octets = Some(gw_octets);
                if net_device.is_connected() {
                    runtime.on_wifi_sta_connected();
                }
                static_ip_arp_pending = true;
                runtime.wifi_state =
                    klipper_cam_fw::WifiStaState::ConnectedGotIp { ip: static_cfg.ip };
                runtime.server.set_active_ip(static_cfg.ip);
                runtime.webservers_started = true;
                ip_configured = true;
                last_ip_acquired_ms = now_u64;
                phy_set_max_tx_power_qdbm(WIFI_MAX_TX_POWER_QDBM);
                if net_device.is_connected() {
                    send_arp_announce(&mut *net_device, mac, ip_octets, gw_octets);
                    last_arp_announce_ms = now_u64;
                }
                println!(
                    concat!(
                        "[KLIPPER-CAM-RS] [NET] Static IP Configured: {}/{} ",
                        "GW {} (DHCP disabled; Port 80 x4 & Port 81 x2 Ready)"
                    ),
                    static_cfg.ip, static_cfg.prefix_len, static_cfg.gateway
                );
            }
        }

        let arp_interval_ms = if now_u64.saturating_sub(connect_ms) < 3000 {
            500
        } else {
            5000
        };
        if net_device.is_connected()
            && now_u64.saturating_sub(last_arp_announce_ms) >= arp_interval_ms
        {
            if let Some((ip_octets, gw_octets)) = current_ip_octets.zip(current_gw_octets) {
                last_arp_announce_ms = now_u64;
                send_arp_announce(&mut *net_device, mac, ip_octets, gw_octets);
            }
        }

        // ============================================================
        // Fallback Single-Core I2S0 Capture (only when Core 1 inactive)
        // ============================================================
        if !CORE1_ACTIVE.load(Ordering::Acquire) {
            let p80_actively_sending = p80_states.iter().any(|s| {
                s.tx_len > 0 && !s.closing && s.close_deadline_ms.saturating_sub(now_u64) > 3_500
            });
            let post_snap_due = post_snapshot_refresh_at_ms
                .is_some_and(|deadline| now_u64 >= deadline)
                && !p80_actively_sending
                && (now_u64.saturating_sub(last_capture_ms) >= 15);
            let idle_refresh_due = p81_stream_owner.is_none()
                && !p80_actively_sending
                && (now_u64.saturating_sub(last_capture_ms) >= 2_000);
            if ov2640_dma_ok
                && ota_accum_owner.is_none()
                && !runtime.server.is_reboot_scheduled()
                && p81_stream_owner.is_none()
                && (post_snap_due || idle_refresh_due)
            {
                post_snapshot_refresh_at_ms = None;
                last_capture_ms = now_u64;
                if let Some(ref mut hw_i2s_cam) = fallback_i2s_cam {
                    if capture_live_hd_jpeg_with_diag(
                        hw_i2s_cam,
                        triple_fb,
                        &mut runtime.server.camera,
                        runtime.server.control_state.framesize,
                        &mut || {
                            poll_net_burst(&mut iface, net_device, &mut sockets);
                        },
                    )
                    .is_ok()
                    {
                        last_capture_ms = now_millis();
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
                    st.conn_active_ms = now_u64;
                }
            } else {
                st.conn_active_ms = 0;
            }

            if st.closing {
                let finished_cleanly =
                    matches!(sock80.state(), TcpState::TimeWait | TcpState::Closed);
                if !sock80.is_active() || finished_cleanly || now_u64 >= st.close_deadline_ms {
                    let was_snapshot = st.is_snapshot;
                    if was_snapshot && post_snapshot_refresh_at_ms.is_none() {
                        post_snapshot_refresh_at_ms = Some(now_u64.saturating_add(50));
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
                    st.close_deadline_ms = now_u64.saturating_add(P80_TX_IDLE_TIMEOUT_MS);
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
                    || p80_tx_deadline_exceeded(now_u64, st.tx_started_ms, st.close_deadline_ms);
                if peer_disconnected {
                    let was_snapshot = st.is_snapshot;
                    if was_snapshot && post_snapshot_refresh_at_ms.is_none() {
                        post_snapshot_refresh_at_ms = Some(now_u64.saturating_add(50));
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
                            st.close_deadline_ms = now_u64.saturating_add(P80_TX_IDLE_TIMEOUT_MS);
                            st.last_send_queue = sock80.send_queue();
                        }
                    }
                } else if sock80.send_queue() == 0 {
                    if st.is_snapshot {
                        st.is_snapshot = false;
                        post_snapshot_refresh_at_ms = Some(now_u64.saturating_add(2));
                    }
                    sock80.close();
                    st.closing = true;
                    st.close_deadline_ms = now_u64.saturating_add(2_000);
                }
            } else {
                let mut aborted_early = false;
                if sock80.can_recv() {
                    if st.req_started_ms == 0 {
                        st.req_started_ms = now_u64;
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

                        // M-081 & M-171: Enforce `check_ota_auth` unconditionally
                        // on EVERY `POST /update` once headers parse, before
                        // accumulator promotion, Core 1 pause, or flash read.
                        if let Ok(parsed) = parse_http_request(&p80_hdr_bufs[idx][..st.req_len]) {
                            let is_update_post =
                                parsed.method == HttpMethod::Post && parsed.path == "/update";
                            let needed_body = match parsed.content_length {
                                Some(cl) if cl > 0 => cl,
                                _ => 0,
                            };
                            let hdr_bytes = st.req_len.saturating_sub(parsed.body.len());
                            let total_needed = hdr_bytes.saturating_add(needed_body);

                            let early_err: Option<(u16, &str, &str)> = if is_update_post {
                                if !runtime.server.is_allowed_host(parsed.host) {
                                    Some((
                                        403,
                                        "Forbidden",
                                        "Host header not allowed for /update\n",
                                    ))
                                } else if let Err(rej) = runtime
                                    .server
                                    .check_ota_auth(parsed.ota_token.unwrap_or(b""))
                                {
                                    Some((rej.status, rej.reason, rej.body))
                                } else if total_needed > p80_hdr_bufs[idx].len() {
                                    if total_needed > http80_ota_accum.len() {
                                        Some((
                                            413,
                                            "Content Too Large",
                                            "OTA payload exceeds buffer capacity\n",
                                        ))
                                    } else if ota_accum_owner.is_none() {
                                        http80_ota_accum[..st.req_len]
                                            .copy_from_slice(&p80_hdr_bufs[idx][..st.req_len]);
                                        st.using_ota_accum = true;
                                        st.req_started_ms = now_u64;
                                        st.ota_rx_started_ms = now_u64;
                                        ota_accum_owner = Some(idx);
                                        None
                                    } else if ota_accum_owner != Some(idx) {
                                        Some((
                                            503,
                                            "Service Unavailable",
                                            "OTA update in progress on another connection\n",
                                        ))
                                    } else {
                                        None
                                    }
                                } else {
                                    None
                                }
                            } else if total_needed > p80_hdr_bufs[idx].len() {
                                Some((
                                    413,
                                    "Content Too Large",
                                    "Payload exceeds maximum buffer capacity\n",
                                ))
                            } else {
                                None
                            };

                            if let Some((status, reason, body)) = early_err {
                                if let Ok(err) = write_http_error(
                                    p80_resp_bufs[idx],
                                    status,
                                    reason,
                                    body,
                                    false,
                                ) {
                                    st.req_len = 0;
                                    st.tx_len = err.len();
                                    st.tx_off = 0;
                                    st.tx_started_ms = now_u64;
                                    st.close_deadline_ms =
                                        now_u64.saturating_add(P80_TX_IDLE_TIMEOUT_MS);
                                    enqueue_tcp_sliding_window(
                                        sock80,
                                        p80_resp_bufs[idx],
                                        &mut st.tx_off,
                                        st.tx_len,
                                    );
                                    st.last_send_queue = sock80.send_queue();
                                } else {
                                    // M-171: Never fall through on error write fail.
                                    sock80.abort();
                                    if ota_accum_owner == Some(idx) {
                                        ota_accum_owner = None;
                                    }
                                    *st = Port80WorkerState::default();
                                    aborted_early = true;
                                }
                            }
                        }
                    }

                    if !aborted_early && st.using_ota_accum && sock80.can_recv() {
                        let avail = http80_ota_accum.len().saturating_sub(st.req_len);
                        if avail > 0 {
                            if let Ok(n) = sock80
                                .recv_slice(&mut http80_ota_accum[st.req_len..st.req_len + avail])
                            {
                                if n > 0 {
                                    st.req_len += n;
                                    st.req_started_ms = now_u64;
                                }
                            }
                        }
                    }

                    if !aborted_early && st.tx_len == 0 {
                        let mut req_complete = false;
                        let mut is_ota_post = false;
                        let mut is_snapshot_req = false;
                        let mut ota_auth_rej: Option<klipper_cam_fw::http::HttpRejection> = None;
                        let peer_closed_rx = matches!(sock80.state(), TcpState::CloseWait);
                        {
                            let req_slice: &[u8] = if st.using_ota_accum {
                                &http80_ota_accum[..st.req_len]
                            } else {
                                &p80_hdr_bufs[idx][..st.req_len]
                            };
                            if let Ok(parsed) = parse_http_request(req_slice) {
                                let needed_body = match parsed.content_length {
                                    Some(cl) if cl > 0 => cl.min(OTA_PARTITION_MAX_SIZE + 4096),
                                    _ => 0,
                                };
                                if parsed.body.len() >= needed_body || peer_closed_rx {
                                    req_complete = true;
                                    if parsed.method == HttpMethod::Post && parsed.path == "/update"
                                    {
                                        is_ota_post = true;
                                        if !runtime.server.is_allowed_host(parsed.host) {
                                            ota_auth_rej =
                                                Some(klipper_cam_fw::http::HttpRejection {
                                                    status: 403,
                                                    reason: "Forbidden",
                                                    body: "Host header not allowed for /update\n",
                                                });
                                        } else if let Err(rej) = runtime
                                            .server
                                            .check_ota_auth(parsed.ota_token.unwrap_or(b""))
                                        {
                                            ota_auth_rej = Some(rej);
                                        }
                                    }
                                    if parsed.method == HttpMethod::Get
                                        && (parsed.path == "/snapshot"
                                            || (parsed.path == "/"
                                                && parsed.query.is_some_and(|q| {
                                                    klipper_cam_fw::http::query_has_pair(
                                                        q,
                                                        "action=snapshot",
                                                    )
                                                })))
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
                                // 1. Preconditions: base and len came from APP_CORE_STACK.
                                // 2. Alignment: `u8` has alignment 1.
                                // 3. Provenance & Lifetime: `'static` `.bss` stack memory.
                                // 4. Aliasing: Raw volatile byte reads form zero refs.
                                // 5. Volatile Semantics: `read_volatile` reads SRAM.
                                // 6. Panic Safety: Bounded scan cannot panic or unwind.
                                unsafe {
                                    measure_stack_hwm_volatile(core1_stack_base, core1_stack_len)
                                }
                            } else {
                                0
                            };
                            runtime.server.set_extended_telemetry(
                                now_u64 as i64,
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

                            let handle_res = if is_ota_post {
                                if let Some(rej) = ota_auth_rej {
                                    write_http_error(
                                        p80_resp_bufs[idx],
                                        rej.status,
                                        rej.reason,
                                        rej.body,
                                        false,
                                    )
                                    .map(|s| s.len())
                                } else if let Ok(outer_guard) =
                                    klipper_cam_fw::Core1PauseGuard::new(&XtensaCore1Park)
                                {
                                    let booted_slot = runtime.server.ota_manager.running_slot();
                                    let res = if runtime
                                        .server
                                        .ota_manager
                                        .pending_reboot_slot()
                                        .is_some()
                                    {
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
                                        match klipper_cam_fw::OtaDataManager::load_from_flash(
                                            &mut flash_adapter,
                                        ) {
                                            Ok(mut loaded) => {
                                                if loaded.active_slot() == booted_slot {
                                                    loaded.set_running_slot_from_mmu(booted_slot);
                                                    runtime.server.ota_manager = loaded;
                                                }
                                                runtime
                                                    .server
                                                    .handle_raw_request_with_flash(
                                                        ServerPort::Control,
                                                        req_slice,
                                                        p80_resp_bufs[idx],
                                                        &mut flash_adapter,
                                                    )
                                                    .map(|s| s.len())
                                            }
                                            Err(err) => {
                                                OTA_FLASH_ERRORS.fetch_add(1, Ordering::Relaxed);
                                                println!(
                                                    concat!(
                                                        "[KLIPPER-CAM-RS] [OTA] otadata load ",
                                                        "failed ({:?}); refusing OTA"
                                                    ),
                                                    err
                                                );
                                                write_http_error(
                                                    p80_resp_bufs[idx],
                                                    503,
                                                    "Service Unavailable",
                                                    "503 Failed to read otadata; retry OTA\n",
                                                    false,
                                                )
                                                .map(|s| s.len())
                                            }
                                        }
                                    };
                                    if runtime.server.is_reboot_scheduled() {
                                        // M-019: Keep Core 1 parked across TX flush and reset.
                                        core::mem::forget(outer_guard);
                                        if reboot_scheduled_at_ms.is_none() {
                                            reboot_scheduled_at_ms = Some(now_u64);
                                        }
                                    } else if outer_guard.resume().is_err() {
                                        println!(
                                            "camera: Core 1 resume timed out after aborted OTA"
                                        );
                                    }
                                    res
                                } else {
                                    println!(
                                        "[KLIPPER-CAM-RS] [CORE1] pause timeout; refusing OTA POST"
                                    );
                                    write_http_error(
                                        p80_resp_bufs[idx],
                                        503,
                                        "Service Unavailable",
                                        "503 Camera core busy; retry OTA\n",
                                        false,
                                    )
                                    .map(|s| s.len())
                                }
                            } else if is_snapshot_req {
                                // M-018: Pin snapshot slot, copy synchronously, then unpin.
                                st.is_snapshot = true;
                                let snap_tok = triple_fb.pin_latest_snapshot_slot();
                                let res = write_snapshot_http_response(
                                    triple_fb.slot_bytes(&snap_tok),
                                    p80_resp_bufs[idx],
                                );
                                triple_fb.unpin_slot(snap_tok);
                                res
                            } else {
                                runtime
                                    .server
                                    .handle_raw_request_with_flash(
                                        ServerPort::Control,
                                        req_slice,
                                        p80_resp_bufs[idx],
                                        &mut flash_adapter,
                                    )
                                    .map(|s| s.len())
                            };

                            if st.using_ota_accum {
                                st.using_ota_accum = false;
                                st.ota_rx_started_ms = 0;
                                if ota_accum_owner == Some(idx) {
                                    ota_accum_owner = None;
                                }
                            }

                            match handle_res {
                                Ok(resp_len) => {
                                    if let Ok(view) =
                                        HttpResponseView::parse(&p80_resp_bufs[idx][..resp_len])
                                    {
                                        runtime.on_http_request_served_status(view.status_code);
                                        if static_ip_arp_pending
                                            && (200..=299).contains(&view.status_code)
                                        {
                                            if let Some(ip_octets) = current_ip_octets {
                                                runtime.on_wifi_ip_confirmed(Ipv4Addr::new(
                                                    ip_octets[0],
                                                    ip_octets[1],
                                                    ip_octets[2],
                                                    ip_octets[3],
                                                ));
                                            }
                                            static_ip_arp_pending = false;
                                        }
                                    }
                                    st.req_len = 0;
                                    st.tx_len = resp_len;
                                    st.tx_off = 0;
                                    st.tx_started_ms = now_u64;
                                    st.close_deadline_ms =
                                        now_u64.saturating_add(P80_TX_IDLE_TIMEOUT_MS);
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
                                    st.close_deadline_ms = now_u64.saturating_add(2_000);
                                }
                            }
                        }
                    }
                }

                if !aborted_early && st.tx_len == 0 && !st.closing {
                    let idle_or_half_closed = (!st.using_ota_accum
                        && st.conn_active_ms > 0
                        && now_u64.saturating_sub(st.conn_active_ms) > 3_000)
                        || (st.using_ota_accum
                            && st.req_len > 0
                            && p80_ota_rx_expired(
                                now_u64,
                                st.req_started_ms,
                                st.ota_rx_started_ms,
                            ))
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
            now_u64,
            &mut last_rollback_retry_ms,
        );

        // Apply pending camera control updates from `/control`. Decoupled from
        // slow `/snapshot` TX so slow snapshot readers cannot starve `/control`
        // (`M-082` / `C3-016`).
        let p80_tx_active = p80_states.iter().any(|s| s.tx_len > 0 && !s.closing);
        let p80_ctrl_tx_active = p80_states
            .iter()
            .any(|s| s.tx_len > 0 && !s.closing && !s.is_snapshot);
        if p80_tx_active {
            poll_net_burst(&mut iface, net_device, &mut sockets);
        }
        if !p80_ctrl_tx_active
            && (runtime.server.has_pending_control_update()
                || last_applied_ctrl.quality_needs_reapply)
        {
            match klipper_cam_fw::Core1PauseGuard::new(&XtensaCore1Park) {
                Err(_) => {
                    println!("camera: Core 1 pause timed out; deferring /control update");
                }
                Ok(guard) => {
                    if let (Some(ref mut sccb), Some(ref mut ov)) =
                        (&mut kept_sccb, &mut live_ov2640)
                    {
                        let mut applier = Ov2640ControlApplier(ov, sccb);
                        let errs = runtime
                            .server
                            .reconcile_controls(&mut applier, &mut last_applied_ctrl);
                        for _ in 0..errs {
                            FW_STATS.inc_sccb_errors();
                        }
                    } else if let Some(ctrl) = runtime.server.take_pending_control_update() {
                        last_applied_ctrl = ctrl;
                        last_applied_ctrl.quality_needs_reapply = false;
                    } else {
                        last_applied_ctrl.quality_needs_reapply = false;
                    }
                    // JUSTIFICATION: Drain stale SPSC chunks while Core 1 is
                    // parked after applying camera control changes (`M-017`).
                    // SAFETY: Core 0 is sole SPSC consumer and outside `pop_with`.
                    unsafe {
                        CORE1_SPSC.reset_consumer();
                    }
                    spsc_asm = Core0SpscAssembler::new();
                    post_snapshot_refresh_at_ms = Some(now_u64.saturating_add(150));
                    if guard.resume().is_err() {
                        println!("camera: Core 1 resume timed out after /control update");
                    }
                }
            }
        }

        // Reboot if OTA firmware update completed and all Port 80 TX buffers
        // have flushed (`tx_off >= tx_len` and `send_queue == 0`, `M-171`), or
        // once the bounded reboot force-timeout expires (`M-082`).
        if runtime.server.is_reboot_scheduled() {
            let sched_ms = *reboot_scheduled_at_ms.get_or_insert(now_u64);
            let all_flushed = (0..PORT80_SOCKET_COUNT).all(|i| {
                p80_states[i].tx_off >= p80_states[i].tx_len
                    && sockets.get_mut::<TcpSocket>(tcp80_handles[i]).send_queue() == 0
            });
            let force_expired = now_u64.saturating_sub(sched_ms) >= OTA_REBOOT_FORCE_TIMEOUT_MS;
            if all_flushed || force_expired {
                let _ = net_device.poll(now_millis());
                let _ = iface.poll(smol_now(), &mut *net_device, &mut sockets);
                delay.delay_millis(250);
                if runtime.server.last_ota_target_slot().is_some() {
                    println!(concat!(
                        "[KLIPPER-CAM-RS] [OTA] Flash complete, deasserting ",
                        "PSRAM CS# and rebooting into updated slot..."
                    ));
                } else {
                    println!(concat!(
                        "[KLIPPER-CAM-RS] [SETUP] Provisioning config saved, ",
                        "deasserting PSRAM CS# and rebooting..."
                    ));
                }
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
                p81_last_tx_progress_ms = now_u64;
            }
            let tx_stalled = (p81_tx_off < p81_tx_len || cur_q > 0)
                && (now_u64.saturating_sub(p81_last_tx_progress_ms) >= 10_000);
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
                if let Some(tok) = p81_pinned_slot.take() {
                    triple_fb.unpin_slot(tok);
                }
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
                        let _ = sock81.recv_slice(p81_drain_buf);
                    }
                    if p81_tx_off < p81_tx_len {
                        if let Some(ref tok) = p81_pinned_slot {
                            let prev_off = p81_tx_off;
                            enqueue_p81_zero_copy(
                                sock81,
                                &p81_hdr_buf[..p81_hdr_len],
                                triple_fb.slot_bytes(tok),
                                &mut p81_tx_off,
                            );
                            if p81_tx_off != prev_off {
                                p81_last_tx_progress_ms = now_u64;
                                p81_last_send_queue = sock81.send_queue();
                                enqueued_more = true;
                            }
                        }
                    }
                    p81_tx_off >= p81_tx_len
                };

                if enqueued_more {
                    poll_net_burst(&mut iface, net_device, &mut sockets);
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
                    // Release previous frame pin before promoting/capturing
                    // so `promote_ready_to_active` can swap `read_idx` (`M-018`).
                    if let Some(tok) = p81_pinned_slot.take() {
                        triple_fb.unpin_slot(tok);
                    }
                    let seq_before_capture = triple_fb.frame_seq();
                    if !CORE1_ACTIVE.load(Ordering::Acquire)
                        && ov2640_dma_ok
                        && !triple_fb.has_new_ready()
                        && ota_accum_owner.is_none()
                        && !runtime.server.is_reboot_scheduled()
                    {
                        if let Some(ref mut hw_i2s_cam) = fallback_i2s_cam {
                            let _ = capture_live_hd_jpeg_with_diag(
                                hw_i2s_cam,
                                triple_fb,
                                &mut runtime.server.camera,
                                runtime.server.control_state.framesize,
                                &mut || {
                                    poll_net_burst(&mut iface, net_device, &mut sockets);
                                },
                            );
                        }
                    }

                    let got_fresh = triple_fb.promote_ready_to_active()
                        || triple_fb.frame_seq() != seq_before_capture;
                    if got_fresh
                        || (!ov2640_dma_ok && now_u64.saturating_sub(last_p81_send_ms) >= 200)
                    {
                        let tok = triple_fb.pin_active_slot();
                        let jpeg_len = tok.len();
                        runtime
                            .server
                            .camera
                            .set_frame_size(runtime.server.control_state.framesize);
                        if let Ok(part_hdr_len) =
                            write_mjpeg_part_header(jpeg_len, p81_hdr_buf).map(|h| h.len())
                        {
                            FW_STATS.inc_p81_frames_sent();
                            p81_hdr_len = part_hdr_len;
                            p81_tx_len = p81_hdr_len + jpeg_len;
                            p81_tx_off = 0;
                            {
                                let sock81 = sockets.get_mut::<TcpSocket>(tcp81_handles[idx]);
                                enqueue_p81_zero_copy(
                                    sock81,
                                    &p81_hdr_buf[..p81_hdr_len],
                                    triple_fb.slot_bytes(&tok),
                                    &mut p81_tx_off,
                                );
                                let sent_ms = now_millis();
                                last_p81_send_ms = sent_ms;
                                p81_last_tx_progress_ms = sent_ms;
                                p81_last_send_queue = sock81.send_queue();
                            }
                            p81_pinned_slot = Some(tok);
                            poll_net_burst(&mut iface, net_device, &mut sockets);
                            drain_core1_spsc_to_triple(
                                &mut spsc_asm,
                                triple_fb,
                                &mut runtime.server.camera,
                                runtime.server.control_state.framesize,
                                true,
                            );
                        } else {
                            triple_fb.unpin_slot(tok);
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
                                st.conn_active_ms = now_u64;
                            }
                        } else {
                            st.conn_active_ms = 0;
                        }

                        if st.closing {
                            let finished_cleanly =
                                matches!(sock81.state(), TcpState::TimeWait | TcpState::Closed);
                            if !sock81.is_active()
                                || finished_cleanly
                                || now_u64 >= st.close_deadline_ms
                            {
                                sock81.abort();
                                *st = Port81WorkerState::default();
                            }
                        } else {
                            if sock81.can_recv() {
                                if st.req_started_ms == 0 {
                                    st.req_started_ms = now_u64;
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
                                        if let Ok(resp) = runtime.server.handle_raw_request(
                                            ServerPort::Stream,
                                            &p81_req_bufs[idx][..st.req_len],
                                            p81_err_buf,
                                        ) {
                                            match sock81.send_slice(resp) {
                                                Ok(sent) if sent == resp.len() => {}
                                                Ok(_) | Err(_) => FW_STATS.inc_send_partial(),
                                            }
                                        }
                                        sock81.close();
                                        st.closing = true;
                                        st.close_deadline_ms = now_u64.saturating_add(2_000);
                                    }
                                } else if st.req_len >= p81_req_bufs[idx].len()
                                    || matches!(sock81.state(), TcpState::CloseWait)
                                {
                                    sock81.close();
                                    st.closing = true;
                                    st.close_deadline_ms = now_u64.saturating_add(2_000);
                                }
                            }

                            if !preempt_and_start && !st.closing {
                                let idle_or_half_closed = (st.conn_active_ms > 0
                                    && now_u64.saturating_sub(st.conn_active_ms) > 3_000)
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
                    if let Some(tok) = p81_pinned_slot.take() {
                        triple_fb.unpin_slot(tok);
                    }
                    p81_tx_off = 0;
                    p81_tx_len = 0;
                    p81_last_send_queue = 0;
                    runtime
                        .server
                        .stream_active_flag()
                        .store(false, Ordering::Release);

                    let _ = triple_fb.promote_ready_to_active();
                    let tok = triple_fb.pin_active_slot();
                    let initial_jpeg_len = tok.len();
                    runtime
                        .server
                        .camera
                        .set_frame_size(runtime.server.control_state.framesize);

                    let mut started_ok = false;
                    if let Some(guard) =
                        StreamSessionGuard::try_acquire(runtime.server.stream_active_flag())
                    {
                        let stream_hdr_res =
                            guard.write_stream_headers(p81_hdr_buf).map(|b| b.len());
                        if let Ok(stream_hdr_len) = stream_hdr_res {
                            let part_hdr_res = write_mjpeg_part_header(
                                initial_jpeg_len,
                                &mut p81_hdr_buf[stream_hdr_len..],
                            )
                            .map(|h| h.len());
                            if let Ok(part_hdr_len) = part_hdr_res {
                                p81_hdr_len = stream_hdr_len + part_hdr_len;
                                p81_tx_len = p81_hdr_len + initial_jpeg_len;
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
                            triple_fb.slot_bytes(&tok),
                            &mut p81_tx_off,
                        );
                        p81_pinned_slot = Some(tok);
                        last_p81_send_ms = now_u64;
                        p81_last_tx_progress_ms = now_u64;
                        p81_last_send_queue = sock81.send_queue();
                    } else {
                        triple_fb.unpin_slot(tok);
                        sock81.close();
                        p81_states[idx].closing = true;
                        p81_states[idx].close_deadline_ms = now_u64.saturating_add(2_000);
                    }
                }
            }
        }
        FW_STATS.add_c0_p81_cycles(ccount().wrapping_sub(p81_t0));

        poll_net_burst(&mut iface, net_device, &mut sockets);

        if now_u64.saturating_sub(last_heartbeat_ms) >= 30_000 {
            last_heartbeat_ms = now_u64;
            println!(
                concat!(
                    "[KLIPPER-CAM-RS] Heartbeat: uptime={}ms connected={} ",
                    "stream_active={} frame_seq={} last_jpeg={}B ",
                    "c1_pushed={} c1_dropped={} ota_err={} c1_to={} vsync_to={}"
                ),
                now_u64,
                net_device.is_connected(),
                p81_stream_owner.is_some(),
                triple_fb.frame_seq(),
                triple_fb.active_len(),
                CORE1_SPSC.frames_pushed(),
                CORE1_SPSC.frames_dropped(),
                OTA_FLASH_ERRORS.load(Ordering::Relaxed),
                CORE1_SYNC_TIMEOUTS.load(Ordering::Relaxed),
                VSYNC_TIMEOUTS.load(Ordering::Relaxed)
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

    // Initialize internal DRAM heap (~80 KB strictly internal alongside static
    // WIFI_DRIVER_SLOT)
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 64 * 1024);
    esp_alloc::heap_allocator!(size: 16 * 1024);

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
    if !ensure_psram_qpi_mode_spi1() {
        println!("[KLIPPER-CAM-RS] [PSRAM] Initial QPI transition timed out; will verify");
    }
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
    // 6. Release Wi-Fi/RF force power-down, initialize calibrated PHY session,
    //    and bring up pure-Rust Wi-Fi STA MAC driver in static internal DRAM.
    esp_rtc_wifi_force_pd(false);
    let phy_session = XtensaPhySession::new();
    core::mem::forget(phy_session);
    feed_rtc_wdt();

    let Some(wifi_driver) = WIFI_DRIVER_SLOT.take() else {
        println!("[KLIPPER-CAM-RS] [WIFI-ERR] WIFI_DRIVER_SLOT already taken");
        delay.delay_millis(20);
        quiesce_psram_and_software_reset();
    };
    // JUSTIFICATION: Initialize the permanently pinned static `.bss` Wi-Fi STA
    // driver (`M-014`, `M-072`) after calibrated PHY session bring-up.
    // SAFETY:
    // 1. Preconditions: `XtensaPhySession::new()` ran above and was forgotten
    //    to keep PHY calibrated and active for `'static`.
    // 2. Alignment & Layout: `wifi_driver` is 4-byte aligned in static `.bss`.
    // 3. Provenance & Lifetime: `&'static mut` from `WIFI_DRIVER_SLOT` is
    //    permanently pinned and never moved or dropped after DMA ring arming.
    // 4. Aliasing: Single-shot `WIFI_DRIVER_SLOT::take()` grants sole `&mut`.
    // 5. Synchronization: Runs on Core 0 before Core 1 or interrupts use Wi-Fi.
    // 6. Panic Safety: Bare-metal firmware builds with `panic = "abort"`.
    let sta_mac = match unsafe { wifi_driver.init(None) } {
        Ok(sta_mac) => sta_mac,
        Err(err) => {
            println!("[KLIPPER-CAM-RS] [WIFI-ERR] wifi_driver.init failed: {err:?}");
            delay.delay_millis(20);
            quiesce_psram_and_software_reset();
        }
    };
    feed_rtc_wdt();
    // JUSTIFICATION: Wrap FlashStorage with multicore_ignore; Core1PauseGuard
    // parks APP_CPU in IRAM before every flash cache disable.
    // SAFETY:
    // 1. Preconditions: Runs after wifi_driver.init before APP_CPU starts.
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

    let mut fw_config = KlipperCamConfig::from_build_env();
    match ProvStore::load(&mut flash_adapter) {
        Ok(Some(loaded)) => {
            let prov_ref: &'static ProvisionedConfig = PROV_CONFIG_CELL.init(loaded);
            if let Err(err) = prov_ref.apply_to(&mut fw_config) {
                println!(
                    "[KLIPPER-CAM-RS] [PROV] Stored flash config invalid ({:?}); ignoring",
                    err
                );
            } else {
                println!(
                    "[KLIPPER-CAM-RS] [PROV] Loaded flash config: ssid='{}' ip_mode={:?}",
                    fw_config.network.wifi_ssid,
                    fw_config.ip_mode()
                );
            }
        }
        Ok(None) => {
            println!("[KLIPPER-CAM-RS] [PROV] No flash provisioning record found");
        }
        Err(err) => {
            println!(
                concat!(
                    "[KLIPPER-CAM-RS] [PROV] Flash provisioning load error ({:?}); ",
                    "using build defaults"
                ),
                err
            );
        }
    }
    let _ = FIRMWARE_RUNTIME.with_mut(|rt| {
        rt.config = fw_config;
        rt.server.config = fw_config;
        rt.server.set_device_mac(sta_mac);
    });
    feed_rtc_wdt();

    if should_enter_setup_on_boot(
        cfg!(feature = "provisioning-ap"),
        fw_config.has_sta_credentials(),
    ) {
        println!(
            "[KLIPPER-CAM-RS] [SETUP] No valid Wi-Fi STA credentials; entering SoftAP Setup Mode"
        );
        run_provisioning_ap_loop(wifi_driver, sta_mac, "", false, flash_adapter);
    }

    let (ssid, pass) = validate_wifi_station_config(&fw_config);
    feed_rtc_wdt();
    run_post_wifi_server_loop(
        wifi_driver,
        sta_mac,
        ssid,
        pass,
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

    #[test]
    fn test_core1_pause_state_nesting_epochs_and_timeout_rollback() {
        let st = Core1PauseState::new();
        assert!(!st.is_pause_requested());
        assert!(!st.is_faulted());

        // Timeout on 0 -> 1 must roll back depth so Core 1 is never stranded.
        assert!(!st.begin_pause(true, 4, || {}));
        assert!(!st.is_pause_requested());
        assert_eq!(st.req_epoch(), 1);
        assert_eq!(st.ack_epoch(), 0);

        // Simulate Core 1 acknowledging epoch 2 on the next pause request.
        st.ack_paused(2);
        // Wait: stale ACK from earlier cycle must be rejected and latch fault.
        assert!(!st.begin_pause(true, 4, || {}));
        assert!(st.is_faulted());
        assert!(!st.is_pause_requested());

        // Fresh state: test clean nested pause (0 -> 1 -> 2 -> 1 -> 0).
        let st2 = Core1PauseState::new();
        // Inactive Core 1 allows nesting without spinning.
        assert!(st2.begin_pause(false, 4, || {}));
        assert!(st2.begin_pause(false, 4, || {}));
        assert!(st2.is_pause_requested());
        assert!(st2.end_pause(false, 4, || {}));
        assert!(st2.is_pause_requested());
        assert!(st2.end_pause(false, 4, || {}));
        assert!(!st2.is_pause_requested());

        // Resume timeout when Core 1 stays parked latches fault and refuses
        // subsequent flash operations (preventing ABA race).
        let st3 = Core1PauseState::new();
        // Pre-seed depth=1, req=1, ack=1 as if Core 1 parked on epoch 1.
        assert!(st3.begin_pause(false, 1, || {}));
        st3.req_epoch.store(1, Ordering::Release);
        st3.ack_paused(1);
        // Nested pause while parked succeeds immediately without bumping epoch.
        assert!(st3.begin_pause(true, 1, || {}));
        // Inner resume decrements 2 -> 1 without unparking.
        assert!(st3.end_pause(true, 1, || {}));
        assert!(st3.is_pause_requested());
        // Outer resume 1 -> 0 times out because Core 1 did not clear ack_epoch.
        assert!(!st3.end_pause(true, 4, || {}));
        assert!(st3.is_faulted());
        assert!(!st3.begin_pause(true, 4, || {}));

        // M-019: On pause cycle >= 2, `req_epoch` MUST be bumped BEFORE
        // `depth` transitions `0 -> 1` so Core 1 observing `is_pause_requested()`
        // immediately reads the new epoch (never the stale previous epoch).
        let st4 = Core1PauseState::new();
        // Cycle 1: acknowledge epoch 1 inside feed_wdt callback (which runs only
        // after `depth: 0 -> 1` is published).
        assert!(st4.begin_pause(true, 16, || {
            assert!(st4.is_pause_requested());
            assert_eq!(st4.req_epoch(), 1);
            st4.ack_paused(st4.req_epoch());
        }));
        assert!(st4.end_pause(true, 16, || {
            assert!(!st4.is_pause_requested());
            st4.clear_ack();
        }));
        assert!(!st4.is_faulted());
        // Cycle 2: as soon as Core 1 observes `is_pause_requested() == true`,
        // `req_epoch()` MUST already be 2 (not stale 1).
        assert!(st4.begin_pause(true, 16, || {
            assert!(st4.is_pause_requested());
            assert_eq!(st4.req_epoch(), 2);
            st4.ack_paused(st4.req_epoch());
        }));
        assert!(!st4.is_faulted());
        assert!(st4.end_pause(true, 16, || {
            st4.clear_ack();
        }));
    }

    #[test]
    fn test_p80_wall_clock_caps_prevent_slowloris_lockout() {
        // Sliding idle deadline pushed forward, but wall-clock cap triggers.
        assert!(!p80_tx_deadline_exceeded(14_999, 1_000, 18_000));
        assert!(p80_tx_deadline_exceeded(16_000, 1_000, 20_000));
        // Idle deadline triggers when no progress is made.
        assert!(p80_tx_deadline_exceeded(5_000, 1_000, 5_000));

        // OTA RX slow-drip hits 120s wall-clock cap despite fresh rx bytes.
        assert!(!p80_ota_rx_expired(120_999, 120_900, 1_000));
        assert!(p80_ota_rx_expired(121_000, 120_950, 1_000));
        // OTA RX idle timeout triggers after 30s without bytes.
        assert!(p80_ota_rx_expired(32_001, 2_000, 1_000));
    }

    #[test]
    fn test_decode_ota_slot_from_mmu_entries_bounds() {
        // Invalid bit 0x100 on both entries returns None (no silent Ota0).
        assert_eq!(decode_ota_slot_from_mmu_entries(0x100, 0x100), None);
        // Out-of-partition page returns None.
        assert_eq!(decode_ota_slot_from_mmu_entries(0x00, 0xFF), None);
        // Valid ota_0 page (0x20000 >> 16 = 0x02).
        assert_eq!(
            decode_ota_slot_from_mmu_entries(0x02, 0x100),
            Some(OtaSlot::Ota0)
        );
        // Valid ota_1 page (0x1D0000 >> 16 = 0x1D).
        assert_eq!(
            decode_ota_slot_from_mmu_entries(0x100, 0x1D),
            Some(OtaSlot::Ota1)
        );
    }

    #[test]
    fn test_should_enter_setup_on_boot_decision_matrix() {
        assert!(should_enter_setup_on_boot(true, false));
        assert!(!should_enter_setup_on_boot(true, true));
        assert!(!should_enter_setup_on_boot(false, false));
        assert!(!should_enter_setup_on_boot(false, true));
    }

    #[test]
    fn test_should_retry_sta_after_ap_idle_threshold_and_guards() {
        // Unprovisioned board (`had_saved_creds == false`) stays in SoftAP.
        assert!(!should_retry_sta_after_ap_idle(false, 0, 500_000, 0));
        // Active client prevents idle fallback reboot.
        assert!(!should_retry_sta_after_ap_idle(true, 1, 500_000, 0));
        // Just below the 180_000 ms threshold does not trigger.
        assert!(!should_retry_sta_after_ap_idle(
            true,
            0,
            10_000 + AP_FALLBACK_IDLE_RETRY_MS - 1,
            10_000
        ));
        // Exactly at and beyond 180_000 ms triggers STA retry reboot.
        assert!(should_retry_sta_after_ap_idle(
            true,
            0,
            10_000 + AP_FALLBACK_IDLE_RETRY_MS,
            10_000
        ));
        assert!(should_retry_sta_after_ap_idle(true, 0, 1_000_000, 10_000));
        // Clock skew / saturating_sub guard never underflows.
        assert!(!should_retry_sta_after_ap_idle(true, 0, 5_000, 10_000));
    }

    #[test]
    fn test_setup_l2_udp_intercept_dhcp_dns_and_passthrough() {
        let ap_mac = [0xD4, 0xD4, 0xDA, 0x5B, 0xD4, 0xEC];
        let client_mac = [0xAA, 0xBB, 0xCC, 0x11, 0x22, 0x33];
        let mut dhcp = DhcpServer::new();
        let mut rx_frame = [0u8; 512];
        let mut tx_frame = [0u8; 512];

        // 1. DHCPDISCOVER -> DHCPOFFER broadcast Ethernet/IPv4/UDP reply.
        let mut dhcp_req = [0u8; 244];
        dhcp_req[0] = 1; // BOOTREQUEST
        dhcp_req[1] = 1; // Ethernet
        dhcp_req[2] = 6; // MAC len
        dhcp_req[4..8].copy_from_slice(&[0x12, 0x34, 0x56, 0x78]);
        dhcp_req[28..34].copy_from_slice(&client_mac);
        dhcp_req[236..240].copy_from_slice(&[99, 130, 83, 99]); // Magic cookie
        dhcp_req[240..244].copy_from_slice(&[53, 1, 1, 255]); // DHCPDISCOVER + END
        rx_frame[42..42 + dhcp_req.len()].copy_from_slice(&dhcp_req);
        let req_len = finalize_udp_eth_reply(
            &mut rx_frame,
            [0xFF; 6],
            client_mac,
            [0, 0, 0, 0],
            [255, 255, 255, 255],
            DHCP_CLIENT_PORT,
            DHCP_SERVER_PORT,
            dhcp_req.len(),
            1,
        )
        .expect("build DHCPDISCOVER frame");
        match process_setup_l2_udp_frame(
            &rx_frame[..req_len],
            ap_mac,
            AP_IP,
            &mut dhcp,
            2,
            &mut tx_frame,
        ) {
            SetupL2UdpAction::Reply(n) => {
                assert!(n >= 42 + 240);
                assert_eq!(&tx_frame[0..6], &[0xFF; 6]);
                assert_eq!(&tx_frame[6..12], &ap_mac);
                assert_eq!(&tx_frame[26..30], &AP_IP);
                assert_eq!(&tx_frame[30..34], &[255, 255, 255, 255]);
                assert_eq!(
                    u16::from_be_bytes([tx_frame[34], tx_frame[35]]),
                    DHCP_SERVER_PORT
                );
                assert_eq!(
                    u16::from_be_bytes([tx_frame[36], tx_frame[37]]),
                    DHCP_CLIENT_PORT
                );
                // IPv4 header checksum validates to 0.
                assert_eq!(
                    internet_checksum_fold(internet_checksum_add_slice(0, &tx_frame[14..34])),
                    0
                );
            }
            other => panic!("expected DHCP Reply, got {:?}", other),
        }

        // 2. DNS A query -> unicast Ethernet/IPv4/UDP answer pointing to 192.168.4.1.
        let dns_query: [u8; 29] = [
            0xAB, 0xCD, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, b'c',
            b'a', b'p', b't', b'i', b'v', b'e', 0x03, b'l', b'a', b'n', 0x00, 0x00, 0x01, 0x00,
            0x01,
        ];
        rx_frame[42..42 + dns_query.len()].copy_from_slice(&dns_query);
        let dns_req_len = finalize_udp_eth_reply(
            &mut rx_frame,
            ap_mac,
            client_mac,
            [192, 168, 4, 2],
            AP_IP,
            54321,
            DNS_PORT,
            dns_query.len(),
            3,
        )
        .expect("build DNS query frame");
        match process_setup_l2_udp_frame(
            &rx_frame[..dns_req_len],
            ap_mac,
            AP_IP,
            &mut dhcp,
            4,
            &mut tx_frame,
        ) {
            SetupL2UdpAction::Reply(n) => {
                assert!(n >= 42 + dns_query.len() + 16);
                assert_eq!(&tx_frame[0..6], &client_mac);
                assert_eq!(&tx_frame[26..30], &AP_IP);
                assert_eq!(&tx_frame[30..34], &[192, 168, 4, 2]);
                assert_eq!(u16::from_be_bytes([tx_frame[34], tx_frame[35]]), DNS_PORT);
                assert_eq!(u16::from_be_bytes([tx_frame[36], tx_frame[37]]), 54321);
                assert_eq!(&tx_frame[n - 4..n], &AP_IP);
            }
            other => panic!("expected DNS Reply, got {:?}", other),
        }

        // 3. Non-DHCP/DNS UDP port (e.g. port 80) passes straight to smoltcp.
        let tcp_or_other_len = finalize_udp_eth_reply(
            &mut rx_frame,
            ap_mac,
            client_mac,
            [192, 168, 4, 2],
            AP_IP,
            50000,
            80,
            4,
            5,
        )
        .expect("build non-setup UDP frame");
        assert_eq!(
            process_setup_l2_udp_frame(
                &rx_frame[..tcp_or_other_len],
                ap_mac,
                AP_IP,
                &mut dhcp,
                6,
                &mut tx_frame,
            ),
            SetupL2UdpAction::PassToStack
        );
    }

    #[test]
    fn test_setup_l2_udp_dhcpnak_broadcasts_when_src_ip_nonzero_and_bcast_flag_clear() {
        let ap_mac = [0xD4, 0xD4, 0xDA, 0x5B, 0xD4, 0xEC];
        let client_mac = [0xAA, 0xBB, 0xCC, 0x11, 0x22, 0x33];
        let wrong_subnet_ip = [172, 22, 1, 52];
        let mut dhcp = DhcpServer::new();
        let mut rx_frame = [0u8; 512];
        let mut tx_frame = [0u8; 512];

        // DHCPREQUEST from wrong subnet (172.22.1.52) with BOOTP broadcast flag B = 0.
        let mut dhcp_req = [0u8; 250];
        dhcp_req[0] = 1; // BOOTREQUEST
        dhcp_req[1] = 1; // Ethernet
        dhcp_req[2] = 6; // MAC len
        dhcp_req[4..8].copy_from_slice(&[0xCA, 0xFE, 0xBA, 0xBE]);
        dhcp_req[10..12].copy_from_slice(&[0x00, 0x00]); // B = 0
        dhcp_req[12..16].copy_from_slice(&wrong_subnet_ip); // ciaddr = 172.22.1.52
        dhcp_req[28..34].copy_from_slice(&client_mac);
        dhcp_req[236..240].copy_from_slice(&[99, 130, 83, 99]); // Magic cookie
        dhcp_req[240..243].copy_from_slice(&[53, 1, 3]); // DHCPREQUEST
        dhcp_req[243..249].copy_from_slice(&[50, 4, 172, 22, 1, 52]); // Requested IP
        dhcp_req[249] = 255; // END
        rx_frame[42..42 + dhcp_req.len()].copy_from_slice(&dhcp_req);

        let req_len = finalize_udp_eth_reply(
            &mut rx_frame,
            ap_mac,
            client_mac,
            wrong_subnet_ip,
            [255, 255, 255, 255],
            DHCP_CLIENT_PORT,
            DHCP_SERVER_PORT,
            dhcp_req.len(),
            1,
        )
        .expect("build wrong-subnet DHCPREQUEST frame");

        match process_setup_l2_udp_frame(
            &rx_frame[..req_len],
            ap_mac,
            AP_IP,
            &mut dhcp,
            2,
            &mut tx_frame,
        ) {
            SetupL2UdpAction::Reply(n) => {
                assert!(n >= 42 + 240);
                assert_eq!(&tx_frame[0..6], &[0xFF; 6]);
                assert_eq!(&tx_frame[6..12], &ap_mac);
                assert_eq!(&tx_frame[26..30], &AP_IP);
                assert_eq!(&tx_frame[30..34], &[255, 255, 255, 255]);
                assert_eq!(
                    u16::from_be_bytes([tx_frame[34], tx_frame[35]]),
                    DHCP_SERVER_PORT
                );
                assert_eq!(
                    u16::from_be_bytes([tx_frame[36], tx_frame[37]]),
                    DHCP_CLIENT_PORT
                );
                assert_eq!(tx_frame[42], 2); // BOOTREPLY
                assert_ne!(tx_frame[42 + 10] & 0x80, 0); // DHCPNAK sets B = 1
                assert!(tx_frame[42 + 240..n].windows(3).any(|w| w == [53, 1, 6]));
                assert_eq!(
                    internet_checksum_fold(internet_checksum_add_slice(0, &tx_frame[14..34])),
                    0
                );
            }
            other => panic!("expected DHCPNAK Reply, got {:?}", other),
        }
    }
}
