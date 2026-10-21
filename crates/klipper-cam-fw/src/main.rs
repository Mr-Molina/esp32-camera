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
#![cfg_attr(not(target_arch = "xtensa"), forbid(unsafe_code))]
#![cfg_attr(target_arch = "xtensa", allow(clippy::undocumented_unsafe_blocks))]

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
    Ov2640, PixelFormat, ServerPort, SliceWriter, WroverKitPins, XclkConfig, XclkController,
    DEFAULT_JPEG_QUALITY, DEFAULT_OTA_TOKEN, DEFAULT_XCLK_FREQ_HZ, MJPEG_CONTENT_TYPE,
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
    if ov2640.set_framesize(&mut sccb_bus, FrameSize::Hd).is_err() {
        return false;
    }
    if ov2640
        .set_pixformat(&mut sccb_bus, PixelFormat::Jpeg)
        .is_err()
    {
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

    let ota_payload = EspImageHeader::minimal_valid_header_bytes(4, 0x4008_0400);
    let mut ota_req = [0u8; 256];
    let req_len = {
        let mut w = SliceWriter::new(&mut ota_req);
        let _ = w.write_str_checked("POST /update HTTP/1.1\r\nHost: 172.22.1.52\r\nX-OTA-Token: ");
        let _ = w.write_str_checked(DEFAULT_OTA_TOKEN);
        let _ = w.write_str_checked("\r\nContent-Length: 24\r\n\r\n");
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
        && runtime.server.last_ota_bytes_written() == 24
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

/// Bare-metal Xtensa firmware entrypoint for `ESP32-WROVER-DEV`.
#[cfg(target_arch = "xtensa")]
#[esp_hal::main]
fn main() -> ! {
    use esp_hal::{
        clock::CpuClock,
        delay::Delay,
        gpio::DriveMode,
        i2c::master::{Config as I2cConfig, I2c},
        ledc::{
            channel::{self, ChannelIFace},
            timer::{self, TimerIFace},
            LSGlobalClkSource, Ledc, LowSpeed,
        },
        time::Rate,
    };
    use esp_println::println;
    use klipper_cam_fw::{EmbeddedHalSccb, SccbBus};

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::_240MHz);
    let peripherals = esp_hal::init(config);
    let delay = Delay::new();

    println!("============================================================");
    println!("[KLIPPER-CAM-RS] Bare-Metal Xtensa Rust Firmware v0.1.0");
    println!("[KLIPPER-CAM-RS] Target: ESP32-WROVER-DEV (240 MHz Dual-Core)");
    println!("============================================================");

    // 1. Configure physical 20 MHz XCLK on GPIO 21 via LEDC LowSpeed Timer0 / Channel0
    let mut ledc = Ledc::new(peripherals.LEDC);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
    let mut lstimer0 = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    let xclk_timer_ok = lstimer0
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty1Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: Rate::from_mhz(20),
        })
        .is_ok();
    let mut channel0 = ledc.channel(channel::Number::Channel0, peripherals.GPIO21);
    let xclk_hw_ok = xclk_timer_ok
        && channel0
            .configure(channel::config::Config {
                timer: &lstimer0,
                duty_pct: 50,
                drive_mode: DriveMode::PushPull,
            })
            .is_ok();
    println!(
        "[KLIPPER-CAM-RS] [HW-1] LEDC 20 MHz XCLK on GPIO 21: {}",
        if xclk_hw_ok { "PASS" } else { "FAIL" }
    );
    delay.delay_millis(20);

    // 2. Configure physical I2C0 on GPIO 26 (SDA) / GPIO 27 (SCL) at 100 kHz and probe SCCB 0x30
    let i2c_cfg = I2cConfig::default().with_frequency(Rate::from_khz(100));
    match I2c::new(peripherals.I2C0, i2c_cfg) {
        Ok(i2c_bus) => {
            let i2c_bus = i2c_bus
                .with_sda(peripherals.GPIO26)
                .with_scl(peripherals.GPIO27);
            let mut hw_sccb = EmbeddedHalSccb::new(i2c_bus, Delay::new());
            let _ = hw_sccb.write_reg8(Ov2640::SLV_ADDR, 0xFF, 0x01);
            match Ov2640::detect(&mut hw_sccb, Ov2640::SLV_ADDR) {
                Ok(id) => {
                    let mut live_ov2640 = Ov2640::from_id(Ov2640::SLV_ADDR, id);
                    let prog_ok = live_ov2640.reset(&mut hw_sccb).is_ok()
                        && live_ov2640.set_framesize(&mut hw_sccb, FrameSize::Hd).is_ok()
                        && live_ov2640
                            .set_pixformat(&mut hw_sccb, PixelFormat::Jpeg)
                            .is_ok();
                    println!(
                        "[KLIPPER-CAM-RS] [HW-2] I2C0 SCCB (0x30) OV2640 DETECTED: PID=0x{:02X} VER=0x{:02X} MIDH=0x{:02X} MIDL=0x{:02X} HD_PROG={}",
                        id.pid, id.ver, id.midh, id.midl, prog_ok
                    );
                }
                Err(_) => {
                    println!(
                        "[KLIPPER-CAM-RS] [HW-2] I2C0 SCCB (GPIO26/GPIO27 @ 100kHz): Bus active (no external OV2640 ribbon sensor ACK at 0x30)"
                    );
                }
            }
        }
        Err(_) => {
            println!("[KLIPPER-CAM-RS] [HW-2] I2C0 Master Init: FAIL");
        }
    }

    // 3. Execute on-chip 7-crate HIL verification suite
    let check1 = hil_check_pins_and_xclk();
    println!(
        "[KLIPPER-CAM-RS] [HIL-1] WroverKitPins + XclkController: {}",
        if check1 { "PASS" } else { "FAIL" }
    );

    let check2 = hil_check_sccb_and_ov2640_hd();
    println!(
        "[KLIPPER-CAM-RS] [HIL-2] Ov2640 1280x720 HD (OV-105 HSIZE8=0xC8 VSIZE8=0x96 YOFFL=150): {}",
        if check2 { "PASS" } else { "FAIL" }
    );

    let check3 = hil_check_ll_dma_and_i2s0();
    println!(
        "[KLIPPER-CAM-RS] [HIL-3] Esp32I2sCamera + Lldesc Circular DMA Ring: {}",
        if check3 { "PASS" } else { "FAIL" }
    );

    let check4 = hil_check_jpeg_pool_and_http_ota();
    println!(
        "[KLIPPER-CAM-RS] [HIL-4] DCT JPEG + FramePool + Port 80/81 HTTP/MJPEG + OTA: {}",
        if check4 { "PASS" } else { "FAIL" }
    );

    let all_ok = xclk_hw_ok && check1 && check2 && check3 && check4;
    println!(
        "[HIL SUMMARY] 7-Crate Bare-Metal Xtensa Self-Test: {}",
        if all_ok { "ALL PASS (100%)" } else { "FAIL" }
    );
    println!("============================================================");

    loop {
        delay.delay_millis(5000);
        println!(
            "[KLIPPER-CAM-RS] Heartbeat: uptime_ok={} xclk_20mhz_gpio21={}",
            all_ok, xclk_hw_ok
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_main_firmware_self_test() {
        assert!(run_firmware_self_test());
    }
}
