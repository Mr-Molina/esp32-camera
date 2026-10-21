//! Bare-metal `#![no_std]` Klipper 3D Printer Camera & OTA firmware library (`HTTP`, `MJPEG`, `OTA`).
//!
//! Replaces:
//! - `examples/camera_example/main/klipper_cam.c`
//! - `examples/camera_example/main/ota_minimal.c`
//! - `examples/camera_example/main/take_picture.c`
//! - `examples/camera_example/main/klipper_cam_config.h`
//! - `examples/camera_example/main/camera_pinout.h`

#![no_std]
#![forbid(unsafe_code)]

pub mod config;
pub mod http;
pub mod ota;

pub use config::{
    active_ota_token, active_wifi_pass, active_wifi_ssid, BoardVariant, CameraPinout, Ipv4Addr,
    KlipperCamConfig, NetworkConfig, StaticIpConfig, TakePictureConfig, AITHINKER_FLASH_LED_GPIO,
    AITHINKER_PINOUT, DEFAULT_CONTROL_PORT, DEFAULT_FB_COUNT, DEFAULT_JPEG_QUALITY,
    DEFAULT_OTA_TOKEN, DEFAULT_STREAM_PACING_MS, DEFAULT_STREAM_PORT, DEFAULT_WIFI_RECONNECT_MS,
    DEFAULT_XCLK_FREQ_HZ, ESP32S3_GOOUUU_PINOUT, ESP32S3_WROOM_PINOUT, ESP32S3_XIAO_PINOUT,
    WROVER_KIT_PINOUT,
};
pub use esp32_cam_conversions::{
    self, decode_jpeg_to_rgb565, decode_jpeg_to_rgb888, encode_grayscale_to_jpeg,
    encode_rgb888_to_jpeg, encode_yuv422_to_jpeg, frame_to_bmp, parse_jpeg_dimensions,
    verify_jpeg_eoi, verify_jpeg_soi,
};
pub use esp32_cam_hal::{
    self, AiThinkerPins, BoardPins, CameraConfig as HalCameraConfig,
    CameraDriver as HalCameraDriver, Esp32S3GoouuuPins, Esp32S3WroomPins, Esp32S3XiaoPins,
    FrameGuard as HalFrameGuard, FramePool, HalCameraSensor, HalOv2640Sensor, Initialized,
    Streaming, Uninitialized, WroverKitPins,
};
pub use esp32_cam_ll::{
    self, calc_dma_sizes, DmaElement, DmaSizeConfig, Esp32I2sCamera, Esp32S2I2sCamera,
    Esp32S3LcdCam, XclkConfig, XclkController,
};
pub use esp32_cam_sccb::{self, EmbeddedHalSccb, MockSccbBus, SccbBus, SccbTransaction};
pub use esp32_cam_sensors::{self, ov2640::Ov2640, CameraSensor};
pub use esp32_cam_types::{
    CameraError, CameraFbLocation, CameraGrabMode, CameraModel, CameraStatus, FrameSize,
    GainCeiling, GrabMode, PixelFormat, SensorId,
};

/// Zero-cost adapter bridging any [`esp32_cam_sensors::CameraSensor`] implementation
/// (such as [`Ov2640`]) into [`esp32_cam_hal::HalCameraSensor`] for [`HalCameraDriver`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct SensorHalAdapter<S>(pub S);

impl<S> SensorHalAdapter<S> {
    /// Wraps a [`CameraSensor`] driver for use with [`HalCameraDriver`].
    #[inline]
    pub const fn new(sensor: S) -> Self {
        Self(sensor)
    }

    /// Returns a shared reference to the inner sensor driver.
    #[inline]
    pub const fn inner(&self) -> &S {
        &self.0
    }

    /// Returns a mutable reference to the inner sensor driver.
    #[inline]
    pub fn inner_mut(&mut self) -> &mut S {
        &mut self.0
    }
}

impl<B: SccbBus, S: CameraSensor<B>> HalCameraSensor<B> for SensorHalAdapter<S> {
    #[inline]
    fn model(&self) -> CameraModel {
        self.0.model()
    }

    #[inline]
    fn id(&self) -> &SensorId {
        self.0.id()
    }

    #[inline]
    fn status(&self) -> &CameraStatus {
        self.0.status()
    }

    #[inline]
    fn status_mut(&mut self) -> &mut CameraStatus {
        self.0.status_mut()
    }

    #[inline]
    fn reset(&mut self, bus: &mut B) -> Result<(), CameraError> {
        self.0.reset(bus)
    }

    #[inline]
    fn init_status(&mut self, bus: &mut B) -> Result<(), CameraError> {
        self.0.init_status(bus)
    }

    #[inline]
    fn set_pixformat(&mut self, bus: &mut B, pixformat: PixelFormat) -> Result<(), CameraError> {
        self.0.set_pixformat(bus, pixformat)
    }

    #[inline]
    fn set_framesize(&mut self, bus: &mut B, framesize: FrameSize) -> Result<(), CameraError> {
        self.0.set_framesize(bus, framesize)
    }

    #[inline]
    fn set_gainceiling(&mut self, bus: &mut B, gainceiling: GainCeiling) -> Result<(), CameraError> {
        self.0.set_gainceiling(bus, gainceiling)
    }

    #[inline]
    fn set_quality(&mut self, bus: &mut B, quality: i32) -> Result<(), CameraError> {
        self.0.set_quality(bus, quality)
    }

    #[inline]
    fn set_bpc(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.0.set_bpc(bus, enable)
    }

    #[inline]
    fn set_wpc(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.0.set_wpc(bus, enable)
    }

    #[inline]
    fn set_lenc(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.0.set_lenc(bus, enable)
    }

    #[inline]
    fn set_hmirror(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.0.set_hmirror(bus, enable)
    }

    #[inline]
    fn set_vflip(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.0.set_vflip(bus, enable)
    }
}
pub use http::{
    format_mjpeg_part_header, format_ota_minimal_root, format_status_json, parse_http_request,
    CameraDriver, FrameGuard, HttpError, HttpMethod, HttpRequest, HttpResponseView,
    KlipperCamServer, ServerPort, SliceWriter, StreamSessionGuard, TelemetrySnapshot,
    KLIPPER_ROOT_STATUS_PAGE, MJPEG_BOUNDARY, MJPEG_CONTENT_TYPE, SYNTHETIC_HD_JFIF_FRAME,
};
pub use ota::{
    crc32_ieee, find_partition_by_label, validate_ota_image_header, verify_ota_token,
    verify_partition_table_integrity, EspImageHeader, OtaDataManager, OtaDataSelect, OtaError,
    OtaImgState, OtaSlot, PartitionEntry, PartitionSubType, PartitionType, ESP_IMAGE_HEADER_LEN,
    ESP_IMAGE_HEADER_MAGIC, ESP_IMAGE_MAX_SEGMENTS, ESP_IMAGE_MIN_SEGMENTS, NVS_OFFSET, NVS_SIZE,
    OTADATA_OFFSET, OTADATA_SECTOR_0_OFFSET, OTADATA_SECTOR_1_OFFSET, OTADATA_SIZE, OTA_0_OFFSET,
    OTA_0_SIZE, OTA_1_OFFSET, OTA_1_SIZE, OTA_CHUNK_SIZE, OTA_PARTITION_MAX_SIZE, PARTITION_TABLE,
    PHY_INIT_OFFSET, PHY_INIT_SIZE,
};

/// Wi-Fi station connection lifecycle state (`wifi_event_handler` in `klipper_cam.c`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum WifiStaState {
    /// Wi-Fi not yet initialized.
    #[default]
    Uninitialized,
    /// `WIFI_EVENT_STA_START` fired; connecting to AP.
    Connecting,
    /// `WIFI_EVENT_STA_DISCONNECTED` fired; non-blocking 2000ms reconnect timer scheduled (`APP-009`).
    DisconnectedReconnectScheduled { delay_ms: u32 },
    /// `IP_EVENT_STA_GOT_IP` fired; IP acquired, rollback cancelled, and HTTP servers active.
    ConnectedGotIp { ip: Ipv4Addr },
}

/// Top-level firmware lifecycle coordinator (`app_main` in `klipper_cam.c`).
pub struct FirmwareRuntime<'a> {
    pub server: KlipperCamServer<'a>,
    pub wifi_state: WifiStaState,
    pub webservers_started: bool,
    pub gpio4_flash_led_held_low: bool,
    pub rollback_cancelled: bool,
}

impl<'a> FirmwareRuntime<'a> {
    /// Initializes a new [`FirmwareRuntime`] with the given configuration and camera driver.
    pub const fn new(config: KlipperCamConfig, camera: CameraDriver<'a>) -> Self {
        let gpio4_low = config.board.has_gpio4_flash_led();
        Self {
            server: KlipperCamServer::new(config, camera),
            wifi_state: WifiStaState::Uninitialized,
            webservers_started: false,
            gpio4_flash_led_held_low: gpio4_low,
            rollback_cancelled: false,
        }
    }

    /// Creates the default `WroverKit` 1280x720 HD [`FirmwareRuntime`].
    pub const fn new_default() -> FirmwareRuntime<'static> {
        FirmwareRuntime::new(KlipperCamConfig::WROVER_KIT_HD, CameraDriver::new_hd_default())
    }

    /// Handles `WIFI_EVENT_STA_START`: transitions station to [`WifiStaState::Connecting`].
    pub fn on_wifi_sta_start(&mut self) {
        self.wifi_state = WifiStaState::Connecting;
    }

    /// Handles `WIFI_EVENT_STA_DISCONNECTED`: schedules the non-blocking 2-second reconnect timer (`APP-009`).
    pub fn on_wifi_sta_disconnected(&mut self) {
        self.wifi_state = WifiStaState::DisconnectedReconnectScheduled {
            delay_ms: self.server.config.network.reconnect_delay_ms,
        };
    }

    /// Handles `IP_EVENT_STA_GOT_IP`:
    /// 1. Records assigned station IPv4 address.
    /// 2. Cancels OTA rollback (`esp_ota_mark_app_valid_cancel_rollback`, `APP-003` / `FW-002`).
    /// 3. Starts the Port 80 and Port 81 HTTP servers if not already running.
    pub fn on_wifi_got_ip(&mut self, ip: Ipv4Addr) -> Result<(), OtaError> {
        self.wifi_state = WifiStaState::ConnectedGotIp { ip };
        let _ = self.server.on_wifi_got_ip_cancel_rollback()?;
        self.rollback_cancelled = true;
        self.webservers_started = true;
        Ok(())
    }
}

impl Default for FirmwareRuntime<'static> {
    #[inline]
    fn default() -> Self {
        Self::new_default()
    }
}

/// Periodic picture capture state machine replacing `examples/camera_example/main/take_picture.c`.
pub struct TakePictureRunner<'a> {
    pub config: TakePictureConfig,
    pub camera: CameraDriver<'a>,
    pub pictures_taken: u32,
    pub last_picture_size: usize,
}

impl<'a> TakePictureRunner<'a> {
    /// Creates a new [`TakePictureRunner`].
    pub const fn new(config: TakePictureConfig, camera: CameraDriver<'a>) -> Self {
        Self {
            config,
            camera,
            pictures_taken: 0,
            last_picture_size: 0,
        }
    }

    /// Executes one capture iteration (`esp_camera_fb_get` + size record + `esp_camera_fb_return`).
    pub fn capture_once(&mut self) -> Result<usize, CameraError> {
        let frame = self.camera.acquire_frame()?;
        let size = frame.len();
        drop(frame);
        self.pictures_taken = self.pictures_taken.saturating_add(1);
        self.last_picture_size = size;
        Ok(size)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_firmware_runtime_boot_and_wifi_lifecycle() {
        let mut rt = FirmwareRuntime::new_default();
        assert!(!rt.gpio4_flash_led_held_low);
        assert!(!rt.webservers_started);
        assert_eq!(rt.wifi_state, WifiStaState::Uninitialized);

        rt.on_wifi_sta_start();
        assert_eq!(rt.wifi_state, WifiStaState::Connecting);

        rt.on_wifi_sta_disconnected();
        assert_eq!(
            rt.wifi_state,
            WifiStaState::DisconnectedReconnectScheduled { delay_ms: 2_000 }
        );

        let ip = Ipv4Addr::new(172, 22, 1, 52);
        assert!(rt.on_wifi_got_ip(ip).is_ok());
        assert_eq!(rt.wifi_state, WifiStaState::ConnectedGotIp { ip });
        assert!(rt.webservers_started);
        assert!(rt.rollback_cancelled);
    }

    #[test]
    fn test_take_picture_runner_lifecycle() {
        let mut runner = TakePictureRunner::new(
            TakePictureConfig::default(),
            CameraDriver::new_hd_default(),
        );
        let len = runner.capture_once().expect("capture picture");
        assert_eq!(len, SYNTHETIC_HD_JFIF_FRAME.len());
        assert_eq!(runner.pictures_taken, 1);
        assert_eq!(runner.last_picture_size, SYNTHETIC_HD_JFIF_FRAME.len());
    }

    #[test]
    fn test_ota_minimal_root_page_formatting() {
        let mgr = OtaDataManager::new_booted_ota0();
        let mut buf = [0u8; 1024];
        let resp = format_ota_minimal_root(&mgr, 210_000, &mut buf).expect("ota minimal root");
        let view = HttpResponseView::parse(resp).expect("parse response");
        assert_eq!(view.status_code, 200);
        assert_eq!(view.content_type, Some("text/plain"));
        let text = core::str::from_utf8(view.body).expect("utf8 body");
        assert!(text.contains("Klipper Cam OTA Minimal Intermediary"));
        assert!(text.contains("Running Partition: ota_0 (0x00020000)"));
        assert!(text.contains("Next Target:      ota_1 (0x001d0000)"));
    }

    #[test]
    fn test_cross_crate_end_to_end_ov2640_hal_jpeg_http_ota_stream() {
        // 1. SCCB bus + Ov2640 sensor detection (`esp32-cam-sccb` + `esp32-cam-sensors`)
        let mut sccb_bus: MockSccbBus<512> = MockSccbBus::new();
        sccb_bus.preset_reg8(Ov2640::SLV_ADDR, esp32_cam_sensors::ov2640::REG_PID, 0x26);
        sccb_bus.preset_reg8(Ov2640::SLV_ADDR, esp32_cam_sensors::ov2640::REG_VER, 0x42);
        sccb_bus.preset_reg8(Ov2640::SLV_ADDR, esp32_cam_sensors::ov2640::REG_MIDH, 0x7F);
        sccb_bus.preset_reg8(Ov2640::SLV_ADDR, esp32_cam_sensors::ov2640::REG_MIDL, 0xA2);

        let sensor_id =
            Ov2640::detect(&mut sccb_bus, Ov2640::SLV_ADDR).expect("detect OV2640 over SCCB");
        let ov2640 = Ov2640::from_id(Ov2640::SLV_ADDR, sensor_id);

        // 2. Type-state HAL CameraDriver<WroverKitPins, ...> (`esp32-cam-hal` + `esp32-cam-ll`)
        let hal_cfg = HalCameraConfig {
            xclk_freq_hz: DEFAULT_XCLK_FREQ_HZ,
            pixel_format: PixelFormat::Jpeg,
            frame_size: FrameSize::Hd,
            jpeg_quality: DEFAULT_JPEG_QUALITY,
            fb_count: DEFAULT_FB_COUNT,
            grab_mode: GrabMode::Latest,
            ..HalCameraConfig::default()
        };
        let uninit_hal: HalCameraDriver<
            WroverKitPins,
            _,
            Uninitialized,
            SensorHalAdapter<Ov2640>,
            2,
            1024,
        > = HalCameraDriver::new(WroverKitPins, sccb_bus, SensorHalAdapter::new(ov2640), hal_cfg);
        let init_hal = uninit_hal.init().expect("init WroverKit HD camera driver");

        // Verify OV-105 invariant: UXGA input geometry (1600x1200 -> HSIZE8=0xC8, VSIZE8=0x96)
        // is preserved while vertical 16:9 crop offset (YOFFL=150=0x96) is applied.
        assert_eq!(
            init_hal
                .bus()
                .get_reg8(Ov2640::SLV_ADDR, esp32_cam_sensors::ov2640::HSIZE8),
            0xC8
        );
        assert_eq!(
            init_hal
                .bus()
                .get_reg8(Ov2640::SLV_ADDR, esp32_cam_sensors::ov2640::VSIZE8),
            0x96
        );
        assert_eq!(
            init_hal
                .bus()
                .get_reg8(Ov2640::SLV_ADDR, esp32_cam_sensors::ov2640::YOFFL),
            150
        );

        let mut streaming_hal = init_hal.start_streaming().expect("start DMA streaming");

        // 3. Encode an RGB888 frame to baseline JFIF JPEG (`esp32-cam-conversions`)
        let width = 16u16;
        let height = 8u16;
        let mut rgb888 = [0u8; 16 * 8 * 3];
        for i in 0..(16 * 8) {
            rgb888[i * 3] = 40;
            rgb888[i * 3 + 1] = 120;
            rgb888[i * 3 + 2] = 200;
        }
        let mut jpeg_encoded = [0u8; 1024];
        let jpeg_len = encode_rgb888_to_jpeg(&rgb888, &mut jpeg_encoded, width, height, 85)
            .expect("encode RGB888 to JPEG");
        assert_eq!(verify_jpeg_soi(&jpeg_encoded[..jpeg_len]), Some(0));
        assert_eq!(
            verify_jpeg_eoi(&jpeg_encoded[..jpeg_len], false),
            Some(jpeg_len - 2)
        );

        // 4. Pack encoded JPEG + trailing DMA padding bytes into I2S0 32-bit DmaElements
        jpeg_encoded[jpeg_len] = 0xAA;
        jpeg_encoded[jpeg_len + 1] = 0xBB;
        let padded_len = jpeg_len + 2;
        let split = padded_len / 2;
        let mut dma_half0 = [0u8; 2048];
        let mut dma_half1 = [0u8; 2048];
        for (i, b) in jpeg_encoded[..split].iter().copied().enumerate() {
            let el = DmaElement::new(b, 0);
            dma_half0[i * 4..i * 4 + 4].copy_from_slice(&el.0);
        }
        for (idx, b) in jpeg_encoded[split..padded_len].iter().copied().enumerate() {
            let el = DmaElement::new(b, 0);
            dma_half1[idx * 4..idx * 4 + 4].copy_from_slice(&el.0);
        }

        let mut captured_jpeg = [0u8; 1024];
        let captured_len = {
            let hal_frame = streaming_hal
                .capture_frame(&[
                    &dma_half0[..split * 4],
                    &dma_half1[..(padded_len - split) * 4],
                ])
                .expect("capture and EOI-trim JPEG frame via HAL");
            assert_eq!(hal_frame.len(), jpeg_len);
            assert_eq!(hal_frame.as_slice(), &jpeg_encoded[..jpeg_len]);
            assert_eq!(
                parse_jpeg_dimensions(hal_frame.as_slice()),
                Some((width, height))
            );
            captured_jpeg[..jpeg_len].copy_from_slice(hal_frame.as_slice());
            hal_frame.len()
        };

        // 5. Wire captured JPEG into KlipperCamServer & FirmwareRuntime (Port 80 + Port 81)
        let fw_cam = CameraDriver::new(&captured_jpeg[..captured_len], FrameSize::Hd);
        let mut runtime = FirmwareRuntime::new(KlipperCamConfig::WROVER_KIT_HD, fw_cam);
        runtime.on_wifi_sta_start();
        runtime
            .on_wifi_got_ip(Ipv4Addr::new(172, 22, 1, 52))
            .expect("Wi-Fi IP acquisition cancels rollback and starts webservers");
        assert!(runtime.webservers_started);
        assert!(runtime.rollback_cancelled);

        // 6. Port 80 GET /snapshot serves the exact HAL-captured JPEG frame
        let mut resp_buf = [0u8; 4096];
        let snap_resp = runtime
            .server
            .handle_raw_request(
                ServerPort::Control,
                b"GET /snapshot HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n",
                &mut resp_buf,
            )
            .expect("Port 80 GET /snapshot");
        let snap_view = HttpResponseView::parse(snap_resp).expect("parse /snapshot response");
        assert_eq!(snap_view.status_code, 200);
        assert_eq!(snap_view.content_type, Some("image/jpeg"));
        assert_eq!(snap_view.body, &captured_jpeg[..captured_len]);

        // 7. Port 81 /stream session + Port 80 GET /status
        {
            let mut stream_guard = runtime
                .server
                .try_start_stream()
                .expect("acquire exclusive Port 81 stream guard");
            assert!(runtime.server.is_stream_active());
            let hdrs = stream_guard
                .write_stream_headers(&mut resp_buf)
                .expect("write Port 81 stream headers");
            let hdr_view = HttpResponseView::parse(hdrs).expect("parse stream headers");
            assert_eq!(hdr_view.status_code, 200);
            assert_eq!(hdr_view.content_type, Some(MJPEG_CONTENT_TYPE));
            assert_eq!(hdr_view.x_framerate, Some("5"));

            let chunk = stream_guard
                .write_next_frame_chunk(&runtime.server.camera, &mut resp_buf)
                .expect("write MJPEG multipart chunk");
            assert!(chunk.ends_with(&captured_jpeg[..captured_len]));
        }
        assert!(!runtime.server.is_stream_active());

        let stream_resp = runtime
            .server
            .handle_raw_request(
                ServerPort::Stream,
                b"GET /stream HTTP/1.1\r\nHost: 172.22.1.52:81\r\n\r\n",
                &mut resp_buf,
            )
            .expect("Port 81 GET /stream");
        let stream_view = HttpResponseView::parse(stream_resp).expect("parse Port 81 /stream");
        assert_eq!(stream_view.status_code, 200);
        assert_eq!(stream_view.content_type, Some(MJPEG_CONTENT_TYPE));
        assert!(stream_view.body.ends_with(&captured_jpeg[..captured_len]));

        let mut status_buf = [0u8; 1024];
        let status_resp = runtime
            .server
            .handle_raw_request(
                ServerPort::Control,
                b"GET /status HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n",
                &mut status_buf,
            )
            .expect("Port 80 GET /status");
        let status_view = HttpResponseView::parse(status_resp).expect("parse /status");
        let status_json = core::str::from_utf8(status_view.body).expect("utf8 json");
        assert!(status_json.contains("\"stream_active\":false"));
        assert!(status_json.contains("\"active_partition\":\"ota_0\""));

        // 8. Port 80 POST /update authenticated OTA upload
        let ota_payload = EspImageHeader::minimal_valid_header_bytes(4, 0x4008_0400);
        let mut ota_req = [0u8; 512];
        let req_len = {
            let mut w = SliceWriter::new(&mut ota_req);
            let _ = w.write_str_checked("POST /update HTTP/1.1\r\nHost: 172.22.1.52\r\nX-OTA-Token: ");
            let _ = w.write_str_checked(DEFAULT_OTA_TOKEN);
            let _ = w.write_str_checked("\r\nContent-Length: 24\r\n\r\n");
            let _ = w.write_bytes(&ota_payload);
            w.written_len()
        };
        let ota_resp = runtime
            .server
            .handle_raw_request(ServerPort::Control, &ota_req[..req_len], &mut resp_buf)
            .expect("Port 80 POST /update");
        let ota_view = HttpResponseView::parse(ota_resp).expect("parse /update response");
        assert_eq!(ota_view.status_code, 200);
        assert!(runtime.server.is_reboot_scheduled());
        assert_eq!(runtime.server.last_ota_target_slot(), Some(OtaSlot::Ota1));
        assert_eq!(runtime.server.last_ota_bytes_written(), 24);

        let _ = streaming_hal.deinit().expect("deinit HAL camera driver");
    }
}
