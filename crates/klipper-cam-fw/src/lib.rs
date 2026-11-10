//! Bare-metal `#![no_std]` Klipper 3D Printer Camera & OTA firmware library
//! (`HTTP`, `MJPEG`, `OTA`).
//!
//! Replaces:
//! - `examples/camera_example/main/klipper_cam.c`
//! - `examples/camera_example/main/ota_minimal.c`
//! - `examples/camera_example/main/take_picture.c`
//! - `examples/camera_example/main/klipper_cam_config.h`
//! - `examples/camera_example/main/camera_pinout.h`

#![no_std]
#![deny(unsafe_code)]

#[cfg(test)]
extern crate alloc;

pub mod ap_select;
pub mod config;
pub mod frame_buffer;
pub mod http;
pub mod ota;
pub mod runtime;
pub mod spsc_ring;

pub use ap_select::{
    roam_decision, ApCandidate, ApTracker, RoamBackoff, RoamDecision, WeakLinkTimer,
    MAX_AP_CANDIDATES, MISSED_PASS_PENALTY_DB, ROAM_CHECK_INTERVAL_MS, ROAM_HYSTERESIS_DB,
    ROAM_MAX_SCAN_INTERVAL_MS, ROAM_MIN_SCAN_INTERVAL_MS, ROAM_TRIGGER_RSSI_DBM,
    ROAM_WEAK_DURATION_MS, SCAN_ACTIVE_MAX_MS, SCAN_ACTIVE_MIN_MS, SCAN_PASSES,
};

pub use config::{
    active_ota_token, active_wifi_pass, active_wifi_ssid, BoardVariant, CameraPinout, Ipv4Addr,
    KlipperCamConfig, NetworkConfig, StaticIpConfig, TakePictureConfig, AITHINKER_FLASH_LED_GPIO,
    AITHINKER_PINOUT, DEFAULT_CONTROL_PORT, DEFAULT_FB_COUNT, DEFAULT_JPEG_QUALITY,
    DEFAULT_OTA_TOKEN, DEFAULT_STREAM_PACING_MS, DEFAULT_STREAM_PORT, DEFAULT_WIFI_RECONNECT_MS,
    DEFAULT_XCLK_FREQ_HZ, ESP32S3_GOOUUU_PINOUT, ESP32S3_WROOM_PINOUT, ESP32S3_XIAO_PINOUT,
    HTTP_PORT, JPEG_QUALITY, OTA_TOKEN, STREAM_PORT, WROVER_KIT_PINOUT,
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
pub use esp32_cam_sensors::ov2640::Ov2640;
pub use esp32_cam_sensors::{self, CameraSensor};
pub use esp32_cam_types::{
    CameraError, CameraFbLocation, CameraGrabMode, CameraModel, CameraStatus, FrameSize,
    GainCeiling, GrabMode, PixelFormat, SensorId,
};
pub use frame_buffer::{PingPongFrameBuffer, TripleFrameBuffer};
pub use http::{
    format_control_json, format_mjpeg_part_header, format_ota_minimal_root, format_status_json,
    framesize_control_code, framesize_label, parse_framesize_val, parse_http_request,
    write_http_error, CameraControlState, CameraDriver, FrameGuard, HttpError, HttpMethod,
    HttpRequest, HttpResponseView, KlipperCamServer, ServerPort, SliceWriter, StreamSessionGuard,
    TelemetrySnapshot, CONTROL_JSON_BUF_LEN, KLIPPER_CONTROL_PANEL_HTML, KLIPPER_ROOT_STATUS_PAGE,
    MJPEG_BOUNDARY, MJPEG_CONTENT_TYPE, SYNTHETIC_HD_JFIF_FRAME,
};
pub use ota::{
    crc32_ieee, find_partition_by_label, validate_ota_image_header, verify_ota_token,
    verify_partition_table_integrity, AlignedFlashChunk, AlignedOtaBuf, EspImageHeader,
    MockNorFlash, NoOpFlashStorage, OtaDataManager, OtaDataSelect, OtaError, OtaFlashStorage,
    OtaImgState, OtaSlot, PartitionEntry, PartitionSubType, PartitionType, ESP_IMAGE_HEADER_LEN,
    ESP_IMAGE_HEADER_MAGIC, ESP_IMAGE_MAX_SEGMENTS, ESP_IMAGE_MIN_SEGMENTS, FLASH_SECTOR_SIZE,
    NVS_OFFSET, NVS_SIZE, OTADATA_OFFSET, OTADATA_SECTOR_0_OFFSET, OTADATA_SECTOR_1_OFFSET,
    OTADATA_SIZE, OTA_0_OFFSET, OTA_0_SIZE, OTA_1_OFFSET, OTA_1_SIZE, OTA_CHUNK_SIZE,
    OTA_PARTITION_MAX_SIZE, PARTITION_TABLE, PHY_INIT_OFFSET, PHY_INIT_SIZE,
};
pub use runtime::{
    FirmwareRuntime, SensorHalAdapter, TakePictureRunner, WifiLifecycleState, WifiStaState,
};
pub use spsc_ring::{
    Core1JpegSpsc, JpegSpscSlot, JPEG_SPSC_CHUNK_BYTES, SPSC_FLAG_ABORT, SPSC_FLAG_EOF,
    SPSC_FLAG_SOF,
};
