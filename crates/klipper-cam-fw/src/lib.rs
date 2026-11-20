//! Bare-metal `#![no_std]` Klipper 3D Printer Camera & OTA firmware library.
//!
//! # Modules
//!
//! - [`captive_net`]: Pure-Rust SoftAP DHCPv4 server and captive-portal DNS.
//! - [`config`]: Hardware pinout, camera sensor, network, and security config.
//! - [`http`]: Non-blocking HTTP control and multipart MJPEG stream servers.
//! - [`ota`]: Flash partition layout, constant-time token check, and rollback.
//! - [`prov_store`]: Brownout-safe dual-sector flash provisioning store.
//! - [`prov_uart`]: Setup-Mode-only UART0 provisioning CLI fallback.
//! - [`runtime`]: Bare-metal multi-core execution and cooperative scheduling.
//! - [`spsc_ring`]: Lock-free SPSC ring buffer for inter-core JPEG transfer.
//! - [`telemetry`]: Performance counters, stream diagnostics, and telemetry.

#![no_std]
#![deny(unsafe_code)]
#![cfg_attr(target_arch = "xtensa", feature(asm_experimental_arch))]

#[cfg(test)]
extern crate alloc;

pub mod ap_select;
pub mod captive_net;
pub mod config;
pub mod frame_buffer;
pub mod http;
pub mod ota;
#[allow(unsafe_code)]
pub mod prov_store;
#[allow(unsafe_code)]
pub mod prov_uart;
pub mod runtime;
pub mod spsc_ring;
#[allow(unsafe_code)]
pub mod telemetry;

pub use ap_select::{
    roam_decision, ApCandidate, ApTracker, RoamBackoff, RoamDecision, WeakLinkTimer,
    MAX_AP_CANDIDATES, MISSED_PASS_PENALTY_DB, ROAM_CHECK_INTERVAL_MS, ROAM_HYSTERESIS_DB,
    ROAM_MAX_SCAN_INTERVAL_MS, ROAM_MIN_SCAN_INTERVAL_MS, ROAM_TRIGGER_RSSI_DBM,
    ROAM_WEAK_DURATION_MS, SCAN_ACTIVE_MAX_MS, SCAN_ACTIVE_MIN_MS, SCAN_PASSES,
};

pub use captive_net::{
    handle_captive_dns_query, DhcpServer, AP_IP, AP_SUBNET_MASK, CAPTIVE_PORTAL_URI,
    DHCP_CLIENT_PORT, DHCP_LEASE_SECS, DHCP_POOL_SIZE, DHCP_POOL_START_HOST, DHCP_SERVER_PORT,
    DNS_PORT, DNS_TTL_SECS,
};

pub use config::{
    active_ota_token, active_wifi_pass, active_wifi_ssid, BoardVariant, CameraPinout, ConfigError,
    IpMode, Ipv4Addr, KlipperCamConfig, NetworkConfig, NvsConfigOverrides, StaticIpConfig,
    TakePictureConfig, WifiAuthMode, AITHINKER_FLASH_LED_GPIO, AITHINKER_PINOUT,
    DEFAULT_CONTROL_PORT, DEFAULT_FB_COUNT, DEFAULT_JPEG_QUALITY, DEFAULT_OTA_TOKEN,
    DEFAULT_STREAM_PACING_MS, DEFAULT_STREAM_PORT, DEFAULT_WIFI_RECONNECT_MS, DEFAULT_XCLK_FREQ_HZ,
    ESP32S3_GOOUUU_PINOUT, ESP32S3_WROOM_PINOUT, ESP32S3_XIAO_PINOUT, HTTP_PORT, JPEG_QUALITY,
    MIN_OTA_TOKEN_LEN, OTA_TOKEN, STREAM_PORT, WROVER_KIT_PINOUT,
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
    write_http_error, write_mjpeg_part_header, CameraControlState, CameraDriver, FrameGuard,
    HttpError, HttpMethod, HttpRequest, HttpResponseView, KlipperCamServer, ServerPort,
    SliceWriter, StreamSessionGuard, TelemetrySnapshot, CONTROL_JSON_BUF_LEN,
    KLIPPER_CONTROL_PANEL_HTML, KLIPPER_ROOT_STATUS_PAGE, MJPEG_BOUNDARY, MJPEG_CONTENT_TYPE,
    STATUS_JSON_BUF_LEN, SYNTHETIC_HD_JFIF_FRAME,
};
pub use ota::{
    crc32_ieee, find_partition_by_label, validate_ota_image_header, verify_ota_token,
    verify_partition_table_integrity, AlignedFlashChunk, AlignedOtaBuf, Core1ParkControl,
    Core1PauseGuard, EspImageHeader, GuardedFlashStorage, MockNorFlash, NoOpFlashStorage,
    OtaDataManager, OtaDataSector, OtaDataSelect, OtaError, OtaFlashStorage, OtaImgState, OtaSlot,
    OtaWriteSession, PartitionEntry, PartitionSubType, PartitionType, ESP_IMAGE_HEADER_LEN,
    ESP_IMAGE_HEADER_MAGIC, ESP_IMAGE_MAX_SEGMENTS, ESP_IMAGE_MIN_SEGMENTS, FLASH_SECTOR_SIZE,
    NVS_OFFSET, NVS_SIZE, OTADATA_OFFSET, OTADATA_SECTOR_0_OFFSET, OTADATA_SECTOR_1_OFFSET,
    OTADATA_SIZE, OTA_0_OFFSET, OTA_0_SIZE, OTA_1_OFFSET, OTA_1_SIZE, OTA_CHUNK_SIZE,
    OTA_PARTITION_MAX_SIZE, PARTITION_TABLE, PHY_INIT_OFFSET, PHY_INIT_SIZE,
};
pub use prov_store::{
    ProvStore, ProvStoreError, ProvisionedConfig, FLAG_PROVISIONED, FLAG_STATIC_IP,
    MAX_OTA_TOKEN_LEN, MAX_PASS_LEN, MAX_SSID_LEN, MAX_STATIC_PREFIX_LEN, MIN_STATIC_PREFIX_LEN,
    PROV_MAGIC, PROV_RECORD_LEN, PROV_SECTOR_A_OFFSET, PROV_SECTOR_B_OFFSET, PROV_VERSION,
};
pub use prov_uart::{UartProvAction, UartProvSession, UartReply};
pub use runtime::{
    FirmwareRuntime, SensorHalAdapter, TakePictureRunner, WifiLifecycleState, WifiStaState,
};
pub use spsc_ring::{
    Core1JpegSpsc, JpegSpscSlot, JPEG_SPSC_CHUNK_BYTES, SPSC_FLAG_ABORT, SPSC_FLAG_EOF,
    SPSC_FLAG_SOF,
};
pub use telemetry::{
    ccount, format_sha256_hex, measure_stack_hwm, measure_stack_hwm_volatile, paint_stack_sentinel,
    reset_reason_label, FwStats, FwStatsSnapshot,
};
