//! Type-state [`CameraDriver`] (`Uninitialized` -> `Initialized` ->
//! `Streaming`), JPEG SOI/EOI frame boundary verification, DMA half-buffer
//! assembly, autofocus (`esp_camera_af.c`), and NVS configuration persistence
//! (`driver/cam_hal.c`, `driver/esp_camera.c`).

#![forbid(unsafe_code)]

use core::marker::PhantomData;
use esp32_cam_ll::{
    calc_dma_sizes, DmaSizeConfig, Esp32I2sCamera, XclkConfig, XclkController,
    DEFAULT_CAMERA_DMA_BUFFER_SIZE_MAX,
};
use esp32_cam_sccb::SccbBus;
use esp32_cam_types::{
    CameraConvMode, CameraError, CameraFbLocation, CameraModel, CameraPid, CameraSccbAddr,
    CameraStatus, FrameSize, GainCeiling, GrabMode, PixelFormat, SensorId,
};

use crate::board_pins::{BoardPins, TargetChip};
use crate::frame_pool::{FrameGuard, FramePool};

/// JPEG Start-Of-Image (`SOI`) 3-byte marker (`0xFF, 0xD8, 0xFF` in
/// `driver/cam_hal.c:161`).
pub const JPEG_SOI_MARKER: [u8; 3] = ::esp32_cam_conversions::JPEG_SOI_MARKER;

/// JPEG End-Of-Image (`EOI`) 2-byte marker (`0xFF, 0xD9` in
/// `driver/cam_hal.c:163`).
pub const JPEG_EOI_MARKER: [u8; 2] = ::esp32_cam_conversions::JPEG_EOI_MARKER;

/// Default byte count probed at the start of a PSRAM JPEG frame for `SOI`
/// (`CAM_SOI_PROBE_BYTES`).
pub const CAM_SOI_PROBE_BYTES: usize = 32;

/// Computes the scan window for JPEG `EOI` detection in PSRAM
/// (`eoi_probe_window` in `driver/cam_hal.c:167-171`).
#[inline]
#[must_use]
pub const fn eoi_probe_window(half_buf_size: usize, frame_len: usize) -> usize {
    ::esp32_cam_conversions::eoi_probe_window(half_buf_size, frame_len)
}

/// Searches `inbuf` for the 3-byte JPEG Start-Of-Image marker
/// (`0xFF, 0xD8, 0xFF`), returning its byte offset (`cam_verify_jpeg_soi` in
/// `driver/cam_hal.c:173-192`).
#[inline]
#[must_use]
pub fn verify_jpeg_soi(inbuf: &[u8]) -> Option<usize> {
    ::esp32_cam_conversions::verify_jpeg_soi(inbuf)
}

/// Searches `inbuf` for the 2-byte JPEG End-Of-Image marker (`0xFF, 0xD9`),
/// returning its byte offset (`cam_verify_jpeg_eoi` in
/// `driver/cam_hal.c:194-246`).
///
/// - When `search_forward == true` (PSRAM direct-DMA tail probe), returns the
///   earliest `EOI` offset.
/// - When `search_forward == false` (SRAM filtered buffer), scans backward from
///   the end of `inbuf`.
#[inline]
#[must_use]
pub fn verify_jpeg_eoi(inbuf: &[u8], search_forward: bool) -> Option<usize> {
    ::esp32_cam_conversions::verify_jpeg_eoi(inbuf, search_forward)
}

/// Hardware-agnostic sensor interface consumed by [`CameraDriver`].
///
/// Automatically implemented for all `esp32_cam_sensors::CameraSensor<B>`
/// drivers when the `sensors` feature is enabled, and implemented directly by
/// [`HalOv2640Sensor`].
pub trait HalCameraSensor<B: SccbBus> {
    /// Returns the [`CameraModel`] discriminant for this sensor.
    fn model(&self) -> CameraModel;

    /// Returns a shared reference to the sensor's [`SensorId`].
    fn id(&self) -> &SensorId;

    /// Returns a shared reference to the sensor's [`CameraStatus`].
    fn status(&self) -> &CameraStatus;

    /// Returns a mutable reference to the sensor's [`CameraStatus`].
    fn status_mut(&mut self) -> &mut CameraStatus;

    /// Resets the sensor hardware (`sensor_t::reset`).
    fn reset(&mut self, bus: &mut B) -> Result<(), CameraError>;

    /// Reads current sensor registers into [`CameraStatus`]
    /// (`sensor_t::init_status`).
    fn init_status(&mut self, _bus: &mut B) -> Result<(), CameraError> {
        Ok(())
    }

    /// Configures output pixel format (`sensor_t::set_pixformat`).
    fn set_pixformat(&mut self, bus: &mut B, pixformat: PixelFormat) -> Result<(), CameraError>;

    /// Configures output frame size (`sensor_t::set_framesize`).
    fn set_framesize(&mut self, bus: &mut B, framesize: FrameSize) -> Result<(), CameraError>;

    /// Sets AGC gain ceiling (`sensor_t::set_gainceiling`).
    fn set_gainceiling(
        &mut self,
        _bus: &mut B,
        _gainceiling: GainCeiling,
    ) -> Result<(), CameraError> {
        Err(CameraError::NotSupported)
    }

    /// Sets JPEG quality (`0..=63`, `sensor_t::set_quality`).
    fn set_quality(&mut self, _bus: &mut B, _quality: i32) -> Result<(), CameraError> {
        Err(CameraError::NotSupported)
    }

    /// Enables or disables black pixel cancellation (`sensor_t::set_bpc`).
    fn set_bpc(&mut self, _bus: &mut B, _enable: bool) -> Result<(), CameraError> {
        Err(CameraError::NotSupported)
    }

    /// Enables or disables white pixel cancellation (`sensor_t::set_wpc`).
    fn set_wpc(&mut self, _bus: &mut B, _enable: bool) -> Result<(), CameraError> {
        Err(CameraError::NotSupported)
    }

    /// Enables or disables lens shading correction (`sensor_t::set_lenc`).
    fn set_lenc(&mut self, _bus: &mut B, _enable: bool) -> Result<(), CameraError> {
        Err(CameraError::NotSupported)
    }

    /// Enables or disables horizontal mirror (`sensor_t::set_hmirror`).
    fn set_hmirror(&mut self, _bus: &mut B, _enable: bool) -> Result<(), CameraError> {
        Err(CameraError::NotSupported)
    }

    /// Enables or disables vertical flip (`sensor_t::set_vflip`).
    fn set_vflip(&mut self, _bus: &mut B, _enable: bool) -> Result<(), CameraError> {
        Err(CameraError::NotSupported)
    }
}

#[cfg(feature = "sensors")]
impl<B: SccbBus, S: esp32_cam_sensors::CameraSensor<B>> HalCameraSensor<B> for S {
    #[inline]
    fn model(&self) -> CameraModel {
        esp32_cam_sensors::CameraSensor::model(self)
    }

    #[inline]
    fn id(&self) -> &SensorId {
        esp32_cam_sensors::CameraSensor::id(self)
    }

    #[inline]
    fn status(&self) -> &CameraStatus {
        esp32_cam_sensors::CameraSensor::status(self)
    }

    #[inline]
    fn status_mut(&mut self) -> &mut CameraStatus {
        esp32_cam_sensors::CameraSensor::status_mut(self)
    }

    #[inline]
    fn reset(&mut self, bus: &mut B) -> Result<(), CameraError> {
        esp32_cam_sensors::CameraSensor::reset(self, bus)
    }

    #[inline]
    fn init_status(&mut self, bus: &mut B) -> Result<(), CameraError> {
        esp32_cam_sensors::CameraSensor::init_status(self, bus)
    }

    #[inline]
    fn set_pixformat(&mut self, bus: &mut B, pixformat: PixelFormat) -> Result<(), CameraError> {
        esp32_cam_sensors::CameraSensor::set_pixformat(self, bus, pixformat)
    }

    #[inline]
    fn set_framesize(&mut self, bus: &mut B, framesize: FrameSize) -> Result<(), CameraError> {
        esp32_cam_sensors::CameraSensor::set_framesize(self, bus, framesize)
    }

    #[inline]
    fn set_gainceiling(
        &mut self,
        bus: &mut B,
        gainceiling: GainCeiling,
    ) -> Result<(), CameraError> {
        esp32_cam_sensors::CameraSensor::set_gainceiling(self, bus, gainceiling)
    }

    #[inline]
    fn set_quality(&mut self, bus: &mut B, quality: i32) -> Result<(), CameraError> {
        esp32_cam_sensors::CameraSensor::set_quality(self, bus, quality)
    }

    #[inline]
    fn set_bpc(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        esp32_cam_sensors::CameraSensor::set_bpc(self, bus, enable)
    }

    #[inline]
    fn set_wpc(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        esp32_cam_sensors::CameraSensor::set_wpc(self, bus, enable)
    }

    #[inline]
    fn set_lenc(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        esp32_cam_sensors::CameraSensor::set_lenc(self, bus, enable)
    }

    #[inline]
    fn set_hmirror(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        esp32_cam_sensors::CameraSensor::set_hmirror(self, bus, enable)
    }

    #[inline]
    fn set_vflip(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        esp32_cam_sensors::CameraSensor::set_vflip(self, bus, enable)
    }
}

/// Built-in minimal OV2640 handle for default [`CameraDriver`] builds.
///
/// # Scope (HAL-09)
///
/// This is a stub, not a complete driver. `reset`, `detect` and `set_quality`
/// are implemented against real registers; `set_framesize`, `set_pixformat`
/// and `set_gainceiling` are not, and return [`CameraError::NotSupported`].
///
/// They used to write the OV2640 bank-select prelude, record the requested
/// value and return `Ok` — so a caller asking for HD got the bank selected,
/// nothing else written, success reported, and HD recorded in `status` while
/// the sensor stayed wherever reset had left it. The single bank-select write
/// is what made it look functional.
///
/// For a complete OV2640 driver use [`esp32_cam_sensors::ov2640::Ov2640`],
/// which implements the full register tables and is covered by the golden
/// transaction tests in that crate. [`CameraDriver`] accepts any
/// [`HalCameraSensor`], so it can be built over that type instead.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct HalOv2640Sensor {
    slv_addr: u8,
    id: SensorId,
    status: CameraStatus,
    pixformat: PixelFormat,
}

impl Default for HalOv2640Sensor {
    fn default() -> Self {
        Self::new()
    }
}

impl HalOv2640Sensor {
    /// Creates a default [`HalOv2640Sensor`] bound to `CameraSccbAddr::OV2640`
    /// (`0x30`).
    pub fn new() -> Self {
        Self {
            slv_addr: CameraSccbAddr::OV2640,
            id: SensorId {
                midh: 0x7F,
                midl: 0xA2,
                pid: CameraPid::OV2640.as_u16(),
                ver: 0x42,
            },
            status: CameraStatus::default(),
            pixformat: PixelFormat::Jpeg,
        }
    }

    /// Probes and verifies an `OV2640` sensor on `bus` at `slv_addr` (`0x30`).
    pub fn detect<B: SccbBus>(bus: &mut B, slv_addr: u8) -> Result<Self, CameraError> {
        bus.write_reg8(slv_addr, 0xFF, 0x01)?;
        let midh = bus.read_reg8(slv_addr, 0x1C)?;
        let midl = bus.read_reg8(slv_addr, 0x1D)?;
        let pid = bus.read_reg8(slv_addr, 0x0A)? as u16;
        let ver = bus.read_reg8(slv_addr, 0x0B)?;
        if pid != CameraPid::OV2640.as_u16() {
            return Err(CameraError::NotDetected);
        }
        Ok(Self {
            slv_addr,
            id: SensorId {
                midh,
                midl,
                pid,
                ver,
            },
            status: CameraStatus::default(),
            pixformat: PixelFormat::Jpeg,
        })
    }

    /// Returns a shared reference to the sensor's [`CameraStatus`].
    #[inline]
    #[must_use]
    pub const fn status(&self) -> &CameraStatus {
        &self.status
    }
}

#[cfg(not(feature = "sensors"))]
impl<B: SccbBus> HalCameraSensor<B> for HalOv2640Sensor {
    fn model(&self) -> CameraModel {
        CameraModel::Ov2640
    }

    fn id(&self) -> &SensorId {
        &self.id
    }

    fn status(&self) -> &CameraStatus {
        &self.status
    }

    fn status_mut(&mut self) -> &mut CameraStatus {
        &mut self.status
    }

    fn reset(&mut self, bus: &mut B) -> Result<(), CameraError> {
        bus.write_reg8(self.slv_addr, 0xFF, 0x01)?;
        bus.write_reg8(self.slv_addr, 0x12, 0x80)?;
        bus.delay_ms(10);
        Ok(())
    }

    /// Always returns [`CameraError::NotSupported`] (HAL-09).
    ///
    /// This stub never implemented the registers this needs; it wrote only
    /// the bank-select prelude, recorded the requested value and returned
    /// `Ok`. Use [`esp32_cam_sensors::ov2640::Ov2640`] for a complete driver.
    fn set_pixformat(&mut self, _bus: &mut B, _pixformat: PixelFormat) -> Result<(), CameraError> {
        Err(CameraError::NotSupported)
    }

    /// Always returns [`CameraError::NotSupported`] (HAL-09).
    ///
    /// This stub never implemented the registers this needs; it wrote only
    /// the bank-select prelude, recorded the requested value and returned
    /// `Ok`. Use [`esp32_cam_sensors::ov2640::Ov2640`] for a complete driver.
    fn set_framesize(&mut self, _bus: &mut B, _framesize: FrameSize) -> Result<(), CameraError> {
        Err(CameraError::NotSupported)
    }

    /// Always returns [`CameraError::NotSupported`] (HAL-09).
    ///
    /// This stub never implemented the registers this needs; it wrote only
    /// the bank-select prelude, recorded the requested value and returned
    /// `Ok`. Use [`esp32_cam_sensors::ov2640::Ov2640`] for a complete driver.
    fn set_gainceiling(
        &mut self,
        _bus: &mut B,
        _gainceiling: GainCeiling,
    ) -> Result<(), CameraError> {
        Err(CameraError::NotSupported)
    }

    fn set_quality(&mut self, bus: &mut B, quality: i32) -> Result<(), CameraError> {
        let q = quality.clamp(0, 63) as u8;
        bus.write_reg8(self.slv_addr, 0xFF, 0x00)?;
        bus.write_reg8(self.slv_addr, 0x44, q)?;
        self.status.quality = q;
        Ok(())
    }

    fn set_bpc(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        bus.write_reg8(self.slv_addr, 0xFF, 0x00)?;
        let val = if enable { 0x04 } else { 0x00 };
        bus.write_reg8_mask(self.slv_addr, 0xC3, 0x04, val)?;
        self.status.bpc = enable;
        Ok(())
    }

    fn set_wpc(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        bus.write_reg8(self.slv_addr, 0xFF, 0x00)?;
        let val = if enable { 0x02 } else { 0x00 };
        bus.write_reg8_mask(self.slv_addr, 0xC3, 0x02, val)?;
        self.status.wpc = enable;
        Ok(())
    }

    fn set_lenc(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        bus.write_reg8(self.slv_addr, 0xFF, 0x00)?;
        let val = if enable { 0x01 } else { 0x00 };
        bus.write_reg8_mask(self.slv_addr, 0xC3, 0x01, val)?;
        self.status.lenc = enable;
        Ok(())
    }

    fn set_hmirror(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        bus.write_reg8(self.slv_addr, 0xFF, 0x01)?;
        let val = if enable { 0x80 } else { 0x00 };
        bus.write_reg8_mask(self.slv_addr, 0x04, 0x80, val)?;
        self.status.hmirror = enable;
        Ok(())
    }

    fn set_vflip(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        bus.write_reg8(self.slv_addr, 0xFF, 0x01)?;
        let val = if enable { 0x50 } else { 0x00 };
        bus.write_reg8_mask(self.slv_addr, 0x04, 0x50, val)?;
        self.status.vflip = enable;
        Ok(())
    }
}

#[cfg(feature = "sensors")]
impl<B: SccbBus> esp32_cam_sensors::CameraSensor<B> for HalOv2640Sensor {
    fn model(&self) -> CameraModel {
        CameraModel::Ov2640
    }

    fn id(&self) -> &SensorId {
        &self.id
    }

    fn status(&self) -> &CameraStatus {
        &self.status
    }

    fn status_mut(&mut self) -> &mut CameraStatus {
        &mut self.status
    }

    fn reset(&mut self, bus: &mut B) -> Result<(), CameraError> {
        bus.write_reg8(self.slv_addr, 0xFF, 0x01)?;
        bus.write_reg8(self.slv_addr, 0x12, 0x80)?;
        bus.delay_ms(10);
        Ok(())
    }

    fn init_status(&mut self, _bus: &mut B) -> Result<(), CameraError> {
        Ok(())
    }

    /// Always returns [`CameraError::NotSupported`] (HAL-09).
    ///
    /// This stub never implemented the registers this needs; it wrote only
    /// the bank-select prelude, recorded the requested value and returned
    /// `Ok`. Use [`esp32_cam_sensors::ov2640::Ov2640`] for a complete driver.
    fn set_pixformat(&mut self, _bus: &mut B, _pixformat: PixelFormat) -> Result<(), CameraError> {
        Err(CameraError::NotSupported)
    }

    /// Always returns [`CameraError::NotSupported`] (HAL-09).
    ///
    /// This stub never implemented the registers this needs; it wrote only
    /// the bank-select prelude, recorded the requested value and returned
    /// `Ok`. Use [`esp32_cam_sensors::ov2640::Ov2640`] for a complete driver.
    fn set_framesize(&mut self, _bus: &mut B, _framesize: FrameSize) -> Result<(), CameraError> {
        Err(CameraError::NotSupported)
    }

    /// Always returns [`CameraError::NotSupported`] (HAL-09).
    ///
    /// This stub never implemented the registers this needs; it wrote only
    /// the bank-select prelude, recorded the requested value and returned
    /// `Ok`. Use [`esp32_cam_sensors::ov2640::Ov2640`] for a complete driver.
    fn set_gainceiling(
        &mut self,
        _bus: &mut B,
        _gainceiling: GainCeiling,
    ) -> Result<(), CameraError> {
        Err(CameraError::NotSupported)
    }

    fn set_quality(&mut self, bus: &mut B, quality: i32) -> Result<(), CameraError> {
        let q = quality.clamp(0, 63) as u8;
        bus.write_reg8(self.slv_addr, 0xFF, 0x00)?;
        bus.write_reg8(self.slv_addr, 0x44, q)?;
        self.status.quality = q;
        Ok(())
    }

    fn set_bpc(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        bus.write_reg8(self.slv_addr, 0xFF, 0x00)?;
        let val = if enable { 0x04 } else { 0x00 };
        bus.write_reg8_mask(self.slv_addr, 0xC3, 0x04, val)?;
        self.status.bpc = enable;
        Ok(())
    }

    fn set_wpc(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        bus.write_reg8(self.slv_addr, 0xFF, 0x00)?;
        let val = if enable { 0x02 } else { 0x00 };
        bus.write_reg8_mask(self.slv_addr, 0xC3, 0x02, val)?;
        self.status.wpc = enable;
        Ok(())
    }

    fn set_lenc(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        bus.write_reg8(self.slv_addr, 0xFF, 0x00)?;
        let val = if enable { 0x01 } else { 0x00 };
        bus.write_reg8_mask(self.slv_addr, 0xC3, 0x01, val)?;
        self.status.lenc = enable;
        Ok(())
    }

    fn set_hmirror(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        bus.write_reg8(self.slv_addr, 0xFF, 0x01)?;
        let val = if enable { 0x80 } else { 0x00 };
        bus.write_reg8_mask(self.slv_addr, 0x04, 0x80, val)?;
        self.status.hmirror = enable;
        Ok(())
    }

    fn set_vflip(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        bus.write_reg8(self.slv_addr, 0xFF, 0x01)?;
        let val = if enable { 0x50 } else { 0x00 };
        bus.write_reg8_mask(self.slv_addr, 0x04, 0x50, val)?;
        self.status.vflip = enable;
        Ok(())
    }
}

/// Type-state marker: camera hardware and sensor have not yet been initialized.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Uninitialized;

/// Type-state marker: camera pins, XCLK, I2S0/LCD_CAM, and sensor registers
/// are initialized.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Initialized;

/// Type-state marker: VSYNC interrupt and DMA frame reception are actively
/// streaming.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Streaming;

/// High-level camera configuration (`camera_config_t` from
/// `driver/include/esp_camera.h`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CameraConfig {
    pub xclk_freq_hz: u32,
    pub ledc_timer: u8,
    pub ledc_channel: u8,
    pub pixel_format: PixelFormat,
    pub frame_size: FrameSize,
    pub jpeg_quality: u8,
    pub fb_count: usize,
    pub fb_location: CameraFbLocation,
    pub grab_mode: GrabMode,
    pub conv_mode: CameraConvMode,
    pub psram_dma_mode: bool,
}

impl Default for CameraConfig {
    fn default() -> Self {
        Self {
            xclk_freq_hz: 20_000_000,
            ledc_timer: 0,
            ledc_channel: 0,
            pixel_format: PixelFormat::Jpeg,
            frame_size: FrameSize::Qvga,
            jpeg_quality: 12,
            fb_count: 2,
            fb_location: CameraFbLocation::Psram,
            grab_mode: GrabMode::WhenEmpty,
            conv_mode: CameraConvMode::Disable,
            psram_dma_mode: false,
        }
    }
}

/// Persisted camera state record (`esp_camera_save_to_nvs` /
/// `esp_camera_load_from_nvs` in `driver/esp_camera.c`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SavedCameraSettings {
    pub status: CameraStatus,
    pub pixformat: PixelFormat,
}

/// Autofocus operating mode (`esp_camera_af_mode_t` in
/// `driver/include/esp_camera_af.h`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Hash)]
pub enum AfMode {
    #[default]
    Auto = 0,
    Single = 1,
    Continuous = 2,
    Manual = 3,
}

/// Autofocus configuration (`esp_camera_af_config_t` in
/// `driver/esp_camera_af.c`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AfConfig {
    pub mode: AfMode,
    pub timeout_ms: u32,
}

impl Default for AfConfig {
    fn default() -> Self {
        Self {
            mode: AfMode::Single,
            timeout_ms: 2000,
        }
    }
}

/// Autofocus status (`esp_camera_af_status_t` in `driver/esp_camera_af.c`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct AfStatus {
    pub raw: u8,
    pub focused: bool,
    pub busy: bool,
}

/// Optional autofocus extension trait (`driver/esp_camera_af.c`).
pub trait CameraAutofocus<B: SccbBus>: HalCameraSensor<B> {
    fn af_is_supported(&self) -> bool {
        false
    }
    fn af_init(&mut self, _bus: &mut B, _config: &AfConfig) -> Result<(), CameraError> {
        Err(CameraError::NotSupported)
    }
    fn af_set_mode(&mut self, _bus: &mut B, _mode: AfMode) -> Result<(), CameraError> {
        Err(CameraError::NotSupported)
    }
    fn af_trigger(&mut self, _bus: &mut B) -> Result<(), CameraError> {
        Err(CameraError::NotSupported)
    }
    fn af_get_status(&mut self, _bus: &mut B) -> Result<AfStatus, CameraError> {
        Err(CameraError::NotSupported)
    }
    fn af_set_manual_position(&mut self, _bus: &mut B, _position: u16) -> Result<(), CameraError> {
        Err(CameraError::NotSupported)
    }
}

#[inline]
fn ignore_not_supported(res: Result<(), CameraError>) -> Result<(), CameraError> {
    match res {
        Ok(()) | Err(CameraError::NotSupported) => Ok(()),
        Err(e) => Err(e),
    }
}

mod private {
    pub trait Sealed {}
    impl Sealed for super::Uninitialized {}
    impl Sealed for super::Initialized {}
}

/// Marker trait for driver states where DMA is stopped and mutating the sensor
/// or SCCB bus directly is safe ([`Uninitialized`] and [`Initialized`], never
/// [`Streaming`], `M-117`).
pub trait NonStreamingState: private::Sealed {}
impl NonStreamingState for Uninitialized {}
impl NonStreamingState for Initialized {}

#[derive(Debug)]
struct CameraDriverInner<
    Board: BoardPins,
    Sccb: SccbBus,
    Sensor: HalCameraSensor<Sccb>,
    const COUNT: usize,
    const BUF_CAP: usize,
> {
    board: Board,
    bus: Sccb,
    sensor: Sensor,
    config: CameraConfig,
    xclk: XclkController,
    /// The I2S0 controller: `None` when another owner holds I2S0 (LL-13,
    /// Xtensa only), in which case `init` fails with `InvalidState`.
    i2s: Option<Esp32I2sCamera>,
    i2s_active: bool,
    stop_failed: bool,
    dma_cfg: Option<DmaSizeConfig>,
    pool: FramePool<COUNT, BUF_CAP>,
    timestamp_counter_us: u64,
    pwdn_pin: Option<u8>,
    reset_pin: Option<u8>,
    max_frame_size: FrameSize,
}

impl<
        Board: BoardPins,
        Sccb: SccbBus,
        Sensor: HalCameraSensor<Sccb>,
        const COUNT: usize,
        const BUF_CAP: usize,
    > CameraDriverInner<Board, Sccb, Sensor, COUNT, BUF_CAP>
{
    /// Quiesces active I2S0 interrupts/DMA/clocks, disables XCLK, and returns
    /// all frame pool slots (`M-116`). Idempotent so repeated teardown or
    /// `Drop` after explicit `deinit()` is a clean no-op.
    fn teardown_hardware(&mut self) {
        if let Some(i2s) = self.i2s.as_mut() {
            if self.i2s_active || i2s.is_running() {
                i2s.vsync_intr_enable(false);
                let _ = i2s.stop();
                let _ = i2s.deinit();
            }
        }
        self.i2s_active = false;
        self.stop_failed = false;
        self.dma_cfg = None;
        if self.xclk.is_enabled() {
            self.xclk.disable_out_clock();
        }
        self.pool.return_all();
    }

    /// Stops active capture (disabling VSYNC before `stop()`), deinitializes
    /// I2S0, disables XCLK, and clears the pool (`M-116`, `M-199`).
    fn deinit_hardware(&mut self) -> Result<(), CameraError> {
        let mut first_err = if self.stop_failed {
            Some(CameraError::HardwareError)
        } else {
            None
        };
        if let Some(i2s) = self.i2s.as_mut() {
            i2s.vsync_intr_enable(false);
            if !i2s.stop() && first_err.is_none() {
                first_err = Some(CameraError::HardwareError);
            }
            if let Err(e) = i2s.deinit() {
                if first_err.is_none() {
                    first_err = Some(e);
                }
            }
        }
        self.i2s_active = false;
        self.stop_failed = false;
        self.dma_cfg = None;
        if self.xclk.is_enabled() {
            self.xclk.disable_out_clock();
        }
        self.pool.return_all();
        match first_err {
            Some(err) => Err(err),
            None => Ok(()),
        }
    }
}

/// Transparent result alias for handle-preserving fallible [`CameraDriver`]
/// state transitions (`M-116`).
pub type TransitionResult<Next, Prev> = Result<Next, (Prev, CameraError)>;

/// Type-state camera driver
/// (`CameraDriver<Board, Sccb, State, Sensor, COUNT, BUF_CAP>`), replacing
/// `driver/esp_camera.c` and `driver/cam_hal.c`.
#[derive(Debug)]
pub struct CameraDriver<
    Board: BoardPins,
    Sccb: SccbBus,
    State = Uninitialized,
    Sensor: HalCameraSensor<Sccb> = HalOv2640Sensor,
    const COUNT: usize = 2,
    const BUF_CAP: usize = 8192,
> {
    inner: Option<CameraDriverInner<Board, Sccb, Sensor, COUNT, BUF_CAP>>,
    _state: PhantomData<State>,
}

impl<
        Board: BoardPins,
        Sccb: SccbBus,
        State,
        Sensor: HalCameraSensor<Sccb>,
        const COUNT: usize,
        const BUF_CAP: usize,
    > Drop for CameraDriver<Board, Sccb, State, Sensor, COUNT, BUF_CAP>
{
    fn drop(&mut self) {
        if let Some(inner) = self.inner.as_mut() {
            inner.teardown_hardware();
        }
    }
}

impl<
        Board: BoardPins,
        Sccb: SccbBus,
        State,
        Sensor: HalCameraSensor<Sccb>,
        const COUNT: usize,
        const BUF_CAP: usize,
    > CameraDriver<Board, Sccb, State, Sensor, COUNT, BUF_CAP>
{
    #[inline]
    const fn inner(&self) -> &CameraDriverInner<Board, Sccb, Sensor, COUNT, BUF_CAP> {
        match self.inner.as_ref() {
            Some(inner) => inner,
            None => unreachable!(),
        }
    }

    #[inline]
    fn inner_mut(&mut self) -> &mut CameraDriverInner<Board, Sccb, Sensor, COUNT, BUF_CAP> {
        match self.inner.as_mut() {
            Some(inner) => inner,
            None => unreachable!(),
        }
    }

    /// Moves `self.inner` into a new typestate without triggering hardware
    /// teardown in `Drop`.
    #[inline]
    fn transition<NextState>(
        mut self,
    ) -> CameraDriver<Board, Sccb, NextState, Sensor, COUNT, BUF_CAP> {
        CameraDriver {
            inner: self.inner.take(),
            _state: PhantomData,
        }
    }
}

impl<
        Board: BoardPins,
        Sccb: SccbBus,
        Sensor: HalCameraSensor<Sccb>,
        const COUNT: usize,
        const BUF_CAP: usize,
    > CameraDriver<Board, Sccb, Uninitialized, Sensor, COUNT, BUF_CAP>
{
    /// Creates a new [`CameraDriver`] in the [`Uninitialized`] type-state.
    pub fn new(board: Board, bus: Sccb, sensor: Sensor, config: CameraConfig) -> Self {
        let grab_mode = config.grab_mode;
        Self {
            inner: Some(CameraDriverInner {
                board,
                bus,
                sensor,
                config,
                xclk: XclkController::new(),
                i2s: new_i2s(),
                i2s_active: false,
                stop_failed: false,
                dma_cfg: None,
                pool: FramePool::new(grab_mode),
                timestamp_counter_us: 0,
                pwdn_pin: None,
                reset_pin: None,
                max_frame_size: config.frame_size,
            }),
            _state: PhantomData,
        }
    }

    /// Initializes the board pins, `XCLK` generator, `I2S0` peripheral, DMA
    /// sizing, and sensor registers, transitioning the driver from
    /// [`Uninitialized`] to [`Initialized`] (`esp_camera_init` in
    /// `driver/esp_camera.c:305-393`).
    ///
    /// This convenience wrapper discards `self` on error (after hardware
    /// teardown). Use [`Self::try_init`] to preserve the driver and recover
    /// `(Board, Sccb, Sensor)` handles via [`CameraDriver::into_parts`] after a
    /// transient failure (`M-116`).
    pub fn init(
        self,
    ) -> Result<CameraDriver<Board, Sccb, Initialized, Sensor, COUNT, BUF_CAP>, CameraError> {
        self.try_init().map_err(|(_driver, err)| err)
    }

    /// Handle-preserving counterpart to [`Self::init`] (`M-116`).
    ///
    /// On any initialization error (e.g. transient `CameraError::SccbNack`),
    /// tears down active I2S0/XCLK hardware and returns `Err((self, err))` so
    /// the caller can retry or reclaim `(Board, Sccb, Sensor)` via
    /// [`CameraDriver::into_parts`].
    #[allow(clippy::result_large_err)]
    pub fn try_init(
        mut self,
    ) -> TransitionResult<CameraDriver<Board, Sccb, Initialized, Sensor, COUNT, BUF_CAP>, Self>
    {
        // Execute full initialization; if any step fails, tear down I2S0
        // (vsync_intr_enable(false) + stop() + deinit()) and XCLK before
        // returning Err((self, e)) (HAL-003, M-116).
        let setup_res = (|| -> Result<DmaSizeConfig, CameraError> {
            let inner = self.inner_mut();
            if inner.config.fb_count == 0
                || inner.config.fb_count > COUNT
                || COUNT == 0
                || BUF_CAP == 0
            {
                return Err(CameraError::InvalidArg);
            }

            // 1. Validate board pinout (FW-001 / DMA-102)
            inner.board.validate_pins()?;
            let pins = inner.board.pins();
            inner.pwdn_pin = pins.pwdn;
            inner.reset_pin = pins.reset;

            // 2. Enable XCLK output
            let xclk_cfg = XclkConfig {
                pin_xclk: pins.xclk,
                ledc_timer: inner.config.ledc_timer,
                ledc_channel: inner.config.ledc_channel,
                xclk_freq_hz: inner.config.xclk_freq_hz,
            };
            inner.xclk.enable_out_clock(&xclk_cfg)?;

            // 3. Configure low-level I2S0 pins on original ESP32 targets
            //    (HAL-003). ESP32-S2/S3 use LCD_CAM whose GPIO/flash/PSRAM
            //    maps differ from original ESP32 I2S0 (already validated by
            //    `board.validate_pins()`).
            let i2s = inner.i2s.as_mut().ok_or(CameraError::InvalidState)?;
            inner.i2s_active = true;
            if pins.chip == TargetChip::Esp32 {
                let ll_pins = pins.to_ll_pin_config(true);
                i2s.set_pin(&ll_pins)?;
            } else if cfg!(target_arch = "xtensa") {
                return Err(CameraError::NotSupported);
            }
            i2s.config()?;

            // 4. Check sensor capabilities against requested format and
            // resolution (`esp_camera.c:337-346`)
            let model_info = inner.sensor.model().info();
            if inner.config.pixel_format == PixelFormat::Jpeg && !model_info.support_jpeg {
                return Err(CameraError::NotSupported);
            }

            let mut frame_size = inner.config.frame_size;
            if (frame_size as u8) > (model_info.max_size as u8) {
                frame_size = model_info.max_size;
                inner.config.frame_size = frame_size;
            }
            inner.max_frame_size = inner.config.frame_size;

            // 5. Configure sampling mode and DMA sizes (`cam_config` in
            // `driver/cam_hal.c:595-666`)
            let sensor_pid = inner.sensor.id().pid;
            let sample_cfg = inner
                .i2s
                .as_mut()
                .ok_or(CameraError::InvalidState)?
                .set_sample_mode(
                    inner.config.pixel_format,
                    inner.config.xclk_freq_hz,
                    sensor_pid,
                )?;
            let res = frame_size.resolution();
            let dma_cfg = calc_dma_sizes(
                inner.config.pixel_format == PixelFormat::Jpeg,
                sample_cfg.sampling_mode,
                res.width,
                res.height,
                sample_cfg.in_bytes_per_pixel,
                DEFAULT_CAMERA_DMA_BUFFER_SIZE_MAX,
            )?;

            // 6. Reset and configure sensor over SCCB
            // (`esp_camera.c:282, 358-385`)
            inner.sensor.reset(&mut inner.bus)?;
            // HAL-09: a sensor that cannot do these reports NotSupported
            // rather than pretending, as the built-in stub now does. Tolerate
            // it here the same way the other call sites already do, so a
            // driver built on a partial sensor still initialises — it just no
            // longer claims the sensor was configured when it was not.
            let pixformat_res = inner
                .sensor
                .set_pixformat(&mut inner.bus, inner.config.pixel_format);
            ignore_not_supported(pixformat_res)?;
            let framesize_res = inner.sensor.set_framesize(&mut inner.bus, frame_size);
            ignore_not_supported(framesize_res)?;

            if sensor_pid == CameraPid::OV2640.as_u16() {
                ignore_not_supported(
                    inner
                        .sensor
                        .set_gainceiling(&mut inner.bus, GainCeiling::Gain2x),
                )?;
                ignore_not_supported(inner.sensor.set_bpc(&mut inner.bus, false))?;
                ignore_not_supported(inner.sensor.set_wpc(&mut inner.bus, true))?;
                ignore_not_supported(inner.sensor.set_lenc(&mut inner.bus, true))?;
            }

            if inner.config.pixel_format == PixelFormat::Jpeg {
                ignore_not_supported(
                    inner
                        .sensor
                        .set_quality(&mut inner.bus, inner.config.jpeg_quality as i32),
                )?;
            }
            ignore_not_supported(inner.sensor.init_status(&mut inner.bus))?;

            Ok(dma_cfg)
        })();

        let dma_cfg = match setup_res {
            Ok(cfg) => cfg,
            Err(e) => {
                self.inner_mut().teardown_hardware();
                return Err((self, e));
            }
        };

        self.inner_mut().dma_cfg = Some(dma_cfg);
        // HAL-003: Do NOT enable vsync_intr_enable(true) inside init();
        // deferred to start_streaming()

        Ok(self.transition())
    }
}

impl<
        Board: BoardPins,
        Sccb: SccbBus,
        State,
        Sensor: HalCameraSensor<Sccb>,
        const COUNT: usize,
        const BUF_CAP: usize,
    > CameraDriver<Board, Sccb, State, Sensor, COUNT, BUF_CAP>
{
    /// Returns a shared reference to the underlying camera sensor.
    #[inline]
    pub const fn sensor(&self) -> &Sensor {
        &self.inner().sensor
    }

    /// Returns a shared reference to the SCCB bus.
    #[inline]
    pub const fn bus(&self) -> &Sccb {
        &self.inner().bus
    }

    /// Returns a shared reference to the active [`CameraConfig`].
    #[inline]
    pub const fn config(&self) -> &CameraConfig {
        &self.inner().config
    }

    /// Returns the computed [`DmaSizeConfig`], if initialized.
    #[inline]
    pub const fn dma_config(&self) -> Option<&DmaSizeConfig> {
        self.inner().dma_cfg.as_ref()
    }

    /// Returns a shared reference to the [`FramePool`].
    #[inline]
    pub const fn pool(&self) -> &FramePool<COUNT, BUF_CAP> {
        &self.inner().pool
    }

    /// Returns a mutable reference to the [`FramePool`].
    #[inline]
    pub fn pool_mut(&mut self) -> &mut FramePool<COUNT, BUF_CAP> {
        &mut self.inner_mut().pool
    }

    /// Returns the validated PWDN pin, if configured.
    #[inline]
    pub const fn pwdn_pin(&self) -> Option<u8> {
        self.inner().pwdn_pin
    }

    /// Returns the validated RESET pin, if configured.
    #[inline]
    pub const fn reset_pin(&self) -> Option<u8> {
        self.inner().reset_pin
    }

    /// Returns the maximum allowed frame size configured at initialization.
    #[inline]
    pub const fn max_frame_size(&self) -> FrameSize {
        self.inner().max_frame_size
    }

    /// Quiesces any active camera hardware (stopping VSYNC/I2S0 DMA,
    /// deinitializing I2S0, disabling XCLK, and clearing the frame pool) and
    /// returns the owned `(Board, Sccb, Sensor)` handles (`M-116`).
    pub fn into_parts(mut self) -> (Board, Sccb, Sensor) {
        let mut inner = match self.inner.take() {
            Some(inner) => inner,
            None => unreachable!(),
        };
        inner.teardown_hardware();
        (inner.board, inner.bus, inner.sensor)
    }

    #[cfg(test)]
    fn i2s_running(&self) -> bool {
        self.inner()
            .i2s
            .as_ref()
            .is_some_and(Esp32I2sCamera::is_running)
    }

    #[cfg(test)]
    fn xclk_enabled(&self) -> bool {
        self.inner().xclk.is_enabled()
    }

    #[cfg(test)]
    fn i2s_active(&self) -> bool {
        self.inner().i2s_active
    }
}

impl<
        Board: BoardPins,
        Sccb: SccbBus,
        State: NonStreamingState,
        Sensor: HalCameraSensor<Sccb>,
        const COUNT: usize,
        const BUF_CAP: usize,
    > CameraDriver<Board, Sccb, State, Sensor, COUNT, BUF_CAP>
{
    /// Returns a mutable reference to the underlying camera sensor
    /// (`esp_camera_sensor_get`).
    ///
    /// Restricted to [`Uninitialized`] and [`Initialized`] states (`M-117`) so
    /// sensor format/resolution cannot be mutated underneath active DMA.
    #[inline]
    pub fn sensor_mut(&mut self) -> &mut Sensor {
        &mut self.inner_mut().sensor
    }

    /// Returns a mutable reference to the SCCB bus.
    ///
    /// Restricted to [`Uninitialized`] and [`Initialized`] states (`M-117`).
    #[inline]
    pub fn bus_mut(&mut self) -> &mut Sccb {
        &mut self.inner_mut().bus
    }

    /// Allows mutating sensor settings with simultaneous mutable access to both
    /// `Sensor` and `Sccb`.
    ///
    /// Restricted to [`Uninitialized`] and [`Initialized`] states (`M-117`) so
    /// sensor format/resolution cannot be mutated underneath active DMA.
    #[inline]
    pub fn with_sensor_and_bus<F, R>(&mut self, f: F) -> R
    where
        F: FnOnce(&mut Sensor, &mut Sccb) -> R,
    {
        let inner = self.inner_mut();
        f(&mut inner.sensor, &mut inner.bus)
    }
}

impl<
        Board: BoardPins,
        Sccb: SccbBus,
        Sensor: HalCameraSensor<Sccb>,
        const COUNT: usize,
        const BUF_CAP: usize,
    > CameraDriver<Board, Sccb, Initialized, Sensor, COUNT, BUF_CAP>
{
    /// Transitions the driver from [`Initialized`] into the [`Streaming`]
    /// type-state (`cam_start`).
    ///
    /// This convenience wrapper discards `self` on error (after hardware
    /// teardown). Use [`Self::try_start_streaming`] to preserve the driver and
    /// recover `(Board, Sccb, Sensor)` handles via [`CameraDriver::into_parts`]
    /// on failure (`M-116`).
    pub fn start_streaming(
        self,
    ) -> Result<CameraDriver<Board, Sccb, Streaming, Sensor, COUNT, BUF_CAP>, CameraError> {
        self.try_start_streaming().map_err(|(_driver, err)| err)
    }

    /// Handle-preserving counterpart to [`Self::start_streaming`] (`M-116`).
    ///
    /// On failure (e.g. `CameraError::NotSupported` or `HardwareError`), tears
    /// down active hardware and returns `Err((self, err))` so the caller can
    /// reclaim `(Board, Sccb, Sensor)` via [`CameraDriver::into_parts`].
    #[allow(clippy::result_large_err)]
    pub fn try_start_streaming(
        mut self,
    ) -> TransitionResult<CameraDriver<Board, Sccb, Streaming, Sensor, COUNT, BUF_CAP>, Self> {
        let start_res = (|| -> Result<(), CameraError> {
            let inner = self.inner_mut();
            if inner.stop_failed {
                return Err(CameraError::HardwareError);
            }
            let Some(dma_cfg) = inner.dma_cfg else {
                return Err(CameraError::InvalidState);
            };
            let Some(i2s) = inner.i2s.as_mut() else {
                return Err(CameraError::InvalidState);
            };
            i2s.vsync_intr_enable(true);
            start_dma(i2s, &dma_cfg)
        })();
        if let Err(e) = start_res {
            self.inner_mut().teardown_hardware();
            return Err((self, e));
        }
        Ok(self.transition())
    }

    /// Assembles a frame from DMA half-buffers, performs JPEG `SOI`/`EOI`
    /// verification and trimming, stores it in [`FramePool`], and returns an
    /// RAII [`FrameGuard`].
    pub fn capture_frame(
        &mut self,
        dma_half_buffers: &[&[u8]],
    ) -> Result<FrameGuard<'_>, CameraError> {
        let inner = self.inner_mut();
        assemble_and_lease_frame(
            inner.i2s.as_mut().ok_or(CameraError::InvalidState)?,
            &inner.dma_cfg,
            &inner.config,
            &inner.sensor,
            &mut inner.pool,
            &mut inner.timestamp_counter_us,
            dma_half_buffers,
        )
    }

    /// Exports the current sensor status and pixel format for NVS persistence
    /// (`esp_camera_save_to_nvs`).
    pub fn save_settings(&self) -> SavedCameraSettings {
        let inner = self.inner();
        SavedCameraSettings {
            status: *inner.sensor.status(),
            pixformat: inner.config.pixel_format,
        }
    }

    /// Restores sensor settings from a [`SavedCameraSettings`] snapshot,
    /// enforcing the DMA overflow and pixel format compatibility guards from
    /// `driver/esp_camera.c:514-548` and recomputing I2S sample mode and DMA
    /// sizes on accepted format/size changes (`M-117`).
    pub fn load_settings(&mut self, saved: &SavedCameraSettings) -> Result<(), CameraError> {
        let inner = self.inner_mut();
        let target_fs = saved.status.framesize;
        if target_fs > inner.max_frame_size {
            return Err(CameraError::InvalidArg);
        }
        if inner.i2s.is_none() {
            return Err(CameraError::InvalidState);
        }

        let mut next_frame_size = inner.config.frame_size;
        if target_fs != inner.config.frame_size {
            match inner.sensor.set_framesize(&mut inner.bus, target_fs) {
                Ok(()) => next_frame_size = target_fs,
                Err(CameraError::NotSupported) => {}
                Err(e) => return Err(e),
            }
        }

        let mut next_pixel_format = inner.config.pixel_format;
        let cur_jpeg = inner.config.pixel_format == PixelFormat::Jpeg;
        let new_jpeg = saved.pixformat == PixelFormat::Jpeg;
        if cur_jpeg == new_jpeg && saved.pixformat != inner.config.pixel_format {
            match inner.sensor.set_pixformat(&mut inner.bus, saved.pixformat) {
                Ok(()) => next_pixel_format = saved.pixformat,
                Err(CameraError::NotSupported) => {}
                Err(e) => return Err(e),
            }
        }

        let quality_applied = match inner
            .sensor
            .set_quality(&mut inner.bus, saved.status.quality as i32)
        {
            Ok(()) => true,
            Err(CameraError::NotSupported) => false,
            Err(e) => return Err(e),
        };
        ignore_not_supported(
            inner
                .sensor
                .set_hmirror(&mut inner.bus, saved.status.hmirror),
        )?;
        ignore_not_supported(inner.sensor.set_vflip(&mut inner.bus, saved.status.vflip))?;

        let recomputed_dma_cfg = if next_frame_size != inner.config.frame_size
            || next_pixel_format != inner.config.pixel_format
        {
            let sensor_pid = inner.sensor.id().pid;
            let sample_cfg = inner
                .i2s
                .as_mut()
                .ok_or(CameraError::InvalidState)?
                .set_sample_mode(next_pixel_format, inner.config.xclk_freq_hz, sensor_pid)?;
            let res = next_frame_size.resolution();
            Some(calc_dma_sizes(
                next_pixel_format == PixelFormat::Jpeg,
                sample_cfg.sampling_mode,
                res.width,
                res.height,
                sample_cfg.in_bytes_per_pixel,
                DEFAULT_CAMERA_DMA_BUFFER_SIZE_MAX,
            )?)
        } else {
            None
        };

        // Commit config and DMA sizing only after all sensor and LL steps
        // succeeded (M-117).
        if let Some(dma_cfg) = recomputed_dma_cfg {
            inner.dma_cfg = Some(dma_cfg);
        }
        inner.config.frame_size = next_frame_size;
        inner.config.pixel_format = next_pixel_format;
        if quality_applied {
            inner.config.jpeg_quality = saved.status.quality;
        }
        Ok(())
    }

    /// Stops DMA capture, disables XCLK, and returns the driver to the
    /// [`Uninitialized`] type-state (`esp_camera_deinit`).
    ///
    /// Use [`Self::try_deinit`] to preserve the driver on hardware teardown
    /// failure and recover `(Board, Sccb, Sensor)` via
    /// [`CameraDriver::into_parts`] (`M-116`).
    pub fn deinit(
        self,
    ) -> Result<CameraDriver<Board, Sccb, Uninitialized, Sensor, COUNT, BUF_CAP>, CameraError> {
        self.try_deinit().map_err(|(_driver, err)| err)
    }

    /// Handle-preserving counterpart to [`Self::deinit`] (`M-116`).
    #[allow(clippy::result_large_err)]
    pub fn try_deinit(
        mut self,
    ) -> TransitionResult<CameraDriver<Board, Sccb, Uninitialized, Sensor, COUNT, BUF_CAP>, Self>
    {
        if let Err(e) = self.inner_mut().deinit_hardware() {
            return Err((self, e));
        }
        Ok(self.transition())
    }
}

impl<
        Board: BoardPins,
        Sccb: SccbBus,
        Sensor: HalCameraSensor<Sccb>,
        const COUNT: usize,
        const BUF_CAP: usize,
    > CameraDriver<Board, Sccb, Streaming, Sensor, COUNT, BUF_CAP>
{
    /// Assembles a frame from DMA half-buffers while in [`Streaming`] state,
    /// performs JPEG `SOI`/`EOI` verification and trimming, and returns an RAII
    /// [`FrameGuard`] (`esp_camera_fb_get` / `cam_take` in
    /// `driver/cam_hal.c:320-458, 730-837`).
    pub fn capture_frame(
        &mut self,
        dma_half_buffers: &[&[u8]],
    ) -> Result<FrameGuard<'_>, CameraError> {
        let inner = self.inner_mut();
        assemble_and_lease_frame(
            inner.i2s.as_mut().ok_or(CameraError::InvalidState)?,
            &inner.dma_cfg,
            &inner.config,
            &inner.sensor,
            &mut inner.pool,
            &mut inner.timestamp_counter_us,
            dma_half_buffers,
        )
    }

    /// Stops active DMA streaming and transitions back to [`Initialized`]
    /// (`cam_stop`).
    ///
    /// If `i2s.stop()` fails, the error is recorded and surfaced by subsequent
    /// [`CameraDriver::start_streaming`] or [`CameraDriver::deinit`] calls
    /// rather than silently discarded (`M-199`). Use
    /// [`Self::try_stop_streaming`] or [`Self::try_stop_streaming_recover`] to
    /// handle stop failures immediately.
    pub fn stop_streaming(
        mut self,
    ) -> CameraDriver<Board, Sccb, Initialized, Sensor, COUNT, BUF_CAP> {
        let inner = self.inner_mut();
        if let Some(i2s) = inner.i2s.as_mut() {
            i2s.vsync_intr_enable(false);
            if !i2s.stop() {
                inner.stop_failed = true;
            }
        }
        self.transition()
    }

    /// Fallible counterpart to [`Self::stop_streaming`] that immediately
    /// propagates `i2s.stop()` failure (`M-199`).
    ///
    /// Use [`Self::try_stop_streaming_recover`] to preserve `self` on failure
    /// and reclaim `(Board, Sccb, Sensor)` via [`CameraDriver::into_parts`]
    /// (`M-116`).
    pub fn try_stop_streaming(
        self,
    ) -> Result<CameraDriver<Board, Sccb, Initialized, Sensor, COUNT, BUF_CAP>, CameraError> {
        self.try_stop_streaming_recover()
            .map_err(|(_driver, err)| err)
    }

    /// Handle-preserving counterpart to [`Self::try_stop_streaming`] (`M-116`,
    /// `M-199`).
    ///
    /// On `i2s.stop()` failure, runs hardware teardown and returns
    /// `Err((self, CameraError::HardwareError))` so `(Board, Sccb, Sensor)`
    /// remain recoverable via [`CameraDriver::into_parts`].
    #[allow(clippy::result_large_err)]
    pub fn try_stop_streaming_recover(
        mut self,
    ) -> TransitionResult<CameraDriver<Board, Sccb, Initialized, Sensor, COUNT, BUF_CAP>, Self>
    {
        let stop_failed = {
            let inner = self.inner_mut();
            if let Some(i2s) = inner.i2s.as_mut() {
                i2s.vsync_intr_enable(false);
                !i2s.stop()
            } else {
                false
            }
        };
        if stop_failed {
            self.inner_mut().teardown_hardware();
            return Err((self, CameraError::HardwareError));
        }
        Ok(self.transition())
    }

    /// Stops DMA capture, disables XCLK, and returns the driver to the
    /// [`Uninitialized`] type-state (`esp_camera_deinit`).
    ///
    /// Use [`Self::try_deinit`] to preserve the driver on hardware teardown
    /// failure and recover `(Board, Sccb, Sensor)` via
    /// [`CameraDriver::into_parts`] (`M-116`).
    pub fn deinit(
        self,
    ) -> Result<CameraDriver<Board, Sccb, Uninitialized, Sensor, COUNT, BUF_CAP>, CameraError> {
        self.try_deinit().map_err(|(_driver, err)| err)
    }

    /// Handle-preserving counterpart to [`Self::deinit`] (`M-116`).
    #[allow(clippy::result_large_err)]
    pub fn try_deinit(
        mut self,
    ) -> TransitionResult<CameraDriver<Board, Sccb, Uninitialized, Sensor, COUNT, BUF_CAP>, Self>
    {
        if let Err(e) = self.inner_mut().deinit_hardware() {
            return Err((self, e));
        }
        Ok(self.transition())
    }
}

/// The I2S0 controller for a new driver: on Xtensa the single instance from
/// [`Esp32I2sCamera::take`] (LL-13), on the host an independent
/// shadow-register simulator.
fn new_i2s() -> Option<Esp32I2sCamera> {
    #[cfg(target_arch = "xtensa")]
    {
        Esp32I2sCamera::take()
    }
    #[cfg(not(target_arch = "xtensa"))]
    {
        Some(Esp32I2sCamera::new())
    }
}

/// Starts the DMA for the HAL's capture pipeline.
///
/// HAL-01 (WP-08): the HAL owns no DMA ring yet (WP-20 gives it a
/// `DmaRing<'static>`), and the low-level API no longer starts DMA at an
/// unchecked address, so on Xtensa streaming and one-shot capture return
/// [`CameraError::NotSupported`]. On the host the start is simulated on the
/// shadow registers; frames come from the caller's half-buffers.
fn start_dma(i2s: &mut Esp32I2sCamera, dma_cfg: &DmaSizeConfig) -> Result<(), CameraError> {
    #[cfg(target_arch = "xtensa")]
    {
        let _ = (i2s, dma_cfg);
        Err(CameraError::NotSupported)
    }
    #[cfg(not(target_arch = "xtensa"))]
    {
        if i2s.simulate_start(dma_cfg) {
            Ok(())
        } else {
            Err(CameraError::HardwareError)
        }
    }
}

fn assemble_and_lease_frame<
    'a,
    Sccb: SccbBus,
    Sensor: HalCameraSensor<Sccb>,
    const COUNT: usize,
    const BUF_CAP: usize,
>(
    i2s: &mut Esp32I2sCamera,
    dma_cfg_opt: &Option<DmaSizeConfig>,
    config: &CameraConfig,
    sensor: &Sensor,
    pool: &'a mut FramePool<COUNT, BUF_CAP>,
    timestamp_counter_us: &mut u64,
    dma_half_buffers: &[&[u8]],
) -> Result<FrameGuard<'a>, CameraError> {
    if dma_half_buffers.is_empty() {
        return Err(CameraError::Timeout);
    }
    let dma_cfg = dma_cfg_opt.ok_or(CameraError::InvalidState)?;

    // HAL-004 / M-199: align one-shot VSYNC and DMA start/stop lifecycle with
    // streaming mode; only start if !i2s.is_running().
    let was_running = i2s.is_running();
    if !was_running {
        i2s.vsync_intr_enable(true);
        if let Err(e) = start_dma(i2s, &dma_cfg) {
            i2s.vsync_intr_enable(false);
            return Err(e);
        }
    }

    let fs = sensor.status().framesize;
    let res = fs.resolution();
    let format = config.pixel_format;
    let is_jpeg = format == PixelFormat::Jpeg;
    let psram_mode = config.psram_dma_mode;

    // M-198: defer timestamp increment until frame assembly succeeds.
    let next_ts = timestamp_counter_us.wrapping_add(33_333);

    let filter = i2s.filter();

    let result = pool.write_and_lease(res.width, res.height, format, next_ts, |fb_buf| {
        let mut total_written = 0usize;
        let mut soi_checked = false;

        for half_buf in dma_half_buffers {
            let written = if psram_mode {
                let Some(end) = total_written.checked_add(half_buf.len()) else {
                    return Err(CameraError::BufferTooSmall);
                };
                if end > fb_buf.len() {
                    return Err(CameraError::BufferTooSmall);
                }
                fb_buf[total_written..end].copy_from_slice(half_buf);
                half_buf.len()
            } else {
                let rem_cap = fb_buf.len().saturating_sub(total_written);
                let expected_out = match filter {
                    esp32_cam_ll::DmaFilterKind::Jpeg
                    | esp32_cam_ll::DmaFilterKind::YuyvHighspeed => half_buf.len() / 4,
                    esp32_cam_ll::DmaFilterKind::Grayscale | esp32_cam_ll::DmaFilterKind::Yuyv => {
                        half_buf.len() / 2
                    }
                    esp32_cam_ll::DmaFilterKind::GrayscaleHighspeed => half_buf.len() / 8,
                };
                // M-198: a non-empty fragment shorter than one DMA stride is a
                // malformed/truncated DMA chunk, not an output buffer overflow.
                if expected_out == 0 && !half_buf.is_empty() {
                    return Err(CameraError::CorruptedFrame);
                }
                if rem_cap < expected_out {
                    return Err(CameraError::BufferTooSmall);
                }
                let unpacked = filter.apply(&mut fb_buf[total_written..], half_buf);
                if unpacked == 0 && !half_buf.is_empty() {
                    return Err(CameraError::CorruptedFrame);
                }
                unpacked
            };

            total_written = total_written
                .checked_add(written)
                .ok_or(CameraError::BufferTooSmall)?;

            // M-115: validate JPEG SOI strictly against `&fb_buf[..total_written]`
            // once enough bytes have been written, never against unzeroed slack.
            if is_jpeg && !soi_checked && total_written >= JPEG_SOI_MARKER.len() {
                if !fb_buf[..total_written].starts_with(&JPEG_SOI_MARKER) {
                    return Err(CameraError::CorruptedFrame);
                }
                soi_checked = true;
            }
        }

        if is_jpeg {
            if total_written < JPEG_SOI_MARKER.len()
                || !fb_buf[..total_written].starts_with(&JPEG_SOI_MARKER)
            {
                return Err(CameraError::CorruptedFrame);
            }

            // Locate JPEG EOI marker (`0xFF, 0xD9`) and trim trailing padding
            // bytes (DMA-101 / PSRAM garbage, `cam_hal.c:787-817`)
            let last_half_len = dma_half_buffers
                .last()
                .map(|b| b.len())
                .unwrap_or(dma_cfg.dma_half_buffer_size);
            let probe_len = eoi_probe_window(last_half_len, total_written);
            let probe_start = total_written.saturating_sub(probe_len);

            let offset_e = if psram_mode {
                verify_jpeg_eoi(&fb_buf[probe_start..total_written], true)
                    .map(|off| probe_start + off)
            } else {
                verify_jpeg_eoi(&fb_buf[probe_start..total_written], false)
                    .map(|off| probe_start + off)
            };

            match offset_e {
                Some(eoi_pos) => Ok(eoi_pos + JPEG_EOI_MARKER.len()),
                None => Err(CameraError::CorruptedFrame),
            }
        } else {
            Ok(total_written)
        }
    });

    if !was_running {
        i2s.vsync_intr_enable(false);
        if !i2s.stop() {
            drop(result);
            return Err(CameraError::HardwareError);
        }
    }
    if result.is_ok() {
        *timestamp_counter_us = next_ts;
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board_pins::WroverKitPins;
    use esp32_cam_ll::DmaElement;
    use esp32_cam_sccb::MockSccbBus;
    use esp32_cam_types::CameraSccbAddr;

    fn pack_dma_elements<const N: usize>(bytes: &[u8]) -> [u8; N] {
        let mut out = [0u8; N];
        for (i, &b) in bytes.iter().enumerate() {
            if i * 4 + 4 <= N {
                let el = DmaElement::new(b, 0);
                out[i * 4..i * 4 + 4].copy_from_slice(&el.0);
            }
        }
        out
    }

    #[test]
    fn test_verify_jpeg_soi_and_eoi() {
        let valid_jpeg = [
            0xFF, 0xD8, 0xFF, 0xE0, 0x00, 0x10, 0x11, 0x22, 0xFF, 0xD9, 0x00, 0x00,
        ];
        assert_eq!(verify_jpeg_soi(&valid_jpeg), Some(0));
        assert_eq!(verify_jpeg_eoi(&valid_jpeg, false), Some(8));
        assert_eq!(verify_jpeg_eoi(&valid_jpeg, true), Some(8));

        // Forward vs backward EOI search when two EOI markers exist
        // (stale tail in PSRAM)
        let double_eoi = [
            0xFF, 0xD8, 0xFF, 0xAA, 0xFF, 0xD9, 0x99, 0x88, 0xFF, 0xD9, 0x00,
        ];
        assert_eq!(verify_jpeg_eoi(&double_eoi, true), Some(4));
        assert_eq!(verify_jpeg_eoi(&double_eoi, false), Some(8));

        assert_eq!(verify_jpeg_soi(&[0xFF, 0xD8]), None);
        assert_eq!(verify_jpeg_eoi(&[0xFF], false), None);
    }

    #[test]
    fn test_typestate_camera_driver_lifecycle_and_jpeg_capture() {
        let mut bus: MockSccbBus<512> = MockSccbBus::new();
        bus.preset_reg8(CameraSccbAddr::OV2640, 0x0A, 0x26);
        bus.preset_reg8(CameraSccbAddr::OV2640, 0x0B, 0x42);
        bus.preset_reg8(CameraSccbAddr::OV2640, 0x1C, 0x7F);
        bus.preset_reg8(CameraSccbAddr::OV2640, 0x1D, 0xA2);

        let sensor =
            HalOv2640Sensor::detect(&mut bus, CameraSccbAddr::OV2640).expect("detect ov2640");
        let cfg = CameraConfig {
            pixel_format: PixelFormat::Jpeg,
            frame_size: FrameSize::Qvga,
            grab_mode: GrabMode::WhenEmpty,
            ..CameraConfig::default()
        };

        // 1. Uninitialized -> Initialized -> Streaming
        let uninit_driver: CameraDriver<WroverKitPins, _, Uninitialized, HalOv2640Sensor, 2, 256> =
            CameraDriver::new(WroverKitPins, bus, sensor, cfg);
        let init_driver = uninit_driver.init().expect("driver init");
        let mut streaming_driver = init_driver.start_streaming().expect("start streaming");

        // 2. Capture valid JPEG across 2 DMA half-buffers with trailing
        // garbage after EOI
        let half1_raw = [0xFF, 0xD8, 0xFF, 0xE0, 0x00, 0x10, 0x4A, 0x46];
        let half2_raw = [0x49, 0x46, 0x00, 0x01, 0xFF, 0xD9, 0xAA, 0xBB];
        let dma_half1: [u8; 32] = pack_dma_elements(&half1_raw);
        let dma_half2: [u8; 32] = pack_dma_elements(&half2_raw);

        {
            let frame = streaming_driver
                .capture_frame(&[&dma_half1, &dma_half2])
                .expect("capture valid jpeg frame");
            // Trailing 0xAA, 0xBB after 0xFF, 0xD9 must be trimmed!
            assert_eq!(frame.len(), 14);
            assert_eq!(
                frame.as_slice(),
                &[
                    0xFF, 0xD8, 0xFF, 0xE0, 0x00, 0x10, 0x4A, 0x46, 0x49, 0x46, 0x00, 0x01, 0xFF,
                    0xD9
                ]
            );
            assert_eq!(frame.width(), 320);
            assert_eq!(frame.height(), 240);
            assert_eq!(frame.format(), PixelFormat::Jpeg);
        }
        // FrameGuard dropped -> pool has all 2 buffers free again
        assert_eq!(streaming_driver.pool().free_count(), 2);

        // 3. Reject corrupted JPEG missing SOI
        let bad_soi_raw = [0x00, 0xD8, 0xFF, 0xE0, 0xFF, 0xD9, 0x00, 0x00];
        let bad_soi_dma: [u8; 32] = pack_dma_elements(&bad_soi_raw);
        assert_eq!(
            streaming_driver
                .capture_frame(&[&bad_soi_dma])
                .expect_err("missing SOI must fail"),
            CameraError::CorruptedFrame
        );
        assert_eq!(streaming_driver.pool().free_count(), 2);

        // 4. Reject corrupted JPEG missing EOI
        let bad_eoi_raw = [0xFF, 0xD8, 0xFF, 0xE0, 0x11, 0x22, 0x33, 0x44];
        let bad_eoi_dma: [u8; 32] = pack_dma_elements(&bad_eoi_raw);
        assert_eq!(
            streaming_driver
                .capture_frame(&[&bad_eoi_dma])
                .expect_err("missing EOI must fail"),
            CameraError::CorruptedFrame
        );

        // 5. Streaming -> Initialized -> Uninitialized
        let mut init_driver = streaming_driver.stop_streaming();
        let saved = init_driver.save_settings();
        assert_eq!(init_driver.load_settings(&saved), Ok(()));
        let _uninit = init_driver.deinit().expect("deinit");
    }

    #[test]
    fn test_hal_002_non_psram_buffer_too_small_detected() {
        let mut bus: MockSccbBus<512> = MockSccbBus::new();
        bus.preset_reg8(CameraSccbAddr::OV2640, 0x0A, 0x26);
        bus.preset_reg8(CameraSccbAddr::OV2640, 0x0B, 0x42);
        bus.preset_reg8(CameraSccbAddr::OV2640, 0x1C, 0x7F);
        bus.preset_reg8(CameraSccbAddr::OV2640, 0x1D, 0xA2);

        let sensor =
            HalOv2640Sensor::detect(&mut bus, CameraSccbAddr::OV2640).expect("detect ov2640");
        let cfg = CameraConfig {
            pixel_format: PixelFormat::Jpeg,
            frame_size: FrameSize::Qvga,
            ..CameraConfig::default()
        };

        // Frame pool buffer capacity is only 4 bytes, but unpacked JPEG
        // half-buffer is 8 bytes
        let driver: CameraDriver<WroverKitPins, _, Uninitialized, HalOv2640Sensor, 2, 4> =
            CameraDriver::new(WroverKitPins, bus, sensor, cfg);
        let mut init_driver = driver.init().expect("driver init");

        let half_raw = [0xFF, 0xD8, 0xFF, 0xE0, 0x00, 0x10, 0x4A, 0x46];
        let dma_half: [u8; 32] = pack_dma_elements(&half_raw);

        // Expected out is 8 bytes, but buffer capacity is only 4 bytes!
        let res = init_driver.capture_frame(&[&dma_half]);
        assert_eq!(res.err(), Some(CameraError::BufferTooSmall));
    }

    #[test]
    fn test_hal_003_hal_004_streaming_vs_oneshot_i2s_lifecycle() {
        let mut bus: MockSccbBus<512> = MockSccbBus::new();
        bus.preset_reg8(CameraSccbAddr::OV2640, 0x0A, 0x26);
        bus.preset_reg8(CameraSccbAddr::OV2640, 0x0B, 0x42);
        bus.preset_reg8(CameraSccbAddr::OV2640, 0x1C, 0x7F);
        bus.preset_reg8(CameraSccbAddr::OV2640, 0x1D, 0xA2);

        let sensor =
            HalOv2640Sensor::detect(&mut bus, CameraSccbAddr::OV2640).expect("detect ov2640");
        let cfg = CameraConfig {
            pixel_format: PixelFormat::Jpeg,
            frame_size: FrameSize::Qvga,
            ..CameraConfig::default()
        };

        let driver: CameraDriver<WroverKitPins, _, Uninitialized, HalOv2640Sensor, 2, 256> =
            CameraDriver::new(WroverKitPins, bus, sensor, cfg);
        let mut init_driver = driver.init().expect("driver init");

        // 1. In Initialized state, i2s is not yet running (HAL-003)
        assert!(!init_driver.i2s_running());

        // 2. One-shot capture from Initialized state starts and then stops i2s
        // (HAL-004)
        let half1 = pack_dma_elements::<32>(&[0xFF, 0xD8, 0xFF, 0xE0, 0x00, 0x10, 0xFF, 0xD9]);
        {
            let frame = init_driver
                .capture_frame(&[&half1])
                .expect("one-shot capture");
            assert_eq!(frame.len(), 8);
        }
        // After one-shot capture, i2s is stopped
        assert!(!init_driver.i2s_running());

        // 3. Start streaming: i2s becomes active
        let mut streaming = init_driver.start_streaming().expect("start streaming");
        assert!(streaming.i2s_running());

        // 4. Capture while streaming: i2s remains active throughout! (HAL-004)
        {
            let frame = streaming
                .capture_frame(&[&half1])
                .expect("streaming capture");
            assert_eq!(frame.len(), 8);
        }
        assert!(streaming.i2s_running());

        // 5. Deinit stops and tears down
        let stopped = streaming.stop_streaming();
        assert!(!stopped.i2s_running());
        let _uninit = stopped.deinit().expect("deinit succeeds");
    }

    #[derive(Debug, Clone)]
    struct MockConfigurableSensor {
        id: SensorId,
        status: CameraStatus,
        pixformat: PixelFormat,
        fail_reset: bool,
        fail_reset_nack: bool,
        fail_vflip: bool,
    }

    impl MockConfigurableSensor {
        fn new() -> Self {
            Self {
                id: SensorId {
                    midh: 0x7F,
                    midl: 0xA2,
                    pid: CameraPid::OV2640.as_u16(),
                    ver: 0x42,
                },
                status: CameraStatus::default(),
                pixformat: PixelFormat::Jpeg,
                fail_reset: false,
                fail_reset_nack: false,
                fail_vflip: false,
            }
        }

        const fn status(&self) -> &CameraStatus {
            &self.status
        }
    }

    #[cfg(not(feature = "sensors"))]
    impl<B: SccbBus> HalCameraSensor<B> for MockConfigurableSensor {
        fn model(&self) -> CameraModel {
            CameraModel::Ov2640
        }

        fn id(&self) -> &SensorId {
            &self.id
        }

        fn status(&self) -> &CameraStatus {
            &self.status
        }

        fn status_mut(&mut self) -> &mut CameraStatus {
            &mut self.status
        }

        fn reset(&mut self, _bus: &mut B) -> Result<(), CameraError> {
            if self.fail_reset_nack {
                return Err(CameraError::SccbNack {
                    slv_addr: CameraSccbAddr::OV2640,
                    phase: esp32_cam_types::SccbPhase::DeviceAddr,
                });
            }
            if self.fail_reset {
                return Err(CameraError::I2cError);
            }
            Ok(())
        }

        fn set_pixformat(
            &mut self,
            _bus: &mut B,
            pixformat: PixelFormat,
        ) -> Result<(), CameraError> {
            self.pixformat = pixformat;
            Ok(())
        }

        fn set_framesize(&mut self, _bus: &mut B, framesize: FrameSize) -> Result<(), CameraError> {
            self.status.framesize = framesize;
            Ok(())
        }

        fn set_quality(&mut self, _bus: &mut B, quality: i32) -> Result<(), CameraError> {
            self.status.quality = quality.clamp(0, 63) as u8;
            Ok(())
        }

        fn set_bpc(&mut self, _bus: &mut B, enable: bool) -> Result<(), CameraError> {
            self.status.bpc = enable;
            Ok(())
        }

        fn set_wpc(&mut self, _bus: &mut B, enable: bool) -> Result<(), CameraError> {
            self.status.wpc = enable;
            Ok(())
        }

        fn set_lenc(&mut self, _bus: &mut B, enable: bool) -> Result<(), CameraError> {
            self.status.lenc = enable;
            Ok(())
        }

        fn set_hmirror(&mut self, _bus: &mut B, enable: bool) -> Result<(), CameraError> {
            self.status.hmirror = enable;
            Ok(())
        }

        fn set_vflip(&mut self, _bus: &mut B, enable: bool) -> Result<(), CameraError> {
            if self.fail_vflip {
                return Err(CameraError::I2cError);
            }
            self.status.vflip = enable;
            Ok(())
        }
    }

    #[cfg(feature = "sensors")]
    impl<B: SccbBus> esp32_cam_sensors::CameraSensor<B> for MockConfigurableSensor {
        fn model(&self) -> CameraModel {
            CameraModel::Ov2640
        }

        fn id(&self) -> &SensorId {
            &self.id
        }

        fn status(&self) -> &CameraStatus {
            &self.status
        }

        fn status_mut(&mut self) -> &mut CameraStatus {
            &mut self.status
        }

        fn reset(&mut self, _bus: &mut B) -> Result<(), CameraError> {
            if self.fail_reset_nack {
                return Err(CameraError::SccbNack {
                    slv_addr: CameraSccbAddr::OV2640,
                    phase: esp32_cam_types::SccbPhase::DeviceAddr,
                });
            }
            if self.fail_reset {
                return Err(CameraError::I2cError);
            }
            Ok(())
        }

        fn init_status(&mut self, _bus: &mut B) -> Result<(), CameraError> {
            Ok(())
        }

        fn set_pixformat(
            &mut self,
            _bus: &mut B,
            pixformat: PixelFormat,
        ) -> Result<(), CameraError> {
            self.pixformat = pixformat;
            Ok(())
        }

        fn set_framesize(&mut self, _bus: &mut B, framesize: FrameSize) -> Result<(), CameraError> {
            self.status.framesize = framesize;
            Ok(())
        }

        fn set_quality(&mut self, _bus: &mut B, quality: i32) -> Result<(), CameraError> {
            self.status.quality = quality.clamp(0, 63) as u8;
            Ok(())
        }

        fn set_bpc(&mut self, _bus: &mut B, enable: bool) -> Result<(), CameraError> {
            self.status.bpc = enable;
            Ok(())
        }

        fn set_wpc(&mut self, _bus: &mut B, enable: bool) -> Result<(), CameraError> {
            self.status.wpc = enable;
            Ok(())
        }

        fn set_lenc(&mut self, _bus: &mut B, enable: bool) -> Result<(), CameraError> {
            self.status.lenc = enable;
            Ok(())
        }

        fn set_hmirror(&mut self, _bus: &mut B, enable: bool) -> Result<(), CameraError> {
            self.status.hmirror = enable;
            Ok(())
        }

        fn set_vflip(&mut self, _bus: &mut B, enable: bool) -> Result<(), CameraError> {
            if self.fail_vflip {
                return Err(CameraError::I2cError);
            }
            self.status.vflip = enable;
            Ok(())
        }
    }

    #[test]
    fn test_hal_007_load_settings_frame_size_and_error_propagation() {
        let bus: MockSccbBus<512> = MockSccbBus::new();
        let sensor = MockConfigurableSensor::new();
        let cfg = CameraConfig {
            pixel_format: PixelFormat::Jpeg,
            frame_size: FrameSize::Vga,
            ..CameraConfig::default()
        };

        let driver: CameraDriver<WroverKitPins, _, Uninitialized, MockConfigurableSensor, 2, 256> =
            CameraDriver::new(WroverKitPins, bus, sensor, cfg);
        let mut init = driver.init().expect("driver init");
        assert_eq!(init.config().frame_size, FrameSize::Vga);
        let vga_dma = *init.dma_config().expect("vga dma config");

        // Load new settings with Qvga frame size and toggled orientation flags
        let mut saved = init.save_settings();
        assert!(!saved.status.bpc, "OV2640 init disables bpc");
        assert!(saved.status.wpc, "OV2640 init enables wpc");
        assert!(saved.status.lenc, "OV2640 init enables lenc");
        assert!(!saved.status.hmirror, "hmirror defaults to false");
        assert!(!saved.status.vflip, "vflip defaults to false");
        saved.status.framesize = FrameSize::Qvga;
        saved.status.hmirror = true;
        saved.status.vflip = true;
        init.load_settings(&saved).expect("load settings");

        // config.frame_size, DMA sizes, and orientation flags MUST reflect
        // loaded settings.
        assert_eq!(init.config().frame_size, FrameSize::Qvga);
        let qvga_dma = *init.dma_config().expect("qvga dma config");
        assert_ne!(
            qvga_dma, vga_dma,
            "load_settings must recompute DMA sizes on frame size change"
        );
        assert!(
            init.sensor().status().hmirror,
            "hmirror restored from saved"
        );
        assert!(init.sensor().status().vflip, "vflip restored from saved");
    }

    #[test]
    #[cfg(feature = "sensors")]
    fn test_driver_init_sets_pixformat_before_framesize_for_ov2640_jpeg() {
        use esp32_cam_sensors::ov2640::Ov2640;
        let mut bus: MockSccbBus<512> = MockSccbBus::new();
        bus.preset_reg8(CameraSccbAddr::OV2640, 0x0A, 0x26);
        bus.preset_reg8(CameraSccbAddr::OV2640, 0x0B, 0x42);
        bus.preset_reg8(CameraSccbAddr::OV2640, 0x1C, 0x7F);
        bus.preset_reg8(CameraSccbAddr::OV2640, 0x1D, 0xA2);

        let sensor = Ov2640::new();
        let cfg = CameraConfig {
            pixel_format: PixelFormat::Jpeg,
            frame_size: FrameSize::Uxga,
            ..CameraConfig::default()
        };

        let driver: CameraDriver<WroverKitPins, _, Uninitialized, Ov2640, 2, 256> =
            CameraDriver::new(WroverKitPins, bus, sensor, cfg);
        let init_driver = driver.init().expect("driver init");

        // Confirming CLKRC 0x11 is 0x00 and R_DVP_SP 0xD3 is 12 (0x0C)
        // rather than 0x8C
        let bus = init_driver.bus();
        assert_eq!(bus.reg8(CameraSccbAddr::OV2640, 0x11), 0x00);
        assert_eq!(bus.reg8(CameraSccbAddr::OV2640, 0xD3), 12);
    }

    #[test]
    fn test_driver_init_validates_pwdn_and_reset_pins() {
        let mut bus: MockSccbBus<512> = MockSccbBus::new();
        bus.preset_reg8(CameraSccbAddr::OV2640, 0x0A, 0x26);
        bus.preset_reg8(CameraSccbAddr::OV2640, 0x0B, 0x42);
        bus.preset_reg8(CameraSccbAddr::OV2640, 0x1C, 0x7F);
        bus.preset_reg8(CameraSccbAddr::OV2640, 0x1D, 0xA2);

        let sensor =
            HalOv2640Sensor::detect(&mut bus, CameraSccbAddr::OV2640).expect("detect ov2640");
        let cfg = CameraConfig::default();

        let mut bad_pwdn_pins = WroverKitPins::PINS;
        bad_pwdn_pins.pwdn = Some(40);
        let bad_pwdn_driver: CameraDriver<
            crate::board_pins::CameraPins,
            _,
            Uninitialized,
            HalOv2640Sensor,
            2,
            256,
        > = CameraDriver::new(bad_pwdn_pins, bus.clone(), sensor.clone(), cfg);
        assert_eq!(bad_pwdn_driver.init().err(), Some(CameraError::InvalidArg));

        let mut bad_reset_pins = WroverKitPins::PINS;
        bad_reset_pins.reset = Some(40);
        let bad_reset_driver: CameraDriver<
            crate::board_pins::CameraPins,
            _,
            Uninitialized,
            HalOv2640Sensor,
            2,
            256,
        > = CameraDriver::new(bad_reset_pins, bus.clone(), sensor.clone(), cfg);
        assert_eq!(bad_reset_driver.init().err(), Some(CameraError::InvalidArg));

        let mut valid_pins = WroverKitPins::PINS;
        valid_pins.pwdn = Some(32);
        valid_pins.reset = Some(33);
        let valid_driver: CameraDriver<
            crate::board_pins::CameraPins,
            _,
            Uninitialized,
            HalOv2640Sensor,
            2,
            256,
        > = CameraDriver::new(valid_pins, bus, sensor, cfg);
        let init = valid_driver.init().expect("valid driver init");
        assert_eq!(init.pwdn_pin(), Some(32));
        assert_eq!(init.reset_pin(), Some(33));
    }

    #[test]
    fn test_driver_init_delegates_pin_validation_allowing_s3_pins() {
        use crate::board_pins::Esp32S3GoouuuPins;

        let mut bus: MockSccbBus<512> = MockSccbBus::new();
        bus.preset_reg8(CameraSccbAddr::OV2640, 0x0A, 0x26);
        bus.preset_reg8(CameraSccbAddr::OV2640, 0x0B, 0x42);
        bus.preset_reg8(CameraSccbAddr::OV2640, 0x1C, 0x7F);
        bus.preset_reg8(CameraSccbAddr::OV2640, 0x1D, 0xA2);

        let sensor =
            HalOv2640Sensor::detect(&mut bus, CameraSccbAddr::OV2640).expect("detect ov2640");
        let cfg = CameraConfig::default();

        // ESP32-S3 allows GPIO 40..=48 for PWDN and RESET
        let mut s3_pins = Esp32S3GoouuuPins::PINS;
        s3_pins.pwdn = Some(41);
        s3_pins.reset = Some(42);
        let s3_driver: CameraDriver<
            crate::board_pins::CameraPins,
            _,
            Uninitialized,
            HalOv2640Sensor,
            2,
            256,
        > = CameraDriver::new(s3_pins, bus, sensor, cfg);
        let init = s3_driver
            .init()
            .expect("valid s3 driver init with pins >= 40");
        assert_eq!(init.pwdn_pin(), Some(41));
        assert_eq!(init.reset_pin(), Some(42));
    }

    #[test]
    fn test_save_and_load_settings_step_down_qvga_and_restore_hd() {
        let bus: MockSccbBus<512> = MockSccbBus::new();
        let sensor = MockConfigurableSensor::new();
        let cfg = CameraConfig {
            pixel_format: PixelFormat::Jpeg,
            frame_size: FrameSize::Hd,
            ..CameraConfig::default()
        };

        let driver: CameraDriver<WroverKitPins, _, Uninitialized, MockConfigurableSensor, 2, 256> =
            CameraDriver::new(WroverKitPins, bus, sensor, cfg);
        let mut init = driver.init().expect("driver init");
        assert_eq!(init.config().frame_size, FrameSize::Hd);
        assert_eq!(init.max_frame_size(), FrameSize::Hd);

        // Step down to Qvga
        let mut qvga_settings = init.save_settings();
        qvga_settings.status.framesize = FrameSize::Qvga;
        init.load_settings(&qvga_settings)
            .expect("load qvga settings");
        assert_eq!(init.config().frame_size, FrameSize::Qvga);
        // max_frame_size must not shrink
        assert_eq!(init.max_frame_size(), FrameSize::Hd);

        // Restore back to Hd (<= max_frame_size)
        let mut hd_settings = init.save_settings();
        hd_settings.status.framesize = FrameSize::Hd;
        init.load_settings(&hd_settings)
            .expect("restore hd settings");
        assert_eq!(init.config().frame_size, FrameSize::Hd);

        // Frame size exceeding max_frame_size must be rejected with InvalidArg
        let mut uxga_settings = init.save_settings();
        uxga_settings.status.framesize = FrameSize::Uxga;
        assert_eq!(
            init.load_settings(&uxga_settings),
            Err(CameraError::InvalidArg)
        );
    }

    #[test]
    fn test_m115_jpeg_soi_validated_against_written_slice_not_stale_slot() {
        let bus: MockSccbBus<512> = MockSccbBus::new();
        let sensor = MockConfigurableSensor::new();
        let cfg = CameraConfig {
            pixel_format: PixelFormat::Jpeg,
            frame_size: FrameSize::Qvga,
            fb_count: 1,
            psram_dma_mode: true,
            ..CameraConfig::default()
        };

        let driver: CameraDriver<WroverKitPins, _, Uninitialized, MockConfigurableSensor, 1, 32> =
            CameraDriver::new(WroverKitPins, bus, sensor, cfg);
        let mut init = driver.init().expect("driver init");

        // Pre-seed slot 0 with stale valid JPEG bytes to simulate an unzeroed
        // recycled buffer.
        if let Some(slot) = init.pool_mut().raw_buffer_mut(0) {
            slot[..6].copy_from_slice(&[0xFF, 0xD8, 0xFF, 0xE0, 0xFF, 0xD9]);
        }

        // Case 1: Empty first half + non-SOI second half MUST NOT bypass SOI
        // check via stale slot bytes.
        assert_eq!(
            init.capture_frame(&[&[], &[0x00, 0xD9]]).err(),
            Some(CameraError::CorruptedFrame)
        );

        // Case 2: Re-seed stale bytes; a 2-byte frame shorter than SOI (3B)
        // MUST fail rather than reading stale byte 2 (`0xFF`).
        if let Some(slot) = init.pool_mut().raw_buffer_mut(0) {
            slot[..6].copy_from_slice(&[0xFF, 0xD8, 0xFF, 0xE0, 0xFF, 0xD9]);
        }
        assert_eq!(
            init.capture_frame(&[&[0xFF, 0xD8]]).err(),
            Some(CameraError::CorruptedFrame)
        );

        // Case 3: Valid SOI split across half 0 (`[0xFF, 0xD8]`) and half 1
        // (`[0xFF, 0xE0, 0xFF, 0xD9]`) on a clean slot MUST succeed.
        let guard = init
            .capture_frame(&[&[0xFF, 0xD8], &[0xFF, 0xE0, 0xFF, 0xD9]])
            .expect("split SOI across halves must succeed");
        assert_eq!(guard.as_slice(), &[0xFF, 0xD8, 0xFF, 0xE0, 0xFF, 0xD9]);
    }

    #[test]
    fn test_m116_init_error_teardown_and_drop_quiesces_hardware() {
        assert!(core::mem::needs_drop::<
            CameraDriver<
                WroverKitPins,
                MockSccbBus<64>,
                Initialized,
                MockConfigurableSensor,
                2,
                64,
            >,
        >());

        // 1. Failing mid-init (sensor.reset returns I2cError) tears down XCLK
        // and I2S0 cleanly.
        let bus: MockSccbBus<64> = MockSccbBus::new();
        let mut failing_sensor = MockConfigurableSensor::new();
        failing_sensor.fail_reset = true;
        let driver: CameraDriver<WroverKitPins, _, Uninitialized, MockConfigurableSensor, 2, 64> =
            CameraDriver::new(WroverKitPins, bus, failing_sensor, CameraConfig::default());
        assert_eq!(driver.init().err(), Some(CameraError::I2cError));

        // 2. Dropping an active Streaming driver runs Drop teardown cleanly.
        let bus2: MockSccbBus<64> = MockSccbBus::new();
        let sensor2 = MockConfigurableSensor::new();
        let driver2: CameraDriver<WroverKitPins, _, Uninitialized, MockConfigurableSensor, 2, 64> =
            CameraDriver::new(WroverKitPins, bus2, sensor2, CameraConfig::default());
        let init2 = driver2.init().expect("init2");
        assert!(init2.xclk_enabled());
        assert!(init2.i2s_active());
        let streaming2 = init2.start_streaming().expect("start streaming2");
        assert!(streaming2.i2s_running());
        drop(streaming2);

        // 3. Explicit deinit clears hardware active flags so subsequent Drop is
        // a no-op.
        let bus3: MockSccbBus<64> = MockSccbBus::new();
        let sensor3 = MockConfigurableSensor::new();
        let driver3: CameraDriver<WroverKitPins, _, Uninitialized, MockConfigurableSensor, 2, 64> =
            CameraDriver::new(WroverKitPins, bus3, sensor3, CameraConfig::default());
        let init3 = driver3.init().expect("init3");
        let streaming3 = init3.start_streaming().expect("start streaming3");
        let uninit3 = streaming3.deinit().expect("streaming deinit");
        assert!(!uninit3.xclk_enabled());
        assert!(!uninit3.i2s_active());
        assert!(!uninit3.i2s_running());
    }

    #[test]
    fn test_m116_try_transitions_and_into_parts_recover_handles_after_sccb_nack() {
        let mut bus: MockSccbBus<64> = MockSccbBus::new();
        bus.preset_reg8(CameraSccbAddr::OV2640, 0x0A, 0x26);
        let mut sensor = MockConfigurableSensor::new();
        sensor.fail_reset_nack = true;
        let cfg = CameraConfig::default();

        // 1. Transient SccbNack during `try_init` tears down hardware and
        // preserves the driver so Board/Sccb/Sensor can be recovered.
        let driver: CameraDriver<WroverKitPins, _, Uninitialized, MockConfigurableSensor, 2, 64> =
            CameraDriver::new(WroverKitPins, bus, sensor, cfg);
        let (recovered, err) = driver
            .try_init()
            .expect_err("transient SccbNack must fail try_init");
        assert_eq!(
            err,
            CameraError::SccbNack {
                slv_addr: CameraSccbAddr::OV2640,
                phase: esp32_cam_types::SccbPhase::DeviceAddr,
            }
        );
        assert!(!recovered.xclk_enabled(), "XCLK must be torn down on error");
        assert!(!recovered.i2s_active(), "I2S0 must be torn down on error");
        assert!(!recovered.i2s_running(), "I2S0 must not be running");
        assert!(recovered.dma_config().is_none(), "DMA cfg must be cleared");

        // 2. Recover `(Board, Sccb, Sensor)` via `into_parts()`, clear the
        // transient NACK, and re-initialize using the exact same handles.
        let (board, bus_back, mut sensor_back) = recovered.into_parts();
        assert_eq!(
            bus_back.reg8(CameraSccbAddr::OV2640, 0x0A),
            0x26,
            "recovered SCCB bus must preserve prior state"
        );
        sensor_back.fail_reset_nack = false;

        let driver2: CameraDriver<WroverKitPins, _, Uninitialized, MockConfigurableSensor, 2, 64> =
            CameraDriver::new(board, bus_back, sensor_back, cfg);
        let init = driver2
            .try_init()
            .expect("retry try_init succeeds after clearing NACK");
        assert!(init.xclk_enabled());
        assert!(init.i2s_active());

        // 3. `try_start_streaming` failure preserves handles and tears down hw.
        let mut init_broken = init;
        init_broken.inner_mut().stop_failed = true;
        let (recovered_init, start_err) = init_broken
            .try_start_streaming()
            .expect_err("stop_failed must reject try_start_streaming");
        assert_eq!(start_err, CameraError::HardwareError);
        assert!(!recovered_init.xclk_enabled());
        assert!(!recovered_init.i2s_active());
        assert!(recovered_init.dma_config().is_none());

        let (board2, bus2, sensor2) = recovered_init.into_parts();
        let driver3: CameraDriver<WroverKitPins, _, Uninitialized, MockConfigurableSensor, 2, 64> =
            CameraDriver::new(board2, bus2, sensor2, cfg);
        let streaming = driver3
            .try_init()
            .expect("re-init")
            .try_start_streaming()
            .expect("try_start_streaming succeeds");
        assert!(streaming.i2s_running());
        let stopped = streaming
            .try_stop_streaming_recover()
            .expect("try_stop_streaming_recover succeeds");
        assert!(!stopped.i2s_running());
        let uninit = stopped.try_deinit().expect("try_deinit succeeds");
        assert!(!uninit.xclk_enabled());
        let (_board3, bus3, _sensor3) = uninit.into_parts();
        assert_eq!(bus3.reg8(CameraSccbAddr::OV2640, 0x0A), 0x26);
    }

    #[test]
    fn test_m117_load_settings_defers_config_and_ignores_not_supported_overwrite() {
        // 1. Sensor returning NotSupported (HalOv2640Sensor) must NOT overwrite
        // driver.config().frame_size.
        let mut bus: MockSccbBus<512> = MockSccbBus::new();
        bus.preset_reg8(CameraSccbAddr::OV2640, 0x0A, 0x26);
        bus.preset_reg8(CameraSccbAddr::OV2640, 0x0B, 0x42);
        bus.preset_reg8(CameraSccbAddr::OV2640, 0x1C, 0x7F);
        bus.preset_reg8(CameraSccbAddr::OV2640, 0x1D, 0xA2);
        let stub =
            HalOv2640Sensor::detect(&mut bus, CameraSccbAddr::OV2640).expect("detect ov2640");
        let cfg = CameraConfig {
            pixel_format: PixelFormat::Jpeg,
            frame_size: FrameSize::Vga,
            ..CameraConfig::default()
        };
        let driver: CameraDriver<WroverKitPins, _, Uninitialized, HalOv2640Sensor, 2, 256> =
            CameraDriver::new(WroverKitPins, bus, stub, cfg);
        let mut init = driver.init().expect("init stub");
        let mut saved = init.save_settings();
        saved.status.framesize = FrameSize::Qvga;
        init.load_settings(&saved)
            .expect("load_settings tolerates NotSupported");
        assert_eq!(
            init.config().frame_size,
            FrameSize::Vga,
            "config.frame_size must not be overwritten on NotSupported"
        );

        // 2. Mid-way SCCB error in load_settings leaves config & dma_cfg
        // untouched.
        let bus2: MockSccbBus<512> = MockSccbBus::new();
        let sensor2 = MockConfigurableSensor::new();
        let cfg2 = CameraConfig {
            pixel_format: PixelFormat::Rgb565,
            frame_size: FrameSize::Vga,
            ..CameraConfig::default()
        };
        let driver2: CameraDriver<WroverKitPins, _, Uninitialized, MockConfigurableSensor, 2, 256> =
            CameraDriver::new(WroverKitPins, bus2, sensor2, cfg2);
        let mut init2 = driver2.init().expect("init2");
        let orig_dma = *init2.dma_config().expect("orig dma");

        // Access via sensor_mut() and with_sensor_and_bus() works in
        // Initialized state.
        init2.sensor_mut().fail_vflip = true;
        init2.with_sensor_and_bus(|s, _b| assert!(s.fail_vflip));

        let mut new_saved = init2.save_settings();
        new_saved.status.framesize = FrameSize::Qvga;
        new_saved.pixformat = PixelFormat::Grayscale;
        new_saved.status.vflip = true;
        assert_eq!(init2.load_settings(&new_saved), Err(CameraError::I2cError));
        assert_eq!(init2.config().frame_size, FrameSize::Vga);
        assert_eq!(init2.config().pixel_format, PixelFormat::Rgb565);
        assert_eq!(init2.dma_config(), Some(&orig_dma));

        // 3. After clearing fail_vflip, changing pixformat Rgb565 -> Grayscale
        // recomputes DMA config and updates config.
        init2.sensor_mut().fail_vflip = false;
        init2.load_settings(&new_saved).expect("load grayscale");
        assert_eq!(init2.config().frame_size, FrameSize::Qvga);
        assert_eq!(init2.config().pixel_format, PixelFormat::Grayscale);
        assert_ne!(init2.dma_config(), Some(&orig_dma));
    }

    #[test]
    fn test_m198_sub_stride_corrupted_and_deferred_timestamp() {
        let bus: MockSccbBus<256> = MockSccbBus::new();
        let sensor = MockConfigurableSensor::new();
        let cfg = CameraConfig {
            pixel_format: PixelFormat::Jpeg,
            frame_size: FrameSize::Qvga,
            psram_dma_mode: false,
            ..CameraConfig::default()
        };
        let driver: CameraDriver<WroverKitPins, _, Uninitialized, MockConfigurableSensor, 2, 64> =
            CameraDriver::new(WroverKitPins, bus, sensor, cfg);
        let mut init = driver.init().expect("init");

        // 2-byte fragment is shorter than JPEG 4-byte DMA stride: must return
        // CorruptedFrame (not BufferTooSmall) and must NOT advance timestamp.
        assert_eq!(
            init.capture_frame(&[&[0xFF, 0xD8]]).err(),
            Some(CameraError::CorruptedFrame)
        );

        // Next valid frame must get the first timestamp (33_333 us), proving
        // the failed capture did not advance timestamp_counter_us.
        let valid_dma = pack_dma_elements::<32>(&[0xFF, 0xD8, 0xFF, 0xE0, 0x00, 0x10, 0xFF, 0xD9]);
        let frame = init.capture_frame(&[&valid_dma]).expect("valid frame");
        assert_eq!(frame.timestamp_us(), 33_333);
    }

    #[test]
    fn test_m199_fb_count_exceeding_pool_count_rejected_and_try_stop_streaming() {
        let bus: MockSccbBus<256> = MockSccbBus::new();
        let sensor = MockConfigurableSensor::new();
        let bad_cfg = CameraConfig {
            fb_count: 3,
            ..CameraConfig::default()
        };
        let bad_driver: CameraDriver<
            WroverKitPins,
            _,
            Uninitialized,
            MockConfigurableSensor,
            2,
            64,
        > = CameraDriver::new(WroverKitPins, bus.clone(), sensor.clone(), bad_cfg);
        assert_eq!(bad_driver.init().err(), Some(CameraError::InvalidArg));

        let ok_driver: CameraDriver<
            WroverKitPins,
            _,
            Uninitialized,
            MockConfigurableSensor,
            2,
            64,
        > = CameraDriver::new(WroverKitPins, bus, sensor, CameraConfig::default());
        let init = ok_driver.init().expect("init ok");
        let streaming = init.start_streaming().expect("start streaming");
        let stopped = streaming
            .try_stop_streaming()
            .expect("try_stop_streaming succeeds");
        assert!(!stopped.i2s_running());
        let _uninit = stopped.deinit().expect("deinit");
    }
}
