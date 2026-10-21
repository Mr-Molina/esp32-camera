//! Type-state [`CameraDriver`] (`Uninitialized` -> `Initialized` -> `Streaming`), JPEG SOI/EOI
//! frame boundary verification, DMA half-buffer assembly, autofocus (`esp_camera_af.c`), and
//! NVS configuration persistence (`driver/cam_hal.c`, `driver/esp_camera.c`).

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

use crate::board_pins::BoardPins;
use crate::frame_pool::{FrameGuard, FramePool};

/// JPEG Start-Of-Image (`SOI`) 3-byte marker (`0xFF, 0xD8, 0xFF` in `driver/cam_hal.c:161`).
pub const JPEG_SOI_MARKER: [u8; 3] = [0xFF, 0xD8, 0xFF];

/// JPEG End-Of-Image (`EOI`) 2-byte marker (`0xFF, 0xD9` in `driver/cam_hal.c:163`).
pub const JPEG_EOI_MARKER: [u8; 2] = [0xFF, 0xD9];

/// Default byte count probed at the start of a PSRAM JPEG frame for `SOI` (`CAM_SOI_PROBE_BYTES`).
pub const CAM_SOI_PROBE_BYTES: usize = 32;

/// Computes the scan window for JPEG `EOI` detection in PSRAM (`eoi_probe_window` in `driver/cam_hal.c:167-171`).
#[inline]
pub const fn eoi_probe_window(half_buf_size: usize, frame_len: usize) -> usize {
    let w = half_buf_size.saturating_add(JPEG_EOI_MARKER.len() - 1);
    if w > frame_len {
        frame_len
    } else {
        w
    }
}

/// Searches `inbuf` for the 3-byte JPEG Start-Of-Image marker (`0xFF, 0xD8, 0xFF`), returning
/// its byte offset (`cam_verify_jpeg_soi` in `driver/cam_hal.c:173-192`).
pub fn verify_jpeg_soi(inbuf: &[u8]) -> Option<usize> {
    if inbuf.len() < JPEG_SOI_MARKER.len() {
        return None;
    }
    let limit = inbuf.len() - JPEG_SOI_MARKER.len();
    let mut i = 0;
    while i <= limit {
        if inbuf[i] == JPEG_SOI_MARKER[0]
            && inbuf[i + 1] == JPEG_SOI_MARKER[1]
            && inbuf[i + 2] == JPEG_SOI_MARKER[2]
        {
            return Some(i);
        }
        i += 1;
    }
    None
}

/// Searches `inbuf` for the 2-byte JPEG End-Of-Image marker (`0xFF, 0xD9`), returning its byte
/// offset (`cam_verify_jpeg_eoi` in `driver/cam_hal.c:194-246`).
///
/// - When `search_forward == true` (PSRAM direct-DMA tail probe), returns the earliest `EOI` offset.
/// - When `search_forward == false` (SRAM filtered buffer), scans backward from the end of `inbuf`.
pub fn verify_jpeg_eoi(inbuf: &[u8], search_forward: bool) -> Option<usize> {
    if inbuf.len() < JPEG_EOI_MARKER.len() {
        return None;
    }
    let last_start = inbuf.len() - JPEG_EOI_MARKER.len();
    if search_forward {
        let mut i = 0;
        while i <= last_start {
            if inbuf[i] == JPEG_EOI_MARKER[0] && inbuf[i + 1] == JPEG_EOI_MARKER[1] {
                return Some(i);
            }
            i += 1;
        }
        None
    } else {
        let mut i = last_start;
        loop {
            if inbuf[i] == JPEG_EOI_MARKER[0] && inbuf[i + 1] == JPEG_EOI_MARKER[1] {
                return Some(i);
            }
            if i == 0 {
                break;
            }
            i -= 1;
        }
        None
    }
}

/// Hardware-agnostic sensor interface consumed by [`CameraDriver`].
///
/// Automatically implemented for all `esp32_cam_sensors::CameraSensor<B>` drivers when the
/// `sensors` feature is enabled, and implemented directly by [`HalOv2640Sensor`].
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

    /// Reads current sensor registers into [`CameraStatus`] (`sensor_t::init_status`).
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
    fn set_gainceiling(&mut self, bus: &mut B, gainceiling: GainCeiling) -> Result<(), CameraError> {
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

/// Built-in lightweight OV2640 HAL sensor handle for default [`CameraDriver`] instantiation.
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
    /// Creates a default [`HalOv2640Sensor`] bound to `CameraSccbAddr::OV2640` (`0x30`).
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
}

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

    fn set_pixformat(&mut self, bus: &mut B, pixformat: PixelFormat) -> Result<(), CameraError> {
        match pixformat {
            PixelFormat::Rgb565 | PixelFormat::Yuv422 | PixelFormat::Grayscale | PixelFormat::Jpeg => {
                bus.write_reg8(self.slv_addr, 0xFF, 0x00)?;
                self.pixformat = pixformat;
                Ok(())
            }
            _ => Err(CameraError::FailedToSetOutFormat),
        }
    }

    fn set_framesize(&mut self, bus: &mut B, framesize: FrameSize) -> Result<(), CameraError> {
        if (framesize as u8) > (FrameSize::Uxga as u8) {
            return Err(CameraError::FailedToSetFrameSize);
        }
        bus.write_reg8(self.slv_addr, 0xFF, 0x00)?;
        self.status.framesize = framesize;
        Ok(())
    }

    fn set_gainceiling(&mut self, bus: &mut B, gainceiling: GainCeiling) -> Result<(), CameraError> {
        bus.write_reg8(self.slv_addr, 0xFF, 0x01)?;
        self.status.gainceiling = gainceiling as u8;
        Ok(())
    }

    fn set_quality(&mut self, bus: &mut B, quality: i32) -> Result<(), CameraError> {
        let q = quality.clamp(0, 63) as u8;
        bus.write_reg8(self.slv_addr, 0xFF, 0x00)?;
        bus.write_reg8(self.slv_addr, 0x44, q)?;
        self.status.quality = q;
        Ok(())
    }

    fn set_bpc(&mut self, _bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.status.bpc = u8::from(enable);
        Ok(())
    }

    fn set_wpc(&mut self, _bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.status.wpc = u8::from(enable);
        Ok(())
    }

    fn set_lenc(&mut self, _bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.status.lenc = u8::from(enable);
        Ok(())
    }

    fn set_hmirror(&mut self, _bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.status.hmirror = u8::from(enable);
        Ok(())
    }

    fn set_vflip(&mut self, _bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.status.vflip = u8::from(enable);
        Ok(())
    }
}

/// Type-state marker: camera hardware and sensor have not yet been initialized.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Uninitialized;

/// Type-state marker: camera pins, XCLK, I2S0/LCD_CAM, and sensor registers are initialized.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Initialized;

/// Type-state marker: VSYNC interrupt and DMA frame reception are actively streaming.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Streaming;

/// High-level camera configuration (`camera_config_t` from `driver/include/esp_camera.h`).
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

/// Persisted camera state record (`esp_camera_save_to_nvs` / `esp_camera_load_from_nvs` in `driver/esp_camera.c`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SavedCameraSettings {
    pub status: CameraStatus,
    pub pixformat: PixelFormat,
}

/// Autofocus operating mode (`esp_camera_af_mode_t` in `driver/include/esp_camera_af.h`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Hash)]
pub enum AfMode {
    #[default]
    Auto = 0,
    Single = 1,
    Continuous = 2,
    Manual = 3,
}

/// Autofocus configuration (`esp_camera_af_config_t` in `driver/esp_camera_af.c`).
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

/// Type-state camera driver (`CameraDriver<Board, Sccb, State, Sensor, COUNT, BUF_CAP>`),
/// replacing `driver/esp_camera.c` and `driver/cam_hal.c`.
#[derive(Debug)]
pub struct CameraDriver<
    Board: BoardPins,
    Sccb: SccbBus,
    State = Uninitialized,
    Sensor: HalCameraSensor<Sccb> = HalOv2640Sensor,
    const COUNT: usize = 2,
    const BUF_CAP: usize = 8192,
> {
    board: Board,
    bus: Sccb,
    sensor: Sensor,
    config: CameraConfig,
    xclk: XclkController,
    i2s: Esp32I2sCamera,
    dma_cfg: Option<DmaSizeConfig>,
    pool: FramePool<COUNT, BUF_CAP>,
    timestamp_counter_us: u64,
    _state: PhantomData<State>,
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
            board,
            bus,
            sensor,
            config,
            xclk: XclkController::new(),
            i2s: Esp32I2sCamera::new(),
            dma_cfg: None,
            pool: FramePool::new(grab_mode),
            timestamp_counter_us: 0,
            _state: PhantomData,
        }
    }

    /// Initializes the board pins, `XCLK` generator, `I2S0` peripheral, DMA sizing, and sensor
    /// registers, transitioning the driver from [`Uninitialized`] to [`Initialized`]
    /// (`esp_camera_init` in `driver/esp_camera.c:305-393`).
    pub fn init(
        mut self,
    ) -> Result<CameraDriver<Board, Sccb, Initialized, Sensor, COUNT, BUF_CAP>, CameraError> {
        if self.config.fb_count == 0 || COUNT == 0 || BUF_CAP == 0 {
            return Err(CameraError::InvalidArg);
        }

        // 1. Validate board pinout (FW-001 / DMA-102)
        self.board.validate_pins()?;
        let pins = self.board.pins();

        // 2. Enable XCLK output
        let xclk_cfg = XclkConfig {
            pin_xclk: pins.xclk,
            ledc_timer: self.config.ledc_timer,
            ledc_channel: self.config.ledc_channel,
            xclk_freq_hz: self.config.xclk_freq_hz,
        };
        self.xclk.enable_out_clock(&xclk_cfg)?;

        // 3. Configure low-level I2S0 pins and registers
        let ll_pins = pins.to_ll_pin_config(true);
        if pins.chip == crate::board_pins::TargetChip::Esp32 {
            self.i2s.set_pin(&ll_pins)?;
        }
        self.i2s.config()?;

        // 4. Check sensor capabilities against requested format and resolution (`esp_camera.c:337-346`)
        let model_info = self.sensor.model().info();
        if self.config.pixel_format == PixelFormat::Jpeg && !model_info.support_jpeg {
            self.xclk.disable_out_clock();
            return Err(CameraError::NotSupported);
        }

        let mut frame_size = self.config.frame_size;
        if (frame_size as u8) > (model_info.max_size as u8) {
            frame_size = model_info.max_size;
            self.config.frame_size = frame_size;
        }

        // 5. Configure sampling mode and DMA sizes (`cam_config` in `driver/cam_hal.c:595-666`)
        let sensor_pid = self.sensor.id().pid;
        let sample_cfg = self.i2s.set_sample_mode(
            self.config.pixel_format,
            self.config.xclk_freq_hz,
            sensor_pid,
        )?;
        let res = frame_size.resolution();
        let dma_cfg = calc_dma_sizes(
            self.config.pixel_format == PixelFormat::Jpeg,
            sample_cfg.sampling_mode,
            res.width,
            res.height,
            sample_cfg.in_bytes_per_pixel,
            DEFAULT_CAMERA_DMA_BUFFER_SIZE_MAX,
        )?;
        self.dma_cfg = Some(dma_cfg);

        // 6. Reset and configure sensor over SCCB (`esp_camera.c:282, 358-385`)
        self.sensor.reset(&mut self.bus)?;
        self.sensor.set_framesize(&mut self.bus, frame_size)?;
        self.sensor
            .set_pixformat(&mut self.bus, self.config.pixel_format)?;

        if sensor_pid == CameraPid::OV2640.as_u16() {
            let _ = self
                .sensor
                .set_gainceiling(&mut self.bus, GainCeiling::Gain2x);
            let _ = self.sensor.set_bpc(&mut self.bus, false);
            let _ = self.sensor.set_wpc(&mut self.bus, true);
            let _ = self.sensor.set_lenc(&mut self.bus, true);
        }

        if self.config.pixel_format == PixelFormat::Jpeg {
            let _ = self
                .sensor
                .set_quality(&mut self.bus, self.config.jpeg_quality as i32);
        }
        let _ = self.sensor.init_status(&mut self.bus);

        // Enable VSYNC interrupt (`cam_start()` at end of `esp_camera_init`)
        self.i2s.vsync_intr_enable(true);

        Ok(CameraDriver {
            board: self.board,
            bus: self.bus,
            sensor: self.sensor,
            config: self.config,
            xclk: self.xclk,
            i2s: self.i2s,
            dma_cfg: self.dma_cfg,
            pool: self.pool,
            timestamp_counter_us: self.timestamp_counter_us,
            _state: PhantomData,
        })
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
        &self.sensor
    }

    /// Returns a mutable reference to the underlying camera sensor (`esp_camera_sensor_get`).
    #[inline]
    pub fn sensor_mut(&mut self) -> &mut Sensor {
        &mut self.sensor
    }

    /// Returns a shared reference to the SCCB bus.
    #[inline]
    pub const fn bus(&self) -> &Sccb {
        &self.bus
    }

    /// Returns a mutable reference to the SCCB bus.
    #[inline]
    pub fn bus_mut(&mut self) -> &mut Sccb {
        &mut self.bus
    }

    /// Allows mutating sensor settings with simultaneous mutable access to both `Sensor` and `Sccb`.
    #[inline]
    pub fn with_sensor_and_bus<F, R>(&mut self, f: F) -> R
    where
        F: FnOnce(&mut Sensor, &mut Sccb) -> R,
    {
        f(&mut self.sensor, &mut self.bus)
    }

    /// Returns a shared reference to the active [`CameraConfig`].
    #[inline]
    pub const fn config(&self) -> &CameraConfig {
        &self.config
    }

    /// Returns the computed [`DmaSizeConfig`], if initialized.
    #[inline]
    pub const fn dma_config(&self) -> Option<&DmaSizeConfig> {
        self.dma_cfg.as_ref()
    }

    /// Returns a shared reference to the [`FramePool`].
    #[inline]
    pub const fn pool(&self) -> &FramePool<COUNT, BUF_CAP> {
        &self.pool
    }

    /// Returns a mutable reference to the [`FramePool`].
    #[inline]
    pub fn pool_mut(&mut self) -> &mut FramePool<COUNT, BUF_CAP> {
        &mut self.pool
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
    /// Transitions the driver from [`Initialized`] into the [`Streaming`] type-state (`cam_start`).
    pub fn start_streaming(
        mut self,
    ) -> Result<CameraDriver<Board, Sccb, Streaming, Sensor, COUNT, BUF_CAP>, CameraError> {
        let dma_cfg = self.dma_cfg.ok_or(CameraError::InvalidState)?;
        self.i2s.vsync_intr_enable(true);
        if !self.i2s.start(&dma_cfg, 0) {
            return Err(CameraError::HardwareError);
        }
        Ok(CameraDriver {
            board: self.board,
            bus: self.bus,
            sensor: self.sensor,
            config: self.config,
            xclk: self.xclk,
            i2s: self.i2s,
            dma_cfg: self.dma_cfg,
            pool: self.pool,
            timestamp_counter_us: self.timestamp_counter_us,
            _state: PhantomData,
        })
    }

    /// Assembles a frame from DMA half-buffers, performs JPEG `SOI`/`EOI` verification and trimming,
    /// stores it in [`FramePool`], and returns an RAII [`FrameGuard`].
    pub fn capture_frame(
        &mut self,
        dma_half_buffers: &[&[u8]],
    ) -> Result<FrameGuard<'_>, CameraError> {
        assemble_and_lease_frame(
            &mut self.i2s,
            &self.dma_cfg,
            &self.config,
            &self.sensor,
            &mut self.pool,
            &mut self.timestamp_counter_us,
            dma_half_buffers,
        )
    }

    /// Exports the current sensor status and pixel format for NVS persistence (`esp_camera_save_to_nvs`).
    pub fn save_settings(&self) -> SavedCameraSettings {
        SavedCameraSettings {
            status: *self.sensor.status(),
            pixformat: self.config.pixel_format,
        }
    }

    /// Restores sensor settings from a [`SavedCameraSettings`] snapshot, enforcing the DMA overflow
    /// and pixel format compatibility guards from `driver/esp_camera.c:514-548`.
    pub fn load_settings(&mut self, saved: &SavedCameraSettings) -> Result<(), CameraError> {
        let configured_max_fs = self.sensor.status().framesize;
        if (saved.status.framesize as u8) <= (configured_max_fs as u8) {
            self.sensor
                .set_framesize(&mut self.bus, saved.status.framesize)?;
        }
        let cur_jpeg = self.config.pixel_format == PixelFormat::Jpeg;
        let new_jpeg = saved.pixformat == PixelFormat::Jpeg;
        if cur_jpeg == new_jpeg {
            self.sensor.set_pixformat(&mut self.bus, saved.pixformat)?;
            self.config.pixel_format = saved.pixformat;
        }
        let _ = self
            .sensor
            .set_quality(&mut self.bus, saved.status.quality as i32);
        let _ = self
            .sensor
            .set_hmirror(&mut self.bus, saved.status.hmirror != 0);
        let _ = self
            .sensor
            .set_vflip(&mut self.bus, saved.status.vflip != 0);
        Ok(())
    }

    /// Stops DMA capture, disables XCLK, and returns the driver to the [`Uninitialized`] type-state
    /// (`esp_camera_deinit`).
    pub fn deinit(
        mut self,
    ) -> Result<CameraDriver<Board, Sccb, Uninitialized, Sensor, COUNT, BUF_CAP>, CameraError> {
        self.i2s.deinit()?;
        self.xclk.disable_out_clock();
        self.pool.return_all();
        Ok(CameraDriver {
            board: self.board,
            bus: self.bus,
            sensor: self.sensor,
            config: self.config,
            xclk: self.xclk,
            i2s: self.i2s,
            dma_cfg: None,
            pool: self.pool,
            timestamp_counter_us: self.timestamp_counter_us,
            _state: PhantomData,
        })
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
    /// Assembles a frame from DMA half-buffers while in [`Streaming`] state, performs JPEG
    /// `SOI`/`EOI` verification and trimming, and returns an RAII [`FrameGuard`]
    /// (`esp_camera_fb_get` / `cam_take` in `driver/cam_hal.c:320-458, 730-837`).
    pub fn capture_frame(
        &mut self,
        dma_half_buffers: &[&[u8]],
    ) -> Result<FrameGuard<'_>, CameraError> {
        assemble_and_lease_frame(
            &mut self.i2s,
            &self.dma_cfg,
            &self.config,
            &self.sensor,
            &mut self.pool,
            &mut self.timestamp_counter_us,
            dma_half_buffers,
        )
    }

    /// Stops active DMA streaming and transitions back to [`Initialized`] (`cam_stop`).
    pub fn stop_streaming(
        mut self,
    ) -> CameraDriver<Board, Sccb, Initialized, Sensor, COUNT, BUF_CAP> {
        self.i2s.vsync_intr_enable(false);
        let _ = self.i2s.stop();
        CameraDriver {
            board: self.board,
            bus: self.bus,
            sensor: self.sensor,
            config: self.config,
            xclk: self.xclk,
            i2s: self.i2s,
            dma_cfg: self.dma_cfg,
            pool: self.pool,
            timestamp_counter_us: self.timestamp_counter_us,
            _state: PhantomData,
        }
    }

    /// Stops DMA capture, disables XCLK, and returns the driver to the [`Uninitialized`] type-state
    /// (`esp_camera_deinit`).
    pub fn deinit(
        self,
    ) -> Result<CameraDriver<Board, Sccb, Uninitialized, Sensor, COUNT, BUF_CAP>, CameraError> {
        self.stop_streaming().deinit()
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
    if !i2s.start(&dma_cfg, 0) {
        return Err(CameraError::HardwareError);
    }

    let fs = sensor.status().framesize;
    let res = fs.resolution();
    let format = config.pixel_format;
    let is_jpeg = format == PixelFormat::Jpeg;
    let psram_mode = config.psram_dma_mode;

    *timestamp_counter_us = timestamp_counter_us.wrapping_add(33_333);
    let ts = *timestamp_counter_us;

    let filter = i2s.filter();

    let result = pool.write_and_lease(res.width, res.height, format, ts, |fb_buf| {
        let mut total_written = 0usize;

        for (cnt, half_buf) in dma_half_buffers.iter().enumerate() {
            let written = if psram_mode {
                if total_written + half_buf.len() > fb_buf.len() {
                    return Err(CameraError::BufferTooSmall);
                }
                fb_buf[total_written..total_written + half_buf.len()].copy_from_slice(half_buf);
                half_buf.len()
            } else {
                let rem_cap = fb_buf.len().saturating_sub(total_written);
                let unpacked = filter.apply(&mut fb_buf[total_written..], half_buf);
                if unpacked == 0 && !half_buf.is_empty() && rem_cap == 0 {
                    return Err(CameraError::BufferTooSmall);
                }
                unpacked
            };

            total_written += written;

            // Check for JPEG SOI at offset 0 of the first half-buffer (`cam_hal.c:350-393`)
            if is_jpeg && cnt == 0 {
                let probe_len = if psram_mode {
                    total_written.min(CAM_SOI_PROBE_BYTES)
                } else {
                    total_written
                };
                match verify_jpeg_soi(&fb_buf[..probe_len]) {
                    Some(0) => {}
                    _ => return Err(CameraError::CorruptedFrame),
                }
            }
        }

        if is_jpeg {
            // Locate JPEG EOI marker (`0xFF, 0xD9`) and trim trailing padding bytes (`cam_hal.c:787-817`)
            let offset_e = if psram_mode {
                let last_half_len = dma_half_buffers
                    .last()
                    .map(|b| b.len())
                    .unwrap_or(dma_cfg.dma_half_buffer_size);
                let probe_len = eoi_probe_window(last_half_len, total_written);
                let probe_start = total_written - probe_len;
                verify_jpeg_eoi(&fb_buf[probe_start..total_written], true)
                    .map(|off| probe_start + off)
            } else {
                verify_jpeg_eoi(&fb_buf[..total_written], false)
            };

            match offset_e {
                Some(eoi_pos) => Ok(eoi_pos + JPEG_EOI_MARKER.len()),
                None => Err(CameraError::CorruptedFrame),
            }
        } else {
            Ok(total_written)
        }
    });

    let _ = i2s.stop();
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
        let valid_jpeg = [0xFF, 0xD8, 0xFF, 0xE0, 0x00, 0x10, 0x11, 0x22, 0xFF, 0xD9, 0x00, 0x00];
        assert_eq!(verify_jpeg_soi(&valid_jpeg), Some(0));
        assert_eq!(verify_jpeg_eoi(&valid_jpeg, false), Some(8));
        assert_eq!(verify_jpeg_eoi(&valid_jpeg, true), Some(8));

        // Forward vs backward EOI search when two EOI markers exist (stale tail in PSRAM)
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

        // 2. Capture valid JPEG across 2 DMA half-buffers with trailing garbage after EOI
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
}
