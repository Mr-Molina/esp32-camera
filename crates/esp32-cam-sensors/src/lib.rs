//! Frozen [`CameraSensor`] trait contract, register table primitives ([`Reg8`], [`Reg16`]),
//! and safe `#![no_std]` sensor driver modules for all 18 `esp32-camera` sensors.

#![no_std]
#![forbid(unsafe_code)]

pub use esp32_cam_sccb::{EmbeddedHalSccb, MockSccbBus, SccbBus, SccbTransaction};
pub use esp32_cam_types::{
    esp_camera_sensor_get_info, AspectRatio, CameraConvMode, CameraError, CameraFbLocation,
    CameraGrabMode, CameraModel, CameraPid, CameraSccbAddr, CameraStatus, FrameSize, GainCeiling,
    PixelFormat, RatioSettings, ResolutionInfo, SensorId, SensorInfo, CAMERA_SENSOR_INFO,
    RESOLUTION_TABLE,
};

pub mod bf20a6;
pub mod bf3005;
pub mod gc0308;
pub mod gc032a;
pub mod gc2145;
pub mod hm0360;
pub mod hm1055;
pub mod mega_ccm;
pub mod nt99141;
pub mod omnivision;
pub mod ov2640;
pub mod ov3640;
pub mod ov3660;
pub mod ov5640;
pub mod ov7670;
pub mod ov7725;
pub mod sc030iot;
pub mod sc031gs;
pub mod sc101iot;
pub mod vendors;

/// 8-bit register address and 8-bit value pair for sensor initialization tables.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Reg8 {
    pub reg: u8,
    pub val: u8,
}

impl Reg8 {
    /// Creates a new [`Reg8`] entry.
    #[inline]
    pub const fn new(reg: u8, val: u8) -> Self {
        Self { reg, val }
    }
}

/// 16-bit register address and 8-bit value pair for 16-bit sensor initialization tables.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Reg16 {
    pub reg: u16,
    pub val: u8,
}

impl Reg16 {
    /// Creates a new [`Reg16`] entry.
    #[inline]
    pub const fn new(reg: u16, val: u8) -> Self {
        Self { reg, val }
    }
}

/// Writes a slice of [`Reg8`] register/value pairs to `slv_addr` over `bus`.
pub fn write_regs8<B: SccbBus>(
    bus: &mut B,
    slv_addr: u8,
    regs: &[Reg8],
) -> Result<(), CameraError> {
    for entry in regs {
        bus.write_reg8(slv_addr, entry.reg, entry.val)?;
    }
    Ok(())
}

/// Writes a slice of [`Reg16`] register/value pairs to `slv_addr` over `bus`.
pub fn write_regs16<B: SccbBus>(
    bus: &mut B,
    slv_addr: u8,
    regs: &[Reg16],
) -> Result<(), CameraError> {
    for entry in regs {
        bus.write_reg16(slv_addr, entry.reg, entry.val)?;
    }
    Ok(())
}

/// Unified hardware-agnostic sensor driver trait replacing C `sensor_t` function pointers.
///
/// Required core lifecycle methods (`model`, `id`, `status`, `status_mut`, `reset`,
/// `set_pixformat`, `set_framesize`) must be implemented by every driver. Optional
/// sensor-specific controls default to returning `Err(CameraError::NotSupported)`.
#[allow(clippy::too_many_arguments)]
pub trait CameraSensor<B: SccbBus> {
    /// Returns the [`CameraModel`] enum discriminant for this sensor.
    fn model(&self) -> CameraModel;

    /// Returns a shared reference to the sensor's [`SensorId`].
    fn id(&self) -> &SensorId;

    /// Returns a shared reference to the sensor's current [`CameraStatus`].
    fn status(&self) -> &CameraStatus;

    /// Returns a mutable reference to the sensor's current [`CameraStatus`].
    fn status_mut(&mut self) -> &mut CameraStatus;

    /// Resets the sensor hardware and applies initial register configuration (`sensor_t::reset`).
    fn reset(&mut self, bus: &mut B) -> Result<(), CameraError>;

    /// Reads current sensor registers into [`CameraStatus`] (`sensor_t::init_status`).
    fn init_status(&mut self, _bus: &mut B) -> Result<(), CameraError> {
        Ok(())
    }

    /// Configures the sensor output pixel format (`sensor_t::set_pixformat`).
    fn set_pixformat(&mut self, bus: &mut B, pixformat: PixelFormat) -> Result<(), CameraError>;

    /// Configures the sensor output frame resolution (`sensor_t::set_framesize`).
    fn set_framesize(&mut self, bus: &mut B, framesize: FrameSize) -> Result<(), CameraError>;

    /// Sets image contrast level (`sensor_t::set_contrast`).
    fn set_contrast(&mut self, _bus: &mut B, _level: i32) -> Result<(), CameraError> {
        Err(CameraError::NotSupported)
    }

    /// Sets image brightness level (`sensor_t::set_brightness`).
    fn set_brightness(&mut self, _bus: &mut B, _level: i32) -> Result<(), CameraError> {
        Err(CameraError::NotSupported)
    }

    /// Sets color saturation level (`sensor_t::set_saturation`).
    fn set_saturation(&mut self, _bus: &mut B, _level: i32) -> Result<(), CameraError> {
        Err(CameraError::NotSupported)
    }

    /// Sets edge sharpness level (`sensor_t::set_sharpness`).
    fn set_sharpness(&mut self, _bus: &mut B, _level: i32) -> Result<(), CameraError> {
        Err(CameraError::NotSupported)
    }

    /// Sets denoise strength (`sensor_t::set_denoise`).
    fn set_denoise(&mut self, _bus: &mut B, _level: i32) -> Result<(), CameraError> {
        Err(CameraError::NotSupported)
    }

    /// Sets AGC gain ceiling (`sensor_t::set_gainceiling`).
    fn set_gainceiling(
        &mut self,
        _bus: &mut B,
        _gainceiling: GainCeiling,
    ) -> Result<(), CameraError> {
        Err(CameraError::NotSupported)
    }

    /// Sets JPEG compression quality (`0..=63`, lower is higher quality; `sensor_t::set_quality`).
    fn set_quality(&mut self, _bus: &mut B, _quality: i32) -> Result<(), CameraError> {
        Err(CameraError::NotSupported)
    }

    /// Enables or disables the internal test colorbar pattern (`sensor_t::set_colorbar`).
    fn set_colorbar(&mut self, _bus: &mut B, _enable: bool) -> Result<(), CameraError> {
        Err(CameraError::NotSupported)
    }

    /// Enables or disables automatic white balance (`sensor_t::set_whitebal`).
    fn set_whitebal(&mut self, _bus: &mut B, _enable: bool) -> Result<(), CameraError> {
        Err(CameraError::NotSupported)
    }

    /// Enables or disables automatic gain control (`sensor_t::set_gain_ctrl`).
    fn set_gain_ctrl(&mut self, _bus: &mut B, _enable: bool) -> Result<(), CameraError> {
        Err(CameraError::NotSupported)
    }

    /// Enables or disables automatic exposure control (`sensor_t::set_exposure_ctrl`).
    fn set_exposure_ctrl(&mut self, _bus: &mut B, _enable: bool) -> Result<(), CameraError> {
        Err(CameraError::NotSupported)
    }

    /// Enables or disables horizontal image mirror (`sensor_t::set_hmirror`).
    fn set_hmirror(&mut self, _bus: &mut B, _enable: bool) -> Result<(), CameraError> {
        Err(CameraError::NotSupported)
    }

    /// Enables or disables vertical image flip (`sensor_t::set_vflip`).
    fn set_vflip(&mut self, _bus: &mut B, _enable: bool) -> Result<(), CameraError> {
        Err(CameraError::NotSupported)
    }

    /// Enables or disables DSP night-mode / AEC2 (`sensor_t::set_aec2`).
    fn set_aec2(&mut self, _bus: &mut B, _enable: bool) -> Result<(), CameraError> {
        Err(CameraError::NotSupported)
    }

    /// Enables or disables manual/advanced AWB gain (`sensor_t::set_awb_gain`).
    fn set_awb_gain(&mut self, _bus: &mut B, _enable: bool) -> Result<(), CameraError> {
        Err(CameraError::NotSupported)
    }

    /// Sets manual AGC gain (`sensor_t::set_agc_gain`).
    fn set_agc_gain(&mut self, _bus: &mut B, _gain: i32) -> Result<(), CameraError> {
        Err(CameraError::NotSupported)
    }

    /// Sets manual AEC exposure value (`sensor_t::set_aec_value`).
    fn set_aec_value(&mut self, _bus: &mut B, _gain: i32) -> Result<(), CameraError> {
        Err(CameraError::NotSupported)
    }

    /// Sets special color effect mode (`sensor_t::set_special_effect`).
    fn set_special_effect(&mut self, _bus: &mut B, _effect: i32) -> Result<(), CameraError> {
        Err(CameraError::NotSupported)
    }

    /// Sets white balance preset mode (`sensor_t::set_wb_mode`).
    fn set_wb_mode(&mut self, _bus: &mut B, _mode: i32) -> Result<(), CameraError> {
        Err(CameraError::NotSupported)
    }

    /// Sets auto-exposure target level (`sensor_t::set_ae_level`).
    fn set_ae_level(&mut self, _bus: &mut B, _level: i32) -> Result<(), CameraError> {
        Err(CameraError::NotSupported)
    }

    /// Enables or disables downsize/DCW (`sensor_t::set_dcw`).
    fn set_dcw(&mut self, _bus: &mut B, _enable: bool) -> Result<(), CameraError> {
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

    /// Enables or disables raw gamma correction (`sensor_t::set_raw_gma`).
    fn set_raw_gma(&mut self, _bus: &mut B, _enable: bool) -> Result<(), CameraError> {
        Err(CameraError::NotSupported)
    }

    /// Enables or disables lens shading correction (`sensor_t::set_lenc`).
    fn set_lenc(&mut self, _bus: &mut B, _enable: bool) -> Result<(), CameraError> {
        Err(CameraError::NotSupported)
    }

    /// Reads a sensor register with mask (`sensor_t::get_reg`).
    fn get_reg(&mut self, _bus: &mut B, _reg: i32, _mask: i32) -> Result<i32, CameraError> {
        Err(CameraError::NotSupported)
    }

    /// Writes a sensor register with mask (`sensor_t::set_reg`).
    fn set_reg(
        &mut self,
        _bus: &mut B,
        _reg: i32,
        _mask: i32,
        _value: i32,
    ) -> Result<(), CameraError> {
        Err(CameraError::NotSupported)
    }

    /// Configures raw sensor window and scaling (`sensor_t::set_res_raw`).
    fn set_res_raw(
        &mut self,
        _bus: &mut B,
        _start_x: i32,
        _start_y: i32,
        _end_x: i32,
        _end_y: i32,
        _offset_x: i32,
        _offset_y: i32,
        _total_x: i32,
        _total_y: i32,
        _output_x: i32,
        _output_y: i32,
        _scale: bool,
        _binning: bool,
    ) -> Result<(), CameraError> {
        Err(CameraError::NotSupported)
    }

    /// Configures sensor internal PLL dividers (`sensor_t::set_pll`).
    fn set_pll(
        &mut self,
        _bus: &mut B,
        _bypass: i32,
        _mul: i32,
        _sys: i32,
        _root: i32,
        _pre: i32,
        _seld5: i32,
        _pclken: i32,
        _pclk: i32,
    ) -> Result<(), CameraError> {
        Err(CameraError::NotSupported)
    }

    /// Configures external clock frequency (`sensor_t::set_xclk`).
    fn set_xclk(&mut self, _bus: &mut B, _timer: i32, _xclk: i32) -> Result<(), CameraError> {
        Err(CameraError::NotSupported)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct DummySensor {
        id: SensorId,
        status: CameraStatus,
    }

    impl<B: SccbBus> CameraSensor<B> for DummySensor {
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
            write_regs8(bus, CameraSccbAddr::OV2640, &[Reg8::new(0x12, 0x80)])
        }

        fn set_pixformat(&mut self, _bus: &mut B, _pixformat: PixelFormat) -> Result<(), CameraError> {
            Ok(())
        }

        fn set_framesize(&mut self, _bus: &mut B, framesize: FrameSize) -> Result<(), CameraError> {
            self.status.framesize = framesize;
            Ok(())
        }
    }

    fn run_sensor_checks<B: SccbBus, S: CameraSensor<B>>(sensor: &mut S, bus: &mut B) {
        assert_eq!(sensor.model(), CameraModel::Ov2640);
        assert_eq!(sensor.id().pid, 0x26);
        assert_eq!(sensor.reset(bus), Ok(()));
        assert_eq!(sensor.init_status(bus), Ok(()));
        assert_eq!(sensor.set_pixformat(bus, PixelFormat::Jpeg), Ok(()));
        assert_eq!(sensor.set_framesize(bus, FrameSize::Hd), Ok(()));
        assert_eq!(sensor.status().framesize, FrameSize::Hd);
        sensor.status_mut().quality = 10;
        assert_eq!(sensor.status().quality, 10);
        assert_eq!(sensor.set_sharpness(bus, 1), Err(CameraError::NotSupported));
    }

    #[test]
    fn test_camera_sensor_trait_defaults() {
        let mut bus: MockSccbBus<32> = MockSccbBus::new();
        let mut sensor = DummySensor {
            id: SensorId {
                midh: 0x7F,
                midl: 0xA2,
                pid: CameraPid::OV2640.as_u16(),
                ver: 0x01,
            },
            status: CameraStatus::default(),
        };

        run_sensor_checks(&mut sensor, &mut bus);
        assert_eq!(bus.get_reg8(CameraSccbAddr::OV2640, 0x12), 0x80);
    }
}
