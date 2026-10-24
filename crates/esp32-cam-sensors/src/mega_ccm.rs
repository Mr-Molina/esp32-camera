//! MEGA_CCM 5MP JPEG/RGB565/YUV422 camera module driver.
//!
//! Ported from `sensors/mega_ccm.c` and `sensors/private_include/mega_ccm_regs.h`.
//! Enforces strict 16-bit register address and 8-bit value bounds on register operations.

use crate::{
    CameraError, CameraModel, CameraPid, CameraSccbAddr, CameraSensor, CameraStatus, FrameSize,
    PixelFormat, SccbBus, SensorId,
};

pub const ID_BASE: u16 = 0x0000;
pub const SENSOR_BASE: u16 = 0x0100;
pub const SYS_CLK_BASE: u16 = 0x0200;
pub const BYPASS_BASE: u16 = 0xFFF0;

pub const SENSOR_ID_HIGH: u16 = ID_BASE;
pub const SENSOR_ID_LOW: u16 = ID_BASE | 0x01;
pub const FIRMWARE_VER: u16 = ID_BASE | 0x02;

pub const CAMERA_RST_REG: u16 = SENSOR_BASE | 0x02;

pub const PIXEL_FMT_REG: u16 = SENSOR_BASE | 0x20;
pub const RESOLUTION_REG: u16 = SENSOR_BASE | 0x21;
pub const BRIGHTNESS_REG: u16 = SENSOR_BASE | 0x22;
pub const CONTRAST_REG: u16 = SENSOR_BASE | 0x23;
pub const SATURATION_REG: u16 = SENSOR_BASE | 0x24;
pub const EXP_COMPENSATE_REG: u16 = SENSOR_BASE | 0x25;
pub const AWB_MODE_REG: u16 = SENSOR_BASE | 0x26;
pub const SPECIAL_REG: u16 = SENSOR_BASE | 0x27;
pub const SHARPNESS_REG: u16 = SENSOR_BASE | 0x28;
pub const FOCUS_REG: u16 = SENSOR_BASE | 0x29;
pub const IMAGE_QUALITY_REG: u16 = SENSOR_BASE | 0x2A;
pub const IMAGE_FLIP_REG: u16 = SENSOR_BASE | 0x2B;
pub const IMAGE_MIRROR_REG: u16 = SENSOR_BASE | 0x2C;

pub const AGC_MODE_REG: u16 = SENSOR_BASE | 0x30;
pub const MANUAL_AGC_REG: u16 = SENSOR_BASE | 0x31;
pub const MANUAL_EXP_H_REG: u16 = SENSOR_BASE | 0x33;
pub const MANUAL_EXP_L_REG: u16 = SENSOR_BASE | 0x34;

pub const SYSTEM_CLK_DIV_REG: u16 = SYS_CLK_BASE;
pub const SYSTEM_PLL_DIV_REG: u16 = SYS_CLK_BASE | 0x01;

/// Detects a MEGA_CCM sensor at `slv_addr` (`0x1F`).
pub fn detect<B: SccbBus>(bus: &mut B, slv_addr: u8) -> Result<SensorId, CameraError> {
    MegaCcm::detect(bus, slv_addr)
}

/// Safe `#![no_std]` driver for the Arducam Mega CCM 5MP camera module.
#[derive(Debug, Clone)]
pub struct MegaCcm {
    id: SensorId,
    status: CameraStatus,
    slv_addr: u8,
    pixformat: PixelFormat,
}

#[allow(non_camel_case_types)]
pub type MEGA_CCM = MegaCcm;

impl Default for MegaCcm {
    fn default() -> Self {
        Self::new()
    }
}

impl MegaCcm {
    /// Creates a new [`MegaCcm`] instance with default SCCB address (`0x1F`) and PID (`0x039E`).
    pub const fn new() -> Self {
        Self {
            id: SensorId {
                midh: 0,
                midl: 0,
                pid: CameraPid::MEGA_CCM.as_u16(),
                ver: 0,
            },
            status: CameraStatus {
                framesize: FrameSize::Vga,
                scale: false,
                binning: false,
                quality: 12,
                brightness: 0,
                contrast: 0,
                saturation: 0,
                sharpness: 0,
                denoise: 0,
                special_effect: 0,
                wb_mode: 0,
                awb: 1,
                awb_gain: 1,
                aec: 1,
                aec2: 0,
                ae_level: 0,
                aec_value: 300,
                agc: 1,
                agc_gain: 0,
                gainceiling: 0,
                bpc: 0,
                wpc: 1,
                raw_gma: 1,
                lenc: 1,
                hmirror: 0,
                vflip: 0,
                dcw: 1,
                colorbar: 0,
            },
            slv_addr: CameraSccbAddr::MEGA_CCM,
            pixformat: PixelFormat::Jpeg,
        }
    }

    /// Creates a new [`MegaCcm`] with an explicit slave address and [`SensorId`].
    pub const fn with_addr(slv_addr: u8, id: SensorId) -> Self {
        Self {
            id,
            slv_addr,
            ..Self::new()
        }
    }

    /// Probes the SCCB bus at `slv_addr` (`0x1F`) and verifies `MEGA_CCM_PID` (`0x039E`).
    pub fn detect<B: SccbBus>(bus: &mut B, slv_addr: u8) -> Result<SensorId, CameraError> {
        if slv_addr != CameraSccbAddr::MEGA_CCM {
            return Err(CameraError::NotDetected);
        }
        let h = bus.read_reg16(slv_addr, SENSOR_ID_HIGH)? as u16;
        let l = bus.read_reg16(slv_addr, SENSOR_ID_LOW)? as u16;
        let pid = (h << 8) | l;
        if pid == CameraPid::MEGA_CCM.as_u16() {
            Ok(SensorId {
                midh: 0,
                midl: 0,
                pid,
                ver: 0,
            })
        } else {
            Err(CameraError::NotDetected)
        }
    }

    /// Returns the 7-bit SCCB slave address (`0x1F`).
    pub const fn slv_addr(&self) -> u8 {
        self.slv_addr
    }

    /// Returns the currently configured [`PixelFormat`].
    pub const fn pixformat(&self) -> PixelFormat {
        self.pixformat
    }

    /// Returns the [`CameraModel`] (`CameraModel::MegaCcm`).
    pub const fn model(&self) -> CameraModel {
        CameraModel::MegaCcm
    }

    /// Returns the detected [`SensorId`].
    pub const fn id(&self) -> &SensorId {
        &self.id
    }

    /// Returns the current [`CameraStatus`].
    pub const fn status(&self) -> &CameraStatus {
        &self.status
    }

    /// Returns a mutable reference to [`CameraStatus`].
    pub fn status_mut(&mut self) -> &mut CameraStatus {
        &mut self.status
    }
}

impl<B: SccbBus> CameraSensor<B> for MegaCcm {
    fn model(&self) -> CameraModel {
        CameraModel::MegaCcm
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
        bus.write_reg16(self.slv_addr, CAMERA_RST_REG, 0x00)?;
        bus.write_reg16(self.slv_addr, CAMERA_RST_REG, 0x01)?;
        bus.delay_ms(1000);
        Ok(())
    }

    fn init_status(&mut self, _bus: &mut B) -> Result<(), CameraError> {
        self.status.brightness = 0;
        self.status.contrast = 0;
        self.status.saturation = 0;
        self.status.sharpness = 0;
        self.status.denoise = 0;
        self.status.ae_level = 0;
        self.status.gainceiling = 0;
        self.status.awb = 0;
        self.status.dcw = 0;
        self.status.agc = 0;
        self.status.aec = 0;
        self.status.hmirror = 0;
        self.status.vflip = 0;
        self.status.colorbar = 0;
        self.status.bpc = 0;
        self.status.wpc = 0;
        self.status.raw_gma = 0;
        self.status.lenc = 0;
        self.status.quality = 0;
        self.status.special_effect = 0;
        self.status.wb_mode = 0;
        self.status.awb_gain = 0;
        self.status.agc_gain = 0;
        self.status.aec_value = 0;
        self.status.aec2 = 0;
        Ok(())
    }

    fn set_pixformat(&mut self, bus: &mut B, pixformat: PixelFormat) -> Result<(), CameraError> {
        let code = match pixformat {
            PixelFormat::Jpeg => 0x01,
            PixelFormat::Rgb565 => 0x02,
            PixelFormat::Yuv422 => 0x03,
            _ => return Err(CameraError::NotSupported),
        };
        bus.write_reg16(self.slv_addr, PIXEL_FMT_REG, code)?;
        self.pixformat = pixformat;
        Ok(())
    }

    fn set_framesize(&mut self, bus: &mut B, framesize: FrameSize) -> Result<(), CameraError> {
        let fs = if framesize > FrameSize::Size5Mp {
            FrameSize::Size5Mp
        } else {
            framesize
        };
        match fs {
            FrameSize::Qvga => {
                bus.write_reg16(self.slv_addr, RESOLUTION_REG, 0x01)?;
                bus.write_reg16(self.slv_addr, SYSTEM_CLK_DIV_REG, 0x02)?;
                bus.write_reg16(self.slv_addr, SYSTEM_PLL_DIV_REG, 0x01)?;
            }
            FrameSize::Vga => {
                bus.write_reg16(self.slv_addr, RESOLUTION_REG, 0x02)?;
                bus.write_reg16(self.slv_addr, SYSTEM_CLK_DIV_REG, 0x02)?;
                bus.write_reg16(self.slv_addr, SYSTEM_PLL_DIV_REG, 0x01)?;
            }
            FrameSize::Hd => {
                bus.write_reg16(self.slv_addr, RESOLUTION_REG, 0x03)?;
                bus.write_reg16(self.slv_addr, SYSTEM_CLK_DIV_REG, 0x02)?;
                bus.write_reg16(self.slv_addr, SYSTEM_PLL_DIV_REG, 0x01)?;
            }
            FrameSize::Uxga => {
                bus.write_reg16(self.slv_addr, RESOLUTION_REG, 0x04)?;
                bus.write_reg16(self.slv_addr, SYSTEM_CLK_DIV_REG, 0x02)?;
                bus.write_reg16(self.slv_addr, SYSTEM_PLL_DIV_REG, 0x01)?;
            }
            FrameSize::Fhd => {
                bus.write_reg16(self.slv_addr, RESOLUTION_REG, 0x05)?;
                bus.write_reg16(self.slv_addr, SYSTEM_CLK_DIV_REG, 0x02)?;
                bus.write_reg16(self.slv_addr, SYSTEM_PLL_DIV_REG, 0x01)?;
            }
            FrameSize::Size5Mp => {
                bus.write_reg16(self.slv_addr, RESOLUTION_REG, 0x06)?;
            }
            FrameSize::Size96x96 => {
                bus.write_reg16(self.slv_addr, RESOLUTION_REG, 0x07)?;
            }
            FrameSize::Size128x128 => {
                bus.write_reg16(self.slv_addr, RESOLUTION_REG, 0x08)?;
            }
            FrameSize::Size320x320 => {
                bus.write_reg16(self.slv_addr, RESOLUTION_REG, 0x09)?;
            }
            _ => return Err(CameraError::FailedToSetFrameSize),
        }
        self.status.framesize = fs;
        Ok(())
    }

    fn set_contrast(&mut self, bus: &mut B, level: i32) -> Result<(), CameraError> {
        let clamped = level.clamp(0, 6);
        bus.write_reg16(self.slv_addr, CONTRAST_REG, clamped as u8)?;
        self.status.contrast = clamped as i8;
        Ok(())
    }

    fn set_brightness(&mut self, bus: &mut B, level: i32) -> Result<(), CameraError> {
        let clamped = level.clamp(0, 8);
        bus.write_reg16(self.slv_addr, BRIGHTNESS_REG, clamped as u8)?;
        self.status.brightness = clamped as i8;
        Ok(())
    }

    fn set_saturation(&mut self, bus: &mut B, level: i32) -> Result<(), CameraError> {
        let clamped = level.clamp(0, 6);
        bus.write_reg16(self.slv_addr, SATURATION_REG, clamped as u8)?;
        self.status.saturation = clamped as i8;
        Ok(())
    }

    fn set_quality(&mut self, bus: &mut B, quality: i32) -> Result<(), CameraError> {
        if !(0..=0xFF).contains(&quality) {
            return Err(CameraError::InvalidArg);
        }
        bus.write_reg16(self.slv_addr, IMAGE_QUALITY_REG, quality as u8)?;
        self.status.quality = quality as u8;
        Ok(())
    }

    fn set_hmirror(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        let val = u8::from(enable);
        bus.write_reg16(self.slv_addr, IMAGE_MIRROR_REG, val)?;
        self.status.hmirror = val;
        Ok(())
    }

    fn set_vflip(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        let val = u8::from(enable);
        bus.write_reg16(self.slv_addr, IMAGE_FLIP_REG, val)?;
        self.status.vflip = val;
        Ok(())
    }

    fn set_aec2(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        let val = u8::from(enable);
        bus.write_reg16(self.slv_addr, AGC_MODE_REG, val)?;
        self.status.aec = val;
        Ok(())
    }

    fn set_agc_gain(&mut self, bus: &mut B, gain: i32) -> Result<(), CameraError> {
        if !(0..=0xFF).contains(&gain) {
            return Err(CameraError::InvalidArg);
        }
        bus.write_reg16(self.slv_addr, MANUAL_AGC_REG, gain as u8)?;
        self.status.agc_gain = gain as u8;
        Ok(())
    }

    fn set_aec_value(&mut self, bus: &mut B, gain: i32) -> Result<(), CameraError> {
        if !(0..=0xFFFF).contains(&gain) {
            return Err(CameraError::InvalidArg);
        }
        bus.write_reg16(self.slv_addr, MANUAL_EXP_H_REG, ((gain >> 8) & 0xFF) as u8)?;
        bus.write_reg16(self.slv_addr, MANUAL_EXP_L_REG, (gain & 0xFF) as u8)?;
        self.status.aec_value = gain as u16;
        Ok(())
    }

    fn set_special_effect(&mut self, bus: &mut B, effect: i32) -> Result<(), CameraError> {
        let clamped = effect.clamp(0, 6);
        bus.write_reg16(self.slv_addr, SPECIAL_REG, clamped as u8)?;
        self.status.special_effect = clamped as u8;
        Ok(())
    }

    fn set_wb_mode(&mut self, bus: &mut B, mode: i32) -> Result<(), CameraError> {
        let clamped = mode.clamp(0, 5);
        bus.write_reg16(self.slv_addr, AWB_MODE_REG, clamped as u8)?;
        self.status.wb_mode = clamped as u8;
        Ok(())
    }

    fn get_reg(&mut self, bus: &mut B, reg: i32, mask: i32) -> Result<i32, CameraError> {
        if !(0..=0xFFFF).contains(&reg) || !(0..=0xFF).contains(&mask) {
            return Err(CameraError::InvalidArg);
        }
        let val = bus.read_reg16(self.slv_addr, reg as u16)? as i32;
        Ok(val & mask)
    }

    fn set_reg(
        &mut self,
        bus: &mut B,
        reg: i32,
        mask: i32,
        value: i32,
    ) -> Result<(), CameraError> {
        if !(0..=0xFFFF).contains(&reg) || !(0..=0xFF).contains(&mask) {
            return Err(CameraError::InvalidArg);
        }
        let reg16 = reg as u16;
        let current = bus.read_reg16(self.slv_addr, reg16)? as i32;
        let next = ((current & !mask) | (value & mask)) as u8;
        bus.write_reg16(self.slv_addr, reg16, next)
    }
}
