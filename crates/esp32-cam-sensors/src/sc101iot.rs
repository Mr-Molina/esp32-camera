//! SC101IOT 720p HD YUV422 CMOS image sensor driver.
//!
//! Ported from `sensors/sc101iot.c` and `sensors/private_include/sc101iot_settings.h`.

use crate::{
    write_regs8, CameraError, CameraModel, CameraPid, CameraSccbAddr, CameraSensor, CameraStatus,
    FrameSize, PixelFormat, Reg8, SccbBus, SensorId,
};

pub const SC101_SENSOR_ID_HIGH_REG: u8 = 0xF7;
pub const SC101_SENSOR_ID_LOW_REG: u8 = 0xF8;
pub const SC101_PAGE_SELECT_REG: u8 = 0xF0;
pub const SC101_MAX_FRAME_WIDTH: u16 = 1280;
pub const SC101_MAX_FRAME_HEIGHT: u16 = 720;

pub const SC101IOT_DEFAULT_INIT_REGS: &[Reg8] = &[
    Reg8::new(0xF0, 0x30),
    Reg8::new(0x01, 0xFF),
    Reg8::new(0x02, 0xE0),
    Reg8::new(0x30, 0x10),
    Reg8::new(0x3F, 0x81),
    Reg8::new(0xF0, 0x00),
    Reg8::new(0x70, 0x6B),
    Reg8::new(0x72, 0x30),
    Reg8::new(0x84, 0xB4),
    Reg8::new(0x8B, 0x00),
    Reg8::new(0x8C, 0x20),
    Reg8::new(0x8D, 0x02),
    Reg8::new(0x8E, 0xEC),
    Reg8::new(0x9E, 0x10),
    Reg8::new(0xB0, 0xC1),
    Reg8::new(0xC8, 0x10),
    Reg8::new(0xC9, 0x10),
    Reg8::new(0xC6, 0x00),
    Reg8::new(0xE0, 0x0F),
    Reg8::new(0xB5, 0xF0),
    Reg8::new(0xDE, 0x80),
    Reg8::new(0xB5, 0xF0),
    Reg8::new(0xDE, 0x80),
    Reg8::new(0xB2, 0x50),
    Reg8::new(0xB3, 0xFC),
    Reg8::new(0xB4, 0x40),
    Reg8::new(0xB5, 0xC0),
    Reg8::new(0xB6, 0x50),
    Reg8::new(0xB7, 0xFC),
    Reg8::new(0xB8, 0x40),
    Reg8::new(0xB9, 0xC0),
    Reg8::new(0xBA, 0xFF),
    Reg8::new(0xBB, 0xCC),
    Reg8::new(0xBC, 0xA9),
    Reg8::new(0xBD, 0x7D),
    Reg8::new(0xC1, 0x77),
    Reg8::new(0xF0, 0x01),
    Reg8::new(0x70, 0x02),
    Reg8::new(0x71, 0x02),
    Reg8::new(0x72, 0x50),
    Reg8::new(0x73, 0x02),
    Reg8::new(0x74, 0xD2),
    Reg8::new(0x75, 0x20),
    Reg8::new(0x76, 0x81),
    Reg8::new(0x77, 0x8C),
    Reg8::new(0x78, 0x81),
    Reg8::new(0xF4, 0x01),
    Reg8::new(0xF5, 0x00),
    Reg8::new(0xF6, 0x00),
    Reg8::new(0xF0, 0x36),
    Reg8::new(0x40, 0x03),
    Reg8::new(0x41, 0x01),
    Reg8::new(0xF0, 0x39),
    Reg8::new(0x02, 0x70),
    Reg8::new(0xF0, 0x32),
    Reg8::new(0x41, 0x00),
    Reg8::new(0x43, 0x01),
    Reg8::new(0x48, 0x02),
    Reg8::new(0xF0, 0x45),
    Reg8::new(0x09, 0x20),
    Reg8::new(0xF0, 0x33),
    Reg8::new(0x33, 0x10),
    Reg8::new(0xF0, 0x30),
    Reg8::new(0x38, 0x44),
    Reg8::new(0xF0, 0x39),
    Reg8::new(0x07, 0x00),
    Reg8::new(0x08, 0x19),
    Reg8::new(0x47, 0x00),
    Reg8::new(0x48, 0x00),
    Reg8::new(0xF0, 0x37),
    Reg8::new(0x24, 0x31),
    Reg8::new(0xF0, 0x34),
    Reg8::new(0x9F, 0x02),
    Reg8::new(0xA6, 0x51),
    Reg8::new(0xA7, 0x57),
    Reg8::new(0xE8, 0x5F),
    Reg8::new(0xA8, 0x50),
    Reg8::new(0xA9, 0x50),
    Reg8::new(0xE9, 0x50),
    Reg8::new(0xF0, 0x33),
    Reg8::new(0xB3, 0x58),
    Reg8::new(0xB2, 0x78),
    Reg8::new(0xF0, 0x34),
    Reg8::new(0x9F, 0x03),
    Reg8::new(0xA6, 0x51),
    Reg8::new(0xA7, 0x57),
    Reg8::new(0xAA, 0x01),
    Reg8::new(0xAB, 0x28),
    Reg8::new(0xAC, 0x01),
    Reg8::new(0xAD, 0x38),
    Reg8::new(0xF0, 0x33),
    Reg8::new(0x0A, 0x01),
    Reg8::new(0x0B, 0x28),
    Reg8::new(0xF0, 0x33),
    Reg8::new(0x64, 0x0F),
    Reg8::new(0xEC, 0x51),
    Reg8::new(0xED, 0x57),
    Reg8::new(0x06, 0x58),
    Reg8::new(0xE9, 0x58),
    Reg8::new(0xEB, 0x68),
    Reg8::new(0xF0, 0x33),
    Reg8::new(0x64, 0x0F),
    Reg8::new(0xF0, 0x36),
    Reg8::new(0x70, 0xDF),
    Reg8::new(0xB6, 0x40),
    Reg8::new(0xB7, 0x51),
    Reg8::new(0xB8, 0x53),
    Reg8::new(0xB9, 0x57),
    Reg8::new(0xBA, 0x5F),
    Reg8::new(0xB0, 0x84),
    Reg8::new(0xB1, 0x82),
    Reg8::new(0xB2, 0x84),
    Reg8::new(0xB3, 0x88),
    Reg8::new(0xB4, 0x90),
    Reg8::new(0xB5, 0x90),
    Reg8::new(0xF0, 0x36),
    Reg8::new(0x7E, 0x50),
    Reg8::new(0x7F, 0x51),
    Reg8::new(0x77, 0x81),
    Reg8::new(0x78, 0x86),
    Reg8::new(0x79, 0x89),
    Reg8::new(0xF0, 0x36),
    Reg8::new(0x70, 0xDF),
    Reg8::new(0x9C, 0x51),
    Reg8::new(0x9D, 0x57),
    Reg8::new(0x90, 0x54),
    Reg8::new(0x91, 0x54),
    Reg8::new(0x92, 0x56),
    Reg8::new(0xF0, 0x36),
    Reg8::new(0xA0, 0x51),
    Reg8::new(0xA1, 0x57),
    Reg8::new(0x96, 0x33),
    Reg8::new(0x97, 0x43),
    Reg8::new(0x98, 0x43),
    Reg8::new(0xF0, 0x36),
    Reg8::new(0x70, 0xDF),
    Reg8::new(0x7C, 0x40),
    Reg8::new(0x7D, 0x53),
    Reg8::new(0x74, 0xD0),
    Reg8::new(0x75, 0xF0),
    Reg8::new(0x76, 0xF0),
    Reg8::new(0xF0, 0x37),
    Reg8::new(0x0F, 0xD5),
    Reg8::new(0x7A, 0x40),
    Reg8::new(0x7B, 0x57),
    Reg8::new(0x71, 0x09),
    Reg8::new(0x72, 0x09),
    Reg8::new(0x73, 0x05),
    Reg8::new(0xF0, 0x33),
    Reg8::new(0x01, 0x44),
    Reg8::new(0xF0, 0x36),
    Reg8::new(0x37, 0xFB),
    Reg8::new(0xF0, 0x36),
    Reg8::new(0x3C, 0x0D),
    Reg8::new(0xF0, 0x33),
    Reg8::new(0x14, 0x95),
    Reg8::new(0xF0, 0x33),
    Reg8::new(0x8F, 0x80),
    Reg8::new(0xF0, 0x37),
    Reg8::new(0x27, 0x14),
    Reg8::new(0x28, 0x03),
    Reg8::new(0xF0, 0x36),
    Reg8::new(0x37, 0xF4),
    Reg8::new(0xF0, 0x33),
    Reg8::new(0x01, 0x44),
    Reg8::new(0xF0, 0x36),
    Reg8::new(0x79, 0x89),
    Reg8::new(0xF0, 0x34),
    Reg8::new(0xAC, 0x01),
    Reg8::new(0xAD, 0x40),
    Reg8::new(0xF0, 0x33),
    Reg8::new(0xEB, 0x70),
    Reg8::new(0xF0, 0x34),
    Reg8::new(0xA8, 0x50),
    Reg8::new(0xA9, 0x50),
    Reg8::new(0xF0, 0x33),
    Reg8::new(0xB3, 0x58),
    Reg8::new(0xF0, 0x36),
    Reg8::new(0x11, 0x80),
    Reg8::new(0xF0, 0x36),
    Reg8::new(0x41, 0x51),
    Reg8::new(0xF0, 0x3F),
    Reg8::new(0x03, 0x09),
    Reg8::new(0xF0, 0x32),
    Reg8::new(0x0C, 0x06),
    Reg8::new(0x0D, 0x82),
    Reg8::new(0x0E, 0x02),
    Reg8::new(0x0F, 0xEE),
    Reg8::new(0xF0, 0x36),
    Reg8::new(0xEA, 0x09),
    Reg8::new(0xEB, 0xF5),
    Reg8::new(0xEC, 0x11),
    Reg8::new(0xED, 0x27),
    Reg8::new(0xE9, 0x20),
];

/// Detects an SC101IOT sensor at `slv_addr` (`0x68`).
pub fn detect<B: SccbBus>(bus: &mut B, slv_addr: u8) -> Result<SensorId, CameraError> {
    Sc101Iot::detect(bus, slv_addr)
}

/// Safe `#![no_std]` driver for the SmartSens SC101IOT 720p HD camera sensor.
#[derive(Debug, Clone)]
pub struct Sc101Iot {
    id: SensorId,
    status: CameraStatus,
    slv_addr: u8,
    pixformat: PixelFormat,
    xclk_freq_hz: u32,
}

pub type SC101IOT = Sc101Iot;
pub type Sc101iot = Sc101Iot;

impl Default for Sc101Iot {
    fn default() -> Self {
        Self::new()
    }
}

impl Sc101Iot {
    /// Creates a new [`Sc101Iot`] instance with default SCCB address (`0x68`) and PID (`0xDA4A`).
    pub const fn new() -> Self {
        Self {
            id: SensorId {
                midh: 0,
                midl: 0,
                pid: CameraPid::SC101IOT.as_u16(),
                ver: 0,
            },
            status: CameraStatus {
                framesize: FrameSize::Hd,
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
            slv_addr: CameraSccbAddr::SC101IOT,
            pixformat: PixelFormat::Yuv422,
            xclk_freq_hz: 20_000_000,
        }
    }

    /// Creates a new [`Sc101Iot`] with an explicit slave address and [`SensorId`].
    pub const fn with_addr(slv_addr: u8, id: SensorId) -> Self {
        Self {
            id,
            slv_addr,
            ..Self::new()
        }
    }

    /// Probes the SCCB bus at `slv_addr` (`0x68`) and verifies `SC101IOT_PID` (`0xDA4A`).
    pub fn detect<B: SccbBus>(bus: &mut B, slv_addr: u8) -> Result<SensorId, CameraError> {
        if slv_addr != CameraSccbAddr::SC101IOT {
            return Err(CameraError::NotDetected);
        }
        let midl = bus.read_reg8(slv_addr, SC101_SENSOR_ID_LOW_REG)? as u16;
        let midh = bus.read_reg8(slv_addr, SC101_SENSOR_ID_HIGH_REG)? as u16;
        let pid = (midh << 8) | midl;
        if pid == CameraPid::SC101IOT.as_u16() {
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

    /// Returns the 7-bit SCCB slave address (`0x68`).
    pub const fn slv_addr(&self) -> u8 {
        self.slv_addr
    }

    /// Returns the currently configured [`PixelFormat`].
    pub const fn pixformat(&self) -> PixelFormat {
        self.pixformat
    }

    /// Returns the configured external clock frequency in Hz.
    pub const fn xclk_freq_hz(&self) -> u32 {
        self.xclk_freq_hz
    }

    /// Returns the [`CameraModel`] (`CameraModel::Sc101Iot`).
    pub const fn model(&self) -> CameraModel {
        CameraModel::Sc101Iot
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

    fn set_reg_bits<B: SccbBus>(
        &mut self,
        bus: &mut B,
        reg: u16,
        offset: u8,
        length: u8,
        value: u8,
    ) -> Result<(), CameraError> {
        let current = self.get_reg(bus, reg as i32, 0xFF)? as u8;
        let mask = (((1u16 << length) - 1) as u8) << offset;
        let next = (current & !mask) | ((value << offset) & mask);
        self.set_reg(bus, reg as i32, 0xFF, next as i32)
    }

    fn set_window<B: SccbBus>(
        &mut self,
        bus: &mut B,
        offset_x: u16,
        offset_y: u16,
        w: u16,
        h: u16,
    ) -> Result<(), CameraError> {
        let end_x = offset_x.wrapping_add(w);
        let end_y = offset_y.wrapping_add(h);

        self.set_reg(bus, 0x0170, 0xFF, (offset_x & 0xFF) as i32)?;
        self.set_reg(bus, 0x0171, 0xFF, (end_x & 0xFF) as i32)?;
        self.set_reg(
            bus,
            0x0172,
            0xFF,
            (((offset_x >> 8) & 0x0F) | ((end_x >> 4) & 0xF0)) as i32,
        )?;

        self.set_reg(bus, 0x0173, 0xFF, (offset_y & 0xFF) as i32)?;
        self.set_reg(bus, 0x0174, 0xFF, (end_y & 0xFF) as i32)?;
        self.set_reg(
            bus,
            0x0175,
            0xFF,
            (((offset_y >> 8) & 0x0F) | ((end_y >> 4) & 0xF0)) as i32,
        )?;

        bus.delay_ms(10);
        Ok(())
    }
}

impl<B: SccbBus> CameraSensor<B> for Sc101Iot {
    fn model(&self) -> CameraModel {
        CameraModel::Sc101Iot
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
        write_regs8(bus, self.slv_addr, SC101IOT_DEFAULT_INIT_REGS)?;
        bus.delay_ms(50);
        Ok(())
    }

    fn init_status(&mut self, _bus: &mut B) -> Result<(), CameraError> {
        Ok(())
    }

    fn set_pixformat(&mut self, _bus: &mut B, pixformat: PixelFormat) -> Result<(), CameraError> {
        match pixformat {
            PixelFormat::Yuv422 => {
                self.pixformat = pixformat;
                Ok(())
            }
            _ => Err(CameraError::NotSupported),
        }
    }

    fn set_framesize(&mut self, bus: &mut B, framesize: FrameSize) -> Result<(), CameraError> {
        let w = framesize.width();
        let h = framesize.height();
        if w > SC101_MAX_FRAME_WIDTH || h > SC101_MAX_FRAME_HEIGHT {
            return Err(CameraError::FailedToSetFrameSize);
        }
        let offset_x = (SC101_MAX_FRAME_WIDTH - w) / 2;
        let offset_y = (SC101_MAX_FRAME_HEIGHT - h) / 2;
        self.set_window(bus, offset_x, offset_y, w, h)?;
        self.status.framesize = framesize;
        Ok(())
    }

    fn set_contrast(&mut self, bus: &mut B, level: i32) -> Result<(), CameraError> {
        self.set_reg_bits(bus, 0x00F5, 6, 1, 0)?;
        self.set_reg(bus, 0x014B, 0xFF, level & 0xFF)?;
        self.status.contrast = level as i8;
        Ok(())
    }

    fn set_saturation(&mut self, bus: &mut B, level: i32) -> Result<(), CameraError> {
        self.set_reg_bits(bus, 0x00F5, 5, 1, 0)?;
        self.set_reg(bus, 0x0149, 0xFF, level & 0xFF)?;
        self.set_reg(bus, 0x014A, 0xFF, (level >> 8) & 0xFF)?;
        self.status.saturation = level as i8;
        Ok(())
    }

    fn set_sharpness(&mut self, bus: &mut B, level: i32) -> Result<(), CameraError> {
        self.set_reg_bits(bus, 0x00E0, 1, 1, 1)?;
        self.set_reg(bus, 0x00D0, 0xFF, level & 0xFF)?;
        self.set_reg(bus, 0x00D2, 0xFF, (level >> 8) & 0xFF)?;
        self.status.sharpness = level as i8;
        Ok(())
    }

    fn set_colorbar(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        let val = u8::from(enable);
        self.set_reg_bits(bus, 0x0100, 7, 1, val)?;
        self.status.colorbar = val;
        Ok(())
    }

    fn set_hmirror(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        let bits = if enable { 0x03 } else { 0x00 };
        self.set_reg_bits(bus, 0x3221, 1, 2, bits)?;
        self.status.hmirror = u8::from(enable);
        Ok(())
    }

    fn set_vflip(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        let bits = if enable { 0x03 } else { 0x00 };
        self.set_reg_bits(bus, 0x3221, 5, 2, bits)?;
        self.status.vflip = u8::from(enable);
        Ok(())
    }

    fn set_awb_gain(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        let val = i32::from(enable);
        self.set_reg_bits(bus, 0x00B0, 0, 1, 1)?;
        self.set_reg(bus, 0x00C8, 0xFF, val & 0xFF)?;
        self.set_reg(bus, 0x00C9, 0xFF, (val >> 8) & 0xFF)?;
        self.status.awb_gain = u8::from(enable);
        Ok(())
    }

    fn set_agc_gain(&mut self, bus: &mut B, gain: i32) -> Result<(), CameraError> {
        self.set_reg_bits(bus, 0x0070, 1, 1, 1)?;
        self.set_reg(bus, 0x0068, 0xFF, gain & 0xFF)?;
        self.set_reg(bus, 0x0069, 0xFF, (gain >> 8) & 0xFF)?;
        self.set_reg(bus, 0x006A, 0xFF, (gain >> 16) & 0xFF)?;
        self.set_reg(bus, 0x006B, 0xFF, (gain >> 24) & 0xFF)?;
        self.status.agc_gain = (gain & 0xFF) as u8;
        Ok(())
    }

    fn set_aec_value(&mut self, bus: &mut B, gain: i32) -> Result<(), CameraError> {
        self.set_reg_bits(bus, 0x0070, 0, 1, 1)?;
        self.set_reg(bus, 0x0072, 0xFF, gain & 0xFF)?;
        self.status.aec_value = (gain & 0xFFFF) as u16;
        Ok(())
    }

    fn set_raw_gma(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        let val = u8::from(enable);
        self.set_reg_bits(bus, 0x00F5, 1, 1, val)?;
        self.status.raw_gma = val;
        Ok(())
    }

    fn get_reg(&mut self, bus: &mut B, reg: i32, mask: i32) -> Result<i32, CameraError> {
        if !(0..=0xFFFF).contains(&reg) || mask < 0 {
            return Err(CameraError::InvalidArg);
        }
        let reg_high = ((reg >> 8) & 0xFF) as u8;
        let reg_low = (reg & 0xFF) as u8;
        bus.write_reg8(self.slv_addr, SC101_PAGE_SELECT_REG, reg_high)?;
        let val = bus.read_reg8(self.slv_addr, reg_low)? as i32;
        Ok(val & mask)
    }

    fn set_reg(
        &mut self,
        bus: &mut B,
        reg: i32,
        mask: i32,
        value: i32,
    ) -> Result<(), CameraError> {
        if !(0..=0xFFFF).contains(&reg) || mask < 0 {
            return Err(CameraError::InvalidArg);
        }
        let reg_high = ((reg >> 8) & 0xFF) as u8;
        let reg_low = (reg & 0xFF) as u8;
        let mask_u8 = (mask & 0xFF) as u8;
        let next = if mask_u8 != 0xFF {
            let current = self.get_reg(bus, reg, 0xFF)? as u8;
            (current & !mask_u8) | ((value as u8) & mask_u8)
        } else {
            (value & 0xFF) as u8
        };
        bus.write_reg8(self.slv_addr, SC101_PAGE_SELECT_REG, reg_high)?;
        bus.write_reg8(self.slv_addr, reg_low, next)
    }

    fn set_xclk(&mut self, _bus: &mut B, _timer: i32, xclk: i32) -> Result<(), CameraError> {
        if xclk <= 0 {
            return Err(CameraError::InvalidArg);
        }
        self.xclk_freq_hz = (xclk as u32).saturating_mul(1_000_000);
        Ok(())
    }
}
