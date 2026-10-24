//! BF3005 VGA YUV422/RGB565/RAW/Grayscale CMOS image sensor driver.
//!
//! Ported from `sensors/bf3005.c` and `sensors/private_include/bf3005_regs.h`.

use crate::{
    CameraError, CameraModel, CameraPid, CameraSccbAddr, CameraSensor, CameraStatus, FrameSize,
    PixelFormat, Reg8, SccbBus, SensorId,
};

pub const DEFAULT_REGS: &[Reg8] = &[
    Reg8::new(0x12, 0x40), // soft reset
    Reg8::new(0xFF, 0xFF), // delay
    Reg8::new(0xFF, 0xFF), // delay
    Reg8::new(0xFF, 0xFF), // delay
    Reg8::new(0xFF, 0xFF), // delay
    Reg8::new(0x13, 0x10),
    Reg8::new(0x8C, 0x00),
    Reg8::new(0x8D, 0x64),
    Reg8::new(0x87, 0x10),
    Reg8::new(0x13, 0x17),
    Reg8::new(0x00, 0x20),
    Reg8::new(0x01, 0x1A),
    Reg8::new(0x02, 0x22),
    Reg8::new(0x09, 0x03),
    Reg8::new(0x0C, 0x80),
    Reg8::new(0x0D, 0x24),
    Reg8::new(0x0E, 0x21),
    Reg8::new(0x0F, 0x28),
    Reg8::new(0x11, 0x08),
    Reg8::new(0x15, 0x10),
    Reg8::new(0x16, 0x03),
    Reg8::new(0x1E, 0x30),
    Reg8::new(0x20, 0x8A),
    Reg8::new(0x21, 0x03),
    Reg8::new(0x23, 0x55),
    Reg8::new(0x24, 0x68),
    Reg8::new(0x25, 0x78),
    Reg8::new(0x2A, 0x00),
    Reg8::new(0x2B, 0x00),
    Reg8::new(0x2D, 0x4F),
    Reg8::new(0x2E, 0x98),
    Reg8::new(0x2F, 0x04),
    Reg8::new(0x30, 0xAD),
    Reg8::new(0x31, 0x17),
    Reg8::new(0x32, 0x6E),
    Reg8::new(0x33, 0x20),
    Reg8::new(0x35, 0xA6),
    Reg8::new(0x3B, 0x00),
    Reg8::new(0x3E, 0x00),
    Reg8::new(0x3F, 0xA8),
    Reg8::new(0x40, 0x38),
    Reg8::new(0x41, 0x32),
    Reg8::new(0x42, 0x2B),
    Reg8::new(0x43, 0x26),
    Reg8::new(0x44, 0x1A),
    Reg8::new(0x45, 0x16),
    Reg8::new(0x46, 0x10),
    Reg8::new(0x47, 0x0F),
    Reg8::new(0x48, 0x0C),
    Reg8::new(0x49, 0x0A),
    Reg8::new(0x4B, 0x09),
    Reg8::new(0x4C, 0x08),
    Reg8::new(0x4D, 0x3C),
    Reg8::new(0x4E, 0x06),
    Reg8::new(0x4F, 0x05),
    Reg8::new(0x50, 0x03),
    Reg8::new(0x51, 0x25),
    Reg8::new(0x52, 0x88),
    Reg8::new(0x53, 0x03),
    Reg8::new(0x63, 0x20),
    Reg8::new(0x64, 0x02),
    Reg8::new(0x65, 0xA6),
    Reg8::new(0x66, 0xB6),
    Reg8::new(0x69, 0x00),
    Reg8::new(0x70, 0xFF),
    Reg8::new(0x71, 0xA6),
    Reg8::new(0x72, 0x2F),
    Reg8::new(0x73, 0x2F),
    Reg8::new(0x74, 0x2F),
    Reg8::new(0x75, 0x0E),
    Reg8::new(0x76, 0x1E),
    Reg8::new(0x77, 0x00),
    Reg8::new(0x78, 0x1E),
    Reg8::new(0x79, 0x8A),
    Reg8::new(0x7D, 0xE2),
    Reg8::new(0x80, 0x44),
    Reg8::new(0x81, 0x00),
    Reg8::new(0x82, 0x18),
    Reg8::new(0x83, 0x1B),
    Reg8::new(0x84, 0x24),
    Reg8::new(0x85, 0x2A),
    Reg8::new(0x86, 0x4F),
    Reg8::new(0x89, 0x82),
    Reg8::new(0x8B, 0x02),
    Reg8::new(0x8E, 0x03),
    Reg8::new(0x8F, 0xFC),
    Reg8::new(0x9D, 0x4D),
    Reg8::new(0x9E, 0x41),
    Reg8::new(0xA1, 0x21),
    Reg8::new(0xA2, 0x12),
    Reg8::new(0xA3, 0x32),
    Reg8::new(0xA4, 0x05),
    Reg8::new(0xA5, 0x32),
    Reg8::new(0xA6, 0x04),
    Reg8::new(0xA7, 0x7F),
    Reg8::new(0xA8, 0x7F),
    Reg8::new(0xA9, 0x21),
    Reg8::new(0xAA, 0x21),
    Reg8::new(0xAB, 0x21),
    Reg8::new(0xAC, 0x0A),
    Reg8::new(0xAD, 0xF0),
    Reg8::new(0xAE, 0xFF),
    Reg8::new(0xAF, 0x1D),
    Reg8::new(0xB0, 0x94),
    Reg8::new(0xB1, 0xC0),
    Reg8::new(0xB2, 0xC0),
    Reg8::new(0xD2, 0x30),
    Reg8::new(0xE0, 0x0D),
    Reg8::new(0xE1, 0x44),
    Reg8::new(0xE7, 0x7C),
    Reg8::new(0xE8, 0x89),
    Reg8::new(0xE9, 0x01),
    Reg8::new(0xEA, 0x01),
    Reg8::new(0xF0, 0x01),
    Reg8::new(0xF3, 0x49),
    Reg8::new(0xF4, 0xFF),
    Reg8::new(0xF5, 0x01),
    Reg8::new(0xF6, 0xF2),
    Reg8::new(0xF7, 0x6F),
    Reg8::new(0x1B, 0x80),
    Reg8::new(0x00, 0x00),
];

#[inline]
fn set_reg_bits<B: SccbBus>(
    bus: &mut B,
    slv_addr: u8,
    reg: u8,
    offset: u8,
    length: u8,
    value: u8,
) -> Result<(), CameraError> {
    let current = bus.read_reg8(slv_addr, reg)?;
    let mask = (((1u16 << length) - 1) as u8) << offset;
    let next = (current & !mask) | ((value << offset) & mask);
    bus.write_reg8(slv_addr, reg, next)
}

#[inline]
fn get_reg_bits<B: SccbBus>(
    bus: &mut B,
    slv_addr: u8,
    reg: u8,
    offset: u8,
    length: u8,
) -> Result<u8, CameraError> {
    let current = bus.read_reg8(slv_addr, reg)?;
    let mask = (((1u16 << length) - 1) as u8) << offset;
    Ok((current & mask) >> offset)
}

/// Detects a BF3005 sensor at `slv_addr` (`0x6E`).
pub fn detect<B: SccbBus>(bus: &mut B, slv_addr: u8) -> Result<SensorId, CameraError> {
    Bf3005::detect(bus, slv_addr)
}

/// Safe `#![no_std]` driver for the BYD BF3005 VGA camera sensor.
#[derive(Debug, Clone)]
pub struct Bf3005 {
    id: SensorId,
    status: CameraStatus,
    slv_addr: u8,
    pixformat: PixelFormat,
    xclk_freq_hz: u32,
}

pub type BF3005 = Bf3005;

impl Default for Bf3005 {
    fn default() -> Self {
        Self::new()
    }
}

impl Bf3005 {
    /// Creates a new [`Bf3005`] instance with default SCCB address (`0x6E`) and PID (`0x30`).
    pub const fn new() -> Self {
        Self {
            id: SensorId {
                midh: 0,
                midl: 0,
                pid: CameraPid::BF3005.as_u16(),
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
            slv_addr: CameraSccbAddr::BF3005,
            pixformat: PixelFormat::Yuv422,
            xclk_freq_hz: 20_000_000,
        }
    }

    /// Creates a new [`Bf3005`] with an explicit slave address and [`SensorId`].
    pub const fn with_addr(slv_addr: u8, id: SensorId) -> Self {
        Self {
            id,
            slv_addr,
            ..Self::new()
        }
    }

    /// Probes the SCCB bus at `slv_addr` (`0x6E`) and verifies `BF3005_PID` (`0x30`).
    pub fn detect<B: SccbBus>(bus: &mut B, slv_addr: u8) -> Result<SensorId, CameraError> {
        if slv_addr != CameraSccbAddr::BF3005 {
            return Err(CameraError::NotDetected);
        }
        let pid = bus.read_reg8(slv_addr, 0xFC)? as u16;
        if pid == CameraPid::BF3005.as_u16() {
            let ver = bus.read_reg8(slv_addr, 0xFD)?;
            let midl = bus.read_reg8(slv_addr, 0xFC)?;
            let midh = bus.read_reg8(slv_addr, 0xFD)?;
            Ok(SensorId {
                midh,
                midl,
                pid,
                ver,
            })
        } else {
            Err(CameraError::NotDetected)
        }
    }

    /// Returns the 7-bit SCCB slave address (`0x6E`).
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

    /// Returns the [`CameraModel`] (`CameraModel::Bf3005`).
    pub const fn model(&self) -> CameraModel {
        CameraModel::Bf3005
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

impl<B: SccbBus> CameraSensor<B> for Bf3005 {
    fn model(&self) -> CameraModel {
        CameraModel::Bf3005
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
        for entry in DEFAULT_REGS {
            if entry.reg == 0x00 && entry.val == 0x00 {
                break;
            }
            if entry.reg == 0xFF && entry.val == 0xFF {
                bus.delay_ms(10);
            } else {
                bus.write_reg8(self.slv_addr, entry.reg, entry.val)?;
            }
        }
        bus.delay_ms(50);
        Ok(())
    }

    fn init_status(&mut self, bus: &mut B) -> Result<(), CameraError> {
        self.status.brightness = bus.read_reg8(self.slv_addr, 0x55)? as i8;
        self.status.contrast = bus.read_reg8(self.slv_addr, 0x56)? as i8;
        self.status.saturation = 0;
        self.status.ae_level = 0;
        self.status.gainceiling = bus.read_reg8(self.slv_addr, 0x87)?;
        self.status.awb = get_reg_bits(bus, self.slv_addr, 0x13, 1, 1)?;
        self.status.awb_gain = bus.read_reg8(self.slv_addr, 0xA6)?;
        self.status.aec = get_reg_bits(bus, self.slv_addr, 0x13, 0, 1)?;
        self.status.agc = get_reg_bits(bus, self.slv_addr, 0x13, 2, 1)?;
        self.status.raw_gma = get_reg_bits(bus, self.slv_addr, 0xF1, 1, 1)?;
        self.status.lenc = get_reg_bits(bus, self.slv_addr, 0xF1, 0, 1)?;
        self.status.hmirror = get_reg_bits(bus, self.slv_addr, 0x1E, 5, 1)?;
        self.status.vflip = get_reg_bits(bus, self.slv_addr, 0x1E, 4, 1)?;
        self.status.colorbar = bus.read_reg8(self.slv_addr, 0xB9)?;
        self.status.sharpness = bus.read_reg8(self.slv_addr, 0x70)? as i8;
        Ok(())
    }

    fn set_pixformat(&mut self, bus: &mut B, pixformat: PixelFormat) -> Result<(), CameraError> {
        match pixformat {
            PixelFormat::Rgb565 => {
                set_reg_bits(bus, self.slv_addr, 0x12, 2, 1, 1)?;
            }
            PixelFormat::Raw => {
                set_reg_bits(bus, self.slv_addr, 0x12, 0, 3, 0x04)?;
            }
            PixelFormat::Yuv422 | PixelFormat::Grayscale => {
                set_reg_bits(bus, self.slv_addr, 0x12, 2, 1, 0)?;
            }
            _ => return Err(CameraError::NotSupported),
        }
        self.pixformat = pixformat;
        bus.delay_ms(30);
        Ok(())
    }

    fn set_framesize(&mut self, bus: &mut B, framesize: FrameSize) -> Result<(), CameraError> {
        if framesize > FrameSize::Vga {
            return Err(CameraError::FailedToSetFrameSize);
        }
        let w = framesize.width();
        let h = framesize.height();
        self.status.framesize = framesize;

        bus.write_reg8(self.slv_addr, 0x17, 0)?;
        bus.write_reg8(self.slv_addr, 0x18, (w >> 2) as u8)?;
        bus.write_reg8(self.slv_addr, 0x19, 0)?;
        bus.write_reg8(self.slv_addr, 0x1A, (h >> 2) as u8)?;
        bus.write_reg8(self.slv_addr, 0x03, 0)?;

        if w <= 320 && h <= 240 {
            set_reg_bits(bus, self.slv_addr, 0x12, 4, 1, 1)?;
            bus.write_reg8(self.slv_addr, 0x17, (80 - w / 4) as u8)?;
            bus.write_reg8(self.slv_addr, 0x18, (80 + w / 4) as u8)?;
            bus.write_reg8(self.slv_addr, 0x19, (60 - h / 4) as u8)?;
            bus.write_reg8(self.slv_addr, 0x1A, (60 + h / 4) as u8)?;
            bus.write_reg8(self.slv_addr, 0x03, 0)?;
        } else if w <= 640 && h <= 480 {
            set_reg_bits(bus, self.slv_addr, 0x12, 4, 1, 0)?;
            bus.write_reg8(self.slv_addr, 0x17, (80 - w / 8) as u8)?;
            bus.write_reg8(self.slv_addr, 0x18, (80 + w / 8) as u8)?;
            bus.write_reg8(self.slv_addr, 0x19, (60 - h / 8) as u8)?;
            bus.write_reg8(self.slv_addr, 0x1A, (60 + h / 8) as u8)?;
            bus.write_reg8(self.slv_addr, 0x03, 0)?;
        }

        bus.delay_ms(30);
        Ok(())
    }

    fn set_contrast(&mut self, bus: &mut B, level: i32) -> Result<(), CameraError> {
        bus.write_reg8(self.slv_addr, 0x56, (level & 0xFF) as u8)?;
        self.status.contrast = level as i8;
        Ok(())
    }

    fn set_brightness(&mut self, bus: &mut B, level: i32) -> Result<(), CameraError> {
        bus.write_reg8(self.slv_addr, 0x55, (level & 0xFF) as u8)?;
        self.status.brightness = level as i8;
        Ok(())
    }

    fn set_sharpness(&mut self, bus: &mut B, level: i32) -> Result<(), CameraError> {
        bus.write_reg8(self.slv_addr, 0x70, (level & 0xFF) as u8)?;
        self.status.sharpness = level as i8;
        Ok(())
    }

    fn set_colorbar(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        let val = u8::from(enable);
        self.status.colorbar = val;
        bus.write_reg8(self.slv_addr, 0xB9, val)
    }

    fn set_whitebal(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        let val = u8::from(enable);
        set_reg_bits(bus, self.slv_addr, 0x13, 1, 1, val)?;
        self.status.awb = val;
        Ok(())
    }

    fn set_gain_ctrl(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        let val = u8::from(enable);
        set_reg_bits(bus, self.slv_addr, 0x13, 2, 1, val)?;
        self.status.agc = val;
        Ok(())
    }

    fn set_exposure_ctrl(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        let val = u8::from(enable);
        set_reg_bits(bus, self.slv_addr, 0x13, 0, 1, val)?;
        self.status.aec = val;
        Ok(())
    }

    fn set_hmirror(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        let val = u8::from(enable);
        set_reg_bits(bus, self.slv_addr, 0x1E, 5, 1, val)?;
        self.status.hmirror = val;
        Ok(())
    }

    fn set_vflip(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        let val = u8::from(enable);
        set_reg_bits(bus, self.slv_addr, 0x1E, 4, 1, val)?;
        self.status.vflip = val;
        Ok(())
    }

    fn set_awb_gain(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        let val = u8::from(enable);
        bus.write_reg8(self.slv_addr, 0xA6, val)?;
        self.status.awb_gain = val;
        Ok(())
    }

    fn set_agc_gain(&mut self, bus: &mut B, gain: i32) -> Result<(), CameraError> {
        let opt = u8::from(gain != 0);
        set_reg_bits(bus, self.slv_addr, 0x13, 4, 1, opt)?;
        self.status.agc_gain = opt;
        Ok(())
    }

    fn set_raw_gma(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        let inv = u8::from(!enable);
        set_reg_bits(bus, self.slv_addr, 0xF1, 1, 1, inv)?;
        self.status.raw_gma = inv;
        Ok(())
    }

    fn set_lenc(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        let inv = u8::from(!enable);
        set_reg_bits(bus, self.slv_addr, 0xF1, 0, 1, inv)?;
        self.status.lenc = inv;
        Ok(())
    }

    fn get_reg(&mut self, bus: &mut B, reg: i32, mask: i32) -> Result<i32, CameraError> {
        if !(0..=0xFF).contains(&reg) || !(0..=0xFF).contains(&mask) {
            return Err(CameraError::InvalidArg);
        }
        let val = bus.read_reg8(self.slv_addr, reg as u8)? as i32;
        Ok(val & mask)
    }

    fn set_reg(
        &mut self,
        bus: &mut B,
        reg: i32,
        mask: i32,
        value: i32,
    ) -> Result<(), CameraError> {
        if !(0..=0xFF).contains(&reg) || !(0..=0xFF).contains(&mask) {
            return Err(CameraError::InvalidArg);
        }
        let reg8 = reg as u8;
        let current = bus.read_reg8(self.slv_addr, reg8)? as i32;
        let next = ((current & !mask) | (value & mask)) as u8;
        bus.write_reg8(self.slv_addr, reg8, next)
    }

    fn set_xclk(&mut self, _bus: &mut B, _timer: i32, xclk: i32) -> Result<(), CameraError> {
        if xclk <= 0 {
            return Err(CameraError::InvalidArg);
        }
        self.xclk_freq_hz = (xclk as u32).saturating_mul(1_000_000);
        Ok(())
    }
}
