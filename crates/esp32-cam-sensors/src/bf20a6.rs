//! BF20A6 VGA YUV422/RAW/Grayscale CMOS image sensor driver.
//!
//! Ported from `sensors/bf20a6.c`, `sensors/private_include/bf20a6_regs.h`, and
//! `sensors/private_include/bf20a6_settings.h`.

use crate::{
    CameraError, CameraModel, CameraPid, CameraSccbAddr, CameraSensor, CameraStatus, FrameSize,
    PixelFormat, Reg16, SccbBus, SensorId,
};

pub const SENSOR_ID_HIGH: u8 = 0xFC;
pub const SENSOR_ID_LOW: u8 = 0xFD;
pub const RESET_RELATED: u8 = 0xF2;

pub const REG_DLY: u16 = 0xFFFF;
pub const REGLIST_TAIL: u16 = 0xFFFF;

pub const BF20A6_DEFAULT_INIT_REGS: &[Reg16] = &[
    Reg16::new(0xF2, 0x01),
    Reg16::new(0x12, 0x20),
    Reg16::new(0x3A, 0x00),
    Reg16::new(0xE1, 0x92),
    Reg16::new(0xE3, 0x12),
    Reg16::new(0xE0, 0x00),
    Reg16::new(0x2A, 0x98),
    Reg16::new(0xCD, 0x17),
    Reg16::new(0xC0, 0x10),
    Reg16::new(0xC6, 0x1D),
    Reg16::new(0x10, 0x35),
    Reg16::new(0xE2, 0x09),
    Reg16::new(0xE4, 0x72),
    Reg16::new(0xE5, 0x22),
    Reg16::new(0xE6, 0x24),
    Reg16::new(0xE7, 0x64),
    Reg16::new(0xE8, 0xA2),
    Reg16::new(0x4A, 0x00),
    Reg16::new(0x00, 0x03),
    Reg16::new(0x1F, 0x02),
    Reg16::new(0x22, 0x02),
    Reg16::new(0x0C, 0x31),
    Reg16::new(0x00, 0x00),
    Reg16::new(0x60, 0x81),
    Reg16::new(0x61, 0x81),
    Reg16::new(0xA0, 0x08),
    Reg16::new(0x01, 0x1A),
    Reg16::new(0x02, 0x15),
    Reg16::new(0x13, 0x08),
    Reg16::new(0x8A, 0x96),
    Reg16::new(0x8B, 0x06),
    Reg16::new(0x87, 0x18),
    Reg16::new(0x34, 0x48),
    Reg16::new(0x35, 0x40),
    Reg16::new(0x36, 0x40),
    Reg16::new(0x71, 0x44),
    Reg16::new(0x72, 0x48),
    Reg16::new(0x74, 0xA2),
    Reg16::new(0x75, 0xA9),
    Reg16::new(0x78, 0x12),
    Reg16::new(0x79, 0xA0),
    Reg16::new(0x7A, 0x94),
    Reg16::new(0x7C, 0x97),
    Reg16::new(0x40, 0x30),
    Reg16::new(0x41, 0x30),
    Reg16::new(0x42, 0x28),
    Reg16::new(0x43, 0x1F),
    Reg16::new(0x44, 0x1C),
    Reg16::new(0x45, 0x16),
    Reg16::new(0x46, 0x13),
    Reg16::new(0x47, 0x10),
    Reg16::new(0x48, 0x0D),
    Reg16::new(0x49, 0x0C),
    Reg16::new(0x4B, 0x0A),
    Reg16::new(0x4C, 0x0B),
    Reg16::new(0x4E, 0x09),
    Reg16::new(0x4F, 0x08),
    Reg16::new(0x50, 0x08),
    Reg16::new(0x5F, 0x29),
    Reg16::new(0x23, 0x33),
    Reg16::new(0xA1, 0x10),
    Reg16::new(0xA2, 0x0D),
    Reg16::new(0xA3, 0x30),
    Reg16::new(0xA4, 0x06),
    Reg16::new(0xA5, 0x22),
    Reg16::new(0xA6, 0x56),
    Reg16::new(0xA7, 0x18),
    Reg16::new(0xA8, 0x1A),
    Reg16::new(0xA9, 0x12),
    Reg16::new(0xAA, 0x12),
    Reg16::new(0xAB, 0x16),
    Reg16::new(0xAC, 0xB1),
    Reg16::new(0xBA, 0x12),
    Reg16::new(0xBB, 0x12),
    Reg16::new(0xAD, 0x12),
    Reg16::new(0xAE, 0x56),
    Reg16::new(0xAF, 0x0A),
    Reg16::new(0x3B, 0x30),
    Reg16::new(0x3C, 0x12),
    Reg16::new(0x3D, 0x22),
    Reg16::new(0x3E, 0x3F),
    Reg16::new(0x3F, 0x28),
    Reg16::new(0xB8, 0xC3),
    Reg16::new(0xB9, 0xA3),
    Reg16::new(0x39, 0x47),
    Reg16::new(0x26, 0x13),
    Reg16::new(0x27, 0x16),
    Reg16::new(0x28, 0x14),
    Reg16::new(0x29, 0x18),
    Reg16::new(0xEE, 0x0D),
    Reg16::new(0x13, 0x05),
    Reg16::new(0x24, 0x3C),
    Reg16::new(0x81, 0x20),
    Reg16::new(0x82, 0x40),
    Reg16::new(0x83, 0x30),
    Reg16::new(0x84, 0x58),
    Reg16::new(0x85, 0x30),
    Reg16::new(0x92, 0x08),
    Reg16::new(0x86, 0x80),
    Reg16::new(0x8A, 0x96),
    Reg16::new(0x91, 0xFF),
    Reg16::new(0x94, 0x62),
    Reg16::new(0x9A, 0x18),
    Reg16::new(0xF0, 0x45),
    Reg16::new(0x51, 0x17),
    Reg16::new(0x52, 0x03),
    Reg16::new(0x53, 0x5F),
    Reg16::new(0x54, 0x47),
    Reg16::new(0x55, 0x66),
    Reg16::new(0x56, 0x0F),
    Reg16::new(0x7E, 0x14),
    Reg16::new(0x57, 0x36),
    Reg16::new(0x58, 0x2A),
    Reg16::new(0x59, 0xAA),
    Reg16::new(0x5A, 0xA8),
    Reg16::new(0x5B, 0x43),
    Reg16::new(0x5C, 0x10),
    Reg16::new(0x5D, 0x00),
    Reg16::new(0x7D, 0x36),
    Reg16::new(0x5E, 0x10),
    Reg16::new(0xD6, 0x88),
    Reg16::new(0xD5, 0x20),
    Reg16::new(0xB0, 0x84),
    Reg16::new(0xB5, 0x08),
    Reg16::new(0xB1, 0xC8),
    Reg16::new(0xB2, 0xC0),
    Reg16::new(0xB3, 0xD0),
    Reg16::new(0xB4, 0xB0),
    Reg16::new(0x32, 0x10),
    Reg16::new(0xA0, 0x09),
    Reg16::new(0x00, 0x03),
    Reg16::new(0x0B, 0x02),
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

fn write_reg_list<B: SccbBus>(
    bus: &mut B,
    slv_addr: u8,
    regs: &[Reg16],
) -> Result<(), CameraError> {
    for entry in regs {
        if entry.reg == REGLIST_TAIL {
            break;
        }
        bus.write_reg8(slv_addr, entry.reg as u8, entry.val)?;
    }
    Ok(())
}

/// Detects a BF20A6 sensor at `slv_addr` (`0x6E`).
pub fn detect<B: SccbBus>(bus: &mut B, slv_addr: u8) -> Result<SensorId, CameraError> {
    Bf20a6::detect(bus, slv_addr)
}

/// Safe `#![no_std]` driver for the BYD BF20A6 VGA camera sensor.
#[derive(Debug, Clone)]
pub struct Bf20a6 {
    id: SensorId,
    status: CameraStatus,
    slv_addr: u8,
    pixformat: PixelFormat,
}

pub type BF20A6 = Bf20a6;
pub type Bf20A6 = Bf20a6;

impl Default for Bf20a6 {
    fn default() -> Self {
        Self::new()
    }
}

impl Bf20a6 {
    /// Creates a new [`Bf20a6`] instance with default SCCB address (`0x6E`) and PID (`0x20A6`).
    pub const fn new() -> Self {
        Self {
            id: SensorId {
                midh: 0,
                midl: 0,
                pid: CameraPid::BF20A6.as_u16(),
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
            slv_addr: CameraSccbAddr::BF20A6,
            pixformat: PixelFormat::Yuv422,
        }
    }

    /// Creates a new [`Bf20a6`] with an explicit slave address and [`SensorId`].
    pub const fn with_addr(slv_addr: u8, id: SensorId) -> Self {
        Self {
            id,
            slv_addr,
            ..Self::new()
        }
    }

    /// Probes the SCCB bus at `slv_addr` (`0x6E`) and verifies `BF20A6_PID` (`0x20A6`).
    pub fn detect<B: SccbBus>(bus: &mut B, slv_addr: u8) -> Result<SensorId, CameraError> {
        if slv_addr != CameraSccbAddr::BF20A6 {
            return Err(CameraError::NotDetected);
        }
        let midl = bus.read_reg8(slv_addr, SENSOR_ID_LOW)? as u16;
        let midh = bus.read_reg8(slv_addr, SENSOR_ID_HIGH)? as u16;
        let pid = (midh << 8) | midl;
        if pid == CameraPid::BF20A6.as_u16() {
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

    /// Returns the 7-bit SCCB slave address (`0x6E`).
    pub const fn slv_addr(&self) -> u8 {
        self.slv_addr
    }

    /// Returns the currently configured [`PixelFormat`].
    pub const fn pixformat(&self) -> PixelFormat {
        self.pixformat
    }

    /// Returns the [`CameraModel`] (`CameraModel::Bf20a6`).
    pub const fn model(&self) -> CameraModel {
        CameraModel::Bf20a6
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

impl<B: SccbBus> CameraSensor<B> for Bf20a6 {
    fn model(&self) -> CameraModel {
        CameraModel::Bf20a6
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
        bus.write_reg8(self.slv_addr, RESET_RELATED, 0x01)?;
        bus.delay_ms(100);
        write_reg_list(bus, self.slv_addr, BF20A6_DEFAULT_INIT_REGS)?;
        bus.delay_ms(100);
        Ok(())
    }

    fn init_status(&mut self, bus: &mut B) -> Result<(), CameraError> {
        self.status.brightness = bus.read_reg8(self.slv_addr, 0x6F)? as i8;
        self.status.contrast = bus.read_reg8(self.slv_addr, 0xD6)? as i8;
        self.status.saturation = 0;
        self.status.sharpness = bus.read_reg8(self.slv_addr, 0x70)? as i8;
        self.status.denoise = 0;
        self.status.ae_level = 0;
        self.status.gainceiling = bus.read_reg8(self.slv_addr, 0x13)?;
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
        match pixformat {
            PixelFormat::Yuv422 => {
                set_reg_bits(bus, self.slv_addr, 0x12, 0, 1, 0)?;
            }
            PixelFormat::Raw => {
                set_reg_bits(bus, self.slv_addr, 0x12, 0, 1, 1)?;
            }
            PixelFormat::Grayscale => {
                bus.write_reg8(self.slv_addr, 0x12, 0x23)?;
                bus.write_reg8(self.slv_addr, 0x3A, 0x00)?;
                bus.write_reg8(self.slv_addr, 0xE1, 0x92)?;
                bus.write_reg8(self.slv_addr, 0xE3, 0x02)?;
            }
            _ => return Err(CameraError::NotSupported),
        }
        self.pixformat = pixformat;
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
        bus.write_reg8(self.slv_addr, 0x1B, 0)?;

        if w <= 320 && h <= 240 {
            bus.write_reg8(self.slv_addr, 0x17, (80 - w / 4) as u8)?;
            bus.write_reg8(self.slv_addr, 0x18, (80 + w / 4) as u8)?;
            bus.write_reg8(self.slv_addr, 0x19, (60 - h / 4) as u8)?;
            bus.write_reg8(self.slv_addr, 0x1A, (60 + h / 4) as u8)?;
        } else if w <= 640 && h <= 480 {
            bus.write_reg8(self.slv_addr, 0x17, (80 - w / 8) as u8)?;
            bus.write_reg8(self.slv_addr, 0x18, (80 + w / 8) as u8)?;
            bus.write_reg8(self.slv_addr, 0x19, (60 - h / 8) as u8)?;
            bus.write_reg8(self.slv_addr, 0x1A, (60 + h / 8) as u8)?;
        }

        bus.delay_ms(30);
        Ok(())
    }

    fn set_sharpness(&mut self, bus: &mut B, level: i32) -> Result<(), CameraError> {
        bus.write_reg8(self.slv_addr, 0x70, (level & 0xFF) as u8)?;
        self.status.sharpness = level as i8;
        Ok(())
    }

    fn set_colorbar(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        let val = u8::from(enable);
        bus.write_reg8(self.slv_addr, 0xB6, val)?;
        self.status.colorbar = val;
        Ok(())
    }

    fn set_hmirror(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        let val = u8::from(enable);
        self.status.hmirror = val;
        set_reg_bits(bus, self.slv_addr, 0x4A, 3, 1, val)
    }

    fn set_vflip(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        let val = u8::from(enable);
        self.status.vflip = val;
        set_reg_bits(bus, self.slv_addr, 0x4A, 2, 1, val)
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
}
