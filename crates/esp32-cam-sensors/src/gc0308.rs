//! GC0308 VGA YUV422/RGB565/Grayscale CMOS image sensor driver.
//!
//! Ported from `sensors/gc0308.c`, `sensors/private_include/gc0308_regs.h`, and
//! `sensors/private_include/gc0308_settings.h`.

use crate::{
    CameraError, CameraModel, CameraPid, CameraSccbAddr, CameraSensor, CameraStatus, FrameSize,
    PixelFormat, Reg8, SccbBus, SensorId,
};

pub const RESET_RELATED: u8 = 0xFE;
pub const EXPOSURE_HIGH: u8 = 0x03;
pub const EXPOSURE_LOW: u8 = 0x04;
pub const ROW_START_H: u8 = 0x05;
pub const ROW_START_L: u8 = 0x06;
pub const COL_START_H: u8 = 0x07;
pub const COL_START_L: u8 = 0x08;
pub const WIN_HEIGHT_H: u8 = 0x09;
pub const WIN_HEIGHT_L: u8 = 0x0A;
pub const WIN_WIDTH_H: u8 = 0x0B;
pub const WIN_WIDTH_L: u8 = 0x0C;
pub const CISCTL_MODE1: u8 = 0x14;
pub const AAAA_EN: u8 = 0x22;
pub const SPECIAL_EFFECT: u8 = 0x23;
pub const OUTPUT_FMT: u8 = 0x24;
pub const PCLK_DIV: u8 = 0x28;
pub const OUT_CTRL: u8 = 0x2E;
pub const GLOBAL_GAIN: u8 = 0x50;
pub const AWB_R_GAIN: u8 = 0x5A;
pub const AWB_G_GAIN: u8 = 0x5B;
pub const AWB_B_GAIN: u8 = 0x5C;
pub const EDGE12_EFFECT: u8 = 0x77;
pub const EDGE_POS_RATIO: u8 = 0x78;
pub const EDGE1_MINMAX: u8 = 0x79;
pub const SATURATION_CB: u8 = 0xB1;
pub const SATURATION_CR: u8 = 0xB2;
pub const CONTRAST: u8 = 0xB3;
pub const AEC_MODE1: u8 = 0xD0;
pub const AEC_MODE2: u8 = 0xD1;
pub const AEC_MODE3: u8 = 0xD2;
pub const AEC_TARGET_Y: u8 = 0xD3;
pub const SUB_COL_N: u8 = 0xF7;
pub const SUB_ROW_N: u8 = 0xF8;
pub const SUB_COL_N1: u8 = 0xF9;
pub const SUB_ROW_N1: u8 = 0xFA;

pub const SUBSAMPLE_EN: u8 = 0x53;
pub const SUBSAMPLE_MODE: u8 = 0x54;
pub const SUBSAMPLE_EN2: u8 = 0x55;
pub const SUBSAMPLE_Y0: u8 = 0x56;
pub const SUBSAMPLE_Y1: u8 = 0x57;
pub const SUBSAMPLE_UV0: u8 = 0x58;
pub const SUBSAMPLE_UV1: u8 = 0x59;

pub const AEC_ENABLE: u8 = 0x01;
pub const AWB_ENABLE: u8 = 0x02;
pub const AGC_ENABLE: u8 = 0x04;
pub const HMIRROR_MASK: u8 = 0x01;
pub const VFLIP_MASK: u8 = 0x02;
pub const EFFECT_NORMAL: u8 = 0x00;
pub const EFFECT_NEGATIVE: u8 = 0x01;
pub const EFFECT_GRAYSCALE: u8 = 0x02;
pub const COLORBAR_ENABLE: u8 = 0x01;
pub const REG_DLY: u8 = 0xFF;

pub const GC0308_SENSOR_DEFAULT_REGS: &[Reg8] = &[
    Reg8::new(0xFE, 0x00),
    Reg8::new(0xEC, 0x20),
    Reg8::new(0x05, 0x00),
    Reg8::new(0x06, 0x00),
    Reg8::new(0x07, 0x00),
    Reg8::new(0x08, 0x00),
    Reg8::new(0x09, 0x01),
    Reg8::new(0x0A, 0xE8),
    Reg8::new(0x0B, 0x02),
    Reg8::new(0x0C, 0x88),
    Reg8::new(0x0D, 0x02),
    Reg8::new(0x0E, 0x02),
    Reg8::new(0x10, 0x26),
    Reg8::new(0x11, 0x0D),
    Reg8::new(0x12, 0x2A),
    Reg8::new(0x13, 0x00),
    Reg8::new(0x14, 0x10),
    Reg8::new(0x15, 0x0A),
    Reg8::new(0x16, 0x05),
    Reg8::new(0x17, 0x01),
    Reg8::new(0x18, 0x44),
    Reg8::new(0x19, 0x44),
    Reg8::new(0x1A, 0x2A),
    Reg8::new(0x1B, 0x00),
    Reg8::new(0x1C, 0x49),
    Reg8::new(0x1D, 0x9A),
    Reg8::new(0x1E, 0x61),
    Reg8::new(0x1F, 0x00),
    Reg8::new(0x20, 0x7F),
    Reg8::new(0x21, 0xFA),
    Reg8::new(0x22, 0x57),
    Reg8::new(0x24, 0xA2),
    Reg8::new(0x25, 0x0F),
    Reg8::new(0x26, 0x03),
    Reg8::new(0x28, 0x00),
    Reg8::new(0x2D, 0x0A),
    Reg8::new(0x2F, 0x01),
    Reg8::new(0x30, 0xF7),
    Reg8::new(0x31, 0x50),
    Reg8::new(0x32, 0x00),
    Reg8::new(0x33, 0x28),
    Reg8::new(0x34, 0x2A),
    Reg8::new(0x35, 0x28),
    Reg8::new(0x39, 0x04),
    Reg8::new(0x3A, 0x20),
    Reg8::new(0x3B, 0x20),
    Reg8::new(0x3C, 0x00),
    Reg8::new(0x3D, 0x00),
    Reg8::new(0x3E, 0x00),
    Reg8::new(0x3F, 0x00),
    Reg8::new(0x50, 0x14),
    Reg8::new(0x52, 0x41),
    Reg8::new(0x53, 0x80),
    Reg8::new(0x54, 0x80),
    Reg8::new(0x55, 0x80),
    Reg8::new(0x56, 0x80),
    Reg8::new(0x5A, 0x56),
    Reg8::new(0x5B, 0x40),
    Reg8::new(0x5C, 0x4A),
    Reg8::new(0x8B, 0x20),
    Reg8::new(0x8C, 0x20),
    Reg8::new(0x8D, 0x20),
    Reg8::new(0x8E, 0x14),
    Reg8::new(0x8F, 0x10),
    Reg8::new(0x90, 0x14),
    Reg8::new(0x91, 0x3C),
    Reg8::new(0x92, 0x50),
    Reg8::new(0x5D, 0x12),
    Reg8::new(0x5E, 0x1A),
    Reg8::new(0x5F, 0x24),
    Reg8::new(0x60, 0x07),
    Reg8::new(0x61, 0x15),
    Reg8::new(0x62, 0x08),
    Reg8::new(0x64, 0x03),
    Reg8::new(0x66, 0xE8),
    Reg8::new(0x67, 0x86),
    Reg8::new(0x68, 0x82),
    Reg8::new(0x69, 0x18),
    Reg8::new(0x6A, 0x0F),
    Reg8::new(0x6B, 0x00),
    Reg8::new(0x6C, 0x5F),
    Reg8::new(0x6D, 0x8F),
    Reg8::new(0x6E, 0x55),
    Reg8::new(0x6F, 0x38),
    Reg8::new(0x70, 0x15),
    Reg8::new(0x71, 0x33),
    Reg8::new(0x72, 0xDC),
    Reg8::new(0x73, 0x00),
    Reg8::new(0x74, 0x02),
    Reg8::new(0x75, 0x3F),
    Reg8::new(0x76, 0x02),
    Reg8::new(0x77, 0x38),
    Reg8::new(0x78, 0x88),
    Reg8::new(0x79, 0x81),
    Reg8::new(0x7A, 0x81),
    Reg8::new(0x7B, 0x22),
    Reg8::new(0x7C, 0xFF),
    Reg8::new(0x93, 0x48),
    Reg8::new(0x94, 0x02),
    Reg8::new(0x95, 0x07),
    Reg8::new(0x96, 0xE0),
    Reg8::new(0x97, 0x40),
    Reg8::new(0x98, 0xF0),
    Reg8::new(0xB1, 0x40),
    Reg8::new(0xB2, 0x40),
    Reg8::new(0xB3, 0x40),
    Reg8::new(0xB6, 0xE0),
    Reg8::new(0xBD, 0x38),
    Reg8::new(0xBE, 0x36),
    Reg8::new(0xD0, 0xCB),
    Reg8::new(0xD1, 0x10),
    Reg8::new(0xD2, 0x90),
    Reg8::new(0xD3, 0x48),
    Reg8::new(0xD5, 0xF2),
    Reg8::new(0xD6, 0x16),
    Reg8::new(0xDB, 0x92),
    Reg8::new(0xDC, 0xA5),
    Reg8::new(0xDF, 0x23),
    Reg8::new(0xD9, 0x00),
    Reg8::new(0xDA, 0x00),
    Reg8::new(0xE0, 0x09),
    Reg8::new(0xED, 0x04),
    Reg8::new(0xEE, 0xA0),
    Reg8::new(0xEF, 0x40),
    Reg8::new(0x80, 0x03),
    Reg8::new(0x9F, 0x10),
    Reg8::new(0xA0, 0x20),
    Reg8::new(0xA1, 0x38),
    Reg8::new(0xA2, 0x4E),
    Reg8::new(0xA3, 0x63),
    Reg8::new(0xA4, 0x76),
    Reg8::new(0xA5, 0x87),
    Reg8::new(0xA6, 0xA2),
    Reg8::new(0xA7, 0xB8),
    Reg8::new(0xA8, 0xCA),
    Reg8::new(0xA9, 0xD8),
    Reg8::new(0xAA, 0xE3),
    Reg8::new(0xAB, 0xEB),
    Reg8::new(0xAC, 0xF0),
    Reg8::new(0xAD, 0xF8),
    Reg8::new(0xAE, 0xFD),
    Reg8::new(0xAF, 0xFF),
    Reg8::new(0xC0, 0x00),
    Reg8::new(0xC1, 0x10),
    Reg8::new(0xC2, 0x1C),
    Reg8::new(0xC3, 0x30),
    Reg8::new(0xC4, 0x43),
    Reg8::new(0xC5, 0x54),
    Reg8::new(0xC6, 0x65),
    Reg8::new(0xC7, 0x75),
    Reg8::new(0xC8, 0x93),
    Reg8::new(0xC9, 0xB0),
    Reg8::new(0xCA, 0xCB),
    Reg8::new(0xCB, 0xE6),
    Reg8::new(0xCC, 0xFF),
    Reg8::new(0xF0, 0x02),
    Reg8::new(0xF1, 0x01),
    Reg8::new(0xF2, 0x02),
    Reg8::new(0xF3, 0x30),
    Reg8::new(0xF7, 0x04),
    Reg8::new(0xF8, 0x02),
    Reg8::new(0xF9, 0x9F),
    Reg8::new(0xFA, 0x78),
    Reg8::new(0xFE, 0x01),
    Reg8::new(0x00, 0xF5),
    Reg8::new(0x02, 0x20),
    Reg8::new(0x04, 0x10),
    Reg8::new(0x05, 0x08),
    Reg8::new(0x06, 0x20),
    Reg8::new(0x08, 0x0A),
    Reg8::new(0x0A, 0xA0),
    Reg8::new(0x0B, 0x60),
    Reg8::new(0x0C, 0x08),
    Reg8::new(0x0E, 0x44),
    Reg8::new(0x0F, 0x32),
    Reg8::new(0x10, 0x41),
    Reg8::new(0x11, 0x37),
    Reg8::new(0x12, 0x22),
    Reg8::new(0x13, 0x19),
    Reg8::new(0x14, 0x44),
    Reg8::new(0x15, 0x44),
    Reg8::new(0x16, 0xC2),
    Reg8::new(0x17, 0xA8),
    Reg8::new(0x18, 0x18),
    Reg8::new(0x19, 0x50),
    Reg8::new(0x1A, 0xD8),
    Reg8::new(0x1B, 0xF5),
    Reg8::new(0x70, 0x40),
    Reg8::new(0x71, 0x58),
    Reg8::new(0x72, 0x30),
    Reg8::new(0x73, 0x48),
    Reg8::new(0x74, 0x20),
    Reg8::new(0x75, 0x60),
    Reg8::new(0x77, 0x20),
    Reg8::new(0x78, 0x32),
    Reg8::new(0x30, 0x03),
    Reg8::new(0x31, 0x40),
    Reg8::new(0x32, 0x10),
    Reg8::new(0x33, 0xE0),
    Reg8::new(0x34, 0xE0),
    Reg8::new(0x35, 0x00),
    Reg8::new(0x36, 0x80),
    Reg8::new(0x37, 0x00),
    Reg8::new(0x38, 0x04),
    Reg8::new(0x39, 0x09),
    Reg8::new(0x3A, 0x12),
    Reg8::new(0x3B, 0x1C),
    Reg8::new(0x3C, 0x28),
    Reg8::new(0x3D, 0x31),
    Reg8::new(0x3E, 0x44),
    Reg8::new(0x3F, 0x57),
    Reg8::new(0x40, 0x6C),
    Reg8::new(0x41, 0x81),
    Reg8::new(0x42, 0x94),
    Reg8::new(0x43, 0xA7),
    Reg8::new(0x44, 0xB8),
    Reg8::new(0x45, 0xD6),
    Reg8::new(0x46, 0xEE),
    Reg8::new(0x47, 0x0D),
    Reg8::new(0x62, 0xF7),
    Reg8::new(0x63, 0x68),
    Reg8::new(0x64, 0xD3),
    Reg8::new(0x65, 0xD3),
    Reg8::new(0x66, 0x60),
    Reg8::new(0xFE, 0x00),
    Reg8::new(0x01, 0x32),
    Reg8::new(0x02, 0x0C),
    Reg8::new(0x0F, 0x01),
    Reg8::new(0xE2, 0x00),
    Reg8::new(0xE3, 0x78),
    Reg8::new(0xE4, 0x00),
    Reg8::new(0xE5, 0xFE),
    Reg8::new(0xE6, 0x01),
    Reg8::new(0xE7, 0xE0),
    Reg8::new(0xE8, 0x01),
    Reg8::new(0xE9, 0xE0),
    Reg8::new(0xEA, 0x01),
    Reg8::new(0xEB, 0xE0),
    Reg8::new(0xFE, 0x00),
];

#[derive(Clone, Copy)]
struct SubsampleCfg {
    ratio_numerator: u16,
    ratio_denominator: u16,
    reg0x54: u8,
    reg0x56: u8,
    reg0x57: u8,
    reg0x58: u8,
    reg0x59: u8,
}

const SUBSAMPLE_CFGS: [SubsampleCfg; 8] = [
    SubsampleCfg {
        ratio_numerator: 84,
        ratio_denominator: 420,
        reg0x54: 0x55,
        reg0x56: 0x00,
        reg0x57: 0x00,
        reg0x58: 0x00,
        reg0x59: 0x00,
    },
    SubsampleCfg {
        ratio_numerator: 105,
        ratio_denominator: 420,
        reg0x54: 0x44,
        reg0x56: 0x00,
        reg0x57: 0x00,
        reg0x58: 0x00,
        reg0x59: 0x00,
    },
    SubsampleCfg {
        ratio_numerator: 140,
        ratio_denominator: 420,
        reg0x54: 0x33,
        reg0x56: 0x00,
        reg0x57: 0x00,
        reg0x58: 0x00,
        reg0x59: 0x00,
    },
    SubsampleCfg {
        ratio_numerator: 210,
        ratio_denominator: 420,
        reg0x54: 0x22,
        reg0x56: 0x00,
        reg0x57: 0x00,
        reg0x58: 0x00,
        reg0x59: 0x00,
    },
    SubsampleCfg {
        ratio_numerator: 240,
        ratio_denominator: 420,
        reg0x54: 0x77,
        reg0x56: 0x02,
        reg0x57: 0x46,
        reg0x58: 0x02,
        reg0x59: 0x46,
    },
    SubsampleCfg {
        ratio_numerator: 252,
        ratio_denominator: 420,
        reg0x54: 0x55,
        reg0x56: 0x02,
        reg0x57: 0x04,
        reg0x58: 0x02,
        reg0x59: 0x04,
    },
    SubsampleCfg {
        ratio_numerator: 280,
        ratio_denominator: 420,
        reg0x54: 0x33,
        reg0x56: 0x02,
        reg0x57: 0x00,
        reg0x58: 0x02,
        reg0x59: 0x00,
    },
    SubsampleCfg {
        ratio_numerator: 420,
        ratio_denominator: 420,
        reg0x54: 0x11,
        reg0x56: 0x00,
        reg0x57: 0x00,
        reg0x58: 0x00,
        reg0x59: 0x00,
    },
];

#[inline]
const fn h8(v: u16) -> u8 {
    (v >> 8) as u8
}

#[inline]
const fn l8(v: u16) -> u8 {
    (v & 0xFF) as u8
}

#[inline]
fn set_reg_bits<B: SccbBus>(
    bus: &mut B,
    slv_addr: u8,
    reg: u8,
    offset: u8,
    mask: u8,
    value: u8,
) -> Result<(), CameraError> {
    let c_value = bus.read_reg8(slv_addr, reg)?;
    let shifted_mask = mask << offset;
    let new_value = (c_value & !shifted_mask) | ((value & mask) << offset);
    bus.write_reg8(slv_addr, reg, new_value)
}

#[inline]
fn check_reg_mask<B: SccbBus>(bus: &mut B, slv_addr: u8, reg: u8, mask: u8) -> Result<u8, CameraError> {
    let val = bus.read_reg8(slv_addr, reg)?;
    Ok(u8::from((val & mask) == mask))
}

fn write_reg_list<B: SccbBus>(
    bus: &mut B,
    slv_addr: u8,
    regs: &[Reg8],
) -> Result<(), CameraError> {
    for entry in regs {
        if entry.reg == REG_DLY {
            bus.delay_ms(entry.val as u32);
        } else {
            bus.write_reg8(slv_addr, entry.reg, entry.val)?;
        }
    }
    Ok(())
}

/// Detects a GC0308 sensor at `slv_addr` (`0x21`).
pub fn detect<B: SccbBus>(bus: &mut B, slv_addr: u8) -> Result<SensorId, CameraError> {
    Gc0308::detect(bus, slv_addr)
}

/// Safe `#![no_std]` driver for the GalaxyCore GC0308 VGA camera sensor.
#[derive(Debug, Clone)]
pub struct Gc0308 {
    id: SensorId,
    status: CameraStatus,
    slv_addr: u8,
    pixformat: PixelFormat,
}

pub type GC0308 = Gc0308;

impl Default for Gc0308 {
    fn default() -> Self {
        Self::new()
    }
}

impl Gc0308 {
    /// Creates a new [`Gc0308`] instance with default SCCB address (`0x21`) and PID (`0x9B`).
    pub const fn new() -> Self {
        Self {
            id: SensorId {
                midh: 0,
                midl: 0,
                pid: CameraPid::GC0308.as_u16(),
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
            slv_addr: CameraSccbAddr::GC0308,
            pixformat: PixelFormat::Rgb565,
        }
    }

    /// Creates a new [`Gc0308`] with an explicit slave address and [`SensorId`].
    pub const fn with_addr(slv_addr: u8, id: SensorId) -> Self {
        Self {
            id,
            slv_addr,
            ..Self::new()
        }
    }

    /// Probes the SCCB bus at `slv_addr` (`0x21`), selects page 0, and verifies `GC0308_PID` (`0x9B`).
    pub fn detect<B: SccbBus>(bus: &mut B, slv_addr: u8) -> Result<SensorId, CameraError> {
        if slv_addr != CameraSccbAddr::GC0308 {
            return Err(CameraError::NotDetected);
        }
        bus.write_reg8(slv_addr, 0xFE, 0x00)?;
        let pid = bus.read_reg8(slv_addr, 0x00)? as u16;
        if pid == CameraPid::GC0308.as_u16() {
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

    /// Returns the 7-bit SCCB slave address (`0x21`).
    pub const fn slv_addr(&self) -> u8 {
        self.slv_addr
    }

    /// Returns the currently configured [`PixelFormat`].
    pub const fn pixformat(&self) -> PixelFormat {
        self.pixformat
    }

    /// Returns the [`CameraModel`] (`CameraModel::Gc0308`).
    pub const fn model(&self) -> CameraModel {
        CameraModel::Gc0308
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

impl<B: SccbBus> CameraSensor<B> for Gc0308 {
    fn model(&self) -> CameraModel {
        CameraModel::Gc0308
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
        bus.write_reg8(self.slv_addr, RESET_RELATED, 0xF0)?;
        bus.delay_ms(80);
        write_reg_list(bus, self.slv_addr, GC0308_SENSOR_DEFAULT_REGS)?;
        bus.delay_ms(80);
        bus.write_reg8(self.slv_addr, 0xFE, 0x00)?;
        set_reg_bits(bus, self.slv_addr, PCLK_DIV, 4, 0x07, 1)
    }

    fn init_status(&mut self, bus: &mut B) -> Result<(), CameraError> {
        bus.write_reg8(self.slv_addr, 0xFE, 0x00)?;

        let contrast_raw = bus.read_reg8(self.slv_addr, CONTRAST)? as i32;
        self.status.contrast = ((contrast_raw - 0x40) / 0x10).clamp(-2, 2) as i8;

        let sat_raw = bus.read_reg8(self.slv_addr, SATURATION_CB)? as i32;
        self.status.saturation = ((sat_raw - 0x40) / 0x10).clamp(-2, 2) as i8;

        let sharp_raw = bus.read_reg8(self.slv_addr, EDGE12_EFFECT)? as i32;
        self.status.sharpness = ((sharp_raw - 0x38) / 0x08).clamp(-2, 2) as i8;

        self.status.denoise = 0;

        let ae_raw = bus.read_reg8(self.slv_addr, AEC_TARGET_Y)? as i32;
        self.status.ae_level = ((ae_raw - 0x48) / 0x08).clamp(-2, 2) as i8;

        self.status.gainceiling = 0;
        self.status.awb = check_reg_mask(bus, self.slv_addr, AAAA_EN, AWB_ENABLE)?;
        self.status.agc = check_reg_mask(bus, self.slv_addr, AAAA_EN, AGC_ENABLE)?;
        self.status.aec = check_reg_mask(bus, self.slv_addr, AAAA_EN, AEC_ENABLE)?;
        self.status.dcw = 0;
        self.status.hmirror = check_reg_mask(bus, self.slv_addr, CISCTL_MODE1, HMIRROR_MASK)?;
        self.status.vflip = check_reg_mask(bus, self.slv_addr, CISCTL_MODE1, VFLIP_MASK)?;
        self.status.colorbar = check_reg_mask(bus, self.slv_addr, OUT_CTRL, 0x01)?;
        self.status.bpc = 0;
        self.status.wpc = 0;
        self.status.raw_gma = 0;
        self.status.lenc = 0;
        self.status.quality = 0;

        let effect_bits = bus.read_reg8(self.slv_addr, SPECIAL_EFFECT)? & 0x03;
        self.status.special_effect = match effect_bits {
            EFFECT_NORMAL => 0,
            EFFECT_NEGATIVE => 1,
            EFFECT_GRAYSCALE => 2,
            _ => 0,
        };

        if self.status.awb != 0 {
            self.status.wb_mode = 0;
        } else {
            let r = bus.read_reg8(self.slv_addr, AWB_R_GAIN)?;
            let g = bus.read_reg8(self.slv_addr, AWB_G_GAIN)?;
            let b = bus.read_reg8(self.slv_addr, AWB_B_GAIN)?;
            if (0x70..=0x78).contains(&r)
                && (0x4E..=0x56).contains(&g)
                && (0x3C..=0x44).contains(&b)
            {
                self.status.wb_mode = 1;
            } else if (0x88..=0x90).contains(&r)
                && (0x4C..=0x54).contains(&g)
                && (0x3C..=0x44).contains(&b)
            {
                self.status.wb_mode = 2;
            } else if (0x44..=0x4C).contains(&r)
                && (0x3C..=0x44).contains(&g)
                && (0x58..=0x60).contains(&b)
            {
                self.status.wb_mode = 3;
            } else if (0x3C..=0x44).contains(&r)
                && (0x3E..=0x46).contains(&g)
                && (0x4C..=0x54).contains(&b)
            {
                self.status.wb_mode = 4;
            } else {
                self.status.wb_mode = 0;
            }
        }

        self.status.awb_gain = 0;
        let gain_raw = bus.read_reg8(self.slv_addr, GLOBAL_GAIN)? as u16;
        self.status.agc_gain = ((gain_raw * 30) / 63).min(30) as u8;

        let exp_high = bus.read_reg8(self.slv_addr, EXPOSURE_HIGH)? as u16;
        let exp_low = bus.read_reg8(self.slv_addr, EXPOSURE_LOW)? as u16;
        self.status.aec_value = ((exp_high << 4) | ((exp_low >> 4) & 0x0F)).min(1200);
        self.status.aec2 = 0;

        Ok(())
    }

    fn set_pixformat(&mut self, bus: &mut B, pixformat: PixelFormat) -> Result<(), CameraError> {
        match pixformat {
            PixelFormat::Rgb565 => {
                bus.write_reg8(self.slv_addr, 0xFE, 0x00)?;
                set_reg_bits(bus, self.slv_addr, OUTPUT_FMT, 0, 0x0F, 6)?;
            }
            PixelFormat::Yuv422 => {
                bus.write_reg8(self.slv_addr, 0xFE, 0x00)?;
                set_reg_bits(bus, self.slv_addr, OUTPUT_FMT, 0, 0x0F, 2)?;
            }
            PixelFormat::Grayscale => {
                bus.write_reg8(self.slv_addr, 0xFE, 0x00)?;
                bus.write_reg8(self.slv_addr, OUTPUT_FMT, 0xB1)?;
            }
            _ => return Err(CameraError::NotSupported),
        }
        self.pixformat = pixformat;
        Ok(())
    }

    fn set_framesize(&mut self, bus: &mut B, framesize: FrameSize) -> Result<(), CameraError> {
        let fs = if framesize > FrameSize::Vga {
            FrameSize::Vga
        } else {
            framesize
        };
        self.status.framesize = fs;
        let w = fs.width();
        let h = fs.height();
        let vga_w = FrameSize::Vga.width();
        let vga_h = FrameSize::Vga.height();

        let mut win_w = vga_w;
        let mut win_h = vga_h;
        let mut row_s = (vga_h - h) / 2;
        let mut col_s = (vga_w - w) / 2;
        let mut chosen = SUBSAMPLE_CFGS[SUBSAMPLE_CFGS.len() - 1];

        for cfg in SUBSAMPLE_CFGS {
            if (u32::from(win_w) * u32::from(cfg.ratio_numerator)
                / u32::from(cfg.ratio_denominator))
                >= u32::from(w)
                && (u32::from(win_h) * u32::from(cfg.ratio_numerator)
                    / u32::from(cfg.ratio_denominator))
                    >= u32::from(h)
            {
                win_w = (u32::from(w) * u32::from(cfg.ratio_denominator)
                    / u32::from(cfg.ratio_numerator)) as u16;
                win_h = (u32::from(h) * u32::from(cfg.ratio_denominator)
                    / u32::from(cfg.ratio_numerator)) as u16;
                row_s = (vga_h.saturating_sub(win_h)) / 2;
                col_s = (vga_w.saturating_sub(win_w)) / 2;
                chosen = cfg;
                break;
            }
        }

        bus.write_reg8(self.slv_addr, 0xFE, 0x00)?;
        bus.write_reg8(self.slv_addr, ROW_START_H, h8(row_s))?;
        bus.write_reg8(self.slv_addr, ROW_START_L, l8(row_s))?;
        bus.write_reg8(self.slv_addr, COL_START_H, h8(col_s))?;
        bus.write_reg8(self.slv_addr, COL_START_L, l8(col_s))?;
        bus.write_reg8(self.slv_addr, WIN_HEIGHT_H, h8(win_h + 8))?;
        bus.write_reg8(self.slv_addr, WIN_HEIGHT_L, l8(win_h + 8))?;
        bus.write_reg8(self.slv_addr, WIN_WIDTH_H, h8(win_w + 8))?;
        bus.write_reg8(self.slv_addr, WIN_WIDTH_L, l8(win_w + 8))?;

        bus.write_reg8(self.slv_addr, 0xFE, 0x01)?;
        set_reg_bits(bus, self.slv_addr, SUBSAMPLE_EN, 7, 0x01, 1)?;
        set_reg_bits(bus, self.slv_addr, SUBSAMPLE_EN2, 0, 0x01, 1)?;
        bus.write_reg8(self.slv_addr, SUBSAMPLE_MODE, chosen.reg0x54)?;
        bus.write_reg8(self.slv_addr, SUBSAMPLE_Y0, chosen.reg0x56)?;
        bus.write_reg8(self.slv_addr, SUBSAMPLE_Y1, chosen.reg0x57)?;
        bus.write_reg8(self.slv_addr, SUBSAMPLE_UV0, chosen.reg0x58)?;
        bus.write_reg8(self.slv_addr, SUBSAMPLE_UV1, chosen.reg0x59)?;

        bus.write_reg8(self.slv_addr, 0xFE, 0x00)
    }

    fn set_contrast(&mut self, bus: &mut B, level: i32) -> Result<(), CameraError> {
        if !(-2..=2).contains(&level) {
            return Err(CameraError::InvalidArg);
        }
        bus.write_reg8(self.slv_addr, 0xFE, 0x00)?;
        let contrast_val = (0x40 + (level * 0x10)) as u8;
        bus.write_reg8(self.slv_addr, CONTRAST, contrast_val)?;
        self.status.contrast = level as i8;
        Ok(())
    }

    fn set_saturation(&mut self, bus: &mut B, level: i32) -> Result<(), CameraError> {
        if !(-2..=2).contains(&level) {
            return Err(CameraError::InvalidArg);
        }
        bus.write_reg8(self.slv_addr, 0xFE, 0x00)?;
        let sat_val = (0x40 + (level * 0x10)) as u8;
        bus.write_reg8(self.slv_addr, SATURATION_CB, sat_val)?;
        bus.write_reg8(self.slv_addr, SATURATION_CR, sat_val)?;
        self.status.saturation = level as i8;
        Ok(())
    }

    fn set_sharpness(&mut self, bus: &mut B, level: i32) -> Result<(), CameraError> {
        if !(-2..=2).contains(&level) {
            return Err(CameraError::InvalidArg);
        }
        bus.write_reg8(self.slv_addr, 0xFE, 0x00)?;
        let edge_effect = (0x38 + (level * 0x08)) as u8;
        let edge_ratio = (0x88 + (level * 0x08)) as u8;
        bus.write_reg8(self.slv_addr, EDGE12_EFFECT, edge_effect)?;
        bus.write_reg8(self.slv_addr, EDGE_POS_RATIO, edge_ratio)?;
        self.status.sharpness = level as i8;
        Ok(())
    }

    fn set_colorbar(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        bus.write_reg8(self.slv_addr, 0xFE, 0x00)?;
        set_reg_bits(bus, self.slv_addr, OUT_CTRL, 0, 0x01, u8::from(enable))?;
        self.status.colorbar = u8::from(enable);
        Ok(())
    }

    fn set_whitebal(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        bus.write_reg8(self.slv_addr, 0xFE, 0x00)?;
        set_reg_bits(bus, self.slv_addr, AAAA_EN, 1, 0x01, u8::from(enable))?;
        self.status.awb = u8::from(enable);
        Ok(())
    }

    fn set_gain_ctrl(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        bus.write_reg8(self.slv_addr, 0xFE, 0x00)?;
        set_reg_bits(bus, self.slv_addr, AAAA_EN, 2, 0x01, u8::from(enable))?;
        self.status.agc = u8::from(enable);
        Ok(())
    }

    fn set_exposure_ctrl(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        bus.write_reg8(self.slv_addr, 0xFE, 0x00)?;
        set_reg_bits(bus, self.slv_addr, AAAA_EN, 0, 0x01, u8::from(enable))?;
        self.status.aec = u8::from(enable);
        Ok(())
    }

    fn set_hmirror(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        bus.write_reg8(self.slv_addr, 0xFE, 0x00)?;
        set_reg_bits(bus, self.slv_addr, CISCTL_MODE1, 0, 0x01, u8::from(enable))?;
        self.status.hmirror = u8::from(enable);
        Ok(())
    }

    fn set_vflip(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        bus.write_reg8(self.slv_addr, 0xFE, 0x00)?;
        set_reg_bits(bus, self.slv_addr, CISCTL_MODE1, 1, 0x01, u8::from(enable))?;
        self.status.vflip = u8::from(enable);
        Ok(())
    }

    fn set_agc_gain(&mut self, bus: &mut B, gain: i32) -> Result<(), CameraError> {
        let clamped = gain.clamp(0, 30);
        let gain_value = ((clamped * 63) / 30) as u8;
        bus.write_reg8(self.slv_addr, 0xFE, 0x00)?;
        bus.write_reg8(self.slv_addr, GLOBAL_GAIN, gain_value)?;
        self.status.agc_gain = clamped as u8;
        Ok(())
    }

    fn set_aec_value(&mut self, bus: &mut B, gain: i32) -> Result<(), CameraError> {
        let clamped = gain.clamp(0, 1200) as u16;
        let exp_high = ((clamped >> 4) & 0xFF) as u8;
        let exp_low = ((clamped << 4) & 0xF0) as u8;
        bus.write_reg8(self.slv_addr, 0xFE, 0x00)?;
        bus.write_reg8(self.slv_addr, EXPOSURE_HIGH, exp_high)?;
        bus.write_reg8(self.slv_addr, EXPOSURE_LOW, exp_low)?;
        self.status.aec_value = clamped;
        Ok(())
    }

    fn set_special_effect(&mut self, bus: &mut B, effect: i32) -> Result<(), CameraError> {
        if !(0..=6).contains(&effect) {
            return Err(CameraError::InvalidArg);
        }
        bus.write_reg8(self.slv_addr, 0xFE, 0x00)?;
        let mode = match effect {
            0 => EFFECT_NORMAL,
            1 => EFFECT_NEGATIVE,
            _ => EFFECT_GRAYSCALE,
        };
        set_reg_bits(bus, self.slv_addr, SPECIAL_EFFECT, 0, 0x03, mode)?;
        self.status.special_effect = effect as u8;
        Ok(())
    }

    fn set_wb_mode(&mut self, bus: &mut B, mode: i32) -> Result<(), CameraError> {
        if !(0..=4).contains(&mode) {
            return Err(CameraError::InvalidArg);
        }
        bus.write_reg8(self.slv_addr, 0xFE, 0x00)?;
        if mode == 0 {
            set_reg_bits(bus, self.slv_addr, AAAA_EN, 1, 0x01, 1)?;
            bus.write_reg8(self.slv_addr, AWB_R_GAIN, 0x56)?;
            bus.write_reg8(self.slv_addr, AWB_G_GAIN, 0x40)?;
            bus.write_reg8(self.slv_addr, AWB_B_GAIN, 0x4A)?;
        } else {
            set_reg_bits(bus, self.slv_addr, AAAA_EN, 1, 0x01, 0)?;
            let (r, g, b) = match mode {
                1 => (0x74, 0x52, 0x40),
                2 => (0x8C, 0x50, 0x40),
                3 => (0x48, 0x40, 0x5C),
                _ => (0x40, 0x42, 0x50),
            };
            bus.write_reg8(self.slv_addr, AWB_R_GAIN, r)?;
            bus.write_reg8(self.slv_addr, AWB_G_GAIN, g)?;
            bus.write_reg8(self.slv_addr, AWB_B_GAIN, b)?;
        }
        self.status.wb_mode = mode as u8;
        Ok(())
    }

    fn set_ae_level(&mut self, bus: &mut B, level: i32) -> Result<(), CameraError> {
        if !(-2..=2).contains(&level) {
            return Err(CameraError::InvalidArg);
        }
        bus.write_reg8(self.slv_addr, 0xFE, 0x00)?;
        let ae_target = (0x48 + (level * 0x08)) as u8;
        bus.write_reg8(self.slv_addr, AEC_TARGET_Y, ae_target)?;
        self.status.ae_level = level as i8;
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
}
