//! GC032A VGA YUV422/RGB565 CMOS image sensor driver.
//!
//! Ported from `sensors/gc032a.c`, `sensors/private_include/gc032a_regs.h`, and
//! `sensors/private_include/gc032a_settings.h`.

use crate::{
    CameraError, CameraModel, CameraPid, CameraSccbAddr, CameraSensor, CameraStatus, FrameSize,
    PixelFormat, Reg16, SccbBus, SensorId,
};

pub const SENSOR_ID_HIGH: u8 = 0xF0;
pub const SENSOR_ID_LOW: u8 = 0xF1;
pub const PAD_VB_HIZ_MODE: u8 = 0xF2;
pub const SYNC_OUTPUT: u8 = 0xF3;
pub const I2C_CONFIG: u8 = 0xF4;
pub const PLL_MODE1: u8 = 0xF7;
pub const PLL_MODE2: u8 = 0xF8;
pub const CM_MODE: u8 = 0xF9;
pub const ISP_DIV_MODE: u8 = 0xFA;
pub const I2C_DEVICE_ID: u8 = 0xFB;
pub const ANALOG_PWC: u8 = 0xFC;
pub const ISP_DIV_MODE2: u8 = 0xFD;
pub const RESET_RELATED: u8 = 0xFE;

pub const P0_EXPOSURE_HIGH: u8 = 0x03;
pub const P0_EXPOSURE_LOW: u8 = 0x04;
pub const P0_HB_HIGH: u8 = 0x05;
pub const P0_HB_LOW: u8 = 0x06;
pub const P0_VB_HIGH: u8 = 0x07;
pub const P0_VB_LOW: u8 = 0x08;
pub const P0_ROW_START_HIGH: u8 = 0x09;
pub const P0_ROW_START_LOW: u8 = 0x0A;
pub const P0_COLUMN_START_HIGH: u8 = 0x0B;
pub const P0_COLUMN_START_LOW: u8 = 0x0C;
pub const P0_WINDOW_HEIGHT_HIGH: u8 = 0x0D;
pub const P0_WINDOW_HEIGHT_LOW: u8 = 0x0E;
pub const P0_WINDOW_WIDTH_HIGH: u8 = 0x0F;
pub const P0_WINDOW_WIDTH_LOW: u8 = 0x10;
pub const P0_SH_DELAY: u8 = 0x11;
pub const P0_VS_ST: u8 = 0x12;
pub const P0_VS_ET: u8 = 0x13;
pub const P0_CISCTL_MODE1: u8 = 0x17;
pub const P0_BLOCK_ENABLE_1: u8 = 0x40;
pub const P0_AAAA_ENABLE: u8 = 0x42;
pub const P0_SPECIAL_EFFECT: u8 = 0x43;
pub const P0_SYNC_MODE: u8 = 0x46;
pub const P0_GAIN_CODE: u8 = 0x48;
pub const P0_DEBUG_MODE2: u8 = 0x4C;
pub const P0_WIN_MODE: u8 = 0x50;
pub const P0_OUT_WIN_Y1_HIGH: u8 = 0x51;
pub const P0_OUT_WIN_Y1_LOW: u8 = 0x52;
pub const P0_OUT_WIN_X1_HIGH: u8 = 0x53;
pub const P0_OUT_WIN_X1_LOW: u8 = 0x54;
pub const P0_OUT_WIN_HEIGHT_HIGH: u8 = 0x55;
pub const P0_OUT_WIN_HEIGHT_LOW: u8 = 0x56;
pub const P0_OUT_WIN_WIDTH_HIGH: u8 = 0x57;
pub const P0_OUT_WIN_WIDTH_LOW: u8 = 0x58;
pub const P0_GLOBAL_SATURATION: u8 = 0xD0;
pub const P0_SATURATION_CB: u8 = 0xD1;
pub const P0_SATURATION_CR: u8 = 0xD2;
pub const P0_LUMA_CONTRAST: u8 = 0xD3;
pub const P0_CONTRAST_CENTER: u8 = 0xD4;
pub const P0_LUMA_OFFSET: u8 = 0xD5;
pub const P0_FIXED_CB: u8 = 0xDA;
pub const P0_FIXED_CR: u8 = 0xDB;

pub const P3_IMAGE_WIDTH_LOW: u8 = 0x5B;
pub const P3_IMAGE_WIDTH_HIGH: u8 = 0x5C;
pub const P3_IMAGE_HEIGHT_LOW: u8 = 0x5D;
pub const P3_IMAGE_HEIGHT_HIGH: u8 = 0x5E;

pub const REG_DLY: u16 = 0xFFFF;
pub const REGLIST_TAIL: u16 = 0x0000;

pub const GC032A_DEFAULT_REGS: &[Reg16] = &[
    Reg16::new(0xF3, 0xFF),
    Reg16::new(0xF5, 0x06),
    Reg16::new(0xF7, 0x01),
    Reg16::new(0xF8, 0x03),
    Reg16::new(0xF9, 0xCE),
    Reg16::new(0xFA, 0x00),
    Reg16::new(0xFC, 0x02),
    Reg16::new(0xFE, 0x02),
    Reg16::new(0x81, 0x03),
    Reg16::new(0xFE, 0x00),
    Reg16::new(0x77, 0x64),
    Reg16::new(0x78, 0x40),
    Reg16::new(0x79, 0x60),
    Reg16::new(0xFE, 0x00),
    Reg16::new(0x03, 0x01),
    Reg16::new(0x04, 0xCE),
    Reg16::new(0x05, 0x01),
    Reg16::new(0x06, 0xAD),
    Reg16::new(0x07, 0x00),
    Reg16::new(0x08, 0x10),
    Reg16::new(0x0A, 0x00),
    Reg16::new(0x0C, 0x00),
    Reg16::new(0x0D, 0x01),
    Reg16::new(0x0E, 0xE8),
    Reg16::new(0x0F, 0x02),
    Reg16::new(0x10, 0x88),
    Reg16::new(0x17, 0x54),
    Reg16::new(0x19, 0x08),
    Reg16::new(0x1A, 0x0A),
    Reg16::new(0x1F, 0x40),
    Reg16::new(0x20, 0x30),
    Reg16::new(0x2E, 0x80),
    Reg16::new(0x2F, 0x2B),
    Reg16::new(0x30, 0x1A),
    Reg16::new(0xFE, 0x02),
    Reg16::new(0x03, 0x02),
    Reg16::new(0x05, 0xD7),
    Reg16::new(0x06, 0x60),
    Reg16::new(0x08, 0x80),
    Reg16::new(0x12, 0x89),
    Reg16::new(0xFE, 0x00),
    Reg16::new(0x18, 0x02),
    Reg16::new(0xFE, 0x02),
    Reg16::new(0x40, 0x22),
    Reg16::new(0x45, 0x00),
    Reg16::new(0x46, 0x00),
    Reg16::new(0x49, 0x20),
    Reg16::new(0x4B, 0x3C),
    Reg16::new(0x50, 0x20),
    Reg16::new(0x42, 0x10),
    Reg16::new(0xFE, 0x01),
    Reg16::new(0x0A, 0xC5),
    Reg16::new(0x45, 0x00),
    Reg16::new(0xFE, 0x00),
    Reg16::new(0x40, 0xFF),
    Reg16::new(0x41, 0x25),
    Reg16::new(0x42, 0xCF),
    Reg16::new(0x43, 0x10),
    Reg16::new(0x44, 0x83),
    Reg16::new(0x46, 0x23),
    Reg16::new(0x49, 0x03),
    Reg16::new(0x52, 0x02),
    Reg16::new(0x54, 0x00),
    Reg16::new(0xFE, 0x02),
    Reg16::new(0x22, 0xF6),
    Reg16::new(0xFE, 0x01),
    Reg16::new(0xC1, 0x38),
    Reg16::new(0xC2, 0x4C),
    Reg16::new(0xC3, 0x00),
    Reg16::new(0xC4, 0x32),
    Reg16::new(0xC5, 0x24),
    Reg16::new(0xC6, 0x16),
    Reg16::new(0xC7, 0x08),
    Reg16::new(0xC8, 0x08),
    Reg16::new(0xC9, 0x00),
    Reg16::new(0xCA, 0x20),
    Reg16::new(0xDC, 0x8A),
    Reg16::new(0xDD, 0xA0),
    Reg16::new(0xDE, 0xA6),
    Reg16::new(0xDF, 0x75),
    Reg16::new(0xFE, 0x01),
    Reg16::new(0x7C, 0x09),
    Reg16::new(0x65, 0x06),
    Reg16::new(0x7C, 0x08),
    Reg16::new(0x56, 0xF4),
    Reg16::new(0x66, 0x0F),
    Reg16::new(0x67, 0x84),
    Reg16::new(0x6B, 0x80),
    Reg16::new(0x6D, 0x12),
    Reg16::new(0x6E, 0xB0),
    Reg16::new(0x86, 0x00),
    Reg16::new(0x87, 0x00),
    Reg16::new(0x88, 0x00),
    Reg16::new(0x89, 0x00),
    Reg16::new(0x8A, 0x00),
    Reg16::new(0x8B, 0x00),
    Reg16::new(0x8C, 0x00),
    Reg16::new(0x8D, 0x00),
    Reg16::new(0x8E, 0x00),
    Reg16::new(0x8F, 0x00),
    Reg16::new(0x90, 0x00),
    Reg16::new(0x91, 0x00),
    Reg16::new(0x92, 0xF4),
    Reg16::new(0x93, 0xD5),
    Reg16::new(0x94, 0x50),
    Reg16::new(0x95, 0x0F),
    Reg16::new(0x96, 0xF4),
    Reg16::new(0x97, 0x2D),
    Reg16::new(0x98, 0x0F),
    Reg16::new(0x99, 0xA6),
    Reg16::new(0x9A, 0x2D),
    Reg16::new(0x9B, 0x0F),
    Reg16::new(0x9C, 0x59),
    Reg16::new(0x9D, 0x2D),
    Reg16::new(0x9E, 0xAA),
    Reg16::new(0x9F, 0x67),
    Reg16::new(0xA0, 0x59),
    Reg16::new(0xA1, 0x00),
    Reg16::new(0xA2, 0x00),
    Reg16::new(0xA3, 0x0A),
    Reg16::new(0xA4, 0x00),
    Reg16::new(0xA5, 0x00),
    Reg16::new(0xA6, 0xD4),
    Reg16::new(0xA7, 0x9F),
    Reg16::new(0xA8, 0x55),
    Reg16::new(0xA9, 0xD4),
    Reg16::new(0xAA, 0x9F),
    Reg16::new(0xAB, 0xAC),
    Reg16::new(0xAC, 0x9F),
    Reg16::new(0xAD, 0x55),
    Reg16::new(0xAE, 0xD4),
    Reg16::new(0xAF, 0xAC),
    Reg16::new(0xB0, 0xD4),
    Reg16::new(0xB1, 0xA3),
    Reg16::new(0xB2, 0x55),
    Reg16::new(0xB3, 0xD4),
    Reg16::new(0xB4, 0xAC),
    Reg16::new(0xB5, 0x00),
    Reg16::new(0xB6, 0x00),
    Reg16::new(0xB7, 0x05),
    Reg16::new(0xB8, 0xD6),
    Reg16::new(0xB9, 0x8C),
    Reg16::new(0xFE, 0x01),
    Reg16::new(0xD0, 0x40),
    Reg16::new(0xD1, 0xF8),
    Reg16::new(0xD2, 0x00),
    Reg16::new(0xD3, 0xFA),
    Reg16::new(0xD4, 0x45),
    Reg16::new(0xD5, 0x02),
    Reg16::new(0xD6, 0x30),
    Reg16::new(0xD7, 0xFA),
    Reg16::new(0xD8, 0x08),
    Reg16::new(0xD9, 0x08),
    Reg16::new(0xDA, 0x58),
    Reg16::new(0xDB, 0x02),
    Reg16::new(0xFE, 0x00),
    Reg16::new(0xFE, 0x00),
    Reg16::new(0xBA, 0x00),
    Reg16::new(0xBB, 0x04),
    Reg16::new(0xBC, 0x0A),
    Reg16::new(0xBD, 0x0E),
    Reg16::new(0xBE, 0x22),
    Reg16::new(0xBF, 0x30),
    Reg16::new(0xC0, 0x3D),
    Reg16::new(0xC1, 0x4A),
    Reg16::new(0xC2, 0x5D),
    Reg16::new(0xC3, 0x6B),
    Reg16::new(0xC4, 0x7A),
    Reg16::new(0xC5, 0x85),
    Reg16::new(0xC6, 0x90),
    Reg16::new(0xC7, 0xA5),
    Reg16::new(0xC8, 0xB5),
    Reg16::new(0xC9, 0xC2),
    Reg16::new(0xCA, 0xCC),
    Reg16::new(0xCB, 0xD5),
    Reg16::new(0xCC, 0xDE),
    Reg16::new(0xCD, 0xEA),
    Reg16::new(0xCE, 0xF5),
    Reg16::new(0xCF, 0xFF),
    Reg16::new(0xFE, 0x00),
    Reg16::new(0x5A, 0x08),
    Reg16::new(0x5B, 0x0F),
    Reg16::new(0x5C, 0x15),
    Reg16::new(0x5D, 0x1C),
    Reg16::new(0x5E, 0x28),
    Reg16::new(0x5F, 0x36),
    Reg16::new(0x60, 0x45),
    Reg16::new(0x61, 0x51),
    Reg16::new(0x62, 0x6A),
    Reg16::new(0x63, 0x7D),
    Reg16::new(0x64, 0x8D),
    Reg16::new(0x65, 0x98),
    Reg16::new(0x66, 0xA2),
    Reg16::new(0x67, 0xB5),
    Reg16::new(0x68, 0xC3),
    Reg16::new(0x69, 0xCD),
    Reg16::new(0x6A, 0xD4),
    Reg16::new(0x6B, 0xDC),
    Reg16::new(0x6C, 0xE3),
    Reg16::new(0x6D, 0xF0),
    Reg16::new(0x6E, 0xF9),
    Reg16::new(0x6F, 0xFF),
    Reg16::new(0xFE, 0x00),
    Reg16::new(0x70, 0x50),
    Reg16::new(0xFE, 0x00),
    Reg16::new(0x4F, 0x01),
    Reg16::new(0xFE, 0x01),
    Reg16::new(0x0D, 0x00),
    Reg16::new(0x12, 0xA0),
    Reg16::new(0x13, 0x3A),
    Reg16::new(0x44, 0x04),
    Reg16::new(0x1F, 0x30),
    Reg16::new(0x20, 0x40),
    Reg16::new(0x26, 0x9A),
    Reg16::new(0x3E, 0x20),
    Reg16::new(0x3F, 0x2D),
    Reg16::new(0x40, 0x40),
    Reg16::new(0x41, 0x5B),
    Reg16::new(0x42, 0x82),
    Reg16::new(0x43, 0xB7),
    Reg16::new(0x04, 0x0A),
    Reg16::new(0x02, 0x79),
    Reg16::new(0x03, 0xC0),
    Reg16::new(0xFE, 0x01),
    Reg16::new(0xCC, 0x08),
    Reg16::new(0xCD, 0x08),
    Reg16::new(0xCE, 0xA4),
    Reg16::new(0xCF, 0xEC),
    Reg16::new(0xFE, 0x00),
    Reg16::new(0x81, 0xB8),
    Reg16::new(0x82, 0x12),
    Reg16::new(0x83, 0x0A),
    Reg16::new(0x84, 0x01),
    Reg16::new(0x86, 0x50),
    Reg16::new(0x87, 0x18),
    Reg16::new(0x88, 0x10),
    Reg16::new(0x89, 0x70),
    Reg16::new(0x8A, 0x20),
    Reg16::new(0x8B, 0x10),
    Reg16::new(0x8C, 0x08),
    Reg16::new(0x8D, 0x0A),
    Reg16::new(0xFE, 0x00),
    Reg16::new(0x8F, 0xAA),
    Reg16::new(0x90, 0x9C),
    Reg16::new(0x91, 0x52),
    Reg16::new(0x92, 0x03),
    Reg16::new(0x93, 0x03),
    Reg16::new(0x94, 0x08),
    Reg16::new(0x95, 0x44),
    Reg16::new(0x97, 0x00),
    Reg16::new(0x98, 0x00),
    Reg16::new(0xFE, 0x00),
    Reg16::new(0xA1, 0x30),
    Reg16::new(0xA2, 0x41),
    Reg16::new(0xA4, 0x30),
    Reg16::new(0xA5, 0x20),
    Reg16::new(0xAA, 0x30),
    Reg16::new(0xAC, 0x32),
    Reg16::new(0xFE, 0x00),
    Reg16::new(0xD1, 0x3C),
    Reg16::new(0xD2, 0x3C),
    Reg16::new(0xD3, 0x38),
    Reg16::new(0xD6, 0xF4),
    Reg16::new(0xD7, 0x1D),
    Reg16::new(0xDD, 0x73),
    Reg16::new(0xDE, 0x84),
    Reg16::new(0xFE, 0x00),
    Reg16::new(0x05, 0x01),
    Reg16::new(0x06, 0xAD),
    Reg16::new(0x07, 0x00),
    Reg16::new(0x08, 0x10),
    Reg16::new(0xFE, 0x01),
    Reg16::new(0x25, 0x00),
    Reg16::new(0x26, 0x9A),
    Reg16::new(0x27, 0x01),
    Reg16::new(0x28, 0xCE),
    Reg16::new(0x29, 0x02),
    Reg16::new(0x2A, 0x68),
    Reg16::new(0x2B, 0x02),
    Reg16::new(0x2C, 0x68),
    Reg16::new(0x2D, 0x07),
    Reg16::new(0x2E, 0xD2),
    Reg16::new(0x2F, 0x0B),
    Reg16::new(0x30, 0x6E),
    Reg16::new(0x31, 0x0E),
    Reg16::new(0x32, 0x70),
    Reg16::new(0x33, 0x12),
    Reg16::new(0x34, 0x0C),
    Reg16::new(0x3C, 0x30),
    Reg16::new(0xFE, 0x00),
    Reg16::new(0x05, 0x01),
    Reg16::new(0x06, 0xA0),
    Reg16::new(0x07, 0x00),
    Reg16::new(0x08, 0x20),
    Reg16::new(0x0A, 0x78),
    Reg16::new(0x0C, 0xA0),
    Reg16::new(0x0D, 0x00),
    Reg16::new(0x0E, 0xF8),
    Reg16::new(0x0F, 0x01),
    Reg16::new(0x10, 0x48),
    Reg16::new(0x55, 0x00),
    Reg16::new(0x56, 0xF0),
    Reg16::new(0x57, 0x01),
    Reg16::new(0x58, 0x40),
    Reg16::new(0xFE, 0x03),
    Reg16::new(0x5B, 0x40),
    Reg16::new(0x5C, 0x01),
    Reg16::new(0x5D, 0xF0),
    Reg16::new(0x5E, 0x00),
    Reg16::new(0xFE, 0x01),
    Reg16::new(0x25, 0x00),
    Reg16::new(0x26, 0x63),
    Reg16::new(0x27, 0x01),
    Reg16::new(0x28, 0x29),
    Reg16::new(0x29, 0x01),
    Reg16::new(0x2A, 0x29),
    Reg16::new(0x2B, 0x01),
    Reg16::new(0x2C, 0x29),
    Reg16::new(0x2D, 0x01),
    Reg16::new(0x2E, 0x29),
    Reg16::new(0x2F, 0x01),
    Reg16::new(0x30, 0x29),
    Reg16::new(0x31, 0x01),
    Reg16::new(0x32, 0x29),
    Reg16::new(0x33, 0x01),
    Reg16::new(0x34, 0x29),
    Reg16::new(0x3C, 0x00),
    Reg16::new(0xFE, 0x01),
    Reg16::new(0xCC, 0x04),
    Reg16::new(0xCD, 0x04),
    Reg16::new(0xCE, 0x72),
    Reg16::new(0xCF, 0x52),
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
    regs: &[Reg16],
) -> Result<(), CameraError> {
    for entry in regs {
        if entry.reg == REGLIST_TAIL {
            break;
        }
        if entry.reg == REG_DLY {
            bus.delay_ms(entry.val as u32);
        } else {
            bus.write_reg8(slv_addr, entry.reg as u8, entry.val)?;
        }
    }
    Ok(())
}

/// Detects a GC032A sensor at `slv_addr` (`0x21`).
pub fn detect<B: SccbBus>(bus: &mut B, slv_addr: u8) -> Result<SensorId, CameraError> {
    Gc032a::detect(bus, slv_addr)
}

/// Safe `#![no_std]` driver for the GalaxyCore GC032A VGA camera sensor.
#[derive(Debug, Clone)]
pub struct Gc032a {
    id: SensorId,
    status: CameraStatus,
    slv_addr: u8,
    pixformat: PixelFormat,
}

pub type GC032A = Gc032a;
pub type Gc032A = Gc032a;

impl Default for Gc032a {
    fn default() -> Self {
        Self::new()
    }
}

impl Gc032a {
    /// Creates a new [`Gc032a`] instance with default SCCB address (`0x21`) and PID (`0x232A`).
    pub const fn new() -> Self {
        Self {
            id: SensorId {
                midh: 0,
                midl: 0,
                pid: CameraPid::GC032A.as_u16(),
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
            slv_addr: CameraSccbAddr::GC032A,
            pixformat: PixelFormat::Rgb565,
        }
    }

    /// Creates a new [`Gc032a`] with an explicit slave address and [`SensorId`].
    pub const fn with_addr(slv_addr: u8, id: SensorId) -> Self {
        Self {
            id,
            slv_addr,
            ..Self::new()
        }
    }

    /// Probes the SCCB bus at `slv_addr` (`0x21`) and verifies `GC032A_PID` (`0x232A`).
    pub fn detect<B: SccbBus>(bus: &mut B, slv_addr: u8) -> Result<SensorId, CameraError> {
        if slv_addr != CameraSccbAddr::GC032A {
            return Err(CameraError::NotDetected);
        }
        let midl = bus.read_reg8(slv_addr, SENSOR_ID_LOW)? as u16;
        let midh = bus.read_reg8(slv_addr, SENSOR_ID_HIGH)? as u16;
        let pid = (midh << 8) | midl;
        if pid == CameraPid::GC032A.as_u16() {
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

    /// Returns the [`CameraModel`] (`CameraModel::Gc032a`).
    pub const fn model(&self) -> CameraModel {
        CameraModel::Gc032a
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

impl<B: SccbBus> CameraSensor<B> for Gc032a {
    fn model(&self) -> CameraModel {
        CameraModel::Gc032a
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
        bus.delay_ms(100);
        write_reg_list(bus, self.slv_addr, GC032A_DEFAULT_REGS)?;
        bus.delay_ms(100);
        bus.write_reg8(self.slv_addr, 0xFE, 0x00)?;
        set_reg_bits(bus, self.slv_addr, 0xF7, 1, 0x01, 1)?;
        set_reg_bits(bus, self.slv_addr, 0xF7, 7, 0x01, 1)?;
        set_reg_bits(bus, self.slv_addr, 0xF8, 0, 0x3F, 8)?;
        set_reg_bits(bus, self.slv_addr, 0xFA, 4, 0x0F, 2)
    }

    fn init_status(&mut self, bus: &mut B) -> Result<(), CameraError> {
        bus.write_reg8(self.slv_addr, 0xFE, 0x00)?;
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
        self.status.hmirror = check_reg_mask(bus, self.slv_addr, P0_CISCTL_MODE1, 0x01)?;
        self.status.vflip = check_reg_mask(bus, self.slv_addr, P0_CISCTL_MODE1, 0x02)?;
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
            PixelFormat::Rgb565 => {
                bus.write_reg8(self.slv_addr, 0xFE, 0x00)?;
                set_reg_bits(bus, self.slv_addr, 0x44, 0, 0x1F, 6)?;
            }
            PixelFormat::Yuv422 => {
                bus.write_reg8(self.slv_addr, 0xFE, 0x00)?;
                set_reg_bits(bus, self.slv_addr, 0x44, 0, 0x1F, 3)?;
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
        let row_s = (FrameSize::Vga.height() - h) / 2;
        let col_s = (FrameSize::Vga.width() - w) / 2;

        bus.write_reg8(self.slv_addr, 0xFE, 0x00)?;
        bus.write_reg8(self.slv_addr, P0_ROW_START_HIGH, h8(row_s))?;
        bus.write_reg8(self.slv_addr, P0_ROW_START_LOW, l8(row_s))?;
        bus.write_reg8(self.slv_addr, P0_COLUMN_START_HIGH, h8(col_s))?;
        bus.write_reg8(self.slv_addr, P0_COLUMN_START_LOW, l8(col_s))?;
        bus.write_reg8(self.slv_addr, P0_WINDOW_HEIGHT_HIGH, h8(h + 8))?;
        bus.write_reg8(self.slv_addr, P0_WINDOW_HEIGHT_LOW, l8(h + 8))?;
        bus.write_reg8(self.slv_addr, P0_WINDOW_WIDTH_HIGH, h8(w + 8))?;
        bus.write_reg8(self.slv_addr, P0_WINDOW_WIDTH_LOW, l8(w + 8))?;

        bus.write_reg8(self.slv_addr, P0_WIN_MODE, 0x01)?;
        bus.write_reg8(self.slv_addr, P0_OUT_WIN_HEIGHT_HIGH, h8(h))?;
        bus.write_reg8(self.slv_addr, P0_OUT_WIN_HEIGHT_LOW, l8(h))?;
        bus.write_reg8(self.slv_addr, P0_OUT_WIN_WIDTH_HIGH, h8(w))?;
        bus.write_reg8(self.slv_addr, P0_OUT_WIN_WIDTH_LOW, l8(w))
    }

    fn set_colorbar(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        bus.write_reg8(self.slv_addr, 0xFE, 0x00)?;
        set_reg_bits(bus, self.slv_addr, P0_DEBUG_MODE2, 3, 0x01, u8::from(enable))?;
        self.status.colorbar = u8::from(enable);
        Ok(())
    }

    fn set_hmirror(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.status.hmirror = u8::from(enable);
        bus.write_reg8(self.slv_addr, 0xFE, 0x00)?;
        set_reg_bits(bus, self.slv_addr, P0_CISCTL_MODE1, 0, 0x01, u8::from(enable))
    }

    fn set_vflip(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.status.vflip = u8::from(enable);
        bus.write_reg8(self.slv_addr, 0xFE, 0x00)?;
        set_reg_bits(bus, self.slv_addr, P0_CISCTL_MODE1, 1, 0x01, u8::from(enable))
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
