//! GC2145 2MP UXGA YUV422/RGB565 CMOS image sensor driver.
//!
//! Ported from `sensors/gc2145.c`, `sensors/private_include/gc2145_regs.h`, and
//! `sensors/private_include/gc2145_settings.h`.

use crate::{
    CameraError, CameraModel, CameraPid, CameraSccbAddr, CameraSensor, CameraStatus, FrameSize,
    PixelFormat, Reg16, SccbBus, SensorId,
};

pub const CHIP_ID_HIGH: u8 = 0xF0;
pub const CHIP_ID_LOW: u8 = 0xF1;
pub const PLL_MODE1: u8 = 0xF7;
pub const PLL_MODE2: u8 = 0xF8;
pub const CM_MODE: u8 = 0xF9;
pub const CLK_DIV_MODE: u8 = 0xFA;
pub const RESET_RELATED: u8 = 0xFE;

pub const P0_EXPOSURE_HIGH: u8 = 0x03;
pub const P0_EXPOSURE_LOW: u8 = 0x04;
pub const P0_HB_HIGH: u8 = 0x05;
pub const P0_HB_LOW: u8 = 0x06;
pub const P0_VB_HIGH: u8 = 0x07;
pub const P0_VB_LOW: u8 = 0x08;
pub const P0_ROW_START_HIGH: u8 = 0x09;
pub const P0_ROW_START_LOW: u8 = 0x0A;
pub const P0_COL_START_HIGH: u8 = 0x0B;
pub const P0_COL_START_LOW: u8 = 0x0C;
pub const P0_WIN_HEIGHT_HIGH: u8 = 0x0D;
pub const P0_WIN_HEIGHT_LOW: u8 = 0x0E;
pub const P0_WIN_WIDTH_HIGH: u8 = 0x0F;
pub const P0_WIN_WIDTH_LOW: u8 = 0x10;
pub const P0_ANALOG_MODE1: u8 = 0x17;
pub const P0_ANALOG_MODE2: u8 = 0x18;
pub const P0_SPECIAL_EFFECT: u8 = 0x83;
pub const P0_OUTPUT_FORMAT: u8 = 0x84;
pub const P0_FRAME_START: u8 = 0x85;
pub const P0_SYNC_MODE: u8 = 0x86;
pub const P0_MODULE_GATING: u8 = 0x88;
pub const P0_BYPASS_MODE: u8 = 0x89;
pub const P0_DEBUG_MODE2: u8 = 0x8C;
pub const P0_DEBUG_MODE3: u8 = 0x8D;
pub const P0_CROP_ENABLE: u8 = 0x90;
pub const P0_OUT_WIN_Y1_HIGH: u8 = 0x91;
pub const P0_OUT_WIN_Y1_LOW: u8 = 0x92;
pub const P0_OUT_WIN_X1_HIGH: u8 = 0x93;
pub const P0_OUT_WIN_X1_LOW: u8 = 0x94;
pub const P0_OUT_WIN_HEIGHT_HIGH: u8 = 0x95;
pub const P0_OUT_WIN_HEIGHT_LOW: u8 = 0x96;
pub const P0_OUT_WIN_WIDTH_HIGH: u8 = 0x97;
pub const P0_OUT_WIN_WIDTH_LOW: u8 = 0x98;
pub const P0_SUBSAMPLE: u8 = 0x99;
pub const P0_SUBSAMPLE_MODE: u8 = 0x9A;

pub const REG_DLY: u16 = 0xFFFF;
pub const REGLIST_TAIL: u16 = 0x0000;

pub const GC2145_DEFAULT_INIT_REGS: &[Reg16] = &[
    Reg16::new(0xFE, 0xF0),
    Reg16::new(0xFE, 0xF0),
    Reg16::new(0xFE, 0xF0),
    Reg16::new(0xFC, 0x06),
    Reg16::new(0xF6, 0x00),
    Reg16::new(0xF7, 0x1D),
    Reg16::new(0xF8, 0x83),
    Reg16::new(0xFA, 0x00),
    Reg16::new(0xF9, 0xFE),
    Reg16::new(0xFD, 0x00),
    Reg16::new(0xC2, 0x00),
    Reg16::new(0xF2, 0x0F),
    Reg16::new(0xFE, 0x00),
    Reg16::new(0x03, 0x04),
    Reg16::new(0x04, 0x62),
    Reg16::new(0x05, 0x01),
    Reg16::new(0x06, 0x3B),
    Reg16::new(0x09, 0x00),
    Reg16::new(0x0A, 0x00),
    Reg16::new(0x0B, 0x00),
    Reg16::new(0x0C, 0x00),
    Reg16::new(0x0D, 0x04),
    Reg16::new(0x0E, 0xC0),
    Reg16::new(0x0F, 0x06),
    Reg16::new(0x10, 0x52),
    Reg16::new(0x12, 0x2E),
    Reg16::new(0x17, 0x14),
    Reg16::new(0x18, 0x22),
    Reg16::new(0x19, 0x0F),
    Reg16::new(0x1A, 0x01),
    Reg16::new(0x1B, 0x4B),
    Reg16::new(0x1C, 0x07),
    Reg16::new(0x1D, 0x10),
    Reg16::new(0x1E, 0x88),
    Reg16::new(0x1F, 0x78),
    Reg16::new(0x20, 0x03),
    Reg16::new(0x21, 0x40),
    Reg16::new(0x22, 0xA0),
    Reg16::new(0x24, 0x1E),
    Reg16::new(0x25, 0x01),
    Reg16::new(0x26, 0x10),
    Reg16::new(0x2D, 0x60),
    Reg16::new(0x30, 0x01),
    Reg16::new(0x31, 0x90),
    Reg16::new(0x33, 0x06),
    Reg16::new(0x34, 0x01),
    Reg16::new(0x80, 0xFF),
    Reg16::new(0x81, 0x24),
    Reg16::new(0x82, 0xFA),
    Reg16::new(0x83, 0x00),
    Reg16::new(0x84, 0x02),
    Reg16::new(0x86, 0x03),
    Reg16::new(0x88, 0x03),
    Reg16::new(0x89, 0x03),
    Reg16::new(0x85, 0x30),
    Reg16::new(0x8A, 0x00),
    Reg16::new(0x8B, 0x00),
    Reg16::new(0xB0, 0x55),
    Reg16::new(0xC3, 0x00),
    Reg16::new(0xC4, 0x80),
    Reg16::new(0xC5, 0x90),
    Reg16::new(0xC6, 0x38),
    Reg16::new(0xC7, 0x40),
    Reg16::new(0xEC, 0x06),
    Reg16::new(0xED, 0x04),
    Reg16::new(0xEE, 0x60),
    Reg16::new(0xEF, 0x90),
    Reg16::new(0xB6, 0x01),
    Reg16::new(0x90, 0x01),
    Reg16::new(0x91, 0x00),
    Reg16::new(0x92, 0x00),
    Reg16::new(0x93, 0x00),
    Reg16::new(0x94, 0x00),
    Reg16::new(0x95, 0x04),
    Reg16::new(0x96, 0xB0),
    Reg16::new(0x97, 0x06),
    Reg16::new(0x98, 0x40),
    Reg16::new(0x18, 0x02),
    Reg16::new(0x40, 0x42),
    Reg16::new(0x41, 0x00),
    Reg16::new(0x43, 0x54),
    Reg16::new(0x5E, 0x00),
    Reg16::new(0x5F, 0x00),
    Reg16::new(0x60, 0x00),
    Reg16::new(0x61, 0x00),
    Reg16::new(0x62, 0x00),
    Reg16::new(0x63, 0x00),
    Reg16::new(0x64, 0x00),
    Reg16::new(0x65, 0x00),
    Reg16::new(0x66, 0x20),
    Reg16::new(0x67, 0x20),
    Reg16::new(0x68, 0x20),
    Reg16::new(0x69, 0x20),
    Reg16::new(0x76, 0x00),
    Reg16::new(0x6A, 0x00),
    Reg16::new(0x6B, 0x00),
    Reg16::new(0x6C, 0x3E),
    Reg16::new(0x6D, 0x3E),
    Reg16::new(0x6E, 0x3F),
    Reg16::new(0x6F, 0x3F),
    Reg16::new(0x70, 0x00),
    Reg16::new(0x71, 0x00),
    Reg16::new(0x76, 0x00),
    Reg16::new(0x72, 0xF0),
    Reg16::new(0x7E, 0x3C),
    Reg16::new(0x7F, 0x00),
    Reg16::new(0xFE, 0x02),
    Reg16::new(0x48, 0x15),
    Reg16::new(0x49, 0x00),
    Reg16::new(0x4B, 0x0B),
    Reg16::new(0xFE, 0x00),
    Reg16::new(0xFE, 0x01),
    Reg16::new(0x01, 0x04),
    Reg16::new(0x02, 0xC0),
    Reg16::new(0x03, 0x04),
    Reg16::new(0x04, 0x90),
    Reg16::new(0x05, 0x30),
    Reg16::new(0x06, 0x90),
    Reg16::new(0x07, 0x20),
    Reg16::new(0x08, 0x70),
    Reg16::new(0x09, 0x00),
    Reg16::new(0x0A, 0xC2),
    Reg16::new(0x0B, 0x11),
    Reg16::new(0x0C, 0x10),
    Reg16::new(0x13, 0x40),
    Reg16::new(0x17, 0x00),
    Reg16::new(0x1C, 0x11),
    Reg16::new(0x1E, 0x61),
    Reg16::new(0x1F, 0x30),
    Reg16::new(0x20, 0x40),
    Reg16::new(0x22, 0x80),
    Reg16::new(0x23, 0x20),
    Reg16::new(0xFE, 0x02),
    Reg16::new(0x0F, 0x04),
    Reg16::new(0xFE, 0x01),
    Reg16::new(0x12, 0x35),
    Reg16::new(0x15, 0x50),
    Reg16::new(0x10, 0x31),
    Reg16::new(0x3E, 0x28),
    Reg16::new(0x3F, 0xE0),
    Reg16::new(0x40, 0x20),
    Reg16::new(0x41, 0x0F),
    Reg16::new(0xFE, 0x02),
    Reg16::new(0x0F, 0x05),
    Reg16::new(0xFE, 0x02),
    Reg16::new(0x90, 0x6C),
    Reg16::new(0x91, 0x03),
    Reg16::new(0x92, 0xC8),
    Reg16::new(0x94, 0x66),
    Reg16::new(0x95, 0xB5),
    Reg16::new(0x97, 0x64),
    Reg16::new(0xA2, 0x11),
    Reg16::new(0xFE, 0x00),
    Reg16::new(0xFE, 0x02),
    Reg16::new(0x80, 0xC1),
    Reg16::new(0x81, 0x08),
    Reg16::new(0x82, 0x08),
    Reg16::new(0x83, 0x08),
    Reg16::new(0x84, 0x0A),
    Reg16::new(0x86, 0xF0),
    Reg16::new(0x87, 0x50),
    Reg16::new(0x88, 0x15),
    Reg16::new(0x89, 0x50),
    Reg16::new(0x8A, 0x30),
    Reg16::new(0x8B, 0x10),
    Reg16::new(0xFE, 0x01),
    Reg16::new(0x21, 0x14),
    Reg16::new(0xFE, 0x02),
    Reg16::new(0xA3, 0x40),
    Reg16::new(0xA4, 0x20),
    Reg16::new(0xA5, 0x40),
    Reg16::new(0xA6, 0x80),
    Reg16::new(0xAB, 0x40),
    Reg16::new(0xAE, 0x0C),
    Reg16::new(0xB3, 0x34),
    Reg16::new(0xB4, 0x44),
    Reg16::new(0xB6, 0x38),
    Reg16::new(0xB7, 0x02),
    Reg16::new(0xB9, 0x30),
    Reg16::new(0x3C, 0x08),
    Reg16::new(0x3D, 0x30),
    Reg16::new(0x4B, 0x0D),
    Reg16::new(0x4C, 0x20),
    Reg16::new(0xFE, 0x00),
    Reg16::new(0xFE, 0x02),
    Reg16::new(0x10, 0x10),
    Reg16::new(0x11, 0x15),
    Reg16::new(0x12, 0x1A),
    Reg16::new(0x13, 0x1F),
    Reg16::new(0x14, 0x2C),
    Reg16::new(0x15, 0x39),
    Reg16::new(0x16, 0x45),
    Reg16::new(0x17, 0x54),
    Reg16::new(0x18, 0x69),
    Reg16::new(0x19, 0x7D),
    Reg16::new(0x1A, 0x8F),
    Reg16::new(0x1B, 0x9D),
    Reg16::new(0x1C, 0xA9),
    Reg16::new(0x1D, 0xBD),
    Reg16::new(0x1E, 0xCD),
    Reg16::new(0x1F, 0xD9),
    Reg16::new(0x20, 0xE3),
    Reg16::new(0x21, 0xEA),
    Reg16::new(0x22, 0xEF),
    Reg16::new(0x23, 0xF5),
    Reg16::new(0x24, 0xF9),
    Reg16::new(0x25, 0xFF),
    Reg16::new(0xFE, 0x02),
    Reg16::new(0x26, 0x0F),
    Reg16::new(0x27, 0x14),
    Reg16::new(0x28, 0x19),
    Reg16::new(0x29, 0x1E),
    Reg16::new(0x2A, 0x27),
    Reg16::new(0x2B, 0x33),
    Reg16::new(0x2C, 0x3B),
    Reg16::new(0x2D, 0x45),
    Reg16::new(0x2E, 0x59),
    Reg16::new(0x2F, 0x69),
    Reg16::new(0x30, 0x7C),
    Reg16::new(0x31, 0x89),
    Reg16::new(0x32, 0x98),
    Reg16::new(0x33, 0xAE),
    Reg16::new(0x34, 0xC0),
    Reg16::new(0x35, 0xCF),
    Reg16::new(0x36, 0xDA),
    Reg16::new(0x37, 0xE2),
    Reg16::new(0x38, 0xE9),
    Reg16::new(0x39, 0xF3),
    Reg16::new(0x3A, 0xF9),
    Reg16::new(0x3B, 0xFF),
    Reg16::new(0xFE, 0x02),
    Reg16::new(0xD1, 0x30),
    Reg16::new(0xD2, 0x30),
    Reg16::new(0xD3, 0x45),
    Reg16::new(0xDD, 0x14),
    Reg16::new(0xDE, 0x86),
    Reg16::new(0xED, 0x01),
    Reg16::new(0xEE, 0x28),
    Reg16::new(0xEF, 0x30),
    Reg16::new(0xD8, 0xD8),
    Reg16::new(0xFE, 0x01),
    Reg16::new(0xA1, 0x80),
    Reg16::new(0xA2, 0x80),
    Reg16::new(0xA4, 0x00),
    Reg16::new(0xA5, 0x00),
    Reg16::new(0xA6, 0x70),
    Reg16::new(0xA7, 0x00),
    Reg16::new(0xA8, 0x77),
    Reg16::new(0xA9, 0x77),
    Reg16::new(0xAA, 0x1F),
    Reg16::new(0xAB, 0x0D),
    Reg16::new(0xAC, 0x19),
    Reg16::new(0xAD, 0x24),
    Reg16::new(0xAE, 0x0E),
    Reg16::new(0xAF, 0x1D),
    Reg16::new(0xB0, 0x12),
    Reg16::new(0xB1, 0x0C),
    Reg16::new(0xB2, 0x06),
    Reg16::new(0xB3, 0x13),
    Reg16::new(0xB4, 0x10),
    Reg16::new(0xB5, 0x0C),
    Reg16::new(0xB6, 0x6A),
    Reg16::new(0xB7, 0x46),
    Reg16::new(0xB8, 0x40),
    Reg16::new(0xB9, 0x0B),
    Reg16::new(0xBA, 0x04),
    Reg16::new(0xBB, 0x00),
    Reg16::new(0xBC, 0x53),
    Reg16::new(0xBD, 0x37),
    Reg16::new(0xBE, 0x2D),
    Reg16::new(0xBF, 0x0A),
    Reg16::new(0xC0, 0x0A),
    Reg16::new(0xC1, 0x14),
    Reg16::new(0xC2, 0x34),
    Reg16::new(0xC3, 0x22),
    Reg16::new(0xC4, 0x18),
    Reg16::new(0xC5, 0x23),
    Reg16::new(0xC6, 0x0F),
    Reg16::new(0xC7, 0x3C),
    Reg16::new(0xC8, 0x20),
    Reg16::new(0xC9, 0x1F),
    Reg16::new(0xCA, 0x17),
    Reg16::new(0xCB, 0x2D),
    Reg16::new(0xCC, 0x12),
    Reg16::new(0xCD, 0x20),
    Reg16::new(0xD0, 0x61),
    Reg16::new(0xD1, 0x2F),
    Reg16::new(0xD2, 0x39),
    Reg16::new(0xD3, 0x45),
    Reg16::new(0xD4, 0x2C),
    Reg16::new(0xD5, 0x21),
    Reg16::new(0xD6, 0x64),
    Reg16::new(0xD7, 0x2D),
    Reg16::new(0xD8, 0x30),
    Reg16::new(0xD9, 0x42),
    Reg16::new(0xDA, 0x27),
    Reg16::new(0xDB, 0x13),
    Reg16::new(0xFE, 0x00),
    Reg16::new(0xFE, 0x01),
    Reg16::new(0x4F, 0x00),
    Reg16::new(0x4F, 0x00),
    Reg16::new(0x4B, 0x01),
    Reg16::new(0x4F, 0x00),
    Reg16::new(0x4C, 0x01),
    Reg16::new(0x4D, 0x6F),
    Reg16::new(0x4E, 0x02),
    Reg16::new(0x4C, 0x01),
    Reg16::new(0x4D, 0x70),
    Reg16::new(0x4E, 0x02),
    Reg16::new(0x4C, 0x01),
    Reg16::new(0x4D, 0x8F),
    Reg16::new(0x4E, 0x02),
    Reg16::new(0x4C, 0x01),
    Reg16::new(0x4D, 0x90),
    Reg16::new(0x4E, 0x02),
    Reg16::new(0x4C, 0x01),
    Reg16::new(0x4D, 0xED),
    Reg16::new(0x4E, 0x33),
    Reg16::new(0x4C, 0x01),
    Reg16::new(0x4D, 0xCD),
    Reg16::new(0x4E, 0x33),
    Reg16::new(0x4C, 0x01),
    Reg16::new(0x4D, 0xEC),
    Reg16::new(0x4E, 0x03),
    Reg16::new(0x4C, 0x01),
    Reg16::new(0x4D, 0x6C),
    Reg16::new(0x4E, 0x03),
    Reg16::new(0x4C, 0x01),
    Reg16::new(0x4D, 0x6D),
    Reg16::new(0x4E, 0x03),
    Reg16::new(0x4C, 0x01),
    Reg16::new(0x4D, 0x6E),
    Reg16::new(0x4E, 0x03),
    Reg16::new(0x4C, 0x01),
    Reg16::new(0x4D, 0x8C),
    Reg16::new(0x4E, 0x03),
    Reg16::new(0x4C, 0x01),
    Reg16::new(0x4D, 0x8D),
    Reg16::new(0x4E, 0x03),
    Reg16::new(0x4C, 0x01),
    Reg16::new(0x4D, 0x8E),
    Reg16::new(0x4E, 0x03),
    Reg16::new(0x4C, 0x01),
    Reg16::new(0x4D, 0xAB),
    Reg16::new(0x4E, 0x03),
    Reg16::new(0x4C, 0x01),
    Reg16::new(0x4D, 0xAC),
    Reg16::new(0x4E, 0x03),
    Reg16::new(0x4C, 0x01),
    Reg16::new(0x4D, 0xAD),
    Reg16::new(0x4E, 0x03),
    Reg16::new(0x4C, 0x01),
    Reg16::new(0x4D, 0xAE),
    Reg16::new(0x4E, 0x03),
    Reg16::new(0x4C, 0x01),
    Reg16::new(0x4D, 0xCB),
    Reg16::new(0x4E, 0x03),
    Reg16::new(0x4C, 0x01),
    Reg16::new(0x4D, 0xCC),
    Reg16::new(0x4E, 0x03),
    Reg16::new(0x4C, 0x01),
    Reg16::new(0x4D, 0xCE),
    Reg16::new(0x4E, 0x03),
    Reg16::new(0x4C, 0x01),
    Reg16::new(0x4D, 0xEB),
    Reg16::new(0x4E, 0x03),
    Reg16::new(0x4C, 0x01),
    Reg16::new(0x4D, 0xEC),
    Reg16::new(0x4E, 0x03),
    Reg16::new(0x4C, 0x01),
    Reg16::new(0x4D, 0xEE),
    Reg16::new(0x4E, 0x03),
    Reg16::new(0x4C, 0x02),
    Reg16::new(0x4D, 0x0C),
    Reg16::new(0x4E, 0x03),
    Reg16::new(0x4C, 0x02),
    Reg16::new(0x4D, 0x0D),
    Reg16::new(0x4E, 0x03),
    Reg16::new(0x4C, 0x01),
    Reg16::new(0x4D, 0xEA),
    Reg16::new(0x4E, 0x03),
    Reg16::new(0x4C, 0x01),
    Reg16::new(0x4D, 0xAF),
    Reg16::new(0x4E, 0x03),
    Reg16::new(0x4C, 0x01),
    Reg16::new(0x4D, 0xCF),
    Reg16::new(0x4E, 0x03),
    Reg16::new(0x4C, 0x01),
    Reg16::new(0x4D, 0xCA),
    Reg16::new(0x4E, 0x04),
    Reg16::new(0x4C, 0x02),
    Reg16::new(0x4D, 0x0B),
    Reg16::new(0x4E, 0x05),
    Reg16::new(0x4C, 0x02),
    Reg16::new(0x4D, 0xC8),
    Reg16::new(0x4E, 0x06),
    Reg16::new(0x4C, 0x02),
    Reg16::new(0x4D, 0xA8),
    Reg16::new(0x4E, 0x06),
    Reg16::new(0x4C, 0x02),
    Reg16::new(0x4D, 0xA9),
    Reg16::new(0x4E, 0x06),
    Reg16::new(0x4C, 0x02),
    Reg16::new(0x4D, 0x89),
    Reg16::new(0x4E, 0x06),
    Reg16::new(0x4C, 0x02),
    Reg16::new(0x4D, 0x69),
    Reg16::new(0x4E, 0x06),
    Reg16::new(0x4C, 0x02),
    Reg16::new(0x4D, 0x6A),
    Reg16::new(0x4E, 0x06),
    Reg16::new(0x4C, 0x02),
    Reg16::new(0x4D, 0xC7),
    Reg16::new(0x4E, 0x07),
    Reg16::new(0x4C, 0x02),
    Reg16::new(0x4D, 0xE7),
    Reg16::new(0x4E, 0x07),
    Reg16::new(0x4C, 0x03),
    Reg16::new(0x4D, 0x07),
    Reg16::new(0x4E, 0x07),
    Reg16::new(0x4C, 0x02),
    Reg16::new(0x4D, 0xE8),
    Reg16::new(0x4E, 0x07),
    Reg16::new(0x4C, 0x02),
    Reg16::new(0x4D, 0xE9),
    Reg16::new(0x4E, 0x07),
    Reg16::new(0x4C, 0x03),
    Reg16::new(0x4D, 0x08),
    Reg16::new(0x4E, 0x07),
    Reg16::new(0x4C, 0x03),
    Reg16::new(0x4D, 0x09),
    Reg16::new(0x4E, 0x07),
    Reg16::new(0x4C, 0x03),
    Reg16::new(0x4D, 0x27),
    Reg16::new(0x4E, 0x07),
    Reg16::new(0x4C, 0x03),
    Reg16::new(0x4D, 0x28),
    Reg16::new(0x4E, 0x07),
    Reg16::new(0x4C, 0x03),
    Reg16::new(0x4D, 0x29),
    Reg16::new(0x4E, 0x07),
    Reg16::new(0x4C, 0x03),
    Reg16::new(0x4D, 0x47),
    Reg16::new(0x4E, 0x07),
    Reg16::new(0x4C, 0x03),
    Reg16::new(0x4D, 0x48),
    Reg16::new(0x4E, 0x07),
    Reg16::new(0x4C, 0x03),
    Reg16::new(0x4D, 0x49),
    Reg16::new(0x4E, 0x07),
    Reg16::new(0x4C, 0x03),
    Reg16::new(0x4D, 0x67),
    Reg16::new(0x4E, 0x07),
    Reg16::new(0x4C, 0x03),
    Reg16::new(0x4D, 0x68),
    Reg16::new(0x4E, 0x07),
    Reg16::new(0x4C, 0x03),
    Reg16::new(0x4D, 0x69),
    Reg16::new(0x4E, 0x07),
    Reg16::new(0x4F, 0x01),
    Reg16::new(0xFE, 0x01),
    Reg16::new(0x50, 0x80),
    Reg16::new(0x51, 0xA8),
    Reg16::new(0x52, 0x57),
    Reg16::new(0x53, 0x38),
    Reg16::new(0x54, 0xC7),
    Reg16::new(0x56, 0x0E),
    Reg16::new(0x58, 0x08),
    Reg16::new(0x5B, 0x00),
    Reg16::new(0x5C, 0x74),
    Reg16::new(0x5D, 0x8B),
    Reg16::new(0x61, 0xD3),
    Reg16::new(0x62, 0xB5),
    Reg16::new(0x63, 0x00),
    Reg16::new(0x65, 0x04),
    Reg16::new(0x67, 0xB2),
    Reg16::new(0x68, 0xAC),
    Reg16::new(0x69, 0x00),
    Reg16::new(0x6A, 0xB2),
    Reg16::new(0x6B, 0xAC),
    Reg16::new(0x6C, 0xB2),
    Reg16::new(0x6D, 0xAC),
    Reg16::new(0x6E, 0x40),
    Reg16::new(0x6F, 0x18),
    Reg16::new(0x73, 0x00),
    Reg16::new(0x70, 0x10),
    Reg16::new(0x71, 0xE8),
    Reg16::new(0x72, 0xC0),
    Reg16::new(0x74, 0x01),
    Reg16::new(0x75, 0x01),
    Reg16::new(0x7F, 0x08),
    Reg16::new(0x76, 0x70),
    Reg16::new(0x77, 0x58),
    Reg16::new(0x78, 0xA0),
    Reg16::new(0xFE, 0x00),
    Reg16::new(0xFE, 0x02),
    Reg16::new(0xC0, 0x01),
    Reg16::new(0xC1, 0x50),
    Reg16::new(0xC2, 0xF9),
    Reg16::new(0xC3, 0x00),
    Reg16::new(0xC4, 0xE8),
    Reg16::new(0xC5, 0x48),
    Reg16::new(0xC6, 0xF0),
    Reg16::new(0xC7, 0x50),
    Reg16::new(0xC8, 0xF2),
    Reg16::new(0xC9, 0x00),
    Reg16::new(0xCA, 0xE0),
    Reg16::new(0xCB, 0x45),
    Reg16::new(0xCC, 0xEC),
    Reg16::new(0xCD, 0x45),
    Reg16::new(0xCE, 0xF0),
    Reg16::new(0xCF, 0x00),
    Reg16::new(0xE3, 0xF0),
    Reg16::new(0xE4, 0x45),
    Reg16::new(0xE5, 0xE8),
    Reg16::new(0xFE, 0x00),
    Reg16::new(0xF2, 0x0F),
    Reg16::new(0xFE, 0x00),
    Reg16::new(0xF7, 0x1D),
    Reg16::new(0xF8, 0x84),
    Reg16::new(0xFA, 0x00),
    Reg16::new(0x05, 0x01),
    Reg16::new(0x06, 0x3B),
    Reg16::new(0x07, 0x01),
    Reg16::new(0x08, 0x0B),
    Reg16::new(0xFE, 0x01),
    Reg16::new(0x25, 0x01),
    Reg16::new(0x26, 0x32),
    Reg16::new(0x27, 0x03),
    Reg16::new(0x28, 0x96),
    Reg16::new(0x29, 0x03),
    Reg16::new(0x2A, 0x96),
    Reg16::new(0x2B, 0x03),
    Reg16::new(0x2C, 0x96),
    Reg16::new(0x2D, 0x04),
    Reg16::new(0x2E, 0x62),
    Reg16::new(0x3C, 0x00),
    Reg16::new(0xFE, 0x00),
    Reg16::new(0xFE, 0x00),
    Reg16::new(0x18, 0x22),
    Reg16::new(0xFE, 0x02),
    Reg16::new(0x40, 0xBF),
    Reg16::new(0x46, 0xCF),
    Reg16::new(0xFE, 0x00),
    Reg16::new(0xFE, 0x00),
    Reg16::new(0xF7, 0x1D),
    Reg16::new(0xF8, 0x84),
    Reg16::new(0xFA, 0x10),
    Reg16::new(0x05, 0x01),
    Reg16::new(0x06, 0x18),
    Reg16::new(0x07, 0x00),
    Reg16::new(0x08, 0x2E),
    Reg16::new(0xFE, 0x01),
    Reg16::new(0x25, 0x00),
    Reg16::new(0x26, 0xA2),
    Reg16::new(0x27, 0x01),
    Reg16::new(0x28, 0xE6),
    Reg16::new(0x29, 0x01),
    Reg16::new(0x2A, 0xE6),
    Reg16::new(0x2B, 0x01),
    Reg16::new(0x2C, 0xE6),
    Reg16::new(0x2D, 0x04),
    Reg16::new(0x2E, 0x62),
    Reg16::new(0x3C, 0x00),
    Reg16::new(0xFE, 0x00),
    Reg16::new(0x09, 0x01),
    Reg16::new(0x0A, 0xD0),
    Reg16::new(0x0B, 0x02),
    Reg16::new(0x0C, 0x70),
    Reg16::new(0x0D, 0x01),
    Reg16::new(0x0E, 0x00),
    Reg16::new(0x0F, 0x01),
    Reg16::new(0x10, 0x50),
    Reg16::new(0x90, 0x01),
    Reg16::new(0x91, 0x00),
    Reg16::new(0x92, 0x00),
    Reg16::new(0x93, 0x00),
    Reg16::new(0x94, 0x00),
    Reg16::new(0x95, 0x00),
    Reg16::new(0x96, 0xF0),
    Reg16::new(0x97, 0x01),
    Reg16::new(0x98, 0x40),
];

#[derive(Clone, Copy)]
struct SubsampleCfg {
    ratio_numerator: u16,
    ratio_denominator: u16,
    reg0x99: u8,
    reg0x9b: u8,
    reg0x9c: u8,
    reg0x9d: u8,
    reg0x9e: u8,
    reg0x9f: u8,
    reg0xa0: u8,
    reg0xa1: u8,
    reg0xa2: u8,
}

const SUBSAMPLE_CFGS: [SubsampleCfg; 6] = [
    SubsampleCfg {
        ratio_numerator: 140,
        ratio_denominator: 420,
        reg0x99: 0x33,
        reg0x9b: 0x00,
        reg0x9c: 0x00,
        reg0x9d: 0x00,
        reg0x9e: 0x00,
        reg0x9f: 0x00,
        reg0xa0: 0x00,
        reg0xa1: 0x00,
        reg0xa2: 0x00,
    },
    SubsampleCfg {
        ratio_numerator: 210,
        ratio_denominator: 420,
        reg0x99: 0x22,
        reg0x9b: 0x00,
        reg0x9c: 0x00,
        reg0x9d: 0x00,
        reg0x9e: 0x00,
        reg0x9f: 0x00,
        reg0xa0: 0x00,
        reg0xa1: 0x00,
        reg0xa2: 0x00,
    },
    SubsampleCfg {
        ratio_numerator: 240,
        ratio_denominator: 420,
        reg0x99: 0x77,
        reg0x9b: 0x02,
        reg0x9c: 0x46,
        reg0x9d: 0x02,
        reg0x9e: 0x46,
        reg0x9f: 0x02,
        reg0xa0: 0x46,
        reg0xa1: 0x02,
        reg0xa2: 0x46,
    },
    SubsampleCfg {
        ratio_numerator: 252,
        ratio_denominator: 420,
        reg0x99: 0x55,
        reg0x9b: 0x02,
        reg0x9c: 0x04,
        reg0x9d: 0x02,
        reg0x9e: 0x04,
        reg0x9f: 0x02,
        reg0xa0: 0x04,
        reg0xa1: 0x02,
        reg0xa2: 0x04,
    },
    SubsampleCfg {
        ratio_numerator: 280,
        ratio_denominator: 420,
        reg0x99: 0x33,
        reg0x9b: 0x00,
        reg0x9c: 0x02,
        reg0x9d: 0x00,
        reg0x9e: 0x00,
        reg0x9f: 0x00,
        reg0xa0: 0x02,
        reg0xa1: 0x00,
        reg0xa2: 0x00,
    },
    SubsampleCfg {
        ratio_numerator: 420,
        ratio_denominator: 420,
        reg0x99: 0x11,
        reg0x9b: 0x00,
        reg0x9c: 0x00,
        reg0x9d: 0x00,
        reg0x9e: 0x00,
        reg0x9f: 0x00,
        reg0xa0: 0x00,
        reg0xa1: 0x00,
        reg0xa2: 0x00,
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

/// Detects a GC2145 sensor at `slv_addr` (`0x3C`).
pub fn detect<B: SccbBus>(bus: &mut B, slv_addr: u8) -> Result<SensorId, CameraError> {
    Gc2145::detect(bus, slv_addr)
}

/// Safe `#![no_std]` driver for the GalaxyCore GC2145 2MP UXGA camera sensor.
#[derive(Debug, Clone)]
pub struct Gc2145 {
    id: SensorId,
    status: CameraStatus,
    slv_addr: u8,
    pixformat: PixelFormat,
}

pub type GC2145 = Gc2145;

impl Default for Gc2145 {
    fn default() -> Self {
        Self::new()
    }
}

impl Gc2145 {
    /// Creates a new [`Gc2145`] instance with default SCCB address (`0x3C`) and PID (`0x2145`).
    pub const fn new() -> Self {
        Self {
            id: SensorId {
                midh: 0,
                midl: 0,
                pid: CameraPid::GC2145.as_u16(),
                ver: 0,
            },
            status: CameraStatus {
                framesize: FrameSize::Uxga,
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
            slv_addr: CameraSccbAddr::GC2145,
            pixformat: PixelFormat::Rgb565,
        }
    }

    /// Creates a new [`Gc2145`] with an explicit slave address and [`SensorId`].
    pub const fn with_addr(slv_addr: u8, id: SensorId) -> Self {
        Self {
            id,
            slv_addr,
            ..Self::new()
        }
    }

    /// Probes the SCCB bus at `slv_addr` (`0x3C`) and verifies `GC2145_PID` (`0x2145`).
    pub fn detect<B: SccbBus>(bus: &mut B, slv_addr: u8) -> Result<SensorId, CameraError> {
        if slv_addr != CameraSccbAddr::GC2145 {
            return Err(CameraError::NotDetected);
        }
        let midl = bus.read_reg8(slv_addr, CHIP_ID_LOW)? as u16;
        let midh = bus.read_reg8(slv_addr, CHIP_ID_HIGH)? as u16;
        let pid = (midh << 8) | midl;
        if pid == CameraPid::GC2145.as_u16() {
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

    /// Returns the 7-bit SCCB slave address (`0x3C`).
    pub const fn slv_addr(&self) -> u8 {
        self.slv_addr
    }

    /// Returns the currently configured [`PixelFormat`].
    pub const fn pixformat(&self) -> PixelFormat {
        self.pixformat
    }

    /// Returns the [`CameraModel`] (`CameraModel::Gc2145`).
    pub const fn model(&self) -> CameraModel {
        CameraModel::Gc2145
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

impl<B: SccbBus> CameraSensor<B> for Gc2145 {
    fn model(&self) -> CameraModel {
        CameraModel::Gc2145
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
        bus.write_reg8(self.slv_addr, RESET_RELATED, 0xE0)?;
        bus.delay_ms(100);
        write_reg_list(bus, self.slv_addr, GC2145_DEFAULT_INIT_REGS)?;
        bus.delay_ms(100);
        bus.write_reg8(self.slv_addr, 0xFE, 0x00)?;
        set_reg_bits(bus, self.slv_addr, 0xF8, 0, 0x3F, 2)?;
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
        self.status.hmirror = check_reg_mask(bus, self.slv_addr, P0_ANALOG_MODE1, 0x01)?;
        self.status.vflip = check_reg_mask(bus, self.slv_addr, P0_ANALOG_MODE1, 0x02)?;
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
                set_reg_bits(bus, self.slv_addr, P0_OUTPUT_FORMAT, 0, 0x1F, 6)?;
            }
            PixelFormat::Yuv422 => {
                bus.write_reg8(self.slv_addr, 0xFE, 0x00)?;
                set_reg_bits(bus, self.slv_addr, P0_OUTPUT_FORMAT, 0, 0x1F, 2)?;
            }
            _ => return Err(CameraError::NotSupported),
        }
        self.pixformat = pixformat;
        Ok(())
    }

    fn set_framesize(&mut self, bus: &mut B, framesize: FrameSize) -> Result<(), CameraError> {
        let fs = if framesize > FrameSize::Uxga {
            FrameSize::Uxga
        } else {
            framesize
        };
        self.status.framesize = fs;
        let w = fs.width();
        let h = fs.height();
        let uxga_w = FrameSize::Uxga.width();
        let uxga_h = FrameSize::Uxga.height();

        let mut win_w = uxga_w;
        let mut win_h = uxga_h;
        let mut row_s = (uxga_h - h) / 2;
        let mut col_s = (uxga_w - w) / 2;
        let start_idx = usize::from(fs >= FrameSize::Qvga);
        let mut chosen = SUBSAMPLE_CFGS[SUBSAMPLE_CFGS.len() - 1];

        for cfg in &SUBSAMPLE_CFGS[start_idx..] {
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
                row_s = (uxga_h.saturating_sub(win_h)) / 2;
                col_s = (uxga_w.saturating_sub(win_w)) / 2;
                chosen = *cfg;
                break;
            }
        }

        bus.write_reg8(self.slv_addr, 0xFE, 0x00)?;
        bus.write_reg8(self.slv_addr, P0_CROP_ENABLE, 0x01)?;
        bus.write_reg8(self.slv_addr, 0x09, h8(row_s))?;
        bus.write_reg8(self.slv_addr, 0x0A, l8(row_s))?;
        bus.write_reg8(self.slv_addr, 0x0B, h8(col_s))?;
        bus.write_reg8(self.slv_addr, 0x0C, l8(col_s))?;
        bus.write_reg8(self.slv_addr, 0x0D, h8(win_h + 8))?;
        bus.write_reg8(self.slv_addr, 0x0E, l8(win_h + 8))?;
        bus.write_reg8(self.slv_addr, 0x0F, h8(win_w + 16))?;
        bus.write_reg8(self.slv_addr, 0x10, l8(win_w + 16))?;

        bus.write_reg8(self.slv_addr, 0x99, chosen.reg0x99)?;
        bus.write_reg8(self.slv_addr, 0x9B, chosen.reg0x9b)?;
        bus.write_reg8(self.slv_addr, 0x9C, chosen.reg0x9c)?;
        bus.write_reg8(self.slv_addr, 0x9D, chosen.reg0x9d)?;
        bus.write_reg8(self.slv_addr, 0x9E, chosen.reg0x9e)?;
        bus.write_reg8(self.slv_addr, 0x9F, chosen.reg0x9f)?;
        bus.write_reg8(self.slv_addr, 0xA0, chosen.reg0xa0)?;
        bus.write_reg8(self.slv_addr, 0xA1, chosen.reg0xa1)?;
        bus.write_reg8(self.slv_addr, 0xA2, chosen.reg0xa2)?;

        bus.write_reg8(self.slv_addr, 0x95, h8(h))?;
        bus.write_reg8(self.slv_addr, 0x96, l8(h))?;
        bus.write_reg8(self.slv_addr, 0x97, h8(w))?;
        bus.write_reg8(self.slv_addr, 0x98, l8(w))
    }

    fn set_colorbar(&mut self, _bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.status.colorbar = u8::from(enable);
        Ok(())
    }

    fn set_hmirror(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.status.hmirror = u8::from(enable);
        bus.write_reg8(self.slv_addr, 0xFE, 0x00)?;
        set_reg_bits(bus, self.slv_addr, P0_ANALOG_MODE1, 0, 0x01, u8::from(enable))
    }

    fn set_vflip(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.status.vflip = u8::from(enable);
        bus.write_reg8(self.slv_addr, 0xFE, 0x00)?;
        set_reg_bits(bus, self.slv_addr, P0_ANALOG_MODE1, 1, 0x01, u8::from(enable))
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
