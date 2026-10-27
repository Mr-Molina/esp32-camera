//! Safe `#![no_std]` OV2640 (2MP UXGA JPEG/RGB565/YUV422) camera sensor driver.
//!
//! Ported from `sensors/ov2640.c`, `sensors/private_include/ov2640_regs.h`, and
//! `sensors/private_include/ov2640_settings.h`.
//!
//! ### `OV-105` Engineering Invariant
//! In [`Ov2640::set_window`], `HSIZE8` (`0xC0`), `VSIZE8` (`0xC1`), and `SIZEL` (`0x8C`)
//! are **never** overwritten with cropped `max_x` / `max_y` dimensions (such as `1600x900`
//! for 16:9 `FrameSize::Hd`). The underlying UXGA (`1600x1200`), SVGA (`800x600`), or
//! CIF (`400x296`) DSP sensor input dimensions established by the mode table are preserved
//! so that vertical cropping (`offset_y = 150`, `max_y = 900` -> `1050 <= 1200`) streams
//! without DSP pipeline stalls.

use crate::{
    AspectRatio, CameraError, CameraModel, CameraPid, CameraSccbAddr, CameraSensor, CameraStatus,
    FrameSize, GainCeiling, PixelFormat, Reg8, SccbBus, SensorId,
};

// DSP register bank (0xFF = 0x00)
pub const R_BYPASS: u8 = 0x05;
pub const QS: u8 = 0x44;
pub const CTRLI: u8 = 0x50;
pub const HSIZE: u8 = 0x51;
pub const VSIZE: u8 = 0x52;
pub const XOFFL: u8 = 0x53;
pub const YOFFL: u8 = 0x54;
pub const VHYX: u8 = 0x55;
pub const DPRP: u8 = 0x56;
pub const TEST: u8 = 0x57;
pub const ZMOW: u8 = 0x5A;
pub const ZMOH: u8 = 0x5B;
pub const ZMHH: u8 = 0x5C;
pub const BPADDR: u8 = 0x7C;
pub const BPDATA: u8 = 0x7D;
pub const CTRL2: u8 = 0x86;
pub const CTRL3: u8 = 0x87;
pub const SIZEL: u8 = 0x8C;
pub const HSIZE8: u8 = 0xC0;
pub const VSIZE8: u8 = 0xC1;
pub const CTRL0: u8 = 0xC2;
pub const CTRL1: u8 = 0xC3;
pub const R_DVP_SP: u8 = 0xD3;
pub const IMAGE_MODE: u8 = 0xDA;
pub const RESET: u8 = 0xE0;
pub const MS_SP: u8 = 0xF0;
pub const SS_ID: u8 = 0xF7;
pub const SS_CTRL: u8 = 0xF7;
pub const MC_BIST: u8 = 0xF9;
pub const MC_AL: u8 = 0xFA;
pub const MC_AH: u8 = 0xFB;
pub const MC_D: u8 = 0xFC;
pub const P_CMD: u8 = 0xFD;
pub const P_STATUS: u8 = 0xFE;
pub const BANK_SEL: u8 = 0xFF;

pub const CTRLI_LP_DP: u8 = 0x80;
pub const CTRLI_ROUND: u8 = 0x40;

pub const CTRL0_AEC_EN: u8 = 0x80;
pub const CTRL0_AEC_SEL: u8 = 0x40;
pub const CTRL0_STAT_SEL: u8 = 0x20;
pub const CTRL0_VFIRST: u8 = 0x10;
pub const CTRL0_YUV422: u8 = 0x08;
pub const CTRL0_YUV_EN: u8 = 0x04;
pub const CTRL0_RGB_EN: u8 = 0x02;
pub const CTRL0_RAW_EN: u8 = 0x01;

pub const CTRL2_DCW_EN: u8 = 0x20;
pub const CTRL2_SDE_EN: u8 = 0x10;
pub const CTRL2_UV_ADJ_EN: u8 = 0x08;
pub const CTRL2_UV_AVG_EN: u8 = 0x04;
pub const CTRL2_CMX_EN: u8 = 0x01;

pub const CTRL3_BPC_EN: u8 = 0x80;
pub const CTRL3_WPC_EN: u8 = 0x40;

pub const R_DVP_SP_AUTO_MODE: u8 = 0x80;

pub const R_BYPASS_DSP_EN: u8 = 0x00;
pub const R_BYPASS_DSP_BYPAS: u8 = 0x01;

pub const IMAGE_MODE_Y8_DVP_EN: u8 = 0x40;
pub const IMAGE_MODE_JPEG_EN: u8 = 0x10;
pub const IMAGE_MODE_YUV422: u8 = 0x00;
pub const IMAGE_MODE_RAW10: u8 = 0x04;
pub const IMAGE_MODE_RGB565: u8 = 0x08;
pub const IMAGE_MODE_HREF_VSYNC: u8 = 0x02;
pub const IMAGE_MODE_LBYTE_FIRST: u8 = 0x01;

pub const RESET_MICROC: u8 = 0x40;
pub const RESET_SCCB: u8 = 0x20;
pub const RESET_JPEG: u8 = 0x10;
pub const RESET_DVP: u8 = 0x04;
pub const RESET_IPU: u8 = 0x02;
pub const RESET_CIF: u8 = 0x01;

pub const MC_BIST_RESET: u8 = 0x80;
pub const MC_BIST_BOOT_ROM_SEL: u8 = 0x40;

// Sensor register bank (0xFF = 0x01)
pub const GAIN: u8 = 0x00;
pub const COM1: u8 = 0x03;
pub const REG04: u8 = 0x04;
pub const REG08: u8 = 0x08;
pub const COM2: u8 = 0x09;
pub const REG_PID: u8 = 0x0A;
pub const REG_VER: u8 = 0x0B;
pub const COM3: u8 = 0x0C;
pub const COM4: u8 = 0x0D;
pub const AEC: u8 = 0x10;
pub const CLKRC: u8 = 0x11;
pub const COM7: u8 = 0x12;
pub const COM8: u8 = 0x13;
pub const COM9: u8 = 0x14;
pub const COM10: u8 = 0x15;
pub const HSTART: u8 = 0x17;
pub const HSTOP: u8 = 0x18;
pub const VSTART: u8 = 0x19;
pub const VSTOP: u8 = 0x1A;
pub const REG_MIDH: u8 = 0x1C;
pub const REG_MIDL: u8 = 0x1D;
pub const AEW: u8 = 0x24;
pub const AEB: u8 = 0x25;
pub const VV: u8 = 0x26;
pub const REG2A: u8 = 0x2A;
pub const FRARL: u8 = 0x2B;
pub const ADDVSL: u8 = 0x2D;
pub const ADDVSH: u8 = 0x2E;
pub const YAVG: u8 = 0x2F;
pub const HSDY: u8 = 0x30;
pub const HEDY: u8 = 0x31;
pub const REG32: u8 = 0x32;
pub const ARCOM2: u8 = 0x34;
pub const REG45: u8 = 0x45;
pub const FLL: u8 = 0x46;
pub const FLH: u8 = 0x47;
pub const COM19: u8 = 0x48;
pub const ZOOMS: u8 = 0x49;
pub const COM22: u8 = 0x4B;
pub const COM25: u8 = 0x4E;
pub const BD50: u8 = 0x4F;
pub const BD60: u8 = 0x50;
pub const REG5D: u8 = 0x5D;
pub const REG5E: u8 = 0x5E;
pub const REG5F: u8 = 0x5F;
pub const REG60: u8 = 0x60;
pub const HISTO_LOW: u8 = 0x61;
pub const HISTO_HIGH: u8 = 0x62;

pub const REG04_DEFAULT: u8 = 0x28;
pub const REG04_HFLIP_IMG: u8 = 0x80;
pub const REG04_VFLIP_IMG: u8 = 0x40;
pub const REG04_VREF_EN: u8 = 0x10;
pub const REG04_HREF_EN: u8 = 0x08;

pub const COM2_STDBY: u8 = 0x10;
pub const COM2_OUT_DRIVE_1X: u8 = 0x00;
pub const COM2_OUT_DRIVE_2X: u8 = 0x01;
pub const COM2_OUT_DRIVE_3X: u8 = 0x02;
pub const COM2_OUT_DRIVE_4X: u8 = 0x03;

pub const COM7_SRST: u8 = 0x80;
pub const COM7_RES_UXGA: u8 = 0x00;
pub const COM7_RES_SVGA: u8 = 0x40;
pub const COM7_RES_CIF: u8 = 0x20;
pub const COM7_ZOOM_EN: u8 = 0x04;
pub const COM7_COLOR_BAR: u8 = 0x02;

pub const COM8_DEFAULT: u8 = 0xC0;
pub const COM8_BNDF_EN: u8 = 0x20;
pub const COM8_AGC_EN: u8 = 0x04;
pub const COM8_AEC_EN: u8 = 0x01;

pub const COM9_DEFAULT: u8 = 0x08;
pub const COM9_AGC_GAIN_8X: u8 = 0x02;

pub const REG32_UXGA: u8 = 0x36;
pub const REG32_SVGA: u8 = 0x09;
pub const REG32_CIF: u8 = 0x89;

/// OV2640 register bank selector (`0xFF`).
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Ov2640Bank {
    Dsp = 0x00,
    Sensor = 0x01,
    Max = 0x02,
}

impl Ov2640Bank {
    #[inline]
    pub const fn from_u8(val: u8) -> Self {
        match val {
            0x00 => Self::Dsp,
            0x01 => Self::Sensor,
            _ => Self::Max,
        }
    }
}

/// OV2640 base resolution mode (`ov2640_sensor_mode_t`).
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Ov2640SensorMode {
    Uxga = 0,
    Svga = 1,
    Cif = 2,
}

impl Ov2640SensorMode {
    #[inline]
    pub const fn from_i32(val: i32) -> Self {
        match val {
            1 => Self::Svga,
            2 => Self::Cif,
            _ => Self::Uxga,
        }
    }
}

/// Aspect ratio crop window entry (`ov2640_ratio_settings_t`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Ov2640RatioSettings {
    pub offset_x: u16,
    pub offset_y: u16,
    pub max_x: u16,
    pub max_y: u16,
}

/// Aspect ratio crop table matching `ratio_table[]` in `ov2640_settings.h`.
pub const RATIO_TABLE: [Ov2640RatioSettings; 9] = [
    Ov2640RatioSettings { offset_x: 0, offset_y: 0, max_x: 1600, max_y: 1200 },   // 4x3
    Ov2640RatioSettings { offset_x: 8, offset_y: 72, max_x: 1584, max_y: 1056 },  // 3x2
    Ov2640RatioSettings { offset_x: 0, offset_y: 100, max_x: 1600, max_y: 1000 }, // 16x10
    Ov2640RatioSettings { offset_x: 0, offset_y: 120, max_x: 1600, max_y: 960 },  // 5x3
    Ov2640RatioSettings { offset_x: 0, offset_y: 150, max_x: 1600, max_y: 900 },  // 16x9
    Ov2640RatioSettings { offset_x: 2, offset_y: 258, max_x: 1596, max_y: 684 },  // 21x9
    Ov2640RatioSettings { offset_x: 50, offset_y: 0, max_x: 1500, max_y: 1200 },  // 5x4
    Ov2640RatioSettings { offset_x: 200, offset_y: 0, max_x: 1200, max_y: 1200 }, // 1x1
    Ov2640RatioSettings { offset_x: 462, offset_y: 0, max_x: 676, max_y: 1200 },  // 9x16
];

pub const OV2640_SETTINGS_CIF: [Reg8; 157] = [
    Reg8::new(BANK_SEL, Ov2640Bank::Dsp as u8),
    Reg8::new(0x2c, 0xff),
    Reg8::new(0x2e, 0xdf),
    Reg8::new(BANK_SEL, Ov2640Bank::Sensor as u8),
    Reg8::new(0x3c, 0x32),
    Reg8::new(CLKRC, 0x01),
    Reg8::new(COM2, COM2_OUT_DRIVE_3X),
    Reg8::new(REG04, REG04_DEFAULT),
    Reg8::new(COM8, COM8_DEFAULT | COM8_BNDF_EN | COM8_AGC_EN | COM8_AEC_EN),
    Reg8::new(COM9, COM9_DEFAULT | (COM9_AGC_GAIN_8X << 5)),
    Reg8::new(0x2c, 0x0c),
    Reg8::new(0x33, 0x78),
    Reg8::new(0x3a, 0x33),
    Reg8::new(0x3b, 0xfb),
    Reg8::new(0x3e, 0x00),
    Reg8::new(0x43, 0x11),
    Reg8::new(0x16, 0x10),
    Reg8::new(0x39, 0x92),
    Reg8::new(0x35, 0xda),
    Reg8::new(0x22, 0x1a),
    Reg8::new(0x37, 0xc3),
    Reg8::new(0x23, 0x00),
    Reg8::new(ARCOM2, 0xc0),
    Reg8::new(0x06, 0x88),
    Reg8::new(0x07, 0xc0),
    Reg8::new(COM4, 0x87),
    Reg8::new(0x0e, 0x41),
    Reg8::new(0x4c, 0x00),
    Reg8::new(0x4a, 0x81),
    Reg8::new(0x21, 0x99),
    Reg8::new(AEW, 0x40),
    Reg8::new(AEB, 0x38),
    Reg8::new(VV, (8 << 4) | 2),
    Reg8::new(0x5c, 0x00),
    Reg8::new(0x63, 0x00),
    Reg8::new(HISTO_LOW, 0x70),
    Reg8::new(HISTO_HIGH, 0x80),
    Reg8::new(0x7c, 0x05),
    Reg8::new(0x20, 0x80),
    Reg8::new(0x28, 0x30),
    Reg8::new(0x6c, 0x00),
    Reg8::new(0x6d, 0x80),
    Reg8::new(0x6e, 0x00),
    Reg8::new(0x70, 0x02),
    Reg8::new(0x71, 0x94),
    Reg8::new(0x73, 0xc1),
    Reg8::new(0x3d, 0x34),
    Reg8::new(0x5a, 0x57),
    Reg8::new(BD50, 0xbb),
    Reg8::new(BD60, 0x9c),
    Reg8::new(COM7, COM7_RES_CIF),
    Reg8::new(HSTART, 0x11),
    Reg8::new(HSTOP, 0x43),
    Reg8::new(VSTART, 0x00),
    Reg8::new(VSTOP, 0x25),
    Reg8::new(REG32, 0x89),
    Reg8::new(0x37, 0xc0),
    Reg8::new(BD50, 0xca),
    Reg8::new(BD60, 0xa8),
    Reg8::new(0x6d, 0x00),
    Reg8::new(0x3d, 0x38),
    Reg8::new(BANK_SEL, Ov2640Bank::Dsp as u8),
    Reg8::new(0xe5, 0x7f),
    Reg8::new(MC_BIST, MC_BIST_RESET | MC_BIST_BOOT_ROM_SEL),
    Reg8::new(0x41, 0x24),
    Reg8::new(RESET, RESET_JPEG | RESET_DVP),
    Reg8::new(0x76, 0xff),
    Reg8::new(0x33, 0xa0),
    Reg8::new(0x42, 0x20),
    Reg8::new(0x43, 0x18),
    Reg8::new(0x4c, 0x00),
    Reg8::new(CTRL3, CTRL3_WPC_EN | 0x10),
    Reg8::new(0x88, 0x3f),
    Reg8::new(0xd7, 0x03),
    Reg8::new(0xd9, 0x10),
    Reg8::new(R_DVP_SP, R_DVP_SP_AUTO_MODE | 0x02),
    Reg8::new(0xc8, 0x08),
    Reg8::new(0xc9, 0x80),
    Reg8::new(BPADDR, 0x00),
    Reg8::new(BPDATA, 0x00),
    Reg8::new(BPADDR, 0x03),
    Reg8::new(BPDATA, 0x48),
    Reg8::new(BPDATA, 0x48),
    Reg8::new(BPADDR, 0x08),
    Reg8::new(BPDATA, 0x20),
    Reg8::new(BPDATA, 0x10),
    Reg8::new(BPDATA, 0x0e),
    Reg8::new(0x90, 0x00),
    Reg8::new(0x91, 0x0e),
    Reg8::new(0x91, 0x1a),
    Reg8::new(0x91, 0x31),
    Reg8::new(0x91, 0x5a),
    Reg8::new(0x91, 0x69),
    Reg8::new(0x91, 0x75),
    Reg8::new(0x91, 0x7e),
    Reg8::new(0x91, 0x88),
    Reg8::new(0x91, 0x8f),
    Reg8::new(0x91, 0x96),
    Reg8::new(0x91, 0xa3),
    Reg8::new(0x91, 0xaf),
    Reg8::new(0x91, 0xc4),
    Reg8::new(0x91, 0xd7),
    Reg8::new(0x91, 0xe8),
    Reg8::new(0x91, 0x20),
    Reg8::new(0x92, 0x00),
    Reg8::new(0x93, 0x06),
    Reg8::new(0x93, 0xe3),
    Reg8::new(0x93, 0x05),
    Reg8::new(0x93, 0x05),
    Reg8::new(0x93, 0x00),
    Reg8::new(0x93, 0x04),
    Reg8::new(0x93, 0x00),
    Reg8::new(0x93, 0x00),
    Reg8::new(0x93, 0x00),
    Reg8::new(0x93, 0x00),
    Reg8::new(0x93, 0x00),
    Reg8::new(0x93, 0x00),
    Reg8::new(0x93, 0x00),
    Reg8::new(0x96, 0x00),
    Reg8::new(0x97, 0x08),
    Reg8::new(0x97, 0x19),
    Reg8::new(0x97, 0x02),
    Reg8::new(0x97, 0x0c),
    Reg8::new(0x97, 0x24),
    Reg8::new(0x97, 0x30),
    Reg8::new(0x97, 0x28),
    Reg8::new(0x97, 0x26),
    Reg8::new(0x97, 0x02),
    Reg8::new(0x97, 0x98),
    Reg8::new(0x97, 0x80),
    Reg8::new(0x97, 0x00),
    Reg8::new(0x97, 0x00),
    Reg8::new(0xa4, 0x00),
    Reg8::new(0xa8, 0x00),
    Reg8::new(0xc5, 0x11),
    Reg8::new(0xc6, 0x51),
    Reg8::new(0xbf, 0x80),
    Reg8::new(0xc7, 0x10),
    Reg8::new(0xb6, 0x66),
    Reg8::new(0xb8, 0xa5),
    Reg8::new(0xb7, 0x64),
    Reg8::new(0xb9, 0x7c),
    Reg8::new(0xb3, 0xaf),
    Reg8::new(0xb4, 0x97),
    Reg8::new(0xb5, 0xff),
    Reg8::new(0xb0, 0xc5),
    Reg8::new(0xb1, 0x94),
    Reg8::new(0xb2, 0x0f),
    Reg8::new(0xc4, 0x5c),
    Reg8::new(CTRL1, 0xfd),
    Reg8::new(0x7f, 0x00),
    Reg8::new(0xe5, 0x1f),
    Reg8::new(0xe1, 0x67),
    Reg8::new(0xdd, 0x7f),
    Reg8::new(IMAGE_MODE, 0x00),
    Reg8::new(RESET, 0x00),
    Reg8::new(R_BYPASS, R_BYPASS_DSP_EN),
];

pub const OV2640_SETTINGS_TO_CIF: [Reg8; 36] = [
    Reg8::new(BANK_SEL, Ov2640Bank::Sensor as u8),
    Reg8::new(COM7, COM7_RES_CIF),
    Reg8::new(COM1, 0x0A),
    Reg8::new(REG32, REG32_CIF),
    Reg8::new(HSTART, 0x11),
    Reg8::new(HSTOP, 0x43),
    Reg8::new(VSTART, 0x00),
    Reg8::new(VSTOP, 0x25),
    Reg8::new(BD50, 0xca),
    Reg8::new(BD60, 0xa8),
    Reg8::new(0x5a, 0x23),
    Reg8::new(0x6d, 0x00),
    Reg8::new(0x3d, 0x38),
    Reg8::new(0x39, 0x92),
    Reg8::new(0x35, 0xda),
    Reg8::new(0x22, 0x1a),
    Reg8::new(0x37, 0xc3),
    Reg8::new(0x23, 0x00),
    Reg8::new(ARCOM2, 0xc0),
    Reg8::new(0x06, 0x88),
    Reg8::new(0x07, 0xc0),
    Reg8::new(COM4, 0x87),
    Reg8::new(0x0e, 0x41),
    Reg8::new(0x4c, 0x00),
    Reg8::new(BANK_SEL, Ov2640Bank::Dsp as u8),
    Reg8::new(RESET, RESET_DVP),
    Reg8::new(HSIZE8, 0x32),
    Reg8::new(VSIZE8, 0x25),
    Reg8::new(SIZEL, 0x00),
    Reg8::new(HSIZE, 0x64),
    Reg8::new(VSIZE, 0x4a),
    Reg8::new(XOFFL, 0x00),
    Reg8::new(YOFFL, 0x00),
    Reg8::new(VHYX, 0x00),
    Reg8::new(TEST, 0x00),
    Reg8::new(CTRL2, CTRL2_DCW_EN | 0x1D),
];

pub const OV2640_SETTINGS_TO_CIF_TAIL: [Reg8; 1] = [Reg8::new(CTRLI, CTRLI_LP_DP)];

pub const OV2640_SETTINGS_TO_SVGA: [Reg8; 39] = [
    Reg8::new(BANK_SEL, Ov2640Bank::Sensor as u8),
    Reg8::new(COM7, COM7_RES_SVGA),
    Reg8::new(COM1, 0x0A),
    Reg8::new(REG32, REG32_SVGA),
    Reg8::new(HSTART, 0x11),
    Reg8::new(HSTOP, 0x43),
    Reg8::new(VSTART, 0x00),
    Reg8::new(VSTOP, 0x4b),
    Reg8::new(0x37, 0xc0),
    Reg8::new(BD50, 0xca),
    Reg8::new(BD60, 0xa8),
    Reg8::new(0x5a, 0x23),
    Reg8::new(0x6d, 0x00),
    Reg8::new(0x3d, 0x38),
    Reg8::new(0x39, 0x92),
    Reg8::new(0x35, 0xda),
    Reg8::new(0x22, 0x1a),
    Reg8::new(0x37, 0xc3),
    Reg8::new(0x23, 0x00),
    Reg8::new(ARCOM2, 0xc0),
    Reg8::new(0x06, 0x88),
    Reg8::new(0x07, 0xc0),
    Reg8::new(COM4, 0x87),
    Reg8::new(0x0e, 0x41),
    Reg8::new(0x42, 0x03),
    Reg8::new(0x4c, 0x00),
    Reg8::new(BANK_SEL, Ov2640Bank::Dsp as u8),
    Reg8::new(RESET, RESET_DVP),
    Reg8::new(HSIZE8, 0x64),
    Reg8::new(VSIZE8, 0x4B),
    Reg8::new(SIZEL, 0x00),
    Reg8::new(HSIZE, 0xC8),
    Reg8::new(VSIZE, 0x96),
    Reg8::new(XOFFL, 0x00),
    Reg8::new(YOFFL, 0x00),
    Reg8::new(VHYX, 0x00),
    Reg8::new(TEST, 0x00),
    Reg8::new(CTRL2, CTRL2_DCW_EN | 0x1D),
    Reg8::new(CTRLI, CTRLI_LP_DP),
];

pub const OV2640_SETTINGS_TO_UXGA: [Reg8; 38] = [
    Reg8::new(BANK_SEL, Ov2640Bank::Sensor as u8),
    Reg8::new(COM7, COM7_RES_UXGA),
    Reg8::new(COM1, 0x0F),
    Reg8::new(REG32, REG32_UXGA),
    Reg8::new(HSTART, 0x11),
    Reg8::new(HSTOP, 0x75),
    Reg8::new(VSTART, 0x01),
    Reg8::new(VSTOP, 0x97),
    Reg8::new(0x3d, 0x34),
    Reg8::new(BD50, 0xbb),
    Reg8::new(BD60, 0x9c),
    Reg8::new(0x5a, 0x57),
    Reg8::new(0x6d, 0x80),
    Reg8::new(0x39, 0x82),
    Reg8::new(0x23, 0x00),
    Reg8::new(0x07, 0xc0),
    Reg8::new(0x4c, 0x00),
    Reg8::new(0x35, 0x88),
    Reg8::new(0x22, 0x0a),
    Reg8::new(0x37, 0x40),
    Reg8::new(ARCOM2, 0xa0),
    Reg8::new(0x06, 0x02),
    Reg8::new(COM4, 0xb7),
    Reg8::new(0x0e, 0x01),
    Reg8::new(0x42, 0x83),
    Reg8::new(BANK_SEL, Ov2640Bank::Dsp as u8),
    Reg8::new(RESET, RESET_DVP),
    Reg8::new(HSIZE8, 0xc8),
    Reg8::new(VSIZE8, 0x96),
    Reg8::new(SIZEL, 0x00),
    Reg8::new(HSIZE, 0x90),
    Reg8::new(VSIZE, 0x2c),
    Reg8::new(XOFFL, 0x00),
    Reg8::new(YOFFL, 0x00),
    Reg8::new(VHYX, 0x88),
    Reg8::new(TEST, 0x00),
    Reg8::new(CTRL2, CTRL2_DCW_EN | 0x1d),
    Reg8::new(CTRLI, 0x00),
];

pub const OV2640_SETTINGS_JPEG3: [Reg8; 13] = [
    Reg8::new(BANK_SEL, Ov2640Bank::Dsp as u8),
    Reg8::new(RESET, RESET_JPEG | RESET_DVP),
    Reg8::new(IMAGE_MODE, IMAGE_MODE_JPEG_EN | IMAGE_MODE_HREF_VSYNC),
    Reg8::new(0xD7, 0x03),
    Reg8::new(0xE1, 0x77),
    Reg8::new(0xE5, 0x1F),
    Reg8::new(0xD9, 0x10),
    Reg8::new(0xDF, 0x80),
    Reg8::new(0x33, 0x80),
    Reg8::new(0x3C, 0x10),
    Reg8::new(0xEB, 0x30),
    Reg8::new(0xDD, 0x7F),
    Reg8::new(RESET, 0x00),
];

pub const OV2640_SETTINGS_YUV422: [Reg8; 6] = [
    Reg8::new(BANK_SEL, Ov2640Bank::Dsp as u8),
    Reg8::new(RESET, RESET_DVP),
    Reg8::new(IMAGE_MODE, IMAGE_MODE_YUV422),
    Reg8::new(0xD7, 0x01),
    Reg8::new(0xE1, 0x67),
    Reg8::new(RESET, 0x00),
];

pub const OV2640_SETTINGS_RGB565: [Reg8; 6] = [
    Reg8::new(BANK_SEL, Ov2640Bank::Dsp as u8),
    Reg8::new(RESET, RESET_DVP),
    Reg8::new(IMAGE_MODE, IMAGE_MODE_RGB565),
    Reg8::new(0xD7, 0x03),
    Reg8::new(0xE1, 0x77),
    Reg8::new(RESET, 0x00),
];

pub const BRIGHTNESS_REGS: [[u8; 5]; 6] = [
    [BPADDR, BPDATA, BPADDR, BPDATA, BPDATA],
    [0x00, 0x04, 0x09, 0x00, 0x00], // -2
    [0x00, 0x04, 0x09, 0x10, 0x00], // -1
    [0x00, 0x04, 0x09, 0x20, 0x00], //  0
    [0x00, 0x04, 0x09, 0x30, 0x00], // +1
    [0x00, 0x04, 0x09, 0x40, 0x00], // +2
];

pub const CONTRAST_REGS: [[u8; 7]; 6] = [
    [BPADDR, BPDATA, BPADDR, BPDATA, BPDATA, BPDATA, BPDATA],
    [0x00, 0x04, 0x07, 0x20, 0x18, 0x34, 0x06], // -2
    [0x00, 0x04, 0x07, 0x20, 0x1c, 0x2a, 0x06], // -1
    [0x00, 0x04, 0x07, 0x20, 0x20, 0x20, 0x06], //  0
    [0x00, 0x04, 0x07, 0x20, 0x24, 0x16, 0x06], // +1
    [0x00, 0x04, 0x07, 0x20, 0x28, 0x0c, 0x06], // +2
];

pub const SATURATION_REGS: [[u8; 5]; 6] = [
    [BPADDR, BPDATA, BPADDR, BPDATA, BPDATA],
    [0x00, 0x02, 0x03, 0x28, 0x28], // -2
    [0x00, 0x02, 0x03, 0x38, 0x38], // -1
    [0x00, 0x02, 0x03, 0x48, 0x48], //  0
    [0x00, 0x02, 0x03, 0x58, 0x58], // +1
    [0x00, 0x02, 0x03, 0x68, 0x68], // +2
];

pub const SPECIAL_EFFECTS_REGS: [[u8; 5]; 8] = [
    [BPADDR, BPDATA, BPADDR, BPDATA, BPDATA],
    [0x00, 0x00, 0x05, 0x80, 0x80], // 0: no effect
    [0x00, 0x40, 0x05, 0x80, 0x80], // 1: negative
    [0x00, 0x18, 0x05, 0x80, 0x80], // 2: black and white
    [0x00, 0x18, 0x05, 0x40, 0xC0], // 3: reddish
    [0x00, 0x18, 0x05, 0x40, 0x40], // 4: greenish
    [0x00, 0x18, 0x05, 0xA0, 0x40], // 5: blue
    [0x00, 0x18, 0x05, 0x40, 0xA6], // 6: retro
];

pub const WB_MODES_REGS: [[u8; 3]; 5] = [
    [0xCC, 0xCD, 0xCE],
    [0x5E, 0x41, 0x54], // 1: sunny
    [0x65, 0x41, 0x4F], // 2: cloudy
    [0x52, 0x41, 0x66], // 3: office
    [0x42, 0x3F, 0x71], // 4: home
];

pub const AE_LEVELS_REGS: [[u8; 3]; 6] = [
    [AEW, AEB, VV],
    [0x20, 0x18, 0x60], // -2
    [0x34, 0x1C, 0x00], // -1
    [0x3E, 0x38, 0x81], //  0
    [0x48, 0x40, 0x81], // +1
    [0x58, 0x50, 0x92], // +2
];

pub const AGC_GAIN_TBL: [u8; 31] = [
    0x00, 0x10, 0x18, 0x30, 0x34, 0x38, 0x3C, 0x70, 0x72, 0x74, 0x76, 0x78, 0x7A, 0x7C, 0x7E, 0xF0,
    0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8, 0xF9, 0xFA, 0xFB, 0xFC, 0xFD, 0xFE, 0xFF,
];

/// OV2640 camera sensor driver state.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Ov2640 {
    slv_addr: u8,
    id: SensorId,
    status: CameraStatus,
    pixformat: PixelFormat,
    xclk_freq_hz: u32,
    reg_bank: Ov2640Bank,
}

impl Default for Ov2640 {
    fn default() -> Self {
        Self::new()
    }
}

impl Ov2640 {
    /// Default 7-bit SCCB slave address for OV2640 (`0x30`).
    pub const SLV_ADDR: u8 = CameraSccbAddr::OV2640;
    /// Expected PID value (`0x26`).
    pub const PID: u16 = CameraPid::OV2640.as_u16();

    /// Creates a new [`Ov2640`] driver instance with default SCCB address (`0x30`).
    pub const fn new() -> Self {
        Self::with_addr(Self::SLV_ADDR)
    }

    /// Creates a new [`Ov2640`] driver instance with a custom SCCB slave address.
    pub const fn with_addr(slv_addr: u8) -> Self {
        Self::from_id(
            slv_addr,
            SensorId {
                midh: 0x7F,
                midl: 0xA2,
                pid: Self::PID,
                ver: 0x01,
            },
        )
    }

    /// Creates a new [`Ov2640`] driver instance from a detected [`SensorId`].
    pub const fn from_id(slv_addr: u8, id: SensorId) -> Self {
        Self {
            slv_addr,
            id,
            status: CameraStatus {
                framesize: FrameSize::Qvga,
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
            pixformat: PixelFormat::Rgb565,
            xclk_freq_hz: 20_000_000,
            reg_bank: Ov2640Bank::Max,
        }
    }

    /// Probes and verifies the OV2640 sensor on `bus` at `slv_addr` (`esp32_camera_ov2640_detect`).
    pub fn detect<B: SccbBus>(bus: &mut B, slv_addr: u8) -> Result<SensorId, CameraError> {
        if slv_addr != Self::SLV_ADDR {
            return Err(CameraError::NotDetected);
        }
        bus.write_reg8(slv_addr, BANK_SEL, Ov2640Bank::Sensor as u8)?;
        let pid = bus.read_reg8(slv_addr, REG_PID)? as u16;
        if pid != Self::PID {
            return Err(CameraError::NotDetected);
        }
        let ver = bus.read_reg8(slv_addr, REG_VER)?;
        let midl = bus.read_reg8(slv_addr, REG_MIDL)?;
        let midh = bus.read_reg8(slv_addr, REG_MIDH)?;
        Ok(SensorId {
            midh,
            midl,
            pid,
            ver,
        })
    }

    /// Returns the configured 7-bit SCCB slave address.
    #[inline]
    pub const fn slv_addr(&self) -> u8 {
        self.slv_addr
    }

    /// Returns the currently configured [`PixelFormat`].
    #[inline]
    pub const fn pixformat(&self) -> PixelFormat {
        self.pixformat
    }

    /// Returns the configured external clock frequency in Hz.
    #[inline]
    pub const fn xclk_freq_hz(&self) -> u32 {
        self.xclk_freq_hz
    }

    /// Returns the currently selected register bank.
    #[inline]
    pub const fn reg_bank(&self) -> Ov2640Bank {
        self.reg_bank
    }

    /// Returns the [`CameraModel`] enum discriminant for this sensor.
    #[inline]
    pub const fn model(&self) -> CameraModel {
        CameraModel::Ov2640
    }

    /// Returns a shared reference to the sensor's [`SensorId`].
    #[inline]
    pub const fn id(&self) -> &SensorId {
        &self.id
    }

    /// Returns a shared reference to the sensor's current [`CameraStatus`].
    #[inline]
    pub const fn status(&self) -> &CameraStatus {
        &self.status
    }

    /// Returns a mutable reference to the sensor's current [`CameraStatus`].
    #[inline]
    pub fn status_mut(&mut self) -> &mut CameraStatus {
        &mut self.status
    }

    fn set_bank<B: SccbBus>(&mut self, bus: &mut B, bank: Ov2640Bank) -> Result<(), CameraError> {
        if self.reg_bank != bank {
            bus.write_reg8(self.slv_addr, BANK_SEL, bank as u8)?;
            self.reg_bank = bank;
        }
        Ok(())
    }

    fn write_regs<B: SccbBus>(&mut self, bus: &mut B, regs: &[Reg8]) -> Result<(), CameraError> {
        for entry in regs {
            if entry.reg == BANK_SEL {
                self.set_bank(bus, Ov2640Bank::from_u8(entry.val))?;
            } else {
                bus.write_reg8(self.slv_addr, entry.reg, entry.val)?;
            }
        }
        Ok(())
    }

    fn write_reg<B: SccbBus>(
        &mut self,
        bus: &mut B,
        bank: Ov2640Bank,
        reg: u8,
        value: u8,
    ) -> Result<(), CameraError> {
        self.set_bank(bus, bank)?;
        bus.write_reg8(self.slv_addr, reg, value)?;
        if reg == BANK_SEL {
            self.reg_bank = Ov2640Bank::from_u8(value);
        } else if reg == COM7 && (value & COM7_SRST) != 0 {
            self.reg_bank = Ov2640Bank::Max;
        }
        Ok(())
    }

    fn read_reg<B: SccbBus>(
        &mut self,
        bus: &mut B,
        bank: Ov2640Bank,
        reg: u8,
    ) -> Result<u8, CameraError> {
        self.set_bank(bus, bank)?;
        bus.read_reg8(self.slv_addr, reg)
    }

    fn set_reg_bits<B: SccbBus>(
        &mut self,
        bus: &mut B,
        bank: Ov2640Bank,
        reg: u8,
        offset: u8,
        mask: u8,
        value: u8,
    ) -> Result<(), CameraError> {
        let c_value = self.read_reg(bus, bank, reg)?;
        let shifted_mask = mask << offset;
        let new_value = (c_value & !shifted_mask) | ((value & mask) << offset);
        self.write_reg(bus, bank, reg, new_value)
    }

    fn get_reg_bits<B: SccbBus>(
        &mut self,
        bus: &mut B,
        bank: Ov2640Bank,
        reg: u8,
        offset: u8,
        mask: u8,
    ) -> Result<u8, CameraError> {
        let val = self.read_reg(bus, bank, reg)?;
        Ok((val >> offset) & mask)
    }

    fn write_reg_bits<B: SccbBus>(
        &mut self,
        bus: &mut B,
        bank: Ov2640Bank,
        reg: u8,
        mask: u8,
        enable: bool,
    ) -> Result<(), CameraError> {
        self.set_reg_bits(bus, bank, reg, 0, mask, if enable { mask } else { 0 })
    }

    /// Configures the OV2640 DSP window, scaling, and pixel clock dividers.
    ///
    /// ### `OV-105` Invariant
    /// `HSIZE8` (`0xC0`), `VSIZE8` (`0xC1`), and `SIZEL` (`0x8C`) are intentionally omitted
    /// from `win_regs` so the base sensor mode input dimensions (`1600x1200` UXGA, `800x600`
    /// SVGA, or `400x296` CIF) set by `OV2640_SETTINGS_TO_*` remain intact when a cropped
    /// aspect ratio window (such as `16:9` `FrameSize::Hd`, `offset_y = 150`, `max_y = 900`)
    /// is applied.
    #[allow(clippy::too_many_arguments)]
    pub fn set_window<B: SccbBus>(
        &mut self,
        bus: &mut B,
        mode: Ov2640SensorMode,
        offset_x: u16,
        offset_y: u16,
        max_x: u16,
        max_y: u16,
        w: u16,
        h: u16,
    ) -> Result<(), CameraError> {
        let max_x_q = max_x / 4;
        let max_y_q = max_y / 4;
        let w_q = w / 4;
        let h_q = h / 4;

        let win_regs = [
            Reg8::new(BANK_SEL, Ov2640Bank::Dsp as u8),
            Reg8::new(HSIZE, (max_x_q & 0xFF) as u8),
            Reg8::new(VSIZE, (max_y_q & 0xFF) as u8),
            Reg8::new(XOFFL, (offset_x & 0xFF) as u8),
            Reg8::new(YOFFL, (offset_y & 0xFF) as u8),
            Reg8::new(
                VHYX,
                (((max_y_q >> 1) & 0x80)
                    | ((offset_y >> 4) & 0x70)
                    | ((max_x_q >> 5) & 0x08)
                    | ((offset_x >> 8) & 0x07)) as u8,
            ),
            Reg8::new(TEST, ((max_x_q >> 2) & 0x80) as u8),
            Reg8::new(ZMOW, (w_q & 0xFF) as u8),
            Reg8::new(ZMOH, (h_q & 0xFF) as u8),
            Reg8::new(ZMHH, (((h_q >> 6) & 0x04) | ((w_q >> 8) & 0x03)) as u8),
        ];

        let (clk_2x, clk_div, pclk_auto, pclk_div): (u8, u8, u8, u8) =
            if self.pixformat == PixelFormat::Jpeg {
                let div = if mode == Ov2640SensorMode::Uxga { 12 } else { 8 };
                (0, 0, 0, div)
            } else {
                let c_div = if mode == Ov2640SensorMode::Cif { 3 } else { 7 };
                let p_div = if mode == Ov2640SensorMode::Uxga { 12 } else { 8 };
                (0, c_div, 1, p_div)
            };

        let clk = (clk_div & 0x3F) | ((clk_2x & 0x01) << 7);
        let pclk = (pclk_div & 0x7F) | ((pclk_auto & 0x01) << 7);

        self.write_reg(bus, Ov2640Bank::Dsp, R_BYPASS, R_BYPASS_DSP_BYPAS)?;
        match mode {
            Ov2640SensorMode::Cif => {
                self.write_regs(bus, &OV2640_SETTINGS_TO_CIF)?;
                self.write_regs(bus, &OV2640_SETTINGS_TO_CIF_TAIL)?;
            }
            Ov2640SensorMode::Svga => {
                self.write_regs(bus, &OV2640_SETTINGS_TO_SVGA)?;
            }
            Ov2640SensorMode::Uxga => {
                self.write_regs(bus, &OV2640_SETTINGS_TO_UXGA)?;
            }
        }
        self.write_regs(bus, &win_regs)?;
        self.write_reg(bus, Ov2640Bank::Sensor, CLKRC, clk)?;
        self.write_reg(bus, Ov2640Bank::Dsp, R_DVP_SP, pclk)?;
        self.write_reg(bus, Ov2640Bank::Dsp, R_BYPASS, R_BYPASS_DSP_EN)?;

        bus.delay_ms(10);
        self.set_pixformat(bus, self.pixformat)
    }
}

impl<B: SccbBus> CameraSensor<B> for Ov2640 {
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
        self.write_reg(bus, Ov2640Bank::Sensor, COM7, COM7_SRST)?;
        bus.delay_ms(10);
        self.reg_bank = Ov2640Bank::Max;
        self.write_regs(bus, &OV2640_SETTINGS_CIF)
    }

    fn init_status(&mut self, bus: &mut B) -> Result<(), CameraError> {
        self.status.brightness = 0;
        self.status.contrast = 0;
        self.status.saturation = 0;
        self.status.ae_level = 0;
        self.status.special_effect = 0;
        self.status.wb_mode = 0;

        self.status.agc_gain = 30;
        let agc_gain = self.read_reg(bus, Ov2640Bank::Sensor, GAIN)?;
        for i in 0..30usize {
            if agc_gain >= AGC_GAIN_TBL[i] && agc_gain < AGC_GAIN_TBL[i + 1] {
                self.status.agc_gain = i as u8;
                break;
            }
        }

        let aec_hi = self.get_reg_bits(bus, Ov2640Bank::Sensor, REG45, 0, 0x3F)? as u16;
        let aec_mid = self.read_reg(bus, Ov2640Bank::Sensor, AEC)? as u16;
        let aec_lo = self.get_reg_bits(bus, Ov2640Bank::Sensor, REG04, 0, 0x03)? as u16;
        self.status.aec_value = (aec_hi << 10) | (aec_mid << 2) | aec_lo;

        self.status.quality = self.read_reg(bus, Ov2640Bank::Dsp, QS)?;
        self.status.gainceiling = self.get_reg_bits(bus, Ov2640Bank::Sensor, COM9, 5, 0x07)?;

        self.status.awb = self.get_reg_bits(bus, Ov2640Bank::Dsp, CTRL1, 3, 1)?;
        self.status.awb_gain = self.get_reg_bits(bus, Ov2640Bank::Dsp, CTRL1, 2, 1)?;
        self.status.aec = self.get_reg_bits(bus, Ov2640Bank::Sensor, COM8, 0, 1)?;
        self.status.aec2 = self.get_reg_bits(bus, Ov2640Bank::Dsp, CTRL0, 6, 1)?;
        self.status.agc = self.get_reg_bits(bus, Ov2640Bank::Sensor, COM8, 2, 1)?;
        self.status.bpc = self.get_reg_bits(bus, Ov2640Bank::Dsp, CTRL3, 7, 1)?;
        self.status.wpc = self.get_reg_bits(bus, Ov2640Bank::Dsp, CTRL3, 6, 1)?;
        self.status.raw_gma = self.get_reg_bits(bus, Ov2640Bank::Dsp, CTRL1, 5, 1)?;
        self.status.lenc = self.get_reg_bits(bus, Ov2640Bank::Dsp, CTRL1, 1, 1)?;
        self.status.hmirror = self.get_reg_bits(bus, Ov2640Bank::Sensor, REG04, 7, 1)?;
        self.status.vflip = self.get_reg_bits(bus, Ov2640Bank::Sensor, REG04, 6, 1)?;
        self.status.dcw = self.get_reg_bits(bus, Ov2640Bank::Dsp, CTRL2, 5, 1)?;
        self.status.colorbar = self.get_reg_bits(bus, Ov2640Bank::Sensor, COM7, 1, 1)?;

        self.status.sharpness = 0;
        self.status.denoise = 0;
        Ok(())
    }

    fn set_pixformat(&mut self, bus: &mut B, pixformat: PixelFormat) -> Result<(), CameraError> {
        let regs: &[Reg8] = match pixformat {
            PixelFormat::Rgb565 | PixelFormat::Rgb888 => &OV2640_SETTINGS_RGB565,
            PixelFormat::Yuv422 | PixelFormat::Grayscale => &OV2640_SETTINGS_YUV422,
            PixelFormat::Jpeg => &OV2640_SETTINGS_JPEG3,
            _ => return Err(CameraError::NotSupported),
        };
        self.write_regs(bus, regs)?;
        bus.delay_ms(10);
        self.pixformat = pixformat;
        Ok(())
    }

    fn set_framesize(&mut self, bus: &mut B, framesize: FrameSize) -> Result<(), CameraError> {
        if framesize > FrameSize::Uxga {
            return Err(CameraError::FailedToSetFrameSize);
        }
        let res = framesize.resolution();
        let w = res.width;
        let h = res.height;
        let ratio: AspectRatio = res.aspect_ratio;
        let ratio_entry = RATIO_TABLE[ratio as usize];
        let mut max_x = ratio_entry.max_x;
        let mut max_y = ratio_entry.max_y;
        let mut offset_x = ratio_entry.offset_x;
        let mut offset_y = ratio_entry.offset_y;
        let mut mode = Ov2640SensorMode::Uxga;

        if (framesize as u8) <= (FrameSize::Cif as u8) {
            mode = Ov2640SensorMode::Cif;
            max_x /= 4;
            max_y /= 4;
            offset_x /= 4;
            offset_y /= 4;
            if max_y > 296 {
                max_y = 296;
            }
        } else if (framesize as u8) <= (FrameSize::Svga as u8) {
            mode = Ov2640SensorMode::Svga;
            max_x /= 2;
            max_y /= 2;
            offset_x /= 2;
            offset_y /= 2;
        }

        self.set_window(bus, mode, offset_x, offset_y, max_x, max_y, w, h)?;
        self.status.framesize = framesize;
        Ok(())
    }

    fn set_contrast(&mut self, bus: &mut B, level: i32) -> Result<(), CameraError> {
        let idx = level + 3;
        if !(1..=5).contains(&idx) {
            return Err(CameraError::InvalidArg);
        }
        self.status.contrast = level as i8;
        let row = &CONTRAST_REGS[idx as usize];
        let addrs = &CONTRAST_REGS[0];
        for i in 0..7 {
            self.write_reg(bus, Ov2640Bank::Dsp, addrs[i], row[i])?;
        }
        Ok(())
    }

    fn set_brightness(&mut self, bus: &mut B, level: i32) -> Result<(), CameraError> {
        let idx = level + 3;
        if !(1..=5).contains(&idx) {
            return Err(CameraError::InvalidArg);
        }
        self.status.brightness = level as i8;
        let row = &BRIGHTNESS_REGS[idx as usize];
        let addrs = &BRIGHTNESS_REGS[0];
        for i in 0..5 {
            self.write_reg(bus, Ov2640Bank::Dsp, addrs[i], row[i])?;
        }
        Ok(())
    }

    fn set_saturation(&mut self, bus: &mut B, level: i32) -> Result<(), CameraError> {
        let idx = level + 3;
        if !(1..=5).contains(&idx) {
            return Err(CameraError::InvalidArg);
        }
        self.status.saturation = level as i8;
        let row = &SATURATION_REGS[idx as usize];
        let addrs = &SATURATION_REGS[0];
        for i in 0..5 {
            self.write_reg(bus, Ov2640Bank::Dsp, addrs[i], row[i])?;
        }
        Ok(())
    }

    fn set_gainceiling(
        &mut self,
        bus: &mut B,
        gainceiling: GainCeiling,
    ) -> Result<(), CameraError> {
        let val = gainceiling as u8;
        self.status.gainceiling = val;
        self.set_reg_bits(bus, Ov2640Bank::Sensor, COM9, 5, 0x07, val)
    }

    fn set_quality(&mut self, bus: &mut B, quality: i32) -> Result<(), CameraError> {
        if !(0..=63).contains(&quality) {
            return Err(CameraError::InvalidArg);
        }
        let q = quality as u8;
        self.status.quality = q;
        self.write_reg(bus, Ov2640Bank::Dsp, QS, q)
    }

    fn set_colorbar(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.status.colorbar = u8::from(enable);
        self.write_reg_bits(bus, Ov2640Bank::Sensor, COM7, COM7_COLOR_BAR, enable)
    }

    fn set_whitebal(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.status.awb = u8::from(enable);
        self.set_reg_bits(bus, Ov2640Bank::Dsp, CTRL1, 3, 1, u8::from(enable))
    }

    fn set_gain_ctrl(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.status.agc = u8::from(enable);
        self.write_reg_bits(bus, Ov2640Bank::Sensor, COM8, COM8_AGC_EN, enable)
    }

    fn set_exposure_ctrl(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.status.aec = u8::from(enable);
        self.write_reg_bits(bus, Ov2640Bank::Sensor, COM8, COM8_AEC_EN, enable)
    }

    fn set_hmirror(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.status.hmirror = u8::from(enable);
        self.write_reg_bits(bus, Ov2640Bank::Sensor, REG04, REG04_HFLIP_IMG, enable)
    }

    fn set_vflip(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.write_reg_bits(bus, Ov2640Bank::Sensor, REG04, REG04_VREF_EN, enable)?;
        self.write_reg_bits(bus, Ov2640Bank::Sensor, REG04, REG04_VFLIP_IMG, enable)?;
        self.status.vflip = u8::from(enable);
        Ok(())
    }

    fn set_aec2(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.status.aec2 = u8::from(enable);
        self.set_reg_bits(bus, Ov2640Bank::Dsp, CTRL0, 6, 1, u8::from(enable))
    }

    fn set_awb_gain(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.status.awb_gain = u8::from(enable);
        self.set_reg_bits(bus, Ov2640Bank::Dsp, CTRL1, 2, 1, u8::from(enable))
    }

    fn set_agc_gain(&mut self, bus: &mut B, gain: i32) -> Result<(), CameraError> {
        if !(0..=30).contains(&gain) {
            return Err(CameraError::InvalidArg);
        }
        let g = gain as usize;
        self.status.agc_gain = g as u8;
        self.write_reg(bus, Ov2640Bank::Sensor, GAIN, AGC_GAIN_TBL[g])
    }

    fn set_aec_value(&mut self, bus: &mut B, value: i32) -> Result<(), CameraError> {
        if !(0..=1200).contains(&value) {
            return Err(CameraError::InvalidArg);
        }
        let val = value as u16;
        self.status.aec_value = val;
        self.set_reg_bits(bus, Ov2640Bank::Sensor, REG04, 0, 0x03, (val & 0x03) as u8)?;
        self.write_reg(bus, Ov2640Bank::Sensor, AEC, ((val >> 2) & 0xFF) as u8)?;
        self.set_reg_bits(
            bus,
            Ov2640Bank::Sensor,
            REG45,
            0,
            0x3F,
            ((val >> 10) & 0x3F) as u8,
        )
    }

    fn set_special_effect(&mut self, bus: &mut B, effect: i32) -> Result<(), CameraError> {
        let idx = effect + 1;
        if !(1..=7).contains(&idx) {
            return Err(CameraError::InvalidArg);
        }
        self.status.special_effect = effect as u8;
        let row = &SPECIAL_EFFECTS_REGS[idx as usize];
        let addrs = &SPECIAL_EFFECTS_REGS[0];
        for i in 0..5 {
            self.write_reg(bus, Ov2640Bank::Dsp, addrs[i], row[i])?;
        }
        Ok(())
    }

    fn set_wb_mode(&mut self, bus: &mut B, mode: i32) -> Result<(), CameraError> {
        if !(0..=4).contains(&mode) {
            return Err(CameraError::InvalidArg);
        }
        self.status.wb_mode = mode as u8;
        self.set_reg_bits(
            bus,
            Ov2640Bank::Dsp,
            0xC7,
            6,
            1,
            if mode != 0 { 1 } else { 0 },
        )?;
        if mode != 0 {
            let row = &WB_MODES_REGS[mode as usize];
            let addrs = &WB_MODES_REGS[0];
            for i in 0..3 {
                self.write_reg(bus, Ov2640Bank::Dsp, addrs[i], row[i])?;
            }
        }
        Ok(())
    }

    fn set_ae_level(&mut self, bus: &mut B, level: i32) -> Result<(), CameraError> {
        let idx = level + 3;
        if !(1..=5).contains(&idx) {
            return Err(CameraError::InvalidArg);
        }
        self.status.ae_level = level as i8;
        let row = &AE_LEVELS_REGS[idx as usize];
        let addrs = &AE_LEVELS_REGS[0];
        for i in 0..3 {
            self.write_reg(bus, Ov2640Bank::Sensor, addrs[i], row[i])?;
        }
        Ok(())
    }

    fn set_dcw(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.status.dcw = u8::from(enable);
        self.set_reg_bits(bus, Ov2640Bank::Dsp, CTRL2, 5, 1, u8::from(enable))
    }

    fn set_bpc(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.status.bpc = u8::from(enable);
        self.set_reg_bits(bus, Ov2640Bank::Dsp, CTRL3, 7, 1, u8::from(enable))
    }

    fn set_wpc(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.status.wpc = u8::from(enable);
        self.set_reg_bits(bus, Ov2640Bank::Dsp, CTRL3, 6, 1, u8::from(enable))
    }

    fn set_raw_gma(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.status.raw_gma = u8::from(enable);
        self.set_reg_bits(bus, Ov2640Bank::Dsp, CTRL1, 5, 1, u8::from(enable))
    }

    fn set_lenc(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.status.lenc = u8::from(enable);
        self.set_reg_bits(bus, Ov2640Bank::Dsp, CTRL1, 1, 1, u8::from(enable))
    }

    fn get_reg(&mut self, bus: &mut B, reg: i32, mask: i32) -> Result<i32, CameraError> {
        if !(0..=0x1FF).contains(&reg) {
            return Err(CameraError::InvalidArg);
        }
        let bank = Ov2640Bank::from_u8(((reg >> 8) & 0x01) as u8);
        let val = self.read_reg(bus, bank, (reg & 0xFF) as u8)? as i32;
        Ok(val & mask)
    }

    fn set_reg(
        &mut self,
        bus: &mut B,
        reg: i32,
        mask: i32,
        value: i32,
    ) -> Result<(), CameraError> {
        if !(0..=0x1FF).contains(&reg) {
            return Err(CameraError::InvalidArg);
        }
        let bank = Ov2640Bank::from_u8(((reg >> 8) & 0x01) as u8);
        let addr = (reg & 0xFF) as u8;
        let current = self.read_reg(bus, bank, addr)? as i32;
        let next = ((current & !mask) | (value & mask)) as u8;
        self.write_reg(bus, bank, addr, next)
    }

    fn set_res_raw(
        &mut self,
        bus: &mut B,
        start_x: i32,
        _start_y: i32,
        _end_x: i32,
        _end_y: i32,
        offset_x: i32,
        offset_y: i32,
        total_x: i32,
        total_y: i32,
        output_x: i32,
        output_y: i32,
        _scale: bool,
        _binning: bool,
    ) -> Result<(), CameraError> {
        if offset_x < 0 || offset_y < 0 || total_x <= 0 || total_y <= 0 || output_x <= 0 || output_y <= 0 {
            return Err(CameraError::InvalidArg);
        }
        let mode = Ov2640SensorMode::from_i32(start_x);
        self.set_window(
            bus,
            mode,
            offset_x as u16,
            offset_y as u16,
            total_x as u16,
            total_y as u16,
            output_x as u16,
            output_y as u16,
        )
    }

    fn set_xclk(&mut self, _bus: &mut B, _timer: i32, xclk: i32) -> Result<(), CameraError> {
        if xclk <= 0 {
            return Err(CameraError::InvalidArg);
        }
        self.xclk_freq_hz = (xclk as u32).saturating_mul(1_000_000);
        Ok(())
    }
}
