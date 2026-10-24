//! HM0360 VGA Grayscale CMOS image sensor driver.
//!
//! Ported from `sensors/hm0360.c`, `sensors/private_include/hm0360_regs.h`, and
//! `sensors/private_include/hm0360_settings.h`.

use crate::{
    CameraError, CameraModel, CameraPid, CameraSccbAddr, CameraSensor, CameraStatus, FrameSize,
    PixelFormat, Reg16, SccbBus, SensorId,
};

pub const MODEL_ID_H: u16 = 0x0000;
pub const MODEL_ID_L: u16 = 0x0001;
pub const SILICON_REV: u16 = 0x0002;
pub const FRAME_COUNT_H: u16 = 0x0005;
pub const FRAME_COUNT_L: u16 = 0x0006;
pub const PIXEL_ORDER: u16 = 0x0007;

pub const MODEL_SELECT: u16 = 0x0100;
pub const IMAGE_ORIENTATION: u16 = 0x0101;
pub const EMBEDDED_LINE_EN: u16 = 0x0102;
pub const SW_RESET: u16 = 0x0103;
pub const COMMAND_UPDATE: u16 = 0x0104;

pub const INTEGRATION_H: u16 = 0x0202;
pub const INTEGRATION_L: u16 = 0x0203;
pub const ANALOG_GAIN: u16 = 0x0205;
pub const DIGITAL_GAIN_H: u16 = 0x020E;
pub const DIGITAL_GAIN_L: u16 = 0x020F;

pub const PLL1CFG: u16 = 0x0300;
pub const PLL2CFG: u16 = 0x0301;
pub const PLL3CFG: u16 = 0x0302;

pub const FRAME_LENGTH_LINES_H: u16 = 0x0340;
pub const FRAME_LENGTH_LINES_L: u16 = 0x0341;
pub const LINE_LENGTH_PCK_H: u16 = 0x0342;
pub const LINE_LENGTH_PCK_L: u16 = 0x0343;

pub const MONO_MODE: u16 = 0x0370;
pub const MONO_MODE_ISP: u16 = 0x0371;
pub const MONO_MODE_SEL: u16 = 0x0372;

pub const H_SUB: u16 = 0x0380;
pub const V_SUB: u16 = 0x0381;
pub const BINNING_MODE: u16 = 0x0382;

pub const TEST_PATTERN_MODE: u16 = 0x0601;
pub const BLC_TGT: u16 = 0x1004;
pub const BLC2_TGT: u16 = 0x1009;
pub const MONO_CTRL: u16 = 0x100A;
pub const OPFM_CTRL: u16 = 0x1014;
pub const CMPRS_CTRL: u16 = 0x102F;

pub const AE_CTRL: u16 = 0x2000;
pub const AE_CTRL1: u16 = 0x2001;
pub const AE_TARGET_MEAN: u16 = 0x2034;
pub const AE_MIN_MEAN: u16 = 0x2035;
pub const AE_TARGET_ZONE: u16 = 0x2036;

pub const PMU_CFG_3: u16 = 0x3024;
pub const PMU_CFG_4: u16 = 0x3025;
pub const WIN_MODE: u16 = 0x3030;
pub const PAD_REGISTER_07: u16 = 0x3112;

pub const REG_DLY: u16 = 0xFFFF;
pub const REGLIST_TAIL: u16 = 0x0000;

pub const SENSOR_DEFAULT_REGS: &[Reg16] = &[
    Reg16::new(0x0103, 0x00),
    Reg16::new(REG_DLY, 100),
    Reg16::new(0x3035, 0x01),
    Reg16::new(0x3126, 0x03),
    Reg16::new(0x3128, 0x57),
    Reg16::new(0x3142, 0x9F),
    Reg16::new(0x311C, 0x10),
    Reg16::new(0x3115, 0x42),
    Reg16::new(0x3116, 0x10),
    Reg16::new(0x3117, 0x0A),
    Reg16::new(0x310B, 0x10),
    Reg16::new(0x3031, 0x01),
    Reg16::new(0x312E, 0x00),
    Reg16::new(0x30D7, 0x00),
    Reg16::new(0x30DC, 0x00),
    Reg16::new(0x30E1, 0x00),
    Reg16::new(0x30E6, 0x00),
    Reg16::new(0x30EB, 0x00),
    Reg16::new(0x30F0, 0x00),
    Reg16::new(0x30F5, 0x00),
    Reg16::new(0x30FA, 0x00),
    Reg16::new(0x30FF, 0x00),
    Reg16::new(0x3104, 0x00),
    Reg16::new(0x30D8, 0xA7),
    Reg16::new(0x30DD, 0x27),
    Reg16::new(0x30E2, 0x27),
    Reg16::new(0x30E7, 0x27),
    Reg16::new(0x30EC, 0x27),
    Reg16::new(0x30F1, 0xA7),
    Reg16::new(0x30F6, 0x27),
    Reg16::new(0x30FB, 0x27),
    Reg16::new(0x3100, 0x27),
    Reg16::new(0x3105, 0x27),
    Reg16::new(0x30D9, 0x00),
    Reg16::new(0x30DE, 0x05),
    Reg16::new(0x30E3, 0x05),
    Reg16::new(0x30E8, 0x05),
    Reg16::new(0x30ED, 0x05),
    Reg16::new(0x30F2, 0x00),
    Reg16::new(0x30F7, 0x05),
    Reg16::new(0x30FC, 0x05),
    Reg16::new(0x3101, 0x05),
    Reg16::new(0x3106, 0x05),
    Reg16::new(0x30C4, 0x10),
    Reg16::new(0x30C5, 0x01),
    Reg16::new(0x30C6, 0x2F),
    Reg16::new(0x30CB, 0xFF),
    Reg16::new(0x30CC, 0xFF),
    Reg16::new(0x30CD, 0x7F),
    Reg16::new(0x30CE, 0x7F),
    Reg16::new(0x30D3, 0x01),
    Reg16::new(0x30D4, 0xFF),
    Reg16::new(0x311F, 0x0E),
    Reg16::new(0x3120, 0x0D),
    Reg16::new(0x3121, 0x0F),
    Reg16::new(0x3122, 0x00),
    Reg16::new(0x3147, 0x18),
    Reg16::new(0x314B, 0x01),
    Reg16::new(0x3149, 0x28),
    Reg16::new(0x3150, 0x50),
    Reg16::new(0x3152, 0x00),
    Reg16::new(0x3156, 0x2C),
    Reg16::new(0x3163, 0x1F),
    Reg16::new(0x3165, 0x7F),
    Reg16::new(0x312B, 0x41),
    Reg16::new(0x3113, 0xA0),
    Reg16::new(0x3114, 0x67),
    Reg16::new(0x317D, 0x02),
    Reg16::new(0x3160, 0x1F),
    Reg16::new(0x318C, 0x00),
    Reg16::new(0x315C, 0xE0),
    Reg16::new(0x311E, 0x0F),
    Reg16::new(0x30D5, 0x00),
    Reg16::new(0x30DA, 0x01),
    Reg16::new(0x30DF, 0x07),
    Reg16::new(0x30E4, 0x47),
    Reg16::new(0x30E9, 0x87),
    Reg16::new(0x30FD, 0x47),
    Reg16::new(0x3102, 0x87),
    Reg16::new(0x311D, 0x06),
    Reg16::new(0x3141, 0x2A),
    Reg16::new(0x315A, 0x0A),
    Reg16::new(0x30D6, 0x40),
    Reg16::new(0x30DB, 0x40),
    Reg16::new(0x30E0, 0x40),
    Reg16::new(0x30E5, 0x30),
    Reg16::new(0x30EA, 0x30),
    Reg16::new(0x30EF, 0x40),
    Reg16::new(0x30F4, 0x40),
    Reg16::new(0x30F9, 0x40),
    Reg16::new(0x30FE, 0x30),
    Reg16::new(0x3103, 0x30),
    Reg16::new(0x3164, 0x7F),
    Reg16::new(0x282A, 0x0F),
    Reg16::new(0x282E, 0x2F),
    Reg16::new(0x282B, 0x08),
    Reg16::new(0x312A, 0x11),
    Reg16::new(0x1000, 0x43),
    Reg16::new(0x1001, 0x80),
    Reg16::new(0x0350, 0xE0),
    Reg16::new(0x101D, 0xCF),
    Reg16::new(0x1021, 0x5D),
    Reg16::new(0x3500, 0x74),
    Reg16::new(0x3501, 0x0A),
    Reg16::new(0x3502, 0x77),
    Reg16::new(0x3503, 0x04),
    Reg16::new(0x3504, 0x14),
    Reg16::new(0x3505, 0x03),
    Reg16::new(0x3506, 0x00),
    Reg16::new(0x3507, 0x00),
    Reg16::new(0x3508, 0x00),
    Reg16::new(0x3509, 0x00),
    Reg16::new(0x350A, 0xFF),
    Reg16::new(0x350B, 0x00),
    Reg16::new(0x350D, 0x01),
    Reg16::new(0x350C, 0x00),
    Reg16::new(0x350F, 0x00),
    Reg16::new(0x3510, 0x01),
    Reg16::new(0x3513, 0x00),
    Reg16::new(0x351C, 0x00),
    Reg16::new(0x3514, 0x00),
    Reg16::new(0x3515, 0x01),
    Reg16::new(0x3516, 0x00),
    Reg16::new(0x3517, 0x02),
    Reg16::new(0x3518, 0x00),
    Reg16::new(0x3519, 0x7F),
    Reg16::new(0x351A, 0x00),
    Reg16::new(0x351B, 0x5F),
    Reg16::new(0x351D, 0x04),
    Reg16::new(0x351E, 0x10),
    Reg16::new(0x352A, 0x01),
    Reg16::new(0x352B, 0x04),
    Reg16::new(0x352C, 0x01),
    Reg16::new(0x352D, 0x38),
    Reg16::new(0x354B, 0x21),
    Reg16::new(0x354C, 0x01),
    Reg16::new(0x354D, 0xE0),
    Reg16::new(0x354E, 0xF0),
    Reg16::new(0x354F, 0x10),
    Reg16::new(0x3550, 0x10),
    Reg16::new(0x3551, 0x10),
    Reg16::new(0x3552, 0x20),
    Reg16::new(0x3553, 0x10),
    Reg16::new(0x3554, 0x01),
    Reg16::new(0x3555, 0x06),
    Reg16::new(0x3556, 0x0C),
    Reg16::new(0x3557, 0x12),
    Reg16::new(0x3558, 0x1C),
    Reg16::new(0x3559, 0x30),
    Reg16::new(0x3549, 0x04),
    Reg16::new(0x354A, 0x35),
    Reg16::new(0x355A, 0x74),
    Reg16::new(0x355B, 0x0A),
    Reg16::new(0x355C, 0x77),
    Reg16::new(0x355D, 0x04),
    Reg16::new(0x355E, 0x14),
    Reg16::new(0x355F, 0x03),
    Reg16::new(0x3560, 0x00),
    Reg16::new(0x3561, 0x01),
    Reg16::new(0x3562, 0x01),
    Reg16::new(0x3563, 0x00),
    Reg16::new(0x3564, 0xFF),
    Reg16::new(0x3565, 0x00),
    Reg16::new(0x3567, 0x01),
    Reg16::new(0x3566, 0x00),
    Reg16::new(0x3569, 0x00),
    Reg16::new(0x356A, 0x01),
    Reg16::new(0x356D, 0x00),
    Reg16::new(0x3576, 0x00),
    Reg16::new(0x356E, 0x00),
    Reg16::new(0x356F, 0x01),
    Reg16::new(0x3570, 0x00),
    Reg16::new(0x3571, 0x02),
    Reg16::new(0x3572, 0x00),
    Reg16::new(0x3573, 0x3F),
    Reg16::new(0x3574, 0x00),
    Reg16::new(0x3575, 0x2F),
    Reg16::new(0x3577, 0x04),
    Reg16::new(0x3578, 0x10),
    Reg16::new(0x3584, 0x01),
    Reg16::new(0x3585, 0x04),
    Reg16::new(0x3586, 0x01),
    Reg16::new(0x3587, 0x38),
    Reg16::new(0x3588, 0x02),
    Reg16::new(0x3589, 0x12),
    Reg16::new(0x358A, 0x04),
    Reg16::new(0x358B, 0x24),
    Reg16::new(0x358C, 0x06),
    Reg16::new(0x358D, 0x36),
    Reg16::new(0x35A5, 0x21),
    Reg16::new(0x35A6, 0x01),
    Reg16::new(0x35A7, 0xE0),
    Reg16::new(0x35A8, 0xF0),
    Reg16::new(0x35A9, 0x10),
    Reg16::new(0x35AA, 0x10),
    Reg16::new(0x35AB, 0x10),
    Reg16::new(0x35AC, 0x20),
    Reg16::new(0x35AD, 0x10),
    Reg16::new(0x35AE, 0x01),
    Reg16::new(0x35AF, 0x06),
    Reg16::new(0x35B0, 0x0C),
    Reg16::new(0x35B1, 0x12),
    Reg16::new(0x35B2, 0x1C),
    Reg16::new(0x35B3, 0x30),
    Reg16::new(0x35A3, 0x02),
    Reg16::new(0x35A4, 0x03),
    Reg16::new(0x3512, 0x3F),
    Reg16::new(0x351F, 0x04),
    Reg16::new(0x3520, 0x03),
    Reg16::new(0x3521, 0x00),
    Reg16::new(0x3523, 0x60),
    Reg16::new(0x3524, 0x08),
    Reg16::new(0x3525, 0x19),
    Reg16::new(0x3526, 0x08),
    Reg16::new(0x3527, 0x10),
    Reg16::new(0x356C, 0x3F),
    Reg16::new(0x3579, 0x04),
    Reg16::new(0x357A, 0x03),
    Reg16::new(0x357B, 0x00),
    Reg16::new(0x357D, 0x60),
    Reg16::new(0x357E, 0x08),
    Reg16::new(0x357F, 0x19),
    Reg16::new(0x3580, 0x08),
    Reg16::new(0x3581, 0x10),
    Reg16::new(0x2048, 0x00),
    Reg16::new(0x2049, 0x10),
    Reg16::new(0x204A, 0x40),
    Reg16::new(0x204E, 0x00),
    Reg16::new(0x204F, 0x38),
    Reg16::new(0x2050, 0xE0),
    Reg16::new(0x204B, 0x00),
    Reg16::new(0x204C, 0x08),
    Reg16::new(0x204D, 0x20),
    Reg16::new(0x2051, 0x00),
    Reg16::new(0x2052, 0x1C),
    Reg16::new(0x2053, 0x70),
    Reg16::new(0x2054, 0x00),
    Reg16::new(0x2055, 0x1A),
    Reg16::new(0x2056, 0xC0),
    Reg16::new(0x2057, 0x00),
    Reg16::new(0x2058, 0x06),
    Reg16::new(0x2059, 0xB0),
    Reg16::new(0x2080, 0x41),
    Reg16::new(0x2083, 0x01),
    Reg16::new(0x208D, 0x02),
    Reg16::new(0x208E, 0x08),
    Reg16::new(0x208F, 0x0D),
    Reg16::new(0x2090, 0x14),
    Reg16::new(0x2091, 0x1D),
    Reg16::new(0x2092, 0x30),
    Reg16::new(0x2093, 0x08),
    Reg16::new(0x2094, 0x0A),
    Reg16::new(0x2095, 0x0F),
    Reg16::new(0x2096, 0x14),
    Reg16::new(0x2097, 0x18),
    Reg16::new(0x2098, 0x20),
    Reg16::new(0x2099, 0x10),
    Reg16::new(0x209A, 0x00),
    Reg16::new(0x209B, 0x01),
    Reg16::new(0x209C, 0x01),
    Reg16::new(0x209D, 0x11),
    Reg16::new(0x209E, 0x06),
    Reg16::new(0x1030, 0x04),
    Reg16::new(0x1031, 0x08),
    Reg16::new(0x1032, 0x10),
    Reg16::new(0x1033, 0x18),
    Reg16::new(0x1034, 0x20),
    Reg16::new(0x1035, 0x28),
    Reg16::new(0x1036, 0x30),
    Reg16::new(0x1037, 0x38),
    Reg16::new(0x1038, 0x40),
    Reg16::new(0x1039, 0x50),
    Reg16::new(0x103A, 0x60),
    Reg16::new(0x103B, 0x70),
    Reg16::new(0x103C, 0x80),
    Reg16::new(0x103D, 0xA0),
    Reg16::new(0x103E, 0xC0),
    Reg16::new(0x103F, 0xE0),
    Reg16::new(0x35B4, 0x74),
    Reg16::new(0x35B5, 0x0A),
    Reg16::new(0x35B6, 0x77),
    Reg16::new(0x35B7, 0x00),
    Reg16::new(0x35B8, 0x94),
    Reg16::new(0x35B9, 0x03),
    Reg16::new(0x35BA, 0x00),
    Reg16::new(0x35BB, 0x03),
    Reg16::new(0x35BD, 0x00),
    Reg16::new(0x35BE, 0xFF),
    Reg16::new(0x35BF, 0x00),
    Reg16::new(0x35C1, 0x01),
    Reg16::new(0x35C0, 0x01),
    Reg16::new(0x35C3, 0x00),
    Reg16::new(0x35C4, 0x00),
    Reg16::new(0x35C6, 0x3F),
    Reg16::new(0x35C7, 0x00),
    Reg16::new(0x35D0, 0x00),
    Reg16::new(0x35D3, 0x04),
    Reg16::new(0x35D7, 0x18),
    Reg16::new(0x35D8, 0x01),
    Reg16::new(0x35D9, 0x20),
    Reg16::new(0x35DA, 0x08),
    Reg16::new(0x35DB, 0x14),
    Reg16::new(0x35DC, 0x70),
    Reg16::new(0x35C8, 0x00),
    Reg16::new(0x35C9, 0x01),
    Reg16::new(0x35CA, 0x00),
    Reg16::new(0x35CB, 0x02),
    Reg16::new(0x35CC, 0x00),
    Reg16::new(0x35CD, 0x0F),
    Reg16::new(0x35CE, 0x00),
    Reg16::new(0x35CF, 0x0B),
    Reg16::new(0x35DE, 0x00),
    Reg16::new(0x35DF, 0x01),
    Reg16::new(0x35FD, 0x00),
    Reg16::new(0x35FE, 0x5E),
    Reg16::new(0x3024, 0x00),
    Reg16::new(0x3025, 0x12),
    Reg16::new(0x3026, 0x03),
    Reg16::new(0x3027, 0x81),
    Reg16::new(0x3028, 0x01),
    Reg16::new(0x3029, 0x00),
    Reg16::new(0x302A, 0x30),
    Reg16::new(0x2061, 0x00),
    Reg16::new(0x2062, 0x00),
    Reg16::new(0x2063, 0xC8),
    Reg16::new(0x1014, 0x00),
    Reg16::new(0x102F, 0x08),
    Reg16::new(0x309E, 0x05),
    Reg16::new(0x309F, 0x02),
    Reg16::new(0x30A0, 0x02),
    Reg16::new(0x30A1, 0x00),
    Reg16::new(0x30A2, 0x08),
    Reg16::new(0x30A3, 0x00),
    Reg16::new(0x30A4, 0x20),
    Reg16::new(0x30A5, 0x04),
    Reg16::new(0x30A6, 0x02),
    Reg16::new(0x30A7, 0x02),
    Reg16::new(0x30A8, 0x01),
    Reg16::new(0x30B0, 0x03),
    Reg16::new(0x3112, 0x04),
    Reg16::new(0x311A, 0x30),
    Reg16::new(0x2800, 0x00),
    Reg16::new(0x0370, 0x00),
    Reg16::new(0x0371, 0x00),
    Reg16::new(0x0372, 0x01),
    Reg16::new(0x100A, 0x05),
    Reg16::new(0x2590, 0x01),
    Reg16::new(0x0104, 0x01),
    Reg16::new(0x0100, 0x01),
];

pub const SENSOR_FRAMESIZE_VGA: &[Reg16] = &[
    Reg16::new(0x0006, 0x10),
    Reg16::new(0x000D, 0x00),
    Reg16::new(0x000E, 0x00),
    Reg16::new(0x0122, 0xEB),
    Reg16::new(0x0125, 0xFF),
    Reg16::new(0x0126, 0x70),
    Reg16::new(0x05E0, 0xC1),
    Reg16::new(0x05E1, 0x00),
    Reg16::new(0x05E2, 0xC1),
    Reg16::new(0x05E3, 0x00),
    Reg16::new(0x05E4, 0x03),
    Reg16::new(0x05E5, 0x00),
    Reg16::new(0x05E6, 0x82),
    Reg16::new(0x05E7, 0x02),
    Reg16::new(0x05E8, 0x04),
    Reg16::new(0x05E9, 0x00),
    Reg16::new(0x05EA, 0xE3),
    Reg16::new(0x05EB, 0x01),
];

pub const SENSOR_FRAMESIZE_QVGA: &[Reg16] = &[
    Reg16::new(0x355A, 0x74),
    Reg16::new(0x355B, 0x0A),
    Reg16::new(0x355C, 0x77),
    Reg16::new(0x355D, 0x01),
    Reg16::new(0x355E, 0x1C),
    Reg16::new(0x355F, 0x03),
    Reg16::new(0x3560, 0x00),
    Reg16::new(0x3561, 0x01),
    Reg16::new(0x3562, 0x01),
    Reg16::new(0x3563, 0x00),
    Reg16::new(0x3564, 0xFF),
    Reg16::new(0x3565, 0x00),
    Reg16::new(0x3567, 0x01),
    Reg16::new(0x3566, 0x00),
    Reg16::new(0x3569, 0x00),
    Reg16::new(0x356A, 0x01),
    Reg16::new(0x356C, 0x7F),
    Reg16::new(0x356D, 0x00),
    Reg16::new(0x3576, 0x00),
    Reg16::new(0x3579, 0x03),
    Reg16::new(0x356E, 0x00),
    Reg16::new(0x356F, 0x01),
    Reg16::new(0x3570, 0x00),
    Reg16::new(0x3571, 0x02),
    Reg16::new(0x3572, 0x00),
    Reg16::new(0x3573, 0x3F),
    Reg16::new(0x3574, 0x00),
    Reg16::new(0x3575, 0x2F),
    Reg16::new(0x3577, 0x04),
    Reg16::new(0x3578, 0x24),
    Reg16::new(0x3584, 0x01),
    Reg16::new(0x3585, 0x04),
    Reg16::new(0x3586, 0x01),
    Reg16::new(0x3587, 0x38),
    Reg16::new(0x3588, 0x02),
    Reg16::new(0x3589, 0x12),
    Reg16::new(0x358A, 0x04),
    Reg16::new(0x358B, 0x24),
    Reg16::new(0x358C, 0x06),
    Reg16::new(0x358D, 0x36),
    Reg16::new(0x35A5, 0x21),
    Reg16::new(0x35A6, 0x01),
    Reg16::new(0x35A7, 0xE0),
    Reg16::new(0x35A8, 0xF0),
    Reg16::new(0x35A9, 0x10),
    Reg16::new(0x35AA, 0x10),
    Reg16::new(0x35AB, 0x10),
    Reg16::new(0x35AC, 0x20),
    Reg16::new(0x35AD, 0x10),
    Reg16::new(0x35AE, 0x01),
    Reg16::new(0x35AF, 0x06),
    Reg16::new(0x35B0, 0x0C),
    Reg16::new(0x35B1, 0x12),
    Reg16::new(0x35BC, 0x1C),
    Reg16::new(0x35B3, 0x30),
    Reg16::new(0x35A3, 0x02),
    Reg16::new(0x35A4, 0x03),
];

pub const SENSOR_FRAMESIZE_240X240: &[Reg16] = &[
    Reg16::new(0x0006, 0x00),
    Reg16::new(0x000D, 0x01),
    Reg16::new(0x000E, 0x11),
    Reg16::new(0x0122, 0xFB),
    Reg16::new(0x0125, 0xFF),
    Reg16::new(0x0126, 0x70),
    Reg16::new(0x05E0, 0xBE),
    Reg16::new(0x05E1, 0x00),
    Reg16::new(0x05E2, 0xBE),
    Reg16::new(0x05E3, 0x00),
    Reg16::new(0x05E4, 0x62),
    Reg16::new(0x05E5, 0x00),
    Reg16::new(0x05E6, 0x51),
    Reg16::new(0x05E7, 0x01),
    Reg16::new(0x05E8, 0x04),
    Reg16::new(0x05E9, 0x00),
    Reg16::new(0x05EA, 0xF3),
    Reg16::new(0x05EB, 0x00),
];

pub const SENSOR_FRAMESIZE_QQVGA: &[Reg16] = &[
    Reg16::new(0x355A, 0x74),
    Reg16::new(0x355B, 0x0A),
    Reg16::new(0x355C, 0x77),
    Reg16::new(0x355D, 0x00),
    Reg16::new(0x355E, 0x8E),
    Reg16::new(0x355F, 0x03),
    Reg16::new(0x3560, 0x00),
    Reg16::new(0x3561, 0x02),
    Reg16::new(0x3562, 0x02),
    Reg16::new(0x3563, 0x00),
    Reg16::new(0x3564, 0xFF),
    Reg16::new(0x3565, 0x00),
    Reg16::new(0x3567, 0x01),
    Reg16::new(0x3566, 0x00),
    Reg16::new(0x3569, 0x00),
    Reg16::new(0x356A, 0x01),
    Reg16::new(0x356C, 0x7F),
    Reg16::new(0x356D, 0x00),
    Reg16::new(0x3576, 0x00),
    Reg16::new(0x3579, 0x03),
    Reg16::new(0x356E, 0x00),
    Reg16::new(0x356F, 0x01),
    Reg16::new(0x3570, 0x00),
    Reg16::new(0x3571, 0x02),
    Reg16::new(0x3572, 0x00),
    Reg16::new(0x3573, 0x1F),
    Reg16::new(0x3574, 0x00),
    Reg16::new(0x3575, 0x17),
    Reg16::new(0x3577, 0x04),
    Reg16::new(0x3578, 0x24),
    Reg16::new(0x3584, 0x01),
    Reg16::new(0x3585, 0x04),
    Reg16::new(0x3586, 0x01),
    Reg16::new(0x3587, 0x38),
    Reg16::new(0x3588, 0x02),
    Reg16::new(0x3589, 0x12),
    Reg16::new(0x358A, 0x04),
    Reg16::new(0x358B, 0x24),
    Reg16::new(0x358C, 0x06),
    Reg16::new(0x358D, 0x36),
    Reg16::new(0x35A5, 0x21),
    Reg16::new(0x35A6, 0x01),
    Reg16::new(0x35A7, 0xD0),
    Reg16::new(0x35A8, 0xF0),
    Reg16::new(0x35A9, 0x10),
    Reg16::new(0x35AA, 0x10),
    Reg16::new(0x35AB, 0x10),
    Reg16::new(0x35AC, 0x20),
    Reg16::new(0x35AD, 0x10),
    Reg16::new(0x35AE, 0x01),
    Reg16::new(0x35AF, 0x06),
    Reg16::new(0x35B0, 0x0C),
    Reg16::new(0x35B1, 0x12),
    Reg16::new(0x35B2, 0x1C),
    Reg16::new(0x35B3, 0x30),
    Reg16::new(0x35A3, 0x00),
    Reg16::new(0x35A4, 0xEA),
];

#[inline]
fn set_reg_bits<B: SccbBus>(
    bus: &mut B,
    slv_addr: u8,
    reg: u16,
    offset: u8,
    mask: u8,
    value: u8,
) -> Result<(), CameraError> {
    let c_value = bus.read_reg16(slv_addr, reg)?;
    let shifted_mask = mask << offset;
    let new_value = (c_value & !shifted_mask) | ((value & mask) << offset);
    bus.write_reg16(slv_addr, reg, new_value)
}

#[inline]
fn check_reg_mask<B: SccbBus>(bus: &mut B, slv_addr: u8, reg: u16, mask: u8) -> Result<u8, CameraError> {
    let val = bus.read_reg16(slv_addr, reg)?;
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
            bus.write_reg16(slv_addr, entry.reg, entry.val)?;
        }
    }
    Ok(())
}

/// Detects an HM0360 sensor at `slv_addr` (`0x12` or `0x24`).
pub fn detect<B: SccbBus>(bus: &mut B, slv_addr: u8) -> Result<SensorId, CameraError> {
    Hm0360::detect(bus, slv_addr)
}

/// Safe `#![no_std]` driver for the Himax HM0360 VGA grayscale camera sensor.
#[derive(Debug, Clone)]
pub struct Hm0360 {
    id: SensorId,
    status: CameraStatus,
    slv_addr: u8,
    pixformat: PixelFormat,
    xclk_freq_hz: u32,
}

pub type HM0360 = Hm0360;

impl Default for Hm0360 {
    fn default() -> Self {
        Self::new()
    }
}

impl Hm0360 {
    /// Creates a new [`Hm0360`] instance with default SCCB address (`0x12`) and PID (`0x0360`).
    pub const fn new() -> Self {
        Self {
            id: SensorId {
                midh: 0,
                midl: 0,
                pid: CameraPid::HM0360.as_u16(),
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
            slv_addr: CameraSccbAddr::HM0360,
            pixformat: PixelFormat::Grayscale,
            xclk_freq_hz: 24_000_000,
        }
    }

    /// Creates a new [`Hm0360`] with an explicit slave address and [`SensorId`].
    pub const fn with_addr(slv_addr: u8, id: SensorId) -> Self {
        Self {
            id,
            slv_addr,
            ..Self::new()
        }
    }

    /// Probes the SCCB bus at `slv_addr` (`0x12` or `0x24`) and verifies `HM0360_PID` (`0x0360`).
    pub fn detect<B: SccbBus>(bus: &mut B, slv_addr: u8) -> Result<SensorId, CameraError> {
        if slv_addr != CameraSccbAddr::HM0360 && slv_addr != CameraSccbAddr::HM1055 {
            return Err(CameraError::NotDetected);
        }
        let h = bus.read_reg16(slv_addr, MODEL_ID_H)? as u16;
        let l = bus.read_reg16(slv_addr, MODEL_ID_L)? as u16;
        let pid = (h << 8) | l;
        if pid == CameraPid::HM0360.as_u16() {
            let ver = bus.read_reg16(slv_addr, SILICON_REV)?;
            Ok(SensorId {
                midh: 0,
                midl: 0,
                pid,
                ver,
            })
        } else {
            Err(CameraError::NotDetected)
        }
    }

    /// Returns the 7-bit SCCB slave address (`0x12`).
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

    /// Returns the [`CameraModel`] (`CameraModel::Hm0360`).
    pub const fn model(&self) -> CameraModel {
        CameraModel::Hm0360
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

impl<B: SccbBus> CameraSensor<B> for Hm0360 {
    fn model(&self) -> CameraModel {
        CameraModel::Hm0360
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
        bus.delay_ms(100);
        bus.write_reg16(self.slv_addr, SW_RESET, 0x00)?;
        bus.delay_ms(100);
        write_reg_list(bus, self.slv_addr, SENSOR_DEFAULT_REGS)?;
        bus.delay_ms(100);
        Ok(())
    }

    fn init_status(&mut self, bus: &mut B) -> Result<(), CameraError> {
        self.status.brightness = 0;
        self.status.contrast = 0;
        self.status.saturation = 0;
        self.status.sharpness = 0;
        self.status.denoise = 0;
        self.status.ae_level = 0;
        self.status.awb = 0;
        self.status.aec = 0;
        self.status.hmirror = check_reg_mask(bus, self.slv_addr, IMAGE_ORIENTATION, 0x01)?;
        self.status.vflip = check_reg_mask(bus, self.slv_addr, IMAGE_ORIENTATION, 0x02)?;
        self.status.lenc = 0;
        self.status.awb_gain = 0;
        self.status.agc_gain = 0;
        self.status.aec_value = 0;
        Ok(())
    }

    fn set_pixformat(&mut self, _bus: &mut B, pixformat: PixelFormat) -> Result<(), CameraError> {
        match pixformat {
            PixelFormat::Grayscale => {
                self.pixformat = pixformat;
                Ok(())
            }
            _ => Err(CameraError::NotSupported),
        }
    }

    fn set_framesize(&mut self, bus: &mut B, framesize: FrameSize) -> Result<(), CameraError> {
        self.status.framesize = framesize;
        write_reg_list(bus, self.slv_addr, SENSOR_DEFAULT_REGS)?;
        match framesize {
            FrameSize::Qqvga => {
                write_reg_list(bus, self.slv_addr, SENSOR_FRAMESIZE_QQVGA)?;
                set_reg_bits(bus, self.slv_addr, PMU_CFG_3, 0, 0x01, 1)?;
            }
            FrameSize::Qvga => {
                write_reg_list(bus, self.slv_addr, SENSOR_FRAMESIZE_QVGA)?;
                set_reg_bits(bus, self.slv_addr, PMU_CFG_3, 0, 0x01, 1)?;
            }
            FrameSize::Vga => {
                set_reg_bits(bus, self.slv_addr, PMU_CFG_3, 0, 0x01, 0)?;
            }
            _ => {
                set_reg_bits(bus, self.slv_addr, PMU_CFG_3, 0, 0x01, 0)?;
            }
        }
        self.set_pll(bus, 0, 0, 0, 0, 0, 0, 0, 0)?;
        bus.write_reg16(self.slv_addr, COMMAND_UPDATE, 0x01)
    }

    fn set_brightness(&mut self, bus: &mut B, level: i32) -> Result<(), CameraError> {
        let ae_mean = match level {
            0 => 60,
            1 => 80,
            2 => 100,
            3 => 127,
            _ => 80,
        };
        bus.write_reg16(self.slv_addr, AE_TARGET_MEAN, ae_mean)?;
        self.status.brightness = level as i8;
        Ok(())
    }

    fn set_colorbar(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        let val = u8::from(enable);
        set_reg_bits(bus, self.slv_addr, TEST_PATTERN_MODE, 0, 0x01, val)?;
        self.status.colorbar = val;
        Ok(())
    }

    fn set_exposure_ctrl(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        let val = u8::from(enable);
        set_reg_bits(bus, self.slv_addr, AE_CTRL, 0, 0x01, val)?;
        self.status.aec = val;
        Ok(())
    }

    fn set_hmirror(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        let val = u8::from(enable);
        set_reg_bits(bus, self.slv_addr, IMAGE_ORIENTATION, 0, 0x01, val)?;
        self.status.hmirror = val;
        Ok(())
    }

    fn set_vflip(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        let val = u8::from(enable);
        set_reg_bits(bus, self.slv_addr, IMAGE_ORIENTATION, 1, 0x01, val)?;
        self.status.vflip = val;
        Ok(())
    }

    fn get_reg(&mut self, bus: &mut B, reg: i32, mask: i32) -> Result<i32, CameraError> {
        if !(0..=0xFFFF).contains(&reg) || mask < 0 {
            return Err(CameraError::InvalidArg);
        }
        let reg16 = reg as u16;
        let val = if mask > 0xFF {
            let mut v = bus.read_reg16_val16(self.slv_addr, reg16)? as i32;
            if mask > 0xFFFF {
                let v2 = bus.read_reg16(self.slv_addr, reg16.wrapping_add(2))? as i32;
                v = (v << 8) | v2;
            }
            v
        } else {
            bus.read_reg16(self.slv_addr, reg16)? as i32
        };
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
        let reg16 = reg as u16;
        let current = if mask > 0xFF {
            let mut v = bus.read_reg16_val16(self.slv_addr, reg16)? as i32;
            if mask > 0xFFFF {
                let v2 = bus.read_reg16(self.slv_addr, reg16.wrapping_add(2))? as i32;
                v = (v << 8) | v2;
            }
            v
        } else {
            bus.read_reg16(self.slv_addr, reg16)? as i32
        };

        let next = (current & !mask) | (value & mask);
        if mask > 0xFFFF {
            bus.write_reg16_val16(self.slv_addr, reg16, ((next >> 8) & 0xFFFF) as u16)?;
            bus.write_reg16(self.slv_addr, reg16.wrapping_add(2), (next & 0xFF) as u8)
        } else if mask > 0xFF {
            bus.write_reg16_val16(self.slv_addr, reg16, (next & 0xFFFF) as u16)
        } else {
            bus.write_reg16(self.slv_addr, reg16, (next & 0xFF) as u8)
        }
    }

    fn set_pll(
        &mut self,
        bus: &mut B,
        _bypass: i32,
        _mul: i32,
        _sys: i32,
        _root: i32,
        _pre: i32,
        _seld5: i32,
        _pclken: i32,
        _pclk: i32,
    ) -> Result<(), CameraError> {
        let value = if self.xclk_freq_hz <= 6_000_000 {
            0x03
        } else if self.xclk_freq_hz <= 12_000_000 {
            0x02
        } else if self.xclk_freq_hz <= 18_000_000 {
            0x01
        } else {
            0x00
        };
        let pll_cfg = bus.read_reg16(self.slv_addr, PLL1CFG)?;
        bus.write_reg16(self.slv_addr, PLL1CFG, (pll_cfg & 0xFC) | value)
    }

    fn set_xclk(&mut self, _bus: &mut B, _timer: i32, xclk: i32) -> Result<(), CameraError> {
        if xclk <= 0 {
            return Err(CameraError::InvalidArg);
        }
        self.xclk_freq_hz = (xclk as u32).saturating_mul(1_000_000);
        Ok(())
    }
}
