//! HM1055 720p HD YUV422/RGB565/RAW CMOS image sensor driver.
//!
//! Ported from `sensors/hm1055.c`, `sensors/private_include/hm1055_regs.h`, and
//! `sensors/private_include/hm1055_settings.h`.

use crate::{
    CameraError, CameraModel, CameraPid, CameraSccbAddr, CameraSensor, CameraStatus, FrameSize,
    PixelFormat, Reg16, SccbBus, SensorId,
};

pub const CMU: u16 = 0x0000;
pub const IDH: u16 = 0x0001;
pub const IDL: u16 = 0x0002;
pub const VID: u16 = 0x0003;
pub const PWDCTRL: u16 = 0x0004;
pub const TGRDCFG: u16 = 0x0005;
pub const RDCFG: u16 = 0x0006;
pub const VREAD: u16 = 0x000D;
pub const HREAD: u16 = 0x000E;
pub const IMG_CFG: u16 = 0x000F;

pub const BLNKRH: u16 = 0x0010;
pub const BLNKRL: u16 = 0x0011;
pub const BLNKCCLK: u16 = 0x0012;
pub const BLNKC: u16 = 0x0013;
pub const INTGH: u16 = 0x0015;
pub const INTGL: u16 = 0x0016;
pub const AGAIN: u16 = 0x0018;
pub const DGAIN: u16 = 0x001D;

pub const OPRTCFG: u16 = 0x0020;
pub const SFTRST: u16 = 0x0022;
pub const IOCTRLH: u16 = 0x0023;
pub const IOCTRLL: u16 = 0x0024;
pub const CKCFG1: u16 = 0x0025;
pub const CKCFG2: u16 = 0x0026;
pub const PORTCTRL: u16 = 0x0027;
pub const TESTIMG: u16 = 0x0028;
pub const CCIR656: u16 = 0x0029;
pub const PLL1: u16 = 0x002A;
pub const CKCFG3: u16 = 0x002B;
pub const PLL2: u16 = 0x002C;

pub const BLCTGT: u16 = 0x0040;
pub const BLCTGT2: u16 = 0x0053;

pub const VAWINSTARTH: u16 = 0x0078;
pub const VAWINSTARTL: u16 = 0x0079;
pub const VAWINENDH: u16 = 0x007A;
pub const VAWINENDL: u16 = 0x007B;

pub const CMU_AE: u16 = 0x0100;
pub const CMU_AWB: u16 = 0x0101;
pub const ISPID: u16 = 0x0105;
pub const ISPCTRL1: u16 = 0x0120;
pub const ISPCTRL2: u16 = 0x0121;
pub const ISPCTRL3: u16 = 0x0122;
pub const ISPCTRL4: u16 = 0x0124;
pub const ISPCTRL5: u16 = 0x0125;
pub const ISPCTRL6: u16 = 0x0126;

pub const RAWNF: u16 = 0x01E4;
pub const ARAWNF: u16 = 0x01E5;
pub const ARAWNFODEL: u16 = 0x01E6;

pub const AEWBCFG: u16 = 0x0380;
pub const AETARGU: u16 = 0x0381;
pub const AETARGL: u16 = 0x0382;
pub const AETARGM: u16 = 0x0383;

pub const SAT: u16 = 0x0480;
pub const ASAT: u16 = 0x0481;
pub const ASATODEL: u16 = 0x0482;
pub const HUESIN: u16 = 0x0486;
pub const HUECOS: u16 = 0x0487;
pub const SCENE: u16 = 0x0488;

pub const CONTM: u16 = 0x04B0;
pub const ACONTM: u16 = 0x04B1;
pub const CONTQ: u16 = 0x04B3;
pub const ACONTQ: u16 = 0x04B4;
pub const CONTN: u16 = 0x04B6;
pub const CONTP: u16 = 0x04B9;
pub const CONTGAIN: u16 = 0x04BC;
pub const YMEAN: u16 = 0x04BD;
pub const BRIGHT: u16 = 0x04C0;

pub const YDN: u16 = 0x0580;
pub const AYDN: u16 = 0x0581;
pub const AYDNODEL: u16 = 0x0582;

pub const EDGE: u16 = 0x05A1;

pub const F2BMEAN: u16 = 0x05D0;
pub const F2BRANGE: u16 = 0x05D1;

pub const YUVSCXL: u16 = 0x05E0;
pub const YUVSCXH: u16 = 0x05E1;
pub const YUVSCYL: u16 = 0x05E2;
pub const YUVSCYH: u16 = 0x05E3;
pub const WINXSTL: u16 = 0x05E4;
pub const WINXSTH: u16 = 0x05E5;
pub const WINXEDL: u16 = 0x05E6;
pub const WINXEDH: u16 = 0x05E7;
pub const WINYSTL: u16 = 0x05E8;
pub const WINYSTH: u16 = 0x05E9;
pub const WINYEDL: u16 = 0x05EA;
pub const WINYEDH: u16 = 0x05EB;

pub const REG_DLY: u16 = 0xFFFF;
pub const REGLIST_TAIL: u16 = 0xFFFF;

pub const SENSOR_DEFAULT_REGS: &[Reg16] = &[
    Reg16::new(0x0022, 0x00),
    Reg16::new(0x0026, 0x77),
    Reg16::new(0x002A, 0x44),
    Reg16::new(0x002B, 0x02),
    Reg16::new(0x002C, 0x00),
    Reg16::new(0x0025, 0x00),
    Reg16::new(0x0020, 0x08),
    Reg16::new(0x0027, 0x40),
    Reg16::new(0x0028, 0xC0),
    Reg16::new(0x0004, 0x10),
    Reg16::new(0x0006, 0x00),
    Reg16::new(0x0012, 0x0F),
    Reg16::new(0x0044, 0x04),
    Reg16::new(0x004A, 0x2A),
    Reg16::new(0x004B, 0x72),
    Reg16::new(0x004E, 0x30),
    Reg16::new(0x0070, 0x2A),
    Reg16::new(0x0071, 0x57),
    Reg16::new(0x0072, 0x55),
    Reg16::new(0x0073, 0x30),
    Reg16::new(0x0077, 0x04),
    Reg16::new(0x0080, 0xC2),
    Reg16::new(0x0082, 0xA2),
    Reg16::new(0x0083, 0xF0),
    Reg16::new(0x0085, 0x11),
    Reg16::new(0x0086, 0x22),
    Reg16::new(0x0087, 0x08),
    Reg16::new(0x0088, 0x6E),
    Reg16::new(0x0089, 0x2A),
    Reg16::new(0x008A, 0x2F),
    Reg16::new(0x008D, 0x20),
    Reg16::new(0x008F, 0x77),
    Reg16::new(0x0090, 0x01),
    Reg16::new(0x0091, 0x02),
    Reg16::new(0x0092, 0x03),
    Reg16::new(0x0093, 0x04),
    Reg16::new(0x0094, 0x14),
    Reg16::new(0x0095, 0x09),
    Reg16::new(0x0096, 0x0A),
    Reg16::new(0x0097, 0x0B),
    Reg16::new(0x0098, 0x0C),
    Reg16::new(0x0099, 0x04),
    Reg16::new(0x009A, 0x14),
    Reg16::new(0x009B, 0x34),
    Reg16::new(0x00A0, 0x00),
    Reg16::new(0x00A1, 0x00),
    Reg16::new(0x0B3B, 0x0B),
    Reg16::new(0x0040, 0x0A),
    Reg16::new(0x0053, 0x0A),
    Reg16::new(0x0120, 0x36),
    Reg16::new(0x0121, 0x80),
    Reg16::new(0x0122, 0xEB),
    Reg16::new(0x0123, 0xCC),
    Reg16::new(0x0124, 0xDE),
    Reg16::new(0x0125, 0xDF),
    Reg16::new(0x0126, 0x70),
    Reg16::new(0x0128, 0x1F),
    Reg16::new(0x0129, 0x8F),
    Reg16::new(0x0132, 0xF8),
    Reg16::new(0x011F, 0x08),
    Reg16::new(0x0144, 0x04),
    Reg16::new(0x0145, 0x00),
    Reg16::new(0x0146, 0x20),
    Reg16::new(0x0147, 0x20),
    Reg16::new(0x0148, 0x14),
    Reg16::new(0x0149, 0x14),
    Reg16::new(0x0156, 0x0C),
    Reg16::new(0x0157, 0x0C),
    Reg16::new(0x0158, 0x0A),
    Reg16::new(0x0159, 0x0A),
    Reg16::new(0x015A, 0x03),
    Reg16::new(0x015B, 0x40),
    Reg16::new(0x015C, 0x21),
    Reg16::new(0x015E, 0x0F),
    Reg16::new(0x0168, 0xC8),
    Reg16::new(0x0169, 0xC8),
    Reg16::new(0x016A, 0x96),
    Reg16::new(0x016B, 0x96),
    Reg16::new(0x016C, 0x64),
    Reg16::new(0x016D, 0x64),
    Reg16::new(0x016E, 0x32),
    Reg16::new(0x016F, 0x32),
    Reg16::new(0x01EF, 0xF1),
    Reg16::new(0x0131, 0x44),
    Reg16::new(0x014C, 0x60),
    Reg16::new(0x014D, 0x24),
    Reg16::new(0x015D, 0x90),
    Reg16::new(0x01D8, 0x40),
    Reg16::new(0x01D9, 0x20),
    Reg16::new(0x01DA, 0x23),
    Reg16::new(0x0150, 0x05),
    Reg16::new(0x0155, 0x07),
    Reg16::new(0x0178, 0x10),
    Reg16::new(0x017A, 0x10),
    Reg16::new(0x01BA, 0x10),
    Reg16::new(0x0176, 0x00),
    Reg16::new(0x0179, 0x10),
    Reg16::new(0x017B, 0x10),
    Reg16::new(0x01BB, 0x10),
    Reg16::new(0x0177, 0x00),
    Reg16::new(0x01E7, 0x20),
    Reg16::new(0x01E8, 0x30),
    Reg16::new(0x01E9, 0x50),
    Reg16::new(0x01E4, 0x18),
    Reg16::new(0x01E5, 0x20),
    Reg16::new(0x01E6, 0x04),
    Reg16::new(0x0210, 0x21),
    Reg16::new(0x0211, 0x0A),
    Reg16::new(0x0212, 0x21),
    Reg16::new(0x01DB, 0x04),
    Reg16::new(0x01DC, 0x14),
    Reg16::new(0x0151, 0x08),
    Reg16::new(0x01F2, 0x18),
    Reg16::new(0x01F8, 0x3C),
    Reg16::new(0x01FE, 0x24),
    Reg16::new(0x0213, 0x03),
    Reg16::new(0x0214, 0x03),
    Reg16::new(0x0215, 0x10),
    Reg16::new(0x0216, 0x08),
    Reg16::new(0x0217, 0x05),
    Reg16::new(0x0218, 0xB8),
    Reg16::new(0x0219, 0x01),
    Reg16::new(0x021A, 0xB8),
    Reg16::new(0x021B, 0x01),
    Reg16::new(0x021C, 0xB8),
    Reg16::new(0x021D, 0x01),
    Reg16::new(0x021E, 0xB8),
    Reg16::new(0x021F, 0x01),
    Reg16::new(0x0220, 0xF1),
    Reg16::new(0x0221, 0x5D),
    Reg16::new(0x0222, 0x0A),
    Reg16::new(0x0223, 0x80),
    Reg16::new(0x0224, 0x50),
    Reg16::new(0x0225, 0x09),
    Reg16::new(0x0226, 0x80),
    Reg16::new(0x022A, 0x56),
    Reg16::new(0x022B, 0x13),
    Reg16::new(0x022C, 0x80),
    Reg16::new(0x022D, 0x11),
    Reg16::new(0x022E, 0x08),
    Reg16::new(0x022F, 0x11),
    Reg16::new(0x0230, 0x08),
    Reg16::new(0x0233, 0x11),
    Reg16::new(0x0234, 0x08),
    Reg16::new(0x0235, 0x88),
    Reg16::new(0x0236, 0x02),
    Reg16::new(0x0237, 0x88),
    Reg16::new(0x0238, 0x02),
    Reg16::new(0x023B, 0x88),
    Reg16::new(0x023C, 0x02),
    Reg16::new(0x023D, 0x68),
    Reg16::new(0x023E, 0x01),
    Reg16::new(0x023F, 0x68),
    Reg16::new(0x0240, 0x01),
    Reg16::new(0x0243, 0x68),
    Reg16::new(0x0244, 0x01),
    Reg16::new(0x0251, 0x0F),
    Reg16::new(0x0252, 0x00),
    Reg16::new(0x0260, 0x00),
    Reg16::new(0x0261, 0x4A),
    Reg16::new(0x0262, 0x2C),
    Reg16::new(0x0263, 0x68),
    Reg16::new(0x0264, 0x40),
    Reg16::new(0x0265, 0x2C),
    Reg16::new(0x0266, 0x6A),
    Reg16::new(0x026A, 0x40),
    Reg16::new(0x026B, 0x30),
    Reg16::new(0x026C, 0x66),
    Reg16::new(0x0278, 0x98),
    Reg16::new(0x0279, 0x20),
    Reg16::new(0x027A, 0x80),
    Reg16::new(0x027B, 0x73),
    Reg16::new(0x027C, 0x08),
    Reg16::new(0x027D, 0x80),
    Reg16::new(0x0280, 0x0D),
    Reg16::new(0x0282, 0x1A),
    Reg16::new(0x0284, 0x30),
    Reg16::new(0x0286, 0x53),
    Reg16::new(0x0288, 0x62),
    Reg16::new(0x028A, 0x6E),
    Reg16::new(0x028C, 0x7A),
    Reg16::new(0x028E, 0x83),
    Reg16::new(0x0290, 0x8B),
    Reg16::new(0x0292, 0x92),
    Reg16::new(0x0294, 0x9D),
    Reg16::new(0x0296, 0xA8),
    Reg16::new(0x0298, 0xBC),
    Reg16::new(0x029A, 0xCF),
    Reg16::new(0x029C, 0xE2),
    Reg16::new(0x029E, 0x2A),
    Reg16::new(0x02A0, 0x02),
    Reg16::new(0x02C0, 0x7D),
    Reg16::new(0x02C1, 0x01),
    Reg16::new(0x02C2, 0x7C),
    Reg16::new(0x02C3, 0x04),
    Reg16::new(0x02C4, 0x01),
    Reg16::new(0x02C5, 0x04),
    Reg16::new(0x02C6, 0x3E),
    Reg16::new(0x02C7, 0x04),
    Reg16::new(0x02C8, 0x90),
    Reg16::new(0x02C9, 0x01),
    Reg16::new(0x02CA, 0x52),
    Reg16::new(0x02CB, 0x04),
    Reg16::new(0x02CC, 0x04),
    Reg16::new(0x02CD, 0x04),
    Reg16::new(0x02CE, 0xA9),
    Reg16::new(0x02CF, 0x04),
    Reg16::new(0x02D0, 0xAD),
    Reg16::new(0x02D1, 0x01),
    Reg16::new(0x0302, 0x00),
    Reg16::new(0x0303, 0x00),
    Reg16::new(0x0304, 0x00),
    Reg16::new(0x02E0, 0x04),
    Reg16::new(0x02F0, 0x4E),
    Reg16::new(0x02F1, 0x04),
    Reg16::new(0x02F2, 0xB1),
    Reg16::new(0x02F3, 0x00),
    Reg16::new(0x02F4, 0x63),
    Reg16::new(0x02F5, 0x04),
    Reg16::new(0x02F6, 0x28),
    Reg16::new(0x02F7, 0x04),
    Reg16::new(0x02F8, 0x29),
    Reg16::new(0x02F9, 0x04),
    Reg16::new(0x02FA, 0x51),
    Reg16::new(0x02FB, 0x00),
    Reg16::new(0x02FC, 0x64),
    Reg16::new(0x02FD, 0x04),
    Reg16::new(0x02FE, 0x6B),
    Reg16::new(0x02FF, 0x04),
    Reg16::new(0x0300, 0xCF),
    Reg16::new(0x0301, 0x00),
    Reg16::new(0x0305, 0x08),
    Reg16::new(0x0306, 0x40),
    Reg16::new(0x0307, 0x00),
    Reg16::new(0x032D, 0x70),
    Reg16::new(0x032E, 0x01),
    Reg16::new(0x032F, 0x00),
    Reg16::new(0x0330, 0x01),
    Reg16::new(0x0331, 0x70),
    Reg16::new(0x0332, 0x01),
    Reg16::new(0x0333, 0x82),
    Reg16::new(0x0334, 0x82),
    Reg16::new(0x0335, 0x86),
    Reg16::new(0x0340, 0x30),
    Reg16::new(0x0341, 0x44),
    Reg16::new(0x0342, 0x4A),
    Reg16::new(0x0343, 0x3C),
    Reg16::new(0x0344, 0x83),
    Reg16::new(0x0345, 0x4D),
    Reg16::new(0x0346, 0x75),
    Reg16::new(0x0347, 0x56),
    Reg16::new(0x0348, 0x68),
    Reg16::new(0x0349, 0x5E),
    Reg16::new(0x034A, 0x5C),
    Reg16::new(0x034B, 0x65),
    Reg16::new(0x034C, 0x52),
    Reg16::new(0x0350, 0x88),
    Reg16::new(0x0352, 0x18),
    Reg16::new(0x0354, 0x80),
    Reg16::new(0x0355, 0x50),
    Reg16::new(0x0356, 0x88),
    Reg16::new(0x0357, 0xE0),
    Reg16::new(0x0358, 0x00),
    Reg16::new(0x035A, 0x00),
    Reg16::new(0x035B, 0xAC),
    Reg16::new(0x0360, 0x02),
    Reg16::new(0x0361, 0x18),
    Reg16::new(0x0362, 0x50),
    Reg16::new(0x0363, 0x6C),
    Reg16::new(0x0364, 0x00),
    Reg16::new(0x0365, 0xF0),
    Reg16::new(0x0366, 0x08),
    Reg16::new(0x036A, 0x10),
    Reg16::new(0x036B, 0x18),
    Reg16::new(0x036E, 0x10),
    Reg16::new(0x0370, 0x10),
    Reg16::new(0x0371, 0x18),
    Reg16::new(0x0372, 0x0C),
    Reg16::new(0x0373, 0x38),
    Reg16::new(0x0374, 0x3A),
    Reg16::new(0x0375, 0x12),
    Reg16::new(0x0376, 0x20),
    Reg16::new(0x0380, 0xFF),
    Reg16::new(0x0381, 0x44),
    Reg16::new(0x0382, 0x34),
    Reg16::new(0x038A, 0x80),
    Reg16::new(0x038B, 0x0A),
    Reg16::new(0x038C, 0xC1),
    Reg16::new(0x038E, 0x3C),
    Reg16::new(0x038F, 0x09),
    Reg16::new(0x0390, 0xE0),
    Reg16::new(0x0391, 0x01),
    Reg16::new(0x0392, 0x03),
    Reg16::new(0x0393, 0x80),
    Reg16::new(0x0395, 0x22),
    Reg16::new(0x0398, 0x02),
    Reg16::new(0x0399, 0xF0),
    Reg16::new(0x039A, 0x03),
    Reg16::new(0x039B, 0xAC),
    Reg16::new(0x039C, 0x04),
    Reg16::new(0x039D, 0x68),
    Reg16::new(0x039E, 0x05),
    Reg16::new(0x039F, 0xE0),
    Reg16::new(0x03A0, 0x07),
    Reg16::new(0x03A1, 0x58),
    Reg16::new(0x03A2, 0x08),
    Reg16::new(0x03A3, 0xD0),
    Reg16::new(0x03A4, 0x0B),
    Reg16::new(0x03A5, 0xC0),
    Reg16::new(0x03A6, 0x18),
    Reg16::new(0x03A7, 0x1C),
    Reg16::new(0x03A8, 0x20),
    Reg16::new(0x03A9, 0x24),
    Reg16::new(0x03AA, 0x28),
    Reg16::new(0x03AB, 0x30),
    Reg16::new(0x03AC, 0x24),
    Reg16::new(0x03AD, 0x21),
    Reg16::new(0x03AE, 0x1C),
    Reg16::new(0x03AF, 0x18),
    Reg16::new(0x03B0, 0x17),
    Reg16::new(0x03B1, 0x13),
    Reg16::new(0x03B7, 0x64),
    Reg16::new(0x03B8, 0x00),
    Reg16::new(0x03B9, 0xB4),
    Reg16::new(0x03BA, 0x00),
    Reg16::new(0x03BB, 0xFF),
    Reg16::new(0x03BC, 0xFF),
    Reg16::new(0x03BD, 0xFF),
    Reg16::new(0x03BE, 0xFF),
    Reg16::new(0x03BF, 0xFF),
    Reg16::new(0x03C0, 0xFF),
    Reg16::new(0x03C1, 0x01),
    Reg16::new(0x03E0, 0x04),
    Reg16::new(0x03E1, 0x11),
    Reg16::new(0x03E2, 0x01),
    Reg16::new(0x03E3, 0x04),
    Reg16::new(0x03E4, 0x10),
    Reg16::new(0x03E5, 0x21),
    Reg16::new(0x03E6, 0x11),
    Reg16::new(0x03E7, 0x00),
    Reg16::new(0x03E8, 0x11),
    Reg16::new(0x03E9, 0x32),
    Reg16::new(0x03EA, 0x12),
    Reg16::new(0x03EB, 0x01),
    Reg16::new(0x03EC, 0x21),
    Reg16::new(0x03ED, 0x33),
    Reg16::new(0x03EE, 0x23),
    Reg16::new(0x03EF, 0x01),
    Reg16::new(0x03F0, 0x11),
    Reg16::new(0x03F1, 0x32),
    Reg16::new(0x03F2, 0x12),
    Reg16::new(0x03F3, 0x01),
    Reg16::new(0x03F4, 0x10),
    Reg16::new(0x03F5, 0x21),
    Reg16::new(0x03F6, 0x11),
    Reg16::new(0x03F7, 0x00),
    Reg16::new(0x03F8, 0x04),
    Reg16::new(0x03F9, 0x11),
    Reg16::new(0x03FA, 0x01),
    Reg16::new(0x03FB, 0x04),
    Reg16::new(0x03DC, 0x47),
    Reg16::new(0x03DD, 0x5A),
    Reg16::new(0x03DE, 0x41),
    Reg16::new(0x03DF, 0x53),
    Reg16::new(0x0420, 0x82),
    Reg16::new(0x0421, 0x00),
    Reg16::new(0x0422, 0x00),
    Reg16::new(0x0423, 0x88),
    Reg16::new(0x0430, 0x08),
    Reg16::new(0x0431, 0x30),
    Reg16::new(0x0432, 0x0C),
    Reg16::new(0x0433, 0x04),
    Reg16::new(0x0435, 0x08),
    Reg16::new(0x0450, 0xFF),
    Reg16::new(0x0451, 0xD0),
    Reg16::new(0x0452, 0xB8),
    Reg16::new(0x0453, 0x88),
    Reg16::new(0x0454, 0x00),
    Reg16::new(0x0458, 0x80),
    Reg16::new(0x0459, 0x03),
    Reg16::new(0x045A, 0x00),
    Reg16::new(0x045B, 0x50),
    Reg16::new(0x045C, 0x00),
    Reg16::new(0x045D, 0x90),
    Reg16::new(0x0465, 0x02),
    Reg16::new(0x0466, 0x14),
    Reg16::new(0x047A, 0x00),
    Reg16::new(0x047B, 0x00),
    Reg16::new(0x047C, 0x04),
    Reg16::new(0x047D, 0x50),
    Reg16::new(0x047E, 0x04),
    Reg16::new(0x047F, 0x90),
    Reg16::new(0x0480, 0x58),
    Reg16::new(0x0481, 0x06),
    Reg16::new(0x0482, 0x08),
    Reg16::new(0x04B0, 0x50),
    Reg16::new(0x04B6, 0x30),
    Reg16::new(0x04B9, 0x10),
    Reg16::new(0x04B3, 0x00),
    Reg16::new(0x04B1, 0x85),
    Reg16::new(0x04B4, 0x00),
    Reg16::new(0x0540, 0x00),
    Reg16::new(0x0541, 0xBC),
    Reg16::new(0x0542, 0x00),
    Reg16::new(0x0543, 0xE1),
    Reg16::new(0x0580, 0x04),
    Reg16::new(0x0581, 0x0F),
    Reg16::new(0x0582, 0x04),
    Reg16::new(0x05A1, 0x0A),
    Reg16::new(0x05A2, 0x21),
    Reg16::new(0x05A3, 0x84),
    Reg16::new(0x05A4, 0x24),
    Reg16::new(0x05A5, 0xFF),
    Reg16::new(0x05A6, 0x00),
    Reg16::new(0x05A7, 0x24),
    Reg16::new(0x05A8, 0x24),
    Reg16::new(0x05A9, 0x02),
    Reg16::new(0x05B1, 0x24),
    Reg16::new(0x05B2, 0x0C),
    Reg16::new(0x05B4, 0x1F),
    Reg16::new(0x05AE, 0x75),
    Reg16::new(0x05AF, 0x78),
    Reg16::new(0x05B6, 0x00),
    Reg16::new(0x05B7, 0x10),
    Reg16::new(0x05BF, 0x20),
    Reg16::new(0x05C1, 0x06),
    Reg16::new(0x05C2, 0x18),
    Reg16::new(0x05C7, 0x00),
    Reg16::new(0x05CC, 0x04),
    Reg16::new(0x05CD, 0x00),
    Reg16::new(0x05CE, 0x03),
    Reg16::new(0x05E4, 0x08),
    Reg16::new(0x05E5, 0x00),
    Reg16::new(0x05E6, 0x07),
    Reg16::new(0x05E7, 0x05),
    Reg16::new(0x05E8, 0x08),
    Reg16::new(0x05E9, 0x00),
    Reg16::new(0x05EA, 0xD7),
    Reg16::new(0x05EB, 0x02),
    Reg16::new(0x0660, 0x00),
    Reg16::new(0x0661, 0x16),
    Reg16::new(0x0662, 0x07),
    Reg16::new(0x0663, 0xF1),
    Reg16::new(0x0664, 0x07),
    Reg16::new(0x0665, 0xDE),
    Reg16::new(0x0666, 0x07),
    Reg16::new(0x0667, 0xE7),
    Reg16::new(0x0668, 0x00),
    Reg16::new(0x0669, 0x35),
    Reg16::new(0x066A, 0x07),
    Reg16::new(0x066B, 0xF9),
    Reg16::new(0x066C, 0x07),
    Reg16::new(0x066D, 0xB7),
    Reg16::new(0x066E, 0x00),
    Reg16::new(0x066F, 0x27),
    Reg16::new(0x0670, 0x07),
    Reg16::new(0x0671, 0xF3),
    Reg16::new(0x0672, 0x07),
    Reg16::new(0x0673, 0xC5),
    Reg16::new(0x0674, 0x07),
    Reg16::new(0x0675, 0xEE),
    Reg16::new(0x0676, 0x00),
    Reg16::new(0x0677, 0x16),
    Reg16::new(0x0678, 0x01),
    Reg16::new(0x0679, 0x80),
    Reg16::new(0x067A, 0x00),
    Reg16::new(0x067B, 0x85),
    Reg16::new(0x067C, 0x07),
    Reg16::new(0x067D, 0xE1),
    Reg16::new(0x067E, 0x07),
    Reg16::new(0x067F, 0xF5),
    Reg16::new(0x0680, 0x07),
    Reg16::new(0x0681, 0xB9),
    Reg16::new(0x0682, 0x00),
    Reg16::new(0x0683, 0x31),
    Reg16::new(0x0684, 0x07),
    Reg16::new(0x0685, 0xE6),
    Reg16::new(0x0686, 0x07),
    Reg16::new(0x0687, 0xD3),
    Reg16::new(0x0688, 0x00),
    Reg16::new(0x0689, 0x18),
    Reg16::new(0x068A, 0x07),
    Reg16::new(0x068B, 0xFA),
    Reg16::new(0x068C, 0x07),
    Reg16::new(0x068D, 0xD2),
    Reg16::new(0x068E, 0x00),
    Reg16::new(0x068F, 0x08),
    Reg16::new(0x0690, 0x00),
    Reg16::new(0x0691, 0x02),
    Reg16::new(0xAFD0, 0x03),
    Reg16::new(0xAFD3, 0x18),
    Reg16::new(0xAFD4, 0x04),
    Reg16::new(0xAFD5, 0xB8),
    Reg16::new(0xAFD6, 0x02),
    Reg16::new(0xAFD7, 0x44),
    Reg16::new(0xAFD8, 0x02),
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
    Reg16::new(0x05E4, 0x3A),
    Reg16::new(0x05E5, 0x00),
    Reg16::new(0x05E6, 0x79),
    Reg16::new(0x05E7, 0x01),
    Reg16::new(0x05E8, 0x04),
    Reg16::new(0x05E9, 0x00),
    Reg16::new(0x05EA, 0xF3),
    Reg16::new(0x05EB, 0x00),
    Reg16::new(0x0000, 0x01),
    Reg16::new(0x0100, 0x01),
    Reg16::new(0x0101, 0x01),
];

pub const SENSOR_FRAMESIZE_HD: &[Reg16] = &[
    Reg16::new(0x0006, 0x00),
    Reg16::new(0x000D, 0x00),
    Reg16::new(0x000E, 0x00),
    Reg16::new(0x0122, 0xFB),
    Reg16::new(0x0125, 0xDF),
    Reg16::new(0x0126, 0x70),
    Reg16::new(0x05E4, 0x08),
    Reg16::new(0x05E5, 0x00),
    Reg16::new(0x05E6, 0x07),
    Reg16::new(0x05E7, 0x05),
    Reg16::new(0x05E8, 0x08),
    Reg16::new(0x05E9, 0x00),
    Reg16::new(0x05EA, 0xD7),
    Reg16::new(0x05EB, 0x02),
];

pub const SENSOR_FRAMESIZE_SVGA: &[Reg16] = &[
    Reg16::new(0x0006, 0x10),
    Reg16::new(0x000D, 0x00),
    Reg16::new(0x000E, 0x00),
    Reg16::new(0x0122, 0xEB),
    Reg16::new(0x0125, 0xDF),
    Reg16::new(0x0126, 0x70),
    Reg16::new(0x05E4, 0x58),
    Reg16::new(0x05E5, 0x00),
    Reg16::new(0x05E6, 0x77),
    Reg16::new(0x05E7, 0x03),
    Reg16::new(0x05E8, 0x42),
    Reg16::new(0x05E9, 0x00),
    Reg16::new(0x05EA, 0x99),
    Reg16::new(0x05EB, 0x02),
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

pub const SENSOR_FRAMESIZE_CIF: &[Reg16] = &[
    Reg16::new(0x0006, 0x00),
    Reg16::new(0x000D, 0x01),
    Reg16::new(0x000E, 0x11),
    Reg16::new(0x0122, 0xFB),
    Reg16::new(0x0125, 0xFF),
    Reg16::new(0x0126, 0x72),
    Reg16::new(0x05E0, 0xC0),
    Reg16::new(0x05E1, 0x00),
    Reg16::new(0x05E2, 0x8F),
    Reg16::new(0x05E3, 0x00),
    Reg16::new(0x05E4, 0x11),
    Reg16::new(0x05E5, 0x00),
    Reg16::new(0x05E6, 0xA0),
    Reg16::new(0x05E7, 0x01),
    Reg16::new(0x05E8, 0x08),
    Reg16::new(0x05E9, 0x00),
    Reg16::new(0x05EA, 0x2F),
    Reg16::new(0x05EB, 0x01),
];

pub const SENSOR_FRAMESIZE_QVGA: &[Reg16] = &[
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
    Reg16::new(0x05E4, 0x3A),
    Reg16::new(0x05E5, 0x00),
    Reg16::new(0x05E6, 0x79),
    Reg16::new(0x05E7, 0x01),
    Reg16::new(0x05E8, 0x04),
    Reg16::new(0x05E9, 0x00),
    Reg16::new(0x05EA, 0xF3),
    Reg16::new(0x05EB, 0x00),
];

pub const SENSOR_FRAMESIZE_QCIF: &[Reg16] = &[
    Reg16::new(0x0006, 0x00),
    Reg16::new(0x000D, 0x01),
    Reg16::new(0x000E, 0x11),
    Reg16::new(0x0122, 0xFB),
    Reg16::new(0x0125, 0xFF),
    Reg16::new(0x0126, 0x72),
    Reg16::new(0x05E0, 0x52),
    Reg16::new(0x05E1, 0x01),
    Reg16::new(0x05E2, 0x38),
    Reg16::new(0x05E3, 0x01),
    Reg16::new(0x05E4, 0x22),
    Reg16::new(0x05E5, 0x00),
    Reg16::new(0x05E6, 0xD1),
    Reg16::new(0x05E7, 0x00),
    Reg16::new(0x05E8, 0x04),
    Reg16::new(0x05E9, 0x00),
    Reg16::new(0x05EA, 0x93),
    Reg16::new(0x05EB, 0x00),
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
    Reg16::new(0x0006, 0x00),
    Reg16::new(0x000D, 0x01),
    Reg16::new(0x000E, 0x11),
    Reg16::new(0x0122, 0xFB),
    Reg16::new(0x0125, 0xFF),
    Reg16::new(0x0126, 0x70),
    Reg16::new(0x05E0, 0x78),
    Reg16::new(0x05E1, 0x01),
    Reg16::new(0x05E2, 0x78),
    Reg16::new(0x05E3, 0x01),
    Reg16::new(0x05E4, 0x1E),
    Reg16::new(0x05E5, 0x00),
    Reg16::new(0x05E6, 0xBD),
    Reg16::new(0x05E7, 0x00),
    Reg16::new(0x05E8, 0x03),
    Reg16::new(0x05E9, 0x00),
    Reg16::new(0x05EA, 0x7A),
    Reg16::new(0x05EB, 0x00),
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
fn write_reg_bits<B: SccbBus>(
    bus: &mut B,
    slv_addr: u8,
    reg: u16,
    mask: u8,
    enable: bool,
) -> Result<(), CameraError> {
    set_reg_bits(bus, slv_addr, reg, 0, mask, if enable { mask } else { 0 })
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
        bus.write_reg16(slv_addr, entry.reg, entry.val)?;
    }
    Ok(())
}

/// Detects an HM1055 sensor at `slv_addr` (`0x24`).
pub fn detect<B: SccbBus>(bus: &mut B, slv_addr: u8) -> Result<SensorId, CameraError> {
    Hm1055::detect(bus, slv_addr)
}

/// Safe `#![no_std]` driver for the Himax HM1055 720p HD camera sensor.
#[derive(Debug, Clone)]
pub struct Hm1055 {
    id: SensorId,
    status: CameraStatus,
    slv_addr: u8,
    pixformat: PixelFormat,
    xclk_freq_hz: u32,
}

pub type HM1055 = Hm1055;

impl Default for Hm1055 {
    fn default() -> Self {
        Self::new()
    }
}

impl Hm1055 {
    /// Creates a new [`Hm1055`] instance with default SCCB address (`0x24`) and PID (`0x0955`).
    pub const fn new() -> Self {
        Self {
            id: SensorId {
                midh: 0,
                midl: 0,
                pid: CameraPid::HM1055.as_u16(),
                ver: 0,
            },
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
            slv_addr: CameraSccbAddr::HM1055,
            pixformat: PixelFormat::Rgb565,
            xclk_freq_hz: 24_000_000,
        }
    }

    /// Creates a new [`Hm1055`] with an explicit slave address and [`SensorId`].
    pub const fn with_addr(slv_addr: u8, id: SensorId) -> Self {
        Self {
            id,
            slv_addr,
            ..Self::new()
        }
    }

    /// Probes the SCCB bus at `slv_addr` (`0x24`) and verifies `HM1055_PID` (`0x0955`).
    pub fn detect<B: SccbBus>(bus: &mut B, slv_addr: u8) -> Result<SensorId, CameraError> {
        if slv_addr != CameraSccbAddr::HM1055 {
            return Err(CameraError::NotDetected);
        }
        let h = bus.read_reg16(slv_addr, IDH)? as u16;
        let l = bus.read_reg16(slv_addr, IDL)? as u16;
        let pid = (h << 8) | l;
        if pid == CameraPid::HM1055.as_u16() {
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

    /// Returns the 7-bit SCCB slave address (`0x24`).
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

    /// Returns the [`CameraModel`] (`CameraModel::Hm1055`).
    pub const fn model(&self) -> CameraModel {
        CameraModel::Hm1055
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

    fn get_sharpness<B: SccbBus>(&self, bus: &mut B) -> Result<i8, CameraError> {
        let ret = bus.read_reg16(self.slv_addr, EDGE)? as i32;
        Ok(((ret - 0x60) / 0x20) as i8)
    }

    fn get_denoise<B: SccbBus>(&self, bus: &mut B) -> Result<u8, CameraError> {
        let ret = bus.read_reg16(self.slv_addr, YDN)? as i32;
        Ok(((ret - 0x07) / 2) as u8)
    }
}

impl<B: SccbBus> CameraSensor<B> for Hm1055 {
    fn model(&self) -> CameraModel {
        CameraModel::Hm1055
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
        bus.write_reg16(self.slv_addr, SFTRST, 0x55)?;
        bus.delay_ms(100);
        write_reg_list(bus, self.slv_addr, SENSOR_DEFAULT_REGS)?;
        bus.delay_ms(100);
        self.set_ae_level(bus, 0)
    }

    fn init_status(&mut self, bus: &mut B) -> Result<(), CameraError> {
        self.status.brightness = 0;
        self.status.contrast = 0;
        self.status.saturation = 0;
        self.status.sharpness = self.get_sharpness(bus)?;
        self.status.denoise = self.get_denoise(bus)?;
        self.status.ae_level = 0;
        self.status.awb = check_reg_mask(bus, self.slv_addr, AEWBCFG, 0x02)?;
        self.status.agc = 1;
        self.status.aec = check_reg_mask(bus, self.slv_addr, AEWBCFG, 0x01)?;
        self.status.hmirror = check_reg_mask(bus, self.slv_addr, RDCFG, 0x02)?;
        self.status.vflip = check_reg_mask(bus, self.slv_addr, RDCFG, 0x01)?;
        self.status.lenc = check_reg_mask(bus, self.slv_addr, ISPCTRL3, 0x40)?;
        self.status.awb_gain = bus.read_reg16(self.slv_addr, DGAIN)?;
        self.status.agc_gain = bus.read_reg16(self.slv_addr, AGAIN)?;
        self.status.aec_value = bus.read_reg16(self.slv_addr, AETARGM)? as u16;
        Ok(())
    }

    fn set_pixformat(&mut self, bus: &mut B, pixformat: PixelFormat) -> Result<(), CameraError> {
        let portctrl = match pixformat {
            PixelFormat::Raw => 0x20,
            PixelFormat::Yuv422 => 0x30,
            PixelFormat::Rgb565 | PixelFormat::Rgb888 => 0x40,
            PixelFormat::Rgb555 => 0x50,
            PixelFormat::Rgb444 => 0x60,
            _ => return Err(CameraError::NotSupported),
        };
        bus.write_reg16(self.slv_addr, PORTCTRL, portctrl)?;
        bus.write_reg16(self.slv_addr, CMU, 0x01)?;
        self.pixformat = pixformat;
        Ok(())
    }

    fn set_framesize(&mut self, bus: &mut B, framesize: FrameSize) -> Result<(), CameraError> {
        if framesize > FrameSize::Hd {
            return Err(CameraError::FailedToSetFrameSize);
        }
        self.status.framesize = framesize;
        write_reg_list(bus, self.slv_addr, SENSOR_DEFAULT_REGS)?;
        match framesize {
            FrameSize::Qqvga => write_reg_list(bus, self.slv_addr, SENSOR_FRAMESIZE_QQVGA)?,
            FrameSize::Qcif => write_reg_list(bus, self.slv_addr, SENSOR_FRAMESIZE_QCIF)?,
            FrameSize::Size240x240 => write_reg_list(bus, self.slv_addr, SENSOR_FRAMESIZE_240X240)?,
            FrameSize::Qvga => write_reg_list(bus, self.slv_addr, SENSOR_FRAMESIZE_QVGA)?,
            FrameSize::Cif => write_reg_list(bus, self.slv_addr, SENSOR_FRAMESIZE_CIF)?,
            FrameSize::Vga => write_reg_list(bus, self.slv_addr, SENSOR_FRAMESIZE_VGA)?,
            FrameSize::Svga => write_reg_list(bus, self.slv_addr, SENSOR_FRAMESIZE_SVGA)?,
            FrameSize::Hd => {
                write_reg_list(bus, self.slv_addr, SENSOR_FRAMESIZE_HD)?;
                self.set_pll(bus, 0, 288, 1, 0, 0, 0, 1, 16)?;
            }
            _ => write_reg_list(bus, self.slv_addr, SENSOR_FRAMESIZE_VGA)?,
        }
        let pixformat = self.pixformat;
        self.set_pixformat(bus, pixformat)?;
        bus.write_reg16(self.slv_addr, CMU, 0x01)?;
        bus.write_reg16(self.slv_addr, TGRDCFG, 0x01)
    }

    fn set_contrast(&mut self, bus: &mut B, level: i32) -> Result<(), CameraError> {
        let ispctrl5 = bus.read_reg16(self.slv_addr, ISPCTRL5)? | 0x80;
        bus.write_reg16(self.slv_addr, ISPCTRL5, ispctrl5)?;
        bus.write_reg16(self.slv_addr, ACONTQ, ((level.wrapping_mul(0x20)) & 0xFF) as u8)?;
        bus.write_reg16(self.slv_addr, CMU, 0x01)?;
        self.status.contrast = level as i8;
        Ok(())
    }

    fn set_brightness(&mut self, bus: &mut B, level: i32) -> Result<(), CameraError> {
        let ispctrl5 = bus.read_reg16(self.slv_addr, ISPCTRL5)? | 0x40;
        let brightness = match level {
            3 => 0xFF,
            2 => 0xBA,
            1 => 0x96,
            0 => 0x72,
            -1 => 0x48,
            -2 => 0x24,
            -3 => 0x00,
            _ => 0x00,
        };
        bus.write_reg16(self.slv_addr, ISPCTRL5, ispctrl5)?;
        bus.write_reg16(self.slv_addr, BRIGHT, brightness)?;
        bus.write_reg16(self.slv_addr, CMU, 0x01)?;
        self.status.brightness = level as i8;
        Ok(())
    }

    fn set_saturation(&mut self, bus: &mut B, level: i32) -> Result<(), CameraError> {
        let val = ((level.wrapping_mul(0x20).wrapping_add(0x4A)) & 0xFF) as u8;
        bus.write_reg16(self.slv_addr, SAT, val)?;
        bus.write_reg16(self.slv_addr, CMU, 0x01)?;
        self.status.saturation = level as i8;
        Ok(())
    }

    fn set_sharpness(&mut self, bus: &mut B, level: i32) -> Result<(), CameraError> {
        let val = ((level.wrapping_mul(0x20).wrapping_add(0x60)) & 0xFF) as u8;
        bus.write_reg16(self.slv_addr, EDGE, val)?;
        bus.write_reg16(self.slv_addr, CMU, 0x01)?;
        self.status.sharpness = level as i8;
        Ok(())
    }

    fn set_denoise(&mut self, bus: &mut B, level: i32) -> Result<(), CameraError> {
        let ispctrl5 = bus.read_reg16(self.slv_addr, ISPCTRL5)? | 0x20;
        bus.write_reg16(self.slv_addr, ISPCTRL5, ispctrl5)?;
        let val = ((level.wrapping_mul(2).wrapping_add(0x07)) & 0xFF) as u8;
        bus.write_reg16(self.slv_addr, YDN, val)?;
        bus.write_reg16(self.slv_addr, CMU, 0x01)?;
        self.status.denoise = (level & 0xFF) as u8;
        Ok(())
    }

    fn set_quality(&mut self, _bus: &mut B, quality: i32) -> Result<(), CameraError> {
        self.status.quality = (quality & 0xFF) as u8;
        Ok(())
    }

    fn set_colorbar(&mut self, _bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.status.colorbar = u8::from(enable);
        Ok(())
    }

    fn set_whitebal(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        write_reg_bits(bus, self.slv_addr, AEWBCFG, 0x02, enable)?;
        bus.write_reg16(self.slv_addr, CMU, 0x01)?;
        self.status.awb = u8::from(enable);
        Ok(())
    }

    fn set_gain_ctrl(&mut self, _bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.status.agc = u8::from(enable);
        Ok(())
    }

    fn set_exposure_ctrl(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        write_reg_bits(bus, self.slv_addr, AEWBCFG, 0x01, enable)?;
        bus.write_reg16(self.slv_addr, CMU, 0x01)?;
        self.status.aec = u8::from(enable);
        Ok(())
    }

    fn set_hmirror(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        write_reg_bits(bus, self.slv_addr, RDCFG, 0x02, enable)?;
        bus.write_reg16(self.slv_addr, CMU, 0x01)?;
        self.status.hmirror = u8::from(enable);
        Ok(())
    }

    fn set_vflip(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        write_reg_bits(bus, self.slv_addr, RDCFG, 0x01, enable)?;
        bus.write_reg16(self.slv_addr, CMU, 0x01)?;
        self.status.vflip = u8::from(enable);
        Ok(())
    }

    fn set_agc_gain(&mut self, bus: &mut B, gain: i32) -> Result<(), CameraError> {
        if !(0..=7).contains(&gain) {
            return Err(CameraError::InvalidArg);
        }
        bus.write_reg16(self.slv_addr, AGAIN, gain as u8)?;
        bus.write_reg16(self.slv_addr, CMU, 0x01)?;
        self.status.agc_gain = gain as u8;
        Ok(())
    }

    fn set_aec_value(&mut self, bus: &mut B, gain: i32) -> Result<(), CameraError> {
        bus.write_reg16(self.slv_addr, AETARGM, (gain & 0xFF) as u8)?;
        bus.write_reg16(self.slv_addr, CMU, 0x01)?;
        self.status.aec_value = (gain & 0xFFFF) as u16;
        Ok(())
    }

    fn set_ae_level(&mut self, bus: &mut B, level: i32) -> Result<(), CameraError> {
        if !(-5..=5).contains(&level) {
            return Err(CameraError::InvalidArg);
        }
        let target_level = (((level + 5) * 10) + 5) as u16;
        let upper = ((target_level * 27) / 25) as u8;
        let lower = ((target_level * 23) / 25) as u8;
        bus.write_reg16(self.slv_addr, AETARGU, upper)?;
        bus.write_reg16(self.slv_addr, AETARGL, lower)?;
        bus.write_reg16(self.slv_addr, CMU, 0x01)?;
        self.status.ae_level = level as i8;
        Ok(())
    }

    fn set_raw_gma(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        write_reg_bits(bus, self.slv_addr, ISPCTRL1, 0x04, enable)?;
        bus.write_reg16(self.slv_addr, CMU, 0x01)?;
        self.status.raw_gma = u8::from(enable);
        Ok(())
    }

    fn set_lenc(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        write_reg_bits(bus, self.slv_addr, ISPCTRL3, 0x40, enable)?;
        bus.write_reg16(self.slv_addr, CMU, 0x01)?;
        self.status.lenc = u8::from(enable);
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
        Ok(())
    }

    fn set_pll(
        &mut self,
        bus: &mut B,
        bypass: i32,
        mul: i32,
        _sys: i32,
        root: i32,
        _pre: i32,
        _seld5: i32,
        pclken: i32,
        pclk: i32,
    ) -> Result<(), CameraError> {
        let mut ckcfg1 = 0u8;
        let mut ckcfg2 = if self.xclk_freq_hz <= 6_000_000 {
            0x00
        } else if self.xclk_freq_hz <= 12_000_000 {
            0x20
        } else if self.xclk_freq_hz <= 18_000_000 {
            0x40
        } else if self.xclk_freq_hz <= 24_000_000 {
            0x60
        } else if self.xclk_freq_hz <= 30_000_000 {
            0x80
        } else if self.xclk_freq_hz <= 36_000_000 {
            0xA0
        } else if self.xclk_freq_hz <= 42_000_000 {
            0xC0
        } else {
            0xE0
        };

        if bypass == 0 {
            ckcfg2 |= match mul {
                204 => 10,
                216 => 11,
                228 => 0x12,
                240 => 0x13,
                288 => 0x17,
                300 => 0x18,
                312 => 0x19,
                324 => 0x1A,
                336 => 0x1B,
                348 => 0x1C,
                360 => 0x1D,
                _ => 0x17,
            };
        }

        if pclken > 0 {
            let pclk_div = pclk.clamp(1, 128) as u8;
            ckcfg1 |= pclk_div - 1;
        }

        let ckcfg3 = if root > 0 { 0x00 } else { 0x01 };
        let pll2 = 0u8;

        bus.write_reg16(self.slv_addr, CKCFG1, ckcfg1)?;
        bus.write_reg16(self.slv_addr, CKCFG2, ckcfg2)?;
        bus.write_reg16(self.slv_addr, CKCFG3, ckcfg3)?;
        bus.write_reg16(self.slv_addr, PLL2, pll2)
    }

    fn set_xclk(&mut self, _bus: &mut B, _timer: i32, xclk: i32) -> Result<(), CameraError> {
        if xclk <= 0 {
            return Err(CameraError::InvalidArg);
        }
        self.xclk_freq_hz = (xclk as u32).saturating_mul(1_000_000);
        Ok(())
    }
}
