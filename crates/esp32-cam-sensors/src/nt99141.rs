//! NT99141 720p HD JPEG/YUV/RGB CMOS image sensor driver.
//!
//! Ported from `sensors/nt99141.c`, `sensors/private_include/nt99141_regs.h`, and
//! `sensors/private_include/nt99141_settings.h`.

use crate::{
    CameraError, CameraModel, CameraPid, CameraSccbAddr, CameraSensor, CameraStatus, FrameSize,
    GainCeiling, PixelFormat, RatioSettings, Reg16, SccbBus, SensorId,
};

pub const SYSTEM_CTROL0: u16 = 0x3021;
pub const FORMAT_CTRL: u16 = 0x501F;
pub const FORMAT_CTRL00: u16 = 0x4300;
pub const FRAME_CTRL01: u16 = 0x4201;
pub const FRAME_CTRL02: u16 = 0x4202;
pub const PRE_ISP_TEST_SETTING_1: u16 = 0x3025;
pub const AEC_PK_MANUAL: u16 = 0x3201;
pub const TIMING_TC_REG20: u16 = 0x3022;
pub const TIMING_TC_REG21: u16 = 0x3022;
pub const CLOCK_POL_CONTROL: u16 = 0x3024;
pub const DRIVE_CAPABILITY: u16 = 0x306A;

pub const X_ADDR_ST_H: u16 = 0x3800;
pub const X_ADDR_ST_L: u16 = 0x3801;
pub const Y_ADDR_ST_H: u16 = 0x3802;
pub const Y_ADDR_ST_L: u16 = 0x3803;
pub const X_ADDR_END_H: u16 = 0x3804;
pub const X_ADDR_END_L: u16 = 0x3805;
pub const Y_ADDR_END_H: u16 = 0x3806;
pub const Y_ADDR_END_L: u16 = 0x3807;
pub const X_OUTPUT_SIZE_H: u16 = 0x3808;
pub const X_OUTPUT_SIZE_L: u16 = 0x3809;
pub const Y_OUTPUT_SIZE_H: u16 = 0x380A;
pub const Y_OUTPUT_SIZE_L: u16 = 0x380B;
pub const X_TOTAL_SIZE_H: u16 = 0x380C;
pub const X_TOTAL_SIZE_L: u16 = 0x380D;
pub const Y_TOTAL_SIZE_H: u16 = 0x380E;
pub const Y_TOTAL_SIZE_L: u16 = 0x380F;
pub const X_OFFSET_H: u16 = 0x3810;
pub const X_OFFSET_L: u16 = 0x3811;
pub const Y_OFFSET_H: u16 = 0x3812;
pub const Y_OFFSET_L: u16 = 0x3813;
pub const X_INCREMENT: u16 = 0x3814;
pub const Y_INCREMENT: u16 = 0x3815;

pub const ISP_CONTROL_01: u16 = 0x3021;
pub const SCALE_CTRL_1: u16 = 0x5601;
pub const SCALE_CTRL_2: u16 = 0x5602;
pub const SCALE_CTRL_3: u16 = 0x5603;
pub const SCALE_CTRL_4: u16 = 0x5604;
pub const SCALE_CTRL_5: u16 = 0x5605;
pub const SCALE_CTRL_6: u16 = 0x5606;
pub const PCLK_RATIO: u16 = 0x3824;
pub const VFIFO_CTRL0C: u16 = 0x460C;
pub const VFIFO_X_SIZE_H: u16 = 0x4602;
pub const VFIFO_X_SIZE_L: u16 = 0x4603;
pub const VFIFO_Y_SIZE_H: u16 = 0x4604;
pub const VFIFO_Y_SIZE_L: u16 = 0x4605;

pub const SC_PLLS_CTRL0: u16 = 0x303A;
pub const SC_PLLS_CTRL1: u16 = 0x303B;
pub const SC_PLLS_CTRL2: u16 = 0x303C;
pub const SC_PLLS_CTRL3: u16 = 0x303D;

pub const COMPRESSION_CTRL00: u16 = 0x4400;
pub const COMPRESSION_CTRL01: u16 = 0x4401;
pub const COMPRESSION_CTRL02: u16 = 0x4402;
pub const COMPRESSION_CTRL03: u16 = 0x4403;
pub const COMPRESSION_CTRL04: u16 = 0x4404;
pub const COMPRESSION_CTRL05: u16 = 0x4405;
pub const COMPRESSION_CTRL06: u16 = 0x4406;
pub const COMPRESSION_CTRL07: u16 = 0x3401;
pub const COMPRESSION_ISI_CTRL: u16 = 0x4408;
pub const COMPRESSION_CTRL09: u16 = 0x4409;
pub const COMPRESSION_CTRL0A: u16 = 0x440A;
pub const COMPRESSION_CTRL0B: u16 = 0x440B;
pub const COMPRESSION_CTRL0C: u16 = 0x440C;
pub const COMPRESSION_CTRL0D: u16 = 0x440D;
pub const COMPRESSION_CTRL0E: u16 = 0x440E;

pub const TEST_COLOR_BAR: u8 = 0x02;
pub const AEC_PK_MANUAL_AGC_MANUALEN: u8 = 0x02;
pub const AEC_PK_MANUAL_AEC_MANUALEN: u8 = 0x01;
pub const TIMING_TC_REG20_VFLIP: u8 = 0x01;
pub const TIMING_TC_REG21_HMIRROR: u8 = 0x02;

pub const REG_DLY: u16 = 0xFFFF;
pub const REGLIST_TAIL: u16 = 0x0000;

pub const RATIO_TABLE: [RatioSettings; 1] = [RatioSettings {
    max_width: 1280,
    max_height: 720,
    start_x: 0,
    start_y: 4,
    end_x: 1283,
    end_y: 723,
    offset_x: 0,
    offset_y: 4,
    total_x: 1660,
    total_y: 963,
}];

pub const SENSOR_DEFAULT_REGS: &[Reg16] = &[
    Reg16::new(0x3021, 0x00),
    Reg16::new(REG_DLY, 100),
    Reg16::new(0x3109, 0x04),
    Reg16::new(0x3040, 0x04),
    Reg16::new(0x3041, 0x02),
    Reg16::new(0x3042, 0xFF),
    Reg16::new(0x3043, 0x08),
    Reg16::new(0x3052, 0xE0),
    Reg16::new(0x305F, 0x33),
    Reg16::new(0x3100, 0x07),
    Reg16::new(0x3106, 0x03),
    Reg16::new(0x3105, 0x01),
    Reg16::new(0x3108, 0x05),
    Reg16::new(0x3110, 0x22),
    Reg16::new(0x3111, 0x57),
    Reg16::new(0x3112, 0x22),
    Reg16::new(0x3113, 0x55),
    Reg16::new(0x3114, 0x05),
    Reg16::new(0x3135, 0x00),
    Reg16::new(0x32F0, 0x01),
    Reg16::new(0x3290, 0x01),
    Reg16::new(0x3291, 0x80),
    Reg16::new(0x3296, 0x01),
    Reg16::new(0x3297, 0x73),
    Reg16::new(0x3250, 0x80),
    Reg16::new(0x3251, 0x03),
    Reg16::new(0x3252, 0xFF),
    Reg16::new(0x3253, 0x00),
    Reg16::new(0x3254, 0x03),
    Reg16::new(0x3255, 0xFF),
    Reg16::new(0x3256, 0x00),
    Reg16::new(0x3257, 0x50),
    Reg16::new(0x3270, 0x00),
    Reg16::new(0x3271, 0x0C),
    Reg16::new(0x3272, 0x18),
    Reg16::new(0x3273, 0x32),
    Reg16::new(0x3274, 0x44),
    Reg16::new(0x3275, 0x54),
    Reg16::new(0x3276, 0x70),
    Reg16::new(0x3277, 0x88),
    Reg16::new(0x3278, 0x9D),
    Reg16::new(0x3279, 0xB0),
    Reg16::new(0x327A, 0xCF),
    Reg16::new(0x327B, 0xE2),
    Reg16::new(0x327C, 0xEF),
    Reg16::new(0x327D, 0xF7),
    Reg16::new(0x327E, 0xFF),
    Reg16::new(0x3302, 0x00),
    Reg16::new(0x3303, 0x40),
    Reg16::new(0x3304, 0x00),
    Reg16::new(0x3305, 0x96),
    Reg16::new(0x3306, 0x00),
    Reg16::new(0x3307, 0x29),
    Reg16::new(0x3308, 0x07),
    Reg16::new(0x3309, 0xBA),
    Reg16::new(0x330A, 0x06),
    Reg16::new(0x330B, 0xF5),
    Reg16::new(0x330C, 0x01),
    Reg16::new(0x330D, 0x51),
    Reg16::new(0x330E, 0x01),
    Reg16::new(0x330F, 0x30),
    Reg16::new(0x3310, 0x07),
    Reg16::new(0x3311, 0x16),
    Reg16::new(0x3312, 0x07),
    Reg16::new(0x3313, 0xBA),
    Reg16::new(0x3326, 0x02),
    Reg16::new(0x32F6, 0x0F),
    Reg16::new(0x32F9, 0x42),
    Reg16::new(0x32FA, 0x24),
    Reg16::new(0x3325, 0x4A),
    Reg16::new(0x3330, 0x00),
    Reg16::new(0x3331, 0x0A),
    Reg16::new(0x3332, 0xFF),
    Reg16::new(0x3338, 0x30),
    Reg16::new(0x3339, 0x84),
    Reg16::new(0x333A, 0x48),
    Reg16::new(0x333F, 0x07),
    Reg16::new(0x3360, 0x10),
    Reg16::new(0x3361, 0x18),
    Reg16::new(0x3362, 0x1F),
    Reg16::new(0x3363, 0x37),
    Reg16::new(0x3364, 0x80),
    Reg16::new(0x3365, 0x80),
    Reg16::new(0x3366, 0x68),
    Reg16::new(0x3367, 0x60),
    Reg16::new(0x3368, 0x30),
    Reg16::new(0x3369, 0x28),
    Reg16::new(0x336A, 0x20),
    Reg16::new(0x336B, 0x10),
    Reg16::new(0x336C, 0x00),
    Reg16::new(0x336D, 0x20),
    Reg16::new(0x336E, 0x1C),
    Reg16::new(0x336F, 0x18),
    Reg16::new(0x3370, 0x10),
    Reg16::new(0x3371, 0x38),
    Reg16::new(0x3372, 0x3C),
    Reg16::new(0x3373, 0x3F),
    Reg16::new(0x3374, 0x3F),
    Reg16::new(0x338A, 0x34),
    Reg16::new(0x338B, 0x7F),
    Reg16::new(0x338C, 0x10),
    Reg16::new(0x338D, 0x23),
    Reg16::new(0x338E, 0x7F),
    Reg16::new(0x338F, 0x14),
    Reg16::new(0x3375, 0x08),
    Reg16::new(0x3376, 0x0C),
    Reg16::new(0x3377, 0x18),
    Reg16::new(0x3378, 0x20),
    Reg16::new(0x3012, 0x02),
    Reg16::new(0x3013, 0xD0),
    Reg16::new(0x3025, 0x02),
];

pub const SENSOR_FMT_JPEG: &[Reg16] = &[Reg16::new(0x32F0, 0x70)];
pub const SENSOR_FMT_RAW: &[Reg16] = &[Reg16::new(0x32F0, 0x50)];
pub const SENSOR_FMT_GRAYSCALE: &[Reg16] = &[Reg16::new(0x32F1, 0x01)];
pub const SENSOR_FMT_YUV422: &[Reg16] = &[Reg16::new(0x32F0, 0x00)];
pub const SENSOR_FMT_RGB565: &[Reg16] = &[Reg16::new(0x32F0, 0x01)];

pub const SENSOR_SATURATION_LEVELS: [u8; 9] =
    [0x60, 0x68, 0x70, 0x78, 0x80, 0x88, 0x90, 0x98, 0xA0];

pub const SENSOR_SPECIAL_EFFECTS: [[u8; 4]; 7] = [
    [0x00, 0x80, 0x80, 0x01], // Normal
    [0x03, 0x80, 0x80, 0x01], // Negative
    [0x01, 0x80, 0x80, 0x01], // Grayscale
    [0x05, 0x2A, 0xF0, 0x01], // Red Tint
    [0x05, 0x60, 0x20, 0x01], // Green Tint
    [0x05, 0xF0, 0x80, 0x01], // Blue Tint
    [0x02, 0x80, 0x80, 0x01], // Sepia
];

pub const SENSOR_AE_LEVEL: [Reg16; 45] = [
    Reg16::new(0x32B8, 0x29),
    Reg16::new(0x32B9, 0x1F),
    Reg16::new(0x32BC, 0x24),
    Reg16::new(0x32BD, 0x27),
    Reg16::new(0x32BE, 0x21),
    Reg16::new(0x32B8, 0x2D),
    Reg16::new(0x32B9, 0x23),
    Reg16::new(0x32BC, 0x28),
    Reg16::new(0x32BD, 0x2B),
    Reg16::new(0x32BE, 0x25),
    Reg16::new(0x32B8, 0x32),
    Reg16::new(0x32B9, 0x26),
    Reg16::new(0x32BC, 0x2C),
    Reg16::new(0x32BD, 0x2F),
    Reg16::new(0x32BE, 0x29),
    Reg16::new(0x32B8, 0x36),
    Reg16::new(0x32B9, 0x2A),
    Reg16::new(0x32BC, 0x30),
    Reg16::new(0x32BD, 0x33),
    Reg16::new(0x32BE, 0x2D),
    Reg16::new(0x32B8, 0x3B),
    Reg16::new(0x32B9, 0x2D),
    Reg16::new(0x32BC, 0x34),
    Reg16::new(0x32BD, 0x38),
    Reg16::new(0x32BE, 0x30),
    Reg16::new(0x32B8, 0x3F),
    Reg16::new(0x32B9, 0x31),
    Reg16::new(0x32BC, 0x38),
    Reg16::new(0x32BD, 0x3C),
    Reg16::new(0x32BE, 0x34),
    Reg16::new(0x32B8, 0x44),
    Reg16::new(0x32B9, 0x34),
    Reg16::new(0x32BC, 0x3C),
    Reg16::new(0x32BD, 0x40),
    Reg16::new(0x32BE, 0x38),
    Reg16::new(0x32B8, 0x48),
    Reg16::new(0x32B9, 0x38),
    Reg16::new(0x32BC, 0x40),
    Reg16::new(0x32BD, 0x44),
    Reg16::new(0x32BE, 0x3C),
    Reg16::new(0x32B8, 0x4D),
    Reg16::new(0x32B9, 0x3B),
    Reg16::new(0x32BC, 0x44),
    Reg16::new(0x32BD, 0x49),
    Reg16::new(0x32BE, 0x3F),
];

pub const SENSOR_FRAMESIZE_HD: &[Reg16] = &[
    Reg16::new(0x3021, 0x00),
    Reg16::new(REG_DLY, 100),
    Reg16::new(0x32BF, 0x60),
    Reg16::new(0x32C0, 0x5A),
    Reg16::new(0x32C1, 0x5A),
    Reg16::new(0x32C2, 0x5A),
    Reg16::new(0x32C3, 0x00),
    Reg16::new(0x32C4, 0x20),
    Reg16::new(0x32C5, 0x20),
    Reg16::new(0x32C6, 0x20),
    Reg16::new(0x32C7, 0x00),
    Reg16::new(0x32C8, 0x3C),
    Reg16::new(0x32C9, 0x5A),
    Reg16::new(0x32CA, 0x7A),
    Reg16::new(0x32CB, 0x7A),
    Reg16::new(0x32CC, 0x7A),
    Reg16::new(0x32CD, 0x7A),
    Reg16::new(0x32DB, 0x5E),
    Reg16::new(0x32F0, 0x70),
    Reg16::new(0x3400, 0x08),
    Reg16::new(0x3400, 0x00),
    Reg16::new(0x3401, 0x4E),
    Reg16::new(0x3404, 0x00),
    Reg16::new(0x3405, 0x00),
    Reg16::new(0x3410, 0x00),
    Reg16::new(0x3200, 0x3E),
    Reg16::new(0x3201, 0x0F),
    Reg16::new(0x3028, 0x0F),
    Reg16::new(0x3029, 0x00),
    Reg16::new(0x302A, 0x08),
    Reg16::new(0x3022, 0x24),
    Reg16::new(0x3023, 0x24),
    Reg16::new(0x3002, 0x00),
    Reg16::new(0x3003, 0x04),
    Reg16::new(0x3004, 0x00),
    Reg16::new(0x3005, 0x04),
    Reg16::new(0x3006, 0x05),
    Reg16::new(0x3007, 0x03),
    Reg16::new(0x3008, 0x02),
    Reg16::new(0x3009, 0xD3),
    Reg16::new(0x300A, 0x06),
    Reg16::new(0x300B, 0x7C),
    Reg16::new(0x300C, 0x02),
    Reg16::new(0x300D, 0xE0),
    Reg16::new(0x300E, 0x05),
    Reg16::new(0x300F, 0x00),
    Reg16::new(0x3010, 0x02),
    Reg16::new(0x3011, 0xD0),
    Reg16::new(0x32B8, 0x3F),
    Reg16::new(0x32B9, 0x31),
    Reg16::new(0x32BB, 0x87),
    Reg16::new(0x32BC, 0x38),
    Reg16::new(0x32BD, 0x3C),
    Reg16::new(0x32BE, 0x34),
    Reg16::new(0x3201, 0x3F),
    Reg16::new(0x3021, 0x06),
    Reg16::new(0x3025, 0x00),
    Reg16::new(0x3400, 0x01),
    Reg16::new(0x3060, 0x01),
];

pub const SENSOR_FRAMESIZE_VGA: &[Reg16] = &[
    Reg16::new(0x3021, 0x00),
    Reg16::new(REG_DLY, 100),
    Reg16::new(0x32BF, 0x60),
    Reg16::new(0x32C0, 0x5A),
    Reg16::new(0x32C1, 0x5A),
    Reg16::new(0x32C2, 0x5A),
    Reg16::new(0x32C3, 0x00),
    Reg16::new(0x32C4, 0x20),
    Reg16::new(0x32C5, 0x20),
    Reg16::new(0x32C6, 0x20),
    Reg16::new(0x32C7, 0x00),
    Reg16::new(0x32C8, 0x4B),
    Reg16::new(0x32C9, 0x5A),
    Reg16::new(0x32CA, 0x7A),
    Reg16::new(0x32CB, 0x7A),
    Reg16::new(0x32CC, 0x7A),
    Reg16::new(0x32CD, 0x7A),
    Reg16::new(0x32DB, 0x62),
    Reg16::new(0x32F0, 0x70),
    Reg16::new(0x3400, 0x08),
    Reg16::new(0x3400, 0x00),
    Reg16::new(0x3401, 0x4E),
    Reg16::new(0x3404, 0x00),
    Reg16::new(0x3405, 0x00),
    Reg16::new(0x3410, 0x00),
    Reg16::new(0x32E0, 0x02),
    Reg16::new(0x32E1, 0x80),
    Reg16::new(0x32E2, 0x01),
    Reg16::new(0x32E3, 0xE0),
    Reg16::new(0x32E4, 0x00),
    Reg16::new(0x32E5, 0x80),
    Reg16::new(0x32E6, 0x00),
    Reg16::new(0x32E7, 0x80),
    Reg16::new(0x3200, 0x3E),
    Reg16::new(0x3201, 0x0F),
    Reg16::new(0x3028, 0x0F),
    Reg16::new(0x3029, 0x00),
    Reg16::new(0x302A, 0x08),
    Reg16::new(0x3022, 0x24),
    Reg16::new(0x3023, 0x24),
    Reg16::new(0x3002, 0x00),
    Reg16::new(0x3003, 0xA4),
    Reg16::new(0x3004, 0x00),
    Reg16::new(0x3005, 0x04),
    Reg16::new(0x3006, 0x04),
    Reg16::new(0x3007, 0x63),
    Reg16::new(0x3008, 0x02),
    Reg16::new(0x3009, 0xD3),
    Reg16::new(0x300A, 0x05),
    Reg16::new(0x300B, 0x3C),
    Reg16::new(0x300C, 0x02),
    Reg16::new(0x300D, 0xE0),
    Reg16::new(0x300E, 0x03),
    Reg16::new(0x300F, 0xC0),
    Reg16::new(0x3010, 0x02),
    Reg16::new(0x3011, 0xD0),
    Reg16::new(0x32B8, 0x3F),
    Reg16::new(0x32B9, 0x31),
    Reg16::new(0x32BB, 0x87),
    Reg16::new(0x32BC, 0x38),
    Reg16::new(0x32BD, 0x3C),
    Reg16::new(0x32BE, 0x34),
    Reg16::new(0x3201, 0x7F),
    Reg16::new(0x3021, 0x06),
    Reg16::new(0x3025, 0x00),
    Reg16::new(0x3400, 0x01),
    Reg16::new(0x3060, 0x01),
];

pub const SENSOR_FRAMESIZE_QVGA: &[Reg16] = &[
    Reg16::new(0x3021, 0x00),
    Reg16::new(REG_DLY, 100),
    Reg16::new(0x32BF, 0x60),
    Reg16::new(0x32C0, 0x5A),
    Reg16::new(0x32C1, 0x5A),
    Reg16::new(0x32C2, 0x5A),
    Reg16::new(0x32C3, 0x00),
    Reg16::new(0x32C4, 0x20),
    Reg16::new(0x32C5, 0x20),
    Reg16::new(0x32C6, 0x20),
    Reg16::new(0x32C7, 0x00),
    Reg16::new(0x32C8, 0x4B),
    Reg16::new(0x32C9, 0x5A),
    Reg16::new(0x32CA, 0x7A),
    Reg16::new(0x32CB, 0x7A),
    Reg16::new(0x32CC, 0x7A),
    Reg16::new(0x32CD, 0x7A),
    Reg16::new(0x32DB, 0x62),
    Reg16::new(0x32F0, 0x70),
    Reg16::new(0x3400, 0x08),
    Reg16::new(0x3400, 0x00),
    Reg16::new(0x3401, 0x4E),
    Reg16::new(0x3404, 0x00),
    Reg16::new(0x3405, 0x00),
    Reg16::new(0x3410, 0x00),
    Reg16::new(0x32E0, 0x01),
    Reg16::new(0x32E1, 0x40),
    Reg16::new(0x32E2, 0x00),
    Reg16::new(0x32E3, 0xF0),
    Reg16::new(0x32E4, 0x02),
    Reg16::new(0x32E5, 0x02),
    Reg16::new(0x32E6, 0x02),
    Reg16::new(0x32E7, 0x03),
    Reg16::new(0x3200, 0x3E),
    Reg16::new(0x3201, 0x0F),
    Reg16::new(0x3028, 0x0F),
    Reg16::new(0x3029, 0x00),
    Reg16::new(0x302A, 0x08),
    Reg16::new(0x3022, 0x24),
    Reg16::new(0x3023, 0x24),
    Reg16::new(0x3002, 0x00),
    Reg16::new(0x3003, 0xA4),
    Reg16::new(0x3004, 0x00),
    Reg16::new(0x3005, 0x04),
    Reg16::new(0x3006, 0x04),
    Reg16::new(0x3007, 0x63),
    Reg16::new(0x3008, 0x02),
    Reg16::new(0x3009, 0xD3),
    Reg16::new(0x300A, 0x05),
    Reg16::new(0x300B, 0x3C),
    Reg16::new(0x300C, 0x02),
    Reg16::new(0x300D, 0xE0),
    Reg16::new(0x300E, 0x03),
    Reg16::new(0x300F, 0xC0),
    Reg16::new(0x3010, 0x02),
    Reg16::new(0x3011, 0xD0),
    Reg16::new(0x32B8, 0x3F),
    Reg16::new(0x32B9, 0x31),
    Reg16::new(0x32BB, 0x87),
    Reg16::new(0x32BC, 0x38),
    Reg16::new(0x32BD, 0x3C),
    Reg16::new(0x32BE, 0x34),
    Reg16::new(0x3201, 0x7F),
    Reg16::new(0x3021, 0x06),
    Reg16::new(0x3025, 0x00),
    Reg16::new(0x3400, 0x01),
    Reg16::new(0x3060, 0x01),
];

pub const SENSOR_FRAMESIZE_VGA_XYSKIP: &[Reg16] = &[
    Reg16::new(0x3021, 0x00),
    Reg16::new(REG_DLY, 100),
    Reg16::new(0x32BF, 0x60),
    Reg16::new(0x320A, 0xB2),
    Reg16::new(0x32C0, 0x64),
    Reg16::new(0x32C1, 0x64),
    Reg16::new(0x32C2, 0x64),
    Reg16::new(0x32C3, 0x00),
    Reg16::new(0x32C4, 0x20),
    Reg16::new(0x32C5, 0x20),
    Reg16::new(0x32C6, 0x20),
    Reg16::new(0x32C7, 0x00),
    Reg16::new(0x32C8, 0x62),
    Reg16::new(0x32C9, 0x64),
    Reg16::new(0x32CA, 0x84),
    Reg16::new(0x32CB, 0x84),
    Reg16::new(0x32CC, 0x84),
    Reg16::new(0x32CD, 0x84),
    Reg16::new(0x32DB, 0x68),
    Reg16::new(0x32F0, 0x70),
    Reg16::new(0x3400, 0x08),
    Reg16::new(0x3400, 0x00),
    Reg16::new(0x3401, 0x4E),
    Reg16::new(0x3404, 0x00),
    Reg16::new(0x3405, 0x00),
    Reg16::new(0x3410, 0x00),
    Reg16::new(0x3200, 0x3E),
    Reg16::new(0x3201, 0x0F),
    Reg16::new(0x3028, 0x0F),
    Reg16::new(0x3029, 0x00),
    Reg16::new(0x302A, 0x08),
    Reg16::new(0x3022, 0x24),
    Reg16::new(0x3023, 0x6C),
    Reg16::new(0x3002, 0x00),
    Reg16::new(0x3003, 0x04),
    Reg16::new(0x3004, 0x00),
    Reg16::new(0x3005, 0x04),
    Reg16::new(0x3006, 0x05),
    Reg16::new(0x3007, 0x03),
    Reg16::new(0x3008, 0x02),
    Reg16::new(0x3009, 0xD3),
    Reg16::new(0x300A, 0x03),
    Reg16::new(0x300B, 0xFC),
    Reg16::new(0x300C, 0x01),
    Reg16::new(0x300D, 0x88),
    Reg16::new(0x300E, 0x02),
    Reg16::new(0x300F, 0x80),
    Reg16::new(0x3010, 0x01),
    Reg16::new(0x3011, 0x68),
    Reg16::new(0x32B8, 0x3F),
    Reg16::new(0x32B9, 0x31),
    Reg16::new(0x32BB, 0x87),
    Reg16::new(0x32BC, 0x38),
    Reg16::new(0x32BD, 0x3C),
    Reg16::new(0x32BE, 0x34),
    Reg16::new(0x3201, 0x3F),
    Reg16::new(0x3025, 0x00),
    Reg16::new(0x3021, 0x06),
    Reg16::new(0x3400, 0x01),
    Reg16::new(0x3060, 0x01),
];

/// Writes a slice of [`Reg16`] entries to `slv_addr`, interpreting `REG_DLY` (`0xFFFF`) as a delay in ms
/// and `REGLIST_TAIL` (`0x0000`) as an early list terminator.
pub fn write_reg_list<B: SccbBus>(
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

#[inline]
fn write_addr_reg<B: SccbBus>(
    bus: &mut B,
    slv_addr: u8,
    reg: u16,
    x_value: u16,
    y_value: u16,
) -> Result<(), CameraError> {
    bus.write_reg16_val16(slv_addr, reg, x_value)?;
    bus.write_reg16_val16(slv_addr, reg.wrapping_add(2), y_value)
}

/// Detects an NT99141 sensor at `slv_addr` (`0x2A`).
pub fn detect<B: SccbBus>(bus: &mut B, slv_addr: u8) -> Result<SensorId, CameraError> {
    Nt99141::detect(bus, slv_addr)
}

/// Safe `#![no_std]` driver for the Novatek NT99141 720p HD camera sensor.
#[derive(Debug, Clone)]
pub struct Nt99141 {
    id: SensorId,
    status: CameraStatus,
    slv_addr: u8,
    pixformat: PixelFormat,
    xclk_freq_hz: u32,
}

pub type NT99141 = Nt99141;

impl Default for Nt99141 {
    fn default() -> Self {
        Self::new()
    }
}

impl Nt99141 {
    /// Creates a new [`Nt99141`] instance with default SCCB address (`0x2A`) and PID (`0x1410`).
    pub const fn new() -> Self {
        Self {
            id: SensorId {
                midh: 0,
                midl: 0,
                pid: CameraPid::NT99141.as_u16(),
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
            slv_addr: CameraSccbAddr::NT99141,
            pixformat: PixelFormat::Jpeg,
            xclk_freq_hz: 10_000_000,
        }
    }

    /// Creates a new [`Nt99141`] with an explicit slave address and [`SensorId`].
    pub const fn with_addr(slv_addr: u8, id: SensorId) -> Self {
        Self {
            id,
            slv_addr,
            ..Self::new()
        }
    }

    /// Probes the SCCB bus at `slv_addr` and verifies `NT99141_PID` (`0x1410`).
    pub fn detect<B: SccbBus>(bus: &mut B, slv_addr: u8) -> Result<SensorId, CameraError> {
        if slv_addr != CameraSccbAddr::NT99141 {
            return Err(CameraError::NotDetected);
        }
        bus.write_reg16(slv_addr, 0x3008, 0x01)?;
        let h = bus.read_reg16(slv_addr, 0x3000)? as u16;
        let l = bus.read_reg16(slv_addr, 0x3001)? as u16;
        let pid = (h << 8) | l;
        if pid == CameraPid::NT99141.as_u16() {
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

    /// Returns the 7-bit SCCB slave address (`0x2A`).
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

    /// Returns the [`CameraModel`] (`CameraModel::Nt99141`).
    pub const fn model(&self) -> CameraModel {
        CameraModel::Nt99141
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

    fn set_image_options<B: SccbBus>(&self, bus: &mut B) -> Result<(), CameraError> {
        let mut bits = 0u8;
        if self.status.vflip != 0 {
            bits |= TIMING_TC_REG20_VFLIP;
        }
        if self.status.hmirror != 0 {
            bits |= TIMING_TC_REG21_HMIRROR;
        }
        set_reg_bits(bus, self.slv_addr, TIMING_TC_REG20, 0, 0x03, bits)
    }
}

impl<B: SccbBus> CameraSensor<B> for Nt99141 {
    fn model(&self) -> CameraModel {
        CameraModel::Nt99141
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
        bus.write_reg16(self.slv_addr, SYSTEM_CTROL0, 0x01)?;
        bus.delay_ms(100);
        write_reg_list(bus, self.slv_addr, SENSOR_DEFAULT_REGS)?;
        self.set_ae_level(bus, 0)?;
        bus.delay_ms(100);
        Ok(())
    }

    fn init_status(&mut self, bus: &mut B) -> Result<(), CameraError> {
        self.status.brightness = 0;
        self.status.contrast = 0;
        self.status.saturation = 0;
        self.status.sharpness = bus.read_reg16(self.slv_addr, 0x3301)? as i8;
        self.status.denoise = (bus.read_reg16(self.slv_addr, 0x5306)? / 4).wrapping_add(1);
        self.status.ae_level = 0;
        self.status.gainceiling = bus.read_reg16(self.slv_addr, 0x32F0)?;
        self.status.awb = check_reg_mask(bus, self.slv_addr, ISP_CONTROL_01, 0x10)?;
        self.status.dcw = u8::from(check_reg_mask(bus, self.slv_addr, 0x5183, 0x80)? == 0);
        self.status.agc = u8::from(
            check_reg_mask(bus, self.slv_addr, AEC_PK_MANUAL, AEC_PK_MANUAL_AGC_MANUALEN)? == 0,
        );
        self.status.aec = u8::from(
            check_reg_mask(bus, self.slv_addr, AEC_PK_MANUAL, AEC_PK_MANUAL_AEC_MANUALEN)? == 0,
        );
        self.status.hmirror =
            check_reg_mask(bus, self.slv_addr, TIMING_TC_REG21, TIMING_TC_REG21_HMIRROR)?;
        self.status.vflip =
            check_reg_mask(bus, self.slv_addr, TIMING_TC_REG20, TIMING_TC_REG20_VFLIP)?;
        self.status.colorbar =
            check_reg_mask(bus, self.slv_addr, PRE_ISP_TEST_SETTING_1, TEST_COLOR_BAR)?;
        self.status.bpc = check_reg_mask(bus, self.slv_addr, 0x5000, 0x04)?;
        self.status.wpc = check_reg_mask(bus, self.slv_addr, 0x5000, 0x02)?;
        self.status.raw_gma = check_reg_mask(bus, self.slv_addr, 0x5000, 0x20)?;
        self.status.lenc = check_reg_mask(bus, self.slv_addr, 0x5000, 0x80)?;
        self.status.quality = bus.read_reg16(self.slv_addr, COMPRESSION_CTRL07)? & 0x3F;
        self.status.special_effect = 0;
        self.status.wb_mode = 0;
        self.status.awb_gain = check_reg_mask(bus, self.slv_addr, 0x3000, 0x01)?;
        self.status.agc_gain = 0;
        self.status.aec_value = 0;
        self.status.aec2 = check_reg_mask(bus, self.slv_addr, 0x3000, 0x04)?;
        Ok(())
    }

    fn set_pixformat(&mut self, bus: &mut B, pixformat: PixelFormat) -> Result<(), CameraError> {
        let regs = match pixformat {
            PixelFormat::Yuv422 => SENSOR_FMT_YUV422,
            PixelFormat::Grayscale => SENSOR_FMT_GRAYSCALE,
            PixelFormat::Rgb565 | PixelFormat::Rgb888 => SENSOR_FMT_RGB565,
            PixelFormat::Jpeg => SENSOR_FMT_JPEG,
            PixelFormat::Raw => SENSOR_FMT_RAW,
            _ => return Err(CameraError::NotSupported),
        };
        write_reg_list(bus, self.slv_addr, regs)?;
        self.pixformat = pixformat;
        Ok(())
    }

    fn set_framesize(&mut self, bus: &mut B, framesize: FrameSize) -> Result<(), CameraError> {
        if framesize > FrameSize::Hd {
            return Err(CameraError::FailedToSetFrameSize);
        }
        self.status.framesize = framesize;
        write_reg_list(bus, self.slv_addr, SENSOR_DEFAULT_REGS)?;
        if framesize == FrameSize::Qvga {
            write_reg_list(bus, self.slv_addr, SENSOR_FRAMESIZE_QVGA)?;
        } else if framesize == FrameSize::Vga {
            write_reg_list(bus, self.slv_addr, SENSOR_FRAMESIZE_VGA_XYSKIP)?;
        } else if framesize >= FrameSize::Hd {
            write_reg_list(bus, self.slv_addr, SENSOR_FRAMESIZE_HD)?;
        } else {
            write_reg_list(bus, self.slv_addr, SENSOR_FRAMESIZE_VGA)?;
        }
        let pixformat = self.pixformat;
        self.set_pixformat(bus, pixformat)
    }

    fn set_contrast(&mut self, bus: &mut B, level: i32) -> Result<(), CameraError> {
        let (value1, value2) = match level {
            3 => (0xD0, 0xB0),
            2 => (0xE0, 0xA0),
            1 => (0xF0, 0x90),
            0 => (0x00, 0x80),
            -1 => (0x10, 0x70),
            -2 => (0x20, 0x60),
            -3 => (0x30, 0x50),
            _ => (0x00, 0x00),
        };
        bus.write_reg16(self.slv_addr, 0x32FC, value1)?;
        bus.write_reg16(self.slv_addr, 0x32F2, value2)?;
        bus.write_reg16(self.slv_addr, 0x3060, 0x01)?;
        self.status.contrast = level as i8;
        Ok(())
    }

    fn set_brightness(&mut self, bus: &mut B, level: i32) -> Result<(), CameraError> {
        let value = match level {
            3 => 0xA0,
            2 => 0x90,
            1 => 0x88,
            -1 => 0x78,
            -2 => 0x70,
            -3 => 0x60,
            _ => 0x00,
        };
        bus.write_reg16(self.slv_addr, 0x32F2, value)?;
        bus.write_reg16(self.slv_addr, 0x3060, 0x01)?;
        self.status.brightness = level as i8;
        Ok(())
    }

    fn set_saturation(&mut self, bus: &mut B, level: i32) -> Result<(), CameraError> {
        if !(-4..=4).contains(&level) {
            return Err(CameraError::InvalidArg);
        }
        let reg_val = SENSOR_SATURATION_LEVELS[(level + 4) as usize];
        bus.write_reg16(self.slv_addr, 0x32F3, reg_val)?;
        bus.write_reg16(self.slv_addr, 0x3060, 0x01)?;
        self.status.saturation = level as i8;
        Ok(())
    }

    fn set_sharpness(&mut self, bus: &mut B, level: i32) -> Result<(), CameraError> {
        if !(-3..=3).contains(&level) {
            return Err(CameraError::InvalidArg);
        }
        let mt_offset_2 = ((level + 3) * 8) as u8;
        let mt_offset_1 = mt_offset_2.wrapping_add(1);
        write_reg_bits(bus, self.slv_addr, 0x5308, 0x40, false)?;
        bus.write_reg16(self.slv_addr, 0x5300, 0x10)?;
        bus.write_reg16(self.slv_addr, 0x5301, 0x10)?;
        bus.write_reg16(self.slv_addr, 0x5302, mt_offset_1)?;
        bus.write_reg16(self.slv_addr, 0x5303, mt_offset_2)?;
        bus.write_reg16(self.slv_addr, 0x5309, 0x10)?;
        bus.write_reg16(self.slv_addr, 0x530A, 0x10)?;
        bus.write_reg16(self.slv_addr, 0x530B, 0x04)?;
        bus.write_reg16(self.slv_addr, 0x530C, 0x06)?;
        self.status.sharpness = level as i8;
        Ok(())
    }

    fn set_denoise(&mut self, _bus: &mut B, _level: i32) -> Result<(), CameraError> {
        Ok(())
    }

    fn set_gainceiling(
        &mut self,
        _bus: &mut B,
        _gainceiling: GainCeiling,
    ) -> Result<(), CameraError> {
        Ok(())
    }

    fn set_quality(&mut self, bus: &mut B, quality: i32) -> Result<(), CameraError> {
        let qs = (quality as u8) & 0x3F;
        bus.write_reg16(self.slv_addr, COMPRESSION_CTRL07, qs)?;
        self.status.quality = quality as u8;
        Ok(())
    }

    fn set_colorbar(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        write_reg_bits(bus, self.slv_addr, PRE_ISP_TEST_SETTING_1, TEST_COLOR_BAR, enable)?;
        self.status.colorbar = u8::from(enable);
        Ok(())
    }

    fn set_whitebal(&mut self, _bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.status.awb = u8::from(enable);
        Ok(())
    }

    fn set_gain_ctrl(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        write_reg_bits(bus, self.slv_addr, 0x32BB, 0x87, enable)?;
        self.status.agc = u8::from(enable);
        Ok(())
    }

    fn set_exposure_ctrl(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        let data = bus.read_reg16(self.slv_addr, 0x3201)?;
        let next = if enable {
            data | (1 << 5)
        } else {
            data & !(1 << 5)
        };
        bus.write_reg16(self.slv_addr, 0x3201, next)?;
        self.status.aec = u8::from(enable);
        Ok(())
    }

    fn set_hmirror(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.status.hmirror = u8::from(enable);
        self.set_image_options(bus)
    }

    fn set_vflip(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.status.vflip = u8::from(enable);
        self.set_image_options(bus)
    }

    fn set_aec2(&mut self, _bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.status.aec2 = u8::from(enable);
        Ok(())
    }

    fn set_awb_gain(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        let old_mode = self.status.wb_mode;
        let mode = if enable { old_mode as i32 } else { 0 };
        self.set_wb_mode(bus, mode)?;
        self.status.wb_mode = old_mode;
        self.status.awb_gain = u8::from(enable);
        Ok(())
    }

    fn set_agc_gain(&mut self, bus: &mut B, gain: i32) -> Result<(), CameraError> {
        let cnt = gain / 2;
        let val = match cnt {
            0 => Some(0x00),
            1 => Some(0x0F),
            2 => Some(0x2F),
            3 => Some(0x37),
            4 => Some(0x3F),
            _ => None,
        };
        if let Some(v) = val {
            bus.write_reg16(self.slv_addr, 0x301D, v)?;
            self.status.agc_gain = gain as u8;
        }
        Ok(())
    }

    fn set_aec_value(&mut self, bus: &mut B, gain: i32) -> Result<(), CameraError> {
        bus.write_reg16(self.slv_addr, 0x3012, ((gain >> 8) & 0xFF) as u8)?;
        bus.write_reg16(self.slv_addr, 0x3013, (gain & 0xFF) as u8)?;
        self.status.aec_value = gain as u16;
        Ok(())
    }

    fn set_special_effect(&mut self, bus: &mut B, effect: i32) -> Result<(), CameraError> {
        if !(0..=6).contains(&effect) {
            return Err(CameraError::InvalidArg);
        }
        let regs = SENSOR_SPECIAL_EFFECTS[effect as usize];
        bus.write_reg16(self.slv_addr, 0x32F1, regs[0])?;
        bus.write_reg16(self.slv_addr, 0x32F4, regs[1])?;
        bus.write_reg16(self.slv_addr, 0x32F5, regs[2])?;
        bus.write_reg16(self.slv_addr, 0x3060, regs[3])?;
        self.status.special_effect = effect as u8;
        Ok(())
    }

    fn set_wb_mode(&mut self, bus: &mut B, mode: i32) -> Result<(), CameraError> {
        if !(0..=4).contains(&mode) {
            return Err(CameraError::InvalidArg);
        }
        write_reg_bits(bus, self.slv_addr, 0x3201, 0x01, mode != 0)?;
        let preset = match mode {
            1 => Some((0x01, 0x38, 0x01, 0x68, 0x01)), // Sunny
            2 => Some((0x01, 0x51, 0x01, 0x00, 0x01)), // Cloudy
            3 => Some((0x01, 0x30, 0x01, 0xCB, 0x01)), // Incandescence
            4 => Some((0x01, 0x70, 0x01, 0xFF, 0x01)), // Fluorescent
            _ => None,
        };
        if let Some((r90, r91, r96, r97, r60)) = preset {
            bus.write_reg16(self.slv_addr, 0x3290, r90)?;
            bus.write_reg16(self.slv_addr, 0x3291, r91)?;
            bus.write_reg16(self.slv_addr, 0x3296, r96)?;
            bus.write_reg16(self.slv_addr, 0x3297, r97)?;
            bus.write_reg16(self.slv_addr, 0x3060, r60)?;
        }
        self.status.wb_mode = mode as u8;
        Ok(())
    }

    fn set_ae_level(&mut self, bus: &mut B, level: i32) -> Result<(), CameraError> {
        let clamped = level.clamp(0, 8) as usize;
        let base = clamped * 5;
        for i in 0..5 {
            let entry = SENSOR_AE_LEVEL[base + i];
            bus.write_reg16(self.slv_addr, entry.reg, entry.val)?;
        }
        bus.write_reg16(self.slv_addr, 0x3060, 0x01)?;
        self.status.ae_level = clamped as i8;
        Ok(())
    }

    fn set_dcw(&mut self, _bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.status.dcw = u8::from(enable);
        Ok(())
    }

    fn set_bpc(&mut self, _bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.status.bpc = u8::from(enable);
        Ok(())
    }

    fn set_wpc(&mut self, _bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.status.wpc = u8::from(enable);
        Ok(())
    }

    fn set_raw_gma(&mut self, _bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.status.raw_gma = u8::from(enable);
        Ok(())
    }

    fn set_lenc(&mut self, _bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.status.lenc = u8::from(enable);
        Ok(())
    }

    fn get_reg(&mut self, bus: &mut B, reg: i32, mask: i32) -> Result<i32, CameraError> {
        if !(0..=0xFFFF).contains(&reg) || mask < 0 {
            return Err(CameraError::InvalidArg);
        }
        let reg16 = reg as u16;
        let mut ret = if mask > 0xFF {
            let mut v = bus.read_reg16_val16(self.slv_addr, reg16)? as i32;
            if mask > 0xFFFF {
                let low = bus.read_reg16(self.slv_addr, reg16.wrapping_add(2))? as i32;
                v = (v << 8) | low;
            }
            v
        } else {
            bus.read_reg16(self.slv_addr, reg16)? as i32
        };
        if ret > 0 {
            ret &= mask;
        }
        Ok(ret)
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
                let low = bus.read_reg16(self.slv_addr, reg16.wrapping_add(2))? as i32;
                v = (v << 8) | low;
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
        bus: &mut B,
        start_x: i32,
        start_y: i32,
        end_x: i32,
        end_y: i32,
        offset_x: i32,
        offset_y: i32,
        total_x: i32,
        total_y: i32,
        output_x: i32,
        output_y: i32,
        scale: bool,
        binning: bool,
    ) -> Result<(), CameraError> {
        write_addr_reg(bus, self.slv_addr, X_ADDR_ST_H, start_x as u16, start_y as u16)?;
        write_addr_reg(bus, self.slv_addr, X_ADDR_END_H, end_x as u16, end_y as u16)?;
        write_addr_reg(bus, self.slv_addr, X_OFFSET_H, offset_x as u16, offset_y as u16)?;
        write_addr_reg(bus, self.slv_addr, X_TOTAL_SIZE_H, total_x as u16, total_y as u16)?;
        write_addr_reg(bus, self.slv_addr, X_OUTPUT_SIZE_H, output_x as u16, output_y as u16)?;
        self.status.scale = scale;
        self.status.binning = binning;
        self.set_image_options(bus)
    }

    fn set_pll(
        &mut self,
        _bus: &mut B,
        _bypass: i32,
        _mul: i32,
        _sys: i32,
        _root: i32,
        _pre: i32,
        _seld5: i32,
        _pclken: i32,
        _pclk: i32,
    ) -> Result<(), CameraError> {
        Err(CameraError::NotSupported)
    }

    fn set_xclk(&mut self, _bus: &mut B, _timer: i32, xclk: i32) -> Result<(), CameraError> {
        if xclk <= 0 {
            return Err(CameraError::InvalidArg);
        }
        let clamped_mhz = (xclk as u32).min(10);
        self.xclk_freq_hz = clamped_mhz * 1_000_000;
        Ok(())
    }
}
