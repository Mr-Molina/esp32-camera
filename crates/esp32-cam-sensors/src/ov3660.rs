//! Safe `#![no_std]` OV3660 (3MP QXGA JPEG/RGB565/YUV422/RAW) camera sensor driver.
//!
//! Ported from `sensors/ov3660.c`, `sensors/private_include/ov3660_regs.h`, and
//! `sensors/private_include/ov3660_settings.h`.

use crate::{
    AspectRatio, CameraError, CameraModel, CameraPid, CameraSccbAddr, CameraSensor, CameraStatus,
    FrameSize, GainCeiling, PixelFormat, RatioSettings, Reg16, SccbBus, SensorId,
};

pub const SYSTEM_CTROL0: u16 = 0x3008;
pub const DRIVE_CAPABILITY: u16 = 0x302C;
pub const SC_PLLS_CTRL0: u16 = 0x303A;
pub const SC_PLLS_CTRL1: u16 = 0x303B;
pub const SC_PLLS_CTRL2: u16 = 0x303C;
pub const SC_PLLS_CTRL3: u16 = 0x303D;
pub const AEC_PK_MANUAL: u16 = 0x3503;
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
pub const TIMING_TC_REG20: u16 = 0x3820;
pub const TIMING_TC_REG21: u16 = 0x3821;
pub const PCLK_RATIO: u16 = 0x3824;
pub const FRAME_CTRL01: u16 = 0x4201;
pub const FRAME_CTRL02: u16 = 0x4202;
pub const FORMAT_CTRL00: u16 = 0x4300;
pub const COMPRESSION_CTRL07: u16 = 0x4407;
pub const COMPRESSION_CTRL0E: u16 = 0x440E;
pub const VFIFO_X_SIZE_H: u16 = 0x4602;
pub const VFIFO_X_SIZE_L: u16 = 0x4603;
pub const VFIFO_Y_SIZE_H: u16 = 0x4604;
pub const VFIFO_Y_SIZE_L: u16 = 0x4605;
pub const VFIFO_CTRL0C: u16 = 0x460C;
pub const CLOCK_POL_CONTROL: u16 = 0x4740;
pub const ISP_CONTROL_01: u16 = 0x5001;
pub const FORMAT_CTRL: u16 = 0x501F;
pub const PRE_ISP_TEST_SETTING_1: u16 = 0x503D;
pub const SCALE_CTRL_1: u16 = 0x5601;
pub const SCALE_CTRL_2: u16 = 0x5602;
pub const SCALE_CTRL_3: u16 = 0x5603;
pub const SCALE_CTRL_4: u16 = 0x5604;
pub const SCALE_CTRL_5: u16 = 0x5605;
pub const SCALE_CTRL_6: u16 = 0x5606;

pub const TEST_COLOR_BAR: u8 = 0xC0;
pub const AEC_PK_MANUAL_AGC_MANUALEN: u8 = 0x02;
pub const AEC_PK_MANUAL_AEC_MANUALEN: u8 = 0x01;
pub const TIMING_TC_REG20_VFLIP: u8 = 0x06;
pub const TIMING_TC_REG21_HMIRROR: u8 = 0x06;

pub const REG_DLY: u16 = 0xFFFF;

/// Aspect ratio window table matching `ratio_table[]` in `ov3660_settings.h`.
pub const RATIO_TABLE: [RatioSettings; 9] = [
    RatioSettings {
        max_width: 2048,
        max_height: 1536,
        start_x: 0,
        start_y: 0,
        end_x: 2079,
        end_y: 1547,
        offset_x: 16,
        offset_y: 6,
        total_x: 2300,
        total_y: 1564,
    }, // 4x3
    RatioSettings {
        max_width: 1920,
        max_height: 1280,
        start_x: 64,
        start_y: 128,
        end_x: 2015,
        end_y: 1419,
        offset_x: 16,
        offset_y: 6,
        total_x: 2172,
        total_y: 1436,
    }, // 3x2
    RatioSettings {
        max_width: 2048,
        max_height: 1280,
        start_x: 0,
        start_y: 128,
        end_x: 2079,
        end_y: 1419,
        offset_x: 16,
        offset_y: 6,
        total_x: 2300,
        total_y: 1436,
    }, // 16x10
    RatioSettings {
        max_width: 1920,
        max_height: 1152,
        start_x: 64,
        start_y: 192,
        end_x: 2015,
        end_y: 1355,
        offset_x: 16,
        offset_y: 6,
        total_x: 2172,
        total_y: 1372,
    }, // 5x3
    RatioSettings {
        max_width: 1920,
        max_height: 1080,
        start_x: 64,
        start_y: 242,
        end_x: 2015,
        end_y: 1333,
        offset_x: 16,
        offset_y: 6,
        total_x: 2172,
        total_y: 1322,
    }, // 16x9
    RatioSettings {
        max_width: 2048,
        max_height: 880,
        start_x: 0,
        start_y: 328,
        end_x: 2079,
        end_y: 1219,
        offset_x: 16,
        offset_y: 6,
        total_x: 2300,
        total_y: 1236,
    }, // 21x9
    RatioSettings {
        max_width: 1920,
        max_height: 1536,
        start_x: 64,
        start_y: 0,
        end_x: 2015,
        end_y: 1547,
        offset_x: 16,
        offset_y: 6,
        total_x: 2172,
        total_y: 1564,
    }, // 5x4
    RatioSettings {
        max_width: 1536,
        max_height: 1536,
        start_x: 256,
        start_y: 0,
        end_x: 1823,
        end_y: 1547,
        offset_x: 16,
        offset_y: 6,
        total_x: 2044,
        total_y: 1564,
    }, // 1x1
    RatioSettings {
        max_width: 864,
        max_height: 1536,
        start_x: 592,
        start_y: 0,
        end_x: 1487,
        end_y: 1547,
        offset_x: 16,
        offset_y: 6,
        total_x: 2044,
        total_y: 1564,
    }, // 9x16
];

pub const SENSOR_DEFAULT_REGS: [Reg16; 198] = [
    Reg16::new(SYSTEM_CTROL0, 0x82),
    Reg16::new(REG_DLY, 10),
    Reg16::new(0x3103, 0x13),
    Reg16::new(SYSTEM_CTROL0, 0x42),
    Reg16::new(0x3017, 0xff),
    Reg16::new(0x3018, 0xff),
    Reg16::new(DRIVE_CAPABILITY, 0xc3),
    Reg16::new(CLOCK_POL_CONTROL, 0x21),
    Reg16::new(0x3611, 0x01),
    Reg16::new(0x3612, 0x2d),
    Reg16::new(0x3032, 0x00),
    Reg16::new(0x3614, 0x80),
    Reg16::new(0x3618, 0x00),
    Reg16::new(0x3619, 0x75),
    Reg16::new(0x3622, 0x80),
    Reg16::new(0x3623, 0x00),
    Reg16::new(0x3624, 0x03),
    Reg16::new(0x3630, 0x52),
    Reg16::new(0x3632, 0x07),
    Reg16::new(0x3633, 0xd2),
    Reg16::new(0x3704, 0x80),
    Reg16::new(0x3708, 0x66),
    Reg16::new(0x3709, 0x12),
    Reg16::new(0x370b, 0x12),
    Reg16::new(0x3717, 0x00),
    Reg16::new(0x371b, 0x60),
    Reg16::new(0x371c, 0x00),
    Reg16::new(0x3901, 0x13),
    Reg16::new(0x3600, 0x08),
    Reg16::new(0x3620, 0x43),
    Reg16::new(0x3702, 0x20),
    Reg16::new(0x3739, 0x48),
    Reg16::new(0x3730, 0x20),
    Reg16::new(0x370c, 0x0c),
    Reg16::new(0x3a18, 0x00),
    Reg16::new(0x3a19, 0xf8),
    Reg16::new(0x3000, 0x10),
    Reg16::new(0x3004, 0xef),
    Reg16::new(0x6700, 0x05),
    Reg16::new(0x6701, 0x19),
    Reg16::new(0x6702, 0xfd),
    Reg16::new(0x6703, 0xd1),
    Reg16::new(0x6704, 0xff),
    Reg16::new(0x6705, 0xff),
    Reg16::new(0x3c01, 0x80),
    Reg16::new(0x3c00, 0x04),
    Reg16::new(0x3a08, 0x00),
    Reg16::new(0x3a09, 0x62),
    Reg16::new(0x3a0e, 0x08),
    Reg16::new(0x3a0a, 0x00),
    Reg16::new(0x3a0b, 0x52),
    Reg16::new(0x3a0d, 0x09),
    Reg16::new(0x3a00, 0x3a),
    Reg16::new(0x3a14, 0x09),
    Reg16::new(0x3a15, 0x30),
    Reg16::new(0x3a02, 0x09),
    Reg16::new(0x3a03, 0x30),
    Reg16::new(COMPRESSION_CTRL0E, 0x08),
    Reg16::new(0x4520, 0x0b),
    Reg16::new(0x460b, 0x37),
    Reg16::new(0x4713, 0x02),
    Reg16::new(0x471c, 0xd0),
    Reg16::new(0x5086, 0x00),
    Reg16::new(0x5002, 0x00),
    Reg16::new(0x501f, 0x00),
    Reg16::new(SYSTEM_CTROL0, 0x02),
    Reg16::new(0x5180, 0xff),
    Reg16::new(0x5181, 0xf2),
    Reg16::new(0x5182, 0x00),
    Reg16::new(0x5183, 0x14),
    Reg16::new(0x5184, 0x25),
    Reg16::new(0x5185, 0x24),
    Reg16::new(0x5186, 0x16),
    Reg16::new(0x5187, 0x16),
    Reg16::new(0x5188, 0x16),
    Reg16::new(0x5189, 0x68),
    Reg16::new(0x518a, 0x60),
    Reg16::new(0x518b, 0xe0),
    Reg16::new(0x518c, 0xb2),
    Reg16::new(0x518d, 0x42),
    Reg16::new(0x518e, 0x35),
    Reg16::new(0x518f, 0x56),
    Reg16::new(0x5190, 0x56),
    Reg16::new(0x5191, 0xf8),
    Reg16::new(0x5192, 0x04),
    Reg16::new(0x5193, 0x70),
    Reg16::new(0x5194, 0xf0),
    Reg16::new(0x5195, 0xf0),
    Reg16::new(0x5196, 0x03),
    Reg16::new(0x5197, 0x01),
    Reg16::new(0x5198, 0x04),
    Reg16::new(0x5199, 0x12),
    Reg16::new(0x519a, 0x04),
    Reg16::new(0x519b, 0x00),
    Reg16::new(0x519c, 0x06),
    Reg16::new(0x519d, 0x82),
    Reg16::new(0x519e, 0x38),
    Reg16::new(0x5381, 0x1d),
    Reg16::new(0x5382, 0x60),
    Reg16::new(0x5383, 0x03),
    Reg16::new(0x5384, 0x0c),
    Reg16::new(0x5385, 0x78),
    Reg16::new(0x5386, 0x84),
    Reg16::new(0x5387, 0x7d),
    Reg16::new(0x5388, 0x6b),
    Reg16::new(0x5389, 0x12),
    Reg16::new(0x538a, 0x01),
    Reg16::new(0x538b, 0x98),
    Reg16::new(0x5480, 0x01),
    Reg16::new(0x5000, 0xa7),
    Reg16::new(0x5800, 0x0C),
    Reg16::new(0x5801, 0x09),
    Reg16::new(0x5802, 0x0C),
    Reg16::new(0x5803, 0x0C),
    Reg16::new(0x5804, 0x0D),
    Reg16::new(0x5805, 0x17),
    Reg16::new(0x5806, 0x06),
    Reg16::new(0x5807, 0x05),
    Reg16::new(0x5808, 0x04),
    Reg16::new(0x5809, 0x06),
    Reg16::new(0x580a, 0x09),
    Reg16::new(0x580b, 0x0E),
    Reg16::new(0x580c, 0x05),
    Reg16::new(0x580d, 0x01),
    Reg16::new(0x580e, 0x01),
    Reg16::new(0x580f, 0x01),
    Reg16::new(0x5810, 0x05),
    Reg16::new(0x5811, 0x0D),
    Reg16::new(0x5812, 0x05),
    Reg16::new(0x5813, 0x01),
    Reg16::new(0x5814, 0x01),
    Reg16::new(0x5815, 0x01),
    Reg16::new(0x5816, 0x05),
    Reg16::new(0x5817, 0x0D),
    Reg16::new(0x5818, 0x08),
    Reg16::new(0x5819, 0x06),
    Reg16::new(0x581a, 0x05),
    Reg16::new(0x581b, 0x07),
    Reg16::new(0x581c, 0x0B),
    Reg16::new(0x581d, 0x0D),
    Reg16::new(0x581e, 0x12),
    Reg16::new(0x581f, 0x0D),
    Reg16::new(0x5820, 0x0E),
    Reg16::new(0x5821, 0x10),
    Reg16::new(0x5822, 0x10),
    Reg16::new(0x5823, 0x1E),
    Reg16::new(0x5824, 0x53),
    Reg16::new(0x5825, 0x15),
    Reg16::new(0x5826, 0x05),
    Reg16::new(0x5827, 0x14),
    Reg16::new(0x5828, 0x54),
    Reg16::new(0x5829, 0x25),
    Reg16::new(0x582a, 0x33),
    Reg16::new(0x582b, 0x33),
    Reg16::new(0x582c, 0x34),
    Reg16::new(0x582d, 0x16),
    Reg16::new(0x582e, 0x24),
    Reg16::new(0x582f, 0x41),
    Reg16::new(0x5830, 0x50),
    Reg16::new(0x5831, 0x42),
    Reg16::new(0x5832, 0x15),
    Reg16::new(0x5833, 0x25),
    Reg16::new(0x5834, 0x34),
    Reg16::new(0x5835, 0x33),
    Reg16::new(0x5836, 0x24),
    Reg16::new(0x5837, 0x26),
    Reg16::new(0x5838, 0x54),
    Reg16::new(0x5839, 0x25),
    Reg16::new(0x583a, 0x15),
    Reg16::new(0x583b, 0x25),
    Reg16::new(0x583c, 0x53),
    Reg16::new(0x583d, 0xCF),
    Reg16::new(0x3a0f, 0x30),
    Reg16::new(0x3a10, 0x28),
    Reg16::new(0x3a1b, 0x30),
    Reg16::new(0x3a1e, 0x28),
    Reg16::new(0x3a11, 0x60),
    Reg16::new(0x3a1f, 0x14),
    Reg16::new(0x5302, 0x28),
    Reg16::new(0x5303, 0x20),
    Reg16::new(0x5306, 0x1c),
    Reg16::new(0x5307, 0x28),
    Reg16::new(0x4002, 0xc5),
    Reg16::new(0x4003, 0x81),
    Reg16::new(0x4005, 0x12),
    Reg16::new(0x5688, 0x11),
    Reg16::new(0x5689, 0x11),
    Reg16::new(0x568a, 0x11),
    Reg16::new(0x568b, 0x11),
    Reg16::new(0x568c, 0x11),
    Reg16::new(0x568d, 0x11),
    Reg16::new(0x568e, 0x11),
    Reg16::new(0x568f, 0x11),
    Reg16::new(0x5580, 0x06),
    Reg16::new(0x5588, 0x00),
    Reg16::new(0x5583, 0x40),
    Reg16::new(0x5584, 0x2c),
    Reg16::new(ISP_CONTROL_01, 0x83),
];

pub const SENSOR_FMT_JPEG: [Reg16; 5] = [
    Reg16::new(FORMAT_CTRL, 0x00),
    Reg16::new(FORMAT_CTRL00, 0x30),
    Reg16::new(0x3002, 0x00),
    Reg16::new(0x3006, 0xff),
    Reg16::new(0x471c, 0x50),
];

pub const SENSOR_FMT_RAW: [Reg16; 1] = [Reg16::new(FORMAT_CTRL00, 0x00)];

pub const SENSOR_FMT_GRAYSCALE: [Reg16; 2] = [
    Reg16::new(FORMAT_CTRL, 0x00),
    Reg16::new(FORMAT_CTRL00, 0x10),
];

pub const SENSOR_FMT_YUV422: [Reg16; 2] = [
    Reg16::new(FORMAT_CTRL, 0x00),
    Reg16::new(FORMAT_CTRL00, 0x30),
];

pub const SENSOR_FMT_RGB565: [Reg16; 2] = [
    Reg16::new(FORMAT_CTRL, 0x01),
    Reg16::new(FORMAT_CTRL00, 0x61),
];

pub const SENSOR_SATURATION_LEVELS: [[u8; 11]; 9] = [
    [0x1d, 0x60, 0x03, 0x07, 0x48, 0x4f, 0x4b, 0x40, 0x0b, 0x01, 0x98], // -4
    [0x1d, 0x60, 0x03, 0x08, 0x54, 0x5c, 0x58, 0x4b, 0x0d, 0x01, 0x98], // -3
    [0x1d, 0x60, 0x03, 0x0a, 0x60, 0x6a, 0x64, 0x56, 0x0e, 0x01, 0x98], // -2
    [0x1d, 0x60, 0x03, 0x0b, 0x6c, 0x77, 0x70, 0x60, 0x10, 0x01, 0x98], // -1
    [0x1d, 0x60, 0x03, 0x0c, 0x78, 0x84, 0x7d, 0x6b, 0x12, 0x01, 0x98], //  0
    [0x1d, 0x60, 0x03, 0x0d, 0x84, 0x91, 0x8a, 0x76, 0x14, 0x01, 0x98], // +1
    [0x1d, 0x60, 0x03, 0x0e, 0x90, 0x9e, 0x96, 0x80, 0x16, 0x01, 0x98], // +2
    [0x1d, 0x60, 0x03, 0x10, 0x9c, 0xac, 0xa2, 0x8b, 0x17, 0x01, 0x98], // +3
    [0x1d, 0x60, 0x03, 0x11, 0xa8, 0xb9, 0xaf, 0x96, 0x19, 0x01, 0x98], // +4
];

pub const SENSOR_SPECIAL_EFFECTS: [[u8; 4]; 7] = [
    [0x06, 0x40, 0x2c, 0x08], // 0: Normal
    [0x46, 0x40, 0x28, 0x08], // 1: Negative
    [0x1e, 0x80, 0x80, 0x08], // 2: Grayscale
    [0x1e, 0x80, 0xc0, 0x08], // 3: Red Tint
    [0x1e, 0x60, 0x60, 0x08], // 4: Green Tint
    [0x1e, 0xa0, 0x40, 0x08], // 5: Blue Tint
    [0x1e, 0x40, 0xa0, 0x08], // 6: Sepia
];

/// OV3660 camera sensor driver state.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Ov3660 {
    slv_addr: u8,
    id: SensorId,
    status: CameraStatus,
    pixformat: PixelFormat,
    xclk_freq_hz: u32,
}

impl Default for Ov3660 {
    fn default() -> Self {
        Self::new()
    }
}

impl Ov3660 {
    /// Default 7-bit SCCB slave address for OV3660 (`0x3C`).
    pub const SLV_ADDR: u8 = CameraSccbAddr::OV3660;
    /// Expected 16-bit PID (`0x3660`).
    pub const PID: u16 = CameraPid::OV3660.as_u16();

    /// Creates a new [`Ov3660`] driver instance with default SCCB address (`0x3C`).
    pub const fn new() -> Self {
        Self::with_addr(Self::SLV_ADDR)
    }

    /// Creates a new [`Ov3660`] driver instance with a custom SCCB slave address.
    pub const fn with_addr(slv_addr: u8) -> Self {
        Self::from_id(
            slv_addr,
            SensorId {
                midh: 0,
                midl: 0,
                pid: Self::PID,
                ver: 0,
            },
        )
    }

    /// Creates a new [`Ov3660`] driver instance from a detected [`SensorId`].
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
        }
    }

    /// Probes and verifies the OV3660 sensor on `bus` at `slv_addr` (`esp32_camera_ov3660_detect`).
    pub fn detect<B: SccbBus>(bus: &mut B, slv_addr: u8) -> Result<SensorId, CameraError> {
        if slv_addr != Self::SLV_ADDR {
            return Err(CameraError::NotDetected);
        }
        let pid = bus.read_reg16_val16(slv_addr, 0x300A)?;
        if pid != Self::PID {
            return Err(CameraError::NotDetected);
        }
        Ok(SensorId {
            midh: 0,
            midl: 0,
            pid,
            ver: 0,
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

    /// Returns the [`CameraModel`] enum discriminant for this sensor.
    #[inline]
    pub const fn model(&self) -> CameraModel {
        CameraModel::Ov3660
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

    fn write_regs_with_delay<B: SccbBus>(
        &self,
        bus: &mut B,
        regs: &[Reg16],
    ) -> Result<(), CameraError> {
        for entry in regs {
            if entry.reg == REG_DLY {
                bus.delay_ms(entry.val as u32);
            } else {
                bus.write_reg16(self.slv_addr, entry.reg, entry.val)?;
            }
        }
        Ok(())
    }

    fn write_reg_bits<B: SccbBus>(
        &self,
        bus: &mut B,
        reg: u16,
        mask: u8,
        enable: bool,
    ) -> Result<(), CameraError> {
        bus.write_reg16_mask(self.slv_addr, reg, if enable { mask } else { 0 }, mask)
    }

    fn check_reg_mask<B: SccbBus>(
        &self,
        bus: &mut B,
        reg: u16,
        mask: u8,
    ) -> Result<bool, CameraError> {
        let v = bus.read_reg16(self.slv_addr, reg)?;
        Ok((v & mask) == mask)
    }

    fn write_addr_reg<B: SccbBus>(
        &self,
        bus: &mut B,
        reg: u16,
        x_value: u16,
        y_value: u16,
    ) -> Result<(), CameraError> {
        bus.write_reg16_val16(self.slv_addr, reg, x_value)?;
        bus.write_reg16_val16(self.slv_addr, reg.wrapping_add(2), y_value)
    }

    #[allow(clippy::too_many_arguments)]
    fn configure_pll<B: SccbBus>(
        &self,
        bus: &mut B,
        bypass: bool,
        multiplier: u8,
        sys_div: u8,
        pre_div: u8,
        root_2x: bool,
        seld5: u8,
        pclk_manual: bool,
        pclk_div: u8,
    ) -> Result<(), CameraError> {
        if multiplier > 31 || sys_div > 15 || pre_div > 3 || pclk_div > 31 || seld5 > 3 {
            return Err(CameraError::InvalidArg);
        }

        bus.write_reg16(self.slv_addr, SC_PLLS_CTRL0, if bypass { 0x80 } else { 0x00 })?;
        bus.write_reg16(self.slv_addr, SC_PLLS_CTRL1, multiplier & 0x1F)?;
        bus.write_reg16(self.slv_addr, SC_PLLS_CTRL2, 0x10 | (sys_div & 0x0F))?;
        bus.write_reg16(
            self.slv_addr,
            SC_PLLS_CTRL3,
            ((pre_div & 0x03) << 4) | (seld5 & 0x03) | if root_2x { 0x40 } else { 0x00 },
        )?;
        bus.write_reg16(self.slv_addr, PCLK_RATIO, pclk_div & 0x1F)?;
        bus.write_reg16(
            self.slv_addr,
            VFIFO_CTRL0C,
            if pclk_manual { 0x22 } else { 0x20 },
        )
    }

    fn set_image_options<B: SccbBus>(&self, bus: &mut B) -> Result<(), CameraError> {
        let mut reg20 = 0u8;
        let mut reg21 = 0u8;
        let mut reg4514_test = 0u8;

        if self.pixformat == PixelFormat::Jpeg {
            reg21 |= 0x20;
        }

        if self.status.binning {
            reg20 |= 0x01;
            reg21 |= 0x01;
            reg4514_test |= 4;
        } else {
            reg20 |= 0x40;
        }

        if self.status.vflip != 0 {
            reg20 |= 0x06;
            reg4514_test |= 1;
        }

        if self.status.hmirror != 0 {
            reg21 |= 0x06;
            reg4514_test |= 2;
        }

        let reg4514 = match reg4514_test {
            0 | 1 => 0x88,
            2 | 3 | 5 | 6 => 0xbb,
            _ => 0xaa,
        };

        bus.write_reg16(self.slv_addr, TIMING_TC_REG20, reg20)?;
        bus.write_reg16(self.slv_addr, TIMING_TC_REG21, reg21)?;
        bus.write_reg16(self.slv_addr, 0x4514, reg4514)?;

        if self.status.binning {
            bus.write_reg16(self.slv_addr, 0x4520, 0x0b)?;
            bus.write_reg16(self.slv_addr, X_INCREMENT, 0x31)?;
            bus.write_reg16(self.slv_addr, Y_INCREMENT, 0x31)
        } else {
            bus.write_reg16(self.slv_addr, 0x4520, 0xb0)?;
            bus.write_reg16(self.slv_addr, X_INCREMENT, 0x11)?;
            bus.write_reg16(self.slv_addr, Y_INCREMENT, 0x11)
        }
    }

    fn get_agc_gain<B: SccbBus>(&self, bus: &mut B) -> Result<u8, CameraError> {
        let ra = bus.read_reg16(self.slv_addr, 0x350A)?;
        let rb = bus.read_reg16(self.slv_addr, 0x350B)?;
        let mut res = ((rb & 0xF0) >> 4) | ((ra & 0x03) << 4);
        if (rb & 0x0F) != 0 {
            res = res.saturating_add(1);
        }
        Ok(res)
    }

    fn get_aec_value<B: SccbBus>(&self, bus: &mut B) -> Result<u16, CameraError> {
        let ra = bus.read_reg16(self.slv_addr, 0x3500)? as u16;
        let rb = bus.read_reg16(self.slv_addr, 0x3501)? as u16;
        let rc = bus.read_reg16(self.slv_addr, 0x3502)? as u16;
        Ok(((ra & 0x0F) << 12) | ((rb & 0xFF) << 4) | ((rc & 0xF0) >> 4))
    }

    fn get_denoise<B: SccbBus>(&self, bus: &mut B) -> Result<u8, CameraError> {
        if !self.check_reg_mask(bus, 0x5308, 0x10)? {
            return Ok(0);
        }
        let v = bus.read_reg16(self.slv_addr, 0x5306)?;
        Ok((v / 4).saturating_add(1))
    }
}

impl<B: SccbBus> CameraSensor<B> for Ov3660 {
    fn model(&self) -> CameraModel {
        CameraModel::Ov3660
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
        bus.write_reg16(self.slv_addr, SYSTEM_CTROL0, 0x82)?;
        bus.delay_ms(100);
        self.write_regs_with_delay(bus, &SENSOR_DEFAULT_REGS)?;
        self.set_ae_level(bus, 0)?;
        bus.delay_ms(100);
        Ok(())
    }

    fn init_status(&mut self, bus: &mut B) -> Result<(), CameraError> {
        self.status.brightness = 0;
        self.status.contrast = 0;
        self.status.saturation = 0;
        let reg5303 = bus.read_reg16(self.slv_addr, 0x5303)? as i16;
        self.status.sharpness = ((reg5303 / 8) - 3) as i8;
        self.status.denoise = self.get_denoise(bus)?;
        self.status.ae_level = 0;

        let reg_val = bus.read_reg16_val16(self.slv_addr, 0x3A18)? & 0x03FF;
        let mut level = 0u8;
        while level < (GainCeiling::Gain128x as u8) && (16u16 << (level + 1)) < reg_val {
            level += 1;
        }
        self.status.gainceiling = level;

        self.status.awb = u8::from(self.check_reg_mask(bus, ISP_CONTROL_01, 0x01)?);
        self.status.dcw = u8::from(!self.check_reg_mask(bus, 0x5183, 0x80)?);
        self.status.agc =
            u8::from(!self.check_reg_mask(bus, AEC_PK_MANUAL, AEC_PK_MANUAL_AGC_MANUALEN)?);
        self.status.aec =
            u8::from(!self.check_reg_mask(bus, AEC_PK_MANUAL, AEC_PK_MANUAL_AEC_MANUALEN)?);
        self.status.hmirror =
            u8::from(self.check_reg_mask(bus, TIMING_TC_REG21, TIMING_TC_REG21_HMIRROR)?);
        self.status.vflip =
            u8::from(self.check_reg_mask(bus, TIMING_TC_REG20, TIMING_TC_REG20_VFLIP)?);
        self.status.colorbar =
            u8::from(self.check_reg_mask(bus, PRE_ISP_TEST_SETTING_1, TEST_COLOR_BAR)?);
        self.status.bpc = u8::from(self.check_reg_mask(bus, 0x5000, 0x04)?);
        self.status.wpc = u8::from(self.check_reg_mask(bus, 0x5000, 0x02)?);
        self.status.raw_gma = u8::from(self.check_reg_mask(bus, 0x5000, 0x20)?);
        self.status.lenc = u8::from(self.check_reg_mask(bus, 0x5000, 0x80)?);
        self.status.quality = bus.read_reg16(self.slv_addr, COMPRESSION_CTRL07)? & 0x3F;
        self.status.special_effect = 0;
        self.status.wb_mode = 0;
        self.status.awb_gain = u8::from(self.check_reg_mask(bus, 0x3406, 0x01)?);
        self.status.agc_gain = self.get_agc_gain(bus)?;
        self.status.aec_value = self.get_aec_value(bus)?;
        self.status.aec2 = u8::from(self.check_reg_mask(bus, 0x3A00, 0x04)?);
        Ok(())
    }

    fn set_pixformat(&mut self, bus: &mut B, pixformat: PixelFormat) -> Result<(), CameraError> {
        let regs: &[Reg16] = match pixformat {
            PixelFormat::Yuv422 => &SENSOR_FMT_YUV422,
            PixelFormat::Grayscale => &SENSOR_FMT_GRAYSCALE,
            PixelFormat::Rgb565 | PixelFormat::Rgb888 => &SENSOR_FMT_RGB565,
            PixelFormat::Jpeg => &SENSOR_FMT_JPEG,
            PixelFormat::Raw => &SENSOR_FMT_RAW,
            _ => return Err(CameraError::NotSupported),
        };
        self.write_regs_with_delay(bus, regs)?;
        self.pixformat = pixformat;
        Ok(())
    }

    fn set_framesize(&mut self, bus: &mut B, framesize: FrameSize) -> Result<(), CameraError> {
        if (framesize as u8) > (FrameSize::Qxga as u8) {
            return Err(CameraError::InvalidArg);
        }
        let old_framesize = self.status.framesize;
        let old_binning = self.status.binning;
        let old_scale = self.status.scale;

        self.status.framesize = framesize;
        let res = framesize.resolution();
        let w = res.width;
        let h = res.height;
        let ratio: AspectRatio = res.aspect_ratio;
        let settings = RATIO_TABLE[ratio as usize];

        self.status.binning = w <= (settings.max_width / 2) && h <= (settings.max_height / 2);
        self.status.scale = !((w == settings.max_width && h == settings.max_height)
            || (w == (settings.max_width / 2) && h == (settings.max_height / 2)));

        let step = (|| -> Result<(), CameraError> {
            self.write_addr_reg(bus, X_ADDR_ST_H, settings.start_x, settings.start_y)?;
            self.write_addr_reg(bus, X_ADDR_END_H, settings.end_x, settings.end_y)?;
            self.write_addr_reg(bus, X_OUTPUT_SIZE_H, w, h)?;

            if self.status.binning {
                self.write_addr_reg(
                    bus,
                    X_TOTAL_SIZE_H,
                    settings.total_x,
                    (settings.total_y / 2) + 1,
                )?;
                self.write_addr_reg(bus, X_OFFSET_H, 8, 2)?;
            } else {
                self.write_addr_reg(bus, X_TOTAL_SIZE_H, settings.total_x, settings.total_y)?;
                self.write_addr_reg(bus, X_OFFSET_H, 16, 6)?;
            }

            self.write_reg_bits(bus, ISP_CONTROL_01, 0x20, self.status.scale)?;
            self.set_image_options(bus)?;

            if self.pixformat == PixelFormat::Jpeg {
                if framesize == FrameSize::Qxga || self.xclk_freq_hz == 16_000_000 {
                    self.configure_pll(bus, false, 24, 1, 3, false, 0, true, 8)?;
                } else {
                    self.configure_pll(bus, false, 30, 1, 3, false, 0, true, 10)?;
                }
            } else if (framesize as u8) > (FrameSize::Hvga as u8) {
                self.configure_pll(bus, false, 4, 1, 0, false, 2, true, 2)?;
            } else if (framesize as u8) >= (FrameSize::Qvga as u8) {
                self.configure_pll(bus, false, 8, 1, 0, false, 2, true, 4)?;
            } else {
                self.configure_pll(bus, false, 8, 1, 0, false, 0, true, 8)?;
            }
            Ok(())
        })();

        if let Err(err) = step {
            self.status.framesize = old_framesize;
            self.status.binning = old_binning;
            self.status.scale = old_scale;
            return Err(err);
        }
        Ok(())
    }

    fn set_contrast(&mut self, bus: &mut B, level: i32) -> Result<(), CameraError> {
        if !(-3..=3).contains(&level) {
            return Err(CameraError::InvalidArg);
        }
        bus.write_reg16(self.slv_addr, 0x5586, ((level + 4) as u8) << 3)?;
        self.status.contrast = level as i8;
        Ok(())
    }

    fn set_brightness(&mut self, bus: &mut B, level: i32) -> Result<(), CameraError> {
        if !(-3..=3).contains(&level) {
            return Err(CameraError::InvalidArg);
        }
        let (value, negative) = match level {
            3 => (0x30u8, false),
            2 => (0x20u8, false),
            1 => (0x10u8, false),
            -1 => (0x10u8, true),
            -2 => (0x20u8, true),
            -3 => (0x30u8, true),
            _ => (0x00u8, false),
        };
        bus.write_reg16(self.slv_addr, 0x5587, value)?;
        self.write_reg_bits(bus, 0x5588, 0x08, negative)?;
        self.status.brightness = level as i8;
        Ok(())
    }

    fn set_saturation(&mut self, bus: &mut B, level: i32) -> Result<(), CameraError> {
        if !(-4..=4).contains(&level) {
            return Err(CameraError::InvalidArg);
        }
        let regs = &SENSOR_SATURATION_LEVELS[(level + 4) as usize];
        for (i, &val) in regs.iter().enumerate() {
            bus.write_reg16(self.slv_addr, 0x5381 + (i as u16), val)?;
        }
        self.status.saturation = level as i8;
        Ok(())
    }

    fn set_sharpness(&mut self, bus: &mut B, level: i32) -> Result<(), CameraError> {
        if !(-3..=3).contains(&level) {
            return Err(CameraError::InvalidArg);
        }
        let mt_offset_2 = ((level + 3) as u8) * 8;
        let mt_offset_1 = mt_offset_2 + 1;

        self.write_reg_bits(bus, 0x5308, 0x40, false)?;
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

    fn set_denoise(&mut self, bus: &mut B, level: i32) -> Result<(), CameraError> {
        if !(0..=8).contains(&level) {
            return Err(CameraError::InvalidArg);
        }
        self.write_reg_bits(bus, 0x5308, 0x10, level > 0)?;
        if level > 0 {
            bus.write_reg16(self.slv_addr, 0x5306, ((level - 1) as u8) * 4)?;
        }
        self.status.denoise = level as u8;
        Ok(())
    }

    fn set_gainceiling(
        &mut self,
        bus: &mut B,
        gainceiling: GainCeiling,
    ) -> Result<(), CameraError> {
        let level = gainceiling as u8;
        let mut reg_val = 16u16 << (level + 1);
        if reg_val > 0x03FF {
            reg_val = 0x03FF;
        }
        bus.write_reg16(self.slv_addr, 0x3A18, ((reg_val >> 8) & 0x03) as u8)?;
        bus.write_reg16(self.slv_addr, 0x3A19, (reg_val & 0xFF) as u8)?;
        self.status.gainceiling = level;
        Ok(())
    }

    fn set_quality(&mut self, bus: &mut B, quality: i32) -> Result<(), CameraError> {
        if !(0..=63).contains(&quality) {
            return Err(CameraError::InvalidArg);
        }
        let qs = (quality as u8) & 0x3F;
        bus.write_reg16(self.slv_addr, COMPRESSION_CTRL07, qs)?;
        self.status.quality = qs;
        Ok(())
    }

    fn set_colorbar(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.write_reg_bits(bus, PRE_ISP_TEST_SETTING_1, TEST_COLOR_BAR, enable)?;
        self.status.colorbar = u8::from(enable);
        Ok(())
    }

    fn set_whitebal(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.write_reg_bits(bus, ISP_CONTROL_01, 0x01, enable)?;
        self.status.awb = u8::from(enable);
        Ok(())
    }

    fn set_gain_ctrl(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.write_reg_bits(bus, AEC_PK_MANUAL, AEC_PK_MANUAL_AGC_MANUALEN, !enable)?;
        self.status.agc = u8::from(enable);
        Ok(())
    }

    fn set_exposure_ctrl(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.write_reg_bits(bus, AEC_PK_MANUAL, AEC_PK_MANUAL_AEC_MANUALEN, !enable)?;
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

    fn set_aec2(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.write_reg_bits(bus, 0x3A00, 0x04, enable)?;
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
        if !(0..=64).contains(&gain) {
            return Err(CameraError::InvalidArg);
        }
        let gainv = ((gain as u16) << 4).saturating_sub(1);
        bus.write_reg16(self.slv_addr, 0x350A, (gainv >> 8) as u8)?;
        bus.write_reg16(self.slv_addr, 0x350B, (gainv & 0xFF) as u8)?;
        self.status.agc_gain = gain as u8;
        Ok(())
    }

    fn set_aec_value(&mut self, bus: &mut B, value: i32) -> Result<(), CameraError> {
        if !(0..=0xFFFF).contains(&value) {
            return Err(CameraError::InvalidArg);
        }
        let max_val = bus.read_reg16_val16(self.slv_addr, Y_TOTAL_SIZE_H)?;
        let mut val = value as u16;
        if max_val > 0 && val > max_val {
            val = max_val;
        }
        bus.write_reg16(self.slv_addr, 0x3500, ((val >> 12) & 0x0F) as u8)?;
        bus.write_reg16(self.slv_addr, 0x3501, ((val >> 4) & 0xFF) as u8)?;
        bus.write_reg16(self.slv_addr, 0x3502, ((val << 4) & 0xF0) as u8)?;
        self.status.aec_value = val;
        Ok(())
    }

    fn set_special_effect(&mut self, bus: &mut B, effect: i32) -> Result<(), CameraError> {
        if !(0..=6).contains(&effect) {
            return Err(CameraError::InvalidArg);
        }
        let regs = &SENSOR_SPECIAL_EFFECTS[effect as usize];
        bus.write_reg16(self.slv_addr, 0x5580, regs[0])?;
        bus.write_reg16(self.slv_addr, 0x5583, regs[1])?;
        bus.write_reg16(self.slv_addr, 0x5584, regs[2])?;
        bus.write_reg16(self.slv_addr, 0x5003, regs[3])?;
        self.status.special_effect = effect as u8;
        Ok(())
    }

    fn set_wb_mode(&mut self, bus: &mut B, mode: i32) -> Result<(), CameraError> {
        if !(0..=4).contains(&mode) {
            return Err(CameraError::InvalidArg);
        }
        bus.write_reg16(self.slv_addr, 0x3406, u8::from(mode != 0))?;
        match mode {
            1 => {
                bus.write_reg16_val16(self.slv_addr, 0x3400, 0x05E0)?;
                bus.write_reg16_val16(self.slv_addr, 0x3402, 0x0410)?;
                bus.write_reg16_val16(self.slv_addr, 0x3404, 0x0540)?;
            }
            2 => {
                bus.write_reg16_val16(self.slv_addr, 0x3400, 0x0650)?;
                bus.write_reg16_val16(self.slv_addr, 0x3402, 0x0410)?;
                bus.write_reg16_val16(self.slv_addr, 0x3404, 0x04F0)?;
            }
            3 => {
                bus.write_reg16_val16(self.slv_addr, 0x3400, 0x0520)?;
                bus.write_reg16_val16(self.slv_addr, 0x3402, 0x0410)?;
                bus.write_reg16_val16(self.slv_addr, 0x3404, 0x0660)?;
            }
            4 => {
                bus.write_reg16_val16(self.slv_addr, 0x3400, 0x0420)?;
                bus.write_reg16_val16(self.slv_addr, 0x3402, 0x03F0)?;
                bus.write_reg16_val16(self.slv_addr, 0x3404, 0x0710)?;
            }
            _ => {}
        }
        self.status.wb_mode = mode as u8;
        Ok(())
    }

    fn set_ae_level(&mut self, bus: &mut B, level: i32) -> Result<(), CameraError> {
        if !(-5..=5).contains(&level) {
            return Err(CameraError::InvalidArg);
        }
        let target_level = ((level + 5) * 10) + 5;
        let level_low = (target_level * 23) / 25;
        let level_high = (target_level * 27) / 25;
        let fast_low = level_low >> 1;
        let mut fast_high = level_high << 1;
        if fast_high > 255 {
            fast_high = 255;
        }

        bus.write_reg16(self.slv_addr, 0x3A0F, level_high as u8)?;
        bus.write_reg16(self.slv_addr, 0x3A10, level_low as u8)?;
        bus.write_reg16(self.slv_addr, 0x3A1B, level_high as u8)?;
        bus.write_reg16(self.slv_addr, 0x3A1E, level_low as u8)?;
        bus.write_reg16(self.slv_addr, 0x3A11, fast_high as u8)?;
        bus.write_reg16(self.slv_addr, 0x3A1F, fast_low as u8)?;

        self.status.ae_level = level as i8;
        Ok(())
    }

    fn set_dcw(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.write_reg_bits(bus, 0x5183, 0x80, !enable)?;
        self.status.dcw = u8::from(enable);
        Ok(())
    }

    fn set_bpc(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.write_reg_bits(bus, 0x5000, 0x04, enable)?;
        self.status.bpc = u8::from(enable);
        Ok(())
    }

    fn set_wpc(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.write_reg_bits(bus, 0x5000, 0x02, enable)?;
        self.status.wpc = u8::from(enable);
        Ok(())
    }

    fn set_raw_gma(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.write_reg_bits(bus, 0x5000, 0x20, enable)?;
        self.status.raw_gma = u8::from(enable);
        Ok(())
    }

    fn set_lenc(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.write_reg_bits(bus, 0x5000, 0x80, enable)?;
        self.status.lenc = u8::from(enable);
        Ok(())
    }

    fn get_reg(&mut self, bus: &mut B, reg: i32, mask: i32) -> Result<i32, CameraError> {
        if !(0..=0xFFFF).contains(&reg) || mask < 0 {
            return Err(CameraError::InvalidArg);
        }
        let addr = reg as u16;
        let mut ret = if mask > 0xFF {
            let v16 = bus.read_reg16_val16(self.slv_addr, addr)? as i32;
            if mask > 0xFFFF {
                let v_low = bus.read_reg16(self.slv_addr, addr.wrapping_add(2))? as i32;
                (v16 << 8) | v_low
            } else {
                v16
            }
        } else {
            bus.read_reg16(self.slv_addr, addr)? as i32
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
        let addr = reg as u16;
        let current = if mask > 0xFF {
            let v16 = bus.read_reg16_val16(self.slv_addr, addr)? as i32;
            if mask > 0xFFFF {
                let v_low = bus.read_reg16(self.slv_addr, addr.wrapping_add(2))? as i32;
                (v16 << 8) | v_low
            } else {
                v16
            }
        } else {
            bus.read_reg16(self.slv_addr, addr)? as i32
        };
        let next = (current & !mask) | (value & mask);
        if mask > 0xFFFF {
            bus.write_reg16_val16(self.slv_addr, addr, ((next >> 8) & 0xFFFF) as u16)?;
            bus.write_reg16(self.slv_addr, addr.wrapping_add(2), (next & 0xFF) as u8)
        } else if mask > 0xFF {
            bus.write_reg16_val16(self.slv_addr, addr, (next & 0xFFFF) as u16)
        } else {
            bus.write_reg16(self.slv_addr, addr, (next & 0xFF) as u8)
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
        if start_x < 0
            || start_y < 0
            || end_x < 0
            || end_y < 0
            || offset_x < 0
            || offset_y < 0
            || total_x <= 0
            || total_y <= 0
            || output_x <= 0
            || output_y <= 0
        {
            return Err(CameraError::InvalidArg);
        }
        self.write_addr_reg(bus, X_ADDR_ST_H, start_x as u16, start_y as u16)?;
        self.write_addr_reg(bus, X_ADDR_END_H, end_x as u16, end_y as u16)?;
        self.write_addr_reg(bus, X_OFFSET_H, offset_x as u16, offset_y as u16)?;
        self.write_addr_reg(bus, X_TOTAL_SIZE_H, total_x as u16, total_y as u16)?;
        self.write_addr_reg(bus, X_OUTPUT_SIZE_H, output_x as u16, output_y as u16)?;
        self.write_reg_bits(bus, ISP_CONTROL_01, 0x20, scale)?;
        self.status.scale = scale;
        self.status.binning = binning;
        self.set_image_options(bus)
    }

    fn set_pll(
        &mut self,
        bus: &mut B,
        bypass: i32,
        mul: i32,
        sys: i32,
        root: i32,
        pre: i32,
        seld5: i32,
        pclken: i32,
        pclk: i32,
    ) -> Result<(), CameraError> {
        let mul = u8::try_from(mul).map_err(|_| CameraError::InvalidArg)?;
        let sys = u8::try_from(sys).map_err(|_| CameraError::InvalidArg)?;
        let pre = u8::try_from(pre).map_err(|_| CameraError::InvalidArg)?;
        let seld5 = u8::try_from(seld5).map_err(|_| CameraError::InvalidArg)?;
        let pclk = u8::try_from(pclk).map_err(|_| CameraError::InvalidArg)?;
        self.configure_pll(
            bus,
            bypass > 0,
            mul,
            sys,
            pre,
            root > 0,
            seld5,
            pclken > 0,
            pclk,
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
