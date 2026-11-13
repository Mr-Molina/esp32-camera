//! Safe `#![no_std]` OV3640 (3MP QXGA JPEG/RGB565/YUV422/RAW) camera
//! sensor driver.
//!
//! Ported from `sensors/ov3640.c`, `sensors/private_include/ov3640_regs.h`, and
//! `sensors/private_include/ov3640_settings.h`.
//!
//! # Unverified register map (SENS-04)
//!
//! This driver's window, PLL, format and JPEG-quality registers are OV3660
//! addresses, not OV3640 ones: the window block sits at 0x38xx, the PLL at
//! 0x303A..0x303D and JPEG quality at 0x4407, and [`RATIO_TABLE`],
//! [`SENSOR_FMT_JPEG`] and the other format tables are aliased directly from
//! [`crate::ov3660`]. An OV3640 is an earlier part and does not have that
//! layout, so programming it through those registers would either do nothing
//! or corrupt unrelated state.
//!
//! Until a register map from an OV3640 reference is ported and checked on
//! hardware, the setters that depend on those blocks refuse with
//! [`CameraError::NotSupported`] rather than write another sensor's map into
//! a real part: [`CameraSensor::set_framesize`], [`CameraSensor::set_pixformat`],
//! [`CameraSensor::set_quality`], [`CameraSensor::set_res_raw`] and
//! [`CameraSensor::set_pll`].
//!
//! The remaining methods, including [`CameraSensor::reset`], still write the
//! unverified default table, so this driver has **not** been validated against
//! real OV3640 hardware. Work order WO-PERF-2026-11, §7 question 6 asks the
//! repository owner whether any OV3640 is in use; if none is, the whole driver
//! should be marked unsupported instead.

use crate::{
    CameraError, CameraModel, CameraPid, CameraSccbAddr, CameraSensor, CameraStatus, FrameSize,
    GainCeiling, PixelFormat, RatioSettings, Reg16, SccbBus, SensorId,
};

pub const SYSTEM_CTROL0: u16 = 0x3008;
pub const PWCOM1: u16 = 0x30A9;

pub const OV3640_BANK_SEL: u16 = 0x00FF;
pub const OV3640_BANK_DEFAULT: u8 = 0x00;
pub const OV3640_BANK_JPEG_MCU: u8 = 0x01;
pub const OV3640_MCU_EIS_CTRL: u16 = 0x00D7;
pub const OV3640_MCU_SLEEP: u16 = 0x00D2;
pub const OV3640_MCU_IQ_CTRL: u16 = 0x00D1;
pub const OV3640_MCU_FIFO_CTRL: u16 = 0x001E;

pub const OV3640_SDE_CTRL: u16 = 0x3302;
pub const OV3640_DPC_CTRL: u16 = 0x3301;
pub const OV3640_SIMPLE_AWB: u16 = 0x3308;
pub const OV3640_AWB_CTRL: u16 = 0x332B;
pub const OV3640_SHARPNESS: u16 = 0x332D;
pub const OV3640_SHARPNESS_AUTO: u16 = 0x332F;
pub const OV3640_AEC_ALGO: u16 = 0x3047;
pub const OV3640_AEC_18: u16 = 0x3018;
pub const OV3640_AEC_19: u16 = 0x3019;
pub const OV3640_AEC_1A: u16 = 0x301A;
pub const OV3640_AWB_R_GAIN: u16 = 0x33A7;
pub const OV3640_AWB_G_GAIN: u16 = 0x33A8;
pub const OV3640_AWB_B_GAIN: u16 = 0x33A9;
pub const OV3640_SDE_ENABLE: u16 = 0x3355;
pub const OV3640_SDE_OFFSET: u16 = 0x3354;
pub const OV3640_BRIGHTNESS: u16 = 0x335E;
pub const OV3640_CONTRAST_Y1: u16 = 0x335C;
pub const OV3640_CONTRAST_Y2: u16 = 0x335D;
pub const OV3640_SATURATION_C: u16 = 0x3358;
pub const OV3640_SATURATION_M: u16 = 0x3359;
pub const OV3640_EFFECT_C1: u16 = 0x335A;
pub const OV3640_EFFECT_C2: u16 = 0x335B;

pub const FORMAT_CTRL: u16 = 0x501F;
pub const FORMAT_CTRL00: u16 = 0x4300;
pub const PRE_ISP_TEST_SETTING_1: u16 = 0x503D;
pub const AEC_PK_MANUAL: u16 = 0x3503;

pub const OV3640_TIMING_CTRL: u16 = 0x307C;
pub const OV3640_ARRAY_CTRL: u16 = 0x3090;
pub const OV3640_ARRAY_VSTART: u16 = 0x3023;

pub const TIMING_TC_REG20: u16 = 0x3820;
pub const TIMING_TC_REG21: u16 = 0x3821;
pub const CLOCK_POL_CONTROL: u16 = 0x4740;
pub const DRIVE_CAPABILITY: u16 = 0x302C;

pub const X_ADDR_ST_H: u16 = 0x3800;
pub const X_ADDR_END_H: u16 = 0x3804;
pub const X_OUTPUT_SIZE_H: u16 = 0x3808;
pub const X_TOTAL_SIZE_H: u16 = 0x380C;
pub const Y_TOTAL_SIZE_H: u16 = 0x380E;
pub const X_OFFSET_H: u16 = 0x3810;
pub const X_INCREMENT: u16 = 0x3814;
pub const Y_INCREMENT: u16 = 0x3815;
pub const ISP_CONTROL_01: u16 = 0x5001;
pub const PCLK_RATIO: u16 = 0x3824;
pub const VFIFO_CTRL0C: u16 = 0x460C;
pub const SC_PLLS_CTRL0: u16 = 0x303A;
pub const SC_PLLS_CTRL1: u16 = 0x303B;
pub const SC_PLLS_CTRL2: u16 = 0x303C;
pub const SC_PLLS_CTRL3: u16 = 0x303D;
pub const COMPRESSION_CTRL07: u16 = 0x4407;

pub const TEST_COLOR_BAR: u8 = 0xC0;
pub const AEC_PK_MANUAL_AGC_MANUALEN: u8 = 0x02;
pub const AEC_PK_MANUAL_AEC_MANUALEN: u8 = 0x01;

pub const REG_DLY: u16 = 0xFFFF;

/// Aspect ratio window table matching `ratio_table[]` in `ov3640_settings.h`.
pub const RATIO_TABLE: [RatioSettings; 9] = crate::ov3660::RATIO_TABLE;

/// SENS-22: `static`, not `const`. A `const` array of this size is a value
/// that each use site may materialise its own copy of; a `static` is one
/// read-only allocation. (The slice-typed tables elsewhere in this crate
/// are `const &[_]`, which already points at a single promoted static.)
pub static SENSOR_DEFAULT_REGS: [Reg16; 113] = [
    Reg16::new(SYSTEM_CTROL0, 0x82),
    Reg16::new(REG_DLY, 10),
    Reg16::new(0x3012, 0x80),
    Reg16::new(0x304d, 0x45),
    Reg16::new(0x30a7, 0x5e),
    Reg16::new(0x3087, 0x16),
    Reg16::new(0x309c, 0x1a),
    Reg16::new(0x30a2, 0xe4),
    Reg16::new(0x30aa, 0x42),
    Reg16::new(0x30b0, 0xff),
    Reg16::new(0x30b1, 0xff),
    Reg16::new(0x30b2, 0x10),
    Reg16::new(0x300e, 0x32),
    Reg16::new(0x300f, 0x21),
    Reg16::new(0x3010, 0x20),
    Reg16::new(0x3011, 0x04),
    Reg16::new(0x304c, 0x81),
    Reg16::new(0x30d7, 0x10),
    Reg16::new(0x30d9, 0x0d),
    Reg16::new(0x30db, 0x08),
    Reg16::new(0x3016, 0x82),
    Reg16::new(0x3018, 0x38),
    Reg16::new(0x3019, 0x30),
    Reg16::new(0x301a, 0x61),
    Reg16::new(0x307d, 0x00),
    Reg16::new(0x3087, 0x02),
    Reg16::new(0x3082, 0x20),
    Reg16::new(0x3015, 0x12),
    Reg16::new(0x3014, 0x04),
    Reg16::new(0x3013, 0xf7),
    Reg16::new(0x303c, 0x08),
    Reg16::new(0x303d, 0x18),
    Reg16::new(0x303e, 0x06),
    Reg16::new(0x303f, 0x0c),
    Reg16::new(0x3030, 0x62),
    Reg16::new(0x3031, 0x26),
    Reg16::new(0x3032, 0xe6),
    Reg16::new(0x3033, 0x6e),
    Reg16::new(0x3034, 0xea),
    Reg16::new(0x3035, 0xae),
    Reg16::new(0x3036, 0xa6),
    Reg16::new(0x3037, 0x6a),
    Reg16::new(0x3104, 0x02),
    Reg16::new(0x3105, 0xfd),
    Reg16::new(0x3106, 0x00),
    Reg16::new(0x3107, 0xff),
    Reg16::new(0x3300, 0x12),
    Reg16::new(0x3301, 0xde),
    Reg16::new(0x3302, 0xef),
    Reg16::new(0x3316, 0xff),
    Reg16::new(0x3317, 0x00),
    Reg16::new(0x3312, 0x26),
    Reg16::new(0x3314, 0x42),
    Reg16::new(0x3313, 0x2b),
    Reg16::new(0x3315, 0x42),
    Reg16::new(0x3310, 0xd0),
    Reg16::new(0x3311, 0xbd),
    Reg16::new(0x330c, 0x18),
    Reg16::new(0x330d, 0x18),
    Reg16::new(0x330e, 0x56),
    Reg16::new(0x330f, 0x5c),
    Reg16::new(0x330b, 0x1c),
    Reg16::new(0x3306, 0x5c),
    Reg16::new(0x3307, 0x11),
    Reg16::new(0x336a, 0x52),
    Reg16::new(0x3370, 0x46),
    Reg16::new(0x3376, 0x38),
    Reg16::new(0x3300, 0x13),
    Reg16::new(0x30b8, 0x20),
    Reg16::new(0x30b9, 0x17),
    Reg16::new(0x30ba, 0x04),
    Reg16::new(0x30bb, 0x08),
    Reg16::new(0x3507, 0x06),
    Reg16::new(0x350a, 0x4f),
    Reg16::new(0x3100, 0x02),
    Reg16::new(0x3301, 0xde),
    Reg16::new(0x3304, 0x00),
    Reg16::new(0x3400, 0x01),
    Reg16::new(0x3404, 0x11),
    Reg16::new(0x335f, 0x68),
    Reg16::new(0x3360, 0x18),
    Reg16::new(0x3361, 0x0c),
    Reg16::new(0x3362, 0x12),
    Reg16::new(0x3363, 0x88),
    Reg16::new(0x3364, 0xe4),
    Reg16::new(0x3403, 0x42),
    Reg16::new(0x3088, 0x02),
    Reg16::new(0x3089, 0x80),
    Reg16::new(0x308a, 0x01),
    Reg16::new(0x308b, 0xe0),
    Reg16::new(0x308d, 0x04),
    Reg16::new(0x3086, 0x03),
    Reg16::new(0x3086, 0x00),
    Reg16::new(0x3011, 0x00),
    Reg16::new(0x304c, 0x85),
    Reg16::new(0x3600, 0xd0),
    Reg16::new(0x335f, 0x68),
    Reg16::new(0x3360, 0x18),
    Reg16::new(0x3361, 0x0c),
    Reg16::new(0x3362, 0x01),
    Reg16::new(0x3363, 0x48),
    Reg16::new(0x3364, 0xf4),
    Reg16::new(0x3403, 0x42),
    Reg16::new(0x3088, 0x01),
    Reg16::new(0x3089, 0x40),
    Reg16::new(0x308a, 0x00),
    Reg16::new(0x308b, 0xf0),
    Reg16::new(0x307c, 0x12),
    Reg16::new(0x3090, 0xc8),
    Reg16::new(0x3600, 0xc4),
    Reg16::new(DRIVE_CAPABILITY, 0xc3),
    Reg16::new(CLOCK_POL_CONTROL, 0x21),
    Reg16::new(ISP_CONTROL_01, 0x83),
];

pub const SENSOR_FMT_JPEG: [Reg16; 5] = crate::ov3660::SENSOR_FMT_JPEG;
pub const SENSOR_FMT_RAW: [Reg16; 1] = crate::ov3660::SENSOR_FMT_RAW;
pub const SENSOR_FMT_GRAYSCALE: [Reg16; 2] = crate::ov3660::SENSOR_FMT_GRAYSCALE;
pub const SENSOR_FMT_YUV422: [Reg16; 2] = crate::ov3660::SENSOR_FMT_YUV422;
pub const SENSOR_FMT_RGB565: [Reg16; 2] = crate::ov3660::SENSOR_FMT_RGB565;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Ov3640WbPreset {
    pub awb_r: u8,
    pub awb_g: u8,
    pub awb_b: u8,
}

pub const OV3640_WB_PRESETS: [Ov3640WbPreset; 5] = [
    Ov3640WbPreset {
        awb_r: 0x00,
        awb_g: 0x00,
        awb_b: 0x00,
    }, // 0: auto
    Ov3640WbPreset {
        awb_r: 0x5e,
        awb_g: 0x40,
        awb_b: 0x46,
    }, // 1: sunny
    Ov3640WbPreset {
        awb_r: 0x68,
        awb_g: 0x40,
        awb_b: 0x4e,
    }, // 2: cloudy
    Ov3640WbPreset {
        awb_r: 0x44,
        awb_g: 0x40,
        awb_b: 0x70,
    }, // 3: office
    Ov3640WbPreset {
        awb_r: 0x52,
        awb_g: 0x40,
        awb_b: 0x58,
    }, // 4: home
];

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Ov3640BrightnessEntry {
    pub reg3354: u8,
    pub reg335e: u8,
}

pub const OV3640_BRIGHTNESS_LEVELS: [Ov3640BrightnessEntry; 7] = [
    Ov3640BrightnessEntry {
        reg3354: 0x09,
        reg335e: 0x30,
    }, // -3
    Ov3640BrightnessEntry {
        reg3354: 0x09,
        reg335e: 0x20,
    }, // -2
    Ov3640BrightnessEntry {
        reg3354: 0x09,
        reg335e: 0x10,
    }, // -1
    Ov3640BrightnessEntry {
        reg3354: 0x01,
        reg335e: 0x00,
    }, //  0
    Ov3640BrightnessEntry {
        reg3354: 0x01,
        reg335e: 0x10,
    }, // +1
    Ov3640BrightnessEntry {
        reg3354: 0x01,
        reg335e: 0x20,
    }, // +2
    Ov3640BrightnessEntry {
        reg3354: 0x01,
        reg335e: 0x30,
    }, // +3
];

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Ov3640ContrastEntry {
    pub y1: u8,
    pub y2: u8,
}

pub const OV3640_CONTRAST_LEVELS: [Ov3640ContrastEntry; 7] = [
    Ov3640ContrastEntry { y1: 0x14, y2: 0x14 }, // -3
    Ov3640ContrastEntry { y1: 0x18, y2: 0x18 }, // -2
    Ov3640ContrastEntry { y1: 0x1c, y2: 0x1c }, // -1
    Ov3640ContrastEntry { y1: 0x20, y2: 0x20 }, //  0
    Ov3640ContrastEntry { y1: 0x24, y2: 0x24 }, // +1
    Ov3640ContrastEntry { y1: 0x28, y2: 0x28 }, // +2
    Ov3640ContrastEntry { y1: 0x2c, y2: 0x2c }, // +3
];

pub const OV3640_SATURATION_LEVELS: [[u8; 2]; 5] = [
    [0x20, 0x20], // -2
    [0x30, 0x30], // -1
    [0x40, 0x40], //  0
    [0x50, 0x50], // +1
    [0x70, 0x70], // +2
];

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Ov3640EffectEntry {
    pub reg3355: u8,
    pub reg335a: u8,
    pub reg335b: u8,
}

pub const OV3640_SPECIAL_EFFECTS: [Ov3640EffectEntry; 7] = [
    Ov3640EffectEntry {
        reg3355: 0x00,
        reg335a: 0x00,
        reg335b: 0x00,
    }, // 0: normal
    Ov3640EffectEntry {
        reg3355: 0x40,
        reg335a: 0x00,
        reg335b: 0x00,
    }, // 1: negative
    Ov3640EffectEntry {
        reg3355: 0x18,
        reg335a: 0x80,
        reg335b: 0x80,
    }, // 2: grayscale
    Ov3640EffectEntry {
        reg3355: 0x18,
        reg335a: 0x80,
        reg335b: 0xc0,
    }, // 3: red
    Ov3640EffectEntry {
        reg3355: 0x18,
        reg335a: 0x60,
        reg335b: 0x60,
    }, // 4: green
    Ov3640EffectEntry {
        reg3355: 0x18,
        reg335a: 0xa0,
        reg335b: 0x40,
    }, // 5: blue
    Ov3640EffectEntry {
        reg3355: 0x18,
        reg335a: 0x40,
        reg335b: 0xa6,
    }, // 6: sepia
];

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Ov3640AeEntry {
    pub reg3018: u8,
    pub reg3019: u8,
    pub reg301a: u8,
}

pub const OV3640_AE_LEVELS: [Ov3640AeEntry; 11] = [
    Ov3640AeEntry {
        reg3018: 0x18,
        reg3019: 0x10,
        reg301a: 0x31,
    }, // -5
    Ov3640AeEntry {
        reg3018: 0x18,
        reg3019: 0x10,
        reg301a: 0x31,
    }, // -4
    Ov3640AeEntry {
        reg3018: 0x20,
        reg3019: 0x18,
        reg301a: 0x41,
    }, // -3
    Ov3640AeEntry {
        reg3018: 0x28,
        reg3019: 0x20,
        reg301a: 0x51,
    }, // -2
    Ov3640AeEntry {
        reg3018: 0x30,
        reg3019: 0x28,
        reg301a: 0x61,
    }, // -1
    Ov3640AeEntry {
        reg3018: 0x38,
        reg3019: 0x30,
        reg301a: 0x61,
    }, //  0
    Ov3640AeEntry {
        reg3018: 0x40,
        reg3019: 0x38,
        reg301a: 0x71,
    }, // +1
    Ov3640AeEntry {
        reg3018: 0x48,
        reg3019: 0x40,
        reg301a: 0x81,
    }, // +2
    Ov3640AeEntry {
        reg3018: 0x50,
        reg3019: 0x48,
        reg301a: 0x91,
    }, // +3
    Ov3640AeEntry {
        reg3018: 0x58,
        reg3019: 0x50,
        reg301a: 0x91,
    }, // +4
    Ov3640AeEntry {
        reg3018: 0x60,
        reg3019: 0x58,
        reg301a: 0xa1,
    }, // +5
];

pub const OV3640_SHARPNESS_LEVELS: [u8; 7] = [0x41, 0x42, 0x43, 0x60, 0x44, 0x45, 0x46];

/// OV3640 camera sensor driver state.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Ov3640 {
    slv_addr: u8,
    id: SensorId,
    status: CameraStatus,
    pixformat: PixelFormat,
    xclk_freq_hz: u32,
}

impl Default for Ov3640 {
    fn default() -> Self {
        Self::new()
    }
}

impl Ov3640 {
    /// Default 7-bit SCCB slave address for OV3640 (`0x3C`).
    pub const SLV_ADDR: u8 = CameraSccbAddr::OV3640;
    /// Canonical 16-bit PID (`0x3640`).
    pub const PID: u16 = CameraPid::OV3640.as_u16();

    /// Creates a new [`Ov3640`] driver instance with default SCCB address (`0x3C`).
    pub const fn new() -> Self {
        Self::with_addr(Self::SLV_ADDR)
    }

    /// Creates a new [`Ov3640`] driver instance with a custom SCCB slave address.
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

    /// Creates a new [`Ov3640`] driver instance from a detected [`SensorId`].
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
                awb: true,
                awb_gain: true,
                aec: true,
                aec2: false,
                ae_level: 0,
                aec_value: 300,
                agc: true,
                agc_gain: 0,
                gainceiling: 0,
                bpc: false,
                wpc: true,
                raw_gma: true,
                lenc: true,
                hmirror: false,
                vflip: false,
                dcw: true,
                colorbar: false,
            },
            pixformat: PixelFormat::Rgb565,
            xclk_freq_hz: 20_000_000,
        }
    }

    /// Probes and verifies the OV3640 family sensor on `bus` at `slv_addr`
    /// (`esp32_camera_ov3640_detect`).
    ///
    /// Matches any OV3640 family PID where `(pid & 0xFFF0) == 0x3640`
    /// (e.g. `0x3640`, `0x364C`) and normalizes `SensorId::pid` to `0x3640`.
    pub fn detect<B: SccbBus>(bus: &mut B, slv_addr: u8) -> Result<SensorId, CameraError> {
        if slv_addr != Self::SLV_ADDR {
            return Err(CameraError::NotDetected);
        }
        let raw_pid = bus.read_reg16_val16(slv_addr, 0x300A)?;
        if (raw_pid & 0xFFF0) != Self::PID {
            return Err(CameraError::NotDetected);
        }
        Ok(SensorId {
            midh: 0,
            midl: 0,
            pid: Self::PID,
            ver: (raw_pid & 0x000F) as u8,
        })
    }

    /// Returns the configured 7-bit SCCB slave address.
    #[inline]
    pub const fn slv_addr(&self) -> u8 {
        self.slv_addr
    }

    /// Returns the [`PixelFormat`] this driver was constructed with.
    ///
    /// SENS-04: while [`CameraSensor::set_pixformat`] refuses, nothing ever
    /// changes this field, so it reports the constructor default rather than
    /// anything the sensor is in. It becomes meaningful again when a verified
    /// OV3640 register map is ported and `set_pixformat` starts recording.
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
        CameraModel::Ov3640
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

    // SENS-04: `write_addr_reg` (the 0x38xx window block) and `configure_pll`
    // (SC_PLLS_CTRL0..3 at 0x303A..0x303D) were removed with the setters that
    // called them. Both programmed OV3660 registers. Recover them from git
    // history as a starting point only when porting a verified OV3640 map.

    fn apply_mirror_flip<B: SccbBus>(&self, bus: &mut B) -> Result<(), CameraError> {
        let (reg307c, reg3023, reg3090) = match (self.status.vflip, self.status.hmirror) {
            (true, true) => (0x13u8, 0x09u8, 0xC8u8),
            (true, false) => (0x11u8, 0x09u8, 0xC0u8),
            (false, true) => (0x12u8, 0x0Au8, 0xC8u8),
            (false, false) => (0x10u8, 0x0Au8, 0xC0u8),
        };
        bus.write_reg16(self.slv_addr, OV3640_TIMING_CTRL, reg307c)?;
        bus.write_reg16(self.slv_addr, OV3640_ARRAY_CTRL, reg3090)?;
        bus.write_reg16(self.slv_addr, OV3640_ARRAY_VSTART, reg3023)
    }

    fn enable_regulator_bypass<B: SccbBus>(&self, bus: &mut B) -> Result<(), CameraError> {
        let val = bus.read_reg16(self.slv_addr, PWCOM1)?;
        let new_val = val | 0x08;
        if new_val != val {
            bus.write_reg16(self.slv_addr, PWCOM1, new_val)?;
        }
        Ok(())
    }

    fn apply_vendor_power_save<B: SccbBus>(&self, bus: &mut B) -> Result<(), CameraError> {
        const MCU_REGS: [Reg16; 4] = [
            Reg16::new(OV3640_MCU_EIS_CTRL, 0x00),
            Reg16::new(OV3640_MCU_SLEEP, 0x00),
            Reg16::new(OV3640_MCU_IQ_CTRL, 0x00),
            Reg16::new(OV3640_MCU_FIFO_CTRL, 0x18),
        ];

        bus.write_reg16(self.slv_addr, OV3640_BANK_SEL, OV3640_BANK_JPEG_MCU)?;
        let mut first_err: Option<CameraError> = None;
        for entry in &MCU_REGS {
            if let Err(err) = bus.write_reg16(self.slv_addr, entry.reg, entry.val) {
                first_err = Some(err);
                break;
            }
        }
        let bank_restore = bus.write_reg16(self.slv_addr, OV3640_BANK_SEL, OV3640_BANK_DEFAULT);
        if let Some(err) = first_err {
            return Err(err);
        }
        bank_restore
    }

    fn apply_vendor_tuning<B: SccbBus>(&self, bus: &mut B) -> Result<(), CameraError> {
        self.enable_regulator_bypass(bus)?;
        self.apply_vendor_power_save(bus)
    }

    fn enable_sde<B: SccbBus>(&self, bus: &mut B) -> Result<(), CameraError> {
        bus.write_reg16(self.slv_addr, OV3640_SDE_CTRL, 0xEF)
    }

    fn init_isp_defaults<B: SccbBus>(&self, bus: &mut B) -> Result<(), CameraError> {
        bus.write_reg16(self.slv_addr, OV3640_DPC_CTRL, 0xDE)?;
        bus.write_reg16(self.slv_addr, OV3640_SIMPLE_AWB, 0xA5)?;
        bus.write_reg16(self.slv_addr, OV3640_AWB_CTRL, 0x00)
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

        if self.status.vflip {
            reg4514_test |= 1;
        }
        if self.status.hmirror {
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
            bus.write_reg16(self.slv_addr, Y_INCREMENT, 0x31)?;
        } else {
            bus.write_reg16(self.slv_addr, 0x4520, 0xb0)?;
            bus.write_reg16(self.slv_addr, X_INCREMENT, 0x11)?;
            bus.write_reg16(self.slv_addr, Y_INCREMENT, 0x11)?;
        }

        self.apply_mirror_flip(bus)
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

impl<B: SccbBus> CameraSensor<B> for Ov3640 {
    fn model(&self) -> CameraModel {
        CameraModel::Ov3640
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
        self.apply_vendor_tuning(bus)?;
        self.init_isp_defaults(bus)?;
        self.set_ae_level(bus, 0)?;
        bus.delay_ms(100);
        Ok(())
    }

    fn init_status(&mut self, bus: &mut B) -> Result<(), CameraError> {
        self.status.brightness = 0;
        self.status.contrast = 0;
        self.status.saturation = 0;
        self.status.sharpness = 0;
        let sharp = bus.read_reg16(self.slv_addr, OV3640_SHARPNESS)?;
        self.status.sharpness = if (sharp & 0x40) != 0 {
            -((sharp & 0x3F) as i8)
        } else {
            (sharp & 0x3F) as i8
        };
        self.status.denoise = self.get_denoise(bus)?;
        self.status.ae_level = 0;

        let reg_val = bus.read_reg16_val16(self.slv_addr, 0x3A18)? & 0x03FF;
        let mut level = 0u8;
        while level < (GainCeiling::Gain128x as u8) && (16u16 << (level + 1)) < reg_val {
            level += 1;
        }
        self.status.gainceiling = level;

        self.status.dcw = !self.check_reg_mask(bus, 0x5183, 0x80)?;
        self.status.agc = !self.check_reg_mask(bus, AEC_PK_MANUAL, AEC_PK_MANUAL_AGC_MANUALEN)?;
        self.status.aec = !self.check_reg_mask(bus, AEC_PK_MANUAL, AEC_PK_MANUAL_AEC_MANUALEN)?;

        let timing_mode = bus.read_reg16(self.slv_addr, OV3640_TIMING_CTRL)? & 0x1F;
        match timing_mode {
            0x11 => {
                self.status.vflip = true;
                self.status.hmirror = false;
            }
            0x12 => {
                self.status.vflip = false;
                self.status.hmirror = true;
            }
            0x13 => {
                self.status.vflip = true;
                self.status.hmirror = true;
            }
            _ => {
                self.status.vflip = false;
                self.status.hmirror = false;
            }
        }

        self.status.colorbar = self.check_reg_mask(bus, PRE_ISP_TEST_SETTING_1, TEST_COLOR_BAR)?;
        self.status.bpc = self.check_reg_mask(bus, 0x5000, 0x04)?;
        self.status.wpc = self.check_reg_mask(bus, 0x5000, 0x02)?;
        self.status.raw_gma = self.check_reg_mask(bus, 0x5000, 0x20)?;
        self.status.lenc = self.check_reg_mask(bus, 0x5000, 0x80)?;
        self.status.quality = bus.read_reg16(self.slv_addr, COMPRESSION_CTRL07)? & 0x3F;
        self.status.special_effect = 0;
        self.status.wb_mode = 0;
        self.status.awb = !self.check_reg_mask(bus, OV3640_AWB_CTRL, 0x08)?;
        self.status.awb_gain = bus.read_reg16(self.slv_addr, 0x3406)? == 0;
        self.status.agc_gain = self.get_agc_gain(bus)?;
        self.status.aec_value = self.get_aec_value(bus)?;
        self.status.aec2 = self.check_reg_mask(bus, 0x3A00, 0x04)?;
        Ok(())
    }

    /// Always returns [`CameraError::NotSupported`] (SENS-04).
    ///
    /// The format tables are aliased straight from [`crate::ov3660`], whose
    /// registers this sensor does not have. See the module documentation.
    fn set_pixformat(&mut self, _bus: &mut B, _pixformat: PixelFormat) -> Result<(), CameraError> {
        Err(CameraError::NotSupported)
    }

    /// Always returns [`CameraError::NotSupported`] (SENS-04).
    ///
    /// The window registers this would program (`X_ADDR_ST_H` and friends at
    /// 0x38xx) and `RATIO_TABLE` both come from [`crate::ov3660`]. See the
    /// module documentation.
    fn set_framesize(&mut self, _bus: &mut B, _framesize: FrameSize) -> Result<(), CameraError> {
        Err(CameraError::NotSupported)
    }

    fn set_contrast(&mut self, bus: &mut B, level: i32) -> Result<(), CameraError> {
        if !(-3..=3).contains(&level) {
            return Err(CameraError::InvalidArg);
        }
        let c = OV3640_CONTRAST_LEVELS[(level + 3) as usize];
        self.enable_sde(bus)?;
        crate::set_reg_bits16(bus, self.slv_addr, OV3640_SDE_ENABLE, 2, 1, 1)?;
        crate::set_reg_bits16(bus, self.slv_addr, OV3640_SDE_OFFSET, 0, 1, 1)?;
        bus.write_reg16(self.slv_addr, OV3640_CONTRAST_Y1, c.y1)?;
        bus.write_reg16(self.slv_addr, OV3640_CONTRAST_Y2, c.y2)?;
        self.status.contrast = level as i8;
        Ok(())
    }

    fn set_brightness(&mut self, bus: &mut B, level: i32) -> Result<(), CameraError> {
        if !(-3..=3).contains(&level) {
            return Err(CameraError::InvalidArg);
        }
        let b = OV3640_BRIGHTNESS_LEVELS[(level + 3) as usize];
        self.enable_sde(bus)?;
        crate::set_reg_bits16(bus, self.slv_addr, OV3640_SDE_ENABLE, 2, 1, 1)?;
        crate::set_reg_bits16(bus, self.slv_addr, OV3640_SDE_OFFSET, 0, 0x09, b.reg3354)?;
        bus.write_reg16(self.slv_addr, OV3640_BRIGHTNESS, b.reg335e)?;
        self.status.brightness = level as i8;
        Ok(())
    }

    fn set_saturation(&mut self, bus: &mut B, level: i32) -> Result<(), CameraError> {
        // SENS-25: the table has five entries. The range check used to
        // admit -4..=4 and then clamp to -2..=2, so -4 and -3 both produced
        // -2 while the call reported success.
        let level = crate::level_in(level, -2, 2)?;
        let clamped = level;
        let s = &OV3640_SATURATION_LEVELS[(clamped + 2) as usize];
        self.enable_sde(bus)?;
        crate::set_reg_bits16(bus, self.slv_addr, OV3640_SDE_ENABLE, 1, 1, 1)?;
        bus.write_reg16(self.slv_addr, OV3640_SATURATION_C, s[0])?;
        bus.write_reg16(self.slv_addr, OV3640_SATURATION_M, s[1])?;
        self.status.saturation = clamped as i8;
        Ok(())
    }

    fn set_sharpness(&mut self, bus: &mut B, level: i32) -> Result<(), CameraError> {
        if !(-3..=3).contains(&level) {
            return Err(CameraError::InvalidArg);
        }
        let sharp = OV3640_SHARPNESS_LEVELS[(level + 3) as usize];
        bus.write_reg16(self.slv_addr, OV3640_SHARPNESS, sharp)?;
        if sharp == 0x60 {
            bus.write_reg16(self.slv_addr, OV3640_SHARPNESS_AUTO, 0x03)?;
        }
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

    /// Always returns [`CameraError::NotSupported`] (SENS-04).
    ///
    /// `COMPRESSION_CTRL07` is 0x4407, an OV3660 address. See the module
    /// documentation.
    fn set_quality(&mut self, _bus: &mut B, _quality: i32) -> Result<(), CameraError> {
        Err(CameraError::NotSupported)
    }

    fn set_colorbar(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.write_reg_bits(bus, PRE_ISP_TEST_SETTING_1, TEST_COLOR_BAR, enable)?;
        self.status.colorbar = enable;
        Ok(())
    }

    fn set_whitebal(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        let val = if enable { 0x00 } else { 0x08 };
        bus.write_reg16(self.slv_addr, OV3640_AWB_CTRL, val)?;
        self.status.awb = enable;
        Ok(())
    }

    fn set_gain_ctrl(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.write_reg_bits(bus, AEC_PK_MANUAL, AEC_PK_MANUAL_AGC_MANUALEN, !enable)?;
        self.status.agc = enable;
        Ok(())
    }

    fn set_exposure_ctrl(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.write_reg_bits(bus, AEC_PK_MANUAL, AEC_PK_MANUAL_AEC_MANUALEN, !enable)?;
        self.status.aec = enable;
        Ok(())
    }

    fn set_hmirror(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.status.hmirror = enable;
        self.set_image_options(bus)
    }

    fn set_vflip(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.status.vflip = enable;
        self.set_image_options(bus)
    }

    fn set_aec2(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.write_reg_bits(bus, 0x3A00, 0x04, enable)?;
        self.status.aec2 = enable;
        Ok(())
    }

    fn set_awb_gain(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        let old_mode = self.status.wb_mode;
        let mode = if enable { old_mode as i32 } else { 0 };
        self.set_wb_mode(bus, mode)?;
        self.status.wb_mode = old_mode;
        self.status.awb_gain = enable;
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
        let e = OV3640_SPECIAL_EFFECTS[effect as usize];
        self.enable_sde(bus)?;
        crate::set_reg_bits16(bus, self.slv_addr, OV3640_SDE_ENABLE, 0, 0x58, e.reg3355)?;
        if e.reg3355 == 0x18 {
            bus.write_reg16(self.slv_addr, OV3640_EFFECT_C1, e.reg335a)?;
            bus.write_reg16(self.slv_addr, OV3640_EFFECT_C2, e.reg335b)?;
        }
        self.status.special_effect = effect as u8;
        Ok(())
    }

    fn set_wb_mode(&mut self, bus: &mut B, mode: i32) -> Result<(), CameraError> {
        if !(0..=4).contains(&mode) {
            return Err(CameraError::InvalidArg);
        }
        if mode == 0 {
            bus.write_reg16(self.slv_addr, OV3640_AWB_CTRL, 0x00)?;
        } else {
            let p = OV3640_WB_PRESETS[mode as usize];
            bus.write_reg16(self.slv_addr, OV3640_AWB_CTRL, 0x08)?;
            bus.write_reg16(self.slv_addr, OV3640_AWB_R_GAIN, p.awb_r)?;
            bus.write_reg16(self.slv_addr, OV3640_AWB_G_GAIN, p.awb_g)?;
            bus.write_reg16(self.slv_addr, OV3640_AWB_B_GAIN, p.awb_b)?;
        }
        self.status.wb_mode = mode as u8;
        self.status.awb = mode == 0;
        Ok(())
    }

    fn set_ae_level(&mut self, bus: &mut B, level: i32) -> Result<(), CameraError> {
        if !(-5..=5).contains(&level) {
            return Err(CameraError::InvalidArg);
        }
        let entry = OV3640_AE_LEVELS[(level + 5) as usize];
        bus.write_reg16(self.slv_addr, OV3640_AEC_ALGO, 0x00)?;
        bus.write_reg16(self.slv_addr, OV3640_AEC_18, entry.reg3018)?;
        bus.write_reg16(self.slv_addr, OV3640_AEC_19, entry.reg3019)?;
        bus.write_reg16(self.slv_addr, OV3640_AEC_1A, entry.reg301a)?;
        self.status.ae_level = level as i8;
        Ok(())
    }

    fn set_dcw(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.write_reg_bits(bus, 0x5183, 0x80, !enable)?;
        self.status.dcw = enable;
        Ok(())
    }

    fn set_bpc(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.write_reg_bits(bus, 0x5000, 0x04, enable)?;
        self.status.bpc = enable;
        Ok(())
    }

    fn set_wpc(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.write_reg_bits(bus, 0x5000, 0x02, enable)?;
        self.status.wpc = enable;
        Ok(())
    }

    fn set_raw_gma(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.write_reg_bits(bus, 0x5000, 0x20, enable)?;
        self.status.raw_gma = enable;
        Ok(())
    }

    fn set_lenc(&mut self, bus: &mut B, enable: bool) -> Result<(), CameraError> {
        self.write_reg_bits(bus, 0x5000, 0x80, enable)?;
        self.status.lenc = enable;
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

    fn set_reg(&mut self, bus: &mut B, reg: i32, mask: i32, value: i32) -> Result<(), CameraError> {
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

    /// Always returns [`CameraError::NotSupported`] (SENS-04).
    ///
    /// Programs the same OV3660 window block as `set_framesize`. See the
    /// module documentation.
    #[allow(clippy::too_many_arguments)]
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
        Err(CameraError::NotSupported)
    }

    /// Always returns [`CameraError::NotSupported`] (SENS-04).
    ///
    /// `SC_PLLS_CTRL0..3` are 0x303A..0x303D, OV3660 addresses. See the
    /// module documentation.
    #[allow(clippy::too_many_arguments)]
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
        self.xclk_freq_hz = (xclk as u32).saturating_mul(1_000_000);
        Ok(())
    }
}
