//! Sensor control state, AGC gain ceilings, and capture mode settings.

use crate::frame_size::FrameSize;

/// Sensor AGC gain ceiling (`gainceiling_t` from `driver/include/sensor.h`).
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum GainCeiling {
    #[default]
    Gain2x = 0,
    Gain4x = 1,
    Gain8x = 2,
    Gain16x = 3,
    Gain32x = 4,
    Gain64x = 5,
    Gain128x = 6,
}

#[allow(non_upper_case_globals)]
impl GainCeiling {
    pub const X2: Self = Self::Gain2x;
    pub const X4: Self = Self::Gain4x;
    pub const X8: Self = Self::Gain8x;
    pub const X16: Self = Self::Gain16x;
    pub const X32: Self = Self::Gain32x;
    pub const X64: Self = Self::Gain64x;
    pub const X128: Self = Self::Gain128x;

    pub const Gain2X: Self = Self::Gain2x;
    pub const Gain4X: Self = Self::Gain4x;
    pub const Gain8X: Self = Self::Gain8x;
    pub const Gain16X: Self = Self::Gain16x;
    pub const Gain32X: Self = Self::Gain32x;
    pub const Gain64X: Self = Self::Gain64x;
    pub const Gain128X: Self = Self::Gain128x;

    /// Converts a raw index (`0..=6`) into a [`GainCeiling`].
    #[inline]
    pub const fn from_u8(val: u8) -> Option<Self> {
        match val {
            0 => Some(Self::Gain2x),
            1 => Some(Self::Gain4x),
            2 => Some(Self::Gain8x),
            3 => Some(Self::Gain16x),
            4 => Some(Self::Gain32x),
            5 => Some(Self::Gain64x),
            6 => Some(Self::Gain128x),
            _ => None,
        }
    }
}

/// Sensor window timing settings (`ratio_settings_t` in `sensor.h`).
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Hash)]
pub struct RatioSettings {
    pub max_width: u16,
    pub max_height: u16,
    pub start_x: u16,
    pub start_y: u16,
    pub end_x: u16,
    pub end_y: u16,
    pub offset_x: u16,
    pub offset_y: u16,
    pub total_x: u16,
    pub total_y: u16,
}

/// Mutable camera control state (`camera_status_t` in `sensor.h`).
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct CameraStatus {
    pub framesize: FrameSize,
    pub scale: bool,
    pub binning: bool,
    pub quality: u8,
    pub brightness: i8,
    pub contrast: i8,
    pub saturation: i8,
    pub sharpness: i8,
    pub denoise: u8,
    pub special_effect: u8,
    pub wb_mode: u8,
    pub awb: u8,
    pub awb_gain: u8,
    pub aec: u8,
    pub aec2: u8,
    pub ae_level: i8,
    pub aec_value: u16,
    pub agc: u8,
    pub agc_gain: u8,
    pub gainceiling: u8,
    pub bpc: u8,
    pub wpc: u8,
    pub raw_gma: u8,
    pub lenc: u8,
    pub hmirror: u8,
    pub vflip: u8,
    pub dcw: u8,
    pub colorbar: u8,
}

/// Clamps a signed image adjustment level (`brightness`, `contrast`,
/// `saturation`, `sharpness`, `ae_level`) into `[-2, 2]`.
#[must_use]
#[inline]
pub const fn clamp_level(level: i8) -> i8 {
    if level < CameraStatus::LEVEL_MIN {
        CameraStatus::LEVEL_MIN
    } else if level > CameraStatus::LEVEL_MAX {
        CameraStatus::LEVEL_MAX
    } else {
        level
    }
}

impl CameraStatus {
    /// Minimum permissible signed level (`brightness`, `contrast`, etc.).
    pub const LEVEL_MIN: i8 = -2;
    /// Maximum permissible signed level (`brightness`, `contrast`, etc.).
    pub const LEVEL_MAX: i8 = 2;
    /// Maximum permissible JPEG quality factor (`0..=63`).
    pub const MAX_QUALITY: u8 = 63;
    /// Maximum permissible denoise level (`0..=8`).
    pub const MAX_DENOISE: u8 = 8;
    /// Maximum permissible special effect index (`0..=6`).
    pub const MAX_SPECIAL_EFFECT: u8 = 6;
    /// Maximum permissible white-balance preset (`0..=4`).
    pub const MAX_WB_MODE: u8 = 4;
    /// Maximum permissible manual AGC gain (`0..=30`).
    pub const MAX_AGC_GAIN: u8 = 30;
    /// Maximum permissible manual AEC exposure value (`0..=1200`).
    pub const MAX_AEC_VALUE: u16 = 1200;
    /// Maximum permissible AGC gain ceiling index (`0..=6`).
    pub const MAX_GAINCEILING: u8 = 6;

    /// Creates a new [`CameraStatus`] with default settings.
    #[must_use]
    pub const fn new() -> Self {
        Self {
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
        }
    }

    /// Clamps a signed adjustment level into
    /// [`Self::LEVEL_MIN`]..=[`Self::LEVEL_MAX`].
    #[must_use]
    #[inline]
    pub const fn clamp_level(level: i8) -> i8 {
        clamp_level(level)
    }

    /// Returns `true` when `level` is within `-2..=2`.
    #[must_use]
    #[inline]
    pub const fn is_valid_level(level: i8) -> bool {
        level >= Self::LEVEL_MIN && level <= Self::LEVEL_MAX
    }

    /// Returns `true` when a C-style boolean control byte is `0` or `1`.
    #[must_use]
    #[inline]
    pub const fn is_valid_flag(flag: u8) -> bool {
        flag <= 1
    }

    /// Normalizes a C-style boolean control byte into `0` or `1`.
    #[must_use]
    #[inline]
    pub const fn clamp_flag(flag: u8) -> u8 {
        if flag == 0 {
            0
        } else {
            1
        }
    }

    /// Decodes a validated C-style boolean control byte (`0` or `1`) to `bool`.
    #[must_use]
    #[inline]
    pub const fn decode_flag(flag: u8) -> Option<bool> {
        match flag {
            0 => Some(false),
            1 => Some(true),
            _ => None,
        }
    }

    /// Clamps manual AGC gain into `0..=30`.
    #[must_use]
    #[inline]
    pub const fn clamp_agc_gain(gain: u8) -> u8 {
        if gain > Self::MAX_AGC_GAIN {
            Self::MAX_AGC_GAIN
        } else {
            gain
        }
    }

    /// Clamps manual AEC exposure value into `0..=1200`.
    #[must_use]
    #[inline]
    pub const fn clamp_aec_value(val: u16) -> u16 {
        if val > Self::MAX_AEC_VALUE {
            Self::MAX_AEC_VALUE
        } else {
            val
        }
    }

    /// Returns a copy with all numeric ranges and `0/1` boolean flags clamped
    /// to valid hardware bounds.
    #[must_use]
    pub const fn clamped(self) -> Self {
        Self {
            framesize: self.framesize,
            scale: self.scale,
            binning: self.binning,
            quality: if self.quality > Self::MAX_QUALITY {
                Self::MAX_QUALITY
            } else {
                self.quality
            },
            brightness: clamp_level(self.brightness),
            contrast: clamp_level(self.contrast),
            saturation: clamp_level(self.saturation),
            sharpness: clamp_level(self.sharpness),
            denoise: if self.denoise > Self::MAX_DENOISE {
                Self::MAX_DENOISE
            } else {
                self.denoise
            },
            special_effect: if self.special_effect > Self::MAX_SPECIAL_EFFECT {
                Self::MAX_SPECIAL_EFFECT
            } else {
                self.special_effect
            },
            wb_mode: if self.wb_mode > Self::MAX_WB_MODE {
                Self::MAX_WB_MODE
            } else {
                self.wb_mode
            },
            awb: Self::clamp_flag(self.awb),
            awb_gain: Self::clamp_flag(self.awb_gain),
            aec: Self::clamp_flag(self.aec),
            aec2: Self::clamp_flag(self.aec2),
            ae_level: clamp_level(self.ae_level),
            aec_value: Self::clamp_aec_value(self.aec_value),
            agc: Self::clamp_flag(self.agc),
            agc_gain: Self::clamp_agc_gain(self.agc_gain),
            gainceiling: if self.gainceiling > Self::MAX_GAINCEILING {
                Self::MAX_GAINCEILING
            } else {
                self.gainceiling
            },
            bpc: Self::clamp_flag(self.bpc),
            wpc: Self::clamp_flag(self.wpc),
            raw_gma: Self::clamp_flag(self.raw_gma),
            lenc: Self::clamp_flag(self.lenc),
            hmirror: Self::clamp_flag(self.hmirror),
            vflip: Self::clamp_flag(self.vflip),
            dcw: Self::clamp_flag(self.dcw),
            colorbar: Self::clamp_flag(self.colorbar),
        }
    }

    /// Validates that all fields are within their permissible hardware ranges.
    #[must_use]
    #[inline]
    pub const fn is_valid(&self) -> bool {
        (self.framesize as usize) < FrameSize::COUNT
            && self.quality <= Self::MAX_QUALITY
            && Self::is_valid_level(self.brightness)
            && Self::is_valid_level(self.contrast)
            && Self::is_valid_level(self.saturation)
            && Self::is_valid_level(self.sharpness)
            && Self::is_valid_level(self.ae_level)
            && self.denoise <= Self::MAX_DENOISE
            && self.special_effect <= Self::MAX_SPECIAL_EFFECT
            && self.wb_mode <= Self::MAX_WB_MODE
            && self.aec_value <= Self::MAX_AEC_VALUE
            && self.agc_gain <= Self::MAX_AGC_GAIN
            && self.gainceiling <= Self::MAX_GAINCEILING
            && Self::is_valid_flag(self.awb)
            && Self::is_valid_flag(self.awb_gain)
            && Self::is_valid_flag(self.aec)
            && Self::is_valid_flag(self.aec2)
            && Self::is_valid_flag(self.agc)
            && Self::is_valid_flag(self.bpc)
            && Self::is_valid_flag(self.wpc)
            && Self::is_valid_flag(self.raw_gma)
            && Self::is_valid_flag(self.lenc)
            && Self::is_valid_flag(self.hmirror)
            && Self::is_valid_flag(self.vflip)
            && Self::is_valid_flag(self.dcw)
            && Self::is_valid_flag(self.colorbar)
    }
}

impl Default for CameraStatus {
    fn default() -> Self {
        Self::new()
    }
}

/// Frame buffer acquisition policy (`camera_grab_mode_t`).
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Hash)]
pub enum CameraGrabMode {
    #[default]
    WhenEmpty = 0,
    Latest = 1,
}

/// Alias for [`CameraGrabMode`].
pub type GrabMode = CameraGrabMode;

/// Frame buffer memory allocation target (`camera_fb_location_t`).
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Hash)]
pub enum CameraFbLocation {
    #[default]
    Psram = 0,
    Dram = 1,
}

/// Alias for [`CameraFbLocation`].
pub type FbLocation = CameraFbLocation;

/// Hardware/software color conversion mode (`camera_conv_mode_t`).
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Hash)]
pub enum CameraConvMode {
    #[default]
    Disable = 0,
    Rgb565ToYuv422 = 1,
    Yuv422ToRgb565 = 2,
    Yuv422ToYuv420 = 3,
}

/// Alias for [`CameraConvMode`].
pub type ConvMode = CameraConvMode;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_camera_status_defaults_and_mutation() {
        let mut status = CameraStatus::default();
        assert_eq!(status.framesize, FrameSize::Qvga);
        assert!(!status.scale);
        assert!(!status.binning);
        assert_eq!(status.quality, 12);
        assert_eq!(status.awb, 1);
        assert_eq!(status.aec, 1);
        assert_eq!(status.agc, 1);

        status.framesize = FrameSize::Hd;
        status.quality = 10;
        status.hmirror = 1;
        status.vflip = 1;
        assert_eq!(status.framesize, FrameSize::Hd);
        assert_eq!(status.quality, 10);
        assert_eq!(status.hmirror, 1);
        assert_eq!(status.vflip, 1);
    }

    #[test]
    fn test_camera_status_new_and_is_valid() {
        let status = CameraStatus::new();
        assert!(status.is_valid());

        const CONST_STATUS: CameraStatus = CameraStatus::new();
        assert!(CONST_STATUS.is_valid());

        let mut invalid = status;
        invalid.quality = 64;
        assert!(!invalid.is_valid());

        let mut invalid_b = status;
        invalid_b.brightness = 3;
        assert!(!invalid_b.is_valid());

        let mut invalid_b_neg = status;
        invalid_b_neg.brightness = -3;
        assert!(!invalid_b_neg.is_valid());

        let mut invalid_contrast = status;
        invalid_contrast.contrast = 3;
        assert!(!invalid_contrast.is_valid());

        let mut invalid_saturation = status;
        invalid_saturation.saturation = -3;
        assert!(!invalid_saturation.is_valid());

        let mut invalid_sharpness = status;
        invalid_sharpness.sharpness = 3;
        assert!(!invalid_sharpness.is_valid());

        let mut invalid_ae = status;
        invalid_ae.ae_level = 3;
        assert!(!invalid_ae.is_valid());

        let mut invalid_denoise = status;
        invalid_denoise.denoise = 9;
        assert!(!invalid_denoise.is_valid());

        let mut invalid_effect = status;
        invalid_effect.special_effect = 7;
        assert!(!invalid_effect.is_valid());

        let mut invalid_wb = status;
        invalid_wb.wb_mode = 5;
        assert!(!invalid_wb.is_valid());

        let mut invalid_awb = status;
        invalid_awb.awb = 2;
        assert!(!invalid_awb.is_valid());

        let mut invalid_vflip = status;
        invalid_vflip.vflip = 2;
        assert!(!invalid_vflip.is_valid());

        let mut invalid_gainceiling = status;
        invalid_gainceiling.gainceiling = 7;
        assert!(!invalid_gainceiling.is_valid());

        let mut valid_agc_max = status;
        valid_agc_max.agc_gain = 30;
        assert!(valid_agc_max.is_valid());

        let mut invalid_agc_gain = status;
        invalid_agc_gain.agc_gain = 31;
        assert!(!invalid_agc_gain.is_valid());

        let mut valid_aec_max = status;
        valid_aec_max.aec_value = 1200;
        assert!(valid_aec_max.is_valid());

        let mut invalid_aec_value = status;
        invalid_aec_value.aec_value = 1201;
        assert!(!invalid_aec_value.is_valid());

        let mut invalid_colorbar = status;
        invalid_colorbar.colorbar = 2;
        assert!(!invalid_colorbar.is_valid());
    }

    #[test]
    fn test_clamp_level_and_status_clamping() {
        assert_eq!(clamp_level(-128), -2);
        assert_eq!(clamp_level(-3), -2);
        assert_eq!(clamp_level(-2), -2);
        assert_eq!(clamp_level(0), 0);
        assert_eq!(clamp_level(2), 2);
        assert_eq!(clamp_level(3), 2);
        assert_eq!(clamp_level(127), 2);
        assert_eq!(CameraStatus::clamp_level(-10), -2);
        assert_eq!(CameraStatus::clamp_level(10), 2);

        const CONST_CLAMPED: i8 = clamp_level(99);
        assert_eq!(CONST_CLAMPED, 2);

        assert_eq!(CameraStatus::clamp_agc_gain(0), 0);
        assert_eq!(CameraStatus::clamp_agc_gain(30), 30);
        assert_eq!(CameraStatus::clamp_agc_gain(31), 30);
        assert_eq!(CameraStatus::clamp_agc_gain(255), 30);

        assert_eq!(CameraStatus::clamp_aec_value(0), 0);
        assert_eq!(CameraStatus::clamp_aec_value(1200), 1200);
        assert_eq!(CameraStatus::clamp_aec_value(1201), 1200);
        assert_eq!(CameraStatus::clamp_aec_value(u16::MAX), 1200);

        assert_eq!(CameraStatus::decode_flag(0), Some(false));
        assert_eq!(CameraStatus::decode_flag(1), Some(true));
        assert_eq!(CameraStatus::decode_flag(2), None);

        let out_of_bounds = CameraStatus {
            framesize: FrameSize::Hd,
            scale: true,
            binning: false,
            quality: 200,
            brightness: -50,
            contrast: 50,
            saturation: -9,
            sharpness: 9,
            denoise: 20,
            special_effect: 99,
            wb_mode: 12,
            awb: 5,
            awb_gain: 2,
            aec: 9,
            aec2: 3,
            ae_level: -10,
            aec_value: 5000,
            agc: 4,
            agc_gain: 99,
            gainceiling: 20,
            bpc: 2,
            wpc: 3,
            raw_gma: 4,
            lenc: 5,
            hmirror: 6,
            vflip: 7,
            dcw: 8,
            colorbar: 9,
        };
        assert!(!out_of_bounds.is_valid());

        let sanitized = out_of_bounds.clamped();
        assert!(sanitized.is_valid());
        assert_eq!(sanitized.quality, 63);
        assert_eq!(sanitized.brightness, -2);
        assert_eq!(sanitized.contrast, 2);
        assert_eq!(sanitized.saturation, -2);
        assert_eq!(sanitized.sharpness, 2);
        assert_eq!(sanitized.ae_level, -2);
        assert_eq!(sanitized.denoise, 8);
        assert_eq!(sanitized.special_effect, 6);
        assert_eq!(sanitized.wb_mode, 4);
        assert_eq!(sanitized.aec_value, 1200);
        assert_eq!(sanitized.agc_gain, 30);
        assert_eq!(sanitized.gainceiling, 6);
        assert_eq!(sanitized.awb, 1);
        assert_eq!(sanitized.hmirror, 1);
        assert_eq!(sanitized.vflip, 1);
        assert_eq!(sanitized.colorbar, 1);
    }
}
