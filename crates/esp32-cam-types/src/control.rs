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
    pub awb: bool,
    pub awb_gain: bool,
    pub aec: bool,
    pub aec2: bool,
    pub ae_level: i8,
    pub aec_value: u16,
    pub agc: bool,
    pub agc_gain: u8,
    pub gainceiling: u8,
    pub bpc: bool,
    pub wpc: bool,
    pub raw_gma: bool,
    pub lenc: bool,
    pub hmirror: bool,
    pub vflip: bool,
    pub dcw: bool,
    pub colorbar: bool,
}

/// Clamps a signed image adjustment level into `[-2, 2]`.
///
/// `[-2, 2]` is the narrowest range any sensor in this workspace accepts, not
/// a hardware bound. Clamping into it substitutes a different setting for a
/// valid one on most parts; see [`CameraStatus::clamped`].
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
    /// The narrowest signed level range any sensor here accepts, not a
    /// workspace-wide contract.
    ///
    /// Most drivers are wider; see [`Self::clamped`].
    pub const LEVEL_MIN: i8 = -2;
    /// The upper end of [`Self::LEVEL_MIN`]'s narrowest-common range, not a
    /// workspace-wide contract.
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
    ///
    /// That is the narrowest range any sensor accepts, so this reports
    /// `false` for levels most drivers take happily — `-3..=3` is common.
    /// It is not a validity check for any particular sensor.
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

    /// Returns a copy with every numeric field forced into this type's own
    /// declared ranges.
    ///
    /// **These are not hardware bounds, despite what this method used to
    /// claim.** The signed levels are clamped to
    /// [`Self::LEVEL_MIN`]..=[`Self::LEVEL_MAX`], which is `-2..=2`, while the
    /// drivers in this workspace accept `-2..=2`, `-3..=3`, `-4..=4`,
    /// `-5..=5` and `0..=8` depending on the sensor and the control. So this
    /// silently rewrites values that are perfectly valid on real parts: an
    /// `ae_level` of 5 is accepted by OV3660 and OV5640 and becomes 2 here,
    /// a materially different exposure target.
    ///
    /// Nothing in this workspace calls it. Validate against the driver's own
    /// range — `esp32_cam_sensors::level_in` — rather than normalising
    /// through this. Kept only because removing public API is the owner's
    /// call; see `test_clamped_narrows_levels_real_sensors_accept`.
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
            awb: self.awb,
            awb_gain: self.awb_gain,
            aec: self.aec,
            aec2: self.aec2,
            ae_level: clamp_level(self.ae_level),
            aec_value: Self::clamp_aec_value(self.aec_value),
            agc: self.agc,
            agc_gain: Self::clamp_agc_gain(self.agc_gain),
            gainceiling: if self.gainceiling > Self::MAX_GAINCEILING {
                Self::MAX_GAINCEILING
            } else {
                self.gainceiling
            },
            bpc: self.bpc,
            wpc: self.wpc,
            raw_gma: self.raw_gma,
            lenc: self.lenc,
            hmirror: self.hmirror,
            vflip: self.vflip,
            dcw: self.dcw,
            colorbar: self.colorbar,
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
        assert!(status.awb);
        assert!(status.awb_gain);
        assert!(status.aec);
        assert!(!status.aec2);
        assert!(status.agc);
        assert!(!status.bpc);
        assert!(status.wpc);
        assert!(status.raw_gma);
        assert!(status.lenc);
        assert!(!status.hmirror);
        assert!(!status.vflip);
        assert!(status.dcw);
        assert!(!status.colorbar);

        status.framesize = FrameSize::Hd;
        status.quality = 10;
        status.hmirror = true;
        status.vflip = true;
        status.colorbar = true;
        assert_eq!(status.framesize, FrameSize::Hd);
        assert_eq!(status.quality, 10);
        assert!(status.hmirror);
        assert!(status.vflip);
        assert!(status.colorbar);
    }

    #[test]
    fn test_camera_status_new_and_is_valid() {
        let status = CameraStatus::new();
        assert!(status.is_valid());

        const CONST_STATUS: CameraStatus = CameraStatus::new();
        assert!(CONST_STATUS.is_valid());

        assert!(CameraStatus::is_valid_flag(0));
        assert!(CameraStatus::is_valid_flag(1));
        assert!(!CameraStatus::is_valid_flag(2));
        assert!(!CameraStatus::is_valid_flag(u8::MAX));

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

        assert_eq!(CameraStatus::clamp_flag(0), 0);
        assert_eq!(CameraStatus::clamp_flag(1), 1);
        assert_eq!(CameraStatus::clamp_flag(2), 1);
        assert_eq!(CameraStatus::clamp_flag(u8::MAX), 1);

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
            awb: false,
            awb_gain: true,
            aec: false,
            aec2: true,
            ae_level: -10,
            aec_value: 5000,
            agc: false,
            agc_gain: 99,
            gainceiling: 20,
            bpc: true,
            wpc: false,
            raw_gma: false,
            lenc: true,
            hmirror: true,
            vflip: true,
            dcw: false,
            colorbar: true,
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
        assert!(!sanitized.awb);
        assert!(sanitized.hmirror);
        assert!(sanitized.vflip);
        assert!(sanitized.colorbar);
    }
}

#[cfg(test)]
mod sweep_tests {
    use super::*;

    /// `clamped()` advertises "valid hardware bounds" no longer, and this is
    /// why: the levels it forces into `-2..=2` are accepted as-is by real
    /// drivers in this workspace. OV3660 and OV5640 take `ae_level -5..=5`;
    /// NT99141, OV2640, OV3660 and OV5640 all take `brightness -3..=3`.
    ///
    /// Nothing calls `clamped()`, so this is a latent trap rather than a live
    /// defect. The test exists so it cannot quietly become a live one.
    #[test]
    fn test_clamped_narrows_levels_real_sensors_accept() {
        let status = CameraStatus {
            ae_level: 5,
            brightness: 3,
            saturation: -4,
            ..CameraStatus::new()
        };
        let clamped = status.clamped();

        assert_eq!(clamped.ae_level, 2, "OV5640 accepts 5; clamped() gives 2");
        assert_eq!(
            clamped.brightness, 2,
            "NT99141 accepts 3; clamped() gives 2"
        );
        assert_eq!(
            clamped.saturation, -2,
            "NT99141 accepts -4; clamped() gives -2"
        );
    }

    /// The same mismatch seen from the predicate side.
    #[test]
    fn test_is_valid_level_rejects_levels_drivers_accept() {
        assert!(!CameraStatus::is_valid_level(3));
        assert!(!CameraStatus::is_valid_level(-5));
        assert!(CameraStatus::is_valid_level(2));
    }
}
