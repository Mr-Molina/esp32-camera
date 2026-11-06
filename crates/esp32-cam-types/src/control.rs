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

impl CameraStatus {
    /// Creates a new [`CameraStatus`] with default settings.
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

    /// Validates that all fields are within their permissible hardware ranges.
    #[inline]
    pub const fn is_valid(&self) -> bool {
        self.quality <= 63
            && self.brightness >= -2 && self.brightness <= 2
            && self.contrast >= -2 && self.contrast <= 2
            && self.saturation >= -2 && self.saturation <= 2
            && self.sharpness >= -2 && self.sharpness <= 2
            && self.ae_level >= -2 && self.ae_level <= 2
            && self.denoise <= 8
            && self.special_effect <= 6
            && self.wb_mode <= 4
            && self.awb <= 1
            && self.awb_gain <= 1
            && self.aec <= 1
            && self.aec2 <= 1
            && self.agc <= 1
            && self.gainceiling <= 6
            && self.bpc <= 1
            && self.wpc <= 1
            && self.raw_gma <= 1
            && self.lenc <= 1
            && self.hmirror <= 1
            && self.vflip <= 1
            && self.dcw <= 1
            && self.colorbar <= 1
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
    }
}
