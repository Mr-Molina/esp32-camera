//! Output frame resolution presets, aspect ratios, and resolution table.

use crate::pixel_format::PixelFormat;

/// Output frame resolution preset (`framesize_t` in `sensor.h`).
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum FrameSize {
    Size96x96 = 0,
    Qqvga = 1,
    Size128x128 = 2,
    Qcif = 3,
    Hqvga = 4,
    Size240x240 = 5,
    #[default]
    Qvga = 6,
    Size320x320 = 7,
    Cif = 8,
    Hvga = 9,
    Vga = 10,
    Svga = 11,
    Xga = 12,
    Hd = 13,
    Sxga = 14,
    Uxga = 15,
    Fhd = 16,
    PHd = 17,
    P3Mp = 18,
    Qxga = 19,
    Qhd = 20,
    Wqxga = 21,
    PFhd = 22,
    Qsxga = 23,
    Size5Mp = 24,
}

#[allow(non_upper_case_globals)]
impl FrameSize {
    pub const COUNT: usize = 25;

    pub const Size96X96: Self = Self::Size96x96;
    pub const F96x96: Self = Self::Size96x96;
    pub const QQVGA: Self = Self::Qqvga;
    pub const Size128X128: Self = Self::Size128x128;
    pub const F128x128: Self = Self::Size128x128;
    pub const QCIF: Self = Self::Qcif;
    pub const HQVGA: Self = Self::Hqvga;
    pub const Size240X240: Self = Self::Size240x240;
    pub const F240x240: Self = Self::Size240x240;
    pub const QVGA: Self = Self::Qvga;
    pub const Size320X320: Self = Self::Size320x320;
    pub const F320x320: Self = Self::Size320x320;
    pub const CIF: Self = Self::Cif;
    pub const HVGA: Self = Self::Hvga;
    pub const VGA: Self = Self::Vga;
    pub const SVGA: Self = Self::Svga;
    pub const XGA: Self = Self::Xga;
    pub const HD: Self = Self::Hd;
    pub const SXGA: Self = Self::Sxga;
    pub const UXGA: Self = Self::Uxga;
    pub const FHD: Self = Self::Fhd;
    pub const P_HD: Self = Self::PHd;
    pub const PortraitHd: Self = Self::PHd;
    pub const P3mp: Self = Self::P3Mp;
    pub const P_3MP: Self = Self::P3Mp;
    pub const Portrait3Mp: Self = Self::P3Mp;
    pub const QXGA: Self = Self::Qxga;
    pub const QHD: Self = Self::Qhd;
    pub const WQXGA: Self = Self::Wqxga;
    pub const P_FHD: Self = Self::PFhd;
    pub const PortraitFhd: Self = Self::PFhd;
    pub const QSXGA: Self = Self::Qsxga;
    pub const Size5mp: Self = Self::Size5Mp;
    pub const F5Mp: Self = Self::Size5Mp;
    pub const FiveMp: Self = Self::Size5Mp;

    /// Returns the table index (`0..25`) for this [`FrameSize`].
    #[inline]
    pub const fn index(self) -> usize {
        self as usize
    }

    /// Returns the [`ResolutionInfo`] entry from [`RESOLUTION_TABLE`].
    #[inline]
    pub const fn resolution(self) -> ResolutionInfo {
        RESOLUTION_TABLE[self as usize]
    }

    /// Returns the horizontal width in pixels.
    #[inline]
    pub const fn width(self) -> u16 {
        RESOLUTION_TABLE[self as usize].width
    }

    /// Returns the vertical height in pixels.
    #[inline]
    pub const fn height(self) -> u16 {
        RESOLUTION_TABLE[self as usize].height
    }

    /// Returns the [`AspectRatio`] for this frame size.
    #[inline]
    pub const fn aspect_ratio(self) -> AspectRatio {
        RESOLUTION_TABLE[self as usize].aspect_ratio
    }

    /// Returns the total pixel count (`width * height`) for this frame size.
    #[inline]
    pub const fn pixel_count(self) -> usize {
        let res = &RESOLUTION_TABLE[self as usize];
        (res.width as usize) * (res.height as usize)
    }

    /// Returns the uncompressed frame buffer size in bytes for the given
    /// [`PixelFormat`], or `None` if the format has variable compressed
    /// size (e.g. [`PixelFormat::Jpeg`]) or on arithmetic overflow.
    #[inline]
    pub const fn checked_byte_size(self, format: PixelFormat) -> Option<usize> {
        let pixels = self.pixel_count();
        match format {
            PixelFormat::Jpeg => None,
            PixelFormat::Grayscale | PixelFormat::Raw | PixelFormat::Raw8 => Some(pixels),
            PixelFormat::Rgb565
            | PixelFormat::Yuv422
            | PixelFormat::Rgb555
            | PixelFormat::Rgb444 => pixels.checked_mul(2),
            PixelFormat::Rgb888 => pixels.checked_mul(3),
            PixelFormat::Yuv420 => match pixels.checked_mul(3) {
                Some(val) => Some(val / 2),
                None => None,
            },
        }
    }

    /// Converts a raw index (`0..25`) into a [`FrameSize`].
    #[inline]
    pub fn from_u8(val: u8) -> Option<Self> {
        ALL_FRAMESIZES.get(val as usize).copied()
    }
}

/// All 25 [`FrameSize`] variants in index order.
pub const ALL_FRAMESIZES: [FrameSize; FrameSize::COUNT] = [
    FrameSize::Size96x96,
    FrameSize::Qqvga,
    FrameSize::Size128x128,
    FrameSize::Qcif,
    FrameSize::Hqvga,
    FrameSize::Size240x240,
    FrameSize::Qvga,
    FrameSize::Size320x320,
    FrameSize::Cif,
    FrameSize::Hvga,
    FrameSize::Vga,
    FrameSize::Svga,
    FrameSize::Xga,
    FrameSize::Hd,
    FrameSize::Sxga,
    FrameSize::Uxga,
    FrameSize::Fhd,
    FrameSize::PHd,
    FrameSize::P3Mp,
    FrameSize::Qxga,
    FrameSize::Qhd,
    FrameSize::Wqxga,
    FrameSize::PFhd,
    FrameSize::Qsxga,
    FrameSize::Size5Mp,
];

/// Frame aspect ratio (`aspect_ratio_t` from `driver/include/sensor.h`).
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum AspectRatio {
    Ratio4x3 = 0,
    Ratio3x2 = 1,
    Ratio16x10 = 2,
    Ratio5x3 = 3,
    Ratio16x9 = 4,
    Ratio21x9 = 5,
    Ratio5x4 = 6,
    Ratio1x1 = 7,
    Ratio9x16 = 8,
}

#[allow(non_upper_case_globals)]
impl AspectRatio {
    pub const Ratio4X3: Self = Self::Ratio4x3;
    pub const Ratio3X2: Self = Self::Ratio3x2;
    pub const Ratio16X10: Self = Self::Ratio16x10;
    pub const Ratio5X3: Self = Self::Ratio5x3;
    pub const Ratio16X9: Self = Self::Ratio16x9;
    pub const Ratio21X9: Self = Self::Ratio21x9;
    pub const Ratio5X4: Self = Self::Ratio5x4;
    pub const Ratio1X1: Self = Self::Ratio1x1;
    pub const Ratio9X16: Self = Self::Ratio9x16;
}

/// Resolution dimensions and aspect ratio from `driver/include/sensor.h`.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ResolutionInfo {
    pub width: u16,
    pub height: u16,
    pub aspect_ratio: AspectRatio,
}

/// Master resolution table matching `resolution[FRAMESIZE_INVALID]`.
pub const RESOLUTION_TABLE: [ResolutionInfo; 25] = [
    // Size96x96
    ResolutionInfo {
        width: 96,
        height: 96,
        aspect_ratio: AspectRatio::Ratio1x1,
    },
    // Qqvga
    ResolutionInfo {
        width: 160,
        height: 120,
        aspect_ratio: AspectRatio::Ratio4x3,
    },
    // Size128x128
    ResolutionInfo {
        width: 128,
        height: 128,
        aspect_ratio: AspectRatio::Ratio1x1,
    },
    // Qcif
    ResolutionInfo {
        width: 176,
        height: 144,
        aspect_ratio: AspectRatio::Ratio5x4,
    },
    // Hqvga
    ResolutionInfo {
        width: 240,
        height: 176,
        aspect_ratio: AspectRatio::Ratio4x3,
    },
    // Size240x240
    ResolutionInfo {
        width: 240,
        height: 240,
        aspect_ratio: AspectRatio::Ratio1x1,
    },
    // Qvga
    ResolutionInfo {
        width: 320,
        height: 240,
        aspect_ratio: AspectRatio::Ratio4x3,
    },
    // Size320x320
    ResolutionInfo {
        width: 320,
        height: 320,
        aspect_ratio: AspectRatio::Ratio1x1,
    },
    // Cif
    ResolutionInfo {
        width: 400,
        height: 296,
        aspect_ratio: AspectRatio::Ratio4x3,
    },
    // Hvga
    ResolutionInfo {
        width: 480,
        height: 320,
        aspect_ratio: AspectRatio::Ratio3x2,
    },
    // Vga
    ResolutionInfo {
        width: 640,
        height: 480,
        aspect_ratio: AspectRatio::Ratio4x3,
    },
    // Svga
    ResolutionInfo {
        width: 800,
        height: 600,
        aspect_ratio: AspectRatio::Ratio4x3,
    },
    // Xga
    ResolutionInfo {
        width: 1024,
        height: 768,
        aspect_ratio: AspectRatio::Ratio4x3,
    },
    // Hd
    ResolutionInfo {
        width: 1280,
        height: 720,
        aspect_ratio: AspectRatio::Ratio16x9,
    },
    // Sxga
    ResolutionInfo {
        width: 1280,
        height: 1024,
        aspect_ratio: AspectRatio::Ratio5x4,
    },
    // Uxga
    ResolutionInfo {
        width: 1600,
        height: 1200,
        aspect_ratio: AspectRatio::Ratio4x3,
    },
    // 3MP Sensors: Fhd
    ResolutionInfo {
        width: 1920,
        height: 1080,
        aspect_ratio: AspectRatio::Ratio16x9,
    },
    // PHd
    ResolutionInfo {
        width: 720,
        height: 1280,
        aspect_ratio: AspectRatio::Ratio9x16,
    },
    // P3Mp
    ResolutionInfo {
        width: 864,
        height: 1536,
        aspect_ratio: AspectRatio::Ratio9x16,
    },
    // Qxga
    ResolutionInfo {
        width: 2048,
        height: 1536,
        aspect_ratio: AspectRatio::Ratio4x3,
    },
    // 5MP Sensors: Qhd
    ResolutionInfo {
        width: 2560,
        height: 1440,
        aspect_ratio: AspectRatio::Ratio16x9,
    },
    // Wqxga
    ResolutionInfo {
        width: 2560,
        height: 1600,
        aspect_ratio: AspectRatio::Ratio16x10,
    },
    // PFhd
    ResolutionInfo {
        width: 1088,
        height: 1920,
        aspect_ratio: AspectRatio::Ratio9x16,
    },
    // Qsxga
    ResolutionInfo {
        width: 2560,
        height: 1920,
        aspect_ratio: AspectRatio::Ratio4x3,
    },
    // Size5Mp
    ResolutionInfo {
        width: 2592,
        height: 1944,
        aspect_ratio: AspectRatio::Ratio4x3,
    },
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_resolution_table_entries() {
        assert_eq!(RESOLUTION_TABLE.len(), 25);

        let hd = FrameSize::Hd.resolution();
        assert_eq!(hd.width, 1280);
        assert_eq!(hd.height, 720);
        assert_eq!(hd.aspect_ratio, AspectRatio::Ratio16x9);
        assert_eq!(FrameSize::Hd.width(), 1280);
        assert_eq!(FrameSize::Hd.height(), 720);
        assert_eq!(FrameSize::Hd.aspect_ratio(), AspectRatio::Ratio16x9);

        let uxga = FrameSize::Uxga.resolution();
        assert_eq!(uxga.width, 1600);
        assert_eq!(uxga.height, 1200);
        assert_eq!(uxga.aspect_ratio, AspectRatio::Ratio4x3);

        let pfhd = FrameSize::PFhd.resolution();
        assert_eq!(pfhd.width, 1088);
        assert_eq!(pfhd.height, 1920);
        assert_eq!(pfhd.aspect_ratio, AspectRatio::Ratio9x16);

        let five_mp = FrameSize::Size5Mp.resolution();
        assert_eq!(five_mp.width, 2592);
        assert_eq!(five_mp.height, 1944);
        assert_eq!(five_mp.aspect_ratio, AspectRatio::Ratio4x3);
    }

    #[test]
    fn test_code_types_001_pixel_count_and_checked_byte_size() {
        let qvga = FrameSize::Qvga;
        assert_eq!(qvga.width(), 320);
        assert_eq!(qvga.height(), 240);
        assert_eq!(qvga.pixel_count(), 320 * 240);

        assert_eq!(qvga.checked_byte_size(PixelFormat::Jpeg), None);
        assert_eq!(qvga.checked_byte_size(PixelFormat::Grayscale), Some(320 * 240));
        assert_eq!(qvga.checked_byte_size(PixelFormat::Raw), Some(320 * 240));
        assert_eq!(qvga.checked_byte_size(PixelFormat::Raw8), Some(320 * 240));
        assert_eq!(qvga.checked_byte_size(PixelFormat::Rgb565), Some(320 * 240 * 2));
        assert_eq!(qvga.checked_byte_size(PixelFormat::Yuv422), Some(320 * 240 * 2));
        assert_eq!(qvga.checked_byte_size(PixelFormat::Rgb888), Some(320 * 240 * 3));
        assert_eq!(qvga.checked_byte_size(PixelFormat::Yuv420), Some((320 * 240 * 3) / 2));
        assert_eq!(qvga.checked_byte_size(PixelFormat::Rgb444), Some(320 * 240 * 2));
        assert_eq!(qvga.checked_byte_size(PixelFormat::Rgb555), Some(320 * 240 * 2));

        assert_eq!(FrameSize::from_u8(0), Some(FrameSize::Size96x96));
        assert_eq!(FrameSize::from_u8(6), Some(FrameSize::Qvga));
        assert_eq!(FrameSize::from_u8(24), Some(FrameSize::Size5Mp));
        assert_eq!(FrameSize::from_u8(25), None);
        assert_eq!(FrameSize::from_u8(255), None);

        const CONST_PIXELS: usize = FrameSize::Vga.pixel_count();
        assert_eq!(CONST_PIXELS, 640 * 480);
        const CONST_BYTES: Option<usize> = FrameSize::Vga.checked_byte_size(PixelFormat::Rgb565);
        assert_eq!(CONST_BYTES, Some(640 * 480 * 2));
    }
}
