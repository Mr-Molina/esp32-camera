//! Output pixel format types and format constants.

/// Output pixel format (`pixformat_t` from `driver/include/sensor.h`).
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum PixelFormat {
    #[default]
    Rgb565 = 0,
    Yuv422 = 1,
    Yuv420 = 2,
    Grayscale = 3,
    Jpeg = 4,
    Rgb888 = 5,
    Raw = 6,
    Rgb444 = 7,
    Rgb555 = 8,
    Raw8 = 9,
}

#[allow(non_upper_case_globals)]
impl PixelFormat {
    /// Total number of [`PixelFormat`] variants (`0..10`).
    pub const COUNT: usize = 10;

    pub const RGB565: Self = Self::Rgb565;
    pub const YUV422: Self = Self::Yuv422;
    pub const YUV420: Self = Self::Yuv420;
    pub const GRAYSCALE: Self = Self::Grayscale;
    pub const JPEG: Self = Self::Jpeg;
    pub const RGB888: Self = Self::Rgb888;
    pub const RAW: Self = Self::Raw;
    pub const RGB444: Self = Self::Rgb444;
    pub const RGB555: Self = Self::Rgb555;
    pub const RAW8: Self = Self::Raw8;

    /// Returns the raw `u8` discriminant (`0..10`) for this [`PixelFormat`].
    #[must_use]
    #[inline]
    pub const fn as_u8(self) -> u8 {
        self as u8
    }

    /// Returns the table index (`0..10`) for this [`PixelFormat`].
    #[must_use]
    #[inline]
    pub const fn index(self) -> usize {
        self as usize
    }

    /// Converts a raw discriminant (`0..10`) into a [`PixelFormat`].
    #[must_use]
    #[inline]
    pub const fn from_u8(val: u8) -> Option<Self> {
        match val {
            0 => Some(Self::Rgb565),
            1 => Some(Self::Yuv422),
            2 => Some(Self::Yuv420),
            3 => Some(Self::Grayscale),
            4 => Some(Self::Jpeg),
            5 => Some(Self::Rgb888),
            6 => Some(Self::Raw),
            7 => Some(Self::Rgb444),
            8 => Some(Self::Rgb555),
            9 => Some(Self::Raw8),
            _ => None,
        }
    }

    /// Returns `true` if this format uses variable-length compression
    /// ([`Self::Jpeg`]).
    #[must_use]
    #[inline]
    pub const fn is_compressed(self) -> bool {
        matches!(self, Self::Jpeg)
    }

    /// Returns the uncompressed bit depth per pixel, or `None` for variable-size
    /// compressed formats ([`Self::Jpeg`]).
    #[must_use]
    #[inline]
    pub const fn bits_per_pixel(self) -> Option<u8> {
        match self {
            Self::Grayscale | Self::Raw | Self::Raw8 => Some(8),
            Self::Yuv420 => Some(12),
            Self::Rgb565 | Self::Yuv422 | Self::Rgb444 | Self::Rgb555 => Some(16),
            Self::Rgb888 => Some(24),
            Self::Jpeg => None,
        }
    }
}

/// All 10 [`PixelFormat`] variants in discriminant order.
pub static ALL_PIXEL_FORMATS: [PixelFormat; PixelFormat::COUNT] = [
    PixelFormat::Rgb565,
    PixelFormat::Yuv422,
    PixelFormat::Yuv420,
    PixelFormat::Grayscale,
    PixelFormat::Jpeg,
    PixelFormat::Rgb888,
    PixelFormat::Raw,
    PixelFormat::Rgb444,
    PixelFormat::Rgb555,
    PixelFormat::Raw8,
];

const _: () = {
    let mut i = 0;
    while i < ALL_PIXEL_FORMATS.len() {
        assert!(ALL_PIXEL_FORMATS[i] as usize == i);
        i += 1;
    }
};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pixel_format_const_conversions_and_order() {
        const JPEG_FROM_U8: Option<PixelFormat> = PixelFormat::from_u8(4);
        const JPEG_AS_U8: u8 = PixelFormat::Jpeg.as_u8();
        const JPEG_COMPRESSED: bool = PixelFormat::Jpeg.is_compressed();
        const JPEG_BPP: Option<u8> = PixelFormat::Jpeg.bits_per_pixel();
        const RGB565_BPP: Option<u8> = PixelFormat::Rgb565.bits_per_pixel();
        const YUV420_BPP: Option<u8> = PixelFormat::Yuv420.bits_per_pixel();
        const RGB888_BPP: Option<u8> = PixelFormat::Rgb888.bits_per_pixel();
        const GRAY_BPP: Option<u8> = PixelFormat::Grayscale.bits_per_pixel();

        const {
            assert!(JPEG_COMPRESSED);
            assert!(!PixelFormat::Rgb565.is_compressed());
        }
        assert_eq!(JPEG_FROM_U8, Some(PixelFormat::Jpeg));
        assert_eq!(JPEG_AS_U8, 4);
        assert!(PixelFormat::Jpeg.is_compressed());
        assert_eq!(JPEG_BPP, None);
        assert_eq!(RGB565_BPP, Some(16));
        assert_eq!(YUV420_BPP, Some(12));
        assert_eq!(RGB888_BPP, Some(24));
        assert_eq!(GRAY_BPP, Some(8));

        assert_eq!(ALL_PIXEL_FORMATS.len(), PixelFormat::COUNT);
        for (idx, &fmt) in ALL_PIXEL_FORMATS.iter().enumerate() {
            let raw = idx as u8;
            assert_eq!(fmt.index(), idx);
            assert_eq!(fmt.as_u8(), raw);
            assert_eq!(PixelFormat::from_u8(raw), Some(fmt));
            assert_eq!(fmt.is_compressed(), fmt == PixelFormat::Jpeg);
        }

        for raw in (PixelFormat::COUNT as u8)..=u8::MAX {
            assert_eq!(PixelFormat::from_u8(raw), None);
        }
    }
}
