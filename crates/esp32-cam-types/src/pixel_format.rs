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
}
