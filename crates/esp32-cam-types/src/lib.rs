//! Core `#![no_std]` types, sensor PIDs, SCCB addresses, frame sizes, resolution tables,
//! and error types for the `esp32-camera` driver stack.

#![no_std]
#![forbid(unsafe_code)]

use core::fmt;

/// Camera sensor Product ID (`camera_pid_t` from `driver/include/sensor.h`).
#[repr(u16)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[allow(non_camel_case_types)]
pub enum CameraPid {
    OV9650 = 0x96,
    OV7725 = 0x77,
    OV2640 = 0x26,
    OV3660 = 0x3660,
    OV3640 = 0x3640,
    OV5640 = 0x5640,
    OV7670 = 0x76,
    NT99141 = 0x1410,
    GC2145 = 0x2145,
    GC032A = 0x232a,
    GC0308 = 0x9b,
    BF3005 = 0x30,
    BF20A6 = 0x20a6,
    SC101IOT = 0xda4a,
    SC030IOT = 0x9a46,
    SC031GS = 0x0031,
    MEGA_CCM = 0x039E,
    HM1055 = 0x0955,
    HM0360 = 0x0360,
}

#[allow(non_upper_case_globals)]
impl CameraPid {
    pub const Ov9650: Self = Self::OV9650;
    pub const Ov7725: Self = Self::OV7725;
    pub const Ov2640: Self = Self::OV2640;
    pub const Ov3660: Self = Self::OV3660;
    pub const Ov3640: Self = Self::OV3640;
    pub const Ov5640: Self = Self::OV5640;
    pub const Ov7670: Self = Self::OV7670;
    pub const Nt99141: Self = Self::NT99141;
    pub const Gc2145: Self = Self::GC2145;
    pub const Gc032a: Self = Self::GC032A;
    pub const Gc032A: Self = Self::GC032A;
    pub const Gc0308: Self = Self::GC0308;
    pub const Bf3005: Self = Self::BF3005;
    pub const Bf20a6: Self = Self::BF20A6;
    pub const Bf20A6: Self = Self::BF20A6;
    pub const Sc101Iot: Self = Self::SC101IOT;
    pub const Sc101iot: Self = Self::SC101IOT;
    pub const Sc030Iot: Self = Self::SC030IOT;
    pub const Sc030iot: Self = Self::SC030IOT;
    pub const Sc031Gs: Self = Self::SC031GS;
    pub const Sc031gs: Self = Self::SC031GS;
    pub const MegaCcm: Self = Self::MEGA_CCM;
    pub const Hm1055: Self = Self::HM1055;
    pub const Hm0360: Self = Self::HM0360;

    /// Returns the numeric 16-bit PID value.
    #[inline]
    pub const fn as_u16(self) -> u16 {
        self as u16
    }

    /// Attempts to decode a 16-bit PID into a known [`CameraPid`].
    #[inline]
    pub const fn from_u16(pid: u16) -> Option<Self> {
        match pid {
            0x96 => Some(Self::OV9650),
            0x77 => Some(Self::OV7725),
            0x26 => Some(Self::OV2640),
            0x3660 => Some(Self::OV3660),
            0x3640 => Some(Self::OV3640),
            0x5640 => Some(Self::OV5640),
            0x76 => Some(Self::OV7670),
            0x1410 => Some(Self::NT99141),
            0x2145 => Some(Self::GC2145),
            0x232a => Some(Self::GC032A),
            0x9b => Some(Self::GC0308),
            0x30 => Some(Self::BF3005),
            0x20a6 => Some(Self::BF20A6),
            0xda4a => Some(Self::SC101IOT),
            0x9a46 => Some(Self::SC030IOT),
            0x0031 => Some(Self::SC031GS),
            0x039E => Some(Self::MEGA_CCM),
            0x0955 => Some(Self::HM1055),
            0x0360 => Some(Self::HM0360),
            _ => None,
        }
    }
}

impl PartialEq<u16> for CameraPid {
    #[inline]
    fn eq(&self, other: &u16) -> bool {
        (*self as u16) == *other
    }
}

impl PartialEq<CameraPid> for u16 {
    #[inline]
    fn eq(&self, other: &CameraPid) -> bool {
        *self == (*other as u16)
    }
}

/// Supported camera sensor models (`camera_model_t` from `driver/include/sensor.h`).
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum CameraModel {
    Ov7725 = 0,
    Ov2640 = 1,
    Ov3660 = 2,
    Ov3640 = 3,
    Ov5640 = 4,
    Ov7670 = 5,
    Nt99141 = 6,
    Gc2145 = 7,
    Gc032a = 8,
    Gc0308 = 9,
    Bf3005 = 10,
    Bf20a6 = 11,
    Sc101Iot = 12,
    Sc030Iot = 13,
    Sc031Gs = 14,
    MegaCcm = 15,
    Hm1055 = 16,
    Hm0360 = 17,
}

#[allow(non_upper_case_globals)]
impl CameraModel {
    pub const COUNT: usize = 18;

    pub const OV7725: Self = Self::Ov7725;
    pub const OV2640: Self = Self::Ov2640;
    pub const OV3660: Self = Self::Ov3660;
    pub const OV3640: Self = Self::Ov3640;
    pub const OV5640: Self = Self::Ov5640;
    pub const OV7670: Self = Self::Ov7670;
    pub const NT99141: Self = Self::Nt99141;
    pub const GC2145: Self = Self::Gc2145;
    pub const GC032A: Self = Self::Gc032a;
    pub const Gc032A: Self = Self::Gc032a;
    pub const GC0308: Self = Self::Gc0308;
    pub const BF3005: Self = Self::Bf3005;
    pub const BF20A6: Self = Self::Bf20a6;
    pub const Bf20A6: Self = Self::Bf20a6;
    pub const SC101IOT: Self = Self::Sc101Iot;
    pub const Sc101iot: Self = Self::Sc101Iot;
    pub const SC030IOT: Self = Self::Sc030Iot;
    pub const Sc030iot: Self = Self::Sc030Iot;
    pub const SC031GS: Self = Self::Sc031Gs;
    pub const Sc031gs: Self = Self::Sc031Gs;
    pub const MEGA_CCM: Self = Self::MegaCcm;
    pub const HM1055: Self = Self::Hm1055;
    pub const HM0360: Self = Self::Hm0360;

    /// Returns static sensor metadata for this model from [`CAMERA_SENSOR_INFO`].
    #[inline]
    pub const fn info(self) -> &'static SensorInfo {
        &CAMERA_SENSOR_INFO[self as usize]
    }
}

/// 7-bit SCCB / I2C slave addresses (`camera_sccb_addr_t` from `driver/include/sensor.h`).
///
/// Because multiple sensor models share physical 7-bit I2C addresses (e.g. `0x3C`, `0x21`,
/// `0x30`, `0x6E`, `0x68`), these are exposed as `u8` constants on [`CameraSccbAddr`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct CameraSccbAddr;

#[allow(non_upper_case_globals)]
impl CameraSccbAddr {
    pub const OV2640: u8 = 0x30;
    pub const OV5640: u8 = 0x3C;
    pub const OV3660: u8 = 0x3C;
    pub const OV3640: u8 = 0x3C;
    pub const OV7725: u8 = 0x21;
    pub const OV7670: u8 = 0x21;
    pub const NT99141: u8 = 0x2A;
    pub const GC2145: u8 = 0x3C;
    pub const GC032A: u8 = 0x21;
    pub const GC0308: u8 = 0x21;
    pub const BF3005: u8 = 0x6E;
    pub const BF20A6: u8 = 0x6E;
    pub const SC101IOT: u8 = 0x68;
    pub const SC030IOT: u8 = 0x68;
    pub const SC031GS: u8 = 0x30;
    pub const MEGA_CCM: u8 = 0x1F;
    pub const HM1055: u8 = 0x24;
    pub const HM0360: u8 = 0x12;

    pub const Ov2640: u8 = Self::OV2640;
    pub const Ov5640: u8 = Self::OV5640;
    pub const Ov3660: u8 = Self::OV3660;
    pub const Ov3640: u8 = Self::OV3640;
    pub const Ov7725: u8 = Self::OV7725;
    pub const Ov7670: u8 = Self::OV7670;
    pub const Nt99141: u8 = Self::NT99141;
    pub const Gc2145: u8 = Self::GC2145;
    pub const Gc032a: u8 = Self::GC032A;
    pub const Gc032A: u8 = Self::GC032A;
    pub const Gc0308: u8 = Self::GC0308;
    pub const Bf3005: u8 = Self::BF3005;
    pub const Bf20a6: u8 = Self::BF20A6;
    pub const Bf20A6: u8 = Self::BF20A6;
    pub const Sc101Iot: u8 = Self::SC101IOT;
    pub const Sc101iot: u8 = Self::SC101IOT;
    pub const Sc030Iot: u8 = Self::SC030IOT;
    pub const Sc030iot: u8 = Self::SC030IOT;
    pub const Sc031Gs: u8 = Self::SC031GS;
    pub const Sc031gs: u8 = Self::SC031GS;
    pub const MegaCcm: u8 = Self::MEGA_CCM;
    pub const Hm1055: u8 = Self::HM1055;
    pub const Hm0360: u8 = Self::HM0360;
}

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

/// Output frame resolution preset (`framesize_t` from `driver/include/sensor.h`).
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

    /// Converts a raw index (`0..25`) into a [`FrameSize`].
    #[inline]
    pub const fn from_u8(val: u8) -> Option<Self> {
        match val {
            0 => Some(Self::Size96x96),
            1 => Some(Self::Qqvga),
            2 => Some(Self::Size128x128),
            3 => Some(Self::Qcif),
            4 => Some(Self::Hqvga),
            5 => Some(Self::Size240x240),
            6 => Some(Self::Qvga),
            7 => Some(Self::Size320x320),
            8 => Some(Self::Cif),
            9 => Some(Self::Hvga),
            10 => Some(Self::Vga),
            11 => Some(Self::Svga),
            12 => Some(Self::Xga),
            13 => Some(Self::Hd),
            14 => Some(Self::Sxga),
            15 => Some(Self::Uxga),
            16 => Some(Self::Fhd),
            17 => Some(Self::PHd),
            18 => Some(Self::P3Mp),
            19 => Some(Self::Qxga),
            20 => Some(Self::Qhd),
            21 => Some(Self::Wqxga),
            22 => Some(Self::PFhd),
            23 => Some(Self::Qsxga),
            24 => Some(Self::Size5Mp),
            _ => None,
        }
    }
}

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

/// Resolution dimensions and aspect ratio (`resolution_info_t` from `driver/include/sensor.h`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ResolutionInfo {
    pub width: u16,
    pub height: u16,
    pub aspect_ratio: AspectRatio,
}

/// Master resolution table matching `resolution[FRAMESIZE_INVALID]` in `driver/sensor.c`.
pub const RESOLUTION_TABLE: [ResolutionInfo; 25] = [
    ResolutionInfo { width: 96, height: 96, aspect_ratio: AspectRatio::Ratio1x1 },     // Size96x96
    ResolutionInfo { width: 160, height: 120, aspect_ratio: AspectRatio::Ratio4x3 },   // Qqvga
    ResolutionInfo { width: 128, height: 128, aspect_ratio: AspectRatio::Ratio1x1 },   // Size128x128
    ResolutionInfo { width: 176, height: 144, aspect_ratio: AspectRatio::Ratio5x4 },   // Qcif
    ResolutionInfo { width: 240, height: 176, aspect_ratio: AspectRatio::Ratio4x3 },   // Hqvga
    ResolutionInfo { width: 240, height: 240, aspect_ratio: AspectRatio::Ratio1x1 },   // Size240x240
    ResolutionInfo { width: 320, height: 240, aspect_ratio: AspectRatio::Ratio4x3 },   // Qvga
    ResolutionInfo { width: 320, height: 320, aspect_ratio: AspectRatio::Ratio1x1 },   // Size320x320
    ResolutionInfo { width: 400, height: 296, aspect_ratio: AspectRatio::Ratio4x3 },   // Cif
    ResolutionInfo { width: 480, height: 320, aspect_ratio: AspectRatio::Ratio3x2 },   // Hvga
    ResolutionInfo { width: 640, height: 480, aspect_ratio: AspectRatio::Ratio4x3 },   // Vga
    ResolutionInfo { width: 800, height: 600, aspect_ratio: AspectRatio::Ratio4x3 },   // Svga
    ResolutionInfo { width: 1024, height: 768, aspect_ratio: AspectRatio::Ratio4x3 },  // Xga
    ResolutionInfo { width: 1280, height: 720, aspect_ratio: AspectRatio::Ratio16x9 }, // Hd
    ResolutionInfo { width: 1280, height: 1024, aspect_ratio: AspectRatio::Ratio5x4 }, // Sxga
    ResolutionInfo { width: 1600, height: 1200, aspect_ratio: AspectRatio::Ratio4x3 }, // Uxga
    // 3MP Sensors
    ResolutionInfo { width: 1920, height: 1080, aspect_ratio: AspectRatio::Ratio16x9 }, // Fhd
    ResolutionInfo { width: 720, height: 1280, aspect_ratio: AspectRatio::Ratio9x16 },  // PHd
    ResolutionInfo { width: 864, height: 1536, aspect_ratio: AspectRatio::Ratio9x16 },  // P3Mp
    ResolutionInfo { width: 2048, height: 1536, aspect_ratio: AspectRatio::Ratio4x3 },  // Qxga
    // 5MP Sensors
    ResolutionInfo { width: 2560, height: 1440, aspect_ratio: AspectRatio::Ratio16x9 },  // Qhd
    ResolutionInfo { width: 2560, height: 1600, aspect_ratio: AspectRatio::Ratio16x10 }, // Wqxga
    ResolutionInfo { width: 1088, height: 1920, aspect_ratio: AspectRatio::Ratio9x16 },  // PFhd
    ResolutionInfo { width: 2560, height: 1920, aspect_ratio: AspectRatio::Ratio4x3 },  // Qsxga
    ResolutionInfo { width: 2592, height: 1944, aspect_ratio: AspectRatio::Ratio4x3 },  // Size5Mp
];

/// Static metadata for a supported camera sensor (`camera_sensor_info_t` from `driver/include/sensor.h`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct SensorInfo {
    pub model: CameraModel,
    pub name: &'static str,
    pub sccb_addr: u8,
    pub pid: CameraPid,
    pub max_size: FrameSize,
    pub support_jpeg: bool,
}

/// Master table of all 18 camera sensors matching `camera_sensor[CAMERA_MODEL_MAX]` in `driver/sensor.c`.
pub const CAMERA_SENSOR_INFO: [SensorInfo; 18] = [
    SensorInfo {
        model: CameraModel::Ov7725,
        name: "OV7725",
        sccb_addr: CameraSccbAddr::OV7725,
        pid: CameraPid::OV7725,
        max_size: FrameSize::Vga,
        support_jpeg: false,
    },
    SensorInfo {
        model: CameraModel::Ov2640,
        name: "OV2640",
        sccb_addr: CameraSccbAddr::OV2640,
        pid: CameraPid::OV2640,
        max_size: FrameSize::Uxga,
        support_jpeg: true,
    },
    SensorInfo {
        model: CameraModel::Ov3660,
        name: "OV3660",
        sccb_addr: CameraSccbAddr::OV3660,
        pid: CameraPid::OV3660,
        max_size: FrameSize::Qxga,
        support_jpeg: true,
    },
    SensorInfo {
        model: CameraModel::Ov3640,
        name: "OV3640",
        sccb_addr: CameraSccbAddr::OV3640,
        pid: CameraPid::OV3640,
        max_size: FrameSize::Qxga,
        support_jpeg: true,
    },
    SensorInfo {
        model: CameraModel::Ov5640,
        name: "OV5640",
        sccb_addr: CameraSccbAddr::OV5640,
        pid: CameraPid::OV5640,
        max_size: FrameSize::Qsxga,
        support_jpeg: true,
    },
    SensorInfo {
        model: CameraModel::Ov7670,
        name: "OV7670",
        sccb_addr: CameraSccbAddr::OV7670,
        pid: CameraPid::OV7670,
        max_size: FrameSize::Vga,
        support_jpeg: false,
    },
    SensorInfo {
        model: CameraModel::Nt99141,
        name: "NT99141",
        sccb_addr: CameraSccbAddr::NT99141,
        pid: CameraPid::NT99141,
        max_size: FrameSize::Hd,
        support_jpeg: true,
    },
    SensorInfo {
        model: CameraModel::Gc2145,
        name: "GC2145",
        sccb_addr: CameraSccbAddr::GC2145,
        pid: CameraPid::GC2145,
        max_size: FrameSize::Uxga,
        support_jpeg: false,
    },
    SensorInfo {
        model: CameraModel::Gc032a,
        name: "GC032A",
        sccb_addr: CameraSccbAddr::GC032A,
        pid: CameraPid::GC032A,
        max_size: FrameSize::Vga,
        support_jpeg: false,
    },
    SensorInfo {
        model: CameraModel::Gc0308,
        name: "GC0308",
        sccb_addr: CameraSccbAddr::GC0308,
        pid: CameraPid::GC0308,
        max_size: FrameSize::Vga,
        support_jpeg: false,
    },
    SensorInfo {
        model: CameraModel::Bf3005,
        name: "BF3005",
        sccb_addr: CameraSccbAddr::BF3005,
        pid: CameraPid::BF3005,
        max_size: FrameSize::Vga,
        support_jpeg: false,
    },
    SensorInfo {
        model: CameraModel::Bf20a6,
        name: "BF20A6",
        sccb_addr: CameraSccbAddr::BF20A6,
        pid: CameraPid::BF20A6,
        max_size: FrameSize::Vga,
        support_jpeg: false,
    },
    SensorInfo {
        model: CameraModel::Sc101Iot,
        name: "SC101IOT",
        sccb_addr: CameraSccbAddr::SC101IOT,
        pid: CameraPid::SC101IOT,
        max_size: FrameSize::Hd,
        support_jpeg: false,
    },
    SensorInfo {
        model: CameraModel::Sc030Iot,
        name: "SC030IOT",
        sccb_addr: CameraSccbAddr::SC030IOT,
        pid: CameraPid::SC030IOT,
        max_size: FrameSize::Vga,
        support_jpeg: false,
    },
    SensorInfo {
        model: CameraModel::Sc031Gs,
        name: "SC031GS",
        sccb_addr: CameraSccbAddr::SC031GS,
        pid: CameraPid::SC031GS,
        max_size: FrameSize::Vga,
        support_jpeg: false,
    },
    SensorInfo {
        model: CameraModel::MegaCcm,
        name: "MEGA_CCM",
        sccb_addr: CameraSccbAddr::MEGA_CCM,
        pid: CameraPid::MEGA_CCM,
        max_size: FrameSize::Size5Mp,
        support_jpeg: true,
    },
    SensorInfo {
        model: CameraModel::Hm1055,
        name: "HM1055",
        sccb_addr: CameraSccbAddr::HM1055,
        pid: CameraPid::HM1055,
        max_size: FrameSize::Hd,
        support_jpeg: false,
    },
    SensorInfo {
        model: CameraModel::Hm0360,
        name: "HM0360",
        sccb_addr: CameraSccbAddr::HM0360,
        pid: CameraPid::HM0360,
        max_size: FrameSize::Vga,
        support_jpeg: false,
    },
];

/// Looks up [`SensorInfo`] by 16-bit sensor PID (`esp_camera_sensor_get_info` in `driver/sensor.c`).
pub fn esp_camera_sensor_get_info(id: &SensorId) -> Option<&'static SensorInfo> {
    CAMERA_SENSOR_INFO
        .iter()
        .find(|info| info.pid.as_u16() == id.pid)
}

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

/// Sensor window and aspect ratio timing settings (`ratio_settings_t` from `driver/include/sensor.h`).
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

/// Sensor hardware identification registers (`sensor_id_t` from `driver/include/sensor.h`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Hash)]
pub struct SensorId {
    pub midh: u8,
    pub midl: u8,
    pub pid: u16,
    pub ver: u8,
}

/// Mutable camera control state (`camera_status_t` from `driver/include/sensor.h`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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

impl Default for CameraStatus {
    fn default() -> Self {
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
}

/// Frame buffer acquisition policy (`camera_grab_mode_t` from `driver/include/esp_camera.h`).
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Hash)]
pub enum CameraGrabMode {
    #[default]
    WhenEmpty = 0,
    Latest = 1,
}

/// Alias for [`CameraGrabMode`].
pub type GrabMode = CameraGrabMode;

/// Frame buffer memory allocation target (`camera_fb_location_t` from `driver/include/esp_camera.h`).
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Hash)]
pub enum CameraFbLocation {
    #[default]
    Psram = 0,
    Dram = 1,
}

/// Alias for [`CameraFbLocation`].
pub type FbLocation = CameraFbLocation;

/// Hardware/software color conversion mode (`camera_conv_mode_t` from `driver/include/esp_camera.h`).
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

/// Unified error type for the `esp32-camera` Rust workspace.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CameraError {
    /// Camera sensor was not detected on the SCCB/I2C bus (`ESP_ERR_CAMERA_NOT_DETECTED`).
    NotDetected,
    /// Failed to configure the requested frame size (`ESP_ERR_CAMERA_FAILED_TO_SET_FRAME_SIZE`).
    FailedToSetFrameSize,
    /// Failed to configure the requested pixel format (`ESP_ERR_CAMERA_FAILED_TO_SET_OUT_FORMAT`).
    FailedToSetOutFormat,
    /// Operation or control is not supported by this sensor (`ESP_ERR_CAMERA_NOT_SUPPORTED`).
    NotSupported,
    /// Invalid argument or out-of-range parameter (`ESP_ERR_INVALID_ARG`).
    InvalidArg,
    /// Driver or peripheral is in an invalid state for the requested operation (`ESP_ERR_INVALID_STATE`).
    InvalidState,
    /// Insufficient memory or caller buffer capacity (`ESP_ERR_NO_MEM`).
    NoMem,
    /// Destination buffer is too small to hold the output.
    BufferTooSmall,
    /// Arithmetic overflow detected during dimension or buffer calculation.
    Overflow,
    /// Underlying SCCB / I2C bus transaction failed.
    SccbError,
    /// Underlying I2C error.
    I2cError,
    /// Operation timed out waiting for hardware or sensor status.
    Timeout,
    /// Captured frame or image data is corrupted (e.g. missing JPEG SOI/EOI markers).
    CorruptedFrame,
    /// Invalid or unsupported image format.
    InvalidFormat,
    /// Image encoder/decoder failure.
    CodecError,
    /// Low-level peripheral or hardware configuration error.
    HardwareError,
}

impl fmt::Display for CameraError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NotDetected => write!(f, "camera sensor not detected"),
            Self::FailedToSetFrameSize => write!(f, "failed to set frame size"),
            Self::FailedToSetOutFormat => write!(f, "failed to set output pixel format"),
            Self::NotSupported => write!(f, "operation not supported by sensor"),
            Self::InvalidArg => write!(f, "invalid argument"),
            Self::InvalidState => write!(f, "invalid driver state"),
            Self::NoMem => write!(f, "out of memory"),
            Self::BufferTooSmall => write!(f, "destination buffer too small"),
            Self::Overflow => write!(f, "integer overflow"),
            Self::SccbError => write!(f, "SCCB bus transaction error"),
            Self::I2cError => write!(f, "I2C bus error"),
            Self::Timeout => write!(f, "operation timed out"),
            Self::CorruptedFrame => write!(f, "corrupted frame data"),
            Self::InvalidFormat => write!(f, "invalid pixel or image format"),
            Self::CodecError => write!(f, "image codec error"),
            Self::HardwareError => write!(f, "hardware peripheral error"),
        }
    }
}

impl core::error::Error for CameraError {}

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
    fn test_camera_sensor_info_table() {
        assert_eq!(CAMERA_SENSOR_INFO.len(), 18);

        let ov2640 = CameraModel::Ov2640.info();
        assert_eq!(ov2640.model, CameraModel::Ov2640);
        assert_eq!(ov2640.name, "OV2640");
        assert_eq!(ov2640.sccb_addr, 0x30);
        assert_eq!(ov2640.pid, 0x26);
        assert_eq!(ov2640.pid, CameraPid::OV2640);
        assert_eq!(ov2640.max_size, FrameSize::Uxga);
        assert!(ov2640.support_jpeg);

        for (idx, entry) in CAMERA_SENSOR_INFO.iter().enumerate() {
            assert_eq!(entry.model as usize, idx);
            let id = SensorId {
                midh: 0,
                midl: 0,
                pid: entry.pid.as_u16(),
                ver: 0,
            };
            let found = esp_camera_sensor_get_info(&id);
            assert_eq!(found, Some(entry));
        }
    }

    #[test]
    fn test_camera_pid_roundtrip() {
        let all_pids = [
            CameraPid::OV9650,
            CameraPid::OV7725,
            CameraPid::OV2640,
            CameraPid::OV3660,
            CameraPid::OV3640,
            CameraPid::OV5640,
            CameraPid::OV7670,
            CameraPid::NT99141,
            CameraPid::GC2145,
            CameraPid::GC032A,
            CameraPid::GC0308,
            CameraPid::BF3005,
            CameraPid::BF20A6,
            CameraPid::SC101IOT,
            CameraPid::SC030IOT,
            CameraPid::SC031GS,
            CameraPid::MEGA_CCM,
            CameraPid::HM1055,
            CameraPid::HM0360,
        ];
        assert_eq!(all_pids.len(), 19);
        for pid in all_pids {
            assert_eq!(CameraPid::from_u16(pid.as_u16()), Some(pid));
        }
        assert_eq!(CameraPid::from_u16(0xFFFF), None);
    }

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
}
