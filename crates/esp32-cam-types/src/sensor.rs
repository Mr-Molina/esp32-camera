//! Sensor PIDs, models, SCCB addresses, and static sensor metadata table.

use crate::frame_size::FrameSize;

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

    /// Returns the canonical numeric 16-bit PID value.
    #[must_use]
    #[inline]
    pub const fn as_u16(self) -> u16 {
        self as u16
    }

    /// Returns the 16-bit comparison mask used when matching raw hardware PIDs.
    ///
    /// Most sensors match all 16 bits (`0xFFFF`); [`Self::OV3640`] masks out
    /// the lower revision nibble (`0xFFF0`) to accept `0x3640..=0x364F`.
    #[must_use]
    #[inline]
    pub const fn pid_mask(self) -> u16 {
        match self {
            Self::OV3640 => 0xFFF0,
            _ => 0xFFFF,
        }
    }

    /// Returns `true` if `raw` matches this sensor's hardware PID, honoring
    /// family revision masks (such as `0x3640..=0x364F` for [`Self::OV3640`]).
    ///
    /// Unlike `self.as_u16() == raw`, which checks exact canonical discriminant
    /// equality, `matches_pid` applies the same range-aware semantics as
    /// [`Self::from_u16`].
    #[must_use]
    #[inline]
    pub const fn matches_pid(self, raw: u16) -> bool {
        (raw & self.pid_mask()) == self.as_u16()
    }

    /// Attempts to decode a 16-bit PID into a known [`CameraPid`].
    #[must_use]
    #[inline]
    pub const fn from_u16(pid: u16) -> Option<Self> {
        match pid {
            0x96 => Some(Self::OV9650),
            0x77 => Some(Self::OV7725),
            0x26 => Some(Self::OV2640),
            0x3660 => Some(Self::OV3660),
            0x3640..=0x364F => Some(Self::OV3640),
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

/// Supported camera sensor models (`camera_model_t` in `sensor.h`).
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

    /// Returns the raw `u8` discriminant (`0..18`) for this [`CameraModel`].
    #[must_use]
    #[inline]
    pub const fn as_u8(self) -> u8 {
        self as u8
    }

    /// Returns the table index (`0..18`) into [`CAMERA_SENSOR_INFO`].
    #[must_use]
    #[inline]
    pub const fn index(self) -> usize {
        self as usize
    }

    /// Returns static sensor metadata for this model from [`CAMERA_SENSOR_INFO`].
    #[must_use]
    #[inline]
    pub const fn info(self) -> &'static SensorInfo {
        &CAMERA_SENSOR_INFO[self as usize]
    }

    /// Converts a raw model index (`0..18`) into a [`CameraModel`].
    #[must_use]
    #[inline]
    pub const fn from_u8(val: u8) -> Option<Self> {
        match val {
            0 => Some(Self::Ov7725),
            1 => Some(Self::Ov2640),
            2 => Some(Self::Ov3660),
            3 => Some(Self::Ov3640),
            4 => Some(Self::Ov5640),
            5 => Some(Self::Ov7670),
            6 => Some(Self::Nt99141),
            7 => Some(Self::Gc2145),
            8 => Some(Self::Gc032a),
            9 => Some(Self::Gc0308),
            10 => Some(Self::Bf3005),
            11 => Some(Self::Bf20a6),
            12 => Some(Self::Sc101Iot),
            13 => Some(Self::Sc030Iot),
            14 => Some(Self::Sc031Gs),
            15 => Some(Self::MegaCcm),
            16 => Some(Self::Hm1055),
            17 => Some(Self::Hm0360),
            _ => None,
        }
    }
}

/// 7-bit SCCB / I2C slave addresses (`camera_sccb_addr_t` in `sensor.h`).
///
/// Because multiple sensor models share physical 7-bit I2C addresses
/// (e.g. `0x3C`, `0x21`, `0x30`, `0x6E`, `0x68`), these are exposed as
/// `u8` constants on [`CameraSccbAddr`].
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

/// Static metadata for a supported camera sensor (`camera_sensor_info_t`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct SensorInfo {
    pub model: CameraModel,
    pub name: &'static str,
    pub sccb_addr: u8,
    pub pid: CameraPid,
    pub max_size: FrameSize,
    pub support_jpeg: bool,
}

const SENSOR_INFO_TABLE: [SensorInfo; CameraModel::COUNT] = [
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

/// Master table of all 18 camera sensors (`camera_sensor[CAMERA_MODEL_MAX]`).
pub static CAMERA_SENSOR_INFO: [SensorInfo; 18] = SENSOR_INFO_TABLE;

/// Alias for [`CAMERA_SENSOR_INFO`].
pub static SENSOR_INFO: [SensorInfo; 18] = SENSOR_INFO_TABLE;

const _: () = {
    let mut i = 0;
    while i < SENSOR_INFO_TABLE.len() {
        assert!(SENSOR_INFO_TABLE[i].model as usize == i);
        i += 1;
    }
};

/// Sensor hardware identification registers (`sensor_id_t` in `sensor.h`).
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Hash)]
pub struct SensorId {
    pub midh: u8,
    pub midl: u8,
    pub pid: u16,
    pub ver: u8,
}

/// Looks up [`SensorInfo`] by 16-bit sensor PID (`esp_camera_sensor_get_info`).
#[must_use]
pub const fn esp_camera_sensor_get_info(id: &SensorId) -> Option<&'static SensorInfo> {
    let mut i = 0;
    while i < SENSOR_INFO_TABLE.len() {
        if SENSOR_INFO_TABLE[i].pid.matches_pid(id.pid) {
            return Some(&CAMERA_SENSOR_INFO[i]);
        }
        i += 1;
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_types04_matches_pid_semantics_and_const_eval() {
        const OV3640_CANONICAL_U16: u16 = CameraPid::OV3640.as_u16();
        const OV3640_MATCHES_364C: bool = CameraPid::OV3640.matches_pid(0x364C);
        const OV3640_MATCHES_3640: bool = CameraPid::OV3640.matches_pid(0x3640);
        const OV3640_MATCHES_364F: bool = CameraPid::OV3640.matches_pid(0x364F);
        const OV3640_MATCHES_3650: bool = CameraPid::OV3640.matches_pid(0x3650);
        const OV2640_MATCHES_0026: bool = CameraPid::OV2640.matches_pid(0x0026);
        const OV2640_MATCHES_364C: bool = CameraPid::OV2640.matches_pid(0x364C);

        const {
            assert!(OV3640_MATCHES_364C);
            assert!(OV3640_MATCHES_3640);
            assert!(OV3640_MATCHES_364F);
            assert!(!OV3640_MATCHES_3650);
            assert!(OV2640_MATCHES_0026);
            assert!(!OV2640_MATCHES_364C);
        }

        assert_eq!(OV3640_CANONICAL_U16, 0x3640);
        assert_ne!(OV3640_CANONICAL_U16, 0x364C);
        assert!(CameraPid::OV3640.matches_pid(0x364C));
        assert!(CameraPid::OV3640.matches_pid(0x3640));
        assert!(CameraPid::OV3640.matches_pid(0x364F));
        assert!(!CameraPid::OV3640.matches_pid(0x3650));
        assert!(CameraPid::OV2640.matches_pid(0x0026));
        assert!(!CameraPid::OV2640.matches_pid(0x364C));
    }

    #[test]
    fn test_camera_model_from_u8_and_const_eval() {
        assert_eq!(CameraModel::from_u8(0), Some(CameraModel::Ov7725));
        assert_eq!(CameraModel::from_u8(1), Some(CameraModel::Ov2640));
        assert_eq!(CameraModel::from_u8(17), Some(CameraModel::Hm0360));
        assert_eq!(CameraModel::from_u8(18), None);
        assert_eq!(CameraModel::from_u8(255), None);

        const MODEL_OV2640: Option<CameraModel> = CameraModel::from_u8(1);
        const MODEL_OV2640_U8: u8 = CameraModel::Ov2640.as_u8();
        const MODEL_OV2640_IDX: usize = CameraModel::Ov2640.index();
        assert_eq!(MODEL_OV2640, Some(CameraModel::Ov2640));
        assert_eq!(MODEL_OV2640_U8, 1);
        assert_eq!(MODEL_OV2640_IDX, 1);
    }

    #[test]
    fn test_camera_sensor_info_table_and_order_invariant() {
        assert_eq!(CAMERA_SENSOR_INFO.len(), CameraModel::COUNT);
        assert_eq!(SENSOR_INFO.len(), CameraModel::COUNT);

        const CONST_OV2640_INFO: &SensorInfo = CameraModel::Ov2640.info();
        const CONST_OV3640_LOOKUP: Option<&SensorInfo> = esp_camera_sensor_get_info(&SensorId {
            midh: 0,
            midl: 0,
            pid: 0x364C,
            ver: 0x0C,
        });
        assert_eq!(CONST_OV2640_INFO.model, CameraModel::Ov2640);
        assert_eq!(
            CONST_OV3640_LOOKUP.map(|info| info.model),
            Some(CameraModel::Ov3640)
        );

        let ov2640 = CameraModel::Ov2640.info();
        assert_eq!(ov2640.model, CameraModel::Ov2640);
        assert_eq!(ov2640.name, "OV2640");
        assert_eq!(ov2640.sccb_addr, 0x30);
        assert_eq!(ov2640.pid.as_u16(), 0x26);
        assert!(ov2640.pid.matches_pid(0x26));
        assert_eq!(ov2640.pid, CameraPid::OV2640);
        assert_eq!(ov2640.max_size, FrameSize::Uxga);
        assert!(ov2640.support_jpeg);

        for (idx, entry) in CAMERA_SENSOR_INFO.iter().enumerate() {
            assert_eq!(entry.model.index(), idx);
            assert_eq!(entry.model.as_u8() as usize, idx);
            assert_eq!(CameraModel::from_u8(idx as u8), Some(entry.model));
            assert!(core::ptr::eq(entry.model.info(), entry));

            let id = SensorId {
                midh: 0,
                midl: 0,
                pid: entry.pid.as_u16(),
                ver: 0,
            };
            let found = esp_camera_sensor_get_info(&id).expect("known sensor PID must resolve");
            assert_eq!(found, entry);
            assert!(core::ptr::eq(found, entry));
        }

        // OV3640 revision PIDs like 0x364C resolve to the static OV3640 entry.
        let rev_id = SensorId {
            midh: 0,
            midl: 0,
            pid: 0x364C,
            ver: 0x0C,
        };
        let info = esp_camera_sensor_get_info(&rev_id).expect("OV3640 revision PID must resolve");
        assert_eq!(info.model, CameraModel::Ov3640);
        assert!(core::ptr::eq(
            info,
            &CAMERA_SENSOR_INFO[CameraModel::Ov3640.index()]
        ));

        // OV9650 (0x96) has no CameraModel entry in CAMERA_SENSOR_INFO.
        let ov9650_id = SensorId {
            midh: 0x7F,
            midl: 0xA2,
            pid: CameraPid::OV9650.as_u16(),
            ver: 0,
        };
        assert_eq!(esp_camera_sensor_get_info(&ov9650_id), None);
    }

    #[test]
    fn test_camera_pid_roundtrip_and_exhaustive_mask_equivalence() {
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
            assert!(pid.matches_pid(pid.as_u16()));
        }
        assert_eq!(CameraPid::from_u16(0xFFFF), None);

        for raw in 0u16..=u16::MAX {
            let decoded = CameraPid::from_u16(raw);
            for pid in all_pids {
                assert_eq!(
                    pid.matches_pid(raw),
                    decoded == Some(pid),
                    "mismatch for pid={pid:?}, raw=0x{raw:04X}"
                );
            }
        }
    }
}
