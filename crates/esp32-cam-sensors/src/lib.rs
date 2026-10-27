//! Frozen [`CameraSensor`] trait contract, register table primitives
//! ([`Reg8`], [`Reg16`]), and safe `#![no_std]` sensor driver modules
//! for all 18 `esp32-camera` sensors.

#![no_std]
#![forbid(unsafe_code)]

pub use esp32_cam_sccb::{EmbeddedHalSccb, MockSccbBus, SccbBus, SccbTransaction};
pub use esp32_cam_types::{
    esp_camera_sensor_get_info, AspectRatio, CameraConvMode, CameraError, CameraFbLocation,
    CameraGrabMode, CameraModel, CameraPid, CameraSccbAddr, CameraStatus, FrameSize, GainCeiling,
    PixelFormat, RatioSettings, ResolutionInfo, SensorId, SensorInfo, CAMERA_SENSOR_INFO,
    RESOLUTION_TABLE,
};

pub mod bf20a6;
pub mod bf3005;
pub mod gc0308;
pub mod gc032a;
pub mod gc2145;
pub mod hm0360;
pub mod hm1055;
pub mod mega_ccm;
pub mod nt99141;
pub mod omnivision;
pub mod ov2640;
pub mod ov3640;
pub mod ov3660;
pub mod ov5640;
pub mod ov7670;
pub mod ov7725;
pub mod registry;
pub mod sc030iot;
pub mod sc031gs;
pub mod sc101iot;
pub mod traits;
pub mod vendors;

pub use registry::*;
pub use traits::*;
