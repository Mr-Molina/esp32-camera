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

#[cfg(feature = "bf20a6")]
pub mod bf20a6;
#[cfg(feature = "bf3005")]
pub mod bf3005;
#[cfg(feature = "gc0308")]
pub mod gc0308;
#[cfg(feature = "gc032a")]
pub mod gc032a;
#[cfg(feature = "gc2145")]
pub mod gc2145;
#[cfg(feature = "hm0360")]
pub mod hm0360;
#[cfg(feature = "hm1055")]
pub mod hm1055;
#[cfg(feature = "mega_ccm")]
pub mod mega_ccm;
#[cfg(feature = "nt99141")]
pub mod nt99141;
pub mod omnivision;
#[cfg(feature = "ov2640")]
pub mod ov2640;
#[cfg(feature = "ov3640")]
pub mod ov3640;
#[cfg(feature = "ov3660")]
pub mod ov3660;
#[cfg(feature = "ov5640")]
pub mod ov5640;
#[cfg(feature = "ov7670")]
pub mod ov7670;
#[cfg(feature = "ov7725")]
pub mod ov7725;
pub mod registry;
#[cfg(feature = "sc030iot")]
pub mod sc030iot;
#[cfg(feature = "sc031gs")]
pub mod sc031gs;
#[cfg(feature = "sc101iot")]
pub mod sc101iot;
pub mod traits;
pub mod vendors;

pub use registry::*;
pub use traits::*;
