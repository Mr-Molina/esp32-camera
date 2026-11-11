//! Core `#![no_std]` types, sensor PIDs, SCCB addresses, frame sizes,
//! resolution tables, and error types for the `esp32-camera` driver stack.

#![no_std]
#![forbid(unsafe_code)]

pub mod control;
pub mod error;
pub mod frame_size;
pub mod pixel_format;
pub mod sensor;

pub use crate::control::{
    clamp_level, CameraConvMode, CameraFbLocation, CameraGrabMode, CameraStatus, ConvMode,
    FbLocation, GainCeiling, GrabMode, RatioSettings,
};
pub use crate::error::{CameraError, SccbPhase};
pub use crate::frame_size::{
    AspectRatio, FrameSize, ResolutionInfo, ALL_FRAMESIZES, RESOLUTION_TABLE,
};
pub use crate::pixel_format::{PixelFormat, ALL_PIXEL_FORMATS};
pub use crate::sensor::{
    esp_camera_sensor_get_info, CameraModel, CameraPid, CameraSccbAddr, SensorId, SensorInfo,
    CAMERA_SENSOR_INFO, SENSOR_INFO,
};
