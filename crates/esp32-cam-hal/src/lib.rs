//! Safe `#![no_std]` type-state camera HAL, board pinout profiles, and RAII [`FramePool`].

#![no_std]
#![forbid(unsafe_code)]

pub use esp32_cam_types::{
    CameraConvMode, CameraError, CameraFbLocation, CameraGrabMode, CameraModel, CameraPid,
    CameraStatus, FrameSize, GrabMode, PixelFormat,
};

pub mod board_pins;
pub mod driver;
pub mod frame_pool;

pub use board_pins::{
    AiThinkerPins, BoardPins, CameraPins, Esp32S3GoouuuPins, Esp32S3WroomPins, Esp32S3XiaoPins,
    TargetChip, WroverKitPins,
};
pub use driver::{
    eoi_probe_window, verify_jpeg_eoi, verify_jpeg_soi, AfConfig, AfMode, AfStatus,
    CameraAutofocus, CameraConfig, CameraDriver, HalCameraSensor, HalOv2640Sensor, Initialized,
    SavedCameraSettings, Streaming, Uninitialized, CAM_SOI_PROBE_BYTES, JPEG_EOI_MARKER,
    JPEG_SOI_MARKER,
};
pub use frame_pool::{
    FrameGuard, FrameMetadata, FramePool, SLOT_FREE, SLOT_LEASED, SLOT_READY,
};
