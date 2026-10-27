//! Core `#![no_std]` types, sensor PIDs, SCCB addresses, frame sizes,
//! resolution tables, and error types for the `esp32-camera` driver stack.

#![no_std]
#![forbid(unsafe_code)]

pub mod control;
pub mod error;
pub mod frame_size;
pub mod pixel_format;
pub mod sensor;

pub use control::*;
pub use error::*;
pub use frame_size::*;
pub use pixel_format::*;
pub use sensor::*;
