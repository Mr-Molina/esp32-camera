//! Safe `#![no_std]` SCCB (Serial Camera Control Bus) abstraction,
//! `embedded-hal` 1.0 adapter, and zero-heap [`MockSccbBus`].

#![no_std]
#![forbid(unsafe_code)]

pub mod adapter;
pub mod bus;
pub mod mock;

pub use adapter::*;
pub use bus::*;
pub use mock::*;
