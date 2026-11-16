//! Pure-Rust RF/clock, regulatory, NVS, and printf compatibility layer for
//! ESP32-WROVER.

#![no_std]
#![cfg_attr(target_arch = "xtensa", feature(asm_experimental_arch))]

pub mod nvs_misc;
pub mod printf_compat;
pub mod printf_fmt;
pub mod regulatory;
pub mod rtc_rf;

pub use self::nvs_misc::link_anchor;
pub use self::printf_compat::link_touch as link_printf_compat;
pub use self::rtc_rf::link_touch as link_rtc_rf;
