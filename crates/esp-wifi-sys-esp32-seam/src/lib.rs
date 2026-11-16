//! Seam crate patching `esp-wifi-sys-esp32` v0.3.0 with pure-Rust RF/clock,
//! regulatory, NVS, and `printf` shims from `esp32-cam-rf-clock` linking only
//! `libphy`.

#![no_std]
#![doc(html_logo_url = "https://avatars.githubusercontent.com/u/46717278")]
#![allow(unnecessary_transmutes)]

extern crate esp32_cam_rf_clock;

mod printf_shim;
mod upstream_bindings;

pub use self::printf_shim::force_rf_clock_link;
pub use self::upstream_bindings::c_types;
#[allow(unused_imports)]
pub(crate) use self::upstream_bindings::fmt;
pub use self::upstream_bindings::include;
