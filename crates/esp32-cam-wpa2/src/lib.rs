//! Pure-Rust WPA/WPA2 Supplicant and 802.11i state machine for ESP32.
//!
//! Provides IEEE 802.11i / WPA2 Personal authentication, 4-way handshake,
//! EAPOL frame processing, and C ABI compatibility layer for the ESP32
//! WiFi driver.

#![cfg_attr(not(test), no_std)]
#![cfg_attr(
    all(target_arch = "xtensa", feature = "c-supplicant-ffi"),
    feature(asm_experimental_arch)
)]

#[deny(unsafe_code)]
pub mod crypto;
#[deny(unsafe_code)]
pub mod handshake;
#[deny(unsafe_code)]
pub mod ie;
#[deny(unsafe_code)]
pub mod kdf;

#[cfg(feature = "c-supplicant-ffi")]
pub mod crypto_ffi;
#[cfg(feature = "c-supplicant-ffi")]
pub mod ffi;
#[cfg(feature = "c-supplicant-ffi")]
pub mod stubs;

#[cfg(feature = "c-supplicant-ffi")]
pub use ffi::link_wpa2;
