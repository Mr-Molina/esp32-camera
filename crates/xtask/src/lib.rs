//! Host automation and verification tasks for `esp32-camera`.

#![cfg_attr(target_arch = "xtensa", no_std)]

#[cfg(not(target_arch = "xtensa"))]
pub mod bootstrap_env;
#[cfg(not(target_arch = "xtensa"))]
pub mod cli;
#[cfg(not(target_arch = "xtensa"))]
pub mod elf_wrap;
#[cfg(not(target_arch = "xtensa"))]
pub mod ota_deploy;
#[cfg(not(target_arch = "xtensa"))]
pub mod pure_rust_audit;
#[cfg(not(target_arch = "xtensa"))]
pub mod serial_passive;
pub mod sha256;
#[cfg(not(target_arch = "xtensa"))]
pub mod verify_invariants;

#[cfg(not(target_arch = "xtensa"))]
pub use cli::run;
