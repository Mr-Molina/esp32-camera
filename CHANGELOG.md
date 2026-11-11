# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Complete Rust workspace transition for ESP32 camera driver (`esp32-cam-*` crates and `klipper-cam-fw`).
- CI validation pipeline with Rust 1.99 host checks, Miri UB validation, and Xtensa ESP32 release build.
- `cargo-deny` configuration for advisories, licenses, bans, and sources.
- Clippy configuration enforcing MSRV 1.99 and function length threshold.
- Cold boot invariants verification script for hardware register timing.
