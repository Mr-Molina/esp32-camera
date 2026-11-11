//! Fuzz target for `parse_jpeg_dimensions` on arbitrary bytes (`CONV-22`).

#![no_main]

use esp32_cam_conversions::{parse_jpeg_dimensions, verify_jpeg_soi};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    if let Some((width, height)) = parse_jpeg_dimensions(data) {
        assert!(
            width > 0 && height > 0,
            "parsed JPEG dimensions must be non-zero: ({width}, {height})"
        );
        assert!(
            verify_jpeg_soi(data).is_some(),
            "parsed JPEG dimensions require a valid SOI marker: len={}",
            data.len()
        );
    }
});
