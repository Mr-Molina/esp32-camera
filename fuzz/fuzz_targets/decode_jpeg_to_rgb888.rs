//! Fuzz target for `decode_jpeg_to_rgb888` and `decode_jpeg_to_rgb565`
//! (`CONV-22`).
//!
//! Guards output allocation to `MAX_FUZZ_DIM x MAX_FUZZ_DIM` so malformed huge
//! `SOF0` headers exercise header/overflow paths without OOM-killing libFuzzer.

#![no_main]

use esp32_cam_conversions::{decode_jpeg_to_rgb565, decode_jpeg_to_rgb888, parse_jpeg_dimensions};
use libfuzzer_sys::fuzz_target;

/// Maximum width or height allocated for full pixel-decode fuzzing (192 KiB).
const MAX_FUZZ_DIM: u16 = 256;

/// Stack scratch capacity (`16x16` RGB888) for oversized or unparsed headers.
const SCRATCH_CAP: usize = 16 * 16 * 3;

fuzz_target!(|data: &[u8]| {
    match parse_jpeg_dimensions(data) {
        Some((width, height)) if width <= MAX_FUZZ_DIM && height <= MAX_FUZZ_DIM => {
            let pixels = usize::from(width) * usize::from(height);
            let rgb888_len = pixels * 3;
            let mut rgb888_out = vec![0u8; rgb888_len];
            if let Ok(written) = decode_jpeg_to_rgb888(data, &mut rgb888_out) {
                assert_eq!(
                    written, rgb888_len,
                    "RGB888 byte count mismatch for {width}x{height}: {written}"
                );
            }

            let rgb565_len = pixels * 2;
            let mut rgb565_out = vec![0u8; rgb565_len];
            if let Ok(written) = decode_jpeg_to_rgb565(data, &mut rgb565_out) {
                assert_eq!(
                    written, rgb565_len,
                    "RGB565 byte count mismatch for {width}x{height}: {written}"
                );
            }
        }
        _ => {
            let mut scratch = [0u8; SCRATCH_CAP];
            let _ = decode_jpeg_to_rgb888(data, &mut scratch);
            let _ = decode_jpeg_to_rgb565(data, &mut scratch);
        }
    }
});
