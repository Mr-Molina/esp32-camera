//! Safe `#![no_std]` JPEG SOI/EOI marker scanner (with 4-byte SWAR fast forward scan)
//! and `SOF0`/`SOF2` frame dimension parser.
//!
//! Replaces `cam_verify_jpeg_soi`, `cam_verify_jpeg_eoi`, and `eoi_probe_window`
//! from `driver/cam_hal.c`.

/// JPEG Start-Of-Image plus following marker prefix (`0xFF, 0xD8, 0xFF`).
pub const JPEG_SOI_MARKER: [u8; 3] = [0xFF, 0xD8, 0xFF];

/// JPEG End-Of-Image marker (`0xFF, 0xD9`).
pub const JPEG_EOI_MARKER: [u8; 2] = [0xFF, 0xD9];

/// Computes the PSRAM cache-invalidation and scan window for JPEG EOI detection.
///
/// Matches `eoi_probe_window` in `driver/cam_hal.c:167-171`, using saturating
/// addition so `half + (JPEG_EOI_MARKER.len() - 1)` never overflows `usize`.
#[inline]
pub const fn eoi_probe_window(half: usize, frame_len: usize) -> usize {
    let w = half.saturating_add(JPEG_EOI_MARKER.len() - 1);
    if w > frame_len {
        frame_len
    } else {
        w
    }
}

/// Scans `buf` forward for the 3-byte [`JPEG_SOI_MARKER`] (`[0xFF, 0xD8, 0xFF]`).
///
/// Returns `Some(offset)` of the first match, or `None` if `buf.len() < 3` or no
/// SOI marker is present. Matches `cam_verify_jpeg_soi` in `driver/cam_hal.c:173-192`.
#[inline]
pub fn verify_jpeg_soi(buf: &[u8]) -> Option<usize> {
    if buf.len() < JPEG_SOI_MARKER.len() {
        return None;
    }
    buf.windows(JPEG_SOI_MARKER.len())
        .position(|w| w == JPEG_SOI_MARKER)
}

/// Scans `buf` for the 2-byte [`JPEG_EOI_MARKER`] (`[0xFF, 0xD9]`).
///
/// - When `search_forward` is `true`, uses the 4-byte SWAR (`0x0101_0101` / `0x8080_8080`)
///   candidate detector from `driver/cam_hal.c:200-233` via safe `u32::from_le_bytes`
///   to locate the earliest `0xFF, 0xD9` marker in `buf`.
/// - When `search_forward` is `false`, scans backward from the end of `buf` to find
///   the last `0xFF, 0xD9` marker (`driver/cam_hal.c:235-245`).
pub fn verify_jpeg_eoi(buf: &[u8], search_forward: bool) -> Option<usize> {
    let length = buf.len();
    if length < JPEG_EOI_MARKER.len() {
        return None;
    }

    if search_forward {
        const A: u32 = (JPEG_EOI_MARKER[0] as u32) * 0x0101_0101;
        const ONE: u32 = 0x0101_0101;
        const HIGH: u32 = 0x8080_8080;

        let mut i = 0usize;
        while i + 4 <= length {
            let w = u32::from_le_bytes([buf[i], buf[i + 1], buf[i + 2], buf[i + 3]]);
            let x = w ^ A;
            let mut m = (!x & x.wrapping_sub(ONE)) & HIGH;
            while m != 0 {
                let off = (m.trailing_zeros() >> 3) as usize;
                let pos = i + off;
                if pos + JPEG_EOI_MARKER.len() <= length
                    && buf[pos] == JPEG_EOI_MARKER[0]
                    && buf[pos + 1] == JPEG_EOI_MARKER[1]
                {
                    return Some(pos);
                }
                m &= m.wrapping_sub(1);
            }
            i += 4;
        }

        while i + JPEG_EOI_MARKER.len() <= length {
            if buf[i] == JPEG_EOI_MARKER[0] && buf[i + 1] == JPEG_EOI_MARKER[1] {
                return Some(i);
            }
            i += 1;
        }
        return None;
    }

    let mut pos = length - JPEG_EOI_MARKER.len();
    loop {
        if buf[pos] == JPEG_EOI_MARKER[0] && buf[pos + 1] == JPEG_EOI_MARKER[1] {
            return Some(pos);
        }
        if pos == 0 {
            break;
        }
        pos -= 1;
    }
    None
}

/// Parses the `(width, height)` dimensions from a JPEG `SOF0` (`0xFFC0`), `SOF1` (`0xFFC1`),
/// or `SOF2` (`0xFFC2`) frame header in `buf`.
///
/// Returns `Some((width, height))` if a valid SOF header with non-zero dimensions is found,
/// or `None` if `buf` is truncated or does not contain a valid SOF segment.
pub fn parse_jpeg_dimensions(buf: &[u8]) -> Option<(u16, u16)> {
    if buf.len() < 9 {
        return None;
    }

    let soi_idx = verify_jpeg_soi(buf)?;
    let mut i = soi_idx + 2;

    while i + 1 < buf.len() {
        if buf[i] != 0xFF {
            i += 1;
            continue;
        }

        // Skip any fill 0xFF bytes.
        while i < buf.len() && buf[i] == 0xFF {
            i += 1;
        }
        if i >= buf.len() {
            return None;
        }

        let marker = buf[i];
        i += 1;

        match marker {
            // Stuffed byte or standalone markers without a length payload
            0x00 | 0x01 | 0xD0..=0xD8 => continue,
            // EOI or SOS reached before SOF
            0xD9 | 0xDA => return None,
            // SOF0 (Baseline DCT), SOF1 (Extended Sequential), SOF2 (Progressive DCT)
            0xC0..=0xC2 => {
                if i + 7 > buf.len() {
                    return None;
                }
                let seg_len = u16::from_be_bytes([buf[i], buf[i + 1]]) as usize;
                if seg_len < 8 || i.checked_add(seg_len)? > buf.len() {
                    return None;
                }
                let height = u16::from_be_bytes([buf[i + 3], buf[i + 4]]);
                let width = u16::from_be_bytes([buf[i + 5], buf[i + 6]]);
                if width == 0 || height == 0 {
                    return None;
                }
                return Some((width, height));
            }
            // Any other marker with a 2-byte big-endian length parameter
            _ => {
                if i + 2 > buf.len() {
                    return None;
                }
                let seg_len = u16::from_be_bytes([buf[i], buf[i + 1]]) as usize;
                if seg_len < 2 {
                    return None;
                }
                i = i.checked_add(seg_len)?;
            }
        }
    }

    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_eoi_probe_window() {
        assert_eq!(eoi_probe_window(1024, 2048), 1025);
        assert_eq!(eoi_probe_window(1024, 1024), 1024);
        assert_eq!(eoi_probe_window(1024, 512), 512);
        assert_eq!(eoi_probe_window(usize::MAX, 4096), 4096);
    }

    #[test]
    fn test_verify_jpeg_soi() {
        assert_eq!(verify_jpeg_soi(&[]), None);
        assert_eq!(verify_jpeg_soi(&[0xFF, 0xD8]), None);
        assert_eq!(verify_jpeg_soi(&[0xFF, 0xD8, 0xFF]), Some(0));
        assert_eq!(
            verify_jpeg_soi(&[0x00, 0x11, 0xFF, 0xD8, 0xFF, 0xE0]),
            Some(2)
        );
        assert_eq!(verify_jpeg_soi(&[0xFF, 0xD8, 0x00, 0xFF, 0xD9]), None);
    }

    #[test]
    fn test_verify_jpeg_eoi_swar_alignments_and_boundaries() {
        assert_eq!(verify_jpeg_eoi(&[], true), None);
        assert_eq!(verify_jpeg_eoi(&[0xFF], true), None);
        assert_eq!(verify_jpeg_eoi(&[0xFF], false), None);

        // Test every byte alignment offset (0..=12) across 4-byte SWAR words
        for offset in 0..=12 {
            let mut buf = [0x12u8; 16];
            buf[offset] = 0xFF;
            buf[offset + 1] = 0xD9;
            assert_eq!(
                verify_jpeg_eoi(&buf[..offset + 2], true),
                Some(offset),
                "forward scan failed at end-of-buffer offset {offset}"
            );
            assert_eq!(
                verify_jpeg_eoi(&buf, true),
                Some(offset),
                "forward scan failed at interior offset {offset}"
            );
            assert_eq!(
                verify_jpeg_eoi(&buf, false),
                Some(offset),
                "backward scan failed at offset {offset}"
            );
        }

        // Split marker at 4-byte word boundary (index 3 is 0xFF, index 4 is 0xD9)
        let split_word = [0x01, 0x02, 0x03, 0xFF, 0xD9, 0x04, 0x05, 0x06];
        assert_eq!(verify_jpeg_eoi(&split_word, true), Some(3));
        assert_eq!(verify_jpeg_eoi(&split_word, false), Some(3));

        // Split marker truncated right at end of buffer (only 0xFF at end)
        let truncated = [0x01, 0x02, 0x03, 0xFF];
        assert_eq!(verify_jpeg_eoi(&truncated, true), None);
        assert_eq!(verify_jpeg_eoi(&truncated, false), None);

        // False candidate 0xFF 0x00 followed by real 0xFF 0xD9 in same 4-byte word
        let stuffed_then_eoi = [0xFF, 0x00, 0xFF, 0xD9];
        assert_eq!(verify_jpeg_eoi(&stuffed_then_eoi, true), Some(2));
        assert_eq!(verify_jpeg_eoi(&stuffed_then_eoi, false), Some(2));

        // Multiple EOI markers: forward returns earliest, backward returns latest
        let multi_eoi = [0xFF, 0xD8, 0xFF, 0xD9, 0xAA, 0xBB, 0xFF, 0xD9, 0x00];
        assert_eq!(verify_jpeg_eoi(&multi_eoi, true), Some(2));
        assert_eq!(verify_jpeg_eoi(&multi_eoi, false), Some(6));
    }

    #[test]
    fn test_parse_jpeg_dimensions_sof0_and_sof2() {
        // Minimal JFIF + SOF0 (1280x720)
        let sof0_stream = [
            0xFF, 0xD8, // SOI
            0xFF, 0xE0, 0x00, 0x04, 0x00, 0x00, // APP0 (length 4)
            0xFF, 0xC0, 0x00, 0x11, 0x08, 0x02, 0xD0, 0x05, 0x00, // SOF0: height=720, width=1280
            0x03, 0x01, 0x22, 0x00, 0x02, 0x11, 0x01, 0x03, 0x11, 0x01, 0xFF, 0xD9,
        ];
        assert_eq!(parse_jpeg_dimensions(&sof0_stream), Some((1280, 720)));

        // Progressive SOF2 (320x240)
        let sof2_stream = [
            0xFF, 0xD8, 0xFF, 0xC2, 0x00, 0x0B, 0x08, 0x00, 0xF0, 0x01, 0x40, 0x01, 0x01, 0x11,
            0x00, 0xFF, 0xD9,
        ];
        assert_eq!(parse_jpeg_dimensions(&sof2_stream), Some((320, 240)));

        // Reject zero dimensions or truncated streams
        let zero_dim = [
            0xFF, 0xD8, 0xFF, 0xC0, 0x00, 0x0B, 0x08, 0x00, 0x00, 0x01, 0x40,
        ];
        assert_eq!(parse_jpeg_dimensions(&zero_dim), None);
        assert_eq!(parse_jpeg_dimensions(&[0xFF, 0xD8, 0xFF, 0xD9]), None);

        // CONV-006: Return None immediately when SOI marker is missing
        let no_soi_stream = [
            0x00, 0x00, 0xFF, 0xC0, 0x00, 0x11, 0x08, 0x02, 0xD0, 0x05, 0x00,
            0x03, 0x01, 0x22, 0x00, 0x02, 0x11, 0x01, 0x03, 0x11, 0x01, 0xFF, 0xD9,
        ];
        assert_eq!(parse_jpeg_dimensions(&no_soi_stream), None);

        // CONV-006: Reject SOF segment whose seg_len exceeds remaining buffer length
        let truncated_sof_seg = [
            0xFF, 0xD8, 0xFF, 0xC0, 0x00, 0x64, 0x08, 0x00, 0xF0, 0x01, 0x40,
        ];
        assert_eq!(parse_jpeg_dimensions(&truncated_sof_seg), None);
    }
}
