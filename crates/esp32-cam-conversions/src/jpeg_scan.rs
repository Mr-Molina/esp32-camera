//! Safe `#![no_std]` JPEG SOI/EOI marker scanner (using slice-window iterator
//! helpers) and strict `SOF0`..`SOF2` frame dimension parser.
//!
//! Replaces `cam_verify_jpeg_soi`, `cam_verify_jpeg_eoi`, and
//! `eoi_probe_window` from `driver/cam_hal.c`.

/// JPEG Start-Of-Image plus following marker prefix (`0xFF, 0xD8, 0xFF`).
pub const JPEG_SOI_MARKER: [u8; 3] = [0xFF, 0xD8, 0xFF];

/// JPEG Start-Of-Image 2-byte marker (`0xFF, 0xD8`), gated to unit tests.
#[cfg(test)]
pub(crate) const JPEG_SOI_2BYTE_MARKER: [u8; 2] = [0xFF, 0xD8];

/// JPEG End-Of-Image marker (`0xFF, 0xD9`).
pub const JPEG_EOI_MARKER: [u8; 2] = [0xFF, 0xD9];

/// Scans `inbuf` forward for the earliest 2-byte marker `[0xFF, second]`.
#[inline]
fn scan_marker_2_forward(inbuf: &[u8], second: u8) -> Option<usize> {
    inbuf.windows(2).position(|w| w == [0xFF, second])
}

/// Scans `inbuf` backward for the last 2-byte marker `[0xFF, second]`.
#[inline]
fn scan_marker_2_backward(inbuf: &[u8], second: u8) -> Option<usize> {
    inbuf.windows(2).rposition(|w| w == [0xFF, second])
}

/// Scans `inbuf` forward for the earliest 3-byte [`JPEG_SOI_MARKER`]
/// (`[0xFF, 0xD8, 0xFF]`).
#[inline]
fn scan_soi_3_forward(inbuf: &[u8]) -> Option<usize> {
    inbuf.windows(3).position(|w| w == JPEG_SOI_MARKER)
}

/// Computes the PSRAM cache-invalidation and scan window for JPEG EOI
/// detection.
///
/// Matches `eoi_probe_window` in `driver/cam_hal.c:167-171`, using saturating
/// addition so `half + (JPEG_EOI_MARKER.len() - 1)` never overflows `usize`.
#[inline]
#[must_use]
pub const fn eoi_probe_window(half: usize, frame_len: usize) -> usize {
    let w = half.saturating_add(JPEG_EOI_MARKER.len() - 1);
    if w > frame_len {
        frame_len
    } else {
        w
    }
}

/// Scans `buf` forward for the 3-byte [`JPEG_SOI_MARKER`]
/// (`[0xFF, 0xD8, 0xFF]`).
///
/// Uses slice-window iteration (`CONV-11`) to locate the earliest SOI marker
/// prefix.
///
/// Returns `Some(offset)` of the first match, or `None` if `buf.len() < 3` or
/// no SOI marker is present. Matches `cam_verify_jpeg_soi` in
/// `driver/cam_hal.c:173-192`.
#[inline]
#[must_use]
pub fn verify_jpeg_soi(buf: &[u8]) -> Option<usize> {
    scan_soi_3_forward(buf)
}

/// Scans `buf` forward for the 2-byte [`JPEG_SOI_2BYTE_MARKER`]
/// (`[0xFF, 0xD8]`) using slice-window iteration.
#[cfg(test)]
#[inline]
#[must_use]
pub(crate) fn verify_jpeg_soi_2byte(buf: &[u8]) -> Option<usize> {
    scan_marker_2_forward(buf, JPEG_SOI_2BYTE_MARKER[1])
}

/// Scans `buf` for the 2-byte [`JPEG_EOI_MARKER`] (`[0xFF, 0xD9]`).
///
/// - When `search_forward` is `true`, scans forward via `windows(2).position`
///   to locate the earliest `0xFF, 0xD9` marker in `buf`.
/// - When `search_forward` is `false`, scans backward from the end of `buf`
///   via `windows(2).rposition` to find the last `0xFF, 0xD9` marker.
#[inline]
#[must_use]
pub fn verify_jpeg_eoi(buf: &[u8], search_forward: bool) -> Option<usize> {
    if search_forward {
        scan_marker_2_forward(buf, JPEG_EOI_MARKER[1])
    } else {
        scan_marker_2_backward(buf, JPEG_EOI_MARKER[1])
    }
}

#[inline]
fn extract_jpeg_payload_with_dir(buf: &[u8], search_forward: bool) -> Option<&[u8]> {
    if buf.len() < 4 {
        return None;
    }
    let soi_offset = verify_jpeg_soi(buf)?;
    let sub = &buf[soi_offset..];
    let rel_eoi = verify_jpeg_eoi(sub, search_forward)?;
    let eoi_offset = soi_offset.checked_add(rel_eoi)?;
    let eoi_end = eoi_offset.checked_add(JPEG_EOI_MARKER.len())?;
    let soi_min_end = soi_offset.checked_add(2)?;
    if soi_min_end > eoi_end || eoi_end > buf.len() {
        return None;
    }
    Some(&buf[soi_offset..eoi_end])
}

/// Extracts the valid JPEG payload slice from `buf`, verifying that:
/// 1. A valid SOI marker exists.
/// 2. A valid EOI marker exists after the SOI marker.
/// 3. The SOI marker occurs strictly before EOI (`soi_offset + 2 <= eoi_end`).
///
/// Safely handles truncated or malformed buffers without panic or arithmetic
/// underflow. Returns `Some(&buf[soi_offset..eoi_end])` on success, or `None`
/// if markers are missing, inverted, or out of bounds.
#[must_use]
pub fn extract_jpeg_payload(buf: &[u8]) -> Option<&[u8]> {
    extract_jpeg_payload_with_dir(buf, false)
}

/// Extracts the first JPEG payload slice from `buf` using forward EOI
/// scanning (`CONV-21` test-only helper).
#[cfg(test)]
#[must_use]
pub(crate) fn extract_first_jpeg_payload(buf: &[u8]) -> Option<&[u8]> {
    extract_jpeg_payload_with_dir(buf, true)
}

/// Validates that `buf` contains a complete JPEG frame with valid SOI/EOI
/// boundaries and parses its `(width, height)` dimensions from `SOF0`..`SOF2`.
///
/// Verifies that `SOI` occurs strictly before `EOI`
/// (`soi_offset + 2 <= eoi_end`) and handles malformed slices without
/// underflow. Returns `Some((width, height))` on success, or `None` if
/// the frame is invalid or truncated.
#[must_use]
pub fn validate_jpeg_frame(buf: &[u8]) -> Option<(u16, u16)> {
    let payload = extract_jpeg_payload(buf)?;
    parse_jpeg_dimensions(payload)
}

/// Parses the `(width, height)` dimensions from a JPEG `SOF0` (`0xFFC0`),
/// `SOF1` (`0xFFC1`), or `SOF2` (`0xFFC2`) frame header in `buf`.
///
/// Enforces ITU-T T.81 marker framing and frame-header rules (`CONV-16`):
/// - Every segment following `SOI` must begin with a `0xFF` marker prefix
///   (optional `0xFF` fill bytes are skipped; non-`0xFF` inter-segment junk
///   and `0x00`/`0x01` markers before `SOS` are rejected).
/// - `SOF0`..`SOF2` must carry `nf` in `1..=4`, exact segment length
///   `seg_len == 8 + 3 * nf` (`>= 11`), and non-zero `width` and `height`.
#[must_use]
pub fn parse_jpeg_dimensions(buf: &[u8]) -> Option<(u16, u16)> {
    // Minimum valid SOI (2) + SOF marker (2) + SOF body for Nf=1 (11) = 15.
    if buf.len() < 15 {
        return None;
    }

    let soi_idx = verify_jpeg_soi(buf)?;
    let mut i = soi_idx + 2;

    while i + 1 < buf.len() {
        // Every marker segment boundary must begin with 0xFF.
        if buf[i] != 0xFF {
            return None;
        }

        // Skip leading 0xFF marker prefix and any optional 0xFF fill bytes.
        while i < buf.len() && buf[i] == 0xFF {
            i += 1;
        }
        if i >= buf.len() {
            return None;
        }

        let marker = buf[i];
        i += 1;

        match marker {
            // Reject stuffed 0x00, TEM (0x01), EOI (0xD9), or SOS (0xDA)
            // appearing before any valid SOF header.
            0x00 | 0x01 | 0xD9 | 0xDA => return None,
            // Standalone restart/SOI markers carry no length payload.
            0xD0..=0xD8 => continue,
            // SOF0 (Baseline), SOF1 (Extended Sequential), SOF2 (Progressive).
            0xC0..=0xC2 => {
                if i + 2 > buf.len() {
                    return None;
                }
                let seg_len = u16::from_be_bytes([buf[i], buf[i + 1]]) as usize;
                if seg_len < 11 || i.checked_add(seg_len)? > buf.len() {
                    return None;
                }
                let nf = buf[i + 7] as usize;
                if !(1..=4).contains(&nf) || seg_len != 8 + 3 * nf {
                    return None;
                }
                let height = u16::from_be_bytes([buf[i + 3], buf[i + 4]]);
                let width = u16::from_be_bytes([buf[i + 5], buf[i + 6]]);
                if width == 0 || height == 0 {
                    return None;
                }
                return Some((width, height));
            }
            // Any other marker segment carries a 2-byte big-endian length.
            _ => {
                if i + 2 > buf.len() {
                    return None;
                }
                let seg_len = u16::from_be_bytes([buf[i], buf[i + 1]]) as usize;
                if seg_len < 2 {
                    return None;
                }
                let next_i = i.checked_add(seg_len)?;
                if next_i > buf.len() {
                    return None;
                }
                i = next_i;
            }
        }
    }

    None
}

#[cfg(test)]
mod tests {
    use super::*;

    fn naive_find_forward(haystack: &[u8], needle: &[u8]) -> Option<usize> {
        haystack.windows(needle.len()).position(|w| w == needle)
    }

    fn naive_find_backward(haystack: &[u8], needle: &[u8]) -> Option<usize> {
        haystack.windows(needle.len()).rposition(|w| w == needle)
    }

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
    fn test_verify_jpeg_soi_2byte() {
        assert_eq!(verify_jpeg_soi_2byte(&[]), None);
        assert_eq!(verify_jpeg_soi_2byte(&[0xFF]), None);
        assert_eq!(verify_jpeg_soi_2byte(&[0xFF, 0xD8]), Some(0));
        assert_eq!(
            verify_jpeg_soi_2byte(&[0x00, 0x11, 0xFF, 0xD8, 0x00]),
            Some(2)
        );
        assert_eq!(
            verify_jpeg_soi_2byte(&[0x00, 0x11, 0xFF, 0xD8, 0xFF, 0xE0]),
            Some(2)
        );
    }

    #[test]
    fn test_extract_first_jpeg_payload() {
        let multi_frame = [
            0xFF, 0xD8, 0xFF, 0xE0, 0xFF, 0xD9, // First frame (len 6)
            0xAA, 0xBB, 0xFF, 0xD8, 0xFF, 0xE0, 0xFF, 0xD9, // Second frame
        ];
        assert_eq!(
            extract_first_jpeg_payload(&multi_frame),
            Some(&multi_frame[..6])
        );
    }

    #[test]
    fn test_verify_jpeg_eoi_swar_alignments_and_boundaries() {
        assert_eq!(verify_jpeg_eoi(&[], true), None);
        assert_eq!(verify_jpeg_eoi(&[0xFF], true), None);
        assert_eq!(verify_jpeg_eoi(&[0xFF], false), None);

        // Test every byte alignment offset (0..=12) across 4-byte SWAR words.
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

        // Split marker at 4-byte word boundary (index 3 is 0xFF, 4 is 0xD9).
        let split_word = [0x01, 0x02, 0x03, 0xFF, 0xD9, 0x04, 0x05, 0x06];
        assert_eq!(verify_jpeg_eoi(&split_word, true), Some(3));
        assert_eq!(verify_jpeg_eoi(&split_word, false), Some(3));

        // Split marker truncated right at end of buffer (only 0xFF at end).
        let truncated = [0x01, 0x02, 0x03, 0xFF];
        assert_eq!(verify_jpeg_eoi(&truncated, true), None);
        assert_eq!(verify_jpeg_eoi(&truncated, false), None);

        // False candidate 0xFF 0x00 followed by real 0xFF 0xD9 in same word.
        let stuffed_then_eoi = [0xFF, 0x00, 0xFF, 0xD9];
        assert_eq!(verify_jpeg_eoi(&stuffed_then_eoi, true), Some(2));
        assert_eq!(verify_jpeg_eoi(&stuffed_then_eoi, false), Some(2));

        // Multiple EOI markers: forward returns first, backward returns last.
        let multi_eoi = [0xFF, 0xD8, 0xFF, 0xD9, 0xAA, 0xBB, 0xFF, 0xD9, 0x00];
        assert_eq!(verify_jpeg_eoi(&multi_eoi, true), Some(2));
        assert_eq!(verify_jpeg_eoi(&multi_eoi, false), Some(6));
    }

    #[test]
    // Exhaustive sweep: minutes to hours under Miri's interpreter, and the
    // same code path as the representative-input tests that do run there.
    #[cfg_attr(miri, ignore)]
    fn test_unified_swar_kernel_all_alignments_and_borrow_patterns() {
        // Test 3-byte SOI and 2-byte SOI/EOI across lengths 0..=24, every
        // alignment offset, and 0xFF/0xFE borrow false-positive patterns.
        for len in 0..=24usize {
            for fill in [0x00u8, 0x7F, 0xFE, 0xFF] {
                let base = [fill; 24];
                let slice = &base[..len];
                assert_eq!(
                    verify_jpeg_soi(slice),
                    naive_find_forward(slice, &JPEG_SOI_MARKER),
                    "SOI mismatch on uniform fill {fill:#04x} len {len}"
                );
                assert_eq!(
                    verify_jpeg_soi_2byte(slice),
                    naive_find_forward(slice, &JPEG_SOI_2BYTE_MARKER),
                    "SOI-2B mismatch on uniform fill {fill:#04x} len {len}"
                );
                assert_eq!(
                    verify_jpeg_eoi(slice, true),
                    naive_find_forward(slice, &JPEG_EOI_MARKER),
                    "EOI fwd mismatch on uniform fill {fill:#04x} len {len}"
                );
                assert_eq!(
                    verify_jpeg_eoi(slice, false),
                    naive_find_backward(slice, &JPEG_EOI_MARKER),
                    "EOI rev mismatch on uniform fill {fill:#04x} len {len}"
                );
            }
        }

        // Test 3-byte SOI across every word boundary and tail length.
        for total_len in 3..=20usize {
            for offset in 0..=(total_len - 3) {
                let mut buf = [0xFEu8; 20];
                // Pre-seed benign 0xFF, 0xFE SWAR borrow patterns before/after.
                if offset >= 2 {
                    buf[offset - 2] = 0xFF;
                    buf[offset - 1] = 0xFE;
                }
                buf[offset..offset + 3].copy_from_slice(&JPEG_SOI_MARKER);
                let slice = &buf[..total_len];
                assert_eq!(
                    verify_jpeg_soi(slice),
                    Some(offset),
                    "SOI failed at offset {offset} total_len {total_len}"
                );
                assert_eq!(
                    verify_jpeg_soi_2byte(slice),
                    Some(offset),
                    "SOI-2B failed at offset {offset} total_len {total_len}"
                );
            }
        }

        // Deterministic PRNG differential check against naive byte scanners.
        let mut state = 0x9E37_79B9_7F4A_7C15u64;
        let mut buf = [0u8; 64];
        for case_idx in 0..512usize {
            let len = case_idx % (buf.len() + 1);
            for b in &mut buf[..len] {
                state ^= state << 13;
                state ^= state >> 7;
                state ^= state << 17;
                // Bias towards 0xFF, 0xFE, 0xD8, 0xD9 to stress SWAR lanes.
                *b = match (state & 0x07) as u8 {
                    0 | 1 => 0xFF,
                    2 => 0xFE,
                    3 => 0xD8,
                    4 => 0xD9,
                    _ => (state >> 8) as u8,
                };
            }
            let slice = &buf[..len];
            assert_eq!(
                verify_jpeg_soi(slice),
                naive_find_forward(slice, &JPEG_SOI_MARKER)
            );
            assert_eq!(
                verify_jpeg_soi_2byte(slice),
                naive_find_forward(slice, &JPEG_SOI_2BYTE_MARKER)
            );
            assert_eq!(
                verify_jpeg_eoi(slice, true),
                naive_find_forward(slice, &JPEG_EOI_MARKER)
            );
            assert_eq!(
                verify_jpeg_eoi(slice, false),
                naive_find_backward(slice, &JPEG_EOI_MARKER)
            );
        }
    }

    #[test]
    fn test_parse_jpeg_dimensions_sof0_and_sof2() {
        // Minimal JFIF + SOF0 (1280x720, Nf=3, seg_len=17)
        let sof0_stream = [
            0xFF, 0xD8, // SOI
            0xFF, 0xE0, 0x00, 0x04, 0x00, 0x00, // APP0 (length 4)
            0xFF, 0xC0, 0x00, 0x11, 0x08, 0x02, 0xD0, 0x05, 0x00, // SOF0
            0x03, 0x01, 0x22, 0x00, 0x02, 0x11, 0x01, 0x03, 0x11, 0x01, // Nf=3
            0xFF, 0xD9,
        ];
        assert_eq!(parse_jpeg_dimensions(&sof0_stream), Some((1280, 720)));

        // Progressive SOF2 (320x240, Nf=1, seg_len=11)
        let sof2_stream = [
            0xFF, 0xD8, 0xFF, 0xC2, 0x00, 0x0B, 0x08, 0x00, 0xF0, 0x01, 0x40, 0x01, 0x01, 0x11,
            0x00, 0xFF, 0xD9,
        ];
        assert_eq!(parse_jpeg_dimensions(&sof2_stream), Some((320, 240)));

        // Valid 4-component CMYK SOF0 (64x48, Nf=4, seg_len=20)
        let cmyk_sof0 = [
            0xFF, 0xD8, 0xFF, 0xC0, 0x00, 0x14, 0x08, 0x00, 0x30, 0x00, 0x40, 0x04, 0x01, 0x11,
            0x00, 0x02, 0x11, 0x00, 0x03, 0x11, 0x00, 0x04, 0x11, 0x00, 0xFF, 0xD9,
        ];
        assert_eq!(parse_jpeg_dimensions(&cmyk_sof0), Some((64, 48)));

        // Reject zero dimensions or truncated streams
        let zero_dim = [
            0xFF, 0xD8, 0xFF, 0xC0, 0x00, 0x0B, 0x08, 0x00, 0x00, 0x01, 0x40, 0x01, 0x01, 0x11,
            0x00, 0xFF, 0xD9,
        ];
        assert_eq!(parse_jpeg_dimensions(&zero_dim), None);
        assert_eq!(parse_jpeg_dimensions(&[0xFF, 0xD8, 0xFF, 0xD9]), None);

        // Return None immediately when SOI marker is missing
        let no_soi_stream = [
            0x00, 0x00, 0xFF, 0xC0, 0x00, 0x11, 0x08, 0x02, 0xD0, 0x05, 0x00, 0x03, 0x01, 0x22,
            0x00, 0x02, 0x11, 0x01, 0x03, 0x11, 0x01, 0xFF, 0xD9,
        ];
        assert_eq!(parse_jpeg_dimensions(&no_soi_stream), None);

        // Reject SOF segment whose seg_len exceeds remaining buffer length
        let truncated_sof_seg = [
            0xFF, 0xD8, 0xFF, 0xC0, 0x00, 0x64, 0x08, 0x00, 0xF0, 0x01, 0x40,
        ];
        assert_eq!(parse_jpeg_dimensions(&truncated_sof_seg), None);
    }

    #[test]
    fn test_parse_jpeg_dimensions_strict_sof_and_framing_validation() {
        // 1. Short SOF length (seg_len = 8 < 11, with Nf = 0)
        let short_sof_nf0 = [
            0xFF, 0xD8, 0xFF, 0xC0, 0x00, 0x08, 0x08, 0x00, 0xF0, 0x01, 0x40, 0x00, 0xFF, 0xD9,
        ];
        assert_eq!(parse_jpeg_dimensions(&short_sof_nf0), None);

        // 2. Malformed SOF length: seg_len = 11 but Nf = 3 (expected 17)
        let bad_sof_len_short = [
            0xFF, 0xD8, 0xFF, 0xC0, 0x00, 0x0B, 0x08, 0x00, 0xF0, 0x01, 0x40, 0x03, 0x01, 0x11,
            0x00, 0xFF, 0xD9,
        ];
        assert_eq!(parse_jpeg_dimensions(&bad_sof_len_short), None);

        // 3. Malformed SOF length: seg_len = 17 but Nf = 1 (expected 11)
        let bad_sof_len_long = [
            0xFF, 0xD8, 0xFF, 0xC0, 0x00, 0x11, 0x08, 0x00, 0xF0, 0x01, 0x40, 0x01, 0x01, 0x11,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xD9,
        ];
        assert_eq!(parse_jpeg_dimensions(&bad_sof_len_long), None);

        // 4. Out-of-bounds component count Nf = 5 even when seg_len == 8 + 3*5
        let bad_nf_five = [
            0xFF, 0xD8, 0xFF, 0xC0, 0x00, 0x17, 0x08, 0x00, 0xF0, 0x01, 0x40, 0x05, 0x01, 0x11,
            0x00, 0x02, 0x11, 0x00, 0x03, 0x11, 0x00, 0x04, 0x11, 0x00, 0x05, 0x11, 0x00, 0xFF,
            0xD9,
        ];
        assert_eq!(parse_jpeg_dimensions(&bad_nf_five), None);

        // 5. False 0xC0 byte without a preceding 0xFF marker prefix after APP0
        let false_c0_no_ff = [
            0xFF, 0xD8, // SOI
            0xFF, 0xE0, 0x00, 0x04, 0x00, 0x00, // APP0 (length 4)
            0x00, 0xC0, 0x00, 0x0B, 0x08, 0x00, 0xF0, 0x01, 0x40, 0x01, 0x01, 0x11, 0x00, 0xFF,
            0xD9,
        ];
        assert_eq!(parse_jpeg_dimensions(&false_c0_no_ff), None);

        // 6. Inter-segment non-0xFF junk byte before real 0xFF 0xC0
        let junk_between_segments = [
            0xFF, 0xD8, // SOI
            0xFF, 0xE0, 0x00, 0x04, 0x00, 0x00, // APP0 (length 4)
            0x42, // Illegal non-0xFF byte between segments
            0xFF, 0xC0, 0x00, 0x0B, 0x08, 0x00, 0xF0, 0x01, 0x40, 0x01, 0x01, 0x11, 0x00, 0xFF,
            0xD9,
        ];
        assert_eq!(parse_jpeg_dimensions(&junk_between_segments), None);

        // 7. Stuffed 0xFF 0x00 and TEM 0xFF 0x01 rejected before SOF/SOS
        let stuffed_before_sof = [
            0xFF, 0xD8, 0xFF, 0x00, 0xFF, 0xC0, 0x00, 0x0B, 0x08, 0x00, 0xF0, 0x01, 0x40, 0x01,
            0x01, 0x11, 0x00, 0xFF, 0xD9,
        ];
        assert_eq!(parse_jpeg_dimensions(&stuffed_before_sof), None);

        let tem_before_sof = [
            0xFF, 0xD8, 0xFF, 0x01, 0xFF, 0xC0, 0x00, 0x0B, 0x08, 0x00, 0xF0, 0x01, 0x40, 0x01,
            0x01, 0x11, 0x00, 0xFF, 0xD9,
        ];
        assert_eq!(parse_jpeg_dimensions(&tem_before_sof), None);

        // 8. Legal 0xFF fill bytes before markers are skipped cleanly
        let fill_bytes_stream = [
            0xFF, 0xD8, // SOI
            0xFF, 0xFF, 0xFF, 0xE0, 0x00, 0x04, 0x00, 0x00, // Fill + APP0
            0xFF, 0xFF, 0xC0, 0x00, 0x0B, 0x08, 0x00, 0xF0, 0x01, 0x40, 0x01, 0x01, 0x11, 0x00,
            0xFF, 0xD9,
        ];
        assert_eq!(parse_jpeg_dimensions(&fill_bytes_stream), Some((320, 240)));

        // 9. Embedded EXIF thumbnail inside APP1 does not spoof main SOF0
        let exif_thumb_stream = [
            0xFF, 0xD8, // Main SOI
            // APP1 segment (seg_len = 2 + 17 = 19 = 0x0013) containing a
            // 160x120 thumbnail JPEG with its own SOI, SOF0, and EOI:
            0xFF, 0xE1, 0x00, 0x13, // APP1 header
            0xFF, 0xD8, 0xFF, 0xC0, 0x00, 0x0B, 0x08, 0x00, 0x78, 0x00, 0xA0, 0x01, 0x01, 0x11,
            0x00, 0xFF, 0xD9, // Thumbnail (160x120)
            // Main image SOF0 (1280x720, Nf=1, seg_len=11):
            0xFF, 0xC0, 0x00, 0x0B, 0x08, 0x02, 0xD0, 0x05, 0x00, 0x01, 0x01, 0x11, 0x00, 0xFF,
            0xD9,
        ];
        assert_eq!(parse_jpeg_dimensions(&exif_thumb_stream), Some((1280, 720)));
    }

    #[test]
    fn test_extract_jpeg_payload_and_validate_frame() {
        assert_eq!(extract_jpeg_payload(&[]), None);
        assert_eq!(extract_jpeg_payload(&[0xFF, 0xD8]), None);
        assert_eq!(extract_jpeg_payload(&[0xFF, 0xD8, 0xFF]), None);

        // Inverted: EOI before SOI
        let inverted = [0xFF, 0xD9, 0x00, 0xFF, 0xD8, 0xFF, 0x00];
        assert_eq!(extract_jpeg_payload(&inverted), None);
        assert_eq!(validate_jpeg_frame(&inverted), None);

        // Minimal valid payload without SOF
        let minimal = [0xFF, 0xD8, 0xFF, 0xD9];
        assert_eq!(extract_jpeg_payload(&minimal), Some(&minimal[..]));
        assert_eq!(validate_jpeg_frame(&minimal), None);

        // Frame with leading and trailing bytes + SOF0
        let mut framed = [0x55u8; 32];
        let valid_stream = [
            0xFF, 0xD8, 0xFF, 0xC0, 0x00, 0x0B, 0x08, 0x00, 0xF0, 0x01, 0x40, 0x01, 0x01, 0x11,
            0x00, 0xFF, 0xD9,
        ];
        framed[4..4 + valid_stream.len()].copy_from_slice(&valid_stream);
        let extracted = extract_jpeg_payload(&framed);
        assert_eq!(extracted, Some(&valid_stream[..]));
        assert_eq!(validate_jpeg_frame(&framed), Some((320, 240)));
    }
}
