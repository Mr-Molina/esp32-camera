//! Fuzz target for JPEG SOI/EOI scanning, frame validation, dimension
//! parsing, and baseline JPEG decoding (`CONV-22`, `M-126`).

#![no_main]

use esp32_cam_conversions::{
    decode_jpeg_to_rgb565, decode_jpeg_to_rgb888, eoi_probe_window, extract_jpeg_payload,
    frame_to_bmp, parse_jpeg_dimensions, validate_jpeg_frame, verify_jpeg_eoi, verify_jpeg_soi,
    PixelFormat, BMP_HEADER_LEN, JPEG_EOI_MARKER, JPEG_SOI_MARKER,
};
use libfuzzer_sys::fuzz_target;

const MAX_FUZZ_OUT_BYTES: usize = 64 * 1024;

fuzz_target!(|data: &[u8]| {
    let window = eoi_probe_window(data.len() / 2, data.len());
    assert!(
        window <= data.len(),
        "eoi_probe_window exceeded frame length: window={window}, len={}",
        data.len()
    );

    // Exercise eoi_probe_window edge bounds and fuzz-driven (half, frame_len).
    let max_half_window = eoi_probe_window(usize::MAX, data.len());
    assert_eq!(
        max_half_window,
        data.len(),
        "eoi_probe_window(usize::MAX, len) must clamp to len"
    );
    let max_frame_window = eoi_probe_window(data.len(), usize::MAX);
    assert!(
        max_frame_window >= data.len(),
        "eoi_probe_window(len, usize::MAX) must be >= len"
    );
    if data.len() >= 16 {
        let half = u64::from_le_bytes([
            data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7],
        ]) as usize;
        let frame_len = u64::from_le_bytes([
            data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15],
        ]) as usize;
        let driven_window = eoi_probe_window(half, frame_len);
        assert!(
            driven_window <= frame_len,
            "eoi_probe_window({half}, {frame_len}) exceeded frame_len: {driven_window}"
        );
    }

    if let Some(soi) = verify_jpeg_soi(data) {
        assert!(
            soi + JPEG_SOI_MARKER.len() <= data.len(),
            "SOI offset out of bounds: soi={soi}, len={}",
            data.len()
        );
        assert_eq!(
            &data[soi..soi + JPEG_SOI_MARKER.len()],
            JPEG_SOI_MARKER.as_slice(),
            "SOI slice mismatch at offset {soi}"
        );
        if soi > 0 {
            assert!(
                verify_jpeg_soi(&data[..soi + JPEG_SOI_MARKER.len() - 1]).is_none(),
                "verify_jpeg_soi did not return earliest SOI offset {soi}"
            );
        }
    }

    let fwd = verify_jpeg_eoi(data, true);
    let bwd = verify_jpeg_eoi(data, false);
    assert_eq!(
        fwd.is_some(),
        bwd.is_some(),
        "forward and backward EOI existence diverged"
    );

    if let Some(fwd_eoi) = fwd {
        assert!(
            fwd_eoi + JPEG_EOI_MARKER.len() <= data.len(),
            "forward EOI offset out of bounds: eoi={fwd_eoi}, len={}",
            data.len()
        );
        assert_eq!(
            &data[fwd_eoi..fwd_eoi + JPEG_EOI_MARKER.len()],
            JPEG_EOI_MARKER.as_slice(),
            "forward EOI slice mismatch at offset {fwd_eoi}"
        );
    }

    if let Some(bwd_eoi) = bwd {
        assert!(
            bwd_eoi + JPEG_EOI_MARKER.len() <= data.len(),
            "backward EOI offset out of bounds: eoi={bwd_eoi}, len={}",
            data.len()
        );
        assert_eq!(
            &data[bwd_eoi..bwd_eoi + JPEG_EOI_MARKER.len()],
            JPEG_EOI_MARKER.as_slice(),
            "backward EOI slice mismatch at offset {bwd_eoi}"
        );
    }

    if let (Some(fwd_eoi), Some(bwd_eoi)) = (fwd, bwd) {
        assert!(
            fwd_eoi <= bwd_eoi,
            "forward EOI ({fwd_eoi}) exceeded backward EOI ({bwd_eoi})"
        );
    }

    if let Some((width, height)) = parse_jpeg_dimensions(data) {
        assert!(
            width > 0 && height > 0,
            "parse_jpeg_dimensions returned zero dimension: ({width}, {height})"
        );
    }

    if let Some(payload) = extract_jpeg_payload(data) {
        assert!(
            payload.len() >= 4,
            "extracted JPEG payload shorter than minimum markers: len={}",
            payload.len()
        );
        assert_eq!(
            &payload[..JPEG_SOI_MARKER.len()],
            JPEG_SOI_MARKER.as_slice(),
            "extracted payload missing leading SOI marker"
        );
        assert_eq!(
            &payload[payload.len() - JPEG_EOI_MARKER.len()..],
            JPEG_EOI_MARKER.as_slice(),
            "extracted payload missing trailing EOI marker"
        );
    }

    if let Some((width, height)) = validate_jpeg_frame(data) {
        assert!(
            width > 0 && height > 0,
            "validated frame returned zero dimension: ({width}, {height})"
        );
        assert!(
            extract_jpeg_payload(data).is_some(),
            "validated frame must have an extractable payload"
        );
    }

    // Exercise the full JPEG bitstream decoder paths with bounded buffers.
    let mut out_buf = vec![0u8; MAX_FUZZ_OUT_BYTES];
    if let Ok(written) = decode_jpeg_to_rgb888(data, &mut out_buf) {
        assert!(
            written <= out_buf.len(),
            "decode_jpeg_to_rgb888 wrote {written} > {}",
            out_buf.len()
        );
    }
    if let Ok(written) = decode_jpeg_to_rgb565(data, &mut out_buf) {
        assert!(
            written <= out_buf.len(),
            "decode_jpeg_to_rgb565 wrote {written} > {}",
            out_buf.len()
        );
    }
    if let Ok(written) = frame_to_bmp(data, &mut out_buf, 0, 0, PixelFormat::Jpeg) {
        assert!(
            written >= BMP_HEADER_LEN && written <= out_buf.len(),
            "frame_to_bmp wrote invalid length {written}"
        );
        assert_eq!(&out_buf[..2], b"BM", "BMP output missing BM signature");
    }
});
