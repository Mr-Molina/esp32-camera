//! Fuzz target for JPEG SOI/EOI scanning and frame validation (`CONV-22`).

#![no_main]

use esp32_cam_conversions::{
    eoi_probe_window, extract_jpeg_payload, validate_jpeg_frame, verify_jpeg_eoi, verify_jpeg_soi,
    JPEG_EOI_MARKER, JPEG_SOI_MARKER,
};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    let window = eoi_probe_window(data.len() / 2, data.len());
    assert!(
        window <= data.len(),
        "eoi_probe_window exceeded frame length: window={window}, len={}",
        data.len()
    );

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
    }

    if let Some(fwd_eoi) = verify_jpeg_eoi(data, true) {
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

    if let Some(bwd_eoi) = verify_jpeg_eoi(data, false) {
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
});
