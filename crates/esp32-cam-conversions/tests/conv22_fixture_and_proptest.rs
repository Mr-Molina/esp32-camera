//! Integration tests for `CONV-22`: external JPEG fixture verification,
//! SWAR scanner `proptest` equivalence, and untrusted input mutation-fuzzing.
//!
//! All checked-in JPEG fixtures in `tests/fixtures/` originate from external
//! encoders (`Pillow`/`libjpeg` and historical upstream `a364a6a` test
//! captures) so decoder verification is never circular against our encoder.
//! Live `172.22.1.52` OV2640 hardware capture remains gated on `[HIL]` network
//! reachability (`README §2.1 Rule 5`).

// Not run under Miri. proptest persists failing cases to a file next to the
// test source, which calls `getcwd`; Miri's isolation forbids that and aborts
// the whole binary. Even with isolation off, hundreds of generated cases per
// property under the interpreter take longer than the job limit. The unit
// tests in src/ cover the same scanners and parsers under Miri with fixed
// inputs.
#![cfg(not(miri))]

#[cfg(test)]
mod conv22_tests {
    use esp32_cam_conversions::{
        decode_jpeg_to_rgb888, encode_rgb888_to_jpeg, frame_to_bmp, parse_jpeg_dimensions,
        validate_jpeg_frame, verify_jpeg_eoi, verify_jpeg_soi, PixelFormat, BMP_HEADER_LEN,
        JPEG_EOI_MARKER, JPEG_SOI_MARKER,
    };
    use proptest::prelude::{any, ProptestConfig};
    use proptest::{collection, prop_assert_eq, proptest, sample};

    const FIXTURE_W: u16 = 160;
    const FIXTURE_H: u16 = 120;
    const RGB_LEN: usize = (FIXTURE_W as usize) * (FIXTURE_H as usize) * 3;

    const OV2640_QQVGA_H2V1_FIXTURE: &[u8] = include_bytes!("fixtures/ov2640_qqvga_h2v1.jpg");
    const ESP32_CAM_INSIDE_QVGA_FIXTURE: &[u8] =
        include_bytes!("fixtures/esp32_cam_test_inside_qvga.jpg");
    const LIBJPEG_ODD_227X149_FIXTURE: &[u8] = include_bytes!("fixtures/libjpeg_odd_227x149.jpg");

    fn build_reference_rgb() -> Vec<u8> {
        let w = usize::from(FIXTURE_W);
        let h = usize::from(FIXTURE_H);
        let mut rgb = vec![0u8; RGB_LEN];
        for y in 0..h {
            for x in 0..w {
                let r = ((x * 255) / (w - 1)) as u8;
                let g = ((y * 255) / (h - 1)) as u8;
                let b = (((x + y) * 127) / (w + h - 2)) as u8;
                let idx = (y * w + x) * 3;
                rgb[idx] = r;
                rgb[idx + 1] = g;
                rgb[idx + 2] = b;
            }
        }
        rgb
    }

    fn collect_collapsed_header_markers(jpeg: &[u8]) -> Vec<u8> {
        assert!(
            jpeg.len() >= 4 && jpeg[0] == 0xFF && jpeg[1] == 0xD8,
            "JPEG buffer must start with SOI"
        );
        let mut out = vec![0xD8u8];
        let mut idx = 2usize;
        while idx < jpeg.len() {
            assert_eq!(jpeg[idx], 0xFF, "expected marker prefix 0xFF at {idx}");
            while idx < jpeg.len() && jpeg[idx] == 0xFF {
                idx += 1;
            }
            assert!(idx < jpeg.len(), "truncated marker byte at end of header");
            let marker = jpeg[idx];
            idx += 1;
            if out.last().copied() != Some(marker) {
                out.push(marker);
            }
            if marker == 0xDA || marker == 0xD9 {
                break;
            }
            assert!(
                idx + 2 <= jpeg.len(),
                "marker 0xFF{marker:02X} missing segment length"
            );
            let seg_len = usize::from(u16::from_be_bytes([jpeg[idx], jpeg[idx + 1]]));
            assert!(seg_len >= 2, "segment length {seg_len} must be >= 2");
            idx += seg_len;
        }
        out
    }

    fn compute_psnr(ref_rgb: &[u8], dec_rgb: &[u8]) -> f64 {
        assert_eq!(ref_rgb.len(), dec_rgb.len(), "RGB slice lengths must match");
        let mut sse = 0u64;
        for (&a, &b) in ref_rgb.iter().zip(dec_rgb.iter()) {
            let diff = i64::from(a) - i64::from(b);
            sse += (diff * diff) as u64;
        }
        if sse == 0 {
            return 99.0;
        }
        let mse = (sse as f64) / (ref_rgb.len() as f64);
        10.0 * ((255.0 * 255.0) / mse).log10()
    }

    fn naive_verify_jpeg_eoi_forward(buf: &[u8]) -> Option<usize> {
        buf.windows(2).position(|w| w == JPEG_EOI_MARKER)
    }

    fn naive_verify_jpeg_eoi_backward(buf: &[u8]) -> Option<usize> {
        buf.windows(2).rposition(|w| w == JPEG_EOI_MARKER)
    }

    #[test]
    fn test_ov2640_qqvga_h2v1_fixture_psnr_and_headers() {
        let ref_rgb = build_reference_rgb();
        assert_eq!(
            collect_collapsed_header_markers(OV2640_QQVGA_H2V1_FIXTURE),
            vec![0xD8, 0xDB, 0xC0, 0xC4, 0xDA],
            "APP0-stripped OV2640-layout fixture must follow SOI->DQT->SOF0->DHT->SOS"
        );
        assert_eq!(
            verify_jpeg_eoi(OV2640_QQVGA_H2V1_FIXTURE, false),
            Some(OV2640_QQVGA_H2V1_FIXTURE.len() - 2),
            "OV2640-layout fixture must terminate with EOI"
        );
        assert_eq!(
            validate_jpeg_frame(OV2640_QQVGA_H2V1_FIXTURE),
            Some((FIXTURE_W, FIXTURE_H)),
            "external Pillow/libjpeg 4:2:2 fixture must pass validate_jpeg_frame"
        );
        assert_eq!(
            parse_jpeg_dimensions(OV2640_QQVGA_H2V1_FIXTURE),
            Some((FIXTURE_W, FIXTURE_H)),
            "fixture dimensions must parse as 160x120"
        );
        let sof_pos = OV2640_QQVGA_H2V1_FIXTURE
            .windows(2)
            .position(|w| w == [0xFF, 0xC0])
            .expect("fixture must contain SOF0 marker");
        assert_eq!(
            OV2640_QQVGA_H2V1_FIXTURE[sof_pos + 11],
            0x21,
            "SOF0 Y sampling factor must be 0x21 (4:2:2 H2V1)"
        );

        let mut decoded = vec![0u8; RGB_LEN];
        let dec_len = decode_jpeg_to_rgb888(OV2640_QQVGA_H2V1_FIXTURE, &mut decoded)
            .expect("decode_jpeg_to_rgb888 must succeed on external H2V1 fixture");
        assert_eq!(dec_len, RGB_LEN, "decoded RGB888 byte length must match");
        let psnr = compute_psnr(&ref_rgb, &decoded);
        assert!(
            psnr > 30.0,
            "external H2V1 fixture PSNR must exceed 30 dB, got {psnr:.2} dB"
        );
    }

    #[test]
    fn test_upstream_esp32_cam_inside_qvga_fixture() {
        assert_eq!(
            ESP32_CAM_INSIDE_QVGA_FIXTURE.len(),
            18_832,
            "upstream test_inside.jpeg fixture must match 18,832-byte payload"
        );
        let markers = collect_collapsed_header_markers(ESP32_CAM_INSIDE_QVGA_FIXTURE);
        assert!(
            markers.contains(&0xE0) && markers.contains(&0xE1),
            "upstream camera capture must exercise APP0 + APP1 Exif segments: {markers:?}"
        );
        assert_eq!(
            parse_jpeg_dimensions(ESP32_CAM_INSIDE_QVGA_FIXTURE),
            Some((320, 240)),
            "upstream camera fixture dimensions must parse as 320x240 QVGA"
        );
        assert_eq!(
            validate_jpeg_frame(ESP32_CAM_INSIDE_QVGA_FIXTURE),
            Some((320, 240)),
            "upstream camera fixture must pass validate_jpeg_frame"
        );

        let expected_len = 320 * 240 * 3;
        let mut decoded = vec![0u8; expected_len];
        let dec_len = decode_jpeg_to_rgb888(ESP32_CAM_INSIDE_QVGA_FIXTURE, &mut decoded)
            .expect("decode_jpeg_to_rgb888 must decode upstream 320x240 camera capture");
        assert_eq!(
            dec_len, expected_len,
            "decoded QVGA RGB888 byte length must equal 320x240x3"
        );
    }

    #[test]
    fn test_upstream_libjpeg_odd_227x149_fixture() {
        assert_eq!(
            LIBJPEG_ODD_227X149_FIXTURE.len(),
            5_764,
            "upstream libjpeg testimg.jpeg fixture must match 5,764-byte payload"
        );
        assert_eq!(
            parse_jpeg_dimensions(LIBJPEG_ODD_227X149_FIXTURE),
            Some((227, 149)),
            "upstream libjpeg odd-MCU fixture dimensions must parse as 227x149"
        );
        assert_eq!(
            validate_jpeg_frame(LIBJPEG_ODD_227X149_FIXTURE),
            Some((227, 149)),
            "upstream libjpeg 227x149 fixture must pass validate_jpeg_frame"
        );

        let expected_len = 227 * 149 * 3;
        let mut decoded = vec![0u8; expected_len];
        let dec_len = decode_jpeg_to_rgb888(LIBJPEG_ODD_227X149_FIXTURE, &mut decoded)
            .expect("decode_jpeg_to_rgb888 must decode 227x149 odd-MCU libjpeg fixture");
        assert_eq!(
            dec_len, expected_len,
            "decoded 227x149 RGB888 byte length must equal 227x149x3"
        );
    }

    #[test]
    fn test_conv22_mutation_fuzz_sweep() {
        let mut state = 0x9E37_79B9_7F4A_7C15u64;
        let mut next_u64 = || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            state
        };
        let mut scratch_rgb = vec![0u8; 64 * 64 * 3];
        let mut mutated = Vec::with_capacity(OV2640_QQVGA_H2V1_FIXTURE.len() + 16);

        for iter in 0..5_000usize {
            mutated.clear();
            let cut = (next_u64() as usize) % (OV2640_QQVGA_H2V1_FIXTURE.len() + 1);
            mutated.extend_from_slice(&OV2640_QQVGA_H2V1_FIXTURE[..cut]);
            let edits = 1 + ((next_u64() as usize) % 6);
            for _ in 0..edits {
                if !mutated.is_empty() {
                    let idx = (next_u64() as usize) % mutated.len();
                    mutated[idx] ^= (next_u64() & 0xFF) as u8;
                }
            }
            if iter % 3 == 0 && !mutated.is_empty() {
                let idx = (next_u64() as usize) % mutated.len();
                mutated[idx] = 0xFF;
            }
            let _ = verify_jpeg_soi(&mutated);
            let _ = verify_jpeg_eoi(&mutated, false);
            let _ = verify_jpeg_eoi(&mutated, true);
            let dims = parse_jpeg_dimensions(&mutated);
            let _ = validate_jpeg_frame(&mutated);
            if dims.is_none_or(|(w, h)| usize::from(w).saturating_mul(usize::from(h)) <= 64 * 64) {
                let _ = decode_jpeg_to_rgb888(&mutated, &mut scratch_rgb);
            }
        }
    }

    #[test]
    fn test_end_to_end_jpeg_to_bmp_via_frame_to_bmp() {
        let width = 8u16;
        let height = 8u16;
        let mut rgb_in = [0u8; 8 * 8 * 3];
        for i in 0..64 {
            rgb_in[i * 3] = 64;
            rgb_in[i * 3 + 1] = 128;
            rgb_in[i * 3 + 2] = 192;
        }

        let mut jpeg_buf = [0u8; 2048];
        let jpeg_len = encode_rgb888_to_jpeg(&rgb_in, &mut jpeg_buf, width, height, 90)
            .expect("encode_rgb888_to_jpeg should succeed");
        let jpeg_slice = &jpeg_buf[..jpeg_len];

        assert_eq!(
            verify_jpeg_soi(jpeg_slice),
            Some(0),
            "SOI marker must start at byte 0"
        );
        assert_eq!(
            verify_jpeg_eoi(jpeg_slice, true),
            Some(jpeg_len - 2),
            "EOI marker must appear at final 2 bytes"
        );
        assert_eq!(
            parse_jpeg_dimensions(jpeg_slice),
            Some((8, 8)),
            "8x8 JPEG dimensions must round-trip"
        );

        let mut bmp_buf = [0u8; BMP_HEADER_LEN + 8 * 8 * 3];
        let bmp_len = frame_to_bmp(jpeg_slice, &mut bmp_buf, width, height, PixelFormat::Jpeg)
            .expect("frame_to_bmp with PixelFormat::Jpeg should succeed");
        assert_eq!(
            bmp_len,
            BMP_HEADER_LEN + 8 * 8 * 3,
            "BMP output length must equal header plus RGB888 payload"
        );
        assert_eq!(&bmp_buf[0..2], b"BM", "BMP magic header must be BM");
        assert_eq!(
            u32::from_le_bytes([bmp_buf[2], bmp_buf[3], bmp_buf[4], bmp_buf[5]]) as usize,
            bmp_len,
            "BMP header file-size field must match serialized byte count"
        );
    }

    #[test]
    fn test_non_multiple_of_8_jpeg_dimensions_roundtrip() {
        let width = 10u16;
        let height = 6u16;
        let rgb_in = [150u8; 10 * 6 * 3];
        let mut jpeg_buf = [0u8; 2048];
        let jpeg_len = encode_rgb888_to_jpeg(&rgb_in, &mut jpeg_buf, width, height, 85)
            .expect("encode_rgb888_to_jpeg should succeed");
        let jpeg_slice = &jpeg_buf[..jpeg_len];

        assert_eq!(
            parse_jpeg_dimensions(jpeg_slice),
            Some((10, 6)),
            "non-MCU-aligned 10x6 dimensions must parse accurately"
        );

        let mut rgb_out = [0u8; 10 * 6 * 3];
        let dec_len = decode_jpeg_to_rgb888(jpeg_slice, &mut rgb_out)
            .expect("decode_jpeg_to_rgb888 should succeed");
        assert_eq!(
            dec_len,
            10 * 6 * 3,
            "decoded RGB888 length must match 10x6x3"
        );
        for &c in &rgb_out {
            assert!(
                (i32::from(c) - 150).abs() <= 8,
                "decoded channel {c} must stay within +/-8 of 150"
            );
        }
    }

    proptest! {
        #![proptest_config(ProptestConfig::with_cases(512))]

        #[test]
        fn proptest_swar_scanners_and_untrusted_parsers(
            buf in collection::vec(any::<u8>(), 0..512),
        ) {
            prop_assert_eq!(
                verify_jpeg_soi(&buf),
                buf.windows(3).position(|w| w == JPEG_SOI_MARKER)
            );
            prop_assert_eq!(
                verify_jpeg_eoi(&buf, true),
                naive_verify_jpeg_eoi_forward(&buf)
            );
            prop_assert_eq!(
                verify_jpeg_eoi(&buf, false),
                naive_verify_jpeg_eoi_backward(&buf)
            );
            let dims = parse_jpeg_dimensions(&buf);
            let _ = validate_jpeg_frame(&buf);
            if dims.is_none_or(|(w, h)| {
                usize::from(w).saturating_mul(usize::from(h)) <= 64 * 64
            }) {
                let mut scratch_rgb = vec![0u8; 64 * 64 * 3];
                let _ = decode_jpeg_to_rgb888(&buf, &mut scratch_rgb);
            }
        }

        #[test]
        fn proptest_swar_scanners_marker_dense_alphabet(
            buf in collection::vec(
                sample::select(&[0x00u8, 0x01, 0xC0, 0xD8, 0xD9, 0xFF][..]),
                0..512,
            ),
        ) {
            prop_assert_eq!(
                verify_jpeg_soi(&buf),
                buf.windows(3).position(|w| w == JPEG_SOI_MARKER)
            );
            prop_assert_eq!(
                verify_jpeg_eoi(&buf, true),
                naive_verify_jpeg_eoi_forward(&buf)
            );
            prop_assert_eq!(
                verify_jpeg_eoi(&buf, false),
                naive_verify_jpeg_eoi_backward(&buf)
            );
            let _ = parse_jpeg_dimensions(&buf);
            let _ = validate_jpeg_frame(&buf);
        }
    }
}
