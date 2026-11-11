//! Integration tests for `CONV-22`: 4:2:2 (`H2V1`) JPEG fixture PSNR
//! verification, SWAR scanner `proptest` equivalence, and untrusted input
//! mutation-fuzzing.

#[cfg(test)]
mod conv22_tests {
    use esp32_cam_conversions::{
        decode_jpeg_to_rgb888, encode_rgb888_to_jpeg, encode_yuv422_to_jpeg_subsampled,
        frame_to_bmp, max_encoded_len, parse_jpeg_dimensions, rgb2yuv_with_range,
        validate_jpeg_frame, verify_jpeg_eoi, verify_jpeg_soi, PixelFormat, Subsampling, YuvRange,
        BMP_HEADER_LEN, JPEG_EOI_MARKER, JPEG_SOI_MARKER,
    };
    use proptest::prelude::{any, ProptestConfig};
    use proptest::{collection, prop_assert_eq, proptest, sample};

    const FIXTURE_W: u16 = 160;
    const FIXTURE_H: u16 = 120;
    const RGB_LEN: usize = (FIXTURE_W as usize) * (FIXTURE_H as usize) * 3;
    const YUYV_LEN: usize = (FIXTURE_W as usize) * (FIXTURE_H as usize) * 2;
    const OV2640_QQVGA_H2V1_FIXTURE: &[u8] = include_bytes!("fixtures/ov2640_qqvga_h2v1.jpg");

    fn build_reference_rgb_and_yuyv() -> (Vec<u8>, Vec<u8>) {
        let w = usize::from(FIXTURE_W);
        let h = usize::from(FIXTURE_H);
        let mut rgb = vec![0u8; RGB_LEN];
        let mut yuyv = vec![0u8; YUYV_LEN];
        for y in 0..h {
            for x_pair in 0..(w / 2) {
                let x0 = x_pair * 2;
                let x1 = x0 + 1;
                let r0 = ((x0 * 255) / (w - 1)) as u8;
                let g0 = ((y * 255) / (h - 1)) as u8;
                let b0 = (((x0 + y) * 127) / (w + h - 2)) as u8;
                let r1 = ((x1 * 255) / (w - 1)) as u8;
                let g1 = g0;
                let b1 = (((x1 + y) * 127) / (w + h - 2)) as u8;

                let rgb0 = (y * w + x0) * 3;
                rgb[rgb0] = r0;
                rgb[rgb0 + 1] = g0;
                rgb[rgb0 + 2] = b0;
                let rgb1 = (y * w + x1) * 3;
                rgb[rgb1] = r1;
                rgb[rgb1 + 1] = g1;
                rgb[rgb1 + 2] = b1;

                let (y0, u0, v0) = rgb2yuv_with_range(r0, g0, b0, YuvRange::Full);
                let (y1, u1, v1) = rgb2yuv_with_range(r1, g1, b1, YuvRange::Full);
                let u_avg = (u16::from(u0) + u16::from(u1)).div_ceil(2) as u8;
                let v_avg = (u16::from(v0) + u16::from(v1)).div_ceil(2) as u8;

                let yuyv_idx = (y * w + x0) * 2;
                yuyv[yuyv_idx] = y0;
                yuyv[yuyv_idx + 1] = u_avg;
                yuyv[yuyv_idx + 2] = y1;
                yuyv[yuyv_idx + 3] = v_avg;
            }
        }
        (rgb, yuyv)
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
        let (ref_rgb, yuyv) = build_reference_rgb_and_yuyv();
        let cap =
            max_encoded_len(FIXTURE_W, FIXTURE_H, 3).expect("QQVGA max_encoded_len must succeed");
        let mut regen = vec![0u8; cap];
        let regen_len = encode_yuv422_to_jpeg_subsampled(
            &yuyv,
            &mut regen,
            FIXTURE_W,
            FIXTURE_H,
            90,
            Subsampling::H2V1,
        )
        .expect("encode_yuv422_to_jpeg_subsampled must succeed");
        assert_eq!(
            OV2640_QQVGA_H2V1_FIXTURE,
            &regen[..regen_len],
            "checked-in 4:2:2 fixture must match deterministic H2V1 encoding"
        );
        assert_eq!(
            validate_jpeg_frame(OV2640_QQVGA_H2V1_FIXTURE),
            Some((FIXTURE_W, FIXTURE_H)),
            "checked-in 4:2:2 fixture must pass validate_jpeg_frame"
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
            .expect("decode_jpeg_to_rgb888 must succeed on H2V1 fixture");
        assert_eq!(dec_len, RGB_LEN, "decoded RGB888 byte length must match");
        let psnr = compute_psnr(&ref_rgb, &decoded);
        assert!(
            psnr > 30.0,
            "H2V1 fixture PSNR must exceed 30 dB, got {psnr:.2} dB"
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
