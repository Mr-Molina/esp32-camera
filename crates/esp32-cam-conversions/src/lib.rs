//! Safe `#![no_std]` image conversions, BMP serializer, and JPEG marker/codec utilities.

#![no_std]
#![forbid(unsafe_code)]

pub mod bmp;
pub mod jpeg_codec;
pub mod jpeg_scan;
pub mod yuv;

pub use bmp::{
    BMP_GRAYSCALE_PALETTE_LEN, BMP_HEADER_LEN, frame_to_bmp, grayscale_to_bmp, rgb565_to_bmp,
    rgb888_to_bmp, write_bmp_header, yuv422_to_bmp,
};
pub use esp32_cam_types::{CameraError, FrameSize, PixelFormat};
pub use jpeg_codec::{
    decode_jpeg_to_rgb565, decode_jpeg_to_rgb888, encode_grayscale_to_jpeg, encode_rgb888_to_jpeg,
    encode_yuv422_to_jpeg,
};
pub use jpeg_scan::{
    JPEG_EOI_MARKER, JPEG_SOI_MARKER, eoi_probe_window, parse_jpeg_dimensions, verify_jpeg_eoi,
    verify_jpeg_soi,
};
pub use yuv::{
    rgb565_to_yuv422, yuv2rgb, yuv422_to_rgb565, yuv422_to_rgb888, yuv422_to_yuv420,
};

#[cfg(test)]
mod tests {
    use super::*;

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

        assert_eq!(verify_jpeg_soi(jpeg_slice), Some(0));
        assert_eq!(verify_jpeg_eoi(jpeg_slice, true), Some(jpeg_len - 2));
        assert_eq!(parse_jpeg_dimensions(jpeg_slice), Some((8, 8)));

        let mut bmp_buf = [0u8; BMP_HEADER_LEN + 8 * 8 * 3];
        let bmp_len = frame_to_bmp(jpeg_slice, &mut bmp_buf, width, height, PixelFormat::Jpeg)
            .expect("frame_to_bmp with PixelFormat::Jpeg should succeed");
        assert_eq!(bmp_len, BMP_HEADER_LEN + 8 * 8 * 3);
        assert_eq!(&bmp_buf[0..2], b"BM");
        assert_eq!(
            u32::from_le_bytes([bmp_buf[2], bmp_buf[3], bmp_buf[4], bmp_buf[5]]) as usize,
            bmp_len
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

        assert_eq!(parse_jpeg_dimensions(jpeg_slice), Some((10, 6)));

        let mut rgb_out = [0u8; 10 * 6 * 3];
        let dec_len = decode_jpeg_to_rgb888(jpeg_slice, &mut rgb_out)
            .expect("decode_jpeg_to_rgb888 should succeed");
        assert_eq!(dec_len, 10 * 6 * 3);
        for &c in &rgb_out {
            assert!((c as i32 - 150).abs() <= 8);
        }
    }
}

