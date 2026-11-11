//! Safe `#![no_std]` image conversions, BMP serializer, and JPEG marker/codec
//! utilities.

#![no_std]
#![forbid(unsafe_code)]

pub mod bmp;
pub mod jpeg_codec;
pub mod jpeg_scan;
pub mod yuv;

pub use crate::bmp::{
    frame_to_bmp, grayscale_to_bmp, rgb565_to_bmp, rgb888_to_bmp, write_bmp_header, yuv422_to_bmp,
    BMP_GRAYSCALE_PALETTE_LEN, BMP_HEADER_LEN,
};
pub use crate::jpeg_codec::{
    decode_jpeg_to_rgb565, decode_jpeg_to_rgb888, encode_grayscale_to_jpeg, encode_rgb888_to_jpeg,
    encode_yuv422_to_jpeg, encode_yuv422_to_jpeg_subsampled, encode_yuv422_to_jpeg_with_range,
    max_encoded_len, Subsampling,
};
pub use crate::jpeg_scan::{
    eoi_probe_window, extract_jpeg_payload, parse_jpeg_dimensions, validate_jpeg_frame,
    verify_jpeg_eoi, verify_jpeg_soi, JPEG_EOI_MARKER, JPEG_SOI_MARKER,
};
pub use crate::yuv::{
    rgb2yuv_with_range, rgb565_to_yuv422, rgb565_to_yuv422_with_range, unpack_rgb565_be_full,
    yuv2rgb, yuv2rgb_with_range, yuv422_pair_to_rgb, yuv422_pair_to_rgb_with_range,
    yuv422_to_grayscale, yuv422_to_rgb565, yuv422_to_rgb565_with_range, yuv422_to_rgb888,
    yuv422_to_rgb888_with_range, yuv422_to_yuv420, YuvRange,
};
pub use esp32_cam_types::{CameraError, FrameSize, PixelFormat};
