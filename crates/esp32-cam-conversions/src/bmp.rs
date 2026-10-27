//! Safe `#![no_std]` little-endian BMP serializer with checked arithmetic overflow guards.
//!
//! Replaces `conversions/to_bmp.c` (`CONV-101`..`CONV-105`), eliminating unaligned
//! packed struct pointer casts and integer overflow vulnerabilities.

use crate::yuv::yuv2rgb;
use esp32_cam_types::{CameraError, PixelFormat};

/// Standard Windows BITMAPFILEHEADER (14 bytes) + BITMAPINFOHEADER (40 bytes) length.
pub const BMP_HEADER_LEN: usize = 54;

/// Size of the 256-entry BGRA palette required for 8-bit grayscale BMP images.
pub const BMP_GRAYSCALE_PALETTE_LEN: usize = 256 * 4;

#[derive(Debug, Clone, Copy)]
struct BmpLayout {
    src_required: usize,
    row_bytes: usize,
    padding: usize,
    image_size: u32,
    palette_size: u32,
    total_size: usize,
}

/// Computes and validates BMP buffer layout using checked 32-bit arithmetic (`CONV-101`..`CONV-105`).
fn compute_bmp_layout(
    width: u16,
    height: u16,
    in_bpp: u32,
    out_bpp: u32,
    palette_size: u32,
) -> Result<BmpLayout, CameraError> {
    if width == 0 || height == 0 {
        return Err(CameraError::InvalidArg);
    }

    let w32 = width as u32;
    let h32 = height as u32;

    let pixels32 = w32.checked_mul(h32).ok_or(CameraError::Overflow)?;
    let src_req32 = pixels32.checked_mul(in_bpp).ok_or(CameraError::Overflow)?;

    let row_bytes32 = w32.checked_mul(out_bpp).ok_or(CameraError::Overflow)?;
    let row_stride32 = row_bytes32
        .checked_add(3)
        .ok_or(CameraError::Overflow)?
        & !3u32;
    let image_size32 = row_stride32.checked_mul(h32).ok_or(CameraError::Overflow)?;
    let total_size32 = (BMP_HEADER_LEN as u32)
        .checked_add(palette_size)
        .and_then(|v| v.checked_add(image_size32))
        .ok_or(CameraError::Overflow)?;

    // Reject sizes exceeding signed 32-bit max to prevent signed offset overflow in BMP readers.
    if total_size32 > i32::MAX as u32 || src_req32 > i32::MAX as u32 {
        return Err(CameraError::Overflow);
    }

    let src_required = usize::try_from(src_req32).map_err(|_| CameraError::Overflow)?;
    let row_bytes = usize::try_from(row_bytes32).map_err(|_| CameraError::Overflow)?;
    let row_stride = usize::try_from(row_stride32).map_err(|_| CameraError::Overflow)?;
    let total_size = usize::try_from(total_size32).map_err(|_| CameraError::Overflow)?;

    Ok(BmpLayout {
        src_required,
        row_bytes,
        padding: row_stride - row_bytes,
        image_size: image_size32,
        palette_size,
        total_size,
    })
}

/// Serializes the 54-byte Windows BMP header (`BM` magic + 12-byte file header + 40-byte DIB header)
/// in little-endian byte order into `dst[..54]`.
///
/// The height field is written as `-height` to specify top-to-bottom scanline order,
/// matching `conversions/to_bmp.c`.
pub fn write_bmp_header(
    dst: &mut [u8],
    width: u16,
    height: u16,
    bits_per_pixel: u16,
    image_size: u32,
    palette_size: u32,
) -> Result<usize, CameraError> {
    if width == 0 || height == 0 || (bits_per_pixel != 8 && bits_per_pixel != 24) {
        return Err(CameraError::InvalidArg);
    }
    let pixel_offset = (BMP_HEADER_LEN as u32)
        .checked_add(palette_size)
        .ok_or(CameraError::Overflow)?;
    let file_size = pixel_offset
        .checked_add(image_size)
        .ok_or(CameraError::Overflow)?;
    if file_size > i32::MAX as u32 {
        return Err(CameraError::Overflow);
    }
    if dst.len() < BMP_HEADER_LEN {
        return Err(CameraError::BufferTooSmall);
    }

    dst[0] = b'B';
    dst[1] = b'M';
    dst[2..6].copy_from_slice(&file_size.to_le_bytes());
    dst[6..10].copy_from_slice(&0u32.to_le_bytes());
    dst[10..14].copy_from_slice(&pixel_offset.to_le_bytes());
    dst[14..18].copy_from_slice(&40u32.to_le_bytes());
    dst[18..22].copy_from_slice(&(width as i32).to_le_bytes());
    dst[22..26].copy_from_slice(&(-(height as i32)).to_le_bytes());
    dst[26..28].copy_from_slice(&1u16.to_le_bytes());
    dst[28..30].copy_from_slice(&bits_per_pixel.to_le_bytes());
    dst[30..34].copy_from_slice(&0u32.to_le_bytes());
    dst[34..38].copy_from_slice(&image_size.to_le_bytes());
    dst[38..42].copy_from_slice(&0x0B13u32.to_le_bytes()); // 2835 ppm (72 DPI)
    dst[42..46].copy_from_slice(&0x0B13u32.to_le_bytes()); // 2835 ppm (72 DPI)
    dst[46..50].copy_from_slice(&0u32.to_le_bytes());
    dst[50..54].copy_from_slice(&0u32.to_le_bytes());

    Ok(BMP_HEADER_LEN)
}

/// Converts a 24-bit RGB888 buffer into a 24-bit BMP image in `dst`.
///
/// Returns the total BMP size in bytes written to `dst`.
pub fn rgb888_to_bmp(
    src: &[u8],
    dst: &mut [u8],
    width: u16,
    height: u16,
) -> Result<usize, CameraError> {
    let layout = compute_bmp_layout(width, height, 3, 3, 0)?;
    if src.len() < layout.src_required {
        return Err(CameraError::InvalidArg);
    }
    if dst.len() < layout.total_size {
        return Err(CameraError::BufferTooSmall);
    }

    write_bmp_header(dst, width, height, 24, layout.image_size, 0)?;

    let mut out_off = BMP_HEADER_LEN;
    let mut in_off = 0usize;
    let w = width as usize;
    let h = height as usize;

    for _y in 0..h {
        for _x in 0..w {
            let r = src[in_off];
            let g = src[in_off + 1];
            let b = src[in_off + 2];
            in_off += 3;

            dst[out_off] = b;
            dst[out_off + 1] = g;
            dst[out_off + 2] = r;
            out_off += 3;
        }
        for _p in 0..layout.padding {
            dst[out_off] = 0;
            out_off += 1;
        }
    }

    Ok(layout.total_size)
}

/// Converts a 16-bit big-endian RGB565 buffer into a 24-bit BMP image in `dst`.
///
/// Matches `conversions/to_bmp.c:330-339` and `373-386`.
pub fn rgb565_to_bmp(
    src: &[u8],
    dst: &mut [u8],
    width: u16,
    height: u16,
) -> Result<usize, CameraError> {
    let layout = compute_bmp_layout(width, height, 2, 3, 0)?;
    if src.len() < layout.src_required {
        return Err(CameraError::InvalidArg);
    }
    if dst.len() < layout.total_size {
        return Err(CameraError::BufferTooSmall);
    }

    write_bmp_header(dst, width, height, 24, layout.image_size, 0)?;

    let mut out_off = BMP_HEADER_LEN;
    let mut in_off = 0usize;
    let w = width as usize;
    let h = height as usize;

    for _y in 0..h {
        for _x in 0..w {
            let hb = src[in_off];
            let lb = src[in_off + 1];
            in_off += 2;

            dst[out_off] = (lb & 0x1F) << 3;
            dst[out_off + 1] = ((hb & 0x07) << 5) | ((lb & 0xE0) >> 3);
            dst[out_off + 2] = hb & 0xF8;
            out_off += 3;
        }
        for _p in 0..layout.padding {
            dst[out_off] = 0;
            out_off += 1;
        }
    }

    Ok(layout.total_size)
}

/// Converts an 8-bit grayscale buffer into an 8-bit indexed BMP image with a 256-entry
/// palette (`1024` bytes) in `dst`.
///
/// Matches `conversions/to_bmp.c:277-321` and `387-395`.
pub fn grayscale_to_bmp(
    src: &[u8],
    dst: &mut [u8],
    width: u16,
    height: u16,
) -> Result<usize, CameraError> {
    let layout = compute_bmp_layout(width, height, 1, 1, BMP_GRAYSCALE_PALETTE_LEN as u32)?;
    if src.len() < layout.src_required {
        return Err(CameraError::InvalidArg);
    }
    if dst.len() < layout.total_size {
        return Err(CameraError::BufferTooSmall);
    }

    write_bmp_header(
        dst,
        width,
        height,
        8,
        layout.image_size,
        layout.palette_size,
    )?;

    let mut out_off = BMP_HEADER_LEN;
    for i in 0..256u16 {
        let v = i as u8;
        dst[out_off] = v;
        dst[out_off + 1] = v;
        dst[out_off + 2] = v;
        dst[out_off + 3] = 0;
        out_off += 4;
    }

    let mut in_off = 0usize;
    let h = height as usize;
    for _y in 0..h {
        dst[out_off..out_off + layout.row_bytes]
            .copy_from_slice(&src[in_off..in_off + layout.row_bytes]);
        out_off += layout.row_bytes;
        in_off += layout.row_bytes;
        for _p in 0..layout.padding {
            dst[out_off] = 0;
            out_off += 1;
        }
    }

    Ok(layout.total_size)
}

/// Converts a packed YUV422 (YUYV: `[Y0, U, Y1, V]`) buffer into a 24-bit BMP image in `dst`.
///
/// Matches `conversions/to_bmp.c:342-362` and `396-428`.
pub fn yuv422_to_bmp(
    src: &[u8],
    dst: &mut [u8],
    width: u16,
    height: u16,
) -> Result<usize, CameraError> {
    if (width & 1) != 0 {
        return Err(CameraError::InvalidArg);
    }
    let layout = compute_bmp_layout(width, height, 2, 3, 0)?;
    if src.len() < layout.src_required {
        return Err(CameraError::InvalidArg);
    }
    if dst.len() < layout.total_size {
        return Err(CameraError::BufferTooSmall);
    }

    write_bmp_header(dst, width, height, 24, layout.image_size, 0)?;

    let mut out_off = BMP_HEADER_LEN;
    let mut in_off = 0usize;
    let w = width as usize;
    let h = height as usize;

    for _y in 0..h {
        for _x in 0..(w / 2) {
            let y0 = src[in_off];
            let u = src[in_off + 1];
            let y1 = src[in_off + 2];
            let v = src[in_off + 3];
            in_off += 4;

            let (r0, g0, b0) = yuv2rgb(y0, u, v);
            dst[out_off] = b0;
            dst[out_off + 1] = g0;
            dst[out_off + 2] = r0;

            let (r1, g1, b1) = yuv2rgb(y1, u, v);
            dst[out_off + 3] = b1;
            dst[out_off + 4] = g1;
            dst[out_off + 5] = r1;
            out_off += 6;
        }
        for _p in 0..layout.padding {
            dst[out_off] = 0;
            out_off += 1;
        }
    }

    Ok(layout.total_size)
}

/// Converts a camera frame buffer of the given [`PixelFormat`] into a BMP image in `dst`.
///
/// Supports [`PixelFormat::Rgb888`], [`PixelFormat::Rgb565`], [`PixelFormat::Grayscale`],
/// [`PixelFormat::Yuv422`], and [`PixelFormat::Jpeg`].
pub fn frame_to_bmp(
    src: &[u8],
    dst: &mut [u8],
    width: u16,
    height: u16,
    format: PixelFormat,
) -> Result<usize, CameraError> {
    match format {
        PixelFormat::Rgb888 => rgb888_to_bmp(src, dst, width, height),
        PixelFormat::Rgb565 => rgb565_to_bmp(src, dst, width, height),
        PixelFormat::Grayscale => grayscale_to_bmp(src, dst, width, height),
        PixelFormat::Yuv422 => yuv422_to_bmp(src, dst, width, height),
        PixelFormat::Jpeg => {
            if let Some((jw, jh)) = crate::jpeg_scan::parse_jpeg_dimensions(src) {
                if (width != 0 && width != jw) || (height != 0 && height != jh) {
                    return Err(CameraError::InvalidArg);
                }
            }
            let (dec_w, dec_h, written) = crate::jpeg_codec::jpeg_to_bmp(src, dst)?;
            if (width != 0 && width != dec_w) || (height != 0 && height != dec_h) {
                return Err(CameraError::InvalidArg);
            }
            Ok(written)
        }
        _ => Err(CameraError::InvalidFormat),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rgb888_to_bmp_with_padding() {
        // 3x2 image: row_bytes = 9, row_stride = 12 (3 bytes padding per row), image_size = 24
        // CONV-002: Provide distinct [R, G, B] pixels to verify little-endian [B, G, R] output
        let mut src = [0u8; 18];
        for i in 0..6 {
            src[i * 3] = 0x11;     // R
            src[i * 3 + 1] = 0x22; // G
            src[i * 3 + 2] = 0x33; // B
        }
        let mut dst = [0u8; BMP_HEADER_LEN + 24];
        let written = rgb888_to_bmp(&src, &mut dst, 3, 2).expect("rgb888_to_bmp should succeed");
        assert_eq!(written, BMP_HEADER_LEN + 24);
        assert_eq!(&dst[0..2], b"BM");
        assert_eq!(u32::from_le_bytes([dst[2], dst[3], dst[4], dst[5]]), 78);
        assert_eq!(u32::from_le_bytes([dst[10], dst[11], dst[12], dst[13]]), 54);
        assert_eq!(i32::from_le_bytes([dst[18], dst[19], dst[20], dst[21]]), 3);
        assert_eq!(i32::from_le_bytes([dst[22], dst[23], dst[24], dst[25]]), -2);
        assert_eq!(u16::from_le_bytes([dst[28], dst[29]]), 24);
        // Verify CONV-002: little-endian BMP pixel byte order is [B, G, R] = [0x33, 0x22, 0x11]
        assert_eq!(&dst[54..57], &[0x33, 0x22, 0x11]);
        assert_eq!(&dst[57..60], &[0x33, 0x22, 0x11]);
        assert_eq!(&dst[60..63], &[0x33, 0x22, 0x11]);
        // Check row 0 padding bytes are 0
        assert_eq!(&dst[63..66], &[0, 0, 0]);
    }

    #[test]
    fn test_rgb565_and_yuv422_to_bmp() {
        // 2x2 image: row_bytes = 6, row_stride = 8 (2 bytes padding), image_size = 16
        let rgb565 = [0xF8, 0x00, 0x07, 0xE0, 0x00, 0x1F, 0xFF, 0xFF];
        let mut dst = [0u8; BMP_HEADER_LEN + 16];
        let n = rgb565_to_bmp(&rgb565, &mut dst, 2, 2).expect("rgb565_to_bmp should succeed");
        assert_eq!(n, BMP_HEADER_LEN + 16);
        // First pixel (0xF800 -> pure red) in BMP is BGR = [0, 0, 0xF8]
        assert_eq!(&dst[54..57], &[0x00, 0x00, 0xF8]);

        let yuv422 = [128u8; 8];
        let n_yuv = yuv422_to_bmp(&yuv422, &mut dst, 2, 2).expect("yuv422_to_bmp should succeed");
        assert_eq!(n_yuv, BMP_HEADER_LEN + 16);
        assert_eq!(&dst[54..60], &[130u8; 6]);
    }

    #[test]
    fn test_grayscale_to_bmp_palette() {
        // 3x2 grayscale: row_bytes = 3, row_stride = 4, image_size = 8, palette = 1024
        let gray = [10, 20, 30, 40, 50, 60];
        let mut dst = [0u8; BMP_HEADER_LEN + BMP_GRAYSCALE_PALETTE_LEN + 8];
        let n = grayscale_to_bmp(&gray, &mut dst, 3, 2).expect("grayscale_to_bmp should succeed");
        assert_eq!(n, 54 + 1024 + 8);
        assert_eq!(
            u32::from_le_bytes([dst[10], dst[11], dst[12], dst[13]]),
            54 + 1024
        );
        assert_eq!(u16::from_le_bytes([dst[28], dst[29]]), 8);

        // Verify 256-entry palette
        for i in 0..256usize {
            let p = 54 + i * 4;
            assert_eq!(dst[p], i as u8);
            assert_eq!(dst[p + 1], i as u8);
            assert_eq!(dst[p + 2], i as u8);
            assert_eq!(dst[p + 3], 0);
        }

        let pix_off = 54 + 1024;
        assert_eq!(&dst[pix_off..pix_off + 4], &[10, 20, 30, 0]);
        assert_eq!(&dst[pix_off + 4..pix_off + 8], &[40, 50, 60, 0]);
    }

    #[test]
    fn test_bmp_integer_overflow_rejection() {
        let mut dst = [0u8; 128];
        assert_eq!(
            rgb888_to_bmp(&[], &mut dst, u16::MAX, u16::MAX),
            Err(CameraError::Overflow)
        );
        assert_eq!(
            rgb565_to_bmp(&[], &mut dst, u16::MAX, u16::MAX),
            Err(CameraError::Overflow)
        );
        assert_eq!(
            grayscale_to_bmp(&[], &mut dst, u16::MAX, u16::MAX),
            Err(CameraError::Overflow)
        );
        assert_eq!(
            yuv422_to_bmp(&[], &mut dst, u16::MAX - 1, u16::MAX),
            Err(CameraError::Overflow)
        );
        assert_eq!(
            frame_to_bmp(&[], &mut dst, u16::MAX, u16::MAX, PixelFormat::Rgb888),
            Err(CameraError::Overflow)
        );
    }

    #[test]
    fn test_yuv422_to_bmp_odd_width_rejection() {
        // CONV-005: Reject odd width in yuv422_to_bmp
        let mut dst = [0u8; 128];
        assert_eq!(
            yuv422_to_bmp(&[128u8; 12], &mut dst, 3, 2),
            Err(CameraError::InvalidArg)
        );
    }

    #[test]
    fn test_frame_to_bmp_jpeg_dimension_mismatch() {
        // CONV-006: In frame_to_bmp(..., PixelFormat::Jpeg), verify caller width/height
        let rgb_in = [100u8; 8 * 8 * 3];
        let mut jpeg_buf = [0u8; 2048];
        let jpeg_len = crate::jpeg_codec::encode_rgb888_to_jpeg(&rgb_in, &mut jpeg_buf, 8, 8, 90)
            .expect("encode should succeed");
        let jpeg_slice = &jpeg_buf[..jpeg_len];

        let mut bmp_buf = [0u8; BMP_HEADER_LEN + 16 * 16 * 3];
        // Mismatched width (16 instead of 8)
        assert_eq!(
            frame_to_bmp(jpeg_slice, &mut bmp_buf, 16, 8, PixelFormat::Jpeg),
            Err(CameraError::InvalidArg)
        );
        // Mismatched height (16 instead of 8)
        assert_eq!(
            frame_to_bmp(jpeg_slice, &mut bmp_buf, 8, 16, PixelFormat::Jpeg),
            Err(CameraError::InvalidArg)
        );
        // Matching dimensions succeed
        assert!(frame_to_bmp(jpeg_slice, &mut bmp_buf, 8, 8, PixelFormat::Jpeg).is_ok());
        // Unspecified dimensions (0, 0) succeed
        assert!(frame_to_bmp(jpeg_slice, &mut bmp_buf, 0, 0, PixelFormat::Jpeg).is_ok());
    }
}
