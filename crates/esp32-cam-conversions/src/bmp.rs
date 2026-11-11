//! Safe `#![no_std]` little-endian BMP serializer with checked arithmetic
//! overflow guards.
//!
//! Replaces `conversions/to_bmp.c` (`CONV-101`..`CONV-105`), eliminating
//! unaligned packed struct pointer casts and integer overflow vulnerabilities.

use crate::jpeg_codec;
use crate::jpeg_scan;
use crate::yuv;
use esp32_cam_types::{CameraError, PixelFormat};

/// Standard Windows BITMAPFILEHEADER (14 bytes) + BITMAPINFOHEADER (40 bytes)
/// length.
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

/// Computes and validates BMP buffer layout using checked 32-bit arithmetic
/// (`CONV-101`..`CONV-105`).
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
    let row_stride32 = row_bytes32.checked_add(3).ok_or(CameraError::Overflow)? & !3u32;
    let image_size32 = row_stride32.checked_mul(h32).ok_or(CameraError::Overflow)?;
    let total_size32 = (BMP_HEADER_LEN as u32)
        .checked_add(palette_size)
        .and_then(|v| v.checked_add(image_size32))
        .ok_or(CameraError::Overflow)?;

    // Reject sizes exceeding signed 32-bit max to prevent signed offset
    // overflow in BMP readers.
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

/// Serializes the 54-byte Windows BMP header (`BM` magic + 12-byte file
/// header + 40-byte DIB header) in little-endian byte order into `dst[..54]`.
///
/// The height field is written as `-height` to specify top-to-bottom
/// scanline order, matching `conversions/to_bmp.c`.
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
    let clr_used: u32 = palette_size / 4;
    dst[46..50].copy_from_slice(&clr_used.to_le_bytes());
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

    let row_stride = layout.row_bytes + layout.padding;
    let src_rows = src[..layout.src_required].chunks_exact(layout.row_bytes);
    let dst_rows = dst[BMP_HEADER_LEN..layout.total_size].chunks_exact_mut(row_stride);

    for (src_row, dst_row) in src_rows.zip(dst_rows) {
        let (in_pixels, _) = src_row.as_chunks::<3>();
        let (out_pixels, _) = dst_row[..layout.row_bytes].as_chunks_mut::<3>();
        for (in_pix, out_pix) in in_pixels.iter().zip(out_pixels) {
            out_pix[0] = in_pix[2];
            out_pix[1] = in_pix[1];
            out_pix[2] = in_pix[0];
        }
        dst_row[layout.row_bytes..].fill(0);
    }

    Ok(layout.total_size)
}

/// Converts a 16-bit big-endian RGB565 buffer into a 24-bit BMP image in `dst`.
///
/// Unpacks each RGB565 pixel with full bit replication via
/// [`yuv::unpack_rgb565_be_full`] (`CONV-15`) so `0xFFFF` maps to
/// `(255, 255, 255)`.
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

    let src_row_bytes = (width as usize) * 2;
    let row_stride = layout.row_bytes + layout.padding;
    let src_rows = src[..layout.src_required].chunks_exact(src_row_bytes);
    let dst_rows = dst[BMP_HEADER_LEN..layout.total_size].chunks_exact_mut(row_stride);

    for (src_row, dst_row) in src_rows.zip(dst_rows) {
        let (in_pixels, _) = src_row.as_chunks::<2>();
        let (out_pixels, _) = dst_row[..layout.row_bytes].as_chunks_mut::<3>();
        for (in_pix, out_pix) in in_pixels.iter().zip(out_pixels) {
            let (r, g, b) = yuv::unpack_rgb565_be_full(in_pix[0], in_pix[1]);
            *out_pix = [b, g, r];
        }
        dst_row[layout.row_bytes..].fill(0);
    }

    Ok(layout.total_size)
}

/// Converts an 8-bit grayscale buffer into an 8-bit indexed BMP image with a
/// 256-entry palette (`1024` bytes) in `dst`.
///
/// Matches `conversions/to_bmp.c:277-321` and `387-395`, emitting scanlines
/// and 4-byte row alignment padding via `chunks_exact_mut` + `.fill(0)`
/// (`CONV-18`).
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

    let palette_end = BMP_HEADER_LEN + BMP_GRAYSCALE_PALETTE_LEN;
    let (palette_entries, _) = dst[BMP_HEADER_LEN..palette_end].as_chunks_mut::<4>();
    for (i, entry) in palette_entries.iter_mut().enumerate() {
        let v = i as u8;
        *entry = [v, v, v, 0];
    }

    let row_stride = layout.row_bytes + layout.padding;
    let src_rows = src[..layout.src_required].chunks_exact(layout.row_bytes);
    let dst_rows = dst[palette_end..layout.total_size].chunks_exact_mut(row_stride);

    for (src_row, dst_row) in src_rows.zip(dst_rows) {
        dst_row[..layout.row_bytes].copy_from_slice(src_row);
        dst_row[layout.row_bytes..].fill(0);
    }

    Ok(layout.total_size)
}

/// Converts a packed YUV422 (YUYV: `[Y0, U, Y1, V]`) buffer into a 24-bit
/// BMP image in `dst`.
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

    let src_row_bytes = (width as usize) * 2;
    let row_stride = layout.row_bytes + layout.padding;
    let src_rows = src[..layout.src_required].chunks_exact(src_row_bytes);
    let dst_rows = dst[BMP_HEADER_LEN..layout.total_size].chunks_exact_mut(row_stride);

    for (src_row, dst_row) in src_rows.zip(dst_rows) {
        let (in_pairs, _) = src_row.as_chunks::<4>();
        let (out_pairs, _) = dst_row[..layout.row_bytes].as_chunks_mut::<6>();
        for (chunk, out) in in_pairs.iter().zip(out_pairs) {
            let ((r0, g0, b0), (r1, g1, b1)) =
                yuv::yuv422_pair_to_rgb(chunk[0], chunk[1], chunk[2], chunk[3]);
            out[0] = b0;
            out[1] = g0;
            out[2] = r0;
            out[3] = b1;
            out[4] = g1;
            out[5] = r1;
        }
        dst_row[layout.row_bytes..].fill(0);
    }

    Ok(layout.total_size)
}

/// Converts a camera frame buffer of the given [`PixelFormat`] into a BMP
/// image in `dst`.
///
/// Supports [`PixelFormat::Rgb888`], [`PixelFormat::Rgb565`],
/// [`PixelFormat::Grayscale`], [`PixelFormat::Yuv422`], and
/// [`PixelFormat::Jpeg`].
///
/// For [`PixelFormat::Jpeg`], skips the pre-decode SOF header scan entirely
/// when `width == 0 && height == 0`, and otherwise validates expected SOF
/// dimensions once up front before `dst` is touched (`CONV-18`).
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
            if width != 0 || height != 0 {
                let (jw, jh) =
                    jpeg_scan::parse_jpeg_dimensions(src).ok_or(CameraError::CorruptedFrame)?;
                if (width != 0 && width != jw) || (height != 0 && height != jh) {
                    return Err(CameraError::InvalidArg);
                }
            }
            let (dec_w, dec_h, written) = jpeg_codec::jpeg_to_bmp(src, dst)?;
            debug_assert!(width == 0 || width == dec_w);
            debug_assert!(height == 0 || height == dec_h);
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
        // 3x2 image: row_bytes = 9, row_stride = 12 (3 bytes padding per row),
        // image_size = 24.
        // CONV-002: Provide distinct [R, G, B] pixels to verify little-endian
        // [B, G, R] output.
        let mut src = [0u8; 18];
        for i in 0..6 {
            src[i * 3] = 0x11; // R
            src[i * 3 + 1] = 0x22; // G
            src[i * 3 + 2] = 0x33; // B
        }
        let mut dst = [0xFFu8; BMP_HEADER_LEN + 24];
        let written = rgb888_to_bmp(&src, &mut dst, 3, 2).expect("rgb888_to_bmp should succeed");
        assert_eq!(written, BMP_HEADER_LEN + 24);
        assert_eq!(&dst[0..2], b"BM");
        assert_eq!(u32::from_le_bytes([dst[2], dst[3], dst[4], dst[5]]), 78);
        assert_eq!(u32::from_le_bytes([dst[10], dst[11], dst[12], dst[13]]), 54);
        assert_eq!(i32::from_le_bytes([dst[18], dst[19], dst[20], dst[21]]), 3);
        assert_eq!(i32::from_le_bytes([dst[22], dst[23], dst[24], dst[25]]), -2);
        assert_eq!(u16::from_le_bytes([dst[28], dst[29]]), 24);
        // SEC-061: clr_used is 0 when palette_size is 0.
        assert_eq!(u32::from_le_bytes([dst[46], dst[47], dst[48], dst[49]]), 0);
        // Verify CONV-002: little-endian BMP pixel byte order is [B, G, R].
        assert_eq!(&dst[54..57], &[0x33, 0x22, 0x11]);
        assert_eq!(&dst[57..60], &[0x33, 0x22, 0x11]);
        assert_eq!(&dst[60..63], &[0x33, 0x22, 0x11]);
        // Check both rows' padding bytes were zeroed over 0xFF sentinel.
        assert_eq!(&dst[63..66], &[0, 0, 0]);
        assert_eq!(&dst[75..78], &[0, 0, 0]);
    }

    #[test]
    fn test_rgb565_and_yuv422_to_bmp() {
        // 2x2 image: row_bytes = 6, row_stride = 8 (2 bytes padding),
        // image_size = 16.
        let rgb565 = [0xF8, 0x00, 0x07, 0xE0, 0x00, 0x1F, 0xFF, 0xFF];
        let mut dst = [0xFFu8; BMP_HEADER_LEN + 16];
        let n = rgb565_to_bmp(&rgb565, &mut dst, 2, 2).expect("rgb565_to_bmp should succeed");
        assert_eq!(n, BMP_HEADER_LEN + 16);
        // Row 0: 0xF800 -> [0, 0, 255], 0x07E0 -> [0, 255, 0] (BGR).
        assert_eq!(&dst[54..57], &[0x00, 0x00, 0xFF]);
        assert_eq!(&dst[57..60], &[0x00, 0xFF, 0x00]);
        assert_eq!(&dst[60..62], &[0, 0]);
        // Row 1: 0x001F -> [255, 0, 0], 0xFFFF -> [255, 255, 255] (CONV-15).
        assert_eq!(&dst[62..65], &[0xFF, 0x00, 0x00]);
        assert_eq!(&dst[65..68], &[0xFF, 0xFF, 0xFF]);
        assert_eq!(&dst[68..70], &[0, 0]);

        let yuv422 = [128u8; 8];
        dst.fill(0xFF);
        let n_yuv = yuv422_to_bmp(&yuv422, &mut dst, 2, 2).expect("yuv422_to_bmp should succeed");
        assert_eq!(n_yuv, BMP_HEADER_LEN + 16);
        assert_eq!(&dst[54..60], &[130u8; 6]);
        assert_eq!(&dst[60..62], &[0, 0]);
        assert_eq!(&dst[68..70], &[0, 0]);
    }

    #[test]
    fn test_grayscale_to_bmp_palette() {
        // 3x2 grayscale: row_bytes = 3, row_stride = 4, image_size = 8,
        // palette = 1024.
        let gray = [10, 20, 30, 40, 50, 60];
        let mut dst = [0xFFu8; BMP_HEADER_LEN + BMP_GRAYSCALE_PALETTE_LEN + 8];
        let n = grayscale_to_bmp(&gray, &mut dst, 3, 2).expect("grayscale_to_bmp should succeed");
        assert_eq!(n, 54 + 1024 + 8);
        assert_eq!(
            u32::from_le_bytes([dst[10], dst[11], dst[12], dst[13]]),
            54 + 1024
        );
        assert_eq!(u16::from_le_bytes([dst[28], dst[29]]), 8);
        // SEC-061: clr_used is 256 (1024 / 4) for grayscale palette.
        assert_eq!(
            u32::from_le_bytes([dst[46], dst[47], dst[48], dst[49]]),
            256
        );

        // Verify 256-entry palette.
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
    fn test_odd_width_bmp_row_padding_all_alignments() {
        // Verify grayscale, RGB888, and RGB565 row padding across odd and
        // non-multiple-of-4 widths when dst starts filled with 0xEE sentinel.
        for width in 1..=5u16 {
            let height = 3u16;
            let w = width as usize;
            let h = height as usize;

            // 8-bit grayscale: row_bytes = w, stride = (w + 3) & !3.
            let gray_stride = (w + 3) & !3;
            let mut gray_src = [0u8; 15];
            for (idx, byte) in gray_src[..w * h].iter_mut().enumerate() {
                *byte = (idx as u8) + 1;
            }
            let mut gray_dst = [0xEEu8; BMP_HEADER_LEN + BMP_GRAYSCALE_PALETTE_LEN + 64];
            let expected_gray_len = BMP_HEADER_LEN + BMP_GRAYSCALE_PALETTE_LEN + gray_stride * h;
            let written_gray = grayscale_to_bmp(&gray_src[..w * h], &mut gray_dst, width, height)
                .expect("grayscale odd-width BMP should succeed");
            assert_eq!(written_gray, expected_gray_len);
            let pix_base = BMP_HEADER_LEN + BMP_GRAYSCALE_PALETTE_LEN;
            for row in 0..h {
                let row_slice = &gray_dst[pix_base + row * gray_stride..][..gray_stride];
                assert_eq!(&row_slice[..w], &gray_src[row * w..(row + 1) * w]);
                for &pad in &row_slice[w..] {
                    assert_eq!(pad, 0, "grayscale pad not zeroed for width {width}");
                }
            }
            assert!(gray_dst[expected_gray_len..].iter().all(|&b| b == 0xEE));

            // 24-bit RGB888: row_bytes = w * 3, stride = (w * 3 + 3) & !3.
            let rgb_row_bytes = w * 3;
            let rgb_stride = (rgb_row_bytes + 3) & !3;
            let rgb_src = [0x44u8; 45];
            let mut rgb_dst = [0xEEu8; BMP_HEADER_LEN + 128];
            let expected_rgb_len = BMP_HEADER_LEN + rgb_stride * h;
            let written_rgb =
                rgb888_to_bmp(&rgb_src[..rgb_row_bytes * h], &mut rgb_dst, width, height)
                    .expect("rgb888 odd-width BMP should succeed");
            assert_eq!(written_rgb, expected_rgb_len);
            for row in 0..h {
                let row_slice = &rgb_dst[BMP_HEADER_LEN + row * rgb_stride..][..rgb_stride];
                assert!(row_slice[..rgb_row_bytes].iter().all(|&b| b == 0x44));
                assert!(row_slice[rgb_row_bytes..].iter().all(|&b| b == 0));
            }
            assert!(rgb_dst[expected_rgb_len..].iter().all(|&b| b == 0xEE));

            // 16-bit RGB565 -> 24-bit BMP: same output stride as RGB888.
            let rgb565_src = [0x00u8; 30];
            let mut rgb565_dst = [0xEEu8; BMP_HEADER_LEN + 128];
            let written_565 =
                rgb565_to_bmp(&rgb565_src[..w * 2 * h], &mut rgb565_dst, width, height)
                    .expect("rgb565 odd-width BMP should succeed");
            assert_eq!(written_565, expected_rgb_len);
            for row in 0..h {
                let row_slice = &rgb565_dst[BMP_HEADER_LEN + row * rgb_stride..][..rgb_stride];
                assert!(row_slice.iter().all(|&b| b == 0));
            }
            assert!(rgb565_dst[expected_rgb_len..].iter().all(|&b| b == 0xEE));
        }
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
        // CONV-005: Reject odd width in yuv422_to_bmp.
        let mut dst = [0u8; 128];
        assert_eq!(
            yuv422_to_bmp(&[128u8; 12], &mut dst, 3, 2),
            Err(CameraError::InvalidArg)
        );
    }

    #[test]
    fn test_frame_to_bmp_jpeg_dimension_mismatch_leaves_dst_untouched() {
        // CONV-18: Verify single-pass SOF check rejects mismatches before
        // touching dst, and odd-width JPEG -> BMP zero-pads scanlines.
        let rgb_in = [100u8; 5 * 3 * 3];
        let mut jpeg_buf = [0u8; 2048];
        let jpeg_len = jpeg_codec::encode_rgb888_to_jpeg(&rgb_in, &mut jpeg_buf, 5, 3, 90)
            .expect("encode should succeed");
        let jpeg_slice = &jpeg_buf[..jpeg_len];

        let mut bmp_buf = [0xA5u8; BMP_HEADER_LEN + 16 * 16 * 3];
        // Mismatched width (16 instead of 5) leaves dst completely untouched.
        assert_eq!(
            frame_to_bmp(jpeg_slice, &mut bmp_buf, 16, 3, PixelFormat::Jpeg),
            Err(CameraError::InvalidArg)
        );
        assert!(bmp_buf.iter().all(|&b| b == 0xA5));

        // Mismatched height (16 instead of 3) leaves dst completely untouched.
        assert_eq!(
            frame_to_bmp(jpeg_slice, &mut bmp_buf, 5, 16, PixelFormat::Jpeg),
            Err(CameraError::InvalidArg)
        );
        assert!(bmp_buf.iter().all(|&b| b == 0xA5));

        // Corrupt JPEG with expected dimensions leaves dst untouched.
        assert_eq!(
            frame_to_bmp(
                &[0xFF, 0xD8, 0xFF, 0xD9],
                &mut bmp_buf,
                5,
                3,
                PixelFormat::Jpeg
            ),
            Err(CameraError::CorruptedFrame)
        );
        assert!(bmp_buf.iter().all(|&b| b == 0xA5));

        // Matching odd dimensions (5x3 -> row_bytes = 15, stride = 16) succeed
        // and zero-fill the 1-byte padding on each row.
        let written = frame_to_bmp(jpeg_slice, &mut bmp_buf, 5, 3, PixelFormat::Jpeg)
            .expect("matching odd-width JPEG should decode to BMP");
        assert_eq!(written, BMP_HEADER_LEN + 16 * 3);
        for row in 0..3usize {
            assert_eq!(bmp_buf[BMP_HEADER_LEN + row * 16 + 15], 0);
        }

        // Unspecified dimensions (0, 0) take the direct single-pass decode path.
        bmp_buf.fill(0xA5);
        let written_auto = frame_to_bmp(jpeg_slice, &mut bmp_buf, 0, 0, PixelFormat::Jpeg)
            .expect("auto-dimension JPEG should decode to BMP");
        assert_eq!(written_auto, BMP_HEADER_LEN + 16 * 3);
        for row in 0..3usize {
            assert_eq!(bmp_buf[BMP_HEADER_LEN + row * 16 + 15], 0);
        }

        // Non-JPEG formats via frame_to_bmp also succeed.
        let gray_src = [77u8; 5 * 3];
        let mut gray_bmp = [0xA5u8; BMP_HEADER_LEN + BMP_GRAYSCALE_PALETTE_LEN + 8 * 3];
        let gray_written = frame_to_bmp(&gray_src, &mut gray_bmp, 5, 3, PixelFormat::Grayscale)
            .expect("frame_to_bmp grayscale should succeed");
        assert_eq!(
            gray_written,
            BMP_HEADER_LEN + BMP_GRAYSCALE_PALETTE_LEN + 8 * 3
        );
    }
}
