//! Safe `#![no_std]` ITU-R BT.601 YUV/RGB color-space conversions.
//!
//! Replaces `conversions/yuv.c` and software YUV422/RGB565/RGB888/YUV420
//! conversion paths across `conversions/to_bmp.c` and `conversions/to_jpg.cpp`.

use esp32_cam_types::CameraError;

/// Luma and chroma excursion range for ITU-R BT.601 YUV conversions.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum YuvRange {
    /// Full-range JFIF YCbCr (`Y` in `0..=255`, `U`/`V` in `0..=255`).
    Full,
    /// Studio-range ITU-R BT.601 (`Y` in `16..=235`, `U`/`V` in `16..=240`).
    #[default]
    Bt601Limited,
}

// Per-channel studio-range ITU-R BT.601 lookup tables
// (`conversions/yuv.c:25-283`).
static YUV_V_Y: [i16; 256] = [
    -18, -17, -16, -15, -13, -12, -11, -10, -9, -8, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 8,
    9, 10, 11, 12, 13, 15, 16, 17, 18, 19, 20, 22, 23, 24, 25, 26, 27, 29, 30, 31, 32, 33, 34, 36,
    37, 38, 39, 40, 41, 43, 44, 45, 46, 47, 48, 50, 51, 52, 53, 54, 55, 57, 58, 59, 60, 61, 62, 64,
    65, 66, 67, 68, 69, 71, 72, 73, 74, 75, 76, 77, 79, 80, 81, 82, 83, 84, 86, 87, 88, 89, 90, 91,
    93, 94, 95, 96, 97, 98, 100, 101, 102, 103, 104, 105, 107, 108, 109, 110, 111, 112, 114, 115,
    116, 117, 118, 119, 121, 122, 123, 124, 125, 126, 128, 129, 130, 131, 132, 133, 135, 136, 137,
    138, 139, 140, 142, 143, 144, 145, 146, 147, 148, 150, 151, 152, 153, 154, 155, 157, 158, 159,
    160, 161, 162, 164, 165, 166, 167, 168, 169, 171, 172, 173, 174, 175, 176, 178, 179, 180, 181,
    182, 183, 185, 186, 187, 188, 189, 190, 192, 193, 194, 195, 196, 197, 199, 200, 201, 202, 203,
    204, 206, 207, 208, 209, 210, 211, 213, 214, 215, 216, 217, 218, 219, 221, 222, 223, 224, 225,
    226, 228, 229, 230, 231, 232, 233, 235, 236, 237, 238, 239, 240, 242, 243, 244, 245, 246, 247,
    249, 250, 251, 252, 253, 254, 256, 257, 258, 259, 260, 261, 263, 264, 265, 266, 267, 268, 270,
    271, 272, 273, 274, 275, 277, 278,
];

static YUV_V_VR: [i16; 256] = [
    -204, -202, -201, -199, -197, -196, -194, -193, -191, -189, -188, -186, -185, -183, -181, -180,
    -178, -177, -175, -173, -172, -170, -169, -167, -165, -164, -162, -161, -159, -158, -156, -154,
    -153, -151, -150, -148, -146, -145, -143, -142, -140, -138, -137, -135, -134, -132, -130, -129,
    -127, -126, -124, -122, -121, -119, -118, -116, -114, -113, -111, -110, -108, -106, -105, -103,
    -102, -100, -98, -97, -95, -94, -92, -90, -89, -87, -86, -84, -82, -81, -79, -78, -76, -75,
    -73, -71, -70, -68, -67, -65, -63, -62, -60, -59, -57, -55, -54, -52, -51, -49, -47, -46, -44,
    -43, -41, -39, -38, -36, -35, -33, -31, -30, -28, -27, -25, -23, -22, -20, -19, -17, -15, -14,
    -12, -11, -9, -7, -6, -4, -3, -1, 0, 1, 3, 4, 6, 7, 9, 11, 12, 14, 15, 17, 19, 20, 22, 23, 25,
    27, 28, 30, 31, 33, 35, 36, 38, 39, 41, 43, 44, 46, 47, 49, 51, 52, 54, 55, 57, 59, 60, 62, 63,
    65, 67, 68, 70, 71, 73, 75, 76, 78, 79, 81, 82, 84, 86, 87, 89, 90, 92, 94, 95, 97, 98, 100,
    102, 103, 105, 106, 108, 110, 111, 113, 114, 116, 118, 119, 121, 122, 124, 126, 127, 129, 130,
    132, 134, 135, 137, 138, 140, 142, 143, 145, 146, 148, 150, 151, 153, 154, 156, 158, 159, 161,
    162, 164, 165, 167, 169, 170, 172, 173, 175, 177, 178, 180, 181, 183, 185, 186, 188, 189, 191,
    193, 194, 196, 197, 199, 201, 202,
];

static YUV_V_VG: [i16; 256] = [
    50, 49, 49, 48, 48, 48, 47, 47, 46, 46, 46, 45, 45, 44, 44, 44, 43, 43, 43, 42, 42, 41, 41, 41,
    40, 40, 39, 39, 39, 38, 38, 37, 37, 37, 36, 36, 35, 35, 35, 34, 34, 34, 33, 33, 32, 32, 32, 31,
    31, 30, 30, 30, 29, 29, 28, 28, 28, 27, 27, 26, 26, 26, 25, 25, 25, 24, 24, 23, 23, 23, 22, 22,
    21, 21, 21, 20, 20, 19, 19, 19, 18, 18, 17, 17, 17, 16, 16, 16, 15, 15, 14, 14, 14, 13, 13, 12,
    12, 12, 11, 11, 10, 10, 10, 9, 9, 8, 8, 8, 7, 7, 7, 6, 6, 5, 5, 5, 4, 4, 3, 3, 3, 2, 2, 1, 1,
    1, 0, 0, 0, 0, 0, -1, -1, -1, -2, -2, -3, -3, -3, -4, -4, -5, -5, -5, -6, -6, -7, -7, -7, -8,
    -8, -8, -9, -9, -10, -10, -10, -11, -11, -12, -12, -12, -13, -13, -14, -14, -14, -15, -15, -16,
    -16, -16, -17, -17, -17, -18, -18, -19, -19, -19, -20, -20, -21, -21, -21, -22, -22, -23, -23,
    -23, -24, -24, -25, -25, -25, -26, -26, -26, -27, -27, -28, -28, -28, -29, -29, -30, -30, -30,
    -31, -31, -32, -32, -32, -33, -33, -34, -34, -34, -35, -35, -35, -36, -36, -37, -37, -37, -38,
    -38, -39, -39, -39, -40, -40, -41, -41, -41, -42, -42, -43, -43, -43, -44, -44, -44, -45, -45,
    -46, -46, -46, -47, -47, -48, -48, -48, -49, -49,
];

static YUV_V_UG: [i16; 256] = [
    104, 103, 102, 101, 100, 99, 99, 98, 97, 96, 95, 95, 94, 93, 92, 91, 91, 90, 89, 88, 87, 86,
    86, 85, 84, 83, 82, 82, 81, 80, 79, 78, 78, 77, 76, 75, 74, 73, 73, 72, 71, 70, 69, 69, 68, 67,
    66, 65, 65, 64, 63, 62, 61, 60, 60, 59, 58, 57, 56, 56, 55, 54, 53, 52, 52, 51, 50, 49, 48, 47,
    47, 46, 45, 44, 43, 43, 42, 41, 40, 39, 39, 38, 37, 36, 35, 34, 34, 33, 32, 31, 30, 30, 29, 28,
    27, 26, 26, 25, 24, 23, 22, 21, 21, 20, 19, 18, 17, 17, 16, 15, 14, 13, 13, 12, 11, 10, 9, 8,
    8, 7, 6, 5, 4, 4, 3, 2, 1, 0, 0, 0, -1, -2, -3, -4, -4, -5, -6, -7, -8, -8, -9, -10, -11, -12,
    -13, -13, -14, -15, -16, -17, -17, -18, -19, -20, -21, -21, -22, -23, -24, -25, -26, -26, -27,
    -28, -29, -30, -30, -31, -32, -33, -34, -34, -35, -36, -37, -38, -39, -39, -40, -41, -42, -43,
    -43, -44, -45, -46, -47, -47, -48, -49, -50, -51, -52, -52, -53, -54, -55, -56, -56, -57, -58,
    -59, -60, -60, -61, -62, -63, -64, -65, -65, -66, -67, -68, -69, -69, -70, -71, -72, -73, -73,
    -74, -75, -76, -77, -78, -78, -79, -80, -81, -82, -82, -83, -84, -85, -86, -86, -87, -88, -89,
    -90, -91, -91, -92, -93, -94, -95, -95, -96, -97, -98, -99, -99, -100, -101, -102, -103,
];

static YUV_V_UB: [i16; 256] = [
    -258, -256, -254, -252, -250, -248, -246, -244, -242, -240, -238, -236, -234, -232, -230, -228,
    -226, -223, -221, -219, -217, -215, -213, -211, -209, -207, -205, -203, -201, -199, -197, -195,
    -193, -191, -189, -187, -185, -183, -181, -179, -177, -175, -173, -171, -169, -167, -165, -163,
    -161, -159, -157, -155, -153, -151, -149, -147, -145, -143, -141, -139, -137, -135, -133, -131,
    -129, -127, -125, -123, -121, -119, -117, -115, -113, -110, -108, -106, -104, -102, -100, -98,
    -96, -94, -92, -90, -88, -86, -84, -82, -80, -78, -76, -74, -72, -70, -68, -66, -64, -62, -60,
    -58, -56, -54, -52, -50, -48, -46, -44, -42, -40, -38, -36, -34, -32, -30, -28, -26, -24, -22,
    -20, -18, -16, -14, -12, -10, -8, -6, -4, -2, 0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24,
    26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64, 66, 68, 70, 72,
    74, 76, 78, 80, 82, 84, 86, 88, 90, 92, 94, 96, 98, 100, 102, 104, 106, 108, 110, 113, 115,
    117, 119, 121, 123, 125, 127, 129, 131, 133, 135, 137, 139, 141, 143, 145, 147, 149, 151, 153,
    155, 157, 159, 161, 163, 165, 167, 169, 171, 173, 175, 177, 179, 181, 183, 185, 187, 189, 191,
    193, 195, 197, 199, 201, 203, 205, 207, 209, 211, 213, 215, 217, 219, 221, 223, 226, 228, 230,
    232, 234, 236, 238, 240, 242, 244, 246, 248, 250, 252, 254, 256,
];

const _: () = assert!(core::mem::size_of_val(&YUV_V_Y) == 512);

/// Converts a single YUV pixel `(y, u, v)` to RGB `(r, g, b)` using the exact
/// studio-range (`YuvRange::Bt601Limited`) table from `conversions/yuv.c`.
#[must_use]
#[inline]
pub fn yuv2rgb(y: u8, u: u8, v: u8) -> (u8, u8, u8) {
    yuv2rgb_with_range(y, u, v, YuvRange::Bt601Limited)
}

/// Converts a single studio-range ITU-R BT.601 YUV pixel `(y, u, v)` to RGB
/// `(r, g, b)` using table-free fixed-point integer multiplication.
#[must_use]
#[inline]
pub const fn yuv2rgb_bt601_mul(y: u8, u: u8, v: u8) -> (u8, u8, u8) {
    let c = y as i32 - 16;
    let d = u as i32 - 128;
    let e = v as i32 - 128;
    (
        clamp_u8((298 * c + 409 * e + 128) >> 8),
        clamp_u8((298 * c - 100 * d - 208 * e + 128) >> 8),
        clamp_u8((298 * c + 516 * d + 128) >> 8),
    )
}

/// Converts a single YUV pixel `(y, u, v)` to RGB `(r, g, b)` using the
/// specified [`YuvRange`].
#[must_use]
#[inline]
pub fn yuv2rgb_with_range(y: u8, u: u8, v: u8, range: YuvRange) -> (u8, u8, u8) {
    match range {
        YuvRange::Bt601Limited => {
            let yi = YUV_V_Y[y as usize];
            let ri = yi + YUV_V_VR[v as usize];
            let gi = yi + YUV_V_VG[u as usize] + YUV_V_UG[v as usize];
            let bi = yi + YUV_V_UB[u as usize];

            (
                ri.clamp(0, 255) as u8,
                gi.clamp(0, 255) as u8,
                bi.clamp(0, 255) as u8,
            )
        }
        YuvRange::Full => {
            let yi = i32::from(y);
            let cb = i32::from(u) - 128;
            let cr = i32::from(v) - 128;

            let r = clamp_u8(yi + ((91881 * cr + 32768) >> 16));
            let g = clamp_u8(yi - ((22554 * cb + 46802 * cr + 32768) >> 16));
            let b = clamp_u8(yi + ((116130 * cb + 32768) >> 16));
            (r, g, b)
        }
    }
}

/// Converts a macropixel pair `(y0, u, y1, v)` to two RGB `(r, g, b)` pixels
/// using [`YuvRange::Bt601Limited`], fetching table rows for `u`/`v` once.
#[must_use]
#[inline]
pub fn yuv422_pair_to_rgb(y0: u8, u: u8, y1: u8, v: u8) -> ((u8, u8, u8), (u8, u8, u8)) {
    yuv422_pair_to_rgb_with_range(y0, u, y1, v, YuvRange::Bt601Limited)
}

/// Converts a macropixel pair `(y0, u, y1, v)` to two RGB `(r, g, b)` pixels
/// sharing chroma `u` and `v` under the specified [`YuvRange`].
#[must_use]
#[inline]
pub fn yuv422_pair_to_rgb_with_range(
    y0: u8,
    u: u8,
    y1: u8,
    v: u8,
    range: YuvRange,
) -> ((u8, u8, u8), (u8, u8, u8)) {
    match range {
        YuvRange::Bt601Limited => {
            let v_vr = YUV_V_VR[v as usize];
            let u_ub = YUV_V_UB[u as usize];
            let uv_g = YUV_V_VG[u as usize] + YUV_V_UG[v as usize];

            let y0i = YUV_V_Y[y0 as usize];
            let r0 = (y0i + v_vr).clamp(0, 255) as u8;
            let g0 = (y0i + uv_g).clamp(0, 255) as u8;
            let b0 = (y0i + u_ub).clamp(0, 255) as u8;

            let y1i = YUV_V_Y[y1 as usize];
            let r1 = (y1i + v_vr).clamp(0, 255) as u8;
            let g1 = (y1i + uv_g).clamp(0, 255) as u8;
            let b1 = (y1i + u_ub).clamp(0, 255) as u8;

            ((r0, g0, b0), (r1, g1, b1))
        }
        YuvRange::Full => {
            let cb = i32::from(u) - 128;
            let cr = i32::from(v) - 128;
            let r_off = (91881 * cr + 32768) >> 16;
            let g_off = (22554 * cb + 46802 * cr + 32768) >> 16;
            let b_off = (116130 * cb + 32768) >> 16;

            let yi0 = i32::from(y0);
            let r0 = clamp_u8(yi0 + r_off);
            let g0 = clamp_u8(yi0 - g_off);
            let b0 = clamp_u8(yi0 + b_off);

            let yi1 = i32::from(y1);
            let r1 = clamp_u8(yi1 + r_off);
            let g1 = clamp_u8(yi1 - g_off);
            let b1 = clamp_u8(yi1 + b_off);

            ((r0, g0, b0), (r1, g1, b1))
        }
    }
}

#[inline]
const fn clamp_u8(v: i32) -> u8 {
    if v < 0 {
        0
    } else if v > 255 {
        255
    } else {
        v as u8
    }
}

/// Converts a single RGB888 pixel `(r, g, b)` to ITU-R BT.601 `(y, u, v)`
/// using the specified [`YuvRange`].
#[must_use]
#[inline]
pub const fn rgb2yuv_with_range(r: u8, g: u8, b: u8, range: YuvRange) -> (u8, u8, u8) {
    let r = r as i32;
    let g = g as i32;
    let b = b as i32;

    match range {
        YuvRange::Full => {
            let y = clamp_u8((19595 * r + 38470 * g + 7471 * b + 32768) >> 16);
            let u = clamp_u8(128 + ((-11059 * r - 21709 * g + 32768 * b + 32768) >> 16));
            let v = clamp_u8(128 + ((32768 * r - 27439 * g - 5329 * b + 32768) >> 16));
            (y, u, v)
        }
        YuvRange::Bt601Limited => {
            let y = clamp_u8(16 + ((16829 * r + 33039 * g + 6416 * b + 32768) >> 16));
            let u = clamp_u8(128 + ((-9714 * r - 19070 * g + 28784 * b + 32768) >> 16));
            let v = clamp_u8(128 + ((28784 * r - 24103 * g - 4681 * b + 32768) >> 16));
            (y, u, v)
        }
    }
}

/// Converts a single RGB888 pixel `(r, g, b)` to full-range JFIF `(y, cb, cr)`.
#[inline]
pub(crate) const fn rgb2yuv(r: u8, g: u8, b: u8) -> (u8, u8, u8) {
    rgb2yuv_with_range(r, g, b, YuvRange::Full)
}

/// Unpacks a big-endian RGB565 pixel (`[b0, b1]`) into full-range 8-bit RGB888
/// `(r, g, b)` using bit-replication (`LOGIC-063`, `CONV-15`).
///
/// Maps 5-bit channels `[0..31]` to `[0..255]` via `(v << 3) | (v >> 2)` and
/// 6-bit green `[0..63]` to `[0..255]` via `(v << 2) | (v >> 4)`.
#[must_use]
#[inline]
pub const fn unpack_rgb565_be_full(b0: u8, b1: u8) -> (u8, u8, u8) {
    let r5 = b0 >> 3;
    let r = (r5 << 3) | (r5 >> 2);

    let g6 = ((b0 & 0x07) << 3) | (b1 >> 5);
    let g = (g6 << 2) | (g6 >> 4);

    let b5 = b1 & 0x1F;
    let b = (b5 << 3) | (b5 >> 2);

    (r, g, b)
}

#[inline]
const fn div255_u16(n: u16) -> u8 {
    ((n + 1 + (n >> 8)) >> 8) as u8
}

#[inline]
const fn pack_rgb565_raw(r: u8, g: u8, b: u8) -> (u8, u8) {
    let r5 = div255_u16(r as u16 * 31 + 127);
    let g6 = div255_u16(g as u16 * 63 + 127);
    let b5 = div255_u16(b as u16 * 31 + 127);
    let hb = (r5 << 3) | (g6 >> 3);
    let lb = ((g6 & 0x07) << 5) | b5;
    (hb, lb)
}

fn checked_pixel_count(width: u16, height: u16, max_bpp: u32) -> Result<usize, CameraError> {
    if width == 0 || height == 0 {
        return Err(CameraError::InvalidArg);
    }
    let pixels32 = (width as u32)
        .checked_mul(height as u32)
        .ok_or(CameraError::Overflow)?;
    let bytes32 = pixels32.checked_mul(max_bpp).ok_or(CameraError::Overflow)?;
    if bytes32 > i32::MAX as u32 {
        return Err(CameraError::Overflow);
    }
    usize::try_from(pixels32).map_err(|_| CameraError::Overflow)
}

/// Converts a packed YUV422 (YUYV: `[Y0, U, Y1, V]`) buffer into
/// 8-bit grayscale by extracting `Y0` and `Y1` without inner bounds
/// checks (`LOGIC-061`).
///
/// Returns the number of bytes written to `dst` (`width * height`).
pub fn yuv422_to_grayscale(
    src: &[u8],
    width: u16,
    height: u16,
    dst: &mut [u8],
) -> Result<usize, CameraError> {
    let pixels = checked_pixel_count(width, height, 2)?;
    if (width & 1) != 0 {
        return Err(CameraError::InvalidArg);
    }
    let src_len = pixels.checked_mul(2).ok_or(CameraError::Overflow)?;
    let dst_len = pixels;
    if src.len() < src_len {
        return Err(CameraError::InvalidArg);
    }
    if dst.len() < dst_len {
        return Err(CameraError::BufferTooSmall);
    }

    let src_chunks = src[..src_len].as_chunks::<4>().0;
    let dst_chunks = dst[..dst_len].as_chunks_mut::<2>().0;

    for (chunk, out) in src_chunks.iter().zip(dst_chunks.iter_mut()) {
        *out = [chunk[0], chunk[2]];
    }

    Ok(dst_len)
}

/// Converts a big-endian RGB565 buffer into packed studio-range YUV422
/// (YUYV: `[Y0, U, Y1, V]`) using [`YuvRange::Bt601Limited`].
///
/// Returns the number of bytes written to `dst` (`width * height * 2`).
pub fn rgb565_to_yuv422(
    src: &[u8],
    dst: &mut [u8],
    width: u16,
    height: u16,
) -> Result<usize, CameraError> {
    rgb565_to_yuv422_with_range(src, dst, width, height, YuvRange::Bt601Limited)
}

/// Converts a big-endian RGB565 buffer into packed YUV422
/// (YUYV: `[Y0, U, Y1, V]`) using the specified [`YuvRange`].
///
/// Returns the number of bytes written to `dst` (`width * height * 2`).
pub fn rgb565_to_yuv422_with_range(
    src: &[u8],
    dst: &mut [u8],
    width: u16,
    height: u16,
    range: YuvRange,
) -> Result<usize, CameraError> {
    let pixels = checked_pixel_count(width, height, 2)?;
    if (width & 1) != 0 {
        return Err(CameraError::InvalidArg);
    }
    let byte_len = pixels.checked_mul(2).ok_or(CameraError::Overflow)?;
    if src.len() < byte_len {
        return Err(CameraError::InvalidArg);
    }
    if dst.len() < byte_len {
        return Err(CameraError::BufferTooSmall);
    }

    let src_chunks = src[..byte_len].as_chunks::<4>().0;
    let dst_chunks = dst[..byte_len].as_chunks_mut::<4>().0;

    for (chunk, out) in src_chunks.iter().zip(dst_chunks.iter_mut()) {
        let (r0, g0, b0) = unpack_rgb565_be_full(chunk[0], chunk[1]);
        let (r1, g1, b1) = unpack_rgb565_be_full(chunk[2], chunk[3]);

        let (y0, u0, v0) = rgb2yuv_with_range(r0, g0, b0, range);
        let (y1, u1, v1) = rgb2yuv_with_range(r1, g1, b1, range);

        *out = [
            y0,
            ((u16::from(u0) + u16::from(u1)) / 2) as u8,
            y1,
            ((u16::from(v0) + u16::from(v1)) / 2) as u8,
        ];
    }

    Ok(byte_len)
}

/// Converts a packed YUV422 (YUYV: `[Y0, U, Y1, V]`) buffer into RGB565 using
/// [`YuvRange::Bt601Limited`].
///
/// When `byte_swap` is `false`, writes big-endian RGB565 (`[hb, lb]`);
/// when `true`, swaps the byte order (`[lb, hb]`).
/// Returns the number of bytes written to `dst` (`width * height * 2`).
pub fn yuv422_to_rgb565(
    src: &[u8],
    dst: &mut [u8],
    width: u16,
    height: u16,
    byte_swap: bool,
) -> Result<usize, CameraError> {
    yuv422_to_rgb565_with_range(src, dst, width, height, byte_swap, YuvRange::Bt601Limited)
}

/// Converts a packed YUV422 (YUYV: `[Y0, U, Y1, V]`) buffer into RGB565 using
/// the specified [`YuvRange`].
///
/// When `byte_swap` is `false`, writes big-endian RGB565 (`[hb, lb]`);
/// when `true`, swaps the byte order (`[lb, hb]`).
/// Returns the number of bytes written to `dst` (`width * height * 2`).
pub fn yuv422_to_rgb565_with_range(
    src: &[u8],
    dst: &mut [u8],
    width: u16,
    height: u16,
    byte_swap: bool,
    range: YuvRange,
) -> Result<usize, CameraError> {
    let pixels = checked_pixel_count(width, height, 2)?;
    if (width & 1) != 0 {
        return Err(CameraError::InvalidArg);
    }
    let byte_len = pixels.checked_mul(2).ok_or(CameraError::Overflow)?;
    if src.len() < byte_len {
        return Err(CameraError::InvalidArg);
    }
    if dst.len() < byte_len {
        return Err(CameraError::BufferTooSmall);
    }

    let src_chunks = src[..byte_len].as_chunks::<4>().0;
    let dst_chunks = dst[..byte_len].as_chunks_mut::<4>().0;

    if byte_swap {
        for (chunk, out) in src_chunks.iter().zip(dst_chunks.iter_mut()) {
            let ((r0, g0, b0), (r1, g1, b1)) =
                yuv422_pair_to_rgb_with_range(chunk[0], chunk[1], chunk[2], chunk[3], range);
            let (hb0, lb0) = pack_rgb565_raw(r0, g0, b0);
            let (hb1, lb1) = pack_rgb565_raw(r1, g1, b1);
            *out = [lb0, hb0, lb1, hb1];
        }
    } else {
        for (chunk, out) in src_chunks.iter().zip(dst_chunks.iter_mut()) {
            let ((r0, g0, b0), (r1, g1, b1)) =
                yuv422_pair_to_rgb_with_range(chunk[0], chunk[1], chunk[2], chunk[3], range);
            let (hb0, lb0) = pack_rgb565_raw(r0, g0, b0);
            let (hb1, lb1) = pack_rgb565_raw(r1, g1, b1);
            *out = [hb0, lb0, hb1, lb1];
        }
    }

    Ok(byte_len)
}

/// Converts a packed YUV422 (YUYV: `[Y0, U, Y1, V]`) buffer into 24-bit
/// RGB888 (`[R, G, B]`) using [`YuvRange::Bt601Limited`].
///
/// Returns the number of bytes written to `dst` (`width * height * 3`).
pub fn yuv422_to_rgb888(
    src: &[u8],
    dst: &mut [u8],
    width: u16,
    height: u16,
) -> Result<usize, CameraError> {
    yuv422_to_rgb888_with_range(src, dst, width, height, YuvRange::Bt601Limited)
}

/// Converts a packed YUV422 (YUYV: `[Y0, U, Y1, V]`) buffer into 24-bit
/// RGB888 (`[R, G, B]`) using the specified [`YuvRange`].
///
/// Returns the number of bytes written to `dst` (`width * height * 3`).
pub fn yuv422_to_rgb888_with_range(
    src: &[u8],
    dst: &mut [u8],
    width: u16,
    height: u16,
    range: YuvRange,
) -> Result<usize, CameraError> {
    let pixels = checked_pixel_count(width, height, 3)?;
    if (width & 1) != 0 {
        return Err(CameraError::InvalidArg);
    }
    let src_len = pixels.checked_mul(2).ok_or(CameraError::Overflow)?;
    let dst_len = pixels.checked_mul(3).ok_or(CameraError::Overflow)?;
    if src.len() < src_len {
        return Err(CameraError::InvalidArg);
    }
    if dst.len() < dst_len {
        return Err(CameraError::BufferTooSmall);
    }

    let src_chunks = src[..src_len].as_chunks::<4>().0;
    let dst_chunks = dst[..dst_len].as_chunks_mut::<6>().0;

    for (chunk, out) in src_chunks.iter().zip(dst_chunks.iter_mut()) {
        let ((r0, g0, b0), (r1, g1, b1)) =
            yuv422_pair_to_rgb_with_range(chunk[0], chunk[1], chunk[2], chunk[3], range);
        *out = [r0, g0, b0, r1, g1, b1];
    }

    Ok(dst_len)
}

/// Converts a packed YUV422 (YUYV: `[Y0, U, Y1, V]`) buffer into planar
/// YUV420 (I420).
///
/// Requires `width` and `height` to be non-zero and even
/// (`width % 2 == 0`, `height % 2 == 0`).
/// Returns the number of bytes written to `dst` (`(width * height * 3) / 2`).
pub fn yuv422_to_yuv420(
    src: &[u8],
    dst: &mut [u8],
    width: u16,
    height: u16,
) -> Result<usize, CameraError> {
    if width == 0 || height == 0 || (width & 1) != 0 || (height & 1) != 0 {
        return Err(CameraError::InvalidArg);
    }
    let pixels = checked_pixel_count(width, height, 2)?;
    let src_len = pixels.checked_mul(2).ok_or(CameraError::Overflow)?;
    let chroma_plane_len = pixels / 4;
    let dst_len = pixels
        .checked_add(chroma_plane_len * 2)
        .ok_or(CameraError::Overflow)?;

    if src.len() < src_len {
        return Err(CameraError::InvalidArg);
    }
    if dst.len() < dst_len {
        return Err(CameraError::BufferTooSmall);
    }

    let w = width as usize;
    let half_w = w / 2;
    let src_row_bytes = w * 2;

    let (y_plane, uv_planes) = dst[..dst_len].split_at_mut(pixels);
    let (u_plane, v_plane) = uv_planes.split_at_mut(chroma_plane_len);

    let row_iter = src[..src_len]
        .chunks_exact(src_row_bytes * 2)
        .zip(y_plane.chunks_exact_mut(w * 2))
        .zip(u_plane.chunks_exact_mut(half_w))
        .zip(v_plane.chunks_exact_mut(half_w));

    for (((src_pair, y_pair), u_row), v_row) in row_iter {
        let (src_r0, src_r1) = src_pair.split_at(src_row_bytes);
        let (y_r0, y_r1) = y_pair.split_at_mut(w);

        let block_iter = src_r0
            .as_chunks::<4>()
            .0
            .iter()
            .zip(src_r1.as_chunks::<4>().0.iter())
            .zip(y_r0.as_chunks_mut::<2>().0.iter_mut())
            .zip(y_r1.as_chunks_mut::<2>().0.iter_mut())
            .zip(u_row.iter_mut())
            .zip(v_row.iter_mut());

        for (((((&[y00, u0, y01, v0], &[y10, u1, y11, v1]), y0_out), y1_out), u_out), v_out) in
            block_iter
        {
            *y0_out = [y00, y01];
            *y1_out = [y10, y11];
            *u_out = ((u16::from(u0) + u16::from(u1)) / 2) as u8;
            *v_out = ((u16::from(v0) + u16::from(v1)) / 2) as u8;
        }
    }

    Ok(dst_len)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn yuv422_to_yuv420_indexed_ref(src: &[u8], dst: &mut [u8], width: u16, height: u16) {
        let w = width as usize;
        let h = height as usize;
        let pixels = w * h;
        let u_offset = pixels;
        let v_offset = pixels + pixels / 4;

        for by in 0..(h / 2) {
            let row0 = by * 2;
            let row1 = row0 + 1;
            for bx in 0..(w / 2) {
                let x0 = bx * 2;
                let s0 = (row0 * w + x0) * 2;
                let s1 = (row1 * w + x0) * 2;

                dst[row0 * w + x0] = src[s0];
                dst[row0 * w + x0 + 1] = src[s0 + 2];
                dst[row1 * w + x0] = src[s1];
                dst[row1 * w + x0 + 1] = src[s1 + 2];

                let uv_idx = by * (w / 2) + bx;
                dst[u_offset + uv_idx] = ((src[s0 + 1] as u16 + src[s1 + 1] as u16) / 2) as u8;
                dst[v_offset + uv_idx] = ((src[s0 + 3] as u16 + src[s1 + 3] as u16) / 2) as u8;
            }
        }
    }

    #[test]
    fn test_yuv2rgb_bt601_table_values() {
        // Neutral black in BT.601 (Y=16, U=128, V=128) -> (0, 0, 0)
        assert_eq!(yuv2rgb(16, 128, 128), (0, 0, 0));
        // Neutral mid-gray (Y=128, U=128, V=128) -> (130, 130, 130) per
        // yuv_table[128].vY = 130
        assert_eq!(yuv2rgb(128, 128, 128), (130, 130, 130));
        // Clamping at extremes
        assert_eq!(yuv2rgb(0, 0, 0), (0, 136, 0));
        assert_eq!(yuv2rgb(255, 255, 255), (255, 126, 255));
    }

    #[test]
    fn test_conv12_yuv_table_size_and_all_24bit_inputs() {
        assert_eq!(
            core::mem::size_of_val(&YUV_V_Y),
            512,
            "each per-channel [i16; 256] table must occupy 512 bytes"
        );
        let total_size = core::mem::size_of_val(&YUV_V_Y)
            + core::mem::size_of_val(&YUV_V_VR)
            + core::mem::size_of_val(&YUV_V_VG)
            + core::mem::size_of_val(&YUV_V_UG)
            + core::mem::size_of_val(&YUV_V_UB);
        assert_eq!(
            total_size, 2560,
            "per-channel YUV tables should occupy 2560 bytes instead of 4096"
        );

        // Anchor rows from conversions/yuv.c:25-283 (indices 0, 16, 128, 235, 255).
        assert_eq!(
            (
                YUV_V_Y[0],
                YUV_V_VR[0],
                YUV_V_VG[0],
                YUV_V_UG[0],
                YUV_V_UB[0]
            ),
            (-18, -204, 50, 104, -258)
        );
        assert_eq!(
            (
                YUV_V_Y[16],
                YUV_V_VR[128],
                YUV_V_VG[128],
                YUV_V_UG[128],
                YUV_V_UB[128]
            ),
            (0, 0, 0, 0, 0)
        );
        assert_eq!(YUV_V_Y[235], 254);
        assert_eq!(
            (
                YUV_V_Y[255],
                YUV_V_VR[255],
                YUV_V_VG[255],
                YUV_V_UG[255],
                YUV_V_UB[255]
            ),
            (278, 202, -49, -103, 256)
        );

        let mut aos_rows = [[0i16; 5]; 256];
        for (i, row) in aos_rows.iter_mut().enumerate() {
            *row = [
                YUV_V_Y[i],
                YUV_V_VR[i],
                YUV_V_VG[i],
                YUV_V_UG[i],
                YUV_V_UB[i],
            ];
        }

        // Exhaustively verify all 2^24 (y, u, v) combinations for exact equality.
        for u in 0..=255u8 {
            let ub = aos_rows[u as usize][4];
            let ug_part = aos_rows[u as usize][2];
            for v in 0..=255u8 {
                let vr = aos_rows[v as usize][1];
                let g_chroma = ug_part + aos_rows[v as usize][3];
                for y in 0..=255u8 {
                    let yv = aos_rows[y as usize][0];
                    let expected = (
                        (yv + vr).clamp(0, 255) as u8,
                        (yv + g_chroma).clamp(0, 255) as u8,
                        (yv + ub).clamp(0, 255) as u8,
                    );
                    assert_eq!(yuv2rgb(y, u, v), expected);
                }
            }
        }
    }

    #[test]
    fn test_yuv2rgb_bt601_mul_max_delta_within_2() {
        const BLACK: (u8, u8, u8) = yuv2rgb_bt601_mul(16, 128, 128);
        assert_eq!(BLACK, (0, 0, 0));

        let samples: [u8; 18] = [
            0, 16, 32, 48, 64, 80, 96, 112, 128, 144, 160, 176, 192, 208, 224, 235, 240, 255,
        ];
        for y in samples {
            for u in samples {
                for v in samples {
                    let (rt, gt, bt) = yuv2rgb(y, u, v);
                    let (rm, gm, bm) = yuv2rgb_bt601_mul(y, u, v);
                    let dr = rt.abs_diff(rm);
                    let dg = gt.abs_diff(gm);
                    let db = bt.abs_diff(bm);
                    assert!(
                        dr <= 2 && dg <= 2 && db <= 2,
                        "delta > 2 at ({y}, {u}, {v}): table=({rt},{gt},{bt}) mul=({rm},{gm},{bm})"
                    );
                }
            }
        }
    }

    #[test]
    fn test_conv09_yuv_range_and_all_65536_rgb565_roundtrip() {
        assert_eq!(YuvRange::default(), YuvRange::Bt601Limited);

        // Full-range endpoints: black -> Y=0, white -> Y=255
        assert_eq!(rgb2yuv_with_range(0, 0, 0, YuvRange::Full), (0, 128, 128));
        assert_eq!(
            rgb2yuv_with_range(255, 255, 255, YuvRange::Full),
            (255, 128, 128)
        );
        assert_eq!(yuv2rgb_with_range(0, 128, 128, YuvRange::Full), (0, 0, 0));
        assert_eq!(
            yuv2rgb_with_range(255, 128, 128, YuvRange::Full),
            (255, 255, 255)
        );

        // Studio-range endpoints: black -> Y=16, white -> Y=235 (v_y=254 in yuv.c)
        assert_eq!(
            rgb2yuv_with_range(0, 0, 0, YuvRange::Bt601Limited),
            (16, 128, 128)
        );
        assert_eq!(
            rgb2yuv_with_range(255, 255, 255, YuvRange::Bt601Limited),
            (235, 128, 128)
        );
        assert_eq!(
            yuv2rgb_with_range(16, 128, 128, YuvRange::Bt601Limited),
            (0, 0, 0)
        );
        assert_eq!(
            yuv2rgb_with_range(235, 128, 128, YuvRange::Bt601Limited),
            (254, 254, 254)
        );

        // Exhaustive RGB565 -> YUV422 -> RGB565 roundtrip within +/-8 across
        // all 65,536 colors for both Bt601Limited and Full ranges.
        for range in [YuvRange::Bt601Limited, YuvRange::Full] {
            let mut yuv_buf = [0u8; 4];
            let mut out_buf = [0u8; 4];
            for code in 0..=u16::MAX {
                let [hb, lb] = code.to_be_bytes();
                let rgb565_pair = [hb, lb, hb, lb];
                let (r_in, g_in, b_in) = unpack_rgb565_be_full(hb, lb);

                rgb565_to_yuv422_with_range(&rgb565_pair, &mut yuv_buf, 2, 1, range)
                    .expect("rgb565_to_yuv422_with_range should succeed");
                yuv422_to_rgb565_with_range(&yuv_buf, &mut out_buf, 2, 1, false, range)
                    .expect("yuv422_to_rgb565_with_range should succeed");

                let (r_out, g_out, b_out) = unpack_rgb565_be_full(out_buf[0], out_buf[1]);
                let dr = (i32::from(r_out) - i32::from(r_in)).abs();
                let dg = (i32::from(g_out) - i32::from(g_in)).abs();
                let db = (i32::from(b_out) - i32::from(b_in)).abs();

                assert!(
                    dr <= 8 && dg <= 8 && db <= 8,
                    "range={range:?} code={code:#06x} in=({r_in},{g_in},{b_in}) \
                     out=({r_out},{g_out},{b_out})"
                );
                if range == YuvRange::Full {
                    assert_eq!((out_buf[0], out_buf[1]), (hb, lb));
                }
            }
        }
    }

    #[test]
    fn test_conv15_unpack_rgb565_be_full_bit_exactness() {
        assert_eq!(unpack_rgb565_be_full(0x00, 0x00), (0, 0, 0));
        assert_eq!(unpack_rgb565_be_full(0xFF, 0xFF), (255, 255, 255));
        assert_eq!(unpack_rgb565_be_full(0xF8, 0x00), (255, 0, 0));
        assert_eq!(unpack_rgb565_be_full(0x07, 0xE0), (0, 255, 0));
        assert_eq!(unpack_rgb565_be_full(0x00, 0x1F), (0, 0, 255));

        for code in 0..=u16::MAX {
            let [hb, lb] = code.to_be_bytes();
            let (r, g, b) = unpack_rgb565_be_full(hb, lb);
            assert_eq!(pack_rgb565_raw(r, g, b), (hb, lb));
        }

        // Verify rgb565_to_yuv422 uses full bit-replicated white (255,255,255)
        let white_pair = [0xFFu8; 4];
        let mut yuv_lim = [0u8; 4];
        let mut yuv_full = [0u8; 4];
        rgb565_to_yuv422(&white_pair, &mut yuv_lim, 2, 1)
            .expect("studio rgb565_to_yuv422 should succeed");
        rgb565_to_yuv422_with_range(&white_pair, &mut yuv_full, 2, 1, YuvRange::Full)
            .expect("full rgb565_to_yuv422_with_range should succeed");
        assert_eq!(yuv_lim, [235, 128, 235, 128]);
        assert_eq!(yuv_full, [255, 128, 255, 128]);
    }

    #[test]
    fn test_conv19_yuv422_to_yuv420_differential_64x32() {
        let width = 64u16;
        let height = 32u16;
        let mut src = [0u8; 64 * 32 * 2];
        let mut state = 0x9E37_79B9u32;
        for byte in &mut src {
            state ^= state << 13;
            state ^= state >> 17;
            state ^= state << 5;
            *byte = (state & 0xFF) as u8;
        }

        let mut dst_actual = [0u8; (64 * 32 * 3) / 2];
        let mut dst_expected = [0u8; (64 * 32 * 3) / 2];

        let written = yuv422_to_yuv420(&src, &mut dst_actual, width, height)
            .expect("64x32 yuv422_to_yuv420 should succeed");
        yuv422_to_yuv420_indexed_ref(&src, &mut dst_expected, width, height);

        assert_eq!(written, dst_expected.len());
        assert_eq!(dst_actual, dst_expected);

        // Reject odd height and undersized destination
        assert_eq!(
            yuv422_to_yuv420(&src[..24], &mut dst_actual, 4, 3),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(
            yuv422_to_yuv420(&src[..16], &mut dst_actual[..5], 2, 2),
            Err(CameraError::BufferTooSmall)
        );
    }

    #[test]
    fn test_rgb565_yuv422_roundtrip_and_byte_swap() {
        let width = 4u16;
        let height = 2u16;
        // 8 pixels of solid white in RGB565 (0xFFFF)
        let rgb565_in = [0xFFu8; 16];
        let mut yuv422 = [0u8; 16];
        let mut rgb565_be = [0u8; 16];
        let mut rgb565_le = [0u8; 16];

        let n_yuv = rgb565_to_yuv422(&rgb565_in, &mut yuv422, width, height)
            .expect("rgb565_to_yuv422 should succeed");
        assert_eq!(n_yuv, 16);

        let n_be = yuv422_to_rgb565(&yuv422, &mut rgb565_be, width, height, false)
            .expect("yuv422_to_rgb565 (BE) should succeed");
        let n_le = yuv422_to_rgb565(&yuv422, &mut rgb565_le, width, height, true)
            .expect("yuv422_to_rgb565 (LE) should succeed");
        assert_eq!(n_be, 16);
        assert_eq!(n_le, 16);
        assert_eq!(rgb565_be, [0xFFu8; 16]);

        for i in 0..8 {
            assert_eq!(rgb565_be[i * 2], rgb565_le[i * 2 + 1]);
            assert_eq!(rgb565_be[i * 2 + 1], rgb565_le[i * 2]);
        }
    }

    #[test]
    fn test_yuv422_to_rgb888_and_yuv420() {
        let width = 2u16;
        let height = 2u16;
        // 4 pixels (2x2), Y=128, U=128, V=128
        let yuv422 = [128, 128, 128, 128, 128, 128, 128, 128];
        let mut rgb888 = [0u8; 12];
        let written_rgb = yuv422_to_rgb888(&yuv422, &mut rgb888, width, height)
            .expect("yuv422_to_rgb888 should succeed");
        assert_eq!(written_rgb, 12);
        assert_eq!(rgb888, [130u8; 12]);

        let written_rgb_full =
            yuv422_to_rgb888_with_range(&yuv422, &mut rgb888, width, height, YuvRange::Full)
                .expect("yuv422_to_rgb888_with_range(Full) should succeed");
        assert_eq!(written_rgb_full, 12);
        assert_eq!(rgb888, [128u8; 12]);

        let mut yuv420 = [0u8; 6];
        let written_420 = yuv422_to_yuv420(&yuv422, &mut yuv420, width, height)
            .expect("yuv422_to_yuv420 should succeed");
        assert_eq!(written_420, 6);
        assert_eq!(yuv420, [128u8; 6]);
    }

    #[test]
    fn test_yuv_bounds_and_overflow_rejection() {
        let mut dst = [0u8; 16];
        assert_eq!(
            rgb565_to_yuv422(&[], &mut dst, u16::MAX, u16::MAX),
            Err(CameraError::Overflow)
        );
        assert_eq!(
            yuv422_to_rgb888(&[], &mut dst, u16::MAX, u16::MAX),
            Err(CameraError::Overflow)
        );
        assert_eq!(
            yuv422_to_yuv420(&[0u8; 8], &mut dst, 3, 2),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(
            yuv422_to_rgb888(&[0u8; 8], &mut dst[..4], 2, 2),
            Err(CameraError::BufferTooSmall)
        );
    }

    #[test]
    fn test_audit_07_yuv_001_odd_width_rejected() {
        let mut dst = [0u8; 32];
        let src = [0u8; 32];
        assert_eq!(
            rgb565_to_yuv422(&src, &mut dst, 3, 2),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(
            yuv422_to_rgb565(&src, &mut dst, 3, 2, false),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(
            yuv422_to_rgb888(&src, &mut dst, 3, 2),
            Err(CameraError::InvalidArg)
        );
    }
}

#[cfg(test)]
mod sweep_round_trip_tests {
    use super::*;

    /// RGB -> YUV -> RGB must come back close to where it started, for each
    /// range independently.
    ///
    /// This is what catches a mismatched pair: the forward transform uses
    /// integer coefficients while the `Bt601Limited` inverse reads lookup
    /// tables, so nothing in the types stops one being BT.601 studio-swing
    /// and the other full-range JFIF. A mismatch of that kind shows up as a
    /// large, biased round-trip error rather than as a compile failure.
    fn round_trip_error(range: YuvRange) -> (i32, f64) {
        let mut worst = 0i32;
        let mut total = 0f64;
        let mut n = 0u32;
        let mut r = 0u16;
        while r < 256 {
            let mut g = 0u16;
            while g < 256 {
                let mut b = 0u16;
                while b < 256 {
                    let (y, u, v) = rgb2yuv_with_range(r as u8, g as u8, b as u8, range);
                    let (r2, g2, b2) = yuv2rgb_with_range(y, u, v, range);
                    for (a, c) in [(r as u8, r2), (g as u8, g2), (b as u8, b2)] {
                        let d = (i32::from(a) - i32::from(c)).abs();
                        worst = worst.max(d);
                        total += f64::from(d);
                        n += 1;
                    }
                    b += 17;
                }
                g += 17;
            }
            r += 17;
        }
        (worst, total / f64::from(n))
    }

    #[test]
    fn test_sweep_rgb_yuv_round_trip_is_tight_for_each_range() {
        for (range, worst_allowed) in [(YuvRange::Full, 4), (YuvRange::Bt601Limited, 8)] {
            let (worst, mean) = round_trip_error(range);
            assert!(
                worst <= worst_allowed,
                "{range:?}: worst round-trip error {worst} exceeds {worst_allowed} \
                 (mean {mean:.3}) — forward and inverse may not be the same standard"
            );
            assert!(
                mean < 2.0,
                "{range:?}: mean round-trip error {mean:.3} is biased, which is what a \
                 mismatched coefficient pair looks like"
            );
        }
    }

    /// Crossing the ranges must be visibly worse than staying within one.
    /// If it is not, the two `YuvRange` arms are not actually different and
    /// the parameter is decorative.
    #[test]
    fn test_sweep_the_two_ranges_are_genuinely_different() {
        let (same, _) = round_trip_error(YuvRange::Full);
        let mut crossed = 0i32;
        let mut r = 0u16;
        while r < 256 {
            let (y, u, v) = rgb2yuv_with_range(r as u8, 128, 64, YuvRange::Full);
            let (r2, _, _) = yuv2rgb_with_range(y, u, v, YuvRange::Bt601Limited);
            crossed = crossed.max((i32::from(r as u8) - i32::from(r2)).abs());
            r += 17;
        }
        assert!(
            crossed > same,
            "mixing the ranges ({crossed}) is no worse than matching them ({same}); \
             the YuvRange parameter would then be doing nothing"
        );
    }
}
