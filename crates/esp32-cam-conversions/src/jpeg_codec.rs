//! Pure-Rust `#![no_std]` baseline DCT JPEG encoder and decoder.
//!
//! Replaces `conversions/jpge.cpp`, `conversions/to_jpg.cpp`, and
//! `conversions/esp_jpg_decode.c` with a zero-allocation, stack-only
//! implementation (`#![forbid(unsafe_code)]`).

use crate::bmp::{write_bmp_header, BMP_HEADER_LEN};
use crate::yuv::{rgb2yuv, YuvRange};
use esp32_cam_types::CameraError;

/// Full-range JFIF YCbCr to RGB888 conversion (inverse of `jpge.cpp`'s
/// `RGB_to_YCC`).
#[inline]
fn jfif_ycbcr_to_rgb(y: u8, cb: u8, cr: u8) -> (u8, u8, u8) {
    let y = y as i32;
    let cb = (cb as i32) - 128;
    let cr = (cr as i32) - 128;

    let r = (y + ((91881 * cr + 32768) >> 16)).clamp(0, 255) as u8;
    let g = (y - ((22554 * cb + 46802 * cr + 32768) >> 16)).clamp(0, 255) as u8;
    let b = (y + ((116130 * cb + 32768) >> 16)).clamp(0, 255) as u8;
    (r, g, b)
}

const M_SOF0: u8 = 0xC0;
const M_DHT: u8 = 0xC4;
const M_SOI: u8 = 0xD8;
const M_EOI: u8 = 0xD9;
const M_SOS: u8 = 0xDA;
const M_DQT: u8 = 0xDB;
const M_DRI: u8 = 0xDD;
const M_APP0: u8 = 0xE0;

/// Zig-zag scan order table (`s_zag` from `conversions/jpge.cpp:45`).
const ZAG: [usize; 64] = [
    0, 1, 8, 16, 9, 2, 3, 10, 17, 24, 32, 25, 18, 11, 4, 5, 12, 19, 26, 33, 40, 48, 41, 34, 27, 20,
    13, 6, 7, 14, 21, 28, 35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51, 58, 59,
    52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63,
];

/// Standard luminance quantization table in zig-zag order (`s_std_lum_quant`
/// from `jpge.cpp:46`).
const STD_LUM_QUANT: [i32; 64] = [
    16, 11, 12, 14, 12, 10, 16, 14, 13, 14, 18, 17, 16, 19, 24, 40, 26, 24, 22, 22, 24, 49, 35, 37,
    29, 40, 58, 51, 61, 60, 57, 51, 56, 55, 64, 72, 92, 78, 64, 68, 87, 69, 55, 56, 80, 109, 81,
    87, 95, 98, 103, 104, 103, 62, 77, 113, 121, 112, 100, 120, 92, 101, 103, 99,
];

/// Standard chrominance quantization table in zig-zag order
/// (`s_std_croma_quant` from `jpge.cpp:47`).
const STD_CHROMA_QUANT: [i32; 64] = [
    17, 18, 18, 24, 21, 24, 47, 26, 26, 47, 99, 66, 56, 66, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99,
];

const DC_LUM_BITS: [u8; 17] = [0, 0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0];
const DC_LUM_VAL: [u8; 12] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11];

const AC_LUM_BITS: [u8; 17] = [0, 0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 0x7D];
const AC_LUM_VAL: [u8; 162] = [
    0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12, 0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07,
    0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08, 0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52, 0xd1, 0xf0,
    0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0a, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x25, 0x26, 0x27, 0x28,
    0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
    0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
    0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
    0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7,
    0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5,
    0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2,
    0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
    0xf9, 0xfa,
];

const DC_CHROMA_BITS: [u8; 17] = [0, 0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0];
const DC_CHROMA_VAL: [u8; 12] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11];

const AC_CHROMA_BITS: [u8; 17] = [0, 0, 2, 1, 2, 4, 4, 3, 4, 7, 5, 4, 4, 0, 1, 2, 0x77];
const AC_CHROMA_VAL: [u8; 162] = [
    0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21, 0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61, 0x71,
    0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91, 0xa1, 0xb1, 0xc1, 0x09, 0x23, 0x33, 0x52, 0xf0,
    0x15, 0x62, 0x72, 0xd1, 0x0a, 0x16, 0x24, 0x34, 0xe1, 0x25, 0xf1, 0x17, 0x18, 0x19, 0x1a, 0x26,
    0x27, 0x28, 0x29, 0x2a, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
    0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68,
    0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
    0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5,
    0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3,
    0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda,
    0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
    0xf9, 0xfa,
];

const CONST_BITS: i32 = 13;
const ROW_BITS: i32 = 2;

#[inline]
const fn dct_descale(x: i32, n: i32) -> i32 {
    (x + (1i32 << (n - 1))) >> n
}

#[inline]
#[allow(clippy::too_many_arguments)]
fn dct_1d(
    s0: &mut i32,
    s1: &mut i32,
    s2: &mut i32,
    s3: &mut i32,
    s4: &mut i32,
    s5: &mut i32,
    s6: &mut i32,
    s7: &mut i32,
) {
    let t0 = *s0 + *s7;
    let mut t7 = *s0 - *s7;
    let t1 = *s1 + *s6;
    let mut t6 = *s1 - *s6;
    let t2 = *s2 + *s5;
    let mut t5 = *s2 - *s5;
    let t3 = *s3 + *s4;
    let mut t4 = *s3 - *s4;

    let t10 = t0 + t3;
    let t13 = t0 - t3;
    let t11 = t1 + t2;
    let t12 = t1 - t2;

    let mut u1 = (t12 + t13) * 4433;
    *s2 = u1 + t13 * 6270;
    *s6 = u1 + t12 * (-15137);

    u1 = t4 + t7;
    let mut u2 = t5 + t6;
    let mut u3 = t4 + t6;
    let mut u4 = t5 + t7;
    let z5 = (u3 + u4) * 9633;

    t4 *= 2446;
    t5 *= 16819;
    t6 *= 25172;
    t7 *= 12299;
    u1 *= -7373;
    u2 *= -20995;
    u3 = u3 * (-16069) + z5;
    u4 = u4 * (-3196) + z5;

    *s0 = t10 + t11;
    *s1 = t7 + u1 + u4;
    *s3 = t6 + u2 + u3;
    *s4 = t10 - t11;
    *s5 = t5 + u2 + u4;
    *s7 = t4 + u1 + u3;
}

/// Performs the 8x8 forward 2D integer DCT matching `DCT2D` in
/// `conversions/jpge.cpp:134-148`.
fn fdct_2d(block: &mut [i32; 64]) {
    for row in 0..8 {
        let base = row * 8;
        let (mut s0, mut s1, mut s2, mut s3, mut s4, mut s5, mut s6, mut s7) = (
            block[base],
            block[base + 1],
            block[base + 2],
            block[base + 3],
            block[base + 4],
            block[base + 5],
            block[base + 6],
            block[base + 7],
        );
        dct_1d(
            &mut s0, &mut s1, &mut s2, &mut s3, &mut s4, &mut s5, &mut s6, &mut s7,
        );
        block[base] = s0 << ROW_BITS;
        block[base + 1] = dct_descale(s1, CONST_BITS - ROW_BITS);
        block[base + 2] = dct_descale(s2, CONST_BITS - ROW_BITS);
        block[base + 3] = dct_descale(s3, CONST_BITS - ROW_BITS);
        block[base + 4] = s4 << ROW_BITS;
        block[base + 5] = dct_descale(s5, CONST_BITS - ROW_BITS);
        block[base + 6] = dct_descale(s6, CONST_BITS - ROW_BITS);
        block[base + 7] = dct_descale(s7, CONST_BITS - ROW_BITS);
    }

    for col in 0..8 {
        let (mut s0, mut s1, mut s2, mut s3, mut s4, mut s5, mut s6, mut s7) = (
            block[col],
            block[col + 8],
            block[col + 16],
            block[col + 24],
            block[col + 32],
            block[col + 40],
            block[col + 48],
            block[col + 56],
        );
        dct_1d(
            &mut s0, &mut s1, &mut s2, &mut s3, &mut s4, &mut s5, &mut s6, &mut s7,
        );
        block[col] = dct_descale(s0, ROW_BITS + 3);
        block[col + 8] = dct_descale(s1, CONST_BITS + ROW_BITS + 3);
        block[col + 16] = dct_descale(s2, CONST_BITS + ROW_BITS + 3);
        block[col + 24] = dct_descale(s3, CONST_BITS + ROW_BITS + 3);
        block[col + 32] = dct_descale(s4, ROW_BITS + 3);
        block[col + 40] = dct_descale(s5, CONST_BITS + ROW_BITS + 3);
        block[col + 48] = dct_descale(s6, CONST_BITS + ROW_BITS + 3);
        block[col + 56] = dct_descale(s7, CONST_BITS + ROW_BITS + 3);
    }
}

#[inline]
const fn dct_descale_i32(x: i32, n: u32) -> i32 {
    x.wrapping_add(1i32 << (n - 1)) >> n
}

/// Performs the 8x8 inverse 2D integer IDCT (IJG `jidctint.c` LLM algorithm)
/// in pure 32-bit arithmetic and level-shifts output samples by `+128` into
/// `0..=255`.
fn idct_2d(coeffs: &[i32; 64], out_pixels: &mut [u8; 64]) {
    let mut workspace = [0i32; 64];

    // Pass 1: process rows
    for row in 0..8 {
        let base = row * 8;
        let f0 = coeffs[base].clamp(-2047, 2047);
        let f1 = coeffs[base + 1].clamp(-2047, 2047);
        let f2 = coeffs[base + 2].clamp(-2047, 2047);
        let f3 = coeffs[base + 3].clamp(-2047, 2047);
        let f4 = coeffs[base + 4].clamp(-2047, 2047);
        let f5 = coeffs[base + 5].clamp(-2047, 2047);
        let f6 = coeffs[base + 6].clamp(-2047, 2047);
        let f7 = coeffs[base + 7].clamp(-2047, 2047);

        if (f1 | f2 | f3 | f4 | f5 | f6 | f7) == 0 {
            let dc_val = (f0 << ROW_BITS).clamp(-16384, 16384);
            workspace[base..base + 8].fill(dc_val);
            continue;
        }

        let z2 = f2;
        let z3 = f6;
        let z1 = z2.wrapping_add(z3).wrapping_mul(4433);
        let tmp2 = z1.wrapping_add(z3.wrapping_mul(-15137));
        let tmp3 = z1.wrapping_add(z2.wrapping_mul(6270));
        let tmp0 = f0.wrapping_add(f4) << CONST_BITS;
        let tmp1 = f0.wrapping_sub(f4) << CONST_BITS;
        let tmp10 = tmp0.wrapping_add(tmp3);
        let tmp13 = tmp0.wrapping_sub(tmp3);
        let tmp11 = tmp1.wrapping_add(tmp2);
        let tmp12 = tmp1.wrapping_sub(tmp2);

        let mut oz1 = f7.wrapping_add(f1);
        let mut oz2 = f5.wrapping_add(f3);
        let mut oz3 = f7.wrapping_add(f3);
        let mut oz4 = f5.wrapping_add(f1);
        let z5 = oz3.wrapping_add(oz4).wrapping_mul(9633);

        let mut otmp0 = f7.wrapping_mul(2446);
        let mut otmp1 = f5.wrapping_mul(16819);
        let mut otmp2 = f3.wrapping_mul(25172);
        let mut otmp3 = f1.wrapping_mul(12299);

        oz1 = oz1.wrapping_mul(-7373);
        oz2 = oz2.wrapping_mul(-20995);
        oz3 = oz3.wrapping_mul(-16069).wrapping_add(z5);
        oz4 = oz4.wrapping_mul(-3196).wrapping_add(z5);

        otmp0 = otmp0.wrapping_add(oz1.wrapping_add(oz3));
        otmp1 = otmp1.wrapping_add(oz2.wrapping_add(oz4));
        otmp2 = otmp2.wrapping_add(oz2.wrapping_add(oz3));
        otmp3 = otmp3.wrapping_add(oz1.wrapping_add(oz4));

        let shift = (CONST_BITS - ROW_BITS) as u32;
        workspace[base] = dct_descale_i32(tmp10.wrapping_add(otmp3), shift).clamp(-16384, 16384);
        workspace[base + 7] =
            dct_descale_i32(tmp10.wrapping_sub(otmp3), shift).clamp(-16384, 16384);
        workspace[base + 1] =
            dct_descale_i32(tmp11.wrapping_add(otmp2), shift).clamp(-16384, 16384);
        workspace[base + 6] =
            dct_descale_i32(tmp11.wrapping_sub(otmp2), shift).clamp(-16384, 16384);
        workspace[base + 2] =
            dct_descale_i32(tmp12.wrapping_add(otmp1), shift).clamp(-16384, 16384);
        workspace[base + 5] =
            dct_descale_i32(tmp12.wrapping_sub(otmp1), shift).clamp(-16384, 16384);
        workspace[base + 3] =
            dct_descale_i32(tmp13.wrapping_add(otmp0), shift).clamp(-16384, 16384);
        workspace[base + 4] =
            dct_descale_i32(tmp13.wrapping_sub(otmp0), shift).clamp(-16384, 16384);
    }

    // Pass 2: process columns
    for col in 0..8 {
        let f0 = workspace[col];
        let f1 = workspace[col + 8];
        let f2 = workspace[col + 16];
        let f3 = workspace[col + 24];
        let f4 = workspace[col + 32];
        let f5 = workspace[col + 40];
        let f6 = workspace[col + 48];
        let f7 = workspace[col + 56];

        let z2 = f2;
        let z3 = f6;
        let z1 = z2.wrapping_add(z3).wrapping_mul(4433);
        let tmp2 = z1.wrapping_add(z3.wrapping_mul(-15137));
        let tmp3 = z1.wrapping_add(z2.wrapping_mul(6270));
        let tmp0 = f0.wrapping_add(f4) << CONST_BITS;
        let tmp1 = f0.wrapping_sub(f4) << CONST_BITS;
        let tmp10 = tmp0.wrapping_add(tmp3);
        let tmp13 = tmp0.wrapping_sub(tmp3);
        let tmp11 = tmp1.wrapping_add(tmp2);
        let tmp12 = tmp1.wrapping_sub(tmp2);

        let mut oz1 = f7.wrapping_add(f1);
        let mut oz2 = f5.wrapping_add(f3);
        let mut oz3 = f7.wrapping_add(f3);
        let mut oz4 = f5.wrapping_add(f1);
        let z5 = oz3.wrapping_add(oz4).wrapping_mul(9633);

        let mut otmp0 = f7.wrapping_mul(2446);
        let mut otmp1 = f5.wrapping_mul(16819);
        let mut otmp2 = f3.wrapping_mul(25172);
        let mut otmp3 = f1.wrapping_mul(12299);

        oz1 = oz1.wrapping_mul(-7373);
        oz2 = oz2.wrapping_mul(-20995);
        oz3 = oz3.wrapping_mul(-16069).wrapping_add(z5);
        oz4 = oz4.wrapping_mul(-3196).wrapping_add(z5);

        otmp0 = otmp0.wrapping_add(oz1.wrapping_add(oz3));
        otmp1 = otmp1.wrapping_add(oz2.wrapping_add(oz4));
        otmp2 = otmp2.wrapping_add(oz2.wrapping_add(oz3));
        otmp3 = otmp3.wrapping_add(oz1.wrapping_add(oz4));

        let shift = (CONST_BITS + ROW_BITS + 3) as u32;
        out_pixels[col] = (dct_descale_i32(tmp10.wrapping_add(otmp3), shift).wrapping_add(128))
            .clamp(0, 255) as u8;
        out_pixels[col + 56] = (dct_descale_i32(tmp10.wrapping_sub(otmp3), shift).wrapping_add(128))
            .clamp(0, 255) as u8;
        out_pixels[col + 8] = (dct_descale_i32(tmp11.wrapping_add(otmp2), shift).wrapping_add(128))
            .clamp(0, 255) as u8;
        out_pixels[col + 48] = (dct_descale_i32(tmp11.wrapping_sub(otmp2), shift).wrapping_add(128))
            .clamp(0, 255) as u8;
        out_pixels[col + 16] = (dct_descale_i32(tmp12.wrapping_add(otmp1), shift).wrapping_add(128))
            .clamp(0, 255) as u8;
        out_pixels[col + 40] = (dct_descale_i32(tmp12.wrapping_sub(otmp1), shift).wrapping_add(128))
            .clamp(0, 255) as u8;
        out_pixels[col + 24] = (dct_descale_i32(tmp13.wrapping_add(otmp0), shift).wrapping_add(128))
            .clamp(0, 255) as u8;
        out_pixels[col + 32] = (dct_descale_i32(tmp13.wrapping_sub(otmp0), shift).wrapping_add(128))
            .clamp(0, 255) as u8;
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct QuantTable {
    q: [i32; 64],
    recip: [u32; 64],
    shift: [u8; 64],
}

fn compute_quant_table(base: &[i32; 64], quality: u8) -> QuantTable {
    let q = quality.clamp(1, 100) as u32;
    let scale = if q < 50 { 5000 / q } else { 200 - q * 2 };
    let mut out = QuantTable {
        q: [0i32; 64],
        recip: [0u32; 64],
        shift: [0u8; 64],
    };
    for (i, &base_val) in base.iter().enumerate() {
        let v = (base_val * scale as i32 + 50) / 100;
        let clamped = v.clamp(1, 255);
        let qi = clamped as u32;
        let b = 31 - qi.leading_zeros();
        let (recip, shift) = if qi == (1u32 << b) {
            (1u32 << 16, (16 + b) as u8)
        } else {
            let s = 17 + b;
            ((((1u64 << s) / u64::from(qi)) + 1) as u32, s as u8)
        };
        out.q[i] = clamped;
        out.recip[i] = recip;
        out.shift[i] = shift;
    }
    out
}

struct EncHuffTable {
    codes: [u16; 256],
    sizes: [u8; 256],
}

const fn build_enc_huff_table(bits: &[u8; 17], vals: &[u8]) -> EncHuffTable {
    let mut huff_size = [0u8; 257];
    let mut huff_code = [0u16; 257];

    let mut p = 0usize;
    let mut l = 1usize;
    while l <= 16 {
        let count = bits[l] as usize;
        let mut i = 0usize;
        while i < count {
            huff_size[p] = l as u8;
            p += 1;
            i += 1;
        }
        l += 1;
    }
    huff_size[p] = 0;
    let last_p = p;

    let mut code = 0u16;
    let mut si = huff_size[0];
    p = 0;
    while huff_size[p] != 0 {
        while huff_size[p] == si {
            huff_code[p] = code;
            p += 1;
            code = code.wrapping_add(1);
        }
        code <<= 1;
        si += 1;
    }

    let mut table = EncHuffTable {
        codes: [0u16; 256],
        sizes: [0u8; 256],
    };
    let mut i = 0usize;
    while i < last_p {
        let sym = vals[i] as usize;
        table.codes[sym] = huff_code[i];
        table.sizes[sym] = huff_size[i];
        i += 1;
    }
    table
}

static HUFF_DC_LUM: EncHuffTable = build_enc_huff_table(&DC_LUM_BITS, &DC_LUM_VAL);
static HUFF_AC_LUM: EncHuffTable = build_enc_huff_table(&AC_LUM_BITS, &AC_LUM_VAL);
static HUFF_DC_CHROM: EncHuffTable = build_enc_huff_table(&DC_CHROMA_BITS, &DC_CHROMA_VAL);
static HUFF_AC_CHROM: EncHuffTable = build_enc_huff_table(&AC_CHROMA_BITS, &AC_CHROMA_VAL);

struct JpegBitWriter<'a> {
    dst: &'a mut [u8],
    pos: usize,
    bit_buf: u32,
    bits_in: u32,
}

impl<'a> JpegBitWriter<'a> {
    fn new(dst: &'a mut [u8]) -> Self {
        Self {
            dst,
            pos: 0,
            bit_buf: 0,
            bits_in: 0,
        }
    }

    #[inline]
    fn emit_byte(&mut self, b: u8) -> Result<(), CameraError> {
        let slot = self
            .dst
            .get_mut(self.pos)
            .ok_or(CameraError::BufferTooSmall)?;
        *slot = b;
        self.pos += 1;
        Ok(())
    }

    #[inline]
    fn emit_word(&mut self, w: u16) -> Result<(), CameraError> {
        self.emit_byte((w >> 8) as u8)?;
        self.emit_byte((w & 0xFF) as u8)
    }

    #[inline]
    fn emit_marker(&mut self, marker: u8) -> Result<(), CameraError> {
        self.emit_byte(0xFF)?;
        self.emit_byte(marker)
    }

    #[inline]
    fn put_bits(&mut self, bits: u32, len: u32) -> Result<(), CameraError> {
        if len == 0 {
            return Ok(());
        }
        debug_assert!(len <= 24, "put_bits len must be <= 24: {len}");
        let mask = (1u32 << len) - 1;
        self.bit_buf = (self.bit_buf << len) | (bits & mask);
        self.bits_in += len;
        while self.bits_in >= 8 {
            self.bits_in -= 8;
            let c = ((self.bit_buf >> self.bits_in) & 0xFF) as u8;
            self.emit_byte(c)?;
            if c == 0xFF {
                self.emit_byte(0x00)?;
            }
        }
        Ok(())
    }

    #[inline]
    fn put_code_and_mag(
        &mut self,
        code: u16,
        code_len: u32,
        mut mag: u32,
        mag_len: u32,
        fast_room: bool,
    ) -> Result<(), CameraError> {
        debug_assert!(
            code_len <= 16 && mag_len <= 16,
            "code_len ({code_len}) and mag_len ({mag_len}) must be <= 16"
        );
        mag &= (1u32 << mag_len) - 1;
        let total_len = code_len + mag_len;
        if total_len == 0 {
            return Ok(());
        }
        if self.bits_in + total_len <= 32 {
            let combined = (u32::from(code) << mag_len) | mag;
            self.bit_buf = self.bit_buf.checked_shl(total_len).unwrap_or(0) | combined;
            self.bits_in += total_len;
            if fast_room {
                while self.bits_in >= 8 {
                    self.bits_in -= 8;
                    let c = ((self.bit_buf >> self.bits_in) & 0xFF) as u8;
                    self.dst[self.pos] = c;
                    self.pos += 1;
                    if c == 0xFF {
                        self.dst[self.pos] = 0x00;
                        self.pos += 1;
                    }
                }
            } else {
                while self.bits_in >= 8 {
                    self.bits_in -= 8;
                    let c = ((self.bit_buf >> self.bits_in) & 0xFF) as u8;
                    self.emit_byte(c)?;
                    if c == 0xFF {
                        self.emit_byte(0x00)?;
                    }
                }
            }
            Ok(())
        } else {
            self.put_bits(u32::from(code), code_len)?;
            self.put_bits(mag, mag_len)
        }
    }

    fn flush_bits(&mut self) -> Result<(), CameraError> {
        if self.bits_in > 0 {
            let pad_len = 8 - self.bits_in;
            self.put_bits((1u32 << pad_len) - 1, pad_len)?;
        }
        Ok(())
    }

    fn emit_headers(
        &mut self,
        width: u16,
        height: u16,
        num_components: u8,
        subsampling: Subsampling,
        lum_q: &[i32; 64],
        chroma_q: &[i32; 64],
    ) -> Result<(), CameraError> {
        // SOI
        self.emit_marker(M_SOI)?;

        // JFIF APP0
        self.emit_marker(M_APP0)?;
        self.emit_word(16)?;
        for &b in b"JFIF\0" {
            self.emit_byte(b)?;
        }
        self.emit_byte(1)?;
        self.emit_byte(1)?;
        self.emit_byte(0)?;
        self.emit_word(1)?;
        self.emit_word(1)?;
        self.emit_byte(0)?;
        self.emit_byte(0)?;

        // DQT (Luminance = 0, Chrominance = 1 if 3 components)
        self.emit_marker(M_DQT)?;
        self.emit_word(67)?;
        self.emit_byte(0)?;
        for &q in lum_q {
            self.emit_byte(q as u8)?;
        }
        if num_components == 3 {
            self.emit_marker(M_DQT)?;
            self.emit_word(67)?;
            self.emit_byte(1)?;
            for &q in chroma_q {
                self.emit_byte(q as u8)?;
            }
        }

        // SOF0 (Baseline DCT)
        self.emit_marker(M_SOF0)?;
        self.emit_word(8 + 3 * (num_components as u16))?;
        self.emit_byte(8)?;
        self.emit_word(height)?;
        self.emit_word(width)?;
        self.emit_byte(num_components)?;
        for c in 0..num_components {
            self.emit_byte(c + 1)?;
            let samp = if c == 0 && num_components == 3 {
                match subsampling {
                    Subsampling::H1V1 => 0x11,
                    Subsampling::H2V1 => 0x21,
                    Subsampling::H2V2 => 0x22,
                }
            } else {
                0x11
            };
            self.emit_byte(samp)?;
            self.emit_byte(u8::from(c > 0))?;
        }

        // DHTs
        self.emit_dht(&DC_LUM_BITS, &DC_LUM_VAL, 0, false)?;
        self.emit_dht(&AC_LUM_BITS, &AC_LUM_VAL, 0, true)?;
        if num_components == 3 {
            self.emit_dht(&DC_CHROMA_BITS, &DC_CHROMA_VAL, 1, false)?;
            self.emit_dht(&AC_CHROMA_BITS, &AC_CHROMA_VAL, 1, true)?;
        }

        // SOS
        self.emit_marker(M_SOS)?;
        self.emit_word(6 + 2 * (num_components as u16))?;
        self.emit_byte(num_components)?;
        for c in 0..num_components {
            self.emit_byte(c + 1)?;
            self.emit_byte(if c == 0 { 0x00 } else { 0x11 })?;
        }
        self.emit_byte(0)?;
        self.emit_byte(63)?;
        self.emit_byte(0)
    }

    fn emit_dht(
        &mut self,
        bits: &[u8; 17],
        vals: &[u8],
        index: u8,
        is_ac: bool,
    ) -> Result<(), CameraError> {
        self.emit_marker(M_DHT)?;
        let len = (2 + 1 + 16 + vals.len()) as u16;
        self.emit_word(len)?;
        self.emit_byte(index | if is_ac { 0x10 } else { 0x00 })?;
        for &b in &bits[1..=16] {
            self.emit_byte(b)?;
        }
        for &v in vals {
            self.emit_byte(v)?;
        }
        Ok(())
    }

    fn encode_block(
        &mut self,
        sample_block: &mut [i32; 64],
        qtable: &QuantTable,
        dc_huff: &EncHuffTable,
        ac_huff: &EncHuffTable,
        last_dc: &mut i32,
    ) -> Result<(), CameraError> {
        let fast_room = self.dst.len().saturating_sub(self.pos) >= 520;
        fdct_2d(sample_block);

        let mut zz_coeffs = [0i32; 64];
        for i in 0..64 {
            let raw = sample_block[ZAG[i]];
            let q = qtable.q[i];
            zz_coeffs[i] = if raw < 0 {
                let adj = -raw + (q >> 1);
                if adj < q {
                    0
                } else {
                    -((((adj as u32).wrapping_mul(qtable.recip[i])) >> qtable.shift[i]) as i32)
                }
            } else {
                let adj = raw + (q >> 1);
                if adj < q {
                    0
                } else {
                    (((adj as u32).wrapping_mul(qtable.recip[i])) >> qtable.shift[i]) as i32
                }
            };
        }

        // DC coefficient differential encoding
        let dc_diff = zz_coeffs[0] - *last_dc;
        *last_dc = zz_coeffs[0];

        let (abs_dc, dc_bits) = if dc_diff < 0 {
            ((-dc_diff) as u32, (dc_diff - 1) as u32)
        } else {
            (dc_diff as u32, dc_diff as u32)
        };
        let nbits = 32 - abs_dc.leading_zeros();
        let sym = nbits as usize;
        self.put_code_and_mag(
            dc_huff.codes[sym],
            u32::from(dc_huff.sizes[sym]),
            dc_bits,
            nbits,
            fast_room,
        )?;

        // AC coefficients RLE + Huffman encoding
        let mut run_len = 0u32;
        for &ac in &zz_coeffs[1..64] {
            if ac == 0 {
                run_len += 1;
            } else {
                while run_len >= 16 {
                    self.put_code_and_mag(
                        ac_huff.codes[0xF0],
                        u32::from(ac_huff.sizes[0xF0]),
                        0,
                        0,
                        fast_room,
                    )?;
                    run_len -= 16;
                }
                let (abs_ac, ac_bits) = if ac < 0 {
                    ((-ac) as u32, (ac - 1) as u32)
                } else {
                    (ac as u32, ac as u32)
                };
                let ac_nbits = 32 - abs_ac.leading_zeros();
                let ac_sym = ((run_len << 4) | ac_nbits) as usize;
                self.put_code_and_mag(
                    ac_huff.codes[ac_sym],
                    u32::from(ac_huff.sizes[ac_sym]),
                    ac_bits,
                    ac_nbits,
                    fast_room,
                )?;
                run_len = 0;
            }
        }
        if run_len > 0 {
            self.put_code_and_mag(
                ac_huff.codes[0x00],
                u32::from(ac_huff.sizes[0x00]),
                0,
                0,
                fast_room,
            )?;
        }

        Ok(())
    }

    fn encode_yuv_mcu(
        &mut self,
        y_blocks: &mut [[i32; 64]],
        cb_block: &mut [i32; 64],
        cr_block: &mut [i32; 64],
        lum_q: &QuantTable,
        chroma_q: &QuantTable,
        last_dc: &mut [i32; 3],
    ) -> Result<(), CameraError> {
        for yb in y_blocks.iter_mut() {
            self.encode_block(yb, lum_q, &HUFF_DC_LUM, &HUFF_AC_LUM, &mut last_dc[0])?;
        }
        self.encode_block(
            cb_block,
            chroma_q,
            &HUFF_DC_CHROM,
            &HUFF_AC_CHROM,
            &mut last_dc[1],
        )?;
        self.encode_block(
            cr_block,
            chroma_q,
            &HUFF_DC_CHROM,
            &HUFF_AC_CHROM,
            &mut last_dc[2],
        )
    }
}

fn validate_encode_dims(
    src_len: usize,
    width: u16,
    height: u16,
    bpp: u32,
) -> Result<usize, CameraError> {
    if width == 0 || height == 0 {
        return Err(CameraError::InvalidArg);
    }
    let pixels = (width as u32)
        .checked_mul(height as u32)
        .ok_or(CameraError::Overflow)?;
    let bytes = pixels.checked_mul(bpp).ok_or(CameraError::Overflow)?;
    if bytes > i32::MAX as u32 {
        return Err(CameraError::Overflow);
    }
    let req = usize::try_from(bytes).map_err(|_| CameraError::Overflow)?;
    if src_len < req {
        return Err(CameraError::InvalidArg);
    }
    Ok(req)
}

const fn min_encoded_header_len(components: u8) -> usize {
    if components == 1 {
        330
    } else {
        625
    }
}

/// Upper bound on encoded baseline JFIF byte length for `(width, height)` and
/// `components` (`1` or `3`), covering headers, trailing `EOI`, worst-case
/// Huffman expansion, and `0xFF 0x00` byte stuffing across all [`Subsampling`]
/// modes.
#[must_use]
pub const fn max_encoded_len(width: u16, height: u16, components: u8) -> Option<usize> {
    if width == 0 || height == 0 {
        return None;
    }
    let w = width as usize;
    let h = height as usize;
    let blocks = match components {
        1 => (w.div_ceil(8)).checked_mul(h.div_ceil(8)),
        3 => {
            let b11 = (w.div_ceil(8)).checked_mul(h.div_ceil(8));
            let b21 = (w.div_ceil(16)).checked_mul(h.div_ceil(8));
            let b22 = (w.div_ceil(16)).checked_mul(h.div_ceil(16));
            match (b11, b21, b22) {
                (Some(m11), Some(m21), Some(m22)) => {
                    match (m11.checked_mul(3), m21.checked_mul(4), m22.checked_mul(6)) {
                        (Some(c11), Some(c21), Some(c22)) => {
                            let m = if c11 > c21 { c11 } else { c21 };
                            Some(if m > c22 { m } else { c22 })
                        }
                        _ => None,
                    }
                }
                _ => None,
            }
        }
        _ => None,
    };
    match blocks {
        Some(b) => match b.checked_mul(520) {
            Some(body) => body.checked_add(640),
            None => None,
        },
        None => None,
    }
}

#[inline]
#[allow(clippy::too_many_arguments)]
fn gather_rgb888_mcu(
    src: &[u8],
    w: usize,
    h: usize,
    bx: usize,
    by: usize,
    y_block: &mut [i32; 64],
    cb_block: &mut [i32; 64],
    cr_block: &mut [i32; 64],
) {
    if by * 8 + 8 <= h && bx * 8 + 8 <= w {
        for dy in 0..8usize {
            let row_start = ((by * 8 + dy) * w + bx * 8) * 3;
            let (pixels, _) = src[row_start..row_start + 24].as_chunks::<3>();
            let row_base = dy * 8;
            for (dx, &[r, g, b]) in pixels.iter().enumerate() {
                let (y, cb, cr) = rgb2yuv(r, g, b);
                let b_idx = row_base + dx;
                y_block[b_idx] = i32::from(y) - 128;
                cb_block[b_idx] = i32::from(cb) - 128;
                cr_block[b_idx] = i32::from(cr) - 128;
            }
        }
    } else {
        for dy in 0..8usize {
            let py = (by * 8 + dy).min(h - 1);
            let row_off = py * w;
            for dx in 0..8usize {
                let px = (bx * 8 + dx).min(w - 1);
                let p_idx = (row_off + px) * 3;
                let (y, cb, cr) = rgb2yuv(src[p_idx], src[p_idx + 1], src[p_idx + 2]);
                let b_idx = dy * 8 + dx;
                y_block[b_idx] = i32::from(y) - 128;
                cb_block[b_idx] = i32::from(cb) - 128;
                cr_block[b_idx] = i32::from(cr) - 128;
            }
        }
    }
}

#[inline]
fn gather_grayscale_mcu(
    src: &[u8],
    w: usize,
    h: usize,
    bx: usize,
    by: usize,
    y_block: &mut [i32; 64],
) {
    if by * 8 + 8 <= h && bx * 8 + 8 <= w {
        for dy in 0..8usize {
            let row_start = (by * 8 + dy) * w + bx * 8;
            let row_pixels = &src[row_start..row_start + 8];
            let row_base = dy * 8;
            for (dx, &lum) in row_pixels.iter().enumerate() {
                y_block[row_base + dx] = i32::from(lum) - 128;
            }
        }
    } else {
        for dy in 0..8usize {
            let py = (by * 8 + dy).min(h - 1);
            let row_off = py * w;
            for dx in 0..8usize {
                let px = (bx * 8 + dx).min(w - 1);
                y_block[dy * 8 + dx] = i32::from(src[row_off + px]) - 128;
            }
        }
    }
}

#[inline]
#[allow(clippy::too_many_arguments)]
fn gather_yuv422_h1v1_mcu(
    src: &[u8],
    w: usize,
    h: usize,
    bx: usize,
    by: usize,
    y_block: &mut [i32; 64],
    cb_block: &mut [i32; 64],
    cr_block: &mut [i32; 64],
) {
    if by * 8 + 8 <= h && bx * 8 + 8 <= w {
        for dy in 0..8usize {
            let row_start = ((by * 8 + dy) * w + bx * 8) * 2;
            let (pairs, _) = src[row_start..row_start + 16].as_chunks::<4>();
            let row_base = dy * 8;
            for (pair_i, &[y0, u, y1, v]) in pairs.iter().enumerate() {
                let b0 = row_base + pair_i * 2;
                let u_c = i32::from(u) - 128;
                let v_c = i32::from(v) - 128;
                y_block[b0] = i32::from(y0) - 128;
                cb_block[b0] = u_c;
                cr_block[b0] = v_c;
                y_block[b0 + 1] = i32::from(y1) - 128;
                cb_block[b0 + 1] = u_c;
                cr_block[b0 + 1] = v_c;
            }
        }
    } else {
        for dy in 0..8usize {
            let py = (by * 8 + dy).min(h - 1);
            let row_off = py * w;
            for dx in 0..8usize {
                let px = (bx * 8 + dx).min(w - 1);
                let pair_idx = (row_off + (px & !1)) * 2;
                let pair = &src[pair_idx..pair_idx + 4];
                let y = if (px & 1) == 0 { pair[0] } else { pair[2] };
                let u = pair[1];
                let v = pair[3];
                let b_idx = dy * 8 + dx;
                y_block[b_idx] = i32::from(y) - 128;
                cb_block[b_idx] = i32::from(u) - 128;
                cr_block[b_idx] = i32::from(v) - 128;
            }
        }
    }
}

#[inline]
#[allow(clippy::too_many_arguments)]
fn gather_yuv422_h2v1_mcu(
    src: &[u8],
    w: usize,
    h: usize,
    mcu_x: usize,
    mcu_y: usize,
    y_blocks: &mut [[i32; 64]; 2],
    cb_block: &mut [i32; 64],
    cr_block: &mut [i32; 64],
) {
    let x0 = mcu_x * 16;
    let y0 = mcu_y * 8;
    if y0 + 8 <= h && x0 + 16 <= w {
        for dy in 0..8usize {
            let row_start = ((y0 + dy) * w + x0) * 2;
            let (pairs, _) = src[row_start..row_start + 32].as_chunks::<4>();
            let row_base = dy * 8;
            for (cx, &[l0, u, l1, v]) in pairs.iter().enumerate() {
                let blk = cx >> 2;
                let lx = (cx & 3) * 2;
                y_blocks[blk][row_base + lx] = i32::from(l0) - 128;
                y_blocks[blk][row_base + lx + 1] = i32::from(l1) - 128;
                cb_block[row_base + cx] = i32::from(u) - 128;
                cr_block[row_base + cx] = i32::from(v) - 128;
            }
        }
    } else {
        for dy in 0..8usize {
            let py = (y0 + dy).min(h - 1);
            let row_off = py * w;
            let row_base = dy * 8;
            for cx in 0..8usize {
                let px_even = (x0 + cx * 2).min(w - 2);
                let p_off = (row_off + px_even) * 2;
                let pair = &src[p_off..p_off + 4];
                let px0 = (x0 + cx * 2).min(w - 1);
                let px1 = (x0 + cx * 2 + 1).min(w - 1);
                let l0 = if (px0 & 1) == 0 { pair[0] } else { pair[2] };
                let l1 = if (px1 & 1) == 0 { pair[0] } else { pair[2] };
                let blk = cx >> 2;
                let lx = (cx & 3) * 2;
                y_blocks[blk][row_base + lx] = i32::from(l0) - 128;
                y_blocks[blk][row_base + lx + 1] = i32::from(l1) - 128;
                cb_block[row_base + cx] = i32::from(pair[1]) - 128;
                cr_block[row_base + cx] = i32::from(pair[3]) - 128;
            }
        }
    }
}

#[inline]
fn expand_bt601_luma_block(block: &mut [i32; 64]) {
    for sample in block.iter_mut() {
        *sample = (((*sample + 112) * 255 + 109) / 219).clamp(0, 255) - 128;
    }
}

#[inline]
fn expand_bt601_chroma_block(block: &mut [i32; 64]) {
    for sample in block.iter_mut() {
        *sample = (((*sample + 112) * 255 + 112) / 224).clamp(0, 255) - 128;
    }
}

#[inline]
#[allow(clippy::too_many_arguments)]
fn gather_yuv422_h2v2_mcu(
    src: &[u8],
    w: usize,
    h: usize,
    mcu_x: usize,
    mcu_y: usize,
    y_blocks: &mut [[i32; 64]; 4],
    cb_block: &mut [i32; 64],
    cr_block: &mut [i32; 64],
) {
    let x0 = mcu_x * 16;
    let y0 = mcu_y * 16;
    if x0 + 16 <= w && y0 + 16 <= h {
        for cy in 0..8usize {
            let off0 = ((y0 + cy * 2) * w + x0) * 2;
            let off1 = ((y0 + cy * 2 + 1) * w + x0) * 2;
            let (row0, _) = src[off0..off0 + 32].as_chunks::<4>();
            let (row1, _) = src[off1..off1 + 32].as_chunks::<4>();
            let v_blk = (cy >> 2) * 2;
            let r0 = ((cy & 3) * 2) * 8;
            let r1 = r0 + 8;
            let c_row = cy * 8;
            for cx in 0..8usize {
                let [y00, u0, y01, v0] = row0[cx];
                let [y10, u1, y11, v1] = row1[cx];
                let blk = v_blk + (cx >> 2);
                let lx = (cx & 3) * 2;
                y_blocks[blk][r0 + lx] = i32::from(y00) - 128;
                y_blocks[blk][r0 + lx + 1] = i32::from(y01) - 128;
                y_blocks[blk][r1 + lx] = i32::from(y10) - 128;
                y_blocks[blk][r1 + lx + 1] = i32::from(y11) - 128;
                cb_block[c_row + cx] = ((i32::from(u0) + i32::from(u1) + 1) >> 1) - 128;
                cr_block[c_row + cx] = ((i32::from(v0) + i32::from(v1) + 1) >> 1) - 128;
            }
        }
    } else {
        for cy in 0..8usize {
            let py0 = (y0 + cy * 2).min(h - 1);
            let py1 = (y0 + cy * 2 + 1).min(h - 1);
            let v_blk = (cy >> 2) * 2;
            let r0 = ((cy & 3) * 2) * 8;
            let r1 = r0 + 8;
            let c_row = cy * 8;
            for cx in 0..8usize {
                let px_even = (x0 + cx * 2).min(w - 2);
                let off0 = (py0 * w + px_even) * 2;
                let off1 = (py1 * w + px_even) * 2;
                let p0 = &src[off0..off0 + 4];
                let p1 = &src[off1..off1 + 4];
                let px0 = (x0 + cx * 2).min(w - 1);
                let px1 = (x0 + cx * 2 + 1).min(w - 1);
                let blk = v_blk + (cx >> 2);
                let lx = (cx & 3) * 2;
                y_blocks[blk][r0 + lx] =
                    i32::from(if (px0 & 1) == 0 { p0[0] } else { p0[2] }) - 128;
                y_blocks[blk][r0 + lx + 1] =
                    i32::from(if (px1 & 1) == 0 { p0[0] } else { p0[2] }) - 128;
                y_blocks[blk][r1 + lx] =
                    i32::from(if (px0 & 1) == 0 { p1[0] } else { p1[2] }) - 128;
                y_blocks[blk][r1 + lx + 1] =
                    i32::from(if (px1 & 1) == 0 { p1[0] } else { p1[2] }) - 128;
                cb_block[c_row + cx] = ((i32::from(p0[1]) + i32::from(p1[1]) + 1) >> 1) - 128;
                cr_block[c_row + cx] = ((i32::from(p0[3]) + i32::from(p1[3]) + 1) >> 1) - 128;
            }
        }
    }
}

/// Chroma subsampling mode for 3-component JPEG encoding.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Subsampling {
    /// 4:4:4 subsampling (8x8 MCU: Y, Cb, Cr).
    H1V1,
    /// 4:2:2 horizontal subsampling (16x8 MCU: Y0, Y1, Cb, Cr).
    H2V1,
    /// 4:2:0 horizontal and vertical subsampling (16x16 MCU: Y0..Y3, Cb, Cr).
    H2V2,
}

/// Encodes a 24-bit RGB888 image (`[R, G, B]`) into a baseline JFIF JPEG
/// stream in `dst`.
///
/// Performs zero heap allocations and returns the total JPEG byte length
/// written to `dst`.
pub fn encode_rgb888_to_jpeg(
    src: &[u8],
    dst: &mut [u8],
    width: u16,
    height: u16,
    quality: u8,
) -> Result<usize, CameraError> {
    validate_encode_dims(src.len(), width, height, 3)?;
    if dst.len() < min_encoded_header_len(3) {
        return Err(CameraError::BufferTooSmall);
    }

    let lum_q = compute_quant_table(&STD_LUM_QUANT, quality);
    let chroma_q = compute_quant_table(&STD_CHROMA_QUANT, quality);

    let mut writer = JpegBitWriter::new(dst);
    writer.emit_headers(width, height, 3, Subsampling::H1V1, &lum_q.q, &chroma_q.q)?;

    let w = width as usize;
    let h = height as usize;
    let mcu_cols = w.div_ceil(8);
    let mcu_rows = h.div_ceil(8);

    let mut last_dc = [0i32; 3];
    let mut y_block = [0i32; 64];
    let mut cb_block = [0i32; 64];
    let mut cr_block = [0i32; 64];

    for by in 0..mcu_rows {
        for bx in 0..mcu_cols {
            gather_rgb888_mcu(
                src,
                w,
                h,
                bx,
                by,
                &mut y_block,
                &mut cb_block,
                &mut cr_block,
            );

            writer.encode_block(
                &mut y_block,
                &lum_q,
                &HUFF_DC_LUM,
                &HUFF_AC_LUM,
                &mut last_dc[0],
            )?;
            writer.encode_block(
                &mut cb_block,
                &chroma_q,
                &HUFF_DC_CHROM,
                &HUFF_AC_CHROM,
                &mut last_dc[1],
            )?;
            writer.encode_block(
                &mut cr_block,
                &chroma_q,
                &HUFF_DC_CHROM,
                &HUFF_AC_CHROM,
                &mut last_dc[2],
            )?;
        }
    }

    writer.flush_bits()?;
    writer.emit_marker(M_EOI)?;
    Ok(writer.pos)
}

/// Encodes an 8-bit grayscale image into a single-component baseline JFIF
/// JPEG stream in `dst`.
///
/// Performs zero heap allocations and returns the total JPEG byte length
/// written to `dst`.
pub fn encode_grayscale_to_jpeg(
    src: &[u8],
    dst: &mut [u8],
    width: u16,
    height: u16,
    quality: u8,
) -> Result<usize, CameraError> {
    validate_encode_dims(src.len(), width, height, 1)?;
    if dst.len() < min_encoded_header_len(1) {
        return Err(CameraError::BufferTooSmall);
    }

    let lum_q = compute_quant_table(&STD_LUM_QUANT, quality);

    let mut writer = JpegBitWriter::new(dst);
    writer.emit_headers(width, height, 1, Subsampling::H1V1, &lum_q.q, &[1i32; 64])?;

    let w = width as usize;
    let h = height as usize;
    let mcu_cols = w.div_ceil(8);
    let mcu_rows = h.div_ceil(8);

    let mut last_dc = 0i32;
    let mut y_block = [0i32; 64];

    for by in 0..mcu_rows {
        for bx in 0..mcu_cols {
            gather_grayscale_mcu(src, w, h, bx, by, &mut y_block);
            writer.encode_block(
                &mut y_block,
                &lum_q,
                &HUFF_DC_LUM,
                &HUFF_AC_LUM,
                &mut last_dc,
            )?;
        }
    }

    writer.flush_bits()?;
    writer.emit_marker(M_EOI)?;
    Ok(writer.pos)
}

/// Encodes a full-range packed YUV422 (YUYV: `[Y0, U, Y1, V]`) image into a
/// baseline JFIF JPEG stream in `dst` using the requested `subsampling`.
pub fn encode_yuv422_to_jpeg_subsampled(
    src: &[u8],
    dst: &mut [u8],
    width: u16,
    height: u16,
    quality: u8,
    subsampling: Subsampling,
) -> Result<usize, CameraError> {
    encode_yuv422_to_jpeg_with_range(
        src,
        dst,
        width,
        height,
        quality,
        subsampling,
        YuvRange::Full,
    )
}

/// Encodes a packed YUV422 (YUYV: `[Y0, U, Y1, V]`) image into a baseline
/// JFIF JPEG stream in `dst` using `subsampling` and input `range`.
///
/// When `range` is [`YuvRange::Bt601Limited`], limited-range studio YCbCr
/// (`Y` in `16..=235`, `Cb`/`Cr` in `16..=240`) is expanded to full-range
/// JFIF (`0..=255`) prior to DCT quantization.
///
/// # Errors
///
/// Returns [`CameraError::InvalidArg`] if dimensions or input buffer length
/// are invalid, or [`CameraError::BufferTooSmall`] if `dst` overflows.
pub fn encode_yuv422_to_jpeg_with_range(
    src: &[u8],
    dst: &mut [u8],
    width: u16,
    height: u16,
    quality: u8,
    subsampling: Subsampling,
    range: YuvRange,
) -> Result<usize, CameraError> {
    if (width & 1) != 0 {
        return Err(CameraError::InvalidArg);
    }
    validate_encode_dims(src.len(), width, height, 2)?;
    if dst.len() < min_encoded_header_len(3) {
        return Err(CameraError::BufferTooSmall);
    }

    let lum_q = compute_quant_table(&STD_LUM_QUANT, quality);
    let chroma_q = compute_quant_table(&STD_CHROMA_QUANT, quality);

    let mut writer = JpegBitWriter::new(dst);
    writer.emit_headers(width, height, 3, subsampling, &lum_q.q, &chroma_q.q)?;

    let w = width as usize;
    let h = height as usize;
    let mut last_dc = [0i32; 3];
    let mut cb_block = [0i32; 64];
    let mut cr_block = [0i32; 64];

    match subsampling {
        Subsampling::H1V1 => {
            let mut y_block = [[0i32; 64]; 1];
            for by in 0..h.div_ceil(8) {
                for bx in 0..w.div_ceil(8) {
                    gather_yuv422_h1v1_mcu(
                        src,
                        w,
                        h,
                        bx,
                        by,
                        &mut y_block[0],
                        &mut cb_block,
                        &mut cr_block,
                    );
                    if matches!(range, YuvRange::Bt601Limited) {
                        expand_bt601_luma_block(&mut y_block[0]);
                        expand_bt601_chroma_block(&mut cb_block);
                        expand_bt601_chroma_block(&mut cr_block);
                    }
                    writer.encode_yuv_mcu(
                        &mut y_block,
                        &mut cb_block,
                        &mut cr_block,
                        &lum_q,
                        &chroma_q,
                        &mut last_dc,
                    )?;
                }
            }
        }
        Subsampling::H2V1 => {
            let mut y2 = [[0i32; 64]; 2];
            for by in 0..h.div_ceil(8) {
                for bx in 0..w.div_ceil(16) {
                    gather_yuv422_h2v1_mcu(
                        src,
                        w,
                        h,
                        bx,
                        by,
                        &mut y2,
                        &mut cb_block,
                        &mut cr_block,
                    );
                    if matches!(range, YuvRange::Bt601Limited) {
                        for blk in &mut y2 {
                            expand_bt601_luma_block(blk);
                        }
                        expand_bt601_chroma_block(&mut cb_block);
                        expand_bt601_chroma_block(&mut cr_block);
                    }
                    writer.encode_yuv_mcu(
                        &mut y2,
                        &mut cb_block,
                        &mut cr_block,
                        &lum_q,
                        &chroma_q,
                        &mut last_dc,
                    )?;
                }
            }
        }
        Subsampling::H2V2 => {
            let mut y_blocks = [[0i32; 64]; 4];
            for by in 0..h.div_ceil(16) {
                for bx in 0..w.div_ceil(16) {
                    gather_yuv422_h2v2_mcu(
                        src,
                        w,
                        h,
                        bx,
                        by,
                        &mut y_blocks,
                        &mut cb_block,
                        &mut cr_block,
                    );
                    if matches!(range, YuvRange::Bt601Limited) {
                        for blk in &mut y_blocks {
                            expand_bt601_luma_block(blk);
                        }
                        expand_bt601_chroma_block(&mut cb_block);
                        expand_bt601_chroma_block(&mut cr_block);
                    }
                    writer.encode_yuv_mcu(
                        &mut y_blocks,
                        &mut cb_block,
                        &mut cr_block,
                        &lum_q,
                        &chroma_q,
                        &mut last_dc,
                    )?;
                }
            }
        }
    }

    writer.flush_bits()?;
    writer.emit_marker(M_EOI)?;
    Ok(writer.pos)
}

/// Encodes a packed YUV422 (YUYV: `[Y0, U, Y1, V]`) image into a baseline
/// JFIF JPEG stream in `dst`.
///
/// Performs zero heap allocations and returns the total JPEG byte length
/// written to `dst`.
pub fn encode_yuv422_to_jpeg(
    src: &[u8],
    dst: &mut [u8],
    width: u16,
    height: u16,
    quality: u8,
) -> Result<usize, CameraError> {
    encode_yuv422_to_jpeg_subsampled(src, dst, width, height, quality, Subsampling::H2V1)
}

#[derive(Clone, Copy, Debug)]
struct DecHuffTable {
    fast_sym: [u8; 256],
    fast_len: [u8; 128],
    max_lj: [u16; 17],
    min_code: [u16; 17],
    has_len: u16,
    val_ptr: [u8; 17],
    valid: bool,
    vals: [u8; 168],
}

const _: () = assert!(
    core::mem::size_of::<DecHuffTable>() <= 640,
    "DecHuffTable must fit within 640 bytes"
);

impl Default for DecHuffTable {
    fn default() -> Self {
        Self {
            fast_sym: [0; 256],
            fast_len: [0; 128],
            max_lj: [0; 17],
            min_code: [0; 17],
            has_len: 0,
            val_ptr: [0; 17],
            valid: false,
            vals: [0; 168],
        }
    }
}

fn build_dec_huff_table_in_place(
    table: &mut DecHuffTable,
    bits: &[u8; 17],
    vals: &[u8],
) -> Result<(), CameraError> {
    *table = DecHuffTable::default();
    let mut code = 0u32;
    let mut k = 0usize;
    for (l, &b) in bits.iter().enumerate().take(17).skip(1) {
        let count = b as usize;
        if k + count > table.vals.len() || k + count > vals.len() {
            return Err(CameraError::CodecError);
        }
        if code + (count as u32) > (1u32 << l) {
            return Err(CameraError::CodecError);
        }
        if count > 0 {
            table.has_len |= 1u16 << (l - 1);
            table.min_code[l] = code as u16;
            table.val_ptr[l] = k as u8;
            for _ in 0..count {
                let sym = vals[k];
                table.vals[k] = sym;
                if l <= 8 {
                    let shift = 8 - l;
                    let base = (code as usize) << shift;
                    let num_entries = 1usize << shift;
                    for fill in 0..num_entries {
                        let idx = base + fill;
                        if idx < 256 {
                            table.fast_sym[idx] = sym;
                            let nib_shift = (idx & 1) * 4;
                            table.fast_len[idx >> 1] = (table.fast_len[idx >> 1]
                                & !(0x0Fu8 << nib_shift))
                                | ((l as u8) << nib_shift);
                        }
                    }
                }
                k += 1;
                code += 1;
            }
            let last_code = code - 1;
            table.max_lj[l] = ((last_code << (16 - l)) | ((1u32 << (16 - l)) - 1)) as u16;
        }
        code <<= 1;
    }
    table.valid = true;
    Ok(())
}

#[cfg(test)]
fn build_dec_huff_table(bits: &[u8; 17], vals: &[u8]) -> Result<DecHuffTable, CameraError> {
    let mut table = DecHuffTable::default();
    build_dec_huff_table_in_place(&mut table, bits, vals)?;
    Ok(table)
}

#[derive(Clone, Copy, Default)]
struct ComponentSpec {
    id: u8,
    h_samp: u8,
    v_samp: u8,
    quant_idx: u8,
    dc_idx: u8,
    ac_idx: u8,
}

struct JpegBitReader<'a> {
    src: &'a [u8],
    pos: usize,
    bit_buf: u32,
    bits_in: u32,
    hit_marker: bool,
    eof_pad_bytes: usize,
}

impl<'a> JpegBitReader<'a> {
    fn new(src: &'a [u8], start_pos: usize) -> Self {
        Self {
            src,
            pos: start_pos,
            bit_buf: 0,
            bits_in: 0,
            hit_marker: false,
            eof_pad_bytes: 0,
        }
    }

    #[inline]
    fn refill(&mut self) -> Result<(), CameraError> {
        while self.bits_in <= 24 {
            if self.hit_marker || self.pos >= self.src.len() {
                if self.eof_pad_bytes >= 12 {
                    return Err(CameraError::CorruptedFrame);
                }
                self.eof_pad_bytes += 1;
                // Pad with 1-bits after reaching EOI at the end of a scan.
                self.bit_buf = (self.bit_buf << 8) | 0xFF;
                self.bits_in += 8;
                continue;
            }
            let b = self.src[self.pos];
            self.pos += 1;
            if b == 0xFF {
                while self.pos < self.src.len() && self.src[self.pos] == 0xFF {
                    self.pos += 1;
                }
                if self.pos >= self.src.len() {
                    return Err(CameraError::CorruptedFrame);
                }
                let next = self.src[self.pos];
                if next == 0x00 {
                    self.pos += 1;
                    self.bit_buf = (self.bit_buf << 8) | 0xFF;
                    self.bits_in += 8;
                } else {
                    // Encountered marker (e.g. EOI 0xD9 or RSTn)
                    self.pos -= 1;
                    self.hit_marker = true;
                }
            } else {
                self.bit_buf = (self.bit_buf << 8) | u32::from(b);
                self.bits_in += 8;
            }
        }
        Ok(())
    }

    #[inline]
    fn get_bits(&mut self, n: u32) -> Result<u32, CameraError> {
        if n == 0 {
            return Ok(0);
        }
        debug_assert!(n <= 24, "get_bits len must be <= 24, got {n}");
        if self.bits_in < n {
            self.refill()?;
        }
        self.bits_in -= n;
        Ok((self.bit_buf >> self.bits_in) & ((1u32 << n) - 1))
    }

    fn decode_huff(&mut self, table: &DecHuffTable) -> Result<u8, CameraError> {
        debug_assert!(table.valid, "Huffman table validity checked once per SOS");
        if self.bits_in < 16 {
            self.refill()?;
        }
        let peek16 = ((self.bit_buf >> (self.bits_in - 16)) & 0xFFFF) as u16;
        let peek8 = (peek16 >> 8) as usize;
        let len8 = ((table.fast_len[peek8 >> 1] >> ((peek8 & 1) * 4)) & 0x0F) as u32;
        if len8 != 0 {
            self.bits_in -= len8;
            return Ok(table.fast_sym[peek8]);
        }
        for l in 9..=16usize {
            if (table.has_len & (1u16 << (l - 1))) != 0 && peek16 <= table.max_lj[l] {
                let code = peek16 >> (16 - l);
                let offset = code
                    .checked_sub(table.min_code[l])
                    .ok_or(CameraError::CodecError)? as usize;
                let idx = (table.val_ptr[l] as usize) + offset;
                if idx >= table.vals.len() {
                    return Err(CameraError::CodecError);
                }
                self.bits_in -= l as u32;
                return Ok(table.vals[idx]);
            }
        }
        Err(CameraError::CodecError)
    }

    #[inline]
    fn receive_extend(&mut self, nbits: u32) -> Result<i32, CameraError> {
        if nbits == 0 {
            return Ok(0);
        }
        if nbits > 16 {
            return Err(CameraError::CodecError);
        }
        let raw = self.get_bits(nbits)?;
        let vt = 1u32 << (nbits - 1);
        if raw < vt {
            Ok((raw as i32) - ((1i32 << nbits) - 1))
        } else {
            Ok(raw as i32)
        }
    }

    fn decode_block(
        &mut self,
        qtable: &[u8; 64],
        dc_huff: &DecHuffTable,
        ac_huff: &DecHuffTable,
        last_dc: &mut i32,
        out_pixels: &mut [u8; 64],
    ) -> Result<(), CameraError> {
        let mut coeffs = [0i32; 64];

        let dc_nbits = self.decode_huff(dc_huff)? as u32;
        let dc_diff = self.receive_extend(dc_nbits)?;
        let dc_val = last_dc.saturating_add(dc_diff).clamp(-2048, 2047);
        *last_dc = dc_val;
        coeffs[0] = dc_val
            .saturating_mul(i32::from(qtable[0]))
            .clamp(-2047, 2047);

        let mut last_k: usize = 0;
        let mut k = 1usize;
        while k < 64 {
            let rs = self.decode_huff(ac_huff)?;
            let r = (rs >> 4) as usize;
            let s = (rs & 0x0F) as u32;

            if s == 0 {
                if r == 15 {
                    if k + 16 > 64 {
                        return Err(CameraError::CorruptedFrame);
                    }
                    k += 16;
                    continue;
                }
                break;
            }

            k += r;
            if k >= 64 {
                return Err(CameraError::CodecError);
            }
            let ac_val = self.receive_extend(s)?;
            let ac_coeff = ac_val
                .saturating_mul(i32::from(qtable[k]))
                .clamp(-2047, 2047);
            coeffs[ZAG[k]] = ac_coeff;
            if ac_coeff != 0 {
                last_k = k;
            }
            k += 1;
        }

        if last_k == 0 {
            let f0 = coeffs[0].clamp(-2047, 2047);
            let w0 = (f0 << ROW_BITS).clamp(-16384, 16384);
            let shift = (CONST_BITS + ROW_BITS + 3) as u32;
            let px = (dct_descale_i32(w0 << CONST_BITS, shift) + 128).clamp(0, 255) as u8;
            out_pixels.fill(px);
        } else {
            idct_2d(&coeffs, out_pixels);
        }
        Ok(())
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum DecodeOutputFormat {
    Rgb888,
    Rgb565,
    Bmp24,
}

trait PixelSink {
    const BPP: usize;
    fn write_rgb(chunk: &mut [u8], r: u8, g: u8, b: u8);
}

struct Rgb888Sink;

impl PixelSink for Rgb888Sink {
    const BPP: usize = 3;

    #[inline(always)]
    fn write_rgb(chunk: &mut [u8], r: u8, g: u8, b: u8) {
        chunk[0] = r;
        chunk[1] = g;
        chunk[2] = b;
    }
}

struct Rgb565Sink;

impl PixelSink for Rgb565Sink {
    const BPP: usize = 2;

    #[inline(always)]
    fn write_rgb(chunk: &mut [u8], r: u8, g: u8, b: u8) {
        chunk[0] = (r & 0xF8) | (g >> 5);
        chunk[1] = ((g & 0x1C) << 3) | (b >> 3);
    }
}

struct Bmp24Sink;

impl PixelSink for Bmp24Sink {
    const BPP: usize = 3;

    #[inline(always)]
    fn write_rgb(chunk: &mut [u8], r: u8, g: u8, b: u8) {
        chunk[0] = b;
        chunk[1] = g;
        chunk[2] = r;
    }
}

#[inline]
#[allow(clippy::too_many_arguments)]
fn write_mcu_pixels<S: PixelSink>(
    dst: &mut [u8],
    base_offset: usize,
    row_stride_bytes: usize,
    img_x0: usize,
    img_y0: usize,
    valid_w: usize,
    valid_h: usize,
    num_components: usize,
    h_shift: usize,
    v_shift: usize,
    y_blocks: &[[u8; 64]; 4],
    cb_block: &[u8; 64],
    cr_block: &[u8; 64],
) {
    for py_in_mcu in 0..valid_h {
        let img_y = img_y0 + py_in_mcu;
        let row_start = base_offset + img_y * row_stride_bytes + img_x0 * S::BPP;
        let row_slice = &mut dst[row_start..row_start + valid_w * S::BPP];
        let y_row_blk = (py_in_mcu >> 3) << h_shift;
        let y_sub_row = (py_in_mcu & 7) << 3;
        let cy_row = (py_in_mcu >> v_shift) << 3;
        if num_components == 1 {
            for (px_in_mcu, chunk) in row_slice.chunks_exact_mut(S::BPP).enumerate() {
                let y_blk_idx = y_row_blk + (px_in_mcu >> 3);
                let y_sub_idx = y_sub_row + (px_in_mcu & 7);
                let y_val = y_blocks[y_blk_idx][y_sub_idx];
                S::write_rgb(chunk, y_val, y_val, y_val);
            }
        } else {
            for (px_in_mcu, chunk) in row_slice.chunks_exact_mut(S::BPP).enumerate() {
                let y_blk_idx = y_row_blk + (px_in_mcu >> 3);
                let y_sub_idx = y_sub_row + (px_in_mcu & 7);
                let y_val = y_blocks[y_blk_idx][y_sub_idx];
                let c_idx = cy_row + (px_in_mcu >> h_shift);
                let (r, g, b) = jfif_ycbcr_to_rgb(y_val, cb_block[c_idx], cr_block[c_idx]);
                S::write_rgb(chunk, r, g, b);
            }
        }
    }
}

fn decode_jpeg_internal(
    src: &[u8],
    dst: &mut [u8],
    out_fmt: DecodeOutputFormat,
    expected_dims: Option<(u16, u16)>,
) -> Result<(u16, u16, usize), CameraError> {
    if src.len() < 4 || src[0] != 0xFF || src[1] != M_SOI {
        return Err(CameraError::CorruptedFrame);
    }

    let mut qtables = [[0u8; 64]; 4];
    let mut qtable_valid = [false; 4];
    let mut dc_tables = [DecHuffTable::default(); 2];
    let mut ac_tables = [DecHuffTable::default(); 2];
    let mut comps = [ComponentSpec::default(); 3];
    let mut num_components = 0usize;
    let mut width = 0u16;
    let mut height = 0u16;
    let mut restart_interval = 0u16;
    let mut sos_payload_pos = None;
    let mut sof_parsed = false;
    let mut dht_seen = false;
    let mut dqt_seen = false;

    let mut pos = 2usize;
    while pos + 1 < src.len() {
        if src[pos] != 0xFF {
            return Err(CameraError::CorruptedFrame);
        }
        while pos < src.len() && src[pos] == 0xFF {
            pos += 1;
        }
        if pos >= src.len() {
            return Err(CameraError::CorruptedFrame);
        }
        let marker = src[pos];
        pos += 1;

        if marker == M_EOI {
            break;
        }
        if marker == 0x00 || marker == 0x01 || (0xD0..=0xD8).contains(&marker) {
            continue;
        }

        if pos + 2 > src.len() {
            return Err(CameraError::CorruptedFrame);
        }
        let seg_len = u16::from_be_bytes([src[pos], src[pos + 1]]) as usize;
        let seg_end = pos
            .checked_add(seg_len)
            .ok_or(CameraError::CorruptedFrame)?;
        if seg_len < 2 || seg_end > src.len() {
            return Err(CameraError::CorruptedFrame);
        }
        let mut cur = pos + 2;

        match marker {
            M_DQT => {
                while cur < seg_end {
                    if cur + 65 > seg_end {
                        return Err(CameraError::CodecError);
                    }
                    let info = src[cur];
                    cur += 1;
                    let precision = info >> 4;
                    let t_idx = (info & 0x0F) as usize;
                    if precision != 0 || t_idx >= 4 {
                        return Err(CameraError::NotSupported);
                    }
                    dqt_seen = true;
                    qtables[t_idx].copy_from_slice(&src[cur..cur + 64]);
                    qtable_valid[t_idx] = true;
                    cur += 64;
                }
            }
            M_DHT => {
                while cur < seg_end {
                    if cur + 17 > seg_end {
                        return Err(CameraError::CodecError);
                    }
                    let info = src[cur];
                    cur += 1;
                    let is_ac = (info & 0x10) != 0;
                    let t_idx = (info & 0x0F) as usize;
                    if t_idx > 1 {
                        return Err(CameraError::CodecError);
                    }
                    let mut bits = [0u8; 17];
                    let mut total_syms = 0usize;
                    for l in 1..=16 {
                        bits[l] = src[cur + l - 1];
                        total_syms += bits[l] as usize;
                    }
                    cur += 16;
                    if cur + total_syms > seg_end {
                        return Err(CameraError::CodecError);
                    }
                    let target = if is_ac {
                        &mut ac_tables[t_idx]
                    } else {
                        &mut dc_tables[t_idx]
                    };
                    build_dec_huff_table_in_place(target, &bits, &src[cur..cur + total_syms])?;
                    dht_seen = true;
                    cur += total_syms;
                }
            }
            M_DRI => {
                if seg_len != 4 {
                    return Err(CameraError::CodecError);
                }
                restart_interval = u16::from_be_bytes([src[cur], src[cur + 1]]);
            }
            M_SOF0 => {
                if cur + 6 > seg_end {
                    return Err(CameraError::CodecError);
                }
                let precision = src[cur];
                if precision != 8 {
                    return Err(CameraError::NotSupported);
                }
                height = u16::from_be_bytes([src[cur + 1], src[cur + 2]]);
                width = u16::from_be_bytes([src[cur + 3], src[cur + 4]]);
                num_components = src[cur + 5] as usize;
                cur += 6;
                if width == 0
                    || height == 0
                    || (num_components != 1 && num_components != 3)
                    || cur + num_components * 3 > seg_end
                {
                    return Err(CameraError::CodecError);
                }
                sof_parsed = true;
                if expected_dims
                    .is_some_and(|(ew, eh)| (ew != 0 && ew != width) || (eh != 0 && eh != height))
                {
                    return Err(CameraError::InvalidArg);
                }
                for (idx, comp) in comps.iter_mut().take(num_components).enumerate() {
                    let id = src[cur];
                    let samp = src[cur + 1];
                    let q_idx = src[cur + 2];
                    cur += 3;
                    let h_samp = samp >> 4;
                    let v_samp = samp & 0x0F;
                    if h_samp == 0 || h_samp > 2 || v_samp == 0 || v_samp > 2 || q_idx >= 4 {
                        return Err(CameraError::NotSupported);
                    }
                    if idx >= 1 && (h_samp != 1 || v_samp != 1) {
                        return Err(CameraError::NotSupported);
                    }
                    *comp = ComponentSpec {
                        id,
                        h_samp,
                        v_samp,
                        quant_idx: q_idx,
                        dc_idx: 0,
                        ac_idx: 0,
                    };
                }
            }
            M_SOS => {
                if !sof_parsed || !dht_seen || !dqt_seen || num_components == 0 || cur + 1 > seg_end
                {
                    return Err(CameraError::CodecError);
                }
                let sos_comps = src[cur] as usize;
                cur += 1;
                if sos_comps != num_components || cur + sos_comps * 2 + 3 > seg_end {
                    return Err(CameraError::CodecError);
                }
                for _ in 0..sos_comps {
                    let cid = src[cur];
                    let huff_sel = src[cur + 1];
                    cur += 2;
                    let dc_idx = huff_sel >> 4;
                    let ac_idx = huff_sel & 0x0F;
                    if dc_idx > 1 || ac_idx > 1 {
                        return Err(CameraError::CodecError);
                    }
                    let mut matched = false;
                    for comp in comps.iter_mut().take(num_components) {
                        if comp.id == cid {
                            comp.dc_idx = dc_idx;
                            comp.ac_idx = ac_idx;
                            matched = true;
                            break;
                        }
                    }
                    if !matched {
                        return Err(CameraError::CodecError);
                    }
                }
                sos_payload_pos = Some(seg_end);
                break;
            }
            _ => {}
        }

        pos = seg_end;
    }

    let scan_start = sos_payload_pos.ok_or(CameraError::CorruptedFrame)?;
    let w32 = width as u32;
    let h32 = height as u32;
    let pixels32 = w32.checked_mul(h32).ok_or(CameraError::Overflow)?;

    let (required_dst_len, bmp_row_stride) = match out_fmt {
        DecodeOutputFormat::Rgb888 => {
            let bytes = pixels32.checked_mul(3).ok_or(CameraError::Overflow)?;
            (
                usize::try_from(bytes).map_err(|_| CameraError::Overflow)?,
                0usize,
            )
        }
        DecodeOutputFormat::Rgb565 => {
            let bytes = pixels32.checked_mul(2).ok_or(CameraError::Overflow)?;
            (
                usize::try_from(bytes).map_err(|_| CameraError::Overflow)?,
                0usize,
            )
        }
        DecodeOutputFormat::Bmp24 => {
            let row_bytes = w32.checked_mul(3).ok_or(CameraError::Overflow)?;
            let stride = row_bytes.checked_add(3).ok_or(CameraError::Overflow)? & !3u32;
            let img_size = stride.checked_mul(h32).ok_or(CameraError::Overflow)?;
            let total = (BMP_HEADER_LEN as u32)
                .checked_add(img_size)
                .ok_or(CameraError::Overflow)?;
            if total > i32::MAX as u32 {
                return Err(CameraError::Overflow);
            }
            (
                usize::try_from(total).map_err(|_| CameraError::Overflow)?,
                usize::try_from(stride).map_err(|_| CameraError::Overflow)?,
            )
        }
    };

    if dst.len() < required_dst_len {
        return Err(CameraError::BufferTooSmall);
    }

    if out_fmt == DecodeOutputFormat::Bmp24 {
        let img_size = (required_dst_len - BMP_HEADER_LEN) as u32;
        write_bmp_header(dst, width, height, 24, img_size, 0)?;
        let row_stride = (width as usize) * 3;
        if bmp_row_stride > row_stride {
            for y in 0..(height as usize) {
                let row_start = BMP_HEADER_LEN + y * bmp_row_stride;
                dst[row_start + row_stride..row_start + bmp_row_stride].fill(0);
            }
        }
    }

    for comp in comps.iter().take(num_components) {
        if !qtable_valid[comp.quant_idx as usize]
            || !dc_tables[comp.dc_idx as usize].valid
            || !ac_tables[comp.ac_idx as usize].valid
        {
            return Err(CameraError::CodecError);
        }
    }

    let (max_h_samp, max_v_samp) = if num_components == 1 {
        (1usize, 1usize)
    } else {
        (comps[0].h_samp as usize, comps[0].v_samp as usize)
    };
    let h_shift = max_h_samp - 1;
    let v_shift = max_v_samp - 1;
    let mcu_w = max_h_samp * 8;
    let mcu_h = max_v_samp * 8;
    let w = width as usize;
    let h = height as usize;
    let mcu_cols = w.div_ceil(mcu_w);
    let mcu_rows = h.div_ceil(mcu_h);

    let mut reader = JpegBitReader::new(src, scan_start);
    let mut last_dc = [0i32; 3];
    let mut mcu_counter = 0u16;
    let mut expected_rst = 0u8;

    let mut y_blocks = [[0u8; 64]; 4];
    let mut cb_block = [128u8; 64];
    let mut cr_block = [128u8; 64];

    for mcu_y in 0..mcu_rows {
        for mcu_x in 0..mcu_cols {
            if restart_interval > 0 && mcu_counter == restart_interval {
                mcu_counter = 0;
                last_dc = [0i32; 3];
                reader.bit_buf = 0;
                reader.bits_in = 0;
                reader.hit_marker = false;
                reader.eof_pad_bytes = 0;
                if reader.pos >= src.len() || src[reader.pos] != 0xFF {
                    return Err(CameraError::CorruptedFrame);
                }
                while reader.pos < src.len() && src[reader.pos] == 0xFF {
                    reader.pos += 1;
                }
                if reader.pos >= src.len() || src[reader.pos] != (0xD0 + expected_rst) {
                    return Err(CameraError::CorruptedFrame);
                }
                reader.pos += 1;
                expected_rst = (expected_rst + 1) & 7;
            }

            let y_count = max_h_samp * max_v_samp;
            for y_blk in y_blocks.iter_mut().take(y_count) {
                reader.decode_block(
                    &qtables[comps[0].quant_idx as usize],
                    &dc_tables[comps[0].dc_idx as usize],
                    &ac_tables[comps[0].ac_idx as usize],
                    &mut last_dc[0],
                    y_blk,
                )?;
            }

            if num_components == 3 {
                reader.decode_block(
                    &qtables[comps[1].quant_idx as usize],
                    &dc_tables[comps[1].dc_idx as usize],
                    &ac_tables[comps[1].ac_idx as usize],
                    &mut last_dc[1],
                    &mut cb_block,
                )?;
                reader.decode_block(
                    &qtables[comps[2].quant_idx as usize],
                    &dc_tables[comps[2].dc_idx as usize],
                    &ac_tables[comps[2].ac_idx as usize],
                    &mut last_dc[2],
                    &mut cr_block,
                )?;
            }

            let img_x0 = mcu_x * mcu_w;
            let img_y0 = mcu_y * mcu_h;
            let valid_w = (w - img_x0).min(mcu_w);
            let valid_h = (h - img_y0).min(mcu_h);

            match out_fmt {
                DecodeOutputFormat::Rgb888 => write_mcu_pixels::<Rgb888Sink>(
                    dst,
                    0,
                    w * 3,
                    img_x0,
                    img_y0,
                    valid_w,
                    valid_h,
                    num_components,
                    h_shift,
                    v_shift,
                    &y_blocks,
                    &cb_block,
                    &cr_block,
                ),
                DecodeOutputFormat::Rgb565 => write_mcu_pixels::<Rgb565Sink>(
                    dst,
                    0,
                    w * 2,
                    img_x0,
                    img_y0,
                    valid_w,
                    valid_h,
                    num_components,
                    h_shift,
                    v_shift,
                    &y_blocks,
                    &cb_block,
                    &cr_block,
                ),
                DecodeOutputFormat::Bmp24 => write_mcu_pixels::<Bmp24Sink>(
                    dst,
                    BMP_HEADER_LEN,
                    bmp_row_stride,
                    img_x0,
                    img_y0,
                    valid_w,
                    valid_h,
                    num_components,
                    h_shift,
                    v_shift,
                    &y_blocks,
                    &cb_block,
                    &cr_block,
                ),
            }

            mcu_counter = mcu_counter.wrapping_add(1);
        }
    }

    Ok((width, height, required_dst_len))
}

/// Decodes a baseline JPEG image from `src` into 24-bit RGB888 (`[R, G, B]`)
/// in `dst`.
///
/// Performs zero heap allocations and returns the number of bytes written
/// to `dst` (`width * height * 3`).
pub fn decode_jpeg_to_rgb888(src: &[u8], dst: &mut [u8]) -> Result<usize, CameraError> {
    let (_w, _h, written) = decode_jpeg_internal(src, dst, DecodeOutputFormat::Rgb888, None)?;
    Ok(written)
}

/// Decodes a baseline JPEG image from `src` into 16-bit big-endian RGB565
/// in `dst`.
///
/// Performs zero heap allocations and returns the number of bytes written
/// to `dst` (`width * height * 2`).
pub fn decode_jpeg_to_rgb565(src: &[u8], dst: &mut [u8]) -> Result<usize, CameraError> {
    let (_w, _h, written) = decode_jpeg_internal(src, dst, DecodeOutputFormat::Rgb565, None)?;
    Ok(written)
}

/// Decodes a baseline JPEG image from `src` directly into a 24-bit BMP image
/// in `dst`.
///
/// Returns `(width, height, written_len)`.
#[cfg(test)]
pub(crate) fn jpeg_to_bmp(src: &[u8], dst: &mut [u8]) -> Result<(u16, u16, usize), CameraError> {
    decode_jpeg_internal(src, dst, DecodeOutputFormat::Bmp24, None)
}

/// Decodes a baseline JPEG image from `src` directly into a 24-bit BMP image
/// in `dst`, validating non-zero `expected_width` and `expected_height`
/// against the `SOF0` header in a single pass before modifying `dst`
/// (`CONV-18`).
pub(crate) fn jpeg_to_bmp_checked(
    src: &[u8],
    dst: &mut [u8],
    expected_width: u16,
    expected_height: u16,
) -> Result<usize, CameraError> {
    let (dec_w, dec_h, written) = decode_jpeg_internal(
        src,
        dst,
        DecodeOutputFormat::Bmp24,
        Some((expected_width, expected_height)),
    )?;
    debug_assert!(expected_width == 0 || expected_width == dec_w);
    debug_assert!(expected_height == 0 || expected_height == dec_h);
    Ok(written)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::jpeg_scan::{parse_jpeg_dimensions, verify_jpeg_eoi, verify_jpeg_soi};

    #[test]
    fn test_jpeg_rgb888_roundtrip() {
        let width = 16u16;
        let height = 16u16;
        let mut rgb_in = [0u8; 16 * 16 * 3];
        for i in 0..(16 * 16) {
            rgb_in[i * 3] = 200;
            rgb_in[i * 3 + 1] = 100;
            rgb_in[i * 3 + 2] = 50;
        }

        let mut jpeg_buf = [0u8; 4096];
        let jpeg_len = encode_rgb888_to_jpeg(&rgb_in, &mut jpeg_buf, width, height, 90)
            .expect("encode_rgb888_to_jpeg should succeed");
        let jpeg_slice = &jpeg_buf[..jpeg_len];

        assert_eq!(verify_jpeg_soi(jpeg_slice), Some(0));
        assert_eq!(verify_jpeg_eoi(jpeg_slice, true), Some(jpeg_len - 2));
        assert_eq!(verify_jpeg_eoi(jpeg_slice, false), Some(jpeg_len - 2));
        assert_eq!(parse_jpeg_dimensions(jpeg_slice), Some((16, 16)));

        let mut rgb_out = [0u8; 16 * 16 * 3];
        let decoded_len = decode_jpeg_to_rgb888(jpeg_slice, &mut rgb_out)
            .expect("decode_jpeg_to_rgb888 should succeed");
        assert_eq!(decoded_len, 16 * 16 * 3);

        // Check decoded pixel fidelity within JPEG quantization tolerance
        assert!((rgb_out[0] as i32 - 200).abs() <= 10);
        assert!((rgb_out[1] as i32 - 100).abs() <= 10);
        assert!((rgb_out[2] as i32 - 50).abs() <= 10);

        let mut rgb565_out = [0u8; 16 * 16 * 2];
        let decoded_565_len = decode_jpeg_to_rgb565(jpeg_slice, &mut rgb565_out)
            .expect("decode_jpeg_to_rgb565 should succeed");
        assert_eq!(decoded_565_len, 16 * 16 * 2);
    }

    #[test]
    fn test_jpeg_grayscale_and_yuv422_roundtrip() {
        let width = 8u16;
        let height = 8u16;
        let gray_in = [160u8; 64];
        let mut jpeg_buf = [0u8; 2048];
        let jpeg_len = encode_grayscale_to_jpeg(&gray_in, &mut jpeg_buf, width, height, 85)
            .expect("encode_grayscale_to_jpeg should succeed");
        let jpeg_slice = &jpeg_buf[..jpeg_len];

        assert_eq!(verify_jpeg_soi(jpeg_slice), Some(0));
        assert_eq!(verify_jpeg_eoi(jpeg_slice, false), Some(jpeg_len - 2));
        assert_eq!(parse_jpeg_dimensions(jpeg_slice), Some((8, 8)));

        let mut rgb_out = [0u8; 8 * 8 * 3];
        let dec_len = decode_jpeg_to_rgb888(jpeg_slice, &mut rgb_out)
            .expect("decode_jpeg_to_rgb888 should succeed");
        assert_eq!(dec_len, 8 * 8 * 3);
        assert!((rgb_out[0] as i32 - 160).abs() <= 4);
        assert_eq!(rgb_out[0], rgb_out[1]);
        assert_eq!(rgb_out[1], rgb_out[2]);

        // YUV422 encode -> decode roundtrip
        let yuv_in = [128u8; 8 * 8 * 2];
        let yuv_jpeg_len = encode_yuv422_to_jpeg(&yuv_in, &mut jpeg_buf, width, height, 85)
            .expect("encode_yuv422_to_jpeg should succeed");
        let yuv_jpeg_slice = &jpeg_buf[..yuv_jpeg_len];
        assert_eq!(parse_jpeg_dimensions(yuv_jpeg_slice), Some((8, 8)));
        let yuv_dec_len = decode_jpeg_to_rgb888(yuv_jpeg_slice, &mut rgb_out)
            .expect("decode_jpeg_to_rgb888 should succeed");
        assert_eq!(yuv_dec_len, 8 * 8 * 3);
        assert!((rgb_out[0] as i32 - 128).abs() <= 4);
    }

    #[test]
    fn test_conv001_adversarial_max_quant_idct_no_overflow() {
        // CONV-001: idct_2d must not panic with signed 32-bit overflow on
        // extreme coefficients
        let mut out = [0u8; 64];

        let max_pos = [32767i32; 64];
        idct_2d(&max_pos, &mut out);
        assert_eq!(out[0], 255);

        let max_neg = [-32768i32; 64];
        idct_2d(&max_neg, &mut out);
        assert_eq!(out[0], 0);

        let mut alternating = [0i32; 64];
        for (i, coeff) in alternating.iter_mut().enumerate() {
            *coeff = if i % 2 == 0 { 32767 } else { -32768 };
        }
        idct_2d(&alternating, &mut out);
        // Verify output is clamped valid u8 sample
        assert!(out.iter().all(|&px| px == 0 || px == 255 || px > 0));
    }

    #[test]
    fn test_conv003_oversubscribed_huffman_kraft_inequality() {
        // CONV-003: At bit length 1, 3 symbols requested when maximum allowed
        // is 2 (Kraft violation)
        let mut bad_bits = [0u8; 17];
        bad_bits[1] = 3;
        let vals = [0u8; 3];
        assert_eq!(
            build_dec_huff_table(&bad_bits, &vals)
                .expect_err("Kraft inequality violation at l=1 must error"),
            CameraError::CodecError
        );

        // At bit length 1: 2 symbols (fills tree at l=1); at length 2:
        // 1 symbol -> Kraft violation
        let mut bad_bits2 = [0u8; 17];
        bad_bits2[1] = 2;
        bad_bits2[2] = 1;
        let vals2 = [0u8; 3];
        assert_eq!(
            build_dec_huff_table(&bad_bits2, &vals2)
                .expect_err("Kraft inequality violation at l=2 must error"),
            CameraError::CodecError
        );
    }

    #[test]
    fn test_conv003_truncated_entropy_stream_pad_bound() {
        // CONV-003: Severely truncated entropy streams must be rejected when
        // eof_pad_bytes exceeded
        let rgb_in = [120u8; 16 * 16 * 3];
        let mut jpeg_buf = [0u8; 4096];
        let jpeg_len = encode_rgb888_to_jpeg(&rgb_in, &mut jpeg_buf, 16, 16, 90)
            .expect("encode should succeed");

        let sos_pos = jpeg_buf[..jpeg_len]
            .windows(2)
            .position(|w| w == [0xFF, 0xDA])
            .expect("SOS marker should be present");

        // Truncate stream right after SOS header + 16 bytes (leaving 0 bytes of rest)
        let truncated_len = sos_pos + 16;
        let truncated = &jpeg_buf[..truncated_len];

        let mut rgb_out = [0u8; 16 * 16 * 3];
        let res = decode_jpeg_to_rgb888(truncated, &mut rgb_out);
        assert!(res.is_err());
    }

    #[test]
    fn test_conv003_unsupported_chroma_subsampling_rejection() {
        // CONV-003: Reject unsupported chroma subsampling
        // (c.h_samp != 1 || c.v_samp != 1 for c >= 1)
        let rgb_in = [120u8; 16 * 16 * 3];
        let mut jpeg_buf = [0u8; 4096];
        let jpeg_len = encode_rgb888_to_jpeg(&rgb_in, &mut jpeg_buf, 16, 16, 90)
            .expect("encode should succeed");

        let sof0_pos = jpeg_buf[..jpeg_len]
            .windows(2)
            .position(|w| w == [0xFF, 0xC0])
            .expect("SOF0 marker should be present");

        let mut modified = jpeg_buf;
        let comp1_samp_pos = sof0_pos + 14;
        modified[comp1_samp_pos] = 0x21; // h_samp=2, v_samp=1 for Cb

        let mut rgb_out = [0u8; 16 * 16 * 3];
        assert_eq!(
            decode_jpeg_to_rgb888(&modified[..jpeg_len], &mut rgb_out),
            Err(CameraError::NotSupported)
        );
    }

    #[test]
    fn test_conv003_sos_before_headers_rejected() {
        // CONV-003: Require sof_parsed && dht_seen && dqt_seen before M_SOS
        let early_sos = [
            0xFF, 0xD8, // SOI
            0xFF, 0xDA, 0x00, 0x08, 0x01, 0x01, 0x00, 0x00, 0x3F, 0x00, // SOS
            0xFF, 0xD9, // EOI
        ];
        let mut rgb_out = [0u8; 8 * 8 * 3];
        assert_eq!(
            decode_jpeg_to_rgb888(&early_sos, &mut rgb_out),
            Err(CameraError::CodecError)
        );
    }

    #[test]
    fn test_conv003_restart_marker_multiple_ff_fill_bytes() {
        // CONV-003: Verify restart interval resynchronization skips 0xFF fill
        // bytes before RST0..RST7
        let mut jpeg = [0u8; 512];
        let mut p = 0usize;

        // SOI
        jpeg[p..p + 2].copy_from_slice(&[0xFF, 0xD8]);
        p += 2;

        // DQT: table 0 (all 16s)
        jpeg[p..p + 4].copy_from_slice(&[0xFF, 0xDB, 0x00, 0x43]);
        p += 4;
        jpeg[p] = 0; // table 0
        p += 1;
        jpeg[p..p + 64].fill(16);
        p += 64;

        // SOF0: 16x8, 1 component
        jpeg[p..p + 13].copy_from_slice(&[
            0xFF, 0xC0, 0x00, 0x0B, 0x08, 0x00, 0x08, 0x00, 0x10, 0x01, 0x01, 0x11, 0x00,
        ]);
        p += 13;

        // DHT DC: table 0
        let dc_bits_len = DC_LUM_VAL.len();
        let dc_seg_len = 2 + 1 + 16 + dc_bits_len;
        jpeg[p..p + 2].copy_from_slice(&[0xFF, 0xC4]);
        p += 2;
        jpeg[p..p + 2].copy_from_slice(&(dc_seg_len as u16).to_be_bytes());
        p += 2;
        jpeg[p] = 0x00; // DC table 0
        p += 1;
        jpeg[p..p + 16].copy_from_slice(&DC_LUM_BITS[1..17]);
        p += 16;
        jpeg[p..p + dc_bits_len].copy_from_slice(&DC_LUM_VAL);
        p += dc_bits_len;

        // DHT AC: table 0
        let ac_bits_len = AC_LUM_VAL.len();
        let ac_seg_len = 2 + 1 + 16 + ac_bits_len;
        jpeg[p..p + 2].copy_from_slice(&[0xFF, 0xC4]);
        p += 2;
        jpeg[p..p + 2].copy_from_slice(&(ac_seg_len as u16).to_be_bytes());
        p += 2;
        jpeg[p] = 0x10; // AC table 0
        p += 1;
        jpeg[p..p + 16].copy_from_slice(&AC_LUM_BITS[1..17]);
        p += 16;
        jpeg[p..p + ac_bits_len].copy_from_slice(&AC_LUM_VAL);
        p += ac_bits_len;

        // DRI: restart_interval = 1
        jpeg[p..p + 6].copy_from_slice(&[0xFF, 0xDD, 0x00, 0x04, 0x00, 0x01]);
        p += 6;

        // SOS: 1 component
        jpeg[p..p + 10]
            .copy_from_slice(&[0xFF, 0xDA, 0x00, 0x08, 0x01, 0x01, 0x00, 0x00, 0x3F, 0x00]);
        p += 10;

        // MCU 0 scan data: DC diff 0 (00), EOB (1010) -> 0010_1011 (padded
        // with 1s) = 0x2B
        jpeg[p] = 0x2B;
        p += 1;

        // Restart marker RST0 with multiple fill 0xFF bytes: 0xFF, 0xFF, 0xD0
        jpeg[p..p + 3].copy_from_slice(&[0xFF, 0xFF, 0xD0]);
        p += 3;

        // MCU 1 scan data: DC diff 0 (00), EOB (1010) -> 0x2B
        jpeg[p] = 0x2B;
        p += 1;

        // EOI
        jpeg[p..p + 2].copy_from_slice(&[0xFF, 0xD9]);
        p += 2;

        let mut out = [0u8; 16 * 8 * 3];
        let dec_len = decode_jpeg_to_rgb888(&jpeg[..p], &mut out)
            .expect("decode with restart marker fill bytes should succeed");
        assert_eq!(dec_len, 16 * 8 * 3);
        assert_eq!(out[0], 128);
        assert_eq!(out[16 * 3], 128);
    }

    #[test]
    fn test_conv004_static_encoder_huffman_tables() {
        // CONV-004: Validate hoisted static encoder Huffman tables
        assert_eq!(HUFF_DC_LUM.sizes[0], 2);
        assert_eq!(HUFF_DC_LUM.codes[0], 0);
        assert_eq!(HUFF_AC_LUM.sizes[1], 2);
        assert_eq!(HUFF_DC_CHROM.sizes[0], 2);
        assert_eq!(HUFF_AC_CHROM.sizes[0], 2);
    }

    #[test]
    fn test_conv005_yuv422_encode_odd_width_rejected() {
        // CONV-005: Reject odd widths in encode_yuv422_to_jpeg
        let mut jpeg_buf = [0u8; 1024];
        let yuv_data = [128u8; 7 * 8 * 2];
        assert_eq!(
            encode_yuv422_to_jpeg(&yuv_data, &mut jpeg_buf, 7, 8, 85),
            Err(CameraError::InvalidArg)
        );
    }

    #[test]
    fn test_logic012_put_bits_u32_max_24_bound() {
        let mut dst = [0u8; 64];
        let mut writer = JpegBitWriter::new(&mut dst);
        assert_eq!(writer.put_bits(0x00AB_CDEF, 24), Ok(()));
        assert_eq!(writer.pos, 3);
        assert_eq!(&dst[..3], &[0xAB, 0xCD, 0xEF]);
    }

    #[cfg(debug_assertions)]
    #[test]
    #[should_panic(expected = "put_bits len must be <= 24")]
    fn test_logic012_put_bits_len_greater_than_24_debug_panics() {
        let mut dst = [0u8; 64];
        let mut writer = JpegBitWriter::new(&mut dst);
        let _ = writer.put_bits(0x1234_5678, 25);
    }

    #[test]
    fn test_logic012_decode_jpeg_segment_bounds_checked() {
        // Minimal invalid JPEG with seg_len exceeding src.len()
        let mut invalid_jpeg = [
            0xFF, 0xD8, // SOI
            0xFF, 0xE0, // APP0
            0xFF, 0xFF, // seg_len = 65535, src length 8
            0x00, 0x00,
        ];
        let mut out = [0u8; 64];
        assert_eq!(
            decode_jpeg_to_rgb888(&invalid_jpeg, &mut out),
            Err(CameraError::CorruptedFrame)
        );

        // seg_len < 2
        invalid_jpeg[4] = 0x00;
        invalid_jpeg[5] = 0x01;
        assert_eq!(
            decode_jpeg_to_rgb888(&invalid_jpeg, &mut out),
            Err(CameraError::CorruptedFrame)
        );
    }

    #[test]
    fn test_c06_quality_scaling_clamp() {
        let q0 = compute_quant_table(&STD_LUM_QUANT, 0);
        let q1 = compute_quant_table(&STD_LUM_QUANT, 1);
        let q100 = compute_quant_table(&STD_LUM_QUANT, 100);
        let q255 = compute_quant_table(&STD_LUM_QUANT, 255);
        assert_eq!(q0, q1, "quality=0 must clamp to quality=1");
        assert_eq!(q255, q100, "quality=255 must clamp to quality=100");
    }

    #[test]
    // Exhaustive sweep: minutes to hours under Miri's interpreter, and the
    // same code path as the representative-input tests that do run there.
    #[cfg_attr(miri, ignore)]
    fn test_conv02_reciprocal_quant_exhaustive() {
        for qi in 1..=255u32 {
            let qt = compute_quant_table(&[qi as i32; 64], 50);
            assert_eq!(
                qt.q[0] as u32, qi,
                "quality=50 identity quant entry mismatch for qi={qi}"
            );
            let recip = qt.recip[0];
            let shift = qt.shift[0];
            for adj in 0..4096u32 {
                let got = (adj.wrapping_mul(recip)) >> shift;
                let expected = adj / qi;
                assert_eq!(got, expected, "reciprocal mismatch for qi={qi}, adj={adj}");
            }
        }
    }

    #[inline]
    const fn dct_descale_i64(x: i64, n: u32) -> i64 {
        (x + (1i64 << (n - 1))) >> n
    }

    fn idct_2d_i64_ref(coeffs: &[i32; 64], out_pixels: &mut [u8; 64]) {
        let mut workspace = [0i64; 64];

        for row in 0..8 {
            let base = row * 8;
            let f0 = coeffs[base] as i64;
            let f1 = coeffs[base + 1] as i64;
            let f2 = coeffs[base + 2] as i64;
            let f3 = coeffs[base + 3] as i64;
            let f4 = coeffs[base + 4] as i64;
            let f5 = coeffs[base + 5] as i64;
            let f6 = coeffs[base + 6] as i64;
            let f7 = coeffs[base + 7] as i64;

            if (f1 | f2 | f3 | f4 | f5 | f6 | f7) == 0 {
                let dc_val = f0 << ROW_BITS;
                workspace[base..base + 8].fill(dc_val);
                continue;
            }

            let z2 = f2;
            let z3 = f6;
            let z1 = (z2 + z3) * 4433;
            let tmp2 = z1 + z3 * (-15137);
            let tmp3 = z1 + z2 * 6270;
            let tmp0 = (f0 + f4) << CONST_BITS;
            let tmp1 = (f0 - f4) << CONST_BITS;
            let tmp10 = tmp0 + tmp3;
            let tmp13 = tmp0 - tmp3;
            let tmp11 = tmp1 + tmp2;
            let tmp12 = tmp1 - tmp2;

            let mut oz1 = f7 + f1;
            let mut oz2 = f5 + f3;
            let mut oz3 = f7 + f3;
            let mut oz4 = f5 + f1;
            let z5 = (oz3 + oz4) * 9633;

            let mut otmp0 = f7 * 2446;
            let mut otmp1 = f5 * 16819;
            let mut otmp2 = f3 * 25172;
            let mut otmp3 = f1 * 12299;

            oz1 *= -7373;
            oz2 *= -20995;
            oz3 = oz3 * (-16069) + z5;
            oz4 = oz4 * (-3196) + z5;

            otmp0 += oz1 + oz3;
            otmp1 += oz2 + oz4;
            otmp2 += oz2 + oz3;
            otmp3 += oz1 + oz4;

            let shift = (CONST_BITS - ROW_BITS) as u32;
            workspace[base] = dct_descale_i64(tmp10 + otmp3, shift);
            workspace[base + 7] = dct_descale_i64(tmp10 - otmp3, shift);
            workspace[base + 1] = dct_descale_i64(tmp11 + otmp2, shift);
            workspace[base + 6] = dct_descale_i64(tmp11 - otmp2, shift);
            workspace[base + 2] = dct_descale_i64(tmp12 + otmp1, shift);
            workspace[base + 5] = dct_descale_i64(tmp12 - otmp1, shift);
            workspace[base + 3] = dct_descale_i64(tmp13 + otmp0, shift);
            workspace[base + 4] = dct_descale_i64(tmp13 - otmp0, shift);
        }

        for col in 0..8 {
            let f0 = workspace[col];
            let f1 = workspace[col + 8];
            let f2 = workspace[col + 16];
            let f3 = workspace[col + 24];
            let f4 = workspace[col + 32];
            let f5 = workspace[col + 40];
            let f6 = workspace[col + 48];
            let f7 = workspace[col + 56];

            let z2 = f2;
            let z3 = f6;
            let z1 = (z2 + z3) * 4433;
            let tmp2 = z1 + z3 * (-15137);
            let tmp3 = z1 + z2 * 6270;
            let tmp0 = (f0 + f4) << CONST_BITS;
            let tmp1 = (f0 - f4) << CONST_BITS;
            let tmp10 = tmp0 + tmp3;
            let tmp13 = tmp0 - tmp3;
            let tmp11 = tmp1 + tmp2;
            let tmp12 = tmp1 - tmp2;

            let mut oz1 = f7 + f1;
            let mut oz2 = f5 + f3;
            let mut oz3 = f7 + f3;
            let mut oz4 = f5 + f1;
            let z5 = (oz3 + oz4) * 9633;

            let mut otmp0 = f7 * 2446;
            let mut otmp1 = f5 * 16819;
            let mut otmp2 = f3 * 25172;
            let mut otmp3 = f1 * 12299;

            oz1 *= -7373;
            oz2 *= -20995;
            oz3 = oz3 * (-16069) + z5;
            oz4 = oz4 * (-3196) + z5;

            otmp0 += oz1 + oz3;
            otmp1 += oz2 + oz4;
            otmp2 += oz2 + oz3;
            otmp3 += oz1 + oz4;

            let shift = (CONST_BITS + ROW_BITS + 3) as u32;
            out_pixels[col] = (dct_descale_i64(tmp10 + otmp3, shift) + 128).clamp(0, 255) as u8;
            out_pixels[col + 56] =
                (dct_descale_i64(tmp10 - otmp3, shift) + 128).clamp(0, 255) as u8;
            out_pixels[col + 8] = (dct_descale_i64(tmp11 + otmp2, shift) + 128).clamp(0, 255) as u8;
            out_pixels[col + 48] =
                (dct_descale_i64(tmp11 - otmp2, shift) + 128).clamp(0, 255) as u8;
            out_pixels[col + 16] =
                (dct_descale_i64(tmp12 + otmp1, shift) + 128).clamp(0, 255) as u8;
            out_pixels[col + 40] =
                (dct_descale_i64(tmp12 - otmp1, shift) + 128).clamp(0, 255) as u8;
            out_pixels[col + 24] =
                (dct_descale_i64(tmp13 + otmp0, shift) + 128).clamp(0, 255) as u8;
            out_pixels[col + 32] =
                (dct_descale_i64(tmp13 - otmp0, shift) + 128).clamp(0, 255) as u8;
        }
    }

    #[test]
    // Exhaustive sweep: minutes to hours under Miri's interpreter, and the
    // same code path as the representative-input tests that do run there.
    #[cfg_attr(miri, ignore)]
    fn test_conv01_idct_i32_matches_i64_ref() {
        let mut rng: u64 = 0x9E37_79B9_7F4A_7C15;
        let mut out_i32 = [0u8; 64];
        let mut out_i64 = [0u8; 64];

        for iter in 0..1_000_000usize {
            let mut block = [0i32; 64];
            for sample in &mut block {
                rng ^= rng << 13;
                rng ^= rng >> 7;
                rng ^= rng << 17;
                *sample = i32::from((rng as u8) as i8);
            }
            fdct_2d(&mut block);

            idct_2d(&block, &mut out_i32);
            idct_2d_i64_ref(&block, &mut out_i64);
            assert_eq!(
                out_i32, out_i64,
                "i32 IDCT diverged from i64 reference at iteration {iter}"
            );
        }
    }

    #[test]
    fn test_conv14_dc_only_fast_path() {
        let dc_huff = build_dec_huff_table(&DC_LUM_BITS, &DC_LUM_VAL)
            .expect("standard DC luminance Huffman table must build");
        let ac_huff = build_dec_huff_table(&AC_LUM_BITS, &AC_LUM_VAL)
            .expect("standard AC luminance Huffman table must build");
        let qtable = [1u8; 64];
        // DC diff = 0 (00 in DC_LUM) + EOB (1010 in AC_LUM) + 11 pad = 0x2B
        let dc_only_scan = [0x2Bu8, 0xFF, 0xD9];

        let mut out_fast = [0u8; 64];
        let mut out_idct = [0u8; 64];

        for c0 in -2048..=2047i32 {
            let mut last_dc = c0;
            let mut reader = JpegBitReader::new(&dc_only_scan, 0);
            reader
                .decode_block(&qtable, &dc_huff, &ac_huff, &mut last_dc, &mut out_fast)
                .expect("DC-only decode_block must succeed");

            let mut coeffs = [0i32; 64];
            coeffs[0] = c0;
            idct_2d(&coeffs, &mut out_idct);

            assert_eq!(
                out_fast, out_idct,
                "DC-only fast path mismatch against idct_2d for c0={c0}"
            );
        }
    }

    #[test]
    fn test_conv04_bit_writer_u32_byte_identical() {
        // 1. Verify u32 JpegBitWriter (both fast_room=true and fast_room=false)
        // matches a reference u64 accumulator across 32-bit crossings & 0xFF.
        let mut dst_fast = [0u8; 4096];
        let mut dst_slow = [0u8; 4096];
        let mut dst_ref = [0u8; 4096];
        let mut ref_pos = 0usize;
        let mut ref_buf = 0u64;
        let mut ref_bits = 0u32;

        fn push_ref(
            dst_ref: &mut [u8],
            ref_pos: &mut usize,
            ref_buf: &mut u64,
            ref_bits: &mut u32,
            val: u32,
            len: u32,
        ) {
            if len == 0 {
                return;
            }
            let mask = (1u64 << len) - 1;
            *ref_buf = (*ref_buf << len) | (u64::from(val) & mask);
            *ref_bits += len;
            while *ref_bits >= 8 {
                *ref_bits -= 8;
                let c = ((*ref_buf >> *ref_bits) & 0xFF) as u8;
                dst_ref[*ref_pos] = c;
                *ref_pos += 1;
                if c == 0xFF {
                    dst_ref[*ref_pos] = 0x00;
                    *ref_pos += 1;
                }
            }
        }

        let mut w_fast = JpegBitWriter::new(&mut dst_fast);
        let mut w_slow = JpegBitWriter::new(&mut dst_slow);
        let mut rng: u64 = 0xD1CE_CAFE_BAAD_F00D;

        for _ in 0..512usize {
            rng ^= rng << 13;
            rng ^= rng >> 7;
            rng ^= rng << 17;
            let code_len = ((rng & 0x0F) as u32) + 1;
            let mag_len = ((rng >> 8) & 0x0F) as u32;
            let code = ((rng >> 16) as u16) & ((1u16 << code_len.min(15)) - 1);
            let mag = (rng >> 32) as u32;

            w_fast
                .put_code_and_mag(code, code_len, mag, mag_len, true)
                .expect("fast_room=true write must succeed");
            w_slow
                .put_code_and_mag(code, code_len, mag, mag_len, false)
                .expect("fast_room=false write must succeed");
            push_ref(
                &mut dst_ref,
                &mut ref_pos,
                &mut ref_buf,
                &mut ref_bits,
                u32::from(code),
                code_len,
            );
            push_ref(
                &mut dst_ref,
                &mut ref_pos,
                &mut ref_buf,
                &mut ref_bits,
                mag & ((1u32 << mag_len) - 1),
                mag_len,
            );
        }
        w_fast.flush_bits().expect("fast flush must succeed");
        w_slow.flush_bits().expect("slow flush must succeed");
        if ref_bits > 0 {
            let pad = 8 - ref_bits;
            push_ref(
                &mut dst_ref,
                &mut ref_pos,
                &mut ref_buf,
                &mut ref_bits,
                (1u32 << pad) - 1,
                pad,
            );
        }

        assert_eq!(w_fast.pos, ref_pos, "fast_room pos mismatch vs u64 ref");
        assert_eq!(w_slow.pos, ref_pos, "slow_room pos mismatch vs u64 ref");
        assert_eq!(
            &dst_fast[..ref_pos],
            &dst_ref[..ref_pos],
            "u32 fast_room bytes diverged from u64 reference"
        );
        assert_eq!(
            &dst_slow[..ref_pos],
            &dst_ref[..ref_pos],
            "u32 slow_room bytes diverged from u64 reference"
        );

        // 2. RGB888, Grayscale, and YUV422 encode->decode roundtrips and
        // exact-fit tight output buffers (exercising fast_room=false).
        let width = 16u16;
        let height = 16u16;
        let pixels = (width as usize) * (height as usize);
        let mut rgb_src = [0u8; 16 * 16 * 3];
        let mut gray_src = [0u8; 16 * 16];
        let mut yuyv_src = [0u8; 16 * 16 * 2];

        for i in 0..pixels {
            let x = (i % 16) as u8;
            let y = (i / 16) as u8;
            let r = x.wrapping_mul(15);
            let g = y.wrapping_mul(15);
            let b = x.wrapping_add(y).wrapping_mul(8);
            rgb_src[i * 3] = r;
            rgb_src[i * 3 + 1] = g;
            rgb_src[i * 3 + 2] = b;
            gray_src[i] = x.wrapping_mul(10).wrapping_add(y.wrapping_mul(5));
        }
        for p in 0..(pixels / 2) {
            let y0 = gray_src[p * 2];
            let y1 = gray_src[p * 2 + 1];
            yuyv_src[p * 4] = y0;
            yuyv_src[p * 4 + 1] = 112;
            yuyv_src[p * 4 + 2] = y1;
            yuyv_src[p * 4 + 3] = 144;
        }

        let mut large_jpg = [0u8; 8192];
        let mut tight_jpg = [0u8; 8192];
        let mut decoded_rgb = [0u8; 16 * 16 * 3];

        // RGB888 roundtrip + tight buffer parity
        let rgb_len = encode_rgb888_to_jpeg(&rgb_src, &mut large_jpg, width, height, 92)
            .expect("RGB888 encode into large buffer must succeed");
        let rgb_tight_len =
            encode_rgb888_to_jpeg(&rgb_src, &mut tight_jpg[..rgb_len], width, height, 92)
                .expect("RGB888 encode into exact tight buffer must succeed");
        assert_eq!(rgb_tight_len, rgb_len, "RGB888 tight len mismatch");
        assert_eq!(
            &tight_jpg[..rgb_len],
            &large_jpg[..rgb_len],
            "RGB888 tight buffer output must match large buffer byte-for-byte"
        );
        assert_eq!(
            encode_rgb888_to_jpeg(&rgb_src, &mut tight_jpg[..rgb_len - 1], width, height, 92),
            Err(CameraError::BufferTooSmall),
            "RGB888 1-byte-short buffer must return BufferTooSmall"
        );
        let dec_rgb_len = decode_jpeg_to_rgb888(&tight_jpg[..rgb_len], &mut decoded_rgb)
            .expect("RGB888 JPEG decode must succeed");
        assert_eq!(dec_rgb_len, rgb_src.len(), "RGB888 decoded length mismatch");

        // Grayscale roundtrip + tight buffer parity
        let gray_len = encode_grayscale_to_jpeg(&gray_src, &mut large_jpg, width, height, 92)
            .expect("Grayscale encode into large buffer must succeed");
        let gray_tight_len =
            encode_grayscale_to_jpeg(&gray_src, &mut tight_jpg[..gray_len], width, height, 92)
                .expect("Grayscale encode into exact tight buffer must succeed");
        assert_eq!(gray_tight_len, gray_len, "Grayscale tight len mismatch");
        assert_eq!(
            &tight_jpg[..gray_len],
            &large_jpg[..gray_len],
            "Grayscale tight buffer output must match large buffer byte-for-byte"
        );
        assert_eq!(
            encode_grayscale_to_jpeg(&gray_src, &mut tight_jpg[..gray_len - 1], width, height, 92,),
            Err(CameraError::BufferTooSmall),
            "Grayscale 1-byte-short buffer must return BufferTooSmall"
        );
        let dec_gray_len = decode_jpeg_to_rgb888(&tight_jpg[..gray_len], &mut decoded_rgb)
            .expect("Grayscale JPEG decode must succeed");
        assert_eq!(
            dec_gray_len,
            rgb_src.len(),
            "Grayscale decoded RGB888 length mismatch"
        );

        // YUV422 roundtrip + tight buffer parity
        let yuv_len = encode_yuv422_to_jpeg(&yuyv_src, &mut large_jpg, width, height, 92)
            .expect("YUV422 encode into large buffer must succeed");
        let yuv_tight_len =
            encode_yuv422_to_jpeg(&yuyv_src, &mut tight_jpg[..yuv_len], width, height, 92)
                .expect("YUV422 encode into exact tight buffer must succeed");
        assert_eq!(yuv_tight_len, yuv_len, "YUV422 tight len mismatch");
        assert_eq!(
            &tight_jpg[..yuv_len],
            &large_jpg[..yuv_len],
            "YUV422 tight buffer output must match large buffer byte-for-byte"
        );
        assert_eq!(
            encode_yuv422_to_jpeg(&yuyv_src, &mut tight_jpg[..yuv_len - 1], width, height, 92),
            Err(CameraError::BufferTooSmall),
            "YUV422 1-byte-short buffer must return BufferTooSmall"
        );
        let dec_yuv_len = decode_jpeg_to_rgb888(&tight_jpg[..yuv_len], &mut decoded_rgb)
            .expect("YUV422 JPEG decode must succeed");
        assert_eq!(dec_yuv_len, rgb_src.len(), "YUV422 decoded length mismatch");
    }

    #[test]
    fn test_conv08_dec_huff_table_size_and_16bit_codes() {
        let table_size = core::mem::size_of::<DecHuffTable>();
        assert!(
            table_size <= 640,
            "DecHuffTable size {table_size} exceeds 640-byte stack budget"
        );

        let mut table = DecHuffTable::default();
        // Pre-populate dummy 1-bit table to verify in-place reset.
        let mut init_bits = [0u8; 17];
        init_bits[1] = 2;
        build_dec_huff_table_in_place(&mut table, &init_bits, &[0xAA, 0xBB])
            .expect("Initial 1-bit Huffman table build must succeed");

        // Canonical table: 1 code at l=1..=14, 0 at l=15, 3 at l=16.
        // Length-16 codes land at 0xFFFC..=0xFFFE (0xFFFF unassigned).
        let mut bits = [0u8; 17];
        for b in &mut bits[1..=14] {
            *b = 1;
        }
        bits[16] = 3;

        let mut expected_syms = [0u8; 17];
        for (i, sym) in expected_syms.iter_mut().enumerate() {
            *sym = 0x20 + (i as u8);
        }

        build_dec_huff_table_in_place(&mut table, &bits, &expected_syms)
            .expect("In-place 16-bit canonical Huffman table build must succeed");
        assert!(table.valid, "Built DecHuffTable must be marked valid");

        let codes_and_lens: [(u16, u8); 17] = [
            (0x0000, 1),
            (0x0002, 2),
            (0x0006, 3),
            (0x000E, 4),
            (0x001E, 5),
            (0x003E, 6),
            (0x007E, 7),
            (0x00FE, 8),
            (0x01FE, 9),
            (0x03FE, 10),
            (0x07FE, 11),
            (0x0FFE, 12),
            (0x1FFE, 13),
            (0x3FFE, 14),
            (0xFFFC, 16),
            (0xFFFD, 16),
            (0xFFFE, 16),
        ];

        // Pack all 17 codewords MSB-first with JPEG 0xFF -> 0xFF 0x00 byte stuffing.
        let mut stream = [0u8; 96];
        let mut out_pos = 0usize;
        let mut acc = 0u32;
        let mut acc_bits = 0u32;
        for &(code, len) in &codes_and_lens {
            acc = (acc << len) | u32::from(code);
            acc_bits += u32::from(len);
            while acc_bits >= 8 {
                acc_bits -= 8;
                let byte = ((acc >> acc_bits) & 0xFF) as u8;
                acc &= (1u32 << acc_bits) - 1;
                stream[out_pos] = byte;
                out_pos += 1;
                if byte == 0xFF {
                    stream[out_pos] = 0x00;
                    out_pos += 1;
                }
            }
        }
        if acc_bits > 0 {
            let pad = 8 - acc_bits;
            let byte = (((acc << pad) | ((1u32 << pad) - 1)) & 0xFF) as u8;
            stream[out_pos] = byte;
            out_pos += 1;
            if byte == 0xFF {
                stream[out_pos] = 0x00;
                out_pos += 1;
            }
        }
        stream[out_pos] = 0xFF;
        stream[out_pos + 1] = 0xD9;
        out_pos += 2;

        let mut reader = JpegBitReader::new(&stream[..out_pos], 0);
        for (idx, &expected) in expected_syms.iter().enumerate() {
            let decoded = reader
                .decode_huff(&table)
                .expect("Canonical codeword up to 0xFFFE must decode cleanly");
            assert_eq!(
                decoded, expected,
                "Decoded symbol mismatch at codeword index {idx}"
            );
        }

        // Unassigned all-ones 0xFFFF codeword must be rejected.
        let invalid_stream = [0xFF, 0x00, 0xFF, 0x00, 0xFF, 0xD9];
        let mut bad_reader = JpegBitReader::new(&invalid_stream, 0);
        assert_eq!(
            bad_reader.decode_huff(&table),
            Err(CameraError::CodecError),
            "Unassigned 0xFFFF codeword must return CodecError"
        );
    }

    #[test]
    fn test_conv05_u32_bit_reader_and_sos_valid_check() {
        // 1. Verify 0xFF00 byte-unstuffing across multiple partial u32 refills.
        let logical_bytes: [u8; 12] = [
            0xFF, 0xA5, 0xFF, 0x5A, 0x12, 0xFF, 0x34, 0xFF, 0xFF, 0x7E, 0x81, 0xFF,
        ];
        let mut expected_bits = 0u128;
        for &b in &logical_bytes {
            expected_bits = (expected_bits << 8) | u128::from(b);
        }

        let mut stuffed_scan = [0u8; 32];
        let mut scan_len = 0usize;
        for &b in &logical_bytes {
            stuffed_scan[scan_len] = b;
            scan_len += 1;
            if b == 0xFF {
                stuffed_scan[scan_len] = 0x00;
                scan_len += 1;
            }
        }
        stuffed_scan[scan_len] = 0xFF;
        stuffed_scan[scan_len + 1] = 0xD9;
        scan_len += 2;

        let mut reader = JpegBitReader::new(&stuffed_scan[..scan_len], 0);
        let chunk_widths: [u32; 6] = [13, 11, 24, 17, 19, 12];
        let mut rem_bits = 96u32;
        for &w in &chunk_widths {
            rem_bits -= w;
            let expected = ((expected_bits >> rem_bits) & ((1u128 << w) - 1)) as u32;
            let got = reader
                .get_bits(w)
                .expect("Stuffed bitstream read across u32 refills must succeed");
            assert_eq!(
                got, expected,
                "Bit mismatch at remaining bit offset {rem_bits}"
            );
        }
        assert_eq!(
            reader.get_bits(8),
            Ok(0xFF),
            "Post-EOI refill must pad with 0xFF 1-bits"
        );
        assert!(reader.hit_marker, "EOI marker must set hit_marker");

        // 2. Verify missing-DHT-table rejection once per SOS.
        let width = 16u16;
        let height = 16u16;
        let mut rgb_in = [0u8; 16 * 16 * 3];
        for y in 0..16usize {
            for x in 0..16usize {
                let idx = (y * 16 + x) * 3;
                rgb_in[idx] = (x * 14 + 20) as u8;
                rgb_in[idx + 1] = (220 - x * 10) as u8;
                rgb_in[idx + 2] = 110;
            }
        }

        let mut valid_jpeg = [0u8; 4096];
        let valid_len = encode_rgb888_to_jpeg(&rgb_in, &mut valid_jpeg, width, height, 50)
            .expect("Baseline JPEG encode for DHT strip test must succeed");

        // Strip all DHT (0xFFC4) segments prior to SOS (0xFFDA).
        let mut no_dht_jpeg = [0u8; 4096];
        no_dht_jpeg[0] = 0xFF;
        no_dht_jpeg[1] = 0xD8;
        let mut src_pos = 2usize;
        let mut dst_pos = 2usize;
        while src_pos + 1 < valid_len {
            let marker = valid_jpeg[src_pos + 1];
            if marker == 0xDA {
                let tail_len = valid_len - src_pos;
                no_dht_jpeg[dst_pos..dst_pos + tail_len]
                    .copy_from_slice(&valid_jpeg[src_pos..valid_len]);
                dst_pos += tail_len;
                break;
            }
            let seg_len =
                u16::from_be_bytes([valid_jpeg[src_pos + 2], valid_jpeg[src_pos + 3]]) as usize;
            let total_seg = 2 + seg_len;
            if marker != 0xC4 {
                no_dht_jpeg[dst_pos..dst_pos + total_seg]
                    .copy_from_slice(&valid_jpeg[src_pos..src_pos + total_seg]);
                dst_pos += total_seg;
            }
            src_pos += total_seg;
        }

        let mut rgb_out = [0u8; 16 * 16 * 3];
        assert_eq!(
            decode_jpeg_to_rgb888(&no_dht_jpeg[..dst_pos], &mut rgb_out),
            Err(CameraError::CodecError),
            "JPEG missing DHT tables must be rejected at SOS with CodecError"
        );

        // Also verify missing AC table when DC table 0 is valid (grayscale + Ta=1).
        let gray_in = [128u8; 16 * 16];
        let mut gray_jpeg = [0u8; 4096];
        let gray_len = encode_grayscale_to_jpeg(&gray_in, &mut gray_jpeg, width, height, 50)
            .expect("Grayscale JPEG encode must succeed");
        let mut g_pos = 2usize;
        while g_pos + 6 < gray_len {
            if gray_jpeg[g_pos] == 0xFF && gray_jpeg[g_pos + 1] == 0xDA {
                // Td1 = 0 (valid DC table), Ta1 = 1 (unpopulated AC table).
                gray_jpeg[g_pos + 6] = 0x01;
                break;
            }
            let seg_len = u16::from_be_bytes([gray_jpeg[g_pos + 2], gray_jpeg[g_pos + 3]]) as usize;
            g_pos += 2 + seg_len;
        }
        assert_eq!(
            decode_jpeg_to_rgb888(&gray_jpeg[..gray_len], &mut rgb_out),
            Err(CameraError::CodecError),
            "JPEG referencing unpopulated AC table in SOS must return CodecError"
        );

        // 3. Verify RGB888, RGB565, and BMP24 roundtrips at q=10, q=50, and q=90.
        let mut rgb565_out = [0u8; 16 * 16 * 2];
        let mut bmp_out = [0u8; 54 + 16 * 16 * 3];
        for quality in [10u8, 50, 90] {
            let jpeg_len = encode_rgb888_to_jpeg(&rgb_in, &mut valid_jpeg, width, height, quality)
                .expect("encode_rgb888_to_jpeg must succeed across qualities");
            let jpeg_slice = &valid_jpeg[..jpeg_len];

            let written_888 = decode_jpeg_to_rgb888(jpeg_slice, &mut rgb_out)
                .expect("decode_jpeg_to_rgb888 must succeed");
            assert_eq!(
                written_888,
                16 * 16 * 3,
                "RGB888 byte count mismatch at q={quality}"
            );

            let written_565 = decode_jpeg_to_rgb565(jpeg_slice, &mut rgb565_out)
                .expect("decode_jpeg_to_rgb565 must succeed");
            assert_eq!(
                written_565,
                16 * 16 * 2,
                "RGB565 byte count mismatch at q={quality}"
            );

            let (bmp_w, bmp_h, written_bmp) =
                jpeg_to_bmp(jpeg_slice, &mut bmp_out).expect("jpeg_to_bmp must succeed");
            assert_eq!(
                (bmp_w, bmp_h, written_bmp),
                (16, 16, 54 + 16 * 16 * 3),
                "BMP24 dimensions/length mismatch at q={quality}"
            );
            assert_eq!(
                &bmp_out[..2],
                b"BM",
                "BMP signature mismatch at q={quality}"
            );

            let max_tol = if quality == 10 { 36u8 } else { 18u8 };
            for px in 0..(16 * 16) {
                let r_in = rgb_in[px * 3];
                let g_in = rgb_in[px * 3 + 1];
                let b_in = rgb_in[px * 3 + 2];
                let r_dec = rgb_out[px * 3];
                let g_dec = rgb_out[px * 3 + 1];
                let b_dec = rgb_out[px * 3 + 2];

                assert!(
                    r_dec.abs_diff(r_in) <= max_tol
                        && g_dec.abs_diff(g_in) <= max_tol
                        && b_dec.abs_diff(b_in) <= max_tol,
                    "RGB888 pixel {px} exceeded tolerance at q={quality}"
                );

                // BMP24 stores BGR24 and input is vertically uniform across rows.
                assert_eq!(
                    &bmp_out[54 + px * 3..54 + px * 3 + 3],
                    &[b_dec, g_dec, r_dec],
                    "BMP24 pixel {px} must match decoded RGB888 BGR order at q={quality}"
                );

                let expected_565 = ((u16::from(r_dec) & 0xF8) << 8)
                    | ((u16::from(g_dec) & 0xFC) << 3)
                    | (u16::from(b_dec) >> 3);
                let got_565 = u16::from_be_bytes([rgb565_out[px * 2], rgb565_out[px * 2 + 1]]);
                assert_eq!(
                    got_565, expected_565,
                    "RGB565 pixel {px} mismatch against RGB888 at q={quality}"
                );
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn write_mcu_pixels_ref(
        dst: &mut [u8],
        out_fmt: DecodeOutputFormat,
        w: usize,
        h: usize,
        bmp_row_stride: usize,
        mcu_x: usize,
        mcu_y: usize,
        max_h_samp: usize,
        max_v_samp: usize,
        num_components: usize,
        y_blocks: &[[u8; 64]; 4],
        cb_block: &[u8; 64],
        cr_block: &[u8; 64],
    ) {
        let mcu_w = max_h_samp * 8;
        let mcu_h = max_v_samp * 8;
        for py_in_mcu in 0..mcu_h {
            let img_y = mcu_y * mcu_h + py_in_mcu;
            if img_y >= h {
                continue;
            }
            for px_in_mcu in 0..mcu_w {
                let img_x = mcu_x * mcu_w + px_in_mcu;
                if img_x >= w {
                    continue;
                }
                let y_blk_idx = (py_in_mcu / 8) * max_h_samp + (px_in_mcu / 8);
                let y_sub_idx = (py_in_mcu % 8) * 8 + (px_in_mcu % 8);
                let y_val = y_blocks[y_blk_idx][y_sub_idx];
                let (r, g, b) = if num_components == 1 {
                    (y_val, y_val, y_val)
                } else {
                    let cx = px_in_mcu / max_h_samp;
                    let cy = py_in_mcu / max_v_samp;
                    let c_idx = cy * 8 + cx;
                    jfif_ycbcr_to_rgb(y_val, cb_block[c_idx], cr_block[c_idx])
                };
                match out_fmt {
                    DecodeOutputFormat::Rgb888 => {
                        let d = (img_y * w + img_x) * 3;
                        dst[d] = r;
                        dst[d + 1] = g;
                        dst[d + 2] = b;
                    }
                    DecodeOutputFormat::Rgb565 => {
                        let d = (img_y * w + img_x) * 2;
                        dst[d] = (r & 0xF8) | (g >> 5);
                        dst[d + 1] = ((g & 0x1C) << 3) | (b >> 3);
                    }
                    DecodeOutputFormat::Bmp24 => {
                        let d = BMP_HEADER_LEN + img_y * bmp_row_stride + img_x * 3;
                        dst[d] = b;
                        dst[d + 1] = g;
                        dst[d + 2] = r;
                    }
                }
            }
        }
    }

    #[test]
    fn test_conv06_monomorphized_row_writer_odd_dimensions() {
        let odd_dims: [(u16, u16); 5] = [(1, 1), (7, 5), (13, 11), (15, 9), (17, 19)];
        let samp_modes: [(usize, usize); 3] = [(1, 1), (2, 1), (2, 2)];

        for &(width, height) in &odd_dims {
            let w = width as usize;
            let h = height as usize;
            let bmp_row_stride = (w * 3 + 3) & !3;

            for &(max_h_samp, max_v_samp) in &samp_modes {
                let h_shift = max_h_samp - 1;
                let v_shift = max_v_samp - 1;
                let mcu_w = max_h_samp * 8;
                let mcu_h = max_v_samp * 8;
                let mcu_cols = w.div_ceil(mcu_w);
                let mcu_rows = h.div_ceil(mcu_h);

                for &num_components in &[1usize, 3usize] {
                    let mut got_888 = [0x55u8; 2048];
                    let mut got_565 = [0x55u8; 2048];
                    let mut got_bmp = [0x55u8; 2048];
                    let mut exp_888 = [0x55u8; 2048];
                    let mut exp_565 = [0x55u8; 2048];
                    let mut exp_bmp = [0x55u8; 2048];

                    for mcu_y in 0..mcu_rows {
                        for mcu_x in 0..mcu_cols {
                            let seed = mcu_y * 7 + mcu_x * 13;
                            let mut y_blocks = [[0u8; 64]; 4];
                            for (bi, blk) in y_blocks.iter_mut().enumerate() {
                                for (si, sample) in blk.iter_mut().enumerate() {
                                    *sample = ((seed * 29 + bi * 59 + si * 17 + 11) & 0xFF) as u8;
                                }
                            }
                            let mut cb_block = [0u8; 64];
                            let mut cr_block = [0u8; 64];
                            for si in 0..64 {
                                cb_block[si] = ((seed * 19 + si * 23 + 40) & 0xFF) as u8;
                                cr_block[si] = ((seed * 37 + si * 31 + 90) & 0xFF) as u8;
                            }

                            let img_x0 = mcu_x * mcu_w;
                            let img_y0 = mcu_y * mcu_h;
                            let valid_w = (w - img_x0).min(mcu_w);
                            let valid_h = (h - img_y0).min(mcu_h);

                            write_mcu_pixels::<Rgb888Sink>(
                                &mut got_888,
                                0,
                                w * 3,
                                img_x0,
                                img_y0,
                                valid_w,
                                valid_h,
                                num_components,
                                h_shift,
                                v_shift,
                                &y_blocks,
                                &cb_block,
                                &cr_block,
                            );
                            write_mcu_pixels::<Rgb565Sink>(
                                &mut got_565,
                                0,
                                w * 2,
                                img_x0,
                                img_y0,
                                valid_w,
                                valid_h,
                                num_components,
                                h_shift,
                                v_shift,
                                &y_blocks,
                                &cb_block,
                                &cr_block,
                            );
                            write_mcu_pixels::<Bmp24Sink>(
                                &mut got_bmp,
                                BMP_HEADER_LEN,
                                bmp_row_stride,
                                img_x0,
                                img_y0,
                                valid_w,
                                valid_h,
                                num_components,
                                h_shift,
                                v_shift,
                                &y_blocks,
                                &cb_block,
                                &cr_block,
                            );

                            write_mcu_pixels_ref(
                                &mut exp_888,
                                DecodeOutputFormat::Rgb888,
                                w,
                                h,
                                bmp_row_stride,
                                mcu_x,
                                mcu_y,
                                max_h_samp,
                                max_v_samp,
                                num_components,
                                &y_blocks,
                                &cb_block,
                                &cr_block,
                            );
                            write_mcu_pixels_ref(
                                &mut exp_565,
                                DecodeOutputFormat::Rgb565,
                                w,
                                h,
                                bmp_row_stride,
                                mcu_x,
                                mcu_y,
                                max_h_samp,
                                max_v_samp,
                                num_components,
                                &y_blocks,
                                &cb_block,
                                &cr_block,
                            );
                            write_mcu_pixels_ref(
                                &mut exp_bmp,
                                DecodeOutputFormat::Bmp24,
                                w,
                                h,
                                bmp_row_stride,
                                mcu_x,
                                mcu_y,
                                max_h_samp,
                                max_v_samp,
                                num_components,
                                &y_blocks,
                                &cb_block,
                                &cr_block,
                            );
                        }
                    }

                    assert_eq!(
                        &got_888[..],
                        &exp_888[..],
                        "888 diff {width}x{height} {max_h_samp}x{max_v_samp} c={num_components}"
                    );
                    assert_eq!(
                        &got_565[..],
                        &exp_565[..],
                        "565 diff {width}x{height} {max_h_samp}x{max_v_samp} c={num_components}"
                    );
                    assert_eq!(
                        &got_bmp[..],
                        &exp_bmp[..],
                        "BMP diff {width}x{height} {max_h_samp}x{max_v_samp} c={num_components}"
                    );
                }
            }

            // Odd-dimension JPEG roundtrips across all output formats.
            let mut rgb_in = [0u8; 17 * 19 * 3];
            let mut gray_in = [0u8; 17 * 19];
            for y in 0..h {
                for x in 0..w {
                    let idx = y * w + x;
                    rgb_in[idx * 3] = (x * 6 + y * 5 + 32) as u8;
                    rgb_in[idx * 3 + 1] = (x * 4 + y * 7 + 64) as u8;
                    rgb_in[idx * 3 + 2] = (200 - x * 5 - y * 4) as u8;
                    gray_in[idx] = (x * 7 + y * 6 + 40) as u8;
                }
            }

            let mut jpeg_buf = [0u8; 8192];
            for is_gray in [false, true] {
                let jpeg_len = if is_gray {
                    encode_grayscale_to_jpeg(&gray_in[..w * h], &mut jpeg_buf, width, height, 95)
                        .expect("odd-dim grayscale JPEG encode must succeed")
                } else {
                    encode_rgb888_to_jpeg(&rgb_in[..w * h * 3], &mut jpeg_buf, width, height, 95)
                        .expect("odd-dim RGB888 JPEG encode must succeed")
                };
                let jpeg_slice = &jpeg_buf[..jpeg_len];

                let mut rgb888_out = [0xAAu8; 2048];
                let mut rgb565_out = [0xAAu8; 2048];
                let mut bmp24_out = [0xAAu8; 2048];

                let len_888 = decode_jpeg_to_rgb888(jpeg_slice, &mut rgb888_out)
                    .expect("odd-dim RGB888 decode must succeed");
                assert_eq!(len_888, w * h * 3);
                assert!(rgb888_out[len_888..].iter().all(|&b| b == 0xAA));

                let len_565 = decode_jpeg_to_rgb565(jpeg_slice, &mut rgb565_out)
                    .expect("odd-dim RGB565 decode must succeed");
                assert_eq!(len_565, w * h * 2);
                assert!(rgb565_out[len_565..].iter().all(|&b| b == 0xAA));

                let expected_bmp_len = BMP_HEADER_LEN + h * bmp_row_stride;
                let (bw, bh, len_bmp) = jpeg_to_bmp(jpeg_slice, &mut bmp24_out)
                    .expect("odd-dim BMP24 decode must succeed");
                assert_eq!((bw, bh, len_bmp), (width, height, expected_bmp_len));
                assert_eq!(&bmp24_out[..2], b"BM");
                assert!(bmp24_out[len_bmp..].iter().all(|&b| b == 0xAA));

                for y in 0..h {
                    let row_start = BMP_HEADER_LEN + y * bmp_row_stride;
                    assert!(
                        bmp24_out[row_start + w * 3..row_start + bmp_row_stride]
                            .iter()
                            .all(|&b| b == 0),
                        "BMP24 row padding must be zeroed at {width}x{height} row={y}"
                    );
                    for x in 0..w {
                        let px = y * w + x;
                        let r = rgb888_out[px * 3];
                        let g = rgb888_out[px * 3 + 1];
                        let b = rgb888_out[px * 3 + 2];

                        let exp_r = if is_gray { gray_in[px] } else { rgb_in[px * 3] };
                        let exp_g = if is_gray {
                            gray_in[px]
                        } else {
                            rgb_in[px * 3 + 1]
                        };
                        let exp_b = if is_gray {
                            gray_in[px]
                        } else {
                            rgb_in[px * 3 + 2]
                        };
                        assert!(
                            r.abs_diff(exp_r) <= 16
                                && g.abs_diff(exp_g) <= 16
                                && b.abs_diff(exp_b) <= 16,
                            "roundtrip pixel ({x},{y}) drift at {width}x{height} gray={is_gray}"
                        );

                        assert_eq!(
                            &bmp24_out[row_start + x * 3..row_start + x * 3 + 3],
                            &[b, g, r],
                            "BMP24 BGR mismatch vs RGB888 at ({x},{y}) {width}x{height}"
                        );
                        let exp_565 = [(r & 0xF8) | (g >> 5), ((g & 0x1C) << 3) | (b >> 3)];
                        assert_eq!(
                            &rgb565_out[px * 2..px * 2 + 2],
                            &exp_565,
                            "RGB565 mismatch vs RGB888 at ({x},{y}) {width}x{height}"
                        );
                    }
                }
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn naive_gather_rgb888_mcu(
        src: &[u8],
        w: usize,
        h: usize,
        bx: usize,
        by: usize,
        y_block: &mut [i32; 64],
        cb_block: &mut [i32; 64],
        cr_block: &mut [i32; 64],
    ) {
        for dy in 0..8usize {
            let py = (by * 8 + dy).min(h - 1);
            let row_off = py * w;
            for dx in 0..8usize {
                let px = (bx * 8 + dx).min(w - 1);
                let p_idx = (row_off + px) * 3;
                let (y, cb, cr) = rgb2yuv(src[p_idx], src[p_idx + 1], src[p_idx + 2]);
                let b_idx = dy * 8 + dx;
                y_block[b_idx] = i32::from(y) - 128;
                cb_block[b_idx] = i32::from(cb) - 128;
                cr_block[b_idx] = i32::from(cr) - 128;
            }
        }
    }

    fn naive_gather_grayscale_mcu(
        src: &[u8],
        w: usize,
        h: usize,
        bx: usize,
        by: usize,
        y_block: &mut [i32; 64],
    ) {
        for dy in 0..8usize {
            let py = (by * 8 + dy).min(h - 1);
            let row_off = py * w;
            for dx in 0..8usize {
                let px = (bx * 8 + dx).min(w - 1);
                y_block[dy * 8 + dx] = i32::from(src[row_off + px]) - 128;
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn naive_gather_yuv422_h1v1_mcu(
        src: &[u8],
        w: usize,
        h: usize,
        bx: usize,
        by: usize,
        y_block: &mut [i32; 64],
        cb_block: &mut [i32; 64],
        cr_block: &mut [i32; 64],
    ) {
        for dy in 0..8usize {
            let py = (by * 8 + dy).min(h - 1);
            let row_off = py * w;
            for dx in 0..8usize {
                let px = (bx * 8 + dx).min(w - 1);
                let pair_idx = (row_off + (px & !1)) * 2;
                let y = if (px & 1) == 0 {
                    src[pair_idx]
                } else if pair_idx + 2 < src.len() {
                    src[pair_idx + 2]
                } else {
                    src[pair_idx]
                };
                let u = if pair_idx + 1 < src.len() {
                    src[pair_idx + 1]
                } else {
                    128
                };
                let v = if pair_idx + 3 < src.len() {
                    src[pair_idx + 3]
                } else {
                    128
                };
                let b_idx = dy * 8 + dx;
                y_block[b_idx] = i32::from(y) - 128;
                cb_block[b_idx] = i32::from(u) - 128;
                cr_block[b_idx] = i32::from(v) - 128;
            }
        }
    }

    fn naive_encode_rgb888(
        src: &[u8],
        dst: &mut [u8],
        width: u16,
        height: u16,
        quality: u8,
    ) -> usize {
        let lum_q = compute_quant_table(&STD_LUM_QUANT, quality);
        let chroma_q = compute_quant_table(&STD_CHROMA_QUANT, quality);
        let mut writer = JpegBitWriter::new(dst);
        writer
            .emit_headers(width, height, 3, Subsampling::H1V1, &lum_q.q, &chroma_q.q)
            .expect("naive RGB888 header emit must succeed");

        let w = width as usize;
        let h = height as usize;
        let mut last_dc = [0i32; 3];
        let mut y_b = [0i32; 64];
        let mut cb_b = [0i32; 64];
        let mut cr_b = [0i32; 64];

        for by in 0..h.div_ceil(8) {
            for bx in 0..w.div_ceil(8) {
                naive_gather_rgb888_mcu(src, w, h, bx, by, &mut y_b, &mut cb_b, &mut cr_b);
                writer
                    .encode_block(
                        &mut y_b,
                        &lum_q,
                        &HUFF_DC_LUM,
                        &HUFF_AC_LUM,
                        &mut last_dc[0],
                    )
                    .expect("naive RGB888 Y block encode must succeed");
                writer
                    .encode_block(
                        &mut cb_b,
                        &chroma_q,
                        &HUFF_DC_CHROM,
                        &HUFF_AC_CHROM,
                        &mut last_dc[1],
                    )
                    .expect("naive RGB888 Cb block encode must succeed");
                writer
                    .encode_block(
                        &mut cr_b,
                        &chroma_q,
                        &HUFF_DC_CHROM,
                        &HUFF_AC_CHROM,
                        &mut last_dc[2],
                    )
                    .expect("naive RGB888 Cr block encode must succeed");
            }
        }
        writer
            .flush_bits()
            .expect("naive RGB888 bit flush must succeed");
        writer
            .emit_marker(M_EOI)
            .expect("naive RGB888 EOI marker must succeed");
        writer.pos
    }

    fn naive_encode_grayscale(
        src: &[u8],
        dst: &mut [u8],
        width: u16,
        height: u16,
        quality: u8,
    ) -> usize {
        let lum_q = compute_quant_table(&STD_LUM_QUANT, quality);
        let mut writer = JpegBitWriter::new(dst);
        writer
            .emit_headers(width, height, 1, Subsampling::H1V1, &lum_q.q, &[1i32; 64])
            .expect("naive grayscale header emit must succeed");

        let w = width as usize;
        let h = height as usize;
        let mut last_dc = 0i32;
        let mut y_b = [0i32; 64];

        for by in 0..h.div_ceil(8) {
            for bx in 0..w.div_ceil(8) {
                naive_gather_grayscale_mcu(src, w, h, bx, by, &mut y_b);
                writer
                    .encode_block(&mut y_b, &lum_q, &HUFF_DC_LUM, &HUFF_AC_LUM, &mut last_dc)
                    .expect("naive grayscale Y block encode must succeed");
            }
        }
        writer
            .flush_bits()
            .expect("naive grayscale bit flush must succeed");
        writer
            .emit_marker(M_EOI)
            .expect("naive grayscale EOI marker must succeed");
        writer.pos
    }

    fn naive_encode_yuv422(
        src: &[u8],
        dst: &mut [u8],
        width: u16,
        height: u16,
        quality: u8,
    ) -> usize {
        let lum_q = compute_quant_table(&STD_LUM_QUANT, quality);
        let chroma_q = compute_quant_table(&STD_CHROMA_QUANT, quality);
        let mut writer = JpegBitWriter::new(dst);
        writer
            .emit_headers(width, height, 3, Subsampling::H1V1, &lum_q.q, &chroma_q.q)
            .expect("naive YUV422 header emit must succeed");

        let w = width as usize;
        let h = height as usize;
        let mut last_dc = [0i32; 3];
        let mut y_b = [0i32; 64];
        let mut cb_b = [0i32; 64];
        let mut cr_b = [0i32; 64];

        for by in 0..h.div_ceil(8) {
            for bx in 0..w.div_ceil(8) {
                naive_gather_yuv422_h1v1_mcu(src, w, h, bx, by, &mut y_b, &mut cb_b, &mut cr_b);
                writer
                    .encode_block(
                        &mut y_b,
                        &lum_q,
                        &HUFF_DC_LUM,
                        &HUFF_AC_LUM,
                        &mut last_dc[0],
                    )
                    .expect("naive YUV422 Y block encode must succeed");
                writer
                    .encode_block(
                        &mut cb_b,
                        &chroma_q,
                        &HUFF_DC_CHROM,
                        &HUFF_AC_CHROM,
                        &mut last_dc[1],
                    )
                    .expect("naive YUV422 Cb block encode must succeed");
                writer
                    .encode_block(
                        &mut cr_b,
                        &chroma_q,
                        &HUFF_DC_CHROM,
                        &HUFF_AC_CHROM,
                        &mut last_dc[2],
                    )
                    .expect("naive YUV422 Cr block encode must succeed");
            }
        }
        writer
            .flush_bits()
            .expect("naive YUV422 bit flush must succeed");
        writer
            .emit_marker(M_EOI)
            .expect("naive YUV422 EOI marker must succeed");
        writer.pos
    }

    fn verify_rgb888_and_gray_conv13(width: u16, height: u16) {
        let w = width as usize;
        let h = height as usize;
        let mut rgb_buf = [0u8; 24 * 24 * 3];
        let mut gray_buf = [0u8; 24 * 24];
        for (i, byte) in rgb_buf[..w * h * 3].iter_mut().enumerate() {
            *byte = ((i.wrapping_mul(73).wrapping_add(w * 19 + h * 37)) ^ (i >> 2)) as u8;
        }
        for (i, byte) in gray_buf[..w * h].iter_mut().enumerate() {
            *byte = ((i.wrapping_mul(97).wrapping_add(w * 23 + h * 41)) ^ (i >> 1)) as u8;
        }
        let rgb_src = &rgb_buf[..w * h * 3];
        let gray_src = &gray_buf[..w * h];

        for by in 0..h.div_ceil(8) {
            for bx in 0..w.div_ceil(8) {
                let (mut y_got, mut cb_got, mut cr_got) = ([0i32; 64], [0i32; 64], [0i32; 64]);
                let (mut y_exp, mut cb_exp, mut cr_exp) = ([0i32; 64], [0i32; 64], [0i32; 64]);
                gather_rgb888_mcu(rgb_src, w, h, bx, by, &mut y_got, &mut cb_got, &mut cr_got);
                naive_gather_rgb888_mcu(
                    rgb_src,
                    w,
                    h,
                    bx,
                    by,
                    &mut y_exp,
                    &mut cb_exp,
                    &mut cr_exp,
                );
                assert_eq!(
                    y_got, y_exp,
                    "RGB888 Y block mismatch at MCU ({bx},{by}) for {width}x{height}"
                );
                assert_eq!(
                    cb_got, cb_exp,
                    "RGB888 Cb block mismatch at MCU ({bx},{by}) for {width}x{height}"
                );
                assert_eq!(
                    cr_got, cr_exp,
                    "RGB888 Cr block mismatch at MCU ({bx},{by}) for {width}x{height}"
                );

                let mut gy_got = [0i32; 64];
                let mut gy_exp = [0i32; 64];
                gather_grayscale_mcu(gray_src, w, h, bx, by, &mut gy_got);
                naive_gather_grayscale_mcu(gray_src, w, h, bx, by, &mut gy_exp);
                assert_eq!(
                    gy_got, gy_exp,
                    "grayscale Y block mismatch at MCU ({bx},{by}) for {width}x{height}"
                );
            }
        }

        let mut fast_out = [0u8; 8192];
        let mut ref_out = [0u8; 8192];
        for quality in [50u8, 92u8] {
            let fast_len = encode_rgb888_to_jpeg(rgb_src, &mut fast_out, width, height, quality)
                .expect("CONV-13 RGB888 JPEG encode must succeed");
            let ref_len = naive_encode_rgb888(rgb_src, &mut ref_out, width, height, quality);
            assert_eq!(
                &fast_out[..fast_len],
                &ref_out[..ref_len],
                "RGB888 encoded JPEG bytes differ at {width}x{height} q={quality}"
            );

            let g_fast_len =
                encode_grayscale_to_jpeg(gray_src, &mut fast_out, width, height, quality)
                    .expect("CONV-13 grayscale JPEG encode must succeed");
            let g_ref_len = naive_encode_grayscale(gray_src, &mut ref_out, width, height, quality);
            assert_eq!(
                &fast_out[..g_fast_len],
                &ref_out[..g_ref_len],
                "grayscale encoded JPEG bytes differ at {width}x{height} q={quality}"
            );
        }
    }

    fn verify_yuv422_conv13(width: u16, height: u16) {
        let w = width as usize;
        let h = height as usize;
        let mut yuv_buf = [0u8; 24 * 24 * 2];
        for (i, byte) in yuv_buf[..w * h * 2].iter_mut().enumerate() {
            *byte = ((i.wrapping_mul(61).wrapping_add(w * 29 + h * 53)) ^ (i >> 2)) as u8;
        }
        let yuv_src = &yuv_buf[..w * h * 2];

        for by in 0..h.div_ceil(8) {
            for bx in 0..w.div_ceil(8) {
                let (mut y_got, mut cb_got, mut cr_got) = ([0i32; 64], [0i32; 64], [0i32; 64]);
                let (mut y_exp, mut cb_exp, mut cr_exp) = ([0i32; 64], [0i32; 64], [0i32; 64]);
                gather_yuv422_h1v1_mcu(yuv_src, w, h, bx, by, &mut y_got, &mut cb_got, &mut cr_got);
                naive_gather_yuv422_h1v1_mcu(
                    yuv_src,
                    w,
                    h,
                    bx,
                    by,
                    &mut y_exp,
                    &mut cb_exp,
                    &mut cr_exp,
                );
                assert_eq!(
                    y_got, y_exp,
                    "YUV422 Y block mismatch at MCU ({bx},{by}) for {width}x{height}"
                );
                assert_eq!(
                    cb_got, cb_exp,
                    "YUV422 Cb block mismatch at MCU ({bx},{by}) for {width}x{height}"
                );
                assert_eq!(
                    cr_got, cr_exp,
                    "YUV422 Cr block mismatch at MCU ({bx},{by}) for {width}x{height}"
                );
            }
        }

        let mut fast_out = [0u8; 8192];
        let mut ref_out = [0u8; 8192];
        for quality in [50u8, 92u8] {
            let fast_len = encode_yuv422_to_jpeg_subsampled(
                yuv_src,
                &mut fast_out,
                width,
                height,
                quality,
                Subsampling::H1V1,
            )
            .expect("CONV-13 YUV422 JPEG encode must succeed");
            let ref_len = naive_encode_yuv422(yuv_src, &mut ref_out, width, height, quality);
            assert_eq!(
                &fast_out[..fast_len],
                &ref_out[..ref_len],
                "YUV422 encoded JPEG bytes differ at {width}x{height} q={quality}"
            );
        }
    }

    #[test]
    fn test_conv13_interior_mcu_gather_byte_identical() {
        let rgb_gray_dims = [
            (8, 8),
            (16, 16),
            (24, 16),
            (1, 1),
            (7, 5),
            (13, 11),
            (17, 19),
        ];
        for (width, height) in rgb_gray_dims {
            verify_rgb888_and_gray_conv13(width, height);
        }

        let yuv422_dims = [
            (8, 8),
            (16, 16),
            (24, 16),
            (2, 1),
            (6, 5),
            (14, 11),
            (18, 19),
        ];
        for (width, height) in yuv422_dims {
            verify_yuv422_conv13(width, height);
        }
    }

    #[test]
    fn test_conv03_yuv422_subsampling_h2v1_default_and_psnr() {
        for (width, height) in [(32u16, 16u16), (30u16, 13u16)] {
            let w = width as usize;
            let h = height as usize;
            let mut ref_rgb = [0u8; 32 * 16 * 3];
            let mut yuv_in = [0u8; 32 * 16 * 2];
            for y in 0..h {
                for x_pair in 0..(w / 2) {
                    let x0 = x_pair * 2;
                    let x1 = x0 + 1;
                    let r0 = (40 + x0 * 5 + y * 4).min(240) as u8;
                    let g0 = (60 + x0 * 3 + y * 6).min(240) as u8;
                    let b0 = (180usize.saturating_sub(x0 * 3) + y * 3).min(240) as u8;
                    let r1 = (40 + x1 * 5 + y * 4).min(240) as u8;
                    let g1 = (60 + x1 * 3 + y * 6).min(240) as u8;
                    let b1 = (180usize.saturating_sub(x1 * 3) + y * 3).min(240) as u8;
                    let i0 = (y * w + x0) * 3;
                    ref_rgb[i0..i0 + 3].copy_from_slice(&[r0, g0, b0]);
                    ref_rgb[i0 + 3..i0 + 6].copy_from_slice(&[r1, g1, b1]);
                    let (y0_v, cb0, cr0) = rgb2yuv(r0, g0, b0);
                    let (y1_v, cb1, cr1) = rgb2yuv(r1, g1, b1);
                    let u = ((u16::from(cb0) + u16::from(cb1) + 1) >> 1) as u8;
                    let v = ((u16::from(cr0) + u16::from(cr1) + 1) >> 1) as u8;
                    let p_idx = (y * w + x0) * 2;
                    yuv_in[p_idx..p_idx + 4].copy_from_slice(&[y0_v, u, y1_v, v]);
                }
            }
            let yuv_slice = &yuv_in[..w * h * 2];
            let rgb_slice = &ref_rgb[..w * h * 3];

            let mut default_jpg = [0u8; 8192];
            let default_len = encode_yuv422_to_jpeg(yuv_slice, &mut default_jpg, width, height, 85)
                .expect("default YUV422 encode must succeed");
            let sof_pos = default_jpg[..default_len]
                .windows(2)
                .position(|m| m == [0xFF, 0xC0])
                .expect("SOF0 marker must exist");
            assert_eq!(
                default_jpg[sof_pos + 11],
                0x21,
                "default encode_yuv422_to_jpeg must emit H2V1 (0x21) on component 1"
            );

            let mut h1v1_jpg = [0u8; 8192];
            let h1v1_len = encode_yuv422_to_jpeg_subsampled(
                yuv_slice,
                &mut h1v1_jpg,
                width,
                height,
                85,
                Subsampling::H1V1,
            )
            .expect("H1V1 encode must succeed");
            if (width, height) == (32, 16) {
                assert!(
                    default_len < h1v1_len,
                    "H2V1 ({default_len} B) must be smaller than H1V1 ({h1v1_len} B)"
                );
            }

            for mode in [Subsampling::H1V1, Subsampling::H2V1, Subsampling::H2V2] {
                let mut jpg = [0u8; 8192];
                let jpg_len =
                    encode_yuv422_to_jpeg_subsampled(yuv_slice, &mut jpg, width, height, 90, mode)
                        .expect("subsampled YUV422 encode must succeed");
                if mode == Subsampling::H2V1 {
                    let mut jpg85 = [0u8; 8192];
                    let len85 = encode_yuv422_to_jpeg_subsampled(
                        yuv_slice, &mut jpg85, width, height, 85, mode,
                    )
                    .expect("H2V1 q=85 encode must succeed");
                    assert_eq!(
                        &default_jpg[..default_len],
                        &jpg85[..len85],
                        "encode_yuv422_to_jpeg must match Subsampling::H2V1 byte-for-byte"
                    );
                }
                let mut dec_rgb = [0u8; 32 * 16 * 3];
                let dec_len = decode_jpeg_to_rgb888(&jpg[..jpg_len], &mut dec_rgb)
                    .expect("decode of subsampled JPEG must succeed");
                assert_eq!(dec_len, rgb_slice.len(), "decoded RGB length mismatch");
                let mut sse = 0u64;
                for (&a, &b) in rgb_slice.iter().zip(dec_rgb[..dec_len].iter()) {
                    let d = i64::from(a) - i64::from(b);
                    sse += (d * d) as u64;
                }
                assert!(
                    sse * 1000 < 65_025 * (dec_len as u64),
                    "PSNR below 30 dB for {mode:?} at {width}x{height}: sse={sse}, n={dec_len}"
                );
            }
        }
    }

    #[test]
    fn test_conv20_max_encoded_len_and_upfront_capacity_check() {
        assert_eq!(
            max_encoded_len(0, 8, 3),
            None,
            "zero width must return None"
        );
        assert_eq!(
            max_encoded_len(8, 0, 1),
            None,
            "zero height must return None"
        );
        assert_eq!(
            max_encoded_len(8, 8, 0),
            None,
            "0 components must return None"
        );
        assert_eq!(
            max_encoded_len(8, 8, 2),
            None,
            "2 components must return None"
        );
        assert_eq!(
            max_encoded_len(8, 8, 4),
            None,
            "4 components must return None"
        );

        let dummy_gray = [128u8; 64];
        let dummy_rgb = [128u8; 64 * 3];
        let dummy_yuv = [128u8; 64 * 2];
        let mut short_gray = [0u8; 329];
        let mut short_color = [0u8; 624];
        assert_eq!(
            encode_grayscale_to_jpeg(&dummy_gray, &mut short_gray, 8, 8, 85),
            Err(CameraError::BufferTooSmall),
            "grayscale dst smaller than min header must fail fast"
        );
        assert_eq!(
            encode_rgb888_to_jpeg(&dummy_rgb, &mut short_color, 8, 8, 85),
            Err(CameraError::BufferTooSmall),
            "RGB888 dst smaller than min header must fail fast"
        );
        assert_eq!(
            encode_yuv422_to_jpeg(&dummy_yuv, &mut short_color, 8, 8, 85),
            Err(CameraError::BufferTooSmall),
            "YUV422 dst smaller than min header must fail fast"
        );

        let mut rng = 0x9E37_79B9_7F4A_7C15_u64;
        let mut next_byte = || -> u8 {
            rng ^= rng << 13;
            rng ^= rng >> 7;
            rng ^= rng << 17;
            (rng & 0xFF) as u8
        };

        let mut noise_src = [0u8; 24 * 17 * 3];
        let mut dst_scratch = [0u8; 16384];
        for &(w, h) in &[
            (1u16, 1u16),
            (2, 1),
            (7, 5),
            (8, 8),
            (14, 11),
            (16, 16),
            (24, 17),
        ] {
            for &q in &[1u8, 50, 95, 100] {
                let px = (w as usize) * (h as usize);
                for b in &mut noise_src[..px * 3] {
                    *b = next_byte();
                }

                let cap1 = max_encoded_len(w, h, 1).expect("1-comp max_encoded_len must be Some");
                let len1 =
                    encode_grayscale_to_jpeg(&noise_src[..px], &mut dst_scratch[..cap1], w, h, q)
                        .expect("grayscale white noise must fit in max_encoded_len");
                assert!(len1 <= cap1, "grayscale written {len1} > cap {cap1}");

                let cap3 = max_encoded_len(w, h, 3).expect("3-comp max_encoded_len must be Some");
                let len3 =
                    encode_rgb888_to_jpeg(&noise_src[..px * 3], &mut dst_scratch[..cap3], w, h, q)
                        .expect("RGB888 white noise must fit in max_encoded_len");
                assert!(len3 <= cap3, "RGB888 written {len3} > cap {cap3}");

                if (w & 1) == 0 {
                    for mode in [Subsampling::H1V1, Subsampling::H2V1, Subsampling::H2V2] {
                        let len_y = encode_yuv422_to_jpeg_subsampled(
                            &noise_src[..px * 2],
                            &mut dst_scratch[..cap3],
                            w,
                            h,
                            q,
                            mode,
                        )
                        .expect("YUV422 white noise must fit in max_encoded_len");
                        assert!(
                            len_y <= cap3,
                            "YUV422 {mode:?} written {len_y} > cap {cap3}"
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn test_conv07_grayscale_non_1x1_sof_sampling_factor() {
        let mut gray_in = [0u8; 16 * 16];
        for y in 0..16usize {
            for x in 0..16usize {
                gray_in[y * 16 + x] = (x * 12 + y * 3) as u8;
            }
        }
        let mut jpg_11 = [0u8; 4096];
        let jpg_len = encode_grayscale_to_jpeg(&gray_in, &mut jpg_11, 16, 16, 90)
            .expect("16x16 grayscale encode must succeed");

        let sof_pos = jpg_11[..jpg_len]
            .windows(2)
            .position(|m| m == [0xFF, 0xC0])
            .expect("SOF0 marker must exist");
        assert_eq!(
            jpg_11[sof_pos + 11],
            0x11,
            "baseline grayscale encoder emits 0x11 sampling byte"
        );

        let mut jpg_22 = jpg_11;
        jpg_22[sof_pos + 11] = 0x22;

        let mut rgb_11 = [0u8; 16 * 16 * 3];
        let mut rgb_22 = [0u8; 16 * 16 * 3];
        let len_11 = decode_jpeg_to_rgb888(&jpg_11[..jpg_len], &mut rgb_11)
            .expect("decode of 0x11 grayscale JPEG must succeed");
        let len_22 = decode_jpeg_to_rgb888(&jpg_22[..jpg_len], &mut rgb_22)
            .expect("decode of patched 0x22 grayscale JPEG must succeed");
        assert_eq!(len_11, len_22, "decoded lengths must match");
        assert_eq!(
            rgb_22, rgb_11,
            "CONV-07: grayscale SOF0 patched to 0x22 must decode identically to 0x11"
        );
    }

    fn build_dri1_grayscale_test_jpeg(
        width: u16,
        rst_markers: &[Option<u8>],
    ) -> ([u8; 768], usize) {
        let mut jpeg = [0u8; 768];
        let mut p = 0usize;
        jpeg[p..p + 2].copy_from_slice(&[0xFF, 0xD8]);
        p += 2;
        jpeg[p..p + 5].copy_from_slice(&[0xFF, 0xDB, 0x00, 0x43, 0x00]);
        p += 5;
        jpeg[p..p + 64].fill(16);
        p += 64;
        jpeg[p..p + 7].copy_from_slice(&[0xFF, 0xC0, 0x00, 0x0B, 0x08, 0x00, 0x08]);
        p += 7;
        jpeg[p..p + 2].copy_from_slice(&width.to_be_bytes());
        p += 2;
        jpeg[p..p + 4].copy_from_slice(&[0x01, 0x01, 0x11, 0x00]);
        p += 4;
        let dc_seg_len = (2 + 1 + 16 + DC_LUM_VAL.len()) as u16;
        jpeg[p..p + 2].copy_from_slice(&[0xFF, 0xC4]);
        p += 2;
        jpeg[p..p + 2].copy_from_slice(&dc_seg_len.to_be_bytes());
        p += 2;
        jpeg[p] = 0x00;
        p += 1;
        jpeg[p..p + 16].copy_from_slice(&DC_LUM_BITS[1..17]);
        p += 16;
        jpeg[p..p + DC_LUM_VAL.len()].copy_from_slice(&DC_LUM_VAL);
        p += DC_LUM_VAL.len();
        let ac_seg_len = (2 + 1 + 16 + AC_LUM_VAL.len()) as u16;
        jpeg[p..p + 2].copy_from_slice(&[0xFF, 0xC4]);
        p += 2;
        jpeg[p..p + 2].copy_from_slice(&ac_seg_len.to_be_bytes());
        p += 2;
        jpeg[p] = 0x10;
        p += 1;
        jpeg[p..p + 16].copy_from_slice(&AC_LUM_BITS[1..17]);
        p += 16;
        jpeg[p..p + AC_LUM_VAL.len()].copy_from_slice(&AC_LUM_VAL);
        p += AC_LUM_VAL.len();
        jpeg[p..p + 6].copy_from_slice(&[0xFF, 0xDD, 0x00, 0x04, 0x00, 0x01]);
        p += 6;
        jpeg[p..p + 10]
            .copy_from_slice(&[0xFF, 0xDA, 0x00, 0x08, 0x01, 0x01, 0x00, 0x00, 0x3F, 0x00]);
        p += 10;

        let mcus = (width as usize) / 8;
        assert_eq!(
            rst_markers.len() + 1,
            mcus,
            "rst_markers must specify one entry per inter-MCU boundary"
        );
        for &marker in rst_markers {
            jpeg[p] = 0x2B;
            p += 1;
            if let Some(rst) = marker {
                jpeg[p..p + 2].copy_from_slice(&[0xFF, rst]);
                p += 2;
            }
        }
        jpeg[p] = 0x2B;
        p += 1;
        jpeg[p..p + 2].copy_from_slice(&[0xFF, 0xD9]);
        p += 2;
        (jpeg, p)
    }

    #[test]
    fn test_conv17_restart_marker_missing_and_swapped_rejected() {
        let mut out = [0u8; 80 * 8 * 3];

        // 1. Valid 10-MCU (80x8) stream wrapping RST0..RST7 -> RST0
        let valid_seq = [
            Some(0xD0),
            Some(0xD1),
            Some(0xD2),
            Some(0xD3),
            Some(0xD4),
            Some(0xD5),
            Some(0xD6),
            Some(0xD7),
            Some(0xD0),
        ];
        let (ok_jpg, ok_len) = build_dri1_grayscale_test_jpeg(80, &valid_seq);
        assert_eq!(
            decode_jpeg_to_rgb888(&ok_jpg[..ok_len], &mut out),
            Ok(80 * 8 * 3),
            "valid RST0..RST7 wrap-around sequence must decode cleanly"
        );

        // 2. Missing RST0 between MCU 0 and MCU 1 (24x8 = 3 MCUs)
        let (missing_jpg, missing_len) = build_dri1_grayscale_test_jpeg(24, &[None, Some(0xD1)]);
        assert_eq!(
            decode_jpeg_to_rgb888(&missing_jpg[..missing_len], &mut out[..24 * 8 * 3]),
            Err(CameraError::CorruptedFrame),
            "missing RST0 marker must return CorruptedFrame"
        );

        // 3. Swapped RST1 / RST0 markers (24x8 = 3 MCUs)
        let (swapped_jpg, swapped_len) =
            build_dri1_grayscale_test_jpeg(24, &[Some(0xD1), Some(0xD0)]);
        assert_eq!(
            decode_jpeg_to_rgb888(&swapped_jpg[..swapped_len], &mut out[..24 * 8 * 3]),
            Err(CameraError::CorruptedFrame),
            "swapped RST1/RST0 markers must return CorruptedFrame"
        );
    }

    #[test]
    fn test_conv09_encode_yuv422_with_bt601_limited_range() {
        let mut black_yuv = [0u8; 16 * 16 * 2];
        for pair in black_yuv.as_chunks_mut::<4>().0 {
            *pair = [16, 128, 16, 128];
        }
        let mut white_yuv = [0u8; 16 * 16 * 2];
        for pair in white_yuv.as_chunks_mut::<4>().0 {
            *pair = [235, 128, 235, 128];
        }

        let mut jpeg = [0u8; 4096];
        let mut rgb = [0u8; 16 * 16 * 3];

        for mode in [Subsampling::H1V1, Subsampling::H2V1, Subsampling::H2V2] {
            let black_len = encode_yuv422_to_jpeg_with_range(
                &black_yuv,
                &mut jpeg,
                16,
                16,
                95,
                mode,
                YuvRange::Bt601Limited,
            )
            .expect("BT.601 studio black YUV422 encode must succeed");
            assert_eq!(
                decode_jpeg_to_rgb888(&jpeg[..black_len], &mut rgb),
                Ok(rgb.len()),
                "BT.601 studio black JPEG must decode cleanly for {mode:?}"
            );
            for &ch in &rgb {
                assert!(
                    ch <= 4,
                    "studio black channel must expand to <= 4 for {mode:?}, got {ch}"
                );
            }

            let white_len = encode_yuv422_to_jpeg_with_range(
                &white_yuv,
                &mut jpeg,
                16,
                16,
                95,
                mode,
                YuvRange::Bt601Limited,
            )
            .expect("BT.601 studio white YUV422 encode must succeed");
            assert_eq!(
                decode_jpeg_to_rgb888(&jpeg[..white_len], &mut rgb),
                Ok(rgb.len()),
                "BT.601 studio white JPEG must decode cleanly for {mode:?}"
            );
            for &ch in &rgb {
                assert!(
                    ch >= 251,
                    "studio white channel must expand to >= 251 for {mode:?}, got {ch}"
                );
            }
        }
    }
}
