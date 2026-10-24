//! Pure-Rust `#![no_std]` baseline DCT JPEG encoder and decoder.
//!
//! Replaces `conversions/jpge.cpp`, `conversions/to_jpg.cpp`, and `conversions/esp_jpg_decode.c`
//! with a zero-allocation, stack-only implementation (`#![forbid(unsafe_code)]`).

use crate::bmp::{BMP_HEADER_LEN, write_bmp_header};
use crate::yuv::rgb2yuv;
use esp32_cam_types::CameraError;

/// Full-range JFIF YCbCr to RGB888 conversion (inverse of `jpge.cpp`'s `RGB_to_YCC`).
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

/// Standard luminance quantization table in zig-zag order (`s_std_lum_quant` from `jpge.cpp:46`).
const STD_LUM_QUANT: [i32; 64] = [
    16, 11, 12, 14, 12, 10, 16, 14, 13, 14, 18, 17, 16, 19, 24, 40, 26, 24, 22, 22, 24, 49, 35, 37,
    29, 40, 58, 51, 61, 60, 57, 51, 56, 55, 64, 72, 92, 78, 64, 68, 87, 69, 55, 56, 80, 109, 81,
    87, 95, 98, 103, 104, 103, 62, 77, 113, 121, 112, 100, 120, 92, 101, 103, 99,
];

/// Standard chrominance quantization table in zig-zag order (`s_std_croma_quant` from `jpge.cpp:47`).
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

/// Performs the 8x8 forward 2D integer DCT matching `DCT2D` in `conversions/jpge.cpp:134-148`.
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

/// Performs the 8x8 inverse 2D integer IDCT (IJG `jidctint.c` LLM algorithm) and level-shifts
/// output samples by `+128` into `0..=255`.
fn idct_2d(coeffs: &[i32; 64], out_pixels: &mut [u8; 64]) {
    let mut workspace = [0i32; 64];

    // Pass 1: process rows
    for row in 0..8 {
        let base = row * 8;
        let f0 = coeffs[base];
        let f1 = coeffs[base + 1];
        let f2 = coeffs[base + 2];
        let f3 = coeffs[base + 3];
        let f4 = coeffs[base + 4];
        let f5 = coeffs[base + 5];
        let f6 = coeffs[base + 6];
        let f7 = coeffs[base + 7];

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

        workspace[base] = dct_descale(tmp10 + otmp3, CONST_BITS - ROW_BITS);
        workspace[base + 7] = dct_descale(tmp10 - otmp3, CONST_BITS - ROW_BITS);
        workspace[base + 1] = dct_descale(tmp11 + otmp2, CONST_BITS - ROW_BITS);
        workspace[base + 6] = dct_descale(tmp11 - otmp2, CONST_BITS - ROW_BITS);
        workspace[base + 2] = dct_descale(tmp12 + otmp1, CONST_BITS - ROW_BITS);
        workspace[base + 5] = dct_descale(tmp12 - otmp1, CONST_BITS - ROW_BITS);
        workspace[base + 3] = dct_descale(tmp13 + otmp0, CONST_BITS - ROW_BITS);
        workspace[base + 4] = dct_descale(tmp13 - otmp0, CONST_BITS - ROW_BITS);
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

        let shift = CONST_BITS + ROW_BITS + 3;
        out_pixels[col] = (dct_descale(tmp10 + otmp3, shift) + 128).clamp(0, 255) as u8;
        out_pixels[col + 56] = (dct_descale(tmp10 - otmp3, shift) + 128).clamp(0, 255) as u8;
        out_pixels[col + 8] = (dct_descale(tmp11 + otmp2, shift) + 128).clamp(0, 255) as u8;
        out_pixels[col + 48] = (dct_descale(tmp11 - otmp2, shift) + 128).clamp(0, 255) as u8;
        out_pixels[col + 16] = (dct_descale(tmp12 + otmp1, shift) + 128).clamp(0, 255) as u8;
        out_pixels[col + 40] = (dct_descale(tmp12 - otmp1, shift) + 128).clamp(0, 255) as u8;
        out_pixels[col + 24] = (dct_descale(tmp13 + otmp0, shift) + 128).clamp(0, 255) as u8;
        out_pixels[col + 32] = (dct_descale(tmp13 - otmp0, shift) + 128).clamp(0, 255) as u8;
    }
}

fn compute_quant_table(base: &[i32; 64], quality: u8) -> [i32; 64] {
    let q_clamped = quality.clamp(1, 100) as i32;
    let scale = if q_clamped < 50 {
        5000 / q_clamped
    } else {
        200 - q_clamped * 2
    };
    let mut out = [0i32; 64];
    for i in 0..64 {
        let v = (base[i] * scale + 50) / 100;
        out[i] = v.clamp(1, 255);
    }
    out
}

struct EncHuffTable {
    codes: [u16; 256],
    sizes: [u8; 256],
}

fn build_enc_huff_table(bits: &[u8; 17], vals: &[u8]) -> EncHuffTable {
    let mut huff_size = [0u8; 257];
    let mut huff_code = [0u16; 257];

    let mut p = 0usize;
    for (l, &b) in bits.iter().enumerate().take(17).skip(1) {
        let count = b as usize;
        for _ in 0..count {
            huff_size[p] = l as u8;
            p += 1;
        }
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
    for i in 0..last_p {
        let sym = vals[i] as usize;
        table.codes[sym] = huff_code[i];
        table.sizes[sym] = huff_size[i];
    }
    table
}

struct JpegBitWriter<'a> {
    dst: &'a mut [u8],
    pos: usize,
    bit_buf: u64,
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
        if self.pos >= self.dst.len() {
            return Err(CameraError::BufferTooSmall);
        }
        self.dst[self.pos] = b;
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
        let mask = (1u64 << len) - 1;
        self.bit_buf = (self.bit_buf << len) | ((bits as u64) & mask);
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

        // SOF0 (Baseline DCT, 1x1 H1V1 sampling for maximum sharpness and zero scanline heap buffer)
        self.emit_marker(M_SOF0)?;
        self.emit_word(8 + 3 * (num_components as u16))?;
        self.emit_byte(8)?;
        self.emit_word(height)?;
        self.emit_word(width)?;
        self.emit_byte(num_components)?;
        for c in 0..num_components {
            self.emit_byte(c + 1)?;
            self.emit_byte(0x11)?; // H1V1
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
        qtable: &[i32; 64],
        dc_huff: &EncHuffTable,
        ac_huff: &EncHuffTable,
        last_dc: &mut i32,
    ) -> Result<(), CameraError> {
        fdct_2d(sample_block);

        let mut zz_coeffs = [0i32; 64];
        for i in 0..64 {
            let raw = sample_block[ZAG[i]];
            let q = qtable[i];
            zz_coeffs[i] = if raw < 0 {
                let adj = -raw + (q >> 1);
                if adj < q { 0 } else { -(adj / q) }
            } else {
                let adj = raw + (q >> 1);
                if adj < q { 0 } else { adj / q }
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
        self.put_bits(dc_huff.codes[sym] as u32, dc_huff.sizes[sym] as u32)?;
        if nbits > 0 {
            self.put_bits(dc_bits & ((1u32 << nbits) - 1), nbits)?;
        }

        // AC coefficients RLE + Huffman encoding
        let mut run_len = 0u32;
        for &ac in &zz_coeffs[1..64] {
            if ac == 0 {
                run_len += 1;
            } else {
                while run_len >= 16 {
                    self.put_bits(ac_huff.codes[0xF0] as u32, ac_huff.sizes[0xF0] as u32)?;
                    run_len -= 16;
                }
                let (abs_ac, ac_bits) = if ac < 0 {
                    ((-ac) as u32, (ac - 1) as u32)
                } else {
                    (ac as u32, ac as u32)
                };
                let ac_nbits = 32 - abs_ac.leading_zeros();
                let ac_sym = ((run_len << 4) | ac_nbits) as usize;
                self.put_bits(ac_huff.codes[ac_sym] as u32, ac_huff.sizes[ac_sym] as u32)?;
                self.put_bits(ac_bits & ((1u32 << ac_nbits) - 1), ac_nbits)?;
                run_len = 0;
            }
        }
        if run_len > 0 {
            self.put_bits(ac_huff.codes[0x00] as u32, ac_huff.sizes[0x00] as u32)?;
        }

        Ok(())
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

/// Encodes a 24-bit RGB888 image (`[R, G, B]`) into a baseline JFIF JPEG stream in `dst`.
///
/// Performs zero heap allocations and returns the total JPEG byte length written to `dst`.
pub fn encode_rgb888_to_jpeg(
    src: &[u8],
    dst: &mut [u8],
    width: u16,
    height: u16,
    quality: u8,
) -> Result<usize, CameraError> {
    validate_encode_dims(src.len(), width, height, 3)?;

    let lum_q = compute_quant_table(&STD_LUM_QUANT, quality);
    let chroma_q = compute_quant_table(&STD_CHROMA_QUANT, quality);
    let dc_lum = build_enc_huff_table(&DC_LUM_BITS, &DC_LUM_VAL);
    let ac_lum = build_enc_huff_table(&AC_LUM_BITS, &AC_LUM_VAL);
    let dc_chroma = build_enc_huff_table(&DC_CHROMA_BITS, &DC_CHROMA_VAL);
    let ac_chroma = build_enc_huff_table(&AC_CHROMA_BITS, &AC_CHROMA_VAL);

    let mut writer = JpegBitWriter::new(dst);
    writer.emit_headers(width, height, 3, &lum_q, &chroma_q)?;

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
            for dy in 0..8usize {
                let py = (by * 8 + dy).min(h - 1);
                for dx in 0..8usize {
                    let px = (bx * 8 + dx).min(w - 1);
                    let p_idx = (py * w + px) * 3;
                    let (y, cb, cr) = rgb2yuv(src[p_idx], src[p_idx + 1], src[p_idx + 2]);
                    let b_idx = dy * 8 + dx;
                    y_block[b_idx] = (y as i32) - 128;
                    cb_block[b_idx] = (cb as i32) - 128;
                    cr_block[b_idx] = (cr as i32) - 128;
                }
            }

            writer.encode_block(&mut y_block, &lum_q, &dc_lum, &ac_lum, &mut last_dc[0])?;
            writer.encode_block(
                &mut cb_block,
                &chroma_q,
                &dc_chroma,
                &ac_chroma,
                &mut last_dc[1],
            )?;
            writer.encode_block(
                &mut cr_block,
                &chroma_q,
                &dc_chroma,
                &ac_chroma,
                &mut last_dc[2],
            )?;
        }
    }

    writer.flush_bits()?;
    writer.emit_marker(M_EOI)?;
    Ok(writer.pos)
}

/// Encodes an 8-bit grayscale image into a single-component baseline JFIF JPEG stream in `dst`.
///
/// Performs zero heap allocations and returns the total JPEG byte length written to `dst`.
pub fn encode_grayscale_to_jpeg(
    src: &[u8],
    dst: &mut [u8],
    width: u16,
    height: u16,
    quality: u8,
) -> Result<usize, CameraError> {
    validate_encode_dims(src.len(), width, height, 1)?;

    let lum_q = compute_quant_table(&STD_LUM_QUANT, quality);
    let dummy_chroma_q = [1i32; 64];
    let dc_lum = build_enc_huff_table(&DC_LUM_BITS, &DC_LUM_VAL);
    let ac_lum = build_enc_huff_table(&AC_LUM_BITS, &AC_LUM_VAL);

    let mut writer = JpegBitWriter::new(dst);
    writer.emit_headers(width, height, 1, &lum_q, &dummy_chroma_q)?;

    let w = width as usize;
    let h = height as usize;
    let mcu_cols = w.div_ceil(8);
    let mcu_rows = h.div_ceil(8);

    let mut last_dc = 0i32;
    let mut y_block = [0i32; 64];

    for by in 0..mcu_rows {
        for bx in 0..mcu_cols {
            for dy in 0..8usize {
                let py = (by * 8 + dy).min(h - 1);
                for dx in 0..8usize {
                    let px = (bx * 8 + dx).min(w - 1);
                    y_block[dy * 8 + dx] = (src[py * w + px] as i32) - 128;
                }
            }
            writer.encode_block(&mut y_block, &lum_q, &dc_lum, &ac_lum, &mut last_dc)?;
        }
    }

    writer.flush_bits()?;
    writer.emit_marker(M_EOI)?;
    Ok(writer.pos)
}

/// Encodes a packed YUV422 (YUYV: `[Y0, U, Y1, V]`) image into a baseline JFIF JPEG stream in `dst`.
///
/// Performs zero heap allocations and returns the total JPEG byte length written to `dst`.
pub fn encode_yuv422_to_jpeg(
    src: &[u8],
    dst: &mut [u8],
    width: u16,
    height: u16,
    quality: u8,
) -> Result<usize, CameraError> {
    validate_encode_dims(src.len(), width, height, 2)?;

    let lum_q = compute_quant_table(&STD_LUM_QUANT, quality);
    let chroma_q = compute_quant_table(&STD_CHROMA_QUANT, quality);
    let dc_lum = build_enc_huff_table(&DC_LUM_BITS, &DC_LUM_VAL);
    let ac_lum = build_enc_huff_table(&AC_LUM_BITS, &AC_LUM_VAL);
    let dc_chroma = build_enc_huff_table(&DC_CHROMA_BITS, &DC_CHROMA_VAL);
    let ac_chroma = build_enc_huff_table(&AC_CHROMA_BITS, &AC_CHROMA_VAL);

    let mut writer = JpegBitWriter::new(dst);
    writer.emit_headers(width, height, 3, &lum_q, &chroma_q)?;

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
            for dy in 0..8usize {
                let py = (by * 8 + dy).min(h - 1);
                for dx in 0..8usize {
                    let px = (bx * 8 + dx).min(w - 1);
                    let pair_idx = (py * w + (px & !1)) * 2;
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
                    y_block[b_idx] = (y as i32) - 128;
                    cb_block[b_idx] = (u as i32) - 128;
                    cr_block[b_idx] = (v as i32) - 128;
                }
            }

            writer.encode_block(&mut y_block, &lum_q, &dc_lum, &ac_lum, &mut last_dc[0])?;
            writer.encode_block(
                &mut cb_block,
                &chroma_q,
                &dc_chroma,
                &ac_chroma,
                &mut last_dc[1],
            )?;
            writer.encode_block(
                &mut cr_block,
                &chroma_q,
                &dc_chroma,
                &ac_chroma,
                &mut last_dc[2],
            )?;
        }
    }

    writer.flush_bits()?;
    writer.emit_marker(M_EOI)?;
    Ok(writer.pos)
}

#[derive(Clone, Copy)]
struct DecHuffTable {
    valid: bool,
    max_code: [i32; 17],
    val_offset: [i32; 17],
    vals: [u8; 256],
}

impl Default for DecHuffTable {
    fn default() -> Self {
        Self {
            valid: false,
            max_code: [-1; 17],
            val_offset: [0; 17],
            vals: [0; 256],
        }
    }
}

fn build_dec_huff_table(bits: &[u8; 17], vals: &[u8]) -> Result<DecHuffTable, CameraError> {
    let mut table = DecHuffTable {
        valid: true,
        ..DecHuffTable::default()
    };
    let mut code = 0i32;
    let mut k = 0usize;
    for (l, &b) in bits.iter().enumerate().take(17).skip(1) {
        let count = b as usize;
        if k + count > 256 || k + count > vals.len() {
            return Err(CameraError::CodecError);
        }
        if count == 0 {
            table.max_code[l] = -1;
        } else {
            table.val_offset[l] = (k as i32) - code;
            for _ in 0..count {
                table.vals[k] = vals[k];
                k += 1;
            }
            code += count as i32;
            table.max_code[l] = code - 1;
        }
        code <<= 1;
    }
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
    bit_buf: u64,
    bits_in: u32,
    hit_marker: bool,
}

impl<'a> JpegBitReader<'a> {
    fn new(src: &'a [u8], start_pos: usize) -> Self {
        Self {
            src,
            pos: start_pos,
            bit_buf: 0,
            bits_in: 0,
            hit_marker: false,
        }
    }

    fn fill_bits(&mut self, min_bits: u32) -> Result<(), CameraError> {
        while self.bits_in < min_bits {
            if self.hit_marker || self.pos >= self.src.len() {
                // Pad with 1-bits if we reached EOI marker at the end of a valid scan
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
                self.bit_buf = (self.bit_buf << 8) | (b as u64);
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
        self.fill_bits(n)?;
        self.bits_in -= n;
        let val = ((self.bit_buf >> self.bits_in) & ((1u64 << n) - 1)) as u32;
        Ok(val)
    }

    fn decode_huff(&mut self, table: &DecHuffTable) -> Result<u8, CameraError> {
        if !table.valid {
            return Err(CameraError::CodecError);
        }
        let mut code = 0i32;
        for l in 1..=16usize {
            let bit = self.get_bits(1)? as i32;
            code = (code << 1) | bit;
            if code <= table.max_code[l] {
                let idx = (code + table.val_offset[l]) as usize;
                if idx >= 256 {
                    return Err(CameraError::CodecError);
                }
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
        qtable: &[i32; 64],
        dc_huff: &DecHuffTable,
        ac_huff: &DecHuffTable,
        last_dc: &mut i32,
        out_pixels: &mut [u8; 64],
    ) -> Result<(), CameraError> {
        let mut coeffs = [0i32; 64];

        let dc_nbits = self.decode_huff(dc_huff)? as u32;
        let dc_diff = self.receive_extend(dc_nbits)?;
        let dc_val = last_dc.wrapping_add(dc_diff);
        *last_dc = dc_val;
        coeffs[0] = dc_val * qtable[0];

        let mut k = 1usize;
        while k < 64 {
            let rs = self.decode_huff(ac_huff)?;
            let r = (rs >> 4) as usize;
            let s = (rs & 0x0F) as u32;

            if s == 0 {
                if r == 15 {
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
            coeffs[ZAG[k]] = ac_val * qtable[k];
            k += 1;
        }

        idct_2d(&coeffs, out_pixels);
        Ok(())
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum DecodeOutputFormat {
    Rgb888,
    Rgb565,
    Bmp24,
}

fn decode_jpeg_internal(
    src: &[u8],
    dst: &mut [u8],
    out_fmt: DecodeOutputFormat,
) -> Result<(u16, u16, usize), CameraError> {
    if src.len() < 4 || src[0] != 0xFF || src[1] != M_SOI {
        return Err(CameraError::CorruptedFrame);
    }

    let mut qtables = [[0i32; 64]; 4];
    let mut qtable_valid = [false; 4];
    let mut dc_tables = [DecHuffTable::default(); 4];
    let mut ac_tables = [DecHuffTable::default(); 4];
    let mut comps = [ComponentSpec::default(); 3];
    let mut num_components = 0usize;
    let mut width = 0u16;
    let mut height = 0u16;
    let mut restart_interval = 0u16;
    let mut sos_payload_pos = None;

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
        if seg_len < 2 || pos + seg_len > src.len() {
            return Err(CameraError::CorruptedFrame);
        }
        let seg_end = pos + seg_len;
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
                    for i in 0..64 {
                        qtables[t_idx][i] = src[cur + i] as i32;
                    }
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
                    if t_idx >= 4 {
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
                    let table = build_dec_huff_table(&bits, &src[cur..cur + total_syms])?;
                    if is_ac {
                        ac_tables[t_idx] = table;
                    } else {
                        dc_tables[t_idx] = table;
                    }
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
                for comp in comps.iter_mut().take(num_components) {
                    let id = src[cur];
                    let samp = src[cur + 1];
                    let q_idx = src[cur + 2];
                    cur += 3;
                    let h_samp = samp >> 4;
                    let v_samp = samp & 0x0F;
                    if h_samp == 0 || h_samp > 2 || v_samp == 0 || v_samp > 2 || q_idx >= 4 {
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
                if num_components == 0 || cur + 1 > seg_end {
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
                    if dc_idx >= 4 || ac_idx >= 4 {
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
        dst[BMP_HEADER_LEN..required_dst_len].fill(0);
    }

    for comp in comps.iter().take(num_components) {
        if !qtable_valid[comp.quant_idx as usize] {
            return Err(CameraError::CodecError);
        }
    }

    let max_h_samp = comps[0].h_samp as usize;
    let max_v_samp = comps[0].v_samp as usize;
    let mcu_w = max_h_samp * 8;
    let mcu_h = max_v_samp * 8;
    let w = width as usize;
    let h = height as usize;
    let mcu_cols = w.div_ceil(mcu_w);
    let mcu_rows = h.div_ceil(mcu_h);

    let mut reader = JpegBitReader::new(src, scan_start);
    let mut last_dc = [0i32; 3];
    let mut mcu_counter = 0u16;

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
                if reader.pos + 2 <= src.len()
                    && src[reader.pos] == 0xFF
                    && (0xD0..=0xD7).contains(&src[reader.pos + 1])
                {
                    reader.pos += 2;
                }
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
                            let d_idx = (img_y * w + img_x) * 3;
                            dst[d_idx] = r;
                            dst[d_idx + 1] = g;
                            dst[d_idx + 2] = b;
                        }
                        DecodeOutputFormat::Rgb565 => {
                            let d_idx = (img_y * w + img_x) * 2;
                            let hb = (r & 0xF8) | (g >> 5);
                            let lb = ((g & 0x1C) << 3) | (b >> 3);
                            dst[d_idx] = hb;
                            dst[d_idx + 1] = lb;
                        }
                        DecodeOutputFormat::Bmp24 => {
                            let d_idx = BMP_HEADER_LEN + img_y * bmp_row_stride + img_x * 3;
                            dst[d_idx] = b;
                            dst[d_idx + 1] = g;
                            dst[d_idx + 2] = r;
                        }
                    }
                }
            }

            mcu_counter = mcu_counter.wrapping_add(1);
        }
    }

    Ok((width, height, required_dst_len))
}

/// Decodes a baseline JPEG image from `src` into 24-bit RGB888 (`[R, G, B]`) in `dst`.
///
/// Performs zero heap allocations and returns the number of bytes written to `dst`
/// (`width * height * 3`).
pub fn decode_jpeg_to_rgb888(src: &[u8], dst: &mut [u8]) -> Result<usize, CameraError> {
    let (_w, _h, written) = decode_jpeg_internal(src, dst, DecodeOutputFormat::Rgb888)?;
    Ok(written)
}

/// Decodes a baseline JPEG image from `src` into 16-bit big-endian RGB565 in `dst`.
///
/// Performs zero heap allocations and returns the number of bytes written to `dst`
/// (`width * height * 2`).
pub fn decode_jpeg_to_rgb565(src: &[u8], dst: &mut [u8]) -> Result<usize, CameraError> {
    let (_w, _h, written) = decode_jpeg_internal(src, dst, DecodeOutputFormat::Rgb565)?;
    Ok(written)
}

/// Decodes a baseline JPEG image from `src` directly into a 24-bit BMP image in `dst`.
pub(crate) fn jpeg_to_bmp(src: &[u8], dst: &mut [u8]) -> Result<usize, CameraError> {
    let (_w, _h, written) = decode_jpeg_internal(src, dst, DecodeOutputFormat::Bmp24)?;
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
}
