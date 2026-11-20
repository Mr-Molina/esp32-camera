//! AES-128, CBC, CMAC (OMAC1), Key Wrap, and GMAC implementations.
//!
//! Compliant with FIPS-197, RFC 3602, RFC 4493, RFC 3394, and NIST SP 800-38D.

use super::hash::{ct_eq, zeroize_bytes, zeroize_u32s};

/// Error type for AES operations.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AesError;

const SBOX: [u8; 256] = [
    0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab, 0x76,
    0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0, 0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0,
    0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15,
    0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2, 0xeb, 0x27, 0xb2, 0x75,
    0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0, 0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84,
    0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf,
    0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45, 0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8,
    0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5, 0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2,
    0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73,
    0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88, 0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb,
    0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c, 0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79,
    0xe7, 0xc8, 0x37, 0x6d, 0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08,
    0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a,
    0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e, 0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e,
    0xe1, 0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf,
    0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb, 0x16,
];

const INV_SBOX: [u8; 256] = [
    0x52, 0x09, 0x6a, 0xd5, 0x30, 0x36, 0xa5, 0x38, 0xbf, 0x40, 0xa3, 0x9e, 0x81, 0xf3, 0xd7, 0xfb,
    0x7c, 0xe3, 0x39, 0x82, 0x9b, 0x2f, 0xff, 0x87, 0x34, 0x8e, 0x43, 0x44, 0xc4, 0xde, 0xe9, 0xcb,
    0x54, 0x7b, 0x94, 0x32, 0xa6, 0xc2, 0x23, 0x3d, 0xee, 0x4c, 0x95, 0x0b, 0x42, 0xfa, 0xc3, 0x4e,
    0x08, 0x2e, 0xa1, 0x66, 0x28, 0xd9, 0x24, 0xb2, 0x76, 0x5b, 0xa2, 0x49, 0x6d, 0x8b, 0xd1, 0x25,
    0x72, 0xf8, 0xf6, 0x64, 0x86, 0x68, 0x98, 0x16, 0xd4, 0xa4, 0x5c, 0xcc, 0x5d, 0x65, 0xb6, 0x92,
    0x6c, 0x70, 0x48, 0x50, 0xfd, 0xed, 0xb9, 0xda, 0x5e, 0x15, 0x46, 0x57, 0xa7, 0x8d, 0x9d, 0x84,
    0x90, 0xd8, 0xab, 0x00, 0x8c, 0xbc, 0xd3, 0x0a, 0xf7, 0xe4, 0x58, 0x05, 0xb8, 0xb3, 0x45, 0x06,
    0xd0, 0x2c, 0x1e, 0x8f, 0xca, 0x3f, 0x0f, 0x02, 0xc1, 0xaf, 0xbd, 0x03, 0x01, 0x13, 0x8a, 0x6b,
    0x3a, 0x91, 0x11, 0x41, 0x4f, 0x67, 0xdc, 0xea, 0x97, 0xf2, 0xcf, 0xce, 0xf0, 0xb4, 0xe6, 0x73,
    0x96, 0xac, 0x74, 0x22, 0xe7, 0xad, 0x35, 0x85, 0xe2, 0xf9, 0x37, 0xe8, 0x1c, 0x75, 0xdf, 0x6e,
    0x47, 0xf1, 0x1a, 0x71, 0x1d, 0x29, 0xc5, 0x89, 0x6f, 0xb7, 0x62, 0x0e, 0xaa, 0x18, 0xbe, 0x1b,
    0xfc, 0x56, 0x3e, 0x4b, 0xc6, 0xd2, 0x79, 0x20, 0x9a, 0xdb, 0xc0, 0xfe, 0x78, 0xcd, 0x5a, 0xf4,
    0x1f, 0xdd, 0xa8, 0x33, 0x88, 0x07, 0xc7, 0x31, 0xb1, 0x12, 0x10, 0x59, 0x27, 0x80, 0xec, 0x5f,
    0x60, 0x51, 0x7f, 0xa9, 0x19, 0xb5, 0x4a, 0x0d, 0x2d, 0xe5, 0x7a, 0x9f, 0x93, 0xc9, 0x9c, 0xef,
    0xa0, 0xe0, 0x3b, 0x4d, 0xae, 0x2a, 0xf5, 0xb0, 0xc8, 0xeb, 0xbb, 0x3c, 0x83, 0x53, 0x99, 0x61,
    0x17, 0x2b, 0x04, 0x7e, 0xba, 0x77, 0xd6, 0x26, 0xe1, 0x69, 0x14, 0x63, 0x55, 0x21, 0x0c, 0x7d,
];

const RCON: [u8; 10] = [0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1b, 0x36];

fn gmul2(x: u8) -> u8 {
    (x << 1) ^ (0u8.wrapping_sub(x >> 7) & 0x1b)
}

fn gmul(b: u8, x: u8) -> u8 {
    let g2 = gmul2(x);
    let g4 = gmul2(g2);
    let g8 = gmul2(g4);
    match b {
        2 => g2,
        3 => g2 ^ x,
        9 => g8 ^ x,
        11 => g8 ^ g2 ^ x,
        13 => g8 ^ g4 ^ x,
        14 => g8 ^ g4 ^ g2,
        _ => 0,
    }
}

/// AES-128 expanded key schedule.
pub struct Aes128Key {
    round_keys: [u32; 44],
}

impl Drop for Aes128Key {
    fn drop(&mut self) {
        zeroize_u32s(&mut self.round_keys);
    }
}

impl Aes128Key {
    /// Expands a 128-bit key into round keys.
    pub fn new(key: &[u8; 16]) -> Self {
        let mut w = [0u32; 44];
        for i in 0..4 {
            w[i] = u32::from_be_bytes([key[i * 4], key[i * 4 + 1], key[i * 4 + 2], key[i * 4 + 3]]);
        }
        for i in 4..44 {
            let mut temp = w[i - 1];
            if i % 4 == 0 {
                temp = temp.rotate_left(8);
                let b0 = SBOX[((temp >> 24) & 0xff) as usize];
                let b1 = SBOX[((temp >> 16) & 0xff) as usize];
                let b2 = SBOX[((temp >> 8) & 0xff) as usize];
                let b3 = SBOX[(temp & 0xff) as usize];
                temp = (u32::from(b0) << 24)
                    | (u32::from(b1) << 16)
                    | (u32::from(b2) << 8)
                    | u32::from(b3);
                temp ^= u32::from(RCON[(i / 4) - 1]) << 24;
            }
            w[i] = w[i - 4] ^ temp;
        }
        Self { round_keys: w }
    }

    /// Encrypts a single 16-byte block.
    pub fn encrypt_block(&self, block: &[u8; 16], out: &mut [u8; 16]) {
        let mut s = *block;
        let mut new_s = [0u8; 16];
        for c in 0..4 {
            let kw = self.round_keys[c].to_be_bytes();
            for r in 0..4 {
                s[r + 4 * c] ^= kw[r];
            }
        }
        for rnd in 1..10 {
            for b in &mut s {
                *b = SBOX[*b as usize];
            }
            s = [
                s[0], s[5], s[10], s[15], s[4], s[9], s[14], s[3], s[8], s[13], s[2], s[7], s[12],
                s[1], s[6], s[11],
            ];
            for c in 0..4 {
                let (a, b, cv, d) = (s[4 * c], s[4 * c + 1], s[4 * c + 2], s[4 * c + 3]);
                new_s[4 * c] = gmul(2, a) ^ gmul(3, b) ^ cv ^ d;
                new_s[4 * c + 1] = a ^ gmul(2, b) ^ gmul(3, cv) ^ d;
                new_s[4 * c + 2] = a ^ b ^ gmul(2, cv) ^ gmul(3, d);
                new_s[4 * c + 3] = gmul(3, a) ^ b ^ cv ^ gmul(2, d);
            }
            s = new_s;
            for c in 0..4 {
                let kw = self.round_keys[rnd * 4 + c].to_be_bytes();
                for r in 0..4 {
                    s[r + 4 * c] ^= kw[r];
                }
            }
        }
        for b in &mut s {
            *b = SBOX[*b as usize];
        }
        s = [
            s[0], s[5], s[10], s[15], s[4], s[9], s[14], s[3], s[8], s[13], s[2], s[7], s[12],
            s[1], s[6], s[11],
        ];
        for c in 0..4 {
            let kw = self.round_keys[40 + c].to_be_bytes();
            for r in 0..4 {
                s[r + 4 * c] ^= kw[r];
            }
        }
        *out = s;
        zeroize_bytes(&mut s);
        zeroize_bytes(&mut new_s);
    }

    /// Decrypts a single 16-byte block.
    pub fn decrypt_block(&self, block: &[u8; 16], out: &mut [u8; 16]) {
        let mut s = *block;
        let mut new_s = [0u8; 16];
        for c in 0..4 {
            let kw = self.round_keys[40 + c].to_be_bytes();
            for r in 0..4 {
                s[r + 4 * c] ^= kw[r];
            }
        }
        for rnd in (1..10).rev() {
            s = [
                s[0], s[13], s[10], s[7], s[4], s[1], s[14], s[11], s[8], s[5], s[2], s[15], s[12],
                s[9], s[6], s[3],
            ];
            for b in &mut s {
                *b = INV_SBOX[*b as usize];
            }
            for c in 0..4 {
                let kw = self.round_keys[rnd * 4 + c].to_be_bytes();
                for r in 0..4 {
                    s[r + 4 * c] ^= kw[r];
                }
            }
            for c in 0..4 {
                let (a, b, cv, d) = (s[4 * c], s[4 * c + 1], s[4 * c + 2], s[4 * c + 3]);
                new_s[4 * c] = gmul(14, a) ^ gmul(11, b) ^ gmul(13, cv) ^ gmul(9, d);
                new_s[4 * c + 1] = gmul(9, a) ^ gmul(14, b) ^ gmul(11, cv) ^ gmul(13, d);
                new_s[4 * c + 2] = gmul(13, a) ^ gmul(9, b) ^ gmul(14, cv) ^ gmul(11, d);
                new_s[4 * c + 3] = gmul(11, a) ^ gmul(13, b) ^ gmul(9, cv) ^ gmul(14, d);
            }
            s = new_s;
        }
        s = [
            s[0], s[13], s[10], s[7], s[4], s[1], s[14], s[11], s[8], s[5], s[2], s[15], s[12],
            s[9], s[6], s[3],
        ];
        for b in &mut s {
            *b = INV_SBOX[*b as usize];
        }
        for c in 0..4 {
            let kw = self.round_keys[c].to_be_bytes();
            for r in 0..4 {
                s[r + 4 * c] ^= kw[r];
            }
        }
        *out = s;
        zeroize_bytes(&mut s);
        zeroize_bytes(&mut new_s);
    }
}

/// In-place AES-128 CBC mode encryption.
///
/// # Errors
///
/// Returns [`AesError`] if `data.len()` is not a multiple of 16 bytes.
pub fn aes_128_cbc_encrypt(key: &[u8; 16], iv: &[u8; 16], data: &mut [u8]) -> Result<(), AesError> {
    if !data.len().is_multiple_of(16) {
        return Err(AesError);
    }
    let cipher = Aes128Key::new(key);
    let mut cur = *iv;
    let mut block = [0u8; 16];
    for chunk in data.as_chunks_mut::<16>().0 {
        for (c, &iv_b) in chunk.iter_mut().zip(cur.iter()) {
            *c ^= iv_b;
        }
        block.copy_from_slice(chunk);
        cipher.encrypt_block(&block, &mut cur);
        chunk.copy_from_slice(&cur);
    }
    zeroize_bytes(&mut cur);
    zeroize_bytes(&mut block);
    Ok(())
}

/// In-place AES-128 CBC mode decryption.
///
/// # Errors
///
/// Returns [`AesError`] if `data.len()` is not a multiple of 16 bytes.
pub fn aes_128_cbc_decrypt(key: &[u8; 16], iv: &[u8; 16], data: &mut [u8]) -> Result<(), AesError> {
    if !data.len().is_multiple_of(16) {
        return Err(AesError);
    }
    let cipher = Aes128Key::new(key);
    let mut cur = *iv;
    let mut block = [0u8; 16];
    let mut out = [0u8; 16];
    for chunk in data.as_chunks_mut::<16>().0 {
        block.copy_from_slice(chunk);
        cipher.decrypt_block(&block, &mut out);
        for (o, &iv_b) in out.iter_mut().zip(cur.iter()) {
            *o ^= iv_b;
        }
        cur = block;
        chunk.copy_from_slice(&out);
    }
    zeroize_bytes(&mut cur);
    zeroize_bytes(&mut block);
    zeroize_bytes(&mut out);
    Ok(())
}

fn shl1(b: &[u8; 16]) -> [u8; 16] {
    let mut out = [0u8; 16];
    let mut carry = 0u8;
    for i in (0..16).rev() {
        out[i] = (b[i] << 1) | carry;
        carry = (b[i] >> 7) & 1;
    }
    out[15] ^= 0u8.wrapping_sub(carry) & 0x87;
    out
}

/// RFC 4493 AES-CMAC (OMAC1) computation across multiple slice fragments.
pub fn omac1_aes_128_vector(key: &[u8; 16], slices: &[&[u8]]) -> [u8; 16] {
    let cipher = Aes128Key::new(key);
    let mut l = [0u8; 16];
    cipher.encrypt_block(&[0u8; 16], &mut l);
    let mut k1 = shl1(&l);
    let mut k2 = shl1(&k1);
    let mut x = [0u8; 16];
    let mut buf = [0u8; 16];
    let mut enc_out = [0u8; 16];
    let mut buf_len = 0usize;
    let mut has_data = false;

    for slice in slices {
        for &byte in *slice {
            if buf_len == 16 {
                for j in 0..16 {
                    x[j] ^= buf[j];
                }
                cipher.encrypt_block(&x, &mut enc_out);
                x = enc_out;
                buf_len = 0;
            }
            buf[buf_len] = byte;
            buf_len += 1;
            has_data = true;
        }
    }
    if has_data && buf_len == 16 {
        for j in 0..16 {
            buf[j] ^= k1[j];
        }
    } else {
        buf[buf_len] = 0x80;
        buf[buf_len + 1..].fill(0);
        for j in 0..16 {
            buf[j] ^= k2[j];
        }
    }
    for j in 0..16 {
        x[j] ^= buf[j];
    }
    let mut out = [0u8; 16];
    cipher.encrypt_block(&x, &mut out);
    zeroize_bytes(&mut l);
    zeroize_bytes(&mut k1);
    zeroize_bytes(&mut k2);
    zeroize_bytes(&mut x);
    zeroize_bytes(&mut buf);
    zeroize_bytes(&mut enc_out);
    out
}

/// RFC 4493 AES-CMAC (OMAC1) computation of a byte slice.
pub fn omac1_aes_128(key: &[u8; 16], data: &[u8]) -> [u8; 16] {
    omac1_aes_128_vector(key, &[data])
}

/// RFC 3394 AES Key Wrap algorithm supporting n >= 1 64-bit blocks.
///
/// # Errors
///
/// Returns [`AesError`] if `plain` is empty, not a multiple of 8 bytes, or
/// `cipher_out.len() != plain.len() + 8`.
pub fn aes_wrap(kek: &[u8; 16], plain: &[u8], cipher_out: &mut [u8]) -> Result<(), AesError> {
    if !plain.len().is_multiple_of(8)
        || plain.is_empty()
        || plain.len().checked_add(8) != Some(cipher_out.len())
    {
        return Err(AesError);
    }
    let n = plain.len() / 8;
    let cipher = Aes128Key::new(kek);
    cipher_out[..8].copy_from_slice(&[0xa6; 8]);
    cipher_out[8..].copy_from_slice(plain);
    let mut block = [0u8; 16];
    let mut enc = [0u8; 16];
    for j in 0..6 {
        for i in 1..=n {
            let t = (n * j + i) as u64;
            block[..8].copy_from_slice(&cipher_out[..8]);
            block[8..].copy_from_slice(&cipher_out[i * 8..(i + 1) * 8]);
            cipher.encrypt_block(&block, &mut enc);
            let a = u64::from_be_bytes([
                enc[0], enc[1], enc[2], enc[3], enc[4], enc[5], enc[6], enc[7],
            ]) ^ t;
            cipher_out[..8].copy_from_slice(&a.to_be_bytes());
            cipher_out[i * 8..(i + 1) * 8].copy_from_slice(&enc[8..16]);
        }
    }
    zeroize_bytes(&mut block);
    zeroize_bytes(&mut enc);
    Ok(())
}

/// RFC 3394 AES Key Unwrap algorithm supporting n >= 1 64-bit blocks.
///
/// # Errors
///
/// Returns [`AesError`] if `cipher` length is invalid or the integrity check
/// value fails verification.
pub fn aes_unwrap(kek: &[u8; 16], cipher: &[u8], plain_out: &mut [u8]) -> Result<(), AesError> {
    if !cipher.len().is_multiple_of(8) || cipher.len() < 16 || plain_out.len() != cipher.len() - 8 {
        return Err(AesError);
    }
    let n = (cipher.len() / 8) - 1;
    let key_sched = Aes128Key::new(kek);
    let mut a = [0u8; 8];
    a.copy_from_slice(&cipher[..8]);
    plain_out.copy_from_slice(&cipher[8..]);
    let mut block = [0u8; 16];
    let mut dec = [0u8; 16];
    for j in (0..6).rev() {
        for i in (1..=n).rev() {
            let t = (n * j + i) as u64;
            let a_val = u64::from_be_bytes(a) ^ t;
            block[..8].copy_from_slice(&a_val.to_be_bytes());
            block[8..].copy_from_slice(&plain_out[(i - 1) * 8..i * 8]);
            key_sched.decrypt_block(&block, &mut dec);
            a.copy_from_slice(&dec[..8]);
            plain_out[(i - 1) * 8..i * 8].copy_from_slice(&dec[8..16]);
        }
    }
    let valid = ct_eq(&a, &[0xa6; 8]);
    zeroize_bytes(&mut a);
    zeroize_bytes(&mut block);
    zeroize_bytes(&mut dec);
    if !valid {
        zeroize_bytes(plain_out);
        return Err(AesError);
    }
    Ok(())
}

fn gf128_mul(x: &[u8; 16], y: &[u8; 16]) -> [u8; 16] {
    let mut z = [0u8; 16];
    let mut v = *y;
    for &xb in x {
        for bit in (0..8).rev() {
            let mask = 0u8.wrapping_sub((xb >> bit) & 1);
            for i in 0..16 {
                z[i] ^= v[i] & mask;
            }
            let r_mask = 0u8.wrapping_sub(v[15] & 1) & 0xe1;
            for i in (1..16).rev() {
                v[i] = (v[i] >> 1) | (v[i - 1] << 7);
            }
            v[0] = (v[0] >> 1) ^ r_mask;
        }
    }
    zeroize_bytes(&mut v);
    z
}

/// NIST SP 800-38D AES-GMAC tag calculation.
///
/// # Errors
///
/// Returns [`AesError`] if `iv` is empty or `iv`/`aad` bit lengths overflow
/// `u64`.
pub fn aes_gmac(
    key: &[u8; 16],
    iv: &[u8],
    aad: &[u8],
    tag_out: &mut [u8; 16],
) -> Result<(), AesError> {
    if iv.is_empty() {
        return Err(AesError);
    }
    let iv_bits = (iv.len() as u64).checked_mul(8).ok_or(AesError)?;
    let aad_bits = (aad.len() as u64).checked_mul(8).ok_or(AesError)?;
    let cipher = Aes128Key::new(key);
    let mut h = [0u8; 16];
    cipher.encrypt_block(&[0u8; 16], &mut h);
    let mut j0 = [0u8; 16];
    if iv.len() == 12 {
        j0[..12].copy_from_slice(iv);
        j0[12..16].copy_from_slice(&1u32.to_be_bytes());
    } else {
        let mut j0_y = [0u8; 16];
        let mut iv_pos = 0;
        let mut chunk = [0u8; 16];
        while iv_pos < iv.len() {
            let take = (iv.len() - iv_pos).min(16);
            chunk.fill(0);
            chunk[..take].copy_from_slice(&iv[iv_pos..iv_pos + take]);
            for i in 0..16 {
                chunk[i] ^= j0_y[i];
            }
            j0_y = gf128_mul(&chunk, &h);
            iv_pos += take;
        }
        let mut iv_len_blk = [0u8; 16];
        iv_len_blk[8..16].copy_from_slice(&iv_bits.to_be_bytes());
        for i in 0..16 {
            iv_len_blk[i] ^= j0_y[i];
        }
        j0 = gf128_mul(&iv_len_blk, &h);
        zeroize_bytes(&mut j0_y);
        zeroize_bytes(&mut chunk);
        zeroize_bytes(&mut iv_len_blk);
    }
    let mut y = [0u8; 16];
    let mut pos = 0;
    let mut chunk = [0u8; 16];
    while pos < aad.len() {
        let take = (aad.len() - pos).min(16);
        chunk.fill(0);
        chunk[..take].copy_from_slice(&aad[pos..pos + take]);
        for i in 0..16 {
            chunk[i] ^= y[i];
        }
        y = gf128_mul(&chunk, &h);
        pos += take;
    }
    let mut len_blk = [0u8; 16];
    len_blk[..8].copy_from_slice(&aad_bits.to_be_bytes());
    for i in 0..16 {
        len_blk[i] ^= y[i];
    }
    y = gf128_mul(&len_blk, &h);
    let mut ej0 = [0u8; 16];
    cipher.encrypt_block(&j0, &mut ej0);
    for i in 0..16 {
        tag_out[i] = y[i] ^ ej0[i];
    }
    zeroize_bytes(&mut h);
    zeroize_bytes(&mut j0);
    zeroize_bytes(&mut y);
    zeroize_bytes(&mut chunk);
    zeroize_bytes(&mut len_blk);
    zeroize_bytes(&mut ej0);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_aes_128_fips197() {
        let key = [
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d,
            0x0e, 0x0f,
        ];
        let pt = [
            0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd,
            0xee, 0xff,
        ];
        let cipher = Aes128Key::new(&key);
        let mut ct = [0u8; 16];
        cipher.encrypt_block(&pt, &mut ct);
        assert_eq!(
            ct,
            [
                0x69, 0xc4, 0xe0, 0xd8, 0x6a, 0x7b, 0x04, 0x30, 0xd8, 0xcd, 0xb7, 0x80, 0x70, 0xb4,
                0xc5, 0x5a
            ]
        );
        let mut dec = [0u8; 16];
        cipher.decrypt_block(&ct, &mut dec);
        assert_eq!(dec, pt);
    }

    #[test]
    fn test_cbc_rfc3602() {
        let key = [
            0x06, 0xa9, 0x21, 0x40, 0x36, 0xb8, 0xa1, 0x5b, 0x51, 0x2e, 0x03, 0xd5, 0x34, 0x12,
            0x00, 0x06,
        ];
        let iv = [
            0x3d, 0xaf, 0xba, 0x42, 0x9d, 0x9e, 0xb4, 0x30, 0xb4, 0x22, 0xda, 0x80, 0x2c, 0x9f,
            0xac, 0x41,
        ];
        let mut data = *b"Single block msg";
        assert!(aes_128_cbc_encrypt(&key, &iv, &mut data).is_ok());
        assert_eq!(
            data,
            [
                0xe3, 0x53, 0x77, 0x9c, 0x10, 0x79, 0xae, 0xb8, 0x27, 0x08, 0x94, 0x2d, 0xbe, 0x77,
                0x18, 0x1a
            ]
        );
        assert!(aes_128_cbc_decrypt(&key, &iv, &mut data).is_ok());
        assert_eq!(&data, b"Single block msg");
    }

    #[test]
    fn test_cmac_rfc4493() {
        let key = [
            0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6, 0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf,
            0x4f, 0x3c,
        ];
        let tag0 = omac1_aes_128(&key, b"");
        assert_eq!(
            tag0,
            [
                0xbb, 0x1d, 0x69, 0x29, 0xe9, 0x59, 0x37, 0x28, 0x7f, 0xa3, 0x7d, 0x12, 0x9b, 0x75,
                0x67, 0x46
            ]
        );
        let msg = [
            0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96, 0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93,
            0x17, 0x2a,
        ];
        let tag16 = omac1_aes_128(&key, &msg);
        assert_eq!(
            tag16,
            [
                0x07, 0x0a, 0x16, 0xb4, 0x6b, 0x4d, 0x41, 0x44, 0xf7, 0x9b, 0xdd, 0x9d, 0xd0, 0x4a,
                0x28, 0x7c
            ]
        );
    }

    #[test]
    fn test_aes_wrap_rfc3394() {
        let kek = [
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d,
            0x0e, 0x0f,
        ];
        let pt = [
            0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd,
            0xee, 0xff,
        ];
        let mut ct = [0u8; 24];
        assert!(aes_wrap(&kek, &pt, &mut ct).is_ok());
        assert_eq!(
            ct,
            [
                0x1f, 0xa6, 0x8b, 0x0a, 0x81, 0x12, 0xb4, 0x47, 0xae, 0xf3, 0x4b, 0xd8, 0xfb, 0x5a,
                0x7b, 0x82, 0x9d, 0x3e, 0x86, 0x23, 0x71, 0xd2, 0xcf, 0xe5
            ]
        );
        let mut unwrap_pt = [0u8; 16];
        assert!(aes_unwrap(&kek, &ct, &mut unwrap_pt).is_ok());
        assert_eq!(unwrap_pt, pt);
    }

    #[test]
    fn test_aes_gmac_nist() {
        let key = [0u8; 16];
        let iv = [0u8; 12];
        let mut tag = [0u8; 16];
        assert!(aes_gmac(&key, &iv, b"", &mut tag).is_ok());
        assert_eq!(
            tag,
            [
                0x58, 0xe2, 0xfc, 0xce, 0xfa, 0x7e, 0x30, 0x61, 0x36, 0x7f, 0x1d, 0x57, 0xa4, 0xe7,
                0x45, 0x5a
            ]
        );
        let aad = [
            0xfe, 0xed, 0xfa, 0xce, 0xde, 0xad, 0xbe, 0xef, 0xfe, 0xed, 0xfa, 0xce, 0xde, 0xad,
            0xbe, 0xef, 0xab, 0xad, 0xda, 0xd2,
        ];
        let mut tag2 = [0u8; 16];
        assert!(aes_gmac(&key, &iv, &aad, &mut tag2).is_ok());
        assert_eq!(
            tag2,
            [
                0x9c, 0x60, 0xdd, 0x25, 0xfc, 0xd1, 0x3a, 0xf7, 0x28, 0x5e, 0x12, 0x94, 0x14, 0x80,
                0x56, 0x03
            ]
        );
    }

    #[test]
    fn test_aes_unwrap_tamper_zeroizes_output_and_cmac_multislice() {
        let kek = [0x11u8; 16];
        let pt = [0x22u8; 16];
        let mut ct = [0u8; 24];
        aes_wrap(&kek, &pt, &mut ct).expect("wrap");
        ct[0] ^= 0x01;
        let mut out = [0xAAu8; 16];
        assert_eq!(aes_unwrap(&kek, &ct, &mut out), Err(AesError));
        assert_eq!(out, [0u8; 16]);

        // Verify lazy-streaming CMAC matches single-slice across boundaries.
        let key = [0x2bu8; 16];
        let msg33 = [0x5au8; 33];
        for len in [0usize, 1, 15, 16, 17, 32, 33] {
            let single = omac1_aes_128(&key, &msg33[..len]);
            let split = len / 2;
            let vectored = omac1_aes_128_vector(&key, &[&msg33[..split], &msg33[split..len]]);
            assert_eq!(single, vectored, "CMAC mismatch at len={len}");
        }
    }
}
