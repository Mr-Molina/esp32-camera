//! Cryptographic hash and HMAC implementations (SHA-1, SHA-256, MD5).
//!
//! FIPS 180-4 and RFC 1321 compliant streaming implementations in pure Rust
//! with zero external dependencies and zero unsafe code.

/// Compares two slices for equality in constant time.
pub fn ct_eq(a: &[u8], b: &[u8]) -> bool {
    if a.len() != b.len() {
        return false;
    }
    let mut diff = 0u8;
    for (&x, &y) in a.iter().zip(b.iter()) {
        diff |= x ^ y;
    }
    diff == 0
}

struct HashBuf {
    buf: [u8; 64],
    len: usize,
    total_len: u64,
}

impl HashBuf {
    fn new() -> Self {
        Self {
            buf: [0u8; 64],
            len: 0,
            total_len: 0,
        }
    }

    fn update<F: FnMut(&[u8; 64])>(&mut self, data: &[u8], mut compress: F) {
        self.total_len = self.total_len.wrapping_add(data.len() as u64);
        let mut offset = 0;
        if self.len > 0 {
            let to_fill = 64 - self.len;
            if data.len() >= to_fill {
                self.buf[self.len..].copy_from_slice(&data[..to_fill]);
                compress(&self.buf);
                self.len = 0;
                offset = to_fill;
            } else {
                self.buf[self.len..self.len + data.len()].copy_from_slice(data);
                self.len += data.len();
                return;
            }
        }
        while offset + 64 <= data.len() {
            let mut block = [0u8; 64];
            block.copy_from_slice(&data[offset..offset + 64]);
            compress(&block);
            offset += 64;
        }
        if offset < data.len() {
            let rem = &data[offset..];
            self.buf[..rem.len()].copy_from_slice(rem);
            self.len = rem.len();
        }
    }

    fn finish<F: FnMut(&[u8; 64])>(mut self, be: bool, mut compress: F) {
        let bit_len = self.total_len.wrapping_mul(8);
        self.buf[self.len] = 0x80;
        self.len += 1;
        if self.len > 56 {
            self.buf[self.len..].fill(0);
            compress(&self.buf);
            self.buf = [0u8; 64];
        } else {
            self.buf[self.len..56].fill(0);
        }
        let len_bytes = if be {
            bit_len.to_be_bytes()
        } else {
            bit_len.to_le_bytes()
        };
        self.buf[56..64].copy_from_slice(&len_bytes);
        compress(&self.buf);
    }
}

// SHA-1 implementation (FIPS 180-4)
fn sha1_compress(state: &mut [u32; 5], block: &[u8; 64]) {
    let mut w = [0u32; 80];
    for (i, chunk) in block.as_chunks::<4>().0.iter().enumerate() {
        w[i] = u32::from_be_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
    }
    for i in 16..80 {
        w[i] = (w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16]).rotate_left(1);
    }
    let [mut a, mut b, mut c, mut d, mut e] = *state;
    for (i, &val) in w.iter().enumerate() {
        let (f, k) = match i {
            0..=19 => ((b & c) | (!b & d), 0x5a827999),
            20..=39 => (b ^ c ^ d, 0x6ed9eba1),
            40..=59 => ((b & c) | (b & d) | (c & d), 0x8f1bbcdc),
            _ => (b ^ c ^ d, 0xca62c1d6),
        };
        let t = a
            .rotate_left(5)
            .wrapping_add(f)
            .wrapping_add(e)
            .wrapping_add(k)
            .wrapping_add(val);
        e = d;
        d = c;
        c = b.rotate_left(30);
        b = a;
        a = t;
    }
    for (s, v) in state.iter_mut().zip([a, b, c, d, e].iter()) {
        *s = s.wrapping_add(*v);
    }
}

/// Computes SHA-1 hash across multiple slice fragments.
pub fn sha1_vector(slices: &[&[u8]]) -> [u8; 20] {
    let mut s = [0x67452301, 0xefcdab89, 0x98badcfe, 0x10325476, 0xc3d2e1f0];
    let mut b = HashBuf::new();
    for sl in slices {
        b.update(sl, |bk| sha1_compress(&mut s, bk));
    }
    b.finish(true, |bk| sha1_compress(&mut s, bk));
    let mut out = [0u8; 20];
    for (i, &w) in s.iter().enumerate() {
        out[i * 4..(i + 1) * 4].copy_from_slice(&w.to_be_bytes());
    }
    out
}

/// Computes SHA-1 hash of a byte slice.
pub fn sha1(data: &[u8]) -> [u8; 20] {
    sha1_vector(&[data])
}

// SHA-256 implementation (FIPS 180-4)
const K256: [u32; 64] = [
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
];

fn sha256_compress(state: &mut [u32; 8], block: &[u8; 64]) {
    let mut w = [0u32; 64];
    for (i, chunk) in block.as_chunks::<4>().0.iter().enumerate() {
        w[i] = u32::from_be_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
    }
    for i in 16..64 {
        let s0 = w[i - 15].rotate_right(7) ^ w[i - 15].rotate_right(18) ^ (w[i - 15] >> 3);
        let s1 = w[i - 2].rotate_right(17) ^ w[i - 2].rotate_right(19) ^ (w[i - 2] >> 10);
        w[i] = w[i - 16]
            .wrapping_add(s0)
            .wrapping_add(w[i - 7])
            .wrapping_add(s1);
    }
    let [mut a, mut b, mut c, mut d, mut e, mut f, mut g, mut h] = *state;
    for i in 0..64 {
        let s1 = e.rotate_right(6) ^ e.rotate_right(11) ^ e.rotate_right(25);
        let ch = (e & f) ^ (!e & g);
        let t1 = h
            .wrapping_add(s1)
            .wrapping_add(ch)
            .wrapping_add(K256[i])
            .wrapping_add(w[i]);
        let s0 = a.rotate_right(2) ^ a.rotate_right(13) ^ a.rotate_right(22);
        let maj = (a & b) ^ (a & c) ^ (b & c);
        let t2 = s0.wrapping_add(maj);
        h = g;
        g = f;
        f = e;
        e = d.wrapping_add(t1);
        d = c;
        c = b;
        b = a;
        a = t1.wrapping_add(t2);
    }
    for (s, v) in state.iter_mut().zip([a, b, c, d, e, f, g, h].iter()) {
        *s = s.wrapping_add(*v);
    }
}

/// Computes SHA-256 hash across multiple slice fragments.
pub fn sha256_vector(slices: &[&[u8]]) -> [u8; 32] {
    let mut s = [
        0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab,
        0x5be0cd19,
    ];
    let mut b = HashBuf::new();
    for sl in slices {
        b.update(sl, |bk| sha256_compress(&mut s, bk));
    }
    b.finish(true, |bk| sha256_compress(&mut s, bk));
    let mut out = [0u8; 32];
    for (i, &w) in s.iter().enumerate() {
        out[i * 4..(i + 1) * 4].copy_from_slice(&w.to_be_bytes());
    }
    out
}

/// Computes SHA-256 hash of a byte slice.
pub fn sha256(data: &[u8]) -> [u8; 32] {
    sha256_vector(&[data])
}

// MD5 implementation (RFC 1321)
const K_MD5: [u32; 64] = [
    0xd76aa478, 0xe8c7b756, 0x242070db, 0xc1bdceee, 0xf57c0faf, 0x4787c62a, 0xa8304613, 0xfd469501,
    0x698098d8, 0x8b44f7af, 0xffff5bb1, 0x895cd7be, 0x6b901122, 0xfd987193, 0xa679438e, 0x49b40821,
    0xf61e2562, 0xc040b340, 0x265e5a51, 0xe9b6c7aa, 0xd62f105d, 0x02441453, 0xd8a1e681, 0xe7d3fbc8,
    0x21e1cde6, 0xc33707d6, 0xf4d50d87, 0x455a14ed, 0xa9e3e905, 0xfcefa3f8, 0x676f02d9, 0x8d2a4c8a,
    0xfffa3942, 0x8771f681, 0x6d9d6122, 0xfde5380c, 0xa4beea44, 0x4bdecfa9, 0xf6bb4b60, 0xbebfbc70,
    0x289b7ec6, 0xeaa127fa, 0xd4ef3085, 0x04881d05, 0xd9d4d039, 0xe6db99e5, 0x1fa27cf8, 0xc4ac5665,
    0xf4292244, 0x432aff97, 0xab9423a7, 0xfc93a039, 0x655b59c3, 0x8f0ccc92, 0xffeff47d, 0x85845dd1,
    0x6fa87e4f, 0xfe2ce6e0, 0xa3014314, 0x4e0811a1, 0xf7537e82, 0xbd3af235, 0x2ad7d2bb, 0xeb86d391,
];

fn md5_compress(state: &mut [u32; 4], block: &[u8; 64]) {
    let mut x = [0u32; 16];
    for (i, chunk) in block.as_chunks::<4>().0.iter().enumerate() {
        x[i] = u32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
    }
    let [mut a, mut b, mut c, mut d] = *state;
    for i in 0..64 {
        let (f, k, s) = match i {
            0..=15 => (((b & c) | (!b & d)), i, [7, 12, 17, 22][i % 4]),
            16..=31 => (
                ((b & d) | (c & !d)),
                (5 * i + 1) % 16,
                [5, 9, 14, 20][i % 4],
            ),
            32..=47 => (b ^ c ^ d, (3 * i + 5) % 16, [4, 11, 16, 23][i % 4]),
            _ => (c ^ (b | !d), (7 * i) % 16, [6, 10, 15, 21][i % 4]),
        };
        let t = a
            .wrapping_add(f)
            .wrapping_add(K_MD5[i])
            .wrapping_add(x[k])
            .rotate_left(s);
        let nb = b.wrapping_add(t);
        a = d;
        d = c;
        c = b;
        b = nb;
    }
    for (s, v) in state.iter_mut().zip([a, b, c, d].iter()) {
        *s = s.wrapping_add(*v);
    }
}

/// Computes MD5 hash across multiple slice fragments.
pub fn md5_vector(slices: &[&[u8]]) -> [u8; 16] {
    let mut s = [0x67452301, 0xefcdab89, 0x98badcfe, 0x10325476];
    let mut b = HashBuf::new();
    for sl in slices {
        b.update(sl, |bk| md5_compress(&mut s, bk));
    }
    b.finish(false, |bk| md5_compress(&mut s, bk));
    let mut out = [0u8; 16];
    for (i, &w) in s.iter().enumerate() {
        out[i * 4..(i + 1) * 4].copy_from_slice(&w.to_le_bytes());
    }
    out
}

/// Computes MD5 hash of a byte slice.
pub fn md5(data: &[u8]) -> [u8; 16] {
    md5_vector(&[data])
}

// HMAC pad preparation
fn hmac_pad<const D: usize, F: Fn(&[u8]) -> [u8; D]>(key: &[u8], hf: F) -> ([u8; 64], [u8; 64]) {
    let mut k = [0u8; 64];
    if key.len() > 64 {
        k[..D].copy_from_slice(&hf(key));
    } else {
        k[..key.len()].copy_from_slice(key);
    }
    let (mut ip, mut op) = ([0u8; 64], [0u8; 64]);
    for i in 0..64 {
        ip[i] = k[i] ^ 0x36;
        op[i] = k[i] ^ 0x5c;
    }
    (ip, op)
}

/// Computes HMAC-SHA1 over multiple slice fragments.
pub fn hmac_sha1_vector(key: &[u8], slices: &[&[u8]]) -> [u8; 20] {
    let (ip, op) = hmac_pad(key, sha1);
    let mut s = [0x67452301, 0xefcdab89, 0x98badcfe, 0x10325476, 0xc3d2e1f0];
    let mut b = HashBuf::new();
    b.update(&ip, |bk| sha1_compress(&mut s, bk));
    for sl in slices {
        b.update(sl, |bk| sha1_compress(&mut s, bk));
    }
    b.finish(true, |bk| sha1_compress(&mut s, bk));
    let mut in_d = [0u8; 20];
    for (i, &w) in s.iter().enumerate() {
        in_d[i * 4..(i + 1) * 4].copy_from_slice(&w.to_be_bytes());
    }
    let mut s_out = [0x67452301, 0xefcdab89, 0x98badcfe, 0x10325476, 0xc3d2e1f0];
    let mut b_out = HashBuf::new();
    b_out.update(&op, |bk| sha1_compress(&mut s_out, bk));
    b_out.update(&in_d, |bk| sha1_compress(&mut s_out, bk));
    b_out.finish(true, |bk| sha1_compress(&mut s_out, bk));
    let mut out = [0u8; 20];
    for (i, &w) in s_out.iter().enumerate() {
        out[i * 4..(i + 1) * 4].copy_from_slice(&w.to_be_bytes());
    }
    out
}

/// Computes HMAC-SHA256 over multiple slice fragments.
pub fn hmac_sha256_vector(key: &[u8], slices: &[&[u8]]) -> [u8; 32] {
    let (ip, op) = hmac_pad(key, sha256);
    let mut s = [
        0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab,
        0x5be0cd19,
    ];
    let mut b = HashBuf::new();
    b.update(&ip, |bk| sha256_compress(&mut s, bk));
    for sl in slices {
        b.update(sl, |bk| sha256_compress(&mut s, bk));
    }
    b.finish(true, |bk| sha256_compress(&mut s, bk));
    let mut in_d = [0u8; 32];
    for (i, &w) in s.iter().enumerate() {
        in_d[i * 4..(i + 1) * 4].copy_from_slice(&w.to_be_bytes());
    }
    let mut s_out = [
        0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab,
        0x5be0cd19,
    ];
    let mut b_out = HashBuf::new();
    b_out.update(&op, |bk| sha256_compress(&mut s_out, bk));
    b_out.update(&in_d, |bk| sha256_compress(&mut s_out, bk));
    b_out.finish(true, |bk| sha256_compress(&mut s_out, bk));
    let mut out = [0u8; 32];
    for (i, &w) in s_out.iter().enumerate() {
        out[i * 4..(i + 1) * 4].copy_from_slice(&w.to_be_bytes());
    }
    out
}

/// Computes HMAC-MD5 over multiple slice fragments.
pub fn hmac_md5_vector(key: &[u8], slices: &[&[u8]]) -> [u8; 16] {
    let (ip, op) = hmac_pad(key, md5);
    let mut s = [0x67452301, 0xefcdab89, 0x98badcfe, 0x10325476];
    let mut b = HashBuf::new();
    b.update(&ip, |bk| md5_compress(&mut s, bk));
    for sl in slices {
        b.update(sl, |bk| md5_compress(&mut s, bk));
    }
    b.finish(false, |bk| md5_compress(&mut s, bk));
    let mut in_d = [0u8; 16];
    for (i, &w) in s.iter().enumerate() {
        in_d[i * 4..(i + 1) * 4].copy_from_slice(&w.to_le_bytes());
    }
    let mut s_out = [0x67452301, 0xefcdab89, 0x98badcfe, 0x10325476];
    let mut b_out = HashBuf::new();
    b_out.update(&op, |bk| md5_compress(&mut s_out, bk));
    b_out.update(&in_d, |bk| md5_compress(&mut s_out, bk));
    b_out.finish(false, |bk| md5_compress(&mut s_out, bk));
    let mut out = [0u8; 16];
    for (i, &w) in s_out.iter().enumerate() {
        out[i * 4..(i + 1) * 4].copy_from_slice(&w.to_le_bytes());
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ct_eq() {
        assert!(ct_eq(b"hello", b"hello"));
        assert!(!ct_eq(b"hello", b"hellp"));
        assert!(!ct_eq(b"hello", b"hell"));
    }

    #[test]
    fn test_sha1_nist() {
        assert_eq!(
            sha1(b""),
            *b"\xda\x39\xa3\xee\x5e\x6b\x4b\x0d\x32\x55\xbf\xef\x95\x60\x18\x90\xaf\xd8\x07\x09"
        );
        assert_eq!(
            sha1(b"abc"),
            *b"\xa9\x99\x3e\x36\x47\x06\x81\x6a\xba\x3e\x25\x71\x78\x50\xc2\x6c\x9c\xd0\xd8\x9d"
        );
    }

    #[test]
    fn test_sha256_nist() {
        let e_empty = [
            0xe3, 0xb0, 0xc4, 0x42, 0x98, 0xfc, 0x1c, 0x14, 0x9a, 0xfb, 0xf4, 0xc8, 0x99, 0x6f,
            0xb9, 0x24, 0x27, 0xae, 0x41, 0xe4, 0x64, 0x9b, 0x93, 0x4c, 0xa4, 0x95, 0x99, 0x1b,
            0x78, 0x52, 0xb8, 0x55,
        ];
        assert_eq!(sha256(b""), e_empty);
        let e_abc = [
            0xba, 0x78, 0x16, 0xbf, 0x8f, 0x01, 0xcf, 0xea, 0x41, 0x41, 0x40, 0xde, 0x5d, 0xae,
            0x22, 0x23, 0xb0, 0x03, 0x61, 0xa3, 0x96, 0x17, 0x7a, 0x9c, 0xb4, 0x10, 0xff, 0x61,
            0xf2, 0x00, 0x15, 0xad,
        ];
        assert_eq!(sha256(b"abc"), e_abc);
    }

    #[test]
    fn test_md5_rfc1321() {
        assert_eq!(
            md5(b""),
            *b"\xd4\x1d\x8c\xd9\x8f\x00\xb2\x04\xe9\x80\x09\x98\xec\xf8\x42\x7e"
        );
        assert_eq!(
            md5(b"abc"),
            *b"\x90\x01\x50\x98\x3c\xd2\x4f\xb0\xd6\x96\x3f\x7d\x28\xe1\x7f\x72"
        );
    }

    #[test]
    fn test_hmac_rfc_vectors() {
        let (k20, k16, msg) = ([0x0b; 20], [0x0b; 16], b"Hi There");
        assert_eq!(
            hmac_sha1_vector(&k20, &[msg]),
            *b"\xb6\x17\x31\x86\x55\x05\x72\x64\xe2\x8b\xc0\xb6\xfb\x37\x8c\x8e\xf1\x46\xbe\x00"
        );
        assert_eq!(
            hmac_md5_vector(&k16, &[msg]),
            *b"\x92\x94\x72\x7a\x36\x38\xbb\x1c\x13\xf4\x8e\xf8\x15\x8b\xfc\x9d"
        );
        let e_s256 = [
            0xb0, 0x34, 0x4c, 0x61, 0xd8, 0xdb, 0x38, 0x53, 0x5c, 0xa8, 0xaf, 0xce, 0xaf, 0x0b,
            0xf1, 0x2b, 0x88, 0x1d, 0xc2, 0x00, 0xc9, 0x83, 0x3d, 0xa7, 0x26, 0xe9, 0x37, 0x6c,
            0x2e, 0x32, 0xcf, 0xf7,
        ];
        assert_eq!(hmac_sha256_vector(&k20, &[msg]), e_s256);
    }
}
