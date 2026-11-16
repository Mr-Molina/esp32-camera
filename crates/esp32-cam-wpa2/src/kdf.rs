//! Key derivation functions (PBKDF2, PRF, PTK, PMKID) for WPA/WPA2/WPA3.
//!
//! Compliant with IEEE 802.11i-2004, IEEE 802.11-2016, and RFC 2898.

use crate::crypto::hash::{hmac_sha1_vector, hmac_sha256_vector};

/// Key derivation error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct KdfError;

/// Derives a key using PBKDF2-HMAC-SHA1 (RFC 2898 / IEEE 802.11i).
pub fn pbkdf2_hmac_sha1(
    passphrase: &[u8],
    ssid: &[u8],
    iterations: u32,
    out: &mut [u8],
) -> Result<(), KdfError> {
    if iterations == 0 {
        return Err(KdfError);
    }

    let mut block_idx = 1u32;
    let mut out_offset = 0;

    while out_offset < out.len() {
        let be_idx = block_idx.to_be_bytes();
        let mut u = hmac_sha1_vector(passphrase, &[ssid, &be_idx]);
        let mut f = u;

        for _ in 1..iterations {
            u = hmac_sha1_vector(passphrase, &[&u]);
            for (fb, ub) in f.iter_mut().zip(u.iter()) {
                *fb ^= *ub;
            }
        }

        let chunk = (out.len() - out_offset).min(20);
        out[out_offset..out_offset + chunk].copy_from_slice(&f[..chunk]);
        out_offset += chunk;
        block_idx = block_idx.checked_add(1).ok_or(KdfError)?;
    }

    Ok(())
}

/// IEEE 802.11i PRF-X using SHA-1 (Clause 8.5.1.1).
pub fn sha1_prf(key: &[u8], label: &[u8], data: &[u8], buf: &mut [u8]) {
    let mut counter = 0u8;
    let mut pos = 0;

    while pos < buf.len() {
        let hash = hmac_sha1_vector(key, &[label, &[0x00], data, &[counter]]);
        let chunk = (buf.len() - pos).min(20);
        buf[pos..pos + chunk].copy_from_slice(&hash[..chunk]);
        pos += chunk;
        counter = counter.wrapping_add(1);
    }
}

/// IEEE 802.11w/r KDF-SHA256 (IEEE Std 802.11-2016 Clause 12.7.1.2).
pub fn sha256_prf(key: &[u8], label: &[u8], data: &[u8], buf: &mut [u8]) {
    let mut counter = 1u16;
    let mut pos = 0;
    let buf_bits = ((buf.len() as u32) * 8) as u16;
    let len_bytes = buf_bits.to_le_bytes();

    while pos < buf.len() {
        let c_bytes = counter.to_le_bytes();
        let hash = hmac_sha256_vector(key, &[&c_bytes, label, data, &len_bytes]);
        let chunk = (buf.len() - pos).min(32);
        buf[pos..pos + chunk].copy_from_slice(&hash[..chunk]);
        pos += chunk;
        counter = counter.wrapping_add(1);
    }
}

/// Pairwise Transient Key (PTK) components for CCMP and TKIP.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Ptk {
    /// Key Confirmation Key (KCK).
    pub kck: [u8; 16],
    /// Key Encryption Key (KEK).
    pub kek: [u8; 16],
    /// Temporal Key (TK).
    pub tk: [u8; 16],
    /// Tx MIC key (for TKIP).
    pub tx_mic: [u8; 8],
    /// Rx MIC key (for TKIP).
    pub rx_mic: [u8; 8],
}

/// Derives Pairwise Transient Key (PTK) from PMK, addresses, and nonces.
pub fn wpa_pmk_to_ptk(
    pmk: &[u8; 32],
    label: &[u8],
    addr1: &[u8; 6],
    addr2: &[u8; 6],
    nonce1: &[u8; 32],
    nonce2: &[u8; 32],
    use_sha256: bool,
) -> Ptk {
    let (min_addr, max_addr) = if addr1 < addr2 {
        (addr1, addr2)
    } else {
        (addr2, addr1)
    };
    let (min_nonce, max_nonce) = if nonce1 < nonce2 {
        (nonce1, nonce2)
    } else {
        (nonce2, nonce1)
    };

    let mut data = [0u8; 76];
    data[0..6].copy_from_slice(min_addr);
    data[6..12].copy_from_slice(max_addr);
    data[12..44].copy_from_slice(min_nonce);
    data[44..76].copy_from_slice(max_nonce);

    let mut ptk_buf = [0u8; 64];
    if use_sha256 {
        sha256_prf(pmk, label, &data, &mut ptk_buf);
    } else {
        sha1_prf(pmk, label, &data, &mut ptk_buf);
    }

    let mut kck = [0u8; 16];
    let mut kek = [0u8; 16];
    let mut tk = [0u8; 16];
    let mut tx_mic = [0u8; 8];
    let mut rx_mic = [0u8; 8];

    kck.copy_from_slice(&ptk_buf[0..16]);
    kek.copy_from_slice(&ptk_buf[16..32]);
    tk.copy_from_slice(&ptk_buf[32..48]);
    tx_mic.copy_from_slice(&ptk_buf[48..56]);
    rx_mic.copy_from_slice(&ptk_buf[56..64]);

    Ptk {
        kck,
        kek,
        tk,
        tx_mic,
        rx_mic,
    }
}

/// Derives RSN PMKID from PMK, Authenticator Address (AA), and Supplicant
/// Address (SPA).
pub fn rsn_pmkid(pmk: &[u8; 32], aa: &[u8; 6], spa: &[u8; 6], use_sha256: bool) -> [u8; 16] {
    let mut out = [0u8; 16];
    if use_sha256 {
        let h = hmac_sha256_vector(pmk, &[b"PMK Name", aa, spa]);
        out.copy_from_slice(&h[0..16]);
    } else {
        let h = hmac_sha1_vector(pmk, &[b"PMK Name", aa, spa]);
        out.copy_from_slice(&h[0..16]);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[cfg(not(miri))]
    fn test_annex_h_pbkdf2_vector() {
        // IEEE 802.11i Annex H.1
        let passphrase = b"password";
        let ssid = b"IEEE";
        let mut pmk = [0u8; 32];
        pbkdf2_hmac_sha1(passphrase, ssid, 4096, &mut pmk)
            .expect("PBKDF2 HMAC-SHA1 vector derivation failed");

        let expected = [
            0xf4, 0x2c, 0x6f, 0xc5, 0x2d, 0xf0, 0xeb, 0xef, 0x9e, 0xbb, 0x4b, 0x90, 0xb3, 0x8a,
            0x5f, 0x90, 0x2e, 0x83, 0xfe, 0x1b, 0x13, 0x5a, 0x70, 0xe2, 0x3a, 0xed, 0x76, 0x2e,
            0x97, 0x10, 0xa1, 0x2e,
        ];
        assert_eq!(pmk, expected);
    }

    #[test]
    #[cfg(miri)]
    fn test_annex_h_pbkdf2_vector() {
        // RFC 6070 PBKDF2 HMAC-SHA1 2-iteration test for rapid Miri execution.
        let mut out32 = [0u8; 32];
        pbkdf2_hmac_sha1(b"password", b"salt", 2, &mut out32)
            .expect("PBKDF2 HMAC-SHA1 2-iteration derivation failed");

        let expected_prefix = [
            0xea, 0x6c, 0x01, 0x4d, 0xc7, 0x2d, 0x6f, 0x8c, 0xcd, 0x1e, 0xd9, 0x2a, 0xce, 0x1d,
            0x41, 0xf0, 0xd8, 0xde, 0x89, 0x57,
        ];
        assert_eq!(&out32[..20], &expected_prefix);
        assert!(out32[20..].iter().any(|&b| b != 0));
    }

    #[test]
    fn test_ptk_derivation() {
        let pmk = [0x55u8; 32];
        let addr1 = [0x00, 0x11, 0x22, 0x33, 0x44, 0x55];
        let addr2 = [0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb];
        let nonce1 = [0x12u8; 32];
        let nonce2 = [0x34u8; 32];

        let ptk = wpa_pmk_to_ptk(
            &pmk,
            b"Pairwise key expansion",
            &addr1,
            &addr2,
            &nonce1,
            &nonce2,
            false,
        );

        assert_ne!(ptk.kck, [0u8; 16]);
        assert_ne!(ptk.kek, [0u8; 16]);
        assert_ne!(ptk.tk, [0u8; 16]);

        // Address swap invariance: min/max ordering guarantees identical PTK
        let ptk_swapped = wpa_pmk_to_ptk(
            &pmk,
            b"Pairwise key expansion",
            &addr2,
            &addr1,
            &nonce2,
            &nonce1,
            false,
        );
        assert_eq!(ptk, ptk_swapped);
    }

    #[test]
    fn test_pmkid_computation() {
        let pmk = [0x77u8; 32];
        let aa = [0x00, 0x11, 0x22, 0x33, 0x44, 0x55];
        let spa = [0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff];

        let id1 = rsn_pmkid(&pmk, &aa, &spa, false);
        let id2 = rsn_pmkid(&pmk, &aa, &spa, true);
        assert_ne!(id1, [0u8; 16]);
        assert_ne!(id2, [0u8; 16]);
        assert_ne!(id1, id2);
    }
}
