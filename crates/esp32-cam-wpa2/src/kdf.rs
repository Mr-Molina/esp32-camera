//! Key derivation functions (PBKDF2, PRF, PTK, PMKID) for WPA/WPA2/WPA3.
//!
//! Compliant with IEEE 802.11i-2004, IEEE 802.11-2016, and RFC 2898.

use crate::crypto::hash::{ct_eq, hmac_sha1_vector, hmac_sha256_vector, zeroize_bytes};

/// Key derivation error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct KdfError;

/// Derives a key using PBKDF2-HMAC-SHA1 (RFC 2898 / IEEE 802.11i).
///
/// # Errors
///
/// Returns [`KdfError`] if `iterations == 0` or the output length requires more
/// than `u32::MAX` 20-byte blocks.
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
        zeroize_bytes(&mut u);
        zeroize_bytes(&mut f);
        out_offset += chunk;
        if out_offset < out.len() {
            block_idx = match block_idx.checked_add(1) {
                Some(next) => next,
                None => {
                    zeroize_bytes(out);
                    return Err(KdfError);
                }
            };
        }
    }

    Ok(())
}

/// IEEE 802.11i PRF-X using SHA-1 (Clause 8.5.1.1).
///
/// # Errors
///
/// Returns [`KdfError`] if `buf.len() > 256 * 20` (5120 bytes), which would
/// overflow the 8-bit counter.
pub fn sha1_prf(key: &[u8], label: &[u8], data: &[u8], buf: &mut [u8]) -> Result<(), KdfError> {
    const MAX_SHA1_PRF_BYTES: usize = 256 * 20;
    if buf.len() > MAX_SHA1_PRF_BYTES {
        return Err(KdfError);
    }
    let mut counter = 0u8;
    let mut pos = 0;

    while pos < buf.len() {
        let mut hash = hmac_sha1_vector(key, &[label, &[0x00], data, &[counter]]);
        let chunk = (buf.len() - pos).min(20);
        buf[pos..pos + chunk].copy_from_slice(&hash[..chunk]);
        zeroize_bytes(&mut hash);
        pos += chunk;
        if pos < buf.len() {
            counter = counter.checked_add(1).ok_or(KdfError)?;
        }
    }
    Ok(())
}

/// IEEE 802.11w/r KDF-SHA256 (IEEE Std 802.11-2016 Clause 12.7.1.2).
///
/// # Errors
///
/// Returns [`KdfError`] if `buf.len() * 8 > u16::MAX` (`buf.len() > 8191`) or
/// the 16-bit block counter overflows.
pub fn sha256_prf(key: &[u8], label: &[u8], data: &[u8], buf: &mut [u8]) -> Result<(), KdfError> {
    let buf_bits = buf
        .len()
        .checked_mul(8)
        .and_then(|bits| u16::try_from(bits).ok())
        .ok_or(KdfError)?;
    let len_bytes = buf_bits.to_le_bytes();
    let mut counter = 1u16;
    let mut pos = 0;

    while pos < buf.len() {
        let c_bytes = counter.to_le_bytes();
        let mut hash = hmac_sha256_vector(key, &[&c_bytes, label, data, &len_bytes]);
        let chunk = (buf.len() - pos).min(32);
        buf[pos..pos + chunk].copy_from_slice(&hash[..chunk]);
        zeroize_bytes(&mut hash);
        pos += chunk;
        if pos < buf.len() {
            counter = counter.checked_add(1).ok_or(KdfError)?;
        }
    }
    Ok(())
}

/// Pairwise Transient Key (PTK) components for CCMP and TKIP.
///
/// Intentionally not `Copy` and implements [`Drop`] to volatile-zeroize all
/// derived key material when dropped.
#[derive(Clone, Eq)]
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

impl Ptk {
    /// Creates a zeroed `Ptk` instance.
    #[must_use]
    pub const fn zeroed() -> Self {
        Self {
            kck: [0u8; 16],
            kek: [0u8; 16],
            tk: [0u8; 16],
            tx_mic: [0u8; 8],
            rx_mic: [0u8; 8],
        }
    }

    /// Volatile-zeroizes all key material contained in this PTK.
    pub fn zeroize(&mut self) {
        zeroize_bytes(&mut self.kck);
        zeroize_bytes(&mut self.kek);
        zeroize_bytes(&mut self.tk);
        zeroize_bytes(&mut self.tx_mic);
        zeroize_bytes(&mut self.rx_mic);
    }
}

impl Drop for Ptk {
    fn drop(&mut self) {
        self.zeroize();
    }
}

impl core::fmt::Debug for Ptk {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Ptk")
            .field("kck", &"[REDACTED]")
            .field("kek", &"[REDACTED]")
            .field("tk", &"[REDACTED]")
            .field("tx_mic", &"[REDACTED]")
            .field("rx_mic", &"[REDACTED]")
            .finish()
    }
}

impl PartialEq for Ptk {
    fn eq(&self, other: &Self) -> bool {
        ct_eq(&self.kck, &other.kck)
            & ct_eq(&self.kek, &other.kek)
            & ct_eq(&self.tk, &other.tk)
            & ct_eq(&self.tx_mic, &other.tx_mic)
            & ct_eq(&self.rx_mic, &other.rx_mic)
    }
}

/// Derives Pairwise Transient Key (PTK) from PMK, addresses, and nonces.
///
/// When `use_sha256` is `true`, computes IEEE 802.11-2016 Clause 12.7.1.2
/// `KDF-384` (48 bytes for CCMP-128: KCK, KEK, TK; `tx_mic`/`rx_mic` zeroed).
/// When `use_sha256` is `false`, computes IEEE 802.11i `PRF-512` (64 bytes).
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
        // KDF-384 (48 B) is statically within sha256_prf's 8191-byte limit.
        let _ = sha256_prf(pmk, label, &data, &mut ptk_buf[..48]);
    } else {
        // PRF-512 (64 B) is statically within sha1_prf's 5120-byte limit.
        let _ = sha1_prf(pmk, label, &data, &mut ptk_buf);
    }

    let mut ptk = Ptk::zeroed();
    ptk.kck.copy_from_slice(&ptk_buf[0..16]);
    ptk.kek.copy_from_slice(&ptk_buf[16..32]);
    ptk.tk.copy_from_slice(&ptk_buf[32..48]);
    ptk.tx_mic.copy_from_slice(&ptk_buf[48..56]);
    ptk.rx_mic.copy_from_slice(&ptk_buf[56..64]);

    zeroize_bytes(&mut ptk_buf);
    zeroize_bytes(&mut data);

    ptk
}

/// Derives RSN PMKID from PMK, Authenticator Address (AA), and Supplicant
/// Address (SPA).
pub fn rsn_pmkid(pmk: &[u8; 32], aa: &[u8; 6], spa: &[u8; 6], use_sha256: bool) -> [u8; 16] {
    let mut out = [0u8; 16];
    if use_sha256 {
        let mut h = hmac_sha256_vector(pmk, &[b"PMK Name", aa, spa]);
        out.copy_from_slice(&h[0..16]);
        zeroize_bytes(&mut h);
    } else {
        let mut h = hmac_sha1_vector(pmk, &[b"PMK Name", aa, spa]);
        out.copy_from_slice(&h[0..16]);
        zeroize_bytes(&mut h);
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

    #[test]
    fn test_ptk_sha256_uses_kdf384_and_redacted_debug_and_zeroize() {
        let pmk = [0x33u8; 32];
        let addr1 = [0x00, 0x11, 0x22, 0x33, 0x44, 0x55];
        let addr2 = [0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb];
        let nonce1 = [0x12u8; 32];
        let nonce2 = [0x34u8; 32];

        let mut ptk = wpa_pmk_to_ptk(
            &pmk,
            b"Pairwise key expansion",
            &addr1,
            &addr2,
            &nonce1,
            &nonce2,
            true,
        );

        // Verify KDF-384 (48 B) matches explicit 48-byte sha256_prf, not KDF-512.
        let mut data = [0u8; 76];
        data[0..6].copy_from_slice(&addr1);
        data[6..12].copy_from_slice(&addr2);
        data[12..44].copy_from_slice(&nonce1);
        data[44..76].copy_from_slice(&nonce2);

        let mut kdf384 = [0u8; 48];
        sha256_prf(&pmk, b"Pairwise key expansion", &data, &mut kdf384).expect("kdf384");
        let mut kdf512 = [0u8; 64];
        sha256_prf(&pmk, b"Pairwise key expansion", &data, &mut kdf512).expect("kdf512");

        assert_eq!(ptk.kck, kdf384[0..16]);
        assert_eq!(ptk.kek, kdf384[16..32]);
        assert_eq!(ptk.tk, kdf384[32..48]);
        assert_ne!(ptk.kck, kdf512[0..16]);
        assert_eq!(ptk.tx_mic, [0u8; 8]);
        assert_eq!(ptk.rx_mic, [0u8; 8]);

        // Redacted Debug must not expose raw key bytes.
        let dbg = format!("{ptk:?}");
        assert!(dbg.contains("[REDACTED]"));

        // Volatile zeroize clears all key fields and Drop is active on Ptk.
        assert!(core::mem::needs_drop::<Ptk>());
        ptk.zeroize();
        assert_eq!(ptk.kck, [0u8; 16]);
        assert_eq!(ptk.kek, [0u8; 16]);
        assert_eq!(ptk.tk, [0u8; 16]);
    }

    #[test]
    fn test_prf_length_bounds() {
        let mut oversized_sha1 = [0u8; 5121];
        assert_eq!(
            sha1_prf(b"k", b"l", b"d", &mut oversized_sha1),
            Err(KdfError)
        );
        let mut oversized_sha256 = [0u8; 8192];
        assert_eq!(
            sha256_prf(b"k", b"l", b"d", &mut oversized_sha256),
            Err(KdfError)
        );
    }
}
