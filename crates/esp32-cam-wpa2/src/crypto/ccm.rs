//! AES-CCM (RFC 3610), IEEE 802.11i CCMP, and RC4 implementations.
//!
//! Compliant with RFC 3610 and IEEE 802.11-2016 Clause 12.5.3.

use super::aes::Aes128Key;
use super::hash::{ct_eq, zeroize_bytes};

/// Maximum valid 48-bit CCMP Packet Number (`2^48 - 1`).
const MAX_CCMP_PN: u64 = 0x0000_FFFF_FFFF_FFFF;

/// Maximum valid AAD length for RFC 3610 2-byte length encoding (`0xFF00 - 1`).
const MAX_CCM_AAD_LEN: usize = 0xFEFF;

/// Error type for CCM, CCMP, and RC4 operations.
#[non_exhaustive]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CcmError {
    /// Invalid authentication tag length (must be even and in `4..=16`).
    InvalidTagLength,
    /// Provided input or output buffer is too small.
    BufferTooSmall,
    /// Plaintext (`> u16::MAX`) or AAD (`>= 0xFF00`) exceeds `L = 2` bounds.
    InvalidInputLength,
    /// CTR block counter overflowed `u16::MAX`.
    CounterOverflow,
    /// Malformed 802.11 MAC or CCMP header.
    InvalidHeader,
    /// Packet Number is zero or exceeds 48 bits.
    InvalidPacketNumber,
    /// Key ID exceeds valid 2-bit range (`0..=3`).
    InvalidKeyId,
    /// Frame type or format is unsupported for CCMP.
    UnsupportedFrame,
    /// Authentication tag (MIC) verification failed.
    AuthFailed,
    /// RC4 key length or skip count is outside permitted bounds.
    InvalidRc4Param,
}

impl core::fmt::Display for CcmError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::InvalidTagLength => write!(f, "invalid CCM tag length"),
            Self::BufferTooSmall => write!(f, "buffer too small"),
            Self::InvalidInputLength => write!(f, "CCM payload or AAD length out of bounds"),
            Self::CounterOverflow => write!(f, "CCM CTR counter overflow"),
            Self::InvalidHeader => write!(f, "invalid 802.11 or CCMP header"),
            Self::InvalidPacketNumber => write!(f, "invalid CCMP packet number"),
            Self::InvalidKeyId => write!(f, "invalid CCMP key ID"),
            Self::UnsupportedFrame => write!(f, "unsupported 802.11 frame format"),
            Self::AuthFailed => write!(f, "CCM authentication tag mismatch"),
            Self::InvalidRc4Param => write!(f, "invalid RC4 parameter"),
        }
    }
}

/// AES-CCM authenticated encryption (RFC 3610) with L=2.
///
/// # Errors
///
/// Returns [`CcmError`] if `m_len_tag` is not an even value in `4..=16`,
/// `plain.len() > u16::MAX`, `aad.len() >= 0xFF00`, output slices are too
/// small, or the CTR counter overflows.
pub fn aes_ccm_ae(
    key: &[u8; 16],
    nonce: &[u8; 13],
    m_len_tag: usize,
    plain: &[u8],
    aad: &[u8],
    crypt_out: &mut [u8],
    tag_out: &mut [u8],
) -> Result<(), CcmError> {
    if !(4..=16).contains(&m_len_tag) || !m_len_tag.is_multiple_of(2) {
        return Err(CcmError::InvalidTagLength);
    }
    if plain.len() > u16::MAX as usize || aad.len() > MAX_CCM_AAD_LEN {
        return Err(CcmError::InvalidInputLength);
    }
    if tag_out.len() < m_len_tag || crypt_out.len() < plain.len() {
        return Err(CcmError::BufferTooSmall);
    }

    let aes = Aes128Key::new(key);
    let mut x = [0u8; 16];
    let mut b0 = [0u8; 16];

    // B_0 formatting: Flags | Nonce (13) | Length (2)
    b0[0] = if !aad.is_empty() { 0x40 } else { 0 };
    b0[0] |= (((m_len_tag - 2) / 2) as u8) << 3;
    b0[0] |= 1; // L - 1 = 2 - 1 = 1
    b0[1..14].copy_from_slice(nonce);
    b0[14..16].copy_from_slice(&(plain.len() as u16).to_be_bytes());

    aes.encrypt_block(&b0, &mut x);

    // Process AAD if present
    let mut aad_block = [0u8; 16];
    if !aad.is_empty() {
        aad_block[0..2].copy_from_slice(&(aad.len() as u16).to_be_bytes());
        let to_copy = (16 - 2).min(aad.len());
        aad_block[2..2 + to_copy].copy_from_slice(&aad[..to_copy]);
        for (xb, ab) in x.iter_mut().zip(aad_block.iter()) {
            *xb ^= *ab;
        }
        let in_blk = x;
        aes.encrypt_block(&in_blk, &mut x);

        let mut offset = to_copy;
        while offset < aad.len() {
            aad_block.fill(0);
            let chunk = (aad.len() - offset).min(16);
            aad_block[..chunk].copy_from_slice(&aad[offset..offset + chunk]);
            for (xb, ab) in x.iter_mut().zip(aad_block.iter()) {
                *xb ^= *ab;
            }
            let in_blk = x;
            aes.encrypt_block(&in_blk, &mut x);
            offset += chunk;
        }
    }

    // CBC-MAC over plain text
    let mut offset = 0;
    while offset < plain.len() {
        let chunk = (plain.len() - offset).min(16);
        for i in 0..chunk {
            x[i] ^= plain[offset + i];
        }
        let in_blk = x;
        aes.encrypt_block(&in_blk, &mut x);
        offset += chunk;
    }

    // CTR encryption
    let mut a = [0u8; 16];
    a[0] = 1; // L - 1
    a[1..14].copy_from_slice(nonce);

    let mut counter = 1u16;
    let mut s = [0u8; 16];
    offset = 0;
    while offset < plain.len() {
        a[14..16].copy_from_slice(&counter.to_be_bytes());
        aes.encrypt_block(&a, &mut s);
        let chunk = (plain.len() - offset).min(16);
        for i in 0..chunk {
            crypt_out[offset + i] = plain[offset + i] ^ s[i];
        }
        offset += chunk;
        if offset < plain.len() {
            counter = match counter.checked_add(1) {
                Some(next) => next,
                None => {
                    zeroize_bytes(&mut crypt_out[..plain.len()]);
                    zeroize_bytes(&mut x);
                    zeroize_bytes(&mut b0);
                    zeroize_bytes(&mut aad_block);
                    zeroize_bytes(&mut a);
                    zeroize_bytes(&mut s);
                    return Err(CcmError::CounterOverflow);
                }
            };
        }
    }

    // Encrypt authentication tag
    a[14..16].copy_from_slice(&0u16.to_be_bytes());
    aes.encrypt_block(&a, &mut s);
    for i in 0..m_len_tag {
        tag_out[i] = x[i] ^ s[i];
    }

    zeroize_bytes(&mut x);
    zeroize_bytes(&mut b0);
    zeroize_bytes(&mut aad_block);
    zeroize_bytes(&mut a);
    zeroize_bytes(&mut s);
    Ok(())
}

/// AES-CCM authenticated decryption (RFC 3610) with L=2.
///
/// # Errors
///
/// Returns [`CcmError`] if parameters are invalid, buffers are undersized, the
/// CTR counter overflows, or the authentication tag does not match (in which
/// case `plain_out[..crypt.len()]` is volatile-zeroized).
pub fn aes_ccm_ad(
    key: &[u8; 16],
    nonce: &[u8; 13],
    m_len_tag: usize,
    crypt: &[u8],
    aad: &[u8],
    tag: &[u8],
    plain_out: &mut [u8],
) -> Result<(), CcmError> {
    if !(4..=16).contains(&m_len_tag) || !m_len_tag.is_multiple_of(2) {
        return Err(CcmError::InvalidTagLength);
    }
    if crypt.len() > u16::MAX as usize || aad.len() > MAX_CCM_AAD_LEN {
        return Err(CcmError::InvalidInputLength);
    }
    if plain_out.len() < crypt.len() || tag.len() < m_len_tag {
        return Err(CcmError::BufferTooSmall);
    }

    let aes = Aes128Key::new(key);
    let mut a = [0u8; 16];
    a[0] = 1; // L - 1
    a[1..14].copy_from_slice(nonce);

    // CTR decryption
    let mut counter = 1u16;
    let mut s = [0u8; 16];
    let mut offset = 0;
    while offset < crypt.len() {
        a[14..16].copy_from_slice(&counter.to_be_bytes());
        aes.encrypt_block(&a, &mut s);
        let chunk = (crypt.len() - offset).min(16);
        for i in 0..chunk {
            plain_out[offset + i] = crypt[offset + i] ^ s[i];
        }
        offset += chunk;
        if offset < crypt.len() {
            counter = match counter.checked_add(1) {
                Some(next) => next,
                None => {
                    zeroize_bytes(&mut plain_out[..crypt.len()]);
                    zeroize_bytes(&mut a);
                    zeroize_bytes(&mut s);
                    return Err(CcmError::CounterOverflow);
                }
            };
        }
    }

    // Verify authentication tag
    let mut x = [0u8; 16];
    let mut b0 = [0u8; 16];
    b0[0] = if !aad.is_empty() { 0x40 } else { 0 };
    b0[0] |= (((m_len_tag - 2) / 2) as u8) << 3;
    b0[0] |= 1;
    b0[1..14].copy_from_slice(nonce);
    b0[14..16].copy_from_slice(&(crypt.len() as u16).to_be_bytes());

    aes.encrypt_block(&b0, &mut x);

    let mut aad_block = [0u8; 16];
    if !aad.is_empty() {
        aad_block[0..2].copy_from_slice(&(aad.len() as u16).to_be_bytes());
        let to_copy = (16 - 2).min(aad.len());
        aad_block[2..2 + to_copy].copy_from_slice(&aad[..to_copy]);
        for (xb, ab) in x.iter_mut().zip(aad_block.iter()) {
            *xb ^= *ab;
        }
        let in_blk = x;
        aes.encrypt_block(&in_blk, &mut x);

        let mut off = to_copy;
        while off < aad.len() {
            aad_block.fill(0);
            let chunk = (aad.len() - off).min(16);
            aad_block[..chunk].copy_from_slice(&aad[off..off + chunk]);
            for (xb, ab) in x.iter_mut().zip(aad_block.iter()) {
                *xb ^= *ab;
            }
            let in_blk = x;
            aes.encrypt_block(&in_blk, &mut x);
            off += chunk;
        }
    }

    offset = 0;
    while offset < crypt.len() {
        let chunk = (crypt.len() - offset).min(16);
        for i in 0..chunk {
            x[i] ^= plain_out[offset + i];
        }
        let in_blk = x;
        aes.encrypt_block(&in_blk, &mut x);
        offset += chunk;
    }

    a[14..16].copy_from_slice(&0u16.to_be_bytes());
    aes.encrypt_block(&a, &mut s);
    let mut expected_tag = [0u8; 16];
    for i in 0..m_len_tag {
        expected_tag[i] = x[i] ^ s[i];
    }

    let auth_ok = ct_eq(&expected_tag[..m_len_tag], &tag[..m_len_tag]);
    zeroize_bytes(&mut a);
    zeroize_bytes(&mut s);
    zeroize_bytes(&mut x);
    zeroize_bytes(&mut b0);
    zeroize_bytes(&mut aad_block);
    zeroize_bytes(&mut expected_tag);

    if auth_ok {
        Ok(())
    } else {
        zeroize_bytes(&mut plain_out[..crypt.len()]);
        Err(CcmError::AuthFailed)
    }
}

/// Constructs CCMP AAD and Nonce from an 802.11 header and 8-byte CCMP IV
/// header per IEEE 802.11-2016 Clause 12.5.3.3.3–12.5.3.3.4.
///
/// # Errors
///
/// Returns [`CcmError::InvalidHeader`] if `hdr` or `data` is shorter than the
/// required 802.11/CCMP header length, or [`CcmError::UnsupportedFrame`] for
/// Control/Extension frames or 4-address Management frames.
pub fn ccmp_aad_nonce(hdr: &[u8], data: &[u8]) -> Result<([u8; 30], usize, [u8; 13]), CcmError> {
    if hdr.len() < 24 || data.len() < 8 {
        return Err(CcmError::InvalidHeader);
    }
    let mut fc = u16::from_le_bytes([hdr[0], hdr[1]]);
    let stype = (fc & 0x00f0) >> 4;
    let type_ = (fc & 0x000c) >> 2;
    let addr4 = (fc & 0x0300) == 0x0300;
    let mut qos = false;
    let mut nonce = [0u8; 13];

    if type_ == 2 {
        // Data frame: mask Subtype bits 4..6 (0x0070) per IEEE 802.11-2016 §12.5.3.3.3.
        fc &= !0x0070;
        if (stype & 0x08) != 0 {
            qos = true;
            fc &= !0x8000; // Mask +HTC/Order bit on QoS Data frames.
        }
    } else if type_ == 0 {
        // Management frame: preserve Subtype bits; set Nonce Flags bit 4 (0x10).
        if addr4 {
            return Err(CcmError::UnsupportedFrame);
        }
        nonce[0] = 0x10;
    } else {
        return Err(CcmError::UnsupportedFrame);
    }

    let qc_offset = 24 + if addr4 { 6 } else { 0 };
    let has_htc = (qos || type_ == 0) && (hdr[1] & 0x80) != 0;
    let req_hdr_len = qc_offset + if qos { 2 } else { 0 } + if has_htc { 4 } else { 0 };
    if hdr.len() < req_hdr_len {
        return Err(CcmError::InvalidHeader);
    }
    if qos {
        nonce[0] = hdr[qc_offset] & 0x0f;
    }

    fc &= !(0x0800 | 0x1000 | 0x2000); // Clear RETRY, PWRMGT, MOREDATA
    fc |= 0x4000; // Set Protected Frame bit

    let mut aad = [0u8; 30];
    aad[0..2].copy_from_slice(&fc.to_le_bytes());
    aad[2..20].copy_from_slice(&hdr[4..22]); // Addr1, Addr2, Addr3
    let seq = u16::from_le_bytes([hdr[22], hdr[23]]) & !0xfff0;
    aad[20..22].copy_from_slice(&seq.to_le_bytes());

    let mut pos = 22;
    if addr4 {
        aad[pos..pos + 6].copy_from_slice(&hdr[24..30]);
        pos += 6;
    }
    if qos {
        aad[pos] = hdr[qc_offset] & 0x0f;
        aad[pos + 1] = 0;
        pos += 2;
    }
    let aad_len = pos;

    nonce[1..7].copy_from_slice(&hdr[10..16]); // Addr2
    nonce[7] = data[7]; // PN5
    nonce[8] = data[6]; // PN4
    nonce[9] = data[5]; // PN3
    nonce[10] = data[4]; // PN2
    nonce[11] = data[1]; // PN1
    nonce[12] = data[0]; // PN0

    Ok((aad, aad_len, nonce))
}

/// Decrypts a CCMP-protected frame payload, always verifying the 8-byte MIC.
///
/// # Errors
///
/// Returns [`CcmError`] if the Protected/ExtIV flags are missing, the extracted
/// PN is zero, buffers are undersized, or MIC verification fails.
pub fn ccmp_decrypt_frame(
    tk: &[u8; 16],
    hdr: &[u8],
    data: &[u8],
    out: &mut [u8],
    espnow_pkt: bool,
) -> Result<usize, CcmError> {
    if data.len() < 16 {
        return Err(CcmError::BufferTooSmall);
    }
    if hdr.len() < 24 || (hdr[1] & 0x40) == 0 || (data[3] & 0x20) == 0 {
        return Err(CcmError::InvalidHeader);
    }
    let pn = (u64::from(data[0]))
        | (u64::from(data[1]) << 8)
        | (u64::from(data[4]) << 16)
        | (u64::from(data[5]) << 24)
        | (u64::from(data[6]) << 32)
        | (u64::from(data[7]) << 40);
    if pn == 0 || pn > MAX_CCMP_PN {
        return Err(CcmError::InvalidPacketNumber);
    }
    let mlen = data.len() - 16;
    if mlen > u16::MAX as usize {
        return Err(CcmError::InvalidInputLength);
    }
    if out.len() < mlen {
        return Err(CcmError::BufferTooSmall);
    }

    let (mut aad, aad_len, mut nonce) = ccmp_aad_nonce(hdr, data)?;
    if espnow_pkt {
        nonce[0] = 0;
    }

    let res = aes_ccm_ad(
        tk,
        &nonce,
        8,
        &data[8..8 + mlen],
        &aad[..aad_len],
        &data[8 + mlen..16 + mlen],
        &mut out[..mlen],
    );
    zeroize_bytes(&mut aad);
    zeroize_bytes(&mut nonce);
    res?;

    Ok(mlen)
}

/// Alias for `ccmp_decrypt_frame`.
#[inline]
pub fn ccmp_decrypt(
    tk: &[u8; 16],
    hdr: &[u8],
    data: &[u8],
    out: &mut [u8],
    espnow_pkt: bool,
) -> Result<usize, CcmError> {
    ccmp_decrypt_frame(tk, hdr, data, out, espnow_pkt)
}

/// Encrypts an 802.11 frame in-place using CCMP.
///
/// Validates all header, length, `keyid`, and non-zero `pn` invariants before
/// mutating `frame`.
///
/// # Errors
///
/// Returns [`CcmError`] if `keyid > 3`, `pn` is all-zero, header/buffer lengths
/// are invalid or overflow, or payload length exceeds `u16::MAX`.
pub fn ccmp_encrypt_frame(
    tk: &[u8; 16],
    frame: &mut [u8],
    len: usize,
    hdrlen: usize,
    pn: &[u8; 6],
    keyid: u8,
) -> Result<usize, CcmError> {
    if keyid > 3 {
        return Err(CcmError::InvalidKeyId);
    }
    if *pn == [0u8; 6] {
        return Err(CcmError::InvalidPacketNumber);
    }
    if hdrlen < 24 || len < hdrlen {
        return Err(CcmError::InvalidHeader);
    }
    let out_len = len.checked_add(16).ok_or(CcmError::BufferTooSmall)?;
    if frame.len() < out_len {
        return Err(CcmError::BufferTooSmall);
    }
    let plen = len - hdrlen;
    if plen > u16::MAX as usize {
        return Err(CcmError::InvalidInputLength);
    }

    // Build CCMP IV on the stack and validate AAD/nonce before mutating `frame`.
    let iv = [
        pn[5], // PN0
        pn[4], // PN1
        0x00,
        0x20 | (keyid << 6),
        pn[3], // PN2
        pn[2], // PN3
        pn[1], // PN4
        pn[0], // PN5
    ];
    let (mut aad, aad_len, mut nonce) = ccmp_aad_nonce(&frame[..hdrlen], &iv)?;
    if aad_len > MAX_CCM_AAD_LEN {
        zeroize_bytes(&mut aad);
        zeroize_bytes(&mut nonce);
        return Err(CcmError::InvalidInputLength);
    }

    // All validation succeeded; mutate `frame` in-place.
    frame[1] |= 0x40;
    frame.copy_within(hdrlen..len, hdrlen + 8);
    frame[hdrlen..hdrlen + 8].copy_from_slice(&iv);

    let mut crypt = [0u8; 16];
    let aes = Aes128Key::new(tk);
    let mut x = [0u8; 16];
    let mut b0 = [0u8; 16];

    b0[0] = if aad_len > 0 { 0x40 } else { 0 } | (3 << 3) | 1; // M=8, L=2
    b0[1..14].copy_from_slice(&nonce);
    b0[14..16].copy_from_slice(&(plen as u16).to_be_bytes());
    aes.encrypt_block(&b0, &mut x);

    // AAD
    let mut aad_block = [0u8; 16];
    aad_block[0..2].copy_from_slice(&(aad_len as u16).to_be_bytes());
    let to_copy = (16 - 2).min(aad_len);
    aad_block[2..2 + to_copy].copy_from_slice(&aad[..to_copy]);
    for (xb, ab) in x.iter_mut().zip(aad_block.iter()) {
        *xb ^= *ab;
    }
    let in_blk = x;
    aes.encrypt_block(&in_blk, &mut x);

    let mut off = to_copy;
    while off < aad_len {
        aad_block.fill(0);
        let chunk = (aad_len - off).min(16);
        aad_block[..chunk].copy_from_slice(&aad[off..off + chunk]);
        for (xb, ab) in x.iter_mut().zip(aad_block.iter()) {
            *xb ^= *ab;
        }
        let in_blk = x;
        aes.encrypt_block(&in_blk, &mut x);
        off += chunk;
    }

    // CBC-MAC over plaintext
    let mut p_off = 0;
    while p_off < plen {
        let chunk = (plen - p_off).min(16);
        for i in 0..chunk {
            x[i] ^= frame[hdrlen + 8 + p_off + i];
        }
        let in_blk = x;
        aes.encrypt_block(&in_blk, &mut x);
        p_off += chunk;
    }

    // CTR encrypt payload in-place
    let mut a = [0u8; 16];
    a[0] = 1;
    a[1..14].copy_from_slice(&nonce);
    let mut counter = 1u16;
    p_off = 0;
    while p_off < plen {
        a[14..16].copy_from_slice(&counter.to_be_bytes());
        aes.encrypt_block(&a, &mut crypt);
        let chunk = (plen - p_off).min(16);
        for i in 0..chunk {
            frame[hdrlen + 8 + p_off + i] ^= crypt[i];
        }
        p_off += chunk;
        if p_off < plen {
            counter = match counter.checked_add(1) {
                Some(next) => next,
                None => {
                    zeroize_bytes(&mut frame[hdrlen + 8..hdrlen + 8 + plen]);
                    zeroize_bytes(&mut crypt);
                    zeroize_bytes(&mut x);
                    zeroize_bytes(&mut b0);
                    zeroize_bytes(&mut aad_block);
                    zeroize_bytes(&mut a);
                    zeroize_bytes(&mut aad);
                    zeroize_bytes(&mut nonce);
                    return Err(CcmError::CounterOverflow);
                }
            };
        }
    }

    // Encrypt MIC tag and append at end
    a[14..16].copy_from_slice(&0u16.to_be_bytes());
    aes.encrypt_block(&a, &mut crypt);
    for i in 0..8 {
        frame[hdrlen + 8 + plen + i] = x[i] ^ crypt[i];
    }

    zeroize_bytes(&mut crypt);
    zeroize_bytes(&mut x);
    zeroize_bytes(&mut b0);
    zeroize_bytes(&mut aad_block);
    zeroize_bytes(&mut a);
    zeroize_bytes(&mut aad);
    zeroize_bytes(&mut nonce);

    Ok(out_len)
}

/// Alias for `ccmp_encrypt_frame`.
#[inline]
pub fn ccmp_encrypt(
    tk: &[u8; 16],
    frame: &mut [u8],
    len: usize,
    hdrlen: usize,
    pn: &[u8; 6],
    keyid: u8,
) -> Result<usize, CcmError> {
    ccmp_encrypt_frame(tk, frame, len, hdrlen, pn, keyid)
}

/// Errors that can occur during IEEE 802.11i CCMP MPDU encryption and
/// decryption.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CcmpMpduError {
    /// Provided output buffer is too small to hold the result.
    BufferTooSmall,
    /// 802.11 MAC header or CCMP parameter is malformed or invalid.
    InvalidHeader,
    /// Frame format is unsupported (e.g., 4-address WDS or Control/Extension).
    UnsupportedFrame,
    /// Protected Frame flag (bit 14) is missing on an encrypted MPDU.
    EncryptedFlagMissing,
    /// CCMP-128 Message Integrity Code (MIC) verification failed.
    MicMismatch,
    /// Packet Number is zero or exceeds the 48-bit CCMP range.
    InvalidPacketNumber,
    /// Key ID exceeds the 2-bit IEEE 802.11 range (`0..=3`).
    InvalidKeyId,
    /// Payload or AAD length exceeds RFC 3610 `L = 2` bounds.
    InvalidInputLength,
}

impl core::fmt::Display for CcmpMpduError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::BufferTooSmall => write!(f, "buffer too small"),
            Self::InvalidHeader => write!(f, "invalid 802.11 header"),
            Self::UnsupportedFrame => write!(f, "unsupported frame format"),
            Self::EncryptedFlagMissing => write!(f, "encrypted flag missing"),
            Self::MicMismatch => write!(f, "CCMP MIC authentication failed"),
            Self::InvalidPacketNumber => write!(f, "invalid CCMP packet number"),
            Self::InvalidKeyId => write!(f, "invalid CCMP key ID"),
            Self::InvalidInputLength => write!(f, "CCMP payload or AAD length out of bounds"),
        }
    }
}

fn map_ccm_error(err: CcmError) -> CcmpMpduError {
    match err {
        CcmError::BufferTooSmall => CcmpMpduError::BufferTooSmall,
        CcmError::InvalidInputLength | CcmError::CounterOverflow => {
            CcmpMpduError::InvalidInputLength
        }
        CcmError::InvalidHeader | CcmError::InvalidTagLength | CcmError::InvalidRc4Param => {
            CcmpMpduError::InvalidHeader
        }
        CcmError::InvalidPacketNumber => CcmpMpduError::InvalidPacketNumber,
        CcmError::InvalidKeyId => CcmpMpduError::InvalidKeyId,
        CcmError::UnsupportedFrame => CcmpMpduError::UnsupportedFrame,
        CcmError::AuthFailed => CcmpMpduError::MicMismatch,
    }
}

/// Computes deterministic 802.11 MAC header length and frame flags for
/// 3-address CCMP Data and Management MPDUs (accounting for QoS and `+HTC`).
fn parse_mpdu_hdr_meta(header: &[u8]) -> Result<(usize, u8, bool), CcmpMpduError> {
    if header.len() < 24 {
        return Err(CcmpMpduError::InvalidHeader);
    }
    let frame_type = (header[0] & 0x0C) >> 2;
    if frame_type != 0 && frame_type != 2 {
        return Err(CcmpMpduError::UnsupportedFrame);
    }
    if (header[1] & 0x03) == 0x03 {
        return Err(CcmpMpduError::UnsupportedFrame);
    }
    let is_qos = frame_type == 2 && (header[0] & 0x80) != 0;
    let has_htc = (is_qos || frame_type == 0) && (header[1] & 0x80) != 0;
    let hdr_len = 24 + if is_qos { 2 } else { 0 } + if has_htc { 4 } else { 0 };
    if header.len() < hdr_len {
        return Err(CcmpMpduError::InvalidHeader);
    }
    Ok((hdr_len, frame_type, is_qos))
}

/// Constructs a 13-byte CCMP-128 Nonce per IEEE 802.11i / RFC 3610.
///
/// Nonce layout (13 bytes):
/// `[Priority (1 byte), Transmitter Address A2 (6 bytes), Packet Number PN (6
/// bytes BE)]`.
#[must_use]
pub fn ccmp_build_nonce(ta: &[u8; 6], pn: u64, priority: u8) -> [u8; 13] {
    [
        priority,
        ta[0],
        ta[1],
        ta[2],
        ta[3],
        ta[4],
        ta[5],
        ((pn >> 40) & 0xff) as u8,
        ((pn >> 32) & 0xff) as u8,
        ((pn >> 24) & 0xff) as u8,
        ((pn >> 16) & 0xff) as u8,
        ((pn >> 8) & 0xff) as u8,
        (pn & 0xff) as u8,
    ]
}

/// Constructs the Additional Authentication Data (AAD) for CCMP-128 per
/// IEEE 802.11-2016 Clause 12.5.3.3.3.
///
/// # Errors
///
/// Returns [`CcmpMpduError::InvalidHeader`] if `header` is shorter than the
/// required 802.11 header length (24 B base, +2 B QoS, +4 B `+HTC`).
/// Returns [`CcmpMpduError::UnsupportedFrame`] if the frame is a 4-address
/// frame (`ToDS == 1` and `FromDS == 1`) or a Control/Extension frame.
pub fn ccmp_build_aad(header: &[u8], out: &mut [u8; 30]) -> Result<usize, CcmpMpduError> {
    let (_hdr_len, frame_type, is_qos) = parse_mpdu_hdr_meta(header)?;
    let is_data = frame_type == 2;

    out.fill(0);
    // Mask Subtype bits 4..6 (0x70) only on Data frames; keep on Mgmt frames.
    out[0] = if is_data { header[0] & 0x8F } else { header[0] };
    // Mask +HTC/Order on QoS Data; clear Retry/PwrMgt/MoreData, set Protected.
    out[1] = if is_qos {
        (header[1] & 0x47) | 0x40
    } else {
        (header[1] & 0xC7) | 0x40
    };
    out[2..20].copy_from_slice(&header[4..22]);
    out[20] = header[22] & 0x0F;
    out[21] = 0x00;

    if is_qos {
        out[22] = header[24] & 0x0F;
        out[23] = 0x00;
        Ok(24)
    } else {
        Ok(22)
    }
}

/// Encrypts an 802.11 MPDU payload using CCMP-128 and constructs the output
/// MPDU.
///
/// Output format:
/// `[MAC Header (24/26/28/30 B) | CCMP Header (8 B) | Ciphertext | MIC (8 B)]`.
///
/// # Errors
///
/// Returns [`CcmpMpduError::InvalidKeyId`] if `key_id > 3`.
/// Returns [`CcmpMpduError::InvalidPacketNumber`] if `pn` is 0 or `> 2^48 - 1`.
/// Returns [`CcmpMpduError::InvalidHeader`] if `mac_header` is too short.
/// Returns [`CcmpMpduError::UnsupportedFrame`] for WDS/Control/Extension.
/// Returns [`CcmpMpduError::InvalidInputLength`] if plaintext exceeds `65535`.
/// Returns [`CcmpMpduError::BufferTooSmall`] if `out_mpdu` is too small.
pub fn ccmp_encrypt_mpdu(
    tk: &[u8; 16],
    key_id: u8,
    pn: u64,
    mac_header: &[u8],
    plaintext: &[u8],
    out_mpdu: &mut [u8],
) -> Result<usize, CcmpMpduError> {
    if key_id > 3 {
        return Err(CcmpMpduError::InvalidKeyId);
    }
    if pn == 0 || pn > MAX_CCMP_PN {
        return Err(CcmpMpduError::InvalidPacketNumber);
    }
    if plaintext.len() > u16::MAX as usize {
        return Err(CcmpMpduError::InvalidInputLength);
    }
    let (hdr_len, frame_type, is_qos) = parse_mpdu_hdr_meta(mac_header)?;

    let total_len = hdr_len
        .checked_add(8)
        .and_then(|v| v.checked_add(plaintext.len()))
        .and_then(|v| v.checked_add(8))
        .ok_or(CcmpMpduError::BufferTooSmall)?;

    if out_mpdu.len() < total_len {
        return Err(CcmpMpduError::BufferTooSmall);
    }

    out_mpdu[..hdr_len].copy_from_slice(&mac_header[..hdr_len]);
    out_mpdu[1] |= 0x40;

    let ccmp_hdr_start = hdr_len;
    out_mpdu[ccmp_hdr_start] = pn as u8;
    out_mpdu[ccmp_hdr_start + 1] = (pn >> 8) as u8;
    out_mpdu[ccmp_hdr_start + 2] = 0x00;
    out_mpdu[ccmp_hdr_start + 3] = 0x20 | (key_id << 6);
    out_mpdu[ccmp_hdr_start + 4] = (pn >> 16) as u8;
    out_mpdu[ccmp_hdr_start + 5] = (pn >> 24) as u8;
    out_mpdu[ccmp_hdr_start + 6] = (pn >> 32) as u8;
    out_mpdu[ccmp_hdr_start + 7] = (pn >> 40) as u8;

    let mut ta = [0u8; 6];
    ta.copy_from_slice(&mac_header[10..16]);
    let priority = if is_qos {
        mac_header[24] & 0x0F
    } else if frame_type == 0 {
        0x10
    } else {
        0
    };

    let mut nonce = ccmp_build_nonce(&ta, pn, priority);
    let mut aad = [0u8; 30];
    let aad_len = ccmp_build_aad(&out_mpdu[..hdr_len], &mut aad)?;

    let payload_start = hdr_len + 8;
    let (crypt_out, tag_out) = out_mpdu[payload_start..total_len].split_at_mut(plaintext.len());

    let res = aes_ccm_ae(
        tk,
        &nonce,
        8,
        plaintext,
        &aad[..aad_len],
        crypt_out,
        tag_out,
    )
    .map_err(map_ccm_error);

    zeroize_bytes(&mut ta);
    zeroize_bytes(&mut nonce);
    zeroize_bytes(&mut aad);
    res?;

    Ok(total_len)
}

/// Decrypts a CCMP-128 protected MPDU and extracts plaintext, Key ID, and
/// Packet Number.
///
/// # Errors
///
/// Returns [`CcmpMpduError::BufferTooSmall`] if `mpdu` is shorter than header +
/// overhead or `out_plaintext` is too small.
/// Returns [`CcmpMpduError::InvalidHeader`] if `ExtIV` bit (bit 5) in CCMP
/// header is not set or the MAC header is truncated.
/// Returns [`CcmpMpduError::UnsupportedFrame`] if frame is 4-address WDS or
/// Control/Extension format.
/// Returns [`CcmpMpduError::EncryptedFlagMissing`] if Protected Frame flag
/// (bit 14) is not set.
/// Returns [`CcmpMpduError::InvalidPacketNumber`] if extracted PN is zero.
/// Returns [`CcmpMpduError::InvalidInputLength`] if payload exceeds `u16::MAX`.
/// Returns [`CcmpMpduError::MicMismatch`] if CCMP-128 MIC check fails.
pub fn ccmp_decrypt_mpdu(
    tk: &[u8; 16],
    mpdu: &[u8],
    out_plaintext: &mut [u8],
) -> Result<(usize, u8, u64), CcmpMpduError> {
    if mpdu.len() < 24 {
        return Err(CcmpMpduError::BufferTooSmall);
    }
    let (hdr_len, frame_type, is_qos) = parse_mpdu_hdr_meta(mpdu)?;

    let min_len = hdr_len
        .checked_add(16)
        .ok_or(CcmpMpduError::BufferTooSmall)?;
    if mpdu.len() < min_len {
        return Err(CcmpMpduError::BufferTooSmall);
    }
    if (mpdu[1] & 0x40) == 0 {
        return Err(CcmpMpduError::EncryptedFlagMissing);
    }
    if (mpdu[hdr_len + 3] & 0x20) == 0 {
        return Err(CcmpMpduError::InvalidHeader);
    }

    let key_id = (mpdu[hdr_len + 3] >> 6) & 0x03;
    let iv = &mpdu[hdr_len..hdr_len + 8];
    let pn = (u64::from(iv[0]))
        | (u64::from(iv[1]) << 8)
        | (u64::from(iv[4]) << 16)
        | (u64::from(iv[5]) << 24)
        | (u64::from(iv[6]) << 32)
        | (u64::from(iv[7]) << 40);
    if pn == 0 || pn > MAX_CCMP_PN {
        return Err(CcmpMpduError::InvalidPacketNumber);
    }

    let pt_len = mpdu.len() - min_len;
    if pt_len > u16::MAX as usize {
        return Err(CcmpMpduError::InvalidInputLength);
    }
    if out_plaintext.len() < pt_len {
        return Err(CcmpMpduError::BufferTooSmall);
    }

    let mut ta = [0u8; 6];
    ta.copy_from_slice(&mpdu[10..16]);
    let priority = if is_qos {
        mpdu[24] & 0x0F
    } else if frame_type == 0 {
        0x10
    } else {
        0
    };

    let mut nonce = ccmp_build_nonce(&ta, pn, priority);
    let mut aad = [0u8; 30];
    let aad_len = ccmp_build_aad(&mpdu[..hdr_len], &mut aad)?;

    let ciphertext = &mpdu[hdr_len + 8..hdr_len + 8 + pt_len];
    let mic = &mpdu[hdr_len + 8 + pt_len..hdr_len + 16 + pt_len];

    let res = aes_ccm_ad(
        tk,
        &nonce,
        8,
        ciphertext,
        &aad[..aad_len],
        mic,
        &mut out_plaintext[..pt_len],
    )
    .map_err(map_ccm_error);

    zeroize_bytes(&mut ta);
    zeroize_bytes(&mut nonce);
    zeroize_bytes(&mut aad);
    res?;

    Ok((pt_len, key_id, pn))
}

/// Checked RC4 stream cipher with initial byte skip (RFC 4345 / WPA1 v1).
///
/// # Errors
///
/// Returns [`CcmError::InvalidRc4Param`] if `key` is empty, `key.len() > 256`,
/// or `skip` is outside `256..=4096`.
pub fn rc4_skip_checked(key: &[u8], skip: usize, data: &mut [u8]) -> Result<(), CcmError> {
    if key.is_empty() || key.len() > 256 || !(256..=4096).contains(&skip) {
        return Err(CcmError::InvalidRc4Param);
    }
    let mut s = [0u8; 256];
    for (i, v) in s.iter_mut().enumerate() {
        *v = i as u8;
    }
    let mut j = 0u8;
    for i in 0..256 {
        j = j.wrapping_add(s[i]).wrapping_add(key[i % key.len()]);
        s.swap(i, j as usize);
    }

    let mut i = 0u8;
    j = 0;
    for _ in 0..skip {
        i = i.wrapping_add(1);
        j = j.wrapping_add(s[i as usize]);
        s.swap(i as usize, j as usize);
    }

    for b in data.iter_mut() {
        i = i.wrapping_add(1);
        j = j.wrapping_add(s[i as usize]);
        s.swap(i as usize, j as usize);
        let idx = s[i as usize].wrapping_add(s[j as usize]);
        *b ^= s[idx as usize];
    }
    zeroize_bytes(&mut s);
    Ok(())
}

/// RC4 stream cipher with initial byte skip (RFC 4345 / legacy WPA1 v1).
///
/// Fails closed by volatile-zeroizing `data` if `key` or `skip` fails
/// [`rc4_skip_checked`] bounds validation.
pub fn rc4_skip(key: &[u8], skip: usize, data: &mut [u8]) {
    if rc4_skip_checked(key, skip, data).is_err() {
        zeroize_bytes(data);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rfc3610_vector_1() {
        let key = [
            0xc0, 0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xcb, 0xcc, 0xcd,
            0xce, 0xcf,
        ];
        let nonce = [
            0x00, 0x00, 0x00, 0x03, 0x02, 0x01, 0x00, 0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5,
        ];
        let aad = [0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07];
        let plain = [
            0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15,
            0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
        ];
        let expected_cipher = [
            0x58, 0x8c, 0x97, 0x9a, 0x61, 0xc6, 0x63, 0xd2, 0xf0, 0x66, 0xd0, 0xc2, 0xc0, 0xf9,
            0x89, 0x80, 0x6d, 0x5f, 0x6b, 0x61, 0xda, 0xc3, 0x84, 0xe0,
        ];
        let expected_tag = [0x0f, 0xe2, 0xa5, 0x21, 0xc4, 0xf8, 0xf9, 0x43];

        let mut crypt_out = [0u8; 24];
        let mut tag_out = [0u8; 8];
        aes_ccm_ae(&key, &nonce, 8, &plain, &aad, &mut crypt_out, &mut tag_out)
            .expect("aes_ccm_ae vector 1 failed");
        assert_eq!(crypt_out, expected_cipher);
        assert_eq!(tag_out, expected_tag);

        let mut plain_out = [0u8; 24];
        aes_ccm_ad(&key, &nonce, 8, &crypt_out, &aad, &tag_out, &mut plain_out)
            .expect("aes_ccm_ad vector 1 failed");
        assert_eq!(plain_out, plain);
    }

    #[test]
    fn test_ccmp_encrypt_decrypt_roundtrip() {
        let tk = [0x42u8; 16];
        let pn = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06];
        let mut frame_buf = [0u8; 64];
        // Minimal 24-byte 802.11 Data frame header
        frame_buf[0] = 0x08; // Data
        frame_buf[1] = 0x00;
        frame_buf[4..10].copy_from_slice(&[0x11, 0x22, 0x33, 0x44, 0x55, 0x66]); // Addr1
        frame_buf[10..16].copy_from_slice(&[0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff]); // Addr2
        frame_buf[16..22].copy_from_slice(&[0x11, 0x22, 0x33, 0x44, 0x55, 0x66]); // Addr3
        frame_buf[22..24].copy_from_slice(&[0x00, 0x00]); // Seq

        let payload = b"Hello IEEE 802.11i CCMP!";
        let hdrlen = 24;
        let plen = payload.len();
        frame_buf[hdrlen..hdrlen + plen].copy_from_slice(payload);
        let unenc_len = hdrlen + plen;

        let total_enc = ccmp_encrypt_frame(&tk, &mut frame_buf, unenc_len, hdrlen, &pn, 0)
            .expect("ccmp_encrypt_frame roundtrip failed");
        assert_eq!(total_enc, unenc_len + 16);

        let mut decrypted = [0u8; 32];
        let dlen = ccmp_decrypt_frame(
            &tk,
            &frame_buf[..hdrlen],
            &frame_buf[hdrlen..total_enc],
            &mut decrypted,
            false,
        )
        .expect("ccmp_decrypt_frame roundtrip failed");
        assert_eq!(dlen, plen);
        assert_eq!(&decrypted[..dlen], payload);
    }

    #[test]
    fn test_espnow_roundtrip_and_zero_tag_rejection() {
        let key = [0x55u8; 16];
        let nonce = [0x11u8; 13];
        let plain = b"espnow packet body";
        let mut crypt = [0u8; 18];
        let mut empty_tag = [0u8; 0];

        // M-006 / C2-001: m_len_tag == 0 MUST be rejected per RFC 3610.
        assert_eq!(
            aes_ccm_ae(&key, &nonce, 0, plain, &[], &mut crypt, &mut empty_tag),
            Err(CcmError::InvalidTagLength)
        );
        let mut decrypted = [0u8; 18];
        assert_eq!(
            aes_ccm_ad(&key, &nonce, 0, &crypt, &[], &empty_tag, &mut decrypted),
            Err(CcmError::InvalidTagLength)
        );

        // Authenticated 8-byte MIC ESP-NOW roundtrip via `ccmp_decrypt_frame`.
        let mut hdr = [0u8; 24];
        hdr[0] = 0xD0; // Management Action frame
        hdr[1] = 0x40; // Protected
        hdr[4..10].copy_from_slice(&[0x11, 0x22, 0x33, 0x44, 0x55, 0x66]);
        hdr[10..16].copy_from_slice(&[0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff]);
        hdr[16..22].copy_from_slice(&[0xff; 6]);
        let mut data = [0u8; 8 + 18 + 8];
        data[0] = 0x01; // PN0 = 1
        data[3] = 0x20; // ExtIV = 1
        let (aad, aad_len, mut espnow_nonce) = ccmp_aad_nonce(&hdr, &data[..8]).expect("aad_nonce");
        espnow_nonce[0] = 0; // ESP-NOW nonce flags = 0
        let (ct_out, tag_out) = data[8..].split_at_mut(plain.len());
        aes_ccm_ae(
            &key,
            &espnow_nonce,
            8,
            plain,
            &aad[..aad_len],
            ct_out,
            tag_out,
        )
        .expect("espnow authenticated encrypt");

        let dlen = ccmp_decrypt_frame(&key, &hdr, &data, &mut decrypted, true)
            .expect("espnow authenticated decrypt");
        assert_eq!(dlen, plain.len());
        assert_eq!(&decrypted[..dlen], plain);

        // Tampering with MIC MUST fail even for espnow_pkt == true, and scrub output.
        data[8 + plain.len()] ^= 0x01;
        assert_eq!(
            ccmp_decrypt_frame(&key, &hdr, &data, &mut decrypted, true),
            Err(CcmError::AuthFailed)
        );
        assert_eq!(decrypted, [0u8; 18]);
    }

    #[test]
    fn test_rc4_skip() {
        let key = b"KeyMaterialForWPA1";
        let mut data = *b"Plaintext";
        assert!(rc4_skip_checked(key, 256, &mut data).is_ok());
        assert_ne!(&data, b"Plaintext");
        rc4_skip(key, 256, &mut data);
        assert_eq!(&data, b"Plaintext");

        // Bounds enforcement (C2-036)
        assert_eq!(
            rc4_skip_checked(&[], 256, &mut data),
            Err(CcmError::InvalidRc4Param)
        );
        assert_eq!(
            rc4_skip_checked(&[0u8; 257], 256, &mut data),
            Err(CcmError::InvalidRc4Param)
        );
        assert_eq!(
            rc4_skip_checked(key, 0, &mut data),
            Err(CcmError::InvalidRc4Param)
        );
        assert_eq!(
            rc4_skip_checked(key, 4097, &mut data),
            Err(CcmError::InvalidRc4Param)
        );

        // Fail-closed scrubbing on invalid parameters via legacy rc4_skip
        let mut secret = *b"Sensitive";
        rc4_skip(&[], 256, &mut secret);
        assert_eq!(secret, [0u8; 9]);
    }

    #[test]
    fn test_ccmp_pn_keyid_overflow_and_pre_mutation_validation() {
        let tk = [0x42u8; 16];
        let mut hdr = [0u8; 24];
        hdr[0] = 0x08; // Data
        hdr[4..10].copy_from_slice(&[1, 2, 3, 4, 5, 6]);
        hdr[10..16].copy_from_slice(&[7, 8, 9, 10, 11, 12]);
        hdr[16..22].copy_from_slice(&[13, 14, 15, 16, 17, 18]);
        let mut mpdu_buf = [0u8; 64];

        // PN == 0 and PN > 48-bit MUST be rejected (M-011 / C2-006)
        assert_eq!(
            ccmp_encrypt_mpdu(&tk, 0, 0, &hdr, b"abc", &mut mpdu_buf),
            Err(CcmpMpduError::InvalidPacketNumber)
        );
        assert_eq!(
            ccmp_encrypt_mpdu(&tk, 0, 1u64 << 48, &hdr, b"abc", &mut mpdu_buf),
            Err(CcmpMpduError::InvalidPacketNumber)
        );
        // key_id > 3 MUST be rejected (C2-055)
        assert_eq!(
            ccmp_encrypt_mpdu(&tk, 4, 1, &hdr, b"abc", &mut mpdu_buf),
            Err(CcmpMpduError::InvalidKeyId)
        );

        // `ccmp_encrypt_frame`: overflow, PN=0, key_id>3, no mutation on error.
        let mut frame = [0u8; 64];
        frame[0] = 0x88; // QoS Data requires 26-byte header
        frame[1] = 0x00;
        frame[24..28].copy_from_slice(b"ABCD");
        let snapshot = frame;
        assert_eq!(
            ccmp_encrypt_frame(&tk, &mut frame, 28, 24, &[0, 0, 0, 0, 0, 1], 0),
            Err(CcmError::InvalidHeader)
        );
        assert_eq!(frame, snapshot, "frame must not be mutated on error");
        assert_eq!(
            ccmp_encrypt_frame(&tk, &mut frame, usize::MAX, 24, &[0, 0, 0, 0, 0, 1], 0),
            Err(CcmError::BufferTooSmall)
        );
        assert_eq!(
            ccmp_encrypt_frame(&tk, &mut frame, 28, 24, &[0; 6], 0),
            Err(CcmError::InvalidPacketNumber)
        );
        assert_eq!(
            ccmp_encrypt_frame(&tk, &mut frame, 28, 24, &[0, 0, 0, 0, 0, 1], 4),
            Err(CcmError::InvalidKeyId)
        );
    }

    #[test]
    fn test_ccmp_mgmt_aad_nonce_and_qos_htc_roundtrip() {
        let tk = [0x99u8; 16];
        // Management Deauthentication frame (Type=0, Subtype=0xC -> 0xC0)
        let mut mgmt_hdr = [0u8; 24];
        mgmt_hdr[0] = 0xC0;
        mgmt_hdr[1] = 0x00;
        mgmt_hdr[4..10].copy_from_slice(&[0xaa; 6]);
        mgmt_hdr[10..16].copy_from_slice(&[0xbb; 6]);
        mgmt_hdr[16..22].copy_from_slice(&[0xcc; 6]);
        let mut aad = [0u8; 30];
        let aad_len = ccmp_build_aad(&mgmt_hdr, &mut aad).expect("mgmt aad");
        assert_eq!(aad_len, 22);
        // Subtype bits 0x70 MUST NOT be masked on Management frames (M-054)
        assert_eq!(aad[0], 0xC0);

        let mut mpdu = [0u8; 64];
        let mlen =
            ccmp_encrypt_mpdu(&tk, 0, 7, &mgmt_hdr, b"deauth", &mut mpdu).expect("encrypt mgmt");
        // Verify nonce flag 0x10 by comparing against `ccmp_decrypt_frame`.
        let mut dec = [0u8; 16];
        let dlen = ccmp_decrypt_frame(&tk, &mpdu[..24], &mpdu[24..mlen], &mut dec, false)
            .expect("decrypt mgmt via frame API");
        assert_eq!(&dec[..dlen], b"deauth");
        // Changing management subtype in FC MUST fail MIC check
        let mut forged = mpdu;
        forged[0] = 0xA0; // Disassociation instead of Deauth
        assert_eq!(
            ccmp_decrypt_mpdu(&tk, &forged[..mlen], &mut dec),
            Err(CcmpMpduError::MicMismatch)
        );

        // QoS + HTC (30-byte header: 24 + 2 QoS + 4 HTC)
        let mut htc_hdr = [0u8; 30];
        htc_hdr[0] = 0x88; // QoS Data
        htc_hdr[1] = 0x81; // ToDS=1, +HTC/Order=1
        htc_hdr[4..10].copy_from_slice(&[0x11; 6]);
        htc_hdr[10..16].copy_from_slice(&[0x22; 6]);
        htc_hdr[16..22].copy_from_slice(&[0x33; 6]);
        htc_hdr[24] = 0x03; // TID 3
        htc_hdr[26..30].copy_from_slice(&[0xde, 0xad, 0xbe, 0xef]); // HT Control
        let aad_len_htc = ccmp_build_aad(&htc_hdr, &mut aad).expect("qos+htc aad");
        assert_eq!(aad_len_htc, 24);
        assert_eq!(aad[1] & 0x80, 0, "Order/+HTC bit must be masked in QoS AAD");

        let mut htc_mpdu = [0u8; 96];
        let hlen = ccmp_encrypt_mpdu(&tk, 0, 9, &htc_hdr, b"qos+htc", &mut htc_mpdu)
            .expect("encrypt qos+htc");
        assert_eq!(hlen, 30 + 8 + 7 + 8);
        let (pt_len, kid, pn) =
            ccmp_decrypt_mpdu(&tk, &htc_mpdu[..hlen], &mut dec).expect("decrypt qos+htc");
        assert_eq!((pt_len, kid, pn), (7, 0, 9));
        assert_eq!(&dec[..pt_len], b"qos+htc");

        // Control frame (Type=1) MUST be rejected
        let mut ctrl_hdr = [0u8; 24];
        ctrl_hdr[0] = 0xD4; // ACK (Type=1)
        assert_eq!(
            ccmp_build_aad(&ctrl_hdr, &mut aad),
            Err(CcmpMpduError::UnsupportedFrame)
        );
    }

    #[test]
    fn test_ccm_length_bounds_enforced() {
        let key = [0x11u8; 16];
        let nonce = [0x22u8; 13];
        let big_aad = [0u8; 0xFF00];
        let mut ct = [0u8; 4];
        let mut tag = [0u8; 8];
        assert_eq!(
            aes_ccm_ae(&key, &nonce, 8, b"test", &big_aad, &mut ct, &mut tag),
            Err(CcmError::InvalidInputLength)
        );
        assert_eq!(
            aes_ccm_ad(&key, &nonce, 8, b"test", &big_aad, &tag, &mut ct),
            Err(CcmError::InvalidInputLength)
        );
    }

    #[test]
    fn test_ccmp_mpdu_non_qos_roundtrip() {
        let tk = [0x5au8; 16];
        let key_id = 1;
        let pn = 0x1234_5678_9abc_u64;
        let mut header = [0u8; 24];
        header[0] = 0x08; // Data
        header[1] = 0x00;
        header[4..10].copy_from_slice(&[0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0x01]);
        header[10..16].copy_from_slice(&[0x11, 0x22, 0x33, 0x44, 0x55, 0x66]);
        header[16..22].copy_from_slice(&[0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0x02]);
        header[22..24].copy_from_slice(&[0x34, 0x12]); // Fragment + Seq

        let payload = b"CCMP MPDU Non-QoS Test Payload";
        let mut mpdu_buf = [0u8; 128];
        let mpdu_len = ccmp_encrypt_mpdu(&tk, key_id, pn, &header, payload, &mut mpdu_buf)
            .expect("encrypt non-qos");
        assert_eq!(mpdu_len, 24 + 8 + payload.len() + 8);
        assert_ne!(mpdu_buf[1] & 0x40, 0);

        let mut out_pt = [0u8; 64];
        let (pt_len, rx_kid, rx_pn) =
            ccmp_decrypt_mpdu(&tk, &mpdu_buf[..mpdu_len], &mut out_pt).expect("decrypt non-qos");
        assert_eq!(pt_len, payload.len());
        assert_eq!(rx_kid, key_id);
        assert_eq!(rx_pn, pn);
        assert_eq!(&out_pt[..pt_len], payload);
    }

    #[test]
    fn test_ccmp_mpdu_qos_roundtrip() {
        let tk = [0x7bu8; 16];
        let key_id = 2;
        let pn = 42u64;
        let mut header = [0u8; 26];
        header[0] = 0x88; // QoS Data
        header[1] = 0x01; // ToDS=1
        header[4..10].copy_from_slice(&[0x00, 0x11, 0x22, 0x33, 0x44, 0x55]);
        header[10..16].copy_from_slice(&[0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb]);
        header[16..22].copy_from_slice(&[0xcc, 0xdd, 0xee, 0x00, 0x11, 0x22]);
        header[22..24].copy_from_slice(&[0x00, 0x00]);
        header[24] = 0x05; // TID 5
        header[25] = 0x00;

        let payload = b"QoS Voice Frame Payload";
        let mut mpdu_buf = [0u8; 128];
        let mpdu_len = ccmp_encrypt_mpdu(&tk, key_id, pn, &header, payload, &mut mpdu_buf)
            .expect("encrypt qos");
        assert_eq!(mpdu_len, 26 + 8 + payload.len() + 8);

        let mut out_pt = [0u8; 64];
        let (pt_len, rx_kid, rx_pn) =
            ccmp_decrypt_mpdu(&tk, &mpdu_buf[..mpdu_len], &mut out_pt).expect("decrypt qos");
        assert_eq!(pt_len, payload.len());
        assert_eq!(rx_kid, key_id);
        assert_eq!(rx_pn, pn);
        assert_eq!(&out_pt[..pt_len], payload);
    }

    #[test]
    fn test_ccmp_mpdu_tampered_ciphertext_rejection() {
        let tk = [0x33u8; 16];
        let mut header = [0u8; 24];
        header[0] = 0x08;
        header[4..10].copy_from_slice(&[1, 2, 3, 4, 5, 6]);
        header[10..16].copy_from_slice(&[7, 8, 9, 10, 11, 12]);
        header[16..22].copy_from_slice(&[13, 14, 15, 16, 17, 18]);
        let payload = b"Tamper Resistance Test";
        let mut mpdu_buf = [0u8; 128];
        let mpdu_len =
            ccmp_encrypt_mpdu(&tk, 0, 1, &header, payload, &mut mpdu_buf).expect("encrypt");

        // Tamper payload byte
        mpdu_buf[24 + 8] ^= 0x01;
        let mut out_pt = [0u8; 64];
        let res = ccmp_decrypt_mpdu(&tk, &mpdu_buf[..mpdu_len], &mut out_pt);
        assert_eq!(res, Err(CcmpMpduError::MicMismatch));
    }

    #[test]
    fn test_ccmp_mpdu_tampered_aad_rejection() {
        let tk = [0x33u8; 16];
        let mut header = [0u8; 24];
        header[0] = 0x08;
        header[4..10].copy_from_slice(&[1, 2, 3, 4, 5, 6]);
        header[10..16].copy_from_slice(&[7, 8, 9, 10, 11, 12]);
        header[16..22].copy_from_slice(&[13, 14, 15, 16, 17, 18]);
        let payload = b"AAD Tamper Test";
        let mut mpdu_buf = [0u8; 128];
        let mpdu_len =
            ccmp_encrypt_mpdu(&tk, 0, 1, &header, payload, &mut mpdu_buf).expect("encrypt");

        // Tamper A2 (Transmitter Address) in MAC header
        let mut tampered_a2 = mpdu_buf;
        tampered_a2[10] ^= 0x01;
        let mut out_pt = [0u8; 64];
        assert_eq!(
            ccmp_decrypt_mpdu(&tk, &tampered_a2[..mpdu_len], &mut out_pt),
            Err(CcmpMpduError::MicMismatch)
        );

        // Tamper SeqCtrl in MAC header
        let mut tampered_seq = mpdu_buf;
        tampered_seq[22] ^= 0x01;
        assert_eq!(
            ccmp_decrypt_mpdu(&tk, &tampered_seq[..mpdu_len], &mut out_pt),
            Err(CcmpMpduError::MicMismatch)
        );
    }

    #[test]
    fn test_ccmp_mpdu_missing_protected_or_extiv_rejection() {
        let tk = [0x33u8; 16];
        let mut header = [0u8; 24];
        header[0] = 0x08;
        header[4..10].copy_from_slice(&[1, 2, 3, 4, 5, 6]);
        header[10..16].copy_from_slice(&[7, 8, 9, 10, 11, 12]);
        header[16..22].copy_from_slice(&[13, 14, 15, 16, 17, 18]);
        let payload = b"Header Flag Validation";
        let mut mpdu_buf = [0u8; 128];
        let mpdu_len =
            ccmp_encrypt_mpdu(&tk, 0, 1, &header, payload, &mut mpdu_buf).expect("encrypt");

        // Clear Protected bit
        let mut no_prot = mpdu_buf;
        no_prot[1] &= !0x40;
        let mut out_pt = [0u8; 64];
        assert_eq!(
            ccmp_decrypt_mpdu(&tk, &no_prot[..mpdu_len], &mut out_pt),
            Err(CcmpMpduError::EncryptedFlagMissing)
        );

        // Clear ExtIV bit (bit 5 of byte 3 of CCMP header)
        let mut no_extiv = mpdu_buf;
        no_extiv[24 + 3] &= !0x20;
        assert_eq!(
            ccmp_decrypt_mpdu(&tk, &no_extiv[..mpdu_len], &mut out_pt),
            Err(CcmpMpduError::InvalidHeader)
        );
    }

    #[test]
    fn test_ccmp_mpdu_unsupported_wds_frame() {
        let tk = [0x33u8; 16];
        let mut header = [0u8; 24];
        header[0] = 0x08;
        header[1] = 0x03; // ToDS=1 and FromDS=1
        let payload = b"WDS";
        let mut mpdu_buf = [0u8; 64];
        assert_eq!(
            ccmp_encrypt_mpdu(&tk, 0, 1, &header, payload, &mut mpdu_buf),
            Err(CcmpMpduError::UnsupportedFrame)
        );
    }
}
