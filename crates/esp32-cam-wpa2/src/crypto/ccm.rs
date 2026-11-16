//! AES-CCM (RFC 3610), IEEE 802.11i CCMP, and RC4 implementations.
//!
//! Compliant with RFC 3610 and IEEE 802.11-2016 Clause 12.5.3.

use super::aes::Aes128Key;
use super::hash::ct_eq;

/// Error type for CCM and CCMP operations.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CcmError;

/// AES-CCM authenticated encryption (RFC 3610) with L=2.
pub fn aes_ccm_ae(
    key: &[u8; 16],
    nonce: &[u8; 13],
    m_len_tag: usize,
    plain: &[u8],
    aad: &[u8],
    crypt_out: &mut [u8],
    tag_out: &mut [u8],
) -> Result<(), CcmError> {
    if m_len_tag > 16 || (m_len_tag > 0 && m_len_tag < 4) || (!m_len_tag.is_multiple_of(2)) {
        return Err(CcmError);
    }
    if tag_out.len() < m_len_tag || crypt_out.len() < plain.len() {
        return Err(CcmError);
    }

    let aes = Aes128Key::new(key);
    let mut x = [0u8; 16];
    let mut b0 = [0u8; 16];

    // B_0 formatting: Flags | Nonce (13) | Length (2)
    b0[0] = if !aad.is_empty() { 0x40 } else { 0 };
    if m_len_tag >= 2 {
        b0[0] |= (((m_len_tag - 2) / 2) as u8) << 3;
    }
    b0[0] |= 1; // L - 1 = 2 - 1 = 1
    b0[1..14].copy_from_slice(nonce);
    b0[14..16].copy_from_slice(&(plain.len() as u16).to_be_bytes());

    aes.encrypt_block(&b0, &mut x);

    // Process AAD if present
    if !aad.is_empty() {
        let mut aad_block = [0u8; 16];
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
        counter = counter.wrapping_add(1);
        offset += chunk;
    }

    // Encrypt authentication tag
    if m_len_tag > 0 {
        a[14..16].copy_from_slice(&0u16.to_be_bytes());
        aes.encrypt_block(&a, &mut s);
        for i in 0..m_len_tag {
            tag_out[i] = x[i] ^ s[i];
        }
    }

    Ok(())
}

/// AES-CCM authenticated decryption (RFC 3610) with L=2.
pub fn aes_ccm_ad(
    key: &[u8; 16],
    nonce: &[u8; 13],
    m_len_tag: usize,
    crypt: &[u8],
    aad: &[u8],
    tag: &[u8],
    plain_out: &mut [u8],
) -> Result<(), CcmError> {
    if m_len_tag > 16 || (m_len_tag > 0 && m_len_tag < 4) || (!m_len_tag.is_multiple_of(2)) {
        return Err(CcmError);
    }
    if plain_out.len() < crypt.len() || tag.len() < m_len_tag {
        return Err(CcmError);
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
        counter = counter.wrapping_add(1);
        offset += chunk;
    }

    if m_len_tag == 0 {
        return Ok(());
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

    if !aad.is_empty() {
        let mut aad_block = [0u8; 16];
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

    if ct_eq(&expected_tag[..m_len_tag], &tag[..m_len_tag]) {
        Ok(())
    } else {
        plain_out[..crypt.len()].fill(0);
        Err(CcmError)
    }
}

/// Constructs CCMP AAD and Nonce from 802.11 header and 8-byte CCMP data
/// header.
pub fn ccmp_aad_nonce(hdr: &[u8], data: &[u8]) -> Result<([u8; 30], usize, [u8; 13]), CcmError> {
    if hdr.len() < 24 || data.len() < 8 {
        return Err(CcmError);
    }
    let mut fc = u16::from_le_bytes([hdr[0], hdr[1]]);
    let stype = (fc & 0x00f0) >> 4;
    let type_ = (fc & 0x000c) >> 2;
    let addr4 = (fc & 0x0300) == 0x0300;
    let mut qos = false;
    let mut nonce = [0u8; 13];

    if type_ == 2 {
        // Data frame
        fc &= !0x0070;
        if (stype & 0x08) != 0 {
            qos = true;
            fc &= !0x8000; // Mask ORDER
            let qc_offset = 24 + if addr4 { 6 } else { 0 };
            if hdr.len() > qc_offset {
                nonce[0] = hdr[qc_offset] & 0x0f;
            }
        }
    } else if type_ == 0 {
        // Management frame
        nonce[0] |= 0x10;
    }

    fc &= !(0x0800 | 0x1000 | 0x2000); // Clear RETRY, PWRMGT, MOREDATA
    fc |= 0x4000; // Set ISWEP (Protected Frame)

    let mut aad = [0u8; 30];
    aad[0..2].copy_from_slice(&fc.to_le_bytes());
    aad[2..20].copy_from_slice(&hdr[4..22]); // Addr1, Addr2, Addr3
    let seq = u16::from_le_bytes([hdr[22], hdr[23]]) & !0xfff0;
    aad[20..22].copy_from_slice(&seq.to_le_bytes());

    let mut pos = 22;
    if addr4 {
        if hdr.len() < 30 {
            return Err(CcmError);
        }
        aad[pos..pos + 6].copy_from_slice(&hdr[24..30]);
        pos += 6;
    }
    if qos {
        let qc_offset = 24 + if addr4 { 6 } else { 0 };
        if hdr.len() < qc_offset + 2 {
            return Err(CcmError);
        }
        aad[pos] = hdr[qc_offset] & !0x70 & !0x80;
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

/// Decrypts a CCMP-protected frame payload.
pub fn ccmp_decrypt_frame(
    tk: &[u8; 16],
    hdr: &[u8],
    data: &[u8],
    out: &mut [u8],
    espnow_pkt: bool,
) -> Result<usize, CcmError> {
    if data.len() < 16 {
        return Err(CcmError);
    }
    let mlen = data.len() - 16;
    if out.len() < mlen {
        return Err(CcmError);
    }

    let (aad, aad_len, mut nonce) = ccmp_aad_nonce(hdr, data)?;
    let tag_len = if espnow_pkt {
        nonce[0] = 0;
        0
    } else {
        8
    };

    aes_ccm_ad(
        tk,
        &nonce,
        tag_len,
        &data[8..8 + mlen],
        &aad[..aad_len],
        &data[8 + mlen..16 + mlen],
        &mut out[..mlen],
    )?;

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
pub fn ccmp_encrypt_frame(
    tk: &[u8; 16],
    frame: &mut [u8],
    len: usize,
    hdrlen: usize,
    pn: &[u8; 6],
    keyid: u8,
) -> Result<usize, CcmError> {
    if hdrlen < 24 || len < hdrlen || frame.len() < len + 16 {
        return Err(CcmError);
    }
    let plen = len - hdrlen;

    // Set Protected Frame bit in header
    frame[1] |= 0x40;

    // Shift payload 8 bytes forward to make room for CCMP IV header
    frame.copy_within(hdrlen..len, hdrlen + 8);

    // Write CCMP IV header
    frame[hdrlen] = pn[5]; // PN0
    frame[hdrlen + 1] = pn[4]; // PN1
    frame[hdrlen + 2] = 0x00;
    frame[hdrlen + 3] = 0x20 | ((keyid & 0x03) << 6);
    frame[hdrlen + 4] = pn[3]; // PN2
    frame[hdrlen + 5] = pn[2]; // PN3
    frame[hdrlen + 6] = pn[1]; // PN4
    frame[hdrlen + 7] = pn[0]; // PN5

    let (aad, aad_len, nonce) = ccmp_aad_nonce(&frame[..hdrlen], &frame[hdrlen..hdrlen + 8])?;

    // Encrypt in-place using temporary stack buffer for plain
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
        counter = counter.wrapping_add(1);
        p_off += chunk;
    }

    // Encrypt MIC tag and append at end
    a[14..16].copy_from_slice(&0u16.to_be_bytes());
    aes.encrypt_block(&a, &mut crypt);
    for i in 0..8 {
        frame[hdrlen + 8 + plen + i] = x[i] ^ crypt[i];
    }

    Ok(len + 16)
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
    /// Frame format is unsupported (e.g., 4-address WDS frames).
    UnsupportedFrame,
    /// Protected Frame flag (bit 14) is missing on an encrypted MPDU.
    EncryptedFlagMissing,
    /// CCMP-128 Message Integrity Code (MIC) verification failed.
    MicMismatch,
}

impl core::fmt::Display for CcmpMpduError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::BufferTooSmall => write!(f, "buffer too small"),
            Self::InvalidHeader => write!(f, "invalid 802.11 header"),
            Self::UnsupportedFrame => write!(f, "unsupported frame format"),
            Self::EncryptedFlagMissing => write!(f, "encrypted flag missing"),
            Self::MicMismatch => write!(f, "CCMP MIC authentication failed"),
        }
    }
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

/// Constructs the Additional Authentication Data (AAD) for CCMP-128.
///
/// # Errors
///
/// Returns [`CcmpMpduError::InvalidHeader`] if `header.len() < 24` or if a QoS
/// frame header has `header.len() < 26`.
/// Returns [`CcmpMpduError::UnsupportedFrame`] if the frame is a 4-address
/// frame (`ToDS == 1` and `FromDS == 1`).
pub fn ccmp_build_aad(header: &[u8], out: &mut [u8; 30]) -> Result<usize, CcmpMpduError> {
    if header.len() < 24 {
        return Err(CcmpMpduError::InvalidHeader);
    }
    if (header[1] & 0x03) == 0x03 {
        return Err(CcmpMpduError::UnsupportedFrame);
    }

    out.fill(0);
    out[0] = header[0] & 0x8F;
    out[1] = (header[1] & 0xC7) | 0x40;
    out[2..20].copy_from_slice(&header[4..22]);
    out[20] = header[22] & 0x0F;
    out[21] = 0x00;

    let is_qos = (header[0] & 0x8C) == 0x88;
    if is_qos {
        if header.len() < 26 {
            return Err(CcmpMpduError::InvalidHeader);
        }
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
/// `[MAC Header (24 or 26 B) | CCMP Header (8 B) | Encrypted Payload | MIC (8
/// B)]`.
///
/// # Errors
///
/// Returns [`CcmpMpduError::InvalidHeader`] if `mac_header` is too short.
/// Returns [`CcmpMpduError::UnsupportedFrame`] if 4-address frame format is
/// supplied.
/// Returns [`CcmpMpduError::BufferTooSmall`] if `out_mpdu` is too small to
/// contain the resulting MPDU.
pub fn ccmp_encrypt_mpdu(
    tk: &[u8; 16],
    key_id: u8,
    pn: u64,
    mac_header: &[u8],
    plaintext: &[u8],
    out_mpdu: &mut [u8],
) -> Result<usize, CcmpMpduError> {
    if mac_header.len() < 24 {
        return Err(CcmpMpduError::InvalidHeader);
    }
    if (mac_header[1] & 0x03) == 0x03 {
        return Err(CcmpMpduError::UnsupportedFrame);
    }

    let is_qos = (mac_header[0] & 0x8C) == 0x88;
    let hdr_len = if is_qos { 26 } else { 24 };
    if mac_header.len() < hdr_len {
        return Err(CcmpMpduError::InvalidHeader);
    }

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
    out_mpdu[ccmp_hdr_start + 3] = 0x20 | ((key_id & 0x03) << 6);
    out_mpdu[ccmp_hdr_start + 4] = (pn >> 16) as u8;
    out_mpdu[ccmp_hdr_start + 5] = (pn >> 24) as u8;
    out_mpdu[ccmp_hdr_start + 6] = (pn >> 32) as u8;
    out_mpdu[ccmp_hdr_start + 7] = (pn >> 40) as u8;

    let mut ta = [0u8; 6];
    ta.copy_from_slice(&mac_header[10..16]);
    let priority = if is_qos { mac_header[24] & 0x0F } else { 0 };

    let nonce = ccmp_build_nonce(&ta, pn, priority);
    let mut aad = [0u8; 30];
    let aad_len = ccmp_build_aad(&out_mpdu[..hdr_len], &mut aad)?;

    let payload_start = hdr_len + 8;
    let (crypt_out, tag_out) = out_mpdu[payload_start..total_len].split_at_mut(plaintext.len());

    aes_ccm_ae(
        tk,
        &nonce,
        8,
        plaintext,
        &aad[..aad_len],
        crypt_out,
        tag_out,
    )
    .map_err(|_| CcmpMpduError::BufferTooSmall)?;

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
/// header is not set.
/// Returns [`CcmpMpduError::UnsupportedFrame`] if frame is 4-address WDS
/// format.
/// Returns [`CcmpMpduError::EncryptedFlagMissing`] if Protected Frame flag
/// (bit 14) is not set.
/// Returns [`CcmpMpduError::MicMismatch`] if CCMP-128 MIC check fails.
pub fn ccmp_decrypt_mpdu(
    tk: &[u8; 16],
    mpdu: &[u8],
    out_plaintext: &mut [u8],
) -> Result<(usize, u8, u64), CcmpMpduError> {
    if mpdu.len() < 24 {
        return Err(CcmpMpduError::BufferTooSmall);
    }
    if (mpdu[1] & 0x03) == 0x03 {
        return Err(CcmpMpduError::UnsupportedFrame);
    }

    let is_qos = (mpdu[0] & 0x8C) == 0x88;
    let hdr_len = if is_qos { 26 } else { 24 };

    if mpdu.len() < hdr_len + 16 {
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

    let pt_len = mpdu.len() - hdr_len - 16;
    if out_plaintext.len() < pt_len {
        return Err(CcmpMpduError::BufferTooSmall);
    }

    let mut ta = [0u8; 6];
    ta.copy_from_slice(&mpdu[10..16]);
    let priority = if is_qos { mpdu[24] & 0x0F } else { 0 };

    let nonce = ccmp_build_nonce(&ta, pn, priority);
    let mut aad = [0u8; 30];
    let aad_len = ccmp_build_aad(&mpdu[..hdr_len], &mut aad)?;

    let ciphertext = &mpdu[hdr_len + 8..hdr_len + 8 + pt_len];
    let mic = &mpdu[hdr_len + 8 + pt_len..hdr_len + 16 + pt_len];

    aes_ccm_ad(
        tk,
        &nonce,
        8,
        ciphertext,
        &aad[..aad_len],
        mic,
        &mut out_plaintext[..pt_len],
    )
    .map_err(|_| CcmpMpduError::MicMismatch)?;

    Ok((pt_len, key_id, pn))
}

/// RC4 stream cipher with initial byte skip (RFC 4345 / WPA1 v1).
pub fn rc4_skip(key: &[u8], skip: usize, data: &mut [u8]) {
    if key.is_empty() {
        return;
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
    fn test_espnow_roundtrip() {
        let key = [0x55u8; 16];
        let nonce = [0x11u8; 13];
        let plain = b"espnow packet body";
        let mut crypt = [0u8; 18];
        let mut tag = [0u8; 0];
        aes_ccm_ae(&key, &nonce, 0, plain, &[], &mut crypt, &mut tag)
            .expect("aes_ccm_ae espnow failed");

        let mut decrypted = [0u8; 18];
        aes_ccm_ad(&key, &nonce, 0, &crypt, &[], &tag, &mut decrypted)
            .expect("aes_ccm_ad espnow failed");
        assert_eq!(&decrypted[..plain.len()], plain);
    }

    #[test]
    fn test_rc4_skip() {
        let key = b"Key";
        let mut data = *b"Plaintext";
        rc4_skip(key, 0, &mut data);
        assert_ne!(&data, b"Plaintext");
        rc4_skip(key, 0, &mut data);
        assert_eq!(&data, b"Plaintext");
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
