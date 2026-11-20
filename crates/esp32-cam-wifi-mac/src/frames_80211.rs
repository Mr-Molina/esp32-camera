//! IEEE 802.11 MAC frame parser, CCMP crypto, and Ethernet-II bridge.
//!
//! Provides zero-allocation, `#![no_std]` parsers and builders for 802.11
//! management frames (Probe, Auth, Assoc, Deauth, Block ACK ADDBA decline)
//! and data frames, bridging 802.11 MPDUs and Ethernet-II frames with
//! hardware CCMP encryption and LLC-SNAP translation.

use core::fmt;
use core::sync::atomic::compiler_fence;
use core::sync::atomic::Ordering;
use esp32_cam_wpa2::crypto::ccm::ccmp_decrypt_mpdu;
use esp32_cam_wpa2::crypto::ccm::ccmp_encrypt_mpdu;
use esp32_cam_wpa2::ie::parse_wpa_or_rsn_ie;

/// Maximum valid 48-bit IEEE 802.11i CCMP Packet Number (`2^48 - 1`).
pub const MAX_CCMP_PN: u64 = (1u64 << 48) - 1;

/// Volatile-zeros a byte slice to prevent dead-store elimination of secrets.
#[inline]
pub(crate) fn volatile_zeroize(buf: &mut [u8]) {
    for byte in buf.iter_mut() {
        // JUSTIFICATION: Volatile byte clear prevents dead-store elimination
        // when scrubbing cryptographic key material and plaintext buffers.
        // SAFETY: `byte` is a valid, aligned, uniquely borrowed `&mut u8`.
        unsafe {
            core::ptr::write_volatile(byte, 0);
        }
    }
    compiler_fence(Ordering::SeqCst);
}

/// IEEE 802.11 frame processing and parsing errors.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameError {
    /// Provided buffer is too small to construct or parse the frame.
    BufferTooSmall,
    /// 802.11 MAC header is malformed, truncated, or contains invalid fields.
    MalformedHeader,
    /// Frame checksum, MIC, or cryptographic authentication tag failed.
    InvalidCrcOrMic,
    /// 802.11 frame type or subtype is not supported by this handler.
    UnsupportedSubtype,
    /// CCMP encryption or decryption operation failed.
    CryptoError,
    /// Frame payload is invalid, truncated, or lacks LLC/SNAP encapsulation.
    InvalidPayload,
    /// Required cryptographic key (PTK or GTK) is missing.
    MissingKey,
    /// Unencrypted data frame was received after encryption was activated.
    UnencryptedPayloadRejected,
    /// CCMP packet number is zero or exceeds the 48-bit ceiling.
    InvalidPacketNumber,
    /// CCMP decrypted key ID does not match the expected PTK/GTK key slot.
    KeyIdMismatch,
}

impl fmt::Display for FrameError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::BufferTooSmall => write!(f, "buffer too small"),
            Self::MalformedHeader => write!(f, "malformed 802.11 header"),
            Self::InvalidCrcOrMic => write!(f, "invalid CRC or CCMP MIC"),
            Self::UnsupportedSubtype => write!(f, "unsupported frame subtype"),
            Self::CryptoError => write!(f, "cryptographic operation error"),
            Self::InvalidPayload => write!(f, "invalid payload or LLC/SNAP encapsulation"),
            Self::MissingKey => write!(f, "missing PTK or GTK cryptographic key"),
            Self::UnencryptedPayloadRejected => {
                write!(f, "unencrypted payload rejected while keys active")
            }
            Self::InvalidPacketNumber => write!(f, "invalid or exhausted CCMP packet number"),
            Self::KeyIdMismatch => write!(f, "CCMP key ID mismatch"),
        }
    }
}

/// Primary IEEE 802.11 frame types.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameType {
    /// Management frame (Type 0).
    Mgmt,
    /// Control frame (Type 1).
    Control,
    /// Data frame (Type 2).
    Data,
    /// Extension frame (Type 3).
    Extension,
}

/// Management frame subtypes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MgmtSubtype {
    /// Association Request (0x0).
    AssocReq,
    /// Association Response (0x1).
    AssocResp,
    /// Probe Request (0x4).
    ProbeReq,
    /// Probe Response (0x5).
    ProbeResp,
    /// Beacon (0x8).
    Beacon,
    /// Disassociation (0xA).
    Disassoc,
    /// Authentication (0xB).
    Auth,
    /// Deauthentication (0xC).
    Deauth,
    /// Action frame (0xD).
    Action,
    /// Other management subtype.
    Other(u8),
}

/// Data frame subtypes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DataSubtype {
    /// Normal Data (0x0).
    Data,
    /// Null Function (no data) (0x4).
    NullData,
    /// QoS Data (0x8).
    QosData,
    /// QoS Null Function (0xC).
    QosNull,
    /// Other data subtype.
    Other(u8),
}

/// High-level parsed representation of an 802.11 MAC frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ParsedFrame<'a> {
    /// Primary frame type.
    pub frame_type: FrameType,
    /// Raw subtype nibble.
    pub subtype: u8,
    /// Frame is destined to the distribution system.
    pub to_ds: bool,
    /// Frame originated from the distribution system.
    pub from_ds: bool,
    /// Frame has the Protected Frame bit set (CCMP/WEP/TKIP).
    pub is_protected: bool,
    /// Receiver Address (Addr1).
    pub addr1: [u8; 6],
    /// Transmitter Address (Addr2).
    pub addr2: [u8; 6],
    /// Third Address (Addr3: BSSID, SA, or DA depending on DS bits).
    pub addr3: [u8; 6],
    /// Sequence Control field value.
    pub seq_ctrl: u16,
    /// Frame payload slice following the MAC header.
    pub payload: &'a [u8],
}

/// Information extracted from a Beacon or Probe Response frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BeaconInfo<'a> {
    /// BSSID (MAC address) of the Access Point.
    pub bssid: [u8; 6],
    /// Service Set Identifier (SSID) bytes.
    pub ssid: &'a [u8],
    /// Primary radio channel frequency index (`1..=14`), if advertised.
    pub channel: Option<u8>,
    /// True if AP announces WEP/WPA/RSN privacy capability.
    pub privacy: bool,
    /// Received signal strength indication (dBm).
    pub rssi: i8,
    /// Raw RSN Information Element slice (Tag 48), if present and valid.
    pub rsn_ie: Option<&'a [u8]>,
    /// Raw WPA Information Element slice (Tag 221 with Microsoft OUI), if valid.
    pub wpa_ie: Option<&'a [u8]>,
    /// True if CCMP (AES-128) pairwise cipher is advertised.
    pub has_ccmp: bool,
    /// True if WPA2-PSK (Pre-Shared Key) authentication is advertised.
    pub has_psk: bool,
}

/// Action frame category and kind.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ActionKind {
    /// Block ACK - ADDBA Request.
    BaAddbaReq,
    /// Block ACK - ADDBA Response.
    BaAddbaResp,
    /// Block ACK - DELBA.
    BaDelba,
    /// Other action frame.
    Other(u8, u8),
}

/// Cryptographic temporal key reference for 802.11 transmission or reception.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DataKey<'a> {
    /// 16-byte CCMP temporal key slice.
    pub key: &'a [u8; 16],
    /// Key index (0 for pairwise PTK, 1..=3 for group GTK).
    pub key_id: u8,
}

/// Authenticated CCMP decryption metadata returned with a received Data MPDU.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CcmpRxMeta {
    /// 48-bit CCMP Packet Number extracted from the authenticated MPDU.
    pub pn: u64,
    /// 2-bit CCMP Key ID (`0` for PTK, `0..=3` for GTK).
    pub key_id: u8,
    /// QoS Traffic Identifier (`Some(0..=15)` for QoS Data, `None` for Data).
    pub qos_tid: Option<u8>,
}

/// Outcome of processing a received 802.11 Data MPDU.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RxDataOutcome {
    /// Valid Ethernet II frame written to destination slice.
    Ethernet {
        /// Number of Ethernet-II bytes written to `out_eth`.
        len: usize,
        /// CCMP metadata if the frame was CCMP-protected.
        ccmp: Option<CcmpRxMeta>,
    },
    /// EAPOL 802.1X handshake payload extracted.
    Eapol {
        /// Number of EAPOL bytes written to `out_eth`.
        len: usize,
        /// CCMP metadata (`Some` if encrypted, `None` if pre-key plaintext).
        ccmp: Option<CcmpRxMeta>,
    },
}

/// Validates common plaintext 802.11 Management frame header invariants and
/// returns `(subtype, body_offset)`.
fn validate_mgmt_header(mpdu: &[u8], body_min_len: usize) -> Result<(u8, usize), FrameError> {
    if mpdu.len() < 24 {
        return Err(FrameError::MalformedHeader);
    }
    let fc = u16::from_le_bytes([mpdu[0], mpdu[1]]);
    // Protocol Version must be 0.
    if (fc & 0x0003) != 0 {
        return Err(FrameError::MalformedHeader);
    }
    let type_val = (fc >> 2) & 0x03;
    if type_val != 0 {
        return Err(FrameError::UnsupportedSubtype);
    }
    let subtype = ((fc >> 4) & 0x0F) as u8;
    // Reject DS bits, More Fragments, Fragment Number != 0, and Protected bit.
    if (fc & 0x0300) != 0 || (fc & 0x0400) != 0 || (mpdu[22] & 0x0F) != 0 || (fc & 0x4000) != 0 {
        return Err(FrameError::MalformedHeader);
    }
    let body_off: usize = if (fc & 0x8000) != 0 { 28 } else { 24 };
    let required = body_off
        .checked_add(body_min_len)
        .ok_or(FrameError::MalformedHeader)?;
    if mpdu.len() < required {
        return Err(FrameError::MalformedHeader);
    }
    Ok((subtype, body_off))
}

/// Parses an 802.11 MAC frame header and payload.
pub fn parse_80211_header(raw_mpdu: &[u8]) -> Result<ParsedFrame<'_>, FrameError> {
    if raw_mpdu.len() < 2 {
        return Err(FrameError::MalformedHeader);
    }
    let fc = u16::from_le_bytes([raw_mpdu[0], raw_mpdu[1]]);
    // Protocol Version must be 0 per IEEE 802.11.
    if (fc & 0x0003) != 0 {
        return Err(FrameError::MalformedHeader);
    }
    let type_val = ((fc >> 2) & 0x03) as u8;
    let subtype = ((fc >> 4) & 0x0F) as u8;
    let to_ds = (fc & 0x0100) != 0;
    let from_ds = (fc & 0x0200) != 0;
    let is_protected = (fc & 0x4000) != 0;

    let frame_type = match type_val {
        0 => FrameType::Mgmt,
        1 => FrameType::Control,
        2 => FrameType::Data,
        _ => return Err(FrameError::UnsupportedSubtype),
    };

    if frame_type == FrameType::Control {
        // Control frames have shorter headers (10 B for ACK/CTS, 16 B for
        // RTS/PS-Poll/BA/BAR/CF-End) and no Addr3 or SeqCtrl.
        let hdr_len = match subtype {
            0x0C | 0x0D => 10,
            0x08..=0x0B | 0x0E | 0x0F => 16,
            _ => return Err(FrameError::UnsupportedSubtype),
        };
        if raw_mpdu.len() < hdr_len {
            return Err(FrameError::MalformedHeader);
        }
        let mut addr1 = [0u8; 6];
        let mut addr2 = [0u8; 6];
        addr1.copy_from_slice(&raw_mpdu[4..10]);
        if hdr_len >= 16 {
            addr2.copy_from_slice(&raw_mpdu[10..16]);
        }
        return Ok(ParsedFrame {
            frame_type,
            subtype,
            to_ds,
            from_ds,
            is_protected,
            addr1,
            addr2,
            addr3: [0u8; 6],
            seq_ctrl: 0,
            payload: &raw_mpdu[hdr_len..],
        });
    }

    if raw_mpdu.len() < 24 {
        return Err(FrameError::MalformedHeader);
    }

    let mut addr1 = [0u8; 6];
    let mut addr2 = [0u8; 6];
    let mut addr3 = [0u8; 6];
    addr1.copy_from_slice(&raw_mpdu[4..10]);
    addr2.copy_from_slice(&raw_mpdu[10..16]);
    addr3.copy_from_slice(&raw_mpdu[16..22]);
    let seq_ctrl = u16::from_le_bytes([raw_mpdu[22], raw_mpdu[23]]);

    let mut hdr_len = 24usize;
    if frame_type == FrameType::Data && to_ds && from_ds {
        // 4-address WDS framing applies only to Data frames.
        hdr_len = 30;
    }

    if frame_type == FrameType::Data && (subtype & 0x08) != 0 {
        // QoS Data frame has 2-byte QoS Control.
        hdr_len += 2;
        if (fc & 0x8000) != 0 {
            // HT Control present.
            hdr_len += 4;
        }
    } else if frame_type == FrameType::Mgmt && (fc & 0x8000) != 0 {
        // HT Control present on Management frame.
        hdr_len += 4;
    }

    if raw_mpdu.len() < hdr_len {
        return Err(FrameError::MalformedHeader);
    }

    Ok(ParsedFrame {
        frame_type,
        subtype,
        to_ds,
        from_ds,
        is_protected,
        addr1,
        addr2,
        addr3,
        seq_ctrl,
        payload: &raw_mpdu[hdr_len..],
    })
}

/// Parses a Beacon or Probe Response MPDU into structured [`BeaconInfo`].
pub fn parse_beacon_or_probe_resp(mpdu: &[u8], rssi: i8) -> Result<BeaconInfo<'_>, FrameError> {
    let (subtype, body_off) = validate_mgmt_header(mpdu, 12)?;
    if subtype != 8 && subtype != 5 {
        return Err(FrameError::UnsupportedSubtype);
    }

    let mut bssid = [0u8; 6];
    bssid.copy_from_slice(&mpdu[16..22]);

    let capabilities = u16::from_le_bytes([mpdu[body_off + 10], mpdu[body_off + 11]]);
    let privacy = (capabilities & 0x0010) != 0;

    let mut ssid: Option<&[u8]> = None;
    let mut ds_channel: Option<u8> = None;
    let mut ht_channel: Option<u8> = None;
    let mut rsn_ie: Option<&[u8]> = None;
    let mut wpa_ie: Option<&[u8]> = None;

    let mut pos = body_off + 12;
    while pos < mpdu.len() {
        if pos + 2 > mpdu.len() {
            return Err(FrameError::MalformedHeader);
        }
        let tag = mpdu[pos];
        let len = usize::from(mpdu[pos + 1]);
        let total = 2 + len;
        if pos + total > mpdu.len() {
            return Err(FrameError::MalformedHeader);
        }
        let elem = &mpdu[pos..pos + total];
        match tag {
            0 => {
                // First-wins SSID IE; reject overlong SSID (> 32 B).
                if ssid.is_none() {
                    let s = &elem[2..];
                    if s.len() > 32 {
                        return Err(FrameError::InvalidPayload);
                    }
                    ssid = Some(s);
                }
            }
            3 => {
                // First-wins DS Parameter Set (1-byte channel 1..=14).
                if ds_channel.is_none() && len == 1 && (1..=14).contains(&elem[2]) {
                    ds_channel = Some(elem[2]);
                }
            }
            48 => {
                // First-wins RSN IE.
                if rsn_ie.is_none() {
                    rsn_ie = Some(elem);
                }
            }
            61 => {
                // First-wins HT Operation primary channel (1..=14).
                if ht_channel.is_none() && len >= 1 && (1..=14).contains(&elem[2]) {
                    ht_channel = Some(elem[2]);
                }
            }
            // First-wins Vendor Specific WPA IE (00:50:F2:01).
            221 if len >= 4 && elem[2..6] == [0x00, 0x50, 0xF2, 0x01] && wpa_ie.is_none() => {
                wpa_ie = Some(elem);
            }
            _ => {}
        }
        pos += total;
    }

    let mut has_ccmp = false;
    let mut has_psk = false;

    if let Some(rie) = rsn_ie {
        if let Ok(data) = parse_wpa_or_rsn_ie(rie) {
            has_ccmp = (data.pairwise_cipher & 0x08) != 0;
            has_psk = (data.key_mgmt & 0x02) != 0;
        } else {
            // Do not expose malformed RSN IE; allow clean WPA fallback check.
            rsn_ie = None;
        }
    }
    if rsn_ie.is_none() {
        if let Some(wie) = wpa_ie {
            if let Ok(data) = parse_wpa_or_rsn_ie(wie) {
                has_ccmp = (data.pairwise_cipher & 0x08) != 0;
                has_psk = (data.key_mgmt & 0x02) != 0;
            } else {
                wpa_ie = None;
            }
        }
    }

    Ok(BeaconInfo {
        bssid,
        ssid: ssid.unwrap_or(&[]),
        channel: ds_channel.or(ht_channel),
        privacy,
        rssi,
        rsn_ie,
        wpa_ie,
        has_ccmp,
        has_psk,
    })
}

/// Parses an Open System Auth Response body returning
/// `(sequence_number, status_code)`.
pub fn parse_auth_resp(mpdu: &[u8]) -> Result<(u16, u16), FrameError> {
    let (subtype, body_off) = validate_mgmt_header(mpdu, 6)?;
    if subtype != 0x0B {
        return Err(FrameError::UnsupportedSubtype);
    }
    let auth_alg = u16::from_le_bytes([mpdu[body_off], mpdu[body_off + 1]]);
    if auth_alg != 0 {
        return Err(FrameError::UnsupportedSubtype);
    }
    let auth_seq = u16::from_le_bytes([mpdu[body_off + 2], mpdu[body_off + 3]]);
    let status_code = u16::from_le_bytes([mpdu[body_off + 4], mpdu[body_off + 5]]);
    Ok((auth_seq, status_code))
}

/// Parses an Association or Reassociation Response body returning
/// `(status_code, association_id)`.
fn parse_assoc_or_reassoc_resp(
    mpdu: &[u8],
    expected_subtype: u8,
) -> Result<(u16, u16), FrameError> {
    let (subtype, body_off) = validate_mgmt_header(mpdu, 6)?;
    if subtype != expected_subtype {
        return Err(FrameError::UnsupportedSubtype);
    }
    let status_code = u16::from_le_bytes([mpdu[body_off + 2], mpdu[body_off + 3]]);
    let raw_aid = u16::from_le_bytes([mpdu[body_off + 4], mpdu[body_off + 5]]);
    let aid = raw_aid & 0x3FFF;
    // IEEE 802.11 §9.4.1.8: top two bits must be 11 and AID in 1..=2007 on success.
    if status_code == 0 && ((raw_aid & 0xC000) != 0xC000 || !(1..=2007).contains(&aid)) {
        return Err(FrameError::MalformedHeader);
    }
    Ok((status_code, aid))
}

/// Parses an Assoc Response body returning `(status_code, association_id)`.
pub fn parse_assoc_resp(mpdu: &[u8]) -> Result<(u16, u16), FrameError> {
    parse_assoc_or_reassoc_resp(mpdu, 0x01)
}

/// Parses a Reassoc Response body returning `(status_code, association_id)`.
pub fn parse_reassoc_resp(mpdu: &[u8]) -> Result<(u16, u16), FrameError> {
    parse_assoc_or_reassoc_resp(mpdu, 0x03)
}

/// Parses a Deauth or Disassoc frame returning `(subtype, reason_code)`.
pub fn parse_deauth_or_disassoc(mpdu: &[u8]) -> Result<(u8, u16), FrameError> {
    let (subtype, body_off) = validate_mgmt_header(mpdu, 2)?;
    if subtype != 0x0A && subtype != 0x0C {
        return Err(FrameError::UnsupportedSubtype);
    }
    let reason_code = u16::from_le_bytes([mpdu[body_off], mpdu[body_off + 1]]);
    Ok((subtype, reason_code))
}

/// Parses an ADDBA Request action frame.
///
/// Returns `(dialog_token, ba_param_set, ba_timeout, start_seq)`.
pub fn parse_addba_req(mpdu: &[u8]) -> Result<(u8, u16, u16, u16), FrameError> {
    let (subtype, body_off) = validate_mgmt_header(mpdu, 9)?;
    if subtype != 0x0D {
        return Err(FrameError::UnsupportedSubtype);
    }
    // Block ACK Category = 3, Action = 0 (ADDBA Request).
    if mpdu[body_off] != 3 || mpdu[body_off + 1] != 0 {
        return Err(FrameError::UnsupportedSubtype);
    }
    let dialog_token = mpdu[body_off + 2];
    let ba_param_set = u16::from_le_bytes([mpdu[body_off + 3], mpdu[body_off + 4]]);
    let ba_timeout = u16::from_le_bytes([mpdu[body_off + 5], mpdu[body_off + 6]]);
    let start_seq = u16::from_le_bytes([mpdu[body_off + 7], mpdu[body_off + 8]]) >> 4;
    Ok((dialog_token, ba_param_set, ba_timeout, start_seq))
}

/// Extended Supported Rates IE (Tag 50): 24, 36, 48, 54 Mbps.
pub const EXT_SUPPORTED_RATES_IE: [u8; 6] = [0x32, 0x04, 0x30, 0x48, 0x60, 0x6C];

/// Builds an 802.11 Probe Request management frame.
pub fn build_probe_req(
    buf: &mut [u8],
    sta_mac: [u8; 6],
    ssid: Option<&[u8]>,
    seq: u16,
) -> Result<usize, FrameError> {
    let ssid_bytes = ssid.unwrap_or(&[]);
    if ssid_bytes.len() > 32 {
        return Err(FrameError::InvalidPayload);
    }
    let total_len = 24usize
        .checked_add(2)
        .and_then(|v| v.checked_add(ssid_bytes.len()))
        .and_then(|v| v.checked_add(10))
        .and_then(|v| v.checked_add(EXT_SUPPORTED_RATES_IE.len()))
        .ok_or(FrameError::BufferTooSmall)?;
    if buf.len() < total_len {
        return Err(FrameError::BufferTooSmall);
    }

    // Frame Control: Type=0 (Mgmt), Subtype=4 (ProbeReq) -> 0x0040
    buf[0] = 0x40;
    buf[1] = 0x00;
    buf[2] = 0x00;
    buf[3] = 0x00;
    buf[4..10].fill(0xFF); // Broadcast DA
    buf[10..16].copy_from_slice(&sta_mac); // SA
    buf[16..22].fill(0xFF); // Broadcast BSSID
    buf[22..24].copy_from_slice(&((seq & 0x0FFF) << 4).to_le_bytes());

    let mut pos = 24;
    // SSID IE (Tag 0)
    buf[pos] = 0;
    buf[pos + 1] = ssid_bytes.len() as u8;
    buf[pos + 2..pos + 2 + ssid_bytes.len()].copy_from_slice(ssid_bytes);
    pos += 2 + ssid_bytes.len();

    // Supported Rates IE (Tag 1): 1, 2, 5.5, 11, 6, 9, 12, 18 Mbps
    buf[pos..pos + 10]
        .copy_from_slice(&[0x01, 0x08, 0x82, 0x84, 0x8B, 0x96, 0x0C, 0x12, 0x18, 0x24]);
    pos += 10;

    // Extended Supported Rates IE (Tag 50): 24, 36, 48, 54 Mbps
    buf[pos..pos + EXT_SUPPORTED_RATES_IE.len()].copy_from_slice(&EXT_SUPPORTED_RATES_IE);
    pos += EXT_SUPPORTED_RATES_IE.len();

    Ok(pos)
}

/// Builds an 802.11 Open System Authentication Request frame.
pub fn build_open_auth_req(
    buf: &mut [u8],
    bssid: [u8; 6],
    sta_mac: [u8; 6],
    seq: u16,
) -> Result<usize, FrameError> {
    const TOTAL_LEN: usize = 24 + 6;
    if buf.len() < TOTAL_LEN {
        return Err(FrameError::BufferTooSmall);
    }

    // Frame Control: Type=0 (Mgmt), Subtype=11 (Auth) -> 0x00B0
    buf[0] = 0xB0;
    buf[1] = 0x00;
    buf[2] = 0x00;
    buf[3] = 0x00;
    buf[4..10].copy_from_slice(&bssid); // DA
    buf[10..16].copy_from_slice(&sta_mac); // SA
    buf[16..22].copy_from_slice(&bssid); // BSSID
    buf[22..24].copy_from_slice(&((seq & 0x0FFF) << 4).to_le_bytes());

    // Fixed params: Auth Algorithm = 0 (Open System), Seq = 1, Status = 0
    buf[24..30].copy_from_slice(&[0x00, 0x00, 0x01, 0x00, 0x00, 0x00]);
    Ok(TOTAL_LEN)
}

/// Builds an 802.11 Association Request frame.
pub fn build_assoc_req(
    buf: &mut [u8],
    bssid: [u8; 6],
    sta_mac: [u8; 6],
    ssid: &[u8],
    rsn_ie: &[u8],
    seq: u16,
    listen_interval: u16,
) -> Result<usize, FrameError> {
    if ssid.len() > 32 {
        return Err(FrameError::InvalidPayload);
    }
    if !rsn_ie.is_empty()
        && (rsn_ie.len() < 2 || rsn_ie.len() != 2usize.saturating_add(usize::from(rsn_ie[1])))
    {
        return Err(FrameError::InvalidPayload);
    }
    let rates_ie_len = 10usize + EXT_SUPPORTED_RATES_IE.len();
    let total_len = 28usize
        .checked_add(2)
        .and_then(|v| v.checked_add(ssid.len()))
        .and_then(|v| v.checked_add(rates_ie_len))
        .and_then(|v| v.checked_add(rsn_ie.len()))
        .ok_or(FrameError::BufferTooSmall)?;
    if buf.len() < total_len {
        return Err(FrameError::BufferTooSmall);
    }

    // Frame Control: Type=0 (Mgmt), Subtype=0 (AssocReq) -> 0x0000
    buf[0] = 0x00;
    buf[1] = 0x00;
    buf[2] = 0x00;
    buf[3] = 0x00;
    buf[4..10].copy_from_slice(&bssid); // DA
    buf[10..16].copy_from_slice(&sta_mac); // SA
    buf[16..22].copy_from_slice(&bssid); // BSSID
    buf[22..24].copy_from_slice(&((seq & 0x0FFF) << 4).to_le_bytes());

    // Capabilities: ESS + Short Slot Time -> 0x0431
    buf[24..26].copy_from_slice(&0x0431u16.to_le_bytes());
    buf[26..28].copy_from_slice(&listen_interval.to_le_bytes());

    let mut pos = 28;
    // SSID IE (Tag 0)
    buf[pos] = 0;
    buf[pos + 1] = ssid.len() as u8;
    buf[pos + 2..pos + 2 + ssid.len()].copy_from_slice(ssid);
    pos += 2 + ssid.len();

    // Supported Rates IE
    buf[pos..pos + 10]
        .copy_from_slice(&[0x01, 0x08, 0x82, 0x84, 0x8B, 0x96, 0x0C, 0x12, 0x18, 0x24]);
    pos += 10;

    // Extended Supported Rates IE (Tag 50), ordered before RSN (Tag 48) as
    // in the 802.11 Association Request element order.
    buf[pos..pos + EXT_SUPPORTED_RATES_IE.len()].copy_from_slice(&EXT_SUPPORTED_RATES_IE);
    pos += EXT_SUPPORTED_RATES_IE.len();

    // RSN IE if present
    if !rsn_ie.is_empty() {
        buf[pos..pos + rsn_ie.len()].copy_from_slice(rsn_ie);
        pos += rsn_ie.len();
    }

    Ok(pos)
}

/// Builds an 802.11 Deauthentication frame.
pub fn build_deauth(
    buf: &mut [u8],
    bssid: [u8; 6],
    sta_mac: [u8; 6],
    reason: u16,
    seq: u16,
) -> Result<usize, FrameError> {
    const TOTAL_LEN: usize = 24 + 2;
    if buf.len() < TOTAL_LEN {
        return Err(FrameError::BufferTooSmall);
    }

    // Frame Control: Type=0 (Mgmt), Subtype=12 (Deauth) -> 0x00C0
    buf[0] = 0xC0;
    buf[1] = 0x00;
    buf[2] = 0x00;
    buf[3] = 0x00;
    buf[4..10].copy_from_slice(&bssid); // DA
    buf[10..16].copy_from_slice(&sta_mac); // SA
    buf[16..22].copy_from_slice(&bssid); // BSSID
    buf[22..24].copy_from_slice(&((seq & 0x0FFF) << 4).to_le_bytes());
    buf[24..26].copy_from_slice(&reason.to_le_bytes());
    Ok(TOTAL_LEN)
}

/// Builds an ADDBA Decline (Status 37) Action frame.
pub fn build_addba_decline(
    buf: &mut [u8],
    bssid: [u8; 6],
    sta_mac: [u8; 6],
    dialog_token: u8,
    tid_param_set: u16,
    seq: u16,
) -> Result<usize, FrameError> {
    const TOTAL_LEN: usize = 24 + 9;
    if buf.len() < TOTAL_LEN {
        return Err(FrameError::BufferTooSmall);
    }

    // Frame Control: Type=0 (Mgmt), Subtype=13 (Action) -> 0x00D0
    buf[0] = 0xD0;
    buf[1] = 0x00;
    buf[2] = 0x00;
    buf[3] = 0x00;
    buf[4..10].copy_from_slice(&bssid); // DA
    buf[10..16].copy_from_slice(&sta_mac); // SA
    buf[16..22].copy_from_slice(&bssid); // BSSID
    buf[22..24].copy_from_slice(&((seq & 0x0FFF) << 4).to_le_bytes());

    buf[24] = 3; // Block ACK Category
    buf[25] = 1; // ADDBA Response Action
    buf[26] = dialog_token;
    buf[27..29].copy_from_slice(&37u16.to_le_bytes()); // Status Code 37 = Decline
    buf[29..31].copy_from_slice(&tid_param_set.to_le_bytes());
    buf[31..33].copy_from_slice(&[0x00, 0x00]); // BA Timeout = 0
    Ok(TOTAL_LEN)
}

/// Builds an 802.11 Null Data keepalive frame.
pub fn build_null_data_keepalive(
    buf: &mut [u8],
    bssid: [u8; 6],
    sta_mac: [u8; 6],
    seq: u16,
    pwr_mgmt: bool,
) -> Result<usize, FrameError> {
    const TOTAL_LEN: usize = 24;
    if buf.len() < TOTAL_LEN {
        return Err(FrameError::BufferTooSmall);
    }

    // Frame Control: Data (Type=2), Null Data (Subtype=4) -> 0x0048,
    // with ToDS=1 -> 0x0148.
    buf[0] = 0x48;
    buf[1] = 0x01 | if pwr_mgmt { 0x10 } else { 0x00 };
    buf[2] = 0x00;
    buf[3] = 0x00;
    buf[4..10].copy_from_slice(&bssid); // RA / BSSID
    buf[10..16].copy_from_slice(&sta_mac); // TA / SA
    buf[16..22].copy_from_slice(&bssid); // DA
    buf[22..24].copy_from_slice(&((seq & 0x0FFF) << 4).to_le_bytes());
    Ok(TOTAL_LEN)
}

/// Encapsulates Ethernet-II into 802.11 Data MPDU, optionally with CCMP.
pub fn eth_to_80211_data(
    out_mpdu: &mut [u8],
    eth_frame: &[u8],
    bssid: [u8; 6],
    seq: u16,
    tx_key: Option<(&[u8; 16], u8, u64)>,
) -> Result<usize, FrameError> {
    if eth_frame.len() < 14 {
        return Err(FrameError::InvalidPayload);
    }
    let da = &eth_frame[0..6];
    let sa = &eth_frame[6..12];
    let ethertype = &eth_frame[12..14];
    let eth_payload = &eth_frame[14..];

    let mut mac_header = [0u8; 24];
    // Frame Control: Type=2 (Data), Subtype=0, ToDS=1 -> 0x0108
    mac_header[0] = 0x08;
    mac_header[1] = if tx_key.is_some() { 0x41 } else { 0x01 };
    mac_header[2] = 0x00;
    mac_header[3] = 0x00;
    mac_header[4..10].copy_from_slice(&bssid); // Addr1: RA (BSSID)
    mac_header[10..16].copy_from_slice(sa); // Addr2: TA (STA MAC)
    mac_header[16..22].copy_from_slice(da); // Addr3: DA
    mac_header[22..24].copy_from_slice(&((seq & 0x0FFF) << 4).to_le_bytes());

    if let Some((tk, key_id, pn)) = tx_key {
        if pn == 0 || pn > MAX_CCMP_PN {
            return Err(FrameError::InvalidPacketNumber);
        }
        if key_id > 3 {
            return Err(FrameError::CryptoError);
        }
        // Plaintext payload contains LLC/SNAP header (8 bytes) + eth_payload.
        let mut plain = [0u8; 1536];
        let plain_len = 8usize
            .checked_add(eth_payload.len())
            .ok_or(FrameError::BufferTooSmall)?;
        if plain_len > plain.len() {
            return Err(FrameError::BufferTooSmall);
        }
        // LLC/SNAP: AA AA 03 00 00 00 <ethertype>.
        plain[0..6].copy_from_slice(&[0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00]);
        plain[6..8].copy_from_slice(ethertype);
        plain[8..plain_len].copy_from_slice(eth_payload);

        let res = ccmp_encrypt_mpdu(tk, key_id, pn, &mac_header, &plain[..plain_len], out_mpdu)
            .map_err(|_| FrameError::CryptoError);
        volatile_zeroize(&mut plain[..plain_len]);
        res
    } else {
        let total_len = 32usize
            .checked_add(eth_payload.len())
            .ok_or(FrameError::BufferTooSmall)?;
        if out_mpdu.len() < total_len {
            return Err(FrameError::BufferTooSmall);
        }
        out_mpdu[0..24].copy_from_slice(&mac_header);
        out_mpdu[24..30].copy_from_slice(&[0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00]);
        out_mpdu[30..32].copy_from_slice(ethertype);
        out_mpdu[32..total_len].copy_from_slice(eth_payload);
        Ok(total_len)
    }
}

/// Decapsulates an 802.11 Data MPDU into an Ethernet-II frame or EAPOL packet.
#[allow(clippy::too_many_arguments)]
pub fn rx_80211_data_to_eth(
    raw_mpdu: &[u8],
    scratch_plain: &mut [u8],
    out_eth: &mut [u8],
    ptk: Option<&[u8; 16]>,
    gtk: Option<(u8, &[u8; 16])>,
    expected_bssid: &[u8; 6],
    sta_mac: &[u8; 6],
    bss_protected: bool,
) -> Result<RxDataOutcome, FrameError> {
    if raw_mpdu.len() < 24 {
        return Err(FrameError::MalformedHeader);
    }
    let fc = u16::from_le_bytes([raw_mpdu[0], raw_mpdu[1]]);
    // Enforce Protocol Version == 0.
    if (fc & 0x0003) != 0 {
        return Err(FrameError::MalformedHeader);
    }
    let type_val = (fc >> 2) & 0x03;
    let subtype = ((fc >> 4) & 0x0F) as u8;
    // Accept only Data (0x0) and QoS Data (0x8) subtypes (reject Null/CF-Ack).
    if type_val != 2 || (subtype != 0x00 && subtype != 0x08) {
        return Err(FrameError::UnsupportedSubtype);
    }
    // Enforce infrastructure AP-to-STA framing: FromDS == 1, ToDS == 0.
    if (fc & 0x0300) != 0x0200 {
        return Err(FrameError::MalformedHeader);
    }
    // FragAttacks (CVE-2020-26142/26146/26147): reject fragmented MPDUs.
    if (fc & 0x0400) != 0 || (raw_mpdu[22] & 0x0F) != 0 {
        return Err(FrameError::UnsupportedSubtype);
    }

    let is_protected = (fc & 0x4000) != 0;
    let is_qos = subtype == 0x08;
    let has_htc = is_qos && (fc & 0x8000) != 0;
    let hdr_len = 24 + if is_qos { 2 } else { 0 } + if has_htc { 4 } else { 0 };
    if raw_mpdu.len() < hdr_len {
        return Err(FrameError::MalformedHeader);
    }

    let qos_tid = if is_qos {
        // FragAttacks (CVE-2020-24588): reject QoS A-MSDU Present frames.
        if (raw_mpdu[24] & 0x80) != 0 {
            return Err(FrameError::UnsupportedSubtype);
        }
        Some(raw_mpdu[24] & 0x0F)
    } else {
        None
    };

    // Verify Addr2 matches expected BSSID for AP-to-STA (FromDS=1) frames.
    if raw_mpdu[10..16] != *expected_bssid {
        return Err(FrameError::MalformedHeader);
    }

    let da = &raw_mpdu[4..10]; // Addr1: DA
    let sa = &raw_mpdu[16..22]; // Addr3: SA
    let is_group = (da[0] & 0x01) != 0;
    // Enforce unicast DA == sta_mac.
    if !is_group && da != sta_mac {
        return Err(FrameError::MalformedHeader);
    }

    let (plain_payload, ccmp_meta): (&[u8], Option<CcmpRxMeta>) = if is_protected {
        let (key, expected_key_id) = if is_group {
            let (kid, k) = gtk.ok_or(FrameError::MissingKey)?;
            (k, kid)
        } else {
            (ptk.ok_or(FrameError::MissingKey)?, 0u8)
        };

        let (pt_len, key_id, pn) =
            ccmp_decrypt_mpdu(key, raw_mpdu, scratch_plain).map_err(|_| FrameError::CryptoError)?;
        if key_id != expected_key_id {
            return Err(FrameError::KeyIdMismatch);
        }
        if pn == 0 || pn > MAX_CCMP_PN {
            return Err(FrameError::InvalidPacketNumber);
        }
        let pt = scratch_plain
            .get(..pt_len)
            .ok_or(FrameError::BufferTooSmall)?;
        (
            pt,
            Some(CcmpRxMeta {
                pn,
                key_id,
                qos_tid,
            }),
        )
    } else {
        // FragAttacks (CVE-2020-26140): once PTK is installed, reject ALL
        // unencrypted frames, including EAPOL.
        if ptk.is_some() {
            return Err(FrameError::UnencryptedPayloadRejected);
        }
        let payload = &raw_mpdu[hdr_len..];
        let is_eapol = payload.len() >= 8
            && payload[0..6] == [0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00]
            && payload[6..8] == [0x88, 0x8E];
        // On a protected BSS (or when GTK is active), reject pre-key plaintext
        // non-EAPOL data frames.
        if (bss_protected || gtk.is_some()) && !is_eapol {
            return Err(FrameError::UnencryptedPayloadRejected);
        }
        (payload, None)
    };

    if plain_payload.len() < 8 {
        return Err(FrameError::InvalidPayload);
    }

    // Verify LLC/SNAP header: AA AA 03 00 00 00.
    if plain_payload[0..6] != [0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00] {
        return Err(FrameError::InvalidPayload);
    }

    let ethertype = [plain_payload[6], plain_payload[7]];
    let body = &plain_payload[8..];

    // EtherType 0x888E demux: EAPOL must be unicast to sta_mac.
    if ethertype == [0x88, 0x8E] {
        if is_group || da != sta_mac {
            return Err(FrameError::MalformedHeader);
        }
        if out_eth.len() < body.len() {
            return Err(FrameError::BufferTooSmall);
        }
        out_eth[..body.len()].copy_from_slice(body);
        Ok(RxDataOutcome::Eapol {
            len: body.len(),
            ccmp: ccmp_meta,
        })
    } else {
        // Standard Ethernet-II frame: DA (6) + SA (6) + EtherType (2) + Body.
        let total_eth_len = 14usize
            .checked_add(body.len())
            .ok_or(FrameError::BufferTooSmall)?;
        if out_eth.len() < total_eth_len {
            return Err(FrameError::BufferTooSmall);
        }
        out_eth[0..6].copy_from_slice(da);
        out_eth[6..12].copy_from_slice(sa);
        out_eth[12..14].copy_from_slice(&ethertype);
        out_eth[14..total_eth_len].copy_from_slice(body);
        Ok(RxDataOutcome::Ethernet {
            len: total_eth_len,
            ccmp: ccmp_meta,
        })
    }
}

/// Open SoftAP capability flags: ESS (`0x0001`) + Short Slot (`0x0400`),
/// with Privacy disabled.
const AP_OPEN_CAPABILITY: u16 = 0x0401;

/// Supported Rates IE (Tag 1): 1B, 2B, 5.5B, 11B, 6, 9, 12, 18 Mbps.
const AP_SUPPORTED_RATES_IE: [u8; 10] =
    [0x01, 0x08, 0x82, 0x84, 0x8B, 0x96, 0x0C, 0x12, 0x18, 0x24];

/// Traffic Indication Map IE (Tag 5) with DTIM count 0, period 1, empty bitmap.
const AP_TIM_IE: [u8; 6] = [0x05, 0x04, 0x00, 0x01, 0x00, 0x00];

/// Standard IEEE 802.2 LLC/SNAP header prefix for Ethernet-II encapsulation.
const AP_LLC_SNAP_HDR: [u8; 6] = [0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00];

/// Information extracted from a received 802.11 Probe Request body.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ProbeReqInfo<'a> {
    /// Requested SSID slice, or `None` for a wildcard (empty or absent) SSID.
    pub ssid: Option<&'a [u8]>,
}

/// Information extracted from a received 802.11 Association Request body.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AssocReqInfo<'a> {
    /// Capability Information field from the Association Request fixed body.
    pub capability: u16,
    /// Listen Interval field in Beacon intervals.
    pub listen_interval: u16,
    /// Requested SSID bytes from the SSID Information Element.
    pub ssid: &'a [u8],
}

/// Writes a 24-byte 802.11 AP Management header with `Addr1 = da`,
/// `Addr2 = bssid`, and `Addr3 = bssid`.
fn write_ap_mgmt_header(out: &mut [u8], fc_lo: u8, da: &[u8; 6], bssid: &[u8; 6], seq: u16) {
    out[0] = fc_lo;
    out[1] = 0x00;
    out[2] = 0x00;
    out[3] = 0x00;
    out[4..10].copy_from_slice(da);
    out[10..16].copy_from_slice(bssid);
    out[16..22].copy_from_slice(bssid);
    out[22..24].copy_from_slice(&((seq & 0x0FFF) << 4).to_le_bytes());
}

/// Builds an Open SoftAP 802.11 Beacon management frame (Subtype 8).
pub fn build_beacon(
    bssid: &[u8; 6],
    ssid: &[u8],
    channel: u8,
    interval_tu: u16,
    timestamp_us: u64,
    seq: u16,
    out: &mut [u8],
) -> Result<usize, FrameError> {
    if ssid.len() > 32 || !(1..=14).contains(&channel) {
        return Err(FrameError::InvalidPayload);
    }
    let total_len = 36usize
        .checked_add(2)
        .and_then(|v| v.checked_add(ssid.len()))
        .and_then(|v| v.checked_add(AP_SUPPORTED_RATES_IE.len()))
        .and_then(|v| v.checked_add(3))
        .and_then(|v| v.checked_add(AP_TIM_IE.len()))
        .and_then(|v| v.checked_add(EXT_SUPPORTED_RATES_IE.len()))
        .ok_or(FrameError::BufferTooSmall)?;
    if out.len() < total_len {
        return Err(FrameError::BufferTooSmall);
    }

    write_ap_mgmt_header(out, 0x80, &[0xFF; 6], bssid, seq);
    out[24..32].copy_from_slice(&timestamp_us.to_le_bytes());
    out[32..34].copy_from_slice(&interval_tu.to_le_bytes());
    out[34..36].copy_from_slice(&AP_OPEN_CAPABILITY.to_le_bytes());

    let mut pos = 36;
    out[pos] = 0x00;
    out[pos + 1] = ssid.len() as u8;
    out[pos + 2..pos + 2 + ssid.len()].copy_from_slice(ssid);
    pos += 2 + ssid.len();

    out[pos..pos + AP_SUPPORTED_RATES_IE.len()].copy_from_slice(&AP_SUPPORTED_RATES_IE);
    pos += AP_SUPPORTED_RATES_IE.len();

    out[pos..pos + 3].copy_from_slice(&[0x03, 0x01, channel]);
    pos += 3;

    out[pos..pos + AP_TIM_IE.len()].copy_from_slice(&AP_TIM_IE);
    pos += AP_TIM_IE.len();

    out[pos..pos + EXT_SUPPORTED_RATES_IE.len()].copy_from_slice(&EXT_SUPPORTED_RATES_IE);
    pos += EXT_SUPPORTED_RATES_IE.len();

    Ok(pos)
}

/// Builds an Open SoftAP 802.11 Probe Response management frame (Subtype 5).
#[allow(clippy::too_many_arguments)]
pub fn build_probe_resp(
    da: &[u8; 6],
    bssid: &[u8; 6],
    ssid: &[u8],
    channel: u8,
    interval_tu: u16,
    timestamp_us: u64,
    seq: u16,
    out: &mut [u8],
) -> Result<usize, FrameError> {
    if ssid.len() > 32 || !(1..=14).contains(&channel) {
        return Err(FrameError::InvalidPayload);
    }
    let total_len = 36usize
        .checked_add(2)
        .and_then(|v| v.checked_add(ssid.len()))
        .and_then(|v| v.checked_add(AP_SUPPORTED_RATES_IE.len()))
        .and_then(|v| v.checked_add(3))
        .and_then(|v| v.checked_add(EXT_SUPPORTED_RATES_IE.len()))
        .ok_or(FrameError::BufferTooSmall)?;
    if out.len() < total_len {
        return Err(FrameError::BufferTooSmall);
    }

    write_ap_mgmt_header(out, 0x50, da, bssid, seq);
    out[24..32].copy_from_slice(&timestamp_us.to_le_bytes());
    out[32..34].copy_from_slice(&interval_tu.to_le_bytes());
    out[34..36].copy_from_slice(&AP_OPEN_CAPABILITY.to_le_bytes());

    let mut pos = 36;
    out[pos] = 0x00;
    out[pos + 1] = ssid.len() as u8;
    out[pos + 2..pos + 2 + ssid.len()].copy_from_slice(ssid);
    pos += 2 + ssid.len();

    out[pos..pos + AP_SUPPORTED_RATES_IE.len()].copy_from_slice(&AP_SUPPORTED_RATES_IE);
    pos += AP_SUPPORTED_RATES_IE.len();

    out[pos..pos + 3].copy_from_slice(&[0x03, 0x01, channel]);
    pos += 3;

    out[pos..pos + EXT_SUPPORTED_RATES_IE.len()].copy_from_slice(&EXT_SUPPORTED_RATES_IE);
    pos += EXT_SUPPORTED_RATES_IE.len();

    Ok(pos)
}

/// Parses an 802.11 Probe Request payload with strict TLV validation.
pub fn parse_probe_req(payload: &[u8]) -> Result<ProbeReqInfo<'_>, FrameError> {
    let mut ssid: Option<&[u8]> = None;
    let mut saw_ssid = false;
    let mut pos = 0usize;
    while pos < payload.len() {
        if pos + 2 > payload.len() {
            return Err(FrameError::InvalidPayload);
        }
        let tag = payload[pos];
        let len = usize::from(payload[pos + 1]);
        let end = pos
            .checked_add(2)
            .and_then(|v| v.checked_add(len))
            .ok_or(FrameError::InvalidPayload)?;
        if end > payload.len() {
            return Err(FrameError::InvalidPayload);
        }
        if tag == 0 && !saw_ssid {
            saw_ssid = true;
            let elem = &payload[pos + 2..end];
            if elem.len() > 32 {
                return Err(FrameError::InvalidPayload);
            }
            if !elem.is_empty() {
                ssid = Some(elem);
            }
        }
        pos = end;
    }
    Ok(ProbeReqInfo { ssid })
}

/// Parses an 802.11 Authentication Request body (`(auth_alg, auth_seq)`).
pub fn parse_open_auth_req(payload: &[u8]) -> Result<(u16, u16), FrameError> {
    if payload.len() < 6 {
        return Err(FrameError::InvalidPayload);
    }
    let auth_alg = u16::from_le_bytes([payload[0], payload[1]]);
    let auth_seq = u16::from_le_bytes([payload[2], payload[3]]);
    Ok((auth_alg, auth_seq))
}

/// Builds an Open System 802.11 Authentication Response frame (Subtype 11).
pub fn build_open_auth_resp(
    da: &[u8; 6],
    bssid: &[u8; 6],
    auth_seq: u16,
    status_code: u16,
    seq: u16,
    out: &mut [u8],
) -> Result<usize, FrameError> {
    const TOTAL_LEN: usize = 24 + 6;
    if out.len() < TOTAL_LEN {
        return Err(FrameError::BufferTooSmall);
    }

    write_ap_mgmt_header(out, 0xB0, da, bssid, seq);
    out[24..26].copy_from_slice(&0u16.to_le_bytes());
    out[26..28].copy_from_slice(&auth_seq.to_le_bytes());
    out[28..30].copy_from_slice(&status_code.to_le_bytes());
    Ok(TOTAL_LEN)
}

/// Parses an 802.11 Association Request payload with strict fixed-body and
/// TLV validation.
pub fn parse_assoc_req(payload: &[u8]) -> Result<AssocReqInfo<'_>, FrameError> {
    if payload.len() < 4 {
        return Err(FrameError::InvalidPayload);
    }
    let capability = u16::from_le_bytes([payload[0], payload[1]]);
    let listen_interval = u16::from_le_bytes([payload[2], payload[3]]);

    let mut ssid: Option<&[u8]> = None;
    let mut pos = 4usize;
    while pos < payload.len() {
        if pos + 2 > payload.len() {
            return Err(FrameError::InvalidPayload);
        }
        let tag = payload[pos];
        let len = usize::from(payload[pos + 1]);
        let end = pos
            .checked_add(2)
            .and_then(|v| v.checked_add(len))
            .ok_or(FrameError::InvalidPayload)?;
        if end > payload.len() {
            return Err(FrameError::InvalidPayload);
        }
        if tag == 0 && ssid.is_none() {
            let elem = &payload[pos + 2..end];
            if elem.len() > 32 {
                return Err(FrameError::InvalidPayload);
            }
            ssid = Some(elem);
        }
        pos = end;
    }

    let ssid = ssid.ok_or(FrameError::InvalidPayload)?;
    Ok(AssocReqInfo {
        capability,
        listen_interval,
        ssid,
    })
}

/// Builds an Open SoftAP 802.11 Association (`fc_lo = 0x10`) or
/// Reassociation (`fc_lo = 0x30`) Response frame.
fn build_assoc_or_reassoc_resp(
    fc_lo: u8,
    da: &[u8; 6],
    bssid: &[u8; 6],
    aid: u16,
    status_code: u16,
    seq: u16,
    out: &mut [u8],
) -> Result<usize, FrameError> {
    if status_code == 0 && !(1..=2007).contains(&aid) {
        return Err(FrameError::InvalidPayload);
    }
    const TOTAL_LEN: usize = 24 + 6 + AP_SUPPORTED_RATES_IE.len() + EXT_SUPPORTED_RATES_IE.len();
    if out.len() < TOTAL_LEN {
        return Err(FrameError::BufferTooSmall);
    }

    write_ap_mgmt_header(out, fc_lo, da, bssid, seq);
    out[24..26].copy_from_slice(&AP_OPEN_CAPABILITY.to_le_bytes());
    out[26..28].copy_from_slice(&status_code.to_le_bytes());
    out[28..30].copy_from_slice(&((aid & 0x3FFF) | 0xC000).to_le_bytes());

    let mut pos = 30;
    out[pos..pos + AP_SUPPORTED_RATES_IE.len()].copy_from_slice(&AP_SUPPORTED_RATES_IE);
    pos += AP_SUPPORTED_RATES_IE.len();
    out[pos..pos + EXT_SUPPORTED_RATES_IE.len()].copy_from_slice(&EXT_SUPPORTED_RATES_IE);
    pos += EXT_SUPPORTED_RATES_IE.len();

    Ok(pos)
}

/// Builds an Open SoftAP 802.11 Association Response frame (Subtype 1).
///
/// # Errors
/// Returns [`FrameError::InvalidPayload`] if `status_code == 0` and `aid` is
/// outside `1..=2007`, or [`FrameError::BufferTooSmall`] if `out` is too short.
pub fn build_assoc_resp(
    da: &[u8; 6],
    bssid: &[u8; 6],
    aid: u16,
    status_code: u16,
    seq: u16,
    out: &mut [u8],
) -> Result<usize, FrameError> {
    build_assoc_or_reassoc_resp(0x10, da, bssid, aid, status_code, seq, out)
}

/// Builds an Open SoftAP 802.11 Reassociation Response frame (Subtype 3,
/// IEEE 802.11 §9.3.3.8).
///
/// # Errors
/// Returns [`FrameError::InvalidPayload`] if `status_code == 0` and `aid` is
/// outside `1..=2007`, or [`FrameError::BufferTooSmall`] if `out` is too short.
pub fn build_reassoc_resp(
    da: &[u8; 6],
    bssid: &[u8; 6],
    aid: u16,
    status_code: u16,
    seq: u16,
    out: &mut [u8],
) -> Result<usize, FrameError> {
    build_assoc_or_reassoc_resp(0x30, da, bssid, aid, status_code, seq, out)
}

/// Builds an Open SoftAP 802.11 Deauthentication frame (Subtype 12).
pub fn build_ap_deauth(
    da: &[u8; 6],
    bssid: &[u8; 6],
    reason_code: u16,
    seq: u16,
    out: &mut [u8],
) -> Result<usize, FrameError> {
    const TOTAL_LEN: usize = 24 + 2;
    if out.len() < TOTAL_LEN {
        return Err(FrameError::BufferTooSmall);
    }

    write_ap_mgmt_header(out, 0xC0, da, bssid, seq);
    out[24..26].copy_from_slice(&reason_code.to_le_bytes());
    Ok(TOTAL_LEN)
}

/// Encapsulates an Ethernet-II frame into an unencrypted AP 802.11 Data MPDU
/// (`ToDS = 0`, `FromDS = 1`).
pub fn ap_eth_to_80211_data(
    eth_frame: &[u8],
    bssid: &[u8; 6],
    seq: u16,
    out: &mut [u8],
) -> Result<usize, FrameError> {
    if eth_frame.len() < 14 {
        return Err(FrameError::InvalidPayload);
    }
    let da = &eth_frame[0..6];
    let sa = &eth_frame[6..12];
    let ethertype = &eth_frame[12..14];
    let eth_payload = &eth_frame[14..];

    let total_len = 32usize
        .checked_add(eth_payload.len())
        .ok_or(FrameError::BufferTooSmall)?;
    if out.len() < total_len {
        return Err(FrameError::BufferTooSmall);
    }

    // Frame Control: Type=Data(2), Subtype=0, ToDS=0, FromDS=1, Protected=0.
    out[0] = 0x08;
    out[1] = 0x02;
    out[2] = 0x00;
    out[3] = 0x00;
    out[4..10].copy_from_slice(da);
    out[10..16].copy_from_slice(bssid);
    out[16..22].copy_from_slice(sa);
    out[22..24].copy_from_slice(&((seq & 0x0FFF) << 4).to_le_bytes());

    out[24..30].copy_from_slice(&AP_LLC_SNAP_HDR);
    out[30..32].copy_from_slice(ethertype);
    out[32..total_len].copy_from_slice(eth_payload);
    Ok(total_len)
}

/// Decapsulates an unencrypted STA-to-AP 802.11 Data or QoS-Data MPDU
/// (`ToDS = 1`, `FromDS = 0`) into an Ethernet-II frame.
///
/// Enforces Protocol Version 0, `Addr1 == bssid`, `Addr2 == expected_sta_mac`,
/// and rejects Protected, Fragmented (`MF` / `FragNum != 0`), and QoS A-MSDU
/// frames.
pub fn ap_rx_80211_data_to_eth(
    raw_mpdu: &[u8],
    bssid: &[u8; 6],
    expected_sta_mac: &[u8; 6],
    eth_out: &mut [u8],
) -> Result<usize, FrameError> {
    if raw_mpdu.len() < 24 {
        return Err(FrameError::MalformedHeader);
    }
    let fc = u16::from_le_bytes([raw_mpdu[0], raw_mpdu[1]]);
    if (fc & 0x0003) != 0 {
        return Err(FrameError::MalformedHeader);
    }
    let type_val = (fc >> 2) & 0x03;
    let subtype = ((fc >> 4) & 0x0F) as u8;
    if type_val != 2 || (subtype != 0x00 && subtype != 0x08) {
        return Err(FrameError::UnsupportedSubtype);
    }
    // Enforce STA-to-AP infrastructure framing: ToDS == 1, FromDS == 0.
    if (fc & 0x0300) != 0x0100 {
        return Err(FrameError::MalformedHeader);
    }
    // Open SoftAP rejects Protected frames and FragAttacks fragments.
    if (fc & 0x4000) != 0 || (fc & 0x0400) != 0 || (raw_mpdu[22] & 0x0F) != 0 {
        return Err(FrameError::UnsupportedSubtype);
    }

    let is_qos = subtype == 0x08;
    let has_htc = is_qos && (fc & 0x8000) != 0;
    let hdr_len = 24 + if is_qos { 2 } else { 0 } + if has_htc { 4 } else { 0 };
    if raw_mpdu.len() < hdr_len {
        return Err(FrameError::MalformedHeader);
    }
    // FragAttacks (CVE-2020-24588): reject QoS A-MSDU Present frames.
    if is_qos && (raw_mpdu[24] & 0x80) != 0 {
        return Err(FrameError::UnsupportedSubtype);
    }

    if raw_mpdu[4..10] != *bssid || raw_mpdu[10..16] != *expected_sta_mac {
        return Err(FrameError::MalformedHeader);
    }

    let payload = &raw_mpdu[hdr_len..];
    if payload.len() < 8 || payload[0..6] != AP_LLC_SNAP_HDR {
        return Err(FrameError::InvalidPayload);
    }

    let ethertype = &payload[6..8];
    let body = &payload[8..];
    let total_eth_len = 14usize
        .checked_add(body.len())
        .ok_or(FrameError::BufferTooSmall)?;
    if eth_out.len() < total_eth_len {
        return Err(FrameError::BufferTooSmall);
    }

    eth_out[0..6].copy_from_slice(&raw_mpdu[16..22]);
    eth_out[6..12].copy_from_slice(&raw_mpdu[10..16]);
    eth_out[12..14].copy_from_slice(ethertype);
    eth_out[14..total_eth_len].copy_from_slice(body);
    Ok(total_eth_len)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_build_and_parse_probe_req() {
        let mut buf = [0u8; 128];
        let sta_mac = [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC];
        let len =
            build_probe_req(&mut buf, sta_mac, Some(b"TestSsid"), 1).expect("valid probe req");
        assert!(len > 24);

        let parsed = parse_80211_header(&buf[..len]).expect("valid 802.11 header");
        assert_eq!(parsed.frame_type, FrameType::Mgmt);
        assert_eq!(parsed.subtype, 0x04);
        assert_eq!(parsed.addr2, sta_mac);

        // header + SSID IE + Supported Rates IE + Extended Supported Rates IE
        assert_eq!(len, 24 + (2 + 8) + 10 + 6);
        // Supported Rates (Tag 1) then Extended Supported Rates (Tag 50):
        // 24, 36, 48, 54 Mbps.
        assert_eq!(&buf[34..36], &[0x01, 0x08]);
        assert_eq!(&buf[44..50], &EXT_SUPPORTED_RATES_IE);
        assert_eq!(EXT_SUPPORTED_RATES_IE, [0x32, 0x04, 0x30, 0x48, 0x60, 0x6C]);
    }

    #[test]
    fn test_build_open_auth_req() {
        let mut buf = [0u8; 64];
        let bssid = [0x00, 0x11, 0x22, 0x33, 0x44, 0x55];
        let sta = [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF];
        let len = build_open_auth_req(&mut buf, bssid, sta, 2).expect("valid auth req");
        assert_eq!(len, 30);

        let (seq_num, status) = parse_auth_resp(&buf[..len]).expect("valid auth resp");
        assert_eq!(seq_num, 1);
        assert_eq!(status, 0);
    }

    #[test]
    fn test_build_assoc_req() {
        let mut buf = [0u8; 128];
        let bssid = [0x00, 0x11, 0x22, 0x33, 0x44, 0x55];
        let sta = [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF];
        let len =
            build_assoc_req(&mut buf, bssid, sta, b"MySsid", &[], 3, 10).expect("valid assoc req");
        assert!(len > 30);
        // header + caps/listen + SSID IE + Supported Rates + Ext Supported Rates
        assert_eq!(len, 24 + 4 + (2 + 6) + 10 + 6);
        assert_eq!(&buf[len - 6..len], &EXT_SUPPORTED_RATES_IE);
    }

    #[test]
    fn test_build_assoc_req_orders_ext_rates_before_rsn_ie() {
        let mut buf = [0u8; 128];
        let bssid = [0x00, 0x11, 0x22, 0x33, 0x44, 0x55];
        let sta = [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF];
        let rsn = [0x30, 0x02, 0x01, 0x00];
        let len =
            build_assoc_req(&mut buf, bssid, sta, b"MySsid", &rsn, 3, 10).expect("valid assoc req");

        // IEs start at 28: SSID (8), Supported Rates (10), Extended
        // Supported Rates (6), then RSN.
        assert_eq!(len, 28 + 8 + 10 + 6 + rsn.len());
        assert_eq!(&buf[36..38], &[0x01, 0x08]);
        assert_eq!(&buf[46..52], &EXT_SUPPORTED_RATES_IE);
        assert_eq!(&buf[52..len], &rsn);
    }

    #[test]
    fn test_builders_reject_buffer_one_byte_short_of_ext_rates() {
        let bssid = [0x00, 0x11, 0x22, 0x33, 0x44, 0x55];
        let sta = [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF];
        let mut buf = [0u8; 24 + 4 + 8 + 10 + 6 - 1];
        assert_eq!(
            build_assoc_req(&mut buf, bssid, sta, b"MySsid", &[], 3, 10),
            Err(FrameError::BufferTooSmall)
        );
        let mut probe = [0u8; 24 + 10 + 10 + 6 - 1];
        assert_eq!(
            build_probe_req(&mut probe, sta, Some(b"TestSsid"), 1),
            Err(FrameError::BufferTooSmall)
        );
    }

    #[test]
    fn test_build_addba_decline() {
        let mut buf = [0u8; 64];
        let bssid = [0x00, 0x11, 0x22, 0x33, 0x44, 0x55];
        let sta = [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF];
        let len = build_addba_decline(&mut buf, bssid, sta, 0x07, 0x1000, 5)
            .expect("valid addba decline");
        assert_eq!(len, 33);
        assert_eq!(buf[27..29], 37u16.to_le_bytes()); // Status Code 37
    }

    #[test]
    fn test_build_null_data() {
        let mut buf = [0u8; 32];
        let bssid = [0x00, 0x11, 0x22, 0x33, 0x44, 0x55];
        let sta = [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF];
        let len = build_null_data_keepalive(&mut buf, bssid, sta, 10, true)
            .expect("valid null data keepalive");
        assert_eq!(len, 24);
        assert_eq!(buf[0], 0x48);
        assert_eq!(buf[1] & 0x10, 0x10); // Power Management bit set
    }

    #[test]
    fn test_ccmp_roundtrip_and_eapol_demux() {
        let bssid = [0x00, 0x11, 0x22, 0x33, 0x44, 0x55];
        let tk = [0x55u8; 16];

        // 1. Construct Ethernet-II frame and encrypt ToDS=1 via eth_to_80211_data
        let mut eth_tx = [0u8; 64];
        eth_tx[0..6].copy_from_slice(&[0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC]); // DA
        eth_tx[6..12].copy_from_slice(&[0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF]); // SA
        eth_tx[12..14].copy_from_slice(&[0x08, 0x00]); // IPv4
        eth_tx[14..24].copy_from_slice(b"Hello IP!!");

        let mut to_ds_mpdu = [0u8; 256];
        let to_ds_len = eth_to_80211_data(
            &mut to_ds_mpdu,
            &eth_tx[..24],
            bssid,
            1,
            Some((&tk, 0, 100)),
        )
        .expect("eth_to_80211_data failed");

        // Verify eth_to_80211_data (ToDS=1) with ccmp_decrypt_mpdu
        let mut to_ds_plain = [0u8; 256];
        let (pt_len, key_id, pn) =
            ccmp_decrypt_mpdu(&tk, &to_ds_mpdu[..to_ds_len], &mut to_ds_plain)
                .expect("ccmp_decrypt_mpdu failed on ToDS frame");
        assert_eq!(key_id, 0);
        assert_eq!(pn, 100);
        assert_eq!(pt_len, 18);
        assert_eq!(&to_ds_plain[0..6], &[0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00]);
        assert_eq!(&to_ds_plain[6..8], &[0x08, 0x00]);
        assert_eq!(&to_ds_plain[8..pt_len], b"Hello IP!!");

        // 2. Encrypt a genuine FromDS=1 MPDU and verify rx_80211_data_to_eth
        let mut from_ds_mac_hdr = [0u8; 24];
        from_ds_mac_hdr[0] = 0x08; // Type=Data, Subtype=0
        from_ds_mac_hdr[1] = 0x42; // FromDS=1, Protected=1
        from_ds_mac_hdr[2..4].copy_from_slice(&[0x00, 0x00]);
        from_ds_mac_hdr[4..10].copy_from_slice(&eth_tx[0..6]); // Addr1: DA
        from_ds_mac_hdr[10..16].copy_from_slice(&bssid); // Addr2: BSSID (TA)
        from_ds_mac_hdr[16..22].copy_from_slice(&eth_tx[6..12]); // Addr3: SA
        from_ds_mac_hdr[22..24].copy_from_slice(&(2u16 << 4).to_le_bytes()); // Seq

        let mut from_ds_plain = [0u8; 18];
        from_ds_plain[0..6].copy_from_slice(&[0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00]);
        from_ds_plain[6..8].copy_from_slice(&[0x08, 0x00]);
        from_ds_plain[8..18].copy_from_slice(b"Hello IP!!");

        let mut from_ds_mpdu = [0u8; 256];
        let from_ds_len = ccmp_encrypt_mpdu(
            &tk,
            0,
            101,
            &from_ds_mac_hdr,
            &from_ds_plain,
            &mut from_ds_mpdu,
        )
        .expect("ccmp_encrypt_mpdu for FromDS frame failed");

        let mut scratch = [0u8; 256];
        let mut eth_rx = [0u8; 256];
        let sta_mac = [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC];

        let outcome = rx_80211_data_to_eth(
            &from_ds_mpdu[..from_ds_len],
            &mut scratch,
            &mut eth_rx,
            Some(&tk),
            None,
            &bssid,
            &sta_mac,
            true,
        )
        .expect("rx_80211_data_to_eth failed on valid FromDS frame");

        match outcome {
            RxDataOutcome::Ethernet { len: rx_len, ccmp } => {
                assert_eq!(rx_len, 24);
                assert_eq!(&eth_rx[..rx_len], &eth_tx[..24]);
                let meta = ccmp.expect("CCMP metadata must be present");
                assert_eq!(meta.pn, 101);
                assert_eq!(meta.key_id, 0);
                assert_eq!(meta.qos_tid, None);
            }
            RxDataOutcome::Eapol { .. } => panic!("expected Ethernet outcome"),
        }

        // 3. Tampered MPDU rejection test
        from_ds_mpdu[from_ds_len - 1] ^= 0xFF; // corrupt MIC
        let err = rx_80211_data_to_eth(
            &from_ds_mpdu[..from_ds_len],
            &mut scratch,
            &mut eth_rx,
            Some(&tk),
            None,
            &bssid,
            &sta_mac,
            true,
        );
        assert_eq!(err, Err(FrameError::CryptoError));

        // 4. Pre-key EAPOL frame test (unicast to sta_mac)
        let mut eapol_tx = [0u8; 40];
        eapol_tx[0..6].copy_from_slice(&sta_mac);
        eapol_tx[6..12].copy_from_slice(&[0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF]);
        eapol_tx[12..14].copy_from_slice(&[0x88, 0x8E]); // EAPOL
        eapol_tx[14..24].copy_from_slice(b"EAPOL_TEST");

        let mut eapol_mpdu = [0u8; 128];
        let eapol_len = eth_to_80211_data(
            &mut eapol_mpdu,
            &eapol_tx[..24],
            bssid,
            2,
            None, // unencrypted EAPOL before keys
        )
        .expect("valid eapol mpdu");

        eapol_mpdu[1] = 0x02; // FromDS=1
        eapol_mpdu[4..10].copy_from_slice(&sta_mac);
        eapol_mpdu[10..16].copy_from_slice(&bssid);
        eapol_mpdu[16..22].copy_from_slice(&eapol_tx[6..12]);

        let eapol_outcome = rx_80211_data_to_eth(
            &eapol_mpdu[..eapol_len],
            &mut scratch,
            &mut eth_rx,
            None,
            None,
            &bssid,
            &sta_mac,
            true,
        )
        .expect("valid rx eapol");

        match eapol_outcome {
            RxDataOutcome::Eapol { len: e_len, ccmp } => {
                assert_eq!(e_len, 10);
                assert_eq!(ccmp, None);
                assert_eq!(&eth_rx[..e_len], b"EAPOL_TEST");
            }
            RxDataOutcome::Ethernet { .. } => panic!("expected Eapol outcome"),
        }
    }

    #[test]
    fn test_fragattacks_and_frame_validation() {
        let bssid = [0x00, 0x11, 0x22, 0x33, 0x44, 0x55];
        let sta_mac = [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC];
        let sa = [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF];
        let tk = [0x42u8; 16];
        let gtk = [0x77u8; 16];
        let mut scratch = [0u8; 256];
        let mut out = [0u8; 256];

        // Build plaintext FromDS=1 EAPOL frame
        let mut eapol_mpdu = [0u8; 42];
        eapol_mpdu[0] = 0x08;
        eapol_mpdu[1] = 0x02; // FromDS=1, Protected=0
        eapol_mpdu[4..10].copy_from_slice(&sta_mac);
        eapol_mpdu[10..16].copy_from_slice(&bssid);
        eapol_mpdu[16..22].copy_from_slice(&sa);
        eapol_mpdu[24..32].copy_from_slice(&[0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00, 0x88, 0x8E]);
        eapol_mpdu[32..42].copy_from_slice(b"EAPOL_BODY");

        // CVE-2020-26140: Plaintext EAPOL MUST be rejected once PTK is active.
        assert_eq!(
            rx_80211_data_to_eth(
                &eapol_mpdu,
                &mut scratch,
                &mut out,
                Some(&tk),
                None,
                &bssid,
                &sta_mac,
                true,
            ),
            Err(FrameError::UnencryptedPayloadRejected)
        );

        // Broadcast EAPOL MUST be rejected even pre-key.
        let mut mcast_eapol = eapol_mpdu;
        mcast_eapol[4..10].fill(0xFF);
        assert_eq!(
            rx_80211_data_to_eth(
                &mcast_eapol,
                &mut scratch,
                &mut out,
                None,
                None,
                &bssid,
                &sta_mac,
                true,
            ),
            Err(FrameError::MalformedHeader)
        );

        // Pre-key plaintext IP data on a protected BSS MUST be rejected.
        let mut ip_mpdu = eapol_mpdu;
        ip_mpdu[30..32].copy_from_slice(&[0x08, 0x00]);
        assert_eq!(
            rx_80211_data_to_eth(
                &ip_mpdu,
                &mut scratch,
                &mut out,
                None,
                None,
                &bssid,
                &sta_mac,
                true,
            ),
            Err(FrameError::UnencryptedPayloadRejected)
        );

        // Fragmented MPDU (More Fragments bit or non-zero Frag Num) rejected.
        let mut frag_mf = eapol_mpdu;
        frag_mf[1] |= 0x04;
        assert_eq!(
            rx_80211_data_to_eth(
                &frag_mf,
                &mut scratch,
                &mut out,
                None,
                None,
                &bssid,
                &sta_mac,
                false,
            ),
            Err(FrameError::UnsupportedSubtype)
        );
        let mut frag_num = eapol_mpdu;
        frag_num[22] = 0x01;
        assert_eq!(
            rx_80211_data_to_eth(
                &frag_num,
                &mut scratch,
                &mut out,
                None,
                None,
                &bssid,
                &sta_mac,
                false,
            ),
            Err(FrameError::UnsupportedSubtype)
        );

        // QoS A-MSDU Present rejected.
        let mut amsdu = [0u8; 44];
        amsdu[0] = 0x88; // QoS Data
        amsdu[1] = 0x02; // FromDS=1
        amsdu[4..10].copy_from_slice(&sta_mac);
        amsdu[10..16].copy_from_slice(&bssid);
        amsdu[16..22].copy_from_slice(&sa);
        amsdu[24] = 0x80; // A-MSDU Present
        assert_eq!(
            rx_80211_data_to_eth(
                &amsdu,
                &mut scratch,
                &mut out,
                None,
                None,
                &bssid,
                &sta_mac,
                false,
            ),
            Err(FrameError::UnsupportedSubtype)
        );

        // Null-Data subtype rejected.
        let mut null_data = eapol_mpdu;
        null_data[0] = 0x48;
        assert_eq!(
            rx_80211_data_to_eth(
                &null_data,
                &mut scratch,
                &mut out,
                None,
                None,
                &bssid,
                &sta_mac,
                false,
            ),
            Err(FrameError::UnsupportedSubtype)
        );

        // Protocol Version != 0, ToDS=1, and wrong unicast DA rejected.
        let mut bad_ver = eapol_mpdu;
        bad_ver[0] |= 0x01;
        assert_eq!(
            rx_80211_data_to_eth(
                &bad_ver,
                &mut scratch,
                &mut out,
                None,
                None,
                &bssid,
                &sta_mac,
                false,
            ),
            Err(FrameError::MalformedHeader)
        );
        let mut to_ds = eapol_mpdu;
        to_ds[1] = 0x01;
        assert_eq!(
            rx_80211_data_to_eth(
                &to_ds,
                &mut scratch,
                &mut out,
                None,
                None,
                &bssid,
                &sta_mac,
                false,
            ),
            Err(FrameError::MalformedHeader)
        );
        let mut wrong_da = eapol_mpdu;
        wrong_da[4] ^= 0x02;
        assert_eq!(
            rx_80211_data_to_eth(
                &wrong_da,
                &mut scratch,
                &mut out,
                None,
                None,
                &bssid,
                &sta_mac,
                false,
            ),
            Err(FrameError::MalformedHeader)
        );

        // CCMP Key ID mismatch rejected (encrypted with key_id 2 vs expected GTK id 1).
        let mut hdr = [0u8; 24];
        hdr[0] = 0x08;
        hdr[1] = 0x42;
        hdr[4..10].fill(0xFF);
        hdr[10..16].copy_from_slice(&bssid);
        hdr[16..22].copy_from_slice(&sa);
        let pt = [0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00, 0x08, 0x00, 0x11, 0x22];
        let mut enc = [0u8; 128];
        let enc_len = ccmp_encrypt_mpdu(&gtk, 2, 1, &hdr, &pt, &mut enc).expect("encrypt");
        assert_eq!(
            rx_80211_data_to_eth(
                &enc[..enc_len],
                &mut scratch,
                &mut out,
                Some(&tk),
                Some((1, &gtk)),
                &bssid,
                &sta_mac,
                true,
            ),
            Err(FrameError::KeyIdMismatch)
        );
    }

    #[test]
    fn test_builders_and_mgmt_parsers_hardening() {
        let bssid = [0x00, 0x11, 0x22, 0x33, 0x44, 0x55];
        let sta = [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF];
        let mut buf = [0u8; 256];

        // Overlong SSID rejected in probe_req and assoc_req.
        assert_eq!(
            build_probe_req(&mut buf, sta, Some(&[b'A'; 33]), 1),
            Err(FrameError::InvalidPayload)
        );
        assert_eq!(
            build_assoc_req(&mut buf, bssid, sta, &[b'A'; 33], &[], 1, 10),
            Err(FrameError::InvalidPayload)
        );
        // Malformed RSN IE TLV rejected in assoc_req.
        assert_eq!(
            build_assoc_req(&mut buf, bssid, sta, b"Ok", &[0x30, 0x05, 0x01], 1, 10),
            Err(FrameError::InvalidPayload)
        );

        // TX PN == 0 or > MAX_CCMP_PN rejected in eth_to_80211_data.
        let mut eth = [0u8; 24];
        eth[12..14].copy_from_slice(&[0x08, 0x00]);
        let tk = [0x11u8; 16];
        assert_eq!(
            eth_to_80211_data(&mut buf, &eth, bssid, 1, Some((&tk, 0, 0))),
            Err(FrameError::InvalidPacketNumber)
        );
        assert_eq!(
            eth_to_80211_data(&mut buf, &eth, bssid, 1, Some((&tk, 0, MAX_CCMP_PN + 1))),
            Err(FrameError::InvalidPacketNumber)
        );

        // Auth resp with non-Open algorithm rejected.
        let mut auth = [0u8; 30];
        auth[0] = 0xB0;
        auth[24] = 0x01; // Shared Key
        auth[26] = 0x02;
        assert_eq!(parse_auth_resp(&auth), Err(FrameError::UnsupportedSubtype));

        // Assoc resp missing 0xC000 bits or AID == 0 rejected on status 0.
        let mut assoc = [0u8; 30];
        assoc[0] = 0x10;
        assoc[28..30].copy_from_slice(&1u16.to_le_bytes()); // missing 0xC000
        assert_eq!(parse_assoc_resp(&assoc), Err(FrameError::MalformedHeader));
        assoc[28..30].copy_from_slice(&0xC000u16.to_le_bytes()); // AID == 0
        assert_eq!(parse_assoc_resp(&assoc), Err(FrameError::MalformedHeader));

        // Beacon first-wins IE + Option channel (channel 1 preserved over Tag 61).
        let mut bcn = [0u8; 64];
        bcn[0] = 0x80;
        bcn[16..22].copy_from_slice(&bssid);
        bcn[34] = 0x11;
        let mut p = 36;
        bcn[p..p + 4].copy_from_slice(&[0, 2, b'O', b'K']);
        p += 4;
        bcn[p..p + 3].copy_from_slice(&[3, 1, 1]); // DS channel 1
        p += 3;
        bcn[p..p + 3].copy_from_slice(&[61, 1, 11]); // HT channel 11 must not overwrite DS ch 1
        p += 3;
        bcn[p..p + 5].copy_from_slice(&[0, 3, b'B', b'A', b'D']); // duplicate SSID ignored
        p += 5;
        let info = parse_beacon_or_probe_resp(&bcn[..p], -50).expect("valid beacon");
        assert_eq!(info.ssid, b"OK");
        assert_eq!(info.channel, Some(1));

        // Control frame (10-byte ACK) parsed without reading past bounds.
        let mut ack = [0u8; 10];
        ack[0] = 0xD4; // Type=Control (1), Subtype=ACK (0xD)
        ack[4..10].copy_from_slice(&sta);
        let parsed_ack = parse_80211_header(&ack).expect("valid control ACK");
        assert_eq!(parsed_ack.frame_type, FrameType::Control);
        assert_eq!(parsed_ack.addr1, sta);
        assert_eq!(parsed_ack.addr2, [0u8; 6]);
    }

    #[test]
    fn test_ap_beacon_and_probe_resp_roundtrip() {
        let bssid = [0xD4, 0xD4, 0xDA, 0x5B, 0xD4, 0xEC];
        let sta = [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC];
        let ssid = b"KlipperCam-Setup-D4EC";
        let mut buf = [0u8; 128];

        let bcn_len = build_beacon(&bssid, ssid, 1, 100, 0x1122_3344_5566_7788, 7, &mut buf)
            .expect("valid beacon");
        assert_eq!(bcn_len, 24 + 12 + (2 + ssid.len()) + 10 + 3 + 6 + 6);
        let bcn_hdr = parse_80211_header(&buf[..bcn_len]).expect("valid beacon hdr");
        assert_eq!(bcn_hdr.frame_type, FrameType::Mgmt);
        assert_eq!(bcn_hdr.subtype, 0x08);
        assert_eq!(bcn_hdr.addr1, [0xFF; 6]);
        assert_eq!(bcn_hdr.addr2, bssid);
        assert_eq!(bcn_hdr.addr3, bssid);
        assert_eq!(bcn_hdr.seq_ctrl, 7u16 << 4);
        assert_eq!(&buf[24..32], &0x1122_3344_5566_7788u64.to_le_bytes());
        assert_eq!(&buf[32..34], &100u16.to_le_bytes());
        assert_eq!(&buf[34..36], &0x0401u16.to_le_bytes());

        let parsed_bcn = parse_beacon_or_probe_resp(&buf[..bcn_len], -40).expect("parse beacon");
        assert_eq!(parsed_bcn.bssid, bssid);
        assert_eq!(parsed_bcn.ssid, ssid);
        assert_eq!(parsed_bcn.channel, Some(1));
        assert!(!parsed_bcn.privacy);
        assert_eq!(parsed_bcn.rsn_ie, None);
        assert_eq!(parsed_bcn.wpa_ie, None);
        assert!(!parsed_bcn.has_ccmp);
        assert!(!parsed_bcn.has_psk);

        let pr_len = build_probe_resp(&sta, &bssid, ssid, 6, 100, 999, 8, &mut buf)
            .expect("valid probe resp");
        assert_eq!(pr_len, 24 + 12 + (2 + ssid.len()) + 10 + 3 + 6);
        let pr_hdr = parse_80211_header(&buf[..pr_len]).expect("valid probe resp hdr");
        assert_eq!(pr_hdr.frame_type, FrameType::Mgmt);
        assert_eq!(pr_hdr.subtype, 0x05);
        assert_eq!(pr_hdr.addr1, sta);
        assert_eq!(pr_hdr.addr2, bssid);
        assert_eq!(pr_hdr.addr3, bssid);
        let parsed_pr = parse_beacon_or_probe_resp(&buf[..pr_len], -45).expect("parse probe resp");
        assert_eq!(parsed_pr.bssid, bssid);
        assert_eq!(parsed_pr.ssid, ssid);
        assert_eq!(parsed_pr.channel, Some(6));
        assert!(!parsed_pr.privacy);

        // Reject SSID > 32 B, invalid channel, and undersized buffer.
        assert_eq!(
            build_beacon(&bssid, &[b'X'; 33], 1, 100, 0, 0, &mut buf),
            Err(FrameError::InvalidPayload)
        );
        assert_eq!(
            build_probe_resp(&sta, &bssid, &[b'X'; 33], 1, 100, 0, 0, &mut buf),
            Err(FrameError::InvalidPayload)
        );
        assert_eq!(
            build_beacon(&bssid, ssid, 0, 100, 0, 0, &mut buf),
            Err(FrameError::InvalidPayload)
        );
        assert_eq!(
            build_probe_resp(&sta, &bssid, ssid, 15, 100, 0, 0, &mut buf),
            Err(FrameError::InvalidPayload)
        );
        let mut short_bcn = [0u8; 63 + 21 - 1];
        assert_eq!(
            build_beacon(&bssid, ssid, 1, 100, 0, 0, &mut short_bcn),
            Err(FrameError::BufferTooSmall)
        );
        let mut short_pr = [0u8; 57 + 21 - 1];
        assert_eq!(
            build_probe_resp(&sta, &bssid, ssid, 1, 100, 0, 0, &mut short_pr),
            Err(FrameError::BufferTooSmall)
        );
    }

    #[test]
    fn test_ap_parse_probe_req_wildcard_exact_and_malformed() {
        let sta = [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC];
        let mut buf = [0u8; 128];

        let wc_len = build_probe_req(&mut buf, sta, None, 1).expect("wildcard probe req");
        let wc_hdr = parse_80211_header(&buf[..wc_len]).expect("wc hdr");
        assert_eq!(
            parse_probe_req(wc_hdr.payload),
            Ok(ProbeReqInfo { ssid: None })
        );

        let ex_len = build_probe_req(&mut buf, sta, Some(b"KlipperCam-Setup-D4EC"), 2)
            .expect("exact probe req");
        let ex_hdr = parse_80211_header(&buf[..ex_len]).expect("ex hdr");
        assert_eq!(
            parse_probe_req(ex_hdr.payload),
            Ok(ProbeReqInfo {
                ssid: Some(b"KlipperCam-Setup-D4EC"),
            })
        );

        assert_eq!(parse_probe_req(&[]), Ok(ProbeReqInfo { ssid: None }));

        // Review Focus #4: reject SSID > 32 B and truncated TLVs.
        let mut long_ssid = [0u8; 35];
        long_ssid[0] = 0;
        long_ssid[1] = 33;
        assert_eq!(parse_probe_req(&long_ssid), Err(FrameError::InvalidPayload));
        assert_eq!(parse_probe_req(&[0x00]), Err(FrameError::InvalidPayload));
        assert_eq!(
            parse_probe_req(&[0x00, 0x04, b'A', b'B']),
            Err(FrameError::InvalidPayload)
        );
        assert_eq!(
            parse_probe_req(&[0x00, 0x02, b'O', b'K', 0x01, 0x04, 0x82]),
            Err(FrameError::InvalidPayload)
        );
    }

    #[test]
    fn test_ap_auth_assoc_and_deauth_roundtrip() {
        let bssid = [0xD4, 0xD4, 0xDA, 0x5B, 0xD4, 0xEC];
        let sta = [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF];
        let ssid = b"KlipperCam-Setup-D4EC";
        let mut buf = [0u8; 128];

        // Open Auth request parse & response build roundtrip.
        let auth_req_len = build_open_auth_req(&mut buf, bssid, sta, 1).expect("auth req");
        let auth_hdr = parse_80211_header(&buf[..auth_req_len]).expect("auth hdr");
        assert_eq!(parse_open_auth_req(auth_hdr.payload), Ok((0, 1)));
        assert_eq!(
            parse_open_auth_req(&[0x01, 0x00, 0x01, 0x00, 0x00, 0x00]),
            Ok((1, 1))
        );
        assert_eq!(
            parse_open_auth_req(&[0x00, 0x00, 0x01, 0x00, 0x00]),
            Err(FrameError::InvalidPayload)
        );

        let auth_resp_len =
            build_open_auth_resp(&sta, &bssid, 2, 0, 9, &mut buf).expect("auth resp");
        assert_eq!(auth_resp_len, 30);
        assert_eq!(parse_auth_resp(&buf[..auth_resp_len]), Ok((2, 0)));
        let ar_hdr = parse_80211_header(&buf[..auth_resp_len]).expect("ar hdr");
        assert_eq!(ar_hdr.addr1, sta);
        assert_eq!(ar_hdr.addr2, bssid);
        assert_eq!(ar_hdr.addr3, bssid);
        let mut small = [0u8; 29];
        assert_eq!(
            build_open_auth_resp(&sta, &bssid, 2, 13, 9, &mut small),
            Err(FrameError::BufferTooSmall)
        );

        // Assoc request parse & response build roundtrip.
        let assoc_req_len =
            build_assoc_req(&mut buf, bssid, sta, ssid, &[], 3, 10).expect("assoc req");
        let assoc_hdr = parse_80211_header(&buf[..assoc_req_len]).expect("assoc hdr");
        assert_eq!(
            parse_assoc_req(assoc_hdr.payload),
            Ok(AssocReqInfo {
                capability: 0x0431,
                listen_interval: 10,
                ssid,
            })
        );
        assert_eq!(
            parse_assoc_req(&[0x31, 0x04, 0x0A]),
            Err(FrameError::InvalidPayload)
        );
        assert_eq!(
            parse_assoc_req(&[0x31, 0x04, 0x0A, 0x00, 0x01, 0x01, 0x82]),
            Err(FrameError::InvalidPayload)
        );
        let mut bad_assoc_ssid = [0u8; 4 + 2 + 33];
        bad_assoc_ssid[4] = 0;
        bad_assoc_ssid[5] = 33;
        assert_eq!(
            parse_assoc_req(&bad_assoc_ssid),
            Err(FrameError::InvalidPayload)
        );
        assert_eq!(
            parse_assoc_req(&[0x31, 0x04, 0x0A, 0x00, 0x00, 0x05, b'A']),
            Err(FrameError::InvalidPayload)
        );

        let assoc_resp_len =
            build_assoc_resp(&sta, &bssid, 3, 0, 10, &mut buf).expect("assoc resp");
        assert_eq!(assoc_resp_len, 46);
        assert_eq!(&buf[24..26], &0x0401u16.to_le_bytes());
        assert_eq!(&buf[28..30], &(3u16 | 0xC000).to_le_bytes());
        assert_eq!(parse_assoc_resp(&buf[..assoc_resp_len]), Ok((0, 3)));
        let assoc_fail_len =
            build_assoc_resp(&sta, &bssid, 0, 17, 11, &mut buf).expect("assoc fail resp");
        assert_eq!(parse_assoc_resp(&buf[..assoc_fail_len]), Ok((17, 0)));
        assert_eq!(
            build_assoc_resp(&sta, &bssid, 0, 0, 11, &mut buf),
            Err(FrameError::InvalidPayload)
        );
        let mut short_assoc = [0u8; 45];
        assert_eq!(
            build_assoc_resp(&sta, &bssid, 1, 0, 11, &mut short_assoc),
            Err(FrameError::BufferTooSmall)
        );

        // Reassoc response build roundtrip (fc_lo == 0x30, subtype == 0x03).
        let reassoc_resp_len =
            build_reassoc_resp(&sta, &bssid, 4, 0, 13, &mut buf).expect("reassoc resp");
        assert_eq!(reassoc_resp_len, 46);
        assert_eq!(buf[0], 0x30);
        let reassoc_hdr = parse_80211_header(&buf[..reassoc_resp_len]).expect("reassoc hdr");
        assert_eq!(reassoc_hdr.frame_type, FrameType::Mgmt);
        assert_eq!(reassoc_hdr.subtype, 0x03);
        assert_eq!(reassoc_hdr.addr1, sta);
        assert_eq!(reassoc_hdr.addr2, bssid);
        assert_eq!(reassoc_hdr.addr3, bssid);
        assert_eq!(reassoc_hdr.seq_ctrl, 13u16 << 4);
        assert_eq!(parse_reassoc_resp(&buf[..reassoc_resp_len]), Ok((0, 4)));
        assert_eq!(
            parse_assoc_resp(&buf[..reassoc_resp_len]),
            Err(FrameError::UnsupportedSubtype)
        );
        assert_eq!(
            build_reassoc_resp(&sta, &bssid, 0, 0, 13, &mut buf),
            Err(FrameError::InvalidPayload)
        );
        assert_eq!(
            build_reassoc_resp(&sta, &bssid, 1, 0, 13, &mut short_assoc),
            Err(FrameError::BufferTooSmall)
        );

        // AP Deauth build roundtrip.
        let deauth_len = build_ap_deauth(&sta, &bssid, 7, 12, &mut buf).expect("ap deauth");
        assert_eq!(deauth_len, 26);
        let d_hdr = parse_80211_header(&buf[..deauth_len]).expect("deauth hdr");
        assert_eq!(d_hdr.addr1, sta);
        assert_eq!(d_hdr.addr2, bssid);
        assert_eq!(d_hdr.addr3, bssid);
        assert_eq!(parse_deauth_or_disassoc(&buf[..deauth_len]), Ok((0x0C, 7)));
        let mut short_deauth = [0u8; 25];
        assert_eq!(
            build_ap_deauth(&sta, &bssid, 7, 12, &mut short_deauth),
            Err(FrameError::BufferTooSmall)
        );
    }

    #[test]
    fn test_ap_data_bridge_and_review_focus_4_rejections() {
        let bssid = [0xD4, 0xD4, 0xDA, 0x5B, 0xD4, 0xEC];
        let sta = [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC];
        let peer_da = [0xFF; 6];
        let mut mpdu = [0u8; 128];
        let mut eth_out = [0u8; 128];

        // 1. AP TX (ToDS=0, FromDS=1) roundtrip via STA rx_80211_data_to_eth.
        let mut eth_down = [0u8; 24];
        eth_down[0..6].copy_from_slice(&sta);
        eth_down[6..12].copy_from_slice(&bssid);
        eth_down[12..14].copy_from_slice(&[0x08, 0x00]);
        eth_down[14..24].copy_from_slice(b"AP_TO_STA!");
        let tx_len = ap_eth_to_80211_data(&eth_down, &bssid, 4, &mut mpdu).expect("ap tx");
        assert_eq!(tx_len, 32 + 10);
        let tx_hdr = parse_80211_header(&mpdu[..tx_len]).expect("tx hdr");
        assert!(!tx_hdr.to_ds);
        assert!(tx_hdr.from_ds);
        assert!(!tx_hdr.is_protected);
        assert_eq!(tx_hdr.addr1, sta);
        assert_eq!(tx_hdr.addr2, bssid);
        assert_eq!(tx_hdr.addr3, bssid);
        let mut scratch = [0u8; 128];
        assert_eq!(
            rx_80211_data_to_eth(
                &mpdu[..tx_len],
                &mut scratch,
                &mut eth_out,
                None,
                None,
                &bssid,
                &sta,
                false,
            ),
            Ok(RxDataOutcome::Ethernet {
                len: 24,
                ccmp: None,
            })
        );
        assert_eq!(&eth_out[..24], &eth_down);
        assert_eq!(
            ap_eth_to_80211_data(&eth_down[..13], &bssid, 4, &mut mpdu),
            Err(FrameError::InvalidPayload)
        );
        let mut short_mpdu = [0u8; 41];
        assert_eq!(
            ap_eth_to_80211_data(&eth_down, &bssid, 4, &mut short_mpdu),
            Err(FrameError::BufferTooSmall)
        );

        // 2. AP RX (ToDS=1, FromDS=0) roundtrip from STA eth_to_80211_data.
        let mut eth_up = [0u8; 24];
        eth_up[0..6].copy_from_slice(&peer_da);
        eth_up[6..12].copy_from_slice(&sta);
        eth_up[12..14].copy_from_slice(&[0x08, 0x06]);
        eth_up[14..24].copy_from_slice(b"STA_TO_AP!");
        let up_len =
            eth_to_80211_data(&mut mpdu, &eth_up, bssid, 5, None).expect("sta tx plaintext");
        let rx_len = ap_rx_80211_data_to_eth(&mpdu[..up_len], &bssid, &sta, &mut eth_out)
            .expect("ap rx valid");
        assert_eq!(rx_len, 24);
        assert_eq!(&eth_out[..rx_len], &eth_up);

        // QoS Data (ToDS=1, A-MSDU=0) accepted.
        let mut qos = [0u8; 44];
        qos[0] = 0x88;
        qos[1] = 0x01;
        qos[4..10].copy_from_slice(&bssid);
        qos[10..16].copy_from_slice(&sta);
        qos[16..22].copy_from_slice(&peer_da);
        qos[24..26].copy_from_slice(&[0x00, 0x00]);
        qos[26..34].copy_from_slice(&[0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00, 0x08, 0x00]);
        qos[34..44].copy_from_slice(b"QOS_DATA!!");
        let qos_len =
            ap_rx_80211_data_to_eth(&qos, &bssid, &sta, &mut eth_out).expect("qos data ok");
        assert_eq!(qos_len, 24);
        assert_eq!(&eth_out[14..24], b"QOS_DATA!!");

        // Review Focus #4: reject Protected, Fragmented, A-MSDU, wrong addrs/DS.
        let mut protected = [0u8; 42];
        protected.copy_from_slice(&mpdu[..up_len]);
        protected[1] |= 0x40;
        assert!(ap_rx_80211_data_to_eth(&protected, &bssid, &sta, &mut eth_out).is_err());

        let mut frag_mf = [0u8; 42];
        frag_mf.copy_from_slice(&mpdu[..up_len]);
        frag_mf[1] |= 0x04;
        assert!(ap_rx_80211_data_to_eth(&frag_mf, &bssid, &sta, &mut eth_out).is_err());

        let mut frag_num = [0u8; 42];
        frag_num.copy_from_slice(&mpdu[..up_len]);
        frag_num[22] |= 0x01;
        assert!(ap_rx_80211_data_to_eth(&frag_num, &bssid, &sta, &mut eth_out).is_err());

        let mut amsdu = qos;
        amsdu[24] |= 0x80;
        assert!(ap_rx_80211_data_to_eth(&amsdu, &bssid, &sta, &mut eth_out).is_err());

        let mut wrong_addr1 = [0u8; 42];
        wrong_addr1.copy_from_slice(&mpdu[..up_len]);
        wrong_addr1[4] ^= 0x01;
        assert!(ap_rx_80211_data_to_eth(&wrong_addr1, &bssid, &sta, &mut eth_out).is_err());

        let mut wrong_addr2 = [0u8; 42];
        wrong_addr2.copy_from_slice(&mpdu[..up_len]);
        wrong_addr2[10] ^= 0x01;
        assert!(ap_rx_80211_data_to_eth(&wrong_addr2, &bssid, &sta, &mut eth_out).is_err());

        let mut from_ds = [0u8; 42];
        from_ds.copy_from_slice(&mpdu[..up_len]);
        from_ds[1] = 0x02;
        assert!(ap_rx_80211_data_to_eth(&from_ds, &bssid, &sta, &mut eth_out).is_err());

        let mut bad_llc = [0u8; 42];
        bad_llc.copy_from_slice(&mpdu[..up_len]);
        bad_llc[24] = 0x00;
        assert_eq!(
            ap_rx_80211_data_to_eth(&bad_llc, &bssid, &sta, &mut eth_out),
            Err(FrameError::InvalidPayload)
        );

        let mut small_eth = [0u8; 23];
        assert_eq!(
            ap_rx_80211_data_to_eth(&mpdu[..up_len], &bssid, &sta, &mut small_eth),
            Err(FrameError::BufferTooSmall)
        );
    }
}
