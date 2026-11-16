//! IEEE 802.11 MAC frame parser, CCMP crypto, and Ethernet-II bridge.
//!
//! Provides zero-allocation, `#![no_std]` parsers and builders for 802.11
//! management frames (Probe, Auth, Assoc, Deauth, Block ACK ADDBA decline)
//! and data frames, bridging 802.11 MPDUs and Ethernet-II frames with
//! hardware CCMP encryption and LLC-SNAP translation.

use core::fmt;
use esp32_cam_wpa2::crypto::ccm::ccmp_decrypt_mpdu;
use esp32_cam_wpa2::crypto::ccm::ccmp_encrypt_mpdu;
use esp32_cam_wpa2::ie::parse_wpa_or_rsn_ie;

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
    /// Primary radio channel frequency index (1..=14).
    pub channel: u8,
    /// True if AP announces WEP/WPA/RSN privacy capability.
    pub privacy: bool,
    /// Received signal strength indication (dBm).
    pub rssi: i8,
    /// Raw RSN Information Element slice (Tag 48), if present.
    pub rsn_ie: Option<&'a [u8]>,
    /// Raw WPA Information Element slice (Tag 221 with Microsoft OUI), if present.
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

/// Outcome of processing a received 802.11 Data MPDU.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RxDataOutcome {
    /// Valid Ethernet II frame written to destination slice with length in bytes.
    Ethernet(usize),
    /// EAPOL 802.1X handshake payload extracted with length in bytes.
    Eapol(usize),
}

/// Parses an 802.11 MAC frame header and payload.
pub fn parse_80211_header(raw_mpdu: &[u8]) -> Result<ParsedFrame<'_>, FrameError> {
    if raw_mpdu.len() < 24 {
        return Err(FrameError::MalformedHeader);
    }
    let fc = u16::from_le_bytes([raw_mpdu[0], raw_mpdu[1]]);
    let type_val = ((fc >> 2) & 0x03) as u8;
    let subtype = ((fc >> 4) & 0x0F) as u8;
    let to_ds = (fc & 0x0100) != 0;
    let from_ds = (fc & 0x0200) != 0;
    let is_protected = (fc & 0x4000) != 0;

    let frame_type = match type_val {
        0 => FrameType::Mgmt,
        1 => FrameType::Control,
        2 => FrameType::Data,
        _ => FrameType::Extension,
    };

    let mut addr1 = [0u8; 6];
    let mut addr2 = [0u8; 6];
    let mut addr3 = [0u8; 6];
    addr1.copy_from_slice(&raw_mpdu[4..10]);
    addr2.copy_from_slice(&raw_mpdu[10..16]);
    addr3.copy_from_slice(&raw_mpdu[16..22]);
    let seq_ctrl = u16::from_le_bytes([raw_mpdu[22], raw_mpdu[23]]);

    let mut hdr_len = 24;
    if to_ds && from_ds {
        // 4-address frame
        hdr_len = 30;
        if raw_mpdu.len() < hdr_len {
            return Err(FrameError::MalformedHeader);
        }
    }

    if frame_type == FrameType::Data && (subtype & 0x08) != 0 {
        // QoS Data frame has 2-byte QoS Control
        hdr_len += 2;
        if (fc & 0x8000) != 0 {
            // HT Control present
            hdr_len += 4;
        }
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
    if mpdu.len() < 36 {
        return Err(FrameError::MalformedHeader);
    }
    let fc = u16::from_le_bytes([mpdu[0], mpdu[1]]);
    let type_val = (fc >> 2) & 0x03;
    let subtype = (fc >> 4) & 0x0F;
    if type_val != 0 || (subtype != 8 && subtype != 5) {
        return Err(FrameError::UnsupportedSubtype);
    }

    let mut bssid = [0u8; 6];
    bssid.copy_from_slice(&mpdu[16..22]);

    let capabilities = u16::from_le_bytes([mpdu[34], mpdu[35]]);
    let privacy = (capabilities & 0x0010) != 0;

    let mut ssid: &[u8] = &[];
    let mut channel: u8 = 1;
    let mut rsn_ie: Option<&[u8]> = None;
    let mut wpa_ie: Option<&[u8]> = None;

    let mut pos = 36;
    while pos + 2 <= mpdu.len() {
        let tag = mpdu[pos];
        let len = mpdu[pos + 1] as usize;
        let total = 2 + len;
        if pos + total > mpdu.len() {
            break;
        }
        let elem = &mpdu[pos..pos + total];
        match tag {
            0 => {
                // SSID IE
                ssid = &elem[2..];
            }
            3 => {
                // DS Parameter Set
                if len >= 1 {
                    channel = elem[2];
                }
            }
            48 => {
                // RSN IE
                rsn_ie = Some(elem);
            }
            61 => {
                // HT Operation
                if len >= 1 && channel == 1 {
                    channel = elem[2];
                }
            }
            // Vendor Specific (WPA IE check: 00:50:F2:01)
            221 if len >= 4 && elem[2..6] == [0x00, 0x50, 0xF2, 0x01] => {
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
        }
    } else if let Some(wie) = wpa_ie {
        if let Ok(data) = parse_wpa_or_rsn_ie(wie) {
            has_ccmp = (data.pairwise_cipher & 0x08) != 0;
            has_psk = (data.key_mgmt & 0x02) != 0;
        }
    }

    Ok(BeaconInfo {
        bssid,
        ssid,
        channel,
        privacy,
        rssi,
        rsn_ie,
        wpa_ie,
        has_ccmp,
        has_psk,
    })
}

/// Parses an Auth Response body returning `(sequence_number, status_code)`.
pub fn parse_auth_resp(mpdu: &[u8]) -> Result<(u16, u16), FrameError> {
    if mpdu.len() < 30 {
        return Err(FrameError::MalformedHeader);
    }
    let fc = u16::from_le_bytes([mpdu[0], mpdu[1]]);
    if ((fc >> 2) & 0x03) != 0 || ((fc >> 4) & 0x0F) != 0x0B {
        return Err(FrameError::UnsupportedSubtype);
    }
    let auth_seq = u16::from_le_bytes([mpdu[26], mpdu[27]]);
    let status_code = u16::from_le_bytes([mpdu[28], mpdu[29]]);
    Ok((auth_seq, status_code))
}

/// Parses an Assoc Response body returning `(status_code, association_id)`.
pub fn parse_assoc_resp(mpdu: &[u8]) -> Result<(u16, u16), FrameError> {
    if mpdu.len() < 30 {
        return Err(FrameError::MalformedHeader);
    }
    let fc = u16::from_le_bytes([mpdu[0], mpdu[1]]);
    if ((fc >> 2) & 0x03) != 0 || ((fc >> 4) & 0x0F) != 0x01 {
        return Err(FrameError::UnsupportedSubtype);
    }
    let status_code = u16::from_le_bytes([mpdu[26], mpdu[27]]);
    let aid = u16::from_le_bytes([mpdu[28], mpdu[29]]) & 0x3FFF;
    Ok((status_code, aid))
}

/// Parses a Deauth or Disassoc frame returning `(subtype, reason_code)`.
pub fn parse_deauth_or_disassoc(mpdu: &[u8]) -> Result<(u8, u16), FrameError> {
    if mpdu.len() < 26 {
        return Err(FrameError::MalformedHeader);
    }
    let fc = u16::from_le_bytes([mpdu[0], mpdu[1]]);
    if ((fc >> 2) & 0x03) != 0 {
        return Err(FrameError::UnsupportedSubtype);
    }
    let subtype = ((fc >> 4) & 0x0F) as u8;
    if subtype != 0x0A && subtype != 0x0C {
        return Err(FrameError::UnsupportedSubtype);
    }
    let reason_code = u16::from_le_bytes([mpdu[24], mpdu[25]]);
    Ok((subtype, reason_code))
}

/// Parses an ADDBA Request action frame.
///
/// Returns `(dialog_token, ba_param_set, ba_timeout, start_seq)`.
pub fn parse_addba_req(mpdu: &[u8]) -> Result<(u8, u16, u16, u16), FrameError> {
    if mpdu.len() < 33 {
        return Err(FrameError::MalformedHeader);
    }
    let fc = u16::from_le_bytes([mpdu[0], mpdu[1]]);
    if ((fc >> 2) & 0x03) != 0 || ((fc >> 4) & 0x0F) != 0x0D {
        return Err(FrameError::UnsupportedSubtype);
    }
    // Block ACK Category = 3, Action = 0 (ADDBA Request)
    if mpdu[24] != 3 || mpdu[25] != 0 {
        return Err(FrameError::UnsupportedSubtype);
    }
    let dialog_token = mpdu[26];
    let ba_param_set = u16::from_le_bytes([mpdu[27], mpdu[28]]);
    let ba_timeout = u16::from_le_bytes([mpdu[29], mpdu[30]]);
    let start_seq = u16::from_le_bytes([mpdu[31], mpdu[32]]) >> 4;
    Ok((dialog_token, ba_param_set, ba_timeout, start_seq))
}

/// Builds an 802.11 Probe Request management frame.
pub fn build_probe_req(
    buf: &mut [u8],
    sta_mac: [u8; 6],
    ssid: Option<&[u8]>,
    seq: u16,
) -> Result<usize, FrameError> {
    let ssid_bytes = ssid.unwrap_or(&[]);
    let total_len = 24 + 2 + ssid_bytes.len() + 2 + 8;
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
    let rates_ie_len = 10;
    let total_len = 24 + 4 + (2 + ssid.len()) + rates_ie_len + rsn_ie.len();
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
        // Plaintext payload contains LLC/SNAP header (8 bytes) + eth_payload
        let mut plain = [0u8; 1536];
        let plain_len = 8 + eth_payload.len();
        if plain_len > plain.len() {
            return Err(FrameError::BufferTooSmall);
        }
        // LLC/SNAP: AA AA 03 00 00 00 <ethertype>
        plain[0..6].copy_from_slice(&[0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00]);
        plain[6..8].copy_from_slice(ethertype);
        plain[8..plain_len].copy_from_slice(eth_payload);

        ccmp_encrypt_mpdu(tk, key_id, pn, &mac_header, &plain[..plain_len], out_mpdu)
            .map_err(|_| FrameError::CryptoError)
    } else {
        let total_len = 24 + 8 + eth_payload.len();
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
pub fn rx_80211_data_to_eth(
    raw_mpdu: &[u8],
    scratch_plain: &mut [u8],
    out_eth: &mut [u8],
    ptk: Option<&[u8; 16]>,
    gtk: Option<(u8, &[u8; 16])>,
    expected_bssid: &[u8; 6],
) -> Result<RxDataOutcome, FrameError> {
    if raw_mpdu.len() < 24 {
        return Err(FrameError::MalformedHeader);
    }
    let fc = u16::from_le_bytes([raw_mpdu[0], raw_mpdu[1]]);
    let type_val = (fc >> 2) & 0x03;
    let subtype = ((fc >> 4) & 0x0F) as u8;
    if type_val != 2 {
        return Err(FrameError::UnsupportedSubtype);
    }

    let is_protected = (fc & 0x4000) != 0;
    let is_qos = (subtype & 0x08) != 0;
    let is_group = (raw_mpdu[4] & 0x01) != 0;

    // Verify Addr2 matches expected BSSID for AP-to-STA (FromDS=1) frames
    if raw_mpdu[10..16] != *expected_bssid {
        return Err(FrameError::MalformedHeader);
    }

    let da = &raw_mpdu[4..10]; // Addr1: DA
    let sa = &raw_mpdu[16..22]; // Addr3: SA

    let plain_payload: &[u8] = if is_protected {
        let key = if is_group {
            gtk.map(|(_, k)| k).ok_or(FrameError::MissingKey)?
        } else {
            ptk.ok_or(FrameError::MissingKey)?
        };

        let (pt_len, _key_id, _pn) =
            ccmp_decrypt_mpdu(key, raw_mpdu, scratch_plain).map_err(|_| FrameError::CryptoError)?;
        &scratch_plain[..pt_len]
    } else {
        // If encryption keys are active, reject unencrypted frames unless pre-key EAPOL
        let keys_active = ptk.is_some() || gtk.is_some();
        let hdr_len = if is_qos { 26 } else { 24 };
        if raw_mpdu.len() < hdr_len {
            return Err(FrameError::MalformedHeader);
        }
        let payload = &raw_mpdu[hdr_len..];

        if keys_active {
            // Check if this is an unencrypted EAPOL frame (EtherType 0x888E)
            let is_eapol = payload.len() >= 8
                && payload[0..6] == [0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00]
                && payload[6..8] == [0x88, 0x8E];
            if !is_eapol {
                return Err(FrameError::UnencryptedPayloadRejected);
            }
        }
        payload
    };

    // Strip optional HT Control if present
    let mut clean_payload = plain_payload;
    if (fc & 0x8000) != 0
        && is_qos
        && clean_payload.len() >= 4
        && clean_payload[0..3] != [0xAA, 0xAA, 0x03]
    {
        clean_payload = &clean_payload[4..];
    }

    if clean_payload.len() < 8 {
        return Err(FrameError::InvalidPayload);
    }

    // Verify LLC/SNAP header: AA AA 03 00 00 00
    if clean_payload[0..6] != [0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00] {
        return Err(FrameError::InvalidPayload);
    }

    let ethertype = [clean_payload[6], clean_payload[7]];
    let body = &clean_payload[8..];

    // EtherType 0x888E demux: EAPOL
    if ethertype == [0x88, 0x8E] {
        if out_eth.len() < body.len() {
            return Err(FrameError::BufferTooSmall);
        }
        out_eth[..body.len()].copy_from_slice(body);
        Ok(RxDataOutcome::Eapol(body.len()))
    } else {
        // Standard Ethernet-II frame: DA (6) + SA (6) + EtherType (2) + Body
        let total_eth_len = 14 + body.len();
        if out_eth.len() < total_eth_len {
            return Err(FrameError::BufferTooSmall);
        }
        out_eth[0..6].copy_from_slice(da);
        out_eth[6..12].copy_from_slice(sa);
        out_eth[12..14].copy_from_slice(&ethertype);
        out_eth[14..total_eth_len].copy_from_slice(body);
        Ok(RxDataOutcome::Ethernet(total_eth_len))
    }
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

        let outcome = rx_80211_data_to_eth(
            &from_ds_mpdu[..from_ds_len],
            &mut scratch,
            &mut eth_rx,
            Some(&tk),
            None,
            &bssid,
        )
        .expect("rx_80211_data_to_eth failed on valid FromDS frame");

        match outcome {
            RxDataOutcome::Ethernet(rx_len) => {
                assert_eq!(rx_len, 24);
                assert_eq!(&eth_rx[..rx_len], &eth_tx[..24]);
            }
            RxDataOutcome::Eapol(_) => panic!("expected Ethernet outcome"),
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
        );
        assert_eq!(err, Err(FrameError::CryptoError));

        // 4. EAPOL frame test
        let mut eapol_tx = [0u8; 40];
        eapol_tx[0..6].copy_from_slice(&bssid);
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
        eapol_mpdu[4..10].copy_from_slice(&eapol_tx[0..6]);
        eapol_mpdu[10..16].copy_from_slice(&bssid);
        eapol_mpdu[16..22].copy_from_slice(&eapol_tx[6..12]);

        let eapol_outcome = rx_80211_data_to_eth(
            &eapol_mpdu[..eapol_len],
            &mut scratch,
            &mut eth_rx,
            None,
            None,
            &bssid,
        )
        .expect("valid rx eapol");

        match eapol_outcome {
            RxDataOutcome::Eapol(e_len) => {
                assert_eq!(e_len, 10);
                assert_eq!(&eth_rx[..e_len], b"EAPOL_TEST");
            }
            RxDataOutcome::Ethernet(_) => panic!("expected Eapol outcome"),
        }
    }
}
