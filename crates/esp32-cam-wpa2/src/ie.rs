//! 802.11 WPA / RSN Information Element and KDE parsing/generation.
//!
//! Enforces ESP-IDF supplicant ABI constants, zero heap allocations,
//! and `#![no_std]`.

pub const WPA_PROTO_WPA: i32 = 1;
pub const WPA_PROTO_RSN: i32 = 2;
pub const WPA_PROTO_WAPI: i32 = 4;

pub const WPA_CIPHER_NONE: u32 = 0x01;
pub const WPA_CIPHER_TKIP: u32 = 0x02;
pub const WPA_CIPHER_CCMP: u32 = 0x08;
pub const WPA_CIPHER_AES_128_CMAC: u32 = 0x20;
pub const WPA_CIPHER_WEP40: u32 = 0x80;
pub const WPA_CIPHER_WEP104: u32 = 0x100;
pub const WPA_CIPHER_SMS4: u32 = 0x400;
pub const WPA_CIPHER_GCMP: u32 = 0x800;
pub const WPA_CIPHER_GCMP_256: u32 = 0x1000;
pub const WPA_CIPHER_AES_GMAC_128: u32 = 0x2000;
pub const WPA_CIPHER_AES_GMAC_256: u32 = 0x4000;

pub const WIFI_CIPHER_TYPE_NONE: i32 = 0;
pub const WIFI_CIPHER_TYPE_WEP40: i32 = 1;
pub const WIFI_CIPHER_TYPE_WEP104: i32 = 2;
pub const WIFI_CIPHER_TYPE_TKIP: i32 = 3;
pub const WIFI_CIPHER_TYPE_CCMP: i32 = 4;
pub const WIFI_CIPHER_TYPE_TKIP_CCMP: i32 = 5;
pub const WIFI_CIPHER_TYPE_AES_CMAC128: i32 = 6;
pub const WIFI_CIPHER_TYPE_SMS4: i32 = 7;
pub const WIFI_CIPHER_TYPE_GCMP: i32 = 8;
pub const WIFI_CIPHER_TYPE_GCMP256: i32 = 9;
pub const WIFI_CIPHER_TYPE_AES_GMAC128: i32 = 10;
pub const WIFI_CIPHER_TYPE_AES_GMAC256: i32 = 11;
pub const WIFI_CIPHER_TYPE_UNKNOWN: i32 = 12;

pub const WPA_KEY_MGMT_IEEE8021X: u32 = 1;
pub const WPA_KEY_MGMT_PSK: u32 = 2;
pub const WPA_KEY_MGMT_NONE: u32 = 0x10;
pub const WPA_KEY_MGMT_FT_PSK: u32 = 0x40;
pub const WPA_KEY_MGMT_IEEE8021X_SHA256: u32 = 0x80;
pub const WPA_KEY_MGMT_PSK_SHA256: u32 = 0x100;

const OUI_RSN: [u8; 3] = [0x00, 0x0F, 0xAC];
const OUI_WPA: [u8; 3] = [0x00, 0x50, 0xF2];

const RSN_CIPHERS: [(u32, u8); 10] = [
    (WPA_CIPHER_NONE, 0),
    (WPA_CIPHER_WEP40, 1),
    (WPA_CIPHER_TKIP, 2),
    (WPA_CIPHER_CCMP, 4),
    (WPA_CIPHER_WEP104, 5),
    (WPA_CIPHER_AES_128_CMAC, 6),
    (WPA_CIPHER_GCMP, 8),
    (WPA_CIPHER_GCMP_256, 9),
    (WPA_CIPHER_AES_GMAC_128, 11),
    (WPA_CIPHER_AES_GMAC_256, 12),
];

const RSN_AKMS: [(u32, u8); 5] = [
    (WPA_KEY_MGMT_IEEE8021X, 1),
    (WPA_KEY_MGMT_PSK, 2),
    (WPA_KEY_MGMT_FT_PSK, 4),
    (WPA_KEY_MGMT_IEEE8021X_SHA256, 5),
    (WPA_KEY_MGMT_PSK_SHA256, 6),
];

const WPA_CIPHERS: [(u32, u8); 5] = [
    (WPA_CIPHER_NONE, 0),
    (WPA_CIPHER_WEP40, 1),
    (WPA_CIPHER_TKIP, 2),
    (WPA_CIPHER_CCMP, 4),
    (WPA_CIPHER_WEP104, 5),
];

const WPA_AKMS: [(u32, u8); 3] = [
    (WPA_KEY_MGMT_NONE, 0),
    (WPA_KEY_MGMT_IEEE8021X, 1),
    (WPA_KEY_MGMT_PSK, 2),
];

fn lookup_suite(tbl: &[(u32, u8)], val: u32) -> Option<u8> {
    for &(c, s) in tbl {
        if c == val {
            return Some(s);
        }
    }
    None
}

fn suite_to_val(expected_oui: &[u8; 3], tbl: &[(u32, u8)], suite: &[u8; 4]) -> u32 {
    if &suite[0..3] != expected_oui {
        return 0;
    }
    for &(c, s) in tbl {
        if s == suite[3] {
            return c;
        }
    }
    0
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct WpaIeData {
    pub proto: i32,
    pub pairwise_cipher: u32,
    pub group_cipher: u32,
    pub key_mgmt: u32,
    pub capabilities: u16,
    pub mgmt_group_cipher: u32,
    pub has_pmkid: bool,
    pub first_pmkid_offset: usize,
    pub rsnxe_capa: u8,
}

pub fn cipher_type_map_supp_to_public(supp: u32) -> i32 {
    match supp {
        WPA_CIPHER_NONE => WIFI_CIPHER_TYPE_NONE,
        WPA_CIPHER_WEP40 => WIFI_CIPHER_TYPE_WEP40,
        WPA_CIPHER_WEP104 => WIFI_CIPHER_TYPE_WEP104,
        WPA_CIPHER_TKIP => WIFI_CIPHER_TYPE_TKIP,
        WPA_CIPHER_CCMP => WIFI_CIPHER_TYPE_CCMP,
        0x0A => WIFI_CIPHER_TYPE_TKIP_CCMP,
        WPA_CIPHER_AES_128_CMAC => WIFI_CIPHER_TYPE_AES_CMAC128,
        WPA_CIPHER_SMS4 => WIFI_CIPHER_TYPE_SMS4,
        WPA_CIPHER_GCMP => WIFI_CIPHER_TYPE_GCMP,
        WPA_CIPHER_GCMP_256 => WIFI_CIPHER_TYPE_GCMP256,
        WPA_CIPHER_AES_GMAC_128 => WIFI_CIPHER_TYPE_AES_GMAC128,
        WPA_CIPHER_AES_GMAC_256 => WIFI_CIPHER_TYPE_AES_GMAC256,
        _ => WIFI_CIPHER_TYPE_UNKNOWN,
    }
}

pub fn cipher_type_map_public_to_supp(public: i32) -> u32 {
    match public {
        WIFI_CIPHER_TYPE_NONE => WPA_CIPHER_NONE,
        WIFI_CIPHER_TYPE_WEP40 => WPA_CIPHER_WEP40,
        WIFI_CIPHER_TYPE_WEP104 => WPA_CIPHER_WEP104,
        WIFI_CIPHER_TYPE_TKIP => WPA_CIPHER_TKIP,
        WIFI_CIPHER_TYPE_CCMP => WPA_CIPHER_CCMP,
        WIFI_CIPHER_TYPE_TKIP_CCMP => WPA_CIPHER_TKIP | WPA_CIPHER_CCMP,
        WIFI_CIPHER_TYPE_AES_CMAC128 => WPA_CIPHER_AES_128_CMAC,
        WIFI_CIPHER_TYPE_SMS4 => WPA_CIPHER_SMS4,
        WIFI_CIPHER_TYPE_GCMP => WPA_CIPHER_GCMP,
        WIFI_CIPHER_TYPE_GCMP256 => WPA_CIPHER_GCMP_256,
        WIFI_CIPHER_TYPE_AES_GMAC128 => WPA_CIPHER_AES_GMAC_128,
        WIFI_CIPHER_TYPE_AES_GMAC256 => WPA_CIPHER_AES_GMAC_256,
        _ => WPA_CIPHER_NONE,
    }
}

/// Information Element parsing or generation error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct IeError;

fn parse_suites(
    body: &[u8],
    pos: &mut usize,
    oui: &[u8; 3],
    tbl: &[(u32, u8)],
) -> Result<u32, IeError> {
    if *pos == body.len() {
        return Ok(0);
    }
    if *pos + 2 > body.len() {
        return Err(IeError);
    }
    let count = u16::from_le_bytes([body[*pos], body[*pos + 1]]) as usize;
    *pos += 2;
    let bytes = count.checked_mul(4).ok_or(IeError)?;
    let end = pos.checked_add(bytes).ok_or(IeError)?;
    if end > body.len() {
        return Err(IeError);
    }
    let mut mask = 0u32;
    for _ in 0..count {
        let mut suite = [0u8; 4];
        suite.copy_from_slice(&body[*pos..*pos + 4]);
        mask |= suite_to_val(oui, tbl, &suite);
        *pos += 4;
    }
    Ok(mask)
}

fn parse_ie_body(
    body: &[u8],
    proto: i32,
    oui: &[u8; 3],
    c_tbl: &[(u32, u8)],
    a_tbl: &[(u32, u8)],
) -> Result<WpaIeData, IeError> {
    if body.len() < 2 || u16::from_le_bytes([body[0], body[1]]) != 1 {
        return Err(IeError);
    }
    let mut data = WpaIeData {
        proto,
        ..Default::default()
    };
    if body.len() == 2 {
        return Ok(data);
    }
    if body.len() < 6 {
        return Err(IeError);
    }
    let mut g_suite = [0u8; 4];
    g_suite.copy_from_slice(&body[2..6]);
    data.group_cipher = suite_to_val(oui, c_tbl, &g_suite);

    let mut pos = 6;
    data.pairwise_cipher = parse_suites(body, &mut pos, oui, c_tbl)?;
    data.key_mgmt = parse_suites(body, &mut pos, oui, a_tbl)?;

    if pos < body.len() {
        if pos + 2 > body.len() {
            return Err(IeError);
        }
        data.capabilities = u16::from_le_bytes([body[pos], body[pos + 1]]);
        pos += 2;
    }
    if proto == WPA_PROTO_RSN {
        if pos < body.len() {
            if pos + 2 > body.len() {
                return Err(IeError);
            }
            let pmkid_count = u16::from_le_bytes([body[pos], body[pos + 1]]) as usize;
            pos += 2;
            let pmkid_bytes = pmkid_count.checked_mul(16).ok_or(IeError)?;
            let end = pos.checked_add(pmkid_bytes).ok_or(IeError)?;
            if end > body.len() {
                return Err(IeError);
            }
            if pmkid_count > 0 {
                data.has_pmkid = true;
                data.first_pmkid_offset = 2 + pos;
                pos = end;
            }
        }
        if pos < body.len() {
            if pos + 4 > body.len() {
                return Err(IeError);
            }
            let mut mgmt_suite = [0u8; 4];
            mgmt_suite.copy_from_slice(&body[pos..pos + 4]);
            data.mgmt_group_cipher = suite_to_val(oui, c_tbl, &mgmt_suite);
            pos += 4;
        }
    }
    if pos != body.len() {
        return Err(IeError);
    }
    Ok(data)
}

/// Parses a standalone IEEE 802.11 RSN Information Element (Tag 0x30).
///
/// Rejects truncated buffers and trailing unparsed bytes
/// (`ie.len() != 2 + len`).
pub fn parse_rsn_ie(ie: &[u8]) -> Result<WpaIeData, IeError> {
    if ie.len() < 4 || ie[0] != 0x30 {
        return Err(IeError);
    }
    let len = ie[1] as usize;
    if ie.len() != 2 + len || len < 2 {
        return Err(IeError);
    }
    parse_ie_body(
        &ie[2..2 + len],
        WPA_PROTO_RSN,
        &OUI_RSN,
        &RSN_CIPHERS,
        &RSN_AKMS,
    )
}

/// Parses a standalone WPA1 Vendor Specific Information Element (Tag 0xDD).
///
/// Rejects truncated buffers and trailing unparsed bytes
/// (`ie.len() != 2 + len`).
pub fn parse_wpa_ie(ie: &[u8]) -> Result<WpaIeData, IeError> {
    if ie.len() < 8 || ie[0] != 0xDD {
        return Err(IeError);
    }
    let len = ie[1] as usize;
    if ie.len() != 2 + len || len < 6 || ie[2..6] != [0x00, 0x50, 0xF2, 0x01] {
        return Err(IeError);
    }
    parse_ie_body(
        &ie[6..2 + len],
        WPA_PROTO_WPA,
        &OUI_WPA,
        &WPA_CIPHERS,
        &WPA_AKMS,
    )
}

pub fn parse_wpa_or_rsn_ie(ie: &[u8]) -> Result<WpaIeData, IeError> {
    if ie.is_empty() {
        return Err(IeError);
    }
    match ie[0] {
        0x30 => parse_rsn_ie(ie),
        0xDD => parse_wpa_ie(ie),
        _ => Err(IeError),
    }
}

pub fn gen_wpa_ie(
    proto: i32,
    pairwise: u32,
    group: u32,
    key_mgmt: u32,
    rsn_caps: u16,
    mgmt_group: u32,
    out: &mut [u8],
) -> Result<usize, IeError> {
    if proto == WPA_PROTO_RSN {
        let gb = lookup_suite(&RSN_CIPHERS, group).ok_or(IeError)?;
        let pb = lookup_suite(&RSN_CIPHERS, pairwise).ok_or(IeError)?;
        let ab = lookup_suite(&RSN_AKMS, key_mgmt).ok_or(IeError)?;
        let mb = if mgmt_group != 0 {
            Some(lookup_suite(&RSN_CIPHERS, mgmt_group).ok_or(IeError)?)
        } else {
            None
        };
        let total = if mb.is_some() { 28 } else { 22 };
        if out.len() < total {
            return Err(IeError);
        }
        out[..4].copy_from_slice(&[0x30, (total - 2) as u8, 0x01, 0x00]);
        out[4..8].copy_from_slice(&[OUI_RSN[0], OUI_RSN[1], OUI_RSN[2], gb]);
        out[8..10].copy_from_slice(&[0x01, 0x00]);
        out[10..14].copy_from_slice(&[OUI_RSN[0], OUI_RSN[1], OUI_RSN[2], pb]);
        out[14..16].copy_from_slice(&[0x01, 0x00]);
        out[16..20].copy_from_slice(&[OUI_RSN[0], OUI_RSN[1], OUI_RSN[2], ab]);
        out[20..22].copy_from_slice(&rsn_caps.to_le_bytes());
        if let Some(m) = mb {
            out[22..24].copy_from_slice(&[0x00, 0x00]);
            out[24..28].copy_from_slice(&[OUI_RSN[0], OUI_RSN[1], OUI_RSN[2], m]);
        }
        Ok(total)
    } else if proto == WPA_PROTO_WPA {
        let gb = lookup_suite(&WPA_CIPHERS, group).ok_or(IeError)?;
        let pb = lookup_suite(&WPA_CIPHERS, pairwise).ok_or(IeError)?;
        let ab = lookup_suite(&WPA_AKMS, key_mgmt).ok_or(IeError)?;
        if out.len() < 24 {
            return Err(IeError);
        }
        out[..6].copy_from_slice(&[0xDD, 22, OUI_WPA[0], OUI_WPA[1], OUI_WPA[2], 0x01]);
        out[6..8].copy_from_slice(&[0x01, 0x00]);
        out[8..12].copy_from_slice(&[OUI_WPA[0], OUI_WPA[1], OUI_WPA[2], gb]);
        out[12..14].copy_from_slice(&[0x01, 0x00]);
        out[14..18].copy_from_slice(&[OUI_WPA[0], OUI_WPA[1], OUI_WPA[2], pb]);
        out[18..20].copy_from_slice(&[0x01, 0x00]);
        out[20..24].copy_from_slice(&[OUI_WPA[0], OUI_WPA[1], OUI_WPA[2], ab]);
        Ok(24)
    } else {
        Err(IeError)
    }
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct ParsedKeyData<'a> {
    pub rsn_ie: Option<&'a [u8]>,
    pub wpa_ie: Option<&'a [u8]>,
    pub gtk_key_id: u8,
    pub gtk_tx: bool,
    pub gtk: Option<&'a [u8]>,
    pub igtk_key_id: u16,
    pub igtk_ipn: [u8; 6],
    pub igtk: Option<&'a [u8]>,
}

/// Parses EAPOL-Key Data IEs and KDEs per IEEE 802.11-2016 Clause 12.7.2.
///
/// # Errors
///
/// Returns [`IeError`] if:
/// - Unexpected `0x00` bytes appear outside trailing `0xDD` padding;
/// - Non-zero trailing bytes follow a `0xDD 0x00` padding terminator;
/// - Any TLV is truncated;
/// - Duplicate RSN IE, WPA IE, GTK KDE, or IGTK KDE elements occur;
/// - A GTK or IGTK KDE carries fewer than 16 or more than 32 key bytes.
pub fn parse_key_data_kdes(key_data: &[u8]) -> Result<ParsedKeyData<'_>, IeError> {
    let mut parsed = ParsedKeyData::default();
    let mut pos = 0;
    while pos < key_data.len() {
        let tag = key_data[pos];
        if tag == 0xDD && pos + 1 == key_data.len() {
            // Single-octet 0xDD padding at end of Key Data (IEEE 802.11 §12.7.2).
            break;
        }
        if pos + 2 > key_data.len() {
            return Err(IeError);
        }
        if tag == 0x00 {
            return Err(IeError);
        }
        let len = key_data[pos + 1] as usize;
        if tag == 0xDD && len == 0 {
            if key_data[pos + 2..].iter().all(|&b| b == 0x00) {
                break;
            }
            return Err(IeError);
        }
        let total = 2 + len;
        if pos + total > key_data.len() {
            return Err(IeError);
        }
        let elem = &key_data[pos..pos + total];
        if tag == 0x30 {
            if parsed.rsn_ie.is_some() {
                return Err(IeError);
            }
            parsed.rsn_ie = Some(elem);
        } else if tag == 0xDD && len >= 4 {
            let oui_type = &key_data[pos + 2..pos + 6];
            if oui_type == [0x00, 0x50, 0xF2, 0x01] {
                if parsed.wpa_ie.is_some() {
                    return Err(IeError);
                }
                parsed.wpa_ie = Some(elem);
            } else if oui_type[0..3] == OUI_RSN {
                let payload = &key_data[pos + 6..pos + total];
                if oui_type[3] == 0x01 {
                    if parsed.gtk.is_some() || payload.len() < 2 + 16 || payload.len() > 2 + 32 {
                        return Err(IeError);
                    }
                    parsed.gtk_key_id = payload[0] & 0x03;
                    parsed.gtk_tx = (payload[0] & 0x04) != 0;
                    parsed.gtk = Some(&payload[2..]);
                } else if oui_type[3] == 0x09 {
                    if parsed.igtk.is_some() || payload.len() < 8 + 16 || payload.len() > 8 + 32 {
                        return Err(IeError);
                    }
                    parsed.igtk_key_id = u16::from_le_bytes([payload[0], payload[1]]);
                    parsed.igtk_ipn.copy_from_slice(&payload[2..8]);
                    parsed.igtk = Some(&payload[8..]);
                }
            }
        }
        pos += total;
    }
    Ok(parsed)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cipher_type_map_roundtrip() {
        assert_eq!(
            cipher_type_map_supp_to_public(WPA_CIPHER_NONE),
            WIFI_CIPHER_TYPE_NONE
        );
        assert_eq!(
            cipher_type_map_supp_to_public(WPA_CIPHER_CCMP),
            WIFI_CIPHER_TYPE_CCMP
        );
        assert_eq!(
            cipher_type_map_supp_to_public(WPA_CIPHER_TKIP),
            WIFI_CIPHER_TYPE_TKIP
        );
        assert_eq!(
            cipher_type_map_supp_to_public(WPA_CIPHER_TKIP | WPA_CIPHER_CCMP),
            WIFI_CIPHER_TYPE_TKIP_CCMP
        );
        assert_eq!(
            cipher_type_map_public_to_supp(WIFI_CIPHER_TYPE_CCMP),
            WPA_CIPHER_CCMP
        );
        assert_eq!(
            cipher_type_map_public_to_supp(WIFI_CIPHER_TYPE_TKIP_CCMP),
            WPA_CIPHER_TKIP | WPA_CIPHER_CCMP
        );
    }

    #[test]
    fn test_rsn_and_wpa_ie_roundtrip_and_trailing_bytes_rejected() {
        let mut buf = [0u8; 32];
        let len = gen_wpa_ie(
            WPA_PROTO_RSN,
            WPA_CIPHER_CCMP,
            WPA_CIPHER_CCMP,
            WPA_KEY_MGMT_PSK,
            0x000c,
            0,
            &mut buf,
        )
        .expect("RSN IE generation should succeed");
        assert_eq!(len, 22);
        let d = parse_rsn_ie(&buf[..len]).expect("RSN IE parse should succeed");
        assert_eq!(
            (d.proto, d.pairwise_cipher, d.group_cipher, d.key_mgmt),
            (
                WPA_PROTO_RSN,
                WPA_CIPHER_CCMP,
                WPA_CIPHER_CCMP,
                WPA_KEY_MGMT_PSK
            )
        );
        // Trailing garbage byte after RSN IE must be rejected (C2-057).
        buf[len] = 0xAA;
        assert_eq!(parse_rsn_ie(&buf[..len + 1]), Err(IeError));

        let wlen = gen_wpa_ie(
            WPA_PROTO_WPA,
            WPA_CIPHER_TKIP,
            WPA_CIPHER_TKIP,
            WPA_KEY_MGMT_PSK,
            0,
            0,
            &mut buf,
        )
        .expect("WPA IE generation should succeed");
        assert_eq!(wlen, 24);
        let wd = parse_wpa_ie(&buf[..wlen]).expect("WPA IE parse should succeed");
        assert_eq!(
            (wd.proto, wd.pairwise_cipher, wd.group_cipher, wd.key_mgmt),
            (
                WPA_PROTO_WPA,
                WPA_CIPHER_TKIP,
                WPA_CIPHER_TKIP,
                WPA_KEY_MGMT_PSK
            )
        );
        // Trailing garbage byte after WPA IE must be rejected (C2-057).
        buf[wlen] = 0xBB;
        assert_eq!(parse_wpa_ie(&buf[..wlen + 1]), Err(IeError));
    }

    #[test]
    fn test_kde_gtk_extraction_and_padding() {
        let mut key_data = [0u8; 32];
        key_data[0] = 0xDD;
        key_data[1] = 22;
        key_data[2..5].copy_from_slice(&OUI_RSN);
        key_data[5] = 0x01; // GTK
        key_data[6] = 0x05; // ID 1, Tx
        key_data[7] = 0x00;
        key_data[8..24].fill(0xAA);
        // Valid 0xDD 0x00 ... 0x00 padding to 32 bytes.
        key_data[24] = 0xDD;

        let parsed = parse_key_data_kdes(&key_data).expect("valid padded KDEs");
        assert_eq!(parsed.gtk_key_id, 1);
        assert!(parsed.gtk_tx);
        assert_eq!(parsed.gtk, Some(&key_data[8..24]));
        assert!(parse_rsn_ie(&[0x30, 0x10, 0x01, 0x00]).is_err());
        assert!(parse_wpa_or_rsn_ie(&[]).is_err());

        // Single-byte 0xDD padding terminator at end is also valid.
        let single_pad = parse_key_data_kdes(&key_data[..25]).expect("single 0xDD pad");
        assert_eq!(single_pad.gtk, Some(&key_data[8..24]));
    }

    #[test]
    fn test_kde_malformed_padding_truncation_duplicates_and_zero_len_rejected() {
        // Non-zero byte smuggled after 0xDD 0x00 padding terminator.
        let mut bad_pad = [0u8; 32];
        bad_pad[0] = 0xDD;
        bad_pad[1] = 22;
        bad_pad[2..5].copy_from_slice(&OUI_RSN);
        bad_pad[5] = 0x01;
        bad_pad[8..24].fill(0x11);
        bad_pad[24] = 0xDD;
        bad_pad[25] = 0x00;
        bad_pad[26] = 0x30;
        assert_eq!(parse_key_data_kdes(&bad_pad), Err(IeError));

        // Unexpected leading 0x00 byte.
        assert_eq!(parse_key_data_kdes(&[0x00, 0xDD, 0x00]), Err(IeError));

        // Truncated TLV.
        assert_eq!(parse_key_data_kdes(&[0x30, 0x10, 0x01, 0x00]), Err(IeError));
        // Single trailing non-0xDD byte.
        assert_eq!(parse_key_data_kdes(&[0x30]), Err(IeError));

        // Zero-length GTK KDE (payload len == 2 -> 0 key bytes).
        let zero_gtk = [0xDD, 6, 0x00, 0x0F, 0xAC, 0x01, 0x01, 0x00];
        assert_eq!(parse_key_data_kdes(&zero_gtk), Err(IeError));

        // Zero-length IGTK KDE (payload len == 8 -> 0 key bytes).
        let zero_igtk = [
            0xDD, 12, 0x00, 0x0F, 0xAC, 0x09, 0x04, 0x00, 0, 0, 0, 0, 0, 0,
        ];
        assert_eq!(parse_key_data_kdes(&zero_igtk), Err(IeError));

        // Duplicate GTK KDE must be rejected (no last-wins overwrite).
        let mut dup_gtk = [0u8; 48];
        for off in [0usize, 24usize] {
            dup_gtk[off] = 0xDD;
            dup_gtk[off + 1] = 22;
            dup_gtk[off + 2..off + 5].copy_from_slice(&OUI_RSN);
            dup_gtk[off + 5] = 0x01;
            dup_gtk[off + 8..off + 24].fill(0x22);
        }
        assert_eq!(parse_key_data_kdes(&dup_gtk), Err(IeError));

        // Duplicate RSN IE must be rejected.
        let dup_rsn = [0x30, 2, 0x01, 0x00, 0x30, 2, 0x01, 0x00];
        assert_eq!(parse_key_data_kdes(&dup_rsn), Err(IeError));
    }
}
