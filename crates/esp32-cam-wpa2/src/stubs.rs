//! STA C stubs and hex decoding routines for WPA supplicant FFI.

const fn hex_val(c: u8) -> Option<u8> {
    match c {
        b'0'..=b'9' => Some(c - b'0'),
        b'a'..=b'f' => Some(c - b'a' + 10),
        b'A'..=b'F' => Some(c - b'A' + 10),
        _ => None,
    }
}

/// Decodes an ASCII hexadecimal string into a raw byte buffer.
///
/// # Safety
///
/// - `hex` must point to at least `len * 2` readable ASCII bytes or be null.
/// - `buf` must point to at least `len` writable bytes or be null.
// JUSTIFICATION: Decodes ASCII hex string to byte buffer for WPA FFI.
// SAFETY: Validates non-null pointers and checks valid hex characters.
#[no_mangle]
pub unsafe extern "C" fn hexstr2bin(
    hex: *const core::ffi::c_char,
    buf: *mut u8,
    len: usize,
) -> i32 {
    // JUSTIFICATION: Raw pointer dereferencing for hex decoding.
    // SAFETY: Validates non-null pointers and validates all hex digits.
    unsafe {
        if hex.is_null() || buf.is_null() {
            return -1;
        }
        let hex_slice = core::slice::from_raw_parts(hex.cast::<u8>(), len * 2);
        for i in 0..len {
            let h = match hex_val(hex_slice[i * 2]) {
                Some(v) => v,
                None => return -1,
            };
            let l = match hex_val(hex_slice[i * 2 + 1]) {
                Some(v) => v,
                None => return -1,
            };
            *buf.add(i) = (h << 4) | l;
        }
        0
    }
}

// JUSTIFICATION: STA-only build stub for hostap initialization.
// SAFETY: Returns null; performs no memory operations.
pub(crate) unsafe extern "C" fn hostap_init() -> *mut core::ffi::c_void {
    core::ptr::null_mut()
}
// JUSTIFICATION: STA-only build stub for hostap deinitialization.
// SAFETY: Safely ignores pointer argument; returns true.
pub(crate) unsafe extern "C" fn hostap_deinit(_data: *mut core::ffi::c_void) -> bool {
    true
}
// JUSTIFICATION: STA-only build stub rejecting AP station associations.
// SAFETY: Safely ignores all parameters; returns false.
pub(crate) unsafe extern "C" fn hostap_sta_join(
    _sm: *mut *mut core::ffi::c_void,
    _bssid: *mut u8,
    _wpa_ie: *mut u8,
    _wpa_ie_len: u8,
    _rsnxe: *mut u8,
    _rsnxe_len: u16,
    _pmf_enable: *mut bool,
    _subtype: i32,
    _pairwise_cipher: *mut u8,
) -> bool {
    false
}
// JUSTIFICATION: STA-only build stub for AP peer removal.
// SAFETY: Safely ignores bssid pointer; returns false.
pub(crate) unsafe extern "C" fn wpa_ap_remove(_bssid: *mut u8) -> bool {
    false
}
// JUSTIFICATION: STA-only build stub for AP WPA IE retrieval.
// SAFETY: Safely ignores len pointer; returns null.
pub(crate) unsafe extern "C" fn wpa_ap_get_wpa_ie(_len: *mut usize) -> *mut u8 {
    core::ptr::null_mut()
}
// JUSTIFICATION: STA-only build stub rejecting incoming AP EAPOL packets.
// SAFETY: Safely ignores packet pointers; returns false.
pub(crate) unsafe extern "C" fn wpa_ap_rx_eapol(
    _hapd: *mut core::ffi::c_void,
    _sm: *mut core::ffi::c_void,
    _data: *mut u8,
    _len: usize,
) -> bool {
    false
}
// JUSTIFICATION: STA-only build stub for peer SPP message inquiry.
// SAFETY: No-op; safely ignores pointers.
pub(crate) unsafe extern "C" fn wpa_ap_get_peer_spp_msg(
    _sm: *mut core::ffi::c_void,
    _cap: *mut bool,
    _req: *mut bool,
) {
}
// JUSTIFICATION: STA-only build stub for config string parsing.
// SAFETY: Safely ignores string pointers; returns null.
pub(crate) unsafe extern "C" fn wpa_config_parse_string(
    _val: *const core::ffi::c_char,
    _len: *mut usize,
) -> *mut core::ffi::c_char {
    core::ptr::null_mut()
}
// JUSTIFICATION: STA-only build stub reporting Michael MIC failure.
// SAFETY: Safely ignores is_unicast parameter; returns 0.
pub(crate) unsafe extern "C" fn wpa_michael_mic_failure(_is_unicast: u16) -> i32 {
    0
}
// JUSTIFICATION: STA-only build stub for configuration completion.
// SAFETY: No-op callback; mutates no state.
pub(crate) unsafe extern "C" fn wpa_config_done() {}
// JUSTIFICATION: STA-only build stub for configuration reload.
// SAFETY: No-op callback; mutates no state.
pub(crate) unsafe extern "C" fn wpa_config_reload() {}
// JUSTIFICATION: STA-only build stub clearing PMKSA cache.
// SAFETY: No-op callback; mutates no state.
pub(crate) unsafe extern "C" fn wpa_sta_clear_curr_pmksa() {}
