//! C ABI bridge and driver integration for ESP32 WPA/WPA2 supplicant.
//!
//! Connects the pure-Rust 4-way handshake state machine to the low-level
//! ESP32 WiFi MAC driver routines.

#![allow(clippy::missing_safety_doc)]

use core::cell::UnsafeCell;
use core::ffi::{c_char, c_int, c_void};
use core::sync::atomic::{AtomicBool, Ordering};

use crate::handshake::{RxEapolAction, TxDoneAction, WpaStaStateMachine};
use crate::ie;

/// Station SSID representation matching C ABI.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct WifiSsid {
    /// SSID length in bytes (0..=32).
    pub len: c_int,
    /// SSID byte array.
    pub ssid: [u8; 32],
}

/// Parsed WPA/RSN Information Element structure matching C ABI.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct WifiWpaIe {
    /// Protocol type (WPA = 1, RSN = 2).
    pub proto: c_int,
    /// Pairwise cipher suite.
    pub pairwise_cipher: c_int,
    /// Group cipher suite.
    pub group_cipher: c_int,
    /// Key management suite.
    pub key_mgmt: c_int,
    /// RSN capabilities bitfield.
    pub capabilities: c_int,
    /// Number of PMKIDs in IE.
    pub num_pmkid: usize,
    /// Pointer to first PMKID.
    pub pmkid: *const u8,
    /// Management group cipher suite.
    pub mgmt_group_cipher: c_int,
    /// RSNXE capability byte.
    pub rsnxe_capa: u8,
}

/// Integrity Group Temporal Key structure matching C ABI.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct WifiWpaIgtk {
    /// 2-byte Key ID (little-endian).
    pub keyid: [u8; 2],
    /// 6-byte Packet Number.
    pub pn: [u8; 6],
    /// 32-byte IGTK key buffer.
    pub igtk: [u8; 32],
}

/// WPA callback function jump table registered with the ESP32 WiFi driver.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct WpaFuncs {
    pub wpa_attach: Option<unsafe extern "C" fn() -> bool>,
    pub wpa_deattach: Option<unsafe extern "C" fn() -> bool>,
    pub wpa_sta_connect: Option<unsafe extern "C" fn(*mut u8) -> c_int>,
    pub wpa_sta_connected_cb: Option<unsafe extern "C" fn(*mut u8)>,
    pub wpa_sta_disconnected_cb: Option<unsafe extern "C" fn(u8)>,
    pub wpa_sm_rx_eapol: Option<unsafe extern "C" fn(*mut u8, *mut u8, usize) -> c_int>,
    pub wpa_sta_in_4way_handshake: Option<unsafe extern "C" fn() -> bool>,
    pub hostap_init: Option<unsafe extern "C" fn() -> *mut c_void>,
    pub hostap_deinit: Option<unsafe extern "C" fn(*mut c_void) -> bool>,
    pub hostap_sta_join: Option<
        unsafe extern "C" fn(
            *mut *mut c_void,
            *mut u8,
            *mut u8,
            u8,
            *mut u8,
            u16,
            *mut bool,
            c_int,
            *mut u8,
        ) -> bool,
    >,
    pub wpa_ap_remove: Option<unsafe extern "C" fn(*mut u8) -> bool>,
    pub wpa_ap_get_wpa_ie: Option<unsafe extern "C" fn(*mut usize) -> *mut u8>,
    pub wpa_ap_rx_eapol:
        Option<unsafe extern "C" fn(*mut c_void, *mut c_void, *mut u8, usize) -> bool>,
    pub wpa_ap_get_peer_spp_msg: Option<unsafe extern "C" fn(*mut c_void, *mut bool, *mut bool)>,
    pub wpa_config_parse_string:
        Option<unsafe extern "C" fn(*const c_char, *mut usize) -> *mut c_char>,
    pub wpa_parse_wpa_ie: Option<unsafe extern "C" fn(*const u8, usize, *mut WifiWpaIe) -> c_int>,
    pub wpa_config_bss: Option<unsafe extern "C" fn(*const c_char, c_int) -> c_int>,
    pub wpa_michael_mic_failure: Option<unsafe extern "C" fn(u16) -> c_int>,
    pub wpa_config_set_unsafe: Option<unsafe extern "C" fn()>,
    pub wpa_config_set_int: Option<unsafe extern "C" fn()>,
    pub wpa_sta_rx_eapol_retry: Option<unsafe extern "C" fn()>,
    pub wpa_sta_rx_mgmt: Option<unsafe extern "C" fn()>,
    pub wpa_config_done: Option<unsafe extern "C" fn()>,
    pub wpa_config_set_array: Option<unsafe extern "C" fn()>,
    pub wpa_config_set_byte_array: Option<unsafe extern "C" fn()>,
    pub wpa_sta_clear_curr_pmksa: Option<unsafe extern "C" fn()>,
    pub wpa_config_reload: Option<unsafe extern "C" fn()>,
}

// JUSTIFICATION: Jump table function pointers are read-only and thread-safe.
// SAFETY: WpaFuncs contains only function pointers with static lifetimes.
unsafe impl Sync for WpaFuncs {}

// JUSTIFICATION: Function pointers can safely be transferred across threads.
// SAFETY: WpaFuncs contains only stateless function pointer fields.
unsafe impl Send for WpaFuncs {}

#[cfg(target_pointer_width = "32")]
const _: () = assert!(core::mem::size_of::<WpaFuncs>() == 108);

/// Transparent cell providing synchronized access without static mut.
#[repr(transparent)]
pub struct SyncCell<T>(pub UnsafeCell<T>);

// JUSTIFICATION: Access to inner cell synchronized via atomic spinlock.
// SAFETY: Exclusive access is guaranteed by external atomic spinlock.
unsafe impl<T> Sync for SyncCell<T> {}

// JUSTIFICATION: Types wrapped in SyncCell can be transferred across threads.
// SAFETY: Exclusive access is enforced via external synchronization.
unsafe impl<T> Send for SyncCell<T> {}

/// Internal supplicant state tracking the active connection and handshake.
struct SupplicantState {
    sm: WpaStaStateMachine,
    own_mac: [u8; 6],
    bssid: [u8; 6],
    assoc_ie: [u8; 128],
    assoc_len: usize,
    pmk_caching_disabled: bool,
    attached: bool,
}

impl SupplicantState {
    const fn new() -> Self {
        Self {
            sm: WpaStaStateMachine::new(),
            own_mac: [0; 6],
            bssid: [0; 6],
            assoc_ie: [0; 128],
            assoc_len: 0,
            pmk_caching_disabled: false,
            attached: false,
        }
    }
}

static STATE_LOCK: AtomicBool = AtomicBool::new(false);
static STATE: SyncCell<SupplicantState> = SyncCell(UnsafeCell::new(SupplicantState::new()));

fn with_state<R, F: FnOnce(&mut SupplicantState) -> R>(f: F) -> R {
    while STATE_LOCK
        .compare_exchange_weak(false, true, Ordering::Acquire, Ordering::Relaxed)
        .is_err()
    {
        core::hint::spin_loop();
    }
    // JUSTIFICATION: Exclusive access guaranteed by atomic spinlock above.
    // SAFETY: STATE_LOCK ensures no concurrent access to UnsafeCell.
    let state = unsafe { &mut *STATE.0.get() };
    let res = f(state);
    STATE_LOCK.store(false, Ordering::Release);
    res
}

/// Static callback jump table registered with the ESP32 WiFi driver.
pub static WPA_FUNCS: WpaFuncs = WpaFuncs {
    wpa_attach: Some(wpa_attach),
    wpa_deattach: Some(wpa_deattach),
    wpa_sta_connect: Some(wpa_sta_connect),
    wpa_sta_connected_cb: Some(wpa_sta_connected_cb),
    wpa_sta_disconnected_cb: Some(wpa_sta_disconnected_cb),
    wpa_sm_rx_eapol: Some(wpa_sm_rx_eapol),
    wpa_sta_in_4way_handshake: Some(wpa_sta_in_4way_handshake),
    hostap_init: Some(crate::stubs::hostap_init),
    hostap_deinit: Some(crate::stubs::hostap_deinit),
    hostap_sta_join: Some(crate::stubs::hostap_sta_join),
    wpa_ap_remove: Some(crate::stubs::wpa_ap_remove),
    wpa_ap_get_wpa_ie: Some(crate::stubs::wpa_ap_get_wpa_ie),
    wpa_ap_rx_eapol: Some(crate::stubs::wpa_ap_rx_eapol),
    wpa_ap_get_peer_spp_msg: Some(crate::stubs::wpa_ap_get_peer_spp_msg),
    wpa_config_parse_string: Some(crate::stubs::wpa_config_parse_string),
    wpa_parse_wpa_ie: Some(wpa_parse_wpa_ie_wrapper),
    wpa_config_bss: None,
    wpa_michael_mic_failure: Some(crate::stubs::wpa_michael_mic_failure),
    wpa_config_set_unsafe: None,
    wpa_config_set_int: None,
    wpa_sta_rx_eapol_retry: None,
    wpa_sta_rx_mgmt: None,
    wpa_config_done: Some(crate::stubs::wpa_config_done),
    wpa_config_set_array: None,
    wpa_config_set_byte_array: None,
    wpa_sta_clear_curr_pmksa: Some(crate::stubs::wpa_sta_clear_curr_pmksa),
    wpa_config_reload: Some(crate::stubs::wpa_config_reload),
};

static G_WPA_FUNCS: SyncCell<WpaFuncs> = SyncCell(UnsafeCell::new(WPA_FUNCS));

// JUSTIFICATION: C callback for supplicant attachment.
// SAFETY: Operates on global supplicant state with synchronized access.
pub unsafe extern "C" fn wpa_attach() -> bool {
    with_state(|state| {
        state.sm.reset();
        state.attached = true;
    });
    // JUSTIFICATION: Registering EAPOL TX completion callback with driver.
    // SAFETY: eapol_txcb has static lifetime and matches required C signature.
    unsafe {
        driver::esp_wifi_register_eapol_txdonecb_internal(Some(eapol_txcb));
    }
    true
}

// JUSTIFICATION: C callback for supplicant detachment.
// SAFETY: Operates on global supplicant state with synchronized access.
pub unsafe extern "C" fn wpa_deattach() -> bool {
    // JUSTIFICATION: Unregistering EAPOL TX completion callback with driver.
    // SAFETY: Passing None unregisters the callback safely.
    unsafe {
        driver::esp_wifi_register_eapol_txdonecb_internal(None);
    }
    with_state(|state| {
        state.sm.reset();
        state.attached = false;
    });
    true
}

// JUSTIFICATION: C callback invoked when STA connects at link layer.
// SAFETY: bssid pointer is validated for null before dereference.
pub unsafe extern "C" fn wpa_sta_connected_cb(bssid: *mut u8) {
    with_state(|state| {
        if !bssid.is_null() {
            // JUSTIFICATION: Dereferencing non-null bssid pointer.
            // SAFETY: Driver supplies 6-byte MAC address when non-null.
            unsafe {
                core::ptr::copy_nonoverlapping(bssid, state.bssid.as_mut_ptr(), 6);
            }
        }
    });
}

// JUSTIFICATION: C callback invoked when STA disconnects.
// SAFETY: Operates on global supplicant state with synchronized access.
pub unsafe extern "C" fn wpa_sta_disconnected_cb(_reason: u8) {
    with_state(|state| {
        state.sm.on_disconnected();
    });
}

// JUSTIFICATION: C callback querying if 4-way handshake is active.
// SAFETY: Operates on global supplicant state with synchronized access.
pub unsafe extern "C" fn wpa_sta_in_4way_handshake() -> bool {
    with_state(|state| state.sm.in_4way_handshake())
}

// JUSTIFICATION: C callback initiated by driver to start connection to BSSID.
// SAFETY: Validates all pointers and buffers before passing to state machine.
pub unsafe extern "C" fn wpa_sta_connect(bssid: *mut u8) -> c_int {
    if bssid.is_null() {
        return -1;
    }
    let mut bssid_arr = [0u8; 6];
    // JUSTIFICATION: Copying target BSSID from driver memory.
    // SAFETY: bssid pointer is checked non-null and valid for 6 bytes.
    unsafe {
        core::ptr::copy_nonoverlapping(bssid, bssid_arr.as_mut_ptr(), 6);
    }

    // JUSTIFICATION: Querying station profile configuration from driver.
    // SAFETY: Driver profile query functions have no preconditions.
    let (is_wpa, is_rsn, authmode) = unsafe {
        (
            driver::esp_wifi_sta_prof_is_wpa_internal(),
            driver::esp_wifi_sta_prof_is_rsn_internal(),
            driver::esp_wifi_sta_get_prof_authmode_internal(),
        )
    };

    let proto = if is_wpa {
        ie::WPA_PROTO_WPA
    } else if is_rsn {
        ie::WPA_PROTO_RSN
    } else if authmode == 0 {
        // JUSTIFICATION: Unsetting application IEs for open network connection.
        // SAFETY: Driver functions handle unsetting IEs cleanly.
        unsafe {
            driver::esp_wifi_unset_appie_internal(3);
            driver::esp_wifi_unset_appie_internal(4);
            return driver::esp_wifi_sta_connect_internal(bssid);
        }
    } else {
        return -1;
    };

    let mut own_mac = [0u8; 6];
    // JUSTIFICATION: Querying local station MAC address from driver.
    // SAFETY: own_mac points to a valid 6-byte stack array.
    let mac_ret = unsafe { driver::esp_wifi_get_macaddr_internal(0, own_mac.as_mut_ptr()) };
    if mac_ret != 0 {
        return -1;
    }

    let mut ssid_buf = [0u8; 32];
    let ssid_len: usize;
    // JUSTIFICATION: Querying SSID from driver station profile.
    // SAFETY: Pointer checked non-null and ssid_len bounded to 0..=32.
    unsafe {
        let ssid_ptr = driver::esp_wifi_sta_get_prof_ssid_internal();
        if ssid_ptr.is_null() {
            return -1;
        }
        let raw_len = (*ssid_ptr).len;
        if !(0..=32).contains(&raw_len) {
            return -1;
        }
        ssid_len = raw_len as usize;
        core::ptr::copy_nonoverlapping(
            core::ptr::addr_of!((*ssid_ptr).ssid).cast::<u8>(),
            ssid_buf.as_mut_ptr(),
            ssid_len,
        );
    }

    // JUSTIFICATION: Querying cipher suites from driver station profile.
    // SAFETY: Driver cipher query functions have no preconditions.
    let (raw_pw, raw_grp) = unsafe {
        (
            driver::esp_wifi_sta_get_pairwise_cipher_internal(),
            driver::esp_wifi_sta_get_group_cipher_internal(),
        )
    };
    let key_mgmt = if authmode == 8 {
        ie::WPA_KEY_MGMT_PSK_SHA256
    } else {
        ie::WPA_KEY_MGMT_PSK
    };

    let mut pairwise = 1u32 << u32::from(raw_pw & 0x1f);
    let mut group = 1u32 << u32::from(raw_grp & 0x1f);
    if pairwise <= ie::WPA_CIPHER_NONE {
        pairwise = if proto == ie::WPA_PROTO_RSN {
            ie::WPA_CIPHER_CCMP
        } else {
            ie::WPA_CIPHER_TKIP
        };
    }
    if group <= ie::WPA_CIPHER_NONE {
        group = if proto == ie::WPA_PROTO_RSN {
            ie::WPA_CIPHER_CCMP
        } else {
            ie::WPA_CIPHER_TKIP
        };
    }

    let mut mgmt_group = 0u32;
    let mut rsn_caps = 0u16;
    if proto == ie::WPA_PROTO_RSN {
        // JUSTIFICATION: Querying driver PMF enablement flag for station profile.
        // SAFETY: Driver query reads static interface configuration cleanly.
        let pmf_enabled = unsafe { driver::esp_wifi_sta_pmf_enabled() };
        if pmf_enabled != 0 {
            // JUSTIFICATION: Querying station PMF config and management group cipher.
            // SAFETY: pmf_cfg points to a valid 2-byte stack array.
            let (raw_mgmt, pmf_cfg) = unsafe {
                let mut cfg = [0u8; 2];
                driver::esp_wifi_get_pmf_config_internal(cfg.as_mut_ptr(), 0);
                (driver::esp_wifi_sta_get_mgmt_group_cipher(), cfg)
            };
            let mapped_mgmt = ie::cipher_type_map_public_to_supp(i32::from(raw_mgmt));
            if mapped_mgmt == ie::WPA_CIPHER_NONE {
                return -1;
            }
            mgmt_group = mapped_mgmt;
            if pmf_cfg[0] != 0 {
                rsn_caps = if pmf_cfg[1] != 0 { 0x00C0 } else { 0x0080 };
            }
        }
    }

    let mut ap_ie_buf = [0u8; 128];
    let mut ap_ie_len = 0usize;
    if proto == ie::WPA_PROTO_RSN {
        // JUSTIFICATION: Fetching cached AP RSN IE from driver BSS entry.
        // SAFETY: Pointer checked non-null and IE TLV length bounded to <= 128 bytes.
        unsafe {
            let ie_ptr = driver::esp_wifi_sta_get_ie(bssid_arr.as_ptr(), 0x30);
            if !ie_ptr.is_null() && *ie_ptr == 0x30 {
                let tlv_len = 2 + usize::from(*ie_ptr.add(1));
                if tlv_len <= ap_ie_buf.len() {
                    let src = core::slice::from_raw_parts(ie_ptr, tlv_len);
                    ap_ie_buf[..tlv_len].copy_from_slice(src);
                    ap_ie_len = tlv_len;
                }
            }
        }
    }

    let mut pass_buf = [0u8; 64];
    let mut pass_len = 0usize;
    // JUSTIFICATION: Querying passphrase from driver station profile.
    // SAFETY: pwd_ptr checked non-null and copied up to NUL byte or 64 bytes.
    unsafe {
        let pwd_ptr = driver::esp_wifi_sta_get_prof_password_internal();
        if !pwd_ptr.is_null() {
            while pass_len < 64 && *pwd_ptr.add(pass_len) != 0 {
                pass_buf[pass_len] = *pwd_ptr.add(pass_len);
                pass_len += 1;
            }
        }
    }

    let mut assoc_buf_copy = [0u8; 128];
    let connect_res = with_state(|state| {
        state.own_mac = own_mac;
        state.bssid = bssid_arr;
        match state.sm.start_connect(
            own_mac,
            bssid_arr,
            &ssid_buf[..ssid_len],
            &pass_buf[..pass_len],
            &ap_ie_buf[..ap_ie_len],
            proto,
            pairwise,
            group,
            key_mgmt,
            mgmt_group,
            rsn_caps,
        ) {
            Ok((assoc_buf, assoc_len)) => {
                state.assoc_ie = assoc_buf;
                state.assoc_len = assoc_len;
                assoc_buf_copy = assoc_buf;
                Some(assoc_len)
            }
            Err(_) => None,
        }
    });

    let Some(assoc_len) = connect_res else {
        return -1;
    };

    let ie_type = if proto == ie::WPA_PROTO_WPA { 3 } else { 4 };
    let opp_type = if ie_type == 3 { 4 } else { 3 };
    // JUSTIFICATION: Clear opposite AppIE slot and register AssocReq IE copy.
    // SAFETY: assoc_buf_copy holds assoc_len bytes; driver copies on flag == 0.
    unsafe {
        driver::esp_wifi_unset_appie_internal(opp_type);
        driver::esp_wifi_set_appie_internal(ie_type, assoc_buf_copy.as_ptr(), assoc_len as u16, 0);
        driver::esp_wifi_sta_connect_internal(bssid)
    }
}

// JUSTIFICATION: C callback for received EAPOL frames.
// SAFETY: Validates pointers and lengths before processing EAPOL payload.
pub unsafe extern "C" fn wpa_sm_rx_eapol(src_addr: *mut u8, buf: *mut u8, len: usize) -> c_int {
    if src_addr.is_null() || buf.is_null() || len == 0 {
        return -1;
    }

    let mut src_mac = [0u8; 6];
    // JUSTIFICATION: Copying source MAC address from driver buffer.
    // SAFETY: src_addr pointer checked non-null and valid for 6 bytes.
    unsafe {
        core::ptr::copy_nonoverlapping(src_addr, src_mac.as_mut_ptr(), 6);
    }

    let mut snonce_rand = [0u8; 32];
    // JUSTIFICATION: Querying hardware random number generator for SNonce.
    // SAFETY: snonce_rand points to a valid 32-byte stack buffer.
    unsafe {
        driver::__esp_radio_esp_fill_random(snonce_rand.as_mut_ptr(), 32);
    }

    // JUSTIFICATION: Creating immutable slice from driver EAPOL buffer.
    // SAFETY: buf is checked non-null and len valid bytes are present.
    let eapol_slice = unsafe { core::slice::from_raw_parts(buf, len) };

    let (action, bssid, own_mac) = with_state(|state| {
        let act = state.sm.on_rx_eapol(&src_mac, eapol_slice, &snonce_rand);
        (act, state.bssid, state.own_mac)
    });

    match action {
        Ok(RxEapolAction::Ignore) => 0,
        Ok(RxEapolAction::SendFrame(frame)) => {
            let total_len = 14 + frame.len;
            if total_len > 400 {
                return -1;
            }
            let mut tx_buf = [0u8; 400];
            tx_buf[0..6].copy_from_slice(&bssid);
            tx_buf[6..12].copy_from_slice(&own_mac);
            tx_buf[12] = 0x88;
            tx_buf[13] = 0x8E;
            tx_buf[14..total_len].copy_from_slice(&frame.data[..frame.len]);
            // JUSTIFICATION: Transmitting EAPOL frame through WiFi driver.
            // SAFETY: tx_buf is a valid stack buffer with length total_len.
            let ret = unsafe {
                driver::esp_wifi_internal_tx(0, tx_buf.as_mut_ptr().cast(), total_len as u16)
            };
            if ret == 0 {
                0
            } else {
                -1
            }
        }
        Ok(RxEapolAction::Deauth(reason)) => {
            // JUSTIFICATION: Deauthenticating station on fatal handshake failure.
            // SAFETY: reason is an 8-bit IEEE 802.11 reason code.
            unsafe {
                driver::esp_wifi_deauthenticate_internal(reason);
            }
            -1
        }
        Err(_) => -1,
    }
}

// JUSTIFICATION: C callback for driver EAPOL TX completion.
// SAFETY: Validates frame buffer pointer and length before inspection.
pub unsafe extern "C" fn eapol_txcb(eapol_payload: *mut u8, len: usize, tx_failure: bool) {
    if eapol_payload.is_null() || len < 99 {
        return;
    }
    // JUSTIFICATION: Checking EAPOL packet type field at offset 1.
    // SAFETY: eapol_payload verified non-null and len >= 99.
    let is_eapol_key = unsafe { *eapol_payload.add(1) == 0x03 };
    if !is_eapol_key {
        return;
    }
    // JUSTIFICATION: Read big-endian Key Info and Key Data Len in EAPOL-Key.
    // SAFETY: eapol_payload is non-null and len >= 99 covers 5..7 and 97..99.
    let (key_info, kd_len) = unsafe {
        (
            u16::from_be_bytes([*eapol_payload.add(5), *eapol_payload.add(6)]),
            u16::from_be_bytes([*eapol_payload.add(97), *eapol_payload.add(98)]),
        )
    };
    if kd_len != 0 && (key_info & 0x0200) == 0 {
        return;
    }

    let (action, bssid) = with_state(|state| {
        let act = state.sm.on_eapol_tx_complete(tx_failure);
        (act, state.bssid)
    });

    if let TxDoneAction::InstallAndComplete(pi) = action {
        if pi.install_ptk {
            let zero_seq = [0u8; 8];
            // JUSTIFICATION: Configuring Pairwise Transient Key in WiFi driver.
            // SAFETY: Pointers to bssid, zero_seq, and ptk_key are valid.
            unsafe {
                driver::esp_wifi_set_sta_key_internal(
                    pi.ptk_alg,
                    bssid.as_ptr(),
                    0,
                    1,
                    zero_seq.as_ptr(),
                    zero_seq.len(),
                    pi.ptk_key.as_ptr(),
                    pi.ptk_key_len,
                    0x2C,
                );
            }
        }
        if pi.gtk_len > 0 {
            // JUSTIFICATION: Configuring Group Temporal Key in WiFi driver.
            // SAFETY: Pointers to bssid, gtk_rsc, and gtk are valid.
            unsafe {
                driver::esp_wifi_set_sta_key_internal(
                    pi.gtk_alg,
                    bssid.as_ptr(),
                    pi.gtk_idx as c_int,
                    0,
                    pi.gtk_rsc.as_ptr(),
                    pi.gtk_rsc.len(),
                    pi.gtk.as_ptr(),
                    pi.gtk_len,
                    0x14,
                );
            }
        }
        if pi.has_igtk {
            let igtk_struct = WifiWpaIgtk {
                keyid: pi.igtk_idx.to_le_bytes(),
                pn: pi.igtk_pn,
                igtk: pi.igtk,
            };
            // JUSTIFICATION: Configuring Integrity GTK in WiFi driver.
            // SAFETY: Passing valid pointer to stack-allocated WifiWpaIgtk.
            unsafe {
                driver::esp_wifi_set_igtk_internal(0, &igtk_struct);
            }
        }
        if pi.notify_auth_done {
            // JUSTIFICATION: Notifying WiFi driver that 4-way handshake completed.
            // SAFETY: Safe notification without pointer parameters.
            unsafe {
                driver::esp_wifi_auth_done_internal();
            }
        }
    }
}

// JUSTIFICATION: C callback for parsing WPA/RSN Information Elements.
// SAFETY: Pointers are verified non-null before writing output.
pub unsafe extern "C" fn wpa_parse_wpa_ie_wrapper(
    wpa_ie: *const u8,
    wpa_ie_len: usize,
    data: *mut WifiWpaIe,
) -> c_int {
    if wpa_ie.is_null() || data.is_null() || wpa_ie_len == 0 {
        return -1;
    }
    // JUSTIFICATION: Initializing output structure to zero bytes.
    // SAFETY: data pointer is verified non-null and valid for WifiWpaIe.
    unsafe {
        core::ptr::write_bytes(data, 0, 1);
    }
    // JUSTIFICATION: Creating immutable slice from IE input buffer.
    // SAFETY: wpa_ie is non-null and valid for wpa_ie_len bytes.
    let ie_slice = unsafe { core::slice::from_raw_parts(wpa_ie, wpa_ie_len) };
    match ie::parse_wpa_or_rsn_ie(ie_slice) {
        Ok(parsed) => {
            // JUSTIFICATION: Populating caller WifiWpaIe fields.
            // SAFETY: data pointer is non-null and valid for write.
            unsafe {
                (*data).proto = parsed.proto;
                (*data).pairwise_cipher =
                    ie::cipher_type_map_supp_to_public(parsed.pairwise_cipher);
                (*data).group_cipher = ie::cipher_type_map_supp_to_public(parsed.group_cipher);
                (*data).key_mgmt = parsed.key_mgmt as c_int;
                (*data).capabilities = i32::from(parsed.capabilities);
                (*data).mgmt_group_cipher =
                    ie::cipher_type_map_supp_to_public(parsed.mgmt_group_cipher);
                (*data).rsnxe_capa = parsed.rsnxe_capa;
                if parsed.has_pmkid && parsed.first_pmkid_offset < wpa_ie_len {
                    (*data).num_pmkid = 1;
                    (*data).pmkid = wpa_ie.add(parsed.first_pmkid_offset);
                } else {
                    (*data).num_pmkid = 0;
                    (*data).pmkid = core::ptr::null();
                }
            }
            0
        }
        Err(_) => -1,
    }
}

// JUSTIFICATION: Inbound C FFI function initializing the WPA supplicant.
// SAFETY: Operates on global state and registers jump table with driver.
#[no_mangle]
pub unsafe extern "C" fn esp_supplicant_init() -> c_int {
    with_state(|state| {
        state.sm.reset();
        state.attached = false;
        state.pmk_caching_disabled = false;
        state.assoc_len = 0;
        state.own_mac = [0; 6];
        state.bssid = [0; 6];
    });
    // JUSTIFICATION: Registering WPA callback jump table with WiFi driver.
    // SAFETY: G_WPA_FUNCS is static and outlives the driver.
    let ret = unsafe { driver::esp_wifi_register_wpa_cb_internal(G_WPA_FUNCS.0.get()) };
    if ret == 0 {
        0
    } else {
        -1
    }
}

// JUSTIFICATION: Inbound C FFI function deinitializing the WPA supplicant.
// SAFETY: Teardown of supplicant resources and unregistration with driver.
#[no_mangle]
pub unsafe extern "C" fn esp_supplicant_deinit() -> c_int {
    // JUSTIFICATION: Unset application IEs (types 0..15) on supplicant deinit.
    // SAFETY: Driver functions handle unsetting IEs cleanly.
    unsafe {
        for t in 0..15u8 {
            driver::esp_wifi_unset_appie_internal(t);
        }
    }
    // JUSTIFICATION: Calling wpa_deattach to unregister EAPOL callbacks.
    // SAFETY: Internal function with static callback cleanup.
    unsafe {
        wpa_deattach();
    }
    with_state(|state| {
        state.sm.reset();
        state.sm.clear_pmk();
        state.attached = false;
        state.assoc_len = 0;
    });
    // JUSTIFICATION: Unregistering WPA callbacks from WiFi driver.
    // SAFETY: Reverts callback registration in driver.
    let ret = unsafe { driver::esp_wifi_unregister_wpa_cb_internal() };
    if ret == 0 {
        0
    } else {
        -1
    }
}

// JUSTIFICATION: Inbound C FFI function to disable or enable PMK caching.
// SAFETY: Updates PMK caching flag in synchronized supplicant state.
#[no_mangle]
pub unsafe extern "C" fn esp_supplicant_disable_pmk_caching(disable: bool) -> c_int {
    with_state(|state| {
        state.pmk_caching_disabled = disable;
        if disable {
            state.sm.clear_pmk();
        }
    });
    0
}

/// Forces linkage of all exported WPA/WPA2 C-ABI symbols.
pub fn link_wpa2() -> usize {
    let a = core::hint::black_box(core::ptr::addr_of!(
        crate::crypto_ffi::g_wifi_default_wpa_crypto_funcs
    ) as usize);
    let b = core::hint::black_box(esp_supplicant_init as *const () as usize);
    let c = core::hint::black_box(esp_supplicant_deinit as *const () as usize);
    let d = core::hint::black_box(esp_supplicant_disable_pmk_caching as *const () as usize);
    let e = core::hint::black_box(crate::stubs::hexstr2bin as *const () as usize);
    a ^ b ^ c ^ d ^ e
}

#[cfg(all(not(test), target_arch = "xtensa"))]
mod driver {
    use super::*;

    extern "C" {
        pub fn __esp_radio_esp_fill_random(dst: *mut u8, len: u32);
        pub fn esp_wifi_internal_tx(wifi_if: c_int, buffer: *mut c_void, len: u16) -> c_int;
        pub fn esp_wifi_sta_prof_is_wpa_internal() -> bool;
        pub fn esp_wifi_sta_prof_is_rsn_internal() -> bool;
        pub fn esp_wifi_sta_get_prof_authmode_internal() -> u8;
        pub fn esp_wifi_sta_get_prof_ssid_internal() -> *const WifiSsid;
        pub fn esp_wifi_get_macaddr_internal(if_index: u8, macaddr: *mut u8) -> c_int;
        pub fn esp_wifi_sta_get_pairwise_cipher_internal() -> u8;
        pub fn esp_wifi_sta_get_group_cipher_internal() -> u8;
        pub fn esp_wifi_sta_get_prof_password_internal() -> *const u8;
        pub fn esp_wifi_sta_pmf_enabled() -> u16;
        pub fn esp_wifi_sta_get_mgmt_group_cipher() -> u8;
        pub fn esp_wifi_get_pmf_config_internal(pmf_cfg: *mut u8, ifx: u8);
        pub fn esp_wifi_sta_get_ie(bssid: *const u8, ie_type: u8) -> *const u8;
        pub fn esp_wifi_set_appie_internal(ie_type: u8, ie: *const u8, len: u16, flag: u8)
            -> c_int;
        pub fn esp_wifi_unset_appie_internal(ie_type: u8) -> c_int;
        pub fn esp_wifi_sta_connect_internal(bssid: *const u8) -> c_int;
        pub fn esp_wifi_auth_done_internal();
        pub fn esp_wifi_deauthenticate_internal(reason_code: u8);
        pub fn esp_wifi_set_sta_key_internal(
            alg: c_int,
            addr: *const u8,
            key_idx: c_int,
            set_tx: c_int,
            seq: *const u8,
            seq_len: usize,
            key: *const u8,
            key_len: usize,
            key_flag: c_int,
        ) -> c_int;
        pub fn esp_wifi_set_igtk_internal(ifx: u8, igtk: *const WifiWpaIgtk) -> c_int;
        pub fn esp_wifi_register_eapol_txdonecb_internal(
            fn_cb: Option<unsafe extern "C" fn(*mut u8, usize, bool)>,
        ) -> c_int;
        pub fn esp_wifi_register_wpa_cb_internal(cb: *mut WpaFuncs) -> c_int;
        pub fn esp_wifi_unregister_wpa_cb_internal() -> c_int;
    }
}

#[cfg(all(not(test), not(target_arch = "xtensa")))]
#[allow(dead_code)]
mod driver {
    use super::*;

    /// # Safety
    ///
    /// Host link stub; does not dereference pointers or access hardware.
    // JUSTIFICATION: Host driver link stub for non-Xtensa builds.
    // SAFETY: Stateless host stub; never dereferences pointer arguments.
    pub(super) unsafe extern "C" fn __esp_radio_esp_fill_random(_dst: *mut u8, _len: u32) {}

    /// # Safety
    ///
    /// Host link stub; does not dereference pointers or access hardware.
    // JUSTIFICATION: Host driver link stub for non-Xtensa builds.
    // SAFETY: Stateless host stub; never dereferences pointer arguments.
    pub(super) unsafe extern "C" fn esp_wifi_internal_tx(
        _wifi_if: c_int,
        _buffer: *mut c_void,
        _len: u16,
    ) -> c_int {
        -1
    }

    /// # Safety
    ///
    /// Host link stub; does not dereference pointers or access hardware.
    // JUSTIFICATION: Host driver link stub for non-Xtensa builds.
    // SAFETY: Stateless host stub; does not perform unsafe operations.
    pub(super) unsafe extern "C" fn esp_wifi_sta_prof_is_wpa_internal() -> bool {
        false
    }

    /// # Safety
    ///
    /// Host link stub; does not dereference pointers or access hardware.
    // JUSTIFICATION: Host driver link stub for non-Xtensa builds.
    // SAFETY: Stateless host stub; does not perform unsafe operations.
    pub(super) unsafe extern "C" fn esp_wifi_sta_prof_is_rsn_internal() -> bool {
        false
    }

    /// # Safety
    ///
    /// Host link stub; does not dereference pointers or access hardware.
    // JUSTIFICATION: Host driver link stub for non-Xtensa builds.
    // SAFETY: Stateless host stub; does not perform unsafe operations.
    pub(super) unsafe extern "C" fn esp_wifi_sta_get_prof_authmode_internal() -> u8 {
        0
    }

    /// # Safety
    ///
    /// Host link stub; does not dereference pointers or access hardware.
    // JUSTIFICATION: Host driver link stub for non-Xtensa builds.
    // SAFETY: Stateless host stub; returns null pointer.
    pub(super) unsafe extern "C" fn esp_wifi_sta_get_prof_ssid_internal() -> *const WifiSsid {
        core::ptr::null()
    }

    /// # Safety
    ///
    /// Host link stub; does not dereference pointers or access hardware.
    // JUSTIFICATION: Host driver link stub for non-Xtensa builds.
    // SAFETY: Stateless host stub; never dereferences pointer arguments.
    pub(super) unsafe extern "C" fn esp_wifi_get_macaddr_internal(
        _if_index: u8,
        _macaddr: *mut u8,
    ) -> c_int {
        -1
    }

    /// # Safety
    ///
    /// Host link stub; does not dereference pointers or access hardware.
    // JUSTIFICATION: Host driver link stub for non-Xtensa builds.
    // SAFETY: Stateless host stub; does not perform unsafe operations.
    pub(super) unsafe extern "C" fn esp_wifi_sta_get_pairwise_cipher_internal() -> u8 {
        0
    }

    /// # Safety
    ///
    /// Host link stub; does not dereference pointers or access hardware.
    // JUSTIFICATION: Host driver link stub for non-Xtensa builds.
    // SAFETY: Stateless host stub; does not perform unsafe operations.
    pub(super) unsafe extern "C" fn esp_wifi_sta_get_group_cipher_internal() -> u8 {
        0
    }

    /// # Safety
    ///
    /// Host link stub; does not dereference pointers or access hardware.
    // JUSTIFICATION: Host driver link stub for non-Xtensa builds.
    // SAFETY: Stateless host stub; returns null pointer.
    pub(super) unsafe extern "C" fn esp_wifi_sta_get_prof_password_internal() -> *const u8 {
        core::ptr::null()
    }

    /// # Safety
    ///
    /// Host link stub; does not dereference pointers or access hardware.
    // JUSTIFICATION: Host driver link stub for non-Xtensa builds.
    // SAFETY: Stateless host stub; does not perform unsafe operations.
    pub(super) unsafe extern "C" fn esp_wifi_sta_pmf_enabled() -> u16 {
        0
    }

    /// # Safety
    ///
    /// Host link stub; does not dereference pointers or access hardware.
    // JUSTIFICATION: Host driver link stub for non-Xtensa builds.
    // SAFETY: Stateless host stub; does not perform unsafe operations.
    pub(super) unsafe extern "C" fn esp_wifi_sta_get_mgmt_group_cipher() -> u8 {
        0
    }

    /// # Safety
    ///
    /// Host link stub; does not dereference pointers or access hardware.
    // JUSTIFICATION: Host driver link stub for non-Xtensa builds.
    // SAFETY: Stateless host stub; never dereferences pointer arguments.
    pub(super) unsafe extern "C" fn esp_wifi_get_pmf_config_internal(_pmf_cfg: *mut u8, _ifx: u8) {}

    /// # Safety
    ///
    /// Host link stub; does not dereference pointers or access hardware.
    // JUSTIFICATION: Host driver link stub for non-Xtensa builds.
    // SAFETY: Stateless host stub; returns null pointer.
    pub(super) unsafe extern "C" fn esp_wifi_sta_get_ie(
        _bssid: *const u8,
        _ie_type: u8,
    ) -> *const u8 {
        core::ptr::null()
    }

    /// # Safety
    ///
    /// Host link stub; does not dereference pointers or access hardware.
    // JUSTIFICATION: Host driver link stub for non-Xtensa builds.
    // SAFETY: Stateless host stub; never dereferences pointer arguments.
    pub(super) unsafe extern "C" fn esp_wifi_set_appie_internal(
        _ie_type: u8,
        _ie: *const u8,
        _len: u16,
        _flag: u8,
    ) -> c_int {
        -1
    }

    /// # Safety
    ///
    /// Host link stub; does not dereference pointers or access hardware.
    // JUSTIFICATION: Host driver link stub for non-Xtensa builds.
    // SAFETY: Stateless host stub; does not perform unsafe operations.
    pub(super) unsafe extern "C" fn esp_wifi_unset_appie_internal(_ie_type: u8) -> c_int {
        -1
    }

    /// # Safety
    ///
    /// Host link stub; does not dereference pointers or access hardware.
    // JUSTIFICATION: Host driver link stub for non-Xtensa builds.
    // SAFETY: Stateless host stub; never dereferences pointer arguments.
    pub(super) unsafe extern "C" fn esp_wifi_sta_connect_internal(_bssid: *const u8) -> c_int {
        -1
    }

    /// # Safety
    ///
    /// Host link stub; does not dereference pointers or access hardware.
    // JUSTIFICATION: Host driver link stub for non-Xtensa builds.
    // SAFETY: Stateless host stub; does not perform unsafe operations.
    pub(super) unsafe extern "C" fn esp_wifi_auth_done_internal() {}

    /// # Safety
    ///
    /// Host link stub; does not dereference pointers or access hardware.
    // JUSTIFICATION: Host driver link stub for non-Xtensa builds.
    // SAFETY: Stateless host stub; does not perform unsafe operations.
    pub(super) unsafe extern "C" fn esp_wifi_deauthenticate_internal(_reason_code: u8) {}

    /// # Safety
    ///
    /// Host link stub; does not dereference pointers or access hardware.
    // JUSTIFICATION: Host driver link stub for non-Xtensa builds.
    // SAFETY: Stateless host stub; never dereferences pointer arguments.
    #[allow(clippy::too_many_arguments)]
    pub(super) unsafe extern "C" fn esp_wifi_set_sta_key_internal(
        _alg: c_int,
        _addr: *const u8,
        _key_idx: c_int,
        _set_tx: c_int,
        _seq: *const u8,
        _seq_len: usize,
        _key: *const u8,
        _key_len: usize,
        _key_flag: c_int,
    ) -> c_int {
        -1
    }

    /// # Safety
    ///
    /// Host link stub; does not dereference pointers or access hardware.
    // JUSTIFICATION: Host driver link stub for non-Xtensa builds.
    // SAFETY: Stateless host stub; never dereferences pointer arguments.
    pub(super) unsafe extern "C" fn esp_wifi_set_igtk_internal(
        _ifx: u8,
        _igtk: *const WifiWpaIgtk,
    ) -> c_int {
        -1
    }

    /// # Safety
    ///
    /// Host link stub; does not dereference pointers or access hardware.
    // JUSTIFICATION: Host driver link stub for non-Xtensa builds.
    // SAFETY: Stateless host stub; does not perform unsafe operations.
    pub(super) unsafe extern "C" fn esp_wifi_register_eapol_txdonecb_internal(
        _fn_cb: Option<unsafe extern "C" fn(*mut u8, usize, bool)>,
    ) -> c_int {
        -1
    }

    /// # Safety
    ///
    /// Host link stub; does not dereference pointers or access hardware.
    // JUSTIFICATION: Host driver link stub for non-Xtensa builds.
    // SAFETY: Stateless host stub; never dereferences pointer arguments.
    pub(super) unsafe extern "C" fn esp_wifi_register_wpa_cb_internal(_cb: *mut WpaFuncs) -> c_int {
        -1
    }

    /// # Safety
    ///
    /// Host link stub; does not dereference pointers or access hardware.
    // JUSTIFICATION: Host driver link stub for non-Xtensa builds.
    // SAFETY: Stateless host stub; does not perform unsafe operations.
    pub(super) unsafe extern "C" fn esp_wifi_unregister_wpa_cb_internal() -> c_int {
        -1
    }
}

#[cfg(test)]
pub mod driver {
    use super::*;

    pub struct MockDriver {
        pub tx_count: usize,
        pub last_tx_len: usize,
        pub last_tx_buf: [u8; 512],
        pub auth_done_called: bool,
        pub deauth_called: bool,
        pub deauth_reason: u8,
        pub ptk_installed: bool,
        pub ptk_alg: c_int,
        pub ptk_key: [u8; 32],
        pub ptk_key_len: usize,
        pub ptk_flag: c_int,
        pub gtk_installed: bool,
        pub gtk_alg: c_int,
        pub gtk_idx: c_int,
        pub gtk_key: [u8; 32],
        pub gtk_len: usize,
        pub gtk_flag: c_int,
        pub igtk_installed: bool,
        pub appie_3_len: usize,
        pub appie_4_len: usize,
        pub appie_3_buf: [u8; 128],
        pub appie_4_buf: [u8; 128],
        pub appie_3_flag: u8,
        pub appie_4_flag: u8,
        pub appie_3_unset: bool,
        pub appie_4_unset: bool,
        pub connected_bssid: [u8; 6],
        pub cb_registered: bool,
        pub cb_unregistered: bool,
        pub txdonecb_registered: bool,
        pub txdonecb: Option<unsafe extern "C" fn(*mut u8, usize, bool)>,
        pub mock_ssid: WifiSsid,
        pub mock_pwd: [u8; 65],
        pub mock_mac: [u8; 6],
        pub is_wpa: bool,
        pub is_rsn: bool,
        pub authmode: u8,
        pub pairwise_cipher: u8,
        pub group_cipher: u8,
        pub pmf_enabled: u16,
        pub pmf_cfg: [u8; 2],
        pub mgmt_group_cipher: u8,
        pub ap_rsn_ie: [u8; 128],
        pub ap_rsn_ie_len: usize,
    }

    impl MockDriver {
        const fn new() -> Self {
            Self {
                tx_count: 0,
                last_tx_len: 0,
                last_tx_buf: [0; 512],
                auth_done_called: false,
                deauth_called: false,
                deauth_reason: 0,
                ptk_installed: false,
                ptk_alg: 0,
                ptk_key: [0; 32],
                ptk_key_len: 0,
                ptk_flag: 0,
                gtk_installed: false,
                gtk_alg: 0,
                gtk_idx: 0,
                gtk_key: [0; 32],
                gtk_len: 0,
                gtk_flag: 0,
                igtk_installed: false,
                appie_3_len: 0,
                appie_4_len: 0,
                appie_3_buf: [0; 128],
                appie_4_buf: [0; 128],
                appie_3_flag: 0,
                appie_4_flag: 0,
                appie_3_unset: false,
                appie_4_unset: false,
                connected_bssid: [0; 6],
                cb_registered: false,
                cb_unregistered: false,
                txdonecb_registered: false,
                txdonecb: None,
                mock_ssid: WifiSsid {
                    len: 0,
                    ssid: [0; 32],
                },
                mock_pwd: [0; 65],
                mock_mac: [0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc],
                is_wpa: false,
                is_rsn: true,
                authmode: 3,
                pairwise_cipher: 3,
                group_cipher: 3,
                pmf_enabled: 0,
                pmf_cfg: [0; 2],
                mgmt_group_cipher: 0,
                ap_rsn_ie: [0; 128],
                ap_rsn_ie_len: 0,
            }
        }
    }

    static MOCK_LOCK: AtomicBool = AtomicBool::new(false);
    static MOCK: SyncCell<MockDriver> = SyncCell(UnsafeCell::new(MockDriver::new()));

    pub fn with_mock<R, F: FnOnce(&mut MockDriver) -> R>(f: F) -> R {
        while MOCK_LOCK
            .compare_exchange_weak(false, true, Ordering::Acquire, Ordering::Relaxed)
            .is_err()
        {
            core::hint::spin_loop();
        }
        // JUSTIFICATION: Atomic spinlock guarantees single-threaded mock access.
        // SAFETY: MOCK_LOCK ensures exclusive access to inner UnsafeCell.
        let m = unsafe { &mut *MOCK.0.get() };
        let res = f(m);
        MOCK_LOCK.store(false, Ordering::Release);
        res
    }

    pub fn reset_mock() {
        with_mock(|m| *m = MockDriver::new());
    }

    // JUSTIFICATION: Mock random generator for host unit testing.
    // SAFETY: dst is verified non-null and valid for len bytes.
    pub unsafe extern "C" fn __esp_radio_esp_fill_random(dst: *mut u8, len: u32) {
        if !dst.is_null() {
            // JUSTIFICATION: Filling destination buffer with test random bytes.
            // SAFETY: dst is non-null and valid for len bytes in tests.
            unsafe {
                for i in 0..len as usize {
                    *dst.add(i) = (i as u8).wrapping_add(0x42);
                }
            }
        }
    }

    // JUSTIFICATION: Mock frame TX handler recording frames for host testing.
    // SAFETY: buffer is validated non-null before copying len bytes.
    pub unsafe extern "C" fn esp_wifi_internal_tx(
        _wifi_if: c_int,
        buffer: *mut c_void,
        len: u16,
    ) -> c_int {
        with_mock(|m| {
            m.tx_count += 1;
            m.last_tx_len = len as usize;
            if !buffer.is_null() && (len as usize) <= m.last_tx_buf.len() {
                // JUSTIFICATION: Copying TX frame into test inspection buffer.
                // SAFETY: buffer is verified non-null and len fits in buffer.
                unsafe {
                    core::ptr::copy_nonoverlapping(
                        buffer.cast::<u8>(),
                        m.last_tx_buf.as_mut_ptr(),
                        len as usize,
                    );
                }
            }
        });
        0
    }

    // JUSTIFICATION: Mock profile query for host unit tests.
    // SAFETY: Operates on synchronized mock driver state without pointers.
    pub unsafe extern "C" fn esp_wifi_sta_prof_is_wpa_internal() -> bool {
        with_mock(|m| m.is_wpa)
    }

    // JUSTIFICATION: Mock profile query for host unit tests.
    // SAFETY: Operates on synchronized mock driver state without pointers.
    pub unsafe extern "C" fn esp_wifi_sta_prof_is_rsn_internal() -> bool {
        with_mock(|m| m.is_rsn)
    }

    // JUSTIFICATION: Mock profile auth mode query for host unit tests.
    // SAFETY: Operates on synchronized mock driver state without pointers.
    pub unsafe extern "C" fn esp_wifi_sta_get_prof_authmode_internal() -> u8 {
        with_mock(|m| m.authmode)
    }

    // JUSTIFICATION: Mock profile SSID query for host unit tests.
    // SAFETY: Returns pointer to static mock SSID structure.
    pub unsafe extern "C" fn esp_wifi_sta_get_prof_ssid_internal() -> *const WifiSsid {
        with_mock(|m| &m.mock_ssid as *const _)
    }

    // JUSTIFICATION: Mock MAC address query for host unit tests.
    // SAFETY: macaddr pointer checked non-null and valid for 6 bytes.
    pub unsafe extern "C" fn esp_wifi_get_macaddr_internal(
        _if_index: u8,
        macaddr: *mut u8,
    ) -> c_int {
        if macaddr.is_null() {
            return -1;
        }
        with_mock(|m| {
            // JUSTIFICATION: Copying mock MAC address to caller buffer.
            // SAFETY: macaddr pointer checked non-null; valid for 6 bytes.
            unsafe {
                core::ptr::copy_nonoverlapping(m.mock_mac.as_ptr(), macaddr, 6);
            }
        });
        0
    }

    // JUSTIFICATION: Mock pairwise cipher query for host unit tests.
    // SAFETY: Operates on synchronized mock driver state without pointers.
    pub unsafe extern "C" fn esp_wifi_sta_get_pairwise_cipher_internal() -> u8 {
        with_mock(|m| m.pairwise_cipher)
    }

    // JUSTIFICATION: Mock group cipher query for host unit tests.
    // SAFETY: Operates on synchronized mock driver state without pointers.
    pub unsafe extern "C" fn esp_wifi_sta_get_group_cipher_internal() -> u8 {
        with_mock(|m| m.group_cipher)
    }

    // JUSTIFICATION: Mock password query for host unit tests.
    // SAFETY: Returns pointer to static mock password array.
    pub unsafe extern "C" fn esp_wifi_sta_get_prof_password_internal() -> *const u8 {
        with_mock(|m| m.mock_pwd.as_ptr())
    }

    // JUSTIFICATION: Mock application IE registration for host unit tests.
    // SAFETY: ie pointer checked non-null and len bounded to 128.
    pub unsafe extern "C" fn esp_wifi_set_appie_internal(
        ie_type: u8,
        ie: *const u8,
        len: u16,
        flag: u8,
    ) -> c_int {
        with_mock(|m| {
            let len_usize = len as usize;
            if ie_type == 3 {
                m.appie_3_len = len_usize;
                m.appie_3_flag = flag;
                if !ie.is_null() && len_usize <= m.appie_3_buf.len() {
                    // JUSTIFICATION: Copying appie_3 payload for mock verification.
                    // SAFETY: ie is non-null and len_usize bounded by buffer size.
                    unsafe {
                        core::ptr::copy_nonoverlapping(ie, m.appie_3_buf.as_mut_ptr(), len_usize);
                    }
                }
            } else if ie_type == 4 {
                m.appie_4_len = len_usize;
                m.appie_4_flag = flag;
                if !ie.is_null() && len_usize <= m.appie_4_buf.len() {
                    // JUSTIFICATION: Copying appie_4 payload for mock verification.
                    // SAFETY: ie is non-null and len_usize bounded by buffer size.
                    unsafe {
                        core::ptr::copy_nonoverlapping(ie, m.appie_4_buf.as_mut_ptr(), len_usize);
                    }
                }
            }
        });
        0
    }

    // JUSTIFICATION: Mock PMF enablement query for host unit tests.
    // SAFETY: Operates on synchronized mock driver state without pointers.
    pub unsafe extern "C" fn esp_wifi_sta_pmf_enabled() -> u16 {
        with_mock(|m| m.pmf_enabled)
    }

    // JUSTIFICATION: Mock management group cipher query for host unit tests.
    // SAFETY: Operates on synchronized mock driver state without pointers.
    pub unsafe extern "C" fn esp_wifi_sta_get_mgmt_group_cipher() -> u8 {
        with_mock(|m| m.mgmt_group_cipher)
    }

    // JUSTIFICATION: Mock PMF config query for host unit tests.
    // SAFETY: pmf_cfg checked non-null and valid for 2 bytes.
    pub unsafe extern "C" fn esp_wifi_get_pmf_config_internal(pmf_cfg: *mut u8, _ifx: u8) {
        if !pmf_cfg.is_null() {
            with_mock(|m| {
                // JUSTIFICATION: Copying mock PMF config to caller buffer.
                // SAFETY: pmf_cfg is non-null and valid for 2 bytes.
                unsafe {
                    core::ptr::copy_nonoverlapping(m.pmf_cfg.as_ptr(), pmf_cfg, 2);
                }
            });
        }
    }

    // JUSTIFICATION: Mock cached IE query for host unit tests.
    // SAFETY: bssid checked non-null; returns pointer to mock IE buffer or null.
    pub unsafe extern "C" fn esp_wifi_sta_get_ie(bssid: *const u8, ie_type: u8) -> *const u8 {
        if bssid.is_null() || ie_type != 0x30 {
            return core::ptr::null();
        }
        with_mock(|m| {
            if m.ap_rsn_ie_len > 0 {
                m.ap_rsn_ie.as_ptr()
            } else {
                core::ptr::null()
            }
        })
    }

    // JUSTIFICATION: Mock application IE unsetting for host unit tests.
    // SAFETY: Operates on synchronized mock driver state without pointers.
    pub unsafe extern "C" fn esp_wifi_unset_appie_internal(ie_type: u8) -> c_int {
        with_mock(|m| {
            if ie_type == 3 {
                m.appie_3_unset = true;
            } else if ie_type == 4 {
                m.appie_4_unset = true;
            }
        });
        0
    }

    // JUSTIFICATION: Mock station connect call for host unit tests.
    // SAFETY: bssid pointer is verified non-null before copying.
    pub unsafe extern "C" fn esp_wifi_sta_connect_internal(bssid: *const u8) -> c_int {
        if bssid.is_null() {
            return -1;
        }
        with_mock(|m| {
            // JUSTIFICATION: Copying target BSSID into mock record.
            // SAFETY: bssid pointer verified non-null and valid for 6 bytes.
            unsafe {
                core::ptr::copy_nonoverlapping(bssid, m.connected_bssid.as_mut_ptr(), 6);
            }
        });
        0
    }

    // JUSTIFICATION: Mock auth done notification for host unit tests.
    // SAFETY: Operates on synchronized mock driver state without pointers.
    pub unsafe extern "C" fn esp_wifi_auth_done_internal() {
        with_mock(|m| m.auth_done_called = true);
    }

    // JUSTIFICATION: Mock deauthentication notification for host unit tests.
    // SAFETY: Operates on synchronized mock driver state without pointers.
    pub unsafe extern "C" fn esp_wifi_deauthenticate_internal(reason_code: u8) {
        with_mock(|m| {
            m.deauth_called = true;
            m.deauth_reason = reason_code;
        });
    }

    // JUSTIFICATION: Mock key configuration handler for host unit tests.
    // SAFETY: Validates key pointer and bounds before copying into mock.
    pub unsafe extern "C" fn esp_wifi_set_sta_key_internal(
        alg: c_int,
        _addr: *const u8,
        key_idx: c_int,
        _set_tx: c_int,
        _seq: *const u8,
        _seq_len: usize,
        key: *const u8,
        key_len: usize,
        key_flag: c_int,
    ) -> c_int {
        with_mock(|m| {
            if key_idx == 0 {
                m.ptk_installed = true;
                m.ptk_alg = alg;
                m.ptk_flag = key_flag;
                m.ptk_key_len = key_len;
                if !key.is_null() && key_len <= 32 {
                    // JUSTIFICATION: Storing PTK key in mock structure.
                    // SAFETY: key is non-null and within 32-byte bounds.
                    unsafe {
                        core::ptr::copy_nonoverlapping(key, m.ptk_key.as_mut_ptr(), key_len);
                    }
                }
            } else {
                m.gtk_installed = true;
                m.gtk_alg = alg;
                m.gtk_idx = key_idx;
                m.gtk_flag = key_flag;
                m.gtk_len = key_len;
                if !key.is_null() && key_len <= 32 {
                    // JUSTIFICATION: Storing GTK key in mock structure.
                    // SAFETY: key is non-null and within 32-byte bounds.
                    unsafe {
                        core::ptr::copy_nonoverlapping(key, m.gtk_key.as_mut_ptr(), key_len);
                    }
                }
            }
        });
        0
    }

    // JUSTIFICATION: Mock IGTK configuration handler for host unit tests.
    // SAFETY: igtk pointer verified non-null before recording call.
    pub unsafe extern "C" fn esp_wifi_set_igtk_internal(
        _ifx: u8,
        igtk: *const WifiWpaIgtk,
    ) -> c_int {
        if !igtk.is_null() {
            with_mock(|m| m.igtk_installed = true);
        }
        0
    }

    // JUSTIFICATION: Mock EAPOL TX done callback registration for host tests.
    // SAFETY: Stores callback pointer in mock state without dereference.
    pub unsafe extern "C" fn esp_wifi_register_eapol_txdonecb_internal(
        fn_cb: Option<unsafe extern "C" fn(*mut u8, usize, bool)>,
    ) -> c_int {
        with_mock(|m| {
            m.txdonecb_registered = fn_cb.is_some();
            m.txdonecb = fn_cb;
        });
        0
    }

    // JUSTIFICATION: Mock WPA callback registration handler for host tests.
    // SAFETY: Checks cb pointer for non-null without dereference.
    pub unsafe extern "C" fn esp_wifi_register_wpa_cb_internal(cb: *mut WpaFuncs) -> c_int {
        with_mock(|m| {
            m.cb_registered = !cb.is_null();
        });
        0
    }

    // JUSTIFICATION: Mock WPA callback unregistration handler for host tests.
    // SAFETY: Operates on synchronized mock driver state without pointers.
    pub unsafe extern "C" fn esp_wifi_unregister_wpa_cb_internal() -> c_int {
        with_mock(|m| {
            m.cb_unregistered = true;
        });
        0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    static TEST_MUTEX: AtomicBool = AtomicBool::new(false);

    struct TestGuard;

    impl Drop for TestGuard {
        fn drop(&mut self) {
            TEST_MUTEX.store(false, Ordering::Release);
        }
    }

    fn lock_test() -> TestGuard {
        while TEST_MUTEX
            .compare_exchange_weak(false, true, Ordering::Acquire, Ordering::Relaxed)
            .is_err()
        {
            std::thread::yield_now();
        }
        TestGuard
    }

    #[test]
    fn test_wpa_parse_wpa_ie_wrapper() {
        let _guard = lock_test();
        let mut out_ie: WifiWpaIe = WifiWpaIe {
            proto: 0,
            pairwise_cipher: 0,
            group_cipher: 0,
            key_mgmt: 0,
            capabilities: 0,
            num_pmkid: 0,
            pmkid: core::ptr::null(),
            mgmt_group_cipher: 0,
            rsnxe_capa: 0,
        };

        // Null pointer and invalid length guards
        // JUSTIFICATION: Testing C FFI null-guard behavior.
        // SAFETY: Passing null pointers to verify error return -1.
        let ret = unsafe { wpa_parse_wpa_ie_wrapper(core::ptr::null(), 10, &mut out_ie) };
        assert_eq!(ret, -1);

        let valid_rsn_ie: [u8; 22] = [
            0x30, 0x14, // Tag 48 (RSN), Len 20
            0x01, 0x00, // Version 1
            0x00, 0x0F, 0xAC, 0x04, // Group Cipher: CCMP
            0x01, 0x00, // Pairwise Cipher Count: 1
            0x00, 0x0F, 0xAC, 0x04, // Pairwise Cipher: CCMP
            0x01, 0x00, // AKM Count: 1
            0x00, 0x0F, 0xAC, 0x02, // AKM: PSK
            0x00, 0x00, // RSN Capabilities
        ];

        // JUSTIFICATION: Testing C FFI parsing with valid RSN IE.
        // SAFETY: valid_rsn_ie and out_ie are valid stack allocations.
        let ret = unsafe {
            wpa_parse_wpa_ie_wrapper(valid_rsn_ie.as_ptr(), valid_rsn_ie.len(), &mut out_ie)
        };
        assert_eq!(ret, 0);
        assert_eq!(out_ie.proto, ie::WPA_PROTO_RSN);
        assert_eq!(out_ie.pairwise_cipher, 4); // CCMP in public enum is 4
        assert_eq!(out_ie.group_cipher, 4);
        assert_eq!(out_ie.key_mgmt, 2); // PSK
    }

    #[test]
    fn test_supplicant_init_deinit_pmk_caching() {
        let _guard = lock_test();
        driver::reset_mock();

        // JUSTIFICATION: Testing C FFI init function.
        // SAFETY: Global test state is initialized cleanly.
        let ret = unsafe { esp_supplicant_init() };
        assert_eq!(ret, 0);
        driver::with_mock(|m| assert!(m.cb_registered));

        // JUSTIFICATION: Testing C FFI PMK caching toggle.
        // SAFETY: disable_pmk_caching has no unsafe preconditions.
        let ret = unsafe { esp_supplicant_disable_pmk_caching(true) };
        assert_eq!(ret, 0);
        with_state(|s| assert!(s.pmk_caching_disabled));

        // JUSTIFICATION: Testing C FFI deinit function.
        // SAFETY: Safe teardown of supplicant resources.
        let ret = unsafe { esp_supplicant_deinit() };
        assert_eq!(ret, 0);
        driver::with_mock(|m| {
            assert!(m.appie_3_unset);
            assert!(m.appie_4_unset);
            assert!(m.cb_unregistered);
        });
    }

    #[test]
    fn test_supplicant_connect_open_and_callbacks() {
        let _guard = lock_test();
        driver::reset_mock();
        // JUSTIFICATION: Initializing supplicant before connection test.
        // SAFETY: Supplicant init has static lifetime.
        unsafe {
            esp_supplicant_init();
            wpa_attach();
        }

        // Test Open network (authmode == 0)
        driver::with_mock(|m| {
            m.is_wpa = false;
            m.is_rsn = false;
            m.authmode = 0;
        });

        let mut test_bssid = [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF];
        // JUSTIFICATION: Calling connect for open network.
        // SAFETY: test_bssid is a valid 6-byte buffer.
        let ret = unsafe { wpa_sta_connect(test_bssid.as_mut_ptr()) };
        assert_eq!(ret, 0);
        driver::with_mock(|m| {
            assert!(m.appie_3_unset);
            assert!(m.appie_4_unset);
            assert_eq!(m.connected_bssid, test_bssid);
        });

        // Connected and disconnected callbacks
        // JUSTIFICATION: Testing driver connected callback.
        // SAFETY: Passing valid bssid pointer.
        unsafe {
            wpa_sta_connected_cb(test_bssid.as_mut_ptr());
        }
        with_state(|s| assert_eq!(s.bssid, test_bssid));

        // JUSTIFICATION: Testing driver disconnected callback.
        // SAFETY: Disconnect notification takes reason code only.
        unsafe {
            wpa_sta_disconnected_cb(3);
        }

        // JUSTIFICATION: Querying handshake state.
        // SAFETY: Synchronized read of handshake state.
        let in_hs = unsafe { wpa_sta_in_4way_handshake() };
        assert!(!in_hs);
    }

    #[test]
    fn test_eapol_txcb_guards_and_deferral() {
        let _guard = lock_test();
        driver::reset_mock();
        // JUSTIFICATION: Initializing supplicant for TX completion tests.
        // SAFETY: Standard test setup sequence.
        unsafe {
            esp_supplicant_init();
            wpa_attach();
        }

        // Null pointer guard
        // JUSTIFICATION: Testing null pointer guard in eapol_txcb.
        // SAFETY: Verifying null pointer is safely rejected.
        unsafe {
            eapol_txcb(core::ptr::null_mut(), 0, false);
        }

        // Short packet guard (< 99)
        let mut short_buf = [0u8; 50];
        // JUSTIFICATION: Testing short length guard in eapol_txcb.
        // SAFETY: Passing valid stack slice shorter than minimum 99 bytes.
        unsafe {
            eapol_txcb(short_buf.as_mut_ptr(), 50, false);
        }

        // Non-EAPOL-Key packet guard
        let mut non_key_buf = [0u8; 100];
        non_key_buf[1] = 0x01; // EAP-Packet, not EAPOL-Key (0x03)
                               // JUSTIFICATION: Testing packet type check in eapol_txcb.
                               // SAFETY: Passing valid stack buffer of 100 bytes.
        unsafe {
            eapol_txcb(non_key_buf.as_mut_ptr(), 100, false);
        }

        driver::with_mock(|m| {
            assert!(!m.ptk_installed);
            assert!(!m.auth_done_called);
        });
    }

    #[test]
    fn test_crypto_ffi_and_stubs_exercise() {
        let _guard = lock_test();
        // Direct exercise of g_wifi_default_wpa_crypto_funcs
        let crypto_funcs = &crate::crypto_ffi::g_wifi_default_wpa_crypto_funcs;
        assert!(crypto_funcs.aes_128_encrypt.is_some());
        assert!(crypto_funcs.sha256_vector.is_some());
        assert!(crypto_funcs.pbkdf2_sha1.is_some());

        let pbkdf2 = crypto_funcs.pbkdf2_sha1.expect("pbkdf2_sha1 must be set");
        let mut out = [0u8; 32];
        #[cfg(miri)]
        let iters = 2;
        #[cfg(not(miri))]
        let iters = 4096;
        let pass = b"password\0";
        let salt = b"salt";
        // JUSTIFICATION: Exercising pbkdf2_sha1 function pointer.
        // SAFETY: pass is NUL-terminated; salt is valid for 4 bytes; out is 32 bytes.
        let rc = unsafe {
            pbkdf2(
                pass.as_ptr().cast::<core::ffi::c_char>(),
                salt.as_ptr().cast::<core::ffi::c_char>(),
                salt.len() as u32,
                iters,
                out.as_mut_ptr(),
                out.len() as u32,
            )
        };
        assert_eq!(rc, 0);
        #[cfg(miri)]
        {
            let expected_prefix = [
                0xea, 0x6c, 0x01, 0x4d, 0xc7, 0x2d, 0x6f, 0x8c, 0xcd, 0x1e, 0xd9, 0x2a, 0xce, 0x1d,
                0x41, 0xf0, 0xd8, 0xde, 0x89, 0x57,
            ];
            assert_eq!(&out[..20], &expected_prefix);
            assert!(out[20..].iter().any(|&b| b != 0));
        }
        #[cfg(not(miri))]
        assert!(out.iter().any(|&b| b != 0));

        // Exercise hexstr2bin stub
        let hex = b"0123456789abcdef\0";
        let mut bin = [0u8; 8];
        // JUSTIFICATION: Testing hexstr2bin C stub.
        // SAFETY: hex is NUL-terminated; bin is valid for 8 bytes.
        let ret = unsafe {
            crate::stubs::hexstr2bin(
                hex.as_ptr().cast::<core::ffi::c_char>(),
                bin.as_mut_ptr(),
                8,
            )
        };
        assert_eq!(ret, 0);
        assert_eq!(bin, [0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef]);

        // Exercise AP and config stubs
        // JUSTIFICATION: Testing hostap and config stubs for coverage.
        // SAFETY: Stub functions have no-op or stubbed implementations.
        unsafe {
            assert!(crate::stubs::hostap_init().is_null());
            assert!(crate::stubs::hostap_deinit(core::ptr::null_mut()));
            assert!(!crate::stubs::wpa_ap_remove(core::ptr::null_mut()));
            crate::stubs::wpa_config_done();
            crate::stubs::wpa_config_reload();
            crate::stubs::wpa_sta_clear_curr_pmksa();
            assert_eq!(crate::stubs::wpa_michael_mic_failure(0), 0);
            crate::crypto_ffi::ccmp_test_free(core::ptr::null_mut());
        }
    }

    #[test]
    fn test_supplicant_lifecycle_and_m1() {
        let _guard = lock_test();
        driver::reset_mock();
        // JUSTIFICATION: Initializing supplicant for full lifecycle test.
        // SAFETY: Standard test setup sequence.
        unsafe {
            esp_supplicant_init();
            wpa_attach();
        }

        // Configure mock station profile
        driver::with_mock(|m| {
            m.is_wpa = false;
            m.is_rsn = true;
            m.authmode = 3;
            m.pairwise_cipher = 3;
            m.group_cipher = 3;
            m.mock_ssid.len = 8;
            m.mock_ssid.ssid[..8].copy_from_slice(b"TestSSID");
            #[cfg(miri)]
            let pwd = b"0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef\0";
            #[cfg(not(miri))]
            let pwd = b"SecretPass123\0";
            m.mock_pwd[..pwd.len()].copy_from_slice(pwd);
        });

        let mut bssid = [0x00, 0x11, 0x22, 0x33, 0x44, 0x55];
        // JUSTIFICATION: Initiating station connect with valid bssid.
        // SAFETY: bssid is a valid 6-byte stack array.
        let ret = unsafe { wpa_sta_connect(bssid.as_mut_ptr()) };
        assert_eq!(ret, 0);

        driver::with_mock(|m| {
            assert!(m.appie_4_len > 0);
            assert_eq!(m.connected_bssid, bssid);
        });

        // JUSTIFICATION: Verifying AssocResp connected callback preserves handshake.
        // SAFETY: bssid is a valid 6-byte stack array.
        unsafe {
            wpa_sta_connected_cb(bssid.as_mut_ptr());
            assert!(wpa_sta_in_4way_handshake());
        }

        // Construct synthetic M1 EAPOL frame (99 bytes)
        let mut m1 = [0u8; 99];
        m1[0] = 2; // EAPOL Version 2
        m1[1] = 3; // EAPOL-Key
        m1[2] = 0; // Length MSB
        m1[3] = 95; // Length LSB (95 bytes body)
        m1[4] = 2; // Descriptor Type: RSN
        m1[5] = 0x00; // Key Info MSB
        m1[6] = 0x8a; // Key Info LSB: Ack=1, Pairwise=1, Key Descr Version=2
        m1[7] = 0; // Key Length MSB
        m1[8] = 16; // Key Length LSB: 16 bytes (CCMP)
        m1[16] = 1; // Key Replay Counter = 1
        m1[17..49].fill(0xAA); // ANonce

        // JUSTIFICATION: Processing received M1 frame through supplicant.
        // SAFETY: bssid and m1 are valid stack allocations.
        let ret = unsafe { wpa_sm_rx_eapol(bssid.as_mut_ptr(), m1.as_mut_ptr(), m1.len()) };
        assert_eq!(ret, 0);

        // Verify M2 was transmitted
        driver::with_mock(|m| {
            assert_eq!(m.tx_count, 1);
            assert!(m.last_tx_len >= 14 + 99);
            // Verify Ethernet destination is BSSID and source is STA MAC
            assert_eq!(&m.last_tx_buf[0..6], &bssid);
            assert_eq!(&m.last_tx_buf[6..12], &[0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc]);
            // Verify EtherType is 0x888E (EAPOL)
            assert_eq!(&m.last_tx_buf[12..14], &[0x88, 0x8E]);
            // Keys must NOT be installed yet
            assert!(!m.ptk_installed);
            assert!(!m.auth_done_called);
        });

        // Trigger EAPOL TX completion for M2
        let mut last_tx = [0u8; 512];
        let mut last_len = 0;
        driver::with_mock(|m| {
            last_len = m.last_tx_len;
            last_tx[..last_len].copy_from_slice(&m.last_tx_buf[..last_len]);
        });

        // JUSTIFICATION: Invoking driver eapol_txcb with M2 frame.
        // SAFETY: Passing EAPOL payload starting at offset 14.
        unsafe {
            eapol_txcb(last_tx[14..].as_mut_ptr(), last_len - 14, false);
        }

        // M2 TX completion must not install keys or notify auth done
        driver::with_mock(|m| {
            assert!(!m.ptk_installed);
            assert!(!m.auth_done_called);
        });

        // Extract SNonce from transmitted M2 frame (offset 14 + 17 = 31..63)
        let mut snonce = [0u8; 32];
        snonce.copy_from_slice(&last_tx[31..63]);

        // Derive PTK to compute M3 MIC
        let mut pmk = [0u8; 32];
        #[cfg(miri)]
        {
            pmk.copy_from_slice(&[
                0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef, 0x01, 0x23, 0x45, 0x67, 0x89, 0xab,
                0xcd, 0xef, 0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef, 0x01, 0x23, 0x45, 0x67,
                0x89, 0xab, 0xcd, 0xef,
            ]);
        }
        #[cfg(not(miri))]
        {
            let pwd = b"SecretPass123";
            let ssid = b"TestSSID";
            let _ = crate::kdf::pbkdf2_hmac_sha1(pwd, ssid, 4096, &mut pmk);
        }
        let own_mac = [0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc];
        let ptk = crate::kdf::wpa_pmk_to_ptk(
            &pmk,
            b"Pairwise key expansion",
            &own_mac,
            &bssid,
            &snonce,
            &[0xAA; 32],
            false,
        );

        // Construct synthetic M3 frame (99 bytes)
        let mut m3 = [0u8; 99];
        m3[0] = 2; // Version
        m3[1] = 3; // EAPOL-Key
        m3[3] = 95; // Body length
        m3[4] = 2; // RSN
        let m3_ki = 0x01cau16.to_be_bytes(); // Ack, Install, Pairwise, Mic, Secure
        m3[5..7].copy_from_slice(&m3_ki);
        let rep2 = [0, 0, 0, 0, 0, 0, 0, 2];
        m3[9..17].copy_from_slice(&rep2);
        m3[17..49].fill(0xAA); // ANonce
        let mic = crate::crypto::hash::hmac_sha1_vector(&ptk.kck, &[&m3]);
        m3[81..97].copy_from_slice(&mic[..16]);

        // JUSTIFICATION: Processing received M3 frame through supplicant.
        // SAFETY: bssid and m3 are valid stack allocations.
        let ret = unsafe { wpa_sm_rx_eapol(bssid.as_mut_ptr(), m3.as_mut_ptr(), m3.len()) };
        assert_eq!(ret, 0);

        // Verify M4 was transmitted
        driver::with_mock(|m| {
            assert_eq!(m.tx_count, 2);
            assert!(!m.ptk_installed);
            assert!(!m.auth_done_called);
        });

        // Trigger EAPOL TX completion for M4 with failure deferral check
        let mut m4_tx = [0u8; 512];
        let mut m4_len = 0;
        driver::with_mock(|m| {
            m4_len = m.last_tx_len;
            m4_tx[..m4_len].copy_from_slice(&m.last_tx_buf[..m4_len]);
        });

        // M2 TX-done (kd_len != 0, !Secure) must not consume pending_install.
        // JUSTIFICATION: Testing M2 TX completion guard while M4 is pending.
        // SAFETY: Passing valid M2 EAPOL payload starting at offset 14.
        unsafe {
            eapol_txcb(last_tx[14..].as_mut_ptr(), last_len - 14, false);
        }
        driver::with_mock(|m| {
            assert!(!m.ptk_installed);
            assert!(!m.auth_done_called);
        });

        // First test TX failure: should NOT install keys
        // JUSTIFICATION: Invoking driver eapol_txcb for M4 with tx_failure = true.
        // SAFETY: Passing valid EAPOL payload.
        unsafe {
            eapol_txcb(m4_tx[14..].as_mut_ptr(), m4_len - 14, true);
        }
        driver::with_mock(|m| {
            assert!(!m.ptk_installed);
            assert!(!m.auth_done_called);
        });

        // Verify successful TX installs PTK 0x2C and finishes 4-way handshake.
        // Trigger with another M3/M4 round to verify successful installation.
        let rep3 = [0, 0, 0, 0, 0, 0, 0, 3];
        m3[9..17].copy_from_slice(&rep3);
        m3[81..97].fill(0);
        let mic2 = crate::crypto::hash::hmac_sha1_vector(&ptk.kck, &[&m3]);
        m3[81..97].copy_from_slice(&mic2[..16]);

        // JUSTIFICATION: Processing re-transmitted M3 frame.
        // SAFETY: bssid and m3 are valid stack allocations.
        let ret = unsafe { wpa_sm_rx_eapol(bssid.as_mut_ptr(), m3.as_mut_ptr(), m3.len()) };
        assert_eq!(ret, 0);

        let mut m4_tx2 = [0u8; 512];
        let mut m4_len2 = 0;
        driver::with_mock(|m| {
            m4_len2 = m.last_tx_len;
            m4_tx2[..m4_len2].copy_from_slice(&m.last_tx_buf[..m4_len2]);
        });

        // JUSTIFICATION: Invoking driver eapol_txcb for M4 with success.
        // SAFETY: Passing valid EAPOL payload.
        unsafe {
            eapol_txcb(m4_tx2[14..].as_mut_ptr(), m4_len2 - 14, false);
        }
        driver::with_mock(|m| {
            assert!(m.ptk_installed);
            assert_eq!(m.ptk_flag, 0x2C);
            assert!(m.auth_done_called);
        });
        // JUSTIFICATION: Confirming 4-way handshake clears after M4 TX.
        // SAFETY: Synchronized read of global state machine status.
        assert!(!unsafe { wpa_sta_in_4way_handshake() });
    }

    #[test]
    fn test_wpa_sta_connect_non_pmf_ccmp_appie_flag0_and_22b_rsn_ie() {
        let _guard = lock_test();
        driver::reset_mock();
        // JUSTIFICATION: Initializing supplicant for test.
        // SAFETY: Standard test setup sequence.
        unsafe {
            esp_supplicant_init();
            wpa_attach();
        }
        driver::with_mock(|m| {
            m.is_wpa = false;
            m.is_rsn = true;
            m.authmode = 3;
            m.pairwise_cipher = 3;
            m.group_cipher = 3;
            m.mock_ssid.len = 8;
            m.mock_ssid.ssid[..8].copy_from_slice(b"TestSSID");
            #[cfg(miri)]
            let pwd = b"0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef\0";
            #[cfg(not(miri))]
            let pwd = b"SecretPass123\0";
            m.mock_pwd[..pwd.len()].copy_from_slice(pwd);
        });

        let mut bssid = [0x00, 0x11, 0x22, 0x33, 0x44, 0x55];
        // JUSTIFICATION: Initiating station connect with valid bssid.
        // SAFETY: bssid is a valid 6-byte stack array.
        let ret = unsafe { wpa_sta_connect(bssid.as_mut_ptr()) };
        assert_eq!(ret, 0);

        let expected_rsn_ie: [u8; 22] = [
            0x30, 0x14, 0x01, 0x00, 0x00, 0x0f, 0xac, 0x04, 0x01, 0x00, 0x00, 0x0f, 0xac, 0x04,
            0x01, 0x00, 0x00, 0x0f, 0xac, 0x02, 0x00, 0x00,
        ];

        driver::with_mock(|m| {
            assert_eq!(m.appie_4_flag, 0);
            assert_eq!(m.appie_4_len, 22);
            assert!(m.appie_3_unset);
            assert_eq!(&m.appie_4_buf[..22], &expected_rsn_ie);
        });
    }

    #[test]
    fn test_wpa_sta_connect_pmf_optional_and_ap_ie_28b() {
        let _guard = lock_test();
        driver::reset_mock();
        // JUSTIFICATION: Initializing supplicant for test.
        // SAFETY: Standard test setup sequence.
        unsafe {
            esp_supplicant_init();
            wpa_attach();
        }
        let ap_ie_22b: [u8; 22] = [
            0x30, 0x14, 0x01, 0x00, 0x00, 0x0f, 0xac, 0x04, 0x01, 0x00, 0x00, 0x0f, 0xac, 0x04,
            0x01, 0x00, 0x00, 0x0f, 0xac, 0x02, 0x00, 0x00,
        ];
        driver::with_mock(|m| {
            m.is_wpa = false;
            m.is_rsn = true;
            m.authmode = 3;
            m.pairwise_cipher = 3;
            m.group_cipher = 3;
            m.mock_ssid.len = 8;
            m.mock_ssid.ssid[..8].copy_from_slice(b"TestSSID");
            #[cfg(miri)]
            let pwd = b"0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef\0";
            #[cfg(not(miri))]
            let pwd = b"SecretPass123\0";
            m.mock_pwd[..pwd.len()].copy_from_slice(pwd);
            m.pmf_enabled = 1;
            m.pmf_cfg = [1, 0];
            m.mgmt_group_cipher = 6;
            m.ap_rsn_ie[..22].copy_from_slice(&ap_ie_22b);
            m.ap_rsn_ie_len = 22;
        });

        let mut bssid = [0x00, 0x11, 0x22, 0x33, 0x44, 0x55];
        // JUSTIFICATION: Initiating station connect with valid bssid.
        // SAFETY: bssid is a valid 6-byte stack array.
        let ret = unsafe { wpa_sta_connect(bssid.as_mut_ptr()) };
        assert_eq!(ret, 0);

        let expected_mgmt_tail: [u8; 8] = [0x80, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xac, 0x06];

        driver::with_mock(|m| {
            assert_eq!(m.appie_4_flag, 0);
            assert_eq!(m.appie_4_len, 28);
            assert_eq!(&m.appie_4_buf[20..28], &expected_mgmt_tail);
        });

        with_state(|s| {
            assert_eq!(s.assoc_len, 28);
            assert_eq!(&s.assoc_ie[20..28], &expected_mgmt_tail);
        });
    }
}
