//! C ABI bridge and driver integration for ESP32 WPA/WPA2 supplicant.
//!
//! Connects the pure-Rust 4-way handshake state machine to the low-level
//! ESP32 WiFi MAC driver routines.

#![allow(clippy::missing_safety_doc)]

use core::cell::UnsafeCell;
use core::ffi::{c_char, c_int, c_void};
use core::ops::{Deref, DerefMut};
use core::sync::atomic::{AtomicBool, Ordering};

use crate::crypto::hash::zeroize_bytes;
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
///
/// Uses `u8` across C-ABI boolean slots (matching 1-byte C `_Bool`) to avoid
/// Rust `bool` UB if C passes non-`{0, 1}` values.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct WpaFuncs {
    // JUSTIFICATION: C-ABI jump table slot matching ESP-IDF wpa_funcs_t.
    // SAFETY: Function pointer invoked by Wi-Fi driver with static lifetime.
    pub wpa_attach: Option<unsafe extern "C" fn() -> u8>,
    // JUSTIFICATION: C-ABI jump table slot matching ESP-IDF wpa_funcs_t.
    // SAFETY: Function pointer invoked by Wi-Fi driver with static lifetime.
    pub wpa_deattach: Option<unsafe extern "C" fn() -> u8>,
    // JUSTIFICATION: C-ABI jump table slot matching ESP-IDF wpa_funcs_t.
    // SAFETY: Caller supplies a valid 6-byte BSSID pointer.
    pub wpa_sta_connect: Option<unsafe extern "C" fn(*mut u8) -> c_int>,
    // JUSTIFICATION: C-ABI jump table slot matching ESP-IDF wpa_funcs_t.
    // SAFETY: Caller supplies null or a valid 6-byte BSSID pointer.
    pub wpa_sta_connected_cb: Option<unsafe extern "C" fn(*mut u8)>,
    // JUSTIFICATION: C-ABI jump table slot matching ESP-IDF wpa_funcs_t.
    // SAFETY: Stateless notification taking an 8-bit reason code.
    pub wpa_sta_disconnected_cb: Option<unsafe extern "C" fn(u8)>,
    // JUSTIFICATION: C-ABI jump table slot matching ESP-IDF wpa_funcs_t.
    // SAFETY: Caller supplies 6-byte src MAC and `len`-byte EAPOL buffer.
    pub wpa_sm_rx_eapol: Option<unsafe extern "C" fn(*mut u8, *mut u8, usize) -> c_int>,
    // JUSTIFICATION: C-ABI jump table slot matching ESP-IDF wpa_funcs_t.
    // SAFETY: Queries synchronized global handshake state.
    pub wpa_sta_in_4way_handshake: Option<unsafe extern "C" fn() -> u8>,
    // JUSTIFICATION: C-ABI jump table slot matching ESP-IDF wpa_funcs_t.
    // SAFETY: STA-only stub returning null.
    pub hostap_init: Option<unsafe extern "C" fn() -> *mut c_void>,
    // JUSTIFICATION: C-ABI jump table slot matching ESP-IDF wpa_funcs_t.
    // SAFETY: STA-only stub ignoring context pointer.
    pub hostap_deinit: Option<unsafe extern "C" fn(*mut c_void) -> u8>,
    // JUSTIFICATION: C-ABI jump table slot matching ESP-IDF wpa_funcs_t.
    // SAFETY: STA-only stub validating pointers before any write.
    pub hostap_sta_join: Option<
        // JUSTIFICATION: Multi-arg C-ABI callback signature for hostap_sta_join.
        // SAFETY: STA-only stub ignores input buffers and returns 0.
        unsafe extern "C" fn(
            *mut *mut c_void,
            *mut u8,
            *mut u8,
            u8,
            *mut u8,
            u16,
            *mut u8,
            c_int,
            *mut u8,
        ) -> u8,
    >,
    // JUSTIFICATION: C-ABI jump table slot matching ESP-IDF wpa_funcs_t.
    // SAFETY: STA-only stub ignoring BSSID pointer.
    pub wpa_ap_remove: Option<unsafe extern "C" fn(*mut u8) -> u8>,
    // JUSTIFICATION: C-ABI jump table slot matching ESP-IDF wpa_funcs_t.
    // SAFETY: STA-only stub returning null.
    pub wpa_ap_get_wpa_ie: Option<unsafe extern "C" fn(*mut usize) -> *mut u8>,
    // JUSTIFICATION: C-ABI jump table slot matching ESP-IDF wpa_funcs_t.
    // SAFETY: STA-only stub returning 0.
    pub wpa_ap_rx_eapol: Option<
        // JUSTIFICATION: C-ABI callback signature for wpa_ap_rx_eapol.
        // SAFETY: STA-only stub ignoring frame pointers.
        unsafe extern "C" fn(*mut c_void, *mut c_void, *mut u8, usize) -> u8,
    >,
    // JUSTIFICATION: C-ABI jump table slot matching ESP-IDF wpa_funcs_t.
    // SAFETY: STA-only stub ignoring pointer arguments.
    pub wpa_ap_get_peer_spp_msg: Option<unsafe extern "C" fn(*mut c_void, *mut u8, *mut u8)>,
    // JUSTIFICATION: C-ABI jump table slot matching ESP-IDF wpa_funcs_t.
    // SAFETY: STA-only stub returning null.
    pub wpa_config_parse_string: Option<
        // JUSTIFICATION: C-ABI callback signature for wpa_config_parse_string.
        // SAFETY: STA-only stub returning null.
        unsafe extern "C" fn(*const c_char, *mut usize) -> *mut c_char,
    >,
    // JUSTIFICATION: C-ABI jump table slot matching ESP-IDF wpa_funcs_t.
    // SAFETY: Validates non-null pointers before reading IE or writing output.
    pub wpa_parse_wpa_ie: Option<unsafe extern "C" fn(*const u8, usize, *mut WifiWpaIe) -> c_int>,
    // JUSTIFICATION: C-ABI jump table slot matching ESP-IDF wpa_funcs_t.
    // SAFETY: STA-only stub ignoring config parameters.
    pub wpa_config_bss: Option<unsafe extern "C" fn(*const c_char, c_int) -> c_int>,
    // JUSTIFICATION: C-ABI jump table slot matching ESP-IDF wpa_funcs_t.
    // SAFETY: STA-only stub taking 16-bit unicast flag.
    pub wpa_michael_mic_failure: Option<unsafe extern "C" fn(u16) -> c_int>,
    // JUSTIFICATION: C-ABI jump table slot matching ESP-IDF wpa_funcs_t.
    // SAFETY: STA-only no-op stub returning 0.
    pub wpa_config_set_unsafe: Option<unsafe extern "C" fn() -> c_int>,
    // JUSTIFICATION: C-ABI jump table slot matching ESP-IDF wpa_funcs_t.
    // SAFETY: STA-only no-op stub returning 0.
    pub wpa_config_set_int: Option<unsafe extern "C" fn() -> c_int>,
    // JUSTIFICATION: C-ABI jump table slot matching ESP-IDF wpa_funcs_t.
    // SAFETY: STA-only no-op stub.
    pub wpa_sta_rx_eapol_retry: Option<unsafe extern "C" fn()>,
    // JUSTIFICATION: C-ABI jump table slot matching ESP-IDF wpa_funcs_t.
    // SAFETY: STA-only no-op stub.
    pub wpa_sta_rx_mgmt: Option<unsafe extern "C" fn()>,
    // JUSTIFICATION: C-ABI jump table slot matching ESP-IDF wpa_funcs_t.
    // SAFETY: STA-only no-op stub.
    pub wpa_config_done: Option<unsafe extern "C" fn()>,
    // JUSTIFICATION: C-ABI jump table slot matching ESP-IDF wpa_funcs_t.
    // SAFETY: STA-only no-op stub returning 0.
    pub wpa_config_set_array: Option<unsafe extern "C" fn() -> c_int>,
    // JUSTIFICATION: C-ABI jump table slot matching ESP-IDF wpa_funcs_t.
    // SAFETY: STA-only no-op stub returning 0.
    pub wpa_config_set_byte_array: Option<unsafe extern "C" fn() -> c_int>,
    // JUSTIFICATION: C-ABI jump table slot matching ESP-IDF wpa_funcs_t.
    // SAFETY: STA-only no-op stub.
    pub wpa_sta_clear_curr_pmksa: Option<unsafe extern "C" fn()>,
    // JUSTIFICATION: C-ABI jump table slot matching ESP-IDF wpa_funcs_t.
    // SAFETY: STA-only no-op stub.
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

/// Encapsulated spinlock-protected mutex with RAII lock release (`T: Send`).
///
/// Guarantees that the lock is always released on unwind/panic via `Drop` and
/// never exposes the inner `UnsafeCell` outside the guard.
pub(crate) struct SpinMutex<T> {
    locked: AtomicBool,
    inner: UnsafeCell<T>,
}

// JUSTIFICATION: Exclusive access to `inner` is serialized by `locked` using
// Acquire/Release ordering; `T: Send` permits cross-thread mutable access.
// SAFETY: Only a single `SpinMutexGuard` can exist at any time.
unsafe impl<T: Send> Sync for SpinMutex<T> {}

// JUSTIFICATION: Transferring ownership of `SpinMutex<T>` across threads is
// safe whenever `T: Send`.
// SAFETY: Exclusive ownership guarantees no concurrent access during move.
unsafe impl<T: Send> Send for SpinMutex<T> {}

pub(crate) struct SpinMutexGuard<'a, T> {
    mutex: &'a SpinMutex<T>,
    #[cfg(target_arch = "xtensa")]
    prev_ps: u32,
}

impl<T> SpinMutex<T> {
    pub(crate) const fn new(val: T) -> Self {
        Self {
            locked: AtomicBool::new(false),
            inner: UnsafeCell::new(val),
        }
    }

    pub(crate) fn lock(&self) -> SpinMutexGuard<'_, T> {
        #[cfg(target_arch = "xtensa")]
        let prev_ps = {
            let ps: u32;
            // JUSTIFICATION: Masking local Xtensa interrupts before acquiring
            // the supplicant spinlock prevents same-core ISR/callback
            // preemption deadlocks (M-065). Omitting `nomem` keeps LLVM from
            // reordering lock memory ops outside the masked critical section.
            // SAFETY: `rsil` atomically reads PS and sets PS.INTLEVEL to 15;
            // `SpinMutexGuard::drop` restores the exact prior PS value.
            unsafe {
                core::arch::asm!("rsil {0}, 15", out(reg) ps, options(nostack));
            }
            ps
        };

        while self
            .locked
            .compare_exchange_weak(false, true, Ordering::Acquire, Ordering::Relaxed)
            .is_err()
        {
            #[cfg(test)]
            std::thread::yield_now();
            #[cfg(not(test))]
            core::hint::spin_loop();
        }
        SpinMutexGuard {
            mutex: self,
            #[cfg(target_arch = "xtensa")]
            prev_ps,
        }
    }
}

impl<T> Deref for SpinMutexGuard<'_, T> {
    type Target = T;

    fn deref(&self) -> &T {
        // JUSTIFICATION: Guard holds exclusive lock on SpinMutex.
        // SAFETY: `locked == true` with Acquire ordering ensures exclusive access.
        unsafe { &*self.mutex.inner.get() }
    }
}

impl<T> DerefMut for SpinMutexGuard<'_, T> {
    fn deref_mut(&mut self) -> &mut T {
        // JUSTIFICATION: Guard holds exclusive lock on SpinMutex.
        // SAFETY: `locked` with Acquire ordering ensures exclusive mut access.
        unsafe { &mut *self.mutex.inner.get() }
    }
}

impl<T> Drop for SpinMutexGuard<'_, T> {
    fn drop(&mut self) {
        self.mutex.locked.store(false, Ordering::Release);
        #[cfg(target_arch = "xtensa")]
        {
            // JUSTIFICATION: Restores the Xtensa processor status interrupt
            // mask captured when acquiring `SpinMutexGuard`; omitting `nomem`
            // prevents LLVM from hoisting unmasking above the lock release.
            // SAFETY: `self.prev_ps` came directly from `rsil` on this core.
            unsafe {
                core::arch::asm!(
                    "wsr.ps {0}",
                    "rsync",
                    in(reg) self.prev_ps,
                    options(nostack)
                );
            }
        }
    }
}

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

static STATE: SpinMutex<SupplicantState> = SpinMutex::new(SupplicantState::new());

fn with_state<R, F: FnOnce(&mut SupplicantState) -> R>(f: F) -> R {
    let mut guard = STATE.lock();
    f(&mut guard)
}

// JUSTIFICATION: C-ABI wrapper translating Rust bool stub to u8 for WpaFuncs.
// SAFETY: Delegates directly to STA-only stub without dereferencing pointers.
unsafe extern "C" fn ffi_hostap_deinit(data: *mut c_void) -> u8 {
    // JUSTIFICATION: Invoking STA-only stub.
    // SAFETY: `stubs::hostap_deinit` ignores `data`.
    u8::from(unsafe { crate::stubs::hostap_deinit(data) })
}

// JUSTIFICATION: C-ABI wrapper eliminating raw `*mut bool` across C FFI.
// SAFETY: Passes a stack-allocated Rust `bool` to the STA stub and writes `u8`.
#[allow(clippy::too_many_arguments)]
unsafe extern "C" fn ffi_hostap_sta_join(
    sm: *mut *mut c_void,
    bssid: *mut u8,
    wpa_ie: *mut u8,
    wpa_ie_len: u8,
    rsnxe: *mut u8,
    rsnxe_len: u16,
    pmf_enable: *mut u8,
    subtype: c_int,
    pairwise_cipher: *mut u8,
) -> u8 {
    let mut pmf = false;
    // JUSTIFICATION: Invoking STA-only stub with valid local bool pointer.
    // SAFETY: `&mut pmf` is a valid aligned Rust bool; stub ignores other pointers.
    let ok = unsafe {
        crate::stubs::hostap_sta_join(
            sm,
            bssid,
            wpa_ie,
            wpa_ie_len,
            rsnxe,
            rsnxe_len,
            &mut pmf,
            subtype,
            pairwise_cipher,
        )
    };
    if !pmf_enable.is_null() {
        // JUSTIFICATION: Writing u8 boolean result to non-null C out-pointer.
        // SAFETY: `pmf_enable` verified non-null.
        unsafe {
            core::ptr::write(pmf_enable, u8::from(pmf));
        }
    }
    u8::from(ok)
}

// JUSTIFICATION: C-ABI wrapper translating Rust bool stub to u8 for WpaFuncs.
// SAFETY: Delegates directly to STA-only stub without dereferencing pointers.
unsafe extern "C" fn ffi_wpa_ap_remove(bssid: *mut u8) -> u8 {
    // JUSTIFICATION: Invoking STA-only stub.
    // SAFETY: `stubs::wpa_ap_remove` ignores `bssid`.
    u8::from(unsafe { crate::stubs::wpa_ap_remove(bssid) })
}

// JUSTIFICATION: C-ABI wrapper translating Rust bool stub to u8 for WpaFuncs.
// SAFETY: Delegates directly to STA-only stub without dereferencing pointers.
unsafe extern "C" fn ffi_wpa_ap_rx_eapol(
    hapd: *mut c_void,
    sm: *mut c_void,
    data: *mut u8,
    len: usize,
) -> u8 {
    // JUSTIFICATION: Invoking STA-only stub.
    // SAFETY: `stubs::wpa_ap_rx_eapol` ignores all pointer parameters.
    u8::from(unsafe { crate::stubs::wpa_ap_rx_eapol(hapd, sm, data, len) })
}

// JUSTIFICATION: C-ABI wrapper eliminating raw `*mut bool` across C FFI.
// SAFETY: Passes stack bool pointers to STA stub and writes u8 if non-null.
unsafe extern "C" fn ffi_wpa_ap_get_peer_spp_msg(sm: *mut c_void, cap: *mut u8, req: *mut u8) {
    let mut cap_b = false;
    let mut req_b = false;
    // JUSTIFICATION: Invoking STA-only stub with valid stack bool pointers.
    // SAFETY: `&mut cap_b` and `&mut req_b` are valid aligned pointers.
    unsafe {
        crate::stubs::wpa_ap_get_peer_spp_msg(sm, &mut cap_b, &mut req_b);
    }
    if !cap.is_null() {
        // JUSTIFICATION: Writing u8 boolean output to caller pointer.
        // SAFETY: `cap` verified non-null.
        unsafe {
            core::ptr::write(cap, u8::from(cap_b));
        }
    }
    if !req.is_null() {
        // JUSTIFICATION: Writing u8 boolean output to caller pointer.
        // SAFETY: `req` verified non-null.
        unsafe {
            core::ptr::write(req, u8::from(req_b));
        }
    }
}

// JUSTIFICATION: Safe non-null stub for unused WpaFuncs BSS config slot.
// SAFETY: Stateless no-op returning 0; never dereferences parameters.
unsafe extern "C" fn wpa_config_bss_stub(_val: *const c_char, _len: c_int) -> c_int {
    0
}

// JUSTIFICATION: Safe non-null stub for unused WpaFuncs integer config slots.
// SAFETY: Stateless no-op returning 0; performs no memory operations.
unsafe extern "C" fn wpa_noop_c_int_stub() -> c_int {
    0
}

// JUSTIFICATION: Safe non-null stub for unused WpaFuncs void callback slots.
// SAFETY: Stateless no-op; performs no memory operations.
unsafe extern "C" fn wpa_noop_void_stub() {}

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
    hostap_deinit: Some(ffi_hostap_deinit),
    hostap_sta_join: Some(ffi_hostap_sta_join),
    wpa_ap_remove: Some(ffi_wpa_ap_remove),
    wpa_ap_get_wpa_ie: Some(crate::stubs::wpa_ap_get_wpa_ie),
    wpa_ap_rx_eapol: Some(ffi_wpa_ap_rx_eapol),
    wpa_ap_get_peer_spp_msg: Some(ffi_wpa_ap_get_peer_spp_msg),
    wpa_config_parse_string: Some(crate::stubs::wpa_config_parse_string),
    wpa_parse_wpa_ie: Some(wpa_parse_wpa_ie_wrapper),
    wpa_config_bss: Some(wpa_config_bss_stub),
    wpa_michael_mic_failure: Some(crate::stubs::wpa_michael_mic_failure),
    wpa_config_set_unsafe: Some(wpa_noop_c_int_stub),
    wpa_config_set_int: Some(wpa_noop_c_int_stub),
    wpa_sta_rx_eapol_retry: Some(wpa_noop_void_stub),
    wpa_sta_rx_mgmt: Some(wpa_noop_void_stub),
    wpa_config_done: Some(crate::stubs::wpa_config_done),
    wpa_config_set_array: Some(wpa_noop_c_int_stub),
    wpa_config_set_byte_array: Some(wpa_noop_c_int_stub),
    wpa_sta_clear_curr_pmksa: Some(crate::stubs::wpa_sta_clear_curr_pmksa),
    wpa_config_reload: Some(crate::stubs::wpa_config_reload),
};

// JUSTIFICATION: C callback for supplicant attachment.
// SAFETY: Operates on global supplicant state with synchronized access.
pub unsafe extern "C" fn wpa_attach() -> u8 {
    with_state(|state| {
        state.sm.reset();
        state.attached = true;
    });
    // JUSTIFICATION: Registering EAPOL TX completion callback with driver.
    // SAFETY: eapol_txcb has static lifetime and matches required C signature.
    unsafe {
        driver::esp_wifi_register_eapol_txdonecb_internal(Some(eapol_txcb));
    }
    1
}

// JUSTIFICATION: C callback for supplicant detachment.
// SAFETY: Unregisters TX callback before clearing synchronized global state.
pub unsafe extern "C" fn wpa_deattach() -> u8 {
    // JUSTIFICATION: Unregistering EAPOL TX completion callback with driver.
    // SAFETY: Passing None unregisters the callback safely.
    unsafe {
        driver::esp_wifi_register_eapol_txdonecb_internal(None);
    }
    with_state(|state| {
        state.sm.reset();
        state.attached = false;
    });
    1
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
pub unsafe extern "C" fn wpa_sta_in_4way_handshake() -> u8 {
    u8::from(with_state(|state| state.sm.in_4way_handshake()))
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
    // SAFETY: Driver profile query functions have no preconditions; u8 avoids
    // raw bool UB across C FFI.
    let (is_wpa, is_rsn, authmode) = unsafe {
        (
            driver::esp_wifi_sta_prof_is_wpa_internal() != 0,
            driver::esp_wifi_sta_prof_is_rsn_internal() != 0,
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
    // SAFETY: Pointer checked non-null; read_unaligned avoids alignment UB;
    // ssid_len bounded to 0..=32.
    unsafe {
        let ssid_ptr = driver::esp_wifi_sta_get_prof_ssid_internal();
        if ssid_ptr.is_null() {
            return -1;
        }
        let raw_len = core::ptr::read_unaligned(core::ptr::addr_of!((*ssid_ptr).len));
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
    let ie_tag = if proto == ie::WPA_PROTO_RSN {
        0x30u8
    } else {
        0xDDu8
    };
    // JUSTIFICATION: Fetching cached AP RSN/WPA IE from driver BSS entry.
    // SAFETY: Pointer checked non-null and IE TLV length bounded to <= 128 bytes.
    unsafe {
        let ie_ptr = driver::esp_wifi_sta_get_ie(bssid_arr.as_ptr(), ie_tag);
        if !ie_ptr.is_null() && *ie_ptr == ie_tag {
            let tlv_len = 2 + usize::from(*ie_ptr.add(1));
            if tlv_len <= ap_ie_buf.len() {
                let src = core::slice::from_raw_parts(ie_ptr, tlv_len);
                ap_ie_buf[..tlv_len].copy_from_slice(src);
                ap_ie_len = tlv_len;
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

    // Validate AP IE and derive/decode PMK OUTSIDE the interrupt-masked
    // `STATE` spinlock so 4096-iteration PBKDF2 (~20-40ms) never blocks ISRs
    // or triggers watchdog resets (M-065).
    if ap_ie_len == 0 {
        zeroize_bytes(&mut pass_buf);
        return -1;
    }
    let Ok(ap_parsed) = ie::parse_wpa_or_rsn_ie(&ap_ie_buf[..ap_ie_len]) else {
        zeroize_bytes(&mut pass_buf);
        return -1;
    };
    if ap_parsed.proto != proto {
        zeroize_bytes(&mut pass_buf);
        return -1;
    }

    let mut derived_pmk = [0u8; 32];
    let has_new_pmk = match crate::handshake::derive_or_decode_pmk(
        &ssid_buf[..ssid_len],
        &pass_buf[..pass_len],
        &mut derived_pmk,
    ) {
        Ok(v) => v,
        Err(_) => {
            zeroize_bytes(&mut pass_buf);
            zeroize_bytes(&mut derived_pmk);
            return -1;
        }
    };
    zeroize_bytes(&mut pass_buf);

    let mut assoc_buf_copy = [0u8; 128];
    let pmk_opt = if has_new_pmk {
        Some(&derived_pmk)
    } else {
        None
    };
    let connect_res = with_state(|state| {
        state.own_mac = own_mac;
        state.bssid = bssid_arr;
        match state.sm.start_connect_with_pmk(
            own_mac,
            bssid_arr,
            pmk_opt,
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
    zeroize_bytes(&mut derived_pmk);

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
    zeroize_bytes(&mut snonce_rand);

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

// JUSTIFICATION: C callback for driver EAPOL TX completion using `u8` instead
// of raw Rust `bool` to prevent UB on non-0/1 C byte values.
// SAFETY: Validates frame buffer pointer and length before inspection.
pub unsafe extern "C" fn eapol_txcb(eapol_payload: *mut u8, len: usize, tx_failure: u8) {
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
        let act = state.sm.on_eapol_tx_complete(tx_failure != 0);
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
            let mut igtk_struct = WifiWpaIgtk {
                keyid: pi.igtk_idx.to_le_bytes(),
                pn: pi.igtk_pn,
                igtk: pi.igtk,
            };
            // JUSTIFICATION: Configuring Integrity GTK in WiFi driver.
            // SAFETY: Passing valid pointer to stack-allocated WifiWpaIgtk.
            unsafe {
                driver::esp_wifi_set_igtk_internal(0, &igtk_struct);
            }
            zeroize_bytes(&mut igtk_struct.igtk);
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
// SAFETY: Pointers are verified non-null; uses write_unaligned for C out-ptr.
pub unsafe extern "C" fn wpa_parse_wpa_ie_wrapper(
    wpa_ie: *const u8,
    wpa_ie_len: usize,
    data: *mut WifiWpaIe,
) -> c_int {
    if wpa_ie.is_null() || data.is_null() || wpa_ie_len == 0 {
        return -1;
    }
    let mut out = WifiWpaIe {
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
    // JUSTIFICATION: Creating immutable slice from IE input buffer.
    // SAFETY: wpa_ie is non-null and valid for wpa_ie_len bytes.
    let ie_slice = unsafe { core::slice::from_raw_parts(wpa_ie, wpa_ie_len) };
    match ie::parse_wpa_or_rsn_ie(ie_slice) {
        Ok(parsed) => {
            out.proto = parsed.proto;
            out.pairwise_cipher = ie::cipher_type_map_supp_to_public(parsed.pairwise_cipher);
            out.group_cipher = ie::cipher_type_map_supp_to_public(parsed.group_cipher);
            out.key_mgmt = parsed.key_mgmt as c_int;
            out.capabilities = i32::from(parsed.capabilities);
            out.mgmt_group_cipher = ie::cipher_type_map_supp_to_public(parsed.mgmt_group_cipher);
            out.rsnxe_capa = parsed.rsnxe_capa;
            if parsed.has_pmkid && parsed.first_pmkid_offset < wpa_ie_len {
                out.num_pmkid = 1;
                // JUSTIFICATION: Computing pointer into caller's IE buffer for PMKID.
                // SAFETY: `parsed.first_pmkid_offset < wpa_ie_len` keeps pointer in bounds.
                out.pmkid = unsafe { wpa_ie.add(parsed.first_pmkid_offset) };
            }
            // JUSTIFICATION: Writing parsed WifiWpaIe to caller buffer safely.
            // SAFETY: `data` verified non-null; write_unaligned avoids alignment UB.
            unsafe {
                core::ptr::write_unaligned(data, out);
            }
            0
        }
        Err(_) => {
            // JUSTIFICATION: Zero-initializing caller WifiWpaIe on parse failure.
            // SAFETY: `data` verified non-null; write_unaligned avoids alignment UB.
            unsafe {
                core::ptr::write_unaligned(data, out);
            }
            -1
        }
    }
}

// JUSTIFICATION: Inbound C FFI function initializing the WPA supplicant.
// SAFETY: Resets synchronized state before registering immutable jump table.
#[no_mangle]
pub unsafe extern "C" fn esp_supplicant_init() -> c_int {
    with_state(|state| {
        state.sm.reset();
        state.attached = false;
        state.pmk_caching_disabled = false;
        state.assoc_len = 0;
        zeroize_bytes(&mut state.assoc_ie);
        state.own_mac = [0; 6];
        state.bssid = [0; 6];
    });
    // JUSTIFICATION: Registering immutable WPA callback jump table with driver.
    // SAFETY: WPA_FUNCS is an immutable static with 'static lifetime.
    let ret = unsafe { driver::esp_wifi_register_wpa_cb_internal(&WPA_FUNCS as *const WpaFuncs) };
    if ret == 0 {
        0
    } else {
        -1
    }
}

// JUSTIFICATION: Inbound C FFI function deinitializing the WPA supplicant.
// SAFETY: Unregisters all driver callbacks first to quiesce concurrent calls
// before zeroizing supplicant state and unsetting App IEs.
#[no_mangle]
pub unsafe extern "C" fn esp_supplicant_deinit() -> c_int {
    // 1. Unregister driver WPA callbacks first so no new callbacks can enter.
    // JUSTIFICATION: Quiescing driver WPA callbacks before state teardown.
    // SAFETY: Reverts callback registration in driver.
    let ret = unsafe { driver::esp_wifi_unregister_wpa_cb_internal() };
    // 2. Unregister EAPOL TX-done callback and detach state machine.
    // JUSTIFICATION: Calling wpa_deattach to unregister EAPOL TX callback.
    // SAFETY: Internal function with static callback cleanup.
    unsafe {
        wpa_deattach();
    }
    // 3. Clear and volatile-zeroize supplicant state after callbacks quiesce.
    with_state(|state| {
        state.sm.reset();
        state.sm.clear_pmk();
        state.attached = false;
        state.assoc_len = 0;
        zeroize_bytes(&mut state.assoc_ie);
        state.own_mac = [0; 6];
        state.bssid = [0; 6];
    });
    // 4. Unset application IEs (types 0..15) after state teardown.
    // JUSTIFICATION: Unset application IEs (types 0..15) on supplicant deinit.
    // SAFETY: Driver functions handle unsetting IEs cleanly.
    unsafe {
        for t in 0..15u8 {
            driver::esp_wifi_unset_appie_internal(t);
        }
    }
    if ret == 0 {
        0
    } else {
        -1
    }
}

// JUSTIFICATION: Inbound C FFI function to disable or enable PMK caching
// using `u8` (C `_Bool` ABI) instead of raw Rust `bool`.
// SAFETY: Updates PMK caching flag in synchronized supplicant state.
#[no_mangle]
pub unsafe extern "C" fn esp_supplicant_disable_pmk_caching(disable: u8) -> c_int {
    let disable_flag = disable != 0;
    with_state(|state| {
        state.pmk_caching_disabled = disable_flag;
        if disable_flag {
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

    // JUSTIFICATION: ESP-IDF Wi-Fi driver internal C symbols required for
    // supplicant integration on Xtensa hardware targets.
    // SAFETY: Signatures match ESP-IDF esp_wifi_driver.h C ABI with `u8` for
    // C `_Bool` params/returns; pointer arguments are validated at call sites.
    extern "C" {
        pub fn __esp_radio_esp_fill_random(dst: *mut u8, len: u32);
        pub fn esp_wifi_internal_tx(wifi_if: c_int, buffer: *mut c_void, len: u16) -> c_int;
        pub fn esp_wifi_sta_prof_is_wpa_internal() -> u8;
        pub fn esp_wifi_sta_prof_is_rsn_internal() -> u8;
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
            // JUSTIFICATION: C-ABI callback pointer for EAPOL TX completion.
            // SAFETY: Driver invokes registered callback with valid payload ptr.
            fn_cb: Option<unsafe extern "C" fn(*mut u8, usize, u8)>,
        ) -> c_int;
        pub fn esp_wifi_register_wpa_cb_internal(cb: *const WpaFuncs) -> c_int;
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
    pub(super) unsafe extern "C" fn esp_wifi_sta_prof_is_wpa_internal() -> u8 {
        0
    }

    /// # Safety
    ///
    /// Host link stub; does not dereference pointers or access hardware.
    // JUSTIFICATION: Host driver link stub for non-Xtensa builds.
    // SAFETY: Stateless host stub; does not perform unsafe operations.
    pub(super) unsafe extern "C" fn esp_wifi_sta_prof_is_rsn_internal() -> u8 {
        0
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
        // JUSTIFICATION: C-ABI callback pointer type for host link stub.
        // SAFETY: Unused parameter; never invoked in host stub.
        _fn_cb: Option<unsafe extern "C" fn(*mut u8, usize, u8)>,
    ) -> c_int {
        -1
    }

    /// # Safety
    ///
    /// Host link stub; does not dereference pointers or access hardware.
    // JUSTIFICATION: Host driver link stub for non-Xtensa builds.
    // SAFETY: Stateless host stub; never dereferences pointer arguments.
    pub(super) unsafe extern "C" fn esp_wifi_register_wpa_cb_internal(
        _cb: *const WpaFuncs,
    ) -> c_int {
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
        // JUSTIFICATION: Mock storage for EAPOL TX callback pointer in host tests.
        // SAFETY: Only stores function pointers with 'static lifetime.
        pub txdonecb: Option<unsafe extern "C" fn(*mut u8, usize, u8)>,
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
        pub deinit_log: [u8; 8],
        pub deinit_log_len: usize,
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
                deinit_log: [0; 8],
                deinit_log_len: 0,
            }
        }

        fn push_event(&mut self, ev: u8) {
            if self.deinit_log_len < self.deinit_log.len() {
                self.deinit_log[self.deinit_log_len] = ev;
                self.deinit_log_len += 1;
            }
        }
    }

    static MOCK: SpinMutex<MockDriver> = SpinMutex::new(MockDriver::new());

    pub fn with_mock<R, F: FnOnce(&mut MockDriver) -> R>(f: F) -> R {
        let mut guard = MOCK.lock();
        f(&mut guard)
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
    pub unsafe extern "C" fn esp_wifi_sta_prof_is_wpa_internal() -> u8 {
        u8::from(with_mock(|m| m.is_wpa))
    }

    // JUSTIFICATION: Mock profile query for host unit tests.
    // SAFETY: Operates on synchronized mock driver state without pointers.
    pub unsafe extern "C" fn esp_wifi_sta_prof_is_rsn_internal() -> u8 {
        u8::from(with_mock(|m| m.is_rsn))
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
        if bssid.is_null() || (ie_type != 0x30 && ie_type != 0xDD) {
            return core::ptr::null();
        }
        with_mock(|m| {
            if m.ap_rsn_ie_len > 0 && m.ap_rsn_ie[0] == ie_type {
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
            if ie_type == 0 {
                m.push_event(3);
            }
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
    #[allow(clippy::too_many_arguments)]
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
        // JUSTIFICATION: C-ABI callback pointer type for mock registration.
        // SAFETY: Stored in MockDriver without invocation.
        fn_cb: Option<unsafe extern "C" fn(*mut u8, usize, u8)>,
    ) -> c_int {
        with_mock(|m| {
            if fn_cb.is_none() {
                m.push_event(2);
            }
            m.txdonecb_registered = fn_cb.is_some();
            m.txdonecb = fn_cb;
        });
        0
    }

    // JUSTIFICATION: Mock WPA callback registration handler for host tests.
    // SAFETY: Checks cb pointer for non-null without dereference.
    pub unsafe extern "C" fn esp_wifi_register_wpa_cb_internal(cb: *const WpaFuncs) -> c_int {
        with_mock(|m| {
            m.cb_registered = !cb.is_null();
        });
        0
    }

    // JUSTIFICATION: Mock WPA callback unregistration handler for host tests.
    // SAFETY: Operates on synchronized mock driver state without pointers.
    pub unsafe extern "C" fn esp_wifi_unregister_wpa_cb_internal() -> c_int {
        with_mock(|m| {
            m.push_event(1);
            m.cb_unregistered = true;
        });
        0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::crypto::aes::aes_wrap;

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

    const AP_RSN_22B: [u8; 22] = [
        0x30, 0x14, 0x01, 0x00, 0x00, 0x0f, 0xac, 0x04, 0x01, 0x00, 0x00, 0x0f, 0xac, 0x04, 0x01,
        0x00, 0x00, 0x0f, 0xac, 0x02, 0x00, 0x00,
    ];

    fn build_encrypted_m3(
        rep: [u8; 8],
        anonce: [u8; 32],
        ap_rsn: &[u8],
        gtk: &[u8; 16],
        ptk: &crate::kdf::Ptk,
    ) -> ([u8; 160], usize) {
        let mut plain = [0u8; 48];
        plain[..ap_rsn.len()].copy_from_slice(ap_rsn);
        let off = ap_rsn.len();
        plain[off] = 0xDD;
        plain[off + 1] = 22;
        plain[off + 2..off + 5].copy_from_slice(&[0x00, 0x0F, 0xAC]);
        plain[off + 5] = 0x01;
        plain[off + 6] = 0x01;
        plain[off + 7] = 0x00;
        plain[off + 8..off + 24].copy_from_slice(gtk);
        plain[off + 24] = 0xDD;
        let mut wrapped = [0u8; 56];
        aes_wrap(&ptk.kek, &plain, &mut wrapped).expect("aes_wrap");

        let total = 99 + wrapped.len();
        let mut m3 = [0u8; 160];
        m3[0] = 2;
        m3[1] = 3;
        m3[2..4].copy_from_slice(&((total - 4) as u16).to_be_bytes());
        m3[4] = 2;
        m3[5..7].copy_from_slice(&0x13cau16.to_be_bytes());
        m3[9..17].copy_from_slice(&rep);
        m3[17..49].copy_from_slice(&anonce);
        m3[97..99].copy_from_slice(&(wrapped.len() as u16).to_be_bytes());
        m3[99..total].copy_from_slice(&wrapped);
        let mic = crate::crypto::hash::hmac_sha1_vector(&ptk.kck, &[&m3[..total]]);
        m3[81..97].copy_from_slice(&mic[..16]);
        (m3, total)
    }

    #[test]
    fn test_wpa_parse_wpa_ie_wrapper() {
        let _guard = lock_test();
        let mut out_ie = WifiWpaIe {
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

        // JUSTIFICATION: Testing C FFI null-guard behavior.
        // SAFETY: Passing null pointers to verify error return -1.
        let ret = unsafe { wpa_parse_wpa_ie_wrapper(core::ptr::null(), 10, &mut out_ie) };
        assert_eq!(ret, -1);

        let ie_ptr = AP_RSN_22B.as_ptr();
        // JUSTIFICATION: Testing C FFI parsing with valid RSN IE.
        // SAFETY: AP_RSN_22B and out_ie are valid stack allocations.
        let ret = unsafe { wpa_parse_wpa_ie_wrapper(ie_ptr, AP_RSN_22B.len(), &mut out_ie) };
        assert_eq!(ret, 0);
        assert_eq!(out_ie.proto, ie::WPA_PROTO_RSN);
        assert_eq!(out_ie.pairwise_cipher, 4);
        assert_eq!(out_ie.group_cipher, 4);
        assert_eq!(out_ie.key_mgmt, 2);
    }

    #[test]
    fn test_supplicant_init_deinit_ordering_and_pmk_caching() {
        let _guard = lock_test();
        driver::reset_mock();

        // JUSTIFICATION: Testing C FFI init function.
        // SAFETY: Global test state is initialized cleanly.
        let ret = unsafe { esp_supplicant_init() };
        assert_eq!(ret, 0);
        driver::with_mock(|m| assert!(m.cb_registered));

        // Non-0/1 u8 value (e.g. 2) must be treated as true without UB (C2-043).
        // JUSTIFICATION: Testing C FFI PMK caching toggle with u8 value 2.
        // SAFETY: disable_pmk_caching accepts u8 across C ABI.
        let ret = unsafe { esp_supplicant_disable_pmk_caching(2) };
        assert_eq!(ret, 0);
        with_state(|s| assert!(s.pmk_caching_disabled));

        // JUSTIFICATION: Testing C FFI deinit function and teardown ordering.
        // SAFETY: Safe teardown of supplicant resources.
        let ret = unsafe { esp_supplicant_deinit() };
        assert_eq!(ret, 0);
        driver::with_mock(|m| {
            assert!(m.appie_3_unset);
            assert!(m.appie_4_unset);
            assert!(m.cb_unregistered);
            // Event 1 (unregister_wpa_cb) -> 2 (unregister txdonecb) -> 3 (unset appie 0).
            assert_eq!(&m.deinit_log[..3], &[1, 2, 3]);
        });
    }

    #[test]
    fn test_spin_mutex_raii_releases_lock_on_panic() {
        let _guard = lock_test();
        let res = std::panic::catch_unwind(|| {
            with_state(|_s| {
                panic!("simulated panic while holding STATE lock");
            });
        });
        assert!(res.is_err());
        // If SpinMutexGuard did not release on unwind, this would deadlock.
        let attached = with_state(|s| s.attached);
        let _ = attached;
    }

    #[test]
    fn test_wpa_funcs_jump_table_has_no_null_slots() {
        assert!(WPA_FUNCS.wpa_attach.is_some());
        assert!(WPA_FUNCS.wpa_deattach.is_some());
        assert!(WPA_FUNCS.wpa_config_bss.is_some());
        assert!(WPA_FUNCS.wpa_config_set_unsafe.is_some());
        assert!(WPA_FUNCS.wpa_config_set_int.is_some());
        assert!(WPA_FUNCS.wpa_sta_rx_eapol_retry.is_some());
        assert!(WPA_FUNCS.wpa_sta_rx_mgmt.is_some());
        assert!(WPA_FUNCS.wpa_config_set_array.is_some());
        assert!(WPA_FUNCS.wpa_config_set_byte_array.is_some());
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
        assert_eq!(in_hs, 0);
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

        // JUSTIFICATION: Testing null pointer guard in eapol_txcb.
        // SAFETY: Verifying null pointer is safely rejected.
        unsafe {
            eapol_txcb(core::ptr::null_mut(), 0, 0);
        }

        let mut short_buf = [0u8; 50];
        // JUSTIFICATION: Testing short length guard in eapol_txcb.
        // SAFETY: Passing valid stack slice shorter than minimum 99 bytes.
        unsafe {
            eapol_txcb(short_buf.as_mut_ptr(), 50, 0);
        }

        let mut non_key_buf = [0u8; 100];
        non_key_buf[1] = 0x01;
        // JUSTIFICATION: Testing packet type check in eapol_txcb.
        // SAFETY: Passing valid stack buffer of 100 bytes.
        unsafe {
            eapol_txcb(non_key_buf.as_mut_ptr(), 100, 0);
        }

        driver::with_mock(|m| {
            assert!(!m.ptk_installed);
            assert!(!m.auth_done_called);
        });
    }

    #[test]
    fn test_crypto_ffi_and_stubs_exercise() {
        let _guard = lock_test();
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

        // JUSTIFICATION: Testing hostap and config stubs for coverage.
        // SAFETY: Stub functions have no-op or stubbed implementations.
        unsafe {
            assert!(crate::stubs::hostap_init().is_null());
            assert_eq!(ffi_hostap_deinit(core::ptr::null_mut()), 1);
            assert_eq!(ffi_wpa_ap_remove(core::ptr::null_mut()), 0);
            assert_eq!(
                ffi_wpa_ap_rx_eapol(
                    core::ptr::null_mut(),
                    core::ptr::null_mut(),
                    core::ptr::null_mut(),
                    0
                ),
                0
            );
            let mut cap = 1u8;
            let mut req = 1u8;
            ffi_wpa_ap_get_peer_spp_msg(core::ptr::null_mut(), &mut cap, &mut req);
            assert_eq!((cap, req), (0, 0));
            assert_eq!(wpa_config_bss_stub(core::ptr::null(), 0), 0);
            assert_eq!(wpa_noop_c_int_stub(), 0);
            wpa_noop_void_stub();
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
            m.ap_rsn_ie[..22].copy_from_slice(&AP_RSN_22B);
            m.ap_rsn_ie_len = 22;
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
            assert_eq!(wpa_sta_in_4way_handshake(), 1);
        }

        let mut m1 = [0u8; 99];
        m1[0] = 2;
        m1[1] = 3;
        m1[3] = 95;
        m1[4] = 2;
        m1[5] = 0x00;
        m1[6] = 0x8a;
        m1[8] = 16;
        m1[16] = 1;
        m1[17..49].fill(0xAA);

        // JUSTIFICATION: Processing received M1 frame through supplicant.
        // SAFETY: bssid and m1 are valid stack allocations.
        let ret = unsafe { wpa_sm_rx_eapol(bssid.as_mut_ptr(), m1.as_mut_ptr(), m1.len()) };
        assert_eq!(ret, 0);

        driver::with_mock(|m| {
            assert_eq!(m.tx_count, 1);
            assert!(m.last_tx_len >= 14 + 99);
            assert_eq!(&m.last_tx_buf[0..6], &bssid);
            assert_eq!(&m.last_tx_buf[6..12], &[0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc]);
            assert_eq!(&m.last_tx_buf[12..14], &[0x88, 0x8E]);
            assert!(!m.ptk_installed);
            assert!(!m.auth_done_called);
        });

        let mut last_tx = [0u8; 512];
        let mut last_len = 0;
        driver::with_mock(|m| {
            last_len = m.last_tx_len;
            last_tx[..last_len].copy_from_slice(&m.last_tx_buf[..last_len]);
        });

        // JUSTIFICATION: Invoking driver eapol_txcb with M2 frame.
        // SAFETY: Passing EAPOL payload starting at offset 14.
        unsafe {
            eapol_txcb(last_tx[14..].as_mut_ptr(), last_len - 14, 0);
        }

        driver::with_mock(|m| {
            assert!(!m.ptk_installed);
            assert!(!m.auth_done_called);
        });

        let mut snonce = [0u8; 32];
        snonce.copy_from_slice(&last_tx[31..63]);

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

        let gtk = [0x77u8; 16];
        let (mut m3, m3_len) = build_encrypted_m3(
            [0, 0, 0, 0, 0, 0, 0, 2],
            [0xAA; 32],
            &AP_RSN_22B,
            &gtk,
            &ptk,
        );

        // JUSTIFICATION: Processing received M3 frame through supplicant.
        // SAFETY: bssid and m3 are valid stack allocations.
        let ret = unsafe { wpa_sm_rx_eapol(bssid.as_mut_ptr(), m3.as_mut_ptr(), m3_len) };
        assert_eq!(ret, 0);

        driver::with_mock(|m| {
            assert_eq!(m.tx_count, 2);
            assert!(!m.ptk_installed);
            assert!(!m.auth_done_called);
        });

        let mut m4_tx = [0u8; 512];
        let mut m4_len = 0;
        driver::with_mock(|m| {
            m4_len = m.last_tx_len;
            m4_tx[..m4_len].copy_from_slice(&m.last_tx_buf[..m4_len]);
        });

        // JUSTIFICATION: Testing M2 TX completion guard while M4 is pending.
        // SAFETY: Passing valid M2 EAPOL payload starting at offset 14.
        unsafe {
            eapol_txcb(last_tx[14..].as_mut_ptr(), last_len - 14, 0);
        }
        driver::with_mock(|m| {
            assert!(!m.ptk_installed);
            assert!(!m.auth_done_called);
        });

        // First test TX failure: should NOT install keys.
        // JUSTIFICATION: Invoking driver eapol_txcb for M4 with tx_failure = 1.
        // SAFETY: Passing valid EAPOL payload.
        unsafe {
            eapol_txcb(m4_tx[14..].as_mut_ptr(), m4_len - 14, 1);
        }
        driver::with_mock(|m| {
            assert!(!m.ptk_installed);
            assert!(!m.auth_done_called);
        });

        // Retransmitted M3 after M4 TX failure succeeds and installs PTK + GTK.
        let (mut m3_2, m3_2_len) = build_encrypted_m3(
            [0, 0, 0, 0, 0, 0, 0, 3],
            [0xAA; 32],
            &AP_RSN_22B,
            &gtk,
            &ptk,
        );
        // JUSTIFICATION: Processing re-transmitted M3 frame.
        // SAFETY: bssid and m3_2 are valid stack allocations.
        let ret = unsafe { wpa_sm_rx_eapol(bssid.as_mut_ptr(), m3_2.as_mut_ptr(), m3_2_len) };
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
            eapol_txcb(m4_tx2[14..].as_mut_ptr(), m4_len2 - 14, 0);
        }
        driver::with_mock(|m| {
            assert!(m.ptk_installed);
            assert_eq!(m.ptk_flag, 0x2C);
            assert!(m.gtk_installed);
            assert!(m.auth_done_called);
        });
        // JUSTIFICATION: Confirming 4-way handshake clears after M4 TX.
        // SAFETY: Synchronized read of global state machine status.
        assert_eq!(unsafe { wpa_sta_in_4way_handshake() }, 0);
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
            m.ap_rsn_ie[..22].copy_from_slice(&AP_RSN_22B);
            m.ap_rsn_ie_len = 22;
        });

        let mut bssid = [0x00, 0x11, 0x22, 0x33, 0x44, 0x55];
        // JUSTIFICATION: Initiating station connect with valid bssid.
        // SAFETY: bssid is a valid 6-byte stack array.
        let ret = unsafe { wpa_sta_connect(bssid.as_mut_ptr()) };
        assert_eq!(ret, 0);

        driver::with_mock(|m| {
            assert_eq!(m.appie_4_flag, 0);
            assert_eq!(m.appie_4_len, 22);
            assert!(m.appie_3_unset);
            assert_eq!(&m.appie_4_buf[..22], &AP_RSN_22B);
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
            m.ap_rsn_ie[..22].copy_from_slice(&AP_RSN_22B);
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
