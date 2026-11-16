//! Miscellaneous NVS/log globals, pre-GC compatibility stubs, and anchor.
//!
//! Symbol ownership boundary: this module intentionally does NOT define or
//! export `g_misc_nvs`, `misc_nvs_init`, `misc_nvs_deinit`, `misc_nvs_restore`,
//! `g_log_level`, `putchar`, `_putchar`, `esp_dport_access_reg_read`, or
//! `__esp_radio_printf`. Those symbols are already provided by `esp-radio`,
//! `esp-phy`, and `esp-wifi-sys-esp32`.

use core::hint;

use crate::regulatory::{g_espnow_user_oui, REGDOMAIN_TABLE, REGULATORY_DATA};

/// Byte size of the `g_log_mod` BSS array in `libcore.a:misc_nvs.o`.
pub const LOG_MOD_SIZE: usize = 24;

/// Byte size of the `g_mt` BSS array in `libespnow.a:manatick.o`.
pub const MT_SIZE: usize = 4;

/// 4-byte aligned, `Sync`-marked zero-initialized byte buffer for C `.bss` ABI.
#[repr(C, align(4))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SyncBytes<const N: usize>(pub [u8; N]);

// JUSTIFICATION: C static `.bss` globals require a `Sync` wrapper in
// `#![no_std]` Rust without external synchronization crates.
// SAFETY: `[u8; N]` contains only plain byte storage with no interior
// mutability; exposing immutable `.bss` stub bytes across threads is sound.
unsafe impl<const N: usize> Sync for SyncBytes<N> {}

#[allow(non_upper_case_globals)]
#[cfg(target_arch = "xtensa")]
#[no_mangle]
#[link_section = ".bss.g_log_mod"]
/// Pre-GC `.bss` compatibility symbol from `libcore.a:misc_nvs.o`.
pub static g_log_mod: SyncBytes<LOG_MOD_SIZE> = SyncBytes([0; LOG_MOD_SIZE]);

#[allow(non_upper_case_globals)]
#[cfg(target_arch = "xtensa")]
#[no_mangle]
#[link_section = ".bss.g_mt"]
/// Pre-GC `.bss` compatibility symbol from `libespnow.a:manatick.o`.
pub static g_mt: SyncBytes<MT_SIZE> = SyncBytes([0; MT_SIZE]);

/// Pre-GC stub for `libespnow.a:manatick.o` (`mt_get_peer_info`).
#[cfg(target_arch = "xtensa")]
#[no_mangle]
#[link_section = ".text.mt_get_peer_info"]
pub extern "C" fn mt_get_peer_info() -> i32 {
    0
}

/// Pre-GC stub for `libmesh.a:mesh_quick.o` (`ieee80211_init_mesh_assoc_ie`).
#[cfg(target_arch = "xtensa")]
#[no_mangle]
#[link_section = ".text.ieee80211_init_mesh_assoc_ie"]
pub extern "C" fn ieee80211_init_mesh_assoc_ie() -> i32 {
    0
}

/// Pre-GC stub for `libmesh.a:mesh_quick.o` (`ieee80211_vnd_mesh_quick_get`).
#[cfg(target_arch = "xtensa")]
#[no_mangle]
#[link_section = ".text.ieee80211_vnd_mesh_quick_get"]
pub extern "C" fn ieee80211_vnd_mesh_quick_get() -> i32 {
    0
}

/// Pre-GC stub for `libmesh.a:mesh_quick.o` (`ieee80211_vnd_mesh_quick_set`).
#[cfg(target_arch = "xtensa")]
#[no_mangle]
#[link_section = ".text.ieee80211_vnd_mesh_quick_set"]
pub extern "C" fn ieee80211_vnd_mesh_quick_set() -> i32 {
    0
}

/// Pre-GC stub for `libmesh.a:mesh_quick.o` (`ieee80211_vnd_mesh_roots_get`).
#[cfg(target_arch = "xtensa")]
#[no_mangle]
#[link_section = ".text.ieee80211_vnd_mesh_roots_get"]
pub extern "C" fn ieee80211_vnd_mesh_roots_get() -> i32 {
    0
}

/// Pre-GC stub for `libmesh.a:mesh_quick.o` (`ieee80211_vnd_mesh_roots_set`).
#[cfg(target_arch = "xtensa")]
#[no_mangle]
#[link_section = ".text.ieee80211_vnd_mesh_roots_set"]
pub extern "C" fn ieee80211_vnd_mesh_roots_set() -> i32 {
    0
}

/// Pre-GC stub for `libmesh.a:mesh_quick.o` (`mesh_clear_parent_candidate`).
#[cfg(target_arch = "xtensa")]
#[no_mangle]
#[link_section = ".text.mesh_clear_parent_candidate"]
pub extern "C" fn mesh_clear_parent_candidate() {}

/// Pre-GC stub for `libmesh.a:mesh_quick.o` (`mesh_get_parent_candidate`).
#[cfg(target_arch = "xtensa")]
#[no_mangle]
#[link_section = ".text.mesh_get_parent_candidate"]
pub extern "C" fn mesh_get_parent_candidate() -> i32 {
    0
}

/// Pre-GC stub for `libmesh.a:mesh_quick.o`
/// (`mesh_get_parent_monitor_config`).
#[cfg(target_arch = "xtensa")]
#[no_mangle]
#[link_section = ".text.mesh_get_parent_monitor_config"]
pub extern "C" fn mesh_get_parent_monitor_config() -> i32 {
    0
}

/// Pre-GC stub for `libmesh.a:mesh_quick.o` (`mesh_get_rssi_threshold`).
#[cfg(target_arch = "xtensa")]
#[no_mangle]
#[link_section = ".text.mesh_get_rssi_threshold"]
pub extern "C" fn mesh_get_rssi_threshold() -> i32 {
    0
}

/// Pre-GC stub for `libmesh.a:mesh_quick.o` (`mesh_set_ie_crypto_config`).
#[cfg(target_arch = "xtensa")]
#[no_mangle]
#[link_section = ".text.mesh_set_ie_crypto_config"]
pub extern "C" fn mesh_set_ie_crypto_config() -> i32 {
    0
}

/// Pre-GC stub for `libmesh.a:mesh_quick.o` (`mesh_set_parent_candidate`).
#[cfg(target_arch = "xtensa")]
#[no_mangle]
#[link_section = ".text.mesh_set_parent_candidate"]
pub extern "C" fn mesh_set_parent_candidate() -> i32 {
    0
}

/// Pre-GC stub for `libmesh.a:mesh_quick.o`
/// (`mesh_set_parent_monitor_config`).
#[cfg(target_arch = "xtensa")]
#[no_mangle]
#[link_section = ".text.mesh_set_parent_monitor_config"]
pub extern "C" fn mesh_set_parent_monitor_config() -> i32 {
    0
}

/// Pre-GC stub for `libmesh.a:mesh_quick.o` (`mesh_set_rssi_threshold`).
#[cfg(target_arch = "xtensa")]
#[no_mangle]
#[link_section = ".text.mesh_set_rssi_threshold"]
pub extern "C" fn mesh_set_rssi_threshold() -> i32 {
    0
}

/// Anchors live `esp32-cam-rf-clock` regulatory symbols so the linker retains
/// the crate's object files before `--gc-sections`.
#[must_use]
pub fn link_anchor() -> usize {
    let reg_table = hint::black_box(&raw const REGDOMAIN_TABLE as usize);
    let reg_data = hint::black_box(&raw const REGULATORY_DATA as usize);
    let oui = hint::black_box(&raw const g_espnow_user_oui as usize);
    reg_table ^ reg_data ^ oui
}

#[cfg(test)]
mod tests {
    use core::mem;

    use super::*;

    #[test]
    fn sync_bytes_layout_meets_4_byte_alignment() {
        assert_eq!(
            mem::size_of::<SyncBytes<LOG_MOD_SIZE>>(),
            24,
            "SyncBytes<24> size must be 24"
        );
        assert!(
            mem::align_of::<SyncBytes<LOG_MOD_SIZE>>() >= 4,
            "SyncBytes<24> align must be >= 4"
        );
        assert_eq!(
            mem::size_of::<SyncBytes<MT_SIZE>>(),
            4,
            "SyncBytes<4> size must be 4"
        );
        assert!(
            mem::align_of::<SyncBytes<MT_SIZE>>() >= 4,
            "SyncBytes<4> align must be >= 4"
        );

        let zero_mod = SyncBytes([0_u8; LOG_MOD_SIZE]);
        let zero_mt = SyncBytes([0_u8; MT_SIZE]);
        assert_eq!(zero_mod.0, [0_u8; 24], "SyncBytes<24> must zero-init");
        assert_eq!(zero_mt.0, [0_u8; 4], "SyncBytes<4> must zero-init");
    }

    #[test]
    fn link_anchor_is_deterministic_xor_of_live_addresses() {
        let anchor = link_anchor();
        let expected = (&raw const REGDOMAIN_TABLE as usize)
            ^ (&raw const REGULATORY_DATA as usize)
            ^ (&raw const g_espnow_user_oui as usize);
        assert_eq!(anchor, expected, "link_anchor address XOR mismatch");
    }
}
