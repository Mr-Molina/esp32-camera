//! Linker anchor and `__esp_radio_printf` shim for `esp32-cam-rf-clock`.

#[cfg(target_arch = "xtensa")]
use core::ffi::c_char;
#[cfg(all(target_arch = "xtensa", feature = "sys-logs"))]
use core::ffi::CStr;

/// Forces the linker to retain every codegen unit in `esp32-cam-rf-clock`.
#[must_use]
pub fn force_rf_clock_link() -> usize {
    esp32_cam_rf_clock::link_anchor()
        ^ esp32_cam_rf_clock::link_rtc_rf()
        ^ esp32_cam_rf_clock::link_printf_compat()
}

#[cfg(all(target_arch = "xtensa", feature = "sys-logs"))]
#[no_mangle]
extern "C" fn __esp_radio_printf(tag: *const c_char, msg: *const c_char) {
    let _ = core::hint::black_box(force_rf_clock_link());
    if tag.is_null() || msg.is_null() {
        return;
    }
    // JUSTIFICATION: ESP-IDF radio blobs and `rust_syslog` call the C ABI
    // `__esp_radio_printf` hook with raw C string pointers when `sys-logs` is
    // enabled.
    // SAFETY: Null pointers are rejected above, and the C ABI caller guarantees
    // `tag` and `msg` point to valid NUL-terminated strings for this call.
    let (tag_str, msg_str) = unsafe {
        (
            CStr::from_ptr(tag).to_str().unwrap_or("<non-utf8>"),
            CStr::from_ptr(msg).to_str().unwrap_or("<non-utf8>"),
        )
    };
    #[cfg(feature = "log")]
    ::log::info!("{} {}", tag_str, msg_str);
    #[cfg(feature = "defmt")]
    ::defmt::info!("{} {}", tag_str, msg_str);
    #[cfg(not(any(feature = "log", feature = "defmt")))]
    let _ = (tag_str, msg_str);
}

#[cfg(all(target_arch = "xtensa", not(feature = "sys-logs")))]
#[no_mangle]
extern "C" fn __esp_radio_printf(_tag: *const c_char, _msg: *const c_char) {
    let _ = core::hint::black_box(force_rf_clock_link());
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn force_rf_clock_link_combines_all_module_anchors() {
        let combined = force_rf_clock_link();
        let expected = esp32_cam_rf_clock::link_anchor()
            ^ esp32_cam_rf_clock::link_rtc_rf()
            ^ esp32_cam_rf_clock::link_printf_compat();
        assert_eq!(combined, expected);
    }
}
