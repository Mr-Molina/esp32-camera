//! Pure-Rust C ABI `printf`/`vsnprintf`/`syslog` compatibility shims and
//! Xtensa Call8 assembly trampolines for `esp-wifi-sys-esp32` blobs.

#[cfg(target_arch = "xtensa")]
use core::arch::global_asm;
use core::hint;

use crate::printf_fmt::{format_syslog_line, rust_vsnprintf, XtensaVaList};

/// Default syslog tag emitted by `*_printf` shims (`"ESP_RADIO\0"`).
#[no_mangle]
pub static ESP_RADIO_TAG: [u8; 10] = *b"ESP_RADIO\0";

/// Formats a bounded 79-byte syslog line from `fmt`/`va` and forwards it to
/// `__esp_radio_printf` on Xtensa targets.
///
/// # Safety
/// `tag` (if non-null) and `fmt` must be valid, NUL-terminated C strings, and
/// `va` must be a valid [`XtensaVaList`] matching `fmt` specifiers.
// JUSTIFICATION: C radio blobs invoke `syslog` and `*_printf` trampolines
// that forward raw C strings and Xtensa Call8 varargs into Rust.
// SAFETY: `buf` is an 80-byte stack array; `format_syslog_line` guarantees NUL
// termination within 80 bytes; caller guarantees `tag`, `fmt`, and `va`.
#[no_mangle]
pub unsafe extern "C" fn rust_syslog(tag: *const u8, fmt: *const u8, va: *mut XtensaVaList) {
    let mut buf = [0_u8; 80];
    // JUSTIFICATION: Formats the bounded syslog line into a fixed stack array.
    // SAFETY: `buf` is a valid `&mut [u8; 80]`; `fmt` and `va` satisfy the
    // caller's `# Safety` contract.
    let len = unsafe { format_syslog_line(&mut buf, fmt, va) };

    #[cfg(target_arch = "xtensa")]
    if len > 0 && !tag.is_null() {
        extern "C" {
            fn __esp_radio_printf(tag: *const core::ffi::c_char, msg: *const core::ffi::c_char);
        }
        // JUSTIFICATION: Calls the ESP-IDF radio logging hook provided by
        // `esp-wifi-sys-esp32`.
        // SAFETY: `tag` is non-null and NUL-terminated per caller contract, and
        // `buf` is NUL-terminated within 80 bytes by `format_syslog_line`.
        unsafe {
            __esp_radio_printf(tag.cast(), buf.as_ptr().cast());
        }
    }

    #[cfg(not(target_arch = "xtensa"))]
    let _ = (len, tag);
}

/// Unmangled C ABI wrapper forwarding to [`rust_vsnprintf`] for assembly
/// trampolines (`vsnprintf`, `snprintf`, `sprintf`).
///
/// # Safety
/// `buf` must be writable for `size` bytes when non-null and `size > 0`, `fmt`
/// must be NUL-terminated, and `va` must match `fmt` specifiers.
// JUSTIFICATION: Xtensa assembly trampolines call an unmangled `extern "C"`
// symbol without relying on Rust symbol mangling.
// SAFETY: Forwards directly to `rust_vsnprintf` under the caller's `# Safety`
// preconditions for `buf`, `size`, `fmt`, and `va`.
#[no_mangle]
pub unsafe extern "C" fn rust_vsnprintf_abi(
    buf: *mut u8,
    size: usize,
    fmt: *const u8,
    va: *mut XtensaVaList,
) -> i32 {
    // JUSTIFICATION: Delegates formatting to the pure-Rust `rust_vsnprintf`.
    // SAFETY: Caller guarantees `buf`, `size`, `fmt`, and `va` satisfy
    // `rust_vsnprintf` preconditions.
    unsafe { rust_vsnprintf(buf, size, fmt, va) }
}

#[cfg(target_arch = "xtensa")]
global_asm!(
    // 1. vsnprintf(a2=buf, a3=count, a4=fmt, a5..a7=va)
    r#".pushsection .text.vsnprintf,"ax",@progbits"#,
    ".align 4",
    ".Lvsnprintf_fn:",
    ".word rust_vsnprintf_abi",
    ".globl vsnprintf",
    ".type vsnprintf,@function",
    ".align 4",
    "vsnprintf:",
    "entry a1, 64",
    "s32i.n a5, a1, 0",
    "s32i.n a6, a1, 4",
    "s32i.n a7, a1, 8",
    "mov.n a10, a2",
    "mov.n a11, a3",
    "mov.n a12, a4",
    "mov.n a13, a1",
    "l32r a8, .Lvsnprintf_fn",
    "callx8 a8",
    "mov.n a2, a10",
    "retw.n",
    ".size vsnprintf, .-vsnprintf",
    ".popsection",
    // 2. snprintf(a2=buf, a3=count, a4=fmt, ...)
    r#".pushsection .text.snprintf,"ax",@progbits"#,
    ".align 4",
    ".Lsnprintf_fn:",
    ".word rust_vsnprintf_abi",
    ".globl snprintf",
    ".type snprintf,@function",
    ".align 4",
    "snprintf:",
    "entry a1, 96",
    "addi a10, a1, 64",
    "addi a9, a1, 32",
    "movi.n a8, 12",
    "s32i.n a5, a1, 44",
    "s32i.n a6, a1, 48",
    "s32i.n a7, a1, 52",
    "s32i.n a10, a1, 0",
    "s32i.n a9, a1, 4",
    "s32i.n a8, a1, 8",
    "mov.n a10, a2",
    "mov.n a11, a3",
    "mov.n a12, a4",
    "mov.n a13, a1",
    "l32r a8, .Lsnprintf_fn",
    "callx8 a8",
    "mov.n a2, a10",
    "retw.n",
    ".size snprintf, .-snprintf",
    ".popsection",
    // 3. sprintf(a2=buf, a3=fmt, ...)
    r#".pushsection .text.sprintf,"ax",@progbits"#,
    ".align 4",
    ".Lsprintf_fn:",
    ".word rust_vsnprintf_abi",
    ".globl sprintf",
    ".type sprintf,@function",
    ".align 4",
    "sprintf:",
    "entry a1, 96",
    "addi a10, a1, 64",
    "addi a9, a1, 32",
    "movi.n a8, 8",
    "s32i.n a4, a1, 40",
    "s32i.n a5, a1, 44",
    "s32i.n a6, a1, 48",
    "s32i.n a7, a1, 52",
    "s32i.n a10, a1, 0",
    "s32i.n a9, a1, 4",
    "s32i.n a8, a1, 8",
    "mov.n a10, a2",
    "movi.n a11, -1",
    "mov.n a12, a3",
    "mov.n a13, a1",
    "l32r a8, .Lsprintf_fn",
    "callx8 a8",
    "mov.n a2, a10",
    "retw.n",
    ".size sprintf, .-sprintf",
    ".popsection",
    // 4. syslog(a2=tag, a3=fmt, a4..a6=va)
    r#".pushsection .text.syslog,"ax",@progbits"#,
    ".align 4",
    ".Lsyslog_fn:",
    ".word rust_syslog",
    ".globl syslog",
    ".type syslog,@function",
    ".align 4",
    "syslog:",
    "entry a1, 64",
    "s32i.n a4, a1, 0",
    "s32i.n a5, a1, 4",
    "s32i.n a6, a1, 8",
    "mov.n a10, a2",
    "mov.n a11, a3",
    "mov.n a12, a1",
    "l32r a8, .Lsyslog_fn",
    "callx8 a8",
    "retw.n",
    ".size syslog, .-syslog",
    ".popsection",
    // 5. rtc_printf(a2=fmt, ...)
    r#".pushsection .text.rtc_printf,"ax",@progbits"#,
    ".align 4",
    ".Lrtc_printf_tag:",
    ".word ESP_RADIO_TAG",
    ".Lrtc_printf_fn:",
    ".word rust_syslog",
    ".globl rtc_printf",
    ".type rtc_printf,@function",
    ".align 4",
    "rtc_printf:",
    "entry a1, 80",
    "addi a12, a1, 48",
    "addi a13, a1, 16",
    "movi.n a8, 4",
    "s32i.n a3, a1, 20",
    "s32i.n a4, a1, 24",
    "s32i.n a5, a1, 28",
    "s32i.n a6, a1, 32",
    "s32i.n a7, a1, 36",
    "s32i.n a12, a1, 0",
    "s32i.n a13, a1, 4",
    "s32i.n a8, a1, 8",
    "l32r a10, .Lrtc_printf_tag",
    "mov.n a11, a2",
    "mov.n a12, a1",
    "l32r a8, .Lrtc_printf_fn",
    "callx8 a8",
    "retw.n",
    ".size rtc_printf, .-rtc_printf",
    ".popsection",
    // 6. coexist_printf(a2=fmt, ...)
    r#".pushsection .text.coexist_printf,"ax",@progbits"#,
    ".align 4",
    ".Lcoexist_printf_tag:",
    ".word ESP_RADIO_TAG",
    ".Lcoexist_printf_fn:",
    ".word rust_syslog",
    ".globl coexist_printf",
    ".type coexist_printf,@function",
    ".align 4",
    "coexist_printf:",
    "entry a1, 80",
    "addi a12, a1, 48",
    "addi a13, a1, 16",
    "movi.n a8, 4",
    "s32i.n a3, a1, 20",
    "s32i.n a4, a1, 24",
    "s32i.n a5, a1, 28",
    "s32i.n a6, a1, 32",
    "s32i.n a7, a1, 36",
    "s32i.n a12, a1, 0",
    "s32i.n a13, a1, 4",
    "s32i.n a8, a1, 8",
    "l32r a10, .Lcoexist_printf_tag",
    "mov.n a11, a2",
    "mov.n a12, a1",
    "l32r a8, .Lcoexist_printf_fn",
    "callx8 a8",
    "retw.n",
    ".size coexist_printf, .-coexist_printf",
    ".popsection",
    // 7. net80211_printf(a2=fmt, ...)
    r#".pushsection .text.net80211_printf,"ax",@progbits"#,
    ".align 4",
    ".Lnet80211_printf_tag:",
    ".word ESP_RADIO_TAG",
    ".Lnet80211_printf_fn:",
    ".word rust_syslog",
    ".globl net80211_printf",
    ".type net80211_printf,@function",
    ".align 4",
    "net80211_printf:",
    "entry a1, 80",
    "addi a12, a1, 48",
    "addi a13, a1, 16",
    "movi.n a8, 4",
    "s32i.n a3, a1, 20",
    "s32i.n a4, a1, 24",
    "s32i.n a5, a1, 28",
    "s32i.n a6, a1, 32",
    "s32i.n a7, a1, 36",
    "s32i.n a12, a1, 0",
    "s32i.n a13, a1, 4",
    "s32i.n a8, a1, 8",
    "l32r a10, .Lnet80211_printf_tag",
    "mov.n a11, a2",
    "mov.n a12, a1",
    "l32r a8, .Lnet80211_printf_fn",
    "callx8 a8",
    "retw.n",
    ".size net80211_printf, .-net80211_printf",
    ".popsection",
    // 8. phy_printf(a2=fmt, ...)
    r#".pushsection .text.phy_printf,"ax",@progbits"#,
    ".align 4",
    ".Lphy_printf_tag:",
    ".word ESP_RADIO_TAG",
    ".Lphy_printf_fn:",
    ".word rust_syslog",
    ".globl phy_printf",
    ".type phy_printf,@function",
    ".align 4",
    "phy_printf:",
    "entry a1, 80",
    "addi a12, a1, 48",
    "addi a13, a1, 16",
    "movi.n a8, 4",
    "s32i.n a3, a1, 20",
    "s32i.n a4, a1, 24",
    "s32i.n a5, a1, 28",
    "s32i.n a6, a1, 32",
    "s32i.n a7, a1, 36",
    "s32i.n a12, a1, 0",
    "s32i.n a13, a1, 4",
    "s32i.n a8, a1, 8",
    "l32r a10, .Lphy_printf_tag",
    "mov.n a11, a2",
    "mov.n a12, a1",
    "l32r a8, .Lphy_printf_fn",
    "callx8 a8",
    "retw.n",
    ".size phy_printf, .-phy_printf",
    ".popsection",
    // 9. pp_printf(a2=fmt, ...)
    r#".pushsection .text.pp_printf,"ax",@progbits"#,
    ".align 4",
    ".Lpp_printf_tag:",
    ".word ESP_RADIO_TAG",
    ".Lpp_printf_fn:",
    ".word rust_syslog",
    ".globl pp_printf",
    ".type pp_printf,@function",
    ".align 4",
    "pp_printf:",
    "entry a1, 80",
    "addi a12, a1, 48",
    "addi a13, a1, 16",
    "movi.n a8, 4",
    "s32i.n a3, a1, 20",
    "s32i.n a4, a1, 24",
    "s32i.n a5, a1, 28",
    "s32i.n a6, a1, 32",
    "s32i.n a7, a1, 36",
    "s32i.n a12, a1, 0",
    "s32i.n a13, a1, 4",
    "s32i.n a8, a1, 8",
    "l32r a10, .Lpp_printf_tag",
    "mov.n a11, a2",
    "mov.n a12, a1",
    "l32r a8, .Lpp_printf_fn",
    "callx8 a8",
    "retw.n",
    ".size pp_printf, .-pp_printf",
    ".popsection",
    // 10. __esp_radio_log_write(a2=level, a3=tag, a4=fmt, ...)
    r#".pushsection .text.__esp_radio_log_write,"ax",@progbits"#,
    ".align 4",
    ".Llogw_fn:",
    ".word rust_syslog",
    ".globl __esp_radio_log_write",
    ".type __esp_radio_log_write,@function",
    ".align 4",
    "__esp_radio_log_write:",
    "entry a1, 80",
    "addi a12, a1, 48",
    "addi a13, a1, 16",
    "movi.n a8, 12",
    "s32i.n a5, a1, 28",
    "s32i.n a6, a1, 32",
    "s32i.n a7, a1, 36",
    "s32i.n a12, a1, 0",
    "s32i.n a13, a1, 4",
    "s32i.n a8, a1, 8",
    "mov.n a10, a3",
    "mov.n a11, a4",
    "mov.n a12, a1",
    "l32r a8, .Llogw_fn",
    "callx8 a8",
    "retw.n",
    ".size __esp_radio_log_write, .-__esp_radio_log_write",
    ".popsection",
    // 11. __esp_radio_log_writev(a2=level, a3=tag, a4=fmt, a5..a7=va)
    r#".pushsection .text.__esp_radio_log_writev,"ax",@progbits"#,
    ".align 4",
    ".Llogwv_fn:",
    ".word rust_syslog",
    ".globl __esp_radio_log_writev",
    ".type __esp_radio_log_writev,@function",
    ".align 4",
    "__esp_radio_log_writev:",
    "entry a1, 64",
    "s32i.n a5, a1, 0",
    "s32i.n a6, a1, 4",
    "s32i.n a7, a1, 8",
    "mov.n a10, a3",
    "mov.n a11, a4",
    "mov.n a12, a1",
    "l32r a8, .Llogwv_fn",
    "callx8 a8",
    "retw.n",
    ".size __esp_radio_log_writev, .-__esp_radio_log_writev",
    ".popsection",
);

/// Anchors `rust_syslog`, `rust_vsnprintf_abi`, [`ESP_RADIO_TAG`], and on
/// Xtensa all 11 assembly trampoline symbols against `--gc-sections`.
#[must_use]
#[inline(never)]
pub fn link_touch() -> usize {
    let syslog_addr = hint::black_box(
        rust_syslog as unsafe extern "C" fn(*const u8, *const u8, *mut XtensaVaList) as *const ()
            as usize,
    );
    let vsnprintf_addr = hint::black_box(
        rust_vsnprintf_abi
            as unsafe extern "C" fn(*mut u8, usize, *const u8, *mut XtensaVaList) -> i32
            as *const () as usize,
    );
    let tag_addr = hint::black_box(&raw const ESP_RADIO_TAG as usize);
    let base = syslog_addr ^ vsnprintf_addr ^ tag_addr;

    #[cfg(target_arch = "xtensa")]
    {
        extern "C" {
            fn vsnprintf();
            fn snprintf();
            fn sprintf();
            fn syslog();
            fn rtc_printf();
            fn coexist_printf();
            fn net80211_printf();
            fn phy_printf();
            fn pp_printf();
            fn __esp_radio_log_write();
            fn __esp_radio_log_writev();
        }
        let mut acc = base;
        let syms: [unsafe extern "C" fn(); 11] = [
            vsnprintf,
            snprintf,
            sprintf,
            syslog,
            rtc_printf,
            coexist_printf,
            net80211_printf,
            phy_printf,
            pp_printf,
            __esp_radio_log_write,
            __esp_radio_log_writev,
        ];
        for sym in syms {
            acc ^= hint::black_box(sym as *const () as usize);
        }
        acc
    }

    #[cfg(not(target_arch = "xtensa"))]
    {
        base
    }
}

#[cfg(test)]
mod tests {
    use core::ptr;

    use super::*;

    #[repr(C, align(8))]
    struct AlignedWords<const N: usize>([u32; N]);

    #[test]
    fn esp_radio_tag_is_nul_terminated_ascii() {
        assert_eq!(ESP_RADIO_TAG, *b"ESP_RADIO\0", "ESP_RADIO_TAG mismatch");
    }

    #[test]
    fn rust_vsnprintf_abi_formats_and_truncates() {
        let mut out = [0_u8; 32];
        let reg = AlignedWords([0, 0, 0, 0x2A, 0xBE, 0]);
        let stk = AlignedWords([0_u32; 12]);
        let mut va = XtensaVaList {
            va_stk: stk.0.as_ptr(),
            va_reg: reg.0.as_ptr(),
            va_ndx: 12,
        };
        // JUSTIFICATION: Calls `rust_vsnprintf_abi` directly against aligned
        // mock register/stack buffers to test C ABI formatting.
        // SAFETY: `out` is 32 writable bytes, format is NUL-terminated, and
        // `va` provides two `u32` slots (`0x2A`, `0xBE`).
        let written = unsafe {
            rust_vsnprintf_abi(
                out.as_mut_ptr(),
                out.len(),
                c"val=%d hex=%02X".as_ptr().cast(),
                &mut va,
            )
        };
        assert_eq!(written, 13, "written char count mismatch");
        assert_eq!(&out[..14], b"val=42 hex=BE\0", "formatted output mismatch");

        let mut small = [0xFF_u8; 6];
        let mut va_small = XtensaVaList {
            va_stk: stk.0.as_ptr(),
            va_reg: reg.0.as_ptr(),
            va_ndx: 12,
        };
        // JUSTIFICATION: Verifies bounded truncation and untruncated length
        // reporting from `rust_vsnprintf_abi`.
        // SAFETY: `small` is 6 writable bytes; `va_small` holds two `u32` args.
        let full_len = unsafe {
            rust_vsnprintf_abi(
                small.as_mut_ptr(),
                small.len(),
                c"val=%d hex=%02X".as_ptr().cast(),
                &mut va_small,
            )
        };
        assert_eq!(full_len, 13, "untruncated return length mismatch");
        assert_eq!(&small, b"val=4\0", "truncated buffer mismatch");
    }

    #[test]
    fn rust_syslog_handles_valid_and_null_inputs() {
        let reg = AlignedWords([0, 7, 9, 0, 0, 0]);
        let stk = AlignedWords([0_u32; 12]);
        let mut va = XtensaVaList {
            va_stk: stk.0.as_ptr(),
            va_reg: reg.0.as_ptr(),
            va_ndx: 4,
        };
        // JUSTIFICATION: Exercises `rust_syslog` with valid and null pointers
        // under host test harnesses and Miri.
        // SAFETY: `ESP_RADIO_TAG` and format string are NUL-terminated and
        // `va` holds two `i32` slots (`7`, `9`).
        unsafe {
            rust_syslog(
                ESP_RADIO_TAG.as_ptr(),
                c"ch=%d rssi=%d \r\n".as_ptr().cast(),
                &mut va,
            );
            rust_syslog(ptr::null(), ptr::null(), ptr::null_mut());
        }
    }

    #[test]
    fn link_touch_returns_nonzero_anchor() {
        let actual = link_touch();
        assert_ne!(actual, 0, "link_touch must return a non-zero address mix");
    }
}
