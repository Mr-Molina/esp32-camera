//! Pure `#![no_std]` Xtensa Call8 varargs (`XtensaVaList`) and bounded
//! `vsnprintf` / syslog line formatter for `esp32-cam-rf-clock`.
// JUSTIFICATION: Bare-metal `#![no_std]` C ABI varargs and `vsnprintf` require
// raw pointer reads/writes into caller buffers and ABI register/stack frames.

const F_LEFT: u8 = 1;
const F_PLUS: u8 = 2;
const F_SPACE: u8 = 4;
const F_ZERO: u8 = 8;
const F_ALT: u8 = 16;

/// GCC Xtensa Call8 ABI `va_list` layout (12 bytes, 4-byte aligned on Xtensa).
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct XtensaVaList {
    /// Overflow argument base (`va_stk[8]` starts first stack arg).
    pub va_stk: *const u32,
    /// Saved incoming register window (`a2..a7`, 6 x `u32`).
    pub va_reg: *const u32,
    /// Current byte cursor across `va_reg` (`0..=0x18`) and `va_stk` (`>= 0x24`).
    pub va_ndx: usize,
}

/// Fetches the next 32-bit word from an Xtensa Call8 [`XtensaVaList`].
///
/// # Safety
/// `va.va_reg` and `va.va_stk` must be valid for 4-byte aligned `u32` reads at
/// the slot selected by `va.va_ndx` per the Xtensa Call8 ABI.
pub unsafe fn va_arg_u32(va: &mut XtensaVaList) -> u32 {
    let mut next = va.va_ndx.wrapping_add(4);
    if next > 0x18 && va.va_ndx < 0x19 {
        next = 0x24;
    }
    let base = if next > 0x18 { va.va_stk } else { va.va_reg };
    va.va_ndx = next;
    if base.is_null() {
        return 0;
    }
    // SAFETY: Caller guarantees `base` is valid and 4-byte aligned at slot.
    unsafe { *base.add((next - 4) / 4) }
}

/// Fetches the next 8-byte aligned 64-bit word from an [`XtensaVaList`].
///
/// # Safety
/// `va.va_reg` and `va.va_stk` must be valid for 4-byte aligned `u32` reads at
/// the aligned 8-byte slot selected by `va.va_ndx`.
pub unsafe fn va_arg_u64(va: &mut XtensaVaList) -> u64 {
    let aligned = va.va_ndx.wrapping_add(7) & !7usize;
    let mut next = aligned.wrapping_add(8);
    if next > 0x18 && aligned < 0x19 {
        next = 0x28;
    }
    let base = if next > 0x18 { va.va_stk } else { va.va_reg };
    va.va_ndx = next;
    if base.is_null() {
        return 0;
    }
    // SAFETY: Caller guarantees `base` is valid at `(next-8)/4` & `(next-4)/4`.
    let (lo, hi) = unsafe { (*base.add((next - 8) / 4), *base.add((next - 4) / 4)) };
    u64::from(lo) | (u64::from(hi) << 32)
}

// SAFETY: `va` must hold a valid pointer argument for target pointer width.
unsafe fn va_arg_ptr(va: &mut XtensaVaList) -> usize {
    #[cfg(target_pointer_width = "64")]
    // SAFETY: Host 64-bit tests encode a pointer across two `u32` slots.
    let (lo, hi) = unsafe { (va_arg_u32(va), va_arg_u32(va)) };
    #[cfg(target_pointer_width = "64")]
    return usize::try_from(lo).unwrap_or(0) | (usize::try_from(hi).unwrap_or(0) << 32);
    #[cfg(target_pointer_width = "32")]
    // SAFETY: Xtensa 32-bit pointers occupy one 32-bit vararg slot.
    return usize::try_from(unsafe { va_arg_u32(va) }).unwrap_or(0);
}

struct FmtBuf {
    buf: *mut u8,
    cap: usize,
    has_buf: bool,
    total: usize,
}

impl FmtBuf {
    fn push(&mut self, b: u8) {
        if self.has_buf && self.total < self.cap {
            // SAFETY: `self.buf` is non-null and `self.total < size - 1`.
            unsafe { *self.buf.add(self.total) = b };
        }
        self.total = self.total.saturating_add(1);
    }

    fn rep(&mut self, b: u8, n: usize) {
        for _ in 0..n {
            self.push(b);
        }
    }
}

#[derive(Default)]
struct FmtSpec {
    flags: u8,
    width: usize,
    prec: Option<usize>,
    bits: u8,
}

fn parse_uint(fmt: &[u8], idx: &mut usize) -> usize {
    let mut val = 0usize;
    while let Some(&b) = fmt.get(*idx) {
        if !b.is_ascii_digit() {
            break;
        }
        val = val.saturating_mul(10).saturating_add(usize::from(b - b'0'));
        *idx += 1;
    }
    val
}

// SAFETY: `va` must hold `i32` args for any `*` width or precision fields.
unsafe fn parse_spec(fmt: &[u8], idx: &mut usize, va: &mut XtensaVaList) -> FmtSpec {
    let mut s = FmtSpec {
        bits: 32,
        ..FmtSpec::default()
    };
    while let Some(&b) = fmt.get(*idx) {
        s.flags |= match b {
            b'-' => F_LEFT,
            b'+' => F_PLUS,
            b' ' => F_SPACE,
            b'0' => F_ZERO,
            b'#' => F_ALT,
            _ => break,
        };
        *idx += 1;
    }
    if fmt.get(*idx) == Some(&b'*') {
        *idx += 1;
        // SAFETY: Caller guarantees `va` holds an `i32` width argument.
        let w = unsafe { va_arg_u32(va) } as i32;
        if w < 0 {
            s.flags |= F_LEFT;
        }
        s.width = usize::try_from(w.unsigned_abs()).unwrap_or(0);
    } else if fmt.get(*idx).is_some_and(u8::is_ascii_digit) {
        s.width = parse_uint(fmt, idx);
    }
    if fmt.get(*idx) == Some(&b'.') {
        *idx += 1;
        if fmt.get(*idx) == Some(&b'*') {
            *idx += 1;
            // SAFETY: Caller guarantees `va` holds an `i32` precision arg.
            s.prec = usize::try_from(unsafe { va_arg_u32(va) } as i32).ok();
        } else {
            s.prec = Some(parse_uint(fmt, idx));
        }
    }
    let (adv, bits) = match (fmt.get(*idx).copied(), fmt.get(*idx + 1).copied()) {
        (Some(b'h'), Some(b'h')) => (2, 8),
        (Some(b'h'), _) => (1, 16),
        (Some(b'l'), Some(b'l')) => (2, 64),
        (Some(b'l' | b'z'), _) => (1, 32),
        _ => (0, 32),
    };
    *idx += adv;
    s.bits = bits;
    if (s.flags & F_LEFT) != 0 {
        s.flags &= !F_ZERO;
    }
    if (s.flags & F_PLUS) != 0 {
        s.flags &= !F_SPACE;
    }
    s
}

fn sign_pfx(neg: bool, flags: u8) -> &'static [u8] {
    if neg {
        b"-"
    } else if (flags & F_PLUS) != 0 {
        b"+"
    } else if (flags & F_SPACE) != 0 {
        b" "
    } else {
        b""
    }
}

fn emit_pad(out: &mut FmtBuf, s: &FmtSpec, pfx: &[u8], zfill: usize, body: &[u8], allow_z: bool) {
    let pad = s.width.saturating_sub(pfx.len() + zfill + body.len());
    let left = (s.flags & F_LEFT) != 0;
    let zero = !left && (s.flags & F_ZERO) != 0 && allow_z;
    if !left && !zero {
        out.rep(b' ', pad);
    }
    for &b in pfx {
        out.push(b);
    }
    if zero {
        out.rep(b'0', pad);
    }
    out.rep(b'0', zfill);
    for &b in body {
        out.push(b);
    }
    if left {
        out.rep(b' ', pad);
    }
}

fn fmt_u64(mut val: u64, base: u64, upper: bool, buf: &mut [u8; 24]) -> &[u8] {
    let digits = if upper {
        b"0123456789ABCDEF"
    } else {
        b"0123456789abcdef"
    };
    let mut pos = buf.len();
    if val == 0 {
        pos -= 1;
        buf[pos] = b'0';
        return &buf[pos..];
    }
    while val > 0 && pos > 0 {
        pos -= 1;
        buf[pos] = digits[usize::try_from(val % base).unwrap_or(0)];
        val /= base;
    }
    &buf[pos..]
}

// SAFETY: `va` must hold an integer argument matching `bits`.
unsafe fn arg_int(bits: u8, signed: bool, va: &mut XtensaVaList) -> (bool, u64) {
    // SAFETY: Caller guarantees `va` holds matching 64-bit or 32-bit int.
    let raw = unsafe {
        if bits == 64 {
            va_arg_u64(va)
        } else {
            u64::from(va_arg_u32(va))
        }
    };
    if !signed {
        let mask = if bits == 8 {
            0xFF
        } else if bits == 16 {
            0xFFFF
        } else {
            u64::MAX
        };
        return (false, raw & mask);
    }
    let v = match bits {
        8 => i64::from((raw as u8) as i8),
        16 => i64::from((raw as u16) as i16),
        64 => raw as i64,
        _ => i64::from(raw as i32),
    };
    (v < 0, v.unsigned_abs())
}

// SAFETY: `va` must hold an integer or pointer argument matching `conv`.
unsafe fn emit_int(out: &mut FmtBuf, s: &FmtSpec, conv: u8, va: &mut XtensaVaList) {
    let mut buf = [0u8; 24];
    let (pfx, mag, base, upper) = if conv == b'p' {
        // SAFETY: Caller guarantees `va` holds a pointer-width argument.
        let addr = unsafe { va_arg_ptr(va) };
        (
            b"0x".as_slice(),
            u64::try_from(addr).unwrap_or(0),
            16u64,
            false,
        )
    } else if matches!(conv, b'd' | b'i') {
        // SAFETY: Caller guarantees `va` holds matching signed integer arg.
        let (neg, mag) = unsafe { arg_int(s.bits, true, va) };
        (sign_pfx(neg, s.flags), mag, 10u64, false)
    } else {
        // SAFETY: Caller guarantees `va` holds matching unsigned int arg.
        let (_, v) = unsafe { arg_int(s.bits, false, va) };
        let alt = (s.flags & F_ALT) != 0 && v != 0;
        let pfx: &[u8] = match conv {
            b'x' if alt => b"0x",
            b'X' if alt => b"0X",
            b'o' if alt => b"0",
            _ => b"",
        };
        let base = if matches!(conv, b'x' | b'X') {
            16
        } else if conv == b'o' {
            8
        } else {
            10
        };
        (pfx, v, base, conv == b'X')
    };
    let body = if mag == 0 && s.prec == Some(0) {
        if conv == b'o' && (s.flags & F_ALT) != 0 {
            b"0".as_slice()
        } else {
            b"".as_slice()
        }
    } else {
        fmt_u64(mag, base, upper, &mut buf)
    };
    let prec_pad = s.prec.map_or(0, |p| p.saturating_sub(body.len()));
    emit_pad(out, s, pfx, prec_pad, body, s.prec.is_none());
}

// SAFETY: `va` must hold a promoted IEEE-754 `f64` argument.
unsafe fn emit_float_fixed(out: &mut FmtBuf, s: &FmtSpec, upper: bool, va: &mut XtensaVaList) {
    // SAFETY: Caller guarantees `va` holds an `f64` argument.
    let bits = unsafe { va_arg_u64(va) };
    let abs_bits = bits & 0x7FFF_FFFF_FFFF_FFFF;
    let abs_val = f64::from_bits(abs_bits);
    let pfx = sign_pfx((bits >> 63) != 0, s.flags);
    if (abs_bits >> 52) == 0x7FF {
        let is_nan = (abs_bits & 0x000F_FFFF_FFFF_FFFF) != 0;
        let tag = match (is_nan, upper) {
            (true, true) => b"NAN".as_slice(),
            (true, false) => b"nan".as_slice(),
            (false, true) => b"INF".as_slice(),
            (false, false) => b"inf".as_slice(),
        };
        emit_pad(out, s, if is_nan { b"" } else { pfx }, 0, tag, false);
        return;
    }
    if abs_val >= 1.8e19 {
        emit_pad(out, s, pfx, 0, b"ovf", false);
        return;
    }
    let prec = s.prec.unwrap_or(6);
    let eff = prec.min(15);
    let scale = 10_u64.pow(u32::try_from(eff).unwrap_or(15));
    let mut whole = abs_val as u64;
    let frac = if abs_val < 9_007_199_254_740_992.0 {
        abs_val - (whole as f64)
    } else {
        0.0
    };
    let mut f_int = (frac * (scale as f64) + 0.5) as u64;
    if f_int >= scale {
        f_int -= scale;
        whole = whole.saturating_add(1);
    }
    let (mut buf, mut tmp) = ([0u8; 48], [0u8; 24]);
    let int_s = fmt_u64(whole, 10, false, &mut tmp);
    let mut len = int_s.len().min(buf.len());
    buf[..len].copy_from_slice(&int_s[..len]);
    if (prec > 0 || (s.flags & F_ALT) != 0) && len < buf.len() {
        buf[len] = b'.';
        len += 1;
        let f_s = fmt_u64(f_int, 10, false, &mut tmp);
        let lz = eff.saturating_sub(f_s.len()).min(buf.len() - len);
        buf[len..len + lz].fill(b'0');
        len += lz;
        if eff > 0 {
            let n = f_s.len().min(buf.len() - len);
            buf[len..len + n].copy_from_slice(&f_s[..n]);
            len += n;
        }
        let tz = prec.saturating_sub(eff).min(buf.len() - len);
        buf[len..len + tz].fill(b'0');
        len += tz;
    }
    emit_pad(out, s, pfx, 0, &buf[..len], true);
}

// SAFETY: For `%s`, non-null pointer must be readable up to NUL or `s.prec`.
unsafe fn emit_str_char(out: &mut FmtBuf, s: &FmtSpec, is_str: bool, va: &mut XtensaVaList) {
    if !is_str {
        // SAFETY: Caller guarantees `va` holds a `u32` char argument.
        let ch = (unsafe { va_arg_u32(va) } & 0xFF) as u8;
        emit_pad(out, s, b"", 0, &[ch], false);
        return;
    }
    // SAFETY: Caller guarantees `va` holds a pointer-width `%s` argument.
    let ptr = unsafe { va_arg_ptr(va) } as *const u8;
    let max_len = s.prec.unwrap_or(usize::MAX);
    let body = if ptr.is_null() {
        &b"(null)"[..6usize.min(max_len)]
    } else {
        let mut len = 0usize;
        // SAFETY: Caller guarantees `ptr` is readable through NUL or `max_len`.
        while len < max_len && unsafe { *ptr.add(len) } != 0 {
            len += 1;
        }
        // SAFETY: `ptr` is readable for `len` bytes verified above.
        unsafe { core::slice::from_raw_parts(ptr, len) }
    };
    emit_pad(out, s, b"", 0, body, false);
}

/// Formats `fmt` with Xtensa Call8 `va` into `buf` (`size` bytes).
///
/// # Safety
/// `buf` must be writable for `size` bytes when non-null, `fmt` must be
/// NUL-terminated, and `va` must match `fmt` specifiers.
pub unsafe fn rust_vsnprintf(
    buf: *mut u8,
    size: usize,
    fmt: *const u8,
    va: *mut XtensaVaList,
) -> i32 {
    let has_buf = !buf.is_null() && size > 0;
    if fmt.is_null() || va.is_null() {
        if has_buf {
            // SAFETY: `buf` is non-null and `size > 0`.
            unsafe { *buf = 0 };
        }
        return 0;
    }
    let mut fmt_len = 0usize;
    // SAFETY: Caller guarantees `fmt` points to a NUL-terminated C string.
    while unsafe { *fmt.add(fmt_len) } != 0 {
        fmt_len += 1;
    }
    // SAFETY: `fmt` is readable for `fmt_len` bytes per NUL scan above.
    let fmt_bytes = unsafe { core::slice::from_raw_parts(fmt, fmt_len) };
    let mut out = FmtBuf {
        buf,
        cap: size.saturating_sub(1),
        has_buf,
        total: 0,
    };
    // SAFETY: Caller guarantees `va` points to a valid mutable `XtensaVaList`.
    let va_ref = unsafe { &mut *va };
    let mut idx = 0usize;
    while let Some(&ch) = fmt_bytes.get(idx) {
        idx += 1;
        if ch != b'%' {
            out.push(ch);
            continue;
        }
        if fmt_bytes.get(idx) == Some(&b'%') {
            idx += 1;
            out.push(b'%');
            continue;
        }
        // SAFETY: `va_ref` is valid for any `*` width/precision args.
        let spec = unsafe { parse_spec(fmt_bytes, &mut idx, va_ref) };
        let Some(&conv) = fmt_bytes.get(idx) else {
            break;
        };
        idx += 1;
        // SAFETY: `va_ref` holds argument matching `conv` per `fmt`.
        unsafe {
            match conv {
                b'd' | b'i' | b'u' | b'x' | b'X' | b'o' | b'p' => {
                    emit_int(&mut out, &spec, conv, va_ref);
                }
                b'f' | b'F' => emit_float_fixed(&mut out, &spec, conv == b'F', va_ref),
                b'c' | b's' => emit_str_char(&mut out, &spec, conv == b's', va_ref),
                b'%' => out.push(b'%'),
                other => {
                    out.push(b'%');
                    out.push(other);
                }
            }
        }
    }
    if out.has_buf {
        let n = out.total.min(out.cap);
        // SAFETY: `n <= out.cap = size - 1`, within `size` bytes.
        unsafe { *out.buf.add(n) = 0 };
    }
    i32::try_from(out.total).unwrap_or(i32::MAX)
}

/// Formats syslog line capped at 79B into `out`, stripping trailing `\r\n `.
///
/// # Safety
/// `fmt` must be NUL-terminated and `va` must match `fmt` specifiers.
pub unsafe fn format_syslog_line(
    out: &mut [u8; 80],
    fmt: *const u8,
    va: *mut XtensaVaList,
) -> usize {
    // SAFETY: `out` is valid for 79 bytes; `fmt`/`va` per caller contract.
    let _ = unsafe { rust_vsnprintf(out.as_mut_ptr(), 79, fmt, va) };
    out[79] = 0;
    let mut len = 0usize;
    while len < 79 && out[len] != 0 {
        len += 1;
    }
    while len > 0 && matches!(out[len - 1], b'\n' | b'\r' | b' ') {
        len -= 1;
    }
    out[len] = 0;
    len
}

#[cfg(test)]
mod tests {
    use super::{format_syslog_line, rust_vsnprintf, XtensaVaList};

    #[repr(C, align(8))]
    struct AlignedWords<const N: usize>([u32; N]);

    fn pack64(dst: &mut [u32], idx: usize, v: u64) {
        (dst[idx], dst[idx + 1]) = (v as u32, (v >> 32) as u32);
    }

    // SAFETY: `fmt` must be NUL-terminated and `reg`/`stk` must match `fmt`.
    unsafe fn run_fmt<const N: usize, const R: usize, const S: usize>(
        fmt: &[u8],
        reg: &AlignedWords<R>,
        stk: &AlignedWords<S>,
    ) -> ([u8; N], usize, i32) {
        let mut out = [0u8; N];
        let mut va = XtensaVaList {
            va_stk: stk.0.as_ptr(),
            va_reg: reg.0.as_ptr(),
            va_ndx: 4,
        };
        // SAFETY: `out`, `fmt`, and `va` are valid for the test call.
        let ret = unsafe { rust_vsnprintf(out.as_mut_ptr(), N, fmt.as_ptr(), &mut va) };
        let mut len = 0usize;
        while len < N && out[len] != 0 {
            len += 1;
        }
        (out, len, ret)
    }

    #[test]
    fn reg_spill_and_u64_f64() {
        let (reg, stk) = (
            AlignedWords([0, 7, 0x0A, 0xFF, 0, 0]),
            AlignedWords([0u32; 12]),
        );
        // SAFETY: `reg` and `stk` match `%d %02x:%02x`.
        let (buf, len, ret) = unsafe { run_fmt::<32, 6, 12>(b"%d %02x:%02x\0", &reg, &stk) };
        assert_eq!((ret, &buf[..len]), (7, b"7 0a:ff".as_slice()));

        let reg7 = AlignedWords([0, 1, 2, 3, 4, 5]);
        let mut stk7 = AlignedWords([0u32; 12]);
        (stk7.0[8], stk7.0[9]) = (6, 7);
        // SAFETY: Seven `i32` slots provided across `reg7` and `stk7[8..=9]`.
        let (b2, l2, r2) = unsafe { run_fmt::<32, 6, 12>(b"%d %d %d %d %d %d %d\0", &reg7, &stk7) };
        assert_eq!((r2, &b2[..l2]), (13, b"1 2 3 4 5 6 7".as_slice()));

        let (mut r64, mut s64) = (AlignedWords([0u32; 6]), AlignedWords([0u32; 14]));
        pack64(&mut r64.0, 2, (-123_456_789_012_345i64) as u64);
        pack64(&mut r64.0, 4, 9_876_543_210_987u64);
        pack64(&mut s64.0, 8, 12.3456_f64.to_bits());
        pack64(&mut s64.0, 10, (-1.2_f64).to_bits());
        // SAFETY: `r64` and `s64` hold paired 64-bit values matching format.
        let (b3, l3, _) = unsafe { run_fmt::<64, 6, 14>(b"%lld %llu %.4f %-6.2f\0", &r64, &s64) };
        assert_eq!(&b3[..l3], b"-123456789012345 9876543210987 12.3456 -1.20 ");
    }

    #[test]
    fn padding_star_null_trunc_syslog() {
        let (mut reg, mut stk) = (AlignedWords([0u32; 6]), AlignedWords([0u32; 12]));
        let addr = c"ok".as_ptr() as usize;
        reg.0[1] = u32::try_from(addr & 0xFFFF_FFFF).unwrap_or(0);
        let i = if cfg!(target_pointer_width = "64") {
            reg.0[2] = u32::try_from(addr >> 32).unwrap_or(0);
            3
        } else {
            2
        };
        (reg.0[i], reg.0[i + 1], reg.0[i + 2], stk.0[8]) = (42, 0x1A2B_3C4D, 7, 0xAB);
        let (fmt, exp): (&[u8], &[u8]) = if cfg!(target_pointer_width = "64") {
            (
                b"%-8s %-8u %08lX %03d %%\0",
                b"ok       42       1A2B3C4D 007 %",
            )
        } else {
            (
                b"%-8s %-8u %08lX %03d %02X %%\0",
                b"ok       42       1A2B3C4D 007 AB %",
            )
        };
        // SAFETY: `reg`, `stk`, and `fmt` match across pointer widths.
        let (buf, len, _) = unsafe { run_fmt::<64, 6, 12>(fmt, &reg, &stk) };
        assert_eq!(&buf[..len], exp);

        let reg_star = AlignedWords([0, 5, 3, 9, 0, 0]);
        // SAFETY: `reg_star` holds width, prec, `u32`, and null pointer.
        let (bs, ls, _) = unsafe { run_fmt::<32, 6, 12>(b"%*.*u %s\0", &reg_star, &stk) };
        assert_eq!(&bs[..ls], b"  009 (null)");

        let reg_hex = AlignedWords([0, 0x0A, 0xFF, 0, 0, 0]);
        // SAFETY: `reg_hex` provides two `u32` values.
        let (bt, lt, rt) = unsafe { run_fmt::<5, 6, 12>(b"%02X-%02X\0", &reg_hex, &stk) };
        assert_eq!((rt, &bt[..lt], bt[4]), (5, b"0A-F".as_slice(), 0));
        let mut va0 = XtensaVaList {
            va_stk: stk.0.as_ptr(),
            va_reg: reg_hex.0.as_ptr(),
            va_ndx: 4,
        };
        let zf = c"%02X-%02X".as_ptr().cast();
        // SAFETY: Null buffer with `size == 0` performs no memory writes.
        let r0 = unsafe { rust_vsnprintf(core::ptr::null_mut(), 0, zf, &mut va0) };
        assert_eq!(r0, 5);

        let mut sys = [b'x'; 80];
        let mut vas = XtensaVaList {
            va_stk: stk.0.as_ptr(),
            va_reg: reg_hex.0.as_ptr(),
            va_ndx: 4,
        };
        let sf = c"%02X \r\n ".as_ptr().cast();
        // SAFETY: `sys` is 80 bytes and `vas` matches format.
        let slen = unsafe { format_syslog_line(&mut sys, sf, &mut vas) };
        assert_eq!((slen, &sys[..slen], sys[79]), (2, b"0A".as_slice(), 0));
        let mut vas2 = XtensaVaList {
            va_stk: stk.0.as_ptr(),
            va_reg: reg_hex.0.as_ptr(),
            va_ndx: 4,
        };
        let sf2 = c"%082d\r\n".as_ptr().cast();
        // SAFETY: `sys` is 80 bytes and `%082d` tests 79-byte cap.
        let slen2 = unsafe { format_syslog_line(&mut sys, sf2, &mut vas2) };
        assert_eq!((slen2, sys[78], sys[79]), (78, 0, 0));
    }
}
