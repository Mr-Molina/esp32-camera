//! Pure `#![no_std]` Xtensa Call8 varargs (`XtensaVaList`) and bounded
//! `vsnprintf` / syslog line formatter for `esp32-cam-rf-clock`.
// JUSTIFICATION: Bare-metal `#![no_std]` C ABI varargs and `vsnprintf` require
// raw pointer reads/writes into caller buffers and ABI register/stack frames.

const F_LEFT: u8 = 1;
const F_PLUS: u8 = 2;
const F_SPACE: u8 = 4;
const F_ZERO: u8 = 8;
const F_ALT: u8 = 16;

/// Upper bound for parsed field width and precision (`i32::MAX`) to prevent
/// arithmetic overflow in padding calculations across C ABI boundaries.
const MAX_FMT_WIDTH: usize = i32::MAX as usize;

/// Maximum `u32` word offset within Rust's `isize::MAX` `ptr::add` byte-offset
/// precondition.
const MAX_VA_WORD_OFFSET: usize = (isize::MAX as usize) / core::mem::size_of::<u32>();

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
/// Returns `0` without advancing `va.va_ndx` or dereferencing memory if
/// advancing `va.va_ndx` would overflow or exceed `isize::MAX` byte bounds.
///
/// # Safety
/// `va.va_reg` and `va.va_stk` must be valid for 4-byte aligned `u32` reads at
/// the slot selected by `va.va_ndx` per the Xtensa Call8 ABI.
pub unsafe fn va_arg_u32(va: &mut XtensaVaList) -> u32 {
    let Some(mut next) = va.va_ndx.checked_add(4) else {
        return 0;
    };
    if next > 0x18 && va.va_ndx < 0x19 {
        next = 0x24;
    }
    let Some(slot_byte_off) = next.checked_sub(4) else {
        return 0;
    };
    let word_off = slot_byte_off / 4;
    if word_off > MAX_VA_WORD_OFFSET {
        return 0;
    }
    let base = if next > 0x18 { va.va_stk } else { va.va_reg };
    va.va_ndx = next;
    if base.is_null() {
        return 0;
    }
    // SAFETY: Caller guarantees `base` is valid and 4-byte aligned at slot;
    // `slot_byte_off` is checked against underflow and `isize::MAX`.
    unsafe { *base.add(word_off) }
}

/// Fetches the next 8-byte aligned 64-bit word from an [`XtensaVaList`].
///
/// Returns `0` without advancing `va.va_ndx` or dereferencing memory if
/// aligning or advancing `va.va_ndx` would overflow or exceed `isize::MAX`.
///
/// # Safety
/// `va.va_reg` and `va.va_stk` must be valid for 4-byte aligned `u32` reads at
/// the aligned 8-byte slot selected by `va.va_ndx`.
pub unsafe fn va_arg_u64(va: &mut XtensaVaList) -> u64 {
    let Some(aligned_sum) = va.va_ndx.checked_add(7) else {
        return 0;
    };
    let aligned = aligned_sum & !7usize;
    let Some(mut next) = aligned.checked_add(8) else {
        return 0;
    };
    if next > 0x18 && aligned < 0x19 {
        next = 0x28;
    }
    let (Some(lo_byte_off), Some(hi_byte_off)) = (next.checked_sub(8), next.checked_sub(4)) else {
        return 0;
    };
    let (lo_off, hi_off) = (lo_byte_off / 4, hi_byte_off / 4);
    if lo_off > MAX_VA_WORD_OFFSET || hi_off > MAX_VA_WORD_OFFSET {
        return 0;
    }
    let base = if next > 0x18 { va.va_stk } else { va.va_reg };
    va.va_ndx = next;
    if base.is_null() {
        return 0;
    }
    // SAFETY: Caller guarantees `base` is valid at `lo_off` and `hi_off`; both
    // word offsets are checked against underflow and `isize::MAX`.
    let (lo, hi) = unsafe { (*base.add(lo_off), *base.add(hi_off)) };
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
            // SAFETY: `self.buf` is non-null and `self.total < self.cap <= isize::MAX`.
            unsafe { *self.buf.add(self.total) = b };
        }
        self.total = self.total.saturating_add(1);
    }

    fn rep(&mut self, b: u8, n: usize) {
        if self.has_buf && self.total < self.cap {
            let writable = self.cap.saturating_sub(self.total).min(n);
            if writable > 0 {
                // SAFETY: `self.buf` is non-null and writable for `self.cap + 1`
                // bytes; `self.total + writable <= self.cap <= isize::MAX`.
                unsafe {
                    core::ptr::write_bytes(self.buf.add(self.total), b, writable);
                }
            }
        }
        self.total = self.total.saturating_add(n);
    }
}

/// Raw-pointer format-string cursor that never forms a `&[u8]` reference across
/// output buffer writes.
struct RawFmtCursor {
    ptr: *const u8,
    idx: usize,
}

impl RawFmtCursor {
    const fn new(ptr: *const u8) -> Self {
        Self { ptr, idx: 0 }
    }

    // SAFETY: `self.ptr` must point to a valid NUL-terminated C string not
    // mutated during formatting, and `self.idx` must not advance past NUL.
    unsafe fn peek(&self) -> Option<u8> {
        if self.idx >= isize::MAX as usize {
            return None;
        }
        // SAFETY: `self.idx < isize::MAX` and caller guarantees `self.ptr` is
        // readable through its terminating NUL byte.
        let b = unsafe { *self.ptr.add(self.idx) };
        if b == 0 {
            None
        } else {
            Some(b)
        }
    }

    // SAFETY: Same as `peek`; caller must only pass `delta == 1` when `peek()`
    // returned `Some(_)`, ensuring `self.idx + 1` is at most the NUL byte.
    unsafe fn peek_at(&self, delta: usize) -> Option<u8> {
        let off = self.idx.checked_add(delta)?;
        if off >= isize::MAX as usize {
            return None;
        }
        // SAFETY: `off < isize::MAX` and `self.idx` is strictly before NUL.
        let b = unsafe { *self.ptr.add(off) };
        if b == 0 {
            None
        } else {
            Some(b)
        }
    }

    fn advance(&mut self, n: usize) {
        self.idx = self.idx.saturating_add(n);
    }
}

#[derive(Default)]
struct FmtSpec {
    flags: u8,
    width: usize,
    prec: Option<usize>,
    bits: u8,
}

// SAFETY: `fmt` must satisfy `RawFmtCursor::peek` preconditions.
unsafe fn parse_uint(fmt: &mut RawFmtCursor) -> usize {
    let mut val = 0usize;
    // SAFETY: `fmt` points to a valid NUL-terminated format string.
    while let Some(b) = unsafe { fmt.peek() } {
        if !b.is_ascii_digit() {
            break;
        }
        val = val
            .saturating_mul(10)
            .saturating_add(usize::from(b - b'0'))
            .min(MAX_FMT_WIDTH);
        fmt.advance(1);
    }
    val
}

// SAFETY: `fmt` must satisfy `RawFmtCursor::peek` preconditions, and `va` must
// hold `i32` args for any `*` width or precision fields.
unsafe fn parse_spec(fmt: &mut RawFmtCursor, va: &mut XtensaVaList) -> FmtSpec {
    let mut s = FmtSpec {
        bits: 32,
        ..FmtSpec::default()
    };
    // SAFETY: `fmt` points to a valid NUL-terminated format string.
    while let Some(b) = unsafe { fmt.peek() } {
        s.flags |= match b {
            b'-' => F_LEFT,
            b'+' => F_PLUS,
            b' ' => F_SPACE,
            b'0' => F_ZERO,
            b'#' => F_ALT,
            _ => break,
        };
        fmt.advance(1);
    }
    // SAFETY: `fmt` points to a valid NUL-terminated format string.
    let width_head = unsafe { fmt.peek() };
    if width_head == Some(b'*') {
        fmt.advance(1);
        // SAFETY: Caller guarantees `va` holds an `i32` width argument.
        let w = unsafe { va_arg_u32(va) } as i32;
        if w < 0 {
            s.flags |= F_LEFT;
        }
        s.width = usize::try_from(w.unsigned_abs())
            .unwrap_or(MAX_FMT_WIDTH)
            .min(MAX_FMT_WIDTH);
    } else if width_head.is_some_and(|b| b.is_ascii_digit()) {
        // SAFETY: `fmt` points to a valid NUL-terminated format string.
        s.width = unsafe { parse_uint(fmt) };
    }
    // SAFETY: `fmt` points to a valid NUL-terminated format string.
    if unsafe { fmt.peek() } == Some(b'.') {
        fmt.advance(1);
        // SAFETY: `fmt` points to a valid NUL-terminated format string.
        if unsafe { fmt.peek() } == Some(b'*') {
            fmt.advance(1);
            // SAFETY: Caller guarantees `va` holds an `i32` precision arg.
            s.prec = usize::try_from(unsafe { va_arg_u32(va) } as i32)
                .ok()
                .map(|p| p.min(MAX_FMT_WIDTH));
        } else {
            // SAFETY: `fmt` points to a valid NUL-terminated format string.
            s.prec = Some(unsafe { parse_uint(fmt) });
        }
    }
    // SAFETY: `peek_at(1)` is only queried when `peek()` returned `Some(_)`,
    // so `idx + 1` is at most the NUL terminator.
    let c0 = unsafe { fmt.peek() };
    let c1 = if c0.is_some() {
        // SAFETY: `c0` is non-NUL, so `idx + 1` is in-bounds.
        unsafe { fmt.peek_at(1) }
    } else {
        None
    };
    let (adv, bits) = match (c0, c1) {
        (Some(b'h'), Some(b'h')) => (2, 8),
        (Some(b'h'), _) => (1, 16),
        (Some(b'l'), Some(b'l')) => (2, 64),
        // C99 `j` (`intmax_t`/`uintmax_t`) and `L` (Xtensa 64-bit `long double`)
        // occupy 64-bit aligned vararg slots; `l`, `z` (`size_t`), and `t`
        // (`ptrdiff_t`) are 32-bit in the Xtensa Call8 ILP32 ABI.
        (Some(b'j' | b'L'), _) => (1, 64),
        (Some(b'l' | b'z' | b't'), _) => (1, 32),
        _ => (0, 32),
    };
    fmt.advance(adv);
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
    let width = s.width.min(MAX_FMT_WIDTH);
    let zfill = zfill.min(MAX_FMT_WIDTH);
    let used = pfx.len().saturating_add(zfill).saturating_add(body.len());
    let pad = width.saturating_sub(used);
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
    let prec_pad = s
        .prec
        .map_or(0, |p| p.min(MAX_FMT_WIDTH).saturating_sub(body.len()));
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

// SAFETY: `ptr` must be non-null and readable for `len` bytes without overlap
// with `out.buf`.
unsafe fn emit_str_raw(out: &mut FmtBuf, s: &FmtSpec, ptr: *const u8, len: usize) {
    let width = s.width.min(MAX_FMT_WIDTH);
    let pad = width.saturating_sub(len);
    let left = (s.flags & F_LEFT) != 0;
    if !left {
        out.rep(b' ', pad);
    }
    if out.has_buf && out.total < out.cap {
        let writable = out.cap.saturating_sub(out.total).min(len);
        for i in 0..writable {
            // SAFETY: `i < writable <= len <= isize::MAX` and caller guarantees
            // `ptr` is readable for `len` bytes; reads via raw pointer avoid
            // holding a `&[u8]` reference across writes into `out.buf`.
            let b = unsafe { *ptr.add(i) };
            out.push(b);
        }
        out.total = out.total.saturating_add(len - writable);
    } else {
        out.total = out.total.saturating_add(len);
    }
    if left {
        out.rep(b' ', pad);
    }
}

// SAFETY: For `%s`, non-null pointer must be readable up to NUL or `s.prec`
// and must not alias `out.buf`.
unsafe fn emit_str_char(out: &mut FmtBuf, s: &FmtSpec, is_str: bool, va: &mut XtensaVaList) {
    if !is_str {
        // SAFETY: Caller guarantees `va` holds a `u32` char argument.
        let ch = (unsafe { va_arg_u32(va) } & 0xFF) as u8;
        emit_pad(out, s, b"", 0, &[ch], false);
        return;
    }
    // SAFETY: Caller guarantees `va` holds a pointer-width `%s` argument.
    let ptr = unsafe { va_arg_ptr(va) } as *const u8;
    let max_len = s
        .prec
        .unwrap_or(isize::MAX as usize)
        .min(isize::MAX as usize);
    if ptr.is_null() {
        emit_pad(out, s, b"", 0, &b"(null)"[..6usize.min(max_len)], false);
        return;
    }
    let mut len = 0usize;
    while len < max_len {
        // SAFETY: Caller guarantees `ptr` is readable through NUL or `max_len`,
        // and `len < max_len <= isize::MAX`.
        if unsafe { *ptr.add(len) } == 0 {
            break;
        }
        len += 1;
    }
    // SAFETY: `ptr` was verified readable for `len` bytes above without
    // creating a `&[u8]` reference across `out.buf` writes.
    unsafe { emit_str_raw(out, s, ptr, len) };
}

/// Formats `fmt` with Xtensa Call8 `va` into `buf` (`size` bytes).
///
/// # Safety
/// - When `buf` is non-null and `size > 0`, `buf` must be writable for
///   `size.min(isize::MAX as usize)` bytes (or the formatted length plus NUL
///   for unbounded `sprintf` callers) and must not alias `fmt`, `va`, or any
///   `%s` argument string.
/// - When `fmt` is non-null, it must point to a valid NUL-terminated byte
///   sequence not mutated during formatting.
/// - When `va` is non-null, it must be properly aligned for [`XtensaVaList`]
///   (4-byte aligned on Xtensa), valid for mutable access, non-aliasing with
///   `buf`/`fmt`/`%s`, and match `fmt` specifiers.
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
    let mut out = FmtBuf {
        buf,
        cap: size.saturating_sub(1).min(isize::MAX as usize),
        has_buf,
        total: 0,
    };
    // SAFETY: Caller guarantees `va` is properly aligned and valid for mutable
    // access without aliasing `buf` or `fmt`.
    let va_ref = unsafe { &mut *va };
    let mut cur = RawFmtCursor::new(fmt);
    // SAFETY: Caller guarantees `fmt` is NUL-terminated and immutable during
    // formatting; raw-pointer reads avoid holding `&[u8]` across `out` writes.
    while let Some(ch) = unsafe { cur.peek() } {
        cur.advance(1);
        if ch != b'%' {
            out.push(ch);
            continue;
        }
        // SAFETY: `cur` still points within the NUL-terminated `fmt` string.
        if unsafe { cur.peek() } == Some(b'%') {
            cur.advance(1);
            out.push(b'%');
            continue;
        }
        // SAFETY: `cur` is valid and `va_ref` holds any `*` width/prec args.
        let spec = unsafe { parse_spec(&mut cur, va_ref) };
        // SAFETY: `cur` points within the NUL-terminated `fmt` string.
        let Some(conv) = (unsafe { cur.peek() }) else {
            break;
        };
        cur.advance(1);
        // SAFETY: `va_ref` holds argument matching `conv` per `fmt`.
        unsafe {
            match conv {
                b'd' | b'i' | b'u' | b'x' | b'X' | b'o' | b'p' => {
                    emit_int(&mut out, &spec, conv, va_ref);
                }
                b'f' | b'F' | b'e' | b'E' | b'g' | b'G' | b'a' | b'A' => {
                    emit_float_fixed(&mut out, &spec, conv.is_ascii_uppercase(), va_ref);
                }
                b'c' | b's' => emit_str_char(&mut out, &spec, conv == b's', va_ref),
                // Safely consume `%n` pointer slot as a strict no-op (never
                // write through caller pointer) so `va_ref` stays in sync.
                b'n' => {
                    let _ = va_arg_ptr(va_ref);
                }
                b'%' => out.push(b'%'),
                other => {
                    out.push(b'%');
                    out.push(other);
                    // Stop on unknown specifier so `va_ref` cannot desync.
                    break;
                }
            }
        }
    }
    if out.has_buf {
        let n = out.total.min(out.cap);
        // SAFETY: `n <= out.cap <= isize::MAX`, within `size` bytes.
        unsafe { *out.buf.add(n) = 0 };
    }
    i32::try_from(out.total).unwrap_or(i32::MAX)
}

/// Formats syslog line capped at 79B into `out`, stripping trailing `\r\n `.
///
/// # Safety
/// `fmt` must be NUL-terminated, `va` must be properly aligned for
/// [`XtensaVaList`] and match `fmt` specifiers, and neither `fmt`, `va`, nor
/// `%s` strings may overlap `out`.
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
    use super::{format_syslog_line, rust_vsnprintf, va_arg_u32, va_arg_u64, XtensaVaList};

    #[repr(C, align(8))]
    struct AlignedWords<const N: usize>([u32; N]);

    fn pack64(dst: &mut [u32], idx: usize, v: u64) {
        (dst[idx], dst[idx + 1]) = (v as u32, (v >> 32) as u32);
    }

    fn pack_ptr(dst: &mut [u32], idx: usize, addr: usize) -> usize {
        dst[idx] = u32::try_from(addr & 0xFFFF_FFFF).unwrap_or(0);
        if cfg!(target_pointer_width = "64") {
            dst[idx + 1] = u32::try_from(addr >> 32).unwrap_or(0);
            idx + 2
        } else {
            idx + 1
        }
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
        let i = pack_ptr(&mut reg.0, 1, addr);
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

    #[test]
    fn saturated_width_prec_and_o1_huge_rep() {
        let reg = AlignedWords([0, 42, 0, 0, 0, 0]);
        let stk = AlignedWords([0u32; 12]);
        // Huge literal width + precision with '+' prefix must not overflow in
        // `emit_pad` and must finish in O(cap) time.
        let fmt = b"%+999999999999999999999999.999999999999999999999999d\0";
        // SAFETY: `reg` holds one `i32` argument matching `%d`.
        let (buf, len, ret) = unsafe { run_fmt::<16, 6, 12>(fmt, &reg, &stk) };
        assert_eq!(ret, i32::MAX);
        assert_eq!(len, 15);
        assert_eq!(&buf[..len], b"+00000000000000");
        assert_eq!(buf[15], 0);

        // `*` width = `i32::MIN` and `*` prec = `i32::MAX` with null/0 buffer
        // must complete in O(1) without looping 2^31 times.
        let reg_star = AlignedWords([0, i32::MIN as u32, i32::MAX as u32, 7, 0, 0]);
        let mut va = XtensaVaList {
            va_stk: stk.0.as_ptr(),
            va_reg: reg_star.0.as_ptr(),
            va_ndx: 4,
        };
        let star_fmt = c"%+*.*d".as_ptr().cast();
        // SAFETY: Null buffer with `size == 0` performs no writes.
        let ret_null = unsafe { rust_vsnprintf(core::ptr::null_mut(), 0, star_fmt, &mut va) };
        assert_eq!(ret_null, i32::MAX);

        // Huge right-aligned width into small buffer fills spaces in O(cap).
        // SAFETY: `reg` holds one `i32` argument matching `%2000000000d`.
        let (buf2, len2, ret2) = unsafe { run_fmt::<8, 6, 12>(b"%2000000000d\0", &reg, &stk) };
        assert_eq!(ret2, 2_000_000_000);
        assert_eq!(&buf2[..len2], b"       ");
        assert_eq!(buf2[7], 0);
    }

    #[test]
    fn va_ndx_overflow_guards_return_zero_without_ub() {
        let reg = AlignedWords([1, 2, 3, 4, 5, 6]);
        let stk = AlignedWords([7u32; 12]);
        for bad_ndx in [
            usize::MAX,
            usize::MAX - 1,
            usize::MAX - 3,
            usize::MAX - 7,
            (isize::MAX as usize).saturating_add(8),
        ] {
            let mut va32 = XtensaVaList {
                va_stk: stk.0.as_ptr(),
                va_reg: reg.0.as_ptr(),
                va_ndx: bad_ndx,
            };
            // SAFETY: Overflow guard returns `0` before dereferencing pointers.
            let v32 = unsafe { va_arg_u32(&mut va32) };
            assert_eq!(v32, 0);
            assert_eq!(va32.va_ndx, bad_ndx);

            let mut va64 = XtensaVaList {
                va_stk: stk.0.as_ptr(),
                va_reg: reg.0.as_ptr(),
                va_ndx: bad_ndx,
            };
            // SAFETY: Overflow guard returns `0` before dereferencing pointers.
            let v64 = unsafe { va_arg_u64(&mut va64) };
            assert_eq!(v64, 0);
            assert_eq!(va64.va_ndx, bad_ndx);
        }
    }

    #[test]
    fn c99_modifiers_float_variants_and_noop_percent_n() {
        // `%n` must consume the pointer slot without writing, keeping `%d` in sync.
        let sentinel: u32 = 0xDEAD_BEEF;
        let mut reg_n = AlignedWords([0u32; 6]);
        let stk = AlignedWords([0u32; 12]);
        let next = pack_ptr(&mut reg_n.0, 1, (&raw const sentinel) as usize);
        reg_n.0[next] = 99;
        // SAFETY: `reg_n` holds pointer for `%n` followed by `i32` for `%d`.
        let (bn, ln, rn) = unsafe { run_fmt::<32, 6, 12>(b"ab%ncd=%d\0", &reg_n, &stk) };
        assert_eq!(sentinel, 0xDEAD_BEEF, "%n must never write to memory");
        assert_eq!((rn, &bn[..ln]), (7, b"abcd=99".as_slice()));

        // `%e`, `%E`, `%g`, `%a`, `%Lf` consume `f64` slots without desyncing `%d`.
        let (mut rf, mut sf) = (AlignedWords([0u32; 6]), AlignedWords([0u32; 16]));
        pack64(&mut rf.0, 2, 1.5_f64.to_bits());
        pack64(&mut rf.0, 4, f64::INFINITY.to_bits());
        pack64(&mut sf.0, 8, 2.25_f64.to_bits());
        pack64(&mut sf.0, 10, 3.5_f64.to_bits());
        sf.0[12] = 77;
        let fmt_f = b"%.1e %.0E %.2g %.1Lf %d\0";
        // SAFETY: Four `f64` slots followed by one `i32` slot match format.
        let (bf, lf, _) = unsafe { run_fmt::<64, 6, 16>(fmt_f, &rf, &sf) };
        assert_eq!(&bf[..lf], b"1.5 INF 2.25 3.5 77");

        // C99 `%zu`, `%td`, `%jd` consume 32/32/64-bit slots without desync.
        let mut rm = AlignedWords([0u32; 6]);
        let mut sm = AlignedWords([0u32; 14]);
        rm.0[1] = 1234;
        rm.0[2] = (-56i32) as u32;
        pack64(&mut rm.0, 4, (-9_000_000_000i64) as u64);
        sm.0[8] = 88;
        // SAFETY: `rm` and `sm` match `%zu %td %jd %d` in Xtensa Call8 layout.
        let (bm, lm, _) = unsafe { run_fmt::<64, 6, 14>(b"%zu %td %jd %d\0", &rm, &sm) };
        assert_eq!(&bm[..lm], b"1234 -56 -9000000000 88");

        // `sprintf`-style `size = usize::MAX` clamps cap cleanly under Miri.
        let mut out = [0u8; 32];
        let reg_sp = AlignedWords([0, 7, 0, 0, 0, 0]);
        let mut va_sp = XtensaVaList {
            va_stk: stk.0.as_ptr(),
            va_reg: reg_sp.0.as_ptr(),
            va_ndx: 4,
        };
        let sp_fmt = c"v=%d".as_ptr().cast();
        // SAFETY: `out` has 32 bytes, sufficient for `"v=7\0"`; `usize::MAX`
        // tests cap clamping to `isize::MAX`.
        let rsp = unsafe { rust_vsnprintf(out.as_mut_ptr(), usize::MAX, sp_fmt, &mut va_sp) };
        assert_eq!((rsp, &out[..4]), (3, b"v=7\0".as_slice()));
    }
}
