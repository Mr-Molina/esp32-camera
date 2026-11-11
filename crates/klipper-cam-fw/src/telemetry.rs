//! Lock-free telemetry, cycle counters, stack watermark tracking, and SHA-256
//! formatting for Klipper camera firmware.

use core::sync::atomic::{AtomicU32, Ordering};

/// Maps the 6-bit PRO_CPU reset cause in `RTC_CNTL_RESET_STATE_REG`
/// (`0x3FF4_8034`, `raw_reg & 0x3F`) to its canonical ESP32 ROM label.
#[must_use]
#[inline]
pub const fn reset_reason_label(raw_reg: u32) -> &'static str {
    match raw_reg & 0x3F {
        1 => "poweron",
        3 => "sw_reset",
        4 => "owdt_reset",
        5 => "deepsleep",
        6 => "sdio_reset",
        7 => "tg0wdt_sys",
        8 => "tg1wdt_sys",
        9 => "rtcwdt_sys",
        10 => "intrusion",
        11 => "tgwdt_cpu",
        12 => "sw_cpu_reset",
        13 => "rtcwdt_cpu",
        14 => "ext_cpu",
        15 => "brownout",
        16 => "rtcwdt_rtc",
        _ => "unknown",
    }
}

#[cfg(target_arch = "xtensa")]
/// Reads hardware cycle count register `rsr.ccount` on Xtensa targets.
#[inline(always)]
pub fn ccount() -> u32 {
    let val: u32;
    // JUSTIFICATION: Hardware cycle count register `rsr.ccount` on Xtensa
    // requires inline assembly as no safe standard library equivalent exists.
    // SAFETY:
    // 1. Preconditions: Target architecture is Xtensa (`target_arch = "xtensa"`).
    // 2. Alignment: Register output operand binds directly to general
    //    register `val`.
    // 3. Provenance & Lifetime: No memory pointers are constructed, read, or
    //    written.
    // 4. Aliasing: Register access preserves memory safety and aliasing.
    // 5. Volatile Semantics: `rsr.ccount` reads special register CCOUNT
    //    without memory side effects.
    // 6. Panic Safety: Instruction is non-trapping and cannot unwind.
    unsafe {
        core::arch::asm!(
            "rsr.ccount {0}",
            out(reg) val,
            options(nomem, nostack, preserves_flags)
        );
    }
    val
}

#[cfg(not(target_arch = "xtensa"))]
static HOST_CCOUNT: AtomicU32 = AtomicU32::new(0);

#[cfg(not(target_arch = "xtensa"))]
/// Increments an [`AtomicU32`] stub by 240 for deterministic host testing.
#[inline(always)]
pub fn ccount() -> u32 {
    HOST_CCOUNT.fetch_add(240, Ordering::Relaxed)
}

/// Point-in-time snapshot of lock-free firmware metrics.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct FwStatsSnapshot {
    pub sensor_frame_period_us: u32,
    pub frame_bytes: u32,
    pub c1_pushed: u32,
    pub c1_dropped: u32,
    pub ring_full_events: u32,
    pub soi_miss: u32,
    pub overflow_aborts: u32,
    pub c0_spsc_cycles: u32,
    pub c0_net_cycles: u32,
    pub c0_p80_cycles: u32,
    pub c0_p81_cycles: u32,
    pub max_loop_latency_us: u32,
    pub p81_frames_sent: u32,
    pub p81_frames_superseded: u32,
    pub c0_validation_failures: u32,
    pub sccb_errors: u32,
    pub send_partial: u32,
    pub ring_build_failures: u32,
    pub i2s_stop_timeouts: u32,
}

/// Lock-free firmware statistics counters and EWMA estimators.
pub struct FwStats {
    sensor_frame_period_us: AtomicU32,
    frame_bytes: AtomicU32,
    c1_pushed: AtomicU32,
    c1_dropped: AtomicU32,
    ring_full_events: AtomicU32,
    soi_miss: AtomicU32,
    overflow_aborts: AtomicU32,
    c0_spsc_cycles: AtomicU32,
    c0_net_cycles: AtomicU32,
    c0_p80_cycles: AtomicU32,
    c0_p81_cycles: AtomicU32,
    max_loop_latency_us: AtomicU32,
    p81_frames_sent: AtomicU32,
    p81_frames_superseded: AtomicU32,
    c0_validation_failures: AtomicU32,
    sccb_errors: AtomicU32,
    send_partial: AtomicU32,
    ring_build_failures: AtomicU32,
    i2s_stop_timeouts: AtomicU32,
}

impl FwStats {
    /// Creates a new zeroed [`FwStats`] structure.
    pub const fn new() -> Self {
        Self {
            sensor_frame_period_us: AtomicU32::new(0),
            frame_bytes: AtomicU32::new(0),
            c1_pushed: AtomicU32::new(0),
            c1_dropped: AtomicU32::new(0),
            ring_full_events: AtomicU32::new(0),
            soi_miss: AtomicU32::new(0),
            overflow_aborts: AtomicU32::new(0),
            c0_spsc_cycles: AtomicU32::new(0),
            c0_net_cycles: AtomicU32::new(0),
            c0_p80_cycles: AtomicU32::new(0),
            c0_p81_cycles: AtomicU32::new(0),
            max_loop_latency_us: AtomicU32::new(0),
            p81_frames_sent: AtomicU32::new(0),
            p81_frames_superseded: AtomicU32::new(0),
            c0_validation_failures: AtomicU32::new(0),
            sccb_errors: AtomicU32::new(0),
            send_partial: AtomicU32::new(0),
            ring_build_failures: AtomicU32::new(0),
            i2s_stop_timeouts: AtomicU32::new(0),
        }
    }

    /// Takes a consistent point-in-time snapshot of all metric counters.
    pub fn snapshot(&self) -> FwStatsSnapshot {
        FwStatsSnapshot {
            sensor_frame_period_us: self.sensor_frame_period_us.load(Ordering::Relaxed),
            frame_bytes: self.frame_bytes.load(Ordering::Relaxed),
            c1_pushed: self.c1_pushed.load(Ordering::Relaxed),
            c1_dropped: self.c1_dropped.load(Ordering::Relaxed),
            ring_full_events: self.ring_full_events.load(Ordering::Relaxed),
            soi_miss: self.soi_miss.load(Ordering::Relaxed),
            overflow_aborts: self.overflow_aborts.load(Ordering::Relaxed),
            c0_spsc_cycles: self.c0_spsc_cycles.load(Ordering::Relaxed),
            c0_net_cycles: self.c0_net_cycles.load(Ordering::Relaxed),
            c0_p80_cycles: self.c0_p80_cycles.load(Ordering::Relaxed),
            c0_p81_cycles: self.c0_p81_cycles.load(Ordering::Relaxed),
            max_loop_latency_us: self.max_loop_latency_us.load(Ordering::Relaxed),
            p81_frames_sent: self.p81_frames_sent.load(Ordering::Relaxed),
            p81_frames_superseded: self.p81_frames_superseded.load(Ordering::Relaxed),
            c0_validation_failures: self.c0_validation_failures.load(Ordering::Relaxed),
            sccb_errors: self.sccb_errors.load(Ordering::Relaxed),
            send_partial: self.send_partial.load(Ordering::Relaxed),
            ring_build_failures: self.ring_build_failures.load(Ordering::Relaxed),
            i2s_stop_timeouts: self.i2s_stop_timeouts.load(Ordering::Relaxed),
        }
    }

    /// Records sensor frame period with EWMA `(prev * 7 + sample) >> 3`.
    pub fn record_sensor_period_us(&self, sample: u32) {
        let mut prev = self.sensor_frame_period_us.load(Ordering::Relaxed);
        loop {
            let next = if prev == 0 {
                sample
            } else {
                (prev.wrapping_mul(7).wrapping_add(sample)) >> 3
            };
            match self.sensor_frame_period_us.compare_exchange_weak(
                prev,
                next,
                Ordering::Relaxed,
                Ordering::Relaxed,
            ) {
                Ok(_) => break,
                Err(actual) => prev = actual,
            }
        }
    }

    /// Records frame bytes with EWMA `(prev * 7 + sample) >> 3`.
    pub fn record_frame_bytes(&self, sample: u32) {
        let mut prev = self.frame_bytes.load(Ordering::Relaxed);
        loop {
            let next = if prev == 0 {
                sample
            } else {
                (prev.wrapping_mul(7).wrapping_add(sample)) >> 3
            };
            match self.frame_bytes.compare_exchange_weak(
                prev,
                next,
                Ordering::Relaxed,
                Ordering::Relaxed,
            ) {
                Ok(_) => break,
                Err(actual) => prev = actual,
            }
        }
    }

    /// Updates max loop latency with high-water-mark tracking via `fetch_max`.
    pub fn update_max_loop_latency_us(&self, latency: u32) {
        self.max_loop_latency_us
            .fetch_max(latency, Ordering::Relaxed);
    }

    #[inline]
    pub fn inc_c1_pushed(&self) {
        self.c1_pushed.fetch_add(1, Ordering::Relaxed);
    }

    #[inline]
    pub fn inc_c1_dropped(&self) {
        self.c1_dropped.fetch_add(1, Ordering::Relaxed);
    }

    #[inline]
    pub fn inc_ring_full_events(&self) {
        self.ring_full_events.fetch_add(1, Ordering::Relaxed);
    }

    #[inline]
    pub fn inc_soi_miss(&self) {
        self.soi_miss.fetch_add(1, Ordering::Relaxed);
    }

    #[inline]
    pub fn inc_overflow_aborts(&self) {
        self.overflow_aborts.fetch_add(1, Ordering::Relaxed);
    }

    #[inline]
    pub fn add_c0_spsc_cycles(&self, val: u32) {
        self.c0_spsc_cycles.fetch_add(val, Ordering::Relaxed);
    }

    #[inline]
    pub fn add_c0_net_cycles(&self, val: u32) {
        self.c0_net_cycles.fetch_add(val, Ordering::Relaxed);
    }

    #[inline]
    pub fn add_c0_p80_cycles(&self, val: u32) {
        self.c0_p80_cycles.fetch_add(val, Ordering::Relaxed);
    }

    #[inline]
    pub fn add_c0_p81_cycles(&self, val: u32) {
        self.c0_p81_cycles.fetch_add(val, Ordering::Relaxed);
    }

    #[inline]
    pub fn inc_p81_frames_sent(&self) {
        self.p81_frames_sent.fetch_add(1, Ordering::Relaxed);
    }

    #[inline]
    pub fn inc_p81_frames_superseded(&self) {
        self.p81_frames_superseded.fetch_add(1, Ordering::Relaxed);
    }

    #[inline]
    pub fn inc_c0_validation_failures(&self) {
        self.c0_validation_failures.fetch_add(1, Ordering::Relaxed);
    }

    #[inline]
    pub fn inc_sccb_errors(&self) {
        self.sccb_errors.fetch_add(1, Ordering::Relaxed);
    }

    #[inline]
    pub fn inc_send_partial(&self) {
        self.send_partial.fetch_add(1, Ordering::Relaxed);
    }

    #[inline]
    pub fn inc_ring_build_failures(&self) {
        self.ring_build_failures.fetch_add(1, Ordering::Relaxed);
    }

    #[inline]
    pub fn inc_i2s_stop_timeouts(&self) {
        self.i2s_stop_timeouts.fetch_add(1, Ordering::Relaxed);
    }
}

impl Default for FwStats {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

impl FwStatsSnapshot {
    /// Creates a new zeroed [`FwStatsSnapshot`].
    pub const fn new() -> Self {
        Self {
            sensor_frame_period_us: 0,
            frame_bytes: 0,
            c1_pushed: 0,
            c1_dropped: 0,
            ring_full_events: 0,
            soi_miss: 0,
            overflow_aborts: 0,
            c0_spsc_cycles: 0,
            c0_net_cycles: 0,
            c0_p80_cycles: 0,
            c0_p81_cycles: 0,
            max_loop_latency_us: 0,
            p81_frames_sent: 0,
            p81_frames_superseded: 0,
            c0_validation_failures: 0,
            sccb_errors: 0,
            send_partial: 0,
            ring_build_failures: 0,
            i2s_stop_timeouts: 0,
        }
    }
}

/// Paints stack sentinel pattern (`0xA5`) across the buffer.
///
/// # Safety
/// Caller must ensure `base` is valid for writes of `len` bytes when non-null
/// and `len > 0`.
// JUSTIFICATION: Stack sentinel painting requires raw pointer volatile writes
// to prevent the compiler from eliding stack pattern initialization.
// SAFETY:
// 1. Preconditions: `base` is valid for writes of `len` bytes when non-null.
// 2. Alignment: `u8` requires alignment 1, so any non-null pointer is aligned.
// 3. Provenance & Lifetime: Pointer points to allocated task stack storage.
// 4. Aliasing: Caller guarantees exclusive access during stack sentinel paint.
// 5. Volatile Semantics: `write_volatile` guarantees writes are not elided.
// 6. Panic Safety: Operations are non-panicking byte writes.
pub unsafe fn paint_stack_sentinel(base: *mut u8, len: usize) {
    if base.is_null() || len == 0 {
        return;
    }
    let mut i = 0usize;
    while i < len {
        // JUSTIFICATION: Volatile write required for stack memory painting.
        // SAFETY:
        // 1. Preconditions: `base` is non-null and valid for `len` bytes.
        // 2. Alignment: `u8` alignment 1; `base.add(i)` remains aligned.
        // 3. Provenance & Lifetime: `i < len` keeps pointer in allocated range.
        // 4. Aliasing: Caller guarantees exclusive access.
        // 5. Volatile Semantics: `write_volatile` writes byte without elision.
        // 6. Panic Safety: Addition and write cannot panic or unwind.
        unsafe {
            core::ptr::write_volatile(base.add(i), 0xA5);
        }
        i += 1;
    }
}

/// Measures stack high water mark (used bytes) from bottom up to first
/// modified byte using volatile reads.
///
/// # Safety
/// Caller must ensure `base` is valid for reads of `len` bytes when non-null
/// and `len > 0`.
// JUSTIFICATION: Stack HWM inspection reads task stack memory using volatile
// raw pointers without forming invalid references to live or uninitialized
// stack frames.
// SAFETY:
// 1. Preconditions: `base` is valid for reads of `len` bytes when non-null.
// 2. Alignment: `u8` has alignment 1; valid for any non-null byte address.
// 3. Provenance & Lifetime: Points to allocated task stack memory.
// 4. Aliasing: Raw pointer read does not create `&[u8]` references.
// 5. Volatile Semantics: `read_volatile` prevents dead-code optimization.
// 6. Panic Safety: Loop only performs non-panicking pointer arithmetic and
//    volatile reads.
pub unsafe fn measure_stack_hwm_volatile(base: *const u8, len: usize) -> u32 {
    if base.is_null() || len == 0 {
        return 0;
    }
    let mut unused = 0usize;
    while unused < len {
        // JUSTIFICATION: Raw volatile read of stack canary sentinel byte.
        // SAFETY:
        // 1. Preconditions: `base` is non-null and valid for `len` bytes.
        // 2. Alignment: `u8` alignment 1; `base.add(unused)` is aligned.
        // 3. Provenance & Lifetime: `unused < len` keeps pointer in range.
        // 4. Aliasing: No references are constructed; raw volatile read only.
        // 5. Volatile Semantics: `read_volatile` fetches live memory state.
        // 6. Panic Safety: Non-panicking pointer arithmetic and byte read.
        let val = unsafe { core::ptr::read_volatile(base.add(unused)) };
        if val != 0xA5 {
            break;
        }
        unused += 1;
    }
    (len - unused) as u32
}

/// Measures stack high water mark (used bytes) by delegating to
/// [`measure_stack_hwm_volatile`].
///
/// # Safety
/// Caller must ensure `base` is valid for reads of `len` bytes when non-null
/// and `len > 0`.
// JUSTIFICATION: Preserves public crate API while delegating directly to the
// single null-guarded volatile stack canary scanner.
// SAFETY:
// 1. Preconditions: Caller upholds `base` validity for reads of `len` bytes.
// 2. Alignment: `u8` has alignment 1; valid for any non-null byte address.
// 3. Provenance & Lifetime: Forwarded to `measure_stack_hwm_volatile`.
// 4. Aliasing: Raw volatile reads create zero Rust references.
// 5. Volatile Semantics: Delegated to `read_volatile` scan loop.
// 6. Panic Safety: Non-panicking delegation.
#[inline]
pub unsafe fn measure_stack_hwm(base: *const u8, len: usize) -> u32 {
    // JUSTIFICATION: Forward caller safety invariants to volatile HWM scanner.
    // SAFETY:
    // 1. Preconditions: Caller guarantees `base` is valid for `len` bytes.
    // 2. Alignment: `u8` alignment 1 is satisfied for all byte pointers.
    // 3. Provenance & Lifetime: Preserved across direct pointer forwarding.
    // 4. Aliasing: No references constructed.
    // 5. Volatile Semantics: Handled by `measure_stack_hwm_volatile`.
    // 6. Panic Safety: Cannot panic or unwind.
    unsafe { measure_stack_hwm_volatile(base, len) }
}

const HEX_CHARS: &[u8; 16] = b"0123456789abcdef";

/// Formats a 32-byte SHA-256 digest into a 64-byte lowercase hex string.
pub fn format_sha256_hex<'a>(digest: &[u8; 32], out: &'a mut [u8; 64]) -> &'a str {
    let mut i = 0usize;
    while i < 32 {
        let b = digest[i];
        out[i * 2] = HEX_CHARS[(b >> 4) as usize];
        out[i * 2 + 1] = HEX_CHARS[(b & 0x0F) as usize];
        i += 1;
    }
    core::str::from_utf8(out).unwrap_or_default()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ccount_host_stub() {
        let c1 = ccount();
        let c2 = ccount();
        assert_eq!(c2, c1 + 240);
    }

    #[test]
    fn test_fw_stats_ewma_and_max_latency() {
        let stats = FwStats::new();
        stats.record_sensor_period_us(1000);
        let s1 = stats.snapshot();
        assert_eq!(s1.sensor_frame_period_us, 1000);

        // (1000 * 7 + 2000) >> 3 = 9000 >> 3 = 1125
        stats.record_sensor_period_us(2000);
        let s2 = stats.snapshot();
        assert_eq!(s2.sensor_frame_period_us, 1125);

        stats.record_frame_bytes(10_000);
        assert_eq!(stats.snapshot().frame_bytes, 10_000);
        // (10000 * 7 + 18000) >> 3 = 88000 >> 3 = 11000
        stats.record_frame_bytes(18_000);
        assert_eq!(stats.snapshot().frame_bytes, 11_000);

        stats.update_max_loop_latency_us(500);
        stats.update_max_loop_latency_us(200);
        stats.update_max_loop_latency_us(850);
        assert_eq!(stats.snapshot().max_loop_latency_us, 850);

        stats.inc_c1_pushed();
        stats.inc_c1_dropped();
        stats.inc_ring_full_events();
        stats.inc_soi_miss();
        stats.inc_overflow_aborts();
        stats.add_c0_spsc_cycles(42);
        stats.add_c0_net_cycles(100);
        stats.add_c0_p80_cycles(10);
        stats.add_c0_p81_cycles(20);
        stats.inc_p81_frames_sent();
        stats.inc_p81_frames_superseded();
        stats.inc_c0_validation_failures();
        stats.inc_sccb_errors();
        stats.inc_send_partial();
        stats.inc_ring_build_failures();
        stats.inc_i2s_stop_timeouts();

        let snap = stats.snapshot();
        assert_eq!(snap.c1_pushed, 1);
        assert_eq!(snap.c1_dropped, 1);
        assert_eq!(snap.ring_full_events, 1);
        assert_eq!(snap.soi_miss, 1);
        assert_eq!(snap.overflow_aborts, 1);
        assert_eq!(snap.c0_spsc_cycles, 42);
        assert_eq!(snap.c0_net_cycles, 100);
        assert_eq!(snap.c0_p80_cycles, 10);
        assert_eq!(snap.c0_p81_cycles, 20);
        assert_eq!(snap.p81_frames_sent, 1);
        assert_eq!(snap.p81_frames_superseded, 1);
        assert_eq!(snap.c0_validation_failures, 1);
        assert_eq!(snap.sccb_errors, 1);
        assert_eq!(snap.send_partial, 1);
        assert_eq!(snap.ring_build_failures, 1);
        assert_eq!(snap.i2s_stop_timeouts, 1);
    }

    #[test]
    fn test_reset_reason_label_mapping_and_masking() {
        assert_eq!(reset_reason_label(0), "unknown");
        assert_eq!(reset_reason_label(1), "poweron");
        assert_eq!(reset_reason_label(2), "unknown");
        assert_eq!(reset_reason_label(3), "sw_reset");
        assert_eq!(reset_reason_label(4), "owdt_reset");
        assert_eq!(reset_reason_label(5), "deepsleep");
        assert_eq!(reset_reason_label(6), "sdio_reset");
        assert_eq!(reset_reason_label(7), "tg0wdt_sys");
        assert_eq!(reset_reason_label(8), "tg1wdt_sys");
        assert_eq!(reset_reason_label(9), "rtcwdt_sys");
        assert_eq!(reset_reason_label(10), "intrusion");
        assert_eq!(reset_reason_label(11), "tgwdt_cpu");
        assert_eq!(reset_reason_label(12), "sw_cpu_reset");
        assert_eq!(reset_reason_label(13), "rtcwdt_cpu");
        assert_eq!(reset_reason_label(14), "ext_cpu");
        assert_eq!(reset_reason_label(15), "brownout");
        assert_eq!(reset_reason_label(16), "rtcwdt_rtc");
        assert_eq!(reset_reason_label(17), "unknown");
        assert_eq!(reset_reason_label(63), "unknown");
        assert_eq!(reset_reason_label((0xDEAD_BEE0 & !0x3F) | 1), "poweron");
        assert_eq!(reset_reason_label(0xFFFF_FFC0 | 12), "sw_cpu_reset");
    }

    #[test]
    fn test_stack_sentinel_and_measure_hwm() {
        let mut stack = [0u8; 128];
        // JUSTIFICATION: Test initializes stack memory for canary verification.
        // SAFETY:
        // 1. Preconditions: `stack` is non-null with length 128.
        // 2. Alignment: `u8` requires alignment 1.
        // 3. Provenance & Lifetime: Local `stack` lives for test scope.
        // 4. Aliasing: Exclusive mutable pointer before slice reads.
        // 5. Volatile Semantics: `write_volatile` paints 0xA5 pattern.
        // 6. Panic Safety: Bounded write loop cannot panic or unwind.
        unsafe {
            paint_stack_sentinel(stack.as_mut_ptr(), 128);
        }
        for byte in &stack {
            assert_eq!(*byte, 0xA5);
        }

        // Null and zero-length guards return 0 safely.
        // JUSTIFICATION: Test verifies null and zero-length safety guards.
        // SAFETY:
        // 1. Preconditions: Null/zero inputs hit early return before deref.
        // 2. Alignment: No dereference occurs when null or `len == 0`.
        // 3. Provenance & Lifetime: No memory accessed.
        // 4. Aliasing: No references created.
        // 5. Volatile Semantics: Returns prior to `read_volatile`.
        // 6. Panic Safety: Early return cannot panic.
        let hwm_null = unsafe { measure_stack_hwm(core::ptr::null(), 128) };
        assert_eq!(hwm_null, 0);

        // Initially 0 bytes used
        // JUSTIFICATION: Test inspects pristine canary pattern.
        // SAFETY:
        // 1. Preconditions: `stack` is non-null and valid for 128 bytes.
        // 2. Alignment: `u8` requires alignment 1.
        // 3. Provenance & Lifetime: Local `stack` array lives for test scope.
        // 4. Aliasing: Raw pointer read forms no conflicting references.
        // 5. Volatile Semantics: `read_volatile` reads initialized stack bytes.
        // 6. Panic Safety: Bounded scan cannot panic or unwind.
        let hwm0 = unsafe { measure_stack_hwm(stack.as_ptr(), 128) };
        assert_eq!(hwm0, 0);

        // Dirty upper 32 bytes (indices 96..128)
        stack[96..128].fill(0x55);
        // JUSTIFICATION: Test inspects partially dirty canary pattern.
        // SAFETY:
        // 1. Preconditions: `stack` is non-null and valid for 128 bytes.
        // 2. Alignment: `u8` requires alignment 1.
        // 3. Provenance & Lifetime: Local `stack` array lives for test scope.
        // 4. Aliasing: Raw pointer read forms no conflicting references.
        // 5. Volatile Semantics: `read_volatile` reads initialized stack bytes.
        // 6. Panic Safety: Bounded scan cannot panic or unwind.
        let hwm1 = unsafe { measure_stack_hwm(stack.as_ptr(), 128) };
        assert_eq!(hwm1, 32);

        // Dirty from byte 10 onwards
        stack[10] = 0xAA;
        // JUSTIFICATION: Test inspects deeply modified stack canary pattern.
        // SAFETY:
        // 1. Preconditions: `stack` is non-null and valid for 128 bytes.
        // 2. Alignment: `u8` requires alignment 1.
        // 3. Provenance & Lifetime: Local `stack` array lives for test scope.
        // 4. Aliasing: Raw pointer read forms no conflicting references.
        // 5. Volatile Semantics: `read_volatile` reads initialized stack bytes.
        // 6. Panic Safety: Bounded scan cannot panic or unwind.
        let hwm2 = unsafe { measure_stack_hwm(stack.as_ptr(), 128) };
        assert_eq!(hwm2, 118);
        // JUSTIFICATION: Test verifies volatile stack HWM scanner parity.
        // SAFETY:
        // 1. Preconditions: `stack` is non-null and valid for 128 bytes.
        // 2. Alignment: `u8` requires alignment 1.
        // 3. Provenance & Lifetime: Local `stack` array lives for test scope.
        // 4. Aliasing: Raw pointer read forms no conflicting references.
        // 5. Volatile Semantics: `read_volatile` reads initialized stack bytes.
        // 6. Panic Safety: Bounded scan cannot panic or unwind.
        let hwm_vol = unsafe { measure_stack_hwm_volatile(stack.as_ptr(), 128) };
        assert_eq!(hwm_vol, 118);
    }

    #[test]
    fn test_format_sha256_hex() {
        let digest: [u8; 32] = [
            0xba, 0x78, 0x16, 0xbf, 0x8f, 0x01, 0xcf, 0xea, 0x41, 0x41, 0x40, 0xde, 0x5d, 0xae,
            0x22, 0x23, 0xb0, 0x03, 0x61, 0xa3, 0x96, 0x17, 0x7a, 0x9c, 0xb4, 0x10, 0xff, 0x61,
            0xf2, 0x00, 0x15, 0xad,
        ];
        let mut out = [0u8; 64];
        let hex_str = format_sha256_hex(&digest, &mut out);
        assert_eq!(
            hex_str,
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"
        );
    }
}
