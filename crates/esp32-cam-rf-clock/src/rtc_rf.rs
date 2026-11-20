//! Pure-Rust compatibility layer for `librtc.a:bt_bb.o` (`coex_bt_high_prio`)
//! and `librtc.a:rtc_analog.o` (`temprature_sens_read`).
//!
//! Hardware register reads/writes and microsecond delays are abstracted behind
//! [`RegOps`] so host unit tests run with zero C libraries while Xtensa builds
//! preserve the exact `memw` + volatile MMIO and ROM/DPORT FFI call sequences.

use core::hint;
#[cfg(target_arch = "xtensa")]
use core::ptr;
#[cfg(any(target_arch = "xtensa", test))]
use core::sync::atomic::{AtomicBool, Ordering};

/// Temperature sensor output register (`sens_reg.h:344`).
///
/// Holds `SENS_TSENS_OUT` in bits `[29:22]` (mask `0x3FC0_0000`, shift `22`).
pub const SENS_SAR_SLAVE_ADDR3_REG: u32 = 0x3FF4_8844;

/// Temperature sensor control register (`sens_reg.h:396`).
///
/// Controls `SENS_TSENS_CLK_DIV` (`[23:16]`), `SENS_TSENS_POWER_UP` (bit `24`),
/// `SENS_TSENS_POWER_UP_FORCE` (bit `25`), and `SENS_TSENS_DUMP_OUT` (`26`).
pub const SENS_SAR_TSENS_CTRL_REG: u32 = 0x3FF4_884C;

/// Proprietary SLC coexistence control register (`librtc.a:bt_bb.o`).
#[allow(non_upper_case_globals)]
pub const COEX_SLC_0x3FF5C080_REG: u32 = 0x3FF5_C080;

/// Proprietary MAC coexistence control register 0 (`librtc.a:bt_bb.o`).
#[allow(non_upper_case_globals)]
pub const COEX_MAC_0x600310D0_REG: u32 = 0x6003_10D0;

/// Proprietary MAC coexistence control register 1 (`librtc.a:bt_bb.o`).
#[allow(non_upper_case_globals)]
pub const COEX_MAC_0x60031300_REG: u32 = 0x6003_1300;

/// Proprietary BT coexistence priority register 0 (`librtc.a:bt_bb.o`).
#[allow(non_upper_case_globals)]
pub const COEX_BT_PRIO0_0x60033D30_REG: u32 = 0x6003_3D30;

/// Proprietary BT coexistence priority register 1 (`librtc.a:bt_bb.o`).
#[allow(non_upper_case_globals)]
pub const COEX_BT_PRIO1_0x60033D38_REG: u32 = 0x6003_3D38;

/// Proprietary BT coexistence priority register 2 (`librtc.a:bt_bb.o`).
#[allow(non_upper_case_globals)]
pub const COEX_BT_PRIO2_0x60033D40_REG: u32 = 0x6003_3D40;

/// Proprietary RF front-end coexistence register (`librtc.a:bt_bb.o`).
#[allow(non_upper_case_globals)]
pub const COEX_FE_0x600041C4_REG: u32 = 0x6000_41C4;

/// Baseband coexistence control register (`DR_REG_BB_BASE + 0x0040`).
#[allow(non_upper_case_globals)]
pub const BB_COEX_0x3FF5D040_REG: u32 = 0x3FF5_D040;

const SENS_TSENS_OUT_MASK: u32 = 0x3FC0_0000;
const SENS_TSENS_OUT_SHIFT: u32 = 22;

/// Abstraction over 32-bit MMIO register accesses and microsecond delays.
pub trait RegOps {
    /// Reads a 32-bit register at MMIO address `addr`.
    fn read_u32(&self, addr: u32) -> u32;

    /// Writes `val` to the 32-bit register at MMIO address `addr`.
    fn write_u32(&mut self, addr: u32, val: u32);

    /// Busy-waits for `us` microseconds.
    fn delay_us(&mut self, us: u32);
}

#[derive(Clone, Copy)]
struct RmwStep {
    addr: u32,
    and_mask: u32,
    or_mask: u32,
}

const COEX_BT_HIGH_PRIO_STEPS: [RmwStep; 13] = [
    RmwStep {
        addr: COEX_SLC_0x3FF5C080_REG,
        and_mask: 0xFFFF_FF3F,
        or_mask: 0,
    },
    RmwStep {
        addr: COEX_MAC_0x600310D0_REG,
        and_mask: 0xFFFF_FF0F,
        or_mask: 0,
    },
    RmwStep {
        addr: COEX_MAC_0x60031300_REG,
        and_mask: 0xFFFF_FF0F,
        or_mask: 0,
    },
    RmwStep {
        addr: COEX_BT_PRIO0_0x60033D30_REG,
        and_mask: u32::MAX,
        or_mask: 0x0000_0001,
    },
    RmwStep {
        addr: COEX_BT_PRIO0_0x60033D30_REG,
        and_mask: 0xF0FF_FFFF,
        or_mask: 0x0100_0000,
    },
    RmwStep {
        addr: COEX_BT_PRIO0_0x60033D30_REG,
        and_mask: 0xFF0F_FFFF,
        or_mask: 0x0040_0000,
    },
    RmwStep {
        addr: COEX_BT_PRIO1_0x60033D38_REG,
        and_mask: 0x0000_FFFF,
        or_mask: 0x0C80_0000,
    },
    RmwStep {
        addr: COEX_BT_PRIO1_0x60033D38_REG,
        and_mask: 0xFFFF_0000,
        or_mask: 0x0000_000A,
    },
    RmwStep {
        addr: COEX_BT_PRIO2_0x60033D40_REG,
        and_mask: 0xFFFF_FFC7,
        or_mask: 0x0000_0010,
    },
    RmwStep {
        addr: COEX_FE_0x600041C4_REG,
        and_mask: u32::MAX,
        or_mask: 0x0000_0040,
    },
    RmwStep {
        addr: COEX_MAC_0x600310D0_REG,
        and_mask: u32::MAX,
        or_mask: 0x0000_0001,
    },
    RmwStep {
        addr: COEX_MAC_0x60031300_REG,
        and_mask: u32::MAX,
        or_mask: 0x0000_0001,
    },
    RmwStep {
        addr: BB_COEX_0x3FF5D040_REG,
        and_mask: 0x7FFF_FFFF,
        or_mask: 0,
    },
];

#[inline]
fn rmw_u32<R: RegOps>(regs: &mut R, addr: u32, and_mask: u32, or_mask: u32) {
    let next = (regs.read_u32(addr) & and_mask) | or_mask;
    regs.write_u32(addr, next);
}

/// Executes the 13 ordered coexistence high-priority RMW steps via `regs`.
pub fn coex_bt_high_prio_ops<R: RegOps>(regs: &mut R) {
    for step in COEX_BT_HIGH_PRIO_STEPS {
        rmw_u32(regs, step.addr, step.and_mask, step.or_mask);
    }
}

/// Reads the ESP32 internal temperature sensor via `regs`.
///
/// Mirrors `librtc.a:rtc_analog.o:temprature_sens_read` with the exact 11
/// ordered register accesses and microsecond delays (`100` us, `2` us),
/// returning `SENS_TSENS_OUT` (`bits [29:22]`).
#[must_use]
pub fn temprature_sens_read_ops<R: RegOps>(regs: &mut R) -> u8 {
    rmw_u32(regs, SENS_SAR_TSENS_CTRL_REG, 0xFF00_FFFF, 0x0002_0000);
    rmw_u32(regs, SENS_SAR_TSENS_CTRL_REG, u32::MAX, 0x0200_0000);
    rmw_u32(regs, SENS_SAR_TSENS_CTRL_REG, 0xFEFF_FFFF, 0);
    rmw_u32(regs, SENS_SAR_TSENS_CTRL_REG, u32::MAX, 0x0100_0000);
    regs.delay_us(100);
    rmw_u32(regs, SENS_SAR_TSENS_CTRL_REG, u32::MAX, 0x0400_0000);
    regs.delay_us(2);
    let raw = regs.read_u32(SENS_SAR_SLAVE_ADDR3_REG);
    rmw_u32(regs, SENS_SAR_TSENS_CTRL_REG, 0xFBFF_FFFF, 0);
    rmw_u32(regs, SENS_SAR_TSENS_CTRL_REG, 0xFEFF_FFFF, 0);
    rmw_u32(regs, SENS_SAR_TSENS_CTRL_REG, 0xFDFF_FFFF, 0);
    ((raw & SENS_TSENS_OUT_MASK) >> SENS_TSENS_OUT_SHIFT) as u8
}

/// Returns `true` iff `addr` is 4-byte aligned and belongs to the verified
/// ESP32 COEX/TSENS MMIO register allowlist for reads.
#[must_use]
#[allow(non_upper_case_globals)]
pub const fn is_valid_mmio_read_addr(addr: u32) -> bool {
    if (addr & 3) != 0 {
        return false;
    }
    matches!(
        addr,
        SENS_SAR_SLAVE_ADDR3_REG
            | SENS_SAR_TSENS_CTRL_REG
            | COEX_SLC_0x3FF5C080_REG
            | COEX_MAC_0x600310D0_REG
            | COEX_MAC_0x60031300_REG
            | COEX_BT_PRIO0_0x60033D30_REG
            | COEX_BT_PRIO1_0x60033D38_REG
            | COEX_BT_PRIO2_0x60033D40_REG
            | COEX_FE_0x600041C4_REG
            | BB_COEX_0x3FF5D040_REG
    )
}

/// Returns `true` iff `addr` is 4-byte aligned and belongs to the verified
/// writable ESP32 COEX/TSENS MMIO register allowlist.
#[must_use]
#[allow(non_upper_case_globals)]
pub const fn is_valid_mmio_write_addr(addr: u32) -> bool {
    if (addr & 3) != 0 {
        return false;
    }
    matches!(
        addr,
        SENS_SAR_TSENS_CTRL_REG
            | COEX_SLC_0x3FF5C080_REG
            | COEX_MAC_0x600310D0_REG
            | COEX_MAC_0x60031300_REG
            | COEX_BT_PRIO0_0x60033D30_REG
            | COEX_BT_PRIO1_0x60033D38_REG
            | COEX_BT_PRIO2_0x60033D40_REG
            | COEX_FE_0x600041C4_REG
            | BB_COEX_0x3FF5D040_REG
    )
}

/// Lightweight `#![no_std]` atomic spinlock serializing dual-core and same-core
/// ISR access to COEX MMIO RMW sequences and the TSENS FSM.
///
/// On Xtensa, acquiring the lock masks local interrupts (`rsil 15`) before
/// spinning so a same-core ISR or higher-priority task cannot preempt the lock
/// holder and deadlock on re-entry; dropping [`SpinLockGuard`] releases the
/// lock and restores the saved `PS` interrupt level (`wsr.ps` + `rsync`).
#[cfg(any(target_arch = "xtensa", test))]
struct AtomicSpinLock {
    locked: AtomicBool,
}

#[cfg(any(target_arch = "xtensa", test))]
struct SpinLockGuard<'a> {
    lock: &'a AtomicSpinLock,
    #[cfg(target_arch = "xtensa")]
    saved_ps: u32,
}

#[cfg(any(target_arch = "xtensa", test))]
impl AtomicSpinLock {
    const fn new() -> Self {
        Self {
            locked: AtomicBool::new(false),
        }
    }

    fn lock(&self) -> SpinLockGuard<'_> {
        #[cfg(target_arch = "xtensa")]
        let saved_ps: u32 = {
            let ps: u32;
            // JUSTIFICATION: Masks local Xtensa interrupts to level 15 before
            // acquiring the spinlock to prevent single-core ISR/preemption
            // deadlock on `COEX_LOCK` or `TSENS_LOCK` (`M-103`).
            // SAFETY:
            // 1. Alignment & Non-null: Register-only `rsil` instruction.
            // 2. Memory layout: Architectural Xtensa LX6 status register op.
            // 3. Provenance: Valid privileged instruction on bare-metal ESP32.
            // 4. Aliasing: Touches no Rust memory.
            // 5. Volatile semantics: `asm!` without `nomem` prevents reordering
            //    critical-section accesses before interrupt masking.
            // 6. Panic safety: Cannot unwind; `SpinLockGuard::drop` restores PS.
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
            while self.locked.load(Ordering::Relaxed) {
                hint::spin_loop();
            }
        }
        SpinLockGuard {
            lock: self,
            #[cfg(target_arch = "xtensa")]
            saved_ps,
        }
    }

    #[cfg(all(test, not(target_arch = "xtensa")))]
    fn try_lock(&self) -> Option<SpinLockGuard<'_>> {
        self.locked
            .compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed)
            .ok()
            .map(|_| SpinLockGuard { lock: self })
    }
}

#[cfg(any(target_arch = "xtensa", test))]
impl Drop for SpinLockGuard<'_> {
    fn drop(&mut self) {
        self.lock.locked.store(false, Ordering::Release);
        #[cfg(target_arch = "xtensa")]
        {
            let ps = self.saved_ps;
            // JUSTIFICATION: Restores the saved Xtensa `PS` interrupt level
            // after leaving the spinlock critical section (`M-103`).
            // SAFETY:
            // 1. Alignment & Non-null: Register-only instruction; no memory
            //    operand.
            // 2. Memory layout: Architectural Xtensa `wsr.ps` + `rsync`.
            // 3. Provenance: `ps` was captured by `rsil 15` in `lock()`.
            // 4. Aliasing: Touches no Rust memory; restores local core `PS`.
            // 5. Volatile semantics: `asm!` without `nomem` orders the
            //    `Release` store before re-enabling interrupts.
            // 6. Panic safety: Cannot unwind.
            unsafe {
                core::arch::asm!(
                    "wsr.ps {0}",
                    "rsync",
                    in(reg) ps,
                    options(nostack),
                );
            }
        }
    }
}

#[cfg(target_arch = "xtensa")]
static COEX_LOCK: AtomicSpinLock = AtomicSpinLock::new();

#[cfg(target_arch = "xtensa")]
static TSENS_LOCK: AtomicSpinLock = AtomicSpinLock::new();

// JUSTIFICATION: Declares ESP32 ROM `ets_delay_us` and ESP-IDF DPORT register
// read helper linked on Xtensa targets for TSENS timing and safe DPORT reads.
// SAFETY: Both symbols use the standard C ABI, take plain `u32` value
// arguments, and never unwind across FFI.
#[cfg(target_arch = "xtensa")]
extern "C" {
    fn ets_delay_us(us: u32);
    fn esp_dport_access_reg_read(reg: u32) -> u32;
}

#[cfg(target_arch = "xtensa")]
struct HwRegOps;

#[cfg(target_arch = "xtensa")]
impl RegOps for HwRegOps {
    #[inline]
    fn read_u32(&self, addr: u32) -> u32 {
        if !is_valid_mmio_read_addr(addr) {
            return 0;
        }
        // JUSTIFICATION: `esp_dport_access_reg_read` is the ESP-IDF DPORT/RTC
        // FFI read primitive required on ESP32 Xtensa silicon.
        // SAFETY:
        // 1. Address validity: `is_valid_mmio_read_addr` verified 4-byte
        //    alignment and membership in the COEX/TSENS MMIO allowlist.
        // 2. ABI & unwind: `extern "C"` helper is leaf-like and never unwinds.
        unsafe { esp_dport_access_reg_read(addr) }
    }

    #[inline]
    fn write_u32(&mut self, addr: u32, val: u32) {
        if !is_valid_mmio_write_addr(addr) {
            return;
        }
        // JUSTIFICATION: Direct MMIO write bracketed by Xtensa `memw` barriers
        // preserves pre- and post-write ordering against subsequent MMIO reads
        // and `ets_delay_us` waits.
        // SAFETY:
        // 1. Alignment & address: `is_valid_mmio_write_addr` verified 4-byte
        //    alignment and membership in the writable COEX/TSENS allowlist.
        // 2. Volatile & barriers: leading and trailing `memw` + `write_volatile`
        //    prevent compiler elision and posted-write skew.
        // 3. Exclusive access: `COEX_LOCK`/`TSENS_LOCK` serialize RMW callers.
        unsafe {
            core::arch::asm!("memw", options(nostack, preserves_flags));
            ptr::write_volatile(addr as *mut u32, val);
            core::arch::asm!("memw", options(nostack, preserves_flags));
        }
    }

    #[inline]
    fn delay_us(&mut self, us: u32) {
        // JUSTIFICATION: Flushes posted RTC/APB register writes via `memw` and
        // a DPORT read-back barrier before ROM `ets_delay_us` busy-waits for
        // the temperature sensor FSM settling interval.
        // SAFETY:
        // 1. Read-back target: `SENS_SAR_TSENS_CTRL_REG` is a verified 4-byte
        //    aligned side-effect-free control register.
        // 2. Value argument: `us` is a plain `u32` with no pointer aliasing.
        // 3. ABI & unwind: ROM busy-wait helper never panics or unwinds.
        unsafe {
            core::arch::asm!("memw", options(nostack, preserves_flags));
            let _ = esp_dport_access_reg_read(SENS_SAR_TSENS_CTRL_REG);
            ets_delay_us(us);
        }
    }
}

/// C ABI compatibility shim for `librtc.a:bt_bb.o:coex_bt_high_prio`.
///
/// # Safety
/// Caller must ensure ESP32 DPORT/MAC/BB clocks are enabled. Dual-core RMW
/// serialization is enforced internally by `COEX_LOCK`.
// JUSTIFICATION: C radio blobs (`libnet80211.a:ieee80211_ioctl.o`) resolve
// the unmangled `coex_bt_high_prio` symbol across the C ABI boundary.
// SAFETY: Acquires `COEX_LOCK` and delegates to `coex_bt_high_prio_ops` using
// `HwRegOps` under the caller's `# Safety` hardware-clock precondition.
#[cfg(target_arch = "xtensa")]
#[no_mangle]
#[link_section = ".text.coex_bt_high_prio"]
pub unsafe extern "C" fn coex_bt_high_prio() {
    let _guard = COEX_LOCK.lock();
    let mut hw = HwRegOps;
    coex_bt_high_prio_ops(&mut hw);
}

/// C ABI compatibility shim for `librtc.a:rtc_analog.o:temprature_sens_read`.
///
/// # Safety
/// Caller must ensure ESP32 RTC/SAR/DPORT clocks are active. Dual-core and
/// same-core ISR TSENS FSM serialization is enforced internally by `TSENS_LOCK`
/// (masking local interrupts via `rsil 15` and restoring `PS` on drop).
// JUSTIFICATION: C PHY calibration blobs (`libphy.a:phy_chip_v7_cal.o`)
// resolve the unmangled `temprature_sens_read` symbol across the C ABI.
// SAFETY: Acquires `TSENS_LOCK` and delegates to `temprature_sens_read_ops`
// using `HwRegOps` under the caller's `# Safety` hardware-clock precondition.
#[cfg(target_arch = "xtensa")]
#[no_mangle]
#[link_section = ".text.temprature_sens_read"]
pub unsafe extern "C" fn temprature_sens_read() -> u8 {
    let _guard = TSENS_LOCK.lock();
    let mut hw = HwRegOps;
    temprature_sens_read_ops(&mut hw)
}

/// References `coex_bt_high_prio` and `temprature_sens_read` on Xtensa so the
/// linker preserves both compatibility symbols before `--gc-sections`.
#[must_use]
#[inline(never)]
pub fn link_touch() -> usize {
    #[cfg(target_arch = "xtensa")]
    {
        // JUSTIFICATION: Takes the address of `coex_bt_high_prio` to retain
        // its symbol across `--gc-sections` without invoking it.
        // SAFETY: Function pointer cast to `*const ()` and `usize` performs no
        // call or memory access.
        let coex_fn: unsafe extern "C" fn() = coex_bt_high_prio;
        let coex = hint::black_box(coex_fn as *const () as usize);

        // JUSTIFICATION: Takes the address of `temprature_sens_read` to retain
        // its symbol across `--gc-sections` without invoking it.
        // SAFETY: Function pointer cast to `*const ()` and `usize` performs no
        // call or memory access.
        let tsens_fn: unsafe extern "C" fn() -> u8 = temprature_sens_read;
        let tsens = hint::black_box(tsens_fn as *const () as usize);
        coex ^ tsens
    }
    #[cfg(not(target_arch = "xtensa"))]
    {
        hint::black_box(0)
    }
}

#[cfg(test)]
mod tests {
    use core::cell::Cell;

    use super::*;

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum TraceEvent {
        Read(u32),
        Write(u32, u32),
        DelayUs(u32),
    }

    struct MockRegOps {
        regs: [(u32, u32); 16],
        reg_len: usize,
        writes: [(u32, u32); 32],
        write_len: usize,
        delays: [u32; 8],
        delay_len: usize,
        trace: [Cell<TraceEvent>; 48],
        trace_len: Cell<usize>,
    }

    impl MockRegOps {
        fn new(initial: &[(u32, u32)]) -> Self {
            assert!(
                initial.len() <= 16,
                "initial register count {} exceeds capacity 16",
                initial.len()
            );
            let mut regs = [(0_u32, 0_u32); 16];
            for (dst, &src) in regs.iter_mut().zip(initial.iter()) {
                *dst = src;
            }
            Self {
                regs,
                reg_len: initial.len(),
                writes: [(0_u32, 0_u32); 32],
                write_len: 0,
                delays: [0_u32; 8],
                delay_len: 0,
                trace: core::array::from_fn(|_| Cell::new(TraceEvent::DelayUs(0))),
                trace_len: Cell::new(0),
            }
        }

        fn push_trace(&self, event: TraceEvent) {
            let idx = self.trace_len.get();
            assert!(idx < self.trace.len(), "trace buffer overflow at {idx}");
            self.trace[idx].set(event);
            self.trace_len.set(idx + 1);
        }

        fn writes(&self) -> &[(u32, u32)] {
            &self.writes[..self.write_len]
        }

        fn delays(&self) -> &[u32] {
            &self.delays[..self.delay_len]
        }

        fn reg(&self, addr: u32) -> u32 {
            for &(k, v) in &self.regs[..self.reg_len] {
                if k == addr {
                    return v;
                }
            }
            panic!("unseeded register 0x{addr:08X}");
        }

        fn trace_eq(&self, expected: &[TraceEvent]) {
            let len = self.trace_len.get();
            assert_eq!(
                len,
                expected.len(),
                "trace length {len} != expected {}",
                expected.len()
            );
            for (idx, &exp) in expected.iter().enumerate() {
                assert_eq!(self.trace[idx].get(), exp, "trace mismatch at {idx}");
            }
        }
    }

    impl RegOps for MockRegOps {
        fn read_u32(&self, addr: u32) -> u32 {
            self.push_trace(TraceEvent::Read(addr));
            self.reg(addr)
        }

        fn write_u32(&mut self, addr: u32, val: u32) {
            assert!(
                self.write_len < self.writes.len(),
                "writes buffer overflow at {}",
                self.write_len
            );
            self.writes[self.write_len] = (addr, val);
            self.write_len += 1;
            self.push_trace(TraceEvent::Write(addr, val));

            for entry in &mut self.regs[..self.reg_len] {
                if entry.0 == addr {
                    entry.1 = val;
                    return;
                }
            }
            panic!("write to unseeded register 0x{addr:08X}");
        }

        fn delay_us(&mut self, us: u32) {
            assert!(
                self.delay_len < self.delays.len(),
                "delays buffer overflow at {}",
                self.delay_len
            );
            self.delays[self.delay_len] = us;
            self.delay_len += 1;
            self.push_trace(TraceEvent::DelayUs(us));
        }
    }

    #[test]
    fn coex_bt_high_prio_executes_exact_13_rmw_steps() {
        let mut mock = MockRegOps::new(&[
            (COEX_SLC_0x3FF5C080_REG, 0xAAAA_AAFF),
            (COEX_MAC_0x600310D0_REG, 0x1234_56FE),
            (COEX_MAC_0x60031300_REG, 0x8765_43FE),
            (COEX_BT_PRIO0_0x60033D30_REG, 0xFFAA_55FE),
            (COEX_BT_PRIO1_0x60033D38_REG, 0x1234_5678),
            (COEX_BT_PRIO2_0x60033D40_REG, 0xDEAD_BEEF),
            (COEX_FE_0x600041C4_REG, 0x5555_55BF),
            (BB_COEX_0x3FF5D040_REG, 0xCAFE_BABE),
        ]);

        coex_bt_high_prio_ops(&mut mock);

        let expected_writes = [
            (COEX_SLC_0x3FF5C080_REG, 0xAAAA_AA3F),
            (COEX_MAC_0x600310D0_REG, 0x1234_560E),
            (COEX_MAC_0x60031300_REG, 0x8765_430E),
            (COEX_BT_PRIO0_0x60033D30_REG, 0xFFAA_55FF),
            (COEX_BT_PRIO0_0x60033D30_REG, 0xF1AA_55FF),
            (COEX_BT_PRIO0_0x60033D30_REG, 0xF14A_55FF),
            (COEX_BT_PRIO1_0x60033D38_REG, 0x0C80_5678),
            (COEX_BT_PRIO1_0x60033D38_REG, 0x0C80_000A),
            (COEX_BT_PRIO2_0x60033D40_REG, 0xDEAD_BED7),
            (COEX_FE_0x600041C4_REG, 0x5555_55FF),
            (COEX_MAC_0x600310D0_REG, 0x1234_560F),
            (COEX_MAC_0x60031300_REG, 0x8765_430F),
            (BB_COEX_0x3FF5D040_REG, 0x4AFE_BABE),
        ];

        assert_eq!(mock.writes(), &expected_writes, "13 COEX writes mismatch");
        assert_eq!(mock.delays(), &[], "COEX must not invoke delay_us");
        assert_eq!(mock.reg(COEX_BT_PRIO0_0x60033D30_REG), 0xF14A_55FF);
        assert_eq!(mock.reg(COEX_BT_PRIO1_0x60033D38_REG), 0x0C80_000A);
        assert_eq!(mock.reg(BB_COEX_0x3FF5D040_REG), 0x4AFE_BABE);
    }

    #[test]
    fn temprature_sens_read_executes_exact_11_steps_and_delays() {
        let mut mock = MockRegOps::new(&[
            (SENS_SAR_TSENS_CTRL_REG, 0x81FF_1234),
            (SENS_SAR_SLAVE_ADDR3_REG, 0xE9FF_FFFF),
        ]);

        let raw_temp = temprature_sens_read_ops(&mut mock);
        assert_eq!(raw_temp, 0xA7, "unexpected TSENS byte 0x{raw_temp:02X}");

        let expected_writes = [
            (SENS_SAR_TSENS_CTRL_REG, 0x8102_1234),
            (SENS_SAR_TSENS_CTRL_REG, 0x8302_1234),
            (SENS_SAR_TSENS_CTRL_REG, 0x8202_1234),
            (SENS_SAR_TSENS_CTRL_REG, 0x8302_1234),
            (SENS_SAR_TSENS_CTRL_REG, 0x8702_1234),
            (SENS_SAR_TSENS_CTRL_REG, 0x8302_1234),
            (SENS_SAR_TSENS_CTRL_REG, 0x8202_1234),
            (SENS_SAR_TSENS_CTRL_REG, 0x8002_1234),
        ];
        assert_eq!(mock.writes(), &expected_writes, "TSENS writes mismatch");
        assert_eq!(mock.delays(), &[100, 2], "TSENS delays mismatch");
        assert_eq!(mock.reg(SENS_SAR_TSENS_CTRL_REG), 0x8002_1234);

        let expected_trace = [
            TraceEvent::Read(SENS_SAR_TSENS_CTRL_REG),
            TraceEvent::Write(SENS_SAR_TSENS_CTRL_REG, 0x8102_1234),
            TraceEvent::Read(SENS_SAR_TSENS_CTRL_REG),
            TraceEvent::Write(SENS_SAR_TSENS_CTRL_REG, 0x8302_1234),
            TraceEvent::Read(SENS_SAR_TSENS_CTRL_REG),
            TraceEvent::Write(SENS_SAR_TSENS_CTRL_REG, 0x8202_1234),
            TraceEvent::Read(SENS_SAR_TSENS_CTRL_REG),
            TraceEvent::Write(SENS_SAR_TSENS_CTRL_REG, 0x8302_1234),
            TraceEvent::DelayUs(100),
            TraceEvent::Read(SENS_SAR_TSENS_CTRL_REG),
            TraceEvent::Write(SENS_SAR_TSENS_CTRL_REG, 0x8702_1234),
            TraceEvent::DelayUs(2),
            TraceEvent::Read(SENS_SAR_SLAVE_ADDR3_REG),
            TraceEvent::Read(SENS_SAR_TSENS_CTRL_REG),
            TraceEvent::Write(SENS_SAR_TSENS_CTRL_REG, 0x8302_1234),
            TraceEvent::Read(SENS_SAR_TSENS_CTRL_REG),
            TraceEvent::Write(SENS_SAR_TSENS_CTRL_REG, 0x8202_1234),
            TraceEvent::Read(SENS_SAR_TSENS_CTRL_REG),
            TraceEvent::Write(SENS_SAR_TSENS_CTRL_REG, 0x8002_1234),
        ];
        mock.trace_eq(&expected_trace);
    }

    #[test]
    fn temprature_sens_read_extracts_bits_29_22_across_edge_patterns() {
        for expected in [0x00_u8, 0x01, 0x5A, 0x80, 0xFF] {
            let surrounding = 0xC03F_FFFF_u32;
            let reg_val = surrounding | (u32::from(expected) << 22);
            let mut mock = MockRegOps::new(&[
                (SENS_SAR_TSENS_CTRL_REG, 0),
                (SENS_SAR_SLAVE_ADDR3_REG, reg_val),
            ]);
            let got = temprature_sens_read_ops(&mut mock);
            assert_eq!(
                got, expected,
                "byte mismatch for reg_val 0x{reg_val:08X}: got 0x{got:02X}"
            );
        }
    }

    #[test]
    fn link_touch_returns_zero_on_host() {
        assert_eq!(link_touch(), 0, "host link_touch must return 0");
    }

    #[test]
    fn hw_reg_ops_address_validation_allows_only_aligned_coex_tsens_regs() {
        // Every RMW step in `COEX_BT_HIGH_PRIO_STEPS` must be read+write valid.
        for step in COEX_BT_HIGH_PRIO_STEPS {
            assert!(
                is_valid_mmio_read_addr(step.addr),
                "COEX step 0x{:08X} must be valid for read",
                step.addr
            );
            assert!(
                is_valid_mmio_write_addr(step.addr),
                "COEX step 0x{:08X} must be valid for write",
                step.addr
            );
        }

        // TSENS control register is read+write; output register is read-only.
        assert!(is_valid_mmio_read_addr(SENS_SAR_TSENS_CTRL_REG));
        assert!(is_valid_mmio_write_addr(SENS_SAR_TSENS_CTRL_REG));
        assert!(is_valid_mmio_read_addr(SENS_SAR_SLAVE_ADDR3_REG));
        assert!(!is_valid_mmio_write_addr(SENS_SAR_SLAVE_ADDR3_REG));

        // Unaligned offsets and non-allowlisted addresses must be rejected.
        let rejected = [
            0x0000_0000,
            SENS_SAR_TSENS_CTRL_REG + 1,
            SENS_SAR_TSENS_CTRL_REG + 2,
            SENS_SAR_TSENS_CTRL_REG + 3,
            COEX_SLC_0x3FF5C080_REG + 2,
            0x3FFB_0000, // DRAM
            0x4008_0000, // IRAM
            0x3FF4_8848, // adjacent unverified SENS register
            0xFFFF_FFFC,
            u32::MAX,
        ];
        for bad in rejected {
            assert!(
                !is_valid_mmio_read_addr(bad),
                "unexpected read allow for 0x{bad:08X}"
            );
            assert!(
                !is_valid_mmio_write_addr(bad),
                "unexpected write allow for 0x{bad:08X}"
            );
        }
    }

    #[test]
    fn atomic_spinlock_enforces_exclusivity_and_releases_on_drop() {
        let lock = AtomicSpinLock::new();
        {
            let _guard = lock.lock();
            assert!(
                lock.try_lock().is_none(),
                "second acquisition must fail while lock is held"
            );
        }
        assert!(
            lock.try_lock().is_some(),
            "lock must be re-acquirable after guard drops"
        );
    }
}
