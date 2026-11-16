//! Pure-Rust compatibility layer for `librtc.a:bt_bb.o` (`coex_bt_high_prio`)
//! and `librtc.a:rtc_analog.o` (`temprature_sens_read`).
//!
//! Hardware register reads/writes and microsecond delays are abstracted behind
//! [`RegOps`] so host unit tests run with zero C libraries while Xtensa builds
//! preserve the exact `memw` + volatile MMIO and ROM/DPORT FFI call sequences.

use core::hint;
#[cfg(target_arch = "xtensa")]
use core::ptr;

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
        // JUSTIFICATION: `esp_dport_access_reg_read` is the ESP-IDF DPORT/RTC
        // FFI read primitive required on ESP32 Xtensa silicon.
        // SAFETY:
        // 1. Address validity: `addr` is a verified 32-bit ESP32 MMIO address.
        // 2. ABI & unwind: `extern "C"` helper is leaf-like and never unwinds.
        unsafe { esp_dport_access_reg_read(addr) }
    }

    #[inline]
    fn write_u32(&mut self, addr: u32, val: u32) {
        // JUSTIFICATION: Direct MMIO write preceded by Xtensa `memw` matches
        // `librtc.a:bt_bb.o` and `rtc_analog.o` ordering guarantees.
        // SAFETY:
        // 1. Alignment & address: `addr` is a 4-byte aligned ESP32 MMIO word.
        // 2. Volatile & barrier: `memw` + `write_volatile` prevents elision.
        // 3. Exclusive access: Caller owns COEX/TSENS MMIO synchronization.
        unsafe {
            core::arch::asm!("memw", options(nostack, preserves_flags));
            ptr::write_volatile(addr as *mut u32, val);
        }
    }

    #[inline]
    fn delay_us(&mut self, us: u32) {
        // JUSTIFICATION: ROM `ets_delay_us` FFI provides the exact microsecond
        // busy-wait required by the temperature sensor FSM timing.
        // SAFETY:
        // 1. Value argument: `us` is a plain `u32` with no pointer aliasing.
        // 2. ABI & unwind: ROM busy-wait helper never panics or unwinds.
        unsafe {
            ets_delay_us(us);
        }
    }
}

/// C ABI compatibility shim for `librtc.a:bt_bb.o:coex_bt_high_prio`.
///
/// # Safety
/// Caller must ensure ESP32 DPORT/MAC/BB clocks are enabled and no concurrent
/// task mutates the coexistence priority registers during execution.
// JUSTIFICATION: C radio blobs (`libnet80211.a:ieee80211_ioctl.o`) resolve
// the unmangled `coex_bt_high_prio` symbol across the C ABI boundary.
// SAFETY: Delegates immediately to `coex_bt_high_prio_ops` using `HwRegOps`
// under the caller's `# Safety` hardware-clock and exclusivity preconditions.
#[cfg(target_arch = "xtensa")]
#[no_mangle]
#[link_section = ".text.coex_bt_high_prio"]
pub unsafe extern "C" fn coex_bt_high_prio() {
    let mut hw = HwRegOps;
    coex_bt_high_prio_ops(&mut hw);
}

/// C ABI compatibility shim for `librtc.a:rtc_analog.o:temprature_sens_read`.
///
/// # Safety
/// Caller must ensure ESP32 RTC/SAR/DPORT clocks are active and no concurrent
/// task mutates `SENS_SAR_TSENS_CTRL_REG` during sensor conversion.
// JUSTIFICATION: C PHY calibration blobs (`libphy.a:phy_chip_v7_cal.o`)
// resolve the unmangled `temprature_sens_read` symbol across the C ABI.
// SAFETY: Delegates immediately to `temprature_sens_read_ops` using `HwRegOps`
// under the caller's `# Safety` hardware-clock and exclusivity preconditions.
#[cfg(target_arch = "xtensa")]
#[no_mangle]
#[link_section = ".text.temprature_sens_read"]
pub unsafe extern "C" fn temprature_sens_read() -> u8 {
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
        let coex =
            hint::black_box(coex_bt_high_prio as unsafe extern "C" fn() as *const () as usize);
        let tsens = hint::black_box(
            temprature_sens_read as unsafe extern "C" fn() -> u8 as *const () as usize,
        );
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
}
