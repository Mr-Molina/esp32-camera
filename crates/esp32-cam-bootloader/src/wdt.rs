//! Watchdog, brownout detection, and Wi-Fi domain isolation for early boot.
//!
//! Enforces Invariants 70 and 72 by re-arming the RTC watchdog timer, disabling
//! timer group 0 watchdog, isolating the Wi-Fi/RF power domain, and enabling
//! hardware brownout reset.

/// RTC CNTL options0 register for software reset (`soc/rtc_cntl_reg.h`).
pub const RTC_CNTL_OPTIONS0_REG: u32 = 0x3FF4_8000;

/// RTC CNTL digital power control register (`soc/rtc_cntl_reg.h`).
pub const RTC_CNTL_DIG_PWC_REG: u32 = 0x3FF4_8084;

/// RTC CNTL digital isolation register (`soc/rtc_cntl_reg.h`).
pub const RTC_CNTL_DIG_ISO_REG: u32 = 0x3FF4_8088;

/// RTC CNTL watchdog configuration register 0 (`soc/rtc_cntl_reg.h`).
pub const RTC_CNTL_WDTCONFIG0_REG: u32 = 0x3FF4_808C;

/// RTC CNTL watchdog configuration register 1 (`soc/rtc_cntl_reg.h`).
pub const RTC_CNTL_WDTCONFIG1_REG: u32 = 0x3FF4_8090;

/// RTC CNTL watchdog feed register (`soc/rtc_cntl_reg.h`).
pub const RTC_CNTL_WDTFEED_REG: u32 = 0x3FF4_80A0;

/// RTC CNTL watchdog write-protection register (`soc/rtc_cntl_reg.h`).
pub const RTC_CNTL_WDTWPROTECT_REG: u32 = 0x3FF4_80A4;

/// RTC CNTL store6 scratch register for boot tracking (`soc/rtc_cntl_reg.h`).
pub const RTC_CNTL_STORE6_REG: u32 = 0x3FF4_80B8;

/// RTC CNTL store7 scratch register for boot stage breadcrumbs
/// (`soc/rtc_cntl_reg.h`).
pub const RTC_CNTL_STORE7_REG: u32 = 0x3FF4_80BC;

/// RTC CNTL brownout detector configuration register (`soc/rtc_cntl_reg.h`).
pub const RTC_CNTL_BROWN_OUT_REG: u32 = 0x3FF4_80D4;

/// Timer Group 0 watchdog configuration register 0 (`soc/timer_group_reg.h`).
pub const TIMG0_WDTCONFIG0_REG: u32 = 0x3FF5_F048;

/// Timer Group 0 watchdog write-protection register (`soc/timer_group_reg.h`).
pub const TIMG0_WDTWPROTECT_REG: u32 = 0x3FF5_F064;

/// Watchdog unlock write key (`0x50D8_3AA1`).
pub const RTC_WDT_WKEY: u32 = 0x50D8_3AA1;

/// Brownout configuration word enabling brownout reset, maximum threshold,
/// and RF power down.
pub const BROWN_OUT_BOOT_CFG: u32 =
    (1 << 30) | (7 << 27) | (1 << 26) | (0x3FF << 16) | (1 << 15) | (1 << 14);

/// RTC watchdog configuration word for ~12s boot timeout with RTC reset action.
pub const RTC_WDT_BOOT_CFG0: u32 = (1 << 31) | (3 << 28) | (1 << 9) | (1 << 8);

/// Abstract MMIO read/write access for Watchdog and RTC registers.
pub trait WdtMmio {
    /// Reads a 32-bit value from the specified MMIO register address.
    fn read_u32(&self, addr: u32) -> u32;

    /// Writes a 32-bit value to the specified MMIO register address.
    fn write_u32(&mut self, addr: u32, val: u32);
}

/// Configures watchdogs, Wi-Fi domain isolation, and brownout detection for
/// early boot.
pub fn configure_watchdogs_and_isolation<M: WdtMmio>(mmio: &mut M) {
    // 1. Disable Timer Group 0 MWDT.
    mmio.write_u32(TIMG0_WDTWPROTECT_REG, RTC_WDT_WKEY);
    mmio.write_u32(TIMG0_WDTCONFIG0_REG, 0);
    mmio.write_u32(TIMG0_WDTWPROTECT_REG, 0);

    // 2. Arm RTC Watchdog for ~12 second timeout.
    mmio.write_u32(RTC_CNTL_WDTWPROTECT_REG, RTC_WDT_WKEY);
    mmio.write_u32(RTC_CNTL_WDTCONFIG1_REG, 0x0010_0000);
    mmio.write_u32(RTC_CNTL_WDTCONFIG0_REG, RTC_WDT_BOOT_CFG0);
    mmio.write_u32(RTC_CNTL_WDTFEED_REG, 1 << 31);
    mmio.write_u32(RTC_CNTL_WDTWPROTECT_REG, 0);

    // 3. Isolate and power down Wi-Fi/RF modem domain.
    let iso = mmio.read_u32(RTC_CNTL_DIG_ISO_REG);
    mmio.write_u32(RTC_CNTL_DIG_ISO_REG, (iso & !(1 << 29)) | (1 << 28));

    let pwc = mmio.read_u32(RTC_CNTL_DIG_PWC_REG);
    mmio.write_u32(RTC_CNTL_DIG_PWC_REG, (pwc & !(1 << 18)) | (1 << 17));

    // 4. Configure brownout detector.
    mmio.write_u32(RTC_CNTL_BROWN_OUT_REG, BROWN_OUT_BOOT_CFG);
}

/// Records a boot stage breadcrumb into `RTC_CNTL_STORE7_REG`.
pub fn record_boot_stage<M: WdtMmio>(mmio: &mut M, stage: u32) {
    mmio.write_u32(RTC_CNTL_STORE7_REG, stage);
}

/// Triggers an immediate software reset via `RTC_CNTL_OPTIONS0_REG`.
pub fn trigger_sw_reset<M: WdtMmio>(mmio: &mut M) {
    let opt = mmio.read_u32(RTC_CNTL_OPTIONS0_REG);
    mmio.write_u32(RTC_CNTL_OPTIONS0_REG, opt | (1 << 31));
}

#[cfg(target_arch = "xtensa")]
/// Real hardware MMIO accessor for Xtensa Wdt and RTC registers.
pub struct XtensaWdtMmio;

#[cfg(target_arch = "xtensa")]
impl WdtMmio for XtensaWdtMmio {
    #[inline]
    fn read_u32(&self, addr: u32) -> u32 {
        // JUSTIFICATION: Reading hardware MMIO registers on physical Xtensa SoC.
        // SAFETY: Address must be a valid 4-byte aligned ESP32 MMIO register.
        unsafe { core::ptr::read_volatile(addr as *const u32) }
    }

    #[inline]
    fn write_u32(&mut self, addr: u32, val: u32) {
        // JUSTIFICATION: Writing hardware MMIO registers on physical Xtensa SoC.
        // SAFETY: Address must be a valid 4-byte aligned ESP32 MMIO register.
        unsafe { core::ptr::write_volatile(addr as *mut u32, val) }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct MockWdtMmio {
        iso_val: u32,
        pwc_val: u32,
        opt_val: u32,
        log: [(u32, u32); 16],
        log_len: usize,
    }

    impl MockWdtMmio {
        const fn new() -> Self {
            Self {
                iso_val: 1 << 29, // WIFI_FORCE_NOISO initially set
                pwc_val: 1 << 18, // WIFI_FORCE_PU initially set
                opt_val: 0,
                log: [(0, 0); 16],
                log_len: 0,
            }
        }
    }

    impl WdtMmio for MockWdtMmio {
        fn read_u32(&self, addr: u32) -> u32 {
            match addr {
                RTC_CNTL_DIG_ISO_REG => self.iso_val,
                RTC_CNTL_DIG_PWC_REG => self.pwc_val,
                RTC_CNTL_OPTIONS0_REG => self.opt_val,
                _ => 0,
            }
        }

        fn write_u32(&mut self, addr: u32, val: u32) {
            if self.log_len < self.log.len() {
                self.log[self.log_len] = (addr, val);
                self.log_len += 1;
            }
        }
    }

    #[test]
    fn test_wdt_and_isolation_flow() {
        let mut mock = MockWdtMmio::new();
        configure_watchdogs_and_isolation(&mut mock);

        // Verify MWDT unlock, disable, lock.
        assert_eq!(mock.log[0], (TIMG0_WDTWPROTECT_REG, RTC_WDT_WKEY));
        assert_eq!(mock.log[1], (TIMG0_WDTCONFIG0_REG, 0));
        assert_eq!(mock.log[2], (TIMG0_WDTWPROTECT_REG, 0));

        // Verify RTC WDT arming sequence.
        assert_eq!(mock.log[3], (RTC_CNTL_WDTWPROTECT_REG, RTC_WDT_WKEY));
        assert_eq!(mock.log[4], (RTC_CNTL_WDTCONFIG1_REG, 0x0010_0000));
        assert_eq!(mock.log[5], (RTC_CNTL_WDTCONFIG0_REG, RTC_WDT_BOOT_CFG0));
        assert_eq!(mock.log[6], (RTC_CNTL_WDTFEED_REG, 1 << 31));
        assert_eq!(mock.log[7], (RTC_CNTL_WDTWPROTECT_REG, 0));

        // Verify Wi-Fi isolation and power down.
        assert_eq!(mock.log[8], (RTC_CNTL_DIG_ISO_REG, 1 << 28));
        assert_eq!(mock.log[9], (RTC_CNTL_DIG_PWC_REG, 1 << 17));

        // Verify Brownout detector.
        assert_eq!(mock.log[10], (RTC_CNTL_BROWN_OUT_REG, BROWN_OUT_BOOT_CFG));
    }

    #[test]
    fn test_boot_stage_and_sw_reset() {
        let mut mock = MockWdtMmio::new();
        record_boot_stage(&mut mock, 0x42);
        assert_eq!(mock.log[0], (RTC_CNTL_STORE7_REG, 0x42));

        trigger_sw_reset(&mut mock);
        assert_eq!(mock.log[1], (RTC_CNTL_OPTIONS0_REG, 1 << 31));
    }
}
