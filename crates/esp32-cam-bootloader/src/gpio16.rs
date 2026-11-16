//! Early GPIO 16 (PSRAM CS#) pull-up and output drive initialization.
//!
//! Enforces Invariants 70, 71, and 72 by ensuring PSRAM chip select is held
//! deasserted (high) during early boot stages, preventing bus floating and
//! maintaining hardware compatibility with ESP32-WROVER modules.

/// IO MUX configuration register for GPIO 16 (`soc/io_mux_reg.h`).
pub const IO_MUX_GPIO16_REG: u32 = 0x3FF4_904C;

/// GPIO 16 pull-up configuration: FUN_PU + FUN_IE + DRV=2 + MCU_SEL=2 GPIO.
pub const GPIO16_PULLUP_CFG: u32 = 0x0000_2B00;

/// RTC CNTL hold force register (`soc/rtc_cntl_reg.h`).
pub const RTC_CNTL_HOLD_FORCE_REG: u32 = 0x3FF4_80C8;

/// RTC IO digital pad hold register (`soc/rtc_io_reg.h`).
pub const RTC_IO_DIG_PAD_HOLD_REG: u32 = 0x3FF4_8474;

/// GPIO output set register for GPIOs 0..31 (`soc/gpio_reg.h`).
pub const GPIO_OUT_W1TS_REG: u32 = 0x3FF4_4008;

/// GPIO function output select configuration register for GPIO 16
/// (`soc/gpio_reg.h`).
pub const GPIO_FUNC16_OUT_SEL_CFG_REG: u32 = 0x3FF4_4570;

/// GPIO output enable set register for GPIOs 0..31 (`soc/gpio_reg.h`).
pub const GPIO_ENABLE_W1TS_REG: u32 = 0x3FF4_4024;

/// GPIO 16 simple output signal selection constant (bypass GPIO matrix).
pub const GPIO16_SIMPLE_OUT_SIG: u32 = 0x0000_0100;

/// Static table of GPIO 16 invariant words preserved in `.rodata` for
/// verification.
#[used]
pub static GPIO16_INVARIANT_WORDS: [u32; 7] = [
    IO_MUX_GPIO16_REG,
    GPIO16_PULLUP_CFG,
    RTC_CNTL_HOLD_FORCE_REG,
    RTC_IO_DIG_PAD_HOLD_REG,
    GPIO_OUT_W1TS_REG,
    GPIO_FUNC16_OUT_SEL_CFG_REG,
    GPIO_ENABLE_W1TS_REG,
];

/// Returns a pointer to the static invariant words table.
#[must_use]
pub fn invariant_words_ptr() -> *const u32 {
    GPIO16_INVARIANT_WORDS.as_ptr()
}

/// Abstract MMIO read/write access for GPIO registers.
pub trait GpioMmio {
    /// Reads a 32-bit value from the specified MMIO register address.
    fn read_u32(&self, addr: u32) -> u32;

    /// Writes a 32-bit value to the specified MMIO register address.
    fn write_u32(&mut self, addr: u32, val: u32);
}

/// Releases any deep-sleep or RTC hold state on GPIO 16 (PSRAM CS#).
pub fn release_psram_cs_hold<M: GpioMmio>(mmio: &mut M) {
    #[cfg(target_arch = "xtensa")]
    {
        // JUSTIFICATION: Volatile read forces retention of invariant words
        // table in .rodata for verify_invariants scanning under aggressive LTO.
        // SAFETY: Pointer is obtained from static array of length 7.
        let _ = unsafe { core::ptr::read_volatile(invariant_words_ptr()) };
    }
    let rtc_pad = mmio.read_u32(RTC_IO_DIG_PAD_HOLD_REG);
    mmio.write_u32(RTC_IO_DIG_PAD_HOLD_REG, rtc_pad & !(1 << 16));

    let rtc_cntl = mmio.read_u32(RTC_CNTL_HOLD_FORCE_REG);
    mmio.write_u32(RTC_CNTL_HOLD_FORCE_REG, rtc_cntl & !(1 << 16));
}

/// Initializes GPIO 16 as PSRAM CS# with pull-up and high output level.
pub fn init_gpio16_psram_cs<M: GpioMmio>(mmio: &mut M) {
    release_psram_cs_hold(mmio);
    mmio.write_u32(IO_MUX_GPIO16_REG, GPIO16_PULLUP_CFG);
    mmio.write_u32(GPIO_FUNC16_OUT_SEL_CFG_REG, GPIO16_SIMPLE_OUT_SIG);
    mmio.write_u32(GPIO_OUT_W1TS_REG, 1 << 16);
    mmio.write_u32(GPIO_ENABLE_W1TS_REG, 1 << 16);
}

#[cfg(target_arch = "xtensa")]
/// Real hardware MMIO accessor for Xtensa target.
pub struct XtensaGpioMmio;

#[cfg(target_arch = "xtensa")]
impl GpioMmio for XtensaGpioMmio {
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

#[cfg(target_arch = "xtensa")]
/// Initializes GPIO 16 on physical Xtensa hardware.
pub fn init_gpio16_xtensa() {
    let mut mmio = XtensaGpioMmio;
    init_gpio16_psram_cs(&mut mmio);
}

#[cfg(test)]
mod tests {
    use super::*;

    struct MockGpioMmio {
        rtc_pad_val: u32,
        rtc_cntl_val: u32,
        log: [(u32, u32); 8],
        log_len: usize,
    }

    impl MockGpioMmio {
        const fn new(rtc_pad_val: u32, rtc_cntl_val: u32) -> Self {
            Self {
                rtc_pad_val,
                rtc_cntl_val,
                log: [(0, 0); 8],
                log_len: 0,
            }
        }
    }

    impl GpioMmio for MockGpioMmio {
        fn read_u32(&self, addr: u32) -> u32 {
            match addr {
                RTC_IO_DIG_PAD_HOLD_REG => self.rtc_pad_val,
                RTC_CNTL_HOLD_FORCE_REG => self.rtc_cntl_val,
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
    fn test_gpio16_init_sequence() {
        let mut mock = MockGpioMmio::new(0xFFFF_FFFF, 0xFFFF_FFFF);
        init_gpio16_psram_cs(&mut mock);

        assert_eq!(mock.log_len, 6);
        assert_eq!(mock.log[0], (RTC_IO_DIG_PAD_HOLD_REG, !(1 << 16)));
        assert_eq!(mock.log[1], (RTC_CNTL_HOLD_FORCE_REG, !(1 << 16)));
        assert_eq!(mock.log[2], (IO_MUX_GPIO16_REG, 0x0000_2B00));
        assert_eq!(mock.log[3], (GPIO_FUNC16_OUT_SEL_CFG_REG, 0x0000_0100));
        assert_eq!(mock.log[4], (GPIO_OUT_W1TS_REG, 1 << 16));
        assert_eq!(mock.log[5], (GPIO_ENABLE_W1TS_REG, 1 << 16));

        assert_eq!(GPIO16_INVARIANT_WORDS[1], 0x2B00);
        assert!(!invariant_words_ptr().is_null());
    }
}
