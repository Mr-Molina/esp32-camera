//! Low-level MAC (LMAC) MMIO driver, register sequencing, and bus abstraction.
//!
//! Provides the [`WifiMmioBus`] trait, [`HardwareWifiMmioBus`] for direct
//! Xtensa/ESP32 MMIO operations, [`MockWifiMmioBus`] for testing without
//! hardware, and [`LmacHal`] implementing reverse-engineered LMAC hardware
//! initialization, BSSID filtering, interrupt handling, conditioned hardware
//! RNG, and TX sequencing.

use crate::regs;
use crate::regs::WifiPhyRate;
use core::cell::Cell;
use core::fmt;
use core::sync::atomic::AtomicBool;
use core::sync::atomic::Ordering;

/// Maximum permitted TX poll iterations in [`LmacHal::submit_and_poll_tx`].
pub const MAX_TX_POLLS: u32 = 1_000_000;

/// Maximum TX active drain poll iterations on abort/deinit.
const MAX_TX_DRAIN_POLLS: u32 = 200;

/// Maximum poll iterations waiting for `RX_CTRL_BIT_RELOAD` to self-clear.
const MAX_RX_RELOAD_POLLS: u32 = 10_000;

/// Software-CCMP TX bypass bits (`tx_info[16] = 0xC0 << 17`) in `TX_PLCP1_REG[24:17]`.
const TX_PLCP1_SW_CRYPTO_BYPASS_BITS: u32 = 0xC0u32 << 17;

/// Singleton acquisition guard for [`HardwareWifiMmioBus::take`].
static HW_BUS_TAKEN: AtomicBool = AtomicBool::new(false);

/// Errors returned by LMAC hardware operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LmacError {
    /// MAC reset timed out waiting for register 0x3D24 bit 0 ready signal.
    MacResetTimeout,
    /// Transmission payload length was zero or exceeded maximum 1600 bytes.
    InvalidPayloadLength,
    /// Hardware transmission error status was asserted.
    TxHardwareError,
    /// Hardware transmission completed but peer ACK was not received.
    TxAckFailed(u32),
    /// Transmission poll loop exceeded maximum iterations without completion.
    TxTimeout,
    /// Hardware TX abort failed to drain `TX_ACTIVE == 0` even after pulsing
    /// the `MAC_INIT_REG_3C58` abort reset; DMA may still hold the TX buffer.
    TxDrainFailed,
    /// DMA descriptor address is null, misaligned, or outside ESP32 DRAM.
    InvalidDmaAddress,
    /// Requested poll limit is zero or exceeds [`MAX_TX_POLLS`].
    InvalidPollCount,
    /// Requested PHY rate is unconfirmed/unsupported (e.g., provisional SGI).
    UnsupportedRate,
    /// Hardware RNG was queried before RF/MAC entropy source initialization.
    RngNotReady,
    /// Hardware RNG failed stuck-at repetition health checks.
    RngHealthCheckFailed,
    /// eFuse factory MAC failed CRC-8 or unicast/non-zero validation.
    InvalidEfuseMac,
}

impl fmt::Display for LmacError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::MacResetTimeout => f.write_str("MAC reset timed out waiting for 3D24 ready bit"),
            Self::InvalidPayloadLength => {
                f.write_str("invalid TX payload length (must be 1..=1600)")
            }
            Self::TxHardwareError => f.write_str("hardware error during transmission"),
            Self::TxAckFailed(status) => {
                write!(f, "transmission ACK failed with status code {status}")
            }
            Self::TxTimeout => f.write_str("transmission timed out waiting for completion"),
            Self::TxDrainFailed => {
                f.write_str("hardware TX remained active after abort and MAC reset pulse")
            }
            Self::InvalidDmaAddress => {
                f.write_str("invalid DMA descriptor address (alignment or DRAM window)")
            }
            Self::InvalidPollCount => {
                write!(f, "invalid TX poll count (must be 1..={MAX_TX_POLLS})")
            }
            Self::UnsupportedRate => {
                f.write_str("unsupported PHY transmission rate (provisional SGI disabled)")
            }
            Self::RngNotReady => f.write_str("hardware RNG queried before MAC/RF initialization"),
            Self::RngHealthCheckFailed => f.write_str("hardware RNG stuck-at health check failed"),
            Self::InvalidEfuseMac => {
                f.write_str("eFuse factory MAC failed CRC-8 or unicast validation")
            }
        }
    }
}

/// Decoded status flags from MAC interrupt status register.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct LmacIrqEvents {
    /// One or more RX DMA descriptors completed and frame is available.
    pub rx_ready: bool,
    /// Frame transmission completed.
    pub tx_done: bool,
    /// Transmission collision detected.
    pub tx_collision: bool,
    /// Transmission watchdog timeout occurred.
    pub tx_timeout: bool,
    /// Wi-Fi MAC panic watchdog interrupt asserted.
    pub panic_wdt: bool,
    /// Raw interrupt status bits not decoded/cleared by this driver.
    pub raw_unhandled: u32,
}

/// Abstract bus interface for reading and writing Wi-Fi MAC MMIO registers.
pub trait WifiMmioBus {
    /// Reads a 32-bit register from the given MMIO address.
    fn read_reg(&self, addr: u32) -> u32;

    /// Writes a 32-bit value to the given MMIO address.
    fn write_reg(&mut self, addr: u32, val: u32);

    /// Delays execution for approximately `us` microseconds.
    fn delay_us(&mut self, us: u32);

    /// Issues a hardware memory write barrier / fence.
    fn mem_barrier(&mut self);
}

/// Direct hardware MMIO bus implementation for ESP32 silicon.
///
/// Intentionally neither `Copy`, `Clone`, nor `Default`, and enforces a strict
/// 4-byte aligned ESP32 Wi-Fi/eFuse/DPORT MMIO allowlist on every register
/// access so safe Rust cannot dereference arbitrary addresses.
#[derive(Debug)]
pub struct HardwareWifiMmioBus {
    _private: (),
}

impl HardwareWifiMmioBus {
    /// Creates a new direct [`HardwareWifiMmioBus`] for `const` static
    /// singleton placement; prefer [`Self::take`] at runtime.
    ///
    /// # Safety
    ///
    /// Caller MUST guarantee that at most one [`HardwareWifiMmioBus`] instance
    /// exists across the entire program (and never mix with [`Self::take`]),
    /// preventing aliased unsynchronized MMIO register access (`M-016`).
    // JUSTIFICATION: `const fn` static singleton construction cannot touch the
    // runtime `HW_BUS_TAKEN` atomic; `unsafe` enforces singleton uniqueness.
    #[allow(clippy::new_without_default)]
    #[inline]
    #[must_use]
    pub const unsafe fn new() -> Self {
        Self { _private: () }
    }

    /// Acquires the singleton [`HardwareWifiMmioBus`] handle once at runtime.
    #[must_use]
    pub fn take() -> Option<Self> {
        HW_BUS_TAKEN
            .compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed)
            .ok()
            .map(|_| Self { _private: () })
    }
}

#[cfg(target_arch = "xtensa")]
#[inline]
fn xtensa_read_ccount() -> u32 {
    let val: u32;
    // JUSTIFICATION: Reading Xtensa CCOUNT provides a non-elidable hardware
    // cycle counter for calibrated microsecond delays.
    // SAFETY: `rsr.ccount` only reads the processor cycle counter register.
    unsafe {
        core::arch::asm!(
            "rsr.ccount {0}",
            out(reg) val,
            options(nomem, nostack, preserves_flags)
        );
    }
    val
}

impl WifiMmioBus for HardwareWifiMmioBus {
    #[inline]
    fn read_reg(&self, addr: u32) -> u32 {
        if !regs::is_valid_wifi_mmio_read_addr(addr) {
            return 0;
        }
        #[cfg(target_arch = "xtensa")]
        {
            // JUSTIFICATION: Hardware MMIO register access needs volatile read.
            // SAFETY: `addr` passed `is_valid_wifi_mmio_read_addr` confirming
            // 4-byte alignment and membership in ESP32 Wi-Fi/eFuse/DPORT MMIO.
            unsafe { core::ptr::read_volatile(addr as *const u32) }
        }
        #[cfg(not(target_arch = "xtensa"))]
        {
            let _ = addr;
            0
        }
    }

    #[inline]
    fn write_reg(&mut self, addr: u32, val: u32) {
        if !regs::is_valid_wifi_mmio_write_addr(addr) {
            return;
        }
        #[cfg(target_arch = "xtensa")]
        {
            // JUSTIFICATION: Hardware MMIO register access needs volatile write.
            // SAFETY: `addr` passed `is_valid_wifi_mmio_write_addr` confirming
            // 4-byte alignment and membership in writable ESP32 Wi-Fi MMIO.
            unsafe { core::ptr::write_volatile(addr as *mut u32, val) }
        }
        #[cfg(not(target_arch = "xtensa"))]
        {
            let _ = (addr, val);
        }
    }

    #[inline]
    fn delay_us(&mut self, us: u32) {
        #[cfg(target_arch = "xtensa")]
        {
            const CYCLES_PER_US: u32 = 240;
            let mut remaining = us;
            while remaining > 0 {
                // Chunk to <= 1_000_000 us (< 2^31 cycles at 240 MHz) to avoid
                // u32 CCOUNT wraparound ambiguity.
                let chunk_us = remaining.min(1_000_000);
                let target_cycles = chunk_us.saturating_mul(CYCLES_PER_US);
                let start = xtensa_read_ccount();
                while xtensa_read_ccount().wrapping_sub(start) < target_cycles {
                    // Volatile CCOUNT read cannot be elided by LLVM.
                }
                remaining -= chunk_us;
            }
        }
        #[cfg(not(target_arch = "xtensa"))]
        {
            let mut spins = us.min(1000).saturating_mul(16);
            while spins > 0 {
                spins = core::hint::black_box(spins - 1);
            }
        }
    }

    #[inline]
    fn mem_barrier(&mut self) {
        #[cfg(target_arch = "xtensa")]
        {
            // JUSTIFICATION: Xtensa bus ordering requires `memw` barrier.
            // SAFETY: `memw` drains write buffers and has no memory operand.
            unsafe {
                core::arch::asm!("memw", options(nostack, preserves_flags));
            }
        }
        #[cfg(not(target_arch = "xtensa"))]
        {
            core::sync::atomic::fence(Ordering::SeqCst);
        }
    }
}

/// Mock MMIO bus for testing LMAC register sequences without hardware.
///
/// Stores up to 512 unique 32-bit register addresses and their values, enforces
/// MMIO alignment/allowlist parity, emulates W1C status clearing and
/// trigger-gated TX completion, and records barrier/delay telemetry.
/// Fully `#![no_std]`, zero heap allocation, and zero `unsafe`.
#[derive(Debug, Clone)]
pub struct MockWifiMmioBus {
    regs: [(u32, u32); 512],
    count: usize,
    /// When `true`, `MAC_INIT_REG_3D24` returns bit 0 set once bit 1 is set.
    pub auto_init_3d24: bool,
    /// When `true`, reading `TX_DONE_STATUS_REG` returns bit 0 set while TX
    /// trigger bits (`TX_PLCP0_TRIGGER_BITS`) are asserted.
    pub auto_tx_done: bool,
    /// ACK status value returned in `TX_PMD_REG` bits [15:12] when
    /// `auto_tx_done` is true and TX was triggered.
    pub auto_tx_status: u32,
    /// When non-zero, reading `TX_ERR_STATUS_REG` returns these bits while
    /// `TX_PLCP0_TRIGGER_BITS` are asserted (simulating in-flight TX HW error).
    pub auto_tx_err: u32,
    /// When `true`, pulsing the `MAC_INIT_REG_3C58` TX abort reset bit
    /// (`2 << 16`) clears `MAC_CLK_RST_BIT_TX_ACTIVE` in `MAC_CLK_RST_REG`.
    pub auto_clear_tx_active_on_abort_reset: bool,
    /// Optional fixed value returned for every `WDEV_RND_REG` read to test
    /// stuck-at RNG health checks. When `None`, reads yield a varying sequence.
    pub stuck_rng: Option<u32>,
    /// Internal PRNG state for varying `WDEV_RND_REG` mock reads.
    rng_state: Cell<u32>,
    /// Count of write operations recorded by the mock bus.
    pub write_count: usize,
    /// Count of `RX_CTRL_BIT_RELOAD` pulses observed on `RX_CTRL_REG`.
    pub rx_ctrl_reload_count: usize,
    /// Last `TX_PLCP0_TRIG_REG` value written with trigger bits asserted.
    pub last_tx_plcp0_triggered: u32,
    /// Value of `HW_CRYPTO_DISABLE_START` observed when `TX_PLCP0_TRIG_REG` was triggered.
    pub crypto_ctrl_at_tx_trigger: u32,
    /// Count of `mem_barrier` calls recorded by the mock bus.
    pub barrier_count: usize,
    /// Total microseconds passed to `delay_us`.
    pub delay_us_total: u64,
}

impl Default for MockWifiMmioBus {
    fn default() -> Self {
        Self::new()
    }
}

impl MockWifiMmioBus {
    /// Creates an empty [`MockWifiMmioBus`].
    #[must_use]
    pub const fn new() -> Self {
        Self {
            regs: [(0, 0); 512],
            count: 0,
            auto_init_3d24: false,
            auto_tx_done: false,
            auto_tx_status: 0,
            auto_tx_err: 0,
            auto_clear_tx_active_on_abort_reset: false,
            stuck_rng: None,
            rng_state: Cell::new(0xA5A5_5A5A),
            write_count: 0,
            rx_ctrl_reload_count: 0,
            last_tx_plcp0_triggered: 0,
            crypto_ctrl_at_tx_trigger: 0,
            barrier_count: 0,
            delay_us_total: 0,
        }
    }

    /// Returns the current value of the register at `addr`, or 0 if unwritten.
    #[must_use]
    pub fn reg(&self, addr: u32) -> u32 {
        let mut idx = 0;
        while idx < self.count {
            if self.regs[idx].0 == addr {
                return self.regs[idx].1;
            }
            idx += 1;
        }
        0
    }

    /// Sets or updates the stored register value at `addr`.
    ///
    /// # Panics
    /// Panics if `addr` is misaligned, outside the ESP32 Wi-Fi/eFuse/DPORT
    /// MMIO allowlist, or if the 512-entry mock register table overflows.
    pub fn set_reg(&mut self, addr: u32, val: u32) {
        assert!(
            regs::is_valid_wifi_mmio_read_addr(addr),
            "MockWifiMmioBus: invalid or misaligned MMIO address 0x{addr:08X}"
        );
        let mut idx = 0;
        while idx < self.count {
            if self.regs[idx].0 == addr {
                self.regs[idx].1 = val;
                return;
            }
            idx += 1;
        }
        assert!(
            self.count < self.regs.len(),
            "MockWifiMmioBus register capacity (512) exceeded"
        );
        self.regs[self.count] = (addr, val);
        self.count += 1;
    }

    #[inline]
    fn tx_triggered(&self) -> bool {
        (self.reg(regs::TX_PLCP0_TRIG_REG) & regs::TX_PLCP0_TRIGGER_BITS)
            == regs::TX_PLCP0_TRIGGER_BITS
    }
}

impl WifiMmioBus for MockWifiMmioBus {
    fn read_reg(&self, addr: u32) -> u32 {
        assert!(
            regs::is_valid_wifi_mmio_read_addr(addr),
            "MockWifiMmioBus::read_reg: invalid MMIO address 0x{addr:08X}"
        );

        if addr == regs::WDEV_RND_REG {
            if let Some(fixed) = self.stuck_rng {
                return fixed;
            }
            let seeded = self.reg(regs::WDEV_RND_REG);
            let mut x = self.rng_state.get();
            if seeded != 0 && x == 0xA5A5_5A5A {
                x = seeded;
            }
            // xorshift32 for non-stuck varying mock entropy words.
            x ^= x << 13;
            x ^= x >> 17;
            x ^= x << 5;
            if x == 0 {
                x = 0x6D2B_79F5;
            }
            self.rng_state.set(x);
            return x;
        }

        let mut val = self.reg(addr);
        if addr == regs::MAC_INIT_REG_3D24 && self.auto_init_3d24 && (val & 2) != 0 {
            val |= 1;
        }
        if addr == regs::TX_ERR_STATUS_REG && self.auto_tx_err != 0 && self.tx_triggered() {
            val |= self.auto_tx_err;
        }
        if addr == regs::TX_DONE_STATUS_REG && self.auto_tx_done && self.tx_triggered() {
            val |= 1;
        }
        if addr == regs::TX_PMD_REG && self.auto_tx_done && self.tx_triggered() {
            val = (val & !0xF000) | ((self.auto_tx_status & 0x0F) << 12);
        }
        val
    }

    fn write_reg(&mut self, addr: u32, val: u32) {
        assert!(
            regs::is_valid_wifi_mmio_write_addr(addr),
            "MockWifiMmioBus::write_reg: invalid/RO MMIO address 0x{addr:08X}"
        );
        self.write_count = self.write_count.saturating_add(1);
        self.set_reg(addr, val);

        if addr == regs::TX_PLCP0_TRIG_REG
            && (val & regs::TX_PLCP0_TRIGGER_BITS) == regs::TX_PLCP0_TRIGGER_BITS
        {
            self.last_tx_plcp0_triggered = val;
            self.crypto_ctrl_at_tx_trigger = self.reg(regs::HW_CRYPTO_DISABLE_START);
        }

        // Emulate hardware W1C / self-clearing side effects on registers.
        if addr == regs::INT_CLEAR_REG {
            let cur = self.reg(regs::INT_STATUS_REG);
            self.set_reg(regs::INT_STATUS_REG, cur & !val);
        } else if addr == regs::TX_DONE_CLEAR_REG {
            let cur = self.reg(regs::TX_DONE_STATUS_REG);
            self.set_reg(regs::TX_DONE_STATUS_REG, cur & !val);
        } else if addr == regs::TX_ERR_CLEAR_REG {
            let cur = self.reg(regs::TX_ERR_STATUS_REG);
            self.set_reg(regs::TX_ERR_STATUS_REG, cur & !val);
        } else if addr == regs::MAC_INIT_REG_3C58
            && self.auto_clear_tx_active_on_abort_reset
            && (val & (2 << 16)) != 0
        {
            let cur = self.reg(regs::MAC_CLK_RST_REG);
            self.set_reg(
                regs::MAC_CLK_RST_REG,
                cur & !regs::MAC_CLK_RST_BIT_TX_ACTIVE,
            );
        } else if addr == regs::RX_CTRL_REG && (val & regs::RX_CTRL_BIT_RELOAD) != 0 {
            self.rx_ctrl_reload_count = self.rx_ctrl_reload_count.saturating_add(1);
            self.set_reg(regs::RX_CTRL_REG, val & !regs::RX_CTRL_BIT_RELOAD);
        }
    }

    fn delay_us(&mut self, us: u32) {
        self.delay_us_total = self.delay_us_total.saturating_add(u64::from(us));
    }

    fn mem_barrier(&mut self) {
        self.barrier_count = self.barrier_count.saturating_add(1);
    }
}

/// Validates a DMA descriptor address for alignment and (on Xtensa) ESP32
/// internal DMA DRAM bounds (`0x3FFA_E000..=0x3FFF_FFFF`).
#[inline]
fn validate_dma_desc_addr(addr: u32) -> Result<(), LmacError> {
    if addr == 0 || (addr & 0x3) != 0 {
        return Err(LmacError::InvalidDmaAddress);
    }
    #[cfg(target_arch = "xtensa")]
    if !regs::is_valid_xtensa_dma_addr(addr) {
        return Err(LmacError::InvalidDmaAddress);
    }
    Ok(())
}

/// Computes ESP32 ROM `esp_rom_crc8` (reflected polynomial `0x8C`, init `0`)
/// over a 6-byte MAC address stored in eFuse Block 0.
#[must_use]
pub const fn efuse_mac_crc8(mac: &[u8; 6]) -> u8 {
    let mut crc = 0u8;
    let mut i = 0;
    while i < 6 {
        let mut b = mac[i];
        let mut bit = 0;
        while bit < 8 {
            if ((crc ^ b) & 1) != 0 {
                crc = (crc >> 1) ^ 0x8C;
            } else {
                crc >>= 1;
            }
            b >>= 1;
            bit += 1;
        }
        i += 1;
    }
    crc
}

/// FIPS 180-4 SHA-256 initial state constants used for RNG conditioning.
const SHA256_IV: [u32; 8] = [
    0x6A09_E667,
    0xBB67_AE85,
    0x3C6E_F372,
    0xA54F_F53A,
    0x510E_527F,
    0x9B05_688C,
    0x1F83_D9AB,
    0x5BE0_CD19,
];

/// FIPS 180-4 SHA-256 round constants.
const SHA256_K: [u32; 64] = [
    0x428A_2F98,
    0x7137_4491,
    0xB5C0_FBCF,
    0xE9B5_DBA5,
    0x3956_C25B,
    0x59F1_11F1,
    0x923F_82A4,
    0xAB1C_5ED5,
    0xD807_AA98,
    0x1283_5B01,
    0x2431_85BE,
    0x550C_7DC3,
    0x72BE_5D74,
    0x80DE_B1FE,
    0x9BDC_06A7,
    0xC19B_F174,
    0xE49B_69C1,
    0xEFBE_4786,
    0x0FC1_9DC6,
    0x240C_A1CC,
    0x2DE9_2C6F,
    0x4A74_84AA,
    0x5CB0_A9DC,
    0x76F9_88DA,
    0x983E_5152,
    0xA831_C66D,
    0xB003_27C8,
    0xBF59_7FC7,
    0xC6E0_0BF3,
    0xD5A7_9147,
    0x06CA_6351,
    0x1429_2967,
    0x27B7_0A85,
    0x2E1B_2138,
    0x4D2C_6DFC,
    0x5338_0D13,
    0x650A_7354,
    0x766A_0ABB,
    0x81C2_C92E,
    0x9272_2C85,
    0xA2BF_E8A1,
    0xA81A_664B,
    0xC24B_8B70,
    0xC76C_51A3,
    0xD192_E819,
    0xD699_0624,
    0xF40E_3585,
    0x106A_A070,
    0x19A4_C116,
    0x1E37_6C08,
    0x2748_774C,
    0x34B0_BCB5,
    0x391C_0CB3,
    0x4ED8_AA4A,
    0x5B9C_CA4F,
    0x682E_6FF3,
    0x748F_82EE,
    0x78A5_636F,
    0x84C8_7814,
    0x8CC7_0208,
    0x90BE_FFFA,
    0xA450_6CEB,
    0xBEF9_A3F7,
    0xC671_78F2,
];

/// Single-block SHA-256 compression conditioning raw hardware RNG samples into
/// the internal entropy pool.
fn sha256_compress_pool(state: &mut [u32; 8], input: &[u32; 16]) {
    let mut w = [0u32; 64];
    let mut i = 0;
    while i < 16 {
        w[i] = input[i];
        i += 1;
    }
    while i < 64 {
        let s0 = w[i - 15].rotate_right(7) ^ w[i - 15].rotate_right(18) ^ (w[i - 15] >> 3);
        let s1 = w[i - 2].rotate_right(17) ^ w[i - 2].rotate_right(19) ^ (w[i - 2] >> 10);
        w[i] = w[i - 16]
            .wrapping_add(s0)
            .wrapping_add(w[i - 7])
            .wrapping_add(s1);
        i += 1;
    }

    let mut a = state[0];
    let mut b = state[1];
    let mut c = state[2];
    let mut d = state[3];
    let mut e = state[4];
    let mut f = state[5];
    let mut g = state[6];
    let mut h = state[7];

    let mut r = 0;
    while r < 64 {
        let s1 = e.rotate_right(6) ^ e.rotate_right(11) ^ e.rotate_right(25);
        let ch = (e & f) ^ ((!e) & g);
        let temp1 = h
            .wrapping_add(s1)
            .wrapping_add(ch)
            .wrapping_add(SHA256_K[r])
            .wrapping_add(w[r]);
        let s0 = a.rotate_right(2) ^ a.rotate_right(13) ^ a.rotate_right(22);
        let maj = (a & b) ^ (a & c) ^ (b & c);
        let temp2 = s0.wrapping_add(maj);

        h = g;
        g = f;
        f = e;
        e = d.wrapping_add(temp1);
        d = c;
        c = b;
        b = a;
        a = temp1.wrapping_add(temp2);
        r += 1;
    }

    state[0] = state[0].wrapping_add(a);
    state[1] = state[1].wrapping_add(b);
    state[2] = state[2].wrapping_add(c);
    state[3] = state[3].wrapping_add(d);
    state[4] = state[4].wrapping_add(e);
    state[5] = state[5].wrapping_add(f);
    state[6] = state[6].wrapping_add(g);
    state[7] = state[7].wrapping_add(h);
}

/// Hardware Abstraction Layer for the ESP32 Wi-Fi Low-Level MAC (LMAC).
pub struct LmacHal<B: WifiMmioBus> {
    bus: B,
    mac_initialized: bool,
    vif0_crypto_ctrl: u32,
    rng_pool: [u32; 8],
    rng_counter: u64,
    last_raw_rng: u32,
}

impl<B: WifiMmioBus> LmacHal<B> {
    /// Creates a new [`LmacHal`] wrapping the provided bus interface.
    #[inline]
    pub const fn new(bus: B) -> Self {
        Self {
            bus,
            mac_initialized: false,
            vif0_crypto_ctrl: 0x0003_0000,
            rng_pool: SHA256_IV,
            rng_counter: 0,
            last_raw_rng: 0,
        }
    }

    /// Returns `true` if [`Self::init_mac`] succeeded and [`Self::deinit_mac`]
    /// has not been called.
    #[inline]
    #[must_use]
    pub const fn is_initialized(&self) -> bool {
        self.mac_initialized
    }

    /// Returns `true` if hardware CCMP RX decryption is enabled for STA VIF 0.
    #[inline]
    #[must_use]
    pub const fn is_sta_hw_crypto_enabled(&self) -> bool {
        self.vif0_crypto_ctrl != 0x0003_0000
    }

    /// Returns an immutable reference to the underlying bus.
    #[inline]
    pub const fn bus(&self) -> &B {
        &self.bus
    }

    /// Returns a mutable reference to the underlying bus.
    #[inline]
    pub fn bus_mut(&mut self) -> &mut B {
        &mut self.bus
    }

    /// Consumes the HAL wrapper and returns the underlying bus.
    #[inline]
    pub fn into_bus(self) -> B {
        self.bus
    }

    /// Initializes MAC clocks, resets core logic, programs operational registers,
    /// clears crypto key slots, configures STA MAC address, arms RX DMA, and
    /// enables operational interrupts last.
    pub fn init_mac(&mut self, sta_mac: [u8; 6], rx_ring_base_addr: u32) -> Result<(), LmacError> {
        // Validate RX DMA ring address before touching any hardware register.
        validate_dma_desc_addr(rx_ring_base_addr)?;

        self.mac_initialized = false;
        self.vif0_crypto_ctrl = 0x0003_0000;

        // 1. hal_mac_init: un-gate MAC clocks
        let r_clk = self.bus.read_reg(regs::MAC_CLK_RST_REG);
        self.bus.write_reg(
            regs::MAC_CLK_RST_REG,
            r_clk & !regs::MAC_CLK_RST_MASK_UNGATE,
        );
        self.bus.mem_barrier();

        // 2. hal_init: assert reset bit, wait for MAC ready bit
        let r3d24 = self.bus.read_reg(regs::MAC_INIT_REG_3D24);
        self.bus.write_reg(regs::MAC_INIT_REG_3D24, r3d24 | 2);
        self.bus.mem_barrier();

        #[cfg(target_arch = "xtensa")]
        const MAX_READY_POLLS: u32 = 1_000_000;
        #[cfg(not(target_arch = "xtensa"))]
        const MAX_READY_POLLS: u32 = 10_000;

        let mut ready = false;
        let mut poll_count = 0;
        while poll_count < MAX_READY_POLLS {
            if (self.bus.read_reg(regs::MAC_INIT_REG_3D24) & 1) != 0 {
                ready = true;
                break;
            }
            core::hint::spin_loop();
            poll_count += 1;
        }
        if !ready {
            // M-165: Clear reset bit and re-gate MAC clocks on timeout.
            let r3d24_cur = self.bus.read_reg(regs::MAC_INIT_REG_3D24);
            self.bus.write_reg(regs::MAC_INIT_REG_3D24, r3d24_cur & !2);
            let clk_cur = self.bus.read_reg(regs::MAC_CLK_RST_REG);
            self.bus.write_reg(
                regs::MAC_CLK_RST_REG,
                clk_cur | regs::MAC_CLK_RST_MASK_UNGATE,
            );
            self.bus.mem_barrier();
            return Err(LmacError::MacResetTimeout);
        }

        // Mask primary and secondary interrupt enable registers before setup.
        self.bus.write_reg(regs::INT_ENABLE_REG, 0);
        self.bus.write_reg(regs::INT_ENABLE_SEC_REG, 0);
        self.bus.write_reg(regs::INT_CLEAR_REG, 0xFFFF_FFFF);
        self.bus.mem_barrier();

        // 3. mac_txrx_init
        let r3c6c = self.bus.read_reg(regs::MAC_INIT_REG_3C6C);
        self.bus
            .write_reg(regs::MAC_INIT_REG_3C6C, r3c6c | 0x8080_2100);

        let mut pol_slot = 0;
        while pol_slot < regs::RX_POLICY_SLOT_COUNT {
            let p_reg = regs::rx_policy_reg(pol_slot);
            let r_pol = self.bus.read_reg(p_reg);
            self.bus.write_reg(p_reg, (r_pol & !0x20) | 0x40);
            pol_slot += 1;
        }

        let r3c74 = self.bus.read_reg(regs::MAC_INIT_REG_3C74);
        self.bus.write_reg(regs::MAC_INIT_REG_3C74, r3c74 | 8);

        let r30f8 = self.bus.read_reg(regs::MAC_RX_REG_30F8);
        self.bus.write_reg(regs::MAC_RX_REG_30F8, r30f8 & 0xFFFF);

        let r30fc = self.bus.read_reg(regs::MAC_RX_REG_30FC);
        self.bus.write_reg(regs::MAC_RX_REG_30FC, r30fc & 0xFFFF);

        let r3100 = self.bus.read_reg(regs::MAC_RX_REG_3100);
        self.bus.write_reg(regs::MAC_RX_REG_3100, r3100 & 0xFFFF);

        let r3104 = self.bus.read_reg(regs::MAC_RX_REG_3104);
        self.bus.write_reg(regs::MAC_RX_REG_3104, r3104 & 0xFFFF);

        let r30f8 = self.bus.read_reg(regs::MAC_RX_REG_30F8);
        self.bus
            .write_reg(regs::MAC_RX_REG_30F8, r30f8 | 0x0500_0000);

        let r30fc = self.bus.read_reg(regs::MAC_RX_REG_30FC);
        self.bus
            .write_reg(regs::MAC_RX_REG_30FC, r30fc | 0x0500_0000);

        let r310c = self.bus.read_reg(regs::MAC_RX_REG_310C);
        self.bus.write_reg(regs::MAC_RX_REG_310C, r310c & !0xF0);

        let r3114 = self.bus.read_reg(regs::MAC_RX_REG_3114);
        self.bus
            .write_reg(regs::MAC_RX_REG_3114, (r3114 & 0xF00F_FFFF) | 0x81B0_0000);

        let r3c78 = self.bus.read_reg(regs::MAC_INIT_REG_3C78);
        self.bus.write_reg(regs::MAC_INIT_REG_3C78, r3c78 | 3);

        let r3d78 = self.bus.read_reg(regs::MAC_INIT_REG_3D78);
        self.bus.write_reg(regs::MAC_INIT_REG_3D78, r3d78 & !0xF0);

        let r3c54 = self.bus.read_reg(regs::MAC_INIT_REG_3C54);
        self.bus
            .write_reg(regs::MAC_INIT_REG_3C54, (r3c54 & 0xFFFF_0000) | 0x0404);

        let r3c1c = self.bus.read_reg(regs::MAC_INIT_REG_3C1C);
        self.bus
            .write_reg(regs::MAC_INIT_REG_3C1C, (r3c1c & 0xFFFF_F000) | 0xC000_00F0);

        let r3c20 = self.bus.read_reg(regs::MAC_INIT_REG_3C20);
        self.bus
            .write_reg(regs::MAC_INIT_REG_3C20, (r3c20 & 0xFFFF_F000) | 0xF0);

        let r3c24 = self.bus.read_reg(regs::MAC_INIT_REG_3C24);
        self.bus
            .write_reg(regs::MAC_INIT_REG_3C24, (r3c24 & 0xFFFF_F000) | 0xF0);

        let r3cac = self.bus.read_reg(regs::MAC_INIT_REG_3CAC);
        self.bus
            .write_reg(regs::MAC_INIT_REG_3CAC, (r3cac & !0xF0) | 0x40);

        let r3c5c = self.bus.read_reg(regs::MAC_INIT_REG_3C5C);
        self.bus
            .write_reg(regs::MAC_INIT_REG_3C5C, r3c5c | 0xFFFF_0000);

        let r3c88 = self.bus.read_reg(regs::MAC_INIT_REG_3C88);
        self.bus
            .write_reg(regs::MAC_INIT_REG_3C88, r3c88 & 0xF0FF_FFFF);

        let r3288 = self.bus.read_reg(regs::MAC_RX_REG_3288);
        self.bus.write_reg(regs::MAC_RX_REG_3288, r3288 | 2);

        crate::phy_bridge::with_mmio_rmw_lock(|| {
            let r_rx_ctrl = self.bus.read_reg(regs::RX_CTRL_REG);
            self.bus
                .write_reg(regs::RX_CTRL_REG, r_rx_ctrl & 0x7FFF_FFFF);
            self.bus.mem_barrier();
        });

        let mut slot = 0;
        while slot < regs::RX_POLICY_SLOT_COUNT {
            let p_reg = regs::rx_policy_reg(slot);
            let val = self.bus.read_reg(p_reg);
            self.bus.write_reg(p_reg, (val | 5) & 0xFFFF_F6FF);
            slot += 1;
        }

        // RX SRAM window & ring
        self.bus.write_reg(regs::MAC_RX_REG_311C, 0x3FFA_E000);
        self.bus.write_reg(regs::MAC_RX_REG_3124, 0x3FFA_E000);
        self.bus.write_reg(regs::MAC_RX_REG_3118, 0x400A_0000);
        self.bus.write_reg(regs::MAC_RX_REG_3120, 0x400A_0000);
        self.bus
            .write_reg(regs::RX_BASE_DSCR_REG, rx_ring_base_addr);
        self.reload_rx_dma();

        let r3080 = self.bus.read_reg(regs::MAC_RX_REG_3080);
        self.bus
            .write_reg(regs::MAC_RX_REG_3080, r3080 & 0xDFFF_FF00);

        self.bus.write_reg(regs::MAC_RX_REG_314C, 0x3000);
        self.bus.write_reg(regs::MAC_RX_REG_3158, 0);
        self.bus.write_reg(regs::MAC_RX_REG_3164, 0x000C);

        let r3148 = self.bus.read_reg(regs::MAC_RX_REG_3148);
        self.bus.write_reg(regs::MAC_RX_REG_3148, r3148 | 0x0900);

        let r309c = self.bus.read_reg(regs::MAC_RX_REG_309C);
        self.bus
            .write_reg(regs::MAC_RX_REG_309C, r309c | 0x0800_0000);

        // 4. hal_mac_rate_autoack_init
        self.bus.write_reg(regs::AUTOACK_RATE_INIT_REGS[1], 0);
        self.bus.write_reg(regs::AUTOACK_RATE_INIT_REGS[0], 0);
        self.bus
            .write_reg(regs::AUTOACK_RATE_INIT_REGS[3], 0x0302_0100);
        self.bus
            .write_reg(regs::AUTOACK_RATE_INIT_REGS[2], 0x0706_0500);
        self.bus
            .write_reg(regs::AUTOACK_RATE_INIT_REGS[5], 0x0B0B_0B0B);
        self.bus
            .write_reg(regs::AUTOACK_RATE_INIT_REGS[4], 0x0B0B_0B0B);
        self.bus
            .write_reg(regs::AUTOACK_RATE_INIT_REGS[7], 0x090B_0B0B);
        self.bus
            .write_reg(regs::AUTOACK_RATE_INIT_REGS[6], 0x0909_0909);
        self.disable_low_rate();

        // 5. hal_attenna_init
        let r_plcp2 = self.bus.read_reg(regs::TX_PLCP2_REG);
        self.bus
            .write_reg(regs::TX_PLCP2_REG, (r_plcp2 & !0x1F) | 0x20);

        let r3284 = self.bus.read_reg(regs::MAC_RX_REG_3284);
        self.bus
            .write_reg(regs::MAC_RX_REG_3284, (r3284 & !0x07) | 0x20);

        // 6. hal_crypto_init (pure-Rust CCMP passthrough: 0x30000 bypass on
        // 0x3FF7_3800..=0x3FF7_3804; 0 on 0x3FF7_3808..=0x3FF7_3814)
        let mut crypto_addr = regs::HW_CRYPTO_DISABLE_START;
        while crypto_addr <= regs::HW_CRYPTO_DISABLE_END {
            let val = if crypto_addr <= regs::HW_CRYPTO_DISABLE_START + 4 {
                0x0003_0000
            } else {
                0
            };
            self.bus.write_reg(crypto_addr, val);
            crypto_addr += 4;
        }

        let mut key_slot = 0;
        while key_slot < regs::KEY_SLOT_COUNT {
            let slot_base = regs::KEY_TABLE_BASE_REG + (key_slot as u32) * regs::KEY_SLOT_STRIDE;
            let mut word_idx = 0;
            while word_idx < 10 {
                self.bus.write_reg(slot_base + word_idx * 4, 0);
                word_idx += 1;
            }
            key_slot += 1;
        }

        // 7. Clear BSSID filter slots 0..=2
        let mut bssid_idx = 0;
        while bssid_idx < regs::BSSID_FILTER_SLOT_COUNT {
            let ctrl = regs::bssid_mask_high_ctrl_reg(bssid_idx);
            let val = self.bus.read_reg(ctrl);
            self.bus.write_reg(ctrl, val & !regs::BSSID_BIT_ENABLE);
            bssid_idx += 1;
        }

        // 8. Program STA MAC address in hardware slot 0
        let mac_low = u32::from_le_bytes([sta_mac[0], sta_mac[1], sta_mac[2], sta_mac[3]]);
        let mac_high = u32::from(sta_mac[4]) | (u32::from(sta_mac[5]) << 8);
        self.bus.write_reg(regs::sta_mac_low_reg(0), mac_low);
        self.bus.write_reg(regs::sta_mac_high_reg(0), mac_high);
        self.bus
            .write_reg(regs::sta_mac_mask_low_reg(0), 0xFFFF_FFFF);
        self.bus
            .write_reg(regs::sta_mac_mask_high_ctrl_reg(0), 0x0001_FFFF);

        // 9. Arm RX DMA BEFORE enabling operational interrupts (M-165).
        let r3c04 = self.bus.read_reg(regs::MAC_CTRL_REG_3C04);
        self.bus.write_reg(
            regs::MAC_CTRL_REG_3C04,
            r3c04 & !regs::MAC_CTRL_TRANSCEIVER_DISABLE_MASK,
        );
        self.bus.mem_barrier();

        let r309c_arm = self.bus.read_reg(regs::MAC_RX_REG_309C);
        self.bus
            .write_reg(regs::MAC_RX_REG_309C, (r309c_arm & !0xFF) | 2);

        crate::phy_bridge::with_mmio_rmw_lock(|| {
            let rx_ctrl_arm = self.bus.read_reg(regs::RX_CTRL_REG);
            self.bus
                .write_reg(regs::RX_CTRL_REG, rx_ctrl_arm | regs::RX_CTRL_BIT_ENABLE);
            self.bus.mem_barrier();
        });

        // Clear stale interrupts after RX DMA arming, then enable IRQs last.
        self.bus.write_reg(regs::INT_CLEAR_REG, 0xFFFF_FFFF);
        self.bus.mem_barrier();
        let _ = self.bus.read_reg(regs::INT_STATUS_REG);
        self.bus
            .write_reg(regs::INT_ENABLE_REG, regs::INT_ENABLE_OPERATIONAL);
        self.bus.mem_barrier();

        self.mac_initialized = true;
        Ok(())
    }

    /// Deinitializes Wi-Fi MAC cleanly: masks and clears interrupts, halts RX
    /// DMA and aborts pending TX triggers, drains active TX/RX DMA while clocks
    /// remain active, disables the transceiver, and gates MAC clocks last.
    pub fn deinit_mac(&mut self) {
        // 1. Mask primary/secondary interrupts and clear pending latches.
        self.bus.write_reg(regs::INT_ENABLE_REG, 0);
        self.bus.write_reg(regs::INT_ENABLE_SEC_REG, 0);
        self.bus.write_reg(regs::INT_CLEAR_REG, 0xFFFF_FFFF);
        self.bus.mem_barrier();
        let _ = self.bus.read_reg(regs::INT_STATUS_REG);

        // 2. Halt RX DMA and abort armed TX trigger while clocks stay active.
        crate::phy_bridge::with_mmio_rmw_lock(|| {
            let r_rx = self.bus.read_reg(regs::RX_CTRL_REG);
            self.bus
                .write_reg(regs::RX_CTRL_REG, r_rx & !regs::RX_CTRL_BIT_ENABLE);
            self.bus.mem_barrier();
        });

        let r_trig = self.bus.read_reg(regs::TX_PLCP0_TRIG_REG);
        self.bus.write_reg(
            regs::TX_PLCP0_TRIG_REG,
            r_trig & !regs::TX_PLCP0_TRIGGER_BITS,
        );
        self.bus.mem_barrier();

        // 3. Drain active TX/RX DMA before gating clocks.
        self.bus.delay_us(32);

        let mut busy = true;
        let mut poll = 0;
        while poll < MAX_TX_DRAIN_POLLS {
            let clk_stat = self.bus.read_reg(regs::MAC_CLK_RST_REG);
            if (clk_stat & regs::MAC_CLK_RST_BIT_TX_ACTIVE) == 0 {
                busy = false;
                break;
            }
            self.bus.delay_us(1);
            poll += 1;
        }

        if busy {
            // Pulse TX/MAC abort reset while clocks are still ungated.
            let r3c58 = self.bus.read_reg(regs::MAC_INIT_REG_3C58);
            self.bus
                .write_reg(regs::MAC_INIT_REG_3C58, (r3c58 & !0x0003_0000) | (2 << 16));
            self.bus.mem_barrier();
            self.bus.delay_us(5);
            let r3c58_clr = self.bus.read_reg(regs::MAC_INIT_REG_3C58);
            self.bus
                .write_reg(regs::MAC_INIT_REG_3C58, r3c58_clr & !0x0003_0000);
            self.bus.mem_barrier();
        }

        // 4. Clear any late-arrived TX/IRQ status bits.
        let r_err_clr = self.bus.read_reg(regs::TX_ERR_CLEAR_REG);
        self.bus.write_reg(
            regs::TX_ERR_CLEAR_REG,
            (r_err_clr & 0xF800_F800) | 0x0001_0001,
        );
        self.bus.write_reg(regs::TX_DONE_CLEAR_REG, 1);
        self.bus.write_reg(regs::INT_CLEAR_REG, 0xFFFF_FFFF);
        self.bus.mem_barrier();

        // 5. Disable transceiver.
        let r3c04 = self.bus.read_reg(regs::MAC_CTRL_REG_3C04);
        self.bus.write_reg(
            regs::MAC_CTRL_REG_3C04,
            r3c04 | regs::MAC_CTRL_TRANSCEIVER_DISABLE_MASK,
        );
        self.bus.mem_barrier();

        // 6. Gate MAC clocks LAST after DMA is quiescent.
        let r_clk = self.bus.read_reg(regs::MAC_CLK_RST_REG);
        self.bus
            .write_reg(regs::MAC_CLK_RST_REG, r_clk | regs::MAC_CLK_RST_MASK_UNGATE);
        self.bus.mem_barrier();

        self.mac_initialized = false;
        self.vif0_crypto_ctrl = 0x0003_0000;
    }

    /// Configures RX policy for passive scanning (promiscuous BSSID bypass).
    ///
    /// Vendor `hal_mac_rx_set_policy` only toggles `0x110` in the policy
    /// register; bit 1 is never written and is preserved as found.
    pub fn set_scan_rx_policy(&mut self) {
        let ctrl_reg = regs::bssid_mask_high_ctrl_reg(0);
        let val = self.bus.read_reg(ctrl_reg);
        self.bus.write_reg(ctrl_reg, val & !regs::BSSID_BIT_ENABLE);

        let pol_reg = regs::RX_POLICY_BASE_REG;
        let pol = self.bus.read_reg(pol_reg);
        self.bus
            .write_reg(pol_reg, pol | regs::RX_POLICY_PROMISC_BYPASS);
    }

    /// Sets the hardware BSSID filter address in slot 0 and enforces
    /// BSSID checking through the BSSID mask enable bit (bit 16 of
    /// `0x3FF7_3024`) and by clearing the promiscuous bypass (`0x110`).
    pub fn set_associated_bssid(&mut self, bssid: [u8; 6]) {
        let ctrl_reg = regs::bssid_mask_high_ctrl_reg(0);
        let ctrl = self.bus.read_reg(ctrl_reg);
        self.bus.write_reg(ctrl_reg, ctrl & !regs::BSSID_BIT_ENABLE);

        let low = u32::from_le_bytes([bssid[0], bssid[1], bssid[2], bssid[3]]);
        self.bus.write_reg(regs::bssid_filter_low_reg(0), low);

        let high_cur = self.bus.read_reg(regs::bssid_filter_high_reg(0));
        let high_val = (high_cur & 0xFFFF_0000) | u32::from(bssid[4]) | (u32::from(bssid[5]) << 8);
        self.bus.write_reg(regs::bssid_filter_high_reg(0), high_val);

        self.bus.write_reg(regs::bssid_mask_low_reg(0), 0xFFFF_FFFF);
        self.bus
            .write_reg(regs::bssid_mask_high_ctrl_reg(0), 0x0001_FFFF);

        let pol_reg = regs::RX_POLICY_BASE_REG;
        let pol = self.bus.read_reg(pol_reg);
        self.bus
            .write_reg(pol_reg, pol & !regs::RX_POLICY_PROMISC_BYPASS);
    }

    /// Programs a hardware CCMP key into MAC key table `slot` (`0..=3` for GTK,
    /// `4` for STA PTK) and enables STA VIF 0 hardware CCMP RX decryption.
    pub fn install_sta_ccmp_key(&mut self, slot: u8, key_idx: u8, bssid: [u8; 6], key: &[u8; 16]) {
        if usize::from(slot) >= regs::KEY_SLOT_COUNT {
            return;
        }
        let slot_base = regs::KEY_TABLE_BASE_REG + u32::from(slot) * regs::KEY_SLOT_STRIDE;
        let word0 = u32::from_le_bytes([bssid[0], bssid[1], bssid[2], bssid[3]]);
        let mac_hi = u32::from(u16::from_le_bytes([bssid[4], bssid[5]]));
        let alg_bits = 3u32 << 18;
        let mode_bits = if slot < 4 { 6u32 << 21 } else { 3u32 << 21 };
        let key_id_bits = u32::from(key_idx & 0x03) << 30;
        let word1 = mac_hi | alg_bits | mode_bits | key_id_bits;

        self.bus.write_reg(slot_base, word0);
        self.bus.write_reg(slot_base + 4, word1);

        let slot_mask = self.bus.read_reg(regs::HW_CRYPTO_DISABLE_END);
        self.bus.write_reg(
            regs::HW_CRYPTO_DISABLE_END,
            slot_mask | (1u32 << u32::from(slot)),
        );

        let mut w = 0usize;
        while w < 4 {
            let base = w * 4;
            let key_word =
                u32::from_le_bytes([key[base], key[base + 1], key[base + 2], key[base + 3]]);
            self.bus.write_reg(slot_base + 8 + (w as u32) * 4, key_word);
            w += 1;
        }

        let mut offset = 24u32;
        while offset < 40 {
            self.bus.write_reg(slot_base + offset, 0);
            offset += 4;
        }

        let r3810_addr = regs::HW_CRYPTO_DISABLE_START + 0x10;
        let r3810 = self.bus.read_reg(r3810_addr);
        self.bus.write_reg(r3810_addr, r3810 & 0xFF00_00FF);

        self.vif0_crypto_ctrl = 0x0003_0103;
        self.bus
            .write_reg(regs::HW_CRYPTO_DISABLE_START, self.vif0_crypto_ctrl);
        self.bus.mem_barrier();
    }

    /// Clears STA VIF 0 hardware CCMP key slots (`0..=4`) and restores
    /// software-crypto passthrough (`0x0003_0000`).
    pub fn clear_sta_ccmp_keys(&mut self) {
        self.vif0_crypto_ctrl = 0x0003_0000;
        self.bus
            .write_reg(regs::HW_CRYPTO_DISABLE_START, 0x0003_0000);

        let slot_mask = self.bus.read_reg(regs::HW_CRYPTO_DISABLE_END);
        self.bus
            .write_reg(regs::HW_CRYPTO_DISABLE_END, slot_mask & !0x0000_001F);

        let mut slot = 0u32;
        while slot <= 4 {
            let slot_base = regs::KEY_TABLE_BASE_REG + slot * regs::KEY_SLOT_STRIDE;
            let mut word_idx = 0u32;
            while word_idx < 10 {
                self.bus.write_reg(slot_base + word_idx * 4, 0);
                word_idx += 1;
            }
            slot += 1;
        }

        self.bus.mem_barrier();
    }

    /// Reads and validates the factory MAC address from ESP32 eFuse Block 0,
    /// verifying the ROM CRC-8 byte (`EFUSE_BLK0_RDATA2_REG[23:16]`),
    /// non-zero/non-broadcast, and unicast bit (`mac[0] & 1 == 0`) (M-220).
    pub fn try_read_efuse_sta_mac(&self) -> Result<[u8; 6], LmacError> {
        let w1 = self.bus.read_reg(regs::EFUSE_BLK0_RDATA1_REG);
        let w2 = self.bus.read_reg(regs::EFUSE_BLK0_RDATA2_REG);
        let mac = [
            (w2 >> 8) as u8,
            w2 as u8,
            (w1 >> 24) as u8,
            (w1 >> 16) as u8,
            (w1 >> 8) as u8,
            w1 as u8,
        ];
        let stored_crc = ((w2 >> 16) & 0xFF) as u8;
        let expected_crc = efuse_mac_crc8(&mac);
        if stored_crc != expected_crc {
            return Err(LmacError::InvalidEfuseMac);
        }
        if mac == [0u8; 6] || mac == [0xFFu8; 6] || (mac[0] & 0x01) != 0 {
            return Err(LmacError::InvalidEfuseMac);
        }
        Ok(mac)
    }

    /// Reads the validated factory MAC address from ESP32 eFuse Block 0,
    /// returning `[0; 6]` if eFuse CRC-8 or unicast validation fails.
    ///
    /// Prefer [`Self::try_read_efuse_sta_mac`] for explicit error handling.
    #[must_use]
    pub fn read_efuse_sta_mac(&self) -> [u8; 6] {
        self.try_read_efuse_sta_mac().unwrap_or_default()
    }

    /// Reads a conditioned 32-bit hardware random number from `WDEV_RND_REG`
    /// with active RF/MAC init gating, inter-read cycle spacing, stuck-at
    /// health check, and SHA-256 entropy pool conditioning (M-015).
    pub fn try_hw_random_u32(&mut self) -> Result<u32, LmacError> {
        if !self.mac_initialized {
            return Err(LmacError::RngNotReady);
        }
        #[cfg(target_arch = "xtensa")]
        if !crate::phy_bridge::is_phy_session_active() {
            return Err(LmacError::RngNotReady);
        }

        // Sample 4 hardware RNG words with >= 2 us (>= 160 APB cycles)
        // inter-read spacing per ESP32 TRM requirements.
        let mut samples = [0u32; 4];
        let mut idx = 0;
        while idx < 4 {
            if idx > 0 {
                self.bus.delay_us(2);
            }
            samples[idx] = self.bus.read_reg(regs::WDEV_RND_REG);
            idx += 1;
        }

        // Stuck-at / repetition health check: reject if all 4 raw samples match.
        let all_equal =
            samples[0] == samples[1] && samples[1] == samples[2] && samples[2] == samples[3];
        if all_equal {
            return Err(LmacError::RngHealthCheckFailed);
        }

        let prev = self.last_raw_rng;
        self.last_raw_rng = samples[3];
        self.rng_counter = self.rng_counter.wrapping_add(1);

        let ctr_lo = self.rng_counter as u32;
        let ctr_hi = (self.rng_counter >> 32) as u32;
        let block = [
            samples[0],
            samples[1],
            samples[2],
            samples[3],
            ctr_lo,
            ctr_hi,
            prev,
            self.rng_pool[0] ^ samples[0],
            self.rng_pool[1] ^ samples[1],
            self.rng_pool[2] ^ samples[2],
            self.rng_pool[3] ^ samples[3],
            self.rng_pool[4] ^ ctr_lo,
            self.rng_pool[5] ^ ctr_hi,
            self.rng_pool[6] ^ 0x8000_0000,
            self.rng_pool[7],
            0x0000_0100,
        ];
        sha256_compress_pool(&mut self.rng_pool, &block);

        Ok(self.rng_pool[0] ^ self.rng_pool[4])
    }

    /// Disables 802.11b low-rate transmissions in MAC rate control tables.
    pub fn disable_low_rate(&mut self) {
        self.bus
            .write_reg(regs::AUTOACK_RATE_INIT_REGS[7], 0x090B_0B0B);
        self.bus
            .write_reg(regs::AUTOACK_RATE_INIT_REGS[6], 0x0909_0909);

        // M-167: Barrier across PeriBus1 (0x3FF7_*) -> PeriBus2 (0x6001_*).
        self.bus.mem_barrier();

        let r6860 = self.bus.read_reg(regs::MAC_LOW_RATE_DISABLE_REG_6860);
        self.bus
            .write_reg(regs::MAC_LOW_RATE_DISABLE_REG_6860, r6860 & 0xFFFF_F3FF);

        let r687c = self.bus.read_reg(regs::MAC_LOW_RATE_DISABLE_REG_687C);
        self.bus
            .write_reg(regs::MAC_LOW_RATE_DISABLE_REG_687C, r687c & 0xFFFF_F7FF);
        self.bus.mem_barrier();
    }

    /// Enables 802.11b 1/2 Mbps CCK low-rate transmissions in MAC tables.
    pub fn enable_low_rate(&mut self) {
        self.bus
            .write_reg(regs::AUTOACK_RATE_INIT_REGS[7], 0x0B0B_0B0B);
        self.bus
            .write_reg(regs::AUTOACK_RATE_INIT_REGS[6], 0x0B0B_0B0B);

        // M-167: Barrier across PeriBus1 (0x3FF7_*) -> PeriBus2 (0x6001_*).
        self.bus.mem_barrier();

        let r6860 = self.bus.read_reg(regs::MAC_LOW_RATE_DISABLE_REG_6860);
        self.bus
            .write_reg(regs::MAC_LOW_RATE_DISABLE_REG_6860, r6860 | 0x0000_0C00);

        let r687c = self.bus.read_reg(regs::MAC_LOW_RATE_DISABLE_REG_687C);
        self.bus
            .write_reg(regs::MAC_LOW_RATE_DISABLE_REG_687C, r687c | 0x0000_0800);
        self.bus.mem_barrier();
    }

    /// Reads the interrupt status register, clears only handled asserted
    /// interrupts (W1C) followed by a memory barrier and APB read-back, and
    /// returns decoded [`LmacIrqEvents`] including any unhandled raw bits.
    pub fn poll_and_clear_irqs(&mut self) -> LmacIrqEvents {
        let status = self.bus.read_reg(regs::INT_STATUS_REG);
        let handled = status & regs::INT_HANDLED_MASK;
        let unhandled = status & !regs::INT_HANDLED_MASK;

        if handled != 0 {
            self.bus.write_reg(regs::INT_CLEAR_REG, handled);
            self.bus.mem_barrier();
            // Dummy APB read-back ensures the W1C write commits before return.
            let _ = self.bus.read_reg(regs::INT_STATUS_REG);
        }

        LmacIrqEvents {
            rx_ready: (status & regs::INT_RX_MASK) != 0,
            tx_done: (status & regs::INT_TX_DONE) != 0,
            tx_collision: (status & regs::INT_TX_COLL) != 0,
            tx_timeout: (status & regs::INT_TX_TIMEOUT) != 0,
            panic_wdt: (status & regs::INT_PANIC_WDT) != 0,
            raw_unhandled: unhandled,
        }
    }

    /// Signals the RX DMA controller to reload descriptor ring pointers under
    /// the crate-wide MMIO RMW spinlock (M-166 / M-072) and bounded-polls until
    /// hardware auto-clears `RX_CTRL_BIT_RELOAD`.
    pub fn reload_rx_dma(&mut self) {
        crate::phy_bridge::with_mmio_rmw_lock(|| {
            let r = self.bus.read_reg(regs::RX_CTRL_REG);
            self.bus
                .write_reg(regs::RX_CTRL_REG, r | regs::RX_CTRL_BIT_RELOAD);
            self.bus.mem_barrier();
        });
        let mut poll = 0;
        while poll < MAX_RX_RELOAD_POLLS {
            if (self.bus.read_reg(regs::RX_CTRL_REG) & regs::RX_CTRL_BIT_RELOAD) == 0 {
                break;
            }
            core::hint::spin_loop();
            poll += 1;
        }
    }

    /// Returns the raw hardware RX end-state byte from register 0x30A4.
    #[must_use]
    pub fn rx_end_state(&self) -> u8 {
        (self.bus.read_reg(regs::MAC_RX_REG_30A4) & 0xFF) as u8
    }

    /// Aborts TX slot 0 trigger, drains `TX_ACTIVE == 0` (pulsing the
    /// `MAC_INIT_REG_3C58` abort reset if `TX_ACTIVE` remains high after
    /// `MAX_TX_DRAIN_POLLS`), re-verifies `TX_ACTIVE == 0`, clears any late
    /// slot-0 completion/error latches, and returns `true` iff TX drained
    /// cleanly (`M-074`).
    fn abort_and_drain_tx(&mut self) -> bool {
        let r_trig = self.bus.read_reg(regs::TX_PLCP0_TRIG_REG);
        self.bus.write_reg(
            regs::TX_PLCP0_TRIG_REG,
            r_trig & !regs::TX_PLCP0_TRIGGER_BITS,
        );
        self.bus.mem_barrier();

        let mut drained = false;
        let mut drain_poll = 0;
        while drain_poll < MAX_TX_DRAIN_POLLS {
            if (self.bus.read_reg(regs::MAC_CLK_RST_REG) & regs::MAC_CLK_RST_BIT_TX_ACTIVE) == 0 {
                drained = true;
                break;
            }
            self.bus.delay_us(1);
            drain_poll += 1;
        }

        if !drained {
            // Pulse TX/MAC abort reset (matching `deinit_mac`) and re-verify
            // `TX_ACTIVE == 0` before reporting drain success.
            let r3c58 = self.bus.read_reg(regs::MAC_INIT_REG_3C58);
            self.bus
                .write_reg(regs::MAC_INIT_REG_3C58, (r3c58 & !0x0003_0000) | (2 << 16));
            self.bus.mem_barrier();
            self.bus.delay_us(5);
            let r3c58_clr = self.bus.read_reg(regs::MAC_INIT_REG_3C58);
            self.bus
                .write_reg(regs::MAC_INIT_REG_3C58, r3c58_clr & !0x0003_0000);
            self.bus.mem_barrier();

            drained =
                (self.bus.read_reg(regs::MAC_CLK_RST_REG) & regs::MAC_CLK_RST_BIT_TX_ACTIVE) == 0;
        }

        let r_err_clr = self.bus.read_reg(regs::TX_ERR_CLEAR_REG);
        self.bus.write_reg(
            regs::TX_ERR_CLEAR_REG,
            (r_err_clr & 0xF800_F800) | 0x0001_0001,
        );
        self.bus.write_reg(regs::TX_DONE_CLEAR_REG, 1);
        self.bus.mem_barrier();
        drained
    }

    /// Submits a frame for transmission and polls until completion, error,
    /// or timeout.
    pub fn submit_and_poll_tx(
        &mut self,
        desc_addr: u32,
        mpdu: &[u8],
        rate: WifiPhyRate,
        dur_us: u16,
        wait_ack: bool,
        max_polls: u32,
    ) -> Result<(), LmacError> {
        let mpdu_len = mpdu.len();
        if mpdu_len == 0 || mpdu_len > 1600 {
            return Err(LmacError::InvalidPayloadLength);
        }
        // M-167: Reject 0 or unbounded poll counts before touching hardware.
        if max_polls == 0 || max_polls > MAX_TX_POLLS {
            return Err(LmacError::InvalidPollCount);
        }
        // M-220: Reject provisional SGI rates until TX_HTSIG1_SHORT_GI verified.
        if rate.is_sgi() {
            return Err(LmacError::UnsupportedRate);
        }
        // M-073: Validate alignment and ESP32 internal DRAM bounds on Xtensa.
        validate_dma_desc_addr(desc_addr)?;

        // 1. Program EDCA transmission timeout via RMW preserving upper bits.
        let r_edca = self.bus.read_reg(regs::TX_TIMEOUT_EDCA_REG);
        self.bus.write_reg(
            regs::TX_TIMEOUT_EDCA_REG,
            (r_edca & 0xF000_0000) | (10 & 0xFFF) | ((0x0F & 0x3FF) << 12) | ((7 & 0x0F) << 24),
        );

        // 2. Program PLCP1 parameters.
        // Hardware TX_PLCP1_REG bits [11:0] require the PSDU length, which
        // includes the 4-byte hardware-appended 802.11 FCS (MPDU length + 4).
        let psdu_len = (mpdu_len as u32).saturating_add(4);
        let is_ht_u32 = if rate.is_ht() { 1u32 } else { 0u32 };
        // Vendor `ieee80211_ccmp_encrypt` sets `tx_info[16] = 0xC0` when FC[1]
        // Protected (0x40) is set, and `mac_tx_set_plcp1` shifts `(u32)tx_info[16] << 17`
        // into `TX_PLCP1_REG[24:17]` (`0x0180_0000`) so hardware skips TX encryption
        // without toggling `HW_CRYPTO_DISABLE_START` (preserving RX HW decryption).
        let sw_crypto_bypass = if mpdu.len() >= 2 && (mpdu[1] & 0x40) != 0 {
            TX_PLCP1_SW_CRYPTO_BYPASS_BITS
        } else {
            0
        };
        let plcp1 = ((is_ht_u32 << 9) << 16)
            | sw_crypto_bypass
            | (u32::from(rate.hw_code() & 0x1F) << 12)
            | (psdu_len & 0xFFF)
            | regs::TX_PLCP1_BANDWIDTH_BIT;
        self.bus.write_reg(regs::TX_PLCP1_REG, plcp1);

        // 3. Program PLCP2 antenna via RMW preserving upper bits (M-167).
        let r_plcp2 = self.bus.read_reg(regs::TX_PLCP2_REG);
        self.bus
            .write_reg(regs::TX_PLCP2_REG, (r_plcp2 & !0x1F) | 0x20);

        // 4. Program transmission duration
        self.bus.write_reg(
            regs::TX_DURATION_REG,
            (u32::from(dur_us) << 16) | u32::from(dur_us),
        );

        // 5. If 802.11n HT (LGI) rate, program HT-SIG1 and HT-SIG2 (vendor
        // `mac_tx_set_htsig` parity). SGI rates are rejected above.
        if rate.is_ht() {
            let htsig1 = regs::TX_HTSIG1_FIXED
                | ((psdu_len & 0xFFFF) << 8)
                | u32::from(rate.hw_code() & 0x07);
            self.bus.write_reg(regs::TX_HTSIG1_REG, htsig1);

            let htsig2 = (psdu_len & 0xFFFF) | regs::TX_HTSIG2_FIXED;
            self.bus.write_reg(regs::TX_HTSIG2_REG, htsig2);
        }

        // 6. Clear stale slot-0 TX_ERR/TX_DONE before arming trigger (M-074).
        let r_err_clr = self.bus.read_reg(regs::TX_ERR_CLEAR_REG);
        self.bus.write_reg(
            regs::TX_ERR_CLEAR_REG,
            (r_err_clr & 0xF800_F800) | 0x0001_0001,
        );
        self.bus.write_reg(regs::TX_DONE_CLEAR_REG, 1);
        self.bus.mem_barrier();

        // 7. Build PLCP0 base command word
        let mode = if rate.is_ht() {
            regs::TX_PLCP0_MODE_HT
        } else {
            regs::TX_PLCP0_MODE_LEGACY
        };
        let ack = if wait_ack {
            regs::TX_PLCP0_WAIT_ACK
        } else {
            regs::TX_PLCP0_NO_ACK
        };
        let base = (desc_addr & 0x000F_FFFF) | mode | ack;

        // Write descriptor base address, barrier, then assert trigger bits
        self.bus.write_reg(regs::TX_PLCP0_TRIG_REG, base);
        self.bus.mem_barrier();
        self.bus
            .write_reg(regs::TX_PLCP0_TRIG_REG, base | regs::TX_PLCP0_TRIGGER_BITS);
        self.bus.mem_barrier();

        // 8. Poll for completion, error, or loop exhaustion
        self.poll_tx_completion(max_polls)
    }

    fn poll_tx_completion(&mut self, max_polls: u32) -> Result<(), LmacError> {
        let mut poll = 0;
        while poll < max_polls {
            let err_status = self.bus.read_reg(regs::TX_ERR_STATUS_REG);
            if (err_status & 0x0001_0001) != 0 {
                let r_err_clr = self.bus.read_reg(regs::TX_ERR_CLEAR_REG);
                self.bus.write_reg(
                    regs::TX_ERR_CLEAR_REG,
                    (r_err_clr & 0xF800_F800) | (err_status & 0x0001_0001),
                );
                if !self.abort_and_drain_tx() {
                    return Err(LmacError::TxDrainFailed);
                }
                return Err(LmacError::TxHardwareError);
            }

            let done_status = self.bus.read_reg(regs::TX_DONE_STATUS_REG);
            if (done_status & 1) != 0 {
                let pmd = self.bus.read_reg(regs::TX_PMD_REG);
                let status = (pmd >> 12) & 0x0F;
                // Write only slot-0 bit 0 (avoid clobbering other slots' W1C).
                self.bus.write_reg(regs::TX_DONE_CLEAR_REG, 1);
                self.bus.write_reg(regs::TX_PLCP0_TRIG_REG, 0);
                self.bus.mem_barrier();
                if status == 0 {
                    return Ok(());
                }
                return Err(LmacError::TxAckFailed(status));
            }

            self.bus.delay_us(1);
            poll += 1;
        }

        // Timeout: abort trigger, barrier, drain TX_ACTIVE == 0 (pulsing
        // MAC_INIT_REG_3C58 reset if needed), clear latches.
        if !self.abort_and_drain_tx() {
            return Err(LmacError::TxDrainFailed);
        }
        Err(LmacError::TxTimeout)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_init_mac_success() {
        let mut mock = MockWifiMmioBus::new();
        mock.auto_init_3d24 = true;
        mock.set_reg(regs::MAC_CTRL_REG_3C04, 0xFF);
        let mut hal = LmacHal::new(mock);

        let mac = [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC];
        let ring_addr = 0x3FFE_0000;
        let res = hal.init_mac(mac, ring_addr);
        assert_eq!(res, Ok(()));
        assert!(hal.is_initialized());

        let bus = hal.bus();
        assert_eq!(bus.reg(regs::sta_mac_low_reg(0)), 0x7856_3412);
        assert_eq!(bus.reg(regs::sta_mac_high_reg(0)), 0xBC9A);
        assert_eq!(bus.reg(regs::sta_mac_mask_low_reg(0)), 0xFFFF_FFFF);
        assert_eq!(bus.reg(regs::sta_mac_mask_high_ctrl_reg(0)), 0x0001_FFFF);

        assert_eq!(bus.reg(regs::RX_BASE_DSCR_REG), ring_addr);
        assert!(bus.rx_ctrl_reload_count >= 1);
        assert_eq!(bus.reg(regs::RX_CTRL_REG) & regs::RX_CTRL_BIT_RELOAD, 0);
        assert_ne!(bus.reg(regs::RX_CTRL_REG) & regs::RX_CTRL_BIT_ENABLE, 0);
        assert_eq!(bus.reg(regs::INT_ENABLE_REG), regs::INT_ENABLE_OPERATIONAL);
        assert_eq!(bus.reg(regs::INT_ENABLE_SEC_REG), 0);
        assert_eq!(bus.reg(regs::MAC_CTRL_REG_3C04), 0x0F);

        assert_eq!(bus.reg(regs::HW_CRYPTO_DISABLE_START), 0x0003_0000);
        assert_eq!(bus.reg(regs::HW_CRYPTO_DISABLE_START + 4), 0x0003_0000);
        assert_eq!(bus.reg(regs::HW_CRYPTO_DISABLE_START + 8), 0);
        assert_eq!(bus.reg(regs::HW_CRYPTO_DISABLE_START + 12), 0);
        assert_eq!(bus.reg(regs::HW_CRYPTO_DISABLE_START + 16), 0);
        assert_eq!(bus.reg(regs::HW_CRYPTO_DISABLE_END), 0);
        assert_eq!(bus.reg(regs::KEY_TABLE_BASE_REG), 0);
        assert!(bus.barrier_count > 0);
    }

    #[test]
    fn test_init_mac_rejects_invalid_dma_addr_and_timeout_regated_clocks() {
        let mock = MockWifiMmioBus::new();
        let mut hal = LmacHal::new(mock);

        // M-073: null and unaligned RX ring addresses rejected pre-HW write.
        assert_eq!(
            hal.init_mac([0x02; 6], 0),
            Err(LmacError::InvalidDmaAddress)
        );
        assert_eq!(
            hal.init_mac([0x02; 6], 0x3FFE_0002),
            Err(LmacError::InvalidDmaAddress)
        );
        assert_eq!(hal.bus().write_count, 0);

        // M-165: timeout re-gates clocks and clears reset bit 1.
        let res = hal.init_mac([0x02; 6], 0x3FFE_0000);
        assert_eq!(res, Err(LmacError::MacResetTimeout));
        assert!(!hal.is_initialized());
        assert_eq!(hal.bus().reg(regs::MAC_INIT_REG_3D24) & 2, 0);
        assert_eq!(
            hal.bus().reg(regs::MAC_CLK_RST_REG) & regs::MAC_CLK_RST_MASK_UNGATE,
            regs::MAC_CLK_RST_MASK_UNGATE
        );
    }

    #[test]
    fn test_deinit_mac_sequence_drains_and_clears_irqs_before_gating() {
        let mut mock = MockWifiMmioBus::new();
        mock.set_reg(regs::MAC_CLK_RST_REG, 0);
        mock.set_reg(regs::INT_STATUS_REG, regs::INT_TX_DONE | regs::INT_RX_MASK);
        mock.set_reg(regs::RX_CTRL_REG, regs::RX_CTRL_BIT_ENABLE);
        mock.set_reg(regs::TX_PLCP0_TRIG_REG, regs::TX_PLCP0_TRIGGER_BITS);
        let mut hal = LmacHal::new(mock);

        hal.deinit_mac();
        assert!(!hal.is_initialized());
        assert_eq!(hal.bus().reg(regs::INT_ENABLE_REG), 0);
        assert_eq!(hal.bus().reg(regs::INT_ENABLE_SEC_REG), 0);
        assert_eq!(hal.bus().reg(regs::INT_STATUS_REG), 0);
        assert_eq!(
            hal.bus().reg(regs::RX_CTRL_REG) & regs::RX_CTRL_BIT_ENABLE,
            0
        );
        assert_eq!(
            hal.bus().reg(regs::TX_PLCP0_TRIG_REG) & regs::TX_PLCP0_TRIGGER_BITS,
            0
        );
        assert_eq!(
            hal.bus().reg(regs::MAC_CLK_RST_REG) & regs::MAC_CLK_RST_MASK_UNGATE,
            regs::MAC_CLK_RST_MASK_UNGATE
        );
        assert!(hal.bus().barrier_count >= 5);
        assert!(hal.bus().delay_us_total >= 32);
    }

    #[test]
    fn test_rx_policy_scan_vs_associated_bssid() {
        let mock = MockWifiMmioBus::new();
        let mut hal = LmacHal::new(mock);

        hal.set_scan_rx_policy();
        let scan_pol = hal.bus().reg(regs::RX_POLICY_BASE_REG);
        assert_eq!(scan_pol & regs::RX_POLICY_BIT_BSSID_CHECK, 0);
        assert_eq!(
            scan_pol & regs::RX_POLICY_PROMISC_BYPASS,
            regs::RX_POLICY_PROMISC_BYPASS
        );

        let bssid = [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF];
        hal.set_associated_bssid(bssid);
        let assoc_pol = hal.bus().reg(regs::RX_POLICY_BASE_REG);
        assert_eq!(assoc_pol & regs::RX_POLICY_BIT_BSSID_CHECK, 0);
        assert_eq!(assoc_pol & regs::RX_POLICY_PROMISC_BYPASS, 0);

        assert_eq!(hal.bus().reg(regs::bssid_filter_low_reg(0)), 0xDDCC_BBAA);
        assert_eq!(hal.bus().reg(regs::bssid_filter_high_reg(0)), 0xFFEE);
        assert_eq!(hal.bus().reg(regs::bssid_mask_low_reg(0)), 0xFFFF_FFFF);
        assert_eq!(
            hal.bus().reg(regs::bssid_mask_high_ctrl_reg(0)),
            0x0001_FFFF
        );
    }

    #[test]
    fn test_rx_policy_toggles_only_promisc_bypass_bits() {
        let mut mock = MockWifiMmioBus::new();
        mock.set_reg(regs::RX_POLICY_BASE_REG, 0x8000_0002);
        let mut hal = LmacHal::new(mock);

        hal.set_scan_rx_policy();
        assert_eq!(hal.bus().reg(regs::RX_POLICY_BASE_REG), 0x8000_0112);

        hal.set_associated_bssid([0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF]);
        assert_eq!(hal.bus().reg(regs::RX_POLICY_BASE_REG), 0x8000_0002);
    }

    #[test]
    fn test_submit_and_poll_tx_ht_writes_vendor_htsig_words() {
        let mut mock = MockWifiMmioBus::new();
        mock.auto_tx_done = true;
        mock.auto_tx_status = 0;
        mock.set_reg(regs::TX_PLCP2_REG, 0xABCD_000F);
        let mut hal = LmacHal::new(mock);

        let res = hal.submit_and_poll_tx(
            0x3FFE_1000,
            &[0u8; 128],
            WifiPhyRate::Mcs7Lgi,
            40,
            true,
            100,
        );
        assert_eq!(res, Ok(()));

        let psdu_len = 132u32;
        assert_eq!(
            hal.bus().reg(regs::TX_HTSIG1_REG),
            0x0700_0000 | (psdu_len << 8) | 7
        );
        assert_eq!(hal.bus().reg(regs::TX_HTSIG2_REG), psdu_len | 0x0005_0000);
        // M-167: TX_PLCP2_REG upper bits preserved via RMW.
        assert_eq!(hal.bus().reg(regs::TX_PLCP2_REG), 0xABCD_0020);
    }

    #[test]
    fn test_submit_and_poll_tx_rejects_provisional_sgi_rate() {
        let mut mock = MockWifiMmioBus::new();
        mock.auto_tx_done = true;
        mock.auto_tx_status = 0;
        let mut hal = LmacHal::new(mock);

        // M-220: Provisional Mcs*Sgi rejected until short-GI bit confirmed.
        let res = hal.submit_and_poll_tx(
            0x3FFE_1000,
            &[0u8; 128],
            WifiPhyRate::Mcs7Sgi,
            40,
            true,
            100,
        );
        assert_eq!(res, Err(LmacError::UnsupportedRate));
        assert_eq!(hal.bus().write_count, 0);
    }

    #[test]
    fn test_poll_and_clear_irqs_w1c_parity_and_unhandled_preservation() {
        let mut mock = MockWifiMmioBus::new();
        let unhandled_bit = 1u32 << 12;
        let test_status =
            regs::INT_RX_MASK | regs::INT_TX_DONE | regs::INT_PANIC_WDT | unhandled_bit;
        mock.set_reg(regs::INT_STATUS_REG, test_status);
        let mut hal = LmacHal::new(mock);

        let events = hal.poll_and_clear_irqs();
        assert!(events.rx_ready);
        assert!(events.tx_done);
        assert!(!events.tx_collision);
        assert!(!events.tx_timeout);
        assert!(events.panic_wdt);
        assert_eq!(events.raw_unhandled, unhandled_bit);

        // W1C cleared only handled bits; unhandled bit remains latched.
        let expected_cleared = regs::INT_RX_MASK | regs::INT_TX_DONE | regs::INT_PANIC_WDT;
        assert_eq!(hal.bus().reg(regs::INT_CLEAR_REG), expected_cleared);
        assert_eq!(hal.bus().reg(regs::INT_STATUS_REG), unhandled_bit);
        assert!(hal.bus().barrier_count >= 1);
    }

    #[test]
    fn test_efuse_mac_crc8_and_conditioned_rng_gating() {
        let mut mock = MockWifiMmioBus::new();
        mock.auto_init_3d24 = true;
        // Unicast MAC: [0xAA, 0xBB, 0x11, 0x22, 0x33, 0x44]
        let expected_mac = [0xAA, 0xBB, 0x11, 0x22, 0x33, 0x44];
        let crc = efuse_mac_crc8(&expected_mac);
        mock.set_reg(regs::EFUSE_BLK0_RDATA1_REG, 0x1122_3344);
        mock.set_reg(regs::EFUSE_BLK0_RDATA2_REG, (u32::from(crc) << 16) | 0xAABB);

        let mut hal = LmacHal::new(mock);
        assert_eq!(hal.try_read_efuse_sta_mac(), Ok(expected_mac));
        assert_eq!(hal.read_efuse_sta_mac(), expected_mac);

        // Corrupt CRC-8 rejected.
        hal.bus_mut().set_reg(
            regs::EFUSE_BLK0_RDATA2_REG,
            (u32::from(crc ^ 0xFF) << 16) | 0xAABB,
        );
        assert_eq!(
            hal.try_read_efuse_sta_mac(),
            Err(LmacError::InvalidEfuseMac)
        );
        assert_eq!(hal.read_efuse_sta_mac(), [0u8; 6]);

        // Multicast MAC rejected even with valid CRC-8.
        let mcast_mac = [0xAB, 0xBB, 0x11, 0x22, 0x33, 0x44];
        let mcast_crc = efuse_mac_crc8(&mcast_mac);
        hal.bus_mut().set_reg(
            regs::EFUSE_BLK0_RDATA2_REG,
            (u32::from(mcast_crc) << 16) | 0xABBB,
        );
        assert_eq!(
            hal.try_read_efuse_sta_mac(),
            Err(LmacError::InvalidEfuseMac)
        );

        // M-015: RNG gated until init_mac succeeds.
        assert_eq!(hal.try_hw_random_u32(), Err(LmacError::RngNotReady));

        assert_eq!(hal.init_mac(expected_mac, 0x3FFE_0000), Ok(()));
        let delay_before = hal.bus().delay_us_total;
        let r1 = hal.try_hw_random_u32();
        assert!(r1.is_ok());
        // Must enforce inter-read cycle spacing (3 delays of 2 us = 6 us).
        assert!(hal.bus().delay_us_total >= delay_before + 6);
        let r2 = hal.try_hw_random_u32();
        assert!(r2.is_ok());
        assert_ne!(r1, r2);

        // Stuck-at RNG detected and rejected.
        hal.bus_mut().stuck_rng = Some(0xDEAD_BEEF);
        assert_eq!(
            hal.try_hw_random_u32(),
            Err(LmacError::RngHealthCheckFailed)
        );
    }

    #[test]
    fn test_submit_and_poll_tx_success_and_stale_status_cleared() {
        let mut mock = MockWifiMmioBus::new();
        // Seed stale error/done latches: pre-trigger W1C must clear them.
        mock.set_reg(regs::TX_ERR_STATUS_REG, 1);
        mock.set_reg(regs::TX_DONE_STATUS_REG, 1);
        mock.set_reg(regs::TX_TIMEOUT_EDCA_REG, 0xA000_FFFF);
        mock.auto_tx_done = true;
        mock.auto_tx_status = 0;
        let mut hal = LmacHal::new(mock);

        let mpdu = [0u8; 128];
        let res = hal.submit_and_poll_tx(0x3FFE_1000, &mpdu, WifiPhyRate::Mcs7Lgi, 40, true, 100);
        assert_eq!(res, Ok(()));

        let bus = hal.bus();
        let expected_psdu_len = (mpdu.len() as u32).saturating_add(4);
        assert_eq!(
            bus.reg(regs::TX_PLCP1_REG) & 0xFFF,
            expected_psdu_len & 0xFFF
        );
        assert_eq!(
            bus.reg(regs::TX_PLCP1_REG) & regs::TX_PLCP1_BANDWIDTH_BIT,
            regs::TX_PLCP1_BANDWIDTH_BIT
        );
        assert_eq!(bus.reg(regs::TX_TIMEOUT_EDCA_REG), 0xA700_F00A);
        assert_eq!(bus.reg(regs::TX_PLCP0_TRIG_REG), 0);
        assert_eq!(
            (bus.reg(regs::TX_HTSIG1_REG) >> 8) & 0xFFFF,
            expected_psdu_len & 0xFFFF
        );
        assert_eq!(bus.reg(regs::TX_ERR_STATUS_REG), 0);
    }

    #[test]
    fn test_submit_and_poll_tx_ack_fail() {
        let mut mock = MockWifiMmioBus::new();
        mock.auto_tx_done = true;
        mock.auto_tx_status = 5;
        let mut hal = LmacHal::new(mock);

        let res = hal.submit_and_poll_tx(
            0x3FFE_1000,
            &[0u8; 128],
            WifiPhyRate::Rate11mS,
            40,
            true,
            100,
        );
        assert_eq!(res, Err(LmacError::TxAckFailed(5)));
    }

    #[test]
    fn test_submit_and_poll_tx_hw_error() {
        let mut mock = MockWifiMmioBus::new();
        mock.auto_tx_err = 1;
        let mut hal = LmacHal::new(mock);

        let res = hal.submit_and_poll_tx(
            0x3FFE_1000,
            &[0u8; 128],
            WifiPhyRate::Rate54m,
            40,
            true,
            100,
        );
        assert_eq!(res, Err(LmacError::TxHardwareError));
        assert_eq!(
            hal.bus().reg(regs::TX_PLCP0_TRIG_REG) & regs::TX_PLCP0_TRIGGER_BITS,
            0
        );
    }

    #[test]
    fn test_submit_and_poll_tx_timeout_aborts_and_drains() {
        // Case 1: TX_ACTIVE clears when MAC_INIT_REG_3C58 abort reset pulses.
        let mut mock = MockWifiMmioBus::new();
        mock.set_reg(regs::MAC_CLK_RST_REG, regs::MAC_CLK_RST_BIT_TX_ACTIVE);
        mock.auto_clear_tx_active_on_abort_reset = true;
        let mut hal = LmacHal::new(mock);

        let res = hal.submit_and_poll_tx(
            0x3FFE_1000,
            &[0u8; 128],
            WifiPhyRate::Rate1mL,
            40,
            false,
            10,
        );
        assert_eq!(res, Err(LmacError::TxTimeout));
        assert_eq!(
            hal.bus().reg(regs::TX_PLCP0_TRIG_REG) & regs::TX_PLCP0_TRIGGER_BITS,
            0
        );
        assert_eq!(
            hal.bus().reg(regs::MAC_CLK_RST_REG) & regs::MAC_CLK_RST_BIT_TX_ACTIVE,
            0
        );
        // 10 poll waits + 200 drain waits + 5 us abort reset pulse.
        assert!(hal.bus().delay_us_total >= 10 + u64::from(MAX_TX_DRAIN_POLLS) + 5);

        // Case 2: TX_ACTIVE stays stuck high even after abort reset -> TxDrainFailed.
        let mut stuck = MockWifiMmioBus::new();
        stuck.set_reg(regs::MAC_CLK_RST_REG, regs::MAC_CLK_RST_BIT_TX_ACTIVE);
        let mut hal_stuck = LmacHal::new(stuck);
        let res_stuck = hal_stuck.submit_and_poll_tx(
            0x3FFE_1000,
            &[0u8; 128],
            WifiPhyRate::Rate1mL,
            40,
            false,
            10,
        );
        assert_eq!(res_stuck, Err(LmacError::TxDrainFailed));
    }

    #[test]
    fn test_submit_and_poll_tx_broadcast_sets_no_ack_bits() {
        let mut mock = MockWifiMmioBus::new();
        mock.auto_tx_done = true;
        mock.auto_tx_status = 0;
        let mut hal = LmacHal::new(mock);

        let res =
            hal.submit_and_poll_tx(0x3FFE_1000, &[0u8; 64], WifiPhyRate::Rate1mL, 0, false, 100);
        assert_eq!(res, Ok(()));

        let plcp0 = hal.bus().last_tx_plcp0_triggered;
        assert_eq!(
            plcp0 & 0x0360_0000,
            regs::TX_PLCP0_NO_ACK | regs::TX_PLCP0_MODE_LEGACY
        );
        assert_eq!(regs::TX_PLCP0_NO_ACK, 0x0000_0000);
        assert_eq!(hal.bus().reg(regs::TX_PLCP0_TRIG_REG), 0);
    }

    #[test]
    fn test_submit_and_poll_tx_unicast_sets_wait_ack_bits() {
        let mut mock = MockWifiMmioBus::new();
        mock.auto_tx_done = true;
        mock.auto_tx_status = 0;
        let mut hal = LmacHal::new(mock);

        let res = hal.submit_and_poll_tx(
            0x3FFE_1000,
            &[0u8; 64],
            WifiPhyRate::Rate1mL,
            314,
            true,
            100,
        );
        assert_eq!(res, Ok(()));

        let plcp0 = hal.bus().last_tx_plcp0_triggered;
        assert_eq!(
            plcp0 & 0x0360_0000,
            regs::TX_PLCP0_WAIT_ACK | regs::TX_PLCP0_MODE_LEGACY
        );
        assert_eq!(hal.bus().reg(regs::TX_PLCP0_TRIG_REG), 0);
    }

    #[test]
    fn test_submit_and_poll_tx_validates_len_polls_and_dma_addr() {
        let mock = MockWifiMmioBus::new();
        let mut hal = LmacHal::new(mock);

        assert_eq!(
            hal.submit_and_poll_tx(0x3FFE_1000, &[], WifiPhyRate::Rate1mL, 0, false, 10),
            Err(LmacError::InvalidPayloadLength)
        );
        assert_eq!(
            hal.submit_and_poll_tx(
                0x3FFE_1000,
                &[0u8; 1601],
                WifiPhyRate::Rate1mL,
                0,
                false,
                10
            ),
            Err(LmacError::InvalidPayloadLength)
        );
        // M-167: max_polls == 0 and > MAX_TX_POLLS rejected.
        assert_eq!(
            hal.submit_and_poll_tx(0x3FFE_1000, &[0u8; 64], WifiPhyRate::Rate1mL, 0, false, 0),
            Err(LmacError::InvalidPollCount)
        );
        assert_eq!(
            hal.submit_and_poll_tx(
                0x3FFE_1000,
                &[0u8; 64],
                WifiPhyRate::Rate1mL,
                0,
                false,
                MAX_TX_POLLS + 1
            ),
            Err(LmacError::InvalidPollCount)
        );
        // M-073: null and misaligned descriptor addresses rejected.
        assert_eq!(
            hal.submit_and_poll_tx(0, &[0u8; 64], WifiPhyRate::Rate1mL, 0, false, 10),
            Err(LmacError::InvalidDmaAddress)
        );
        assert_eq!(
            hal.submit_and_poll_tx(0x3FFE_1002, &[0u8; 64], WifiPhyRate::Rate1mL, 0, false, 10),
            Err(LmacError::InvalidDmaAddress)
        );
        assert_eq!(hal.bus().write_count, 0);
    }

    #[test]
    fn test_init_mac_writes_3d24_reset_bit_before_ready_poll() {
        let mut mock = MockWifiMmioBus::new();
        mock.auto_init_3d24 = true;
        let mut hal = LmacHal::new(mock);

        let mac = [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC];
        let ring_addr = 0x3FFE_0000;
        let res = hal.init_mac(mac, ring_addr);
        assert_eq!(res, Ok(()));
        assert_ne!(hal.bus().reg(regs::MAC_INIT_REG_3D24) & 2, 0);
    }

    #[test]
    fn test_tx_collision_and_timeout_error_bits() {
        let mut mock = MockWifiMmioBus::new();
        mock.auto_tx_err = 1 << 16;
        let mut hal = LmacHal::new(mock);

        let res = hal.submit_and_poll_tx(
            0x3FFE_1000,
            &[0u8; 128],
            WifiPhyRate::Rate54m,
            40,
            true,
            100,
        );
        assert_eq!(res, Err(LmacError::TxHardwareError));
        assert_eq!(hal.bus().reg(regs::TX_ERR_CLEAR_REG) & (1 << 16), 1 << 16);
    }

    #[test]
    fn test_hal_crypto_init_passthrough_registers() {
        let mut mock = MockWifiMmioBus::new();
        mock.auto_init_3d24 = true;
        let mut offset = 0u32;
        while offset <= 20 {
            mock.set_reg(regs::HW_CRYPTO_DISABLE_START + offset, 0xDEAD_BEEF);
            offset += 4;
        }
        let mut hal = LmacHal::new(mock);

        assert_eq!(hal.init_mac([0x02; 6], 0x3FFE_0000), Ok(()));

        let expected = [0x0003_0000u32, 0x0003_0000, 0, 0, 0, 0];
        for (idx, &exp) in expected.iter().enumerate() {
            let addr = regs::HW_CRYPTO_DISABLE_START + (idx as u32) * 4;
            assert_eq!(hal.bus().reg(addr), exp);
        }
    }

    #[test]
    fn test_install_and_clear_sta_ccmp_keys() {
        let mut mock = MockWifiMmioBus::new();
        mock.auto_init_3d24 = true;
        mock.set_reg(regs::HW_CRYPTO_DISABLE_START + 0x10, 0xA5FF_FF5A);
        let mut hal = LmacHal::new(mock);
        assert!(!hal.is_sta_hw_crypto_enabled());

        let bssid = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66];
        let ptk = [
            0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E,
            0x0F, 0x10,
        ];
        let gtk = [
            0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xAB, 0xAC, 0xAD, 0xAE,
            0xAF, 0xB0,
        ];

        // Install PTK in slot 4 (key_idx 0 -> mode_bits = 3 << 21).
        hal.install_sta_ccmp_key(4, 0, bssid, &ptk);
        assert!(hal.is_sta_hw_crypto_enabled());
        let ptk_base = regs::KEY_TABLE_BASE_REG + 4 * regs::KEY_SLOT_STRIDE;
        assert_eq!(hal.bus().reg(ptk_base), 0x4433_2211);
        assert_eq!(
            hal.bus().reg(ptk_base + 4),
            0x6655 | (3u32 << 18) | (3u32 << 21)
        );
        assert_eq!(hal.bus().reg(ptk_base + 8), 0x0403_0201);
        assert_eq!(hal.bus().reg(ptk_base + 12), 0x0807_0605);
        assert_eq!(hal.bus().reg(ptk_base + 16), 0x0C0B_0A09);
        assert_eq!(hal.bus().reg(ptk_base + 20), 0x100F_0E0D);
        for off in [24u32, 28, 32, 36] {
            assert_eq!(hal.bus().reg(ptk_base + off), 0);
        }
        assert_eq!(
            hal.bus().reg(regs::HW_CRYPTO_DISABLE_START + 0x10),
            0xA500_005A
        );
        assert_eq!(hal.bus().reg(regs::HW_CRYPTO_DISABLE_START), 0x0003_0103);
        assert_eq!(
            hal.bus().reg(regs::HW_CRYPTO_DISABLE_END) & (1 << 4),
            1 << 4
        );

        // Install GTK in slot 1 (key_idx 1 -> mode_bits = 6 << 21, key_id_bits = 1 << 30).
        hal.install_sta_ccmp_key(1, 1, bssid, &gtk);
        let gtk_base = regs::KEY_TABLE_BASE_REG + regs::KEY_SLOT_STRIDE;
        assert_eq!(hal.bus().reg(gtk_base), 0x4433_2211);
        assert_eq!(
            hal.bus().reg(gtk_base + 4),
            0x6655 | (3u32 << 18) | (6u32 << 21) | (1u32 << 30)
        );
        assert_eq!(hal.bus().reg(gtk_base + 8), 0xA4A3_A2A1);
        assert_eq!(
            hal.bus().reg(regs::HW_CRYPTO_DISABLE_END) & ((1 << 4) | (1 << 1)),
            (1 << 4) | (1 << 1)
        );

        // Out-of-range slot is safely ignored.
        let writes_before = hal.bus().write_count;
        hal.install_sta_ccmp_key(regs::KEY_SLOT_COUNT as u8, 0, bssid, &ptk);
        assert_eq!(hal.bus().write_count, writes_before);

        // Preserve unrelated upper slot mask bit when clearing STA slots 0..=4.
        hal.bus_mut()
            .set_reg(regs::HW_CRYPTO_DISABLE_END, (1 << 5) | (1 << 4) | (1 << 1));
        hal.clear_sta_ccmp_keys();
        assert!(!hal.is_sta_hw_crypto_enabled());
        assert_eq!(hal.bus().reg(regs::HW_CRYPTO_DISABLE_START), 0x0003_0000);
        assert_eq!(hal.bus().reg(regs::HW_CRYPTO_DISABLE_END), 1 << 5);
        for slot in 0..=4u32 {
            let base = regs::KEY_TABLE_BASE_REG + slot * regs::KEY_SLOT_STRIDE;
            for word_idx in 0..10u32 {
                assert_eq!(hal.bus().reg(base + word_idx * 4), 0);
            }
        }
    }

    #[test]
    fn test_submit_and_poll_tx_plcp1_sw_crypto_bypass_and_unchanged_crypto_ctrl() {
        let bssid = [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF];
        let key = [0x55u8; 16];

        let mut protected_mpdu = [0u8; 64];
        protected_mpdu[0] = 0x08;
        protected_mpdu[1] = 0x41; // ToDS | Protected (0x40)

        let mut unprotected_mpdu = [0u8; 64];
        unprotected_mpdu[0] = 0x08;
        unprotected_mpdu[1] = 0x01; // ToDS only, Protected == 0

        // Case 1: Protected == 1 sets TX_PLCP1_REG[24:17] = 0xC0 << 17 (0x0180_0000)
        // and leaves HW_CRYPTO_DISABLE_START continuously at 0x0003_0103.
        let mut mock_ok = MockWifiMmioBus::new();
        mock_ok.auto_tx_done = true;
        mock_ok.auto_tx_status = 0;
        let mut hal_ok = LmacHal::new(mock_ok);
        hal_ok.install_sta_ccmp_key(4, 0, bssid, &key);
        assert_eq!(
            hal_ok.submit_and_poll_tx(
                0x3FFE_1000,
                &protected_mpdu,
                WifiPhyRate::Rate6m,
                40,
                true,
                100
            ),
            Ok(())
        );
        assert_eq!(
            hal_ok.bus().reg(regs::TX_PLCP1_REG) & 0x01FE_0000,
            0x0180_0000
        );
        assert_eq!(hal_ok.bus().crypto_ctrl_at_tx_trigger, 0x0003_0103);
        assert_eq!(hal_ok.bus().reg(regs::HW_CRYPTO_DISABLE_START), 0x0003_0103);

        // Case 2: Protected == 0 (both normal and 1-byte MPDU) leaves TX_PLCP1_REG[24:17] == 0.
        assert_eq!(
            hal_ok.submit_and_poll_tx(
                0x3FFE_1000,
                &unprotected_mpdu,
                WifiPhyRate::Rate6m,
                40,
                true,
                100
            ),
            Ok(())
        );
        assert_eq!(hal_ok.bus().reg(regs::TX_PLCP1_REG) & 0x01FE_0000, 0);
        assert_eq!(hal_ok.bus().crypto_ctrl_at_tx_trigger, 0x0003_0103);
        assert_eq!(hal_ok.bus().reg(regs::HW_CRYPTO_DISABLE_START), 0x0003_0103);

        assert_eq!(
            hal_ok.submit_and_poll_tx(0x3FFE_1000, &[0x40], WifiPhyRate::Rate6m, 40, false, 100),
            Ok(())
        );
        assert_eq!(hal_ok.bus().reg(regs::TX_PLCP1_REG) & 0x01FE_0000, 0);

        // Case 3: TxAckFailed keeps HW_CRYPTO_DISABLE_START at 0x0003_0103.
        let mut mock_ack_err = MockWifiMmioBus::new();
        mock_ack_err.auto_tx_done = true;
        mock_ack_err.auto_tx_status = 3;
        let mut hal_ack_err = LmacHal::new(mock_ack_err);
        hal_ack_err.install_sta_ccmp_key(4, 0, bssid, &key);
        assert_eq!(
            hal_ack_err.submit_and_poll_tx(
                0x3FFE_1000,
                &protected_mpdu,
                WifiPhyRate::Rate6m,
                40,
                true,
                100
            ),
            Err(LmacError::TxAckFailed(3))
        );
        assert_eq!(
            hal_ack_err.bus().reg(regs::TX_PLCP1_REG) & 0x01FE_0000,
            0x0180_0000
        );
        assert_eq!(hal_ack_err.bus().crypto_ctrl_at_tx_trigger, 0x0003_0103);
        assert_eq!(
            hal_ack_err.bus().reg(regs::HW_CRYPTO_DISABLE_START),
            0x0003_0103
        );

        // Case 4: TxHardwareError keeps HW_CRYPTO_DISABLE_START at 0x0003_0103.
        let mut mock_hw_err = MockWifiMmioBus::new();
        mock_hw_err.auto_tx_err = 1;
        let mut hal_hw_err = LmacHal::new(mock_hw_err);
        hal_hw_err.install_sta_ccmp_key(4, 0, bssid, &key);
        assert_eq!(
            hal_hw_err.submit_and_poll_tx(
                0x3FFE_1000,
                &protected_mpdu,
                WifiPhyRate::Rate6m,
                40,
                true,
                100
            ),
            Err(LmacError::TxHardwareError)
        );
        assert_eq!(
            hal_hw_err.bus().reg(regs::HW_CRYPTO_DISABLE_START),
            0x0003_0103
        );

        // Case 5: TxTimeout keeps HW_CRYPTO_DISABLE_START at 0x0003_0103.
        let mut mock_to = MockWifiMmioBus::new();
        mock_to.set_reg(regs::MAC_CLK_RST_REG, regs::MAC_CLK_RST_BIT_TX_ACTIVE);
        mock_to.auto_clear_tx_active_on_abort_reset = true;
        let mut hal_to = LmacHal::new(mock_to);
        hal_to.install_sta_ccmp_key(4, 0, bssid, &key);
        assert_eq!(
            hal_to.submit_and_poll_tx(
                0x3FFE_1000,
                &protected_mpdu,
                WifiPhyRate::Rate6m,
                40,
                false,
                5
            ),
            Err(LmacError::TxTimeout)
        );
        assert_eq!(hal_to.bus().reg(regs::HW_CRYPTO_DISABLE_START), 0x0003_0103);
    }

    #[test]
    fn test_reload_rx_dma_pulses_and_autoclears_reload_bit() {
        let mut mock = MockWifiMmioBus::new();
        mock.set_reg(regs::RX_CTRL_REG, regs::RX_CTRL_BIT_ENABLE);
        let mut hal = LmacHal::new(mock);

        hal.reload_rx_dma();
        assert_eq!(hal.bus().rx_ctrl_reload_count, 1);
        assert_eq!(hal.bus().reg(regs::RX_CTRL_REG), regs::RX_CTRL_BIT_ENABLE);
    }

    #[test]
    fn test_enable_low_rate_restores_11b_cck_bits_with_peribus_barrier() {
        let mut mock = MockWifiMmioBus::new();
        mock.auto_init_3d24 = true;
        let mut hal = LmacHal::new(mock);

        assert_eq!(hal.init_mac([0x02; 6], 0x3FFE_0000), Ok(()));
        assert_eq!(hal.bus().reg(regs::AUTOACK_RATE_INIT_REGS[7]), 0x090B_0B0B);

        let barriers_before = hal.bus().barrier_count;
        hal.enable_low_rate();
        assert!(hal.bus().barrier_count >= barriers_before + 2);
        assert_eq!(hal.bus().reg(regs::AUTOACK_RATE_INIT_REGS[7]), 0x0B0B_0B0B);
        assert_eq!(hal.bus().reg(regs::AUTOACK_RATE_INIT_REGS[6]), 0x0B0B_0B0B);
        assert_eq!(
            hal.bus().reg(regs::MAC_LOW_RATE_DISABLE_REG_6860) & 0x0000_0C00,
            0x0000_0C00
        );
        assert_eq!(
            hal.bus().reg(regs::MAC_LOW_RATE_DISABLE_REG_687C) & 0x0000_0800,
            0x0000_0800
        );
    }

    #[test]
    fn test_hardware_bus_rejects_disallowed_mmio_without_ub() {
        // JUSTIFICATION: Host test constructs a no-op HardwareWifiMmioBus to
        // verify allowlist gating without touching real MMIO.
        // SAFETY: Host build performs no raw pointer dereferences.
        let mut bus = unsafe { HardwareWifiMmioBus::new() };
        // Disallowed RAM/null/misaligned addresses safely return 0 / no-op.
        assert_eq!(bus.read_reg(0x0000_0000), 0);
        assert_eq!(bus.read_reg(0x3FFB_0000), 0);
        assert_eq!(bus.read_reg(regs::RX_CTRL_REG + 1), 0);
        bus.write_reg(0x0000_0000, 0x1234_5678);
        bus.write_reg(regs::WDEV_RND_REG, 0x1234_5678);
        bus.write_reg(regs::DPORT_WIFI_CLK_EN_REG, 0x1234_5678);
        bus.write_reg(regs::DPORT_CORE_RST_EN_REG, 0x1234_5678);
    }
}
