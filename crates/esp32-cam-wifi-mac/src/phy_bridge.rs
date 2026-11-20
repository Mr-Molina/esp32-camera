//! ESP32 Wi-Fi PHY abstraction bridge and host simulation fixture.
//!
//! Bridges low-level Wi-Fi physical layer (PHY) control routines provided by
//! `esp-phy` and `libphy.a` on Xtensa targets, and exposes a pure-Rust atomic
//! mock state machine for unit tests and simulation on host targets.

use core::fmt;
use core::sync::atomic::AtomicBool;
use core::sync::atomic::AtomicU8;
use core::sync::atomic::Ordering;

#[cfg(target_arch = "xtensa")]
use crate::regs::DPORT_CORE_RST_EN_REG;
#[cfg(target_arch = "xtensa")]
use crate::regs::DPORT_WIFIMAC_RST;
#[cfg(target_arch = "xtensa")]
use crate::regs::DPORT_WIFI_CLK_EN_REG;
#[cfg(target_arch = "xtensa")]
use crate::regs::DPORT_WIFI_CLK_WIFI_BT_COMMON_M;
#[cfg(target_arch = "xtensa")]
use crate::regs::DPORT_WIFI_CLK_WIFI_EN_M;
#[cfg(target_arch = "xtensa")]
use crate::regs::RX_CTRL_REG;

#[cfg(not(target_arch = "xtensa"))]
use core::sync::atomic::AtomicI8;
#[cfg(not(target_arch = "xtensa"))]
use core::sync::atomic::AtomicU32;

/// Minimum allowed TX power ceiling in quarter-dBm (2 dBm, ESP-IDF limit).
pub const MIN_TX_POWER_QDBM: i8 = 8;
/// Maximum allowed TX power ceiling in quarter-dBm (21 dBm, ESP-IDF limit).
pub const MAX_TX_POWER_QDBM: i8 = 84;

/// Regulatory domain policy controlling permitted 2.4 GHz Wi-Fi channels.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum RegulatoryDomain {
    /// FCC (United States / North America): channels 1..=11.
    Fcc = 0,
    /// ETSI (Europe): channels 1..=13.
    Etsi = 1,
    /// TELEC (Japan): channels 1..=14.
    Japan = 2,
}

impl RegulatoryDomain {
    /// Returns the maximum permitted 2.4 GHz channel number for this domain.
    #[inline]
    #[must_use]
    pub const fn max_channel(self) -> u8 {
        match self {
            Self::Fcc => 11,
            Self::Etsi => 13,
            Self::Japan => 14,
        }
    }

    #[inline]
    const fn from_u8(val: u8) -> Self {
        match val {
            1 => Self::Etsi,
            2 => Self::Japan,
            _ => Self::Fcc,
        }
    }
}

/// Active regulatory domain (defaults to ETSI channels 1..=13 to match the
/// STA scanner sweep while blocking channel 14; callers may tighten to FCC).
static ACTIVE_REG_DOMAIN: AtomicU8 = AtomicU8::new(RegulatoryDomain::Etsi as u8);

/// Sets the active 2.4 GHz regulatory domain policy.
pub fn phy_set_regulatory_domain(domain: RegulatoryDomain) {
    ACTIVE_REG_DOMAIN.store(domain as u8, Ordering::Release);
}

/// Returns the active 2.4 GHz regulatory domain policy.
#[must_use]
pub fn phy_regulatory_domain() -> RegulatoryDomain {
    RegulatoryDomain::from_u8(ACTIVE_REG_DOMAIN.load(Ordering::Acquire))
}

/// Errors returned by Wi-Fi PHY bridge operations.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PhyBridgeError {
    /// Specified channel is outside the active regulatory 2.4 GHz range.
    InvalidChannel(u8),
    /// Requested a PHY/MAC reset while no [`XtensaPhySession`] (or host mock
    /// PHY session) is active.
    PhySessionInactive,
}

impl fmt::Display for PhyBridgeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match *self {
            Self::InvalidChannel(chan) => write!(
                f,
                "invalid Wi-Fi 2.4 GHz channel: {chan} (outside regulatory domain)"
            ),
            Self::PhySessionInactive => {
                f.write_str("PHY session is not active (initialize XtensaPhySession first)")
            }
        }
    }
}

// ============================================================================
// Cross-Core / ISR-Safe Spinlock & ESP32 ECO 3.10 Protection
// ============================================================================

/// Cross-core spinlock guarding shared `DPORT_*`, `RX_CTRL_REG`, and
/// non-reentrant `libphy.a` / `libcoexist.a` RMW sequences.
static MMIO_RMW_LOCK: AtomicBool = AtomicBool::new(false);

/// Tracks whether an [`XtensaPhySession`] (or mock PHY session) is active.
static PHY_SESSION_ACTIVE: AtomicBool = AtomicBool::new(false);

/// Returns `true` if a hardware (or mock) PHY session is currently active.
#[inline]
#[must_use]
pub fn is_phy_session_active() -> bool {
    PHY_SESSION_ACTIVE.load(Ordering::Acquire)
}

/// RAII guard for [`MMIO_RMW_LOCK`] that masks local interrupts on Xtensa
/// (preventing ISR preemption deadlocks and ESP32 ECO 3.10 DPORT read
/// contention) and always releases on drop.
struct MmioRmwGuard {
    #[cfg(target_arch = "xtensa")]
    prev_ps: u32,
}

impl MmioRmwGuard {
    #[inline]
    fn acquire() -> Self {
        #[cfg(target_arch = "xtensa")]
        let prev_ps = {
            let ps: u32;
            // JUSTIFICATION: Masking Xtensa interrupts before acquiring the
            // cross-core MMIO spinlock prevents ISR reentrancy deadlocks and
            // satisfies ESP32 ECO 3.10 DPORT read errata requirements. Omitting
            // `nomem` prevents LLVM from reordering lock ops before `rsil`.
            // SAFETY: `rsil` reads and sets PS.INTLEVEL to 15 atomically;
            // `Drop` restores the exact prior PS value.
            unsafe {
                core::arch::asm!("rsil {0}, 15", out(reg) ps, options(nostack));
            }
            ps
        };

        while MMIO_RMW_LOCK
            .compare_exchange_weak(false, true, Ordering::Acquire, Ordering::Relaxed)
            .is_err()
        {
            core::hint::spin_loop();
        }

        #[cfg(target_arch = "xtensa")]
        {
            // JUSTIFICATION: ESP32 ECO 3.10 requires draining prior bus
            // transactions via `memw` before DPORT/MMIO register reads.
            // SAFETY: `memw` orders memory operations with no memory operands.
            unsafe {
                core::arch::asm!("memw", options(nostack, preserves_flags));
            }
        }

        Self {
            #[cfg(target_arch = "xtensa")]
            prev_ps,
        }
    }
}

impl Drop for MmioRmwGuard {
    #[inline]
    fn drop(&mut self) {
        #[cfg(target_arch = "xtensa")]
        {
            // JUSTIFICATION: Drain all MMIO writes before releasing the
            // cross-core spinlock and unmasking interrupts.
            // SAFETY: `memw` orders memory operations with no memory operands.
            unsafe {
                core::arch::asm!("memw", options(nostack, preserves_flags));
            }
        }

        MMIO_RMW_LOCK.store(false, Ordering::Release);

        #[cfg(target_arch = "xtensa")]
        {
            // JUSTIFICATION: Restores the Xtensa processor status interrupt
            // mask captured when acquiring `MmioRmwGuard`; omitting `nomem`
            // prevents LLVM from hoisting unmasking above the lock release.
            // SAFETY: `self.prev_ps` came directly from `rsil` on this core.
            unsafe {
                core::arch::asm!(
                    "wsr.ps {0}",
                    "rsync",
                    in(reg) self.prev_ps,
                    options(nostack)
                );
            }
        }
    }
}

/// Executes `f` inside the crate-wide MMIO RMW / ECO 3.10 critical section.
#[inline]
pub(crate) fn with_mmio_rmw_lock<R, F: FnOnce() -> R>(f: F) -> R {
    let _guard = MmioRmwGuard::acquire();
    f()
}

/// Non-elidable calibrated cycle delay on Xtensa using `rsr.ccount`.
#[cfg(target_arch = "xtensa")]
#[inline]
fn xtensa_delay_us_ccount(us: u32) {
    const CYCLES_PER_US: u32 = 240;
    let target_cycles = us.saturating_mul(CYCLES_PER_US);
    let start: u32;
    // JUSTIFICATION: Reading Xtensa CCOUNT provides a non-elidable hardware
    // cycle counter for calibrated reset pulse durations.
    // SAFETY: `rsr.ccount` only reads the processor cycle counter register.
    unsafe {
        core::arch::asm!(
            "rsr.ccount {0}",
            out(reg) start,
            options(nomem, nostack, preserves_flags)
        );
    }
    loop {
        let now: u32;
        // JUSTIFICATION: Volatile CCOUNT read prevents LLVM from eliding the
        // hardware reset hold loop.
        // SAFETY: `rsr.ccount` only reads the processor cycle counter register.
        unsafe {
            core::arch::asm!(
                "rsr.ccount {0}",
                out(reg) now,
                options(nomem, nostack, preserves_flags)
            );
        }
        if now.wrapping_sub(start) >= target_cycles {
            break;
        }
    }
}

// ============================================================================
// Host Simulation State (not(target_arch = "xtensa"))
// ============================================================================

#[cfg(not(target_arch = "xtensa"))]
static MOCK_CHANNEL: AtomicU8 = AtomicU8::new(1);
#[cfg(not(target_arch = "xtensa"))]
static MOCK_AGC_ENABLED: AtomicBool = AtomicBool::new(true);
#[cfg(not(target_arch = "xtensa"))]
static MOCK_RX_ENABLED: AtomicBool = AtomicBool::new(false);
#[cfg(not(target_arch = "xtensa"))]
static MOCK_PHY_ENABLED: AtomicBool = AtomicBool::new(false);
#[cfg(not(target_arch = "xtensa"))]
static MOCK_MAX_TX_POWER_QDBM: AtomicI8 = AtomicI8::new(0);
#[cfg(not(target_arch = "xtensa"))]
static MOCK_BB_WDT_ENABLED: AtomicBool = AtomicBool::new(false);
#[cfg(not(target_arch = "xtensa"))]
static MOCK_LOW_RATE_DISABLED: AtomicBool = AtomicBool::new(false);
#[cfg(not(target_arch = "xtensa"))]
static MOCK_LOW_RATE_ENABLED: AtomicBool = AtomicBool::new(false);
#[cfg(not(target_arch = "xtensa"))]
static MOCK_PWCTRL_BG_CALLS: AtomicU32 = AtomicU32::new(0);

#[cfg(not(target_arch = "xtensa"))]
/// Snapshot of the host-side PHY mock state for test inspection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HostPhyMockState {
    /// Active 802.11 2.4 GHz channel (1..=14).
    pub channel: u8,
    /// Whether Wi-Fi automatic gain control (AGC) is currently active.
    pub agc_enabled: bool,
    /// Whether Wi-Fi RX baseband reception is enabled.
    pub rx_enabled: bool,
    /// Whether RF PHY hardware is active.
    pub phy_enabled: bool,
    /// Maximum RF TX power in quarter-dBm units.
    pub max_tx_power_qdbm: i8,
    /// Whether baseband watchdog interrupt is armed.
    pub bb_wdt_enabled: bool,
    /// Whether legacy 802.11b 1Mbps/2Mbps CCK low rates are disabled.
    pub low_rate_disabled: bool,
    /// Number of background TX power control recalibration cycles run.
    pub pwctrl_bg_calls: u32,
}

#[cfg(not(target_arch = "xtensa"))]
/// Captures a snapshot of the host mock PHY state.
#[must_use]
pub fn host_mock_snapshot() -> HostPhyMockState {
    HostPhyMockState {
        channel: MOCK_CHANNEL.load(Ordering::SeqCst),
        agc_enabled: MOCK_AGC_ENABLED.load(Ordering::SeqCst),
        rx_enabled: MOCK_RX_ENABLED.load(Ordering::SeqCst),
        phy_enabled: MOCK_PHY_ENABLED.load(Ordering::SeqCst),
        max_tx_power_qdbm: MOCK_MAX_TX_POWER_QDBM.load(Ordering::SeqCst),
        bb_wdt_enabled: MOCK_BB_WDT_ENABLED.load(Ordering::SeqCst),
        low_rate_disabled: MOCK_LOW_RATE_DISABLED.load(Ordering::SeqCst),
        pwctrl_bg_calls: MOCK_PWCTRL_BG_CALLS.load(Ordering::SeqCst),
    }
}

#[cfg(not(target_arch = "xtensa"))]
/// Resets the host mock PHY state to baseline defaults.
pub fn host_mock_reset() {
    MOCK_CHANNEL.store(1, Ordering::SeqCst);
    MOCK_AGC_ENABLED.store(true, Ordering::SeqCst);
    MOCK_RX_ENABLED.store(false, Ordering::SeqCst);
    MOCK_PHY_ENABLED.store(false, Ordering::SeqCst);
    PHY_SESSION_ACTIVE.store(false, Ordering::SeqCst);
    MOCK_MAX_TX_POWER_QDBM.store(0, Ordering::SeqCst);
    MOCK_BB_WDT_ENABLED.store(false, Ordering::SeqCst);
    MOCK_LOW_RATE_DISABLED.store(false, Ordering::SeqCst);
    MOCK_LOW_RATE_ENABLED.store(false, Ordering::SeqCst);
    MOCK_PWCTRL_BG_CALLS.store(0, Ordering::SeqCst);
    ACTIVE_REG_DOMAIN.store(RegulatoryDomain::Etsi as u8, Ordering::SeqCst);
}

#[cfg(not(target_arch = "xtensa"))]
/// Returns whether 802.11b low-rate CCK modulation is enabled on host.
#[must_use]
pub fn host_mock_is_low_rate_enabled() -> bool {
    MOCK_LOW_RATE_ENABLED.load(Ordering::SeqCst)
}

#[cfg(not(target_arch = "xtensa"))]
/// Sets the mock PHY hardware enabled state on host.
pub fn host_mock_set_phy_enabled(enabled: bool) {
    MOCK_PHY_ENABLED.store(enabled, Ordering::SeqCst);
    PHY_SESSION_ACTIVE.store(enabled, Ordering::SeqCst);
}

// ============================================================================
// Xtensa Hardware FFI & Guards (target_arch = "xtensa")
// ============================================================================

// JUSTIFICATION: Symbols provided by pinned `esp-wifi-sys-esp32` (`0.3.0`)
// `libphy.a` and `libcoexist.a` archives on ESP32 Xtensa targets; RF/PA
// calibration registers are undocumented in public hardware manuals.
// SAFETY: Signatures match ESP-IDF v5.x `libphy.a` / `libcoexist.a` Xtensa
// calling convention (32-bit register-promoted integer args in a2..a5).
// All calls are serialized via `with_mmio_rmw_lock` with validated args.
#[cfg(target_arch = "xtensa")]
extern "C" {
    // Disassembly of `libphy.a` (`esp-wifi-sys-esp32 0.3.0`) proves
    // `phy_change_channel_nomac(a2, a3, a4, a5)` forwards `(a2=chan, a5=ext)`
    // to `set_chanfreq_nomac` -> `chip_v7_set_chan_nomac(chan, ext)` while
    // ignoring `a3`/`a4`. Passing `0` (`HT20` / `chan_type=0`) for both `bw20`
    // and `ext_chan` matches `esp32-open-mac`'s `chip_v7_set_chan_nomac(ch, 0)`.
    fn phy_change_channel_nomac(chan: u8, bw20: u8, zero: u8, ext_chan: u8);
    fn chip_v7_set_chan_misc(chan: u8, ext_chan: u8);
    fn phy_chan_filt_set(chan: u8, bw: u8);
    fn disable_wifi_agc();
    fn enable_wifi_agc();
    fn phy_set_most_tpw(max_tx_power_qdbm: i8);
    fn bb_wdt_int_enable(enable: u32);
    fn tx_pwctrl_background(en: u32, calibrate_now: u32);
    fn phy_disable_low_rate();
    fn phy_enable_low_rate();
    fn coex_bt_high_prio();
}

#[cfg(target_arch = "xtensa")]
/// RAII session guard for active Xtensa PHY and PHY clock initialization.
///
/// Dropping this session automatically disables PHY and PHY clocks in reverse
/// initialization order via `esp_phy` RAII guards; out-of-band shutdown is
/// intentionally disallowed.
pub struct XtensaPhySession {
    _init_guard: esp_phy::PhyInitGuard<'static>,
    _clock_guard: esp_phy::PhyClockGuard<'static>,
}

#[cfg(target_arch = "xtensa")]
impl XtensaPhySession {
    /// Initializes PHY and PHY clocks, returning an RAII session token.
    #[inline]
    #[must_use]
    pub fn new() -> Self {
        let _ = core::hint::black_box(esp_wifi_sys_esp32::force_rf_clock_link());

        with_mmio_rmw_lock(|| {
            // JUSTIFICATION: DPORT Wi-Fi and Bluetooth common clock gating
            // requires volatile MMIO read-modify-write under ECO 3.10 lock.
            // SAFETY: DPORT_WIFI_CLK_EN_REG is a valid 4-byte aligned ESP32
            // MMIO register (soc/dport_reg.h). Interrupts are masked and
            // cross-core lock + `memw` are held by `with_mmio_rmw_lock`.
            unsafe {
                let clk_ptr = DPORT_WIFI_CLK_EN_REG as *mut u32;
                let clk = core::ptr::read_volatile(clk_ptr);
                core::ptr::write_volatile(
                    clk_ptr,
                    clk | (DPORT_WIFI_CLK_WIFI_EN_M | DPORT_WIFI_CLK_WIFI_BT_COMMON_M),
                );
                core::arch::asm!("memw", options(nostack, preserves_flags));
            }
        });

        let clock_guard = esp_phy::enable_phy_clock();
        let init_guard = esp_phy::enable_phy();
        PHY_SESSION_ACTIVE.store(true, Ordering::Release);

        Self {
            _init_guard: init_guard,
            _clock_guard: clock_guard,
        }
    }

    /// Resets the Wi-Fi MAC core and coexistence state while holding an active
    /// PHY session. Caller must ensure RX/TX DMA is stopped first.
    #[inline]
    pub fn reset_mac_and_coex(&mut self) -> Result<(), PhyBridgeError> {
        phy_reset_mac_and_coex()
    }
}

#[cfg(target_arch = "xtensa")]
impl Drop for XtensaPhySession {
    fn drop(&mut self) {
        PHY_SESSION_ACTIVE.store(false, Ordering::Release);
    }
}

#[cfg(target_arch = "xtensa")]
impl Default for XtensaPhySession {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Cross-Platform Unified PHY Bridge API
// ============================================================================

/// Enables Wi-Fi reception in the PHY layer.
pub fn phy_enable_rx() {
    #[cfg(not(target_arch = "xtensa"))]
    {
        MOCK_RX_ENABLED.store(true, Ordering::SeqCst);
    }
    #[cfg(target_arch = "xtensa")]
    {
        with_mmio_rmw_lock(|| {
            esp_phy::enable_phy_with_wifi_rx();
        });
    }
}

/// Disables Wi-Fi reception in the PHY layer.
pub fn phy_disable_rx() {
    #[cfg(not(target_arch = "xtensa"))]
    {
        MOCK_RX_ENABLED.store(false, Ordering::SeqCst);
    }
    #[cfg(target_arch = "xtensa")]
    {
        with_mmio_rmw_lock(|| {
            esp_phy::disable_phy_with_wifi_rx();
        });
    }
}

/// Resets Wi-Fi hardware MAC, synchronizes coexistence, and clears RX DMA.
///
/// Restricted to crate-internal callers and gated on [`is_phy_session_active`]
/// so external code must hold an [`XtensaPhySession`] (`M-072`). Guards all
/// `DPORT_*` and `RX_CTRL_REG` RMWs with the cross-core/ISR ECO 3.10 spinlock
/// and holds the MAC reset pulse for a calibrated 10 us via `CCOUNT`.
///
/// # Errors
///
/// Returns [`PhyBridgeError::PhySessionInactive`] if no PHY session is active.
pub(crate) fn phy_reset_mac_and_coex() -> Result<(), PhyBridgeError> {
    if !is_phy_session_active() {
        return Err(PhyBridgeError::PhySessionInactive);
    }
    #[cfg(not(target_arch = "xtensa"))]
    {
        with_mmio_rmw_lock(|| {
            // Host simulation: exercises the lock path cleanly.
        });
    }
    #[cfg(target_arch = "xtensa")]
    {
        with_mmio_rmw_lock(|| {
            // JUSTIFICATION: Hardware Wi-Fi MAC core reset, clock gating, and
            // RX DMA state clearing require direct MMIO writes and fences
            // under ECO 3.10 interrupt-masked cross-core serialization.
            // SAFETY: DPORT_WIFI_CLK_EN_REG, DPORT_CORE_RST_EN_REG, and
            // RX_CTRL_REG are valid 4-byte aligned ESP32 hardware MMIO
            // registers. `coex_bt_high_prio` is serialized under the lock.
            unsafe {
                let clk_ptr = DPORT_WIFI_CLK_EN_REG as *mut u32;
                let clk = core::ptr::read_volatile(clk_ptr);
                core::ptr::write_volatile(
                    clk_ptr,
                    clk | (DPORT_WIFI_CLK_WIFI_EN_M | DPORT_WIFI_CLK_WIFI_BT_COMMON_M),
                );
                core::arch::asm!("memw", options(nostack, preserves_flags));

                let rst_ptr = DPORT_CORE_RST_EN_REG as *mut u32;
                let rst = core::ptr::read_volatile(rst_ptr);
                core::ptr::write_volatile(rst_ptr, rst | DPORT_WIFIMAC_RST);
                core::arch::asm!("memw", options(nostack, preserves_flags));

                // Hold reset for >= 10 us using non-elidable CCOUNT loop.
                xtensa_delay_us_ccount(10);

                let rst_active = core::ptr::read_volatile(rst_ptr);
                core::ptr::write_volatile(rst_ptr, rst_active & !DPORT_WIFIMAC_RST);
                core::arch::asm!("memw", options(nostack, preserves_flags));

                coex_bt_high_prio();

                let rx_ctrl_ptr = RX_CTRL_REG as *mut u32;
                let rx_ctrl = core::ptr::read_volatile(rx_ctrl_ptr);
                core::ptr::write_volatile(rx_ctrl_ptr, rx_ctrl & 0x7FFF_FFFF);
                core::arch::asm!("memw", options(nostack, preserves_flags));
            }
        });
    }
    Ok(())
}

/// Switches 2.4 GHz 20 MHz operating channel with AGC protection.
///
/// Disables Wi-Fi AGC, updates RF synthesizer channels and filter
/// coefficients, then restores AGC. Enforces the active [`RegulatoryDomain`]
/// channel ceiling (defaults to FCC 1..=11).
///
/// # Errors
///
/// Returns [`PhyBridgeError::InvalidChannel`] if `chan` is `0` or exceeds the
/// active regulatory domain's maximum channel.
pub fn phy_switch_channel_20mhz(chan: u8) -> Result<(), PhyBridgeError> {
    let max_chan = phy_regulatory_domain().max_channel();
    if chan == 0 || chan > max_chan {
        return Err(PhyBridgeError::InvalidChannel(chan));
    }

    #[cfg(not(target_arch = "xtensa"))]
    {
        MOCK_AGC_ENABLED.store(false, Ordering::SeqCst);
        MOCK_CHANNEL.store(chan, Ordering::SeqCst);
        MOCK_AGC_ENABLED.store(true, Ordering::SeqCst);
    }

    #[cfg(target_arch = "xtensa")]
    {
        with_mmio_rmw_lock(|| {
            // JUSTIFICATION: libphy.a channel configuration requires disabling
            // AGC, updating RF channels/filters, and re-enabling AGC in order.
            // SAFETY: Serialized via `with_mmio_rmw_lock`; `chan` is validated
            // within `1..=max_chan`, and `0` selects 20 MHz HT20 (`chan_type=0`)
            // matching `chip_v7_set_chan_nomac(chan, 0)`.
            unsafe {
                disable_wifi_agc();
                phy_change_channel_nomac(chan, 0, 0, 0);
                chip_v7_set_chan_misc(chan, 0);
                phy_chan_filt_set(chan, 0);
                enable_wifi_agc();
            }
        });
    }

    Ok(())
}

/// Configures the baseband hardware watchdog interrupt enable state.
pub fn phy_arm_bb_wdt(enable: bool) {
    #[cfg(not(target_arch = "xtensa"))]
    {
        MOCK_BB_WDT_ENABLED.store(enable, Ordering::SeqCst);
    }
    #[cfg(target_arch = "xtensa")]
    {
        let en_val = if enable { 1_u32 } else { 0_u32 };
        with_mmio_rmw_lock(|| {
            // JUSTIFICATION: Configures hardware baseband watchdog in libphy.
            // SAFETY: `bb_wdt_int_enable` accepts 0 or 1 and is serialized.
            unsafe {
                bb_wdt_int_enable(en_val);
            }
        });
    }
}

/// Sets maximum RF transmit power ceiling in quarter-dBm units, clamped to
/// safe ESP32 PA bounds ([`MIN_TX_POWER_QDBM`]..=[`MAX_TX_POWER_QDBM`]).
pub fn phy_set_max_tx_power_qdbm(power_qdbm: i8) {
    let clamped = power_qdbm.clamp(MIN_TX_POWER_QDBM, MAX_TX_POWER_QDBM);
    #[cfg(not(target_arch = "xtensa"))]
    {
        MOCK_MAX_TX_POWER_QDBM.store(clamped, Ordering::SeqCst);
    }
    #[cfg(target_arch = "xtensa")]
    {
        with_mmio_rmw_lock(|| {
            // JUSTIFICATION: Programs maximum RF TX power ceiling in libphy.
            // SAFETY: `clamped` is bounded within ESP-IDF PA limits 8..=84 qdBm
            // and serialized under `with_mmio_rmw_lock`.
            unsafe {
                phy_set_most_tpw(clamped);
            }
        });
    }
}

/// Disables legacy 802.11b 1Mbps and 2Mbps CCK low data rates.
pub fn phy_disable_11b_low_rate() {
    #[cfg(not(target_arch = "xtensa"))]
    {
        MOCK_LOW_RATE_DISABLED.store(true, Ordering::SeqCst);
    }
    #[cfg(target_arch = "xtensa")]
    {
        with_mmio_rmw_lock(|| {
            // JUSTIFICATION: Disables 802.11b CCK low rates to save airtime.
            // SAFETY: `phy_disable_low_rate` takes no args and is serialized.
            unsafe {
                phy_disable_low_rate();
            }
        });
    }
}

/// Enables or disables legacy 802.11b 1Mbps/2Mbps CCK low rates.
pub fn phy_set_low_rate_enabled(enabled: bool) {
    #[cfg(not(target_arch = "xtensa"))]
    {
        MOCK_LOW_RATE_ENABLED.store(enabled, Ordering::SeqCst);
    }
    #[cfg(target_arch = "xtensa")]
    {
        with_mmio_rmw_lock(|| {
            // JUSTIFICATION: Controls 802.11b CCK low-rate state in libphy.
            // SAFETY: Both FFI symbols take no args and are serialized.
            unsafe {
                if enabled {
                    phy_enable_low_rate();
                } else {
                    phy_disable_low_rate();
                }
            }
        });
    }
}

/// Runs a single background TX power control recalibration cycle.
pub fn phy_run_tx_pwctrl_bg() {
    #[cfg(not(target_arch = "xtensa"))]
    {
        MOCK_PWCTRL_BG_CALLS.fetch_add(1, Ordering::SeqCst);
    }
    #[cfg(target_arch = "xtensa")]
    {
        with_mmio_rmw_lock(|| {
            // JUSTIFICATION: Performs background TX power calibration against
            // thermal drift in libphy.
            // SAFETY: Fixed valid control flags (en=1, cal=0), serialized.
            unsafe {
                tx_pwctrl_background(1, 0);
            }
        });
    }
}

// ============================================================================
// Unit Tests
// ============================================================================

#[cfg(test)]
pub(crate) mod test_sync {
    extern crate std;

    static HOST_PHY_TEST_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

    pub(crate) fn lock_host_phy() -> std::sync::MutexGuard<'static, ()> {
        HOST_PHY_TEST_LOCK
            .lock()
            .expect("host phy test lock not poisoned")
    }
}

#[cfg(test)]
mod tests {
    use super::test_sync::lock_host_phy;
    use super::*;

    #[test]
    fn test_channel_validation_default_etsi_and_domains() {
        let _guard = lock_host_phy();
        host_mock_reset();

        assert_eq!(phy_regulatory_domain(), RegulatoryDomain::Etsi);
        assert!(matches!(
            phy_switch_channel_20mhz(0),
            Err(PhyBridgeError::InvalidChannel(0))
        ));
        assert!(phy_switch_channel_20mhz(13).is_ok());
        assert!(matches!(
            phy_switch_channel_20mhz(14),
            Err(PhyBridgeError::InvalidChannel(14))
        ));
        assert!(matches!(
            phy_switch_channel_20mhz(15),
            Err(PhyBridgeError::InvalidChannel(15))
        ));
        assert!(matches!(
            phy_switch_channel_20mhz(255),
            Err(PhyBridgeError::InvalidChannel(255))
        ));

        // FCC restricts to 1..=11 and rejects 12.
        phy_set_regulatory_domain(RegulatoryDomain::Fcc);
        assert!(phy_switch_channel_20mhz(11).is_ok());
        assert!(matches!(
            phy_switch_channel_20mhz(12),
            Err(PhyBridgeError::InvalidChannel(12))
        ));

        // Japan allows 1..=14, rejects 15.
        phy_set_regulatory_domain(RegulatoryDomain::Japan);
        assert!(phy_switch_channel_20mhz(14).is_ok());
        assert!(matches!(
            phy_switch_channel_20mhz(15),
            Err(PhyBridgeError::InvalidChannel(15))
        ));

        host_mock_reset();
    }

    #[test]
    fn test_channel_switching_and_agc_invariant() {
        let _guard = lock_host_phy();
        host_mock_reset();

        for chan in 1..=11 {
            assert!(phy_switch_channel_20mhz(chan).is_ok());
            let snap = host_mock_snapshot();
            assert_eq!(snap.channel, chan);
            assert!(snap.agc_enabled);
        }
    }

    #[test]
    fn test_rx_enable_disable() {
        let _guard = lock_host_phy();
        host_mock_reset();

        let initial = host_mock_snapshot();
        assert!(!initial.rx_enabled);

        phy_enable_rx();
        assert!(host_mock_snapshot().rx_enabled);

        phy_disable_rx();
        assert!(!host_mock_snapshot().rx_enabled);
    }

    #[test]
    fn test_bb_wdt_arming() {
        let _guard = lock_host_phy();
        host_mock_reset();

        assert!(!host_mock_snapshot().bb_wdt_enabled);

        phy_arm_bb_wdt(true);
        assert!(host_mock_snapshot().bb_wdt_enabled);

        phy_arm_bb_wdt(false);
        assert!(!host_mock_snapshot().bb_wdt_enabled);
    }

    #[test]
    fn test_max_tx_power_clamped() {
        let _guard = lock_host_phy();
        host_mock_reset();

        phy_set_max_tx_power_qdbm(78);
        assert_eq!(host_mock_snapshot().max_tx_power_qdbm, 78);

        phy_set_max_tx_power_qdbm(-10);
        assert_eq!(host_mock_snapshot().max_tx_power_qdbm, MIN_TX_POWER_QDBM);

        phy_set_max_tx_power_qdbm(120);
        assert_eq!(host_mock_snapshot().max_tx_power_qdbm, MAX_TX_POWER_QDBM);
    }

    #[test]
    fn test_low_rate_disable() {
        let _guard = lock_host_phy();
        host_mock_reset();

        assert!(!host_mock_snapshot().low_rate_disabled);

        phy_disable_11b_low_rate();
        assert!(host_mock_snapshot().low_rate_disabled);
    }

    #[test]
    fn test_tx_pwctrl_counter() {
        let _guard = lock_host_phy();
        host_mock_reset();

        assert_eq!(host_mock_snapshot().pwctrl_bg_calls, 0);

        for expected in 1..=5 {
            phy_run_tx_pwctrl_bg();
            assert_eq!(host_mock_snapshot().pwctrl_bg_calls, expected);
        }
    }

    #[test]
    fn test_phy_enabled_host_mock() {
        let _guard = lock_host_phy();
        host_mock_reset();

        assert!(!host_mock_snapshot().phy_enabled);
        assert!(!is_phy_session_active());

        host_mock_set_phy_enabled(true);
        assert!(host_mock_snapshot().phy_enabled);
        assert!(is_phy_session_active());

        host_mock_set_phy_enabled(false);
        assert!(!host_mock_snapshot().phy_enabled);
        assert!(!is_phy_session_active());
    }

    #[test]
    fn test_error_display_and_traits() {
        let err = PhyBridgeError::InvalidChannel(77);
        let cloned = err;
        assert_eq!(err, cloned);
        assert_eq!(err, PhyBridgeError::InvalidChannel(77));
        assert_ne!(err, PhyBridgeError::PhySessionInactive);
    }

    #[test]
    fn test_phy_reset_mac_and_coex_requires_active_session() {
        let _guard = lock_host_phy();
        host_mock_reset();

        // M-072: Must fail when no PHY session is active.
        assert_eq!(
            phy_reset_mac_and_coex(),
            Err(PhyBridgeError::PhySessionInactive)
        );

        host_mock_set_phy_enabled(true);
        assert_eq!(phy_reset_mac_and_coex(), Ok(()));

        host_mock_reset();
    }

    #[test]
    fn test_phy_set_low_rate_enabled() {
        let _guard = lock_host_phy();
        host_mock_reset();

        assert!(!host_mock_is_low_rate_enabled());
        phy_set_low_rate_enabled(true);
        assert!(host_mock_is_low_rate_enabled());
        phy_set_low_rate_enabled(false);
        assert!(!host_mock_is_low_rate_enabled());
    }
}
