//! ESP32 Wi-Fi PHY abstraction bridge and host simulation fixture.
//!
//! Bridges low-level Wi-Fi physical layer (PHY) control routines provided by
//! `esp-phy` and `libphy.a` on Xtensa targets, and exposes a pure-Rust atomic
//! mock state machine for unit tests and simulation on host targets.

use core::fmt;

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
use core::sync::atomic::AtomicBool;
#[cfg(not(target_arch = "xtensa"))]
use core::sync::atomic::AtomicI8;
#[cfg(not(target_arch = "xtensa"))]
use core::sync::atomic::AtomicU32;
#[cfg(not(target_arch = "xtensa"))]
use core::sync::atomic::AtomicU8;
#[cfg(not(target_arch = "xtensa"))]
use core::sync::atomic::Ordering;

/// Errors returned by Wi-Fi PHY bridge operations.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PhyBridgeError {
    /// Specified channel is outside the valid 2.4 GHz range (1..=14).
    InvalidChannel(u8),
}

impl fmt::Display for PhyBridgeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match *self {
            Self::InvalidChannel(chan) => {
                write!(f, "invalid Wi-Fi 2.4 GHz channel: {chan} (must be 1..=14)")
            }
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
    MOCK_MAX_TX_POWER_QDBM.store(0, Ordering::SeqCst);
    MOCK_BB_WDT_ENABLED.store(false, Ordering::SeqCst);
    MOCK_LOW_RATE_DISABLED.store(false, Ordering::SeqCst);
    MOCK_LOW_RATE_ENABLED.store(false, Ordering::SeqCst);
    MOCK_PWCTRL_BG_CALLS.store(0, Ordering::SeqCst);
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
}

// ============================================================================
// Xtensa Hardware FFI & Guards (target_arch = "xtensa")
// ============================================================================

#[cfg(target_arch = "xtensa")]
extern "C" {
    fn phy_change_channel_nomac(chan: u8, bw20: u8, zero: u8, ext_chan: i32);
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

        // JUSTIFICATION: DPORT Wi-Fi and Bluetooth common clock gating
        // requires volatile MMIO read-modify-write cycles and an Xtensa write fence.
        // SAFETY: DPORT_WIFI_CLK_EN_REG is a valid 4-byte aligned ESP32 MMIO
        // register (soc/dport_reg.h). asm!("memw") drains write buffers to the
        // hardware peripheral bus.
        unsafe {
            let clk_ptr = DPORT_WIFI_CLK_EN_REG as *mut u32;
            let clk = core::ptr::read_volatile(clk_ptr);
            core::ptr::write_volatile(
                clk_ptr,
                clk | (DPORT_WIFI_CLK_WIFI_EN_M | DPORT_WIFI_CLK_WIFI_BT_COMMON_M),
            );
            core::arch::asm!("memw");
        }

        let clock_guard = esp_phy::enable_phy_clock();
        let init_guard = esp_phy::enable_phy();

        Self {
            _init_guard: init_guard,
            _clock_guard: clock_guard,
        }
    }
}

#[cfg(target_arch = "xtensa")]
impl Default for XtensaPhySession {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(target_arch = "xtensa")]
/// Enables hardware PHY, returning an RAII initialization guard.
#[inline]
pub fn phy_init() -> esp_phy::PhyInitGuard<'static> {
    esp_phy::enable_phy()
}

#[cfg(target_arch = "xtensa")]
/// Enables hardware PHY clock, returning an RAII clock guard.
#[inline]
pub fn phy_clock() -> esp_phy::PhyClockGuard<'static> {
    esp_phy::enable_phy_clock()
}

#[cfg(target_arch = "xtensa")]
/// Disables hardware PHY.
#[inline]
pub fn phy_shutdown() {
    esp_phy::disable_phy();
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
        esp_phy::enable_phy_with_wifi_rx();
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
        esp_phy::disable_phy_with_wifi_rx();
    }
}

/// Resets Wi-Fi hardware MAC, synchronizes coexistence, and clears RX DMA.
pub fn phy_reset_mac_and_coex() {
    #[cfg(not(target_arch = "xtensa"))]
    {
        // No-op in host simulation mode.
    }
    #[cfg(target_arch = "xtensa")]
    {
        // JUSTIFICATION: Hardware Wi-Fi MAC core reset, clock gating, and RX
        // DMA state clearing require direct MMIO writes and memory fences.
        // SAFETY: DPORT_WIFI_CLK_EN_REG, DPORT_CORE_RST_EN_REG, and
        // RX_CTRL_REG are valid 4-byte aligned ESP32 hardware MMIO registers
        // (soc/dport_reg.h). coex_bt_high_prio is an authoritative C-ABI
        // coexistence helper. asm!("memw") drains peripheral bus writes.
        unsafe {
            let clk_ptr = DPORT_WIFI_CLK_EN_REG as *mut u32;
            let clk = core::ptr::read_volatile(clk_ptr);
            core::ptr::write_volatile(
                clk_ptr,
                clk | (DPORT_WIFI_CLK_WIFI_EN_M | DPORT_WIFI_CLK_WIFI_BT_COMMON_M),
            );
            core::arch::asm!("memw");

            let rst_ptr = DPORT_CORE_RST_EN_REG as *mut u32;
            let rst = core::ptr::read_volatile(rst_ptr);
            core::ptr::write_volatile(rst_ptr, rst | DPORT_WIFIMAC_RST);
            core::arch::asm!("memw");

            for _ in 0..64 {
                core::hint::spin_loop();
            }

            let rst_active = core::ptr::read_volatile(rst_ptr);
            core::ptr::write_volatile(rst_ptr, rst_active & !DPORT_WIFIMAC_RST);
            core::arch::asm!("memw");

            coex_bt_high_prio();

            let rx_ctrl_ptr = RX_CTRL_REG as *mut u32;
            let rx_ctrl = core::ptr::read_volatile(rx_ctrl_ptr);
            core::ptr::write_volatile(rx_ctrl_ptr, rx_ctrl & 0x7FFF_FFFF);
            core::arch::asm!("memw");
        }
    }
}

/// Switches 2.4 GHz 20 MHz operating channel with AGC protection.
///
/// Disables Wi-Fi AGC, updates RF synthesizer channels and filter
/// coefficients, then restores AGC.
///
/// # Errors
///
/// Returns [`PhyBridgeError::InvalidChannel`] if `chan` is outside 1..=14.
pub fn phy_switch_channel_20mhz(chan: u8) -> Result<(), PhyBridgeError> {
    if !(1..=14).contains(&chan) {
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
        // JUSTIFICATION: libphy.a channel configuration requires disabling AGC,
        // updating RF synthesizer channels and filter coefficients, and
        // re-enabling AGC in strict sequential order.
        // SAFETY: All functions are thread-safe within the PHY state machine;
        // chan is validated in 1..=14 above, and bw parameters are valid constants.
        unsafe {
            disable_wifi_agc();
            phy_change_channel_nomac(chan, 1, 0, 0);
            chip_v7_set_chan_misc(chan, 0);
            phy_chan_filt_set(chan, 0);
            enable_wifi_agc();
        }
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
        // JUSTIFICATION: Configures hardware baseband watchdog in libphy.
        // SAFETY: bb_wdt_int_enable accepts flag 0 or 1, safely passed as u32.
        unsafe {
            bb_wdt_int_enable(en_val);
        }
    }
}

/// Sets maximum RF transmit power ceiling in quarter-dBm units.
pub fn phy_set_max_tx_power_qdbm(power_qdbm: i8) {
    #[cfg(not(target_arch = "xtensa"))]
    {
        MOCK_MAX_TX_POWER_QDBM.store(power_qdbm, Ordering::SeqCst);
    }
    #[cfg(target_arch = "xtensa")]
    {
        // JUSTIFICATION: Programs maximum RF TX power ceiling in libphy.
        // SAFETY: phy_set_most_tpw takes signed 8-bit quarter-dBm clamped by PA.
        unsafe {
            phy_set_most_tpw(power_qdbm);
        }
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
        // JUSTIFICATION: Disables legacy 802.11b CCK rates to preserve airtime.
        // SAFETY: phy_disable_low_rate takes no parameters and mutates rate masks.
        unsafe {
            phy_disable_low_rate();
        }
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
        // JUSTIFICATION: Controls 802.11b CCK low-rate state in libphy.
        // SAFETY: Both FFI symbols take no args and update PHY masks.
        unsafe {
            if enabled {
                phy_enable_low_rate();
            } else {
                phy_disable_low_rate();
            }
        }
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
        // JUSTIFICATION: Performs background TX power calibration against
        // thermal drift in libphy.
        // SAFETY: Parameters are fixed valid control flags (en=1,
        // calibrate_now=0).
        unsafe {
            tx_pwctrl_background(1, 0);
        }
    }
}

// ============================================================================
// Unit Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_channel_validation_out_of_range() {
        host_mock_reset();

        assert!(matches!(
            phy_switch_channel_20mhz(0),
            Err(PhyBridgeError::InvalidChannel(0))
        ));
        assert!(matches!(
            phy_switch_channel_20mhz(15),
            Err(PhyBridgeError::InvalidChannel(15))
        ));
        assert!(matches!(
            phy_switch_channel_20mhz(255),
            Err(PhyBridgeError::InvalidChannel(255))
        ));

        let snap = host_mock_snapshot();
        assert_eq!(snap.channel, 1);
    }

    #[test]
    fn test_channel_switching_and_agc_invariant() {
        host_mock_reset();

        for chan in 1..=14 {
            assert!(phy_switch_channel_20mhz(chan).is_ok());
            let snap = host_mock_snapshot();
            assert_eq!(snap.channel, chan);
            assert!(snap.agc_enabled);
        }
    }

    #[test]
    fn test_rx_enable_disable() {
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
        host_mock_reset();

        assert!(!host_mock_snapshot().bb_wdt_enabled);

        phy_arm_bb_wdt(true);
        assert!(host_mock_snapshot().bb_wdt_enabled);

        phy_arm_bb_wdt(false);
        assert!(!host_mock_snapshot().bb_wdt_enabled);
    }

    #[test]
    fn test_max_tx_power() {
        host_mock_reset();

        phy_set_max_tx_power_qdbm(78);
        assert_eq!(host_mock_snapshot().max_tx_power_qdbm, 78);

        phy_set_max_tx_power_qdbm(-10);
        assert_eq!(host_mock_snapshot().max_tx_power_qdbm, -10);
    }

    #[test]
    fn test_low_rate_disable() {
        host_mock_reset();

        assert!(!host_mock_snapshot().low_rate_disabled);

        phy_disable_11b_low_rate();
        assert!(host_mock_snapshot().low_rate_disabled);
    }

    #[test]
    fn test_tx_pwctrl_counter() {
        host_mock_reset();

        assert_eq!(host_mock_snapshot().pwctrl_bg_calls, 0);

        for expected in 1..=5 {
            phy_run_tx_pwctrl_bg();
            assert_eq!(host_mock_snapshot().pwctrl_bg_calls, expected);
        }
    }

    #[test]
    fn test_phy_enabled_host_mock() {
        host_mock_reset();

        assert!(!host_mock_snapshot().phy_enabled);

        host_mock_set_phy_enabled(true);
        assert!(host_mock_snapshot().phy_enabled);

        host_mock_set_phy_enabled(false);
        assert!(!host_mock_snapshot().phy_enabled);
    }

    #[test]
    fn test_error_display_and_traits() {
        let err = PhyBridgeError::InvalidChannel(77);
        let cloned = err;
        assert_eq!(err, cloned);
        assert_eq!(err, PhyBridgeError::InvalidChannel(77));
    }

    #[test]
    fn test_phy_reset_mac_and_coex() {
        phy_reset_mac_and_coex();
    }

    #[test]
    fn test_phy_set_low_rate_enabled() {
        host_mock_reset();

        assert!(!host_mock_is_low_rate_enabled());
        phy_set_low_rate_enabled(true);
        assert!(host_mock_is_low_rate_enabled());
        phy_set_low_rate_enabled(false);
        assert!(!host_mock_is_low_rate_enabled());
    }
}
