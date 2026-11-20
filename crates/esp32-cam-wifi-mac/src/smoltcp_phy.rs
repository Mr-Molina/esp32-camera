//! smoltcp physical device adapter and high-level 802.11 STA Wi-Fi driver.
//!
//! Provides [`Esp32StaWifiDriver`] bridging hardware LMAC DMA rings and
//! IEEE 802.11 STA state management to [`smoltcp::phy::Device`].

use core::fmt::Display;
use core::fmt::Formatter;
use core::fmt::Result as FmtResult;
use core::marker::PhantomPinned;
use core::pin::Pin;

use crate::ap_fsm::ApConfig;
use crate::ap_fsm::ApError;
use crate::ap_fsm::ApMacCore;
use crate::ap_fsm::ApStats;
use crate::ap_fsm::ApTxSink;
use crate::dma_ring::DmaRingError;
use crate::dma_ring::RxPop;
use crate::dma_ring::WifiRxRing;
use crate::dma_ring::WifiTxSlot;
use crate::lmac_mmio::LmacError;
use crate::lmac_mmio::LmacHal;
use crate::lmac_mmio::WifiMmioBus;
use crate::phy_bridge::PhyBridgeError;
use crate::rate_ctrl::estimate_tx_duration_us;
use crate::regs::WifiPhyRate;
use crate::sta_fsm::ScanCandidate;
use crate::sta_fsm::StaConnectConfig;
use crate::sta_fsm::StaError;
use crate::sta_fsm::StaMacCore;
use crate::sta_fsm::StaState;
use crate::sta_fsm::StaTxSink;

/// Error type emitted by the Wi-Fi PHY device driver.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DriverError {
    /// DMA ring descriptor allocation or sequencing failure.
    Ring(DmaRingError),
    /// Low-level MAC register access or command failure.
    Lmac(LmacError),
    /// Baseband or RF PHY calibration/channel failure.
    Phy(PhyBridgeError),
    /// 802.11 STA state machine or protocol failure.
    Sta(StaError),
    /// 802.11 Open SoftAP state machine or protocol failure.
    Ap(ApError),
    /// The driver has not been initialized via [`Esp32StaWifiDriver::init`].
    NotInitialized,
}

impl Display for DriverError {
    fn fmt(&self, f: &mut Formatter<'_>) -> FmtResult {
        match self {
            Self::Ring(e) => write!(f, "DMA ring error: {e}"),
            Self::Lmac(e) => write!(f, "LMAC MMIO error: {e}"),
            Self::Phy(e) => write!(f, "PHY bridge error: {e}"),
            Self::Sta(e) => write!(f, "STA FSM error: {e}"),
            Self::Ap(e) => write!(f, "AP FSM error: {e}"),
            Self::NotInitialized => write!(f, "Wi-Fi driver not initialized"),
        }
    }
}

impl From<DmaRingError> for DriverError {
    fn from(err: DmaRingError) -> Self {
        Self::Ring(err)
    }
}

impl From<LmacError> for DriverError {
    fn from(err: LmacError) -> Self {
        Self::Lmac(err)
    }
}

impl From<PhyBridgeError> for DriverError {
    fn from(err: PhyBridgeError) -> Self {
        Self::Phy(err)
    }
}

impl From<StaError> for DriverError {
    fn from(err: StaError) -> Self {
        Self::Sta(err)
    }
}

impl From<ApError> for DriverError {
    fn from(err: ApError) -> Self {
        Self::Ap(err)
    }
}

/// smoltcp RX token holding an immutable Ethernet frame slice.
pub struct StaRxToken {
    buf: [u8; 1536],
    len: usize,
}

impl smoltcp::phy::RxToken for StaRxToken {
    fn consume<R, F: FnOnce(&[u8]) -> R>(self, f: F) -> R {
        let valid_len = self.len.min(self.buf.len());
        f(&self.buf[..valid_len])
    }
}

/// Maximum hardware RX MPDUs processed per [`Esp32StaWifiDriver::poll`] call
/// to prevent remote frame-flood WDT starvation.
pub const MAX_RX_PER_POLL: usize = 16;

/// Hardware `rx_state` (`-0x0b` in `wdev.o`: `wDev_ProcessRxSucData`) emitted
/// when an encrypted MPDU passes through for software crypto decryption.
const RX_STATE_SW_CRYPTO_PASSTHROUGH: u8 = 0xF5;

/// smoltcp TX token providing a staging buffer for an outgoing Ethernet frame.
pub struct StaTxToken<'a, B: WifiMmioBus> {
    driver: &'a mut Esp32StaWifiDriver<B>,
}

impl<'a, B: WifiMmioBus> smoltcp::phy::TxToken for StaTxToken<'a, B> {
    fn consume<R, F: FnOnce(&mut [u8]) -> R>(self, len: usize, f: F) -> R {
        if len > self.driver.smol_tx_buf.len() {
            self.driver.pending_eth_tx_failures =
                self.driver.pending_eth_tx_failures.saturating_add(1);
            return f(&mut []);
        }
        let result = f(&mut self.driver.smol_tx_buf[..len]);
        self.driver.transmit_smol_tx_buf(len);
        result
    }
}

/// Bridge adapter fulfilling [`StaTxSink`] via the driver's LMAC and TX slot.
struct DriverTxSink<'a, B: WifiMmioBus> {
    lmac: &'a mut LmacHal<B>,
    tx_slot: &'a mut WifiTxSlot<1600>,
}

impl<'a, B: WifiMmioBus> StaTxSink for DriverTxSink<'a, B> {
    fn send_mpdu(
        &mut self,
        mpdu: &[u8],
        rate: WifiPhyRate,
        wait_ack: bool,
        _now_ms: u64,
    ) -> Result<(), StaError> {
        // JUSTIFICATION: `DriverTxSink` is only created on an initialized
        // `Esp32StaWifiDriver`, whose `init` / `init_pinned` contract guarantees
        // `self.tx_slot` remains pinned in place until `shutdown()`.
        // SAFETY: `self.tx_slot` is never moved while TX DMA is in flight.
        let desc_addr = unsafe { self.tx_slot.tx_prepare(mpdu) }.map_err(|err| match err {
            DmaRingError::TxBusy => StaError::Hardware(LmacError::TxDrainFailed),
            _ => StaError::BufferTooSmall,
        })?;
        let dur = estimate_tx_duration_us(mpdu.len(), rate);
        let res = self
            .lmac
            .submit_and_poll_tx(desc_addr, mpdu, rate, dur, wait_ack, 50_000);
        // M-074: Only reclaim TX slot ownership when TX succeeded, was never
        // triggered, or was aborted and verified drained (`TX_ACTIVE == 0`).
        // Leave `in_flight = true` on `TxDrainFailed` so subsequent `tx_prepare`
        // fails safely with `TxBusy` instead of overwriting a live DMA buffer.
        if !matches!(res, Err(LmacError::TxDrainFailed)) {
            // JUSTIFICATION: LMAC confirmed TX finished or drained cleanly.
            // SAFETY: Hardware TX DMA is quiescent (`TX_ACTIVE == 0`).
            unsafe {
                self.tx_slot.mark_completed();
            }
        }
        res.map_err(StaError::Hardware)
    }

    fn on_channel_switch(&mut self, channel: u8) -> Result<(), StaError> {
        crate::phy_bridge::phy_switch_channel_20mhz(channel).map_err(StaError::Phy)
    }

    fn on_scan_rx_policy(&mut self) -> Result<(), StaError> {
        self.lmac.set_scan_rx_policy();
        Ok(())
    }

    fn on_associated_bssid(&mut self, bssid: [u8; 6]) -> Result<(), StaError> {
        self.lmac.set_associated_bssid(bssid);
        Ok(())
    }

    fn hw_random_u32(&mut self) -> Result<u32, StaError> {
        self.lmac.try_hw_random_u32().map_err(StaError::Hardware)
    }

    fn on_install_ccmp_key(
        &mut self,
        slot: u8,
        key_idx: u8,
        bssid: [u8; 6],
        key: &[u8; 16],
    ) -> Result<(), StaError> {
        self.lmac.install_sta_ccmp_key(slot, key_idx, bssid, key);
        Ok(())
    }

    fn on_clear_crypto_keys(&mut self) -> Result<(), StaError> {
        self.lmac.clear_sta_ccmp_keys();
        Ok(())
    }
}

/// Bridge adapter fulfilling [`ApTxSink`] via the driver's LMAC and TX slot.
struct ApDriverTxSink<'a, B: WifiMmioBus> {
    lmac: &'a mut LmacHal<B>,
    tx_slot: &'a mut WifiTxSlot<1600>,
}

impl<'a, B: WifiMmioBus> ApTxSink for ApDriverTxSink<'a, B> {
    fn send_mpdu(&mut self, mpdu: &[u8], rate: WifiPhyRate, wait_ack: bool) -> Result<(), ApError> {
        // JUSTIFICATION: `ApDriverTxSink` is only created on an initialized
        // `Esp32StaWifiDriver`, whose `init` / `init_pinned` contract guarantees
        // `self.tx_slot` remains pinned in place until `shutdown()`.
        // SAFETY: `self.tx_slot` is never moved while TX DMA is in flight.
        let desc_addr = unsafe { self.tx_slot.tx_prepare(mpdu) }.map_err(|err| match err {
            DmaRingError::TxBusy => ApError::Hardware(LmacError::TxDrainFailed),
            _ => ApError::BufferTooSmall,
        })?;
        let dur = estimate_tx_duration_us(mpdu.len(), rate);
        let res = self
            .lmac
            .submit_and_poll_tx(desc_addr, mpdu, rate, dur, wait_ack, 50_000);
        // M-074: Only reclaim TX slot ownership when TX succeeded or drained
        // cleanly (`TX_ACTIVE == 0`). Leave `in_flight = true` on
        // `TxDrainFailed` so subsequent TX cannot overwrite live DMA memory.
        if !matches!(res, Err(LmacError::TxDrainFailed)) {
            // JUSTIFICATION: LMAC confirmed TX finished or drained cleanly.
            // SAFETY: Hardware TX DMA is quiescent (`TX_ACTIVE == 0`).
            unsafe {
                self.tx_slot.mark_completed();
            }
        }
        res.map_err(ApError::Hardware)
    }
}

/// High-level ESP32 802.11 STA Wi-Fi driver and smoltcp PHY device.
///
/// Marked `!Unpin` (`PhantomPinned`) because its embedded RX DMA ring and TX
/// DMA slot contain self-referential physical DMA addresses once initialized.
#[repr(C, align(4))]
pub struct Esp32StaWifiDriver<B: WifiMmioBus> {
    lmac: LmacHal<B>,
    rx_ring: WifiRxRing<8, 1600>,
    tx_slot: WifiTxSlot<1600>,
    sta: StaMacCore,
    ap: Option<ApMacCore>,
    rx_eth_queue: [[u8; 1536]; 8],
    rx_eth_lens: [u16; 8],
    rx_head: usize,
    rx_tail: usize,
    rx_count: usize,
    rx_mpdu_scratch: [u8; 1600],
    smol_tx_buf: [u8; 1536],
    now_ms: u64,
    initialized: bool,
    pending_eth_tx_failures: u32,
    eth_tx_ok: u32,
    rx_dropped_overflow: u32,
    last_sta_error: Option<StaError>,
    last_connected_rx_info: Option<(u8, u8, u16)>,
    rx_popped_total: u32,
    rx_malformed_total: u32,
    last_non_beacon_rx: Option<(u8, u8, u8, u16)>,
    rx_protected_total: u32,
    last_protected_rx_info: Option<(u8, u8)>,
    _pin: PhantomPinned,
}

impl<B: WifiMmioBus> Esp32StaWifiDriver<B> {
    /// Creates an uninitialized Wi-Fi driver bound to the specified MMIO bus.
    pub const fn new(bus: B) -> Self {
        Self {
            lmac: LmacHal::new(bus),
            rx_ring: WifiRxRing::new(),
            tx_slot: WifiTxSlot::new(),
            sta: StaMacCore::new([0; 6]),
            ap: None,
            rx_eth_queue: [[0u8; 1536]; 8],
            rx_eth_lens: [0u16; 8],
            rx_head: 0,
            rx_tail: 0,
            rx_count: 0,
            rx_mpdu_scratch: [0u8; 1600],
            smol_tx_buf: [0u8; 1536],
            now_ms: 0,
            initialized: false,
            pending_eth_tx_failures: 0,
            eth_tx_ok: 0,
            rx_dropped_overflow: 0,
            last_sta_error: None,
            last_connected_rx_info: None,
            rx_popped_total: 0,
            rx_malformed_total: 0,
            last_non_beacon_rx: None,
            rx_protected_total: 0,
            last_protected_rx_info: None,
            _pin: PhantomPinned,
        }
    }

    /// Initializes hardware MAC, DMA rings, and baseband PHY on a pinned driver.
    #[inline(never)]
    pub fn init_pinned(
        self: Pin<&mut Self>,
        custom_mac: Option<[u8; 6]>,
    ) -> Result<[u8; 6], DriverError> {
        // JUSTIFICATION: `init` arms self-referential DMA descriptors in-place
        // without moving `self`.
        // SAFETY: `self: Pin<&mut Self>` enforces address stability for DMA.
        let this = unsafe { self.get_unchecked_mut() };
        // JUSTIFICATION: Pinned storage satisfies `init`'s fixed-address contract.
        // SAFETY: `this` is pinned and will not be relocated while DMA is active.
        unsafe { this.init(custom_mac) }
    }

    /// Initializes hardware MAC, DMA rings, and baseband PHY.
    ///
    /// Prefer [`Self::init_pinned`] when holding a `Pin<&mut Self>`.
    ///
    /// # Safety
    ///
    /// Caller MUST guarantee that `self` remains at a fixed memory address
    /// (never moved or relocated while initialized and until [`Self::shutdown`]
    /// stops hardware DMA), e.g., when stored in a `'static` slot (`M-014`).
    // JUSTIFICATION: Initializes self-referential hardware RX/TX DMA rings;
    // `unsafe` enforces address stability when `&mut self` is used instead of
    // `init_pinned(Pin<&mut Self>)`.
    #[inline(never)]
    pub unsafe fn init(&mut self, custom_mac: Option<[u8; 6]>) -> Result<[u8; 6], DriverError> {
        if !crate::phy_bridge::is_phy_session_active() {
            return Err(DriverError::Phy(PhyBridgeError::PhySessionInactive));
        }
        // JUSTIFICATION: Caller upholds the pinned-address contract of `init`.
        // SAFETY: `self.rx_ring` stays at a stable physical address until `shutdown`.
        unsafe {
            self.rx_ring.arm_ring()?;
        }
        if let Err(err) = crate::phy_bridge::phy_reset_mac_and_coex() {
            crate::phy_bridge::phy_disable_rx();
            return Err(DriverError::Phy(err));
        }
        let mac = match custom_mac {
            Some(m) => m,
            None => self.lmac.read_efuse_sta_mac(),
        };
        self.sta.reset_with_mac(mac);
        if let Err(err) = self.lmac.init_mac(mac, self.rx_ring.base_desc_addr()) {
            crate::phy_bridge::phy_disable_rx();
            return Err(DriverError::Lmac(err));
        }
        // Enable PHY RX after MAC reset and `init_mac` so `DPORT_WIFIMAC_RST`
        // cannot wipe `phy_wifi_enable_set(1)` state (`esp32-open-mac` hwinit).
        crate::phy_bridge::phy_enable_rx();
        // Tune Channel 1 after `init_mac` ungates `0x3CB8` MAC clocks, matching
        // `esp32-open-mac` / `esp-wifi-hal`.
        if let Err(err) = crate::phy_bridge::phy_switch_channel_20mhz(1) {
            crate::phy_bridge::phy_disable_rx();
            self.lmac.deinit_mac();
            return Err(DriverError::Phy(err));
        }
        crate::phy_bridge::phy_set_low_rate_enabled(true);
        self.lmac.enable_low_rate();
        crate::phy_bridge::phy_arm_bb_wdt(true);
        crate::phy_bridge::phy_set_max_tx_power_qdbm(78);
        crate::phy_bridge::phy_run_tx_pwctrl_bg();
        self.initialized = true;
        Ok(mac)
    }

    /// Precomputes and caches the WPA2 PMK for `config` ahead of [`Self::connect`].
    pub fn warm_pmk_cache(&mut self, config: &StaConnectConfig) -> Result<(), DriverError> {
        self.sta.precompute_pmk(config).map_err(DriverError::Sta)
    }

    /// Initiates association to an access point with the given configuration.
    pub fn connect(&mut self, config: StaConnectConfig, now_ms: u64) -> Result<(), DriverError> {
        if !self.initialized {
            return Err(DriverError::NotInitialized);
        }
        self.last_sta_error = None;
        self.last_connected_rx_info = None;
        self.last_non_beacon_rx = None;
        self.now_ms = now_ms;
        let mut sink = DriverTxSink {
            lmac: &mut self.lmac,
            tx_slot: &mut self.tx_slot,
        };
        self.sta.start_connect(config, now_ms, &mut sink)?;
        Ok(())
    }

    /// Disconnects from the current access point.
    pub fn disconnect(&mut self, reason: u16, now_ms: u64) -> Result<(), DriverError> {
        if !self.initialized {
            return Err(DriverError::NotInitialized);
        }
        self.now_ms = now_ms;
        let mut sink = DriverTxSink {
            lmac: &mut self.lmac,
            tx_slot: &mut self.tx_slot,
        };
        self.sta.disconnect(reason, now_ms, &mut sink)?;
        Ok(())
    }

    /// Switches the shared-DMA driver into pure-Rust Open SoftAP mode.
    ///
    /// Reuses the existing `LmacHal`, `WifiRxRing`, and `WifiTxSlot` without
    /// allocating extra DMA buffers, tunes the PHY to `cfg.channel`, and arms
    /// promiscuous-BSSID scan RX policy so wildcard Probe Requests reach
    /// [`ApMacCore`].
    ///
    /// # Errors
    /// Returns [`DriverError::NotInitialized`] if [`Self::init`] has not run,
    /// [`DriverError::Ap`] if `cfg` is invalid, or [`DriverError::Phy`] if
    /// channel tuning fails.
    pub fn enter_ap_mode(&mut self, cfg: ApConfig) -> Result<(), DriverError> {
        if !self.initialized {
            return Err(DriverError::NotInitialized);
        }
        let ap = ApMacCore::new(cfg)?;
        crate::phy_bridge::phy_switch_channel_20mhz(cfg.channel)?;
        self.lmac.set_scan_rx_policy();
        self.rx_head = 0;
        self.rx_tail = 0;
        self.rx_count = 0;
        self.ap = Some(ap);
        Ok(())
    }

    /// Exits Open SoftAP mode, sending best-effort Deauth frames to connected
    /// stations and restoring STA mode on the shared DMA driver.
    pub fn exit_ap_mode(&mut self) {
        if let Some(mut ap) = self.ap.take() {
            if self.initialized {
                let mut sink = ApDriverTxSink {
                    lmac: &mut self.lmac,
                    tx_slot: &mut self.tx_slot,
                };
                ap.deauth_all(&mut sink);
            }
            self.rx_head = 0;
            self.rx_tail = 0;
            self.rx_count = 0;
        }
    }

    /// Returns `true` when the driver is currently operating in SoftAP mode.
    #[must_use]
    pub const fn is_ap_mode(&self) -> bool {
        self.ap.is_some()
    }

    /// Returns the number of stations currently associated with the SoftAP,
    /// or `0` when operating in STA mode.
    #[must_use]
    pub fn ap_associated_clients(&self) -> usize {
        self.ap
            .as_ref()
            .map_or(0, ApMacCore::associated_client_count)
    }

    /// Returns the timestamp (in ms) of the last SoftAP client activity,
    /// or `0` when operating in STA mode.
    #[must_use]
    pub const fn ap_last_activity_ms(&self) -> u64 {
        match &self.ap {
            Some(ap) => ap.last_activity_ms(),
            None => 0,
        }
    }

    /// Returns a snapshot of the SoftAP TX/RX diagnostic counters, or `None`
    /// when operating in STA mode.
    #[must_use]
    pub fn ap_stats(&self) -> Option<ApStats> {
        self.ap.as_ref().map(ApMacCore::stats)
    }

    /// Polls RX descriptors, handles incoming frames, and advances timers.
    pub fn poll(&mut self, now_ms: u64) -> Result<(), DriverError> {
        if !self.initialized {
            return Err(DriverError::NotInitialized);
        }
        self.now_ms = now_ms;

        let irq = self.lmac.poll_and_clear_irqs();
        if irq.panic_wdt {
            crate::phy_bridge::phy_arm_bb_wdt(true);
        }

        let mut processed = 0usize;
        while processed < MAX_RX_PER_POLL {
            let (ctrl, mpdu) = match self.rx_ring.pop_received() {
                RxPop::Empty => break,
                RxPop::Malformed => {
                    // M-071/M-089: Always recycle malformed SW-owned descriptors
                    // (e.g., multi-descriptor jumbo frames or DMA runts) so a
                    // single bad descriptor can never wedge the RX ring head.
                    processed += 1;
                    self.rx_malformed_total = self.rx_malformed_total.saturating_add(1);
                    let _ = self.rx_ring.recycle_current();
                    self.lmac.reload_rx_dma();
                    continue;
                }
                RxPop::Frame(ctrl, mpdu) => (ctrl, mpdu),
            };
            processed += 1;
            self.rx_popped_total = self.rx_popped_total.saturating_add(1);

            let sig_len = usize::from(ctrl.sig_len);
            let mpdu_len = mpdu.len();
            if mpdu_len >= 2 && (mpdu[1] & 0x40) != 0 {
                self.rx_protected_total = self.rx_protected_total.saturating_add(1);
                self.last_protected_rx_info = Some((ctrl.rx_state, mpdu[0]));
            }
            if self.sta.is_connected() && mpdu_len >= 10 && mpdu[0] != 0x80 {
                self.last_non_beacon_rx = Some((ctrl.rx_state, mpdu[0], mpdu[4], ctrl.sig_len));
            }
            if self.sta.is_connected()
                && mpdu_len >= 10
                && (mpdu[4..10] == self.sta.sta_mac()
                    || ((mpdu[4] & 1) != 0 && ((mpdu[0] >> 2) & 0x03) == 2))
            {
                self.last_connected_rx_info = Some((ctrl.rx_state, mpdu[0], ctrl.sig_len));
            }
            let payload_no_fcs_len = sig_len.saturating_sub(4);
            // Accept both silicon RX DMA buffers (which strip the 4-byte FCS
            // and 4-byte-pad the payload, so `sig_len - 4 <= mpdu_len < sig_len`)
            // and mock buffers that retain the 4-byte FCS (`mpdu_len == sig_len`).
            // Also accept `RX_STATE_SW_CRYPTO_PASSTHROUGH` (0xF5) when the
            // 802.11 Protected bit (`mpdu[1] & 0x40`) is set so SW-decrypted
            // MPDUs are not dropped. Drop other RX errors, sig_len == 0
            // fallbacks, runts (<= 4 B), and oversized MPDUs (> scratch capacity).
            let rx_state_ok = ctrl.rx_state == 0
                || (ctrl.rx_state == RX_STATE_SW_CRYPTO_PASSTHROUGH
                    && mpdu_len >= 2
                    && (mpdu[1] & 0x40) != 0);
            let valid_frame = rx_state_ok
                && sig_len > 4
                && payload_no_fcs_len <= mpdu_len
                && mpdu_len <= sig_len + 3
                && payload_no_fcs_len <= self.rx_mpdu_scratch.len();
            if valid_frame {
                self.rx_mpdu_scratch[..payload_no_fcs_len]
                    .copy_from_slice(&mpdu[..payload_no_fcs_len]);
            }
            let _ = self.rx_ring.recycle_current();
            self.lmac.reload_rx_dma();

            if !valid_frame {
                continue;
            }

            let mpdu_no_fcs = &self.rx_mpdu_scratch[..payload_no_fcs_len];
            if let Some(ap) = &mut self.ap {
                let mut ap_sink = ApDriverTxSink {
                    lmac: &mut self.lmac,
                    tx_slot: &mut self.tx_slot,
                };
                if self.rx_count < 8 {
                    if let Some(eth_len) = ap.handle_rx_mpdu(
                        mpdu_no_fcs,
                        now_ms,
                        &mut ap_sink,
                        &mut self.rx_eth_queue[self.rx_tail],
                    ) {
                        self.rx_eth_lens[self.rx_tail] = eth_len as u16;
                        self.rx_tail = (self.rx_tail + 1) % 8;
                        self.rx_count += 1;
                    }
                } else {
                    let mut discard = [0u8; 0];
                    let _ = ap.handle_rx_mpdu(mpdu_no_fcs, now_ms, &mut ap_sink, &mut discard);
                    if mpdu_no_fcs.len() >= 24 && ((mpdu_no_fcs[0] >> 2) & 0x03) == 2 {
                        self.rx_dropped_overflow = self.rx_dropped_overflow.saturating_add(1);
                    }
                }
            } else {
                let hw_ccmp_decrypted = ctrl.rx_state == 0 && self.lmac.is_sta_hw_crypto_enabled();
                let mut sink = DriverTxSink {
                    lmac: &mut self.lmac,
                    tx_slot: &mut self.tx_slot,
                };
                if let Err(err) = self.sta.handle_rx_mpdu_ext(
                    mpdu_no_fcs,
                    ctrl.rssi,
                    hw_ccmp_decrypted,
                    now_ms,
                    &mut sink,
                    |eth| {
                        if eth.len() > 1536 {
                            self.rx_dropped_overflow = self.rx_dropped_overflow.saturating_add(1);
                            return;
                        }
                        if self.rx_count < 8 {
                            let copy_len = eth.len();
                            self.rx_eth_queue[self.rx_tail][..copy_len].copy_from_slice(eth);
                            self.rx_eth_lens[self.rx_tail] = copy_len as u16;
                            self.rx_tail = (self.rx_tail + 1) % 8;
                            self.rx_count += 1;
                        } else {
                            self.rx_dropped_overflow = self.rx_dropped_overflow.saturating_add(1);
                        }
                    },
                ) {
                    self.last_sta_error = Some(err);
                }
            }
        }

        if let Some(ap) = &mut self.ap {
            let mut ap_sink = ApDriverTxSink {
                lmac: &mut self.lmac,
                tx_slot: &mut self.tx_slot,
            };
            ap.poll_tick(now_ms, &mut ap_sink);
        } else {
            let mut sink = DriverTxSink {
                lmac: &mut self.lmac,
                tx_slot: &mut self.tx_slot,
            };
            if let Err(err) = self.sta.poll_tick(now_ms, &mut sink) {
                self.last_sta_error = Some(err);
                return Err(DriverError::Sta(err));
            }
        }

        Ok(())
    }

    /// Transmits an Ethernet frame staged in [`Esp32StaWifiDriver::smol_tx_buf`].
    pub fn transmit_smol_tx_buf(&mut self, len: usize) {
        if !self.initialized || len == 0 || len > self.smol_tx_buf.len() {
            self.pending_eth_tx_failures = self.pending_eth_tx_failures.saturating_add(1);
            return;
        }
        let tx_failed = if let Some(ap) = &mut self.ap {
            let mut ap_sink = ApDriverTxSink {
                lmac: &mut self.lmac,
                tx_slot: &mut self.tx_slot,
            };
            ap.transmit_eth(&self.smol_tx_buf[..len], &mut ap_sink)
                .is_err()
        } else {
            let mut sink = DriverTxSink {
                lmac: &mut self.lmac,
                tx_slot: &mut self.tx_slot,
            };
            match self
                .sta
                .transmit_eth(&self.smol_tx_buf[..len], self.now_ms, &mut sink)
            {
                Ok(()) => false,
                Err(e) => {
                    self.last_sta_error = Some(e);
                    true
                }
            }
        };
        if tx_failed {
            self.pending_eth_tx_failures = self.pending_eth_tx_failures.saturating_add(1);
        } else {
            self.eth_tx_ok = self.eth_tx_ok.saturating_add(1);
        }
    }

    /// Injects a mock MPDU into the driver receive pipeline for testing.
    pub fn push_mock_rx_mpdu(&mut self, mpdu: &[u8], rssi: i8) -> Result<(), DriverError> {
        if !self.initialized {
            return Err(DriverError::NotInitialized);
        }
        if let Some(ap) = &mut self.ap {
            let mut ap_sink = ApDriverTxSink {
                lmac: &mut self.lmac,
                tx_slot: &mut self.tx_slot,
            };
            if self.rx_count < 8 {
                if let Some(eth_len) = ap.handle_rx_mpdu(
                    mpdu,
                    self.now_ms,
                    &mut ap_sink,
                    &mut self.rx_eth_queue[self.rx_tail],
                ) {
                    self.rx_eth_lens[self.rx_tail] = eth_len as u16;
                    self.rx_tail = (self.rx_tail + 1) % 8;
                    self.rx_count += 1;
                }
            } else {
                let mut discard = [0u8; 0];
                let _ = ap.handle_rx_mpdu(mpdu, self.now_ms, &mut ap_sink, &mut discard);
                self.rx_dropped_overflow = self.rx_dropped_overflow.saturating_add(1);
            }
            return Ok(());
        }
        let mut sink = DriverTxSink {
            lmac: &mut self.lmac,
            tx_slot: &mut self.tx_slot,
        };
        self.sta
            .handle_rx_mpdu(mpdu, rssi, self.now_ms, &mut sink, |eth| {
                if eth.len() > 1536 {
                    self.rx_dropped_overflow = self.rx_dropped_overflow.saturating_add(1);
                    return;
                }
                if self.rx_count < 8 {
                    let copy_len = eth.len();
                    self.rx_eth_queue[self.rx_tail][..copy_len].copy_from_slice(eth);
                    self.rx_eth_lens[self.rx_tail] = copy_len as u16;
                    self.rx_tail = (self.rx_tail + 1) % 8;
                    self.rx_count += 1;
                } else {
                    self.rx_dropped_overflow = self.rx_dropped_overflow.saturating_add(1);
                }
            })?;
        Ok(())
    }

    /// Returns `true` if associated and 4-way handshake completed.
    pub fn is_connected(&self) -> bool {
        self.sta.is_connected()
    }

    /// Returns the current state of the 802.11 STA state machine.
    pub fn state(&self) -> StaState {
        self.sta.state()
    }

    /// Returns the local station MAC address.
    pub fn sta_mac(&self) -> [u8; 6] {
        self.sta.sta_mac()
    }

    /// Returns the BSSID of the associated access point, if connected.
    pub fn bssid(&self) -> Option<[u8; 6]> {
        self.sta.bssid()
    }

    /// Returns the active 20 MHz Wi-Fi channel.
    pub fn channel(&self) -> u8 {
        if let Some(ap) = &self.ap {
            return ap.config().channel;
        }
        self.sta.channel()
    }

    /// Returns the last measured RSSI from the associated AP, or -127 dBm.
    #[must_use]
    pub const fn rssi(&self) -> i8 {
        self.sta.rssi()
    }

    /// Returns the assigned Association Identifier (AID), or 0 if unassigned.
    #[must_use]
    pub const fn aid(&self) -> u16 {
        self.sta.aid()
    }

    /// Returns candidate access points discovered during scanning.
    #[must_use]
    pub fn scan_candidates(&self) -> &[ScanCandidate] {
        self.sta.scan_candidates()
    }

    /// Returns the number of Ethernet frames waiting in the RX queue.
    pub fn rx_queue_len(&self) -> usize {
        self.rx_count
    }

    /// Returns an immutable reference to the transmit slot.
    #[must_use]
    pub const fn tx_slot(&self) -> &WifiTxSlot<1600> {
        &self.tx_slot
    }

    /// Returns an immutable reference to the low-level MAC controller.
    pub fn lmac(&self) -> &LmacHal<B> {
        &self.lmac
    }

    /// Returns a mutable reference to the low-level MAC controller.
    pub fn lmac_mut(&mut self) -> &mut LmacHal<B> {
        &mut self.lmac
    }

    /// Returns `true` if the driver has completed [`Self::init`].
    pub fn is_initialized(&self) -> bool {
        self.initialized
    }

    /// Returns the cumulative number of failed outbound Ethernet frames.
    pub fn pending_eth_tx_failures(&self) -> u32 {
        self.pending_eth_tx_failures
    }

    /// Returns the cumulative count of dropped RX frames due to ring overflow.
    pub fn rx_dropped_overflow(&self) -> u32 {
        self.rx_dropped_overflow
    }

    /// Returns and clears the most recent [`StaError`] captured during `poll()`.
    pub fn take_last_sta_error(&mut self) -> Option<StaError> {
        self.last_sta_error.take()
    }

    /// Returns and clears `(rx_state, mpdu_fc0, sig_len)` for the most recent
    /// unicast or broadcast/multicast Data RX frame seen while connected.
    pub fn take_last_connected_rx_info(&mut self) -> Option<(u8, u8, u16)> {
        self.last_connected_rx_info.take()
    }

    /// Returns a diagnostic snapshot of RX ring & LMAC DMA state and clears
    /// `last_non_beacon_rx`:
    /// `(rx_popped_total, rx_ctrl_reg, rx_next_dscr_reg, head_word0, eof_mask, last_non_beacon_rx, eth_tx_ok, rx_malformed_total)`.
    #[allow(clippy::type_complexity)]
    pub fn take_rx_diag_snapshot(
        &mut self,
    ) -> (u32, u32, u32, u32, u8, Option<(u8, u8, u8, u16)>, u32, u32) {
        let rx_ctrl_reg = self.lmac.bus().read_reg(crate::regs::RX_CTRL_REG);
        let rx_next_dscr_reg = self.lmac.bus().read_reg(crate::regs::RX_NEXT_DSCR_REG);
        let head_word0 = self.rx_ring.current_desc().map_or(0, |d| d.word0);
        let mut eof_mask = 0u8;
        for idx in 0..8 {
            if self
                .rx_ring
                .desc(idx)
                .is_some_and(|d| (d.word0 & crate::dma_ring::WifiRxDmaDesc::EOF_BIT) != 0)
            {
                eof_mask |= 1u8 << idx;
            }
        }
        (
            self.rx_popped_total,
            rx_ctrl_reg,
            rx_next_dscr_reg,
            head_word0,
            eof_mask,
            self.last_non_beacon_rx.take(),
            self.eth_tx_ok,
            self.rx_malformed_total,
        )
    }

    /// Returns `(data_tx_primary_ok, data_tx_fallback_ok, rx_protected_total, last_protected_rx_info)`.
    #[must_use]
    pub const fn extra_diag_counts(&self) -> (u32, u32, u32, Option<(u8, u8)>) {
        let (pri_ok, fb_ok) = self.sta.data_tx_counts();
        (
            pri_ok,
            fb_ok,
            self.rx_protected_total,
            self.last_protected_rx_info,
        )
    }

    /// Shuts down the Wi-Fi baseband and resets hardware MAC state.
    ///
    /// Idempotent: returns immediately when the driver is not initialized.
    pub fn shutdown(&mut self) {
        if !self.initialized {
            return;
        }
        crate::phy_bridge::phy_disable_rx();
        crate::phy_bridge::phy_arm_bb_wdt(false);
        self.lmac.deinit_mac();
        self.ap = None;
        self.last_connected_rx_info = None;
        self.last_non_beacon_rx = None;
        self.initialized = false;
    }
}

impl<B: WifiMmioBus> Drop for Esp32StaWifiDriver<B> {
    fn drop(&mut self) {
        if self.initialized {
            self.shutdown();
        }
    }
}

impl<B: WifiMmioBus> smoltcp::phy::Device for Esp32StaWifiDriver<B> {
    type RxToken<'a>
        = StaRxToken
    where
        Self: 'a;
    type TxToken<'a>
        = StaTxToken<'a, B>
    where
        Self: 'a;

    fn receive(
        &mut self,
        _timestamp: smoltcp::time::Instant,
    ) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        if self.rx_count == 0 {
            return None;
        }
        let len = self.rx_eth_lens[self.rx_head] as usize;
        let mut buf = [0u8; 1536];
        let copy_len = len.min(buf.len());
        buf[..copy_len].copy_from_slice(&self.rx_eth_queue[self.rx_head][..copy_len]);
        self.rx_head = (self.rx_head + 1) % 8;
        self.rx_count -= 1;

        let rx_token = StaRxToken { buf, len: copy_len };
        let tx_token = StaTxToken { driver: self };
        Some((rx_token, tx_token))
    }

    fn transmit(&mut self, _timestamp: smoltcp::time::Instant) -> Option<Self::TxToken<'_>> {
        Some(StaTxToken { driver: self })
    }

    fn capabilities(&self) -> smoltcp::phy::DeviceCapabilities {
        let mut caps = smoltcp::phy::DeviceCapabilities::default();
        caps.medium = smoltcp::phy::Medium::Ethernet;
        caps.max_transmission_unit = 1514;
        caps.max_burst_size = Some(4);
        caps
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dma_ring::DmaBuffer;
    use crate::dma_ring::WifiRxDmaDesc;
    use crate::dma_ring::RX_CTRL_LEN;
    use crate::lmac_mmio::MockWifiMmioBus;
    use esp32_cam_wpa2::ie::gen_wpa_ie;
    use esp32_cam_wpa2::ie::WPA_CIPHER_CCMP;
    use esp32_cam_wpa2::ie::WPA_KEY_MGMT_PSK;
    use esp32_cam_wpa2::ie::WPA_PROTO_RSN;
    use smoltcp::phy::Device;
    use smoltcp::phy::TxToken;
    use smoltcp::time::Instant;

    /// Test-only shadow layout matching `#[repr(C, align(4))] WifiRxRing<8, 1600>`
    /// so driver tests can simulate hardware DMA writes into HW-owned slots.
    #[repr(C, align(4))]
    struct MockRxRingLayout {
        descs: core::cell::UnsafeCell<[WifiRxDmaDesc; 8]>,
        buffers: core::cell::UnsafeCell<[DmaBuffer<1600>; 8]>,
        head: usize,
        armed: bool,
        _pin: core::marker::PhantomPinned,
    }

    fn sim_hw_rx_slot(
        driver: &mut Esp32StaWifiDriver<MockWifiMmioBus>,
        idx: usize,
        rssi_byte: u8,
        rx_state: u8,
        sig_len: u16,
        desc_payload_len: usize,
        body: &[u8],
    ) {
        assert!(idx < 8);
        assert_eq!(
            core::mem::size_of::<WifiRxRing<8, 1600>>(),
            core::mem::size_of::<MockRxRingLayout>()
        );
        assert_eq!(
            core::mem::align_of::<WifiRxRing<8, 1600>>(),
            core::mem::align_of::<MockRxRingLayout>()
        );
        let desc_len = (RX_CTRL_LEN + desc_payload_len) as u32;
        let ring_ptr = core::ptr::addr_of_mut!(driver.rx_ring).cast::<MockRxRingLayout>();
        // JUSTIFICATION: Driver-level test simulating hardware DMA completion
        // into an armed `WifiRxRing<8, 1600>` slot via its `#[repr(C, align(4))]`
        // `UnsafeCell` storage.
        // SAFETY: `MockRxRingLayout` has identical `#[repr(C, align(4))]` field
        // order, size, and alignment as `WifiRxRing<8, 1600>`, `idx < 8`, and
        // `driver` is exclusively borrowed in a single-threaded host test.
        unsafe {
            let b = &mut (*(*ring_ptr).buffers.get())[idx].0;
            let mut words = [0u32; 7];
            words[0] = u32::from(rssi_byte);
            words[6] = (u32::from(rx_state) << 24) | u32::from(sig_len & 0x0FFF);
            for (w_idx, &val) in words.iter().enumerate() {
                let off = w_idx * 4;
                b[off..off + 4].copy_from_slice(&val.to_le_bytes());
            }
            let copy_len = body.len().min(1600 - RX_CTRL_LEN);
            b[RX_CTRL_LEN..RX_CTRL_LEN + copy_len].copy_from_slice(&body[..copy_len]);

            let d = &mut (*(*ring_ptr).descs.get())[idx];
            let word0 = WifiRxDmaDesc::SOF_BIT
                | WifiRxDmaDesc::EOF_BIT
                | ((desc_len & 0x0FFF) << 12)
                | (1600u32 & 0x0FFF);
            core::ptr::write_volatile(core::ptr::addr_of_mut!(d.word0), word0);
        }
    }

    fn build_valid_rsn_beacon(bssid: [u8; 6], ssid: &[u8], ch: u8) -> ([u8; 128], usize) {
        let mut rsn = [0u8; 32];
        let rsn_len = gen_wpa_ie(
            WPA_PROTO_RSN,
            WPA_CIPHER_CCMP,
            WPA_CIPHER_CCMP,
            WPA_KEY_MGMT_PSK,
            0,
            0,
            &mut rsn,
        )
        .expect("rsn ie");
        let mut b = [0u8; 128];
        b[0] = 0x80;
        b[4..10].fill(0xFF);
        b[10..16].copy_from_slice(&bssid);
        b[16..22].copy_from_slice(&bssid);
        b[34] = 0x11;
        let mut p = 36;
        b[p] = 0;
        b[p + 1] = ssid.len() as u8;
        b[p + 2..p + 2 + ssid.len()].copy_from_slice(ssid);
        p += 2 + ssid.len();
        b[p..p + 3].copy_from_slice(&[3, 1, ch]);
        p += 3;
        b[p..p + rsn_len].copy_from_slice(&rsn[..rsn_len]);
        p += rsn_len;
        (b, p)
    }

    #[test]
    fn test_poll_fcs_stripping_and_runt_rejection() {
        let _guard = crate::phy_bridge::test_sync::lock_host_phy();
        crate::phy_bridge::host_mock_reset();
        crate::phy_bridge::host_mock_set_phy_enabled(true);

        let mut bus = MockWifiMmioBus::new();
        bus.auto_init_3d24 = true;
        bus.auto_tx_done = true;
        let mut driver = Esp32StaWifiDriver::new(bus);
        // JUSTIFICATION: Stack-pinned local driver never moved after init.
        // SAFETY: `driver` stays at a fixed stack address for the test lifetime.
        assert!(unsafe { Pin::new_unchecked(&mut driver) }
            .init_pinned(Some([0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc]))
            .is_ok());

        // Slot 0: Runt frame with <= 4 bytes (4 bytes payload + 28 bytes header).
        sim_hw_rx_slot(&mut driver, 0, 0xC4, 0, 4, 4, &[0x11, 0x22, 0x33, 0x44]);

        // Slot 1: Valid frame with > 4 bytes payload (including 4-byte FCS).
        sim_hw_rx_slot(
            &mut driver,
            1,
            0xC4,
            0,
            14,
            14,
            &[
                0x80, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xAA, 0xBB, 0xCC, 0xDD,
            ],
        );

        // Poll should process slot 0 (dropping runt and recycling it) and slot 1.
        assert!(driver.poll(100).is_ok());

        // Verify both descriptors have been recycled back to HW ownership.
        if let Some(d0) = driver.rx_ring.desc(0) {
            assert!(d0.is_owner_hw());
        }
        if let Some(d1) = driver.rx_ring.desc(1) {
            assert!(d1.is_owner_hw());
        }

        // Verify rx_ring is now empty and head advanced past both slots.
        assert_eq!(driver.rx_ring.pop_received(), RxPop::Empty);
        assert_eq!(driver.rx_ring.head_index(), 2);

        // Forward getters check.
        assert_eq!(driver.rssi(), -127);
        assert_eq!(driver.aid(), 0);
        assert_eq!(driver.scan_candidates().len(), 0);
    }

    #[test]
    fn test_poll_padded_descriptor_fcs_stripping_and_rx_state_drop() {
        let _guard = crate::phy_bridge::test_sync::lock_host_phy();
        crate::phy_bridge::host_mock_reset();
        crate::phy_bridge::host_mock_set_phy_enabled(true);

        let mut bus = MockWifiMmioBus::new();
        bus.auto_init_3d24 = true;
        bus.auto_tx_done = true;
        let mut driver = Esp32StaWifiDriver::new(bus);
        // JUSTIFICATION: Stack-pinned local driver never moved after init.
        // SAFETY: `driver` stays at a fixed stack address for the test lifetime.
        unsafe { Pin::new_unchecked(&mut driver) }
            .init_pinned(Some([0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc]))
            .expect("init");
        #[cfg(miri)]
        let pass: &[u8] = b"0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";
        #[cfg(not(miri))]
        let pass: &[u8] = b"password123";
        let cfg = StaConnectConfig::from_slices(b"PadNet", pass, None, -90).expect("cfg");
        driver.connect(cfg, 10).expect("connect");
        assert_eq!(driver.state(), StaState::Scanning);

        let bssid_bad = [0x02, 0xAA, 0x00, 0x00, 0x00, 0x01];
        let bssid_fallback = [0x02, 0xAA, 0x00, 0x00, 0x00, 0x02];
        let bssid_padded = [0x02, 0xAA, 0x00, 0x00, 0x00, 0x03];

        // Slot 0: Valid beacon but ctrl.rx_state != 0 -> MUST be dropped.
        let (mut b0, l0) = build_valid_rsn_beacon(bssid_bad, b"PadNet", 1);
        b0[l0..l0 + 4].copy_from_slice(&[0xDE, 0xAD, 0xBE, 0xEF]);
        sim_hw_rx_slot(
            &mut driver,
            0,
            0xC4,
            0x01,
            (l0 + 4) as u16,
            l0 + 4,
            &b0[..l0 + 4],
        );

        // Slot 1: sig_len == 0 fallback (padded descriptor) -> MUST be dropped.
        let (mut b1, l1) = build_valid_rsn_beacon(bssid_fallback, b"PadNet", 1);
        b1[l1..l1 + 4].copy_from_slice(&[0xDE, 0xAD, 0xBE, 0xEF]);
        let padded_len1 = (l1 + 4 + 3) & !3;
        sim_hw_rx_slot(
            &mut driver,
            1,
            0xC4,
            0x00,
            0,
            padded_len1,
            &b1[..padded_len1],
        );

        // Slot 2: Word-padded DMA descriptor (desc_payload_len > sig_len) with
        // valid sig_len = l2 + 4 -> FCS stripped cleanly at sig_len - 4,
        // preserving exact RSN IE tail bytes and recording candidate!
        let (mut b2, l2) = build_valid_rsn_beacon(bssid_padded, b"PadNet", 1);
        b2[l2..l2 + 4].copy_from_slice(&[0x11, 0x22, 0x33, 0x44]); // 4-byte FCS
        b2[l2 + 4..l2 + 8].copy_from_slice(&[0xEE, 0xEE, 0xEE, 0xEE]); // 4-byte DMA padding
        let sig_len2 = (l2 + 4) as u16;
        let desc_payload_len2 = l2 + 8;
        sim_hw_rx_slot(
            &mut driver,
            2,
            0xC4,
            0x00,
            sig_len2,
            desc_payload_len2,
            &b2[..desc_payload_len2],
        );

        driver.poll(20).expect("poll");
        assert_eq!(driver.scan_candidates().len(), 1);
        assert_eq!(driver.scan_candidates()[0].bssid, bssid_padded);
    }

    #[test]
    fn test_poll_recycles_malformed_rx_descriptor_without_wedging() {
        let _guard = crate::phy_bridge::test_sync::lock_host_phy();
        crate::phy_bridge::host_mock_reset();
        crate::phy_bridge::host_mock_set_phy_enabled(true);

        let mut bus = MockWifiMmioBus::new();
        bus.auto_init_3d24 = true;
        bus.auto_tx_done = true;
        let mut driver = Esp32StaWifiDriver::new(bus);
        // JUSTIFICATION: Stack-pinned local driver never moved after init.
        // SAFETY: `driver` stays at a fixed stack address for the test lifetime.
        unsafe { Pin::new_unchecked(&mut driver) }
            .init_pinned(Some([0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc]))
            .expect("init");
        #[cfg(miri)]
        let pass: &[u8] = b"0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";
        #[cfg(not(miri))]
        let pass: &[u8] = b"password123";
        let cfg = StaConnectConfig::from_slices(b"NoWedge", pass, None, -90).expect("cfg");
        driver.connect(cfg, 10).expect("connect");

        // Slot 0: Malformed SW-owned descriptor (missing EOF — multi-descriptor
        // jumbo frame). Previously caused `poll()` to `break` and wedge forever.
        let ring_ptr = core::ptr::addr_of_mut!(driver.rx_ring).cast::<MockRxRingLayout>();
        // JUSTIFICATION: Host test simulates hardware writing a multi-descriptor
        // SOF-only slot without EOF into `WifiRxRing`.
        // SAFETY: `MockRxRingLayout` matches `WifiRxRing<8, 1600>` layout.
        unsafe {
            let d0 = &mut (*(*ring_ptr).descs.get())[0];
            let word0 = WifiRxDmaDesc::SOF_BIT | ((1600u32 & 0x0FFF) << 12) | (1600u32 & 0x0FFF);
            core::ptr::write_volatile(core::ptr::addr_of_mut!(d0.word0), word0);
        }

        // Slot 1: Valid beacon right after the malformed descriptor.
        let bssid_ok = [0x02, 0xBB, 0x00, 0x00, 0x00, 0x01];
        let (mut b1, l1) = build_valid_rsn_beacon(bssid_ok, b"NoWedge", 1);
        b1[l1..l1 + 4].copy_from_slice(&[0x11, 0x22, 0x33, 0x44]);
        sim_hw_rx_slot(
            &mut driver,
            1,
            0xC4,
            0x00,
            (l1 + 4) as u16,
            l1 + 4,
            &b1[..l1 + 4],
        );

        driver.poll(20).expect("poll");

        // Slot 0 MUST be recycled back to HW and slot 1 MUST be processed!
        assert!(driver.rx_ring.desc(0).expect("d0").is_owner_hw());
        assert!(driver.rx_ring.desc(1).expect("d1").is_owner_hw());
        assert_eq!(driver.rx_ring.head_index(), 2);
        assert_eq!(driver.rx_ring.pop_received(), RxPop::Empty);
        assert_eq!(driver.scan_candidates().len(), 1);
        assert_eq!(driver.scan_candidates()[0].bssid, bssid_ok);
    }

    #[test]
    fn test_tx_drain_failure_keeps_tx_slot_in_flight() {
        let _guard = crate::phy_bridge::test_sync::lock_host_phy();
        crate::phy_bridge::host_mock_reset();
        crate::phy_bridge::host_mock_set_phy_enabled(true);

        let mut bus = MockWifiMmioBus::new();
        bus.auto_init_3d24 = true;
        // Trigger immediate TX HW error while TX_ACTIVE stays stuck high so
        // `abort_and_drain_tx()` pulses MAC_INIT_REG_3C58 and returns
        // `TxDrainFailed` rapidly (including under Miri).
        bus.auto_tx_done = false;
        bus.auto_tx_err = 1;
        let mut driver = Esp32StaWifiDriver::new(bus);
        // JUSTIFICATION: Stack-pinned local driver never moved after init.
        // SAFETY: `driver` stays at a fixed stack address for the test lifetime.
        unsafe { Pin::new_unchecked(&mut driver) }
            .init_pinned(Some([0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc]))
            .expect("init");
        driver.lmac_mut().bus_mut().set_reg(
            crate::regs::MAC_CLK_RST_REG,
            crate::regs::MAC_CLK_RST_BIT_TX_ACTIVE,
        );

        let mut sink = DriverTxSink {
            lmac: &mut driver.lmac,
            tx_slot: &mut driver.tx_slot,
        };
        let frame1 = [0xAAu8; 24];
        let err1 = sink.send_mpdu(&frame1, WifiPhyRate::Rate1mL, false, 100);
        assert_eq!(err1, Err(StaError::Hardware(LmacError::TxDrainFailed)));
        // M-074: Slot MUST remain in_flight so DMA buffer cannot be overwritten.
        assert!(driver.tx_slot().is_in_flight());
        assert_eq!(&driver.tx_slot().buffer_slice()[..24], &frame1);

        // Second TX attempt MUST fail with TxDrainFailed (from TxBusy) and MUST
        // NOT overwrite the in-flight buffer contents.
        let mut sink2 = DriverTxSink {
            lmac: &mut driver.lmac,
            tx_slot: &mut driver.tx_slot,
        };
        let frame2 = [0x55u8; 24];
        let err2 = sink2.send_mpdu(&frame2, WifiPhyRate::Rate1mL, false, 101);
        assert_eq!(err2, Err(StaError::Hardware(LmacError::TxDrainFailed)));
        assert!(driver.tx_slot().is_in_flight());
        assert_eq!(&driver.tx_slot().buffer_slice()[..24], &frame1);
    }

    #[test]
    fn test_init_fails_when_phy_session_inactive() {
        let _guard = crate::phy_bridge::test_sync::lock_host_phy();
        crate::phy_bridge::host_mock_reset();

        let mut bus = MockWifiMmioBus::new();
        bus.auto_init_3d24 = true;
        let mut driver = Esp32StaWifiDriver::new(bus);
        // JUSTIFICATION: Stack-pinned local driver never moved.
        // SAFETY: `driver` stays at a fixed stack address for the test lifetime.
        let res = unsafe { Pin::new_unchecked(&mut driver) }
            .init_pinned(Some([0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc]));
        assert_eq!(
            res,
            Err(DriverError::Phy(PhyBridgeError::PhySessionInactive))
        );
        assert!(!driver.is_initialized());
        assert!(!driver.rx_ring.is_armed());
    }

    #[test]
    fn test_oversized_tx_token_rejected_without_truncation() {
        let _guard = crate::phy_bridge::test_sync::lock_host_phy();
        crate::phy_bridge::host_mock_reset();
        crate::phy_bridge::host_mock_set_phy_enabled(true);

        let mut bus = MockWifiMmioBus::new();
        bus.auto_init_3d24 = true;
        let mut driver = Esp32StaWifiDriver::new(bus);
        // JUSTIFICATION: Stack-pinned local driver never moved after init.
        // SAFETY: `driver` stays at a fixed stack address for the test lifetime.
        unsafe { Pin::new_unchecked(&mut driver) }
            .init_pinned(Some([0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc]))
            .expect("init");
        let tx = driver.transmit(Instant::from_millis(0)).expect("tx token");
        let seen_len = tx.consume(2000, |buf| buf.len());
        assert_eq!(seen_len, 0);
        assert_eq!(driver.pending_eth_tx_failures(), 1);
    }

    #[test]
    fn test_init_enables_11b_low_rate() {
        let _guard = crate::phy_bridge::test_sync::lock_host_phy();
        crate::phy_bridge::host_mock_reset();
        crate::phy_bridge::host_mock_set_phy_enabled(true);

        let mut bus = MockWifiMmioBus::new();
        bus.auto_init_3d24 = true;
        let mut driver = Esp32StaWifiDriver::new(bus);
        // JUSTIFICATION: Stack-pinned local driver never moved after init.
        // SAFETY: `driver` stays at a fixed stack address for the test lifetime.
        assert!(unsafe { Pin::new_unchecked(&mut driver) }
            .init_pinned(Some([0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc]))
            .is_ok());

        let m = driver.lmac().bus();
        assert_eq!(m.reg(crate::regs::AUTOACK_RATE_INIT_REGS[7]), 0x0B0B_0B0B);
        assert_eq!(m.reg(crate::regs::AUTOACK_RATE_INIT_REGS[6]), 0x0B0B_0B0B);
        assert_eq!(
            m.reg(crate::regs::MAC_LOW_RATE_DISABLE_REG_6860) & 0x0000_0C00,
            0x0000_0C00
        );
        assert_eq!(
            m.reg(crate::regs::MAC_LOW_RATE_DISABLE_REG_687C) & 0x0000_0800,
            0x0000_0800
        );
    }

    struct SharedMockBus<'a>(&'a core::cell::RefCell<MockWifiMmioBus>);

    impl WifiMmioBus for SharedMockBus<'_> {
        fn read_reg(&self, addr: u32) -> u32 {
            self.0.borrow().read_reg(addr)
        }

        fn write_reg(&mut self, addr: u32, val: u32) {
            self.0.borrow_mut().write_reg(addr, val);
        }

        fn delay_us(&mut self, us: u32) {
            self.0.borrow_mut().delay_us(us);
        }

        fn mem_barrier(&mut self) {
            self.0.borrow_mut().mem_barrier();
        }
    }

    #[test]
    fn test_drop_initialized_driver_shuts_down_and_halts_dma() {
        let _guard = crate::phy_bridge::test_sync::lock_host_phy();
        crate::phy_bridge::host_mock_reset();
        crate::phy_bridge::host_mock_set_phy_enabled(true);

        let mut raw_bus = MockWifiMmioBus::new();
        raw_bus.auto_init_3d24 = true;
        let shared_bus = core::cell::RefCell::new(raw_bus);

        {
            let mut driver = core::pin::pin!(Esp32StaWifiDriver::new(SharedMockBus(&shared_bus)));
            driver
                .as_mut()
                .init_pinned(Some([0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc]))
                .expect("init_pinned");

            assert!(driver.is_initialized());
            let snap = crate::phy_bridge::host_mock_snapshot();
            assert!(snap.rx_enabled);
            assert!(snap.bb_wdt_enabled);
            assert_eq!(snap.channel, 1);
            assert_eq!(snap.pwctrl_bg_calls, 1);
            assert_ne!(
                shared_bus.borrow().reg(crate::regs::RX_CTRL_REG) & crate::regs::RX_CTRL_BIT_ENABLE,
                0
            );
            assert_eq!(
                shared_bus.borrow().reg(crate::regs::INT_ENABLE_REG),
                crate::regs::INT_ENABLE_OPERATIONAL
            );
            // Exiting scope drops `driver` without an explicit `shutdown()`.
        }

        let snap_after = crate::phy_bridge::host_mock_snapshot();
        assert!(!snap_after.rx_enabled);
        assert!(!snap_after.bb_wdt_enabled);
        assert_eq!(
            shared_bus.borrow().reg(crate::regs::RX_CTRL_REG) & crate::regs::RX_CTRL_BIT_ENABLE,
            0
        );
        assert_eq!(shared_bus.borrow().reg(crate::regs::INT_ENABLE_REG), 0);
        assert_eq!(
            shared_bus.borrow().reg(crate::regs::MAC_CLK_RST_REG)
                & crate::regs::MAC_CLK_RST_MASK_UNGATE,
            crate::regs::MAC_CLK_RST_MASK_UNGATE
        );
    }

    #[test]
    fn test_init_mac_failure_disables_phy_rx() {
        let _guard = crate::phy_bridge::test_sync::lock_host_phy();
        crate::phy_bridge::host_mock_reset();
        crate::phy_bridge::host_mock_set_phy_enabled(true);
        crate::phy_bridge::phy_switch_channel_20mhz(6).expect("pre-seed channel 6");

        // `auto_init_3d24` defaults to false, causing `init_mac` to time out
        // before `phy_enable_rx()`, Channel 1 tuning, or
        // `phy_run_tx_pwctrl_bg()` run.
        let bus = MockWifiMmioBus::new();
        let mut driver = core::pin::pin!(Esp32StaWifiDriver::new(bus));
        let res = driver
            .as_mut()
            .init_pinned(Some([0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc]));
        assert_eq!(res, Err(DriverError::Lmac(LmacError::MacResetTimeout)));
        assert!(!driver.is_initialized());
        let snap = crate::phy_bridge::host_mock_snapshot();
        assert!(!snap.rx_enabled);
        assert_eq!(snap.channel, 6);
        assert_eq!(snap.pwctrl_bg_calls, 0);
    }

    #[test]
    fn test_enter_ap_mode_shared_dma_rx_tx_and_exit() {
        let _guard = crate::phy_bridge::test_sync::lock_host_phy();
        crate::phy_bridge::host_mock_reset();
        crate::phy_bridge::host_mock_set_phy_enabled(true);

        let ap_mac = [0xD4, 0xD4, 0xDA, 0x5B, 0xD4, 0xEC];
        let sta_mac = [0x02, 0x11, 0x22, 0x33, 0x44, 0x55];
        let cfg = ApConfig::for_setup_mac(ap_mac, 6);

        let mut bus = MockWifiMmioBus::new();
        bus.auto_init_3d24 = true;
        bus.auto_tx_done = true;
        let mut driver = Esp32StaWifiDriver::new(bus);

        // Must fail before init.
        assert_eq!(driver.enter_ap_mode(cfg), Err(DriverError::NotInitialized));

        // JUSTIFICATION: Stack-pinned local driver never moved after init.
        // SAFETY: `driver` stays at a fixed stack address for the test lifetime.
        unsafe { Pin::new_unchecked(&mut driver) }
            .init_pinned(Some(ap_mac))
            .expect("init_pinned");

        assert!(!driver.is_ap_mode());
        driver.enter_ap_mode(cfg).expect("enter_ap_mode");
        assert!(driver.is_ap_mode());
        assert_eq!(driver.channel(), 6);
        assert_eq!(crate::phy_bridge::host_mock_snapshot().channel, 6);
        assert_ne!(
            driver.lmac().bus().reg(crate::regs::RX_POLICY_BASE_REG)
                & crate::regs::RX_POLICY_PROMISC_BYPASS,
            0
        );

        // Poll emits initial Beacon through the shared TX DMA slot.
        driver.poll(10).expect("ap poll beacon");
        let mut expected_beacon = [0u8; 128];
        let beacon_len = crate::frames_80211::build_beacon(
            &ap_mac,
            b"KlipperCam-Setup-D4EC",
            6,
            100,
            10_000,
            0,
            &mut expected_beacon,
        )
        .expect("expected beacon");
        assert_eq!(
            &driver.tx_slot().buffer_slice()[..beacon_len],
            &expected_beacon[..beacon_len]
        );
        let beacon = crate::frames_80211::parse_beacon_or_probe_resp(
            &driver.tx_slot().buffer_slice()[..beacon_len],
            -35,
        )
        .expect("valid beacon in shared tx_slot");
        assert_eq!(beacon.bssid, ap_mac);
        assert_eq!(beacon.ssid, b"KlipperCam-Setup-D4EC");
        assert_eq!(beacon.channel, Some(6));

        // Feed AuthReq (slot 0), AssocReq (slot 1), Malformed (slot 2),
        // and ToDS=1 Data (slot 3) through the shared RX DMA ring.
        let mut auth = [0u8; 64];
        let auth_len =
            crate::frames_80211::build_open_auth_req(&mut auth, ap_mac, sta_mac, 1).expect("auth");
        auth[auth_len..auth_len + 4].copy_from_slice(&[0x11, 0x22, 0x33, 0x44]);
        sim_hw_rx_slot(
            &mut driver,
            0,
            0xC4,
            0,
            (auth_len + 4) as u16,
            auth_len + 4,
            &auth[..auth_len + 4],
        );

        let mut assoc = [0u8; 128];
        let assoc_len = crate::frames_80211::build_assoc_req(
            &mut assoc,
            ap_mac,
            sta_mac,
            b"KlipperCam-Setup-D4EC",
            &[],
            2,
            10,
        )
        .expect("assoc");
        assoc[assoc_len..assoc_len + 4].copy_from_slice(&[0x11, 0x22, 0x33, 0x44]);
        sim_hw_rx_slot(
            &mut driver,
            1,
            0xC4,
            0,
            (assoc_len + 4) as u16,
            assoc_len + 4,
            &assoc[..assoc_len + 4],
        );

        // Malformed SOF-only slot MUST be recycled without wedging AP mode.
        let ring_ptr = core::ptr::addr_of_mut!(driver.rx_ring).cast::<MockRxRingLayout>();
        // JUSTIFICATION: Host test injects a multi-descriptor SOF-only slot.
        // SAFETY: `MockRxRingLayout` matches `WifiRxRing<8, 1600>` layout.
        unsafe {
            let d2 = &mut (*(*ring_ptr).descs.get())[2];
            let word0 = WifiRxDmaDesc::SOF_BIT | ((1600u32 & 0x0FFF) << 12) | (1600u32 & 0x0FFF);
            core::ptr::write_volatile(core::ptr::addr_of_mut!(d2.word0), word0);
        }

        let mut eth_in = [0u8; 32];
        eth_in[0..6].copy_from_slice(&ap_mac);
        eth_in[6..12].copy_from_slice(&sta_mac);
        eth_in[12..14].copy_from_slice(&[0x08, 0x00]);
        eth_in[14..22].copy_from_slice(b"ap-hello");
        let mut data_mpdu = [0u8; 128];
        let data_len =
            crate::frames_80211::eth_to_80211_data(&mut data_mpdu, &eth_in[..22], ap_mac, 3, None)
                .expect("data mpdu");
        data_mpdu[data_len..data_len + 4].copy_from_slice(&[0xAA, 0xBB, 0xCC, 0xDD]);
        sim_hw_rx_slot(
            &mut driver,
            3,
            0xC4,
            0,
            (data_len + 4) as u16,
            data_len + 4,
            &data_mpdu[..data_len + 4],
        );

        driver.poll(50).expect("ap poll rx");
        assert_eq!(driver.ap_associated_clients(), 1);
        assert_eq!(driver.ap_last_activity_ms(), 50);
        assert_eq!(driver.rx_queue_len(), 1);
        for idx in 0..4 {
            assert!(driver.rx_ring.desc(idx).expect("desc").is_owner_hw());
        }

        // Verify smoltcp::phy::Device::receive returns decapsulated Ethernet frame.
        let (rx_tok, tx_tok) = driver
            .receive(Instant::from_millis(50))
            .expect("smoltcp rx token");
        let mut seen = [0u8; 32];
        let seen_len = smoltcp::phy::RxToken::consume(rx_tok, |buf| {
            seen[..buf.len()].copy_from_slice(buf);
            buf.len()
        });
        assert_eq!(seen_len, 22);
        assert_eq!(&seen[14..22], b"ap-hello");

        // Verify smoltcp::phy::Device::transmit sends FromDS=1 Data to STA.
        tx_tok.consume(20, |buf| {
            buf[0..6].copy_from_slice(&sta_mac);
            buf[6..12].copy_from_slice(&ap_mac);
            buf[12..14].copy_from_slice(&[0x08, 0x00]);
            buf[14..20].copy_from_slice(b"reply!");
        });
        assert_eq!(driver.pending_eth_tx_failures(), 0);
        assert_eq!(driver.tx_slot().buffer_slice()[1], 0x02); // FromDS=1

        driver.exit_ap_mode();
        assert!(!driver.is_ap_mode());
        assert_eq!(driver.ap_associated_clients(), 0);
        assert_eq!(driver.ap_last_activity_ms(), 0);
    }

    #[test]
    fn test_ap_mode_drop_shuts_down_and_halts_dma() {
        let _guard = crate::phy_bridge::test_sync::lock_host_phy();
        crate::phy_bridge::host_mock_reset();
        crate::phy_bridge::host_mock_set_phy_enabled(true);

        let ap_mac = [0xD4, 0xD4, 0xDA, 0x5B, 0xD4, 0xEC];
        let mut raw_bus = MockWifiMmioBus::new();
        raw_bus.auto_init_3d24 = true;
        raw_bus.auto_tx_done = true;
        let shared_bus = core::cell::RefCell::new(raw_bus);

        {
            let mut driver = core::pin::pin!(Esp32StaWifiDriver::new(SharedMockBus(&shared_bus)));
            driver
                .as_mut()
                .init_pinned(Some(ap_mac))
                .expect("init_pinned");
            // JUSTIFICATION: Entering AP mode mutates fields in place without
            // relocating the pinned DMA driver.
            // SAFETY: `driver` remains pinned for its entire lifetime.
            let drv = unsafe { driver.as_mut().get_unchecked_mut() };
            drv.enter_ap_mode(ApConfig::for_setup_mac(ap_mac, 1))
                .expect("enter_ap_mode");
            drv.poll(0).expect("ap poll");
            assert!(drv.is_ap_mode());
        }

        let snap_after = crate::phy_bridge::host_mock_snapshot();
        assert!(!snap_after.rx_enabled);
        assert!(!snap_after.bb_wdt_enabled);
        assert_eq!(
            shared_bus.borrow().reg(crate::regs::RX_CTRL_REG) & crate::regs::RX_CTRL_BIT_ENABLE,
            0
        );
        assert_eq!(shared_bus.borrow().reg(crate::regs::INT_ENABLE_REG), 0);
    }

    #[test]
    fn test_ap_tx_drain_failure_keeps_tx_slot_in_flight() {
        let _guard = crate::phy_bridge::test_sync::lock_host_phy();
        crate::phy_bridge::host_mock_reset();
        crate::phy_bridge::host_mock_set_phy_enabled(true);

        let mut bus = MockWifiMmioBus::new();
        bus.auto_init_3d24 = true;
        bus.auto_tx_done = false;
        bus.auto_tx_err = 1;
        let mut driver = Esp32StaWifiDriver::new(bus);
        // JUSTIFICATION: Stack-pinned local driver never moved after init.
        // SAFETY: `driver` stays at a fixed stack address for the test lifetime.
        unsafe { Pin::new_unchecked(&mut driver) }
            .init_pinned(Some([0xD4, 0xD4, 0xDA, 0x5B, 0xD4, 0xEC]))
            .expect("init");
        driver.lmac_mut().bus_mut().set_reg(
            crate::regs::MAC_CLK_RST_REG,
            crate::regs::MAC_CLK_RST_BIT_TX_ACTIVE,
        );

        let mut sink = ApDriverTxSink {
            lmac: &mut driver.lmac,
            tx_slot: &mut driver.tx_slot,
        };
        let frame1 = [0xAAu8; 24];
        let err1 = sink.send_mpdu(&frame1, WifiPhyRate::Rate1mL, false);
        assert_eq!(err1, Err(ApError::Hardware(LmacError::TxDrainFailed)));
        assert!(driver.tx_slot().is_in_flight());
        assert_eq!(&driver.tx_slot().buffer_slice()[..24], &frame1);
    }

    #[test]
    fn test_driver_ap_stats_sta_none_and_ap_some() {
        let _guard = crate::phy_bridge::test_sync::lock_host_phy();
        crate::phy_bridge::host_mock_reset();
        crate::phy_bridge::host_mock_set_phy_enabled(true);

        let mut bus = MockWifiMmioBus::new();
        bus.auto_init_3d24 = true;
        bus.auto_tx_done = true;
        let mut driver = Esp32StaWifiDriver::new(bus);
        // JUSTIFICATION: Stack-pinned local driver never moved after init.
        // SAFETY: `driver` stays at a fixed stack address for the test lifetime.
        unsafe { Pin::new_unchecked(&mut driver) }
            .init_pinned(Some([0xD4, 0xD4, 0xDA, 0x5B, 0xD4, 0xEC]))
            .expect("init");
        assert_eq!(driver.ap_stats(), None);

        driver
            .enter_ap_mode(ApConfig::for_setup_mac(
                [0xD4, 0xD4, 0xDA, 0x5B, 0xD4, 0xEC],
                1,
            ))
            .expect("enter ap");
        driver.poll(0).expect("poll");
        let stats = driver.ap_stats().expect("ap stats");
        assert_eq!(stats.beacons_ok, 1);
        assert_eq!(stats.beacons_err, 0);
    }

    #[test]
    fn test_init_enables_phy_rx_after_mac_init() {
        struct InitOrderBus {
            inner: MockWifiMmioBus,
            saw_rx_enabled_during_mmio: bool,
        }

        impl WifiMmioBus for InitOrderBus {
            fn read_reg(&self, addr: u32) -> u32 {
                self.inner.read_reg(addr)
            }

            fn write_reg(&mut self, addr: u32, val: u32) {
                if crate::phy_bridge::host_mock_snapshot().rx_enabled {
                    self.saw_rx_enabled_during_mmio = true;
                }
                self.inner.write_reg(addr, val);
            }

            fn delay_us(&mut self, us: u32) {
                self.inner.delay_us(us);
            }

            fn mem_barrier(&mut self) {
                self.inner.mem_barrier();
            }
        }

        let _guard = crate::phy_bridge::test_sync::lock_host_phy();
        crate::phy_bridge::host_mock_reset();
        crate::phy_bridge::host_mock_set_phy_enabled(true);

        let mut inner = MockWifiMmioBus::new();
        inner.auto_init_3d24 = true;
        let bus = InitOrderBus {
            inner,
            saw_rx_enabled_during_mmio: false,
        };
        let mut driver = core::pin::pin!(Esp32StaWifiDriver::new(bus));
        driver
            .as_mut()
            .init_pinned(Some([0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc]))
            .expect("init_pinned");

        assert!(crate::phy_bridge::host_mock_snapshot().rx_enabled);
        // `enable_low_rate()` writes MMIO after `phy_enable_rx()`, while
        // `init_mac()` runs before `phy_enable_rx()`. Verify `rx_enabled`
        // was false during `init_mac()` by asserting only post-enable MMIO saw it.
        assert!(driver.lmac().bus().saw_rx_enabled_during_mmio);

        // Verify directly via MAC reset reg check: run a failing `init_mac` and
        // ensure `rx_enabled` was never turned on during MMIO writes.
        crate::phy_bridge::host_mock_reset();
        crate::phy_bridge::host_mock_set_phy_enabled(true);
        let fail_bus = InitOrderBus {
            inner: MockWifiMmioBus::new(),
            saw_rx_enabled_during_mmio: false,
        };
        let mut fail_driver = core::pin::pin!(Esp32StaWifiDriver::new(fail_bus));
        let err = fail_driver
            .as_mut()
            .init_pinned(Some([0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc]));
        assert_eq!(err, Err(DriverError::Lmac(LmacError::MacResetTimeout)));
        assert!(!fail_driver.lmac().bus().saw_rx_enabled_during_mmio);
        assert!(!crate::phy_bridge::host_mock_snapshot().rx_enabled);
    }

    #[test]
    fn test_ap_rx_accepts_silicon_stripped_padded_and_mock_fcs_frames() {
        let _guard = crate::phy_bridge::test_sync::lock_host_phy();
        crate::phy_bridge::host_mock_reset();
        crate::phy_bridge::host_mock_set_phy_enabled(true);

        let ap_mac = [0xD4, 0xD4, 0xDA, 0x5B, 0xD4, 0xEC];
        let sta_mac = [0x02, 0x11, 0x22, 0x33, 0x44, 0x55];
        let cfg = ApConfig::for_setup_mac(ap_mac, 6);

        let mut bus = MockWifiMmioBus::new();
        bus.auto_init_3d24 = true;
        bus.auto_tx_done = true;
        let mut driver = core::pin::pin!(Esp32StaWifiDriver::new(bus));
        driver
            .as_mut()
            .init_pinned(Some(ap_mac))
            .expect("init_pinned");

        // JUSTIFICATION: Entering AP mode mutates fields in place without
        // relocating the pinned DMA driver.
        // SAFETY: `driver` remains pinned for its entire lifetime.
        let drv = unsafe { driver.as_mut().get_unchecked_mut() };
        drv.enter_ap_mode(cfg).expect("enter_ap_mode");
        drv.poll(10).expect("ap poll beacon");

        // Associate `sta_mac` first so `ApMacCore` accepts ToDS=1 Data MPDUs.
        let mut auth = [0u8; 64];
        let auth_len =
            crate::frames_80211::build_open_auth_req(&mut auth, ap_mac, sta_mac, 1).expect("auth");
        auth[auth_len..auth_len + 4].copy_from_slice(&[0x11, 0x22, 0x33, 0x44]);
        sim_hw_rx_slot(
            drv,
            0,
            0xC4,
            0,
            (auth_len + 4) as u16,
            auth_len + 4,
            &auth[..auth_len + 4],
        );

        let mut assoc = [0u8; 128];
        let assoc_len = crate::frames_80211::build_assoc_req(
            &mut assoc,
            ap_mac,
            sta_mac,
            b"KlipperCam-Setup-D4EC",
            &[],
            2,
            10,
        )
        .expect("assoc");
        assoc[assoc_len..assoc_len + 4].copy_from_slice(&[0x11, 0x22, 0x33, 0x44]);
        sim_hw_rx_slot(
            drv,
            1,
            0xC4,
            0,
            (assoc_len + 4) as u16,
            assoc_len + 4,
            &assoc[..assoc_len + 4],
        );
        drv.poll(20).expect("ap poll assoc");
        assert_eq!(drv.ap_associated_clients(), 1);

        // Slot 2: Silicon-stripped 4-byte-padded RX frame.
        // 35-byte Ethernet -> 53-byte 802.11 Data MPDU; silicon strips 4-byte
        // FCS and 4-byte-pads DMA payload to 56 bytes (`desc_len = 28 + 56`),
        // while `ctrl.sig_len` stays 57 (53 + 4).
        let mut eth_silicon = [0u8; 35];
        eth_silicon[0..6].copy_from_slice(&ap_mac);
        eth_silicon[6..12].copy_from_slice(&sta_mac);
        eth_silicon[12..14].copy_from_slice(&[0x08, 0x00]);
        eth_silicon[14..35].copy_from_slice(b"0123456789abcdefghijk");
        let mut mpdu_silicon = [0u8; 64];
        let mpdu_silicon_len = crate::frames_80211::eth_to_80211_data(
            &mut mpdu_silicon,
            &eth_silicon,
            ap_mac,
            3,
            None,
        )
        .expect("silicon data mpdu");
        assert_eq!(mpdu_silicon_len, 53);
        mpdu_silicon[53..56].copy_from_slice(&[0xEE, 0xEE, 0xEE]);
        sim_hw_rx_slot(drv, 2, 0xC4, 0, 57, 56, &mpdu_silicon[..56]);

        // Slot 3: Mock FCS-included RX frame (`sig_len = 60`, `desc_len = 28 + 60`).
        // 38-byte Ethernet -> 56-byte 802.11 Data MPDU + 4-byte FCS = 60 bytes.
        let mut eth_mock = [0u8; 38];
        eth_mock[0..6].copy_from_slice(&ap_mac);
        eth_mock[6..12].copy_from_slice(&sta_mac);
        eth_mock[12..14].copy_from_slice(&[0x08, 0x00]);
        eth_mock[14..38].copy_from_slice(b"mock-fcs-included-24byte");
        let mut mpdu_mock = [0u8; 64];
        let mpdu_mock_len =
            crate::frames_80211::eth_to_80211_data(&mut mpdu_mock, &eth_mock, ap_mac, 4, None)
                .expect("mock data mpdu");
        assert_eq!(mpdu_mock_len, 56);
        mpdu_mock[56..60].copy_from_slice(&[0xAA, 0xBB, 0xCC, 0xDD]);
        sim_hw_rx_slot(drv, 3, 0xC4, 0, 60, 60, &mpdu_mock[..60]);

        drv.poll(30).expect("ap poll data");
        assert_eq!(drv.rx_queue_len(), 2);

        let (rx_tok1, _) = drv
            .receive(Instant::from_millis(30))
            .expect("silicon rx token");
        let mut seen1 = [0u8; 64];
        let seen_len1 = smoltcp::phy::RxToken::consume(rx_tok1, |buf| {
            seen1[..buf.len()].copy_from_slice(buf);
            buf.len()
        });
        assert_eq!(seen_len1, 35);
        assert_eq!(&seen1[..35], &eth_silicon[..]);

        let (rx_tok2, _) = drv
            .receive(Instant::from_millis(30))
            .expect("mock rx token");
        let mut seen2 = [0u8; 64];
        let seen_len2 = smoltcp::phy::RxToken::consume(rx_tok2, |buf| {
            seen2[..buf.len()].copy_from_slice(buf);
            buf.len()
        });
        assert_eq!(seen_len2, 38);
        assert_eq!(&seen2[..38], &eth_mock[..]);
    }

    #[test]
    fn test_take_last_sta_error_captures_and_clears() {
        let _guard = crate::phy_bridge::test_sync::lock_host_phy();
        crate::phy_bridge::host_mock_reset();
        crate::phy_bridge::host_mock_set_phy_enabled(true);

        let mut bus = MockWifiMmioBus::new();
        bus.auto_init_3d24 = true;
        bus.auto_tx_done = true;
        let mut driver = core::pin::pin!(Esp32StaWifiDriver::new(bus));
        driver
            .as_mut()
            .init_pinned(Some([0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc]))
            .expect("init_pinned");

        // JUSTIFICATION: Driving `poll()` / `connect()` on the stack-pinned
        // driver without relocating it.
        // SAFETY: `driver` remains pinned for the entire test lifetime.
        let drv = unsafe { driver.as_mut().get_unchecked_mut() };
        assert_eq!(drv.take_last_sta_error(), None);

        #[cfg(miri)]
        let pass: &[u8] = b"0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";
        #[cfg(not(miri))]
        let pass: &[u8] = b"password123";
        let cfg = StaConnectConfig::from_slices(b"MissingAp", pass, None, -90).expect("cfg");
        let mut now_ms = 1_000u64;
        drv.connect(cfg, now_ms).expect("connect");

        for _ in 1..13 {
            now_ms += 95;
            drv.poll(now_ms).expect("scan dwell poll");
        }
        now_ms += 95;
        let res = drv.poll(now_ms);
        assert_eq!(res, Err(DriverError::Sta(StaError::NoSuitableAp)));
        assert_eq!(drv.take_last_sta_error(), Some(StaError::NoSuitableAp));
        assert_eq!(drv.take_last_sta_error(), None);

        // Re-trigger via scan sweep and verify `connect()` clears `last_sta_error`.
        now_ms += 2_000;
        drv.connect(cfg, now_ms).expect("reconnect");
        for _ in 1..13 {
            now_ms += 95;
            drv.poll(now_ms).expect("scan dwell poll 2");
        }
        now_ms += 95;
        assert_eq!(
            drv.poll(now_ms),
            Err(DriverError::Sta(StaError::NoSuitableAp))
        );
        now_ms += 2_000;
        drv.connect(cfg, now_ms).expect("connect clears last error");
        assert_eq!(drv.take_last_sta_error(), None);
    }

    #[test]
    fn test_poll_rx_state_sw_crypto_passthrough_requires_protected_bit() {
        let _guard = crate::phy_bridge::test_sync::lock_host_phy();
        crate::phy_bridge::host_mock_reset();
        crate::phy_bridge::host_mock_set_phy_enabled(true);

        let sta_mac = [0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc];
        let bssid = [0x02, 0xCC, 0x00, 0x00, 0x00, 0x01];

        let mut bus = MockWifiMmioBus::new();
        bus.auto_init_3d24 = true;
        bus.auto_tx_done = true;
        let mut driver = core::pin::pin!(Esp32StaWifiDriver::new(bus));
        driver
            .as_mut()
            .init_pinned(Some(sta_mac))
            .expect("init_pinned");

        // JUSTIFICATION: Driving `connect()` and `poll()` on the stack-pinned
        // driver without relocating it.
        // SAFETY: `driver` remains pinned for the entire test lifetime.
        let drv = unsafe { driver.as_mut().get_unchecked_mut() };

        #[cfg(miri)]
        let pass: &[u8] = b"0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";
        #[cfg(not(miri))]
        let pass: &[u8] = b"password123";
        let cfg = StaConnectConfig::from_slices(b"Passthrough", pass, None, -90).expect("cfg");
        let mut now_ms = 1_000u64;
        drv.connect(cfg, now_ms).expect("connect");
        assert_eq!(drv.state(), StaState::Scanning);

        let (mut beacon, beacon_len) = build_valid_rsn_beacon(bssid, b"Passthrough", 1);
        beacon[beacon_len..beacon_len + 4].copy_from_slice(&[0x11, 0x22, 0x33, 0x44]);
        sim_hw_rx_slot(
            drv,
            0,
            0xC4,
            0x00,
            (beacon_len + 4) as u16,
            beacon_len + 4,
            &beacon[..beacon_len + 4],
        );
        now_ms += 10;
        drv.poll(now_ms).expect("beacon poll");
        assert_eq!(drv.scan_candidates().len(), 1);

        // Advance through remaining channels (2..=13) to enter AuthWait.
        for _ in 1..=13 {
            now_ms += 95;
            drv.poll(now_ms).expect("scan dwell");
        }
        assert_eq!(drv.state(), StaState::AuthWait);

        // Slot 1: Open Auth Response (seq=2, status=0) -> AssocWait.
        let mut auth_resp = [0u8; 34];
        auth_resp[0] = 0xB0;
        auth_resp[4..10].copy_from_slice(&sta_mac);
        auth_resp[10..16].copy_from_slice(&bssid);
        auth_resp[16..22].copy_from_slice(&bssid);
        auth_resp[26..28].copy_from_slice(&2u16.to_le_bytes());
        sim_hw_rx_slot(drv, 1, 0xC4, 0x00, 34, 34, &auth_resp);
        now_ms += 10;
        drv.poll(now_ms).expect("auth resp poll");
        assert_eq!(drv.state(), StaState::AssocWait);

        // Slot 2: Assoc Response (status=0, aid=0xC001) -> FourWayWait.
        let mut assoc_resp = [0u8; 34];
        assoc_resp[0] = 0x10;
        assoc_resp[4..10].copy_from_slice(&sta_mac);
        assoc_resp[10..16].copy_from_slice(&bssid);
        assoc_resp[16..22].copy_from_slice(&bssid);
        assoc_resp[24..26].copy_from_slice(&0x0011u16.to_le_bytes());
        assoc_resp[28..30].copy_from_slice(&0xC001u16.to_le_bytes());
        sim_hw_rx_slot(drv, 2, 0xC4, 0x00, 34, 34, &assoc_resp);
        now_ms += 10;
        drv.poll(now_ms).expect("assoc resp poll");
        assert_eq!(drv.state(), StaState::FourWayWait);
        assert_eq!(drv.take_last_sta_error(), None);

        // Construct a Data MPDU addressed to `sta_mac` from `bssid`.
        let mut data_mpdu = [0u8; 52];
        data_mpdu[0] = 0x08;
        data_mpdu[4..10].copy_from_slice(&sta_mac);
        data_mpdu[10..16].copy_from_slice(&bssid);
        data_mpdu[16..22].copy_from_slice(&bssid);

        // 1. rx_state == 0xF5 with Protected=1 -> MUST pass `valid_frame` and reach STA FSM.
        data_mpdu[1] = 0x42; // FromDS=1 | Protected=1
        sim_hw_rx_slot(
            drv,
            3,
            0xC4,
            RX_STATE_SW_CRYPTO_PASSTHROUGH,
            52,
            52,
            &data_mpdu,
        );
        now_ms += 10;
        drv.poll(now_ms).expect("passthrough protected=1 poll");
        assert!(drv.take_last_sta_error().is_some());

        // 2. rx_state == 0xF5 with Protected=0 -> MUST be dropped before STA FSM.
        data_mpdu[1] = 0x02; // FromDS=1 | Protected=0
        sim_hw_rx_slot(
            drv,
            4,
            0xC4,
            RX_STATE_SW_CRYPTO_PASSTHROUGH,
            52,
            52,
            &data_mpdu,
        );
        now_ms += 10;
        drv.poll(now_ms).expect("passthrough protected=0 poll");
        assert_eq!(drv.take_last_sta_error(), None);

        // 3. rx_state == 0x01 with Protected=1 -> MUST be dropped before STA FSM.
        data_mpdu[1] = 0x42; // FromDS=1 | Protected=1
        sim_hw_rx_slot(drv, 5, 0xC4, 0x01, 52, 52, &data_mpdu);
        now_ms += 10;
        drv.poll(now_ms).expect("rx_state 0x01 poll");
        assert_eq!(drv.take_last_sta_error(), None);
    }

    #[test]
    fn test_poll_accepts_hw_ccmp_decrypted_protected_data_mpdu() {
        let _guard = crate::phy_bridge::test_sync::lock_host_phy();
        crate::phy_bridge::host_mock_reset();
        crate::phy_bridge::host_mock_set_phy_enabled(true);

        let sta_mac = [0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc];
        let bssid = [0x02, 0xCC, 0x00, 0x00, 0x00, 0x01];
        let sa = [0x02, 0xDD, 0xEE, 0xFF, 0x00, 0x02];
        let hex_psk = b"0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";
        let pmk = [
            0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef, 0x01, 0x23, 0x45, 0x67, 0x89, 0xab,
            0xcd, 0xef, 0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef, 0x01, 0x23, 0x45, 0x67,
            0x89, 0xab, 0xcd, 0xef,
        ];

        let mut bus = MockWifiMmioBus::new();
        bus.auto_init_3d24 = true;
        bus.auto_tx_done = true;
        let mut driver = core::pin::pin!(Esp32StaWifiDriver::new(bus));
        driver
            .as_mut()
            .init_pinned(Some(sta_mac))
            .expect("init_pinned");

        // JUSTIFICATION: Driving `connect()` and `poll()` on the stack-pinned
        // driver without relocating it.
        // SAFETY: `driver` remains pinned for the entire test lifetime.
        let drv = unsafe { driver.as_mut().get_unchecked_mut() };
        assert!(!drv.lmac().is_sta_hw_crypto_enabled());

        let cfg = StaConnectConfig::from_slices(b"HwCrypto", hex_psk, None, -90).expect("cfg");
        let mut now_ms = 1_000u64;
        drv.connect(cfg, now_ms).expect("connect");

        let (mut beacon, beacon_len) = build_valid_rsn_beacon(bssid, b"HwCrypto", 1);
        beacon[beacon_len..beacon_len + 4].copy_from_slice(&[0x11, 0x22, 0x33, 0x44]);
        sim_hw_rx_slot(
            drv,
            0,
            0xC4,
            0x00,
            (beacon_len + 4) as u16,
            beacon_len + 4,
            &beacon[..beacon_len + 4],
        );
        now_ms += 10;
        drv.poll(now_ms).expect("beacon poll");

        for _ in 1..=13 {
            now_ms += 95;
            drv.poll(now_ms).expect("scan dwell");
        }
        assert_eq!(drv.state(), StaState::AuthWait);

        let mut auth_resp = [0u8; 34];
        auth_resp[0] = 0xB0;
        auth_resp[4..10].copy_from_slice(&sta_mac);
        auth_resp[10..16].copy_from_slice(&bssid);
        auth_resp[16..22].copy_from_slice(&bssid);
        auth_resp[26..28].copy_from_slice(&2u16.to_le_bytes());
        sim_hw_rx_slot(drv, 1, 0xC4, 0x00, 34, 34, &auth_resp);
        now_ms += 10;
        drv.poll(now_ms).expect("auth resp poll");
        assert_eq!(drv.state(), StaState::AssocWait);

        let mut assoc_resp = [0u8; 34];
        assoc_resp[0] = 0x10;
        assoc_resp[4..10].copy_from_slice(&sta_mac);
        assoc_resp[10..16].copy_from_slice(&bssid);
        assoc_resp[16..22].copy_from_slice(&bssid);
        assoc_resp[24..26].copy_from_slice(&0x0011u16.to_le_bytes());
        assoc_resp[28..30].copy_from_slice(&0xC001u16.to_le_bytes());
        sim_hw_rx_slot(drv, 2, 0xC4, 0x00, 34, 34, &assoc_resp);
        now_ms += 10;
        drv.poll(now_ms).expect("assoc resp poll");
        assert_eq!(drv.state(), StaState::FourWayWait);

        // Slot 3: EAPOL M1 (FromDS=1, unencrypted).
        let anonce = [0x11u8; 32];
        let mut m1 = [0u8; 32 + 99 + 4];
        m1[0] = 0x08;
        m1[1] = 0x02;
        m1[4..10].copy_from_slice(&sta_mac);
        m1[10..16].copy_from_slice(&bssid);
        m1[16..22].copy_from_slice(&bssid);
        m1[24..32].copy_from_slice(&[0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00, 0x88, 0x8E]);
        {
            let eapol = &mut m1[32..32 + 99];
            eapol[0] = 0x02;
            eapol[1] = 0x03;
            eapol[2..4].copy_from_slice(&95u16.to_be_bytes());
            eapol[4] = 0x02;
            eapol[5..7].copy_from_slice(&0x008Au16.to_be_bytes());
            eapol[7..9].copy_from_slice(&16u16.to_be_bytes());
            eapol[16] = 1;
            eapol[17..49].copy_from_slice(&anonce);
        }
        sim_hw_rx_slot(drv, 3, 0xC4, 0x00, m1.len() as u16, m1.len(), &m1);
        now_ms += 10;
        drv.poll(now_ms).expect("M1 poll");

        let mut snonce = [0u8; 32];
        snonce.copy_from_slice(&drv.tx_slot().buffer_slice()[32 + 17..32 + 49]);
        let ptk = esp32_cam_wpa2::kdf::wpa_pmk_to_ptk(
            &pmk,
            b"Pairwise key expansion",
            &bssid,
            &sta_mac,
            &anonce,
            &snonce,
            false,
        );

        // Slot 4: EAPOL M3 with wrapped RSN IE + GTK KDE (key_id=1).
        let mut ap_rsn = [0u8; 32];
        let ap_rsn_len = gen_wpa_ie(
            WPA_PROTO_RSN,
            WPA_CIPHER_CCMP,
            WPA_CIPHER_CCMP,
            WPA_KEY_MGMT_PSK,
            0,
            0,
            &mut ap_rsn,
        )
        .expect("ap rsn");
        assert_eq!(ap_rsn_len, 22);
        let gtk = [0x99u8; 16];
        let mut plain_kd = [0u8; 48];
        plain_kd[..22].copy_from_slice(&ap_rsn[..22]);
        plain_kd[22] = 0xDD;
        plain_kd[23] = 22;
        plain_kd[24..27].copy_from_slice(&[0x00, 0x0F, 0xAC]);
        plain_kd[27] = 0x01;
        plain_kd[28] = 1; // GTK key_id = 1
        plain_kd[30..46].copy_from_slice(&gtk);
        plain_kd[46] = 0xDD;
        let mut wrapped_kd = [0u8; 56];
        esp32_cam_wpa2::crypto::aes_wrap(&ptk.kek, &plain_kd, &mut wrapped_kd).expect("aes_wrap");

        let mut m3 = [0u8; 32 + 99 + 56 + 4];
        m3[0] = 0x08;
        m3[1] = 0x02;
        m3[4..10].copy_from_slice(&sta_mac);
        m3[10..16].copy_from_slice(&bssid);
        m3[16..22].copy_from_slice(&bssid);
        m3[24..32].copy_from_slice(&[0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00, 0x88, 0x8E]);
        {
            let eapol = &mut m3[32..32 + 99 + 56];
            eapol[0] = 0x02;
            eapol[1] = 0x03;
            eapol[2..4].copy_from_slice(&((95 + 56) as u16).to_be_bytes());
            eapol[4] = 0x02;
            eapol[5..7].copy_from_slice(&0x13CAu16.to_be_bytes());
            eapol[7..9].copy_from_slice(&16u16.to_be_bytes());
            eapol[16] = 2;
            eapol[17..49].copy_from_slice(&anonce);
            eapol[97..99].copy_from_slice(&(56u16).to_be_bytes());
            eapol[99..99 + 56].copy_from_slice(&wrapped_kd);
            let mic = esp32_cam_wpa2::crypto::hmac_sha1_vector(&ptk.kck, &[&eapol[..]]);
            eapol[81..97].copy_from_slice(&mic[..16]);
        }
        sim_hw_rx_slot(drv, 4, 0xC4, 0x00, m3.len() as u16, m3.len(), &m3);
        now_ms += 10;
        drv.poll(now_ms).expect("M3 poll");

        assert_eq!(drv.state(), StaState::Connected);
        assert!(drv.lmac().is_sta_hw_crypto_enabled());
        assert_eq!(drv.take_last_sta_error(), None);

        // Slot 5: Hardware-decrypted Protected=1 FromDS=1 Data MPDU (`rx_state == 0`).
        // 24B HDR + 8B CCMP IV (PN=1, ExtIV=1, KeyID=0) + 8B LLC/SNAP + 6B payload + 8B MIC = 54B.
        let mut hw_mpdu = [0u8; 54 + 4];
        hw_mpdu[0] = 0x08;
        hw_mpdu[1] = 0x42; // FromDS=1 | Protected=1
        hw_mpdu[4..10].copy_from_slice(&sta_mac);
        hw_mpdu[10..16].copy_from_slice(&bssid);
        hw_mpdu[16..22].copy_from_slice(&sa);
        hw_mpdu[24] = 1; // PN0 = 1
        hw_mpdu[27] = 0x20; // ExtIV=1, KeyID=0
        hw_mpdu[32..40].copy_from_slice(&[0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00, 0x08, 0x00]);
        hw_mpdu[40..46].copy_from_slice(b"HWCCMP");
        sim_hw_rx_slot(
            drv,
            5,
            0xC4,
            0x00,
            hw_mpdu.len() as u16,
            hw_mpdu.len(),
            &hw_mpdu,
        );
        now_ms += 10;
        drv.poll(now_ms).expect("hw-decrypted data poll");
        assert_eq!(drv.take_last_sta_error(), None);
        assert_eq!(drv.rx_queue_len(), 1);

        let (rx_tok, _) = drv
            .receive(Instant::from_millis(now_ms as i64))
            .expect("smoltcp rx token");
        let mut seen = [0u8; 32];
        let seen_len = smoltcp::phy::RxToken::consume(rx_tok, |buf| {
            seen[..buf.len()].copy_from_slice(buf);
            buf.len()
        });
        assert_eq!(seen_len, 20);
        assert_eq!(&seen[0..6], &sta_mac);
        assert_eq!(&seen[6..12], &sa);
        assert_eq!(&seen[12..14], &[0x08, 0x00]);
        assert_eq!(&seen[14..20], b"HWCCMP");

        drv.disconnect(3, now_ms + 10).expect("disconnect");
        assert!(!drv.lmac().is_sta_hw_crypto_enabled());
    }
}
