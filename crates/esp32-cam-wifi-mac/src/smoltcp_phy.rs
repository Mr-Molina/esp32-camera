//! smoltcp physical device adapter and high-level 802.11 STA Wi-Fi driver.
//!
//! Provides [`Esp32StaWifiDriver`] bridging hardware LMAC DMA rings and
//! IEEE 802.11 STA state management to [`smoltcp::phy::Device`].

use core::fmt::Display;
use core::fmt::Formatter;
use core::fmt::Result as FmtResult;

use crate::dma_ring::DmaRingError;
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

/// smoltcp TX token providing a staging buffer for an outgoing Ethernet frame.
pub struct StaTxToken<'a, B: WifiMmioBus> {
    driver: &'a mut Esp32StaWifiDriver<B>,
}

impl<'a, B: WifiMmioBus> smoltcp::phy::TxToken for StaTxToken<'a, B> {
    fn consume<R, F: FnOnce(&mut [u8]) -> R>(self, len: usize, f: F) -> R {
        let copy_len = len.min(self.driver.smol_tx_buf.len());
        let result = f(&mut self.driver.smol_tx_buf[..copy_len]);
        self.driver.transmit_smol_tx_buf(copy_len);
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
        let desc_addr = self
            .tx_slot
            .tx_prepare(mpdu)
            .map_err(|_| StaError::BufferTooSmall)?;
        let dur = estimate_tx_duration_us(mpdu.len(), rate);
        let res = self
            .lmac
            .submit_and_poll_tx(desc_addr, mpdu.len(), rate, dur, wait_ack, 50_000);
        self.tx_slot.mark_completed();
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

    fn hw_random_u32(&mut self) -> u32 {
        self.lmac.hw_random_u32()
    }
}

/// High-level ESP32 802.11 STA Wi-Fi driver and smoltcp PHY device.
#[repr(C, align(4))]
pub struct Esp32StaWifiDriver<B: WifiMmioBus> {
    lmac: LmacHal<B>,
    rx_ring: WifiRxRing<8, 1600>,
    tx_slot: WifiTxSlot<1600>,
    sta: StaMacCore,
    rx_eth_queue: [[u8; 1536]; 8],
    rx_eth_lens: [u16; 8],
    rx_head: usize,
    rx_tail: usize,
    rx_count: usize,
    smol_tx_buf: [u8; 1536],
    now_ms: u64,
    initialized: bool,
    pending_eth_tx_failures: u32,
    rx_dropped_overflow: u32,
}

impl<B: WifiMmioBus> Esp32StaWifiDriver<B> {
    /// Creates an uninitialized Wi-Fi driver bound to the specified MMIO bus.
    pub const fn new(bus: B) -> Self {
        Self {
            lmac: LmacHal::new(bus),
            rx_ring: WifiRxRing::new(),
            tx_slot: WifiTxSlot::new(),
            sta: StaMacCore::new([0; 6]),
            rx_eth_queue: [[0u8; 1536]; 8],
            rx_eth_lens: [0u16; 8],
            rx_head: 0,
            rx_tail: 0,
            rx_count: 0,
            smol_tx_buf: [0u8; 1536],
            now_ms: 0,
            initialized: false,
            pending_eth_tx_failures: 0,
            rx_dropped_overflow: 0,
        }
    }

    /// Initializes hardware MAC, DMA rings, and baseband PHY.
    #[inline(never)]
    pub fn init(&mut self, custom_mac: Option<[u8; 6]>) -> Result<[u8; 6], DriverError> {
        self.rx_ring.arm_ring()?;
        crate::phy_bridge::phy_enable_rx();
        crate::phy_bridge::phy_reset_mac_and_coex();
        let mac = match custom_mac {
            Some(m) => m,
            None => self.lmac.read_efuse_sta_mac(),
        };
        self.sta.reset_with_mac(mac);
        let _ = crate::phy_bridge::phy_switch_channel_20mhz(1);
        self.lmac.init_mac(mac, self.rx_ring.base_desc_addr())?;
        crate::phy_bridge::phy_set_low_rate_enabled(true);
        self.lmac.enable_low_rate();
        crate::phy_bridge::phy_arm_bb_wdt(true);
        crate::phy_bridge::phy_set_max_tx_power_qdbm(78);
        self.initialized = true;
        Ok(mac)
    }

    /// Initiates association to an access point with the given configuration.
    pub fn connect(&mut self, config: StaConnectConfig, now_ms: u64) -> Result<(), DriverError> {
        if !self.initialized {
            return Err(DriverError::NotInitialized);
        }
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

        let mut scratch = [0u8; 1600];
        while let Some((ctrl, mpdu)) = self.rx_ring.pop_received() {
            let mpdu_len = mpdu.len().min(scratch.len());
            scratch[..mpdu_len].copy_from_slice(&mpdu[..mpdu_len]);
            let _ = self.rx_ring.recycle_current();
            self.lmac.reload_rx_dma();

            if mpdu_len <= 4 {
                continue;
            }

            let mut sink = DriverTxSink {
                lmac: &mut self.lmac,
                tx_slot: &mut self.tx_slot,
            };
            let _ = self.sta.handle_rx_mpdu(
                &scratch[..mpdu_len - 4],
                ctrl.rssi,
                now_ms,
                &mut sink,
                |eth| {
                    if self.rx_count < 8 {
                        let copy_len = eth.len().min(1536);
                        self.rx_eth_queue[self.rx_tail][..copy_len]
                            .copy_from_slice(&eth[..copy_len]);
                        self.rx_eth_lens[self.rx_tail] = copy_len as u16;
                        self.rx_tail = (self.rx_tail + 1) % 8;
                        self.rx_count += 1;
                    } else {
                        self.rx_dropped_overflow = self.rx_dropped_overflow.saturating_add(1);
                    }
                },
            );
        }

        let mut sink = DriverTxSink {
            lmac: &mut self.lmac,
            tx_slot: &mut self.tx_slot,
        };
        self.sta.poll_tick(now_ms, &mut sink)?;

        Ok(())
    }

    /// Transmits an Ethernet frame staged in [`Esp32StaWifiDriver::smol_tx_buf`].
    pub fn transmit_smol_tx_buf(&mut self, len: usize) {
        if !self.initialized || len == 0 || len > self.smol_tx_buf.len() {
            self.pending_eth_tx_failures = self.pending_eth_tx_failures.saturating_add(1);
            return;
        }
        let mut sink = DriverTxSink {
            lmac: &mut self.lmac,
            tx_slot: &mut self.tx_slot,
        };
        if self
            .sta
            .transmit_eth(&self.smol_tx_buf[..len], self.now_ms, &mut sink)
            .is_err()
        {
            self.pending_eth_tx_failures = self.pending_eth_tx_failures.saturating_add(1);
        }
    }

    /// Injects a mock MPDU into the driver receive pipeline for testing.
    pub fn push_mock_rx_mpdu(&mut self, mpdu: &[u8], rssi: i8) -> Result<(), DriverError> {
        if !self.initialized {
            return Err(DriverError::NotInitialized);
        }
        let mut sink = DriverTxSink {
            lmac: &mut self.lmac,
            tx_slot: &mut self.tx_slot,
        };
        self.sta
            .handle_rx_mpdu(mpdu, rssi, self.now_ms, &mut sink, |eth| {
                if self.rx_count < 8 {
                    let copy_len = eth.len().min(1536);
                    self.rx_eth_queue[self.rx_tail][..copy_len].copy_from_slice(&eth[..copy_len]);
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

    /// Shuts down the Wi-Fi baseband and resets hardware MAC state.
    pub fn shutdown(&mut self) {
        crate::phy_bridge::phy_disable_rx();
        crate::phy_bridge::phy_arm_bb_wdt(false);
        self.lmac.deinit_mac();
        self.initialized = false;
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
    use crate::dma_ring::WifiRxDmaDesc;
    use crate::lmac_mmio::MockWifiMmioBus;

    #[test]
    fn test_poll_fcs_stripping_and_runt_rejection() {
        let mut bus = MockWifiMmioBus::new();
        bus.auto_init_3d24 = true;
        let mut driver = Esp32StaWifiDriver::new(bus);
        assert!(driver
            .init(Some([0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc]))
            .is_ok());

        // Slot 0: Runt frame with <= 4 bytes (4 bytes payload + 28 bytes header).
        if let Some(buf) = driver.rx_ring.raw_buffer_mut(0) {
            let mut words = [0u32; 7];
            words[0] = 0xC4; // rssi = -60
            words[6] = 4; // sig_len = 4
            for (w_idx, &val) in words.iter().enumerate() {
                let off = w_idx * 4;
                buf[off..off + 4].copy_from_slice(&val.to_le_bytes());
            }
            buf[28..32].copy_from_slice(&[0x11, 0x22, 0x33, 0x44]);
        }
        if let Some(desc) = driver.rx_ring.desc_mut(0) {
            desc.word0 = WifiRxDmaDesc::EOF_BIT | ((1600 & 0xFFF) << 12) | (28 + 4);
        }

        // Slot 1: Valid frame with > 4 bytes payload (including 4-byte FCS).
        // 10 bytes mock 802.11 MPDU + 4 bytes FCS (0xAA, 0xBB, 0xCC, 0xDD).
        if let Some(buf) = driver.rx_ring.raw_buffer_mut(1) {
            let mut words = [0u32; 7];
            words[0] = 0xC4; // rssi = -60
            words[6] = 14; // sig_len = 10-byte MPDU + 4-byte FCS = 14
            for (w_idx, &val) in words.iter().enumerate() {
                let off = w_idx * 4;
                buf[off..off + 4].copy_from_slice(&val.to_le_bytes());
            }
            buf[28..38]
                .copy_from_slice(&[0x80, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]);
            buf[38..42].copy_from_slice(&[0xAA, 0xBB, 0xCC, 0xDD]);
        }
        if let Some(desc) = driver.rx_ring.desc_mut(1) {
            desc.word0 = WifiRxDmaDesc::EOF_BIT | ((1600 & 0xFFF) << 12) | (28 + 14);
        }

        // Poll should process slot 0 (dropping runt and recycling it) and slot 1.
        assert!(driver.poll(100).is_ok());

        // Verify both descriptors have been recycled back to HW ownership.
        if let Some(d0) = driver.rx_ring.desc(0) {
            assert!(d0.is_owner_hw());
        }
        if let Some(d1) = driver.rx_ring.desc(1) {
            assert!(d1.is_owner_hw());
        }

        // Verify rx_ring is now empty.
        assert!(driver.rx_ring.pop_received().is_none());

        // Forward getters check.
        assert_eq!(driver.rssi(), -127);
        assert_eq!(driver.aid(), 0);
        assert_eq!(driver.scan_candidates().len(), 0);
    }

    #[test]
    fn test_init_enables_11b_low_rate() {
        let mut bus = MockWifiMmioBus::new();
        bus.auto_init_3d24 = true;
        let mut driver = Esp32StaWifiDriver::new(bus);
        assert!(driver
            .init(Some([0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc]))
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
}
