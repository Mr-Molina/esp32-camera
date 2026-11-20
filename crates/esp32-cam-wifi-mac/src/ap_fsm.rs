//! Pure-Rust IEEE 802.11 Open SoftAP MAC Core State Machine.
//!
//! Implements zero-allocation, `#![no_std]` Open Access Point management
//! (periodic Beacons, Probe Responses, Open System Authentication,
//! Association/Reassociation, client table with idle and LRU eviction) and
//! unencrypted `ToDS`/`FromDS` Ethernet-II data bridging.

use core::fmt;

use crate::frames_80211::ap_eth_to_80211_data;
use crate::frames_80211::ap_rx_80211_data_to_eth;
use crate::frames_80211::build_ap_deauth;
use crate::frames_80211::build_assoc_resp;
use crate::frames_80211::build_beacon;
use crate::frames_80211::build_open_auth_resp;
use crate::frames_80211::build_probe_resp;
use crate::frames_80211::build_reassoc_resp;
use crate::frames_80211::parse_80211_header;
use crate::frames_80211::parse_assoc_req;
use crate::frames_80211::parse_deauth_or_disassoc;
use crate::frames_80211::parse_open_auth_req;
use crate::frames_80211::parse_probe_req;
use crate::frames_80211::FrameError;
use crate::frames_80211::FrameType;
use crate::lmac_mmio::LmacError;
use crate::regs::WifiPhyRate;

/// Maximum number of concurrent SoftAP stations tracked by [`ApMacCore`].
pub const MAX_AP_CLIENTS: usize = 4;

/// Beacon transmit cadence in milliseconds (100 ms / ~100 TU).
const BEACON_INTERVAL_MS: u64 = 100;

/// Beacon Interval field encoded in Beacon and Probe Response frames (TU).
const BEACON_INTERVAL_TU: u16 = 100;

/// Inactivity timeout after which an idle client is evicted (60 seconds).
const CLIENT_IDLE_TIMEOUT_MS: u64 = 60_000;

/// IEEE 802.11 broadcast MAC address (`FF:FF:FF:FF:FF:FF`).
const BROADCAST_MAC: [u8; 6] = [0xFF; 6];

/// IEEE 802.11 Status Code 0: Successful.
const STATUS_SUCCESS: u16 = 0;

/// IEEE 802.11 Status Code 13: Authentication algorithm not supported.
const STATUS_UNSUPPORTED_AUTH_ALG: u16 = 13;

/// IEEE 802.11 Reason Code 3: Deauthenticated because sending STA is leaving.
const REASON_DEAUTH_LEAVING: u16 = 3;

/// SoftAP state machine errors.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ApError {
    /// Invalid AP configuration (empty/oversized SSID or invalid channel).
    InvalidConfig,
    /// 802.11 frame parsing or construction error.
    Frame(FrameError),
    /// Low-level LMAC hardware transmit failure.
    Hardware(LmacError),
    /// Unicast destination MAC is not in the associated client table.
    ClientNotAssociated,
    /// Scratch or transmit buffer capacity is insufficient.
    BufferTooSmall,
}

impl fmt::Display for ApError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidConfig => write!(f, "invalid SoftAP configuration"),
            Self::Frame(e) => write!(f, "SoftAP frame error: {e}"),
            Self::Hardware(e) => write!(f, "SoftAP hardware error: {e}"),
            Self::ClientNotAssociated => write!(f, "destination STA is not associated"),
            Self::BufferTooSmall => write!(f, "SoftAP buffer too small"),
        }
    }
}

impl From<FrameError> for ApError {
    fn from(err: FrameError) -> Self {
        Self::Frame(err)
    }
}

impl From<LmacError> for ApError {
    fn from(err: LmacError) -> Self {
        Self::Hardware(err)
    }
}

/// Configuration parameters for an Open SoftAP instance.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ApConfig {
    /// MAC address / BSSID of the SoftAP interface.
    pub ap_mac: [u8; 6],
    /// UTF-8 SSID bytes (only `ssid[..ssid_len]` are active).
    pub ssid: [u8; 32],
    /// Active length of [`Self::ssid`] in bytes (`1..=32`).
    pub ssid_len: usize,
    /// Operating 20 MHz 2.4 GHz Wi-Fi channel (`1..=11`).
    pub channel: u8,
}

impl ApConfig {
    /// Builds a setup-mode AP configuration with SSID `"KlipperCam-Setup-XXXX"`
    /// where `XXXX` is the last two bytes of `mac` in uppercase hexadecimal.
    #[must_use]
    pub const fn for_setup_mac(mac: [u8; 6], channel: u8) -> Self {
        const PREFIX: &[u8] = b"KlipperCam-Setup-";
        const HEX: &[u8; 16] = b"0123456789ABCDEF";

        let mut ssid = [0u8; 32];
        let mut idx = 0usize;
        while idx < PREFIX.len() {
            ssid[idx] = PREFIX[idx];
            idx += 1;
        }
        ssid[17] = HEX[(mac[4] >> 4) as usize];
        ssid[18] = HEX[(mac[4] & 0x0F) as usize];
        ssid[19] = HEX[(mac[5] >> 4) as usize];
        ssid[20] = HEX[(mac[5] & 0x0F) as usize];

        Self {
            ap_mac: mac,
            ssid,
            ssid_len: 21,
            channel,
        }
    }

    /// Returns the active SSID byte slice.
    #[must_use]
    pub fn ssid_bytes(&self) -> &[u8] {
        let len = self.ssid_len.min(self.ssid.len());
        &self.ssid[..len]
    }
}

/// Diagnostic TX/RX counters for [`ApMacCore`].
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ApStats {
    /// Number of Beacon frames transmitted successfully.
    pub beacons_ok: u32,
    /// Number of Beacon frame transmissions that failed.
    pub beacons_err: u32,
    /// Number of matching Probe Request frames received.
    pub probe_req_rx: u32,
    /// Number of Probe Response frames transmitted successfully.
    pub probe_resp_ok: u32,
    /// Number of Probe Response frame transmissions that failed.
    pub probe_resp_err: u32,
    /// Total number of received MPDUs passed to [`ApMacCore::handle_rx_mpdu`].
    pub rx_frames: u32,
    /// Most recent low-level LMAC transmit error observed by the AP FSM.
    pub last_tx_err: Option<LmacError>,
}

/// Hardware transmit sink interface consumed by [`ApMacCore`].
pub trait ApTxSink {
    /// Transmits a serialized 802.11 MPDU at the specified PHY rate.
    ///
    /// # Errors
    /// Returns [`ApError`] if the underlying radio or DMA slot fails.
    fn send_mpdu(&mut self, mpdu: &[u8], rate: WifiPhyRate, wait_ack: bool) -> Result<(), ApError>;
}

/// Association lifecycle state for a tracked SoftAP station.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ApClientState {
    Authenticated,
    Associated,
}

/// Entry in the fixed-size SoftAP client table.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct ApClient {
    mac: [u8; 6],
    state: ApClientState,
    aid: u16,
    last_seen_ms: u64,
}

/// Pure-Rust Open SoftAP 802.11 MAC state machine.
pub struct ApMacCore {
    config: ApConfig,
    clients: [Option<ApClient>; MAX_AP_CLIENTS],
    tx_seq: u16,
    last_beacon_ms: Option<u64>,
    last_activity_ms: u64,
    stats: ApStats,
    tx_mpdu_buf: [u8; 1600],
}

impl ApMacCore {
    /// Creates a new Open SoftAP MAC core after validating `cfg`.
    ///
    /// # Errors
    /// Returns [`ApError::InvalidConfig`] if `cfg.ssid_len` is outside `1..=32`
    /// or `cfg.channel` is outside `1..=11`.
    pub fn new(cfg: ApConfig) -> Result<Self, ApError> {
        if cfg.ssid_len == 0 || cfg.ssid_len > 32 || !(1..=11).contains(&cfg.channel) {
            return Err(ApError::InvalidConfig);
        }
        Ok(Self {
            config: cfg,
            clients: [None; MAX_AP_CLIENTS],
            tx_seq: 0,
            last_beacon_ms: None,
            last_activity_ms: 0,
            stats: ApStats::default(),
            tx_mpdu_buf: [0u8; 1600],
        })
    }

    /// Returns the immutable SoftAP configuration.
    #[must_use]
    pub const fn config(&self) -> &ApConfig {
        &self.config
    }

    /// Returns a snapshot of the SoftAP TX/RX diagnostic counters.
    #[must_use]
    pub const fn stats(&self) -> ApStats {
        self.stats
    }

    /// Returns the number of currently associated stations (`0..=4`).
    #[must_use]
    pub fn associated_client_count(&self) -> usize {
        let mut count = 0usize;
        for slot in &self.clients {
            let Some(client) = slot else {
                continue;
            };
            if client.state == ApClientState::Associated {
                count += 1;
            }
        }
        count
    }

    /// Returns the timestamp (in milliseconds) of the last client auth,
    /// association, or valid data activity.
    #[must_use]
    pub const fn last_activity_ms(&self) -> u64 {
        self.last_activity_ms
    }

    /// Advances SoftAP timers: expires idle stations (>= 60 s) and emits
    /// periodic Beacons every 100 ms (`Rate1mL`, `wait_ack = false`).
    pub fn poll_tick(&mut self, now_ms: u64, sink: &mut impl ApTxSink) {
        self.expire_idle_clients(now_ms);

        let should_beacon = match self.last_beacon_ms {
            None => true,
            Some(last) => now_ms.saturating_sub(last) >= BEACON_INTERVAL_MS,
        };
        if !should_beacon {
            return;
        }

        let seq = self.next_seq();
        let mut buf = [0u8; 128];
        if let Ok(len) = build_beacon(
            &self.config.ap_mac,
            self.config.ssid_bytes(),
            self.config.channel,
            BEACON_INTERVAL_TU,
            now_ms.saturating_mul(1_000),
            seq,
            &mut buf,
        ) {
            match sink.send_mpdu(&buf[..len], WifiPhyRate::Rate1mL, false) {
                Ok(()) => {
                    self.stats.beacons_ok = self.stats.beacons_ok.saturating_add(1);
                }
                Err(err) => {
                    self.stats.beacons_err = self.stats.beacons_err.saturating_add(1);
                    self.record_tx_err(err);
                }
            }
        }
        self.last_beacon_ms = Some(now_ms);
    }

    /// Processes a received 802.11 MPDU (FCS already stripped).
    ///
    /// Management frames are answered via `sink`. Valid unencrypted `ToDS=1`
    /// Data/QoS-Data frames from associated stations are decapsulated into
    /// `eth_out` and return `Some(eth_len)`. All other frames return `None`.
    pub fn handle_rx_mpdu(
        &mut self,
        raw_mpdu: &[u8],
        now_ms: u64,
        sink: &mut impl ApTxSink,
        eth_out: &mut [u8],
    ) -> Option<usize> {
        self.stats.rx_frames = self.stats.rx_frames.saturating_add(1);
        let parsed = parse_80211_header(raw_mpdu).ok()?;
        // Reject frames originating from our own MAC or a multicast/broadcast TA.
        if parsed.addr2 == self.config.ap_mac || (parsed.addr2[0] & 0x01) != 0 {
            return None;
        }

        match parsed.frame_type {
            FrameType::Mgmt => {
                // Reject management frames with DS bits, Protected bit, or fragments.
                let more_frag = (raw_mpdu[1] & 0x04) != 0;
                let frag_num = parsed.seq_ctrl & 0x000F;
                let invalid_mgmt = parsed.to_ds
                    || parsed.from_ds
                    || parsed.is_protected
                    || more_frag
                    || frag_num != 0;
                if invalid_mgmt {
                    return None;
                }
                self.handle_mgmt_frame(raw_mpdu, &parsed, now_ms, sink);
                None
            }
            FrameType::Data => self.handle_data_frame(raw_mpdu, &parsed, now_ms, eth_out),
            FrameType::Control | FrameType::Extension => None,
        }
    }

    /// Encapsulates and transmits an outbound Ethernet-II frame from the SoftAP.
    ///
    /// Broadcast and multicast frames transmit at `Rate1mL` without ACK.
    /// Unicast frames require the destination MAC to be associated and transmit
    /// at `Rate6m` with a single `Rate1mL` fallback retry on non-drain failures.
    ///
    /// # Errors
    /// Returns [`ApError::ClientNotAssociated`] for unicast frames targeting an
    /// unassociated station, or propagates frame/hardware errors.
    pub fn transmit_eth(
        &mut self,
        eth_frame: &[u8],
        sink: &mut impl ApTxSink,
    ) -> Result<(), ApError> {
        if eth_frame.len() < 14 {
            return Err(ApError::Frame(FrameError::InvalidPayload));
        }
        let mut da = [0u8; 6];
        da.copy_from_slice(&eth_frame[0..6]);
        let is_group = (da[0] & 0x01) != 0;

        if !is_group && !self.is_associated(&da) {
            return Err(ApError::ClientNotAssociated);
        }

        let seq = self.next_seq();
        let mpdu_len =
            ap_eth_to_80211_data(eth_frame, &self.config.ap_mac, seq, &mut self.tx_mpdu_buf)?;

        if is_group {
            return sink.send_mpdu(&self.tx_mpdu_buf[..mpdu_len], WifiPhyRate::Rate1mL, false);
        }

        match sink.send_mpdu(&self.tx_mpdu_buf[..mpdu_len], WifiPhyRate::Rate6m, true) {
            Ok(()) => Ok(()),
            Err(ApError::Hardware(LmacError::TxDrainFailed)) => {
                Err(ApError::Hardware(LmacError::TxDrainFailed))
            }
            Err(_) => sink.send_mpdu(&self.tx_mpdu_buf[..mpdu_len], WifiPhyRate::Rate1mL, true),
        }
    }

    /// Sends a best-effort Deauthentication frame to every tracked client and
    /// clears the client table.
    pub fn deauth_all(&mut self, sink: &mut impl ApTxSink) {
        let mut buf = [0u8; 32];
        for idx in 0..MAX_AP_CLIENTS {
            if let Some(client) = self.clients[idx].take() {
                let seq = self.next_seq();
                if let Ok(len) = build_ap_deauth(
                    &client.mac,
                    &self.config.ap_mac,
                    REASON_DEAUTH_LEAVING,
                    seq,
                    &mut buf,
                ) {
                    let _ = sink.send_mpdu(&buf[..len], WifiPhyRate::Rate1mL, false);
                }
            }
        }
    }

    fn handle_mgmt_frame(
        &mut self,
        raw_mpdu: &[u8],
        parsed: &crate::frames_80211::ParsedFrame<'_>,
        now_ms: u64,
        sink: &mut impl ApTxSink,
    ) {
        match parsed.subtype {
            0x04 => self.handle_probe_req(parsed, now_ms, sink),
            0x0B => self.handle_auth_req(parsed, now_ms, sink),
            0x00 => self.handle_assoc_req(parsed, now_ms, sink, false),
            0x02 => self.handle_assoc_req(parsed, now_ms, sink, true),
            0x0A | 0x0C => self.handle_deauth_or_disassoc(raw_mpdu, parsed),
            _ => {}
        }
    }

    fn handle_probe_req(
        &mut self,
        parsed: &crate::frames_80211::ParsedFrame<'_>,
        now_ms: u64,
        sink: &mut impl ApTxSink,
    ) {
        let dst_matches = parsed.addr1 == BROADCAST_MAC || parsed.addr1 == self.config.ap_mac;
        let bssid_matches = parsed.addr3 == BROADCAST_MAC || parsed.addr3 == self.config.ap_mac;
        if !dst_matches || !bssid_matches {
            return;
        }
        let Ok(info) = parse_probe_req(parsed.payload) else {
            return;
        };
        let ssid_matches = match info.ssid {
            None => true,
            Some(req_ssid) => req_ssid == self.config.ssid_bytes(),
        };
        if !ssid_matches {
            return;
        }

        self.stats.probe_req_rx = self.stats.probe_req_rx.saturating_add(1);

        let seq = self.next_seq();
        let mut buf = [0u8; 128];
        if let Ok(len) = build_probe_resp(
            &parsed.addr2,
            &self.config.ap_mac,
            self.config.ssid_bytes(),
            self.config.channel,
            BEACON_INTERVAL_TU,
            now_ms.saturating_mul(1_000),
            seq,
            &mut buf,
        ) {
            match sink.send_mpdu(&buf[..len], WifiPhyRate::Rate1mL, false) {
                Ok(()) => {
                    self.stats.probe_resp_ok = self.stats.probe_resp_ok.saturating_add(1);
                }
                Err(err) => {
                    self.stats.probe_resp_err = self.stats.probe_resp_err.saturating_add(1);
                    self.record_tx_err(err);
                }
            }
        }
    }

    fn handle_auth_req(
        &mut self,
        parsed: &crate::frames_80211::ParsedFrame<'_>,
        now_ms: u64,
        sink: &mut impl ApTxSink,
    ) {
        if parsed.addr1 != self.config.ap_mac || parsed.addr3 != self.config.ap_mac {
            return;
        }
        let Ok((auth_alg, auth_seq)) = parse_open_auth_req(parsed.payload) else {
            return;
        };

        self.expire_idle_clients(now_ms);

        let mut buf = [0u8; 40];
        if auth_alg != 0 {
            let seq = self.next_seq();
            if let Ok(len) = build_open_auth_resp(
                &parsed.addr2,
                &self.config.ap_mac,
                2,
                STATUS_UNSUPPORTED_AUTH_ALG,
                seq,
                &mut buf,
            ) {
                let _ = sink.send_mpdu(&buf[..len], WifiPhyRate::Rate1mL, true);
            }
            return;
        }

        if auth_seq != 1 {
            return;
        }

        self.upsert_authenticated_client(parsed.addr2, now_ms);
        self.last_activity_ms = now_ms;

        let seq = self.next_seq();
        if let Ok(len) = build_open_auth_resp(
            &parsed.addr2,
            &self.config.ap_mac,
            2,
            STATUS_SUCCESS,
            seq,
            &mut buf,
        ) {
            let _ = sink.send_mpdu(&buf[..len], WifiPhyRate::Rate1mL, true);
        }
    }

    fn handle_assoc_req(
        &mut self,
        parsed: &crate::frames_80211::ParsedFrame<'_>,
        now_ms: u64,
        sink: &mut impl ApTxSink,
        is_reassoc: bool,
    ) {
        if parsed.addr1 != self.config.ap_mac || parsed.addr3 != self.config.ap_mac {
            return;
        }

        let req_ssid = if is_reassoc {
            // ReassocReq fixed body: Cap (2) + Listen Interval (2) + Current AP (6).
            if parsed.payload.len() < 10 {
                return;
            }
            let Ok(info) = parse_probe_req(&parsed.payload[10..]) else {
                return;
            };
            let Some(ssid) = info.ssid else {
                return;
            };
            ssid
        } else {
            let Ok(info) = parse_assoc_req(parsed.payload) else {
                return;
            };
            info.ssid
        };

        if req_ssid != self.config.ssid_bytes() {
            return;
        }

        self.expire_idle_clients(now_ms);

        let Some(slot_idx) = self.find_client_slot(&parsed.addr2) else {
            return;
        };
        let aid = (slot_idx as u16) + 1;
        self.clients[slot_idx] = Some(ApClient {
            mac: parsed.addr2,
            state: ApClientState::Associated,
            aid,
            last_seen_ms: now_ms,
        });
        self.last_activity_ms = now_ms;

        let seq = self.next_seq();
        let mut buf = [0u8; 64];
        let resp_res = if is_reassoc {
            build_reassoc_resp(
                &parsed.addr2,
                &self.config.ap_mac,
                aid,
                STATUS_SUCCESS,
                seq,
                &mut buf,
            )
        } else {
            build_assoc_resp(
                &parsed.addr2,
                &self.config.ap_mac,
                aid,
                STATUS_SUCCESS,
                seq,
                &mut buf,
            )
        };
        if let Ok(len) = resp_res {
            let _ = sink.send_mpdu(&buf[..len], WifiPhyRate::Rate1mL, true);
        }
    }

    fn handle_deauth_or_disassoc(
        &mut self,
        raw_mpdu: &[u8],
        parsed: &crate::frames_80211::ParsedFrame<'_>,
    ) {
        if parsed.addr1 != self.config.ap_mac || parsed.addr3 != self.config.ap_mac {
            return;
        }
        if parse_deauth_or_disassoc(raw_mpdu).is_err() {
            return;
        }
        if let Some(idx) = self.find_client_slot(&parsed.addr2) {
            self.clients[idx] = None;
        }
    }

    fn handle_data_frame(
        &mut self,
        raw_mpdu: &[u8],
        parsed: &crate::frames_80211::ParsedFrame<'_>,
        now_ms: u64,
        eth_out: &mut [u8],
    ) -> Option<usize> {
        if !parsed.to_ds || parsed.from_ds || parsed.addr1 != self.config.ap_mac {
            return None;
        }

        self.expire_idle_clients(now_ms);

        let slot_idx = self.find_client_slot(&parsed.addr2)?;
        let client = self.clients[slot_idx]?;
        if client.state != ApClientState::Associated {
            return None;
        }

        // Null Data / QoS Null keepalive refreshes station activity timestamp.
        if (parsed.subtype == 0x04 || parsed.subtype == 0x0C) && !parsed.is_protected {
            if let Some(entry) = &mut self.clients[slot_idx] {
                entry.last_seen_ms = now_ms;
            }
            self.last_activity_ms = now_ms;
            return None;
        }

        let eth_len =
            ap_rx_80211_data_to_eth(raw_mpdu, &self.config.ap_mac, &client.mac, eth_out).ok()?;
        if let Some(entry) = &mut self.clients[slot_idx] {
            entry.last_seen_ms = now_ms;
        }
        self.last_activity_ms = now_ms;
        Some(eth_len)
    }

    fn expire_idle_clients(&mut self, now_ms: u64) {
        for slot in &mut self.clients {
            if let Some(client) = *slot {
                if now_ms.saturating_sub(client.last_seen_ms) >= CLIENT_IDLE_TIMEOUT_MS {
                    *slot = None;
                }
            }
        }
    }

    fn find_client_slot(&self, mac: &[u8; 6]) -> Option<usize> {
        for (idx, slot) in self.clients.iter().enumerate() {
            if let Some(client) = slot {
                if client.mac == *mac {
                    return Some(idx);
                }
            }
        }
        None
    }

    fn is_associated(&self, mac: &[u8; 6]) -> bool {
        let Some(idx) = self.find_client_slot(mac) else {
            return false;
        };
        matches!(
            self.clients[idx],
            Some(ApClient {
                state: ApClientState::Associated,
                ..
            })
        )
    }

    fn upsert_authenticated_client(&mut self, mac: [u8; 6], now_ms: u64) {
        if let Some(idx) = self.find_client_slot(&mac) {
            self.clients[idx] = Some(ApClient {
                mac,
                state: ApClientState::Authenticated,
                aid: 0,
                last_seen_ms: now_ms,
            });
            return;
        }

        for idx in 0..MAX_AP_CLIENTS {
            if self.clients[idx].is_none() {
                self.clients[idx] = Some(ApClient {
                    mac,
                    state: ApClientState::Authenticated,
                    aid: 0,
                    last_seen_ms: now_ms,
                });
                return;
            }
        }

        // Table full: evict the least-recently-active slot.
        let mut lru_idx = 0usize;
        let mut oldest_ms = u64::MAX;
        for (idx, slot) in self.clients.iter().enumerate() {
            if let Some(client) = slot {
                if client.last_seen_ms < oldest_ms {
                    oldest_ms = client.last_seen_ms;
                    lru_idx = idx;
                }
            }
        }
        self.clients[lru_idx] = Some(ApClient {
            mac,
            state: ApClientState::Authenticated,
            aid: 0,
            last_seen_ms: now_ms,
        });
    }

    fn next_seq(&mut self) -> u16 {
        let seq = self.tx_seq;
        self.tx_seq = self.tx_seq.wrapping_add(1) & 0x0FFF;
        seq
    }

    fn record_tx_err(&mut self, err: ApError) {
        if let ApError::Hardware(lmac_err) = err {
            self.stats.last_tx_err = Some(lmac_err);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frames_80211::build_assoc_req;
    use crate::frames_80211::build_deauth;
    use crate::frames_80211::build_open_auth_req;
    use crate::frames_80211::build_probe_req;
    use crate::frames_80211::eth_to_80211_data;
    use crate::frames_80211::parse_assoc_resp;
    use crate::frames_80211::parse_auth_resp;
    use crate::frames_80211::parse_beacon_or_probe_resp;
    use crate::frames_80211::parse_reassoc_resp;

    const AP_MAC: [u8; 6] = [0xD4, 0xD4, 0xDA, 0x5B, 0xD4, 0xEC];

    struct RecordingSink {
        frames: [[u8; 1600]; 16],
        lens: [usize; 16],
        rates: [WifiPhyRate; 16],
        wait_acks: [bool; 16],
        count: usize,
        fail_first_rate6m: bool,
        rate6m_attempts: usize,
    }

    impl RecordingSink {
        fn new() -> Self {
            Self {
                frames: [[0u8; 1600]; 16],
                lens: [0usize; 16],
                rates: [WifiPhyRate::Rate1mL; 16],
                wait_acks: [false; 16],
                count: 0,
                fail_first_rate6m: false,
                rate6m_attempts: 0,
            }
        }

        fn clear(&mut self) {
            self.count = 0;
        }

        fn last_frame(&self) -> &[u8] {
            assert!(self.count > 0, "expected at least one recorded TX frame");
            let idx = self.count - 1;
            &self.frames[idx][..self.lens[idx]]
        }
    }

    impl ApTxSink for RecordingSink {
        fn send_mpdu(
            &mut self,
            mpdu: &[u8],
            rate: WifiPhyRate,
            wait_ack: bool,
        ) -> Result<(), ApError> {
            assert!(
                self.count < self.frames.len(),
                "RecordingSink frame capacity exceeded"
            );
            let idx = self.count;
            self.frames[idx][..mpdu.len()].copy_from_slice(mpdu);
            self.lens[idx] = mpdu.len();
            self.rates[idx] = rate;
            self.wait_acks[idx] = wait_ack;
            self.count += 1;

            if rate == WifiPhyRate::Rate6m {
                self.rate6m_attempts += 1;
                if self.fail_first_rate6m && self.rate6m_attempts == 1 {
                    return Err(ApError::Hardware(LmacError::TxTimeout));
                }
            }
            Ok(())
        }
    }

    fn associate_sta(
        ap: &mut ApMacCore,
        sink: &mut RecordingSink,
        sta_mac: [u8; 6],
        now_ms: u64,
    ) -> u16 {
        let mut auth = [0u8; 64];
        let auth_len = build_open_auth_req(&mut auth, AP_MAC, sta_mac, 1).expect("build auth");
        let mut eth_out = [0u8; 1536];
        assert_eq!(
            ap.handle_rx_mpdu(&auth[..auth_len], now_ms, sink, &mut eth_out),
            None
        );
        let (auth_seq, auth_status) = parse_auth_resp(sink.last_frame()).expect("parse auth resp");
        assert_eq!(auth_seq, 2);
        assert_eq!(auth_status, 0);

        let mut assoc = [0u8; 128];
        let ssid = ap.config().ssid_bytes();
        let assoc_len =
            build_assoc_req(&mut assoc, AP_MAC, sta_mac, ssid, &[], 2, 10).expect("build assoc");
        assert_eq!(
            ap.handle_rx_mpdu(&assoc[..assoc_len], now_ms, sink, &mut eth_out),
            None
        );
        let (assoc_status, aid) = parse_assoc_resp(sink.last_frame()).expect("parse assoc resp");
        assert_eq!(assoc_status, 0);
        aid
    }

    fn make_sta_data_mpdu(
        sta_mac: [u8; 6],
        dst_mac: [u8; 6],
        payload: &[u8],
    ) -> ([u8; 256], usize) {
        let mut eth = [0u8; 128];
        eth[0..6].copy_from_slice(&dst_mac);
        eth[6..12].copy_from_slice(&sta_mac);
        eth[12..14].copy_from_slice(&[0x08, 0x00]);
        eth[14..14 + payload.len()].copy_from_slice(payload);
        let mut mpdu = [0u8; 256];
        let len = eth_to_80211_data(&mut mpdu, &eth[..14 + payload.len()], AP_MAC, 7, None)
            .expect("build sta data");
        (mpdu, len)
    }

    #[test]
    fn test_setup_config_ssid_formatting_and_validation() {
        let cfg = ApConfig::for_setup_mac(AP_MAC, 1);
        assert_eq!(cfg.ssid_bytes(), b"KlipperCam-Setup-D4EC");
        assert_eq!(cfg.channel, 1);
        assert!(ApMacCore::new(cfg).is_ok());

        let mut bad_ch = cfg;
        bad_ch.channel = 0;
        assert_eq!(ApMacCore::new(bad_ch).err(), Some(ApError::InvalidConfig));
        bad_ch.channel = 12;
        assert_eq!(ApMacCore::new(bad_ch).err(), Some(ApError::InvalidConfig));

        let mut bad_ssid = cfg;
        bad_ssid.ssid_len = 0;
        assert_eq!(ApMacCore::new(bad_ssid).err(), Some(ApError::InvalidConfig));
        bad_ssid.ssid_len = 33;
        assert_eq!(ApMacCore::new(bad_ssid).err(), Some(ApError::InvalidConfig));
    }

    #[test]
    fn test_beacon_100ms_cadence() {
        let cfg = ApConfig::for_setup_mac(AP_MAC, 6);
        let mut ap = ApMacCore::new(cfg).expect("ap");
        let mut sink = RecordingSink::new();

        ap.poll_tick(0, &mut sink);
        assert_eq!(sink.count, 1);
        assert_eq!(sink.rates[0], WifiPhyRate::Rate1mL);
        assert!(!sink.wait_acks[0]);
        let info = parse_beacon_or_probe_resp(sink.last_frame(), -30).expect("beacon");
        assert_eq!(info.bssid, AP_MAC);
        assert_eq!(info.ssid, b"KlipperCam-Setup-D4EC");
        assert_eq!(info.channel, Some(6));
        assert!(!info.privacy);

        ap.poll_tick(50, &mut sink);
        assert_eq!(sink.count, 1);
        ap.poll_tick(99, &mut sink);
        assert_eq!(sink.count, 1);
        ap.poll_tick(100, &mut sink);
        assert_eq!(sink.count, 2);
        assert_eq!(sink.rates[1], WifiPhyRate::Rate1mL);
        assert!(!sink.wait_acks[1]);
    }

    #[test]
    fn test_probe_req_wildcard_exact_and_mismatch() {
        let cfg = ApConfig::for_setup_mac(AP_MAC, 1);
        let mut ap = ApMacCore::new(cfg).expect("ap");
        let mut sink = RecordingSink::new();
        let mut eth_out = [0u8; 1536];
        let sta = [0x02, 0x11, 0x22, 0x33, 0x44, 0x55];

        let mut req = [0u8; 128];
        let len = build_probe_req(&mut req, sta, None, 1).expect("wildcard probe");
        ap.handle_rx_mpdu(&req[..len], 10, &mut sink, &mut eth_out);
        assert_eq!(sink.count, 1);
        assert_eq!(sink.rates[0], WifiPhyRate::Rate1mL);
        assert!(!sink.wait_acks[0]);
        let resp = parse_beacon_or_probe_resp(sink.last_frame(), -40).expect("probe resp");
        assert_eq!(resp.ssid, b"KlipperCam-Setup-D4EC");

        let len =
            build_probe_req(&mut req, sta, Some(b"KlipperCam-Setup-D4EC"), 2).expect("exact probe");
        ap.handle_rx_mpdu(&req[..len], 20, &mut sink, &mut eth_out);
        assert_eq!(sink.count, 2);
        assert!(!sink.wait_acks[1]);

        let len = build_probe_req(&mut req, sta, Some(b"WrongNetwork"), 3).expect("other probe");
        ap.handle_rx_mpdu(&req[..len], 30, &mut sink, &mut eth_out);
        assert_eq!(sink.count, 2);
    }

    #[test]
    fn test_open_auth_and_unsupported_alg_status_13() {
        let cfg = ApConfig::for_setup_mac(AP_MAC, 1);
        let mut ap = ApMacCore::new(cfg).expect("ap");
        let mut sink = RecordingSink::new();
        let mut eth_out = [0u8; 1536];
        let sta = [0x02, 0xAA, 0xBB, 0xCC, 0xDD, 0x01];

        let mut auth = [0u8; 64];
        let len = build_open_auth_req(&mut auth, AP_MAC, sta, 1).expect("auth");
        // Set non-zero Auth Algorithm (Shared Key = 1).
        auth[24..26].copy_from_slice(&1u16.to_le_bytes());
        ap.handle_rx_mpdu(&auth[..len], 100, &mut sink, &mut eth_out);
        assert_eq!(sink.count, 1);
        assert!(sink.wait_acks[0]);
        let (seq, status) = parse_auth_resp(sink.last_frame()).expect("auth reject");
        assert_eq!(seq, 2);
        assert_eq!(status, 13);
        assert_eq!(ap.associated_client_count(), 0);

        // Now send valid Open Auth (alg = 0, seq = 1).
        auth[24..26].copy_from_slice(&0u16.to_le_bytes());
        ap.handle_rx_mpdu(&auth[..len], 110, &mut sink, &mut eth_out);
        assert_eq!(sink.count, 2);
        let (seq2, status2) = parse_auth_resp(sink.last_frame()).expect("auth ok");
        assert_eq!(seq2, 2);
        assert_eq!(status2, 0);
        assert_eq!(ap.last_activity_ms(), 110);
    }

    #[test]
    fn test_assoc_reassoc_deauth_and_data_bridge() {
        let cfg = ApConfig::for_setup_mac(AP_MAC, 1);
        let mut ap = ApMacCore::new(cfg).expect("ap");
        let mut sink = RecordingSink::new();
        let mut eth_out = [0u8; 1536];
        let sta = [0x02, 0xAA, 0xBB, 0xCC, 0xDD, 0x02];

        // Assoc without prior Auth MUST be ignored.
        let mut assoc = [0u8; 128];
        let assoc_len = build_assoc_req(
            &mut assoc,
            AP_MAC,
            sta,
            b"KlipperCam-Setup-D4EC",
            &[],
            1,
            10,
        )
        .expect("assoc");
        ap.handle_rx_mpdu(&assoc[..assoc_len], 10, &mut sink, &mut eth_out);
        assert_eq!(sink.count, 0);
        assert_eq!(ap.associated_client_count(), 0);

        // Auth + Assoc succeeds with AID 1.
        let aid = associate_sta(&mut ap, &mut sink, sta, 200);
        assert_eq!(aid, 1);
        assert_eq!(ap.associated_client_count(), 1);
        assert_eq!(ap.last_activity_ms(), 200);

        // ReassocReq (subtype 2, 10-byte fixed header) keeps client associated.
        let mut reassoc = [0u8; 128];
        reassoc[0] = 0x20; // Mgmt ReassocReq
        reassoc[4..10].copy_from_slice(&AP_MAC);
        reassoc[10..16].copy_from_slice(&sta);
        reassoc[16..22].copy_from_slice(&AP_MAC);
        reassoc[24..26].copy_from_slice(&0x0401u16.to_le_bytes());
        reassoc[26..28].copy_from_slice(&10u16.to_le_bytes());
        reassoc[28..34].copy_from_slice(&AP_MAC);
        let ssid = b"KlipperCam-Setup-D4EC";
        reassoc[34] = 0;
        reassoc[35] = ssid.len() as u8;
        reassoc[36..36 + ssid.len()].copy_from_slice(ssid);
        let reassoc_len = 36 + ssid.len();
        sink.clear();
        ap.handle_rx_mpdu(&reassoc[..reassoc_len], 250, &mut sink, &mut eth_out);
        assert_eq!(sink.count, 1);
        let resp = sink.last_frame();
        assert_eq!(resp[0], 0x30);
        let resp_hdr = parse_80211_header(resp).expect("reassoc resp hdr");
        assert_eq!(resp_hdr.frame_type, FrameType::Mgmt);
        assert_eq!(resp_hdr.subtype, 0x03);
        let (status, reassoc_aid) = parse_reassoc_resp(resp).expect("reassoc resp");
        assert_eq!(status, 0);
        assert_eq!(reassoc_aid, 1);

        // Valid ToDS=1 Data from associated STA delivers Ethernet-II frame.
        let (mpdu, mpdu_len) = make_sta_data_mpdu(sta, AP_MAC, b"hello-ap");
        let eth_len = ap
            .handle_rx_mpdu(&mpdu[..mpdu_len], 300, &mut sink, &mut eth_out)
            .expect("data eth");
        assert_eq!(&eth_out[0..6], &AP_MAC);
        assert_eq!(&eth_out[6..12], &sta);
        assert_eq!(&eth_out[14..eth_len], b"hello-ap");
        assert_eq!(ap.last_activity_ms(), 300);

        // Deauth removes client and blocks subsequent Data.
        let mut deauth = [0u8; 32];
        let d_len = build_deauth(&mut deauth, AP_MAC, sta, 3, 9).expect("deauth");
        ap.handle_rx_mpdu(&deauth[..d_len], 310, &mut sink, &mut eth_out);
        assert_eq!(ap.associated_client_count(), 0);
        assert_eq!(
            ap.handle_rx_mpdu(&mpdu[..mpdu_len], 320, &mut sink, &mut eth_out),
            None
        );
    }

    #[test]
    fn test_review_focus_4_rogue_and_malformed_data_rejected() {
        let cfg = ApConfig::for_setup_mac(AP_MAC, 1);
        let mut ap = ApMacCore::new(cfg).expect("ap");
        let mut sink = RecordingSink::new();
        let mut eth_out = [0u8; 1536];
        let rogue_sta = [0x02, 0xDE, 0xAD, 0xBE, 0xEF, 0x01];
        let auth_sta = [0x02, 0xDE, 0xAD, 0xBE, 0xEF, 0x02];

        // 1. Unassociated STA ToDS=1 Data MUST be dropped.
        let (rogue_mpdu, rogue_len) = make_sta_data_mpdu(rogue_sta, AP_MAC, b"rogue");
        assert_eq!(
            ap.handle_rx_mpdu(&rogue_mpdu[..rogue_len], 100, &mut sink, &mut eth_out),
            None
        );

        // Associate `auth_sta` for malformed-frame tests.
        let _ = associate_sta(&mut ap, &mut sink, auth_sta, 200);

        // 2. Protected Data frame MUST be rejected in Open AP mode.
        let (mut prot_mpdu, prot_len) = make_sta_data_mpdu(auth_sta, AP_MAC, b"secret");
        prot_mpdu[1] |= 0x40;
        assert_eq!(
            ap.handle_rx_mpdu(&prot_mpdu[..prot_len], 210, &mut sink, &mut eth_out),
            None
        );

        // 3. Fragmented Data (More Fragments bit and FragNum != 0) MUST be rejected.
        let (mut frag_mf, frag_len) = make_sta_data_mpdu(auth_sta, AP_MAC, b"frag1");
        frag_mf[1] |= 0x04;
        assert_eq!(
            ap.handle_rx_mpdu(&frag_mf[..frag_len], 220, &mut sink, &mut eth_out),
            None
        );
        let (mut frag_num, frag_num_len) = make_sta_data_mpdu(auth_sta, AP_MAC, b"frag2");
        frag_num[22] |= 0x01;
        assert_eq!(
            ap.handle_rx_mpdu(&frag_num[..frag_num_len], 225, &mut sink, &mut eth_out),
            None
        );

        // 4. QoS Data with A-MSDU Present (bit 7 of QoS Ctrl) MUST be rejected.
        let mut qos_amsdu = [0u8; 64];
        qos_amsdu[0] = 0x88; // QoS Data
        qos_amsdu[1] = 0x01; // ToDS=1, FromDS=0
        qos_amsdu[4..10].copy_from_slice(&AP_MAC);
        qos_amsdu[10..16].copy_from_slice(&auth_sta);
        qos_amsdu[16..22].copy_from_slice(&AP_MAC);
        qos_amsdu[24] = 0x80; // A-MSDU Present
        qos_amsdu[26..34].copy_from_slice(&[0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00, 0x08, 0x00]);
        assert_eq!(
            ap.handle_rx_mpdu(&qos_amsdu[..40], 230, &mut sink, &mut eth_out),
            None
        );
    }

    #[test]
    fn test_review_focus_4_lru_eviction_and_60s_idle_eviction() {
        let cfg = ApConfig::for_setup_mac(AP_MAC, 1);
        let mut ap = ApMacCore::new(cfg).expect("ap");
        let mut sink = RecordingSink::new();
        let mut eth_out = [0u8; 1536];

        let stas = [
            [0x02, 0x01, 0x00, 0x00, 0x00, 0x01],
            [0x02, 0x01, 0x00, 0x00, 0x00, 0x02],
            [0x02, 0x01, 0x00, 0x00, 0x00, 0x03],
            [0x02, 0x01, 0x00, 0x00, 0x00, 0x04],
            [0x02, 0x01, 0x00, 0x00, 0x00, 0x05],
        ];

        // Fill all 4 slots at staggered timestamps; refresh STA 0 so STA 1 is LRU.
        assert_eq!(associate_sta(&mut ap, &mut sink, stas[0], 1_000), 1);
        assert_eq!(associate_sta(&mut ap, &mut sink, stas[1], 1_100), 2);
        assert_eq!(associate_sta(&mut ap, &mut sink, stas[2], 1_200), 3);
        assert_eq!(associate_sta(&mut ap, &mut sink, stas[3], 1_300), 4);
        assert_eq!(ap.associated_client_count(), 4);

        let (sta0_data, sta0_len) = make_sta_data_mpdu(stas[0], AP_MAC, b"ping");
        assert!(ap
            .handle_rx_mpdu(&sta0_data[..sta0_len], 1_500, &mut sink, &mut eth_out)
            .is_some());

        // 5th client MUST evict LRU (STA 1, last_seen = 1_100, slot 1 -> AID 2).
        let aid5 = associate_sta(&mut ap, &mut sink, stas[4], 1_600);
        assert_eq!(aid5, 2);
        assert_eq!(ap.associated_client_count(), 4);

        // Evicted STA 1 Data MUST now be dropped; STA 0 and STA 4 still pass.
        let (sta1_data, sta1_len) = make_sta_data_mpdu(stas[1], AP_MAC, b"stale");
        assert_eq!(
            ap.handle_rx_mpdu(&sta1_data[..sta1_len], 1_700, &mut sink, &mut eth_out),
            None
        );
        let (sta4_data, sta4_len) = make_sta_data_mpdu(stas[4], AP_MAC, b"fresh");
        assert!(ap
            .handle_rx_mpdu(&sta4_data[..sta4_len], 1_700, &mut sink, &mut eth_out)
            .is_some());

        // Advance time by 60 seconds past STA 2 (1_200 -> 61_200) -> STA 2 evicted.
        ap.poll_tick(61_200, &mut sink);
        assert_eq!(ap.associated_client_count(), 3);

        // Advance past all remaining stations (1_700 + 60_000 = 61_700) -> all evicted.
        ap.poll_tick(61_700, &mut sink);
        assert_eq!(ap.associated_client_count(), 0);
    }

    #[test]
    fn test_transmit_eth_broadcast_and_unicast_rate_fallback() {
        let cfg = ApConfig::for_setup_mac(AP_MAC, 1);
        let mut ap = ApMacCore::new(cfg).expect("ap");
        let mut sink = RecordingSink::new();
        let sta = [0x02, 0xCC, 0xDD, 0xEE, 0xFF, 0x01];

        // Unicast to unassociated STA MUST fail.
        let mut eth_uni = [0u8; 32];
        eth_uni[0..6].copy_from_slice(&sta);
        eth_uni[6..12].copy_from_slice(&AP_MAC);
        eth_uni[12..14].copy_from_slice(&[0x08, 0x00]);
        assert_eq!(
            ap.transmit_eth(&eth_uni, &mut sink),
            Err(ApError::ClientNotAssociated)
        );

        // Broadcast succeeds with 0 stations at Rate1mL, wait_ack=false.
        let mut eth_bcast = [0u8; 32];
        eth_bcast[0..6].fill(0xFF);
        eth_bcast[6..12].copy_from_slice(&AP_MAC);
        eth_bcast[12..14].copy_from_slice(&[0x08, 0x06]);
        ap.transmit_eth(&eth_bcast, &mut sink).expect("bcast tx");
        assert_eq!(sink.count, 1);
        assert_eq!(sink.rates[0], WifiPhyRate::Rate1mL);
        assert!(!sink.wait_acks[0]);

        // Associate STA and test Rate6m failure -> single Rate1mL retry fallback.
        let _ = associate_sta(&mut ap, &mut sink, sta, 500);
        sink.clear();
        sink.fail_first_rate6m = true;
        ap.transmit_eth(&eth_uni, &mut sink)
            .expect("unicast fallback tx");
        assert_eq!(sink.count, 2);
        assert_eq!(sink.rates[0], WifiPhyRate::Rate6m);
        assert!(sink.wait_acks[0]);
        assert_eq!(sink.rates[1], WifiPhyRate::Rate1mL);
        assert!(sink.wait_acks[1]);
    }

    struct FailingSink {
        err: ApError,
    }

    impl ApTxSink for FailingSink {
        fn send_mpdu(
            &mut self,
            _mpdu: &[u8],
            _rate: WifiPhyRate,
            _wait_ack: bool,
        ) -> Result<(), ApError> {
            Err(self.err)
        }
    }

    #[test]
    fn test_ap_stats_counters_and_tx_errors() {
        let cfg = ApConfig::for_setup_mac(AP_MAC, 1);
        let mut ap = ApMacCore::new(cfg).expect("ap");
        assert_eq!(ap.stats(), ApStats::default());

        let mut ok_sink = RecordingSink::new();
        let mut err_sink = FailingSink {
            err: ApError::Hardware(LmacError::TxTimeout),
        };
        let mut eth_out = [0u8; 1536];
        let sta = [0x02, 0x11, 0x22, 0x33, 0x44, 0x55];

        ap.poll_tick(0, &mut ok_sink);
        ap.poll_tick(100, &mut err_sink);

        let mut req = [0u8; 128];
        let wild_len = build_probe_req(&mut req, sta, None, 1).expect("wildcard probe");
        ap.handle_rx_mpdu(&req[..wild_len], 110, &mut ok_sink, &mut eth_out);

        let mut other_req = [0u8; 128];
        let mismatch_len =
            build_probe_req(&mut other_req, sta, Some(b"Other"), 2).expect("other probe");
        ap.handle_rx_mpdu(&other_req[..mismatch_len], 120, &mut ok_sink, &mut eth_out);

        let mut err_probe_sink = FailingSink {
            err: ApError::Hardware(LmacError::TxDrainFailed),
        };
        ap.handle_rx_mpdu(&req[..wild_len], 130, &mut err_probe_sink, &mut eth_out);

        assert_eq!(
            ap.stats(),
            ApStats {
                beacons_ok: 1,
                beacons_err: 1,
                probe_req_rx: 2,
                probe_resp_ok: 1,
                probe_resp_err: 1,
                rx_frames: 3,
                last_tx_err: Some(LmacError::TxDrainFailed),
            }
        );
    }
}
