//! 802.11 Station (STA) MAC Core State Machine.
//!
//! Implements IEEE 802.11 active scanning, open authentication, association,
//! WPA2 4-Way / Group Handshake integration, CCMP encryption/decryption,
//! replay protection, and keepalives in safe, `#![no_std]` Rust.

use crate::frames_80211::build_addba_decline;
use crate::frames_80211::build_assoc_req;
use crate::frames_80211::build_deauth;
use crate::frames_80211::build_null_data_keepalive;
use crate::frames_80211::build_open_auth_req;
use crate::frames_80211::build_probe_req;
use crate::frames_80211::eth_to_80211_data;
use crate::frames_80211::parse_80211_header;
use crate::frames_80211::parse_addba_req;
use crate::frames_80211::parse_assoc_resp;
use crate::frames_80211::parse_auth_resp;
use crate::frames_80211::parse_beacon_or_probe_resp;
use crate::frames_80211::parse_deauth_or_disassoc;
use crate::frames_80211::rx_80211_data_to_eth;
use crate::frames_80211::FrameError;
use crate::frames_80211::FrameType;
use crate::frames_80211::RxDataOutcome;
use crate::lmac_mmio::LmacError;
use crate::phy_bridge::PhyBridgeError;
use crate::rate_ctrl::RateController;
use crate::regs::WifiPhyRate;
use core::fmt;
use esp32_cam_wpa2::handshake::HandshakeError;
use esp32_cam_wpa2::handshake::RxEapolAction;
use esp32_cam_wpa2::handshake::TxDoneAction;
use esp32_cam_wpa2::handshake::WpaStaStateMachine;
use esp32_cam_wpa2::handshake::WIFI_WPA_ALG_CCMP;
use esp32_cam_wpa2::ie::parse_rsn_ie;
use esp32_cam_wpa2::ie::parse_wpa_ie;
use esp32_cam_wpa2::ie::WPA_CIPHER_CCMP;
use esp32_cam_wpa2::ie::WPA_KEY_MGMT_PSK;
use esp32_cam_wpa2::ie::WPA_PROTO_RSN;
use esp32_cam_wpa2::ie::WPA_PROTO_WPA;
use esp32_cam_wpa2::kdf::pbkdf2_hmac_sha1;

/// STA state machine errors.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StaError {
    /// Configuration parameter was invalid.
    InvalidConfig,
    /// No matching or acceptable AP candidate was found after scanning.
    NoSuitableAp,
    /// 802.11 frame parsing or serialization failure.
    Frame(FrameError),
    /// WPA/WPA2 4-Way or Group key handshake error.
    Handshake(HandshakeError),
    /// CCMP frame replay attack detected.
    ReplayDetected,
    /// Low-level LMAC hardware error.
    Hardware(LmacError),
    /// PHY configuration or RF error.
    Phy(PhyBridgeError),
    /// STA is not connected to an AP.
    NotConnected,
    /// Scratch or output buffer capacity is insufficient.
    BufferTooSmall,
}

impl fmt::Display for StaError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidConfig => write!(f, "invalid STA configuration"),
            Self::NoSuitableAp => write!(f, "no suitable access point found"),
            Self::Frame(e) => write!(f, "frame error: {e}"),
            Self::Handshake(e) => write!(f, "WPA handshake error: {e:?}"),
            Self::ReplayDetected => write!(f, "CCMP replay attack detected"),
            Self::Hardware(e) => write!(f, "LMAC hardware error: {e}"),
            Self::Phy(e) => write!(f, "PHY error: {e}"),
            Self::NotConnected => write!(f, "STA is not connected"),
            Self::BufferTooSmall => write!(f, "buffer too small for operation"),
        }
    }
}

impl From<FrameError> for StaError {
    fn from(err: FrameError) -> Self {
        Self::Frame(err)
    }
}

impl From<HandshakeError> for StaError {
    fn from(err: HandshakeError) -> Self {
        Self::Handshake(err)
    }
}

impl From<LmacError> for StaError {
    fn from(err: LmacError) -> Self {
        Self::Hardware(err)
    }
}

impl From<PhyBridgeError> for StaError {
    fn from(err: PhyBridgeError) -> Self {
        Self::Phy(err)
    }
}

/// Operational state of the STA MAC core.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StaState {
    /// Inactive / unconfigured.
    Idle,
    /// Sweeping channels 1..=13 sending probe requests.
    Scanning,
    /// Awaiting 802.11 Open Authentication response.
    AuthWait,
    /// Awaiting 802.11 Association response.
    AssocWait,
    /// Performing WPA/WPA2 4-way handshake.
    FourWayWait,
    /// Associated, authenticated, and passing data traffic.
    Connected,
    /// Disconnected, undergoing exponential backoff before scan retry.
    Disconnected,
}

/// Configuration for connecting to a target Wi-Fi network.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StaConnectConfig {
    /// Target SSID bytes.
    pub ssid: [u8; 32],
    /// Length of target SSID.
    pub ssid_len: usize,
    /// WPA/WPA2 passphrase or 64-hex-digit PSK.
    pub passphrase: [u8; 64],
    /// Length of passphrase.
    pub passphrase_len: usize,
    /// Optional specific BSSID to lock onto.
    pub target_bssid: Option<[u8; 6]>,
    /// Minimum acceptable RSSI in dBm.
    pub min_rssi: i8,
}

impl StaConnectConfig {
    /// Creates an empty, default connection configuration.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            ssid: [0u8; 32],
            ssid_len: 0,
            passphrase: [0u8; 64],
            passphrase_len: 0,
            target_bssid: None,
            min_rssi: -128,
        }
    }

    /// Validates and constructs configuration from byte slices.
    pub fn from_slices(
        ssid: &[u8],
        passphrase: &[u8],
        target_bssid: Option<[u8; 6]>,
        min_rssi: i8,
    ) -> Result<Self, StaError> {
        if ssid.is_empty() || ssid.len() > 32 || passphrase.len() > 64 {
            return Err(StaError::InvalidConfig);
        }
        let mut cfg = Self::new();
        cfg.ssid[..ssid.len()].copy_from_slice(ssid);
        cfg.ssid_len = ssid.len();
        cfg.passphrase[..passphrase.len()].copy_from_slice(passphrase);
        cfg.passphrase_len = passphrase.len();
        cfg.target_bssid = target_bssid;
        cfg.min_rssi = min_rssi;
        Ok(cfg)
    }

    /// Returns the target SSID slice.
    #[must_use]
    pub fn ssid_bytes(&self) -> &[u8] {
        &self.ssid[..self.ssid_len]
    }

    /// Returns the passphrase slice.
    #[must_use]
    pub fn passphrase_bytes(&self) -> &[u8] {
        &self.passphrase[..self.passphrase_len]
    }
}

impl Default for StaConnectConfig {
    fn default() -> Self {
        Self::new()
    }
}

/// Recorded candidate AP discovered during channel scanning.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ScanCandidate {
    /// BSSID of the candidate AP.
    pub bssid: [u8; 6],
    /// Channel where candidate was observed (1..=13).
    pub channel: u8,
    /// Received Signal Strength Indicator in dBm.
    pub rssi: i8,
    /// Privacy capability flag asserted.
    pub privacy: bool,
    /// CCMP pairwise cipher supported.
    pub has_ccmp: bool,
    /// PSK key management supported.
    pub has_psk: bool,
    /// Group cipher suite selector (from RSN/WPA IE).
    pub group_cipher: u32,
    /// Protocol version (WPA_PROTO_RSN or WPA_PROTO_WPA).
    pub proto: i32,
    /// Raw AP Information Element payload.
    pub ap_ie: [u8; 128],
    /// Valid length of AP IE payload.
    pub ap_ie_len: usize,
    /// Evaluation score for candidate selection ranking.
    pub score: i32,
}

const EMPTY_CANDIDATE: ScanCandidate = ScanCandidate {
    bssid: [0u8; 6],
    channel: 0,
    rssi: -128,
    privacy: false,
    has_ccmp: false,
    has_psk: false,
    group_cipher: 0,
    proto: 0,
    ap_ie: [0u8; 128],
    ap_ie_len: 0,
    score: i32::MIN,
};

/// Hardware transmit sink and environment interface.
pub trait StaTxSink {
    /// Transmits an 802.11 MPDU at the given rate.
    fn send_mpdu(
        &mut self,
        mpdu: &[u8],
        rate: WifiPhyRate,
        wait_ack: bool,
        now_ms: u64,
    ) -> Result<(), StaError>;

    /// Switches the active RF channel.
    fn on_channel_switch(&mut self, channel: u8) -> Result<(), StaError>;

    /// Configures RX packet filters for scanning.
    fn on_scan_rx_policy(&mut self) -> Result<(), StaError>;

    /// Notifies hardware layer of associated BSSID for hardware ACK filtering.
    fn on_associated_bssid(&mut self, bssid: [u8; 6]) -> Result<(), StaError>;

    /// Generates a hardware-backed random 32-bit unsigned integer.
    fn hw_random_u32(&mut self) -> u32;
}

const MAX_SCAN_CANDIDATES: usize = 16;
const SCAN_CHANNEL_DWELL_MS: u64 = 90;
const AUTH_TIMEOUT_MS: u64 = 2000;
const ASSOC_TIMEOUT_MS: u64 = 2000;
const FOUR_WAY_TIMEOUT_MS: u64 = 5000;
const KEEPALIVE_INTERVAL_MS: u64 = 30000;
const INITIAL_BACKOFF_MS: u64 = 500;
const MAX_BACKOFF_MS: u64 = 4000;

/// Pure-Rust IEEE 802.11 STA MAC Core.
pub struct StaMacCore {
    sta_mac: [u8; 6],
    state: StaState,
    config: StaConnectConfig,
    rate_ctrl: RateController,
    wpa: WpaStaStateMachine,
    current_channel: u8,
    seq_num: u16,
    candidates: [ScanCandidate; MAX_SCAN_CANDIDATES],
    candidate_count: usize,
    current_scan_channel: u8,
    channel_dwell_start_ms: u64,
    state_entered_ms: u64,
    last_tx_or_rx_ms: u64,
    backoff_ms: u64,
    reconnect_due_ms: u64,
    selected_candidate: Option<ScanCandidate>,
    aid: u16,
    last_rssi: i8,
    cached_pmk: [u8; 32],
    cached_pmk_hex: [u8; 64],
    pmk_cached: bool,
    pmk_cache_ssid: [u8; 32],
    pmk_cache_ssid_len: usize,
    pmk_cache_passphrase: [u8; 64],
    pmk_cache_passphrase_len: usize,
    assoc_ie: [u8; 128],
    assoc_ie_len: usize,
    ptk: Option<[u8; 16]>,
    ptk_tx_pn: u64,
    unicast_rx_pn: u64,
    gtk: [Option<[u8; 16]>; 4],
    gtk_rx_pn: [u64; 4],
    active_gtk_id: Option<u8>,
    tx_buf: [u8; 1600],
    scratch_buf: [u8; 1600],
    eth_buf: [u8; 1600],
}

impl StaMacCore {
    /// Creates a new STA MAC core with the specified hardware MAC address.
    #[must_use]
    pub const fn new(sta_mac: [u8; 6]) -> Self {
        Self {
            sta_mac,
            state: StaState::Idle,
            config: StaConnectConfig::new(),
            rate_ctrl: RateController::new(),
            wpa: WpaStaStateMachine::new(),
            current_channel: 1,
            seq_num: 0,
            candidates: [EMPTY_CANDIDATE; MAX_SCAN_CANDIDATES],
            candidate_count: 0,
            current_scan_channel: 1,
            channel_dwell_start_ms: 0,
            state_entered_ms: 0,
            last_tx_or_rx_ms: 0,
            backoff_ms: INITIAL_BACKOFF_MS,
            reconnect_due_ms: 0,
            selected_candidate: None,
            aid: 0,
            last_rssi: -127,
            cached_pmk: [0u8; 32],
            cached_pmk_hex: [0u8; 64],
            pmk_cached: false,
            pmk_cache_ssid: [0u8; 32],
            pmk_cache_ssid_len: 0,
            pmk_cache_passphrase: [0u8; 64],
            pmk_cache_passphrase_len: 0,
            assoc_ie: [0u8; 128],
            assoc_ie_len: 0,
            ptk: None,
            ptk_tx_pn: 0,
            unicast_rx_pn: 0,
            gtk: [None; 4],
            gtk_rx_pn: [0u64; 4],
            active_gtk_id: None,
            tx_buf: [0u8; 1600],
            scratch_buf: [0u8; 1600],
            eth_buf: [0u8; 1600],
        }
    }

    /// Resets all internal state, buffers, and cryptographic contexts in place
    /// with the specified MAC address, avoiding large stack allocations.
    pub fn reset_with_mac(&mut self, sta_mac: [u8; 6]) {
        self.sta_mac = sta_mac;
        self.state = StaState::Idle;
        self.config = StaConnectConfig::new();
        self.rate_ctrl = RateController::new();
        self.wpa = WpaStaStateMachine::new();
        self.current_channel = 1;
        self.seq_num = 0;
        self.candidates.fill(EMPTY_CANDIDATE);
        self.candidate_count = 0;
        self.current_scan_channel = 1;
        self.channel_dwell_start_ms = 0;
        self.state_entered_ms = 0;
        self.last_tx_or_rx_ms = 0;
        self.backoff_ms = INITIAL_BACKOFF_MS;
        self.reconnect_due_ms = 0;
        self.selected_candidate = None;
        self.aid = 0;
        self.last_rssi = -127;
        self.cached_pmk.fill(0);
        self.cached_pmk_hex.fill(0);
        self.pmk_cached = false;
        self.pmk_cache_ssid.fill(0);
        self.pmk_cache_ssid_len = 0;
        self.pmk_cache_passphrase.fill(0);
        self.pmk_cache_passphrase_len = 0;
        self.assoc_ie.fill(0);
        self.assoc_ie_len = 0;
        self.ptk = None;
        self.ptk_tx_pn = 0;
        self.unicast_rx_pn = 0;
        self.gtk.fill(None);
        self.gtk_rx_pn.fill(0);
        self.active_gtk_id = None;
        self.tx_buf.fill(0);
        self.scratch_buf.fill(0);
        self.eth_buf.fill(0);
    }

    /// Returns the current state machine status.
    #[must_use]
    pub const fn state(&self) -> StaState {
        self.state
    }

    /// Returns true if currently associated, authenticated, and passing data.
    #[must_use]
    pub const fn is_connected(&self) -> bool {
        matches!(self.state, StaState::Connected)
    }

    /// Returns the active BSSID if connected or in an association sequence.
    #[must_use]
    pub fn bssid(&self) -> Option<[u8; 6]> {
        self.selected_candidate.map(|c| c.bssid)
    }

    /// Returns the active operating RF channel.
    #[must_use]
    pub const fn channel(&self) -> u8 {
        self.current_channel
    }

    /// Returns the STA hardware MAC address.
    #[must_use]
    pub const fn sta_mac(&self) -> [u8; 6] {
        self.sta_mac
    }

    /// Returns a reference to the active rate controller.
    #[must_use]
    pub const fn rate_controller(&self) -> &RateController {
        &self.rate_ctrl
    }

    /// Returns the last measured RSSI from the associated AP, or -127 dBm.
    #[must_use]
    pub const fn rssi(&self) -> i8 {
        self.last_rssi
    }

    /// Returns the assigned Association Identifier (AID), or 0 if unassigned.
    #[must_use]
    pub const fn aid(&self) -> u16 {
        self.aid
    }

    /// Returns the candidate access points discovered in the latest scan.
    #[must_use]
    pub fn scan_candidates(&self) -> &[ScanCandidate] {
        &self.candidates[..self.candidate_count]
    }

    /// Initiates connection sequence to a Wi-Fi network.
    pub fn start_connect(
        &mut self,
        config: StaConnectConfig,
        now_ms: u64,
        sink: &mut dyn StaTxSink,
    ) -> Result<(), StaError> {
        if config.ssid_len == 0 || config.ssid_len > 32 || config.passphrase_len > 64 {
            return Err(StaError::InvalidConfig);
        }
        self.config = config;
        self.backoff_ms = INITIAL_BACKOFF_MS;
        self.start_scan(now_ms, sink)
    }

    /// Disconnects from the current AP, sending a deauthentication frame.
    pub fn disconnect(
        &mut self,
        reason: u16,
        now_ms: u64,
        sink: &mut dyn StaTxSink,
    ) -> Result<(), StaError> {
        if let Some(bssid) = self.bssid() {
            let seq = self.next_seq();
            if let Ok(len) = build_deauth(&mut self.tx_buf, bssid, self.sta_mac, reason, seq) {
                let _ = sink.send_mpdu(
                    &self.tx_buf[..len],
                    self.rate_ctrl.mgmt_rate(),
                    false,
                    now_ms,
                );
            }
        }
        self.reset_crypto_and_keys();
        self.wpa.on_disconnected();
        self.state = StaState::Disconnected;
        self.reconnect_due_ms = now_ms.saturating_add(self.backoff_ms);
        Ok(())
    }

    /// Periodic polling tick for channel dwell, timers, keepalives, backoffs.
    pub fn poll_tick(&mut self, now_ms: u64, sink: &mut dyn StaTxSink) -> Result<(), StaError> {
        match self.state {
            StaState::Scanning => {
                if now_ms.saturating_sub(self.channel_dwell_start_ms) >= SCAN_CHANNEL_DWELL_MS {
                    if self.current_scan_channel < 13 {
                        self.current_scan_channel += 1;
                        self.channel_dwell_start_ms = now_ms;
                        sink.on_channel_switch(self.current_scan_channel)?;
                        let seq = self.next_seq();
                        let len = build_probe_req(
                            &mut self.tx_buf,
                            self.sta_mac,
                            Some(self.config.ssid_bytes()),
                            seq,
                        )?;
                        sink.send_mpdu(
                            &self.tx_buf[..len],
                            self.rate_ctrl.mgmt_rate(),
                            false,
                            now_ms,
                        )?;
                    } else {
                        self.finish_scan_sweep(now_ms, sink)?;
                    }
                }
            }
            StaState::AuthWait => {
                if now_ms.saturating_sub(self.state_entered_ms) >= AUTH_TIMEOUT_MS {
                    self.handle_disconnection(now_ms, sink)?;
                }
            }
            StaState::AssocWait => {
                if now_ms.saturating_sub(self.state_entered_ms) >= ASSOC_TIMEOUT_MS {
                    self.handle_disconnection(now_ms, sink)?;
                }
            }
            StaState::FourWayWait => {
                if now_ms.saturating_sub(self.state_entered_ms) >= FOUR_WAY_TIMEOUT_MS {
                    self.handle_disconnection(now_ms, sink)?;
                }
            }
            StaState::Connected => {
                if now_ms.saturating_sub(self.last_tx_or_rx_ms) >= KEEPALIVE_INTERVAL_MS {
                    if let Some(bssid) = self.bssid() {
                        let seq = self.next_seq();
                        let len = build_null_data_keepalive(
                            &mut self.tx_buf,
                            bssid,
                            self.sta_mac,
                            seq,
                            false,
                        )?;
                        sink.send_mpdu(
                            &self.tx_buf[..len],
                            self.rate_ctrl.mgmt_rate(),
                            true,
                            now_ms,
                        )?;
                        self.last_tx_or_rx_ms = now_ms;
                    }
                }
            }
            StaState::Disconnected => {
                if self.reconnect_due_ms != 0 && now_ms >= self.reconnect_due_ms {
                    self.start_scan(now_ms, sink)?;
                }
            }
            StaState::Idle => {}
        }
        Ok(())
    }

    /// Processes an incoming raw MPDU from the hardware RX ring.
    pub fn handle_rx_mpdu<F: FnMut(&[u8])>(
        &mut self,
        raw_mpdu: &[u8],
        rssi: i8,
        now_ms: u64,
        sink: &mut dyn StaTxSink,
        mut on_eth_rx: F,
    ) -> Result<(), StaError> {
        let parsed = match parse_80211_header(raw_mpdu) {
            Ok(p) => p,
            Err(_) => return Ok(()),
        };

        let is_broadcast = (parsed.addr1[0] & 0x01) != 0;
        let is_for_us = parsed.addr1 == self.sta_mac || is_broadcast;
        if !is_for_us {
            return Ok(());
        }

        if let Some(cand) = self.selected_candidate {
            if parsed.addr2 == cand.bssid {
                self.last_rssi = rssi;
            }
        }

        match parsed.frame_type {
            FrameType::Mgmt => self.handle_mgmt_frame(raw_mpdu, &parsed, rssi, now_ms, sink),
            FrameType::Data => {
                self.handle_data_frame(raw_mpdu, &parsed, now_ms, sink, &mut on_eth_rx)
            }
            _ => Ok(()),
        }
    }

    /// Encapsulates and transmits an Ethernet-II frame over the Wi-Fi link.
    pub fn transmit_eth(
        &mut self,
        eth_frame: &[u8],
        now_ms: u64,
        sink: &mut dyn StaTxSink,
    ) -> Result<(), StaError> {
        if self.state != StaState::Connected {
            return Err(StaError::NotConnected);
        }
        let bssid = self.bssid().ok_or(StaError::NotConnected)?;
        let seq = self.next_seq();
        let tk = self.ptk.as_ref().ok_or(StaError::NotConnected)?;
        self.ptk_tx_pn = self.ptk_tx_pn.saturating_add(1);
        let tx_key = Some((tk, 0u8, self.ptk_tx_pn));

        let len = eth_to_80211_data(&mut self.tx_buf, eth_frame, bssid, seq, tx_key)?;
        let rate = self.rate_ctrl.current_data_rate();
        let res = sink.send_mpdu(&self.tx_buf[..len], rate, true, now_ms);
        match res {
            Ok(()) => {
                self.rate_ctrl.on_tx_success();
                self.last_tx_or_rx_ms = now_ms;
                Ok(())
            }
            Err(e) => {
                self.rate_ctrl.on_tx_failure();
                Err(e)
            }
        }
    }

    fn start_scan(&mut self, now_ms: u64, sink: &mut dyn StaTxSink) -> Result<(), StaError> {
        self.candidate_count = 0;
        self.selected_candidate = None;
        self.reset_crypto_and_keys();
        self.wpa.reset();
        self.current_scan_channel = 1;
        self.channel_dwell_start_ms = now_ms;
        self.state = StaState::Scanning;
        self.state_entered_ms = now_ms;

        sink.on_scan_rx_policy()?;
        sink.on_channel_switch(1)?;

        let seq = self.next_seq();
        let len = build_probe_req(
            &mut self.tx_buf,
            self.sta_mac,
            Some(self.config.ssid_bytes()),
            seq,
        )?;
        sink.send_mpdu(
            &self.tx_buf[..len],
            self.rate_ctrl.mgmt_rate(),
            false,
            now_ms,
        )?;
        self.last_tx_or_rx_ms = now_ms;
        Ok(())
    }

    fn finish_scan_sweep(&mut self, now_ms: u64, sink: &mut dyn StaTxSink) -> Result<(), StaError> {
        if self.candidate_count == 0 {
            self.state = StaState::Disconnected;
            self.reconnect_due_ms = now_ms.saturating_add(self.backoff_ms);
            self.backoff_ms = (self.backoff_ms.saturating_mul(2)).min(MAX_BACKOFF_MS);
            return Err(StaError::NoSuitableAp);
        }

        let mut best_idx = 0;
        let mut best_score = self.candidates[0].score;
        let mut i = 1;
        while i < self.candidate_count {
            if self.candidates[i].score > best_score {
                best_score = self.candidates[i].score;
                best_idx = i;
            }
            i += 1;
        }

        let candidate = self.candidates[best_idx];
        self.selected_candidate = Some(candidate);
        self.current_channel = candidate.channel;
        self.last_rssi = candidate.rssi;
        sink.on_channel_switch(candidate.channel)?;
        sink.on_associated_bssid(candidate.bssid)?;

        self.ensure_cached_pmk()?;
        let (assoc_ie, assoc_ie_len) = self.wpa.start_connect(
            self.sta_mac,
            candidate.bssid,
            self.config.ssid_bytes(),
            &self.cached_pmk_hex,
            &candidate.ap_ie[..candidate.ap_ie_len],
            candidate.proto,
            WPA_CIPHER_CCMP,
            candidate.group_cipher,
            WPA_KEY_MGMT_PSK,
            0,
            0,
        )?;
        self.assoc_ie = assoc_ie;
        self.assoc_ie_len = assoc_ie_len;

        let seq = self.next_seq();
        let len = build_open_auth_req(&mut self.tx_buf, candidate.bssid, self.sta_mac, seq)?;
        sink.send_mpdu(
            &self.tx_buf[..len],
            self.rate_ctrl.mgmt_rate(),
            true,
            now_ms,
        )?;

        self.state = StaState::AuthWait;
        self.state_entered_ms = now_ms;
        self.last_tx_or_rx_ms = now_ms;
        Ok(())
    }

    fn handle_mgmt_frame(
        &mut self,
        raw_mpdu: &[u8],
        parsed: &crate::frames_80211::ParsedFrame<'_>,
        rssi: i8,
        now_ms: u64,
        sink: &mut dyn StaTxSink,
    ) -> Result<(), StaError> {
        match parsed.subtype {
            8 | 5 => {
                if self.state == StaState::Scanning {
                    self.record_scan_beacon(raw_mpdu, rssi);
                }
            }
            11 => {
                if self.state == StaState::AuthWait {
                    if let Some(cand) = self.selected_candidate {
                        if parsed.addr2 == cand.bssid {
                            let (seq, status) = parse_auth_resp(raw_mpdu)?;
                            if seq == 2 && status == 0 {
                                let seq = self.next_seq();
                                let len = build_assoc_req(
                                    &mut self.tx_buf,
                                    cand.bssid,
                                    self.sta_mac,
                                    self.config.ssid_bytes(),
                                    &self.assoc_ie[..self.assoc_ie_len],
                                    seq,
                                    10,
                                )?;
                                sink.send_mpdu(
                                    &self.tx_buf[..len],
                                    self.rate_ctrl.mgmt_rate(),
                                    true,
                                    now_ms,
                                )?;
                                self.state = StaState::AssocWait;
                                self.state_entered_ms = now_ms;
                                self.last_tx_or_rx_ms = now_ms;
                            } else {
                                self.handle_disconnection(now_ms, sink)?;
                            }
                        }
                    }
                }
            }
            1 => {
                if self.state == StaState::AssocWait {
                    if let Some(cand) = self.selected_candidate {
                        if parsed.addr2 == cand.bssid {
                            let (status, aid) = parse_assoc_resp(raw_mpdu)?;
                            if status == 0 {
                                self.aid = aid;
                                sink.on_associated_bssid(cand.bssid)?;
                                self.state = StaState::FourWayWait;
                                self.state_entered_ms = now_ms;
                                self.last_tx_or_rx_ms = now_ms;
                            } else {
                                self.handle_disconnection(now_ms, sink)?;
                            }
                        }
                    }
                }
            }
            10 | 12 => {
                if let Some(cand) = self.selected_candidate {
                    if parsed.addr2 == cand.bssid {
                        let _ = parse_deauth_or_disassoc(raw_mpdu);
                        self.handle_disconnection(now_ms, sink)?;
                    }
                }
            }
            13 => {
                if let Ok((dialog_token, tid_param_set, _, _)) = parse_addba_req(raw_mpdu) {
                    if let Some(cand) = self.selected_candidate {
                        if parsed.addr2 == cand.bssid {
                            let seq = self.next_seq();
                            let len = build_addba_decline(
                                &mut self.tx_buf,
                                cand.bssid,
                                self.sta_mac,
                                dialog_token,
                                tid_param_set,
                                seq,
                            )?;
                            let _ = sink.send_mpdu(
                                &self.tx_buf[..len],
                                self.rate_ctrl.mgmt_rate(),
                                true,
                                now_ms,
                            );
                        }
                    }
                }
            }
            _ => {}
        }
        Ok(())
    }

    fn record_scan_beacon(&mut self, raw_mpdu: &[u8], rssi: i8) {
        let info = match parse_beacon_or_probe_resp(raw_mpdu, rssi) {
            Ok(i) => i,
            Err(_) => return,
        };

        if info.ssid != self.config.ssid_bytes() {
            return;
        }
        if let Some(target) = self.config.target_bssid {
            if info.bssid != target {
                return;
            }
        }
        if info.rssi < self.config.min_rssi {
            return;
        }
        if !info.privacy || !info.has_ccmp || !info.has_psk {
            return;
        }

        let mut proto = WPA_PROTO_RSN;
        let mut group_cipher = WPA_CIPHER_CCMP;
        let mut ap_ie_buf = [0u8; 128];
        let mut ap_ie_len = 0;

        if let Some(rie) = info.rsn_ie {
            if let Ok(data) = parse_rsn_ie(rie) {
                proto = WPA_PROTO_RSN;
                group_cipher = data.group_cipher;
                let copy_len = rie.len().min(ap_ie_buf.len());
                ap_ie_buf[..copy_len].copy_from_slice(&rie[..copy_len]);
                ap_ie_len = copy_len;
            }
        } else if let Some(wie) = info.wpa_ie {
            if let Ok(data) = parse_wpa_ie(wie) {
                proto = WPA_PROTO_WPA;
                group_cipher = data.group_cipher;
                let copy_len = wie.len().min(ap_ie_buf.len());
                ap_ie_buf[..copy_len].copy_from_slice(&wie[..copy_len]);
                ap_ie_len = copy_len;
            }
        } else {
            return;
        }

        let mut score: i32 = i32::from(info.rssi);
        if self.config.target_bssid == Some(info.bssid) {
            score = score.saturating_add(1000);
        }
        if proto == WPA_PROTO_RSN {
            score = score.saturating_add(500);
        }

        let candidate = ScanCandidate {
            bssid: info.bssid,
            channel: info.channel,
            rssi: info.rssi,
            privacy: info.privacy,
            has_ccmp: info.has_ccmp,
            has_psk: info.has_psk,
            group_cipher,
            proto,
            ap_ie: ap_ie_buf,
            ap_ie_len,
            score,
        };

        let mut existing_idx = None;
        let mut i = 0;
        while i < self.candidate_count {
            if self.candidates[i].bssid == candidate.bssid {
                existing_idx = Some(i);
                break;
            }
            i += 1;
        }

        if let Some(idx) = existing_idx {
            if candidate.score > self.candidates[idx].score {
                self.candidates[idx] = candidate;
            }
        } else if self.candidate_count < MAX_SCAN_CANDIDATES {
            self.candidates[self.candidate_count] = candidate;
            self.candidate_count += 1;
        } else {
            let mut min_idx = 0;
            let mut min_score = self.candidates[0].score;
            let mut j = 1;
            while j < self.candidate_count {
                if self.candidates[j].score < min_score {
                    min_score = self.candidates[j].score;
                    min_idx = j;
                }
                j += 1;
            }
            if candidate.score > min_score {
                self.candidates[min_idx] = candidate;
            }
        }
    }

    fn handle_data_frame<F: FnMut(&[u8])>(
        &mut self,
        raw_mpdu: &[u8],
        parsed: &crate::frames_80211::ParsedFrame<'_>,
        now_ms: u64,
        sink: &mut dyn StaTxSink,
        on_eth_rx: &mut F,
    ) -> Result<(), StaError> {
        if self.state != StaState::FourWayWait && self.state != StaState::Connected {
            return Ok(());
        }
        let cand = match self.selected_candidate {
            Some(c) => c,
            None => return Ok(()),
        };
        if parsed.addr2 != cand.bssid {
            return Ok(());
        }

        let mut ccmp_pn = 0u64;
        let mut ccmp_key_id = 0u8;
        if parsed.is_protected {
            let fc = u16::from_le_bytes([raw_mpdu[0], raw_mpdu[1]]);
            let mut hdr_len = 24;
            if parsed.to_ds && parsed.from_ds {
                hdr_len = 30;
            }
            if (parsed.subtype & 0x08) != 0 {
                hdr_len += 2;
                if (fc & 0x8000) != 0 {
                    hdr_len += 4;
                }
            }
            if raw_mpdu.len() < hdr_len + 8 {
                return Err(StaError::Frame(FrameError::MalformedHeader));
            }
            let iv = &raw_mpdu[hdr_len..hdr_len + 8];
            ccmp_key_id = (iv[3] >> 6) & 0x03;
            ccmp_pn = (iv[0] as u64)
                | ((iv[1] as u64) << 8)
                | ((iv[4] as u64) << 16)
                | ((iv[5] as u64) << 24)
                | ((iv[6] as u64) << 32)
                | ((iv[7] as u64) << 40);
        }

        let gtk_arg = match self.active_gtk_id {
            Some(id) => self.gtk[id as usize].as_ref().map(|k| (id, k)),
            None => None,
        };

        let outcome = rx_80211_data_to_eth(
            raw_mpdu,
            &mut self.scratch_buf,
            &mut self.eth_buf,
            self.ptk.as_ref(),
            gtk_arg,
            &cand.bssid,
        )?;

        if parsed.is_protected {
            let is_group = (raw_mpdu[4] & 0x01) != 0;
            if is_group {
                let idx = (ccmp_key_id & 0x03) as usize;
                if ccmp_pn <= self.gtk_rx_pn[idx] {
                    return Err(StaError::ReplayDetected);
                }
                self.gtk_rx_pn[idx] = ccmp_pn;
            } else {
                if ccmp_pn <= self.unicast_rx_pn {
                    return Err(StaError::ReplayDetected);
                }
                self.unicast_rx_pn = ccmp_pn;
            }
        }

        match outcome {
            RxDataOutcome::Eapol(elen) => {
                let mut snonce = [0u8; 32];
                let mut offset = 0;
                while offset < 32 {
                    let r = sink.hw_random_u32();
                    let bytes = r.to_le_bytes();
                    let rem = 32 - offset;
                    let to_copy = if rem < 4 { rem } else { 4 };
                    snonce[offset..offset + to_copy].copy_from_slice(&bytes[..to_copy]);
                    offset += to_copy;
                }

                let act = self
                    .wpa
                    .on_rx_eapol(&cand.bssid, &self.eth_buf[..elen], &snonce)?;
                match act {
                    RxEapolAction::Ignore => {}
                    RxEapolAction::SendFrame(txf) => {
                        let mut eth = [0u8; 512];
                        let eth_len = 14 + txf.len;
                        if eth_len > eth.len() {
                            return Err(StaError::BufferTooSmall);
                        }
                        eth[0..6].copy_from_slice(&cand.bssid);
                        eth[6..12].copy_from_slice(&self.sta_mac);
                        eth[12..14].copy_from_slice(&[0x88, 0x8E]);
                        eth[14..eth_len].copy_from_slice(&txf.data[..txf.len]);

                        let seq = self.next_seq();
                        let tx_key = if let Some(ref tk) = self.ptk {
                            self.ptk_tx_pn = self.ptk_tx_pn.saturating_add(1);
                            Some((tk, 0u8, self.ptk_tx_pn))
                        } else {
                            None
                        };
                        let mpdu_len = eth_to_80211_data(
                            &mut self.tx_buf,
                            &eth[..eth_len],
                            cand.bssid,
                            seq,
                            tx_key,
                        )?;
                        let send_res = sink.send_mpdu(
                            &self.tx_buf[..mpdu_len],
                            self.rate_ctrl.mgmt_rate(),
                            true,
                            now_ms,
                        );
                        let tx_failed = send_res.is_err();
                        let done_act = self.wpa.on_eapol_tx_complete(tx_failed);
                        self.handle_tx_done_action(done_act);
                        send_res?;
                    }
                    RxEapolAction::Deauth(reason) => {
                        let seq = self.next_seq();
                        let len = build_deauth(
                            &mut self.tx_buf,
                            cand.bssid,
                            self.sta_mac,
                            reason as u16,
                            seq,
                        )?;
                        let _ = sink.send_mpdu(
                            &self.tx_buf[..len],
                            self.rate_ctrl.mgmt_rate(),
                            false,
                            now_ms,
                        );
                        self.handle_disconnection(now_ms, sink)?;
                    }
                }
            }
            RxDataOutcome::Ethernet(total_eth_len) => {
                self.last_tx_or_rx_ms = now_ms;
                on_eth_rx(&self.eth_buf[..total_eth_len]);
            }
        }
        Ok(())
    }

    fn handle_tx_done_action(&mut self, action: TxDoneAction) {
        match action {
            TxDoneAction::None | TxDoneAction::TxFailed => {}
            TxDoneAction::InstallAndComplete(install) => {
                if install.install_ptk && install.ptk_alg == WIFI_WPA_ALG_CCMP {
                    let mut tk = [0u8; 16];
                    tk.copy_from_slice(&install.ptk_key[..16]);
                    self.ptk = Some(tk);
                    self.ptk_tx_pn = 0;
                    self.unicast_rx_pn = 0;
                }
                if install.gtk_len >= 16 && install.gtk_alg == WIFI_WPA_ALG_CCMP {
                    let idx = (install.gtk_idx & 0x03) as usize;
                    let mut gtk_key = [0u8; 16];
                    gtk_key.copy_from_slice(&install.gtk[..16]);
                    self.gtk[idx] = Some(gtk_key);
                    self.gtk_rx_pn[idx] = 0;
                    self.active_gtk_id = Some(install.gtk_idx & 0x03);
                }
                if install.notify_auth_done {
                    self.wpa.on_connected();
                    self.state = StaState::Connected;
                    self.backoff_ms = INITIAL_BACKOFF_MS;
                }
            }
        }
    }

    fn handle_disconnection(
        &mut self,
        now_ms: u64,
        _sink: &mut dyn StaTxSink,
    ) -> Result<(), StaError> {
        self.reset_crypto_and_keys();
        self.wpa.on_disconnected();
        self.state = StaState::Disconnected;
        self.reconnect_due_ms = now_ms.saturating_add(self.backoff_ms);
        self.backoff_ms = (self.backoff_ms.saturating_mul(2)).min(MAX_BACKOFF_MS);
        Ok(())
    }

    fn reset_crypto_and_keys(&mut self) {
        self.ptk = None;
        self.ptk_tx_pn = 0;
        self.unicast_rx_pn = 0;
        self.gtk = [None; 4];
        self.gtk_rx_pn = [0u64; 4];
        self.active_gtk_id = None;
        self.aid = 0;
    }

    fn next_seq(&mut self) -> u16 {
        let s = self.seq_num;
        self.seq_num = (self.seq_num + 1) & 0x0FFF;
        s
    }

    fn ensure_cached_pmk(&mut self) -> Result<(), StaError> {
        let ssid = self.config.ssid_bytes();
        let pass = self.config.passphrase_bytes();
        if self.pmk_cached
            && self.pmk_cache_ssid_len == ssid.len()
            && &self.pmk_cache_ssid[..self.pmk_cache_ssid_len] == ssid
            && self.pmk_cache_passphrase_len == pass.len()
            && &self.pmk_cache_passphrase[..self.pmk_cache_passphrase_len] == pass
        {
            return Ok(());
        }

        let mut pmk = [0u8; 32];
        if let Some(dec) = decode_hex_pmk(pass) {
            pmk = dec;
            encode_hex_pmk(&pmk, &mut self.cached_pmk_hex);
        } else if (8..=63).contains(&pass.len()) {
            pbkdf2_hmac_sha1(pass, ssid, 4096, &mut pmk)
                .map_err(|_| StaError::Handshake(HandshakeError::BadParam))?;
            encode_hex_pmk(&pmk, &mut self.cached_pmk_hex);
        } else {
            return Err(StaError::InvalidConfig);
        }

        self.cached_pmk = pmk;
        self.pmk_cache_ssid[..ssid.len()].copy_from_slice(ssid);
        self.pmk_cache_ssid_len = ssid.len();
        self.pmk_cache_passphrase[..pass.len()].copy_from_slice(pass);
        self.pmk_cache_passphrase_len = pass.len();
        self.pmk_cached = true;
        Ok(())
    }
}

fn decode_hex_pmk(hex: &[u8]) -> Option<[u8; 32]> {
    if hex.len() != 64 {
        return None;
    }
    let mut out = [0u8; 32];
    let mut i = 0;
    while i < 32 {
        let hi = hex_nibble(hex[i * 2])?;
        let lo = hex_nibble(hex[i * 2 + 1])?;
        out[i] = (hi << 4) | lo;
        i += 1;
    }
    Some(out)
}

const fn hex_nibble(b: u8) -> Option<u8> {
    match b {
        b'0'..=b'9' => Some(b - b'0'),
        b'a'..=b'f' => Some(b - b'a' + 10),
        b'A'..=b'F' => Some(b - b'A' + 10),
        _ => None,
    }
}

fn encode_hex_pmk(pmk: &[u8; 32], out: &mut [u8; 64]) {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut i = 0;
    while i < 32 {
        let b = pmk[i];
        out[i * 2] = HEX[(b >> 4) as usize];
        out[i * 2 + 1] = HEX[(b & 0x0F) as usize];
        i += 1;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct MockSink {
        sent_frames: usize,
        last_channel: u8,
        switched_channels: usize,
        last_rate: WifiPhyRate,
        associated_bssid: Option<[u8; 6]>,
        random_counter: u32,
    }

    impl MockSink {
        fn new() -> Self {
            Self {
                sent_frames: 0,
                last_channel: 1,
                switched_channels: 0,
                last_rate: WifiPhyRate::Rate1mL,
                associated_bssid: None,
                random_counter: 0x1234_5678,
            }
        }
    }

    impl StaTxSink for MockSink {
        fn send_mpdu(
            &mut self,
            _mpdu: &[u8],
            rate: WifiPhyRate,
            _wait_ack: bool,
            _now_ms: u64,
        ) -> Result<(), StaError> {
            self.sent_frames += 1;
            self.last_rate = rate;
            Ok(())
        }

        fn on_channel_switch(&mut self, channel: u8) -> Result<(), StaError> {
            self.last_channel = channel;
            self.switched_channels += 1;
            Ok(())
        }

        fn on_scan_rx_policy(&mut self) -> Result<(), StaError> {
            Ok(())
        }

        fn on_associated_bssid(&mut self, bssid: [u8; 6]) -> Result<(), StaError> {
            self.associated_bssid = Some(bssid);
            Ok(())
        }

        fn hw_random_u32(&mut self) -> u32 {
            self.random_counter = self.random_counter.wrapping_add(1);
            self.random_counter
        }
    }

    #[test]
    fn test_sta_connect_config_validation() {
        let ok = StaConnectConfig::from_slices(b"MySsid", b"secret123", None, -80);
        assert!(ok.is_ok());
        if let Ok(cfg) = ok {
            assert_eq!(cfg.ssid_bytes(), b"MySsid");
            assert_eq!(cfg.passphrase_bytes(), b"secret123");
            assert_eq!(cfg.min_rssi, -80);
        }

        let empty_ssid = StaConnectConfig::from_slices(b"", b"secret123", None, -80);
        assert_eq!(empty_ssid, Err(StaError::InvalidConfig));

        let overlong_ssid = [b'A'; 33];
        assert_eq!(
            StaConnectConfig::from_slices(&overlong_ssid, b"pass", None, -80),
            Err(StaError::InvalidConfig)
        );

        let overlong_pass = [b'B'; 65];
        assert_eq!(
            StaConnectConfig::from_slices(b"Ssid", &overlong_pass, None, -80),
            Err(StaError::InvalidConfig)
        );
    }

    #[test]
    fn test_scan_candidate_scoring() {
        let bssid_a = [0x11; 6];
        let bssid_b = [0x22; 6];
        let cand_wpa = ScanCandidate {
            bssid: bssid_a,
            channel: 1,
            rssi: -60,
            privacy: true,
            has_ccmp: true,
            has_psk: true,
            group_cipher: WPA_CIPHER_CCMP,
            proto: WPA_PROTO_WPA,
            ap_ie: [0u8; 128],
            ap_ie_len: 0,
            score: -60,
        };
        let cand_rsn = ScanCandidate {
            bssid: bssid_b,
            channel: 6,
            rssi: -60,
            privacy: true,
            has_ccmp: true,
            has_psk: true,
            group_cipher: WPA_CIPHER_CCMP,
            proto: WPA_PROTO_RSN,
            ap_ie: [0u8; 128],
            ap_ie_len: 0,
            score: -60 + 500,
        };
        assert!(cand_rsn.score > cand_wpa.score);
    }

    #[test]
    fn test_addba_decline_generation() {
        let sta_mac = [0x00, 0x11, 0x22, 0x33, 0x44, 0x55];
        let bssid = [0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB];
        let mut fsm = StaMacCore::new(sta_mac);
        let mut sink = MockSink::new();

        fsm.selected_candidate = Some(ScanCandidate {
            bssid,
            channel: 1,
            rssi: -50,
            privacy: true,
            has_ccmp: true,
            has_psk: true,
            group_cipher: WPA_CIPHER_CCMP,
            proto: WPA_PROTO_RSN,
            ap_ie: [0u8; 128],
            ap_ie_len: 0,
            score: 0,
        });

        let mut addba_req = [0u8; 33];
        addba_req[0] = 0xD0;
        addba_req[1] = 0x00;
        addba_req[4..10].copy_from_slice(&sta_mac);
        addba_req[10..16].copy_from_slice(&bssid);
        addba_req[16..22].copy_from_slice(&bssid);
        addba_req[24] = 3;
        addba_req[25] = 0;
        addba_req[26] = 0x05;
        addba_req[27..29].copy_from_slice(&0x1000u16.to_le_bytes());

        let res = fsm.handle_rx_mpdu(&addba_req, -50, 1000, &mut sink, |_| {});
        assert!(res.is_ok());
        assert_eq!(sink.sent_frames, 1);
    }

    #[test]
    fn test_backoff_and_reconnect() {
        let sta_mac = [0x00, 0x11, 0x22, 0x33, 0x44, 0x55];
        let mut fsm = StaMacCore::new(sta_mac);
        let mut sink = MockSink::new();

        let cfg = StaConnectConfig::from_slices(b"Target", b"Pass", None, -90);
        assert!(cfg.is_ok());
        if let Ok(c) = cfg {
            let res = fsm.start_connect(c, 1000, &mut sink);
            assert!(res.is_ok());
            assert_eq!(fsm.state(), StaState::Scanning);
        }

        // Channel dwell ticks up to channel 13
        let mut time = 1000;
        for _ in 1..13 {
            time += 95;
            let _ = fsm.poll_tick(time, &mut sink);
        }
        assert_eq!(fsm.current_scan_channel, 13);

        // Sweep finish with 0 candidates triggers Disconnected + backoff
        time += 95;
        let sweep_res = fsm.poll_tick(time, &mut sink);
        assert_eq!(sweep_res, Err(StaError::NoSuitableAp));
        assert_eq!(fsm.state(), StaState::Disconnected);
        assert_eq!(fsm.backoff_ms, 1000);
    }

    #[test]
    fn test_reset_with_mac() {
        let sta_mac = [0x00, 0x11, 0x22, 0x33, 0x44, 0x55];
        let mut fsm = StaMacCore::new(sta_mac);
        let new_mac = [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF];
        fsm.reset_with_mac(new_mac);
        assert_eq!(fsm.sta_mac(), new_mac);
        assert_eq!(fsm.state(), StaState::Idle);
    }
}
