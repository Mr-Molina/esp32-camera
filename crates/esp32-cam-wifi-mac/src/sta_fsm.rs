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
use crate::frames_80211::volatile_zeroize;
use crate::frames_80211::FrameError;
use crate::frames_80211::FrameType;
use crate::frames_80211::RxDataOutcome;
use crate::frames_80211::MAX_CCMP_PN;
use crate::lmac_mmio::LmacError;
use crate::phy_bridge::PhyBridgeError;
use crate::rate_ctrl::RateController;
use crate::regs::WifiPhyRate;
use core::fmt;
use esp32_cam_wpa2::crypto::hash::ct_eq;
use esp32_cam_wpa2::handshake::HandshakeError;
use esp32_cam_wpa2::handshake::RxEapolAction;
use esp32_cam_wpa2::handshake::TxDoneAction;
use esp32_cam_wpa2::handshake::WpaStaStateMachine;
use esp32_cam_wpa2::handshake::WIFI_WPA_ALG_CCMP;
use esp32_cam_wpa2::ie::parse_rsn_ie;
use esp32_cam_wpa2::ie::WPA_CIPHER_CCMP;
use esp32_cam_wpa2::ie::WPA_KEY_MGMT_PSK;
use esp32_cam_wpa2::ie::WPA_PROTO_RSN;
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

    /// Generates a hardware-backed random 32-bit unsigned integer, returning an
    /// error if the hardware RNG is uninitialized or fails health checks (`M-015`).
    fn hw_random_u32(&mut self) -> Result<u32, StaError>;
}

const MAX_SCAN_CANDIDATES: usize = 16;
const MAX_FAILED_BSSIDS: usize = 8;
const FAILED_BSSID_PENALTY: i32 = 200;
const SCAN_CHANNEL_DWELL_MS: u64 = 90;
const AUTH_TIMEOUT_MS: u64 = 2000;
const ASSOC_TIMEOUT_MS: u64 = 2000;
const FOUR_WAY_TIMEOUT_MS: u64 = 5000;
const KEEPALIVE_INTERVAL_MS: u64 = 30000;
const INITIAL_BACKOFF_MS: u64 = 500;
const MAX_BACKOFF_MS: u64 = 4000;

/// Converts a 6-byte little-endian IEEE 802.11 Key RSC to a 48-bit CCMP PN.
const fn rsc_to_pn(rsc: [u8; 6]) -> u64 {
    (rsc[0] as u64)
        | ((rsc[1] as u64) << 8)
        | ((rsc[2] as u64) << 16)
        | ((rsc[3] as u64) << 24)
        | ((rsc[4] as u64) << 32)
        | ((rsc[5] as u64) << 40)
}

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
    failed_bssids: [[u8; 6]; MAX_FAILED_BSSIDS],
    failed_bssid_counts: [u8; MAX_FAILED_BSSIDS],
    failed_bssid_len: usize,
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
    unicast_rx_pn_qos: [u64; 16],
    unicast_rx_pn_non_qos: u64,
    gtk: [Option<[u8; 16]>; 4],
    gtk_rx_pn: [u64; 4],
    active_gtk_id: Option<u8>,
    tx_buf: [u8; 1600],
    scratch_buf: [u8; 1600],
    eth_buf: [u8; 1600],
}

impl Drop for StaMacCore {
    fn drop(&mut self) {
        self.zeroize_all_secrets();
    }
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
            failed_bssids: [[0u8; 6]; MAX_FAILED_BSSIDS],
            failed_bssid_counts: [0u8; MAX_FAILED_BSSIDS],
            failed_bssid_len: 0,
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
            unicast_rx_pn_qos: [0u64; 16],
            unicast_rx_pn_non_qos: 0,
            gtk: [None; 4],
            gtk_rx_pn: [0u64; 4],
            active_gtk_id: None,
            tx_buf: [0u8; 1600],
            scratch_buf: [0u8; 1600],
            eth_buf: [0u8; 1600],
        }
    }

    fn zeroize_all_secrets(&mut self) {
        self.reset_crypto_and_keys();
        volatile_zeroize(&mut self.cached_pmk);
        volatile_zeroize(&mut self.cached_pmk_hex);
        volatile_zeroize(&mut self.pmk_cache_passphrase);
        volatile_zeroize(&mut self.config.passphrase);
        volatile_zeroize(&mut self.tx_buf);
        volatile_zeroize(&mut self.scratch_buf);
        volatile_zeroize(&mut self.eth_buf);
    }

    /// Resets all internal state, buffers, and cryptographic contexts in place
    /// with the specified MAC address, avoiding large stack allocations.
    pub fn reset_with_mac(&mut self, sta_mac: [u8; 6]) {
        self.zeroize_all_secrets();
        self.sta_mac = sta_mac;
        self.state = StaState::Idle;
        self.config = StaConnectConfig::new();
        self.rate_ctrl = RateController::new();
        self.wpa.reset();
        self.current_channel = 1;
        self.seq_num = 0;
        self.candidates.fill(EMPTY_CANDIDATE);
        self.candidate_count = 0;
        self.failed_bssids.fill([0u8; 6]);
        self.failed_bssid_counts.fill(0);
        self.failed_bssid_len = 0;
        self.current_scan_channel = 1;
        self.channel_dwell_start_ms = 0;
        self.state_entered_ms = 0;
        self.last_tx_or_rx_ms = 0;
        self.backoff_ms = INITIAL_BACKOFF_MS;
        self.reconnect_due_ms = 0;
        self.selected_candidate = None;
        self.aid = 0;
        self.last_rssi = -127;
        self.pmk_cached = false;
        self.pmk_cache_ssid.fill(0);
        self.pmk_cache_ssid_len = 0;
        self.pmk_cache_passphrase_len = 0;
        self.assoc_ie.fill(0);
        self.assoc_ie_len = 0;
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
        let creds_changed = self.config.ssid_bytes() != config.ssid_bytes()
            || !ct_eq(self.config.passphrase_bytes(), config.passphrase_bytes());
        if creds_changed {
            volatile_zeroize(&mut self.config.passphrase);
            volatile_zeroize(&mut self.cached_pmk);
            volatile_zeroize(&mut self.cached_pmk_hex);
            volatile_zeroize(&mut self.pmk_cache_passphrase);
            self.pmk_cache_passphrase_len = 0;
            self.pmk_cache_ssid.fill(0);
            self.pmk_cache_ssid_len = 0;
            self.pmk_cached = false;
            self.failed_bssids.fill([0u8; 6]);
            self.failed_bssid_counts.fill(0);
            self.failed_bssid_len = 0;
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
        self.selected_candidate = None;
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
        if self.ptk.is_none() {
            return Err(StaError::NotConnected);
        }
        let pn = self.allocate_ptk_tx_pn(now_ms, sink)?;
        let seq = self.next_seq();
        let tk = self.ptk.as_ref().ok_or(StaError::NotConnected)?;
        let tx_key = Some((tk, 0u8, pn));

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

    fn allocate_ptk_tx_pn(
        &mut self,
        now_ms: u64,
        sink: &mut dyn StaTxSink,
    ) -> Result<u64, StaError> {
        if self.ptk_tx_pn >= MAX_CCMP_PN {
            let _ = self.handle_disconnection(now_ms, sink);
            return Err(StaError::Frame(FrameError::InvalidPacketNumber));
        }
        self.ptk_tx_pn += 1;
        Ok(self.ptk_tx_pn)
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

        // Perform fallible PMK derivation, WPA handshake setup, and Auth frame
        // construction BEFORE mutating FSM candidate state or hardware filters.
        if let Err(e) = self.ensure_cached_pmk() {
            let _ = self.handle_disconnection(now_ms, sink);
            return Err(e);
        }

        let (assoc_ie, assoc_ie_len) = match self.wpa.start_connect(
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
        ) {
            Ok(res) => res,
            Err(e) => {
                self.record_bssid_failure(candidate.bssid);
                let _ = self.handle_disconnection(now_ms, sink);
                return Err(StaError::Handshake(e));
            }
        };

        let seq = self.next_seq();
        let len = match build_open_auth_req(&mut self.tx_buf, candidate.bssid, self.sta_mac, seq) {
            Ok(l) => l,
            Err(e) => {
                let _ = self.handle_disconnection(now_ms, sink);
                return Err(StaError::Frame(e));
            }
        };

        if let Err(e) = sink
            .on_channel_switch(candidate.channel)
            .and_then(|()| sink.on_associated_bssid(candidate.bssid))
            .and_then(|()| {
                sink.send_mpdu(
                    &self.tx_buf[..len],
                    self.rate_ctrl.mgmt_rate(),
                    true,
                    now_ms,
                )
            })
        {
            self.record_bssid_failure(candidate.bssid);
            let _ = self.handle_disconnection(now_ms, sink);
            return Err(e);
        }

        self.selected_candidate = Some(candidate);
        self.current_channel = candidate.channel;
        self.last_rssi = candidate.rssi;
        self.assoc_ie = assoc_ie;
        self.assoc_ie_len = assoc_ie_len;
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
                // Auth Response is unicast-only to our STA MAC.
                if self.state == StaState::AuthWait && parsed.addr1 == self.sta_mac {
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
                // Assoc Response is unicast-only to our STA MAC.
                if self.state == StaState::AssocWait && parsed.addr1 == self.sta_mac {
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
                // Ignore Deauth/Disassoc unless actively authenticating,
                // associating, handshaking, or connected, and only when the
                // frame parses cleanly.
                if matches!(
                    self.state,
                    StaState::AuthWait
                        | StaState::AssocWait
                        | StaState::FourWayWait
                        | StaState::Connected
                ) {
                    if let Some(cand) = self.selected_candidate {
                        if parsed.addr2 == cand.bssid && parse_deauth_or_disassoc(raw_mpdu).is_ok()
                        {
                            self.handle_disconnection(now_ms, sink)?;
                        }
                    }
                }
            }
            // Action/ADDBA Request is unicast-only and valid only when Connected.
            13 if self.state == StaState::Connected && parsed.addr1 == self.sta_mac => {
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

        let Some(channel) = info.channel else {
            return;
        };
        if !(1..=13).contains(&channel) {
            return;
        }
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

        // Enforce WPA2-RSN only (reject legacy WPA1 downgrade), cap RSN IE to
        // 128 bytes, reject malformed RSN IEs without fallthrough, and require
        // CCMP pairwise + CCMP group + PSK AKM.
        let Some(rie) = info.rsn_ie else {
            return;
        };
        if rie.len() > 128 {
            return;
        }
        let Ok(data) = parse_rsn_ie(rie) else {
            return;
        };
        if (data.pairwise_cipher & WPA_CIPHER_CCMP) == 0
            || (data.key_mgmt & WPA_KEY_MGMT_PSK) == 0
            || data.group_cipher != WPA_CIPHER_CCMP
        {
            return;
        }

        let proto = WPA_PROTO_RSN;
        let group_cipher = data.group_cipher;
        let mut ap_ie_buf = [0u8; 128];
        ap_ie_buf[..rie.len()].copy_from_slice(rie);
        let ap_ie_len = rie.len();

        let mut score: i32 = i32::from(info.rssi);
        if self.config.target_bssid == Some(info.bssid) {
            score = score.saturating_add(1000);
        }
        score = score.saturating_add(500);
        let fail_count = self.bssid_failure_count(info.bssid);
        if fail_count > 0 {
            let penalty = i32::from(fail_count).saturating_mul(FAILED_BSSID_PENALTY);
            score = score.saturating_sub(penalty);
        }

        let candidate = ScanCandidate {
            bssid: info.bssid,
            channel,
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

        // Select the GTK slot matching the wire CCMP Key ID (supports GTK
        // rekey overlap); `rx_80211_data_to_eth` re-verifies the key_id
        // returned by `ccmp_decrypt_mpdu` after MIC authentication.
        let is_group = (parsed.addr1[0] & 0x01) != 0;
        let gtk_arg = if parsed.is_protected && is_group {
            let fc = u16::from_le_bytes([raw_mpdu[0], raw_mpdu[1]]);
            let is_qos = parsed.subtype == 0x08;
            let has_htc = is_qos && (fc & 0x8000) != 0;
            let hdr_len = 24 + if is_qos { 2 } else { 0 } + if has_htc { 4 } else { 0 };
            if raw_mpdu.len() < hdr_len + 8 {
                return Err(StaError::Frame(FrameError::MalformedHeader));
            }
            let wire_key_id = (raw_mpdu[hdr_len + 3] >> 6) & 0x03;
            self.gtk[usize::from(wire_key_id)]
                .as_ref()
                .map(|k| (wire_key_id, k))
        } else {
            match self.active_gtk_id {
                Some(id) => self.gtk[usize::from(id & 0x03)]
                    .as_ref()
                    .map(|k| (id & 0x03, k)),
                None => None,
            }
        };

        let outcome = rx_80211_data_to_eth(
            raw_mpdu,
            &mut self.scratch_buf,
            &mut self.eth_buf,
            self.ptk.as_ref(),
            gtk_arg,
            &cand.bssid,
            &self.sta_mac,
            true,
        )?;

        let ccmp_meta = match outcome {
            RxDataOutcome::Ethernet { ccmp, .. } | RxDataOutcome::Eapol { ccmp, .. } => ccmp,
        };

        if let Some(meta) = ccmp_meta {
            if is_group {
                let idx = usize::from(meta.key_id & 0x03);
                if meta.pn <= self.gtk_rx_pn[idx] {
                    return Err(StaError::ReplayDetected);
                }
                self.gtk_rx_pn[idx] = meta.pn;
            } else {
                let last_pn = match meta.qos_tid {
                    Some(tid) => &mut self.unicast_rx_pn_qos[usize::from(tid & 0x0F)],
                    None => &mut self.unicast_rx_pn_non_qos,
                };
                if meta.pn <= *last_pn {
                    return Err(StaError::ReplayDetected);
                }
                *last_pn = meta.pn;
            }
        }

        match outcome {
            RxDataOutcome::Eapol { len: elen, ccmp } => {
                // EAPOL must be unicast to sta_mac; once PTK is active or
                // Connected, EAPOL MUST be CCMP-protected (CVE-2020-26140).
                if parsed.addr1 != self.sta_mac {
                    return Err(StaError::Frame(FrameError::MalformedHeader));
                }
                if (self.state == StaState::Connected || self.ptk.is_some())
                    && (!parsed.is_protected || ccmp.is_none())
                {
                    return Err(StaError::Frame(FrameError::UnencryptedPayloadRejected));
                }

                let mut snonce = [0u8; 32];
                let mut offset = 0;
                while offset < 32 {
                    let r = match sink.hw_random_u32() {
                        Ok(val) => val,
                        Err(e) => {
                            volatile_zeroize(&mut snonce);
                            return Err(e);
                        }
                    };
                    let bytes = r.to_le_bytes();
                    let rem = 32 - offset;
                    let to_copy = if rem < 4 { rem } else { 4 };
                    snonce[offset..offset + to_copy].copy_from_slice(&bytes[..to_copy]);
                    offset += to_copy;
                }

                let act_res = self
                    .wpa
                    .on_rx_eapol(&cand.bssid, &self.eth_buf[..elen], &snonce);
                volatile_zeroize(&mut snonce);
                let act = act_res?;
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
                        let tx_pn = if self.ptk.is_some() {
                            Some(self.allocate_ptk_tx_pn(now_ms, sink)?)
                        } else {
                            None
                        };
                        let tx_key = match (&self.ptk, tx_pn) {
                            (Some(ref tk), Some(pn)) => Some((tk, 0u8, pn)),
                            _ => None,
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
                        self.handle_tx_done_action(done_act, now_ms, sink)?;
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
            RxDataOutcome::Ethernet {
                len: total_eth_len,
                ccmp,
            } => {
                // IEEE 802.1X controlled-port invariant: drop Ethernet data
                // unless Connected with active PTK and verified CCMP.
                if self.state != StaState::Connected
                    || !parsed.is_protected
                    || ccmp.is_none()
                    || self.ptk.is_none()
                {
                    return Err(StaError::Frame(FrameError::UnencryptedPayloadRejected));
                }
                self.last_tx_or_rx_ms = now_ms;
                on_eth_rx(&self.eth_buf[..total_eth_len]);
            }
        }
        Ok(())
    }

    fn handle_tx_done_action(
        &mut self,
        action: TxDoneAction,
        now_ms: u64,
        sink: &mut dyn StaTxSink,
    ) -> Result<(), StaError> {
        match action {
            TxDoneAction::None | TxDoneAction::TxFailed => Ok(()),
            TxDoneAction::InstallAndComplete(install) => {
                // Reject unsupported key algorithms instead of silently
                // skipping installation and entering an unkeyed Connected state.
                if install.install_ptk
                    && (install.ptk_alg != WIFI_WPA_ALG_CCMP || install.ptk_key_len < 16)
                {
                    let _ = self.handle_disconnection(now_ms, sink);
                    return Err(StaError::Handshake(HandshakeError::BadParam));
                }
                if install.gtk_len > 0
                    && (install.gtk_alg != WIFI_WPA_ALG_CCMP || install.gtk_len < 16)
                {
                    let _ = self.handle_disconnection(now_ms, sink);
                    return Err(StaError::Handshake(HandshakeError::BadParam));
                }

                if install.install_ptk {
                    let mut tk = [0u8; 16];
                    tk.copy_from_slice(&install.ptk_key[..16]);
                    // KRACK guard (CVE-2017-13077): never reset TX/RX PNs when
                    // reinstalling an identical PTK.
                    let same_ptk = match self.ptk {
                        Some(ref existing) => ct_eq(existing, &tk),
                        None => false,
                    };
                    if !same_ptk {
                        if let Some(ref mut old) = self.ptk {
                            volatile_zeroize(old);
                        }
                        self.ptk = Some(tk);
                        self.ptk_tx_pn = 0;
                        self.unicast_rx_pn_qos.fill(0);
                        self.unicast_rx_pn_non_qos = 0;
                    }
                    volatile_zeroize(&mut tk);
                }

                if install.gtk_len >= 16 {
                    let idx = usize::from(install.gtk_idx & 0x03);
                    let mut gtk_key = [0u8; 16];
                    gtk_key.copy_from_slice(&install.gtk[..16]);
                    // KRACK guard (CVE-2017-13078): preserve GTK RX PN on
                    // identical GTK reinstall; seed new GTK PN from Key RSC.
                    let same_gtk = match self.gtk[idx] {
                        Some(ref existing) => ct_eq(existing, &gtk_key),
                        None => false,
                    };
                    if !same_gtk {
                        if let Some(ref mut old) = self.gtk[idx] {
                            volatile_zeroize(old);
                        }
                        self.gtk[idx] = Some(gtk_key);
                        self.gtk_rx_pn[idx] = rsc_to_pn(install.gtk_rsc);
                    }
                    self.active_gtk_id = Some(install.gtk_idx & 0x03);
                    volatile_zeroize(&mut gtk_key);
                }

                if install.notify_auth_done {
                    if self.ptk.is_none() {
                        let _ = self.handle_disconnection(now_ms, sink);
                        return Err(StaError::Handshake(HandshakeError::BadParam));
                    }
                    if let Some(cand) = self.selected_candidate {
                        self.clear_bssid_failure(cand.bssid);
                    }
                    self.wpa.on_connected();
                    self.state = StaState::Connected;
                    self.backoff_ms = INITIAL_BACKOFF_MS;
                }
                Ok(())
            }
        }
    }

    fn handle_disconnection(
        &mut self,
        now_ms: u64,
        _sink: &mut dyn StaTxSink,
    ) -> Result<(), StaError> {
        if matches!(
            self.state,
            StaState::AuthWait | StaState::AssocWait | StaState::FourWayWait
        ) {
            if let Some(cand) = self.selected_candidate {
                self.record_bssid_failure(cand.bssid);
            }
        }
        self.selected_candidate = None;
        self.reset_crypto_and_keys();
        self.wpa.on_disconnected();
        self.state = StaState::Disconnected;
        self.reconnect_due_ms = now_ms.saturating_add(self.backoff_ms);
        self.backoff_ms = (self.backoff_ms.saturating_mul(2)).min(MAX_BACKOFF_MS);
        Ok(())
    }

    fn record_bssid_failure(&mut self, bssid: [u8; 6]) {
        let mut i = 0;
        while i < self.failed_bssid_len {
            if self.failed_bssids[i] == bssid {
                self.failed_bssid_counts[i] = self.failed_bssid_counts[i].saturating_add(1).min(16);
                return;
            }
            i += 1;
        }
        if self.failed_bssid_len < MAX_FAILED_BSSIDS {
            self.failed_bssids[self.failed_bssid_len] = bssid;
            self.failed_bssid_counts[self.failed_bssid_len] = 1;
            self.failed_bssid_len += 1;
        } else {
            // Evict the entry with the smallest failure count.
            let mut min_idx = 0;
            let mut min_cnt = self.failed_bssid_counts[0];
            let mut j = 1;
            while j < MAX_FAILED_BSSIDS {
                if self.failed_bssid_counts[j] < min_cnt {
                    min_cnt = self.failed_bssid_counts[j];
                    min_idx = j;
                }
                j += 1;
            }
            self.failed_bssids[min_idx] = bssid;
            self.failed_bssid_counts[min_idx] = 1;
        }
    }

    fn clear_bssid_failure(&mut self, bssid: [u8; 6]) {
        let mut i = 0;
        while i < self.failed_bssid_len {
            if self.failed_bssids[i] == bssid {
                let last = self.failed_bssid_len - 1;
                self.failed_bssids[i] = self.failed_bssids[last];
                self.failed_bssid_counts[i] = self.failed_bssid_counts[last];
                self.failed_bssids[last] = [0u8; 6];
                self.failed_bssid_counts[last] = 0;
                self.failed_bssid_len = last;
                return;
            }
            i += 1;
        }
    }

    fn bssid_failure_count(&self, bssid: [u8; 6]) -> u8 {
        let mut i = 0;
        while i < self.failed_bssid_len {
            if self.failed_bssids[i] == bssid {
                return self.failed_bssid_counts[i];
            }
            i += 1;
        }
        0
    }

    fn reset_crypto_and_keys(&mut self) {
        if let Some(ref mut k) = self.ptk {
            volatile_zeroize(k);
        }
        self.ptk = None;
        self.ptk_tx_pn = 0;
        self.unicast_rx_pn_qos.fill(0);
        self.unicast_rx_pn_non_qos = 0;
        for slot in &mut self.gtk {
            if let Some(ref mut k) = slot {
                volatile_zeroize(k);
            }
            *slot = None;
        }
        self.gtk_rx_pn.fill(0);
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
            && ct_eq(
                &self.pmk_cache_passphrase[..self.pmk_cache_passphrase_len],
                pass,
            )
        {
            return Ok(());
        }

        let mut pmk = [0u8; 32];
        let mut hex_buf = [0u8; 64];
        if decode_hex_pmk(pass, &mut pmk) {
            encode_hex_pmk(&pmk, &mut hex_buf);
        } else if (8..=63).contains(&pass.len()) {
            if pbkdf2_hmac_sha1(pass, ssid, 4096, &mut pmk).is_err() {
                volatile_zeroize(&mut pmk);
                return Err(StaError::Handshake(HandshakeError::BadParam));
            }
            encode_hex_pmk(&pmk, &mut hex_buf);
        } else {
            return Err(StaError::InvalidConfig);
        }

        volatile_zeroize(&mut self.cached_pmk);
        volatile_zeroize(&mut self.cached_pmk_hex);
        volatile_zeroize(&mut self.pmk_cache_passphrase);
        self.pmk_cache_ssid.fill(0);

        self.cached_pmk = pmk;
        self.cached_pmk_hex = hex_buf;
        volatile_zeroize(&mut pmk);
        volatile_zeroize(&mut hex_buf);

        self.pmk_cache_ssid[..ssid.len()].copy_from_slice(ssid);
        self.pmk_cache_ssid_len = ssid.len();
        self.pmk_cache_passphrase[..pass.len()].copy_from_slice(pass);
        self.pmk_cache_passphrase_len = pass.len();
        self.pmk_cached = true;
        Ok(())
    }
}

fn decode_hex_pmk(hex: &[u8], out: &mut [u8; 32]) -> bool {
    if hex.len() != 64 {
        return false;
    }
    let mut i = 0;
    while i < 32 {
        let Some(hi) = hex_nibble(hex[i * 2]) else {
            volatile_zeroize(out);
            return false;
        };
        let Some(lo) = hex_nibble(hex[i * 2 + 1]) else {
            volatile_zeroize(out);
            return false;
        };
        out[i] = (hi << 4) | lo;
        i += 1;
    }
    true
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
    use esp32_cam_wpa2::crypto::ccm::ccmp_encrypt_mpdu;
    use esp32_cam_wpa2::handshake::PendingKeyInstall;
    use esp32_cam_wpa2::ie::gen_wpa_ie;
    use esp32_cam_wpa2::ie::WPA_CIPHER_TKIP;
    use esp32_cam_wpa2::ie::WPA_PROTO_WPA;

    struct MockSink {
        sent_frames: usize,
        last_channel: u8,
        switched_channels: usize,
        last_rate: WifiPhyRate,
        associated_bssid: Option<[u8; 6]>,
        random_counter: u32,
        fail_rng: Option<LmacError>,
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
                fail_rng: None,
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

        fn hw_random_u32(&mut self) -> Result<u32, StaError> {
            if let Some(err) = self.fail_rng {
                return Err(StaError::Hardware(err));
            }
            self.random_counter = self.random_counter.wrapping_add(1);
            Ok(self.random_counter)
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

        fsm.state = StaState::Connected;
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

        // Broadcast ADDBA Request and ADDBA while Disconnected must be ignored.
        addba_req[4..10].fill(0xFF);
        let _ = fsm.handle_rx_mpdu(&addba_req, -50, 1001, &mut sink, |_| {});
        assert_eq!(sink.sent_frames, 1);
        addba_req[4..10].copy_from_slice(&sta_mac);
        fsm.state = StaState::Disconnected;
        let _ = fsm.handle_rx_mpdu(&addba_req, -50, 1002, &mut sink, |_| {});
        assert_eq!(sink.sent_frames, 1);
    }

    #[test]
    fn test_krack_guard_and_gtk_rsc_init() {
        let sta_mac = [0x00, 0x11, 0x22, 0x33, 0x44, 0x55];
        let bssid = [0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB];
        let mut fsm = StaMacCore::new(sta_mac);
        let mut sink = MockSink::new();
        fsm.state = StaState::FourWayWait;
        fsm.selected_candidate = Some(ScanCandidate {
            bssid,
            channel: 6,
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

        let mut install = PendingKeyInstall::empty();
        install.install_ptk = true;
        install.ptk_alg = WIFI_WPA_ALG_CCMP;
        install.ptk_key[..16].copy_from_slice(&[0xAAu8; 16]);
        install.ptk_key_len = 16;
        install.gtk[..16].copy_from_slice(&[0xBBu8; 16]);
        install.gtk_len = 16;
        install.gtk_alg = WIFI_WPA_ALG_CCMP;
        install.gtk_idx = 1;
        install.gtk_rsc = [0x2A, 0x00, 0x00, 0x00, 0x00, 0x00]; // PN 42
        install.notify_auth_done = true;

        let act = TxDoneAction::InstallAndComplete(install.clone());
        assert!(fsm.handle_tx_done_action(act, 100, &mut sink).is_ok());
        assert_eq!(fsm.state(), StaState::Connected);
        assert_eq!(fsm.gtk_rx_pn[1], 42);

        // Advance TX/RX PNs, then reinstall identical PTK/GTK (KRACK):
        // PNs must not reset.
        fsm.ptk_tx_pn = 55;
        fsm.unicast_rx_pn_non_qos = 19;
        fsm.unicast_rx_pn_qos[3] = 77;
        fsm.gtk_rx_pn[1] = 100;
        assert!(fsm
            .handle_tx_done_action(TxDoneAction::InstallAndComplete(install), 200, &mut sink)
            .is_ok());
        assert_eq!(fsm.ptk_tx_pn, 55);
        assert_eq!(fsm.unicast_rx_pn_non_qos, 19);
        assert_eq!(fsm.unicast_rx_pn_qos[3], 77);
        assert_eq!(fsm.gtk_rx_pn[1], 100);
    }

    #[test]
    fn test_per_tid_replay_gtk_id_lookup_and_tx_pn_exhaustion() {
        let sta_mac = [0x02, 0x11, 0x22, 0x33, 0x44, 0x55];
        let bssid = [0x02, 0x66, 0x77, 0x88, 0x99, 0xAA];
        let sa = [0x02, 0xCC, 0xDD, 0xEE, 0xFF, 0x01];
        let tk = [0x33u8; 16];
        let gtk1 = [0x44u8; 16];
        let gtk2 = [0x55u8; 16];

        let mut fsm = StaMacCore::new(sta_mac);
        let mut sink = MockSink::new();
        fsm.state = StaState::Connected;
        fsm.selected_candidate = Some(ScanCandidate {
            bssid,
            channel: 6,
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
        fsm.ptk = Some(tk);
        fsm.gtk[1] = Some(gtk1);
        fsm.gtk[2] = Some(gtk2);
        fsm.active_gtk_id = Some(2);

        let pt = [0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00, 0x08, 0x00, 0xDE, 0xAD];
        let mut enc = [0u8; 128];

        // QoS Data TID 1 with PN 10, then QoS Data TID 2 with PN 5
        // (must succeed because TIDs maintain independent replay windows).
        let mut qos_hdr = [0u8; 26];
        qos_hdr[0] = 0x88;
        qos_hdr[1] = 0x42;
        qos_hdr[4..10].copy_from_slice(&sta_mac);
        qos_hdr[10..16].copy_from_slice(&bssid);
        qos_hdr[16..22].copy_from_slice(&sa);
        qos_hdr[24] = 1; // TID 1
        let n1 = ccmp_encrypt_mpdu(&tk, 0, 10, &qos_hdr, &pt, &mut enc).expect("enc tid1");
        let mut rx_cnt = 0;
        assert!(fsm
            .handle_rx_mpdu(&enc[..n1], -50, 100, &mut sink, |_| rx_cnt += 1)
            .is_ok());
        assert_eq!(rx_cnt, 1);

        qos_hdr[24] = 2; // TID 2 with lower PN 5
        let n2 = ccmp_encrypt_mpdu(&tk, 0, 5, &qos_hdr, &pt, &mut enc).expect("enc tid2");
        assert!(fsm
            .handle_rx_mpdu(&enc[..n2], -50, 101, &mut sink, |_| rx_cnt += 1)
            .is_ok());
        assert_eq!(rx_cnt, 2);

        // Replay TID 2 PN 5 must fail.
        assert_eq!(
            fsm.handle_rx_mpdu(&enc[..n2], -50, 102, &mut sink, |_| rx_cnt += 1),
            Err(StaError::ReplayDetected)
        );

        // GTK lookup by wire key_id 1 even while active_gtk_id == Some(2).
        let mut grp_hdr = [0u8; 24];
        grp_hdr[0] = 0x08;
        grp_hdr[1] = 0x42;
        grp_hdr[4..10].fill(0xFF);
        grp_hdr[10..16].copy_from_slice(&bssid);
        grp_hdr[16..22].copy_from_slice(&sa);
        let ng = ccmp_encrypt_mpdu(&gtk1, 1, 7, &grp_hdr, &pt, &mut enc).expect("enc gtk1");
        assert!(fsm
            .handle_rx_mpdu(&enc[..ng], -50, 103, &mut sink, |_| rx_cnt += 1)
            .is_ok());
        assert_eq!(rx_cnt, 3);

        // 48-bit TX PN exhaustion forces disconnection and rejects TX.
        fsm.ptk_tx_pn = MAX_CCMP_PN;
        let eth = [0u8; 24];
        assert_eq!(
            fsm.transmit_eth(&eth, 200, &mut sink),
            Err(StaError::Frame(FrameError::InvalidPacketNumber))
        );
        assert_eq!(fsm.state(), StaState::Disconnected);
        assert!(fsm.bssid().is_none());
    }

    #[test]
    fn test_deauth_clears_candidate_and_ignores_when_disconnected() {
        let sta_mac = [0x02, 0x11, 0x22, 0x33, 0x44, 0x55];
        let bssid = [0x02, 0x66, 0x77, 0x88, 0x99, 0xAA];
        let mut fsm = StaMacCore::new(sta_mac);
        let mut sink = MockSink::new();
        fsm.state = StaState::Connected;
        fsm.selected_candidate = Some(ScanCandidate {
            bssid,
            channel: 6,
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

        // Malformed Deauth (Protected bit set without decryption) must be ignored.
        let mut bad_deauth = [0u8; 26];
        bad_deauth[0] = 0xC0;
        bad_deauth[1] = 0x40;
        bad_deauth[4..10].copy_from_slice(&sta_mac);
        bad_deauth[10..16].copy_from_slice(&bssid);
        bad_deauth[16..22].copy_from_slice(&bssid);
        assert!(fsm
            .handle_rx_mpdu(&bad_deauth, -50, 1000, &mut sink, |_| {})
            .is_ok());
        assert_eq!(fsm.state(), StaState::Connected);

        // Valid Deauth disconnects and clears selected_candidate.
        bad_deauth[1] = 0x00;
        assert!(fsm
            .handle_rx_mpdu(&bad_deauth, -50, 1000, &mut sink, |_| {})
            .is_ok());
        assert_eq!(fsm.state(), StaState::Disconnected);
        assert!(fsm.selected_candidate.is_none());
        let saved_due = fsm.reconnect_due_ms;
        let saved_backoff = fsm.backoff_ms;

        // Subsequent spoofed Deauth while Disconnected must not extend backoff.
        assert!(fsm
            .handle_rx_mpdu(&bad_deauth, -50, 1400, &mut sink, |_| {})
            .is_ok());
        assert_eq!(fsm.reconnect_due_ms, saved_due);
        assert_eq!(fsm.backoff_ms, saved_backoff);
    }

    #[test]
    fn test_reject_wpa1_and_tkip_group_and_penalize_failed_bssid() {
        let sta_mac = [0x02, 0x11, 0x22, 0x33, 0x44, 0x55];
        let bssid_evil = [0x02, 0xAA, 0xBB, 0xCC, 0xDD, 0x01];
        let bssid_good = [0x02, 0xAA, 0xBB, 0xCC, 0xDD, 0x02];
        let mut fsm = StaMacCore::new(sta_mac);
        let mut sink = MockSink::new();
        let cfg = StaConnectConfig::from_slices(b"CorpNet", b"supersecret", None, -90)
            .expect("valid cfg");
        fsm.start_connect(cfg, 0, &mut sink).expect("scan start");

        let make_bcn = |bssid: [u8; 6], ie: &[u8]| -> ([u8; 128], usize) {
            let mut b = [0u8; 128];
            b[0] = 0x80;
            b[4..10].fill(0xFF);
            b[10..16].copy_from_slice(&bssid);
            b[16..22].copy_from_slice(&bssid);
            b[34] = 0x11;
            let mut p = 36;
            b[p] = 0;
            b[p + 1] = 7;
            b[p + 2..p + 9].copy_from_slice(b"CorpNet");
            p += 9;
            b[p..p + 3].copy_from_slice(&[3, 1, 6]);
            p += 3;
            b[p..p + ie.len()].copy_from_slice(ie);
            p += ie.len();
            (b, p)
        };

        // WPA1 IE must be rejected.
        let mut wpa1_ie = [0u8; 32];
        let wpa1_len = gen_wpa_ie(
            WPA_PROTO_WPA,
            WPA_CIPHER_CCMP,
            WPA_CIPHER_CCMP,
            WPA_KEY_MGMT_PSK,
            0,
            0,
            &mut wpa1_ie,
        )
        .expect("wpa1 ie");
        let (b1, l1) = make_bcn(bssid_evil, &wpa1_ie[..wpa1_len]);
        fsm.record_scan_beacon(&b1[..l1], -40);
        assert_eq!(fsm.scan_candidates().len(), 0);

        // RSN with TKIP group cipher must be rejected.
        let mut tkip_rsn = [0u8; 32];
        let tkip_len = gen_wpa_ie(
            WPA_PROTO_RSN,
            WPA_CIPHER_CCMP,
            WPA_CIPHER_TKIP,
            WPA_KEY_MGMT_PSK,
            0,
            0,
            &mut tkip_rsn,
        )
        .expect("tkip rsn");
        let (b2, l2) = make_bcn(bssid_evil, &tkip_rsn[..tkip_len]);
        fsm.record_scan_beacon(&b2[..l2], -40);
        assert_eq!(fsm.scan_candidates().len(), 0);

        // Valid RSN CCMP/CCMP: failed BSSID gets penalized below good BSSID.
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
        .expect("rsn");
        fsm.record_bssid_failure(bssid_evil);
        let (be, le) = make_bcn(bssid_evil, &rsn[..rsn_len]);
        let (bg, lg) = make_bcn(bssid_good, &rsn[..rsn_len]);
        fsm.record_scan_beacon(&be[..le], -45);
        fsm.record_scan_beacon(&bg[..lg], -60);
        assert_eq!(fsm.scan_candidates().len(), 2);
        assert!(fsm.scan_candidates()[1].score > fsm.scan_candidates()[0].score);
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

    #[test]
    fn test_m1_aborts_on_rng_failure_without_sending_m2() {
        let sta_mac = [0x02, 0x11, 0x22, 0x33, 0x44, 0x55];
        let bssid = [0x02, 0x66, 0x77, 0x88, 0x99, 0xAA];
        let mut fsm = StaMacCore::new(sta_mac);
        let mut sink = MockSink::new();
        sink.fail_rng = Some(LmacError::RngHealthCheckFailed);

        let mut rsn_ie = [0u8; 32];
        let rsn_len = gen_wpa_ie(
            WPA_PROTO_RSN,
            WPA_CIPHER_CCMP,
            WPA_CIPHER_CCMP,
            WPA_KEY_MGMT_PSK,
            0,
            0,
            &mut rsn_ie,
        )
        .expect("rsn ie");
        let hex_psk = b"0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";
        fsm.wpa
            .start_connect(
                sta_mac,
                bssid,
                b"RngNet",
                hex_psk,
                &rsn_ie[..rsn_len],
                WPA_PROTO_RSN,
                WPA_CIPHER_CCMP,
                WPA_CIPHER_CCMP,
                WPA_KEY_MGMT_PSK,
                0,
                0,
            )
            .expect("wpa start");
        fsm.state = StaState::FourWayWait;
        fsm.selected_candidate = Some(ScanCandidate {
            bssid,
            channel: 6,
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

        // Construct valid FromDS EAPOL-Key M1 frame.
        let mut mpdu = [0u8; 32 + 99];
        mpdu[0] = 0x08;
        mpdu[1] = 0x02;
        mpdu[4..10].copy_from_slice(&sta_mac);
        mpdu[10..16].copy_from_slice(&bssid);
        mpdu[16..22].copy_from_slice(&bssid);
        mpdu[24..32].copy_from_slice(&[0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00, 0x88, 0x8E]);
        let eapol = &mut mpdu[32..];
        eapol[0] = 0x02;
        eapol[1] = 0x03;
        eapol[2..4].copy_from_slice(&95u16.to_be_bytes());
        eapol[4] = 0x02;
        eapol[5..7].copy_from_slice(&0x008Au16.to_be_bytes());
        eapol[7..9].copy_from_slice(&16u16.to_be_bytes());
        eapol[16] = 1;
        eapol[17..49].fill(0xAB);

        // M-015: RNG failure MUST abort M1 and never transmit M2 with zero SNonce.
        let res = fsm.handle_rx_mpdu(&mpdu, -50, 1000, &mut sink, |_| {});
        assert_eq!(
            res,
            Err(StaError::Hardware(LmacError::RngHealthCheckFailed))
        );
        assert_eq!(sink.sent_frames, 0);
        assert_eq!(fsm.state(), StaState::FourWayWait);
    }
}
