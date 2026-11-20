//! Pure-Rust STA WPA2 4-Way & Group Key Handshake State Machine.

use crate::crypto::aes::{aes_unwrap, omac1_aes_128};
use crate::crypto::ccm::rc4_skip_checked;
use crate::crypto::hash::{ct_eq, hmac_md5_vector, hmac_sha1_vector, zeroize_bytes};
use crate::ie::{
    gen_wpa_ie, parse_key_data_kdes, parse_wpa_or_rsn_ie, ParsedKeyData, WPA_CIPHER_AES_128_CMAC,
    WPA_CIPHER_AES_GMAC_128, WPA_CIPHER_AES_GMAC_256, WPA_CIPHER_TKIP, WPA_KEY_MGMT_PSK_SHA256,
    WPA_PROTO_RSN, WPA_PROTO_WPA,
};
use crate::kdf::{pbkdf2_hmac_sha1, wpa_pmk_to_ptk, Ptk};

/// TKIP cipher algorithm identifier for driver installation.
pub const WIFI_WPA_ALG_TKIP: i32 = 2;
/// CCMP (AES-128-CCM) cipher algorithm identifier for driver installation.
pub const WIFI_WPA_ALG_CCMP: i32 = 3;

/// Stack buffer wrapper that always volatile-zeroizes its contents on drop.
struct ZeroizeBuf<const N: usize>([u8; N]);

impl<const N: usize> ZeroizeBuf<N> {
    const fn new() -> Self {
        Self([0u8; N])
    }
}

impl<const N: usize> Drop for ZeroizeBuf<N> {
    fn drop(&mut self) {
        zeroize_bytes(&mut self.0);
    }
}

/// Handshake processing error classifications.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HandshakeError {
    InvalidFrame,
    InvalidKeyInfo,
    ReplayRollback,
    MicMismatch,
    IeMismatch,
    DecryptFailed,
    BadParam,
    NoPmk,
}

/// State of the STA WPA2 state machine.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HandshakeState {
    Disconnected,
    Associating,
    FourWayM1Done,
    FourWayM3PendingTx,
    GroupRekeyPendingTx,
    Completed,
}

/// Parameters for atomic hardware key installation.
#[derive(Clone, Eq)]
pub struct PendingKeyInstall {
    pub install_ptk: bool,
    pub ptk_alg: i32,
    pub ptk_key: [u8; 32],
    pub ptk_key_len: usize,
    pub gtk: [u8; 32],
    pub gtk_len: usize,
    pub gtk_alg: i32,
    pub gtk_idx: u8,
    pub gtk_rsc: [u8; 6],
    pub igtk_idx: u16,
    pub igtk_pn: [u8; 6],
    pub igtk: [u8; 32],
    pub has_igtk: bool,
    pub notify_auth_done: bool,
}

impl PendingKeyInstall {
    pub const fn empty() -> Self {
        Self {
            install_ptk: false,
            ptk_alg: 0,
            ptk_key: [0u8; 32],
            ptk_key_len: 0,
            gtk: [0u8; 32],
            gtk_len: 0,
            gtk_alg: 0,
            gtk_idx: 0,
            gtk_rsc: [0u8; 6],
            igtk_idx: 0,
            igtk_pn: [0u8; 6],
            igtk: [0u8; 32],
            has_igtk: false,
            notify_auth_done: false,
        }
    }
}

impl Drop for PendingKeyInstall {
    fn drop(&mut self) {
        zeroize_bytes(&mut self.ptk_key);
        zeroize_bytes(&mut self.gtk);
        zeroize_bytes(&mut self.igtk);
    }
}

impl core::fmt::Debug for PendingKeyInstall {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("PendingKeyInstall")
            .field("install_ptk", &self.install_ptk)
            .field("ptk_alg", &self.ptk_alg)
            .field("ptk_key", &"[REDACTED]")
            .field("ptk_key_len", &self.ptk_key_len)
            .field("gtk", &"[REDACTED]")
            .field("gtk_len", &self.gtk_len)
            .field("gtk_alg", &self.gtk_alg)
            .field("gtk_idx", &self.gtk_idx)
            .field("gtk_rsc", &self.gtk_rsc)
            .field("igtk_idx", &self.igtk_idx)
            .field("igtk_pn", &self.igtk_pn)
            .field("igtk", &"[REDACTED]")
            .field("has_igtk", &self.has_igtk)
            .field("notify_auth_done", &self.notify_auth_done)
            .finish()
    }
}

impl PartialEq for PendingKeyInstall {
    fn eq(&self, other: &Self) -> bool {
        self.install_ptk == other.install_ptk
            && self.ptk_alg == other.ptk_alg
            && self.ptk_key_len == other.ptk_key_len
            && self.gtk_len == other.gtk_len
            && self.gtk_alg == other.gtk_alg
            && self.gtk_idx == other.gtk_idx
            && self.gtk_rsc == other.gtk_rsc
            && self.igtk_idx == other.igtk_idx
            && self.igtk_pn == other.igtk_pn
            && self.has_igtk == other.has_igtk
            && self.notify_auth_done == other.notify_auth_done
            && (ct_eq(&self.ptk_key, &other.ptk_key)
                & ct_eq(&self.gtk, &other.gtk)
                & ct_eq(&self.igtk, &other.igtk))
    }
}

/// EAPOL frame buffer for transmission (excluding 14B Ethernet header).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EapolTxFrame {
    pub data: [u8; 384],
    pub len: usize,
}

impl EapolTxFrame {
    pub const fn new() -> Self {
        Self {
            data: [0u8; 384],
            len: 0,
        }
    }
}

impl Default for EapolTxFrame {
    fn default() -> Self {
        Self::new()
    }
}

/// Action to perform upon processing an incoming EAPOL frame.
// Rationale: In #![no_std] bare-metal firmware, EapolTxFrame is stored inline
// without heap allocation; stack frames remain bounded and statically sized.
#[allow(clippy::large_enum_variant)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RxEapolAction {
    Ignore,
    SendFrame(EapolTxFrame),
    Deauth(u8),
}

/// Action resulting from an EAPOL transmission completion notification.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum TxDoneAction {
    None,
    TxFailed,
    InstallAndComplete(PendingKeyInstall),
}

/// Station-side WPA/WPA2 4-Way & Group Key state machine.
pub struct WpaStaStateMachine {
    state: HandshakeState,
    own_addr: [u8; 6],
    bssid: [u8; 6],
    pmk: [u8; 32],
    has_pmk: bool,
    proto: i32,
    pairwise_supp: u32,
    group_supp: u32,
    key_mgmt: u32,
    mgmt_group_supp: u32,
    rsn_caps: u16,
    ap_rsn_caps: u16,
    assoc_ie: [u8; 128],
    assoc_ie_len: usize,
    ap_ie: [u8; 128],
    ap_ie_len: usize,
    snonce: [u8; 32],
    snonce_renew: bool,
    tptk_anonce: [u8; 32],
    ptk_anonce: [u8; 32],
    tptk: Ptk,
    tptk_set: bool,
    ptk: Ptk,
    ptk_set: bool,
    ptk_installed: bool,
    installed_ptk: [u8; 32],
    installed_ptk_len: usize,
    gtk_installed: bool,
    installed_gtk: [u8; 32],
    installed_gtk_len: usize,
    installed_gtk_idx: u8,
    igtk_installed: bool,
    installed_igtk: [u8; 32],
    installed_igtk_idx: u16,
    replay_counter: [u8; 8],
    replay_set: bool,
    pending_install: Option<PendingKeyInstall>,
}

impl WpaStaStateMachine {
    pub const fn new() -> Self {
        Self {
            state: HandshakeState::Disconnected,
            own_addr: [0u8; 6],
            bssid: [0u8; 6],
            pmk: [0u8; 32],
            has_pmk: false,
            proto: 0,
            pairwise_supp: 0,
            group_supp: 0,
            key_mgmt: 0,
            mgmt_group_supp: 0,
            rsn_caps: 0,
            ap_rsn_caps: 0,
            assoc_ie: [0u8; 128],
            assoc_ie_len: 0,
            ap_ie: [0u8; 128],
            ap_ie_len: 0,
            snonce: [0u8; 32],
            snonce_renew: true,
            tptk_anonce: [0u8; 32],
            ptk_anonce: [0u8; 32],
            tptk: Ptk::zeroed(),
            tptk_set: false,
            ptk: Ptk::zeroed(),
            ptk_set: false,
            ptk_installed: false,
            installed_ptk: [0u8; 32],
            installed_ptk_len: 0,
            gtk_installed: false,
            installed_gtk: [0u8; 32],
            installed_gtk_len: 0,
            installed_gtk_idx: 0,
            igtk_installed: false,
            installed_igtk: [0u8; 32],
            installed_igtk_idx: 0,
            replay_counter: [0u8; 8],
            replay_set: false,
            pending_install: None,
        }
    }

    fn clear_ephemeral_secrets(&mut self) {
        zeroize_bytes(&mut self.snonce);
        zeroize_bytes(&mut self.tptk_anonce);
        zeroize_bytes(&mut self.ptk_anonce);
        self.tptk.zeroize();
        self.tptk_set = false;
        self.ptk.zeroize();
        self.ptk_set = false;
        zeroize_bytes(&mut self.installed_ptk);
        self.installed_ptk_len = 0;
        self.ptk_installed = false;
        zeroize_bytes(&mut self.installed_gtk);
        self.installed_gtk_len = 0;
        self.installed_gtk_idx = 0;
        self.gtk_installed = false;
        zeroize_bytes(&mut self.installed_igtk);
        self.installed_igtk_idx = 0;
        self.igtk_installed = false;
        zeroize_bytes(&mut self.replay_counter);
        self.replay_set = false;
        self.pending_install = None;
    }

    pub fn reset(&mut self) {
        self.state = HandshakeState::Disconnected;
        self.assoc_ie_len = 0;
        self.ap_ie_len = 0;
        self.ap_rsn_caps = 0;
        self.snonce_renew = true;
        self.clear_ephemeral_secrets();
    }

    pub fn set_cached_pmk(&mut self, pmk: &[u8; 32]) {
        self.pmk = *pmk;
        self.has_pmk = true;
    }

    pub fn clear_pmk(&mut self) {
        zeroize_bytes(&mut self.pmk);
        self.has_pmk = false;
    }

    pub fn on_connected(&mut self) {
        self.state = HandshakeState::Completed;
        self.snonce_renew = true;
    }

    pub fn on_disconnected(&mut self) {
        self.reset();
    }

    pub fn in_4way_handshake(&self) -> bool {
        matches!(
            self.state,
            HandshakeState::Associating
                | HandshakeState::FourWayM1Done
                | HandshakeState::FourWayM3PendingTx
        )
    }

    fn expected_d_type(&self) -> Result<u8, HandshakeError> {
        match self.proto {
            WPA_PROTO_RSN => Ok(2),
            WPA_PROTO_WPA => Ok(254),
            _ => Err(HandshakeError::BadParam),
        }
    }

    fn expected_kver(&self) -> Result<u16, HandshakeError> {
        if (self.key_mgmt & WPA_KEY_MGMT_PSK_SHA256) != 0 {
            Ok(3)
        } else if self.pairwise_supp == WPA_CIPHER_TKIP {
            Ok(1)
        } else {
            Ok(2)
        }
    }

    fn pmf_active(&self) -> bool {
        self.proto == WPA_PROTO_RSN
            && ((self.rsn_caps & 0x0040) != 0
                || ((self.rsn_caps & 0x0080) != 0 && (self.ap_rsn_caps & 0x0080) != 0))
    }

    #[allow(clippy::too_many_arguments)]
    pub fn start_connect(
        &mut self,
        own_addr: [u8; 6],
        bssid: [u8; 6],
        ssid: &[u8],
        passphrase_or_psk: &[u8],
        ap_ie_bytes: &[u8],
        proto: i32,
        pairwise_supp: u32,
        group_supp: u32,
        key_mgmt: u32,
        mgmt_group_supp: u32,
        rsn_caps: u16,
    ) -> Result<([u8; 128], usize), HandshakeError> {
        // Validate AP IE first before spending time on PBKDF2.
        if ap_ie_bytes.is_empty() || ap_ie_bytes.len() > self.ap_ie.len() {
            return Err(HandshakeError::BadParam);
        }
        let ap_parsed = parse_wpa_or_rsn_ie(ap_ie_bytes).map_err(|_| HandshakeError::BadParam)?;
        if ap_parsed.proto != proto {
            return Err(HandshakeError::BadParam);
        }

        // Derive/decode PMK directly into a local zeroizing buffer before committing.
        let mut new_pmk = ZeroizeBuf::<32>::new();
        let has_new_pmk = derive_or_decode_pmk(ssid, passphrase_or_psk, &mut new_pmk.0)?;
        let pmk_opt = if has_new_pmk { Some(&new_pmk.0) } else { None };
        self.start_connect_with_pmk(
            own_addr,
            bssid,
            pmk_opt,
            ap_ie_bytes,
            proto,
            pairwise_supp,
            group_supp,
            key_mgmt,
            mgmt_group_supp,
            rsn_caps,
        )
    }

    /// Starts a connection using an already-derived/decoded PMK (or `None` to
    /// reuse a previously cached PMK), allowing callers holding spinlocks or
    /// interrupt masks to run PBKDF2 outside the critical section.
    #[allow(clippy::too_many_arguments)]
    pub fn start_connect_with_pmk(
        &mut self,
        own_addr: [u8; 6],
        bssid: [u8; 6],
        pmk: Option<&[u8; 32]>,
        ap_ie_bytes: &[u8],
        proto: i32,
        pairwise_supp: u32,
        group_supp: u32,
        key_mgmt: u32,
        mgmt_group_supp: u32,
        rsn_caps: u16,
    ) -> Result<([u8; 128], usize), HandshakeError> {
        // Validate AP IE first: reject empty, oversized, or protocol-mismatched AP IE.
        if ap_ie_bytes.is_empty() || ap_ie_bytes.len() > self.ap_ie.len() {
            return Err(HandshakeError::BadParam);
        }
        let ap_parsed = parse_wpa_or_rsn_ie(ap_ie_bytes).map_err(|_| HandshakeError::BadParam)?;
        if ap_parsed.proto != proto {
            return Err(HandshakeError::BadParam);
        }

        // Validate STA AssocReq IE generation before mutating self.
        let mut abuf = [0u8; 128];
        let alen = gen_wpa_ie(
            proto,
            pairwise_supp,
            group_supp,
            key_mgmt,
            rsn_caps,
            mgmt_group_supp,
            &mut abuf,
        )
        .map_err(|_| HandshakeError::BadParam)?;

        if pmk.is_none() && !self.has_pmk {
            return Err(HandshakeError::BadParam);
        }

        // All validation passed; clear stale per-connection state and commit.
        self.clear_ephemeral_secrets();
        if let Some(new_pmk) = pmk {
            self.pmk = *new_pmk;
            self.has_pmk = true;
        }
        self.ap_ie.fill(0);
        self.ap_ie[..ap_ie_bytes.len()].copy_from_slice(ap_ie_bytes);
        self.ap_ie_len = ap_ie_bytes.len();
        self.ap_rsn_caps = ap_parsed.capabilities;
        self.assoc_ie = abuf;
        self.assoc_ie_len = alen;
        self.own_addr = own_addr;
        self.bssid = bssid;
        self.proto = proto;
        self.pairwise_supp = pairwise_supp;
        self.group_supp = group_supp;
        self.key_mgmt = key_mgmt;
        self.mgmt_group_supp = mgmt_group_supp;
        self.rsn_caps = rsn_caps;
        self.state = HandshakeState::Associating;
        self.snonce_renew = true;
        Ok((abuf, alen))
    }

    pub fn on_rx_eapol(
        &mut self,
        src: &[u8; 6],
        eapol: &[u8],
        snonce_rand: &[u8; 32],
    ) -> Result<RxEapolAction, HandshakeError> {
        if self.state == HandshakeState::Disconnected {
            return Err(HandshakeError::InvalidKeyInfo);
        }
        if src != &self.bssid {
            return Ok(RxEapolAction::Ignore);
        }
        if eapol.len() < 4 || (eapol[0] != 1 && eapol[0] != 2) || eapol[1] != 3 {
            return Err(HandshakeError::InvalidFrame);
        }
        let blen = u16::from_be_bytes([eapol[2], eapol[3]]) as usize;
        if eapol.len() < 4 + blen || blen < 95 {
            return Err(HandshakeError::InvalidFrame);
        }
        let frame = &eapol[..4 + blen];
        let d_type = frame[4];
        if d_type != self.expected_d_type()? {
            return Err(HandshakeError::InvalidFrame);
        }
        let ki = u16::from_be_bytes([frame[5], frame[6]]);
        let kver = ki & 0x0007;
        let pair = (ki & 0x0008) != 0;
        let install = (ki & 0x0040) != 0;
        let ack = (ki & 0x0080) != 0;
        let mic = (ki & 0x0100) != 0;
        let sec = (ki & 0x0200) != 0;
        let err_bit = (ki & 0x0400) != 0;
        let req_bit = (ki & 0x0800) != 0;
        let enc = (ki & 0x1000) != 0;
        let smk = (ki & 0x2000) != 0;

        if kver != self.expected_kver()? || err_bit || req_bit || smk {
            return Err(HandshakeError::InvalidKeyInfo);
        }

        let mut rep = [0u8; 8];
        rep.copy_from_slice(&frame[9..17]);
        let mut nonce = [0u8; 32];
        nonce.copy_from_slice(&frame[17..49]);
        let mut iv = [0u8; 16];
        iv.copy_from_slice(&frame[49..65]);
        let mut rsc = [0u8; 8];
        rsc.copy_from_slice(&frame[65..73]);
        let mut rx_mic = [0u8; 16];
        rx_mic.copy_from_slice(&frame[81..97]);
        let kdlen = u16::from_be_bytes([frame[97], frame[98]]) as usize;
        if 99 + kdlen > frame.len() {
            return Err(HandshakeError::InvalidFrame);
        }
        let kd = &frame[99..99 + kdlen];
        if self.replay_set && rep <= self.replay_counter {
            return Err(HandshakeError::ReplayRollback);
        }

        if pair && ack && !mic {
            // 4-Way Handshake Message 1 (unauthenticated).
            if install || sec || enc {
                return Err(HandshakeError::InvalidKeyInfo);
            }
            if !self.has_pmk {
                return Err(HandshakeError::NoPmk);
            }
            if self.snonce_renew {
                self.snonce = *snonce_rand;
                self.snonce_renew = false;
            }
            self.tptk_anonce = nonce;
            let sha = (self.key_mgmt & WPA_KEY_MGMT_PSK_SHA256) != 0;
            self.tptk.zeroize();
            self.tptk = wpa_pmk_to_ptk(
                &self.pmk,
                b"Pairwise key expansion",
                &self.own_addr,
                &self.bssid,
                &self.snonce,
                &self.tptk_anonce,
                sha,
            );
            self.tptk_set = true;
            // Never mutate replay_counter / replay_set on unauthenticated M1.
            let m2_ki = (kver & 0x07) | 0x0108;
            let kl = if d_type == 2 {
                0
            } else if self.pairwise_supp == WPA_CIPHER_TKIP {
                32
            } else {
                0
            };
            let m2 = build_eapol(
                d_type,
                m2_ki,
                kl,
                rep,
                self.snonce,
                &self.assoc_ie[..self.assoc_ie_len],
                &self.tptk.kck,
            )?;
            // Never downgrade Completed (or pending TX) on unauthenticated M1.
            if self.state == HandshakeState::Associating {
                self.state = HandshakeState::FourWayM1Done;
            }
            Ok(RxEapolAction::SendFrame(m2))
        } else if pair && ack && mic {
            // 4-Way Handshake Message 3.
            if !install {
                return Err(HandshakeError::InvalidKeyInfo);
            }
            if self.proto == WPA_PROTO_RSN && (!enc || !sec || kd.is_empty()) {
                return Err(HandshakeError::InvalidKeyInfo);
            }
            if !self.tptk_set && !self.ptk_set {
                return Err(HandshakeError::BadParam);
            }
            let tptk_verified = self.tptk_set && verify_mic(frame, kver, &self.tptk.kck, &rx_mic)?;
            let ptk_verified =
                !tptk_verified && self.ptk_set && verify_mic(frame, kver, &self.ptk.kck, &rx_mic)?;
            if !tptk_verified && !ptk_verified {
                return Err(HandshakeError::MicMismatch);
            }
            let expected_anonce = if tptk_verified {
                &self.tptk_anonce
            } else {
                &self.ptk_anonce
            };
            if !ct_eq(&nonce, expected_anonce) {
                return Err(HandshakeError::BadParam);
            }
            // KRACK guard: reject M3 in Completed unless a fresh M1 derived
            // a distinct tptk.
            if self.state == HandshakeState::Completed && !tptk_verified {
                return Err(HandshakeError::InvalidKeyInfo);
            }

            let active_kek: &[u8; 16] = if tptk_verified {
                &self.tptk.kek
            } else {
                &self.ptk.kek
            };
            let mut pbuf = ZeroizeBuf::<256>::new();
            let pslice = if enc && !kd.is_empty() {
                let plen = decrypt_kd(kver, active_kek, &iv, kd, &mut pbuf.0)?;
                &pbuf.0[..plen]
            } else {
                kd
            };
            let pk = parse_key_data_kdes(pslice).map_err(|_| HandshakeError::InvalidFrame)?;
            let exp = &self.ap_ie[..self.ap_ie_len];
            let ie_ok = if self.proto == WPA_PROTO_RSN {
                pk.rsn_ie == Some(exp)
            } else {
                pk.wpa_ie == Some(exp)
            };
            if !ie_ok {
                return Err(HandshakeError::IeMismatch);
            }

            let active_ptk = if tptk_verified { &self.tptk } else { &self.ptk };
            let mut pi = PendingKeyInstall::empty();
            pi.install_ptk = true;
            pi.notify_auth_done = true;
            if self.pairwise_supp == WPA_CIPHER_TKIP {
                pi.ptk_alg = WIFI_WPA_ALG_TKIP;
                pi.ptk_key_len = 32;
                pi.ptk_key[..16].copy_from_slice(&active_ptk.tk);
                pi.ptk_key[16..24].copy_from_slice(&active_ptk.tx_mic);
                pi.ptk_key[24..32].copy_from_slice(&active_ptk.rx_mic);
            } else {
                pi.ptk_alg = WIFI_WPA_ALG_CCMP;
                pi.ptk_key_len = 16;
                pi.ptk_key[..16].copy_from_slice(&active_ptk.tk);
            }

            let same_ptk = self.ptk_installed
                && self.installed_ptk_len == pi.ptk_key_len
                && ct_eq(
                    &self.installed_ptk[..pi.ptk_key_len],
                    &pi.ptk_key[..pi.ptk_key_len],
                );
            if self.state == HandshakeState::Completed && same_ptk {
                return Err(HandshakeError::InvalidKeyInfo);
            }
            if same_ptk {
                pi.install_ptk = false;
                zeroize_bytes(&mut pi.ptk_key);
                pi.ptk_key_len = 0;
            }

            if self.proto == WPA_PROTO_RSN {
                populate_gtk(
                    &mut pi,
                    &pk,
                    &rsc,
                    self.group_supp,
                    self.mgmt_group_supp,
                    self.pmf_active(),
                    true,
                )?;
            }
            self.strip_already_installed_group_keys(&mut pi);

            if tptk_verified {
                core::mem::swap(&mut self.ptk, &mut self.tptk);
                self.ptk_set = true;
                self.ptk_anonce = self.tptk_anonce;
                self.tptk.zeroize();
                self.tptk_set = false;
                zeroize_bytes(&mut self.tptk_anonce);
            }
            self.replay_counter = rep;
            self.replay_set = true;

            let m4_ki = (kver & 0x07) | 0x0108 | if sec { 0x0200 } else { 0 };
            let m4 = build_eapol(d_type, m4_ki, 0, rep, [0u8; 32], &[], &self.ptk.kck)?;
            if pi.install_ptk || pi.gtk_len > 0 || pi.has_igtk || !self.ptk_installed {
                self.pending_install = Some(pi);
                self.state = HandshakeState::FourWayM3PendingTx;
            }
            Ok(RxEapolAction::SendFrame(m4))
        } else if !pair && ack && mic && sec {
            // Group Key Handshake Message 1.
            if install {
                return Err(HandshakeError::InvalidKeyInfo);
            }
            if !self.ptk_set
                || !matches!(
                    self.state,
                    HandshakeState::Completed | HandshakeState::GroupRekeyPendingTx
                )
            {
                return Err(HandshakeError::BadParam);
            }
            if self.proto == WPA_PROTO_RSN && !enc {
                return Err(HandshakeError::InvalidKeyInfo);
            }
            if kd.is_empty() {
                return Err(HandshakeError::InvalidFrame);
            }
            if !verify_mic(frame, kver, &self.ptk.kck, &rx_mic)? {
                return Err(HandshakeError::MicMismatch);
            }
            let mut pbuf = ZeroizeBuf::<256>::new();
            let plen = decrypt_kd(kver, &self.ptk.kek, &iv, kd, &mut pbuf.0)?;
            let pslice = &pbuf.0[..plen];

            let mut pi = PendingKeyInstall::empty();
            pi.notify_auth_done = true;
            if self.proto == WPA_PROTO_RSN {
                let pk = parse_key_data_kdes(pslice).map_err(|_| HandshakeError::InvalidFrame)?;
                populate_gtk(
                    &mut pi,
                    &pk,
                    &rsc,
                    self.group_supp,
                    self.mgmt_group_supp,
                    false,
                    true,
                )?;
            } else {
                populate_wpa1_gtk(&mut pi, pslice, ki, &rsc, self.group_supp)?;
            }
            self.strip_already_installed_group_keys(&mut pi);

            self.replay_counter = rep;
            self.replay_set = true;
            let gm2_ki = (kver & 0x07)
                | 0x0300
                | if self.proto == WPA_PROTO_WPA {
                    ki & 0x30
                } else {
                    0
                };
            let gm2 = build_eapol(d_type, gm2_ki, 0, rep, [0u8; 32], &[], &self.ptk.kck)?;
            if pi.gtk_len > 0 || pi.has_igtk {
                self.pending_install = Some(pi);
                self.state = HandshakeState::GroupRekeyPendingTx;
            } else {
                self.pending_install = None;
                self.state = HandshakeState::Completed;
            }
            Ok(RxEapolAction::SendFrame(gm2))
        } else {
            Err(HandshakeError::InvalidKeyInfo)
        }
    }

    fn strip_already_installed_group_keys(&self, pi: &mut PendingKeyInstall) {
        if pi.gtk_len > 0
            && self.gtk_installed
            && self.installed_gtk_idx == pi.gtk_idx
            && self.installed_gtk_len == pi.gtk_len
            && ct_eq(&self.installed_gtk[..pi.gtk_len], &pi.gtk[..pi.gtk_len])
        {
            zeroize_bytes(&mut pi.gtk);
            pi.gtk_len = 0;
        }
        if pi.has_igtk
            && self.igtk_installed
            && self.installed_igtk_idx == pi.igtk_idx
            && ct_eq(&self.installed_igtk, &pi.igtk)
        {
            zeroize_bytes(&mut pi.igtk);
            pi.has_igtk = false;
        }
    }

    pub fn on_eapol_tx_complete(&mut self, tx_failure: bool) -> TxDoneAction {
        if !matches!(
            self.state,
            HandshakeState::FourWayM3PendingTx | HandshakeState::GroupRekeyPendingTx
        ) {
            return TxDoneAction::None;
        }
        if tx_failure {
            self.pending_install = None;
            self.state = match self.state {
                HandshakeState::FourWayM3PendingTx => {
                    if self.ptk_installed {
                        HandshakeState::Completed
                    } else {
                        HandshakeState::FourWayM1Done
                    }
                }
                HandshakeState::GroupRekeyPendingTx => HandshakeState::Completed,
                other => other,
            };
            TxDoneAction::TxFailed
        } else if let Some(pi) = self.pending_install.take() {
            if pi.install_ptk {
                self.installed_ptk.copy_from_slice(&pi.ptk_key);
                self.installed_ptk_len = pi.ptk_key_len;
                self.ptk_installed = true;
            }
            if pi.gtk_len > 0 {
                self.installed_gtk.copy_from_slice(&pi.gtk);
                self.installed_gtk_len = pi.gtk_len;
                self.installed_gtk_idx = pi.gtk_idx;
                self.gtk_installed = true;
            }
            if pi.has_igtk {
                self.installed_igtk.copy_from_slice(&pi.igtk);
                self.installed_igtk_idx = pi.igtk_idx;
                self.igtk_installed = true;
            }
            self.state = HandshakeState::Completed;
            self.snonce_renew = true;
            TxDoneAction::InstallAndComplete(pi)
        } else {
            self.state = HandshakeState::Completed;
            TxDoneAction::None
        }
    }
}

impl Drop for WpaStaStateMachine {
    fn drop(&mut self) {
        self.clear_pmk();
        self.clear_ephemeral_secrets();
    }
}

impl Default for WpaStaStateMachine {
    fn default() -> Self {
        Self::new()
    }
}

fn compute_mic(kver: u16, kck: &[u8; 16], frame: &[u8]) -> Result<[u8; 16], HandshakeError> {
    match kver {
        1 => Ok(hmac_md5_vector(kck, &[frame])),
        2 => {
            let mut full = ZeroizeBuf::<20>::new();
            full.0 = hmac_sha1_vector(kck, &[frame]);
            let mut mic = [0u8; 16];
            mic.copy_from_slice(&full.0[..16]);
            Ok(mic)
        }
        3 => Ok(omac1_aes_128(kck, frame)),
        _ => Err(HandshakeError::InvalidKeyInfo),
    }
}

fn verify_mic(f: &[u8], kv: u16, kck: &[u8; 16], rx: &[u8; 16]) -> Result<bool, HandshakeError> {
    let mut z = ZeroizeBuf::<384>::new();
    if f.len() > z.0.len() {
        return Err(HandshakeError::InvalidFrame);
    }
    z.0[..f.len()].copy_from_slice(f);
    z.0[81..97].fill(0);
    let mut expected = ZeroizeBuf::<16>::new();
    expected.0 = compute_mic(kv, kck, &z.0[..f.len()])?;
    Ok(ct_eq(&expected.0, rx))
}

#[allow(clippy::too_many_arguments)]
fn build_eapol(
    desc: u8,
    ki: u16,
    kl: u16,
    rep: [u8; 8],
    nonce: [u8; 32],
    kd: &[u8],
    kck: &[u8; 16],
) -> Result<EapolTxFrame, HandshakeError> {
    let blen = 95 + kd.len();
    let total = 4 + blen;
    if total > 384 {
        return Err(HandshakeError::BadParam);
    }
    let mut tx = EapolTxFrame::new();
    tx.len = total;
    tx.data[0] = 1;
    tx.data[1] = 3;
    let (bl, ki_b, kl_b, kd_b) = (
        (blen as u16).to_be_bytes(),
        ki.to_be_bytes(),
        kl.to_be_bytes(),
        (kd.len() as u16).to_be_bytes(),
    );
    tx.data[2..4].copy_from_slice(&bl);
    tx.data[4] = desc;
    tx.data[5..7].copy_from_slice(&ki_b);
    tx.data[7..9].copy_from_slice(&kl_b);
    tx.data[9..17].copy_from_slice(&rep);
    tx.data[17..49].copy_from_slice(&nonce);
    tx.data[97..99].copy_from_slice(&kd_b);
    if !kd.is_empty() {
        tx.data[99..total].copy_from_slice(kd);
    }
    let mic = compute_mic(ki & 0x07, kck, &tx.data[..total])?;
    tx.data[81..97].copy_from_slice(&mic);
    Ok(tx)
}

fn decrypt_kd(
    kv: u16,
    kek: &[u8; 16],
    iv: &[u8; 16],
    kd: &[u8],
    out: &mut [u8; 256],
) -> Result<usize, HandshakeError> {
    if kv == 2 || kv == 3 {
        if kd.len() < 16 || !kd.len().is_multiple_of(8) || kd.len() - 8 > out.len() {
            return Err(HandshakeError::DecryptFailed);
        }
        let l = kd.len() - 8;
        if aes_unwrap(kek, kd, &mut out[..l]).is_err() {
            zeroize_bytes(&mut out[..l]);
            return Err(HandshakeError::DecryptFailed);
        }
        Ok(l)
    } else if kv == 1 {
        if kd.is_empty() || kd.len() > out.len() {
            return Err(HandshakeError::DecryptFailed);
        }
        let mut rc4_k = ZeroizeBuf::<32>::new();
        rc4_k.0[..16].copy_from_slice(iv);
        rc4_k.0[16..32].copy_from_slice(kek);
        out[..kd.len()].copy_from_slice(kd);
        if rc4_skip_checked(&rc4_k.0, 256, &mut out[..kd.len()]).is_err() {
            zeroize_bytes(&mut out[..kd.len()]);
            return Err(HandshakeError::DecryptFailed);
        }
        Ok(kd.len())
    } else {
        Err(HandshakeError::InvalidKeyInfo)
    }
}

fn expected_gtk_len_and_alg(group_supp: u32) -> (usize, i32) {
    if group_supp == WPA_CIPHER_TKIP {
        (32, WIFI_WPA_ALG_TKIP)
    } else {
        (16, WIFI_WPA_ALG_CCMP)
    }
}

fn expected_igtk_len(mgmt_group_supp: u32) -> Result<usize, HandshakeError> {
    match mgmt_group_supp {
        WPA_CIPHER_AES_128_CMAC | WPA_CIPHER_AES_GMAC_128 => Ok(16),
        WPA_CIPHER_AES_GMAC_256 => Ok(32),
        _ => Err(HandshakeError::BadParam),
    }
}

fn swap_tkip_mic_halves(key: &mut [u8; 32]) {
    let mut mic_tx = ZeroizeBuf::<8>::new();
    mic_tx.0.copy_from_slice(&key[16..24]);
    key.copy_within(24..32, 16);
    key[24..32].copy_from_slice(&mic_tx.0);
}

fn populate_wpa1_gtk(
    pi: &mut PendingKeyInstall,
    gtk: &[u8],
    ki: u16,
    rsc: &[u8; 8],
    group_supp: u32,
) -> Result<(), HandshakeError> {
    let (exp_len, alg) = expected_gtk_len_and_alg(group_supp);
    if gtk.len() != exp_len {
        return Err(HandshakeError::BadParam);
    }
    pi.gtk[..exp_len].copy_from_slice(gtk);
    if exp_len == 32 {
        swap_tkip_mic_halves(&mut pi.gtk);
    }
    pi.gtk_len = exp_len;
    pi.gtk_idx = ((ki & 0x30) >> 4) as u8;
    pi.gtk_alg = alg;
    pi.gtk_rsc.copy_from_slice(&rsc[..6]);
    Ok(())
}

fn populate_gtk(
    pi: &mut PendingKeyInstall,
    pk: &ParsedKeyData,
    rsc: &[u8; 8],
    group_supp: u32,
    mgmt_group_supp: u32,
    require_igtk: bool,
    require_gtk: bool,
) -> Result<(), HandshakeError> {
    let (exp_gtk_len, gtk_alg) = expected_gtk_len_and_alg(group_supp);
    match pk.gtk {
        Some(gtk) => {
            if gtk.len() != exp_gtk_len {
                return Err(HandshakeError::BadParam);
            }
            pi.gtk[..exp_gtk_len].copy_from_slice(gtk);
            if exp_gtk_len == 32 {
                swap_tkip_mic_halves(&mut pi.gtk);
            }
            pi.gtk_len = exp_gtk_len;
            pi.gtk_idx = pk.gtk_key_id;
            pi.gtk_alg = gtk_alg;
            pi.gtk_rsc.copy_from_slice(&rsc[..6]);
        }
        None if require_gtk => return Err(HandshakeError::InvalidFrame),
        None => {}
    }

    match pk.igtk {
        Some(igtk) => {
            let exp_igtk_len = expected_igtk_len(mgmt_group_supp)?;
            if igtk.len() != exp_igtk_len {
                return Err(HandshakeError::BadParam);
            }
            pi.igtk[..exp_igtk_len].copy_from_slice(igtk);
            pi.igtk_idx = pk.igtk_key_id;
            pi.igtk_pn = pk.igtk_ipn;
            pi.has_igtk = true;
        }
        None if require_igtk => return Err(HandshakeError::InvalidFrame),
        None => {}
    }
    Ok(())
}

/// Derives a 32-byte PMK via PBKDF2-HMAC-SHA1 (8..=63 byte passphrase) or
/// decodes a 64-hex-digit PSK directly into `out`, zeroizing `out` on error.
///
/// Returns `Ok(true)` if a new PMK was written into `out`, or `Ok(false)` if
/// `passphrase_or_psk` is empty (signaling cached-PMK reuse).
pub fn derive_or_decode_pmk(
    ssid: &[u8],
    passphrase_or_psk: &[u8],
    out: &mut [u8; 32],
) -> Result<bool, HandshakeError> {
    if decode_hex_pmk(passphrase_or_psk, out) {
        Ok(true)
    } else if (8..=63).contains(&passphrase_or_psk.len()) {
        if pbkdf2_hmac_sha1(passphrase_or_psk, ssid, 4096, out).is_err() {
            zeroize_bytes(out);
            return Err(HandshakeError::BadParam);
        }
        Ok(true)
    } else if passphrase_or_psk.is_empty() {
        Ok(false)
    } else {
        zeroize_bytes(out);
        Err(HandshakeError::BadParam)
    }
}

fn decode_hex_pmk(hex: &[u8], out: &mut [u8; 32]) -> bool {
    if hex.len() != 64 {
        return false;
    }
    for (i, byte) in out.iter_mut().enumerate() {
        let Some(hi) = hex_val(hex[i * 2]) else {
            zeroize_bytes(out);
            return false;
        };
        let Some(lo) = hex_val(hex[i * 2 + 1]) else {
            zeroize_bytes(out);
            return false;
        };
        *byte = (hi << 4) | lo;
    }
    true
}

const fn hex_val(b: u8) -> Option<u8> {
    match b {
        b'0'..=b'9' => Some(b - b'0'),
        b'a'..=b'f' => Some(b - b'a' + 10),
        b'A'..=b'F' => Some(b - b'A' + 10),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::crypto::aes::aes_wrap;
    use crate::ie::{WPA_CIPHER_AES_128_CMAC, WPA_CIPHER_CCMP, WPA_KEY_MGMT_PSK};

    const BSSID: [u8; 6] = [0x00, 0x11, 0x22, 0x33, 0x44, 0x55];
    const STA: [u8; 6] = [0x00, 0xaa, 0xbb, 0xcc, 0xdd, 0xee];

    const AP_RSN_CCMP_IE: [u8; 22] = [
        0x30, 0x14, 0x01, 0x00, 0x00, 0x0f, 0xac, 0x04, 0x01, 0x00, 0x00, 0x0f, 0xac, 0x04, 0x01,
        0x00, 0x00, 0x0f, 0xac, 0x02, 0x00, 0x00,
    ];

    fn test_passphrase() -> &'static [u8] {
        #[cfg(miri)]
        {
            b"0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
        }
        #[cfg(not(miri))]
        {
            b"12345678"
        }
    }

    fn setup_sm() -> WpaStaStateMachine {
        let mut sm = WpaStaStateMachine::new();
        sm.start_connect(
            STA,
            BSSID,
            b"test_ssid",
            test_passphrase(),
            &AP_RSN_CCMP_IE,
            WPA_PROTO_RSN,
            WPA_CIPHER_CCMP,
            WPA_CIPHER_CCMP,
            WPA_KEY_MGMT_PSK,
            0,
            0,
        )
        .expect("start_connect should succeed");
        sm
    }

    fn make_frame_with_kd(
        d_type: u8,
        ki: u16,
        rep: [u8; 8],
        nonce: [u8; 32],
        iv: [u8; 16],
        kd: &[u8],
        kck: Option<&[u8; 16]>,
    ) -> ([u8; 256], usize) {
        let total = 99 + kd.len();
        let mut f = [0u8; 256];
        f[0] = 2;
        f[1] = 3;
        f[2..4].copy_from_slice(&((total - 4) as u16).to_be_bytes());
        f[4] = d_type;
        f[5..7].copy_from_slice(&ki.to_be_bytes());
        f[9..17].copy_from_slice(&rep);
        f[17..49].copy_from_slice(&nonce);
        f[49..65].copy_from_slice(&iv);
        f[97..99].copy_from_slice(&(kd.len() as u16).to_be_bytes());
        f[99..total].copy_from_slice(kd);
        if let Some(k) = kck {
            let mic = compute_mic(ki & 0x07, k, &f[..total]).expect("compute_mic should succeed");
            f[81..97].copy_from_slice(&mic);
        }
        (f, total)
    }

    fn make_rsn_m3(
        rep: [u8; 8],
        anonce: [u8; 32],
        ap_ie: &[u8],
        gtk: &[u8; 16],
        gtk_id: u8,
        ptk: &Ptk,
    ) -> ([u8; 256], usize) {
        let mut plain = [0u8; 48];
        plain[..ap_ie.len()].copy_from_slice(ap_ie);
        let off = ap_ie.len();
        plain[off] = 0xDD;
        plain[off + 1] = 22;
        plain[off + 2..off + 5].copy_from_slice(&[0x00, 0x0F, 0xAC]);
        plain[off + 5] = 0x01;
        plain[off + 6] = gtk_id & 0x03;
        plain[off + 7] = 0x00;
        plain[off + 8..off + 24].copy_from_slice(gtk);
        plain[off + 24] = 0xDD;
        let mut wrapped = [0u8; 56];
        aes_wrap(&ptk.kek, &plain, &mut wrapped).expect("aes_wrap M3");
        make_frame_with_kd(2, 0x13ca, rep, anonce, [0u8; 16], &wrapped, Some(&ptk.kck))
    }

    fn make_rsn_gm1(rep: [u8; 8], gtk: &[u8; 16], gtk_id: u8, ptk: &Ptk) -> ([u8; 256], usize) {
        let mut plain = [0u8; 24];
        plain[0] = 0xDD;
        plain[1] = 22;
        plain[2..5].copy_from_slice(&[0x00, 0x0F, 0xAC]);
        plain[5] = 0x01;
        plain[6] = gtk_id & 0x03;
        plain[7] = 0x00;
        plain[8..24].copy_from_slice(gtk);
        let mut wrapped = [0u8; 32];
        aes_wrap(&ptk.kek, &plain, &mut wrapped).expect("aes_wrap GM1");
        make_frame_with_kd(
            2,
            0x1382,
            rep,
            [0u8; 32],
            [0u8; 16],
            &wrapped,
            Some(&ptk.kck),
        )
    }

    #[test]
    fn test_full_m1_m4_handshake() {
        let mut sm = setup_sm();
        assert_eq!(sm.state, HandshakeState::Associating);
        let anonce = [0x11u8; 32];
        let (m1, m1_len) = make_frame_with_kd(
            2,
            0x008a,
            [0, 0, 0, 0, 0, 0, 0, 1],
            anonce,
            [0; 16],
            &[],
            None,
        );
        assert!(matches!(
            sm.on_rx_eapol(&BSSID, &m1[..m1_len], &[0x22u8; 32])
                .expect("M1 rx should succeed"),
            RxEapolAction::SendFrame(_)
        ));
        assert_eq!(sm.state, HandshakeState::FourWayM1Done);
        let gtk = [0x99u8; 16];
        let (m3, m3_len) = make_rsn_m3(
            [0, 0, 0, 0, 0, 0, 0, 2],
            anonce,
            &AP_RSN_CCMP_IE,
            &gtk,
            1,
            &sm.tptk,
        );
        assert!(matches!(
            sm.on_rx_eapol(&BSSID, &m3[..m3_len], &[0x33u8; 32])
                .expect("M3 rx should succeed"),
            RxEapolAction::SendFrame(_)
        ));
        assert_eq!(sm.state, HandshakeState::FourWayM3PendingTx);
        let tx = sm.on_eapol_tx_complete(false);
        assert!(matches!(
            tx,
            TxDoneAction::InstallAndComplete(p) if p.install_ptk && p.gtk_len == 16
        ));
        assert_eq!(sm.state, HandshakeState::Completed);
    }

    #[test]
    fn test_decode_hex_pmk_writes_in_place_and_zeroizes_on_error() {
        let mut out = [0xAAu8; 32];
        let valid_hex = b"0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";
        assert!(decode_hex_pmk(valid_hex, &mut out));
        assert_eq!(out[0], 0x01);
        assert_eq!(out[31], 0xef);

        // Invalid hex digit halfway through MUST return false and zeroize `out`.
        let mut bad_hex = *valid_hex;
        bad_hex[40] = b'Z';
        assert!(!decode_hex_pmk(&bad_hex, &mut out));
        assert_eq!(out, [0u8; 32]);
    }

    #[test]
    fn test_krack_m3_and_group_m1_reinstallation_blocked() {
        let mut sm = setup_sm();
        let anonce = [0x11u8; 32];
        let (m1, m1_len) = make_frame_with_kd(
            2,
            0x008a,
            [0, 0, 0, 0, 0, 0, 0, 1],
            anonce,
            [0; 16],
            &[],
            None,
        );
        sm.on_rx_eapol(&BSSID, &m1[..m1_len], &[0x22u8; 32])
            .expect("M1");
        let gtk1 = [0xAAu8; 16];
        let ptk_copy = sm.tptk.clone();
        let (m3, m3_len) = make_rsn_m3(
            [0, 0, 0, 0, 0, 0, 0, 2],
            anonce,
            &AP_RSN_CCMP_IE,
            &gtk1,
            1,
            &ptk_copy,
        );
        sm.on_rx_eapol(&BSSID, &m3[..m3_len], &[0x33u8; 32])
            .expect("M3");
        let tx = sm.on_eapol_tx_complete(false);
        assert!(matches!(tx, TxDoneAction::InstallAndComplete(_)));
        assert_eq!(sm.state, HandshakeState::Completed);

        // KRACK CVE-2017-13077: Retransmitted M3 with incremented replay
        // counter in Completed MUST be rejected.
        let (m3_re, m3_re_len) = make_rsn_m3(
            [0, 0, 0, 0, 0, 0, 0, 3],
            anonce,
            &AP_RSN_CCMP_IE,
            &gtk1,
            1,
            &ptk_copy,
        );
        assert_eq!(
            sm.on_rx_eapol(&BSSID, &m3_re[..m3_re_len], &[0x44u8; 32]),
            Err(HandshakeError::InvalidKeyInfo)
        );

        // KRACK CVE-2017-13078: Group M1 retransmitting the same installed GTK
        // must NOT reinstall it.
        let (gm1_same, gm1_same_len) = make_rsn_gm1([0, 0, 0, 0, 0, 0, 0, 4], &gtk1, 1, &ptk_copy);
        assert!(matches!(
            sm.on_rx_eapol(&BSSID, &gm1_same[..gm1_same_len], &[0; 32])
                .expect("GM1 same GTK sends GM2"),
            RxEapolAction::SendFrame(_)
        ));
        assert_eq!(sm.on_eapol_tx_complete(false), TxDoneAction::None);

        // A genuinely new GTK MUST be installed.
        let gtk2 = [0xBBu8; 16];
        let (gm1_new, gm1_new_len) = make_rsn_gm1([0, 0, 0, 0, 0, 0, 0, 5], &gtk2, 2, &ptk_copy);
        assert!(matches!(
            sm.on_rx_eapol(&BSSID, &gm1_new[..gm1_new_len], &[0; 32])
                .expect("GM1 new GTK"),
            RxEapolAction::SendFrame(_)
        ));
        let tx_gtk = sm.on_eapol_tx_complete(false);
        assert!(matches!(
            tx_gtk,
            TxDoneAction::InstallAndComplete(p)
                if !p.install_ptk && p.gtk_len == 16 && p.gtk_idx == 2
        ));
    }

    #[test]
    fn test_unauthenticated_m1_cannot_poison_replay_or_downgrade_completed() {
        // EAPOL in Disconnected must be rejected.
        let mut disconn = WpaStaStateMachine::new();
        let (m1_disc, m1_disc_len) = make_frame_with_kd(
            2,
            0x008a,
            [0, 0, 0, 0, 0, 0, 0, 1],
            [0x11; 32],
            [0; 16],
            &[],
            None,
        );
        assert_eq!(
            disconn.on_rx_eapol(&BSSID, &m1_disc[..m1_disc_len], &[0x22; 32]),
            Err(HandshakeError::InvalidKeyInfo)
        );

        let mut sm = setup_sm();
        // Spoofed M1 with max replay counter [0xFF; 8] must NOT poison replay_counter.
        let (spoof_m1, spoof_len) =
            make_frame_with_kd(2, 0x008a, [0xFF; 8], [0x55; 32], [0; 16], &[], None);
        sm.on_rx_eapol(&BSSID, &spoof_m1[..spoof_len], &[0x22; 32])
            .expect("M1 processed");
        assert!(!sm.replay_set);

        // Legitimate M1 (rep=1) and M3 (rep=2) still succeed!
        let anonce = [0x11u8; 32];
        let (m1, m1_len) = make_frame_with_kd(
            2,
            0x008a,
            [0, 0, 0, 0, 0, 0, 0, 1],
            anonce,
            [0; 16],
            &[],
            None,
        );
        sm.on_rx_eapol(&BSSID, &m1[..m1_len], &[0x22; 32])
            .expect("legit M1 after spoof");
        let ptk_copy = sm.tptk.clone();
        let (m3, m3_len) = make_rsn_m3(
            [0, 0, 0, 0, 0, 0, 0, 2],
            anonce,
            &AP_RSN_CCMP_IE,
            &[0x77; 16],
            1,
            &ptk_copy,
        );
        sm.on_rx_eapol(&BSSID, &m3[..m3_len], &[0x33; 32])
            .expect("legit M3");
        sm.on_eapol_tx_complete(false);
        assert_eq!(sm.state, HandshakeState::Completed);

        // Spoofed M1 in Completed must NOT downgrade state or poison replay.
        let (spoof2, spoof2_len) =
            make_frame_with_kd(2, 0x008a, [0xFF; 8], [0x66; 32], [0; 16], &[], None);
        sm.on_rx_eapol(&BSSID, &spoof2[..spoof2_len], &[0x88; 32])
            .expect("M1 in completed");
        assert_eq!(sm.state, HandshakeState::Completed);
        assert!(!sm.in_4way_handshake());
        assert_eq!(sm.replay_counter, [0, 0, 0, 0, 0, 0, 0, 2]);
    }

    #[test]
    fn test_replay_rollback_rejected_after_authenticated_m3() {
        let mut sm = setup_sm();
        let an = [0x11u8; 32];
        let (m1, m1_len) =
            make_frame_with_kd(2, 0x008a, [0, 0, 0, 0, 0, 0, 0, 5], an, [0; 16], &[], None);
        sm.on_rx_eapol(&BSSID, &m1[..m1_len], &[0x22u8; 32])
            .expect("M1");
        let ptk_copy = sm.tptk.clone();
        let (m3, m3_len) = make_rsn_m3(
            [0, 0, 0, 0, 0, 0, 0, 6],
            an,
            &AP_RSN_CCMP_IE,
            &[0x44; 16],
            1,
            &ptk_copy,
        );
        sm.on_rx_eapol(&BSSID, &m3[..m3_len], &[0x33u8; 32])
            .expect("M3");
        let (old, old_len) =
            make_frame_with_kd(2, 0x008a, [0, 0, 0, 0, 0, 0, 0, 5], an, [0; 16], &[], None);
        assert_eq!(
            sm.on_rx_eapol(&BSSID, &old[..old_len], &[0x33u8; 32])
                .expect_err("old replay frame should fail"),
            HandshakeError::ReplayRollback
        );
    }

    #[test]
    fn test_kver_dtype_downgrade_and_plaintext_rsn_m3_and_empty_ap_ie_rejected() {
        // Empty AP IE must be rejected in start_connect (C2-016).
        let mut empty_ie_sm = WpaStaStateMachine::new();
        assert_eq!(
            empty_ie_sm.start_connect(
                STA,
                BSSID,
                b"test_ssid",
                test_passphrase(),
                &[],
                WPA_PROTO_RSN,
                WPA_CIPHER_CCMP,
                WPA_CIPHER_CCMP,
                WPA_KEY_MGMT_PSK,
                0,
                0,
            ),
            Err(HandshakeError::BadParam)
        );
        assert!(!empty_ie_sm.has_pmk);

        let mut sm = setup_sm();
        let an = [0x11u8; 32];
        // Downgrade to kver=1 on CCMP/RSN must fail.
        let (bad_kver, len1) =
            make_frame_with_kd(2, 0x0089, [0, 0, 0, 0, 0, 0, 0, 1], an, [0; 16], &[], None);
        assert_eq!(
            sm.on_rx_eapol(&BSSID, &bad_kver[..len1], &[0x22; 32]),
            Err(HandshakeError::InvalidKeyInfo)
        );
        // Wrong d_type (254 WPA1 on RSN) must fail.
        let (bad_dt, len2) = make_frame_with_kd(
            254,
            0x008a,
            [0, 0, 0, 0, 0, 0, 0, 1],
            an,
            [0; 16],
            &[],
            None,
        );
        assert_eq!(
            sm.on_rx_eapol(&BSSID, &bad_dt[..len2], &[0x22; 32]),
            Err(HandshakeError::InvalidFrame)
        );
        // Error/Request/SMK bits must fail.
        let (bad_err, len3) =
            make_frame_with_kd(2, 0x048a, [0, 0, 0, 0, 0, 0, 0, 1], an, [0; 16], &[], None);
        assert_eq!(
            sm.on_rx_eapol(&BSSID, &bad_err[..len3], &[0x22; 32]),
            Err(HandshakeError::InvalidKeyInfo)
        );

        // Legit M1 then plaintext RSN M3 (missing enc/sec) must be rejected.
        let (m1, m1_len) =
            make_frame_with_kd(2, 0x008a, [0, 0, 0, 0, 0, 0, 0, 1], an, [0; 16], &[], None);
        sm.on_rx_eapol(&BSSID, &m1[..m1_len], &[0x22; 32])
            .expect("M1");
        let (plain_m3, pm3_len) = make_frame_with_kd(
            2,
            0x01ca,
            [0, 0, 0, 0, 0, 0, 0, 2],
            an,
            [0; 16],
            &AP_RSN_CCMP_IE,
            Some(&sm.tptk.kck),
        );
        assert_eq!(
            sm.on_rx_eapol(&BSSID, &plain_m3[..pm3_len], &[0x33; 32]),
            Err(HandshakeError::InvalidKeyInfo)
        );
    }

    #[test]
    fn test_m3_dual_key_fallback_preserves_ptk_after_spoofed_m1() {
        let mut sm = setup_sm();
        let an1 = [0x11u8; 32];
        let (m1, m1_len) =
            make_frame_with_kd(2, 0x008a, [0, 0, 0, 0, 0, 0, 0, 1], an1, [0; 16], &[], None);
        sm.on_rx_eapol(&BSSID, &m1[..m1_len], &[0x22; 32])
            .expect("M1");
        let real_ptk = sm.tptk.clone();
        let (m3_1, m3_1_len) = make_rsn_m3(
            [0, 0, 0, 0, 0, 0, 0, 2],
            an1,
            &AP_RSN_CCMP_IE,
            &[0x33; 16],
            1,
            &real_ptk,
        );
        sm.on_rx_eapol(&BSSID, &m3_1[..m3_1_len], &[0x33; 32])
            .expect("M3 #1");
        // M4 TX fails -> state transitions back to FourWayM1Done, ptk is set.
        assert_eq!(sm.on_eapol_tx_complete(true), TxDoneAction::TxFailed);
        assert_eq!(sm.state, HandshakeState::FourWayM1Done);

        // Attacker injects spoofed M1 with another ANonce, setting bogus tptk.
        let (spoof_m1, spoof_len) = make_frame_with_kd(
            2,
            0x008a,
            [0, 0, 0, 0, 0, 0, 0, 3],
            [0xFE; 32],
            [0; 16],
            &[],
            None,
        );
        sm.on_rx_eapol(&BSSID, &spoof_m1[..spoof_len], &[0x44; 32])
            .expect("spoof M1");
        assert!(sm.tptk_set);
        assert_ne!(sm.tptk, real_ptk);

        // AP retransmits M3 authenticated under real_ptk (self.ptk).
        // Must verify with self.ptk.kck, decrypt with self.ptk.kek, and never
        // overwrite self.ptk with bogus self.tptk.
        let (m3_2, m3_2_len) = make_rsn_m3(
            [0, 0, 0, 0, 0, 0, 0, 4],
            an1,
            &AP_RSN_CCMP_IE,
            &[0x33; 16],
            1,
            &real_ptk,
        );
        sm.on_rx_eapol(&BSSID, &m3_2[..m3_2_len], &[0x55; 32])
            .expect("retransmitted M3 verified via self.ptk");
        assert_eq!(sm.ptk, real_ptk);
        let tx = sm.on_eapol_tx_complete(false);
        assert!(matches!(
            tx,
            TxDoneAction::InstallAndComplete(p) if p.install_ptk && p.ptk_key[..16] == real_ptk.tk
        ));
    }

    #[test]
    fn test_wpa1_group_m1_decrypts_and_swaps_tkip_mic() {
        let mut wpa_ie = [0u8; 32];
        let wpa_len = gen_wpa_ie(
            WPA_PROTO_WPA,
            WPA_CIPHER_TKIP,
            WPA_CIPHER_TKIP,
            WPA_KEY_MGMT_PSK,
            0,
            0,
            &mut wpa_ie,
        )
        .expect("gen WPA IE");
        let mut sm = WpaStaStateMachine::new();
        sm.start_connect(
            STA,
            BSSID,
            b"test_ssid",
            test_passphrase(),
            &wpa_ie[..wpa_len],
            WPA_PROTO_WPA,
            WPA_CIPHER_TKIP,
            WPA_CIPHER_TKIP,
            WPA_KEY_MGMT_PSK,
            0,
            0,
        )
        .expect("WPA1 start_connect");

        let anonce = [0x11u8; 32];
        // WPA1 M1 (d_type=254, kver=1, pair=1, ack=1)
        let (m1, m1_len) = make_frame_with_kd(
            254,
            0x0089,
            [0, 0, 0, 0, 0, 0, 0, 1],
            anonce,
            [0; 16],
            &[],
            None,
        );
        sm.on_rx_eapol(&BSSID, &m1[..m1_len], &[0x22; 32])
            .expect("WPA1 M1");
        let ptk_copy = sm.tptk.clone();

        // WPA1 M3 (d_type=254, kver=1, pair=1, install=1, ack=1, mic=1)
        let (m3, m3_len) = make_frame_with_kd(
            254,
            0x01c9,
            [0, 0, 0, 0, 0, 0, 0, 2],
            anonce,
            [0; 16],
            &wpa_ie[..wpa_len],
            Some(&ptk_copy.kck),
        );
        sm.on_rx_eapol(&BSSID, &m3[..m3_len], &[0x33; 32])
            .expect("WPA1 M3");
        sm.on_eapol_tx_complete(false);
        assert_eq!(sm.state, HandshakeState::Completed);

        // WPA1 Group M1: enc=0, kver=1 (RC4 with IV||KEK), key_idx=1 (0x10).
        let mut raw_gtk = [0u8; 32];
        for (i, b) in raw_gtk.iter_mut().enumerate() {
            *b = (i as u8).wrapping_add(0x10);
        }
        let iv = [0xA5u8; 16];
        let mut rc4_k = [0u8; 32];
        rc4_k[..16].copy_from_slice(&iv);
        rc4_k[16..32].copy_from_slice(&ptk_copy.kek);
        let mut enc_gtk = raw_gtk;
        rc4_skip_checked(&rc4_k, 256, &mut enc_gtk).expect("RC4 encrypt GTK");
        assert_ne!(enc_gtk, raw_gtk);

        let (gm1, gm1_len) = make_frame_with_kd(
            254,
            0x0391,
            [0, 0, 0, 0, 0, 0, 0, 3],
            [0; 32],
            iv,
            &enc_gtk,
            Some(&ptk_copy.kck),
        );
        sm.on_rx_eapol(&BSSID, &gm1[..gm1_len], &[0; 32])
            .expect("WPA1 GM1");
        let tx = sm.on_eapol_tx_complete(false);
        match tx {
            TxDoneAction::InstallAndComplete(pi) => {
                assert_eq!(pi.gtk_len, 32);
                assert_eq!(pi.gtk_alg, WIFI_WPA_ALG_TKIP);
                assert_eq!(pi.gtk_idx, 1);
                assert_eq!(&pi.gtk[..16], &raw_gtk[..16]);
                assert_eq!(&pi.gtk[16..24], &raw_gtk[24..32]);
                assert_eq!(&pi.gtk[24..32], &raw_gtk[16..24]);
            }
            other => panic!("expected InstallAndComplete, got {other:?}"),
        }
    }

    #[test]
    fn test_wrong_m3_mic_rejected() {
        let mut sm = setup_sm();
        let an = [0x11u8; 32];
        let (m1, m1_len) =
            make_frame_with_kd(2, 0x008a, [0, 0, 0, 0, 0, 0, 0, 1], an, [0; 16], &[], None);
        sm.on_rx_eapol(&BSSID, &m1[..m1_len], &[0x22u8; 32])
            .expect("M1 rx should succeed");
        let (mut m3, m3_len) = make_rsn_m3(
            [0, 0, 0, 0, 0, 0, 0, 2],
            an,
            &AP_RSN_CCMP_IE,
            &[0x12; 16],
            1,
            &sm.tptk,
        );
        m3[81] ^= 0xff;
        assert_eq!(
            sm.on_rx_eapol(&BSSID, &m3[..m3_len], &[0x33u8; 32])
                .expect_err("M3 with corrupted mic should fail"),
            HandshakeError::MicMismatch
        );
    }

    #[test]
    fn test_tx_failure_blocks_key_install_and_recovers_state() {
        let mut sm = setup_sm();
        let an = [0x11u8; 32];
        let (m1, m1_len) =
            make_frame_with_kd(2, 0x008a, [0, 0, 0, 0, 0, 0, 0, 1], an, [0; 16], &[], None);
        sm.on_rx_eapol(&BSSID, &m1[..m1_len], &[0x22u8; 32])
            .expect("M1 rx should succeed");
        let (m3, m3_len) = make_rsn_m3(
            [0, 0, 0, 0, 0, 0, 0, 2],
            an,
            &AP_RSN_CCMP_IE,
            &[0x55; 16],
            1,
            &sm.tptk,
        );
        sm.on_rx_eapol(&BSSID, &m3[..m3_len], &[0x33u8; 32])
            .expect("M3 rx should succeed");
        assert_eq!(sm.on_eapol_tx_complete(true), TxDoneAction::TxFailed);
        assert!(sm.pending_install.is_none());
        assert_eq!(sm.state, HandshakeState::FourWayM1Done);
        assert_eq!(sm.on_eapol_tx_complete(false), TxDoneAction::None);
    }

    #[test]
    fn test_group_key_rekey() {
        let mut sm = setup_sm();
        let an = [0x11u8; 32];
        let (m1, m1_len) =
            make_frame_with_kd(2, 0x008a, [0, 0, 0, 0, 0, 0, 0, 1], an, [0; 16], &[], None);
        sm.on_rx_eapol(&BSSID, &m1[..m1_len], &[0x22u8; 32])
            .expect("M1 rx should succeed");
        let ptk_copy = sm.tptk.clone();
        let (m3, m3_len) = make_rsn_m3(
            [0, 0, 0, 0, 0, 0, 0, 2],
            an,
            &AP_RSN_CCMP_IE,
            &[0x11; 16],
            1,
            &ptk_copy,
        );
        sm.on_rx_eapol(&BSSID, &m3[..m3_len], &[0x33u8; 32])
            .expect("M3 rx should succeed");
        sm.on_eapol_tx_complete(false);
        let (gm1, gm1_len) = make_rsn_gm1([0, 0, 0, 0, 0, 0, 0, 3], &[0x22; 16], 2, &ptk_copy);
        assert!(matches!(
            sm.on_rx_eapol(&BSSID, &gm1[..gm1_len], &[0u8; 32])
                .expect("GM1 rx should succeed"),
            RxEapolAction::SendFrame(_)
        ));
        assert_eq!(sm.state, HandshakeState::GroupRekeyPendingTx);
        let tx = sm.on_eapol_tx_complete(false);
        assert!(matches!(
            tx,
            TxDoneAction::InstallAndComplete(p) if !p.install_ptk && p.gtk_idx == 2
        ));
        assert_eq!(sm.state, HandshakeState::Completed);
    }

    #[test]
    fn test_start_connect_non_pmf_ccmp_22_byte_rsn_ie() {
        let mut sm = WpaStaStateMachine::new();
        let (ie, len) = sm
            .start_connect(
                STA,
                BSSID,
                b"test_ssid",
                test_passphrase(),
                &AP_RSN_CCMP_IE,
                WPA_PROTO_RSN,
                WPA_CIPHER_CCMP,
                WPA_CIPHER_CCMP,
                WPA_KEY_MGMT_PSK,
                0,
                0,
            )
            .expect("start_connect non-PMF should succeed");

        assert_eq!(len, 22);
        assert_eq!(&ie[..len], &AP_RSN_CCMP_IE);
    }

    #[test]
    fn test_start_connect_pmf_optional_and_required_28_byte_rsn_ie() {
        let mut sm = WpaStaStateMachine::new();
        let (ie_opt, len_opt) = sm
            .start_connect(
                STA,
                BSSID,
                b"test_ssid",
                test_passphrase(),
                &AP_RSN_CCMP_IE,
                WPA_PROTO_RSN,
                WPA_CIPHER_CCMP,
                WPA_CIPHER_CCMP,
                WPA_KEY_MGMT_PSK,
                WPA_CIPHER_AES_128_CMAC,
                0x0080,
            )
            .expect("start_connect PMF optional should succeed");
        assert_eq!(len_opt, 28);
        assert_eq!(&ie_opt[20..22], &[0x80, 0x00]);
        assert_eq!(&ie_opt[22..24], &[0x00, 0x00]);
        assert_eq!(&ie_opt[24..28], &[0x00, 0x0f, 0xac, 0x06]);

        let (ie_req, len_req) = sm
            .start_connect(
                STA,
                BSSID,
                b"test_ssid",
                test_passphrase(),
                &AP_RSN_CCMP_IE,
                WPA_PROTO_RSN,
                WPA_CIPHER_CCMP,
                WPA_CIPHER_CCMP,
                WPA_KEY_MGMT_PSK,
                WPA_CIPHER_AES_128_CMAC,
                0x00c0,
            )
            .expect("start_connect PMF required should succeed");
        assert_eq!(len_req, 28);
        assert_eq!(&ie_req[20..22], &[0xc0, 0x00]);
        assert_eq!(&ie_req[22..24], &[0x00, 0x00]);
        assert_eq!(&ie_req[24..28], &[0x00, 0x0f, 0xac, 0x06]);
    }

    #[test]
    fn test_m1_ver2_emits_m2_ver1_with_valid_mic() {
        let mut sm = setup_sm();
        let anonce = [0x11u8; 32];
        let (m1, m1_len) = make_frame_with_kd(
            2,
            0x008a,
            [0, 0, 0, 0, 0, 0, 0, 1],
            anonce,
            [0; 16],
            &[],
            None,
        );
        assert_eq!(m1[0], 2);
        let action = sm
            .on_rx_eapol(&BSSID, &m1[..m1_len], &[0x22u8; 32])
            .expect("M1 rx should succeed");
        let m2 = match action {
            RxEapolAction::SendFrame(f) => f,
            _ => panic!("expected SendFrame"),
        };
        assert_eq!(m2.data[0], 1);
        let mut m2_mic = [0u8; 16];
        m2_mic.copy_from_slice(&m2.data[81..97]);
        assert_eq!(
            verify_mic(&m2.data[..m2.len], 2, &sm.tptk.kck, &m2_mic),
            Ok(true)
        );
    }

    #[test]
    fn test_populate_gtk_tkip_swaps_mic_halves_ccmp_unchanged_and_redacted_debug() {
        let rsc = [1, 2, 3, 4, 5, 6, 0, 0];

        let mut tkip_gtk = [0u8; 32];
        for (i, b) in tkip_gtk.iter_mut().enumerate() {
            *b = i as u8;
        }
        let pk_tkip = ParsedKeyData {
            gtk: Some(&tkip_gtk),
            gtk_key_id: 1,
            ..ParsedKeyData::default()
        };
        let mut pi_tkip = PendingKeyInstall::empty();
        populate_gtk(
            &mut pi_tkip,
            &pk_tkip,
            &rsc,
            WPA_CIPHER_TKIP,
            0,
            false,
            true,
        )
        .expect("TKIP GTK");
        assert_eq!(pi_tkip.gtk_len, 32);
        assert_eq!(pi_tkip.gtk_alg, 2);
        assert_eq!(pi_tkip.gtk_idx, 1);
        assert_eq!(&pi_tkip.gtk[..16], &tkip_gtk[..16]);
        assert_eq!(&pi_tkip.gtk[16..24], &tkip_gtk[24..32]);
        assert_eq!(&pi_tkip.gtk[24..32], &tkip_gtk[16..24]);

        // Redacted Debug must not expose raw keys.
        let dbg = format!("{pi_tkip:?}");
        assert!(dbg.contains("[REDACTED]"));

        // Mismatched GTK length must be rejected.
        let mut pi_bad = PendingKeyInstall::empty();
        assert_eq!(
            populate_gtk(&mut pi_bad, &pk_tkip, &rsc, WPA_CIPHER_CCMP, 0, false, true),
            Err(HandshakeError::BadParam)
        );

        let ccmp_gtk = [0x55u8; 16];
        let pk_ccmp = ParsedKeyData {
            gtk: Some(&ccmp_gtk),
            gtk_key_id: 2,
            ..ParsedKeyData::default()
        };
        let mut pi_ccmp = PendingKeyInstall::empty();
        populate_gtk(
            &mut pi_ccmp,
            &pk_ccmp,
            &rsc,
            WPA_CIPHER_CCMP,
            0,
            false,
            true,
        )
        .expect("CCMP GTK");
        assert_eq!(pi_ccmp.gtk_len, 16);
        assert_eq!(pi_ccmp.gtk_alg, 3);
        assert_eq!(pi_ccmp.gtk_idx, 2);
        assert_eq!(&pi_ccmp.gtk[..16], &ccmp_gtk);

        // PMF required without IGTK must be rejected.
        let mut pi_pmf = PendingKeyInstall::empty();
        assert_eq!(
            populate_gtk(
                &mut pi_pmf,
                &pk_ccmp,
                &rsc,
                WPA_CIPHER_CCMP,
                WPA_CIPHER_AES_128_CMAC,
                true,
                true,
            ),
            Err(HandshakeError::InvalidFrame)
        );
    }
}
