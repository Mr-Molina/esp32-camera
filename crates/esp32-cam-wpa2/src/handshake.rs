//! Pure-Rust STA WPA2 4-Way & Group Key Handshake State Machine.

use crate::crypto::aes::{aes_unwrap, omac1_aes_128};
use crate::crypto::ccm::rc4_skip;
use crate::crypto::hash::{ct_eq, hmac_md5_vector, hmac_sha1_vector};
use crate::ie::{
    gen_wpa_ie, parse_key_data_kdes, ParsedKeyData, WPA_CIPHER_TKIP, WPA_KEY_MGMT_PSK_SHA256,
    WPA_PROTO_RSN, WPA_PROTO_WPA,
};
use crate::kdf::{pbkdf2_hmac_sha1, wpa_pmk_to_ptk, Ptk};

/// TKIP cipher algorithm identifier for driver installation.
pub const WIFI_WPA_ALG_TKIP: i32 = 2;
/// CCMP (AES-128-CCM) cipher algorithm identifier for driver installation.
pub const WIFI_WPA_ALG_CCMP: i32 = 3;

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
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
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
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TxDoneAction {
    None,
    TxFailed,
    InstallAndComplete(PendingKeyInstall),
}

const EMPTY_PTK: Ptk = Ptk {
    kck: [0u8; 16],
    kek: [0u8; 16],
    tk: [0u8; 16],
    tx_mic: [0u8; 8],
    rx_mic: [0u8; 8],
};

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
    assoc_ie: [u8; 128],
    assoc_ie_len: usize,
    ap_ie: [u8; 128],
    ap_ie_len: usize,
    snonce: [u8; 32],
    snonce_renew: bool,
    anonce: [u8; 32],
    tptk: Ptk,
    tptk_set: bool,
    ptk: Ptk,
    ptk_set: bool,
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
            assoc_ie: [0u8; 128],
            assoc_ie_len: 0,
            ap_ie: [0u8; 128],
            ap_ie_len: 0,
            snonce: [0u8; 32],
            snonce_renew: true,
            anonce: [0u8; 32],
            tptk: EMPTY_PTK,
            tptk_set: false,
            ptk: EMPTY_PTK,
            ptk_set: false,
            replay_counter: [0u8; 8],
            replay_set: false,
            pending_install: None,
        }
    }

    pub fn reset(&mut self) {
        self.state = HandshakeState::Disconnected;
        self.assoc_ie_len = 0;
        self.ap_ie_len = 0;
        self.snonce_renew = true;
        self.tptk_set = false;
        self.ptk_set = false;
        self.replay_set = false;
        self.pending_install = None;
    }

    pub fn set_cached_pmk(&mut self, pmk: &[u8; 32]) {
        self.pmk = *pmk;
        self.has_pmk = true;
    }

    pub fn clear_pmk(&mut self) {
        self.pmk = [0u8; 32];
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
        if let Some(dec) = decode_hex_pmk(passphrase_or_psk) {
            self.pmk = dec;
            self.has_pmk = true;
        } else if (8..=63).contains(&passphrase_or_psk.len()) {
            pbkdf2_hmac_sha1(passphrase_or_psk, ssid, 4096, &mut self.pmk)
                .map_err(|_| HandshakeError::BadParam)?;
            self.has_pmk = true;
        } else if !passphrase_or_psk.is_empty() || !self.has_pmk {
            return Err(HandshakeError::BadParam);
        }
        if ap_ie_bytes.len() > self.ap_ie.len() {
            return Err(HandshakeError::BadParam);
        }
        self.ap_ie[..ap_ie_bytes.len()].copy_from_slice(ap_ie_bytes);
        self.ap_ie_len = ap_ie_bytes.len();
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
        self.replay_set = false;
        Ok((abuf, alen))
    }

    pub fn on_rx_eapol(
        &mut self,
        src: &[u8; 6],
        eapol: &[u8],
        snonce_rand: &[u8; 32],
    ) -> Result<RxEapolAction, HandshakeError> {
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
        if frame[4] != 2 && frame[4] != 254 {
            return Err(HandshakeError::InvalidFrame);
        }
        let d_type = frame[4];
        let ki = u16::from_be_bytes([frame[5], frame[6]]);
        let (kver, pair, ack, mic, sec, enc) = (
            ki & 0x07,
            (ki & 0x08) != 0,
            (ki & 0x80) != 0,
            (ki & 0x100) != 0,
            (ki & 0x200) != 0,
            (ki & 0x1000) != 0,
        );
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
            if !self.has_pmk {
                return Err(HandshakeError::NoPmk);
            }
            if self.snonce_renew {
                self.snonce = *snonce_rand;
                self.snonce_renew = false;
            }
            self.anonce = nonce;
            let sha = (self.key_mgmt & WPA_KEY_MGMT_PSK_SHA256) != 0 || kver == 3;
            self.tptk = wpa_pmk_to_ptk(
                &self.pmk,
                b"Pairwise key expansion",
                &self.own_addr,
                &self.bssid,
                &self.snonce,
                &self.anonce,
                sha,
            );
            self.tptk_set = true;
            self.replay_counter = rep;
            self.replay_set = true;
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
            self.state = HandshakeState::FourWayM1Done;
            Ok(RxEapolAction::SendFrame(m2))
        } else if pair && ack && mic {
            if !self.tptk_set && !self.ptk_set {
                return Err(HandshakeError::BadParam);
            }
            let valid = (self.tptk_set && verify_mic(frame, kver, &self.tptk.kck, &rx_mic)?)
                || (self.ptk_set && verify_mic(frame, kver, &self.ptk.kck, &rx_mic)?);
            if !valid {
                return Err(HandshakeError::MicMismatch);
            }
            if !ct_eq(&nonce, &self.anonce) {
                return Err(HandshakeError::BadParam);
            }
            let mut pbuf = [0u8; 256];
            let pslice = if enc && !kd.is_empty() {
                let plen = decrypt_kd(kver, &self.tptk.kek, &iv, kd, &mut pbuf)?;
                &pbuf[..plen]
            } else {
                kd
            };
            let pk = parse_key_data_kdes(pslice);
            if self.ap_ie_len > 0 {
                let exp = &self.ap_ie[..self.ap_ie_len];
                let ok = match (pk.rsn_ie, pk.wpa_ie) {
                    (Some(r), _) if r == exp => true,
                    (_, Some(w)) if w == exp => true,
                    _ => false,
                };
                if !ok {
                    return Err(HandshakeError::IeMismatch);
                }
            }
            self.ptk = self.tptk;
            self.ptk_set = true;
            let mut pi = PendingKeyInstall::empty();
            pi.install_ptk = true;
            pi.notify_auth_done = true;
            if self.pairwise_supp == WPA_CIPHER_TKIP {
                pi.ptk_alg = WIFI_WPA_ALG_TKIP;
                pi.ptk_key_len = 32;
                pi.ptk_key[..16].copy_from_slice(&self.ptk.tk);
                pi.ptk_key[16..24].copy_from_slice(&self.ptk.tx_mic);
                pi.ptk_key[24..32].copy_from_slice(&self.ptk.rx_mic);
            } else {
                pi.ptk_alg = WIFI_WPA_ALG_CCMP;
                pi.ptk_key_len = 16;
                pi.ptk_key[..16].copy_from_slice(&self.ptk.tk);
            }
            populate_gtk(&mut pi, &pk, &rsc);
            self.pending_install = Some(pi);
            self.replay_counter = rep;
            self.replay_set = true;
            let m4_ki = (kver & 0x07) | 0x0108 | if sec { 0x0200 } else { 0 };
            let m4 = build_eapol(d_type, m4_ki, 0, rep, [0u8; 32], &[], &self.ptk.kck)?;
            self.state = HandshakeState::FourWayM3PendingTx;
            Ok(RxEapolAction::SendFrame(m4))
        } else if !pair && ack && mic && sec {
            if !self.ptk_set || !verify_mic(frame, kver, &self.ptk.kck, &rx_mic)? {
                return Err(if !self.ptk_set {
                    HandshakeError::BadParam
                } else {
                    HandshakeError::MicMismatch
                });
            }
            let mut pbuf = [0u8; 256];
            let need_dec = (enc || self.proto == WPA_PROTO_RSN) && !kd.is_empty();
            let pslice = if need_dec {
                let plen = decrypt_kd(kver, &self.ptk.kek, &iv, kd, &mut pbuf)?;
                &pbuf[..plen]
            } else {
                kd
            };
            let mut pi = PendingKeyInstall::empty();
            pi.notify_auth_done = true;
            if self.proto == WPA_PROTO_RSN {
                populate_gtk(&mut pi, &parse_key_data_kdes(pslice), &rsc);
            } else {
                let l = pslice.len().min(32);
                pi.gtk[..l].copy_from_slice(&pslice[..l]);
                pi.gtk_len = l;
                pi.gtk_idx = ((ki & 0x30) >> 4) as u8;
                pi.gtk_alg = if l == 32 { 2 } else { 3 };
                pi.gtk_rsc.copy_from_slice(&rsc[..6]);
            }
            self.pending_install = Some(pi);
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
            self.state = HandshakeState::GroupRekeyPendingTx;
            Ok(RxEapolAction::SendFrame(gm2))
        } else {
            Err(HandshakeError::InvalidKeyInfo)
        }
    }

    pub fn on_eapol_tx_complete(&mut self, tx_failure: bool) -> TxDoneAction {
        if tx_failure {
            self.pending_install = None;
            TxDoneAction::TxFailed
        } else if let Some(pi) = self.pending_install.take() {
            self.state = HandshakeState::Completed;
            self.snonce_renew = true;
            TxDoneAction::InstallAndComplete(pi)
        } else {
            TxDoneAction::None
        }
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
            let mut mic = [0u8; 16];
            mic.copy_from_slice(&hmac_sha1_vector(kck, &[frame])[..16]);
            Ok(mic)
        }
        3 => Ok(omac1_aes_128(kck, frame)),
        _ => Err(HandshakeError::InvalidKeyInfo),
    }
}

fn verify_mic(f: &[u8], kv: u16, kck: &[u8; 16], rx: &[u8; 16]) -> Result<bool, HandshakeError> {
    let mut z = [0u8; 384];
    if f.len() > z.len() {
        return Err(HandshakeError::InvalidFrame);
    }
    z[..f.len()].copy_from_slice(f);
    z[81..97].fill(0);
    Ok(ct_eq(&compute_mic(kv, kck, &z[..f.len()])?, rx))
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
    if kv >= 2 {
        if kd.len() < 8 || kd.len() - 8 > out.len() {
            return Err(HandshakeError::DecryptFailed);
        }
        let l = kd.len() - 8;
        aes_unwrap(kek, kd, &mut out[..l]).map_err(|_| HandshakeError::DecryptFailed)?;
        Ok(l)
    } else if kv == 1 {
        if kd.len() > out.len() {
            return Err(HandshakeError::DecryptFailed);
        }
        let mut rc4_k = [0u8; 32];
        rc4_k[..16].copy_from_slice(iv);
        rc4_k[16..32].copy_from_slice(kek);
        out[..kd.len()].copy_from_slice(kd);
        rc4_skip(&rc4_k, 256, &mut out[..kd.len()]);
        Ok(kd.len())
    } else {
        Err(HandshakeError::InvalidKeyInfo)
    }
}

fn populate_gtk(pi: &mut PendingKeyInstall, pk: &ParsedKeyData, rsc: &[u8; 8]) {
    if let Some(gtk) = pk.gtk {
        let l = gtk.len().min(32);
        pi.gtk[..l].copy_from_slice(&gtk[..l]);
        if l == 32 {
            let mut mic_tx = [0u8; 8];
            mic_tx.copy_from_slice(&pi.gtk[16..24]);
            pi.gtk.copy_within(24..32, 16);
            pi.gtk[24..32].copy_from_slice(&mic_tx);
        }
        pi.gtk_len = l;
        pi.gtk_idx = pk.gtk_key_id;
        pi.gtk_alg = if l == 32 { 2 } else { 3 };
        pi.gtk_rsc.copy_from_slice(&rsc[..6]);
    }
    if let Some(igtk) = pk.igtk {
        let l = igtk.len().min(32);
        pi.igtk[..l].copy_from_slice(&igtk[..l]);
        pi.igtk_idx = pk.igtk_key_id;
        pi.igtk_pn = pk.igtk_ipn;
        pi.has_igtk = true;
    }
}

fn decode_hex_pmk(hex: &[u8]) -> Option<[u8; 32]> {
    if hex.len() != 64 {
        return None;
    }
    let mut out = [0u8; 32];
    for (i, byte) in out.iter_mut().enumerate() {
        let hi = hex_val(hex[i * 2])?;
        let lo = hex_val(hex[i * 2 + 1])?;
        *byte = (hi << 4) | lo;
    }
    Some(out)
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
    use crate::ie::{WPA_CIPHER_AES_128_CMAC, WPA_CIPHER_CCMP, WPA_KEY_MGMT_PSK};

    const BSSID: [u8; 6] = [0x00, 0x11, 0x22, 0x33, 0x44, 0x55];
    const STA: [u8; 6] = [0x00, 0xaa, 0xbb, 0xcc, 0xdd, 0xee];

    fn setup_sm() -> WpaStaStateMachine {
        let mut sm = WpaStaStateMachine::new();
        #[cfg(miri)]
        let passphrase = b"0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";
        #[cfg(not(miri))]
        let passphrase = b"12345678";

        sm.start_connect(
            STA,
            BSSID,
            b"test_ssid",
            passphrase,
            &[],
            WPA_PROTO_RSN,
            WPA_CIPHER_TKIP,
            WPA_CIPHER_TKIP,
            WPA_KEY_MGMT_PSK,
            0,
            0,
        )
        .expect("start_connect should succeed");
        sm
    }

    fn make_frame(ki: u16, rep: [u8; 8], nonce: [u8; 32], kck: Option<&[u8; 16]>) -> [u8; 99] {
        let mut f = [0u8; 99];
        f[0] = 2;
        f[1] = 3;
        f[3] = 95;
        f[4] = 2;
        let ki_b = ki.to_be_bytes();
        f[5..7].copy_from_slice(&ki_b);
        f[9..17].copy_from_slice(&rep);
        f[17..49].copy_from_slice(&nonce);
        if let Some(k) = kck {
            let mic = compute_mic(ki & 0x07, k, &f).expect("compute_mic should succeed");
            f[81..97].copy_from_slice(&mic);
        }
        f
    }

    #[test]
    fn test_full_m1_m4_handshake() {
        let mut sm = setup_sm();
        assert_eq!(sm.state, HandshakeState::Associating);
        let anonce = [0x11u8; 32];
        let m1 = make_frame(0x008a, [0, 0, 0, 0, 0, 0, 0, 1], anonce, None);
        assert!(matches!(
            sm.on_rx_eapol(&BSSID, &m1, &[0x22u8; 32])
                .expect("M1 rx should succeed"),
            RxEapolAction::SendFrame(_)
        ));
        assert_eq!(sm.state, HandshakeState::FourWayM1Done);
        let m3 = make_frame(0x01ca, [0, 0, 0, 0, 0, 0, 0, 2], anonce, Some(&sm.tptk.kck));
        assert!(matches!(
            sm.on_rx_eapol(&BSSID, &m3, &[0x33u8; 32])
                .expect("M3 rx should succeed"),
            RxEapolAction::SendFrame(_)
        ));
        assert_eq!(sm.state, HandshakeState::FourWayM3PendingTx);
        let tx = sm.on_eapol_tx_complete(false);
        assert!(matches!(
            tx,
            TxDoneAction::InstallAndComplete(p) if p.install_ptk
        ));
        assert_eq!(sm.state, HandshakeState::Completed);
    }

    #[test]
    fn test_wrong_m3_mic_rejected() {
        let mut sm = setup_sm();
        let an = [0x11u8; 32];
        let m1 = make_frame(0x008a, [0, 0, 0, 0, 0, 0, 0, 1], an, None);
        sm.on_rx_eapol(&BSSID, &m1, &[0x22u8; 32])
            .expect("M1 rx should succeed");
        let mut m3 = make_frame(0x01ca, [0, 0, 0, 0, 0, 0, 0, 2], an, Some(&sm.tptk.kck));
        m3[81] ^= 0xff;
        assert_eq!(
            sm.on_rx_eapol(&BSSID, &m3, &[0x33u8; 32])
                .expect_err("M3 with corrupted mic should fail"),
            HandshakeError::MicMismatch
        );
    }

    #[test]
    fn test_replay_rollback_rejected() {
        let mut sm = setup_sm();
        let an = [0x11u8; 32];
        let m1 = make_frame(0x008a, [0, 0, 0, 0, 0, 0, 0, 5], an, None);
        sm.on_rx_eapol(&BSSID, &m1, &[0x22u8; 32])
            .expect("M1 rx should succeed");
        let old = make_frame(0x008a, [0, 0, 0, 0, 0, 0, 0, 4], an, None);
        assert_eq!(
            sm.on_rx_eapol(&BSSID, &old, &[0x33u8; 32])
                .expect_err("old replay frame should fail"),
            HandshakeError::ReplayRollback
        );
    }

    #[test]
    fn test_tx_failure_blocks_key_install() {
        let mut sm = setup_sm();
        let an = [0x11u8; 32];
        let m1 = make_frame(0x008a, [0, 0, 0, 0, 0, 0, 0, 1], an, None);
        sm.on_rx_eapol(&BSSID, &m1, &[0x22u8; 32])
            .expect("M1 rx should succeed");
        let m3 = make_frame(0x01ca, [0, 0, 0, 0, 0, 0, 0, 2], an, Some(&sm.tptk.kck));
        sm.on_rx_eapol(&BSSID, &m3, &[0x33u8; 32])
            .expect("M3 rx should succeed");
        assert_eq!(sm.on_eapol_tx_complete(true), TxDoneAction::TxFailed);
        assert!(sm.pending_install.is_none());
        assert_eq!(sm.on_eapol_tx_complete(false), TxDoneAction::None);
    }

    #[test]
    fn test_group_key_rekey() {
        let mut sm = setup_sm();
        let an = [0x11u8; 32];
        let m1 = make_frame(0x008a, [0, 0, 0, 0, 0, 0, 0, 1], an, None);
        sm.on_rx_eapol(&BSSID, &m1, &[0x22u8; 32])
            .expect("M1 rx should succeed");
        let m3 = make_frame(0x01ca, [0, 0, 0, 0, 0, 0, 0, 2], an, Some(&sm.tptk.kck));
        sm.on_rx_eapol(&BSSID, &m3, &[0x33u8; 32])
            .expect("M3 rx should succeed");
        sm.on_eapol_tx_complete(false);
        let gm1 = make_frame(
            0x0382,
            [0, 0, 0, 0, 0, 0, 0, 3],
            [0u8; 32],
            Some(&sm.ptk.kck),
        );
        assert!(matches!(
            sm.on_rx_eapol(&BSSID, &gm1, &[0u8; 32])
                .expect("GM1 rx should succeed"),
            RxEapolAction::SendFrame(_)
        ));
        assert_eq!(sm.state, HandshakeState::GroupRekeyPendingTx);
        let tx = sm.on_eapol_tx_complete(false);
        assert!(matches!(
            tx,
            TxDoneAction::InstallAndComplete(p) if !p.install_ptk
        ));
        assert_eq!(sm.state, HandshakeState::Completed);
    }

    #[test]
    fn test_start_connect_non_pmf_ccmp_22_byte_rsn_ie() {
        let mut sm = WpaStaStateMachine::new();
        #[cfg(miri)]
        let passphrase = b"0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";
        #[cfg(not(miri))]
        let passphrase = b"12345678";

        let (ie, len) = sm
            .start_connect(
                STA,
                BSSID,
                b"test_ssid",
                passphrase,
                &[],
                WPA_PROTO_RSN,
                WPA_CIPHER_CCMP,
                WPA_CIPHER_CCMP,
                WPA_KEY_MGMT_PSK,
                0,
                0,
            )
            .expect("start_connect non-PMF should succeed");

        let expected = [
            0x30, 0x14, 0x01, 0x00, 0x00, 0x0f, 0xac, 0x04, 0x01, 0x00, 0x00, 0x0f, 0xac, 0x04,
            0x01, 0x00, 0x00, 0x0f, 0xac, 0x02, 0x00, 0x00,
        ];
        assert_eq!(len, 22);
        assert_eq!(&ie[..len], &expected);
    }

    #[test]
    fn test_start_connect_pmf_optional_and_required_28_byte_rsn_ie() {
        let mut sm = WpaStaStateMachine::new();
        #[cfg(miri)]
        let passphrase = b"0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";
        #[cfg(not(miri))]
        let passphrase = b"12345678";

        let (ie_opt, len_opt) = sm
            .start_connect(
                STA,
                BSSID,
                b"test_ssid",
                passphrase,
                &[],
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
                passphrase,
                &[],
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
        let m1 = make_frame(0x008a, [0, 0, 0, 0, 0, 0, 0, 1], anonce, None);
        assert_eq!(m1[0], 2);
        let action = sm
            .on_rx_eapol(&BSSID, &m1, &[0x22u8; 32])
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
    fn test_populate_gtk_tkip_swaps_mic_halves_ccmp_unchanged() {
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
        populate_gtk(&mut pi_tkip, &pk_tkip, &rsc);
        assert_eq!(pi_tkip.gtk_len, 32);
        assert_eq!(pi_tkip.gtk_alg, 2);
        assert_eq!(pi_tkip.gtk_idx, 1);
        assert_eq!(&pi_tkip.gtk[..16], &tkip_gtk[..16]);
        assert_eq!(&pi_tkip.gtk[16..24], &tkip_gtk[24..32]);
        assert_eq!(&pi_tkip.gtk[24..32], &tkip_gtk[16..24]);

        let ccmp_gtk = [0x55u8; 16];
        let pk_ccmp = ParsedKeyData {
            gtk: Some(&ccmp_gtk),
            gtk_key_id: 2,
            ..ParsedKeyData::default()
        };
        let mut pi_ccmp = PendingKeyInstall::empty();
        populate_gtk(&mut pi_ccmp, &pk_ccmp, &rsc);
        assert_eq!(pi_ccmp.gtk_len, 16);
        assert_eq!(pi_ccmp.gtk_alg, 3);
        assert_eq!(pi_ccmp.gtk_idx, 2);
        assert_eq!(&pi_ccmp.gtk[..16], &ccmp_gtk);
    }
}
