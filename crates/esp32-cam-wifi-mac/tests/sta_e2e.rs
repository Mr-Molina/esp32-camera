//! End-to-end integration tests for `esp32-cam-wifi-mac`.

use esp32_cam_wifi_mac::frames_80211::parse_80211_header;
use esp32_cam_wifi_mac::frames_80211::FrameType;
use esp32_cam_wifi_mac::phy_bridge::host_mock_reset;
use esp32_cam_wifi_mac::phy_bridge::host_mock_snapshot;
use esp32_cam_wifi_mac::Esp32StaWifiDriver;
use esp32_cam_wifi_mac::MockWifiMmioBus;
use esp32_cam_wifi_mac::StaConnectConfig;
use esp32_cam_wifi_mac::StaState;
use esp32_cam_wpa2::crypto::aes::aes_wrap;
use esp32_cam_wpa2::crypto::ccm::ccmp_decrypt_mpdu;
use esp32_cam_wpa2::crypto::ccm::ccmp_encrypt_mpdu;
use esp32_cam_wpa2::crypto::hash::hmac_sha1_vector;
use esp32_cam_wpa2::ie::gen_wpa_ie;
use esp32_cam_wpa2::ie::WPA_CIPHER_CCMP;
use esp32_cam_wpa2::ie::WPA_KEY_MGMT_PSK;
use esp32_cam_wpa2::ie::WPA_PROTO_RSN;
#[cfg(not(miri))]
use esp32_cam_wpa2::kdf::pbkdf2_hmac_sha1;
use esp32_cam_wpa2::kdf::wpa_pmk_to_ptk;
use esp32_cam_wpa2::kdf::Ptk;
use smoltcp::phy::Device;
use smoltcp::phy::RxToken;
use smoltcp::phy::TxToken;
use smoltcp::time::Instant;

const STA_MAC: [u8; 6] = [0xD4, 0xD4, 0xDA, 0x5B, 0xD4, 0xEC];
const AP_WEAK_BSSID: [u8; 6] = [0x02, 0x00, 0x00, 0x00, 0x01, 0x01];
const AP_BEST_BSSID: [u8; 6] = [0x02, 0x11, 0x22, 0x33, 0x44, 0x55];
const SSID: &[u8] = b"KlipperCam";

#[cfg(miri)]
fn test_pmk_and_cred() -> ([u8; 32], &'static [u8]) {
    let mut pmk = [0u8; 32];
    let hex = b"0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";
    let mut idx = 0usize;
    while idx < 32 {
        let hi = match hex[idx * 2] {
            b'0'..=b'9' => hex[idx * 2] - b'0',
            _ => hex[idx * 2] - b'a' + 10,
        };
        let lo = match hex[idx * 2 + 1] {
            b'0'..=b'9' => hex[idx * 2 + 1] - b'0',
            _ => hex[idx * 2 + 1] - b'a' + 10,
        };
        pmk[idx] = (hi << 4) | lo;
        idx += 1;
    }
    (pmk, hex)
}

#[cfg(not(miri))]
fn test_pmk_and_cred() -> ([u8; 32], &'static [u8]) {
    let pass = b"supersecretpsk";
    let mut pmk = [0u8; 32];
    pbkdf2_hmac_sha1(pass, SSID, 4096, &mut pmk).expect("PBKDF2 PMK derivation should succeed");
    (pmk, pass)
}

fn build_beacon(bssid: [u8; 6], ssid: &[u8], ch: u8, rsn: &[u8]) -> [u8; 128] {
    let mut buf = [0u8; 128];
    buf[0] = 0x80;
    buf[1] = 0x00;
    buf[4..10].fill(0xFF);
    buf[10..16].copy_from_slice(&bssid);
    buf[16..22].copy_from_slice(&bssid);
    buf[32] = 0x64;
    buf[34] = 0x11;
    buf[35] = 0x04;
    let mut pos = 36;
    buf[pos] = 0x00;
    buf[pos + 1] = ssid.len() as u8;
    buf[pos + 2..pos + 2 + ssid.len()].copy_from_slice(ssid);
    pos += 2 + ssid.len();
    buf[pos] = 0x03;
    buf[pos + 1] = 0x01;
    buf[pos + 2] = ch;
    pos += 3;
    buf[pos..pos + rsn.len()].copy_from_slice(rsn);
    let _total = pos + rsn.len();
    buf
}

fn build_beacon_vec(bssid: [u8; 6], ssid: &[u8], ch: u8, rsn: &[u8]) -> ([u8; 128], usize) {
    let buf = build_beacon(bssid, ssid, ch, rsn);
    let len = 36 + 2 + ssid.len() + 3 + rsn.len();
    (buf, len)
}

fn build_auth_resp(bssid: [u8; 6], sta: [u8; 6]) -> [u8; 30] {
    let mut buf = [0u8; 30];
    buf[0] = 0xB0;
    buf[4..10].copy_from_slice(&sta);
    buf[10..16].copy_from_slice(&bssid);
    buf[16..22].copy_from_slice(&bssid);
    buf[26] = 0x02;
    buf
}

fn build_assoc_resp(bssid: [u8; 6], sta: [u8; 6], aid: u16) -> [u8; 30] {
    let mut buf = [0u8; 30];
    buf[0] = 0x10;
    buf[4..10].copy_from_slice(&sta);
    buf[10..16].copy_from_slice(&bssid);
    buf[16..22].copy_from_slice(&bssid);
    buf[24] = 0x11;
    buf[25] = 0x04;
    buf[28..30].copy_from_slice(&(aid | 0xC000).to_le_bytes());
    buf
}

fn build_addba_req(bssid: [u8; 6], sta: [u8; 6], dialog: u8) -> [u8; 33] {
    let mut buf = [0u8; 33];
    buf[0] = 0xD0;
    buf[4..10].copy_from_slice(&sta);
    buf[10..16].copy_from_slice(&bssid);
    buf[16..22].copy_from_slice(&bssid);
    buf[24] = 0x03;
    buf[25] = 0x00;
    buf[26] = dialog;
    buf[27] = 0x02;
    buf[28] = 0x10;
    buf
}

fn build_deauth(bssid: [u8; 6], sta: [u8; 6], reason: u16) -> [u8; 26] {
    let mut buf = [0u8; 26];
    buf[0] = 0xC0;
    buf[4..10].copy_from_slice(&sta);
    buf[10..16].copy_from_slice(&bssid);
    buf[16..22].copy_from_slice(&bssid);
    buf[24..26].copy_from_slice(&reason.to_le_bytes());
    buf
}

fn wrap_fromds_eapol(bssid: [u8; 6], sta: [u8; 6], seq: u16, eapol: &[u8]) -> ([u8; 256], usize) {
    let mut buf = [0u8; 256];
    buf[0] = 0x08;
    buf[1] = 0x02;
    buf[4..10].copy_from_slice(&sta);
    buf[10..16].copy_from_slice(&bssid);
    buf[16..22].copy_from_slice(&bssid);
    buf[22..24].copy_from_slice(&((seq & 0x0FFF) << 4).to_le_bytes());
    buf[24..32].copy_from_slice(&[0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00, 0x88, 0x8E]);
    let total = 32 + eapol.len();
    buf[32..total].copy_from_slice(eapol);
    (buf, total)
}

fn build_eapol_m1(anonce: &[u8; 32], replay: u64) -> ([u8; 99], usize) {
    let mut eapol = [0u8; 99];
    eapol[0] = 0x02;
    eapol[1] = 0x03;
    eapol[2..4].copy_from_slice(&95u16.to_be_bytes());
    eapol[4] = 0x02;
    eapol[5..7].copy_from_slice(&0x008Au16.to_be_bytes());
    eapol[7..9].copy_from_slice(&16u16.to_be_bytes());
    eapol[9..17].copy_from_slice(&replay.to_be_bytes());
    eapol[17..49].copy_from_slice(anonce);
    (eapol, 99)
}

fn build_eapol_m3(
    anonce: &[u8; 32],
    replay: u64,
    ap_rsn: &[u8],
    gtk: &[u8; 16],
    gtk_id: u8,
    ptk: &Ptk,
) -> ([u8; 160], usize) {
    let mut plain = [0u8; 48];
    plain[..ap_rsn.len()].copy_from_slice(ap_rsn);
    let mut kde_off = ap_rsn.len();
    plain[kde_off] = 0xDD;
    plain[kde_off + 1] = 22;
    plain[kde_off + 2..kde_off + 5].copy_from_slice(&[0x00, 0x0F, 0xAC]);
    plain[kde_off + 5] = 0x01;
    plain[kde_off + 6] = gtk_id & 0x03;
    plain[kde_off + 7] = 0x00;
    plain[kde_off + 8..kde_off + 24].copy_from_slice(gtk);
    kde_off += 24;
    plain[kde_off] = 0xDD;
    let mut wrapped = [0u8; 56];
    aes_wrap(&ptk.kek, &plain, &mut wrapped).expect("AES key wrap should succeed");

    let total = 99 + wrapped.len();
    let mut eapol = [0u8; 160];
    eapol[0] = 0x02;
    eapol[1] = 0x03;
    eapol[2..4].copy_from_slice(&((total - 4) as u16).to_be_bytes());
    eapol[4] = 0x02;
    eapol[5..7].copy_from_slice(&0x13CAu16.to_be_bytes());
    eapol[7..9].copy_from_slice(&16u16.to_be_bytes());
    eapol[9..17].copy_from_slice(&replay.to_be_bytes());
    eapol[17..49].copy_from_slice(anonce);
    eapol[97..99].copy_from_slice(&(wrapped.len() as u16).to_be_bytes());
    eapol[99..total].copy_from_slice(&wrapped);

    let full_mic = hmac_sha1_vector(&ptk.kck, &[&eapol[..total]]);
    eapol[81..97].copy_from_slice(&full_mic[..16]);
    (eapol, total)
}

fn wrap_fromds_ccmp(
    addrs: ([u8; 6], [u8; 6], [u8; 6]),
    seq: u16,
    key: &[u8; 16],
    key_id: u8,
    pn: u64,
    ip_payload: &[u8],
) -> ([u8; 256], usize) {
    let (da, bssid, sa) = addrs;
    let mut hdr = [0u8; 24];
    hdr[0] = 0x08;
    hdr[1] = 0x42;
    hdr[4..10].copy_from_slice(&da);
    hdr[10..16].copy_from_slice(&bssid);
    hdr[16..22].copy_from_slice(&sa);
    hdr[22..24].copy_from_slice(&((seq & 0x0FFF) << 4).to_le_bytes());

    let mut llc = [0u8; 128];
    llc[0..8].copy_from_slice(&[0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00, 0x08, 0x00]);
    let pt_len = 8 + ip_payload.len();
    llc[8..pt_len].copy_from_slice(ip_payload);

    let mut out = [0u8; 256];
    let len = ccmp_encrypt_mpdu(key, key_id, pn, &hdr, &llc[..pt_len], &mut out)
        .expect("AP CCMP encrypt should succeed");
    (out, len)
}

fn read_tx_mpdu(driver: &Esp32StaWifiDriver<MockWifiMmioBus>) -> &[u8] {
    let len = driver.tx_slot().desc().payload_len() as usize;
    &driver.tx_slot().buffer_slice()[..len]
}

#[test]
fn e2e_ap_sta_wpa2_ccmp_and_smoltcp_lifecycle() {
    host_mock_reset();
    let (pmk, cred) = test_pmk_and_cred();

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
    .expect("RSN IE build should succeed");
    let ap_rsn = &rsn_ie[..rsn_len];

    let mut bus = MockWifiMmioBus::new();
    bus.auto_init_3d24 = true;
    bus.auto_tx_done = true;
    bus.auto_tx_status = 0;

    let mut driver = Esp32StaWifiDriver::new(bus);
    let mac = driver.init(Some(STA_MAC)).expect("driver init should pass");
    assert_eq!(mac, STA_MAC);

    let cfg = StaConnectConfig::from_slices(SSID, cred, None, -90)
        .expect("connect config should be valid");
    let mut now = 1_000u64;
    driver.connect(cfg, now).expect("start connect should pass");
    assert_eq!(driver.state(), StaState::Scanning);
    assert_eq!(host_mock_snapshot().channel, 1);

    let (weak_bcn, weak_len) = build_beacon_vec(AP_WEAK_BSSID, SSID, 1, ap_rsn);
    driver
        .push_mock_rx_mpdu(&weak_bcn[..weak_len], -85)
        .expect("push weak beacon");
    driver.poll(now).expect("poll ch1 weak beacon");

    let (best_bcn, best_len) = build_beacon_vec(AP_BEST_BSSID, SSID, 6, ap_rsn);
    for _step in 0..14 {
        now += 95;
        if host_mock_snapshot().channel == 6 {
            driver
                .push_mock_rx_mpdu(&best_bcn[..best_len], -55)
                .expect("push best beacon on ch6");
        }
        driver.poll(now).expect("scan poll step");
        if driver.state() == StaState::AuthWait {
            break;
        }
    }

    assert_eq!(driver.state(), StaState::AuthWait);
    assert_eq!(driver.bssid(), Some(AP_BEST_BSSID));
    assert_eq!(driver.channel(), 6);
    assert_eq!(host_mock_snapshot().channel, 6);

    let auth_req = parse_80211_header(read_tx_mpdu(&driver)).expect("parse auth req");
    assert_eq!(auth_req.frame_type, FrameType::Mgmt);
    assert_eq!(auth_req.subtype, 0x0B);

    let auth_resp = build_auth_resp(AP_BEST_BSSID, STA_MAC);
    now += 10;
    driver
        .push_mock_rx_mpdu(&auth_resp, -55)
        .expect("push auth resp");
    driver.poll(now).expect("poll auth resp");
    assert_eq!(driver.state(), StaState::AssocWait);

    let assoc_req = parse_80211_header(read_tx_mpdu(&driver)).expect("parse assoc req");
    assert_eq!(assoc_req.frame_type, FrameType::Mgmt);
    assert_eq!(assoc_req.subtype, 0x00);

    let assoc_resp = build_assoc_resp(AP_BEST_BSSID, STA_MAC, 1);
    now += 10;
    driver
        .push_mock_rx_mpdu(&assoc_resp, -54)
        .expect("push assoc resp");
    driver.poll(now).expect("poll assoc resp");
    assert_eq!(driver.state(), StaState::FourWayWait);

    let anonce = [0xA1u8; 32];
    let (m1, m1_len) = build_eapol_m1(&anonce, 1);
    let (m1_mpdu, m1_mpdu_len) = wrap_fromds_eapol(AP_BEST_BSSID, STA_MAC, 10, &m1[..m1_len]);
    now += 10;
    driver
        .push_mock_rx_mpdu(&m1_mpdu[..m1_mpdu_len], -53)
        .expect("push M1");
    driver.poll(now).expect("poll M1");

    let m2_raw = read_tx_mpdu(&driver);
    let m2_eapol = &m2_raw[32..];
    assert_eq!(m2_eapol[1], 0x03);
    let mut snonce = [0u8; 32];
    snonce.copy_from_slice(&m2_eapol[17..49]);
    let ptk = wpa_pmk_to_ptk(
        &pmk,
        b"Pairwise key expansion",
        &STA_MAC,
        &AP_BEST_BSSID,
        &snonce,
        &anonce,
        false,
    );

    let mut m2_zero = [0u8; 256];
    let m2_len = m2_eapol.len();
    m2_zero[..m2_len].copy_from_slice(m2_eapol);
    m2_zero[81..97].fill(0);
    let expected_m2_mic = hmac_sha1_vector(&ptk.kck, &[&m2_zero[..m2_len]]);
    assert_eq!(&m2_eapol[81..97], &expected_m2_mic[..16]);

    let gtk = [0x77u8; 16];
    let gtk_id = 1u8;
    let (m3, m3_len) = build_eapol_m3(&anonce, 2, ap_rsn, &gtk, gtk_id, &ptk);
    let (m3_mpdu, m3_mpdu_len) = wrap_fromds_eapol(AP_BEST_BSSID, STA_MAC, 11, &m3[..m3_len]);
    now += 10;
    driver
        .push_mock_rx_mpdu(&m3_mpdu[..m3_mpdu_len], -52)
        .expect("push M3");
    driver.poll(now).expect("poll M3");
    assert!(driver.is_connected());
    assert_eq!(driver.state(), StaState::Connected);

    let addba_req = build_addba_req(AP_BEST_BSSID, STA_MAC, 7);
    now += 10;
    driver
        .push_mock_rx_mpdu(&addba_req, -52)
        .expect("push ADDBA req");
    driver.poll(now).expect("poll ADDBA req");
    let addba_resp = read_tx_mpdu(&driver);
    assert_eq!(addba_resp[0], 0xD0);
    assert_eq!(addba_resp[24], 0x03);
    assert_eq!(addba_resp[25], 0x01);
    assert_eq!(u16::from_le_bytes([addba_resp[27], addba_resp[28]]), 37);

    let tx_token = driver
        .transmit(Instant::from_millis(now as i64))
        .expect("TxToken available while connected");
    let dst_mac = [0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE];
    let mut tx_eth = [0u8; 32];
    tx_eth[0..6].copy_from_slice(&dst_mac);
    tx_eth[6..12].copy_from_slice(&STA_MAC);
    tx_eth[12..14].copy_from_slice(&[0x08, 0x00]);
    tx_eth[14..32].copy_from_slice(b"ESP32_CCMP_TX_TEST");
    tx_token.consume(32, |buf| {
        buf.copy_from_slice(&tx_eth);
    });

    let enc_tx = read_tx_mpdu(&driver);
    assert_ne!(enc_tx[1] & 0x40, 0);
    let mut ap_plain = [0u8; 256];
    let (ap_pt_len, tx_kid, tx_pn) =
        ccmp_decrypt_mpdu(&ptk.tk, enc_tx, &mut ap_plain).expect("AP decrypt TX CCMP");
    assert_eq!(tx_kid, 0);
    assert!(tx_pn >= 1);
    assert_eq!(&ap_plain[8..ap_pt_len], b"ESP32_CCMP_TX_TEST");

    let (rx_mpdu, rx_len) = wrap_fromds_ccmp(
        (STA_MAC, AP_BEST_BSSID, dst_mac),
        20,
        &ptk.tk,
        0,
        1,
        b"REPLY_OK",
    );
    now += 10;
    driver
        .push_mock_rx_mpdu(&rx_mpdu[..rx_len], -50)
        .expect("push encrypted unicast RX");
    driver.poll(now).expect("poll unicast RX");
    assert_eq!(driver.rx_queue_len(), 1);
    let (rx_tok, _tx_tok) = driver
        .receive(Instant::from_millis(now as i64))
        .expect("RxToken available");
    rx_tok.consume(|eth| {
        assert_eq!(&eth[0..6], &STA_MAC);
        assert_eq!(&eth[6..12], &dst_mac);
        assert_eq!(&eth[14..], b"REPLY_OK");
    });

    let replay_res = driver.push_mock_rx_mpdu(&rx_mpdu[..rx_len], -50);
    assert_eq!(
        replay_res,
        Err(esp32_cam_wifi_mac::DriverError::Sta(
            esp32_cam_wifi_mac::StaError::ReplayDetected
        ))
    );
    driver.poll(now + 5).expect("poll replay packet");
    assert_eq!(driver.rx_queue_len(), 0);
    assert!(driver.receive(Instant::from_millis(now as i64)).is_none());

    let bcast = [0xFFu8; 6];
    let (mcast_mpdu, mcast_len) = wrap_fromds_ccmp(
        (bcast, AP_BEST_BSSID, dst_mac),
        21,
        &gtk,
        gtk_id,
        1,
        b"GTK_OK",
    );
    driver
        .push_mock_rx_mpdu(&mcast_mpdu[..mcast_len], -51)
        .expect("push GTK multicast");
    driver.poll(now + 10).expect("poll GTK multicast");
    assert_eq!(driver.rx_queue_len(), 1);
    let (mcast_tok, _) = driver
        .receive(Instant::from_millis(now as i64))
        .expect("GTK RxToken");
    mcast_tok.consume(|eth| {
        assert_eq!(&eth[0..6], &bcast);
        assert_eq!(&eth[14..], b"GTK_OK");
    });

    let deauth = build_deauth(AP_BEST_BSSID, STA_MAC, 7);
    driver.push_mock_rx_mpdu(&deauth, -55).expect("push deauth");
    driver.poll(now + 15).expect("poll deauth");
    assert!(!driver.is_connected());
    assert_eq!(driver.state(), StaState::Disconnected);
}
