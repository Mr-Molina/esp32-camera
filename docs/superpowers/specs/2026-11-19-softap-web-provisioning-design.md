# Design Spec: Pure-Rust SoftAP Captive-Portal Web Provisioning & Flash Config Store

- **Date**: `2026-11-19`
- **Repository**: `s:\Github\esp32-camera`
- **Target Hardware**: `ESP32-WROVER-DEV` (`esp32` rev `v3.0`, 4 MB DIO Flash, 4 MB PSRAM, `40 MHz`, `MAC d4:d4:da:5b:d4:ec` on `COM20`; `COM13` untouched)
- **Selected Approach**: **Approach B** — Clean-room pure-Rust Open SoftAP + Captive Portal `/setup` Web UI + Dual-Sector Flash Config Store (`0x11000` `nvs` partition) + Setup-Mode-Only UART0 Fallback

---

## 1. Intent & Why

Currently, `klipper-cam-fw` resolves `WIFI_SSID`, `WIFI_PASS`, and `WIFI_OTA_TOKEN` strictly at compile time via `option_env!` (`config.rs:27-34, 93-107, 630-650`), and `main.rs` never loads persistent configuration from flash. Programming a board requires injecting local secrets into the build environment before every initial flash and reflashing whenever Wi-Fi credentials or IP settings change.

This design adds a self-contained, zero-external-dependency, pure-Rust **SoftAP + Captive Portal Web Provisioning** flow so a freshly flashed ESP32-WROVER boots into a local setup access point (`KlipperCam-Setup-<MAC4>` at `192.168.4.1`), serves a mobile-friendly `/setup` web form to configure Wi-Fi SSID, WPA2 passphrase, private OTA token, and IP mode (DHCP or Static), persists those settings power-fail-safely into the unused `nvs` flash partition (`0x11000..0x17000`), and reboots into normal STA mode — with a Setup-Mode-only UART0 fallback to de-risk first-time silicon bring-up of Phase 3C LMAC on `COM20`.

---

## 2. Goals, Non-Goals & Anti-Goals

### Goals
1. Boot into Open SoftAP (`KlipperCam-Setup-D4EC`, IP `192.168.4.1/24`) automatically when no valid Wi-Fi/OTA credentials are provisioned, or when STA fails to associate on boot for `30s` (under `--features provisioning-ap`).
2. Provide pure-Rust UDP DHCPv4 (`67->68`) + captive-portal DNS (`53 -> 192.168.4.1`) + HTTP `/setup` form so phones, laptops, and desktops auto-open or browse directly to `http://192.168.4.1/setup`.
3. Persist Wi-Fi SSID, WPA2 passphrase, private OTA token (`>= 16` printable ASCII, `!= DEFAULT_OTA_TOKEN`), and IP configuration (DHCP-only or Static IP + Gateway + Prefix) in a brownout-safe A/B dual-sector CRC32 store at `0x11000` / `0x12000`.
4. Provide a Setup-Mode-only UART0 fallback (`status`, `set ssid/pass/token/ip`, `save`, `clear`, `reboot`) that never echoes secrets and is completely inactive once running in normal STA mode.
5. Support token-authenticated credential rotation via `GET /setup` & `POST /setup` in normal STA mode (gated by `X-OTA-Token`, `Host` allowlist, and same-origin checks).
6. Keep all 15 Phase 4 verification gates green (`cargo fmt`, `clippy -D warnings`, host tests, Miri 0 UB, Xtensa clippy/build `<= 0x1B0000`, `verify-invariants`, `verify-pure-rust --allow-phase1`, `cargo deny check`, `google_rust_audit.py --strict` `0 errors, 0 warnings`, `pre-commit`).

### Non-Goals
- No WPA2-PSK encryption on the SoftAP in this iteration (Approach C deferred).
- No new external crates (`Cargo.lock` untouched).
- No changes to existing Klipper/Mainsail/Fluidd LAN endpoints (`/status`, `/snapshot`, `/control`, `/update`, `:81/stream`).
- No modifications to `COM13` (Squeezelite-ESP32 Louder) or `esp32-cam-bootloader`.

### Anti-Goals
- **Never** weaken `klipper-cam-fw/build.rs` default release secret enforcement: blank compile-time secrets are allowed **only** when `--features provisioning-ap` is explicitly enabled.
- **Never** permit `POST /update` OTA when `ota_token` is empty, default, or invalid (`is_valid_ota_token` remains enforced fail-closed).
- **Never** duplicate the ~20 KB Wi-Fi DMA `WifiRxRing`/`WifiTxSlot` buffers in internal DRAM — STA and AP modes must share the single static `LmacHal` + DMA buffer allocation.
- **Never** print or echo `wifi_password` or `ota_token` to UART0 logs or HTTP responses.
- **Never** break GPIO16 PSRAM cold-boot behavior or Core 1 flash-pause safety invariants.

---

## 3. Approaches Considered

| Approach | Summary | Trade-Offs | Decision |
|---|---|---|---|
| **Approach A** | Open SoftAP + captive portal `/setup` + dual-sector flash store only | Cleanest Setup Mode surface, but has no fallback if Phase 3C LMAC silicon bring-up hits a radio issue on `COM20` | Rejected |
| **Approach B** | Open SoftAP + captive portal `/setup` + dual-sector flash store + Setup-Mode-only UART0 fallback | Adds ~150 LOC active only in Setup Mode; de-risks first-silicon bring-up on `COM20` while keeping full browser UX | **Selected** |
| **Approach C** | WPA2-PSK SoftAP + captive portal `/setup` + dual-sector flash store | Encrypts setup credentials over the air, but requires a full clean-room WPA2 Authenticator + AP CCMP on untested silicon (~2x scope/risk) | Rejected |

---

## 4. Technical Design

### 4.1 `esp32-cam-wifi-mac`: Clean-Room Open SoftAP (`frames_80211.rs`, `ap_fsm.rs`, `smoltcp_phy.rs`)

1. **802.11 AP Frame Primitives (`crates/esp32-cam-wifi-mac/src/frames_80211.rs`)** — additive only; zero edits to existing STA functions:
   - Management builders:
     - `build_beacon(bssid, ssid, channel, beacon_interval_tu, timestamp_us, seq, out) -> Result<usize, FrameError>`: Type=Mgmt(0), Subtype=Beacon(8), DA=`FF:FF:FF:FF:FF:FF`, SA=BSSID=`ap_mac`, fixed 12 B (`timestamp_us`, `interval = 100 TU`, `cap = 0x0401` ESS + Short Slot, Privacy=0), IEs: SSID(0), Supported Rates(1: `1B,2B,5.5B,11B,6,9,12,18`), DS Param Set(3: `channel`), TIM(5: `[0, 1, 0, 0]`), Extended Supported Rates(50: `24,36,48,54`).
     - `build_probe_resp(da, bssid, ssid, channel, beacon_interval_tu, timestamp_us, seq, out)`: Subtype=5, unicast DA, identical fixed fields & IEs minus TIM.
     - `build_open_auth_resp(da, bssid, auth_seq, status_code, seq, out)`: Subtype=11 (`alg = 0`, `seq = 2`, `status = 0` or `13`/`17`).
     - `build_assoc_resp(da, bssid, aid, status_code, seq, out)`: Subtype=1 (`cap = 0x0401`, `status`, `aid | 0xC000`, Supported + Extended Rates).
     - `build_ap_deauth(da, bssid, reason_code, seq, out)`: Subtype=12 (`Addr1 = da`, `Addr2 = bssid`, `Addr3 = bssid`).
   - Management parsers:
     - `parse_probe_req(payload) -> Result<ProbeReqInfo, FrameError>`: strict TLV framing check; extracts optional SSID slice (empty = wildcard).
     - `parse_assoc_req(payload) -> Result<AssocReqInfo, FrameError>`: validates `>= 4 B` fixed header + TLV framing; extracts SSID slice.
     - `parse_open_auth_req(payload) -> Result<(u16, u16), FrameError>`: extracts `(alg, seq)` from 6 B Auth body.
   - AP Data <-> Ethernet Bridge (unencrypted Open AP only):
     - `ap_eth_to_80211_data(eth_frame, bssid, seq, out) -> Result<usize, FrameError>`: Type=Data(2), Subtype=0, `ToDS=0, FromDS=1`, `Addr1 = eth_da`, `Addr2 = bssid`, `Addr3 = eth_sa`, LLC/SNAP `AA AA 03 00 00 00` + EtherType + payload.
     - `ap_rx_80211_data_to_eth(raw_mpdu, bssid, expected_sta_mac, eth_out) -> Result<usize, FrameError>`: enforces Protocol Version 0, Data/QoS-Data subtype, `ToDS=1 && FromDS=0`, `Addr1 == bssid`, `Addr2 == expected_sta_mac`, rejects Protected, Fragmented (`MF`/`FragNum != 0`), and QoS A-MSDU frames (retaining all FragAttacks defenses), strips LLC/SNAP -> Ethernet `(DA = Addr3, SA = Addr2, EtherType, payload)`.

2. **Open AP State Machine (`crates/esp32-cam-wifi-mac/src/ap_fsm.rs`)** — pure `#![no_std]`, zero hardware calls, Miri-verified:
   - `ApConfig { ap_mac: [u8; 6], ssid: [u8; 32], ssid_len: usize, channel: u8 }` (channel validated `1..=11`, default `1`; SSID format `KlipperCam-Setup-XXXX` from `ap_mac[4..6]` uppercase hex).
   - `MAX_AP_CLIENTS = 4` client table (`ApClient { mac, state: Authenticated | Associated, aid: u16, last_seen_ms: u64 }`), evicting idle clients after `60_000 ms` or oldest slot when full.
   - `ApTxSink` trait (`send_mpdu(&mut self, mpdu: &[u8], rate: WifiPhyRate, wait_ack: bool) -> Result<(), ApError>`).
   - `poll_tick(now_ms, sink)`: emits periodic Beacon every `100 ms` (`Rate1mL`, `wait_ack = false`) and expires stale clients.
   - `handle_rx_mpdu(raw_mpdu, now_ms, sink, eth_out) -> Option<usize>`:
     - ProbeReq (`Addr1 == bcast || ap_mac`, wildcard or exact SSID) -> `ProbeResp` (`Rate1mL`, `wait_ack = false`).
     - AuthReq (`Addr1 == ap_mac`, `alg == 0`, `seq == 1`) -> upsert `Authenticated` client, send `AuthResp(status = 0)` (`Rate1mL`, `wait_ack = true`; non-zero alg -> `status = 13`).
     - AssocReq / ReassocReq (`Addr1 == ap_mac`, exact SSID match, client `Authenticated` or `Associated`) -> promote `Associated`, assign `aid` `1..=4`, send `AssocResp(status = 0, aid)` (`Rate1mL`, `wait_ack = true`).
     - Deauth / Disassoc (`Addr1 == ap_mac`) -> remove client.
     - Data / QoS Data (`Addr1 == ap_mac`, `ToDS=1, FromDS=0`, `Addr2` in `Associated` clients) -> `ap_rx_80211_data_to_eth` -> returns `Some(eth_len)` (updates `last_seen_ms`).
   - `transmit_eth(eth_frame, sink)`: broadcast/multicast (`da[0] & 1 != 0`) -> `Rate1mL`, `wait_ack = false`; unicast -> requires `da` in `Associated` client table, transmits at `Rate6m` with automatic single retry at `Rate1mL` if `wait_ack` fails.

3. **Shared-DMA Mode Switching on `Esp32StaWifiDriver<B>` (`crates/esp32-cam-wifi-mac/src/smoltcp_phy.rs`)**:
   - **DRAM-Saving Builder Decision**: Rather than allocating a second 20 KB DMA driver struct, add an internal `WifiMacMode` enum (`Sta` | `Ap(ApMacCore)`) on `Esp32StaWifiDriver<B>` (or `enter_ap_mode(&mut self, cfg: ApConfig) -> Result<(), DriverError>` / `exit_ap_mode(&mut self)`).
   - In AP mode: switches PHY to `cfg.channel` (`phy_switch_channel_20mhz`), sets `lmac.set_scan_rx_policy()` (BSSID promiscuous bypass so wildcard-BSSID Probe Requests reach `ApMacCore`, which strictly filters `Addr1 == ap_mac || broadcast` in software), keeps `WifiRxRing`/`WifiTxSlot`/`LmacHal`/Drop/malformed-RX-recycle invariants 100% shared, and routes `poll()` and `smoltcp::phy::Device` TX/RX through `ApMacCore`!
   - When `WifiMacMode::Sta` is active, behavior is 100% identical to existing certified STA code.

---

### 4.2 Dual-Sector Power-Fail-Safe Provisioning Store (`crates/klipper-cam-fw/src/prov_store.rs`)

- **Flash Layout** (inside existing `nvs` partition `0x11000..0x17000` in `partitions_rust_ota.csv`):
  - `PROV_SECTOR_A_OFFSET = 0x11000` (`4096 B`)
  - `PROV_SECTOR_B_OFFSET = 0x12000` (`4096 B`)
- **Record Layout (`PROV_RECORD_LEN = 256 B`, little-endian, 4-byte aligned)**:
  - `0..4`: `magic = *b"KCFG"`
  - `4..6`: `version: u16 = 1`
  - `6..8`: `flags: u16` (`FLAG_PROVISIONED = 1 << 0`, `FLAG_STATIC_IP = 1 << 1`)
  - `8..12`: `seq: u32` (`1..=u32::MAX - 1`)
  - `12`: `ssid_len: u8`, `13..45`: `ssid: [u8; 32]`
  - `45`: `pass_len: u8`, `46..109`: `pass: [u8; 63]`
  - `109`: `token_len: u8`, `110..174`: `token: [u8; 64]`
  - `174..178`: `static_ip: [u8; 4]`
  - `178..182`: `gateway: [u8; 4]`
  - `182..186`: `dns: [u8; 4]`
  - `186`: `prefix_len: u8`
  - `187..252`: `reserved: [u8; 65]` (must be `0`)
  - `252..256`: `crc32: u32` (IEEE 802.3 CRC32 over `0..252`)
- **Validation & Security**:
  - `ProvisionedConfig` implements `Drop` with volatile zeroization of `pass` and `token`, and manual `Debug` redacting both secrets.
  - Strict validation on both `decode()` and `save()`:
    - `ssid`: `1..=32` valid UTF-8, no control chars (`< 0x20` or `0x7F`).
    - `pass`: `8..=63` valid UTF-8 printable ASCII (`0x20..=0x7E`), or `0` only under `cfg!(feature = "allow-open-wifi")`.
    - `token`: `is_valid_ota_token(token)` (`16..=64` printable ASCII `0x21..=0x7E`, `!= DEFAULT_OTA_TOKEN`).
    - If `FLAG_STATIC_IP` is set: non-zero unicast IPv4 (`!ip.is_unspecified() && !ip.is_broadcast() && !ip.is_multicast()`), `prefix_len in 8..=30`, valid gateway/dns.
- **Brownout-Safe A/B Write**:
  - `load(flash)` reads both sectors via `OtaFlashStorage::read_aligned`; picks the valid record with highest `seq` (returns `Ok(None)` if both erased/invalid).
  - `save(flash, cfg)` computes `next_seq = active_seq.checked_add(1)` (rejecting `>= u32::MAX`), selects the **inactive** sector (`B` if `A` is active, else `A`), erases that single 4 KB sector (`erase_range`), writes the 256 B record (`write_aligned`), reads back 256 B and verifies exact byte equality + CRC32! Because `EspFlashAdapter` automatically wraps every flash op in `Core1PauseGuard::with_core1_paused` + `ensure_spi1_flash_cs0_only()`, flash writes are completely safe even if Core 1 is active in STA mode!

---

### 4.3 Pure-Rust Captive-Portal Network Services (`crates/klipper-cam-fw/src/captive_net.rs`)

- **UDP DHCPv4 Server** (bound to `0.0.0.0:67`, responses sent to `255.255.255.255:68` or unicast client IP):
  - Fixed 4-slot lease table (`192.168.4.2`..`192.168.4.5`, `lease_secs = 3600`).
  - Handles `DHCPDISCOVER (1)` -> `DHCPOFFER (2)` and `DHCPREQUEST (3)` -> `DHCPACK (5)` (or `DHCPNAK (6)` if requested IP is outside `192.168.4.2..=5`).
  - Emits Options: `53` (Msg Type), `54` (`192.168.4.1`), `51` (`3600`), `1` (`255.255.255.0`), `3` (`192.168.4.1`), `6` (`192.168.4.1`), `114` (RFC 8910 Captive-Portal URI `"http://192.168.4.1/setup"`), `255` (End).
- **UDP Captive-Portal DNS Responder** (bound to `192.168.4.1:53`):
  - Validates 12 B header (`QR == 0`, `QDCOUNT >= 1`), parses QNAME safely with strict bounds (`<= 255 B`, rejects compression pointers in queries), reads `QTYPE` & `QCLASS`.
  - For `QTYPE == 1 (A)` or `255 (ANY)` with `QCLASS == 1 (IN)`: echoes query + answers `192.168.4.1` (TTL `30s`, `AA = 1, RA = 1`).
  - For `QTYPE == 28 (AAAA)` or other types: returns clean `NOERROR` with `ANCOUNT = 0` (prevents OS captive-portal stalls).
- **Socket Budget (`main.rs`)**:
  - Setup Mode runs its own dedicated local `smoltcp` `Interface` (`192.168.4.1/24`) and small local `SocketSet` (`1` DHCP UDP socket + `1` DNS UDP socket + `2` Port 80 TCP sockets) inside `run_provisioning_ap_loop` before Core 1 camera capture starts — zero impact on STA Mode's `SOCKET_STORAGE` or PSRAM/camera RAM!

---

### 4.4 Web Provisioning UI & Setup-Mode UART0 Fallback (`http.rs`, `prov_uart.rs`, `main.rs`, `build.rs`)

1. **HTTP `/setup` Endpoints (`crates/klipper-cam-fw/src/http.rs`)**:
   - **In SoftAP Setup Mode** (dedicated handler `handle_setup_http_request`, active only on `192.168.4.1`):
     - `GET /generate_204`, `/gen_204`, `/hotspot-detect.html`, `/connecttest.txt`, `/ncsi.txt`, `/canonical.html`, `/redirect`, `/` -> `302 Found` with `Location: http://192.168.4.1/setup`.
     - `GET /setup` -> `200 OK` compact HTML form (`<= 3 KB`, zero external resources, `X-Frame-Options: DENY`, `Content-Security-Policy: default-src 'self' 'unsafe-inline'; frame-ancestors 'none'`, `X-Content-Type-Options: nosniff`, `Cache-Control: no-store`):
       - Displays Device MAC (`d4:d4:da:5b:d4:ec`) and AP SSID.
       - Form inputs: `ssid`, `pass` (`type="password"`), `token` (`type="password"`, placeholder `>= 16 chars`), `ip_mode` (`dhcp` | `static`), `static_ip` (`172.22.1.52`), `gateway` (`172.22.1.1`), `prefix` (`24`).
     - `POST /setup` (`application/x-www-form-urlencoded`, body `<= 512 B`, strict percent-decoding rejecting invalid `%XX` / control bytes):
       - Validates all fields identically to `ProvStore::save`.
       - On validation error -> `400 Bad Request` HTML showing the exact field error (never echoing back `pass` or `token`).
       - On valid submission -> writes to `ProvStore` (A/B sector + readback verify), returns `200 OK` HTML ("Configuration saved. Rebooting into Wi-Fi STA mode..."), flushes TCP socket (`<= 3s` cap), and calls `quiesce_psram_and_software_reset()`.
   - **In Normal STA Mode**:
     - `GET /setup` serves the blank setup HTML form (displaying only non-secret MAC, current SSID, and current IP mode — never echoing `wifi_password` or `ota_token`) with a `Current OTA Token` field whose inline JS submits `POST /setup` via `fetch('/setup', { headers: { 'X-OTA-Token': ..., 'X-Requested-With': 'fetch' } })` (matching the existing `/update` browser pattern).
     - `POST /setup` strictly enforces constant-time `check_ota_auth` (`X-OTA-Token`), `Host` allowlist (`M-047`), and `is_same_origin` (`M-046`), and validates a complete replacement credential + IP record (no partial-merge ambiguity). On success, it saves via `ProvStore` and schedules a clean reboot.

2. **Setup-Mode-Only UART0 Fallback (`crates/klipper-cam-fw/src/prov_uart.rs`)**:
   - Polled **exclusively** inside `run_provisioning_ap_loop` via non-blocking UART0 RX FIFO read (`UART0_STATUS_REG` `RXFIFO_CNT` + `UART0_FIFO_REG`, zero blocking).
   - Fixed 192 B line buffer (`CR`/`LF` terminated, backspace handling, drops oversized lines safely).
   - Commands: `help`, `status` (prints MAC, AP SSID, `192.168.4.1`, staged fields with `pass`/`token` redacted as `[set, N bytes]`), `set ssid <v>`, `set pass <v>`, `set token <v>`, `set ip dhcp`, `set ip static <ip> <gw> <prefix>`, `save`, `clear`, `reboot`.
   - Zeroizes staging struct and line buffer on `Drop` and after `save`/`clear`.

3. **Boot Flow, Credential Precedence, IP Modes & Feature Gate (`Cargo.toml`, `build.rs`, `config.rs`, `main.rs`)**:
   - Add feature `provisioning-ap = []` to `crates/klipper-cam-fw/Cargo.toml` (off by default).
   - `build.rs`: `check_release_secrets` accepts `allow_provisioning_ap: bool` (mirrors `CARGO_FEATURE_PROVISIONING_AP`). When `true`, empty `WIFI_PASS` / empty `WIFI_OTA_TOKEN` are allowed at compile time, while any *non-empty* value is still strictly validated (no whitespace/CTLs, `!= DEFAULT_OTA_TOKEN`, `>= 16 B` token, `8..=63 B` pass).
   - **Exact `has_sta_credentials()` & Precedence Contract (`config.rs`)**:
     1. A valid `ProvStore` record in flash always takes highest precedence and counts as provisioned STA credentials.
     2. When `ProvStore` is empty (`Ok(None)`), compile-time credentials count as valid **only** if `option_env!("WIFI_SSID")` is explicitly set and non-empty AND (`option_env!("WIFI_PASS")` is 8..=63 bytes, OR `cfg!(feature = "allow-open-wifi")`). Fallback `DEFAULT_WIFI_SSID` (`"iot"`) with unset `WIFI_PASS` is **never** treated as provisioned credentials under `--features provisioning-ap`, and `DEFAULT_OTA_TOKEN` is never accepted as an active OTA token under `provisioning-ap`.
     3. If valid STA credentials exist (from flash or explicit compile-time env) while `ota_token` is blank, the firmware boots into STA mode with `POST /update` and STA `POST /setup` hard-locked at `403`; if STA credentials are missing, it enters `run_provisioning_ap_loop` immediately on boot (avoiding a 30s scan or `boot_wifi_auth()` panic).
   - **Exact `IpMode` Contract (`config.rs`, `main.rs`)**:
     - `IpMode::Static(cfg)` (default `StaticIpConfig::KLIPPER_DEFAULT` for backward compatibility when unprovisioned/non-provisioning builds): configures only `cfg.ip`/`cfg.prefix_len`/`cfg.gateway` in `smoltcp`, gratuitous ARP announcements, and `Host` allowlist (`active_ip`), with `dhcpv4::Socket` disabled.
     - `IpMode::Dhcp`: omits the hardcoded `172.22.1.52/24` static IP, runs `dhcpv4::Socket` exclusively, and updates `runtime.server.set_active_ip(lease_ip)` + ARP announce upon lease acquisition. If L2 STA associates cleanly but DHCP does not acquire a lease within `90s`, the existing watchdog reboots and retries cleanly without breaking slow DHCP servers.
   - **Power-Outage Recovery Timer in SoftAP Fallback**:
     - If `run_provisioning_ap_loop` was entered on an **unprovisioned** board (`ProvStore` empty), stay in SoftAP indefinitely until provisioned.
     - If `run_provisioning_ap_loop` was entered as a **STA fallback while valid `ProvStore` credentials exist** (e.g., ESP32 booted faster than a router after a power outage), track SoftAP client association and HTTP/UART activity; if idle with zero associated clients for `180s`, reboot cleanly to retry STA (resetting the idle timer on any client association, HTTP request, or UART command so active setup is never interrupted).
   - `main.rs` startup sequence:
     1. Init hardware, PSRAM, PHY, `wifi_driver`, `EspFlashAdapter`.
     2. Read `ProvStore::load(&mut flash)` from `0x11000..0x12FFF` and overlay onto `KlipperCamConfig`.
     3. If `cfg!(feature = "provisioning-ap")` and `!config.has_sta_credentials()` -> enter `run_provisioning_ap_loop` immediately (before starting Core 1 camera producer).
     4. Otherwise, attempt STA `connect_strongest_ap` (2 attempts, ~30s total). If STA fails AND `cfg!(feature = "provisioning-ap")`, log warning and enter `run_provisioning_ap_loop` with the 180s idle-retry timer.
     5. Once provisioned and rebooted into STA mode, normal dual-core camera + HTTP Port 80/81 server runs identically to before.

---

## 5. Verification Plan & Hardware Bring-Up (`COM20`)

1. **Host & Miri Tests**:
   - `esp32-cam-wifi-mac`: unit + Miri tests for all AP frame builders/parsers, `ApMacCore` (Beacon cadence, ProbeReq->ProbeResp, Auth, Assoc, client eviction, unassociated-STA rejection, FragAttacks rejection on `ToDS=1`, broadcast/unicast TX), and `Esp32StaWifiDriver` AP mode switch + `Drop`.
   - `klipper-cam-fw`: unit tests for `ProvStore` (encode/decode, CRC corruption, A/B brownout recovery, validation rejection, secret redaction/zeroize), `captive_net` (DHCP DISCOVER/OFFER/REQUEST/ACK/NAK, DNS A/AAAA/malformed bounds), `/setup` HTTP GET/POST + form URL-decoding + captive redirects + STA token-gated `/setup`, `prov_uart` line parser + secret redaction, and `wp09_build_policy` (`provisioning-ap` feature matrix).
2. **Workspace Gates**: Full 15-gate suite (`fmt`, host `clippy -D warnings`, `cargo test --workspace --locked`, 7-crate Miri, Xtensa `clippy` + `build --release --features provisioning-ap`, `espflash save-image <= 0x1B0000`, `verify-invariants`, `verify-pure-rust --allow-phase1`, `cargo deny check`, `google_rust_audit.py crates --strict` `0 errors, 0 warnings`).
3. **Live `COM20` Bring-Up (After Plan Approval)**:
   - Flash `COM20` (`MAC d4:d4:da:5b:d4:ec`, `COM13` untouched) using `espflash.toml` (`esp32-cam-bootloader.bin` at `0x1000`, `partitions_rust_ota.csv` at `0x8000`, `klipper-cam-fw` `--features provisioning-ap` at `0x20000`).
   - Capture live UART0 boot log on `COM20` confirming GPIO16 PSRAM init, pure-Rust bootloader jump, `KlipperCam-Setup-D4EC` SoftAP startup on `192.168.4.1`, and readiness for you to connect and enter credentials via `http://192.168.4.1/setup`!

---

## 6. Calls Made by Designer (For Your Review)

- **SoftAP SSID & IP**: `KlipperCam-Setup-D4EC` (last 2 bytes of hardware MAC) on Channel `1`, IP `192.168.4.1/24`, DHCP pool `192.168.4.2..=5`.
- **Flash Storage**: Custom 256 B A/B dual-sector CRC32 record at `0x11000` & `0x12000` inside `nvs` partition (matching `otadata.rs`'s brownout-safe pattern with zero new dependencies).
- **Fresh-Flash Credential Rule**: Under `--features provisioning-ap`, built-in fallback `DEFAULT_WIFI_SSID` (`"iot"`) without a passphrase is never treated as provisioned; a fresh flash boots directly into SoftAP Setup Mode without wasting 30s scanning.
- **STA-Mode `/setup` Browser Flow**: `GET /setup` renders the blank form (no secrets echoed) with a `Current OTA Token` input that submits `POST /setup` via `fetch()` with `X-OTA-Token` + same-origin + `Host` checks, requiring a complete replacement record.
- **Power-Outage Recovery**: When SoftAP is entered as a fallback with existing saved credentials (e.g., ESP32 booted faster than the router), it reboots to retry STA after `180s` of zero associated clients / zero HTTP or UART activity; an unprovisioned board stays in SoftAP indefinitely.
- **IP Mode Separation**: `IpMode::Static` disables DHCPv4 completely (pure static IP/gateway/prefix); `IpMode::Dhcp` omits the hardcoded `172.22.1.52/24` static IP and runs DHCPv4 exclusively.
- **OTA Security**: `POST /update` remains hard-locked (`403`) until a valid `>= 16`-byte private OTA token is saved via `/setup` (or Setup-Mode UART0).
