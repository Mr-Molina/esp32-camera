# Pure-Rust SoftAP Web Provisioning & Flash Config Store Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add clean-room pure-Rust Open SoftAP (`KlipperCam-Setup-D4EC`), UDP DHCPv4 + captive-portal DNS, HTTP `/setup` web UI, brownout-safe dual-sector flash config persistence, and Setup-Mode-only UART0 fallback to `esp32-camera`, then build, verify, and flash `COM20` (`MAC d4:d4:da:5b:d4:ec`).

**Architecture:** `esp32-cam-wifi-mac` adds additive Open AP 802.11 frames, `ApMacCore`, and shared-DMA mode switching on `Esp32StaWifiDriver<B>` (zero extra ~20 KB DRAM allocation; STA behavior untouched). `klipper-cam-fw` adds a 256 B A/B CRC32 `ProvStore` at `0x11000`/`0x12000` (`nvs`), pure-Rust UDP DHCPv4/DNS responders (`captive_net`), `/setup` SoftAP + STA handlers (`http`), a Setup-Mode-only UART0 CLI (`prov_uart`), and `IpMode::{Dhcp, Static}` + `--features provisioning-ap` boot orchestration in `main.rs`.

**Tech Stack:** Pure `#![no_std]` Rust (`1.99.1` host / `+esp` Xtensa `xtensa-esp32-none-elf`), `smoltcp`, `espflash 4.6.0`, zero new crates (`Cargo.lock` unchanged).

**Spec:** `docs/superpowers/specs/2026-11-19-softap-web-provisioning-design.md`

## Global Constraints

- Work on `master`; **do NOT commit or push** (`CLAUDE.md`).
- Pure-Rust repository: zero `.py`, `.sh`, `.c`, `.cpp`, `.h`, or `.S` files; `Cargo.lock` unchanged (`--locked`).
- Zero `unwrap()` in production code; dual `// JUSTIFICATION:` + `// SAFETY:` on every `unsafe` block; `google_rust_audit.py crates --strict` MUST exit `0` (`0 error(s), 0 warning(s)`).
- Never weaken default `build.rs` release secret enforcement: blank compile-time secrets allowed **only** under `--features provisioning-ap`; `POST /update` remains `403` until a valid `>= 16`-byte private OTA token is provisioned.
- Never print or echo `wifi_password` or `ota_token` to UART0 or HTTP; volatile-zeroize secret buffers on `Drop`.
- Never touch `COM13` (Squeezelite-ESP32 Louder); target **`COM20`** (`MAC d4:d4:da:5b:d4:ec`) exclusively.

## Review Focus

1. **Fresh-flash default leakage**: under `--features provisioning-ap` with empty `ProvStore`, built-in `DEFAULT_WIFI_SSID` (`"iot"`) and `DEFAULT_OTA_TOKEN` (`"klipper-cam-ota-2026"`) must NOT satisfy `has_sta_credentials()` or `is_valid_ota_token()` (pinned in Task 3).
2. **Form percent-encoding edge cases**: `POST /setup` with `+`, `%20`, `%26`, `%3D`, `%25`, invalid `%ZZ`, NUL `%00`, or CR/LF `%0D%0A` must decode accurately or reject with `400` without echoing `pass`/`token` (pinned in Task 7).
3. **Brownout / torn sector recovery in `ProvStore`**: corrupt CRC or `0xFF` torn write in one sector while the other holds valid `seq = 3` must cleanly load `seq = 3` and write `seq = 4` to the corrupt sector (pinned in Task 5).
4. **Rogue / malformed SoftAP frames**: unassociated STA `ToDS=1` Data, Protected/Fragmented/A-MSDU Data, and client-table saturation (>4 STAs) must be rejected/evicted safely without leaking Ethernet frames (pinned in Tasks 1 & 4).
5. **Adversarial DHCPv4 & DNS packets**: truncated `<240 B` DHCP, wrong magic cookie, out-of-pool `DHCPREQUEST` (`DHCPNAK`), DNS compression-pointer loop (`0xC0 0x0C`), and `AAAA` queries must never panic or infinite-loop (pinned in Task 6).

---

### Task 1: Open AP 802.11 Frame Builders & Parsers (`esp32-cam-wifi-mac`)

**Files:**
- Modify: `crates/esp32-cam-wifi-mac/src/frames_80211.rs`

**Interfaces:**
- Consumes: existing `FrameError`, `parse_80211_header`, LLC/SNAP constants in `frames_80211.rs`.
- Produces:
  - `pub struct ProbeReqInfo<'a> { pub ssid: Option<&'a [u8]> }`
  - `pub struct AssocReqInfo<'a> { pub capability: u16, pub listen_interval: u16, pub ssid: &'a [u8] }`
  - `pub fn build_beacon(bssid: &[u8; 6], ssid: &[u8], channel: u8, interval_tu: u16, timestamp_us: u64, seq: u16, out: &mut [u8]) -> Result<usize, FrameError>`
  - `pub fn build_probe_resp(da: &[u8; 6], bssid: &[u8; 6], ssid: &[u8], channel: u8, interval_tu: u16, timestamp_us: u64, seq: u16, out: &mut [u8]) -> Result<usize, FrameError>`
  - `pub fn parse_probe_req(payload: &[u8]) -> Result<ProbeReqInfo<'_>, FrameError>`
  - `pub fn parse_open_auth_req(payload: &[u8]) -> Result<(u16, u16), FrameError>`
  - `pub fn build_open_auth_resp(da: &[u8; 6], bssid: &[u8; 6], auth_seq: u16, status_code: u16, seq: u16, out: &mut [u8]) -> Result<usize, FrameError>`
  - `pub fn parse_assoc_req(payload: &[u8]) -> Result<AssocReqInfo<'_>, FrameError>`
  - `pub fn build_assoc_resp(da: &[u8; 6], bssid: &[u8; 6], aid: u16, status_code: u16, seq: u16, out: &mut [u8]) -> Result<usize, FrameError>`
  - `pub fn build_ap_deauth(da: &[u8; 6], bssid: &[u8; 6], reason_code: u16, seq: u16, out: &mut [u8]) -> Result<usize, FrameError>`
  - `pub fn ap_eth_to_80211_data(eth_frame: &[u8], bssid: &[u8; 6], seq: u16, out: &mut [u8]) -> Result<usize, FrameError>`
  - `pub fn ap_rx_80211_data_to_eth(raw_mpdu: &[u8], bssid: &[u8; 6], expected_sta_mac: &[u8; 6], eth_out: &mut [u8]) -> Result<usize, FrameError>`

- [ ] **Step 1: Write failing unit tests in `frames_80211.rs`** covering beacon/probe-resp roundtrip (`parse_beacon_or_probe_resp`), wildcard & exact `parse_probe_req`, `parse_open_auth_req`/`build_open_auth_resp`, `parse_assoc_req`/`build_assoc_resp`, `ap_eth_to_80211_data` (`ToDS=0, FromDS=1`), `ap_rx_80211_data_to_eth` (`ToDS=1, FromDS=0`), and Review Focus #4 (rejecting Protected, Fragmented, A-MSDU, wrong `Addr1`/`Addr2`, SSID > 32 B, truncated TLV).
- [ ] **Step 2: Run `cargo test -p esp32-cam-wifi-mac --locked -- frames_80211::`** and confirm tests fail to compile before implementation.
- [ ] **Step 3: Implement the additive AP builders/parsers in `frames_80211.rs`** using `cap = 0x0401`, Supported Rates `[0x82, 0x84, 0x8B, 0x96, 0x0C, 0x12, 0x18, 0x24]`, Extended Rates `[0x30, 0x48, 0x60, 0x6C]`, TIM `[0, 1, 0, 0]`, `aid | 0xC000`, and zero edits to existing STA functions.
- [ ] **Step 4: Verify** `cargo test -p esp32-cam-wifi-mac --locked`, `cargo +nightly miri test --locked -p esp32-cam-wifi-mac -- frames_80211::`, `cargo clippy -p esp32-cam-wifi-mac --all-targets --locked -- -D warnings`, and `google_rust_audit.py crates/esp32-cam-wifi-mac/src/frames_80211.rs --strict` all exit `0`.

---

### Task 2: `provisioning-ap` Cargo Feature & `build.rs` Policy Gate (`klipper-cam-fw`)

**Files:**
- Modify: `crates/klipper-cam-fw/Cargo.toml`
- Modify: `crates/klipper-cam-fw/build.rs`
- Modify: `crates/klipper-cam-fw/tests/wp09_build_policy.rs`

**Interfaces:**
- Consumes: existing `check_release_secrets` and `OTA_TOKEN_VARS` in `build.rs`.
- Produces:
  - Feature `provisioning-ap = []` in `crates/klipper-cam-fw/Cargo.toml` (disabled by default).
  - Updated `pub fn check_release_secrets_with_mode(target_arch: &str, profile: &str, allow_open_wifi: bool, allow_provisioning_ap: bool, var: &dyn Fn(&str) -> Option<String>) -> Result<(), String>` (keeping `check_release_secrets` forwarding with `allow_provisioning_ap = false` for backward compatibility).

- [ ] **Step 1: Add failing policy tests in `wp09_build_policy.rs`** asserting: (a) `allow_provisioning_ap = true` permits unset/empty `WIFI_PASS` and `WIFI_OTA_TOKEN` on Xtensa `release`; (b) `allow_provisioning_ap = true` STILL rejects `DEFAULT_OTA_TOKEN`, `< 16 B` non-empty token, whitespace token, and `< 8 B` non-empty `WIFI_PASS`; (c) `allow_provisioning_ap = false` retains 100% existing strict rejections.
- [ ] **Step 2: Run `cargo test -p klipper-cam-fw --test wp09_build_policy --locked`** and confirm failure.
- [ ] **Step 3: Add `provisioning-ap = []` to `Cargo.toml` and implement `check_release_secrets_with_mode` in `build.rs`** wiring `CARGO_FEATURE_PROVISIONING_AP` in `main()`.
- [ ] **Step 4: Verify** `cargo test -p klipper-cam-fw --test wp09_build_policy --locked`, `cargo clippy -p klipper-cam-fw --all-targets --locked -- -D warnings`, and `google_rust_audit.py crates/klipper-cam-fw/build.rs --strict` exit `0`.

---

### Task 3: `IpMode`, Provisioned Credentials Precedence & `has_sta_credentials()` (`config.rs`)

**Files:**
- Modify: `crates/klipper-cam-fw/src/config.rs`

**Interfaces:**
- Consumes: `StaticIpConfig`, `NetworkConfig`, `KlipperCamConfig`, `NvsConfigOverrides`, `is_valid_ota_token` in `config.rs`.
- Produces:
  - `#[derive(Debug, Clone, Copy, PartialEq, Eq)] pub enum IpMode { Dhcp, Static(StaticIpConfig) }`
  - `NetworkConfig.ip_mode: IpMode` (default `IpMode::Static(StaticIpConfig::KLIPPER_DEFAULT)`)
  - `KlipperCamConfig::has_sta_credentials(&self) -> bool`
  - `KlipperCamConfig::has_valid_ota_token(&self) -> bool`
  - `NvsConfigOverrides.ip_mode: Option<IpMode>`
  - Compile-time helpers `has_explicit_build_wifi_ssid()`, `has_explicit_build_wifi_pass()`, and `active_ota_token()` returning `""` under `cfg!(feature = "provisioning-ap")` when no env var is set (never falling back to `DEFAULT_OTA_TOKEN`).

- [ ] **Step 1: Add failing unit tests in `config.rs`** covering Review Focus #1: fresh `from_build_env()` under `provisioning-ap` (or empty `wifi_password` / unprovisioned defaults) returns `has_sta_credentials() == false` and `has_valid_ota_token() == false`; `with_nvs_overrides` setting SSID + WPA2 pass + valid token + `IpMode::Dhcp` yields `has_sta_credentials() == true`, `has_valid_ota_token() == true`, and `ip_mode == IpMode::Dhcp`; `IpMode::Static` validates IP/gateway/prefix.
- [ ] **Step 2: Run `cargo test -p klipper-cam-fw --lib --locked -- config::`** and confirm failure.
- [ ] **Step 3: Implement `IpMode`, `has_sta_credentials()`, `has_valid_ota_token()`, and updated `from_build_env()`/`with_nvs_overrides()` in `config.rs`** keeping all existing `WROVER_KIT_HD` & non-`provisioning-ap` unit tests passing unchanged.
- [ ] **Step 4: Verify** `cargo test -p klipper-cam-fw --locked`, `cargo clippy -p klipper-cam-fw --all-targets --locked -- -D warnings`, and `google_rust_audit.py crates/klipper-cam-fw/src/config.rs --strict` exit `0`.

---

### Task 4: Open SoftAP FSM & Shared-DMA AP Mode (`esp32-cam-wifi-mac`)

**Files:**
- Create: `crates/esp32-cam-wifi-mac/src/ap_fsm.rs`
- Modify: `crates/esp32-cam-wifi-mac/src/smoltcp_phy.rs`
- Modify: `crates/esp32-cam-wifi-mac/src/lib.rs`

**Interfaces:**
- Consumes: Task 1 AP functions in `frames_80211.rs`, `LmacHal`, `WifiRxRing`, `WifiTxSlot`, `WifiPhyRate`.
- Produces:
  - `pub const MAX_AP_CLIENTS: usize = 4;`
  - `pub struct ApConfig { pub ap_mac: [u8; 6], pub ssid: [u8; 32], pub ssid_len: usize, pub channel: u8 }` with `ApConfig::for_setup_mac(mac: [u8; 6], channel: u8) -> Self` (`"KlipperCam-Setup-XXXX"`).
  - `pub trait ApTxSink { fn send_mpdu(&mut self, mpdu: &[u8], rate: WifiPhyRate, wait_ack: bool) -> Result<(), ApError>; }`
  - `pub struct ApMacCore` with `new(cfg: ApConfig) -> Result<Self, ApError>`, `poll_tick(&mut self, now_ms: u64, sink: &mut impl ApTxSink)`, `handle_rx_mpdu(&mut self, raw_mpdu: &[u8], now_ms: u64, sink: &mut impl ApTxSink, eth_out: &mut [u8]) -> Option<usize>`, `transmit_eth(&mut self, eth_frame: &[u8], sink: &mut impl ApTxSink) -> Result<(), ApError>`, `associated_client_count(&self) -> usize`, `last_activity_ms(&self) -> u64`.
  - On `Esp32StaWifiDriver<B>`: `pub fn enter_ap_mode(&mut self, cfg: ApConfig) -> Result<(), DriverError>`, `pub fn exit_ap_mode(&mut self)`, `pub fn is_ap_mode(&self) -> bool`, `pub fn ap_associated_clients(&self) -> usize`, `pub fn ap_last_activity_ms(&self) -> u64`.

- [ ] **Step 1: Write failing unit tests in `ap_fsm.rs` & `smoltcp_phy.rs`** verifying: 100 ms Beacon cadence, ProbeReq->ProbeResp, Open Auth (`alg=0` ok, `alg=1` status `13`), AssocReq->AssocResp with AID `1..=4`, Review Focus #4 (unassociated STA data dropped, 5th client evicts LRU, idle 60s eviction), broadcast vs unicast TX (with `Rate6m`->`Rate1mL` fallback), and `Esp32StaWifiDriver::enter_ap_mode()` sharing the single DMA ring + `set_scan_rx_policy()` + `Drop::shutdown()`.
- [ ] **Step 2: Run `cargo test -p esp32-cam-wifi-mac --locked`** and confirm failure.
- [ ] **Step 3: Implement `ap_fsm.rs`, export in `lib.rs`, and wire shared-DMA AP mode into `Esp32StaWifiDriver<B>` in `smoltcp_phy.rs`** (preserving 100% of existing STA behavior and `Drop`/malformed-RX-recycle invariants).
- [ ] **Step 4: Verify** `cargo test -p esp32-cam-wifi-mac --locked`, `cargo +nightly miri test --locked -p esp32-cam-wifi-mac`, `cargo clippy -p esp32-cam-wifi-mac --all-targets --locked -- -D warnings`, Xtensa clippy on `esp32-cam-wifi-mac`, and `google_rust_audit.py crates/esp32-cam-wifi-mac/src --strict` exit `0`.

---

### Task 5: Dual-Sector Brownout-Safe `ProvStore` (`klipper-cam-fw`)

**Files:**
- Create: `crates/klipper-cam-fw/src/prov_store.rs`
- Modify: `crates/klipper-cam-fw/src/lib.rs`

**Interfaces:**
- Consumes: `OtaFlashStorage` (`ota.rs`), `IpMode`, `StaticIpConfig`, `is_valid_ota_token` (`config.rs`).
- Produces:
  - `pub const PROV_SECTOR_A_OFFSET: u32 = 0x11000;`
  - `pub const PROV_SECTOR_B_OFFSET: u32 = 0x12000;`
  - `pub const PROV_RECORD_LEN: usize = 256;`
  - `pub struct ProvisionedConfig` (fixed-size buffers, redacted `Debug`, volatile-zeroizing `Drop`, getters `ssid_str()`, `pass_str()`, `ota_token_str()`, `ip_mode()`, `validate()`, `apply_to(&self, cfg: &mut KlipperCamConfig) -> Result<(), ProvStoreError>`).
  - `pub struct ProvStore;` with `load<F: OtaFlashStorage>(flash: &mut F) -> Result<Option<ProvisionedConfig>, ProvStoreError>`, `save<F: OtaFlashStorage>(flash: &mut F, cfg: &ProvisionedConfig) -> Result<u32, ProvStoreError>`, `erase_all<F: OtaFlashStorage>(flash: &mut F) -> Result<(), ProvStoreError>`.

- [ ] **Step 1: Write failing unit tests in `prov_store.rs`** using an in-memory mock `OtaFlashStorage` covering: blank `0xFF` returns `Ok(None)`, `save`+`load` roundtrip for both `IpMode::Dhcp` and `IpMode::Static`, alternating A/B sectors with monotonic `seq`, Review Focus #3 (corrupt CRC / torn `0xFF` in Sector B recovers Sector A `seq=3` and writes `seq=4` to Sector B), validation rejections (empty SSID, short/whitespace/default token, invalid static IP/prefix), and `Debug` secret redaction.
- [ ] **Step 2: Run `cargo test -p klipper-cam-fw --lib --locked -- prov_store::`** and confirm failure.
- [ ] **Step 3: Implement `prov_store.rs` and export `pub mod prov_store;` in `lib.rs`** using IEEE 802.3 CRC32, readback byte-equality check after write, and volatile zeroization in `Drop`.
- [ ] **Step 4: Verify** `cargo test -p klipper-cam-fw --lib --locked -- prov_store::`, `cargo clippy -p klipper-cam-fw --all-targets --locked -- -D warnings`, and `google_rust_audit.py crates/klipper-cam-fw/src/prov_store.rs --strict` exit `0`.

---

### Task 6: Pure-Rust Captive Portal UDP Services & Setup-Mode UART0 CLI (`klipper-cam-fw`)

**Files:**
- Create: `crates/klipper-cam-fw/src/captive_net.rs`
- Create: `crates/klipper-cam-fw/src/prov_uart.rs`
- Modify: `crates/klipper-cam-fw/src/lib.rs`

**Interfaces:**
- Consumes: `ProvisionedConfig`, `IpMode`, `StaticIpConfig`, `is_valid_ota_token` (`prov_store.rs`, `config.rs`).
- Produces:
  - `captive_net.rs`:
    - `pub const AP_IP: [u8; 4] = [192, 168, 4, 1];`
    - `pub struct DhcpServer` with `new() -> Self`, `handle_packet(&mut self, req: &[u8], out: &mut [u8]) -> Option<usize>` (pool `192.168.4.2..=5`, RFC 8910 Option 114 `"http://192.168.4.1/setup"`).
    - `pub fn handle_captive_dns_query(req: &[u8], ap_ip: [u8; 4], out: &mut [u8]) -> Option<usize>`
  - `prov_uart.rs`:
    - `pub enum UartProvAction { Reply(heapless::String<256>), SaveAndReboot(ProvisionedConfig), Reboot, None }` (or fixed `[u8; 256]` + `len` zero-heap buffer).
    - `pub struct UartProvSession` with `new() -> Self`, `feed_byte(&mut self, byte: u8, mac: &[u8; 6], ap_ssid: &str) -> UartProvAction` (zeroizing `Drop`, never echoes `pass` or `token`).

- [ ] **Step 1: Write failing unit tests in `captive_net.rs` & `prov_uart.rs`** covering: DHCP DISCOVER->OFFER, REQUEST->ACK, out-of-pool REQUEST->NAK, 4-client lease reuse, DNS `A`/`ANY` -> `192.168.4.1`, `AAAA` -> `NOERROR` empty, Review Focus #5 (truncated `<240 B` DHCP, bad cookie, DNS compression-pointer loop `0xC0 0x0C`, oversized QNAME), and UART commands (`help`, `status` redacting secrets, `set ssid/pass/token/ip`, `save` returning `SaveAndReboot`, invalid field errors).
- [ ] **Step 2: Run `cargo test -p klipper-cam-fw --lib --locked`** and confirm failure.
- [ ] **Step 3: Implement `captive_net.rs`, `prov_uart.rs`, and export both in `lib.rs`**.
- [ ] **Step 4: Verify** `cargo test -p klipper-cam-fw --lib --locked`, `cargo clippy -p klipper-cam-fw --all-targets --locked -- -D warnings`, and `google_rust_audit.py crates/klipper-cam-fw/src --strict` exit `0`.

---

### Task 7: `/setup` Captive-Portal & STA-Mode HTTP Endpoints (`http.rs`)

**Files:**
- Modify: `crates/klipper-cam-fw/src/http.rs`

**Interfaces:**
- Consumes: `ProvisionedConfig`, `ProvStore`, `IpMode`, `StaticIpConfig`, `is_valid_ota_token`, `check_ota_auth`, `is_allowed_host`, `is_same_origin`.
- Produces:
  - `pub enum SetupHttpOutcome { Responded(usize), SavedAndReboot { resp_len: usize, config: ProvisionedConfig } }`
  - `pub fn handle_setup_http_request(raw_req: &[u8], mac: &[u8; 6], ap_ssid: &str, current_ssid: &str, out_buf: &mut [u8]) -> SetupHttpOutcome` (SoftAP mode: captive probe `302 -> http://192.168.4.1/setup`, `GET /setup` HTML form `<= 3 KB`, `POST /setup` form validation + percent-decoding).
  - STA-mode `/setup` route in `KlipperCamServer`: `GET /setup` renders blank form with `Current OTA Token` JS `fetch()` submit; `POST /setup` enforces `check_ota_auth` + `is_allowed_host` + `is_same_origin`, validates full replacement `ProvisionedConfig`, saves via `ProvStore::save(flash, &cfg)`, and sets `reboot_scheduled = true`.

- [ ] **Step 1: Write failing unit tests in `http.rs`** covering: all captive-portal probes (`/generate_204`, `/hotspot-detect.html`, `/connecttest.txt`, `/`) returning `302 -> http://192.168.4.1/setup`; `GET /setup` returning `200` with `X-Frame-Options: DENY`, `CSP`, `Cache-Control: no-store` and no echoed secrets; Review Focus #2 (`POST /setup` decoding `+`, `%20`, `%26`, `%3D`, `%25`, rejecting `%ZZ`/`%00`/`%0D%0A` with `400` and zero secret echo); STA-mode `GET /setup` returning `200` without secrets; STA-mode `POST /setup` rejecting missing/wrong `X-OTA-Token` (`401`/`403`) or bad `Host` (`403`) and persisting + scheduling reboot on valid token.
- [ ] **Step 2: Run `cargo test -p klipper-cam-fw --lib --locked -- http::`** and confirm failure.
- [ ] **Step 3: Implement `handle_setup_http_request`, strict form percent-decoder, and STA-mode `/setup` routes in `http.rs`**.
- [ ] **Step 4: Verify** `cargo test -p klipper-cam-fw --locked`, `cargo clippy -p klipper-cam-fw --all-targets --locked -- -D warnings`, and `google_rust_audit.py crates/klipper-cam-fw/src/http.rs --strict` exit `0`.

---

### Task 8: Firmware Boot Flow, `IpMode` Wiring & `run_provisioning_ap_loop` (`main.rs`)

**Files:**
- Modify: `crates/klipper-cam-fw/src/main.rs`

**Interfaces:**
- Consumes: `ProvStore`, `ProvisionedConfig`, `DhcpServer`, `handle_captive_dns_query`, `UartProvSession`, `handle_setup_http_request`, `ApConfig`, `Esp32StaWifiDriver::{enter_ap_mode, ap_associated_clients, ap_last_activity_ms}`, `IpMode`.
- Produces: Complete end-to-end boot + SoftAP Setup Mode + STA fallback + `IpMode::{Dhcp, Static}` + STA `/setup` persistence in `klipper-cam-fw`.

- [ ] **Step 1: Write/extend host unit tests in `main.rs`** verifying: (a) Setup Mode boot decision helper (`should_enter_setup_on_boot(provisioning_ap, has_sta_creds)`); (b) power-outage fallback idle timeout helper (`should_retry_sta_after_ap_idle(had_saved_creds, associated_clients, now_ms, last_activity_ms) -> true` after `180_000 ms` only when `had_saved_creds && associated_clients == 0`).
- [ ] **Step 2: Run `cargo test -p klipper-cam-fw --locked`** and confirm new tests fail.
- [ ] **Step 3: Wire `ProvStore::load`, `IpMode::{Dhcp, Static}` (pure static disables DHCPv4; DHCP omits hardcoded `172.22.1.52/24`), `run_provisioning_ap_loop` (SoftAP + UDP 67/53 + Port 80 `/setup` + non-blocking UART0 RX FIFO fallback + 180s fallback idle-retry), 30s STA fallback, and STA `POST /setup` persistence into `main.rs`** while preserving ALL `verify-invariants` hardware rules (GPIO16 PSRAM, `SPI0_PIN_REG`, `arm_rtc_wdt_15s`, Core 1 pause).
- [ ] **Step 4: Verify** host `cargo test -p klipper-cam-fw --locked`, host `clippy`, Xtensa `clippy` (both default AND `--features provisioning-ap`), `cargo xtask verify-invariants`, and `google_rust_audit.py crates/klipper-cam-fw/src --strict` exit `0`.

---

### Task 9: Full 15-Gate Verification Suite & Live `COM20` Flash + Serial Bring-Up

**Files:**
- No code edits (verification + flashing `COM20` only; `COM13` untouched).

- [ ] **Step 1: Run all host, Miri, static, and supply-chain gates** (`cargo fmt --all -- --check`, `cargo clippy --workspace --all-targets --locked -- -D warnings`, `cargo test --workspace --locked`, 7-crate Miri, `verify-invariants`, `verify-pure-rust --allow-phase1`, `cargo deny check`, `google_rust_audit.py crates --strict`, `pre-commit run --all-files`).
- [ ] **Step 2: Build Xtensa release binaries** — verify default release build (with ephemeral CI placeholder env) AND build the deployable `--features provisioning-ap` release image (`cargo +esp build --locked --target xtensa-esp32-none-elf -Zbuild-std=core,alloc -p klipper-cam-fw --release --features provisioning-ap`) + verify `espflash save-image <= 0x1B0000` (`1,769,472 B`).
- [ ] **Step 3: Flash `COM20` (`MAC d4:d4:da:5b:d4:ec`, leaving `COM13` untouched)** using `espflash.toml` (`esp32-cam-bootloader.bin` at `0x1000`, `partitions_rust_ota.csv` at `0x8000`, `klipper-cam-fw` at `0x20000`, erasing `otadata` via `--erase-data-parts ota` and verifying `nvs` clean if needed).
- [ ] **Step 4: Capture live UART0 boot log on `COM20`** (~10–15s bounded capture, then cleanly release `COM20` port handle) confirming GPIO16 PSRAM init, pure-Rust bootloader jump, `KlipperCam-Setup-D4EC` SoftAP startup on `192.168.4.1`, and `GET /setup` + UART0 fallback readiness.
