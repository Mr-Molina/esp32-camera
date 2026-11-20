# ESP32-WROVER Pure-Rust Wi-Fi STA WPA2-CCMP Data-Plane Handoff Plan

- **Date**: `2026-11-20`
- **Repository**: `esp32-camera` (`/home/console-admin/Github/esp32-camera`, branch `master`, synced to `origin/master`)
- **Base Plan**: `docs/superpowers/plans/2026-11-14-rea-full-rust-firmware-plan.md`
- **Prior Handoff**: `docs/superpowers/plans/2026-11-16-rea-pure-rust-handoff-plan.md`
- **Obsidian Research Note**: `[[ESP32 Bare-Metal Wi-Fi MAC WPA2-CCMP Data-Plane RX and TX Architecture]]`
- **Handoff Status**: Phases `0–3B` `[EXECUTED & VERIFIED]`; Phase `3C` (`esp32-cam-wifi-mac`) `[SILICON ASSOCIATED & WPA2 4-WAY CONNECTED (1.23s) — POST-CONNECTED CCMP DATA PLANE OPEN]`; Phases `3D` (`esp32-cam-phy` / `libphy.a` retirement) and `3E` (`EspAppDesc` cutover & dropping `--allow-phase1`) `[QUEUED]`.

---

## Executive "Resume Here" (`2026-11-20` Session Handoff)

### What Works 100% on Silicon (`[EXECUTED & VERIFIED]` — 6/6 Consecutive Boots)

1. **Bootloader, PSRAM, & Camera Pipeline**: Pure-Rust 2nd-stage bootloader (`0x1000`, `17,632 B`, `GPIO 16` PSRAM CS pull-up `0x3FF4_904C = 0x2B00`) + 4 MiB PSRAM (`0x3F80_0000`) + 7-crate HIL self-test + live OV2640 I2S0 DMA `1280x720` JPEG capture (`~19.7–22.2 KB`, `attempt 1`).
2. **Flash Provisioning (`KCFG`) & Precomputed PMK**: `KCFG` `seq=2` flash config load (`0x12000` in dual-sector `0x11000..0x13000`, `--features provisioning-ap`, landed in `5c023b8`) + pre-`connect()` WPA2 PMK precomputation (`StaMacCore::precompute_pmk` / `Esp32StaWifiDriver::warm_pmk_cache` + `feed_rtc_wdt()`, landed in `78cd656`), eliminating the `2,050 ms` `pbkdf2_hmac_sha1` stall inside `finish_scan_sweep(now_ms)`.
3. **802.11 Scan, Auth, Association, & Full WPA2-PSK 4-Way Handshake**: 13-channel active scan (`+0..1175 ms`), Open System Auth (`+1190 ms`), Association (`+1195 ms`, `aid 2..6` with Extended Supported Rates + RSN + 9-byte WMM IE), and full WPA2-PSK 4-way handshake (`EAPOL M1..M4`) into `StaState::Connected` at **`+1,230 ms`** (`tx_fail = 0`), with zero AP `EAPOL M3` retransmits and zero `Deauth` frames for 30+ seconds (proving 100% that UniFi accepted `EAPOL M4`, installed PTK, and opened our 802.1X port).

### Exact Remaining Blocker (`Phase 3C` Post-`Connected` WPA2-CCMP Data Plane)

- After `StaState::Connected`, `smoltcp` transmits encrypted `DHCP DISCOVER` frames (`tx_ok = 3, tx_fail = 0, pri_ok = 1` at `11 Mbps` CCK, `fb_ok = 2`), which receive hardware 802.11 ACKs from `02:ec:da:b7:c8:b4`.
- However, `WifiRxRing` receives **`0` `Protected=1` frames (`rx_prot = 0, prot = None, last_data_rx = None`)** while continuing to receive `150–290` `Protected=0` management/beacon frames (`rx_pop`, `rx_ctrl = 0x8000_0000`, `malf = 0`).
- Because no encrypted data frames arrive, `smoltcp` never receives `DHCP OFFER` and no `172.22.1.X` lease is acquired.

---

## 1. Phase & Architecture Status Matrix (`13` Workspace Crates)

| Phase / Workstream | Scope & Crates | Status | Verified Metrics & Notes |
|---|---|---|---|
| **Phase 0** (`REA Baseline`) | `rea` evidence & ELF symbol attribution | `[EXECUTED & VERIFIED]` | Baseline `1,928 / 2,328` symbols attributed; `phase0_sta_reachability.json` |
| **Phase 1** (`Host Tooling`) | `crates/xtask` (replaces legacy Python/Shell scripts) | `[EXECUTED & VERIFIED]` | Host verification, pure-Rust audit, passive serial guard |
| **Phase 2A–2C** (`Bootloader`) | `crates/esp32-cam-bootloader` (`0x1000`) | `[EXECUTED & VERIFIED]` | `17,632` B <= `28,672` B (`0x7000`), `GPIO 16` pull-up (`0x3FF4_904C = 0x2B00`), 4 MiB PSRAM, live `ota_0`/`ota_1` + OTADATA |
| **Phase 3A** (`RF/Clock & Seam`) | `crates/esp32-cam-rf-clock`, `crates/esp-wifi-sys-esp32-seam` | `[EXECUTED & VERIFIED]` | Retired `10 / 14` Espressif C archives |
| **Phase 3B** (`WPA2 Supplicant`) | `crates/esp32-cam-wpa2` | `[EXECUTED & VERIFIED]` | Pure-Rust 802.11i WPA2-PSK supplicant & AES/CCMP/HMAC/PBKDF2 crypto; retired `libwpa_supplicant.a` |
| **Phase 3C** (`Wi-Fi MAC & LMAC`) | `crates/esp32-cam-wifi-mac`, `crates/klipper-cam-fw` | `[SILICON ASSOCIATED & WPA2 4-WAY CONNECTED (1.23s) — POST-CONNECTED CCMP DATA PLANE OPEN]` | Retired `libnet80211.a`, `libpp.a`, `esp-radio`, `esp-rtos`; 6/6 consecutive silicon boots reach `StaState::Connected` (`+1,230 ms`); blocked on post-`Connected` `rx_prot = 0` (`DHCP OFFER` never received) |
| **Phase 3D** (`PHY Crate`) | `crates/esp32-cam-phy` (retire `libphy.a` + seam) | `[QUEUED]` | **DO NOT START** until Phase 3C live DHCP + HTTP `/status`, `/snapshot`, and `:81/stream` are certified on silicon |
| **Phase 3E** (`App Desc & Cutover`) | `crates/klipper-cam-fw`, `crates/xtask`, CI | `[QUEUED]` | Clean-room `EspAppDesc`, drop `--allow-phase1`, enforce `0` C/Assembly DWARF CUs |

### Workspace Crate Map (`13` Crates)

1. `crates/esp32-cam-types` — Domain types, pixel formats, controls, errors (`#![no_std]`, Miri-verified)
2. `crates/esp32-cam-ll` — MMIO registers, DMA descriptors, compile-fail contracts (`#![no_std]`, Miri-verified)
3. `crates/esp32-cam-conversions` — Pixel format conversions (`#![no_std]`, Miri-verified)
4. `crates/esp32-cam-sccb` — SCCB/I2C bus abstraction & mock bus (`#![no_std]`)
5. `crates/esp32-cam-sensors` — Camera sensor drivers (`OV2640`, etc.) (`#![no_std]`)
6. `crates/esp32-cam-hal` — DMA frame buffer pool & camera pipeline (`#![no_std]`)
7. `crates/klipper-cam-fw` — Top-level ESP32-WROVER firmware binary (`#![no_std]`)
8. `crates/xtask` — Host verification gates, pure-Rust auditing, serial passive checks
9. `crates/esp32-cam-bootloader` — Pure-Rust 2nd-stage bootloader (`0x1000`, `17,632` B) (`#![no_std]`, Miri-verified)
10. `crates/esp32-cam-rf-clock` — Pure-Rust RF/coex MMIO, regulatory tables, printf compat (`#![no_std]`, Miri-verified)
11. `crates/esp-wifi-sys-esp32-seam` — Seam crate linking `phy` only on Xtensa
12. `crates/esp32-cam-wpa2` — Pure-Rust WPA2-PSK supplicant & AES/CCMP crypto (`#![no_std]`, Miri-verified)
13. `crates/esp32-cam-wifi-mac` — Pure-Rust 802.11 STA MAC, LMAC DMA/MMIO, `smoltcp` PHY adapter (`#![no_std]`, Miri-verified)

---

## 2. Attached Hardware, Provisioning, & Network Ground Truth

### Hardware & Serial Interface (`ESP32-WROVER-DEV`)

- **Board**: `ESP32-WROVER-DEV` (Dual-Core 240 MHz, 4 MiB DIO Flash, 4 MiB PSRAM @ `0x3F80_0000`, OV2640 HD `1280x720` on SCCB `0x30`).
- **Hardware STA eFuse MAC (THIS unit)**: `d4:d4:da:5a:e7:3c` (distinct physical unit from the earlier `d4:d4:da:5b:d4:ec` board).
- **Serial Port**: `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` (`TIOCMGET = 0x0000`, `DTR=0, RTS=0` passive invariant enforced via `cargo --locked xtask ensure-serial-passive`).
  - **NEVER use `espflash monitor`** (it hangs / toggles modem lines).
  - Capture passive raw UART0 output using:
    ```bash
    /bin/stty -F /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 115200 raw -echo -hupcl
    /usr/bin/timeout 32s /bin/cat /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
    cargo --locked xtask ensure-serial-passive
    ```
- **Flash Provisioning State (`--features provisioning-ap`, landed in `5c023b8`)**:
  - Flash Sector B (`0x12000`, inside dual-sector `KCFG` `0x11000..0x13000`) holds verified `KCFG` `seq=2` (`ssid='iot'`, `IpMode::Dhcp`, valid CRC32 + provisioned passphrase + OTA token).
  - Flashing `ota_0` (`0x20000`) preserves `0x11000..0x13000`.
  - **Security invariant**: Never read, print, commit, or log Wi-Fi passphrases, `.env.local`, or OTA tokens.

### Host Network & AP Routing

- **Target AP**: Ubiquiti UniFi `'iot'` AP (`bssid=02:ec:da:b7:c8:b4`, Channel 1 `2412 MHz`, WPA2-PSK CCMP).
  - **Supported Rates IE**: `[0x96, 0x0c, 0x12, 0x18, 0x24, 0x30, 0x48, 0x60]` -> **`0x96` = 11 Mbps minimum basic rate**!
  - **Subnet / Gateway**: `172.22.1.0/24`, gateway `172.22.1.1`.
- **Host Interface**: Host `eno1` (`10.0.0.90` via `10.0.0.1`).
  - Always bind host probes explicitly: `ping -I eno1 ...` / `curl --interface eno1 ...`.
- **Multicast/Broadcast Isolation Note**: UniFi `'iot'` blocks LAN-to-WLAN multicast/broadcast by default, so host broadcast `arping` will NOT reach wireless STAs; only AP-originated unicast/EAPOL/DHCP responses reach the STA.

---

## 3. Authoritative Disassembly Reference (ESP-IDF v5.5.4 `libpp.a`, `libnet80211.a`, `libwpa_supplicant.a`)

Every register, bitfield, and byte contract below is verified from ESP-IDF v5.5.4 archives so the receiving agent wastes zero cycles re-disassembling them.

### 3.1 Hardware Crypto Key Table (`0x3FF7_4400 + slot * 0x28`)

- **Call chain (`libpp.a:hal_crypto.o` `hal_crypto_set_key_entry` & `wdev.o` `wDev_Insert_KeyEntry`)**:
  `esp_wifi_set_sta_key_internal -> ppInstallKey -> ic_set_key -> wDev_Insert_KeyEntry -> hal_crypto_set_key_entry`
- **40-byte (`10-word`) entry layout at `0x3FF7_4400 + slot * 0x28`**:
  - `Word 0` (`+0x00`): `u32::from_le_bytes(bssid[0..4])` (`wpa.c.obj` `wpa_supplicant_install_ptk` and `wpa_supplicant_install_gtk` BOTH pass `gWpaSm+0x382` = `ap_bssid`).
  - `Word 1` (`+0x04`):
    - `[15:0]` = `u16::from_le_bytes(bssid[4..6])`
    - `[20:18]` = `alg` (`3` for CCMP -> `0x000C_0000`)
    - `[23:21]` = `mode` (`6` -> `0x00C0_0000` for group GTK `slot < 4`; `3` -> `0x0060_0000` for pairwise PTK `slot >= 4`)
    - `[25:24]` = `vif_idx` (`0` for STA VIF 0)
    - `[31:30]` = `key_idx & 0x03`
  - `Words 2..5` (`+0x08..+0x14`): 16-byte CCMP TK/GTK (`LE32`).
  - `Words 6..9` (`+0x18..+0x24`): `0`.
- **Slot mapping (`esp_wifi_get_sta_hw_key_idx_internal`)**:
  - Pairwise PTK -> `slot = 4` (`key_idx = 0`)
  - Group GTK -> `slot = 0` (`key_idx = 1`), `slot = 1` (`key_idx = 0` or `2`)
- **Key-valid bitmask (`0x3FF7_3814`)**: RMW `|= (1 << slot)`.
- **VIF crypto enable (`hal_crypto_enable(0, 3, 1, 0)`)**:
  - `0x3FF7_3810 &= 0xFF00_00FF`
  - `0x3FF7_3800 = 0x0003_0103`
  - `0x3FF7_3808 = 0`
  - `0x3FF7_380C = 0`
  - *Verification status*: `lmac_mmio.rs::install_sta_ccmp_key` is 100% bit-exact.

### 3.2 RX Hardware Decap & Software CCMP

- **RX Hardware Decap (`libnet80211.a:ieee80211_crypto_ccmp.o` `ccmp_decap`)**:
  - When hardware CCMP decrypts an incoming frame (`rx_state == 0x00`), BOTH the 8-byte CCMP IV (`+hdrlen..+hdrlen+8`) and the 8-byte trailing MIC (`len-8..len`) remain in the RX MPDU buffer (`rx_80211_data_to_eth_ext` matches `ccmp_decap`).
- **Software CCMP (`libwpa_supplicant.a:ccmp.c.obj` / `aes-ccm.c.obj`)**:
  - `esp32-cam-wpa2` (`crypto/ccm.rs`) CCMP-128 header, 13-byte Nonce (`[priority, A2, PN5..PN0]`), 22B/24B AAD (`0x8F`, `0xC7`/`0x47 | 0x40`, `0x0F`), and RFC 3610 `B_0`/`A_i` blocks are 100% byte-identical to `ccmp.c.obj` / `aes-ccm.c.obj`.

### 3.3 TX `PLCP1` (`0x3FF7_4258`) Encryption Mode & Bandwidth Contract

From `libpp.a:hal_mac_tx.o` (`mac_tx_set_plcp1`) and `libnet80211.a:ieee80211_output.o` / `ieee80211_crypto_ccmp.o`:

- **`TX_PLCP1_REG[24:17] = (tx_info[16] as u32) << 17`**:
  1. **Unencrypted (`key == NULL`)**: `tx_info[16] = 0x00` -> `0x0000_0000`.
  2. **Software-CCMP fallback (`ieee80211_ccmp_encrypt` `0xdf..0xed`)**: Used in ESP-IDF **ONLY when `slot >= 25`**: `tx_info->word0 &= !1`, `tx_info[16] = 0xC0` -> `0x0180_0000` (currently implemented in `WO-STA-FIX-14`).
  3. **Native STA Hardware-CCMP TX (`ccmp_encap` + `ieee80211_encap_esfbuf` `0x461..0x484` — USED FOR ALL `slot < 25` IN ESP-IDF, NOT YET TESTED IN RUST)**:
     - `ccmp_encap` prepends **ONLY** the 8-byte CCMP IV (`[PN0, PN1, 0, (kid<<6)|0x20, PN2..PN5]`, with `PN` starting at `1`) before plaintext LLC/SNAP + IP (`esf_buf->len += 8`; NO software AES-CCM and NO 8-byte MIC in `esf_buf->len`).
     - Sets `FC[1] |= 0x40`, sets `tx_info->word0 |= 1`, sets `tx_info[17] |= 3` (`ic_cipher = 3`), and sets `tx_info[16] = 0x40 | slot` = `0x44` -> **`TX_PLCP1_REG[24:17] = 0x0088_0000`**.
- **`TX_PLCP1_REG` bit 28 (`0x1000_0000`)**:
  - `mac_tx_set_plcp1` sets bit 28 ONLY when both CBW40 flags (`tx_info[0]` bit 14 and `tx_info[8]` bit 15) are `1` (`0` for 20 MHz legacy/OFDM).
  - Note: CCK `AuthReq` on silicon needed `0x1000_0000` preserved in Attempt 4 (`WO-STA-FIX-03`), so only test clearing bit 28 on `rate.is_ofdm()` if touching OFDM rates.

### 3.4 Post-Association RX Policy & TSF

- `sta_recv_assoc` (`ieee80211_supplicant.o`) calls `wifi_set_rx_policy(6)` -> `ic_set_bssid(0, bssid)` + `ic_set_rx_policy(0, 1, 1, 1)` + `ic_set_rx_policy_ubssid_check(0, 1)` (`0x3FF7_30D8 |= 0x02`).
- `wifi_station_start` (`hal_tsf.o`) calls `hal_enable_sta_tsf` (`0x3FF7_5020 |= 0x9800_0000`).

---

## 4. Graveyard of Tested / Falsified Hypotheses (`Attempts 1–16` — DO NOT REPEAT)

Do **not** re-attempt or loop on any of these already-falsified hypotheses:

| Work Order / Attempt | Change Tested | Observed Silicon Outcome / Ruling |
|---|---|---|
| `WO-STA-FIX-01` (Attempt 2) | Remove `+ 4` FCS tail in `dma_ring.rs::tx_prepare` / `lmac_mmio.rs` | Hardware FCS overwrote the last 4 bytes of `AuthReq`. **Keep `+ 4` FCS tail.** |
| `WO-STA-FIX-03` (Attempt 4) | Clear `0x1000_0000` (`TX_PLCP1_REG` bit 28) globally on CCK `AuthReq` | Caused `TxAckFailed(5)` on `AuthReq`. **Never clear bit 28 on CCK frames.** |
| `WO-STA-FIX-05` (Attempt 6) | Use `Rate11mL` for pre-assoc `ProbeReq`/`AssocReq` | Regressed `AssocWait`. **Keep `Rate1mL` for management + EAPOL frames.** |
| `WO-STA-FIX-06` (Attempt 7) | Zero-delay synchronous retries on `AssocReq` / `EAPOL M2` | Collided with AP's `EAPOL M1` / `M3`. |
| `WO-STA-FIX-08/09` (Attempts 9–10) | Modify `phy_switch_channel_20mhz` in `phy_bridge.rs` | Caused `TxHardwareError`. **`5c023b8` `phy_bridge.rs` works 100%; do not alter.** |
| `WO-STA-FIX-11` (Attempt 12) | Set `0x3FF7_30D8 |= 0x02` alone before HW crypto was programmed | No effect without HW keys installed. |
| `WO-STA-FIX-13` (Attempt 14) | Bracket-toggle `0x3FF7_3800 = 0x0003_0000` around TX | `rx_prot = 0` (`pri_ok = 0, fb_ok = 2`). |
| `WO-STA-FIX-14` (Attempt 15) & `WO-STA-FIX-15` (Attempt 16) | Software-CCMP TX with `TX_PLCP1_REG |= 0x0180_0000` + WMM IE `DD 07 00 50 F2 02 00 01 00` & 26B `0x88` QoS Data | Both associated cleanly (`1,230 ms`) and got `pri_ok = 1` (`11 Mbps` 802.11 ACK), while `rx_prot = 0` persisted. Note: `WO-STA-FIX-14` and `WO-STA-FIX-15` remain cleanly in `master` as verified vendor-parity improvements. |

---

## 5. Ordered, Single-Variable Action Plan for the Receiving Agent

> **Operational Guardrails**:
> - Obtain **explicit user confirmation** before flashing, touching `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`, or probing the LAN.
> - Change **ONE variable per flash cycle** (`Rule 5`).
> - Honor `/3strikes` (`Rule 2`, `Rule 7`): after 3 consecutive failures on a subsystem, stop modifying and perform deep research.

### Step 1A: Fastest Over-the-Air Bisect (If a 2nd Linux Wi-Fi Interface / USB Sniffer Is Available)

1. Put a secondary Wi-Fi adapter into 20 MHz monitor mode on Channel 1 (`2412 MHz`).
2. Capture `EAPOL M1..M4` + `DHCP DISCOVER` during 1 boot.
3. Inspect the capture in `tshark` / `wireshark`:
   - **(a)** Does the ESP32's `0x88` `DHCP DISCOVER` have a valid 802.11 FCS **and** decrypt cleanly under the session PTK? (Proves TX is 100% valid vs. corrupted by LMAC.)
   - **(b)** Does `02:ec:da:b7:c8:b4` transmit an encrypted `DHCP OFFER` (unicast PTK vs. broadcast GTK) on the air?

### Step 1B: Highest-Probability Code-Only Hypothesis — Native ESP-IDF Hardware-CCMP TX (`ccmp_encap` + `PLCP1 = 0x0088_0000`)

- **Rationale**: In ESP-IDF (`libnet80211.a`), `ccmp_encap` (`tx_info[16] = 0x44` -> `PLCP1[24:17] = 0x0088_0000`) is the **only** TX path exercised when `0x3FF7_3800 = 0x0003_0103` and `slot = 4` are installed (`ieee80211_ccmp_encrypt` SW-CCMP is fallback code for `slot >= 25`). When `0x3FF7_3800 = 0x0003_0103` is enabled on VIF 0, ESP32 TX hardware may mishandle `FC.Protected == 1` frames unless fed `ccmp_encap` format.
- **Pre-Code Disassembly Check (MANDATORY)**:
  - Disassemble `ppTxPkt` / `wDev_AppendTx` in `libpp.a` to confirm exact `lldesc.length` and `TX_PLCP1_REG[11:0]` (`psdu_len`) accounting for the 8-byte hardware-appended CCMP MIC + 4-byte FCS (`26B QoS hdr + 8B CCMP IV + plain_len` in DMA buffer vs. `+ 8B MIC + 4B FCS` in `PLCP1[11:0]`).
- **Execution**: Implement hardware CCMP TX (`PLCP1 |= 0x0088_0000`) as a **single-variable** test, unit-test, and certify on silicon.

### Step 2: Data TX Rate Table Hardening

- Update `StaMacCore::transmit_eth` from `[Rate11mS (0x07), Rate6m (0x0B), Rate1mL (0x00)]` to `[Rate11mL (0x03), Rate11mS (0x07), Rate6m (0x0B)]`.
- Clear `TX_PLCP1_BANDWIDTH_BIT` (`0x1000_0000`) **only** when `rate.is_ofdm()`.
- **Rationale**: Fallback retries must never drop to `1 Mbps` (`Rate1mL`), because UniFi `'iot'` enforces an `11 Mbps` minimum basic data rate (`0x96`) and drops lower-rate data frames after L1 auto-ACK.

### Step 3: Hardware RX Filter / TSF / Error Counter Audit (If TX Is Confirmed Valid)

If Step 1A/1B confirms TX frames decrypt cleanly on the air or if `rx_prot = 0` persists:
1. Test `0x3FF7_30D8 |= 0x02` (`ic_set_rx_policy_ubssid_check(0, 1)` in `set_associated_bssid`, cleared in `set_scan_rx_policy`) **plus** `0x3FF7_5020 |= 0x9800_0000` (`hal_enable_sta_tsf`).
2. Read ESP32 MAC RX hardware drop/decrypt-error counters (`0x3FF7_3xxx` `hal_mac_rx.o` statistics registers) in `[DHCP-WAIT]` to pinpoint any LMAC RX filter discard reason.

### Step 4: Live Certification, Diagnostic Cleanup, & Phases 3D–3E

1. **Live Silicon Certification**:
   - Confirm UART0 logs `[KLIPPER-CAM-RS] [NET] DHCP IP Configured: 172.22.1.X/24`.
   - Verify HTTP endpoints via `eno1`:
     - `curl --interface eno1 -fsS http://172.22.1.X/status`
     - `curl --interface eno1 -fsS http://172.22.1.X/snapshot` (`1280x720` JPEG)
     - `curl --interface eno1 -fsS http://172.22.1.X:81/stream`
   - Verify passive serial invariant (`TIOCMGET = 0x0000`) via `cargo --locked xtask ensure-serial-passive --port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`.
2. **Diagnostic Scaffolding Cleanup**:
   - Strip temporary `[DHCP-WAIT]` diagnostic scaffolding (`take_rx_diag_snapshot`, `last_connected_rx_info`, `data_tx_primary_ok`/`fallback_ok`, `rx_protected_seen`) from `smoltcp_phy.rs`, `sta_fsm.rs`, and `klipper-cam-fw/src/main.rs`.
3. **CI Gates & Subsequent Phases**:
   - Run all 10 CI gates (`cargo fmt --all -- --check`, host clippy, host tests, Miri tests, Xtensa clippy, Xtensa build, `espflash save-image` size check `<= 0x1B0000`, `cargo test -p xtask`, `cargo xtask verify-invariants`, `cargo xtask verify-pure-rust --allow-phase1`).
   - Proceed to **Phase 3D** (`crates/esp32-cam-phy`, retiring `libphy.a` + seam) and **Phase 3E** (`EspAppDesc` cutover, dropping `--allow-phase1`, enforcing `0` C/Assembly DWARF CUs).
