# ESP32-WROVER Pure-Rust Firmware Migration — Comprehensive Handoff Plan

- **Date**: `2026-11-16`
- **Repository**: `esp32-camera` (`/home/console-admin/Github/esp32-camera`, branch `master`)
- **Base Plan**: `docs/superpowers/plans/2026-11-14-rea-full-rust-firmware-plan.md`
- **Handoff Status**: Phases `0–3B` `[EXECUTED & VERIFIED]`; Phase `3C` (`esp32-cam-wifi-mac`) passes host, Miri, Xtensa, and REA static gates and is paused at `/3strikes` during live silicon association with verified Xtensa disassembly root causes isolated below; Phases `3D` (`libphy.a`) and `3E` (`esp-bootloader-esp-idf` retirement) are queued for completion by the receiving agent.

---

## 1. Phase & Architecture Status Matrix (`13` Workspace Crates)

| Phase / Workstream | Scope & Crates | Status | Verified Metrics & Notes |
|---|---|---|---|
| **Phase 0** (`REA Baseline`) | `rea` evidence & ELF symbol attribution | `[EXECUTED & VERIFIED]` | Baseline `1,928 / 2,328` symbols attributed; `phase0_sta_reachability.json` |
| **Phase 1** (`Host Tooling`) | `crates/xtask` (replaces 7 Python/Shell scripts) | `[EXECUTED & VERIFIED]` | Removed `scripts/*` and `tests/*.py`; `cargo test -p xtask` passing |
| **Phase 2A–2C** (`Bootloader`) | `crates/esp32-cam-bootloader` (`0x1000`) | `[EXECUTED & VERIFIED]` | `15,904` B <= `28,672` B (`0x7000`), `0` `"GCC:"` strings, GPIO 16 pull-up (`0x3FF4_904C = 0x2B00`), 32-bit `s32i` IRAM loader (`mmu_jump.rs`), live `ota_0`/`ota_1` + OTADATA rollback Drill 3 certified; legacy `firmware/bootloader_v554_gpio16.bin` deleted |
| **Phase 3A** (`RF/Clock & Seam`) | `crates/esp32-cam-rf-clock`, `crates/esp-wifi-sys-esp32-seam` | `[EXECUTED & VERIFIED]` | Retired `10 / 14` Espressif C archives (`libbtdm_app.a`, `libcoexist.a`, `libcore.a`, `libespnow.a`, `libmesh.a`, `libprintf.a`, `libregulatory.a`, `librtc.a`, `libsmartconfig.a`, `libwapi.a`) |
| **Phase 3B** (`WPA2 Supplicant`) | `crates/esp32-cam-wpa2` | `[EXECUTED & VERIFIED]` | Retired `libwpa_supplicant.a`; live STA association, `1280x720` `/snapshot` & `:81/stream` certified in Attempt 53 |
| **Phase 3C** (`Wi-Fi MAC & LMAC`) | `crates/esp32-cam-wifi-mac`, `klipper-cam-fw` | `[HALTED AT /3strikes — ROOT CAUSES ISOLATED]` | Retired `libnet80211.a`, `libpp.a`, `esp-radio`, `esp-rtos`; `586` host tests, `129` Miri tests, `klipper-cam-fw.bin` `471,408` B (`26.64%`), `5` non-Rust DWARF CUs left (`phy*` + `esp_rom_spiflash.c`); live `172.22.1.52` association blocked on `TxAckFailed(4)` + swapped `lldesc_t` fields |
| **Phase 3D** (`PHY Crate`) | `crates/esp32-cam-phy` (retire `libphy.a` + seam) | `[QUEUED]` | Replace `phy_chip_v7.c`, `phy.c`, `phy_chip_v7_cal.c`, `phy_chip_v7_ana.c` |
| **Phase 3E** (`App Desc & Cutover`) | `crates/klipper-cam-fw`, `crates/xtask`, CI | `[QUEUED]` | Retire `esp-bootloader-esp-idf::esp_app_desc!()`, remove `--allow-phase1`, enforce `0` C/Assembly DWARF CUs |

---

## 2. Target Hardware, Network & Toolchain Ground Truth

### Hardware & Flash Layout (`ESP32-WROVER-DEV`)
- **MCU / Silicon**: `esp32` revision `v3.0`, 40 MHz crystal, 4 MiB DIO Flash, 4 MiB PSRAM mapped at `0x3F80_0000` (`40 MHz Flash / 40 MHz PSRAM`), OV2640 sensor on SCCB `0x30`, hardware MAC `d4:d4:da:5b:d4:ec`.
- **Serial Interface**: `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` (`-> /dev/ttyUSB0`).
  - **Passive Serial Invariant**: After every flash or serial inspect, enforce `TIOCMGET = 0x0000` (`DTR=0, RTS=0`, `~HUPCL`) via:
    `cargo xtask ensure-serial-passive --port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`
  - Host xHCI port `1-12` lacks `uhubctl` per-port VBUS switching.
- **Partitions (`partitions_rust_ota.csv`)**:
  - `0x1000`: Pure-Rust bootloader (`15,904` B already flashed and verified — do not re-flash unless bootloader code changes).
  - `0x8000`: Partition table (`partitions_rust_ota.bin`, already flashed).
  - `0x18000`: `otadata`.
  - `0x20000`: `ota_0` (`1,769,472` B / `0x1B0000` max).
  - `0x1D0000`: `ota_1`.
- **Board-Currently-Flashed State**: The ESP32 flash currently holds Attempt 60 firmware (`471,408` B). Fix `3C-1` (`TX_PLCP0_NO_ACK` in `regs.rs` & `lmac_mmio.rs`) is edited in the working tree (passes `cargo fmt` and `46` `esp32-cam-wifi-mac` lib unit tests) and has **not** yet been compiled or flashed to hardware.

### Network & Credentials
- **STA Config**: Static IP `172.22.1.52/24`, Gateway `172.22.1.1`, HTTP control port `80`, MJPEG stream port `81`, SSID `'iot'` (Channel 1, `bssid=02:ec:da:b7:c8:b4`).
- **Host Interface**: All ICMP/HTTP probes MUST bind explicitly to `eno1` (`172.22.1.52 via 10.0.0.1 dev eno1 src 10.0.0.90`):
  - `ping -I eno1 -c 5 -W 2 172.22.1.52`
  - `curl --interface eno1 -fsS http://172.22.1.52/status`
- **Build Secrets**: Wi-Fi credentials are read at compile time from gitignored `.env.local`; NEVER inspect or print `.env.local` or session `.ota_token` into logs or commits.

### Toolchain & MCP Conventions
- Always run plain `cargo xtask ...` (never invoke a hardcoded `~/.rustup/toolchains/stable-.../bin/cargo xtask ...`, which triggers `E0514` crate metadata mismatches).
- Invoke Xtensa builds via `cargo +esp clippy` / `cargo +esp build`.
- In `rea` MCP tools:
  - `export_evidence_bundle` uses `path` + `overwrite: true` (not `output_path`).
  - `evaluate_reconstruction_coverage` expects `coverage` as an unquoted JSON object.

---

## 3. Verified Phase 3C Root Causes & Exact Remediation Steps

Per **Rule 2 / Invariant 14 (`/3strikes`)** and **Rule 5 (one logical variable per flash/test cycle)**, execute Fixes `3C-1` through `3C-3` sequentially with UART0 verification if any step stops before `172.22.1.52` comes online. Local Ghidra/objdump disassembly in `.agents/rea_evidence/phase3c_wifi_mac_decomp/pp/` and ESP-IDF SDK headers are already extracted, OR run external cross-verification (e.g., `esp32-open-mac` / ESP32 LMAC TRM research) first if confirming register edge cases.

### Fix 3C-1: `TX_PLCP0` No-ACK Bits `[25:24] = 3` (`0x0300_0000`) — `[SOURCE EDITED IN TREE; NEEDS UNIT TEST & SILICON TEST]`
- **Failure Observed (`Attempts 58–60`)**:
  - `connect('iot') start error: Sta(Hardware(TxAckFailed(4)))` on the initial Channel 1 broadcast Probe Request (`DA = FF:FF:FF:FF:FF:FF`, `Rate1mL`).
- **Disassembly Proof** (`.agents/rea_evidence/phase3c_wifi_mac_decomp/pp/hal_mac.o.objdump.txt` `<mac_tx_set_plcp0>` `0x58..0x90`, `hal_mac.o.ghidra.c` `L9-44`, `lmac.o.ghidra.c` `lmacProcessTxComplete`/`lmacProcessTxError`):
  - Bits `[25:24]` of `TX_PLCP0_TRIG_REG` (`0x3FF7_3D20`):
    - `0` (`0x0000_0000`): Hardware RTS/CTS protection (`Wait CTS`) -> transmits RTS to broadcast `FF:FF:FF:FF:FF:FF`, times out waiting for CTS, and latches `TX_PMD_REG` (`0x3FF7_4270`) `[15:12] == 4`, subcode `0x00` (`lmacProcessCtsTimeout`), mapped by `lmac_mmio.rs` to `LmacError::TxAckFailed(4)`.
    - `1` (`0x0100_0000`): Unicast `Wait ACK`.
    - `2` (`0x0200_0000`): BlockAck.
    - `3` (`0x0300_0000`): Broadcast / Probe Request `No-ACK`.
- **Changes Already in Working Tree**:
  - `crates/esp32-cam-wifi-mac/src/regs.rs:255-256`: added `pub const TX_PLCP0_NO_ACK: u32 = 0x0300_0000;`.
  - `crates/esp32-cam-wifi-mac/src/lmac_mmio.rs:720-724`: changed `ack` when `wait_ack == false` from `0` to `regs::TX_PLCP0_NO_ACK`.
- **Remaining Code Action**:
  - Add unit test `test_submit_and_poll_tx_broadcast_sets_no_ack_bits` in `crates/esp32-cam-wifi-mac/src/lmac_mmio.rs` asserting `TX_PLCP0_NO_ACK | TX_PLCP0_MODE_LEGACY` on `0x3FF7_3D20` when `wait_ack == false`.

### Fix 3C-2: `lldesc_t` Swapped `size` `[11:0]` vs. `length` `[23:12]` & `sig_len` (`crates/esp32-cam-wifi-mac/src/dma_ring.rs`) — `[REQUIRED]`
- **Vendor SDK & Disassembly Proof** (`/home/console-admin/esp-matter-build/esp-idf/components/esp_rom/include/esp_rom_lldesc.h:28-41`, `esp32/rom/lldesc.h:74-93`, `.agents/rea_evidence/phase3c_wifi_mac_decomp/pp/wdev.o.ghidra.c:1370-1386, 1689-1707, 2195`):
  - `LLDESC_SIZE_MASK = 0x0000_0FFF` (bits `[11:0]` = buffer capacity `size`).
  - `LLDESC_LENGTH_MASK = 0x00FF_F000` (bits `[23:12]` = DMA frame `length`).
  - Currently `dma_ring.rs` has `[11:0]` and `[23:12]` swapped in BOTH RX and TX descriptors:
    - `WifiRxDmaDesc::arm(capacity)` (`L269-272`) sets `OWNER_HW | ((capacity & 0xFFF) << 12)`, leaving bits `[11:0]` (`size`) == `0`, so hardware RX DMA drops every incoming frame. Vendor `wDev_AppendRxBlocks` writes `0x8000_0000 | ((cap & 0xFFF) << 12) | (cap & 0xFFF)`.
    - `WifiTxDmaDesc` (`L295-355`) uses `CTRL_FLAGS = 0xE000_0000` (`sosf = 1`) and `(cap << 12) | len`, whereas single-buffer vendor TX sets `0xC000_0000 | ((len & 0xFFF) << 12) | (cap & 0xFFF)` (`owner=1, eof=1, sosf=0`).
    - `WifiRxRing::peek_received` (`L544-555`): hardware SLC DMA pads `desc_len` up to 4-byte boundaries; prefer unpadded `ctrl.sig_len` (`w6 & 0xFFF`) whenever `ctrl.sig_len > 0` (`(ctrl.sig_len as usize).min(BUF_LEN.saturating_sub(RX_CTRL_LEN))`) so trailing SLC padding never corrupts CCMP MIC checks, keeping `desc_len - RX_CTRL_LEN` fallback when `ctrl.sig_len == 0` in mock unit tests.
  - Note: `RX_CTRL_LEN = 28` (`0x1C`) IS already verified exact against `wDev_ProcessRxSucData` (`puVar9 = (ushort *)((int)puVar6 + uVar8 + 0x1c)`).
- **Exact Edits (`crates/esp32-cam-wifi-mac/src/dma_ring.rs`)**:
  1. `WifiRxDmaDesc` (`L243-273`): `payload_len()` -> `((self.word0 >> 12) & 0xFFF) as u16`; `buffer_cap()` -> `(self.word0 & 0xFFF) as u16`; `arm()` -> `let c = u32::from(capacity) & 0xFFF; self.word0 = Self::OWNER_HW | (c << 12) | c;`.
  2. `WifiTxDmaDesc` (`L306-355`): `CTRL_FLAGS = 0xC000_0000`; swap `payload_len()`/`buffer_cap()`; `prepare()` -> `self.ctrl = Self::CTRL_FLAGS | ((u32::from(len) & 0xFFF) << 12) | (u32::from(cap) & 0xFFF);`.
  3. `WifiRxRing::peek_received` (`L544-555`): prioritize `ctrl.sig_len > 0` over `desc_len - RX_CTRL_LEN`.
  4. Unit tests (`L796`, `L837-839`): update `test_rx_ring_lifecycle_and_recycling` (`d.word0 = WifiRxDmaDesc::EOF_BIT | (32 << 12) | (256 & 0xFFF);`) and add bit-position checks in `test_tx_slot_prepare_and_completion`.

### Fix 3C-3: RX Policy Bit `1`, Extended Supported Rates, Legacy Data Rate & `TX_HTSIG1/2` (Verify/Apply After 3C-1 + 3C-2)
- **`0x3FF7_30D8` Bit `1` (`lmac_mmio.rs:540-577`)**: `<hal_mac_rx_set_policy>` (`hal_mac.o.objdump.txt:1405-1513`) only toggles `0x110` (`RX_POLICY_PROMISC_BYPASS`) on `0x3FF7_30D8` and bit `16` (`0x0001_0000`) on `0x3FF7_3024`, never setting bit `1` (`RX_POLICY_BIT_BSSID_CHECK = 0x2`). Remove bit `1` writes in `set_scan_rx_policy` & `set_associated_bssid` and update `test_rx_policy_scan_vs_associated_bssid` (`L846-860`).
- **802.11g Extended Supported Rates & Non-HT Data Rate**:
  - `frames_80211.rs` (`build_probe_req` `L405-440` & `build_assoc_req` `L470-518`): append Extended Supported Rates IE Tag 50 (`[0x32, 0x04, 0x30, 0x48, 0x60, 0x6C]`) because `AssocReq` currently advertises Non-HT/Non-QoS (`0x0108`/`0x4108`) up to `18 Mbps`, while `RateController::new()` (`rate_ctrl.rs:349-355`) defaults to HT `WifiPhyRate::Mcs3Lgi` (`hw_code = 0x13`).
  - `rate_ctrl.rs`: default/cap data TX to legacy 802.11g OFDM (`WifiPhyRate::Rate24m`, index `6`, or add `Rate54m`) until Tag 45 HT Capabilities + QoS Data (`Subtype 0x8`) are added.
  - `lmac_mmio.rs:702-712` (`mac_tx_set_htsig` parity): write `0x0700_0000` in `TX_HTSIG1_REG` (`0x3FF7_4260`) and `(psdu_len & 0xFFFF) | 0x0005_0000` in `TX_HTSIG2_REG` (`0x3FF7_4264`).
- **Lower-Priority `hal_mac.o` / `phy_bridge.rs` Observations (Apply ONLY if hardware UART0 diagnostics implicate init/TX-power after 3C-1/2/3)**:
  - `mac_txrx_init`: `0x3FF7_3C54` clears all `[15:0]` (`& 0xFFFF_0000 | 0x0404` vs `0xFFFF_0003` in `lmac_mmio.rs:340`); `hal_init` spins on `0x3FF7_3D24 & 1` before asserting reset; `hal_mac_tx_set_cca` configures `0x3FF7_3C58` bits `[31:30]`; `phy_run_tx_pwctrl_bg()` (`tx_pwctrl_background`) is exposed in `phy_bridge.rs` but not invoked in `Esp32StaWifiDriver::init`.

### Phase 3C Verification & Live Silicon Certification Commands
1. **Static & Miri Gates (`x86_64`)**:
   - `cargo fmt --all -- --check`
   - `python3 /home/console-admin/.gemini/config/plugins/rust-pro/skills/rust-pro/scripts/google_rust_audit.py /home/console-admin/Github/esp32-camera/crates/esp32-cam-wifi-mac/src --strict`
   - `cargo clippy --workspace --all-targets --target x86_64-unknown-linux-gnu --locked -- -D warnings`
   - `cargo test --workspace --target x86_64-unknown-linux-gnu --locked`
   - `cargo +nightly miri test --target x86_64-unknown-linux-gnu -p esp32-cam-ll -p esp32-cam-conversions -p esp32-cam-types -p esp32-cam-bootloader -p esp32-cam-rf-clock -p esp32-cam-wpa2 -p esp32-cam-wifi-mac`
   - `cargo test -p xtask --target x86_64-unknown-linux-gnu --locked && cargo xtask verify-invariants && cargo xtask verify-pure-rust --allow-phase1 && cargo deny check`
2. **Xtensa Build & REA Parity**:
   - `cargo +esp clippy --target xtensa-esp32-none-elf -Zbuild-std=core,alloc --workspace --release --locked -- -D warnings`
   - `cargo +esp build --target xtensa-esp32-none-elf -Zbuild-std=core,alloc -p klipper-cam-fw --release --locked`
   - `espflash save-image --chip esp32 target/xtensa-esp32-none-elf/release/klipper-cam-fw target/xtensa-esp32-none-elf/release/klipper-cam-fw.bin`
   - Check DWARF CUs (`xtensa-esp32-elf-readelf -wi` -> `5` `.c`/`.S` CUs: `phy_chip_v7.c`, `phy.c`, `phy_chip_v7_cal.c`, `phy_chip_v7_ana.c`, `esp_rom_spiflash.c`; `0` `net80211`/`pp`/`wpa_supplicant`).
   - Run `rea:inspect_rust_binary`, `rea:export_evidence_bundle` -> `.agents/rea_evidence/phase3c_wifi_mac_post_bundle.json`, and `rea:evaluate_reconstruction_coverage` (`status: "ready"`), then update `phase3c_wifi_mac_parity.{json,_summary.md}` and `phase2c_bootloader_parity.{json,_summary.md}` (`mmu_jump.rs`, `15,904` B BIN, `1,434,544` B ELF).
3. **Physical Flash & Live `172.22.1.52` Certification**:
   - `/home/console-admin/.cargo/bin/espflash write-bin --chip esp32 --port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 --non-interactive 0x20000 target/xtensa-esp32-none-elf/release/klipper-cam-fw.bin && cargo xtask ensure-serial-passive --port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`
   - Wait `15s` (never use bare `sleep`), verify `ip route get 172.22.1.52` and `ping -I eno1 -c 5 -W 2 172.22.1.52` (`0%` loss).
   - Verify `curl --interface eno1 -fsS http://172.22.1.52/status` (`"active_partition":"ota_0"`, `"framesize":"1280x720"`, `"sccb_errors":0`, live `"wifi_rssi"`), valid `1280x720` JPEG on `/snapshot`, `>= 100` frames in `10s` on `:81/stream`, hardware reset (`espflash board-info` + `ensure-serial-passive` + `15s` soak + `ping`), and `cargo xtask verify-invariants --live-status-url http://172.22.1.52/status` (`8/8 [PASS]`).

---

## 4. Phase 3D Plan: Replace `libphy.a` (`crates/esp32-cam-phy`)
1. **REA Extraction & Obligation Ledger (`WO-REA-P3D-FACTS` / `WO-REA-P3D-LEDGER`)**:
   - Extract `phy_chip_v7.o`, `phy.o`, `phy_chip_v7_cal.o`, `phy_chip_v7_ana.o`, and Mask ROM `phy_get_romfuncs()` dispatch targets via `rea` / Ghidra `12.1.4` and `xtensa-esp32-elf-objdump`.
   - Record all analog/baseband/FE/I2C MMIO registers in `crates/xtask/src/verify_invariants.rs` (`AUTHORITATIVE_MMIO_REGISTERS`) and author `.agents/rea_evidence/phase3d_phy_obligations.json`.
2. **Clean-Room Rust `crates/esp32-cam-phy` (`#![no_std]`)**:
   - Implement every symbol currently imported in `crates/esp32-cam-wifi-mac/src/phy_bridge.rs`: `esp_phy_enable` / `register_chipv7_phy`, `phy_enable_low_rate` / `phy_disable_low_rate`, `chip_v7_set_chan` / `phy_change_channel_freq`, `bb_wdt_int_enable`, `tx_pwctrl_background`, and calibration state tables.
   - Retire `esp-phy` and `crates/esp-wifi-sys-esp32-seam` completely (`0` C static archives linked into `klipper-cam-fw`).
3. **Gates**: Pass all Miri, `google_rust_audit.py --strict`, Xtensa Clippy/build, `rea` coverage (`status: "ready"`), and live `172.22.1.52` streaming/reset certification.

---

## 5. Phase 3E Plan: Retire `esp-bootloader-esp-idf` & Enforce 100% Pure-Rust Gate
1. **Pure-Rust `EspAppDesc` (`crates/klipper-cam-fw`)**:
   - Remove `esp-bootloader-esp-idf` from `crates/klipper-cam-fw/Cargo.toml` and replace `esp_bootloader_esp_idf::esp_app_desc!()` in `crates/klipper-cam-fw/src/main.rs` with a clean-room `#[repr(C)]` `256`-byte `EspAppDesc` (`magic_word: 0xABCD_5432`) in `#[link_section = ".rodata_desc"]` matching `crates/esp32-cam-bootloader/src/image_verify.rs`.
2. **Zero-Exception `verify-pure-rust` & CI Cutover**:
   - Remove `--allow-phase1` allowances from `crates/xtask/src/verify_pure_rust.rs` and `.github/workflows/rust.yml`.
   - Verify `xtensa-esp32-elf-readelf -wi` reports `0` C/Assembly DWARF CUs, `0` `.a`/`.o` archives in link inputs, `100%` Linguist Rust across the workspace, and `cargo xtask verify-pure-rust` Exit `0`.
3. **Final Acceptance**: Full Gates `1–10`, live `172.22.1.52` HTTP/MJPEG soak, `ota_0 <-> ota_1` + OTADATA rollback re-verification, and cold-boot passive-serial verification.
