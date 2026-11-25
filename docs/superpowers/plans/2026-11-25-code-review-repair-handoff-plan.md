# Code Review and Repair Handoff Plan (2026-11-25)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Finish the whole-repository code review that was paused on 2026-11-25, then repair the defects it finds, starting with the post-`Connected` WPA2-CCMP data plane (zero protected RX, no DHCP lease).

**Architecture:** The review is split into five independent read-only workstreams (Task 1). Repairs follow test-driven development: pin each defect with a host test first, then fix, and run every silicon change as a single-variable A/B test with the owner's confirmation.

**Tech Stack:** Rust 1.99.1 `#![no_std]` workspace, Xtensa ESP32 (`esp` toolchain), smoltcp, host tests plus Miri.

**Spec:** `docs/superpowers/plans/2026-11-20-wifi-sta-wpa2-data-plane-handoff-plan.md` (current silicon status, register contracts, and the experiments already tried and ruled out — read its sections 2–5 first). Base plan: `docs/superpowers/plans/2026-11-14-rea-full-rust-firmware-plan.md`.

## Global Constraints

- Work on `master`, commit and push only when the owner asks, never force-push (`CLAUDE.md`).
- Run the host gates before claiming work is done: `cargo fmt --all -- --check`, `cargo clippy --workspace --all-targets --locked -- -D warnings`, `cargo test --workspace --locked`, `cargo deny check`, `cargo xtask verify-invariants --allow-missing-bootloader`, `cargo xtask verify-pure-rust --allow-phase1`.
- Never run flashing, serial or LAN commands (`espflash`, `cargo run --target xtensa-esp32-none-elf`, `cargo xtask ota-deploy`, `cargo xtask ensure-serial-passive`) without the owner's explicit confirmation.
- Change one variable per flash cycle. Do not repeat the 2026-11-20 handoff's attempts 1–16 (section 4) as they stand.
- Keep the GPIO16 PSRAM chip-select and Core 1 producer invariants. Do not alter `phy_switch_channel_20mhz` without a hardware A/B.
- Never log, print or commit Wi-Fi passphrases, `.env.local` or OTA tokens.
- Do not commit third-party reference sources. Cite them by upstream URL instead.

## Review Focus

1. **A protected unicast data frame from the AP (the DHCP OFFER).** It must reach smoltcp. Today it never reaches the RX ring (Finding A1).
2. **Our CCMP-encrypted QoS-data TX frame.** It must decrypt at the AP. The AP's 802.11 ACK only proves the PHY received it (Finding A3).
3. **A malformed or short 802.11, EAPOL, HTTP, DHCP or DNS frame from the air or the LAN.** It must be rejected without a panic, because a panic resets the device (Task 1 workstreams).
4. **Power loss during an OTA or KCFG write.** The device must boot the last good image (Task 1 workstream C).
5. **smoltcp asking for a TX buffer larger than the driver's staging buffer.** It must get an error, not an empty slice (Finding A4).

---

## 0. Where this session stopped (read first)

- **Repository state:** `master` == `origin/master` at `b061952` before this document's commit. The tree was clean.
- **Host gates run on `b061952` (Windows host, 2026-11-20):** `cargo fmt --all -- --check` passed and `cargo test --workspace --locked` passed with 0 failures. Clippy, `cargo deny`, Miri, the xtask verifiers and the Xtensa build were not run in this session.
- **Review status: incomplete.** Seven read-only reviewers were dispatched on 2026-11-20. All were cut off by usage limits before reporting, so none of their findings were recovered. Five replacement reviewers were dispatched on 2026-11-25 and then stopped at the owner's request ("hold the work"), also before reporting. **Treat every subsystem not listed under "Verified correct" in section 1 as unreviewed.**
- **The coordinator reviewed the following directly:** the RX path in `smoltcp_phy.rs`, the LMAC init / RX policy / key install / TX submit code in `lmac_mmio.rs`, `regs.rs`, the STA data-path code in `sta_fsm.rs` and `frames_80211.rs`, CCMP and PTK derivation in `esp32-cam-wpa2`, and the DHCP loop in `klipper-cam-fw/src/main.rs`. It also cross-checked the open-source ESP32 MAC reverse-engineering sources (section 2, "Reference sources").

---

## 1. Data-plane findings so far

The severities below are the coordinator's. Each "Verified" note says how the finding was checked.

### A1 — The missing protected frames are lost in hardware or on the air, not in the software filter (HIGH, diagnostic)

- `crates/esp32-cam-wifi-mac/src/smoltcp_phy.rs:516-533` (`Esp32StaWifiDriver::poll`) increments `rx_protected_total` (`rx_prot`) and records `last_connected_rx_info` (`last_data_rx`) **before** the `rx_state` / length validity filter at `:542-560`.
- The silicon logs show `rx_prot = 0` and `last_data_rx = None`, with `malf = 0`. So no data frame addressed to us, protected or not, ever leaves the RX DMA ring after `Connected`. No software filter is dropping them.
- Unprotected EAPOL M1 and M3 (unicast data to our MAC) did arrive before keys were installed. So the RA/BSSID filter in `LmacHal::set_associated_bssid` passes unicast data.
- Two things change at `Connected`:
  - `LmacHal::install_sta_ccmp_key` enables hardware crypto (`lmac_mmio.rs:1109-1153`).
  - The AP switches to protected frames, and only sends the DHCP OFFER if it could decrypt our DISCOVER.
- The UniFi `iot` SSID blocks LAN-to-WLAN broadcast and multicast (2026-11-20 handoff, section 2). So a unicast OFFER may be the only protected frame we should ever expect.
- **Verified:** by reading the code path.

### A2 — Hardware RX key programming matches the open-source reference (verified correct)

The repo's key-slot programming agrees with `esp32-open-mac/esp-wifi-hal`:

| Item | Repo | Reference |
|---|---|---|
| Key table | `0x3FF7_4400 + slot*0x28` (`regs.rs:465-470`) | 25 slots at `0x1400`, stride 40 |
| Word 1 fields | written at `lmac_mmio.rs:1114-1119` | ADDR[15:0], WEP_104 bit 16, ALGORITHM[20:18] (CCMP = 3), PAIRWISE bit 21, "unknown, always 1" bit 22, GROUP bit 23, INTERFACE_ID[25:24], BITS_256 bit 26, KEY_ID[31:30] |
| Pairwise `mode_bits` | `3<<21` (bits 21 and 22) | PAIRWISE + unknown bit |
| Group `mode_bits` | `6<<21` (bits 22 and 23) | unknown bit + GROUP |
| Key words | little-endian | little-endian |
| Key-valid mask | `0x3FF7_3814`, bit per slot | same |
| General crypto control | `0x3FF7_3810` | same |
| Interface crypto control | `0x3FF7_3800 + 4*vif` | same |
| VIF 0 value after install | `0x0003_0103` | init `0x0003_0000`; set bits `0x10103`; PMF_DISABLE bit 17 set when PMF is off; bit 28 set **only for WEP**; for CCMP without PMF this gives `0x0003_0103` |

The reference names bit 28 `aead_cipher`, but its own `is_aead()` returns true only for WEP. Do not "fix" the repo by setting bit 28 for CCMP.

The reference runs `setup_crypto` at init and writes `0x0003_0000` to all four interface registers. The repo writes it to VIF 0 and VIF 1 and writes 0 to VIF 2 and VIF 3 (`lmac_mmio.rs:911-922`). The difference is low risk and only worth noting.

### A3 — TX crypto programming disagrees with the reference (HIGH, top hypothesis; needs a hardware A/B)

The reference PLCP1 (per-slot TX register) field map:

| Field | Bits |
|---|---|
| `LEN` | 11:0 |
| `RATE` | 16:12 |
| `KEY_SLOT_ID` | 21:17 |
| `IS_80211_N` | 25 |
| `BANDWIDTH` | 28 |
| `INTERFACE_ID` | 31:30 |

The reference does **hardware** CCMP on TX (`examples/src/bin/crypto_tx.rs`):

1. Software writes the 8-byte CCMP header with the PN (`crypto_wrap(CryptoHeader::new(pn, key_id), MicState::Short)`).
2. Software leaves the payload in plaintext.
3. Software passes `key_slot_index = Some(slot)`, and that index goes into `KEY_SLOT_ID`.

The repo instead encrypts in software. For every frame with the Protected bit set, it ORs `0xC0 << 17` (`0x0180_0000`, bits 23–24) into PLCP1 (`lmac_mmio.rs:1458-1467`, constant at `:26`). That leaves `KEY_SLOT_ID = 0`, and slot 0 holds the GTK. Bits 23–24 are undocumented in the reference. "Bypass" is only an inference from `ieee80211_ccmp_encrypt` disassembly. With interface crypto enabled, the hardware may re-process or corrupt these frames. The AP would still ACK them, because ACKs are sent before decryption, and then silently drop them. That matches every symptom.

This is the 2026-11-20 handoff's Step 1B. The reference now gives a concrete frame and register contract for it.

**Before coding**, answer this from `esp-wifi-hal` and its `ieee80211` frame crate: what `MicState::Short` puts in the buffer (8 placeholder MIC bytes or none), and what `LEN` the reference programs (MPDU + MIC + FCS?).

### A4 — `StaTxToken::consume` violates the smoltcp contract (MEDIUM)

- `smoltcp_phy.rs:122-127` calls `f(&mut [])` when `len > smol_tx_buf.len()`. smoltcp writes `len` bytes into the slice it receives, so an empty slice causes an out-of-bounds panic, which is a device reset.
- It is unreachable today, because `capabilities()` reports MTU 1514 and the buffer holds 1536. It becomes a crash as soon as either value changes.
- **Fix:** make `consume` always run `f` on a buffer of length `len`. Either size the buffer to the MTU, or stage oversize frames in a fallback buffer and drop them after `f` returns. Assert `max_transmission_unit <= smol_tx_buf.len()` at compile time.
- **Verified:** by reading the code.

### A5 — Data TX fallback ends at 1 Mbps on an AP whose lowest basic rate is 11 Mbps (MEDIUM)

- `sta_fsm.rs:727-741` (`transmit_eth`) tries `Rate11mS`, then `Rate6m`, then `Rate1mL`.
- This is the 2026-11-20 handoff's Step 2, and it is still present on `b061952`.

### A6 — Only RFC 3610 CCM vectors; no full-MPDU 802.11 vector (MEDIUM, test gap)

- `esp32-cam-wpa2/src/crypto/ccm.rs:997` has RFC 3610 vector 1. No test pins `ccmp_encrypt_mpdu` / `ccmp_decrypt_mpdu` against an IEEE 802.11-2016 Annex J CCMP MPDU vector.
- The coordinator checked the AAD masks, nonce, PN byte order and CCMP header against clause 12.5.3 and found them correct. A published vector would stop regressions.

### A7 — Two separate CCMP AAD/nonce implementations (LOW)

- `ccm.rs:315` (`ccmp_aad_nonce`) and `ccm.rs:732` (`ccmp_build_aad`) plus `ccm.rs:704` (`ccmp_build_nonce`) duplicate the same logic.
- Keep one and test it with the Annex J vector, so the two cannot drift apart.

### A8 — The hardware-decrypted RX path trusts `rx_state == 0` (LOW, security note)

- `frames_80211.rs:966-1001` takes a protected frame as already decrypted by hardware if plaintext LLC/SNAP appears after the CCMP header. It then skips software MIC verification.
- That is safe only if the hardware drops MIC failures or reports them in `rx_state`. Confirm this against the reference RX crypto example (`examples/src/bin/crypto_rx.rs`) when that path is first exercised on silicon.

### Verified correct (do not re-audit)

- **CCMP construction (IEEE 802.11-2016 clause 12.5.3)**, checked in `esp32-cam-wpa2/src/crypto/ccm.rs`:
  - AAD: FC subtype bits 4–6 masked, Retry/PwrMgt/MoreData masked, Protected forced on, Order masked for QoS, SC reduced to the fragment number, QC reduced to the TID.
  - Nonce: TID, A2, PN5..PN0.
  - CCMP header: `PN0, PN1, 0, 0x20|keyid<<6, PN2..PN5`.
- **PTK derivation** (`kdf.rs:192-232`): PRF-512 over min/max addresses and nonces; KCK, KEK and TK at bytes 0/16/32; the TK reaches the key table from `pi.ptk_key[..16]` (`handshake.rs:632`). The AP accepting our M2 and M4 MICs independently confirms the KCK.
- **The Power-Management bit is never set.** The only Null-data frame is the keepalive, which is built with `pwr_mgmt = false` (`sta_fsm.rs:625-631`). So AP-side buffering is not the cause.
- **M4 goes out unencrypted on the first handshake.** `ptk` is `None` when M4 is built, and the PTK is installed after the M4 TX completes (`sta_fsm.rs:1244-1269`, `:1335-1356`).
- **Association Request** (`frames_80211.rs:626-697`): capability `0x0431` (ESS, Privacy, Short Preamble, Short Slot), rates `1/2/5.5/11 (basic), 6/9/12/18` plus extended `24/36/48/54`, RSN IE, and WMM info IE `DD 07 00 50 F2 02 00 01 00`. No HT capabilities are advertised, so the AP will not send A-MPDUs.

---

## 2. Remaining review (not started or not reported)

Run these as five parallel **read-only** reviewers. The prompts used on 2026-11-25 are summarised below; each needs about 45 tool calls. Give every reviewer section 1 of this document, so it does not re-audit what is already verified. Each reviewer must report numbered findings with:

- severity;
- `path:line`;
- evidence;
- confidence;
- the fix;
- a pinning test;

followed by "Checked and found correct" and "Not reviewed".

- **A. Wi-Fi MAC hardware layer.** `lmac_mmio.rs`, `regs.rs`, `dma_ring.rs`, `phy_bridge.rs`, `smoltcp_phy.rs`.
  - Compare the RX filter, scan/promiscuous, RX DMA (base/next/last/reload) and TX done/error registers against `esp-wifi-hal` `src/ll.rs` (`setup_filtering`, `set_filter_*`, `set_bssid_check_enable`, `set_scanning_mode_parameters`, `reload_hw_rx_descriptors`, `process_tx_status`) and the PAC patch YAML (`common_patches/wifi/mac/{rx,rx_filter_masked,tx,mac_interrupt}.yaml`).
  - Check RX buffer sizing against the worst-case frame (28 + 26 + 8 + 1500 + 8 + 4 bytes).
  - Check whether calling `reload_rx_dma` after every recycled descriptor is correct.
  - Check the 50,000-poll synchronous TX wait, DMA aliasing and ordering, and dual-core races.
- **B. STA FSM, frames and WPA2 state machine.** `sta_fsm.rs`, `frames_80211.rs`, `rate_ctrl.rs`, `ap_fsm.rs`, `tests/sta_e2e.rs`, and `esp32-cam-wpa2` (`handshake.rs`, `ie.rs`, PBKDF2/HMAC/AES/key-unwrap, `ffi.rs`, `crypto_ffi.rs`, `stubs.rs`).
  - M2 RSN IE byte-equality with the Association Request.
  - Retransmitted M1/M3 and the group-key handshake after `Connected`.
  - The single shared sequence counter (`next_seq`, `sta_fsm.rs:1505`).
  - Whether retransmissions reuse the same PN and sequence number.
  - Panics on short EAPOL or 802.11 frames.
  - Whether the tests are only self-consistent (own builder fed to own parser) rather than using published vectors (IEEE 802.11 Annex J PSK/PRF/CCMP, RFC 3394 key unwrap, RFC 2202 HMAC).
- **C. Bootloader and OTA format.** `crates/esp32-cam-bootloader/*`, `xtask/src/elf_wrap.rs`, `xtask/src/sha256.rs`, and the OTADATA, mark-valid and rollback parts of `klipper-cam-fw/src/ota.rs`.
  - Partition table, OTADATA entry and CRC, the rollback state machine, power loss between erase and write.
  - Image header, checksum and SHA-256 coverage; MMU mapping for both CPUs.
  - Decode of the GPIO16 IO_MUX value `0x2B00`.
  - Every ROM address in `rom_boundary.rs` against ESP-IDF `components/esp_rom/esp32/ld/esp32.rom.ld`.
- **D. Firmware application.** `crates/klipper-cam-fw/src/*`: the main loop and DHCP bring-up (`main.rs` ~4860–5450), HTTP parsing and authentication on mutating endpoints, OTA flash writes while the Core 1 producer runs, KCFG provisioning power-loss safety, captive DNS/DHCP parsers, cross-core SPSC ordering. Also check which findings from `docs/workorders/2026-11-10-perf-review/` are still open.
- **E. Camera pipeline, RF/clock, tooling and CI.**
  - `esp32-cam-ll`, `-hal`, `-sccb`, `-sensors`, `-conversions`, `-types`: JPEG SOI/EOI split across DMA buffers, OV2640 720p tables.
  - `esp32-cam-rf-clock`: the varargs `printf` shim soundness, NVS stubs.
  - `esp-wifi-sys-esp32-seam/build.rs`.
  - `xtask` verifiers that pass vacuously.
  - The CI workflow, the release profile (`overflow-checks`), and fuzz target coverage of network-facing parsers.

**Reference sources.** These are untrusted third-party code: read them for facts, do not commit them.

- https://github.com/esp32-open-mac/esp-wifi-hal — `src/ll.rs` (`set_plcp1` ~L1240, `setup_crypto` ~L1318, `set_key_slot_parameters` ~L1393, `set_interface_crypto_parameters` ~L1445), `src/crypto.rs` (`is_aead` ~L144), `src/async_driver.rs` (key install ~L240–300), `examples/src/bin/crypto_{tx,rx}.rs`.
- The esp32-open-mac PAC patches: `common_patches/wifi/mac/{crypto,rx,rx_filter_masked,tx,mac_interrupt}.yaml` and `esp32/wifi.yaml` (`CRYPTO_CONTROL` at offset `0x800`, `CRYPTO_KEY_SLOT` at `0x1400`).
- https://github.com/esp32-open-mac/esp32-open-mac — the C hardware layer.

Line numbers come from copies fetched on 2026-11-20 and may drift.

---

## 3. Repair tasks

### Task 1: Finish the review (five workstreams in section 2)

**Files:** read-only. Append the results to this document as section 5, "Review findings".

- [ ] **Step 1:** Dispatch reviewers A–E in parallel, read-only, each with section 1 of this document.
- [ ] **Step 2:** For each CRITICAL or HIGH finding, re-read the cited code yourself before recording it. Reviewers can be confidently wrong.
- [ ] **Step 3:** Add one repair task per confirmed defect below, in this document's task format.
- [ ] **Step 4:** Commit only the document update, when the owner asks.

### Task 2: Pin CCMP and the WPA2 KDFs with published vectors (host, TDD)

**Files:**
- Test: `crates/esp32-cam-wpa2/src/crypto/ccm.rs` (tests module)
- Test: `crates/esp32-cam-wpa2/src/kdf.rs` (tests module)
- Modify: `ccm.rs`, to remove the duplicate AAD/nonce code (finding A7)

**Interfaces:**
- Produces: a single `ccmp_build_aad(header: &[u8], out: &mut [u8; 30]) -> Result<usize, CcmpMpduError>` and a single `ccmp_build_nonce(ta: &[u8; 6], pn: u64, priority: u8) -> [u8; 13]`.
- `ccmp_aad_nonce` becomes a thin wrapper over them, or is removed and its callers updated.

Steps:

- [ ] **Step 1:** Add `test_ccmp_mpdu_ieee_annex_j_vector`. It encrypts the IEEE 802.11-2016 Annex J CCMP test MPDU with `ccmp_encrypt_mpdu` and asserts the published ciphertext and MIC byte-for-byte. It also decrypts the result with `ccmp_decrypt_mpdu` and asserts the plaintext, the PN and key id 0. Copy the vector exactly from the standard; do not generate it with this crate.
- [ ] **Step 2:** Add `test_pbkdf2_ieee_annex_j_psk_vectors` (for example passphrase `"password"` with SSID `"IEEE"`, and the other Annex J PSK cases), asserting the published 32-byte PMKs.
- [ ] **Step 3:** Add `test_prf_sha1_ieee_annex_j_vectors`, asserting the published PRF outputs.
- [ ] **Step 4:** Run `cargo test -p esp32-cam-wpa2 --locked`. The new tests should pass, since the code was reviewed as correct. If one fails, treat it as a CRITICAL defect and debug it with superpowers:systematic-debugging.
- [ ] **Step 5:** Merge the duplicate AAD/nonce implementations and re-run the crate tests plus `cargo +nightly miri test -p esp32-cam-wpa2`.
- [ ] **Step 6:** Commit with the message `test(wpa2): pin CCMP/PBKDF2/PRF to IEEE Annex J vectors; dedupe CCMP AAD`.

### Task 3: Build PLCP1 from named fields and add a hardware-CCMP TX mode (host, TDD; silicon A/B in Task 6)

**Files:**
- Modify: `crates/esp32-cam-wifi-mac/src/regs.rs` (PLCP1 field constants)
- Modify: `crates/esp32-cam-wifi-mac/src/lmac_mmio.rs:1449-1468` (`submit_and_poll_tx`)
- Modify: `crates/esp32-cam-wifi-mac/src/frames_80211.rs` (`eth_to_80211_data`)
- Modify: `crates/esp32-cam-wifi-mac/src/sta_fsm.rs` (`transmit_eth`, `StaTxSink::send_mpdu`)
- Test: the `lmac_mmio.rs` and `frames_80211.rs` tests modules

**Interfaces:**
- Produces in `regs.rs`:
  - `TX_PLCP1_LEN_MASK = 0xFFF`
  - `TX_PLCP1_RATE_SHIFT = 12`
  - `TX_PLCP1_KEY_SLOT_SHIFT = 17`
  - `TX_PLCP1_KEY_SLOT_MASK = 0x1F << 17`
  - `TX_PLCP1_IS_HT = 1 << 25`
  - `TX_PLCP1_BANDWIDTH_BIT = 1 << 28` (existing)
  - `TX_PLCP1_INTERFACE_SHIFT = 30`
- Produces a TX crypto mode: `pub enum TxCrypto { None, SoftwareBypass, Hardware { key_slot: u8 } }`, passed through `send_mpdu`.
  - `SoftwareBypass` keeps today's `0xC0 << 17` behaviour, so the two modes can be A/B tested.
  - `Hardware` writes `key_slot` into `KEY_SLOT_ID` and sets the encrypt-enable bit that the vendor `tx_info[16] = 0x40 | slot` implies (bit 23). Confirm bit 23 from the `ieee80211_ccmp_encrypt` / `mac_tx_set_plcp1` disassembly first.
- Produces `eth_to_80211_data_hw_ccmp(out, eth, bssid, seq, key_id, pn) -> Result<usize, FrameError>`. It writes the QoS header with Protected set, the 8-byte CCMP header and the plaintext LLC/SNAP payload, plus the MIC placeholder exactly as `esp-wifi-hal`'s `MicState::Short` does. Resolve that question first (finding A3).

Steps:

- [ ] **Step 1:** Write `test_plcp1_fields_match_reference_layout`. For rate `Rate11mS`, an MPDU of length 100, `TxCrypto::Hardware { key_slot: 4 }` and interface 0, assert that the written PLCP1 equals `(104 & 0xFFF) | (0x07 << 12) | (4 << 17) | <encrypt-enable bit> | (1 << 28)`. Run it and confirm it fails.
- [ ] **Step 2:** Write `test_plcp1_unprotected_frame_has_no_crypto_bits`: an unprotected management frame has bits 17–24 clear.
- [ ] **Step 3:** Write `test_hw_ccmp_frame_layout`. Assert the header bytes `88 41`, the CCMP header bytes for PN 1 and key id 0 (`01 00 00 20 00 00 00 00`), the plaintext `AA AA 03 00 00 00` followed by the EtherType, and the total length.
- [ ] **Step 4:** Implement the code to make the tests pass. Keep `SoftwareBypass` as the default until Task 6 picks a winner on silicon.
- [ ] **Step 5:** Run `cargo test -p esp32-cam-wifi-mac --locked`, then `cargo clippy --workspace --all-targets --locked -- -D warnings`.
- [ ] **Step 6:** Commit with the message `feat(wifi-mac): named PLCP1 fields and selectable HW-CCMP TX mode`.

### Task 4: Harden the data TX rate table (2026-11-20 handoff Step 2)

**Files:**
- Modify: `crates/esp32-cam-wifi-mac/src/sta_fsm.rs:727-741`
- Modify: `crates/esp32-cam-wifi-mac/src/lmac_mmio.rs:1463-1467` (bandwidth bit)

Steps:

- [ ] **Step 1:** Write `test_transmit_eth_never_falls_back_below_11mbps`. With a mock sink that fails every send, record the attempted rates and assert they are exactly `[Rate11mL, Rate11mS, Rate6m]`.
- [ ] **Step 2:** Write `test_plcp1_bandwidth_bit_cleared_only_for_ofdm`.
- [ ] **Step 3:** Change the fallback chain and clear `TX_PLCP1_BANDWIDTH_BIT` only when `rate.is_ofdm()`. This needs a new `WifiPhyRate::is_ofdm()` covering `0x08..=0x0F`.
- [ ] **Step 4:** Run the crate tests and commit with the message `fix(wifi-mac): data TX fallback stays at or above 11 Mbps basic rate`.

### Task 5: Fix the smoltcp TX token contract (finding A4)

**Files:**
- Modify: `crates/esp32-cam-wifi-mac/src/smoltcp_phy.rs:121-131`, `:921-927`

Steps:

- [ ] **Step 1:** Write `test_tx_token_oversize_len_gets_full_length_buffer`. Call `consume(2000, |b| b.len())` and assert it returns 2000 without panicking, and that `pending_eth_tx_failures` increments.
- [ ] **Step 2:** Implement the fix with a static scratch buffer for oversize frames (the frame is dropped after `f` returns). Add `const _: () = assert!(MTU <= SMOL_TX_BUF_LEN);`.
- [ ] **Step 3:** Run the crate tests, then Miri for `esp32-cam-wifi-mac`, and commit with the message `fix(wifi-mac): smoltcp TxToken always yields a len-sized buffer`.

### Task 6: Silicon experiments to find the dead data plane (needs the owner's confirmation before each flash)

Run in this order and stop at the first one that resolves the problem. Record the `[DHCP-WAIT]` line (`rx_prot`, `last_data_rx`, `pri_ok`, `fb_ok`) for each.

- [ ] **E1 — Over-the-air capture (2026-11-20 handoff Step 1A).** If a second adapter can run in monitor mode, capture one boot on channel 1. Check that our QoS-data DISCOVER decrypts under the session PTK, and whether the AP sends an encrypted OFFER. Decrypting needs the PTK, so derive it on the host from the capture's handshake; never print the passphrase.
- [ ] **E2 — Hardware-CCMP TX** (Task 3, `TxCrypto::Hardware { key_slot: 4 }`). This is the single variable.
- [ ] **E3 — RX with no hardware keys.** Skip `on_install_ccmp_key`, so `0x3FF7_3800` stays `0x0003_0000` and the key-valid mask stays 0, and rely on software CCMP decryption. If `rx_prot` becomes non-zero, the hardware RX crypto setup is dropping frames.
- [ ] **E4 — Linux hostapd test AP**, with WPA2-PSK/CCMP and `-dd` logging. Associate the ESP32 to it and check the hostapd/kernel logs for CCMP decryption or MIC failures on our frames. This answers "does our TX decrypt" without a sniffer.
- [ ] **E5 — MAC RX error counters.** Read them during `[DHCP-WAIT]` (2026-11-20 handoff Step 3), using the register names from the reference PAC `rx.yaml`.

### Task 7: Certify, then clean up (2026-11-20 handoff Step 4)

- [ ] **Step 1:** Run live DHCP and the HTTP `/status`, `/snapshot` and `:81/stream` checks from the 2026-11-20 handoff's Step 4.
- [ ] **Step 2:** Strip the `[DHCP-WAIT]` diagnostic scaffolding: `take_rx_diag_snapshot`, `extra_diag_counts`, `last_connected_rx_info`, `rx_protected_total`, and the counters in `sta_fsm.rs` and `main.rs`.
- [ ] **Step 3:** Run all host gates, Miri, the Xtensa clippy and build, and the size gate (`<= 0x1B0000`).
- [ ] **Step 4:** Only then start Phase 3D (PHY crate) and Phase 3E.

---

## 4. Execution notes for the next agent

- The open review (Task 1) is independent of Tasks 2–5. Tasks 2, 4 and 5 are host-only and independent of each other. Task 3 must land before E2 in Task 6.
- Running from a nested `.claude/worktrees/` checkout doubles the Xtensa linker flags. Set `RUSTFLAGS="-C link-arg=-nostartfiles -C link-arg=-Tlinkall.x"` for firmware builds there (`CLAUDE.md`).
- Previous reviewers were lost to usage limits twice. Give each reviewer an explicit tool-call budget and ask it to report partial results rather than run until it is cut off.
