# Findings: `klipper-cam-fw` HTTP, OTA, Wi-Fi (FW-NET)

- **Baseline commit:** `5a21ce2`
- **Scope:**
  - `crates/klipper-cam-fw/src/{http.rs, ota.rs, config.rs, ap_select.rs}`
  - `main.rs`: the network device, TCP enqueue, Wi-Fi glue, the HTTP and OTA dispatch inside `run_post_wifi_server_loop`, and roam/reconnect
- **Line numbers** refer to the baseline commit. Confirm them with `grep -n` before you edit.

## Summary

The HTTP parser is in better shape than most bare-metal camera servers. It rejects Transfer-Encoding, duplicate headers and whitespace in header names. The Port 81 send path is also solid: it queues the part header and the JPEG straight from PSRAM with a 91 KB in-flight cap, and every socket has a deadline.

The serious problems are in the OTA and Wi-Fi glue code:
- **Fake rollback state.** The rollback state is built from the MMU, so cancelling rollback never writes to flash.
- **Premature rollback cancel.** The static-IP path cancels rollback before Wi-Fi has been proven to work.
- **Open-network fallback.** A failed WPA passphrase conversion silently falls back to an open network.
- **Public OTA token.** The publicly documented default OTA token is accepted.
- **Hard-coded address.** 172.22.1.52/24 is added and ARP-announced on every network.

Performance problems:
- **Snapshot copies.** `/snapshot` copies the JPEG three times.
- **OTA buffering.** OTA validation runs a byte-wise SHA-256 over 1.8 MB after Core 1 is paused, with the network unpolled. OTA also keeps a 1.8 MB buffer reserved for good and receives through an 8 KB window.
- **Blocking Wi-Fi calls.** Roam and reconnect block the whole loop.

---

### FW-NET-01: The OTA manager is built from the MMU at boot, not from otadata, so rollback-cancel never runs
- **Severity:** P0 if the shipped bootloader has rollback enabled, otherwise P1. **Category:** reliability
- **Location:** main.rs:487-506 (verified), main.rs:2376-2390, main.rs:2743
- **Evidence:**
  ```rust
  if drom_is_ota1 || irom_is_ota1 {
      klipper_cam_fw::OtaDataManager::from_sectors(
          klipper_cam_fw::OtaDataSelect::new(1, klipper_cam_fw::OtaImgState::Valid),
          klipper_cam_fw::OtaDataSelect::new(2, klipper_cam_fw::OtaImgState::Valid),
  ```
- **Problem:** Both records are always `Valid`, and the real otadata is never read at boot. `load_from_flash` is called only during OTA POST (main.rs:3756). So `handle_wifi_got_ip_rollback` never sees `PendingVerify`.
  - With rollback enabled, the next reset after an OTA reverts to the old firmware.
  - Without rollback, the feature does nothing.
- **Proposed fix:**
  - At boot (`capture_boot_dma_and_init_runtime`), load the manager from flash with `OtaDataManager::load_from_flash(&mut flash_adapter)`. This must run before Core 1 starts, or under the FW-CORE-01 pause guard.
  - Use the MMU slot only as a cross-check, and log a mismatch.
  - Keep `PendingVerify`, so the got-IP path persists `Valid`.
- **Effort:** S
- **Verification:**
  - Host test: a `MockNorFlash` whose sector 1 holds seq 2/New is loaded at boot. After got-IP, sector 1 reads `Valid`.
  - Hardware test: run an OTA and reboot twice. The device stays on the new slot.

### FW-NET-02: The static-IP path cancels rollback before Wi-Fi works
- **Severity:** P1. **Category:** reliability
- **Location:** main.rs:3396-3412
- **Evidence:** `handle_wifi_got_ip_rollback(runtime, &mut flash_adapter, fw_ip);` runs unconditionally on the first iteration.
- **Proposed fix:**
  - Gate the call on `wifi_controller.is_connected()` and a confirmed IP: a DHCP `Configured` event, or a static IP plus an ARP reply from the gateway.
  - Better still, gate it on the first HTTP request served.
- **Effort:** S
- **Verification:** A new image with a wrong SSID rolls back after a reset. This requires FW-NET-01 and a rollback-enabled bootloader.

### FW-NET-03: A WPA passphrase conversion error silently falls back to an Open network
- **Severity:** P1. **Category:** security
- **Location:** main.rs:2764-2772; config.rs:17-21
- **Evidence:**
  ```rust
  match pass.try_into() {
      Ok(pw) => AuthenticationMethodConfig::WpaWpa2Personal(pw),
      Err(_) => AuthenticationMethodConfig::Open,
  ```
- **Problem:** The device joins any open access point with the same SSID (an evil twin) and exposes OTA on it. An empty default password also selects Open.
- **Proposed fix:**
  - Make the `Err` branch fatal.
  - Add a compile-time assert that the password length is 0 or 8..=63.
  - Call `KlipperCamConfig::is_valid()` at boot.
  - Allow open networks only behind an explicit feature flag.
- **Effort:** S
- **Verification:** A build with a 70-character `WIFI_PASS` fails at compile time or at boot, and never associates with an Open network.

### FW-NET-04: The default OTA token is public and accepted
- **Severity:** P1. **Category:** security. **Duplicate of:** BUILD-07
- **Location:** config.rs:13-15, 85-97, 108-123; README.md:128; http.rs:1717 (`handle_ota_post`)
- **Evidence:** `pub const DEFAULT_OTA_TOKEN: &str = "klipper-cam-ota-2026";`. Neither `is_using_default_ota_token()` nor `is_default_ota_token()` is called anywhere.
- **Proposed fix:**
  - Return 403 from `handle_ota_post` when the default token is in use.
  - Add a `build.rs` that fails a release Xtensa build without `WIFI_OTA_TOKEN` (see BUILD-07).
  - Print a warning at boot.
  - Replace the README value with a placeholder.
- **Effort:** S
- **Verification:**
  - `curl -X POST -H "X-OTA-Token: klipper-cam-ota-2026" ...` returns 403.
  - A unit test in http.rs covers the 403.

### FW-NET-05: 172.22.1.52 is hard-coded, added to the interface and ARP-announced on every network
- **Severity:** P1. **Category:** correctness. **Merged with:** FW-CORE-22, FW-NET-15
- **Location:** main.rs:265, 1519, 3250, 3320, 3343-3369, 3383, 3396-3434 (verified)
- **Evidence:**
  ```rust
  addrs.push(IpCidr::new(IpAddress::v4(172, 22, 1, 52), prefix_len));
  send_arp_announce(&mut net_device, mac, [172, 22, 1, 52], [172, 22, 1, 1]);
  ```
- **Problem:** `NetworkConfig::use_dhcp` and `static_ip` are never read. Every 5 s the device sends an ARP claiming 172.22.1.52, which poisons neighbour caches on any network where another host owns that address.
- **Proposed fix:**
  - Pass `runtime.server.config.network` into the loop.
  - With DHCP, add only the leased address and router. With a static IP, use the configured IP, prefix and gateway.
  - Change the announcement to a true gratuitous ARP (target IP equals source IP).
  - Delete all the literals.
- **Effort:** M
- **Verification:** On a 192.168.x network using DHCP, Wireshark shows no ARP for 172.22.1.52.

### FW-NET-06: `/snapshot` copies the JPEG three times and reserves 512 KB of PSRAM
- **Severity:** P1. **Category:** performance. **Merged with:** FW-CORE-07
- **Location:** http.rs:1322-1336; main.rs:2558-2561; main.rs:3839-3847
- **Problem:** Each copy costs about 5-12 ms of PSRAM time. A frame of 131,072 bytes minus the roughly 170-byte header triggers `BufferTooSmall`, and the connection then closes with no response.
- **Proposed fix:** Use a header-only buffer, send the body zero-copy from a pinned slot, and shrink the response buffers to 8 KB (frees about 480 KB of PSRAM).
- **Effort:** M
- **Verification:**
  - Run `curl -w '%{time_total}'` 100 times while streaming.
  - A host test confirms the body is byte-identical to the frame.

### FW-NET-07: OTA validation is a byte-wise SHA-256 over 1.8 MB, run after the Core 1 pause with no network polling
- **Severity:** P1. **Category:** performance
- **Location:** ota.rs:1339-1351, 1023, 1586-1589; main.rs:3750-3799
- **Evidence:**
  ```rust
  while i < data.len() {
      self.buffer[self.buf_len] = data[i];
      self.buf_len += 1;
  ```
- **Problem:** SHA-256 takes about 2-4 s, then erase plus write takes several more, all in one loop iteration. The 10 s stall watchdog on Port 81 drops viewers.
- **Proposed fix:**
  - Validate before pausing Core 1.
  - Hash whole 64-byte blocks, or use the ESP32 hardware SHA through esp-hal (first verify that the API exists in 1.2.2 `unstable`).
  - Call `poll_net_burst` between erase and write chunks.
- **Effort:** M
- **Verification:** Log timing for each phase. Core 1 pause time should be no longer than erase plus write, and the stream should not abort.

### FW-NET-08: Small unauthenticated `POST /update` requests pause Core 1 and read otadata before the token check
- **Severity:** P2. **Category:** security (denial of service)
- **Location:** main.rs:3750-3760
- **Proposed fix:** Before calling `pause_core1_camera_producer()`, require both a constant-time token match and a complete body. Otherwise reply 401/400 without pausing.
- **Effort:** S
- **Verification:** 200 bare POSTs while streaming leave FPS and `c1_dropped` unchanged.

### FW-NET-09: The pre-auth gate compares tokens with early-exit `PartialEq`
- **Severity:** P2. **Category:** security
- **Location:** main.rs:3619
- **Evidence:** `} else if parsed.ota_token != Some(runtime.server.ota_token()) {`
- **Proposed fix:** Use `!constant_time_eq(parsed.ota_token.unwrap_or(b""), runtime.server.ota_token())`.
- **Effort:** S
- **Verification:** `grep -n "ota_token !=" main.rs` returns nothing.

### FW-NET-10: OTA upload throughput is capped by the 8 KB receive window and one drain per loop iteration
- **Severity:** P2. **Category:** performance
- **Location:** main.rs:2546-2549; main.rs:3689-3701
- **Proposed fix:**
  - Run a bounded inner `recv` and `poll_net_burst` loop (about 20 ms) while this socket owns the OTA accumulator.
  - Alternatively, use 32 KB receive buffers taken from the PSRAM freed by FW-NET-06.
- **Effort:** S
- **Verification:** Time a 1.7 MB upload sent with `-H 'Expect:'`, before and after the change.

### FW-NET-11: 1.8 MB OTA accumulator permanently reserved (about 45% of PSRAM)
- **Severity:** P2. **Category:** performance / memory
- **Location:** main.rs:2568; ota.rs:924-969 (unused `begin_ota` / `write_chunk` / `finish_ota`)
- **Proposed fix:** Stream the image to flash:
  - Erase incrementally ahead of the write cursor.
  - Hash incrementally.
  - Commit otadata only after the digest matches.
  - Use a staging buffer of 4 KB or less.
- **Effort:** L
- **Verification:**
  - A host test with random chunk sizes produces identical flash contents.
  - A corrupted last byte leaves otadata untouched.
  - `free_psram` grows by about 1.8 MB.

### FW-NET-12: When otadata disagrees with the MMU slot, OTA falls back to a synthetic seq 1/2 and the commit is ineffective
- **Severity:** P2. **Category:** correctness
- **Location:** main.rs:3754-3761; ota.rs:1096-1110
- **Proposed fix:**
  - Return 500 on a load error or a slot mismatch.
  - Compute `current_max` from the data loaded from flash.
  - FW-NET-01 removes the synthetic manager.
- **Effort:** S
- **Verification:** Host test with sectors at seq 7/8 and ota_1 booted: the commit writes seq 9 into sector 0.

### FW-NET-13: Wi-Fi `block_on_with_timeout` freezes the loop; a reconnect can take about 44 s
- **Severity:** P2. **Category:** performance. **Merged with:** FW-CORE-10
- **Location:** main.rs:765-797, 1365-1381, 1527-1645, 3215, 3307-3308
- **Proposed fix:**
  - Pass an idle callback that runs the poll and the drain.
  - On reconnect, try the locked BSSID first, without rescanning.
- **Effort:** M
- **Verification:** During a forced roam, measure the maximum gap between polls and the `c1_dropped` delta.

### FW-NET-14: Each new `GET /stream` aborts the current viewer, while the tested library contract says 500
- **Severity:** P2. **Category:** correctness
- **Location:** main.rs:4186-4194; http.rs:1829-1847
- **Problem:** Two consumers, for example Mainsail and Moonraker, get into a reconnect storm.
- **Proposed fix:**
  - **Option A:** fan out the same frame to both sockets, each with its own `tx_off`.
  - **Option B:** preempt only an owner that has stalled for more than 2 s, and otherwise return 503.
  - In either case, make `handle_port81` and its tests match the chosen behaviour.
- **Effort:** M
- **Verification:** Two `curl` streams run for 60 s with steady data and no resets.

### FW-NET-15: The DHCP address can be lost; the 2-address interface is full and the lease is never removed
- **Severity:** P2. **Category:** reliability. **Merged into:** WO-15 together with FW-NET-05
- **Location:** main.rs:3343-3361, 3375-3386
- **Proposed fix:**
  - On `Configured`, drop non-static addresses with `retain` and then push the new one.
  - On `Deconfigured`, remove the lease.
  - Log a failed `push`.
- **Effort:** S
- **Verification:** After a forced lease change, the device answers on the new address within one renewal.

### FW-NET-16: The roam weak-link timer ignores hysteresis and compares RSSI samples that are not comparable
- **Severity:** P2. **Category:** reliability
- **Location:** main.rs:3200; ap_select.rs:45, 348-372, 451-457
- **Proposed fix:**
  - Call `update_with_hysteresis(.., ROAM_RECOVERY_HYSTERESIS_DB)`.
  - In `roam_scan_decision`, compare against the current AP's averaged RSSI from the scan.
- **Effort:** S
- **Verification:** A host test with an alternating -66/-64 dBm sequence crosses the weak-link threshold only when hysteresis is used.

### FW-NET-17: The Port 80/81 per-socket state machines are inline, Xtensa-only and untested
- **Severity:** P2. **Category:** maintainability. **Merged with:** FW-CORE-16
- **Location:** main.rs:2886-4282; the error-enqueue block is duplicated 4× at 3598-3685
- **Proposed fix:**
  - Extract a host-testable `Port80Conn` behind a socket trait.
  - Add a single `queue_error(..)` helper.
- **Effort:** L
- **Verification:** `cargo test` covers slowloris, OTA promotion auth, 413, and a peer close in the middle of the body.

### FW-NET-18: OTA header validation gaps: no chip_id check, DROM entry points accepted, loose digest placement, no read-back
- **Severity:** P2. **Category:** security
- **Location:** ota.rs:1278-1279, 1606-1621, 1026-1051
- **Proposed fix:**
  - Require `chip_id == 0` and an entry point in IRAM or IROM.
  - Require `aligned_end + 32 == len` when a hash is appended.
  - Read back and re-hash before `commit_update`.
- **Effort:** M
- **Verification:**
  - Unit tests reject an S3 image and a misplaced digest.
  - A mock bit-flip during write leaves otadata unchanged.

### FW-NET-19: Each MJPEG part header is formatted through `core::fmt`
- **Severity:** P3. **Category:** performance / code size
- **Location:** http.rs:754-763; main.rs:4063-4064
- **Proposed fix:** Use a const prefix plus a small decimal writer, and reuse it for `Content-Length` everywhere.
- **Effort:** S
- **Verification:** `test_zero_copy_mjpeg_part_header` passes, and `.text` shrinks.

### FW-NET-20: Panel and OTA HTML are served uncompressed through `fmt` into a 128 KB buffer
- **Severity:** P3. **Category:** performance
- **Location:** http.rs:686-750, 1625-1636, 1644-1683
- **Proposed fix:** Serve pre-gzipped `&'static` bytes with a const header, using the zero-copy send.
- **Effort:** S
- **Verification:** `curl --compressed -sI .../panel` shows gzip at about 1.6 KB.

### FW-NET-21: Stale metadata: "4-5 FPS", LwIP, `X-Framerate: 5`, no `Cache-Control` on the stream
- **Severity:** P3
- **Location:** http.rs:50-66, 970
- **Proposed fix:**
  - Derive the advertised framerate from config, or remove it.
  - Fix the page text.
  - Add `Cache-Control: no-store`.
- **Effort:** S
- **Verification:** Update the tests that assert on the old values.

### FW-NET-22: Dead configuration fields and duplicated token comparators
- **Severity:** P3
- **Location:** config.rs:536-566, 648-680, 683-735, 108-123 vs 622-635
- **Proposed fix:**
  - Wire up the config fields or delete them.
  - Derive the socket-pool sizes from config.
  - Keep a single comparator, `verify_ota_token`.
- **Effort:** S
- **Verification:** Clippy is clean on host and Xtensa with no `allow(dead_code)`.

### FW-NET-23: Loose `?action=snapshot` detection; `Content-Length` accepts `+5`; `isize` content length
- **Severity:** P3
- **Location:** main.rs:3729-3734; http.rs:348-355, 1272-1274
- **Proposed fix:**
  - Add `query_has_pair(q, "action=snapshot")`.
  - Accept digits only in `Content-Length`.
  - Store the length as `Option<usize>`.
- **Effort:** S
- **Verification:** Unit tests for both cases.

### FW-NET-24: Silent clamp of the TX token length to 1480
- **Severity:** P3. **Merged with:** FW-CORE-18
- **Location:** main.rs:711-719
- **Effort:** S

### FW-NET-25: FIN is sent only after every byte is ACKed
- **Severity:** P3. **Category:** performance
- **Location:** main.rs:3564-3572
- **Proposed fix:** Call `close()` as soon as `tx_off == tx_len`. smoltcp sends the FIN after the pending data.
- **Effort:** S
- **Verification:** Compare the median `curl /status` time over 100 runs, before and after.

### FW-NET-26: No `100 Continue` reply, so `curl` uploads stall about 1 s
- **Severity:** P3
- **Location:** http.rs:1896; main.rs:3590-3600
- **Proposed fix:** After authentication, send `HTTP/1.1 100 Continue` when the request has an `Expect: 100-continue` header.
- **Effort:** S
- **Verification:** `curl -v` shows `100 Continue` immediately.

### FW-NET-27: `/control` changes camera state over GET with `ACAO: *` (CSRF)
- **Severity:** P3. **Category:** security
- **Location:** http.rs:1369-1603, 1606-1620
- **Proposed fix:**
  - Require POST, or a custom header such as `X-Requested-With`, for any request that changes state.
  - Drop CORS on changing requests.
- **Effort:** S
- **Verification:** A GET with `var=` returns 405.

### FW-NET-28: otadata helpers tie sectors to slots; seq comparison is inconsistent; CRC loop duplicated
- **Severity:** P3
- **Location:** ota.rs:871-882, 1096-1110, 321-345
- **Proposed fix:**
  - Delete `sync_slot_to_flash` and `sync_sector_to_flash`.
  - Use plain max comparison everywhere, as the bootloader does.
  - Give both CRC functions one shared core.
- **Effort:** S
- **Verification:** A new test checks that commits always land in the lower-seq sector.

### FW-NET-29: `rssi().unwrap_or(-50)` reports a healthy link when RSSI is unknown
- **Severity:** P3
- **Location:** main.rs:3779
- **Proposed fix:** Cache the RSSI from the roam check, and report `0` or `null` when it is unknown.
- **Effort:** S

---

## What looks good
- **HTTP parsing.** `parse_http_request` is strict: it rejects Transfer-Encoding, duplicate Content-Length or token headers, whitespace in header names, unknown versions and targets without a leading slash.
- **Token check.** `verify_ota_token` is branchless and constant-length. The 401 response carries no CORS headers.
- **OTA commit order.** otadata is committed only after validation and writing. The erase is bounded and feeds the watchdog. Flash writes go through a 1 KB internal-RAM chunk.
- **otadata format.** The layout and CRC match ESP-IDF. Segment bounds use `checked_add`.
- **Port 81 send path.** The header goes into a 512 B DRAM buffer, the JPEG is borrowed `&'static`, with 91 KB in flight, a 45 KB low-water mark, Nagle off and a 10 s stall abort.
- **Slowloris protection.** Header, OTA, TX-progress and close deadlines bound slow clients.
- **`ap_select.rs`.** It is pure and heap-free, uses saturating arithmetic and exponential backoff, and has 20 host tests.

## Open questions
1. **Bootloader rollback.** Is `firmware/bootloader_v554_gpio16.bin` built with `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE`? The answer decides between P0 and P1 for FW-NET-01.
2. **Self-test token.** The boot self-test (main.rs:316) posts OTA with `DEFAULT_OTA_TOKEN`, so it fails on custom-token builds. Is that intended?
3. **`max_burst_size = Some(12)`.** What effect does it have on the receive-window advertisement?
4. **Wi-Fi task starvation.** Does the busy Core 0 loop starve esp-radio tasks?
5. **Frame size limit.** Can UXGA at quality 4 produce JPEGs larger than the 128 KB slots?
6. **Secrets in flash.** Wi-Fi credentials and the token are stored in plaintext in flash. Is flash encryption planned?
