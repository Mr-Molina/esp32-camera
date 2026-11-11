# Findings: `klipper-cam-fw` core pipeline (FW-CORE)

- **Baseline commit:** `5a21ce2` (branch `master`)
- **Scope:** `crates/klipper-cam-fw/src/main.rs` (Core 1 producer, SPSC drain, server loop, capture fallback, TCP enqueue, statics, boot), `runtime.rs`, `spsc_ring.rs`, `frame_buffer.rs`
- **Line numbers** refer to the baseline commit. Re-locate them with `grep -n` before editing, because earlier work orders may shift lines.

## Summary

The dual-core split is a reasonable design. The SPSC ring's memory orderings are correct, and the two-phase pause/resume handshake is sound. The rest of `main.rs` (4,532 lines) relies on `static mut` aliasing and `&'static` lifetime laundering.

One integration bug is serious. In esp-storage 0.10, every flash erase or write is refused while APP_CPU is running. OTA and the post-OTA rollback-cancel are therefore broken whenever the Core 1 producer is active (FW-CORE-01). This was independently verified against `esp-storage-0.10.0/src/common.rs:119-121, 280-285`.

The three main performance limiters:

1. **The sensor clock is a probable hard cap (not yet measured).** The OV2640 at HD is set up with `CLKRC=0x00` and `R_DVP_SP=0x0C` on a 20 MHz XCLK, using the UXGA window. The measured 12.53 FPS is almost exactly 15 × 20/24, the OV2640's UXGA rate scaled down from a 24 MHz clock to the 20 MHz in use.
2. **Core 0 copies each frame several times through 40 MHz PSRAM.** It is set to 40 MHz (not 80), and each streamed frame crosses PSRAM about 4 times.
3. **Core 0 stalls make Core 1 drop whole frames.** Stalls come from `/snapshot` copies, multi-second `block_on_with_timeout` calls and blocking `println!`. Core 1 gives up after a spin count and drops the entire frame.

---

### FW-CORE-01: esp-storage refuses flash writes while Core 1 runs, so OTA and rollback-cancel fail
- **Severity:** P0. **Category:** correctness / reliability
- **Location:** main.rs:4425; main.rs:2378-2391 (called from 1519, 3365, 3412 while Core 1 is running); main.rs:3750-3760; Core 1 is started at main.rs:3156-3176
- **Evidence:**
  ```rust
  let flash_adapter = EspFlashAdapter(esp_storage::FlashStorage::new(peripherals.FLASH));
  ```
  ```rust
  if s0 == OtaImgState::PendingVerify || s1 == OtaImgState::PendingVerify {
      let _ = runtime.on_wifi_got_ip_with_flash(ip, flash_adapter);
  ```
  In `esp-storage-0.10.0/src/common.rs:119-121` the strategy is `multi_core => MultiCoreStrategy::Error`. Its `pre_write` contains `if is_running(other_cpu) { return Err(FlashStorageError::OtherCoreRunning); }`.
- **Problem:** `is_running(AppCpu)` in `esp-hal-1.2.2/src/soc/esp32/cpu_control.rs:41` returns false only when the core is clock-gated, RUNSTALLed or SW-stalled. A Core 1 parked in its IRAM spin loop by `pause_core1_camera_producer` still counts as running. Three consequences:
  - (a) Every `POST /update` erase and write fails with `OtherCoreRunning`, which is mapped to `FlashWriteError`. OTA is broken whenever `ov2640_dma_ok` is set, which is the normal case.
  - (b) After an OTA, `cancel_rollback_in_flash` (ota.rs:863-882) runs without pausing Core 1. It fails, the error is dropped by `let _`, and the image stays `PendingVerify`.
  - (c) `read_nor` is not guarded by the strategy, so SPI1 reads can overlap Core 1's flash-cache fetches.
- **Proposed fix:**
  1. Build the adapter with `unsafe { FlashStorage::new(..).multicore_ignore() }`. This is sound only together with step 2.
  2. Add an RAII `Core1PauseGuard`: `new()` calls `pause_core1_camera_producer()` and returns `Err(OtaError::FlashWriteError)` on timeout; `Drop` calls `resume_core1_camera_producer()`.
  3. Acquire the guard inside every `EspFlashAdapter::{read_aligned, erase_range, write_aligned}`, or once per OTA or cancel-rollback operation.
  4. Remove the ad-hoc pause at 3752.
  5. Make `handle_wifi_got_ip_rollback` log the error and retry, instead of discarding it with `let _`.
  6. Do not use `multicore_auto_park()`: it stalls Core 1 at an arbitrary instruction.
- **Effort:** S
- **Verification:**
  - HIL with the camera attached: `POST /update` returns 200 and the board reboots into the other slot. After two power cycles it stays on the new slot.
  - Host test: a mock `OtaFlashStorage` asserts that a "paused" flag is set during every erase and write.

### FW-CORE-02: Core 1 drain consumes the first half-buffer without checking DMA ownership (corrupt frames)
- **Severity:** P0. **Category:** correctness (frame data corruption)
- **Location:** main.rs:2218-2246 (Core 1); main.rs:1831-1838 (fallback)
- **Evidence:**
  ```rust
  for drained in 0..8u8 {
      if drained > 0 && desc_slice[2 * half_idx + 1].owner() {
          break;
      }
  ```
- **Problem:** The first half drained after each `IN_SUC_EOF` (`drained == 0`) is consumed unconditionally. The EOF bit is cleared at 2212, before the drain. If Core 1 runs late, it consumes half k+1 at `drained > 0`, and k+1's EOF sets the raw bit again. The next outer iteration then consumes half k+2 while the DMA is still filling it. That inserts zero-filled bytes, left by the `fill(0)`, into the frame. The SOI and EOI checks still pass, so a corrupt JPEG is committed. Core 1 runs late exactly under backpressure, when `push_spsc_bounded` is spinning.
- **Proposed fix:** Gate only on descriptor ownership:
  ```rust
  while !desc[2*h+1].owner() && n < 8 { consume; zero; re-arm; h = (h+1) & 7; n += 1 }
  ```
  Treat EOF only as a wake-up hint. Put this in one shared `drain_ready_halves(desc, dma, half_idx, sink)` used by both capture paths (see LL-15).
- **Effort:** S
- **Verification:**
  - Host test: halves 0 and 1 done, two EOF events, half 2 still DMA-owned; assert half 2 is not consumed.
  - HIL: inject a 300 µs delay per push and count zero runs of 64 bytes or more in committed frames over 1,000 frames. The expected count is 0.
- **Note:** this relies on DMA owner write-back; see LL Open question 1.

### FW-CORE-03: Aliased `&mut` to `FIRMWARE_RUNTIME` and to the PSRAM buffers (UB)
- **Severity:** P0 (UB; no miscompile observed). **Category:** safety
- **Location:** main.rs:1964 vs main.rs:2919 (both live across the calls at 3469 and 4049); main.rs:2914-2925 vs 2993-3062
- **Evidence:** `if let Some(runtime) = unsafe { FIRMWARE_RUNTIME.as_mut() } {` inside `capture_live_hd_jpeg_with_diag`, while `run_post_wifi_server_loop` holds another. Twelve `from_raw_parts_mut` re-borrows of `psram_bufs` fields.
- **Problem:** Two live `&mut` to the same static, and a second `&'static mut` to each socket buffer, are immediate UB. The code works only because of `#[inline(never)]`. LTO (WO-02) could break it. The lint is hidden by `allow(static_mut_refs)` at main.rs:20.
- **Proposed fix:**
  - Pass `camera: &mut CameraDriver<'static>` into `capture_live_hd_jpeg_with_diag` and delete its access to `FIRMWARE_RUNTIME`.
  - Take ownership with `let PsramBuffers { .. } = unsafe { PSRAM_BUFFERS.take() }.unwrap_or_else(|| quiesce_psram_and_software_reset());`. This deletes all twelve `from_raw_parts_mut` blocks.
  - Remove `static_mut_refs` from the allow list and fix the resulting errors.
- **Effort:** S
- **Verification:**
  - Xtensa clippy passes with `static_mut_refs` not allowed.
  - `grep -n from_raw_parts_mut main.rs` finds only the PSRAM carve.
  - HIL regression test.

### FW-CORE-04: No per-stage timing; the sensor clock is the probable FPS cap
- **Severity:** P1. **Category:** performance (diagnosis gap)
- **Location:** esp32-cam-sensors/src/ov2640.rs:1092-1098; main.rs:4303-4308 (XCLK 20 MHz); main.rs:4263-4279 (heartbeat)
- **Evidence:**
  ```rust
  let pclk_div = if (self.status.framesize as u8) > (FrameSize::Svga as u8) { 0x0c } else { 0x04 };
  self.write_reg(bus, Ov2640Bank::Sensor, CLKRC, 0x00)?;
  ```
  `frequency: Rate::from_mhz(20),`
- **Problem:** No pipeline stage is timed, and the heartbeat reports only counters. 720p is cropped from the UXGA window at 20 MHz XCLK with no clock doubler. 15 FPS × 20/24 = 12.5 FPS matches the measurement almost exactly. If the sensor emits one frame every ~80 ms, Core 0 optimisations cannot reach 18-22 FPS.
- **Proposed fix:**
  - On Core 1, timestamp with CCOUNT (`rsr.ccount`) at each DMA start and at each detected VSYNC end.
  - Keep EWMAs of `sensor_frame_period_us` and `frame_bytes` in `AtomicU32`s.
  - On Core 0, accumulate cycles spent in drain, enqueue, poll and request handling.
  - Expose all of these plus `p81_frames_sent` and `p81_frames_superseded` in `/status`.
  - Then run register experiments on `R_DVP_SP`, `CLKRC`, XCLK, VTS and dummy lines (see WO-05).
- **Effort:** S (instrumentation); M (sensor tuning)
- **Verification:** If `/status.sensor_frame_period_us` reads about 79,800, the sensor cap is confirmed. Per-stage ms/frame is the counter value divided by the frame count.

### FW-CORE-05: PSRAM runs at 40 MHz, not 80 MHz
- **Severity:** P1. **Category:** performance
- **Location:** main.rs:4445-4452
- **Evidence:** `cache_speed: PsramCacheSpeed::PsramCacheF40mS40m,` (esp-hal 1.2.2 also offers `PsramCacheF80mS80m`).
- **Problem:** Every PSRAM byte on the hot path costs about 2× what it would at 80 MHz. This was chosen for cold-boot stability (comment at 4433-4437), and `scripts/verify_cold_boot_invariants.py:205` enforces it.
- **Proposed fix:** A/B test `PsramCacheF80mS80m`. It requires flash at 80 MHz and a matching bootloader header (`espflash.toml` `[flash] frequency`). Add a boot-time PSRAM copy benchmark that prints MB/s. Keep 40 MHz as a cfg-feature fallback.
- **Effort:** S (code); M (validation)
- **Verification:** About 2× MB/s in the benchmark. 100 cold power cycles and a 24 h soak with no PSRAM verify failures and no rise in `c1_dropped`.

### FW-CORE-06: Streamed frames cross PSRAM about 4× on Core 0
- **Severity:** P1. **Category:** performance
- **Location:** main.rs:2833-2836 (SPSC to PSRAM write slot); main.rs:898 (`send_slice` from PSRAM into the PSRAM socket ring); main.rs:2563-2566 (port-81 TX rings in PSRAM); main.rs:712-719 (copy into the radio TX buffer)
- **Evidence:** `dst[asm.write_len..asm.write_len + chunk.len()].copy_from_slice(chunk);` / `match sock.send_slice(seg) {` / `let (tcp81_tx0, rem) = rem.split_at_mut(128 * 1024);`
- **Problem:** Each N-byte frame causes about 4N of PSRAM traffic on Core 0. At N ≈ 80 KB that is about 30 ms per frame at 40 MHz. "zero_copy" is a misnomer, because smoltcp `send_slice` always copies.
- **Proposed fix:**
  - (a) Move the port-81 TX socket buffers to internal DRAM: about 32 KB for the active socket and 2 KB for the standby socket. Retune `TCP_MAX_INFLIGHT_BYTES` and `P81_PIPELINE_LOW_WATER_BYTES` (main.rs:840-847) to about 90% and 45% of the new buffer size.
  - (b) Combined with FW-CORE-05, this cuts PSRAM traffic to 2N at twice the bandwidth.
  - (c) Longer term, Core 1 could assemble frames directly in PSRAM. This depends on cache coherence; see Open question 3.
- **Effort:** M
- **Verification:**
  - The FW-CORE-04 counters show at least 40% less Core 0 time per frame.
  - `free_dram` stays above a safe floor.
  - Stream FPS is the same or better.

### FW-CORE-07: `/snapshot` copies the full frame twice and blocks Core 0
- **Severity:** P1. **Category:** performance. **Duplicate of:** FW-NET-06
- **Location:** http.rs:1335; main.rs:3768-3799; main.rs:2558-2561
- **Problem:** The handler synchronously copies the frame from PSRAM into a 128 KB PSRAM response buffer, then again into the TX ring. That blocks for about 15-20 ms, during which the SPSC ring (about 25 ms of headroom) can overflow and drop a stream frame. Four 128 KB response buffers also waste 512 KB of PSRAM.
- **Proposed fix:**
  - Write only the headers into a small buffer.
  - Stream the body from a pinned frame slot across loop iterations, using the same pattern as port 81.
  - Add a pin, or a 4th slot, to `TripleFrameBuffer`.
  - Shrink the response buffers to 8 KB.
- **Effort:** M
- **Verification:**
  - 100 snapshots taken while streaming leave `c1_dropped` unchanged.
  - The snapshot body is byte-identical to the frame.

### FW-CORE-08: Frame slot references laundered to `&'static [u8]`
- **Severity:** P1. **Category:** safety / correctness hazard
- **Location:** main.rs:1985-2017 (`active_triple_static_slice`, `latest_snapshot_static_slice`); main.rs:3134; used at 1965, 2874, 3771, 3813, 4058, 4208
- **Problem:** `'static` slices of rotating slots are stored while `write_slot_mut()` later hands out `&mut` to the same memory. Today no stale read happens, but only by convention. A streamed snapshot (FW-CORE-07) would turn this into a use-after-recycle.
- **Proposed fix:** Replace the slices with a `FrameRef { slot, len, seq }` handle. Add `TripleFrameBuffer::get(&self, r) -> Option<&[u8]>`, which checks a per-slot generation, plus `pin(r)` / `unpin(r)` that `commit_write_slot` honours.
- **Effort:** M
- **Verification:**
  - Both `*_static_slice` functions are deleted.
  - A host test shows a pinned slot is never chosen as the write slot.

### FW-CORE-09: Ring-full backoff is spin-count based and drops the whole frame
- **Severity:** P2. **Category:** performance / reliability
- **Location:** main.rs:2089-2108; call sites at 2270, 2321, 2338, 2364, 2367; main.rs:366
- **Problem:** After 8,000 spins (duration depends on cache and code layout) Core 1 aborts the entire frame. Ring plus DMA headroom is only about 25 ms, and Core 0 regularly stalls longer than that.
- **Proposed fix:**
  - Use a CCOUNT deadline of about 5-10 ms that still honours `CORE1_PAUSE_REQ`.
  - Add a `CORE1_RING_FULL_EVENTS` counter.
  - Optionally use 2 KB chunks × 16 slots.
- **Effort:** S
- **Verification:** `c1_dropped/c1_pushed` stays near 0 while hammering `/snapshot` and `/status`.

### FW-CORE-10: `block_on_with_timeout` freezes networking and the SPSC drain for up to 15 s
- **Severity:** P2. **Category:** performance / reliability. **Duplicate of:** FW-NET-13
- **Location:** main.rs:771-797; main.rs:1372, 1483, 1615, 3215, 3234, 3307
- **Problem:** The function busy-polls a single future with no `iface.poll` and no SPSC drain. A roam scan while streaming freezes the stream for up to 5 s, and Core 1 drops every frame meanwhile.
- **Proposed fix:**
  - Add a `service: &mut dyn FnMut()` parameter, called about every 1 ms, that runs the drain and `poll_net_burst`. This requires restructuring the loop state into a context struct.
  - Alternatively, suppress roam scans while streaming unless RSSI falls below a hard floor.
- **Effort:** M
- **Verification:** A forced roam scan during streaming keeps frames flowing, with fewer than 5 `c1_dropped` per scan.

### FW-CORE-11: `poll_net_burst` caps transmission at about 23 KB per main-loop iteration
- **Severity:** P2. **Category:** performance
- **Location:** main.rs:906-918; call sites at 3336, 4011, 4083, 4261
- **Problem:** `Interface::poll` sends at most one segment per TCP socket per pass. Sixteen passes × 1426 B ≈ 22.8 KB, so an 80 KB frame needs at least 4 loop iterations, which couples throughput to main-loop latency. `PollResult` is ignored. `max_burst_size = Some(12)` (main.rs:753) only clamps the receive window.
- **Proposed fix:**
  - Keep polling while the result is `SocketStateChanged` or `poll_delay == ZERO`, within a CCOUNT budget of about 1.5 ms.
  - Interleave one SPSC drain per N polls.
  - Count how often the budget is exhausted.
- **Effort:** S
- **Verification:** Segments per iteration rise. Throughput of a synthetic 100 KB frame, in Mbit/s, improves.

### FW-CORE-12: Frames that will never be sent are still copied into PSRAM
- **Severity:** P2. **Category:** performance
- **Location:** main.rs:2805-2881 (commit at 2870-2875); main.rs:4021-4028
- **Problem:** When the network is the bottleneck, Core 0 assembles every Core 1 frame into PSRAM (about 2N of traffic each), and most are then superseded before port 81 sends them.
- **Proposed fix:**
  - Add `CORE0_WANTS_FRAME: AtomicBool`. Core 1 skips pushing chunks for frames nobody wants and counts them as `frames_skipped`.
  - Alternatively, have Core 0 call `reset_consumer()` when the send queue is far above the low-water mark.
- **Effort:** M
- **Verification:**
  - `p81_frames_superseded` drops to about 0, and Core 0 drain cycles fall proportionally.
  - Snapshots stay no older than one frame period.

### FW-CORE-13: Core 1 timeouts are loop-iteration counts
- **Severity:** P2. **Category:** reliability
- **Location:** main.rs:2129 (`3_000_000`), 2141, 2292 (`600_000`), 2351 (`4_000_000`)
- **Problem:** Wall-clock duration depends on APB MMIO latency and code placement. The 600k idle-EOF heuristic is probably over 100 ms, while the fallback path uses 25 ms (main.rs:1884).
- **Proposed fix:** Add an `#[inline(always)] fn ccount()` helper and express every timeout in cycles with wrapping subtraction. Match the 25 ms last-EOF rule.
- **Effort:** S
- **Verification:** The logged timeout durations match the configured ms. When the camera cable is pulled, recovery happens within the configured time.

### FW-CORE-14: `/control` reprogramming proceeds even if the Core 1 pause times out; stale chunks stay in the ring
- **Severity:** P2. **Category:** concurrency / correctness
- **Location:** main.rs:3878-3923; main.rs:405-421
- **Proposed fix:**
  - If the pause fails, requeue the control update.
  - Call `CORE1_SPSC.reset_consumer()` after reprogramming and before resume.
  - Make `resume_core1_camera_producer` return `bool` and count failures.
- **Effort:** S
- **Verification:** 200 rapid framesize toggles while streaming commit no invalid frames (new `c0_validation_failures` counter = 0) and cause no stalls.

### FW-CORE-15: Core 1 stack is 4 KB, 1 KB of it `unpack_buf`, with no guard
- **Severity:** P2. **Category:** reliability
- **Location:** main.rs:363; main.rs:2110
- **Proposed fix:**
  - Raise the stack to 8 KB, or remove `unpack_buf` (see FW-CORE-17).
  - Paint the stack with `0xA5` at startup and expose the high-water mark in `/status`.
- **Effort:** S
- **Verification:** After a 1 h stream plus a forced Core 1 panic test, the high-water mark is below 75% of the stack.

### FW-CORE-16: `main.rs` is monolithic, and its capture, SPI1 and error-response logic is duplicated
- **Severity:** P2. **Category:** maintainability. **Duplicate of:** FW-NET-17 and LL-15
- **Location:** main.rs:2886-4281 (about 1,395 lines in one function); 1704-1981 vs 2021-2374 (duplicated drain); 951-1121 vs 1128-1271 (duplicated SPI1 save/restore); 2993-3062 (12 copy-pasted unsafe blocks); 3600-3684 (4 copies of the error-response block)
- **Proposed fix:** Split into these modules:

  | Module | Contents |
  | --- | --- |
  | `core1/producer.rs` | Core 1 producer loop |
  | `capture/dma_drain.rs` | shared drain logic |
  | `boot/psram_spi1.rs` | SPI1 save/restore |
  | `boot/rtc.rs` | RTC setup |
  | `net/phy.rs` | network device |
  | `net/tcp_tx.rs` | TCP send path |
  | `http/port80_pool.rs` | Port 80 socket pool |
  | `http/port81_stream.rs` | Port 81 stream |
  | `wifi/manager.rs` | Wi-Fi management |
  | `ota_glue.rs` | OTA glue |

  Each module owns its state struct, and the main loop shrinks to about 100 lines.
- **Effort:** L
- **Verification:** `clippy::too_many_lines` set to 150 passes. The HIL regression suite passes.

### FW-CORE-17: Core 1 filters into a stack buffer, then copies into the ring slot
- **Severity:** P3. **Category:** performance
- **Location:** main.rs:2223-2224, 2270; spsc_ring.rs:114-135
- **Proposed fix:** Add `push_with(&self, flags, f: impl FnOnce(&mut [u8; N]) -> usize)` so the filter writes directly into the slot, then publishes with a Release store. Delete `unpack_buf`.
- **Effort:** S
- **Verification:** A host unit test covers wraparound and the full-ring case. CCOUNT shows fewer cycles per chunk.

### FW-CORE-18: Silent TX truncation to 1480 bytes, and a conservative MTU
- **Severity:** P3. **Duplicate of:** FW-NET-24
- **Location:** main.rs:712-719; main.rs:752
- **Proposed fix:** Replace the clamp with `debug_assert!(len <= MTU)`. Confirm the esp-radio MTU (1492?) and derive `TCP_MSS_BYTES = MTU - 14 - 20 - 20`.
- **Effort:** S
- **Verification:** A pcap shows the expected segment size and no truncation.

### FW-CORE-19: A Core 1 panic makes Core 1 wait for its own pause acknowledgement
- **Severity:** P3
- **Location:** main.rs:1312-1321; main.rs:385-399
- **Proposed fix:** When `Cpu::current() == AppCpu`, `pause_core1_camera_producer` returns true immediately. Stop I2S0 directly before the reset.
- **Effort:** S
- **Verification:** A forced Core 1 panic resets the chip within 1 ms.

### FW-CORE-20: Blocking UART `println!` heartbeat and roam logs, about 3-4 ms each
- **Severity:** P3
- **Location:** main.rs:4263-4279; 3207-3280
- **Proposed fix:** Shorten the heartbeat to under 120 bytes, or move telemetry to `/status`, or use a non-blocking ring logger.
- **Effort:** S
- **Verification:** The max-loop-latency counter no longer spikes every 30 s.

### FW-CORE-21: Hot-path errors are silently discarded
- **Severity:** P3
- **Location:** main.rs:4157 (`let _ = sock81.send_slice(resp);`); 3336; 2157; 2116 (`let _ = build_circular_ring(..)`); 2359; 3883-3917
- **Proposed fix:**
  - Add an `FwStats` struct of `AtomicU32` counters, exposed in `/status`.
  - Send port-81 error responses with the `tx_off` mechanism.
  - Update `last_applied_ctrl` only for writes that succeeded.
  - Do not start DMA if the ring build failed.
- **Effort:** S
- **Verification:** The counters appear in `/status`. A mock SCCB fault leaves `last_applied_ctrl` unchanged.

### FW-CORE-22: Hard-coded static IP and gateway
- **Severity:** P3 (raised to P1 in FW-NET-05). **Duplicate of:** FW-NET-05 and FW-NET-15
- **Location:** main.rs:1519, 3250, 3320, 3347-3360, 3397-3407, 3433
- **Proposed fix:** Add a single `apply_ip_config(&mut iface, &cfg)` that uses `KlipperCamConfig.network`.
- **Effort:** S
- **Verification:** On a DHCP-only network in another subnet, the device is reachable.

### FW-CORE-23: Dead aliases, the boot HIL self-test linked into release, and the 128 KB magic number repeated
- **Severity:** P3. **Duplicate of:** BUILD-24
- **Location:** main.rs:1673-1675, 1698-1700 (dead aliases); main.rs:2607-2642 (HIL); 2073, 2257, 2320, 2330, 2542-2544 (`128 * 1024`)
- **Proposed fix:**
  - Delete the dead aliases.
  - Gate `hil_check_*` behind a `hil-selftest` feature.
  - Introduce `const FRAME_SLOT_BYTES: usize = 128 * 1024;`.
- **Effort:** S
- **Verification:** `cargo bloat` shows smaller `.text`. `grep -n "128 \* 1024"` returns only the const.

---

## What looks good
- **SPSC orderings are correct.** The producer reads `tail` with Acquire and publishes `head` with Release, and the consumer mirrors this. The ring sits in internal DRAM. Power-of-two sizing is enforced at compile time. Wraparound, full and abort cases are unit-tested.
- **The pause/resume handshake is two-phase.** The parked loop touches only DRAM atomics inside a `#[ram]` function. OTA returns 503 when the pause times out.
- **Descriptor access is ordered.** `Lldesc::owner()` uses `read_volatile` plus a fence. The EOF clear is followed by `memw`.
- **Time does not wrap.** The time base is u64 ms, and `saturating_sub` is used throughout.
- **No panics on the runtime path.** There is no `unwrap` or `expect` on the Xtensa runtime path. Fatal paths go through `quiesce_psram_and_software_reset`.
- **Frame validation is cheap.** EOI verification scans backward, and the SOI check is bounded to 256 B.
- **Port-81 promotion is gated correctly.** A new frame is promoted only after the previous one is fully enqueued. The RTC watchdog is fed in every bounded wait.

## Open questions
1. **Sensor frame period.** What is the real VSYNC period at HD on GPIO25? Is it about 80 ms? This decides whether the sensor clock (FW-CORE-04) or Core 0 bandwidth (FW-CORE-05/06) is the limiter.
2. **Rollback and OTA testing.** Is bootloader app-rollback enabled? Has `POST /update` been tested on hardware with Core 1 active since commit `75625d8`?
3. **PSRAM mapping on Core 1.** With `PsramVaddrMode::Normal`, does APP_CPU's cache map PSRAM coherently? This must be answered before Core 1 writes frames into PSRAM.
4. **I2S in-link overrun.** Does it overwrite unconsumed halves when `check_owner` is off? See LL-07.
5. **VSYNC re-arm gap.** Is the gap long enough for the tail drain plus a ring rebuild plus `start()`? Count SOI-miss aborts.
6. **Wi-Fi task scheduling.** Does the never-yielding Core 0 loop starve esp-radio tasks?
7. **DRAM headroom.** How much internal DRAM is free at runtime with Wi-Fi up? This determines whether FW-CORE-06(a) fits.
8. **80 MHz stability.** Is 80 MHz PSRAM stable on this board?
9. **Frame size.** What is the actual JPEG size at the default quality?
