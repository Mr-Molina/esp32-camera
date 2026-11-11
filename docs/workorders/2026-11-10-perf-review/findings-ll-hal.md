# Findings: `esp32-cam-ll` and `esp32-cam-hal` (LL and HAL)

**Baseline commit:** `5a21ce2`. **Scope:** every line of `crates/esp32-cam-ll/src/*` and `crates/esp32-cam-hal/src/*`. Line numbers refer to the baseline commit; confirm them before editing.

## Summary

The pixel filters are clean. The I2S register bit positions match the ESP32 TRM and `ll_cam.c` wherever they could be checked.

The serious problems are these:

- **Unsound safe APIs.** `Esp32I2sCamera::start` is a safe function that takes a raw descriptor address and starts DMA. `build_circular_ring` lets DMA write through a `&[u8]`. The HAL calls `start(&cfg, 0)`, so `#![forbid(unsafe_code)]` code can start DMA from address 0.
- **Fragile frame-end detection.** The polled JPEG loop gates on the first descriptor's owner/length write-back, so it misses the frame end on about half of all frames. It also searches stale DMA memory, so it can match an old EOI. Ring overrun is never detected.
- **The FPS ceiling is the sensor clock, not DMA or the filter.** 12.53 FPS is roughly 15 × 20/24.
- **The HAL cannot run on the target.** It owns no descriptors, buffers or XCLK hardware. It moves its whole inline pool on every type-state change and allows only one lease at a time.

---

## LL findings

### LL-01: `Esp32I2sCamera::start` is a safe fn that loads any address into `in_link` and starts DMA
- **Severity:** P0 · **Category:** soundness
- **Location:** esp32_i2s.rs:937-1030; dma_ring.rs:540-605
- **Evidence:** `pub fn start(&mut self, dma_cfg: &DmaSizeConfig, first_desc_addr: u32) -> bool {` … `xtensa_mmio_write(I2S0_BASE_ADDR + I2S_IN_LINK_REG_OFFSET, link_val);`
- **Problem:** Safe code can start DMA at address 0 (see HAL-01), point it at PSRAM (which is not DMA-capable), or drop the buffer while the DMA is still writing. Only `capture_jpeg_frame_polled` checks the 0x3FFA_E000..=0x3FFF_FFFF window. The firmware self-test (main.rs:158-171) already builds a ring over a local, immutable, 1-byte-aligned `[0u8; 512]` and starts DMA on it.
- **Proposed fix:** Make `start` an `unsafe fn` with a `# Safety` contract. Better: replace the `u32` with an owned `DmaRing<'a>` from a checked constructor that validates alignment, the DMA window for descriptors and buffer, and node size (≤ 4092, multiple of 4). `start(&'a mut DmaRing)` then returns a `RunningCapture<'a>` guard whose `Drop` stops and resets the DMA.
- **Effort:** M
- **Verification:** A trybuild compile-fail test for drop-while-running. Unit tests that reject a PSRAM-range or misaligned ring. Miri on the guard's drop.

### LL-02: The exported `build_circular_ring` takes `&[u8]` but the hardware writes through it
- **Severity:** P1 · **Category:** soundness
- **Location:** dma_ring.rs:540-544, 592; lib.rs:20-24
- **Evidence:** `buffer: &[u8],` … `chunk_ptr = buf_base.add(byte_offset) as *mut u8;`
- **Problem:** Writing through a pointer derived from a shared borrow is UB. A fixed `build_circular_ring_mut` (SEC-011) exists but is not exported. Every caller, including firmware main.rs:160, 1724 and 2061, uses the `&[u8]` variant.
- **Proposed fix:** Delete the `&[u8]` builder, rename `_mut` to `build_circular_ring`, export only that, and update the callers. Return the typed ring from LL-01.
- **Effort:** S
- **Verification:** A Miri test that writes through `desc.buf` after building. Grep shows no `&[u8]` builder left.

### LL-03: Descriptor and DMA memory are modelled as plain Rust references; no barrier before starting DMA
- **Severity:** P1 · **Category:** soundness
- **Location:** dma_ring.rs:35-41, 101-114, 116-129, 596-600; esp32_i2s.rs:1020-1025, 1260-1270
- **Problem:** Hardware writes memory that Rust holds `&`/`&mut` to, which is UB, so LLVM may cache or reorder the loads. Descriptor writes are plain stores with no `compiler_fence(Release)`/`memw` before the volatile `in_link` start write. Filter reads may move past the volatile re-arm. The setters are volatile read-modify-writes that can overwrite a concurrent owner/length write-back. Each getter issues two SeqCst fences.
- **Proposed fix:**
  - Make `size_length_flags` a `VolatileCell<u32>` or `AtomicU32`.
  - Hold the ring as `NonNull<[u8]>` and expose half-buffer reads only after `memw` plus `compiler_fence(Acquire)`.
  - Add `compiler_fence(Release)` and `memw` before the `in_link` write, and after the filter reads but before re-arm.
  - Write the packed setter value with a single volatile store.
- **Effort:** M
- **Verification:**
  - Miri.
  - Disassemble `start` and confirm `memw` precedes the store to 0x3FF4_F034.
  - A 10,000-frame soak with a CRC over the OV2640 colour-bar test pattern.

### LL-04: `calc_dma_sizes` returns 128 KiB for JPEG while C uses 32 KiB
- **Severity:** P1 · **Category:** performance/memory
- **Location:** dma_ring.rs:482-519; test at 899-903
- **Evidence:** `let dma_half_buffer_size = ((dma_buffer_size_max / 2) & !3).max(4);` … `let dma_buffer_size = dma_half_buffer_cnt * dma_half_buffer_size;`
- **Problem:** C uses node 2048, half 4096, total 32 KiB. The Rust function returns half 16 KiB, total 128 KiB and 64 descriptors, which is most of the DMA-capable DRAM. EOF arrives only every 4 KiB of JPEG, which adds frame-end latency. The firmware avoids this by hard-coding its own config (main.rs:1727-1738), but the HAL uses the function (driver.rs:774-781).
- **Proposed fix:** Match C (`node = min(2048, cap)`, `half = 2*node`, `cnt = 8`). Derive the `JPEG_DMA_*` constants from the same function. Fix the test.
- **Effort:** S
- **Verification:** A unit test that 640x480 JPEG gives `{node:2048, half:4096, total:32768, nodes:16}`.

### LL-05: The frame-end fast path depends on the first descriptor's write-back and misses about 50% of frame ends
- **Severity:** P1 · **Category:** performance
- **Location:** esp32_i2s.rs:1304-1314 (same pattern at firmware main.rs:1894)
- **Evidence:**
  ```rust
  let has_new_data = !descriptors[2 * half_buf_idx].owner()
      || descriptors[2 * half_buf_idx].length() > 0;
  if (vsync_eof || idle_since_eof >= 2048) && has_new_data {
  ```
- **Problem:** VSYNC never closes a descriptor. When the JPEG ends inside the first 2048 B of the current half, `has_new_data` stays false. The loop then waits for the next frame's data. Because DMA restarts per frame, this can miss the next frame start and double the period for that frame.
- **Proposed fix:** On a debounced VSYNC end (or an idle timeout), process the current half unconditionally. Rely on poisoning (LL-06) to rule out stale EOIs. Remove the owner/length gate.
- **Effort:** S
- **Verification:** Log the VSYNC-to-return latency over 1,000 frames. The bimodal distribution should collapse.

### LL-06: The EOI search reads stale bytes from the previous lap of the DMA ring (spliced frames)
- **Severity:** P1 · **Category:** correctness
- **Location:** esp32_i2s.rs:1315-1326, 1340-1352
- **Problem:** Parts of the ring the DMA has not reached yet still hold the previous frame. An old `FF D9` there satisfies the EOI test. SOI and EOI both pass, so the HAL cannot reject the result. (The firmware's Core 1 path zeroes halves after use; the ll crate does not.)
- **Proposed fix:** When re-arming a half, fill it with a non-marker pattern (about 4 µs per 4 KiB). Alternatively, track exactly which bytes were written using the EOF count.
- **Effort:** S
- **Verification:**
  - Host test: seed the ring with an old frame containing an EOI, then capture a shorter new frame. Expect `CorruptedFrame` or the correct length.
  - JPEG decode success rate over 10,000 frames on hardware.

### LL-07: Ring overrun is undetected, `check_owner` is off, and DSCR_ERR/EMPTY are ignored
- **Severity:** P1 · **Category:** reliability
- **Location:** esp32_i2s.rs:862-866, 988-996, 1215-1221, 1256-1257
- **Evidence:** `xtensa_mmio_write(I2S0_BASE_ADDR + I2S_LC_CONF_REG_OFFSET, 0);`
- **Problem:**
  - Writing `lc_conf=0` turns off `check_owner`, so the DMA overwrites halves that have not been consumed yet.
  - If the CPU falls 8 halves behind (about 1 ms stall), the ring laps silently.
  - The drain loop may process up to 8 halves that were never written.
- **Proposed fix:**
  - Track a running EOF count from the `in_eof_des_addr` delta. Fail the frame with `Overflow` when more than `half_buf_cnt - 1` halves are pending.
  - Treat DSCR_ERR/EMPTY as a frame abort.
  - Evaluate `check_owner=1`.
  - Never drain past `target_half`.
- **Effort:** M
- **Verification:** Inject a 5 ms stall on hardware. `Overflow` should be returned, not a corrupt `Ok`.

### LL-08: XCLK is bookkeeping only, and any frequency is accepted; the FPS cap is the sensor clock
- **Severity:** P1 · **Category:** performance
- **Location:** xclk.rs:25-30, 120-145
- **Problem:** `XclkController` never touches the LEDC registers; the firmware uses esp-hal LEDC at main.rs:4290-4310. Arbitrary frequencies are accepted although 80 MHz APB with 1-bit duty yields only integer divisors. 24 MHz would give a fractional divider and jitter. OV2640 HD at 20 MHz is about 12.5 fps.
- **Proposed fix:**
  - Validate that the requested frequency can be realized, or report the achieved frequency.
  - For more FPS, output exactly 24 MHz, either from APLL through CLK_OUT on GPIO0, or by reducing VTS/dummy lines in the sensor (see SENS-01).
- **Effort:** M
- **Verification:** Measure XCLK on a scope. Compare FPS at 20 vs 24 MHz.

### LL-09: Full-register writes to `I2S_FIFO_CONF` zero `rx_data_num`
- **Severity:** P2 · **Category:** correctness
- **Location:** esp32_i2s.rs:832-836, 887, 916-919
- **Evidence:** `let fifo_val = (1 << 12) | (1 << 20) | ((self.sampling_mode as u32) << 16);`
- **Problem:** The write zeroes `rx_data_num` (reset value 32), `tx_data_num` and `tx_fifo_mod`. C only sets bitfields. This may change the FIFO-to-DMA burst threshold and when the tail bytes land at VSYNC.
- **Proposed fix:** Read-modify-write only bits 12, 16..18 and 20, or set `rx_data_num=32` explicitly.
- **Effort:** S
- **Verification:** Dump the register on hardware and compare with an IDF build. A/B test the tail latency.

### LL-10: Filter, copy and scan go through PSRAM; byte scans have triple bounds checks
- **Severity:** P2 · **Category:** performance
- **Location:** esp32_i2s.rs:546-557, 1260-1281, 445-475
- **Problem:** The code filters into a stack chunk, copies that into `jpeg_out` (PSRAM for 720p), then scans `jpeg_out` for EOI. That is an extra copy plus a PSRAM read-back for each 1 KiB chunk.
- **Proposed fix:** Scan the SRAM chunk for EOI, carrying the last byte across chunks, before copying. After SOI, filter straight into `jpeg_out[written..]`. Use `iter().position(|&b| b == 0xFF)`.
- **Effort:** S
- **Verification:** CCOUNT per half-buffer. Expect about 30-50% less.

### LL-11: Capture is a busy poll with iteration-count timeouts and no ISR
- **Severity:** P2 · **Category:** performance
- **Location:** esp32_i2s.rs:944-947, 1208-1215, 1305-1314
- **Proposed fix:** Provide an interrupt-driven variant: an IN_SUC_EOF ISR records `in_eof_des_addr` into an atomic. At minimum, base timeouts on CCOUNT and accept an idle callback (see LL-15).
- **Effort:** M
- **Verification:** Compare APP_CPU utilization and FPS with and without the ISR.

### LL-12: Unlocked DPORT read-modify-write on `PERIP_CLK_EN`/`PERIP_RST_EN`
- **Severity:** P2 · **Category:** reliability
- **Location:** esp32_i2s.rs:846-851, 1090-1093
- **Problem:** These registers race with esp-hal on the other core and can clear the LEDC (XCLK) clock bit. The DPORT dual-core read errata adds risk.
- **Proposed fix:** Wrap the accesses in `critical_section::with`, or delegate them to the esp-hal clock and reset APIs.
- **Effort:** S
- **Verification:** HIL stress test: toggle clocks on one core while re-initializing the camera on the other. XCLK must stay present.

### LL-13: `Esp32I2sCamera` is `Clone`/`Default`, so multiple owners of I2S0 can exist
- **Severity:** P2 · **Category:** soundness
- **Location:** esp32_i2s.rs:563-590, 594-596
- **Problem:** Dropping a clone stops the DMA that the original believes is running.
- **Proposed fix:** Remove `Clone`. Take the esp-hal `I2S0` peripheral token or use an atomic singleton. Keep `Drop`.
- **Effort:** S
- **Verification:** A compile-fail test on `.clone()`. A unit test that a second `take()` returns `None`.

### LL-14: Every operation writes both a shadow register block and real MMIO, and the two sequences drift
- **Severity:** P2 · **Category:** maintainability
- **Location:** esp32_i2s.rs:283-321, 806-843, 916-925; esp32s2_i2s.rs:253-281; esp32s3_lcd_cam.rs:293-347
- **Problem:** Host tests check the shadow, not what the hardware receives. The S2 and S3 modules are shadow-only and have no hardware effect.
- **Proposed fix:**
  - Add a `RegisterIo` trait with an `Mmio` implementation (Xtensa) and a `Recording` implementation (host).
  - Use one code path for both.
  - Test against a golden write trace transcribed from `ll_cam.c`.
  - Mark S2/S3 `#[doc(hidden)]` until they have real backends.
- **Effort:** L
- **Verification:**
  - Golden-trace tests.
  - Smaller firmware binary.

### LL-15: The ll capture loop cannot service the network, so the firmware duplicates it with raw register addresses
- **Severity:** P2 · **Category:** maintainability. **Merged with:** FW-CORE-16
- **Location:** esp32_i2s.rs:1120-1126; firmware main.rs:1705-1712 and 2040+
- **Problem:** Fixes to LL-05, LL-06 and LL-07 (and FW-CORE-02) must each be made in three places.
- **Proposed fix:**
  - Add a `CaptureHooks` trait (`idle()`, `now_us()`), or a `&mut dyn FnMut()` parameter.
  - Return rich diagnostics from the loop.
  - Make the firmware call the ll loop.
  - Delete the `0x3FF4_F0xx` literals from the firmware.
- **Effort:** M
- **Verification:**
  - Grep shows no `0x3FF4_F0` in klipper-cam-fw.
  - FPS is equal or better.

### LL-16: Duplication across the three chip variants has already drifted
- **Severity:** P2 · **Category:** maintainability
- **Location:** dma_filter.rs:349-357; esp32s2_i2s.rs:292-298, 320-359; esp32s3_lcd_cam.rs:377-384, 406-445, 470-473; dma_ring.rs:296-300
- **Problem:**
  - The native-Y8 sensor list exists three times, and only S3 includes HM0360.
  - JPEG `dma_sizes` is identical in S2 and S3.
  - The S3 link-address width (20-bit mask vs 32-bit doc) is inconsistent.
- **Proposed fix:** Add a `CamLl` trait, plus a shared `is_native_y8` and JPEG sizing function.
- **Effort:** M
- **Verification:** Shared unit tests across all three variants.

### LL-17: "Miri-verified MMIO" is false; the real capture loop is untested on the host
- **Severity:** P2 · **Category:** tests
- **Location:** esp32_i2s.rs:1385-1443, 1760-1787; dma_ring.rs:899-903; dma_filter.rs:449-557; README.md:5
- **Problem:**
  - All MMIO is `cfg(xtensa)`, so Miri only sees the shadow writes and the filters.
  - The host branch of `capture_jpeg_frame_polled` runs a different algorithm.
  - The filter tests use tiny fixed inputs.
- **Proposed fix:**
  - Build a simulated I2S model behind `RegisterIo` (LL-14) and run the real loop on the host and under Miri.
  - Add proptests comparing each filter with a scalar reference.
  - Add golden register-value tests.
  - Correct the README.
- **Effort:** M
- **Verification:**
  - Host coverage of esp32_i2s.rs:1158-1383 is above 0%.
  - Miri stays green.

### LL-18: Filters dispatch through a function pointer and gather byte-by-byte
- **Severity:** P3 · **Category:** performance
- **Location:** dma_filter.rs:119-137, 153-171
- **Problem:** The filter costs only about 2.5 ms per 100 KB frame, so it is not the bottleneck.
- **Proposed fix:**
  - Use a `match` with direct calls and `#[inline]`.
  - For JPEG, combine the four `(w>>16)&0xFF` lanes into one u32 store.
- **Effort:** S
- **Verification:**
  - CCOUNT micro-benchmark.
  - The randomized reference test from LL-17.

### LL-19: Per-frame `start()` rewrites all 12 GPIO-matrix entries; the EOF-to-half mapping is a linear scan
- **Severity:** P3 · **Category:** performance
- **Location:** esp32_i2s.rs:1004-1018, 1242-1254
- **Proposed fix:**
  - Program the GPIO matrix only in `set_pin`.
  - Compute the half index arithmetically: `h = ((eof - base) / 12) / 2`.
- **Effort:** S
- **Verification:**
  - A unit test of the arithmetic.
  - CCOUNT on `start()`.

### LL-20: Misattached docs, safe MMIO helpers, stale comments, and lints at `warn`
- **Severity:** P3 · **Category:** maintainability
- **Location:** esp32_i2s.rs:633-658, 323-362, 402-406
- **Problem:**
  - The doc comment and `#[inline]` belong to `regs()` but sit on `check_and_clear_dma_err`.
  - `xtensa_mmio_write` and `xtensa_mmio_read` are safe fns that dereference any `usize`.
  - The IO_MUX comment on `FUN_WPD` is wrong.
  - The README says 16-byte alignment; the actual alignment is 4.
  - Workspace lints are `warn`, not `deny`.
- **Proposed fix:**
  - Move the attributes to `regs()`.
  - Make the MMIO helpers `unsafe fn`.
  - Fix the comments and the README.
  - Set the ll lints to `deny`.
- **Effort:** S
- **Verification:** `cargo doc` and clippy pass with `-D warnings`.

### LL-21: `stop()` does not reset the AHB DMA, so late writes can land after the borrows end
- **Severity:** P3 · **Category:** soundness
- **Location:** esp32_i2s.rs:1033-1067, 1282-1290
- **Proposed fix:** After stop, pulse `lc_conf.in_rst` and `ahbm_rst`, then `memw`.
- **Effort:** S
- **Verification:** Poison the buffer after return, wait 1 ms, and confirm nothing changed.

---

## HAL findings

### HAL-01: The `forbid(unsafe)` HAL passes descriptor address 0 to `start`
- **Severity:** P0 · **Category:** soundness
- **Location:** driver.rs:937-946, 1142-1147
- **Evidence:** `if !self.i2s.start(&dma_cfg, 0) {`
- **Problem:**
  - On Xtensa the DMA fetches descriptors from the 0x3FF0_0000 peripheral window and writes wherever they point.
  - The driver owns no ring and no buffer.
  - `XclkController::enable_out_clock` touches no hardware, so the sensor never gets a clock.
- **Proposed fix:**
  - Until LL-01 lands, gate `start_streaming` and one-shot capture with `cfg(not(target_arch = "xtensa"))`, or return `NotSupported`.
  - Then give `CameraDriver` an owned `DmaRing<'static>` in internal RAM and real LEDC XCLK.
- **Effort:** M
- **Verification:**
  - No `start(.., 0)` call remains.
  - On hardware, `init().start_streaming().capture_frame()` returns a decodable JPEG.

### HAL-02: `FramePool` stores buffers inline and is moved on every type-state transition
- **Severity:** P1 · **Category:** performance
- **Location:** frame_pool.rs:215-249; driver.rs:666-687, 822-836, 947-961, 1096-1110
- **Problem:**
  - Every transition moves `COUNT*BUF_CAP` bytes, plus copies on the stack.
  - A realistic 720p pool of 2×128 KiB overflows any task stack.
  - The pool cannot be placed in PSRAM because `new` is not `const`.
- **Proposed fix:**
  - Have `FramePool` borrow its storage (`&'a mut [AlignedBuf<CAP>; N]`) and provide `const fn new`.
  - Have `CameraDriver` hold `&'a mut FramePool`.
- **Effort:** M
- **Verification:**
  - Stack high-water mark across transitions.
  - A static `FramePool<2, 131072>` construction test.

### HAL-03: A `FrameGuard` borrows the whole pool mutably, allowing only one lease at a time
- **Severity:** P1 · **Category:** performance
- **Location:** frame_pool.rs:426-437, 492, 104-110
- **Problem:** The producer cannot fill slot B while the consumer reads slot A. This is why the firmware rolled its own `TripleFrameBuffer`.
- **Proposed fix:**
  - Split into `Producer` and `Consumer` handles that take `&self`.
  - Move slot states with `compare_exchange` through FREE→WRITING→READY→LEASED, using Acquire/Release.
  - Have the guard borrow only its slot.
- **Effort:** M
- **Verification:**
  - A loom or threaded test, plus Miri.
  - On hardware, replace `TripleFrameBuffer` and compare.

### HAL-04: The HAL reserves `len/2` for grayscale, but the filter outputs `len/4`
- **Severity:** P2 · **Category:** correctness
- **Location:** driver.rs:1172-1182; dma_filter.rs:177-180
- **Problem:** The HAL returns a spurious `BufferTooSmall` when the pool is more than half full.
- **Proposed fix:** Add `DmaFilterKind::output_len(in_len)` in ll and use it in the HAL.
- **Effort:** S
- **Verification:** A property test that `apply()` returns exactly `output_len()`.

### HAL-05: SOI must be at offset 0; the EOI probe mixes units; the backward EOI search can return a stale marker
- **Severity:** P2 · **Category:** correctness
- **Location:** driver.rs:35-37, 1193, 1201-1214
- **Proposed fix:**
  - Search for SOI within `CAM_SOI_PROBE_BYTES` and trim before it.
  - Compute the probe window from `output_len`.
  - Search forward for EOI starting after SOI.
- **Effort:** S
- **Verification:** Tests for leading padding and for a stale trailing `FF D9`.

### HAL-06: `psram_dma_mode` copies raw DMA words on ESP32, where PSRAM DMA is impossible anyway
- **Severity:** P2 · **Category:** correctness
- **Location:** driver.rs:1153, 1164-1169
- **Proposed fix:** Reject `psram_dma_mode` unless the target is S2 or S3 (`dma_bytes_per_item == 1`).
- **Effort:** S
- **Verification:** A unit test that `init` returns `NotSupported`.

### HAL-07: The driver always uses the ESP32 I2S backend, so S3 boards fail `init`
- **Severity:** P2 · **Category:** maintainability
- **Location:** driver.rs:679, 706, 747-749; esp32_i2s.rs:711-731
- **Problem:** `Esp32S3XiaoPins` uses GPIOs above 40, so ESP32 `set_pin` rejects them. `vsync_invert` is hard-coded to `true`.
- **Proposed fix:**
  - Make the backend a type parameter bound by `CamLl` (LL-16).
  - Read `vsync_invert` from the config.
- **Effort:** M
- **Verification:** A unit test that the S3 XIAO driver initializes.

### HAL-08: Pin validation ignores PSRAM-reserved pins
- **Severity:** P2 · **Category:** correctness
- **Location:** board_pins.rs:63-71, 140-236, 56-61
- **Problem:**
  - `is_octal_psram_reserved` exists but is never called.
  - GPIO16 and GPIO17 (PSRAM CS and CLK on WROVER) are not reserved.
- **Proposed fix:** Add a `psram: PsramKind` field to the pin config and check it in `validate_pins`.
- **Effort:** S
- **Verification:** Unit tests that both cases return `InvalidArg`.

### HAL-09: The default `HalOv2640Sensor` writes only bank-select, but `status` claims it was configured
- **Severity:** P2 · **Category:** correctness
- **Location:** driver.rs:321-426, 428-537
- **Problem:** About 110 lines are duplicated across the two `sensors` feature states.
- **Proposed fix:**
  - Delegate to the real ov2640 tables, or rename the type to `MockOv2640`.
  - Remove the duplication.
- **Effort:** S
- **Verification:** A mock-bus test that the full HD framesize sequence is recorded.

### HAL-10: A failed type-state transition consumes the driver
- **Severity:** P3 · **Category:** reliability
- **Location:** driver.rs:937-946, 1026-1043
- **Proposed fix:** Return `Result<Next, (Self, CameraError)>`.
- **Effort:** S
- **Verification:** A test that retries after a forced failure.

### HAL-11: O(COUNT) rescans; `return_all` cannot free leaked guards; timestamps are fabricated at 30 fps
- **Severity:** P3 · **Category:** performance
- **Location:** frame_pool.rs:292-300, 330-373, 400-406; driver.rs:1155-1156
- **Proposed fix:**
  - Have `return_all` free every slot.
  - Keep a `newest_ready` index.
  - Take timestamps from a `now_us` source.
- **Effort:** S
- **Verification:** `forget(guard); return_all(); free_count() == COUNT`.

### HAL-12: HAL tests use only 256-byte pools and the shadow I2S
- **Severity:** P2 · **Category:** tests
- **Location:** driver.rs:1271-1347; frame_pool.rs:550-755
- **Problem:** No tests cover the pixel-format length checks, SOI and EOI edge cases, large pools, pool state-machine properties, or S3 boards.
- **Proposed fix:**
  - Add proptest state-machine tests.
  - Add per-format table tests.
  - Add a static large-pool test.
- **Effort:** M
- **Verification:** The new tests fail on today's code and pass after the fixes.

---

## What looks good
- **Register bits.** The I2S0 offsets and bits checked against the TRM are correct: `conf`, `int_raw` bits 9/13/15, `in_link`, `lc_conf`, `conf2`, `pd_conf=0x0A`, `rx_chan_mod`, `rx_bits_mod`, `rx_fifo_mod`, and `rx_eof_num = half/4`.
- **Sample mode and filter.** JPEG uses `SM_0A00_0B00` paired with `filter_jpeg` taking byte 2, which matches C.
- **Filter loops.** They use `as_chunks`, with no per-byte bounds checks and exact remainder handling.
- **DMA safety checks.** Filtering runs on SRAM before PSRAM, and the polled path rejects out-of-window rings.
- **EOF handling.** EOF catch-up uses `in_eof_des_addr`, and the consume side uses `memw` plus Acquire.
- **Descriptor layout.** `Lldesc` is `repr(C, align(4))`, 12 bytes, with nodes capped at 4092 and checked to be multiples of 4.
- **Pin and pad setup.** RTC_IO mux-disable bits, IO_MUX values and GPIO-matrix `IN_SEL` are correct.
- **`FrameGuard::drop`.** It is a single Release store. `write_and_lease` filters directly into the slot with no copy.

## Open questions
1. **Descriptor write-back.** Does the ESP32 I2S inlink write back `owner=0` and `length` when `check_owner=0`? FW-CORE-02 and LL-05 depend on this. Can `I2S_INLINK_DSCR` (around 0x48/0x4C/0x50, unverified) report the current descriptor?
2. **`rx_data_num=0`.** What does it do in camera mode?
3. **WS input index.** Is `I2S0I_WS_IN_IDX = 28` correct? The working 12.5 FPS suggests yes.
4. **VSYNC polarity.** Could DMA start mid-frame with `vsync_invert=true`? Confirm OV2640 COM10.
5. **Burst setting.** Does `indscr_burst_en` (lc_conf bit 10) help on ESP32?
6. **S3 GDMA link width.** Is the `IN_LINK` address 20 or 32 bits?
7. **Native-Y8 list.** Should HM0360 be in the ESP32 and S2 native-Y8 lists?
8. **Reduced VTS.** Can reducing VTS at 20 MHz XCLK go above 12.5 fps? See SENS-01.
