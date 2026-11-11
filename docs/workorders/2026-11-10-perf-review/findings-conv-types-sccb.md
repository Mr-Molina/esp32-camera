# Findings: `esp32-cam-conversions`, `esp32-cam-types`, `esp32-cam-sccb` (CONV, TYPES, SCCB)

**Baseline commit:** `5a21ce2`. **Scope:** every line of these three crates, plus the firmware call sites of the JPEG scanners. Line numbers are against the baseline commit.

## Summary

All three crates are memory-safe: no `unsafe`, no heap allocation, and every slice index in `jpeg_scan.rs` is guarded. No panicking input was found.

The per-frame cost on the device is small. The firmware runs `verify_jpeg_soi` on at most 256 bytes and a backward `verify_jpeg_eoi` from the end of the frame (main.rs:1937-1940, 2858-2859). `parse_jpeg_dimensions` and `validate_jpeg_frame` are not on the per-frame path.

The real performance problems are in the codec and the shared types:
- The IDCT uses `i64` on a 32-bit core.
- The encoder divides on every coefficient and only emits 4:4:4.
- Both bit readers use `u64`.
- `CameraError` is 12 bytes, which probably forces every `Result` on the hot paths (including SCCB) to be returned through memory.

The "SWAR" scanner gains little on Xtensa. Its real risk is an unbounded backward scan when a frame has no EOI. The SCCB layer is correct but conservative: there is no burst batching, and the masked-write helpers silently skip writes when the value is unchanged.

---

## CONV

### CONV-01: IDCT computed in 64-bit integers
- **Severity:** P1 · **Category:** performance
- **Location:** jpeg_codec.rs:208-333
- **Evidence:** `let mut workspace = [0i64; 64];` … `let f0 = coeffs[base] as i64;`
- **Problem:** i64 multiply/add costs about 2.5-3× an i32 LLM IDCT, roughly 2k vs 700 cycles per block. That adds about 50 ms per VGA 4:2:2 decode and doubles stack use. i64 was chosen only so that `test_conv001` (±32767 coefficients) cannot overflow.
- **Proposed fix:** Use IJG `jidctint` i32 arithmetic with `wrapping_*` operations. Clamp dequantized coefficients to ±2047 and the pass-1 workspace to ±(1<<14).
- **Effort:** M
- **Verification:**
  - Differential test against the i64 version over 1M valid-range blocks: results must be bit-identical.
  - `test_conv001` still passes.
  - CCOUNT benchmark.

### CONV-02: Encoder divides every coefficient during quantization
- **Severity:** P1 · **Category:** performance
- **Location:** jpeg_codec.rs:579-596
- **Evidence:** `((adj as u32) / (q as u32)) as i32`
- **Problem:** About 921k hardware divides per VGA frame, roughly 40-115 ms.
- **Proposed fix:** Precompute reciprocals in `compute_quant_table`, as libjpeg's `compute_reciprocal` does, and quantize with `(adj * recip) >> 16`.
- **Effort:** S
- **Verification:**
  - Exhaustive equality test over adj 0..4096 and q 1..=255.
  - Encoder output is byte-identical.

### CONV-03: Encoder only emits 4:4:4, even from YUV422 input
- **Severity:** P1 · **Category:** performance
- **Location:** jpeg_codec.rs:513-525, 829-857
- **Problem:** Chroma is upsampled and then encoded as duplicated data. H2V1 encoding is 33% cheaper and the output 20-30% smaller.
- **Proposed fix:** Add a `Subsampling { H1V1, H2V1, H2V2 }` parameter, and make H2V1 the default for YUV422 input, with 16x8 MCUs.
- **Effort:** M
- **Verification:**
  - Round-trip PSNR test through the decoder.
  - Size and cycle benchmark.

### CONV-04: Encoder bit writer uses a u64 buffer and a checked store per byte
- **Severity:** P2 · **Category:** performance
- **Location:** jpeg_codec.rs:402-462
- **Proposed fix:**
  - Use a u32 accumulator with at most 7 pending bits.
  - Combine the code and magnitude writes into one call.
  - Check headroom once per MCU.
  - Turn the `len > 32` check into a `debug_assert!`.
- **Effort:** M
- **Verification:** Output is byte-identical on the test corpus, and a cycle benchmark shows the gain.

### CONV-05: Decoder bit reader refills a byte at a time and decodes codes longer than 8 bits bit by bit
- **Severity:** P2 · **Category:** performance
- **Location:** jpeg_codec.rs:965-1063
- **Proposed fix:**
  - Use a u32 buffer refilled to 25 bits or more per refill.
  - Decode with a 9-bit fast LUT, then a `max_code` compare on a 16-bit peek.
  - Check `valid` once per SOS.
- **Effort:** M
- **Verification:** Decoder output is identical, and a q=90 VGA benchmark shows the gain.

### CONV-06: Decoder per-pixel output loop does divides, a format match and bounds checks
- **Severity:** P2 · **Category:** performance
- **Location:** jpeg_codec.rs:1458-1503
- **Proposed fix:**
  - Use shifts, since the sampling factor is only 1 or 2.
  - Monomorphize the writer per output format.
  - Iterate rows with `chunks_exact_mut`.
- **Effort:** M
- **Verification:** Output is identical at odd image sizes.

### CONV-07: Grayscale JPEG with a non-1x1 sampling factor decodes scrambled
- **Severity:** P2 · **Category:** correctness
- **Location:** jpeg_codec.rs:1278-1295, 1396-1439
- **Problem:** A single-component scan always uses one 8x8 block per MCU (T.81 A.2.2), whatever sampling factor the header declares.
- **Proposed fix:** When `num_components == 1`, force an 8x8 MCU.
- **Effort:** S
- **Verification:** Patch a grayscale SOF to `0x22` and confirm the decoded pixels are identical.

### CONV-08: Decoder uses about 5 KB of stack
- **Severity:** P2 · **Category:** robustness
- **Location:** jpeg_codec.rs:891-897, 1151-1152, 1090, 215
- **Proposed fix:**
  - Store `max_code` and `val_offset` as i16.
  - Size `vals` to 162 or more.
  - Build tables in place.
  - Drop the i64 workspace (CONV-01).
- **Effort:** S
- **Verification:** Add `assert!(size_of::<DecHuffTable>() <= 640)` and measure the stack high-water mark.

### CONV-09: YUV range is inconsistent: full-range `rgb2yuv` but studio-range `yuv2rgb`
- **Severity:** P2 · **Category:** correctness
- **Location:** yuv.rs:1867-1876 vs 8-19, 1817-1831
- **Problem:** The round trip is not an identity. `encode_yuv422_to_jpeg` treats sensor YUV as full range, while the BMP and RGB paths treat it as studio range.
- **Proposed fix:** Pick one range per sensor, through a `YuvRange` parameter, and make `rgb2yuv` and the lookup table agree.
- **Effort:** M
- **Verification:** RGB565 → YUV → RGB565 stays within ±8 for all 65,536 colours.

### CONV-10: Backward EOI scan is unbounded; firmware passes the whole frame
- **Severity:** P2 · **Category:** performance/robustness
- **Location:** jpeg_scan.rs:139-219 (backward branch at 178-206); firmware main.rs:1937-1940, 2858-2859
- **Evidence:** `if let Some(eoi_rel) = verify_jpeg_eoi(&dst[soi_pos..written_len], false) {`
- **Problem:** A frame with no EOI makes the scan walk about 100-128 KB of PSRAM (about 2.5 ms) and evicts the cache. A stale EOI in the tail of the last chunk can be chosen instead of the real one. The HAL already bounds this scan with `eoi_probe_window`; the firmware does not.
- **Proposed fix:** Add `verify_jpeg_eoi_tail(buf, max_window)`, or slice the buffer with `eoi_probe_window` at both firmware call sites.
- **Effort:** S
- **Verification:**
  - Test a stale `FF D9` placed after the real EOI.
  - Test a frame with no EOI and assert the number of bytes touched stays bounded.

### CONV-11: "SWAR" scanners gain little on Xtensa and are copied three times
- **Severity:** P2 · **Category:** performance/maintainability
- **Location:** jpeg_scan.rs:41-81, 92-128, 139-219
- **Problem:** A u32 built from four `l8ui` loads costs about 15 instructions per 4 bytes, which is not better than a byte loop.
- **Proposed fix:**
  - Replace the copies with one simple iterator helper, e.g. `windows(2).rposition` on the bounded tail.
  - Measure the result.
- **Effort:** S
- **Verification:**
  - Keep the alignment test matrix.
  - Add a proptest against a naive reference.
  - Run a CCOUNT benchmark.

### CONV-12: YUV lookup table padded to 4 KB by `align(16)`
- **Severity:** P2 · **Category:** performance
- **Location:** yuv.rs:8-19, 1836-1852
- **Proposed fix:**
  - Drop `align(16)`.
  - Split into per-channel tables, or compute directly with 4 multiplies and benchmark both.
- **Effort:** S
- **Verification:**
  - Equality test over all 2^24 inputs on the host.
  - CCOUNT benchmark on device.

### CONV-13: Encoder pixel gather does redundant clamps and length checks
- **Severity:** P2 · **Category:** performance
- **Location:** jpeg_codec.rs:698-713, 829-857
- **Proposed fix:**
  - Fast path for interior MCUs using `as_chunks::<3>`.
  - Clamp only on edge MCUs.
  - Remove the length branches that validation has already guaranteed.
- **Effort:** S
- **Verification:** Output is byte-identical at both odd and full-MCU sizes.

### CONV-14: No fast path for DC-only blocks in the decoder
- **Severity:** P2 · **Category:** performance
- **Location:** jpeg_codec.rs:1082-1129, 229-233
- **Proposed fix:** Track `last_k`. If no AC coefficients were decoded, fill the block with the DC value.
- **Effort:** S
- **Verification:**
  - Equality test for every DC value in -2048..=2047.
  - Benchmark on a flat image.

### CONV-15: Two incompatible RGB565 unpack paths (truncation vs bit replication)
- **Severity:** P3 · **Category:** correctness
- **Location:** bmp.rs:195-199; yuv.rs:1879-1884
- **Proposed fix:** Use `unpack_rgb565_be_full` in both paths.
- **Effort:** S
- **Verification:** 0xFFFF maps to (255,255,255) on both paths.

### CONV-16: `parse_jpeg_dimensions` is lax about SOF length and junk between segments
- **Severity:** P3 · **Category:** robustness
- **Location:** jpeg_scan.rs:292-329
- **Proposed fix:**
  - Require `seg_len >= 11 && seg_len == 8 + 3*Nf`.
  - Return `None` when a segment boundary byte is not 0xFF.
  - Reject 0x00 and 0x01 markers before SOS.
- **Effort:** S
- **Verification:** Unit tests for a short SOF, a bad Nf, junk bytes, fill bytes, and an EXIF thumbnail.

### CONV-17: Restart markers are never checked for sequence
- **Severity:** P3 · **Category:** robustness
- **Location:** jpeg_codec.rs:1415-1428
- **Proposed fix:** Expect `RST(n mod 8)`. If it is missing, return `CorruptedFrame` or resync to the next marker.
- **Effort:** S
- **Verification:** Tests for missing and swapped RST markers.

### CONV-18: `frame_to_bmp` (JPEG) parses headers twice; grayscale padding is per byte
- **Severity:** P3 · **Category:** performance
- **Location:** bmp.rs:328-338, 244-255
- **Proposed fix:**
  - Check the expected dimensions right after SOF.
  - Pad rows with `chunks_exact_mut` and `fill(0)`.
- **Effort:** S
- **Verification:** `dst` is untouched when the dimensions don't match.

### CONV-19: `yuv422_to_yuv420` uses indexed loads and stores
- **Severity:** P3 · **Category:** performance
- **Location:** yuv.rs:2136-2162
- **Proposed fix:** Iterate row pairs with `chunks_exact` and `as_chunks::<4>`.
- **Effort:** S
- **Verification:** Equality test on random 64x32 input.

### CONV-20: Encoder cannot check output capacity up front
- **Severity:** P3 · **Category:** robustness
- **Location:** jpeg_codec.rs:645-666, 420-428
- **Proposed fix:** Export `const fn max_encoded_len(w, h, comps)` and fail fast when the output buffer is smaller.
- **Effort:** S
- **Verification:** Property test that encoding random noise always fits in the computed size.

### CONV-21: Duplicated and dead scanner API
- **Severity:** P3 · **Category:** maintainability
- **Location:** jpeg_scan.rs:22-29, 92-128, 250-264; HAL copy at driver.rs:42-49
- **Proposed fix:**
  - Make the HAL reuse `eoi_probe_window`.
  - Delete `verify_jpeg_soi_2byte` and `extract_first_jpeg_payload`, or gate them behind `cfg(test)`.
- **Effort:** S
- **Verification:** `cargo check` passes.

### CONV-22: No fuzzing or property tests on parsers that take untrusted input
- **Severity:** P2 · **Category:** tests
- **Location:** jpeg_scan.rs:351-519; jpeg_codec.rs:1541-1879
- **Problem:** No real OV2640 4:2:2 frame is used as a test fixture.
- **Proposed fix:**
  - Add `fuzz/` targets for `parse_jpeg_dimensions`, `validate_jpeg_frame` and `decode_jpeg_to_rgb888`.
  - Check in real OV2640 frames as fixtures.
  - Add proptests for the scanners.
- **Effort:** M
- **Verification:**
  - 10 minutes of fuzzing with no crash.
  - Fixture PSNR above 30 dB.

---

## TYPES

### TYPES-01: `CameraError` is 12 bytes because one variant carries a `&'static str`
- **Severity:** P2 · **Category:** performance
- **Location:** error.rs:8-45 (variant at line 30)
- **Evidence:** `SccbNack { slv_addr: u8, phase: &'static str },`
- **Problem:** Every `Result<_, CameraError>` returns through a hidden pointer. That covers the inner loops of the codec and every SCCB call.
- **Proposed fix:** Replace the string with `#[repr(u8)] enum SccbPhase`, which brings the error to 3 bytes.
- **Effort:** S (M across crates)
- **Verification:**
  - Add `const _: () = assert!(size_of::<CameraError>() <= 4);`.
  - Inspect the generated assembly for `decode_huff`.

### TYPES-02: `Display` uses `write!` for literal strings
- **Severity:** P3 · **Category:** code size
- **Location:** error.rs:47-74
- **Proposed fix:**
  - Use `f.write_str` for literal arms.
  - Add `const fn as_str()`.
- **Effort:** S
- **Verification:** `cargo bloat` before and after.

### TYPES-03: `CameraStatus` stores booleans as `u8`; `agc_gain` and `aec_value` are never validated
- **Severity:** P3 · **Category:** correctness
- **Location:** control.rs:72-101, 140-169
- **Proposed fix:**
  - Store the flags as `bool`.
  - Validate `agc_gain <= 30` and `aec_value <= 1200`.
  - Add a `clamp_level` helper.
- **Effort:** S
- **Verification:** Extend the existing validity test.

### TYPES-04: `CameraPid == u16` (range-aware) disagrees with `as_u16`
- **Severity:** P3 · **Category:** correctness
- **Location:** sensor.rs:66-104
- **Proposed fix:** Rename the comparison to `matches_pid` and drop the cross-type `PartialEq`.
- **Effort:** S
- **Verification:** A semantics test.

### TYPES-05: Tables indexed at runtime are `pub const` (possible duplication); table order is untested
- **Severity:** P3 · **Category:** maintainability
- **Location:** frame_size.rs:137-139, 143, 209; sensor.rs:257
- **Proposed fix:**
  - Mirror the tables as `static`.
  - Make `from_u8` a `const fn`.
  - Add an order-invariant test.
- **Effort:** S
- **Verification:**
  - Order test across all 25 entries.
  - `cargo bloat` duplicate check.

---

## SCCB

### SCCB-01: Masked writes are skipped when the value looks unchanged
- **Severity:** P2 · **Category:** correctness
- **Location:** bus.rs:26-48, 57-79
- **Problem:** This breaks registers whose write has a side effect, such as group-hold launch 0x3212, trigger and strobe bits, and reads taken while the bank cache is stale. C always writes.
- **Proposed fix:** Always write by default, and add an opt-in `write_reg8_mask_if_changed`.
- **Effort:** S
- **Verification:** Update `test_write_reg8_mask_fast_paths` and add a test for the opt-in variant.

### SCCB-02: Bulk writes issue one I2C transaction per register
- **Severity:** P2 · **Category:** performance
- **Location:** bus.rs:103-122; adapter.rs:96-104
- **Problem:** OV5640 init is about 300 registers, roughly 110 ms or more.
- **Proposed fix:**
  - Add `write_reg16_seq(slv, start, data)`, implemented with `i2c.transaction([Write(addr), Write(data)])`.
  - Coalesce contiguous runs in the sensor table writers.
- **Effort:** M
- **Verification:**
  - Mock register state is identical after a coalesced write.
  - On-device init time drops.

### SCCB-03: 16-bit value helpers are not atomic
- **Severity:** P3 · **Category:** correctness
- **Location:** bus.rs:82-100
- **Proposed fix:**
  - Document the limitation.
  - Use the `*_seq` variants.
  - Add a group-hold wrapper.
- **Effort:** S
- **Verification:** A mock test shows hold and launch wrapping the pair.

### SCCB-04: Error mapping drops the NACK source
- **Severity:** P3 · **Category:** correctness
- **Location:** adapter.rs:10-19
- **Proposed fix:** Map `NoAcknowledgeSource` into `SccbPhase` (see TYPES-01).
- **Effort:** S
- **Verification:** Extend `test_map_i2c_error_variants`.

### SCCB-05: Mock transaction log silently drops entries when full
- **Severity:** P2 · **Category:** tests
- **Location:** mock.rs:263-274
- **Proposed fix:**
  - Add a `dropped_count()`, or panic on overflow.
  - Add `assert_transactions`.
  - Add `fail_at(n)`.
  - Log failed operations.
- **Effort:** S
- **Verification:**
  - Overflow test.
  - `fail_at(3)` test.

### SCCB-06: Mock register store cannot model banks; 8-bit and 16-bit registers alias
- **Severity:** P3 · **Category:** tests
- **Location:** mock.rs:24-30, 222-261, 291
- **Proposed fix:**
  - Key registers by `(slv, width, bank, reg)`.
  - Add strict reads for unset registers.
- **Effort:** M
- **Verification:** An OV2640 test reads distinct values for register 0x44 in each bank.

---

## What looks good
- **jpeg_scan.rs is panic-free.** Segments are skipped by length, and EXIF thumbnails cannot spoof the SOF.
- **The firmware hot path is cheap.** It is a 256-byte SOI probe plus a backward EOI scan.
- **Encoder Huffman tables are built at compile time**, and the FDCT is an i32 AAN transform.
- **Overflow checks are thorough.** `checked_mul` is used throughout, and Kraft-violating tables are rejected.
- **YUV and BMP converters use `as_chunks`**, and RGB565 is consistently big-endian.
- **OV2640 bank caching** lives in the sensor layer and is invalidated on soft reset. 16-bit addresses are encoded big-endian.

## Open questions
1. **Overflow checks.** Are overflow checks ever enabled in a firmware profile? If so, the i32 IDCT must use `wrapping_*` operations.
2. **Stale tail bytes.** Can the tail of the last 1 KB chunk contain stale bytes from an earlier frame? This bears on CONV-10.
3. **YUV range.** Does the sensor output YUV in full range or studio range? This decides CONV-09.
4. **Zero-length writes.** Does esp-hal I2C accept a zero-length write, as used by `probe`?
5. **On-device decoding.** Will the decoder ever run on the device on untrusted JPEGs?
6. **Task stacks.** How large are the task stacks of any caller of the decoder?
7. **Sequential writes and bus speed.** Do the target modules accept OV5640/OV3660 sequential writes, and run at 400 kHz SCCB?
