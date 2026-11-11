# Findings: `esp32-cam-sensors` (SENS)

**Baseline commit:** `5a21ce2`.

**Scope:** all 18 drivers, with priority on `ov2640.rs`, plus `traits.rs`, `registry.rs` and `vendors.rs`.

**Comparisons with the upstream C driver** (espressif/esp32-camera) were made from the reviewer's own knowledge, because that driver is not in this repo. Uncertain comparisons are flagged.

## Summary

No wrong register was found on the production path. OV2640 at 1280x720 JPEG matches C in these settings:
- UXGA sensor mode.
- `CLKRC=0x00` and `R_DVP_SP=0x0C`.
- Window and zoom registers: `HSIZE 0x90`, `VSIZE 0xE1`, `YOFFL 0x96`, `VHYX 0x08`, `ZMOW 0x40`, `ZMOH 0xB4`, `ZMHH 0x01`.

The measured 12.53 FPS equals the OV2640's UXGA-mode readout rate at a 20 MHz XCLK. The sensor timing therefore sets the frame rate, not the clock divider.

OV2640 issues:
- The last commit changed the JPEG PCLK divider for CIF/SVGA from 8 (the C value) to 4. This was verified with `git show 75625d8`.
- The bank cache is not invalidated after a bus error.
- Several registers are written redundantly.
- The firmware re-sends every control after each framesize change.

Systemic issues across the other 17 drivers:
- Setters that return `Ok` without doing anything.
- Signed levels written raw (`level & 0xFF`).
- Full default tables re-sent on every framesize change.
- Probe order that writes stray bytes into 8-bit sensors.
- An OV3640 driver that uses OV3660 registers.

---

### SENS-01: The HD frame rate is capped by UXGA readout, not by the clock divider (the biggest FPS lever)
- **Severity:** P1 · **Category:** performance
- **Location:** ov2640.rs:1118-1135, 960-978; firmware main.rs:4307
- **Evidence:** `let mut mode = Ov2640SensorMode::Uxga;` … `(0, 0, 0, div)` · `frequency: Rate::from_mhz(20),`
- **Problem:** HD reads out all 1600x1200 rows. The DSP then crops to 1600x900 and scales to 1280x720.
  - At 20 MHz XCLK this gives about 12.5 fps, which matches the measured 12.53 FPS. At 24 MHz it would be about 15 fps.
  - About 25% of each frame period goes on 300 rows that are discarded.
- **Proposed fix:** Try three levers on hardware, one at a time:
  1. Raise XCLK to 24 MHz, which should give about 15 fps (+20%). LEDC cannot produce 24 MHz cleanly from 80 MHz (see LL-08), so use APLL or confirm the jitter is acceptable.
  2. Add an optional "HD fast" window. Program the sensor readout window (VSTART/VSTOP 0x19/0x1A, COM1 0x03, REG32) to 900 rows, and set HSIZE8/VSIZE8/SIZEL together to 1600x900 with YOFFL=0. This should give about 16 fps at 20 MHz. The OV-105 invariant (ov2640.rs:6-13) forbids only changing the DSP-side size alone.
  3. Leave the clk_2x doubler (CLKRC bit 7) for last, because it doubles PCLK.
- **Effort:** M
- **Verification:** Measure FPS on hardware and the EOI success rate. Add a mock-bus test for the alternate register set.

### SENS-02: The JPEG PCLK divider for CIF/SVGA is 4; C and the original port use 8
- **Severity:** P1 · **Category:** correctness
- **Location:** ov2640.rs:960-966, 1091-1099; test at 1436-1468
- **Evidence:** `Ov2640SensorMode::Cif | Ov2640SensorMode::Svga => 4,` · `} else { 0x04 };`. `git show 75625d8` shows the removed `8`.
- **Problem:** Halving the divider doubles the DVP PCLK for every framesize up to SVGA. A `/control` framesize change to SVGA or below can then exceed the I2S capture's sampling limit, which gives corrupt or truncated JPEGs. HD is not affected. The test now locks in the deviation.
- **Proposed fix:** Restore 8, or put 4 behind a documented opt-in only after it passes a hardware test. Update the test to expect 0x08.
- **Effort:** S
- **Verification:** Mock test that `R_DVP_SP == 0x08` at SVGA. Run a VGA/SVGA stream on hardware for 10 minutes with no bad JPEG headers or EOIs.

### SENS-03: The OV2640 bank cache is not invalidated after a bus error
- **Severity:** P1 · **Category:** correctness
- **Location:** ov2640.rs:822-831, 850-865, 833-848
- **Evidence:** `if self.reg_bank != bank { bus.write_reg8(self.slv_addr, BANK_SEL, bank as u8)?; self.reg_bank = bank; }`
- **Problem:** After a failed transaction, `reg_bank` can disagree with the hardware, so DSP values land in sensor registers or the other way round. This happens silently, because the firmware ignores errors from `/control` setters (main.rs:3883-3916).
- **Proposed fix:** When `write_reg`, `read_reg` or `write_regs` returns any `Err`, set `self.reg_bank = Ov2640Bank::Max` so the next access re-selects the bank.
- **Effort:** S
- **Verification:** Use a mock that injects a fault on the Nth transaction, and assert the next access re-sends 0xFF.

### SENS-04: OV3640 uses OV3660 register addresses for window, PLL, format and JPEG quality
- **Severity:** P1 (P0 if any OV3640 user exists) · **Category:** correctness
- **Location:** ov3640.rs:55-75, 84, 202-206, 880-943
- **Evidence:** `pub const X_ADDR_ST_H: u16 = 0x3800;` · `pub const SC_PLLS_CTRL0: u16 = 0x303A;` · `pub const RATIO_TABLE: [RatioSettings; 9] = crate::ov3660::RATIO_TABLE;`
- **Problem:** The OV3640 probably has none of these registers, so framesize, quality and clock changes do nothing or corrupt state. This is uncertain: upstream may not have an `ov3640.c`.
- **Proposed fix:** Port the OV3640 window/format/JPEG registers from an OV3640 reference. Until then, return `NotSupported` from the affected setters.
- **Effort:** L
- **Verification:** Hardware test with an OV3640. Mock test that nothing is written to 0x38xx, 0x44xx or 0x46xx.

### SENS-05: NT99141 `set_ae_level` clamps to 0..8, so level 0 picks the darkest preset
- **Severity:** P1 · **Category:** correctness
- **Location:** nt99141.rs:1074-1083, 788, 237-283
- **Evidence:** `let clamped = level.clamp(0, 8) as usize; let base = clamped * 5;`
- **Proposed fix:** Use index `(level.clamp(-4, 4) + 4) * 5`. Confirm this against C.
- **Effort:** S
- **Verification:** Mock test that `reset()` writes 0x32B8=0x3B.

### SENS-06: Redundant writes in OV2640 `set_framesize` and at boot
- **Severity:** P2 · **Category:** performance
- **Location:** ov2640.rs:994-1002, 1091-1099; firmware main.rs:2460-2462
- **Problem:** One HD `set_framesize` is about 71 SCCB writes plus 20 ms of delays:
  - CLKRC is written twice and R_DVP_SP three times, with the same values.
  - Two extra bank flips.
  - At boot, `set_pixformat` runs twice.
- **Proposed fix:**
  - Add a private `apply_format_regs` that writes JPEG3 and then R_DVP_SP once.
  - Write CLKRC inside the sensor-bank section.
  - Have the firmware skip the explicit `set_pixformat` before `set_framesize`.
- **Effort:** S
- **Verification:** Mock count test: HD `set_framesize` takes 64 writes or fewer, and writes 0x11 and 0xD3 once each.

### SENS-07: The firmware re-sends all 11 controls on every framesize change and ignores failures
- **Severity:** P2 · **Category:** performance
- **Location:** firmware main.rs:3881-3921
- **Evidence:** `if fs_changed || ctrl.brightness != last_applied_ctrl.brightness {` … `let _ = ov.set_framesize(sccb, ctrl.framesize);` … `last_applied_ctrl = ctrl;`
- **Problem:** The framesize tables do not touch these controls. Re-sending them costs about 40 extra transactions (about 8 ms) while Core 1 is paused. The applied state is also recorded even when a write fails.
- **Proposed fix:** After a framesize change, re-apply quality only. Update `last_applied_ctrl` only for controls whose write succeeded. This overlaps with FW-CORE-21.
- **Effort:** S
- **Verification:** On hardware, change framesize and read back REG04, CTRL1, COM8, AEW and the effect registers.

### SENS-08: SCCB runs at 200 kHz; OV2640 supports 400 kHz, and a log line says 100 kHz
- **Severity:** P2 · **Category:** performance
- **Location:** firmware main.rs:4344 (verified), 2480
- **Evidence:** `I2cConfig::default().with_frequency(Rate::from_khz(200));` · `"... (GPIO26/GPIO27 @ 100kHz): "`
- **Problem:** About 290 transactions run at boot and about 110 on each framesize change, and both scale with the bus speed.
- **Proposed fix:** Use 400 kHz if a hardware check shows clean signal edges. Fix the log line.
- **Effort:** S
- **Verification:** Boot-time measurement, a logic-analyser check, and a 24 h `/control` stress test.

### SENS-09: `detect_sensor` probes SC031GS (16-bit addresses) at 0x30 before OV2640, writing stray data into it
- **Severity:** P2 · **Category:** correctness
- **Location:** registry.rs:30-52
- **Problem:** The 16-bit read of 0x3107 sends [0x31,0x07]. An 8-bit OV2640 treats that as a write of 0x07 to register 0x31. Address 0x3C has the same problem for GC2145.
- **Proposed fix:** Probe 8-bit sensors before 16-bit ones at each shared address. Alternatively, probe address ACK only first.
- **Effort:** S
- **Verification:** Registry mock test with an OV2640: no `Write8{reg:0x31}` appears.

### SENS-10: HM0360 `set_framesize(240x240)` returns `Ok` but the sensor stays at VGA
- **Severity:** P2 · **Category:** correctness
- **Location:** hm0360.rs:817-841, 501, 420
- **Proposed fix:** Add the 240x240 arm. Return `FailedToSetFrameSize` for any size that has no table.
- **Effort:** S
- **Verification:** Mock test.

### SENS-11: NT99141 brightness 0 writes 0x00 to 0x32F2, which contrast sets to 0x80
- **Severity:** P2 · **Category:** correctness
- **Location:** nt99141.rs:872-904
- **Proposed fix:** Map 0 to 0x80. Reject levels outside -3..=3.
- **Effort:** S
- **Verification:** Mock test.

### SENS-12: HM1055 `set_pll` uses decimal 10/11 where the pattern requires 0x10/0x11
- **Severity:** P2 · **Category:** correctness
- **Location:** hm1055.rs:1327-1342
- **Evidence:** `204 => 10, 216 => 11, 228 => 0x12,`
- **Proposed fix:** Compute `0x10 + (mul - 204) / 12` with a range check.
- **Effort:** S
- **Verification:** Unit test for every multiplier.

### SENS-13: Many setters return `Ok` without touching hardware, against the trait contract
- **Severity:** P2 · **Category:** correctness
- **Location:** traits.rs:114-118; nt99141.rs:936-946, 967-970; hm1055.rs:1140-1143, 1277-1294; ov2640.rs:1421-1427
- **Proposed fix:** Delete these overrides so the `NotSupported` defaults apply. Rename `set_xclk` or document what it actually does.
- **Effort:** S
- **Verification:** Per-driver tests that the setters return `NotSupported`.

### SENS-14: Signed levels written raw and truncated (`level & 0xFF`, `as i8`)
- **Severity:** P2 · **Category:** correctness
- **Location:** ov7725.rs:506-516; bf3005.rs:396-420; bf20a6.rs:416-423; hm1055.rs:1079-1137; sc101iot.rs:449-470; sc030iot.rs:469-488; nt99141.rs:948-952
- **Problem:** A level of -2 is written as 0xFE, the opposite extreme.
- **Proposed fix:** Add a shared `level_in(level, lo, hi)` helper and per-sensor lookup tables.
- **Effort:** M
- **Verification:** Table-driven test for each driver.

### SENS-15: HM1055, HM0360 and NT99141 re-send the whole default table, with 100 ms delays, on every `set_framesize`
- **Severity:** P2 · **Category:** performance
- **Location:** hm1055.rs:1066; hm0360.rs:822 (table at line 70); nt99141.rs:858
- **Problem:** This costs up to about 200 ms per framesize change and undoes the user's earlier control settings.
- **Proposed fix:** Write only the framesize-specific delta.
- **Effort:** M
- **Verification:** Mock budget tests on transaction count and delay.

### SENS-16: OV5640 autofocus firmware is uploaded one byte per I2C transaction
- **Severity:** P2 · **Category:** performance
- **Location:** ov5640.rs:851-870
- **Problem:** 4,077 transactions, about 0.8 s at 200 kHz.
- **Proposed fix:** Burst-write in 256-byte chunks using SCCB-02's `write_reg16_seq`, which brings it to about 0.2 s.
- **Effort:** M (shared with SCCB-02)
- **Verification:** Mock test of burst content, and AF init time on hardware.

### SENS-17: The mock bus does not model OV2640 banks, and no test pins transaction counts or delays
- **Severity:** P2 · **Category:** tests
- **Location:** esp32-cam-sccb/src/mock.rs:203; ov2640.rs:1436-1468; omnivision.rs:22-134
- **Proposed fix:**
  - Add a banked mock (overlaps SCCB-06).
  - Add golden tests: reset is 159 writes and 10 ms; HD `set_framesize` has a write budget; `set_quality` is one write.
  - Assert the ordered transaction list.
- **Effort:** M
- **Verification:** The new tests, gated in CI.

### SENS-18: Register-table end and delay markers collide or truncate silently
- **Severity:** P2 · **Category:** correctness
- **Location:** bf20a6.rs:15-16, 168-180; hm1055.rs:111-112, 834-845; gc032a.rs:72-73, 447-463; gc2145.rs:55-56, 778-794; hm0360.rs:65-66; nt99141.rs:88-89
- **Problem:** In two drivers `REG_DLY == REGLIST_TAIL == 0xFFFF`. In others the end marker is 0x0000, so a write to register 0x0000 ends the table early.
- **Proposed fix:** Drop end markers, since slices carry their length. Use an `enum RegOp { W(..), Delay(..) }` or a single table writer.
- **Effort:** S
- **Verification:** Test that the count written equals the slice length for every table.

### SENS-19: OV2640 `init_status` uses about 33 transactions where 14 would do
- **Severity:** P3 · **Category:** performance
- **Location:** ov2640.rs:1031-1073
- **Proposed fix:** Read each register once, grouped by bank, and decode locally.
- **Effort:** S
- **Verification:** Mock count test.

### SENS-20: OV2640 `set_reg_bits` always writes, and toggles always read first
- **Severity:** P3 · **Category:** performance
- **Location:** ov2640.rs:877-890, 1212-1242
- **Proposed fix:** Skip the write when the value is unchanged. This is safe for these plain storage registers; see the SCCB-01 caveat. Optionally keep shadows of REG04, COM8 and CTRL1.
- **Effort:** S
- **Verification:** Mock count test.

### SENS-21: OV2640 quality is written raw like C, but out-of-range values are rejected, not clamped; the default is 12, not 10
- **Severity:** P3
- **Location:** ov2640.rs:1198-1205; config.rs:43
- **Proposed fix:** Document the behaviour. Decide whether the intended default is 10 (the design spec) or 12.
- **Effort:** S
- **Verification:** The existing omnivision.rs:73-78 test still passes.

### SENS-22: No per-sensor cargo features; 8-bit-address tables stored as `Reg16`
- **Severity:** P3 · **Category:** binary-size
- **Location:** esp32-cam-sensors/Cargo.toml; esp32-cam-hal/Cargo.toml:12,16; gc2145.rs:58; gc032a.rs:75; bf20a6.rs:18
- **Problem:** The firmware probably links only OV2640 today; this is not verified with `cargo bloat`. A single `detect_sensor` call would pull in every driver (about 20 KB of tables). The GC/BF tables waste about 2 KB as `Reg16`.
- **Proposed fix:** Add per-sensor features. Store those tables as `Reg8`. Use `static` for large tables.
- **Effort:** M
- **Verification:** `cargo bloat` before and after.

### SENS-23: About 1,500 lines duplicated across OV3660, OV5640, OV3640 and HM1055
- **Severity:** P3 · **Category:** maintainability
- **Location:** ov3660.rs:674-720 vs ov5640.rs:1179-1225 vs ov3640.rs:701-746; ov3660.rs:1174-1221 vs ov5640.rs:1689-1736 vs hm1055.rs:1232-1275; traits.rs:41-63 vs bus.rs:103-122
- **Proposed fix:** Create an `omnivision_common` module, one masked get/set, and one table writer.
- **Effort:** L
- **Verification:** Existing tests stay green.

### SENS-24: Unsupported framesizes are handled inconsistently, and `status` is updated before writes that can fail
- **Severity:** P3 · **Category:** maintainability
- **Location:** ov3660.rs:831-833; ov2640.rs:1106-1108; gc032a.rs:671-676; gc0308.rs:698-703; mega_ccm.rs:242-246; ov2640.rs:1227-1230 vs 1232-1241
- **Proposed fix:** One policy: return `FailedToSetFrameSize`, never clamp. Update `status` only after a successful write.
- **Effort:** S
- **Verification:** A shared conformance test run across all 18 drivers.

### SENS-25: Minor deviations from C: OV7725 threshold and format mask, GC0308 effects, OV3640 saturation clamp
- **Severity:** P3 · **Category:** correctness
- **Location:** ov7725.rs:454, 485; gc0308.rs:859-863; ov3640.rs:976-982
- **Proposed fix:** Match C, or document the deviation. Return `NotSupported` or `InvalidArg` instead of remapping silently.
- **Effort:** S
- **Verification:** Mock test for each case.

---

## Coverage table
| File | Lines | Tests | Notable deviation from C |
|---|---|---|---|
| ov2640.rs | 1469 | y | PCLK divider 4 vs 8 (SENS-02); bank cache on error (SENS-03) |
| ov3660.rs | 1304 | y | none found |
| ov3640.rs | 1316 | y | OV3660 registers (SENS-04); saturation clamp |
| ov5640.rs | 1819 | y | AF upload byte-at-a-time (SENS-16) |
| ov7670.rs | 553 | y | none found |
| ov7725.rs | 677 | y | QVGA/VGA threshold; YUV mask; raw signed writes |
| omnivision.rs | 607 | tests only | n/a |
| nt99141.rs | 1231 | y | AE indexing (SENS-05); brightness 0 (SENS-11); no-op setters |
| gc2145.rs | 1154 | y | subsample table unverified; `Reg16` for 8-bit registers |
| gc0308.rs | 923 | y | effects 3-6 collapse to grayscale |
| gc032a.rs | 763 | y | none found |
| bf3005.rs | 562 | y | end-of-table check (open question 4); raw signed writes |
| bf20a6.rs | 484 | y | delay and end markers are both 0xFFFF |
| sc101iot.rs | 559 | y | no range checks (matches C) |
| sc030iot.rs | 571 | y | levels 0..0xFFFF stored as `i8` |
| sc031gs.rs | 655 | y | probed before OV2640 (SENS-09) |
| mega_ccm.rs | 392 | y | none found |
| hm1055.rs | 1365 | y | PLL 10/11 (SENS-12); full table re-sent; no-op quality setter |
| hm0360.rs | 967 | y | 240x240 table unused (SENS-10); SCCB address unverified |

## What looks good
- **OV2640 HD path:** matches C. HSIZE8/VSIZE8/SIZEL stay at UXGA.
- **Bank-select cache:** removes redundant 0xFF writes, and soft reset invalidates it correctly.
- **`set_quality`:** a single QS write. `set_framesize` sends only the mode delta.
- **Register tables:** const slices, with no allocation. `no_std` and `forbid(unsafe)` hold throughout.
- **OV2640 level indexing:** `level + 3`, with range checks.
- **Dispatch:** generic `CameraSensor<B>` with `NotSupported` defaults. The firmware uses `Ov2640` with static dispatch.
- **`detect_sensor`:** exits on the first PID match, and the whole probe costs about 18 short transactions.
- **OV3660 and OV5640 ports:** match C.

## Open questions
1. **OV2640 "negative" special effect** (ov2640.rs:640): C is believed to have `0x00` where Rust has `0x05`. Diff against upstream.
2. **OV3640 source:** did upstream ever ship an `ov3640.c`? If so, which registers does it use?
3. **HM0360 SCCB address:** is it 0x12 or 0x24?
4. **BF3005 reset table:** do the end-of-table entry and the `(0xFF,0xFF)` entry mean the same as in C?
5. **GC2145 subsample table:** does upstream have 8 entries?
6. **NT99141:** do the brightness and AE quirks copy upstream?
7. **`RESET_JPEG` and QS:** does the `RESET_JPEG` pulse reset QS (0x44)? If not, quality need not be re-applied after a framesize change.
8. **PCLK divider 4:** was it validated on hardware at VGA/SVGA?
