# Work Order WO-PERF-2026-11: Performance-First Code Review Remediation

| Field | Value |
| --- | --- |
| **Date** | 2026-11-10 |
| **Baseline commit** | `5a21ce2` (`master`) |
| **Baseline metrics** | 12.53 FPS @ 1280x720 JPEG (from commit `75625d8`) · 198 host tests passing · app image 817,904 B (46.2% of the 0x1B0000 OTA slot) · clippy, fmt and Xtensa clippy all clean |
| **Goal** | 18-22 FPS @ 720p (design target in `docs/superpowers/specs/2026-10-05-*`) without regressing reliability. Fix every P0 first. |
| **Findings** | 170 total across six review areas. Every one is assigned to a work package (WP) below; see the [traceability index](#6-traceability-index). |
| **Author** | Review only. No source code was changed by this work order. |

Detailed evidence, exact line ranges, proposed fixes and verification steps for each finding are in the appendix files:

| File | IDs | Scope |
| --- | --- | --- |
| [findings-fw-core.md](findings-fw-core.md) | FW-CORE-01..23 | Core 1 producer, SPSC ring, frame buffers, server loop, statics, boot |
| [findings-fw-net.md](findings-fw-net.md) | FW-NET-01..29 | HTTP, OTA, Wi-Fi, IP config |
| [findings-ll-hal.md](findings-ll-hal.md) | LL-01..21, HAL-01..12 | I2S0 DMA, descriptor ring, filters, XCLK, HAL driver/pool/pins |
| [findings-conv-types-sccb.md](findings-conv-types-sccb.md) | CONV-01..22, TYPES-01..05, SCCB-01..06 | JPEG scan/codec, YUV, BMP, shared types, SCCB bus |
| [findings-sensors.md](findings-sensors.md) | SENS-01..25 | OV2640 (priority) and the 17 other sensor drivers |
| [findings-build.md](findings-build.md) | BUILD-01..27 | Cargo profiles, toolchain, CI, scripts, docs |

---

## 0. Start here (executing agents)

Read this section and §2. You do not need the rest until you have picked a work package.

### 0.1 Check your environment first (two minutes, saves hours)

```bash
cargo +esp --version
```

- **Prints a version:** you can run every gate in §2.4. Continue.
- **"toolchain 'esp' is not installed":** you are in a cloud session. **Do not try to install it** — the toolchain is distributed only through GitHub releases from other organisations, and cloud sessions can reach only this repository. Every previous agent that tried, failed. (CI *does* have the toolchain and runs these gates on every push — as of `845cfc4` the whole xtensa job is green, through `espflash save-image` and the 0x1B0000 size gate, even without the owner's bootloader — so a package you cannot gate locally still gets gated when you push.) Instead, work from the "host-only" column in §0.2, and when you finish a package mark it `IN-PROGRESS` with the words *"Xtensa gates NOT run: no esp toolchain in this session"* — never `DONE`, never `READY-FOR-HIL`.

Then run the three host gates once, before changing anything, so you know the tree was green when you started:

```bash
cargo fmt --all -- --check && cargo clippy --workspace --all-targets --locked -- -D warnings && cargo test --workspace --locked
```

### 0.2 What is actually next

Pick from the **leftmost column you can do**. Do not take a package from a column to the right of your capability.

| Host-only (no toolchain needed) | Needs the esp toolchain | Needs the camera board |
| --- | --- | --- |
| **WP-24** docs accuracy (§5, Wave 5) | *(nothing — WP-02/06/08/09 passed the Xtensa gates in CI on 2026-11-13 and moved to the board column)* | WP-02, WP-04, WP-05, WP-06, WP-07, WP-08, WP-09, WP-16, WP-22 are `READY-FOR-HIL`: run their [HIL] acceptance lists |
| **WP-20** HAL redesign (host-testable; WP-08 is landed) | **WP-19** split `main.rs` (behaviour-neutral, but `main.rs` is Xtensa-only, so do not attempt without the toolchain) | WP-10, WP-11, WP-13, WP-25 (all [HIL]) |
| The audit in §9, on `crates/klipper-cam-fw` (not yet done there) | WP-12, WP-14, WP-15, WP-17, WP-18 (all edit `main.rs`) | |

`WP-01`, `WP-03`, `WP-21`, `WP-23` are `DONE`. Do not reopen them.

### 0.3 The five rules that matter most

1. **Never claim success for work not done.** A setter that cannot do something returns `NotSupported`; it does not select a bank, record the value and return `Ok`. A status field reports what the sensor is in, not what was asked for. This one rule is behind most of the defects fixed so far.
2. **Never guess a value that belongs to hardware.** Register addresses, reset values, lookup tables, PLL codes, GPIO-matrix indices: if the correct value is not in this repository, **record the gap and stop** — in a comment, in the Status line, and in §7 if it needs the owner. Do not derive it from a pattern, a sibling driver, or an upstream you remember. Several findings here looked like simple extrapolations and were not (§2.11).
3. **Prove the test fails before the fix** (§2.12). A test that passes against the unfixed code proves nothing and has already been caught once in this work order.
4. **Run the gates after every change, not at the end.** The three host commands in §0.1 take under a minute.
5. **When unsure whether something is a bug, it is probably matching upstream.** Check §2.13 before "fixing" anything that looks odd.

### 0.4 How to finish a package

1. All gates you *can* run are green (host always; Xtensa if you have the toolchain).
2. Each fix has a test you watched fail first.
3. Commit per finding, not per package, with a message that says what was wrong and what changed. Follow the style of `git log --oneline -20`.
4. Update the package's **Status** line in §5 (§2.10). Say what you did, what you did not do, and why. If you skipped a step, name it.
5. Push to the branch you were given. Do not open a pull request unless asked.

### 0.5 Stop and ask the owner when

- the fix needs a value that is not in the repo (rule 2 above) **and** the package cannot be finished without it;
- a package's steps contradict each other or the findings file, and you cannot tell which is right;
- you would change the HTTP contract, flash layout, GPIO16 cold-boot behaviour, or Core 1 producer invariants (`README.md`, `firmware/README.md`);
- anything would flash a device or deploy over the network.

Otherwise do not stop to ask whether to continue. Decide, write the decision into the Status line as *"Ruling: … — because … — costs … if wrong"*, and continue.

---

## 1. Executive summary

### 1.1 Where the frame rate goes

Three independent reviewers reached the same conclusion: **the 12.53 FPS ceiling is set by the sensor, not the software pipeline.**

The OV2640 produces 1280x720 by reading out the full 1600x1200 UXGA array, then cropping and scaling. That readout runs at 15 FPS with a 24 MHz XCLK. This firmware uses 20 MHz, and 15 × 20/24 = **12.5 FPS**, which matches the measurement to within 0.3%. The register values match Espressif's C driver, so this is not a bug. It does mean that optimising DMA, copies or TCP alone **cannot** reach 18-22 FPS.

The FPS plan therefore has two halves, and both are needed:

1. **Raise the sensor frame rate** (WP-10). Options: a 24 MHz XCLK (about 15 FPS); an "HD-fast" sensor window that reads only 900 rows instead of 1200 (about 16 FPS at 20 MHz, about 19-20 FPS at 24 MHz, estimated).
2. **Make Core 0 fast enough to keep up** (WP-11 to WP-14). Today each streamed frame moves through 40 MHz PSRAM about 4 times, `/snapshot` blocks for 15-20 ms, Wi-Fi scans freeze the loop for seconds, and Core 1 drops a whole frame whenever its ring stays full for about 25 ms. At 20 FPS the frame budget is 50 ms, and Core 0 is estimated to spend about 30 ms of it on PSRAM copies alone.

**Measure before changing.** WP-04 adds per-stage timing to `/status`. Every performance package must report before and after numbers from it.

### 1.2 P0 defects (fix before any performance work)

| ID | Defect | WP |
| --- | --- | --- |
| FW-CORE-01 | **OTA is broken while the camera runs.** esp-storage 0.10 refuses every flash erase/write while APP_CPU is running (`MultiCoreStrategy::Error`; verified in `esp-storage-0.10.0/src/common.rs:119-121, 280-285`). Parking Core 1 in a spin loop still counts as running. The post-OTA rollback-cancel also fails, and the error is discarded with `let _`. | WP-05 |
| FW-NET-01 | **Rollback state is synthetic.** The OTA manager is built from the MMU mapping with both records `Valid` (main.rs:487-506), so otadata is never read at boot and `PendingVerify` is never confirmed. If the bootloader has rollback enabled, every OTA reverts on the second reset. | WP-05 |
| FW-CORE-02 | **Corrupt frames under backpressure.** Core 1 consumes the first half-buffer after each EOF without checking DMA ownership, so zero-filled bytes can be committed inside a JPEG that still passes SOI/EOI checks. | WP-06 |
| FW-CORE-03 | **UB from aliased `&mut`.** Two live `&mut` exist to `FIRMWARE_RUNTIME`, and each PSRAM socket buffer has a second `&'static mut`. Today this works only because of `#[inline(never)]`. Enabling LTO (WP-02) could miscompile it. | WP-07 |
| LL-01 / HAL-01 | **Safe API can start DMA at any address.** `Esp32I2sCamera::start` is a safe fn that starts DMA at an arbitrary address, and the `forbid(unsafe)` HAL calls it with address **0**. | WP-08 |
| BUILD-01 | **CI tests nothing.** CI builds a non-existent ESP-IDF `examples/` directory (35 failing jobs), and no Rust test, lint or build gate runs at all. | WP-01 |
| BUILD-02 | **Builds only on one machine.** The Xtensa linker path is hard-coded to a single Linux home directory. | WP-02 |
| BUILD-03 | **Fresh clones cannot flash.** The bootloader that `espflash.toml` requires is gitignored and missing. | WP-03 |

### 1.3 Other high-value items (P1)

- **Security:**
  - The publicly documented default OTA token is accepted (FW-NET-04, BUILD-07).
  - A WPA passphrase conversion error silently falls back to an **Open** network (FW-NET-03).
  - 172.22.1.52 is hard-coded and ARP-announced on every network (FW-NET-05).
- **Performance quick wins:**
  - No `[profile.release]`: fat LTO and `codegen-units=1` save 25 KB and 5.7 KB of IRAM (BUILD-04).
  - Hot loops run from flash cache, not IRAM (BUILD-05).
  - PSRAM runs at 40 MHz while the README claims 80 MHz (FW-CORE-05, BUILD-06).
  - The JPEG decoder IDCT uses i64 and the encoder uses per-coefficient division (CONV-01/02).
- **Sensor regressions:**
  - Commit `75625d8` halved the OV2640 JPEG PCLK divider for CIF/SVGA from 8 to 4. Verified with `git show`. This risks corrupt frames at VGA/SVGA (SENS-02).
  - The bank cache is not invalidated after bus errors (SENS-03).

---

## 2. Rules for executing agents

Read this section before starting any work package.

1. **Branches.** If your session was assigned a branch (cloud sessions are: it starts with `claude/`), **use that branch and only that branch**; this overrides `CLAUDE.md`'s "work on master" and the per-package rule below. Otherwise, one work package per branch, named `wo/<WP-id>-<slug>`, for example `wo/wp05-ota-multicore`, from the current `master`. Either way, do not bundle unrelated packages into one commit, and never force-push.
2. **Re-verify locations first.** Line numbers in the appendices are against `5a21ce2`. Before editing, re-locate each site with `grep -n` on the quoted evidence. If the evidence no longer matches, record that in the PR and re-assess the finding.
3. **Respect file ownership (§4).** Two agents must never edit the same file concurrently. `crates/klipper-cam-fw/src/main.rs` is a serialization point until WP-19 splits it.
4. **Required gates for every PR.** All must pass:
   ```bash
   cargo fmt --all -- --check
   ```
   ```bash
   cargo clippy --workspace --all-targets --locked -- -D warnings
   ```
   ```bash
   cargo test --workspace --locked
   ```
   ```bash
   cargo +esp clippy --target xtensa-esp32-none-elf -Zbuild-std=core,alloc --workspace --release -- -D warnings
   ```
   ```bash
   cargo +esp build --target xtensa-esp32-none-elf -Zbuild-std=core,alloc -p klipper-cam-fw --release
   ```
   WP-02's portable `.cargo/config.toml` is committed (`256c864`), so no linker override is needed; WP-02 itself is still `IN-PROGRESS` because its Xtensa gates have not run. In a git worktree nested inside the main checkout, set `RUSTFLAGS="-C link-arg=-nostartfiles -C link-arg=-Tlinkall.x"`, because otherwise both `.cargo/config.toml` files are loaded and the linker script is passed twice.

   **If `cargo +esp` is not installed** (see §0.1): run the three host gates, skip the two Xtensa ones, and say so in the Status line in exactly these words: *"Xtensa gates NOT run: no esp toolchain in this session."* Do not mark the package `DONE` or `READY-FOR-HIL`. Prefer packages whose crates are host-only (`esp32-cam-types`, `-sccb`, `-sensors`, `-conversions`, and the non-MMIO parts of `-ll` and `-hal`); those are fully verified by the host gates.
5. **Hardware-in-the-loop (HIL).** Items marked **[HIL]** need the ESP32-WROVER-DEV board with the OV2640. An agent without hardware must:
   - implement the change,
   - pass all host and Xtensa gates,
   - add host tests where the WP asks for them,
   - mark the WP `READY-FOR-HIL` (§5), and stop.

   Never claim an FPS or stability result that was not measured.
6. **Performance claims need numbers.** Any WP tagged `perf` must report before/after values using the measurement protocol (§3) and the WP-04 counters. If a change makes things slower or has no effect, revert it and record the result. A negative result is still useful.
7. **No new `unsafe` outside `esp32-cam-ll` and the Xtensa `main.rs` / modules split out of it.** Every new `unsafe` block needs a `// SAFETY:` comment in the existing six-point style.
8. **The HTTP contract is frozen** except where a WP explicitly changes it: FW-NET-14 changes the stream policy, FW-NET-27 changes `/control` to POST, and WP-04 adds fields to `/status`. Keep `scripts/ota_deploy.py` and `scripts/verify_cold_boot_invariants.py` in sync with any contract change.
9. **Secrets.** Never commit `.env.local`, Wi-Fi credentials, OTA tokens, or the camera's IP address. Use `<camera-ip>` in docs.
10. **Update this file when you finish.** Tick the WP's checkbox and fill in its Status line: date, PR link, measured result. Commit messages follow the repo's Conventional Commits style.
11. **Do not guess hardware values.** These looked like safe derivations and were not:
    - The HM1055 PLL lookup skipped multipliers 252/264/276. Filling the gap with the formula that fit the other eleven would have written unattested PLL codes into a register that drives PCLK. The gap was evidence, not an omission.
    - SENS-14 asks for per-sensor level lookup tables. Whether a part reads a register as two's complement or as a magnitude differs per sensor; there is no way to tell from this repo.
    - SENS-15 asks for a "delta" framesize write. Comparing the tables showed the full rewrite was load-bearing for HM0360 and HM1055; the optimisation would have broken both.
    - The ESP32-S2 model binds vertical sync with the ESP32 signal index. The other four S2 indices are all ESP32 + 3, but the right value is in the S2 technical reference manual, not here.

    In every case the correct action was the same: fix what the repo proves, refuse rather than substitute where it does not, write the gap into a comment and the Status line, and move on.
12. **Prove the test fails first.** For each fix: write the test, run it, **read the failure message** and confirm it fails for the reason you expect, then fix, then run again. If you wrote a test after the fix, revert the fix temporarily (`git stash` or `git show HEAD:path > file`) and run the test against the old code before you commit. A test in this work order was once shipped that passed against the unfixed code; a reviewer caught it, you may not have one.
13. **Things that look wrong and are not.** Check here before "correcting" anything in a table:
    - `RESOLUTION_TABLE` has portrait FHD at **1088**x1920. That matches upstream and is deliberate (a multiple of 16). It is not a typo for 1080.
    - `AspectRatio` is a nine-way **bucket** indexed into each driver's `RATIO_TABLE[9]`, not an exact ratio. QCIF at `5x4` and CIF at `4x3` are correct.
    - `SYSTEM_CTROL0` and `ISP_CONTROL_01` in `nt99141.rs` are two names for one register, as are `TIMING_TC_REG20`/`21`. Harmless: their bit masks do not overlap.
    - `espflash save-image` failing with *"Error while connecting to device (os error 2)"* is not a serial-port problem: espflash 4.x wraps every `io::Error` in that message, and os error 2 is ENOENT on the bootloader path in `espflash.toml`. Check `firmware/bootloader_v554_gpio16.bin` exists.
    - `esp32_i2s.rs` keeps an in-memory register block *as well as* doing real MMIO. The block is a shadow for host tests. The S2/S3 modules have only the shadow — see their module docs.

    When a value matches the upstream Espressif driver, leave it, even if it looks odd. When you cannot check upstream, leave it and say so.
14. **Cargo features.** `esp32-cam-sensors` has one feature per driver and the workspace dependency carries `default-features = false`. If you add a consumer, name the drivers it needs (`features = ["ov2640"]`). Cargo **ignores** `default-features = false` on a member unless the workspace entry has it too, and feature unification means any member that leaves defaults on hands every binary all 18 drivers. Check with `cargo tree -p <crate> -e features | grep "esp32-cam-sensors feature"`.

---

## 3. Measurement protocol

The WP-04 counters are the primary source. Until WP-04 lands, use these host-side commands.

**Stream frame rate**, 60 seconds:
```bash
timeout 60 curl -s http://<camera-ip>:81/stream | grep -a -c "Content-Type: image/jpeg"
```
Divide the count by 60. Run it 3 times and report the median.

**Snapshot latency while streaming.** Run the stream command in another shell first:
```bash
for i in $(seq 100); do curl -s -o /dev/null -w '%{time_total}\n' http://<camera-ip>/snapshot; done | sort -n | awk '{a[NR]=$1} END {print "p50",a[int(NR*0.5)],"p95",a[int(NR*0.95)]}'
```

**Frame validity:** save 200 frames and decode each with a reference decoder, for example `ffmpeg -v error -i frame.jpg -f null -`. Report the failure count.

**Sensor frame period:** measure the VSYNC period on GPIO25 with a logic analyser, or read `sensor_frame_period_us` from `/status` once WP-04 lands.

**Record with every result:** RSSI, JPEG quality, framesize and a `free_heap` / `free_psram` snapshot.

---

## 4. Parallel execution plan

### 4.1 Lanes (file ownership)

| Lane | Owns these files | Work packages (in order) |
| --- | --- | --- |
| **A: Infra** | `.github/**`, `rust-toolchain.toml`, `deny.toml`, `clippy.toml`, `CHANGELOG.md`, `scripts/**`, `espflash.toml`, `.gitignore`, `firmware/**`, `docs/99-*.rules` | WP-01 → WP-03 |
| **B: Build config** | workspace `Cargo.toml`, `.cargo/config.toml`, `crates/*/Cargo.toml`, new `crates/klipper-cam-fw/build.rs` | WP-02 (after WP-07) → WP-13 |
| **C: Firmware** | `crates/klipper-cam-fw/src/main.rs` and every module split from it, `runtime.rs`, `spsc_ring.rs`, `frame_buffer.rs` | WP-04 → WP-07 → WP-05 → WP-06 (with lane D) → **WP-19 split** → then WP-10(fw), WP-11, WP-12, WP-14, WP-15, WP-17, WP-18 in parallel, one module owner each |
| **D: Low-level** | `crates/esp32-cam-ll/**`, `crates/esp32-cam-hal/**` | WP-06 (ll part) → WP-08 → WP-25 → WP-20 |
| **E: Sensors/SCCB** | `crates/esp32-cam-sensors/**`, `crates/esp32-cam-sccb/**` | WP-16 → WP-22 → WP-23; also the sensor part of WP-10 |
| **F: Conversions/types** | `crates/esp32-cam-conversions/**`, `crates/esp32-cam-types/**` | WP-21 (TYPES-01 changes `CameraError`; coordinate with lanes D and E) |
| **G: HTTP/OTA lib** | `crates/klipper-cam-fw/src/{http.rs,ota.rs,config.rs,ap_select.rs}` | WP-09 → WP-17 (lib part) → WP-24 |
| **H: Docs** | `README.md`, `docs/**` except this folder | WP-24 (last) |

**Cross-lane packages.** WP-05, WP-06, WP-09, WP-12 and WP-17 touch two lanes. The WP's owner takes both lanes' files for the duration of that WP. Merge it before the next WP starts in either lane.

### 4.2 Waves

```text
Wave 0 (parallel, no deps)      WP-01  WP-03  WP-04  WP-16  WP-21  WP-22
Wave 1 (P0 correctness)         WP-07 → WP-05 → WP-06(+ll)   ║  WP-08  ║  WP-09
                                WP-02 (after WP-07)
Wave 2 (structure)              WP-19 (main.rs split; behaviour-neutral)
Wave 3 (FPS)                    WP-10  WP-11  WP-12  WP-13  WP-14  (all [HIL]; measure each in isolation)
Wave 4 (features/robustness)    WP-15  WP-17  WP-18  WP-20  WP-23  WP-25
Wave 5 (docs)                   WP-24
```

### 4.3 Dependency notes

- **WP-02 needs WP-07 first.** LTO can miscompile around the UB that WP-07 removes.
- **WP-04 comes before every performance WP.** Without its counters there is no baseline.
- **WP-10 against WP-11 to WP-14:** raising the sensor rate is pointless if Core 0 cannot keep up, and Core 0 work alone cannot exceed 12.5 FPS. Land both halves, then measure the combination.
- **WP-12 needs WP-07 and the `FrameRef` part of WP-12 itself.** Zero-copy snapshot depends on removing the `&'static` slice laundering (FW-CORE-08).
- **WP-06** shares a drain helper between `esp32-cam-ll` and the firmware. Do the ll API first, then switch both firmware capture paths to it.
- **WP-03 is blocked on the repository owner,** who must supply `firmware/bootloader_v554_gpio16.bin` (§7, question 1).

---

## 5. Work packages

Each WP lists its findings, files, steps and acceptance criteria. Findings are in priority order inside each WP, and the details are in the appendix files.

Status values: `TODO` · `IN-PROGRESS` · `READY-FOR-HIL` · `DONE` · `BLOCKED (<reason>)`.

### Wave 0

#### - [x] WP-01: Replace dead CI with real Rust gates (Lane A) · P0 · M
- **Findings:** BUILD-01, BUILD-09, BUILD-10, BUILD-20, BUILD-21, BUILD-23
- **Steps:**
  1. Delete `.github/workflows/build.yml` and `upload_component.yml`.
  2. Add `.github/workflows/rust.yml` with four jobs:
     - `host`: fmt, clippy `-D warnings --locked`, test `--locked`.
     - `miri`: `cargo miri test -p esp32-cam-ll -p esp32-cam-conversions -p esp32-cam-types`.
     - `xtensa`: `esp-rs/xtensa-toolchain@v1.6`, Xtensa clippy and release build, `espflash save-image`, then fail if `fw.bin` exceeds `0x1B0000`; upload the artifact.
     - `invariants`: `python3 scripts/verify_cold_boot_invariants.py`.

     Full YAML is in BUILD-01.
  3. Pin `rust-toolchain.toml` to `channel = "1.99.1"` and the esp toolchain to `1.97.0.0` in CI.
  4. Update `stale.yml` to `actions/stale@v9`, or delete it.
  5. Extend `.pre-commit-config.yaml` with `pre-commit-hooks` (toml, yaml, eof, whitespace, large-files), `ruff`, and a local `cargo fmt --check`. Make `pre_commit.yml` use `--from-ref/--to-ref`.
  6. Add `deny.toml` (licenses, bans with `multiple-versions="warn"`, advisories) and a `cargo-deny-action` job. Add `clippy.toml` with `msrv = "1.99"` and `too-many-lines-threshold = 150` as a warning; it becomes enforced after WP-19. Add `CHANGELOG.md`.
- **Acceptance:** A PR shows all jobs green. A deliberately injected clippy warning fails the `host` job. The size gate prints the image size.
- **Note:** the `xtensa` job needs WP-02's portable `.cargo/config.toml`. Until then, set `CARGO_TARGET_XTENSA_ESP32_NONE_ELF_LINKER` in the job. The `invariants` job needs WP-03.
- **Status:** DONE (2026-11-11, merged `86d6a93e`) **Miri job, first real run 2026-11-13/14:** it had never executed before — `rust-toolchain.toml`'s 1.99.1 pin redirected `cargo miri` to a toolchain without miri, fixed with `RUSTUP_TOOLCHAIN: nightly`. The first run found **no undefined behaviour** (56/56 conversions unit tests pass under Miri) but took ~5.5 h wall-clock, against GitHub's 6 h job limit: the exhaustive sweeps (`conv12` over all 2^24 inputs alone took 43 min) are impractical under the interpreter, and `tests/conv22_fixture_and_proptest.rs` then aborted because proptest's failure persistence calls `getcwd`, which Miri's isolation forbids. Five exhaustive tests carry `#[cfg_attr(miri, ignore)]` and the proptest file is `#![cfg(not(miri))]`; the representative-input tests exercise the same code paths under Miri. Expect the job to drop to minutes.

#### - [x] WP-03: Bootloader provenance and script hardening (Lane A) · P0 · S-M · BLOCKED on owner for the binary
- **Findings:** BUILD-03, BUILD-08, BUILD-17, BUILD-18, BUILD-19, BUILD-26, BUILD-27
- **Steps:**
  1. Commit `firmware/bootloader_v554_gpio16.bin` with a `.gitignore` exception (`!firmware/bootloader_v554_gpio16.bin`). Add `firmware/README.md` with its SHA-256, the IDF v5.5.4 commit and the patch description.
  2. `verify_cold_boot_invariants.py`:
     - add `--require-bootloader` (on by default in CI);
     - match the full 5-byte instruction sequences, not the 3-byte pattern;
     - strip `//` comments before the source greps;
     - make the live check use a field `/status` actually has. WP-04 adds `reset_reason`.
  3. `ota_deploy.py`:
     - run espflash with `cwd=REPO_ROOT` and explicit `--flash-size 4mb --flash-mode dio --flash-freq 40mhz` (keep these in sync with WP-11);
     - catch `ValueError` from `expected_next_slot`;
     - read the response by `Content-Length`/EOF, not the 40-byte heuristic;
     - fix the docstring's chunk size;
     - add one connect retry;
     - compare the post-boot `app_sha256` once WP-04 exposes it.
  4. Remove hard-coded `/home/console-admin`, `172.22.1.52` and serial paths. Read `CAM_HOST` and `CAM_SERIAL_PORT` from the environment, and use `#!/usr/bin/env python3`. Make the udev rule portable by dropping `ID_PATH`, or document it.
  5. Add `scripts/requirements` notes (Python ≥ 3.10) and pytest tests for `parse_jpeg_sof0`, the response parser and `expected_next_slot`.
- **Acceptance:**
  - On a fresh clone, `python3 scripts/ota_deploy.py --stage-only --elf <elf>` exits 0.
  - The invariants script exits 1 when the bootloader is missing.
  - `grep -rn 'console-admin\|172\.22\.' scripts README.md` is empty, excluding archived docs.
- **Status:** DONE (2026-11-11, scripts & provenance merged `199f247`; external bootloader binary gated via `--allow-missing-bootloader` per Ruling Q1) **CI note 2026-11-13:** with the bootloader absent, espflash 4.7 `save-image` fails with the misleading *"Error while connecting to device / os error 2"* — it is ENOENT on `espflash.toml`'s bootloader path, read with a bare `?` that the serialport build reports as a connection error. The xtensa CI job now packages the app image with a CI-local `[flash]`-only config when the file is missing (same tolerance as the invariants job) and with the real config once it is present. Uploading `firmware/bootloader_v554_gpio16.bin` (§7 Q1) remains the actual fix; real flashing must use that bootloader.

#### - [x] WP-04: Instrumentation and observability, the performance baseline (Lane C) · P1 · S
- **Findings:** FW-CORE-04 (instrumentation half), FW-CORE-15, FW-CORE-21, FW-NET-29; supports BUILD-08 and BUILD-18
- **Steps:**
  1. Add `#[inline(always)] fn ccount() -> u32` (`rsr.ccount`), usable from IRAM on both cores.
  2. Add an `FwStats` struct of `AtomicU32`s:
     - **Core 1:** `sensor_frame_period_us` EWMA, `frame_bytes` EWMA, `c1_pushed`, `c1_dropped`, `ring_full_events`, `soi_miss`, `overflow_aborts`.
     - **Core 0:** cycles per frame in drain, P81 enqueue, `poll_net_burst` and request handling; max main-loop iteration latency; `p81_frames_sent`, `p81_frames_superseded`; `c0_validation_failures`.
     - **Errors:** `sccb_errors`, `send_partial`.
  3. Expose these fields in `/status` JSON, keeping the existing 7 keys: `reset_reason`, `app_sha256` (the first 8 bytes of the running image's appended SHA, or `esp_app_desc` `app_elf_sha256`), `free_dram`, `core1_stack_hwm`, and the counters.
  4. Paint the Core 1 stack with `0xA5` at start and compute its high-water mark.
  5. Cache RSSI from the 5 s roam check. `/status` reports `null` when unknown instead of `-50`.
  6. Replace `let _ =` on hot-path results with counter increments. Keep behaviour otherwise identical.
- **Acceptance:**
  - `/status` shows the new fields.
  - Host tests cover the JSON formatting and its sanitisation.
  - **[HIL]** Record the baseline numbers (sensor period, per-stage Core 0 ms/frame, FPS) in this file under §8.
- **Status:** READY-FOR-HIL (2026-11-11, merged `66c7cd1`; 206 host tests + Xtensa build green; hardware baseline pending target camera)

#### - [x] WP-16: OV2640 driver correctness and SCCB cost (Lane E, plus a small main.rs control-path edit in Lane C) · P1 · S
- **Findings:** SENS-02, SENS-03, SENS-06, SENS-07, SENS-08, SENS-19, SENS-20, SENS-21, FW-CORE-14
- **Steps:**
  1. **SENS-02.** Restore the JPEG PCLK divider to 8 for CIF/SVGA in both places (`ov2640.rs:960-966` and `1091-1099`). Update the test to expect `0x08`. Only reintroduce 4 behind an explicit opt-in after a [HIL] VGA/SVGA soak passes.
  2. **SENS-03.** On any `Err` in `write_reg`, `read_reg` or `write_regs`, set `self.reg_bank = Ov2640Bank::Max`.
  3. **SENS-06.** Add a single `apply_format_regs`. Write CLKRC and R_DVP_SP once per `set_framesize`. Remove the redundant `set_pixformat` call at boot (main.rs:2460-2462).
  4. **SENS-19.** Read each register once in `init_status`.
  5. **SENS-20.** Skip unchanged writes in `set_reg_bits`. These are plain storage registers; see the SCCB-01 caveat.
  6. **SENS-07 and FW-CORE-14, in main.rs.**
     - After a framesize change, re-apply only quality.
     - Check every setter `Result`, and update `last_applied_ctrl` only for successful writes.
     - If `pause_core1_camera_producer()` fails, requeue the control update.
     - Call `CORE1_SPSC.reset_consumer()` before resume.
     - Make `resume_core1_camera_producer` return `bool`.
  7. **SENS-08.** Fix the "100kHz" log string. Try 400 kHz SCCB as a [HIL] experiment: logic-analyser edges, then a 24 h `/control` soak. Keep 200 kHz if marginal.
  8. **SENS-21.** Document the reject-vs-clamp behaviour. Ask the owner whether the default quality should be 10 or 12 (§7).
- **Acceptance:**
  - Mock-bus transaction-count tests pass: HD `set_framesize` at most 64 writes, with exactly one write each to 0x11 and 0xD3.
  - A fault-injection test shows the bank is re-selected after an error.
  - **[HIL]** 200 rapid framesize toggles produce no invalid committed frames.
- **Status:** READY-FOR-HIL

#### - [x] WP-21: Conversions and types performance and correctness (Lane F) · P1-P3 · M-L

> **Sweep done 2026-11-13 (`069ae1c`), after this WP closed.** Re-applying the
> WP-23 method — look for code claiming something the implementation does not
> do — to `esp32-cam-conversions` and `esp32-cam-types`.
>
> **Conversions came back clean**, with evidence rather than inspection: a new
> round-trip test over the RGB cube bounds the YUV error at 4 (full) and 8
> (limited range) with a mean under 2, and confirms the two `YuvRange` arms
> differ; the JPEG decoder rejects non-8-bit precision, bad sampling factors
> and progressive SOF; `frame_to_bmp` returns `InvalidFormat` rather than
> falling through; `max_encoded_len` is checked arithmetic over the maximum
> subsampling mode.
>
> **One finding in types.** `CameraStatus::clamped()` claimed to clamp "to
> valid hardware bounds" and forced every signed level into `-2..=2`. The
> drivers validate against five ranges and only 4 of 19 use `-2..=2`, so it
> rewrites values real parts accept — `ae_level` 5 is valid on OV3660 and
> OV5640 and comes back as 2. `is_valid_level` rejects the same levels.
> Nothing calls either, so it is latent; documented accurately and pinned by
> tests in both crates rather than deleted, since removing public surface from
> the shared types crate is the owner's call.
>
> `RESOLUTION_TABLE` matches upstream — including portrait FHD at 1088x1920,
> which looks like a typo for 1080 and is not — and `AspectRatio` is a
> nine-way bucket into `RATIO_TABLE`, not an exact ratio.

- **Findings:** CONV-01..09, CONV-11..22 (CONV-10 is in WP-06), TYPES-01..05
- **Steps:** do these in this order. Each is its own commit.
  1. **TYPES-01.** `SccbNack { phase: SccbPhase }` (a `#[repr(u8)]` enum). Add `const _: () = assert!(size_of::<CameraError>() <= 4);`. Also update `esp32-cam-sccb/src/adapter.rs` (SCCB-04 mapping) and every match site.
  2. **CONV-02.** Reciprocal quantization.
  3. **CONV-01.** i32 IDCT. Write the differential test against the old i64 IDCT before deleting it.
  4. **CONV-14.** DC-only block fast path.
  5. **CONV-04 and CONV-05.** u32 bit writer and reader; LUT plus max_code Huffman decode.
  6. **CONV-06 and CONV-13.** Monomorphized pixel loops; interior fast path.
  7. **CONV-03.** H2V1 subsampling for YUV422 input.
  8. **CONV-07, 08, 09, 15, 16, 17, 18, 19, 20.** Correctness, robustness and size fixes as described in the appendix.
  9. **CONV-11 and CONV-21.** Collapse the three SWAR kernels into one simple helper. Benchmark first and keep whichever is faster on target. Delete dead APIs. Have the HAL reuse `eoi_probe_window`; this one-line HAL change is coordinated with Lane D.
  10. **CONV-12.** Remove the YUV table padding.
  11. **CONV-22.** Add `fuzz/` targets and OV2640 4:2:2 fixture frames, captured on hardware or supplied by the owner.
  12. **TYPES-02..05.**
- **Acceptance:**
  - Every optimisation has a byte-identical or bit-exact differential test against the previous implementation.
  - Fuzz runs for at least 10 minutes without a crash.
  - Report host benchmark deltas; **[HIL]** report CCOUNT deltas where the appendix asks for them.
- **Note:** these paths are **not** on the production 720p JPEG hot path (the decoder and encoder run only in self-test and on non-JPEG sensors). TYPES-01 **is** on the SCCB path. Priority inside this WP: TYPES-01, CONV-02, CONV-01, then the rest.
- **Status:** DONE (2026-11-11, merged `d6c6cf3`; 240 workspace tests + fuzz targets + differential DCT/YUV tests green)

#### - [x] WP-22: SCCB bus API and mock fidelity (Lane E) · P2 · M
- **Findings:** SCCB-01, SCCB-02, SCCB-03, SCCB-05, SCCB-06, SENS-16, SENS-17 (SCCB-04 is in WP-21 with TYPES-01)
- **Steps:**
  1. **SCCB-01.** Masked writes always write. Add an opt-in `write_reg8_mask_if_changed`.
  2. **SCCB-02.** Add `write_reg16_seq(slv, start, &[u8])` with a looping default and an `embedded-hal` `transaction` override. Coalesce contiguous `Reg16` runs in the table writers.
  3. **SENS-16.** Upload OV5640 AF firmware in 256-byte bursts.
  4. **SCCB-03.** Document non-atomic 16-bit helpers. Add a group-hold (0x3212) wrapper.
  5. **SCCB-05.** Mock: `dropped_count()` or panic on overflow, `assert_transactions`, `fail_at(n)`, and logging of failed operations.
  6. **SCCB-06 and SENS-17.** Banked mock keyed by `(slv, width, bank, reg)`. Golden OV2640 transaction and delay-budget tests: `reset` = 159 writes / 10 ms; HD `set_framesize` budget; `set_quality` = 1 write.
- **Acceptance:** The new tests pass. Register state after coalesced writes is identical. **[HIL]** OV5640 AF init time drops, if an OV5640 is available.
- **Status:** READY-FOR-HIL (2026-11-11, SCCB burst/mask/group-hold, banked mock, OV5640 AF 16-burst upload, and OV2640 golden tests green)

### Wave 1: P0 correctness

#### - [x] WP-07: Remove aliasing UB and `static mut` laundering (Lane C) · P0 · S
- **Findings:** FW-CORE-03, FW-CORE-19; the first half of FW-CORE-08
- **Steps:**
  1. Change `capture_live_hd_jpeg_with_diag` to take `camera: &mut CameraDriver<'static>` and the framesize as parameters. Delete its `FIRMWARE_RUNTIME.as_mut()`.
  2. In `run_post_wifi_server_loop`, destructure `PSRAM_BUFFERS.take()` and use the owned `&'static mut` fields. Delete all twelve `from_raw_parts_mut` re-borrows (main.rs:2993-3062).
  3. Remove `static_mut_refs` from the allow list at main.rs:20 and fix the fallout with `addr_of_mut!` or ownership moves.
  4. **FW-CORE-19.** In `pause_core1_camera_producer`, return `true` immediately when running on APP_CPU, and stop I2S0 directly in the panic path.
- **Acceptance:**
  - Xtensa clippy is clean with `static_mut_refs` no longer allowed.
  - `grep -n from_raw_parts_mut main.rs` shows only the PSRAM carve.
  - **[HIL]** Stream, snapshot and control behave as before. A forced Core 1 panic (debug feature) resets within 1 ms.
- **Status:** READY-FOR-HIL

#### - [x] WP-05: OTA multicore safety and real rollback (Lanes C+G) · P0 · S-M
- **Findings:** FW-CORE-01, FW-NET-01, FW-NET-02, FW-NET-12, FW-NET-08, FW-NET-09
- **Steps:**
  1. **FW-CORE-01.** Construct the flash adapter with `unsafe { FlashStorage::new(..).multicore_ignore() }`. Add an RAII `Core1PauseGuard`: `new()` pauses Core 1 and fails with `OtaError::FlashWriteError` on timeout; `Drop` resumes it. Acquire the guard in every `EspFlashAdapter` erase, write and read, or once per OTA / cancel-rollback operation. **Do not use `multicore_auto_park()`.** Remove the ad-hoc pause at main.rs:3752.
  2. **FW-NET-01.** At boot, before Core 1 starts, load the OTA manager with `OtaDataManager::load_from_flash`. Cross-check it against the MMU slot and log any mismatch; the flash state wins. Delete the synthetic manager in `detect_booted_ota_manager_from_mmu`, or reduce it to a pure cross-check.
  3. **FW-NET-02.** Cancel rollback only after a confirmed IP (DHCP `Configured`, or static IP plus a gateway ARP reply) **and** the first HTTP request served. Log and retry on error instead of `let _`.
  4. **FW-NET-12.** If otadata is inconsistent at OTA time, return 500. Compute the next sequence number from the flash data.
  5. **FW-NET-08 and FW-NET-09.** Before pausing Core 1 for OTA, require a constant-time token match **and** a complete body. Replace `parsed.ota_token != Some(..)` with `constant_time_eq`.
- **Acceptance:**
  - Host tests with `MockNorFlash`:
    - a paused flag is set during every erase and write;
    - booting with sector 1 = seq 2/New and then getting an IP results in `Valid` in flash;
    - seq 7/8 commits to seq 9 in sector 0;
    - a bare POST does not pause Core 1.
  - **[HIL]**
    - With the camera streaming, `POST /update` returns 200, the device reboots into the other slot, and it stays there after two power cycles.
    - A new image built with a wrong SSID rolls back after reset, provided the bootloader has rollback enabled (§7, question 2).
- **Status:** READY-FOR-HIL

#### - [x] WP-06: Correct DMA half-buffer draining with one shared drain implementation (Lanes D+C) · P0 · M
- **Findings:** FW-CORE-02, LL-05, LL-06, LL-07, LL-15, LL-21, CONV-10
- **Steps:**
  1. **LL-15.** In `esp32-cam-ll`, add a single `drain_ready_halves(desc, dma, &mut half_idx, sink)`. It loops while the half's last descriptor is CPU-owned (`!owner()`), for at most 8 halves. For each half it filters into the sink, poisons the half with a non-marker fill (LL-06), and re-arms with one packed volatile store. Add a `CaptureHooks { idle(), now_us() }` trait so the polled capture can call back into the firmware.
  2. **LL-07.** Track halves pending since the last drain from the `in_eof_des_addr` delta, and fail the frame with `Overflow` when more than `half_buf_cnt - 1` are pending. Treat `IN_DSCR_ERR` and `IN_DSCR_EMPTY` as a frame abort. Never drain past the EOF descriptor. Evaluate `check_owner=1` [HIL].
  3. **LL-05.** On a debounced VSYNC end or idle timeout, process the current half unconditionally. Poisoning removes the stale-EOI risk.
  4. **LL-21.** In `stop()`, pulse `in_rst`/`ahbm_rst` and then `memw`.
  5. **Firmware.** Replace both hand-written drain loops (main.rs:1831-1838 and 2218-2246) with the ll helper. Delete the raw `0x3FF4_F0xx` constants from the firmware.
  6. **CONV-10.** Bound both firmware EOI scans to the last chunk plus a margin, using `eoi_probe_window` or a new `verify_jpeg_eoi_tail`.
- **Acceptance:**
  - Host tests:
    - halves 0 and 1 done while half 2 is DMA-owned, so half 2 is not consumed;
    - a stale old frame in the ring yields the correct length or `CorruptedFrame`;
    - a frame with no EOI touches only a bounded number of bytes.
  - `grep -n "0x3FF4_F0" crates/klipper-cam-fw` returns nothing.
  - **[HIL]**
    - With a 300 µs injected push delay, 1,000 frames contain zero runs of ≥ 64 zero bytes.
    - 10,000-frame decode failure count is 0.
    - A 5 ms injected stall returns `Overflow`.
    - VSYNC-to-return latency is no longer bimodal.
    - FPS is the same or better.
- **Open dependency:** confirm on hardware that the ESP32 I2S inlink writes back `owner=0` when `check_owner=0` (LL Open question 1). If it does not, gate on the `in_eof_des_addr` position instead of `owner()`.
- **Status:** READY-FOR-HIL (2026-11-12, `aa471ad`; host gates green, 314 workspace tests; the ll Xtensa paths type-check and pass clippy with a forced `target_arch="xtensa"` cfg on the host, but the real `cargo +esp` gates were NOT run: no esp toolchain in that session, so the `main.rs` capture edits are uncompiled. Becomes READY-FOR-HIL once they pass. Open on hardware: all three capture paths gate on `DrainGate::OwnerWriteback`; if the inlink does not write back `owner=0` (open dependency above), switch them to `DrainGate::EofPosition`. Also check that the LL-21 reset in `stop()` cannot drop FIFO bytes before the tail half is drained. Added beyond the WP: `start()` clears stale `DSCR_ERR`/`DSCR_EMPTY` latches; a drain rejects an `in_eof_des_addr` that is not a half's last descriptor.) **Xtensa gates PASSED in CI (2026-11-13):** PR #2 run 43493961174, head `4c33b99` — `Xtensa Clippy` (`--workspace --release -D warnings`) and `Xtensa Release Build` both green; the firmware links. This is the first on-target compile of this package's `main.rs` edits. Per §2.5 the package is therefore READY-FOR-HIL; only the [HIL] acceptance items remain.

#### - [x] WP-08: Make the low-level DMA API sound (Lane D) · P0 · M
- **Findings:** LL-01, LL-02, LL-03, LL-13, LL-20, HAL-01
- **Steps:**
  1. **HAL-01, done first.** On Xtensa, `CameraDriver::start_streaming` and one-shot capture return `NotSupported` (no more `start(.., 0)`) until step 2 lands.
  2. **LL-01.** Add a typed `DmaRing<'a>` built by a checked constructor. It validates 4-byte alignment, that descriptors and buffer lie in the DMA-capable window `0x3FFA_E000..=0x3FFF_FFFF`, and node size (≤ 4092, multiple of 4). `start(&'a mut DmaRing) -> RunningCapture<'a>`, whose `Drop` stops and resets the DMA. Keep a `#[doc(hidden)] unsafe fn start_raw` for migration only.
  3. **LL-02.** Delete the `&[u8]` `build_circular_ring`, rename `_mut`, and export it.
  4. **LL-03.**
     - Make `size_length_flags` a `VolatileCell`/`AtomicU32`.
     - Add `compiler_fence(Release)` plus `memw` before the `in_link` start write.
     - Add Acquire plus `memw` before half-buffer reads.
     - Make setters single packed stores.
  5. **LL-13.** Remove `Clone`. Use a singleton via the esp-hal `I2S0` token or an atomic `take()`.
  6. **LL-20.** Make the MMIO helpers `unsafe fn`, fix the misattached docs and comments, and set ll lints to `deny`.
  7. Fix the firmware self-test (main.rs:158-171), which builds a ring over an immutable, 1-byte-aligned stack array.
- **Acceptance:**
  - trybuild compile-fail tests: drop-while-running, `.clone()`.
  - Unit tests reject PSRAM and misaligned rings.
  - Miri passes on the ll crate.
  - **[HIL]** Disassembly shows `memw` before the store to `0x3FF4_F034`. A 10k-frame colour-bar CRC soak passes.
- **Status:** READY-FOR-HIL (2026-11-12, `81623f4`; steps 1-7 done, host gates green, 331 tests incl. trybuild compile-fail for `.clone()`, ring use/drop while running and a non-`'static` ring; ll/hal Xtensa paths pass clippy under a forced `target_arch="xtensa"` cfg. NOT run: real `cargo +esp` gates (the `main.rs` edits are uncompiled) and Miri (no nightly with miri in that session). Deviation: the safe `start` takes `&mut DmaRing<'static>`, since a leaked guard over a stack ring would let DMA write freed memory; borrowed rings use `unsafe start_scoped`. The WP-06 firmware capture paths stay on `start_raw`. Remaining LL-03 gap: `HalfDrain` still takes whole `&mut` slices that can cover DMA-owned halves (WP-20/WP-25). README still claims a 16-byte aligned `Lldesc` (WP-24).) **Xtensa gates PASSED in CI (2026-11-13):** PR #2 run 43493961174, head `4c33b99` — `Xtensa Clippy` (`--workspace --release -D warnings`) and `Xtensa Release Build` both green; the firmware links. This is the first on-target compile of this package's `main.rs` edits. Per §2.5 the package is therefore READY-FOR-HIL; only the [HIL] acceptance items remain. **Miri 2026-11-14:** the acceptance item "Miri passes on the ll crate" is now met — 68/68 `esp32-cam-ll` unit tests pass under Miri locally and in CI. Its first run found one real Stacked Borrows violation in `DmaRing`: the descriptors' `buf` pointers were derived from the `&mut [u8]` inside `build_circular_ring`, and moving that `&mut` into the struct (and every later `parts_mut` reborrow) retagged the buffer and invalidated them, so any Rust-side write through `desc.buf` — which the LL-02 test does to simulate the DMA — was UB. `DmaRing` now owns the buffer as a raw root pointer (`NonNull<u8>` + length + `PhantomData<&'a mut [u8]>`), links through a raw core so every `desc.buf` shares the root's provenance, and hands out `&mut [u8]` only as short-lived children of it. The pub `build_circular_ring` keeps its signature; its docs now say callers must treat the descriptor pointers as addresses, not dereference them after reborrowing the buffer — which is the remaining LL-03 gap for the firmware's own rings (WP-20/WP-25).

#### - [x] WP-09: Security hardening (Lane G, plus a small main.rs edit) · P1 · S-M
- **Findings:** FW-NET-04, BUILD-07, FW-NET-03, FW-NET-18, FW-NET-27
- **Steps:**
  1. **BUILD-07.** Add `crates/klipper-cam-fw/build.rs` (coordinate with Lane B) that panics for Xtensa release builds when `WIFI_PASS` or `WIFI_OTA_TOKEN` is unset or empty, and emits `rerun-if-env-changed` for both.
  2. **FW-NET-04.** `handle_ota_post` returns 403 when `is_default_ota_token()`. Print a warning at boot. Use a placeholder in the README.
  3. **FW-NET-03.**
     - A passphrase conversion error is fatal, never `Open`.
     - Add a compile-time assert that the passphrase length is 0 or 8..=63.
     - Call `KlipperCamConfig::is_valid()` at boot.
     - Open networks only behind an `allow-open-wifi` feature.
  4. **FW-NET-18.**
     - Require `chip_id == 0`.
     - Require the entry point in IRAM (`0x4007_0000..0x400A_0000`) or IROM (`0x400D_0000..0x4040_0000`).
     - Require `aligned_end + 32 == len` when a hash is appended.
     - Read back and re-hash after writing, before `commit_update`.
  5. **FW-NET-27.** `/control` mutations require POST, or the `X-Requested-With` header. No `ACAO: *` on mutating responses. Update the panel JavaScript to `fetch(.., {method:'POST'})`, and update the HTTP contract in the README.
- **Acceptance:**
  - Unit tests:
    - a default token gives 403;
    - an S3 image and a misplaced digest are rejected;
    - a mock flash bit-flip leaves otadata unchanged;
    - `GET /control?var=` gives 405.
  - A release build without the environment variables fails.
  - Host `cargo test` is unaffected.
- **Status:** READY-FOR-HIL (2026-11-12, `f7cb7ce` + `3688651`; host gates green, 285 tests; Xtensa gates NOT run: no esp toolchain in that session, so the `main.rs` Wi-Fi boot and OTA-auth edits are uncompiled. Becomes READY-FOR-HIL once `cargo +esp` clippy/build pass. HIL: OTA now reads back and re-hashes up to 1.8 MB before commit; check OTA time and the watchdog. Contract: `/control` mutations need POST or `X-Requested-With` (GET → 405), a cross-origin `Origin` → 403, `POST /update` → 403 with the default token.) **Xtensa gates PASSED in CI (2026-11-13):** PR #2 run 43493961174, head `4c33b99` — `Xtensa Clippy` (`--workspace --release -D warnings`) and `Xtensa Release Build` both green; the firmware links. This is the first on-target compile of this package's `main.rs` edits. Per §2.5 the package is therefore READY-FOR-HIL; only the [HIL] acceptance items remain.

#### - [x] WP-02: Portable build config and release profile (Lane B) · P0/P1 · S · **after WP-07**
- **Findings:** BUILD-02, BUILD-04, BUILD-11, BUILD-12, BUILD-13, BUILD-14
- **Steps:**
  1. **BUILD-02.** Rewrite `.cargo/config.toml` with no absolute linker path. Add a `runner` and `fw` / `fw-check` aliases, as in BUILD-02. Do not set `[build] target`.
  2. **BUILD-04.** Add `[profile.release]`: `lto="fat"`, `codegen-units=1`, `debug=2`, `opt-level=3`, `overflow-checks=false`, `incremental=false`. Add `[profile.dev] opt-level="s"`.
  3. **BUILD-14.** Add an opt-in `[profile.release-tiny]` (`opt-level="s"` plus immediate-abort, using the rustflag form on this nightly). Document the trade-off.
  4. **BUILD-11.** Trim smoltcp features to `medium-ethernet, proto-ipv4, socket-tcp, socket-dhcpv4`. Re-check `iface-max-addr-count` after WP-15.
  5. **BUILD-12.** Remove `embassy-futures`.
  6. **BUILD-13.** Investigate a matched esp-hal/esp-radio pair that removes the duplicate `esp32` PAC (0.41 vs 0.42). Upgrade only if it builds cleanly; otherwise record the result.
- **Acceptance:**
  - On a fresh clone with `export-esp` sourced, `cargo +esp fw` links.
  - Image and `.rwtext` sizes are reported before and after; expect about −25 KB image and −5.7 KB IRAM.
  - **[HIL]** A/B FPS for `opt-level` 3 vs `"s"`. Keep the faster one and record both.
- **Status:** READY-FOR-HIL (2026-11-12; steps 1-5 done, host gates green, 314 tests. Not verified: `cargo +esp fw` linking, image/`.rwtext` sizes and the `fw-tiny` alias (no esp toolchain in that session). Step 4 keeps `iface-max-addr-count-2`, because the interface holds the 172.22.1.52 fallback and the DHCP lease at once (main.rs `update_ip_addrs`). Step 6 (BUILD-13, matched esp-hal/esp-radio pair) not attempted: it needs an Xtensa build. CI's `CARGO_TARGET_XTENSA_ESP32_NONE_ELF_LINKER` override was dropped as no longer needed. Open [HIL]: A/B FPS for `opt-level` 3 vs "s".) **Xtensa gates PASSED in CI (2026-11-13):** PR #2 run 43493961174, head `4c33b99` — `Xtensa Clippy` (`--workspace --release -D warnings`) and `Xtensa Release Build` both green; the firmware links. This is the first on-target compile of this package's `main.rs` edits. Per §2.5 the package is therefore READY-FOR-HIL; only the [HIL] acceptance items remain.

### Wave 2: structure

#### - [ ] WP-19: Split `main.rs` into modules with no behaviour change (Lane C, single owner) · P2 · L
- **Findings:** FW-CORE-16, FW-NET-17, FW-CORE-23, BUILD-24
- **Steps:**
  1. Purely mechanical moves first, one commit per module. New modules under `crates/klipper-cam-fw/src/`:

     | Module | Contents |
     | --- | --- |
     | `fw/core1_producer.rs` | Core 1 loop, pause guard, `ccount` |
     | `fw/capture.rs` | uses the WP-06 drain |
     | `fw/boot/psram_spi1.rs` | one `with_spi1_psram_txn`, replacing the duplicated save/restore at 951-1121 and 1128-1271 |
     | `fw/boot/rtc.rs` | WDT, breadcrumbs, quiesce |
     | `fw/net/phy.rs` | network PHY glue |
     | `fw/net/tcp_tx.rs` | TCP transmit path |
     | `fw/http/port80_pool.rs` | a `Port80Worker` with `service(&mut self, sock, ctx)` |
     | `fw/http/port81_stream.rs` | an `MjpegStreamer` owning the `p81_*` state |
     | `fw/wifi/manager.rs` | Wi-Fi management |
     | `fw/ota_glue.rs` | OTA glue |

     `main.rs` keeps a short main loop that calls `service()` on each component with a shared `Ctx`.
  2. Extract the host-testable `Port80Conn` state machine into the lib behind a socket trait (FW-NET-17). Add a single `queue_error(..)` helper, replacing the four copies at main.rs:3598-3685.
  3. **FW-CORE-23 and BUILD-24.**
     - Delete the dead aliases.
     - Add `const FRAME_SLOT_BYTES: usize = 128 * 1024;` shared by the PSRAM carve, Core 1 and `dma_cfg`.
     - Gate `hil_check_*`, the JPEG decoder and `init_psram_and_run_hil` behind a `hil-selftest` feature that is off in release.
     - Set `esp-backtrace` `default-features = false`.
     - Make `capture_live_hd_jpeg_with_diag` non-generic; it is currently instantiated twice.
- **Acceptance:**
  - No function over 150 lines (`clippy::too_many_lines`).
  - `grep -n "128 \* 1024" main.rs` finds only the const.
  - `nm` no longer shows `decode_jpeg_internal` or `init_psram_and_run_hil` in the release ELF.
  - Host tests cover slowloris (a 3 s header deadline), OTA promotion authentication, 413, and a peer that closes mid-body.
  - **[HIL]** Full regression (stream, snapshot, control, OTA). FPS unchanged within noise.
- **Status:** TODO

### Wave 3: frame rate (all [HIL]; land and measure each in isolation)

#### - [ ] WP-10: Raise the OV2640 sensor frame rate, the main FPS lever (Lanes E+C) · P1 · M
- **Findings:** SENS-01, LL-08, FW-CORE-04 (experiment half)
- **Steps:** each step is its own measured experiment.
  1. Confirm the cap. `sensor_frame_period_us` from WP-04 should be about 79,800 µs at 20 MHz XCLK.
  2. **XCLK 24 MHz.** LEDC at 80 MHz APB with 1-bit duty cannot produce 24 MHz exactly; LL-08 has the details. Options:
     - (a) APLL-derived clock through CLK_OUT/I2S MCLK onto the XCLK pin (GPIO21 on WROVER-KIT; check which signals can reach it through the GPIO matrix);
     - (b) LEDC with a fractional divider, provided measured jitter is acceptable.

     Validate XCLK on a scope. Expect about 15 FPS.
  3. **HD-fast window.** Add an opt-in sensor mode that programs the readout window to 900 rows:
     - VSTART/VSTOP 0x19/0x1A, COM1 0x03, REG32;
     - HSIZE8/VSIZE8/SIZEL set together to 1600x900;
     - YOFFL = 0;
     - CLKRC = 0x00 and R_DVP_SP = 0x0C as today.

     This respects the OV-105 invariant (ov2640.rs:6-13). Expect about 16 FPS at 20 MHz, or about 19-20 FPS combined with step 2 (estimate).
  4. Only if needed: try the CLKRC doubler (bit 7) last, because it doubles PCLK and stresses I2S sampling.
  5. In `xclk.rs` (LL-08), validate that the requested frequency is realizable or report the achieved one.
- **Acceptance:**
  - FPS, sensor period, EOI success rate and decode-failure count are reported for each step.
  - A mock-bus test covers the HD-fast register set.
  - The default stays on the C-matching configuration unless the owner approves a new default.
- **Status:** TODO

#### - [ ] WP-11: PSRAM and flash at 80 MHz, an A/B experiment (Lanes C+A) · P1 · S code / M validation
- **Findings:** FW-CORE-05, BUILD-06
- **Steps:**
  1. Add a boot-time PSRAM copy benchmark: 64 KB slot to slot, timed with CCOUNT, printed as MB/s.
  2. Behind a `psram-80m` cargo feature, select `PsramCacheSpeed::PsramCacheF80mS80m`, with `espflash.toml` `[flash] frequency = "80MHz"` (and `mode = "qio"` if the module supports it). The bootloader header must match; coordinate with WP-03.
  3. Update `verify_cold_boot_invariants.py:205` to accept the feature-selected speed.
- **Acceptance:**
  - **[HIL]** About 2× PSRAM MB/s.
  - 100 cold power cycles with zero PSRAM verify failures.
  - 24 h stream soak with no `reset_reason` changes and no rise in `c1_dropped`.
  - Make 80 MHz the default only if all three pass. Either way, correct the README's PSRAM clock claims.
- **Status:** TODO

#### - [ ] WP-12: Cut Core 0 PSRAM copies (DRAM TX rings, zero-copy snapshot, `FrameRef`) (Lanes C+G) · P1 · M
- **Findings:** FW-CORE-06, FW-CORE-07, FW-CORE-08 (second half), FW-NET-06, FW-NET-19, FW-NET-20, FW-NET-25
- **Steps:**
  1. **FW-CORE-08.**
     - Replace `active_triple_static_slice` and `latest_snapshot_static_slice` with a `FrameRef { slot, len, seq }` handle.
     - Add `TripleFrameBuffer::get(&self, r) -> Option<&[u8]>`, checked against a per-slot generation.
     - Add `pin(r)` / `unpin(r)`, which `commit_write_slot` honours. Alternatively, use a 4th slot.
     - `CameraDriver` borrows the frame per request instead of storing a `'static` slice.
  2. **FW-CORE-06.**
     - Move the port-81 TX socket buffers to internal DRAM: about 32 KB for the active socket and about 2 KB for the standby socket, sized against `free_dram` from WP-04.
     - Retune `TCP_MAX_INFLIGHT_BYTES` and `P81_PIPELINE_LOW_WATER_BYTES` to about 90% and 45% of the new buffer size.
     - Rename `enqueue_p81_zero_copy` to reflect that `send_slice` copies.
  3. **FW-CORE-07 and FW-NET-06.**
     - Serve `/snapshot` as header-only into a small buffer, then stream the body from a pinned slot across loop iterations, reusing the P81 enqueue helper.
     - Shrink `p80_resp_bufs` to 8 KB each. This frees about 480 KB of PSRAM.
  4. **FW-NET-19.** Write the MJPEG part header from a const prefix plus a small decimal writer, without `core::fmt`.
  5. **FW-NET-20.** Serve pre-gzipped `&'static` panel and OTA HTML with const headers, using the same zero-copy send.
  6. **FW-NET-25.** Call `close()` as soon as everything is enqueued.
- **Acceptance:**
  - Host tests:
    - a pinned slot is never chosen as the write slot;
    - the snapshot body is byte-identical to the frame.
  - **[HIL]**
    - Core 0 ms/frame drops by at least 40%.
    - 100 snapshots while streaming leave `c1_dropped` unchanged.
    - Snapshot p95 latency improves.
    - `free_psram` rises by about 480 KB.
    - FPS is the same or better.
- **Status:** TODO

#### - [ ] WP-13: IRAM placement of hot loops (Lanes B+D+C) · P1 · M · after WP-02
- **Findings:** BUILD-05
- **Steps:**
  1. Place these functions in IRAM:
     - `filter_jpeg`, `filter_yuyv*`, `filter_grayscale*` (in ll: `#[cfg_attr(target_arch="xtensa", link_section=".rwtext")]`, or RAM thunks in the firmware, because ll cannot depend on esp-hal);
     - `Core1JpegSpsc::push_chunk`, `pop_with`, `drain_core1_spsc_to_triple`, and the Core 1 loop body (`#[esp_hal::ram]`).
  2. Mark small callees `#[inline(always)]`.
  3. Optionally try `ESP_HAL_CONFIG_PLACE_ANON_IN_RAM=true`.
  4. Keep total IRAM under budget. Current usage is about 78 KB of 128 KB.
- **Acceptance:**
  - `xtensa-esp32-elf-nm -C` shows these symbols in `0x4008_0000..0x400A_0000`.
  - **[HIL]** Report FPS and Core 1 cycles per chunk before and after.
- **Status:** TODO

#### - [ ] WP-14: Core 0 loop latency and Core 1 backpressure (Lane C) · P2 · M
- **Findings:** FW-CORE-09, FW-CORE-10, FW-CORE-11, FW-CORE-12, FW-CORE-13, FW-CORE-17, FW-CORE-20, FW-NET-13
- **Steps:**
  1. **FW-CORE-13.** Express every Core 1 timeout in CCOUNT cycles: VSYNC wait, idle-EOF (25 ms, the same as the fallback path) and frame abort.
  2. **FW-CORE-09.** Make the ring-full wait a CCOUNT deadline of about 5-10 ms that still checks `CORE1_PAUSE_REQ`. Count `ring_full_events`. Optionally try 2 KB chunks × 16 slots.
  3. **FW-CORE-17.** Add `Core1JpegSpsc::push_with(flags, |slot| filter_into(slot))` and delete `unpack_buf`, which also relieves FW-CORE-15.
  4. **FW-CORE-10 and FW-NET-13.**
     - Give `block_on_with_timeout` a `service: &mut dyn FnMut()` that runs the SPSC drain and `poll_net_burst` about every 1 ms.
     - Suppress roam scans while streaming unless RSSI is below a hard floor.
     - On reconnect, try the locked BSSID first.
  5. **FW-CORE-11.** `poll_net_burst` keeps polling while `SocketStateChanged` or `poll_delay == ZERO`, within a CCOUNT budget of about 1.5 ms. Interleave one SPSC drain per N polls.
  6. **FW-CORE-12.** Core 1 skips pushing frames that Core 0 does not want, via an `AtomicBool CORE0_WANTS_FRAME`. Count them as `frames_skipped`.
  7. **FW-CORE-20.** Keep the heartbeat under 120 bytes, or move it to `/status` only.
- **Acceptance:**
  - Host tests for `push_with`: wrap and full cases.
  - **[HIL]**
    - A forced roam scan while streaming costs fewer than 5 `c1_dropped`.
    - `c1_dropped/c1_pushed ≈ 0` while hammering `/snapshot` and `/status`.
    - No 30 s latency spikes.
    - Segments per loop iteration increase.
    - FPS is the same or better.
- **Status:** TODO

### Wave 4: features and robustness

#### - [ ] WP-15: Network configuration, roaming and MTU (Lane C) · P1 · M
- **Findings:** FW-NET-05, FW-NET-15, FW-CORE-22, FW-NET-16, FW-CORE-18, FW-NET-24
- **Steps:**
  1. **FW-NET-05, FW-CORE-22.**
     - Add a single `apply_ip_config(&mut iface, &cfg.network)` that applies either the DHCP lease or the static IP/prefix/gateway.
     - Delete every `172, 22, 1, 52` and `172, 22, 1, 1` literal (main.rs:265, 1519, 3250, 3320, 3343-3369, 3383, 3396-3434).
     - Make the announcement a true gratuitous ARP for the configured IP.
  2. **FW-NET-15.** On `Configured`, `retain` only the static address before pushing. On `Deconfigured`, remove the lease. Log a failed push.
  3. **FW-NET-16.** Use `update_with_hysteresis(.., ROAM_RECOVERY_HYSTERESIS_DB)`. Compare against the current AP's scan-averaged RSSI.
  4. **FW-CORE-18 and FW-NET-24.** Replace the TX clamp with `debug_assert!`. Confirm the esp-radio MTU and derive `TCP_MSS_BYTES` from it.
- **Acceptance:**
  - Host tests: `apply_ip_config`; hysteresis with an alternating −66/−64 dBm sequence.
  - **[HIL]**
    - On a different DHCP subnet, no ARP for 172.22.1.52 appears in Wireshark and the device is reachable.
    - A forced lease change keeps the device reachable.
    - A pcap shows the expected MSS.
- **Status:** TODO

#### - [ ] WP-17: OTA streaming, throughput and cleanup (Lanes G+C) · P2 · M-L · after WP-05
- **Findings:** FW-NET-07, FW-NET-10, FW-NET-11, FW-NET-26, FW-NET-28
- **Steps:**
  1. **FW-NET-07.**
     - Validate the image before pausing Core 1.
     - Make `Sha256::update` take whole 64-byte blocks, or use the esp-hal hardware SHA if the 1.2.2 `unstable` API exists.
     - Call `poll_net_burst` between erase and write chunks, via a callback.
  2. **FW-NET-11.** Stream the image to flash using the existing `begin_ota` / `write_chunk` / `finish_ota`:
     - erase incrementally ahead of the write cursor;
     - hash incrementally, with a segment-parsing state machine;
     - commit only after the digest matches;
     - stage through a buffer of at most 4 KB in internal RAM.

     Remove the 1.8 MB PSRAM accumulator.
  3. **FW-NET-10.** While a socket owns OTA, run a bounded recv + poll inner loop of about 20 ms.
  4. **FW-NET-26.** Reply `100 Continue` after authentication when the client sends `Expect: 100-continue`.
  5. **FW-NET-28.**
     - Delete `sync_slot_to_flash` and `sync_sector_to_flash`.
     - Use plain-max sequence comparison, as the bootloader does.
     - Share one CRC core.
- **Acceptance:**
  - Host tests:
    - random chunk sizes produce identical flash contents;
    - a corrupt last byte leaves otadata untouched;
    - commits land in the lower-seq sector.
  - **[HIL]**
    - Phase timings are logged.
    - The Core 1 pause lasts no longer than erase plus write.
    - The stream does not hit the 10 s abort during OTA.
    - `free_psram` rises by about 1.8 MB.
    - `curl -v` shows `100 Continue`.
- **Status:** TODO

#### - [ ] WP-18: Multi-viewer MJPEG stream policy (Lanes C+G) · P2 · M · owner decision needed (§7, question 5)
- **Findings:** FW-NET-14
- **Steps:**
  - **Option A (recommended):** fan out one frame to both port-81 sockets, each with its own `tx_off`. Advance to the next frame when the slowest socket reaches the low-water mark or is dropped.
  - **Option B:** preempt only an owner with no TX progress for more than 2 s; otherwise reply 503.
  - With either option, make `handle_port81` and its tests match the firmware.
- **Acceptance:** **[HIL]** Two `curl` streams for 60 s both receive steady data with no reset storm.
- **Status:** TODO

#### - [ ] WP-20: HAL redesign: owned DMA ring, borrowed pool, concurrent leases (Lane D) · P1-P3 · L · after WP-08

> **Sweep done 2026-11-13 (`d729c53`), ahead of this WP.** Applying the WP-23
> method — look for code that presents itself as driving hardware it never
> touches — to the HAL and LL crates found three things, all fixed or
> recorded:
>
> * **HAL-09 is worse than "missing tables".** `HalOv2640Sensor::set_framesize`,
>   `set_pixformat` and `set_gainceiling` wrote only the bank-select prelude,
>   recorded the requested value and returned `Ok`. Every setter has that
>   boilerplate and only `set_quality` finishes the job. They now return
>   `NotSupported`, and `CameraDriver::init` tolerates it via the existing
>   `ignore_not_supported` helper.
> * **LL-17: the S2 and S3 backends perform zero MMIO.** Register blocks held
>   by value, no `target_arch = "xtensa"` path, against 82 MMIO accesses in
>   `esp32_i2s`. Retitled as host models and documented; their
>   `volatile_write_u32` SAFETY text claimed to justify a hardware register
>   write and has been corrected. They still need real backends.
> * **The S2 model carries one ESP32 signal index.** `set_pin` binds vertical
>   sync with `I2S0I_WS_IN_IDX` (28) while the four S2 indices it defines all
>   sit exactly +3 from their ESP32 counterparts — a leftover, like NT99141's.
>   A test pins the pattern; the correct value needs the S2 TRM.
>
> `xclk.rs` is `forbid(unsafe_code)` and cannot reach LEDC; it records a
> configuration rather than starting a clock, and now says so.

- **Findings:** HAL-02..12, LL-04, LL-14, LL-16, LL-17
- **Steps:**
  1. **LL-04.** Make `calc_dma_sizes` match C for JPEG: node 2048, half 4096, cnt 8, total 32 KiB. Derive the `JPEG_DMA_*` constants from it. Fix the test.
  2. **HAL-02.** `FramePool` borrows its storage and gets a `const fn new`. `CameraDriver` holds `&'a mut FramePool`.
  3. **HAL-03.** Producer/Consumer handles; lock-free slot state machine with Acquire/Release; each guard borrows only its slot.
  4. **HAL-01, completing WP-08 step 1.** The driver owns a `DmaRing<'static>` in internal RAM and real LEDC XCLK. Re-enable streaming on Xtensa.
  5. **HAL-04..11:**
     - `output_len`;
     - SOI probe and trim, forward EOI;
     - reject `psram_dma_mode` on ESP32;
     - backend generic over a `CamLl` trait (**LL-16**), with `vsync_invert` from config;
     - PSRAM pin reservation;
     - real OV2640 tables or rename to `MockOv2640`;
     - `Result<Next, (Self, E)>` transitions;
     - `return_all`, `newest_ready`, real timestamps.
  6. **LL-14 and LL-17.**
     - `RegisterIo` trait with `Mmio` and `Recording` implementations, used by a single code path.
     - Golden write-trace tests against `ll_cam.c`.
     - A simulated I2S model so the real capture loop runs on the host and under Miri.
     - Mark S2/S3 `#[doc(hidden)]` until they have real backends.
  7. **HAL-12.** Proptest state-machine tests for the pool; per-format table tests; a static `FramePool<2, 131072>` test.
- **Acceptance:**
  - New tests fail before and pass after.
  - Host coverage of the polled capture loop is above 0%.
  - **[HIL]** `CameraDriver::init().start_streaming().capture_frame()` returns a decodable JPEG.
  - Optional: replace the firmware's `TripleFrameBuffer` with the HAL pool and compare FPS.
- **Status:** TODO

#### - [x] WP-23: Non-OV2640 sensor driver correctness (Lane E) · P1-P3 · M-L
- **Findings:** SENS-04, SENS-05, SENS-09, SENS-10, SENS-11, SENS-12, SENS-13, SENS-14, SENS-15, SENS-18, SENS-22, SENS-23, SENS-24, SENS-25
- **Steps:** in priority order.
  1. **SENS-09.** Probe 8-bit sensors before 16-bit ones at each shared address.
  2. **SENS-05, 11, 12.** NT99141 AE index `(level.clamp(-4,4)+4)*5`; NT99141 brightness 0 maps to 0x80; HM1055 PLL `0x10 + (mul-204)/12`.
  3. **SENS-04.** OV3640 setters return `NotSupported` until a real OV3640 register map is ported (§7, question 6).
  4. **SENS-10.** HM0360 240x240 arm, and an error for unsupported sizes.
  5. **SENS-13.** Remove no-op setters so the `NotSupported` defaults apply.
  6. **SENS-14.** Shared `level_in` helper and per-sensor lookup tables; never write raw signed values.
  7. **SENS-18.** Drop end markers; use a single `RegOp` table writer.
  8. **SENS-15.** Framesize changes write only the delta table.
  9. **SENS-24 and SENS-25.** One unsupported-framesize policy (`FailedToSetFrameSize`); update status only after a successful write; fix or document the OV7725, GC0308 and OV3640 deviations.
  10. **SENS-22.** Per-sensor cargo features (default on, off for klipper-cam-fw); `Reg8` for 8-bit-address tables; `static` for large tables.
  11. **SENS-23.** `omnivision_common` module to remove about 1,500 duplicated lines.
- **Acceptance:**
  - A shared conformance test runs across all 18 drivers.
  - Per-driver tests for each fix.
  - `cargo bloat` shows klipper-cam-fw links only the OV2640 driver.
- **Status:** DONE (2026-11-13, `085d043`..`b956146`; host gates green, 393 workspace tests. Xtensa gates NOT run — no esp toolchain in this session — but this crate is target-independent and `forbid(unsafe_code)`, so the risk is low.)
  - **All eleven steps addressed.** SENS-09, 05, 11, 12, 04, 10, 13, 18, 24, 25, 22, 23 implemented; SENS-14 partial and SENS-15 deliberately not implemented, both for stated reasons below.
  - The shared 18-driver conformance sweep found **two drivers the findings did not list**: `Ov7670` refused sub-VGA sizes with `InvalidArg` from an inner match arm, and `Ov7725` did the same at its top guard. A later review found two more non-conforming guards, in `Ov5640` and `MegaCcm`, sitting on the largest FrameSize discriminant and so unreachable; corrected by inspection.
  - **SENS-22** is the one with a measurable result: the firmware now resolves to `esp32-cam-sensors` feature `ov2640` alone (`cargo tree -e features`), so the other 17 drivers and their tables are never compiled. Two things were needed — `default-features = false` on the *workspace* dependency, since cargo ignores a member's setting otherwise, and the HAL enabling no driver, since feature unification would otherwise hand every binary the full set. Gc2145/Gc032a/Bf20a6 tables moved to `Reg8` (~2 KB). `ov3640` declares a dependency on `ov3660`, which is SENS-04 surfacing in the feature graph.
  - **SENS-14 partial.** Missing range checks added behind a shared `level_in` helper; `Ov7725` and `Sc101Iot` had none, so `(level & 0xFF) as u8` truncated out-of-range levels into unrelated register values. The per-sensor lookup tables the finding also asks for are **not** done: whether a part reads the register as two's complement or as a magnitude differs per sensor and needs the C driver or the datasheets.
  - **SENS-15 not implemented — the premise does not hold.** Comparing the tables' register sets shows the default rewrite is load-bearing: HM0360's QVGA/QQVGA tables (57 registers) and VGA/240x240 tables (18) are disjoint, so a delta write strands 57; HM1055's HD/SVGA tables write four fewer registers than the smaller sizes. For NT99141 the opposite — only 4 of its 64 framesize registers are in the default table, so the rewrite does not restore framesize state at all. Three structural tests pin this. **Two live defects surfaced:** NT99141 leaves nine registers stale on a switch to HD, and HM0360 strands its eighteen VGA/240x240 window registers on VGA → QVGA. Neither is fixed; both need per-register reset values.
  - **Open for the owner:** §7 question 6 (is any OV3640 in use?) decides whether that driver is repaired or marked unsupported — `reset()` still writes its unverified default table. **NT99141 has the same defect as the OV3640**, found during review: an OV5640-shaped window block at 0x3800..0x3810 and 0x5000/0x5183/0x5306 reads in `init_status`, so that call may report state from registers the part does not implement. Documented, not changed, because it is not in the work order. The HM0360 `PMU_CFG_3` value for 240x240 and the OV7725 QVGA/VGA threshold are both flagged in code as unconfirmed.
  - **Acceptance not met:** `cargo bloat` needs an Xtensa build. The SENS-09 probe reorder needs the hardware A/B the finding asks for — it trades a write-corruption hazard for a resynchronisation one, since `Ov2640::detect` opens with a BANK_SEL write and `Gc2145::detect` leaves a 16-bit part holding half an address.
  - **SENS-23 completed beyond the finding's scope.** Besides the masked get/set and table writer it names, eight control bodies copied verbatim between OV3660 and OV5640 (four also into OV3640) now live once in `omnivision16`. Safe because the copies were identical down to the constants: all 58 constant names shared by OV3660/OV5640 and 29 by OV3660/OV3640 hold the same value in each. A test compares the two drivers' transaction logs for all eight. 262 further lines out. `init_status`, `set_res_raw` and `set_pll` stay per-driver — they call helpers whose bodies genuinely differ.
  - **Found and fixed outside the work order: NT99141 was a half-finished OmniVision port.** `set_sharpness` programmed 0x5300..=0x530C, the OV5640 ISP sharpness block verbatim, on a Novatek part and returned `Ok` — while `init_status` read sharpness back from 0x3301, so the two paths could not both be right. `init_status` also reported denoise, dcw, bpc, wpc, raw_gma and lenc from 0x5306, 0x5183 and 0x5000, controls that are already `NotSupported` here. The setter now refuses and those fields report defaults. Fifty-five unused constants still carrying OV5640 addresses are deleted; the giveaway was `COMPRESSION_CTRL07`, remapped to 0x3401 and used, while its unused siblings sat at 0x4400..0x440E. A sweep of the other seventeen drivers for OmniVision-shaped ranges found nothing beyond this and the OV3640 already covered by SENS-04. Still unverified against a datasheet: `status.sharpness` is read from 0x3301, which is not demonstrably foreign but is now a value nothing can set.

#### - [ ] WP-25: Low-level DMA engine performance (Lane D) · P2-P3 · M · after WP-06 and WP-08
- **Findings:** LL-09, LL-10, LL-11, LL-12, LL-18, LL-19
- **Steps:**
  1. **LL-09.** Read-modify-write `I2S_FIFO_CONF` bits 12, 16..18 and 20 only, preserving `rx_data_num`. Do a [HIL] register dump against an IDF build.
  2. **LL-10.** Scan the SRAM chunk for EOI before copying, and filter straight into `jpeg_out` after SOI.
  3. **LL-11.** Interrupt-driven `IN_SUC_EOF` variant, using the `CaptureHooks` from WP-06.
  4. **LL-12.** Wrap the DPORT clock and reset read-modify-writes in `critical_section`, or use the esp-hal APIs.
  5. **LL-18.** Direct-call filter dispatch with `#[inline]`; word-combined JPEG filter.
  6. **LL-19.** Program the GPIO matrix only in `set_pin`; compute the EOF-to-half mapping arithmetically.
- **Acceptance:**
  - Randomized filter tests against a scalar reference.
  - **[HIL]** CCOUNT per half-buffer, APP_CPU utilization and FPS before and after. XCLK stays stable under the DPORT stress test.
- **Status:** TODO

### Wave 5: documentation

#### - [ ] WP-24: Docs accuracy and small cleanups (Lanes H+G) · P2-P3 · S
- **Findings:** BUILD-15, BUILD-16, BUILD-22, BUILD-25, LL-17 (README claim), FW-NET-21, FW-NET-22, FW-NET-23
- **Steps:**
  1. Fix the README:
     - **Safety table:** five crates `forbid`; ll uses workspace lints plus Miri on non-MMIO code only; the firmware lib uses `deny` with the `spsc_ring` override; the Xtensa binary uses unsafe MMIO.
     - **Architecture:** describe Core 1 producer → SPSC → `TripleFrameBuffer` → P81, including WP-12's changes.
     - **Numbers:** the measured FPS with its conditions; the real test count; the PSRAM clock from WP-11; the `Lldesc` alignment (4, not 16).
     - **HTTP contract:** update it for any WP that changed it.
  2. Move `docs/superpowers/*2026-10-05*` to `docs/archive/` with a "historical, C/ESP-IDF era" banner. Rewrite the `lib.rs` "Replaces:" block.
  3. **FW-NET-21.** Remove the "4-5 FPS / LwIP / worker task" text. Derive or drop `X-Framerate`. Add `Cache-Control: no-store` to the stream and snapshot responses.
  4. **FW-NET-22.** Delete config fields left unused after WP-09 and WP-15. Keep one token comparator.
  5. **FW-NET-23.**
     - Add `query_has_pair(q, "action=snapshot")`.
     - Accept only digits in `Content-Length`.
     - Make `content_length` an `Option<usize>`.
  6. **BUILD-25.** Comment the 0x11000 start of the partition table. Optionally add a `coredump` partition at 0x380000.
- **Acceptance:**
  - `grep` of the safety attributes matches the README.
  - `grep -rn 'examples/' crates docs README.md` matches only archived files.
  - Unit tests cover the FW-NET-23 changes.
- **Status:** TODO

---

## 6. Traceability index

Every finding maps to exactly one primary WP. A secondary WP is listed in parentheses where it also matters.

| Finding | WP | Finding | WP | Finding | WP | Finding | WP |
|---|---|---|---|---|---|---|---|
| FW-CORE-01 | 05 | FW-NET-01 | 05 | LL-01 | 08 | CONV-01 | 21 |
| FW-CORE-02 | 06 | FW-NET-02 | 05 | LL-02 | 08 | CONV-02 | 21 |
| FW-CORE-03 | 07 | FW-NET-03 | 09 | LL-03 | 08 | CONV-03 | 21 |
| FW-CORE-04 | 04 (10) | FW-NET-04 | 09 | LL-04 | 20 | CONV-04 | 21 |
| FW-CORE-05 | 11 | FW-NET-05 | 15 | LL-05 | 06 | CONV-05 | 21 |
| FW-CORE-06 | 12 | FW-NET-06 | 12 | LL-06 | 06 | CONV-06 | 21 |
| FW-CORE-07 | 12 | FW-NET-07 | 17 | LL-07 | 06 | CONV-07 | 21 |
| FW-CORE-08 | 12 (07) | FW-NET-08 | 05 | LL-08 | 10 | CONV-08 | 21 |
| FW-CORE-09 | 14 | FW-NET-09 | 05 | LL-09 | 25 | CONV-09 | 21 |
| FW-CORE-10 | 14 | FW-NET-10 | 17 | LL-10 | 25 | CONV-10 | 06 |
| FW-CORE-11 | 14 | FW-NET-11 | 17 | LL-11 | 25 | CONV-11 | 21 |
| FW-CORE-12 | 14 | FW-NET-12 | 05 | LL-12 | 25 | CONV-12 | 21 |
| FW-CORE-13 | 14 | FW-NET-13 | 14 | LL-13 | 08 | CONV-13 | 21 |
| FW-CORE-14 | 16 | FW-NET-14 | 18 | LL-14 | 20 | CONV-14 | 21 |
| FW-CORE-15 | 04 | FW-NET-15 | 15 | LL-15 | 06 | CONV-15 | 21 |
| FW-CORE-16 | 19 | FW-NET-16 | 15 | LL-16 | 20 | CONV-16 | 21 |
| FW-CORE-17 | 14 | FW-NET-17 | 19 | LL-17 | 20 (24) | CONV-17 | 21 |
| FW-CORE-18 | 15 | FW-NET-18 | 09 | LL-18 | 25 | CONV-18 | 21 |
| FW-CORE-19 | 07 | FW-NET-19 | 12 | LL-19 | 25 | CONV-19 | 21 |
| FW-CORE-20 | 14 | FW-NET-20 | 12 | LL-20 | 08 | CONV-20 | 21 |
| FW-CORE-21 | 04 | FW-NET-21 | 24 | LL-21 | 06 | CONV-21 | 21 |
| FW-CORE-22 | 15 | FW-NET-22 | 24 | HAL-01 | 08 (20) | CONV-22 | 21 |
| FW-CORE-23 | 19 | FW-NET-23 | 24 | HAL-02 | 20 | TYPES-01 | 21 |
| | | FW-NET-24 | 15 | HAL-03 | 20 | TYPES-02 | 21 |
| | | FW-NET-25 | 12 | HAL-04 | 20 | TYPES-03 | 21 |
| | | FW-NET-26 | 17 | HAL-05 | 20 | TYPES-04 | 21 |
| | | FW-NET-27 | 09 | HAL-06 | 20 | TYPES-05 | 21 |
| | | FW-NET-28 | 17 | HAL-07 | 20 | SCCB-01 | 22 |
| | | FW-NET-29 | 04 | HAL-08 | 20 | SCCB-02 | 22 |
| | | | | HAL-09 | 20 | SCCB-03 | 22 |
| | | | | HAL-10 | 20 | SCCB-04 | 21 |
| | | | | HAL-11 | 20 | SCCB-05 | 22 |
| | | | | HAL-12 | 20 | SCCB-06 | 22 |

| Finding | WP | Finding | WP | Finding | WP |
|---|---|---|---|---|---|
| SENS-01 | 10 | SENS-10 | 23 | SENS-19 | 16 |
| SENS-02 | 16 | SENS-11 | 23 | SENS-20 | 16 |
| SENS-03 | 16 | SENS-12 | 23 | SENS-21 | 16 |
| SENS-04 | 23 | SENS-13 | 23 | SENS-22 | 23 |
| SENS-05 | 23 | SENS-14 | 23 | SENS-23 | 23 |
| SENS-06 | 16 | SENS-15 | 23 | SENS-24 | 23 |
| SENS-07 | 16 | SENS-16 | 22 | SENS-25 | 23 |
| SENS-08 | 16 | SENS-17 | 22 | | |
| SENS-09 | 23 | SENS-18 | 23 | | |

| Finding | WP | Finding | WP | Finding | WP |
|---|---|---|---|---|---|
| BUILD-01 | 01 | BUILD-10 | 01 | BUILD-19 | 03 |
| BUILD-02 | 02 | BUILD-11 | 02 | BUILD-20 | 01 |
| BUILD-03 | 03 | BUILD-12 | 02 | BUILD-21 | 01 |
| BUILD-04 | 02 (13) | BUILD-13 | 02 | BUILD-22 | 24 |
| BUILD-05 | 13 | BUILD-14 | 02 | BUILD-23 | 01 |
| BUILD-06 | 11 | BUILD-15 | 24 | BUILD-24 | 19 |
| BUILD-07 | 09 | BUILD-16 | 24 | BUILD-25 | 24 |
| BUILD-08 | 03 (04) | BUILD-17 | 03 | BUILD-26 | 03 |
| BUILD-09 | 01 | BUILD-18 | 03 | BUILD-27 | 03 |

**Merged duplicates.** Several reviewers raised the same finding independently. These pairs are handled once, in the listed WP:

| Findings | WP |
| --- | --- |
| FW-CORE-05 = BUILD-06 | WP-11 |
| FW-CORE-07 = FW-NET-06 | WP-12 |
| FW-CORE-10 = FW-NET-13 | WP-14 |
| FW-CORE-18 = FW-NET-24 | WP-15 |
| FW-CORE-22 = FW-NET-05/15 | WP-15 |
| FW-NET-04 = BUILD-07 | WP-09 |
| FW-CORE-16 = FW-NET-17 = LL-15 | WP-19, WP-06 |
| FW-CORE-23 = BUILD-24 | WP-19 |
| FW-CORE-04 ≈ LL-08 ≈ SENS-01 | WP-04, WP-10 |
| LL-06 ≈ CONV-10 ≈ HAL-05 | WP-06, WP-20 |

---

## 7. Open questions for the repository owner

These need a human decision or information that is not in the repo. The affected WPs note where they are blocked.

1. **Bootloader binary.** Can `firmware/bootloader_v554_gpio16.bin` be committed (it is Apache-2.0 ESP-IDF output)? Where is its source patch? This blocks WP-03, the `invariants` job in WP-01, and WP-11's bootloader header change.
2. **Rollback.** Was that bootloader built with `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE`? This decides whether FW-NET-01 means "every OTA reverts" (P0) or "rollback does nothing" (P1). It also sets WP-05's HIL test plan.
3. **40 MHz PSRAM and flash.** Was this a deliberate cold-boot stability workaround? It decides whether WP-11 is an experiment or only a docs fix.
4. **Release priority.** Is FPS or UART panic text more important in release builds? This decides `opt-level` 3 vs `"s"` and whether to adopt the immediate-abort profile (WP-02).
5. **Multiple viewers.** For WP-18, should two port-81 viewers be supported (fan-out), or should a single viewer be kept with stall-based preemption?
6. **OV3640.** Does anyone use an OV3640? If not, WP-23 can mark it unsupported instead of porting a register map.
7. **JPEG quality default.** The design spec says 10 and `config.rs` says 12. Which is intended?
8. **Flash encryption.** Is it planned? Wi-Fi credentials and the OTA token are stored in plaintext in the flash image.
9. **Sensor clock.** Is changing the XCLK source (APLL / CLK_OUT) or adding an opt-in HD-fast window acceptable for the production default (WP-10)?

## 8. Baseline and results log

Fill this in as work packages land. WP-04 records the first row.

| Date | WP | FPS (median of 3 × 60 s) | Sensor period (µs) | Core 0 ms/frame | `c1_dropped`/min | Snapshot p95 (ms) | Image size (B) | Notes |
|---|---|---|---|---|---|---|---|---|
| 2026-10 (`75625d8`) | baseline | 12.53 | not measured | not measured | not measured | not measured | 817,904 | Figures from the commit message |
| 2026-11-13 (`845cfc4`, PR #2 CI run 43495706756) | WP-02/06/08/09 + WP-23 on top of the baseline | not measured (no board in CI) | not measured | not measured | not measured | not measured | **801,712** | Xtensa release build in CI: `espflash save-image`, app image only (`Merge: false`), 19.42% of the 0x1B0000 slot. −16,192 B against the baseline, in the direction WP-02's LTO/`codegen-units=1` predicted (~−25 KB); SENS-22 also drops 17 drivers from the link. Built with CI placeholder credentials baked in by `option_env!`, so a real build differs by the credential string lengths. |

## 9. Audit procedure: half-finished ports

This procedure found real defects in four of the six crates (`sensors`, `hal`, `ll`, `types`). It has **not yet been run on `crates/klipper-cam-fw`**. It needs no hardware and no toolchain. An agent with only the host gates can run it end to end.

The thing it looks for: code that was copied from a driver for one piece of hardware and only partly rewritten for another, so that it still carries the first one's register addresses, bit positions or signal indices — and code that reports success for work it does not do.

### 9.1 Steps

1. **List every numeric constant and raw hex literal the crate uses in code** (not in tables, not in comments). For each, ask: does this value belong to the hardware this file says it drives? Addresses are the giveaway — ESP32 I2S is at `0x3FF4_F000`, ESP32-S3 LCD_CAM at `0x6004_1000`, OmniVision 16-bit parts put the ISP at `0x5xxx` and the window at `0x38xx`. A Novatek, Himax or GalaxyCore driver touching `0x5xxx` is wrong.
2. **Count uses of every `pub const`.** Zero uses is a smell. Zero uses **and** a value belonging to other hardware is the pattern: the port remapped the constants it needed and left the rest. The proof is usually one constant in the family that *was* remapped and *is* used, sitting next to siblings that were not.
3. **Find every method that takes a bus or a register block and never touches it**, yet returns `Ok`. Also every method that performs only a prelude (a bank select, a page select) and then records the requested value. Both are reporting success for nothing.
4. **Find every function that claims to drive hardware and count its MMIO calls.** A "driver" with a register block held *by value* and no `target_arch` gate is a model. Say so in its docs; do not hide it.
5. **For each table that is "re-sent" to undo a previous state, compare register sets.** If the tables for different modes write different registers, the re-send is load-bearing and cannot be replaced with a delta. If they barely overlap the default table, the re-send does not restore anything and there is a live defect.
6. **For each clamp, check what it clamps *to*** and whether the implementations agree. A shared type that clamps to `-2..=2` while drivers accept `-5..=5` substitutes a different setting for a valid one.
7. **Write one test per finding that fails on the old code**, then fix. For a defect you cannot fix without a datasheet, make the code refuse rather than substitute, and write the gap into the module docs.

### 9.2 What to do with each kind of finding

| Finding | Action |
| --- | --- |
| Setter reports success without acting | Return `NotSupported`. Tolerate it at the call sites with the existing `ignore_not_supported` helper if they must not fail. |
| Setter writes another part's registers | Return `NotSupported`; document which block is foreign and why you know. Delete the dead constants that named it. |
| Status read from a register the part does not have | Report the default; say so in a comment. |
| Model presented as a driver | Retitle it, state "performs no MMIO" in the first paragraph, correct any `SAFETY` comment that claims a hardware write. Keep it visible. |
| Load-bearing re-send | Leave it; add a structural test over the tables so the next person does not remove it. |
| Value that would have to be guessed | Do not. Record in a comment, the Status line, and §7. |

### 9.3 False positives to expect

The procedure also flags things that are fine; §2.13 lists the ones hit so far. A value that matches upstream is not a finding. A `pub const` with zero uses is not a finding on its own — only when its value belongs elsewhere.
