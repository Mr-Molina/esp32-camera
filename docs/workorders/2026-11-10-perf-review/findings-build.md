# Findings: build, CI, tooling, scripts, docs (BUILD)

**Baseline commit:** `5a21ce2`. **Scope:** workspace and crate manifests, `Cargo.lock`, `rust-toolchain.toml`, `.cargo/config.toml`, `espflash.toml`, the partition table, `.github/workflows/*`, `scripts/*.py`, the README, and `docs/**`.

## Toolchain and test results (Windows 11 host, 2026-11-10)

| Check | Result |
|---|---|
| `cargo test --workspace --offline` | **198 passed**, 0 failed, 0 warnings. Per crate: conversions 33, hal 21, ll 23, sccb 13, sensors 28, types 9, fw lib 70, fw main 1. 0 doc-tests. |
| `cargo clippy --workspace --all-targets -- -D warnings` | clean |
| `cargo fmt --all -- --check` | clean |
| Xtensa build exactly as the README gives it | **fails**: `linker ... /home/console-admin/.../xtensa-esp32-elf-gcc not found` |
| Xtensa build with the linker and RUSTFLAGS overridden | success, 0 warnings, 27.5 s |
| Xtensa clippy `-D warnings` | clean |

**Toolchains on this host:** stable 1.99.1; `+esp` 1.97.0-nightly with xtensa GCC 15.2.0 (the config names 16.2.0); espflash 4.6.0; Python 3.13. Miri, cargo-deny and cargo-audit are not installed.

**Image size:** ELF 1,064,596 B; app image 817,904 B, which is 46.2% of the 0x1B0000 slot. Sections: `.text` 589,733; `.rodata` 86,304; `.rwtext` (IRAM) 25,928; `.rwtext.wifi` 51,416; `.bss` 138,728.

**Profile experiments** (environment overrides only; size measured with `espflash save-image`):

| Variant | App image | Δ | `.text` | `.rwtext` |
|---|---|---|---|---|
| Current (no `[profile]`) | 817,904 | - | 589,733 | 25,928 |
| `lto="fat"`, `codegen-units=1` | 792,944 | −24,960 | 588,701 | 20,176 |
| fat LTO + `opt-level="s"` | 704,320 | −113,584 | 500,525 | 19,316 |
| fat LTO + `-Cpanic=immediate-abort` | 762,144 | −55,760 | 575,393 | 30,640 |

**Scripts:**
- `verify_cold_boot_invariants.py` reports PASS, but it silently skipped the bootloader check because the file is missing.
- `ota_deploy.py --stage-only` fails from the repo root because of BUILD-03.

**Note for executing agents:** the reviewer's `cargo +nightly --version` probe made rustup auto-install the `nightly-x86_64-pc-windows-msvc` toolchain on this host. Remove it with `rustup toolchain uninstall nightly` if it is not wanted.

**Worktree caveat:** a worktree nested inside the main checkout also loads the parent `.cargo/config.toml`, so both `rustflags` arrays are concatenated and the link fails with `linkall.x appears multiple times`. In nested worktrees, set `RUSTFLAGS` explicitly.

---

### BUILD-01: CI builds a nonexistent ESP-IDF example; no Rust gate runs at all
- **Severity:** P0 · **Category:** ci
- **Location:** .github/workflows/build.yml:1-32
- **Evidence:** `working-directory: examples/camera_example` · `idf.py build`. The `examples/` directory does not exist.
- **Problem:** All 35 matrix jobs fail on every push. No tests, clippy, fmt, Miri or Xtensa build run. The workflow uses `actions/checkout@v1`.
- **Proposed fix:** Delete `build.yml` and add `.github/workflows/rust.yml` with four jobs:
  - `host`: fmt check, `clippy -D warnings --locked`, `test --locked`, using `dtolnay/rust-toolchain` and `Swatinem/rust-cache`.
  - `miri`: nightly, `cargo miri test -p esp32-cam-ll -p esp32-cam-conversions -p esp32-cam-types`.
  - `xtensa`: `esp-rs/xtensa-toolchain@v1.6`, Xtensa clippy and release build, `espflash save-image`, a size gate of `stat -c %s fw.bin` ≤ `0x1B0000`, and upload of the artifact.
  - `invariants`: `python3 scripts/verify_cold_boot_invariants.py`.

  The full YAML is in the original review and is reproduced in WO-01 of the work order README.
- **Effort:** M
- **Verification:** A PR shows 4 green jobs. A deliberately added clippy warning turns the job red.

### BUILD-02: `.cargo/config.toml` hard-codes one Linux user's absolute linker path
- **Severity:** P0 · **Category:** build
- **Location:** .cargo/config.toml:1-6
- **Problem:** The firmware links only on one machine. The setting is redundant anyway: the target spec already defaults to `xtensa-esp32-elf-gcc` on PATH. The `-nostartfiles -Tlinkall.x` flags are correct for esp-hal 1.2.2.
- **Proposed fix:**
  ```toml
  [target.xtensa-esp32-none-elf]
  runner = "espflash flash --monitor --chip esp32 --partition-table partitions_rust_ota.csv --target-app-partition ota_0"
  rustflags = ["-C", "link-arg=-nostartfiles", "-C", "link-arg=-Tlinkall.x"]

  [alias]
  fw       = "build  --target xtensa-esp32-none-elf -Zbuild-std=core,alloc -p klipper-cam-fw --release"
  fw-check = "clippy --target xtensa-esp32-none-elf -Zbuild-std=core,alloc --workspace --release -- -D warnings"
  ```
  Do not set `[build] target`, because that would break host tests.
- **Effort:** S
- **Verification:** On a fresh clone with `export-esp` sourced, `cargo +esp fw` links.

### BUILD-03: The bootloader `espflash.toml` points to is gitignored and missing
- **Severity:** P0 · **Category:** build
- **Location:** espflash.toml:4-6; .gitignore:6 (`*.bin`)
- **Problem:** `espflash save-image`/`flash` and `ota_deploy.py` fail from the repo root with a misleading error ("Error while connecting to device … os error 3"). The GPIO16 PSRAM-CS pull-up invariant depends on this patched bootloader.
- **Proposed fix:** Commit the binary with an ignore exception (`!firmware/bootloader_v554_gpio16.bin`), plus a SHA-256 and provenance notes (IDF v5.5.4 commit and patch). Alternatively, fetch it from a release asset. Run espflash with `cwd=REPO_ROOT` (see BUILD-18).
- **Effort:** S. **Blocked on:** the user must supply the binary.
- **Verification:** On a fresh clone, `ota_deploy.py --stage-only` exits 0, and the invariants script checks the binary instead of skipping it.

### BUILD-04: No `[profile.release]` at all
- **Severity:** P1 · **Category:** performance
- **Location:** Cargo.toml:1-34
- **Problem:** No cross-crate inlining of the ll, conversions and SPSC hot paths into the firmware. Fat LTO with `codegen-units=1` saves 25 KB of image and **5.7 KB of IRAM**. With `debug=0`, esp-backtrace addresses cannot be symbolized.
- **Proposed fix:**
  ```toml
  [profile.release]
  opt-level = 3            # A/B against "s" on hardware FPS
  lto = "fat"
  codegen-units = 1
  debug = 2                # ELF only, not flashed
  overflow-checks = false
  debug-assertions = false
  incremental = false

  [profile.dev]
  opt-level = "s"
  ```
- **Effort:** S
- **Verification:** Compare sizes, and A/B FPS for opt-level 3 vs `"s"` against the 12.53 FPS baseline.
- **Prerequisite:** FW-CORE-03 (aliasing UB) must be fixed first, because LTO can change codegen around the UB.

### BUILD-05: Hot loops run from flash cache, not IRAM
- **Severity:** P1 · **Category:** performance
- **Location:** esp32-cam-ll/src/dma_filter.rs; klipper-cam-fw/src/spsc_ring.rs; main.rs (`drain_core1_spsc_to_triple`)
- **Evidence:** `nm`:
  ```
  40129ff8 t esp32_cam_ll::dma_filter::filter_jpeg
  401309c8 T Core1JpegSpsc<32>::push_chunk
  401305b8 T ...pop_with
  40127448 t klipper_cam_fw::drain_core1_spsc_to_triple
  ```
- **Problem:** Flash and PSRAM share each core's 32 KB cache and the SPI bus, so I-cache misses compete with frame traffic. IRAM has about 50 KB of headroom.
- **Proposed fix:**
  - Put `#[cfg_attr(target_arch="xtensa", esp_hal::ram)]` on these functions: the filters, push/pop, the drain, and the Core 1 loop.
  - Mark small callees `#[inline(always)]`.
  - Because ll cannot depend on esp-hal, use `#[link_section = ".rwtext"]` gated on xtensa, or RAM thunks in the firmware.
- **Effort:** M
- **Verification:** `nm` shows these symbols at 0x4008_xxxx–0x4009_xxxx. A/B test FPS.

### BUILD-06: PSRAM and flash run at 40 MHz while the README says 80 MHz
- **Severity:** P1 · **Category:** performance/docs
- **Location:** main.rs:4447-4451; espflash.toml:8-11; README.md:19, 62, 67; verify_cold_boot_invariants.py:205
- **Proposed fix:** Either document 40 MHz as intentional, or A/B test `PsramCacheF80mS80m` with flash in QIO at 80 MHz (and a matching bootloader), followed by a 24 h soak. Overlaps FW-CORE-05.
- **Effort:** S (docs) / M (A/B)
- **Verification:** FPS comparison and soak test.

### BUILD-07: A release build silently uses the public OTA token and an empty Wi-Fi password
- **Severity:** P1 · **Category:** build/security
- **Location:** config.rs:15, 21; README.md:124-128
- **Proposed fix:** Add a klipper-cam-fw `build.rs` that panics when `CARGO_CFG_TARGET_ARCH=xtensa`, `PROFILE=release`, and `WIFI_PASS` or `WIFI_OTA_TOKEN` is unset or empty. Emit `rerun-if-env-changed` for both. Put a placeholder in the README. Overlaps FW-NET-04.
- **Effort:** S
- **Verification:** A release build without the variables fails. Host `cargo test` is unaffected.

### BUILD-08: `verify_cold_boot_invariants.py` passes vacuously; its live check can never pass
- **Severity:** P1 · **Category:** scripts
- **Location:** verify_cold_boot_invariants.py:265-271, 282, 318-320; http.rs:523-524
- **Problem:**
  - It returns PASS when the bootloader is missing.
  - `/status` has no `reset_reason`, so the live check always fails.
  - It accepts a 3-byte pattern as proof of the pull-up.
  - Its substring greps also match text inside comments.
- **Proposed fix:**
  - Add `--require-bootloader` (default on in CI).
  - Add `reset_reason` to `/status` and to the README table, or drop the check.
  - Match the full 5-byte instruction sequence.
  - Strip `//` comments before grepping.
- **Effort:** S
- **Verification:** Running without `firmware/` returns rc 1. Running against a live device passes.

### BUILD-09: `upload_component.yml` would publish this repo as `espressif/esp32-camera`
- **Severity:** P1 · **Category:** ci
- **Location:** .github/workflows/upload_component.yml:1-19
- **Proposed fix:** Delete the workflow. Optionally add a tag-triggered GitHub Release job that attaches `fw.bin` and its SHA-256.
- **Effort:** S
- **Verification:** A tag on a fork triggers no upload.

### BUILD-10: Neither the host toolchain nor the esp toolchain is pinned
- **Severity:** P1 · **Category:** build
- **Location:** rust-toolchain.toml:1-3; .cargo/config.toml:2
- **Proposed fix:**
  - Set `channel = "1.99.1"`.
  - Pin esp to `1.97.0.0` in the README (`espup install --toolchain-version`) and in CI.
  - Bump both through PRs.
- **Effort:** S
- **Verification:** CI logs show the same `rustc -V` on every run.

### BUILD-11: Unused smoltcp features
- **Severity:** P2 · **Category:** dependencies
- **Location:** klipper-cam-fw/Cargo.toml:27
- **Problem:** `socket-raw`, `socket-icmp`, `socket-udp`, `proto-dhcpv4` (already implied) and `iface-max-addr-count-2` are enabled but unused. Each enabled socket type enlarges every socket slot.
- **Proposed fix:** `features = ["medium-ethernet", "proto-ipv4", "socket-tcp", "socket-dhcpv4"]`. Revisit the address count after FW-NET-05/15: one static address plus one DHCP address may still need 2.
- **Effort:** S
- **Verification:** Size shrinks, ping still works, and DHCP still works.

### BUILD-12: Unused `embassy-futures` dependency
- **Severity:** P2 · **Category:** dependencies
- **Location:** klipper-cam-fw/Cargo.toml:25
- **Proposed fix:** Delete it. Consider `cargo-machete` in CI.
- **Effort:** S
- **Verification:** The build passes.

### BUILD-13: Two `esp32` PAC versions; pre-release radio; duplicate transitive crates
- **Severity:** P2 · **Category:** dependencies
- **Location:** klipper-cam-fw/Cargo.toml:17, 23; Cargo.lock
- **Problem:** esp-hal pulls PAC 0.41 while esp-radio and esp-phy pull 0.42. Several other crates are duplicated: embassy-sync ×3, rand_core ×3, plus heapless, embedded-io, digest and embedded-hal ×2 each.
- **Proposed fix:** Move to a matched esp-hal/esp-radio release pair. Add `cargo deny` with bans `multiple-versions = "warn"`.
- **Effort:** M
- **Verification:** `cargo +esp tree -d` shows a single `esp32` PAC.

### BUILD-14: No panic-immediate-abort size profile
- **Severity:** P2 · **Category:** performance/size
- **Problem:** Immediate-abort saves another 30.8 KB over fat LTO, but loses panic messages. On this nightly the old build-std feature errors out, and the new form is `-Zunstable-options -Cpanic=immediate-abort`.
- **Proposed fix:** Make it an opt-in `[profile.release-tiny]` only.
- **Effort:** S
- **Verification:** Size comparison, and a boot test confirming the watchdog resets after a panic.

### BUILD-15: README safety claims don't match the code
- **Severity:** P2 · **Category:** docs
- **Location:** README.md:5, 17, 19; klipper-cam-fw/src/lib.rs:12; spsc_ring.rs:8; main.rs:15-24
- **Problem:** Only 5 crates use `forbid(unsafe_code)`, not 6. ll has no crate-level attribute. The firmware lib uses `deny` with an override in `spsc_ring`. The Xtensa bin allows `static_mut_refs`, `undocumented_unsafe_blocks` and `unwrap_used`.
- **Proposed fix:** Correct the table. Consider re-enabling `undocumented_unsafe_blocks` on Xtensa.
- **Effort:** S
- **Verification:** A grep of the safety attributes matches the README.

### BUILD-16: README architecture describes the old ping-pong design and claims 92+ tests
- **Severity:** P2 · **Category:** docs
- **Location:** README.md:64-75, 106, 109-110
- **Proposed fix:** Describe the triple buffer and the Core 1 SPSC design. State the measured FPS and its conditions. Use the real test count (198).
- **Effort:** S
- **Verification:** Review the README against main.rs.

### BUILD-17: Docs and scripts hard-code a home directory, LAN IP and USB path
- **Severity:** P2 · **Category:** docs
- **Location:** README.md:124, 134, 161, 166, 175, 178, 194-195, 200; ota_deploy.py:1, 31; ensure_serial_passive.py:23-24
- **Proposed fix:** Use repo-relative paths and a `<camera-ip>` placeholder. Read `CAM_HOST` and `CAM_SERIAL_PORT` from the environment. Use `#!/usr/bin/env python3`.
- **Effort:** S
- **Verification:** `grep -rn 'console-admin\|172\.22\.' README.md scripts docs` returns nothing, with the archive excepted.

### BUILD-18: `ota_deploy.py` depends on the current directory and is fragile
- **Severity:** P2 · **Category:** scripts
- **Location:** ota_deploy.py:79-87, 352-354, 391-396
- **Proposed fix:**
  - Run espflash with `cwd=REPO_ROOT` and explicit flash arguments.
  - Catch the `ValueError` from `expected_next_slot`.
  - Add `app_sha256` to `/status` and compare it after deploy.
- **Effort:** M
- **Verification:** Stage-only works from another directory, and the SHAs match.

### BUILD-19: `ota_deploy.py` reads the response with a 40-byte heuristic; its docstring has the wrong chunk size
- **Severity:** P3 · **Category:** scripts
- **Location:** ota_deploy.py:211-212, 278-283
- **Proposed fix:** Read until `Content-Length` or EOF. Fix the docstring. Add one connect retry.
- **Effort:** S
- **Verification:** A unit test with the response split across reads.

### BUILD-20: Pre-commit runs only codespell; the CI diff handling is fragile
- **Severity:** P3 · **Category:** ci
- **Location:** .pre-commit-config.yaml; .github/workflows/pre_commit.yml:35-52
- **Proposed fix:**
  - Add `pre-commit-hooks` (toml, yaml, eof, whitespace, large files), ruff, and a local `cargo fmt --check`.
  - Use `--diff-filter=d`, or `pre-commit run --from-ref --to-ref`.
- **Effort:** S
- **Verification:** A PR with bad TOML fails.

### BUILD-21: `stale.yml` uses deprecated `actions/stale@v3`
- **Severity:** P3 · **Category:** ci
- **Location:** .github/workflows/stale.yml:21
- **Proposed fix:** Upgrade to v9, or delete the workflow.
- **Effort:** S
- **Verification:** The scheduled run shows no deprecation warning.

### BUILD-22: Stale ESP-IDF/C-era docs and `examples/` references
- **Severity:** P3 · **Category:** docs
- **Location:** klipper-cam-fw/src/lib.rs:4-9; docs/superpowers/plans/2026-10-05-*; docs/superpowers/specs/2026-10-05-*
- **Proposed fix:** Move the 2026-10-05 docs to `docs/archive/` under a banner. Rewrite the "Replaces:" block in lib.rs.
- **Effort:** S
- **Verification:** A grep for `examples/` matches only archived files.

### BUILD-23: Missing `deny.toml`, `CHANGELOG.md` and `clippy.toml`; no audit
- **Severity:** P3 · **Category:** hygiene
- **Proposed fix:**
  - Add `deny.toml` with licenses, bans and advisories, run in CI through `cargo-deny-action`.
  - Add a `CHANGELOG.md`.
  - Add `clippy.toml` with `msrv` and `too-many-lines-threshold`.
- **Effort:** S
- **Verification:** The deny job is green.

### BUILD-24: The release image carries esp-backtrace colors, the HIL self-test and the JPEG decoder
- **Severity:** P3 · **Category:** build. **Merged with:** FW-CORE-23
- **Location:** klipper-cam-fw/Cargo.toml:20; main.rs:247
- **Problem:** About 25 KB of flash and I-cache footprint. `capture_live_hd_jpeg_with_diag` is instantiated twice (0x10fa + 0x150a bytes).
- **Proposed fix:**
  - Set `default-features = false` on esp-backtrace.
  - Put the HIL code and the decoder behind a `hil` feature.
  - Make the capture function non-generic.
- **Effort:** M
- **Verification:** `nm` no longer shows `decode_jpeg_internal` or `init_psram_and_run_hil`.

### BUILD-25: Unused flash regions and no coredump partition
- **Severity:** P3 · **Category:** build
- **Location:** partitions_rust_ota.csv:2-6
- **Proposed fix:** Comment why the table starts at 0x11000. Consider `coredump, data, coredump, 0x380000, 0x10000`.
- **Effort:** S
- **Verification:** `espflash partition-table` shows no overlaps.

### BUILD-26: The udev rule is pinned to one PC's USB port
- **Severity:** P3 · **Category:** hygiene
- **Location:** docs/99-esp32-wrover.rules:2
- **Proposed fix:** Drop `ID_PATH`, or document it. Make the scripts default to `/dev/ttyUSB-esp32`.
- **Effort:** S
- **Verification:** The symlink appears when the board is on a different USB port.

### BUILD-27: Scripts are not in CI or pre-commit; no stated Python minimum; no tests
- **Severity:** P3 · **Category:** scripts
- **Location:** ensure_serial_passive.py:16-21, 46-51; ota_deploy.py:45
- **Proposed fix:**
  - Require Python 3.10 or later.
  - Add ruff and `py_compile` to pre-commit.
  - Add pytest tests for `parse_jpeg_sof0`, the response parser and `expected_next_slot`.
- **Effort:** S
- **Verification:** The CI Python job is green.

---

## What looks good
- **Host gates are healthy:** 198 tests pass, and clippy and fmt are clean. The Xtensa clippy and release build have 0 warnings.
- **Workspace hygiene:** lints are centralized, the lockfile is committed, and the Xtensa-only dependencies are correctly target-scoped.
- **No logging bloat:** smoltcp uses `default-features = false`, with no `log` or `defmt` in the graph.
- **Size headroom:** the image fills 46% of the slot, and the slot math matches across the CSV, the README and the script.
- **`ota_deploy.py` validation:** it checks the magic number, segments, SHA and size before uploading. It guards against CRLF injection, uses bounded timeouts, and keeps the token out of the URL.
- **Secrets handling:** secrets come in through `option_env!`, and `.env*.local` is gitignored.

## Open questions
1. Is PSRAM and flash at 40 MHz a deliberate stability workaround?
2. Can `firmware/bootloader_v554_gpio16.bin` be committed, and where is its source patch?
3. Which matters more for release: FPS or keeping panic messages? This decides opt-level 3 vs `"s"` and whether to use immediate-abort.
4. Should CI build the S2/S3 paths?
5. Is there an esp-hal/esp-radio release pair that shares PAC 0.42?
