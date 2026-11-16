# CLAUDE.md — esp32-camera

Pure bare-metal `#![no_std]` Rust workspace for an ESP32-WROVER OV2640 camera and Klipper HD MJPEG streaming firmware: the application firmware, a pure-Rust second-stage bootloader, and clean-room RF/clock, WPA2 and Wi-Fi MAC crates that replace the Espressif C archives. Full architecture and hardware notes are in `README.md`; bootloader provenance is in `firmware/README.md`.

## Where work stands (read first)

- Current handoff: `docs/superpowers/plans/2026-11-16-rea-pure-rust-handoff-plan.md` (base plan: `docs/superpowers/plans/2026-11-14-rea-full-rust-firmware-plan.md`). Phases 0–3B are executed and verified. Phase 3C (`esp32-cam-wifi-mac`) fixes 3C-1..3C-3 are implemented, unit-tested and on `master`, but not yet flashed: the next action is the live STA association test on the Linux host with the board, which needs explicit user confirmation. Start at the "Resume Here" section of the handoff. Phases 3D (PHY crate) and 3E (app-desc cutover, drop `--allow-phase1`) are queued; do not start 3D before 3C is certified on silicon.
- The handoff was written on a Linux host (`/home/console-admin/...`, `/dev/serial/by-id/...`, `eno1`). The repo is also worked on from Windows; see the host notes under "Build and verify".
- Older workorder findings live in `docs/workorders/`; superseded designs in `docs/archive/`.

## Branch policy

- Work directly on `master` (the default branch). Do not create feature branches unless the user asks.
- Push only to `master` unless told otherwise. Never force-push.
- Commit and push only when the user asks.
- The Claude desktop app may run a session in a worktree under `.claude/worktrees/` (gitignored). Run every command from that worktree root; never `cd` into the main checkout from it.

## Build and verify (run before claiming work is done)

Toolchain is pinned in `rust-toolchain.toml` (Rust 1.99.1, with rustfmt and clippy). Host-side tooling is pure Rust (`crates/xtask`): there are no Python or shell scripts in this repo any more, and `cargo xtask verify-pure-rust` fails if a `.py`, `.sh`, `.c`, `.cpp`, `.h` or `.S` file is added anywhere outside `.git/`, `target/` and `.agents/`.

Host gates (match the `host` and `invariants` jobs in `.github/workflows/rust.yml`):

```bash
cargo fmt --all -- --check
cargo clippy --workspace --all-targets --locked -- -D warnings
cargo test --workspace --locked
cargo deny check
cargo xtask verify-invariants --allow-missing-bootloader
cargo xtask verify-pure-rust --allow-phase1
```

Miri (nightly; CI `miri` job):

```bash
cargo +nightly miri test -p esp32-cam-ll -p esp32-cam-conversions -p esp32-cam-types -p esp32-cam-bootloader -p esp32-cam-rf-clock -p esp32-cam-wpa2 -p esp32-cam-wifi-mac
```

Firmware (Xtensa, needs the `esp` toolchain from `espup` and `-Zbuild-std`; no hardware needed to build):

```bash
cargo +esp clippy --target xtensa-esp32-none-elf -Zbuild-std=core,alloc --workspace --release -- -D warnings
cargo +esp build --target xtensa-esp32-none-elf -Zbuild-std=core,alloc -p klipper-cam-fw --release
cargo xtask build-bootloader
cargo xtask verify-invariants
```

`cargo +esp fw` and `cargo +esp fw-check` are aliases for the first two (see `.cargo/config.toml`). CI additionally runs `espflash save-image` and enforces a firmware size gate of at most `0x1B0000` bytes (1,769,472 B). The CI `host` and `xtensa` jobs stage the upstream `esp-wifi-sys-esp32` 0.3.0 crate for `crates/esp-wifi-sys-esp32-seam`; locally the seam `build.rs` finds it in the cargo registry after `cargo fetch`, or via `ESP_WIFI_SYS_ESP32_UPSTREAM_DIR`.

- Use `--locked`. Do not change `Cargo.lock` unless a dependency change requires it.
- The second-stage bootloader is built from `crates/esp32-cam-bootloader` by `cargo xtask build-bootloader` into `target/xtensa-esp32-none-elf/release/esp32-cam-bootloader.bin`. No bootloader binary is committed (the legacy `firmware/bootloader_v554_gpio16.bin` was deleted in the pure-Rust migration), and `verify-pure-rust` without `--allow-phase1` rejects one.
- Hardware and LAN subcommands touch a real device: `cargo xtask ota-deploy`, `cargo xtask ensure-serial-passive`, `espflash flash`, and `cargo run --target xtensa-esp32-none-elf` (its runner flashes). Never run them without explicit user confirmation.
- Windows host notes: `cargo xtask` builds the tooling for the current host (the alias deliberately pins no `--target`). `verify-invariants` skips the udev and `/dev/serial` checks when they do not exist. `ensure-serial-passive` is Linux-only.
- Nested worktree note: a worktree under `.claude/worktrees/` sits inside the main checkout, so cargo merges both `.cargo/config.toml` files and doubles the Xtensa linker flags. The firmware link then fails with `linker script file 'linkall.x' appears multiple times`. For Xtensa builds from such a worktree, set `RUSTFLAGS="-C link-arg=-nostartfiles -C link-arg=-Tlinkall.x"`, which replaces the merged value. Host gates and CI are unaffected.
- Dependency audit: `cargo deny check` is the CI gate; `cargo audit` is also available locally. The two unmaintained-crate notices (`paste`, `unsafe-libyaml`) are upstream esp-rs build-time transitive deps and are allow-listed in `deny.toml`.

## Workspace layout (13 crates)

- `crates/esp32-cam-types` — shared types, errors, frame sizes, pixel formats
- `crates/esp32-cam-sccb` — SCCB (I2C-like) bus and mock
- `crates/esp32-cam-sensors` — 18 sensor drivers plus golden tests
- `crates/esp32-cam-conversions` — JPEG/YUV/BMP conversion, fixtures and proptests
- `crates/esp32-cam-ll` — low-level I2S/DMA ring, drain, compile-fail UI tests
- `crates/esp32-cam-hal` — driver layer
- `crates/klipper-cam-fw` — firmware binary (HTTP ports 80/81, OTA, Wi-Fi, telemetry)
- `crates/esp32-cam-bootloader` — pure-Rust second-stage bootloader at `0x1000` (GPIO16 PSRAM pull-up, OTADATA rollback, image verify, MMU jump)
- `crates/esp32-cam-rf-clock` — RTC/RF clock, regulatory and printf-compat code replacing Espressif `librtc`/`libregulatory`/`libprintf`
- `crates/esp32-cam-wpa2` — WPA2 supplicant (AES, CCM, EAPOL) replacing `libwpa_supplicant.a`
- `crates/esp32-cam-wifi-mac` — 802.11 STA MAC/LMAC MMIO, DMA ring, rate control, smoltcp PHY replacing `libnet80211.a`/`libpp.a`
- `crates/esp-wifi-sys-esp32-seam` — temporary seam over upstream `esp-wifi-sys-esp32` 0.3.0 that retains only the PHY archive (retired in Phase 3D)
- `crates/xtask` — host tooling: `verify-invariants`, `verify-pure-rust`, `build-bootloader`, `wrap-esp-image-elf`, `ota-deploy`, `ensure-serial-passive`, `bootstrap-env`
- `fuzz/` — cargo-fuzz targets (standalone workspace, not part of the default build)

## Safety rules for this codebase

- Hardware-unsafe code (`unsafe`, DMA, MMIO, PSRAM, static state) must keep `// SAFETY:` comments (`clippy::undocumented_unsafe_blocks` is warn).
- No `unwrap()` in non-test code (`clippy::unwrap_used` is warn). Prefer typed errors.
- Preserve the GPIO16 PSRAM chip-select cold-boot behaviour and the Core 1 / APP_CPU producer invariants documented in `README.md` and `firmware/README.md`. `cargo xtask verify-invariants` checks the static side; changing them needs a hardware A/B, not just tests.
- Do not commit secrets. `.env.local` and `WIFI_*` / `WIFI_OTA_TOKEN` values stay local.

## Working with Claude (Superpowers and token economy)

- Plugins: Superpowers (`Mr-Molina/superpowers`, marketplace name `superpowers-dev`) and Microsoft TokenEconomizer (`microsoft/TokenEconomizer`). Local sessions load them from `.claude/settings.json` (`extraKnownMarketplaces` + `enabledPlugins`). Cloud sessions ignore those keys, so point the cloud environment's setup command at `cargo xtask bootstrap-env`, which installs both with the `claude plugin` CLI.
- Use the Superpowers workflow: brainstorm before building features, write a plan for multi-step changes, use test-driven development for bug fixes and new behaviour, debug with systematic root-cause analysis, and check verification output before claiming success.
- For broad searches across crates, delegate to an Explore subagent and ask for the conclusion, not file dumps.
- Read large files in ranges (`offset`/`limit`) and search before reading. The biggest sources are `klipper-cam-fw/src/main.rs`, `http.rs`, `jpeg_codec.rs`, `esp32_i2s.rs`, and in `esp32-cam-wifi-mac` `sta_fsm.rs` and `lmac_mmio.rs`.
- Do not read `target/`, `Cargo.lock` (unless a dependency question needs it), `.git/`, or `.agents/`.
- Prefer `Grep` with `head_limit` over `cat`-ing files.
- `.agents/` is a gitignored, per-machine governance ledger seeded by the user's wakeup tooling. Do not commit it or depend on it from code. Root `AGENTS.md` is a pointer to this file for non-Claude agents.
