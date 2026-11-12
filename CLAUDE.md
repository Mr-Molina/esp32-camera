# CLAUDE.md — esp32-camera

Pure bare-metal `#![no_std]` Rust workspace for an ESP32-WROVER OV2640 camera and Klipper HD MJPEG streaming firmware. Full architecture and hardware notes are in `README.md`; bootloader provenance is in `firmware/README.md`.

## Branch policy

- Work directly on `master` (the default branch). Do not create feature branches unless the user asks.
- Push only to `master` unless told otherwise. Never force-push.
- Commit and push only when the user asks.

## Build and verify (run before claiming work is done)

Toolchain is pinned in `rust-toolchain.toml` (Rust 1.99.1, with rustfmt and clippy).

Host gates (match CI in `.github/workflows/rust.yml`):

```bash
cargo fmt --all -- --check
cargo clippy --workspace --all-targets --locked -- -D warnings
cargo test --workspace --locked
```

Python scripts and tests:

```bash
ruff check scripts tests
pytest tests
```

Firmware (Xtensa, needs the `esp` toolchain and `-Zbuild-std`; no hardware needed to build):

```bash
cargo clippy --target xtensa-esp32-none-elf -Zbuild-std=core,alloc --workspace --release -- -D warnings
cargo build --target xtensa-esp32-none-elf -Zbuild-std=core,alloc -p klipper-cam-fw --release
```

- Use `--locked`. Do not change `Cargo.lock` unless a dependency change requires it.
- `firmware/bootloader_v554_gpio16.bin` is pending upload by the owner. Image packaging and `ota_deploy.py --stage-only` fail without it. `scripts/verify_cold_boot_invariants.py` accepts `--allow-missing-bootloader` for development only.
- Flashing and live OTA deploys touch real hardware and a LAN device. Never run them without explicit user confirmation.

## Workspace layout

- `crates/esp32-cam-types` — shared types, errors, frame sizes, pixel formats
- `crates/esp32-cam-sccb` — SCCB (I2C-like) bus and mock
- `crates/esp32-cam-sensors` — 18 sensor drivers plus golden tests
- `crates/esp32-cam-conversions` — JPEG/YUV/BMP conversion, fixtures and proptests
- `crates/esp32-cam-ll` — low-level I2S/DMA ring, drain, compile-fail UI tests
- `crates/esp32-cam-hal` — driver layer
- `crates/klipper-cam-fw` — firmware binary (HTTP ports 80/81, OTA, Wi-Fi, telemetry)
- `fuzz/` — cargo-fuzz targets (not part of the default workspace build)
- `scripts/`, `tests/` — Python deploy, OTA and invariant tooling

## Safety rules for this codebase

- Hardware-unsafe code (`unsafe`, DMA, MMIO, PSRAM, static state) must keep `// SAFETY:` comments (`clippy::undocumented_unsafe_blocks` is warn).
- No `unwrap()` in non-test code (`clippy::unwrap_used` is warn). Prefer typed errors.
- Preserve the GPIO16 PSRAM chip-select cold-boot behaviour and the Core 1 / APP_CPU producer invariants documented in `README.md` and `firmware/README.md`. Changing them needs a hardware A/B, not just tests.
- Do not commit secrets. `.env.local` and `WIFI_*` / `WIFI_OTA_TOKEN` values stay local.

## Working with Claude (Superpowers and token economy)

- Plugins are declared in `.claude/settings.json` (`extraKnownMarketplaces` + `enabledPlugins`) so local and cloud sessions both load them: Superpowers (`Mr-Molina/superpowers`) and Microsoft TokenEconomizer (`microsoft/TokenEconomizer`).
- Use the Superpowers workflow: brainstorm before building features, write a plan for multi-step changes, use test-driven development for bug fixes and new behaviour, debug with systematic root-cause analysis, and check verification output before claiming success.
- For broad searches across crates, delegate to an Explore subagent and ask for the conclusion, not file dumps.
- Read large files in ranges (`offset`/`limit`) and search before reading. The biggest sources are `klipper-cam-fw/src/main.rs`, `http.rs`, `jpeg_codec.rs` and `esp32_i2s.rs`.
- Do not read `target/`, `Cargo.lock` (unless a dependency question needs it), or `.git/`.
- Prefer `Grep` with `head_limit` over `cat`-ing files.
