# Design Specification: ESP32 `I2S0` Hardware Route Unification, Ping-Pong PSRAM Double-Buffering, & 4-Socket `smoltcp` Control Pool

- **Date:** 2026-10-24
- **Branch:** `rust-experiment`
- **Target Hardware:** `ESP32-WROVER-DEV` (`ESP32-D0WDQ6-V3` rev 3.0, 240 MHz dual-core Xtensa LX6, 4 MB 80 MHz PSRAM, `OV2640` `1280x720` HD sensor, `/dev/ttyUSB0`, `172.22.1.52`)
- **Crates in Scope:**
  - `crates/esp32-cam-ll` (`src/esp32_i2s.rs`, `src/lib.rs`)
  - `crates/klipper-cam-fw` (`src/lib.rs`, `src/http.rs`, `src/main.rs`)

---

## 1. Intent & Problem Statement

The bare-metal `#![no_std]` Rust firmware on `rust-experiment` has demonstrated 61+ hours of zero-leak continuous operation (`uptime_ms > 220,165,000`, `free_heap = 52,800 B`). Three architectural bottlenecks remain from the initial bring-up:

1. **Split Low-Level `I2S0` Pin/MMIO Configuration**:
   - `crates/esp32-cam-ll/src/esp32_i2s.rs` defines `I2S0I_WS_IN_IDX = 158` instead of the ESP32 Technical Reference Manual / `esp-metadata-generated` input signal index `28` (`I2S0I_WS` / `PCLK`), omits `I2S_PD_CONF_REG` (`0x3FF4_F0A4 = 0x0A` FIFO/PLC memory power-up), and clears bits `24..=27` instead of `27..=30` (`SENSE1..4_MUX_SEL`) on `RTC_IO_SENSOR_PADS_REG` (`0x3FF4_847C`). Consequently, `crates/klipper-cam-fw/src/main.rs` currently maintains a duplicate `xtensa_route_i2s0_camera_pins` helper.
2. **Synchronous Blocking Frame Capture Stalling `smoltcp` TX/RX**:
   - Every `/snapshot` (Port 80) and `/stream` frame (Port 81) currently invokes `capture_live_hd_jpeg_with_diag` synchronously, blocking the main loop for `~70–120 ms` while waiting on `VSYNC` vertical blanking and polling `I2S0` `IN_SUC_EOF` DMA descriptors. During this window, `iface.poll()` cannot process incoming TCP SYNs, ACKs, or transmit queued MJPEG segments.
3. **Single-Socket Port 80 Listener Bottleneck**:
   - Port 80 currently allocates a single `smoltcp` `TcpSocket`. When an HTTP client closes a connection, that socket transitions through TCP `FinWait1`/`LastAck`/`TimeWait` teardown before re-listening on Port 80, causing back-to-back or concurrent HTTP requests from Mainsail/Fluidd/Crowsnest to encounter connection resets or latency spikes.

---

## 2. Goals, Non-Goals, & Anti-Goals

### Goals
1. **Single Authoritative `esp32-cam-ll` Hardware Configuration**:
   - Fix `I2S0I_WS_IN_IDX = 28`, add `I2S_PD_CONF_REG_OFFSET = 0xA4` (`I2S_PD_FIFO_MEM_FORCE_PU = 0x0A`), and fix `RTC_IO_SENSOR_PADS_REG` (`bits 27..=30`) in `crates/esp32-cam-ll/src/esp32_i2s.rs`.
   - Remove the duplicate `xtensa_route_i2s0_camera_pins` helper from `crates/klipper-cam-fw/src/main.rs`.
2. **Ping-Pong PSRAM Double-Buffer & Non-Blocking `I2S0` DMA Capture**:
   - Implement a `#![no_std]` `#![forbid(unsafe_code)]` `PingPongFrameBuffer<'a>` (or `DoubleFrameBuffer<'a>`) managing two independent `128 KB` PSRAM JPEG frame slots (`slot_a` and `slot_b`) with atomic/tracked active read slot vs. back-buffer write slot semantics.
   - Implement a non-blocking `I2S0` frame capture state machine (`WaitVsyncHigh` -> `WaitVsyncLowStartDma` -> `DrainDmaEof`) so `VSYNC` synchronization never blocks `smoltcp` network polling and active `I2S0` DMA `IN_SUC_EOF` half-buffer draining is interleaved with `iface.poll()`.
   - Run background capture continuously while Port 81 `/stream` is active and at a periodic idle refresh cadence (`250 ms` / `4 Hz`) when Port 81 is idle, ensuring `GET /snapshot` on Port 80 always serves a validated `1280x720` JFIF frame from the active read slot in `< 15 ms` with zero camera bus contention.
3. **4-Socket `smoltcp` Listener Pool on Port 80**:
   - Replace the single Port 80 `TcpSocket` with a 4-worker pool (`PORT80_SOCKET_COUNT = 4`) in `SocketSet` (total 7 sockets: 1 DHCPv4 + 4 Port 80 + 2 Port 81).
   - Provide each Port 80 worker with its own dedicated PSRAM RX buffer (`8 KB`), TX buffer (`64 KB`), request header buffer (`4 KB`), and response assembly buffer (`128 KB`) so 4 concurrent `/snapshot` or `/status` requests can be received, rendered, and transmitted simultaneously without sharing response slices or stalling on TCP `FIN` teardown.
   - Share the `1,800 KB` PSRAM OTA payload accumulator on-demand when a worker receives `POST /update`.

### Non-Goals
- Changing the HTTP endpoint contract (`/`, `/status`, `/snapshot`, `/update` on Port 80; `/stream` on Port 81).
- Modifying sensor register tables in `crates/esp32-cam-sensors`.

### Anti-Goals
- Introducing `unsafe` code into `crates/klipper-cam-fw/src/lib.rs`, `src/http.rs`, or `src/ota.rs` (all library modules in `klipper-cam-fw` must retain `#![forbid(unsafe_code)]`).
- Exhausting internal DRAM heap (`free_heap` in `/status` must remain `>= 48,000` bytes).

---

## 3. Detailed Architecture

### 3.1 `esp32-cam-ll` Hardware MMIO Unification (`crates/esp32-cam-ll/src/esp32_i2s.rs`)
- Update `pub const I2S0I_WS_IN_IDX: u8 = 28;`.
- Add:
  ```rust
  pub const I2S_PD_CONF_REG_OFFSET: usize = 0xA4;
  pub const I2S_PD_FIFO_MEM_FORCE_PU: u32 = 0x0A;
  ```
- In `Esp32I2sCamera::config()`, write `I2S_PD_FIFO_MEM_FORCE_PU` (`0x0A`) to `I2S0_BASE_ADDR + I2S_PD_CONF_REG_OFFSET` (`0x3FF4_F0A4`) and record `pd_conf: u32` in `I2s0RegisterBlock`.
- In `xtensa_configure_input_pad(gpio_num: u8, enable_pullup: bool)`:
  - For `36..=39`, clear `SENSE1..4_MUX_SEL` (`bits 27..=30`) in `RTC_IO_SENSOR_PADS_REG` (`0x3FF4_847C`).
- Add `Esp32I2sCamera::poll_eof_and_Rearm` / non-blocking DMA capture step helpers so callers can interleave `smoltcp` network polling during `VSYNC` wait and `I2S0` DMA readout.

### 3.2 Safe Ping-Pong Double Buffer (`crates/klipper-cam-fw/src/lib.rs`)
- Add `#![forbid(unsafe_code)]` `PingPongFrameBuffer<'a>` holding:
  - `slot_a: &'a mut [u8]` (`128 KB`)
  - `slot_b: &'a mut [u8]` (`128 KB`)
  - `len_a: usize`
  - `len_b: usize`
  - `active_is_b: bool`
  - `frame_seq: u64`
- Operations:
  - `new(slot_a: &'a mut [u8], slot_b: &'a mut [u8], initial_frame: &[u8]) -> Result<Self, CameraError>`: initializes both slots with `initial_frame` (`len_a = len_b = initial_frame.len()`).
  - `active_frame(&self) -> &[u8]`: returns `&self.slot_b[..self.len_b]` if `active_is_b` else `&self.slot_a[..self.len_a]`.
  - `inactive_slot_and_active_frame(&mut self) -> (&mut [u8], &[u8])`: uses disjoint struct field borrowing (`&mut self.slot_a` + `&self.slot_b[..self.len_b]` or vice versa) to allow writing a new frame into the back buffer while simultaneously reading the active front buffer with zero `unsafe` code.
  - `commit_inactive(&mut self, new_len: usize) -> Result<u64, CameraError>`: validates `new_len > 0 && new_len <= inactive_cap`, updates the inactive slot length, flips `active_is_b = !self.active_is_b`, increments `frame_seq`, and returns the new `frame_seq`.

### 3.3 Non-Blocking `I2S0` Capture Pipeline & Interleaved `smoltcp` Polling (`crates/klipper-cam-fw/src/main.rs`)
- Replace blocking `VSYNC` wait loops with a non-blocking state machine:
  1. **`Idle { last_capture_ms }`**:
     - Triggers a new frame capture immediately if `p81_streaming` is `true` (`now_ms - last_capture_ms >= 20`) or every `250 ms` when idle (`now_ms - last_capture_ms >= 250`).
  2. **`WaitVsyncHigh { start_ms }`**:
     - Checks `read_vsync()` on each main-loop iteration (`< 1 µs`). As soon as `read_vsync()` is `true` (or `now_ms - start_ms >= 45`), transitions to `WaitVsyncFallingEdge`.Meanwhile, `iface.poll()` and all 6 TCP sockets run without delay.
  3. **`WaitVsyncFallingEdge { start_ms }`**:
     - Checks `!read_vsync()` on each main-loop iteration. As soon as `VSYNC` falls (start of active frame readout), arms the circular `Lldesc` ring (`i2s_cam.start(...)`) and transitions to `ActiveDmaReadout`.
  4. **`ActiveDmaReadout`**:
     - Drains `I2S_IN_SUC_EOF_BIT` half-buffers into `jpeg_scratch_buf` (`1024` unpacked bytes per half-buffer) until `VSYNC` ends or `EOI` is reached (`max 85 ms`), calling `iface.poll()` between `EOF` half-buffer events so network RX/TX never stalls for more than `~1.2 ms`.
     - Upon completion, validates `SOI` (`0xFFD8`) and `EOI` (`0xFFD9`), copies the exact `SOI..=EOI` slice into `ping_pong`'s inactive back buffer, and calls `ping_pong.commit_inactive(exact_len)`.

### 3.4 4-Socket Port 80 Listener Pool (`crates/klipper-cam-fw/src/main.rs`)
- Allocate `SocketStorage` for `7` sockets:
  - `1` DHCPv4 socket (`dhcp_handle`)
  - `4` Port 80 HTTP sockets (`tcp80_handles: [SocketHandle; 4]`)
  - `1` Port 81 primary MJPEG stream socket (`tcp81_handle`)
  - `1` Port 81 busy-rejection socket (`tcp81_busy_handle`)
- PSRAM Carve Map (`3,406 KB` carved / `690 KB` free in `esp_alloc::HEAP`):
  - `ping_pong_slot_a`: `128 KB`
  - `ping_pong_slot_b`: `128 KB`
  - `jpeg_scratch_buf`: `128 KB`
  - Per Port 80 Worker (`x 4` = `1,152 KB`):
    - `rx_storage`: `8 KB` (`32 KB` total)
    - `tx_storage`: `64 KB` (`256 KB` total)
    - `req_hdr_buf`: `4 KB` (`16 KB` total)
    - `resp_buf`: `128 KB` (`512 KB` total)
  - Port 81 Primary (`292 KB`):
    - `rx_storage`: `4 KB`
    - `tx_storage`: `160 KB`
    - `resp_buf`: `128 KB`
  - Port 81 Busy (`4 KB`):
    - `rx_storage`: `2 KB`, `tx_storage`: `2 KB`
  - Shared `ota_req_accum` (`1,800 KB`):
    - Claimed exclusively by whichever Port 80 worker receives a request with `Content-Length > 4096` (`POST /update`).

---

## 4. Verification Criteria
1. `cargo test --workspace --offline`: 100% pass (`exit code 0`), including new unit tests for `I2S0I_WS_IN_IDX == 28`, `I2S_PD_CONF_REG_OFFSET`, and `PingPongFrameBuffer`.
2. `cargo clippy --workspace --all-targets --offline -- -D warnings`: `0` warnings (`exit code 0`).
3. `cargo +nightly miri test -p esp32-cam-ll -p esp32-cam-conversions -p esp32-cam-types --offline`: 100% pass, `0` UB (`exit code 0`).
4. `cargo +esp clippy --target xtensa-esp32-none-elf -Zbuild-std=core,alloc --offline --workspace --release -- -D warnings`: `0` warnings (`exit code 0`).
5. Live Hardware Flash & Challenger Verification on `172.22.1.52`:
   - `20` rapid back-to-back `GET /status` and `GET /snapshot` requests complete with `100%` `HTTP 200 OK` (`0` connection resets or timeouts).
   - `4` concurrent parallel `GET /snapshot` requests complete with `100%` `HTTP 200 OK` valid `1280x720` JFIF images.
   - Concurrent `GET http://172.22.1.52:81/stream` + `GET http://172.22.1.52/snapshot` + `GET http://172.22.1.52/status` succeed simultaneously with improved stream frame rate and `< 50 ms` snapshot response latency.
