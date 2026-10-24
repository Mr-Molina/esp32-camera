# `esp32-camera` — Bare-Metal `#![no_std]` Rust Camera Driver & Klipper HD Streaming Firmware

A 100% pure `#![no_std]` bare-metal Rust camera driver workspace and production Klipper 3D printer camera firmware (`klipper-cam-fw`) for the Espressif **ESP32** (`ESP32-WROVER-DEV` / `ESP32-CAM` / `ESP32-S3`).

Zero C/C++ files or `esp-idf` C runtime dependencies remain in the workspace. Six of the seven crates enforce `#![forbid(unsafe_code)]` at the crate root, while all hardware memory-mapped I/O (`I2S0` parallel DVP DMA, `GPIO` matrix, `RTC_IO`, `IO_MUX`) is isolated and Miri-verified inside `esp32-cam-ll`.

---

## Workspace Architecture (7 `#![no_std]` Crates)

| Crate | Path | Safety Policy | Responsibility |
| :--- | :--- | :--- | :--- |
| **`esp32-cam-types`** | `crates/esp32-cam-types` | `#![forbid(unsafe_code)]` | Core zero-dependency domain types: `PixelFormat`, `FrameSize` (`96x96` through `2560x1920`), `CameraSensorId`, `SensorPid`, `SensorInfo`, `CameraStatus`, `CameraGrabMode`, `CameraFbLocation`, and `CamError`. |
| **`esp32-cam-sccb`** | `crates/esp32-cam-sccb` | `#![forbid(unsafe_code)]` | `SccbBus` trait and `EmbeddedHalSccb<I2C>` adapter over `embedded_hal::i2c::I2c` supporting 8-bit and 16-bit register addressing, bank switching, bitmask updates, and register table batch programming. |
| **`esp32-cam-sensors`** | `crates/esp32-cam-sensors` | `#![forbid(unsafe_code)]` | Complete `#![no_std]` driver implementations for **all 18 camera sensors** implementing the `CameraSensor` trait, plus `detect_and_init()` auto-probing. |
| **`esp32-cam-conversions`** | `crates/esp32-cam-conversions` | `#![forbid(unsafe_code)]` | Safe image codecs & frame validators: `validate_jpeg_frame` (`SOI`/`EOI`/`SOF0` dimension parser), `rgb565`/`yuv422`/`grayscale` to `rgb888`, `frame_to_bmp`, and baseline `encode_rgb888_to_jpeg`. |
| **`esp32-cam-ll`** | `crates/esp32-cam-ll` | `#![deny(unsafe_op_in_unsafe_fn)]` + Miri | ESP32 `I2S0` (`0x3FF4_F000`) parallel camera DMA engine, 16-byte aligned `Lldesc` descriptor ring, `GPIO` matrix (`I2S0I_DATA_IN0..7`, `V_SYNC`, `H_ENABLE`, `WS_IN`), `RTC_IO` analog mux disable (`GPIO 36`/`39`), and `XclkConfig` (`LEDC` 20 MHz). |
| **`esp32-cam-hal`** | `crates/esp32-cam-hal` | `#![forbid(unsafe_code)]` | Type-state `CameraDriver<State, B>` (`Uninitialized` -> `Configured` -> `Streaming`), RAII `FrameLease`, bounded `FramePool` in PSRAM, and pre-defined board pinouts (`WROVER-KIT`, `AI-Thinker`, `ESP32-S3`). |
| **`klipper-cam-fw`** | `crates/klipper-cam-fw` | `#![forbid(unsafe_code)]` | Production bare-metal Xtensa firmware (`esp-hal` + `esp-radio` + `smoltcp` + `esp-alloc` + `esp-storage`) featuring `PingPongFrameBuffer` in 80 MHz PSRAM, 4-socket Port 80 HTTP control/snapshot/OTA server, Port 81 MJPEG streamer, `/control` + `/panel` live `OV2640` tuning, and dual-slot `otadata` rollback manager. |

---

## Supported Camera Sensors (18 Drivers in `esp32-cam-sensors`)

| Sensor | Vendor | PID | Max Resolution | Formats | Special Capabilities |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **`OV2640`** | OmniVision | `0x26` | `1600 x 1200` (`UXGA`) | JPEG, YUV422, RGB565, Grayscale | Hardware JPEG encoder, 16:9 `1280x720` (`HD`) windowing, DSP/Sensor bank tuning |
| **`OV3660`** | OmniVision | `0x3660` | `2048 x 1536` (`QXGA`) | JPEG, YUV422, RGB565, Raw | 16-bit SCCB addressing, PLL clock divider control |
| **`OV3640`** | OmniVision | `0x364C` | `2048 x 1536` (`QXGA`) | JPEG, YUV422, RGB565, Raw | 16-bit SCCB addressing, hardware JPEG compression |
| **`OV5640`** | OmniVision | `0x5640` | `2592 x 1944` (`QSXGA`) | JPEG, YUV422, RGB565, Raw | 16-bit SCCB addressing, embedded MCU autofocus (`ov5640_af`) firmware loader |
| **`OV7670`** | OmniVision | `0x76` | `640 x 480` (`VGA`) | YUV422, RGB565, Raw Bayer | Classic VGA DVP timing & color matrix control |
| **`OV7725`** | OmniVision | `0x77` | `640 x 480` (`VGA`) | YUV422, RGB565, Raw RGB | High-sensitivity low-light VGA sensor |
| **`NT99141`** | Novatek | `0x1410` | `1280 x 720` (`HD`) | JPEG, YUV422, RGB565 | Native 720p HD JPEG output |
| **`GC2145`** | GalaxyCore | `0x2145` | `1600 x 1200` (`UXGA`) | YUV422, RGB565, Raw Bayer | Page-banked register control |
| **`GC0308`** | GalaxyCore | `0x9B` | `640 x 480` (`VGA`) | YUV422, RGB565, Grayscale | Low-power VGA sensor |
| **`GC032A`** | GalaxyCore | `0x232A` | `640 x 480` (`VGA`) | YUV422, RGB565, Raw Bayer | Sub-miniature 1/10" VGA sensor |
| **`BF3005`** | BYD | `0x30` | `640 x 480` (`VGA`) | YUV422, RGB565, Raw Bayer | Integrated auto-exposure / white-balance |
| **`BF20A6`** | BYD | `0x20A6` | `640 x 480` (`VGA`) | YUV422, Grayscale | Ultra-compact VGA sensor |
| **`SC101IOT`** | SmartSens | `0xDA4A` | `1280 x 720` (`HD`) | YUV422, Raw RGB | 720p IoT CMOS image sensor |
| **`SC030IOT`** | SmartSens | `0x9A46` | `640 x 480` (`VGA`) | YUV422, Raw Bayer | Page-switched VGA IoT sensor |
| **`SC031GS`** | SmartSens | `0x0031` | `640 x 480` (`VGA`) | Grayscale (`RAW MONO`) | Global-shutter monochrome sensor |
| **`MEGA_CCM`** | Arducam | `0x039E` | `2592 x 1944` (`5MP`) | JPEG, YUV422, RGB565 | 16-bit SCCB bridge camera module |
| **`HM1055`** | Himax | `0x1055` | `1280 x 720` (`HD`) | YUV422, RGB565, Raw | 720p color CMOS sensor |
| **`HM0360`** | Himax | `0x0360` | `640 x 480` (`VGA`) | Grayscale (`RAW MONO`) | Ultra-low-power wake-on-motion monochrome sensor |

---

## Hardware Architecture (`ESP32-WROVER-DEV` + `OV2640`)

### Pin Mapping (`WROVER_KIT_PINOUT`)

| Signal | ESP32 GPIO | Direction | Notes |
| :--- | :--- | :--- | :--- |
| **`XCLK`** | `GPIO 21` | Output (`LEDC_CH0`) | `20 MHz` master clock (`40 MHz` APB / 2) |
| **`SIOD` (`SDA`)** | `GPIO 26` | Bi-directional (`I2C0`) | `100 kHz` SCCB data |
| **`SIOC` (`SCL`)** | `GPIO 27` | Output (`I2C0`) | `100 kHz` SCCB clock |
| **`VSYNC`** | `GPIO 25` | Input (`I2S0I_V_SYNC`) | Frame vertical sync (`inverted` in `GPIO_FUNC_IN_SEL_CFG`) |
| **`HREF`** | `GPIO 23` | Input (`I2S0I_H_ENABLE`) | Line horizontal reference |
| **`PCLK`** | `GPIO 22` | Input (`I2S0I_WS_IN`) | Pixel clock (`signal index 28`) |
| **`D7` .. `D4`** | `GPIO 35, 34, 39, 36` | Input (`I2S0I_DATA_IN7..4`) | Input-only pads (`RTC_IO_SENSOR_PADS_REG` digital mux enabled) |
| **`D3` .. `D0`** | `GPIO 19, 18, 5, 4` | Input (`I2S0I_DATA_IN3..0`) | `GPIO 4` is strictly `D0` on `WROVER-KIT` (never driven as Flash LED) |
| **`PSRAM CS# / CLK`** | `GPIO 16, 17` | Internal (`80 MHz` QSPI) | `4 MB` external PSRAM (`VDD_SDIO = 3.3V`) |

### Non-Blocking `PingPongFrameBuffer` & 4-Socket `smoltcp` Pool

1. **Double-Buffered PSRAM Frame Pool (`PingPongFrameBuffer`)**:
   - Allocates two disjoint `128 KB` buffers (`slot_a`, `slot_b`) in external `80 MHz` PSRAM via `esp-alloc`.
   - While HTTP clients on Port 80 (`/snapshot`) and Port 81 (`/stream`) read the immutable **active** slot, the `I2S0` DMA engine captures the next `OV2640` JPEG frame into the **inactive** slot.
   - Once `validate_jpeg_frame` confirms valid `SOI (0xFFD8)` and `EOI (0xFFD9)` markers, `commit_inactive()` atomically swaps the active slot and increments `frame_seq`.
2. **Interleaved Non-Blocking DMA & TCP Polling (`DmaCapturePhase`)**:
   - `VSYNC` alignment (`WaitVsyncHigh` -> `WaitVsyncFalling`) runs as a non-blocking state machine inside the main network loop, polling `iface.poll()` every iteration so `/status` and `/control` queries experience zero VSYNC wait blocking.
   - During active `I2S0` pixel transfer (~30–40 ms per 720p frame), `iface.poll()` is interleaved every 16 DMA half-buffer drains so ARP, ICMP, and TCP ACKs are processed continuously.
3. **Dual-Port TCP Socket Pool**:
   - **Port 80 (`PORT80_SOCKET_COUNT = 4`)**: 4 independent `smoltcp` TCP server sockets handle concurrent `/status`, `/snapshot`, `/control`, `/panel`, and `/update` requests without head-of-line blocking.
   - **Port 81 (`1` Dedicated Stream Socket)**: Streams continuous `multipart/x-mixed-replace` MJPEG frames without starving Port 80 control or snapshot endpoints.

---

## HTTP API Reference (`klipper-cam-fw`)

### Port 80 — Control, Telemetry, Snapshot, & Wireless OTA

| Method & Path | Content-Type | Description |
| :--- | :--- | :--- |
| **`GET /`** | `text/html` | Landing page linking to `/status`, `/snapshot`, `:81/stream`, `/panel`, and `/update`. |
| **`GET /status`** | `application/json` | 7-key JSON health & memory telemetry: `{"uptime_ms":...,"free_heap":...,"free_psram":...,"wifi_rssi":...,"active_partition":"ota_0","framesize":"1280x720","stream_active":false}`. |
| **`GET /snapshot`** | `image/jpeg` | Latest validated `1280x720` JFIF frame from the active `PingPongFrameBuffer` slot (`Cache-Control: no-store`). |
| **`GET /control`** | `application/json` | Current `OV2640` camera parameters: `framesize`, `framesize_label`, `quality`, `brightness`, `contrast`, `saturation`, `special_effect`, `hmirror`, `vflip`, `awb`, `agc`, `aec`, `ae_level`. |
| **`GET /control?var=<k>&val=<v>`** | `application/json` | Live `OV2640` `I2C0` SCCB register update (`quality` `4..=63`, `brightness`/`contrast`/`saturation`/`ae_level` `-2..=2`, `special_effect` `0..=6`, `hmirror`/`vflip`/`awb`/`agc`/`aec` `0..=1`, `framesize` `5..=13` or width `320..=1600`). Returns `400 Bad Request` on invalid input. |
| **`GET /panel`** | `text/html` | Self-contained dark-mode interactive Camera Control & Live Preview dashboard. |
| **`GET /update`** | `text/html` | Interactive Wireless OTA firmware upload UI. |
| **`POST /update`** | `text/plain` | Authenticated wireless OTA firmware upload (`X-OTA-Token: <token>` header required; returns `401 Unauthorized` without wildcard CORS on mismatch). Stops `I2S0` DMA, validates the ESP-IDF `0xE9` image header, writes to the passive OTA slot (`ota_0` or `ota_1`), updates `otadata` (`0x17000`) with ESP-IDF CRC32, and triggers `software_reset()`. |

### Port 81 — Dedicated MJPEG Stream

| Method & Path | Content-Type | Description |
| :--- | :--- | :--- |
| **`GET :81/stream`** | `multipart/x-mixed-replace;boundary=123456789000000000000987654321` | Continuous MJPEG video stream for Klipper / Crowsnest / Moonraker / Mainsail / Fluidd. |

---

## Building, Testing, & Verification

### 1. Host Unit Tests, Clippy, & Miri Soundness Checks

All 7 crates compile and run their unit and integration test suites on the host (`x86_64-unknown-linux-gnu`) without hardware attached:

```bash
# Run all workspace unit & integration tests (92+ tests)
cargo test --workspace --offline

# Zero-warning Clippy lint gate across all host targets
cargo clippy --workspace --all-targets --offline -- -D warnings

# Run Miri undefined-behavior checker on low-level DMA, image conversions, and core types
cargo +nightly miri test -p esp32-cam-ll -p esp32-cam-conversions -p esp32-cam-types --offline
```

### 2. Configuring Local Wi-Fi Credentials (`.env.local`)

Secrets are never hardcoded or tracked in Git. Store your compile-time `WIFI_PASS` (and optional `WIFI_SSID` / `WIFI_OTA_TOKEN`) in an untracked `.env.local` file with `0600` permissions:

```bash
install -m 600 /dev/null /home/console-admin/Github/esp32-camera/.env.local
# Edit /home/console-admin/Github/esp32-camera/.env.local and add:
# WIFI_SSID=iot
# WIFI_PASS=<your-wpa2-passphrase>
# WIFI_OTA_TOKEN=klipper-cam-ota-2026
```

### 3. Building the Bare-Metal Xtensa Firmware (`+esp`)

```bash
set -a && source /home/console-admin/Github/esp32-camera/.env.local && set +a

# Lint the bare-metal Xtensa target with zero warnings
cargo +esp clippy --target xtensa-esp32-none-elf -Zbuild-std=core,alloc --offline --workspace --release -- -D warnings

# Build the release ELF binary for ESP32
cargo +esp build --target xtensa-esp32-none-elf -Zbuild-std=core,alloc --offline -p klipper-cam-fw --release
```

### 4. Dual-Slot Partition Table (`partitions_rust_ota.csv`) & Serial Flashing

The repository includes `partitions_rust_ota.csv`, which provisions two `1,728 KB` (`0x1B0000`) application slots (`ota_0 @ 0x20000` and `ota_1 @ 0x1D0000`) and the `8 KB` `otadata` selector partition (`0x17000`):

```csv
# Name,   Type, SubType, Offset,   Size,     Flags
nvs,      data, nvs,     0x11000,  0x6000,
otadata,  data, ota,     0x17000,  0x2000,
phy_init, data, phy,     0x19000,  0x1000,
ota_0,    app,  ota_0,   0x20000,  0x1B0000,
ota_1,    app,  ota_1,   0x1D0000, 0x1B0000,
```

To flash the initial dual-OTA layout over USB serial (`/dev/ttyUSB0`):

```bash
espflash flash \
  --port /dev/ttyUSB0 \
  --partition-table /home/console-admin/Github/esp32-camera/partitions_rust_ota.csv \
  --partition-table-offset 0x8000 \
  --target-app-partition ota_0 \
  --erase-data-parts ota \
  --non-interactive \
  /home/console-admin/Github/esp32-camera/target/xtensa-esp32-none-elf/release/klipper-cam-fw
```

### 5. Wireless Dual-Slot OTA Deployment (`scripts/ota_deploy.py`)

Once the dual-slot partition table is installed, subsequent firmware updates can be staged and deployed wirelessly over HTTP `POST /update` with automatic `ota_0 <-> ota_1` slot alternation and post-boot `/status` + `/snapshot` verification:

```bash
# Stage the .bin image and verify the 0xE9 header, segment count, size, and SHA-256 without touching the network:
/usr/bin/python3 /home/console-admin/Github/esp32-camera/scripts/ota_deploy.py --stage-only

# Execute a live wireless OTA deployment to 172.22.1.52 and verify slot transition + post-boot 1280x720 JFIF snapshot:
/usr/bin/python3 /home/console-admin/Github/esp32-camera/scripts/ota_deploy.py --host 172.22.1.52
```

---

## Klipper / Moonraker / Mainsail / Fluidd Configuration

### `moonraker.conf` / Mainsail / Fluidd Webcam Entry

Configure your webcam in Mainsail or Fluidd (`Settings -> Webcams`) or in `moonraker.conf`:

```ini
[webcam klipper_esp32_cam]
location: printer
service: mjpegstreamer-adaptive
target_fps: 5
stream_url: http://172.22.1.52:81/stream
snapshot_url: http://172.22.1.52/snapshot
flip_horizontal: False
flip_vertical: False
```

Live image orientation (`hmirror`, `vflip`), JPEG compression quality (`quality`), exposure (`ae_level`), and resolution (`framesize`) can be tuned on the fly from a browser at `http://172.22.1.52/panel` or via `curl "http://172.22.1.52/control?var=hmirror&val=1"`.

---

## License

Dual-licensed under `Apache-2.0 OR MIT`.
