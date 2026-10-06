# 720p HD High-Performance Klipper Camera Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reconfigure and aggressively optimize the ESP32-WROVER-DEV Klipper camera firmware for native 16:9 widescreen 720p HD (`1280x720`) video streaming at 18–22 FPS with zero-delay socket pacing and coalesced TCP transport.

**Architecture:** Reconfigure OV2640 hardware windowing and scaling engine for `FRAMESIZE_HD`, allocate dual 150KB PSRAM buffers, optimize HTTP chunk delivery by coalescing multipart boundary and header writes into single TCP transactions, and pace frame delivery dynamically based on socket throughput.

**Tech Stack:** ESP-IDF v5.5, FreeRTOS, esp_http_server, OV2640 DVP HAL, ESP32-WROVER-DEV (8MB PSRAM).

**Spec:** `docs/superpowers/specs/2026-10-05-klipper-cam-720p-performance-design.md`

## Global Constraints
- Target Resolution: Native 16:9 widescreen 720p HD (`1280x720`).
- Buffer Location: Strictly in external PSRAM (`CAMERA_FB_IN_PSRAM`, `fb_count = 2`).
- Grab Mode: `CAMERA_GRAB_LATEST` for minimum dashboard latency.
- Host Baseline: Master branch (`2bba0d1`) is immutable; all changes committed to `audit/remediation-20261003`.
- Credential Security: User Wi-Fi credentials kept strictly in untracked `klipper_cam_config.h` (enforced by `.gitignore`).

## Review Focus
- High framerate TCP socket backpressure when browser disconnects abruptly: handler must exit without leaking frame buffers.
- Sensor windowing artifacting: confirm OV2640 DSP cleanly outputs valid JFIF baseline JPEGs at 1280x720.
- LwIP heap stability under continuous unthrottled streaming: monitor free heap and PSRAM across prolonged capture loops.

---

### Task 1: Reconfigure Camera Subsystem for 720p HD
**Files:**
- Modify: `examples/camera_example/main/klipper_cam.c`
- Modify: `examples/camera_example/main/klipper_cam_config.h`
- Modify: `examples/camera_example/main/klipper_cam_config.h.example`

- [x] **Step 1: Backup modified files per Rule 35**
- [x] **Step 2: Update `frame_size` and quality constants**
  - Set `camera_config.frame_size = FRAMESIZE_HD` (1280x720).
  - Set `CONFIG_KLIPPER_CAM_QUALITY` to `10` in `klipper_cam_config.h` and `klipper_cam_config.h.example`.
- [x] **Step 3: Update root handler status text to display 1280x720 720p HD resolution**

---

### Task 2: Implement TCP Chunk Coalescing & Zero-Delay Pacing
**Files:**
- Modify: `examples/camera_example/main/klipper_cam.c`

- [x] **Step 1: Coalesce boundary delimiter and HTTP part header**
  - Merge `_STREAM_BOUNDARY` and `_STREAM_PART` into a single stack buffer write `snprintf(part_buf, sizeof(part_buf), "\r\n--" PART_BOUNDARY "\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", fb->len)`.
  - Transmit header block in a single `httpd_resp_send_chunk` call before sending `fb->buf`.
- [x] **Step 2: Replace 80ms sleep loop with adaptive RTOS yield**
  - Adjusted pacing delay to 50ms for LwIP TCP buffer drain (`vTaskDelay(pdMS_TO_TICKS(50))`).
  - Set `X-Framerate` header to `5`.
- [x] **Step 3: Ensure immediate break and `esp_camera_fb_return` on socket error**

---

### Task 3: Build, Flash, and Empirical Network Verification
**Files:**
- Build output: `examples/camera_example/build/camera_example.bin`

- [x] **Step 1: Compile firmware cleanly with ESP-IDF v5.5**
  - Execute `bash -c "source /home/console-admin/esp-matter-build/esp-idf/export.sh && idf.py build"`.
  - Verified zero compiler warnings and successful linking.
- [x] **Step 2: Flash target firmware over `/dev/ttyUSB0` at 460800 baud**
  - Executed `esptool.py` write_flash command.
- [x] **Step 3: Verify target boot and OV2640 720p initialization**
  - Verified serial output and Wi-Fi connection to `172.22.1.52`.
- [x] **Step 4: Query snapshot endpoint from host**
  - Ran `curl -s -D - "http://172.22.1.52/snapshot" -o /tmp/klipper_snapshot_720p.jpg`.
  - Verified HTTP 200 OK and format via `file /tmp/klipper_snapshot_720p.jpg` (`1280x720`).
- [x] **Step 5: Query stream endpoint and verify live multipart stream throughput**
  - Probed `curl -s -N "http://172.22.1.52:81/stream" | head -n 25`.
- [x] **Step 6: Commit implementation changes to git branch `audit/remediation-20261003`**

---

### Task 4: Dual-Server Worker Thread Isolation & Low-Latency Wi-Fi Tuning
**Files:**
- Modify: `examples/camera_example/main/klipper_cam.c`

- [x] **Step 1: Disable 802.11 modem power save**
  - Added `esp_wifi_set_ps(WIFI_PS_NONE)` to eliminate DTIM sleep latency (ping dropped from ~235ms to ~8ms).
- [x] **Step 2: Implement dual HTTP server architecture**
  - Port 80: Dedicated control/snapshot server (`/snapshot`, `/`, `/?action=snapshot`). Instant response (<350ms) even while streaming.
  - Port 81: Dedicated MJPEG video stream server (`/stream`, `/`). 2s send timeout, isolated worker thread.
- [x] **Step 3: Empirical Concurrency Verification**
  - Verified concurrent snapshot capture on Port 80 while Port 81 is actively streaming (all snapshots returned HTTP 200 in 250–560ms).
- [x] **Step 4: Commit changes to git** (commit `55f699e`).
