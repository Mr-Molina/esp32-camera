# Specification: 720p HD High-Performance Klipper Camera (ESP32-WROVER-DEV)

- **Date:** 2026-10-05
- **Status:** Approved
- **Target Hardware:** ESP32-WROVER-DEV (ESP32-D0WDQ6-V3, 8MB PSRAM, OV2640)
- **Target Network:** Wi-Fi Station (SSID: `iot`, Subnet: `172.22.1.0/24`)

---

## 1. Objective & Scope

Optimize the dedicated Klipper 3D printer network camera firmware for native 16:9 widescreen 720p HD (`1280x720`) video streaming, maximizing framerate (18–22 FPS) and minimizing transport latency for Mainsail/Fluidd web dashboards and Moonraker timelapses, with zero CPU load on the Klipper host.

---

## 2. Technical Architecture & Specifications

### 2.1 Sensor & Resolution Configuration
* **Sensor Model**: OmniVision OV2640 (DVP parallel bus on `BOARD_WROVER_KIT` pinout).
* **Target Resolution**: `FRAMESIZE_HD` (1280 × 720, 16:9 widescreen windowing).
* **JPEG Quality**: `jpeg_quality = 10` (high visual fidelity for nozzle, layer extrusion, and first-layer inspection).
* **Frame Buffers**: Dual 150,000-byte frame buffers allocated in 8MB external PSRAM.
* **Grab Mode**: `CAMERA_GRAB_LATEST` (ensures live stream fetches the freshest completed frame from DMA, eliminating buffer queue lag).

### 2.2 Streaming Pipeline & Transport Tuning
* **Elimination of Static Delay**: Remove the fixed 80ms sleep loop. Pacing will be driven by client TCP socket drainage with a 1-tick yield (`vTaskDelay(1)`) to avoid RTOS starvation.
* **HTTP Chunk Coalescing**:
  - Pre-format the multipart boundary delimiter and HTTP headers (`Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n`) into a single contiguous stack buffer chunk.
  - Reduce TCP socket transactions from 3 to 2 per frame, cutting network syscall overhead by ~33%.
* **Backpressure & Disconnect Detection**:
  - Immediately trap `ESP_FAIL` / `ECONNRESET` on `httpd_resp_send_chunk` and exit cleanly, freeing frame buffers and sockets without heap or task leaks.

### 2.3 Endpoints
* `GET /stream` (and `/?action=stream`): Live multipart MJPEG stream (`multipart/x-mixed-replace`) @ 18–22 FPS.
* `GET /snapshot` (and `/?action=snapshot`): Discrete 1280×720 JPEG snapshot for Moonraker `[timelapse]`.
* `GET /`: Plain-text configuration and status page.

---

## 3. Verification Protocol

1. **Compilation Verification**: Clean build with zero warnings under ESP-IDF v5.5.
2. **Target Flashing**: Program target over `/dev/ttyUSB0` at 460800 baud.
3. **Endpoint Probing**:
   - `curl -s -D - "http://172.22.1.52/snapshot" -o /tmp/klipper_snapshot_720p.jpg`
   - Validate HTTP 200 and image format via `file`: `JPEG image data, 1280x720`.
   - Validate `/stream` continuous multipart chunks and monitor framerate.
   - Host ping latency check.
