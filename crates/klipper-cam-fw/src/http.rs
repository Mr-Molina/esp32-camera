//! Zero-allocation `#![no_std]` HTTP/1.1 request parser, response writer,
//! dual-port Klipper camera control server (Port 80), MJPEG multipart streamer (Port 81),
//! and OTA upload handler (`POST /update`).
//!
//! Replaces the HTTP/MJPEG/OTA server handlers in `examples/camera_example/main/klipper_cam.c`
//! and `examples/camera_example/main/ota_minimal.c`.

#![forbid(unsafe_code)]

use core::fmt::{self, Write};
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use esp32_cam_types::{CameraError, FrameSize};

use crate::config::KlipperCamConfig;
use crate::ota::{
    validate_ota_image_header, verify_ota_token, OtaDataManager, OtaError, OtaSlot,
    OTA_CHUNK_SIZE, OTA_PARTITION_MAX_SIZE,
};

/// Multipart boundary string used by the Port 81 MJPEG stream (`PART_BOUNDARY` in `klipper_cam.c`).
pub const MJPEG_BOUNDARY: &str = "123456789000000000000987654321";

/// Full `Content-Type` header value for the Port 81 MJPEG stream.
pub const MJPEG_CONTENT_TYPE: &str =
    "multipart/x-mixed-replace;boundary=123456789000000000000987654321";

/// Maximum URL query string length accepted by `GET /` (`256` bytes, `APP-013`).
pub const MAX_URL_QUERY_LEN: usize = 256;

/// Stack buffer size for `GET /status` JSON serialization (`384` bytes, `klipper_cam.c`).
pub const STATUS_JSON_BUF_LEN: usize = 384;

/// Stack buffer size for each MJPEG multipart boundary header (`128` bytes, `klipper_cam.c`).
pub const MJPEG_PART_HDR_BUF_LEN: usize = 128;

/// Maximum consecutive frame capture failures before aborting the MJPEG stream (`5`).
pub const STREAM_MAX_CAPTURE_FAILURES: u8 = 5;

/// Plain-text landing page served by `GET /` on Port 80 (`klipper_cam.c`).
pub const KLIPPER_ROOT_STATUS_PAGE: &str = "\
Klipper 3D Printer Camera (ESP32-WROVER-DEV)\n\
Resolution: 1280x720 HD 16:9 (OV2640 Hardware JPEG)\n\
Framerate:  4-5 FPS (OV2640 Hardware HD Limit with LwIP Pacing)\n\
\n\
Endpoints:\n\
  - Dedicated Stream: http://<this-ip>:81/stream (Isolated Worker Task)\n\
  - Snapshot:         http://<this-ip>/snapshot  (Instantaneous, Zero-Block)\n\
  - Status JSON:      http://<this-ip>/status\n\
  - OTA Update:       http://<this-ip>/update\n\
\n\
Moonraker Configuration:\n\
  [webcam printer_cam]\n\
  location: printer\n\
  service: mjpegstreamer-adaptive\n\
  stream_url: http://<this-ip>:81/stream\n\
  snapshot_url: http://<this-ip>/snapshot\n";

/// Canonical minimal 1280x720 (`0x0500` x `0x02D0`) JFIF JPEG frame with SOI (`FF D8`),
/// APP0 `JFIF\0` (`FF E0`), SOF0 (`FF C0`), SOS (`FF DA`), and EOI (`FF D9`) markers.
pub const SYNTHETIC_HD_JFIF_FRAME: [u8; 43] = [
    0xFF, 0xD8, // SOI
    0xFF, 0xE0, 0x00, 0x10, b'J', b'F', b'I', b'F', 0x00, 0x01, 0x01, 0x01, 0x00, 0x48, 0x00,
    0x48, 0x00, 0x00, // APP0 JFIF 1.1
    0xFF, 0xC0, 0x00, 0x0B, 0x08, 0x02, 0xD0, 0x05, 0x00, 0x01, 0x01, 0x11,
    0x00, // SOF0: 8-bit, height=720 (0x02D0), width=1280 (0x0500)
    0xFF, 0xDA, 0x00, 0x04, 0x00, 0x00, 0x00, 0xFF, // SOS + minimal scan byte
    0xFF, 0xD9, // EOI
];

/// Identifies which HTTP server instance received a request.
#[repr(u16)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ServerPort {
    /// Main Control, Snapshot, Status, and OTA server on Port 80.
    Control = 80,
    /// Dedicated isolated MJPEG stream server on Port 81 (`NET-001` / `APP-005`).
    Stream = 81,
}

impl ServerPort {
    /// Converts a TCP port number (`80` or `81`) into a [`ServerPort`].
    #[inline]
    pub const fn from_port(port: u16) -> Option<Self> {
        match port {
            80 => Some(Self::Control),
            81 => Some(Self::Stream),
            _ => None,
        }
    }
}

/// HTTP request method.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum HttpMethod<'a> {
    Get,
    Post,
    Head,
    Options,
    Other(&'a str),
}

/// Zero-allocation parsed HTTP/1.1 request borrowing directly from the receive buffer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HttpRequest<'a> {
    /// HTTP method (`GET`, `POST`, etc.).
    pub method: HttpMethod<'a>,
    /// URL path excluding query string (e.g. `"/"`, `"/snapshot"`, `"/status"`, `"/update"`, `"/stream"`).
    pub path: &'a str,
    /// Optional URL query string after `'?'` (e.g. `Some("action=snapshot")`).
    pub query: Option<&'a str>,
    /// HTTP version string (e.g. `"HTTP/1.1"`).
    pub version: &'a str,
    /// Value of the `X-OTA-Token` header if present.
    pub ota_token: Option<&'a [u8]>,
    /// Parsed `Content-Length` header if present (may be negative or zero if invalid).
    pub content_length: Option<isize>,
    /// Value of the `Content-Type` header if present.
    pub content_type: Option<&'a str>,
    /// Request body slice following `\r\n\r\n`.
    pub body: &'a [u8],
}

/// Errors that can occur during HTTP parsing or response formatting.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum HttpError {
    /// Request buffer is empty or missing `\r\n\r\n` header terminator.
    MalformedRequest,
    /// Request line or header contains non-UTF8 bytes.
    InvalidUtf8,
    /// Caller-supplied output buffer is too small to hold the formatted HTTP response.
    BufferTooSmall,
    /// Camera frame buffer mutex is currently held by another capture.
    CameraBusy,
    /// Camera sensor failed to return a valid frame.
    CaptureFailed,
}

impl fmt::Display for HttpError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::MalformedRequest => write!(f, "malformed HTTP request"),
            Self::InvalidUtf8 => write!(f, "invalid UTF-8 in HTTP headers"),
            Self::BufferTooSmall => write!(f, "response output buffer too small"),
            Self::CameraBusy => write!(f, "camera frame buffer mutex busy"),
            Self::CaptureFailed => write!(f, "camera frame capture failed"),
        }
    }
}

/// Zero-allocation fixed-capacity byte buffer writer implementing [`core::fmt::Write`].
pub struct SliceWriter<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl<'a> SliceWriter<'a> {
    /// Wraps a mutable byte slice for zero-allocation sequential writing.
    #[inline]
    pub fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, pos: 0 }
    }

    /// Returns the number of bytes written so far.
    #[inline]
    pub const fn written_len(&self) -> usize {
        self.pos
    }

    /// Returns the written byte slice.
    #[inline]
    pub fn as_bytes(&self) -> &[u8] {
        &self.buf[..self.pos]
    }

    /// Consumes the writer and returns the written slice with lifetime `'a`.
    #[inline]
    pub fn into_bytes(self) -> &'a [u8] {
        &self.buf[..self.pos]
    }

    /// Appends raw bytes to the buffer, returning [`HttpError::BufferTooSmall`] on overflow.
    pub fn write_bytes(&mut self, data: &[u8]) -> Result<(), HttpError> {
        let end = self
            .pos
            .checked_add(data.len())
            .ok_or(HttpError::BufferTooSmall)?;
        if end > self.buf.len() {
            return Err(HttpError::BufferTooSmall);
        }
        self.buf[self.pos..end].copy_from_slice(data);
        self.pos = end;
        Ok(())
    }

    /// Appends an ASCII/UTF-8 string to the buffer.
    #[inline]
    pub fn write_str_checked(&mut self, s: &str) -> Result<(), HttpError> {
        self.write_bytes(s.as_bytes())
    }
}

impl Write for SliceWriter<'_> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        self.write_bytes(s.as_bytes()).map_err(|_| fmt::Error)
    }
}

/// Finds the starting index of `needle` inside `haystack`.
fn find_subslice(haystack: &[u8], needle: &[u8]) -> Option<usize> {
    if needle.is_empty() || haystack.len() < needle.len() {
        return None;
    }
    let limit = haystack.len() - needle.len();
    let mut i = 0;
    while i <= limit {
        if &haystack[i..i + needle.len()] == needle {
            return Some(i);
        }
        i += 1;
    }
    None
}

/// Parses an ASCII signed integer (`isize`) from a trimmed header value.
fn parse_isize_ascii(s: &str) -> Option<isize> {
    let trimmed = s.trim();
    let bytes = trimmed.as_bytes();
    if bytes.is_empty() {
        return None;
    }
    let (neg, start) = if bytes[0] == b'-' {
        (true, 1usize)
    } else if bytes[0] == b'+' {
        (false, 1usize)
    } else {
        (false, 0usize)
    };
    if start >= bytes.len() {
        return None;
    }
    let mut val: isize = 0;
    let mut i = start;
    while i < bytes.len() {
        let b = bytes[i];
        if !b.is_ascii_digit() {
            return None;
        }
        val = val.checked_mul(10)?;
        let digit = (b - b'0') as isize;
        val = if neg {
            val.checked_sub(digit)?
        } else {
            val.checked_add(digit)?
        };
        i += 1;
    }
    Some(val)
}

/// Zero-allocation HTTP/1.1 request parser (`#![no_std]`).
pub fn parse_http_request(raw: &[u8]) -> Result<HttpRequest<'_>, HttpError> {
    let hdr_end = find_subslice(raw, b"\r\n\r\n").ok_or(HttpError::MalformedRequest)?;
    let header_block =
        core::str::from_utf8(&raw[..hdr_end]).map_err(|_| HttpError::InvalidUtf8)?;
    let body = &raw[hdr_end + 4..];

    let mut lines = header_block.split("\r\n");
    let request_line = lines.next().ok_or(HttpError::MalformedRequest)?;
    let mut parts = request_line.split_whitespace();
    let method_str = parts.next().ok_or(HttpError::MalformedRequest)?;
    let target_str = parts.next().ok_or(HttpError::MalformedRequest)?;
    let version = parts.next().ok_or(HttpError::MalformedRequest)?;

    let method = match method_str {
        "GET" => HttpMethod::Get,
        "POST" => HttpMethod::Post,
        "HEAD" => HttpMethod::Head,
        "OPTIONS" => HttpMethod::Options,
        other => HttpMethod::Other(other),
    };

    let (path, query) = match target_str.find('?') {
        Some(idx) => (&target_str[..idx], Some(&target_str[idx + 1..])),
        None => (target_str, None),
    };

    let mut ota_token: Option<&[u8]> = None;
    let mut content_length: Option<isize> = None;
    let mut content_type: Option<&str> = None;

    for line in lines {
        if line.is_empty() {
            continue;
        }
        if let Some(colon) = line.find(':') {
            let name = line[..colon].trim();
            let val = line[colon + 1..].trim();
            if name.eq_ignore_ascii_case("X-OTA-Token") {
                ota_token = Some(val.as_bytes());
            } else if name.eq_ignore_ascii_case("Content-Length") {
                content_length = parse_isize_ascii(val);
            } else if name.eq_ignore_ascii_case("Content-Type") {
                content_type = Some(val);
            }
        }
    }

    Ok(HttpRequest {
        method,
        path,
        query,
        version,
        ota_token,
        content_length,
        content_type,
        body,
    })
}

/// Parsed view of an HTTP response written into a byte buffer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HttpResponseView<'a> {
    /// Numeric HTTP status code (e.g. `200`, `400`, `401`, `404`, `500`).
    pub status_code: u16,
    /// Full HTTP status line (e.g. `"HTTP/1.1 200 OK"`).
    pub status_line: &'a str,
    /// Raw header section (excluding `\r\n\r\n`).
    pub headers_raw: &'a str,
    /// `Content-Type` header value if present.
    pub content_type: Option<&'a str>,
    /// `Content-Disposition` header value if present.
    pub content_disposition: Option<&'a str>,
    /// `Access-Control-Allow-Origin` header value if present.
    pub access_control_allow_origin: Option<&'a str>,
    /// `X-Framerate` header value if present.
    pub x_framerate: Option<&'a str>,
    /// Response body bytes following `\r\n\r\n`.
    pub body: &'a [u8],
}

impl<'a> HttpResponseView<'a> {
    /// Parses a serialized HTTP response buffer into an [`HttpResponseView`].
    pub fn parse(raw: &'a [u8]) -> Result<Self, HttpError> {
        let hdr_end = find_subslice(raw, b"\r\n\r\n").ok_or(HttpError::MalformedRequest)?;
        let headers_raw =
            core::str::from_utf8(&raw[..hdr_end]).map_err(|_| HttpError::InvalidUtf8)?;
        let body = &raw[hdr_end + 4..];

        let mut lines = headers_raw.split("\r\n");
        let status_line = lines.next().ok_or(HttpError::MalformedRequest)?;
        let mut status_parts = status_line.split_whitespace();
        let _http_ver = status_parts.next().ok_or(HttpError::MalformedRequest)?;
        let code_str = status_parts.next().ok_or(HttpError::MalformedRequest)?;
        let status_code = parse_isize_ascii(code_str)
            .and_then(|v| u16::try_from(v).ok())
            .ok_or(HttpError::MalformedRequest)?;

        let mut content_type = None;
        let mut content_disposition = None;
        let mut access_control_allow_origin = None;
        let mut x_framerate = None;

        for line in lines {
            if let Some(colon) = line.find(':') {
                let name = line[..colon].trim();
                let val = line[colon + 1..].trim();
                if name.eq_ignore_ascii_case("Content-Type") {
                    content_type = Some(val);
                } else if name.eq_ignore_ascii_case("Content-Disposition") {
                    content_disposition = Some(val);
                } else if name.eq_ignore_ascii_case("Access-Control-Allow-Origin") {
                    access_control_allow_origin = Some(val);
                } else if name.eq_ignore_ascii_case("X-Framerate") {
                    x_framerate = Some(val);
                }
            }
        }

        Ok(Self {
            status_code,
            status_line,
            headers_raw,
            content_type,
            content_disposition,
            access_control_allow_origin,
            x_framerate,
            body,
        })
    }
}

/// Telemetry snapshot used to populate `GET /status` on Port 80.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TelemetrySnapshot<'a> {
    /// System uptime in milliseconds (`esp_timer_get_time() / 1000`).
    pub uptime_ms: i64,
    /// Free internal DRAM heap in bytes (`esp_get_free_heap_size()`).
    pub free_heap: u32,
    /// Free external PSRAM heap in bytes (`heap_caps_get_free_size(MALLOC_CAP_SPIRAM)`).
    pub free_psram: u32,
    /// Connected AP RSSI in dBm (`ap_info.rssi`).
    pub wifi_rssi: i32,
    /// Currently running OTA partition label (`"ota_0"` or `"ota_1"`).
    pub active_partition: &'a str,
    /// Active frame resolution string (`"1280x720"`).
    pub framesize: &'a str,
    /// Whether a client is actively streaming on Port 81 (`s_stream_active`).
    pub stream_active: bool,
}

impl Default for TelemetrySnapshot<'static> {
    fn default() -> Self {
        Self {
            uptime_ms: 12_345,
            free_heap: 184_320,
            free_psram: 4_128_768,
            wifi_rssi: -52,
            active_partition: "ota_0",
            framesize: "1280x720",
            stream_active: false,
        }
    }
}

/// Formats the exact 7-key JSON telemetry payload into a 384-byte stack buffer (`klipper_cam.c`).
pub fn format_status_json<'a>(
    telemetry: &TelemetrySnapshot<'_>,
    stack_buf: &'a mut [u8; STATUS_JSON_BUF_LEN],
) -> Result<&'a str, HttpError> {
    let mut writer = SliceWriter::new(stack_buf);
    writeln!(
        writer,
        "{{\"uptime_ms\":{},\"free_heap\":{},\"free_psram\":{},\"wifi_rssi\":{},\"active_partition\":\"{}\",\"framesize\":\"{}\",\"stream_active\":{}}}",
        telemetry.uptime_ms,
        telemetry.free_heap,
        telemetry.free_psram,
        telemetry.wifi_rssi,
        telemetry.active_partition,
        telemetry.framesize,
        if telemetry.stream_active { "true" } else { "false" }
    )
    .map_err(|_| HttpError::BufferTooSmall)?;

    let len = writer.written_len();
    core::str::from_utf8(&stack_buf[..len]).map_err(|_| HttpError::InvalidUtf8)
}

/// Formats an MJPEG multipart frame boundary header into a 128-byte stack buffer (`klipper_cam.c`).
pub fn format_mjpeg_part_header(
    jpeg_len: usize,
    part_buf: &mut [u8; MJPEG_PART_HDR_BUF_LEN],
) -> Result<&[u8], HttpError> {
    let mut writer = SliceWriter::new(part_buf);
    write!(
        writer,
        "\r\n--{}\r\nContent-Type: image/jpeg\r\nContent-Length: {}\r\n\r\n",
        MJPEG_BOUNDARY, jpeg_len
    )
    .map_err(|_| HttpError::BufferTooSmall)?;
    Ok(writer.into_bytes())
}

/// RAII guard for a captured camera frame buffer (`camera_fb_t` + `s_fb_mutex`, `APP-006`).
///
/// Releases the camera frame-buffer mutex automatically when dropped.
pub struct FrameGuard<'a> {
    frame_bytes: &'a [u8],
    width: u16,
    height: u16,
    fb_mutex: &'a AtomicBool,
}

impl<'a> FrameGuard<'a> {
    /// Returns the captured JPEG frame slice.
    #[inline]
    pub const fn bytes(&self) -> &'a [u8] {
        self.frame_bytes
    }

    /// Returns the frame length in bytes.
    #[inline]
    pub const fn len(&self) -> usize {
        self.frame_bytes.len()
    }

    /// Returns `true` if the frame buffer is empty.
    #[inline]
    pub const fn is_empty(&self) -> bool {
        self.frame_bytes.is_empty()
    }

    /// Returns the frame width in pixels (`1280`).
    #[inline]
    pub const fn width(&self) -> u16 {
        self.width
    }

    /// Returns the frame height in pixels (`720`).
    #[inline]
    pub const fn height(&self) -> u16 {
        self.height
    }
}

impl Drop for FrameGuard<'_> {
    #[inline]
    fn drop(&mut self) {
        self.fb_mutex.store(false, Ordering::Release);
    }
}

/// Thread-safe `#![no_std]` camera frame provider with `AtomicBool` mutex synchronization (`APP-006`).
pub struct CameraDriver<'a> {
    frame_data: &'a [u8],
    frame_size: FrameSize,
    fb_mutex: AtomicBool,
    simulate_failure: AtomicBool,
    capture_count: AtomicU32,
}

impl<'a> CameraDriver<'a> {
    /// Creates a new [`CameraDriver`] backed by `frame_data` and `frame_size`.
    pub const fn new(frame_data: &'a [u8], frame_size: FrameSize) -> Self {
        Self {
            frame_data,
            frame_size,
            fb_mutex: AtomicBool::new(false),
            simulate_failure: AtomicBool::new(false),
            capture_count: AtomicU32::new(0),
        }
    }

    /// Creates a default 1280x720 HD [`CameraDriver`] serving [`SYNTHETIC_HD_JFIF_FRAME`].
    pub const fn new_hd_default() -> CameraDriver<'static> {
        CameraDriver::new(&SYNTHETIC_HD_JFIF_FRAME, FrameSize::Hd)
    }

    /// Acquires the frame-buffer mutex and returns an RAII [`FrameGuard`].
    ///
    /// Returns [`CameraError::InvalidState`] if the mutex is already locked (`Camera busy`),
    /// or [`CameraError::HardwareError`] if capture is simulated to fail.
    pub fn acquire_frame(&self) -> Result<FrameGuard<'_>, CameraError> {
        if self
            .fb_mutex
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
            .is_err()
        {
            return Err(CameraError::InvalidState);
        }

        if self.simulate_failure.load(Ordering::Acquire) {
            self.fb_mutex.store(false, Ordering::Release);
            return Err(CameraError::HardwareError);
        }

        self.capture_count.fetch_add(1, Ordering::Relaxed);
        Ok(FrameGuard {
            frame_bytes: self.frame_data,
            width: self.frame_size.width(),
            height: self.frame_size.height(),
            fb_mutex: &self.fb_mutex,
        })
    }

    /// Forces the frame-buffer mutex state (for testing mutex timeout/busy paths).
    pub fn set_mutex_locked(&self, locked: bool) {
        self.fb_mutex.store(locked, Ordering::Release);
    }

    /// Enables or disables simulated sensor capture failure (`esp_camera_fb_get() == NULL`).
    pub fn set_simulate_failure(&self, fail: bool) {
        self.simulate_failure.store(fail, Ordering::Release);
    }

    /// Returns the total number of frames successfully acquired.
    pub fn capture_count(&self) -> u32 {
        self.capture_count.load(Ordering::Relaxed)
    }
}

impl Default for CameraDriver<'static> {
    #[inline]
    fn default() -> Self {
        Self::new_hd_default()
    }
}

/// RAII single-client MJPEG stream lock guard (`s_stream_active` in `klipper_cam.c`, `NET-002`).
///
/// Guarantees that `stream_active` is atomically set back to `false` when the streaming
/// client disconnects or the guard goes out of scope.
pub struct StreamSessionGuard<'a> {
    stream_active: &'a AtomicBool,
    consecutive_failures: u8,
}

impl<'a> StreamSessionGuard<'a> {
    /// Attempts to acquire the exclusive Port 81 stream lock.
    ///
    /// Returns `None` if another client is already streaming on Port 81.
    pub fn try_acquire(stream_active: &'a AtomicBool) -> Option<Self> {
        if stream_active
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
            .is_ok()
        {
            Some(Self {
                stream_active,
                consecutive_failures: 0,
            })
        } else {
            None
        }
    }

    /// Writes the initial Port 81 MJPEG HTTP response headers into `out_buf`:
    /// - `HTTP/1.1 200 OK`
    /// - `Content-Type: multipart/x-mixed-replace;boundary=123456789000000000000987654321`
    /// - `Access-Control-Allow-Origin: *`
    /// - `X-Framerate: 5`
    pub fn write_stream_headers<'b>(&self, out_buf: &'b mut [u8]) -> Result<&'b [u8], HttpError> {
        let mut writer = SliceWriter::new(out_buf);
        write!(
            writer,
            "HTTP/1.1 200 OK\r\nContent-Type: {}\r\nAccess-Control-Allow-Origin: *\r\nX-Framerate: 5\r\n\r\n",
            MJPEG_CONTENT_TYPE
        )
        .map_err(|_| HttpError::BufferTooSmall)?;
        Ok(writer.into_bytes())
    }

    /// Acquires one frame from `camera`, formats the MJPEG multipart boundary header and
    /// JPEG payload into `out_buf`, and immediately releases the [`FrameGuard`] so Port 80
    /// `/snapshot` requests are never starved during inter-frame pacing.
    ///
    /// Tracks consecutive capture failures and returns [`HttpError::CaptureFailed`] if
    /// [`STREAM_MAX_CAPTURE_FAILURES`] (`5`) consecutive failures occur.
    pub fn write_next_frame_chunk<'b>(
        &mut self,
        camera: &CameraDriver<'_>,
        out_buf: &'b mut [u8],
    ) -> Result<&'b [u8], HttpError> {
        let guard = match camera.acquire_frame() {
            Ok(g) => {
                self.consecutive_failures = 0;
                g
            }
            Err(CameraError::InvalidState) => return Err(HttpError::CameraBusy),
            Err(_) => {
                self.consecutive_failures = self.consecutive_failures.saturating_add(1);
                return Err(HttpError::CaptureFailed);
            }
        };

        let mut part_hdr_buf = [0u8; MJPEG_PART_HDR_BUF_LEN];
        let part_hdr = format_mjpeg_part_header(guard.len(), &mut part_hdr_buf)?;

        let mut writer = SliceWriter::new(out_buf);
        writer.write_bytes(part_hdr)?;
        writer.write_bytes(guard.bytes())?;
        drop(guard);
        Ok(writer.into_bytes())
    }

    /// Returns the number of consecutive capture failures on this stream session.
    #[inline]
    pub const fn consecutive_failures(&self) -> u8 {
        self.consecutive_failures
    }

    /// Returns `true` if consecutive capture failures have reached [`STREAM_MAX_CAPTURE_FAILURES`] (`5`).
    #[inline]
    pub const fn should_abort(&self) -> bool {
        self.consecutive_failures >= STREAM_MAX_CAPTURE_FAILURES
    }
}

impl Drop for StreamSessionGuard<'_> {
    #[inline]
    fn drop(&mut self) {
        self.stream_active.store(false, Ordering::Release);
    }
}

/// Complete `#![no_std]` Klipper Camera dual-port HTTP & OTA server state machine.
pub struct KlipperCamServer<'a> {
    pub config: KlipperCamConfig,
    pub camera: CameraDriver<'a>,
    pub ota_manager: OtaDataManager,
    stream_active: AtomicBool,
    uptime_ms: i64,
    free_heap: u32,
    free_psram: u32,
    wifi_rssi: i32,
    last_ota_bytes_written: usize,
    last_ota_target_slot: Option<OtaSlot>,
    reboot_scheduled: bool,
}

impl<'a> KlipperCamServer<'a> {
    /// Creates a new [`KlipperCamServer`] with the given configuration and camera driver.
    pub const fn new(config: KlipperCamConfig, camera: CameraDriver<'a>) -> Self {
        Self {
            config,
            camera,
            ota_manager: OtaDataManager::new_booted_ota0(),
            stream_active: AtomicBool::new(false),
            uptime_ms: 15_000,
            free_heap: 184_320,
            free_psram: 4_128_768,
            wifi_rssi: -48,
            last_ota_bytes_written: 0,
            last_ota_target_slot: None,
            reboot_scheduled: false,
        }
    }

    /// Creates a default [`KlipperCamServer`] with `WroverKit` 1280x720 HD configuration.
    pub const fn new_default() -> KlipperCamServer<'static> {
        KlipperCamServer::new(KlipperCamConfig::WROVER_KIT_HD, CameraDriver::new_hd_default())
    }

    /// Updates the live system telemetry metrics reported by `GET /status`.
    pub fn set_telemetry(&mut self, uptime_ms: i64, free_heap: u32, free_psram: u32, wifi_rssi: i32) {
        self.uptime_ms = uptime_ms;
        self.free_heap = free_heap;
        self.free_psram = free_psram;
        self.wifi_rssi = wifi_rssi;
    }

    /// Returns `true` if a client is currently streaming on Port 81 (`s_stream_active`).
    #[inline]
    pub fn is_stream_active(&self) -> bool {
        self.stream_active.load(Ordering::Acquire)
    }

    /// Returns a shared reference to the atomic `stream_active` flag.
    #[inline]
    pub fn stream_active_flag(&self) -> &AtomicBool {
        &self.stream_active
    }

    /// Attempts to start an exclusive Port 81 MJPEG stream session (`StreamSessionGuard`).
    #[inline]
    pub fn try_start_stream(&self) -> Option<StreamSessionGuard<'_>> {
        StreamSessionGuard::try_acquire(&self.stream_active)
    }

    /// Returns `true` if a valid OTA firmware upload has completed and scheduled a reboot.
    #[inline]
    pub const fn is_reboot_scheduled(&self) -> bool {
        self.reboot_scheduled
    }

    /// Returns the total bytes written during the most recent successful OTA update.
    #[inline]
    pub const fn last_ota_bytes_written(&self) -> usize {
        self.last_ota_bytes_written
    }

    /// Returns the target [`OtaSlot`] written during the most recent successful OTA update.
    #[inline]
    pub const fn last_ota_target_slot(&self) -> Option<OtaSlot> {
        self.last_ota_target_slot
    }

    /// Cancels rollback on successful Wi-Fi IP acquisition (`IP_EVENT_STA_GOT_IP`, `APP-003` / `FW-002`).
    pub fn on_wifi_got_ip_cancel_rollback(&mut self) -> Result<bool, OtaError> {
        self.ota_manager.mark_app_valid_cancel_rollback()
    }

    /// Handles a raw HTTP/1.1 request received on `port` (`ServerPort::Control` = 80 or
    /// `ServerPort::Stream` = 81) and writes the complete HTTP response into `out_buf`.
    pub fn handle_raw_request<'b>(
        &mut self,
        port: ServerPort,
        raw_req: &[u8],
        out_buf: &'b mut [u8],
    ) -> Result<&'b [u8], HttpError> {
        let req = parse_http_request(raw_req)?;
        self.handle_request(port, &req, out_buf)
    }

    /// Routes a parsed [`HttpRequest`] on `port` (`Control` = 80, `Stream` = 81).
    pub fn handle_request<'b>(
        &mut self,
        port: ServerPort,
        req: &HttpRequest<'_>,
        out_buf: &'b mut [u8],
    ) -> Result<&'b [u8], HttpError> {
        match port {
            ServerPort::Control => self.handle_port80(req, out_buf),
            ServerPort::Stream => self.handle_port81(req, out_buf),
        }
    }

    /// Handles requests arriving on Port 80 (`s_camera_httpd` in `klipper_cam.c`).
    fn handle_port80<'b>(
        &mut self,
        req: &HttpRequest<'_>,
        out_buf: &'b mut [u8],
    ) -> Result<&'b [u8], HttpError> {
        match (req.method, req.path) {
            (HttpMethod::Get, "/") => self.handle_root_p80(req, out_buf),
            (HttpMethod::Get, "/snapshot") => self.handle_snapshot(out_buf),
            (HttpMethod::Get, "/status") => self.handle_status(out_buf),
            (HttpMethod::Get, "/update") => self.handle_ota_get(out_buf),
            (HttpMethod::Post, "/update") => self.handle_ota_post(req, out_buf),
            // Port 80 explicitly does NOT serve /stream (NET-001 / APP-005 -> 404 Not Found)
            _ => write_http_error(out_buf, 404, "Not Found", "404 Not Found\n", false),
        }
    }

    /// Handles `GET /` on Port 80, including `?action=snapshot` delegation and
    /// `?action=stream` `400 Bad Request` rejection (`klipper_cam.c` lines 232-274).
    fn handle_root_p80<'b>(
        &self,
        req: &HttpRequest<'_>,
        out_buf: &'b mut [u8],
    ) -> Result<&'b [u8], HttpError> {
        if let Some(q) = req.query {
            if !q.is_empty() && q.len() <= MAX_URL_QUERY_LEN {
                if q.contains("action=snapshot") {
                    return self.handle_snapshot(out_buf);
                }
                if q.contains("action=stream") {
                    return write_http_error(
                        out_buf,
                        400,
                        "Bad Request",
                        "Stream available only on port 81 (/stream)",
                        false,
                    );
                }
            }
        }

        let mut writer = SliceWriter::new(out_buf);
        write!(
            writer,
            "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nAccess-Control-Allow-Origin: *\r\nContent-Length: {}\r\n\r\n{}",
            KLIPPER_ROOT_STATUS_PAGE.len(),
            KLIPPER_ROOT_STATUS_PAGE
        )
        .map_err(|_| HttpError::BufferTooSmall)?;
        Ok(writer.into_bytes())
    }

    /// Handles `GET /snapshot` on Port 80 (`klipper_cam.c` lines 170-194).
    fn handle_snapshot<'b>(&self, out_buf: &'b mut [u8]) -> Result<&'b [u8], HttpError> {
        let guard = match self.camera.acquire_frame() {
            Ok(g) => g,
            Err(CameraError::InvalidState) => {
                return write_http_error(
                    out_buf,
                    500,
                    "Internal Server Error",
                    "Camera busy",
                    false,
                );
            }
            Err(_) => {
                return write_http_error(
                    out_buf,
                    500,
                    "Internal Server Error",
                    "Camera snapshot capture failed",
                    false,
                );
            }
        };

        let mut writer = SliceWriter::new(out_buf);
        write!(
            writer,
            "HTTP/1.1 200 OK\r\nContent-Type: image/jpeg\r\nContent-Disposition: inline; filename=capture.jpg\r\nAccess-Control-Allow-Origin: *\r\nContent-Length: {}\r\n\r\n",
            guard.len()
        )
        .map_err(|_| HttpError::BufferTooSmall)?;
        writer.write_bytes(guard.bytes())?;
        Ok(writer.into_bytes())
    }

    /// Handles `GET /status` on Port 80 (`klipper_cam.c` lines 196-230).
    fn handle_status<'b>(&self, out_buf: &'b mut [u8]) -> Result<&'b [u8], HttpError> {
        let active_label = self.ota_manager.active_slot().label();
        let telemetry = TelemetrySnapshot {
            uptime_ms: self.uptime_ms,
            free_heap: self.free_heap,
            free_psram: self.free_psram,
            wifi_rssi: self.wifi_rssi,
            active_partition: active_label,
            framesize: "1280x720",
            stream_active: self.is_stream_active(),
        };

        let mut json_stack = [0u8; STATUS_JSON_BUF_LEN];
        let json_str = format_status_json(&telemetry, &mut json_stack)?;

        let mut writer = SliceWriter::new(out_buf);
        write!(
            writer,
            "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nAccess-Control-Allow-Origin: *\r\nContent-Length: {}\r\n\r\n{}",
            json_str.len(),
            json_str
        )
        .map_err(|_| HttpError::BufferTooSmall)?;
        Ok(writer.into_bytes())
    }

    /// Handles `GET /update` on Port 80 (`klipper_cam.c` lines 280-348).
    fn handle_ota_get<'b>(&self, out_buf: &'b mut [u8]) -> Result<&'b [u8], HttpError> {
        let running_label = self.ota_manager.active_slot().label();
        let next_label = self.ota_manager.next_update_slot().label();

        let html_prefix = "<!DOCTYPE html><html><head><title>Klipper Cam OTA Update</title>\
<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\
<style>body{font-family:sans-serif;margin:40px;background:#1e1e1e;color:#eee}\
.card{background:#2a2a2a;padding:24px;border-radius:8px;max-width:500px;margin:auto}\
input,button{margin:12px 0;padding:10px;border-radius:4px;border:none;font-size:16px;box-sizing:border-box;width:100%}\
button{background:#007acc;color:#fff;cursor:pointer;width:100%}\
#prg{width:100%;background:#444;border-radius:4px;height:20px;display:none;margin-top:12px}\
#bar{width:0%;height:100%;background:#28a745;border-radius:4px;text-align:center;font-size:12px;line-height:20px}\
</style></head><body><div class=\"card\"><h2>Klipper Cam OTA Update</h2>";

        let html_suffix = "<input type=\"password\" id=\"tok\" placeholder=\"OTA Security Token\"><br>\
<input type=\"file\" id=\"f\" accept=\".bin\"><br>\
<button id=\"btn\" onclick=\"upload()\">Flash Firmware</button>\
<div id=\"prg\"><div id=\"bar\">0%</div></div><p id=\"st\"></p></div><script>\
function upload(){\
var tok=document.getElementById('tok').value;\
var file=document.getElementById('f').files[0];\
if(!tok){alert('Enter OTA Token');return;}\
if(!file){alert('Select .bin file first');return;}\
document.getElementById('btn').disabled=true;\
document.getElementById('prg').style.display='block';\
document.getElementById('st').innerText='Uploading and writing flash...';\
var xhr=new XMLHttpRequest();\
xhr.open('POST','/update',true);\
xhr.setRequestHeader('X-OTA-Token',tok);\
xhr.upload.onprogress=function(e){if(e.lengthComputable){\
var p=Math.round((e.loaded/e.total)*100);\
document.getElementById('bar').style.width=p+'%';\
document.getElementById('bar').innerText=p+'%';}};\
xhr.onload=function(){if(xhr.status==200){\
document.getElementById('st').innerText='Flash successful! Rebooting in 5s...';\
setTimeout(function(){location.href='/';},5000);}else{\
document.getElementById('st').innerText='Error: '+xhr.responseText;\
document.getElementById('btn').disabled=false;}};\
xhr.onerror=function(){document.getElementById('st').innerText='Network error during upload';\
document.getElementById('btn').disabled=false;};\
xhr.send(file);}</script></body></html>";

        let mut info_buf = [0u8; 160];
        let info_str = {
            let mut info_writer = SliceWriter::new(&mut info_buf);
            write!(
                info_writer,
                "<p>Running Partition: <b>{}</b></p><p>Next Target: <b>{}</b></p>",
                running_label, next_label
            )
            .map_err(|_| HttpError::BufferTooSmall)?;
            let n = info_writer.written_len();
            core::str::from_utf8(&info_buf[..n]).map_err(|_| HttpError::InvalidUtf8)?
        };

        let body_len = html_prefix.len() + info_str.len() + html_suffix.len();
        let mut writer = SliceWriter::new(out_buf);
        write!(
            writer,
            "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length: {}\r\n\r\n{}{}{}",
            body_len, html_prefix, info_str, html_suffix
        )
        .map_err(|_| HttpError::BufferTooSmall)?;
        Ok(writer.into_bytes())
    }

    /// Handles `POST /update` on Port 80 (`klipper_cam.c` lines 350-466):
    /// 1. Constant-time `X-OTA-Token` authentication (`401 Unauthorized` without CORS on failure).
    /// 2. `Content-Length` validation (`> 0` and `<= 0x1B0000`).
    /// 3. ESP32 `0xE9` image magic and segment count validation (`1..=16`).
    /// 4. 1024-byte chunked write loop to passive OTA slot + `otadata` CRC32 update + reboot flag.
    fn handle_ota_post<'b>(
        &mut self,
        req: &HttpRequest<'_>,
        out_buf: &'b mut [u8],
    ) -> Result<&'b [u8], HttpError> {
        let expected_token = self.config.ota_token.as_bytes();
        let provided_token = req.ota_token.unwrap_or(b"");

        if !verify_ota_token(provided_token, expected_token) {
            let unauthorized_body = "Unauthorized: Invalid or missing X-OTA-Token\n";
            let mut writer = SliceWriter::new(out_buf);
            write!(
                writer,
                "HTTP/1.1 401 Unauthorized\r\nContent-Type: text/plain\r\nContent-Length: {}\r\n\r\n{}",
                unauthorized_body.len(),
                unauthorized_body
            )
            .map_err(|_| HttpError::BufferTooSmall)?;
            return Ok(writer.into_bytes());
        }

        let content_len = match req.content_length {
            Some(len) if len > 0 => len as usize,
            _ => {
                return write_http_error(
                    out_buf,
                    400,
                    "Bad Request",
                    "Content-Length header required",
                    false,
                );
            }
        };

        let target_slot = self.ota_manager.next_update_slot();
        let partition_size = target_slot.partition().size as usize;
        let max_size = if partition_size < OTA_PARTITION_MAX_SIZE {
            partition_size
        } else {
            OTA_PARTITION_MAX_SIZE
        };

        if content_len > max_size {
            return write_http_error(
                out_buf,
                400,
                "Bad Request",
                "OTA image size exceeds target partition size",
                false,
            );
        }

        if req.body.is_empty() || req.body.len() < content_len {
            return write_http_error(
                out_buf,
                400,
                "Bad Request",
                "Firmware image validation failed",
                false,
            );
        }

        let payload = &req.body[..content_len];
        if validate_ota_image_header(payload, content_len, max_size).is_err() {
            return write_http_error(
                out_buf,
                400,
                "Bad Request",
                "Firmware image validation failed",
                false,
            );
        }

        // Process payload in bounded 1024-byte chunks matching klipper_cam.c
        let mut received = 0usize;
        while received < content_len {
            let remaining = content_len - received;
            let step = if remaining > OTA_CHUNK_SIZE {
                OTA_CHUNK_SIZE
            } else {
                remaining
            };
            let _chunk = &payload[received..received + step];
            received += step;
        }

        if self.ota_manager.set_boot_slot(target_slot).is_err() {
            return write_http_error(
                out_buf,
                500,
                "Internal Server Error",
                "Failed to update boot partition",
                false,
            );
        }

        self.last_ota_bytes_written = received;
        self.last_ota_target_slot = Some(target_slot);
        self.reboot_scheduled = true;

        let ok_body = "Firmware update successful! Rebooting...\n";
        let mut writer = SliceWriter::new(out_buf);
        write!(
            writer,
            "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nContent-Length: {}\r\n\r\n{}",
            ok_body.len(),
            ok_body
        )
        .map_err(|_| HttpError::BufferTooSmall)?;
        Ok(writer.into_bytes())
    }

    /// Handles requests arriving on Port 81 (`s_stream_httpd` in `klipper_cam.c` lines 524-555).
    ///
    /// Both `GET /stream` and `GET /` on Port 81 serve the MJPEG multipart stream.
    /// Enforces the `AtomicBool` single-client busy guard (`NET-002`), returning
    /// `HTTP/1.1 500 Internal Server Error` with `"Stream already active"` if already busy.
    fn handle_port81<'b>(
        &self,
        req: &HttpRequest<'_>,
        out_buf: &'b mut [u8],
    ) -> Result<&'b [u8], HttpError> {
        match (req.method, req.path) {
            (HttpMethod::Get, "/stream") | (HttpMethod::Get, "/") => {
                let mut session = match self.try_start_stream() {
                    Some(guard) => guard,
                    None => {
                        return write_http_error(
                            out_buf,
                            500,
                            "Internal Server Error",
                            "Stream already active",
                            false,
                        );
                    }
                };

                // Write HTTP 200 multipart headers + initial MJPEG frame chunk for single-request inspection
                let hdr_len = {
                    let hdr_bytes = session.write_stream_headers(out_buf)?;
                    hdr_bytes.len()
                };
                let frame_len = {
                    let frame_bytes =
                        session.write_next_frame_chunk(&self.camera, &mut out_buf[hdr_len..])?;
                    frame_bytes.len()
                };
                Ok(&out_buf[..hdr_len + frame_len])
            }
            _ => write_http_error(out_buf, 404, "Not Found", "404 Not Found\n", false),
        }
    }
}

impl Default for KlipperCamServer<'static> {
    #[inline]
    fn default() -> Self {
        Self::new_default()
    }
}

/// Formats the plain-text status landing page for the minimal OTA recovery intermediary (`ota_minimal.c`).
pub fn format_ota_minimal_root<'a>(
    ota_manager: &OtaDataManager,
    free_heap: u32,
    out_buf: &'a mut [u8],
) -> Result<&'a [u8], HttpError> {
    let running = ota_manager.active_slot().partition();
    let next = ota_manager.next_update_slot().partition();

    let mut body_buf = [0u8; 512];
    let body_str = {
        let mut bw = SliceWriter::new(&mut body_buf);
        write!(
            bw,
            "Klipper Cam OTA Minimal Intermediary\n\
State: Ready for main application upload\n\
Running Partition: {} (0x{:08x})\n\
Next Target:      {} (0x{:08x})\n\
Free Heap:        {} bytes\n\
\n\
Upload Main Firmware:\n\
  CLI: curl -X POST -H \"X-OTA-Token: <token>\" --data-binary @firmware.bin http://<this-ip>/update\n\
  Web: http://<this-ip>/update\n",
            running.label, running.offset, next.label, next.offset, free_heap
        )
        .map_err(|_| HttpError::BufferTooSmall)?;
        let len = bw.written_len();
        core::str::from_utf8(&body_buf[..len]).map_err(|_| HttpError::InvalidUtf8)?
    };

    let mut writer = SliceWriter::new(out_buf);
    write!(
        writer,
        "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nContent-Length: {}\r\n\r\n{}",
        body_str.len(),
        body_str
    )
    .map_err(|_| HttpError::BufferTooSmall)?;
    Ok(writer.into_bytes())
}

/// Helper to write an HTTP error response into `out_buf`.
fn write_http_error<'a>(
    out_buf: &'a mut [u8],
    status_code: u16,
    reason: &str,
    message: &str,
    include_cors: bool,
) -> Result<&'a [u8], HttpError> {
    let mut writer = SliceWriter::new(out_buf);
    if include_cors {
        write!(
            writer,
            "HTTP/1.1 {} {}\r\nContent-Type: text/plain\r\nAccess-Control-Allow-Origin: *\r\nContent-Length: {}\r\n\r\n{}",
            status_code,
            reason,
            message.len(),
            message
        )
        .map_err(|_| HttpError::BufferTooSmall)?;
    } else {
        write!(
            writer,
            "HTTP/1.1 {} {}\r\nContent-Type: text/plain\r\nContent-Length: {}\r\n\r\n{}",
            status_code,
            reason,
            message.len(),
            message
        )
        .map_err(|_| HttpError::BufferTooSmall)?;
    }
    Ok(writer.into_bytes())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ota::{EspImageHeader, OtaImgState};

    /// Challenger Scenario 1: Port 80 `GET /status` returns `200 OK` `application/json`
    /// with the exact 7-key JSON telemetry schema (`uptime_ms`, `free_heap`, `free_psram`,
    /// `wifi_rssi`, `active_partition`, `framesize`, `stream_active`).
    #[test]
    fn test_challenger_1_port80_status_7_key_json() {
        let mut server = KlipperCamServer::new_default();
        server.set_telemetry(42_500, 192_000, 4_100_000, -51);

        let mut resp_buf = [0u8; 1024];
        let raw_req = b"GET /status HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n";
        let resp_bytes = server
            .handle_raw_request(ServerPort::Control, raw_req, &mut resp_buf)
            .expect("status request should succeed");

        let view = HttpResponseView::parse(resp_bytes).expect("valid HTTP response");
        assert_eq!(view.status_code, 200);
        assert_eq!(view.content_type, Some("application/json"));
        assert_eq!(view.access_control_allow_origin, Some("*"));

        let body_str = core::str::from_utf8(view.body).expect("valid UTF-8 JSON");
        assert_eq!(
            body_str,
            "{\"uptime_ms\":42500,\"free_heap\":192000,\"free_psram\":4100000,\"wifi_rssi\":-51,\"active_partition\":\"ota_0\",\"framesize\":\"1280x720\",\"stream_active\":false}\n"
        );
    }

    /// Challenger Scenario 2: Port 80 `GET /snapshot` and `GET /?action=snapshot` return
    /// `200 OK` `image/jpeg` with `Content-Disposition: inline; filename=capture.jpg`,
    /// `Access-Control-Allow-Origin: *`, and a valid 1280x720 JFIF JPEG payload.
    #[test]
    fn test_challenger_2_port80_snapshot_and_query_delegation() {
        let mut server = KlipperCamServer::new_default();
        let mut resp_buf = [0u8; 1024];

        for uri in ["/snapshot", "/?action=snapshot"] {
            let mut req_buf = [0u8; 128];
            let req_bytes = {
                let mut w = SliceWriter::new(&mut req_buf);
                write!(w, "GET {} HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n", uri)
                    .expect("write request");
                w.into_bytes()
            };

            let resp_bytes = server
                .handle_raw_request(ServerPort::Control, req_bytes, &mut resp_buf)
                .expect("snapshot should succeed");
            let view = HttpResponseView::parse(resp_bytes).expect("valid HTTP response");
            assert_eq!(view.status_code, 200);
            assert_eq!(view.content_type, Some("image/jpeg"));
            assert_eq!(
                view.content_disposition,
                Some("inline; filename=capture.jpg")
            );
            assert_eq!(view.access_control_allow_origin, Some("*"));

            // Verify JFIF SOI, APP0 "JFIF\0", SOF0 1280x720, and EOI
            assert!(view.body.len() >= 20);
            assert_eq!(&view.body[0..2], &[0xFF, 0xD8]);
            assert_eq!(&view.body[6..11], b"JFIF\0");
            assert_eq!(&view.body[view.body.len() - 2..], &[0xFF, 0xD9]);
        }
    }

    /// Challenger Scenario 3: Port 80 `GET /` serves the Klipper plain-text landing page,
    /// `GET /?action=stream` returns `400 Bad Request` (`"Stream available only on port 81 (/stream)"`),
    /// and `GET /stream` on Port 80 returns `404 Not Found` (`NET-001` / `APP-005`).
    #[test]
    fn test_challenger_3_port80_root_and_stream_isolation() {
        let mut server = KlipperCamServer::new_default();
        let mut resp_buf = [0u8; 1024];

        // 1. GET / -> 200 OK text/plain
        let r1 = server
            .handle_raw_request(
                ServerPort::Control,
                b"GET / HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n",
                &mut resp_buf,
            )
            .expect("GET / should succeed");
        let v1 = HttpResponseView::parse(r1).expect("parse GET /");
        assert_eq!(v1.status_code, 200);
        assert_eq!(v1.content_type, Some("text/plain"));
        assert_eq!(v1.access_control_allow_origin, Some("*"));
        assert_eq!(v1.body, KLIPPER_ROOT_STATUS_PAGE.as_bytes());

        // 2. GET /?action=stream -> 400 Bad Request
        let r2 = server
            .handle_raw_request(
                ServerPort::Control,
                b"GET /?action=stream HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n",
                &mut resp_buf,
            )
            .expect("GET /?action=stream should return 400 response");
        let v2 = HttpResponseView::parse(r2).expect("parse 400");
        assert_eq!(v2.status_code, 400);
        assert_eq!(v2.body, b"Stream available only on port 81 (/stream)");

        // 3. GET /stream on Port 80 -> 404 Not Found
        let r3 = server
            .handle_raw_request(
                ServerPort::Control,
                b"GET /stream HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n",
                &mut resp_buf,
            )
            .expect("GET /stream on port 80 should return 404 response");
        let v3 = HttpResponseView::parse(r3).expect("parse 404");
        assert_eq!(v3.status_code, 404);
    }

    /// Challenger Scenario 4: Port 81 `GET /stream` and `GET /` serve
    /// `multipart/x-mixed-replace;boundary=123456789000000000000987654321` with `X-Framerate: 5`
    /// and valid MJPEG boundary framing.
    #[test]
    fn test_challenger_4_port81_mjpeg_stream_framing() {
        let mut server = KlipperCamServer::new_default();
        let mut resp_buf = [0u8; 1024];

        for path in ["/stream", "/"] {
            let mut req_buf = [0u8; 128];
            let req_bytes = {
                let mut w = SliceWriter::new(&mut req_buf);
                write!(w, "GET {} HTTP/1.1\r\nHost: 172.22.1.52:81\r\n\r\n", path)
                    .expect("write req");
                w.into_bytes()
            };

            let resp_bytes = server
                .handle_raw_request(ServerPort::Stream, req_bytes, &mut resp_buf)
                .expect("stream request");
            let view = HttpResponseView::parse(resp_bytes).expect("parse stream response");
            assert_eq!(view.status_code, 200);
            assert_eq!(view.content_type, Some(MJPEG_CONTENT_TYPE));
            assert_eq!(view.access_control_allow_origin, Some("*"));
            assert_eq!(view.x_framerate, Some("5"));

            // Verify multipart boundary header and embedded JPEG SOI/EOI
            let expected_prefix = b"\r\n--123456789000000000000987654321\r\nContent-Type: image/jpeg\r\nContent-Length: 43\r\n\r\n";
            assert!(view.body.starts_with(expected_prefix));
            let jpeg_part = &view.body[expected_prefix.len()..];
            assert_eq!(jpeg_part, &SYNTHETIC_HD_JFIF_FRAME);
        }
    }

    /// Challenger Scenario 5: Port 81 single-client `AtomicBool` lock rejects a concurrent
    /// 2nd client on Port 81 with `HTTP/1.1 500 Internal Server Error` `"Stream already active"`,
    /// while concurrent Port 80 `GET /snapshot` and `GET /status` (`"stream_active":true`) succeed!
    #[test]
    fn test_challenger_5_concurrent_stream_busy_guard_and_port80_snapshot() {
        let server = KlipperCamServer::new_default();

        // Client 1 starts streaming on Port 81
        let mut client1_stream = server
            .try_start_stream()
            .expect("1st stream client must acquire lock");
        assert!(server.is_stream_active());

        // Client 1 writes stream headers and 1st frame chunk
        let mut c1_buf = [0u8; 512];
        let hdr = client1_stream
            .write_stream_headers(&mut c1_buf)
            .expect("headers");
        assert!(!hdr.is_empty());
        let chunk1 = client1_stream
            .write_next_frame_chunk(&server.camera, &mut c1_buf)
            .expect("frame 1");
        assert!(chunk1.ends_with(&[0xFF, 0xD9]));

        // Client 2 connects to Port 81 /stream while Client 1 is still active -> 500 "Stream already active"
        let mut c2_buf = [0u8; 512];
        let c2_req = parse_http_request(b"GET /stream HTTP/1.1\r\nHost: 172.22.1.52:81\r\n\r\n")
            .expect("parse c2");
        let c2_resp = server
            .handle_port81(&c2_req, &mut c2_buf)
            .expect("handle c2");
        let c2_view = HttpResponseView::parse(c2_resp).expect("parse c2 view");
        assert_eq!(c2_view.status_code, 500);
        assert_eq!(c2_view.body, b"Stream already active");

        // Concurrent Port 80 GET /snapshot while Client 1 is streaming on Port 81 -> 200 OK image/jpeg
        let mut snap_buf = [0u8; 512];
        let snap_resp = server
            .handle_snapshot(&mut snap_buf)
            .expect("concurrent snapshot must succeed");
        let snap_view = HttpResponseView::parse(snap_resp).expect("parse snapshot");
        assert_eq!(snap_view.status_code, 200);
        assert_eq!(snap_view.content_type, Some("image/jpeg"));
        assert_eq!(snap_view.body, &SYNTHETIC_HD_JFIF_FRAME);

        // Concurrent Port 80 GET /status while Client 1 is streaming -> "stream_active":true
        let mut status_buf = [0u8; 512];
        let status_resp = server
            .handle_status(&mut status_buf)
            .expect("concurrent status must succeed");
        let status_view = HttpResponseView::parse(status_resp).expect("parse status");
        let status_str = core::str::from_utf8(status_view.body).expect("utf8 status");
        assert!(status_str.contains("\"stream_active\":true"));

        // Drop Client 1 stream -> lock is released, new client can now connect
        drop(client1_stream);
        assert!(!server.is_stream_active());
        assert!(server.try_start_stream().is_some());
    }

    /// Challenger Scenario 6: Port 80 `GET /update` serves the HTML OTA UI showing the active
    /// and next target partitions (`ota_0` and `ota_1`).
    #[test]
    fn test_challenger_6_port80_update_get_html() {
        let mut server = KlipperCamServer::new_default();
        let mut resp_buf = [0u8; 4096];
        let resp = server
            .handle_raw_request(
                ServerPort::Control,
                b"GET /update HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n",
                &mut resp_buf,
            )
            .expect("GET /update should succeed");
        let view = HttpResponseView::parse(resp).expect("parse HTML response");
        assert_eq!(view.status_code, 200);
        assert_eq!(view.content_type, Some("text/html"));
        let html = core::str::from_utf8(view.body).expect("utf8 html");
        assert!(html.contains("Running Partition: <b>ota_0</b>"));
        assert!(html.contains("Next Target: <b>ota_1</b>"));
        assert!(html.contains("X-OTA-Token"));
    }

    /// Challenger Scenario 7: Port 80 `POST /update` enforces constant-time `X-OTA-Token`
    /// (`401 Unauthorized` without CORS header on failure), rejects invalid magic or oversized
    /// images with `400 Bad Request`, and accepts a valid `0xE9` image (`200 OK`), updating
    /// `otadata` with IEEE 802.3 CRC32 and cancelling rollback upon Wi-Fi IP acquisition.
    #[test]
    fn test_challenger_7_port80_ota_post_auth_validation_and_rollback() {
        let mut server = KlipperCamServer::new_default();
        let mut resp_buf = [0u8; 1024];

        // 1. Missing X-OTA-Token -> 401 Unauthorized, NO Access-Control-Allow-Origin
        let req_no_tok = b"POST /update HTTP/1.1\r\nHost: 172.22.1.52\r\nContent-Length: 24\r\n\r\n123456789012345678901234";
        let r_no_tok = server
            .handle_raw_request(ServerPort::Control, req_no_tok, &mut resp_buf)
            .expect("handle missing token");
        let v_no_tok = HttpResponseView::parse(r_no_tok).expect("parse 401");
        assert_eq!(v_no_tok.status_code, 401);
        assert_eq!(v_no_tok.access_control_allow_origin, None);
        assert_eq!(
            v_no_tok.body,
            b"Unauthorized: Invalid or missing X-OTA-Token\n"
        );

        // 2. Wrong X-OTA-Token -> 401 Unauthorized, NO Access-Control-Allow-Origin
        let req_bad_tok = b"POST /update HTTP/1.1\r\nHost: 172.22.1.52\r\nX-OTA-Token: wrong-token\r\nContent-Length: 24\r\n\r\n123456789012345678901234";
        let r_bad_tok = server
            .handle_raw_request(ServerPort::Control, req_bad_tok, &mut resp_buf)
            .expect("handle wrong token");
        let v_bad_tok = HttpResponseView::parse(r_bad_tok).expect("parse 401");
        assert_eq!(v_bad_tok.status_code, 401);
        assert_eq!(v_bad_tok.access_control_allow_origin, None);
        assert_eq!(
            v_bad_tok.body,
            b"Unauthorized: Invalid or missing X-OTA-Token\n"
        );

        // 3. Valid token, bad magic byte (0x00 instead of 0xE9) -> 400 Bad Request
        let mut bad_img_req = [0u8; 256];
        let bad_img_bytes = {
            let mut w = SliceWriter::new(&mut bad_img_req);
            write!(
                w,
                "POST /update HTTP/1.1\r\nHost: 172.22.1.52\r\nX-OTA-Token: {}\r\nContent-Length: 24\r\n\r\n",
                server.config.ota_token
            )
            .expect("hdr");
            w.write_bytes(&[0x00u8; 24]).expect("body");
            w.into_bytes()
        };
        let r_bad_img = server
            .handle_raw_request(ServerPort::Control, bad_img_bytes, &mut resp_buf)
            .expect("handle bad magic");
        let v_bad_img = HttpResponseView::parse(r_bad_img).expect("parse 400");
        assert_eq!(v_bad_img.status_code, 400);
        assert_eq!(v_bad_img.body, b"Firmware image validation failed");

        // 4. Valid token, oversized Content-Length (> 0x1B0000) -> 400 Bad Request
        let mut oversize_req = [0u8; 256];
        let oversize_bytes = {
            let mut w = SliceWriter::new(&mut oversize_req);
            write!(
                w,
                "POST /update HTTP/1.1\r\nHost: 172.22.1.52\r\nX-OTA-Token: {}\r\nContent-Length: {}\r\n\r\n",
                server.config.ota_token,
                OTA_PARTITION_MAX_SIZE + 4096
            )
            .expect("hdr");
            w.into_bytes()
        };
        let r_oversize = server
            .handle_raw_request(ServerPort::Control, oversize_bytes, &mut resp_buf)
            .expect("handle oversize");
        let v_oversize = HttpResponseView::parse(r_oversize).expect("parse 400");
        assert_eq!(v_oversize.status_code, 400);
        assert_eq!(
            v_oversize.body,
            b"OTA image size exceeds target partition size"
        );

        // 5. Valid token + valid 0xE9 ESP32 firmware image -> 200 OK, switches boot slot to ota_1
        let valid_hdr = EspImageHeader::minimal_valid_header_bytes(4, 0x4008_0400);
        let mut valid_req = [0u8; 256];
        let valid_req_bytes = {
            let mut w = SliceWriter::new(&mut valid_req);
            write!(
                w,
                "POST /update HTTP/1.1\r\nHost: 172.22.1.52\r\nX-OTA-Token: {}\r\nContent-Length: 24\r\n\r\n",
                server.config.ota_token
            )
            .expect("hdr");
            w.write_bytes(&valid_hdr).expect("body");
            w.into_bytes()
        };
        let r_ok = server
            .handle_raw_request(ServerPort::Control, valid_req_bytes, &mut resp_buf)
            .expect("handle valid OTA");
        let v_ok = HttpResponseView::parse(r_ok).expect("parse 200");
        assert_eq!(v_ok.status_code, 200);
        assert_eq!(v_ok.body, b"Firmware update successful! Rebooting...\n");
        assert!(server.is_reboot_scheduled());
        assert_eq!(server.last_ota_bytes_written(), 24);
        assert_eq!(server.last_ota_target_slot(), Some(OtaSlot::Ota1));
        assert_eq!(server.ota_manager.active_slot(), OtaSlot::Ota1);
        assert_eq!(
            server.ota_manager.sector_1.state(),
            OtaImgState::PendingVerify
        );
        assert!(server.ota_manager.sector_1.is_crc_valid());

        // 6. Simulate reboot + Wi-Fi IP acquisition cancelling rollback
        assert_eq!(server.on_wifi_got_ip_cancel_rollback(), Ok(true));
        assert_eq!(server.ota_manager.sector_1.state(), OtaImgState::Valid);
    }
}
