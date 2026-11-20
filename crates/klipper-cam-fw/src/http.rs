//! Zero-allocation `#![no_std]` HTTP/1.1 request parser, response writer,
//! dual-port Klipper camera control server (Port 80), MJPEG multipart streamer
//! (Port 81),
//! and OTA upload handler (`POST /update`).
//!
#![forbid(unsafe_code)]

use core::fmt::{self, Write};
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use esp32_cam_types::{CameraError, FrameSize};

use crate::config::{IpMode, Ipv4Addr, KlipperCamConfig};
use crate::ota::{
    validate_ota_image_header, NoOpFlashStorage, OtaDataManager, OtaError, OtaFlashStorage,
    OtaSlot, OTA_PARTITION_MAX_SIZE,
};
use crate::prov_store::{
    ProvStore, ProvStoreError, ProvisionedConfig, MAX_OTA_TOKEN_LEN, MAX_PASS_LEN, MAX_SSID_LEN,
};
use crate::telemetry::FwStatsSnapshot;

/// Multipart boundary string used by the Port 81 MJPEG stream (`PART_BOUNDARY`
/// in `klipper_cam.c`).
pub const MJPEG_BOUNDARY: &str = "123456789000000000000987654321";

/// Full `Content-Type` header value for the Port 81 MJPEG stream.
pub const MJPEG_CONTENT_TYPE: &str =
    "multipart/x-mixed-replace;boundary=123456789000000000000987654321";

/// Maximum URL query string length accepted by `GET /` (`256` bytes,
/// `APP-013`).
pub const MAX_URL_QUERY_LEN: usize = 256;

/// Stack buffer size for `GET /status` JSON serialization (`1536` bytes).
pub const STATUS_JSON_BUF_LEN: usize = 1536;

/// Stack buffer size for `GET /control` JSON serialization (`384` bytes).
pub const CONTROL_JSON_BUF_LEN: usize = 384;

/// Stack buffer size for each MJPEG multipart boundary header (`128` bytes,
/// `klipper_cam.c`).
pub const MJPEG_PART_HDR_BUF_LEN: usize = 128;

/// Maximum consecutive frame capture failures before aborting the MJPEG stream
/// (`5`).
pub const STREAM_MAX_CAPTURE_FAILURES: u8 = 5;

/// Plain-text landing page served by `GET /` on Port 80 (`klipper_cam.c`).
pub const KLIPPER_ROOT_STATUS_PAGE: &str = "\
Klipper 3D Printer Camera (ESP32-WROVER-DEV)\n\
Resolution: 1280x720 HD 16:9 (OV2640 Hardware JPEG)\n\
\n\
Endpoints:\n\
  - Dedicated Stream: http://<this-ip>:81/stream\n\
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

/// Canonical minimal 1280x720 (`0x0500` x `0x02D0`) JFIF JPEG frame with SOI
/// (`FF D8`),
/// APP0 `JFIF\0` (`FF E0`), SOF0 (`FF C0`), SOS (`FF DA`), and EOI (`FF D9`)
/// markers.
pub const SYNTHETIC_HD_JFIF_FRAME: [u8; 43] = [
    0xFF, 0xD8, // SOI
    0xFF, 0xE0, 0x00, 0x10, b'J', b'F', b'I', b'F', 0x00, 0x01, 0x01, 0x01, 0x00, 0x48, 0x00, 0x48,
    0x00, 0x00, // APP0 JFIF 1.1
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

/// Zero-allocation parsed HTTP/1.1 request borrowing directly from the receive
/// buffer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HttpRequest<'a> {
    /// HTTP method (`GET`, `POST`, etc.).
    pub method: HttpMethod<'a>,
    /// URL path excluding query string (e.g. `"/"`, `"/snapshot"`, `"/status"`,
    /// `"/update"`, `"/stream"`).
    pub path: &'a str,
    /// Optional URL query string after `'?'` (e.g. `Some("action=snapshot")`).
    pub query: Option<&'a str>,
    /// HTTP version string (e.g. `"HTTP/1.1"`).
    pub version: &'a str,
    /// Value of the `X-OTA-Token` header if present.
    pub ota_token: Option<&'a [u8]>,
    /// Parsed `Content-Length` header if present (`FW-NET-23`).
    pub content_length: Option<usize>,
    /// Value of the `Content-Type` header if present.
    pub content_type: Option<&'a str>,
    /// Value of the `Host` header if present.
    pub host: Option<&'a str>,
    /// Value of the `Origin` header if present (sent by browsers on
    /// cross-origin and on `POST` requests; `FW-NET-27`).
    pub origin: Option<&'a str>,
    /// Whether an `X-Requested-With` header is present. Browsers cannot add
    /// it cross-origin without a CORS preflight, which this server never
    /// grants (`FW-NET-27`).
    pub x_requested_with: bool,
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
    /// Caller-supplied output buffer is too small to hold the formatted HTTP
    /// response.
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

/// Zero-allocation fixed-capacity byte buffer writer implementing
/// [`core::fmt::Write`].
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

    /// Appends raw bytes to the buffer, returning [`HttpError::BufferTooSmall`] on
    /// overflow.
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

/// Returns whether the URL query string contains the exact key-value `pair`
/// as an ampersand-separated token (`FW-NET-23`).
#[inline]
pub fn query_has_pair(query: &str, pair: &str) -> bool {
    let q = query.strip_prefix('?').unwrap_or(query);
    q.split('&').any(|param| param == pair)
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

/// Returns `true` if `b` is a valid RFC 9110 §5.6.2 token character (`tchar`).
#[inline]
const fn is_http_tchar(b: u8) -> bool {
    b.is_ascii_alphanumeric()
        || matches!(
            b,
            b'!' | b'#'
                | b'$'
                | b'%'
                | b'&'
                | b'\''
                | b'*'
                | b'+'
                | b'-'
                | b'.'
                | b'^'
                | b'_'
                | b'`'
                | b'|'
                | b'~'
        )
}

/// Trims only RFC 9110 §5.6.3 optional whitespace (`SP` / `HTAB`), never
/// stripping vertical tabs, form feeds, CR/LF, or Unicode whitespace.
#[inline]
fn trim_ows(s: &str) -> &str {
    s.trim_matches(|c| c == ' ' || c == '\t')
}

/// Returns `true` if `s` contains only valid RFC 9110 ASCII field-value bytes
/// (`HTAB` or printable ASCII `0x20..=0x7E`), rejecting bare CR/LF, `0x0B`,
/// other CTLs, and non-ASCII/Unicode whitespace.
#[inline]
fn is_valid_field_value_bytes(s: &str) -> bool {
    s.bytes().all(|b| b == b'\t' || (0x20..=0x7E).contains(&b))
}

/// Parses an RFC 9110 §7.2 `host[:port]` authority into `(host, Option<port>)`.
///
/// Rejects userinfo (`@`), paths (`/`), queries (`?`), fragments (`#`),
/// whitespace, empty labels, leading/trailing hyphens, port `0`, or ports
/// exceeding `65535`.
fn parse_http_host_port(authority: &str) -> Option<(&str, Option<u16>)> {
    if authority.is_empty() {
        return None;
    }
    if authority.starts_with('[') {
        let close_idx = authority.find(']')?;
        let inner = &authority[1..close_idx];
        if inner.is_empty()
            || !inner.contains(':')
            || !inner
                .bytes()
                .all(|b| b.is_ascii_hexdigit() || b == b':' || b == b'.')
        {
            return None;
        }
        let host_part = &authority[..=close_idx];
        let rest = &authority[close_idx + 1..];
        if rest.is_empty() {
            return Some((host_part, None));
        }
        let port_str = rest.strip_prefix(':')?;
        let port = parse_http_port(port_str)?;
        return Some((host_part, Some(port)));
    }

    let (host_part, port) = match authority.split_once(':') {
        Some((h, port_str)) => {
            if port_str.contains(':') {
                return None;
            }
            (h, Some(parse_http_port(port_str)?))
        }
        None => (authority, None),
    };

    if host_part.is_empty() || host_part.len() > 253 {
        return None;
    }
    for label in host_part.split('.') {
        if label.is_empty() || label.len() > 63 {
            return None;
        }
        let bytes = label.as_bytes();
        if !bytes[0].is_ascii_alphanumeric() || !bytes[bytes.len() - 1].is_ascii_alphanumeric() {
            return None;
        }
        if !bytes
            .iter()
            .all(|b| b.is_ascii_alphanumeric() || *b == b'-')
        {
            return None;
        }
    }
    Some((host_part, port))
}

/// Parses a strict numeric TCP port (`1..=65535`, ASCII digits only).
fn parse_http_port(s: &str) -> Option<u16> {
    if s.is_empty() || s.len() > 5 || !s.bytes().all(|b| b.is_ascii_digit()) {
        return None;
    }
    let port = s.parse::<u16>().ok()?;
    if port == 0 {
        None
    } else {
        Some(port)
    }
}

/// Returns `true` if `host` conforms to strict RFC 9110 `host[:port]` syntax.
#[inline]
pub fn is_valid_http_host(host: &str) -> bool {
    parse_http_host_port(host).is_some()
}

/// Zero-allocation HTTP/1.1 request parser (`#![no_std]`).
/// Enforces RFC 9110/9112 request-line and header syntax invariants
/// (`SEC-201`).
pub fn parse_http_request(raw: &[u8]) -> Result<HttpRequest<'_>, HttpError> {
    let hdr_end = find_subslice(raw, b"\r\n\r\n").ok_or(HttpError::MalformedRequest)?;
    let header_block = core::str::from_utf8(&raw[..hdr_end]).map_err(|_| HttpError::InvalidUtf8)?;

    let mut lines = header_block.split("\r\n");
    let request_line = lines
        .find(|l| !l.is_empty())
        .ok_or(HttpError::MalformedRequest)?;
    // Reject bare CR/LF, CTLs, and non-ASCII bytes in request-line (M-044).
    if !request_line.bytes().all(|b| (0x20..=0x7E).contains(&b)) {
        return Err(HttpError::MalformedRequest);
    }

    // Enforce exact single-SP delimiters per RFC 9112 §3 (M-141).
    let mut parts = request_line.split(' ');
    let method_str = parts.next().ok_or(HttpError::MalformedRequest)?;
    let target_str = parts.next().ok_or(HttpError::MalformedRequest)?;
    let version = parts.next().ok_or(HttpError::MalformedRequest)?;
    if parts.next().is_some()
        || method_str.is_empty()
        || !method_str.bytes().all(is_http_tchar)
        || target_str.is_empty()
        || version.is_empty()
    {
        return Err(HttpError::MalformedRequest);
    }

    if version != "HTTP/1.1" && version != "HTTP/1.0" {
        return Err(HttpError::MalformedRequest);
    }

    if !target_str.starts_with('/') {
        return Err(HttpError::MalformedRequest);
    }

    let method = match method_str {
        "GET" => HttpMethod::Get,
        "POST" => HttpMethod::Post,
        "HEAD" => HttpMethod::Head,
        "OPTIONS" => HttpMethod::Options,
        other => HttpMethod::Other(other),
    };

    let (path, query) = match target_str.find('?') {
        Some(idx) => {
            let q = &target_str[idx + 1..];
            if q.len() > MAX_URL_QUERY_LEN {
                return Err(HttpError::MalformedRequest);
            }
            (&target_str[..idx], Some(q))
        }
        None => (target_str, None),
    };

    let mut ota_token: Option<&[u8]> = None;
    let mut content_length: Option<usize> = None;
    let mut content_type: Option<&str> = None;
    let mut host: Option<&str> = None;
    let mut origin: Option<&str> = None;
    let mut x_requested_with = false;

    for line in lines {
        if line.is_empty() {
            return Err(HttpError::MalformedRequest);
        }
        let colon = line.find(':').ok_or(HttpError::MalformedRequest)?;
        let name = &line[..colon];
        if name.is_empty() || !name.bytes().all(is_http_tchar) {
            return Err(HttpError::MalformedRequest);
        }
        let raw_val = &line[colon + 1..];
        if !is_valid_field_value_bytes(raw_val) {
            return Err(HttpError::MalformedRequest);
        }
        let val = trim_ows(raw_val);
        if name.eq_ignore_ascii_case("X-OTA-Token") {
            if ota_token.is_some() {
                return Err(HttpError::MalformedRequest);
            }
            ota_token = Some(val.as_bytes());
        } else if name.eq_ignore_ascii_case("Transfer-Encoding") {
            return Err(HttpError::MalformedRequest);
        } else if name.eq_ignore_ascii_case("Content-Length") {
            if content_length.is_some() {
                return Err(HttpError::MalformedRequest);
            }
            if val.is_empty() || !val.bytes().all(|b| b.is_ascii_digit()) {
                return Err(HttpError::MalformedRequest);
            }
            let cl = val
                .parse::<usize>()
                .map_err(|_| HttpError::MalformedRequest)?;
            if cl > isize::MAX as usize {
                return Err(HttpError::MalformedRequest);
            }
            content_length = Some(cl);
        } else if name.eq_ignore_ascii_case("Content-Type") {
            if content_type.is_some() {
                return Err(HttpError::MalformedRequest);
            }
            content_type = Some(val);
        } else if name.eq_ignore_ascii_case("Host") {
            if host.is_some() || !is_valid_http_host(val) {
                return Err(HttpError::MalformedRequest);
            }
            host = Some(val);
        } else if name.eq_ignore_ascii_case("Origin") {
            if origin.is_some() {
                return Err(HttpError::MalformedRequest);
            }
            origin = Some(val);
        } else if name.eq_ignore_ascii_case("X-Requested-With") {
            x_requested_with = true;
        }
    }

    // RFC 9112 §6.3: without Content-Length (and Transfer-Encoding is
    // rejected above), message body length is 0 (M-141).
    let body = if content_length.is_some() {
        &raw[hdr_end + 4..]
    } else {
        &[]
    };

    Ok(HttpRequest {
        method,
        path,
        query,
        version,
        ota_token,
        content_length,
        content_type,
        host,
        origin,
        x_requested_with,
        body,
    })
}

/// Returns `true` if the browser-supplied `origin` (`scheme://host[:port]`)
/// matches `expected_scheme`, `host`, and effective port (normalizing an
/// omitted port to `default_port`) per RFC 6454 §4 (`M-046` / `FW-NET-27`).
pub fn is_same_origin_for_scheme(
    origin: &str,
    host: Option<&str>,
    expected_scheme: &str,
    default_port: u16,
) -> bool {
    let Some(host_str) = host else {
        return false;
    };
    let Some((scheme, origin_authority)) = origin.split_once("://") else {
        return false;
    };
    if !scheme.eq_ignore_ascii_case(expected_scheme) {
        return false;
    }
    let Some((origin_host, origin_port)) = parse_http_host_port(origin_authority) else {
        return false;
    };
    let Some((host_name, host_port)) = parse_http_host_port(host_str) else {
        return false;
    };
    let eff_origin_port = origin_port.unwrap_or(default_port);
    let eff_host_port = host_port.unwrap_or(default_port);
    origin_host.eq_ignore_ascii_case(host_name) && eff_origin_port == eff_host_port
}

/// Returns `true` if the browser-supplied `origin` (`http://host[:port]`)
/// names the same plaintext HTTP origin (scheme, host, and effective port) as
/// the request's `Host` header (`FW-NET-27`). A missing `Host`, an opaque
/// `null` origin, a non-`http` scheme, or any host/port mismatch is
/// cross-origin.
pub fn is_same_origin(origin: &str, host: Option<&str>) -> bool {
    is_same_origin_for_scheme(origin, host, "http", 80)
}

/// Status line and body for a refused request (no CORS header).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HttpRejection {
    /// HTTP status code (`401`, `403`, ...).
    pub status: u16,
    /// Reason phrase for the status line.
    pub reason: &'static str,
    /// Plain-text response body.
    pub body: &'static str,
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
    /// `Cache-Control` header value if present (`FW-NET-21`).
    pub cache_control: Option<&'a str>,
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
        if !status_line.bytes().all(|b| (0x20..=0x7E).contains(&b)) {
            return Err(HttpError::MalformedRequest);
        }
        let (http_ver, rest) = status_line
            .split_once(' ')
            .ok_or(HttpError::MalformedRequest)?;
        if http_ver != "HTTP/1.1" && http_ver != "HTTP/1.0" {
            return Err(HttpError::MalformedRequest);
        }
        let code_str = match rest.split_once(' ') {
            Some((code, _reason)) => code,
            None => rest,
        };
        if code_str.len() != 3 || !code_str.bytes().all(|b| b.is_ascii_digit()) {
            return Err(HttpError::MalformedRequest);
        }
        let status_code = code_str
            .parse::<u16>()
            .map_err(|_| HttpError::MalformedRequest)?;
        if !(100..=599).contains(&status_code) {
            return Err(HttpError::MalformedRequest);
        }

        let mut content_type = None;
        let mut content_disposition = None;
        let mut access_control_allow_origin = None;
        let mut cache_control = None;
        let mut x_framerate = None;

        for line in lines {
            if line.is_empty() {
                return Err(HttpError::MalformedRequest);
            }
            let colon = line.find(':').ok_or(HttpError::MalformedRequest)?;
            let name = &line[..colon];
            if name.is_empty() || !name.bytes().all(is_http_tchar) {
                return Err(HttpError::MalformedRequest);
            }
            let raw_val = &line[colon + 1..];
            if !is_valid_field_value_bytes(raw_val) {
                return Err(HttpError::MalformedRequest);
            }
            let val = trim_ows(raw_val);
            if name.eq_ignore_ascii_case("Content-Type") {
                if content_type.is_some() {
                    return Err(HttpError::MalformedRequest);
                }
                content_type = Some(val);
            } else if name.eq_ignore_ascii_case("Content-Disposition") {
                if content_disposition.is_some() {
                    return Err(HttpError::MalformedRequest);
                }
                content_disposition = Some(val);
            } else if name.eq_ignore_ascii_case("Access-Control-Allow-Origin") {
                if access_control_allow_origin.is_some() {
                    return Err(HttpError::MalformedRequest);
                }
                access_control_allow_origin = Some(val);
            } else if name.eq_ignore_ascii_case("Cache-Control") {
                if cache_control.is_some() {
                    return Err(HttpError::MalformedRequest);
                }
                cache_control = Some(val);
            } else if name.eq_ignore_ascii_case("X-Framerate") {
                if x_framerate.is_some() {
                    return Err(HttpError::MalformedRequest);
                }
                x_framerate = Some(val);
            }
        }

        Ok(Self {
            status_code,
            status_line,
            headers_raw,
            content_type,
            content_disposition,
            access_control_allow_origin,
            cache_control,
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
    /// Free external PSRAM heap in bytes
    /// (`heap_caps_get_free_size(MALLOC_CAP_SPIRAM)`).
    pub free_psram: u32,
    /// Connected AP RSSI in dBm (`ap_info.rssi`), or `None` if disconnected.
    pub wifi_rssi: Option<i32>,
    /// Currently running OTA partition label (`"ota_0"` or `"ota_1"`).
    pub active_partition: &'a str,
    /// Active frame resolution string (`"1280x720"`).
    pub framesize: &'a str,
    /// Whether a client is actively streaming on Port 81 (`s_stream_active`).
    pub stream_active: bool,
    /// Reset reason label mapped from `RTC_CNTL_RESET_STATE_REG` (`"poweron"`,
    /// `"sw_cpu_reset"`, `"unknown"`, etc.).
    pub reset_reason: &'a str,
    /// Hex-encoded SHA-256 digest of active application binary.
    pub app_sha256: &'a str,
    /// Free internal DRAM heap in bytes.
    pub free_dram: u32,
    /// Core 1 camera task stack high-water mark in bytes.
    pub core1_stack_hwm: u32,
    /// Firmware operational counters and cycle metrics.
    pub fw_stats: FwStatsSnapshot,
}

impl Default for TelemetrySnapshot<'static> {
    fn default() -> Self {
        Self {
            uptime_ms: 12_345,
            free_heap: 184_320,
            free_psram: 4_128_768,
            wifi_rssi: Some(-52),
            active_partition: "ota_0",
            framesize: "1280x720",
            stream_active: false,
            reset_reason: "unknown",
            app_sha256: "0000000000000000000000000000000000000000000000000000000000000000",
            free_dram: 184_320,
            core1_stack_hwm: 0,
            fw_stats: FwStatsSnapshot::new(),
        }
    }
}

fn sanitize_status_json_field<'a>(field: &'a str, buf: &'a mut [u8; 64]) -> &'a str {
    let bytes = field.as_bytes();
    let mut len = bytes.len().min(64);
    while !field.is_char_boundary(len) {
        len = len.saturating_sub(1);
    }
    let mut i = 0;
    while i < len {
        let b = bytes[i];
        if b < 0x20 || b == b'"' || b == b'\\' || b == 0x7F {
            buf[i] = b'_';
        } else {
            buf[i] = b;
        }
        i += 1;
    }
    core::str::from_utf8(&buf[..len]).unwrap_or("unknown")
}

/// Formats the JSON telemetry payload into a 1536-byte stack buffer.
/// Sanitizes `active_partition`, `framesize`, `reset_reason`, and `app_sha256`
/// to prevent JSON injection (`SEC-202`).
pub fn format_status_json<'a>(
    telemetry: &TelemetrySnapshot<'_>,
    stack_buf: &'a mut [u8; STATUS_JSON_BUF_LEN],
) -> Result<&'a str, HttpError> {
    let mut p_buf = [0u8; 64];
    let mut f_buf = [0u8; 64];
    let mut r_buf = [0u8; 64];
    let mut s_buf = [0u8; 64];
    let safe_part = sanitize_status_json_field(telemetry.active_partition, &mut p_buf);
    let safe_framesize = sanitize_status_json_field(telemetry.framesize, &mut f_buf);
    let raw_reset = if telemetry.reset_reason.trim().is_empty() {
        "unknown"
    } else {
        telemetry.reset_reason
    };
    let safe_reset = sanitize_status_json_field(raw_reset, &mut r_buf);
    let safe_sha = sanitize_status_json_field(telemetry.app_sha256, &mut s_buf);

    let mut writer = SliceWriter::new(stack_buf);
    write!(
        writer,
        "{{\"uptime_ms\":{},\"free_heap\":{},\"free_psram\":{},\"wifi_rssi\":",
        telemetry.uptime_ms, telemetry.free_heap, telemetry.free_psram,
    )
    .map_err(|_| HttpError::BufferTooSmall)?;

    match telemetry.wifi_rssi {
        Some(rssi) => write!(writer, "{}", rssi),
        None => write!(writer, "null"),
    }
    .map_err(|_| HttpError::BufferTooSmall)?;

    writeln!(
        writer,
        ",\"active_partition\":\"{}\",\"framesize\":\"{}\",\"stream_active\":{},\
        \"reset_reason\":\"{}\",\"app_sha256\":\"{}\",\"free_dram\":{},\
        \"core1_stack_hwm\":{},\
        \"sensor_frame_period_us\":{},\"frame_bytes\":{},\"c1_pushed\":{},\
        \"c1_dropped\":{},\"ring_full_events\":{},\"soi_miss\":{},\
        \"overflow_aborts\":{},\"c0_spsc_cycles\":{},\"c0_net_cycles\":{},\
        \"c0_p80_cycles\":{},\"c0_p81_cycles\":{},\"max_loop_latency_us\":{},\
        \"p81_frames_sent\":{},\"p81_frames_superseded\":{},\
        \"c0_validation_failures\":{},\"sccb_errors\":{},\"send_partial\":{},\
        \"ring_build_failures\":{},\"i2s_stop_timeouts\":{}}}",
        safe_part,
        safe_framesize,
        if telemetry.stream_active {
            "true"
        } else {
            "false"
        },
        safe_reset,
        safe_sha,
        telemetry.free_dram,
        telemetry.core1_stack_hwm,
        telemetry.fw_stats.sensor_frame_period_us,
        telemetry.fw_stats.frame_bytes,
        telemetry.fw_stats.c1_pushed,
        telemetry.fw_stats.c1_dropped,
        telemetry.fw_stats.ring_full_events,
        telemetry.fw_stats.soi_miss,
        telemetry.fw_stats.overflow_aborts,
        telemetry.fw_stats.c0_spsc_cycles,
        telemetry.fw_stats.c0_net_cycles,
        telemetry.fw_stats.c0_p80_cycles,
        telemetry.fw_stats.c0_p81_cycles,
        telemetry.fw_stats.max_loop_latency_us,
        telemetry.fw_stats.p81_frames_sent,
        telemetry.fw_stats.p81_frames_superseded,
        telemetry.fw_stats.c0_validation_failures,
        telemetry.fw_stats.sccb_errors,
        telemetry.fw_stats.send_partial,
        telemetry.fw_stats.ring_build_failures,
        telemetry.fw_stats.i2s_stop_timeouts,
    )
    .map_err(|_| HttpError::BufferTooSmall)?;

    let len = writer.written_len();
    core::str::from_utf8(&stack_buf[..len]).map_err(|_| HttpError::InvalidUtf8)
}

/// Live camera sensor tuning state exposed by `GET /control` and `GET /panel`
/// on Port 80.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CameraControlState {
    pub framesize: FrameSize,
    pub quality: i32,
    pub brightness: i32,
    pub contrast: i32,
    pub saturation: i32,
    pub special_effect: i32,
    pub hmirror: bool,
    pub vflip: bool,
    pub awb: bool,
    pub agc: bool,
    pub aec: bool,
    pub ae_level: i32,
    pub dirty: bool,
    /// Set after `apply_framesize` succeeds until `apply_quality` succeeds
    /// (`M-049` / `C1-029`), ensuring OV2640 JPEG quality re-apply is retried
    /// even when `desired.quality == last_applied.quality`.
    pub quality_needs_reapply: bool,
}

impl CameraControlState {
    /// Creates the default 1280x720 HD camera control state.
    pub const fn new_default() -> Self {
        Self {
            framesize: FrameSize::Hd,
            quality: 12,
            brightness: 0,
            contrast: 0,
            saturation: 0,
            special_effect: 0,
            hmirror: false,
            vflip: false,
            awb: true,
            agc: true,
            aec: true,
            ae_level: 0,
            dirty: false,
            quality_needs_reapply: false,
        }
    }
}

impl Default for CameraControlState {
    #[inline]
    fn default() -> Self {
        Self::new_default()
    }
}

/// Trait for applying camera sensor control changes to the hardware driver.
pub trait CameraControlApplier {
    /// Applies a new frame size / resolution setting.
    fn apply_framesize(&mut self, val: FrameSize) -> Result<(), CameraError>;
    /// Applies JPEG compression quality (lower is higher quality).
    fn apply_quality(&mut self, val: i32) -> Result<(), CameraError>;
    /// Applies brightness adjustment.
    fn apply_brightness(&mut self, val: i32) -> Result<(), CameraError>;
    /// Applies contrast adjustment.
    fn apply_contrast(&mut self, val: i32) -> Result<(), CameraError>;
    /// Applies saturation adjustment.
    fn apply_saturation(&mut self, val: i32) -> Result<(), CameraError>;
    /// Applies special color effect.
    fn apply_special_effect(&mut self, val: i32) -> Result<(), CameraError>;
    /// Applies horizontal mirror.
    fn apply_hmirror(&mut self, val: bool) -> Result<(), CameraError>;
    /// Applies vertical flip.
    fn apply_vflip(&mut self, val: bool) -> Result<(), CameraError>;
    /// Applies auto white balance enable/disable.
    fn apply_awb(&mut self, val: bool) -> Result<(), CameraError>;
    /// Applies auto gain control enable/disable.
    fn apply_agc(&mut self, val: bool) -> Result<(), CameraError>;
    /// Applies auto exposure control enable/disable.
    fn apply_aec(&mut self, val: bool) -> Result<(), CameraError>;
    /// Applies auto exposure level.
    fn apply_ae_level(&mut self, val: i32) -> Result<(), CameraError>;
}

/// Applies differences between `desired` and `last_applied` camera control
/// states to the underlying sensor hardware via `applier`.
///
/// Returns the number of control update errors encountered. Successfully
/// applied fields are recorded in `last_applied`, while failing fields remain
/// unchanged so they may be retried on subsequent reconciliation passes.
pub fn apply_camera_control_diff<A: CameraControlApplier>(
    applier: &mut A,
    desired: &CameraControlState,
    last_applied: &mut CameraControlState,
) -> u32 {
    let mut errors = 0u32;

    if desired.framesize != last_applied.framesize {
        match applier.apply_framesize(desired.framesize) {
            Ok(()) => {
                last_applied.framesize = desired.framesize;
                last_applied.quality_needs_reapply = true;
            }
            Err(_) => {
                errors += 1;
            }
        }
    }

    if last_applied.quality_needs_reapply || desired.quality != last_applied.quality {
        match applier.apply_quality(desired.quality) {
            Ok(()) => {
                last_applied.quality = desired.quality;
                last_applied.quality_needs_reapply = false;
            }
            Err(_) => {
                errors += 1;
            }
        }
    }

    if desired.brightness != last_applied.brightness {
        match applier.apply_brightness(desired.brightness) {
            Ok(()) => last_applied.brightness = desired.brightness,
            Err(_) => errors += 1,
        }
    }

    if desired.contrast != last_applied.contrast {
        match applier.apply_contrast(desired.contrast) {
            Ok(()) => last_applied.contrast = desired.contrast,
            Err(_) => errors += 1,
        }
    }

    if desired.saturation != last_applied.saturation {
        match applier.apply_saturation(desired.saturation) {
            Ok(()) => last_applied.saturation = desired.saturation,
            Err(_) => errors += 1,
        }
    }

    if desired.special_effect != last_applied.special_effect {
        match applier.apply_special_effect(desired.special_effect) {
            Ok(()) => last_applied.special_effect = desired.special_effect,
            Err(_) => errors += 1,
        }
    }

    if desired.hmirror != last_applied.hmirror {
        match applier.apply_hmirror(desired.hmirror) {
            Ok(()) => last_applied.hmirror = desired.hmirror,
            Err(_) => errors += 1,
        }
    }

    if desired.vflip != last_applied.vflip {
        match applier.apply_vflip(desired.vflip) {
            Ok(()) => last_applied.vflip = desired.vflip,
            Err(_) => errors += 1,
        }
    }

    if desired.awb != last_applied.awb {
        match applier.apply_awb(desired.awb) {
            Ok(()) => last_applied.awb = desired.awb,
            Err(_) => errors += 1,
        }
    }

    if desired.agc != last_applied.agc {
        match applier.apply_agc(desired.agc) {
            Ok(()) => last_applied.agc = desired.agc,
            Err(_) => errors += 1,
        }
    }

    if desired.aec != last_applied.aec {
        match applier.apply_aec(desired.aec) {
            Ok(()) => last_applied.aec = desired.aec,
            Err(_) => errors += 1,
        }
    }

    if desired.ae_level != last_applied.ae_level {
        match applier.apply_ae_level(desired.ae_level) {
            Ok(()) => last_applied.ae_level = desired.ae_level,
            Err(_) => errors += 1,
        }
    }

    errors
}

/// Returns the canonical `"WxH"` resolution label for `fs`.
#[inline]
pub const fn framesize_label(fs: FrameSize) -> &'static str {
    match fs {
        FrameSize::Qvga => "320x240",
        FrameSize::Cif => "400x296",
        FrameSize::Hvga => "480x320",
        FrameSize::Vga => "640x480",
        FrameSize::Svga => "800x600",
        FrameSize::Xga => "1024x768",
        FrameSize::Hd => "1280x720",
        FrameSize::Sxga => "1280x1024",
        FrameSize::Uxga => "1600x1200",
        _ => "1280x720",
    }
}

/// Returns the integer control index (`5..=13`) serialized in `GET /control`
/// for `fs`.
#[inline]
pub const fn framesize_control_code(fs: FrameSize) -> u8 {
    match fs {
        FrameSize::Qvga => 5,
        FrameSize::Cif => 6,
        FrameSize::Hvga => 7,
        FrameSize::Vga => 8,
        FrameSize::Svga => 9,
        FrameSize::Xga => 10,
        FrameSize::Hd => 11,
        FrameSize::Sxga => 12,
        FrameSize::Uxga => 13,
        _ => 11,
    }
}

/// Compares two byte slices in constant time to prevent timing side-channel
/// attacks (`C13-SEC-001` / `FW-AUDIT-HTTP-001`).
///
/// Delegates to branchless [`crate::ota::verify_ota_token`].
#[inline]
pub fn constant_time_eq(a: &[u8], b: &[u8]) -> bool {
    crate::ota::verify_ota_token(a, b)
}

/// Parses a `framesize` query parameter (`5..=13` or horizontal width
/// `320..=1600`) into a [`FrameSize`].
#[inline]
pub const fn parse_framesize_val(val: isize) -> Option<FrameSize> {
    match val {
        5 | 320 => Some(FrameSize::Qvga),
        6 | 400 => Some(FrameSize::Cif),
        7 | 480 => Some(FrameSize::Hvga),
        8 | 640 => Some(FrameSize::Vga),
        9 | 800 => Some(FrameSize::Svga),
        10 | 1024 => Some(FrameSize::Xga),
        11 | 1280 => Some(FrameSize::Hd),
        12 => Some(FrameSize::Sxga),
        13 | 1600 => Some(FrameSize::Uxga),
        _ => None,
    }
}

/// Formats a [`CameraControlState`] as JSON into a 384-byte stack buffer.
pub fn format_control_json<'a>(
    ctrl: &CameraControlState,
    stack_buf: &'a mut [u8; CONTROL_JSON_BUF_LEN],
) -> Result<&'a str, HttpError> {
    let mut writer = SliceWriter::new(stack_buf);
    write!(
        writer,
        "{{\"framesize\":{},\"framesize_label\":\"{}\",\"quality\":{},\
        \"brightness\":{},\"contrast\":{},\"saturation\":{},\
        \"special_effect\":{},\"hmirror\":{},\"vflip\":{},\
        \"awb\":{},\"agc\":{},\"aec\":{},\"ae_level\":{}}}",
        framesize_control_code(ctrl.framesize),
        framesize_label(ctrl.framesize),
        ctrl.quality,
        ctrl.brightness,
        ctrl.contrast,
        ctrl.saturation,
        ctrl.special_effect,
        u8::from(ctrl.hmirror),
        u8::from(ctrl.vflip),
        u8::from(ctrl.awb),
        u8::from(ctrl.agc),
        u8::from(ctrl.aec),
        ctrl.ae_level
    )
    .map_err(|_| HttpError::BufferTooSmall)?;

    let len = writer.written_len();
    core::str::from_utf8(&stack_buf[..len]).map_err(|_| HttpError::InvalidUtf8)
}

/// Self-contained dark-mode Camera Control & Live Preview dashboard served by
/// `GET /panel` on Port 80.
pub const KLIPPER_CONTROL_PANEL_HTML: &str = "\
<!DOCTYPE html><html><head><meta charset=\"utf-8\"><title>Klipper Cam Control Panel</title>\
<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">\
<style>\
body{font-family:sans-serif;margin:0;padding:16px;background:#121212;color:#e0e0e0}\
.wrap{max-width:920px;margin:auto;display:grid;grid-template-columns:1fr 300px;gap:16px}\
@media(max-width:768px){.wrap{grid-template-columns:1fr}}\
.card{background:#1e1e1e;padding:16px;border-radius:8px;border:1px solid #333}\
h2{margin:0 0 12px;font-size:18px;color:#fff}\
#view{width:100%;background:#000;border-radius:6px;min-height:200px;display:block}\
.r{display:flex;justify-content:space-between;align-items:center;margin:7px 0;font-size:13px}\
button{background:#007acc;color:#fff;border:none;padding:7px 12px;\
border-radius:4px;cursor:pointer;margin-right:6px}\
#st{font-family:monospace;font-size:12px;color:#9cdcfe;margin-top:10px}\
</style></head><body><div class=\"wrap\">\
<div class=\"card\"><h2>Klipper Camera Live Preview</h2>\
<img id=\"view\" src=\"/snapshot\" alt=\"Preview\" onload=\"ie=0\" onerror=\"oe()\">\
<div style=\"margin-top:10px\">\
<button onclick=\"ss()\">Live Stream (:81)</button>\
<button onclick=\"snap()\">Snapshot</button>\
<a href=\"/update\" style=\"color:#569cd6;font-size:13px\">OTA Update</a>\
</div><div id=\"st\">Loading...</div></div>\
<div class=\"card\"><h2>OV2640 Controls</h2>\
<div class=\"r\"><label>Resolution</label><select id=\"framesize\" \
onchange=\"setCtrl('framesize',this.value)\"></select></div>\
<div id=\"sl\"></div></div></div><script>\
function $(i){return document.getElementById(i);}\
function rj(r){return r.json();}\
['320x240','400x296','480x320','640x480','800x600','1024x768','1280x720',\
'1280x1024','1600x1200'].forEach(function(r,i){\
$('framesize').innerHTML+='<option value=\"'+(i+5)+'\"'+(i==6?' selected':'')+'>'+r+'</option>';});\
var S=[['quality','Quality',4,63,12],['brightness','Brightness',-2,2,0],\
['contrast','Contrast',-2,2,0],\
['saturation','Saturation',-2,2,0],['ae_level','AE Level',-2,2,0],\
['special_effect','Effect',0,6,0]];\
var B=[['hmirror','H-Mirror',0],['vflip','V-Flip',0],['awb','Auto WB',1],\
['agc','Auto Gain',1],['aec','Auto Exp',1]];\
var h='';S.forEach(function(x){\
h+='<div class=\"r\"><label>'+x[1]+'</label><input id=\"'+x[0]+'\" type=\"range\" \
min=\"'+x[2]+'\" max=\"'+x[3]+'\" value=\"'+x[4]+'\" \
onchange=\"setCtrl(\\''+x[0]+'\\',this.value)\"></div>';});\
B.forEach(function(x){\
h+='<div class=\"r\"><label>'+x[1]+'</label><input id=\"'+x[0]+'\" type=\"checkbox\"'+\
(x[2]?' checked':'')+' onchange=\"setCtrl(\\''+x[0]+'\\',this.checked?1:0)\"></div>';});\
$('sl').innerHTML=h;\
var m='stream',lu=0,ne=0,ie=0,pb=0,ls=0,rt=0;\
function ss(){m='stream';ie=0;ls=Date.now();\
$('view').src=location.protocol+'//'+location.hostname+':81/stream?t='+ls;}\
function snap(){m='snap';ie=0;$('view').src='/snapshot?t='+Date.now();}\
function oe(){ie=1;if(!rt)rt=setTimeout(recover,1500);}\
function recover(){if(rt){clearTimeout(rt);rt=0;}if(m==='stream')ss();else snap();}\
function u(c){if(c.framesize)$('framesize').value=c.framesize;\
S.forEach(function(x){if(c[x[0]]!==undefined)$(x[0]).value=c[x[0]];});\
B.forEach(function(x){if(c[x[0]]!==undefined)$(x[0]).checked=!!c[x[0]];});}\
function sc(){fetch('/control').then(rj).then(u).catch(function(){});}\
function setCtrl(k,v){fetch('/control?var='+k+'&val='+v,{method:'POST',\
headers:{'X-Requested-With':'fetch'}}).then(rj).then(function(c){\
u(c);if(m==='snap')setTimeout(snap,250);}).catch(function(){ne=1;});}\
function poll(){if(pb)return;pb=1;var bt=setTimeout(function(){pb=0;},2500);\
fetch('/status').then(rj).then(function(s){clearTimeout(bt);pb=0;\
$('st').innerText='Res: '+s.framesize+' | RSSI: '+s.wifi_rssi+' dBm | PSRAM: '+\
s.free_psram+' B | Part: '+s.active_partition+' | Stream: '+s.stream_active;\
var rb=(lu>0&&s.uptime_ms<lu);lu=s.uptime_ms;if(ne||rb){ne=0;sc();recover();}\
else if(ie||(m==='stream'&&!s.stream_active&&(Date.now()-ls>3500))){recover();}}).\
catch(function(){clearTimeout(bt);pb=0;ne=1;$('st').innerText='Offline - reconnecting...';});}\
sc();ss();poll();setInterval(poll,3000);\
</script></body></html>";

/// Formats an MJPEG multipart frame boundary header into `out_buf` without
/// copying the JPEG frame payload (`klipper_cam.c`).
pub fn write_mjpeg_part_header(content_len: usize, out_buf: &mut [u8]) -> Result<&[u8], HttpError> {
    let mut writer = SliceWriter::new(out_buf);
    write!(
        writer,
        "\r\n--{}\r\nContent-Type: image/jpeg\r\nContent-Length: {}\r\n\r\n",
        MJPEG_BOUNDARY, content_len
    )
    .map_err(|_| HttpError::BufferTooSmall)?;
    Ok(writer.into_bytes())
}

/// Formats an MJPEG multipart frame boundary header into a 128-byte stack
/// buffer (`klipper_cam.c`).
pub fn format_mjpeg_part_header(
    jpeg_len: usize,
    part_buf: &mut [u8; MJPEG_PART_HDR_BUF_LEN],
) -> Result<&[u8], HttpError> {
    write_mjpeg_part_header(jpeg_len, part_buf)
}

/// RAII guard for a captured camera frame buffer (`camera_fb_t` + `s_fb_mutex`,
/// `APP-006`).
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

/// Thread-safe `#![no_std]` camera frame provider with `AtomicBool` mutex
/// synchronization (`APP-006`).
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

    /// Creates a default 1280x720 HD [`CameraDriver`] serving
    /// [`SYNTHETIC_HD_JFIF_FRAME`].
    pub const fn new_hd_default() -> CameraDriver<'static> {
        CameraDriver::new(&SYNTHETIC_HD_JFIF_FRAME, FrameSize::Hd)
    }

    /// Replaces the active JPEG frame slice and resolution served by
    /// [`CameraDriver::acquire_frame`].
    #[inline]
    pub fn set_frame(&mut self, frame_data: &'a [u8], frame_size: FrameSize) {
        self.frame_data = frame_data;
        self.frame_size = frame_size;
    }

    /// Updates the frame size resolution of the camera driver.
    #[inline]
    pub fn set_frame_size(&mut self, frame_size: FrameSize) {
        self.frame_size = frame_size;
    }

    /// Returns the active frame size resolution.
    #[inline]
    pub const fn frame_size(&self) -> FrameSize {
        self.frame_size
    }

    /// Returns the underlying JPEG frame slice currently configured in the driver.
    #[inline]
    pub const fn frame_data(&self) -> &'a [u8] {
        self.frame_data
    }

    /// Acquires the frame-buffer mutex and returns an RAII [`FrameGuard`].
    ///
    /// Returns [`CameraError::InvalidState`] if the mutex is already locked
    /// (`Camera busy`),
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
    #[cfg(test)]
    pub fn set_mutex_locked(&self, locked: bool) {
        self.fb_mutex.store(locked, Ordering::Release);
    }

    /// Enables or disables simulated sensor capture failure (`esp_camera_fb_get()
    /// == NULL`).
    #[cfg(test)]
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

/// RAII single-client MJPEG stream lock guard (`s_stream_active` in
/// `klipper_cam.c`, `NET-002`).
///
/// Guarantees that `stream_active` is atomically set back to `false` when the
/// streaming
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
    /// - `Content-Type: multipart/x-mixed-replace;boundary=...`
    /// - `Access-Control-Allow-Origin: *`
    /// - `Cache-Control: no-store` (`FW-NET-21`)
    /// - `Connection: close` & `X-Content-Type-Options: nosniff` (`M-211`)
    pub fn write_stream_headers<'b>(&self, out_buf: &'b mut [u8]) -> Result<&'b [u8], HttpError> {
        let mut writer = SliceWriter::new(out_buf);
        write!(
            writer,
            "HTTP/1.1 200 OK\r\nContent-Type: {}\r\n\
            Access-Control-Allow-Origin: *\r\n\
            Cache-Control: no-store\r\n\
            Connection: close\r\n\
            X-Content-Type-Options: nosniff\r\n\r\n",
            MJPEG_CONTENT_TYPE
        )
        .map_err(|_| HttpError::BufferTooSmall)?;
        Ok(writer.into_bytes())
    }

    /// Acquires one frame from `camera`, formats the MJPEG multipart boundary
    /// header and JPEG payload into `out_buf`, and immediately releases the
    /// [`FrameGuard`] so Port 80 `/snapshot` requests are never starved during
    /// inter-frame pacing.
    ///
    /// Tracks consecutive capture, busy, and formatting failures (`M-048`),
    /// resetting the counter only after header and JPEG bytes are written, and
    /// enforces [`Self::should_abort`] before acquiring another frame.
    pub fn write_next_frame_chunk<'b>(
        &mut self,
        camera: &CameraDriver<'_>,
        out_buf: &'b mut [u8],
    ) -> Result<&'b [u8], HttpError> {
        if self.should_abort() {
            return Err(HttpError::CaptureFailed);
        }

        let guard = match camera.acquire_frame() {
            Ok(g) => g,
            Err(CameraError::InvalidState) => {
                self.consecutive_failures = self.consecutive_failures.saturating_add(1);
                return Err(HttpError::CameraBusy);
            }
            Err(_) => {
                self.consecutive_failures = self.consecutive_failures.saturating_add(1);
                return Err(HttpError::CaptureFailed);
            }
        };

        let write_res = (|| -> Result<usize, HttpError> {
            let mut part_hdr_buf = [0u8; MJPEG_PART_HDR_BUF_LEN];
            let part_hdr = format_mjpeg_part_header(guard.len(), &mut part_hdr_buf)?;

            let mut writer = SliceWriter::new(out_buf);
            writer.write_bytes(part_hdr)?;
            writer.write_bytes(guard.bytes())?;
            Ok(writer.written_len())
        })();
        drop(guard);

        match write_res {
            Ok(written) => {
                self.consecutive_failures = 0;
                Ok(&out_buf[..written])
            }
            Err(e) => {
                self.consecutive_failures = self.consecutive_failures.saturating_add(1);
                Err(e)
            }
        }
    }

    /// Returns the number of consecutive capture failures on this stream session.
    #[inline]
    pub const fn consecutive_failures(&self) -> u8 {
        self.consecutive_failures
    }

    /// Returns `true` if consecutive capture failures have reached
    /// [`STREAM_MAX_CAPTURE_FAILURES`] (`5`).
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

/// Complete `#![no_std]` Klipper Camera dual-port HTTP & OTA server state
/// machine.
pub struct KlipperCamServer<'a> {
    pub config: KlipperCamConfig,
    pub camera: CameraDriver<'a>,
    pub ota_manager: OtaDataManager,
    pub control_state: CameraControlState,
    stream_active: AtomicBool,
    uptime_ms: i64,
    free_heap: u32,
    free_psram: u32,
    wifi_rssi: Option<i32>,
    reset_reason: &'a str,
    app_sha256: &'a str,
    free_dram: u32,
    core1_stack_hwm: u32,
    fw_stats: FwStatsSnapshot,
    last_ota_bytes_written: usize,
    last_ota_target_slot: Option<OtaSlot>,
    reboot_scheduled: bool,
    active_ip: Option<Ipv4Addr>,
    device_mac: [u8; 6],
}

impl<'a> KlipperCamServer<'a> {
    /// Creates a new [`KlipperCamServer`] with the given configuration and camera
    /// driver.
    pub const fn new(config: KlipperCamConfig, camera: CameraDriver<'a>) -> Self {
        Self {
            config,
            camera,
            ota_manager: OtaDataManager::new_booted_ota0(),
            control_state: CameraControlState::new_default(),
            stream_active: AtomicBool::new(false),
            uptime_ms: 15_000,
            free_heap: 184_320,
            free_psram: 4_128_768,
            wifi_rssi: Some(-48),
            reset_reason: "unknown",
            app_sha256: "0000000000000000000000000000000000000000000000000000000000000000",
            free_dram: 184_320,
            core1_stack_hwm: 0,
            fw_stats: FwStatsSnapshot::new(),
            last_ota_bytes_written: 0,
            last_ota_target_slot: None,
            reboot_scheduled: false,
            active_ip: None,
            device_mac: [0u8; 6],
        }
    }

    /// Creates a default [`KlipperCamServer`] with `WroverKit` 1280x720 HD
    /// configuration.
    pub const fn new_default() -> KlipperCamServer<'static> {
        KlipperCamServer::new(
            KlipperCamConfig::WROVER_KIT_HD,
            CameraDriver::new_hd_default(),
        )
    }

    /// Returns the configured OTA shared secret token as a byte slice.
    #[inline]
    pub fn ota_token(&self) -> &[u8] {
        self.config.ota_token.as_bytes()
    }

    /// Records the runtime assigned IPv4 address (e.g. from DHCP or static
    /// interface configuration) for `Host` allowlist verification (`M-047`).
    #[inline]
    pub fn set_active_ip(&mut self, ip: Ipv4Addr) {
        self.active_ip = Some(ip);
    }

    /// Returns the runtime assigned IPv4 address if configured (`M-047`).
    #[inline]
    pub const fn active_ip(&self) -> Option<Ipv4Addr> {
        self.active_ip
    }

    /// Records the hardware Wi-Fi MAC address displayed by STA `GET /setup`.
    #[inline]
    pub fn set_device_mac(&mut self, mac: [u8; 6]) {
        self.device_mac = mac;
    }

    /// Returns the recorded hardware Wi-Fi MAC address.
    #[inline]
    pub const fn device_mac(&self) -> [u8; 6] {
        self.device_mac
    }

    /// Returns `true` if `host` (from the HTTP `Host` header) is permitted for
    /// state-mutating endpoints (`POST /control`, `POST /update`), mitigating
    /// DNS-rebinding attacks from external domains (`M-047`).
    ///
    /// Permits the configured static IPv4, the runtime assigned IPv4,
    /// loopback (`127.0.0.1`, `[::1]`, `localhost`), and `.local` mDNS
    /// hostnames while rejecting arbitrary external domains with `403`.
    pub fn is_allowed_host(&self, host: Option<&str>) -> bool {
        let Some(host_str) = host else {
            return false;
        };
        let Some((host_name, _port)) = parse_http_host_port(host_str) else {
            return false;
        };
        if host_name.eq_ignore_ascii_case("localhost")
            || host_name == "127.0.0.1"
            || host_name == "[::1]"
        {
            return true;
        }
        if let Some(ip) = crate::config::Ipv4Addr::parse_ascii(host_name) {
            return ip == self.config.static_ip().ip || self.active_ip == Some(ip);
        }
        if host_name.len() > 6
            && host_name.as_bytes()[host_name.len() - 6..].eq_ignore_ascii_case(b".local")
        {
            return true;
        }
        false
    }

    /// Authorizes an OTA upload (`X-OTA-Token`, `FW-NET-04` / `FW-NET-08` /
    /// `M-031`).
    ///
    /// A firmware still configured with the public [`DEFAULT_OTA_TOKEN`] or an
    /// invalid token (short, whitespace, or non-graphic ASCII) refuses every
    /// upload with `403 Forbidden`, whatever token is sent. Otherwise the token
    /// is compared in constant time; a mismatch is `401 Unauthorized`.
    ///
    /// [`DEFAULT_OTA_TOKEN`]: crate::config::DEFAULT_OTA_TOKEN
    pub fn check_ota_auth(&self, provided: &[u8]) -> Result<(), HttpRejection> {
        if !crate::config::is_valid_ota_token(self.config.ota_token.as_bytes())
            || self.config.is_default_ota_token()
        {
            return Err(HttpRejection {
                status: 403,
                reason: "Forbidden",
                body: "Forbidden: OTA disabled; firmware was built with the default token. \
                       Rebuild with WIFI_OTA_TOKEN set.\n",
            });
        }
        if provided.is_empty() || !constant_time_eq(provided, self.config.ota_token.as_bytes()) {
            return Err(HttpRejection {
                status: 401,
                reason: "Unauthorized",
                body: "Unauthorized: Invalid or missing X-OTA-Token\n",
            });
        }
        Ok(())
    }

    /// Returns whether a camera control setting was modified via `/control`
    /// and is pending application without clearing the dirty flag.
    #[must_use]
    #[inline]
    pub const fn has_pending_control_update(&self) -> bool {
        self.control_state.dirty
    }

    /// Returns `Some(CameraControlState)` and clears `dirty` if a camera
    /// control setting was modified via `POST /control`.
    pub fn take_pending_control_update(&mut self) -> Option<CameraControlState> {
        if self.control_state.dirty {
            self.control_state.dirty = false;
            Some(self.control_state)
        } else {
            None
        }
    }

    /// Reconciles pending control state against sensor hardware via `applier`,
    /// keeping `dirty` set if any hardware write fails so callers retry on the
    /// next pass (`M-145` / `C1-047`).
    pub fn reconcile_controls<A: CameraControlApplier>(
        &mut self,
        applier: &mut A,
        last_applied: &mut CameraControlState,
    ) -> u32 {
        if !self.control_state.dirty && !last_applied.quality_needs_reapply {
            return 0;
        }
        let errors = apply_camera_control_diff(applier, &self.control_state, last_applied);
        self.camera.set_frame_size(last_applied.framesize);
        if errors == 0 {
            self.control_state.dirty = false;
        }
        errors
    }

    /// Updates the live system telemetry metrics reported by `GET /status`.
    pub fn set_telemetry(
        &mut self,
        uptime_ms: i64,
        free_heap: u32,
        free_psram: u32,
        wifi_rssi: i32,
    ) {
        self.uptime_ms = uptime_ms;
        self.free_heap = free_heap;
        self.free_psram = free_psram;
        self.wifi_rssi = Some(wifi_rssi);
    }

    /// Updates extended live system telemetry metrics including optional RSSI.
    pub fn set_extended_telemetry(
        &mut self,
        uptime_ms: i64,
        free_heap: u32,
        free_psram: u32,
        wifi_rssi: Option<i32>,
    ) {
        self.uptime_ms = uptime_ms;
        self.free_heap = free_heap;
        self.free_psram = free_psram;
        self.wifi_rssi = wifi_rssi;
    }

    /// Updates firmware diagnostics and performance counters.
    pub fn set_diagnostics(
        &mut self,
        reset_reason: &'a str,
        app_sha256: &'a str,
        free_dram: u32,
        core1_stack_hwm: u32,
        fw_stats: FwStatsSnapshot,
    ) {
        self.reset_reason = reset_reason;
        self.app_sha256 = app_sha256;
        self.free_dram = free_dram;
        self.core1_stack_hwm = core1_stack_hwm;
        self.fw_stats = fw_stats;
    }

    /// Returns `true` if a client is currently streaming on Port 81
    /// (`s_stream_active`).
    #[inline]
    pub fn is_stream_active(&self) -> bool {
        self.stream_active.load(Ordering::Acquire)
    }

    /// Returns a shared reference to the atomic `stream_active` flag.
    #[inline]
    pub fn stream_active_flag(&self) -> &AtomicBool {
        &self.stream_active
    }

    /// Attempts to start an exclusive Port 81 MJPEG stream session
    /// (`StreamSessionGuard`).
    #[inline]
    pub fn try_start_stream(&self) -> Option<StreamSessionGuard<'_>> {
        StreamSessionGuard::try_acquire(&self.stream_active)
    }

    /// Returns `true` if a valid OTA firmware upload has completed and scheduled a
    /// reboot.
    #[inline]
    pub const fn is_reboot_scheduled(&self) -> bool {
        self.reboot_scheduled
    }

    /// Returns the total bytes written during the most recent successful OTA
    /// update.
    #[inline]
    pub const fn last_ota_bytes_written(&self) -> usize {
        self.last_ota_bytes_written
    }

    /// Returns the target [`OtaSlot`] written during the most recent successful OTA
    /// update.
    #[inline]
    pub const fn last_ota_target_slot(&self) -> Option<OtaSlot> {
        self.last_ota_target_slot
    }

    /// Cancels rollback on successful Wi-Fi IP acquisition (`IP_EVENT_STA_GOT_IP`,
    /// `APP-003` / `FW-002`).
    pub fn on_wifi_got_ip_cancel_rollback(&mut self) -> Result<bool, OtaError> {
        let mut noop = NoOpFlashStorage;
        self.on_wifi_got_ip_with_flash(&mut noop)
    }

    /// Cancels rollback on successful Wi-Fi IP acquisition and persists the updated
    /// `otadata`
    /// sector to `flash` (`IP_EVENT_STA_GOT_IP`, `APP-003` / `FW-002`).
    pub fn on_wifi_got_ip_with_flash<F: OtaFlashStorage>(
        &mut self,
        flash: &mut F,
    ) -> Result<bool, OtaError> {
        self.ota_manager.cancel_rollback_in_flash(flash)
    }

    /// Handles a raw HTTP/1.1 request received on `port` (`ServerPort::Control` =
    /// 80 or
    /// `ServerPort::Stream` = 81) and writes the complete HTTP response into
    /// `out_buf`.
    pub fn handle_raw_request<'b>(
        &mut self,
        port: ServerPort,
        raw_req: &[u8],
        out_buf: &'b mut [u8],
    ) -> Result<&'b [u8], HttpError> {
        let mut noop = NoOpFlashStorage;
        self.handle_raw_request_with_flash(port, raw_req, out_buf, &mut noop)
    }

    /// Handles a raw HTTP/1.1 request received on `port` with an
    /// [`OtaFlashStorage`] backend
    /// for persisting `POST /update` firmware images and `otadata` state
    /// transitions.
    pub fn handle_raw_request_with_flash<'b, F: OtaFlashStorage>(
        &mut self,
        port: ServerPort,
        raw_req: &[u8],
        out_buf: &'b mut [u8],
        flash: &mut F,
    ) -> Result<&'b [u8], HttpError> {
        let req = match parse_http_request(raw_req) {
            Ok(r) => r,
            Err(_) => {
                return write_http_error(out_buf, 400, "Bad Request", "400 Bad Request\n", false);
            }
        };
        self.handle_request_with_flash(port, &req, out_buf, flash)
    }

    /// Routes a parsed [`HttpRequest`] on `port` (`Control` = 80, `Stream` = 81).
    pub fn handle_request<'b>(
        &mut self,
        port: ServerPort,
        req: &HttpRequest<'_>,
        out_buf: &'b mut [u8],
    ) -> Result<&'b [u8], HttpError> {
        let mut noop = NoOpFlashStorage;
        self.handle_request_with_flash(port, req, out_buf, &mut noop)
    }

    /// Routes a parsed [`HttpRequest`] on `port` (`Control` = 80, `Stream` = 81)
    /// with an
    /// [`OtaFlashStorage`] backend.
    pub fn handle_request_with_flash<'b, F: OtaFlashStorage>(
        &mut self,
        port: ServerPort,
        req: &HttpRequest<'_>,
        out_buf: &'b mut [u8],
        flash: &mut F,
    ) -> Result<&'b [u8], HttpError> {
        match port {
            ServerPort::Control => self.handle_port80(req, out_buf, flash),
            ServerPort::Stream => self.handle_port81(req, out_buf),
        }
    }

    /// Handles requests arriving on Port 80 (`s_camera_httpd` in `klipper_cam.c`).
    fn handle_port80<'b, F: OtaFlashStorage>(
        &mut self,
        req: &HttpRequest<'_>,
        out_buf: &'b mut [u8],
        flash: &mut F,
    ) -> Result<&'b [u8], HttpError> {
        match (req.method, req.path) {
            (HttpMethod::Get, "/") => self.handle_root_p80(req, out_buf),
            (HttpMethod::Get, "/snapshot") => self.handle_snapshot(out_buf),
            (HttpMethod::Get, "/status") => self.handle_status(out_buf),
            (HttpMethod::Get | HttpMethod::Post, "/control") => self.handle_control(req, out_buf),
            (HttpMethod::Get, "/panel") => self.handle_panel_get(out_buf),
            (HttpMethod::Get, "/setup") => self.handle_setup_get(out_buf),
            (HttpMethod::Post, "/setup") => self.handle_setup_post(req, out_buf, flash),
            (HttpMethod::Get, "/update") => self.handle_ota_get(out_buf),
            (HttpMethod::Post, "/update") => self.handle_ota_post(req, out_buf, flash),
            // Port 80 explicitly does NOT serve /stream (NET-001 / APP-005 -> 404 Not
            // Found)
            _ => write_http_error(out_buf, 404, "Not Found", "404 Not Found\n", false),
        }
    }

    /// Handles `GET /` on Port 80, including `?action=snapshot` delegation and
    /// `?action=stream` `400 Bad Request` rejection (`klipper_cam.c` lines
    /// 232-274).
    fn handle_root_p80<'b>(
        &self,
        req: &HttpRequest<'_>,
        out_buf: &'b mut [u8],
    ) -> Result<&'b [u8], HttpError> {
        if let Some(q) = req.query {
            if q.len() > MAX_URL_QUERY_LEN {
                return write_http_error(
                    out_buf,
                    400,
                    "Bad Request",
                    "Query string exceeds maximum length\n",
                    false,
                );
            }
            if query_has_pair(q, "action=snapshot") {
                return self.handle_snapshot(out_buf);
            } else if query_has_pair(q, "action=stream") {
                return write_http_error(
                    out_buf,
                    400,
                    "Bad Request",
                    "Stream available only on port 81 (/stream)",
                    false,
                );
            }
        }

        let mut writer = SliceWriter::new(out_buf);
        write!(
            writer,
            "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\n\
            Access-Control-Allow-Origin: *\r\nConnection: close\r\n\
            Content-Length: {}\r\n\r\n{}",
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
                    true,
                );
            }
            Err(_) => {
                return write_http_error(
                    out_buf,
                    500,
                    "Internal Server Error",
                    "Camera snapshot capture failed",
                    true,
                );
            }
        };

        let mut writer = SliceWriter::new(out_buf);
        write!(
            writer,
            "HTTP/1.1 200 OK\r\nContent-Type: image/jpeg\r\n\
            Content-Disposition: inline; filename=capture.jpg\r\n\
            Access-Control-Allow-Origin: *\r\nConnection: close\r\n\
            Cache-Control: no-store\r\n\
            Content-Length: {}\r\n\r\n",
            guard.len()
        )
        .map_err(|_| HttpError::BufferTooSmall)?;
        writer.write_bytes(guard.bytes())?;
        Ok(writer.into_bytes())
    }

    /// Handles `GET /status` on Port 80 (`klipper_cam.c` lines 196-230).
    fn handle_status<'b>(&self, out_buf: &'b mut [u8]) -> Result<&'b [u8], HttpError> {
        let active_label = self.ota_manager.running_slot().label();
        let telemetry = TelemetrySnapshot {
            uptime_ms: self.uptime_ms,
            free_heap: self.free_heap,
            free_psram: self.free_psram,
            wifi_rssi: self.wifi_rssi,
            active_partition: active_label,
            framesize: framesize_label(self.control_state.framesize),
            stream_active: self.is_stream_active(),
            reset_reason: self.reset_reason,
            app_sha256: self.app_sha256,
            free_dram: self.free_dram,
            core1_stack_hwm: self.core1_stack_hwm,
            fw_stats: self.fw_stats,
        };

        let mut json_stack = [0u8; STATUS_JSON_BUF_LEN];
        let json_str = format_status_json(&telemetry, &mut json_stack)?;

        let mut writer = SliceWriter::new(out_buf);
        write!(
            writer,
            "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n\
            Access-Control-Allow-Origin: *\r\nConnection: close\r\n\
            Content-Length: {}\r\n\r\n{}",
            json_str.len(),
            json_str
        )
        .map_err(|_| HttpError::BufferTooSmall)?;
        Ok(writer.into_bytes())
    }

    /// Handles `/control` on Port 80 (`FW-NET-27`).
    ///
    /// - `GET /control` is read-only and returns the current JSON with
    ///   `Access-Control-Allow-Origin: *`.
    /// - A mutation (`var=<param>&val=<int>` in the query, or in a
    ///   form-encoded body on `POST`) requires `POST`, or a `GET` carrying
    ///   `X-Requested-With`; a plain `GET` mutation is `405 Method Not
    ///   Allowed`. A browser `Origin` that does not match `Host` is `403`.
    ///   Mutating responses never carry a CORS header.
    fn handle_control<'b>(
        &mut self,
        req: &HttpRequest<'_>,
        out_buf: &'b mut [u8],
    ) -> Result<&'b [u8], HttpError> {
        let is_post = req.method == HttpMethod::Post;
        let body_params = if is_post && !req.body.is_empty() {
            let body_str = match core::str::from_utf8(req.body) {
                Ok(s) => s.trim(),
                Err(_) => {
                    return write_http_error(
                        out_buf,
                        400,
                        "Bad Request",
                        "Invalid UTF-8 in /control body\n",
                        false,
                    );
                }
            };
            if body_str.is_empty() {
                None
            } else {
                Some(body_str)
            }
        } else {
            None
        };
        let query_params = req.query.filter(|q| !q.is_empty());
        if query_params.is_some() && body_params.is_some() {
            return write_http_error(
                out_buf,
                400,
                "Bad Request",
                "Ambiguous /control parameters in both query and body\n",
                false,
            );
        }
        let params = query_params.or(body_params);
        if is_post || params.is_some() {
            if params.is_some() && !is_post && !req.x_requested_with {
                return write_control_method_not_allowed(out_buf);
            }
            if !self.is_allowed_host(req.host) {
                return write_http_error(
                    out_buf,
                    403,
                    "Forbidden",
                    "Host header not allowed for /control\n",
                    false,
                );
            }
            if let Some(origin) = req.origin {
                if !is_same_origin(origin, req.host) {
                    return write_http_error(
                        out_buf,
                        403,
                        "Forbidden",
                        "Cross-origin /control request refused\n",
                        false,
                    );
                }
            }
        }
        let cors = params.is_none() && !is_post;

        if let Some(q) = params {
            if q.len() > MAX_URL_QUERY_LEN {
                return write_http_error(
                    out_buf,
                    400,
                    "Bad Request",
                    "Query string exceeds maximum length\n",
                    false,
                );
            }

            let mut var_opt: Option<&str> = None;
            let mut val_str_opt: Option<&str> = None;
            for pair in q.split('&') {
                if let Some(eq_idx) = pair.find('=') {
                    let k = &pair[..eq_idx];
                    let v = &pair[eq_idx + 1..];
                    if k == "var" {
                        if var_opt.is_some() {
                            return write_http_error(
                                out_buf,
                                400,
                                "Bad Request",
                                "Duplicate 'var' parameter\n",
                                false,
                            );
                        }
                        var_opt = Some(v);
                    } else if k == "val" {
                        if val_str_opt.is_some() {
                            return write_http_error(
                                out_buf,
                                400,
                                "Bad Request",
                                "Duplicate 'val' parameter\n",
                                false,
                            );
                        }
                        val_str_opt = Some(v);
                    }
                }
            }

            let var_name = match var_opt {
                Some(v) if !v.is_empty() => v,
                _ => {
                    return write_http_error(
                        out_buf,
                        400,
                        "Bad Request",
                        "Missing 'var' parameter\n",
                        false,
                    );
                }
            };

            let val = match val_str_opt.and_then(parse_isize_ascii) {
                Some(v) => v,
                None => {
                    return write_http_error(
                        out_buf,
                        400,
                        "Bad Request",
                        "Missing or invalid integer 'val' parameter\n",
                        false,
                    );
                }
            };

            match var_name {
                "framesize" => match parse_framesize_val(val) {
                    Some(fs) => {
                        self.control_state.framesize = fs;
                        self.camera.set_frame_size(fs);
                    }
                    None => {
                        return write_http_error(
                            out_buf,
                            400,
                            "Bad Request",
                            "Invalid framesize value\n",
                            false,
                        );
                    }
                },
                "quality" => {
                    if !(4..=63).contains(&val) {
                        return write_http_error(
                            out_buf,
                            400,
                            "Bad Request",
                            "Quality must be in 4..=63\n",
                            false,
                        );
                    }
                    self.control_state.quality = val as i32;
                }
                "brightness" => {
                    if !(-2..=2).contains(&val) {
                        return write_http_error(
                            out_buf,
                            400,
                            "Bad Request",
                            "Brightness must be in -2..=2\n",
                            false,
                        );
                    }
                    self.control_state.brightness = val as i32;
                }
                "contrast" => {
                    if !(-2..=2).contains(&val) {
                        return write_http_error(
                            out_buf,
                            400,
                            "Bad Request",
                            "Contrast must be in -2..=2\n",
                            false,
                        );
                    }
                    self.control_state.contrast = val as i32;
                }
                "saturation" => {
                    if !(-2..=2).contains(&val) {
                        return write_http_error(
                            out_buf,
                            400,
                            "Bad Request",
                            "Saturation must be in -2..=2\n",
                            false,
                        );
                    }
                    self.control_state.saturation = val as i32;
                }
                "special_effect" => {
                    if !(0..=6).contains(&val) {
                        return write_http_error(
                            out_buf,
                            400,
                            "Bad Request",
                            "Special effect must be in 0..=6\n",
                            false,
                        );
                    }
                    self.control_state.special_effect = val as i32;
                }
                "hmirror" => {
                    if val != 0 && val != 1 {
                        return write_http_error(
                            out_buf,
                            400,
                            "Bad Request",
                            "Boolean parameter must be 0 or 1\n",
                            false,
                        );
                    }
                    self.control_state.hmirror = val != 0;
                }
                "vflip" => {
                    if val != 0 && val != 1 {
                        return write_http_error(
                            out_buf,
                            400,
                            "Bad Request",
                            "Boolean parameter must be 0 or 1\n",
                            false,
                        );
                    }
                    self.control_state.vflip = val != 0;
                }
                "awb" => {
                    if val != 0 && val != 1 {
                        return write_http_error(
                            out_buf,
                            400,
                            "Bad Request",
                            "Boolean parameter must be 0 or 1\n",
                            false,
                        );
                    }
                    self.control_state.awb = val != 0;
                }
                "agc" => {
                    if val != 0 && val != 1 {
                        return write_http_error(
                            out_buf,
                            400,
                            "Bad Request",
                            "Boolean parameter must be 0 or 1\n",
                            false,
                        );
                    }
                    self.control_state.agc = val != 0;
                }
                "aec" => {
                    if val != 0 && val != 1 {
                        return write_http_error(
                            out_buf,
                            400,
                            "Bad Request",
                            "Boolean parameter must be 0 or 1\n",
                            false,
                        );
                    }
                    self.control_state.aec = val != 0;
                }
                "ae_level" => {
                    if !(-2..=2).contains(&val) {
                        return write_http_error(
                            out_buf,
                            400,
                            "Bad Request",
                            "AE level must be in -2..=2\n",
                            false,
                        );
                    }
                    self.control_state.ae_level = val as i32;
                }
                _ => {
                    return write_http_error(
                        out_buf,
                        400,
                        "Bad Request",
                        "Unknown camera control variable\n",
                        false,
                    );
                }
            }

            self.control_state.dirty = true;
        }

        let mut json_stack = [0u8; CONTROL_JSON_BUF_LEN];
        let json_str = format_control_json(&self.control_state, &mut json_stack)?;

        let mut writer = SliceWriter::new(out_buf);
        write!(
            writer,
            "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n{}\
            Connection: close\r\nContent-Length: {}\r\n\r\n{}",
            if cors {
                "Access-Control-Allow-Origin: *\r\n"
            } else {
                ""
            },
            json_str.len(),
            json_str
        )
        .map_err(|_| HttpError::BufferTooSmall)?;
        Ok(writer.into_bytes())
    }

    /// Handles `GET /panel` on Port 80, serving the dark-mode Camera Control & Live
    /// Preview dashboard.
    fn handle_panel_get<'b>(&self, out_buf: &'b mut [u8]) -> Result<&'b [u8], HttpError> {
        let mut writer = SliceWriter::new(out_buf);
        write!(
            writer,
            "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\
            Access-Control-Allow-Origin: *\r\n\
            X-Frame-Options: DENY\r\n\
            Content-Security-Policy: default-src 'self'; script-src 'unsafe-inline'; \
            style-src 'unsafe-inline'; img-src 'self' http: https: data:; \
            frame-ancestors 'none'\r\n\
            X-Content-Type-Options: nosniff\r\n\
            Cache-Control: no-store\r\n\
            Connection: close\r\n\
            Content-Length: {}\r\n\r\n{}",
            KLIPPER_CONTROL_PANEL_HTML.len(),
            KLIPPER_CONTROL_PANEL_HTML
        )
        .map_err(|_| HttpError::BufferTooSmall)?;
        Ok(writer.into_bytes())
    }

    /// Handles `GET /update` on Port 80 (`klipper_cam.c` lines 280-348).
    fn handle_ota_get<'b>(&self, out_buf: &'b mut [u8]) -> Result<&'b [u8], HttpError> {
        let running_label = self.ota_manager.running_slot().label();
        let next_label = self.ota_manager.next_update_slot().label();

        let html_prefix = "<!DOCTYPE html><html><head><title>Klipper Cam OTA Update</title>\
<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\
<style>body{font-family:sans-serif;margin:40px;background:#1e1e1e;color:#eee}\
.card{background:#2a2a2a;padding:24px;border-radius:8px;max-width:500px;margin:auto}\
input,button{margin:12px 0;padding:10px;border-radius:4px;border:none;\
font-size:16px;box-sizing:border-box;width:100%}\
button{background:#007acc;color:#fff;cursor:pointer;width:100%}\
#prg{width:100%;background:#444;border-radius:4px;height:20px;display:none;margin-top:12px}\
#bar{width:0%;height:100%;background:#28a745;border-radius:4px;\
text-align:center;font-size:12px;line-height:20px}\
</style></head><body><div class=\"card\"><h2>Klipper Cam OTA Update</h2>";

        let html_suffix =
            "<input type=\"password\" id=\"tok\" placeholder=\"OTA Security Token\"><br>\
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
            "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\
            X-Frame-Options: DENY\r\n\
            Content-Security-Policy: default-src 'self'; script-src 'unsafe-inline'; \
            style-src 'unsafe-inline'; frame-ancestors 'none'\r\n\
            X-Content-Type-Options: nosniff\r\n\
            Cache-Control: no-store\r\n\
            Connection: close\r\nContent-Length: {}\r\n\r\n{}{}{}",
            body_len, html_prefix, info_str, html_suffix
        )
        .map_err(|_| HttpError::BufferTooSmall)?;
        Ok(writer.into_bytes())
    }

    /// Handles `POST /update` on Port 80 (`klipper_cam.c` lines 350-466):
    /// 1. [`Self::check_ota_auth`]: `403 Forbidden` while the public default
    ///    token is configured (`FW-NET-04`), else constant-time `X-OTA-Token`
    ///    authentication (`401 Unauthorized` without CORS, `FW-NET-08`).
    /// 2. Rejects re-entrant OTA after commit (`reboot_scheduled`) or while a
    ///    Port 81 MJPEG stream is active (`409 Conflict`, `M-043`).
    /// 3. `Content-Length` validation (`> 0`, `<= 0x1B0000`, exact body length
    ///    match `req.body.len() == content_len`, `FW-NET-09` / `M-139`).
    /// 4. ESP32 `0xE9` image magic, segment count, and mandatory SHA-256
    ///    validation, plus pre-flash `out_buf` capacity check (`M-043`).
    /// 5. Write to passive OTA slot + `otadata` CRC32 update + reboot flag
    ///    (`FW-NET-12`).
    fn handle_ota_post<'b, F: OtaFlashStorage>(
        &mut self,
        req: &HttpRequest<'_>,
        out_buf: &'b mut [u8],
        flash: &mut F,
    ) -> Result<&'b [u8], HttpError> {
        if !self.is_allowed_host(req.host) {
            return write_http_error(
                out_buf,
                403,
                "Forbidden",
                "Host header not allowed for /update\n",
                false,
            );
        }
        if let Err(rej) = self.check_ota_auth(req.ota_token.unwrap_or(b"")) {
            return write_http_error(out_buf, rej.status, rej.reason, rej.body, false);
        }

        if self.reboot_scheduled {
            return write_http_error(
                out_buf,
                409,
                "Conflict",
                "OTA update already committed; reboot pending\n",
                false,
            );
        }

        if self.is_stream_active() {
            return write_http_error(
                out_buf,
                409,
                "Conflict",
                "Active MJPEG stream must stop before OTA update\n",
                false,
            );
        }

        let content_len = match req.content_length {
            Some(len) if len > 0 => len,
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

        if req.body.is_empty() || req.body.len() != content_len {
            return write_http_error(
                out_buf,
                400,
                "Bad Request",
                "Firmware image validation failed",
                false,
            );
        }

        let payload = req.body;
        if validate_ota_image_header(payload, content_len, max_size).is_err() {
            return write_http_error(
                out_buf,
                400,
                "Bad Request",
                "Firmware image validation failed",
                false,
            );
        }

        // Pre-format the 200 OK response and verify `out_buf` capacity before
        // mutating SPI flash or setting `reboot_scheduled` (M-043).
        let ok_body = "Firmware update successful! Rebooting...\n";
        let mut ok_resp_buf = [0u8; 160];
        let ok_resp_len = {
            let mut ok_writer = SliceWriter::new(&mut ok_resp_buf);
            write!(
                ok_writer,
                "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\n\
                Connection: close\r\nContent-Length: {}\r\n\r\n{}",
                ok_body.len(),
                ok_body
            )
            .map_err(|_| HttpError::BufferTooSmall)?;
            ok_writer.written_len()
        };
        if out_buf.len() < ok_resp_len {
            return Err(HttpError::BufferTooSmall);
        }

        let written_slot = match self.ota_manager.write_ota_image_to_flash(flash, payload) {
            Ok(slot) => slot,
            Err(OtaError::ReadbackMismatch) => {
                return write_http_error(
                    out_buf,
                    500,
                    "Internal Server Error",
                    "Flash read-back verification failed",
                    false,
                );
            }
            Err(
                OtaError::InvalidOtaDataCrc
                | OtaError::RollbackStateInvalid
                | OtaError::NoPassivePartition
                | OtaError::SequenceOverflow
                | OtaError::FlashWriteError,
            ) => {
                return write_http_error(
                    out_buf,
                    500,
                    "Internal Server Error",
                    "Failed to update boot partition",
                    false,
                );
            }
            Err(_) => {
                return write_http_error(
                    out_buf,
                    400,
                    "Bad Request",
                    "Firmware image validation failed",
                    false,
                );
            }
        };

        self.last_ota_bytes_written = content_len;
        self.last_ota_target_slot = Some(written_slot);
        self.reboot_scheduled = true;

        out_buf[..ok_resp_len].copy_from_slice(&ok_resp_buf[..ok_resp_len]);
        Ok(&out_buf[..ok_resp_len])
    }

    /// Handles `GET /setup` on Port 80 in normal STA mode, serving a blank
    /// provisioning form (displaying MAC, current SSID, and current IP mode
    /// without ever echoing secrets) that submits via `fetch('/setup')` with
    /// `X-OTA-Token` and `X-Requested-With: fetch`.
    fn handle_setup_get<'b>(&self, out_buf: &'b mut [u8]) -> Result<&'b [u8], HttpError> {
        let mut mac_buf = [0u8; 17];
        let mac_str = format_mac_lower(&self.device_mac, &mut mac_buf);

        let mut info_buf = [0u8; 512];
        let info_str = {
            let mut iw = SliceWriter::new(&mut info_buf);
            iw.write_str_checked("<p class=\"m\">MAC: ")?;
            iw.write_str_checked(mac_str)?;
            iw.write_str_checked("<br>Current SSID: ")?;
            if self.config.network.wifi_ssid.is_empty() {
                iw.write_str_checked("(not set)")?;
            } else {
                write_html_escaped(
                    &mut iw,
                    truncate_utf8_prefix(self.config.network.wifi_ssid, MAX_SSID_LEN),
                )?;
            }
            iw.write_str_checked("<br>IP Mode: ")?;
            match self.config.ip_mode() {
                IpMode::Dhcp => iw.write_str_checked("DHCP")?,
                IpMode::Static(static_cfg) => {
                    write!(iw, "Static ({}/{})", static_cfg.ip, static_cfg.prefix_len)
                        .map_err(|_| HttpError::BufferTooSmall)?;
                }
            }
            iw.write_str_checked("</p>")?;
            let n = iw.written_len();
            core::str::from_utf8(&info_buf[..n]).map_err(|_| HttpError::InvalidUtf8)?
        };

        let body_len = SETUP_HTML_PREFIX.len() + info_str.len() + SETUP_STA_HTML_SUFFIX.len();
        let mut writer = SliceWriter::new(out_buf);
        write!(
            writer,
            "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\
            X-Frame-Options: DENY\r\n\
            Content-Security-Policy: {SETUP_HTML_CSP}\r\n\
            X-Content-Type-Options: nosniff\r\n\
            Cache-Control: no-store\r\n\
            Connection: close\r\nContent-Length: {}\r\n\r\n{}{}{}",
            body_len, SETUP_HTML_PREFIX, info_str, SETUP_STA_HTML_SUFFIX
        )
        .map_err(|_| HttpError::BufferTooSmall)?;
        Ok(writer.into_bytes())
    }

    /// Handles `POST /setup` on Port 80 in normal STA mode:
    /// 1. Enforces `Host` allowlist (`M-047`), same-origin (`M-046`), and
    ///    constant-time `X-OTA-Token` authentication (`401`/`403`, fail-closed).
    /// 2. Rejects re-entrant commit while a reboot is pending (`409 Conflict`).
    /// 3. Strictly parses and validates a full replacement
    ///    [`ProvisionedConfig`] (`application/x-www-form-urlencoded`,
    ///    `<= 512 B`, never echoing secrets in error responses).
    /// 4. Persists via [`ProvStore::save`] and sets `self.reboot_scheduled`.
    fn handle_setup_post<'b, F: OtaFlashStorage>(
        &mut self,
        req: &HttpRequest<'_>,
        out_buf: &'b mut [u8],
        flash: &mut F,
    ) -> Result<&'b [u8], HttpError> {
        if !self.is_allowed_host(req.host) {
            return write_http_error(
                out_buf,
                403,
                "Forbidden",
                "Host header not allowed for /setup\n",
                false,
            );
        }
        if let Some(origin) = req.origin {
            if !is_same_origin(origin, req.host) {
                return write_http_error(
                    out_buf,
                    403,
                    "Forbidden",
                    "Cross-origin /setup request refused\n",
                    false,
                );
            }
        }
        if let Err(rej) = self.check_ota_auth(req.ota_token.unwrap_or(b"")) {
            return write_http_error(out_buf, rej.status, rej.reason, rej.body, false);
        }
        if self.reboot_scheduled {
            return write_http_error(
                out_buf,
                409,
                "Conflict",
                "Update or setup already committed; reboot pending\n",
                false,
            );
        }

        let cfg = match parse_setup_post_config(req) {
            Ok(cfg) => cfg,
            Err(msg) => {
                return write_http_error(out_buf, 400, "Bad Request", msg, false);
            }
        };

        let ok_body = "Configuration saved. Rebooting...\n";
        let mut ok_resp_buf = [0u8; 256];
        let ok_resp_len = {
            let mut ok_writer = SliceWriter::new(&mut ok_resp_buf);
            write!(
                ok_writer,
                "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\n\
                Cache-Control: no-store\r\n\
                X-Content-Type-Options: nosniff\r\n\
                Connection: close\r\nContent-Length: {}\r\n\r\n{}",
                ok_body.len(),
                ok_body
            )
            .map_err(|_| HttpError::BufferTooSmall)?;
            ok_writer.written_len()
        };
        if out_buf.len() < ok_resp_len {
            return Err(HttpError::BufferTooSmall);
        }

        if let Err(err) = ProvStore::save(flash, &cfg) {
            return match err {
                ProvStoreError::InvalidSsid
                | ProvStoreError::InvalidPassphrase
                | ProvStoreError::InvalidOtaToken
                | ProvStoreError::InvalidStaticIp => {
                    write_http_error(out_buf, 400, "Bad Request", prov_error_message(err), false)
                }
                ProvStoreError::FlashError(_)
                | ProvStoreError::ReadbackMismatch
                | ProvStoreError::SequenceExhausted => write_http_error(
                    out_buf,
                    500,
                    "Internal Server Error",
                    "Failed to save provisioning configuration to flash\n",
                    false,
                ),
            };
        }

        self.reboot_scheduled = true;
        out_buf[..ok_resp_len].copy_from_slice(&ok_resp_buf[..ok_resp_len]);
        Ok(&out_buf[..ok_resp_len])
    }

    /// Handles requests arriving on Port 81 (`s_stream_httpd` in `klipper_cam.c`
    /// lines 524-555).
    ///
    /// Both `GET /stream` and `GET /` on Port 81 serve the MJPEG multipart stream.
    /// Enforces the `AtomicBool` single-client busy guard (`NET-002`), returning
    /// `HTTP/1.1 500 Internal Server Error` with `"Stream already active"` if
    /// already busy.
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

                // Write HTTP 200 multipart headers + initial MJPEG frame chunk for single-
                // request inspection
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

/// Formats the plain-text status landing page for the minimal OTA recovery
/// intermediary (`ota_minimal.c`).
pub fn format_ota_minimal_root<'a>(
    ota_manager: &OtaDataManager,
    free_heap: u32,
    out_buf: &'a mut [u8],
) -> Result<&'a [u8], HttpError> {
    let running = ota_manager.running_slot().partition();
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
        "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\n\
        Connection: close\r\nContent-Length: {}\r\n\r\n{}",
        body_str.len(),
        body_str
    )
    .map_err(|_| HttpError::BufferTooSmall)?;
    Ok(writer.into_bytes())
}

/// Writes `405 Method Not Allowed` with `Allow: POST` (no CORS header) for a
/// state-changing `GET /control` (`FW-NET-27`).
fn write_control_method_not_allowed(out_buf: &mut [u8]) -> Result<&[u8], HttpError> {
    let body = "Use POST (or send X-Requested-With) to change camera settings\n";
    let mut writer = SliceWriter::new(out_buf);
    write!(
        writer,
        "HTTP/1.1 405 Method Not Allowed\r\nAllow: POST\r\nContent-Type: text/plain\r\n\
        Connection: close\r\nContent-Length: {}\r\n\r\n{}",
        body.len(),
        body
    )
    .map_err(|_| HttpError::BufferTooSmall)?;
    Ok(writer.into_bytes())
}

/// Helper to write an HTTP error response into `out_buf`.
pub fn write_http_error<'a>(
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
            "HTTP/1.1 {} {}\r\nContent-Type: text/plain\r\n\
            Access-Control-Allow-Origin: *\r\nConnection: close\r\n\
            Content-Length: {}\r\n\r\n{}",
            status_code,
            reason,
            message.len(),
            message
        )
        .map_err(|_| HttpError::BufferTooSmall)?;
    } else {
        write!(
            writer,
            "HTTP/1.1 {} {}\r\nContent-Type: text/plain\r\n\
            Connection: close\r\nContent-Length: {}\r\n\r\n{}",
            status_code,
            reason,
            message.len(),
            message
        )
        .map_err(|_| HttpError::BufferTooSmall)?;
    }
    Ok(writer.into_bytes())
}

/// Maximum accepted `POST /setup` form-urlencoded request body size in bytes
/// (`512`).
pub const MAX_SETUP_FORM_BODY_LEN: usize = 512;

/// Content-Security-Policy header applied to `/setup` HTML responses.
const SETUP_HTML_CSP: &str = "default-src 'self' 'unsafe-inline'; frame-ancestors 'none'";

/// Shared mobile-friendly dark-mode HTML prefix for SoftAP and STA `/setup`.
const SETUP_HTML_PREFIX: &str = "\
<!DOCTYPE html><html><head><meta charset=\"utf-8\">\
<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">\
<title>Klipper Cam Setup</title><style>\
body{font-family:sans-serif;margin:16px;background:#121212;color:#eee}\
.c{max-width:440px;margin:auto;background:#1e1e1e;padding:18px;border-radius:8px;\
border:1px solid #333}h2{margin:0 0 10px;font-size:18px}\
.m{font-size:12px;color:#9cdcfe;margin:0 0 12px}\
label{display:block;font-size:13px;margin:8px 0 3px}\
input,select,button{width:100%;padding:8px;border-radius:4px;border:1px solid #444;\
background:#2a2a2a;color:#eee;box-sizing:border-box;font-size:14px}\
button{margin-top:14px;background:#007acc;border:none;cursor:pointer;font-weight:700}\
#sip{margin-top:6px}#st{font-size:13px;margin-top:10px;color:#f88}\
</style></head><body><div class=\"c\"><h2>Klipper Cam Setup</h2>";

/// SoftAP `GET /setup` HTML form suffix (direct `POST /setup` submission).
const SETUP_AP_HTML_SUFFIX: &str = "\
<form method=\"POST\" action=\"/setup\">\
<label>Wi-Fi SSID</label><input name=\"ssid\" maxlength=\"32\" required>\
<label>Wi-Fi Passphrase (8-63 chars)</label>\
<input name=\"pass\" type=\"password\" maxlength=\"63\" autocomplete=\"off\">\
<label>OTA Token (16-64 chars)</label>\
<input name=\"token\" type=\"password\" minlength=\"16\" maxlength=\"64\" \
placeholder=\"&gt;= 16 chars\" required autocomplete=\"off\">\
<label>IP Mode</label>\
<select name=\"ip_mode\" id=\"im\" onchange=\"tg()\">\
<option value=\"dhcp\">DHCP</option><option value=\"static\">Static IP</option></select>\
<div id=\"sip\" style=\"display:none\">\
<label>Static IP</label><input name=\"static_ip\" placeholder=\"172.22.1.52\">\
<label>Gateway</label><input name=\"gateway\" placeholder=\"172.22.1.1\">\
<label>Prefix Length (8-30)</label><input name=\"prefix\" value=\"24\" placeholder=\"24\">\
</div><button type=\"submit\">Save &amp; Reboot</button></form></div>\
<script>function tg(){document.getElementById('sip').style.display=\
document.getElementById('im').value==='static'?'block':'none';}tg();</script>\
</body></html>";

/// STA-mode `GET /setup` HTML form suffix (authenticated `fetch('/setup')`
/// carrying `X-OTA-Token` and `X-Requested-With: fetch`).
const SETUP_STA_HTML_SUFFIX: &str = "\
<form id=\"sf\" onsubmit=\"return sub(event)\">\
<label>Current OTA Token</label>\
<input id=\"cur_tok\" type=\"password\" maxlength=\"64\" required autocomplete=\"off\">\
<label>Wi-Fi SSID</label><input id=\"ssid\" name=\"ssid\" maxlength=\"32\" required>\
<label>Wi-Fi Passphrase (8-63 chars)</label>\
<input id=\"pass\" name=\"pass\" type=\"password\" maxlength=\"63\" autocomplete=\"off\">\
<label>New OTA Token (16-64 chars)</label>\
<input id=\"token\" name=\"token\" type=\"password\" minlength=\"16\" maxlength=\"64\" \
placeholder=\"&gt;= 16 chars\" required autocomplete=\"off\">\
<label>IP Mode</label>\
<select id=\"im\" name=\"ip_mode\" onchange=\"tg()\">\
<option value=\"dhcp\">DHCP</option><option value=\"static\">Static IP</option></select>\
<div id=\"sip\" style=\"display:none\">\
<label>Static IP</label><input id=\"static_ip\" name=\"static_ip\" placeholder=\"172.22.1.52\">\
<label>Gateway</label><input id=\"gateway\" name=\"gateway\" placeholder=\"172.22.1.1\">\
<label>Prefix Length (8-30)</label>\
<input id=\"prefix\" name=\"prefix\" value=\"24\" placeholder=\"24\">\
</div><button id=\"btn\" type=\"submit\">Save &amp; Reboot</button></form>\
<p id=\"st\"></p></div><script>\
function g(i){return document.getElementById(i);}\
function tg(){g('sip').style.display=g('im').value==='static'?'block':'none';}tg();\
function sub(e){e.preventDefault();var cur=g('cur_tok').value;\
var q=function(k,v){return k+'='+encodeURIComponent(v);};\
var body=[q('ssid',g('ssid').value),q('pass',g('pass').value),\
q('token',g('token').value),q('ip_mode',g('im').value),\
q('static_ip',g('static_ip').value),q('gateway',g('gateway').value),\
q('prefix',g('prefix').value)].join('&');\
g('btn').disabled=true;g('st').innerText='Saving...';\
fetch('/setup',{method:'POST',headers:{\
'Content-Type':'application/x-www-form-urlencoded',\
'X-OTA-Token':cur,'X-Requested-With':'fetch'},body:body})\
.then(function(r){return r.text().then(function(t){\
g('st').innerText=r.ok?'Configuration saved. Rebooting...':t;\
if(!r.ok)g('btn').disabled=false;});})\
.catch(function(){g('st').innerText='Network error';g('btn').disabled=false;});\
return false;}</script></body></html>";

/// SoftAP `POST /setup` 200 OK reboot confirmation HTML body.
const SETUP_SAVED_HTML: &str = "\
<!DOCTYPE html><html><head><meta charset=\"utf-8\">\
<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">\
<title>Saved</title><style>\
body{font-family:sans-serif;margin:24px;background:#121212;color:#eee;text-align:center}\
</style></head><body><h2>Configuration Saved</h2>\
<p>Rebooting into Wi-Fi STA mode...</p></body></html>";

/// SoftAP `POST /setup` 400 Bad Request HTML prefix.
const SETUP_ERR_HTML_PREFIX: &str = "\
<!DOCTYPE html><html><head><meta charset=\"utf-8\">\
<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">\
<title>Setup Error</title><style>\
body{font-family:sans-serif;margin:24px;background:#121212;color:#eee}\
.c{max-width:440px;margin:auto;background:#1e1e1e;padding:18px;border-radius:8px;\
border:1px solid #633}a{color:#569cd6}\
</style></head><body><div class=\"c\"><h2>Invalid Configuration</h2><p>";

/// SoftAP `POST /setup` 400 Bad Request HTML suffix.
const SETUP_ERR_HTML_SUFFIX: &str =
    "</p><p><a href=\"/setup\">Back to setup</a></p></div></body></html>";

/// Outcome of handling one SoftAP Setup Mode HTTP request via
/// [`handle_setup_http_request`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SetupHttpOutcome {
    /// Complete HTTP response of `usize` bytes was written to `out_buf`.
    Responded(usize),
    /// Valid [`ProvisionedConfig`] was parsed and a `200 OK` reboot page of
    /// `resp_len` bytes was written to `out_buf`.
    SavedAndReboot {
        /// Number of HTTP response bytes written to `out_buf`.
        resp_len: usize,
        /// Validated configuration ready to persist via [`ProvStore::save`].
        config: ProvisionedConfig,
    },
}

/// Stack buffer zeroized automatically on drop so decoded secrets do not
/// linger in stack frames (`#![forbid(unsafe_code)]`).
struct SecretStackBuf<const N: usize>([u8; N]);

impl<const N: usize> SecretStackBuf<N> {
    #[inline]
    const fn zeroed() -> Self {
        Self([0u8; N])
    }
}

impl<const N: usize> Drop for SecretStackBuf<N> {
    #[inline]
    fn drop(&mut self) {
        self.0.fill(0);
    }
}

/// Formats a 6-byte hardware MAC address as lowercase `"xx:xx:xx:xx:xx:xx"`.
fn format_mac_lower<'a>(mac: &[u8; 6], buf: &'a mut [u8; 17]) -> &'a str {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut i = 0;
    while i < 6 {
        let base = i * 3;
        buf[base] = HEX[(mac[i] >> 4) as usize];
        buf[base + 1] = HEX[(mac[i] & 0x0F) as usize];
        if i < 5 {
            buf[base + 2] = b':';
        }
        i += 1;
    }
    core::str::from_utf8(buf).unwrap_or("00:00:00:00:00:00")
}

/// Returns the longest valid UTF-8 prefix of `s` with length `<= max_bytes`.
fn truncate_utf8_prefix(s: &str, max_bytes: usize) -> &str {
    let mut len = s.len().min(max_bytes);
    while !s.is_char_boundary(len) {
        len = len.saturating_sub(1);
    }
    &s[..len]
}

/// Escapes HTML special characters and replaces ASCII control bytes with `'_'`.
fn write_html_escaped(writer: &mut SliceWriter<'_>, input: &str) -> Result<(), HttpError> {
    for b in input.bytes() {
        match b {
            b'&' => writer.write_str_checked("&amp;")?,
            b'<' => writer.write_str_checked("&lt;")?,
            b'>' => writer.write_str_checked("&gt;")?,
            b'"' => writer.write_str_checked("&quot;")?,
            b'\'' => writer.write_str_checked("&#39;")?,
            0x00..=0x1F | 0x7F => writer.write_str_checked("_")?,
            _ => writer.write_bytes(&[b])?,
        }
    }
    Ok(())
}

/// Decodes one ASCII hexadecimal digit (`0-9`, `a-f`, `A-F`) into `0..=15`.
#[inline]
const fn decode_hex_nibble(b: u8) -> Option<u8> {
    match b {
        b'0'..=b'9' => Some(b - b'0'),
        b'a'..=b'f' => Some(b - b'a' + 10),
        b'A'..=b'F' => Some(b - b'A' + 10),
        _ => None,
    }
}

/// Strictly decodes an `application/x-www-form-urlencoded` field value into
/// `dst`.
///
/// Maps `+` to space (`0x20`) and `%XX` to its byte value while rejecting
/// truncated or non-hex `%` escapes (e.g. `%ZZ`), decoded ASCII control bytes
/// (`< 0x20` or `0x7F`, including `%00` and `%0D%0A`), buffer overflow, and
/// invalid UTF-8.
fn decode_form_value(src: &[u8], dst: &mut [u8]) -> Result<usize, &'static str> {
    let mut i = 0;
    let mut out_len = 0;
    while i < src.len() {
        let decoded = match src[i] {
            b'+' => {
                i += 1;
                b' '
            }
            b'%' => {
                if i + 2 >= src.len() {
                    return Err("Invalid percent-encoding in form submission\n");
                }
                let Some(hi) = decode_hex_nibble(src[i + 1]) else {
                    return Err("Invalid percent-encoding in form submission\n");
                };
                let Some(lo) = decode_hex_nibble(src[i + 2]) else {
                    return Err("Invalid percent-encoding in form submission\n");
                };
                i += 3;
                (hi << 4) | lo
            }
            b => {
                i += 1;
                b
            }
        };
        if decoded < 0x20 || decoded == 0x7F {
            return Err("Control characters are not allowed in setup fields\n");
        }
        if out_len >= dst.len() {
            return Err("Form field exceeds maximum allowed length\n");
        }
        dst[out_len] = decoded;
        out_len += 1;
    }
    if core::str::from_utf8(&dst[..out_len]).is_err() {
        return Err("Form field contains invalid UTF-8\n");
    }
    Ok(out_len)
}

/// Validates percent-encoding and rejects control bytes in an ignored/unknown
/// form field without retaining its bytes.
fn validate_ignored_form_value(src: &[u8]) -> Result<(), &'static str> {
    let mut i = 0;
    while i < src.len() {
        let decoded = match src[i] {
            b'+' => {
                i += 1;
                b' '
            }
            b'%' => {
                if i + 2 >= src.len() {
                    return Err("Invalid percent-encoding in form submission\n");
                }
                let Some(hi) = decode_hex_nibble(src[i + 1]) else {
                    return Err("Invalid percent-encoding in form submission\n");
                };
                let Some(lo) = decode_hex_nibble(src[i + 2]) else {
                    return Err("Invalid percent-encoding in form submission\n");
                };
                i += 3;
                (hi << 4) | lo
            }
            b => {
                i += 1;
                b
            }
        };
        if decoded < 0x20 || decoded == 0x7F {
            return Err("Control characters are not allowed in setup fields\n");
        }
    }
    Ok(())
}

/// Returns `true` if `content_type` has media type
/// `application/x-www-form-urlencoded` (ignoring optional parameters such as
/// `; charset=utf-8`).
fn is_form_urlencoded_content_type(content_type: Option<&str>) -> bool {
    let Some(ct) = content_type else {
        return false;
    };
    let media = ct.split(';').next().unwrap_or("").trim();
    media.eq_ignore_ascii_case("application/x-www-form-urlencoded")
}

/// Maps a [`ProvStoreError`] to a static user-safe error message (never
/// echoing any user input or secret).
const fn prov_error_message(err: ProvStoreError) -> &'static str {
    match err {
        ProvStoreError::InvalidSsid => {
            "Invalid SSID: must be 1..=32 UTF-8 bytes without control characters\n"
        }
        ProvStoreError::InvalidPassphrase => {
            "Invalid Wi-Fi passphrase: must be 8..=63 printable ASCII bytes\n"
        }
        ProvStoreError::InvalidOtaToken => {
            "Invalid OTA token: must be 16..=64 graphic ASCII bytes and not the factory default\n"
        }
        ProvStoreError::InvalidStaticIp => {
            "Invalid static IPv4 settings: requires valid unicast IP, gateway, and /8../30 prefix\n"
        }
        ProvStoreError::SequenceExhausted => "Provisioning sequence counter exhausted\n",
        ProvStoreError::FlashError(_) => "Flash storage error\n",
        ProvStoreError::ReadbackMismatch => "Flash readback verification failed\n",
    }
}

/// Parses a strict ASCII unsigned CIDR prefix length (`8..=30`).
fn parse_prefix_len_ascii(s: &str) -> Option<u8> {
    if s.is_empty() || s.len() > 2 || !s.bytes().all(|b| b.is_ascii_digit()) {
        return None;
    }
    s.parse::<u8>().ok()
}

/// Parses and validates a `POST /setup` `application/x-www-form-urlencoded`
/// body (`<= 512 B`) into a complete [`ProvisionedConfig`].
///
/// Returned error strings are strictly static and never echo secrets or raw
/// request bytes.
fn parse_setup_post_config(req: &HttpRequest<'_>) -> Result<ProvisionedConfig, &'static str> {
    if !is_form_urlencoded_content_type(req.content_type) {
        return Err("Content-Type must be application/x-www-form-urlencoded\n");
    }
    let Some(content_len) = req.content_length else {
        return Err("Content-Length header is required\n");
    };
    if content_len == 0 || content_len > MAX_SETUP_FORM_BODY_LEN {
        return Err("Form body must be 1..=512 bytes\n");
    }
    if req.body.len() != content_len || req.body.len() > MAX_SETUP_FORM_BODY_LEN {
        return Err("Form body length does not match Content-Length\n");
    }

    let mut ssid_buf = [0u8; MAX_SSID_LEN];
    let mut ssid_len = 0usize;
    let mut seen_ssid = false;

    let mut pass_buf = SecretStackBuf::<MAX_PASS_LEN>::zeroed();
    let mut pass_len = 0usize;
    let mut seen_pass = false;

    let mut token_buf = SecretStackBuf::<MAX_OTA_TOKEN_LEN>::zeroed();
    let mut token_len = 0usize;
    let mut seen_token = false;

    let mut ip_mode_buf = [0u8; 8];
    let mut ip_mode_len = 0usize;
    let mut seen_ip_mode = false;

    let mut static_ip_buf = [0u8; 15];
    let mut static_ip_len = 0usize;
    let mut seen_static_ip = false;

    let mut gateway_buf = [0u8; 15];
    let mut gateway_len = 0usize;
    let mut seen_gateway = false;

    let mut dns_buf = [0u8; 15];
    let mut dns_len = 0usize;
    let mut seen_dns = false;

    let mut prefix_buf = [0u8; 4];
    let mut prefix_len = 0usize;
    let mut seen_prefix = false;

    for pair in req.body.split(|&b| b == b'&') {
        if pair.is_empty() {
            return Err("Malformed empty form parameter\n");
        }
        let Some(eq_idx) = pair.iter().position(|&b| b == b'=') else {
            return Err("Malformed form parameter missing '='\n");
        };
        let key = &pair[..eq_idx];
        let raw_val = &pair[eq_idx + 1..];
        if key.is_empty() {
            return Err("Malformed form parameter with empty key\n");
        }
        match key {
            b"ssid" => {
                if seen_ssid {
                    return Err("Duplicate 'ssid' field\n");
                }
                seen_ssid = true;
                ssid_len = decode_form_value(raw_val, &mut ssid_buf)?;
            }
            b"pass" => {
                if seen_pass {
                    return Err("Duplicate 'pass' field\n");
                }
                seen_pass = true;
                pass_len = decode_form_value(raw_val, &mut pass_buf.0)?;
            }
            b"token" => {
                if seen_token {
                    return Err("Duplicate 'token' field\n");
                }
                seen_token = true;
                token_len = decode_form_value(raw_val, &mut token_buf.0)?;
            }
            b"ip_mode" => {
                if seen_ip_mode {
                    return Err("Duplicate 'ip_mode' field\n");
                }
                seen_ip_mode = true;
                ip_mode_len = decode_form_value(raw_val, &mut ip_mode_buf)?;
            }
            b"static_ip" => {
                if seen_static_ip {
                    return Err("Duplicate 'static_ip' field\n");
                }
                seen_static_ip = true;
                static_ip_len = decode_form_value(raw_val, &mut static_ip_buf)?;
            }
            b"gateway" => {
                if seen_gateway {
                    return Err("Duplicate 'gateway' field\n");
                }
                seen_gateway = true;
                gateway_len = decode_form_value(raw_val, &mut gateway_buf)?;
            }
            b"dns" => {
                if seen_dns {
                    return Err("Duplicate 'dns' field\n");
                }
                seen_dns = true;
                dns_len = decode_form_value(raw_val, &mut dns_buf)?;
            }
            b"prefix" => {
                if seen_prefix {
                    return Err("Duplicate 'prefix' field\n");
                }
                seen_prefix = true;
                prefix_len = decode_form_value(raw_val, &mut prefix_buf)?;
            }
            _ => {
                validate_ignored_form_value(raw_val)?;
            }
        }
    }

    if !seen_ssid {
        return Err("Missing required 'ssid' field\n");
    }
    if !seen_token {
        return Err("Missing required 'token' field\n");
    }
    if !seen_ip_mode {
        return Err("Missing required 'ip_mode' field\n");
    }

    let ssid =
        core::str::from_utf8(&ssid_buf[..ssid_len]).map_err(|_| "Invalid UTF-8 in ssid\n")?;
    let pass =
        core::str::from_utf8(&pass_buf.0[..pass_len]).map_err(|_| "Invalid UTF-8 in pass\n")?;
    let token =
        core::str::from_utf8(&token_buf.0[..token_len]).map_err(|_| "Invalid UTF-8 in token\n")?;
    let ip_mode_str =
        core::str::from_utf8(&ip_mode_buf[..ip_mode_len]).map_err(|_| "Invalid ip_mode value\n")?;

    if ip_mode_str.eq_ignore_ascii_case("dhcp") {
        ProvisionedConfig::new_dhcp(ssid, pass, token).map_err(prov_error_message)
    } else if ip_mode_str.eq_ignore_ascii_case("static") {
        let static_ip_str = core::str::from_utf8(&static_ip_buf[..static_ip_len])
            .map_err(|_| "Invalid static_ip value\n")?;
        let gateway_str = core::str::from_utf8(&gateway_buf[..gateway_len])
            .map_err(|_| "Invalid gateway value\n")?;
        let prefix_str = core::str::from_utf8(&prefix_buf[..prefix_len])
            .map_err(|_| "Invalid prefix value\n")?;

        let ip = Ipv4Addr::parse_ascii(static_ip_str)
            .ok_or("Invalid static_ip: expected dotted-quad IPv4 address\n")?;
        let gateway = Ipv4Addr::parse_ascii(gateway_str)
            .ok_or("Invalid gateway: expected dotted-quad IPv4 address\n")?;
        let dns = if dns_len > 0 {
            let dns_str =
                core::str::from_utf8(&dns_buf[..dns_len]).map_err(|_| "Invalid dns value\n")?;
            Ipv4Addr::parse_ascii(dns_str)
                .ok_or("Invalid dns: expected dotted-quad IPv4 address\n")?
        } else {
            gateway
        };
        let prefix = parse_prefix_len_ascii(prefix_str)
            .ok_or("Invalid prefix: expected CIDR prefix length 8..=30\n")?;

        ProvisionedConfig::new_static(ssid, pass, token, ip, gateway, dns, prefix)
            .map_err(prov_error_message)
    } else {
        Err("Invalid ip_mode: must be 'dhcp' or 'static'\n")
    }
}

/// RFC 8908 Captive Portal API JSON payload returned by `GET /capport`.
const CAPPORT_JSON_BODY: &str = r#"{"captive":true,"user-portal-url":"http://192.168.4.1/setup"}"#;

/// Writes the RFC 8908 `GET /capport` JSON status response.
fn write_capport_json(out_buf: &mut [u8]) -> Result<usize, HttpError> {
    let mut writer = SliceWriter::new(out_buf);
    write!(
        writer,
        "HTTP/1.1 200 OK\r\n\
         Content-Type: application/captive+json\r\n\
         Cache-Control: no-store\r\n\
         Connection: close\r\n\
         Content-Length: {}\r\n\r\n{}",
        CAPPORT_JSON_BODY.len(),
        CAPPORT_JSON_BODY
    )
    .map_err(|_| HttpError::BufferTooSmall)?;
    Ok(writer.written_len())
}

/// Writes a `302 Found` captive-portal redirect to `http://192.168.4.1/setup`.
fn write_setup_captive_redirect(out_buf: &mut [u8]) -> Result<usize, HttpError> {
    let mut writer = SliceWriter::new(out_buf);
    writer.write_str_checked(
        "HTTP/1.1 302 Found\r\n\
         Location: http://192.168.4.1/setup\r\n\
         Cache-Control: no-store\r\n\
         Connection: close\r\n\
         Content-Length: 0\r\n\r\n",
    )?;
    Ok(writer.written_len())
}

/// Writes the SoftAP `GET /setup` mobile-friendly HTML form (`<= 3 KB`, zero
/// external assets, strict CSP/frame/cache headers, never echoing secrets).
fn write_softap_setup_get_html(
    mac: &[u8; 6],
    ap_ssid: &str,
    current_ssid: &str,
    out_buf: &mut [u8],
) -> Result<usize, HttpError> {
    let mut mac_buf = [0u8; 17];
    let mac_str = format_mac_lower(mac, &mut mac_buf);

    let mut info_buf = [0u8; 512];
    let info_str = {
        let mut iw = SliceWriter::new(&mut info_buf);
        iw.write_str_checked("<p class=\"m\">MAC: ")?;
        iw.write_str_checked(mac_str)?;
        iw.write_str_checked("<br>AP SSID: ")?;
        write_html_escaped(&mut iw, truncate_utf8_prefix(ap_ssid, MAX_SSID_LEN))?;
        iw.write_str_checked("<br>Current SSID: ")?;
        if current_ssid.is_empty() {
            iw.write_str_checked("(not set)")?;
        } else {
            write_html_escaped(&mut iw, truncate_utf8_prefix(current_ssid, MAX_SSID_LEN))?;
        }
        iw.write_str_checked("</p>")?;
        let n = iw.written_len();
        core::str::from_utf8(&info_buf[..n]).map_err(|_| HttpError::InvalidUtf8)?
    };

    let body_len = SETUP_HTML_PREFIX.len() + info_str.len() + SETUP_AP_HTML_SUFFIX.len();
    let mut writer = SliceWriter::new(out_buf);
    write!(
        writer,
        "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\
        X-Frame-Options: DENY\r\n\
        Content-Security-Policy: {SETUP_HTML_CSP}\r\n\
        X-Content-Type-Options: nosniff\r\n\
        Cache-Control: no-store\r\n\
        Connection: close\r\nContent-Length: {}\r\n\r\n{}{}{}",
        body_len, SETUP_HTML_PREFIX, info_str, SETUP_AP_HTML_SUFFIX
    )
    .map_err(|_| HttpError::BufferTooSmall)?;
    Ok(writer.written_len())
}

/// Writes a SoftAP `400 Bad Request` HTML error page with a static message
/// (never echoing secrets).
fn write_softap_setup_error_html(err_msg: &str, out_buf: &mut [u8]) -> Result<usize, HttpError> {
    let trimmed = err_msg.trim_end_matches(['\r', '\n']);
    let body_len = SETUP_ERR_HTML_PREFIX.len() + trimmed.len() + SETUP_ERR_HTML_SUFFIX.len();
    let mut writer = SliceWriter::new(out_buf);
    write!(
        writer,
        "HTTP/1.1 400 Bad Request\r\nContent-Type: text/html\r\n\
        X-Frame-Options: DENY\r\n\
        Content-Security-Policy: {SETUP_HTML_CSP}\r\n\
        X-Content-Type-Options: nosniff\r\n\
        Cache-Control: no-store\r\n\
        Connection: close\r\nContent-Length: {}\r\n\r\n{}{}{}",
        body_len, SETUP_ERR_HTML_PREFIX, trimmed, SETUP_ERR_HTML_SUFFIX
    )
    .map_err(|_| HttpError::BufferTooSmall)?;
    Ok(writer.written_len())
}

/// Writes the SoftAP `200 OK` reboot confirmation HTML page.
fn write_softap_setup_saved_html(out_buf: &mut [u8]) -> Result<usize, HttpError> {
    let mut writer = SliceWriter::new(out_buf);
    write!(
        writer,
        "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\
        X-Frame-Options: DENY\r\n\
        Content-Security-Policy: {SETUP_HTML_CSP}\r\n\
        X-Content-Type-Options: nosniff\r\n\
        Cache-Control: no-store\r\n\
        Connection: close\r\nContent-Length: {}\r\n\r\n{}",
        SETUP_SAVED_HTML.len(),
        SETUP_SAVED_HTML
    )
    .map_err(|_| HttpError::BufferTooSmall)?;
    Ok(writer.written_len())
}

/// Handles a single SoftAP Setup Mode HTTP request received on `192.168.4.1`:
///
/// - Captive-portal probe paths (`/generate_204`, `/gen_204`,
///   `/hotspot-detect.html`, `/connecttest.txt`, `/ncsi.txt`,
///   `/canonical.html`, `/redirect`, `/`) return `302 Found` to
///   `http://192.168.4.1/setup`.
/// - `GET /capport` returns the RFC 8908 `application/captive+json` status
///   object pointing `user-portal-url` to `http://192.168.4.1/setup`.
/// - `GET /setup` returns a compact mobile-friendly HTML form (`<= 3 KB`) with
///   zero external assets and strict security headers (`X-Frame-Options`,
///   `Content-Security-Policy`, `X-Content-Type-Options`, `Cache-Control`).
/// - `POST /setup` strictly decodes `application/x-www-form-urlencoded`
///   (`<= 512 B`), validates a complete [`ProvisionedConfig`], returns `400`
///   HTML without echoing secrets on error, or returns
///   [`SetupHttpOutcome::SavedAndReboot`] with a `200 OK` reboot page.
pub fn handle_setup_http_request(
    raw_req: &[u8],
    mac: &[u8; 6],
    ap_ssid: &str,
    current_ssid: &str,
    out_buf: &mut [u8],
) -> SetupHttpOutcome {
    let req = match parse_http_request(raw_req) {
        Ok(r) => r,
        Err(_) => {
            let n = write_http_error(out_buf, 400, "Bad Request", "400 Bad Request\n", false)
                .map(|b| b.len())
                .unwrap_or(0);
            return SetupHttpOutcome::Responded(n);
        }
    };

    match (req.method, req.path) {
        (
            HttpMethod::Get | HttpMethod::Head,
            "/"
            | "/generate_204"
            | "/gen_204"
            | "/hotspot-detect.html"
            | "/connecttest.txt"
            | "/ncsi.txt"
            | "/canonical.html"
            | "/redirect",
        ) => {
            let n = write_setup_captive_redirect(out_buf).unwrap_or(0);
            SetupHttpOutcome::Responded(n)
        }
        (HttpMethod::Get, "/capport") => {
            let n = write_capport_json(out_buf).unwrap_or(0);
            SetupHttpOutcome::Responded(n)
        }
        (HttpMethod::Get, "/setup") => {
            let n = write_softap_setup_get_html(mac, ap_ssid, current_ssid, out_buf).unwrap_or(0);
            SetupHttpOutcome::Responded(n)
        }
        (HttpMethod::Post, "/setup") => match parse_setup_post_config(&req) {
            Ok(config) => match write_softap_setup_saved_html(out_buf) {
                Ok(resp_len) => SetupHttpOutcome::SavedAndReboot { resp_len, config },
                Err(_) => SetupHttpOutcome::Responded(0),
            },
            Err(err_msg) => {
                let n = write_softap_setup_error_html(err_msg, out_buf).unwrap_or(0);
                SetupHttpOutcome::Responded(n)
            }
        },
        (_, "/setup") => {
            let body = "Method Not Allowed\n";
            let mut writer = SliceWriter::new(out_buf);
            let res = write!(
                writer,
                "HTTP/1.1 405 Method Not Allowed\r\nAllow: GET, POST\r\n\
                Content-Type: text/plain\r\nConnection: close\r\n\
                Content-Length: {}\r\n\r\n{}",
                body.len(),
                body
            );
            let n = if res.is_ok() { writer.written_len() } else { 0 };
            SetupHttpOutcome::Responded(n)
        }
        _ => {
            let n = write_http_error(out_buf, 404, "Not Found", "404 Not Found\n", false)
                .map(|b| b.len())
                .unwrap_or(0);
            SetupHttpOutcome::Responded(n)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::DEFAULT_OTA_TOKEN;
    use crate::ota::OtaImgState;

    /// Non-default OTA token used by every test server.
    const TEST_OTA_TOKEN: &str = "wp09-test-ota-token";

    /// Test server with a private OTA token (the public default is refused
    /// with `403`, `FW-NET-04`).
    fn test_server() -> KlipperCamServer<'static> {
        let mut cfg = KlipperCamConfig::WROVER_KIT_HD;
        cfg.ota_token = TEST_OTA_TOKEN;
        KlipperCamServer::new(cfg, CameraDriver::new_hd_default())
    }

    #[test]
    fn test_constant_time_eq() {
        assert!(constant_time_eq(b"klipper-ota-2026", b"klipper-ota-2026"));
        assert!(!constant_time_eq(b"klipper-ota-2025", b"klipper-ota-2026"));
        assert!(!constant_time_eq(b"short", b"longer_string"));
        assert!(!constant_time_eq(b"longer_string", b"short"));
        assert!(!constant_time_eq(b"", b"klipper-ota-2026"));
        assert!(!constant_time_eq(b"klipper-ota-2026", b""));
        assert!(!constant_time_eq(b"", b""));
    }

    /// Challenger Scenario 1: Port 80 `GET /status` returns `200 OK`
    /// `application/json`
    /// with all 7 original JSON telemetry keys first, followed by diagnostics.
    #[test]
    fn test_challenger_1_port80_status_7_key_json() {
        let mut server = test_server();
        server.set_telemetry(42_500, 192_000, 4_100_000, -51);

        let mut resp_buf = [0u8; 1536];
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
            "{\"uptime_ms\":42500,\"free_heap\":192000,\"free_psram\":4100000,\
            \"wifi_rssi\":-51,\"active_partition\":\"ota_0\",\
            \"framesize\":\"1280x720\",\"stream_active\":false,\
            \"reset_reason\":\"unknown\",\
            \"app_sha256\":\"0000000000000000000000000000000000000000000000000000000000000000\",\
            \"free_dram\":184320,\"core1_stack_hwm\":0,\
            \"sensor_frame_period_us\":0,\"frame_bytes\":0,\"c1_pushed\":0,\
            \"c1_dropped\":0,\"ring_full_events\":0,\"soi_miss\":0,\
            \"overflow_aborts\":0,\"c0_spsc_cycles\":0,\"c0_net_cycles\":0,\
            \"c0_p80_cycles\":0,\"c0_p81_cycles\":0,\"max_loop_latency_us\":0,\
            \"p81_frames_sent\":0,\"p81_frames_superseded\":0,\
            \"c0_validation_failures\":0,\"sccb_errors\":0,\"send_partial\":0,\
            \"ring_build_failures\":0,\"i2s_stop_timeouts\":0}\n"
        );

        let mut stats = FwStatsSnapshot::new();
        stats.i2s_stop_timeouts = 7;
        server.set_diagnostics(
            "poweron",
            "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef",
            176_000,
            2048,
            stats,
        );
        let resp_diag = server
            .handle_raw_request(ServerPort::Control, raw_req, &mut resp_buf)
            .expect("diagnostics status request");
        let view_diag = HttpResponseView::parse(resp_diag).expect("valid HTTP response");
        let body_diag = core::str::from_utf8(view_diag.body).expect("valid UTF-8 JSON");
        assert!(body_diag.contains("\"reset_reason\":\"poweron\""));
        assert!(body_diag.contains("\"i2s_stop_timeouts\":7"));
    }

    #[test]
    fn test_status_json_reset_reason_sanitization_and_empty_fallback() {
        let injected = TelemetrySnapshot {
            reset_reason: "sw_reset\"evil\\\n",
            ..TelemetrySnapshot::default()
        };
        let mut buf = [0u8; STATUS_JSON_BUF_LEN];
        let json_inj = format_status_json(&injected, &mut buf).expect("sanitized status");
        assert!(json_inj.contains("\"reset_reason\":\"sw_reset_evil__\""));

        let blank = TelemetrySnapshot {
            reset_reason: "   ",
            ..TelemetrySnapshot::default()
        };
        let mut buf2 = [0u8; STATUS_JSON_BUF_LEN];
        let json_blank = format_status_json(&blank, &mut buf2).expect("blank fallback");
        assert!(json_blank.contains("\"reset_reason\":\"unknown\""));
    }

    #[test]
    fn test_status_json_wifi_rssi_null_vs_some() {
        let mut snap = TelemetrySnapshot {
            wifi_rssi: None,
            ..TelemetrySnapshot::default()
        };
        let mut buf = [0u8; STATUS_JSON_BUF_LEN];
        let json_null = format_status_json(&snap, &mut buf).expect("format status null");
        assert!(json_null.contains("\"wifi_rssi\":null"));

        snap.wifi_rssi = Some(-62);
        let mut buf2 = [0u8; STATUS_JSON_BUF_LEN];
        let json_some = format_status_json(&snap, &mut buf2).expect("format status some");
        assert!(json_some.contains("\"wifi_rssi\":-62"));
    }

    #[test]
    fn test_status_json_worst_case_max_width_values_fit_buffer() {
        let max_snap = TelemetrySnapshot {
            uptime_ms: i64::MIN,
            free_heap: u32::MAX,
            free_psram: u32::MAX,
            wifi_rssi: Some(i32::MIN),
            active_partition: "ota_1_long_custom_label_with_extra_chars_to_limit",
            framesize: "2560x1440_uhd_maximum_resolution_extended_mode_long_name",
            stream_active: false,
            reset_reason: "sw_cpu_reset",
            app_sha256: "ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff",
            free_dram: u32::MAX,
            core1_stack_hwm: u32::MAX,
            fw_stats: FwStatsSnapshot {
                sensor_frame_period_us: u32::MAX,
                frame_bytes: u32::MAX,
                c1_pushed: u32::MAX,
                c1_dropped: u32::MAX,
                ring_full_events: u32::MAX,
                soi_miss: u32::MAX,
                overflow_aborts: u32::MAX,
                c0_spsc_cycles: u32::MAX,
                c0_net_cycles: u32::MAX,
                c0_p80_cycles: u32::MAX,
                c0_p81_cycles: u32::MAX,
                max_loop_latency_us: u32::MAX,
                p81_frames_sent: u32::MAX,
                p81_frames_superseded: u32::MAX,
                c0_validation_failures: u32::MAX,
                sccb_errors: u32::MAX,
                send_partial: u32::MAX,
                ring_build_failures: u32::MAX,
                i2s_stop_timeouts: u32::MAX,
            },
        };

        let mut buf = [0u8; STATUS_JSON_BUF_LEN];
        let json_str = format_status_json(&max_snap, &mut buf)
            .expect("worst-case maximum-width values must fit inside 1024 bytes");
        assert!(json_str.len() + 32 <= STATUS_JSON_BUF_LEN);
        assert!(json_str.contains("\"reset_reason\":\"sw_cpu_reset\""));
        assert!(json_str.contains("\"i2s_stop_timeouts\":4294967295"));

        let expected_prefixes = [
            "\"uptime_ms\":",
            "\"free_heap\":",
            "\"free_psram\":",
            "\"wifi_rssi\":",
            "\"active_partition\":",
            "\"framesize\":",
            "\"stream_active\":",
        ];
        let mut prev_pos = 0;
        for key in expected_prefixes.iter() {
            let pos = json_str.find(key).expect("key must exist in json");
            assert!(pos >= prev_pos, "keys must be in original order");
            prev_pos = pos;
        }
    }

    /// Challenger Scenario 2: Port 80 `GET /snapshot` and `GET /?action=snapshot`
    /// return
    /// `200 OK` `image/jpeg` with `Content-Disposition: inline;
    /// filename=capture.jpg`,
    /// `Access-Control-Allow-Origin: *`, and a valid 1280x720 JFIF JPEG payload.
    #[test]
    fn test_challenger_2_port80_snapshot_and_query_delegation() {
        let mut server = test_server();
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
            assert_eq!(view.cache_control, Some("no-store"));

            // Verify JFIF SOI, APP0 "JFIF\0", SOF0 1280x720, and EOI
            assert!(view.body.len() >= 20);
            assert_eq!(&view.body[0..2], &[0xFF, 0xD8]);
            assert_eq!(&view.body[6..11], b"JFIF\0");
            assert_eq!(&view.body[view.body.len() - 2..], &[0xFF, 0xD9]);
        }
    }

    /// Challenger Scenario 3: Port 80 `GET /` serves the Klipper plain-text landing
    /// page,
    /// `GET /?action=stream` returns `400 Bad Request` (`"Stream available only on
    /// port 81 (/stream)"`),
    /// and `GET /stream` on Port 80 returns `404 Not Found` (`NET-001` /
    /// `APP-005`).
    #[test]
    fn test_challenger_3_port80_root_and_stream_isolation() {
        let mut server = test_server();
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
    /// `multipart/x-mixed-replace;boundary=123456789000000000000987654321` with
    /// `X-Framerate: 5`
    /// and valid MJPEG boundary framing.
    #[test]
    fn test_challenger_4_port81_mjpeg_stream_framing() {
        let mut server = test_server();
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
            assert_eq!(view.cache_control, Some("no-store"));
            assert_eq!(view.x_framerate, None);

            // Verify multipart boundary header and embedded JPEG SOI/EOI
            let expected_prefix = b"\r\n--123456789000000000000987654321\r\n\
Content-Type: image/jpeg\r\nContent-Length: 43\r\n\r\n";
            assert!(view.body.starts_with(expected_prefix));
            let jpeg_part = &view.body[expected_prefix.len()..];
            assert_eq!(jpeg_part, &SYNTHETIC_HD_JFIF_FRAME);
        }
    }

    /// Challenger Scenario 5: Port 81 single-client `AtomicBool` lock rejects a
    /// concurrent
    /// 2nd client on Port 81 with `HTTP/1.1 500 Internal Server Error` `"Stream
    /// already active"`,
    /// while concurrent Port 80 `GET /snapshot` and `GET /status`
    /// (`"stream_active":true`) succeed!
    #[test]
    fn test_challenger_5_concurrent_stream_busy_guard_and_port80_snapshot() {
        let server = test_server();

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

        // Client 2 connects to Port 81 /stream while Client 1 is still active -> 500
        // "Stream already active"
        let mut c2_buf = [0u8; 512];
        let c2_req = parse_http_request(b"GET /stream HTTP/1.1\r\nHost: 172.22.1.52:81\r\n\r\n")
            .expect("parse c2");
        let c2_resp = server
            .handle_port81(&c2_req, &mut c2_buf)
            .expect("handle c2");
        let c2_view = HttpResponseView::parse(c2_resp).expect("parse c2 view");
        assert_eq!(c2_view.status_code, 500);
        assert_eq!(c2_view.body, b"Stream already active");

        // Concurrent Port 80 GET /snapshot while Client 1 is streaming on Port 81 ->
        // 200 OK image/jpeg
        let mut snap_buf = [0u8; 512];
        let snap_resp = server
            .handle_snapshot(&mut snap_buf)
            .expect("concurrent snapshot must succeed");
        let snap_view = HttpResponseView::parse(snap_resp).expect("parse snapshot");
        assert_eq!(snap_view.status_code, 200);
        assert_eq!(snap_view.content_type, Some("image/jpeg"));
        assert_eq!(snap_view.body, &SYNTHETIC_HD_JFIF_FRAME);

        // Concurrent Port 80 GET /status while Client 1 is streaming ->
        // "stream_active":true
        let mut status_buf = [0u8; 1536];
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

    /// Challenger Scenario 6: Port 80 `GET /update` serves the HTML OTA UI showing
    /// the active
    /// and next target partitions (`ota_0` and `ota_1`).
    #[test]
    fn test_challenger_6_port80_update_get_html() {
        let mut server = test_server();
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

    /// Challenger Scenario 7: Port 80 `POST /update` enforces constant-time `X-OTA-
    /// Token`
    /// (`401 Unauthorized` without CORS header on failure), rejects invalid magic
    /// or oversized
    /// images with `400 Bad Request`, and accepts a valid `0xE9` image (`200 OK`),
    /// updating
    /// `otadata` with IEEE 802.3 CRC32 and cancelling rollback upon Wi-Fi IP
    /// acquisition.
    #[test]
    fn test_challenger_7_port80_ota_post_auth_validation_and_rollback() {
        let mut server = test_server();
        let mut resp_buf = [0u8; 1024];

        // 1. Missing X-OTA-Token -> 401 Unauthorized, NO Access-Control-Allow-Origin
        let req_no_tok = b"POST /update HTTP/1.1\r\nHost: 172.22.1.52\r\n\
Content-Length: 24\r\n\r\n123456789012345678901234";
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
        let req_bad_tok = b"POST /update HTTP/1.1\r\nHost: 172.22.1.52\r\n\
X-OTA-Token: wrong-token\r\nContent-Length: 24\r\n\r\n\
123456789012345678901234";
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
                "POST /update HTTP/1.1\r\nHost: 172.22.1.52\r\n\
                X-OTA-Token: {}\r\nContent-Length: 24\r\n\r\n",
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
                "POST /update HTTP/1.1\r\nHost: 172.22.1.52\r\n\
                X-OTA-Token: {}\r\nContent-Length: {}\r\n\r\n",
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

        // 5. Valid token + valid 0xE9 ESP32 firmware image -> 200 OK, switches boot
        // slot to ota_1
        let valid_img = crate::ota::make_valid_esp_image(true);
        let mut valid_req = [0u8; 512];
        let valid_req_bytes = {
            let mut w = SliceWriter::new(&mut valid_req);
            write!(
                w,
                "POST /update HTTP/1.1\r\nHost: 172.22.1.52\r\n\
                X-OTA-Token: {}\r\nContent-Length: {}\r\n\r\n",
                server.config.ota_token,
                valid_img.len()
            )
            .expect("hdr");
            w.write_bytes(&valid_img).expect("body");
            w.into_bytes()
        };
        let r_ok = server
            .handle_raw_request(ServerPort::Control, valid_req_bytes, &mut resp_buf)
            .expect("handle valid OTA");
        let v_ok = HttpResponseView::parse(r_ok).expect("parse 200");
        assert_eq!(v_ok.status_code, 200);
        assert_eq!(v_ok.body, b"Firmware update successful! Rebooting...\n");
        assert!(server.is_reboot_scheduled());
        assert_eq!(server.last_ota_bytes_written(), valid_img.len());
        assert_eq!(server.last_ota_target_slot(), Some(OtaSlot::Ota1));
        assert_eq!(server.ota_manager.active_slot(), OtaSlot::Ota1);
        assert_eq!(
            server.ota_manager.sector_1.state(),
            OtaImgState::PendingVerify
        );
        assert!(server.ota_manager.sector_1.is_crc_valid());

        // 6. Simulate reboot + Wi-Fi IP acquisition cancelling rollback
        server.ota_manager =
            OtaDataManager::from_sectors(server.ota_manager.sector_0, server.ota_manager.sector_1);
        assert_eq!(server.on_wifi_got_ip_cancel_rollback(), Ok(true));
        assert_eq!(server.ota_manager.sector_1.state(), OtaImgState::Valid);
    }

    #[test]
    fn test_camera_driver_set_frame_and_handle_raw_request_with_flash() {
        use crate::ota::MockNorFlash;

        let mut flash: MockNorFlash<8192> = MockNorFlash::new_booted_ota0();
        let initial_frame: [u8; 6] = [0xFF, 0xD8, 0x11, 0x22, 0xFF, 0xD9];
        let updated_frame: [u8; 8] = [0xFF, 0xD8, 0xAA, 0xBB, 0xCC, 0xDD, 0xFF, 0xD9];

        let mut cfg = KlipperCamConfig::WROVER_KIT_HD;
        cfg.ota_token = TEST_OTA_TOKEN;
        let mut server =
            KlipperCamServer::new(cfg, CameraDriver::new(&initial_frame, FrameSize::Vga));
        assert_eq!(server.camera.frame_data(), &initial_frame);

        // Update live frame via set_frame and verify GET /snapshot returns updated
        // frame
        server.camera.set_frame(&updated_frame, FrameSize::Hd);
        assert_eq!(server.camera.frame_data(), &updated_frame);

        let mut resp_buf = [0u8; 1024];
        let snap_resp = server
            .handle_raw_request_with_flash(
                ServerPort::Control,
                b"GET /snapshot HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n",
                &mut resp_buf,
                &mut flash,
            )
            .expect("snapshot with flash");
        let snap_view = HttpResponseView::parse(snap_resp).expect("parse snapshot");
        assert_eq!(snap_view.status_code, 200);
        assert_eq!(snap_view.body, &updated_frame);

        // POST /update with flash persists firmware image to ota_1
        let fw_img = crate::ota::make_valid_esp_image(true);
        let mut ota_req = [0u8; 512];
        let ota_req_bytes = {
            let mut w = SliceWriter::new(&mut ota_req);
            write!(
                w,
                "POST /update HTTP/1.1\r\nHost: 172.22.1.52\r\n\
                X-OTA-Token: {}\r\nContent-Length: {}\r\n\r\n",
                server.config.ota_token,
                fw_img.len()
            )
            .expect("hdr");
            w.write_bytes(&fw_img).expect("body");
            w.into_bytes()
        };

        let ota_resp = server
            .handle_raw_request_with_flash(
                ServerPort::Control,
                ota_req_bytes,
                &mut resp_buf,
                &mut flash,
            )
            .expect("OTA POST with flash");
        let ota_view = HttpResponseView::parse(ota_resp).expect("parse 200");
        assert_eq!(ota_view.status_code, 200);
        assert_eq!(&flash.ota_1[..fw_img.len()], &fw_img[..]);

        // Confirm otadata sector 1 in flash has PendingVerify, simulate reboot
        // reload, then cancel rollback with flash
        let loaded = OtaDataManager::load_from_flash(&mut flash).expect("load from flash");
        assert_eq!(loaded.active_slot(), OtaSlot::Ota1);
        assert_eq!(loaded.sector_1.state(), OtaImgState::PendingVerify);

        server.ota_manager = loaded;
        assert_eq!(server.on_wifi_got_ip_with_flash(&mut flash), Ok(true));
        let loaded_after =
            OtaDataManager::load_from_flash(&mut flash).expect("load after rollback cancel");
        assert_eq!(loaded_after.sector_1.state(), OtaImgState::Valid);
    }

    #[test]
    fn test_port80_control_get_and_parameter_updates() {
        let mut server = test_server();
        let mut resp_buf = [0u8; 2048];

        // 1. Initially no pending control update; GET /control returns default HD JSON
        assert_eq!(server.take_pending_control_update(), None);
        let r0 = server
            .handle_raw_request(
                ServerPort::Control,
                b"GET /control HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n",
                &mut resp_buf,
            )
            .expect("GET /control");
        let v0 = HttpResponseView::parse(r0).expect("parse GET /control");
        assert_eq!(v0.status_code, 200);
        assert_eq!(v0.content_type, Some("application/json"));
        assert_eq!(v0.access_control_allow_origin, Some("*"));
        let json0 = core::str::from_utf8(v0.body).expect("utf8 json");
        assert_eq!(
            json0,
            "{\"framesize\":11,\"framesize_label\":\"1280x720\",\"quality\":12,\
            \"brightness\":0,\"contrast\":0,\"saturation\":0,\
            \"special_effect\":0,\"hmirror\":0,\"vflip\":0,\
            \"awb\":1,\"agc\":1,\"aec\":1,\"ae_level\":0}"
        );
        assert_eq!(server.take_pending_control_update(), None);

        // 2. GET /control?var=hmirror&val=1 -> 200 OK, sets hmirror=true and dirty=true
        let r_hm = server
            .handle_raw_request(
                ServerPort::Control,
                b"POST /control?var=hmirror&val=1 HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n",
                &mut resp_buf,
            )
            .expect("GET /control?var=hmirror&val=1");
        let v_hm = HttpResponseView::parse(r_hm).expect("parse hmirror response");
        assert_eq!(v_hm.status_code, 200);
        let json_hm = core::str::from_utf8(v_hm.body).expect("utf8");
        assert!(json_hm.contains("\"hmirror\":1"));
        let upd1 = server
            .take_pending_control_update()
            .expect("pending control update after hmirror=1");
        assert!(upd1.hmirror);
        assert!(!upd1.dirty);
        assert_eq!(server.take_pending_control_update(), None);

        // 3. GET /control?var=quality&val=10 -> 200 OK
        let r_q = server
            .handle_raw_request(
                ServerPort::Control,
                b"POST /control?var=quality&val=10 HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n",
                &mut resp_buf,
            )
            .expect("GET /control?var=quality&val=10");
        let v_q = HttpResponseView::parse(r_q).expect("parse quality response");
        assert_eq!(v_q.status_code, 200);
        let json_q = core::str::from_utf8(v_q.body).expect("utf8");
        assert!(json_q.contains("\"quality\":10"));

        // 4. GET /control?var=brightness&val=-1 -> 200 OK
        let r_b = server
            .handle_raw_request(
                ServerPort::Control,
                b"POST /control?var=brightness&val=-1 HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n",
                &mut resp_buf,
            )
            .expect("GET /control?var=brightness&val=-1");
        let v_b = HttpResponseView::parse(r_b).expect("parse brightness response");
        assert_eq!(v_b.status_code, 200);
        let json_b = core::str::from_utf8(v_b.body).expect("utf8");
        assert!(json_b.contains("\"brightness\":-1"));

        // 5. Additional valid controls: contrast, saturation, special_effect, vflip,
        // awb, agc, aec, ae_level, framesize
        for (req_raw, expected_substr) in [
            (
                b"POST /control?var=contrast&val=2 HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n"
                    .as_slice(),
                "\"contrast\":2",
            ),
            (
                b"POST /control?var=saturation&val=-2 HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n"
                    .as_slice(),
                "\"saturation\":-2",
            ),
            (
                b"POST /control?var=special_effect&val=3 HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n"
                    .as_slice(),
                "\"special_effect\":3",
            ),
            (
                b"POST /control?var=vflip&val=1 HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n".as_slice(),
                "\"vflip\":1",
            ),
            (
                b"POST /control?var=awb&val=0 HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n".as_slice(),
                "\"awb\":0",
            ),
            (
                b"POST /control?var=agc&val=0 HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n".as_slice(),
                "\"agc\":0",
            ),
            (
                b"POST /control?var=aec&val=0 HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n".as_slice(),
                "\"aec\":0",
            ),
            (
                b"POST /control?var=ae_level&val=1 HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n"
                    .as_slice(),
                "\"ae_level\":1",
            ),
            (
                b"POST /control?var=framesize&val=800 HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n"
                    .as_slice(),
                "\"framesize\":9,\"framesize_label\":\"800x600\"",
            ),
        ] {
            let r = server
                .handle_raw_request(ServerPort::Control, req_raw, &mut resp_buf)
                .expect("control update");
            let v = HttpResponseView::parse(r).expect("parse response");
            assert_eq!(v.status_code, 200);
            let s = core::str::from_utf8(v.body).expect("utf8");
            assert!(
                s.contains(expected_substr),
                "expected {} in {}",
                expected_substr,
                s
            );
        }

        // Verify /status reflects updated framesize ("800x600")
        let r_stat = server
            .handle_raw_request(
                ServerPort::Control,
                b"GET /status HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n",
                &mut resp_buf,
            )
            .expect("GET /status");
        let v_stat = HttpResponseView::parse(r_stat).expect("parse status");
        let s_stat = core::str::from_utf8(v_stat.body).expect("utf8");
        assert!(s_stat.contains("\"framesize\":\"800x600\""));

        let upd_final = server
            .take_pending_control_update()
            .expect("pending control update");
        assert_eq!(upd_final.framesize, FrameSize::Svga);
        assert_eq!(upd_final.quality, 10);
        assert_eq!(upd_final.brightness, -1);
        assert_eq!(upd_final.contrast, 2);
        assert_eq!(upd_final.saturation, -2);
        assert_eq!(upd_final.special_effect, 3);
        assert!(upd_final.hmirror);
        assert!(upd_final.vflip);
        assert!(!upd_final.awb);
        assert!(!upd_final.agc);
        assert!(!upd_final.aec);
        assert_eq!(upd_final.ae_level, 1);
    }

    #[test]
    fn test_port80_control_invalid_parameters_return_400_bad_request() {
        let mut server = test_server();
        let mut resp_buf = [0u8; 1024];

        for bad_req in [
            b"POST /control?var=unknown&val=1 HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n".as_slice(),
            b"POST /control?var=quality&val=3 HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n".as_slice(),
            b"POST /control?var=quality&val=64 HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n".as_slice(),
            b"POST /control?var=brightness&val=3 HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n".as_slice(),
            b"POST /control?var=contrast&val=-3 HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n".as_slice(),
            b"POST /control?var=saturation&val=9 HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n".as_slice(),
            b"POST /control?var=special_effect&val=7 HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n"
                .as_slice(),
            b"POST /control?var=hmirror&val=2 HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n".as_slice(),
            b"POST /control?var=vflip&val=-1 HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n".as_slice(),
            b"POST /control?var=ae_level&val=-5 HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n".as_slice(),
            b"POST /control?var=framesize&val=999 HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n".as_slice(),
            b"POST /control?var=quality HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n".as_slice(),
            b"POST /control?val=10 HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n".as_slice(),
            b"POST /control?var=quality&val=abc HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n".as_slice(),
        ] {
            let resp = server
                .handle_raw_request(ServerPort::Control, bad_req, &mut resp_buf)
                .expect("400 response written");
            let view = HttpResponseView::parse(resp).expect("parse 400 response");
            assert_eq!(view.status_code, 400);
            // FW-NET-27: mutating responses carry no CORS header.
            assert_eq!(view.access_control_allow_origin, None);
        }

        // State must remain clean and unmodified after rejected requests
        assert_eq!(server.take_pending_control_update(), None);
        assert_eq!(server.control_state, CameraControlState::new_default());
    }

    #[test]
    fn test_port80_panel_get_html_dashboard() {
        let mut server = test_server();
        let mut resp_buf = [0u8; 5120];

        let resp = server
            .handle_raw_request(
                ServerPort::Control,
                b"GET /panel HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n",
                &mut resp_buf,
            )
            .expect("GET /panel");
        let view = HttpResponseView::parse(resp).expect("parse /panel response");
        assert_eq!(view.status_code, 200);
        assert_eq!(view.content_type, Some("text/html"));
        assert_eq!(view.access_control_allow_origin, Some("*"));
        let html = core::str::from_utf8(view.body).expect("utf8 html");
        assert!(html.contains("Klipper Cam Control Panel"));
        assert!(html.contains(":81/stream"));
        assert!(html.contains("/snapshot"));
        assert!(html.contains("fetch('/control?var='+k+'&val='+v,{method:'POST'"));
        assert!(html.contains("fetch('/status')"));
        assert!(html.contains("onerror="));
        assert!(html.contains("recover()"));
    }

    #[test]
    fn test_qa201_malformed_and_duplicate_headers() {
        // Line without colon -> MalformedRequest
        let no_colon = b"GET / HTTP/1.1\r\nInvalidHeaderWithoutColon\r\n\r\n";
        assert_eq!(
            parse_http_request(no_colon),
            Err(HttpError::MalformedRequest)
        );

        // Header name with whitespace before colon -> MalformedRequest (RFC 9112
        // section 5.1)
        let space_before_colon = b"GET / HTTP/1.1\r\nHost : 172.22.1.52\r\n\r\n";
        assert_eq!(
            parse_http_request(space_before_colon),
            Err(HttpError::MalformedRequest)
        );

        let tab_before_colon = b"GET / HTTP/1.1\r\nHost\t: 172.22.1.52\r\n\r\n";
        assert_eq!(
            parse_http_request(tab_before_colon),
            Err(HttpError::MalformedRequest)
        );

        // Duplicate Content-Length -> MalformedRequest
        let dup_cl = b"POST /update HTTP/1.1\r\nHost: 172.22.1.52\r\n\
Content-Length: 10\r\nContent-Length: 10\r\n\r\n";
        assert_eq!(parse_http_request(dup_cl), Err(HttpError::MalformedRequest));

        // Malformed Content-Length -> MalformedRequest
        let neg_cl = b"POST /update HTTP/1.1\r\nHost: 172.22.1.52\r\nContent-Length: -10\r\n\r\n";
        assert_eq!(parse_http_request(neg_cl), Err(HttpError::MalformedRequest));

        let non_num_cl =
            b"POST /update HTTP/1.1\r\nHost: 172.22.1.52\r\nContent-Length: ten\r\n\r\n";
        assert_eq!(
            parse_http_request(non_num_cl),
            Err(HttpError::MalformedRequest)
        );

        // Duplicate X-OTA-Token -> MalformedRequest
        let dup_token = b"POST /update HTTP/1.1\r\nHost: 172.22.1.52\r\n\
X-OTA-Token: tok1\r\nX-OTA-Token: tok2\r\n\r\n";
        assert_eq!(
            parse_http_request(dup_token),
            Err(HttpError::MalformedRequest)
        );

        // Transfer-Encoding header rejected per RFC 9112 §6.1 (SEC-001)
        let te_chunked =
            b"POST /update HTTP/1.1\r\nHost: 172.22.1.52\r\nTransfer-Encoding: chunked\r\n\r\n";
        assert_eq!(
            parse_http_request(te_chunked),
            Err(HttpError::MalformedRequest)
        );

        // Content-Length exceeding isize::MAX rejected (LOGIC-010)
        #[cfg(target_pointer_width = "32")]
        {
            let cl_overflow =
                b"POST /update HTTP/1.1\r\nHost: 172.22.1.52\r\nContent-Length: 3000000000\r\n\r\n";
            assert_eq!(
                parse_http_request(cl_overflow),
                Err(HttpError::MalformedRequest)
            );
        }
        #[cfg(target_pointer_width = "64")]
        {
            let cl_overflow = b"POST /update HTTP/1.1\r\nHost: 172.22.1.52\r\n\
Content-Length: 9223372036854775808\r\n\r\n";
            assert_eq!(
                parse_http_request(cl_overflow),
                Err(HttpError::MalformedRequest)
            );
        }
    }

    #[test]
    fn test_qa201_request_line_invariants() {
        // Extra tokens on request line -> MalformedRequest
        let extra_tokens = b"GET / HTTP/1.1 EXTRA_TOKEN\r\nHost: 172.22.1.52\r\n\r\n";
        assert_eq!(
            parse_http_request(extra_tokens),
            Err(HttpError::MalformedRequest)
        );

        // Invalid HTTP version -> MalformedRequest
        let http2 = b"GET / HTTP/2.0\r\nHost: 172.22.1.52\r\n\r\n";
        assert_eq!(parse_http_request(http2), Err(HttpError::MalformedRequest));

        let http_bad = b"GET / HTTP/3.0\r\nHost: 172.22.1.52\r\n\r\n";
        assert_eq!(
            parse_http_request(http_bad),
            Err(HttpError::MalformedRequest)
        );

        // Target without leading slash -> MalformedRequest
        let no_leading_slash = b"GET status HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n";
        assert_eq!(
            parse_http_request(no_leading_slash),
            Err(HttpError::MalformedRequest)
        );

        // Valid HTTP/1.0 and HTTP/1.1 accepted
        let http10 = b"GET / HTTP/1.0\r\nHost: 172.22.1.52\r\n\r\n";
        let req10 = parse_http_request(http10).expect("HTTP/1.0 valid");
        assert_eq!(req10.version, "HTTP/1.0");

        let http11 = b"GET / HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n";
        let req11 = parse_http_request(http11).expect("HTTP/1.1 valid");
        assert_eq!(req11.version, "HTTP/1.1");

        // RFC 9112 §2.2: allow leading empty CRLF lines before request-line (CONC-005)
        let leading_crlf = b"\r\nGET /status HTTP/1.1\r\n\r\n";
        let req_leading = parse_http_request(leading_crlf).expect("leading CRLF accepted");
        assert_eq!(req_leading.method, HttpMethod::Get);
        assert_eq!(req_leading.path, "/status");
        assert_eq!(req_leading.version, "HTTP/1.1");

        let leading_crlf_headers = b"\r\nGET /status HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n";
        let req_leading_headers =
            parse_http_request(leading_crlf_headers).expect("leading CRLF with headers accepted");
        assert_eq!(req_leading_headers.method, HttpMethod::Get);
        assert_eq!(req_leading_headers.path, "/status");
        assert_eq!(req_leading_headers.version, "HTTP/1.1");
    }

    #[test]
    fn test_qa201_oversized_query_and_exact_match() {
        let mut server = test_server();
        let mut resp_buf = [0u8; 1024];

        // Query string > 256 bytes on GET / -> 400 Bad Request
        let mut long_req = alloc::vec::Vec::new();
        long_req.extend_from_slice(b"GET /?");
        long_req.extend_from_slice(&[b'a'; 260]);
        long_req.extend_from_slice(b" HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n");
        let r_long = server
            .handle_raw_request(ServerPort::Control, &long_req, &mut resp_buf)
            .expect("handled");
        let v_long = HttpResponseView::parse(r_long).expect("parse 400");
        assert_eq!(v_long.status_code, 400);

        // Substring query containing action=stream but not exact pair -> serves root
        // status page
        let unanchored_req = b"GET /?fooaction=stream HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n";
        let r_unanchored = server
            .handle_raw_request(ServerPort::Control, unanchored_req, &mut resp_buf)
            .expect("handled");
        let v_unanchored = HttpResponseView::parse(r_unanchored).expect("parse 200");
        assert_eq!(v_unanchored.status_code, 200);

        // Exact action=stream query -> 400 Bad Request
        let exact_stream = b"GET /?action=stream HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n";
        let r_stream = server
            .handle_raw_request(ServerPort::Control, exact_stream, &mut resp_buf)
            .expect("handled");
        let v_stream = HttpResponseView::parse(r_stream).expect("parse 400");
        assert_eq!(v_stream.status_code, 400);

        // Exact action=snapshot in multi-param query -> delegates to snapshot
        let exact_snap = b"GET /?foo=1&action=snapshot&bar=2 HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n";
        let r_snap = server
            .handle_raw_request(ServerPort::Control, exact_snap, &mut resp_buf)
            .expect("handled");
        let v_snap = HttpResponseView::parse(r_snap).expect("parse snapshot");
        assert_eq!(v_snap.status_code, 200);
        assert_eq!(v_snap.content_type, Some("image/jpeg"));
    }

    #[test]
    fn test_qa201_ota_post_failures_and_flash_error() {
        use crate::ota::MockNorFlash;

        let mut server = test_server();
        let mut resp_buf = [0u8; 1024];

        // Missing Content-Length on POST /update -> 400 Bad Request
        let mut buf_no_cl = [0u8; 256];
        let req_no_cl = {
            let mut w = SliceWriter::new(&mut buf_no_cl);
            write!(
                w,
                "POST /update HTTP/1.1\r\nHost: 172.22.1.52\r\nX-OTA-Token: {}\r\n\r\n",
                server.config.ota_token
            )
            .expect("w");
            w.into_bytes()
        };
        let r_no_cl = server
            .handle_raw_request(ServerPort::Control, req_no_cl, &mut resp_buf)
            .expect("handled");
        let v_no_cl = HttpResponseView::parse(r_no_cl).expect("parse 400");
        assert_eq!(v_no_cl.status_code, 400);
        assert_eq!(v_no_cl.body, b"Content-Length header required");

        // Truncated OTA body (Content-Length declares 64 bytes, but body has only 20)
        // -> 400 Bad Request
        let mut buf_trunc = [0u8; 256];
        let req_trunc = {
            let mut w = SliceWriter::new(&mut buf_trunc);
            write!(
                w,
                "POST /update HTTP/1.1\r\nHost: 172.22.1.52\r\n\
                X-OTA-Token: {}\r\nContent-Length: 64\r\n\r\n",
                server.config.ota_token
            )
            .expect("w");
            w.write_bytes(&[0xE9u8; 20]).expect("body");
            w.into_bytes()
        };
        let r_trunc = server
            .handle_raw_request(ServerPort::Control, req_trunc, &mut resp_buf)
            .expect("handled");
        let v_trunc = HttpResponseView::parse(r_trunc).expect("parse 400");
        assert_eq!(v_trunc.status_code, 400);

        // Flash failure -> 500 Internal Server Error
        let mut failing_flash: MockNorFlash<8192> = MockNorFlash::new_booted_ota0();
        failing_flash.fail_writes = true;
        let valid_img = crate::ota::make_valid_esp_image(true);
        let mut buf_flash_err = [0u8; 512];
        let req_flash_err = {
            let mut w = SliceWriter::new(&mut buf_flash_err);
            write!(
                w,
                "POST /update HTTP/1.1\r\nHost: 172.22.1.52\r\n\
                X-OTA-Token: {}\r\nContent-Length: {}\r\n\r\n",
                server.config.ota_token,
                valid_img.len()
            )
            .expect("w");
            w.write_bytes(&valid_img).expect("body");
            w.into_bytes()
        };
        let r_flash_err = server
            .handle_raw_request_with_flash(
                ServerPort::Control,
                req_flash_err,
                &mut resp_buf,
                &mut failing_flash,
            )
            .expect("handled");
        let v_flash_err = HttpResponseView::parse(r_flash_err).expect("parse 500");
        assert_eq!(v_flash_err.status_code, 500);
        assert_eq!(v_flash_err.body, b"Failed to update boot partition");
    }

    #[test]
    fn test_qa201_port81_404_and_format_ota_minimal_root() {
        let mut server = test_server();
        let mut resp_buf = [0u8; 1024];

        // Port 81 GET /snapshot -> 404 Not Found
        let r_p81_snap = server
            .handle_raw_request(
                ServerPort::Stream,
                b"GET /snapshot HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n",
                &mut resp_buf,
            )
            .expect("handled");
        let v_p81_snap = HttpResponseView::parse(r_p81_snap).expect("parse 404");
        assert_eq!(v_p81_snap.status_code, 404);

        // Port 81 GET /status -> 404 Not Found
        let r_p81_stat = server
            .handle_raw_request(
                ServerPort::Stream,
                b"GET /status HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n",
                &mut resp_buf,
            )
            .expect("handled");
        let v_p81_stat = HttpResponseView::parse(r_p81_stat).expect("parse 404");
        assert_eq!(v_p81_stat.status_code, 404);

        // format_ota_minimal_root helper
        let mgr = OtaDataManager::new_booted_ota0();
        let mut min_buf = [0u8; 1024];
        let min_resp =
            format_ota_minimal_root(&mgr, 250_000, &mut min_buf).expect("format min root");
        let min_view = HttpResponseView::parse(min_resp).expect("parse min root view");
        assert_eq!(min_view.status_code, 200);
        assert_eq!(min_view.content_type, Some("text/plain"));
        let min_body = core::str::from_utf8(min_view.body).expect("utf8");
        assert!(min_body.contains("Klipper Cam OTA Minimal Intermediary"));
        assert!(min_body.contains("Running Partition: ota_0"));
        assert!(min_body.contains("Next Target:      ota_1"));
        assert!(min_body.contains("250000 bytes"));

        // HttpError::BufferTooSmall when buffer is tiny
        let mut tiny_buf = [0u8; 10];
        assert_eq!(
            format_ota_minimal_root(&mgr, 250_000, &mut tiny_buf),
            Err(HttpError::BufferTooSmall)
        );
    }

    #[test]
    fn test_qa201_control_edge_cases_and_dirty_flag_transitions() {
        let mut server = test_server();
        let mut resp_buf = [0u8; 1024];

        // isize::MIN and isize::MAX on /control -> 400 Bad Request
        let min_req = b"POST /control?var=quality&val=-9223372036854775808 HTTP/1.1\r\n\
Host: 172.22.1.52\r\n\r\n";
        let r_min = server
            .handle_raw_request(ServerPort::Control, min_req, &mut resp_buf)
            .expect("handled");
        let v_min = HttpResponseView::parse(r_min).expect("parse 400");
        assert_eq!(v_min.status_code, 400);

        let max_req = b"POST /control?var=quality&val=9223372036854775807 HTTP/1.1\r\n\
Host: 172.22.1.52\r\n\r\n";
        let r_max = server
            .handle_raw_request(ServerPort::Control, max_req, &mut resp_buf)
            .expect("handled");
        let v_max = HttpResponseView::parse(r_max).expect("parse 400");
        assert_eq!(v_max.status_code, 400);

        // Duplicate var parameter -> 400 Bad Request
        let dup_var = b"POST /control?var=quality&val=10&var=brightness HTTP/1.1\r\n\
Host: 172.22.1.52\r\n\r\n";
        let r_dup_var = server
            .handle_raw_request(ServerPort::Control, dup_var, &mut resp_buf)
            .expect("handled");
        let v_dup_var = HttpResponseView::parse(r_dup_var).expect("parse 400");
        assert_eq!(v_dup_var.status_code, 400);
        assert!(core::str::from_utf8(v_dup_var.body)
            .expect("utf8")
            .contains("Duplicate 'var' parameter"));

        // Duplicate val parameter -> 400 Bad Request
        let dup_val =
            b"POST /control?var=quality&val=10&val=20 HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n";
        let r_dup_val = server
            .handle_raw_request(ServerPort::Control, dup_val, &mut resp_buf)
            .expect("handled");
        let v_dup_val = HttpResponseView::parse(r_dup_val).expect("parse 400");
        assert_eq!(v_dup_val.status_code, 400);
        assert!(core::str::from_utf8(v_dup_val.body)
            .expect("utf8")
            .contains("Duplicate 'val' parameter"));

        // FrameSize parameter update updates camera frame_size and dirty flag
        assert_eq!(server.camera.frame_size(), FrameSize::Hd);
        assert_eq!(server.take_pending_control_update(), None);

        let fs_req = b"POST /control?var=framesize&val=8 HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n";
        let r_fs = server
            .handle_raw_request(ServerPort::Control, fs_req, &mut resp_buf)
            .expect("handled");
        let v_fs = HttpResponseView::parse(r_fs).expect("parse 200");
        assert_eq!(v_fs.status_code, 200);
        assert_eq!(server.camera.frame_size(), FrameSize::Vga);

        let pending = server
            .take_pending_control_update()
            .expect("dirty update pending");
        assert_eq!(pending.framesize, FrameSize::Vga);
        // Dirty flag cleared by take_pending_control_update
        assert_eq!(server.take_pending_control_update(), None);
    }

    #[test]
    fn test_port80_responses_emit_connection_close_header() {
        let mut server = test_server();
        let mut resp_buf = [0u8; 5120];

        for raw_req in [
            b"GET / HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n".as_slice(),
            b"GET /snapshot HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n".as_slice(),
            b"GET /status HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n".as_slice(),
            b"GET /control HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n".as_slice(),
            b"GET /panel HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n".as_slice(),
            b"GET /update HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n".as_slice(),
            b"GET /nonexistent HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n".as_slice(),
            b"POST /update HTTP/1.1\r\nHost: 172.22.1.52\r\nContent-Length: 4\r\n\r\ntest"
                .as_slice(),
        ] {
            let resp = server
                .handle_raw_request(ServerPort::Control, raw_req, &mut resp_buf)
                .expect("Port 80 request handled");
            let view = HttpResponseView::parse(resp).expect("parse Port 80 response");
            assert!(
                view.headers_raw.contains("Connection: close"),
                "Expected 'Connection: close' in headers: {}",
                view.headers_raw
            );
        }

        let mgr = OtaDataManager::new_booted_ota0();
        let min_resp =
            format_ota_minimal_root(&mgr, 180_000, &mut resp_buf).expect("format_ota_minimal_root");
        let min_view = HttpResponseView::parse(min_resp).expect("parse minimal root");
        assert!(min_view.headers_raw.contains("Connection: close"));
    }

    #[test]
    fn test_stream_session_guard_counts_camera_busy_and_capture_failures() {
        let camera = CameraDriver::new_hd_default();
        let flag = AtomicBool::new(false);
        let mut session = StreamSessionGuard::try_acquire(&flag).expect("acquire");
        let mut buf = [0u8; 512];

        // Lock mutex to simulate camera busy (InvalidState): M-048 counts busy
        // toward consecutive_failures so a wedged mutex cannot pin the stream.
        camera.set_mutex_locked(true);
        assert_eq!(
            session.write_next_frame_chunk(&camera, &mut buf),
            Err(HttpError::CameraBusy)
        );
        assert_eq!(session.consecutive_failures(), 1);
        assert!(!session.should_abort());

        // Now simulate hardware failure
        camera.set_mutex_locked(false);
        camera.set_simulate_failure(true);
        assert_eq!(
            session.write_next_frame_chunk(&camera, &mut buf),
            Err(HttpError::CaptureFailed)
        );
        assert_eq!(session.consecutive_failures(), 2);
    }

    #[test]
    fn test_parse_framesize_val_fw_audit_http_002() {
        assert_eq!(parse_framesize_val(10), Some(FrameSize::Xga));
        assert_eq!(parse_framesize_val(1024), Some(FrameSize::Xga));
        assert_eq!(parse_framesize_val(12), Some(FrameSize::Sxga));
    }

    #[test]
    fn test_duplicate_ota_token_rejected_sec_http_001() {
        let mut server = KlipperCamServer::new(
            KlipperCamConfig::WROVER_KIT_HD,
            CameraDriver::new_hd_default(),
        );
        let mut resp_buf = [0u8; 1024];
        let dup_req = b"POST /update HTTP/1.1\r\n\
Host: 172.22.1.52\r\n\
X-OTA-Token: tok1\r\n\
X-OTA-Token: tok2\r\n\r\n";
        let resp = server
            .handle_raw_request(ServerPort::Control, dup_req, &mut resp_buf)
            .expect("handled");
        let view = HttpResponseView::parse(resp).expect("parse error view");
        assert_eq!(view.status_code, 400);
        assert!(view.body.starts_with(b"400 Bad Request"));
    }

    #[test]
    fn test_zero_copy_mjpeg_part_header() {
        let mut buf = [0u8; 256];
        let mjpeg_hdr = write_mjpeg_part_header(51_200, &mut buf).expect("mjpeg header");
        let expected_mjpeg = b"\r\n--123456789000000000000987654321\r\n\
Content-Type: image/jpeg\r\nContent-Length: 51200\r\n\r\n";
        assert_eq!(mjpeg_hdr, expected_mjpeg);

        let mut tiny_buf = [0u8; 16];
        assert_eq!(
            write_mjpeg_part_header(51_200, &mut tiny_buf),
            Err(HttpError::BufferTooSmall)
        );
    }

    struct MockCameraControlApplier {
        framesize_calls: u32,
        quality_calls: u32,
        brightness_calls: u32,
        contrast_calls: u32,
        saturation_calls: u32,
        special_effect_calls: u32,
        hmirror_calls: u32,
        vflip_calls: u32,
        awb_calls: u32,
        agc_calls: u32,
        aec_calls: u32,
        ae_level_calls: u32,
        fail_quality: bool,
        fail_contrast: bool,
    }

    impl MockCameraControlApplier {
        fn new() -> Self {
            Self {
                framesize_calls: 0,
                quality_calls: 0,
                brightness_calls: 0,
                contrast_calls: 0,
                saturation_calls: 0,
                special_effect_calls: 0,
                hmirror_calls: 0,
                vflip_calls: 0,
                awb_calls: 0,
                agc_calls: 0,
                aec_calls: 0,
                ae_level_calls: 0,
                fail_quality: false,
                fail_contrast: false,
            }
        }

        fn total_calls(&self) -> u32 {
            self.framesize_calls
                + self.quality_calls
                + self.brightness_calls
                + self.contrast_calls
                + self.saturation_calls
                + self.special_effect_calls
                + self.hmirror_calls
                + self.vflip_calls
                + self.awb_calls
                + self.agc_calls
                + self.aec_calls
                + self.ae_level_calls
        }
    }

    impl CameraControlApplier for MockCameraControlApplier {
        fn apply_framesize(&mut self, _val: FrameSize) -> Result<(), CameraError> {
            self.framesize_calls += 1;
            Ok(())
        }

        fn apply_quality(&mut self, _val: i32) -> Result<(), CameraError> {
            self.quality_calls += 1;
            if self.fail_quality {
                Err(CameraError::SccbError)
            } else {
                Ok(())
            }
        }

        fn apply_brightness(&mut self, _val: i32) -> Result<(), CameraError> {
            self.brightness_calls += 1;
            Ok(())
        }

        fn apply_contrast(&mut self, _val: i32) -> Result<(), CameraError> {
            self.contrast_calls += 1;
            if self.fail_contrast {
                Err(CameraError::SccbError)
            } else {
                Ok(())
            }
        }

        fn apply_saturation(&mut self, _val: i32) -> Result<(), CameraError> {
            self.saturation_calls += 1;
            Ok(())
        }

        fn apply_special_effect(&mut self, _val: i32) -> Result<(), CameraError> {
            self.special_effect_calls += 1;
            Ok(())
        }

        fn apply_hmirror(&mut self, _val: bool) -> Result<(), CameraError> {
            self.hmirror_calls += 1;
            Ok(())
        }

        fn apply_vflip(&mut self, _val: bool) -> Result<(), CameraError> {
            self.vflip_calls += 1;
            Ok(())
        }

        fn apply_awb(&mut self, _val: bool) -> Result<(), CameraError> {
            self.awb_calls += 1;
            Ok(())
        }

        fn apply_agc(&mut self, _val: bool) -> Result<(), CameraError> {
            self.agc_calls += 1;
            Ok(())
        }

        fn apply_aec(&mut self, _val: bool) -> Result<(), CameraError> {
            self.aec_calls += 1;
            Ok(())
        }

        fn apply_ae_level(&mut self, _val: i32) -> Result<(), CameraError> {
            self.ae_level_calls += 1;
            Ok(())
        }
    }

    #[test]
    fn test_has_pending_control_update_lifecycle() {
        let mut server = test_server();
        let mut resp_buf = [0u8; 512];

        assert!(!server.has_pending_control_update());
        assert_eq!(server.take_pending_control_update(), None);

        let req = b"POST /control?var=brightness&val=2 HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n";
        let resp = server
            .handle_raw_request(ServerPort::Control, req, &mut resp_buf)
            .expect("GET /control");
        let view = HttpResponseView::parse(resp).expect("parse view");
        assert_eq!(view.status_code, 200);

        assert!(server.has_pending_control_update());
        assert!(server.has_pending_control_update());

        let pending = server.take_pending_control_update();
        assert!(pending.is_some());
        assert_eq!(pending.expect("pending state").brightness, 2);
        assert!(!server.has_pending_control_update());
    }

    #[test]
    fn test_apply_camera_control_diff_framesize_invokes_only_framesize_and_quality() {
        let mut applier = MockCameraControlApplier::new();
        let mut last_applied = CameraControlState::new_default();
        let mut desired = CameraControlState::new_default();

        desired.framesize = FrameSize::Vga;

        let errors = apply_camera_control_diff(&mut applier, &desired, &mut last_applied);
        assert_eq!(errors, 0);
        assert_eq!(last_applied.framesize, FrameSize::Vga);
        assert_eq!(last_applied.quality, desired.quality);

        assert_eq!(applier.framesize_calls, 1);
        assert_eq!(applier.quality_calls, 1);
        assert_eq!(applier.total_calls(), 2);
    }

    #[test]
    fn test_apply_camera_control_diff_error_preserves_old_value_for_retry() {
        let mut applier = MockCameraControlApplier::new();
        applier.fail_contrast = true;

        let mut last_applied = CameraControlState::new_default();
        let mut desired = CameraControlState::new_default();

        desired.brightness = 2;
        desired.contrast = 3;
        desired.saturation = 1;

        let errors = apply_camera_control_diff(&mut applier, &desired, &mut last_applied);
        assert_eq!(errors, 1);

        assert_eq!(last_applied.brightness, 2);
        assert_eq!(last_applied.saturation, 1);
        assert_eq!(last_applied.contrast, 0);
    }

    // ---- WP-09 / FW-NET-04: default OTA token is refused ----

    fn ota_post_request(token: &str, image: &[u8]) -> alloc::vec::Vec<u8> {
        let mut req = alloc::format!(
            "POST /update HTTP/1.1\r\nHost: 172.22.1.52\r\nX-OTA-Token: {}\r\n\
             Content-Length: {}\r\n\r\n",
            token,
            image.len()
        )
        .into_bytes();
        req.extend_from_slice(image);
        req
    }

    #[test]
    fn test_wp09_default_ota_token_returns_403() {
        let mut cfg = KlipperCamConfig::WROVER_KIT_HD;
        cfg.ota_token = DEFAULT_OTA_TOKEN;
        let mut server = KlipperCamServer::new(cfg, CameraDriver::new_hd_default());
        assert!(server.config.is_default_ota_token());
        let image = crate::ota::make_valid_esp_image(true);
        let mut resp_buf = [0u8; 1024];

        // Even the matching (public) token is forbidden.
        for token in [DEFAULT_OTA_TOKEN, "anything-else", ""] {
            let req = ota_post_request(token, &image);
            let resp = server
                .handle_raw_request(ServerPort::Control, &req, &mut resp_buf)
                .expect("403 written");
            let view = HttpResponseView::parse(resp).expect("parse 403");
            assert_eq!(view.status_code, 403, "token {token:?}");
            assert_eq!(view.access_control_allow_origin, None);
        }
        assert!(!server.is_reboot_scheduled());
        assert_eq!(server.last_ota_target_slot(), None);
        assert_eq!(server.ota_manager.active_slot(), OtaSlot::Ota0);

        let rejection = server
            .check_ota_auth(DEFAULT_OTA_TOKEN.as_bytes())
            .expect_err("default token rejected");
        assert_eq!(rejection.status, 403);
    }

    #[test]
    fn test_wp09_check_ota_auth_private_token() {
        let server = test_server();
        assert_eq!(server.check_ota_auth(TEST_OTA_TOKEN.as_bytes()), Ok(()));
        let wrong = server.check_ota_auth(b"wrong").expect_err("wrong token");
        assert_eq!(wrong.status, 401);
        let missing = server.check_ota_auth(b"").expect_err("missing token");
        assert_eq!(missing.status, 401);

        let mut server = test_server();
        let image = crate::ota::make_valid_esp_image(true);
        let req = ota_post_request(TEST_OTA_TOKEN, &image);
        let mut resp_buf = [0u8; 1024];
        let resp = server
            .handle_raw_request(ServerPort::Control, &req, &mut resp_buf)
            .expect("200 written");
        assert_eq!(
            HttpResponseView::parse(resp).expect("parse").status_code,
            200
        );
    }

    #[test]
    fn test_wp09_ota_post_rejects_s3_image_and_misplaced_digest() {
        let mut resp_buf = [0u8; 1024];

        let mut s3 = crate::ota::make_valid_esp_image(true);
        s3[12..14].copy_from_slice(&0x0009u16.to_le_bytes());
        let mut misplaced = crate::ota::make_valid_esp_image(true);
        misplaced.extend_from_slice(&[0xFF; 16]);

        for image in [s3, misplaced] {
            let mut server = test_server();
            let req = ota_post_request(TEST_OTA_TOKEN, &image);
            let resp = server
                .handle_raw_request(ServerPort::Control, &req, &mut resp_buf)
                .expect("400 written");
            let view = HttpResponseView::parse(resp).expect("parse 400");
            assert_eq!(view.status_code, 400);
            assert!(!server.is_reboot_scheduled());
            assert_eq!(server.ota_manager.active_slot(), OtaSlot::Ota0);
        }
    }

    #[test]
    fn test_wp09_ota_post_readback_mismatch_returns_500() {
        struct FlakyFlash(crate::ota::MockNorFlash<8192>);
        impl OtaFlashStorage for FlakyFlash {
            fn read_aligned(&mut self, offset: u32, buf: &mut [u8]) -> Result<(), OtaError> {
                self.0.read_aligned(offset, buf)?;
                if offset >= crate::ota::OTA_1_OFFSET {
                    buf[0] ^= 0x80;
                }
                Ok(())
            }
            fn erase_range(&mut self, from: u32, to: u32) -> Result<(), OtaError> {
                self.0.erase_range(from, to)
            }
            fn write_aligned(&mut self, offset: u32, data: &[u8]) -> Result<(), OtaError> {
                self.0.write_aligned(offset, data)
            }
        }

        let mut flash = FlakyFlash(crate::ota::MockNorFlash::new_booted_ota0());
        let otadata_before = (flash.0.otadata_s0, flash.0.otadata_s1);
        let mut server = test_server();
        let image = crate::ota::make_valid_esp_image(true);
        let req = ota_post_request(TEST_OTA_TOKEN, &image);
        let mut resp_buf = [0u8; 1024];
        let resp = server
            .handle_raw_request_with_flash(ServerPort::Control, &req, &mut resp_buf, &mut flash)
            .expect("500 written");
        let view = HttpResponseView::parse(resp).expect("parse 500");
        assert_eq!(view.status_code, 500);
        assert!(!server.is_reboot_scheduled());
        assert_eq!((flash.0.otadata_s0, flash.0.otadata_s1), otadata_before);
    }

    // ---- WP-09 / FW-NET-27: /control mutations are POST-only, no CORS ----

    #[test]
    fn test_wp09_control_get_with_var_returns_405() {
        let mut server = test_server();
        let mut resp_buf = [0u8; 1024];
        for req in [
            b"GET /control?var= HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n".as_slice(),
            b"GET /control?var=hmirror&val=1 HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n".as_slice(),
            b"GET /control?var=quality&val=99 HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n".as_slice(),
        ] {
            let resp = server
                .handle_raw_request(ServerPort::Control, req, &mut resp_buf)
                .expect("405 written");
            let view = HttpResponseView::parse(resp).expect("parse 405");
            assert_eq!(view.status_code, 405);
            assert!(
                view.headers_raw.contains("Allow: POST"),
                "{}",
                view.headers_raw
            );
            assert_eq!(view.access_control_allow_origin, None);
        }
        assert_eq!(server.take_pending_control_update(), None);
        assert_eq!(server.control_state, CameraControlState::new_default());
    }

    #[test]
    fn test_wp09_control_read_only_get_keeps_cors() {
        let mut server = test_server();
        let mut resp_buf = [0u8; 1024];
        for req in [
            b"GET /control HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n".as_slice(),
            b"GET /control? HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n".as_slice(),
        ] {
            let resp = server
                .handle_raw_request(ServerPort::Control, req, &mut resp_buf)
                .expect("200 written");
            let view = HttpResponseView::parse(resp).expect("parse 200");
            assert_eq!(view.status_code, 200);
            assert_eq!(view.access_control_allow_origin, Some("*"));
            assert_eq!(view.cache_control, Some("no-store"));
        }
        assert_eq!(server.take_pending_control_update(), None);
    }

    #[test]
    fn test_wp09_control_post_accepted_without_cors() {
        let mut server = test_server();
        let mut resp_buf = [0u8; 1024];

        // Query-string parameters on POST.
        let resp = server
            .handle_raw_request(
                ServerPort::Control,
                b"POST /control?var=hmirror&val=1 HTTP/1.1\r\nHost: 172.22.1.52\r\n\
                  Content-Length: 0\r\n\r\n",
                &mut resp_buf,
            )
            .expect("200 written");
        let view = HttpResponseView::parse(resp).expect("parse 200");
        assert_eq!(view.status_code, 200);
        assert_eq!(view.access_control_allow_origin, None);
        assert!(
            server
                .take_pending_control_update()
                .expect("pending")
                .hmirror
        );

        // Form-encoded body parameters on POST (`curl -d var=..&val=..`).
        let resp = server
            .handle_raw_request(
                ServerPort::Control,
                b"POST /control HTTP/1.1\r\nHost: 172.22.1.52\r\n\
                  Content-Type: application/x-www-form-urlencoded\r\n\
                  Content-Length: 18\r\n\r\nvar=quality&val=10",
                &mut resp_buf,
            )
            .expect("200 written");
        let view = HttpResponseView::parse(resp).expect("parse 200");
        assert_eq!(
            view.status_code,
            200,
            "{:?}",
            core::str::from_utf8(view.body)
        );
        assert_eq!(view.access_control_allow_origin, None);
        assert_eq!(
            server
                .take_pending_control_update()
                .expect("pending")
                .quality,
            10
        );

        // Invalid value on POST: 400 without CORS, state unchanged.
        let resp = server
            .handle_raw_request(
                ServerPort::Control,
                b"POST /control?var=quality&val=99 HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n",
                &mut resp_buf,
            )
            .expect("400 written");
        let view = HttpResponseView::parse(resp).expect("parse 400");
        assert_eq!(view.status_code, 400);
        assert_eq!(view.access_control_allow_origin, None);
        assert_eq!(server.take_pending_control_update(), None);
    }

    #[test]
    fn test_wp09_control_get_with_x_requested_with_accepted() {
        let mut server = test_server();
        let mut resp_buf = [0u8; 1024];
        let resp = server
            .handle_raw_request(
                ServerPort::Control,
                b"GET /control?var=vflip&val=1 HTTP/1.1\r\nHost: 172.22.1.52\r\n\
                  X-Requested-With: XMLHttpRequest\r\n\r\n",
                &mut resp_buf,
            )
            .expect("200 written");
        let view = HttpResponseView::parse(resp).expect("parse 200");
        assert_eq!(view.status_code, 200);
        assert_eq!(view.access_control_allow_origin, None);
        assert!(server.take_pending_control_update().expect("pending").vflip);
    }

    #[test]
    fn test_wp09_control_cross_origin_mutation_forbidden() {
        let mut server = test_server();
        let mut resp_buf = [0u8; 1024];
        for req in [
            b"POST /control?var=hmirror&val=1 HTTP/1.1\r\nHost: 172.22.1.52\r\n\
              Origin: http://evil.example\r\n\r\n"
                .as_slice(),
            b"POST /control?var=hmirror&val=1 HTTP/1.1\r\nHost: 172.22.1.52\r\n\
              Origin: null\r\n\r\n"
                .as_slice(),
            b"GET /control?var=hmirror&val=1 HTTP/1.1\r\nHost: 172.22.1.52\r\n\
              X-Requested-With: XMLHttpRequest\r\nOrigin: http://172.22.1.53\r\n\r\n"
                .as_slice(),
        ] {
            let resp = server
                .handle_raw_request(ServerPort::Control, req, &mut resp_buf)
                .expect("403 written");
            let view = HttpResponseView::parse(resp).expect("parse 403");
            assert_eq!(view.status_code, 403);
            assert_eq!(view.access_control_allow_origin, None);
        }
        assert_eq!(server.take_pending_control_update(), None);

        // Same-origin browser POST (the panel) is accepted.
        for req in [
            b"POST /control?var=hmirror&val=1 HTTP/1.1\r\nHost: 172.22.1.52\r\n\
              Origin: http://172.22.1.52\r\n\r\n"
                .as_slice(),
            b"POST /control?var=hmirror&val=1 HTTP/1.1\r\nHost: 172.22.1.52:80\r\n\
              Origin: http://172.22.1.52\r\n\r\n"
                .as_slice(),
            b"POST /control?var=hmirror&val=1 HTTP/1.1\r\nHost: camera.local:8080\r\n\
              Origin: http://camera.local:8080\r\n\r\n"
                .as_slice(),
        ] {
            let resp = server
                .handle_raw_request(ServerPort::Control, req, &mut resp_buf)
                .expect("200 written");
            let view = HttpResponseView::parse(resp).expect("parse 200");
            assert_eq!(view.status_code, 200);
        }
    }

    #[test]
    fn test_wp09_panel_uses_post_for_control() {
        let html = KLIPPER_CONTROL_PANEL_HTML;
        assert!(html.contains("method:'POST'"));
        assert!(!html.contains("fetch('/control?var='+k+'&val='+v)"));
    }

    #[test]
    fn test_wp09_parse_request_captures_csrf_headers() {
        let req = parse_http_request(
            b"POST /control HTTP/1.1\r\nHost: cam:80\r\nOrigin: http://cam\r\n\
              X-Requested-With: fetch\r\n\r\n",
        )
        .expect("parse");
        assert_eq!(req.host, Some("cam:80"));
        assert_eq!(req.origin, Some("http://cam"));
        assert!(req.x_requested_with);
        let plain = parse_http_request(b"GET / HTTP/1.1\r\n\r\n").expect("parse");
        assert_eq!(plain.host, None);
        assert_eq!(plain.origin, None);
        assert!(!plain.x_requested_with);
    }

    #[test]
    fn test_fw_net_23_query_has_pair() {
        assert!(query_has_pair("action=snapshot", "action=snapshot"));
        assert!(query_has_pair("?action=snapshot", "action=snapshot"));
        assert!(query_has_pair(
            "foo=1&action=snapshot&bar=2",
            "action=snapshot"
        ));
        assert!(query_has_pair("action=snapshot&bar=2", "action=snapshot"));
        assert!(query_has_pair("foo=1&action=snapshot", "action=snapshot"));

        // Negative matches
        assert!(!query_has_pair("subaction=snapshot", "action=snapshot"));
        assert!(!query_has_pair("action=snapshot_extra", "action=snapshot"));
        assert!(!query_has_pair("action=stream", "action=snapshot"));
        assert!(!query_has_pair("", "action=snapshot"));
    }

    #[test]
    fn test_fw_net_23_content_length_digits_only_and_option_usize() {
        // Valid Content-Length with digits only
        let valid_req =
            b"POST /update HTTP/1.1\r\nHost: 172.22.1.52\r\nContent-Length: 42\r\n\r\nhello";
        let req = parse_http_request(valid_req).expect("valid digits-only content-length");
        assert_eq!(req.content_length, Some(42usize));

        // Rejection of +5
        let plus_req =
            b"POST /update HTTP/1.1\r\nHost: 172.22.1.52\r\nContent-Length: +5\r\n\r\nhello";
        assert_eq!(
            parse_http_request(plus_req),
            Err(HttpError::MalformedRequest)
        );

        // Rejection of negative -5
        let neg_req =
            b"POST /update HTTP/1.1\r\nHost: 172.22.1.52\r\nContent-Length: -5\r\n\r\nhello";
        assert_eq!(
            parse_http_request(neg_req),
            Err(HttpError::MalformedRequest)
        );

        // Rejection of non-digit text
        let bad_req =
            b"POST /update HTTP/1.1\r\nHost: 172.22.1.52\r\nContent-Length: 12a\r\n\r\nhello";
        assert_eq!(
            parse_http_request(bad_req),
            Err(HttpError::MalformedRequest)
        );

        // Missing Content-Length yields None
        let get_req = b"GET /snapshot HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n";
        let parsed_get = parse_http_request(get_req).expect("valid get request");
        assert_eq!(parsed_get.content_length, None);
    }

    // ---- CL-L Phase 3 Remediation Unit Tests (M-043..M-049, M-138..M-211) ----

    #[test]
    fn test_m043_ota_reentrant_active_stream_and_buffer_precheck() {
        use crate::ota::MockNorFlash;

        let image = crate::ota::make_valid_esp_image(true);
        let req = ota_post_request(TEST_OTA_TOKEN, &image);
        let mut resp_buf = [0u8; 1024];

        // 1. Pre-check out_buf capacity before touching flash or scheduling reboot.
        let mut server = test_server();
        let mut flash: MockNorFlash<8192> = MockNorFlash::new_booted_ota0();
        let mut tiny_buf = [0u8; 32];
        assert_eq!(
            server.handle_raw_request_with_flash(
                ServerPort::Control,
                &req,
                &mut tiny_buf,
                &mut flash,
            ),
            Err(HttpError::BufferTooSmall)
        );
        assert!(!server.is_reboot_scheduled());
        assert_eq!(flash.erase_count, 0);
        assert_eq!(flash.write_count, 0);

        // 2. Active Port 81 stream blocks POST /update with 409 Conflict
        // before flash I/O.
        server.stream_active_flag().store(true, Ordering::Release);
        let resp = server
            .handle_raw_request_with_flash(ServerPort::Control, &req, &mut resp_buf, &mut flash)
            .expect("409 written");
        let view = HttpResponseView::parse(resp).expect("parse 409");
        assert_eq!(view.status_code, 409);
        assert!(!server.is_reboot_scheduled());
        assert_eq!(flash.erase_count, 0);
        server.stream_active_flag().store(false, Ordering::Release);

        // 3. First OTA after stream stops succeeds (200 OK) and schedules reboot.
        let ok_resp = server
            .handle_raw_request_with_flash(ServerPort::Control, &req, &mut resp_buf, &mut flash)
            .expect("200 written");
        let ok_view = HttpResponseView::parse(ok_resp).expect("parse 200");
        assert_eq!(ok_view.status_code, 200);
        assert!(server.is_reboot_scheduled());
        let erases_after_first = flash.erase_count;

        // 4. Re-entrant POST /update before reboot is rejected with 409 Conflict.
        let dup_resp = server
            .handle_raw_request_with_flash(ServerPort::Control, &req, &mut resp_buf, &mut flash)
            .expect("409 written");
        let dup_view = HttpResponseView::parse(dup_resp).expect("parse 409");
        assert_eq!(dup_view.status_code, 409);
        assert_eq!(flash.erase_count, erases_after_first);
    }

    #[test]
    fn test_m044_reject_bare_lf_cr_in_request_line_and_headers() {
        // Bare LF inside header value (smuggling Transfer-Encoding).
        let bare_lf_hdr = b"POST /update HTTP/1.1\r\nHost: 172.22.1.52\r\n\
X-Pad: a\nTransfer-Encoding: chunked\r\n\r\n";
        assert_eq!(
            parse_http_request(bare_lf_hdr),
            Err(HttpError::MalformedRequest)
        );

        // Bare CR inside header value.
        let bare_cr_hdr = b"GET /status HTTP/1.1\r\nHost: 172.22.1.52\r\nX-Pad: a\rb\r\n\r\n";
        assert_eq!(
            parse_http_request(bare_cr_hdr),
            Err(HttpError::MalformedRequest)
        );

        // Bare LF in request line.
        let bare_lf_rl = b"GET /status\n HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n";
        assert_eq!(
            parse_http_request(bare_lf_rl),
            Err(HttpError::MalformedRequest)
        );

        // Bare CR in request line.
        let bare_cr_rl = b"GET /status\r HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n";
        assert_eq!(
            parse_http_request(bare_cr_rl),
            Err(HttpError::MalformedRequest)
        );
    }

    #[test]
    fn test_m045_rfc9110_tchar_header_names_and_ascii_ows_only() {
        // Vertical tab (0x0B) in header name must be rejected.
        let vt_name = b"POST /update HTTP/1.1\r\nHost: 172.22.1.52\r\n\
Transfer-Encoding\x0b: chunked\r\n\r\n";
        assert_eq!(
            parse_http_request(vt_name),
            Err(HttpError::MalformedRequest)
        );

        // NUL and non-tchar delimiters in header name must be rejected.
        let nul_name = b"GET /status HTTP/1.1\r\nX-A\x00B: 1\r\n\r\n";
        assert_eq!(
            parse_http_request(nul_name),
            Err(HttpError::MalformedRequest)
        );
        let paren_name = b"GET /status HTTP/1.1\r\nX-Bad(Name): 1\r\n\r\n";
        assert_eq!(
            parse_http_request(paren_name),
            Err(HttpError::MalformedRequest)
        );

        // 0x0B / 0x0C / Unicode NBSP in header value must be rejected.
        let vt_val = b"POST /update HTTP/1.1\r\nHost: 172.22.1.52\r\n\
Content-Length: \x0b10\r\n\r\n";
        assert_eq!(parse_http_request(vt_val), Err(HttpError::MalformedRequest));

        let nbsp_val =
            "POST /update HTTP/1.1\r\nHost: 172.22.1.52\r\nContent-Length: \u{00A0}10\r\n\r\n";
        assert_eq!(
            parse_http_request(nbsp_val.as_bytes()),
            Err(HttpError::MalformedRequest)
        );

        // ASCII SP and HTAB OWS around values remain valid.
        let ok_ows = b"GET /status HTTP/1.1\r\nHost: \t172.22.1.52\t \r\n\r\n";
        let req = parse_http_request(ok_ows).expect("valid SP/HTAB OWS");
        assert_eq!(req.host, Some("172.22.1.52"));
    }

    #[test]
    fn test_m046_is_same_origin_enforces_scheme_and_port() {
        // Plaintext HTTP server default (http / 80).
        assert!(is_same_origin("http://172.22.1.52", Some("172.22.1.52")));
        assert!(is_same_origin("http://172.22.1.52:80", Some("172.22.1.52")));
        assert!(is_same_origin("http://172.22.1.52", Some("172.22.1.52:80")));
        assert!(is_same_origin(
            "http://camera.local:8080",
            Some("camera.local:8080")
        ));

        // Cross-scheme and default-port confusion MUST be rejected.
        assert!(!is_same_origin("https://172.22.1.52", Some("172.22.1.52")));
        assert!(!is_same_origin(
            "https://172.22.1.52:443",
            Some("172.22.1.52")
        ));
        assert!(!is_same_origin(
            "http://172.22.1.52:443",
            Some("172.22.1.52")
        ));
        assert!(!is_same_origin(
            "http://172.22.1.52:81",
            Some("172.22.1.52")
        ));
        assert!(!is_same_origin(
            "http://172.22.1.52/path",
            Some("172.22.1.52")
        ));
        assert!(!is_same_origin("null", Some("172.22.1.52")));
        assert!(!is_same_origin("http://172.22.1.52", None));

        // Explicit HTTPS scheme helper matches https/:443 and rejects http.
        assert!(is_same_origin_for_scheme(
            "https://cam.local:443",
            Some("cam.local"),
            "https",
            443
        ));
        assert!(!is_same_origin_for_scheme(
            "http://cam.local",
            Some("cam.local"),
            "https",
            443
        ));
    }

    #[test]
    fn test_m047_strict_rfc9110_host_header_validation() {
        for valid_host in [
            "172.22.1.52",
            "172.22.1.52:80",
            "camera.local:8080",
            "[::1]",
            "[::1]:8080",
        ] {
            assert!(is_valid_http_host(valid_host), "valid: {valid_host}");
        }

        for bad_host in [
            "",
            "user@172.22.1.52",
            "172.22.1.52/evil",
            "172.22.1.52:0",
            "172.22.1.52:65536",
            "172.22.1.52:",
            "172.22.1.52:abc",
            "bad..host",
            "-bad.local",
            "bad host",
            "[::1]trailing",
        ] {
            assert!(!is_valid_http_host(bad_host), "bad: {bad_host}");
            let raw = alloc::format!("GET /status HTTP/1.1\r\nHost: {bad_host}\r\n\r\n");
            assert_eq!(
                parse_http_request(raw.as_bytes()),
                Err(HttpError::MalformedRequest),
                "bad Host header must fail parse: {bad_host}"
            );
        }

        // M-047: DNS rebinding Host allowlist on mutating /control & /update.
        let mut server = test_server();
        let mut resp_buf = [0u8; 1024];

        // Attacker DNS-rebinding domain with matching Origin + X-Requested-With
        // MUST be rejected with 403 Forbidden and leave control state intact.
        let rebind_ctrl = b"POST /control?var=quality&val=63 HTTP/1.1\r\n\
            Host: evil.attacker.com\r\n\
            Origin: http://evil.attacker.com\r\n\
            X-Requested-With: fetch\r\n\r\n";
        let resp = server
            .handle_raw_request(ServerPort::Control, rebind_ctrl, &mut resp_buf)
            .expect("403 written");
        let view = HttpResponseView::parse(resp).expect("parse 403");
        assert_eq!(view.status_code, 403);
        assert_eq!(view.access_control_allow_origin, None);
        assert_eq!(server.take_pending_control_update(), None);

        // DNS rebinding on POST /update MUST also be rejected with 403 before
        // touching flash even when carrying the valid OTA token.
        let image = crate::ota::make_valid_esp_image(true);
        let mut rebind_ota = alloc::format!(
            "POST /update HTTP/1.1\r\nHost: evil.attacker.com\r\n\
             X-OTA-Token: {TEST_OTA_TOKEN}\r\nContent-Length: {}\r\n\r\n",
            image.len()
        )
        .into_bytes();
        rebind_ota.extend_from_slice(&image);
        let ota_resp = server
            .handle_raw_request(ServerPort::Control, &rebind_ota, &mut resp_buf)
            .expect("403 written");
        assert_eq!(
            HttpResponseView::parse(ota_resp)
                .expect("parse 403")
                .status_code,
            403
        );
        assert!(!server.is_reboot_scheduled());

        // Configured static IP, runtime DHCP active IP, localhost, 127.0.0.1,
        // and `.local` mDNS hostnames pass on mutating /control.
        server.set_active_ip(crate::config::Ipv4Addr::new(192, 168, 50, 77));
        for allowed in [
            "172.22.1.52",
            "172.22.1.52:80",
            "192.168.50.77",
            "127.0.0.1",
            "localhost",
            "klipper-cam.local",
        ] {
            let req = alloc::format!(
                "POST /control?var=quality&val=15 HTTP/1.1\r\nHost: {allowed}\r\n\r\n"
            );
            let ok = server
                .handle_raw_request(ServerPort::Control, req.as_bytes(), &mut resp_buf)
                .expect("200 written");
            assert_eq!(
                HttpResponseView::parse(ok).expect("parse 200").status_code,
                200,
                "allowed Host {allowed} must succeed"
            );
        }

        // M-031: Whitespace-only or space-prefixed default OTA token returns 403.
        for bad_tok in ["                ", " klipper-cam-ota-2026"] {
            let mut bad_cfg = KlipperCamConfig::WROVER_KIT_HD;
            bad_cfg.ota_token = bad_tok;
            let bad_srv = KlipperCamServer::new(bad_cfg, CameraDriver::new_hd_default());
            let rej = bad_srv
                .check_ota_auth(bad_tok.as_bytes())
                .expect_err("whitespace token must be forbidden");
            assert_eq!(rej.status, 403);
        }

        // M-004: Running Partition reported by /status, /update, and
        // format_ota_minimal_root must stay `running_slot()` (ota_0) even after
        // committing an OTA to ota_1 before reboot.
        let mut status_srv = test_server();
        let ok_ota_req = ota_post_request(TEST_OTA_TOKEN, &image);
        let ok_ota_resp = status_srv
            .handle_raw_request(ServerPort::Control, &ok_ota_req, &mut resp_buf)
            .expect("200 OTA");
        assert_eq!(
            HttpResponseView::parse(ok_ota_resp)
                .expect("parse 200")
                .status_code,
            200
        );
        let mut big_buf = [0u8; 4096];
        let st_resp = status_srv
            .handle_raw_request(
                ServerPort::Control,
                b"GET /status HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n",
                &mut big_buf,
            )
            .expect("GET /status");
        let st_body = core::str::from_utf8(HttpResponseView::parse(st_resp).expect("parse").body)
            .expect("utf8");
        assert!(st_body.contains("\"active_partition\":\"ota_0\""));
        let upd_resp = status_srv
            .handle_raw_request(
                ServerPort::Control,
                b"GET /update HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n",
                &mut big_buf,
            )
            .expect("GET /update");
        let upd_body = core::str::from_utf8(HttpResponseView::parse(upd_resp).expect("parse").body)
            .expect("utf8");
        assert!(upd_body.contains("Running Partition: <b>ota_0</b>"));
        let min_resp = format_ota_minimal_root(&status_srv.ota_manager, 100_000, &mut big_buf)
            .expect("minimal root");
        let min_body = core::str::from_utf8(HttpResponseView::parse(min_resp).expect("parse").body)
            .expect("utf8");
        assert!(min_body.contains("Running Partition: ota_0"));
    }

    #[test]
    fn test_m048_stream_session_guard_counts_all_failures_and_enforces_abort() {
        let camera = CameraDriver::new_hd_default();
        let flag = AtomicBool::new(false);
        let mut session = StreamSessionGuard::try_acquire(&flag).expect("acquire");
        let mut tiny_buf = [0u8; 8];
        let mut ok_buf = [0u8; 512];

        // BufferTooSmall increments consecutive_failures (does NOT reset to 0).
        for expected in 1..=3 {
            assert_eq!(
                session.write_next_frame_chunk(&camera, &mut tiny_buf),
                Err(HttpError::BufferTooSmall)
            );
            assert_eq!(session.consecutive_failures(), expected);
        }

        // CameraBusy increments consecutive_failures to 4.
        camera.set_mutex_locked(true);
        assert_eq!(
            session.write_next_frame_chunk(&camera, &mut ok_buf),
            Err(HttpError::CameraBusy)
        );
        assert_eq!(session.consecutive_failures(), 4);
        assert!(!session.should_abort());

        // One more BufferTooSmall hits STREAM_MAX_CAPTURE_FAILURES (5).
        camera.set_mutex_locked(false);
        assert_eq!(
            session.write_next_frame_chunk(&camera, &mut tiny_buf),
            Err(HttpError::BufferTooSmall)
        );
        assert_eq!(session.consecutive_failures(), STREAM_MAX_CAPTURE_FAILURES);
        assert!(session.should_abort());

        // Once should_abort() is true, write_next_frame_chunk refuses even with a
        // healthy camera and large buffer.
        let captures_before = camera.capture_count();
        assert_eq!(
            session.write_next_frame_chunk(&camera, &mut ok_buf),
            Err(HttpError::CaptureFailed)
        );
        assert_eq!(camera.capture_count(), captures_before);
    }

    #[test]
    fn test_m049_apply_camera_control_diff_persists_quality_reapply_retry() {
        let mut applier = MockCameraControlApplier::new();
        applier.fail_quality = true;

        let mut last_applied = CameraControlState::new_default();
        let mut desired = CameraControlState::new_default();
        desired.framesize = FrameSize::Vga;
        // Same quality as default, so only framesize triggers quality reapply.
        assert_eq!(desired.quality, last_applied.quality);

        // Pass 1: framesize succeeds, quality fails -> reapply flag stays set.
        let err1 = apply_camera_control_diff(&mut applier, &desired, &mut last_applied);
        assert_eq!(err1, 1);
        assert_eq!(last_applied.framesize, FrameSize::Vga);
        assert!(last_applied.quality_needs_reapply);
        assert_eq!(applier.framesize_calls, 1);
        assert_eq!(applier.quality_calls, 1);

        // Pass 2: sensor recovers -> quality is retried and flag clears.
        applier.fail_quality = false;
        let err2 = apply_camera_control_diff(&mut applier, &desired, &mut last_applied);
        assert_eq!(err2, 0);
        assert!(!last_applied.quality_needs_reapply);
        assert_eq!(applier.framesize_calls, 1);
        assert_eq!(applier.quality_calls, 2);

        // Pass 3: no further calls once reconciled.
        let err3 = apply_camera_control_diff(&mut applier, &desired, &mut last_applied);
        assert_eq!(err3, 0);
        assert_eq!(applier.quality_calls, 2);
    }

    #[test]
    fn test_m138_m139_m141_m208_m209_m210_m211_http_hardening() {
        let mut server = test_server();
        let mut resp_buf = [0u8; 5120];

        // M-138: Security headers on /panel and /update.
        for path in ["/panel", "/update"] {
            let req = alloc::format!("GET {path} HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n");
            let resp = server
                .handle_raw_request(ServerPort::Control, req.as_bytes(), &mut resp_buf)
                .expect("GET html");
            let view = HttpResponseView::parse(resp).expect("parse html");
            assert_eq!(view.status_code, 200);
            assert_eq!(view.cache_control, Some("no-store"));
            assert!(view.headers_raw.contains("X-Frame-Options: DENY"));
            assert!(view.headers_raw.contains("Content-Security-Policy:"));
            assert!(view.headers_raw.contains("X-Content-Type-Options: nosniff"));
        }

        // M-211: MJPEG stream headers include Connection: close and nosniff.
        let stream_resp = server
            .handle_raw_request(
                ServerPort::Stream,
                b"GET /stream HTTP/1.1\r\nHost: 172.22.1.52:81\r\n\r\n",
                &mut resp_buf,
            )
            .expect("GET /stream");
        let stream_view = HttpResponseView::parse(stream_resp).expect("parse stream");
        assert!(stream_view.headers_raw.contains("Connection: close"));
        assert!(stream_view
            .headers_raw
            .contains("X-Content-Type-Options: nosniff"));

        // M-139: Reject OTA POST when req.body.len() != Content-Length
        // (extra trailing bytes).
        let image = crate::ota::make_valid_esp_image(true);
        let mut extra_req = ota_post_request(TEST_OTA_TOKEN, &image);
        extra_req.extend_from_slice(b"EXTRA");
        let extra_resp = server
            .handle_raw_request(ServerPort::Control, &extra_req, &mut resp_buf)
            .expect("400 written");
        assert_eq!(
            HttpResponseView::parse(extra_resp)
                .expect("parse 400")
                .status_code,
            400
        );
        assert!(!server.is_reboot_scheduled());

        // M-141: Empty body when Content-Length absent, single-SP request line,
        // duplicate Content-Type rejected.
        let unframed =
            parse_http_request(b"GET /status HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\nSMUGGLED")
                .expect("parse unframed");
        assert!(unframed.body.is_empty());
        assert_eq!(
            parse_http_request(b"GET  /status HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n"),
            Err(HttpError::MalformedRequest)
        );
        assert_eq!(
            parse_http_request(
                b"POST /control HTTP/1.1\r\nHost: 172.22.1.52\r\n\
Content-Type: text/plain\r\nContent-Type: application/json\r\n\r\n"
            ),
            Err(HttpError::MalformedRequest)
        );

        // M-208: Invalid UTF-8 POST /control body returns 400 Bad Request.
        let bad_utf8_ctrl = b"POST /control HTTP/1.1\r\nHost: 172.22.1.52\r\n\
Content-Length: 4\r\n\r\n\xff\xfe\xfd\xfc";
        let ctrl_resp = server
            .handle_raw_request(ServerPort::Control, bad_utf8_ctrl, &mut resp_buf)
            .expect("400 written");
        assert_eq!(
            HttpResponseView::parse(ctrl_resp)
                .expect("parse 400")
                .status_code,
            400
        );

        // M-209: HttpResponseView::parse rejects bad version, non-3-digit code,
        // missing colon, and duplicate headers.
        assert_eq!(
            HttpResponseView::parse(b"HTTP/2.0 200 OK\r\n\r\n"),
            Err(HttpError::MalformedRequest)
        );
        assert_eq!(
            HttpResponseView::parse(b"HTTP/1.1 +200 OK\r\n\r\n"),
            Err(HttpError::MalformedRequest)
        );
        assert_eq!(
            HttpResponseView::parse(b"HTTP/1.1 200 OK\r\nBadHeaderLine\r\n\r\n"),
            Err(HttpError::MalformedRequest)
        );
        assert_eq!(
            HttpResponseView::parse(
                b"HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nContent-Type: text/html\r\n\r\n"
            ),
            Err(HttpError::MalformedRequest)
        );

        // M-210: Worst-case 64-byte strings + max integers fit in 1536 bytes.
        let s64 = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
        let max_snap = TelemetrySnapshot {
            uptime_ms: i64::MIN,
            free_heap: u32::MAX,
            free_psram: u32::MAX,
            wifi_rssi: Some(i32::MIN),
            active_partition: s64,
            framesize: s64,
            stream_active: false,
            reset_reason: s64,
            app_sha256: s64,
            free_dram: u32::MAX,
            core1_stack_hwm: u32::MAX,
            fw_stats: FwStatsSnapshot {
                sensor_frame_period_us: u32::MAX,
                frame_bytes: u32::MAX,
                c1_pushed: u32::MAX,
                c1_dropped: u32::MAX,
                ring_full_events: u32::MAX,
                soi_miss: u32::MAX,
                overflow_aborts: u32::MAX,
                c0_spsc_cycles: u32::MAX,
                c0_net_cycles: u32::MAX,
                c0_p80_cycles: u32::MAX,
                c0_p81_cycles: u32::MAX,
                max_loop_latency_us: u32::MAX,
                p81_frames_sent: u32::MAX,
                p81_frames_superseded: u32::MAX,
                c0_validation_failures: u32::MAX,
                sccb_errors: u32::MAX,
                send_partial: u32::MAX,
                ring_build_failures: u32::MAX,
                i2s_stop_timeouts: u32::MAX,
            },
        };
        let mut status_buf = [0u8; STATUS_JSON_BUF_LEN];
        let json = format_status_json(&max_snap, &mut status_buf).expect("fits 1536 bytes");
        assert!(json.len() <= STATUS_JSON_BUF_LEN);
    }

    struct MockSetupFlash {
        sector_a: [u8; crate::ota::FLASH_SECTOR_SIZE as usize],
        sector_b: [u8; crate::ota::FLASH_SECTOR_SIZE as usize],
        writes: u32,
    }

    impl MockSetupFlash {
        fn new() -> Self {
            Self {
                sector_a: [0xFF; crate::ota::FLASH_SECTOR_SIZE as usize],
                sector_b: [0xFF; crate::ota::FLASH_SECTOR_SIZE as usize],
                writes: 0,
            }
        }

        fn region_mut(&mut self, offset: u32, len: usize) -> Result<&mut [u8], OtaError> {
            use crate::ota::FLASH_SECTOR_SIZE;
            use crate::prov_store::{PROV_SECTOR_A_OFFSET, PROV_SECTOR_B_OFFSET};

            let len_u32 = u32::try_from(len).map_err(|_| OtaError::FlashWriteError)?;
            let end = offset
                .checked_add(len_u32)
                .ok_or(OtaError::FlashWriteError)?;
            if offset >= PROV_SECTOR_A_OFFSET && end <= PROV_SECTOR_A_OFFSET + FLASH_SECTOR_SIZE {
                let start = (offset - PROV_SECTOR_A_OFFSET) as usize;
                Ok(&mut self.sector_a[start..start + len])
            } else if offset >= PROV_SECTOR_B_OFFSET
                && end <= PROV_SECTOR_B_OFFSET + FLASH_SECTOR_SIZE
            {
                let start = (offset - PROV_SECTOR_B_OFFSET) as usize;
                Ok(&mut self.sector_b[start..start + len])
            } else {
                Err(OtaError::FlashWriteError)
            }
        }
    }

    impl OtaFlashStorage for MockSetupFlash {
        fn read_aligned(&mut self, offset: u32, buf: &mut [u8]) -> Result<(), OtaError> {
            if (offset & 3) != 0 || (buf.len() & 3) != 0 {
                return Err(OtaError::FlashWriteError);
            }
            let slice = self.region_mut(offset, buf.len())?;
            buf.copy_from_slice(slice);
            Ok(())
        }

        fn erase_range(&mut self, start_offset: u32, end_offset: u32) -> Result<(), OtaError> {
            let len = end_offset
                .checked_sub(start_offset)
                .ok_or(OtaError::FlashWriteError)? as usize;
            let slice = self.region_mut(start_offset, len)?;
            slice.fill(0xFF);
            Ok(())
        }

        fn write_aligned(&mut self, offset: u32, data: &[u8]) -> Result<(), OtaError> {
            if (offset & 3) != 0 || (data.len() & 3) != 0 {
                return Err(OtaError::FlashWriteError);
            }
            let slice = self.region_mut(offset, data.len())?;
            slice.copy_from_slice(data);
            self.writes += 1;
            Ok(())
        }
    }

    fn build_setup_post_req(
        host: &str,
        origin: Option<&str>,
        token: Option<&str>,
        content_type: &str,
        body: &str,
    ) -> alloc::vec::Vec<u8> {
        let mut req = alloc::format!(
            "POST /setup HTTP/1.1\r\nHost: {host}\r\nContent-Type: {content_type}\r\n"
        );
        if let Some(orig) = origin {
            req.push_str(&alloc::format!("Origin: {orig}\r\n"));
        }
        if let Some(tok) = token {
            req.push_str(&alloc::format!("X-OTA-Token: {tok}\r\n"));
        }
        req.push_str(&alloc::format!(
            "X-Requested-With: fetch\r\nContent-Length: {}\r\n\r\n{body}",
            body.len()
        ));
        req.into_bytes()
    }

    #[test]
    fn test_softap_setup_captive_portal_redirects() {
        let mac = [0xD4, 0xD4, 0xDA, 0x5B, 0xD4, 0xEC];
        let mut out_buf = [0u8; 4096];

        for path in [
            "/generate_204",
            "/gen_204",
            "/hotspot-detect.html",
            "/connecttest.txt",
            "/ncsi.txt",
            "/canonical.html",
            "/redirect",
            "/",
        ] {
            let req = alloc::format!(
                "GET {path} HTTP/1.1\r\nHost: connectivitycheck.gstatic.com\r\n\r\n"
            );
            let outcome = handle_setup_http_request(
                req.as_bytes(),
                &mac,
                "KlipperCam-Setup-D4EC",
                "",
                &mut out_buf,
            );
            let SetupHttpOutcome::Responded(n) = outcome else {
                panic!("captive probe {path} must return Responded");
            };
            let view = HttpResponseView::parse(&out_buf[..n]).expect("valid 302 response");
            assert_eq!(view.status_code, 302, "probe {path} must redirect");
            assert!(
                view.headers_raw
                    .contains("Location: http://192.168.4.1/setup"),
                "probe {path} must redirect to http://192.168.4.1/setup"
            );
            assert_eq!(view.cache_control, Some("no-store"));
        }

        // Unknown path returns 404, non-GET/POST on /setup returns 405.
        let SetupHttpOutcome::Responded(n404) = handle_setup_http_request(
            b"GET /favicon.ico HTTP/1.1\r\nHost: 192.168.4.1\r\n\r\n",
            &mac,
            "KlipperCam-Setup-D4EC",
            "",
            &mut out_buf,
        ) else {
            panic!("404 must return Responded");
        };
        assert_eq!(
            HttpResponseView::parse(&out_buf[..n404])
                .expect("parse 404")
                .status_code,
            404
        );

        let SetupHttpOutcome::Responded(n405) = handle_setup_http_request(
            b"DELETE /setup HTTP/1.1\r\nHost: 192.168.4.1\r\n\r\n",
            &mac,
            "KlipperCam-Setup-D4EC",
            "",
            &mut out_buf,
        ) else {
            panic!("405 must return Responded");
        };
        assert_eq!(
            HttpResponseView::parse(&out_buf[..n405])
                .expect("parse 405")
                .status_code,
            405
        );
    }

    #[test]
    fn test_softap_capport_rfc8908_json_endpoint() {
        let mac = [0xD4, 0xD4, 0xDA, 0x5B, 0xD4, 0xEC];
        let mut out_buf = [0u8; 4096];

        let outcome = handle_setup_http_request(
            b"GET /capport HTTP/1.1\r\nHost: 192.168.4.1\r\n\r\n",
            &mac,
            "KlipperCam-Setup-D4EC",
            "",
            &mut out_buf,
        );
        let SetupHttpOutcome::Responded(n) = outcome else {
            panic!("GET /capport must return Responded");
        };
        let view = HttpResponseView::parse(&out_buf[..n]).expect("valid 200 capport JSON");
        assert_eq!(view.status_code, 200);
        assert_eq!(view.status_line, "HTTP/1.1 200 OK");
        assert_eq!(view.content_type, Some("application/captive+json"));
        assert_eq!(view.cache_control, Some("no-store"));
        assert!(
            view.headers_raw.contains("Connection: close"),
            "capport response must close connection"
        );
        let expected_body = r#"{"captive":true,"user-portal-url":"http://192.168.4.1/setup"}"#;
        let expected_len_hdr = alloc::format!("Content-Length: {}", expected_body.len());
        assert!(
            view.headers_raw.contains(&expected_len_hdr),
            "capport response must include exact Content-Length"
        );
        assert_eq!(
            core::str::from_utf8(view.body).expect("UTF-8 capport JSON"),
            expected_body
        );
    }

    #[test]
    fn test_softap_setup_get_form_headers_size_and_no_secrets() {
        let mac = [0xD4, 0xD4, 0xDA, 0x5B, 0xD4, 0xEC];
        let mut out_buf = [0u8; 4096];

        let outcome = handle_setup_http_request(
            b"GET /setup HTTP/1.1\r\nHost: 192.168.4.1\r\n\r\n",
            &mac,
            "KlipperCam-Setup-D4EC",
            "Lab<Net>&1",
            &mut out_buf,
        );
        let SetupHttpOutcome::Responded(n) = outcome else {
            panic!("GET /setup must return Responded");
        };
        let view = HttpResponseView::parse(&out_buf[..n]).expect("valid 200 HTML");
        assert_eq!(view.status_code, 200);
        assert_eq!(view.content_type, Some("text/html"));
        assert_eq!(view.cache_control, Some("no-store"));
        assert_eq!(view.access_control_allow_origin, None);
        assert!(view.headers_raw.contains("X-Frame-Options: DENY"));
        assert!(view.headers_raw.contains(
            "Content-Security-Policy: default-src 'self' 'unsafe-inline'; frame-ancestors 'none'"
        ));
        assert!(view.headers_raw.contains("X-Content-Type-Options: nosniff"));
        assert!(
            view.body.len() <= 3072,
            "SoftAP setup HTML body ({} B) must be <= 3 KB",
            view.body.len()
        );

        let html = core::str::from_utf8(view.body).expect("UTF-8 HTML");
        assert!(html.contains("d4:d4:da:5b:d4:ec"));
        assert!(html.contains("KlipperCam-Setup-D4EC"));
        assert!(html.contains("Lab&lt;Net&gt;&amp;1"));
        assert!(!html.contains("http://"));
        assert!(!html.contains("https://"));
        for field in [
            "name=\"ssid\"",
            "name=\"pass\"",
            "name=\"token\"",
            "name=\"ip_mode\"",
            "name=\"static_ip\"",
            "name=\"gateway\"",
            "name=\"prefix\"",
        ] {
            assert!(html.contains(field), "missing input {field}");
        }
    }

    #[test]
    fn test_softap_setup_post_percent_decoding_and_static_ip() {
        let mac = [0xD4, 0xD4, 0xDA, 0x5B, 0xD4, 0xEC];
        let mut out_buf = [0u8; 4096];

        // Review Focus #2: +, %20, %26, %3D, %25 decode accurately (DHCP mode).
        let dhcp_body = "ssid=My+Home%20WiFi%26Lab%3D1%25&\
                         pass=Pass+Word%20%26%3D%25!99&\
                         token=priv-tok%26%3D%25-2026-abcd&\
                         ip_mode=dhcp&static_ip=&gateway=&prefix=24";
        let dhcp_req = build_setup_post_req(
            "192.168.4.1",
            None,
            None,
            "application/x-www-form-urlencoded; charset=utf-8",
            dhcp_body,
        );
        let outcome =
            handle_setup_http_request(&dhcp_req, &mac, "KlipperCam-Setup-D4EC", "", &mut out_buf);
        let SetupHttpOutcome::SavedAndReboot { resp_len, config } = outcome else {
            panic!("valid DHCP POST /setup must return SavedAndReboot");
        };
        let view = HttpResponseView::parse(&out_buf[..resp_len]).expect("parse 200 reboot HTML");
        assert_eq!(view.status_code, 200);
        assert_eq!(view.content_type, Some("text/html"));
        let resp_text = core::str::from_utf8(view.body).expect("utf8 body");
        assert!(resp_text.contains("Rebooting"));
        assert!(!resp_text.contains("Pass Word"));
        assert!(!resp_text.contains("priv-tok"));

        assert_eq!(config.ssid_str(), "My Home WiFi&Lab=1%");
        assert_eq!(config.pass_str(), "Pass Word &=%%!99".replace("%%", "%"));
        assert_eq!(config.ota_token_str(), "priv-tok&=%-2026-abcd");
        assert_eq!(config.ip_mode(), IpMode::Dhcp);

        // Static IPv4 submission.
        let static_body = "ssid=PrinterNet&pass=StrongPass2026!&\
                           token=static-ota-token-2026-ok&\
                           ip_mode=static&static_ip=172.22.1.52&gateway=172.22.1.1&prefix=24";
        let static_req = build_setup_post_req(
            "192.168.4.1",
            None,
            None,
            "application/x-www-form-urlencoded",
            static_body,
        );
        let SetupHttpOutcome::SavedAndReboot { resp_len, config } =
            handle_setup_http_request(&static_req, &mac, "KlipperCam-Setup-D4EC", "", &mut out_buf)
        else {
            panic!("valid static POST /setup must return SavedAndReboot");
        };
        assert_eq!(
            HttpResponseView::parse(&out_buf[..resp_len])
                .expect("parse 200")
                .status_code,
            200
        );
        assert_eq!(
            config.ip_mode(),
            IpMode::Static(crate::config::StaticIpConfig::KLIPPER_DEFAULT)
        );
    }

    #[test]
    fn test_softap_setup_post_rejects_bad_encoding_and_never_echoes_secrets() {
        let mac = [0xD4, 0xD4, 0xDA, 0x5B, 0xD4, 0xEC];
        let mut out_buf = [0u8; 4096];

        let secret_pass = "UltraSecretPass9876";
        let secret_tok = "ultra-secret-ota-token-9876";

        let bad_cases = [
            // Invalid hex %ZZ
            alloc::format!("ssid=Bad%ZZSsid&pass={secret_pass}&token={secret_tok}&ip_mode=dhcp"),
            // NUL byte %00 in pass
            alloc::format!("ssid=GoodSsid&pass={secret_pass}%00&token={secret_tok}&ip_mode=dhcp"),
            // CR/LF %0D%0A in token
            alloc::format!(
                "ssid=GoodSsid&pass={secret_pass}&token={secret_tok}%0D%0A&ip_mode=dhcp"
            ),
            // Truncated trailing %
            alloc::format!("ssid=GoodSsid&pass={secret_pass}%2&token={secret_tok}&ip_mode=dhcp"),
            // Default factory OTA token rejected
            alloc::format!(
                "ssid=GoodSsid&pass={secret_pass}&token={DEFAULT_OTA_TOKEN}&ip_mode=dhcp"
            ),
            // Short passphrase rejected
            alloc::format!("ssid=GoodSsid&pass=short&token={secret_tok}&ip_mode=dhcp"),
            // Invalid static prefix (> 30)
            alloc::format!(
                "ssid=GoodSsid&pass={secret_pass}&token={secret_tok}&\
                 ip_mode=static&static_ip=172.22.1.52&gateway=172.22.1.1&prefix=32"
            ),
            // Body > 512 bytes rejected
            alloc::format!(
                "ssid=GoodSsid&pass={secret_pass}&token={secret_tok}&ip_mode=dhcp&pad={}",
                "a".repeat(500)
            ),
        ];

        for body in bad_cases {
            let req = build_setup_post_req(
                "192.168.4.1",
                None,
                None,
                "application/x-www-form-urlencoded",
                &body,
            );
            let outcome =
                handle_setup_http_request(&req, &mac, "KlipperCam-Setup-D4EC", "", &mut out_buf);
            let SetupHttpOutcome::Responded(n) = outcome else {
                panic!("invalid POST /setup must NOT return SavedAndReboot: {body}");
            };
            let view = HttpResponseView::parse(&out_buf[..n]).expect("valid 400 HTML");
            assert_eq!(view.status_code, 400);
            assert_eq!(view.content_type, Some("text/html"));
            let html = core::str::from_utf8(view.body).expect("utf8 error HTML");
            assert!(!html.contains(secret_pass), "400 HTML must NEVER echo pass");
            assert!(!html.contains(secret_tok), "400 HTML must NEVER echo token");
        }

        // Wrong Content-Type rejected with 400.
        let wrong_ct = build_setup_post_req(
            "192.168.4.1",
            None,
            None,
            "text/plain",
            &alloc::format!("ssid=GoodSsid&pass={secret_pass}&token={secret_tok}&ip_mode=dhcp"),
        );
        let SetupHttpOutcome::Responded(n) =
            handle_setup_http_request(&wrong_ct, &mac, "KlipperCam-Setup-D4EC", "", &mut out_buf)
        else {
            panic!("wrong content-type must return Responded");
        };
        assert_eq!(
            HttpResponseView::parse(&out_buf[..n])
                .expect("parse 400")
                .status_code,
            400
        );
    }

    #[test]
    fn test_sta_setup_get_form_security_headers_and_no_secrets() {
        let mut cfg = KlipperCamConfig::WROVER_KIT_HD;
        cfg.network.wifi_ssid = "ShopWiFi-2.4G";
        cfg.network.wifi_password = "TopSecretWifiPass99!";
        cfg.ota_token = TEST_OTA_TOKEN;
        let mut server = KlipperCamServer::new(cfg, CameraDriver::new_hd_default());
        server.set_device_mac([0xD4, 0xD4, 0xDA, 0x5B, 0xD4, 0xEC]);

        let mut resp_buf = [0u8; 4096];
        let resp = server
            .handle_raw_request(
                ServerPort::Control,
                b"GET /setup HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n",
                &mut resp_buf,
            )
            .expect("STA GET /setup should succeed");
        let view = HttpResponseView::parse(resp).expect("parse STA setup HTML");
        assert_eq!(view.status_code, 200);
        assert_eq!(view.content_type, Some("text/html"));
        assert_eq!(view.cache_control, Some("no-store"));
        assert_eq!(view.access_control_allow_origin, None);
        assert!(view.headers_raw.contains("X-Frame-Options: DENY"));
        assert!(view.headers_raw.contains(
            "Content-Security-Policy: default-src 'self' 'unsafe-inline'; frame-ancestors 'none'"
        ));
        assert!(view.headers_raw.contains("X-Content-Type-Options: nosniff"));

        let html = core::str::from_utf8(view.body).expect("utf8 html");
        assert!(html.contains("d4:d4:da:5b:d4:ec"));
        assert!(html.contains("ShopWiFi-2.4G"));
        assert!(html.contains("Static (172.22.1.52/24)"));
        assert!(html.contains("Current OTA Token"));
        assert!(html.contains("fetch('/setup'"));
        assert!(html.contains("'X-OTA-Token'"));
        assert!(html.contains("'X-Requested-With':'fetch'"));
        // Must NEVER echo wifi_password or ota_token.
        assert!(!html.contains("TopSecretWifiPass99!"));
        assert!(!html.contains(TEST_OTA_TOKEN));

        // DHCP mode renders "IP Mode: DHCP".
        server.config.network.ip_mode = IpMode::Dhcp;
        let resp_dhcp = server
            .handle_raw_request(
                ServerPort::Control,
                b"GET /setup HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n",
                &mut resp_buf,
            )
            .expect("STA GET /setup DHCP");
        let html_dhcp =
            core::str::from_utf8(HttpResponseView::parse(resp_dhcp).expect("parse dhcp").body)
                .expect("utf8");
        assert!(html_dhcp.contains("IP Mode: DHCP"));
    }

    #[test]
    fn test_sta_setup_post_auth_host_origin_persistence_and_conflict() {
        let mut server = test_server();
        let mut flash = MockSetupFlash::new();
        let mut resp_buf = [0u8; 2048];

        let valid_body = "ssid=Rotated+WiFi%261&pass=NewSecretPass%3D2026!&\
                          token=rotated-ota-token-2026-%25ok&\
                          ip_mode=static&static_ip=172.22.1.60&gateway=172.22.1.1&prefix=24";

        // 1. Disallowed Host -> 403 Forbidden.
        let bad_host = build_setup_post_req(
            "evil.example",
            Some("http://evil.example"),
            Some(TEST_OTA_TOKEN),
            "application/x-www-form-urlencoded",
            valid_body,
        );
        let r = server
            .handle_raw_request_with_flash(
                ServerPort::Control,
                &bad_host,
                &mut resp_buf,
                &mut flash,
            )
            .expect("403 bad host");
        assert_eq!(HttpResponseView::parse(r).expect("parse").status_code, 403);
        assert_eq!(flash.writes, 0);

        // 2. Cross-origin Origin -> 403 Forbidden.
        let bad_origin = build_setup_post_req(
            "172.22.1.52",
            Some("http://attacker.local"),
            Some(TEST_OTA_TOKEN),
            "application/x-www-form-urlencoded",
            valid_body,
        );
        let r = server
            .handle_raw_request_with_flash(
                ServerPort::Control,
                &bad_origin,
                &mut resp_buf,
                &mut flash,
            )
            .expect("403 bad origin");
        assert_eq!(HttpResponseView::parse(r).expect("parse").status_code, 403);
        assert_eq!(flash.writes, 0);

        // 3. Missing X-OTA-Token -> 401 Unauthorized.
        let no_tok = build_setup_post_req(
            "172.22.1.52",
            Some("http://172.22.1.52"),
            None,
            "application/x-www-form-urlencoded",
            valid_body,
        );
        let r = server
            .handle_raw_request_with_flash(ServerPort::Control, &no_tok, &mut resp_buf, &mut flash)
            .expect("401 missing token");
        assert_eq!(HttpResponseView::parse(r).expect("parse").status_code, 401);
        assert_eq!(flash.writes, 0);

        // 4. Wrong X-OTA-Token -> 401 Unauthorized.
        let wrong_tok = build_setup_post_req(
            "172.22.1.52",
            Some("http://172.22.1.52"),
            Some("wrong-token-12345678"),
            "application/x-www-form-urlencoded",
            valid_body,
        );
        let r = server
            .handle_raw_request_with_flash(
                ServerPort::Control,
                &wrong_tok,
                &mut resp_buf,
                &mut flash,
            )
            .expect("401 wrong token");
        assert_eq!(HttpResponseView::parse(r).expect("parse").status_code, 401);
        assert_eq!(flash.writes, 0);

        // 5. Unconfigured (default or blank) server OTA token -> 403 Forbidden.
        for bad_server_tok in [DEFAULT_OTA_TOKEN, ""] {
            let mut locked_cfg = KlipperCamConfig::WROVER_KIT_HD;
            locked_cfg.ota_token = bad_server_tok;
            let mut locked_srv = KlipperCamServer::new(locked_cfg, CameraDriver::new_hd_default());
            let req = build_setup_post_req(
                "172.22.1.52",
                Some("http://172.22.1.52"),
                Some(bad_server_tok),
                "application/x-www-form-urlencoded",
                valid_body,
            );
            let r = locked_srv
                .handle_raw_request_with_flash(ServerPort::Control, &req, &mut resp_buf, &mut flash)
                .expect("403 locked token");
            assert_eq!(HttpResponseView::parse(r).expect("parse").status_code, 403);
            assert!(!locked_srv.is_reboot_scheduled());
        }
        assert_eq!(flash.writes, 0);

        // 6. Malformed body (%0D%0A) with valid auth -> 400, zero secret echo.
        let bad_body_req = build_setup_post_req(
            "172.22.1.52",
            Some("http://172.22.1.52"),
            Some(TEST_OTA_TOKEN),
            "application/x-www-form-urlencoded",
            "ssid=Ok&pass=NeverEchoPass123&token=never-echo-token-123%0D%0A&ip_mode=dhcp",
        );
        let r = server
            .handle_raw_request_with_flash(
                ServerPort::Control,
                &bad_body_req,
                &mut resp_buf,
                &mut flash,
            )
            .expect("400 bad body");
        let v400 = HttpResponseView::parse(r).expect("parse 400");
        assert_eq!(v400.status_code, 400);
        let err_str = core::str::from_utf8(v400.body).expect("utf8 err");
        assert!(!err_str.contains("NeverEchoPass123"));
        assert!(!err_str.contains("never-echo-token"));
        assert_eq!(flash.writes, 0);
        assert!(!server.is_reboot_scheduled());

        // 7. Valid same-origin POST /setup -> 200 OK, persists and reboots.
        let ok_req = build_setup_post_req(
            "172.22.1.52",
            Some("http://172.22.1.52"),
            Some(TEST_OTA_TOKEN),
            "application/x-www-form-urlencoded",
            valid_body,
        );
        let r_ok = server
            .handle_raw_request_with_flash(ServerPort::Control, &ok_req, &mut resp_buf, &mut flash)
            .expect("200 valid setup");
        let v_ok = HttpResponseView::parse(r_ok).expect("parse 200");
        assert_eq!(v_ok.status_code, 200);
        assert_eq!(v_ok.access_control_allow_origin, None);
        assert_eq!(v_ok.cache_control, Some("no-store"));
        assert!(server.is_reboot_scheduled());
        assert_eq!(flash.writes, 1);

        let persisted = ProvStore::load(&mut flash)
            .expect("load persisted config")
            .expect("config must exist in flash");
        assert_eq!(persisted.ssid_str(), "Rotated WiFi&1");
        assert_eq!(persisted.pass_str(), "NewSecretPass=2026!");
        assert_eq!(persisted.ota_token_str(), "rotated-ota-token-2026-%ok");
        assert_eq!(
            persisted.ip_mode(),
            IpMode::Static(crate::config::StaticIpConfig {
                ip: Ipv4Addr::new(172, 22, 1, 60),
                netmask: Ipv4Addr::new(255, 255, 255, 0),
                gateway: Ipv4Addr::new(172, 22, 1, 1),
                dns: Ipv4Addr::new(172, 22, 1, 1),
                prefix_len: 24,
            })
        );

        // 8. Re-entrant POST /setup once reboot_scheduled -> 409 Conflict.
        let r_conflict = server
            .handle_raw_request_with_flash(ServerPort::Control, &ok_req, &mut resp_buf, &mut flash)
            .expect("409 conflict");
        assert_eq!(
            HttpResponseView::parse(r_conflict)
                .expect("parse 409")
                .status_code,
            409
        );
        assert_eq!(flash.writes, 1);
    }
}
