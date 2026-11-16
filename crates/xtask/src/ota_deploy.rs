//! Wireless Dual-Slot OTA Deployment & Staging Tool for Klipper Camera (ESP32-WROVER-DEV).
//!
//! Replicates `scripts/ota_deploy.py` in pure Rust with zero external network crates.

use crate::sha256::Sha256;
use std::env;
use std::fs;
use std::io::{Read, Write};
use std::net::{TcpStream, ToSocketAddrs};
use std::path::{Path, PathBuf};
use std::process::Command;
use std::thread::sleep;
use std::time::{Duration, Instant};

pub const ESP_IMAGE_HEADER_MAGIC: u8 = 0xE9;
pub const ESP_IMAGE_MIN_SEGMENTS: u8 = 1;
pub const ESP_IMAGE_MAX_SEGMENTS: u8 = 16;
pub const ESP_IMAGE_HEADER_LEN: usize = 24;
pub const OTA_PARTITION_MAX_SIZE: usize = 0x001B_0000; // 1,769,472 bytes
pub const CHUNK_SIZE: usize = 16384; // 16 KB chunks for high-throughput upload

/// Validated ESP32 image header metadata.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EspImageHeader {
    pub magic: u8,
    pub segment_count: u8,
    pub flash_mode: u8,
    pub flash_size_freq: u8,
    pub entry_addr: u32,
    pub hash_appended: bool,
    pub size: usize,
    pub sha256_hex: String,
}

/// Parsed fields from GET `/snapshot`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SnapshotInfo {
    pub len: usize,
    pub has_soi: bool,
    pub has_eoi: bool,
    pub dims: Option<(u16, u16)>,
}

/// Parsed fields from GET `/status`.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct OtaStatusInfo {
    pub active_partition: String,
    pub uptime_ms: u64,
    pub framesize: String,
    pub wifi_rssi: i32,
    pub app_sha256: Option<String>,
}

/// Options parsed from command-line arguments.
#[derive(Debug, Clone)]
pub struct OtaDeployOptions {
    pub host: String,
    pub port: u16,
    pub elf_path: PathBuf,
    pub bin_path: PathBuf,
    pub token: Option<String>,
    pub stage_only: bool,
    pub expected_dims: String,
}

impl Default for OtaDeployOptions {
    fn default() -> Self {
        let root = repo_root();
        let default_elf = root
            .join("target")
            .join("xtensa-esp32-none-elf")
            .join("release")
            .join("klipper-cam-fw");
        let default_bin = root
            .join("target")
            .join("xtensa-esp32-none-elf")
            .join("release")
            .join("klipper-cam-fw.bin");

        Self {
            host: env::var("CAM_HOST").unwrap_or_default(),
            port: 80,
            elf_path: default_elf,
            bin_path: default_bin,
            token: resolve_default_token(),
            stage_only: false,
            expected_dims: "1280x720".to_string(),
        }
    }
}

/// Discovers repository root relative to current executable or directory.
#[must_use]
pub fn repo_root() -> PathBuf {
    let current_dir = env::current_dir().unwrap_or_else(|_| PathBuf::from("."));
    if current_dir.join("Cargo.toml").is_file() && current_dir.join("crates").is_dir() {
        return current_dir;
    }
    // Search upwards
    let mut cursor = current_dir.as_path();
    while let Some(parent) = cursor.parent() {
        if parent.join("Cargo.toml").is_file() && parent.join("crates").is_dir() {
            return parent.to_path_buf();
        }
        cursor = parent;
    }
    PathBuf::from("/home/console-admin/Github/esp32-camera")
}

/// Resolves the default OTA authentication token from environment or local configuration.
#[must_use]
pub fn resolve_default_token() -> Option<String> {
    for var in &[
        "CAM_OTA_TOKEN",
        "ESP32_CAM_OTA_TOKEN",
        "OTA_TOKEN",
        "WIFI_OTA_TOKEN",
    ] {
        if let Ok(val) = env::var(var) {
            let trimmed = val.trim();
            if !trimmed.is_empty() {
                return Some(trimmed.to_string());
            }
        }
    }

    let env_local = repo_root().join(".env.local");
    if let Ok(content) = fs::read_to_string(env_local) {
        for line in content.lines() {
            let trimmed = line.trim();
            for prefix in &[
                "CAM_OTA_TOKEN=",
                "ESP32_CAM_OTA_TOKEN=",
                "OTA_TOKEN=",
                "WIFI_OTA_TOKEN=",
            ] {
                if let Some(val) = trimmed.strip_prefix(prefix) {
                    let cleaned = val.trim().trim_matches('"').trim_matches('\'');
                    if !cleaned.is_empty() {
                        return Some(cleaned.to_string());
                    }
                }
            }
        }
    }

    None
}

/// Maps current active partition to expected target partition after OTA slot flip.
///
/// # Errors
/// Returns an error if the current slot label is not `ota_0` or `ota_1`.
pub fn expected_next_slot(current_slot: &str) -> Result<&'static str, String> {
    match current_slot {
        "ota_0" => Ok("ota_1"),
        "ota_1" => Ok("ota_0"),
        other => Err(format!("Unknown active_partition label: '{other}'")),
    }
}

/// Extracts (width, height) from the JPEG SOF0/SOF2 marker in a byte slice.
#[must_use]
pub fn parse_jpeg_dimensions(data: &[u8]) -> Option<(u16, u16)> {
    if data.len() < 4 || data[0] != 0xFF || data[1] != 0xD8 {
        return None;
    }
    let mut i = 2;
    let n = data.len();
    while i + 8 < n {
        if data[i] != 0xFF {
            i += 1;
            continue;
        }
        while i < n && data[i] == 0xFF {
            i += 1;
        }
        if i >= n {
            break;
        }
        let marker = data[i];
        i += 1;
        if marker == 0xD8 || marker == 0xD9 || (0xD0..=0xD7).contains(&marker) {
            continue;
        }
        if i + 2 > n {
            break;
        }
        let seg_len = u16::from_be_bytes([data[i], data[i + 1]]) as usize;
        if seg_len < 2 || i + seg_len > n {
            break;
        }
        if (marker == 0xC0 || marker == 0xC2) && i + 7 <= n {
            let height = u16::from_be_bytes([data[i + 3], data[i + 4]]);
            let width = u16::from_be_bytes([data[i + 5], data[i + 6]]);
            return Some((width, height));
        }
        i += seg_len;
    }
    None
}

/// Helper to extract string or value tokens from simple flat JSON strings.
#[must_use]
pub fn extract_json_field<'a>(json: &'a str, key: &str) -> Option<&'a str> {
    let pattern = format!("\"{key}\"");
    let key_pos = json.find(&pattern)?;
    let rest = &json[key_pos + pattern.len()..];
    let colon_pos = rest.find(':')?;
    let after_colon = rest[colon_pos + 1..].trim_start();
    if let Some(stripped) = after_colon.strip_prefix('"') {
        let end_quote = stripped.find('"')?;
        Some(&stripped[..end_quote])
    } else {
        let end_idx = after_colon
            .find(|c: char| c == ',' || c == '}' || c.is_whitespace())
            .unwrap_or(after_colon.len());
        Some(&after_colon[..end_idx])
    }
}

/// Parses the `/status` JSON endpoint response.
///
/// # Errors
/// Returns error if required fields cannot be parsed.
pub fn parse_status_json(raw: &str) -> Result<OtaStatusInfo, String> {
    let active_partition = extract_json_field(raw, "active_partition")
        .unwrap_or("")
        .to_string();
    let uptime_ms = extract_json_field(raw, "uptime_ms")
        .and_then(|v| v.parse::<u64>().ok())
        .unwrap_or(0);
    let framesize = extract_json_field(raw, "framesize")
        .unwrap_or("")
        .to_string();
    let wifi_rssi = extract_json_field(raw, "wifi_rssi")
        .and_then(|v| v.parse::<i32>().ok())
        .unwrap_or(0);
    let app_sha256 = extract_json_field(raw, "app_sha256").map(|s| s.to_ascii_lowercase());

    Ok(OtaStatusInfo {
        active_partition,
        uptime_ms,
        framesize,
        wifi_rssi,
        app_sha256,
    })
}

/// Validates ESP32 binary image header, segment count, size bound, and appended SHA-256.
///
/// # Errors
/// Returns error if magic, segment count, size limit, or SHA-256 check fails.
pub fn validate_esp_image(payload: &[u8]) -> Result<EspImageHeader, String> {
    let size = payload.len();
    if size < ESP_IMAGE_HEADER_LEN {
        return Err(format!(
            "Image too small ({size} bytes < {ESP_IMAGE_HEADER_LEN} bytes)"
        ));
    }
    if size > OTA_PARTITION_MAX_SIZE {
        return Err(format!(
            "Image size ({size} bytes) exceeds OTA slot limit (0x{OTA_PARTITION_MAX_SIZE:X} = {OTA_PARTITION_MAX_SIZE} bytes)"
        ));
    }

    let magic = payload[0];
    let segment_count = payload[1];
    let flash_mode = payload[2];
    let flash_size_freq = payload[3];
    let entry_addr = u32::from_le_bytes([payload[4], payload[5], payload[6], payload[7]]);

    if magic != ESP_IMAGE_HEADER_MAGIC {
        return Err(format!(
            "Invalid ESP32 image magic: 0x{magic:02X} (expected 0xE9)"
        ));
    }
    if !(ESP_IMAGE_MIN_SEGMENTS..=ESP_IMAGE_MAX_SEGMENTS).contains(&segment_count) {
        return Err(format!(
            "Invalid ESP32 segment count: {segment_count} (expected {ESP_IMAGE_MIN_SEGMENTS}..={ESP_IMAGE_MAX_SEGMENTS})"
        ));
    }

    let hash_appended = payload[23] == 1;
    if hash_appended {
        if size < ESP_IMAGE_HEADER_LEN + 32 {
            return Err(format!(
                "Image too small for appended SHA-256 ({size} bytes)"
            ));
        }
        let calc_digest = Sha256::digest(&payload[..size - 32]);
        let appended_digest = &payload[size - 32..];
        if calc_digest != appended_digest {
            return Err(format!(
                "Appended SHA-256 mismatch: computed {} != appended {}",
                Sha256::digest_hex(&payload[..size - 32]),
                payload[size - 32..]
                    .iter()
                    .map(|b| format!("{b:02x}"))
                    .collect::<String>()
            ));
        }
    }

    let sha256_hex = Sha256::digest_hex(payload);

    Ok(EspImageHeader {
        magic,
        segment_count,
        flash_mode,
        flash_size_freq,
        entry_addr,
        hash_appended,
        size,
        sha256_hex,
    })
}

/// Stages an ESP-IDF application image using `espflash save-image` and validates its header.
///
/// # Errors
/// Returns error if file conversion or validation fails.
pub fn stage_and_validate_image(
    elf_path: &Path,
    bin_path: &Path,
    host: &str,
    port: u16,
) -> Result<Vec<u8>, String> {
    if !elf_path.is_file() {
        return Err(format!("ELF binary not found at {}", elf_path.display()));
    }

    let espflash_cmd = find_espflash()?;
    let status = Command::new(&espflash_cmd)
        .current_dir(repo_root())
        .args([
            "save-image",
            "--chip",
            "esp32",
            "--flash-size",
            "4mb",
            "--flash-mode",
            "dio",
            "--flash-freq",
            "40mhz",
            "--skip-update-check",
            &elf_path.to_string_lossy(),
            &bin_path.to_string_lossy(),
        ])
        .status()
        .map_err(|e| format!("Failed to execute espflash: {e}"))?;

    if !status.success() {
        return Err(format!(
            "espflash save-image failed with exit status {:?}",
            status.code()
        ));
    }

    if !bin_path.is_file() {
        return Err(format!(
            "Generated .bin file not found at {}",
            bin_path.display()
        ));
    }

    let payload =
        fs::read(bin_path).map_err(|e| format!("Failed to read {}: {e}", bin_path.display()))?;

    let header = validate_esp_image(&payload)?;

    let target_url = if !host.is_empty() {
        format!("http://{host}:{port}/update")
    } else {
        "(staged locally; no host specified)".to_string()
    };

    println!("================================================================================");
    println!("[OTA-STAGE] ESP32 Firmware Image Staged & Validated");
    println!("--------------------------------------------------------------------------------");
    println!("  Local Source ELF Path  : {}", elf_path.display());
    println!("  Local Staged BIN Path  : {}", bin_path.display());
    println!("  Target Remote Endpoint : {target_url}");
    println!(
        "  File Size              : {} bytes ({:.2} KB / max {} bytes)",
        header.size,
        header.size as f64 / 1024.0,
        OTA_PARTITION_MAX_SIZE
    );
    println!(
        "  ESP32 Image Header     : magic=0x{:02X} segments={} mode=0x{:02X} size_freq=0x{:02X} entry=0x{:08X}",
        header.magic, header.segment_count, header.flash_mode, header.flash_size_freq, header.entry_addr
    );
    println!("  SHA-256 Digest         : {}", header.sha256_hex);
    println!("================================================================================");

    Ok(payload)
}

fn find_espflash() -> Result<PathBuf, String> {
    if let Ok(path) = env::var("ESPFLASH_BIN") {
        let p = PathBuf::from(path);
        if p.is_file() {
            return Ok(p);
        }
    }
    let home = env::var("HOME").unwrap_or_else(|_| "/home/console-admin".to_string());
    let cargo_bin = PathBuf::from(home)
        .join(".cargo")
        .join("bin")
        .join("espflash");
    if cargo_bin.is_file() {
        return Ok(cargo_bin);
    }
    // Check PATH
    if let Ok(path_var) = env::var("PATH") {
        for dir in env::split_paths(&path_var) {
            let candidate = dir.join("espflash");
            if candidate.is_file() {
                return Ok(candidate);
            }
        }
    }
    Err("espflash binary not found in PATH or ~/.cargo/bin".to_string())
}

/// Queries GET `/status` and returns parsed [`OtaStatusInfo`].
pub fn fetch_status(host: &str, port: u16, timeout: Duration) -> Result<OtaStatusInfo, String> {
    let raw = query_http(host, port, "/status", timeout)?;
    parse_status_json(&raw)
}

/// Queries GET `/snapshot` and returns snapshot metadata.
pub fn fetch_snapshot_info(
    host: &str,
    port: u16,
    timeout: Duration,
) -> Result<SnapshotInfo, String> {
    let payload = query_http_bytes(host, port, "/snapshot", timeout)?;
    let len = payload.len();
    let has_soi = len >= 2 && payload[0] == 0xFF && payload[1] == 0xD8;
    let has_eoi = len >= 4 && payload[len - 2] == 0xFF && payload[len - 1] == 0xD9;
    let dims = parse_jpeg_dimensions(&payload);
    Ok(SnapshotInfo {
        len,
        has_soi,
        has_eoi,
        dims,
    })
}

fn query_http(host: &str, port: u16, path: &str, timeout: Duration) -> Result<String, String> {
    let bytes = query_http_bytes(host, port, path, timeout)?;
    String::from_utf8(bytes).map_err(|e| format!("Invalid UTF-8 response from {path}: {e}"))
}

fn query_http_bytes(
    host: &str,
    port: u16,
    path: &str,
    timeout: Duration,
) -> Result<Vec<u8>, String> {
    let addr = format!("{host}:{port}")
        .to_socket_addrs()
        .map_err(|e| format!("Could not resolve {host}:{port}: {e}"))?
        .next()
        .ok_or_else(|| format!("No address for {host}:{port}"))?;

    let mut stream = TcpStream::connect_timeout(&addr, timeout)
        .map_err(|e| format!("Connect to {host}:{port} failed: {e}"))?;
    stream
        .set_read_timeout(Some(timeout))
        .map_err(|e| e.to_string())?;
    stream
        .set_write_timeout(Some(timeout))
        .map_err(|e| e.to_string())?;

    let req = format!("GET {path} HTTP/1.1\r\nHost: {host}:{port}\r\nConnection: close\r\n\r\n");
    stream
        .write_all(req.as_bytes())
        .map_err(|e| format!("Write failed: {e}"))?;

    let mut resp = Vec::new();
    stream
        .read_to_end(&mut resp)
        .map_err(|e| format!("Read failed: {e}"))?;

    let header_end = resp
        .windows(4)
        .position(|w| w == b"\r\n\r\n")
        .ok_or_else(|| "Malformed HTTP response (no header terminator)".to_string())?;

    let header_str = String::from_utf8_lossy(&resp[..header_end]);
    if !header_str.contains("200 OK") {
        return Err(format!("Server returned non-200 status:\n{header_str}"));
    }

    Ok(resp[header_end + 4..].to_vec())
}

/// Streams HTTP POST `/update` in 16KB chunks over a raw TCP stream.
pub fn stream_ota_post(
    host: &str,
    port: u16,
    token: &str,
    payload: &[u8],
    timeout: Duration,
) -> Result<String, String> {
    if host.contains('\r') || host.contains('\n') || token.contains('\r') || token.contains('\n') {
        return Err("CRLF injection detected in host or token".to_string());
    }

    let addr = format!("{host}:{port}")
        .to_socket_addrs()
        .map_err(|e| format!("Could not resolve {host}:{port}: {e}"))?
        .next()
        .ok_or_else(|| format!("No address for {host}:{port}"))?;

    let mut stream = TcpStream::connect_timeout(&addr, timeout)
        .map_err(|e| format!("TCP connect failed: {e}"))?;
    let _ = stream.set_nodelay(true);
    let _ = stream.set_read_timeout(Some(timeout));
    let _ = stream.set_write_timeout(Some(timeout));

    let headers = format!(
        "POST /update HTTP/1.1\r\n\
         Host: {host}:{port}\r\n\
         X-OTA-Token: {token}\r\n\
         Content-Type: application/octet-stream\r\n\
         Content-Length: {}\r\n\
         Connection: close\r\n\
         \r\n",
        payload.len()
    );

    stream
        .write_all(headers.as_bytes())
        .map_err(|e| format!("Failed to send headers: {e}"))?;

    let total = payload.len();
    let mut sent = 0;
    let mut next_report_pct = 25;
    let start_time = Instant::now();

    while sent < total {
        let end = (sent + CHUNK_SIZE).min(total);
        stream
            .write_all(&payload[sent..end])
            .map_err(|e| format!("Chunk send failed: {e}"))?;
        sent = end;
        let pct = (sent * 100) / total;
        if pct >= next_report_pct {
            let elapsed = start_time.elapsed().as_secs_f64();
            println!("[OTA-UPLOAD] Progress: {sent}/{total} bytes ({pct}%) in {elapsed:.2}s");
            next_report_pct += 25;
        }
        if sent < total {
            sleep(Duration::from_millis(2));
        }
    }

    let upload_elapsed = start_time.elapsed().as_secs_f64();
    let rate_kb = (sent as f64 / 1024.0) / upload_elapsed.max(0.001);
    println!(
        "[OTA-UPLOAD] Sent {sent} bytes in {upload_elapsed:.2}s ({rate_kb:.1} KB/s). Waiting for NOR flash write & HTTP response..."
    );

    let mut resp_chunks = Vec::new();
    let mut buf = [0u8; 4096];
    let mut content_length: Option<usize> = None;
    let mut header_done = false;

    loop {
        match stream.read(&mut buf) {
            Ok(0) => break, // EOF
            Ok(n) => {
                resp_chunks.extend_from_slice(&buf[..n]);
                if !header_done {
                    if let Some(pos) = resp_chunks.windows(4).position(|w| w == b"\r\n\r\n") {
                        header_done = true;
                        let header_part = String::from_utf8_lossy(&resp_chunks[..pos]);
                        for line in header_part.lines() {
                            if line.to_ascii_lowercase().starts_with("content-length:") {
                                if let Some((_, val)) = line.split_once(':') {
                                    content_length = val.trim().parse::<usize>().ok();
                                }
                            }
                        }
                    }
                }
                if header_done {
                    if let Some(pos) = resp_chunks.windows(4).position(|w| w == b"\r\n\r\n") {
                        let body_len = resp_chunks.len() - (pos + 4);
                        if let Some(cl) = content_length {
                            if body_len >= cl {
                                break;
                            }
                        }
                    }
                }
            }
            Err(e) => return Err(format!("Timed out or read failed from POST /update: {e}")),
        }
    }

    let raw_resp = String::from_utf8_lossy(&resp_chunks).to_string();
    if !raw_resp.contains("200 OK")
        || !raw_resp.contains("Firmware update successful! Rebooting...")
    {
        return Err(format!(
            "Unexpected HTTP response from POST /update:\n{raw_resp}"
        ));
    }

    Ok(raw_resp.trim().to_string())
}

/// Runs the `ota-deploy` subcommand.
///
/// # Errors
/// Returns error if argument parsing, staging, upload, or post-boot verification fails.
pub fn run(args: &[String]) -> Result<(), String> {
    if args.iter().any(|a| a == "--help" || a == "-h") {
        print_help();
        return Ok(());
    }

    let mut opts = OtaDeployOptions::default();
    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "--host" => {
                i += 1;
                if i >= args.len() {
                    return Err("Missing value for --host".to_string());
                }
                opts.host = args[i].clone();
            }
            "--port" => {
                i += 1;
                if i >= args.len() {
                    return Err("Missing value for --port".to_string());
                }
                opts.port = args[i]
                    .parse::<u16>()
                    .map_err(|e| format!("Invalid port '{}': {e}", args[i]))?;
            }
            "--elf" => {
                i += 1;
                if i >= args.len() {
                    return Err("Missing value for --elf".to_string());
                }
                opts.elf_path = PathBuf::from(&args[i]);
            }
            "--bin" => {
                i += 1;
                if i >= args.len() {
                    return Err("Missing value for --bin".to_string());
                }
                opts.bin_path = PathBuf::from(&args[i]);
            }
            "--token" => {
                i += 1;
                if i >= args.len() {
                    return Err("Missing value for --token".to_string());
                }
                opts.token = Some(args[i].clone());
            }
            "--stage-only" => {
                opts.stage_only = true;
            }
            "--expected-dims" => {
                i += 1;
                if i >= args.len() {
                    return Err("Missing value for --expected-dims".to_string());
                }
                opts.expected_dims = args[i].clone();
            }
            other => return Err(format!("Unknown argument '{other}'")),
        }
        i += 1;
    }

    if !opts.stage_only && opts.host.is_empty() {
        return Err(
            "[OTA-ERROR] No camera host provided. Pass --host <camera-ip> or set CAM_HOST environment variable."
                .to_string(),
        );
    }

    if !opts.stage_only && opts.token.is_none() {
        return Err(
            "[OTA-ERROR] No OTA authentication token provided. Pass --token <SECRET> or set CAM_OTA_TOKEN environment variable."
                .to_string(),
        );
    }

    let payload = stage_and_validate_image(&opts.elf_path, &opts.bin_path, &opts.host, opts.port)?;

    if opts.stage_only {
        println!("[OTA-STAGE] --stage-only complete (exit 0).");
        return Ok(());
    }

    let pre_status = fetch_status(&opts.host, opts.port, Duration::from_secs(5)).map_err(|e| {
        format!(
            "[OTA-ERROR] Failed to query pre-update /status on {}:{}: {e}",
            opts.host, opts.port
        )
    })?;

    let pre_slot = pre_status.active_partition.as_str();
    let target_slot = expected_next_slot(pre_slot)?;

    println!(
        "[OTA-PRE] Current active_partition='{}' (uptime_ms={}) -> Target passive slot='{}'",
        pre_slot, pre_status.uptime_ms, target_slot
    );

    let token = opts.token.as_deref().unwrap_or("");
    let http_resp = stream_ota_post(
        &opts.host,
        opts.port,
        token,
        &payload,
        Duration::from_secs(60),
    )
    .map_err(|e| format!("[OTA-ERROR] Wireless OTA upload failed: {e}"))?;

    let first_line = http_resp.lines().next().unwrap_or("");
    println!(
        "[OTA-HTTP] Server responded: {first_line} | Firmware update successful! Rebooting..."
    );

    println!("[OTA-WAIT] Waiting for ESP32 software reset and Wi-Fi reconnection...");
    sleep(Duration::from_millis(3500));

    let deadline = Instant::now() + Duration::from_secs(45);
    let mut post_status = None;
    let mut last_err = String::new();

    while Instant::now() < deadline {
        match fetch_status(&opts.host, opts.port, Duration::from_millis(2500)) {
            Ok(st) => {
                if st.active_partition == target_slot {
                    post_status = Some(st);
                    break;
                }
                last_err = format!("active_partition still '{}'", st.active_partition);
            }
            Err(e) => last_err = e,
        }
        sleep(Duration::from_secs(1));
    }

    let post_status = post_status.ok_or_else(|| {
        format!(
            "[OTA-ERROR] Timed out waiting for board to boot into '{target_slot}' (last error: {last_err})"
        )
    })?;

    println!(
        "[OTA-POST] Verified boot into active_partition='{}' (uptime_ms={}, framesize={}, wifi_rssi={} dBm)",
        post_status.active_partition, post_status.uptime_ms, post_status.framesize, post_status.wifi_rssi
    );

    if let Some(reported_sha) = post_status.app_sha256 {
        let reported = reported_sha.trim().to_ascii_lowercase();
        let hash_appended = payload.len() >= ESP_IMAGE_HEADER_LEN + 32 && payload[23] == 1;
        let expected_app_sha = if hash_appended {
            Sha256::digest_hex(&payload[..payload.len() - 32])
        } else {
            Sha256::digest_hex(&payload)
        };
        let full_bin_sha = Sha256::digest_hex(&payload);

        if reported != expected_app_sha && reported != full_bin_sha {
            return Err(format!(
                "[OTA-ERROR] Post-boot app_sha256 mismatch: device reported '{reported}', expected '{expected_app_sha}' (or '{full_bin_sha}')"
            ));
        }
        println!("[OTA-VERIFY] Post-boot app_sha256 verified match: {reported}");
    }

    let snap = fetch_snapshot_info(&opts.host, opts.port, Duration::from_secs(10))
        .map_err(|e| format!("[OTA-ERROR] Post-boot GET /snapshot failed: {e}"))?;

    let dims_valid = if let Some((w, h)) = snap.dims {
        if opts.expected_dims.eq_ignore_ascii_case("any") {
            true
        } else if let Some((exp_w_s, exp_h_s)) = opts.expected_dims.split_once('x') {
            let exp_w = exp_w_s.trim().parse::<u16>().unwrap_or(0);
            let exp_h = exp_h_s.trim().parse::<u16>().unwrap_or(0);
            w == exp_w && h == exp_h
        } else {
            false
        }
    } else {
        false
    };

    if !snap.has_soi || !snap.has_eoi || !dims_valid {
        return Err(format!(
            "[OTA-ERROR] Post-boot /snapshot invalid JPEG (len={}, soi={}, eoi={}, dims={:?}, expected={})",
            snap.len, snap.has_soi, snap.has_eoi, snap.dims, opts.expected_dims
        ));
    }

    println!(
        "[OTA-VERIFY] Post-boot GET /snapshot 200 OK ({} bytes, SOI=0xFFD8, EOI=0xFFD9, dimensions={:?})",
        snap.len, snap.dims
    );
    println!(
        "[OTA-SUCCESS] Wireless OTA transition {} -> {} verified!",
        pre_slot, post_status.active_partition
    );

    Ok(())
}

fn print_help() {
    println!("Usage: xtask ota-deploy [OPTIONS]");
    println!();
    println!("Stage and deploy bare-metal Rust OTA firmware to ESP32 Klipper Camera.");
    println!();
    println!("Options:");
    println!("  --host <HOST>            Target IP address (default: $CAM_HOST)");
    println!("  --port <PORT>            Target HTTP port (default: 80)");
    println!("  --elf <PATH>             Path to built Xtensa ELF binary");
    println!("  --bin <PATH>             Path to output ESP-IDF .bin image");
    println!(
        "  --token <TOKEN>          X-OTA-Token authentication secret (default: $CAM_OTA_TOKEN)"
    );
    println!("  --stage-only             Convert ELF to .bin and validate 0xE9 header without touching network");
    println!("  --expected-dims <DIMS>   Expected JPEG dimensions (default: 1280x720, or 'any')");
    println!("  -h, --help               Print help information");
}
