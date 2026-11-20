//! Wireless Dual-Slot OTA Deployment & Staging Tool for Klipper Camera
//! (ESP32-WROVER-DEV).
//!
//! Replicates `scripts/ota_deploy.py` in pure Rust with zero external network
//! crates.

use crate::sha256::Sha256;
use std::collections::BTreeMap;
use std::env;
use std::fmt;
use std::fs;
use std::io::{Read, Write};
use std::net::{TcpStream, ToSocketAddrs};
use std::ops::Range;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::sync::atomic::{compiler_fence, Ordering};
use std::thread::sleep;
use std::time::{Duration, Instant};

pub const ESP_IMAGE_HEADER_MAGIC: u8 = 0xE9;
pub const ESP_IMAGE_MIN_SEGMENTS: u8 = 1;
pub const ESP_IMAGE_MAX_SEGMENTS: u8 = 16;
pub const ESP_IMAGE_MAX_SPI_MODE: u8 = 5;
pub const ESP_IMAGE_CHIP_ID_ESP32: u16 = 0x0000;
pub const ESP_IMAGE_HEADER_LEN: usize = 24;
pub const ESP_IMAGE_DIGEST_LEN: usize = 32;
pub const OTA_PARTITION_MAX_SIZE: usize = 0x001B_0000; // 1,769,472 bytes
pub const CHUNK_SIZE: usize = 16384; // 16 KB chunks for high-throughput upload
pub const MAX_OTA_RESPONSE_BYTES: usize = 64 * 1024; // 64 KiB cap (M-132)
pub const MAX_HTTP_GET_BYTES: usize = 4 * 1024 * 1024; // 4 MiB snapshot cap
pub const DEFAULT_OTA_TOKEN: &str = "klipper-cam-ota-2026";

const ESP32_IRAM_ENTRY_RANGE: Range<u32> = 0x4007_8000..0x400A_0000;
const ESP32_IROM_ENTRY_RANGE: Range<u32> = 0x400D_0000..0x4040_0000;

const OTA_TOKEN_ENV_KEYS: [&str; 4] = [
    "CAM_OTA_TOKEN",
    "ESP32_CAM_OTA_TOKEN",
    "OTA_TOKEN",
    "WIFI_OTA_TOKEN",
];

/// Heap-allocated secret string that redacts formatting and zeroizes on drop.
#[derive(Clone, PartialEq, Eq)]
pub struct SecretString(String);

impl SecretString {
    /// Wraps a plaintext secret string.
    #[must_use]
    pub fn new(secret: String) -> Self {
        Self(secret)
    }

    /// Exposes the underlying secret string slice.
    #[must_use]
    pub fn expose(&self) -> &str {
        &self.0
    }

    /// Returns `true` if the secret is empty.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }
}

impl fmt::Debug for SecretString {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("[REDACTED]")
    }
}

impl fmt::Display for SecretString {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("[REDACTED]")
    }
}

impl Drop for SecretString {
    fn drop(&mut self) {
        // JUSTIFICATION: Volatile byte overwrite prevents dead-store removal
        // when zeroizing secret buffers on drop.
        // SAFETY: `self.0` is exclusively owned by `&mut self`; all bytes in
        // `as_mut_vec()` are valid initialized memory and writing 0 keeps the
        // buffer valid UTF-8 before clearing.
        unsafe {
            let bytes = self.0.as_mut_vec();
            for byte in bytes.iter_mut() {
                std::ptr::write_volatile(byte, 0);
            }
            compiler_fence(Ordering::SeqCst);
            bytes.clear();
        }
    }
}

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
    pub token: Option<SecretString>,
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

/// Validates a candidate OTA token, rejecting empty or hardcoded default
/// values (`M-131` / `C1-033`).
///
/// # Errors
/// Returns an error if the token is empty, contains CR/LF, or matches
/// [`DEFAULT_OTA_TOKEN`].
pub fn validate_ota_token(raw: &str) -> Result<SecretString, String> {
    let trimmed = raw.trim();
    if trimmed.is_empty() {
        return Err("[OTA-ERROR] OTA authentication token must not be empty.".to_string());
    }
    if trimmed.contains('\r') || trimmed.contains('\n') {
        return Err("[OTA-ERROR] CRLF injection detected in OTA token.".to_string());
    }
    if trimmed == DEFAULT_OTA_TOKEN {
        return Err(
            "[OTA-ERROR] Refusing hardcoded DEFAULT_OTA_TOKEN; set a unique \
             CAM_OTA_TOKEN."
                .to_string(),
        );
    }
    Ok(SecretString::new(trimmed.to_string()))
}

/// Parses a single `.env` value with symmetric quote handling and unquoted
/// inline `#` comment stripping.
fn parse_env_value(raw: &str) -> Option<String> {
    let trimmed = raw.trim();
    if trimmed.is_empty() {
        return None;
    }
    if let Some(after_open) = trimmed.strip_prefix('"') {
        let mut out = String::new();
        let mut chars = after_open.chars();
        while let Some(ch) = chars.next() {
            match ch {
                '\\' => out.push(chars.next()?),
                '"' => {
                    let tail = chars.as_str().trim();
                    if tail.is_empty() || tail.starts_with('#') {
                        return Some(out);
                    }
                    return None;
                }
                other => out.push(other),
            }
        }
        return None;
    }
    if let Some(after_open) = trimmed.strip_prefix('\'') {
        let close_idx = after_open.find('\'')?;
        let tail = after_open[close_idx + 1..].trim();
        if tail.is_empty() || tail.starts_with('#') {
            return Some(after_open[..close_idx].to_string());
        }
        return None;
    }

    // Unquoted value: strip trailing `# comment`.
    let value_part = match trimmed.find('#') {
        Some(idx) => trimmed[..idx].trim(),
        None => trimmed,
    };
    if value_part.is_empty() {
        None
    } else {
        Some(value_part.to_string())
    }
}

/// Parses `.env.local` contents into a zeroizing key map and selects the
/// highest-precedence valid OTA token.
#[must_use]
pub fn resolve_token_from_env_content(content: &str) -> Option<SecretString> {
    let mut entries: BTreeMap<String, SecretString> = BTreeMap::new();
    for raw_line in content.lines() {
        let trimmed = raw_line.trim();
        if trimmed.is_empty() || trimmed.starts_with('#') {
            continue;
        }
        let body = trimmed
            .strip_prefix("export")
            .filter(|rest| rest.starts_with(char::is_whitespace))
            .map_or(trimmed, str::trim_start);
        let Some((key_raw, val_raw)) = body.split_once('=') else {
            continue;
        };
        let key = key_raw.trim();
        if !OTA_TOKEN_ENV_KEYS.contains(&key) {
            continue;
        }
        if let Some(parsed) = parse_env_value(val_raw) {
            entries.insert(key.to_string(), SecretString::new(parsed));
        }
    }

    for key in OTA_TOKEN_ENV_KEYS {
        if let Some(val) = entries.get(key) {
            if let Ok(valid) = validate_ota_token(val.expose()) {
                return Some(valid);
            }
        }
    }
    None
}

/// Resolves the default OTA authentication token from environment or local
/// configuration, rejecting empty or hardcoded default values.
#[must_use]
pub fn resolve_default_token() -> Option<SecretString> {
    for var in OTA_TOKEN_ENV_KEYS {
        if let Ok(raw_val) = env::var(var) {
            let secret = SecretString::new(raw_val);
            if let Ok(valid) = validate_ota_token(secret.expose()) {
                return Some(valid);
            }
        }
    }

    let env_local = repo_root().join(".env.local");
    if let Ok(raw_content) = fs::read_to_string(env_local) {
        let content_secret = SecretString::new(raw_content);
        return resolve_token_from_env_content(content_secret.expose());
    }

    None
}

/// Maps current active partition to expected target partition after OTA slot
/// flip.
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

/// Extracts `(width, height)` from the JPEG SOF0/SOF2 marker in a byte slice.
#[must_use]
pub fn parse_jpeg_dimensions(data: &[u8]) -> Option<(u16, u16)> {
    if data.len() < 4 || data[0] != 0xFF || data[1] != 0xD8 {
        return None;
    }
    let mut i = 2;
    let n = data.len();
    while i < n {
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
        if marker == 0x00
            || marker == 0x01
            || marker == 0xD8
            || marker == 0xD9
            || (0xD0..=0xD7).contains(&marker)
        {
            continue;
        }
        if i + 2 > n {
            break;
        }
        let seg_len = u16::from_be_bytes([data[i], data[i + 1]]) as usize;
        if seg_len < 2 || i + seg_len > n {
            break;
        }
        if marker == 0xC0 || marker == 0xC2 {
            // SOF0/SOF2 requires at least 7 bytes: length (2) + precision (1)
            // + height (2) + width (2) (`M-207` / `C1-052`).
            if seg_len < 7 || i + 7 > n {
                return None;
            }
            let height = u16::from_be_bytes([data[i + 3], data[i + 4]]);
            let width = u16::from_be_bytes([data[i + 5], data[i + 6]]);
            return Some((width, height));
        }
        i += seg_len;
    }
    None
}

#[derive(Debug, Clone, PartialEq, Eq)]
enum JsonValue<'a> {
    Str(&'a str),
    Raw(&'a str),
}

/// Parses a JSON string token starting at `s[0] == '"'`, returning the inner
/// slice and the byte index immediately after the closing quote.
fn parse_json_string_token(s: &str) -> Option<(&str, usize)> {
    let bytes = s.as_bytes();
    if bytes.first().copied() != Some(b'"') {
        return None;
    }
    let mut i = 1;
    while i < bytes.len() {
        if bytes[i] == b'\\' {
            if i + 1 >= bytes.len() {
                return None;
            }
            i += 2;
        } else if bytes[i] == b'"' {
            return Some((&s[1..i], i + 1));
        } else {
            i += 1;
        }
    }
    None
}

/// Skips a nested JSON `{...}` or `[...]` value starting at `s[0]`.
fn skip_nested_json_value(s: &str) -> Option<usize> {
    let bytes = s.as_bytes();
    let mut depth = 0usize;
    let mut i = 0usize;
    while i < bytes.len() {
        match bytes[i] {
            b'"' => {
                let (_, adv) = parse_json_string_token(&s[i..])?;
                i += adv;
            }
            b'{' | b'[' => {
                depth += 1;
                i += 1;
            }
            b'}' | b']' => {
                depth = depth.checked_sub(1)?;
                i += 1;
                if depth == 0 {
                    return Some(i);
                }
            }
            _ => i += 1,
        }
    }
    None
}

/// Strictly parses a flat top-level JSON object into key-value pairs.
fn parse_flat_json_object(raw: &str) -> Result<BTreeMap<&str, JsonValue<'_>>, String> {
    let trimmed = raw.trim();
    if !trimmed.starts_with('{') || !trimmed.ends_with('}') {
        return Err("Invalid JSON object: missing enclosing braces".to_string());
    }
    let mut rest = trimmed[1..trimmed.len() - 1].trim();
    let mut fields = BTreeMap::new();

    while !rest.is_empty() {
        let (key, after_key) = parse_json_string_token(rest)
            .ok_or_else(|| "Malformed JSON key in status payload".to_string())?;
        rest = rest[after_key..].trim_start();
        rest = rest
            .strip_prefix(':')
            .ok_or_else(|| format!("Missing ':' after JSON key '{key}'"))?
            .trim_start();

        if rest.starts_with('"') {
            let (val, after_val) = parse_json_string_token(rest)
                .ok_or_else(|| format!("Unterminated JSON string for '{key}'"))?;
            fields.insert(key, JsonValue::Str(val));
            rest = rest[after_val..].trim_start();
        } else if rest.starts_with('{') || rest.starts_with('[') {
            let adv = skip_nested_json_value(rest)
                .ok_or_else(|| format!("Malformed nested JSON value for '{key}'"))?;
            fields.insert(key, JsonValue::Raw(&rest[..adv]));
            rest = rest[adv..].trim_start();
        } else {
            let end_idx = rest
                .find(|c: char| c == ',' || c.is_whitespace())
                .unwrap_or(rest.len());
            if end_idx == 0 {
                return Err(format!("Empty JSON value for '{key}'"));
            }
            fields.insert(key, JsonValue::Raw(&rest[..end_idx]));
            rest = rest[end_idx..].trim_start();
        }

        if let Some(after_comma) = rest.strip_prefix(',') {
            rest = after_comma.trim_start();
            if rest.is_empty() {
                return Err("Trailing comma in JSON object".to_string());
            }
        } else if !rest.is_empty() {
            return Err("Expected ',' between JSON object fields".to_string());
        }
    }

    Ok(fields)
}

/// Helper to extract top-level string or scalar tokens from flat JSON objects.
#[must_use]
pub fn extract_json_field<'a>(json: &'a str, key: &str) -> Option<&'a str> {
    let fields = parse_flat_json_object(json).ok()?;
    match fields.get(key)? {
        JsonValue::Str(s) | JsonValue::Raw(s) => Some(*s),
    }
}

/// Parses the `/status` JSON endpoint response, failing closed when required
/// fields are missing or malformed (`M-133` / `C1-035`).
///
/// # Errors
/// Returns an error if JSON is malformed or if required fields
/// (`active_partition`, `uptime_ms`) or numeric fields are missing/invalid.
pub fn parse_status_json(raw: &str) -> Result<OtaStatusInfo, String> {
    let fields = parse_flat_json_object(raw)?;

    let active_partition = match fields.get("active_partition") {
        Some(JsonValue::Str(s)) if !s.trim().is_empty() => (*s).to_string(),
        Some(_) => {
            return Err("Invalid 'active_partition' field in /status JSON".to_string());
        }
        None => {
            return Err("Missing required 'active_partition' field in /status JSON".to_string());
        }
    };

    let uptime_ms = match fields.get("uptime_ms") {
        Some(JsonValue::Raw(raw_num)) => raw_num
            .parse::<u64>()
            .map_err(|_| format!("Non-numeric 'uptime_ms' value '{raw_num}' in /status JSON"))?,
        Some(JsonValue::Str(_)) => {
            return Err("Expected numeric 'uptime_ms' in /status JSON".to_string());
        }
        None => return Err("Missing required 'uptime_ms' field in /status JSON".to_string()),
    };

    let framesize = match fields.get("framesize") {
        Some(JsonValue::Str(s)) => (*s).to_string(),
        Some(_) => return Err("Invalid non-string 'framesize' in /status JSON".to_string()),
        None => String::new(),
    };

    let wifi_rssi = match fields.get("wifi_rssi") {
        Some(JsonValue::Raw(raw_num)) => raw_num
            .parse::<i32>()
            .map_err(|_| format!("Non-numeric 'wifi_rssi' value '{raw_num}' in /status JSON"))?,
        Some(JsonValue::Str(_)) => {
            return Err("Expected numeric 'wifi_rssi' in /status JSON".to_string());
        }
        None => 0,
    };

    let app_sha256 = match fields.get("app_sha256") {
        Some(JsonValue::Str(s)) if !s.is_empty() => Some(s.to_ascii_lowercase()),
        Some(JsonValue::Str(_)) | Some(JsonValue::Raw("null")) | None => None,
        Some(_) => return Err("Invalid 'app_sha256' field in /status JSON".to_string()),
    };

    Ok(OtaStatusInfo {
        active_partition,
        uptime_ms,
        framesize,
        wifi_rssi,
        app_sha256,
    })
}

fn format_digest_hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}

/// Strictly validates an ESP32 application binary image with full parity to
/// firmware `ota.rs::validate_ota_image_integrity` (`M-028`, `M-029`).
///
/// # Errors
/// Returns an error if header magic, segment count, SPI mode, chip ID, entry
/// address, mandatory SHA-256 flag, segment bounds/alignment, XOR checksum,
/// exact aligned image length, or SHA-256 digest verification fails.
pub fn validate_esp_image_strict(payload: &[u8]) -> Result<EspImageHeader, String> {
    let size = payload.len();
    if size < ESP_IMAGE_HEADER_LEN {
        return Err(format!(
            "Image too small ({size} bytes < {ESP_IMAGE_HEADER_LEN} bytes)"
        ));
    }
    if size > OTA_PARTITION_MAX_SIZE {
        return Err(format!(
            "Image size ({size} bytes) exceeds OTA slot limit \
             (0x{OTA_PARTITION_MAX_SIZE:X} = {OTA_PARTITION_MAX_SIZE} bytes)"
        ));
    }

    let magic = payload[0];
    let segment_count = payload[1];
    let flash_mode = payload[2];
    let flash_size_freq = payload[3];
    let entry_addr = u32::from_le_bytes([payload[4], payload[5], payload[6], payload[7]]);
    let chip_id = u16::from_le_bytes([payload[12], payload[13]]);

    if magic != ESP_IMAGE_HEADER_MAGIC {
        return Err(format!(
            "Invalid ESP32 image magic: 0x{magic:02X} (expected 0xE9)"
        ));
    }
    if !(ESP_IMAGE_MIN_SEGMENTS..=ESP_IMAGE_MAX_SEGMENTS).contains(&segment_count) {
        return Err(format!(
            "Invalid ESP32 segment count: {segment_count} \
             (expected {ESP_IMAGE_MIN_SEGMENTS}..={ESP_IMAGE_MAX_SEGMENTS})"
        ));
    }
    if flash_mode > ESP_IMAGE_MAX_SPI_MODE {
        return Err(format!(
            "Invalid ESP32 SPI flash mode: {flash_mode} (expected 0..={ESP_IMAGE_MAX_SPI_MODE})"
        ));
    }
    if chip_id != ESP_IMAGE_CHIP_ID_ESP32 {
        return Err(format!(
            "Invalid ESP32 chip_id: 0x{chip_id:04X} (expected 0x{ESP_IMAGE_CHIP_ID_ESP32:04X})"
        ));
    }
    let valid_entry = ESP32_IRAM_ENTRY_RANGE.contains(&entry_addr)
        || ESP32_IROM_ENTRY_RANGE.contains(&entry_addr);
    if !valid_entry {
        return Err(format!(
            "Invalid ESP32 entry address: 0x{entry_addr:08X} (outside IRAM/IROM)"
        ));
    }
    if payload[23] != 1 {
        return Err(format!(
            "Mandatory appended SHA-256 required (header[23]={}, expected 1)",
            payload[23]
        ));
    }

    let mut offset = ESP_IMAGE_HEADER_LEN;
    let mut xor_chk = 0xEFu8;
    for seg_idx in 0..segment_count {
        let hdr_end = offset
            .checked_add(8)
            .ok_or_else(|| "Segment header offset overflow".to_string())?;
        if hdr_end > size {
            return Err(format!(
                "Truncated segment {seg_idx} header at offset {offset}"
            ));
        }
        let load_addr = u32::from_le_bytes([
            payload[offset],
            payload[offset + 1],
            payload[offset + 2],
            payload[offset + 3],
        ]);
        let data_len = u32::from_le_bytes([
            payload[offset + 4],
            payload[offset + 5],
            payload[offset + 6],
            payload[offset + 7],
        ]) as usize;
        if data_len > 0 && (load_addr & 3) != 0 {
            return Err(format!(
                "Unaligned segment {seg_idx} load address: 0x{load_addr:08X}"
            ));
        }
        offset = hdr_end;
        let seg_end = offset
            .checked_add(data_len)
            .ok_or_else(|| "Segment length overflow".to_string())?;
        if seg_end > size {
            return Err(format!(
                "Segment {seg_idx} exceeds image length ({seg_end} > {size})"
            ));
        }
        for &byte in &payload[offset..seg_end] {
            xor_chk ^= byte;
        }
        offset = seg_end;
    }

    let aligned_end = offset
        .checked_add(16)
        .ok_or_else(|| "Image alignment overflow".to_string())?
        & !15;
    let checksum_offset = aligned_end
        .checked_sub(1)
        .ok_or_else(|| "Image checksum offset underflow".to_string())?;
    if size <= checksum_offset {
        return Err(format!(
            "Image truncated before XOR checksum byte at offset {checksum_offset}"
        ));
    }
    if payload[checksum_offset] != xor_chk {
        return Err(format!(
            "ESP32 image XOR checksum mismatch: found 0x{:02X}, computed 0x{xor_chk:02X}",
            payload[checksum_offset]
        ));
    }

    let expected_total_len = aligned_end
        .checked_add(ESP_IMAGE_DIGEST_LEN)
        .ok_or_else(|| "Image digest length overflow".to_string())?;
    if size != expected_total_len {
        return Err(format!(
            "Image length mismatch: expected exact {expected_total_len} bytes \
             (aligned_end={aligned_end} + 32-byte SHA-256), got {size} bytes"
        ));
    }

    let calc_digest = Sha256::digest(&payload[..aligned_end]);
    let appended_digest = &payload[aligned_end..];
    if calc_digest != appended_digest {
        return Err(format!(
            "Appended SHA-256 mismatch: computed {} != appended {}",
            Sha256::digest_hex(&payload[..aligned_end]),
            format_digest_hex(appended_digest)
        ));
    }

    let sha256_hex = Sha256::digest_hex(payload);

    Ok(EspImageHeader {
        magic,
        segment_count,
        flash_mode,
        flash_size_freq,
        entry_addr,
        hash_appended: true,
        size,
        sha256_hex,
    })
}

/// Validates an ESP32 binary image header, mandatory SHA-256 (`image[23]==1`),
/// SPI flash mode (`<= 5`), IRAM/IROM entry address, segment bounds/alignment,
/// 8-bit XOR checksum, exact `aligned_end + 32 == payload.len()`, and appended
/// SHA-256 digest (`M-028`, `M-029`).
///
/// Unconditionally delegates to [`validate_esp_image_strict`].
///
/// # Errors
/// Returns an error if magic, segment count, SPI mode, chip ID, entry address,
/// mandatory SHA-256, structural integrity, or digest check fails.
pub fn validate_esp_image(payload: &[u8]) -> Result<EspImageHeader, String> {
    validate_esp_image_strict(payload)
}

/// Stages an ESP-IDF application image using `espflash save-image` and
/// validates its full header, segments, checksum, and SHA-256 digest.
///
/// # Errors
/// Returns an error if file conversion or validation fails.
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
            "--",
        ])
        .arg(elf_path.as_os_str())
        .arg(bin_path.as_os_str())
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

    let header = validate_esp_image_strict(&payload)?;

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
        "  ESP32 Image Header     : magic=0x{:02X} segments={} mode=0x{:02X} \
         size_freq=0x{:02X} entry=0x{:08X}",
        header.magic,
        header.segment_count,
        header.flash_mode,
        header.flash_size_freq,
        header.entry_addr
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
    let home = env::var("HOME")
        .or_else(|_| env::var("USERPROFILE"))
        .unwrap_or_else(|_| "/home/console-admin".to_string());
    let bin_name = if cfg!(windows) {
        "espflash.exe"
    } else {
        "espflash"
    };
    let cargo_bin = PathBuf::from(home)
        .join(".cargo")
        .join("bin")
        .join(bin_name);
    if cargo_bin.is_file() {
        return Ok(cargo_bin);
    }
    if let Ok(path_var) = env::var("PATH") {
        for dir in env::split_paths(&path_var) {
            let candidate = dir.join(bin_name);
            if candidate.is_file() {
                return Ok(candidate);
            }
        }
    }
    Err("espflash binary not found in PATH or ~/.cargo/bin".to_string())
}

/// Checks whether an HTTP status line strictly represents HTTP/1.x 200 OK.
fn is_http_200_status_line(status_line: &str) -> bool {
    let trimmed = status_line.trim_end_matches('\r');
    let Some(rest) = trimmed
        .strip_prefix("HTTP/1.1 ")
        .or_else(|| trimmed.strip_prefix("HTTP/1.0 "))
    else {
        return false;
    };
    rest == "200" || rest.starts_with("200 ")
}

/// Queries GET `/status` and returns parsed [`OtaStatusInfo`].
///
/// # Errors
/// Returns an error if the HTTP request fails or the JSON payload is invalid.
pub fn fetch_status(host: &str, port: u16, timeout: Duration) -> Result<OtaStatusInfo, String> {
    let raw = query_http(host, port, "/status", timeout)?;
    parse_status_json(&raw)
}

/// Queries GET `/snapshot` and returns snapshot metadata.
///
/// # Errors
/// Returns an error if the HTTP request fails.
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
    if host.contains('\r') || host.contains('\n') || path.contains('\r') || path.contains('\n') {
        return Err("CRLF injection detected in host or path".to_string());
    }

    let addr = format!("{host}:{port}")
        .to_socket_addrs()
        .map_err(|e| format!("Could not resolve {host}:{port}: {e}"))?
        .next()
        .ok_or_else(|| format!("No address for {host}:{port}"))?;

    let deadline = Instant::now() + timeout;
    let mut stream = TcpStream::connect_timeout(&addr, timeout)
        .map_err(|e| format!("Connect to {host}:{port} failed: {e}"))?;
    stream
        .set_read_timeout(Some(timeout))
        .map_err(|e| format!("Failed to set read timeout: {e}"))?;
    stream
        .set_write_timeout(Some(timeout))
        .map_err(|e| format!("Failed to set write timeout: {e}"))?;

    let req = format!("GET {path} HTTP/1.1\r\nHost: {host}:{port}\r\nConnection: close\r\n\r\n");
    stream
        .write_all(req.as_bytes())
        .map_err(|e| format!("Write failed: {e}"))?;

    let mut resp = Vec::new();
    let mut buf = [0u8; 4096];
    loop {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            return Err(format!("Timed out reading HTTP response from {path}"));
        }
        stream
            .set_read_timeout(Some(remaining))
            .map_err(|e| format!("Failed to update read timeout: {e}"))?;
        match stream.read(&mut buf) {
            Ok(0) => break,
            Ok(n) => {
                if resp.len().saturating_add(n) > MAX_HTTP_GET_BYTES {
                    return Err(format!(
                        "HTTP response from {path} exceeded {MAX_HTTP_GET_BYTES} byte limit"
                    ));
                }
                resp.extend_from_slice(&buf[..n]);
            }
            Err(e) => return Err(format!("Read failed: {e}")),
        }
    }

    let header_end = resp
        .windows(4)
        .position(|w| w == b"\r\n\r\n")
        .ok_or_else(|| "Malformed HTTP response (no header terminator)".to_string())?;

    let header_str = String::from_utf8_lossy(&resp[..header_end]);
    let status_line = header_str.lines().next().unwrap_or("");
    if !is_http_200_status_line(status_line) {
        return Err(format!("Server returned non-200 status:\n{header_str}"));
    }

    Ok(resp[header_end + 4..].to_vec())
}

/// Streams HTTP POST `/update` in 16KB chunks over a raw TCP stream.
///
/// Enforces a 64 KiB response cap, overall deadline, O(N) header detection,
/// socket timeout propagation, and strict HTTP status-line parsing (`M-132`).
///
/// # Errors
/// Returns an error on CRLF injection, socket/timeout error, response limit
/// overflow, or non-200 / unexpected HTTP response.
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
    stream
        .set_read_timeout(Some(timeout))
        .map_err(|e| format!("Failed to set read timeout: {e}"))?;
    stream
        .set_write_timeout(Some(timeout))
        .map_err(|e| format!("Failed to set write timeout: {e}"))?;

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
        "[OTA-UPLOAD] Sent {sent} bytes in {upload_elapsed:.2}s \
         ({rate_kb:.1} KB/s). Waiting for NOR flash write & HTTP response..."
    );

    let resp_deadline = Instant::now() + timeout;
    let mut resp_chunks = Vec::new();
    let mut buf = [0u8; 4096];
    let mut content_length: Option<usize> = None;
    let mut header_end_pos: Option<usize> = None;
    let mut scan_from: usize = 0;

    loop {
        let remaining = resp_deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            return Err("Timed out waiting for HTTP response from POST /update".to_string());
        }
        stream
            .set_read_timeout(Some(remaining))
            .map_err(|e| format!("Failed to update read timeout: {e}"))?;

        match stream.read(&mut buf) {
            Ok(0) => break, // EOF
            Ok(n) => {
                if resp_chunks.len().saturating_add(n) > MAX_OTA_RESPONSE_BYTES {
                    return Err(format!(
                        "HTTP response from POST /update exceeded \
                         {MAX_OTA_RESPONSE_BYTES} byte limit"
                    ));
                }
                resp_chunks.extend_from_slice(&buf[..n]);
                if header_end_pos.is_none() {
                    let window_start = scan_from.saturating_sub(3);
                    if let Some(rel) = resp_chunks[window_start..]
                        .windows(4)
                        .position(|w| w == b"\r\n\r\n")
                    {
                        let pos = window_start + rel;
                        header_end_pos = Some(pos);
                        let header_part = String::from_utf8_lossy(&resp_chunks[..pos]);
                        for line in header_part.lines() {
                            if let Some((name, val)) = line.split_once(':') {
                                if name.trim().eq_ignore_ascii_case("content-length") {
                                    content_length = val.trim().parse::<usize>().ok();
                                }
                            }
                        }
                    }
                    scan_from = resp_chunks.len();
                }
                if let (Some(pos), Some(cl)) = (header_end_pos, content_length) {
                    let body_len = resp_chunks.len().saturating_sub(pos + 4);
                    if body_len >= cl {
                        break;
                    }
                }
            }
            Err(e) => return Err(format!("Timed out or read failed from POST /update: {e}")),
        }
    }

    let raw_resp = String::from_utf8_lossy(&resp_chunks).to_string();
    let Some(pos) = header_end_pos else {
        return Err(format!(
            "Unexpected HTTP response from POST /update (missing headers):\n{raw_resp}"
        ));
    };

    let header_str = String::from_utf8_lossy(&resp_chunks[..pos]);
    let status_line = header_str.lines().next().unwrap_or("");
    let body_str = String::from_utf8_lossy(&resp_chunks[pos + 4..]);

    if !is_http_200_status_line(status_line)
        || !body_str.contains("Firmware update successful! Rebooting...")
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
/// Returns an error if argument parsing, staging, upload, or post-boot
/// verification fails.
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
                opts.token = Some(validate_ota_token(&args[i])?);
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
            "[OTA-ERROR] No camera host provided. Pass --host <camera-ip> or \
             set CAM_HOST environment variable."
                .to_string(),
        );
    }

    if !opts.stage_only && opts.token.is_none() {
        return Err(
            "[OTA-ERROR] No valid OTA authentication token provided. Pass \
             --token <SECRET> or set CAM_OTA_TOKEN environment variable \
             (DEFAULT_OTA_TOKEN is rejected)."
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
        "[OTA-PRE] Current active_partition='{}' (uptime_ms={}) -> \
         Target passive slot='{}'",
        pre_slot, pre_status.uptime_ms, target_slot
    );

    let token_secret = opts
        .token
        .as_ref()
        .ok_or_else(|| "[OTA-ERROR] Missing OTA token".to_string())?;
    let http_resp = stream_ota_post(
        &opts.host,
        opts.port,
        token_secret.expose(),
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
            "[OTA-ERROR] Timed out waiting for board to boot into \
             '{target_slot}' (last error: {last_err})"
        )
    })?;

    println!(
        "[OTA-POST] Verified boot into active_partition='{}' \
         (uptime_ms={}, framesize={}, wifi_rssi={} dBm)",
        post_status.active_partition,
        post_status.uptime_ms,
        post_status.framesize,
        post_status.wifi_rssi
    );

    if let Some(reported_sha) = post_status.app_sha256 {
        let reported = reported_sha.trim().to_ascii_lowercase();
        let expected_app_sha = Sha256::digest_hex(&payload[..payload.len() - ESP_IMAGE_DIGEST_LEN]);
        let full_bin_sha = Sha256::digest_hex(&payload);

        if reported != expected_app_sha && reported != full_bin_sha {
            return Err(format!(
                "[OTA-ERROR] Post-boot app_sha256 mismatch: device reported \
                 '{reported}', expected '{expected_app_sha}' (or '{full_bin_sha}')"
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
            "[OTA-ERROR] Post-boot /snapshot invalid JPEG \
             (len={}, soi={}, eoi={}, dims={:?}, expected={})",
            snap.len, snap.has_soi, snap.has_eoi, snap.dims, opts.expected_dims
        ));
    }

    println!(
        "[OTA-VERIFY] Post-boot GET /snapshot 200 OK ({} bytes, \
         SOI=0xFFD8, EOI=0xFFD9, dimensions={:?})",
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
    println!(
        "  --stage-only             Convert ELF to .bin and validate 0xE9 \
         header without touching network"
    );
    println!("  --expected-dims <DIMS>   Expected JPEG dimensions (default: 1280x720, or 'any')");
    println!("  -h, --help               Print help information");
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::net::TcpListener;
    use std::thread;

    fn build_valid_strict_image(entry_addr: u32, spi_mode: u8, hash_flag: u8) -> Vec<u8> {
        let mut img = vec![0u8; 24];
        img[0] = ESP_IMAGE_HEADER_MAGIC;
        img[1] = 1;
        img[2] = spi_mode;
        img[3] = 0x20;
        img[4..8].copy_from_slice(&entry_addr.to_le_bytes());
        img[23] = hash_flag;

        // Segment 0: load_addr = 0x4008_0400, len = 16
        img.extend_from_slice(&0x4008_0400u32.to_le_bytes());
        img.extend_from_slice(&16u32.to_le_bytes());
        let mut chk = 0xEFu8;
        for i in 0u8..16 {
            let b = i ^ 0x5A;
            chk ^= b;
            img.push(b);
        }
        let pad_len = 16 - (img.len() % 16);
        img.resize(img.len() + pad_len - 1, 0u8);
        img.push(chk);

        let digest = Sha256::digest(&img);
        img.extend_from_slice(&digest);
        img
    }

    #[test]
    fn test_validate_esp_image_strict_enforces_parity() {
        let valid = build_valid_strict_image(0x4008_0400, 2, 1);
        assert!(validate_esp_image_strict(&valid).is_ok());
        assert!(validate_esp_image(&valid).is_ok());

        // Mandatory SHA-256 flag (M-028)
        let no_hash = build_valid_strict_image(0x4008_0400, 2, 0);
        assert!(validate_esp_image_strict(&no_hash).is_err());
        assert!(validate_esp_image(&no_hash).is_err());

        // Invalid spi_mode > 5 (M-029)
        let bad_mode = build_valid_strict_image(0x4008_0400, 6, 1);
        assert!(validate_esp_image_strict(&bad_mode).is_err());
        assert!(validate_esp_image(&bad_mode).is_err());

        // Trailing bytes rejected (M-028/M-029)
        let mut trailing = valid.clone();
        trailing.push(0xAA);
        assert!(validate_esp_image_strict(&trailing).is_err());
        assert!(validate_esp_image(&trailing).is_err());
    }

    #[test]
    fn test_validate_esp_image_rejects_zero_entry_and_trailing_garbage() {
        // M-029 counterexample: entry_addr == 0 must be rejected unconditionally.
        let zero_entry_strict = build_valid_strict_image(0, 2, 1);
        let err = validate_esp_image(&zero_entry_strict).expect_err("entry_addr == 0 must fail");
        assert!(err.contains("entry address"), "got: {err}");

        // Legacy non-segment payload with entry_addr == 0 and valid SHA-256
        // must also fail.
        let mut legacy = vec![0u8; ESP_IMAGE_HEADER_LEN];
        legacy[0] = ESP_IMAGE_HEADER_MAGIC;
        legacy[1] = 1;
        legacy[23] = 1;
        legacy.extend_from_slice(b"ARBITRARY_TRAILING_GARBAGE_BYTES");
        let legacy_digest = Sha256::digest(&legacy);
        legacy.extend_from_slice(&legacy_digest);
        assert!(validate_esp_image(&legacy).is_err());

        // Trailing garbage inserted before a recomputed SHA-256 digest on a valid
        // IRAM entry image must be rejected by exact aligned_end + 32 == len.
        let valid = build_valid_strict_image(0x4008_0400, 2, 1);
        let aligned_body_len = valid.len() - ESP_IMAGE_DIGEST_LEN;
        let mut with_garbage = valid[..aligned_body_len].to_vec();
        with_garbage.extend_from_slice(b"EXTRA_GARBAGE");
        let recomputed_digest = Sha256::digest(&with_garbage);
        with_garbage.extend_from_slice(&recomputed_digest);
        let garbage_err = validate_esp_image(&with_garbage)
            .expect_err("trailing garbage before digest must fail");
        assert!(
            garbage_err.contains("length mismatch"),
            "got: {garbage_err}"
        );

        // Unaligned segment load_addr (0x4008_0401) must fail even with valid SHA-256.
        let mut unaligned = valid[..aligned_body_len].to_vec();
        unaligned[24..28].copy_from_slice(&0x4008_0401u32.to_le_bytes());
        let unaligned_digest = Sha256::digest(&unaligned);
        unaligned.extend_from_slice(&unaligned_digest);
        let unaligned_err =
            validate_esp_image(&unaligned).expect_err("unaligned load_addr must fail");
        assert!(
            unaligned_err.contains("Unaligned segment"),
            "got: {unaligned_err}"
        );

        // Corrupted XOR checksum must fail even with recomputed SHA-256.
        let mut bad_xor = valid[..aligned_body_len].to_vec();
        let last = bad_xor.len() - 1;
        bad_xor[last] ^= 0xFF;
        let bad_xor_digest = Sha256::digest(&bad_xor);
        bad_xor.extend_from_slice(&bad_xor_digest);
        let xor_err = validate_esp_image(&bad_xor).expect_err("corrupted XOR must fail");
        assert!(xor_err.contains("XOR checksum"), "got: {xor_err}");
    }

    #[test]
    fn test_secret_string_redacted_and_env_precedence() {
        let secret = SecretString::new("super-secret-value-12345".to_string());
        assert_eq!(format!("{secret:?}"), "[REDACTED]");
        assert_eq!(format!("{secret}"), "[REDACTED]");

        assert!(validate_ota_token(DEFAULT_OTA_TOKEN).is_err());

        let env_doc = "\
            # comment\n\
            WIFI_OTA_TOKEN=low-priority-token # inline comment\n\
            export CAM_OTA_TOKEN=\"high-priority-token\" # trailing\n\
            OTA_TOKEN='mid-priority'\n";
        let resolved = resolve_token_from_env_content(env_doc).expect("resolved");
        assert_eq!(resolved.expose(), "high-priority-token");

        // Reject default token even when highest precedence, falling back.
        let env_default = "\
            CAM_OTA_TOKEN=klipper-cam-ota-2026\n\
            OTA_TOKEN=real-custom-token-value\n";
        let fallback = resolve_token_from_env_content(env_default).expect("fallback");
        assert_eq!(fallback.expose(), "real-custom-token-value");
    }

    #[test]
    fn test_parse_status_json_fails_closed() {
        assert!(parse_status_json("").is_err());
        assert!(parse_status_json("{}").is_err());
        assert!(parse_status_json(r#"{"active_partition": "ota_0"}"#).is_err());
        assert!(parse_status_json(r#"{"active_partition": "ota_0", "uptime_ms": "bad"}"#).is_err());
        assert!(parse_status_json(
            r#"{"active_partition": "ota_0", "uptime_ms": 10, "wifi_rssi": "x"}"#
        )
        .is_err());
        // Key inside a string value must not satisfy required fields.
        let nested_str = r#"{"note": "\"uptime_ms\": 10, \"active_partition\": \"ota_0\""}"#;
        assert!(parse_status_json(nested_str).is_err());
    }

    #[test]
    fn test_parse_jpeg_dimensions_rejects_truncated_sof() {
        // SOF0 with seg_len = 5 (< 7) followed by trailing bytes must return None.
        let truncated = [
            0xFF, 0xD8, 0xFF, 0xC0, 0x00, 0x05, 0x08, 0x02, 0xD0, 0x05, 0x00, 0xFF, 0xD9,
        ];
        assert_eq!(parse_jpeg_dimensions(&truncated), None);
    }

    fn drain_mock_request(s: &mut TcpStream, body_len: usize) {
        let _ = s.set_read_timeout(Some(Duration::from_secs(2)));
        let mut received = Vec::new();
        let mut chunk = [0u8; 512];
        loop {
            if let Some(pos) = received.windows(4).position(|w| w == b"\r\n\r\n") {
                if received.len() >= pos + 4 + body_len {
                    return;
                }
            }
            match s.read(&mut chunk) {
                Ok(0) | Err(_) => return,
                Ok(n) => received.extend_from_slice(&chunk[..n]),
            }
        }
    }

    #[test]
    fn test_stream_ota_post_rejects_spoofed_status_and_oversized_resp() {
        let listener = TcpListener::bind("127.0.0.1:0").expect("bind");
        let port = listener.local_addr().expect("addr").port();
        let handle = thread::spawn(move || {
            if let Ok((mut s, _)) = listener.accept() {
                drain_mock_request(&mut s, 16);
                let resp = "HTTP/1.1 500 Error\r\nContent-Length: 50\r\n\r\n\
                            200 OK Firmware update successful! Rebooting...";
                let _ = s.write_all(resp.as_bytes());
                let _ = s.flush();
            }
        });
        let err = stream_ota_post("127.0.0.1", port, "tok", &[1u8; 16], Duration::from_secs(2))
            .expect_err("must reject 500 with 200 OK in body");
        let _ = handle.join();
        assert!(err.contains("Unexpected HTTP response"), "got: {err}");

        // Oversized response > 64 KiB must fail closed.
        let listener2 = TcpListener::bind("127.0.0.1:0").expect("bind2");
        let port2 = listener2.local_addr().expect("addr2").port();
        let handle2 = thread::spawn(move || {
            if let Ok((mut s, _)) = listener2.accept() {
                drain_mock_request(&mut s, 16);
                let big = vec![b'A'; MAX_OTA_RESPONSE_BYTES + 8192];
                let _ = s.write_all(&big);
                let _ = s.flush();
            }
        });
        let err2 = stream_ota_post(
            "127.0.0.1",
            port2,
            "tok",
            &[1u8; 16],
            Duration::from_secs(2),
        )
        .expect_err("must reject oversized response");
        let _ = handle2.join();
        assert!(err2.contains("exceeded"), "got: {err2}");
    }
}
