//! Host CLI and Script Test Parity Suite.
//!
//! Ports 100% of unit tests from `tests/test_scripts.py` (`TestOtaDeploy`,
//! `TestEnsureSerialPassive`, `TestVerifyColdBootInvariants`) plus
//! `wrap-esp-image-elf` and `verify-pure-rust` fixture tests into pure Rust.

use std::fs;
use std::io::{Read, Write};
use std::net::TcpListener;
use std::path::{Path, PathBuf};
use std::thread;
use std::time::Duration;

use xtask::elf_wrap::{
    pack_esp_bootloader_bin, wrap_esp_image_to_elf, ELFCLASS32, ELFDATA2LSB, EM_XTENSA,
    ESP_IMAGE_HEADER_MAGIC, ET_EXEC, EV_CURRENT, PT_LOAD,
};
use xtask::ota_deploy::{
    expected_next_slot, parse_jpeg_dimensions, parse_status_json, stream_ota_post,
    validate_esp_image,
};
use xtask::pure_rust_audit::audit_pure_rust;
use xtask::serial_passive::{
    ensure_passive, parse_camera_status, SerialPassiveOptions, DEFAULT_SERIAL_PORT,
};
use xtask::sha256::Sha256;
use xtask::verify_invariants::{
    strip_rust_comments, verify_bootloader_bin, verify_bootloader_sources, verify_mmio_registers,
    AUTHORITATIVE_MMIO_REGISTERS,
};

/// RAII helper for managing unique temporary directories with guaranteed cleanup.
struct TestTempDir {
    path: PathBuf,
}

impl TestTempDir {
    fn new(prefix: &str) -> Self {
        let unique = format!(
            "esp32_cam_test_{prefix}_{}_{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos()
        );
        let path = std::env::temp_dir().join(unique);
        fs::create_dir_all(&path).expect("failed to create temp test directory");
        Self { path }
    }

    fn path(&self) -> &Path {
        &self.path
    }
}

impl Drop for TestTempDir {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.path);
    }
}

// ============================================================================
// 1. Tests ported from TestOtaDeploy
// ============================================================================

#[test]
fn test_expected_next_slot_valid() {
    assert_eq!(expected_next_slot("ota_0").expect("valid slot"), "ota_1");
    assert_eq!(expected_next_slot("ota_1").expect("valid slot"), "ota_0");
}

#[test]
fn test_expected_next_slot_invalid() {
    let invalid_slots = ["", "factory", "ota_2", "invalid_slot", "OTA_0"];
    for slot in invalid_slots {
        let res = expected_next_slot(slot);
        assert!(res.is_err(), "Slot '{slot}' should be rejected");
        let err_msg = res.expect_err("expected error");
        assert!(
            err_msg.contains("Unknown active_partition label"),
            "Expected 'Unknown active_partition label' in error, got: {err_msg}"
        );
    }
}

#[test]
fn test_missing_host_exit_error() {
    let res = xtask::ota_deploy::run(&[
        "--port".to_string(),
        "80".to_string(),
        "--host".to_string(),
        "".to_string(),
    ]);
    assert!(res.is_err(), "Missing host must return Err");
    let err = res.expect_err("expected error");
    assert!(
        err.contains("No camera host provided"),
        "Expected missing host message, got: {err}"
    );
}

#[test]
fn test_read_http_response_content_length() {
    let listener = TcpListener::bind("127.0.0.1:0").expect("bind ephemeral port");
    let port = listener.local_addr().expect("local addr").port();

    let server_thread = thread::spawn(move || {
        if let Ok((mut stream, _)) = listener.accept() {
            let mut req_buf = [0u8; 1024];
            let _ = stream.read(&mut req_buf);
            let body = "Firmware update successful! Rebooting...\n";
            let response = format!(
                "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{}",
                body.len(),
                body
            );
            let _ = stream.write_all(response.as_bytes());
        }
    });

    let dummy_payload = vec![0u8; 64];
    let resp = stream_ota_post(
        "127.0.0.1",
        port,
        "test_token",
        &dummy_payload,
        Duration::from_secs(3),
    );
    let _ = server_thread.join();

    assert!(resp.is_ok(), "Expected stream_ota_post success: {resp:?}");
    let resp_text = resp.expect("valid response");
    assert!(resp_text.contains("200 OK"));
    assert!(resp_text.contains("Firmware update successful! Rebooting..."));
}

#[test]
fn test_read_http_response_invalid_raises_error() {
    let listener = TcpListener::bind("127.0.0.1:0").expect("bind ephemeral port");
    let port = listener.local_addr().expect("local addr").port();

    let server_thread = thread::spawn(move || {
        if let Ok((mut stream, _)) = listener.accept() {
            let mut req_buf = [0u8; 1024];
            let _ = stream.read(&mut req_buf);
            let response = "HTTP/1.1 500 Internal Server Error\r\nContent-Length: 0\r\n\r\n";
            let _ = stream.write_all(response.as_bytes());
        }
    });

    let dummy_payload = vec![0u8; 64];
    let resp = stream_ota_post(
        "127.0.0.1",
        port,
        "test_token",
        &dummy_payload,
        Duration::from_secs(3),
    );
    let _ = server_thread.join();

    assert!(resp.is_err(), "Expected error on 500 response");
    let err_msg = resp.expect_err("expected error");
    assert!(
        err_msg.contains("Unexpected HTTP response"),
        "Expected 'Unexpected HTTP response', got: {err_msg}"
    );
}

#[test]
fn test_app_sha256_check() {
    let mut header = [0u8; 24];
    header[0] = 0xE9; // Magic
    header[1] = 1; // Segment count
    header[23] = 1; // hash_appended = true

    let mut payload_body = Vec::new();
    payload_body.extend_from_slice(&header);
    payload_body.extend_from_slice(b"ESP32_FIRMWARE_PAYLOAD_TEST_DATA");

    let valid_digest = Sha256::digest(&payload_body);

    // Case A: Corrupt appended SHA-256
    let mut corrupt_payload = payload_body.clone();
    corrupt_payload.extend_from_slice(&[0u8; 32]);
    let corrupt_res = validate_esp_image(&corrupt_payload);
    assert!(corrupt_res.is_err());
    let err_msg = corrupt_res.expect_err("expected error");
    assert!(
        err_msg.contains("Appended SHA-256 mismatch"),
        "expected SHA-256 mismatch error, got: {err_msg}"
    );

    // Case B: Valid appended SHA-256
    let mut valid_payload = payload_body;
    valid_payload.extend_from_slice(&valid_digest);
    let valid_res = validate_esp_image(&valid_payload);
    assert!(valid_res.is_ok(), "Valid image failed: {valid_res:?}");
    let parsed_header = valid_res.expect("valid header");
    assert_eq!(parsed_header.magic, 0xE9);
    assert_eq!(parsed_header.segment_count, 1);
    assert!(parsed_header.hash_appended);
}

#[test]
fn test_jpeg_dimensions_parser() {
    assert_eq!(parse_jpeg_dimensions(&[]), None);
    assert_eq!(parse_jpeg_dimensions(&[0xFF, 0xD8]), None);

    // Build synthetic JPEG with SOF0 marker (0xFF, 0xC0)
    let mut jpeg = vec![0xFF, 0xD8]; // SOI
    jpeg.extend_from_slice(&[0xFF, 0xC0]); // SOF0
    jpeg.extend_from_slice(&11u16.to_be_bytes()); // segment length
    jpeg.push(8); // precision
    jpeg.extend_from_slice(&720u16.to_be_bytes()); // height
    jpeg.extend_from_slice(&1280u16.to_be_bytes()); // width
    jpeg.push(3); // components
    jpeg.extend_from_slice(&[0u8; 2]); // pad
    jpeg.extend_from_slice(&[0xFF, 0xD9]); // EOI

    let dims = parse_jpeg_dimensions(&jpeg);
    assert_eq!(dims, Some((1280, 720)));
}

#[test]
fn test_parse_status_json() {
    let json = r#"{"active_partition": "ota_0", "uptime_ms": 12345, "framesize": "1280x720", "wifi_rssi": -55, "app_sha256": "abcdef123456"}"#;
    let status = parse_status_json(json).expect("parse status");
    assert_eq!(status.active_partition, "ota_0");
    assert_eq!(status.uptime_ms, 12345);
    assert_eq!(status.framesize, "1280x720");
    assert_eq!(status.wifi_rssi, -55);
    assert_eq!(status.app_sha256.as_deref(), Some("abcdef123456"));
}

// ============================================================================
// 2. Tests ported from TestEnsureSerialPassive
// ============================================================================

#[test]
fn test_cli_and_env_defaults() {
    assert_eq!(DEFAULT_SERIAL_PORT, "/dev/ttyUSB-esp32");

    let opts = SerialPassiveOptions::default();
    assert!(!opts.port.is_empty());
}

#[test]
fn test_serial_status_json_uptime_monotonicity() {
    let json_pre = r#"{"uptime_ms": 10500, "active_partition": "ota_0"}"#;
    let st_pre = parse_camera_status(json_pre).expect("parse pre");
    assert_eq!(st_pre.uptime_ms, 10500);
    assert_eq!(st_pre.active_partition, "ota_0");

    let json_post_monotonic = r#"{"uptime_ms": 11200, "active_partition": "ota_0"}"#;
    let st_post_ok = parse_camera_status(json_post_monotonic).expect("parse post");
    assert!(st_post_ok.uptime_ms >= st_pre.uptime_ms);

    let json_post_reset = r#"{"uptime_ms": 500, "active_partition": "ota_0"}"#;
    let st_post_reset = parse_camera_status(json_post_reset).expect("parse reset");
    assert!(
        st_post_reset.uptime_ms < st_pre.uptime_ms,
        "Uptime reset detected"
    );
}

#[test]
fn test_serial_port_nonexistent_returns_err() {
    let opts = SerialPassiveOptions {
        port: "/dev/nonexistent_serial_port_999".to_string(),
        host: String::new(),
        status_url: String::new(),
        check_http: false,
        timeout: Duration::from_millis(100),
    };
    let res = ensure_passive(&opts);
    assert!(res.is_err());
}

// ============================================================================
// 3. Tests ported from TestVerifyColdBootInvariants
// ============================================================================

#[test]
fn test_missing_bootloader_without_flag() {
    let tmp = TestTempDir::new("missing_bl");
    let non_existent = tmp.path().join("missing_bootloader.bin");
    let (errors, size) = verify_bootloader_bin(&non_existent, false);
    assert!(!errors.is_empty());
    assert!(size.is_none());
    assert!(errors[0].contains("Missing patched bootloader binary"));
}

#[test]
fn test_missing_bootloader_with_allow_missing() {
    let tmp = TestTempDir::new("missing_bl_allow");
    let non_existent = tmp.path().join("missing_bootloader.bin");
    let (errors, size) = verify_bootloader_bin(&non_existent, true);
    assert!(errors.is_empty());
    assert!(size.is_none());
}

#[test]
fn test_corrupt_bootloader_fails_verification() {
    let tmp = TestTempDir::new("corrupt_bl");
    let corrupt_bin = tmp.path().join("corrupt_bootloader.bin");

    // Case A: Too small or bad magic byte
    fs::write(&corrupt_bin, [0u8; 1024]).expect("write");
    let (errors_a, _) = verify_bootloader_bin(&corrupt_bin, false);
    assert!(errors_a
        .iter()
        .any(|e| e.contains("Invalid ESP32 bootloader header or size")));

    // Case B: Correct magic and size, but missing MMIO register and 0x2B00 sequence
    let mut valid_header_empty_body = vec![0u8; 16384];
    valid_header_empty_body[0] = 0xE9;
    fs::write(&corrupt_bin, &valid_header_empty_body).expect("write");
    let (errors_b, _) = verify_bootloader_bin(&corrupt_bin, false);
    assert!(errors_b
        .iter()
        .any(|e| e.contains("missing IO_MUX_GPIO16_REG")));
    assert!(errors_b
        .iter()
        .any(|e| e.contains("missing GPIO 16 pull-up constant 0x2B00")));

    // Case C: Oversized image (> 28,672 bytes / 0x7000) fails size check
    let mut oversized = vec![0u8; 28_673];
    oversized[0] = 0xE9;
    oversized[32..36].copy_from_slice(&0x3FF4_904Cu32.to_le_bytes());
    oversized[36..40].copy_from_slice(&0x0000_2B00u32.to_le_bytes());
    oversized[40..44].copy_from_slice(&0x3FF4_80C8u32.to_le_bytes());
    oversized[44..48].copy_from_slice(&0x3FF4_8474u32.to_le_bytes());
    oversized[48..52].copy_from_slice(&0x3FF4_4008u32.to_le_bytes());
    oversized[52..56].copy_from_slice(&0x3FF4_4570u32.to_le_bytes());
    oversized[56..60].copy_from_slice(&0x3FF4_4024u32.to_le_bytes());
    fs::write(&corrupt_bin, &oversized).expect("write oversized");
    let (errors_c, _) = verify_bootloader_bin(&corrupt_bin, false);
    assert!(
        errors_c
            .iter()
            .any(|e| e.contains("Invalid ESP32 bootloader header or size")),
        "Oversized bootloader must fail size check, got: {errors_c:?}"
    );
}

#[test]
fn test_verify_bootloader_bin_accepts_synthetic_and_built_images() {
    let tmp = TestTempDir::new("valid_bl");
    let synthetic_bin = tmp.path().join("synthetic_bootloader.bin");

    let mut image = vec![0u8; 2_048];
    image[0] = 0xE9;
    let words: [u32; 7] = [
        0x3FF4_904C, // IO_MUX_GPIO16_REG
        0x0000_2B00, // GPIO16 pull-up config
        0x3FF4_80C8, // RTC_CNTL_HOLD_FORCE_REG
        0x3FF4_8474, // RTC_IO_DIG_PAD_HOLD_REG
        0x3FF4_4008, // GPIO_OUT_W1TS_REG
        0x3FF4_4570, // GPIO_FUNC16_OUT_SEL_CFG_REG
        0x3FF4_4024, // GPIO_ENABLE_W1TS_REG
    ];
    for (idx, word) in words.iter().enumerate() {
        let start = 32 + idx * 4;
        image[start..start + 4].copy_from_slice(&word.to_le_bytes());
    }
    fs::write(&synthetic_bin, &image).expect("write synthetic bootloader");

    let (syn_errors, syn_size) = verify_bootloader_bin(&synthetic_bin, false);
    assert!(
        syn_errors.is_empty(),
        "Synthetic 2048-byte bootloader should pass, got: {syn_errors:?}"
    );
    assert_eq!(syn_size, Some(2_048));

    let built_bin = xtask::ota_deploy::repo_root()
        .join("target")
        .join("xtensa-esp32-none-elf")
        .join("release")
        .join("esp32-cam-bootloader.bin");
    if built_bin.is_file() {
        let (built_errors, built_size) = verify_bootloader_bin(&built_bin, false);
        assert!(
            built_errors.is_empty(),
            "Built pure-Rust bootloader {} failed verification: {built_errors:?}",
            built_bin.display()
        );
        let len = built_size.expect("built bootloader size");
        assert!(
            (2_048..=28_672).contains(&len),
            "Built bootloader size {len} outside 2048..=28672"
        );
    }
}

#[test]
fn test_verify_bootloader_sources_handles_zero_reg_and_catches_violations() {
    let repo_errs = verify_bootloader_sources(&xtask::ota_deploy::repo_root());
    assert!(
        repo_errs.is_empty(),
        "Real bootloader sources failed verification: {repo_errs:?}"
    );

    let tmp = TestTempDir::new("bl_sources");
    let src_dir = tmp
        .path()
        .join("crates")
        .join("esp32-cam-bootloader")
        .join("src");
    fs::create_dir_all(&src_dir).expect("mkdir src");
    fs::write(
        src_dir.join("partitions.rs"),
        b"pub const MAGIC: u16 = 0x50AA;\n",
    )
    .expect("write partitions.rs");
    fs::write(
        src_dir.join("gpio16.rs"),
        b"pub const IO_MUX_GPIO16_REG: u32 = 0x3FF4_904C;\n\
          pub const CFG: u32 = 0x0000_2B00;\n\
          fn init() { write_u32(IO_MUX_GPIO16_REG, CFG); }\n",
    )
    .expect("write gpio16.rs");
    fs::write(
        src_dir.join("wdt.rs"),
        b"pub const RTC_CNTL_WDTCONFIG0_REG: u32 = 0x3FF4_808C;\n\
          fn arm() { write_u32(RTC_CNTL_WDTCONFIG0_REG, 1); }\n",
    )
    .expect("write wdt.rs");

    let valid_errs = verify_bootloader_sources(tmp.path());
    assert!(
        valid_errs.is_empty(),
        "Valid bootloader sources with zero-REG module failed: {valid_errs:?}"
    );

    fs::write(
        src_dir.join("bad.rs"),
        b"pub const FAKE_BOOTLOADER_REG: u32 = 0x3FF9_9999;\n\
          pub const IO_MUX_GPIO7_REG: u32 = 0x3FF4_9060;\n\
          fn bad() { write_u32(GPIO_ENABLE_W1TS_REG, 1 << 4); }\n",
    )
    .expect("write bad.rs");
    let bad_errs = verify_bootloader_sources(tmp.path());
    assert!(bad_errs.iter().any(|e| e.contains("FAKE_BOOTLOADER_REG")));
    assert!(bad_errs.iter().any(|e| e.contains("IO_MUX_GPIO7_REG")));
    assert!(bad_errs.iter().any(|e| e.contains("GPIO 4")));
}

#[test]
fn test_full_script_allow_missing_bootloader_main() {
    let repo_root_str = xtask::ota_deploy::repo_root().to_string_lossy().to_string();
    let res = xtask::verify_invariants::run(&[
        "--allow-missing-bootloader".to_string(),
        "--repo-root".to_string(),
        repo_root_str,
    ]);
    assert_eq!(res, Ok(()));
}

#[test]
fn test_strip_rust_comments() {
    let src = r##"
        // Line comment
        const FOO: u32 = 42; /* Block comment */
        let str_val = "// this is a string literal /* not a comment */";
        let raw_str = r#"/* raw string */ // not comment"#;
    "##;
    let stripped = strip_rust_comments(src);
    assert!(!stripped.contains("Line comment"));
    assert!(!stripped.contains("Block comment"));
    assert!(stripped.contains("FOO"));
    assert!(stripped.contains(r#"// this is a string literal /* not a comment */"#));
    assert!(stripped.contains(r#"/* raw string */ // not comment"#));
}

#[test]
fn test_verify_mmio_registers_valid_and_invalid() {
    let valid_src = r#"
        pub const IO_MUX_GPIO16_REG: u32 = 0x3FF4_904C;
        pub const RTC_CNTL_STORE6_REG: u32 = 0x3FF4_80B8;
        pub const DPORT_PRO_FLASH_MMU_TABLE: u32 = 0x3FF1_0000;
    "#;
    let errors = verify_mmio_registers(valid_src);
    assert!(
        errors.is_empty(),
        "Expected no errors for valid MMIO, got: {errors:?}"
    );

    let invalid_unknown = r#"
        pub const UNKNOWN_FAKE_REG: u32 = 0x3FF9_9999;
    "#;
    let errors_unknown = verify_mmio_registers(invalid_unknown);
    assert!(errors_unknown
        .iter()
        .any(|e| e.contains("missing from AUTHORITATIVE_MMIO_REGISTERS")));

    let invalid_mismatch = r#"
        pub const IO_MUX_GPIO16_REG: u32 = 0x3FF4_0000;
    "#;
    let errors_mismatch = verify_mmio_registers(invalid_mismatch);
    assert!(errors_mismatch
        .iter()
        .any(|e| e.contains("MMIO address mismatch")));
}

#[test]
fn test_authoritative_mmio_registers_table_integrity() {
    assert!(
        !AUTHORITATIVE_MMIO_REGISTERS.is_empty(),
        "MMIO register table must not be empty"
    );
    for &(name, addr) in AUTHORITATIVE_MMIO_REGISTERS {
        assert!(
            name.ends_with("_REG") || name.ends_with("_MMU_TABLE"),
            "Invalid MMIO name format: {name}"
        );
        assert!(addr >= 0x3FF0_0000, "MMIO address {addr:#X} out of range");
    }
}

// ============================================================================
// 4. wrap-esp-image-elf Bootloader Round-Trip Test
// ============================================================================

fn build_synthetic_esp_image(entry_addr: u32, segments: &[(u32, &[u8])]) -> Vec<u8> {
    let mut image = vec![0u8; 24];
    image[0] = ESP_IMAGE_HEADER_MAGIC;
    image[1] = u8::try_from(segments.len()).expect("segment count fits in u8");
    image[2] = 0x02; // DIO
    image[3] = 0x20; // 40MHz / 4MB
    image[4..8].copy_from_slice(&entry_addr.to_le_bytes());
    image[8] = 0xEE; // WP pin disabled
    image[17..19].copy_from_slice(&399u16.to_le_bytes());
    image[23] = 1; // hash_appended = true

    let mut xor_chk = 0xEFu8;
    for &(load_addr, data) in segments {
        let len_u32 = u32::try_from(data.len()).expect("segment length fits in u32");
        image.extend_from_slice(&load_addr.to_le_bytes());
        image.extend_from_slice(&len_u32.to_le_bytes());
        for &b in data {
            xor_chk ^= b;
        }
        image.extend_from_slice(data);
    }

    let pad_len = 16 - (image.len() % 16);
    image.resize(image.len() + pad_len - 1, 0u8);
    image.push(xor_chk);

    let digest = Sha256::digest(&image);
    image.extend_from_slice(&digest);
    image
}

fn assert_wrapped_bootloader_elf(elf_bytes: &[u8]) -> (u32, Vec<u32>) {
    // 1. Verify ELF32 Little Endian header
    assert!(elf_bytes.len() >= 52, "ELF too small");
    assert_eq!(&elf_bytes[..4], b"\x7fELF", "Magic mismatch");
    assert_eq!(elf_bytes[4], ELFCLASS32, "Expected ELFCLASS32 (1)");
    assert_eq!(elf_bytes[5], ELFDATA2LSB, "Expected ELFDATA2LSB (1)");
    assert_eq!(elf_bytes[6], EV_CURRENT, "Expected EV_CURRENT (1)");

    let e_type = u16::from_le_bytes([elf_bytes[16], elf_bytes[17]]);
    assert_eq!(e_type, ET_EXEC, "Expected ET_EXEC (2)");

    let e_machine = u16::from_le_bytes([elf_bytes[18], elf_bytes[19]]);
    assert_eq!(e_machine, EM_XTENSA);
    assert_eq!(e_machine, 94);

    let e_entry = u32::from_le_bytes([elf_bytes[24], elf_bytes[25], elf_bytes[26], elf_bytes[27]]);

    // 2. Program headers (PT_LOAD)
    let e_phoff =
        u32::from_le_bytes([elf_bytes[28], elf_bytes[29], elf_bytes[30], elf_bytes[31]]) as usize;
    let e_phentsize = u16::from_le_bytes([elf_bytes[42], elf_bytes[43]]) as usize;
    let e_phnum = u16::from_le_bytes([elf_bytes[44], elf_bytes[45]]) as usize;
    assert!(e_phnum >= 1, "Expected at least 1 PT_LOAD program header");
    assert_eq!(e_phentsize, 32);

    let mut load_addrs = Vec::with_capacity(e_phnum);
    for i in 0..e_phnum {
        let ph_start = e_phoff + i * e_phentsize;
        let p_type = u32::from_le_bytes([
            elf_bytes[ph_start],
            elf_bytes[ph_start + 1],
            elf_bytes[ph_start + 2],
            elf_bytes[ph_start + 3],
        ]);
        assert_eq!(p_type, PT_LOAD, "Program header {i} must be PT_LOAD");

        let p_vaddr = u32::from_le_bytes([
            elf_bytes[ph_start + 8],
            elf_bytes[ph_start + 9],
            elf_bytes[ph_start + 10],
            elf_bytes[ph_start + 11],
        ]);
        load_addrs.push(p_vaddr);
    }

    // 3. Synthetic MMIO symbol presence
    assert!(
        elf_bytes
            .windows(b"IO_MUX_GPIO16_REG".len())
            .any(|w| w == b"IO_MUX_GPIO16_REG"),
        "IO_MUX_GPIO16_REG symbol must be present in ELF symbol/string table"
    );

    (e_entry, load_addrs)
}

#[test]
fn test_wrap_esp_image_elf_bootloader_roundtrip() {
    // 1. Unconditional synthetic ESP32 image wrap-to-ELF test (always runs on CI).
    let syn_segments: &[(u32, &[u8])] = &[
        (
            0x3FFF_0030,
            &[0x11, 0x22, 0x33, 0x44, 0xAA, 0xBB, 0xCC, 0xDD],
        ),
        (
            0x4007_8000,
            &[
                0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B,
            ],
        ),
        (
            0x4008_0400,
            &[0x36, 0x41, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00],
        ),
    ];
    let syn_bin = build_synthetic_esp_image(0x4008_065C, syn_segments);
    let syn_elf = wrap_esp_image_to_elf(&syn_bin, None).expect("wrap synthetic bin to ELF");
    let (syn_entry, syn_vaddrs) = assert_wrapped_bootloader_elf(&syn_elf);
    assert_eq!(syn_entry, 0x4008_065C);
    assert_eq!(syn_vaddrs, vec![0x3FFF_0030, 0x4007_8000, 0x4008_0400]);

    let syn_repacked =
        pack_esp_bootloader_bin(&syn_elf).expect("repack synthetic ELF back to ESP32 bin");
    assert_eq!(syn_repacked, syn_bin);

    // 2. Real pure-Rust bootloader round-trip when built artifact is present
    let built_bin = xtask::ota_deploy::repo_root()
        .join("target")
        .join("xtensa-esp32-none-elf")
        .join("release")
        .join("esp32-cam-bootloader.bin");
    if built_bin.is_file() {
        let bin_bytes = fs::read(&built_bin).expect("read built esp32-cam-bootloader.bin");
        let tmp = TestTempDir::new("elf_wrap_test");
        let out_elf = tmp.path().join("bootloader_wrapped.elf");
        let elf_bytes = wrap_esp_image_to_elf(&bin_bytes, None).expect("wrap built bin to ELF");
        fs::write(&out_elf, &elf_bytes).expect("write temp ELF");

        let (entry, load_addrs) = assert_wrapped_bootloader_elf(&elf_bytes);
        assert_ne!(entry, 0, "Bootloader e_entry must be non-zero");
        assert!(
            !load_addrs.is_empty(),
            "Bootloader must contain at least 1 PT_LOAD segment"
        );

        let repacked =
            pack_esp_bootloader_bin(&elf_bytes).expect("repack built bootloader ELF to bin");
        assert_eq!(
            repacked.len(),
            bin_bytes.len(),
            "Repacked bootloader byte length mismatch"
        );
        assert_eq!(
            repacked, bin_bytes,
            "Repacked bootloader must match esp32-cam-bootloader.bin byte-for-byte"
        );
    }
}

// ============================================================================
// 5. verify-pure-rust Synthetic Fixture Tests
// ============================================================================

#[test]
fn test_verify_pure_rust_synthetic_fixtures() {
    // Case 1: Directory with disallowed .py and .sh files fails
    let dirty_tmp = TestTempDir::new("pure_rust_dirty");
    fs::write(dirty_tmp.path().join("deploy.py"), b"# python script").expect("write py");
    fs::write(dirty_tmp.path().join("build.sh"), b"#!/bin/sh\necho hi").expect("write sh");
    fs::create_dir_all(dirty_tmp.path().join("src")).expect("mkdir src");
    fs::write(
        dirty_tmp.path().join("src").join("main.rs"),
        b"fn main() {}",
    )
    .expect("write rs");

    let dirty_report = audit_pure_rust(dirty_tmp.path(), true).expect("audit");
    assert!(!dirty_report.is_clean(), "Dirty tree must not be clean");
    assert_eq!(dirty_report.offending_files.len(), 2);
    let offending_names: Vec<String> = dirty_report
        .offending_files
        .iter()
        .map(|p| {
            p.file_name()
                .expect("file name")
                .to_string_lossy()
                .to_string()
        })
        .collect();
    assert!(offending_names.contains(&"deploy.py".to_string()));
    assert!(offending_names.contains(&"build.sh".to_string()));

    // Case 2: Clean directory with only Rust source passes
    let clean_tmp = TestTempDir::new("pure_rust_clean");
    fs::create_dir_all(clean_tmp.path().join("src")).expect("mkdir src");
    fs::write(
        clean_tmp.path().join("src").join("lib.rs"),
        b"pub fn helper() {}",
    )
    .expect("write lib.rs");
    fs::write(
        clean_tmp.path().join("Cargo.toml"),
        b"[package]\nname = \"clean\"\nversion = \"0.1.0\"\n",
    )
    .expect("write toml");

    let clean_report = audit_pure_rust(clean_tmp.path(), true).expect("audit");
    assert!(clean_report.is_clean(), "Clean tree must pass audit");
    assert!(clean_report.offending_files.is_empty());

    // Case 3: Cargo.lock with esp-radio or esp-rtos rejected across all phases
    let legacy_wifi_tmp = TestTempDir::new("pure_rust_legacy_wifi");
    fs::create_dir_all(legacy_wifi_tmp.path().join("src")).expect("mkdir src");
    fs::write(
        legacy_wifi_tmp.path().join("src").join("lib.rs"),
        b"pub fn ok() {}",
    )
    .expect("write lib.rs");
    fs::write(
        legacy_wifi_tmp.path().join("Cargo.lock"),
        b"[[package]]\nname = \"esp-radio\"\n\n[[package]]\nname = \"esp-rtos\"\n",
    )
    .expect("write lock");

    let wifi_report = audit_pure_rust(legacy_wifi_tmp.path(), true).expect("audit");
    assert!(
        !wifi_report.is_clean(),
        "Trees with esp-radio / esp-rtos must be rejected in phase 1"
    );
    assert!(wifi_report
        .disallowed_dependencies
        .contains(&"esp-radio".to_string()));
    assert!(wifi_report
        .disallowed_dependencies
        .contains(&"esp-rtos".to_string()));

    // Case 4: esp-wifi-sys allowed in phase 1, rejected post-phase 1
    let seam_tmp = TestTempDir::new("pure_rust_seam");
    fs::create_dir_all(seam_tmp.path().join("src")).expect("mkdir src");
    fs::write(
        seam_tmp.path().join("src").join("lib.rs"),
        b"pub fn ok() {}",
    )
    .expect("write lib.rs");
    fs::write(
        seam_tmp.path().join("Cargo.lock"),
        b"[[package]]\nname = \"esp-wifi-sys\"\n",
    )
    .expect("write lock");

    let seam_phase1 = audit_pure_rust(seam_tmp.path(), true).expect("audit");
    assert!(seam_phase1.is_clean(), "esp-wifi-sys permitted in phase 1");

    let seam_post = audit_pure_rust(seam_tmp.path(), false).expect("audit");
    assert!(
        !seam_post.is_clean(),
        "esp-wifi-sys rejected in post-phase 1"
    );
    assert!(seam_post
        .disallowed_dependencies
        .contains(&"esp-wifi-sys".to_string()));
}
