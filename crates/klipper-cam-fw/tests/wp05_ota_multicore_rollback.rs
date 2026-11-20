//! WP-05 integration tests: multicore parking during flash writes and
//! two-phase OTA rollback cancellation.
//!
//! Verifies:
//! 1. Core 1 is actively parked during all SPI NOR flash erases and writes.
//! 2. Two-phase rollback cancellation waits for both IP and HTTP request.
//! 3. Ping-pong sector rotation increments sequence numbers and detects CRC
//!    corruption.
//! 4. Unauthenticated or truncated POST /update requests reject without
//!    parking Core 1.

#![forbid(unsafe_code)]

extern crate alloc;

use alloc::format;
use alloc::vec::Vec;
use core::sync::atomic::AtomicBool;
use core::sync::atomic::AtomicU32;
use core::sync::atomic::Ordering;
use klipper_cam_fw::CameraDriver;
use klipper_cam_fw::Core1ParkControl;
use klipper_cam_fw::EspImageHeader;
use klipper_cam_fw::FirmwareRuntime;
use klipper_cam_fw::GuardedFlashStorage;
use klipper_cam_fw::Ipv4Addr;
use klipper_cam_fw::KlipperCamConfig;
use klipper_cam_fw::KlipperCamServer;
use klipper_cam_fw::MockNorFlash;
use klipper_cam_fw::OtaDataManager;
use klipper_cam_fw::OtaDataSelect;
use klipper_cam_fw::OtaError;
use klipper_cam_fw::OtaFlashStorage;
use klipper_cam_fw::OtaImgState;
use klipper_cam_fw::OtaSlot;
use klipper_cam_fw::ServerPort;

/// Private OTA token for these tests; the public default token is refused
/// with `403` (WP-09, `FW-NET-04`).
const TEST_OTA_TOKEN: &str = "wp05-test-ota-token";

/// Server with a private OTA token.
fn test_server() -> KlipperCamServer<'static> {
    let mut cfg = KlipperCamConfig::WROVER_KIT_HD;
    cfg.ota_token = TEST_OTA_TOKEN;
    KlipperCamServer::new(cfg, CameraDriver::new_hd_default())
}

/// Mock multicore parking controller tracking pause invocations and state.
struct MockPark {
    is_paused: AtomicBool,
    pause_calls: AtomicU32,
    allow_pause: AtomicBool,
}

impl MockPark {
    const fn new() -> Self {
        Self {
            is_paused: AtomicBool::new(false),
            pause_calls: AtomicU32::new(0),
            allow_pause: AtomicBool::new(true),
        }
    }
}

impl Core1ParkControl for MockPark {
    fn pause_core1(&self) -> bool {
        if !self.allow_pause.load(Ordering::SeqCst) {
            return false;
        }
        self.pause_calls.fetch_add(1, Ordering::SeqCst);
        self.is_paused.store(true, Ordering::SeqCst);
        true
    }

    fn resume_core1(&self) -> bool {
        self.is_paused.store(false, Ordering::SeqCst);
        true
    }
}

impl Core1ParkControl for &MockPark {
    fn pause_core1(&self) -> bool {
        (*self).pause_core1()
    }

    fn resume_core1(&self) -> bool {
        (*self).resume_core1()
    }
}

/// Storage wrapper asserting that Core 1 is actively parked on every call.
struct AssertingFlash<'a, const APP_WINDOW: usize> {
    inner: MockNorFlash<APP_WINDOW>,
    is_paused: &'a AtomicBool,
}

impl<'a, const APP_WINDOW: usize> OtaFlashStorage for AssertingFlash<'a, APP_WINDOW> {
    fn read_aligned(&mut self, offset: u32, buf: &mut [u8]) -> Result<(), OtaError> {
        assert!(
            self.is_paused.load(Ordering::SeqCst),
            "Core 1 must be parked during read_aligned"
        );
        self.inner.read_aligned(offset, buf)
    }

    fn erase_range(&mut self, from: u32, to: u32) -> Result<(), OtaError> {
        assert!(
            self.is_paused.load(Ordering::SeqCst),
            "Core 1 must be parked during erase_range"
        );
        self.inner.erase_range(from, to)
    }

    fn write_aligned(&mut self, offset: u32, data: &[u8]) -> Result<(), OtaError> {
        assert!(
            self.is_paused.load(Ordering::SeqCst),
            "Core 1 must be parked during write_aligned"
        );
        self.inner.write_aligned(offset, data)
    }

    fn has_persistent_otadata(&self) -> bool {
        self.inner.has_persistent_otadata()
    }
}

/// Decodes the 32-byte OTA select record from a 4 KiB otadata sector.
fn read_otadata_record(sector: &[u8; 4096]) -> OtaDataSelect {
    let mut bytes = [0u8; 32];
    bytes.copy_from_slice(&sector[..32]);
    OtaDataSelect::from_bytes(&bytes)
}

/// Helper building raw HTTP POST /update requests for testing.
fn make_update_request(token: Option<&str>, body: &[u8], content_length: Option<usize>) -> Vec<u8> {
    let mut req = Vec::new();
    req.extend_from_slice(b"POST /update HTTP/1.1\r\nHost: 172.22.1.52\r\n");
    if let Some(tok) = token {
        req.extend_from_slice(b"X-OTA-Token: ");
        req.extend_from_slice(tok.as_bytes());
        req.extend_from_slice(b"\r\n");
    }
    if let Some(len) = content_length {
        let len_hdr = format!("Content-Length: {}\r\n", len);
        req.extend_from_slice(len_hdr.as_bytes());
    }
    req.extend_from_slice(b"\r\n");
    req.extend_from_slice(body);
    req
}

#[test]
fn paused_flag_set_during_every_erase_and_write() {
    let park = MockPark::new();
    let inner_flash = MockNorFlash::<8192>::new_booted_ota0();
    let asserting_flash = AssertingFlash {
        inner: inner_flash,
        is_paused: &park.is_paused,
    };
    let mut guarded = GuardedFlashStorage::new(asserting_flash, &park);

    let mut server = test_server();
    let valid_image = EspImageHeader::minimal_valid_image_64();
    let req_bytes =
        make_update_request(Some(TEST_OTA_TOKEN), &valid_image, Some(valid_image.len()));

    let mut out_buf = [0u8; 1024];
    let resp = server
        .handle_raw_request_with_flash(ServerPort::Control, &req_bytes, &mut out_buf, &mut guarded)
        .expect("handle valid POST /update");

    assert!(
        resp.starts_with(b"HTTP/1.1 200 OK"),
        "expected 200 OK response"
    );
    assert!(
        park.pause_calls.load(Ordering::SeqCst) > 0,
        "Core 1 must have been paused at least once"
    );
    assert!(
        !park.is_paused.load(Ordering::SeqCst),
        "Core 1 must be resumed after guard drop"
    );

    // When Core 1 parking times out / fails on a fresh server (since `M-043`
    // returns `409 Conflict` once `reboot_scheduled` is true), OTA returns 500.
    let park_500 = MockPark::new();
    park_500.allow_pause.store(false, Ordering::SeqCst);
    let inner_flash_500 = MockNorFlash::<8192>::new_booted_ota0();
    let asserting_flash_500 = AssertingFlash {
        inner: inner_flash_500,
        is_paused: &park_500.is_paused,
    };
    let mut guarded_500 = GuardedFlashStorage::new(asserting_flash_500, &park_500);
    let mut server_500 = test_server();
    let mut out_buf_500 = [0u8; 1024];
    let resp_500 = server_500
        .handle_raw_request_with_flash(
            ServerPort::Control,
            &req_bytes,
            &mut out_buf_500,
            &mut guarded_500,
        )
        .expect("handle POST /update with failed parking");

    assert!(
        resp_500.starts_with(b"HTTP/1.1 500 Internal Server Error"),
        "expected 500 when Core 1 cannot be parked"
    );
    assert!(
        !park_500.is_paused.load(Ordering::SeqCst),
        "Core 1 must remain resumed on failure"
    );
}

#[test]
fn boot_sector1_seq2_new_then_ip_and_http_commits_valid() {
    let mut flash: MockNorFlash<8192> = MockNorFlash::new();
    let s1_bytes = OtaDataSelect::new(2, OtaImgState::New).to_bytes();
    flash.otadata_s1[..32].copy_from_slice(&s1_bytes);

    let mgr = OtaDataManager::load_from_flash(&mut flash).expect("load otadata from flash");
    assert_eq!(mgr.active_slot(), OtaSlot::Ota1);

    let mut runtime = FirmwareRuntime::new_default();
    runtime.server.ota_manager = mgr;

    // Phase 1: Wi-Fi IP confirmed alone must not cancel rollback before HTTP
    runtime.on_wifi_ip_confirmed(Ipv4Addr::new(172, 22, 1, 52));
    let pre_http_result = runtime
        .try_cancel_rollback_with_flash(&mut flash)
        .expect("pre-http try_cancel_rollback_with_flash");
    assert!(
        !pre_http_result,
        "rollback must not be cancelled before first HTTP request"
    );
    assert!(
        !runtime.rollback_cancelled,
        "runtime rollback_cancelled must be false"
    );

    // Phase 2: Non-2xx HTTP status must not confirm rollback; 2xx confirms it
    runtime.on_http_request_served_status(401);
    assert!(
        !runtime.first_http_served,
        "non-2xx response must not mark first_http_served"
    );
    runtime.on_http_request_served_status(200);
    assert!(
        runtime.first_http_served,
        "2xx response must mark first_http_served"
    );

    // Transient flash failure leaves rollback_cancelled false for retry
    flash.fail_writes = true;
    let write_err = runtime.try_cancel_rollback_with_flash(&mut flash);
    assert!(
        write_err.is_err(),
        "transient flash failure must return Err"
    );
    assert!(
        !runtime.rollback_cancelled,
        "rollback_cancelled must remain false after write failure"
    );

    // Flash recovery succeeds and commits OtaImgState::Valid with valid CRC
    flash.fail_writes = false;
    let post_http_result = runtime
        .try_cancel_rollback_with_flash(&mut flash)
        .expect("successful try_cancel_rollback_with_flash");
    assert!(
        post_http_result,
        "rollback must be cancelled after first HTTP request"
    );
    assert!(
        runtime.rollback_cancelled,
        "runtime rollback_cancelled must be true"
    );

    // M-037: Because sector 1 was the sole bootable sector (`sector 0` erased),
    // rollback cancellation writes `(seq + 2 = 4, Valid)` atomically to sector 0
    // without ever erasing sector 1 (zero brick window).
    let decoded_s0 = read_otadata_record(&flash.otadata_s0);
    assert_eq!(decoded_s0.ota_seq, 4);
    assert_eq!(decoded_s0.state(), OtaImgState::Valid);
    assert!(
        decoded_s0.is_crc_valid(),
        "persisted sector 0 CRC must be valid"
    );
    let decoded_s1 = read_otadata_record(&flash.otadata_s1);
    assert_eq!(decoded_s1.ota_seq, 2);
    assert_eq!(
        decoded_s1.state(),
        OtaImgState::New,
        "sole active sector 1 must remain intact without erase"
    );

    // M-039: Direct `on_wifi_got_ip_with_flash` MUST NOT cancel rollback in
    // flash before the first HTTP 2xx response is served; once HTTP 2xx is
    // served, it delegates to `try_cancel_rollback_with_flash` and commits.
    let mut flash2: MockNorFlash<8192> = MockNorFlash::new();
    flash2.otadata_s1[..32].copy_from_slice(&s1_bytes);
    let mgr2 = OtaDataManager::load_from_flash(&mut flash2).expect("load mgr2 from flash");
    let mut runtime2 = FirmwareRuntime::new_default();
    runtime2.server.ota_manager = mgr2;

    runtime2
        .on_wifi_got_ip_with_flash(Ipv4Addr::new(172, 22, 1, 52), &mut flash2)
        .expect("on_wifi_got_ip_with_flash before HTTP 2xx");
    assert!(
        !runtime2.rollback_cancelled,
        "GOT_IP alone must not cancel rollback before HTTP 2xx"
    );
    assert_eq!(
        read_otadata_record(&flash2.otadata_s1).state(),
        OtaImgState::New
    );

    runtime2.on_http_request_served_status(200);
    runtime2
        .on_wifi_got_ip_with_flash(Ipv4Addr::new(172, 22, 1, 52), &mut flash2)
        .expect("on_wifi_got_ip_with_flash after HTTP 2xx");
    assert!(runtime2.rollback_cancelled);
    let decoded2_s0 = read_otadata_record(&flash2.otadata_s0);
    assert_eq!(decoded2_s0.ota_seq, 4);
    assert_eq!(decoded2_s0.state(), OtaImgState::Valid);
    assert!(
        decoded2_s0.is_crc_valid(),
        "post-HTTP got_ip path must persist Valid state with valid CRC"
    );
}

#[test]
fn seq_7_8_commits_to_seq_9_in_sector_0_and_inconsistent_otadata_returns_500() {
    let mut flash: MockNorFlash<8192> = MockNorFlash::new();
    let s0_init = OtaDataSelect::new(7, OtaImgState::Valid).to_bytes();
    let s1_init = OtaDataSelect::new(8, OtaImgState::Valid).to_bytes();
    flash.otadata_s0[..32].copy_from_slice(&s0_init);
    flash.otadata_s1[..32].copy_from_slice(&s1_init);

    let mgr = OtaDataManager::load_from_flash(&mut flash).expect("load 7/8 flash");
    assert_eq!(mgr.active_slot(), OtaSlot::Ota1);
    assert_eq!(mgr.next_update_slot(), OtaSlot::Ota0);

    let mut server = test_server();
    server.ota_manager = mgr;

    let valid_image = EspImageHeader::minimal_valid_image_64();
    let req_bytes =
        make_update_request(Some(TEST_OTA_TOKEN), &valid_image, Some(valid_image.len()));

    let mut out_buf = [0u8; 1024];
    let resp = server
        .handle_raw_request_with_flash(ServerPort::Control, &req_bytes, &mut out_buf, &mut flash)
        .expect("handle valid POST /update on seq 7/8");

    assert!(
        resp.starts_with(b"HTTP/1.1 200 OK"),
        "expected 200 OK on successful OTA update"
    );

    let s0_after = read_otadata_record(&flash.otadata_s0);
    assert_eq!(
        s0_after.ota_seq, 9,
        "sector 0 ota_seq must increment from 8 to 9"
    );
    assert_eq!(
        s0_after.state(),
        OtaImgState::PendingVerify,
        "newly written sector must be in PendingVerify state"
    );
    assert!(s0_after.is_crc_valid(), "sector 0 CRC must be valid");

    // M-038: Corrupting the ACTIVE sector (making flash disagree with
    // `running_slot` or leaving neither sector bootable) must abort with HTTP
    // 500 without erasing application partitions.
    let mut flash_corrupt: MockNorFlash<8192> = MockNorFlash::new();
    flash_corrupt.otadata_s0[..32].copy_from_slice(&s0_init);
    flash_corrupt.otadata_s1[..32].copy_from_slice(&s1_init);
    flash_corrupt.otadata_s1[0] ^= 0x55; // Corrupt CRC in active sector 1

    let mut server_corrupt = test_server();
    server_corrupt.ota_manager = mgr;

    let erases_before = flash_corrupt.erase_count;
    let mut out_buf_corrupt = [0u8; 1024];
    let resp_corrupt = server_corrupt
        .handle_raw_request_with_flash(
            ServerPort::Control,
            &req_bytes,
            &mut out_buf_corrupt,
            &mut flash_corrupt,
        )
        .expect("handle POST /update with corrupted active otadata");

    assert!(
        resp_corrupt.starts_with(b"HTTP/1.1 500 Internal Server Error"),
        "expected 500 on corrupted active otadata flash"
    );
    assert_eq!(
        flash_corrupt.erase_count, erases_before,
        "corrupted active otadata must abort before erasing any flash partitions"
    );

    // M-038: Conversely, corrupting only the INACTIVE sector while the active
    // sector (`sector_1`, seq 8, Valid) matches `running_slot` (`Ota1`) must
    // recover cleanly by overwriting the corrupt inactive sector.
    let mut flash_recover: MockNorFlash<8192> = MockNorFlash::new();
    flash_recover.otadata_s0[..32].copy_from_slice(&s0_init);
    flash_recover.otadata_s1[..32].copy_from_slice(&s1_init);
    flash_recover.otadata_s0[28..32].copy_from_slice(&0xFFFF_FF9Au32.to_le_bytes());

    let mut server_recover = test_server();
    server_recover.ota_manager = mgr;
    let mut out_buf_recover = [0u8; 1024];
    let resp_recover = server_recover
        .handle_raw_request_with_flash(
            ServerPort::Control,
            &req_bytes,
            &mut out_buf_recover,
            &mut flash_recover,
        )
        .expect("handle POST /update recovering corrupt inactive sector");
    assert!(
        resp_recover.starts_with(b"HTTP/1.1 200 OK"),
        "corrupt inactive sector must be recoverable when active sector is valid"
    );
    let s0_recovered = read_otadata_record(&flash_recover.otadata_s0);
    assert_eq!(s0_recovered.ota_seq, 9);
    assert!(s0_recovered.is_crc_valid());
}

#[test]
fn bare_or_unauthed_or_incomplete_post_does_not_pause_core1() {
    let park = MockPark::new();
    let flash = MockNorFlash::<8192>::new_booted_ota0();
    let mut guarded = GuardedFlashStorage::new(flash, &park);
    let mut server = test_server();
    let valid_image = EspImageHeader::minimal_valid_image_64();

    // 1. Bare POST /update with no headers or token
    let bare_req = b"POST /update HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n";
    let mut out_buf1 = [0u8; 1024];
    let resp1 = server
        .handle_raw_request_with_flash(ServerPort::Control, bare_req, &mut out_buf1, &mut guarded)
        .expect("handle bare POST /update");
    assert!(
        resp1.starts_with(b"HTTP/1.1 401 Unauthorized"),
        "bare request must be rejected with 401 Unauthorized"
    );
    assert_eq!(
        park.pause_calls.load(Ordering::SeqCst),
        0,
        "Core 1 must not be paused for bare unauthenticated request"
    );

    // 2. Invalid X-OTA-Token
    let bad_token_req = make_update_request(
        Some("invalid-token-1234"),
        &valid_image,
        Some(valid_image.len()),
    );
    let mut out_buf2 = [0u8; 1024];
    let resp2 = server
        .handle_raw_request_with_flash(
            ServerPort::Control,
            &bad_token_req,
            &mut out_buf2,
            &mut guarded,
        )
        .expect("handle invalid token POST /update");
    assert!(
        resp2.starts_with(b"HTTP/1.1 401 Unauthorized"),
        "invalid token must be rejected with 401 Unauthorized"
    );
    assert_eq!(
        park.pause_calls.load(Ordering::SeqCst),
        0,
        "Core 1 must not be paused for invalid token"
    );

    // 3. Truncated body (Content-Length declares 4096 bytes but only 64 provided)
    let truncated_req = make_update_request(Some(TEST_OTA_TOKEN), &valid_image, Some(4096));
    let mut out_buf3 = [0u8; 1024];
    let resp3 = server
        .handle_raw_request_with_flash(
            ServerPort::Control,
            &truncated_req,
            &mut out_buf3,
            &mut guarded,
        )
        .expect("handle truncated POST /update");
    assert!(
        resp3.starts_with(b"HTTP/1.1 400 Bad Request"),
        "truncated body must be rejected with 400 Bad Request"
    );
    assert_eq!(
        park.pause_calls.load(Ordering::SeqCst),
        0,
        "Core 1 must not be paused for incomplete/truncated body"
    );
}
