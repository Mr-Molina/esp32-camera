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
use klipper_cam_fw::Core1ParkControl;
use klipper_cam_fw::EspImageHeader;
use klipper_cam_fw::FirmwareRuntime;
use klipper_cam_fw::GuardedFlashStorage;
use klipper_cam_fw::Ipv4Addr;
use klipper_cam_fw::KlipperCamServer;
use klipper_cam_fw::MockNorFlash;
use klipper_cam_fw::OtaDataManager;
use klipper_cam_fw::OtaDataSelect;
use klipper_cam_fw::OtaError;
use klipper_cam_fw::OtaFlashStorage;
use klipper_cam_fw::OtaImgState;
use klipper_cam_fw::OtaSlot;
use klipper_cam_fw::ServerPort;
use klipper_cam_fw::DEFAULT_OTA_TOKEN;

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

    let mut server = KlipperCamServer::new_default();
    let valid_image = EspImageHeader::minimal_valid_image_64();
    let req_bytes = make_update_request(
        Some(DEFAULT_OTA_TOKEN),
        &valid_image,
        Some(valid_image.len()),
    );

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

    // When Core 1 parking times out / fails, OTA returns HTTP 500
    park.allow_pause.store(false, Ordering::SeqCst);
    let mut out_buf_500 = [0u8; 1024];
    let resp_500 = server
        .handle_raw_request_with_flash(
            ServerPort::Control,
            &req_bytes,
            &mut out_buf_500,
            &mut guarded,
        )
        .expect("handle POST /update with failed parking");

    assert!(
        resp_500.starts_with(b"HTTP/1.1 500 Internal Server Error"),
        "expected 500 when Core 1 cannot be parked"
    );
    assert!(
        !park.is_paused.load(Ordering::SeqCst),
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

    // Phase 2: First HTTP request served
    runtime.on_http_request_served();

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

    let decoded_s1 = read_otadata_record(&flash.otadata_s1);
    assert_eq!(decoded_s1.ota_seq, 2);
    assert_eq!(decoded_s1.state(), OtaImgState::Valid);
    assert!(
        decoded_s1.is_crc_valid(),
        "persisted sector 1 CRC must be valid"
    );

    // Also test direct on_wifi_got_ip_with_flash path on a second flash instance
    let mut flash2: MockNorFlash<8192> = MockNorFlash::new();
    flash2.otadata_s1[..32].copy_from_slice(&s1_bytes);
    let mgr2 = OtaDataManager::load_from_flash(&mut flash2).expect("load mgr2 from flash");
    let mut runtime2 = FirmwareRuntime::new_default();
    runtime2.server.ota_manager = mgr2;

    runtime2
        .on_wifi_got_ip_with_flash(Ipv4Addr::new(172, 22, 1, 52), &mut flash2)
        .expect("on_wifi_got_ip_with_flash");

    let decoded2 = read_otadata_record(&flash2.otadata_s1);
    assert_eq!(decoded2.ota_seq, 2);
    assert_eq!(decoded2.state(), OtaImgState::Valid);
    assert!(
        decoded2.is_crc_valid(),
        "direct got_ip path must persist Valid state with valid CRC"
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

    let mut server = KlipperCamServer::new_default();
    server.ota_manager = mgr;

    let valid_image = EspImageHeader::minimal_valid_image_64();
    let req_bytes = make_update_request(
        Some(DEFAULT_OTA_TOKEN),
        &valid_image,
        Some(valid_image.len()),
    );

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

    // Inconsistent / corrupted otadata on flash must abort with HTTP 500
    // without erasing application partitions
    let mut flash_corrupt: MockNorFlash<8192> = MockNorFlash::new();
    flash_corrupt.otadata_s0[..32].copy_from_slice(&s0_init);
    flash_corrupt.otadata_s1[..32].copy_from_slice(&s1_init);
    flash_corrupt.otadata_s0[0] ^= 0x55; // Corrupt CRC in sector 0

    let mut server_corrupt = KlipperCamServer::new_default();
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
        .expect("handle POST /update with corrupted otadata");

    assert!(
        resp_corrupt.starts_with(b"HTTP/1.1 500 Internal Server Error"),
        "expected 500 on corrupted otadata flash"
    );
    assert_eq!(
        flash_corrupt.erase_count, erases_before,
        "corrupted otadata must abort before erasing any flash partitions"
    );
}

#[test]
fn bare_or_unauthed_or_incomplete_post_does_not_pause_core1() {
    let park = MockPark::new();
    let flash = MockNorFlash::<8192>::new_booted_ota0();
    let mut guarded = GuardedFlashStorage::new(flash, &park);
    let mut server = KlipperCamServer::new_default();
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
    let truncated_req = make_update_request(Some(DEFAULT_OTA_TOKEN), &valid_image, Some(4096));
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
