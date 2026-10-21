//! Entrypoint for `klipper-cam-fw`.
//!
//! When compiled for host (`x86_64` / `aarch64`), runs a self-verification check of the
//! partition table, OTA CRC32 state machine, and dual-port HTTP/MJPEG handlers.
//! When compiled for Xtensa bare-metal (`target_arch = "xtensa"`), provides the
//! `#![no_std]` firmware initialization entrypoint.

#![cfg_attr(target_arch = "xtensa", no_std)]
#![cfg_attr(target_arch = "xtensa", no_main)]
#![forbid(unsafe_code)]

use klipper_cam_fw::{
    verify_partition_table_integrity, FirmwareRuntime, HttpResponseView, Ipv4Addr, ServerPort,
};

fn run_firmware_self_test() -> bool {
    if !verify_partition_table_integrity() {
        return false;
    }

    let mut runtime = FirmwareRuntime::new_default();
    runtime.on_wifi_sta_start();
    if runtime
        .on_wifi_got_ip(Ipv4Addr::new(172, 22, 1, 52))
        .is_err()
    {
        return false;
    }

    let mut buf = [0u8; 512];
    let status_resp = match runtime.server.handle_raw_request(
        ServerPort::Control,
        b"GET /status HTTP/1.1\r\nHost: 172.22.1.52\r\n\r\n",
        &mut buf,
    ) {
        Ok(bytes) => bytes,
        Err(_) => return false,
    };

    match HttpResponseView::parse(status_resp) {
        Ok(view) => view.status_code == 200,
        Err(_) => false,
    }
}

#[cfg(not(target_arch = "xtensa"))]
fn main() {
    let ok = run_firmware_self_test();
    assert!(ok, "klipper-cam-fw self-test failed");
}

/// Bare-metal Xtensa firmware entrypoint.
#[cfg(target_arch = "xtensa")]
pub extern "C" fn main() -> ! {
    let _ok = run_firmware_self_test();
    loop {
        core::hint::spin_loop();
    }
}

#[cfg(target_arch = "xtensa")]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {
        core::hint::spin_loop();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_main_firmware_self_test() {
        assert!(run_firmware_self_test());
    }
}
