//! Binary entrypoint for `esp32-cam-bootloader`.

#![cfg_attr(target_arch = "xtensa", no_std)]
#![cfg_attr(target_arch = "xtensa", no_main)]

#[cfg(target_arch = "xtensa")]
#[no_mangle]
pub extern "C" fn _start() -> ! {
    // JUSTIFICATION: Linker script `linkall.x` exports `.bss` boundary symbols
    // via FFI static declarations.
    extern "C" {
        static mut _bss_start: u32;
        static mut _bss_end: u32;
    }

    // JUSTIFICATION: Bare-metal Xtensa entry point must zero `.bss` before
    // executing Rust code that accesses static storage.
    // SAFETY: `_bss_start` and `_bss_end` are 4-byte aligned linker symbols
    // bounding `.bss` in `dram_seg` (`linkall.x`).
    unsafe {
        let mut cursor = core::ptr::addr_of_mut!(_bss_start);
        let end = core::ptr::addr_of_mut!(_bss_end);
        while (cursor as usize) < (end as usize) {
            core::ptr::write_volatile(cursor, 0);
            cursor = cursor.add(1);
        }
    }

    esp32_cam_bootloader::run_xtensa_boot()
}

#[cfg(target_arch = "xtensa")]
#[allow(clippy::empty_loop)]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    let mut wdt = esp32_cam_bootloader::XtensaWdtMmio;
    esp32_cam_bootloader::trigger_sw_reset(&mut wdt);
    loop {}
}

#[cfg(not(target_arch = "xtensa"))]
fn main() {
    // Host executable placeholder for workspace compilation verification.
}
