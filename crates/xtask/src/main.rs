//! Entrypoint for repository xtask tooling.

#![cfg_attr(target_arch = "xtensa", no_std)]
#![cfg_attr(target_arch = "xtensa", no_main)]

#[cfg(not(target_arch = "xtensa"))]
fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if let Err(err) = xtask::run(&args) {
        eprintln!("[FAIL] {err}");
        std::process::exit(1);
    }
}

#[cfg(target_arch = "xtensa")]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
