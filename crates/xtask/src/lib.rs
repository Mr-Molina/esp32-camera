//! Host automation and verification tasks for `esp32-camera`.

#![cfg_attr(target_arch = "xtensa", no_std)]

#[cfg(not(target_arch = "xtensa"))]
pub mod bootstrap_env;
#[cfg(not(target_arch = "xtensa"))]
pub mod elf_wrap;
#[cfg(not(target_arch = "xtensa"))]
pub mod ota_deploy;
#[cfg(not(target_arch = "xtensa"))]
pub mod pure_rust_audit;
#[cfg(not(target_arch = "xtensa"))]
pub mod serial_passive;
pub mod sha256;
#[cfg(not(target_arch = "xtensa"))]
pub mod verify_invariants;

#[cfg(not(target_arch = "xtensa"))]
/// Dispatches CLI commands to their respective submodules.
///
/// # Errors
/// Returns an error message if the command fails or is unknown.
pub fn run(args: &[String]) -> Result<(), String> {
    let Some(cmd) = args.first() else {
        return Err(
            "No subcommand provided. Available: verify-invariants, ota-deploy, \
             ensure-serial-passive, bootstrap-env, wrap-esp-image-elf, \
             build-bootloader, verify-pure-rust"
                .to_string(),
        );
    };

    let rest = &args[1..];
    match cmd.as_str() {
        "verify-invariants" => verify_invariants::run(rest),
        "ota-deploy" => ota_deploy::run(rest),
        "ensure-serial-passive" => serial_passive::run(rest),
        "bootstrap-env" => bootstrap_env::run(rest),
        "wrap-esp-image-elf" => elf_wrap::run(rest),
        "build-bootloader" => elf_wrap::run_build_bootloader(rest),
        "verify-pure-rust" => pure_rust_audit::run(rest),
        other => Err(format!("Unknown subcommand '{other}'")),
    }
}
