//! Top-level CLI subcommand dispatcher for `xtask`.

use crate::{
    bootstrap_env, elf_wrap, ota_deploy, pure_rust_audit, serial_passive, verify_invariants,
};

const AVAILABLE_SUBCOMMANDS: &str = "verify-invariants, ota-deploy, ensure-serial-passive, \
    bootstrap-env, wrap-esp-image-elf, build-bootloader, verify-pure-rust";

/// Dispatches CLI commands to their respective submodules after validating
/// the command name and argument stream for control-byte injection.
///
/// # Errors
/// Returns an error message if no subcommand is given, an argument contains
/// NUL bytes, or the invoked subcommand fails.
pub fn run(args: &[String]) -> Result<(), String> {
    let Some(cmd) = args.first() else {
        return Err(format!(
            "No subcommand provided. Available: {AVAILABLE_SUBCOMMANDS}"
        ));
    };

    if args.iter().any(|arg| arg.contains('\0')) {
        return Err("Invalid CLI argument containing NUL byte".to_string());
    }

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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_run_rejects_empty_and_unknown_and_nul_args() {
        assert!(run(&[]).is_err());
        assert!(run(&["no-such-cmd".to_string()]).is_err());
        assert!(run(&["verify-pure-rust".to_string(), "bad\0arg".to_string()]).is_err());
    }
}
