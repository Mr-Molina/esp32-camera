//! Toolchain and environment bootstrap subcommand for cloud and container sessions.
//!
//! Replaces the retired `scripts/cloud-setup.sh` in pure Rust, ensuring the
//! developer plugins that `.claude/settings.json` enables for local sessions
//! (`tokeneconomizer@tokeneconomizer` and `superpowers@superpowers-dev`) are
//! also installed in headless/cloud environments, which ignore those keys.

use std::process::Command;

/// Runs the `bootstrap-env` subcommand.
///
/// # Errors
/// Returns an error message if argument parsing fails. Individual plugin
/// installation failures are non-fatal, matching the retired `cloud-setup.sh`.
pub fn run(args: &[String]) -> Result<(), String> {
    if args.iter().any(|a| a == "--help" || a == "-h") {
        print_help();
        return Ok(());
    }

    println!("[BOOTSTRAP] Running cloud environment setup...");

    // 1. Add Microsoft TokenEconomizer marketplace
    install_plugin_step(
        "marketplace add microsoft/TokenEconomizer",
        &["plugin", "marketplace", "add", "microsoft/TokenEconomizer"],
    );

    // 2. Install tokeneconomizer plugin
    install_plugin_step(
        "install tokeneconomizer@tokeneconomizer -y",
        &["plugin", "install", "tokeneconomizer@tokeneconomizer", "-y"],
    );

    // 3. Add the Superpowers development marketplace (named `superpowers-dev`
    //    by its marketplace.json, matching `.claude/settings.json`)
    install_plugin_step(
        "marketplace add Mr-Molina/superpowers",
        &["plugin", "marketplace", "add", "Mr-Molina/superpowers"],
    );

    // 4. Install superpowers plugin
    install_plugin_step(
        "install superpowers@superpowers-dev -y",
        &["plugin", "install", "superpowers@superpowers-dev", "-y"],
    );

    println!("[BOOTSTRAP] Environment setup complete.");
    Ok(())
}

fn install_plugin_step(description: &str, args: &[&str]) {
    println!("[BOOTSTRAP] Executing: claude {}", description);
    match Command::new("claude").args(args).status() {
        Ok(status) => {
            if status.success() {
                println!("[BOOTSTRAP] Successfully completed: claude {}", description);
            } else {
                eprintln!(
                    "[BOOTSTRAP-WARN] Non-zero exit status ({:?}) for: claude {}",
                    status.code(),
                    description
                );
            }
        }
        Err(err) => {
            eprintln!(
                "[BOOTSTRAP-WARN] Could not execute 'claude {}' (non-fatal): {}",
                description, err
            );
        }
    }
}

fn print_help() {
    println!("Usage: xtask bootstrap-env [OPTIONS]");
    println!();
    println!("Bootstrap cloud and container developer environment plugins.");
    println!();
    println!("Installs the plugins `.claude/settings.json` enables for local sessions:");
    println!("  tokeneconomizer@tokeneconomizer  (marketplace microsoft/TokenEconomizer)");
    println!("  superpowers@superpowers-dev      (marketplace Mr-Molina/superpowers)");
    println!();
    println!("Options:");
    println!("  -h, --help    Print help information");
}
