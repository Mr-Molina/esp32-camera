//! Pure-Rust codebase audit subcommand.
//!
//! Enforces zero-tolerance repository compliance:
//! - Disallows .py, .sh, .c, .cpp, .h, and .S files in the codebase (excluding
//!   .git, target, .agents).
//! - Disallows legacy Wi-Fi stack dependencies (esp-radio, esp-rtos) in
//!   Cargo.lock across all phases.
//! - In post-phase1 mode (!allow_phase1), also disallows firmware/bootloader*.bin
//!   blobs and legacy dependencies (esp-wifi-sys, esp-bootloader-esp-idf) in
//!   Cargo.lock.

use std::fs;
use std::path::{Path, PathBuf};

/// Audit report summarizing pure-Rust compliance findings.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct AuditReport {
    /// Relative paths of disallowed non-Rust source files (.py, .sh, .c, .cpp, .h, .S).
    pub offending_files: Vec<PathBuf>,
    /// Relative paths of legacy binary bootloader blobs.
    pub bootloader_blobs: Vec<PathBuf>,
    /// Disallowed dependencies found in Cargo.lock.
    pub disallowed_dependencies: Vec<String>,
}

impl AuditReport {
    /// Returns true if the repository passes all audit criteria.
    #[must_use]
    pub fn is_clean(&self) -> bool {
        self.offending_files.is_empty()
            && self.bootloader_blobs.is_empty()
            && self.disallowed_dependencies.is_empty()
    }

    /// Alias for [`Self::is_clean`].
    #[must_use]
    pub fn passed(&self) -> bool {
        self.is_clean()
    }
}

/// Recursively walks `dir`, collecting non-Rust source files and bootloader blobs.
fn walk_dir(
    dir: &Path,
    repo_root: &Path,
    allow_phase1: bool,
    offending: &mut Vec<PathBuf>,
    bootloaders: &mut Vec<PathBuf>,
) -> Result<(), String> {
    let entries = fs::read_dir(dir)
        .map_err(|e| format!("Failed to read directory '{}': {e}", dir.display()))?;

    for entry in entries {
        let entry = entry
            .map_err(|e| format!("Failed to read directory entry in '{}': {e}", dir.display()))?;
        let path = entry.path();
        let file_name = entry.file_name();
        let name_str = file_name.to_string_lossy();

        if path.is_dir() {
            // Skip .git, target, and .agents directories
            if name_str == ".git" || name_str == "target" || name_str == ".agents" {
                continue;
            }
            walk_dir(&path, repo_root, allow_phase1, offending, bootloaders)?;
        } else if path.is_file() {
            let rel_path = path.strip_prefix(repo_root).unwrap_or(&path).to_path_buf();

            if let Some(ext) = path.extension().and_then(|s| s.to_str()) {
                let lower_ext = ext.to_ascii_lowercase();
                if lower_ext == "py"
                    || lower_ext == "sh"
                    || lower_ext == "c"
                    || lower_ext == "cpp"
                    || lower_ext == "h"
                    || ext == "S"
                {
                    offending.push(rel_path.clone());
                }
            }

            if !allow_phase1
                && name_str.starts_with("bootloader")
                && name_str.ends_with(".bin")
                && rel_path.starts_with("firmware")
            {
                bootloaders.push(rel_path);
            }
        }
    }

    Ok(())
}

/// Audits the given repository root for pure-Rust invariants.
///
/// # Errors
/// Returns an error message if filesystem traversal or reading Cargo.lock fails.
pub fn audit_pure_rust(repo_root: &Path, allow_phase1: bool) -> Result<AuditReport, String> {
    let mut offending_files = Vec::new();
    let mut bootloader_blobs = Vec::new();
    let mut disallowed_dependencies = Vec::new();

    walk_dir(
        repo_root,
        repo_root,
        allow_phase1,
        &mut offending_files,
        &mut bootloader_blobs,
    )?;

    let cargo_lock = repo_root.join("Cargo.lock");
    if cargo_lock.exists() {
        let content = fs::read_to_string(&cargo_lock)
            .map_err(|e| format!("Failed to read Cargo.lock: {e}"))?;
        if content.contains("esp-radio") {
            disallowed_dependencies.push("esp-radio".to_string());
        }
        if content.contains("esp-rtos") {
            disallowed_dependencies.push("esp-rtos".to_string());
        }
        if !allow_phase1 {
            if content.contains("esp-wifi-sys") {
                disallowed_dependencies.push("esp-wifi-sys".to_string());
            }
            if content.contains("esp-bootloader-esp-idf") {
                disallowed_dependencies.push("esp-bootloader-esp-idf".to_string());
            }
        }
    }

    offending_files.sort();
    bootloader_blobs.sort();
    disallowed_dependencies.sort();

    Ok(AuditReport {
        offending_files,
        bootloader_blobs,
        disallowed_dependencies,
    })
}

/// Runs the `verify-pure-rust` CLI subcommand.
///
/// # Errors
/// Returns an error if the audit fails (violations found) or filesystem error occurs.
pub fn run(args: &[String]) -> Result<(), String> {
    let mut allow_phase1 = false;
    let mut repo_root = PathBuf::from(".");

    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "--allow-phase1" | "--allow-phase-1" => {
                allow_phase1 = true;
            }
            "--allow-phase" => {
                if i + 1 < args.len() && args[i + 1] == "1" {
                    allow_phase1 = true;
                    i += 1;
                } else {
                    return Err("Expected '1' following --allow-phase".to_string());
                }
            }
            "--repo-root" | "-r" => {
                i += 1;
                if i < args.len() {
                    repo_root = PathBuf::from(&args[i]);
                } else {
                    return Err("Missing argument for --repo-root".to_string());
                }
            }
            "--help" | "-h" => {
                println!(
                    "Usage: cargo run -p xtask -- verify-pure-rust [--allow-phase1] [--repo-root <PATH>]"
                );
                return Ok(());
            }
            other => {
                return Err(format!("Unknown option '{other}' for verify-pure-rust"));
            }
        }
        i += 1;
    }

    let report = audit_pure_rust(&repo_root, allow_phase1)?;

    if report.is_clean() {
        println!("Pure-Rust audit passed: 0 non-Rust source files (allow_phase1={allow_phase1}).");
        Ok(())
    } else {
        eprintln!("Pure-Rust audit FAILED:");
        if !report.offending_files.is_empty() {
            eprintln!(
                "  Disallowed non-Rust source files ({}/{}):",
                report.offending_files.len(),
                report.offending_files.len()
            );
            for path in &report.offending_files {
                eprintln!("    - {}", path.display());
            }
        }
        if !report.bootloader_blobs.is_empty() {
            eprintln!(
                "  Disallowed legacy bootloader blobs ({}/{}):",
                report.bootloader_blobs.len(),
                report.bootloader_blobs.len()
            );
            for blob in &report.bootloader_blobs {
                eprintln!("    - {}", blob.display());
            }
        }
        if !report.disallowed_dependencies.is_empty() {
            eprintln!(
                "  Disallowed Cargo.lock dependencies ({}/{}):",
                report.disallowed_dependencies.len(),
                report.disallowed_dependencies.len()
            );
            for dep in &report.disallowed_dependencies {
                eprintln!("    - {dep}");
            }
        }

        let total_violations = report.offending_files.len()
            + report.bootloader_blobs.len()
            + report.disallowed_dependencies.len();
        Err(format!(
            "Pure-Rust audit failed with {total_violations} violations"
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_audit_flags_esp_radio_and_esp_rtos_in_phase1() {
        let unique = format!(
            "esp32_pure_rust_test_{}_{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map_or(0, |d| d.as_nanos())
        );
        let temp_dir = std::env::temp_dir().join(unique);
        fs::create_dir_all(&temp_dir).expect("create temp dir");

        let cargo_lock = temp_dir.join("Cargo.lock");
        fs::write(
            &cargo_lock,
            "[[package]]\nname = \"esp-radio\"\n\n[[package]]\nname = \"esp-rtos\"\n",
        )
        .expect("write Cargo.lock");

        let report = audit_pure_rust(&temp_dir, true).expect("audit");
        assert!(!report.is_clean());
        assert!(report
            .disallowed_dependencies
            .contains(&"esp-radio".to_string()));
        assert!(report
            .disallowed_dependencies
            .contains(&"esp-rtos".to_string()));

        let _ = fs::remove_dir_all(&temp_dir);
    }
}
