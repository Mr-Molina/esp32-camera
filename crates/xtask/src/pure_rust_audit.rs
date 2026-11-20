//! Pure-Rust codebase audit subcommand.
//!
//! Enforces zero-tolerance repository compliance:
//! - Disallows .py, .sh, .c, .cpp, .h, and .S files in the codebase (excluding
//!   .git, target, .agents).
//! - Disallows legacy Wi-Fi and C/FFI toolchain crates (`esp-radio`,
//!   `esp-rtos`, `esp-wifi`, `cc`, `cmake`, `bindgen`, `cxx`, `pkg-config`,
//!   `esp-idf-*`) in workspace/member manifests and lockfiles across all
//!   phases.
//! - In post-phase1 mode (`!allow_phase1`), also disallows
//!   `firmware/bootloader*.bin` blobs and transitional dependencies
//!   (`esp-wifi-sys*`, `esp-bootloader-esp-idf`).

use std::fs;
use std::path::{Path, PathBuf};

/// Audit report summarizing pure-Rust compliance findings.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct AuditReport {
    /// Relative paths of disallowed non-Rust source files
    /// (`.py`, `.sh`, `.c`, `.cpp`, `.h`, `.S`).
    pub offending_files: Vec<PathBuf>,
    /// Relative paths of legacy binary bootloader blobs.
    pub bootloader_blobs: Vec<PathBuf>,
    /// Disallowed dependencies found in `Cargo.lock` or `Cargo.toml` files.
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

/// Returns `true` if `crate_name` is disallowed under the requested phase.
fn is_disallowed_crate(crate_name: &str, allow_phase1: bool) -> bool {
    let always_blocked = matches!(
        crate_name,
        "esp-radio"
            | "esp-rtos"
            | "esp-wifi"
            | "cc"
            | "cmake"
            | "bindgen"
            | "cxx"
            | "cxx-build"
            | "pkg-config"
            | "esp-idf"
    ) || crate_name.starts_with("esp-idf-");

    if always_blocked {
        return true;
    }

    if !allow_phase1 {
        return crate_name == "esp-wifi-sys"
            || crate_name.starts_with("esp-wifi-sys-")
            || crate_name == "esp-bootloader-esp-idf";
    }

    false
}

/// Strips a trailing `#` comment from a TOML line outside quoted strings.
fn strip_toml_comment(line: &str) -> &str {
    let mut in_double = false;
    let mut in_single = false;
    let mut escaped = false;
    for (idx, ch) in line.char_indices() {
        if in_double {
            if escaped {
                escaped = false;
            } else if ch == '\\' {
                escaped = true;
            } else if ch == '"' {
                in_double = false;
            }
        } else if in_single {
            if ch == '\'' {
                in_single = false;
            }
        } else if ch == '"' {
            in_double = true;
        } else if ch == '\'' {
            in_single = true;
        } else if ch == '#' {
            return &line[..idx];
        }
    }
    line
}

/// Extracts a quoted TOML string literal starting at the front of `raw`.
fn parse_toml_string(raw: &str) -> Option<&str> {
    let trimmed = raw.trim();
    if let Some(rest) = trimmed.strip_prefix('"') {
        let end = rest.find('"')?;
        Some(&rest[..end])
    } else if let Some(rest) = trimmed.strip_prefix('\'') {
        let end = rest.find('\'')?;
        Some(&rest[..end])
    } else {
        None
    }
}

/// Scans a `Cargo.lock` file for disallowed package names.
fn scan_cargo_lock(
    path: &Path,
    allow_phase1: bool,
    disallowed: &mut Vec<String>,
) -> Result<(), String> {
    let content =
        fs::read_to_string(path).map_err(|e| format!("Failed to read {}: {e}", path.display()))?;
    let mut in_package = false;

    for raw_line in content.lines() {
        let line = strip_toml_comment(raw_line).trim();
        if line.is_empty() {
            continue;
        }
        if line.starts_with("[[") {
            in_package = line == "[[package]]";
            continue;
        }
        if line.starts_with('[') {
            in_package = false;
            continue;
        }
        if !in_package {
            continue;
        }
        if let Some((key, val)) = line.split_once('=') {
            if key.trim() == "name" {
                if let Some(pkg_name) = parse_toml_string(val) {
                    if is_disallowed_crate(pkg_name, allow_phase1) {
                        disallowed.push(pkg_name.to_string());
                    }
                }
            }
        }
    }

    Ok(())
}

/// Returns `true` if a TOML table header defines Cargo dependencies.
fn is_dependency_section(header: &str) -> bool {
    let inner = header.trim().trim_start_matches('[').trim_end_matches(']');
    let segs: Vec<&str> = inner.split('.').map(str::trim).collect();
    matches!(
        segs.as_slice(),
        ["dependencies" | "dev-dependencies" | "build-dependencies"]
            | ["workspace", "dependencies"]
            | [
                "target",
                _,
                "dependencies" | "dev-dependencies" | "build-dependencies"
            ]
    )
}

/// Extracts `package = "<name>"` from an inline TOML dependency table, if set.
fn extract_inline_package_name(val: &str) -> Option<&str> {
    let mut cursor = val;
    while let Some(pos) = cursor.find("package") {
        let before_ok = pos == 0
            || !cursor[..pos]
                .chars()
                .next_back()
                .is_some_and(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-');
        let after = &cursor[pos + 7..];
        let trimmed = after.trim_start();
        if before_ok {
            if let Some(after_eq) = trimmed.strip_prefix('=') {
                if let Some(pkg) = parse_toml_string(after_eq) {
                    return Some(pkg);
                }
            }
        }
        cursor = &cursor[pos + 7..];
    }
    None
}

/// Scans a `Cargo.toml` manifest for disallowed crates across dependency
/// tables (including `package = "..."` renames and subtables).
fn scan_cargo_toml(
    path: &Path,
    allow_phase1: bool,
    disallowed: &mut Vec<String>,
) -> Result<(), String> {
    let content =
        fs::read_to_string(path).map_err(|e| format!("Failed to read {}: {e}", path.display()))?;
    let mut in_dep_table = false;
    let mut in_dep_subtable = false;

    for raw_line in content.lines() {
        let line = strip_toml_comment(raw_line).trim();
        if line.is_empty() {
            continue;
        }
        if line.starts_with('[') && line.ends_with(']') && !line.starts_with("[[") {
            let inner = line.trim_start_matches('[').trim_end_matches(']').trim();
            in_dep_table = is_dependency_section(line);
            in_dep_subtable = false;

            // Handle `[dependencies.<crate>]` or `[target.*.dependencies.<crate>]`.
            if !in_dep_table {
                let parts: Vec<&str> = inner.split('.').map(str::trim).collect();
                if let Some((last, prefix)) = parts.split_last() {
                    let prefix_hdr = format!("[{}]", prefix.join("."));
                    if is_dependency_section(&prefix_hdr) {
                        in_dep_subtable = true;
                        let dep_key = last.trim_matches('"').trim_matches('\'');
                        if is_disallowed_crate(dep_key, allow_phase1) {
                            disallowed.push(dep_key.to_string());
                        }
                    }
                }
            }
            continue;
        }

        if let Some((raw_key, raw_val)) = line.split_once('=') {
            let key = raw_key.trim().trim_matches('"').trim_matches('\'');
            if in_dep_table {
                if is_disallowed_crate(key, allow_phase1) {
                    disallowed.push(key.to_string());
                }
                if let Some(renamed) = extract_inline_package_name(raw_val) {
                    if is_disallowed_crate(renamed, allow_phase1) {
                        disallowed.push(renamed.to_string());
                    }
                }
            } else if in_dep_subtable && key == "package" {
                if let Some(renamed) = parse_toml_string(raw_val) {
                    if is_disallowed_crate(renamed, allow_phase1) {
                        disallowed.push(renamed.to_string());
                    }
                }
            }
        }
    }

    Ok(())
}

/// Recursively walks `dir`, collecting non-Rust source files, bootloader
/// blobs, and Cargo manifest/lockfile paths.
fn walk_dir(
    dir: &Path,
    repo_root: &Path,
    allow_phase1: bool,
    offending: &mut Vec<PathBuf>,
    bootloaders: &mut Vec<PathBuf>,
    cargo_locks: &mut Vec<PathBuf>,
    cargo_tomls: &mut Vec<PathBuf>,
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
            // Skip .git, target, and .agents directories.
            if name_str == ".git" || name_str == "target" || name_str == ".agents" {
                continue;
            }
            walk_dir(
                &path,
                repo_root,
                allow_phase1,
                offending,
                bootloaders,
                cargo_locks,
                cargo_tomls,
            )?;
        } else if path.is_file() {
            let rel_path = path.strip_prefix(repo_root).unwrap_or(&path).to_path_buf();

            if name_str == "Cargo.lock" {
                cargo_locks.push(path.clone());
            } else if name_str == "Cargo.toml" {
                cargo_tomls.push(path.clone());
            }

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
/// Canonicalizes `repo_root` and inspects all discovered `Cargo.toml` and
/// `Cargo.lock` files across the workspace tree.
///
/// # Errors
/// Returns an error message if `repo_root` cannot be canonicalized, is not a
/// directory, or filesystem traversal / manifest reading fails.
pub fn audit_pure_rust(repo_root: &Path, allow_phase1: bool) -> Result<AuditReport, String> {
    let canonical_root = fs::canonicalize(repo_root).map_err(|e| {
        format!(
            "Failed to canonicalize repo root '{}': {e}",
            repo_root.display()
        )
    })?;
    if !canonical_root.is_dir() {
        return Err(format!(
            "Repository root '{}' is not a directory",
            canonical_root.display()
        ));
    }

    let mut offending_files = Vec::new();
    let mut bootloader_blobs = Vec::new();
    let mut disallowed_dependencies = Vec::new();
    let mut cargo_locks = Vec::new();
    let mut cargo_tomls = Vec::new();

    walk_dir(
        &canonical_root,
        &canonical_root,
        allow_phase1,
        &mut offending_files,
        &mut bootloader_blobs,
        &mut cargo_locks,
        &mut cargo_tomls,
    )?;

    cargo_locks.sort();
    cargo_tomls.sort();

    for lock_path in &cargo_locks {
        scan_cargo_lock(lock_path, allow_phase1, &mut disallowed_dependencies)?;
    }
    for toml_path in &cargo_tomls {
        scan_cargo_toml(toml_path, allow_phase1, &mut disallowed_dependencies)?;
    }

    offending_files.sort();
    bootloader_blobs.sort();
    disallowed_dependencies.sort();
    disallowed_dependencies.dedup();

    Ok(AuditReport {
        offending_files,
        bootloader_blobs,
        disallowed_dependencies,
    })
}

/// Strictly audits a workspace root, failing closed if root `Cargo.toml` or
/// `Cargo.lock` is missing.
///
/// # Errors
/// Returns an error if `repo_root` is invalid, lacks root `Cargo.toml` or
/// `Cargo.lock`, or fails filesystem traversal.
pub fn audit_pure_rust_workspace(
    repo_root: &Path,
    allow_phase1: bool,
) -> Result<AuditReport, String> {
    let canonical_root = fs::canonicalize(repo_root).map_err(|e| {
        format!(
            "Failed to canonicalize repo root '{}': {e}",
            repo_root.display()
        )
    })?;
    if !canonical_root.is_dir() {
        return Err(format!(
            "Repository root '{}' is not a directory",
            canonical_root.display()
        ));
    }
    if !canonical_root.join("Cargo.toml").is_file() {
        return Err(format!(
            "Missing required workspace Cargo.toml at '{}'",
            canonical_root.display()
        ));
    }
    if !canonical_root.join("Cargo.lock").is_file() {
        return Err(format!(
            "Missing required Cargo.lock at '{}' (fail-closed pure-Rust audit)",
            canonical_root.display()
        ));
    }

    audit_pure_rust(&canonical_root, allow_phase1)
}

/// Runs the `verify-pure-rust` CLI subcommand.
///
/// # Errors
/// Returns an error if the audit fails (violations found), root manifests are
/// missing, or a filesystem error occurs.
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
                    "Usage: cargo run -p xtask -- verify-pure-rust \
                     [--allow-phase1] [--repo-root <PATH>]"
                );
                return Ok(());
            }
            other => {
                return Err(format!("Unknown option '{other}' for verify-pure-rust"));
            }
        }
        i += 1;
    }

    if allow_phase1 {
        println!(
            "[PHASE-1 MODE] Running verify-pure-rust with --allow-phase1 \
             (transitional exceptions active for esp-wifi-sys* and \
             esp-bootloader-esp-idf; NOT strict pure-Rust)."
        );
    }

    let report = audit_pure_rust_workspace(&repo_root, allow_phase1)?;

    if report.is_clean() {
        if allow_phase1 {
            println!(
                "Pure-Rust audit passed (PHASE-1 transitional mode): \
                 0 disallowed files, 0 blocked crates."
            );
        } else {
            println!("Pure-Rust audit passed (STRICT mode): 0 violations.");
        }
        Ok(())
    } else {
        eprintln!("Pure-Rust audit FAILED:");
        if !report.offending_files.is_empty() {
            eprintln!(
                "  Disallowed non-Rust source files ({}):",
                report.offending_files.len()
            );
            for path in &report.offending_files {
                eprintln!("    - {}", path.display());
            }
        }
        if !report.bootloader_blobs.is_empty() {
            eprintln!(
                "  Disallowed legacy bootloader blobs ({}):",
                report.bootloader_blobs.len()
            );
            for blob in &report.bootloader_blobs {
                eprintln!("    - {}", blob.display());
            }
        }
        if !report.disallowed_dependencies.is_empty() {
            eprintln!(
                "  Disallowed Cargo dependencies ({}):",
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

    fn unique_temp_dir(label: &str) -> PathBuf {
        let unique = format!(
            "esp32_pure_rust_{label}_{}_{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map_or(0, |d| d.as_nanos())
        );
        let dir = std::env::temp_dir().join(unique);
        fs::create_dir_all(&dir).expect("create temp dir");
        dir
    }

    #[test]
    fn test_audit_flags_esp_radio_and_esp_rtos_in_phase1() {
        let temp_dir = unique_temp_dir("legacy_wifi");
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

    #[test]
    fn test_audit_workspace_fails_closed_when_manifest_or_lock_missing() {
        let empty_dir = unique_temp_dir("missing_files");
        let err = audit_pure_rust_workspace(&empty_dir, true).expect_err("must fail closed");
        assert!(err.contains("Cargo.toml"), "got: {err}");

        fs::write(empty_dir.join("Cargo.toml"), "[package]\nname = \"x\"\n")
            .expect("write Cargo.toml");
        let err2 = audit_pure_rust_workspace(&empty_dir, true).expect_err("must fail closed");
        assert!(err2.contains("Cargo.lock"), "got: {err2}");

        let _ = fs::remove_dir_all(&empty_dir);
    }

    #[test]
    fn test_audit_flags_c_ffi_crates_in_manifests_and_lockfiles() {
        let temp_dir = unique_temp_dir("c_ffi");
        fs::create_dir_all(temp_dir.join("crates").join("member")).expect("mkdir member");
        fs::write(
            temp_dir.join("Cargo.toml"),
            "[workspace]\nmembers = [\"crates/member\"]\n",
        )
        .expect("write root Cargo.toml");
        fs::write(
            temp_dir.join("crates").join("member").join("Cargo.toml"),
            "[package]\nname = \"member\"\n# comment mentioning bindgen and cc\n\
             description = \"Mentions pkg-config safely\"\n\
             [build-dependencies]\ncc = \"1.0\"\n\
             my_cmake = { package = \"cmake\", version = \"0.1\" }\n\
             [dependencies]\nesp-idf-sys = \"0.35\"\n",
        )
        .expect("write member Cargo.toml");
        fs::write(
            temp_dir.join("Cargo.lock"),
            "[[package]]\nname = \"bindgen\"\n\n[[package]]\nname = \"cxx\"\n\
             \n[[package]]\nname = \"pkg-config\"\n",
        )
        .expect("write Cargo.lock");

        let report = audit_pure_rust_workspace(&temp_dir, true).expect("audit");
        for expected in ["bindgen", "cc", "cmake", "cxx", "esp-idf-sys", "pkg-config"] {
            assert!(
                report
                    .disallowed_dependencies
                    .contains(&expected.to_string()),
                "missing {expected} in {:?}",
                report.disallowed_dependencies
            );
        }

        let _ = fs::remove_dir_all(&temp_dir);
    }
}
