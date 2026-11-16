//! Build script for the `esp-wifi-sys-esp32` seam crate.
//!
//! Copies upstream generated binding sources (`c_types.rs`, `fmt.rs`,
//! `include.rs`) into `OUT_DIR` and, when targeting `xtensa`, copies and
//! links only the retained PHY static archive (`phy`).

use std::env;
use std::error::Error;
use std::ffi::OsString;
use std::fs;
use std::io;
use std::path::{Path, PathBuf};

const UPSTREAM_CRATE_DIR_NAME: &str = "esp-wifi-sys-esp32-0.3.0";
const UPSTREAM_SRC_FILES: [&str; 3] = ["c_types.rs", "fmt.rs", "include.rs"];
const RETAINED_XTENSA_ARCHIVES: [&str; 1] = ["phy"];

fn non_empty_env_path(key: &str) -> Option<PathBuf> {
    let value: OsString = env::var_os(key)?;
    if value.is_empty() {
        return None;
    }
    Some(PathBuf::from(value))
}

fn is_valid_upstream_dir(dir: &Path) -> bool {
    dir.join("src").join("include.rs").is_file()
}

fn cargo_registry_src_dir() -> Option<PathBuf> {
    let cargo_home = if let Some(dir) = non_empty_env_path("CARGO_HOME") {
        dir
    } else {
        non_empty_env_path("HOME")?.join(".cargo")
    };
    Some(cargo_home.join("registry").join("src"))
}

fn locate_upstream_dir() -> Result<PathBuf, Box<dyn Error>> {
    if let Some(override_dir) = non_empty_env_path("ESP_WIFI_SYS_ESP32_UPSTREAM_DIR") {
        if is_valid_upstream_dir(&override_dir) {
            return Ok(override_dir);
        }
        return Err(format!(
            "ESP_WIFI_SYS_ESP32_UPSTREAM_DIR ({}) does not contain src/include.rs",
            override_dir.display()
        )
        .into());
    }

    if let Some(registry_src) = cargo_registry_src_dir() {
        if registry_src.is_dir() {
            let mut candidates = Vec::new();
            let entries = fs::read_dir(&registry_src).map_err(|err| {
                format!(
                    "failed to read Cargo registry directory {}: {err}",
                    registry_src.display()
                )
            })?;
            for entry in entries {
                let Ok(entry) = entry else {
                    continue;
                };
                let candidate = entry.path().join(UPSTREAM_CRATE_DIR_NAME);
                if is_valid_upstream_dir(&candidate) {
                    candidates.push(candidate);
                }
            }
            candidates.sort();
            if let Some(found) = candidates.into_iter().next() {
                return Ok(found);
            }
        }
    }

    Err(io::Error::new(
        io::ErrorKind::NotFound,
        format!(
            "could not locate upstream {UPSTREAM_CRATE_DIR_NAME} (missing src/include.rs); \
             run `cargo fetch` or set ESP_WIFI_SYS_ESP32_UPSTREAM_DIR"
        ),
    )
    .into())
}

fn copy_upstream_sources(upstream_dir: &Path, out_dir: &Path) -> Result<(), Box<dyn Error>> {
    for file_name in UPSTREAM_SRC_FILES {
        let src = upstream_dir.join("src").join(file_name);
        let dst = out_dir.join(file_name);
        println!("cargo:rerun-if-changed={}", src.display());
        fs::copy(&src, &dst).map_err(|err| {
            format!(
                "failed to copy upstream source {} to {}: {err}",
                src.display(),
                dst.display()
            )
        })?;
    }
    Ok(())
}

fn link_retained_xtensa_archives(
    upstream_dir: &Path,
    out_dir: &Path,
) -> Result<(), Box<dyn Error>> {
    // Emit OUT_DIR as the sole native search path so retired archives in
    // upstream libs/ are never visible to the linker.
    println!("cargo:rustc-link-search=native={}", out_dir.display());
    for lib in RETAINED_XTENSA_ARCHIVES {
        let archive_name = format!("lib{lib}.a");
        let src = upstream_dir.join("libs").join(&archive_name);
        let dst = out_dir.join(&archive_name);
        println!("cargo:rerun-if-changed={}", src.display());
        fs::copy(&src, &dst).map_err(|err| {
            format!(
                "failed to copy retained archive {} to {}: {err}",
                src.display(),
                dst.display()
            )
        })?;
        println!("cargo:rustc-link-lib=static={lib}");
    }
    Ok(())
}

/// Writes `OUT_DIR/upstream_mods.rs` with `#[path]` attributes for the
/// copied upstream sources so lint suppressions apply cleanly to bindgen code.
fn write_upstream_mods_file(out_dir: &Path) -> Result<(), Box<dyn Error>> {
    let c_types = out_dir.join("c_types.rs");
    let fmt_file = out_dir.join("fmt.rs");
    let include = out_dir.join("include.rs");
    let mods_path = out_dir.join("upstream_mods.rs");

    let c_types_str = c_types
        .to_str()
        .ok_or_else(|| format!("non-UTF-8 path: {}", c_types.display()))?;
    let fmt_str = fmt_file
        .to_str()
        .ok_or_else(|| format!("non-UTF-8 path: {}", fmt_file.display()))?;
    let include_str = include
        .to_str()
        .ok_or_else(|| format!("non-UTF-8 path: {}", include.display()))?;

    let contents = format!(
        concat!(
            "/// Upstream C ABI primitive type aliases from `esp-wifi-sys-esp32`.\n",
            "#[path = {c_types:?}]\n",
            "pub mod c_types;\n\n",
            "#[path = {fmt_file:?}]\n",
            "pub(crate) mod fmt;\n\n",
            "/// Upstream bindgen-generated ESP-IDF Wi-Fi/PHY FFI declarations.\n",
            "#[allow(\n",
            "    unknown_lints,\n",
            "    suspicious_runtime_symbol_definitions,\n",
            "    clippy::all,\n",
            "    clippy::undocumented_unsafe_blocks,\n",
            "    unsafe_op_in_unsafe_fn\n",
            ")]\n",
            "#[path = {include:?}]\n",
            "pub mod include;\n",
        ),
        c_types = c_types_str,
        fmt_file = fmt_str,
        include = include_str,
    );

    fs::write(&mods_path, contents).map_err(|err| {
        format!(
            "failed to write upstream module manifest {}: {err}",
            mods_path.display()
        )
    })?;
    Ok(())
}

fn main() -> Result<(), Box<dyn Error>> {
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-env-changed=ESP_WIFI_SYS_ESP32_UPSTREAM_DIR");
    println!("cargo:rerun-if-env-changed=CARGO_HOME");
    println!("cargo:rerun-if-env-changed=HOME");
    println!("cargo:rerun-if-env-changed=CARGO_CFG_TARGET_ARCH");

    let out_dir =
        non_empty_env_path("OUT_DIR").ok_or("OUT_DIR environment variable must be set by Cargo")?;
    let upstream_dir = locate_upstream_dir()?;

    copy_upstream_sources(&upstream_dir, &out_dir)?;
    write_upstream_mods_file(&out_dir)?;

    if env::var("CARGO_CFG_TARGET_ARCH").as_deref() == Ok("xtensa") {
        link_retained_xtensa_archives(&upstream_dir, &out_dir)?;
    }

    Ok(())
}
