//! Build script for the `esp-wifi-sys-esp32` seam crate.
//!
//! Copies upstream generated binding sources (`c_types.rs`, `fmt.rs`,
//! `include.rs`) into `OUT_DIR` and, when targeting `xtensa`, copies and
//! links only the retained PHY static archive (`phy`).

use std::env;
use std::error::Error;
use std::ffi::OsString;
use std::fmt::Write as _;
use std::fs;
use std::io;
use std::path::{Path, PathBuf};

const UPSTREAM_CRATE_DIR_NAME: &str = "esp-wifi-sys-esp32-0.3.0";

/// Upstream source files copied from `esp-wifi-sys-esp32 0.3.0`, paired with
/// their pinned SHA-256 digests from the crates.io package
/// (`c4644d49bde1bdeb3cd20fcc8c08be09f8f67af4b80123e54f665fdebd9a3358`).
const UPSTREAM_SRC_FILES: [(&str, &str); 3] = [
    (
        "c_types.rs",
        "09e1e3a90802fa9833a1951a3b47a6ff6d5ec2096de568dafbbb4071fc054417",
    ),
    (
        "fmt.rs",
        "9b78319a4f971023dfa2e1f155f813ef261f977ce0634f7444c59178622f0761",
    ),
    (
        "include.rs",
        "b305fe3f39ffafea455d895d29d0bf86e2acd44780365d32a59325be650da7f6",
    ),
];

/// Retained Xtensa static archives paired with their pinned SHA-256 digests.
const RETAINED_XTENSA_ARCHIVES: [(&str, &str); 1] = [(
    "phy",
    "c2521071695734afa8f6f1ca3d3b604a1d08dd63fe52060449a11cfe8bac5740",
)];

const SHA256_H0: [u32; 8] = [
    0x6a09_e667,
    0xbb67_ae85,
    0x3c6e_f372,
    0xa54f_f53a,
    0x510e_527f,
    0x9b05_688c,
    0x1f83_d9ab,
    0x5be0_cd19,
];

const SHA256_K: [u32; 64] = [
    0x428a_2f98,
    0x7137_4491,
    0xb5c0_fbcf,
    0xe9b5_dba5,
    0x3956_c25b,
    0x59f1_11f1,
    0x923f_82a4,
    0xab1c_5ed5,
    0xd807_aa98,
    0x1283_5b01,
    0x2431_85be,
    0x550c_7dc3,
    0x72be_5d74,
    0x80de_b1fe,
    0x9bdc_06a7,
    0xc19b_f174,
    0xe49b_69c1,
    0xefbe_4786,
    0x0fc1_9dc6,
    0x240c_a1cc,
    0x2de9_2c6f,
    0x4a74_84aa,
    0x5cb0_a9dc,
    0x76f9_88da,
    0x983e_5152,
    0xa831_c66d,
    0xb003_27c8,
    0xbf59_7fc7,
    0xc6e0_0bf3,
    0xd5a7_9147,
    0x06ca_6351,
    0x1429_2967,
    0x27b7_0a85,
    0x2e1b_2138,
    0x4d2c_6dfc,
    0x5338_0d13,
    0x650a_7354,
    0x766a_0abb,
    0x81c2_c92e,
    0x9272_2c85,
    0xa2bf_e8a1,
    0xa81a_664b,
    0xc24b_8b70,
    0xc76c_51a3,
    0xd192_e819,
    0xd699_0624,
    0xf40e_3585,
    0x106a_a070,
    0x19a4_c116,
    0x1e37_6c08,
    0x2748_774c,
    0x34b0_bcb5,
    0x391c_0cb3,
    0x4ed8_aa4a,
    0x5b9c_ca4f,
    0x682e_6ff3,
    0x748f_82ee,
    0x78a5_636f,
    0x84c8_7814,
    0x8cc7_0208,
    0x90be_fffa,
    0xa450_6ceb,
    0xbef9_a3f7,
    0xc671_78f2,
];

fn sha256_compress(state: &mut [u32; 8], block: &[u8; 64]) {
    let mut w = [0u32; 64];
    for (i, chunk) in block.as_chunks::<4>().0.iter().take(16).enumerate() {
        w[i] = u32::from_be_bytes(*chunk);
    }
    for i in 16..64 {
        let s0 = w[i - 15].rotate_right(7) ^ w[i - 15].rotate_right(18) ^ (w[i - 15] >> 3);
        let s1 = w[i - 2].rotate_right(17) ^ w[i - 2].rotate_right(19) ^ (w[i - 2] >> 10);
        w[i] = w[i - 16]
            .wrapping_add(s0)
            .wrapping_add(w[i - 7])
            .wrapping_add(s1);
    }

    let [mut a, mut b, mut c, mut d, mut e, mut f, mut g, mut h] = *state;
    for i in 0..64 {
        let s1 = e.rotate_right(6) ^ e.rotate_right(11) ^ e.rotate_right(25);
        let ch = (e & f) ^ ((!e) & g);
        let temp1 = h
            .wrapping_add(s1)
            .wrapping_add(ch)
            .wrapping_add(SHA256_K[i])
            .wrapping_add(w[i]);
        let s0 = a.rotate_right(2) ^ a.rotate_right(13) ^ a.rotate_right(22);
        let maj = (a & b) ^ (a & c) ^ (b & c);
        let temp2 = s0.wrapping_add(maj);

        h = g;
        g = f;
        f = e;
        e = d.wrapping_add(temp1);
        d = c;
        c = b;
        b = a;
        a = temp1.wrapping_add(temp2);
    }

    state[0] = state[0].wrapping_add(a);
    state[1] = state[1].wrapping_add(b);
    state[2] = state[2].wrapping_add(c);
    state[3] = state[3].wrapping_add(d);
    state[4] = state[4].wrapping_add(e);
    state[5] = state[5].wrapping_add(f);
    state[6] = state[6].wrapping_add(g);
    state[7] = state[7].wrapping_add(h);
}

fn sha256_hex(data: &[u8]) -> String {
    let mut state = SHA256_H0;
    let (blocks, rem) = data.as_chunks::<64>();
    for block in blocks {
        sha256_compress(&mut state, block);
    }

    let mut tail = [0u8; 128];
    tail[..rem.len()].copy_from_slice(rem);
    tail[rem.len()] = 0x80;

    let total_bits = (data.len() as u64).wrapping_mul(8);
    let tail_len = if rem.len() < 56 { 64 } else { 128 };
    tail[tail_len - 8..tail_len].copy_from_slice(&total_bits.to_be_bytes());

    for block in tail[..tail_len].as_chunks::<64>().0 {
        sha256_compress(&mut state, block);
    }

    let mut hex = String::with_capacity(64);
    for word in state {
        let _ = write!(&mut hex, "{word:08x}");
    }
    hex
}

fn copy_verified_file(src: &Path, dst: &Path, expected_sha256: &str) -> Result<(), Box<dyn Error>> {
    let bytes = fs::read(src)
        .map_err(|err| format!("failed to read upstream file {}: {err}", src.display()))?;
    let actual_sha256 = sha256_hex(&bytes);
    if actual_sha256 != expected_sha256 {
        return Err(format!(
            "SHA-256 mismatch for {}: expected {expected_sha256}, got {actual_sha256}",
            src.display()
        )
        .into());
    }
    fs::write(dst, &bytes).map_err(|err| {
        format!(
            "failed to write verified upstream file {} to {}: {err}",
            src.display(),
            dst.display()
        )
    })?;
    Ok(())
}

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
        non_empty_env_path("HOME")
            .or_else(|| non_empty_env_path("USERPROFILE"))?
            .join(".cargo")
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
            let mut crates_io_candidates = Vec::new();
            let mut other_candidates = Vec::new();
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
                    let is_crates_io = entry
                        .file_name()
                        .to_str()
                        .is_some_and(|name| name.starts_with("index.crates.io-"));
                    if is_crates_io {
                        crates_io_candidates.push(candidate);
                    } else {
                        other_candidates.push(candidate);
                    }
                }
            }
            crates_io_candidates.sort();
            if let Some(found) = crates_io_candidates.into_iter().next() {
                return Ok(found);
            }
            other_candidates.sort();
            if let Some(found) = other_candidates.into_iter().next() {
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
    for (file_name, expected_sha256) in UPSTREAM_SRC_FILES {
        let src = upstream_dir.join("src").join(file_name);
        let dst = out_dir.join(file_name);
        println!("cargo:rerun-if-changed={}", src.display());
        copy_verified_file(&src, &dst, expected_sha256)?;
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
    for (lib, expected_sha256) in RETAINED_XTENSA_ARCHIVES {
        let archive_name = format!("lib{lib}.a");
        let src = upstream_dir.join("libs").join(&archive_name);
        let dst = out_dir.join(&archive_name);
        println!("cargo:rerun-if-changed={}", src.display());
        copy_verified_file(&src, &dst, expected_sha256)?;
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
    println!("cargo:rerun-if-env-changed=USERPROFILE");
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
