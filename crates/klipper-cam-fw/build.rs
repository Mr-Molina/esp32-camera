//! Build-time secret gate for `klipper-cam-fw` (`BUILD-07`, `FW-NET-03`,
//! `FW-NET-04`).
//!
//! An Xtensa release build must not ship the public default OTA token or an
//! empty Wi-Fi passphrase, so it fails here when they are missing. Host
//! builds (`cargo test`, clippy) and debug builds are never affected.
//!
//! The policy lives in [`check_release_secrets`] so the host test
//! `tests/wp09_build_policy.rs` can exercise it without a toolchain.

use std::env;

/// Copy of `config::DEFAULT_OTA_TOKEN` (a build script cannot depend on its
/// own crate); `tests/wp09_build_policy.rs` keeps the two in sync.
pub const DEFAULT_OTA_TOKEN: &str = "klipper-cam-ota-2026";

/// Minimum trimmed byte length for a private OTA token on Xtensa release
/// builds.
pub const MIN_OTA_TOKEN_LEN: usize = 16;

/// OTA token variables in the exact 1:1 precedence order of
/// `config::active_ota_token` (first non-empty wins).
pub const OTA_TOKEN_VARS: [&str; 4] = [
    "CAM_OTA_TOKEN",
    "OTA_TOKEN",
    "WIFI_OTA_TOKEN",
    "ESP32_CAM_OTA_TOKEN",
];

/// Every variable read through `option_env!` in `config.rs` or OTA deploy
/// tooling.
pub const WATCHED_ENV_VARS: [&str; 6] = [
    "WIFI_PASS",
    "WIFI_OTA_TOKEN",
    "CAM_OTA_TOKEN",
    "OTA_TOKEN",
    "ESP32_CAM_OTA_TOKEN",
    "WIFI_SSID",
];

/// Returns `Err` with a human-readable reason when an Xtensa release build
/// lacks a usable OTA token or Wi-Fi passphrase.
///
/// Forwards to [`check_release_secrets_with_mode`] with
/// `allow_provisioning_ap = false` for backward compatibility.
///
/// - `target_arch` is `CARGO_CFG_TARGET_ARCH`, `profile` is `PROFILE`.
/// - `allow_open_wifi` mirrors the `allow-open-wifi` feature, which permits
///   an empty `WIFI_PASS`.
/// - `var` looks up an environment variable.
pub fn check_release_secrets(
    target_arch: &str,
    profile: &str,
    allow_open_wifi: bool,
    var: &dyn Fn(&str) -> Option<String>,
) -> Result<(), String> {
    check_release_secrets_with_mode(target_arch, profile, allow_open_wifi, false, var)
}

/// Returns `Err` with a human-readable reason when an Xtensa release build
/// lacks a usable OTA token or Wi-Fi passphrase, accounting for build features.
///
/// - `target_arch` is `CARGO_CFG_TARGET_ARCH`, `profile` is `PROFILE`.
/// - `allow_open_wifi` mirrors the `allow-open-wifi` feature, which permits
///   an empty `WIFI_PASS`.
/// - `allow_provisioning_ap` mirrors the `provisioning-ap` feature, which
///   permits unset/empty `WIFI_PASS` and `WIFI_OTA_TOKEN` at compile time
///   while still strictly rejecting any invalid non-empty value.
/// - `var` looks up an environment variable.
pub fn check_release_secrets_with_mode(
    target_arch: &str,
    profile: &str,
    allow_open_wifi: bool,
    allow_provisioning_ap: bool,
    var: &dyn Fn(&str) -> Option<String>,
) -> Result<(), String> {
    if target_arch != "xtensa" || matches!(profile, "debug" | "test") {
        return Ok(());
    }

    let token = OTA_TOKEN_VARS
        .iter()
        .find_map(|name| var(name).filter(|v| !v.is_empty()).map(|v| (*name, v)));
    match token {
        None => {
            if !allow_provisioning_ap {
                return Err(String::from(
                    "WIFI_OTA_TOKEN is unset or empty; an Xtensa release build needs a private \
                     OTA token (see README, `.env.local`), or enable the `provisioning-ap` \
                     feature",
                ));
            }
        }
        Some((name, value)) => {
            if !value.bytes().all(|b| (0x21..=0x7E).contains(&b)) {
                return Err(format!(
                    "{name} contains whitespace or non-graphic ASCII characters; \
                     an Xtensa release OTA token must use printable ASCII 0x21..=0x7E \
                     without spaces"
                ));
            }
            if value == DEFAULT_OTA_TOKEN {
                return Err(format!(
                    "{name} is the public default OTA token; set a private WIFI_OTA_TOKEN"
                ));
            }
            if value.len() < MIN_OTA_TOKEN_LEN {
                return Err(format!(
                    "{name} is {} bytes; an Xtensa release OTA token must be \
                     at least {MIN_OTA_TOKEN_LEN} bytes",
                    value.len()
                ));
            }
        }
    }

    let pass = var("WIFI_PASS").unwrap_or_default();
    if pass.is_empty() {
        if !allow_open_wifi && !allow_provisioning_ap {
            return Err(String::from(
                "WIFI_PASS is unset or empty; set it, or enable the `allow-open-wifi` or \
                 `provisioning-ap` feature",
            ));
        }
    } else if !(8..=63).contains(&pass.len()) {
        return Err(format!(
            "WIFI_PASS is {} bytes; WPA2-PSK needs 8..=63",
            pass.len()
        ));
    }
    Ok(())
}

fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    for name in WATCHED_ENV_VARS {
        println!("cargo:rerun-if-env-changed={name}");
    }

    let target_arch = env::var("CARGO_CFG_TARGET_ARCH").unwrap_or_default();
    let profile = env::var("PROFILE").unwrap_or_default();
    let allow_open_wifi = env::var_os("CARGO_FEATURE_ALLOW_OPEN_WIFI").is_some();
    let allow_provisioning_ap = env::var_os("CARGO_FEATURE_PROVISIONING_AP").is_some();
    let lookup = |name: &str| env::var(name).ok();

    if let Err(reason) = check_release_secrets_with_mode(
        &target_arch,
        &profile,
        allow_open_wifi,
        allow_provisioning_ap,
        &lookup,
    ) {
        panic!("klipper-cam-fw release build refused: {reason}");
    }
}
