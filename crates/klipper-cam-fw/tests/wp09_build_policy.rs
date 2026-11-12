//! WP-09 / BUILD-07: host tests for the `build.rs` release secret gate.
//!
//! The build script is compiled into this test as a module so its policy
//! function runs on the host without an Xtensa toolchain.

#![forbid(unsafe_code)]

#[path = "../build.rs"]
#[allow(dead_code)]
mod build_script;

use build_script::check_release_secrets;

fn env_of<'a>(pairs: &'a [(&'a str, &'a str)]) -> impl Fn(&str) -> Option<String> + 'a {
    move |key| {
        pairs
            .iter()
            .find(|(k, _)| *k == key)
            .map(|(_, v)| (*v).to_string())
    }
}

const GOOD: &[(&str, &str)] = &[
    ("WIFI_PASS", "correct-horse-battery"),
    ("WIFI_OTA_TOKEN", "a-long-random-secret"),
];

#[test]
fn host_and_debug_builds_need_no_secrets() {
    let none = env_of(&[]);
    assert_eq!(
        check_release_secrets("x86_64", "debug", false, &none),
        Ok(())
    );
    assert_eq!(
        check_release_secrets("x86_64", "release", false, &none),
        Ok(())
    );
    assert_eq!(
        check_release_secrets("aarch64", "release", false, &none),
        Ok(())
    );
    assert_eq!(
        check_release_secrets("xtensa", "debug", false, &none),
        Ok(())
    );
}

#[test]
fn xtensa_release_with_secrets_passes() {
    assert_eq!(
        check_release_secrets("xtensa", "release", false, &env_of(GOOD)),
        Ok(())
    );
}

#[test]
fn xtensa_release_without_token_fails() {
    let env = env_of(&[("WIFI_PASS", "correct-horse-battery")]);
    let err = check_release_secrets("xtensa", "release", false, &env).expect_err("must fail");
    assert!(err.contains("WIFI_OTA_TOKEN"), "{err}");

    let empty = env_of(&[
        ("WIFI_PASS", "correct-horse-battery"),
        ("WIFI_OTA_TOKEN", ""),
    ]);
    assert!(check_release_secrets("xtensa", "release", false, &empty).is_err());
}

#[test]
fn xtensa_release_with_default_token_fails() {
    let env = env_of(&[
        ("WIFI_PASS", "correct-horse-battery"),
        ("WIFI_OTA_TOKEN", klipper_cam_fw::DEFAULT_OTA_TOKEN),
    ]);
    let err = check_release_secrets("xtensa", "release", false, &env).expect_err("must fail");
    assert!(err.contains("default"), "{err}");
}

#[test]
fn token_aliases_follow_firmware_precedence() {
    // `CAM_OTA_TOKEN` and `OTA_TOKEN` override `WIFI_OTA_TOKEN` in
    // `config::active_ota_token`, so the gate checks the same winner.
    let alias = env_of(&[
        ("WIFI_PASS", "correct-horse-battery"),
        ("CAM_OTA_TOKEN", "a-long-random-secret"),
    ]);
    assert_eq!(
        check_release_secrets("xtensa", "release", false, &alias),
        Ok(())
    );
    let shadowed = env_of(&[
        ("WIFI_PASS", "correct-horse-battery"),
        ("CAM_OTA_TOKEN", klipper_cam_fw::DEFAULT_OTA_TOKEN),
        ("WIFI_OTA_TOKEN", "a-long-random-secret"),
    ]);
    assert!(check_release_secrets("xtensa", "release", false, &shadowed).is_err());
}

#[test]
fn xtensa_release_without_wifi_pass_fails_unless_open_allowed() {
    let env = env_of(&[("WIFI_OTA_TOKEN", "a-long-random-secret")]);
    let err = check_release_secrets("xtensa", "release", false, &env).expect_err("must fail");
    assert!(err.contains("WIFI_PASS"), "{err}");

    let empty = env_of(&[
        ("WIFI_PASS", ""),
        ("WIFI_OTA_TOKEN", "a-long-random-secret"),
    ]);
    assert!(check_release_secrets("xtensa", "release", false, &empty).is_err());
    assert_eq!(
        check_release_secrets("xtensa", "release", true, &empty),
        Ok(())
    );
}

#[test]
fn xtensa_release_rejects_bad_passphrase_length() {
    let long = "p".repeat(70);
    let env_long = [("WIFI_PASS", long.as_str()), ("WIFI_OTA_TOKEN", "secret-x")];
    assert!(check_release_secrets("xtensa", "release", true, &env_of(&env_long)).is_err());
    let env_short = [("WIFI_PASS", "1234567"), ("WIFI_OTA_TOKEN", "secret-x")];
    assert!(check_release_secrets("xtensa", "release", true, &env_of(&env_short)).is_err());
}

#[test]
fn default_token_copy_matches_firmware() {
    assert_eq!(
        build_script::DEFAULT_OTA_TOKEN,
        klipper_cam_fw::DEFAULT_OTA_TOKEN
    );
}
