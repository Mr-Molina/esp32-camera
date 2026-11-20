//! WP-09 / BUILD-07: host tests for the `build.rs` release secret gate.
//!
//! The build script is compiled into this test as a module so its policy
//! function runs on the host without an Xtensa toolchain.

#![forbid(unsafe_code)]

#[path = "../build.rs"]
#[allow(dead_code)]
mod build_script;

use build_script::{check_release_secrets, check_release_secrets_with_mode};

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
    assert_eq!(
        build_script::OTA_TOKEN_VARS,
        klipper_cam_fw::config::OTA_TOKEN_VARS,
        "build.rs and config::active_ota_token must check the exact same variables in order"
    );
}

#[test]
fn m027_rejects_whitespace_and_non_graphic_ascii_tokens() {
    for bad_token in [
        "  valid-16-byte-tk  ",
        " valid-16-byte-tk",
        "valid-16-byte-tk ",
        "valid 16 byte tk!",
        "                ",
        "\tvalid-16-byte-tk",
        "valid-16-byte-tk\n",
        "valid-16-byte-\u{00A0}k",
    ] {
        let pairs = [
            ("WIFI_PASS", "correct-horse-battery"),
            ("WIFI_OTA_TOKEN", bad_token),
        ];
        let env = env_of(&pairs);
        assert!(
            check_release_secrets("xtensa", "release", false, &env).is_err(),
            "build.rs must reject whitespace/non-graphic token: {bad_token:?}"
        );
    }

    // ESP32_CAM_OTA_TOKEN is in both build.rs::OTA_TOKEN_VARS and
    // config::active_ota_token() (1:1 parity).
    let alias = env_of(&[
        ("WIFI_PASS", "correct-horse-battery"),
        ("ESP32_CAM_OTA_TOKEN", "my-20-char-secret-tk"),
    ]);
    assert_eq!(
        check_release_secrets("xtensa", "release", false, &alias),
        Ok(())
    );
}

/// The CI xtensa job supplies placeholder credentials so this gate lets the
/// size-gate build through. Read them out of the workflow file itself, so
/// the workflow and the policy cannot drift apart without this failing.
#[test]
fn ci_placeholder_secrets_satisfy_the_release_gate() {
    const WORKFLOW: &str = include_str!("../../../.github/workflows/rust.yml");

    let value_of = |key: &str| -> String {
        WORKFLOW
            .lines()
            .map(str::trim)
            .find_map(|line| {
                line.strip_prefix(key)
                    .and_then(|rest| rest.strip_prefix(':'))
            })
            .map(|v| v.trim().to_string())
            .unwrap_or_else(|| panic!("{key} is not set in .github/workflows/rust.yml"))
    };
    let pass = value_of("WIFI_PASS");
    let token = value_of("WIFI_OTA_TOKEN");

    assert_ne!(
        token,
        build_script::DEFAULT_OTA_TOKEN,
        "CI must not use the public default token"
    );
    assert!(
        (8..=63).contains(&pass.len()),
        "CI WIFI_PASS must satisfy the WPA2-PSK length rule"
    );

    let pairs = [
        ("WIFI_PASS", pass.as_str()),
        ("WIFI_OTA_TOKEN", token.as_str()),
    ];
    let ci = env_of(&pairs);
    assert_eq!(
        check_release_secrets("xtensa", "release", false, &ci),
        Ok(()),
        "the xtensa CI job's placeholders must pass the gate"
    );
}

#[test]
fn provisioning_ap_permits_unset_and_empty_secrets_on_xtensa_release() {
    let none = env_of(&[]);
    assert_eq!(
        check_release_secrets_with_mode("xtensa", "release", false, true, &none),
        Ok(())
    );

    let both_empty = env_of(&[("WIFI_PASS", ""), ("WIFI_OTA_TOKEN", "")]);
    assert_eq!(
        check_release_secrets_with_mode("xtensa", "release", false, true, &both_empty),
        Ok(())
    );

    let empty_pass_valid_token = env_of(&[
        ("WIFI_PASS", ""),
        ("WIFI_OTA_TOKEN", "a-long-random-secret"),
    ]);
    assert_eq!(
        check_release_secrets_with_mode("xtensa", "release", false, true, &empty_pass_valid_token),
        Ok(())
    );

    let valid_pass_unset_token = env_of(&[("WIFI_PASS", "correct-horse-battery")]);
    assert_eq!(
        check_release_secrets_with_mode("xtensa", "release", false, true, &valid_pass_unset_token),
        Ok(())
    );

    assert_eq!(
        check_release_secrets_with_mode("xtensa", "release", false, true, &env_of(GOOD)),
        Ok(())
    );
}

#[test]
fn provisioning_ap_still_rejects_invalid_non_empty_tokens() {
    let default_tok = env_of(&[("WIFI_OTA_TOKEN", klipper_cam_fw::DEFAULT_OTA_TOKEN)]);
    let err = check_release_secrets_with_mode("xtensa", "release", false, true, &default_tok)
        .expect_err("default token must be rejected even under provisioning-ap");
    assert!(err.contains("default"), "{err}");

    let shadowed_default = env_of(&[
        ("CAM_OTA_TOKEN", klipper_cam_fw::DEFAULT_OTA_TOKEN),
        ("WIFI_OTA_TOKEN", "a-long-random-secret"),
    ]);
    assert!(
        check_release_secrets_with_mode("xtensa", "release", false, true, &shadowed_default)
            .is_err()
    );

    let short_tok = env_of(&[("WIFI_OTA_TOKEN", "123456789012345")]);
    let err = check_release_secrets_with_mode("xtensa", "release", false, true, &short_tok)
        .expect_err("15-byte token must be rejected even under provisioning-ap");
    assert!(err.contains("16"), "{err}");

    for bad_token in [
        "  valid-16-byte-tk  ",
        " valid-16-byte-tk",
        "valid-16-byte-tk ",
        "valid 16 byte tk!",
        "                ",
        "\tvalid-16-byte-tk",
        "valid-16-byte-tk\n",
        "valid-16-byte-\u{00A0}k",
    ] {
        let pairs = [("WIFI_OTA_TOKEN", bad_token)];
        let env = env_of(&pairs);
        assert!(
            check_release_secrets_with_mode("xtensa", "release", false, true, &env).is_err(),
            "provisioning-ap must reject whitespace/non-graphic token: {bad_token:?}"
        );
    }
}

#[test]
fn provisioning_ap_still_rejects_bad_non_empty_wifi_pass() {
    let short_pass = env_of(&[("WIFI_PASS", "1234567")]);
    let err = check_release_secrets_with_mode("xtensa", "release", false, true, &short_pass)
        .expect_err("short non-empty WIFI_PASS must be rejected");
    assert!(err.contains("8..=63"), "{err}");

    let long_64 = "p".repeat(64);
    let long_pairs = [("WIFI_PASS", long_64.as_str())];
    let err =
        check_release_secrets_with_mode("xtensa", "release", false, true, &env_of(&long_pairs))
            .expect_err("64-byte WIFI_PASS must be rejected");
    assert!(err.contains("8..=63"), "{err}");

    let min_valid = env_of(&[("WIFI_PASS", "12345678")]);
    assert_eq!(
        check_release_secrets_with_mode("xtensa", "release", false, true, &min_valid),
        Ok(())
    );

    let max_63 = "p".repeat(63);
    let max_pairs = [("WIFI_PASS", max_63.as_str())];
    assert_eq!(
        check_release_secrets_with_mode("xtensa", "release", false, true, &env_of(&max_pairs)),
        Ok(())
    );
}

#[test]
fn provisioning_ap_disabled_retains_strict_rejections() {
    let none = env_of(&[]);
    assert!(check_release_secrets_with_mode("xtensa", "release", false, false, &none).is_err());

    let no_token = env_of(&[("WIFI_PASS", "correct-horse-battery")]);
    let err = check_release_secrets_with_mode("xtensa", "release", false, false, &no_token)
        .expect_err("missing token must fail when provisioning-ap is disabled");
    assert!(err.contains("WIFI_OTA_TOKEN"), "{err}");

    let no_pass = env_of(&[("WIFI_OTA_TOKEN", "a-long-random-secret")]);
    let err = check_release_secrets_with_mode("xtensa", "release", false, false, &no_pass)
        .expect_err("missing pass must fail when provisioning-ap and allow-open-wifi are disabled");
    assert!(err.contains("WIFI_PASS"), "{err}");

    assert_eq!(
        check_release_secrets_with_mode("xtensa", "release", true, false, &no_pass),
        Ok(())
    );
    assert!(check_release_secrets_with_mode("xtensa", "release", true, false, &none).is_err());
}
