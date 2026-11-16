//! Level 3 Automated Verification Gate for ESP32-WROVER Boot Invariants.
//!
//! Replicates `scripts/verify_cold_boot_invariants.py` with behavioral parity.

use std::collections::BTreeMap;
use std::io::{Read, Write};
use std::net::TcpStream;
use std::path::{Path, PathBuf};
use std::time::Duration;

/// Minimum valid 2nd-stage bootloader `.bin` size in bytes.
pub const BOOTLOADER_MIN_BYTES: usize = 2_048;

/// Maximum valid 2nd-stage bootloader `.bin` size (`0x7000` bytes).
pub const BOOTLOADER_MAX_BYTES: usize = 28_672;

const FORBIDDEN_SD_PINS: [&str; 8] = [
    "IO_MUX_GPIO7_REG",
    "IO_MUX_GPIO8_REG",
    "IO_MUX_GPIO9_REG",
    "IO_MUX_GPIO10_REG",
    "GPIO_FUNC7_OUT_SEL_CFG_REG",
    "GPIO_FUNC8_OUT_SEL_CFG_REG",
    "GPIO_FUNC9_OUT_SEL_CFG_REG",
    "GPIO_FUNC10_OUT_SEL_CFG_REG",
];

const FORBIDDEN_GPIO_FUNC_SD_PINS: [&str; 4] = [
    "GPIO_FUNC7_OUT_SEL_CFG_REG",
    "GPIO_FUNC8_OUT_SEL_CFG_REG",
    "GPIO_FUNC9_OUT_SEL_CFG_REG",
    "GPIO_FUNC10_OUT_SEL_CFG_REG",
];

const FORBIDDEN_IO_MUX_SD_PINS: [&str; 4] = [
    "IO_MUX_GPIO7_REG",
    "IO_MUX_GPIO8_REG",
    "IO_MUX_GPIO9_REG",
    "IO_MUX_GPIO10_REG",
];

const GPIO4_CONTENTION_TOKENS: [&str; 2] = ["GPIO_OUT_W1TC_REG", "GPIO_ENABLE_W1TS_REG"];

/// Authoritative ESP-IDF v5.5.4 `soc/*_reg.h` MMIO Register Address Map.
///
/// Enforces Invariant 72 / `INFRA-001`.
pub const AUTHORITATIVE_MMIO_REGISTERS: &[(&str, u32)] = &[
    ("DPORT_CPU_PER_CONF_REG", 0x3FF0_003C),
    ("DPORT_PRO_CACHE_CTRL_REG", 0x3FF0_0040),
    ("DPORT_PRO_CACHE_CTRL1_REG", 0x3FF0_0044),
    ("DPORT_APP_CACHE_CTRL_REG", 0x3FF0_0058),
    ("DPORT_APP_CACHE_CTRL1_REG", 0x3FF0_005C),
    ("DPORT_HOST_INF_SEL_REG", 0x3FF0_00BC),
    ("DPORT_WIFI_CLK_EN_REG", 0x3FF0_00CC),
    ("DPORT_CORE_RST_EN_REG", 0x3FF0_00D0),
    ("DPORT_PRO_FLASH_MMU_TABLE", 0x3FF1_0000),
    ("DPORT_APP_FLASH_MMU_TABLE", 0x3FF1_2000),
    ("UART0_CLKDIV_REG", 0x3FF4_0014),
    ("SPI1_CMD_REG", 0x3FF4_2000),
    ("SPI1_ADDR_REG", 0x3FF4_2004),
    ("SPI1_CTRL_REG", 0x3FF4_2008),
    ("SPI1_CTRL2_REG", 0x3FF4_2014),
    ("SPI1_CLOCK_REG", 0x3FF4_2018),
    ("SPI1_USER_REG", 0x3FF4_201C),
    ("SPI1_USER1_REG", 0x3FF4_2020),
    ("SPI1_USER2_REG", 0x3FF4_2024),
    ("SPI1_MOSI_DLEN_REG", 0x3FF4_2028),
    ("SPI1_MISO_DLEN_REG", 0x3FF4_202C),
    ("SPI1_PIN_REG", 0x3FF4_2034),
    ("SPI1_W0_REG", 0x3FF4_2080),
    ("SPI1_EXT2_REG", 0x3FF4_20F8),
    ("SPI0_CTRL_REG", 0x3FF4_3008),
    ("SPI0_CTRL2_REG", 0x3FF4_3014),
    ("SPI0_CLOCK_REG", 0x3FF4_3018),
    ("SPI0_USER_REG", 0x3FF4_301C),
    ("SPI0_USER1_REG", 0x3FF4_3020),
    ("SPI0_PIN_REG", 0x3FF4_3034),
    ("SPI0_EXT2_REG", 0x3FF4_30F8),
    ("GPIO_OUT_W1TS_REG", 0x3FF4_4008),
    ("GPIO_OUT_W1TC_REG", 0x3FF4_400C),
    ("GPIO_ENABLE_W1TS_REG", 0x3FF4_4024),
    ("GPIO_IN_REG", 0x3FF4_403C),
    ("GPIO_FUNC16_OUT_SEL_CFG_REG", 0x3FF4_4570),
    ("GPIO_FUNC17_OUT_SEL_CFG_REG", 0x3FF4_4574),
    ("RTC_CNTL_OPTIONS0_REG", 0x3FF4_8000),
    ("RTC_CNTL_RESET_STATE_REG", 0x3FF4_8034),
    ("RTC_CNTL_INT_ENA_REG", 0x3FF4_8040),
    ("RTC_CNTL_INT_CLR_REG", 0x3FF4_8048),
    ("RTC_CNTL_CLK_CONF_REG", 0x3FF4_8070),
    ("RTC_CNTL_SDIO_CONF_REG", 0x3FF4_8074),
    ("RTC_CNTL_REG", 0x3FF4_807C),
    ("RTC_CNTL_DIG_PWC_REG", 0x3FF4_8084),
    ("RTC_CNTL_DIG_ISO_REG", 0x3FF4_8088),
    ("RTC_CNTL_WDTCONFIG0_REG", 0x3FF4_808C),
    ("RTC_CNTL_WDTCONFIG1_REG", 0x3FF4_8090),
    ("RTC_CNTL_WDTFEED_REG", 0x3FF4_80A0),
    ("RTC_CNTL_WDTWPROTECT_REG", 0x3FF4_80A4),
    ("RTC_CNTL_STORE6_REG", 0x3FF4_80B8),
    ("RTC_CNTL_STORE7_REG", 0x3FF4_80BC),
    ("RTC_CNTL_HOLD_FORCE_REG", 0x3FF4_80C8),
    ("RTC_CNTL_BROWN_OUT_REG", 0x3FF4_80D4),
    ("RTC_IO_DIG_PAD_HOLD_REG", 0x3FF4_8474),
    ("SENS_SAR_SLAVE_ADDR3_REG", 0x3FF4_8844),
    ("SENS_SAR_TSENS_CTRL_REG", 0x3FF4_884C),
    ("IO_MUX_GPIO16_REG", 0x3FF4_904C),
    ("IO_MUX_GPIO17_REG", 0x3FF4_9050),
    ("IO_MUX_GPIO9_REG", 0x3FF4_9054),
    ("IO_MUX_GPIO10_REG", 0x3FF4_9058),
    ("IO_MUX_GPIO11_REG", 0x3FF4_905C),
    ("IO_MUX_GPIO6_REG", 0x3FF4_9060),
    ("IO_MUX_GPIO7_REG", 0x3FF4_9064),
    ("IO_MUX_GPIO8_REG", 0x3FF4_9068),
    ("I2S_INT_RAW_REG", 0x3FF4_F00C),
    ("I2S_INT_CLR_REG", 0x3FF4_F018),
    ("EFUSE_BLK0_RDATA0_REG", 0x3FF5_A000),
    ("EFUSE_BLK0_RDATA1_REG", 0x3FF5_A004),
    ("EFUSE_BLK0_RDATA2_REG", 0x3FF5_A008),
    ("EFUSE_BLK0_RDATA3_REG", 0x3FF5_A00C),
    ("TIMG0_WDTCONFIG0_REG", 0x3FF5_F048),
    ("TIMG0_WDTWPROTECT_REG", 0x3FF5_F064),
    ("SYSCON_XTAL_TICK_CONF_REG", 0x3FF6_6004),
    ("SYSCON_PLL_TICK_CONF_REG", 0x3FF6_6008),
    ("RNG_DATA_REG", 0x3FF7_5144),
    // ESP32-S3 LCD_CAM Registers (soc/esp32s3/register/soc/lcd_cam_reg.h)
    ("LCD_CAM_LCD_CLOCK_REG", 0x6004_1000),
    ("LCD_CAM_CAM_CTRL_REG", 0x6004_1004),
    ("LCD_CAM_CAM_CTRL1_REG", 0x6004_1008),
    ("LCD_CAM_CAM_RGB_YUV_REG", 0x6004_100C),
    ("LCD_CAM_LCD_RGB_YUV_REG", 0x6004_1010),
    ("LCD_CAM_LCD_USER_REG", 0x6004_1014),
    ("LCD_CAM_LCD_MISC_REG", 0x6004_1018),
    ("LCD_CAM_LCD_CTRL_REG", 0x6004_101C),
    ("LCD_CAM_LCD_CTRL1_REG", 0x6004_1020),
    ("LCD_CAM_LCD_CTRL2_REG", 0x6004_1024),
    ("LCD_CAM_LCD_CMD_VAL_REG", 0x6004_1028),
    ("LCD_CAM_LCD_DLY_MODE_REG", 0x6004_1030),
    ("LCD_CAM_LCD_DATA_DOUT_MODE_REG", 0x6004_1038),
    ("LCD_CAM_LC_DMA_INT_ENA_REG", 0x6004_1064),
    ("LCD_CAM_LC_DMA_INT_RAW_REG", 0x6004_1068),
    ("LCD_CAM_LC_DMA_INT_ST_REG", 0x6004_106C),
    ("LCD_CAM_LC_DMA_INT_CLR_REG", 0x6004_1070),
    ("LCD_CAM_LC_REG_DATE_REG", 0x6004_10FC),
];

/// REA-proven proprietary RF coexistence MMIO registers (`librtc.a:bt_bb.o`).
///
/// Permitted exclusively inside `crates/esp32-cam-rf-clock/src/rtc_rf.rs`.
pub const REA_PROVEN_RF_COEX_MMIO_REGISTERS: &[(&str, u32)] = &[
    ("COEX_SLC_0x3FF5C080_REG", 0x3FF5_C080),
    ("COEX_MAC_0x600310D0_REG", 0x6003_10D0),
    ("COEX_MAC_0x60031300_REG", 0x6003_1300),
    ("COEX_BT_PRIO0_0x60033D30_REG", 0x6003_3D30),
    ("COEX_BT_PRIO1_0x60033D38_REG", 0x6003_3D38),
    ("COEX_BT_PRIO2_0x60033D40_REG", 0x6003_3D40),
    ("COEX_FE_0x600041C4_REG", 0x6000_41C4),
    ("BB_COEX_0x3FF5D040_REG", 0x3FF5_D040),
];

/// REA-proven proprietary Wi-Fi MAC MMIO registers.
///
/// Permitted inside `crates/esp32-cam-wifi-mac/src/`.
pub const REA_PROVEN_WIFI_MAC_MMIO_REGISTERS: &[(&str, u32)] = &[
    ("MAC_RX_REG_3080", 0x3FF7_3080),
    ("RX_CTRL_REG", 0x3FF7_3084),
    ("RX_BASE_DSCR_REG", 0x3FF7_3088),
    ("RX_NEXT_DSCR_REG", 0x3FF7_308C),
    ("RX_LAST_DSCR_REG", 0x3FF7_3090),
    ("MAC_RX_REG_309C", 0x3FF7_309C),
    ("MAC_RX_REG_30A4", 0x3FF7_30A4),
    ("RX_POLICY_BASE_REG", 0x3FF7_30D8),
    ("MAC_RX_REG_30F8", 0x3FF7_30F8),
    ("MAC_RX_REG_30FC", 0x3FF7_30FC),
    ("MAC_RX_REG_3100", 0x3FF7_3100),
    ("MAC_RX_REG_3104", 0x3FF7_3104),
    ("MAC_RX_REG_310C", 0x3FF7_310C),
    ("MAC_RX_REG_3114", 0x3FF7_3114),
    ("MAC_RX_REG_3118", 0x3FF7_3118),
    ("MAC_RX_REG_311C", 0x3FF7_311C),
    ("MAC_RX_REG_3120", 0x3FF7_3120),
    ("MAC_RX_REG_3124", 0x3FF7_3124),
    ("MAC_RX_REG_3148", 0x3FF7_3148),
    ("MAC_RX_REG_314C", 0x3FF7_314C),
    ("MAC_RX_REG_3158", 0x3FF7_3158),
    ("MAC_RX_REG_3164", 0x3FF7_3164),
    ("MAC_RX_REG_3284", 0x3FF7_3284),
    ("MAC_RX_REG_3288", 0x3FF7_3288),
    ("MAC_CTRL_REG_3C04", 0x3FF7_3C04),
    ("MAC_INIT_REG_3C1C", 0x3FF7_3C1C),
    ("MAC_INIT_REG_3C20", 0x3FF7_3C20),
    ("MAC_INIT_REG_3C24", 0x3FF7_3C24),
    ("INT_ENABLE_REG", 0x3FF7_3C40),
    ("INT_ENABLE_SEC_REG", 0x3FF7_3C44),
    ("INT_STATUS_REG", 0x3FF7_3C48),
    ("INT_CLEAR_REG", 0x3FF7_3C4C),
    ("MAC_INIT_REG_3C54", 0x3FF7_3C54),
    ("MAC_INIT_REG_3C58", 0x3FF7_3C58),
    ("MAC_INIT_REG_3C5C", 0x3FF7_3C5C),
    ("MAC_INIT_REG_3C6C", 0x3FF7_3C6C),
    ("MAC_INIT_REG_3C74", 0x3FF7_3C74),
    ("MAC_INIT_REG_3C78", 0x3FF7_3C78),
    ("MAC_INIT_REG_3C88", 0x3FF7_3C88),
    ("MAC_INIT_REG_3CAC", 0x3FF7_3CAC),
    ("MAC_CLK_RST_REG", 0x3FF7_3CB8),
    ("TX_ERR_CLEAR_REG", 0x3FF7_3CBC),
    ("TX_ERR_STATUS_REG", 0x3FF7_3CC0),
    ("TX_DONE_CLEAR_REG", 0x3FF7_3CC4),
    ("TX_DONE_STATUS_REG", 0x3FF7_3CC8),
    ("TX_TIMEOUT_EDCA_REG", 0x3FF7_3D1C),
    ("TX_PLCP0_TRIG_REG", 0x3FF7_3D20),
    ("MAC_INIT_REG_3D24", 0x3FF7_3D24),
    ("MAC_INIT_REG_3D78", 0x3FF7_3D78),
    ("TX_PLCP1_REG", 0x3FF7_4258),
    ("TX_PLCP2_REG", 0x3FF7_425C),
    ("TX_HTSIG1_REG", 0x3FF7_4260),
    ("TX_HTSIG2_REG", 0x3FF7_4264),
    ("TX_DURATION_REG", 0x3FF7_4268),
    ("TX_PMD_REG", 0x3FF7_4270),
    ("KEY_TABLE_BASE_REG", 0x3FF7_4400),
    ("WDEV_RND_REG", 0x3FF7_5144),
    ("MAC_LOW_RATE_DISABLE_REG_6860", 0x6001_C860),
    ("MAC_LOW_RATE_DISABLE_REG_687C", 0x6001_C87C),
];

/// Strips `//` line comments and `/* ... */` block comments from Rust source.
///
/// Preserves string and raw-string literals verbatim.
#[must_use]
pub fn strip_rust_comments(src: &str) -> String {
    let mut out = String::with_capacity(src.len());
    let bytes = src.as_bytes();
    let len = bytes.len();
    let mut i = 0;
    let mut last_copied = 0;

    while i < len {
        if bytes[i] == b'"' {
            i += 1;
            while i < len {
                if bytes[i] == b'\\' && i + 1 < len {
                    i += 2;
                } else if bytes[i] == b'"' {
                    i += 1;
                    break;
                } else {
                    i += 1;
                }
            }
        } else if bytes[i] == b'r' && i + 1 < len && (bytes[i + 1] == b'"' || bytes[i + 1] == b'#')
        {
            i += 1;
            let mut hashes = 0;
            while i < len && bytes[i] == b'#' {
                hashes += 1;
                i += 1;
            }
            if i < len && bytes[i] == b'"' {
                i += 1;
                while i < len {
                    if bytes[i] == b'"' {
                        let after_quote = i + 1;
                        if after_quote + hashes <= len
                            && bytes[after_quote..after_quote + hashes]
                                .iter()
                                .all(|&c| c == b'#')
                        {
                            i = after_quote + hashes;
                            break;
                        }
                    }
                    i += 1;
                }
            }
        } else if bytes[i] == b'/' && i + 1 < len && bytes[i + 1] == b'/' {
            out.push_str(&src[last_copied..i]);
            i += 2;
            while i < len && bytes[i] != b'\n' {
                i += 1;
            }
            last_copied = i;
        } else if bytes[i] == b'/' && i + 1 < len && bytes[i + 1] == b'*' {
            out.push_str(&src[last_copied..i]);
            i += 2;
            let mut depth = 1;
            while i < len && depth > 0 {
                if bytes[i] == b'/' && i + 1 < len && bytes[i + 1] == b'*' {
                    depth += 1;
                    i += 2;
                } else if bytes[i] == b'*' && i + 1 < len && bytes[i + 1] == b'/' {
                    depth -= 1;
                    i += 2;
                } else {
                    i += 1;
                }
            }
            last_copied = i;
        } else {
            i += 1;
        }
    }
    if last_copied < len {
        out.push_str(&src[last_copied..len]);
    }
    out
}

fn authoritative_map() -> BTreeMap<&'static str, u32> {
    AUTHORITATIVE_MMIO_REGISTERS.iter().copied().collect()
}

fn rea_coex_map() -> BTreeMap<&'static str, u32> {
    REA_PROVEN_RF_COEX_MMIO_REGISTERS.iter().copied().collect()
}

fn rea_wifi_mac_map() -> BTreeMap<&'static str, u32> {
    REA_PROVEN_WIFI_MAC_MMIO_REGISTERS.iter().copied().collect()
}

/// Parses MMIO register declarations from source code.
#[must_use]
pub fn parse_mmio_declarations(src: &str) -> Vec<(&str, u32, String)> {
    let mut results = Vec::new();
    let mut cursor = 0;

    while let Some(pos) = src[cursor..].find("const ") {
        let decl_start = cursor + pos + 6;
        cursor = decl_start;
        let rest = &src[decl_start..];

        let name_end = match rest.find(|c: char| c.is_whitespace() || c == ':') {
            Some(e) => e,
            None => continue,
        };
        let name = &rest[..name_end];
        if !name.ends_with("_REG") && !name.ends_with("_MMU_TABLE") {
            continue;
        }

        let after_name = &rest[name_end..];
        let colon_pos = match after_name.find(':') {
            Some(p) => p,
            None => continue,
        };
        let after_colon = &after_name[colon_pos + 1..];
        let eq_pos = match after_colon.find('=') {
            Some(p) => p,
            None => continue,
        };
        let after_eq = after_colon[eq_pos + 1..].trim_start();

        if !after_eq.starts_with("0x") && !after_eq.starts_with("0X") {
            continue;
        }

        let hex_slice = &after_eq[2..];
        let hex_end = hex_slice
            .find(|c: char| !c.is_ascii_hexdigit() && c != '_')
            .unwrap_or(hex_slice.len());
        let raw_hex = format!("0x{}", &hex_slice[..hex_end]);
        let clean_hex: String = hex_slice[..hex_end].chars().filter(|&c| c != '_').collect();

        if let Ok(addr) = u32::from_str_radix(&clean_hex, 16) {
            results.push((name, addr, raw_hex));
        }
    }
    results
}

fn verify_mmio_declarations_with_named_extra(
    file_label: &str,
    matches: &[(&str, u32, String)],
    extra: Option<(&BTreeMap<&'static str, u32>, &str, &str)>,
) -> Vec<String> {
    let mut errors = Vec::new();
    let auth_map = authoritative_map();

    for (reg_name, actual_addr, hex_literal) in matches {
        let found = auth_map
            .get(reg_name)
            .map(|&addr| (addr, "ESP-IDF v5.5.4 soc/*_reg.h"))
            .or_else(|| {
                extra.and_then(|(map, label, _)| map.get(reg_name).map(|&addr| (addr, label)))
            });
        match found {
            None => {
                let extra_note = match extra {
                    Some((_, _, list_name)) => format!(" and {list_name}"),
                    None => String::new(),
                };
                errors.push(format!(
                    "{file_label}: unverified MMIO register `{reg_name}` ({hex_literal}) \
                     missing from AUTHORITATIVE_MMIO_REGISTERS{extra_note}"
                ));
            }
            Some((expected_addr, source_label)) => {
                if *actual_addr != expected_addr {
                    errors.push(format!(
                        "{file_label}: MMIO address mismatch for `{reg_name}`: \
                         found 0x{actual_addr:08X}, expected 0x{expected_addr:08X} \
                         per {source_label}"
                    ));
                }
            }
        }
    }
    errors
}

fn verify_mmio_declarations_with_extra(
    file_label: &str,
    matches: &[(&str, u32, String)],
    extra_map: Option<&BTreeMap<&'static str, u32>>,
) -> Vec<String> {
    let extra = extra_map.map(|map| {
        (
            map,
            "REA-proven librtc.a:bt_bb.o",
            "REA_PROVEN_RF_COEX_MMIO_REGISTERS",
        )
    });
    verify_mmio_declarations_with_named_extra(file_label, matches, extra)
}

fn verify_mmio_declarations_from_matches(
    file_label: &str,
    matches: &[(&str, u32, String)],
) -> Vec<String> {
    verify_mmio_declarations_with_extra(file_label, matches, None)
}

/// Verifies that all MMIO register constants match the authoritative map.
#[must_use]
pub fn verify_mmio_registers(src: &str) -> Vec<String> {
    let matches = parse_mmio_declarations(src);
    if matches.is_empty() {
        return vec!["main.rs: failed to parse any MMIO `const *_REG` declarations".to_string()];
    }
    verify_mmio_declarations_from_matches("main.rs", &matches)
}

fn check_forbidden_sd_pins(file_label: &str, src: &str, errors: &mut Vec<String>) {
    for sym in FORBIDDEN_SD_PINS {
        if src.contains(sym) {
            errors.push(format!(
                "{file_label}: forbidden SD0..SD3 bit-banging symbol '{sym}' present"
            ));
        }
    }
}

fn has_gpio_pushpull_drive(src: &str, tokens: &[&str], pin: u8) -> bool {
    let shift_a = format!("1 << {pin}");
    let shift_b = format!("1u32 << {pin}");

    for token in tokens {
        let mut offset = 0;
        while let Some(pos) = src[offset..].find(token) {
            let check_window = &src[offset + pos..];
            let mut window_len = check_window.len().min(80);
            while window_len > 0 && !check_window.is_char_boundary(window_len) {
                window_len -= 1;
            }
            let window = &check_window[..window_len];
            if window.contains(&shift_a) || window.contains(&shift_b) {
                return true;
            }
            offset += pos + token.len();
        }
    }
    false
}

/// Verifies udev rules for forbidden reset hooks.
#[must_use]
pub fn verify_udev_rules(repo_root: &Path) -> Vec<String> {
    let mut errors = Vec::new();
    let forbidden_tokens = ["RUN+=", "TIOCMBIS", "TIOCMSET", "stty"];

    // 1. Repository template docs/99-esp32-wrover.rules
    let docs_rule = repo_root.join("docs").join("99-esp32-wrover.rules");
    if !docs_rule.is_file() {
        errors.push(format!(
            "Missing udev rule template file: {}",
            docs_rule.display()
        ));
    } else if let Ok(text) = std::fs::read_to_string(&docs_rule) {
        for tok in forbidden_tokens {
            if text.contains(tok) {
                errors.push(format!(
                    "Forbidden token '{tok}' found in {}",
                    docs_rule.display()
                ));
            }
        }
    }

    // 2. Host udev rule
    let etc_rule = Path::new("/etc/udev/rules.d/99-esp32-wrover.rules");
    if etc_rule.is_file() {
        if let Ok(text) = std::fs::read_to_string(etc_rule) {
            for tok in forbidden_tokens {
                if text.contains(tok) {
                    errors.push(format!(
                        "Forbidden token '{tok}' found in {}",
                        etc_rule.display()
                    ));
                }
            }
        }
    } else {
        println!(
            "[INFO] Host udev rule {} not present (skipping non-hardware host check)",
            etc_rule.display()
        );
    }

    errors
}

/// Verifies the main firmware source code invariants.
#[must_use]
pub fn verify_main_rs(repo_root: &Path) -> Vec<String> {
    let mut errors = Vec::new();
    let main_rs = repo_root
        .join("crates")
        .join("klipper-cam-fw")
        .join("src")
        .join("main.rs");

    if !main_rs.is_file() {
        return vec![format!(
            "Missing firmware entrypoint: {}",
            main_rs.display()
        )];
    }
    let raw_src = match std::fs::read_to_string(&main_rs) {
        Ok(s) => s,
        Err(e) => return vec![format!("Failed to read {}: {e}", main_rs.display())],
    };
    let src = strip_rust_comments(&raw_src);

    // 1. Zero SD0..SD3 (GPIO 7..10) bit-banging
    check_forbidden_sd_pins("main.rs", &src, &mut errors);

    // 2. arm_rtc_wdt_15s() immediately after esp_hal::init(config)
    let init_idx = src.find("esp_hal::init(config);");
    let wdt_called = if let Some(idx) = init_idx {
        let after_init = src[idx + 22..].trim_start();
        after_init.starts_with("arm_rtc_wdt_15s();")
    } else {
        false
    };
    if !wdt_called {
        errors.push(
            "main.rs: arm_rtc_wdt_15s() is not called immediately after esp_hal::init(config)"
                .to_string(),
        );
    }

    // 3. Hardware SPI1 0xF5 and 0x35 commands + PsramCacheF40mS40m
    if !src.contains("fn psram_spi1_cmd(qpi_mode: bool, cmd_byte: u8)") {
        errors.push(
            "main.rs: missing hardware SPI1 helper \
             `fn psram_spi1_cmd(qpi_mode: bool, cmd_byte: u8)`"
                .to_string(),
        );
    }
    if !src.contains("psram_spi1_cmd(true, 0xF5)") || !src.contains("psram_spi1_cmd(false, 0x35)") {
        errors.push(
            "main.rs: missing `psram_spi1_cmd(true, 0xF5)` or `psram_spi1_cmd(false, 0x35)` calls"
                .to_string(),
        );
    }
    if !src.contains("ensure_psram_qpi_mode_spi1()") {
        errors.push("main.rs: missing `ensure_psram_qpi_mode_spi1()`".to_string());
    }
    if !src.contains("PsramCacheSpeed::PsramCacheF40mS40m") {
        errors.push(
            "main.rs: must use `PsramCacheSpeed::PsramCacheF40mS40m` matching ESP-IDF v5.5.4"
                .to_string(),
        );
    }

    // 4. Zero boot-time FlashStorage calls before pure-Rust Wi-Fi init
    if let Some(main_idx) = src.find("fn main() -> !") {
        let main_src = &src[main_idx..];
        let wifi_init_pos = main_src
            .find("wifi_driver.init(")
            .or_else(|| main_src.find("XtensaPhySession::new("))
            .or_else(|| main_src.find("WifiController::new("));
        if let Some(wifi_idx) = wifi_init_pos {
            let pre_wifi = &main_src[..wifi_idx];
            let forbidden = [
                ".read_nor(",
                ".erase(",
                ".write_nor(",
                "OtaDataManager::load_from_flash",
                "cancel_rollback_in_flash",
            ];
            for op in forbidden {
                if pre_wifi.contains(op) {
                    errors.push(format!(
                        "main.rs: forbidden boot-time flash operation '{op}' \
                         before Wi-Fi initialization"
                    ));
                }
            }
        } else {
            errors.push("main.rs: missing `WifiController::new(` in main()".to_string());
        }
    } else {
        errors.push("main.rs: missing `fn main() -> !`".to_string());
    }

    // 5. Authoritative ESP-IDF v5.5.4 MMIO register check (Invariant 72)
    errors.extend(verify_mmio_registers(&src));

    // 6. Zero GPIO 4 push-pull contention on ESP32-WROVER-DEV (Invariant 71)
    if has_gpio_pushpull_drive(&src, &GPIO4_CONTENTION_TOKENS, 4) {
        errors.push(
            "main.rs: forbidden push-pull output drive on GPIO 4 (`1 << 4`), \
             which contentions with OV2640 CAM_PIN_D0 (Y2) on ESP32-WROVER-DEV"
                .to_string(),
        );
    }

    // 7. Active OTA slot detection via DPORT_PRO_FLASH_MMU_TABLE (0x3FF1_0000)
    if !src.contains("detect_booted_ota_manager_from_mmu()") {
        errors.push(
            "main.rs: must detect active booted OTA slot from DPORT_PRO_FLASH_MMU_TABLE \
             via `detect_booted_ota_manager_from_mmu()`"
                .to_string(),
        );
    }

    // 8. H3-CFG-003: hold_psram_cs_high_safe()
    if !src.contains("hold_psram_cs_high_safe()") {
        errors.push("main.rs: missing `hold_psram_cs_high_safe()`".to_string());
    }

    // 9. H3-CFG-003: SPI0_PIN_REG CS1_DIS gating for SPI1 PSRAM transactions
    let has_spi0_pin_gate = if let Some(idx) = src.find("SPI0_PIN_REG") {
        let after = &src[idx..];
        after.contains("1 << 1") || after.contains("1u32 << 1")
    } else {
        false
    };
    if !has_spi0_pin_gate {
        errors.push(
            "main.rs: missing `SPI0_PIN_REG` CS1_DIS bit (1 << 1) gating \
             before SPI1 PSRAM transactions"
                .to_string(),
        );
    }

    // 10. H3-CFG-003: 50 ms PSRAM power-up delay (50_000 / delay_millis(50))
    let has_delay = src.contains("50_000")
        || src.contains("delay_millis(50)")
        || src.contains("delay_micros(50_000)");
    if !has_delay {
        errors.push(
            "main.rs: missing 50 ms PSRAM power-up delay (50_000 / delay_millis(50))".to_string(),
        );
    }

    // 11. LOGIC-186: capture_live_hd_jpeg_with_diag present
    if !src.contains("capture_live_hd_jpeg_with_diag") {
        errors.push("main.rs: missing `capture_live_hd_jpeg_with_diag`".to_string());
    }

    errors
}

fn collect_rs_files(src_dir: &Path, label: &str) -> Result<Vec<PathBuf>, String> {
    if !src_dir.is_dir() {
        return Err(format!(
            "Missing {label} source directory: {}",
            src_dir.display()
        ));
    }

    let entries = std::fs::read_dir(src_dir).map_err(|e| {
        format!(
            "Failed to read {label} source directory {}: {e}",
            src_dir.display()
        )
    })?;

    let mut rs_files = Vec::new();
    for entry in entries {
        let Ok(entry) = entry else { continue };
        let path = entry.path();
        if path.extension().and_then(|s| s.to_str()) == Some("rs") {
            rs_files.push(path);
        }
    }
    rs_files.sort();

    if rs_files.is_empty() {
        return Err(format!(
            "No Rust source files found in {}",
            src_dir.display()
        ));
    }

    Ok(rs_files)
}

/// Verifies all Rust sources in `crates/esp32-cam-bootloader/src/`.
///
/// Enforces Invariants 70, 71, and 72 across every module, handling modules
/// with zero `const *_REG` declarations cleanly while validating every MMIO
/// declaration found against [`AUTHORITATIVE_MMIO_REGISTERS`].
#[must_use]
pub fn verify_bootloader_sources(repo_root: &Path) -> Vec<String> {
    let mut errors = Vec::new();
    let bootloader_src = repo_root
        .join("crates")
        .join("esp32-cam-bootloader")
        .join("src");

    let rs_files = match collect_rs_files(&bootloader_src, "bootloader") {
        Ok(files) => files,
        Err(err) => return vec![err],
    };

    let mut total_mmio_decls = 0usize;
    let mut saw_gpio16_rs = false;
    let mut saw_wdt_rs = false;

    for path in rs_files {
        let file_name = path
            .file_name()
            .map(|n| n.to_string_lossy().into_owned())
            .unwrap_or_else(|| path.display().to_string());
        let file_label = format!("esp32-cam-bootloader/{file_name}");

        let raw_src = match std::fs::read_to_string(&path) {
            Ok(s) => s,
            Err(e) => {
                errors.push(format!("Failed to read {}: {e}", path.display()));
                continue;
            }
        };
        let src = strip_rust_comments(&raw_src);

        let decls = parse_mmio_declarations(&src);
        total_mmio_decls += decls.len();
        errors.extend(verify_mmio_declarations_from_matches(&file_label, &decls));

        for sym in FORBIDDEN_GPIO_FUNC_SD_PINS {
            if src.contains(sym) {
                errors.push(format!(
                    "{file_label}: forbidden SD0..SD3 bit-banging symbol '{sym}' present"
                ));
            }
        }

        if file_name == "clock_spi_psram.rs" {
            if src.contains("GPIO_ENABLE") {
                errors.push(format!(
                    "{file_label}: forbidden GPIO_ENABLE token present \
                     to prevent push-pull GPIO bit-banging"
                ));
            }
        } else {
            for sym in FORBIDDEN_IO_MUX_SD_PINS {
                if src.contains(sym) {
                    errors.push(format!(
                        "{file_label}: forbidden SD0..SD3 bit-banging symbol '{sym}' present"
                    ));
                }
            }
        }

        if has_gpio_pushpull_drive(&src, &GPIO4_CONTENTION_TOKENS, 4) {
            errors.push(format!(
                "{file_label}: forbidden push-pull output drive on GPIO 4 (`1 << 4`), \
                 which contentions with OV2640 CAM_PIN_D0 (Y2) on ESP32-WROVER-DEV"
            ));
        }

        if file_name == "gpio16.rs" {
            saw_gpio16_rs = true;
            if !src.contains("IO_MUX_GPIO16_REG") || !src.contains("0x3FF4_904C") {
                errors.push(format!(
                    "{file_label}: missing IO_MUX_GPIO16_REG (0x3FF4_904C) declaration"
                ));
            }
            if !src.contains("0x2B00") && !src.contains("0x0000_2B00") {
                errors.push(format!(
                    "{file_label}: missing GPIO 16 pull-up config constant 0x2B00"
                ));
            }
            if !src.contains("write_u32(IO_MUX_GPIO16_REG") {
                errors.push(format!(
                    "{file_label}: missing MMIO write to IO_MUX_GPIO16_REG"
                ));
            }
        } else if file_name == "wdt.rs" {
            saw_wdt_rs = true;
            if !src.contains("RTC_CNTL_WDTCONFIG0_REG") || !src.contains("0x3FF4_808C") {
                errors.push(format!(
                    "{file_label}: missing RTC_CNTL_WDTCONFIG0_REG (0x3FF4_808C) declaration"
                ));
            }
            if !src.contains("write_u32(RTC_CNTL_WDTCONFIG0_REG") {
                errors.push(format!(
                    "{file_label}: missing RTC_CNTL_WDTCONFIG0_REG arming write"
                ));
            }
        }
    }

    if total_mmio_decls == 0 {
        errors.push(
            "esp32-cam-bootloader: failed to parse any MMIO `const *_REG` declarations".to_string(),
        );
    }
    if !saw_gpio16_rs {
        errors.push("esp32-cam-bootloader: missing required module gpio16.rs".to_string());
    }
    if !saw_wdt_rs {
        errors.push("esp32-cam-bootloader: missing required module wdt.rs".to_string());
    }

    errors
}

const EXPECTED_RF_CLOCK_MODULES: [&str; 6] = [
    "lib.rs",
    "nvs_misc.rs",
    "printf_compat.rs",
    "printf_fmt.rs",
    "regulatory.rs",
    "rtc_rf.rs",
];

fn verify_rf_clock_rtc_declarations(
    file_label: &str,
    matches: &[(&str, u32, String)],
) -> Vec<String> {
    let coex_map = rea_coex_map();
    let mut errors = verify_mmio_declarations_with_extra(file_label, matches, Some(&coex_map));

    if matches.is_empty() {
        errors.push(format!(
            "{file_label}: failed to parse any MMIO `const *_REG` declarations"
        ));
    }

    for required in ["SENS_SAR_SLAVE_ADDR3_REG", "SENS_SAR_TSENS_CTRL_REG"] {
        if !matches.iter().any(|(name, _, _)| *name == required) {
            errors.push(format!(
                "{file_label}: missing required `sens_reg.h` declaration `{required}`"
            ));
        }
    }

    errors
}

/// Verifies all Rust sources in `crates/esp32-cam-rf-clock/src/`.
///
/// Enforces Invariants 70, 71, and 72 across all six required modules,
/// validating `rtc_rf.rs` MMIO declarations against
/// [`AUTHORITATIVE_MMIO_REGISTERS`] or [`REA_PROVEN_RF_COEX_MMIO_REGISTERS`]
/// and requiring strict [`AUTHORITATIVE_MMIO_REGISTERS`] parity elsewhere.
#[must_use]
pub fn verify_rf_clock_sources(repo_root: &Path) -> Vec<String> {
    let rf_clock_src = repo_root
        .join("crates")
        .join("esp32-cam-rf-clock")
        .join("src");
    let rs_files = match collect_rs_files(&rf_clock_src, "rf-clock") {
        Ok(files) => files,
        Err(err) => return vec![err],
    };

    let mut errors = Vec::new();
    let mut seen_modules = [false; EXPECTED_RF_CLOCK_MODULES.len()];

    for path in rs_files {
        let file_name = path
            .file_name()
            .map(|n| n.to_string_lossy().into_owned())
            .unwrap_or_else(|| path.display().to_string());
        if let Some(idx) = EXPECTED_RF_CLOCK_MODULES
            .iter()
            .position(|&expected| expected == file_name)
        {
            seen_modules[idx] = true;
        }
        let file_label = format!("esp32-cam-rf-clock/{file_name}");

        let raw_src = match std::fs::read_to_string(&path) {
            Ok(s) => s,
            Err(e) => {
                errors.push(format!("Failed to read {}: {e}", path.display()));
                continue;
            }
        };
        let src = strip_rust_comments(&raw_src);

        check_forbidden_sd_pins(&file_label, &src, &mut errors);

        if has_gpio_pushpull_drive(&src, &GPIO4_CONTENTION_TOKENS, 4) {
            errors.push(format!(
                "{file_label}: forbidden push-pull output drive on GPIO 4 (`1 << 4`), \
                 which contentions with OV2640 CAM_PIN_D0 (Y2) on ESP32-WROVER-DEV"
            ));
        }

        let decls = parse_mmio_declarations(&src);
        if file_name == "rtc_rf.rs" {
            errors.extend(verify_rf_clock_rtc_declarations(&file_label, &decls));
        } else {
            errors.extend(verify_mmio_declarations_from_matches(&file_label, &decls));
        }
    }

    for (idx, &module_name) in EXPECTED_RF_CLOCK_MODULES.iter().enumerate() {
        if !seen_modules[idx] {
            errors.push(format!(
                "esp32-cam-rf-clock: missing required module {module_name}"
            ));
        }
    }

    errors
}

/// Verifies all Rust sources in `crates/esp32-cam-wifi-mac/src/`.
///
/// Enforces Invariants 70, 71, and 72 across all Wi-Fi MAC modules,
/// validating MMIO declarations against [`AUTHORITATIVE_MMIO_REGISTERS`]
/// or [`REA_PROVEN_WIFI_MAC_MMIO_REGISTERS`] and ensuring zero SD0..SD3
/// bit-banging and zero GPIO 4 output contention.
#[must_use]
pub fn verify_wifi_mac_sources(repo_root: &Path) -> Vec<String> {
    let wifi_mac_src = repo_root
        .join("crates")
        .join("esp32-cam-wifi-mac")
        .join("src");
    let rs_files = match collect_rs_files(&wifi_mac_src, "wifi-mac") {
        Ok(files) => files,
        Err(err) => return vec![err],
    };

    let mut errors = Vec::new();
    let mut saw_regs_rs = false;
    let wifi_map = rea_wifi_mac_map();
    let extra = Some((
        &wifi_map,
        "REA-proven Wi-Fi MAC",
        "REA_PROVEN_WIFI_MAC_MMIO_REGISTERS",
    ));

    for path in rs_files {
        let file_name = path
            .file_name()
            .map(|n| n.to_string_lossy().into_owned())
            .unwrap_or_else(|| path.display().to_string());
        if file_name == "regs.rs" {
            saw_regs_rs = true;
        }
        let file_label = format!("esp32-cam-wifi-mac/{file_name}");

        let raw_src = match std::fs::read_to_string(&path) {
            Ok(s) => s,
            Err(e) => {
                errors.push(format!("Failed to read {}: {e}", path.display()));
                continue;
            }
        };
        let src = strip_rust_comments(&raw_src);

        check_forbidden_sd_pins(&file_label, &src, &mut errors);

        if has_gpio_pushpull_drive(&src, &GPIO4_CONTENTION_TOKENS, 4) {
            errors.push(format!(
                "{file_label}: forbidden push-pull output drive on GPIO 4 (`1 << 4`), \
                 which contentions with OV2640 CAM_PIN_D0 (Y2) on ESP32-WROVER-DEV"
            ));
        }

        let decls = parse_mmio_declarations(&src);
        if file_name == "regs.rs" && decls.is_empty() {
            errors.push(format!(
                "{file_label}: failed to parse any MMIO `const *_REG` declarations"
            ));
        }
        errors.extend(verify_mmio_declarations_with_named_extra(
            &file_label,
            &decls,
            extra,
        ));
    }

    if !saw_regs_rs {
        errors.push("esp32-cam-wifi-mac: missing required module regs.rs".to_string());
    }

    errors
}

fn contains_subslice(data: &[u8], sub: &[u8]) -> bool {
    if sub.is_empty() || sub.len() > data.len() {
        return false;
    }
    data.windows(sub.len()).any(|w| w == sub)
}

/// Verifies the 2nd-stage bootloader binary.
#[must_use]
pub fn verify_bootloader_bin(
    target_path: &Path,
    allow_missing: bool,
) -> (Vec<String>, Option<usize>) {
    let mut errors = Vec::new();
    if !target_path.is_file() {
        if allow_missing {
            println!(
                "[INFO] Patched 2nd-stage bootloader binary not present at {} \
                 (skipped via --allow-missing-bootloader)",
                target_path.display()
            );
            return (errors, None);
        }
        errors.push(format!(
            "Missing patched bootloader binary: {} \
             (pass --allow-missing-bootloader to bypass in dev environments)",
            target_path.display()
        ));
        return (errors, None);
    }

    let data = match std::fs::read(target_path) {
        Ok(d) => d,
        Err(e) => {
            errors.push(format!("Failed to read {}: {e}", target_path.display()));
            return (errors, None);
        }
    };

    let len = data.len();
    if !(BOOTLOADER_MIN_BYTES..=BOOTLOADER_MAX_BYTES).contains(&len) || data[0] != 0xE9 {
        errors.push(format!(
            "Invalid ESP32 bootloader header or size ({len} bytes) in {}",
            target_path.display()
        ));
    }
    if !contains_subslice(&data, b"\x4c\x90\xf4\x3f") {
        errors.push(format!(
            "{}: missing IO_MUX_GPIO16_REG literal 0x3FF4904C (\\x4c\\x90\\xf4\\x3f)",
            target_path.display()
        ));
    }

    let valid_gpio16_patterns: &[&[u8]] = &[
        b"\x00\x2b\x00\x00",
        b"\x2c\xb9\x80\x99\x11",
        b"\x0c\xb9\x80\x99\x11",
        b"\x2c\xba\x80\xaa\x11",
        b"\x0c\xba\x80\xaa\x11",
        b"\x2c\xa9\x80\x88\x11",
        b"\x0c\xa9\x80\x88\x11",
    ];
    let has_gpio16_pat = valid_gpio16_patterns
        .iter()
        .any(|pat| contains_subslice(&data, pat));
    if !has_gpio16_pat {
        errors.push(format!(
            "{}: missing GPIO 16 pull-up constant 0x2B00 sequence in bootloader_before_init",
            target_path.display()
        ));
    }

    let reg_literals: &[(&str, &[u8])] = &[
        ("RTC_CNTL_HOLD_FORCE_REG 0x3FF480C8", b"\xc8\x80\xf4\x3f"),
        ("RTC_IO_DIG_PAD_HOLD_REG 0x3FF48474", b"\x74\x84\xf4\x3f"),
        ("GPIO_OUT_W1TS_REG 0x3FF44008", b"\x08\x40\xf4\x3f"),
        (
            "GPIO_FUNC16_OUT_SEL_CFG_REG 0x3FF44570",
            b"\x70\x45\xf4\x3f",
        ),
        ("GPIO_ENABLE_W1TS_REG 0x3FF44024", b"\x24\x40\xf4\x3f"),
    ];
    for (reg_label, reg_bytes) in reg_literals {
        if !contains_subslice(&data, reg_bytes) {
            errors.push(format!(
                "{}: missing {reg_label} in bootloader_before_init",
                target_path.display()
            ));
        }
    }

    (errors, Some(len))
}

/// Verifies that esp32_i2s.rs masks I2S_SAMPLE_RATE_CONF_REG_OFFSET.
#[must_use]
pub fn verify_esp32_i2s_rs(repo_root: &Path) -> Vec<String> {
    let mut errors = Vec::new();
    let i2s_path = repo_root
        .join("crates")
        .join("esp32-cam-ll")
        .join("src")
        .join("esp32_i2s.rs");

    if !i2s_path.is_file() {
        errors.push(format!("Missing esp32_i2s.rs at {}", i2s_path.display()));
        return errors;
    }

    let raw_content = match std::fs::read_to_string(&i2s_path) {
        Ok(s) => s,
        Err(e) => {
            errors.push(format!("Failed to read {}: {e}", i2s_path.display()));
            return errors;
        }
    };
    let content = strip_rust_comments(&raw_content);
    if !content.contains("I2S_SAMPLE_RATE_CONF_REG_OFFSET") {
        errors.push("esp32_i2s.rs does not reference I2S_SAMPLE_RATE_CONF_REG_OFFSET".to_string());
    }
    if !content.contains("& !(0x3F << 18)") {
        errors.push(
            "esp32_i2s.rs does not mask I2S_SAMPLE_RATE_CONF_REG_OFFSET with '& !(0x3F << 18)'"
                .to_string(),
        );
    }

    if errors.is_empty() {
        println!(
            "[PASS] esp32_i2s.rs ({}) masks I2S_SAMPLE_RATE_CONF_REG_OFFSET with & !(0x3F << 18)",
            i2s_path.display()
        );
    }
    errors
}

/// Verifies the live HTTP status endpoint.
#[must_use]
pub fn verify_live_status(url: &str) -> Vec<String> {
    let mut errors = Vec::new();
    if !url.starts_with("http://") {
        errors.push(format!("Invalid live status URL scheme: {url}"));
        return errors;
    }

    let without_scheme = &url[7..];
    let (host_port, path) = match without_scheme.find('/') {
        Some(idx) => (&without_scheme[..idx], &without_scheme[idx..]),
        None => (without_scheme, "/"),
    };

    let host_port_with_default = if host_port.contains(':') {
        host_port.to_string()
    } else {
        format!("{host_port}:80")
    };

    let host_only = host_port.split(':').next().unwrap_or(host_port);

    let default_addr = std::net::SocketAddr::V4(std::net::SocketAddrV4::new(
        std::net::Ipv4Addr::LOCALHOST,
        80,
    ));
    let stream_res = TcpStream::connect_timeout(
        &host_port_with_default.parse().unwrap_or_else(|_| {
            // Fallback DNS lookup
            use std::net::ToSocketAddrs;
            host_port_with_default
                .to_socket_addrs()
                .ok()
                .and_then(|mut iter| iter.next())
                .unwrap_or(default_addr)
        }),
        Duration::from_secs(3),
    );

    let mut stream = match stream_res {
        Ok(s) => s,
        Err(e) => {
            errors.push(format!("Failed to query live status from {url}: {e}"));
            return errors;
        }
    };
    let _ = stream.set_read_timeout(Some(Duration::from_secs(3)));
    let _ = stream.set_write_timeout(Some(Duration::from_secs(3)));

    let request = format!("GET {path} HTTP/1.1\r\nHost: {host_only}\r\nConnection: close\r\n\r\n");
    if let Err(e) = stream.write_all(request.as_bytes()) {
        errors.push(format!("Failed to write HTTP request to {url}: {e}"));
        return errors;
    }

    let mut resp = Vec::new();
    if let Err(e) = stream.read_to_end(&mut resp) {
        errors.push(format!("Failed to read HTTP response from {url}: {e}"));
        return errors;
    }

    let resp_str = String::from_utf8_lossy(&resp);
    let mut lines = resp_str.lines();
    let status_line = lines.next().unwrap_or("");
    if !status_line.contains("200") {
        errors.push(format!("Live status endpoint {url} returned {status_line}"));
        return errors;
    }

    let body = match resp_str.find("\r\n\r\n") {
        Some(idx) => &resp_str[idx + 4..],
        None => &resp_str,
    };

    let uptime = body.contains("\"uptime_ms\"");
    let partition = body.contains("\"active_partition\"");
    if !uptime {
        errors.push(format!("Live status from {url} missing 'uptime_ms'"));
    }
    if !partition {
        errors.push(format!("Live status from {url} missing 'active_partition'"));
    }

    if uptime && partition {
        println!("[PASS] Live status from {url} verified.");
    }

    errors
}

/// Executes the full invariant verification gate suite.
///
/// # Errors
/// Returns an error string containing all failures if any invariant checks
/// fail.
pub fn run(args: &[String]) -> Result<(), String> {
    let mut allow_missing_bootloader = false;
    let mut live_status_url: Option<String> = std::env::var("CAM_STATUS_URL").ok().or_else(|| {
        std::env::var("CAM_HOST")
            .ok()
            .map(|h| format!("http://{h}/status"))
    });
    let mut repo_root = std::env::current_dir().unwrap_or_else(|_| PathBuf::from("."));

    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "--allow-missing-bootloader" => {
                allow_missing_bootloader = true;
                i += 1;
            }
            "--live-status-url" => {
                if i + 1 < args.len() {
                    live_status_url = Some(args[i + 1].clone());
                    i += 2;
                } else {
                    return Err("Missing value for --live-status-url".to_string());
                }
            }
            "--repo-root" => {
                if i + 1 < args.len() {
                    repo_root = PathBuf::from(&args[i + 1]);
                    i += 2;
                } else {
                    return Err("Missing value for --repo-root".to_string());
                }
            }
            other => {
                if let Some(url) = other.strip_prefix("--live-status-url=") {
                    live_status_url = Some(url.to_string());
                } else if let Some(path) = other.strip_prefix("--repo-root=") {
                    repo_root = PathBuf::from(path);
                } else if other == "-h" || other == "--help" {
                    println!(
                        "Usage: cargo xtask verify-invariants \
                         [--allow-missing-bootloader] [--live-status-url <URL>] \
                         [--repo-root <PATH>]"
                    );
                    return Ok(());
                } else {
                    return Err(format!("Unknown option '{other}' for verify-invariants"));
                }
                i += 1;
            }
        }
    }

    // Ensure repo_root points to the workspace root containing Cargo.toml.
    if !repo_root.join("Cargo.toml").is_file() && repo_root.join("..").join("Cargo.toml").is_file()
    {
        if let Some(parent) = repo_root.parent() {
            repo_root = parent.to_path_buf();
        }
    }

    let mut all_errors = Vec::new();
    all_errors.extend(verify_udev_rules(&repo_root));
    all_errors.extend(verify_main_rs(&repo_root));
    all_errors.extend(verify_bootloader_sources(&repo_root));
    all_errors.extend(verify_rf_clock_sources(&repo_root));
    all_errors.extend(verify_wifi_mac_sources(&repo_root));

    let bootloader_path = repo_root
        .join("target")
        .join("xtensa-esp32-none-elf")
        .join("release")
        .join("esp32-cam-bootloader.bin");
    let (boot_errs, boot_size) = verify_bootloader_bin(&bootloader_path, allow_missing_bootloader);
    all_errors.extend(boot_errs);

    all_errors.extend(verify_esp32_i2s_rs(&repo_root));

    if let Some(url) = &live_status_url {
        all_errors.extend(verify_live_status(url));
    }

    if !all_errors.is_empty() {
        for err in &all_errors {
            eprintln!("[FAIL] {err}");
        }
        return Err(format!(
            "Verification failed with {} error(s)",
            all_errors.len()
        ));
    }

    println!(
        "[PASS] Invariant 69: /etc/udev/rules.d/99-esp32-wrover.rules & \
         docs/99-esp32-wrover.rules are clean (zero RUN+=/stty/TIOCM* hooks)."
    );
    println!(
        "[PASS] Invariant 70/71/72 (main.rs): zero SD0..SD3 bit-banging, \
         arm_rtc_wdt_15s(), hardware SPI1 0xF5+0x35 (PsramCacheF40mS40m), \
         zero boot-time FlashStorage I/O, zero GPIO 4 output contention, \
         MMU active OTA slot detection, and authoritative MMIO map verified."
    );
    println!(
        "[PASS] Invariant 70/71/72 (esp32-cam-bootloader): GPIO 16 pull-up, \
         RTC_CNTL_WDTCONFIG0_REG arming, zero GPIO 4/7..10 violations, and \
         all MMIO declarations verified."
    );
    println!(
        "[PASS] Invariant 70/71/72 (esp32-cam-rf-clock): all 6 modules present, \
         zero GPIO 4/7..10 violations, sens_reg.h + REA-proven COEX MMIO map verified."
    );
    println!(
        "[PASS] Invariant 70/71/72 (esp32-cam-wifi-mac): regs.rs + modules present, \
         zero GPIO 4/7..10 violations, soc/dport_reg.h + REA Wi-Fi MAC MMIO map verified."
    );
    if let Some(size) = boot_size {
        println!(
            "[PASS] Invariant 70 (bootloader): {} ({size} B) contains \
             0x3FF4_904C & 0x2B00 GPIO 16 pull-up.",
            bootloader_path.display()
        );
    }
    if let Some(url) = &live_status_url {
        println!("[PASS] Live device cold-boot / runtime status verified via {url}.");
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    struct TempDirGuard(PathBuf);

    impl Drop for TempDirGuard {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    #[test]
    fn test_authoritative_register_count() {
        assert_eq!(
            AUTHORITATIVE_MMIO_REGISTERS.len(),
            94,
            "expected 94 authoritative MMIO registers"
        );
    }

    #[test]
    fn test_rea_proven_wifi_mac_register_count() {
        assert_eq!(
            REA_PROVEN_WIFI_MAC_MMIO_REGISTERS.len(),
            59,
            "expected 59 REA-proven Wi-Fi MAC MMIO registers"
        );
    }

    #[test]
    fn test_verify_bootloader_sources_clock_spi_psram_rules() {
        let unique = format!(
            "esp32_bl_inv_test_{}_{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map_or(0, |d| d.as_nanos())
        );
        let temp_root = std::env::temp_dir().join(unique);
        let _guard = TempDirGuard(temp_root.clone());

        let src_dir = temp_root
            .join("crates")
            .join("esp32-cam-bootloader")
            .join("src");
        std::fs::create_dir_all(&src_dir).expect("create temp bootloader src dir");

        std::fs::write(
            src_dir.join("gpio16.rs"),
            "pub const IO_MUX_GPIO16_REG: u32 = 0x3FF4_904C;\n\
             pub const PULLUP: u32 = 0x2B00;\n\
             fn init() { write_u32(IO_MUX_GPIO16_REG, PULLUP); }\n",
        )
        .expect("write gpio16.rs");

        std::fs::write(
            src_dir.join("wdt.rs"),
            "pub const RTC_CNTL_WDTCONFIG0_REG: u32 = 0x3FF4_808C;\n\
             fn arm() { write_u32(RTC_CNTL_WDTCONFIG0_REG, 1); }\n",
        )
        .expect("write wdt.rs");

        // clock_spi_psram.rs allows IO_MUX_GPIO7_REG, but rejects
        // GPIO_FUNC7_OUT_SEL_CFG_REG and rejects GPIO_ENABLE tokens.
        std::fs::write(
            src_dir.join("clock_spi_psram.rs"),
            "pub const IO_MUX_GPIO7_REG: u32 = 0x3FF4_9064;\n\
             pub const GPIO_FUNC7_OUT_SEL_CFG_REG: u32 = 0x3FF4_4570;\n\
             fn bad_enable() { write_u32(GPIO_ENABLE_W1TS_REG, 1); }\n",
        )
        .expect("write clock_spi_psram.rs");

        // other.rs with IO_MUX_GPIO8_REG is rejected.
        std::fs::write(
            src_dir.join("other.rs"),
            "pub const IO_MUX_GPIO8_REG: u32 = 0x3FF4_9068;\n",
        )
        .expect("write other.rs");

        let errors = verify_bootloader_sources(&temp_root);
        assert!(
            errors
                .iter()
                .any(|e| e.contains("clock_spi_psram.rs")
                    && e.contains("GPIO_FUNC7_OUT_SEL_CFG_REG")),
            "expected GPIO_FUNC rejection in clock_spi_psram.rs, got: {errors:?}"
        );
        assert!(
            errors
                .iter()
                .any(|e| e.contains("clock_spi_psram.rs") && e.contains("GPIO_ENABLE")),
            "expected GPIO_ENABLE rejection in clock_spi_psram.rs, got: {errors:?}"
        );
        assert!(
            !errors.iter().any(
                |e| e.contains("clock_spi_psram.rs") && e.contains("symbol 'IO_MUX_GPIO7_REG'")
            ),
            "IO_MUX_GPIO7_REG should be permitted in clock_spi_psram.rs, got: {errors:?}"
        );
        assert!(
            errors
                .iter()
                .any(|e| e.contains("other.rs") && e.contains("IO_MUX_GPIO8_REG")),
            "expected IO_MUX_GPIO8_REG rejection in other.rs, got: {errors:?}"
        );
    }

    #[test]
    fn test_rea_proven_rf_coex_register_count() {
        assert_eq!(
            REA_PROVEN_RF_COEX_MMIO_REGISTERS.len(),
            8,
            "expected 8 REA-proven RF COEX MMIO registers"
        );
    }

    #[test]
    fn test_verify_rf_clock_sources_clean() {
        let repo_root = Path::new(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .and_then(Path::parent)
            .expect("xtask crate has workspace root grandparent");
        let errors = verify_rf_clock_sources(repo_root);
        assert!(
            errors.is_empty(),
            "esp32-cam-rf-clock sources failed verification: {errors:?}"
        );
    }

    #[test]
    fn test_verify_rf_clock_sources_rejects_violations() {
        let unique = format!(
            "esp32_rf_clock_inv_test_{}_{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map_or(0, |d| d.as_nanos())
        );
        let temp_root = std::env::temp_dir().join(unique);
        let _guard = TempDirGuard(temp_root.clone());

        let src_dir = temp_root
            .join("crates")
            .join("esp32-cam-rf-clock")
            .join("src");
        std::fs::create_dir_all(&src_dir).expect("create temp rf-clock src dir");

        for module in [
            "lib.rs",
            "printf_compat.rs",
            "printf_fmt.rs",
            "regulatory.rs",
        ] {
            std::fs::write(src_dir.join(module), "// clean stub\n").expect("write clean stub");
        }
        std::fs::write(
            src_dir.join("nvs_misc.rs"),
            "pub const COEX_SLC_0x3FF5C080_REG: u32 = 0x3FF5_C080;\n\
             pub const IO_MUX_GPIO7_REG: u32 = 0x3FF4_9060;\n\
             fn bad() { write_u32(GPIO_ENABLE_W1TS_REG, 1 << 4); }\n",
        )
        .expect("write bad nvs_misc.rs");
        std::fs::write(
            src_dir.join("rtc_rf.rs"),
            "pub const SENS_SAR_TSENS_CTRL_REG: u32 = 0x3FF4_0000;\n\
             pub const COEX_SLC_0x3FF5C080_REG: u32 = 0x3FF5_FFFF;\n\
             pub const UNLISTED_FAKE_REG: u32 = 0x3FF9_9999;\n",
        )
        .expect("write bad rtc_rf.rs");

        // Remove `regulatory.rs` to also verify missing-module detection.
        std::fs::remove_file(src_dir.join("regulatory.rs")).expect("remove regulatory.rs");

        let errors = verify_rf_clock_sources(&temp_root);
        assert!(
            errors
                .iter()
                .any(|e| e.contains("missing required module regulatory.rs")),
            "expected missing module error, got: {errors:?}"
        );
        assert!(
            errors
                .iter()
                .any(|e| e.contains("nvs_misc.rs") && e.contains("COEX_SLC_0x3FF5C080_REG")),
            "expected coex reg rejection outside rtc_rf.rs, got: {errors:?}"
        );
        assert!(
            errors.iter().any(|e| e.contains("IO_MUX_GPIO7_REG")),
            "expected SD pin rejection, got: {errors:?}"
        );
        assert!(
            errors.iter().any(|e| e.contains("GPIO 4")),
            "expected GPIO 4 contention rejection, got: {errors:?}"
        );
        assert!(
            errors
                .iter()
                .any(|e| e.contains("SENS_SAR_TSENS_CTRL_REG") && e.contains("mismatch")),
            "expected SENS_SAR_TSENS_CTRL_REG mismatch error, got: {errors:?}"
        );
        assert!(
            errors
                .iter()
                .any(|e| e.contains("COEX_SLC_0x3FF5C080_REG") && e.contains("mismatch")),
            "expected COEX_SLC mismatch error, got: {errors:?}"
        );
        assert!(
            errors.iter().any(|e| e.contains("UNLISTED_FAKE_REG")),
            "expected unlisted register error, got: {errors:?}"
        );
        assert!(
            errors
                .iter()
                .any(|e| e.contains("SENS_SAR_SLAVE_ADDR3_REG")),
            "expected missing SENS_SAR_SLAVE_ADDR3_REG error, got: {errors:?}"
        );
    }

    #[test]
    fn test_comment_stripping() {
        let rust_code = r#"
            // Comment: const DPORT_HOST_INF_SEL_REG: u32 = 0x0000_0000;
            /* Block comment with 0xDEADBEEF */
            const DPORT_HOST_INF_SEL_REG: *const u32 = 0x3FF0_00BC as *const u32;
            let msg = "/* string */ // string";
        "#;
        let stripped = strip_rust_comments(rust_code);
        assert!(!stripped.contains("0x0000_0000"));
        assert!(!stripped.contains("0xDEADBEEF"));
        assert!(stripped.contains("0x3FF0_00BC"));
        assert!(stripped.contains("/* string */ // string"));
    }
}
