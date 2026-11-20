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

const FORBIDDEN_GPIO_FUNC_SD_PINS: [(&str, u32); 4] = [
    ("GPIO_FUNC7_OUT_SEL_CFG_REG", 0x3FF4_454C),
    ("GPIO_FUNC8_OUT_SEL_CFG_REG", 0x3FF4_4550),
    ("GPIO_FUNC9_OUT_SEL_CFG_REG", 0x3FF4_4554),
    ("GPIO_FUNC10_OUT_SEL_CFG_REG", 0x3FF4_4558),
];

const FORBIDDEN_IO_MUX_SD_PINS: [(&str, u32); 4] = [
    ("IO_MUX_GPIO7_REG", 0x3FF4_9064),
    ("IO_MUX_GPIO8_REG", 0x3FF4_9068),
    ("IO_MUX_GPIO9_REG", 0x3FF4_9054),
    ("IO_MUX_GPIO10_REG", 0x3FF4_9058),
];

const GPIO4_CONTENTION_TOKENS: [&str; 4] = [
    "GPIO_OUT_W1TS_REG",
    "GPIO_OUT_W1TC_REG",
    "GPIO_ENABLE_REG",
    "GPIO_ENABLE_W1TS_REG",
];
const GPIO4_CONTENTION_RAW_ADDRS: [u32; 4] = [0x3FF4_4008, 0x3FF4_400C, 0x3FF4_4020, 0x3FF4_4024];
const GPIO_ENABLE_RAW_ADDRS: [u32; 3] = [0x3FF4_4020, 0x3FF4_4024, 0x3FF4_4028];

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
    ("UART0_STATUS_REG", 0x3FF4_001C),
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

/// Advances `i` past a Rust string literal (`"..."`) or raw string literal
/// (`r#"..."#`) starting at `bytes[i]`, returning the new index if matched.
fn skip_rust_string_literal(bytes: &[u8], start: usize) -> Option<usize> {
    let len = bytes.len();
    if start >= len {
        return None;
    }
    if bytes[start] == b'"' {
        let mut i = start + 1;
        while i < len {
            if bytes[i] == b'\\' && i + 1 < len {
                i += 2;
            } else if bytes[i] == b'"' {
                return Some(i + 1);
            } else {
                i += 1;
            }
        }
        return Some(len);
    }
    if bytes[start] == b'r'
        && start + 1 < len
        && (bytes[start + 1] == b'"' || bytes[start + 1] == b'#')
    {
        let prev_is_ident =
            start > 0 && (bytes[start - 1].is_ascii_alphanumeric() || bytes[start - 1] == b'_');
        if prev_is_ident && bytes[start - 1] != b'b' {
            return None;
        }
        let mut i = start + 1;
        let mut hashes = 0usize;
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
                        return Some(after_quote + hashes);
                    }
                }
                i += 1;
            }
            return Some(len);
        }
    }
    None
}

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
        if let Some(next_i) = skip_rust_string_literal(bytes, i) {
            i = next_i;
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

/// Returns the end byte offset of a `#[...]` attribute starting at `bytes[i]`.
fn parse_attribute_end(bytes: &[u8], start: usize) -> Option<usize> {
    if start + 1 >= bytes.len() || bytes[start] != b'#' || bytes[start + 1] != b'[' {
        return None;
    }
    let mut i = start + 2;
    let mut depth = 1usize;
    while i < bytes.len() && depth > 0 {
        if let Some(next_i) = skip_rust_string_literal(bytes, i) {
            i = next_i;
            continue;
        }
        match bytes[i] {
            b'[' => depth += 1,
            b']' => depth -= 1,
            _ => {}
        }
        i += 1;
    }
    if depth == 0 {
        Some(i)
    } else {
        None
    }
}

/// Checks whether an attribute body (inside `#[...]`) is `cfg(test)`.
fn is_cfg_test_attr(attr_inner: &str) -> bool {
    let compact: String = attr_inner.chars().filter(|c| !c.is_whitespace()).collect();
    compact == "cfg(test)"
}

/// Strips `#[cfg(test)]`-annotated modules and items from comment-stripped
/// Rust source so unit test fixtures cannot spoof or trip hardware invariants.
#[must_use]
pub fn strip_cfg_test_sections(src: &str) -> String {
    let bytes = src.as_bytes();
    let len = bytes.len();
    let mut out = String::with_capacity(len);
    let mut i = 0usize;
    let mut last_copied = 0usize;

    while i < len {
        if let Some(next_i) = skip_rust_string_literal(bytes, i) {
            i = next_i;
            continue;
        }
        if bytes[i] == b'#' {
            if let Some(attr_end) = parse_attribute_end(bytes, i) {
                let inner = &src[i + 2..attr_end - 1];
                if is_cfg_test_attr(inner) {
                    out.push_str(&src[last_copied..i]);
                    let mut j = attr_end;
                    // Skip any chained attributes before the annotated item.
                    loop {
                        while j < len && bytes[j].is_ascii_whitespace() {
                            j += 1;
                        }
                        if j < len && bytes[j] == b'#' {
                            if let Some(next_attr_end) = parse_attribute_end(bytes, j) {
                                j = next_attr_end;
                                continue;
                            }
                        }
                        break;
                    }
                    // Advance to the item's opening `{` or terminating `;`.
                    while j < len {
                        if let Some(next_j) = skip_rust_string_literal(bytes, j) {
                            j = next_j;
                            continue;
                        }
                        if bytes[j] == b';' {
                            j += 1;
                            break;
                        }
                        if bytes[j] == b'{' {
                            let mut brace_depth = 1usize;
                            j += 1;
                            while j < len && brace_depth > 0 {
                                if let Some(next_j) = skip_rust_string_literal(bytes, j) {
                                    j = next_j;
                                    continue;
                                }
                                match bytes[j] {
                                    b'{' => brace_depth += 1,
                                    b'}' => brace_depth -= 1,
                                    _ => {}
                                }
                                j += 1;
                            }
                            break;
                        }
                        j += 1;
                    }
                    i = j;
                    last_copied = j;
                    continue;
                }
            }
        }
        i += 1;
    }

    if last_copied < len {
        out.push_str(&src[last_copied..len]);
    }
    out
}

/// Replaces string and raw-string literal contents with empty `""` so log
/// strings cannot spoof code invariants (`M-129`).
#[must_use]
fn strip_string_literals(src: &str) -> String {
    let bytes = src.as_bytes();
    let len = bytes.len();
    let mut out = String::with_capacity(len);
    let mut i = 0usize;
    let mut last_copied = 0usize;

    while i < len {
        if let Some(next_i) = skip_rust_string_literal(bytes, i) {
            out.push_str(&src[last_copied..i]);
            out.push_str("\"\"");
            i = next_i;
            last_copied = next_i;
        } else {
            i += 1;
        }
    }
    if last_copied < len {
        out.push_str(&src[last_copied..len]);
    }
    out
}

/// Strips comments and `#[cfg(test)]` sections from Rust source.
fn strip_comments_and_tests(raw: &str) -> String {
    let no_comments = strip_rust_comments(raw);
    strip_cfg_test_sections(&no_comments)
}

/// Extracts the brace-balanced body `{ ... }` of a function matching `sig`.
fn extract_fn_body<'a>(src: &'a str, sig: &str) -> Option<&'a str> {
    let sig_pos = src.find(sig)?;
    let bytes = src.as_bytes();
    let mut i = sig_pos + sig.len();
    while i < bytes.len() && bytes[i] != b'{' {
        if let Some(next_i) = skip_rust_string_literal(bytes, i) {
            i = next_i;
            continue;
        }
        i += 1;
    }
    if i >= bytes.len() || bytes[i] != b'{' {
        return None;
    }
    let body_start = i + 1;
    i += 1;
    let mut depth = 1usize;
    while i < bytes.len() && depth > 0 {
        if let Some(next_i) = skip_rust_string_literal(bytes, i) {
            i = next_i;
            continue;
        }
        match bytes[i] {
            b'{' => depth += 1,
            b'}' => depth -= 1,
            _ => {}
        }
        i += 1;
    }
    if depth == 0 {
        Some(&src[body_start..i - 1])
    } else {
        None
    }
}

/// Extracts all hex integer literals (`0x...`) from `src` with their parsed
/// `u32` values.
fn extract_hex_u32_literals(src: &str) -> Vec<(usize, u32)> {
    let bytes = src.as_bytes();
    let mut out = Vec::new();
    let mut i = 0usize;
    while i + 2 < bytes.len() {
        if let Some(next_i) = skip_rust_string_literal(bytes, i) {
            i = next_i;
            continue;
        }
        if bytes[i] == b'0' && (bytes[i + 1] == b'x' || bytes[i + 1] == b'X') {
            let prev_ok = i == 0 || !(bytes[i - 1].is_ascii_alphanumeric() || bytes[i - 1] == b'_');
            if prev_ok {
                let mut end = i + 2;
                while end < bytes.len() && (bytes[end].is_ascii_hexdigit() || bytes[end] == b'_') {
                    end += 1;
                }
                let clean: String = src[i + 2..end].chars().filter(|&c| c != '_').collect();
                if !clean.is_empty() {
                    if let Ok(val) = u32::from_str_radix(&clean, 16) {
                        out.push((i, val));
                    }
                }
                i = end;
                continue;
            }
        }
        i += 1;
    }
    out
}

/// Returns `true` if `addr` falls within the ESP32 or ESP32-S3 MMIO ranges.
fn is_esp32_mmio_addr(addr: u32) -> bool {
    (0x3FF0_0000..0x3FFA_0000).contains(&addr) || (0x6000_0000..0x6005_0000).contains(&addr)
}

/// Parses a single integer literal (`0x...`, `0b...`, or decimal) stripping
/// underscores and integer type suffixes (`u32`, `usize`, etc.).
fn parse_rust_u32_literal(raw: &str) -> Option<u32> {
    let mut s = raw.trim();
    for suffix in ["u32", "usize", "u64", "u16", "u8", "i32", "isize"] {
        if let Some(stripped) = s.strip_suffix(suffix) {
            s = stripped.trim_end_matches('_');
            break;
        }
    }
    if let Some(hex) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        let clean: String = hex.chars().filter(|&c| c != '_').collect();
        u32::from_str_radix(&clean, 16).ok()
    } else if let Some(bin) = s.strip_prefix("0b").or_else(|| s.strip_prefix("0B")) {
        let clean: String = bin.chars().filter(|&c| c != '_').collect();
        u32::from_str_radix(&clean, 2).ok()
    } else {
        let clean: String = s.chars().filter(|&c| c != '_').collect();
        clean.parse::<u32>().ok()
    }
}

/// Strips a single top-level trailing `as <scalar/pointer type>` cast from an
/// expression, returning the LHS operand if matched.
fn strip_trailing_as_cast(expr: &str) -> Option<&str> {
    let bytes = expr.as_bytes();
    let mut depth = 0usize;
    let mut last_as: Option<usize> = None;
    let mut i = 0usize;
    while i + 2 <= bytes.len() {
        match bytes[i] {
            b'(' => depth += 1,
            b')' => depth = depth.saturating_sub(1),
            b'a' if depth == 0 && i + 1 < bytes.len() && bytes[i + 1] == b's' => {
                let prev_ok = i > 0 && (bytes[i - 1].is_ascii_whitespace() || bytes[i - 1] == b')');
                let next_ok = i + 2 < bytes.len() && bytes[i + 2].is_ascii_whitespace();
                if prev_ok && next_ok {
                    last_as = Some(i);
                }
            }
            _ => {}
        }
        i += 1;
    }
    let as_idx = last_as?;
    let ty = expr[as_idx + 2..].trim();
    let is_supported_ty = matches!(ty, "u32" | "usize" | "u64" | "u16" | "u8" | "i32" | "isize")
        || ty.starts_with("*mut ")
        || ty.starts_with("*const ");
    if is_supported_ty {
        Some(expr[..as_idx].trim())
    } else {
        None
    }
}

/// Splits `expr` on a top-level single-character binary operator `op` (avoiding
/// doubled operators such as `||` or `&&`).
fn split_top_level_single_op(expr: &str, op: u8) -> Vec<&str> {
    let bytes = expr.as_bytes();
    let mut depth = 0usize;
    let mut parts = Vec::new();
    let mut start = 0usize;
    let mut idx = 0usize;
    while idx < bytes.len() {
        match bytes[idx] {
            b'(' => depth += 1,
            b')' => depth = depth.saturating_sub(1),
            b if b == op && depth == 0 => {
                let is_double = (idx > 0 && bytes[idx - 1] == op)
                    || (idx + 1 < bytes.len() && bytes[idx + 1] == op);
                if !is_double {
                    parts.push(&expr[start..idx]);
                    start = idx + 1;
                }
            }
            _ => {}
        }
        idx += 1;
    }
    if !parts.is_empty() {
        parts.push(&expr[start..]);
    }
    parts
}

/// Evaluates simple constant `u32` expressions (`a | b`, `a & b`, `a << b`,
/// `a + b`, `as` casts, parens, literals, and previously resolved constants).
fn eval_const_u32_expr(expr: &str, env: &BTreeMap<String, u32>) -> Option<u32> {
    let trimmed = expr.trim();
    if trimmed.is_empty() {
        return None;
    }
    // Strip outer balanced parentheses.
    if trimmed.starts_with('(') && trimmed.ends_with(')') {
        let mut depth = 0usize;
        let mut wraps_all = true;
        for (idx, ch) in trimmed.char_indices() {
            match ch {
                '(' => depth += 1,
                ')' => {
                    depth -= 1;
                    if depth == 0 && idx + 1 < trimmed.len() {
                        wraps_all = false;
                        break;
                    }
                }
                _ => {}
            }
        }
        if wraps_all {
            return eval_const_u32_expr(&trimmed[1..trimmed.len() - 1], env);
        }
    }

    // Precedence (lowest to highest): `|` < `&` < `<<` < `+` < `as`.
    let or_parts = split_top_level_single_op(trimmed, b'|');
    if !or_parts.is_empty() {
        let mut acc = 0u32;
        for part in or_parts {
            acc |= eval_const_u32_expr(part, env)?;
        }
        return Some(acc);
    }

    let and_parts = split_top_level_single_op(trimmed, b'&');
    if !and_parts.is_empty() {
        let mut iter = and_parts.into_iter();
        let first = iter.next()?;
        let mut acc = eval_const_u32_expr(first, env)?;
        for part in iter {
            acc &= eval_const_u32_expr(part, env)?;
        }
        return Some(acc);
    }

    // Split on top-level `<<`.
    let bytes = trimmed.as_bytes();
    let mut depth = 0usize;
    for idx in 0..bytes.len().saturating_sub(1) {
        match bytes[idx] {
            b'(' => depth += 1,
            b')' => depth = depth.saturating_sub(1),
            b'<' if depth == 0 && bytes[idx + 1] == b'<' => {
                let lhs = eval_const_u32_expr(&trimmed[..idx], env)?;
                let rhs = eval_const_u32_expr(&trimmed[idx + 2..], env)?;
                return lhs.checked_shl(rhs);
            }
            _ => {}
        }
    }

    // Split on top-level `+` (base + offset folding for M-128).
    let add_parts = split_top_level_single_op(trimmed, b'+');
    if !add_parts.is_empty() {
        let mut acc = 0u32;
        for part in add_parts {
            acc = acc.checked_add(eval_const_u32_expr(part, env)?)?;
        }
        return Some(acc);
    }

    // Strip trailing `as <ty>` cast (`u32`, `usize`, `*mut u32`, `*const u32`).
    if let Some(lhs) = strip_trailing_as_cast(trimmed) {
        return eval_const_u32_expr(lhs, env);
    }

    if let Some(val) = parse_rust_u32_literal(trimmed) {
        return Some(val);
    }
    env.get(trimmed).copied()
}

/// Collects `const`/`static` constant `u32` bindings from comment-stripped
/// Rust source, iterating to a fixed point so forward references fold cleanly.
fn collect_u32_consts(src: &str) -> BTreeMap<String, u32> {
    let mut map = BTreeMap::new();
    let bytes = src.as_bytes();
    for _pass in 0..4 {
        let mut changed = false;
        for kw in ["const ", "static "] {
            let mut cursor = 0usize;
            while let Some(pos) = src[cursor..].find(kw) {
                let kw_start = cursor + pos;
                let decl_start = kw_start + kw.len();
                cursor = decl_start;
                if kw_start > 0
                    && (bytes[kw_start - 1].is_ascii_alphanumeric() || bytes[kw_start - 1] == b'_')
                {
                    continue;
                }
                let rest = src[decl_start..].trim_start();
                let rest = rest.strip_prefix("mut ").map_or(rest, str::trim_start);
                let Some(semi) = rest.find(';') else {
                    continue;
                };
                let stmt = &rest[..semi];
                let Some((lhs, rhs)) = stmt.split_once('=') else {
                    continue;
                };
                let Some((name_part, _ty_part)) = lhs.split_once(':') else {
                    continue;
                };
                let name = name_part.trim();
                if name.is_empty() || !name.chars().all(|c| c.is_ascii_alphanumeric() || c == '_') {
                    continue;
                }
                if !map.contains_key(name) {
                    if let Some(val) = eval_const_u32_expr(rhs, &map) {
                        map.insert(name.to_string(), val);
                        changed = true;
                    }
                }
            }
        }
        if !changed {
            break;
        }
    }
    map
}

/// Collects local `let <ident> [: <ty>] = <const_expr>;` bindings within the
/// enclosing function up to the end of `prefix`, shadowing `base_env`.
fn collect_local_let_u32_bindings(
    prefix: &str,
    base_env: &BTreeMap<String, u32>,
) -> BTreeMap<String, u32> {
    let mut env = base_env.clone();
    let fn_start = prefix.rfind("fn ").unwrap_or(0);
    let scope = &prefix[fn_start..];
    let bytes = scope.as_bytes();
    let mut cursor = 0usize;
    while let Some(pos) = scope[cursor..].find("let ") {
        let kw_start = cursor + pos;
        let decl_start = kw_start + 4;
        cursor = decl_start;
        if kw_start > 0
            && (bytes[kw_start - 1].is_ascii_alphanumeric() || bytes[kw_start - 1] == b'_')
        {
            continue;
        }
        let rest = scope[decl_start..].trim_start();
        let rest = rest.strip_prefix("mut ").map_or(rest, str::trim_start);
        let Some(semi) = rest.find(';') else {
            continue;
        };
        let stmt = &rest[..semi];
        let Some((lhs, rhs)) = stmt.split_once('=') else {
            continue;
        };
        let name = lhs.split(':').next().unwrap_or("").trim();
        if name.is_empty() || !name.chars().all(|c| c.is_ascii_alphanumeric() || c == '_') {
            continue;
        }
        if let Some(val) = eval_const_u32_expr(rhs, &env) {
            env.insert(name.to_string(), val);
        } else {
            env.remove(name);
        }
    }
    env
}

/// Extracts `(call_pos, arg1, arg2)` tuples for all two-argument calls to
/// `fn_name(arg1, arg2)` in `src`.
fn extract_two_arg_calls<'a>(src: &'a str, fn_name: &str) -> Vec<(usize, &'a str, &'a str)> {
    let mut out = Vec::new();
    let needle = format!("{fn_name}(");
    let bytes_all = src.as_bytes();
    let mut cursor = 0usize;
    while let Some(pos) = src[cursor..].find(&needle) {
        let call_pos = cursor + pos;
        let arg_start = call_pos + needle.len();
        cursor = arg_start;
        if call_pos > 0
            && (bytes_all[call_pos - 1].is_ascii_alphanumeric() || bytes_all[call_pos - 1] == b'_')
        {
            continue;
        }
        let rest = &src[arg_start..];
        let bytes = rest.as_bytes();
        let mut depth = 1usize;
        let mut comma_pos = None;
        let mut end_pos = None;
        let mut i = 0usize;
        while i < bytes.len() && depth > 0 {
            if let Some(next_i) = skip_rust_string_literal(bytes, i) {
                i = next_i;
                continue;
            }
            match bytes[i] {
                b'(' => depth += 1,
                b')' => {
                    depth -= 1;
                    if depth == 0 {
                        end_pos = Some(i);
                        break;
                    }
                }
                b',' if depth == 1 && comma_pos.is_none() => {
                    comma_pos = Some(i);
                }
                _ => {}
            }
            i += 1;
        }
        if let (Some(cpos), Some(epos)) = (comma_pos, end_pos) {
            let arg1 = rest[..cpos].trim();
            let arg2 = rest[cpos + 1..epos].trim();
            out.push((call_pos, arg1, arg2));
        }
    }
    out
}

/// Extracts `(call_pos, arg1)` tuples for all one-argument calls to
/// `fn_name(arg1)` in `src`.
fn extract_one_arg_calls<'a>(src: &'a str, fn_name: &str) -> Vec<(usize, &'a str)> {
    let mut out = Vec::new();
    let needle = format!("{fn_name}(");
    let bytes_all = src.as_bytes();
    let mut cursor = 0usize;
    while let Some(pos) = src[cursor..].find(&needle) {
        let call_pos = cursor + pos;
        let arg_start = call_pos + needle.len();
        cursor = arg_start;
        if call_pos > 0
            && (bytes_all[call_pos - 1].is_ascii_alphanumeric() || bytes_all[call_pos - 1] == b'_')
        {
            continue;
        }
        let rest = &src[arg_start..];
        let bytes = rest.as_bytes();
        let mut depth = 1usize;
        let mut has_comma = false;
        let mut end_pos = None;
        let mut i = 0usize;
        while i < bytes.len() && depth > 0 {
            if let Some(next_i) = skip_rust_string_literal(bytes, i) {
                i = next_i;
                continue;
            }
            match bytes[i] {
                b'(' => depth += 1,
                b')' => {
                    depth -= 1;
                    if depth == 0 {
                        end_pos = Some(i);
                        break;
                    }
                }
                b',' if depth == 1 => {
                    has_comma = true;
                }
                _ => {}
            }
            i += 1;
        }
        if !has_comma {
            if let Some(epos) = end_pos {
                out.push((call_pos, rest[..epos].trim()));
            }
        }
    }
    out
}

/// Extracts second-argument slices from `<fn_name>(<first_arg>, <second_arg>)`
/// call sites where the first argument matches `target_reg`.
fn extract_write_second_args<'a>(src: &'a str, fn_name: &str, target_reg: &str) -> Vec<&'a str> {
    extract_two_arg_calls(src, fn_name)
        .into_iter()
        .filter_map(|(_, arg1, arg2)| (arg1 == target_reg).then_some(arg2))
        .collect()
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

/// Parses MMIO register declarations (`const`/`static *_REG` and `*_MMU_TABLE`)
/// from source code, folding base+offset and constant bindings (`M-128`).
#[must_use]
pub fn parse_mmio_declarations(src: &str) -> Vec<(&str, u32, String)> {
    let mut results = Vec::new();
    let consts = collect_u32_consts(src);
    let bytes = src.as_bytes();
    for kw in ["const ", "static "] {
        let mut cursor = 0;
        while let Some(pos) = src[cursor..].find(kw) {
            let kw_start = cursor + pos;
            let decl_start = kw_start + kw.len();
            cursor = decl_start;
            if kw_start > 0
                && (bytes[kw_start - 1].is_ascii_alphanumeric() || bytes[kw_start - 1] == b'_')
            {
                continue;
            }
            let rest = src[decl_start..].trim_start();
            let rest = rest.strip_prefix("mut ").map_or(rest, str::trim_start);

            let Some(name_end) = rest.find(|c: char| c.is_whitespace() || c == ':') else {
                continue;
            };
            let name = &rest[..name_end];
            if !name.ends_with("_REG") && !name.ends_with("_MMU_TABLE") {
                continue;
            }

            let after_name = &rest[name_end..];
            let Some(colon_pos) = after_name.find(':') else {
                continue;
            };
            let after_colon = &after_name[colon_pos + 1..];
            let Some(semi_pos) = after_colon.find(';') else {
                continue;
            };
            let decl_tail = &after_colon[..semi_pos];
            let Some(eq_pos) = decl_tail.find('=') else {
                continue;
            };
            let after_eq = decl_tail[eq_pos + 1..].trim();

            if let Some(addr) = eval_const_u32_expr(after_eq, &consts) {
                let is_hex_lit = after_eq.starts_with("0x") || after_eq.starts_with("0X");
                if !is_hex_lit && !is_esp32_mmio_addr(addr) {
                    continue;
                }
                let raw_hex = if is_hex_lit {
                    let hex_slice = &after_eq[2..];
                    let hex_end = hex_slice
                        .find(|c: char| !c.is_ascii_hexdigit() && c != '_')
                        .unwrap_or(hex_slice.len());
                    format!("0x{}", &hex_slice[..hex_end])
                } else {
                    format!("0x{addr:08X}")
                };
                results.push((name, addr, raw_hex));
            }
        }
    }
    results
}

/// Extracts the LHS operand expression immediately preceding an `as *mut` or
/// `as *const` cast at byte offset `as_pos`, unwinding intermediate scalar
/// casts such as `as usize`.
fn extract_cast_lhs(src: &str, as_pos: usize) -> &str {
    let bytes = src.as_bytes();
    let mut end = as_pos;
    while end > 0 && bytes[end - 1].is_ascii_whitespace() {
        end -= 1;
    }
    let orig_end = end;
    loop {
        let atom_start = if end > 0 && bytes[end - 1] == b')' {
            let mut depth = 0usize;
            let mut i = end;
            while i > 0 {
                i -= 1;
                match bytes[i] {
                    b')' => depth += 1,
                    b'(' => {
                        depth -= 1;
                        if depth == 0 {
                            break;
                        }
                    }
                    _ => {}
                }
            }
            i
        } else {
            let mut i = end;
            while i > 0
                && (bytes[i - 1].is_ascii_alphanumeric()
                    || bytes[i - 1] == b'_'
                    || bytes[i - 1] == b':')
            {
                i -= 1;
            }
            i
        };
        let atom = &src[atom_start..end];
        let mut prev = atom_start;
        while prev > 0 && bytes[prev - 1].is_ascii_whitespace() {
            prev -= 1;
        }
        let preceded_by_as = prev >= 2
            && &bytes[prev - 2..prev] == b"as"
            && (prev == 2 || !(bytes[prev - 3].is_ascii_alphanumeric() || bytes[prev - 3] == b'_'));
        if matches!(atom, "usize" | "u32" | "u64" | "isize" | "i32") && preceded_by_as {
            end = prev - 2;
            while end > 0 && bytes[end - 1].is_ascii_whitespace() {
                end -= 1;
            }
            continue;
        }
        return src[atom_start..orig_end].trim();
    }
}

/// Extracts folded `<base> + <offset>` MMIO addresses across `src`, including
/// `const` and local `let` bindings (`M-128`).
fn extract_folded_mmio_exprs(src: &str, consts: &BTreeMap<String, u32>) -> Vec<(usize, u32)> {
    let bytes = src.as_bytes();
    let mut out = Vec::new();
    let mut i = 0usize;
    while i < bytes.len() {
        if let Some(next_i) = skip_rust_string_literal(bytes, i) {
            i = next_i;
            continue;
        }
        if bytes[i] == b'+' {
            let lhs = extract_cast_lhs(src, i);
            let mut rhs_start = i + 1;
            while rhs_start < bytes.len() && bytes[rhs_start].is_ascii_whitespace() {
                rhs_start += 1;
            }
            let rhs_end = if rhs_start < bytes.len() && bytes[rhs_start] == b'(' {
                let mut depth = 0usize;
                let mut j = rhs_start;
                while j < bytes.len() {
                    match bytes[j] {
                        b'(' => depth += 1,
                        b')' => {
                            depth -= 1;
                            if depth == 0 {
                                j += 1;
                                break;
                            }
                        }
                        _ => {}
                    }
                    j += 1;
                }
                j
            } else {
                let mut j = rhs_start;
                while j < bytes.len()
                    && (bytes[j].is_ascii_alphanumeric() || bytes[j] == b'_' || bytes[j] == b':')
                {
                    j += 1;
                }
                j
            };
            let rhs = src[rhs_start..rhs_end].trim();
            if !lhs.is_empty() && !rhs.is_empty() {
                let env = collect_local_let_u32_bindings(&src[..i], consts);
                if let (Some(base), Some(off)) = (
                    eval_const_u32_expr(lhs, &env),
                    eval_const_u32_expr(rhs, &env),
                ) {
                    if is_esp32_mmio_addr(base) && off <= 0xFFFF {
                        if let Some(folded) = base.checked_add(off) {
                            if is_esp32_mmio_addr(folded) {
                                out.push((i, folded));
                            }
                        }
                    }
                }
            }
        }
        i += 1;
    }
    out
}

/// Collects all candidate MMIO `u32` values appearing in `src`, including raw
/// hex literals, constant bindings, and folded `base + offset` expressions.
fn collect_all_mmio_u32_values(src: &str) -> Vec<u32> {
    let consts = collect_u32_consts(src);
    let mut vals: Vec<u32> = extract_hex_u32_literals(src)
        .into_iter()
        .map(|(_, v)| v)
        .collect();
    vals.extend(consts.values().copied());
    vals.extend(
        extract_folded_mmio_exprs(src, &consts)
            .into_iter()
            .map(|(_, v)| v),
    );
    vals
}

/// Flags raw ESP32 MMIO pointer casts (`0x3FFx_xxxx[u32|usize] as *mut/*const`,
/// `const` bindings, and folded `base + offset` expressions) and direct MMIO
/// helper calls whose address is absent from the permitted MMIO allowlist
/// (`M-128`).
fn check_raw_mmio_pointer_casts(
    file_label: &str,
    src: &str,
    extra_map: Option<&BTreeMap<&'static str, u32>>,
) -> Vec<String> {
    let mut errors = Vec::new();
    let auth_addrs: Vec<u32> = AUTHORITATIVE_MMIO_REGISTERS
        .iter()
        .map(|&(_, a)| a)
        .collect();
    let extra_addrs: Vec<u32> = extra_map
        .map(|m| m.values().copied().collect())
        .unwrap_or_default();
    let consts = collect_u32_consts(src);
    let bytes = src.as_bytes();

    // 1. Inspect every `as *mut` and `as *const` cast (folding suffixes, consts,
    //    local `let` bindings, and `base + offset` expressions).
    for needle in ["as *mut", "as *const"] {
        let mut cursor = 0usize;
        while let Some(rel) = src[cursor..].find(needle) {
            let pos = cursor + rel;
            cursor = pos + needle.len();
            if pos > 0 && (bytes[pos - 1].is_ascii_alphanumeric() || bytes[pos - 1] == b'_') {
                continue;
            }
            let lhs = extract_cast_lhs(src, pos);
            if lhs.is_empty() {
                continue;
            }
            let env = collect_local_let_u32_bindings(&src[..pos], &consts);
            if let Some(addr) = eval_const_u32_expr(lhs, &env) {
                if is_esp32_mmio_addr(addr)
                    && !auth_addrs.contains(&addr)
                    && !extra_addrs.contains(&addr)
                {
                    errors.push(format!(
                        "{file_label}: unverified raw MMIO pointer cast 0x{addr:08X} \
                         missing from authoritative register map"
                    ));
                }
            }
        }
    }

    // 2. Inspect direct MMIO helper calls (`write_u32`, `read_u32`,
    //    `write_volatile`, `read_volatile`) when arg1 is not already a raw
    //    pointer cast, catching non-`_REG` consts and `base + offset` args.
    for fn_name in ["write_u32", "write_volatile"] {
        for (call_pos, arg1, _) in extract_two_arg_calls(src, fn_name) {
            if arg1.contains("as *mut") || arg1.contains("as *const") {
                continue;
            }
            let env = collect_local_let_u32_bindings(&src[..call_pos], &consts);
            if let Some(addr) = eval_const_u32_expr(arg1, &env) {
                if is_esp32_mmio_addr(addr)
                    && !auth_addrs.contains(&addr)
                    && !extra_addrs.contains(&addr)
                {
                    errors.push(format!(
                        "{file_label}: unverified raw MMIO access 0x{addr:08X} \
                         missing from authoritative register map"
                    ));
                }
            }
        }
    }
    for fn_name in ["read_u32", "read_volatile"] {
        for (call_pos, arg1) in extract_one_arg_calls(src, fn_name) {
            if arg1.contains("as *mut") || arg1.contains("as *const") {
                continue;
            }
            let env = collect_local_let_u32_bindings(&src[..call_pos], &consts);
            if let Some(addr) = eval_const_u32_expr(arg1, &env) {
                if is_esp32_mmio_addr(addr)
                    && !auth_addrs.contains(&addr)
                    && !extra_addrs.contains(&addr)
                {
                    errors.push(format!(
                        "{file_label}: unverified raw MMIO access 0x{addr:08X} \
                         missing from authoritative register map"
                    ));
                }
            }
        }
    }

    errors
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
    let mut errors = verify_mmio_declarations_from_matches("main.rs", &matches);
    errors.extend(check_raw_mmio_pointer_casts("main.rs", src, None));
    errors
}

fn check_forbidden_sd_pins(file_label: &str, src: &str, errors: &mut Vec<String>) {
    for sym in FORBIDDEN_SD_PINS {
        if src.contains(sym) {
            errors.push(format!(
                "{file_label}: forbidden SD0..SD3 bit-banging symbol '{sym}' present"
            ));
        }
    }
    let mmio_vals = collect_all_mmio_u32_values(src);
    for &(sym, addr) in FORBIDDEN_GPIO_FUNC_SD_PINS
        .iter()
        .chain(FORBIDDEN_IO_MUX_SD_PINS.iter())
    {
        if mmio_vals.contains(&addr) && !src.contains(sym) {
            errors.push(format!(
                "{file_label}: forbidden raw SD0..SD3 MMIO address 0x{addr:08X} ({sym}) present"
            ));
        }
    }
}

/// Evaluates whether a bitmask expression sets `1 << pin` via constant folding
/// (`(val & (1 << pin)) != 0`), resolving `const` and prior `let` bindings as
/// well as runtime-ORed expressions (`prev | 0x0001_0010`) (`M-128`).
fn expr_sets_gpio_pin(expr: &str, pin: u8, env: &BTreeMap<String, u32>) -> bool {
    let pin_mask = 1u32 << pin;
    if let Some(val) = eval_const_u32_expr(expr, env) {
        return (val & pin_mask) != 0;
    }
    let or_parts = split_top_level_single_op(expr.trim(), b'|');
    if !or_parts.is_empty() {
        return or_parts
            .into_iter()
            .any(|part| expr_sets_gpio_pin(part, pin, env));
    }
    window_targets_gpio_pin(expr, pin)
}

/// Returns `true` if `window` contains a bitmask expression targeting `pin`
/// (tolerant of whitespace, numeric suffixes, hex, binary, and decimal forms).
fn window_targets_gpio_pin(window: &str, pin: u8) -> bool {
    let compact: String = window
        .chars()
        .filter(|c| !c.is_whitespace() && *c != '_')
        .collect();
    let compact_lower = compact.to_ascii_lowercase();
    let pin_mask = 1u32 << pin;

    // Check `1<<pin`, `1u32<<pin`, `1usize<<pin`, `1<<0x{pin:x}`, etc.
    for lhs in ["1<<", "1u32<<", "1usize<<", "1u64<<"] {
        let mut cursor = 0usize;
        while let Some(pos) = compact_lower[cursor..].find(lhs) {
            let after = &compact_lower[cursor + pos + lhs.len()..];
            let dec_pat = format!("{pin}");
            let hex_pat = format!("0x{pin:x}");
            for pat in [dec_pat.as_str(), hex_pat.as_str()] {
                if let Some(rest) = after.strip_prefix(pat) {
                    let boundary = rest.chars().next().is_none_or(|c| !c.is_ascii_hexdigit());
                    if boundary {
                        return true;
                    }
                }
            }
            cursor += pos + lhs.len();
        }
    }

    // Check any hex literal in `window` via bitmask test `(val & pin_mask) != 0`.
    for (_, val) in extract_hex_u32_literals(window) {
        if !is_esp32_mmio_addr(val) && (val & pin_mask) != 0 {
            return true;
        }
    }

    // Check explicit binary (`0b10000`) or decimal (`16`) arg.
    let bin_exact = format!("0b{pin_mask:b}");
    let mut cursor = 0usize;
    while let Some(pos) = compact_lower[cursor..].find(&bin_exact) {
        let prev_ok = pos == 0
            || !compact_lower[..cursor + pos]
                .chars()
                .next_back()
                .is_some_and(|c| c.is_ascii_alphanumeric());
        let after = &compact_lower[cursor + pos + bin_exact.len()..];
        let next_ok = after.chars().next().is_none_or(|c| !c.is_ascii_hexdigit());
        if prev_ok && next_ok {
            return true;
        }
        cursor += pos + bin_exact.len();
    }

    // Check decimal mask as a standalone second argument `,16)` or `,16u32)`.
    let dec_arg = format!(",{pin_mask}");
    if let Some(pos) = compact_lower.find(&dec_arg) {
        let after = &compact_lower[pos + dec_arg.len()..];
        if after.starts_with(')') || after.starts_with("u32)") || after.starts_with("usize)") {
            return true;
        }
    }

    false
}

fn has_gpio_pushpull_drive(src: &str, tokens: &[&str], pin: u8) -> bool {
    let code = strip_string_literals(src);
    let file_consts = collect_u32_consts(&code);

    // 1. Inspect explicit MMIO write calls (`write_volatile`, `write_u32`),
    //    resolving both target address aliases / base+offset and constant/let
    //    bitmasks via `(val & (1 << pin)) != 0` (`M-128`).
    for fn_name in ["write_volatile", "write_u32"] {
        for (call_pos, arg1, arg2) in extract_two_arg_calls(&code, fn_name) {
            let env = collect_local_let_u32_bindings(&code[..call_pos], &file_consts);
            let targets_gpio_reg = tokens.iter().any(|tok| arg1.contains(tok))
                || eval_const_u32_expr(arg1, &env)
                    .is_some_and(|addr| GPIO4_CONTENTION_RAW_ADDRS.contains(&addr));
            if targets_gpio_reg && expr_sets_gpio_pin(arg2, pin, &env) {
                return true;
            }
        }
    }

    // 2. Scan remaining token, raw address, and folded `base + offset` sites
    //    (e.g., custom write helpers or raw pointer dereference writes).
    let mut trigger_offsets = Vec::new();
    for token in tokens {
        let mut offset = 0;
        while let Some(pos) = code[offset..].find(token) {
            trigger_offsets.push((offset + pos, offset + pos + token.len()));
            offset += pos + token.len();
        }
    }
    for (pos, addr) in extract_hex_u32_literals(&code) {
        if GPIO4_CONTENTION_RAW_ADDRS.contains(&addr) {
            trigger_offsets.push((pos, pos));
        }
    }
    for (pos, addr) in extract_folded_mmio_exprs(&code, &file_consts) {
        if GPIO4_CONTENTION_RAW_ADDRS.contains(&addr) {
            trigger_offsets.push((pos, pos));
        }
    }

    for (trigger_start, window_start) in trigger_offsets {
        // Ignore `const` / `static` register definitions themselves.
        let stmt_start = code[..trigger_start]
            .rfind([';', '{', '}'])
            .map_or(0, |i| i + 1);
        let stmt_head = code[stmt_start..trigger_start].trim_start();
        if stmt_head.starts_with("const ")
            || stmt_head.starts_with("pub const ")
            || stmt_head.starts_with("static ")
            || stmt_head.starts_with("pub static ")
        {
            continue;
        }

        let rest = &code[window_start..];
        let end_bound = rest.find(';').unwrap_or(rest.len()).min(200);
        let mut window_len = rest.len().min(end_bound);
        while window_len > 0 && !rest.is_char_boundary(window_len) {
            window_len -= 1;
        }
        let window = &rest[..window_len];
        let env = collect_local_let_u32_bindings(&code[..trigger_start], &file_consts);

        if let Some(comma_idx) = window.find(',') {
            let after_comma = &window[comma_idx + 1..];
            let mask_end = {
                let mut depth = 0usize;
                let mut end = after_comma.len();
                for (idx, ch) in after_comma.char_indices() {
                    match ch {
                        '(' => depth += 1,
                        ')' => {
                            if depth == 0 {
                                end = idx;
                                break;
                            }
                            depth -= 1;
                        }
                        _ => {}
                    }
                }
                end
            };
            if expr_sets_gpio_pin(after_comma[..mask_end].trim(), pin, &env) {
                return true;
            }
        } else if let Some(eq_idx) = window.find('=') {
            let after_eq = &window[eq_idx + 1..];
            if !after_eq.starts_with('=') && expr_sets_gpio_pin(after_eq.trim(), pin, &env) {
                return true;
            }
        } else if window_targets_gpio_pin(window, pin) {
            return true;
        }
    }
    false
}

/// Checks a single udev rules file for forbidden command-execution or modem
/// reset hooks (`M-205` / `C4-047`).
fn inspect_udev_rules_file(path: &Path, errors: &mut Vec<String>) {
    let text = match std::fs::read_to_string(path) {
        Ok(t) => t,
        Err(e) => {
            errors.push(format!(
                "Failed to read udev rule file {}: {e} (fail-closed)",
                path.display()
            ));
            return;
        }
    };

    let forbidden_literals = ["TIOCMBIS", "TIOCMSET", "stty"];
    for (line_no, raw_line) in text.lines().enumerate() {
        let line = raw_line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        for tok in forbidden_literals {
            if line.contains(tok) {
                errors.push(format!(
                    "Forbidden token '{tok}' found in {}:{}",
                    path.display(),
                    line_no + 1
                ));
            }
        }
        // Parse comma-separated key-operator pairs to catch RUN=, RUN+=,
        // RUN +=, RUN{program}+=, RUN{builtin}+=, PROGRAM=, IMPORT{program}=.
        for clause in line.split(',') {
            let clause = clause.trim();
            let key_part = clause
                .split(['=', '+', ':', '!'])
                .next()
                .unwrap_or("")
                .trim();
            let key_upper = key_part.to_ascii_uppercase();
            let is_forbidden_key = key_upper == "RUN"
                || key_upper.starts_with("RUN{")
                || key_upper == "PROGRAM"
                || key_upper.starts_with("IMPORT{PROGRAM");
            if is_forbidden_key {
                errors.push(format!(
                    "Forbidden udev execution directive '{key_part}' found in {}:{}",
                    path.display(),
                    line_no + 1
                ));
            }
        }
    }
}

/// Verifies udev rules for forbidden reset hooks, failing closed on I/O errors
/// (`M-205` / `C4-047`).
#[must_use]
pub fn verify_udev_rules(repo_root: &Path) -> Vec<String> {
    let mut errors = Vec::new();

    // 1. Repository template docs/99-esp32-wrover.rules
    let docs_rule = repo_root.join("docs").join("99-esp32-wrover.rules");
    if !docs_rule.is_file() {
        errors.push(format!(
            "Missing udev rule template file: {}",
            docs_rule.display()
        ));
    } else {
        inspect_udev_rules_file(&docs_rule, &mut errors);
    }

    // 2. Host udev rules under /etc/udev/rules.d/
    let etc_dir = Path::new("/etc/udev/rules.d");
    let etc_rule = etc_dir.join("99-esp32-wrover.rules");
    if etc_rule.is_file() {
        inspect_udev_rules_file(&etc_rule, &mut errors);
    } else {
        println!(
            "[INFO] Host udev rule {} not present (skipping non-hardware host check)",
            etc_rule.display()
        );
    }

    if etc_dir.is_dir() {
        match std::fs::read_dir(etc_dir) {
            Ok(entries) => {
                for entry in entries {
                    let Ok(entry) = entry else {
                        errors.push(format!(
                            "Failed to read directory entry in {}",
                            etc_dir.display()
                        ));
                        continue;
                    };
                    let path = entry.path();
                    if path == etc_rule
                        || !path.is_file()
                        || path.extension().and_then(|s| s.to_str()) != Some("rules")
                    {
                        continue;
                    }
                    if let Ok(text) = std::fs::read_to_string(&path) {
                        let lower = text.to_ascii_lowercase();
                        let touches_esp32 = lower.contains("1a86")
                            || lower.contains("10c4")
                            || lower.contains("ttyusb-esp32");
                        if touches_esp32 {
                            inspect_udev_rules_file(&path, &mut errors);
                        }
                    }
                }
            }
            Err(e) => {
                errors.push(format!("Failed to list {}: {e}", etc_dir.display()));
            }
        }
    }

    errors
}

/// Checks whether `src` contains a `write_volatile(SPI0_PIN_REG, ...)` call
/// whose value sets bit 1 (`1 << 1`, `1u32 << 1`, or `0x2`).
fn has_spi0_pin_cs1_dis_write(src: &str) -> bool {
    for arg in extract_write_second_args(src, "write_volatile", "SPI0_PIN_REG") {
        let compact: String = arg
            .chars()
            .filter(|c| !c.is_whitespace() && *c != '_')
            .collect();
        let lower = compact.to_ascii_lowercase();
        if lower.contains("1<<1")
            || lower.contains("1u32<<1")
            || lower.contains("1usize<<1")
            || lower.contains("|0x2")
            || lower.contains("|2")
        {
            return true;
        }
    }
    false
}

/// Verifies the main firmware source code invariants across `main.rs` and all
/// sibling modules under `crates/klipper-cam-fw/src/**/*.rs`
/// (`M-127`..`M-129`).
#[must_use]
pub fn verify_main_rs(repo_root: &Path) -> Vec<String> {
    let mut errors = Vec::new();
    let fw_src_dir = repo_root.join("crates").join("klipper-cam-fw").join("src");
    let main_rs = fw_src_dir.join("main.rs");

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
    let src = strip_comments_and_tests(&raw_src);
    let code = strip_string_literals(&src);

    // 1. Zero SD0..SD3 (GPIO 7..10) bit-banging
    check_forbidden_sd_pins("main.rs", &code, &mut errors);

    // 2. arm_rtc_wdt_15s() immediately after esp_hal::init(config) in main()
    //    AND arm_rtc_wdt_15s writes non-zero config to RTC_CNTL_WDTCONFIG0_REG.
    let main_body = extract_fn_body(&code, "fn main() -> !");
    let wdt_called = if let Some(body) = main_body {
        if let Some(idx) = body.find("esp_hal::init(config);") {
            let after_init = body[idx + 22..].trim_start();
            after_init.starts_with("arm_rtc_wdt_15s();")
        } else {
            false
        }
    } else {
        false
    };
    if !wdt_called {
        errors.push(
            "main.rs: arm_rtc_wdt_15s() is not called immediately after \
             esp_hal::init(config) inside fn main() -> !"
                .to_string(),
        );
    }
    if let Some(wdt_body) = extract_fn_body(&code, "fn arm_rtc_wdt_15s()") {
        let wdt_consts = collect_u32_consts(wdt_body);
        let has_nonzero_wdt =
            extract_write_second_args(wdt_body, "write_volatile", "RTC_CNTL_WDTCONFIG0_REG")
                .into_iter()
                .any(|arg| eval_const_u32_expr(arg, &wdt_consts).is_some_and(|v| v != 0));
        if !has_nonzero_wdt {
            errors.push(
                "main.rs: arm_rtc_wdt_15s() does not write a provably non-zero const \
                 config to RTC_CNTL_WDTCONFIG0_REG"
                    .to_string(),
            );
        }
    } else {
        errors.push("main.rs: missing `fn arm_rtc_wdt_15s()` definition".to_string());
    }

    // 3. Hardware SPI1 0xF5 and 0x35 commands scoped inside
    //    `ensure_psram_qpi_mode_spi1()`, plus `ensure_psram_qpi_mode_spi1()`
    //    and `PsramCacheSpeed::PsramCacheF40mS40m` scoped inside `main()`
    //    (`M-129`).
    if !code.contains("fn psram_spi1_cmd(qpi_mode: bool, cmd_byte: u8)") {
        errors.push(
            "main.rs: missing hardware SPI1 helper \
             `fn psram_spi1_cmd(qpi_mode: bool, cmd_byte: u8)`"
                .to_string(),
        );
    }
    if let Some(qpi_body) = extract_fn_body(&code, "fn ensure_psram_qpi_mode_spi1()") {
        if !qpi_body.contains("psram_spi1_cmd(true, 0xF5)")
            || !qpi_body.contains("psram_spi1_cmd(false, 0x35)")
        {
            errors.push(
                "main.rs: missing `psram_spi1_cmd(true, 0xF5)` or \
                 `psram_spi1_cmd(false, 0x35)` calls inside `ensure_psram_qpi_mode_spi1()`"
                    .to_string(),
            );
        }
    } else {
        errors.push("main.rs: missing `fn ensure_psram_qpi_mode_spi1()` definition".to_string());
    }
    if !main_body.is_some_and(|b| b.contains("ensure_psram_qpi_mode_spi1()")) {
        errors.push("main.rs: missing `ensure_psram_qpi_mode_spi1()` call in main()".to_string());
    }
    if !main_body.is_some_and(|b| b.contains("PsramCacheSpeed::PsramCacheF40mS40m")) {
        errors.push(
            "main.rs: must use `PsramCacheSpeed::PsramCacheF40mS40m` in main() \
             matching ESP-IDF v5.5.4"
                .to_string(),
        );
    }

    // 4. Zero boot-time FlashStorage calls before earliest Wi-Fi init in
    //    main().
    if let Some(body) = main_body {
        let wifi_init_pos = [
            body.find("wifi_driver.init("),
            body.find("XtensaPhySession::new("),
            body.find("WifiController::new("),
        ]
        .into_iter()
        .flatten()
        .min();

        if let Some(wifi_idx) = wifi_init_pos {
            let pre_wifi = &body[..wifi_idx];
            let forbidden = [
                ".read_nor(",
                ".erase(",
                ".write_nor(",
                "FlashStorage::new",
                "esp_rom_spiflash_",
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
            errors.push("main.rs: missing Wi-Fi initialization call in main()".to_string());
        }
    } else {
        errors.push("main.rs: missing `fn main() -> !`".to_string());
    }

    // 5. Authoritative ESP-IDF v5.5.4 MMIO register check (Invariant 72)
    errors.extend(verify_mmio_registers(&code));

    // 6. Zero GPIO 4 push-pull contention on ESP32-WROVER-DEV (Invariant 71)
    if has_gpio_pushpull_drive(&code, &GPIO4_CONTENTION_TOKENS, 4) {
        errors.push(
            "main.rs: forbidden push-pull output drive on GPIO 4 (`1 << 4`), \
             which contentions with OV2640 CAM_PIN_D0 (Y2) on ESP32-WROVER-DEV"
                .to_string(),
        );
    }

    // 7. Active OTA slot detection via DPORT_PRO_FLASH_MMU_TABLE (0x3FF1_0000)
    if !main_body.is_some_and(|b| b.contains("detect_booted_ota_manager_from_mmu()")) {
        errors.push(
            "main.rs: must detect active booted OTA slot from DPORT_PRO_FLASH_MMU_TABLE \
             via `detect_booted_ota_manager_from_mmu()` in main()"
                .to_string(),
        );
    }

    // 8. H3-CFG-003: hold_psram_cs_high_safe() called in main()
    if !main_body.is_some_and(|b| b.contains("hold_psram_cs_high_safe()")) {
        errors.push("main.rs: missing `hold_psram_cs_high_safe()` call in main()".to_string());
    }

    // 9. H3-CFG-003: SPI0_PIN_REG CS1_DIS gating bound to actual write statement
    if !has_spi0_pin_cs1_dis_write(&code) {
        errors.push(
            "main.rs: missing `SPI0_PIN_REG` CS1_DIS bit (1 << 1) gating \
             write before SPI1 PSRAM transactions"
                .to_string(),
        );
    }

    // 10. H3-CFG-003: 50 ms PSRAM power-up delay inside main()
    let has_delay = main_body
        .is_some_and(|b| b.contains("delay_millis(50)") || b.contains("delay_micros(50_000)"));
    if !has_delay {
        errors.push(
            "main.rs: missing 50 ms PSRAM power-up delay \
             (delay_millis(50) / delay_micros(50_000)) in main()"
                .to_string(),
        );
    }

    // 11. LOGIC-186: capture_live_hd_jpeg_with_diag present in code
    if !code.contains("capture_live_hd_jpeg_with_diag") {
        errors.push("main.rs: missing `capture_live_hd_jpeg_with_diag`".to_string());
    }

    // 12. Scan all sibling klipper-cam-fw/src/**/*.rs modules (M-127).
    match collect_rs_files(&fw_src_dir, "klipper-cam-fw") {
        Ok(rs_files) => {
            for path in rs_files {
                if path == main_rs {
                    continue;
                }
                let rel = path
                    .strip_prefix(&fw_src_dir)
                    .unwrap_or(&path)
                    .display()
                    .to_string();
                let file_label = format!("klipper-cam-fw/{rel}");
                let sibling_raw = match std::fs::read_to_string(&path) {
                    Ok(s) => s,
                    Err(e) => {
                        errors.push(format!("Failed to read {}: {e}", path.display()));
                        continue;
                    }
                };
                let sibling_code = strip_string_literals(&strip_comments_and_tests(&sibling_raw));
                check_forbidden_sd_pins(&file_label, &sibling_code, &mut errors);
                if has_gpio_pushpull_drive(&sibling_code, &GPIO4_CONTENTION_TOKENS, 4) {
                    errors.push(format!(
                        "{file_label}: forbidden push-pull output drive on GPIO 4 (`1 << 4`), \
                         which contentions with OV2640 CAM_PIN_D0 (Y2) on ESP32-WROVER-DEV"
                    ));
                }
                let decls = parse_mmio_declarations(&sibling_code);
                errors.extend(verify_mmio_declarations_from_matches(&file_label, &decls));
                errors.extend(check_raw_mmio_pointer_casts(
                    &file_label,
                    &sibling_code,
                    None,
                ));
            }
        }
        Err(err) => errors.push(err),
    }

    errors
}

fn collect_rs_files_recursive(
    dir: &Path,
    label: &str,
    out: &mut Vec<PathBuf>,
) -> Result<(), String> {
    let entries = std::fs::read_dir(dir).map_err(|e| {
        format!(
            "Failed to read {label} source directory {}: {e}",
            dir.display()
        )
    })?;

    for entry in entries {
        let entry = entry.map_err(|e| {
            format!(
                "Failed to read directory entry in {label} directory {}: {e}",
                dir.display()
            )
        })?;
        let path = entry.path();
        if path.is_dir() {
            collect_rs_files_recursive(&path, label, out)?;
        } else if path.is_file() && path.extension().and_then(|s| s.to_str()) == Some("rs") {
            out.push(path);
        }
    }
    Ok(())
}

fn collect_rs_files(src_dir: &Path, label: &str) -> Result<Vec<PathBuf>, String> {
    if !src_dir.is_dir() {
        return Err(format!(
            "Missing {label} source directory: {}",
            src_dir.display()
        ));
    }

    let mut rs_files = Vec::new();
    collect_rs_files_recursive(src_dir, label, &mut rs_files)?;
    rs_files.sort();

    if rs_files.is_empty() {
        return Err(format!(
            "No Rust source files found in {}",
            src_dir.display()
        ));
    }

    Ok(rs_files)
}

/// Verifies all Rust sources in `crates/esp32-cam-bootloader/src/**/*.rs`.
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
        let rel_label = path
            .strip_prefix(&bootloader_src)
            .unwrap_or(&path)
            .display()
            .to_string();
        let file_label = format!("esp32-cam-bootloader/{rel_label}");

        let raw_src = match std::fs::read_to_string(&path) {
            Ok(s) => s,
            Err(e) => {
                errors.push(format!("Failed to read {}: {e}", path.display()));
                continue;
            }
        };
        let src = strip_string_literals(&strip_comments_and_tests(&raw_src));

        let decls = parse_mmio_declarations(&src);
        total_mmio_decls += decls.len();
        errors.extend(verify_mmio_declarations_from_matches(&file_label, &decls));
        errors.extend(check_raw_mmio_pointer_casts(&file_label, &src, None));

        let hex_vals = collect_all_mmio_u32_values(&src);

        for &(sym, addr) in &FORBIDDEN_GPIO_FUNC_SD_PINS {
            if src.contains(sym) {
                errors.push(format!(
                    "{file_label}: forbidden SD0..SD3 bit-banging symbol '{sym}' present"
                ));
            } else if hex_vals.contains(&addr) {
                errors.push(format!(
                    "{file_label}: forbidden raw SD0..SD3 address 0x{addr:08X} ({sym}) present"
                ));
            }
        }

        if file_name == "clock_spi_psram.rs" {
            // Exclude read-only `is_allowed_clock_mmio_reg` allowlist predicate
            // so allowlist references to `crate::gpio16::GPIO_ENABLE_W1TS_REG`
            // do not trigger a false positive while any real write/declaration
            // is still rejected.
            let stripped_allowlist =
                if let Some(body) = extract_fn_body(&src, "fn is_allowed_clock_mmio_reg(") {
                    src.replace(body, "")
                } else {
                    src.clone()
                };
            let enable_hex_vals = collect_all_mmio_u32_values(&stripped_allowlist);
            if stripped_allowlist.contains("GPIO_ENABLE")
                || GPIO_ENABLE_RAW_ADDRS
                    .iter()
                    .any(|a| enable_hex_vals.contains(a))
            {
                errors.push(format!(
                    "{file_label}: forbidden GPIO_ENABLE token or raw address \
                     present to prevent push-pull GPIO bit-banging"
                ));
            }
        } else {
            for &(sym, addr) in &FORBIDDEN_IO_MUX_SD_PINS {
                if src.contains(sym) {
                    errors.push(format!(
                        "{file_label}: forbidden SD0..SD3 bit-banging symbol '{sym}' present"
                    ));
                } else if hex_vals.contains(&addr) {
                    errors.push(format!(
                        "{file_label}: forbidden raw SD0..SD3 address 0x{addr:08X} ({sym}) present"
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
            let consts = collect_u32_consts(&src);
            if consts.get("IO_MUX_GPIO16_REG") != Some(&0x3FF4_904C) {
                errors.push(format!(
                    "{file_label}: missing IO_MUX_GPIO16_REG (0x3FF4_904C) declaration"
                ));
            }
            let writes = extract_write_second_args(&src, "write_u32", "IO_MUX_GPIO16_REG");
            let writes_2b00 = writes
                .iter()
                .any(|arg| eval_const_u32_expr(arg, &consts) == Some(0x2B00));
            if !writes_2b00 {
                errors.push(format!(
                    "{file_label}: missing MMIO write_u32(IO_MUX_GPIO16_REG, 0x2B00) \
                     pull-up configuration"
                ));
            }
        } else if file_name == "wdt.rs" {
            saw_wdt_rs = true;
            let consts = collect_u32_consts(&src);
            if consts.get("RTC_CNTL_WDTCONFIG0_REG") != Some(&0x3FF4_808C) {
                errors.push(format!(
                    "{file_label}: missing RTC_CNTL_WDTCONFIG0_REG (0x3FF4_808C) declaration"
                ));
            }
            let writes = extract_write_second_args(&src, "write_u32", "RTC_CNTL_WDTCONFIG0_REG");
            let arms_nonzero = writes
                .iter()
                .any(|arg| eval_const_u32_expr(arg, &consts).is_some_and(|val| val != 0));
            if !arms_nonzero {
                errors.push(format!(
                    "{file_label}: missing non-zero RTC_CNTL_WDTCONFIG0_REG arming write"
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

/// Verifies all Rust sources in `crates/esp32-cam-rf-clock/src/**/*.rs`.
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
    let coex_map = rea_coex_map();

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
        let rel_label = path
            .strip_prefix(&rf_clock_src)
            .unwrap_or(&path)
            .display()
            .to_string();
        let file_label = format!("esp32-cam-rf-clock/{rel_label}");

        let raw_src = match std::fs::read_to_string(&path) {
            Ok(s) => s,
            Err(e) => {
                errors.push(format!("Failed to read {}: {e}", path.display()));
                continue;
            }
        };
        let src = strip_string_literals(&strip_comments_and_tests(&raw_src));

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
            errors.extend(check_raw_mmio_pointer_casts(
                &file_label,
                &src,
                Some(&coex_map),
            ));
        } else {
            errors.extend(verify_mmio_declarations_from_matches(&file_label, &decls));
            errors.extend(check_raw_mmio_pointer_casts(&file_label, &src, None));
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

/// Verifies all Rust sources in `crates/esp32-cam-wifi-mac/src/**/*.rs`.
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
        let rel_label = path
            .strip_prefix(&wifi_mac_src)
            .unwrap_or(&path)
            .display()
            .to_string();
        let file_label = format!("esp32-cam-wifi-mac/{rel_label}");

        let raw_src = match std::fs::read_to_string(&path) {
            Ok(s) => s,
            Err(e) => {
                errors.push(format!("Failed to read {}: {e}", path.display()));
                continue;
            }
        };
        let src = strip_string_literals(&strip_comments_and_tests(&raw_src));

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
        errors.extend(check_raw_mmio_pointer_casts(
            &file_label,
            &src,
            Some(&wifi_map),
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
    let content = strip_string_literals(&strip_comments_and_tests(&raw_content));
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
        "[PASS] Invariant 70/71/72 (klipper-cam-fw): zero SD0..SD3 bit-banging, \
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

    fn unique_temp_root(prefix: &str) -> PathBuf {
        let unique = format!(
            "esp32_{prefix}_{}_{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map_or(0, |d| d.as_nanos())
        );
        std::env::temp_dir().join(unique)
    }

    #[test]
    fn test_authoritative_register_count() {
        assert_eq!(
            AUTHORITATIVE_MMIO_REGISTERS.len(),
            95,
            "expected 95 authoritative MMIO registers"
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
        let temp_root = unique_temp_root("bl_inv_test");
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
    fn test_recursive_collection_and_hardened_bypasses() {
        let temp_root = unique_temp_root("bl_nested_bypass");
        let _guard = TempDirGuard(temp_root.clone());

        let nested = temp_root
            .join("crates")
            .join("esp32-cam-bootloader")
            .join("src")
            .join("sub");
        std::fs::create_dir_all(&nested).expect("mkdir sub");

        let src_dir = nested.parent().expect("parent");
        // Vacuous gpio16.rs (writes 0 instead of 0x2B00) and disarming wdt.rs
        // (writes 0).
        std::fs::write(
            src_dir.join("gpio16.rs"),
            "pub const IO_MUX_GPIO16_REG: u32 = 0x3FF4_904C;\n\
             pub const UNUSED: u32 = 0x2B00;\n\
             fn init() { write_u32(IO_MUX_GPIO16_REG, 0); }\n",
        )
        .expect("write gpio16.rs");
        std::fs::write(
            src_dir.join("wdt.rs"),
            "pub const RTC_CNTL_WDTCONFIG0_REG: u32 = 0x3FF4_808C;\n\
             fn disarm() { write_u32(RTC_CNTL_WDTCONFIG0_REG, 0); }\n",
        )
        .expect("write wdt.rs");
        // Nested module with raw hex bypasses for GPIO4 (0x10) and SD0 (0x3FF49064).
        std::fs::write(
            nested.join("hidden.rs"),
            "fn stealth() {\n\
               write_u32(0x3FF44024, 0x10);\n\
               let _sd0 = 0x3FF4_9064u32;\n\
               let _raw = 0x3FF9_9999 as *mut u32;\n\
             }\n\
             #[cfg(test)]\n\
             mod tests {\n\
               const FAKE_TEST_ONLY_REG: u32 = 0x3FF9_8888;\n\
             }\n",
        )
        .expect("write hidden.rs");

        let errors = verify_bootloader_sources(&temp_root);
        assert!(errors
            .iter()
            .any(|e| e.contains("gpio16.rs") && e.contains("0x2B00")));
        assert!(errors
            .iter()
            .any(|e| e.contains("wdt.rs") && e.contains("non-zero")));
        assert!(errors
            .iter()
            .any(|e| e.contains("hidden.rs") && e.contains("GPIO 4")));
        assert!(errors
            .iter()
            .any(|e| e.contains("hidden.rs") && e.contains("0x3FF49064")));
        assert!(errors
            .iter()
            .any(|e| e.contains("hidden.rs") && e.contains("0x3FF99999")));
        // #[cfg(test)] block must be stripped cleanly.
        assert!(!errors.iter().any(|e| e.contains("FAKE_TEST_ONLY_REG")));
    }

    #[test]
    fn test_udev_rules_rejects_run_program_import_variants() {
        let temp_root = unique_temp_root("udev_hardening");
        let _guard = TempDirGuard(temp_root.clone());
        let docs = temp_root.join("docs");
        std::fs::create_dir_all(&docs).expect("mkdir docs");
        std::fs::write(
            docs.join("99-esp32-wrover.rules"),
            "SUBSYSTEM==\"tty\", RUN{program}+=\"/bin/true\", PROGRAM=\"/bin/sh\"\n",
        )
        .expect("write rules");

        let errors = verify_udev_rules(&temp_root);
        assert!(errors.iter().any(|e| e.contains("RUN{program}")));
        assert!(errors.iter().any(|e| e.contains("PROGRAM")));
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
        let temp_root = unique_temp_root("rf_clock_inv_test");
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

    #[test]
    fn test_m128_mmio_suffix_const_and_base_offset_casts_rejected() {
        let cases = [
            "fn a() { let _p = 0x3FF9_9999u32 as *mut u32; }",
            "fn b() { let _p = 0x3FF9_9999usize as *mut u32; }",
            "fn c() { let _p = 0x3FF9_9999_u32 as *const u32; }",
            "const BAD: u32 = 0x3FF9_9999; fn d() { let _p = BAD as *mut u32; }",
            "const BASE: u32 = 0x3FF9_9900; fn e() { let _p = (BASE + 0x99) as *mut u32; }",
            "fn f() { let _p = (0x3FF9_9900u32 + 0x99) as *mut u32; }",
            "const BAD: u32 = 0x3FF9_9999; fn g() { write_u32(BAD, 1); }",
        ];
        for src in cases {
            let errs = check_raw_mmio_pointer_casts("test.rs", src, None);
            assert!(
                errs.iter().any(|e| e.contains("0x3FF99999")),
                "expected 0x3FF99999 rejection for `{src}`, got: {errs:?}"
            );
        }

        // Base+offset constructing forbidden SD0 (IO_MUX_GPIO7_REG = 0x3FF4_9064).
        let mut sd_errs = Vec::new();
        check_forbidden_sd_pins(
            "test.rs",
            "const IO_BASE: u32 = 0x3FF4_9000; fn s() { let _p = (IO_BASE + 0x64) as *mut u32; }",
            &mut sd_errs,
        );
        assert!(
            sd_errs.iter().any(|e| e.contains("0x3FF49064")),
            "expected folded SD0 0x3FF49064 rejection, got: {sd_errs:?}"
        );

        let mut sd_lit_errs = Vec::new();
        check_forbidden_sd_pins(
            "test.rs",
            "fn s() { let _p = (0x3FF4_9000 + 0x64) as *mut u32; }",
            &mut sd_lit_errs,
        );
        assert!(
            sd_lit_errs.iter().any(|e| e.contains("0x3FF49064")),
            "expected literal base+offset SD0 rejection, got: {sd_lit_errs:?}"
        );

        // Legitimate authoritative register with integer suffix must pass.
        let clean = check_raw_mmio_pointer_casts(
            "test.rs",
            "fn ok() { let _p = 0x3FF0_00BCu32 as *const u32; }",
            None,
        );
        assert!(
            clean.is_empty(),
            "unexpected errors for valid cast: {clean:?}"
        );
    }

    #[test]
    fn test_m128_gpio4_registers_combined_masks_and_let_bindings_rejected() {
        let forbidden_cases = [
            "fn f() { write_volatile(GPIO_ENABLE_REG, 1 << 4); }",
            "fn f() { write_volatile(GPIO_OUT_W1TS_REG, 1 << 4); }",
            "fn f() { write_u32(0x3FF4_4020, 0x0001_0010); }",
            "fn f() { write_u32(0x3FF4_4008, 0x0001_0010); }",
            "fn f() { let m = 1 << 4; write_volatile(GPIO_ENABLE_W1TS_REG, m); }",
            "fn f() { let m: u32 = 0x0001_0010; write_volatile(GPIO_ENABLE_REG, m); }",
            "fn f() { let p = read_volatile(GPIO_ENABLE_REG); \
             write_volatile(GPIO_ENABLE_REG, p | 0x0001_0010); }",
        ];
        for src in forbidden_cases {
            assert!(
                has_gpio_pushpull_drive(src, &GPIO4_CONTENTION_TOKENS, 4),
                "expected GPIO 4 contention rejection for `{src}`"
            );
        }

        // Valid GPIO 16/17 PSRAM CS/CLK operations must not trip GPIO 4.
        let allowed = "
            const GPIO_OUT_W1TS_REG: *mut u32 = 0x3FF4_4008 as *mut u32;
            const GPIO_ENABLE_W1TS_REG: *mut u32 = 0x3FF4_4024 as *mut u32;
            fn ok() {
                let cs_mask = (1u32 << 16) | (1u32 << 17);
                write_volatile(GPIO_OUT_W1TS_REG, 1 << 16);
                write_volatile(GPIO_ENABLE_W1TS_REG, cs_mask);
            }
        ";
        assert!(
            !has_gpio_pushpull_drive(allowed, &GPIO4_CONTENTION_TOKENS, 4),
            "GPIO 16/17 operations must not trigger GPIO 4 contention"
        );
    }

    #[test]
    fn test_m129_scoped_psram_and_fail_closed_wdt() {
        let temp_root = unique_temp_root("m129_bypass_test");
        let _guard = TempDirGuard(temp_root.clone());
        let fw_src = temp_root.join("crates").join("klipper-cam-fw").join("src");
        std::fs::create_dir_all(&fw_src).expect("mkdir fw src");

        // Bypass attempt: non-const local zero in arm_rtc_wdt_15s(), dead helper
        // holding psram_spi1_cmd / PsramCacheF40mS40m, and ensure_psram_qpi_mode_spi1
        // defined without being called in main().
        let bypassed_main = r#"
            const RTC_CNTL_WDTCONFIG0_REG: *mut u32 = 0x3FF4_808C as *mut u32;
            const SPI0_PIN_REG: *mut u32 = 0x3FF4_3034 as *mut u32;
            fn arm_rtc_wdt_15s() {
                let v = 0u32;
                core::ptr::write_volatile(RTC_CNTL_WDTCONFIG0_REG, v);
            }
            fn psram_spi1_cmd(qpi_mode: bool, cmd_byte: u8) -> bool {
                let _ = (qpi_mode, cmd_byte);
                core::ptr::write_volatile(SPI0_PIN_REG, 1u32 << 1);
                true
            }
            fn ensure_psram_qpi_mode_spi1() -> bool {
                true
            }
            fn dead_spoof_helper() {
                let _ = psram_spi1_cmd(true, 0xF5);
                let _ = psram_spi1_cmd(false, 0x35);
                let _ = PsramCacheSpeed::PsramCacheF40mS40m;
            }
            fn capture_live_hd_jpeg_with_diag() {}
            fn main() -> ! {
                esp_hal::init(config);
                arm_rtc_wdt_15s();
                hold_psram_cs_high_safe();
                delay_millis(50);
                let _ = detect_booted_ota_manager_from_mmu();
                wifi_driver.init();
                loop {}
            }
        "#;
        std::fs::write(fw_src.join("main.rs"), bypassed_main).expect("write bypassed main.rs");

        let errs = verify_main_rs(&temp_root);
        assert!(
            errs.iter().any(|e| e.contains("RTC_CNTL_WDTCONFIG0_REG")),
            "expected fail-closed WDT error, got: {errs:?}"
        );
        assert!(
            errs.iter()
                .any(|e| e.contains("inside `ensure_psram_qpi_mode_spi1()`")),
            "expected scoped psram_spi1_cmd error, got: {errs:?}"
        );
        assert!(
            errs.iter()
                .any(|e| e.contains("`ensure_psram_qpi_mode_spi1()` call in main()")),
            "expected scoped ensure_psram_qpi_mode_spi1() call in main() error, got: {errs:?}"
        );
        assert!(
            errs.iter()
                .any(|e| e.contains("PsramCacheSpeed::PsramCacheF40mS40m")),
            "expected scoped PsramCacheF40mS40m in main() error, got: {errs:?}"
        );
    }
}
