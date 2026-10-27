#!/usr/bin/env python3
"""Level 3 Automated Verification Gate for ESP32-WROVER Standalone Cold-Boot Invariants.

Enforces Invariant 69 (PM-2026-1025-027), Invariant 70 (PM-2026-1025-028 / WO-RUST-FIX-CPP-PARITY-01),
Invariant 71 (PM-2026-1026-001), and Invariant 72 (PM-2026-1026-002 / INFRA-001):
1. Host udev rule (`/etc/udev/rules.d/99-esp32-wrover.rules`) and repo template
   (`docs/99-esp32-wrover.rules`) contain zero `RUN+=`, `TIOCMBIS`, `TIOCMSET`, or `stty`
   reset workarounds.
2. `crates/klipper-cam-fw/src/main.rs`:
   - Contains ZERO `IO_MUX_GPIO7_REG`..`IO_MUX_GPIO10_REG` or `GPIO_FUNC7_OUT_SEL_CFG_REG`..`GPIO_FUNC10_OUT_SEL_CFG_REG`
     (`SD0..SD3`) bit-banging.
   - Calls `arm_rtc_wdt_15s()` immediately after `esp_hal::init(config)`.
   - Uses hardware `SPI1` `0xF5` (`psram_spi1_cmd(true, 0xF5)`) and `0x35` (`psram_spi1_cmd(false, 0x35)`,
     1-bit SPI mode) in `ensure_psram_qpi_mode_spi1()` with `PsramCacheSpeed::PsramCacheF40mS40m`.
   - Performs ZERO boot-time `FlashStorage` (`read_nor`/`erase`/`write_nor` or `OtaDataManager::load_from_flash`)
     operations in `main()` prior to `WifiController::new()`.
   - Matches every `const *_REG` / `*_MMU_TABLE` raw MMIO address against `AUTHORITATIVE_MMIO_REGISTERS`
     (`ESP-IDF v5.5.4` `soc/*_reg.h`, Invariant 72 / `INFRA-001`).
   - Contains ZERO push-pull output writes on `GPIO 4` (`1 << 4`, `CAM_PIN_D0` / `OV2640` `Y2`, Invariant 71 / `LOGIC-002`).
   - Detects active booted OTA slot from `DPORT_PRO_FLASH_MMU_TABLE` (`0x3FF1_0000`) via
     `detect_booted_ota_manager_from_mmu()` (`LOGIC-003` / `FW-004`).
3. `firmware/bootloader_v554_gpio16.bin` exists and contains the `0x3FF4_904C`
   (`\\x4c\\x90\\xf4\\x3f`) and `0x2B00` (`\\x00\\x2b\\x00\\x00` or Xtensa `movi.n+slli`
   `\\x2c\\xb9\\x80\\x99\\x11` / `\\x0c\\xb9\\x80\\x99\\x11` / `\\x80\\x99\\x11`) `bootloader_before_init`
   `GPIO 16` pull-up initialization.
"""

import re
import sys
from pathlib import Path

REPO_ROOT = Path("/home/console-admin/Github/esp32-camera")
ETC_UDEV_RULE = Path("/etc/udev/rules.d/99-esp32-wrover.rules")
DOCS_UDEV_RULE = REPO_ROOT / "docs" / "99-esp32-wrover.rules"
MAIN_RS = REPO_ROOT / "crates" / "klipper-cam-fw" / "src" / "main.rs"
BOOTLOADER_BIN = REPO_ROOT / "firmware" / "bootloader_v554_gpio16.bin"

# Authoritative ESP-IDF v5.5.4 `soc/*_reg.h` MMIO Register Address Map (`Invariant 72` / `INFRA-001`)
AUTHORITATIVE_MMIO_REGISTERS: dict[str, int] = {
    "DPORT_HOST_INF_SEL_REG": 0x3FF0_00BC,
    "DPORT_PRO_FLASH_MMU_TABLE": 0x3FF1_0000,
    "SPI1_CMD_REG": 0x3FF4_2000,
    "SPI1_ADDR_REG": 0x3FF4_2004,
    "SPI1_CTRL_REG": 0x3FF4_2008,
    "SPI1_USER_REG": 0x3FF4_201C,
    "SPI1_USER1_REG": 0x3FF4_2020,
    "SPI1_USER2_REG": 0x3FF4_2024,
    "SPI1_MOSI_DLEN_REG": 0x3FF4_2028,
    "SPI1_PIN_REG": 0x3FF4_2034,
    "SPI1_W0_REG": 0x3FF4_2080,
    "SPI1_EXT2_REG": 0x3FF4_20F8,
    "SPI0_PIN_REG": 0x3FF4_3034,
    "SPI0_EXT2_REG": 0x3FF4_30F8,
    "GPIO_OUT_W1TS_REG": 0x3FF4_4008,
    "GPIO_OUT_W1TC_REG": 0x3FF4_400C,
    "GPIO_ENABLE_W1TS_REG": 0x3FF4_4024,
    "GPIO_IN_REG": 0x3FF4_403C,
    "GPIO_FUNC16_OUT_SEL_CFG_REG": 0x3FF4_4570,
    "GPIO_FUNC17_OUT_SEL_CFG_REG": 0x3FF4_4574,
    "RTC_CNTL_RESET_STATE_REG": 0x3FF4_8034,
    "RTC_CNTL_DIG_PWC_REG": 0x3FF4_8084,
    "RTC_CNTL_DIG_ISO_REG": 0x3FF4_8088,
    "RTC_CNTL_WDTCONFIG0_REG": 0x3FF4_808C,
    "RTC_CNTL_WDTCONFIG1_REG": 0x3FF4_8090,
    "RTC_CNTL_WDTFEED_REG": 0x3FF4_80A0,
    "RTC_CNTL_WDTWPROTECT_REG": 0x3FF4_80A4,
    "RTC_CNTL_STORE6_REG": 0x3FF4_80B8,
    "RTC_CNTL_STORE7_REG": 0x3FF4_80BC,
    "RTC_CNTL_HOLD_FORCE_REG": 0x3FF4_80C8,
    "IO_MUX_GPIO16_REG": 0x3FF4_904C,
    "IO_MUX_GPIO17_REG": 0x3FF4_9050,
    "I2S_INT_RAW_REG": 0x3FF4_F00C,
    "I2S_INT_CLR_REG": 0x3FF4_F018,
    "RNG_DATA_REG": 0x3FF7_5144,
}


def verify_mmio_registers(src: str) -> list[str]:
    errors: list[str] = []
    reg_pattern = re.compile(
        r"const\s+([A-Z0-9_]+(?:_REG|_MMU_TABLE))\s*:\s*\*(?:const|mut)\s+u32\s*=\s*(0x[0-9A-Fa-f_]+)\s+as\s+\*(?:const|mut)\s+u32"
    )
    matches = reg_pattern.findall(src)
    if not matches:
        errors.append("main.rs: failed to parse any MMIO `const *_REG` declarations")
        return errors

    for reg_name, hex_literal in matches:
        actual_addr = int(hex_literal.replace("_", ""), 16)
        expected_addr = AUTHORITATIVE_MMIO_REGISTERS.get(reg_name)
        if expected_addr is None:
            errors.append(
                f"main.rs: unverified MMIO register `{reg_name}` ({hex_literal}) missing from AUTHORITATIVE_MMIO_REGISTERS"
            )
        elif actual_addr != expected_addr:
            errors.append(
                f"main.rs: MMIO address mismatch for `{reg_name}`: found 0x{actual_addr:08X}, expected 0x{expected_addr:08X} per ESP-IDF v5.5.4 soc/*_reg.h"
            )
    return errors


def verify_udev_rules() -> list[str]:
    errors: list[str] = []
    forbidden_tokens = ("RUN+=", "TIOCMBIS", "TIOCMSET", "stty")
    for path in (ETC_UDEV_RULE, DOCS_UDEV_RULE):
        if not path.is_file():
            errors.append(f"Missing udev rule file: {path}")
            continue
        text = path.read_text(encoding="utf-8")
        for tok in forbidden_tokens:
            if tok in text:
                errors.append(f"Forbidden token '{tok}' found in {path}")
    return errors


def verify_main_rs() -> list[str]:
    errors: list[str] = []
    if not MAIN_RS.is_file():
        return [f"Missing firmware entrypoint: {MAIN_RS}"]
    src = MAIN_RS.read_text(encoding="utf-8")

    # 1. Zero SD0..SD3 (GPIO 7..10) bit-banging
    forbidden_sd_pins = (
        "IO_MUX_GPIO7_REG",
        "IO_MUX_GPIO8_REG",
        "IO_MUX_GPIO9_REG",
        "IO_MUX_GPIO10_REG",
        "GPIO_FUNC7_OUT_SEL_CFG_REG",
        "GPIO_FUNC8_OUT_SEL_CFG_REG",
        "GPIO_FUNC9_OUT_SEL_CFG_REG",
        "GPIO_FUNC10_OUT_SEL_CFG_REG",
    )
    for sym in forbidden_sd_pins:
        if sym in src:
            errors.append(f"main.rs: forbidden SD0..SD3 bit-banging symbol '{sym}' present")

    # 2. arm_rtc_wdt_15s() immediately after esp_hal::init(config)
    init_pattern = re.compile(
        r"let\s+peripherals\s*=\s*esp_hal::init\(config\);\s*arm_rtc_wdt_15s\(\);",
        re.MULTILINE,
    )
    if not init_pattern.search(src):
        errors.append("main.rs: arm_rtc_wdt_15s() is not called immediately after esp_hal::init(config)")

    # 3. Hardware SPI1 0xF5 (qpi_mode = true) and 0x35 (qpi_mode = false) + PsramCacheF40mS40m
    if "fn psram_spi1_cmd(qpi_mode: bool, cmd_byte: u8)" not in src:
        errors.append("main.rs: missing hardware SPI1 helper `fn psram_spi1_cmd(qpi_mode: bool, cmd_byte: u8)`")
    if "psram_spi1_cmd(true, 0xF5)" not in src or "psram_spi1_cmd(false, 0x35)" not in src:
        errors.append("main.rs: missing `psram_spi1_cmd(true, 0xF5)` or `psram_spi1_cmd(false, 0x35)` calls")
    if "ensure_psram_qpi_mode_spi1()" not in src:
        errors.append("main.rs: missing `ensure_psram_qpi_mode_spi1()`")
    if "PsramCacheSpeed::PsramCacheF40mS40m" not in src:
        errors.append("main.rs: must use `PsramCacheSpeed::PsramCacheF40mS40m` matching ESP-IDF v5.5.4")

    # 4. Zero boot-time FlashStorage (read_nor/erase/write_nor) calls in main() before WifiController::new
    main_idx = src.find("fn main() -> !")
    if main_idx == -1:
        errors.append("main.rs: missing `fn main() -> !`")
    else:
        main_src = src[main_idx:]
        wifi_idx = main_src.find("WifiController::new(")
        if wifi_idx == -1:
            errors.append("main.rs: missing `WifiController::new(` in main()")
        else:
            pre_wifi_src = main_src[:wifi_idx]
            for forbidden_flash_op in (
                ".read_nor(",
                ".erase(",
                ".write_nor(",
                "OtaDataManager::load_from_flash",
                "cancel_rollback_in_flash",
            ):
                if forbidden_flash_op in pre_wifi_src:
                    errors.append(
                        f"main.rs: forbidden boot-time flash operation '{forbidden_flash_op}' before WifiController::new()"
                    )

    # 5. Authoritative ESP-IDF v5.5.4 MMIO register dictionary check (Invariant 72 / INFRA-001)
    errors.extend(verify_mmio_registers(src))

    # 6. Zero GPIO 4 push-pull LOW output contention on ESP32-WROVER-DEV (`CAM_PIN_D0` / `OV2640` `Y2`, Invariant 71 / LOGIC-002)
    if re.search(r"GPIO_(?:OUT_W1TC|ENABLE_W1TS)_REG\s*,\s*1(?:u32)?\s*<<\s*4\b", src):
        errors.append(
            "main.rs: forbidden push-pull output drive on GPIO 4 (`1 << 4`), which contentions with OV2640 CAM_PIN_D0 (Y2) on ESP32-WROVER-DEV"
        )

    # 7. Active OTA slot detection via DPORT_PRO_FLASH_MMU_TABLE (`0x3FF1_0000`, LOGIC-003 / FW-004)
    if "detect_booted_ota_manager_from_mmu()" not in src:
        errors.append(
            "main.rs: must detect active booted OTA slot from DPORT_PRO_FLASH_MMU_TABLE via `detect_booted_ota_manager_from_mmu()`"
        )

    return errors


def verify_bootloader_bin() -> list[str]:
    errors: list[str] = []
    if not BOOTLOADER_BIN.is_file():
        return [f"Missing patched 2nd-stage bootloader binary: {BOOTLOADER_BIN}"]
    data = BOOTLOADER_BIN.read_bytes()
    if len(data) < 16384 or data[0] != 0xE9:
        errors.append(f"Invalid ESP32 bootloader header or size ({len(data)} bytes) in {BOOTLOADER_BIN}")
    if b"\x4c\x90\xf4\x3f" not in data:
        errors.append(f"{BOOTLOADER_BIN}: missing IO_MUX_GPIO16_REG literal 0x3FF4904C (\\x4c\\x90\\xf4\\x3f)")
    if (
        b"\x00\x2b\x00\x00" not in data
        and b"\x2c\xb9\x80\x99\x11" not in data
        and b"\x0c\xa9\x80\x88\x11" not in data
        and b"\x80\x99\x11" not in data
        and b"\x80\x88\x11" not in data
    ):
        errors.append(
            f"{BOOTLOADER_BIN}: missing GPIO 16 pull-up constant 0x2B00 in bootloader_before_init"
        )
    return errors


def main() -> int:
    all_errors: list[str] = []
    all_errors.extend(verify_udev_rules())
    all_errors.extend(verify_main_rs())
    all_errors.extend(verify_bootloader_bin())

    if all_errors:
        for err in all_errors:
            print(f"[FAIL] {err}", file=sys.stderr)
        return 1

    print("[PASS] Invariant 69: /etc/udev/rules.d/99-esp32-wrover.rules & docs/99-esp32-wrover.rules are clean (zero RUN+=/stty/TIOCM* hooks).")
    print("[PASS] Invariant 70/71/72 (main.rs): zero SD0..SD3 bit-banging, arm_rtc_wdt_15s(), hardware SPI1 0xF5+0x35 (PsramCacheF40mS40m), zero boot-time FlashStorage I/O, zero GPIO 4 output contention, MMU active OTA slot detection, and authoritative ESP-IDF v5.5.4 MMIO register map verified.")
    print(f"[PASS] Invariant 70 (bootloader): {BOOTLOADER_BIN} ({BOOTLOADER_BIN.stat().st_size} B) contains 0x3FF4_904C & 0x2B00 GPIO 16 pull-up.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
