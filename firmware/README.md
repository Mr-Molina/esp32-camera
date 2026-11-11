# ESP32-WROVER 2nd-Stage Bootloader Provenance

This directory holds the 2nd-stage bootloader binary required for flashing and operating the ESP32-WROVER camera firmware.

- **Target Binary:** `firmware/bootloader_v554_gpio16.bin` (flash offset `0x1000`)
- **Base Toolchain:** ESP-IDF v5.5.4 (`components/bootloader_support`)
- **Status:** **Pending repository owner upload** (`BUILD-03`)

---

## 1. Status & Upload Notice (BUILD-03)

The pre-compiled binary `firmware/bootloader_v554_gpio16.bin` is currently pending drop-in upload by the repository owner.

- `.gitignore` includes an explicit exception rule:
  ```gitignore
  *.bin
  !firmware/bootloader_v554_gpio16.bin
  ```
- Until the repository owner uploads `firmware/bootloader_v554_gpio16.bin`:
  - `espflash flash`, `espflash save-image`, and `ota_deploy.py --stage-only` will fail on clean clones with file-not-found errors when attempting to package the bootloader.
  - `scripts/verify_cold_boot_invariants.py` defaults to exit 1 to protect production gates; pass `--allow-missing-bootloader` in development or CI environments to report an informational skip for the binary verification step while asserting static source and udev invariants.

---

## 2. Bootloader Provenance & Technical Specification

### The GPIO16 PSRAM-CS Cold-Boot Hazard
On the ESP32-WROVER module, **GPIO16** is connected directly to the Chip Select (`/CS`) pin of the integrated SPI PSRAM (AP_3216 / ESP-PSRAM32).

During cold power-on, the GPIO16 pin floats or idles low before the application firmware initializes GPIO matrices and PSRAM drivers. A floating or low `/CS` asserts the PSRAM chip, causing it to respond to and drive data onto the shared SPI flash bus lines during early 2nd-stage bootloader reads. This bus contention results in flash read corruption, watchdog timeouts, and unrecoverable boot loops.

### Register Configuration in `bootloader_before_init`
To eliminate this hardware hazard, the 2nd-stage bootloader is compiled from ESP-IDF v5.5.4 with a hardware patch placed in `bootloader_before_init()` (executed before general bootloader subsystem initialization):

1. **`IO_MUX_GPIO16_REG` (`0x3FF4_904C`)**:
   Configured with value **`0x2B00`**:
   - `MCU_SEL = 1` (selects standard GPIO function rather than peripheral routing)
   - `FUN_PU = 1` (bit 8, enables internal pull-up resistor)
2. **Complementary MMIO Registers**:
   - `RTC_CNTL_HOLD_FORCE_REG` (`0x3FF4_80C8`): Clears pad hold forces.
   - `RTC_IO_DIG_PAD_HOLD_REG` (`0x3FF4_8474`): Disables digital pad hold gating.
   - `GPIO_OUT_W1TS_REG` (`0x3FF4_4008`): Writes bit `1 << 16` high to drive GPIO16 output high immediately.
   - `GPIO_FUNC16_OUT_SEL_CFG_REG` (`0x3FF4_4570`): Connects GPIO output signal index `0x100` (simple output).
   - `GPIO_ENABLE_W1TS_REG` (`0x3FF4_4024`): Enables output driver for GPIO16 (`1 << 16`).

Driving GPIO16 HIGH at the earliest stage of 2nd-stage boot ensures the PSRAM remains deselected in standby mode throughout bootloader execution.

---

## 3. Verification Commands

### SHA-256 Checksum Verification
Once `bootloader_v554_gpio16.bin` is placed in this directory, verify its SHA-256 hash:

```bash
# Linux / macOS
sha256sum firmware/bootloader_v554_gpio16.bin
```

```powershell
# Windows PowerShell
Get-FileHash -Path firmware\bootloader_v554_gpio16.bin -Algorithm SHA256
```

### Static Invariant Verification Gate
Run the repository verification suite from the project root:

```bash
python scripts/verify_cold_boot_invariants.py
```

The script automatically validates:
1. Bootloader image header (magic byte `0xE9` at offset 0, size >= 16,384 bytes).
2. Literal `IO_MUX_GPIO16_REG` address `0x3FF4904C` (`\x4c\x90\xf4\x3f`).
3. GPIO16 pull-up constant `0x2B00` (`\x00\x2b\x00\x00`) or Xtensa opcode equivalents (`movi.n+slli`).
4. Register address literals for `RTC_CNTL_HOLD_FORCE_REG` (`0x3FF480C8`), `RTC_IO_DIG_PAD_HOLD_REG` (`0x3FF48474`), `GPIO_OUT_W1TS_REG` (`0x3FF44008`), `GPIO_FUNC16_OUT_SEL_CFG_REG` (`0x3FF44570`), and `GPIO_ENABLE_W1TS_REG` (`0x3FF44024`).

### Live Hardware Cold-Boot Verification
When hardware is connected and running:

```bash
python scripts/verify_cold_boot_invariants.py --live-status-url http://<camera-ip>/status
```

Asserts that `/status` responds with HTTP 200, contains `reset_reason` (e.g. `POWERON_RESET`), reports `uptime_ms`, and shows normal boot operation from the active OTA slot.
