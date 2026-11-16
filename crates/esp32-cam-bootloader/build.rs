//! Build script for `esp32-cam-bootloader`.
//!
//! Generates `linkall.x` with SRAM-only memory geometry and Mask ROM symbols
//! when targeting `xtensa-esp32-none-elf`.

use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-env-changed=CARGO_CFG_TARGET_ARCH");

    let target_arch = env::var("CARGO_CFG_TARGET_ARCH").unwrap_or_default();
    if target_arch == "xtensa" {
        let out_dir =
            PathBuf::from(env::var("OUT_DIR").expect("OUT_DIR must be set by Cargo during build"));
        let linkall_path = out_dir.join("linkall.x");
        let linker_script = r#"MEMORY
{
  iram_seg (RWX) : ORIGIN = 0x40078000, LENGTH = 0x8000
  dram_seg (RW)  : ORIGIN = 0x3FFF0000, LENGTH = 0x8000
}
ENTRY(_start)
PROVIDE(Cache_Read_Disable_rom = 0x40009ab8);
PROVIDE(Cache_Flush_rom = 0x40009a14);
PROVIDE(Cache_Read_Init_rom = 0x40009950);
PROVIDE(mmu_init = 0x400095a4);
PROVIDE(cache_flash_mmu_set_rom = 0x400095e0);
PROVIDE(Cache_Read_Enable_rom = 0x40009a84);
PROVIDE(uart_tx_flush = 0x40009258);
PROVIDE(ets_printf = 0x40007d54);
PROVIDE(ets_delay_us = 0x40008534);
PROVIDE(esp_rom_spiflash_attach = 0x40062a6c);
PROVIDE(esp_rom_spiflash_config_readmode = 0x40062b64);
PROVIDE(esp_rom_spiflash_config_clk = 0x40062bc8);
PROVIDE(esp_rom_spiflash_config_param = 0x40063238);
PROVIDE(esp_rom_spiflash_read = 0x40062ed8);
PROVIDE(esp_rom_spiflash_write = 0x40062d50);
PROVIDE(esp_rom_spiflash_erase_sector = 0x40062ccc);
PROVIDE(esp_rom_spiflash_read_status = 0x4006226c);
PROVIDE(esp_rom_spiflash_write_status = 0x400622f0);
PROVIDE(esp_rom_spiflash_wait_idle = 0x400622c0);
PROVIDE(esp_rom_spiflash_read_user_cmd = 0x400621b0);
PROVIDE(rom_i2c_writeReg = 0x400041a4);
PROVIDE(ets_update_cpu_frequency_rom = 0x40008550);
PROVIDE(ets_efuse_get_spiconfig = 0x40008658);
PROVIDE(gpio_matrix_in = 0x40009edc);
PROVIDE(gpio_matrix_out = 0x40009f0c);
PROVIDE(g_rom_flashchip = 0x3ffae270);
PROVIDE(g_rom_spiflash_dummy_len_plus = 0x3ffae290);
SECTIONS
{
  .iram.text : ALIGN(4)
  {
    _iram_start = ABSOLUTE(.);
    *(.literal .text .literal.* .text.*)
    . = ALIGN(4);
    _iram_end = ABSOLUTE(.);
  } > iram_seg
  .bss (NOLOAD) : ALIGN(8)
  {
    _bss_start = ABSOLUTE(.);
    *(.bss .bss.* COMMON)
    . = ALIGN(8);
    _bss_end = ABSOLUTE(.);
  } > dram_seg
  . = MAX(., 0x3FFF0030);
  .dram.data : ALIGN(16)
  {
    _data_start = ABSOLUTE(.);
    *(.rodata .rodata.* .data .data.*)
    . = ALIGN(4);
    _data_end = ABSOLUTE(.);
  } > dram_seg
  _stack_top = 0x3FFF7FF0;
  /DISCARD/ :
  {
    *(.eh_frame*)
    *(.note*)
  }
}
"#;
        fs::write(&linkall_path, linker_script).expect("failed to write linkall.x to OUT_DIR");
        println!("cargo:rustc-link-search={}", out_dir.display());
    }
}
