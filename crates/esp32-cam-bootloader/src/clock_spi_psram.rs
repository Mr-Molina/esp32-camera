//! Clock, SPI flash, and PSRAM early hardware domain configuration.
//!
//! Enforces Invariants 70 and 72 by providing verified MMIO register
//! definitions and pure `#![no_std]` helper functions for clock ticks,
//! UART0 baud rate dividers, and Mask ROM SPI flash configuration.

/// eFuse block 0 read data register 0 (`soc/efuse_reg.h`).
pub const EFUSE_BLK0_RDATA0_REG: u32 = 0x3FF5_A000;

/// eFuse block 0 read data register 3 (`soc/efuse_reg.h`).
pub const EFUSE_BLK0_RDATA3_REG: u32 = 0x3FF5_A00C;

/// RTC CNTL clock configuration register (`soc/rtc_cntl_reg.h`).
pub const RTC_CNTL_CLK_CONF_REG: u32 = 0x3FF4_8070;

/// SYSCON XTAL tick configuration register (`soc/syscon_reg.h`).
pub const SYSCON_XTAL_TICK_CONF_REG: u32 = 0x3FF6_6004;

/// SYSCON PLL tick configuration register (`soc/syscon_reg.h`).
pub const SYSCON_PLL_TICK_CONF_REG: u32 = 0x3FF6_6008;

/// UART0 clock divider register (`soc/uart_reg.h`).
pub const UART0_CLKDIV_REG: u32 = 0x3FF4_0014;

/// SPI0 control register (`soc/spi_reg.h`).
pub const SPI0_CTRL_REG: u32 = 0x3FF4_3008;

/// SPI0 control register 2 (`soc/spi_reg.h`).
pub const SPI0_CTRL2_REG: u32 = 0x3FF4_3014;

/// SPI0 clock prescaler and timing register (`soc/spi_reg.h`).
pub const SPI0_CLOCK_REG: u32 = 0x3FF4_3018;

/// SPI0 user configuration register (`soc/spi_reg.h`).
pub const SPI0_USER_REG: u32 = 0x3FF4_301C;

/// SPI0 user configuration register 1 (`soc/spi_reg.h`).
pub const SPI0_USER1_REG: u32 = 0x3FF4_3020;

/// SPI1 user command register (`soc/spi_reg.h`).
pub const SPI1_CMD_REG: u32 = 0x3FF4_2000;

/// SPI1 control register (`soc/spi_reg.h`).
pub const SPI1_CTRL_REG: u32 = 0x3FF4_2008;

/// SPI1 control register 2 (`soc/spi_reg.h`).
pub const SPI1_CTRL2_REG: u32 = 0x3FF4_2014;

/// SPI1 clock prescaler and timing register (`soc/spi_reg.h`).
pub const SPI1_CLOCK_REG: u32 = 0x3FF4_2018;

/// SPI1 user configuration register 0 (`soc/spi_reg.h`).
pub const SPI1_USER_REG: u32 = 0x3FF4_201C;

/// SPI1 user configuration register 1 (`soc/spi_reg.h`).
pub const SPI1_USER1_REG: u32 = 0x3FF4_2020;

/// SPI1 user configuration register 2 (`soc/spi_reg.h`).
pub const SPI1_USER2_REG: u32 = 0x3FF4_2024;

/// SPI1 pin control register (`soc/spi_reg.h`).
pub const SPI1_PIN_REG: u32 = 0x3FF4_2034;

/// SPI1 external timing configuration register 2 (`soc/spi_reg.h`).
pub const SPI1_EXT2_REG: u32 = 0x3FF4_20F8;

/// DPORT CPU clock configuration register (`soc/dport_reg.h`).
pub const DPORT_CPU_PER_CONF_REG: u32 = 0x3FF0_003C;

/// RTC controller options 0 register (`soc/rtc_cntl_reg.h`).
pub const RTC_CNTL_OPTIONS0_REG: u32 = 0x3FF4_8000;

/// RTC controller control register (`soc/rtc_cntl_reg.h`).
pub const RTC_CNTL_REG: u32 = 0x3FF4_807C;

/// IO MUX configuration register for GPIO9 (`soc/io_mux_reg.h`).
pub const IO_MUX_GPIO9_REG: u32 = 0x3FF4_9054;

/// IO MUX configuration register for GPIO10 (`soc/io_mux_reg.h`).
pub const IO_MUX_GPIO10_REG: u32 = 0x3FF4_9058;

/// IO MUX configuration register for GPIO11 (`soc/io_mux_reg.h`).
pub const IO_MUX_GPIO11_REG: u32 = 0x3FF4_905C;

/// IO MUX configuration register for GPIO6 (`soc/io_mux_reg.h`).
pub const IO_MUX_GPIO6_REG: u32 = 0x3FF4_9060;

/// IO MUX configuration register for GPIO7 (`soc/io_mux_reg.h`).
pub const IO_MUX_GPIO7_REG: u32 = 0x3FF4_9064;

/// IO MUX configuration register for GPIO8 (`soc/io_mux_reg.h`).
pub const IO_MUX_GPIO8_REG: u32 = 0x3FF4_9068;

/// SPI1 MOSI bit length register (`soc/spi_reg.h`).
pub const SPI1_MOSI_DLEN_REG: u32 = 0x3FF4_2028;

/// SPI1 MISO bit length register (`soc/spi_reg.h`).
pub const SPI1_MISO_DLEN_REG: u32 = 0x3FF4_202C;

/// SPI1 buffer data register 0 (`soc/spi_reg.h`).
pub const SPI1_W0_REG: u32 = 0x3FF4_2080;

/// BBPLL power-down mask in `RTC_CNTL_OPTIONS0_REG`.
pub const RTC_OPTIONS0_BBPLL_PD_MASK: u32 = (1 << 18) | (1 << 10) | (1 << 8) | (1 << 6);

/// Shift offset for digital DBIAS in wake mode in `RTC_CNTL_REG`.
pub const RTC_DIG_DBIAS_WAK_SHIFT: u32 = 11;

/// Mask for digital DBIAS in wake mode in `RTC_CNTL_REG`.
pub const RTC_DIG_DBIAS_WAK_MASK: u32 = 0x7 << 11;

/// Digital voltage DBIAS setting for 1.25V operation.
pub const DIG_DBIAS_1V25: u32 = 7;

/// Shift offset for SOC clock selection in `RTC_CNTL_REG`.
pub const RTC_SOC_CLK_SEL_SHIFT: u32 = 27;

/// Mask for SOC clock selection in `RTC_CNTL_REG`.
pub const RTC_SOC_CLK_SEL_MASK: u32 = 0x3 << 27;

/// SOC clock source selection setting for PLL.
pub const RTC_SOC_CLK_PLL: u32 = 1;

/// Mask for CPU period selection in `DPORT_CPU_PER_CONF_REG`.
pub const DPORT_CPUPERIOD_SEL_MASK: u32 = 0x3;

/// CPU period selection value for 240 MHz PLL operation.
pub const DPORT_CPUPERIOD_240M: u32 = 2;

/// Shift offset for MCU pin function selection in IO MUX registers.
pub const IO_MUX_MCU_SEL_SHIFT: u32 = 12;

/// Mask for MCU pin function selection in IO MUX registers.
pub const IO_MUX_MCU_SEL_MASK: u32 = 0x7 << 12;

/// Shift offset for pin drive strength selection in IO MUX registers.
pub const IO_MUX_FUN_DRV_SHIFT: u32 = 10;

/// Mask for pin drive strength selection in IO MUX registers.
pub const IO_MUX_FUN_DRV_MASK: u32 = 0x3 << 10;

/// User command execution trigger bit in `SPI1_CMD_REG`.
pub const SPI_USR_BIT: u32 = 1 << 18;

/// Enable user command phase bit in `SPI1_USER_REG`.
pub const SPI_USR_COMMAND_BIT: u32 = 1 << 31;

/// Enable MISO read phase bit in `SPI1_USER_REG`.
pub const SPI_USR_MISO_BIT: u32 = 1 << 28;

/// Command configuration value for RDID operation in `SPI1_USER2_REG`.
pub const SPI_RDID_USER2_VAL: u32 = (7 << 28) | 0x9F;

/// Fast read mode bit in `SPI_CTRL_REG` (`soc/spi_reg.h`).
pub const SPI_FASTRD_MODE: u32 = 1 << 13;

/// Dual read mode bit in `SPI_CTRL_REG` (`soc/spi_reg.h`).
pub const SPI_FREAD_DUAL: u32 = 1 << 14;

/// Quad read mode bit in `SPI_CTRL_REG` (`soc/spi_reg.h`).
pub const SPI_FREAD_QUAD: u32 = 1 << 20;

/// Dual I/O read mode bit in `SPI_CTRL_REG` (`soc/spi_reg.h`).
pub const SPI_FREAD_DIO: u32 = 1 << 23;

/// Quad I/O read mode bit in `SPI_CTRL_REG` (`soc/spi_reg.h`).
pub const SPI_FREAD_QIO: u32 = 1 << 24;

/// CS setup timing enable bit in `SPI_USER_REG` (`soc/spi_reg.h`).
pub const SPI_CS_SETUP: u32 = 1 << 5;

/// CS hold timing enable bit in `SPI_USER_REG` (`soc/spi_reg.h`).
pub const SPI_CS_HOLD: u32 = 1 << 4;

/// Clock output edge bit in `SPI_USER_REG` (`soc/spi_reg.h`).
pub const SPI_CK_OUT_EDGE: u32 = 1 << 7;

/// User dummy phase enable bit in `SPI_USER_REG` (`soc/spi_reg.h`).
pub const SPI_USR_DUMMY: u32 = 1 << 29;

/// CS setup/hold time bitmask in `SPI_CTRL2_REG` (`soc/spi_reg.h`).
pub const SPI_SETUP_HOLD_TIME_MASK: u32 = 0x0000_00FF;

/// 1-cycle CS hold timing word in `SPI_CTRL2_REG` (`soc/spi_reg.h`).
pub const SPI_HOLD_TIME_1_CYCLE: u32 = 1 << 4;

/// Dummy cycle length bitmask in `SPI_USER1_REG` (`soc/spi_reg.h`).
pub const SPI_USR_DUMMY_CYCLELEN_MASK: u32 = 0x0000_00FF;

/// Address bit length shift offset in `SPI_USER1_REG` (`soc/spi_reg.h`).
pub const SPI_USR_ADDR_BITLEN_SHIFT: u32 = 26;

/// Address bit length bitmask in `SPI_USER1_REG` (`soc/spi_reg.h`).
pub const SPI_USR_ADDR_BITLEN_MASK: u32 = 0xFC00_0000;

/// DIO read mode address bit length for SPI0 (`esp32/rom/spi_flash.h`).
pub const SPI0_R_DIO_ADDR_BITSLEN: u32 = 27;

/// DIO read mode hardware dummy cycle length for SPI0
/// (`esp32/rom/spi_flash.h`).
pub const SPI0_R_DIO_DUMMY_CYCLELEN: u32 = 1;

/// Additional dummy length for 20 MHz SPI flash read (`esp32/rom/spi_flash.h`).
pub const ESP_ROM_SPIFLASH_DUMMY_LEN_PLUS_20M: u8 = 0;

/// Additional dummy length for 40 MHz SPI flash read (`esp32/rom/spi_flash.h`).
pub const ESP_ROM_SPIFLASH_DUMMY_LEN_PLUS_40M: u8 = 1;

/// Mask ROM SPI flash chip geometry and status definition
/// (`esp_rom_spiflash.h`).
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RomSpiflashChip {
    /// Low 32 bits of the flash JEDEC ID.
    pub device_id: u32,
    /// Total flash chip size in bytes.
    pub chip_size: u32,
    /// Erase block size in bytes.
    pub block_size: u32,
    /// Erase sector size in bytes.
    pub sector_size: u32,
    /// Write page size in bytes.
    pub page_size: u32,
    /// Status register read bitmask.
    pub status_mask: u32,
}

/// Abstract MMIO read/write access for Clock and SPI registers.
pub trait ClockMmio {
    /// Reads a 32-bit value from the specified MMIO register address.
    fn read_u32(&self, addr: u32) -> u32;

    /// Writes a 32-bit value to the specified MMIO register address.
    fn write_u32(&mut self, addr: u32, val: u32);
}

/// Computes the UART0 clock divider configuration word for 115,200 baud.
///
/// Returns a 32-bit value where bits `[19:0]` represent the integral divider
/// and bits `[23:20]` represent the 1/16th fractional divider.
#[must_use]
pub fn uart0_clkdiv_for_115200(apb_hz: u32) -> u32 {
    if apb_hz == 0 {
        return 0;
    }
    let int_part = apb_hz / 115_200;
    let rem = apb_hz % 115_200;
    let frac_part = (rem.saturating_mul(16)) / 115_200;
    (int_part & 0x000F_FFFF) | ((frac_part & 0x0F) << 20)
}

/// Configures system timer tick rates and sets UART0 baud divider for 115,200.
pub fn configure_ticks<M: ClockMmio>(mmio: &mut M, xtal_mhz: u32, pll_mhz: u32) {
    mmio.write_u32(SYSCON_XTAL_TICK_CONF_REG, xtal_mhz.saturating_sub(1));
    mmio.write_u32(SYSCON_PLL_TICK_CONF_REG, pll_mhz.saturating_sub(1));
    let apb_hz = xtal_mhz.saturating_mul(1_000_000);
    let clkdiv = uart0_clkdiv_for_115200(apb_hz);
    mmio.write_u32(UART0_CLKDIV_REG, clkdiv);
}

/// Configures timer tick rates and UART0 divider from APB frequency.
pub fn configure_ticks_apb<M: ClockMmio>(mmio: &mut M, xtal_mhz: u32, pll_mhz: u32, apb_mhz: u32) {
    mmio.write_u32(SYSCON_XTAL_TICK_CONF_REG, xtal_mhz.saturating_sub(1));
    mmio.write_u32(SYSCON_PLL_TICK_CONF_REG, pll_mhz.saturating_sub(1));
    let apb_hz = apb_mhz.saturating_mul(1_000_000);
    let clkdiv = uart0_clkdiv_for_115200(apb_hz);
    mmio.write_u32(UART0_CLKDIV_REG, clkdiv);
}

/// Configures `SPI0` and `SPI1` MMIO timing and Dual I/O cache parameters for
/// 40 MHz DIO flash operation without resetting flash pin attachments.
pub fn configure_spi_flash_40m_dio_mmio<M: ClockMmio>(mmio: &mut M) {
    let ctrl_clear_mask = SPI_FREAD_QIO | SPI_FREAD_QUAD | SPI_FREAD_DUAL;
    let ctrl_set_mask = SPI_FASTRD_MODE | SPI_FREAD_DIO;

    let spi0_ctrl = mmio.read_u32(SPI0_CTRL_REG);
    let spi0_ctrl_next = (spi0_ctrl & !ctrl_clear_mask) | ctrl_set_mask;
    mmio.write_u32(SPI0_CTRL_REG, spi0_ctrl_next);

    let dio_dummy = SPI0_R_DIO_DUMMY_CYCLELEN + u32::from(ESP_ROM_SPIFLASH_DUMMY_LEN_PLUS_40M);
    let user1 = mmio.read_u32(SPI0_USER1_REG);
    let user1_mask = SPI_USR_ADDR_BITLEN_MASK | SPI_USR_DUMMY_CYCLELEN_MASK;
    let user1_next =
        (user1 & !user1_mask) | (SPI0_R_DIO_ADDR_BITSLEN << SPI_USR_ADDR_BITLEN_SHIFT) | dio_dummy;
    mmio.write_u32(SPI0_USER1_REG, user1_next);

    let spi0_user = mmio.read_u32(SPI0_USER_REG);
    let spi0_user_next =
        (spi0_user & !SPI_CK_OUT_EDGE) | SPI_USR_DUMMY | SPI_CS_SETUP | SPI_CS_HOLD;
    mmio.write_u32(SPI0_USER_REG, spi0_user_next);

    let spi0_ctrl2 = mmio.read_u32(SPI0_CTRL2_REG);
    let spi0_ctrl2_next = (spi0_ctrl2 & !SPI_SETUP_HOLD_TIME_MASK) | SPI_HOLD_TIME_1_CYCLE;
    mmio.write_u32(SPI0_CTRL2_REG, spi0_ctrl2_next);

    let spi1_user = mmio.read_u32(SPI1_USER_REG);
    let spi1_user_next = (spi1_user & !SPI_CK_OUT_EDGE) | SPI_CS_SETUP | SPI_CS_HOLD;
    mmio.write_u32(SPI1_USER_REG, spi1_user_next);

    let spi1_ctrl2 = mmio.read_u32(SPI1_CTRL2_REG);
    let spi1_ctrl2_next = (spi1_ctrl2 & !SPI_SETUP_HOLD_TIME_MASK) | SPI_HOLD_TIME_1_CYCLE;
    mmio.write_u32(SPI1_CTRL2_REG, spi1_ctrl2_next);
}

/// Swaps byte order of 24-bit raw RDID value read from flash.
#[must_use]
pub const fn swap_rdid_24bit(raw: u32) -> u32 {
    ((raw & 0xFF) << 16) | (raw & 0xFF00) | ((raw >> 16) & 0xFF)
}

/// Clears BBPLL power-down bits in RTC controller options 0 register.
pub fn bbpll_clear_pd_mmio<M: ClockMmio>(mmio: &mut M) {
    let val = mmio.read_u32(RTC_CNTL_OPTIONS0_REG);
    mmio.write_u32(RTC_CNTL_OPTIONS0_REG, val & !RTC_OPTIONS0_BBPLL_PD_MASK);
}

/// Configures digital DBIAS voltage to 1.25V for BBPLL in RTC control register.
pub fn bbpll_set_dbias_1v25_mmio<M: ClockMmio>(mmio: &mut M) {
    let val = mmio.read_u32(RTC_CNTL_REG);
    let next = (val & !RTC_DIG_DBIAS_WAK_MASK) | (DIG_DBIAS_1V25 << RTC_DIG_DBIAS_WAK_SHIFT);
    mmio.write_u32(RTC_CNTL_REG, next);
}

/// Switches CPU clock to 240 MHz PLL and updates APB tick divider.
pub fn switch_cpu_240m_pll_mmio<M: ClockMmio>(mmio: &mut M) {
    let dport = mmio.read_u32(DPORT_CPU_PER_CONF_REG);
    let dport_next = (dport & !DPORT_CPUPERIOD_SEL_MASK) | DPORT_CPUPERIOD_240M;
    mmio.write_u32(DPORT_CPU_PER_CONF_REG, dport_next);

    configure_ticks_apb(mmio, 40, 80, 80);

    let clk_conf = mmio.read_u32(RTC_CNTL_CLK_CONF_REG);
    let clk_conf_next =
        (clk_conf & !RTC_SOC_CLK_SEL_MASK) | (RTC_SOC_CLK_PLL << RTC_SOC_CLK_SEL_SHIFT);
    mmio.write_u32(RTC_CNTL_CLK_CONF_REG, clk_conf_next);
}

/// Configures IO MUX pin routing and drive strength for SPI flash interface.
pub fn configure_flash_iomux_mmio<M: ClockMmio>(mmio: &mut M) {
    for reg in [
        IO_MUX_GPIO7_REG,
        IO_MUX_GPIO8_REG,
        IO_MUX_GPIO9_REG,
        IO_MUX_GPIO10_REG,
        IO_MUX_GPIO11_REG,
    ] {
        let val = mmio.read_u32(reg);
        let next = (val & !IO_MUX_MCU_SEL_MASK) | (2 << IO_MUX_MCU_SEL_SHIFT);
        mmio.write_u32(reg, next);
    }

    let val6 = mmio.read_u32(IO_MUX_GPIO6_REG);
    let mask6 = IO_MUX_MCU_SEL_MASK | IO_MUX_FUN_DRV_MASK;
    let next6 = (val6 & !mask6) | (1 << IO_MUX_MCU_SEL_SHIFT) | (2 << IO_MUX_FUN_DRV_SHIFT);
    mmio.write_u32(IO_MUX_GPIO6_REG, next6);
}

/// Configures SPI1 registers to execute the RDID flash command.
pub fn spi1_setup_rdid_mmio<M: ClockMmio>(mmio: &mut M) {
    mmio.write_u32(SPI1_CTRL_REG, 0);
    mmio.write_u32(SPI1_USER_REG, SPI_USR_COMMAND_BIT | SPI_USR_MISO_BIT);
    mmio.write_u32(SPI1_USER1_REG, 0);
    mmio.write_u32(SPI1_USER2_REG, SPI_RDID_USER2_VAL);
    mmio.write_u32(SPI1_MOSI_DLEN_REG, 0);
    mmio.write_u32(SPI1_MISO_DLEN_REG, 23);
}

#[cfg(target_arch = "xtensa")]
// JUSTIFICATION: Mask ROM SPI flash, cache, and flash chip descriptor symbols
// for Xtensa ESP32 require FFI linkage against fixed ROM addresses in
// `linkall.x`.
// SAFETY: Symbol addresses are verified against `esp32.rom.ld` and
// `esp32.rom.spiflash_legacy.ld`; signatures match `esp32/rom/spi_flash.h`.
extern "C" {
    static mut g_rom_flashchip: RomSpiflashChip;
    static mut g_rom_spiflash_dummy_len_plus: [u8; 3];

    fn Cache_Read_Disable_rom(cpu: u32);
    fn Cache_Flush_rom(cpu: u32);
    fn esp_rom_spiflash_config_param(
        device_id: u32,
        chip_size: u32,
        block_size: u32,
        sector_size: u32,
        page_size: u32,
        status_mask: u32,
    ) -> i32;
    fn esp_rom_spiflash_wait_idle(spi: *mut RomSpiflashChip) -> i32;
    fn rom_i2c_writeReg(block: u8, host_id: u8, reg_add: u8, data: u8);
    fn ets_delay_us(us: u32);
    fn ets_update_cpu_frequency_rom(ticks_per_us: u32);
    fn ets_efuse_get_spiconfig() -> u32;
    fn esp_rom_spiflash_config_clk(freqdiv: u8, spi: u8) -> i32;
    fn gpio_matrix_in(gpio: u32, signal_idx: u32, inv: bool);
    fn gpio_matrix_out(gpio: u32, signal_idx: u32, out_inv: bool, oen_inv: bool);
}

#[cfg(target_arch = "xtensa")]
/// Configures BBPLL to 240 MHz CPU frequency and 80 MHz APB bus clock.
pub fn configure_bbpll_240m_apb_80m() {
    let mut mmio = XtensaClockMmio;
    bbpll_clear_pd_mmio(&mut mmio);
    // JUSTIFICATION: Direct Mask ROM calls to configure BBPLL analog I2C
    // registers and update ROM frequency state for 240 MHz operation.
    // SAFETY: Fixed ROM addresses and register definitions match ESP32
    // TRM and ESP-IDF bootloader clock initialization sequences.
    unsafe {
        for (reg, val) in [(0, 0x18), (1, 0x20), (4, 0x9A), (10, 0), (12, 0)] {
            rom_i2c_writeReg(0x66, 4, reg, val);
        }
        bbpll_set_dbias_1v25_mmio(&mut mmio);
        ets_delay_us(3);
        for (reg, val) in [(11, 0xC3), (9, 0x74), (2, 0), (3, 0x1C), (5, 0xC6)] {
            rom_i2c_writeReg(0x66, 4, reg, val);
        }
        ets_delay_us(160);
        switch_cpu_240m_pll_mmio(&mut mmio);
        ets_update_cpu_frequency_rom(240);
        ets_delay_us(30);
    }
}

#[cfg(target_arch = "xtensa")]
/// Configures `SPI0` and `SPI1` for 40 MHz DIO SPI flash, attaches GPIO matrix
/// if unconfigured, and synchronizes the ROM flash chip descriptor.
///
/// # Errors
///
/// Returns `Err(())` if any Mask ROM SPI configuration or status check fails.
#[allow(clippy::result_unit_err)]
pub fn attach_flash_4mb_dio() -> Result<(), ()> {
    // JUSTIFICATION: 2nd-stage bootloader runs in IRAM/DRAM and calibrates
    // SPI flash for 40 MHz DIO mode, GPIO matrix routing, and cache flush.
    // SAFETY: Symbol addresses match fixed ESP32 Mask ROM exports, PRO
    // cache is disabled during reconfiguration and flushed before return,
    // and MMIO accesses target valid SPI1/EFUSE peripheral registers.
    unsafe {
        let chip_ptr = core::ptr::addr_of_mut!(g_rom_flashchip);
        if esp_rom_spiflash_wait_idle(chip_ptr) != 0 {
            return Err(());
        }

        Cache_Read_Disable_rom(0);
        let _ = esp_rom_spiflash_config_clk(2, 1);

        let mut mmio = XtensaClockMmio;
        let saved_ctrl = mmio.read_u32(SPI1_CTRL_REG);
        let saved_user = mmio.read_u32(SPI1_USER_REG);
        let saved_user1 = mmio.read_u32(SPI1_USER1_REG);
        let saved_user2 = mmio.read_u32(SPI1_USER2_REG);

        spi1_setup_rdid_mmio(&mut mmio);
        mmio.write_u32(SPI1_W0_REG, 0);
        let cmd = mmio.read_u32(SPI1_CMD_REG) | SPI_USR_BIT;
        mmio.write_u32(SPI1_CMD_REG, cmd);

        for _ in 0..100_000 {
            if (mmio.read_u32(SPI1_CMD_REG) & SPI_USR_BIT) == 0 {
                break;
            }
        }

        let rdid = swap_rdid_24bit(mmio.read_u32(SPI1_W0_REG));
        mmio.write_u32(SPI1_CTRL_REG, saved_ctrl);
        mmio.write_u32(SPI1_USER_REG, saved_user);
        mmio.write_u32(SPI1_USER1_REG, saved_user1);
        mmio.write_u32(SPI1_USER2_REG, saved_user2);

        let mut device_id = (*chip_ptr).device_id;
        if rdid != 0 && rdid != 0x00FF_FFFF {
            device_id = rdid;
            (*chip_ptr).device_id = device_id;
        }

        let spiconfig = ets_efuse_get_spiconfig();
        let pkg_ver = (mmio.read_u32(EFUSE_BLK0_RDATA3_REG) >> 9) & 0x7;
        if spiconfig == 0 && !matches!(pkg_ver, 2 | 4 | 5 | 6) {
            gpio_matrix_out(11, 5, false, false);
            for (gpio, sig) in [(7, 1), (8, 2), (10, 4), (9, 3)] {
                gpio_matrix_out(gpio, sig, false, false);
                gpio_matrix_in(gpio, sig, false);
            }
            configure_flash_iomux_mmio(&mut mmio);
        }

        let _ = esp_rom_spiflash_config_clk(2, 0);
        let _ = esp_rom_spiflash_config_clk(2, 1);

        let dummy_ptr = core::ptr::addr_of_mut!(g_rom_spiflash_dummy_len_plus).cast::<u8>();
        core::ptr::write_volatile(dummy_ptr, ESP_ROM_SPIFLASH_DUMMY_LEN_PLUS_40M);
        core::ptr::write_volatile(dummy_ptr.add(1), ESP_ROM_SPIFLASH_DUMMY_LEN_PLUS_40M);

        configure_spi_flash_40m_dio_mmio(&mut mmio);

        let param_ret =
            esp_rom_spiflash_config_param(device_id, 4 * 1024 * 1024, 64 * 1024, 4096, 256, 0xFFFF);
        if param_ret != 0 {
            return Err(());
        }

        Cache_Flush_rom(0);

        if esp_rom_spiflash_wait_idle(chip_ptr) != 0 {
            return Err(());
        }
    }
    Ok(())
}

#[cfg(target_arch = "xtensa")]
/// Real hardware MMIO accessor for Xtensa Clock and SPI registers.
pub struct XtensaClockMmio;

#[cfg(target_arch = "xtensa")]
impl ClockMmio for XtensaClockMmio {
    #[inline]
    fn read_u32(&self, addr: u32) -> u32 {
        // JUSTIFICATION: Reading hardware MMIO registers on physical Xtensa
        // SoC.
        // SAFETY: Address must be a valid 4-byte aligned ESP32 MMIO register.
        unsafe { core::ptr::read_volatile(addr as *const u32) }
    }

    #[inline]
    fn write_u32(&mut self, addr: u32, val: u32) {
        // JUSTIFICATION: Writing hardware MMIO registers on physical Xtensa
        // SoC.
        // SAFETY: Address must be a valid 4-byte aligned ESP32 MMIO register.
        unsafe { core::ptr::write_volatile(addr as *mut u32, val) }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct MockClockMmio {
        regs: [(u32, u32); 32],
        regs_len: usize,
        log: [(u32, u32); 32],
        log_len: usize,
    }

    impl MockClockMmio {
        const fn new() -> Self {
            Self {
                regs: [(0, 0); 32],
                regs_len: 0,
                log: [(0, 0); 32],
                log_len: 0,
            }
        }

        fn seed(&mut self, addr: u32, val: u32) {
            for i in 0..self.regs_len {
                if self.regs[i].0 == addr {
                    self.regs[i].1 = val;
                    return;
                }
            }
            if self.regs_len < self.regs.len() {
                self.regs[self.regs_len] = (addr, val);
                self.regs_len += 1;
            }
        }
    }

    impl ClockMmio for MockClockMmio {
        fn read_u32(&self, addr: u32) -> u32 {
            for i in 0..self.regs_len {
                if self.regs[i].0 == addr {
                    return self.regs[i].1;
                }
            }
            0
        }

        fn write_u32(&mut self, addr: u32, val: u32) {
            self.seed(addr, val);
            if self.log_len < self.log.len() {
                self.log[self.log_len] = (addr, val);
                self.log_len += 1;
            }
        }
    }

    #[test]
    fn test_uart0_clkdiv_arithmetic() {
        assert_eq!(uart0_clkdiv_for_115200(0), 0);

        // 40 MHz XTAL: 40_000_000 / 115_200 = 347 rem 25600; frac = 3
        let div_40m = uart0_clkdiv_for_115200(40_000_000);
        assert_eq!(div_40m & 0x000F_FFFF, 347);
        assert_eq!((div_40m >> 20) & 0x0F, 3);
        assert_eq!(div_40m, 347 | (3 << 20));

        // 80 MHz APB: 80_000_000 / 115_200 = 694 rem 51200; frac = 7
        let div_80m = uart0_clkdiv_for_115200(80_000_000);
        assert_eq!(div_80m & 0x000F_FFFF, 694);
        assert_eq!((div_80m >> 20) & 0x0F, 7);
        assert_eq!(div_80m, 694 | (7 << 20));
    }

    #[test]
    fn test_configure_ticks() {
        let mut mock = MockClockMmio::new();
        configure_ticks(&mut mock, 40, 480);

        assert_eq!(mock.log_len, 3);
        assert_eq!(mock.log[0], (SYSCON_XTAL_TICK_CONF_REG, 39));
        assert_eq!(mock.log[1], (SYSCON_PLL_TICK_CONF_REG, 479));
        assert_eq!(mock.log[2], (UART0_CLKDIV_REG, 347 | (3 << 20)));
    }

    #[test]
    fn test_configure_spi_flash_40m_dio_mmio_sets_and_preserves_bits() {
        let mut mock = MockClockMmio::new();
        // Seed unrelated upper/lower bits plus conflicting QIO/QUAD/DUAL flags.
        mock.seed(
            SPI0_CTRL_REG,
            (1 << 26) | SPI_FREAD_QIO | SPI_FREAD_QUAD | SPI_FREAD_DUAL,
        );
        let spi1_ctrl_initial = (1 << 25) | SPI_FREAD_QIO | SPI_FREAD_QUAD | SPI_FREAD_DUAL;
        mock.seed(SPI1_CTRL_REG, spi1_ctrl_initial);
        mock.seed(SPI0_USER1_REG, (0x123 << 8) | (11 << 26) | 0x87);
        mock.seed(SPI0_USER_REG, (1 << 12) | SPI_CK_OUT_EDGE);
        mock.seed(SPI0_CTRL2_REG, 0xABCD_00FF);
        mock.seed(SPI1_USER_REG, (1 << 31) | SPI_CK_OUT_EDGE);
        mock.seed(SPI1_CTRL2_REG, 0x5555_00FF);

        configure_spi_flash_40m_dio_mmio(&mut mock);

        assert_eq!(mock.log_len, 6);
        let spi0_ctrl = mock.read_u32(SPI0_CTRL_REG);
        assert_eq!(spi0_ctrl, (1 << 26) | SPI_FASTRD_MODE | SPI_FREAD_DIO);
        assert_eq!(
            spi0_ctrl & (SPI_FASTRD_MODE | SPI_FREAD_DIO),
            SPI_FASTRD_MODE | SPI_FREAD_DIO
        );
        assert_eq!(
            spi0_ctrl & (SPI_FREAD_QIO | SPI_FREAD_QUAD | SPI_FREAD_DUAL),
            0
        );

        // Assert SPI1_CTRL_REG is untouched and was never written.
        assert_eq!(mock.read_u32(SPI1_CTRL_REG), spi1_ctrl_initial);
        assert!(!mock.log[..mock.log_len]
            .iter()
            .any(|&(addr, _)| addr == SPI1_CTRL_REG));

        let spi0_user1 = mock.read_u32(SPI0_USER1_REG);
        assert_eq!(
            spi0_user1 & SPI_USR_DUMMY_CYCLELEN_MASK,
            SPI0_R_DIO_DUMMY_CYCLELEN + u32::from(ESP_ROM_SPIFLASH_DUMMY_LEN_PLUS_40M)
        );
        assert_eq!(
            spi0_user1 >> SPI_USR_ADDR_BITLEN_SHIFT,
            SPI0_R_DIO_ADDR_BITSLEN
        );
        assert_eq!(
            spi0_user1,
            (0x123 << 8) | (SPI0_R_DIO_ADDR_BITSLEN << SPI_USR_ADDR_BITLEN_SHIFT) | 2
        );

        // Assert SPI0_USER_REG clears SPI_CK_OUT_EDGE and sets SPI_USR_DUMMY
        // along with CS timing.
        let spi0_user = mock.read_u32(SPI0_USER_REG);
        assert_eq!(spi0_user & SPI_CK_OUT_EDGE, 0);
        assert!((spi0_user & SPI_USR_DUMMY) != 0);
        assert!((spi0_user & SPI_CS_SETUP) != 0);
        assert!((spi0_user & SPI_CS_HOLD) != 0);
        assert_eq!(
            spi0_user,
            (1 << 12) | SPI_USR_DUMMY | SPI_CS_SETUP | SPI_CS_HOLD
        );
        assert_eq!(mock.read_u32(SPI0_CTRL2_REG), 0xABCD_0010);
        let spi1_user = mock.read_u32(SPI1_USER_REG);
        assert_eq!(spi1_user & SPI_CK_OUT_EDGE, 0);
        assert_eq!(spi1_user, (1 << 31) | SPI_CS_SETUP | SPI_CS_HOLD);
        assert_eq!(mock.read_u32(SPI1_CTRL2_REG), 0x5555_0010);
    }

    #[test]
    fn test_bbpll_iomux_and_rdid_helpers() {
        assert_eq!(swap_rdid_24bit(0x0016_40EF), 0x00EF_4016);

        let mut mock = MockClockMmio::new();
        configure_ticks_apb(&mut mock, 40, 80, 80);
        assert_eq!(mock.log_len, 3);
        assert_eq!(mock.log[0], (SYSCON_XTAL_TICK_CONF_REG, 39));
        assert_eq!(mock.log[1], (SYSCON_PLL_TICK_CONF_REG, 79));
        assert_eq!(mock.log[2], (UART0_CLKDIV_REG, 694 | (7 << 20)));

        mock.seed(
            RTC_CNTL_OPTIONS0_REG,
            0x1230_0000 | RTC_OPTIONS0_BBPLL_PD_MASK,
        );
        bbpll_clear_pd_mmio(&mut mock);
        assert_eq!(mock.read_u32(RTC_CNTL_OPTIONS0_REG), 0x1230_0000);

        mock.seed(RTC_CNTL_REG, 0x5678_0000);
        bbpll_set_dbias_1v25_mmio(&mut mock);
        let expected_reg = 0x5678_0000 | (DIG_DBIAS_1V25 << RTC_DIG_DBIAS_WAK_SHIFT);
        assert_eq!(mock.read_u32(RTC_CNTL_REG), expected_reg);

        mock.seed(DPORT_CPU_PER_CONF_REG, 0x9ABC_0000);
        mock.seed(RTC_CNTL_CLK_CONF_REG, 0xC6F0_0000 | RTC_SOC_CLK_SEL_MASK);
        switch_cpu_240m_pll_mmio(&mut mock);
        assert_eq!(
            mock.read_u32(DPORT_CPU_PER_CONF_REG),
            0x9ABC_0000 | DPORT_CPUPERIOD_240M
        );
        let expected_clk_conf = 0xC6F0_0000 | (RTC_SOC_CLK_PLL << RTC_SOC_CLK_SEL_SHIFT);
        assert_eq!(mock.read_u32(RTC_CNTL_CLK_CONF_REG), expected_clk_conf);

        for reg in [
            IO_MUX_GPIO7_REG,
            IO_MUX_GPIO8_REG,
            IO_MUX_GPIO9_REG,
            IO_MUX_GPIO10_REG,
            IO_MUX_GPIO11_REG,
        ] {
            mock.seed(reg, 0x1111_0000);
        }
        mock.seed(IO_MUX_GPIO6_REG, 0x2222_0000);
        configure_flash_iomux_mmio(&mut mock);

        for reg in [
            IO_MUX_GPIO7_REG,
            IO_MUX_GPIO8_REG,
            IO_MUX_GPIO9_REG,
            IO_MUX_GPIO10_REG,
            IO_MUX_GPIO11_REG,
        ] {
            assert_eq!(
                mock.read_u32(reg),
                0x1111_0000 | (2 << IO_MUX_MCU_SEL_SHIFT)
            );
        }
        assert_eq!(
            mock.read_u32(IO_MUX_GPIO6_REG),
            0x2222_0000 | (1 << IO_MUX_MCU_SEL_SHIFT) | (2 << IO_MUX_FUN_DRV_SHIFT)
        );

        mock.seed(SPI1_CTRL_REG, 0xFFFF_FFFF);
        mock.seed(SPI1_USER_REG, 0xFFFF_FFFF);
        mock.seed(SPI1_USER1_REG, 0xFFFF_FFFF);
        mock.seed(SPI1_USER2_REG, 0xFFFF_FFFF);
        mock.seed(SPI1_MOSI_DLEN_REG, 0xFFFF_FFFF);
        mock.seed(SPI1_MISO_DLEN_REG, 0xFFFF_FFFF);
        spi1_setup_rdid_mmio(&mut mock);

        assert_eq!(mock.read_u32(SPI1_CTRL_REG), 0);
        assert_eq!(
            mock.read_u32(SPI1_USER_REG),
            SPI_USR_COMMAND_BIT | SPI_USR_MISO_BIT
        );
        assert_eq!(mock.read_u32(SPI1_USER1_REG), 0);
        assert_eq!(mock.read_u32(SPI1_USER2_REG), SPI_RDID_USER2_VAL);
        assert_eq!(mock.read_u32(SPI1_MOSI_DLEN_REG), 0);
        assert_eq!(mock.read_u32(SPI1_MISO_DLEN_REG), 23);
    }
}
