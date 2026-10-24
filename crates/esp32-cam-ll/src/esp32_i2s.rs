//! ESP32 I2S0 camera parallel interface low-level driver (`target/esp32/ll_cam.c`).
//!
//! Provides MMIO register constants, `IO_MUX` pad address lookup, GPIO matrix signal indices,
//! and the [`Esp32I2sCamera`] controller implementing `config`, `set_pin`, `start`, `stop`,
//! `deinit`, `vsync_intr_enable`, `set_sample_mode`, and `capture_jpeg_frame_polled`.

use core::ptr;
use esp32_cam_types::{CameraError, PixelFormat};

use crate::dma_filter::{select_sample_mode, DmaFilterKind, I2sSamplingMode, SampleModeConfig};
use crate::dma_ring::{build_circular_ring, DmaSizeConfig, Lldesc};

/// Physical base address of the `DPORT` peripheral on ESP32 (`0x3FF0_0000`).
pub const DPORT_BASE_ADDR: usize = 0x3FF0_0000;

/// Physical address of `DPORT_PERIP_CLK_EN_REG` on ESP32 (`0x3FF0_00C0`).
pub const DPORT_PERIP_CLK_EN_REG: usize = 0x3FF0_00C0;

/// Physical address of `DPORT_PERIP_RST_EN_REG` on ESP32 (`0x3FF0_00C4`).
pub const DPORT_PERIP_RST_EN_REG: usize = 0x3FF0_00C4;

/// Bit mask for `DPORT_I2S0_CLK_EN` (`bit 4`).
pub const DPORT_I2S0_CLK_EN: u32 = 1 << 4;

/// Bit mask for `DPORT_I2S0_RST` (`bit 4`).
pub const DPORT_I2S0_RST: u32 = 1 << 4;

/// Physical base address of the `GPIO` peripheral on ESP32 (`0x3FF4_4000`).
pub const GPIO_BASE_ADDR: usize = 0x3FF4_4000;

/// Physical address of `GPIO_ENABLE_W1TC_REG` (`0x3FF4_4028`, clears output enable for GPIO `0..=31`).
pub const GPIO_ENABLE_W1TC_REG: usize = 0x3FF4_4028;

/// Physical address of `GPIO_ENABLE1_W1TC_REG` (`0x3FF4_4034`, clears output enable for GPIO `32..=39`).
pub const GPIO_ENABLE1_W1TC_REG: usize = 0x3FF4_4034;

/// Physical address of `GPIO_IN_REG` (`0x3FF4_403C`, reads input level for GPIO `0..=31`).
pub const GPIO_IN_REG: usize = 0x3FF4_403C;

/// Physical address of `GPIO_IN1_REG` (`0x3FF4_4040`, reads input level for GPIO `32..=39`).
pub const GPIO_IN1_REG: usize = 0x3FF4_4040;

/// Physical address of `GPIO_PIN0_REG` (`0x3FF4_4088`, stride `4` bytes per GPIO `0..=39`).
pub const GPIO_PIN0_REG: usize = 0x3FF4_4088;

/// Physical address of `GPIO_FUNC0_IN_SEL_CFG_REG` (`0x3FF4_4130`, stride `4` bytes per signal index `0..=255`).
pub const GPIO_FUNC0_IN_SEL_CFG_REG: usize = 0x3FF4_4130;

/// Physical address of `GPIO_FUNC0_OUT_SEL_CFG_REG` (`0x3FF4_4530`, stride `4` bytes per GPIO `0..=33`).
pub const GPIO_FUNC0_OUT_SEL_CFG_REG: usize = 0x3FF4_4530;

/// Physical base address of the `RTC_IO` peripheral on ESP32 (`0x3FF4_8400`).
pub const RTC_IO_BASE_ADDR: usize = 0x3FF4_8400;

/// Physical address of `RTC_IO_SENSOR_PADS_REG` (`0x3FF4_847C`, controls digital/RTC mux for GPIO `36..=39`).
pub const RTC_IO_SENSOR_PADS_REG: usize = 0x3FF4_847C;

/// Physical address of `RTC_IO_ADC_PAD_REG` (`0x3FF4_8480`, controls digital/RTC mux for GPIO `34..=35`).
pub const RTC_IO_ADC_PAD_REG: usize = 0x3FF4_8480;

/// Physical address of `RTC_IO_PAD_DAC1_REG` (`0x3FF4_8484`, controls digital/RTC mux and pull-up for GPIO `25`).
pub const RTC_IO_PAD_DAC1_REG: usize = 0x3FF4_8484;

/// Physical address of `RTC_IO_PAD_DAC2_REG` (`0x3FF4_8488`, controls digital/RTC mux for GPIO `26`).
pub const RTC_IO_PAD_DAC2_REG: usize = 0x3FF4_8488;

/// Physical address of `RTC_IO_TOUCH_PAD0_REG` (`0x3FF4_8494`, controls digital/RTC mux for GPIO `4`).
pub const RTC_IO_TOUCH_PAD0_REG: usize = 0x3FF4_8494;

/// Physical base address of the `IO_MUX` peripheral on ESP32 (`0x3FF4_9000`).
pub const IO_MUX_BASE_ADDR: usize = 0x3FF4_9000;

/// Physical base address of the `I2S0` peripheral on ESP32 (`0x3FF4_F000`).
pub const I2S0_BASE_ADDR: usize = 0x3FF4_F000;

/// Maximum valid GPIO number on ESP32 (`GPIO_NUM_MAX = 40`, valid indices `0..=39`).
pub const ESP32_GPIO_NUM_MAX: u8 = 40;

/// GPIO matrix input signal index for `I2S0I_DATA_IN0_IDX` (`140`, `D0..D7` = `140..=147`).
pub const I2S0I_DATA_IN0_IDX: u8 = 140;
/// GPIO matrix input signal index for `I2S0I_DATA_IN1_IDX` (`141`).
pub const I2S0I_DATA_IN1_IDX: u8 = 141;
/// GPIO matrix input signal index for `I2S0I_DATA_IN2_IDX` (`142`).
pub const I2S0I_DATA_IN2_IDX: u8 = 142;
/// GPIO matrix input signal index for `I2S0I_DATA_IN3_IDX` (`143`).
pub const I2S0I_DATA_IN3_IDX: u8 = 143;
/// GPIO matrix input signal index for `I2S0I_DATA_IN4_IDX` (`144`).
pub const I2S0I_DATA_IN4_IDX: u8 = 144;
/// GPIO matrix input signal index for `I2S0I_DATA_IN5_IDX` (`145`).
pub const I2S0I_DATA_IN5_IDX: u8 = 145;
/// GPIO matrix input signal index for `I2S0I_DATA_IN6_IDX` (`146`).
pub const I2S0I_DATA_IN6_IDX: u8 = 146;
/// GPIO matrix input signal index for `I2S0I_DATA_IN7_IDX` (`147`).
pub const I2S0I_DATA_IN7_IDX: u8 = 147;

/// GPIO matrix input signal index for `I2S0I_WS_IN_IDX` (`PCLK`, `28`, ESP32 TRM Table 4-2).
pub const I2S0I_WS_IN_IDX: u8 = 28;

/// GPIO matrix input signal index for `I2S0I_H_SYNC_IDX` (`HREF`, `190`).
pub const I2S0I_H_SYNC_IDX: u8 = 190;

/// GPIO matrix input signal index for `I2S0I_V_SYNC_IDX` (`VSYNC`, `191`).
pub const I2S0I_V_SYNC_IDX: u8 = 191;

/// GPIO matrix input signal index for `I2S0I_H_ENABLE_IDX` (`192`).
pub const I2S0I_H_ENABLE_IDX: u8 = 192;

/// Constant logic-high (`1`) virtual GPIO matrix source (`0x38` in `ll_cam.c:452`).
pub const GPIO_MATRIX_CONST_ONE_INPUT: u8 = 0x38;

/// `I2S0.conf` register offset (`0x08`) and bit definitions.
pub const I2S_CONF_REG_OFFSET: usize = 0x08;
pub const I2S_RX_RESET_BIT: u32 = 1 << 1;
pub const I2S_RX_FIFO_RESET_BIT: u32 = 1 << 3;
pub const I2S_RX_START_BIT: u32 = 1 << 5;
pub const I2S_RX_SLAVE_MOD_BIT: u32 = 1 << 7;

/// `I2S0.int_raw` (`0x0C`), `I2S0.int_st` (`0x10`), `I2S0.int_ena` (`0x14`), and `I2S0.int_clr` (`0x18`).
pub const I2S_INT_RAW_REG_OFFSET: usize = 0x0C;
pub const I2S_INT_ST_REG_OFFSET: usize = 0x10;
pub const I2S_INT_ENA_REG_OFFSET: usize = 0x14;
pub const I2S_INT_CLR_REG_OFFSET: usize = 0x18;
pub const I2S_IN_SUC_EOF_INT_BIT: u32 = 1 << 9;

/// `I2S0.timing` (`0x1C`), `I2S0.fifo_conf` (`0x20`), `I2S0.rx_eof_num` (`0x24`), and `I2S0.conf_chan` (`0x2C`).
pub const I2S_TIMING_REG_OFFSET: usize = 0x1C;
pub const I2S_FIFO_CONF_REG_OFFSET: usize = 0x20;
pub const I2S_RXEOF_NUM_REG_OFFSET: usize = 0x24;
pub const I2S_CONF_CHAN_REG_OFFSET: usize = 0x2C;

/// `I2S0.in_link` (`0x34`) and `I2S0.in_eof_des_addr` (`0x3C`) definitions.
pub const I2S_IN_LINK_REG_OFFSET: usize = 0x34;
pub const I2S_IN_EOF_DES_ADDR_REG_OFFSET: usize = 0x3C;
pub const I2S_INLINK_ADDR_MASK: u32 = 0x000F_FFFF;
pub const I2S_INLINK_STOP_BIT: u32 = 1 << 28;
pub const I2S_INLINK_START_BIT: u32 = 1 << 29;

/// `I2S0.lc_conf` (`0x60`) register offset and DMA reset bit definitions.
pub const I2S_LC_CONF_REG_OFFSET: usize = 0x60;
pub const I2S_LC_IN_RST_BIT: u32 = 1 << 0;
pub const I2S_LC_AHBM_FIFO_RST_BIT: u32 = 1 << 2;
pub const I2S_LC_AHBM_RST_BIT: u32 = 1 << 3;

/// `I2S0.pd_conf` (`0xA4`), `I2S0.conf2` (`0xA8`), `I2S0.clkm_conf` (`0xAC`), and `I2S0.sample_rate_conf` (`0xB0`) offsets.
pub const I2S_PD_CONF_REG_OFFSET: usize = 0xA4;
pub const I2S_PD_FIFO_MEM_FORCE_PU: u32 = 0x0A;
pub const I2S_CONF2_REG_OFFSET: usize = 0xA8;
pub const I2S_CLKM_CONF_REG_OFFSET: usize = 0xAC;
pub const I2S_SAMPLE_RATE_CONF_REG_OFFSET: usize = 0xB0;
pub const I2S_CAMERA_EN_BIT: u32 = 1 << 0;
pub const I2S_LCD_EN_BIT: u32 = 1 << 5;

/// Standard JPEG DMA descriptor node buffer size (`2048` bytes).
pub const JPEG_DMA_NODE_SIZE: usize = 2048;

/// Standard JPEG DMA half-buffer size (`4096` bytes = `2` nodes of `2048` bytes).
pub const JPEG_DMA_HALF_BUFFER_SIZE: usize = 4096;

/// Unpacked JPEG byte count per `4096`-byte DMA half-buffer (`1024` bytes, 1 byte per 4-byte `DmaElement`).
pub const JPEG_UNPACKED_HALF_BUFFER_SIZE: usize = 1024;

/// Maps an ESP32 GPIO number (`0..=39`) to its physical `IO_MUX` pad register address
/// (`0x3FF4_9004..=0x3FF4_908C`, ESP32 Technical Reference Manual Table 4-3).
///
/// Returns `None` for unbonded/reserved GPIO indices (`24`, `28..=31`, or `>= 40`).
#[inline]
pub const fn io_mux_reg_addr(gpio_num: u8) -> Option<usize> {
    let offset = match gpio_num {
        0 => 0x44,
        1 => 0x88,
        2 => 0x40,
        3 => 0x84,
        4 => 0x48,
        5 => 0x6C,
        6 => 0x60,
        7 => 0x64,
        8 => 0x68,
        9 => 0x54,
        10 => 0x58,
        11 => 0x5C,
        12 => 0x34,
        13 => 0x38,
        14 => 0x30,
        15 => 0x3C,
        16 => 0x4C,
        17 => 0x50,
        18 => 0x70,
        19 => 0x74,
        20 => 0x78,
        21 => 0x7C,
        22 => 0x80,
        23 => 0x8C,
        25 => 0x24,
        26 => 0x28,
        27 => 0x2C,
        32 => 0x1C,
        33 => 0x20,
        34 => 0x14,
        35 => 0x18,
        36 => 0x04,
        37 => 0x08,
        38 => 0x0C,
        39 => 0x10,
        _ => return None,
    };
    Some(IO_MUX_BASE_ADDR + offset)
}

/// Pin routing configuration passed to `set_pin` (`ll_cam_set_pin` in `target/esp32/ll_cam.c:387-454`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct CamPinConfig {
    pub pin_vsync: i8,
    pub pin_href: i8,
    pub pin_pclk: i8,
    pub pin_xclk: i8,
    pub data_pins: [i8; 8],
    pub vsync_invert: bool,
}

/// Recorded GPIO matrix input routing entry (`gpio_matrix_in(pin, signal_idx, inv)`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct GpioMatrixBinding {
    pub gpio_num: u8,
    pub signal_idx: u8,
    pub inverted: bool,
}

/// Simulated / shadow MMIO register block for `I2S0` and `DPORT` on ESP32.
#[repr(C, align(4))]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct I2s0RegisterBlock {
    pub dport_clk_en: u32,
    pub dport_rst_en: u32,
    pub conf: u32,
    pub int_raw: u32,
    pub int_st: u32,
    pub int_ena: u32,
    pub int_clr: u32,
    pub timing: u32,
    pub fifo_conf: u32,
    pub rx_eof_num: u32,
    pub in_link: u32,
    pub lc_conf: u32,
    pub clkm_conf: u32,
    pub sample_rate_conf: u32,
    pub pd_conf: u32,
    pub conf2: u32,
    pub conf_chan: u32,
}

#[inline]
fn volatile_write_u32(reg: &mut u32, val: u32) {
    let ptr = ptr::from_mut(reg);
    // SAFETY:
    // 1. Alignment & Non-null: `ptr` is obtained via `ptr::from_mut` from a valid `&mut u32` reference,
    //    guaranteeing non-null and 4-byte alignment.
    // 2. Memory layout: `u32` is a 4-byte primitive register word inside `#[repr(C, align(4))]`.
    // 3. Provenance & Lifetime: `ptr` has valid mutable provenance for the duration of this call.
    // 4. Exclusive aliasing: `reg: &mut u32` holds unique access to the target word.
    // 5. Volatile semantics: `write_volatile` ensures the compiler does not elide or reorder register updates.
    // 6. Panic safety: `write_volatile` on a stack/struct `u32` cannot unwind.
    unsafe {
        ptr::write_volatile(ptr, val);
    }
}

#[inline]
fn volatile_read_u32(reg: &u32) -> u32 {
    let ptr = ptr::from_ref(reg);
    // SAFETY:
    // 1. Alignment & Non-null: `ptr` is obtained via `ptr::from_ref` from a valid `&u32` reference,
    //    guaranteeing non-null and 4-byte alignment.
    // 2. Memory layout: `u32` is a 4-byte primitive register word inside `#[repr(C, align(4))]`.
    // 3. Provenance & Lifetime: `ptr` has valid shared provenance for the duration of this call.
    // 4. Aliasing: No concurrent mutable reference exists during this read.
    // 5. Volatile semantics: `read_volatile` prevents compiler read elision.
    // 6. Panic safety: `read_volatile` on a valid `u32` cannot unwind.
    unsafe { ptr::read_volatile(ptr) }
}

#[cfg(target_arch = "xtensa")]
#[inline]
fn xtensa_mmio_write(addr: usize, val: u32) {
    let ptr = addr as *mut u32;
    // SAFETY:
    // 1. Alignment & Non-null: Caller passes 4-byte aligned ESP32 MMIO register addresses in `0x3FF0_0000..0x3FF7_FFFF`.
    // 2. Memory layout: ESP32 DPORT, GPIO, RTC_IO, IO_MUX, and I2S0 peripheral registers are 32-bit memory-mapped words.
    // 3. Provenance: Hardware peripheral address space on Xtensa ESP32 target.
    // 4. Aliasing: Executed under peripheral ownership of `Esp32I2sCamera`.
    // 5. Volatile semantics: `write_volatile` is required for hardware MMIO side effects.
    // 6. Panic safety: Hardware MMIO store does not unwind.
    unsafe {
        ptr::write_volatile(ptr, val);
    }
}

#[cfg(target_arch = "xtensa")]
#[inline]
fn xtensa_mmio_read(addr: usize) -> u32 {
    let ptr = addr as *const u32;
    // SAFETY:
    // 1. Alignment & Non-null: Caller passes 4-byte aligned ESP32 MMIO register addresses in `0x3FF0_0000..0x3FF7_FFFF`.
    // 2. Memory layout: ESP32 DPORT, GPIO, RTC_IO, IO_MUX, and I2S0 peripheral registers are 32-bit memory-mapped words.
    // 3. Provenance: Hardware peripheral address space on Xtensa ESP32 target.
    // 4. Aliasing: Volatile hardware register read does not alias Rust stack or heap references.
    // 5. Volatile semantics: `read_volatile` prevents compiler elision or reordering of hardware status reads.
    // 6. Panic safety: Hardware MMIO load does not unwind.
    unsafe { ptr::read_volatile(ptr) }
}

#[cfg(target_arch = "xtensa")]
#[inline]
fn xtensa_configure_input_pad(gpio_num: u8, enable_pullup: bool) {
    // Route RTC_IO-capable pads to the digital IO_MUX before configuring IO_MUX and GPIO matrix
    match gpio_num {
        36..=39 => {
            let v = xtensa_mmio_read(RTC_IO_SENSOR_PADS_REG)
                & !((1 << 30) | (1 << 29) | (1 << 28) | (1 << 27));
            xtensa_mmio_write(RTC_IO_SENSOR_PADS_REG, v);
        }
        34 | 35 => {
            let v = xtensa_mmio_read(RTC_IO_ADC_PAD_REG) & !((1 << 29) | (1 << 28));
            xtensa_mmio_write(RTC_IO_ADC_PAD_REG, v);
        }
        25 => {
            let mut v = xtensa_mmio_read(RTC_IO_PAD_DAC1_REG)
                & !((1 << 17) | (1 << 28) | (1 << 18) | (1 << 10));
            if enable_pullup {
                v |= 1 << 27;
            } else {
                v &= !(1 << 27);
            }
            xtensa_mmio_write(RTC_IO_PAD_DAC1_REG, v);
        }
        26 => {
            let v = xtensa_mmio_read(RTC_IO_PAD_DAC2_REG)
                & !((1 << 17) | (1 << 28) | (1 << 18) | (1 << 10));
            xtensa_mmio_write(RTC_IO_PAD_DAC2_REG, v);
        }
        4 => {
            let v = xtensa_mmio_read(RTC_IO_TOUCH_PAD0_REG) & !(1 << 19);
            xtensa_mmio_write(RTC_IO_TOUCH_PAD0_REG, v);
        }
        _ => {}
    }

    // Configure IO_MUX: MCU_SEL = 2 (GPIO function, bits 12..14), FUN_DRV = 2 (bits 10..11),
    // FUN_IE = 1 (bit 9), and optional FUN_WPU = 1 (bit 8).
    if let Some(mux_addr) = io_mux_reg_addr(gpio_num) {
        let mux_val = if enable_pullup { 0x2B00 } else { 0x2A00 };
        xtensa_mmio_write(mux_addr, mux_val);
    }

    // Disable GPIO output driver
    if gpio_num < 32 {
        xtensa_mmio_write(GPIO_ENABLE_W1TC_REG, 1u32 << gpio_num);
    } else if gpio_num < ESP32_GPIO_NUM_MAX {
        xtensa_mmio_write(GPIO_ENABLE1_W1TC_REG, 1u32 << (gpio_num - 32));
    }

    // Clear open-drain / interrupt bits in GPIO_PINn_REG
    xtensa_mmio_write(GPIO_PIN0_REG + 4 * (gpio_num as usize), 0);

    // Disconnect output matrix for output-capable pads (0..=33) using SIG_GPIO_OUT_IDX (0x100)
    if gpio_num < 34 {
        xtensa_mmio_write(GPIO_FUNC0_OUT_SEL_CFG_REG + 4 * (gpio_num as usize), 0x100);
    }
}

#[cfg(target_arch = "xtensa")]
#[inline]
fn xtensa_read_gpio_level(gpio_num: u8) -> bool {
    if gpio_num < 32 {
        ((xtensa_mmio_read(GPIO_IN_REG) >> gpio_num) & 1) != 0
    } else {
        ((xtensa_mmio_read(GPIO_IN1_REG) >> (gpio_num - 32)) & 1) != 0
    }
}

#[inline]
fn find_jpeg_soi(buf: &[u8]) -> Option<usize> {
    if buf.len() < 3 {
        return None;
    }
    let limit = buf.len() - 3;
    let mut i = 0;
    while i <= limit {
        if buf[i] == 0xFF && buf[i + 1] == 0xD8 && buf[i + 2] == 0xFF {
            return Some(i);
        }
        i += 1;
    }
    None
}

#[inline]
fn find_jpeg_eoi(buf: &[u8]) -> Option<usize> {
    if buf.len() < 2 {
        return None;
    }
    let limit = buf.len() - 2;
    let mut i = 0;
    while i <= limit {
        if buf[i] == 0xFF && buf[i + 1] == 0xD9 {
            return Some(i);
        }
        i += 1;
    }
    None
}

/// Appends an unpacked `1024`-byte DMA chunk into `jpeg_out`, handling `SOI` (`0xFF, 0xD8, 0xFF`)
/// alignment (including cross-chunk straddling) and returning `Ok(Some(total_len))` as soon as
/// `EOI` (`0xFF, 0xD9`) is encountered after `SOI`.
fn append_unpacked_jpeg_chunk(
    chunk: &[u8],
    jpeg_out: &mut [u8],
    written: &mut usize,
    soi_found: &mut bool,
    prev_tail: &mut [u8; 2],
    prev_tail_len: &mut usize,
) -> Result<Option<usize>, CameraError> {
    let n = chunk.len();
    if n == 0 {
        return Ok(None);
    }

    if !*soi_found {
        if *prev_tail_len == 2 && prev_tail[0] == 0xFF && prev_tail[1] == 0xD8 && chunk[0] == 0xFF {
            if 2 + n > jpeg_out.len() {
                return Err(CameraError::BufferTooSmall);
            }
            jpeg_out[0] = 0xFF;
            jpeg_out[1] = 0xD8;
            jpeg_out[2..2 + n].copy_from_slice(chunk);
            *written = 2 + n;
            *soi_found = true;
        } else if *prev_tail_len >= 1
            && prev_tail[*prev_tail_len - 1] == 0xFF
            && n >= 2
            && chunk[0] == 0xD8
            && chunk[1] == 0xFF
        {
            if 1 + n > jpeg_out.len() {
                return Err(CameraError::BufferTooSmall);
            }
            jpeg_out[0] = 0xFF;
            jpeg_out[1..1 + n].copy_from_slice(chunk);
            *written = 1 + n;
            *soi_found = true;
        } else if let Some(soi_off) = find_jpeg_soi(chunk) {
            let copy_len = n - soi_off;
            if copy_len > jpeg_out.len() {
                return Err(CameraError::BufferTooSmall);
            }
            jpeg_out[..copy_len].copy_from_slice(&chunk[soi_off..]);
            *written = copy_len;
            *soi_found = true;
        } else {
            if n >= 2 {
                prev_tail[0] = chunk[n - 2];
                prev_tail[1] = chunk[n - 1];
                *prev_tail_len = 2;
            } else {
                prev_tail[0] = chunk[0];
                *prev_tail_len = 1;
            }
            return Ok(None);
        }

        if *written >= 5 {
            if let Some(rel_eoi) = find_jpeg_eoi(&jpeg_out[3..*written]) {
                return Ok(Some(3 + rel_eoi + 2));
            }
        }
        return Ok(None);
    }

    if *written + n > jpeg_out.len() {
        return Err(CameraError::BufferTooSmall);
    }
    let search_start = (*written).saturating_sub(1).max(3);
    jpeg_out[*written..*written + n].copy_from_slice(chunk);
    *written += n;

    if *written >= search_start + 2 {
        if let Some(rel_eoi) = find_jpeg_eoi(&jpeg_out[search_start..*written]) {
            return Ok(Some(search_start + rel_eoi + 2));
        }
    }

    Ok(None)
}

/// Low-level ESP32 I2S0 camera peripheral controller (`target/esp32/ll_cam.c`).
#[derive(Debug, Clone)]
pub struct Esp32I2sCamera {
    regs: I2s0RegisterBlock,
    sampling_mode: I2sSamplingMode,
    filter: DmaFilterKind,
    in_bytes_per_pixel: u8,
    fb_bytes_per_pixel: u8,
    vsync_pin: Option<u8>,
    vsync_invert: bool,
    vsync_intr_enabled: bool,
    gpio_bindings: [GpioMatrixBinding; 12],
    gpio_binding_count: usize,
    running: bool,
}

impl Default for Esp32I2sCamera {
    fn default() -> Self {
        Self::new()
    }
}

impl Esp32I2sCamera {
    /// Creates an uninitialized [`Esp32I2sCamera`] controller.
    #[inline]
    pub const fn new() -> Self {
        Self {
            regs: I2s0RegisterBlock {
                dport_clk_en: 0,
                dport_rst_en: 0,
                conf: 0,
                int_raw: 0,
                int_st: 0,
                int_ena: 0,
                int_clr: 0,
                timing: 0,
                fifo_conf: 0,
                rx_eof_num: 0,
                in_link: 0,
                lc_conf: 0,
                clkm_conf: 0,
                sample_rate_conf: 0,
                pd_conf: 0,
                conf2: 0,
                conf_chan: 0,
            },
            sampling_mode: I2sSamplingMode::Sm0A000B00,
            filter: DmaFilterKind::Jpeg,
            in_bytes_per_pixel: 1,
            fb_bytes_per_pixel: 1,
            vsync_pin: None,
            vsync_invert: true,
            vsync_intr_enabled: false,
            gpio_bindings: [GpioMatrixBinding {
                gpio_num: 0,
                signal_idx: 0,
                inverted: false,
            }; 12],
            gpio_binding_count: 0,
            running: false,
        }
    }

    /// Returns a shared reference to the underlying register block for inspection.
    #[inline]
    pub const fn regs(&self) -> &I2s0RegisterBlock {
        &self.regs
    }

    /// Returns the active [`I2sSamplingMode`].
    #[inline]
    pub const fn sampling_mode(&self) -> I2sSamplingMode {
        self.sampling_mode
    }

    /// Returns the active [`DmaFilterKind`].
    #[inline]
    pub const fn filter(&self) -> DmaFilterKind {
        self.filter
    }

    /// Returns the configured GPIO matrix bindings (`PCLK`, `VSYNC`, `HREF`, `D0..D7`, `H_ENABLE`).
    #[inline]
    pub fn gpio_bindings(&self) -> &[GpioMatrixBinding] {
        &self.gpio_bindings[..self.gpio_binding_count]
    }

    /// Returns `true` if VSYNC GPIO interrupt is enabled.
    #[inline]
    pub const fn is_vsync_intr_enabled(&self) -> bool {
        self.vsync_intr_enabled
    }

    /// Returns `true` if I2S0 RX DMA capture is currently running.
    #[inline]
    pub const fn is_running(&self) -> bool {
        self.running
    }

    /// Returns the DMA alignment requirement on ESP32 (`0`, matching `ll_cam_get_dma_align` in `ll_cam.c:465-468`).
    #[inline]
    pub const fn dma_align(&self) -> u8 {
        0
    }

    /// Configures GPIO matrix routing for `PCLK`, `VSYNC`, `HREF`, `D0..D7`, and `0x38 -> H_ENABLE`,
    /// rejecting any pin `< 0` or `>= 40` (`ll_cam_set_pin` in `target/esp32/ll_cam.c:387-454`).
    pub fn set_pin(&mut self, cfg: &CamPinConfig) -> Result<(), CameraError> {
        if cfg.pin_vsync < 0
            || (cfg.pin_vsync as u8) >= ESP32_GPIO_NUM_MAX
            || cfg.pin_pclk < 0
            || (cfg.pin_pclk as u8) >= ESP32_GPIO_NUM_MAX
            || cfg.pin_href < 0
            || (cfg.pin_href as u8) >= ESP32_GPIO_NUM_MAX
        {
            return Err(CameraError::InvalidArg);
        }

        let mut i = 0;
        while i < 8 {
            let pin = cfg.data_pins[i];
            if pin < 0 || (pin as u8) >= ESP32_GPIO_NUM_MAX {
                return Err(CameraError::InvalidArg);
            }
            i += 1;
        }

        self.vsync_pin = Some(cfg.pin_vsync as u8);
        self.vsync_invert = cfg.vsync_invert;
        self.vsync_intr_enabled = false;

        self.gpio_bindings[0] = GpioMatrixBinding {
            gpio_num: cfg.pin_pclk as u8,
            signal_idx: I2S0I_WS_IN_IDX,
            inverted: false,
        };
        self.gpio_bindings[1] = GpioMatrixBinding {
            gpio_num: cfg.pin_vsync as u8,
            signal_idx: I2S0I_V_SYNC_IDX,
            inverted: false,
        };
        self.gpio_bindings[2] = GpioMatrixBinding {
            gpio_num: cfg.pin_href as u8,
            signal_idx: I2S0I_H_SYNC_IDX,
            inverted: false,
        };

        let mut d = 0;
        while d < 8 {
            self.gpio_bindings[3 + d] = GpioMatrixBinding {
                gpio_num: cfg.data_pins[d] as u8,
                signal_idx: I2S0I_DATA_IN0_IDX + (d as u8),
                inverted: false,
            };
            d += 1;
        }

        self.gpio_bindings[11] = GpioMatrixBinding {
            gpio_num: GPIO_MATRIX_CONST_ONE_INPUT,
            signal_idx: I2S0I_H_ENABLE_IDX,
            inverted: false,
        };
        self.gpio_binding_count = 12;

        #[cfg(target_arch = "xtensa")]
        {
            xtensa_configure_input_pad(cfg.pin_pclk as u8, false);
            xtensa_configure_input_pad(cfg.pin_vsync as u8, true);
            xtensa_configure_input_pad(cfg.pin_href as u8, false);

            let mut idx = 0;
            while idx < 8 {
                xtensa_configure_input_pad(cfg.data_pins[idx] as u8, false);
                idx += 1;
            }

            let mut b_idx = 0;
            while b_idx < 12 {
                let b = self.gpio_bindings[b_idx];
                let inv = if b.signal_idx == I2S0I_V_SYNC_IDX {
                    self.vsync_invert
                } else {
                    b.inverted
                };
                let sel_val = (1u32 << 7) | ((inv as u32) << 6) | ((b.gpio_num as u32) & 0x3F);
                xtensa_mmio_write(
                    GPIO_FUNC0_IN_SEL_CFG_REG + 4 * (b.signal_idx as usize),
                    sel_val,
                );
                b_idx += 1;
            }
        }

        Ok(())
    }

    /// Enables and configures the `I2S0` peripheral in parallel camera slave mode (`ll_cam_config`
    /// in `target/esp32/ll_cam.c:330-376`).
    pub fn config(&mut self) -> Result<(), CameraError> {
        let clk = volatile_read_u32(&self.regs.dport_clk_en) | DPORT_I2S0_CLK_EN;
        volatile_write_u32(&mut self.regs.dport_clk_en, clk);
        let rst = volatile_read_u32(&self.regs.dport_rst_en) & !DPORT_I2S0_RST;
        volatile_write_u32(&mut self.regs.dport_rst_en, rst);

        // Pulse RX reset, RX FIFO reset, LC in_rst, AHBM FIFO reset, AHBM reset
        volatile_write_u32(
            &mut self.regs.conf,
            I2S_RX_RESET_BIT | I2S_RX_FIFO_RESET_BIT,
        );
        volatile_write_u32(&mut self.regs.conf, I2S_RX_SLAVE_MOD_BIT);
        volatile_write_u32(&mut self.regs.lc_conf, 0);

        // Force I2S0 FIFO and PLC memory power-up (`FIFO_FORCE_PU | PLC_MEM_FORCE_PU`)
        volatile_write_u32(&mut self.regs.pd_conf, I2S_PD_FIFO_MEM_FORCE_PU);

        // Enable LCD + Camera mode (`I2S0.conf2.lcd_en = 1; I2S0.conf2.camera_en = 1;`)
        volatile_write_u32(&mut self.regs.conf2, I2S_LCD_EN_BIT | I2S_CAMERA_EN_BIT);

        // Clock divider `clkm_div_num = 2, clkm_div_a = 0, clkm_div_b = 0`
        volatile_write_u32(&mut self.regs.clkm_conf, 2);

        // FIFO config: `dscr_en = 1 (bit 12), rx_fifo_mod_force_en = 1 (bit 20), rx_fifo_mod = sampling_mode (bits 16..18)`
        let fifo_val = (1 << 12) | (1 << 20) | ((self.sampling_mode as u32) << 16);
        volatile_write_u32(&mut self.regs.fifo_conf, fifo_val);

        // `rx_chan_mod = 1`, `rx_bits_mod = 0`, `timing.rx_dsync_sw = 1 (bit 21)`
        volatile_write_u32(&mut self.regs.conf_chan, 1 << 3);
        volatile_write_u32(&mut self.regs.sample_rate_conf, 0);
        volatile_write_u32(&mut self.regs.timing, 1 << 21);

        #[cfg(target_arch = "xtensa")]
        {
            let hw_clk = xtensa_mmio_read(DPORT_PERIP_CLK_EN_REG) | DPORT_I2S0_CLK_EN;
            xtensa_mmio_write(DPORT_PERIP_CLK_EN_REG, hw_clk);

            let hw_rst = xtensa_mmio_read(DPORT_PERIP_RST_EN_REG);
            xtensa_mmio_write(DPORT_PERIP_RST_EN_REG, hw_rst | DPORT_I2S0_RST);
            xtensa_mmio_write(DPORT_PERIP_RST_EN_REG, hw_rst & !DPORT_I2S0_RST);

            // Reset RX module and RX FIFO, then enable RX slave mode
            xtensa_mmio_write(
                I2S0_BASE_ADDR + I2S_CONF_REG_OFFSET,
                I2S_RX_RESET_BIT | I2S_RX_FIFO_RESET_BIT,
            );
            xtensa_mmio_write(I2S0_BASE_ADDR + I2S_CONF_REG_OFFSET, 0);
            xtensa_mmio_write(I2S0_BASE_ADDR + I2S_CONF_REG_OFFSET, I2S_RX_SLAVE_MOD_BIT);

            // Reset DMA in-link and AHB master FIFO
            xtensa_mmio_write(
                I2S0_BASE_ADDR + I2S_LC_CONF_REG_OFFSET,
                I2S_LC_IN_RST_BIT | I2S_LC_AHBM_FIFO_RST_BIT | I2S_LC_AHBM_RST_BIT,
            );
            xtensa_mmio_write(I2S0_BASE_ADDR + I2S_LC_CONF_REG_OFFSET, 0);

            // Force I2S0 FIFO and PLC memory power-up (`FIFO_FORCE_PU | PLC_MEM_FORCE_PU`)
            xtensa_mmio_write(
                I2S0_BASE_ADDR + I2S_PD_CONF_REG_OFFSET,
                I2S_PD_FIFO_MEM_FORCE_PU,
            );

            // Enable LCD + Camera slave mode (`0x21`)
            xtensa_mmio_write(
                I2S0_BASE_ADDR + I2S_CONF2_REG_OFFSET,
                I2S_LCD_EN_BIT | I2S_CAMERA_EN_BIT,
            );

            // Configure I2S0 clock divider (`CLK_EN = 1 << 20`, `CLKM_DIV_NUM = 2`)
            xtensa_mmio_write(I2S0_BASE_ADDR + I2S_CLKM_CONF_REG_OFFSET, (1 << 20) | 2);

            // Configure FIFO (`rx_fifo_mod_force_en = 1`, `rx_fifo_mod = sampling_mode`, `dscr_en = 1`)
            xtensa_mmio_write(I2S0_BASE_ADDR + I2S_FIFO_CONF_REG_OFFSET, fifo_val);

            // Configure single RX channel (`rx_chan_mod = 1`), `rx_bits_mod = 0`, `rx_dsync_sw = 1`
            xtensa_mmio_write(I2S0_BASE_ADDR + I2S_CONF_CHAN_REG_OFFSET, 1 << 3);
            let sr = xtensa_mmio_read(I2S0_BASE_ADDR + I2S_SAMPLE_RATE_CONF_REG_OFFSET)
                & !(0x3F << 18);
            xtensa_mmio_write(I2S0_BASE_ADDR + I2S_SAMPLE_RATE_CONF_REG_OFFSET, sr);
            xtensa_mmio_write(I2S0_BASE_ADDR + I2S_TIMING_REG_OFFSET, 1 << 21);
        }

        Ok(())
    }

    /// Configures sampling mode and FIFO mode for the requested pixel format, XCLK frequency, and sensor PID
    /// (`ll_cam_set_sample_mode` in `target/esp32/ll_cam.c:571-619`).
    pub fn set_sample_mode(
        &mut self,
        pix_format: PixelFormat,
        xclk_freq_hz: u32,
        sensor_pid: u16,
    ) -> Result<SampleModeConfig, CameraError> {
        let mode_cfg = select_sample_mode(pix_format, xclk_freq_hz, sensor_pid)?;
        self.sampling_mode = mode_cfg.sampling_mode;
        self.filter = mode_cfg.filter;
        self.in_bytes_per_pixel = mode_cfg.in_bytes_per_pixel;
        self.fb_bytes_per_pixel = mode_cfg.fb_bytes_per_pixel;

        let fifo_val = (volatile_read_u32(&self.regs.fifo_conf) & !(0x7 << 16))
            | (1 << 12)
            | (1 << 20)
            | ((self.sampling_mode as u32) << 16);
        volatile_write_u32(&mut self.regs.fifo_conf, fifo_val);

        #[cfg(target_arch = "xtensa")]
        {
            xtensa_mmio_write(I2S0_BASE_ADDR + I2S_FIFO_CONF_REG_OFFSET, fifo_val);
        }

        Ok(mode_cfg)
    }

    /// Enables or disables the VSYNC GPIO interrupt (`ll_cam_vsync_intr_enable`).
    pub fn vsync_intr_enable(&mut self, en: bool) {
        self.vsync_intr_enabled = en;
    }

    /// Starts I2S0 DMA reception for a frame (`ll_cam_start` in `target/esp32/ll_cam.c:305-328`).
    pub fn start(&mut self, dma_cfg: &DmaSizeConfig, first_desc_addr: u32) -> bool {
        if dma_cfg.dma_half_buffer_size == 0 {
            return false;
        }
        let conf_stopped = volatile_read_u32(&self.regs.conf) & !I2S_RX_START_BIT;
        volatile_write_u32(&mut self.regs.conf, conf_stopped);

        // Enable `in_suc_eof` interrupt
        volatile_write_u32(&mut self.regs.int_clr, I2S_IN_SUC_EOF_INT_BIT);
        let int_ena = volatile_read_u32(&self.regs.int_ena) | I2S_IN_SUC_EOF_INT_BIT;
        volatile_write_u32(&mut self.regs.int_ena, int_ena);

        // `I2S0.rx_eof_num = cam->dma_half_buffer_size / sizeof(dma_elem_t)`
        let eof_num = (dma_cfg.dma_half_buffer_size / 4) as u32;
        volatile_write_u32(&mut self.regs.rx_eof_num, eof_num);

        // `I2S0.in_link.addr = addr & 0xfffff; I2S0.in_link.start = 1;`
        let link_val = (first_desc_addr & I2S_INLINK_ADDR_MASK) | I2S_INLINK_START_BIT;
        volatile_write_u32(&mut self.regs.in_link, link_val);

        // `I2S0.conf.rx_start = 1;`
        volatile_write_u32(&mut self.regs.conf, conf_stopped | I2S_RX_START_BIT);

        #[cfg(target_arch = "xtensa")]
        {
            let conf_base =
                (xtensa_mmio_read(I2S0_BASE_ADDR + I2S_CONF_REG_OFFSET) & !I2S_RX_START_BIT)
                    | I2S_RX_SLAVE_MOD_BIT;
            xtensa_mmio_write(I2S0_BASE_ADDR + I2S_CONF_REG_OFFSET, conf_base);

            xtensa_mmio_write(
                I2S0_BASE_ADDR + I2S_INT_CLR_REG_OFFSET,
                I2S_IN_SUC_EOF_INT_BIT,
            );
            let hw_int_ena = xtensa_mmio_read(I2S0_BASE_ADDR + I2S_INT_ENA_REG_OFFSET)
                | I2S_IN_SUC_EOF_INT_BIT;
            xtensa_mmio_write(I2S0_BASE_ADDR + I2S_INT_ENA_REG_OFFSET, hw_int_ena);

            // Pulse RX reset and RX FIFO reset
            xtensa_mmio_write(
                I2S0_BASE_ADDR + I2S_CONF_REG_OFFSET,
                conf_base | I2S_RX_RESET_BIT,
            );
            xtensa_mmio_write(I2S0_BASE_ADDR + I2S_CONF_REG_OFFSET, conf_base);
            xtensa_mmio_write(
                I2S0_BASE_ADDR + I2S_CONF_REG_OFFSET,
                conf_base | I2S_RX_FIFO_RESET_BIT,
            );
            xtensa_mmio_write(I2S0_BASE_ADDR + I2S_CONF_REG_OFFSET, conf_base);

            // Pulse DMA in-link and AHB resets
            xtensa_mmio_write(I2S0_BASE_ADDR + I2S_LC_CONF_REG_OFFSET, I2S_LC_IN_RST_BIT);
            xtensa_mmio_write(I2S0_BASE_ADDR + I2S_LC_CONF_REG_OFFSET, 0);
            xtensa_mmio_write(
                I2S0_BASE_ADDR + I2S_LC_CONF_REG_OFFSET,
                I2S_LC_AHBM_FIFO_RST_BIT,
            );
            xtensa_mmio_write(I2S0_BASE_ADDR + I2S_LC_CONF_REG_OFFSET, 0);
            xtensa_mmio_write(I2S0_BASE_ADDR + I2S_LC_CONF_REG_OFFSET, I2S_LC_AHBM_RST_BIT);
            xtensa_mmio_write(I2S0_BASE_ADDR + I2S_LC_CONF_REG_OFFSET, 0);

            // Ensure I2S0 FIFO memory remains powered up and GPIO matrix input bindings stay latched
            xtensa_mmio_write(
                I2S0_BASE_ADDR + I2S_PD_CONF_REG_OFFSET,
                I2S_PD_FIFO_MEM_FORCE_PU,
            );
            let mut b_idx = 0;
            while b_idx < self.gpio_binding_count {
                let b = self.gpio_bindings[b_idx];
                let inv = if b.signal_idx == I2S0I_V_SYNC_IDX {
                    self.vsync_invert
                } else {
                    b.inverted
                };
                let sel_val = (1u32 << 7) | ((inv as u32) << 6) | ((b.gpio_num as u32) & 0x3F);
                xtensa_mmio_write(
                    GPIO_FUNC0_IN_SEL_CFG_REG + 4 * (b.signal_idx as usize),
                    sel_val,
                );
                b_idx += 1;
            }

            xtensa_mmio_write(I2S0_BASE_ADDR + I2S_RXEOF_NUM_REG_OFFSET, eof_num);
            xtensa_mmio_write(I2S0_BASE_ADDR + I2S_IN_LINK_REG_OFFSET, link_val);
            xtensa_mmio_write(
                I2S0_BASE_ADDR + I2S_CONF_REG_OFFSET,
                conf_base | I2S_RX_START_BIT,
            );
        }

        self.running = true;
        true
    }

    /// Stops I2S0 DMA reception (`ll_cam_stop` in `target/esp32/ll_cam.c:264-270`).
    pub fn stop(&mut self) -> bool {
        let conf_stopped = volatile_read_u32(&self.regs.conf) & !I2S_RX_START_BIT;
        volatile_write_u32(&mut self.regs.conf, conf_stopped);

        let int_ena = volatile_read_u32(&self.regs.int_ena) & !I2S_IN_SUC_EOF_INT_BIT;
        volatile_write_u32(&mut self.regs.int_ena, int_ena);
        volatile_write_u32(&mut self.regs.int_clr, I2S_IN_SUC_EOF_INT_BIT);

        let in_link = (volatile_read_u32(&self.regs.in_link) & !I2S_INLINK_START_BIT)
            | I2S_INLINK_STOP_BIT;
        volatile_write_u32(&mut self.regs.in_link, in_link);

        #[cfg(target_arch = "xtensa")]
        {
            let hw_conf = xtensa_mmio_read(I2S0_BASE_ADDR + I2S_CONF_REG_OFFSET) & !I2S_RX_START_BIT;
            xtensa_mmio_write(I2S0_BASE_ADDR + I2S_CONF_REG_OFFSET, hw_conf);

            let hw_int_ena = xtensa_mmio_read(I2S0_BASE_ADDR + I2S_INT_ENA_REG_OFFSET)
                & !I2S_IN_SUC_EOF_INT_BIT;
            xtensa_mmio_write(I2S0_BASE_ADDR + I2S_INT_ENA_REG_OFFSET, hw_int_ena);
            xtensa_mmio_write(
                I2S0_BASE_ADDR + I2S_INT_CLR_REG_OFFSET,
                I2S_IN_SUC_EOF_INT_BIT,
            );

            let hw_in_link = (xtensa_mmio_read(I2S0_BASE_ADDR + I2S_IN_LINK_REG_OFFSET)
                & !I2S_INLINK_START_BIT)
                | I2S_INLINK_STOP_BIT;
            xtensa_mmio_write(I2S0_BASE_ADDR + I2S_IN_LINK_REG_OFFSET, hw_in_link);
        }

        self.running = false;
        true
    }

    /// Deinitializes the I2S0 camera peripheral and gates its DPORT clock (`ll_cam_deinit`
    /// in `target/esp32/ll_cam.c:272-303`).
    pub fn deinit(&mut self) -> Result<(), CameraError> {
        let _ = self.stop();
        self.vsync_intr_enabled = false;
        self.vsync_pin = None;

        let clk = volatile_read_u32(&self.regs.dport_clk_en) & !DPORT_I2S0_CLK_EN;
        volatile_write_u32(&mut self.regs.dport_clk_en, clk);
        let rst = volatile_read_u32(&self.regs.dport_rst_en) | DPORT_I2S0_RST;
        volatile_write_u32(&mut self.regs.dport_rst_en, rst);

        #[cfg(target_arch = "xtensa")]
        {
            let hw_clk = xtensa_mmio_read(DPORT_PERIP_CLK_EN_REG) & !DPORT_I2S0_CLK_EN;
            xtensa_mmio_write(DPORT_PERIP_CLK_EN_REG, hw_clk);
            let hw_rst = xtensa_mmio_read(DPORT_PERIP_RST_EN_REG) | DPORT_I2S0_RST;
            xtensa_mmio_write(DPORT_PERIP_RST_EN_REG, hw_rst);
        }

        Ok(())
    }

    /// Unpacks a raw DMA half-buffer `src` into `dst` using the active DMA filter (`ll_cam_memcpy`).
    #[inline]
    pub fn memcpy_filtered(&self, dst: &mut [u8], src: &[u8]) -> usize {
        self.filter.apply(dst, src)
    }

    /// Captures a complete JPEG frame (`0xFF, 0xD8, 0xFF` .. `0xFF, 0xD9`) using a circular
    /// [`Lldesc`] DMA ring over `dma_buf`, unpacking `4096`-byte DMA half-buffers into `1024`-byte
    /// JPEG chunks in `jpeg_out` and returning the exact `SOI..=EOI` byte length.
    ///
    /// - On `#[cfg(target_arch = "xtensa")]`, synchronizes with `VSYNC` (`GPIO_IN_REG`), starts
    ///   `I2S0` DMA, polls `I2S_INT_RAW_REG` (`IN_SUC_EOF`), unpacks and re-arms completed
    ///   half-buffers, and unpacks the final partial half-buffer when `VSYNC` asserts at end of frame.
    /// - On `#[cfg(not(target_arch = "xtensa"))]`, builds the circular ring and unpacks the
    ///   pre-populated DMA half-buffers in `dma_buf` for deterministic host unit and Miri verification.
    pub fn capture_jpeg_frame_polled(
        &mut self,
        descriptors: &mut [Lldesc],
        dma_buf: &mut [u8],
        jpeg_out: &mut [u8],
        max_poll_iters: u32,
    ) -> Result<usize, CameraError> {
        let desc_cnt = descriptors.len();
        if desc_cnt < 2
            || !desc_cnt.is_multiple_of(2)
            || jpeg_out.len() < JPEG_UNPACKED_HALF_BUFFER_SIZE
            || max_poll_iters == 0
        {
            return Err(CameraError::InvalidArg);
        }

        let required_dma_bytes = desc_cnt
            .checked_mul(JPEG_DMA_NODE_SIZE)
            .ok_or(CameraError::Overflow)?;
        if dma_buf.len() < required_dma_bytes {
            return Err(CameraError::BufferTooSmall);
        }

        let half_buf_cnt = desc_cnt / 2;
        let dma_cfg = DmaSizeConfig {
            dma_bytes_per_item: self.sampling_mode.bytes_per_sample(),
            dma_buffer_size: required_dma_bytes,
            dma_half_buffer_size: JPEG_DMA_HALF_BUFFER_SIZE,
            dma_half_buffer_cnt: half_buf_cnt,
            dma_node_buffer_size: JPEG_DMA_NODE_SIZE,
            dma_node_cnt: desc_cnt,
            lines_per_node: 1,
            nodes_per_line: 1,
            lines_per_half_buffer: 1,
            image_size: jpeg_out.len(),
        };

        #[cfg(target_arch = "xtensa")]
        {
            dma_buf[..required_dma_bytes].fill(0);
            build_circular_ring(descriptors, &dma_buf[..required_dma_bytes], JPEG_DMA_NODE_SIZE)?;
            let first_desc_addr = descriptors[0].link_addr_20bit();
            let vsync_pin = self.vsync_pin.unwrap_or(25);
            let vsync_sync_level = !self.vsync_invert;

            // Synchronize to VSYNC vertical blanking transition before starting I2S0 DMA
            let mut sync_iters = 0u32;
            while xtensa_read_gpio_level(vsync_pin) == vsync_sync_level
                && sync_iters < max_poll_iters
            {
                sync_iters += 1;
            }
            while xtensa_read_gpio_level(vsync_pin) != vsync_sync_level
                && sync_iters < max_poll_iters
            {
                sync_iters += 1;
            }

            if !self.start(&dma_cfg, first_desc_addr) {
                return Err(CameraError::HardwareError);
            }

            let mut half_buf_idx = 0usize;
            let mut written = 0usize;
            let mut soi_found = false;
            let mut prev_tail = [0u8; 2];
            let mut prev_tail_len = 0usize;
            let mut unpacked_chunk = [0u8; JPEG_UNPACKED_HALF_BUFFER_SIZE];
            let mut seen_active_video = false;
            let mut idle_since_eof = 0u32;
            let mut poll_iters = 0u32;

            while poll_iters < max_poll_iters {
                poll_iters += 1;

                if xtensa_read_gpio_level(vsync_pin) != vsync_sync_level {
                    seen_active_video = true;
                }

                let int_raw = xtensa_mmio_read(I2S0_BASE_ADDR + I2S_INT_RAW_REG_OFFSET);
                if (int_raw & I2S_IN_SUC_EOF_INT_BIT) != 0 {
                    xtensa_mmio_write(
                        I2S0_BASE_ADDR + I2S_INT_CLR_REG_OFFSET,
                        I2S_IN_SUC_EOF_INT_BIT,
                    );
                    idle_since_eof = 0;

                    let eof_desc_addr =
                        xtensa_mmio_read(I2S0_BASE_ADDR + I2S_IN_EOF_DES_ADDR_REG_OFFSET)
                            & I2S_INLINK_ADDR_MASK;
                    let mut target_half = None;
                    let mut h = 0usize;
                    while h < half_buf_cnt {
                        if descriptors[2 * h + 1].link_addr_20bit() == eof_desc_addr
                            || descriptors[2 * h].link_addr_20bit() == eof_desc_addr
                        {
                            target_half = Some(h);
                            break;
                        }
                        h += 1;
                    }

                    let mut drained = 0usize;
                    while drained < half_buf_cnt {
                        let cur_h = half_buf_idx;
                        let half_offset = cur_h * JPEG_DMA_HALF_BUFFER_SIZE;
                        let _ = self.memcpy_filtered(
                            &mut unpacked_chunk,
                            &dma_buf[half_offset..half_offset + JPEG_DMA_HALF_BUFFER_SIZE],
                        );
                        dma_buf[half_offset..half_offset + JPEG_DMA_HALF_BUFFER_SIZE].fill(0);
                        let flags =
                            Lldesc::pack_flags(JPEG_DMA_NODE_SIZE as u16, 0, 0, false, false, true);
                        volatile_write_u32(&mut descriptors[2 * cur_h].size_length_flags, flags);
                        volatile_write_u32(
                            &mut descriptors[2 * cur_h + 1].size_length_flags,
                            flags,
                        );
                        half_buf_idx = (cur_h + 1) % half_buf_cnt;
                        drained += 1;

                        match append_unpacked_jpeg_chunk(
                            &unpacked_chunk,
                            jpeg_out,
                            &mut written,
                            &mut soi_found,
                            &mut prev_tail,
                            &mut prev_tail_len,
                        ) {
                            Ok(Some(total_len)) => {
                                let _ = self.stop();
                                return Ok(total_len);
                            }
                            Ok(None) => {}
                            Err(e) => {
                                let _ = self.stop();
                                return Err(e);
                            }
                        }

                        if target_half == Some(cur_h) || target_half.is_none() {
                            break;
                        }
                    }
                    continue;
                }

                // If SOI has been found, check if the final partial half-buffer (< 4096 DMA bytes)
                // already contains EOI (`0xFF, 0xD9`) on VSYNC end-of-frame or idle window (`cam_hal.c:408-411`).
                if soi_found {
                    idle_since_eof = idle_since_eof.saturating_add(1);
                    let vsync_eof = seen_active_video
                        && xtensa_read_gpio_level(vsync_pin) == vsync_sync_level
                        && xtensa_read_gpio_level(vsync_pin) == vsync_sync_level
                        && xtensa_read_gpio_level(vsync_pin) == vsync_sync_level
                        && xtensa_read_gpio_level(vsync_pin) == vsync_sync_level;

                    if vsync_eof || idle_since_eof >= 2048 {
                        let half_offset = half_buf_idx * JPEG_DMA_HALF_BUFFER_SIZE;
                        let _ = self.memcpy_filtered(
                            &mut unpacked_chunk,
                            &dma_buf[half_offset..half_offset + JPEG_DMA_HALF_BUFFER_SIZE],
                        );
                        let prev_byte = if written > 0 { jpeg_out[written - 1] } else { 0 };
                        let boundary_eoi = prev_byte == 0xFF && unpacked_chunk[0] == 0xD9;
                        if boundary_eoi || find_jpeg_eoi(&unpacked_chunk).is_some() {
                            let _ = self.stop();
                            if let Some(total_len) = append_unpacked_jpeg_chunk(
                                &unpacked_chunk,
                                jpeg_out,
                                &mut written,
                                &mut soi_found,
                                &mut prev_tail,
                                &mut prev_tail_len,
                            )? {
                                return Ok(total_len);
                            }
                        } else if vsync_eof {
                            // Also check the next half-buffer in case one full half-buffer was just written
                            let next_h = (half_buf_idx + 1) % half_buf_cnt;
                            let next_offset = next_h * JPEG_DMA_HALF_BUFFER_SIZE;
                            let mut next_chunk = [0u8; JPEG_UNPACKED_HALF_BUFFER_SIZE];
                            let _ = self.memcpy_filtered(
                                &mut next_chunk,
                                &dma_buf[next_offset..next_offset + JPEG_DMA_HALF_BUFFER_SIZE],
                            );
                            if find_jpeg_eoi(&next_chunk).is_some() {
                                let _ = self.stop();
                                let _ = append_unpacked_jpeg_chunk(
                                    &unpacked_chunk,
                                    jpeg_out,
                                    &mut written,
                                    &mut soi_found,
                                    &mut prev_tail,
                                    &mut prev_tail_len,
                                )?;
                                if let Some(total_len) = append_unpacked_jpeg_chunk(
                                    &next_chunk,
                                    jpeg_out,
                                    &mut written,
                                    &mut soi_found,
                                    &mut prev_tail,
                                    &mut prev_tail_len,
                                )? {
                                    return Ok(total_len);
                                }
                            }
                        }
                        idle_since_eof = 0;
                    }
                }
            }

            let _ = self.stop();
            Err(CameraError::Timeout)
        }

        #[cfg(not(target_arch = "xtensa"))]
        {
            build_circular_ring(descriptors, &dma_buf[..required_dma_bytes], JPEG_DMA_NODE_SIZE)?;
            let first_desc_addr = descriptors[0].link_addr_20bit();
            if !self.start(&dma_cfg, first_desc_addr) {
                return Err(CameraError::HardwareError);
            }

            let mut written = 0usize;
            let mut soi_found = false;
            let mut prev_tail = [0u8; 2];
            let mut prev_tail_len = 0usize;
            let mut unpacked_chunk = [0u8; JPEG_UNPACKED_HALF_BUFFER_SIZE];
            let max_halves = (max_poll_iters as usize).min(half_buf_cnt);

            let mut h = 0usize;
            while h < max_halves {
                let half_offset = h * JPEG_DMA_HALF_BUFFER_SIZE;
                let _ = self.memcpy_filtered(
                    &mut unpacked_chunk,
                    &dma_buf[half_offset..half_offset + JPEG_DMA_HALF_BUFFER_SIZE],
                );
                dma_buf[half_offset..half_offset + JPEG_DMA_HALF_BUFFER_SIZE].fill(0);
                let flags = Lldesc::pack_flags(JPEG_DMA_NODE_SIZE as u16, 0, 0, false, false, true);
                volatile_write_u32(&mut descriptors[2 * h].size_length_flags, flags);
                volatile_write_u32(&mut descriptors[2 * h + 1].size_length_flags, flags);

                match append_unpacked_jpeg_chunk(
                    &unpacked_chunk,
                    jpeg_out,
                    &mut written,
                    &mut soi_found,
                    &mut prev_tail,
                    &mut prev_tail_len,
                ) {
                    Ok(Some(total_len)) => {
                        let _ = self.stop();
                        return Ok(total_len);
                    }
                    Ok(None) => {}
                    Err(e) => {
                        let _ = self.stop();
                        return Err(e);
                    }
                }
                h += 1;
            }

            let _ = self.stop();
            if max_halves < half_buf_cnt {
                Err(CameraError::Timeout)
            } else {
                Err(CameraError::CorruptedFrame)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dma_filter::DmaElement;
    use crate::dma_ring::{calc_dma_sizes, DEFAULT_CAMERA_DMA_BUFFER_SIZE_MAX};
    use esp32_cam_types::CameraPid;

    #[test]
    fn test_io_mux_reg_addr_lookup_wrover_and_aithinker_pins() {
        // WROVER-KIT camera pins
        assert_eq!(io_mux_reg_addr(21), Some(0x3FF4_907C)); // XCLK
        assert_eq!(io_mux_reg_addr(22), Some(0x3FF4_9080)); // PCLK
        assert_eq!(io_mux_reg_addr(23), Some(0x3FF4_908C)); // HREF
        assert_eq!(io_mux_reg_addr(25), Some(0x3FF4_9024)); // VSYNC
        assert_eq!(io_mux_reg_addr(26), Some(0x3FF4_9028)); // SIOD
        assert_eq!(io_mux_reg_addr(27), Some(0x3FF4_902C)); // SIOC
        assert_eq!(io_mux_reg_addr(4), Some(0x3FF4_9048));  // D0
        assert_eq!(io_mux_reg_addr(5), Some(0x3FF4_906C));  // D1
        assert_eq!(io_mux_reg_addr(18), Some(0x3FF4_9070)); // D2
        assert_eq!(io_mux_reg_addr(19), Some(0x3FF4_9074)); // D3
        assert_eq!(io_mux_reg_addr(36), Some(0x3FF4_9004)); // D4
        assert_eq!(io_mux_reg_addr(39), Some(0x3FF4_9010)); // D5
        assert_eq!(io_mux_reg_addr(34), Some(0x3FF4_9014)); // D6
        assert_eq!(io_mux_reg_addr(35), Some(0x3FF4_9018)); // D7

        // Unbonded / out-of-range pins
        assert_eq!(io_mux_reg_addr(24), None);
        assert_eq!(io_mux_reg_addr(28), None);
        assert_eq!(io_mux_reg_addr(40), None);
    }

    #[test]
    fn test_esp32_i2s_pin_validation_and_matrix_routing() {
        let mut cam = Esp32I2sCamera::new();
        let valid_pins = CamPinConfig {
            pin_vsync: 25,
            pin_href: 23,
            pin_pclk: 22,
            pin_xclk: 21,
            data_pins: [4, 5, 18, 19, 36, 39, 34, 35],
            vsync_invert: true,
        };
        assert_eq!(cam.set_pin(&valid_pins), Ok(()));
        assert_eq!(cam.gpio_bindings().len(), 12);
        assert_eq!(I2S0I_WS_IN_IDX, 28);
        assert_eq!(cam.gpio_bindings()[0].signal_idx, I2S0I_WS_IN_IDX);
        assert_eq!(cam.gpio_bindings()[0].signal_idx, 28);
        assert_eq!(cam.gpio_bindings()[1].signal_idx, I2S0I_V_SYNC_IDX);
        assert_eq!(cam.gpio_bindings()[2].signal_idx, I2S0I_H_SYNC_IDX);
        assert_eq!(cam.gpio_bindings()[3].signal_idx, I2S0I_DATA_IN0_IDX);
        assert_eq!(cam.gpio_bindings()[10].signal_idx, I2S0I_DATA_IN7_IDX);
        assert_eq!(
            cam.gpio_bindings()[11].gpio_num,
            GPIO_MATRIX_CONST_ONE_INPUT
        );
        assert_eq!(cam.gpio_bindings()[11].signal_idx, I2S0I_H_ENABLE_IDX);

        // Out-of-range GPIO >= 40 rejected
        let mut bad_pins = valid_pins;
        bad_pins.pin_vsync = 40;
        assert_eq!(cam.set_pin(&bad_pins), Err(CameraError::InvalidArg));

        let mut bad_data = valid_pins;
        bad_data.data_pins[7] = -1;
        assert_eq!(cam.set_pin(&bad_data), Err(CameraError::InvalidArg));
    }

    #[test]
    fn test_esp32_i2s_state_transitions() {
        let mut cam = Esp32I2sCamera::new();
        assert_eq!(cam.config(), Ok(()));
        assert_eq!(cam.regs().dport_clk_en & DPORT_I2S0_CLK_EN, DPORT_I2S0_CLK_EN);
        assert_eq!(cam.regs().dport_rst_en & DPORT_I2S0_RST, 0);
        assert_eq!(cam.regs().pd_conf, I2S_PD_FIFO_MEM_FORCE_PU);
        assert_eq!(I2S_PD_CONF_REG_OFFSET, 0xA4);
        assert_eq!(I2S_PD_FIFO_MEM_FORCE_PU, 0x0A);
        assert_eq!(cam.regs().conf2, I2S_LCD_EN_BIT | I2S_CAMERA_EN_BIT);

        let mode = cam
            .set_sample_mode(PixelFormat::Jpeg, 20_000_000, CameraPid::OV2640.as_u16())
            .expect("jpeg sample mode");
        assert_eq!(mode.sampling_mode, I2sSamplingMode::Sm0A000B00);

        let dma_cfg = calc_dma_sizes(
            true,
            mode.sampling_mode,
            640,
            480,
            mode.in_bytes_per_pixel,
            DEFAULT_CAMERA_DMA_BUFFER_SIZE_MAX,
        )
        .expect("dma sizes");

        cam.vsync_intr_enable(true);
        assert!(cam.is_vsync_intr_enabled());

        assert!(cam.start(&dma_cfg, 0x3FFB_1234));
        assert!(cam.is_running());
        assert_eq!(cam.regs().rx_eof_num, 1024); // 4096 / 4
        assert_eq!(cam.regs().in_link & I2S_INLINK_ADDR_MASK, 0xFB_1234 & 0xF_FFFF);
        assert_ne!(cam.regs().in_link & I2S_INLINK_START_BIT, 0);
        assert_ne!(cam.regs().conf & I2S_RX_START_BIT, 0);

        assert!(cam.stop());
        assert!(!cam.is_running());
        assert_eq!(cam.regs().conf & I2S_RX_START_BIT, 0);
        assert_ne!(cam.regs().in_link & I2S_INLINK_STOP_BIT, 0);

        assert_eq!(cam.deinit(), Ok(()));
        assert_eq!(cam.regs().dport_clk_en & DPORT_I2S0_CLK_EN, 0);
        assert_eq!(cam.regs().dport_rst_en & DPORT_I2S0_RST, DPORT_I2S0_RST);
    }

    #[test]
    fn test_capture_jpeg_frame_polled_unpacking_and_trimming() {
        let mut cam = Esp32I2sCamera::new();
        assert_eq!(cam.config(), Ok(()));
        let _ = cam
            .set_sample_mode(PixelFormat::Jpeg, 20_000_000, CameraPid::OV2640.as_u16())
            .expect("set jpeg sample mode");

        let mut descriptors = [Lldesc::empty(); 4];
        let mut dma_buf = [0u8; 4 * JPEG_DMA_NODE_SIZE];
        let mut jpeg_out = [0u8; 2048];

        // Embed 4 leading padding bytes, followed by a 1030-byte JPEG frame spanning across
        // half-buffer 0 (1024 unpacked bytes) and half-buffer 1 (partial unpacked bytes),
        // followed by trailing padding bytes.
        let mut synthetic_stream = [0x55u8; 1100];
        synthetic_stream[0] = 0x00;
        synthetic_stream[1] = 0x00;
        synthetic_stream[2] = 0x00;
        synthetic_stream[3] = 0x00;
        // JPEG SOI at offset 4
        synthetic_stream[4] = 0xFF;
        synthetic_stream[5] = 0xD8;
        synthetic_stream[6] = 0xFF;
        synthetic_stream[7] = 0xE0;
        // JPEG EOI at offset 4 + 1028 = 1032..=1033 (total JPEG length = 1030 bytes)
        synthetic_stream[1032] = 0xFF;
        synthetic_stream[1033] = 0xD9;

        for (i, &b) in synthetic_stream.iter().enumerate() {
            let el = DmaElement::new(b, 0);
            dma_buf[i * 4..i * 4 + 4].copy_from_slice(&el.0);
        }

        let captured_len = cam
            .capture_jpeg_frame_polled(&mut descriptors, &mut dma_buf, &mut jpeg_out, 16)
            .expect("polled JPEG capture should succeed");
        assert_eq!(captured_len, 1030);
        assert_eq!(&jpeg_out[..3], &[0xFF, 0xD8, 0xFF]);
        assert_eq!(&jpeg_out[captured_len - 2..captured_len], &[0xFF, 0xD9]);
        assert!(!cam.is_running());
    }
}
