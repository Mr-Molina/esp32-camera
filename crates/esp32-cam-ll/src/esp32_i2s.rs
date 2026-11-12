//! ESP32 I2S0 camera parallel interface low-level driver
//! (`target/esp32/ll_cam.c`).
//!
//! Provides MMIO register constants, `IO_MUX` pad address lookup, GPIO matrix
//! signal indices,
//! and the [`Esp32I2sCamera`] controller implementing `config`, `set_pin`,
//! `start`, `stop`,
//! `deinit`, `vsync_intr_enable`, `set_sample_mode`, and
//! `capture_jpeg_frame_polled`.

use core::marker::PhantomData;
use core::ops::ControlFlow;
use core::ptr;
use core::sync::atomic::{compiler_fence, AtomicBool, Ordering};
use esp32_cam_types::{CameraError, PixelFormat};

use crate::dma_drain::{
    CaptureHooks, CaptureStep, DmaStatus, HalfDrain, NoCaptureHooks, PolledJpegCapture,
};
use crate::dma_filter::{select_sample_mode, DmaFilterKind, I2sSamplingMode, SampleModeConfig};
use crate::dma_ring::{DmaRing, DmaSizeConfig, Lldesc, RingParts};

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

/// Physical address of `GPIO_ENABLE_W1TC_REG` (`0x3FF4_4028`, clears output
/// enable for GPIO `0..=31`).
pub const GPIO_ENABLE_W1TC_REG: usize = 0x3FF4_4028;

/// Physical address of `GPIO_ENABLE1_W1TC_REG` (`0x3FF4_4034`, clears output
/// enable for GPIO `32..=39`).
pub const GPIO_ENABLE1_W1TC_REG: usize = 0x3FF4_4034;

/// Physical address of `GPIO_IN_REG` (`0x3FF4_403C`, reads input level for GPIO
/// `0..=31`).
pub const GPIO_IN_REG: usize = 0x3FF4_403C;

/// Physical address of `GPIO_IN1_REG` (`0x3FF4_4040`, reads input level for
/// GPIO `32..=39`).
pub const GPIO_IN1_REG: usize = 0x3FF4_4040;

/// Physical address of `GPIO_PIN0_REG` (`0x3FF4_4088`, stride `4` bytes per
/// GPIO `0..=39`).
pub const GPIO_PIN0_REG: usize = 0x3FF4_4088;

/// Physical address of `GPIO_FUNC0_IN_SEL_CFG_REG` (`0x3FF4_4130`, stride `4`
/// bytes per signal index `0..=255`).
pub const GPIO_FUNC0_IN_SEL_CFG_REG: usize = 0x3FF4_4130;

/// Physical address of `GPIO_FUNC0_OUT_SEL_CFG_REG` (`0x3FF4_4530`, stride `4`
/// bytes per GPIO `0..=33`).
pub const GPIO_FUNC0_OUT_SEL_CFG_REG: usize = 0x3FF4_4530;

/// Physical base address of the `RTC_IO` peripheral on ESP32 (`0x3FF4_8400`).
pub const RTC_IO_BASE_ADDR: usize = 0x3FF4_8400;

/// Physical address of `RTC_IO_SENSOR_PADS_REG` (`0x3FF4_847C`, controls
/// digital/RTC mux for GPIO `36..=39`).
pub const RTC_IO_SENSOR_PADS_REG: usize = 0x3FF4_847C;

/// Physical address of `RTC_IO_ADC_PAD_REG` (`0x3FF4_8480`, controls
/// digital/RTC mux for GPIO `34..=35`).
pub const RTC_IO_ADC_PAD_REG: usize = 0x3FF4_8480;

/// Physical address of `RTC_IO_PAD_DAC1_REG` (`0x3FF4_8484`, controls
/// digital/RTC mux and pull-up for GPIO `25`).
pub const RTC_IO_PAD_DAC1_REG: usize = 0x3FF4_8484;

/// Physical address of `RTC_IO_PAD_DAC2_REG` (`0x3FF4_8488`, controls
/// digital/RTC mux for GPIO `26`).
pub const RTC_IO_PAD_DAC2_REG: usize = 0x3FF4_8488;

/// Physical address of `RTC_IO_TOUCH_PAD0_REG` (`0x3FF4_8494`, controls
/// digital/RTC mux for GPIO `4`).
pub const RTC_IO_TOUCH_PAD0_REG: usize = 0x3FF4_8494;

/// Physical base address of the `IO_MUX` peripheral on ESP32 (`0x3FF4_9000`).
pub const IO_MUX_BASE_ADDR: usize = 0x3FF4_9000;

/// Physical base address of the `I2S0` peripheral on ESP32 (`0x3FF4_F000`).
pub const I2S0_BASE_ADDR: usize = 0x3FF4_F000;

/// Maximum valid GPIO number on ESP32 (`GPIO_NUM_MAX = 40`, valid indices
/// `0..=39`).
pub const ESP32_GPIO_NUM_MAX: u8 = 40;

/// GPIO matrix input signal index for `I2S0I_DATA_IN0_IDX` (`140`, `D0..D7` =
/// `140..=147`).
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

/// GPIO matrix input signal index for `I2S0I_WS_IN_IDX` (`PCLK`, `28`, ESP32
/// TRM Table 4-2).
pub const I2S0I_WS_IN_IDX: u8 = 28;

/// GPIO matrix input signal index for `I2S0I_H_SYNC_IDX` (`HREF`, `190`).
pub const I2S0I_H_SYNC_IDX: u8 = 190;

/// GPIO matrix input signal index for `I2S0I_V_SYNC_IDX` (`VSYNC`, `191`).
pub const I2S0I_V_SYNC_IDX: u8 = 191;

/// GPIO matrix input signal index for `I2S0I_H_ENABLE_IDX` (`192`).
pub const I2S0I_H_ENABLE_IDX: u8 = 192;

/// Constant logic-high (`1`) virtual GPIO matrix source (`0x38` in
/// `ll_cam.c:452`).
pub const GPIO_MATRIX_CONST_ONE_INPUT: u8 = 0x38;

/// `I2S0.conf` register offset (`0x08`) and bit definitions.
pub const I2S_CONF_REG_OFFSET: usize = 0x08;
pub const I2S_RX_RESET_BIT: u32 = 1 << 1;
pub const I2S_RX_FIFO_RESET_BIT: u32 = 1 << 3;
pub const I2S_RX_START_BIT: u32 = 1 << 5;
pub const I2S_RX_SLAVE_MOD_BIT: u32 = 1 << 7;

/// `I2S0.int_raw` (`0x0C`), `I2S0.int_st` (`0x10`), `I2S0.int_ena` (`0x14`),
/// and `I2S0.int_clr` (`0x18`).
pub const I2S_INT_RAW_REG_OFFSET: usize = 0x0C;
pub const I2S_INT_ST_REG_OFFSET: usize = 0x10;
pub const I2S_INT_ENA_REG_OFFSET: usize = 0x14;
pub const I2S_INT_CLR_REG_OFFSET: usize = 0x18;
pub const I2S_IN_SUC_EOF_INT_BIT: u32 = 1 << 9;
/// Raw interrupt flag for RX descriptor error (`BIT(13)` in `I2S_INT_RAW_REG`).
pub const I2S_IN_DSCR_ERR_INT_RAW: u32 = 1 << 13;
/// Raw interrupt flag for RX descriptor empty (`BIT(15)` in `I2S_INT_RAW_REG`).
pub const I2S_IN_DSCR_EMPTY_INT_RAW: u32 = 1 << 15;
/// Inlink status latches cleared by [`Esp32I2sCamera::start`]: `IN_SUC_EOF`,
/// `IN_DSCR_ERR` and `IN_DSCR_EMPTY` (`I2S_INT_CLR_REG` uses the raw bit
/// positions).
const INLINK_STATUS_LATCHES: u32 =
    I2S_IN_SUC_EOF_INT_BIT | I2S_IN_DSCR_ERR_INT_RAW | I2S_IN_DSCR_EMPTY_INT_RAW;

/// `I2S0.timing` (`0x1C`), `I2S0.fifo_conf` (`0x20`), `I2S0.rx_eof_num`
/// (`0x24`), and `I2S0.conf_chan` (`0x2C`).
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

/// `I2S0.pd_conf` (`0xA4`), `I2S0.conf2` (`0xA8`), `I2S0.clkm_conf` (`0xAC`),
/// and `I2S0.sample_rate_conf` (`0xB0`) offsets.
pub const I2S_PD_CONF_REG_OFFSET: usize = 0xA4;
pub const I2S_PD_FIFO_MEM_FORCE_PU: u32 = 0x0A;
pub const I2S_CONF2_REG_OFFSET: usize = 0xA8;
pub const I2S_CLKM_CONF_REG_OFFSET: usize = 0xAC;
pub const I2S_SAMPLE_RATE_CONF_REG_OFFSET: usize = 0xB0;
pub const I2S_CAMERA_EN_BIT: u32 = 1 << 0;
pub const I2S_LCD_EN_BIT: u32 = 1 << 5;

/// Standard JPEG DMA descriptor node buffer size (`2048` bytes).
pub const JPEG_DMA_NODE_SIZE: usize = 2048;

/// Standard JPEG DMA half-buffer size (`4096` bytes = `2` nodes of `2048`
/// bytes).
pub const JPEG_DMA_HALF_BUFFER_SIZE: usize = 4096;

/// Unpacked JPEG byte count per `4096`-byte DMA half-buffer (`1024` bytes, 1
/// byte per 4-byte `DmaElement`).
pub const JPEG_UNPACKED_HALF_BUFFER_SIZE: usize = 1024;

/// Maps an ESP32 GPIO number (`0..=39`) to its physical `IO_MUX` pad register
/// address
/// (`0x3FF4_9004..=0x3FF4_908C`, ESP32 Technical Reference Manual Table 4-3).
///
/// Returns `None` for unbonded/reserved GPIO indices (`20`, `24`, `28..=31`, or
/// `>= 40`).
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

/// Pin routing configuration passed to `set_pin` (`ll_cam_set_pin` in
/// `target/esp32/ll_cam.c:387-454`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct CamPinConfig {
    pub pin_vsync: i8,
    pub pin_href: i8,
    pub pin_pclk: i8,
    pub pin_xclk: i8,
    pub data_pins: [i8; 8],
    pub vsync_invert: bool,
}

/// Recorded GPIO matrix input routing entry (`gpio_matrix_in(pin, signal_idx,
/// inv)`).
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
    /// Shadow of `I2S_IN_EOF_DES_ADDR_REG` (address of the last EOF descriptor).
    pub in_eof_des_addr: u32,
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
    // JUSTIFICATION: Direct register write requires unsafe raw pointer dereference.
    // SAFETY:
    // 1. Alignment & Non-null: `ptr` is obtained via `ptr::from_mut` from a valid
    //    `&mut u32` reference,
    //    guaranteeing non-null and 4-byte alignment.
    // 2. Memory layout: `u32` is a 4-byte primitive register word inside `#[repr(C,
    //    align(4))]`.
    // 3. Provenance & Lifetime: `ptr` has valid mutable provenance for the duration
    //    of this call.
    // 4. Exclusive aliasing: `reg: &mut u32` holds unique access to the target
    //    word.
    // 5. Volatile semantics: `write_volatile` ensures the compiler does not elide
    //    or reorder register updates.
    // 6. Panic safety: `write_volatile` on a stack/struct `u32` cannot unwind.
    unsafe {
        ptr::write_volatile(ptr, val);
    }
}

#[inline]
fn volatile_read_u32(reg: &u32) -> u32 {
    let ptr = ptr::from_ref(reg);
    // JUSTIFICATION: Direct register read requires unsafe raw pointer dereference.
    // SAFETY:
    // 1. Alignment & Non-null: `ptr` is obtained via `ptr::from_ref` from a valid
    //    `&u32` reference,
    //    guaranteeing non-null and 4-byte alignment.
    // 2. Memory layout: `u32` is a 4-byte primitive register word inside `#[repr(C,
    //    align(4))]`.
    // 3. Provenance & Lifetime: `ptr` has valid shared provenance for the duration
    //    of this call.
    // 4. Aliasing: No concurrent mutable reference exists during this read.
    // 5. Volatile semantics: `read_volatile` prevents compiler read elision.
    // 6. Panic safety: `read_volatile` on a valid `u32` cannot unwind.
    unsafe { ptr::read_volatile(ptr) }
}

/// Writes `val` to the 32-bit MMIO register at `addr` (LL-20: `unsafe`).
///
/// # Safety
///
/// `addr` must be a 4-byte aligned ESP32 peripheral register in
/// `0x3FF0_0000..0x3FF8_0000`, the caller must own that peripheral (or be in
/// an emergency teardown that preempts its owner), and the write must not
/// make the hardware access memory the caller does not own (for example a
/// DMA start at an unchecked descriptor address).
#[cfg(target_arch = "xtensa")]
#[inline]
unsafe fn xtensa_mmio_write(addr: usize, val: u32) {
    // JUSTIFICATION: ESP32 MMIO register write requires unsafe raw pointer
    // dereference.
    // SAFETY:
    // 1. Alignment & Non-null: The caller passes a 4-byte aligned ESP32 MMIO
    //    register address in `0x3FF0_0000..0x3FF8_0000`.
    // 2. Memory layout: ESP32 DPORT, GPIO, RTC_IO, IO_MUX, and I2S0 peripheral
    //    registers are 32-bit memory-mapped words.
    // 3. Provenance: Hardware peripheral address space on Xtensa ESP32 target.
    // 4. Aliasing: The caller owns the peripheral (function contract).
    // 5. Volatile semantics: `write_volatile` is required for hardware MMIO side
    //    effects.
    // 6. Panic safety: Hardware MMIO store does not unwind.
    unsafe {
        ptr::write_volatile(addr as *mut u32, val);
    }
}

/// Reads the 32-bit MMIO register at `addr` (LL-20: `unsafe`).
///
/// # Safety
///
/// `addr` must be a 4-byte aligned ESP32 peripheral register in
/// `0x3FF0_0000..0x3FF8_0000` whose read has no side effect the caller does
/// not own.
#[cfg(target_arch = "xtensa")]
#[inline]
unsafe fn xtensa_mmio_read(addr: usize) -> u32 {
    // JUSTIFICATION: ESP32 MMIO register read requires unsafe raw pointer
    // dereference.
    // SAFETY:
    // 1. Alignment & Non-null: The caller passes a 4-byte aligned ESP32 MMIO
    //    register address in `0x3FF0_0000..0x3FF8_0000`.
    // 2. Memory layout: ESP32 DPORT, GPIO, RTC_IO, IO_MUX, and I2S0 peripheral
    //    registers are 32-bit memory-mapped words.
    // 3. Provenance: Hardware peripheral address space on Xtensa ESP32 target.
    // 4. Aliasing: Volatile hardware register read does not alias Rust stack or
    //    heap references.
    // 5. Volatile semantics: `read_volatile` prevents compiler elision or
    //    reordering of hardware status reads.
    // 6. Panic safety: Hardware MMIO load does not unwind.
    unsafe { ptr::read_volatile(addr as *const u32) }
}

/// Orders CPU memory accesses against the DMA engine (LL-03): a compiler
/// fence with `ordering` plus, on Xtensa, a `memw` that drains the
/// load/store and write buffers to internal SRAM.
///
/// Use `Release` before handing memory to the DMA (descriptor writes before
/// the `in_link` start store) and `Acquire` before reading what the DMA
/// wrote.
#[inline(always)]
pub(crate) fn dma_fence(ordering: Ordering) {
    compiler_fence(ordering);
    #[cfg(target_arch = "xtensa")]
    {
        // JUSTIFICATION: Xtensa memory barrier between CPU and DMA accesses.
        // SAFETY:
        // 1. Alignment & Non-null: `memw` takes no memory operand.
        // 2. Memory layout: Architectural Xtensa memory barrier instruction.
        // 3. Provenance: Valid instruction on the ESP32 Xtensa LX6 core.
        // 4. Aliasing: Touches no Rust-visible memory; only drains the
        //    processor's load/store and write buffers.
        // 5. Volatile semantics: The `asm!` block is opaque to the compiler
        //    (no `nomem`), so it is also a full compiler barrier.
        // 6. Panic safety: `memw` cannot trap or unwind.
        unsafe {
            core::arch::asm!("memw", options(nostack, preserves_flags));
        }
    }
}

/// Routes `gpio_num` to the digital `IO_MUX` as a GPIO-matrix input.
///
/// # Safety
///
/// `gpio_num` must be a bonded ESP32 GPIO (`io_mux_reg_addr(gpio_num)` is
/// `Some`, so `gpio_num < 40`) that the caller owns for the camera.
#[cfg(target_arch = "xtensa")]
#[inline]
unsafe fn xtensa_configure_input_pad(gpio_num: u8, enable_pullup: bool) {
    // JUSTIFICATION: IO_MUX, RTC_IO and GPIO matrix MMIO for one camera pad.
    // SAFETY:
    // 1. Alignment & Non-null: Every address is a fixed, 4-byte aligned
    //    RTC_IO / IO_MUX / GPIO register; the per-pin offsets are bounded by
    //    `gpio_num < 40` (function contract).
    // 2. Memory layout: 32-bit peripheral registers.
    // 3. Provenance: ESP32 peripheral address space.
    // 4. Aliasing: The caller owns the pad (function contract) and these are
    //    read-modify-writes of that pad's bits only.
    // 5. Volatile semantics: All accesses are volatile MMIO.
    // 6. Panic safety: MMIO accesses cannot unwind.
    unsafe {
        // Route RTC_IO-capable pads to the digital IO_MUX before configuring
        // IO_MUX and the GPIO matrix.
        match gpio_num {
            36..=39 => {
                let bit = 27 - (gpio_num - 36);
                let v = xtensa_mmio_read(RTC_IO_SENSOR_PADS_REG) & !(1u32 << bit);
                xtensa_mmio_write(RTC_IO_SENSOR_PADS_REG, v);
            }
            34 | 35 => {
                let bit = if gpio_num == 34 { 29 } else { 28 };
                let v = xtensa_mmio_read(RTC_IO_ADC_PAD_REG) & !(1u32 << bit);
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

        // Configure IO_MUX: MCU_SEL (bits 12..=14), FUN_DRV (bits 10..=11),
        // FUN_IE (bit 9), FUN_WPU (bit 8), FUN_WPD (bit 7).
        // - Input-only pads (GPIO >= 34): 0x0200 = MCU_SEL 0 (function 1),
        //   FUN_IE 1, no pulls (these pads have none).
        // - Other pads: 0x2A00 = MCU_SEL 2 (GPIO), FUN_DRV 2, FUN_IE 1, no
        //   pull; 0x2B00 additionally sets FUN_WPU (pull-up). FUN_WPD is
        //   never set.
        if let Some(mux_addr) = io_mux_reg_addr(gpio_num) {
            let mux_val = if gpio_num >= 34 {
                0x0200
            } else if enable_pullup {
                0x2B00
            } else {
                0x2A00
            };
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

        // Disconnect output matrix for output-capable pads (0..=33) using
        // SIG_GPIO_OUT_IDX (0x100)
        if gpio_num < 34 {
            xtensa_mmio_write(GPIO_FUNC0_OUT_SEL_CFG_REG + 4 * (gpio_num as usize), 0x100);
        }
    }
}

/// Reads the input level of `gpio_num` (`0..=39`) from `GPIO_IN_REG` /
/// `GPIO_IN1_REG`.
#[cfg(target_arch = "xtensa")]
#[inline]
fn xtensa_read_gpio_level(gpio_num: u8) -> bool {
    // JUSTIFICATION: Read-only sample of the GPIO input level registers.
    // SAFETY:
    // 1. Alignment & Non-null: `GPIO_IN_REG` / `GPIO_IN1_REG` are fixed,
    //    4-byte aligned GPIO registers.
    // 2. Memory layout: 32-bit peripheral registers.
    // 3. Provenance: ESP32 GPIO peripheral address space.
    // 4. Aliasing: A read of the input latches has no side effect and does
    //    not conflict with any owner of the GPIO block.
    // 5. Volatile semantics: Volatile MMIO read, re-sampled on every call.
    // 6. Panic safety: MMIO reads cannot unwind; shifts are bounded by
    //    `gpio_num & 31`.
    unsafe {
        if gpio_num < 32 {
            ((xtensa_mmio_read(GPIO_IN_REG) >> gpio_num) & 1) != 0
        } else {
            ((xtensa_mmio_read(GPIO_IN1_REG) >> ((gpio_num - 32) & 31)) & 1) != 0
        }
    }
}

/// [`DmaStatus`] over the I2S0 inlink status registers, borrowed from an
/// [`Esp32I2sCamera`] with [`Esp32I2sCamera::dma_status`].
///
/// On Xtensa it reads and clears the hardware `I2S_INT_RAW_REG`,
/// `I2S_INT_CLR_REG` and `I2S_IN_EOF_DES_ADDR_REG`; on the host it operates
/// on the shadow [`I2s0RegisterBlock`]. Clears are mirrored into the shadow
/// block on both targets.
#[derive(Debug)]
pub struct I2s0DmaStatus<'a> {
    regs: &'a mut I2s0RegisterBlock,
}

/// SAFETY comment shared by the read-only I2S0 status register reads below.
#[cfg(target_arch = "xtensa")]
macro_rules! i2s0_status_read {
    ($off:expr) => {{
        // JUSTIFICATION: Read of an I2S0 status register.
        // SAFETY:
        // 1. Alignment & Non-null: `I2S0_BASE_ADDR + off` is a fixed, 4-byte
        //    aligned I2S0 register.
        // 2. Memory layout: 32-bit peripheral register.
        // 3. Provenance: ESP32 I2S0 peripheral address space.
        // 4. Aliasing: `I2s0DmaStatus` mutably borrows the owning
        //    `Esp32I2sCamera`, so it is the only I2S0 accessor.
        // 5. Volatile semantics: Volatile MMIO read.
        // 6. Panic safety: MMIO reads cannot unwind.
        unsafe { xtensa_mmio_read(I2S0_BASE_ADDR + $off) }
    }};
}

impl DmaStatus for I2s0DmaStatus<'_> {
    #[inline]
    fn int_raw(&mut self) -> u32 {
        #[cfg(target_arch = "xtensa")]
        {
            i2s0_status_read!(I2S_INT_RAW_REG_OFFSET)
        }
        #[cfg(not(target_arch = "xtensa"))]
        {
            volatile_read_u32(&self.regs.int_raw)
        }
    }

    #[inline]
    fn clear_int(&mut self, mask: u32) {
        #[cfg(target_arch = "xtensa")]
        // JUSTIFICATION: Write-1-to-clear of I2S0 inlink status latches.
        // SAFETY:
        // 1. Alignment & Non-null: `I2S_INT_CLR_REG` is a fixed, 4-byte
        //    aligned I2S0 register.
        // 2. Memory layout: 32-bit peripheral register.
        // 3. Provenance: ESP32 I2S0 peripheral address space.
        // 4. Aliasing: `I2s0DmaStatus` mutably borrows the owning
        //    `Esp32I2sCamera`; clearing status latches starts no DMA.
        // 5. Volatile semantics: Volatile MMIO write.
        // 6. Panic safety: MMIO writes cannot unwind.
        unsafe {
            xtensa_mmio_write(I2S0_BASE_ADDR + I2S_INT_CLR_REG_OFFSET, mask);
        }
        volatile_write_u32(&mut self.regs.int_clr, mask);
        let raw = volatile_read_u32(&self.regs.int_raw) & !mask;
        volatile_write_u32(&mut self.regs.int_raw, raw);
    }

    #[inline]
    fn eof_desc_addr(&mut self) -> u32 {
        #[cfg(target_arch = "xtensa")]
        {
            i2s0_status_read!(I2S_IN_EOF_DES_ADDR_REG_OFFSET) & I2S_INLINK_ADDR_MASK
        }
        #[cfg(not(target_arch = "xtensa"))]
        {
            volatile_read_u32(&self.regs.in_eof_des_addr) & I2S_INLINK_ADDR_MASK
        }
    }

    /// `memw` plus a `SeqCst` compiler fence (LL-03): orders the status
    /// reads before the SRAM reads of a half (Acquire side) and the
    /// poisoning before the descriptor re-arm (Release side).
    #[inline]
    fn barrier(&mut self) {
        dma_fence(Ordering::SeqCst);
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
pub(crate) fn find_jpeg_eoi(buf: &[u8]) -> Option<usize> {
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

/// Appends an unpacked `1024`-byte DMA chunk into `jpeg_out`, handling `SOI`
/// (`0xFF, 0xD8, 0xFF`)
/// alignment (including cross-chunk straddling) and returning
/// `Ok(Some(total_len))` as soon as
/// `EOI` (`0xFF, 0xD9`) is encountered after `SOI`.
pub(crate) fn append_unpacked_jpeg_chunk(
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

        if *written >= 4 {
            if let Some(rel_eoi) = find_jpeg_eoi(&jpeg_out[2..*written]) {
                return Ok(Some(2 + rel_eoi + 2));
            }
        }
        return Ok(None);
    }

    if *written + n > jpeg_out.len() {
        return Err(CameraError::BufferTooSmall);
    }
    let search_start = (*written).saturating_sub(1).max(2);
    jpeg_out[*written..*written + n].copy_from_slice(chunk);
    *written += n;

    if *written >= search_start + 2 {
        if let Some(rel_eoi) = find_jpeg_eoi(&jpeg_out[search_start..*written]) {
            return Ok(Some(search_start + rel_eoi + 2));
        }
    }

    Ok(None)
}

/// Claim flag for the I2S0 peripheral (LL-13): set by
/// [`Esp32I2sCamera::take`], cleared when that owner drops.
static I2S0_TAKEN: AtomicBool = AtomicBool::new(false);

/// Low-level ESP32 I2S0 camera peripheral controller (`target/esp32/ll_cam.c`).
///
/// LL-13: I2S0 has exactly one owner. The type is neither `Clone` nor (on
/// Xtensa) `Default`; [`Self::take`] hands out the single instance and its
/// `Drop` stops the DMA and releases the claim. On the host, where every
/// register access goes to a private shadow block and no DMA engine exists,
/// [`Self::new`] creates independent simulators for tests.
#[derive(Debug)]
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
    /// `true` for the instance from [`Self::take`], which owns `I2S0_TAKEN`.
    claimed: bool,
    /// Inlink/AHB DMA reset pulses issued by [`Self::stop`] (host tests).
    #[cfg(test)]
    dma_resets: u32,
}

#[cfg(not(target_arch = "xtensa"))]
impl Default for Esp32I2sCamera {
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for Esp32I2sCamera {
    fn drop(&mut self) {
        if self.running {
            let _ = self.stop();
        }
        if self.claimed {
            I2S0_TAKEN.store(false, Ordering::Release);
        }
    }
}

impl Esp32I2sCamera {
    /// Claims the I2S0 peripheral and returns its single controller, or
    /// `None` while another [`Esp32I2sCamera`] from `take` is alive (LL-13).
    /// Dropping the controller stops the DMA and releases the claim.
    pub fn take() -> Option<Self> {
        if I2S0_TAKEN.swap(true, Ordering::AcqRel) {
            return None;
        }
        let mut cam = Self::unclaimed();
        cam.claimed = true;
        Some(cam)
    }

    /// Creates an independent host simulator of the controller. Every
    /// register access goes to its private shadow block; no hardware is
    /// touched. Not available on Xtensa, where [`Self::take`] is the only
    /// constructor.
    #[cfg(not(target_arch = "xtensa"))]
    #[inline]
    pub const fn new() -> Self {
        Self::unclaimed()
    }

    const fn unclaimed() -> Self {
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
                in_eof_des_addr: 0,
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
            vsync_invert: false,
            vsync_intr_enabled: false,
            gpio_bindings: [GpioMatrixBinding {
                gpio_num: 0,
                signal_idx: 0,
                inverted: false,
            }; 12],
            gpio_binding_count: 0,
            running: false,
            claimed: false,
            #[cfg(test)]
            dma_resets: 0,
        }
    }

    /// Checks for DMA descriptor errors (`IN_DSCR_ERR` or `IN_DSCR_EMPTY`),
    /// clears the interrupt flags in `int_clr`, and returns `true` if an error
    /// was detected.
    pub fn check_and_clear_dma_err(&mut self) -> bool {
        let err_mask = I2S_IN_DSCR_ERR_INT_RAW | I2S_IN_DSCR_EMPTY_INT_RAW;
        #[cfg(target_arch = "xtensa")]
        {
            let raw = i2s0_status_read!(I2S_INT_RAW_REG_OFFSET);
            if (raw & err_mask) != 0 {
                self.dma_status().clear_int(raw & err_mask);
                return true;
            }
        }
        let raw = volatile_read_u32(&self.regs.int_raw);
        if (raw & err_mask) != 0 {
            volatile_write_u32(&mut self.regs.int_clr, raw & err_mask);
            volatile_write_u32(&mut self.regs.int_raw, raw & !err_mask);
            true
        } else {
            false
        }
    }

    /// Inlink/AHB DMA reset pulses issued by [`Self::stop`] so far.
    #[cfg(test)]
    pub(crate) const fn dma_resets(&self) -> u32 {
        self.dma_resets
    }

    /// Returns a shared reference to the shadow register block for inspection.
    #[inline]
    pub const fn regs(&self) -> &I2s0RegisterBlock {
        &self.regs
    }

    /// Borrows the I2S0 inlink status registers as a [`DmaStatus`] for the
    /// shared half-buffer drain ([`crate::HalfDrain`]).
    #[inline]
    pub fn dma_status(&mut self) -> I2s0DmaStatus<'_> {
        I2s0DmaStatus {
            regs: &mut self.regs,
        }
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

    /// Returns the configured GPIO matrix bindings (`PCLK`, `VSYNC`, `HREF`,
    /// `D0..D7`, `H_ENABLE`).
    #[inline]
    pub fn gpio_bindings(&self) -> &[GpioMatrixBinding] {
        &self.gpio_bindings[..self.gpio_binding_count]
    }

    /// Returns the number of configured GPIO matrix bindings.
    #[inline]
    pub const fn gpio_binding_count(&self) -> usize {
        self.gpio_binding_count
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

    /// Returns the DMA alignment requirement on ESP32 (`0`, matching
    /// `ll_cam_get_dma_align` in `ll_cam.c:465-468`).
    #[inline]
    pub const fn dma_align(&self) -> u8 {
        0
    }

    /// Configures GPIO matrix routing for `PCLK`, `VSYNC`, `HREF`, `D0..D7`, and
    /// `0x38 -> H_ENABLE`,
    /// rejecting any pin `< 0` or `>= 40` (`ll_cam_set_pin` in
    /// `target/esp32/ll_cam.c:387-454`).
    pub fn set_pin(&mut self, cfg: &CamPinConfig) -> Result<(), CameraError> {
        if cfg.pin_vsync < 0
            || (cfg.pin_vsync as u8) >= ESP32_GPIO_NUM_MAX
            || io_mux_reg_addr(cfg.pin_vsync as u8).is_none()
            || cfg.pin_pclk < 0
            || (cfg.pin_pclk as u8) >= ESP32_GPIO_NUM_MAX
            || io_mux_reg_addr(cfg.pin_pclk as u8).is_none()
            || cfg.pin_href < 0
            || (cfg.pin_href as u8) >= ESP32_GPIO_NUM_MAX
            || io_mux_reg_addr(cfg.pin_href as u8).is_none()
        {
            return Err(CameraError::InvalidArg);
        }

        let mut i = 0;
        while i < 8 {
            let pin = cfg.data_pins[i];
            if pin < 0 || (pin as u8) >= ESP32_GPIO_NUM_MAX || io_mux_reg_addr(pin as u8).is_none()
            {
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
            inverted: cfg.vsync_invert,
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
        // JUSTIFICATION: Pad and GPIO-matrix routing for the camera inputs.
        // SAFETY:
        // 1. Alignment & Non-null: Every pin was validated above as a bonded
        //    GPIO (`io_mux_reg_addr` is `Some`, `< 40`), and every signal
        //    index is `<= 192`, so `GPIO_FUNC0_IN_SEL_CFG_REG + 4 * idx` is a
        //    4-byte aligned GPIO-matrix register.
        // 2. Memory layout: 32-bit peripheral registers.
        // 3. Provenance: ESP32 GPIO / IO_MUX / RTC_IO address space.
        // 4. Aliasing: `&mut self` is the single I2S0 owner (LL-13), which
        //    owns the camera pins it was configured with.
        // 5. Volatile semantics: Volatile MMIO accesses.
        // 6. Panic safety: MMIO accesses cannot unwind.
        unsafe {
            xtensa_configure_input_pad(cfg.pin_pclk as u8, false);
            xtensa_configure_input_pad(cfg.pin_vsync as u8, true);
            xtensa_configure_input_pad(cfg.pin_href as u8, false);

            let mut idx = 0;
            while idx < 8 {
                xtensa_configure_input_pad(cfg.data_pins[idx] as u8, false);
                idx += 1;
            }

            self.write_gpio_matrix_bindings();
        }

        Ok(())
    }

    /// Writes the recorded bindings into the GPIO input matrix.
    ///
    /// # Safety
    ///
    /// The bindings must come from a successful [`Self::set_pin`] (validated
    /// pins and I2S0 signal indices).
    #[cfg(target_arch = "xtensa")]
    unsafe fn write_gpio_matrix_bindings(&self) {
        let mut b_idx = 0;
        while b_idx < self.gpio_binding_count {
            let b = self.gpio_bindings[b_idx];
            let inv = if b.signal_idx == I2S0I_V_SYNC_IDX {
                self.vsync_invert
            } else {
                b.inverted
            };
            let sel_val = (1u32 << 7) | ((inv as u32) << 6) | ((b.gpio_num as u32) & 0x3F);
            // JUSTIFICATION: GPIO-matrix input selection for one I2S0 signal.
            // SAFETY:
            // 1. Alignment & Non-null: `signal_idx <= 192` (an I2S0 input
            //    signal from `set_pin`), so the address is a 4-byte aligned
            //    `GPIO_FUNCn_IN_SEL_CFG_REG`.
            // 2. Memory layout: 32-bit peripheral register.
            // 3. Provenance: ESP32 GPIO address space.
            // 4. Aliasing: Only the I2S0 camera input signals are routed.
            // 5. Volatile semantics: Volatile MMIO write.
            // 6. Panic safety: MMIO writes cannot unwind.
            unsafe {
                xtensa_mmio_write(
                    GPIO_FUNC0_IN_SEL_CFG_REG + 4 * (b.signal_idx as usize),
                    sel_val,
                );
            }
            b_idx += 1;
        }
    }

    /// Enables and configures the `I2S0` peripheral in parallel camera slave mode
    /// (`ll_cam_config`
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

        // Force I2S0 FIFO and PLC memory power-up (`FIFO_FORCE_PU |
        // PLC_MEM_FORCE_PU`)
        volatile_write_u32(&mut self.regs.pd_conf, I2S_PD_FIFO_MEM_FORCE_PU);

        // Enable LCD + Camera mode (`I2S0.conf2.lcd_en = 1;
        // I2S0.conf2.camera_en = 1;`)
        volatile_write_u32(&mut self.regs.conf2, I2S_LCD_EN_BIT | I2S_CAMERA_EN_BIT);

        // Clock divider `clkm_div_num = 2, clkm_div_a = 0, clkm_div_b = 0`,
        // `clk_en = 1 << 20`
        volatile_write_u32(&mut self.regs.clkm_conf, (1 << 20) | 2);

        // FIFO config: `dscr_en = 1 (bit 12)`,
        // `rx_fifo_mod_force_en = 1 (bit 20)`,
        // `rx_fifo_mod = sampling_mode (bits 16..18)`
        let fifo_val = (1 << 12) | (1 << 20) | ((self.sampling_mode as u32) << 16);
        volatile_write_u32(&mut self.regs.fifo_conf, fifo_val);

        // `rx_chan_mod = 1`, `rx_bits_mod = 0`,
        // `timing.rx_dsync_sw = 1 (bit 21)`
        volatile_write_u32(&mut self.regs.conf_chan, 1 << 3);
        volatile_write_u32(&mut self.regs.sample_rate_conf, 0);
        volatile_write_u32(&mut self.regs.timing, 1 << 21);

        #[cfg(target_arch = "xtensa")]
        // JUSTIFICATION: Enables, resets and configures the I2S0 block.
        // SAFETY:
        // 1. Alignment & Non-null: Fixed, 4-byte aligned DPORT and I2S0
        //    registers.
        // 2. Memory layout: 32-bit peripheral registers.
        // 3. Provenance: ESP32 DPORT / I2S0 address space.
        // 4. Aliasing: `&mut self` is the single I2S0 owner (LL-13); the
        //    DPORT read-modify-writes touch only the I2S0 clock/reset bits.
        //    `lc_conf` resets the inlink; no DMA is started.
        // 5. Volatile semantics: Volatile MMIO accesses in program order.
        // 6. Panic safety: MMIO accesses cannot unwind.
        unsafe {
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

            // Force I2S0 FIFO and PLC memory power-up (`FIFO_FORCE_PU |
            // PLC_MEM_FORCE_PU`)
            xtensa_mmio_write(
                I2S0_BASE_ADDR + I2S_PD_CONF_REG_OFFSET,
                I2S_PD_FIFO_MEM_FORCE_PU,
            );

            // Enable LCD + Camera slave mode (`0x21`)
            xtensa_mmio_write(
                I2S0_BASE_ADDR + I2S_CONF2_REG_OFFSET,
                I2S_LCD_EN_BIT | I2S_CAMERA_EN_BIT,
            );

            // Configure I2S0 clock divider (`CLK_EN = 1 << 20`,
            // `CLKM_DIV_NUM = 2`)
            xtensa_mmio_write(I2S0_BASE_ADDR + I2S_CLKM_CONF_REG_OFFSET, (1 << 20) | 2);

            // Configure FIFO (`rx_fifo_mod_force_en = 1`,
            // `rx_fifo_mod = sampling_mode`, `dscr_en = 1`)
            xtensa_mmio_write(I2S0_BASE_ADDR + I2S_FIFO_CONF_REG_OFFSET, fifo_val);

            // Configure single RX channel (`rx_chan_mod = 1`),
            // `rx_bits_mod = 0`, `rx_dsync_sw = 1`
            xtensa_mmio_write(I2S0_BASE_ADDR + I2S_CONF_CHAN_REG_OFFSET, 1 << 3);
            let sr =
                xtensa_mmio_read(I2S0_BASE_ADDR + I2S_SAMPLE_RATE_CONF_REG_OFFSET) & !(0x3F << 18);
            xtensa_mmio_write(I2S0_BASE_ADDR + I2S_SAMPLE_RATE_CONF_REG_OFFSET, sr);
            xtensa_mmio_write(I2S0_BASE_ADDR + I2S_TIMING_REG_OFFSET, 1 << 21);
        }

        Ok(())
    }

    /// Configures sampling mode and FIFO mode for the requested pixel format, XCLK
    /// frequency, and sensor PID
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
        // JUSTIFICATION: Programs the I2S0 RX FIFO mode.
        // SAFETY:
        // 1. Alignment & Non-null: `I2S_FIFO_CONF_REG` is a fixed, 4-byte
        //    aligned I2S0 register.
        // 2. Memory layout: 32-bit peripheral register.
        // 3. Provenance: ESP32 I2S0 address space.
        // 4. Aliasing: `&mut self` is the single I2S0 owner (LL-13).
        // 5. Volatile semantics: Volatile MMIO write.
        // 6. Panic safety: MMIO writes cannot unwind.
        unsafe {
            xtensa_mmio_write(I2S0_BASE_ADDR + I2S_FIFO_CONF_REG_OFFSET, fifo_val);
        }

        Ok(mode_cfg)
    }

    /// Enables or disables the VSYNC GPIO interrupt (`ll_cam_vsync_intr_enable`).
    pub fn vsync_intr_enable(&mut self, en: bool) {
        self.vsync_intr_enabled = en;
    }

    /// Starts I2S0 DMA reception into `ring` and returns the guard that owns
    /// the running capture (LL-01, `ll_cam_start` in
    /// `target/esp32/ll_cam.c:305-328`).
    ///
    /// The ring was validated when it was built ([`DmaRing::new`]: alignment,
    /// node size, and on Xtensa the DMA-capable SRAM window). While the
    /// returned [`RunningCapture`] lives, both the camera and the ring are
    /// mutably borrowed; dropping or [stopping](RunningCapture::stop) it
    /// halts and resets the inlink DMA. The ring must be `'static` so that
    /// leaking the guard (`mem::forget`) can never leave the DMA writing into
    /// freed memory; a leaked guard leaves the ring
    /// [armed](DmaRing::is_armed) and unusable. Use [`Self::start_scoped`]
    /// for a borrowed ring.
    ///
    /// Fails with [`CameraError::InvalidState`] when the camera is already
    /// running or the ring is armed, and with [`CameraError::InvalidArg`]
    /// unless `dma_cfg.dma_half_buffer_size` is a non-zero multiple of the
    /// ring's node size that divides the ring.
    pub fn start<'r>(
        &'r mut self,
        dma_cfg: &DmaSizeConfig,
        ring: &'r mut DmaRing<'static>,
    ) -> Result<RunningCapture<'r>, CameraError> {
        // JUSTIFICATION: Safe wrapper over `start_scoped` for `'static` rings.
        // SAFETY:
        // 1. Alignment & Non-null: Checked by `DmaRing::new`.
        // 2. Memory layout: Checked by `DmaRing::new` and `start_scoped`.
        // 3. Provenance & Lifetime: The ring's memory is `'static`, so it
        //    outlives the DMA even if the guard is leaked, which is the
        //    only obligation `start_scoped` puts on its caller.
        // 4. Aliasing: The guard borrows `self` and `ring` mutably for `'r`.
        // 5. Volatile semantics: See `start_scoped`.
        // 6. Panic safety: `start_scoped` does not panic.
        unsafe { self.start_scoped(dma_cfg, ring) }
    }

    /// Starts I2S0 DMA reception into a borrowed `ring`, like
    /// [`Self::start`].
    ///
    /// # Safety
    ///
    /// The returned [`RunningCapture`] must be dropped or
    /// [stopped](RunningCapture::stop) before the memory `ring` borrows is
    /// freed or reused: it must not be leaked (`mem::forget`, `Box::leak`,
    /// reference cycles), or the DMA keeps writing into that memory.
    pub unsafe fn start_scoped<'r>(
        &'r mut self,
        dma_cfg: &DmaSizeConfig,
        ring: &'r mut DmaRing<'_>,
    ) -> Result<RunningCapture<'r>, CameraError> {
        if self.running || ring.is_armed() {
            return Err(CameraError::InvalidState);
        }
        let half = dma_cfg.dma_half_buffer_size;
        if half == 0
            || !half.is_multiple_of(ring.node_size())
            || half > ring.len_bytes()
            || !ring.len_bytes().is_multiple_of(half)
        {
            return Err(CameraError::InvalidArg);
        }
        let first_desc_addr = ring.first_desc_addr();
        let parts = ring.arm()?;
        if !self.start_hw(dma_cfg, first_desc_addr) {
            *parts.armed = false;
            return Err(CameraError::HardwareError);
        }
        Ok(RunningCapture {
            cam: self,
            ring: parts,
            _marker: PhantomData,
        })
    }

    /// Starts I2S0 DMA at a raw 20-bit descriptor address (the pre-WP-08
    /// `start`). Kept only so the firmware's static-ring capture loops keep
    /// working until they move to [`DmaRing`] / [`RunningCapture`].
    ///
    /// # Safety
    ///
    /// `first_desc_addr` must be the `in_link` address of the first
    /// descriptor of a circular [`Lldesc`] ring in DMA-capable internal SRAM
    /// (`0x3FFA_E000..=0x3FFF_FFFF`, see [`crate::validate_dma_ring`]) whose
    /// descriptors and buffers stay valid, and are not accessed by the CPU
    /// except through the WP-06 drain, until [`Self::stop`] returns.
    #[doc(hidden)]
    pub unsafe fn start_raw(&mut self, dma_cfg: &DmaSizeConfig, first_desc_addr: u32) -> bool {
        self.start_hw(dma_cfg, first_desc_addr)
    }

    /// Host simulation of a DMA start for the HAL's software pipeline: marks
    /// the controller running and programs the shadow registers (with
    /// descriptor address 0). Not available on Xtensa, where only
    /// [`Self::start`] starts the DMA.
    #[cfg(not(target_arch = "xtensa"))]
    pub fn simulate_start(&mut self, dma_cfg: &DmaSizeConfig) -> bool {
        self.start_hw(dma_cfg, 0)
    }

    /// Programs and starts the inlink DMA at `first_desc_addr`.
    fn start_hw(&mut self, dma_cfg: &DmaSizeConfig, first_desc_addr: u32) -> bool {
        if dma_cfg.dma_half_buffer_size == 0 {
            return false;
        }
        let conf_stopped = volatile_read_u32(&self.regs.conf) & !I2S_RX_START_BIT;
        volatile_write_u32(&mut self.regs.conf, conf_stopped);

        // Clear the inlink status latches of any earlier run (WP-06: a stale
        // `IN_DSCR_ERR` / `IN_DSCR_EMPTY` would abort every new frame in the
        // shared drain), then enable the `in_suc_eof` interrupt.
        volatile_write_u32(&mut self.regs.int_clr, INLINK_STATUS_LATCHES);
        let int_raw = volatile_read_u32(&self.regs.int_raw) & !INLINK_STATUS_LATCHES;
        volatile_write_u32(&mut self.regs.int_raw, int_raw);
        let int_ena = volatile_read_u32(&self.regs.int_ena) | I2S_IN_SUC_EOF_INT_BIT;
        volatile_write_u32(&mut self.regs.int_ena, int_ena);

        // `I2S0.rx_eof_num = cam->dma_half_buffer_size / sizeof(dma_elem_t)`
        let eof_num = (dma_cfg.dma_half_buffer_size / 4) as u32;
        volatile_write_u32(&mut self.regs.rx_eof_num, eof_num);

        // `I2S0.in_link.addr = addr & 0xfffff; I2S0.in_link.start = 1;`
        let link_val = (first_desc_addr & I2S_INLINK_ADDR_MASK) | I2S_INLINK_START_BIT;
        // LL-03: descriptor and buffer writes happen before the start store.
        dma_fence(Ordering::Release);
        volatile_write_u32(&mut self.regs.in_link, link_val);

        // `I2S0.conf.rx_start = 1;`
        volatile_write_u32(&mut self.regs.conf, conf_stopped | I2S_RX_START_BIT);

        #[cfg(target_arch = "xtensa")]
        // JUSTIFICATION: Resets the I2S0 receiver and inlink, then starts the
        // DMA at `first_desc_addr`.
        // SAFETY:
        // 1. Alignment & Non-null: Fixed, 4-byte aligned I2S0 registers; the
        //    GPIO-matrix writes reuse the bindings validated by `set_pin`.
        // 2. Memory layout: 32-bit peripheral registers.
        // 3. Provenance: ESP32 I2S0 / GPIO address space.
        // 4. Aliasing: `&mut self` is the single I2S0 owner (LL-13). The
        //    `in_link` start makes the DMA write into the ring at
        //    `first_desc_addr`: `start_scoped` passes a validated, armed
        //    `DmaRing`; `start_raw` callers guarantee the same.
        // 5. Volatile semantics: Volatile MMIO in program order; the
        //    `dma_fence(Release)` (`memw`) right before the `in_link` store
        //    completes every descriptor and buffer write first (LL-03).
        // 6. Panic safety: MMIO accesses cannot unwind.
        unsafe {
            let conf_base = (xtensa_mmio_read(I2S0_BASE_ADDR + I2S_CONF_REG_OFFSET)
                & !I2S_RX_START_BIT)
                | I2S_RX_SLAVE_MOD_BIT;
            xtensa_mmio_write(I2S0_BASE_ADDR + I2S_CONF_REG_OFFSET, conf_base);

            xtensa_mmio_write(
                I2S0_BASE_ADDR + I2S_INT_CLR_REG_OFFSET,
                INLINK_STATUS_LATCHES,
            );
            let hw_int_ena =
                xtensa_mmio_read(I2S0_BASE_ADDR + I2S_INT_ENA_REG_OFFSET) | I2S_IN_SUC_EOF_INT_BIT;
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

            // Ensure I2S0 FIFO memory remains powered up and GPIO matrix input bindings
            // stay latched
            xtensa_mmio_write(
                I2S0_BASE_ADDR + I2S_PD_CONF_REG_OFFSET,
                I2S_PD_FIFO_MEM_FORCE_PU,
            );
            self.write_gpio_matrix_bindings();

            xtensa_mmio_write(I2S0_BASE_ADDR + I2S_RXEOF_NUM_REG_OFFSET, eof_num);
            // LL-03: `compiler_fence(Release)` + `memw` before the store to
            // `I2S0.in_link` (`0x3FF4_F034`) that starts the DMA.
            dma_fence(Ordering::Release);
            xtensa_mmio_write(I2S0_BASE_ADDR + I2S_IN_LINK_REG_OFFSET, link_val);
            xtensa_mmio_write(
                I2S0_BASE_ADDR + I2S_CONF_REG_OFFSET,
                conf_base | I2S_RX_START_BIT,
            );
        }

        self.running = true;
        true
    }

    /// Stops I2S0 DMA reception (`ll_cam_stop` in `target/esp32/ll_cam.c:264-270`),
    /// then resets the inlink and AHB master (LL-21) and fences (`memw`), so
    /// no DMA write lands after it returns.
    pub fn stop(&mut self) -> bool {
        let conf_stopped = volatile_read_u32(&self.regs.conf) & !I2S_RX_START_BIT;
        volatile_write_u32(&mut self.regs.conf, conf_stopped);

        let int_ena = volatile_read_u32(&self.regs.int_ena) & !I2S_IN_SUC_EOF_INT_BIT;
        volatile_write_u32(&mut self.regs.int_ena, int_ena);
        volatile_write_u32(&mut self.regs.int_clr, I2S_IN_SUC_EOF_INT_BIT);

        let in_link =
            (volatile_read_u32(&self.regs.in_link) & !I2S_INLINK_START_BIT) | I2S_INLINK_STOP_BIT;
        volatile_write_u32(&mut self.regs.in_link, in_link);

        // LL-21: pulse the inlink and AHB master resets so a write the DMA
        // already queued cannot land after the caller's buffer borrows end.
        volatile_write_u32(
            &mut self.regs.lc_conf,
            I2S_LC_IN_RST_BIT | I2S_LC_AHBM_RST_BIT,
        );
        volatile_write_u32(&mut self.regs.lc_conf, 0);
        #[cfg(test)]
        {
            self.dma_resets += 1;
        }

        #[cfg(target_arch = "xtensa")]
        // JUSTIFICATION: Halts the I2S0 receiver and inlink DMA.
        // SAFETY:
        // 1. Alignment & Non-null: Fixed, 4-byte aligned I2S0 registers.
        // 2. Memory layout: 32-bit peripheral registers.
        // 3. Provenance: ESP32 I2S0 address space.
        // 4. Aliasing: `&mut self` is the single I2S0 owner (LL-13). Stopping
        //    and resetting the DMA only ends its writes into the ring.
        // 5. Volatile semantics: Volatile MMIO in program order.
        // 6. Panic safety: MMIO accesses cannot unwind.
        unsafe {
            let hw_conf =
                xtensa_mmio_read(I2S0_BASE_ADDR + I2S_CONF_REG_OFFSET) & !I2S_RX_START_BIT;
            xtensa_mmio_write(I2S0_BASE_ADDR + I2S_CONF_REG_OFFSET, hw_conf);

            let hw_int_ena =
                xtensa_mmio_read(I2S0_BASE_ADDR + I2S_INT_ENA_REG_OFFSET) & !I2S_IN_SUC_EOF_INT_BIT;
            xtensa_mmio_write(I2S0_BASE_ADDR + I2S_INT_ENA_REG_OFFSET, hw_int_ena);
            xtensa_mmio_write(
                I2S0_BASE_ADDR + I2S_INT_CLR_REG_OFFSET,
                I2S_IN_SUC_EOF_INT_BIT,
            );

            let hw_in_link = (xtensa_mmio_read(I2S0_BASE_ADDR + I2S_IN_LINK_REG_OFFSET)
                & !I2S_INLINK_START_BIT)
                | I2S_INLINK_STOP_BIT;
            xtensa_mmio_write(I2S0_BASE_ADDR + I2S_IN_LINK_REG_OFFSET, hw_in_link);

            // LL-21: reset the inlink state machine and the AHB master.
            xtensa_mmio_write(
                I2S0_BASE_ADDR + I2S_LC_CONF_REG_OFFSET,
                I2S_LC_IN_RST_BIT | I2S_LC_AHBM_RST_BIT,
            );
            xtensa_mmio_write(I2S0_BASE_ADDR + I2S_LC_CONF_REG_OFFSET, 0);
        }
        // Drain the write buffers before the caller reads or reuses the ring
        // (Acquire side of the DMA hand-back).
        dma_fence(Ordering::SeqCst);

        self.running = false;
        true
    }
}

/// A running I2S0 capture into a [`DmaRing`], returned by
/// [`Esp32I2sCamera::start`] (LL-01).
///
/// It mutably borrows the camera and the ring, so neither can be used,
/// moved or dropped while the DMA writes. Completed halves are consumed
/// through [`Self::drain_ready_halves`] / [`Self::poll`], which reach the
/// ring only through the shared WP-06 drain. Dropping the guard, or
/// [`Self::stop`], stops and resets the DMA (LL-21); after that the
/// tail half can be drained from [`DmaRing::parts_mut`] (WP-06 order:
/// stop first, then the tail).
#[must_use = "dropping a RunningCapture immediately stops the DMA"]
#[derive(Debug)]
pub struct RunningCapture<'r> {
    cam: &'r mut Esp32I2sCamera,
    ring: RingParts<'r>,
    _marker: PhantomData<&'r mut [u8]>,
}

impl RunningCapture<'_> {
    /// `true` until the capture is stopped.
    #[inline]
    pub const fn is_running(&self) -> bool {
        self.cam.running
    }

    /// The camera, for inspection (filter, shadow registers).
    #[inline]
    pub fn camera(&self) -> &Esp32I2sCamera {
        self.cam
    }

    /// The ring's node size in bytes.
    #[inline]
    pub const fn node_size(&self) -> usize {
        self.ring.node_size
    }

    /// Stops and resets the DMA (the same as dropping the guard).
    #[inline]
    pub fn stop(self) {
        drop(self);
    }

    /// The ring and the DMA status for one drain call.
    fn ring_and_status(&mut self) -> (&mut [Lldesc], &mut [u8], I2s0DmaStatus<'_>) {
        // JUSTIFICATION: Materializes the armed ring for one drain call.
        // SAFETY:
        // 1. Alignment & Non-null: The pointers come from the `&mut` slices
        //    of a validated `DmaRing` (`DmaRing::arm`).
        // 2. Memory layout: `[Lldesc]` and `[u8]` slices of the original
        //    lengths.
        // 3. Provenance & Lifetime: The ring stays mutably borrowed for `'r`
        //    (and its memory outlives the guard), and the returned
        //    references are tied to `&mut self`, so they end before the
        //    guard does.
        // 4. Aliasing: No other CPU reference to the ring exists while it is
        //    armed (`DmaRing` accessors fail), and `&mut self` makes these
        //    the only ones. The DMA engine writes only the halves it owns,
        //    and the drain reads a half only after the EOF position, the
        //    owner bit and a `memw` (`I2s0DmaStatus::barrier`) say the DMA
        //    has finished it (LL-03, WP-06); the headers are atomics.
        // 5. Volatile semantics: Descriptor headers use atomics; buffer
        //    reads follow the drain's barrier.
        // 6. Panic safety: Creating references cannot unwind.
        let (desc, buf) = unsafe { (self.ring.descriptors.as_mut(), self.ring.buffer.as_mut()) };
        (desc, buf, self.cam.dma_status())
    }

    /// Consumes every completed half through `drain`
    /// ([`HalfDrain::drain_ready_halves`]).
    pub fn drain_ready_halves<F>(
        &mut self,
        drain: &mut HalfDrain,
        scratch: &mut [u8],
        sink: F,
    ) -> Result<usize, CameraError>
    where
        F: FnMut(&[u8]) -> ControlFlow<()>,
    {
        let (desc, buf, mut status) = self.ring_and_status();
        drain.drain_ready_halves(desc, buf, &mut status, scratch, sink)
    }

    /// Runs one [`PolledJpegCapture::poll`] step on this capture. On
    /// [`CaptureStep::FrameEnded`], stop the capture and call
    /// [`PolledJpegCapture::finish`] on the ring.
    pub fn poll<H, V>(
        &mut self,
        capture: &mut PolledJpegCapture,
        jpeg_out: &mut [u8],
        vsync_active: &mut V,
        hooks: &mut H,
    ) -> CaptureStep
    where
        H: CaptureHooks,
        V: FnMut() -> bool,
    {
        let (desc, buf, mut status) = self.ring_and_status();
        capture.poll(desc, buf, &mut status, jpeg_out, vsync_active, hooks)
    }

    /// Test hook: the DMA writes `bytes` at ring offset `at`.
    #[cfg(test)]
    pub(crate) fn simulate_dma_write(&mut self, at: usize, bytes: &[u8]) {
        let (_, buf, _) = self.ring_and_status();
        buf[at..at + bytes.len()].copy_from_slice(bytes);
    }

    /// Test hook: the DMA completes JPEG half `h` (two descriptors): owner
    /// write-back, EOF on the last descriptor, `IN_SUC_EOF` raised.
    #[cfg(test)]
    pub(crate) fn simulate_half_complete(&mut self, h: usize) {
        let node = self.ring.node_size as u16;
        let (desc, _, _) = self.ring_and_status();
        for d in [2 * h, 2 * h + 1] {
            desc[d].store_flags(Lldesc::pack_flags(
                node,
                node,
                0,
                false,
                d == 2 * h + 1,
                false,
            ));
        }
        let eof_addr = desc[2 * h + 1].link_addr_20bit();
        self.cam.regs.in_eof_des_addr = eof_addr;
        self.cam.regs.int_raw |= I2S_IN_SUC_EOF_INT_BIT;
    }
}

impl Drop for RunningCapture<'_> {
    fn drop(&mut self) {
        let _ = self.cam.stop();
        *self.ring.armed = false;
    }
}

/// Halts hardware I2S0 RX and DMA in-link during emergency panic/reset
/// teardown without requiring `&mut Esp32I2sCamera`.
///
/// # Safety
///
/// This function directly accesses hardware MMIO registers without
/// ownership of the [`Esp32I2sCamera`] instance. The caller must ensure
/// that no concurrent access to I2S0 registers occurs and that halting RX
/// and DMA in-link is safe in the current execution state (e.g. panic or
/// reset teardown).
pub unsafe fn emergency_stop_i2s0_hw() {
    #[cfg(target_arch = "xtensa")]
    // JUSTIFICATION: Emergency stop directly manipulates I2S0 hardware MMIO
    // registers without holding an exclusive `&mut Esp32I2sCamera` handle.
    // SAFETY:
    // 1. Alignment & Non-null: All target register addresses are 4-byte
    //    aligned ESP32 I2S0 MMIO addresses (`0x3FF4_F000..0x3FF4_F0FF`).
    // 2. Memory layout: I2S0 MMIO registers are 32-bit memory-mapped words.
    // 3. Provenance: Fixed peripheral memory map on Xtensa ESP32.
    // 4. Aliasing: The caller guarantees no concurrent I2S0 access (function
    //    contract); halting the DMA only ends its writes.
    // 5. Volatile semantics: Volatile MMIO stores in program order.
    // 6. Panic safety: MMIO register writes on Xtensa do not unwind or panic.
    unsafe {
        let hw_conf = xtensa_mmio_read(I2S0_BASE_ADDR + I2S_CONF_REG_OFFSET) & !I2S_RX_START_BIT;
        xtensa_mmio_write(I2S0_BASE_ADDR + I2S_CONF_REG_OFFSET, hw_conf);

        let hw_int_ena =
            xtensa_mmio_read(I2S0_BASE_ADDR + I2S_INT_ENA_REG_OFFSET) & !I2S_IN_SUC_EOF_INT_BIT;
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
}

impl Esp32I2sCamera {
    /// Halts hardware I2S0 RX and DMA in-link during emergency panic/reset
    /// teardown without requiring `&mut Esp32I2sCamera`.
    ///
    /// # Safety
    ///
    /// See module-level [`emergency_stop_i2s0_hw`] for full safety contract.
    #[inline]
    pub unsafe fn emergency_stop_i2s0_hw() {
        // JUSTIFICATION: Delegates directly to module-level emergency stop function.
        // SAFETY: Caller satisfies all emergency stop preconditions forwarded
        // to `emergency_stop_i2s0_hw`.
        unsafe {
            emergency_stop_i2s0_hw();
        }
    }

    /// Deinitializes the I2S0 camera peripheral and gates its DPORT clock
    /// (`ll_cam_deinit`
    /// in `target/esp32/ll_cam.c:272-303`).
    pub fn deinit(&mut self) -> Result<(), CameraError> {
        let _ = self.stop();
        self.vsync_intr_enabled = false;
        self.vsync_pin = None;
        self.gpio_binding_count = 0;

        let clk = volatile_read_u32(&self.regs.dport_clk_en) & !DPORT_I2S0_CLK_EN;
        volatile_write_u32(&mut self.regs.dport_clk_en, clk);
        let rst = volatile_read_u32(&self.regs.dport_rst_en) | DPORT_I2S0_RST;
        volatile_write_u32(&mut self.regs.dport_rst_en, rst);

        #[cfg(target_arch = "xtensa")]
        // JUSTIFICATION: Resets the inlink and gates the I2S0 clock.
        // SAFETY:
        // 1. Alignment & Non-null: Fixed, 4-byte aligned I2S0 and DPORT
        //    registers.
        // 2. Memory layout: 32-bit peripheral registers.
        // 3. Provenance: ESP32 I2S0 / DPORT address space.
        // 4. Aliasing: `&mut self` is the single I2S0 owner (LL-13); the DMA
        //    was stopped above and the DPORT read-modify-writes touch only
        //    the I2S0 bits.
        // 5. Volatile semantics: Volatile MMIO in program order.
        // 6. Panic safety: MMIO accesses cannot unwind.
        unsafe {
            xtensa_mmio_write(
                I2S0_BASE_ADDR + I2S_LC_CONF_REG_OFFSET,
                I2S_LC_IN_RST_BIT | I2S_LC_AHBM_RST_BIT | I2S_LC_AHBM_FIFO_RST_BIT,
            );
            xtensa_mmio_write(I2S0_BASE_ADDR + I2S_LC_CONF_REG_OFFSET, 0);
            let hw_clk = xtensa_mmio_read(DPORT_PERIP_CLK_EN_REG) & !DPORT_I2S0_CLK_EN;
            xtensa_mmio_write(DPORT_PERIP_CLK_EN_REG, hw_clk);
            let hw_rst = xtensa_mmio_read(DPORT_PERIP_RST_EN_REG) | DPORT_I2S0_RST;
            xtensa_mmio_write(DPORT_PERIP_RST_EN_REG, hw_rst);
        }

        Ok(())
    }

    /// Unpacks a raw DMA half-buffer `src` into `dst` using the active DMA filter
    /// (`ll_cam_memcpy`).
    #[inline]
    pub fn memcpy_filtered(&self, dst: &mut [u8], src: &[u8]) -> usize {
        self.filter.apply(dst, src)
    }

    /// Captures a complete JPEG frame (`0xFF, 0xD8, 0xFF` .. `0xFF, 0xD9`) using a
    /// circular
    /// [`Lldesc`] DMA ring over `dma_buf`, unpacking `4096`-byte DMA half-buffers
    /// into `1024`-byte
    /// JPEG chunks in `jpeg_out` and returning the exact `SOI..=EOI` byte length.
    ///
    /// Equivalent to [`Self::capture_jpeg_frame_polled_with_hooks`] with
    /// [`NoCaptureHooks`].
    pub fn capture_jpeg_frame_polled(
        &mut self,
        descriptors: &mut [Lldesc],
        dma_buf: &mut [u8],
        jpeg_out: &mut [u8],
        max_poll_iters: u32,
    ) -> Result<usize, CameraError> {
        self.capture_jpeg_frame_polled_with_hooks(
            descriptors,
            dma_buf,
            jpeg_out,
            max_poll_iters,
            &mut NoCaptureHooks,
        )
    }

    /// Captures a complete JPEG frame like [`Self::capture_jpeg_frame_polled`],
    /// calling back into `hooks` while it waits (LL-15).
    ///
    /// - On `#[cfg(target_arch = "xtensa")]`, poisons the ring, synchronizes
    ///   with `VSYNC` (`GPIO_IN_REG`), starts `I2S0` DMA and runs a
    ///   [`PolledJpegCapture`]: completed half-buffers are consumed through the
    ///   shared [`crate::HalfDrain`] (EOF position and owner write-back gated,
    ///   poisoned and re-armed, overrun reported as
    ///   [`CameraError::Overflow`]), and the final partial half-buffer is
    ///   processed unconditionally once `VSYNC` ends.
    /// - On `#[cfg(not(target_arch = "xtensa"))]`, builds circular ring
    ///   and unpacks the pre-populated DMA half-buffers in `dma_buf`
    ///   for deterministic host unit and Miri verification.
    pub fn capture_jpeg_frame_polled_with_hooks<H: CaptureHooks>(
        &mut self,
        descriptors: &mut [Lldesc],
        dma_buf: &mut [u8],
        jpeg_out: &mut [u8],
        max_poll_iters: u32,
        hooks: &mut H,
    ) -> Result<usize, CameraError> {
        let desc_cnt = descriptors.len();
        if desc_cnt < 2
            || !desc_cnt.is_multiple_of(2)
            || jpeg_out.len() < JPEG_UNPACKED_HALF_BUFFER_SIZE
            || max_poll_iters == 0
            || !dma_buf.as_ptr().cast::<u32>().is_aligned()
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

        // LL-01: the checked constructor validates alignment and node size,
        // and on Xtensa that the descriptors and the buffer are in the
        // DMA-capable SRAM window (no PSRAM).
        let mut ring = DmaRing::new(
            descriptors,
            &mut dma_buf[..required_dma_bytes],
            JPEG_DMA_NODE_SIZE,
        )?;

        #[cfg(target_arch = "xtensa")]
        {
            // LL-06: no byte of an earlier frame survives anywhere in the ring.
            ring.poison()?;
            let vsync_pin = self.vsync_pin.unwrap_or(25);
            let vsync_sync_level = !self.vsync_invert;

            // Synchronize to VSYNC vertical blanking transition before starting I2S0 DMA
            let mut sync_iters = 0u32;
            while xtensa_read_gpio_level(vsync_pin) != vsync_sync_level
                && sync_iters < max_poll_iters
            {
                sync_iters += 1;
                if sync_iters.is_multiple_of(PolledJpegCapture::IDLE_HOOK_POLLS) {
                    hooks.idle();
                }
            }
            if sync_iters >= max_poll_iters {
                return Err(CameraError::Timeout);
            }

            let mut capture = PolledJpegCapture::new(
                half_buf_cnt,
                self.filter,
                crate::dma_drain::DrainGate::OwnerWriteback,
            )?;
            // JUSTIFICATION: Starts the DMA on a ring borrowed for this call.
            // SAFETY:
            // 1. Alignment & Non-null: `ring` was validated by `DmaRing::new`
            //    (including the DMA window on Xtensa).
            // 2. Memory layout: Checked by `start_scoped`.
            // 3. Provenance & Lifetime: The guard is consumed by
            //    `run_polled_capture`, which stops the DMA on every exit
            //    before this function returns, so the borrowed buffers
            //    outlive every DMA write.
            // 4. Aliasing: The guard borrows `self` and `ring` mutably.
            // 5. Volatile semantics: See `start_scoped`.
            // 6. Panic safety: The guard is never leaked; on unwind its
            //    `Drop` stops the DMA.
            let running = unsafe { self.start_scoped(&dma_cfg, &mut ring) }?;

            let mut vsync_active = || xtensa_read_gpio_level(vsync_pin) != vsync_sync_level;
            let ended = Self::run_polled_capture(
                running,
                &mut capture,
                jpeg_out,
                max_poll_iters,
                &mut vsync_active,
                hooks,
            )?;
            match ended {
                Some(len) => Ok(len),
                None => {
                    // WP-06 order: the DMA is stopped, now drain the tail.
                    let (desc, buf) = ring.parts_mut()?;
                    capture.finish(desc, buf, &mut self.dma_status(), jpeg_out)
                }
            }
        }

        #[cfg(not(target_arch = "xtensa"))]
        {
            let _ = hooks;
            let first_desc_addr = ring.first_desc_addr();
            // Host: no DMA engine exists; this only programs the shadow
            // registers. The halves were pre-populated by the caller.
            if !self.start_hw(&dma_cfg, first_desc_addr) {
                return Err(CameraError::HardwareError);
            }
            let (descriptors, dma_buf) = ring.parts_mut()?;

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
                let flags = Lldesc::pack_flags(JPEG_DMA_NODE_SIZE as u16, 0, 0, false, false, true);
                descriptors[2 * h].store_flags(flags);
                descriptors[2 * h + 1].store_flags(flags);

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

    /// Drives a started [`PolledJpegCapture`] for at most `max_poll_iters`
    /// polls and stops the DMA on every exit by consuming `running`.
    /// Returns `Ok(Some(len))` for a complete frame and `Ok(None)` when the
    /// frame ended and the caller must [finish](PolledJpegCapture::finish)
    /// it on the stopped ring. Target-independent so the host build
    /// type-checks the loop the Xtensa capture runs.
    #[cfg_attr(not(target_arch = "xtensa"), allow(dead_code))]
    fn run_polled_capture<H: CaptureHooks, V: FnMut() -> bool>(
        mut running: RunningCapture<'_>,
        capture: &mut PolledJpegCapture,
        jpeg_out: &mut [u8],
        max_poll_iters: u32,
        vsync_active: &mut V,
        hooks: &mut H,
    ) -> Result<Option<usize>, CameraError> {
        let mut poll_iters = 0u32;
        while poll_iters < max_poll_iters {
            poll_iters += 1;
            match running.poll(capture, jpeg_out, vsync_active, hooks) {
                CaptureStep::Pending => {}
                CaptureStep::Complete(len) => {
                    running.stop();
                    return Ok(Some(len));
                }
                CaptureStep::Failed(e) => {
                    running.stop();
                    return Err(e);
                }
                CaptureStep::FrameEnded => {
                    running.stop();
                    return Ok(None);
                }
            }
        }
        running.stop();
        Err(CameraError::Timeout)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dma_filter::DmaElement;
    use crate::dma_ring::{calc_dma_sizes, DmaRingStorage, DEFAULT_CAMERA_DMA_BUFFER_SIZE_MAX};
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
        assert_eq!(io_mux_reg_addr(4), Some(0x3FF4_9048)); // D0
        assert_eq!(io_mux_reg_addr(5), Some(0x3FF4_906C)); // D1
        assert_eq!(io_mux_reg_addr(18), Some(0x3FF4_9070)); // D2
        assert_eq!(io_mux_reg_addr(19), Some(0x3FF4_9074)); // D3
        assert_eq!(io_mux_reg_addr(36), Some(0x3FF4_9004)); // D4
        assert_eq!(io_mux_reg_addr(39), Some(0x3FF4_9010)); // D5
        assert_eq!(io_mux_reg_addr(34), Some(0x3FF4_9014)); // D6
        assert_eq!(io_mux_reg_addr(35), Some(0x3FF4_9018)); // D7

        // Unbonded / out-of-range pins (LL-005)
        assert_eq!(io_mux_reg_addr(20), None);
        assert_eq!(io_mux_reg_addr(24), None);
        assert_eq!(io_mux_reg_addr(28), None);
        assert_eq!(io_mux_reg_addr(40), None);
    }

    #[test]
    fn test_check_and_clear_dma_err() {
        let mut cam = Esp32I2sCamera::new();
        assert!(!cam.check_and_clear_dma_err());
        cam.regs.int_raw = I2S_IN_DSCR_ERR_INT_RAW;
        assert!(cam.check_and_clear_dma_err());
        assert_eq!(cam.regs().int_clr, I2S_IN_DSCR_ERR_INT_RAW);
        assert_eq!(cam.regs().int_raw & I2S_IN_DSCR_ERR_INT_RAW, 0);

        cam.regs.int_raw = I2S_IN_DSCR_EMPTY_INT_RAW;
        assert!(cam.check_and_clear_dma_err());
        assert_eq!(cam.regs().int_clr, I2S_IN_DSCR_EMPTY_INT_RAW);
        assert_eq!(cam.regs().int_raw & I2S_IN_DSCR_EMPTY_INT_RAW, 0);
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
        assert!(cam.gpio_bindings()[1].inverted);
        assert_eq!(cam.gpio_bindings()[2].signal_idx, I2S0I_H_SYNC_IDX);
        assert_eq!(cam.gpio_bindings()[3].signal_idx, I2S0I_DATA_IN0_IDX);
        assert_eq!(cam.gpio_bindings()[10].signal_idx, I2S0I_DATA_IN7_IDX);
        assert_eq!(
            cam.gpio_bindings()[11].gpio_num,
            GPIO_MATRIX_CONST_ONE_INPUT
        );
        assert_eq!(cam.gpio_bindings()[11].signal_idx, I2S0I_H_ENABLE_IDX);

        // Out-of-range GPIO >= 40 and unbonded GPIO 20, 24, 28 rejected (LL-005)
        let mut bad_pins = valid_pins;
        bad_pins.pin_vsync = 40;
        assert_eq!(cam.set_pin(&bad_pins), Err(CameraError::InvalidArg));

        let mut bad_pin20 = valid_pins;
        bad_pin20.pin_vsync = 20;
        assert_eq!(cam.set_pin(&bad_pin20), Err(CameraError::InvalidArg));

        let mut bad_pin24 = valid_pins;
        bad_pin24.pin_href = 24;
        assert_eq!(cam.set_pin(&bad_pin24), Err(CameraError::InvalidArg));

        let mut bad_pin28 = valid_pins;
        bad_pin28.data_pins[0] = 28;
        assert_eq!(cam.set_pin(&bad_pin28), Err(CameraError::InvalidArg));

        let mut bad_data = valid_pins;
        bad_data.data_pins[7] = -1;
        assert_eq!(cam.set_pin(&bad_data), Err(CameraError::InvalidArg));
    }

    #[test]
    fn test_esp32_i2s_state_transitions() {
        let mut cam = Esp32I2sCamera::new();
        assert_eq!(cam.config(), Ok(()));
        assert_eq!(
            cam.regs().dport_clk_en & DPORT_I2S0_CLK_EN,
            DPORT_I2S0_CLK_EN
        );
        assert_eq!(cam.regs().dport_rst_en & DPORT_I2S0_RST, 0);
        assert_eq!(cam.regs().pd_conf, I2S_PD_FIFO_MEM_FORCE_PU);
        assert_eq!(I2S_PD_CONF_REG_OFFSET, 0xA4);
        assert_eq!(I2S_PD_FIFO_MEM_FORCE_PU, 0x0A);
        assert_eq!(cam.regs().conf2, I2S_LCD_EN_BIT | I2S_CAMERA_EN_BIT);
        assert_eq!(cam.regs().clkm_conf, (1 << 20) | 2);

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

        // JUSTIFICATION: Exercises the raw migration entry point.
        // SAFETY:
        // 1. Alignment & Non-null: On the host no DMA engine exists; the
        //    address is only stored in the shadow `in_link` register.
        // 2. Memory layout: No descriptor is dereferenced.
        // 3. Provenance & Lifetime: Nothing is read or written through it.
        // 4. Aliasing: `cam` is a host shadow-register simulator.
        // 5. Volatile semantics: Shadow writes only.
        // 6. Panic safety: `start_raw` does not panic.
        assert!(unsafe { cam.start_raw(&dma_cfg, 0x3FFB_1234) });
        assert!(cam.is_running());
        assert_eq!(cam.regs().rx_eof_num, 4096);
        assert_eq!(
            cam.regs().in_link & I2S_INLINK_ADDR_MASK,
            0xFB_1234 & 0xF_FFFF
        );
        assert_ne!(cam.regs().in_link & I2S_INLINK_START_BIT, 0);
        assert_ne!(cam.regs().conf & I2S_RX_START_BIT, 0);

        assert!(cam.stop());
        assert!(!cam.is_running());
        assert_eq!(cam.regs().conf & I2S_RX_START_BIT, 0);
        assert_ne!(cam.regs().in_link & I2S_INLINK_STOP_BIT, 0);

        assert_eq!(cam.deinit(), Ok(()));
        assert_eq!(cam.regs().dport_clk_en & DPORT_I2S0_CLK_EN, 0);
        assert_eq!(cam.regs().dport_rst_en & DPORT_I2S0_RST, DPORT_I2S0_RST);
        assert_eq!(cam.gpio_binding_count(), 0);
    }

    #[test]
    fn test_vsync_invert_gpio_binding() {
        let mut cam = Esp32I2sCamera::new();
        let mut pins = CamPinConfig {
            pin_vsync: 25,
            pin_href: 23,
            pin_pclk: 22,
            pin_xclk: 21,
            data_pins: [4, 5, 18, 19, 36, 39, 34, 35],
            vsync_invert: false,
        };
        assert_eq!(cam.set_pin(&pins), Ok(()));
        assert!(!cam.gpio_bindings()[1].inverted);

        pins.vsync_invert = true;
        assert_eq!(cam.set_pin(&pins), Ok(()));
        assert!(cam.gpio_bindings()[1].inverted);
    }

    #[test]
    fn test_esp32_i2s_deinit_resets_gpio_binding_count() {
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
        assert_eq!(cam.gpio_binding_count(), 12);
        assert_eq!(cam.gpio_bindings().len(), 12);

        assert_eq!(cam.deinit(), Ok(()));
        assert_eq!(cam.gpio_binding_count(), 0);
        assert_eq!(cam.gpio_bindings().len(), 0);
    }

    #[test]
    fn test_capture_jpeg_frame_polled_unpacking_and_trimming() {
        let mut cam = Esp32I2sCamera::new();
        assert_eq!(cam.config(), Ok(()));
        let _ = cam
            .set_sample_mode(PixelFormat::Jpeg, 20_000_000, CameraPid::OV2640.as_u16())
            .expect("set jpeg sample mode");

        let mut descriptors = [Lldesc::EMPTY; 4];
        #[repr(C, align(4))]
        struct AlignedBuf([u8; 4 * JPEG_DMA_NODE_SIZE]);
        let mut dma_storage = AlignedBuf([0u8; 4 * JPEG_DMA_NODE_SIZE]);
        let dma_buf = &mut dma_storage.0;
        let mut jpeg_out = [0u8; 2048];

        // Embed 4 leading padding bytes, followed by a 1030-byte JPEG frame spanning
        // across
        // half-buffer 0 (1024 unpacked bytes) and half-buffer 1 (partial unpacked
        // bytes),
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
            .capture_jpeg_frame_polled(&mut descriptors, dma_buf, &mut jpeg_out, 16)
            .expect("polled JPEG capture should succeed");
        assert_eq!(captured_len, 1030);
        assert_eq!(&jpeg_out[..3], &[0xFF, 0xD8, 0xFF]);
        assert_eq!(&jpeg_out[captured_len - 2..captured_len], &[0xFF, 0xD9]);
        assert!(!cam.is_running());

        // Verify descriptor 0 address was programmed into in_link (LL-001)
        let expected_desc_addr =
            (core::ptr::from_ref(&descriptors[0]) as usize as u32) & I2S_INLINK_ADDR_MASK;
        assert_eq!(
            cam.regs().in_link & I2S_INLINK_ADDR_MASK,
            expected_desc_addr
        );
    }

    #[test]
    fn test_polled_jpeg_capture_scans_and_truncates_at_eoi() {
        let mut cam = Esp32I2sCamera::new();
        assert_eq!(cam.config(), Ok(()));
        let _ = cam
            .set_sample_mode(PixelFormat::Jpeg, 20_000_000, CameraPid::OV2640.as_u16())
            .expect("set jpeg sample mode");

        let mut descriptors = [Lldesc::EMPTY; 2];
        #[repr(C, align(4))]
        struct AlignedBuf([u8; 2 * JPEG_DMA_NODE_SIZE]);
        let mut dma_storage = AlignedBuf([0u8; 2 * JPEG_DMA_NODE_SIZE]);
        let dma_buf = &mut dma_storage.0;
        let mut jpeg_out = [0u8; 1024];

        // Verify LL-002: unaligned dma_buf is rejected
        #[repr(C, align(4))]
        struct UnalignedTestBuf([u8; 2 * JPEG_DMA_NODE_SIZE + 4]);
        let mut aligned = UnalignedTestBuf([0u8; 2 * JPEG_DMA_NODE_SIZE + 4]);
        let unaligned_slice = &mut aligned.0[1..1 + 2 * JPEG_DMA_NODE_SIZE];
        assert_eq!(
            cam.capture_jpeg_frame_polled(&mut descriptors, unaligned_slice, &mut jpeg_out, 16),
            Err(CameraError::InvalidArg)
        );

        // Verify LL-004: EOI at indices 2..=3
        let el_soi0 = DmaElement::new(0xFF, 0);
        let el_soi1 = DmaElement::new(0xD8, 0);
        let el_eoi0 = DmaElement::new(0xFF, 0);
        let el_eoi1 = DmaElement::new(0xD9, 0);
        dma_buf[0..4].copy_from_slice(&el_soi0.0);
        dma_buf[4..8].copy_from_slice(&el_soi1.0);
        dma_buf[8..12].copy_from_slice(&el_eoi0.0);
        dma_buf[12..16].copy_from_slice(&el_eoi1.0);

        let captured = cam
            .capture_jpeg_frame_polled(&mut descriptors, dma_buf, &mut jpeg_out, 16)
            .expect("eoi at 2..=3 captured");
        assert_eq!(captured, 4);
        assert_eq!(&jpeg_out[..4], &[0xFF, 0xD8, 0xFF, 0xD9]);

        // Verify LL-001: in_link holds descriptors[0] address, not descriptors[1]
        let expected_desc0 =
            (core::ptr::from_ref(&descriptors[0]) as usize as u32) & I2S_INLINK_ADDR_MASK;
        assert_eq!(cam.regs().in_link & I2S_INLINK_ADDR_MASK, expected_desc0);

        // Verify LL-002: Drop stops running camera
        {
            let mut cam2 = Esp32I2sCamera::new();
            assert_eq!(cam2.config(), Ok(()));
            let dma_cfg = DmaSizeConfig {
                dma_bytes_per_item: 4,
                dma_buffer_size: 4096,
                dma_half_buffer_size: 2048,
                dma_half_buffer_cnt: 2,
                dma_node_buffer_size: 2048,
                dma_node_cnt: 2,
                lines_per_node: 1,
                nodes_per_line: 1,
                lines_per_half_buffer: 1,
                image_size: 1024,
            };
            assert!(cam2.simulate_start(&dma_cfg));
            assert!(cam2.is_running());
            drop(cam2); // Drops and executes stop()
        }
    }

    #[test]
    fn test_i2s_rx_bits_mod_clear_mask() {
        // ESP32 Technical Reference Manual (Table 12-4):
        // I2S_SAMPLE_RATE_CONF_REG:
        // - I2S_TX_BCK_DIV_NUM: bits [5:0]
        // - I2S_RX_BCK_DIV_NUM: bits [11:6]
        // - I2S_TX_BITS_MOD:    bits [17:12] (mask: 0x3F << 12)
        // - I2S_RX_BITS_MOD:    bits [23:18] (mask: 0x3F << 18)
        let rx_bits_mod_mask: u32 = 0x3F << 18;
        let tx_bits_mod_mask: u32 = 0x3F << 12;

        // Verify that the I2S_RX_BITS_MOD clear mask !(0x3F << 18) clears
        // bits [23:18] and preserves I2S_TX_BITS_MOD bits [17:12].
        let initial_val: u32 = 0xFFFF_FFFF;
        let cleared = initial_val & !rx_bits_mod_mask;

        // Bits [23:18] must be 0
        assert_eq!(cleared & rx_bits_mod_mask, 0);
        // Bits [17:12] (TX_BITS_MOD) must be preserved
        assert_eq!(cleared & tx_bits_mod_mask, tx_bits_mod_mask);

        // Verify with arbitrary distinct bit patterns in RX and TX fields
        let sample_reg: u32 = (0x2A << 18) | (0x15 << 12) | 0x3F;
        let result = sample_reg & !(0x3F << 18);
        assert_eq!(result & (0x3F << 18), 0);
        assert_eq!(result & (0x3F << 12), 0x15 << 12);
        assert_eq!(result & 0x3F, 0x3F);
    }

    #[test]
    fn test_dma_status_reads_and_clears_the_shadow_registers() {
        use crate::dma_drain::DmaStatus;
        let mut cam = Esp32I2sCamera::new();
        cam.regs.int_raw = I2S_IN_SUC_EOF_INT_BIT | I2S_IN_DSCR_ERR_INT_RAW;
        cam.regs.in_eof_des_addr = 0x0004_1234;
        {
            let mut st = cam.dma_status();
            assert_eq!(
                st.int_raw(),
                I2S_IN_SUC_EOF_INT_BIT | I2S_IN_DSCR_ERR_INT_RAW
            );
            assert_eq!(st.eof_desc_addr(), 0x0004_1234);
            st.clear_int(I2S_IN_DSCR_ERR_INT_RAW);
            st.barrier();
        }
        assert_eq!(cam.regs().int_raw, I2S_IN_SUC_EOF_INT_BIT);
        assert_eq!(cam.regs().int_clr, I2S_IN_DSCR_ERR_INT_RAW);
    }

    #[test]
    fn test_dma_status_feeds_the_shared_half_drain() {
        use crate::dma_drain::{DrainGate, HalfDrain};
        use core::ops::ControlFlow;

        let mut cam = Esp32I2sCamera::new();
        let mut descriptors = [Lldesc::EMPTY; 4];
        #[repr(C, align(4))]
        struct AlignedBuf([u8; 4 * JPEG_DMA_NODE_SIZE]);
        let mut storage = AlignedBuf([0u8; 4 * JPEG_DMA_NODE_SIZE]);
        let dma_buf = &mut storage.0;
        crate::dma_ring::build_circular_ring(&mut descriptors, dma_buf, JPEG_DMA_NODE_SIZE)
            .expect("ring");
        dma_buf[..4].copy_from_slice(&DmaElement::new(0xFF, 0).0);

        // The DMA completed half 0.
        cam.regs.int_raw = I2S_IN_SUC_EOF_INT_BIT;
        cam.regs.in_eof_des_addr = descriptors[1].link_addr_20bit();

        let mut drain = HalfDrain::new(2, JPEG_DMA_NODE_SIZE, cam.filter(), DrainGate::EofPosition)
            .expect("drain");
        let mut scratch = [0u8; JPEG_UNPACKED_HALF_BUFFER_SIZE];
        let mut first = 0u8;
        let n = drain
            .drain_ready_halves(
                &mut descriptors,
                dma_buf,
                &mut cam.dma_status(),
                &mut scratch,
                |c| {
                    first = c[0];
                    ControlFlow::Continue(())
                },
            )
            .expect("drain");
        assert_eq!(n, 1);
        assert_eq!(first, 0xFF);
        assert_eq!(cam.regs().int_raw & I2S_IN_SUC_EOF_INT_BIT, 0);
        assert_eq!(drain.half_idx(), 1);
    }

    #[test]
    fn test_capture_with_hooks_matches_plain_capture_on_host() {
        let mut cam = Esp32I2sCamera::new();
        assert_eq!(cam.config(), Ok(()));
        let mut descriptors = [Lldesc::EMPTY; 2];
        #[repr(C, align(4))]
        struct AlignedBuf([u8; 2 * JPEG_DMA_NODE_SIZE]);
        let mut storage = AlignedBuf([0u8; 2 * JPEG_DMA_NODE_SIZE]);
        let dma_buf = &mut storage.0;
        for (i, b) in [0xFFu8, 0xD8, 0xFF, 0xD9].into_iter().enumerate() {
            dma_buf[i * 4..i * 4 + 4].copy_from_slice(&DmaElement::new(b, 0).0);
        }
        let mut jpeg_out = [0u8; 1024];
        let mut hooks = crate::dma_drain::NoCaptureHooks;
        let r = cam.capture_jpeg_frame_polled_with_hooks(
            &mut descriptors,
            dma_buf,
            &mut jpeg_out,
            16,
            &mut hooks,
        );
        assert_eq!(r, Ok(4));
        assert!(!cam.is_running());
    }

    #[test]
    fn test_start_clears_stale_inlink_status_latches() {
        let mut cam = Esp32I2sCamera::new();
        let stale = I2S_IN_SUC_EOF_INT_BIT | I2S_IN_DSCR_ERR_INT_RAW | I2S_IN_DSCR_EMPTY_INT_RAW;
        cam.regs.int_raw = stale | 1;
        let dma_cfg = calc_dma_sizes(
            true,
            I2sSamplingMode::Sm0A000B00,
            1280,
            720,
            1,
            DEFAULT_CAMERA_DMA_BUFFER_SIZE_MAX,
        )
        .expect("dma sizes");
        assert!(cam.simulate_start(&dma_cfg));
        assert_eq!(cam.regs().int_raw, 1, "only the inlink latches are cleared");
        assert_eq!(cam.regs().int_clr, stale);
    }

    /// JPEG geometry for a ring of `desc_cnt` 2048-byte nodes, two per half.
    fn jpeg_ring_cfg(desc_cnt: usize) -> DmaSizeConfig {
        DmaSizeConfig {
            dma_bytes_per_item: 4,
            dma_buffer_size: desc_cnt * JPEG_DMA_NODE_SIZE,
            dma_half_buffer_size: JPEG_DMA_HALF_BUFFER_SIZE,
            dma_half_buffer_cnt: desc_cnt / 2,
            dma_node_buffer_size: JPEG_DMA_NODE_SIZE,
            dma_node_cnt: desc_cnt,
            lines_per_node: 1,
            nodes_per_line: 1,
            lines_per_half_buffer: 1,
            image_size: 1024,
        }
    }

    #[test]
    fn take_hands_out_i2s0_once_until_the_owner_drops() {
        let first = Esp32I2sCamera::take().expect("I2S0 is free");
        assert!(
            Esp32I2sCamera::take().is_none(),
            "a second owner of I2S0 must not exist"
        );
        drop(first);
        let again = Esp32I2sCamera::take().expect("released on drop");
        drop(again);
    }

    static START_RING: DmaRingStorage<4, { 4 * JPEG_DMA_NODE_SIZE }> = DmaRingStorage::new();

    #[test]
    fn running_capture_starts_on_the_ring_and_drop_stops_it() {
        let mut ring = START_RING
            .take_ring(JPEG_DMA_NODE_SIZE)
            .expect("storage is free");
        assert!(
            START_RING.take_ring(JPEG_DMA_NODE_SIZE).is_err(),
            "ring storage is handed out once"
        );
        let first = ring.first_desc_addr();
        let mut cam = Esp32I2sCamera::new();
        assert_eq!(cam.config(), Ok(()));
        let cfg = jpeg_ring_cfg(4);
        {
            let running = cam.start(&cfg, &mut ring).expect("start");
            assert!(running.is_running());
            assert_eq!(running.camera().regs().rx_eof_num, 1024);
            assert_eq!(
                running.camera().regs().in_link & I2S_INLINK_ADDR_MASK,
                first
            );
            assert_ne!(running.camera().regs().in_link & I2S_INLINK_START_BIT, 0);
        }
        // Drop of the guard stopped and reset the DMA.
        assert!(!cam.is_running());
        assert_ne!(cam.regs().in_link & I2S_INLINK_STOP_BIT, 0);
        assert_eq!(cam.regs().conf & I2S_RX_START_BIT, 0);
        assert_eq!(cam.dma_resets(), 1);

        // The ring is usable again, and can be restarted.
        assert!(ring.parts_mut().is_ok());
        let running = cam.start(&cfg, &mut ring).expect("restart");
        running.stop();
        assert!(!cam.is_running());
        assert_eq!(cam.dma_resets(), 2);
    }

    static GEOMETRY_RING: DmaRingStorage<4, { 4 * JPEG_DMA_NODE_SIZE }> = DmaRingStorage::new();

    #[test]
    fn start_rejects_geometry_the_ring_cannot_hold() {
        let mut ring = GEOMETRY_RING.take_ring(JPEG_DMA_NODE_SIZE).expect("ring");
        let mut cam = Esp32I2sCamera::new();
        for half in [
            0,
            3000,
            JPEG_DMA_NODE_SIZE / 2,
            3 * JPEG_DMA_NODE_SIZE,
            8 * JPEG_DMA_NODE_SIZE,
        ] {
            let cfg = DmaSizeConfig {
                dma_half_buffer_size: half,
                ..jpeg_ring_cfg(4)
            };
            assert!(
                matches!(cam.start(&cfg, &mut ring), Err(CameraError::InvalidArg)),
                "half buffer {half}"
            );
            assert!(!cam.is_running());
        }
        // A camera started through the raw path cannot be started again.
        assert!(cam.simulate_start(&jpeg_ring_cfg(4)));
        assert!(matches!(
            cam.start(&jpeg_ring_cfg(4), &mut ring),
            Err(CameraError::InvalidState)
        ));
        assert!(cam.stop());
    }

    static DRAIN_RING: DmaRingStorage<4, { 4 * JPEG_DMA_NODE_SIZE }> = DmaRingStorage::new();

    #[test]
    fn running_capture_drains_halves_and_the_tail_after_stop() {
        use crate::dma_drain::{DrainGate, HalfDrain};
        use core::ops::ControlFlow;

        let mut ring = DRAIN_RING.take_ring(JPEG_DMA_NODE_SIZE).expect("ring");
        ring.poison().expect("idle ring");
        let mut cam = Esp32I2sCamera::new();
        let mut drain = HalfDrain::new(
            2,
            JPEG_DMA_NODE_SIZE,
            cam.filter(),
            DrainGate::OwnerWriteback,
        )
        .expect("drain");
        let mut scratch = [0u8; JPEG_UNPACKED_HALF_BUFFER_SIZE];
        let mut seen = [0u8; 2];
        {
            let mut running = cam.start(&jpeg_ring_cfg(4), &mut ring).expect("start");
            // Simulate the DMA: write half 0, write back both descriptors
            // and raise IN_SUC_EOF at descriptor 1; half 1 is partial.
            running.simulate_dma_write(0, &DmaElement::new(0x11, 0).0);
            running.simulate_dma_write(JPEG_DMA_HALF_BUFFER_SIZE, &DmaElement::new(0x22, 0).0);
            running.simulate_half_complete(0);

            let mut n = 0usize;
            let r = running.drain_ready_halves(&mut drain, &mut scratch, |c| {
                seen[n] = c[0];
                n += 1;
                ControlFlow::Continue(())
            });
            assert_eq!(r, Ok(1));
            assert_eq!(seen[0], 0x11);
            // Half 1 is still DMA-owned: nothing more is consumed.
            assert_eq!(
                running.drain_ready_halves(&mut drain, &mut scratch, |_| ControlFlow::Continue(())),
                Ok(0)
            );
            running.stop();
        }
        // WP-06 order: stop first, then the tail half is drained.
        assert!(!cam.is_running());
        let (desc, buf) = ring.parts_mut().expect("stopped ring");
        let n = drain
            .drain_tail_half(desc, buf, &mut cam.dma_status(), &mut scratch)
            .expect("tail");
        assert_eq!(n, JPEG_UNPACKED_HALF_BUFFER_SIZE);
        assert_eq!(scratch[0], 0x22);
    }

    static LEAK_RING: DmaRingStorage<4, { 4 * JPEG_DMA_NODE_SIZE }> = DmaRingStorage::new();

    #[test]
    fn a_leaked_guard_leaves_the_ring_locked() {
        let mut ring = LEAK_RING.take_ring(JPEG_DMA_NODE_SIZE).expect("ring");
        let mut cam = Esp32I2sCamera::new();
        let running = cam.start(&jpeg_ring_cfg(4), &mut ring).expect("start");
        core::mem::forget(running);
        // The DMA may still be writing: the CPU gets no access to the ring.
        assert!(matches!(ring.parts_mut(), Err(CameraError::InvalidState)));
        assert!(matches!(ring.descriptors(), Err(CameraError::InvalidState)));
        assert!(cam.is_running());
        assert!(matches!(
            cam.start(&jpeg_ring_cfg(4), &mut ring),
            Err(CameraError::InvalidState)
        ));
    }
}
