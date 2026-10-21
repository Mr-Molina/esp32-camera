//! ESP32 I2S0 camera parallel interface low-level driver (`target/esp32/ll_cam.c`).
//!
//! Provides MMIO register constants, GPIO matrix signal indices, and the [`Esp32I2sCamera`]
//! controller implementing `config`, `set_pin`, `start`, `stop`, `deinit`, `vsync_intr_enable`,
//! and `set_sample_mode`.

use core::ptr;
use esp32_cam_types::{CameraError, PixelFormat};

use crate::dma_filter::{select_sample_mode, DmaFilterKind, I2sSamplingMode, SampleModeConfig};
use crate::dma_ring::DmaSizeConfig;

/// Physical base address of the `I2S0` peripheral on ESP32 (`0x3FF4_F000`).
pub const I2S0_BASE_ADDR: usize = 0x3FF4_F000;

/// Physical address of `DPORT_PERIP_CLK_EN_REG` on ESP32 (`0x3FF0_00C0`).
pub const DPORT_PERIP_CLK_EN_REG: usize = 0x3FF0_00C0;

/// Physical address of `DPORT_PERIP_RST_EN_REG` on ESP32 (`0x3FF0_00C4`).
pub const DPORT_PERIP_RST_EN_REG: usize = 0x3FF0_00C4;

/// Bit mask for `DPORT_I2S0_CLK_EN` (`bit 4`).
pub const DPORT_I2S0_CLK_EN: u32 = 1 << 4;

/// Bit mask for `DPORT_I2S0_RST` (`bit 4`).
pub const DPORT_I2S0_RST: u32 = 1 << 4;

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

/// GPIO matrix input signal index for `I2S0I_WS_IN_IDX` (`PCLK`, `158`).
pub const I2S0I_WS_IN_IDX: u8 = 158;

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

/// `I2S0.int_ena` (`0x14`) and `I2S0.int_clr` (`0x18`) bit definitions.
pub const I2S_INT_ENA_REG_OFFSET: usize = 0x14;
pub const I2S_INT_CLR_REG_OFFSET: usize = 0x18;
pub const I2S_IN_SUC_EOF_INT_BIT: u32 = 1 << 9;

/// `I2S0.in_link` (`0x34`) bit definitions.
pub const I2S_IN_LINK_REG_OFFSET: usize = 0x34;
pub const I2S_INLINK_ADDR_MASK: u32 = 0x000F_FFFF;
pub const I2S_INLINK_STOP_BIT: u32 = 1 << 28;
pub const I2S_INLINK_START_BIT: u32 = 1 << 29;

/// `I2S0.conf2` (`0xA8`) bit definitions.
pub const I2S_CAMERA_EN_BIT: u32 = 1 << 0;
pub const I2S_LCD_EN_BIT: u32 = 1 << 5;

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
    // 2. Memory layout: ESP32 DPORT and I2S0 peripheral registers are 32-bit memory-mapped words.
    // 3. Provenance: Hardware peripheral address space on Xtensa ESP32 target.
    // 4. Aliasing: Executed under peripheral ownership of `Esp32I2sCamera`.
    // 5. Volatile semantics: `write_volatile` is required for hardware MMIO side effects.
    // 6. Panic safety: Hardware MMIO store does not unwind.
    unsafe {
        ptr::write_volatile(ptr, val);
    }
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
            xtensa_mmio_write(DPORT_PERIP_CLK_EN_REG, self.regs.dport_clk_en);
            xtensa_mmio_write(DPORT_PERIP_RST_EN_REG, self.regs.dport_rst_en);
            xtensa_mmio_write(I2S0_BASE_ADDR + I2S_CONF_REG_OFFSET, self.regs.conf);
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
            | ((self.sampling_mode as u32) << 16);
        volatile_write_u32(&mut self.regs.fifo_conf, fifo_val);
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

        Ok(())
    }

    /// Unpacks a raw DMA half-buffer `src` into `dst` using the active DMA filter (`ll_cam_memcpy`).
    #[inline]
    pub fn memcpy_filtered(&self, dst: &mut [u8], src: &[u8]) -> usize {
        self.filter.apply(dst, src)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dma_ring::{calc_dma_sizes, DEFAULT_CAMERA_DMA_BUFFER_SIZE_MAX};
    use esp32_cam_types::CameraPid;

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
        assert_eq!(cam.gpio_bindings()[0].signal_idx, I2S0I_WS_IN_IDX);
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
}
