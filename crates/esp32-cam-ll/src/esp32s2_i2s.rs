//! ESP32-S2 I2S0 camera parallel interface low-level driver (`target/esp32s2/ll_cam.c`).
//!
//! Supports 1-byte-per-item DMA packing (`rx_bits_mod = 8`, `rx_fifo_mod = 2`), high-bit GPIO
//! matrix data alignment (`I2S0I_DATA_IN0_IDX + 8 + i`), 64-byte PSRAM DMA alignment, and
//! YUV-to-Grayscale decimation (`ll_cam_memcpy`).

use core::ptr;
use esp32_cam_types::{CameraError, CameraPid, PixelFormat};

use crate::dma_ring::{calc_rgb_dma_sizes, DmaSizeConfig};
use crate::esp32_i2s::{
    CamPinConfig, GpioMatrixBinding, GPIO_MATRIX_CONST_ONE_INPUT, I2S0I_DATA_IN0_IDX,
    I2S0I_H_ENABLE_IDX, I2S0I_H_SYNC_IDX, I2S0I_V_SYNC_IDX, I2S0I_WS_IN_IDX,
    I2S_CAMERA_EN_BIT, I2S_INLINK_ADDR_MASK, I2S_INLINK_START_BIT, I2S_INLINK_STOP_BIT,
    I2S_IN_SUC_EOF_INT_BIT, I2S_LCD_EN_BIT, I2S_RX_START_BIT,
};

/// Maximum valid GPIO number on ESP32-S2 (`GPIO_NUM_MAX = 47`, valid indices `0..=46`).
pub const ESP32S2_GPIO_NUM_MAX: u8 = 47;

/// Required PSRAM DMA alignment on ESP32-S2 (`64` bytes, `target/esp32s2/ll_cam.c:280-283`).
pub const ESP32S2_DMA_ALIGN: u8 = 64;

/// Simulated / shadow MMIO register block for `I2S0` on ESP32-S2.
#[repr(C, align(4))]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Esp32S2I2sRegisterBlock {
    pub clkm_conf: u32,
    pub conf: u32,
    pub fifo_conf: u32,
    pub lc_conf: u32,
    pub timing: u32,
    pub int_ena: u32,
    pub int_clr: u32,
    pub conf2: u32,
    pub sample_rate_conf: u32,
    pub conf_chan: u32,
    pub rx_eof_num: u32,
    pub in_link: u32,
}

#[inline]
fn volatile_write_u32(reg: &mut u32, val: u32) {
    let ptr = ptr::from_mut(reg);
    // SAFETY:
    // 1. Alignment & Non-null: `ptr` is derived from a valid `&mut u32` reference (non-null, 4-byte aligned).
    // 2. Memory layout: `u32` register word in `#[repr(C, align(4))]`.
    // 3. Provenance & Lifetime: Valid mutable provenance for the duration of this call.
    // 4. Exclusive aliasing: `reg` is uniquely borrowed.
    // 5. Volatile semantics: `write_volatile` prevents register write elision.
    // 6. Panic safety: Cannot unwind.
    unsafe {
        ptr::write_volatile(ptr, val);
    }
}

#[inline]
fn volatile_read_u32(reg: &u32) -> u32 {
    let ptr = ptr::from_ref(reg);
    // SAFETY:
    // 1. Alignment & Non-null: `ptr` is derived from a valid `&u32` reference (non-null, 4-byte aligned).
    // 2. Memory layout: `u32` register word in `#[repr(C, align(4))]`.
    // 3. Provenance & Lifetime: Valid shared provenance for the duration of this call.
    // 4. Aliasing: No concurrent mutable access exists.
    // 5. Volatile semantics: `read_volatile` prevents read elision.
    // 6. Panic safety: Cannot unwind.
    unsafe { ptr::read_volatile(ptr) }
}

/// Low-level ESP32-S2 I2S0 camera peripheral controller (`target/esp32s2/ll_cam.c`).
#[derive(Debug, Clone)]
pub struct Esp32S2I2sCamera {
    regs: Esp32S2I2sRegisterBlock,
    in_bytes_per_pixel: u8,
    fb_bytes_per_pixel: u8,
    jpeg_mode: bool,
    psram_mode: bool,
    swap_data: bool,
    vsync_pin: Option<u8>,
    vsync_invert: bool,
    vsync_intr_enabled: bool,
    gpio_bindings: [GpioMatrixBinding; 12],
    gpio_binding_count: usize,
    running: bool,
}

impl Default for Esp32S2I2sCamera {
    fn default() -> Self {
        Self::new()
    }
}

impl Esp32S2I2sCamera {
    /// Creates an uninitialized [`Esp32S2I2sCamera`] instance.
    #[inline]
    pub const fn new() -> Self {
        Self {
            regs: Esp32S2I2sRegisterBlock {
                clkm_conf: 0,
                conf: 0,
                fifo_conf: 0,
                lc_conf: 0,
                timing: 0,
                int_ena: 0,
                int_clr: 0,
                conf2: 0,
                sample_rate_conf: 0,
                conf_chan: 0,
                rx_eof_num: 0,
                in_link: 0,
            },
            in_bytes_per_pixel: 1,
            fb_bytes_per_pixel: 1,
            jpeg_mode: false,
            psram_mode: false,
            swap_data: false,
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

    /// Returns a shared reference to the register block.
    #[inline]
    pub const fn regs(&self) -> &Esp32S2I2sRegisterBlock {
        &self.regs
    }

    /// Returns the PSRAM DMA alignment requirement on ESP32-S2 (`64` bytes).
    #[inline]
    pub const fn dma_align(&self) -> u8 {
        ESP32S2_DMA_ALIGN
    }

    /// Enables or disables PSRAM direct-DMA mode.
    #[inline]
    pub fn set_psram_mode(&mut self, psram_mode: bool) {
        self.psram_mode = psram_mode;
    }

    /// Returns the configured GPIO matrix bindings.
    #[inline]
    pub fn gpio_bindings(&self) -> &[GpioMatrixBinding] {
        &self.gpio_bindings[..self.gpio_binding_count]
    }

    /// Configures GPIO matrix routing on ESP32-S2 (`target/esp32s2/ll_cam.c:221-264`).
    ///
    /// Note that on ESP32-S2 `D0..D7` are mapped to `I2S0I_DATA_IN0_IDX + 8 + i` (`148..=155`)
    /// for 8-bit high-bit alignment.
    pub fn set_pin(&mut self, cfg: &CamPinConfig) -> Result<(), CameraError> {
        if cfg.pin_vsync < 0
            || (cfg.pin_vsync as u8) >= ESP32S2_GPIO_NUM_MAX
            || cfg.pin_pclk < 0
            || (cfg.pin_pclk as u8) >= ESP32S2_GPIO_NUM_MAX
            || cfg.pin_href < 0
            || (cfg.pin_href as u8) >= ESP32S2_GPIO_NUM_MAX
        {
            return Err(CameraError::InvalidArg);
        }

        let mut i = 0;
        while i < 8 {
            let pin = cfg.data_pins[i];
            if pin < 0 || (pin as u8) >= ESP32S2_GPIO_NUM_MAX {
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
                signal_idx: I2S0I_DATA_IN0_IDX + 8 + (d as u8),
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

    /// Configures ESP32-S2 `I2S0` registers (`ll_cam_config` in `target/esp32s2/ll_cam.c:138-210`).
    pub fn config(&mut self) -> Result<(), CameraError> {
        // `clkm_div_num = 2, clk_sel = 2, clk_en = 1`
        volatile_write_u32(&mut self.regs.clkm_conf, 2 | (2 << 21) | (1 << 20));
        volatile_write_u32(&mut self.regs.timing, 0);
        volatile_write_u32(&mut self.regs.int_ena, 0);
        volatile_write_u32(&mut self.regs.int_clr, u32::MAX);

        // `lcd_en = 1, camera_en = 1, i_v_sync_filter_en = 1, i_v_sync_filter_thres = 4`
        volatile_write_u32(
            &mut self.regs.conf2,
            I2S_LCD_EN_BIT | I2S_CAMERA_EN_BIT | (1 << 11) | (4 << 8),
        );

        // `rx_bck_div_num = 1, rx_bits_mod = 8`
        volatile_write_u32(&mut self.regs.sample_rate_conf, 1 | (8 << 18));
        volatile_write_u32(&mut self.regs.conf_chan, 1 << 3);

        // `dscr_en = 1 (bit 12), rx_fifo_mod_force_en = 1 (bit 20), rx_data_num = 32 (bits 6..11), rx_fifo_mod = 2 (bits 16..18)`
        volatile_write_u32(
            &mut self.regs.fifo_conf,
            (1 << 12) | (1 << 20) | (32 << 6) | (2 << 16),
        );

        let swap_bit = if self.swap_data { 1 << 9 } else { 0 };
        volatile_write_u32(
            &mut self.regs.conf,
            (1 << 7) | (1 << 26) | swap_bit | I2S_RX_START_BIT,
        );

        Ok(())
    }

    /// Configures byte-per-pixel ratios on ESP32-S2 (`ll_cam_set_sample_mode` in `target/esp32s2/ll_cam.c:404-424`).
    pub fn set_sample_mode(
        &mut self,
        pix_format: PixelFormat,
        sensor_pid: u16,
    ) -> Result<(u8, u8), CameraError> {
        match pix_format {
            PixelFormat::Grayscale => {
                let is_y8 = sensor_pid == CameraPid::OV3660.as_u16()
                    || sensor_pid == CameraPid::OV3640.as_u16()
                    || sensor_pid == CameraPid::OV5640.as_u16()
                    || sensor_pid == CameraPid::NT99141.as_u16()
                    || sensor_pid == CameraPid::SC031GS.as_u16()
                    || sensor_pid == CameraPid::BF20A6.as_u16()
                    || sensor_pid == CameraPid::GC0308.as_u16();
                self.in_bytes_per_pixel = if is_y8 { 1 } else { 2 };
                self.fb_bytes_per_pixel = 1;
                self.jpeg_mode = false;
            }
            PixelFormat::Yuv422 | PixelFormat::Rgb565 => {
                self.in_bytes_per_pixel = 2;
                self.fb_bytes_per_pixel = 2;
                self.jpeg_mode = false;
            }
            PixelFormat::Jpeg => {
                self.in_bytes_per_pixel = 1;
                self.fb_bytes_per_pixel = 1;
                self.jpeg_mode = true;
            }
            _ => return Err(CameraError::NotSupported),
        }
        Ok((self.in_bytes_per_pixel, self.fb_bytes_per_pixel))
    }

    /// Computes DMA buffer geometry on ESP32-S2 (`ll_cam_dma_sizes` in `target/esp32s2/ll_cam.c:362-381`).
    pub fn dma_sizes(
        &self,
        width: u16,
        height: u16,
        recv_size: usize,
        dma_buffer_size_max: usize,
    ) -> Result<DmaSizeConfig, CameraError> {
        if self.jpeg_mode {
            let dma_half_buffer_size = 1024;
            let (dma_buffer_size, dma_half_buffer_cnt) = if self.psram_mode {
                let cnt = recv_size / dma_half_buffer_size;
                if cnt == 0 {
                    return Err(CameraError::InvalidArg);
                }
                (recv_size, cnt)
            } else {
                (16 * 1024, 16)
            };
            Ok(DmaSizeConfig {
                dma_bytes_per_item: 1,
                dma_buffer_size,
                dma_half_buffer_size,
                dma_half_buffer_cnt,
                dma_node_buffer_size: dma_half_buffer_size,
                dma_node_cnt: dma_buffer_size / dma_half_buffer_size,
                lines_per_node: 1,
                nodes_per_line: 1,
                lines_per_half_buffer: 1,
                image_size: recv_size,
            })
        } else {
            calc_rgb_dma_sizes(
                width,
                height,
                self.in_bytes_per_pixel,
                1,
                dma_buffer_size_max,
            )
        }
    }

    /// Starts ESP32-S2 DMA capture (`ll_cam_start` in `target/esp32s2/ll_cam.c:107-136`).
    pub fn start(&mut self, dma_cfg: &DmaSizeConfig, first_desc_addr: u32) -> bool {
        if dma_cfg.dma_half_buffer_size == 0 {
            return false;
        }
        let conf_stopped = volatile_read_u32(&self.regs.conf) & !I2S_RX_START_BIT;
        volatile_write_u32(&mut self.regs.conf, conf_stopped);

        if self.jpeg_mode || !self.psram_mode {
            volatile_write_u32(&mut self.regs.int_clr, I2S_IN_SUC_EOF_INT_BIT);
            let int_ena = volatile_read_u32(&self.regs.int_ena) | I2S_IN_SUC_EOF_INT_BIT;
            volatile_write_u32(&mut self.regs.int_ena, int_ena);
        }

        volatile_write_u32(
            &mut self.regs.rx_eof_num,
            dma_cfg.dma_half_buffer_size as u32,
        );
        volatile_write_u32(
            &mut self.regs.in_link,
            (first_desc_addr & I2S_INLINK_ADDR_MASK) | I2S_INLINK_START_BIT,
        );
        volatile_write_u32(&mut self.regs.conf, conf_stopped | I2S_RX_START_BIT);
        self.running = true;
        true
    }

    /// Stops ESP32-S2 DMA capture (`ll_cam_stop` in `target/esp32s2/ll_cam.c:84-94`).
    pub fn stop(&mut self) -> bool {
        let conf_stopped = volatile_read_u32(&self.regs.conf) & !I2S_RX_START_BIT;
        volatile_write_u32(&mut self.regs.conf, conf_stopped);

        if self.jpeg_mode || !self.psram_mode {
            let int_ena = volatile_read_u32(&self.regs.int_ena) & !I2S_IN_SUC_EOF_INT_BIT;
            volatile_write_u32(&mut self.regs.int_ena, int_ena);
            volatile_write_u32(&mut self.regs.int_clr, I2S_IN_SUC_EOF_INT_BIT);
        }

        let in_link = (volatile_read_u32(&self.regs.in_link) & !I2S_INLINK_START_BIT)
            | I2S_INLINK_STOP_BIT;
        volatile_write_u32(&mut self.regs.in_link, in_link);
        self.running = false;
        true
    }

    /// Enables or disables VSYNC interrupt (`ll_cam_vsync_intr_enable`).
    pub fn vsync_intr_enable(&mut self, en: bool) {
        self.vsync_intr_enabled = en;
    }

    /// Deinitializes ESP32-S2 camera peripheral (`ll_cam_deinit`).
    pub fn deinit(&mut self) -> Result<(), CameraError> {
        let _ = self.stop();
        self.vsync_intr_enabled = false;
        self.vsync_pin = None;
        Ok(())
    }

    /// Copies or decimates a DMA buffer slice (`ll_cam_memcpy` in `target/esp32s2/ll_cam.c:383-402`).
    pub fn memcpy(&self, dst: &mut [u8], src: &[u8]) -> usize {
        if self.in_bytes_per_pixel == 2 && self.fb_bytes_per_pixel == 1 {
            let end = (src.len() / 8).min(dst.len() / 4);
            let mut i = 0;
            while i < end {
                let in_off = i * 8;
                let out_off = i * 4;
                dst[out_off] = src[in_off];
                dst[out_off + 1] = src[in_off + 2];
                dst[out_off + 2] = src[in_off + 4];
                dst[out_off + 3] = src[in_off + 6];
                i += 1;
            }
            end * 4
        } else {
            let n = src.len().min(dst.len());
            dst[..n].copy_from_slice(&src[..n]);
            n
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dma_ring::DEFAULT_CAMERA_DMA_BUFFER_SIZE_MAX;

    #[test]
    fn test_esp32s2_high_bit_gpio_routing_and_memcpy() {
        let mut cam = Esp32S2I2sCamera::new();
        let pins = CamPinConfig {
            pin_vsync: 6,
            pin_href: 7,
            pin_pclk: 13,
            pin_xclk: 15,
            data_pins: [11, 9, 8, 10, 12, 18, 17, 16],
            vsync_invert: true,
        };
        assert_eq!(cam.set_pin(&pins), Ok(()));
        // On S2, D0 is routed to I2S0I_DATA_IN0_IDX + 8 = 148
        assert_eq!(cam.gpio_bindings()[3].signal_idx, I2S0I_DATA_IN0_IDX + 8);
        assert_eq!(cam.gpio_bindings()[10].signal_idx, I2S0I_DATA_IN0_IDX + 15);
        assert_eq!(cam.dma_align(), 64);

        assert_eq!(cam.config(), Ok(()));
        assert_eq!(
            cam.set_sample_mode(PixelFormat::Grayscale, CameraPid::OV2640.as_u16()),
            Ok((2, 1))
        );

        let src = [10, 99, 20, 99, 30, 99, 40, 99];
        let mut dst = [0u8; 4];
        assert_eq!(cam.memcpy(&mut dst, &src), 4);
        assert_eq!(dst, [10, 20, 30, 40]);

        let dma_cfg = cam
            .dma_sizes(320, 240, 320 * 240 * 2, DEFAULT_CAMERA_DMA_BUFFER_SIZE_MAX)
            .expect("valid s2 dma config");
        assert!(cam.start(&dma_cfg, 0x3FFB_0000));
        assert!(cam.stop());
        assert_eq!(cam.deinit(), Ok(()));
    }
}
