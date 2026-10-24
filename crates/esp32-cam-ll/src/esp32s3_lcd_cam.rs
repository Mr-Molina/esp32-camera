//! ESP32-S3 `LCD_CAM` + `GDMA` camera parallel interface low-level driver (`target/esp32s3/ll_cam.c`).
//!
//! Models the `LCD_CAM` camera peripheral (`0x6004_1000`), `GDMA` RX channel (`0x6003_F000`),
//! integrated `160 MHz` `CAM_CLK` divider (`xclk_timer_conf`), and hardware YUV/RGB color converter.

use core::ptr;
use esp32_cam_types::{CameraConvMode, CameraError, CameraPid, PixelFormat};

use crate::dma_ring::{calc_rgb_dma_sizes, DmaSizeConfig, LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE};
use crate::esp32_i2s::{CamPinConfig, GpioMatrixBinding};

/// Physical base address of `LCD_CAM` peripheral on ESP32-S3 (`0x6004_1000`).
pub const ESP32S3_LCD_CAM_BASE_ADDR: usize = 0x6004_1000;

/// Physical base address of `GDMA` peripheral on ESP32-S3 (`0x6003_F000`).
pub const ESP32S3_GDMA_BASE_ADDR: usize = 0x6003_F000;

/// Maximum valid GPIO number on ESP32-S3 (`GPIO_NUM_MAX = 49`, valid indices `0..=48`).
pub const ESP32S3_GPIO_NUM_MAX: u8 = 49;

/// ESP32-S3 GPIO matrix input/output signal indices for `LCD_CAM` (`soc/gpio_sig_map.h`).
pub const CAM_PCLK_IDX: u8 = 133;
pub const CAM_H_ENABLE_IDX: u8 = 134;
pub const CAM_H_SYNC_IDX: u8 = 135;
pub const CAM_V_SYNC_IDX: u8 = 136;
pub const CAM_DATA_IN0_IDX: u8 = 137;
pub const CAM_CLK_IDX: u8 = 133;

/// GDMA peripheral selector value for `LCD_CAM` (`GDMA.channel[n].in.peri_sel.sel = 5`).
pub const GDMA_PERI_SEL_LCD_CAM: u8 = 5;

/// Simulated / shadow MMIO register block for `LCD_CAM` and `GDMA` on ESP32-S3.
#[repr(C, align(4))]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Esp32S3LcdCamRegisterBlock {
    pub cam_ctrl: u32,
    pub cam_ctrl1: u32,
    pub cam_rgb_yuv: u32,
    pub lc_dma_int_ena: u32,
    pub lc_dma_int_st: u32,
    pub lc_dma_int_clr: u32,
    pub gdma_in_conf0: u32,
    pub gdma_in_conf1: u32,
    pub gdma_in_int_ena: u32,
    pub gdma_in_int_clr: u32,
    pub gdma_in_link: u32,
    pub gdma_in_peri_sel: u32,
}

#[inline]
fn volatile_write_u32(reg: &mut u32, val: u32) {
    let ptr = ptr::from_mut(reg);
    // SAFETY:
    // 1. Alignment & Non-null: `ptr` is obtained via `ptr::from_mut` from a valid `&mut u32` reference.
    // 2. Memory layout: `u32` word inside `#[repr(C, align(4))]`.
    // 3. Provenance & Lifetime: Valid mutable provenance for this call.
    // 4. Exclusive aliasing: `reg` is uniquely borrowed.
    // 5. Volatile semantics: `write_volatile` prevents compiler elision or reordering.
    // 6. Panic safety: Cannot unwind.
    unsafe {
        ptr::write_volatile(ptr, val);
    }
}

#[inline]
fn volatile_read_u32(reg: &u32) -> u32 {
    let ptr = ptr::from_ref(reg);
    // SAFETY:
    // 1. Alignment & Non-null: `ptr` is obtained via `ptr::from_ref` from a valid `&u32` reference.
    // 2. Memory layout: `u32` word inside `#[repr(C, align(4))]`.
    // 3. Provenance & Lifetime: Valid shared provenance for this call.
    // 4. Aliasing: No concurrent mutable borrow exists.
    // 5. Volatile semantics: `read_volatile` prevents read elision.
    // 6. Panic safety: Cannot unwind.
    unsafe { ptr::read_volatile(ptr) }
}

/// Low-level ESP32-S3 `LCD_CAM` + `GDMA` camera peripheral controller (`target/esp32s3/ll_cam.c`).
#[derive(Debug, Clone)]
pub struct Esp32S3LcdCam {
    regs: Esp32S3LcdCamRegisterBlock,
    dma_num: u8,
    psram_mode: bool,
    jpeg_mode: bool,
    swap_data: bool,
    conv_mode: CameraConvMode,
    in_bytes_per_pixel: u8,
    fb_bytes_per_pixel: u8,
    gpio_bindings: [GpioMatrixBinding; 12],
    gpio_binding_count: usize,
    running: bool,
}

impl Default for Esp32S3LcdCam {
    fn default() -> Self {
        Self::new()
    }
}

impl Esp32S3LcdCam {
    /// Creates an uninitialized [`Esp32S3LcdCam`] controller.
    #[inline]
    pub const fn new() -> Self {
        Self {
            regs: Esp32S3LcdCamRegisterBlock {
                cam_ctrl: 0,
                cam_ctrl1: 0,
                cam_rgb_yuv: 0,
                lc_dma_int_ena: 0,
                lc_dma_int_st: 0,
                lc_dma_int_clr: 0,
                gdma_in_conf0: 0,
                gdma_in_conf1: 0,
                gdma_in_int_ena: 0,
                gdma_in_int_clr: 0,
                gdma_in_link: 0,
                gdma_in_peri_sel: 0,
            },
            dma_num: 0,
            psram_mode: false,
            jpeg_mode: false,
            swap_data: false,
            conv_mode: CameraConvMode::Disable,
            in_bytes_per_pixel: 1,
            fb_bytes_per_pixel: 1,
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
    pub const fn regs(&self) -> &Esp32S3LcdCamRegisterBlock {
        &self.regs
    }

    /// Returns the active GDMA RX channel number (`dma_num`).
    #[inline]
    pub const fn dma_num(&self) -> u8 {
        self.dma_num
    }

    /// Sets the active GDMA RX channel number (`dma_num`).
    #[inline]
    pub fn set_dma_num(&mut self, dma_num: u8) {
        self.dma_num = dma_num;
    }

    /// Returns `true` if byte swapping (`cam_byte_order`) is enabled.
    #[inline]
    pub const fn swap_data(&self) -> bool {
        self.swap_data
    }

    /// Sets byte swapping (`cam_byte_order`).
    #[inline]
    pub fn set_swap_data(&mut self, swap_data: bool) {
        self.swap_data = swap_data;
    }

    /// Returns the required PSRAM DMA alignment (`16 << in_ext_mem_bk_size`, `target/esp32s3/ll_cam.c:457-460`).
    #[inline]
    pub fn dma_align(&self) -> u8 {
        let bk_size = (volatile_read_u32(&self.regs.gdma_in_conf1) >> 12) & 0x3;
        16u8 << bk_size
    }

    /// Resets and configures the GDMA RX channel (`ll_cam_dma_reset` in `target/esp32s3/ll_cam.c:73-96`).
    pub fn dma_reset(&mut self) {
        volatile_write_u32(&mut self.regs.gdma_in_int_clr, u32::MAX);
        volatile_write_u32(&mut self.regs.gdma_in_int_ena, 0);

        let burst_bits = if !self.psram_mode {
            (1 << 2) | (1 << 3) // `indscr_burst_en = 1, in_data_burst_en = 1`
        } else {
            0
        };
        volatile_write_u32(&mut self.regs.gdma_in_conf0, burst_bits);
        volatile_write_u32(&mut self.regs.gdma_in_conf1, 0);
        volatile_write_u32(
            &mut self.regs.gdma_in_peri_sel,
            GDMA_PERI_SEL_LCD_CAM as u32,
        );
    }

    /// Configures GPIO matrix routing for ESP32-S3 `LCD_CAM` (`ll_cam_set_pin` in `target/esp32s3/ll_cam.c:386-420`).
    pub fn set_pin(&mut self, cfg: &CamPinConfig) -> Result<(), CameraError> {
        if cfg.pin_vsync < 0
            || (cfg.pin_vsync as u8) >= ESP32S3_GPIO_NUM_MAX
            || cfg.pin_pclk < 0
            || (cfg.pin_pclk as u8) >= ESP32S3_GPIO_NUM_MAX
            || cfg.pin_href < 0
            || (cfg.pin_href as u8) >= ESP32S3_GPIO_NUM_MAX
            || cfg.pin_xclk >= (ESP32S3_GPIO_NUM_MAX as i8)
        {
            return Err(CameraError::InvalidArg);
        }

        let mut i = 0;
        while i < 8 {
            let pin = cfg.data_pins[i];
            if pin < 0 || (pin as u8) >= ESP32S3_GPIO_NUM_MAX {
                return Err(CameraError::InvalidArg);
            }
            i += 1;
        }

        self.gpio_bindings[0] = GpioMatrixBinding {
            gpio_num: cfg.pin_pclk as u8,
            signal_idx: CAM_PCLK_IDX,
            inverted: false,
        };
        self.gpio_bindings[1] = GpioMatrixBinding {
            gpio_num: cfg.pin_vsync as u8,
            signal_idx: CAM_V_SYNC_IDX,
            inverted: cfg.vsync_invert,
        };
        self.gpio_bindings[2] = GpioMatrixBinding {
            gpio_num: cfg.pin_href as u8,
            signal_idx: CAM_H_ENABLE_IDX,
            inverted: false,
        };

        let mut d = 0;
        while d < 8 {
            self.gpio_bindings[3 + d] = GpioMatrixBinding {
                gpio_num: cfg.data_pins[d] as u8,
                signal_idx: CAM_DATA_IN0_IDX + (d as u8),
                inverted: false,
            };
            d += 1;
        }

        if cfg.pin_xclk >= 0 {
            self.gpio_bindings[11] = GpioMatrixBinding {
                gpio_num: cfg.pin_xclk as u8,
                signal_idx: CAM_CLK_IDX,
                inverted: false,
            };
            self.gpio_binding_count = 12;
        } else {
            self.gpio_binding_count = 11;
        }

        Ok(())
    }

    /// Configures the `LCD_CAM` peripheral, clock divider, optional color converter, and GDMA channel
    /// (`ll_cam_config` in `target/esp32s3/ll_cam.c:315-374`).
    pub fn config(
        &mut self,
        xclk_freq_hz: u32,
        pixel_format: PixelFormat,
        conv_mode: CameraConvMode,
    ) -> Result<(), CameraError> {
        let div_num = 160_000_000u32.checked_div(xclk_freq_hz).unwrap_or(0) & 0xFF;
        let swap_bit = u32::from(self.swap_data) << 15;
        // `cam_clkm_div_num` (bits 0..7), `cam_clk_sel = 3` (bits 21..22), `cam_vsync_filter_thres = 4` (bits 11..13), `cam_update = 1` (bit 14)
        let ctrl_val = div_num | (4 << 11) | (1 << 14) | swap_bit | (3 << 21);
        volatile_write_u32(&mut self.regs.cam_ctrl, ctrl_val);

        // `cam_rec_data_bytelen = LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE - 1`, `cam_vsync_filter_en = 1 (bit 22)`, `cam_start = 1 (bit 29)`
        let ctrl1_val =
            ((LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE as u32 - 1) & 0xFFFF) | (1 << 22) | (1 << 29);
        volatile_write_u32(&mut self.regs.cam_ctrl1, ctrl1_val);

        volatile_write_u32(&mut self.regs.cam_rgb_yuv, 0);
        if conv_mode != CameraConvMode::Disable {
            match conv_mode {
                CameraConvMode::Yuv422ToYuv420 => {
                    if pixel_format != PixelFormat::Yuv422 {
                        return Err(CameraError::InvalidFormat);
                    }
                    volatile_write_u32(&mut self.regs.cam_rgb_yuv, 1 | (1 << 4) | (1 << 30) | (1 << 31));
                }
                CameraConvMode::Yuv422ToRgb565 => {
                    if pixel_format != PixelFormat::Yuv422 {
                        return Err(CameraError::InvalidFormat);
                    }
                    volatile_write_u32(&mut self.regs.cam_rgb_yuv, 3 | (1 << 30) | (1 << 31));
                }
                _ => return Err(CameraError::NotSupported),
            }
        }
        self.conv_mode = conv_mode;

        self.dma_reset();
        Ok(())
    }

    /// Dynamically updates the `LCD_CAM` `XCLK` divider (`xclk_timer_conf` in `target/esp32s3/ll_cam.c:617-627`).
    pub fn xclk_timer_conf(&mut self, xclk_freq_hz: u32) -> Result<(), CameraError> {
        if xclk_freq_hz == 0 || xclk_freq_hz > 40_000_000 {
            return Err(CameraError::InvalidArg);
        }
        let div_num = (160_000_000 / xclk_freq_hz) & 0xFF;
        let prev = volatile_read_u32(&self.regs.cam_ctrl) & !0xFF;
        volatile_write_u32(&mut self.regs.cam_ctrl, prev | div_num | (3 << 21) | (1 << 14));
        Ok(())
    }

    /// Configures byte-per-pixel ratios on ESP32-S3 (`ll_cam_set_sample_mode` in `target/esp32s3/ll_cam.c:580-614`).
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
                    || sensor_pid == CameraPid::GC0308.as_u16()
                    || sensor_pid == CameraPid::HM0360.as_u16();
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

    /// Computes DMA buffer geometry on ESP32-S3 (`ll_cam_dma_sizes` in `target/esp32s3/ll_cam.c:538-557`).
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

    /// Enables or disables the `LCD_CAM` VSYNC interrupt (`ll_cam_vsync_intr_enable` in `target/esp32s3/ll_cam.c:376-384`).
    pub fn vsync_intr_enable(&mut self, en: bool) {
        volatile_write_u32(&mut self.regs.lc_dma_int_clr, 1);
        volatile_write_u32(&mut self.regs.lc_dma_int_ena, u32::from(en));
    }

    /// Starts `LCD_CAM` + `GDMA` frame reception (`ll_cam_start` in `target/esp32s3/ll_cam.c:152-181`).
    pub fn start(&mut self, dma_cfg: &DmaSizeConfig, first_desc_addr: u32) -> bool {
        if dma_cfg.dma_half_buffer_size == 0 {
            return false;
        }
        if self.jpeg_mode || !self.psram_mode {
            volatile_write_u32(&mut self.regs.gdma_in_int_clr, 1);
            volatile_write_u32(&mut self.regs.gdma_in_int_ena, 1);
        }

        let rec_len = ((dma_cfg.dma_half_buffer_size - 1) as u32) & 0xFFFF;
        volatile_write_u32(
            &mut self.regs.cam_ctrl1,
            rec_len | (1 << 22) | (1 << 29),
        );
        volatile_write_u32(
            &mut self.regs.gdma_in_link,
            (first_desc_addr & 0x000F_FFFF) | (1 << 29),
        );
        self.running = true;
        true
    }

    /// Stops `LCD_CAM` + `GDMA` frame reception (`ll_cam_stop` in `target/esp32s3/ll_cam.c:142-150`).
    pub fn stop(&mut self) -> bool {
        if self.jpeg_mode || !self.psram_mode {
            volatile_write_u32(&mut self.regs.gdma_in_int_ena, 0);
            volatile_write_u32(&mut self.regs.gdma_in_int_clr, 1);
        }
        let link = (volatile_read_u32(&self.regs.gdma_in_link) & !(1 << 29)) | (1 << 28);
        volatile_write_u32(&mut self.regs.gdma_in_link, link);
        self.running = false;
        true
    }

    /// Deinitializes `LCD_CAM` + `GDMA` (`ll_cam_deinit` in `target/esp32s3/ll_cam.c:183-204`).
    pub fn deinit(&mut self) -> Result<(), CameraError> {
        let _ = self.stop();
        volatile_write_u32(&mut self.regs.cam_ctrl1, 0);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dma_ring::DEFAULT_CAMERA_DMA_BUFFER_SIZE_MAX;

    #[test]
    fn test_esp32s3_lcd_cam_config_and_lifecycle() {
        let mut s3 = Esp32S3LcdCam::new();
        let pins = CamPinConfig {
            pin_vsync: 38,
            pin_href: 47,
            pin_pclk: 13,
            pin_xclk: 10,
            data_pins: [15, 17, 18, 16, 14, 12, 11, 48],
            vsync_invert: true,
        };
        assert_eq!(s3.set_pin(&pins), Ok(()));
        assert_eq!(s3.dma_align(), 16);

        assert_eq!(
            s3.config(20_000_000, PixelFormat::Yuv422, CameraConvMode::Yuv422ToRgb565),
            Ok(())
        );
        // 160MHz / 20MHz = 8
        assert_eq!(s3.regs().cam_ctrl & 0xFF, 8);
        assert_eq!(s3.regs().gdma_in_peri_sel, GDMA_PERI_SEL_LCD_CAM as u32);

        // Reject invalid color converter input format
        assert_eq!(
            s3.config(20_000_000, PixelFormat::Jpeg, CameraConvMode::Yuv422ToRgb565),
            Err(CameraError::InvalidFormat)
        );

        // HM0360 native Y8 grayscale on S3
        assert_eq!(
            s3.set_sample_mode(PixelFormat::Grayscale, CameraPid::HM0360.as_u16()),
            Ok((1, 1))
        );

        let dma_cfg = s3
            .dma_sizes(320, 240, 320 * 240, DEFAULT_CAMERA_DMA_BUFFER_SIZE_MAX)
            .expect("valid s3 dma sizes");
        s3.vsync_intr_enable(true);
        assert_eq!(s3.regs().lc_dma_int_ena, 1);

        assert!(s3.start(&dma_cfg, 0x3FC8_9ABC));
        assert!(s3.stop());
        assert_eq!(s3.deinit(), Ok(()));
    }
}
