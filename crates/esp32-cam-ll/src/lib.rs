//! Low-level `#![no_std]` DMA descriptor ring, I2S0/LCD_CAM capture filter, and XCLK configuration.

#![no_std]
#![cfg_attr(target_arch = "xtensa", feature(asm_experimental_arch))]

pub use esp32_cam_types::{CameraConvMode, CameraError, CameraPid, FrameSize, PixelFormat};

pub mod dma_filter;
pub mod dma_ring;
pub mod esp32_i2s;
pub mod esp32s2_i2s;
pub mod esp32s3_lcd_cam;
pub mod xclk;

pub use dma_filter::{
    filter_grayscale, filter_grayscale_highspeed, filter_jpeg, filter_yuyv,
    filter_yuyv_highspeed, select_sample_mode, DmaElement, DmaFilterFn, DmaFilterKind,
    I2sSamplingMode, SampleModeConfig,
};
pub use dma_ring::{
    build_circular_ring, calc_dma_sizes, calc_rgb_dma_sizes, verify_circular_ring, DmaSizeConfig,
    Lldesc, DEFAULT_CAMERA_DMA_BUFFER_SIZE_MAX, LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE,
    MAX_DMA_IMAGE_SIZE,
};
pub use esp32_i2s::{
    io_mux_reg_addr, CamPinConfig, Esp32I2sCamera, GpioMatrixBinding, I2s0RegisterBlock,
    DPORT_BASE_ADDR, DPORT_I2S0_CLK_EN, DPORT_I2S0_RST, DPORT_PERIP_CLK_EN_REG,
    DPORT_PERIP_RST_EN_REG, ESP32_GPIO_NUM_MAX, GPIO_BASE_ADDR, GPIO_ENABLE1_W1TC_REG,
    GPIO_ENABLE_W1TC_REG, GPIO_FUNC0_IN_SEL_CFG_REG, GPIO_FUNC0_OUT_SEL_CFG_REG, GPIO_IN1_REG,
    GPIO_IN_REG, GPIO_MATRIX_CONST_ONE_INPUT, GPIO_PIN0_REG, I2S0I_DATA_IN0_IDX,
    I2S0I_H_ENABLE_IDX, I2S0I_H_SYNC_IDX, I2S0I_V_SYNC_IDX, I2S0I_WS_IN_IDX, I2S0_BASE_ADDR,
    I2S_CLKM_CONF_REG_OFFSET, I2S_CONF2_REG_OFFSET, I2S_CONF_CHAN_REG_OFFSET, I2S_CONF_REG_OFFSET,
    I2S_FIFO_CONF_REG_OFFSET, I2S_INT_CLR_REG_OFFSET, I2S_INT_ENA_REG_OFFSET,
    I2S_INT_RAW_REG_OFFSET, I2S_INT_ST_REG_OFFSET, I2S_IN_EOF_DES_ADDR_REG_OFFSET,
    I2S_IN_LINK_REG_OFFSET, I2S_LC_CONF_REG_OFFSET, I2S_PD_CONF_REG_OFFSET,
    I2S_PD_FIFO_MEM_FORCE_PU, I2S_RXEOF_NUM_REG_OFFSET, I2S_SAMPLE_RATE_CONF_REG_OFFSET,
    I2S_TIMING_REG_OFFSET, IO_MUX_BASE_ADDR, JPEG_DMA_HALF_BUFFER_SIZE, JPEG_DMA_NODE_SIZE,
    JPEG_UNPACKED_HALF_BUFFER_SIZE, RTC_IO_ADC_PAD_REG, RTC_IO_BASE_ADDR, RTC_IO_PAD_DAC1_REG,
    RTC_IO_PAD_DAC2_REG, RTC_IO_SENSOR_PADS_REG, RTC_IO_TOUCH_PAD0_REG,
};
pub use esp32s2_i2s::{
    Esp32S2I2sCamera, Esp32S2I2sRegisterBlock, ESP32S2_DMA_ALIGN, ESP32S2_GPIO_NUM_MAX,
};
pub use esp32s3_lcd_cam::{
    Esp32S3LcdCam, Esp32S3LcdCamRegisterBlock, CAM_CLK_IDX, CAM_DATA_IN0_IDX, CAM_H_ENABLE_IDX,
    CAM_H_SYNC_IDX, CAM_PCLK_IDX, CAM_V_SYNC_IDX, ESP32S3_GDMA_BASE_ADDR, ESP32S3_GPIO_NUM_MAX,
    ESP32S3_LCD_CAM_BASE_ADDR, GDMA_PERI_SEL_LCD_CAM,
};
pub use xclk::{
    align_cache_region, XclkConfig, XclkController, DEFAULT_DCACHE_LINE_SIZE,
    LEDC_DUTY_50_PERCENT_1BIT, LEDC_LOW_SPEED_MODE, LEDC_TIMER_1_BIT, MAX_XCLK_FREQ_HZ,
    NO_CAMERA_LEDC_CHANNEL, NO_CAMERA_LEDC_TIMER,
};
