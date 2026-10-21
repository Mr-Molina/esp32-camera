//! Low-level `#![no_std]` DMA descriptor ring, I2S0/LCD_CAM capture filter, and XCLK configuration.

#![no_std]

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
    CamPinConfig, Esp32I2sCamera, GpioMatrixBinding, I2s0RegisterBlock, DPORT_I2S0_CLK_EN,
    DPORT_I2S0_RST, DPORT_PERIP_CLK_EN_REG, DPORT_PERIP_RST_EN_REG, ESP32_GPIO_NUM_MAX,
    GPIO_MATRIX_CONST_ONE_INPUT, I2S0I_DATA_IN0_IDX, I2S0I_H_ENABLE_IDX, I2S0I_H_SYNC_IDX,
    I2S0I_V_SYNC_IDX, I2S0I_WS_IN_IDX, I2S0_BASE_ADDR,
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
