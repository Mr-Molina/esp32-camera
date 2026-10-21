//! LEDC XCLK clock generator and PSRAM cache line alignment helpers (`target/xclk.c`, `driver/cam_hal.c:88-139`).

#![forbid(unsafe_code)]

use esp32_cam_types::CameraError;

/// Sentinel indicating no LEDC channel is currently active (`NO_CAMERA_LEDC_CHANNEL` in `target/xclk.c:16`).
pub const NO_CAMERA_LEDC_CHANNEL: u8 = 0xFF;

/// Sentinel indicating no LEDC timer is currently active (`NO_CAMERA_LEDC_TIMER` in `target/xclk.c:17`).
pub const NO_CAMERA_LEDC_TIMER: u8 = 0xFF;

/// LEDC 1-bit duty resolution (`LEDC_TIMER_1_BIT = 1`).
pub const LEDC_TIMER_1_BIT: u8 = 1;

/// LEDC low-speed timer/channel mode (`LEDC_LOW_SPEED_MODE = 1`).
pub const LEDC_LOW_SPEED_MODE: u8 = 1;

/// LEDC 50% duty value for 1-bit resolution (`duty = 1`, `hpoint = 0`).
pub const LEDC_DUTY_50_PERCENT_1BIT: u32 = 1;

/// Maximum supported XCLK output frequency in Hz (`40 MHz`, half of 80 MHz APB clock).
pub const MAX_XCLK_FREQ_HZ: u32 = 40_000_000;

/// Default ESP32/S2/S3 external memory DCache line size in bytes (`32` bytes).
pub const DEFAULT_DCACHE_LINE_SIZE: usize = 32;

/// Aligns a memory region `(addr, len)` outward to cache line boundaries (`line_size`),
/// matching `cam_drop_psram_cache` and `cam_writeback_psram_cache` in `driver/cam_hal.c:108-139`.
///
/// If `line_size == 0` or is not a power of two, falls back to [`DEFAULT_DCACHE_LINE_SIZE`] (`32`).
#[inline]
pub const fn align_cache_region(addr: usize, len: usize, line_size: usize) -> (usize, usize) {
    let line = if line_size == 0 || !line_size.is_power_of_two() {
        DEFAULT_DCACHE_LINE_SIZE
    } else {
        line_size
    };
    let mask = !(line - 1);
    let start = addr & mask;
    let offset = addr - start;
    let padded = len.saturating_add(offset).saturating_add(line - 1);
    let sync_len = padded & mask;
    (start, sync_len)
}

/// Configuration for the camera external clock (`XCLK`) generator (`target/xclk.c`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct XclkConfig {
    pub pin_xclk: u8,
    pub ledc_timer: u8,
    pub ledc_channel: u8,
    pub xclk_freq_hz: u32,
}

impl Default for XclkConfig {
    fn default() -> Self {
        Self {
            pin_xclk: 21,
            ledc_timer: 0,
            ledc_channel: 0,
            xclk_freq_hz: 20_000_000,
        }
    }
}

/// Deterministic `#![no_std]` state machine tracking LEDC timer and channel configuration for `XCLK`
/// (`xclk_timer_conf`, `camera_enable_out_clock`, `camera_disable_out_clock` in `target/xclk.c`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct XclkController {
    active_channel: u8,
    active_timer: u8,
    active_pin: Option<u8>,
    freq_hz: u32,
    duty_resolution_bits: u8,
    duty: u32,
    hpoint: u32,
    enabled: bool,
}

impl Default for XclkController {
    fn default() -> Self {
        Self::new()
    }
}

impl XclkController {
    /// Creates an unconfigured [`XclkController`] (`g_ledc_channel = 0xFF`, `g_ledc_timer = 0xFF`).
    #[inline]
    pub const fn new() -> Self {
        Self {
            active_channel: NO_CAMERA_LEDC_CHANNEL,
            active_timer: NO_CAMERA_LEDC_TIMER,
            active_pin: None,
            freq_hz: 0,
            duty_resolution_bits: LEDC_TIMER_1_BIT,
            duty: 0,
            hpoint: 0,
            enabled: false,
        }
    }

    /// Configures the LEDC timer frequency (`xclk_timer_conf` in `target/xclk.c:21-41`).
    pub fn timer_conf(&mut self, ledc_timer: u8, xclk_freq_hz: u32) -> Result<(), CameraError> {
        if ledc_timer > 3 || xclk_freq_hz == 0 || xclk_freq_hz > MAX_XCLK_FREQ_HZ {
            return Err(CameraError::InvalidArg);
        }
        self.active_timer = ledc_timer;
        self.freq_hz = xclk_freq_hz;
        self.duty_resolution_bits = LEDC_TIMER_1_BIT;
        Ok(())
    }

    /// Enables the `XCLK` output clock on `config.pin_xclk` (`camera_enable_out_clock` in `target/xclk.c:43-75`).
    pub fn enable_out_clock(&mut self, config: &XclkConfig) -> Result<(), CameraError> {
        if config.pin_xclk > 48 || config.ledc_channel > 7 {
            return Err(CameraError::InvalidArg);
        }
        self.timer_conf(config.ledc_timer, config.xclk_freq_hz)?;
        self.active_channel = config.ledc_channel;
        self.active_pin = Some(config.pin_xclk);
        self.duty = LEDC_DUTY_50_PERCENT_1BIT;
        self.hpoint = 0;
        self.enabled = true;
        Ok(())
    }

    /// Disables the `XCLK` output clock and resets active LEDC channel/timer sentinels
    /// (`camera_disable_out_clock` in `target/xclk.c:77-94`).
    pub fn disable_out_clock(&mut self) {
        if self.active_channel != NO_CAMERA_LEDC_CHANNEL {
            self.active_channel = NO_CAMERA_LEDC_CHANNEL;
        }
        if self.active_timer != NO_CAMERA_LEDC_TIMER {
            self.active_timer = NO_CAMERA_LEDC_TIMER;
        }
        self.active_pin = None;
        self.duty = 0;
        self.enabled = false;
    }

    /// Returns `true` if the `XCLK` output clock is currently enabled.
    #[inline]
    pub const fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// Returns the currently configured `XCLK` frequency in Hz (`0` if never configured).
    #[inline]
    pub const fn freq_hz(&self) -> u32 {
        self.freq_hz
    }

    /// Returns the active LEDC channel (`NO_CAMERA_LEDC_CHANNEL` if disabled).
    #[inline]
    pub const fn active_channel(&self) -> u8 {
        self.active_channel
    }

    /// Returns the active LEDC timer (`NO_CAMERA_LEDC_TIMER` if disabled).
    #[inline]
    pub const fn active_timer(&self) -> u8 {
        self.active_timer
    }

    /// Returns the GPIO pin bound to `XCLK`, if enabled.
    #[inline]
    pub const fn active_pin(&self) -> Option<u8> {
        self.active_pin
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_align_cache_region() {
        // Already aligned
        assert_eq!(align_cache_region(0x3F80_0000, 64, 32), (0x3F80_0000, 64));

        // Unaligned start and length spanning two 32-byte cache lines
        let (start, len) = align_cache_region(0x3F80_0015, 20, 32);
        assert_eq!(start, 0x3F80_0000);
        assert_eq!(len, 64);

        // Fallback when line_size == 0
        assert_eq!(
            align_cache_region(0x3F80_0004, 10, 0),
            (0x3F80_0000, 32)
        );
    }

    #[test]
    fn test_xclk_controller_lifecycle() {
        let mut xclk = XclkController::new();
        assert!(!xclk.is_enabled());
        assert_eq!(xclk.active_channel(), NO_CAMERA_LEDC_CHANNEL);
        assert_eq!(xclk.active_timer(), NO_CAMERA_LEDC_TIMER);

        let cfg = XclkConfig {
            pin_xclk: 21,
            ledc_timer: 0,
            ledc_channel: 0,
            xclk_freq_hz: 20_000_000,
        };
        assert_eq!(xclk.enable_out_clock(&cfg), Ok(()));
        assert!(xclk.is_enabled());
        assert_eq!(xclk.freq_hz(), 20_000_000);
        assert_eq!(xclk.active_channel(), 0);
        assert_eq!(xclk.active_timer(), 0);
        assert_eq!(xclk.active_pin(), Some(21));

        xclk.disable_out_clock();
        assert!(!xclk.is_enabled());
        assert_eq!(xclk.active_channel(), NO_CAMERA_LEDC_CHANNEL);
        assert_eq!(xclk.active_timer(), NO_CAMERA_LEDC_TIMER);
        assert_eq!(xclk.active_pin(), None);

        // Invalid parameters
        assert_eq!(xclk.timer_conf(4, 20_000_000), Err(CameraError::InvalidArg));
        assert_eq!(xclk.timer_conf(0, 0), Err(CameraError::InvalidArg));
        assert_eq!(xclk.timer_conf(0, 80_000_000), Err(CameraError::InvalidArg));
    }
}
