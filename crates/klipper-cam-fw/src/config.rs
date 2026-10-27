//! Hardware pinout, camera sensor, network, and OTA security configuration
//! for the bare-metal Klipper camera firmware (`klipper-cam-fw`).
//!
//! Replaces `examples/camera_example/main/camera_pinout.h`,
//! `examples/camera_example/main/klipper_cam_config.h`, and configuration blocks
//! from `klipper_cam.c` and `take_picture.c`.

#![forbid(unsafe_code)]

use core::fmt;
use esp32_cam_types::{CameraFbLocation, CameraGrabMode, FrameSize, PixelFormat};

/// Default OTA authentication token when `WIFI_OTA_TOKEN` is not set at compile time.
pub const DEFAULT_OTA_TOKEN: &str = "klipper-cam-ota-2026";

/// Default Wi-Fi SSID when `WIFI_SSID` is not set at compile time.
pub const DEFAULT_WIFI_SSID: &str = "iot";

/// Default Wi-Fi WPA2 passphrase when `WIFI_PASS` is not set at compile time.
pub const DEFAULT_WIFI_PASS: &str = "";

/// Optional compile-time override for `WIFI_OTA_TOKEN`.
pub const COMPILE_TIME_OTA_TOKEN: Option<&str> = option_env!("WIFI_OTA_TOKEN");

/// Optional compile-time override for `WIFI_SSID`.
pub const COMPILE_TIME_WIFI_SSID: Option<&str> = option_env!("WIFI_SSID");

/// Optional compile-time override for `WIFI_PASS`.
pub const COMPILE_TIME_WIFI_PASS: Option<&str> = option_env!("WIFI_PASS");

/// AI-Thinker high-power white flash LED GPIO pin (`GPIO_NUM_4`).
///
/// Note (`APP-004` / `FW-001`): On `WroverKit`, GPIO 4 is `CAM_PIN_D0` and must
/// NEVER be driven as a GPIO output after or before camera initialization.
pub const AITHINKER_FLASH_LED_GPIO: u8 = 4;

/// Default XCLK frequency in Hz (`20 MHz`).
pub const DEFAULT_XCLK_FREQ_HZ: u32 = 20_000_000;

/// Default JPEG compression quality (`12`, range `0..=63`, lower is higher quality).
pub const DEFAULT_JPEG_QUALITY: u8 = 12;

/// Default frame buffer count in PSRAM (`2` for double-buffered continuous capture).
pub const DEFAULT_FB_COUNT: usize = 2;

/// Default HTTP control port (`80`).
pub const DEFAULT_CONTROL_PORT: u16 = 80;

/// Default MJPEG stream port (`81`).
pub const DEFAULT_STREAM_PORT: u16 = 81;

/// Default internal control socket port for the Port 80 server (`32768`).
pub const DEFAULT_CONTROL_CTRL_PORT: u16 = 32768;

/// Default internal control socket port for the Port 81 stream server (`32769`).
pub const DEFAULT_STREAM_CTRL_PORT: u16 = 32769;

/// Default socket pacing delay between MJPEG frames in milliseconds (`50 ms`).
pub const DEFAULT_STREAM_PACING_MS: u32 = 50;

/// Default non-blocking Wi-Fi reconnect timer interval in milliseconds (`2000 ms`, `APP-009`).
pub const DEFAULT_WIFI_RECONNECT_MS: u32 = 2_000;

/// Returns the active OTA token resolved at compile time via `option_env!("OTA_TOKEN")`
/// or `option_env!("WIFI_OTA_TOKEN")`, falling back to [`DEFAULT_OTA_TOKEN`] if unset or empty.
#[inline]
pub const fn active_ota_token() -> &'static str {
    match option_env!("OTA_TOKEN") {
        Some(token) if !token.is_empty() => token,
        _ => match COMPILE_TIME_OTA_TOKEN {
            Some(token) if !token.is_empty() => token,
            _ => DEFAULT_OTA_TOKEN,
        },
    }
}

/// Returns the active Wi-Fi SSID resolved at compile time via `option_env!("WIFI_SSID")`,
/// falling back to [`DEFAULT_WIFI_SSID`] if unset or empty.
#[inline]
pub const fn active_wifi_ssid() -> &'static str {
    match COMPILE_TIME_WIFI_SSID {
        Some(ssid) if !ssid.is_empty() => ssid,
        _ => DEFAULT_WIFI_SSID,
    }
}

/// Returns the active Wi-Fi password resolved at compile time via `option_env!("WIFI_PASS")`,
/// falling back to [`DEFAULT_WIFI_PASS`] if unset.
#[inline]
pub const fn active_wifi_pass() -> &'static str {
    match COMPILE_TIME_WIFI_PASS {
        Some(pass) => pass,
        None => DEFAULT_WIFI_PASS,
    }
}

/// Supported ESP32 / ESP32-S3 camera carrier board variants (`camera_pinout.h`).
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum BoardVariant {
    /// Espressif `ESP32-WROVER-KIT` / `ESP32-WROVER-DEV` (default Klipper camera board).
    #[default]
    WroverKit = 0,
    /// `ESP32-CAM` by AI-Thinker.
    AiThinker = 1,
    /// `ESP32-S3-WROOM` camera board.
    Esp32S3Wroom = 2,
    /// `ESP32-S3-GOOUUU` camera board.
    Esp32S3Goouuu = 3,
    /// Seeed Studio `XIAO ESP32S3 Sense`.
    Esp32S3Xiao = 4,
}

impl BoardVariant {
    /// Returns the camera GPIO pin mapping for this board variant.
    #[inline]
    pub const fn pinout(self) -> CameraPinout {
        match self {
            Self::WroverKit => WROVER_KIT_PINOUT,
            Self::AiThinker => AITHINKER_PINOUT,
            Self::Esp32S3Wroom => ESP32S3_WROOM_PINOUT,
            Self::Esp32S3Goouuu => ESP32S3_GOOUUU_PINOUT,
            Self::Esp32S3Xiao => ESP32S3_XIAO_PINOUT,
        }
    }

    /// Returns `true` only if this board maps GPIO 4 to an external flash LED (`AiThinker`)
    /// rather than `CAM_PIN_D0` (`WroverKit`) (`APP-004` / `FW-001`).
    #[inline]
    pub const fn has_gpio4_flash_led(self) -> bool {
        matches!(self, Self::AiThinker)
    }
}

/// Complete camera parallel DVP + SCCB GPIO pin assignment (`camera_pinout.h`).
///
/// Unused pins (`PWDN`, `RESET`) are represented as `-1`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct CameraPinout {
    pub pin_pwdn: i8,
    pub pin_reset: i8,
    pub pin_xclk: i8,
    pub pin_sccb_sda: i8,
    pub pin_sccb_scl: i8,
    pub pin_d7: i8,
    pub pin_d6: i8,
    pub pin_d5: i8,
    pub pin_d4: i8,
    pub pin_d3: i8,
    pub pin_d2: i8,
    pub pin_d1: i8,
    pub pin_d0: i8,
    pub pin_vsync: i8,
    pub pin_href: i8,
    pub pin_pclk: i8,
}

/// `ESP32-WROVER-KIT` / `ESP32-WROVER-DEV` pin mapping (`BOARD_WROVER_KIT`).
pub const WROVER_KIT_PINOUT: CameraPinout = CameraPinout {
    pin_pwdn: -1,
    pin_reset: -1,
    pin_xclk: 21,
    pin_sccb_sda: 26,
    pin_sccb_scl: 27,
    pin_d7: 35,
    pin_d6: 34,
    pin_d5: 39,
    pin_d4: 36,
    pin_d3: 19,
    pin_d2: 18,
    pin_d1: 5,
    pin_d0: 4,
    pin_vsync: 25,
    pin_href: 23,
    pin_pclk: 22,
};

/// `ESP32-CAM` AI-Thinker pin mapping (`BOARD_ESP32CAM_AITHINKER`).
pub const AITHINKER_PINOUT: CameraPinout = CameraPinout {
    pin_pwdn: 32,
    pin_reset: -1,
    pin_xclk: 0,
    pin_sccb_sda: 26,
    pin_sccb_scl: 27,
    pin_d7: 35,
    pin_d6: 34,
    pin_d5: 39,
    pin_d4: 36,
    pin_d3: 21,
    pin_d2: 19,
    pin_d1: 18,
    pin_d0: 5,
    pin_vsync: 25,
    pin_href: 23,
    pin_pclk: 22,
};

/// `ESP32-S3-WROOM` pin mapping (`BOARD_ESP32S3_WROOM`).
pub const ESP32S3_WROOM_PINOUT: CameraPinout = CameraPinout {
    pin_pwdn: 38,
    pin_reset: -1,
    pin_xclk: 15,
    pin_sccb_sda: 4,
    pin_sccb_scl: 5,
    pin_d7: 16,
    pin_d6: 17,
    pin_d5: 18,
    pin_d4: 12,
    pin_d3: 10,
    pin_d2: 8,
    pin_d1: 9,
    pin_d0: 11,
    pin_vsync: 6,
    pin_href: 7,
    pin_pclk: 13,
};

/// `ESP32-S3-GOOUUU` pin mapping (`BOARD_ESP32S3_GOOUUU`).
pub const ESP32S3_GOOUUU_PINOUT: CameraPinout = CameraPinout {
    pin_pwdn: -1,
    pin_reset: -1,
    pin_xclk: 15,
    pin_sccb_sda: 4,
    pin_sccb_scl: 5,
    pin_d7: 16,
    pin_d6: 17,
    pin_d5: 18,
    pin_d4: 12,
    pin_d3: 10,
    pin_d2: 8,
    pin_d1: 9,
    pin_d0: 11,
    pin_vsync: 6,
    pin_href: 7,
    pin_pclk: 13,
};

/// `XIAO ESP32S3 Sense` pin mapping (`BOARD_ESP32S3_XIAO`).
pub const ESP32S3_XIAO_PINOUT: CameraPinout = CameraPinout {
    pin_pwdn: -1,
    pin_reset: -1,
    pin_xclk: 10,
    pin_sccb_sda: 40,
    pin_sccb_scl: 39,
    pin_d7: 48,
    pin_d6: 11,
    pin_d5: 12,
    pin_d4: 14,
    pin_d3: 16,
    pin_d2: 18,
    pin_d1: 17,
    pin_d0: 15,
    pin_vsync: 38,
    pin_href: 47,
    pin_pclk: 13,
};

/// `#![no_std]` IPv4 address representation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Ipv4Addr(pub [u8; 4]);

impl Ipv4Addr {
    /// Creates an [`Ipv4Addr`] from four octets.
    #[inline]
    pub const fn new(a: u8, b: u8, c: u8, d: u8) -> Self {
        Self([a, b, c, d])
    }

    /// Returns the four octets of this IPv4 address.
    #[inline]
    pub const fn octets(self) -> [u8; 4] {
        self.0
    }

    /// Converts the IPv4 address to big-endian (network byte order) `u32`.
    #[inline]
    pub const fn to_u32_be(self) -> u32 {
        u32::from_be_bytes(self.0)
    }

    /// Converts the IPv4 address to little-endian `u32` (matching LwIP `ip4_addr_t::addr` on Xtensa).
    #[inline]
    pub const fn to_u32_le(self) -> u32 {
        u32::from_le_bytes(self.0)
    }

    /// Parses a dotted-quad ASCII string (e.g. `"172.22.1.52"`) into an [`Ipv4Addr`].
    /// Rejects leading zeros on multi-digit octets (e.g. `"01"`, `"001"`).
    pub const fn parse_ascii(input: &str) -> Option<Self> {
        let bytes = input.as_bytes();
        if bytes.is_empty() {
            return None;
        }
        let mut octets = [0u8; 4];
        let mut octet_idx = 0usize;
        let mut current_val = 0u16;
        let mut digits_in_octet = 0usize;
        let mut leading_zero = false;
        let mut i = 0usize;

        while i < bytes.len() {
            let b = bytes[i];
            if b == b'.' {
                if digits_in_octet == 0 || octet_idx >= 3 || (leading_zero && digits_in_octet > 1) {
                    return None;
                }
                octets[octet_idx] = current_val as u8;
                octet_idx += 1;
                current_val = 0;
                digits_in_octet = 0;
                leading_zero = false;
            } else if b >= b'0' && b <= b'9' {
                if digits_in_octet == 0 && b == b'0' {
                    leading_zero = true;
                }
                if digits_in_octet >= 3 {
                    return None;
                }
                current_val = current_val * 10 + (b - b'0') as u16;
                if current_val > 255 {
                    return None;
                }
                digits_in_octet += 1;
                if leading_zero && digits_in_octet > 1 {
                    return None;
                }
            } else {
                return None;
            }
            i += 1;
        }

        if octet_idx != 3 || digits_in_octet == 0 || (leading_zero && digits_in_octet > 1) {
            return None;
        }
        octets[3] = current_val as u8;
        Some(Self(octets))
    }
}

impl fmt::Display for Ipv4Addr {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}.{}.{}.{}", self.0[0], self.0[1], self.0[2], self.0[3])
    }
}

/// Static IPv4 configuration for the Klipper camera station interface.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct StaticIpConfig {
    /// Station IPv4 address (`172.22.1.52`).
    pub ip: Ipv4Addr,
    /// Subnet mask (`255.255.255.0` for `/24`).
    pub netmask: Ipv4Addr,
    /// Default gateway (`172.22.1.1`).
    pub gateway: Ipv4Addr,
    /// Primary DNS server (`172.22.1.1`).
    pub dns: Ipv4Addr,
    /// CIDR prefix length (`24`).
    pub prefix_len: u8,
}

impl StaticIpConfig {
    /// Default Klipper camera static IP configuration (`172.22.1.52/24`, GW `172.22.1.1`).
    pub const KLIPPER_DEFAULT: Self = Self {
        ip: Ipv4Addr::new(172, 22, 1, 52),
        netmask: Ipv4Addr::new(255, 255, 255, 0),
        gateway: Ipv4Addr::new(172, 22, 1, 1),
        dns: Ipv4Addr::new(172, 22, 1, 1),
        prefix_len: 24,
    };

    /// Legacy fallback static IP configuration (`10.0.0.150/24`, GW `10.0.0.1`).
    pub const LEGACY_FALLBACK: Self = Self {
        ip: Ipv4Addr::new(10, 0, 0, 150),
        netmask: Ipv4Addr::new(255, 255, 255, 0),
        gateway: Ipv4Addr::new(10, 0, 0, 1),
        dns: Ipv4Addr::new(10, 0, 0, 1),
        prefix_len: 24,
    };

    /// Returns `true` if the static IP configuration is structurally valid.
    #[inline]
    pub const fn is_valid(&self) -> bool {
        self.prefix_len > 0
            && self.prefix_len <= 32
            && (self.ip.0[0] != 0
                || self.ip.0[1] != 0
                || self.ip.0[2] != 0
                || self.ip.0[3] != 0)
            && (self.gateway.0[0] != 0
                || self.gateway.0[1] != 0
                || self.gateway.0[2] != 0
                || self.gateway.0[3] != 0)
    }
}

impl Default for StaticIpConfig {
    #[inline]
    fn default() -> Self {
        Self::KLIPPER_DEFAULT
    }
}

/// Wi-Fi and IPv4 network configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct NetworkConfig {
    /// Whether to use dynamic DHCP (`true`) or static IP (`false`).
    pub use_dhcp: bool,
    /// Static IPv4 settings (`172.22.1.52/24`, GW `172.22.1.1`).
    pub static_ip: StaticIpConfig,
    /// 2.4 GHz Wi-Fi SSID.
    pub wifi_ssid: &'static str,
    /// WPA2-PSK passphrase.
    pub wifi_password: &'static str,
    /// Disable Wi-Fi modem sleep (`WIFI_PS_NONE`) for line-rate MJPEG throughput.
    pub disable_modem_sleep: bool,
    /// Non-blocking reconnect timer delay in milliseconds (`2000 ms`, `APP-009`).
    pub reconnect_delay_ms: u32,
}

impl NetworkConfig {
    /// Default network configuration with DHCP enabled and `172.22.1.52/24` static profile ready.
    pub const DEFAULT: Self = Self {
        use_dhcp: true,
        static_ip: StaticIpConfig::KLIPPER_DEFAULT,
        wifi_ssid: active_wifi_ssid(),
        wifi_password: active_wifi_pass(),
        disable_modem_sleep: true,
        reconnect_delay_ms: DEFAULT_WIFI_RECONNECT_MS,
    };

    /// Returns a network configuration pinned to static IP `172.22.1.52/24` (GW `172.22.1.1`).
    #[inline]
    pub const fn with_static_ip() -> Self {
        Self {
            use_dhcp: false,
            static_ip: StaticIpConfig::KLIPPER_DEFAULT,
            wifi_ssid: active_wifi_ssid(),
            wifi_password: active_wifi_pass(),
            disable_modem_sleep: true,
            reconnect_delay_ms: DEFAULT_WIFI_RECONNECT_MS,
        }
    }
}

impl Default for NetworkConfig {
    #[inline]
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Complete configuration for the Klipper 3D printer camera firmware (`klipper_cam.c`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct KlipperCamConfig {
    /// Target carrier board variant (`BoardVariant::WroverKit` by default).
    pub board: BoardVariant,
    /// Camera DVP and SCCB pin mapping.
    pub pinout: CameraPinout,
    /// External clock frequency in Hz (`20_000_000`).
    pub xclk_freq_hz: u32,
    /// LEDC timer index (`0`).
    pub ledc_timer: u8,
    /// LEDC channel index (`0`).
    pub ledc_channel: u8,
    /// Output pixel format (`PixelFormat::Jpeg`).
    pub pixel_format: PixelFormat,
    /// Output frame size (`FrameSize::Hd`, `1280x720` 16:9 widescreen).
    pub frame_size: FrameSize,
    /// JPEG compression quality (`12`).
    pub quality: u8,
    /// Number of frame buffers in PSRAM (`2` for double buffering).
    pub fb_count: usize,
    /// Frame buffer allocation target (`CameraFbLocation::Psram`).
    pub fb_location: CameraFbLocation,
    /// Frame buffer grab mode (`CameraGrabMode::Latest`).
    pub grab_mode: CameraGrabMode,
    /// Vertical flip (`false` by default).
    pub vflip: bool,
    /// Horizontal mirror (`false` by default).
    pub hmirror: bool,
    /// Control HTTP server port (`80`).
    pub control_port: u16,
    /// Dedicated MJPEG stream HTTP server port (`81`).
    pub stream_port: u16,
    /// Control HTTP server internal ctrl port (`32768`).
    pub control_ctrl_port: u16,
    /// Stream HTTP server internal ctrl port (`32769`).
    pub stream_ctrl_port: u16,
    /// Maximum open sockets on Port 80 (`4`).
    pub control_max_sockets: u8,
    /// Maximum open sockets on Port 81 (`2`).
    pub stream_max_sockets: u8,
    /// Inter-frame socket pacing delay in milliseconds (`50`).
    pub stream_pacing_ms: u32,
    /// Advertised `X-Framerate` header value (`5`).
    pub target_fps: u8,
    /// Network and Wi-Fi configuration.
    pub network: NetworkConfig,
    /// OTA shared secret token for `X-OTA-Token` authentication.
    pub ota_token: &'static str,
}

impl KlipperCamConfig {
    /// Canonical `WroverKit` 1280x720 HD JPEG configuration for Klipper/Moonraker.
    pub const WROVER_KIT_HD: Self = Self {
        board: BoardVariant::WroverKit,
        pinout: WROVER_KIT_PINOUT,
        xclk_freq_hz: DEFAULT_XCLK_FREQ_HZ,
        ledc_timer: 0,
        ledc_channel: 0,
        pixel_format: PixelFormat::Jpeg,
        frame_size: FrameSize::Hd,
        quality: DEFAULT_JPEG_QUALITY,
        fb_count: DEFAULT_FB_COUNT,
        fb_location: CameraFbLocation::Psram,
        grab_mode: CameraGrabMode::Latest,
        vflip: false,
        hmirror: false,
        control_port: DEFAULT_CONTROL_PORT,
        stream_port: DEFAULT_STREAM_PORT,
        control_ctrl_port: DEFAULT_CONTROL_CTRL_PORT,
        stream_ctrl_port: DEFAULT_STREAM_CTRL_PORT,
        control_max_sockets: 4,
        stream_max_sockets: 2,
        stream_pacing_ms: DEFAULT_STREAM_PACING_MS,
        target_fps: 5,
        network: NetworkConfig::DEFAULT,
        ota_token: active_ota_token(),
    };

    /// Creates a configuration for a specific [`BoardVariant`] while preserving HD Klipper defaults.
    #[inline]
    pub const fn for_board(board: BoardVariant) -> Self {
        let mut cfg = Self::WROVER_KIT_HD;
        cfg.board = board;
        cfg.pinout = board.pinout();
        cfg
    }

    /// Returns the horizontal width in pixels for the configured [`FrameSize`].
    #[inline]
    pub const fn width(&self) -> u16 {
        self.frame_size.width()
    }

    /// Returns the vertical height in pixels for the configured [`FrameSize`].
    #[inline]
    pub const fn height(&self) -> u16 {
        self.frame_size.height()
    }

    /// Default OTA authentication token string.
    pub const DEFAULT_OTA_TOKEN: &'static str = DEFAULT_OTA_TOKEN;

    /// Returns `true` if the configured OTA token matches the factory default.
    #[inline]
    pub fn is_default_ota_token(&self) -> bool {
        let expected = Self::DEFAULT_OTA_TOKEN.as_bytes();
        let actual = self.ota_token.as_bytes();
        if expected.len() != actual.len() {
            return false;
        }
        let mut diff = 0usize;
        let mut i = 0;
        while i < expected.len() {
            diff |= (expected[i] ^ actual[i]) as usize;
            i += 1;
        }
        diff == 0
    }

    /// Returns the HTTP control port.
    #[inline]
    pub const fn http_port(&self) -> u16 {
        self.control_port
    }

    /// Returns the static IP configuration.
    #[inline]
    pub const fn static_ip(&self) -> &StaticIpConfig {
        &self.network.static_ip
    }

    /// Returns `true` if this configuration is structurally and semantically valid.
    pub const fn is_valid(&self) -> bool {
        let ssid = self.network.wifi_ssid.as_bytes();
        let pass = self.network.wifi_password.as_bytes();
        let token = self.ota_token.as_bytes();

        if ssid.is_empty() || ssid.len() > 32 {
            return false;
        }
        if !pass.is_empty() && (pass.len() < 8 || pass.len() > 63) {
            return false;
        }
        if token.is_empty() {
            return false;
        }
        if !self.network.static_ip.is_valid() {
            return false;
        }
        if self.control_port == 0
            || self.stream_port == 0
            || self.control_port == self.stream_port
        {
            return false;
        }
        true
    }

    /// Resolves `KlipperCamConfig` from compile-time build environment variables.
    pub const fn from_build_env() -> Self {
        let mut cfg = Self::WROVER_KIT_HD;
        let token = match option_env!("OTA_TOKEN") {
            Some(tok) if !tok.is_empty() => tok,
            _ => match option_env!("WIFI_OTA_TOKEN") {
                Some(tok) if !tok.is_empty() => tok,
                _ => Self::DEFAULT_OTA_TOKEN,
            },
        };
        cfg.ota_token = token;
        cfg
    }
}

impl Default for KlipperCamConfig {
    #[inline]
    fn default() -> Self {
        Self::WROVER_KIT_HD
    }
}

/// Configuration for the standalone periodic capture runner (`take_picture.c`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct TakePictureConfig {
    pub board: BoardVariant,
    pub pinout: CameraPinout,
    pub xclk_freq_hz: u32,
    pub pixel_format: PixelFormat,
    pub frame_size: FrameSize,
    pub quality: u8,
    pub fb_count: usize,
    pub fb_location: CameraFbLocation,
    pub grab_mode: CameraGrabMode,
    pub capture_interval_ms: u32,
}

impl TakePictureConfig {
    /// Default periodic capture configuration matching `take_picture.c`.
    pub const DEFAULT: Self = Self {
        board: BoardVariant::WroverKit,
        pinout: WROVER_KIT_PINOUT,
        xclk_freq_hz: DEFAULT_XCLK_FREQ_HZ,
        pixel_format: PixelFormat::Rgb565,
        frame_size: FrameSize::Qvga,
        quality: DEFAULT_JPEG_QUALITY,
        fb_count: 1,
        fb_location: CameraFbLocation::Psram,
        grab_mode: CameraGrabMode::WhenEmpty,
        capture_interval_ms: 5_000,
    };
}

impl Default for TakePictureConfig {
    #[inline]
    fn default() -> Self {
        Self::DEFAULT
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_wrover_kit_hd_defaults() {
        let cfg = KlipperCamConfig::default();
        assert_eq!(cfg.board, BoardVariant::WroverKit);
        assert_eq!(cfg.frame_size, FrameSize::Hd);
        assert_eq!(cfg.width(), 1280);
        assert_eq!(cfg.height(), 720);
        assert_eq!(cfg.pixel_format, PixelFormat::Jpeg);
        assert_eq!(cfg.quality, 12);
        assert_eq!(cfg.fb_count, 2);
        assert_eq!(cfg.xclk_freq_hz, 20_000_000);
        assert_eq!(cfg.control_port, 80);
        assert_eq!(cfg.stream_port, 81);
        assert_eq!(cfg.pinout.pin_d0, 4);
        assert!(!cfg.board.has_gpio4_flash_led());
    }

    #[test]
    fn test_aithinker_board_pinout_and_flash_led_guard() {
        let cfg = KlipperCamConfig::for_board(BoardVariant::AiThinker);
        assert_eq!(cfg.board, BoardVariant::AiThinker);
        assert_eq!(cfg.pinout.pin_pwdn, 32);
        assert_eq!(cfg.pinout.pin_xclk, 0);
        assert_eq!(cfg.pinout.pin_d0, 5);
        assert!(cfg.board.has_gpio4_flash_led());
    }

    #[test]
    fn test_static_ip_and_parsing() {
        let static_cfg = StaticIpConfig::default();
        assert_eq!(static_cfg.ip, Ipv4Addr::new(172, 22, 1, 52));
        assert_eq!(static_cfg.gateway, Ipv4Addr::new(172, 22, 1, 1));
        assert_eq!(static_cfg.dns, Ipv4Addr::new(172, 22, 1, 1));
        assert_eq!(static_cfg.netmask, Ipv4Addr::new(255, 255, 255, 0));
        assert_eq!(static_cfg.prefix_len, 24);
        assert!(static_cfg.is_valid());

        assert_eq!(
            Ipv4Addr::parse_ascii("172.22.1.52"),
            Some(Ipv4Addr::new(172, 22, 1, 52))
        );
        assert_eq!(
            Ipv4Addr::parse_ascii("172.22.1.1"),
            Some(Ipv4Addr::new(172, 22, 1, 1))
        );
        assert_eq!(
            Ipv4Addr::parse_ascii("0.0.0.0"),
            Some(Ipv4Addr::new(0, 0, 0, 0))
        );
        // Reject leading zeros
        assert_eq!(Ipv4Addr::parse_ascii("01.2.3.4"), None);
        assert_eq!(Ipv4Addr::parse_ascii("172.022.1.1"), None);
        assert_eq!(Ipv4Addr::parse_ascii("172.22.001.1"), None);
        assert_eq!(Ipv4Addr::parse_ascii("256.0.0.1"), None);
        assert_eq!(Ipv4Addr::parse_ascii("172.22.1"), None);
        assert_eq!(Ipv4Addr::parse_ascii(""), None);

        let cfg = KlipperCamConfig::default();
        assert!(cfg.is_valid());
        assert!(cfg.is_default_ota_token());
        assert_eq!(cfg.http_port(), 80);
        assert_eq!(cfg.static_ip().prefix_len, 24);

        let env_cfg = KlipperCamConfig::from_build_env();
        assert!(env_cfg.is_valid());
    }

    #[test]
    fn test_take_picture_config_defaults() {
        let tp = TakePictureConfig::default();
        assert_eq!(tp.frame_size, FrameSize::Qvga);
        assert_eq!(tp.pixel_format, PixelFormat::Rgb565);
        assert_eq!(tp.quality, 12);
        assert_eq!(tp.fb_count, 1);
        assert_eq!(tp.grab_mode, CameraGrabMode::WhenEmpty);
        assert_eq!(tp.capture_interval_ms, 5000);
    }

    #[test]
    fn test_ota_token_resolution() {
        let token = active_ota_token();
        assert!(!token.is_empty());
    }
}
