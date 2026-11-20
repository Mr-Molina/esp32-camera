//! Hardware pinout, camera sensor, network, and OTA security configuration
//! for the bare-metal Klipper camera firmware (`klipper-cam-fw`).

#![forbid(unsafe_code)]

use core::fmt;
use esp32_cam_types::{CameraFbLocation, CameraGrabMode, FrameSize, PixelFormat};

/// Default OTA authentication token when `WIFI_OTA_TOKEN` is unset at
/// compile time.
///
/// This value is public. A firmware still running with it refuses every
/// `POST /update` with `403 Forbidden` (`FW-NET-04`), and Xtensa release
/// builds without a real token fail in `build.rs` (`BUILD-07`).
pub const DEFAULT_OTA_TOKEN: &str = "klipper-cam-ota-2026";

/// Minimum required length in bytes for a valid production OTA authentication
/// token (`M-031` / `C1-010`).
pub const MIN_OTA_TOKEN_LEN: usize = 16;

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

/// Default JPEG compression quality (`12`, range `0..=63`, lower is higher
/// quality).
pub const DEFAULT_JPEG_QUALITY: u8 = 12;

/// Default frame buffer count in PSRAM (`2` for double-buffered continuous
/// capture).
pub const DEFAULT_FB_COUNT: usize = 2;

/// Default HTTP control port (`80`).
pub const DEFAULT_CONTROL_PORT: u16 = 80;

/// Default MJPEG stream port (`81`).
pub const DEFAULT_STREAM_PORT: u16 = 81;

/// Default socket pacing delay between MJPEG frames in milliseconds (`50 ms`).
pub const DEFAULT_STREAM_PACING_MS: u32 = 50;

/// Default non-blocking Wi-Fi reconnect timer interval in milliseconds
/// (`2000 ms`, `APP-009`).
pub const DEFAULT_WIFI_RECONNECT_MS: u32 = 2_000;

/// Active HTTP control server port (`80` by default).
pub const HTTP_PORT: u16 = DEFAULT_CONTROL_PORT;

/// Active MJPEG video stream server port (`81` by default).
pub const STREAM_PORT: u16 = DEFAULT_STREAM_PORT;

/// Active JPEG compression quality (`12` by default, range `1..=63`).
pub const JPEG_QUALITY: u8 = DEFAULT_JPEG_QUALITY;

/// Environment variables checked by [`active_ota_token`] in exact precedence
/// order (`M-027`, kept 1:1 with `build.rs::OTA_TOKEN_VARS`).
pub const OTA_TOKEN_VARS: [&str; 4] = [
    "CAM_OTA_TOKEN",
    "OTA_TOKEN",
    "WIFI_OTA_TOKEN",
    "ESP32_CAM_OTA_TOKEN",
];

/// Returns the active OTA token resolved at compile time via `option_env!`.
///
/// Checked in priority order (1:1 with `build.rs::OTA_TOKEN_VARS`):
/// 1. `CAM_OTA_TOKEN`
/// 2. `OTA_TOKEN`
/// 3. `WIFI_OTA_TOKEN`
/// 4. `ESP32_CAM_OTA_TOKEN`
/// 5. Under `provisioning-ap`, returns `""` (never [`DEFAULT_OTA_TOKEN`]);
///    otherwise falls back to [`DEFAULT_OTA_TOKEN`] if unset or empty.
#[inline]
pub const fn active_ota_token() -> &'static str {
    let fallback = if cfg!(feature = "provisioning-ap") {
        ""
    } else {
        DEFAULT_OTA_TOKEN
    };
    match option_env!("CAM_OTA_TOKEN") {
        Some(token) if !token.is_empty() => token,
        _ => match option_env!("OTA_TOKEN") {
            Some(token) if !token.is_empty() => token,
            _ => match COMPILE_TIME_OTA_TOKEN {
                Some(token) if !token.is_empty() => token,
                _ => match option_env!("ESP32_CAM_OTA_TOKEN") {
                    Some(token) if !token.is_empty() => token,
                    _ => fallback,
                },
            },
        },
    }
}

/// Returns `true` if `token` is a valid production OTA authentication token
/// (`M-027` / `M-031` / `C1-010`):
/// - at least [`MIN_OTA_TOKEN_LEN`] (`16`) bytes
/// - every byte is graphic ASCII (`0x21..=0x7E`, rejecting all whitespace,
///   control characters, and non-ASCII bytes)
/// - not equal to [`DEFAULT_OTA_TOKEN`]
#[inline]
pub const fn is_valid_ota_token(token: &[u8]) -> bool {
    if token.len() < MIN_OTA_TOKEN_LEN {
        return false;
    }
    let mut i = 0;
    while i < token.len() {
        let b = token[i];
        if b < 0x21 || b > 0x7E {
            return false;
        }
        i += 1;
    }
    !const_bytes_eq(token, DEFAULT_OTA_TOKEN.as_bytes())
}

/// Active OTA authentication token resolved at compile time.
///
/// Overridden at build time by setting the `CAM_OTA_TOKEN` environment variable
/// (with fallback checking `OTA_TOKEN` and `WIFI_OTA_TOKEN`). If none of these
/// variables are set, defaults to [`DEFAULT_OTA_TOKEN`] so offline builds and
/// tests compile cleanly without environment requirements.
pub const OTA_TOKEN: &str = active_ota_token();

/// `true` when the crate is built with the `allow-open-wifi` feature, the
/// only way an empty `WIFI_PASS` may join an Open (unencrypted) network
/// (`FW-NET-03`).
pub const ALLOW_OPEN_WIFI: bool = cfg!(feature = "allow-open-wifi");

/// Returns `true` if `len` is a usable WPA2-PSK passphrase length: `0`
/// (Open network, subject to [`ALLOW_OPEN_WIFI`]) or `8..=63` bytes.
#[inline]
pub const fn is_valid_wifi_passphrase_len(len: usize) -> bool {
    len == 0 || (len >= 8 && len <= 63)
}

/// Wi-Fi station authentication selected at boot by [`wifi_auth_mode`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum WifiAuthMode {
    /// Unencrypted network; only reachable with the `allow-open-wifi` feature.
    Open,
    /// WPA/WPA2-Personal with the configured passphrase.
    WpaWpa2Personal,
}

/// Fatal boot-time configuration errors (`FW-NET-03`). The firmware refuses
/// to start Wi-Fi rather than fall back to an Open network.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ConfigError {
    /// [`KlipperCamConfig::is_valid`] returned `false`.
    InvalidConfig,
    /// Empty `WIFI_PASS` without the `allow-open-wifi` feature.
    OpenWifiNotAllowed,
    /// `WIFI_PASS` length (bytes) is outside `8..=63`.
    InvalidPassphraseLength(usize),
}

impl fmt::Display for ConfigError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidConfig => f.write_str(
                "invalid configuration (SSID 1..=32 bytes, WIFI_PASS empty or 8..=63 bytes, \
                 non-default OTA token >= 16 bytes, quality 1..=63, distinct non-zero ports)",
            ),
            Self::OpenWifiNotAllowed => f.write_str(
                "WIFI_PASS is empty; build with the `allow-open-wifi` feature \
                 to join an Open network",
            ),
            Self::InvalidPassphraseLength(len) => {
                write!(f, "WIFI_PASS is {} bytes; WPA2-PSK needs 8..=63", len)
            }
        }
    }
}

/// Selects the station authentication for `pass`, failing closed
/// (`FW-NET-03`): an empty passphrase is Open only when `allow_open` is set,
/// and a passphrase outside `8..=63` bytes is an error, never Open.
#[inline]
pub const fn wifi_auth_mode(pass: &str, allow_open: bool) -> Result<WifiAuthMode, ConfigError> {
    let len = pass.len();
    if len == 0 {
        if allow_open {
            Ok(WifiAuthMode::Open)
        } else {
            Err(ConfigError::OpenWifiNotAllowed)
        }
    } else if is_valid_wifi_passphrase_len(len) {
        Ok(WifiAuthMode::WpaWpa2Personal)
    } else {
        Err(ConfigError::InvalidPassphraseLength(len))
    }
}

const _: () = assert!(HTTP_PORT != STREAM_PORT);
// FW-NET-03: a compile-time `WIFI_PASS` must be empty or a valid WPA2-PSK
// passphrase (8..=63 bytes); a 70-byte passphrase fails the build here.
const _: () = assert!(
    is_valid_wifi_passphrase_len(active_wifi_pass().len()),
    "WIFI_PASS must be empty or 8..=63 bytes"
);
const _: () = assert!(JPEG_QUALITY >= 1 && JPEG_QUALITY <= 63);
#[cfg(not(feature = "provisioning-ap"))]
const _: () = assert!(!OTA_TOKEN.is_empty());

/// Returns `true` when `WIFI_SSID` was explicitly set to a non-empty value at
/// compile time (as opposed to falling back to [`DEFAULT_WIFI_SSID`]).
#[inline]
pub const fn has_explicit_build_wifi_ssid() -> bool {
    match COMPILE_TIME_WIFI_SSID {
        Some(ssid) => !ssid.is_empty(),
        None => false,
    }
}

/// Returns `true` when `WIFI_PASS` was explicitly set to a valid WPA2-PSK
/// passphrase (`8..=63` bytes) at compile time.
#[inline]
pub const fn has_explicit_build_wifi_pass() -> bool {
    match COMPILE_TIME_WIFI_PASS {
        Some(pass) => pass.len() >= 8 && pass.len() <= 63,
        None => false,
    }
}

/// Returns the active Wi-Fi SSID resolved at compile time via
/// `option_env!("WIFI_SSID")`, falling back to [`DEFAULT_WIFI_SSID`] if unset
/// or empty.
#[inline]
pub const fn active_wifi_ssid() -> &'static str {
    match COMPILE_TIME_WIFI_SSID {
        Some(ssid) if !ssid.is_empty() => ssid,
        _ => DEFAULT_WIFI_SSID,
    }
}

/// Returns the active Wi-Fi passphrase resolved at compile time via
/// `option_env!("WIFI_PASS")`, falling back to [`DEFAULT_WIFI_PASS`] if unset.
#[inline]
pub const fn active_wifi_pass() -> &'static str {
    match COMPILE_TIME_WIFI_PASS {
        Some(pass) => pass,
        None => DEFAULT_WIFI_PASS,
    }
}

/// Supported ESP32 / ESP32-S3 camera carrier board variants
/// (`camera_pinout.h`).
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum BoardVariant {
    /// Espressif `ESP32-WROVER-KIT` / `ESP32-WROVER-DEV` (default Klipper
    /// camera board).
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

    /// Returns `true` only if this board maps GPIO 4 to an external flash LED
    /// (`AiThinker`) rather than `CAM_PIN_D0` (`WroverKit`) (`APP-004` /
    /// `FW-001`).
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

    /// Converts the IPv4 address to little-endian `u32` (matching LwIP
    /// `ip4_addr_t::addr` on Xtensa).
    #[inline]
    pub const fn to_u32_le(self) -> u32 {
        u32::from_le_bytes(self.0)
    }

    /// Parses a dotted-quad ASCII string (e.g. `"172.22.1.52"`) into an
    /// [`Ipv4Addr`]. Rejects leading zeros on multi-digit octets (e.g. `"01"`,
    /// `"001"`).
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
    /// Default Klipper camera static IP configuration (`172.22.1.52/24`,
    /// GW `172.22.1.1`).
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

    /// Returns `true` if the static IP configuration is structurally valid
    /// (`LOGIC-171`).
    #[inline]
    pub const fn is_valid(&self) -> bool {
        self.prefix_len > 0
            && self.prefix_len <= 32
            && (self.ip.0[0] != 0 || self.ip.0[1] != 0 || self.ip.0[2] != 0 || self.ip.0[3] != 0)
            && (self.gateway.0[0] != 0
                || self.gateway.0[1] != 0
                || self.gateway.0[2] != 0
                || self.gateway.0[3] != 0)
            && self.ip.to_u32_be() != self.gateway.to_u32_be()
            && (self.ip.to_u32_be() & self.netmask.to_u32_be())
                == (self.gateway.to_u32_be() & self.netmask.to_u32_be())
    }
}

impl Default for StaticIpConfig {
    #[inline]
    fn default() -> Self {
        Self::KLIPPER_DEFAULT
    }
}

/// IPv4 address assignment mode for the Wi-Fi station interface.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum IpMode {
    /// Obtain IPv4 address, netmask, gateway, and DNS via DHCPv4.
    Dhcp,
    /// Use a fixed static IPv4 configuration (DHCPv4 disabled).
    Static(StaticIpConfig),
}

impl IpMode {
    /// Returns `true` if this IP mode is structurally valid.
    #[inline]
    pub const fn is_valid(&self) -> bool {
        match self {
            Self::Dhcp => true,
            Self::Static(cfg) => cfg.is_valid(),
        }
    }
}

impl Default for IpMode {
    #[inline]
    fn default() -> Self {
        Self::Static(StaticIpConfig::KLIPPER_DEFAULT)
    }
}

/// Wi-Fi and IPv4 network configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct NetworkConfig {
    /// IPv4 address assignment mode (static by default for backward
    /// compatibility).
    pub ip_mode: IpMode,
    /// Static IPv4 settings (`172.22.1.52/24`, GW `172.22.1.1`).
    pub static_ip: StaticIpConfig,
    /// 2.4 GHz Wi-Fi SSID.
    pub wifi_ssid: &'static str,
    /// WPA2-PSK passphrase.
    pub wifi_password: &'static str,
    /// Non-blocking reconnect timer delay in milliseconds (`2000 ms`, `APP-009`).
    pub reconnect_delay_ms: u32,
}

impl NetworkConfig {
    /// Default network configuration and `172.22.1.52/24` static profile ready.
    pub const DEFAULT: Self = Self {
        ip_mode: IpMode::Static(StaticIpConfig::KLIPPER_DEFAULT),
        static_ip: StaticIpConfig::KLIPPER_DEFAULT,
        wifi_ssid: active_wifi_ssid(),
        wifi_password: active_wifi_pass(),
        reconnect_delay_ms: DEFAULT_WIFI_RECONNECT_MS,
    };
}

impl Default for NetworkConfig {
    #[inline]
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Complete configuration for the Klipper 3D printer camera firmware
/// (`klipper_cam.c`).
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
    /// Inter-frame socket pacing delay in milliseconds (`50`).
    pub stream_pacing_ms: u32,
    /// Advertised `X-Framerate` header value (`5`).
    pub target_fps: u8,
    /// Network and Wi-Fi configuration.
    pub network: NetworkConfig,
    /// OTA authentication token for `X-OTA-Token` verification.
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
        stream_pacing_ms: DEFAULT_STREAM_PACING_MS,
        target_fps: 5,
        network: NetworkConfig::DEFAULT,
        ota_token: active_ota_token(),
    };

    /// Creates a configuration for a specific [`BoardVariant`] while
    /// preserving HD Klipper defaults.
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
        crate::ota::verify_ota_token(self.ota_token.as_bytes(), DEFAULT_OTA_TOKEN.as_bytes())
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

    /// Returns the configured IPv4 assignment mode (`Dhcp` or `Static`).
    #[inline]
    pub const fn ip_mode(&self) -> IpMode {
        self.network.ip_mode
    }

    /// Returns `true` if this configuration holds usable Wi-Fi station
    /// credentials (`wifi_ssid` in `1..=32` bytes and either a `8..=63`-byte
    /// WPA2-PSK passphrase or an empty passphrase with `allow-open-wifi`).
    ///
    /// Under `provisioning-ap`, the built-in [`DEFAULT_WIFI_SSID`] fallback
    /// (`"iot"`) paired with an empty passphrase never satisfies this check
    /// unless `WIFI_SSID` was explicitly provided at build time.
    #[inline]
    pub const fn has_sta_credentials(&self) -> bool {
        let ssid = self.network.wifi_ssid.as_bytes();
        let pass = self.network.wifi_password.as_bytes();

        if ssid.is_empty() || ssid.len() > 32 {
            return false;
        }
        if cfg!(feature = "provisioning-ap")
            && !has_explicit_build_wifi_ssid()
            && pass.is_empty()
            && const_bytes_eq(ssid, DEFAULT_WIFI_SSID.as_bytes())
        {
            return false;
        }
        if pass.is_empty() {
            ALLOW_OPEN_WIFI
        } else {
            pass.len() >= 8 && pass.len() <= 63
        }
    }

    /// Returns `true` if the configured OTA authentication token satisfies
    /// [`is_valid_ota_token`] (at least [`MIN_OTA_TOKEN_LEN`] graphic ASCII
    /// bytes and not equal to [`DEFAULT_OTA_TOKEN`]).
    #[inline]
    pub const fn has_valid_ota_token(&self) -> bool {
        is_valid_ota_token(self.ota_token.as_bytes())
    }

    /// Returns `true` if this configuration is structurally and semantically valid.
    ///
    /// Enforces (`M-031` / `C1-010`):
    /// - `wifi_ssid` length in `1..=32` bytes
    /// - `wifi_password` empty or `8..=63` bytes
    /// - `ota_token` at least [`MIN_OTA_TOKEN_LEN`] (`16`) bytes and not equal to
    ///   [`DEFAULT_OTA_TOKEN`]
    /// - `quality` in `1..=63`
    /// - valid `ip_mode` / `static_ip` and distinct non-zero `control_port` /
    ///   `stream_port`
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
        if !is_valid_ota_token(token) {
            return false;
        }
        if self.quality < 1 || self.quality > 63 {
            return false;
        }
        if let IpMode::Static(static_cfg) = self.network.ip_mode {
            if !static_cfg.is_valid() || !self.network.static_ip.is_valid() {
                return false;
            }
        }
        if self.control_port == 0 || self.stream_port == 0 || self.control_port == self.stream_port
        {
            return false;
        }
        true
    }

    /// Boot-time configuration gate (`FW-NET-03`): runs [`Self::is_valid`]
    /// and then selects the Wi-Fi authentication via [`wifi_auth_mode`].
    /// Any error must stop Wi-Fi bring-up.
    pub const fn boot_wifi_auth(&self, allow_open: bool) -> Result<WifiAuthMode, ConfigError> {
        if !self.is_valid() {
            return Err(ConfigError::InvalidConfig);
        }
        wifi_auth_mode(self.network.wifi_password, allow_open)
    }

    /// Resolves `KlipperCamConfig` from compile-time build environment variables.
    ///
    /// Under the `provisioning-ap` feature, an unset `WIFI_SSID` resolves to
    /// `""` instead of [`DEFAULT_WIFI_SSID`] so a freshly flashed device never
    /// mistakes the built-in fallback for provisioned station credentials.
    pub const fn from_build_env() -> Self {
        let mut cfg = Self::WROVER_KIT_HD;
        cfg.ota_token = active_ota_token();
        if cfg!(feature = "provisioning-ap") && !has_explicit_build_wifi_ssid() {
            cfg.network.wifi_ssid = "";
        }
        cfg
    }

    /// Applies optional configuration overrides loaded from NVS (`LOGIC-172` /
    /// `M-031`), validating every overridden field and rejecting weak/default
    /// tokens, invalid passphrases, out-of-range quality, or invalid static IP.
    #[inline]
    pub fn with_nvs_overrides(
        mut self,
        overrides: NvsConfigOverrides<'static>,
    ) -> Result<Self, ConfigError> {
        if let Some(ssid) = overrides.wifi_ssid {
            if ssid.is_empty() || ssid.len() > 32 {
                return Err(ConfigError::InvalidConfig);
            }
            self.network.wifi_ssid = ssid;
        }
        if let Some(pass) = overrides.wifi_password {
            let _ = wifi_auth_mode(pass, ALLOW_OPEN_WIFI)?;
            self.network.wifi_password = pass;
        }
        if let Some(token) = overrides.ota_token {
            if !is_valid_ota_token(token.as_bytes()) {
                return Err(ConfigError::InvalidConfig);
            }
            self.ota_token = token;
        }
        if let Some(vflip) = overrides.vflip {
            self.vflip = vflip;
        }
        if let Some(hmirror) = overrides.hmirror {
            self.hmirror = hmirror;
        }
        if let Some(quality) = overrides.quality {
            if !(1..=63).contains(&quality) {
                return Err(ConfigError::InvalidConfig);
            }
            self.quality = quality;
        }
        if let Some(frame_size) = overrides.frame_size {
            self.frame_size = frame_size;
        }
        if let Some(static_ip) = overrides.static_ip {
            if !static_ip.is_valid() {
                return Err(ConfigError::InvalidConfig);
            }
            self.network.static_ip = static_ip;
            self.network.ip_mode = IpMode::Static(static_ip);
        }
        if let Some(ip_mode) = overrides.ip_mode {
            match ip_mode {
                IpMode::Dhcp => {
                    self.network.ip_mode = IpMode::Dhcp;
                }
                IpMode::Static(static_ip) => {
                    if !static_ip.is_valid() {
                        return Err(ConfigError::InvalidConfig);
                    }
                    self.network.static_ip = static_ip;
                    self.network.ip_mode = IpMode::Static(static_ip);
                }
            }
        }
        if !self.is_valid() {
            return Err(ConfigError::InvalidConfig);
        }
        Ok(self)
    }
}

/// Const-compatible byte-slice equality helper for `is_valid`.
#[inline]
const fn const_bytes_eq(a: &[u8], b: &[u8]) -> bool {
    if a.len() != b.len() {
        return false;
    }
    let mut i = 0;
    while i < a.len() {
        if a[i] != b[i] {
            return false;
        }
        i += 1;
    }
    true
}

/// Optional configuration overrides retrieved from non-volatile storage
/// (`LOGIC-172`).
///
/// Implements a manual [`fmt::Debug`] that redacts `wifi_password` and
/// `ota_token` (`M-134` / `C1-036`).
#[derive(Clone, Copy, PartialEq, Eq, Default)]
pub struct NvsConfigOverrides<'a> {
    /// Optional Wi-Fi SSID override.
    pub wifi_ssid: Option<&'a str>,
    /// Optional Wi-Fi password override.
    pub wifi_password: Option<&'a str>,
    /// Optional OTA authentication token override.
    pub ota_token: Option<&'a str>,
    /// Optional vertical flip override.
    pub vflip: Option<bool>,
    /// Optional horizontal mirror override.
    pub hmirror: Option<bool>,
    /// Optional JPEG compression quality override (`1..=63`).
    pub quality: Option<u8>,
    /// Optional frame size override.
    pub frame_size: Option<FrameSize>,
    /// Optional static IP configuration override.
    pub static_ip: Option<StaticIpConfig>,
    /// Optional IPv4 assignment mode override (`Dhcp` or `Static`).
    pub ip_mode: Option<IpMode>,
}

impl fmt::Debug for NvsConfigOverrides<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("NvsConfigOverrides")
            .field("wifi_ssid", &self.wifi_ssid)
            .field("wifi_password", &self.wifi_password.map(|_| "[REDACTED]"))
            .field("ota_token", &self.ota_token.map(|_| "[REDACTED]"))
            .field("vflip", &self.vflip)
            .field("hmirror", &self.hmirror)
            .field("quality", &self.quality)
            .field("frame_size", &self.frame_size)
            .field("static_ip", &self.static_ip)
            .field("ip_mode", &self.ip_mode)
            .finish()
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

        let mut cfg = KlipperCamConfig::default();
        assert!(
            !cfg.is_valid(),
            "factory default OTA token must fail is_valid()"
        );
        #[cfg(not(feature = "provisioning-ap"))]
        assert!(cfg.is_default_ota_token());
        #[cfg(feature = "provisioning-ap")]
        assert!(!cfg.has_valid_ota_token());
        cfg.ota_token = "private-test-ota-token-2026";
        assert!(cfg.is_valid());
        assert!(!cfg.is_default_ota_token());
        assert_eq!(cfg.http_port(), 80);
        assert_eq!(cfg.static_ip().prefix_len, 24);

        // LOGIC-171: IP and Gateway cannot be identical or on different subnets
        let mut bad_same_ip = static_cfg;
        bad_same_ip.ip = bad_same_ip.gateway;
        assert!(!bad_same_ip.is_valid());

        let mut bad_diff_subnet = static_cfg;
        bad_diff_subnet.gateway = Ipv4Addr::new(10, 0, 0, 1);
        assert!(!bad_diff_subnet.is_valid());

        // LOGIC-172: NvsConfigOverrides application
        let overrides = NvsConfigOverrides {
            vflip: Some(true),
            hmirror: Some(true),
            quality: Some(20),
            frame_size: Some(FrameSize::Vga),
            ..Default::default()
        };
        let overridden = cfg
            .with_nvs_overrides(overrides)
            .expect("valid NVS overrides");
        assert!(overridden.vflip);
        assert!(overridden.hmirror);
        assert_eq!(overridden.quality, 20);
        assert_eq!(overridden.frame_size, FrameSize::Vga);

        let mut env_cfg = KlipperCamConfig::from_build_env();
        if env_cfg.is_default_ota_token() || !env_cfg.has_valid_ota_token() {
            assert!(!env_cfg.is_valid());
            env_cfg.ota_token = "private-build-env-ota-token";
        }
        #[cfg(feature = "provisioning-ap")]
        if env_cfg.network.wifi_ssid.is_empty() {
            env_cfg.network.wifi_ssid = "test-build-env-ssid";
        }
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
        #[cfg(not(feature = "provisioning-ap"))]
        assert!(!token.is_empty());
        #[cfg(feature = "provisioning-ap")]
        assert_ne!(token, DEFAULT_OTA_TOKEN);
    }

    #[test]
    fn test_compile_time_invariants_and_constants() {
        assert_ne!(HTTP_PORT, STREAM_PORT);
        assert!((1..=63).contains(&JPEG_QUALITY));
        #[cfg(not(feature = "provisioning-ap"))]
        assert!(!OTA_TOKEN.is_empty());
        #[cfg(feature = "provisioning-ap")]
        assert_ne!(OTA_TOKEN, DEFAULT_OTA_TOKEN);
    }

    // ---- WP-09 / FW-NET-03 + FW-NET-04: fail-closed boot configuration ----

    #[test]
    fn test_wp09_wifi_auth_mode_fails_closed() {
        // Empty passphrase is an Open network: only with the explicit opt-in.
        assert_eq!(
            wifi_auth_mode("", false),
            Err(ConfigError::OpenWifiNotAllowed)
        );
        assert_eq!(wifi_auth_mode("", true), Ok(WifiAuthMode::Open));

        // WPA2-PSK passphrases must be 8..=63 bytes; anything else is fatal,
        // never a silent fallback to Open.
        assert_eq!(
            wifi_auth_mode("short", false),
            Err(ConfigError::InvalidPassphraseLength(5))
        );
        assert_eq!(
            wifi_auth_mode("short", true),
            Err(ConfigError::InvalidPassphraseLength(5))
        );
        let seventy = "x".repeat(70);
        assert_eq!(
            wifi_auth_mode(&seventy, true),
            Err(ConfigError::InvalidPassphraseLength(70))
        );
        assert_eq!(
            wifi_auth_mode("12345678", false),
            Ok(WifiAuthMode::WpaWpa2Personal)
        );
        let max = "y".repeat(63);
        assert_eq!(
            wifi_auth_mode(&max, false),
            Ok(WifiAuthMode::WpaWpa2Personal)
        );
        assert!(is_valid_wifi_passphrase_len(0));
        assert!(!is_valid_wifi_passphrase_len(7));
        assert!(is_valid_wifi_passphrase_len(8));
        assert!(is_valid_wifi_passphrase_len(63));
        assert!(!is_valid_wifi_passphrase_len(64));
    }

    #[test]
    fn test_wp09_boot_wifi_auth_checks_is_valid_first() {
        let mut cfg = KlipperCamConfig::WROVER_KIT_HD;
        cfg.ota_token = "private-boot-auth-ota-token";
        cfg.network.wifi_password = "correct-horse";
        assert_eq!(cfg.boot_wifi_auth(false), Ok(WifiAuthMode::WpaWpa2Personal));

        let mut bad_ssid = cfg;
        bad_ssid.network.wifi_ssid = "";
        assert_eq!(
            bad_ssid.boot_wifi_auth(true),
            Err(ConfigError::InvalidConfig)
        );

        let mut bad_ports = cfg;
        bad_ports.stream_port = bad_ports.control_port;
        assert_eq!(
            bad_ports.boot_wifi_auth(true),
            Err(ConfigError::InvalidConfig)
        );

        let mut open = cfg;
        open.network.wifi_password = "";
        assert_eq!(
            open.boot_wifi_auth(false),
            Err(ConfigError::OpenWifiNotAllowed)
        );
        assert_eq!(open.boot_wifi_auth(true), Ok(WifiAuthMode::Open));

        let mut bad_pass = cfg;
        bad_pass.network.wifi_password = "1234567";
        assert_eq!(
            bad_pass.boot_wifi_auth(true),
            Err(ConfigError::InvalidConfig)
        );
    }

    #[test]
    fn test_wp09_config_error_display_is_actionable() {
        extern crate alloc;
        use alloc::string::ToString;
        assert!(ConfigError::OpenWifiNotAllowed
            .to_string()
            .contains("allow-open-wifi"));
        assert!(ConfigError::InvalidPassphraseLength(70)
            .to_string()
            .contains("70"));
        assert!(!ConfigError::InvalidConfig.to_string().is_empty());
    }

    #[test]
    fn test_wp09_allow_open_wifi_tracks_feature() {
        assert_eq!(ALLOW_OPEN_WIFI, cfg!(feature = "allow-open-wifi"));
    }

    #[test]
    fn test_wp09_default_token_detection() {
        let mut cfg = KlipperCamConfig::WROVER_KIT_HD;
        cfg.ota_token = DEFAULT_OTA_TOKEN;
        assert!(cfg.is_default_ota_token());
        cfg.ota_token = "a-real-secret";
        assert!(!cfg.is_default_ota_token());
    }

    #[test]
    fn test_m031_is_valid_and_nvs_overrides_reject_weak_and_invalid() {
        let mut cfg = KlipperCamConfig::WROVER_KIT_HD;
        cfg.ota_token = DEFAULT_OTA_TOKEN;
        assert!(!cfg.is_valid(), "DEFAULT_OTA_TOKEN must be rejected");

        cfg.ota_token = "too-short-15chr";
        assert!(!cfg.is_valid(), "tokens < 16 bytes must be rejected");

        // M-031: whitespace and non-graphic ASCII tokens must be rejected.
        for bad in [
            "                ",
            " klipper-cam-ota-2026",
            "valid-16-byte-tk ",
            "valid 16 byte tk",
            "\tvalid-16-byte-tk",
            "valid-16-byte-t\x7F",
        ] {
            assert!(!is_valid_ota_token(bad.as_bytes()), "bad token: {bad:?}");
            cfg.ota_token = bad;
            assert!(!cfg.is_valid(), "is_valid must reject {bad:?}");
        }

        cfg.ota_token = "valid-16-byte-tk";
        assert!(cfg.is_valid());

        // M-031: NVS overrides must also reject whitespace/non-graphic tokens.
        for bad in [
            "                ",
            " klipper-cam-ota-2026",
            "valid 16 byte tk",
        ] {
            assert_eq!(
                cfg.with_nvs_overrides(NvsConfigOverrides {
                    ota_token: Some(bad),
                    ..Default::default()
                }),
                Err(ConfigError::InvalidConfig),
                "with_nvs_overrides must reject {bad:?}"
            );
        }

        let mut q0 = cfg;
        q0.quality = 0;
        assert!(!q0.is_valid(), "quality 0 must be rejected");
        let mut q64 = cfg;
        q64.quality = 64;
        assert!(!q64.is_valid(), "quality 64 must be rejected");
        let mut q1 = cfg;
        q1.quality = 1;
        assert!(q1.is_valid());
        let mut q63 = cfg;
        q63.quality = 63;
        assert!(q63.is_valid());

        assert_eq!(
            cfg.with_nvs_overrides(NvsConfigOverrides {
                ota_token: Some(DEFAULT_OTA_TOKEN),
                ..Default::default()
            }),
            Err(ConfigError::InvalidConfig)
        );
        assert_eq!(
            cfg.with_nvs_overrides(NvsConfigOverrides {
                ota_token: Some("short-token"),
                ..Default::default()
            }),
            Err(ConfigError::InvalidConfig)
        );
        assert_eq!(
            cfg.with_nvs_overrides(NvsConfigOverrides {
                quality: Some(0),
                ..Default::default()
            }),
            Err(ConfigError::InvalidConfig)
        );
        assert_eq!(
            cfg.with_nvs_overrides(NvsConfigOverrides {
                quality: Some(64),
                ..Default::default()
            }),
            Err(ConfigError::InvalidConfig)
        );
        assert_eq!(
            cfg.with_nvs_overrides(NvsConfigOverrides {
                wifi_ssid: Some(""),
                ..Default::default()
            }),
            Err(ConfigError::InvalidConfig)
        );
        assert_eq!(
            cfg.with_nvs_overrides(NvsConfigOverrides {
                wifi_password: Some("short"),
                ..Default::default()
            }),
            Err(ConfigError::InvalidPassphraseLength(5))
        );
        if !ALLOW_OPEN_WIFI {
            assert_eq!(
                cfg.with_nvs_overrides(NvsConfigOverrides {
                    wifi_password: Some(""),
                    ..Default::default()
                }),
                Err(ConfigError::OpenWifiNotAllowed)
            );
        }
        let mut bad_ip = StaticIpConfig::KLIPPER_DEFAULT;
        bad_ip.ip = bad_ip.gateway;
        assert_eq!(
            cfg.with_nvs_overrides(NvsConfigOverrides {
                static_ip: Some(bad_ip),
                ..Default::default()
            }),
            Err(ConfigError::InvalidConfig)
        );
    }

    #[test]
    fn test_m134_nvs_config_overrides_redacts_secrets_in_debug() {
        extern crate alloc;
        use alloc::format;
        let overrides = NvsConfigOverrides {
            wifi_ssid: Some("printer-net"),
            wifi_password: Some("super-secret-wifi-pass"),
            ota_token: Some("super-secret-ota-token-value"),
            quality: Some(15),
            ..Default::default()
        };
        let rendered = format!("{:?}", overrides);
        assert!(rendered.contains("printer-net"));
        assert!(rendered.contains("[REDACTED]"));
        assert!(!rendered.contains("super-secret-wifi-pass"));
        assert!(!rendered.contains("super-secret-ota-token-value"));
    }

    // ---- SoftAP Provisioning: IpMode, credentials, Review Focus #1 ----

    #[test]
    fn test_ip_mode_default_and_validity() {
        assert_eq!(
            IpMode::default(),
            IpMode::Static(StaticIpConfig::KLIPPER_DEFAULT)
        );
        assert_eq!(
            NetworkConfig::DEFAULT.ip_mode,
            IpMode::Static(StaticIpConfig::KLIPPER_DEFAULT)
        );
        assert_eq!(
            KlipperCamConfig::WROVER_KIT_HD.ip_mode(),
            IpMode::Static(StaticIpConfig::KLIPPER_DEFAULT)
        );
        assert!(IpMode::Dhcp.is_valid());
        assert!(IpMode::Static(StaticIpConfig::KLIPPER_DEFAULT).is_valid());

        let mut bad_same = StaticIpConfig::KLIPPER_DEFAULT;
        bad_same.ip = bad_same.gateway;
        assert!(!IpMode::Static(bad_same).is_valid());

        let mut bad_prefix = StaticIpConfig::KLIPPER_DEFAULT;
        bad_prefix.prefix_len = 0;
        assert!(!IpMode::Static(bad_prefix).is_valid());

        let mut bad_subnet = StaticIpConfig::KLIPPER_DEFAULT;
        bad_subnet.gateway = Ipv4Addr::new(10, 0, 0, 1);
        assert!(!IpMode::Static(bad_subnet).is_valid());
    }

    #[test]
    fn test_review_focus_1_fresh_build_env_and_default_leakage() {
        assert_eq!(
            has_explicit_build_wifi_ssid(),
            COMPILE_TIME_WIFI_SSID.is_some_and(|s| !s.is_empty())
        );
        assert_eq!(
            has_explicit_build_wifi_pass(),
            COMPILE_TIME_WIFI_PASS.is_some_and(|p| (8..=63).contains(&p.len()))
        );

        let env_cfg = KlipperCamConfig::from_build_env();
        if cfg!(feature = "provisioning-ap") {
            let any_ota_env = option_env!("CAM_OTA_TOKEN").is_some_and(|s| !s.is_empty())
                || option_env!("OTA_TOKEN").is_some_and(|s| !s.is_empty())
                || COMPILE_TIME_OTA_TOKEN.is_some_and(|s| !s.is_empty())
                || option_env!("ESP32_CAM_OTA_TOKEN").is_some_and(|s| !s.is_empty());
            if !any_ota_env {
                assert_eq!(
                    active_ota_token(),
                    "",
                    "unset OTA env under provisioning-ap must return empty string"
                );
                assert_eq!(env_cfg.ota_token, "");
                assert!(!env_cfg.has_valid_ota_token());
            }
            if !has_explicit_build_wifi_ssid() {
                assert_eq!(
                    env_cfg.network.wifi_ssid, "",
                    "unset WIFI_SSID under provisioning-ap must not leak DEFAULT_WIFI_SSID"
                );
                assert!(!env_cfg.has_sta_credentials());
            }
        }

        // Built-in DEFAULT_WIFI_SSID ("iot") + empty pass + DEFAULT_OTA_TOKEN
        // must never satisfy has_valid_ota_token() and must never satisfy
        // has_sta_credentials() under provisioning-ap (or without open-wifi).
        let mut fallback_cfg = KlipperCamConfig::WROVER_KIT_HD;
        fallback_cfg.network.wifi_ssid = DEFAULT_WIFI_SSID;
        fallback_cfg.network.wifi_password = DEFAULT_WIFI_PASS;
        fallback_cfg.ota_token = DEFAULT_OTA_TOKEN;
        assert!(!fallback_cfg.has_valid_ota_token());
        if cfg!(feature = "provisioning-ap") && !has_explicit_build_wifi_ssid() {
            assert!(
                !fallback_cfg.has_sta_credentials(),
                "built-in DEFAULT_WIFI_SSID + empty pass must fail has_sta_credentials()"
            );
        } else if !ALLOW_OPEN_WIFI {
            assert!(!fallback_cfg.has_sta_credentials());
        }
    }

    #[test]
    fn test_has_sta_credentials_and_has_valid_ota_token_matrix() {
        let mut cfg = KlipperCamConfig::WROVER_KIT_HD;

        // Empty or > 32 B SSID fails.
        cfg.network.wifi_ssid = "";
        cfg.network.wifi_password = "valid-passphrase";
        assert!(!cfg.has_sta_credentials());

        cfg.network.wifi_ssid = "123456789012345678901234567890123";
        assert!(!cfg.has_sta_credentials());

        // 32 B SSID + 8 B passphrase succeeds.
        cfg.network.wifi_ssid = "12345678901234567890123456789012";
        cfg.network.wifi_password = "12345678";
        assert!(cfg.has_sta_credentials());

        // 7 B and 64 B passphrases fail; 63 B succeeds.
        cfg.network.wifi_password = "1234567";
        assert!(!cfg.has_sta_credentials());

        cfg.network.wifi_password =
            "1234567890123456789012345678901234567890123456789012345678901234";
        assert!(!cfg.has_sta_credentials());

        cfg.network.wifi_password =
            "123456789012345678901234567890123456789012345678901234567890123";
        assert!(cfg.has_sta_credentials());

        // Real WPA2 network genuinely named DEFAULT_WIFI_SSID ("iot") succeeds.
        cfg.network.wifi_ssid = DEFAULT_WIFI_SSID;
        cfg.network.wifi_password = "real-iot-passphrase";
        assert!(cfg.has_sta_credentials());

        // OTA token validity checks.
        cfg.ota_token = "";
        assert!(!cfg.has_valid_ota_token());
        cfg.ota_token = DEFAULT_OTA_TOKEN;
        assert!(!cfg.has_valid_ota_token());
        cfg.ota_token = "short-15-bytes!";
        assert!(!cfg.has_valid_ota_token());
        cfg.ota_token = "valid-16-byte-tk";
        assert!(cfg.has_valid_ota_token());
    }

    #[test]
    fn test_with_nvs_overrides_ip_mode_dhcp_and_static() {
        extern crate alloc;
        use alloc::format;

        let base = KlipperCamConfig::from_build_env();
        let dhcp_overrides = NvsConfigOverrides {
            wifi_ssid: Some("KlipperShop"),
            wifi_password: Some("wpa2-shop-secret"),
            ota_token: Some("private-shop-ota-token-2026"),
            ip_mode: Some(IpMode::Dhcp),
            ..Default::default()
        };
        let rendered = format!("{dhcp_overrides:?}");
        assert!(rendered.contains("Dhcp"));
        assert!(!rendered.contains("wpa2-shop-secret"));
        assert!(!rendered.contains("private-shop-ota-token-2026"));

        let dhcp_cfg = base
            .with_nvs_overrides(dhcp_overrides)
            .expect("DHCP provisioned overrides must succeed");
        assert!(dhcp_cfg.has_sta_credentials());
        assert!(dhcp_cfg.has_valid_ota_token());
        assert!(dhcp_cfg.is_valid());
        assert_eq!(dhcp_cfg.network.ip_mode, IpMode::Dhcp);
        assert_eq!(dhcp_cfg.ip_mode(), IpMode::Dhcp);

        // Switching to Static via ip_mode keeps static_ip synchronized.
        let custom_static = StaticIpConfig {
            ip: Ipv4Addr::new(192, 168, 50, 52),
            netmask: Ipv4Addr::new(255, 255, 255, 0),
            gateway: Ipv4Addr::new(192, 168, 50, 1),
            dns: Ipv4Addr::new(192, 168, 50, 1),
            prefix_len: 24,
        };
        let static_cfg = dhcp_cfg
            .with_nvs_overrides(NvsConfigOverrides {
                ip_mode: Some(IpMode::Static(custom_static)),
                ..Default::default()
            })
            .expect("Static IpMode override must succeed");
        assert_eq!(static_cfg.network.ip_mode, IpMode::Static(custom_static));
        assert_eq!(*static_cfg.static_ip(), custom_static);
        assert!(static_cfg.is_valid());

        // Setting static_ip directly also keeps ip_mode synchronized.
        let legacy = StaticIpConfig::LEGACY_FALLBACK;
        let synced_cfg = dhcp_cfg
            .with_nvs_overrides(NvsConfigOverrides {
                static_ip: Some(legacy),
                ..Default::default()
            })
            .expect("static_ip override must succeed");
        assert_eq!(synced_cfg.network.ip_mode, IpMode::Static(legacy));
        assert_eq!(*synced_cfg.static_ip(), legacy);

        // Invalid StaticIpConfig inside IpMode::Static is rejected.
        let mut bad_ip = custom_static;
        bad_ip.ip = bad_ip.gateway;
        assert_eq!(
            dhcp_cfg.with_nvs_overrides(NvsConfigOverrides {
                ip_mode: Some(IpMode::Static(bad_ip)),
                ..Default::default()
            }),
            Err(ConfigError::InvalidConfig)
        );

        let mut bad_prefix = custom_static;
        bad_prefix.prefix_len = 33;
        assert_eq!(
            dhcp_cfg.with_nvs_overrides(NvsConfigOverrides {
                ip_mode: Some(IpMode::Static(bad_prefix)),
                ..Default::default()
            }),
            Err(ConfigError::InvalidConfig)
        );

        let mut bad_gw = custom_static;
        bad_gw.gateway = Ipv4Addr::new(10, 0, 0, 1);
        assert_eq!(
            dhcp_cfg.with_nvs_overrides(NvsConfigOverrides {
                ip_mode: Some(IpMode::Static(bad_gw)),
                ..Default::default()
            }),
            Err(ConfigError::InvalidConfig)
        );
    }
}
