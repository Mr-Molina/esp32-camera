//! Compile-time board pinout profiles (`examples/camera_example/main/camera_pinout.h`)
//! and GPIO conflict / range validation (`FW-001`, `DMA-102`).

#![forbid(unsafe_code)]

use esp32_cam_ll::CamPinConfig;
use esp32_cam_types::CameraError;

/// Target ESP32 family chip discriminant used for GPIO range and input-only pin validation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum TargetChip {
    /// Original ESP32 (`GPIO 0..=39`, `34..=39` are input-only).
    Esp32,
    /// ESP32-S2 (`GPIO 0..=46`, `46` is input-only).
    Esp32S2,
    /// ESP32-S3 (`GPIO 0..=48`).
    Esp32S3,
}

impl TargetChip {
    /// Returns the maximum valid GPIO index (inclusive) for this chip.
    #[inline]
    pub const fn max_gpio(self) -> u8 {
        match self {
            Self::Esp32 => 39,
            Self::Esp32S2 => 46,
            Self::Esp32S3 => 48,
        }
    }

    /// Returns `true` if `gpio` is an input-only pin on this chip and cannot drive output signals.
    #[inline]
    pub const fn is_input_only(self, gpio: u8) -> bool {
        match self {
            Self::Esp32 => gpio >= 34 && gpio <= 39,
            Self::Esp32S2 => gpio == 46,
            Self::Esp32S3 => false,
        }
    }

    /// Returns `true` if `gpio` is a non-existent physical silicon pad on this chip.
    #[inline]
    pub const fn is_non_existent(self, gpio: u8) -> bool {
        match self {
            Self::Esp32 => gpio == 20 || gpio == 24 || (gpio >= 28 && gpio <= 31),
            Self::Esp32S2 | Self::Esp32S3 => gpio >= 22 && gpio <= 25,
        }
    }

    /// Returns `true` if `gpio` is reserved for integrated SPI flash/PSRAM bus.
    #[inline]
    pub const fn is_reserved_flash(self, gpio: u8) -> bool {
        match self {
            Self::Esp32 => gpio >= 6 && gpio <= 11,
            Self::Esp32S2 | Self::Esp32S3 => gpio >= 26 && gpio <= 32,
        }
    }
}

/// Complete camera pinout profile matching `camera_config_t` and `camera_pinout.h`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct CameraPins {
    pub chip: TargetChip,
    pub pwdn: Option<u8>,
    pub reset: Option<u8>,
    pub xclk: u8,
    pub siod: u8,
    pub sioc: u8,
    pub d7: u8,
    pub d6: u8,
    pub d5: u8,
    pub d4: u8,
    pub d3: u8,
    pub d2: u8,
    pub d1: u8,
    pub d0: u8,
    pub vsync: u8,
    pub href: u8,
    pub pclk: u8,
    pub flash_led: Option<u8>,
}

impl CameraPins {
    /// Returns the 8 parallel data pins `[D0, D1, D2, D3, D4, D5, D6, D7]`.
    #[inline]
    pub const fn data_pins(&self) -> [u8; 8] {
        [
            self.d0, self.d1, self.d2, self.d3, self.d4, self.d5, self.d6, self.d7,
        ]
    }

    /// Converts this pin profile into a low-level [`CamPinConfig`] for `esp32-cam-ll`.
    #[inline]
    pub const fn to_ll_pin_config(&self, vsync_invert: bool) -> CamPinConfig {
        CamPinConfig {
            pin_vsync: self.vsync as i8,
            pin_href: self.href as i8,
            pin_pclk: self.pclk as i8,
            pin_xclk: self.xclk as i8,
            data_pins: [
                self.d0 as i8,
                self.d1 as i8,
                self.d2 as i8,
                self.d3 as i8,
                self.d4 as i8,
                self.d5 as i8,
                self.d6 as i8,
                self.d7 as i8,
            ],
            vsync_invert,
        }
    }

    /// Validates that:
    /// 1. Every assigned GPIO index is within `0..=self.chip.max_gpio()`.
    /// 2. Output-capable pins (`xclk`, `siod`, `sioc`, `pwdn`, `reset`, `flash_led`) are not
    ///    assigned to input-only GPIOs (`34..=39` on ESP32, `46` on ESP32-S2).
    /// 3. No pin is assigned to non-existent silicon GPIOs (`20, 24, 28..=31` on ESP32; `22..=25` on ESP32-S2/S3)
    ///    or reserved SPI Flash GPIOs (`6..=11` on ESP32; `26..=32` on ESP32-S2/S3).
    /// 4. No two camera signals share the same physical GPIO pin (`FW-001` / `DMA-102`).
    pub const fn validate_pins(&self) -> Result<(), CameraError> {
        let max_gpio = self.chip.max_gpio();

        // Check output-capable signals against range, input-only, non-existent, and reserved flash constraints
        let output_pins = [self.xclk, self.siod, self.sioc];
        let mut i = 0;
        while i < output_pins.len() {
            let p = output_pins[i];
            if p > max_gpio
                || self.chip.is_input_only(p)
                || self.chip.is_non_existent(p)
                || self.chip.is_reserved_flash(p)
            {
                return Err(CameraError::InvalidArg);
            }
            i += 1;
        }

        if let Some(p) = self.pwdn {
            if p > max_gpio
                || self.chip.is_input_only(p)
                || self.chip.is_non_existent(p)
                || self.chip.is_reserved_flash(p)
            {
                return Err(CameraError::InvalidArg);
            }
        }
        if let Some(p) = self.reset {
            if p > max_gpio
                || self.chip.is_input_only(p)
                || self.chip.is_non_existent(p)
                || self.chip.is_reserved_flash(p)
            {
                return Err(CameraError::InvalidArg);
            }
        }
        if let Some(p) = self.flash_led {
            if p > max_gpio
                || self.chip.is_input_only(p)
                || self.chip.is_non_existent(p)
                || self.chip.is_reserved_flash(p)
            {
                return Err(CameraError::InvalidArg);
            }
        }

        // Collect all active pins and verify range + uniqueness + existence + flash reservations
        let mut active = [0xFFu8; 17];
        active[0] = self.xclk;
        active[1] = self.siod;
        active[2] = self.sioc;
        active[3] = self.d0;
        active[4] = self.d1;
        active[5] = self.d2;
        active[6] = self.d3;
        active[7] = self.d4;
        active[8] = self.d5;
        active[9] = self.d6;
        active[10] = self.d7;
        active[11] = self.vsync;
        active[12] = self.href;
        active[13] = self.pclk;
        let mut count = 14usize;

        if let Some(p) = self.pwdn {
            active[count] = p;
            count += 1;
        }
        if let Some(p) = self.reset {
            active[count] = p;
            count += 1;
        }
        if let Some(p) = self.flash_led {
            active[count] = p;
            count += 1;
        }

        let mut a = 0;
        while a < count {
            let p = active[a];
            if p > max_gpio
                || self.chip.is_non_existent(p)
                || self.chip.is_reserved_flash(p)
            {
                return Err(CameraError::InvalidArg);
            }
            let mut b = a + 1;
            while b < count {
                if active[a] == active[b] {
                    return Err(CameraError::InvalidArg);
                }
                b += 1;
            }
            a += 1;
        }

        Ok(())
    }
}

/// Trait implemented by compile-time board pinout markers (`camera_pinout.h`).
pub trait BoardPins: Copy + Clone + Default {
    /// Board human-readable identifier.
    const NAME: &'static str;
    /// Static pinout definition for this board.
    const PINS: CameraPins;

    /// Returns the [`CameraPins`] definition for this board instance.
    #[inline]
    fn pins(&self) -> CameraPins {
        Self::PINS
    }

    /// Validates all pins in this board profile (`FW-001` / `DMA-102`).
    #[inline]
    fn validate_pins(&self) -> Result<(), CameraError> {
        Self::PINS.validate_pins()
    }
}

/// ESP-WROVER-KIT camera pinout (`BOARD_WROVER_KIT` in `camera_pinout.h:5-24`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Hash)]
pub struct WroverKitPins;

impl WroverKitPins {
    pub const PWDN: Option<u8> = None;
    pub const RESET: Option<u8> = None;
    pub const XCLK: u8 = 21;
    pub const SIOD: u8 = 26;
    pub const SIOC: u8 = 27;
    pub const D7: u8 = 35;
    pub const D6: u8 = 34;
    pub const D5: u8 = 39;
    pub const D4: u8 = 36;
    pub const D3: u8 = 19;
    pub const D2: u8 = 18;
    pub const D1: u8 = 5;
    pub const D0: u8 = 4;
    pub const VSYNC: u8 = 25;
    pub const HREF: u8 = 23;
    pub const PCLK: u8 = 22;
    pub const FLASH_LED: Option<u8> = None;
}

impl BoardPins for WroverKitPins {
    const NAME: &'static str = "BOARD_WROVER_KIT";
    const PINS: CameraPins = CameraPins {
        chip: TargetChip::Esp32,
        pwdn: Self::PWDN,
        reset: Self::RESET,
        xclk: Self::XCLK,
        siod: Self::SIOD,
        sioc: Self::SIOC,
        d7: Self::D7,
        d6: Self::D6,
        d5: Self::D5,
        d4: Self::D4,
        d3: Self::D3,
        d2: Self::D2,
        d1: Self::D1,
        d0: Self::D0,
        vsync: Self::VSYNC,
        href: Self::HREF,
        pclk: Self::PCLK,
        flash_led: Self::FLASH_LED,
    };
}

/// AI-Thinker ESP32-CAM pinout (`BOARD_ESP32CAM_AITHINKER` in `camera_pinout.h:26-45`).
///
/// Includes `FLASH_LED = Some(4)` on GPIO 4 (shared only with SD 1-bit/4-bit data when SD is unused,
/// distinct from `D0 = 5`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Hash)]
pub struct AiThinkerPins;

impl AiThinkerPins {
    pub const PWDN: Option<u8> = Some(32);
    pub const RESET: Option<u8> = None;
    pub const XCLK: u8 = 0;
    pub const SIOD: u8 = 26;
    pub const SIOC: u8 = 27;
    pub const D7: u8 = 35;
    pub const D6: u8 = 34;
    pub const D5: u8 = 39;
    pub const D4: u8 = 36;
    pub const D3: u8 = 21;
    pub const D2: u8 = 19;
    pub const D1: u8 = 18;
    pub const D0: u8 = 5;
    pub const VSYNC: u8 = 25;
    pub const HREF: u8 = 23;
    pub const PCLK: u8 = 22;
    pub const FLASH_LED: Option<u8> = Some(4);
}

impl BoardPins for AiThinkerPins {
    const NAME: &'static str = "BOARD_ESP32CAM_AITHINKER";
    const PINS: CameraPins = CameraPins {
        chip: TargetChip::Esp32,
        pwdn: Self::PWDN,
        reset: Self::RESET,
        xclk: Self::XCLK,
        siod: Self::SIOD,
        sioc: Self::SIOC,
        d7: Self::D7,
        d6: Self::D6,
        d5: Self::D5,
        d4: Self::D4,
        d3: Self::D3,
        d2: Self::D2,
        d1: Self::D1,
        d0: Self::D0,
        vsync: Self::VSYNC,
        href: Self::HREF,
        pclk: Self::PCLK,
        flash_led: Self::FLASH_LED,
    };
}

/// ESP-EYE camera pinout (`BOARD_ESP_EYE` in `camera_pinout.h`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Hash)]
pub struct EspEyePins;

impl EspEyePins {
    pub const PWDN: Option<u8> = None;
    pub const RESET: Option<u8> = None;
    pub const XCLK: u8 = 4;
    pub const SIOD: u8 = 18;
    pub const SIOC: u8 = 23;
    pub const D7: u8 = 36;
    pub const D6: u8 = 37;
    pub const D5: u8 = 38;
    pub const D4: u8 = 39;
    pub const D3: u8 = 35;
    pub const D2: u8 = 14;
    pub const D1: u8 = 13;
    pub const D0: u8 = 34;
    pub const VSYNC: u8 = 5;
    pub const HREF: u8 = 27;
    pub const PCLK: u8 = 25;
    pub const FLASH_LED: Option<u8> = None;
}

impl BoardPins for EspEyePins {
    const NAME: &'static str = "BOARD_ESP_EYE";
    const PINS: CameraPins = CameraPins {
        chip: TargetChip::Esp32,
        pwdn: Self::PWDN,
        reset: Self::RESET,
        xclk: Self::XCLK,
        siod: Self::SIOD,
        sioc: Self::SIOC,
        d7: Self::D7,
        d6: Self::D6,
        d5: Self::D5,
        d4: Self::D4,
        d3: Self::D3,
        d2: Self::D2,
        d1: Self::D1,
        d0: Self::D0,
        vsync: Self::VSYNC,
        href: Self::HREF,
        pclk: Self::PCLK,
        flash_led: Self::FLASH_LED,
    };
}

/// ESP32-S3 WROOM CAM pinout (`BOARD_ESP32S3_WROOM` in `camera_pinout.h:47-64`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Hash)]
pub struct Esp32S3WroomPins;

impl Esp32S3WroomPins {
    pub const PWDN: Option<u8> = Some(38);
    pub const RESET: Option<u8> = None;
    pub const VSYNC: u8 = 6;
    pub const HREF: u8 = 7;
    pub const PCLK: u8 = 13;
    pub const XCLK: u8 = 15;
    pub const SIOD: u8 = 4;
    pub const SIOC: u8 = 5;
    pub const D0: u8 = 11;
    pub const D1: u8 = 9;
    pub const D2: u8 = 8;
    pub const D3: u8 = 10;
    pub const D4: u8 = 12;
    pub const D5: u8 = 18;
    pub const D6: u8 = 17;
    pub const D7: u8 = 16;
    pub const FLASH_LED: Option<u8> = None;
}

impl BoardPins for Esp32S3WroomPins {
    const NAME: &'static str = "BOARD_ESP32S3_WROOM";
    const PINS: CameraPins = CameraPins {
        chip: TargetChip::Esp32S3,
        pwdn: Self::PWDN,
        reset: Self::RESET,
        xclk: Self::XCLK,
        siod: Self::SIOD,
        sioc: Self::SIOC,
        d7: Self::D7,
        d6: Self::D6,
        d5: Self::D5,
        d4: Self::D4,
        d3: Self::D3,
        d2: Self::D2,
        d1: Self::D1,
        d0: Self::D0,
        vsync: Self::VSYNC,
        href: Self::HREF,
        pclk: Self::PCLK,
        flash_led: Self::FLASH_LED,
    };
}

/// ESP32-S3 GOOUUU TECH CAM pinout (`BOARD_ESP32S3_GOOUUU` in `camera_pinout.h:66-83`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Hash)]
pub struct Esp32S3GoouuuPins;

impl Esp32S3GoouuuPins {
    pub const PWDN: Option<u8> = None;
    pub const RESET: Option<u8> = None;
    pub const VSYNC: u8 = 6;
    pub const HREF: u8 = 7;
    pub const PCLK: u8 = 13;
    pub const XCLK: u8 = 15;
    pub const SIOD: u8 = 4;
    pub const SIOC: u8 = 5;
    pub const D0: u8 = 11;
    pub const D1: u8 = 9;
    pub const D2: u8 = 8;
    pub const D3: u8 = 10;
    pub const D4: u8 = 12;
    pub const D5: u8 = 18;
    pub const D6: u8 = 17;
    pub const D7: u8 = 16;
    pub const FLASH_LED: Option<u8> = None;
}

impl BoardPins for Esp32S3GoouuuPins {
    const NAME: &'static str = "BOARD_ESP32S3_GOOUUU";
    const PINS: CameraPins = CameraPins {
        chip: TargetChip::Esp32S3,
        pwdn: Self::PWDN,
        reset: Self::RESET,
        xclk: Self::XCLK,
        siod: Self::SIOD,
        sioc: Self::SIOC,
        d7: Self::D7,
        d6: Self::D6,
        d5: Self::D5,
        d4: Self::D4,
        d3: Self::D3,
        d2: Self::D2,
        d1: Self::D1,
        d0: Self::D0,
        vsync: Self::VSYNC,
        href: Self::HREF,
        pclk: Self::PCLK,
        flash_led: Self::FLASH_LED,
    };
}

/// Seeed Studio XIAO ESP32S3 Sense camera pinout (`BOARD_ESP32S3_XIAO` in `camera_pinout.h:85-102`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Hash)]
pub struct Esp32S3XiaoPins;

impl Esp32S3XiaoPins {
    pub const PWDN: Option<u8> = None;
    pub const RESET: Option<u8> = None;
    pub const VSYNC: u8 = 38;
    pub const HREF: u8 = 47;
    pub const PCLK: u8 = 13;
    pub const XCLK: u8 = 10;
    pub const SIOD: u8 = 40;
    pub const SIOC: u8 = 39;
    pub const D0: u8 = 15;
    pub const D1: u8 = 17;
    pub const D2: u8 = 18;
    pub const D3: u8 = 16;
    pub const D4: u8 = 14;
    pub const D5: u8 = 12;
    pub const D6: u8 = 11;
    pub const D7: u8 = 48;
    pub const FLASH_LED: Option<u8> = None;
}

impl BoardPins for Esp32S3XiaoPins {
    const NAME: &'static str = "BOARD_ESP32S3_XIAO";
    const PINS: CameraPins = CameraPins {
        chip: TargetChip::Esp32S3,
        pwdn: Self::PWDN,
        reset: Self::RESET,
        xclk: Self::XCLK,
        siod: Self::SIOD,
        sioc: Self::SIOC,
        d7: Self::D7,
        d6: Self::D6,
        d5: Self::D5,
        d4: Self::D4,
        d3: Self::D3,
        d2: Self::D2,
        d1: Self::D1,
        d0: Self::D0,
        vsync: Self::VSYNC,
        href: Self::HREF,
        pclk: Self::PCLK,
        flash_led: Self::FLASH_LED,
    };
}

impl Default for CameraPins {
    #[inline]
    fn default() -> Self {
        WroverKitPins::PINS
    }
}

/// Allows runtime/custom [`CameraPins`] configurations to be used wherever a [`BoardPins`] is expected.
impl BoardPins for CameraPins {
    const NAME: &'static str = "CUSTOM_PINS";
    const PINS: CameraPins = WroverKitPins::PINS;

    #[inline]
    fn pins(&self) -> CameraPins {
        *self
    }

    #[inline]
    fn validate_pins(&self) -> Result<(), CameraError> {
        CameraPins::validate_pins(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_all_five_board_pinouts_match_c_header_and_validate() {
        assert_eq!(WroverKitPins.validate_pins(), Ok(()));
        assert_eq!(AiThinkerPins.validate_pins(), Ok(()));
        assert_eq!(EspEyePins.validate_pins(), Ok(()));
        assert_eq!(Esp32S3WroomPins.validate_pins(), Ok(()));
        assert_eq!(Esp32S3GoouuuPins.validate_pins(), Ok(()));
        assert_eq!(Esp32S3XiaoPins.validate_pins(), Ok(()));

        // Verify exact pin numbers against examples/camera_example/main/camera_pinout.h
        assert_eq!(WroverKitPins::XCLK, 21);
        assert_eq!(WroverKitPins::D0, 4);
        assert_eq!(WroverKitPins::PWDN, None);

        assert_eq!(AiThinkerPins::PWDN, Some(32));
        assert_eq!(AiThinkerPins::XCLK, 0);
        assert_eq!(AiThinkerPins::D0, 5);
        assert_eq!(AiThinkerPins::FLASH_LED, Some(4));

        assert_eq!(EspEyePins::PWDN, None);
        assert_eq!(EspEyePins::RESET, None);
        assert_eq!(EspEyePins::XCLK, 4);
        assert_eq!(EspEyePins::D0, 34);
        assert_eq!(EspEyePins::D7, 36);

        assert_eq!(Esp32S3WroomPins::PWDN, Some(38));
        assert_eq!(Esp32S3GoouuuPins::PWDN, None);
        assert_eq!(Esp32S3XiaoPins::D7, 48);
        assert_eq!(Esp32S3XiaoPins::HREF, 47);
    }

    #[test]
    fn test_validate_pins_rejects_out_of_range_and_conflicts() {
        // Out of range GPIO on ESP32 (>= 40)
        let mut bad_range = WroverKitPins::PINS;
        bad_range.d0 = 40;
        assert_eq!(bad_range.validate_pins(), Err(CameraError::InvalidArg));

        // Output signal assigned to ESP32 input-only pin (GPIO 34)
        let mut bad_output = WroverKitPins::PINS;
        bad_output.xclk = 34;
        assert_eq!(bad_output.validate_pins(), Err(CameraError::InvalidArg));

        // Duplicate GPIO conflict (D0 == D1)
        let mut dup = AiThinkerPins::PINS;
        dup.d1 = dup.d0;
        assert_eq!(dup.validate_pins(), Err(CameraError::InvalidArg));

        // HAL-006: Non-existent silicon GPIOs on ESP32 (20, 24, 28..=31)
        for &gpio in &[20, 24, 28, 29, 30, 31] {
            let mut bad_silicon = WroverKitPins::PINS;
            bad_silicon.d0 = gpio;
            assert_eq!(bad_silicon.validate_pins(), Err(CameraError::InvalidArg));
        }

        // HAL-006: Reserved SPI flash GPIOs on ESP32 (6..=11)
        for gpio in 6..=11 {
            let mut bad_flash = WroverKitPins::PINS;
            bad_flash.d0 = gpio;
            assert_eq!(bad_flash.validate_pins(), Err(CameraError::InvalidArg));
        }

        // HAL-006: Non-existent silicon GPIOs on ESP32-S3 (22..=25)
        for gpio in 22..=25 {
            let mut bad_s3 = Esp32S3WroomPins::PINS;
            bad_s3.d0 = gpio;
            assert_eq!(bad_s3.validate_pins(), Err(CameraError::InvalidArg));
        }

        // HAL-006: Reserved SPI flash GPIOs on ESP32-S3 (26..=32)
        for gpio in 26..=32 {
            let mut bad_flash_s3 = Esp32S3WroomPins::PINS;
            bad_flash_s3.d0 = gpio;
            assert_eq!(bad_flash_s3.validate_pins(), Err(CameraError::InvalidArg));
        }
    }
}
