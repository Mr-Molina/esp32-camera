//! Per-driver correctness tests and the shared 18-driver conformance suite
//! for `WP-23` (non-Ov2640 sensor driver correctness).

use esp32_cam_sccb::MockSccbBus;
use esp32_cam_sensors::hm0360::Hm0360;
use esp32_cam_sensors::hm1055::{Hm1055, CKCFG2};
use esp32_cam_sensors::nt99141::Nt99141;
use esp32_cam_sensors::ov3640::Ov3640;
use esp32_cam_sensors::CameraSensor;
use esp32_cam_types::{CameraError, CameraSccbAddr, FrameSize, GainCeiling, PixelFormat};

/// NT99141 auto-exposure level register, first of the five written per level.
const NT99141_AE_REG: u16 = 0x32B8;
/// NT99141 brightness / contrast value register.
const NT99141_BRIGHTNESS_REG: u16 = 0x32F2;

// ---------------------------------------------------------------------------
// SENS-05: NT99141 `set_ae_level` indexing
// ---------------------------------------------------------------------------

/// The AE table holds nine five-register groups for levels -4..=4. Indexing it
/// with `level.clamp(0, 8)` made level 0 select group 0, the *darkest* preset,
/// so a sensor reset left the image four stops under.
#[test]
fn test_sens05_reset_selects_the_neutral_ae_group() {
    let mut bus: MockSccbBus<256> = MockSccbBus::new();
    let mut sensor = Nt99141::new();

    assert_eq!(sensor.reset(&mut bus), Ok(()));

    assert_eq!(
        bus.reg16(CameraSccbAddr::NT99141, NT99141_AE_REG),
        0x3B,
        "reset() must leave the neutral AE group (level 0) applied"
    );
}

#[test]
fn test_sens05_set_ae_level_spans_the_signed_range() {
    for (level, expected) in [(-4i32, 0x29u8), (0, 0x3B), (4, 0x4D)] {
        let mut bus: MockSccbBus<256> = MockSccbBus::new();
        let mut sensor = Nt99141::new();

        assert_eq!(sensor.set_ae_level(&mut bus, level), Ok(()));
        assert_eq!(
            bus.reg16(CameraSccbAddr::NT99141, NT99141_AE_REG),
            expected,
            "ae level {level} selected the wrong group"
        );
        assert_eq!(
            sensor.status().ae_level,
            level as i8,
            "status must record the requested level, not the table index"
        );
    }
}

#[test]
fn test_sens05_set_ae_level_clamps_out_of_range() {
    let mut bus: MockSccbBus<256> = MockSccbBus::new();
    let mut sensor = Nt99141::new();

    assert_eq!(sensor.set_ae_level(&mut bus, 99), Ok(()));
    assert_eq!(bus.reg16(CameraSccbAddr::NT99141, NT99141_AE_REG), 0x4D);

    assert_eq!(sensor.set_ae_level(&mut bus, -99), Ok(()));
    assert_eq!(bus.reg16(CameraSccbAddr::NT99141, NT99141_AE_REG), 0x29);
}

// ---------------------------------------------------------------------------
// SENS-11: NT99141 brightness level 0
// ---------------------------------------------------------------------------

/// Brightness and contrast share register 0x32F2, where 0x80 is the neutral
/// midpoint. `set_brightness(0)` fell through to the catch-all arm and wrote
/// 0x00, the darkest value, which also undid whatever contrast had set.
#[test]
fn test_sens11_brightness_zero_is_the_neutral_midpoint() {
    let mut bus: MockSccbBus<256> = MockSccbBus::new();
    let mut sensor = Nt99141::new();

    assert_eq!(sensor.set_brightness(&mut bus, 0), Ok(()));
    assert_eq!(
        bus.reg16(CameraSccbAddr::NT99141, NT99141_BRIGHTNESS_REG),
        0x80,
        "brightness 0 must write the neutral midpoint"
    );
    assert_eq!(sensor.status().brightness, 0);
}

#[test]
fn test_sens11_brightness_rejects_out_of_range_levels() {
    let mut bus: MockSccbBus<256> = MockSccbBus::new();
    let mut sensor = Nt99141::new();

    for level in [-4i32, 4, 99, -99] {
        assert_eq!(
            sensor.set_brightness(&mut bus, level),
            Err(CameraError::InvalidArg),
            "brightness {level} is outside -3..=3 and must be rejected"
        );
    }
}

#[test]
fn test_sens11_brightness_in_range_levels_are_monotonic() {
    let expected = [
        (-3i32, 0x60u8),
        (-2, 0x70),
        (-1, 0x78),
        (0, 0x80),
        (1, 0x88),
        (2, 0x90),
        (3, 0xA0),
    ];
    for (level, want) in expected {
        let mut bus: MockSccbBus<256> = MockSccbBus::new();
        let mut sensor = Nt99141::new();
        assert_eq!(sensor.set_brightness(&mut bus, level), Ok(()));
        assert_eq!(
            bus.reg16(CameraSccbAddr::NT99141, NT99141_BRIGHTNESS_REG),
            want,
            "brightness {level}"
        );
    }
}

// ---------------------------------------------------------------------------
// SENS-04: OV3640 window / format / PLL / JPEG registers are OV3660's
// ---------------------------------------------------------------------------

/// `ov3640.rs` addresses the window block at 0x38xx, the PLL at 0x303A..0x303D
/// and JPEG quality at 0x4407, and aliases OV3660's ratio and format tables
/// outright. Those are OV3660-generation registers; an OV3640 does not have
/// them. Rather than write another sensor's map into real hardware, the
/// setters that depend on those blocks refuse until a verified OV3640 map is
/// ported (work order §7, question 6).
#[test]
fn test_sens04_ov3640_window_format_setters_refuse() {
    let mut bus: MockSccbBus<256> = MockSccbBus::new();
    let mut sensor = Ov3640::new();

    assert_eq!(
        sensor.set_framesize(&mut bus, FrameSize::Vga),
        Err(CameraError::NotSupported)
    );
    assert_eq!(
        sensor.set_pixformat(&mut bus, PixelFormat::Jpeg),
        Err(CameraError::NotSupported)
    );
    assert_eq!(
        sensor.set_quality(&mut bus, 10),
        Err(CameraError::NotSupported)
    );
    assert_eq!(
        sensor.set_pll(&mut bus, 0, 288, 1, 0, 0, 0, 1, 16),
        Err(CameraError::NotSupported)
    );
    assert_eq!(
        sensor.set_res_raw(&mut bus, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, false, false),
        Err(CameraError::NotSupported)
    );

    assert_eq!(
        bus.transaction_count(),
        0,
        "a refusing setter must not touch the bus"
    );
}

// ---------------------------------------------------------------------------
// SENS-10: HM0360 240x240 and unsupported framesizes
// ---------------------------------------------------------------------------

/// A distinctive register from `SENSOR_FRAMESIZE_240X240`.
const HM0360_240_WINDOW_REG: u16 = 0x05E0;

/// `SENSOR_FRAMESIZE_240X240` was defined but never written: a catch-all arm
/// swallowed every size below VGA that had no table of its own and returned
/// `Ok`, leaving the sensor at whatever the default table had set.
#[test]
fn test_sens10_hm0360_240x240_writes_its_table() {
    let mut bus: MockSccbBus<1024> = MockSccbBus::new();
    let mut sensor = Hm0360::new();

    assert_eq!(
        sensor.set_framesize(&mut bus, FrameSize::Size240x240),
        Ok(())
    );
    assert_eq!(
        bus.reg16(CameraSccbAddr::HM0360, HM0360_240_WINDOW_REG),
        0xBE,
        "the 240x240 window table must reach the bus"
    );
    assert_eq!(sensor.status().framesize, FrameSize::Size240x240);
}

#[test]
fn test_sens10_hm0360_rejects_framesizes_with_no_table() {
    for framesize in [
        FrameSize::Size96x96,
        FrameSize::Size128x128,
        FrameSize::Qcif,
        FrameSize::Hqvga,
        FrameSize::Size320x320,
        FrameSize::Cif,
        FrameSize::Hvga,
        FrameSize::Svga,
        FrameSize::Uxga,
    ] {
        let mut bus: MockSccbBus<1024> = MockSccbBus::new();
        let mut sensor = Hm0360::new();
        let before = sensor.status().framesize;

        assert_eq!(
            sensor.set_framesize(&mut bus, framesize),
            Err(CameraError::FailedToSetFrameSize),
            "{framesize:?} has no register table and must be rejected"
        );
        assert_eq!(
            sensor.status().framesize,
            before,
            "a rejected framesize must not be recorded in status"
        );
    }
}

#[test]
fn test_sens10_hm0360_supported_framesizes_still_apply() {
    for framesize in [
        FrameSize::Qqvga,
        FrameSize::Size240x240,
        FrameSize::Qvga,
        FrameSize::Vga,
    ] {
        let mut bus: MockSccbBus<1024> = MockSccbBus::new();
        let mut sensor = Hm0360::new();
        assert_eq!(sensor.set_framesize(&mut bus, framesize), Ok(()));
        assert_eq!(sensor.status().framesize, framesize);
    }
}

// ---------------------------------------------------------------------------
// SENS-13: setters that returned Ok without touching hardware
// ---------------------------------------------------------------------------

/// The trait's contract is that a control the sensor cannot do returns
/// [`CameraError::NotSupported`]. A pile of overrides instead recorded the
/// requested value in `status` and returned `Ok`, so callers — and
/// `/control`, which reads the status back — were told a setting had been
/// applied when nothing had reached the sensor.
#[test]
fn test_sens13_nt99141_unsupported_controls_say_so() {
    let mut bus: MockSccbBus<256> = MockSccbBus::new();
    let mut sensor = Nt99141::new();

    assert_eq!(
        sensor.set_denoise(&mut bus, 4),
        Err(CameraError::NotSupported)
    );
    assert_eq!(
        sensor.set_gainceiling(&mut bus, GainCeiling::X16),
        Err(CameraError::NotSupported)
    );
    assert_eq!(
        sensor.set_whitebal(&mut bus, true),
        Err(CameraError::NotSupported)
    );
    assert_eq!(
        sensor.set_aec2(&mut bus, true),
        Err(CameraError::NotSupported)
    );
    assert_eq!(
        sensor.set_dcw(&mut bus, true),
        Err(CameraError::NotSupported)
    );
    assert_eq!(
        sensor.set_bpc(&mut bus, true),
        Err(CameraError::NotSupported)
    );
    assert_eq!(
        sensor.set_wpc(&mut bus, true),
        Err(CameraError::NotSupported)
    );
    assert_eq!(
        sensor.set_raw_gma(&mut bus, true),
        Err(CameraError::NotSupported)
    );
    assert_eq!(
        sensor.set_lenc(&mut bus, true),
        Err(CameraError::NotSupported)
    );
    assert_eq!(
        sensor.set_res_raw(&mut bus, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, false, false),
        Err(CameraError::NotSupported)
    );
    assert_eq!(
        sensor.set_pll(&mut bus, 0, 0, 0, 0, 0, 0, 0, 0),
        Err(CameraError::NotSupported)
    );

    assert_eq!(
        bus.transaction_count(),
        0,
        "an unsupported control must not touch the bus"
    );
}

#[test]
fn test_sens13_hm1055_unsupported_controls_say_so() {
    let mut bus: MockSccbBus<256> = MockSccbBus::new();
    let mut sensor = Hm1055::new();

    assert_eq!(
        sensor.set_quality(&mut bus, 10),
        Err(CameraError::NotSupported)
    );
    assert_eq!(
        sensor.set_res_raw(&mut bus, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, false, false),
        Err(CameraError::NotSupported)
    );

    assert_eq!(bus.transaction_count(), 0);
}

// ---------------------------------------------------------------------------
// SENS-12: HM1055 PLL multiplier encoding
// ---------------------------------------------------------------------------

/// The multiplier codes are consecutive from 0x10 at mul 204, rising one per
/// step of 12. Two entries were written in decimal (`10`, `11`) rather than
/// hex, so multipliers 204 and 216 programmed codes 0x0A and 0x0B — a
/// different PLL setting entirely.
#[test]
fn test_sens12_pll_multiplier_codes_are_consecutive_from_0x10() {
    for (step, mul) in (204..=360).step_by(12).enumerate() {
        let mut bus: MockSccbBus<256> = MockSccbBus::new();
        let mut sensor = Hm1055::new();

        assert_eq!(
            sensor.set_pll(&mut bus, 0, mul, 1, 0, 0, 0, 1, 16),
            Ok(()),
            "mul {mul} must be accepted"
        );

        let code = bus.reg16(CameraSccbAddr::HM1055, CKCFG2) & 0x1F;
        let want = 0x10 + u8::try_from(step).expect("step fits in u8");
        assert_eq!(code, want, "multiplier {mul} encoded as {code:#04x}");
    }
}

#[test]
fn test_sens12_pll_rejects_unrepresentable_multipliers() {
    for mul in [0i32, 100, 203, 210, 361, 372] {
        let mut bus: MockSccbBus<256> = MockSccbBus::new();
        let mut sensor = Hm1055::new();

        assert_eq!(
            sensor.set_pll(&mut bus, 0, mul, 1, 0, 0, 0, 1, 16),
            Err(CameraError::InvalidArg),
            "mul {mul} has no register encoding and must be rejected"
        );
    }
}

#[test]
fn test_sens12_pll_bypass_ignores_the_multiplier() {
    let mut bus: MockSccbBus<256> = MockSccbBus::new();
    let mut sensor = Hm1055::new();

    // With the PLL bypassed the multiplier is never encoded, so an
    // unrepresentable value is not an error.
    assert_eq!(sensor.set_pll(&mut bus, 1, 0, 1, 0, 0, 0, 1, 16), Ok(()));
    assert_eq!(bus.reg16(CameraSccbAddr::HM1055, CKCFG2) & 0x1F, 0x00);
}
