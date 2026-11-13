//! Per-driver correctness tests and the shared 18-driver conformance suite
//! for `WP-23` (non-Ov2640 sensor driver correctness).

use esp32_cam_sccb::MockSccbBus;
use esp32_cam_sensors::bf20a6::Bf20a6;
use esp32_cam_sensors::bf3005::Bf3005;
use esp32_cam_sensors::gc0308::Gc0308;
use esp32_cam_sensors::gc032a::Gc032a;
use esp32_cam_sensors::gc2145::Gc2145;
use esp32_cam_sensors::hm0360::Hm0360;
use esp32_cam_sensors::hm1055::{Hm1055, CKCFG2};
use esp32_cam_sensors::mega_ccm::MegaCcm;
use esp32_cam_sensors::nt99141::Nt99141;
use esp32_cam_sensors::ov2640::Ov2640;
use esp32_cam_sensors::ov3640::Ov3640;
use esp32_cam_sensors::ov3660::Ov3660;
use esp32_cam_sensors::ov5640::Ov5640;
use esp32_cam_sensors::ov7670::Ov7670;
use esp32_cam_sensors::ov7725::{self, Ov7725};
use esp32_cam_sensors::sc030iot::Sc030Iot;
use esp32_cam_sensors::sc031gs::Sc031Gs;
use esp32_cam_sensors::sc101iot::Sc101Iot;
use esp32_cam_sensors::{CameraSensor, Reg16};
use esp32_cam_types::{
    CameraError, CameraSccbAddr, FrameSize, GainCeiling, PixelFormat, ALL_FRAMESIZES,
};

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
// SENS-14: signed levels written without a range check
// ---------------------------------------------------------------------------

/// `(level & 0xFF) as u8` silently truncates anything outside a byte, so a
/// level of 300 reached the sensor as 0x2C — an arbitrary setting nobody
/// asked for, and the caller was never told, because the firmware discards
/// errors from `/control` setters.
#[test]
fn test_sens14_ov7725_rejects_levels_that_would_truncate() {
    for level in [300i32, -300, 1000, i32::MIN, i32::MAX] {
        let mut bus: MockSccbBus<256> = MockSccbBus::new();
        let mut sensor = Ov7725::new();

        assert_eq!(
            sensor.set_contrast(&mut bus, level),
            Err(CameraError::InvalidArg),
            "contrast {level} would truncate"
        );
        assert_eq!(
            sensor.set_brightness(&mut bus, level),
            Err(CameraError::InvalidArg),
            "brightness {level} would truncate"
        );
        assert_eq!(bus.transaction_count(), 0);
    }
}

#[test]
fn test_sens14_ov7725_keeps_accepting_representable_levels() {
    let mut bus: MockSccbBus<256> = MockSccbBus::new();
    let mut sensor = Ov7725::new();

    assert_eq!(sensor.set_contrast(&mut bus, -2), Ok(()));
    assert_eq!(bus.reg8(CameraSccbAddr::OV7725, ov7725::CONTRAST), 0xFE);
    assert_eq!(sensor.set_brightness(&mut bus, 2), Ok(()));
    assert_eq!(bus.reg8(CameraSccbAddr::OV7725, ov7725::BRIGHTNESS), 0x02);
}

#[test]
fn test_sens14_sc101iot_rejects_out_of_range_levels() {
    let mut bus: MockSccbBus<256> = MockSccbBus::new();
    let mut sensor = Sc101Iot::new();

    assert_eq!(
        sensor.set_contrast(&mut bus, 300),
        Err(CameraError::InvalidArg)
    );
    // Saturation and sharpness span two registers, so they take the same
    // 0..=0xFFFF range its sibling Sc030Iot already declares.
    assert_eq!(
        sensor.set_saturation(&mut bus, -1),
        Err(CameraError::InvalidArg)
    );
    assert_eq!(
        sensor.set_saturation(&mut bus, 0x1_0000),
        Err(CameraError::InvalidArg)
    );
    assert_eq!(
        sensor.set_sharpness(&mut bus, -1),
        Err(CameraError::InvalidArg)
    );
}

#[test]
fn test_sens14_sc101iot_keeps_accepting_in_range_levels() {
    let mut bus: MockSccbBus<256> = MockSccbBus::new();
    let mut sensor = Sc101Iot::new();

    // Registers are addressed as 0xPPRR: page PP via the page-select
    // register, then an 8-bit write to RR.
    assert_eq!(sensor.set_saturation(&mut bus, 0x1234), Ok(()));
    assert_eq!(bus.reg8(CameraSccbAddr::SC101IOT, 0x49), 0x34);
    assert_eq!(bus.reg8(CameraSccbAddr::SC101IOT, 0x4A), 0x12);
}

/// Drivers that already declared a narrower range keep it.
#[test]
fn test_sens14_bf3005_keeps_its_declared_range() {
    let mut bus: MockSccbBus<256> = MockSccbBus::new();
    let mut sensor = Bf3005::new();

    assert_eq!(sensor.set_contrast(&mut bus, 2), Ok(()));
    assert_eq!(
        sensor.set_contrast(&mut bus, 3),
        Err(CameraError::InvalidArg)
    );
}

// ---------------------------------------------------------------------------
// SENS-18: register-table end markers
// ---------------------------------------------------------------------------

/// Every table writer must apply every entry of the slice it is given.
///
/// There used to be an end-of-table sentinel. In Bf20a6 and Hm1055 it was
/// `0xFFFF`, the *same value* as `REG_DLY`, so the first delay entry in a
/// table would have ended it. In Gc032a, Gc2145, Hm0360 and Nt99141 it was
/// `0x0000`, so a legitimate write to register 0 ended it. Either way the
/// rest of the table was dropped without an error. Slices carry their own
/// length, so the sentinel is gone.
macro_rules! table_writer_cases {
    ($($name:ident => $writer:path;)*) => {
        $(
            #[test]
            fn $name() {
                let mut bus: MockSccbBus<64> = MockSccbBus::new();
                let table = [
                    Reg16::new(0x0012, 0xAA),
                    // A real write to register 0: the old 0x0000 sentinel.
                    Reg16::new(0x0000, 0xBB),
                    // A delay: the old 0xFFFF sentinel in two drivers.
                    Reg16::new(0xFFFF, 5),
                    Reg16::new(0x0034, 0xCC),
                ];

                assert_eq!($writer(&mut bus, 0x30, &table), Ok(()));

                assert_eq!(
                    bus.transaction_count(),
                    table.len(),
                    "every entry of the table must reach the bus"
                );
                assert_eq!(
                    bus.total_delay_ms(),
                    5,
                    "the REG_DLY entry must delay, not terminate the table"
                );
            }
        )*
    };
}

table_writer_cases! {
    test_sens18_bf20a6_writes_every_entry  => esp32_cam_sensors::bf20a6::write_reg_list;
    test_sens18_hm1055_writes_every_entry  => esp32_cam_sensors::hm1055::write_reg_list;
    test_sens18_gc032a_writes_every_entry  => esp32_cam_sensors::gc032a::write_reg_list;
    test_sens18_gc2145_writes_every_entry  => esp32_cam_sensors::gc2145::write_reg_list;
    test_sens18_hm0360_writes_every_entry  => esp32_cam_sensors::hm0360::write_reg_list;
    test_sens18_nt99141_writes_every_entry => esp32_cam_sensors::nt99141::write_reg_list;
}

// ---------------------------------------------------------------------------
// SENS-24: one framesize policy, checked across all 18 drivers
// ---------------------------------------------------------------------------

/// The shared 18-driver conformance sweep (WP-23 acceptance).
///
/// `set_framesize` must obey one policy everywhere:
///
///   * it either applies the size the caller asked for, or refuses;
///   * refusing means `FailedToSetFrameSize` — not `InvalidArg`, and not
///     silently substituting a size the caller did not ask for;
///   * `status().framesize` reports what the sensor was actually set to, so
///     it changes on success and stays put on refusal.
///
/// Gc032a and Gc0308 used to clamp an oversized request down to VGA, write
/// that, record it and return `Ok`, so a caller asking for SVGA got VGA and
/// was told it had succeeded. Ov3660, Ov7725 and Ov7670 refused with
/// `InvalidArg`, which the firmware reports differently.
///
/// Ov5640 and MegaCcm also had non-conforming guards, but on the largest
/// FrameSize discriminant, so they are unreachable and this sweep cannot
/// exercise them. They were corrected by inspection, not by this test.
///
/// `NotSupported` is allowed as a refusal for a driver that cannot do
/// framesizes at all — Ov3640, whose register map is another part's
/// (SENS-04).
macro_rules! framesize_conformance {
    ($($name:ident => $driver:ty;)*) => {
        $(
            #[test]
            fn $name() {
                for framesize in ALL_FRAMESIZES {
                    let mut bus: MockSccbBus<4096> = MockSccbBus::new();
                    let mut sensor = <$driver>::new();
                    let before = sensor.status().framesize;

                    match sensor.set_framesize(&mut bus, framesize) {
                        Ok(()) => assert_eq!(
                            sensor.status().framesize,
                            framesize,
                            "{} accepted {framesize:?} but recorded a different size: \
                             a size it cannot do must be refused, not quietly swapped",
                            stringify!($driver),
                        ),
                        Err(err) => {
                            assert!(
                                matches!(
                                    err,
                                    CameraError::FailedToSetFrameSize
                                        | CameraError::NotSupported
                                ),
                                "{} refused {framesize:?} with {err:?}; the policy is \
                                 FailedToSetFrameSize",
                                stringify!($driver),
                            );
                            assert_eq!(
                                sensor.status().framesize,
                                before,
                                "{} refused {framesize:?} but still recorded it",
                                stringify!($driver),
                            );
                        }
                    }
                }
            }
        )*
    };
}

framesize_conformance! {
    test_sens24_conformance_ov2640   => Ov2640;
    test_sens24_conformance_ov3660   => Ov3660;
    test_sens24_conformance_ov3640   => Ov3640;
    test_sens24_conformance_ov5640   => Ov5640;
    test_sens24_conformance_ov7670   => Ov7670;
    test_sens24_conformance_ov7725   => Ov7725;
    test_sens24_conformance_nt99141  => Nt99141;
    test_sens24_conformance_gc2145   => Gc2145;
    test_sens24_conformance_gc0308   => Gc0308;
    test_sens24_conformance_gc032a   => Gc032a;
    test_sens24_conformance_bf3005   => Bf3005;
    test_sens24_conformance_bf20a6   => Bf20a6;
    test_sens24_conformance_sc101iot => Sc101Iot;
    test_sens24_conformance_sc030iot => Sc030Iot;
    test_sens24_conformance_sc031gs  => Sc031Gs;
    test_sens24_conformance_mega_ccm => MegaCcm;
    test_sens24_conformance_hm1055   => Hm1055;
    test_sens24_conformance_hm0360   => Hm0360;
}

// ---------------------------------------------------------------------------
// Review follow-ups
// ---------------------------------------------------------------------------

/// SENS-11, second half: `set_contrast` writes the same 0x32F2 register as
/// `set_brightness`, and kept the catch-all arm that writes 0x00 — the
/// darkest value — for any level outside -3..=3. `set_contrast(4)` returned
/// `Ok`, blacked the image out and recorded 4 as the applied contrast.
#[test]
fn test_nt99141_contrast_rejects_out_of_range_instead_of_blacking_out() {
    for level in [4i32, -4, 99, -99] {
        let mut bus: MockSccbBus<256> = MockSccbBus::new();
        let mut sensor = Nt99141::new();
        assert_eq!(
            sensor.set_contrast(&mut bus, level),
            Err(CameraError::InvalidArg),
            "contrast {level} is outside -3..=3"
        );
    }
}

#[test]
fn test_nt99141_contrast_in_range_levels_are_monotonic() {
    for (level, want) in [
        (-3i32, 0x50u8),
        (-2, 0x60),
        (-1, 0x70),
        (0, 0x80),
        (1, 0x90),
        (2, 0xA0),
        (3, 0xB0),
    ] {
        let mut bus: MockSccbBus<256> = MockSccbBus::new();
        let mut sensor = Nt99141::new();
        assert_eq!(sensor.set_contrast(&mut bus, level), Ok(()));
        assert_eq!(
            bus.reg16(CameraSccbAddr::NT99141, NT99141_BRIGHTNESS_REG),
            want,
            "contrast {level}"
        );
    }
}

/// SENS-12, narrowed: the original lookup jumped from 240 straight to 288,
/// skipping 252, 264 and 276 — three consecutive steps of 12. That gap is
/// evidence the real encoding is not a uniform sequence, so the computed
/// formula is used only for the multipliers the table actually attested and
/// the three it skipped are refused rather than given invented codes.
#[test]
fn test_hm1055_pll_refuses_the_multipliers_the_table_skipped() {
    for mul in [252i32, 264, 276] {
        let mut bus: MockSccbBus<256> = MockSccbBus::new();
        let mut sensor = Hm1055::new();
        assert_eq!(
            sensor.set_pll(&mut bus, 0, mul, 1, 0, 0, 0, 1, 16),
            Err(CameraError::InvalidArg),
            "multiplier {mul} has no attested encoding"
        );
    }
}

/// SENS-24, second half, applied to the drivers it was missed in: a framesize
/// write that fails partway must not leave `status` claiming a size the
/// sensor never reached.
#[test]
fn test_framesize_status_not_recorded_when_a_write_fails() {
    macro_rules! case {
        ($driver:ty, $size:expr) => {{
            // Find a failure point inside the register sequence.
            let mut probe: MockSccbBus<4096> = MockSccbBus::new();
            let mut sensor = <$driver>::new();
            if sensor.set_framesize(&mut probe, $size).is_ok() {
                let ops = probe.transaction_count();
                assert!(ops > 1, "{} wrote too little to probe", stringify!($driver));

                let mut bus: MockSccbBus<4096> = MockSccbBus::new();
                let mut sensor = <$driver>::new();
                let before = sensor.status().framesize;
                bus.fail_at(ops / 2);

                assert!(
                    sensor.set_framesize(&mut bus, $size).is_err(),
                    "{} should surface the injected bus failure",
                    stringify!($driver),
                );
                assert_eq!(
                    sensor.status().framesize,
                    before,
                    "{} recorded {:?} although the writes failed",
                    stringify!($driver),
                    $size,
                );
            }
        }};
    }

    case!(Nt99141, FrameSize::Qvga);
    case!(Bf3005, FrameSize::Vga);
    case!(Bf20a6, FrameSize::Vga);
    case!(Gc032a, FrameSize::Qvga);
    case!(Gc0308, FrameSize::Qvga);
    case!(Hm0360, FrameSize::Qvga);
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
    // The multipliers the original lookup attested, with the codes the
    // formula derives. 252, 264 and 276 are deliberately absent: the old
    // table skipped them, so no encoding for them is attested.
    for (mul, want) in [
        (204i32, 0x10u8),
        (216, 0x11),
        (228, 0x12),
        (240, 0x13),
        (288, 0x17),
        (300, 0x18),
        (312, 0x19),
        (324, 0x1A),
        (336, 0x1B),
        (348, 0x1C),
        (360, 0x1D),
    ] {
        let mut bus: MockSccbBus<256> = MockSccbBus::new();
        let mut sensor = Hm1055::new();

        assert_eq!(
            sensor.set_pll(&mut bus, 0, mul, 1, 0, 0, 0, 1, 16),
            Ok(()),
            "mul {mul} must be accepted"
        );

        let code = bus.reg16(CameraSccbAddr::HM1055, CKCFG2) & 0x1F;
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

// ---------------------------------------------------------------------------
// SENS-25: controls that silently substituted a different setting
// ---------------------------------------------------------------------------

/// GC0308's special-effect register field is two bits wide, so it can express
/// normal, negative and grayscale and nothing else. The driver accepted the
/// full 0..=6 range anyway and mapped 2 through 6 all onto grayscale, so
/// asking for a red tint or sepia produced a grayscale image, returned `Ok`
/// and recorded the effect that was asked for rather than the one applied.
#[test]
fn test_sens25_gc0308_refuses_effects_it_cannot_express() {
    for effect in [3i32, 4, 5, 6] {
        let mut bus: MockSccbBus<256> = MockSccbBus::new();
        let mut sensor = Gc0308::new();
        let before = sensor.status().special_effect;

        assert_eq!(
            sensor.set_special_effect(&mut bus, effect),
            Err(CameraError::NotSupported),
            "effect {effect} has no encoding in a two-bit field"
        );
        assert_eq!(
            sensor.status().special_effect,
            before,
            "a refused effect must not be recorded"
        );
    }
}

#[test]
fn test_sens25_gc0308_still_applies_the_effects_it_has() {
    for effect in [0i32, 1, 2] {
        let mut bus: MockSccbBus<256> = MockSccbBus::new();
        let mut sensor = Gc0308::new();
        assert_eq!(sensor.set_special_effect(&mut bus, effect), Ok(()));
        assert_eq!(sensor.status().special_effect, effect as u8);
    }
}

/// OV3640 has five saturation steps. `set_saturation` accepted -4..=4 and
/// then clamped to -2..=2, so -4 and -3 both produced -2 while the call
/// reported success.
#[test]
fn test_sens25_ov3640_saturation_refuses_levels_it_cannot_reach() {
    for level in [-4i32, -3, 3, 4] {
        let mut bus: MockSccbBus<256> = MockSccbBus::new();
        let mut sensor = Ov3640::new();

        assert_eq!(
            sensor.set_saturation(&mut bus, level),
            Err(CameraError::InvalidArg),
            "saturation {level} is outside the five-entry table"
        );
    }
}

#[test]
fn test_sens25_ov3640_saturation_applies_its_five_steps() {
    for level in [-2i32, -1, 0, 1, 2] {
        let mut bus: MockSccbBus<256> = MockSccbBus::new();
        let mut sensor = Ov3640::new();
        assert_eq!(sensor.set_saturation(&mut bus, level), Ok(()));
        assert_eq!(sensor.status().saturation, level as i8);
    }
}

// ---------------------------------------------------------------------------
// SENS-15: why the default-table rewrite cannot simply be dropped
// ---------------------------------------------------------------------------

/// SENS-15 asks these drivers to stop re-sending the whole default table on
/// every `set_framesize` and write "only the framesize-specific delta".
/// These tests record why that is not safe as stated: the framesize tables do
/// not cover the same registers as each other, so the default rewrite is what
/// currently undoes the previous size's settings.
///
/// They assert structure over the public tables, not sensor behaviour. If
/// someone reorganises the tables to make a delta write possible, these fail
/// and the assumption gets rechecked rather than silently lost.
fn regs_of(table: &[Reg16]) -> std::collections::BTreeSet<u16> {
    table.iter().map(|e| e.reg).collect()
}

#[test]
fn test_sens15_hm0360_default_table_is_load_bearing() {
    use esp32_cam_sensors::hm0360::{
        SENSOR_DEFAULT_REGS, SENSOR_FRAMESIZE_QVGA, SENSOR_FRAMESIZE_VGA,
    };
    let default = regs_of(SENSOR_DEFAULT_REGS);
    let qvga = regs_of(SENSOR_FRAMESIZE_QVGA);
    let vga = regs_of(SENSOR_FRAMESIZE_VGA);

    // The two tables are disjoint: switching between them leaves the other's
    // registers set unless something resets them.
    assert!(
        qvga.is_disjoint(&vga),
        "HM0360 QVGA and VGA tables now overlap; recheck the SENS-15 analysis"
    );

    // Going QVGA -> VGA, the QVGA registers are undone only because the
    // default table rewrites them. Dropping that rewrite strands all 57.
    let stranded: Vec<_> = qvga
        .difference(&vga)
        .filter(|r| default.contains(r))
        .collect();
    assert_eq!(
        stranded.len(),
        57,
        "the default rewrite is what resets the QVGA-only registers"
    );
}

#[test]
fn test_sens15_hm1055_default_table_is_load_bearing() {
    use esp32_cam_sensors::hm1055::{
        SENSOR_DEFAULT_REGS, SENSOR_FRAMESIZE_HD, SENSOR_FRAMESIZE_VGA,
    };
    let default = regs_of(SENSOR_DEFAULT_REGS);
    let hd = regs_of(SENSOR_FRAMESIZE_HD);
    let vga = regs_of(SENSOR_FRAMESIZE_VGA);

    // HD writes four fewer registers than VGA does, so VGA -> HD depends on
    // the default rewrite to restore them.
    let only_in_vga: Vec<_> = vga.difference(&hd).copied().collect();
    assert_eq!(only_in_vga.len(), 4);
    for reg in &only_in_vga {
        assert!(
            default.contains(reg),
            "{reg:#06x} is written by VGA, not by HD, and not restored by the \
             default table: VGA -> HD would strand it"
        );
    }
}

/// NT99141 is worse: its framesize tables barely intersect the default table,
/// so the rewrite does not restore framesize state at all. Nine registers
/// that other sizes write are absent from the HD table and absent from the
/// default table, so VGA -> HD leaves them at their VGA values today. This is
/// a live defect, not something SENS-15 would introduce — recorded here
/// because the SENS-15 analysis surfaced it.
#[test]
fn test_sens15_nt99141_default_rewrite_does_not_restore_framesize_state() {
    use esp32_cam_sensors::nt99141::{
        SENSOR_DEFAULT_REGS, SENSOR_FRAMESIZE_HD, SENSOR_FRAMESIZE_QVGA, SENSOR_FRAMESIZE_VGA,
        SENSOR_FRAMESIZE_VGA_XYSKIP,
    };
    let default = regs_of(SENSOR_DEFAULT_REGS);
    let union: std::collections::BTreeSet<u16> = regs_of(SENSOR_FRAMESIZE_HD)
        .union(&regs_of(SENSOR_FRAMESIZE_VGA))
        .copied()
        .collect::<std::collections::BTreeSet<_>>()
        .union(&regs_of(SENSOR_FRAMESIZE_QVGA))
        .copied()
        .collect::<std::collections::BTreeSet<_>>()
        .union(&regs_of(SENSOR_FRAMESIZE_VGA_XYSKIP))
        .copied()
        .collect();

    let covered = union.intersection(&default).count();
    assert_eq!(
        (union.len(), covered),
        (64, 4),
        "only 4 of the 64 framesize registers are in the default table, so \
         re-sending it does not undo the previous size"
    );

    let hd = regs_of(SENSOR_FRAMESIZE_HD);
    let stale: Vec<_> = union
        .difference(&hd)
        .filter(|r| !default.contains(r))
        .collect();
    assert_eq!(
        stale.len(),
        9,
        "registers another size writes that HD neither writes nor has \
         restored: these go stale on a switch to HD"
    );
}
