//! OmniVision camera sensor driver family re-exports and comprehensive unit test suite.
//!
//! Exposes all 6 OmniVision drivers (`OV2640`, `OV3660`, `OV3640`, `OV5640` + `OV5640-AF`,
//! `OV7725`, and `OV7670`).

pub use crate::ov2640::{self, Ov2640, Ov2640Bank};
pub use crate::ov3640::{self, Ov3640};
pub use crate::ov3660::{self, Ov3660};
pub use crate::ov5640::{self, Ov5640, Ov5640Af, Ov5640AfStatus};
pub use crate::ov7670::{self, Ov7670};
pub use crate::ov7725::{self, Ov7725};

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        CameraError, CameraModel, CameraPid, CameraSccbAddr, CameraSensor, FrameSize, GainCeiling,
        MockSccbBus, PixelFormat,
    };

    #[test]
    fn test_ov2640_detect_reset_and_ov105_hd_window_invariant() {
        let mut bus: MockSccbBus<512> = MockSccbBus::new();
        bus.preset_reg8(CameraSccbAddr::OV2640, ov2640::REG_PID, 0x26);
        bus.preset_reg8(CameraSccbAddr::OV2640, ov2640::REG_VER, 0x42);
        bus.preset_reg8(CameraSccbAddr::OV2640, ov2640::REG_MIDH, 0x7F);
        bus.preset_reg8(CameraSccbAddr::OV2640, ov2640::REG_MIDL, 0xA2);

        let id = Ov2640::detect(&mut bus, CameraSccbAddr::OV2640);
        assert!(id.is_ok());
        assert_eq!(
            Ov2640::detect(&mut bus, 0x99),
            Err(CameraError::NotDetected)
        );

        let mut sensor = Ov2640::new();
        assert_eq!(sensor.model(), CameraModel::Ov2640);
        assert_eq!(sensor.id().pid, CameraPid::OV2640.as_u16());

        assert_eq!(sensor.reset(&mut bus), Ok(()));
        assert_eq!(sensor.init_status(&mut bus), Ok(()));
        assert_eq!(sensor.set_pixformat(&mut bus, PixelFormat::Jpeg), Ok(()));

        // Reset again to load UXGA defaults (`HSIZE8 = 0xC8`, `VSIZE8 = 0x96`, `SIZEL = 0x00`)
        // then configure `FrameSize::Hd` (1280x720, 16:9 crop with `offset_y = 150`, `max_y = 900`).
        assert_eq!(sensor.reset(&mut bus), Ok(()));
        assert_eq!(sensor.set_framesize(&mut bus, FrameSize::Hd), Ok(()));
        assert_eq!(sensor.status().framesize, FrameSize::Hd);

        // Critical OV-105 Invariant verification:
        // `HSIZE8` (0xC0), `VSIZE8` (0xC1), and `SIZEL` (0x8C) MUST preserve UXGA (1600x1200)
        // and MUST NOT be overwritten with cropped `max_x`/`max_y` (1600x900 = 0xC8, 0x70, 0x04).
        assert_eq!(bus.get_reg8(CameraSccbAddr::OV2640, ov2640::HSIZE8), 0xC8);
        assert_eq!(bus.get_reg8(CameraSccbAddr::OV2640, ov2640::VSIZE8), 0x96);
        assert_eq!(bus.get_reg8(CameraSccbAddr::OV2640, ov2640::SIZEL), 0x00);

        // Verify 1280x720 output scaling and 1600x900 sensor windowing registers:
        assert_eq!(
            bus.get_reg8(CameraSccbAddr::OV2640, ov2640::ZMOW),
            (1280 >> 2) as u8
        );
        assert_eq!(
            bus.get_reg8(CameraSccbAddr::OV2640, ov2640::ZMOH),
            (720 >> 2) as u8
        );
        assert_eq!(bus.get_reg8(CameraSccbAddr::OV2640, ov2640::REG32), 0x36);

        // Verify controls and bounds
        assert_eq!(sensor.set_quality(&mut bus, 12), Ok(()));
        assert_eq!(sensor.status().quality, 12);
        assert_eq!(
            sensor.set_quality(&mut bus, 64),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(sensor.set_contrast(&mut bus, 2), Ok(()));
        assert_eq!(sensor.set_brightness(&mut bus, -1), Ok(()));
        assert_eq!(sensor.set_saturation(&mut bus, 1), Ok(()));
        assert_eq!(sensor.set_special_effect(&mut bus, 2), Ok(()));
        assert_eq!(sensor.set_wb_mode(&mut bus, 1), Ok(()));
        assert_eq!(sensor.set_ae_level(&mut bus, 2), Ok(()));
        assert_eq!(
            sensor.set_gainceiling(&mut bus, GainCeiling::Gain16x),
            Ok(())
        );
        assert_eq!(sensor.set_hmirror(&mut bus, true), Ok(()));
        assert_eq!(sensor.set_vflip(&mut bus, true), Ok(()));
        assert_eq!(sensor.set_colorbar(&mut bus, true), Ok(()));
        assert_eq!(sensor.set_agc_gain(&mut bus, 15), Ok(()));
        assert_eq!(sensor.set_aec_value(&mut bus, 600), Ok(()));

        // Verify bus error propagation
        bus.set_fail(true);
        assert_eq!(sensor.set_quality(&mut bus, 10), Err(CameraError::SccbError));
    }

    #[test]
    fn test_ov3660_detect_reset_window_and_16bit_bounds() {
        let mut bus: MockSccbBus<512> = MockSccbBus::new();
        bus.preset_reg16(CameraSccbAddr::OV3660, 0x300A, 0x36);
        bus.preset_reg16(CameraSccbAddr::OV3660, 0x300B, 0x60);

        assert_eq!(
            Ov3660::detect(&mut bus, CameraSccbAddr::OV3660),
            Ok(crate::SensorId {
                midh: 0,
                midl: 0,
                pid: CameraPid::OV3660.as_u16(),
                ver: 0,
            })
        );

        let mut sensor = Ov3660::new();
        assert_eq!(sensor.model(), CameraModel::Ov3660);
        assert_eq!(sensor.reset(&mut bus), Ok(()));
        assert_eq!(sensor.init_status(&mut bus), Ok(()));

        assert_eq!(sensor.set_pixformat(&mut bus, PixelFormat::Jpeg), Ok(()));
        assert_eq!(sensor.set_framesize(&mut bus, FrameSize::Qxga), Ok(()));
        assert!(!sensor.status().binning);
        assert!(!sensor.status().scale);

        assert_eq!(sensor.set_framesize(&mut bus, FrameSize::Svga), Ok(()));
        assert!(sensor.status().binning);
        assert!(sensor.status().scale);

        assert_eq!(
            sensor.set_framesize(&mut bus, FrameSize::Qhd),
            Err(CameraError::InvalidArg)
        );

        // Enforce 16-bit register upper bounds on `set_aec_value` and `set_agc_gain`
        assert_eq!(
            sensor.set_agc_gain(&mut bus, -1),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(
            sensor.set_agc_gain(&mut bus, 65),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(sensor.set_agc_gain(&mut bus, 64), Ok(()));
        assert_eq!(sensor.status().agc_gain, 64);

        assert_eq!(
            sensor.set_aec_value(&mut bus, -1),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(
            sensor.set_aec_value(&mut bus, 0x1_0000),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(sensor.set_aec_value(&mut bus, 0x0400), Ok(()));
        // On SVGA (`total_y = 784`), `max_level = total_y - 1 = 783`
        assert_eq!(sensor.status().aec_value, 783);

        // Verify ISP controls and error propagation
        assert_eq!(sensor.set_contrast(&mut bus, 1), Ok(()));
        assert_eq!(sensor.set_brightness(&mut bus, -2), Ok(()));
        assert_eq!(sensor.set_saturation(&mut bus, 3), Ok(()));
        assert_eq!(sensor.set_sharpness(&mut bus, 2), Ok(()));
        assert_eq!(sensor.set_denoise(&mut bus, 4), Ok(()));
        assert_eq!(sensor.set_wb_mode(&mut bus, 2), Ok(()));
        assert_eq!(sensor.set_special_effect(&mut bus, 3), Ok(()));

        bus.set_fail(true);
        assert_eq!(sensor.set_aec_value(&mut bus, 100), Err(CameraError::SccbError));
    }

    #[test]
    fn test_ov3640_family_pid_regulator_bypass_and_mirror_flip() {
        let mut bus: MockSccbBus<512> = MockSccbBus::new();
        // OV3640 family mask `(pid & 0xFFF0) == 0x3640` accepts 0x364C as well as 0x3640
        bus.preset_reg16(CameraSccbAddr::OV3640, 0x300A, 0x36);
        bus.preset_reg16(CameraSccbAddr::OV3640, 0x300B, 0x4C);

        let id = Ov3640::detect(&mut bus, CameraSccbAddr::OV3640);
        assert_eq!(
            id,
            Ok(crate::SensorId {
                midh: 0,
                midl: 0,
                pid: 0x3640,
                ver: 0x0C,
            })
        );

        let mut sensor = Ov3640::new();
        assert_eq!(sensor.model(), CameraModel::Ov3640);
        assert_eq!(sensor.reset(&mut bus), Ok(()));
        // Regulator bypass register PWCOM1 (0x30A9) must have bit 3 (0x08) set and bank switch 0x00FF restored to 0x00
        assert_eq!(bus.get_reg16(CameraSccbAddr::OV3640, ov3640::PWCOM1), 0x08);
        assert_eq!(bus.get_reg16(CameraSccbAddr::OV3640, 0x00FF), 0x00);

        assert_eq!(sensor.set_pixformat(&mut bus, PixelFormat::Jpeg), Ok(()));
        assert_eq!(sensor.set_framesize(&mut bus, FrameSize::Uxga), Ok(()));
        assert_eq!(sensor.set_framesize(&mut bus, FrameSize::Svga), Ok(()));

        // Verify mirror and flip update 0x307C, 0x3090, and 0x3023
        assert_eq!(sensor.set_hmirror(&mut bus, true), Ok(()));
        assert_eq!(
            bus.get_reg16(CameraSccbAddr::OV3640, 0x307C) & 0x02,
            0x02
        );
        assert_eq!(
            bus.get_reg16(CameraSccbAddr::OV3640, 0x3090) & 0x08,
            0x08
        );

        assert_eq!(sensor.set_vflip(&mut bus, true), Ok(()));
        assert_eq!(
            bus.get_reg16(CameraSccbAddr::OV3640, 0x307C) & 0x01,
            0x01
        );
        assert_eq!(bus.get_reg16(CameraSccbAddr::OV3640, 0x3023), 0x09);

        assert_eq!(sensor.set_contrast(&mut bus, 2), Ok(()));
        assert_eq!(sensor.set_brightness(&mut bus, 1), Ok(()));
        assert_eq!(sensor.set_saturation(&mut bus, -2), Ok(()));
        assert_eq!(sensor.set_sharpness(&mut bus, 3), Ok(()));
        assert_eq!(sensor.set_denoise(&mut bus, 5), Ok(()));
        assert_eq!(sensor.set_agc_gain(&mut bus, 32), Ok(()));
        assert_eq!(sensor.set_aec_value(&mut bus, 500), Ok(()));
    }

    #[test]
    fn test_ov5640_and_ov5640_af_lifecycle_and_bounds() {
        let mut bus: MockSccbBus<512> = MockSccbBus::new();
        bus.preset_reg16(CameraSccbAddr::OV5640, 0x300A, 0x56);
        bus.preset_reg16(CameraSccbAddr::OV5640, 0x300B, 0x40);

        assert_eq!(
            Ov5640::detect(&mut bus, CameraSccbAddr::OV5640),
            Ok(crate::SensorId {
                midh: 0,
                midl: 0,
                pid: CameraPid::OV5640.as_u16(),
                ver: 0,
            })
        );

        let mut sensor = Ov5640::new();
        assert_eq!(sensor.model(), CameraModel::Ov5640);
        assert!(sensor.af_is_supported());
        assert_eq!(sensor.reset(&mut bus), Ok(()));
        assert_eq!(sensor.init_status(&mut bus), Ok(()));

        assert_eq!(sensor.set_pixformat(&mut bus, PixelFormat::Jpeg), Ok(()));
        assert_eq!(sensor.set_framesize(&mut bus, FrameSize::Qsxga), Ok(()));
        assert!(!sensor.status().binning);
        assert!(!sensor.status().scale);

        assert_eq!(sensor.set_framesize(&mut bus, FrameSize::Hd), Ok(()));
        assert!(sensor.status().binning);
        assert!(!sensor.status().scale);

        // Enforce 16-bit register upper bounds on `set_aec_value` and `set_agc_gain`
        assert_eq!(
            sensor.set_agc_gain(&mut bus, 65),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(
            sensor.set_aec_value(&mut bus, 0x1_0000),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(sensor.set_agc_gain(&mut bus, 32), Ok(()));
        assert_eq!(sensor.set_aec_value(&mut bus, 800), Ok(()));

        // Verify Ov5640Af status decoding, mode setting, and timeout handling
        assert_eq!(ov5640::OV5640_AF_FIRMWARE.len(), 4077);
        bus.preset_reg16(
            CameraSccbAddr::OV5640,
            ov5640::OV5640_CMD_FW_STATUS,
            ov5640::OV5640_FW_STATUS_FOCUSED,
        );
        assert_eq!(
            sensor.af_get_status(&mut bus),
            Ok(Ov5640AfStatus {
                raw: ov5640::OV5640_FW_STATUS_FOCUSED,
                focused: true,
                busy: false,
            })
        );

        bus.preset_reg16(CameraSccbAddr::OV5640, ov5640::OV5640_CMD_FW_STATUS, 0x00);
        assert_eq!(
            sensor.af_get_status(&mut bus),
            Ok(Ov5640AfStatus {
                raw: 0x00,
                focused: false,
                busy: true,
            })
        );

        assert_eq!(
            sensor.af_set_mode(&mut bus, ov5640::OV5640_AF_MODE_MANUAL),
            Ok(())
        );
        assert_eq!(
            bus.get_reg16(CameraSccbAddr::OV5640, ov5640::OV5640_CMD_MAIN),
            0x00
        );
        assert_eq!(
            sensor.af_set_mode(&mut bus, 99),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(
            sensor.af_set_manual_position(&mut bus, 100),
            Err(CameraError::NotSupported)
        );

        // When MCU does not transition 0x3029 from 0x7F to 0x70, `af_init` returns `CameraError::Timeout`
        // after writing the 4077-byte firmware to 0x8000..=0x8FEC and setting 0x3029 = 0x7F.
        assert_eq!(sensor.af_init(&mut bus, 10), Err(CameraError::Timeout));
        let fw_check_idx = ov5640::OV5640_AF_FIRMWARE.len() - 2;
        assert_eq!(
            bus.get_reg16(
                CameraSccbAddr::OV5640,
                ov5640::OV5640_AF_FW_BASE_ADDR + (fw_check_idx as u16)
            ),
            ov5640::OV5640_AF_FIRMWARE[fw_check_idx]
        );
        assert_eq!(
            bus.get_reg16(CameraSccbAddr::OV5640, ov5640::OV5640_CMD_FW_STATUS),
            0x7F
        );

        // When 0x3029 is `OV5640_FW_STATUS_IDLE` (0x70), `wait_fw_idle` succeeds immediately
        bus.preset_reg16(
            CameraSccbAddr::OV5640,
            ov5640::OV5640_CMD_FW_STATUS,
            ov5640::OV5640_FW_STATUS_IDLE,
        );
        assert_eq!(sensor.af().wait_fw_idle(&mut bus, 10), Ok(()));
    }

    #[test]
    fn test_ov7725_detect_reset_framesize_and_controls() {
        let mut bus: MockSccbBus<512> = MockSccbBus::new();
        bus.preset_reg8(CameraSccbAddr::OV7725, ov7725::REG_PID, 0x77);
        bus.preset_reg8(CameraSccbAddr::OV7725, ov7725::REG_VER, 0x21);
        bus.preset_reg8(CameraSccbAddr::OV7725, ov7725::REG_MIDH, 0x7F);
        bus.preset_reg8(CameraSccbAddr::OV7725, ov7725::REG_MIDL, 0xA2);

        assert!(Ov7725::detect(&mut bus, CameraSccbAddr::OV7725).is_ok());

        let mut sensor = Ov7725::new();
        assert_eq!(sensor.model(), CameraModel::Ov7725);
        assert_eq!(sensor.reset(&mut bus), Ok(()));
        assert_eq!(sensor.init_status(&mut bus), Ok(()));

        assert_eq!(sensor.set_pixformat(&mut bus, PixelFormat::Rgb565), Ok(()));
        assert_eq!(
            sensor.set_pixformat(&mut bus, PixelFormat::Jpeg),
            Err(CameraError::NotSupported)
        );

        assert_eq!(sensor.set_framesize(&mut bus, FrameSize::Qvga), Ok(()));
        assert_eq!(
            bus.get_reg8(CameraSccbAddr::OV7725, ov7725::HOUTSIZE),
            (320 >> 2) as u8
        );
        assert_eq!(
            bus.get_reg8(CameraSccbAddr::OV7725, ov7725::VOUTSIZE),
            (240 >> 1) as u8
        );

        assert_eq!(sensor.set_framesize(&mut bus, FrameSize::Vga), Ok(()));
        assert_eq!(bus.get_reg8(CameraSccbAddr::OV7725, ov7725::DSPAUTO), 0xF3);
        assert_eq!(
            sensor.set_framesize(&mut bus, FrameSize::Svga),
            Err(CameraError::InvalidArg)
        );

        assert_eq!(sensor.set_dcw(&mut bus, true), Ok(()));
        assert_eq!(
            bus.get_reg8(CameraSccbAddr::OV7725, ov7725::DSP_CTRL2) & 0x04,
            0x00
        );
        assert_eq!(sensor.set_colorbar(&mut bus, true), Ok(()));
        assert_eq!(sensor.set_hmirror(&mut bus, true), Ok(()));
        assert_eq!(sensor.set_vflip(&mut bus, true), Ok(()));
        assert_eq!(sensor.set_aec_value(&mut bus, 0x1234), Ok(()));
        assert_eq!(bus.get_reg8(CameraSccbAddr::OV7725, ov7725::AEC), 0x34);
        assert_eq!(bus.get_reg8(CameraSccbAddr::OV7725, ov7725::AECH), 0x12);
    }

    #[test]
    fn test_ov7670_detect_reset_clkrc_persistence_and_framesize() {
        let mut bus: MockSccbBus<512> = MockSccbBus::new();
        bus.preset_reg8(CameraSccbAddr::OV7670, ov7670::REG_PID, 0x76);
        bus.preset_reg8(CameraSccbAddr::OV7670, ov7670::REG_VER, 0x73);
        bus.preset_reg8(CameraSccbAddr::OV7670, ov7670::REG_MIDH, 0x7F);
        bus.preset_reg8(CameraSccbAddr::OV7670, ov7670::REG_MIDL, 0xA2);

        assert!(Ov7670::detect(&mut bus, CameraSccbAddr::OV7670).is_ok());

        let mut sensor = Ov7670::new();
        assert_eq!(sensor.model(), CameraModel::Ov7670);
        assert_eq!(sensor.reset(&mut bus), Ok(()));
        assert_eq!(sensor.init_status(&mut bus), Ok(()));

        // Preset CLKRC to 0x07 before set_framesize, then verify set_pixformat(Rgb565) restores it
        bus.preset_reg8(CameraSccbAddr::OV7670, ov7670::CLKRC, 0x07);
        assert_eq!(sensor.set_framesize(&mut bus, FrameSize::Qvga), Ok(()));
        assert_eq!(sensor.clkrc(), 0x07);

        bus.preset_reg8(CameraSccbAddr::OV7670, ov7670::CLKRC, 0x00);
        assert_eq!(sensor.set_pixformat(&mut bus, PixelFormat::Rgb565), Ok(()));
        assert_eq!(bus.get_reg8(CameraSccbAddr::OV7670, ov7670::CLKRC), 0x07);

        assert_eq!(sensor.set_framesize(&mut bus, FrameSize::Vga), Ok(()));
        assert_eq!(sensor.set_framesize(&mut bus, FrameSize::Qqvga), Ok(()));
        assert_eq!(
            sensor.set_framesize(&mut bus, FrameSize::Cif),
            Err(CameraError::InvalidArg)
        );

        assert_eq!(sensor.set_colorbar(&mut bus, true), Ok(()));
        assert_eq!(
            bus.get_reg8(CameraSccbAddr::OV7670, ov7670::SCALING_YSC) & 0x80,
            0x80
        );
        assert_eq!(sensor.set_hmirror(&mut bus, true), Ok(()));
        assert_eq!(sensor.set_vflip(&mut bus, true), Ok(()));
        assert_eq!(sensor.set_whitebal(&mut bus, true), Ok(()));
        assert_eq!(sensor.set_gain_ctrl(&mut bus, true), Ok(()));
        assert_eq!(sensor.set_exposure_ctrl(&mut bus, true), Ok(()));
    }
}
