//! Automatic camera sensor detection registry.

use esp32_cam_sccb::SccbBus;
use esp32_cam_types::{CameraError, CameraModel, CameraSccbAddr, SensorId};

use crate::{
    bf20a6, bf3005, gc0308, gc032a, gc2145, hm0360, hm1055, mega_ccm, nt99141, ov2640, ov3640,
    ov3660, ov5640, ov7670, ov7725, sc030iot, sc031gs, sc101iot,
};

/// Automatically detects an attached camera sensor by querying each of
/// the 18 supported sensor models at their canonical SCCB addresses in order.
pub fn detect_sensor<B: SccbBus>(bus: &mut B) -> Result<(CameraModel, u8, SensorId), CameraError> {
    macro_rules! probe {
        ($res:expr, $model:expr, $addr:expr) => {
            match $res {
                Ok(id) => return Ok(($model, $addr, id)),
                Err(CameraError::I2cError) => return Err(CameraError::I2cError),
                Err(_) => {}
            }
        };
    }

    // 1. Ov7725 (0x21)
    probe!(
        ov7725::Ov7725::detect(bus, CameraSccbAddr::OV7725),
        CameraModel::Ov7725,
        CameraSccbAddr::OV7725
    );
    // 2. 16-bit SmartSens sensors at 0x30 / 0x68 probed before Ov2640
    // (which writes bank register 0xFF)
    probe!(
        sc031gs::Sc031Gs::detect(bus, CameraSccbAddr::SC031GS),
        CameraModel::Sc031Gs,
        CameraSccbAddr::SC031GS
    );
    probe!(
        sc101iot::Sc101Iot::detect(bus, CameraSccbAddr::SC101IOT),
        CameraModel::Sc101Iot,
        CameraSccbAddr::SC101IOT
    );
    probe!(
        sc030iot::Sc030Iot::detect(bus, CameraSccbAddr::SC030IOT),
        CameraModel::Sc030Iot,
        CameraSccbAddr::SC030IOT
    );
    // 3. Ov2640 (0x30)
    probe!(
        ov2640::Ov2640::detect(bus, CameraSccbAddr::OV2640),
        CameraModel::Ov2640,
        CameraSccbAddr::OV2640
    );
    // 4. Ov3660 (0x3C)
    probe!(
        ov3660::Ov3660::detect(bus, CameraSccbAddr::OV3660),
        CameraModel::Ov3660,
        CameraSccbAddr::OV3660
    );
    // 5. Ov3640 (0x3C)
    probe!(
        ov3640::Ov3640::detect(bus, CameraSccbAddr::OV3640),
        CameraModel::Ov3640,
        CameraSccbAddr::OV3640
    );
    // 6. Ov5640 (0x3C)
    probe!(
        ov5640::Ov5640::detect(bus, CameraSccbAddr::OV5640),
        CameraModel::Ov5640,
        CameraSccbAddr::OV5640
    );
    // 7. Ov7670 (0x21)
    probe!(
        ov7670::Ov7670::detect(bus, CameraSccbAddr::OV7670),
        CameraModel::Ov7670,
        CameraSccbAddr::OV7670
    );
    // 8. Nt99141 (0x2A)
    probe!(
        nt99141::Nt99141::detect(bus, CameraSccbAddr::NT99141),
        CameraModel::Nt99141,
        CameraSccbAddr::NT99141
    );
    // 9. Gc2145 (0x3C)
    probe!(
        gc2145::Gc2145::detect(bus, CameraSccbAddr::GC2145),
        CameraModel::Gc2145,
        CameraSccbAddr::GC2145
    );
    // 10. Gc032a (0x21)
    probe!(
        gc032a::Gc032a::detect(bus, CameraSccbAddr::GC032A),
        CameraModel::Gc032a,
        CameraSccbAddr::GC032A
    );
    // 11. Gc0308 (0x21)
    probe!(
        gc0308::Gc0308::detect(bus, CameraSccbAddr::GC0308),
        CameraModel::Gc0308,
        CameraSccbAddr::GC0308
    );
    // 12. Bf3005 (0x6E)
    probe!(
        bf3005::Bf3005::detect(bus, CameraSccbAddr::BF3005),
        CameraModel::Bf3005,
        CameraSccbAddr::BF3005
    );
    // 13. Bf20a6 (0x6E)
    probe!(
        bf20a6::Bf20a6::detect(bus, CameraSccbAddr::BF20A6),
        CameraModel::Bf20a6,
        CameraSccbAddr::BF20A6
    );
    // 14. MegaCcm (0x1F)
    probe!(
        mega_ccm::MegaCcm::detect(bus, CameraSccbAddr::MEGA_CCM),
        CameraModel::MegaCcm,
        CameraSccbAddr::MEGA_CCM
    );
    // 15. Hm1055 (0x24)
    probe!(
        hm1055::Hm1055::detect(bus, CameraSccbAddr::HM1055),
        CameraModel::Hm1055,
        CameraSccbAddr::HM1055
    );
    // 16. Hm0360 (0x12)
    probe!(
        hm0360::Hm0360::detect(bus, CameraSccbAddr::HM0360),
        CameraModel::Hm0360,
        CameraSccbAddr::HM0360
    );

    Err(CameraError::NotDetected)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::traits::{set_reg_bits16, set_reg_bits8};
    use esp32_cam_sccb::MockSccbBus;

    struct FaultyI2cBus;

    impl SccbBus for FaultyI2cBus {
        fn read_reg8(&mut self, _addr: u8, _reg: u8) -> Result<u8, CameraError> {
            Err(CameraError::I2cError)
        }
        fn write_reg8(&mut self, _addr: u8, _reg: u8, _val: u8) -> Result<(), CameraError> {
            Err(CameraError::I2cError)
        }
        fn read_reg16(&mut self, _addr: u8, _reg: u16) -> Result<u8, CameraError> {
            Err(CameraError::I2cError)
        }
        fn write_reg16(&mut self, _addr: u8, _reg: u16, _val: u8) -> Result<(), CameraError> {
            Err(CameraError::I2cError)
        }
        fn delay_ms(&mut self, _ms: u32) {}
    }

    #[test]
    fn test_detect_sensor_i2c_error_early_abort() {
        let mut bus = FaultyI2cBus;
        assert_eq!(detect_sensor(&mut bus), Err(CameraError::I2cError));
    }

    #[test]
    fn test_detect_sensor_and_set_reg_bits() {
        let mut bus: MockSccbBus<64> = MockSccbBus::new();

        // Initially no sensor is present
        assert_eq!(detect_sensor(&mut bus), Err(CameraError::NotDetected));

        // Preset OV2640 registers
        bus.preset_reg8(CameraSccbAddr::OV2640, ov2640::REG_PID, 0x26);
        bus.preset_reg8(CameraSccbAddr::OV2640, ov2640::REG_VER, 0x42);
        bus.preset_reg8(CameraSccbAddr::OV2640, ov2640::REG_MIDH, 0x7F);
        bus.preset_reg8(CameraSccbAddr::OV2640, ov2640::REG_MIDL, 0xA2);

        let detected = detect_sensor(&mut bus);
        assert!(detected.is_ok());
        if let Ok((model, addr, id)) = detected {
            assert_eq!(model, CameraModel::Ov2640);
            assert_eq!(addr, CameraSccbAddr::OV2640);
            assert_eq!(id.pid, 0x26);
        }

        // Test set_reg_bits8
        bus.preset_reg8(0x30, 0x10, 0b1010_0000);
        assert_eq!(set_reg_bits8(&mut bus, 0x30, 0x10, 1, 0x07, 0b101), Ok(()));
        assert_eq!(bus.reg8(0x30, 0x10), 0b1010_1010);

        // Test set_reg_bits16
        bus.preset_reg16(0x3C, 0x3000, 0b0000_1111);
        assert_eq!(
            set_reg_bits16(&mut bus, 0x3C, 0x3000, 4, 0x03, 0b10),
            Ok(())
        );
        assert_eq!(bus.reg16(0x3C, 0x3000), 0b0010_1111);
    }
}
