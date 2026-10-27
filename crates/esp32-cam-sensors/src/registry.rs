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
    // 1. Ov7725 (0x21)
    if let Ok(id) = ov7725::Ov7725::detect(bus, CameraSccbAddr::OV7725) {
        return Ok((CameraModel::Ov7725, CameraSccbAddr::OV7725, id));
    }
    // 2. Ov2640 (0x30)
    if let Ok(id) = ov2640::Ov2640::detect(bus, CameraSccbAddr::OV2640) {
        return Ok((CameraModel::Ov2640, CameraSccbAddr::OV2640, id));
    }
    // 3. Ov3660 (0x3C)
    if let Ok(id) = ov3660::Ov3660::detect(bus, CameraSccbAddr::OV3660) {
        return Ok((CameraModel::Ov3660, CameraSccbAddr::OV3660, id));
    }
    // 4. Ov3640 (0x3C)
    if let Ok(id) = ov3640::Ov3640::detect(bus, CameraSccbAddr::OV3640) {
        return Ok((CameraModel::Ov3640, CameraSccbAddr::OV3640, id));
    }
    // 5. Ov5640 (0x3C)
    if let Ok(id) = ov5640::Ov5640::detect(bus, CameraSccbAddr::OV5640) {
        return Ok((CameraModel::Ov5640, CameraSccbAddr::OV5640, id));
    }
    // 6. Ov7670 (0x21)
    if let Ok(id) = ov7670::Ov7670::detect(bus, CameraSccbAddr::OV7670) {
        return Ok((CameraModel::Ov7670, CameraSccbAddr::OV7670, id));
    }
    // 7. Nt99141 (0x2A)
    if let Ok(id) = nt99141::Nt99141::detect(bus, CameraSccbAddr::NT99141) {
        return Ok((CameraModel::Nt99141, CameraSccbAddr::NT99141, id));
    }
    // 8. Gc2145 (0x3C)
    if let Ok(id) = gc2145::Gc2145::detect(bus, CameraSccbAddr::GC2145) {
        return Ok((CameraModel::Gc2145, CameraSccbAddr::GC2145, id));
    }
    // 9. Gc032a (0x21)
    if let Ok(id) = gc032a::Gc032a::detect(bus, CameraSccbAddr::GC032A) {
        return Ok((CameraModel::Gc032a, CameraSccbAddr::GC032A, id));
    }
    // 10. Gc0308 (0x21)
    if let Ok(id) = gc0308::Gc0308::detect(bus, CameraSccbAddr::GC0308) {
        return Ok((CameraModel::Gc0308, CameraSccbAddr::GC0308, id));
    }
    // 11. Bf3005 (0x6E)
    if let Ok(id) = bf3005::Bf3005::detect(bus, CameraSccbAddr::BF3005) {
        return Ok((CameraModel::Bf3005, CameraSccbAddr::BF3005, id));
    }
    // 12. Bf20a6 (0x6E)
    if let Ok(id) = bf20a6::Bf20a6::detect(bus, CameraSccbAddr::BF20A6) {
        return Ok((CameraModel::Bf20a6, CameraSccbAddr::BF20A6, id));
    }
    // 13. Sc101Iot (0x68)
    if let Ok(id) = sc101iot::Sc101Iot::detect(bus, CameraSccbAddr::SC101IOT) {
        return Ok((CameraModel::Sc101Iot, CameraSccbAddr::SC101IOT, id));
    }
    // 14. Sc030Iot (0x68)
    if let Ok(id) = sc030iot::Sc030Iot::detect(bus, CameraSccbAddr::SC030IOT) {
        return Ok((CameraModel::Sc030Iot, CameraSccbAddr::SC030IOT, id));
    }
    // 15. Sc031Gs (0x30)
    if let Ok(id) = sc031gs::Sc031Gs::detect(bus, CameraSccbAddr::SC031GS) {
        return Ok((CameraModel::Sc031Gs, CameraSccbAddr::SC031GS, id));
    }
    // 16. MegaCcm (0x1F)
    if let Ok(id) = mega_ccm::MegaCcm::detect(bus, CameraSccbAddr::MEGA_CCM) {
        return Ok((CameraModel::MegaCcm, CameraSccbAddr::MEGA_CCM, id));
    }
    // 17. Hm1055 (0x24)
    if let Ok(id) = hm1055::Hm1055::detect(bus, CameraSccbAddr::HM1055) {
        return Ok((CameraModel::Hm1055, CameraSccbAddr::HM1055, id));
    }
    // 18. Hm0360 (0x12)
    if let Ok(id) = hm0360::Hm0360::detect(bus, CameraSccbAddr::HM0360) {
        return Ok((CameraModel::Hm0360, CameraSccbAddr::HM0360, id));
    }

    Err(CameraError::NotDetected)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::traits::{set_reg_bits16, set_reg_bits8};
    use esp32_cam_sccb::MockSccbBus;

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
