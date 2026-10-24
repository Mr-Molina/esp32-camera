//! Multi-vendor sensor family re-exports (`nt99141`, `gc0308`, `gc2145`, `gc032a`, `bf3005`,
//! `bf20a6`, `sc101iot`, `sc030iot`, `sc031gs`, `mega_ccm`, `hm1055`, `hm0360`).

pub use crate::bf20a6::{Bf20A6, Bf20a6, BF20A6};
pub use crate::bf3005::{Bf3005, BF3005};
pub use crate::gc0308::{Gc0308, GC0308};
pub use crate::gc032a::{Gc032A, Gc032a, GC032A};
pub use crate::gc2145::{Gc2145, GC2145};
pub use crate::hm0360::{Hm0360, HM0360};
pub use crate::hm1055::{Hm1055, HM1055};
pub use crate::mega_ccm::{MegaCcm, MEGA_CCM};
pub use crate::nt99141::{Nt99141, NT99141};
pub use crate::sc030iot::{Sc030Iot, Sc030iot, SC030IOT};
pub use crate::sc031gs::{Sc031Gs, Sc031gs, SC031GS};
pub use crate::sc101iot::{Sc101Iot, Sc101iot, SC101IOT};
pub use crate::{
    bf20a6, bf3005, gc0308, gc032a, gc2145, hm0360, hm1055, mega_ccm, nt99141, sc030iot, sc031gs,
    sc101iot,
};

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        CameraError, CameraModel, CameraPid, CameraSccbAddr, CameraSensor, FrameSize, MockSccbBus,
        PixelFormat,
    };

    #[test]
    fn test_nt99141_lifecycle() {
        let mut bus: MockSccbBus<512> = MockSccbBus::new();
        bus.set_reg16(CameraSccbAddr::NT99141, 0x3000, 0x14);
        bus.set_reg16(CameraSccbAddr::NT99141, 0x3001, 0x10);

        assert_eq!(
            Nt99141::detect(&mut bus, 0x00),
            Err(CameraError::NotDetected)
        );
        let id = Nt99141::detect(&mut bus, CameraSccbAddr::NT99141);
        assert!(id.is_ok());
        assert_eq!(id.map(|i| i.pid), Ok(CameraPid::NT99141.as_u16()));

        let mut sensor = Nt99141::new();
        assert_eq!(sensor.model(), CameraModel::Nt99141);
        assert_eq!(sensor.reset(&mut bus), Ok(()));
        assert_eq!(sensor.set_pixformat(&mut bus, PixelFormat::Jpeg), Ok(()));
        assert_eq!(sensor.set_framesize(&mut bus, FrameSize::Hd), Ok(()));
        assert_eq!(sensor.status().framesize, FrameSize::Hd);
        assert_eq!(sensor.set_hmirror(&mut bus, true), Ok(()));
        assert_eq!(sensor.status().hmirror, 1);
        assert_eq!(sensor.set_vflip(&mut bus, true), Ok(()));
        assert_eq!(sensor.status().vflip, 1);
        assert_eq!(sensor.set_reg(&mut bus, 0x3022, 0xFF, 0x5A), Ok(()));
        assert_eq!(sensor.get_reg(&mut bus, 0x3022, 0xFF), Ok(0x5A));
    }

    #[test]
    fn test_gc0308_lifecycle() {
        let mut bus: MockSccbBus<512> = MockSccbBus::new();
        bus.set_reg8(CameraSccbAddr::GC0308, 0x00, 0x9B);

        assert_eq!(
            Gc0308::detect(&mut bus, 0x00),
            Err(CameraError::NotDetected)
        );
        let id = Gc0308::detect(&mut bus, CameraSccbAddr::GC0308);
        assert!(id.is_ok());
        assert_eq!(id.map(|i| i.pid), Ok(CameraPid::GC0308.as_u16()));

        let mut sensor = Gc0308::new();
        assert_eq!(sensor.model(), CameraModel::Gc0308);
        assert_eq!(sensor.reset(&mut bus), Ok(()));
        assert_eq!(sensor.set_pixformat(&mut bus, PixelFormat::Rgb565), Ok(()));
        assert_eq!(sensor.set_framesize(&mut bus, FrameSize::Qvga), Ok(()));
        assert_eq!(sensor.status().framesize, FrameSize::Qvga);
        assert_eq!(sensor.set_hmirror(&mut bus, true), Ok(()));
        assert_eq!(sensor.status().hmirror, 1);
        assert_eq!(sensor.set_vflip(&mut bus, true), Ok(()));
        assert_eq!(sensor.status().vflip, 1);
        assert_eq!(sensor.set_reg(&mut bus, 0x14, 0xFF, 0x3C), Ok(()));
        assert_eq!(sensor.get_reg(&mut bus, 0x14, 0xFF), Ok(0x3C));
    }

    #[test]
    fn test_gc2145_lifecycle() {
        let mut bus: MockSccbBus<512> = MockSccbBus::new();
        bus.set_reg8(CameraSccbAddr::GC2145, 0xF0, 0x21);
        bus.set_reg8(CameraSccbAddr::GC2145, 0xF1, 0x45);

        assert_eq!(
            Gc2145::detect(&mut bus, 0x00),
            Err(CameraError::NotDetected)
        );
        let id = Gc2145::detect(&mut bus, CameraSccbAddr::GC2145);
        assert!(id.is_ok());
        assert_eq!(id.map(|i| i.pid), Ok(CameraPid::GC2145.as_u16()));

        let mut sensor = Gc2145::new();
        assert_eq!(sensor.model(), CameraModel::Gc2145);
        assert_eq!(sensor.reset(&mut bus), Ok(()));
        assert_eq!(sensor.set_pixformat(&mut bus, PixelFormat::Rgb565), Ok(()));
        assert_eq!(sensor.set_framesize(&mut bus, FrameSize::Svga), Ok(()));
        assert_eq!(sensor.status().framesize, FrameSize::Svga);
        assert_eq!(sensor.set_hmirror(&mut bus, true), Ok(()));
        assert_eq!(sensor.status().hmirror, 1);
        assert_eq!(sensor.set_vflip(&mut bus, true), Ok(()));
        assert_eq!(sensor.status().vflip, 1);
        assert_eq!(sensor.set_reg(&mut bus, 0x17, 0xFF, 0x15), Ok(()));
        assert_eq!(sensor.get_reg(&mut bus, 0x17, 0xFF), Ok(0x15));
    }

    #[test]
    fn test_gc032a_lifecycle() {
        let mut bus: MockSccbBus<512> = MockSccbBus::new();
        bus.set_reg8(CameraSccbAddr::GC032A, 0xF0, 0x23);
        bus.set_reg8(CameraSccbAddr::GC032A, 0xF1, 0x2A);

        assert_eq!(
            Gc032a::detect(&mut bus, 0x00),
            Err(CameraError::NotDetected)
        );
        let id = Gc032a::detect(&mut bus, CameraSccbAddr::GC032A);
        assert!(id.is_ok());
        assert_eq!(id.map(|i| i.pid), Ok(CameraPid::GC032A.as_u16()));

        let mut sensor = Gc032a::new();
        assert_eq!(sensor.model(), CameraModel::Gc032a);
        assert_eq!(sensor.reset(&mut bus), Ok(()));
        assert_eq!(sensor.set_pixformat(&mut bus, PixelFormat::Yuv422), Ok(()));
        assert_eq!(sensor.set_framesize(&mut bus, FrameSize::Vga), Ok(()));
        assert_eq!(sensor.status().framesize, FrameSize::Vga);
        assert_eq!(sensor.set_hmirror(&mut bus, true), Ok(()));
        assert_eq!(sensor.status().hmirror, 1);
        assert_eq!(sensor.set_vflip(&mut bus, true), Ok(()));
        assert_eq!(sensor.status().vflip, 1);
        assert_eq!(sensor.set_reg(&mut bus, 0x44, 0xFF, 0x83), Ok(()));
        assert_eq!(sensor.get_reg(&mut bus, 0x44, 0xFF), Ok(0x83));
    }

    #[test]
    fn test_bf3005_lifecycle() {
        let mut bus: MockSccbBus<512> = MockSccbBus::new();
        bus.set_reg8(CameraSccbAddr::BF3005, 0xFC, 0x30);
        bus.set_reg8(CameraSccbAddr::BF3005, 0xFD, 0x05);

        assert_eq!(
            Bf3005::detect(&mut bus, 0x00),
            Err(CameraError::NotDetected)
        );
        let id = Bf3005::detect(&mut bus, CameraSccbAddr::BF3005);
        assert!(id.is_ok());
        assert_eq!(id.map(|i| i.pid), Ok(CameraPid::BF3005.as_u16()));

        let mut sensor = Bf3005::new();
        assert_eq!(sensor.model(), CameraModel::Bf3005);
        assert_eq!(sensor.reset(&mut bus), Ok(()));
        assert_eq!(sensor.set_pixformat(&mut bus, PixelFormat::Rgb565), Ok(()));
        assert_eq!(sensor.set_framesize(&mut bus, FrameSize::Qvga), Ok(()));
        assert_eq!(sensor.status().framesize, FrameSize::Qvga);
        assert_eq!(sensor.set_hmirror(&mut bus, true), Ok(()));
        assert_eq!(sensor.status().hmirror, 1);
        assert_eq!(sensor.set_vflip(&mut bus, true), Ok(()));
        assert_eq!(sensor.status().vflip, 1);
        assert_eq!(sensor.set_reg(&mut bus, 0x55, 0xFF, 0x42), Ok(()));
        assert_eq!(sensor.get_reg(&mut bus, 0x55, 0xFF), Ok(0x42));
    }

    #[test]
    fn test_bf20a6_lifecycle() {
        let mut bus: MockSccbBus<512> = MockSccbBus::new();
        bus.set_reg8(CameraSccbAddr::BF20A6, 0xFC, 0x20);
        bus.set_reg8(CameraSccbAddr::BF20A6, 0xFD, 0xA6);

        assert_eq!(
            Bf20a6::detect(&mut bus, 0x00),
            Err(CameraError::NotDetected)
        );
        let id = Bf20a6::detect(&mut bus, CameraSccbAddr::BF20A6);
        assert!(id.is_ok());
        assert_eq!(id.map(|i| i.pid), Ok(CameraPid::BF20A6.as_u16()));

        let mut sensor = Bf20a6::new();
        assert_eq!(sensor.model(), CameraModel::Bf20a6);
        assert_eq!(sensor.reset(&mut bus), Ok(()));
        assert_eq!(sensor.set_pixformat(&mut bus, PixelFormat::Yuv422), Ok(()));
        assert_eq!(sensor.set_framesize(&mut bus, FrameSize::Vga), Ok(()));
        assert_eq!(sensor.status().framesize, FrameSize::Vga);
        assert_eq!(sensor.set_hmirror(&mut bus, true), Ok(()));
        assert_eq!(sensor.status().hmirror, 1);
        assert_eq!(sensor.set_vflip(&mut bus, true), Ok(()));
        assert_eq!(sensor.status().vflip, 1);
        assert_eq!(sensor.set_reg(&mut bus, 0x70, 0xFF, 0x77), Ok(()));
        assert_eq!(sensor.get_reg(&mut bus, 0x70, 0xFF), Ok(0x77));
    }

    #[test]
    fn test_sc101iot_lifecycle() {
        let mut bus: MockSccbBus<512> = MockSccbBus::new();
        bus.set_reg8(CameraSccbAddr::SC101IOT, 0xF7, 0xDA);
        bus.set_reg8(CameraSccbAddr::SC101IOT, 0xF8, 0x4A);

        assert_eq!(
            Sc101Iot::detect(&mut bus, 0x00),
            Err(CameraError::NotDetected)
        );
        let id = Sc101Iot::detect(&mut bus, CameraSccbAddr::SC101IOT);
        assert!(id.is_ok());
        assert_eq!(id.map(|i| i.pid), Ok(CameraPid::SC101IOT.as_u16()));

        let mut sensor = Sc101Iot::new();
        assert_eq!(sensor.model(), CameraModel::Sc101Iot);
        assert_eq!(sensor.reset(&mut bus), Ok(()));
        assert_eq!(sensor.set_pixformat(&mut bus, PixelFormat::Yuv422), Ok(()));
        assert_eq!(
            sensor.set_pixformat(&mut bus, PixelFormat::Rgb565),
            Err(CameraError::NotSupported)
        );
        assert_eq!(sensor.set_framesize(&mut bus, FrameSize::Hd), Ok(()));
        assert_eq!(sensor.status().framesize, FrameSize::Hd);
        assert_eq!(sensor.set_hmirror(&mut bus, true), Ok(()));
        assert_eq!(sensor.status().hmirror, 1);
        assert_eq!(sensor.set_vflip(&mut bus, true), Ok(()));
        assert_eq!(sensor.status().vflip, 1);
        assert_eq!(sensor.set_reg(&mut bus, 0x014B, 0xFF, 0x40), Ok(()));
        assert_eq!(sensor.get_reg(&mut bus, 0x014B, 0xFF), Ok(0x40));
    }

    #[test]
    fn test_sc030iot_lifecycle() {
        let mut bus: MockSccbBus<512> = MockSccbBus::new();
        bus.set_reg8(CameraSccbAddr::SC030IOT, 0xF7, 0x9A);
        bus.set_reg8(CameraSccbAddr::SC030IOT, 0xF8, 0x46);

        assert_eq!(
            Sc030Iot::detect(&mut bus, 0x00),
            Err(CameraError::NotDetected)
        );
        let id = Sc030Iot::detect(&mut bus, CameraSccbAddr::SC030IOT);
        assert!(id.is_ok());
        assert_eq!(id.map(|i| i.pid), Ok(CameraPid::SC030IOT.as_u16()));

        let mut sensor = Sc030Iot::new();
        assert_eq!(sensor.model(), CameraModel::Sc030Iot);
        assert_eq!(sensor.reset(&mut bus), Ok(()));
        assert_eq!(sensor.set_pixformat(&mut bus, PixelFormat::Yuv422), Ok(()));
        assert_eq!(sensor.set_framesize(&mut bus, FrameSize::Vga), Ok(()));
        assert_eq!(sensor.status().framesize, FrameSize::Vga);
        assert_eq!(sensor.set_hmirror(&mut bus, true), Ok(()));
        assert_eq!(sensor.status().hmirror, 1);
        assert_eq!(sensor.set_vflip(&mut bus, true), Ok(()));
        assert_eq!(sensor.status().vflip, 1);
        assert_eq!(sensor.set_reg(&mut bus, 0x014B, 0xFF, 0x33), Ok(()));
        assert_eq!(sensor.get_reg(&mut bus, 0x014B, 0xFF), Ok(0x33));

        // Verify signed SCCB error propagation on page-select and register accesses
        bus.set_fail(true);
        assert_eq!(
            sensor.get_reg(&mut bus, 0x014B, 0xFF),
            Err(CameraError::SccbError)
        );
        assert_eq!(
            sensor.set_hmirror(&mut bus, false),
            Err(CameraError::SccbError)
        );
    }

    #[test]
    fn test_sc031gs_lifecycle() {
        let mut bus: MockSccbBus<512> = MockSccbBus::new();
        bus.set_reg16(CameraSccbAddr::SC031GS, 0x3107, 0x00);
        bus.set_reg16(CameraSccbAddr::SC031GS, 0x3108, 0x31);

        assert_eq!(
            Sc031Gs::detect(&mut bus, 0x00),
            Err(CameraError::NotDetected)
        );
        let id = Sc031Gs::detect(&mut bus, CameraSccbAddr::SC031GS);
        assert!(id.is_ok());
        assert_eq!(id.map(|i| i.pid), Ok(CameraPid::SC031GS.as_u16()));

        let mut sensor = Sc031Gs::new();
        assert_eq!(sensor.model(), CameraModel::Sc031Gs);
        assert_eq!(sensor.reset(&mut bus), Ok(()));
        assert_eq!(
            sensor.set_pixformat(&mut bus, PixelFormat::Grayscale),
            Ok(())
        );
        assert_eq!(sensor.set_framesize(&mut bus, FrameSize::Vga), Ok(()));
        assert_eq!(sensor.status().framesize, FrameSize::Vga);
        assert_eq!(sensor.set_hmirror(&mut bus, true), Ok(()));
        assert_eq!(sensor.status().hmirror, 1);
        assert_eq!(sensor.set_vflip(&mut bus, true), Ok(()));
        assert_eq!(sensor.status().vflip, 1);
        assert_eq!(sensor.set_reg(&mut bus, 0x3E01, 0xFF, 0x18), Ok(()));
        assert_eq!(sensor.get_reg(&mut bus, 0x3E01, 0xFF), Ok(0x18));
    }

    #[test]
    fn test_mega_ccm_lifecycle() {
        let mut bus: MockSccbBus<512> = MockSccbBus::new();
        bus.set_reg16(CameraSccbAddr::MEGA_CCM, 0x0000, 0x03);
        bus.set_reg16(CameraSccbAddr::MEGA_CCM, 0x0001, 0x9E);

        assert_eq!(
            MegaCcm::detect(&mut bus, 0x00),
            Err(CameraError::NotDetected)
        );
        let id = MegaCcm::detect(&mut bus, CameraSccbAddr::MEGA_CCM);
        assert!(id.is_ok());
        assert_eq!(id.map(|i| i.pid), Ok(CameraPid::MEGA_CCM.as_u16()));

        let mut sensor = MegaCcm::new();
        assert_eq!(sensor.model(), CameraModel::MegaCcm);
        assert_eq!(sensor.reset(&mut bus), Ok(()));
        assert_eq!(sensor.set_pixformat(&mut bus, PixelFormat::Jpeg), Ok(()));
        assert_eq!(sensor.set_framesize(&mut bus, FrameSize::Fhd), Ok(()));
        assert_eq!(sensor.status().framesize, FrameSize::Fhd);
        assert_eq!(sensor.set_hmirror(&mut bus, true), Ok(()));
        assert_eq!(sensor.status().hmirror, 1);
        assert_eq!(sensor.set_vflip(&mut bus, true), Ok(()));
        assert_eq!(sensor.status().vflip, 1);
        assert_eq!(sensor.set_reg(&mut bus, 0x012A, 0xFF, 0x0A), Ok(()));
        assert_eq!(sensor.get_reg(&mut bus, 0x012A, 0xFF), Ok(0x0A));
        assert_eq!(
            sensor.get_reg(&mut bus, 0x1_0000, 0xFF),
            Err(CameraError::InvalidArg)
        );
    }

    #[test]
    fn test_hm1055_lifecycle() {
        let mut bus: MockSccbBus<512> = MockSccbBus::new();
        bus.set_reg16(CameraSccbAddr::HM1055, 0x0001, 0x09);
        bus.set_reg16(CameraSccbAddr::HM1055, 0x0002, 0x55);

        assert_eq!(
            Hm1055::detect(&mut bus, 0x00),
            Err(CameraError::NotDetected)
        );
        let id = Hm1055::detect(&mut bus, CameraSccbAddr::HM1055);
        assert!(id.is_ok());
        assert_eq!(id.map(|i| i.pid), Ok(CameraPid::HM1055.as_u16()));

        let mut sensor = Hm1055::new();
        assert_eq!(sensor.model(), CameraModel::Hm1055);
        assert_eq!(sensor.reset(&mut bus), Ok(()));
        assert_eq!(sensor.set_pixformat(&mut bus, PixelFormat::Rgb565), Ok(()));
        assert_eq!(sensor.set_framesize(&mut bus, FrameSize::Hd), Ok(()));
        assert_eq!(sensor.status().framesize, FrameSize::Hd);
        assert_eq!(sensor.set_hmirror(&mut bus, true), Ok(()));
        assert_eq!(sensor.status().hmirror, 1);
        assert_eq!(sensor.set_vflip(&mut bus, true), Ok(()));
        assert_eq!(sensor.status().vflip, 1);
        assert_eq!(sensor.set_lenc(&mut bus, true), Ok(()));
        assert_eq!(sensor.set_reg(&mut bus, 0x0018, 0xFF, 0x05), Ok(()));
        assert_eq!(sensor.get_reg(&mut bus, 0x0018, 0xFF), Ok(0x05));
    }

    #[test]
    fn test_hm0360_lifecycle() {
        let mut bus: MockSccbBus<512> = MockSccbBus::new();
        bus.set_reg16(CameraSccbAddr::HM0360, 0x0000, 0x03);
        bus.set_reg16(CameraSccbAddr::HM0360, 0x0001, 0x60);
        bus.set_reg16(CameraSccbAddr::HM0360, 0x0002, 0x01);

        assert_eq!(
            Hm0360::detect(&mut bus, 0x00),
            Err(CameraError::NotDetected)
        );
        let id = Hm0360::detect(&mut bus, CameraSccbAddr::HM0360);
        assert!(id.is_ok());
        assert_eq!(id.map(|i| i.pid), Ok(CameraPid::HM0360.as_u16()));

        let mut sensor = Hm0360::new();
        assert_eq!(sensor.model(), CameraModel::Hm0360);
        assert_eq!(sensor.reset(&mut bus), Ok(()));
        assert_eq!(
            sensor.set_pixformat(&mut bus, PixelFormat::Grayscale),
            Ok(())
        );
        assert_eq!(sensor.set_framesize(&mut bus, FrameSize::Qvga), Ok(()));
        assert_eq!(sensor.status().framesize, FrameSize::Qvga);
        assert_eq!(sensor.set_hmirror(&mut bus, true), Ok(()));
        assert_eq!(sensor.status().hmirror, 1);
        assert_eq!(sensor.set_vflip(&mut bus, true), Ok(()));
        assert_eq!(sensor.status().vflip, 1);
        assert_eq!(sensor.set_reg(&mut bus, 0x2034, 0xFF, 0x64), Ok(()));
        assert_eq!(sensor.get_reg(&mut bus, 0x2034, 0xFF), Ok(0x64));
    }
}
