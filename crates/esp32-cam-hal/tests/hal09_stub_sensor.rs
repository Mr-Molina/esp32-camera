//! HAL-09: the built-in `HalOv2640Sensor` is a stub, and must not report
//! success for configuration it never performs.

use esp32_cam_hal::{HalCameraSensor, HalOv2640Sensor};
use esp32_cam_sccb::{MockSccbBus, SccbTransaction};
use esp32_cam_types::{CameraError, CameraSccbAddr, FrameSize, GainCeiling, PixelFormat};

/// Every setter opened with the OV2640 bank-select prelude and then stopped,
/// recording the requested value and returning `Ok`. Only `set_quality` went
/// on to write a real register. So `set_framesize(Hd)` selected a bank, wrote
/// nothing else, reported success and recorded HD — while the sensor stayed
/// at whatever reset had left it. The single bank-select write is what made
/// it look functional.
#[test]
fn test_hal09_incomplete_setters_refuse_instead_of_claiming_success() {
    let mut bus: MockSccbBus<128> = MockSccbBus::new();
    let mut sensor = HalOv2640Sensor::new();
    let before = sensor.status().framesize;

    assert_eq!(
        HalCameraSensor::set_framesize(&mut sensor, &mut bus, FrameSize::Hd),
        Err(CameraError::NotSupported)
    );
    assert_eq!(
        HalCameraSensor::set_pixformat(&mut sensor, &mut bus, PixelFormat::Jpeg),
        Err(CameraError::NotSupported)
    );
    assert_eq!(
        HalCameraSensor::set_gainceiling(&mut sensor, &mut bus, GainCeiling::X16),
        Err(CameraError::NotSupported)
    );

    assert_eq!(
        sensor.status().framesize,
        before,
        "a refused framesize must not be recorded"
    );
    assert_eq!(
        bus.transaction_count(),
        0,
        "a refusing setter must not even select a bank"
    );
}

/// `set_quality` and `reset` are the parts that were finished; they keep
/// working, so the stub is still useful for what it actually implements.
#[test]
fn test_hal09_implemented_operations_still_work() {
    let mut bus: MockSccbBus<128> = MockSccbBus::new();
    let mut sensor = HalOv2640Sensor::new();

    assert_eq!(
        HalCameraSensor::set_quality(&mut sensor, &mut bus, 12),
        Ok(())
    );
    assert!(
        bus.transactions().iter().any(|tx| matches!(
            tx,
            SccbTransaction::Write8 { slv_addr, reg: 0x44, val: 12 }
                if *slv_addr == CameraSccbAddr::OV2640
        )),
        "set_quality must still write the QS register"
    );

    assert_eq!(HalCameraSensor::reset(&mut sensor, &mut bus), Ok(()));
}
