//! `embedded-hal` 1.0 adapter implementing [`SccbBus`] over I2C and Delay.

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::{ErrorKind, I2c};
use esp32_cam_types::CameraError;

use crate::bus::SccbBus;

/// Maps an `embedded-hal` I2C error to [`CameraError`].
fn map_i2c_error<E: embedded_hal::i2c::Error>(e: E, slv_addr: u8) -> CameraError {
    match embedded_hal::i2c::Error::kind(&e) {
        ErrorKind::NoAcknowledge(_) => CameraError::SccbNack {
            slv_addr,
            phase: "device_addr",
        },
        ErrorKind::Bus | ErrorKind::ArbitrationLoss | ErrorKind::Overrun => CameraError::I2cError,
        _ => CameraError::SccbError,
    }
}

/// `embedded-hal` 1.0 adapter implementing [`SccbBus`] over I2C and Delay.
///
/// Follows the two-phase SCCB read protocol from `driver/sccb.c` (separate
/// write of register address followed by STOP and subsequent read).
#[derive(Debug)]
pub struct EmbeddedHalSccb<I2C, D> {
    i2c: I2C,
    delay: D,
}

impl<I2C, D> EmbeddedHalSccb<I2C, D> {
    /// Creates a new [`EmbeddedHalSccb`] bus wrapper.
    pub const fn new(i2c: I2C, delay: D) -> Self {
        Self { i2c, delay }
    }

    /// Consumes the adapter and returns the underlying `(I2C, D)` peripherals.
    pub fn into_parts(self) -> (I2C, D) {
        (self.i2c, self.delay)
    }

    /// Returns a shared reference to the underlying I2C bus.
    pub const fn i2c(&self) -> &I2C {
        &self.i2c
    }

    /// Returns a mutable reference to the underlying I2C bus.
    pub fn i2c_mut(&mut self) -> &mut I2C {
        &mut self.i2c
    }
}

impl<I2C, D> SccbBus for EmbeddedHalSccb<I2C, D>
where
    I2C: I2c,
    D: DelayNs,
{
    fn read_reg8(&mut self, slv_addr: u8, reg: u8) -> Result<u8, CameraError> {
        if slv_addr == 0 || slv_addr >= 0x80 {
            return Err(CameraError::InvalidArg);
        }
        let mut data = [0u8; 1];
        self.i2c
            .write(slv_addr, &[reg])
            .map_err(|e| map_i2c_error(e, slv_addr))?;
        self.i2c
            .read(slv_addr, &mut data)
            .map_err(|e| map_i2c_error(e, slv_addr))?;
        Ok(data[0])
    }

    fn write_reg8(&mut self, slv_addr: u8, reg: u8, data: u8) -> Result<(), CameraError> {
        if slv_addr == 0 || slv_addr >= 0x80 {
            return Err(CameraError::InvalidArg);
        }
        self.i2c
            .write(slv_addr, &[reg, data])
            .map_err(|e| map_i2c_error(e, slv_addr))
    }

    fn read_reg16(&mut self, slv_addr: u8, reg: u16) -> Result<u8, CameraError> {
        if slv_addr == 0 || slv_addr >= 0x80 {
            return Err(CameraError::InvalidArg);
        }
        let reg_bytes = reg.to_be_bytes();
        let mut data = [0u8; 1];
        self.i2c
            .write(slv_addr, &reg_bytes)
            .map_err(|e| map_i2c_error(e, slv_addr))?;
        self.i2c
            .read(slv_addr, &mut data)
            .map_err(|e| map_i2c_error(e, slv_addr))?;
        Ok(data[0])
    }

    fn write_reg16(&mut self, slv_addr: u8, reg: u16, data: u8) -> Result<(), CameraError> {
        if slv_addr == 0 || slv_addr >= 0x80 {
            return Err(CameraError::InvalidArg);
        }
        let reg_bytes = reg.to_be_bytes();
        self.i2c
            .write(slv_addr, &[reg_bytes[0], reg_bytes[1], data])
            .map_err(|e| map_i2c_error(e, slv_addr))
    }

    fn probe(&mut self, slv_addr: u8) -> Result<(), CameraError> {
        if slv_addr == 0 || slv_addr >= 0x80 {
            return Err(CameraError::InvalidArg);
        }
        self.i2c
            .write(slv_addr, &[])
            .map_err(|e| map_i2c_error(e, slv_addr))
    }

    fn delay_ms(&mut self, ms: u32) {
        self.delay.delay_ms(ms);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal::i2c::{Error, ErrorKind, ErrorType, NoAcknowledgeSource, Operation};

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    struct MockI2cError(ErrorKind);

    impl Error for MockI2cError {
        fn kind(&self) -> ErrorKind {
            self.0
        }
    }

    struct MockI2c {
        err: Option<ErrorKind>,
        read_data: u8,
    }

    impl ErrorType for MockI2c {
        type Error = MockI2cError;
    }

    impl I2c for MockI2c {
        fn transaction(
            &mut self,
            _address: u8,
            operations: &mut [Operation<'_>],
        ) -> Result<(), Self::Error> {
            if let Some(err) = self.err {
                return Err(MockI2cError(err));
            }
            for op in operations {
                if let Operation::Read(buf) = op {
                    for b in buf.iter_mut() {
                        *b = self.read_data;
                    }
                }
            }
            Ok(())
        }
    }

    struct MockDelay;

    impl DelayNs for MockDelay {
        fn delay_ns(&mut self, _ns: u32) {}
    }

    #[test]
    fn test_map_i2c_error_variants() {
        let nack_err = MockI2cError(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address));
        assert_eq!(
            map_i2c_error(nack_err, 0x30),
            CameraError::SccbNack {
                slv_addr: 0x30,
                phase: "device_addr",
            }
        );

        let bus_err = MockI2cError(ErrorKind::Bus);
        assert_eq!(map_i2c_error(bus_err, 0x30), CameraError::I2cError);

        let arb_err = MockI2cError(ErrorKind::ArbitrationLoss);
        assert_eq!(map_i2c_error(arb_err, 0x30), CameraError::I2cError);

        let over_err = MockI2cError(ErrorKind::Overrun);
        assert_eq!(map_i2c_error(over_err, 0x30), CameraError::I2cError);

        let other_err = MockI2cError(ErrorKind::Other);
        assert_eq!(map_i2c_error(other_err, 0x30), CameraError::SccbError);
    }

    #[test]
    fn test_adapter_operations_success_and_errors() {
        let mut adapter = EmbeddedHalSccb::new(
            MockI2c {
                err: None,
                read_data: 0x42,
            },
            MockDelay,
        );

        assert_eq!(adapter.read_reg8(0x30, 0x10), Ok(0x42));
        assert_eq!(adapter.write_reg8(0x30, 0x10, 0x55), Ok(()));
        assert_eq!(adapter.read_reg16(0x30, 0x3000), Ok(0x42));
        assert_eq!(adapter.write_reg16(0x30, 0x3000, 0x55), Ok(()));
        assert_eq!(adapter.probe(0x30), Ok(()));

        // Test error propagation to SccbNack
        adapter.i2c_mut().err = Some(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address));
        assert_eq!(
            adapter.probe(0x30),
            Err(CameraError::SccbNack {
                slv_addr: 0x30,
                phase: "device_addr",
            })
        );
        assert_eq!(
            adapter.write_reg8(0x30, 0x10, 0x55),
            Err(CameraError::SccbNack {
                slv_addr: 0x30,
                phase: "device_addr",
            })
        );

        // Test error propagation to I2cError
        adapter.i2c_mut().err = Some(ErrorKind::Bus);
        assert_eq!(adapter.probe(0x30), Err(CameraError::I2cError));
        assert_eq!(adapter.read_reg8(0x30, 0x10), Err(CameraError::I2cError));
    }
}
