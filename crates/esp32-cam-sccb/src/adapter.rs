//! `embedded-hal` 1.0 adapter implementing [`SccbBus`] over I2C and Delay.

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::{ErrorKind, I2c, NoAcknowledgeSource, Operation};
use esp32_cam_types::{CameraError, SccbPhase};

use crate::bus::{is_valid_sccb_addr, SccbBus};

/// Maps an `embedded-hal` I2C error to [`CameraError`].
fn map_i2c_error<E: embedded_hal::i2c::Error>(e: E, slv_addr: u8) -> CameraError {
    match embedded_hal::i2c::Error::kind(&e) {
        ErrorKind::NoAcknowledge(source) => {
            let phase = match source {
                NoAcknowledgeSource::Address => SccbPhase::DeviceAddr,
                NoAcknowledgeSource::Data => SccbPhase::Data,
                NoAcknowledgeSource::Unknown => SccbPhase::Unknown,
            };
            CameraError::SccbNack { slv_addr, phase }
        }
        ErrorKind::Bus | ErrorKind::ArbitrationLoss | ErrorKind::Overrun => CameraError::I2cError,
        _ => CameraError::SccbError,
    }
}

/// `embedded-hal` 1.0 adapter implementing [`SccbBus`] over I2C and Delay.
///
/// Follows the two-phase SCCB read protocol from `driver/sccb.c` (separate
/// write of register address followed by STOP and subsequent read) and single
/// scatter-gather `transaction` bursts for sequential register writes.
#[derive(Debug)]
pub struct EmbeddedHalSccb<I2C, D> {
    i2c: I2C,
    delay: D,
}

impl<I2C, D> EmbeddedHalSccb<I2C, D> {
    /// Creates a new [`EmbeddedHalSccb`] bus wrapper.
    #[must_use]
    pub const fn new(i2c: I2C, delay: D) -> Self {
        Self { i2c, delay }
    }

    /// Consumes the adapter and returns the underlying `(I2C, D)` peripherals.
    #[must_use]
    pub fn into_parts(self) -> (I2C, D) {
        (self.i2c, self.delay)
    }

    /// Returns a shared reference to the underlying I2C bus.
    #[must_use]
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
        if !is_valid_sccb_addr(slv_addr) {
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
        if !is_valid_sccb_addr(slv_addr) {
            return Err(CameraError::InvalidArg);
        }
        self.i2c
            .write(slv_addr, &[reg, data])
            .map_err(|e| map_i2c_error(e, slv_addr))
    }

    fn read_reg16(&mut self, slv_addr: u8, reg: u16) -> Result<u8, CameraError> {
        if !is_valid_sccb_addr(slv_addr) {
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
        if !is_valid_sccb_addr(slv_addr) {
            return Err(CameraError::InvalidArg);
        }
        let reg_bytes = reg.to_be_bytes();
        self.i2c
            .write(slv_addr, &[reg_bytes[0], reg_bytes[1], data])
            .map_err(|e| map_i2c_error(e, slv_addr))
    }

    fn write_reg8_seq(
        &mut self,
        slv_addr: u8,
        start_reg: u8,
        data: &[u8],
    ) -> Result<(), CameraError> {
        if !is_valid_sccb_addr(slv_addr) {
            return Err(CameraError::InvalidArg);
        }
        if data.is_empty() {
            return Ok(());
        }
        let last_off = u8::try_from(data.len() - 1).map_err(|_| CameraError::InvalidArg)?;
        let _ = start_reg
            .checked_add(last_off)
            .ok_or(CameraError::InvalidArg)?;
        let addr_buf = [start_reg];
        let mut ops = [Operation::Write(&addr_buf), Operation::Write(data)];
        self.i2c
            .transaction(slv_addr, &mut ops)
            .map_err(|e| map_i2c_error(e, slv_addr))
    }

    fn write_reg16_seq(
        &mut self,
        slv_addr: u8,
        start_reg: u16,
        data: &[u8],
    ) -> Result<(), CameraError> {
        if !is_valid_sccb_addr(slv_addr) {
            return Err(CameraError::InvalidArg);
        }
        if data.is_empty() {
            return Ok(());
        }
        let last_off = u16::try_from(data.len() - 1).map_err(|_| CameraError::InvalidArg)?;
        let _ = start_reg
            .checked_add(last_off)
            .ok_or(CameraError::InvalidArg)?;
        let addr_buf = start_reg.to_be_bytes();
        let mut ops = [Operation::Write(&addr_buf), Operation::Write(data)];
        self.i2c
            .transaction(slv_addr, &mut ops)
            .map_err(|e| map_i2c_error(e, slv_addr))
    }

    fn probe(&mut self, slv_addr: u8) -> Result<(), CameraError> {
        if !is_valid_sccb_addr(slv_addr) {
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
    use embedded_hal::i2c::{Error, ErrorType};

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
        tx_calls: usize,
        written: [u8; 32],
        written_len: usize,
    }

    impl MockI2c {
        const fn new(read_data: u8) -> Self {
            Self {
                err: None,
                read_data,
                tx_calls: 0,
                written: [0; 32],
                written_len: 0,
            }
        }
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
            self.tx_calls += 1;
            if let Some(err) = self.err {
                return Err(MockI2cError(err));
            }
            for op in operations {
                match op {
                    Operation::Read(buf) => {
                        for b in buf.iter_mut() {
                            *b = self.read_data;
                        }
                    }
                    Operation::Write(bytes) => {
                        for &b in *bytes {
                            if self.written_len < self.written.len() {
                                self.written[self.written_len] = b;
                                self.written_len += 1;
                            }
                        }
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
        let addr_nack = MockI2cError(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address));
        assert_eq!(
            map_i2c_error(addr_nack, 0x30),
            CameraError::SccbNack {
                slv_addr: 0x30,
                phase: SccbPhase::DeviceAddr,
            }
        );

        let data_nack = MockI2cError(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Data));
        assert_eq!(
            map_i2c_error(data_nack, 0x30),
            CameraError::SccbNack {
                slv_addr: 0x30,
                phase: SccbPhase::Data,
            }
        );

        let unknown_nack = MockI2cError(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Unknown));
        assert_eq!(
            map_i2c_error(unknown_nack, 0x30),
            CameraError::SccbNack {
                slv_addr: 0x30,
                phase: SccbPhase::Unknown,
            }
        );

        assert_eq!(
            map_i2c_error(MockI2cError(ErrorKind::Bus), 0x30),
            CameraError::I2cError
        );
        assert_eq!(
            map_i2c_error(MockI2cError(ErrorKind::ArbitrationLoss), 0x30),
            CameraError::I2cError
        );
        assert_eq!(
            map_i2c_error(MockI2cError(ErrorKind::Overrun), 0x30),
            CameraError::I2cError
        );
        assert_eq!(
            map_i2c_error(MockI2cError(ErrorKind::Other), 0x30),
            CameraError::SccbError
        );
    }

    #[test]
    fn test_adapter_operations_and_seq_burst() {
        let mut adapter = EmbeddedHalSccb::new(MockI2c::new(0x42), MockDelay);

        assert_eq!(adapter.read_reg8(0x30, 0x10), Ok(0x42));
        assert_eq!(adapter.write_reg8(0x30, 0x10, 0x55), Ok(()));
        assert_eq!(adapter.read_reg16(0x30, 0x3000), Ok(0x42));
        assert_eq!(adapter.write_reg16(0x30, 0x3000, 0x55), Ok(()));
        assert_eq!(adapter.probe(0x30), Ok(()));

        // SCCB-02: write_reg16_seq emits a single I2C transaction [addr_be, payload].
        adapter.i2c_mut().tx_calls = 0;
        adapter.i2c_mut().written_len = 0;
        assert_eq!(
            adapter.write_reg16_seq(0x3C, 0x8000, &[0x11, 0x22, 0x33, 0x44]),
            Ok(())
        );
        assert_eq!(
            adapter.i2c().tx_calls,
            1,
            "write_reg16_seq must issue 1 I2C transaction"
        );
        assert_eq!(
            &adapter.i2c().written[..adapter.i2c().written_len],
            &[0x80, 0x00, 0x11, 0x22, 0x33, 0x44]
        );

        adapter.i2c_mut().tx_calls = 0;
        adapter.i2c_mut().written_len = 0;
        assert_eq!(adapter.write_reg8_seq(0x30, 0x10, &[0xAA, 0xBB]), Ok(()));
        assert_eq!(
            adapter.i2c().tx_calls,
            1,
            "write_reg8_seq must issue 1 I2C transaction"
        );
        assert_eq!(
            &adapter.i2c().written[..adapter.i2c().written_len],
            &[0x10, 0xAA, 0xBB]
        );

        adapter.i2c_mut().err = Some(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address));
        assert_eq!(
            adapter.probe(0x30),
            Err(CameraError::SccbNack {
                slv_addr: 0x30,
                phase: SccbPhase::DeviceAddr,
            })
        );
        adapter.i2c_mut().err = Some(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Data));
        assert_eq!(
            adapter.write_reg16_seq(0x3C, 0x3000, &[0x01, 0x02]),
            Err(CameraError::SccbNack {
                slv_addr: 0x3C,
                phase: SccbPhase::Data,
            })
        );
        adapter.i2c_mut().err = Some(ErrorKind::Bus);
        assert_eq!(adapter.read_reg8(0x30, 0x10), Err(CameraError::I2cError));
    }
}
