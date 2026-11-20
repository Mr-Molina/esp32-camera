//! `embedded-hal` 1.0 adapter implementing [`SccbBus`] over I2C and Delay.

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::{ErrorKind, I2c, NoAcknowledgeSource, Operation};
use esp32_cam_types::{CameraError, SccbPhase};

use crate::bus::{is_valid_sccb_addr, SccbBus};

/// Maps an `embedded-hal` I2C error to [`CameraError`], tagging data-phase
/// NACKs with `data_phase` so register-address writes report
/// [`SccbPhase::RegAddr`].
fn map_i2c_error_with_phase<E: embedded_hal::i2c::Error>(
    e: E,
    slv_addr: u8,
    data_phase: SccbPhase,
) -> CameraError {
    match embedded_hal::i2c::Error::kind(&e) {
        ErrorKind::NoAcknowledge(source) => {
            let phase = match source {
                NoAcknowledgeSource::Address => SccbPhase::DeviceAddr,
                NoAcknowledgeSource::Data => data_phase,
                NoAcknowledgeSource::Unknown => SccbPhase::Unknown,
            };
            CameraError::SccbNack { slv_addr, phase }
        }
        ErrorKind::Bus | ErrorKind::ArbitrationLoss | ErrorKind::Overrun => CameraError::I2cError,
        _ => CameraError::SccbError,
    }
}

/// Maps an `embedded-hal` I2C error to [`CameraError`].
fn map_i2c_error<E: embedded_hal::i2c::Error>(e: E, slv_addr: u8) -> CameraError {
    map_i2c_error_with_phase(e, slv_addr, SccbPhase::Data)
}

/// Returns true when an `embedded-hal` I2C error is a transient
/// multi-master arbitration loss that is safe to retry for idempotent reads.
fn is_arbitration_loss<E: embedded_hal::i2c::Error>(e: &E) -> bool {
    matches!(
        embedded_hal::i2c::Error::kind(e),
        ErrorKind::ArbitrationLoss
    )
}

/// `embedded-hal` 1.0 adapter implementing [`SccbBus`] over I2C and Delay.
///
/// Follows the two-phase SCCB read protocol from `driver/sccb.c` (separate
/// write of register address followed by STOP and subsequent read) and single
/// scatter-gather `transaction` bursts for sequential register writes.
///
/// # Shared-Bus Locking Requirement
/// OmniVision SCCB sensors forbid I2C Repeated-START between the register
/// address write and data read, so [`SccbBus::read_reg8`] and
/// [`SccbBus::read_reg16`] must issue two separate bus calls with an
/// intervening STOP. Wrap `I2C` exclusively (or hold an external shared-bus
/// lock across each `SccbBus` call) so another master/task cannot interleave
/// between the two SCCB phases.
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
        for attempt in 0..2u8 {
            if let Err(e) = self.i2c.write(slv_addr, &[reg]) {
                if attempt == 0 && is_arbitration_loss(&e) {
                    continue;
                }
                return Err(map_i2c_error_with_phase(e, slv_addr, SccbPhase::RegAddr));
            }
            let mut data = [0u8; 1];
            match self.i2c.read(slv_addr, &mut data) {
                Ok(()) => return Ok(data[0]),
                Err(e) if attempt == 0 && is_arbitration_loss(&e) => continue,
                Err(e) => return Err(map_i2c_error_with_phase(e, slv_addr, SccbPhase::Data)),
            }
        }
        Err(CameraError::I2cError)
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
        for attempt in 0..2u8 {
            if let Err(e) = self.i2c.write(slv_addr, &reg_bytes) {
                if attempt == 0 && is_arbitration_loss(&e) {
                    continue;
                }
                return Err(map_i2c_error_with_phase(e, slv_addr, SccbPhase::RegAddr));
            }
            let mut data = [0u8; 1];
            match self.i2c.read(slv_addr, &mut data) {
                Ok(()) => return Ok(data[0]),
                Err(e) if attempt == 0 && is_arbitration_loss(&e) => continue,
                Err(e) => return Err(map_i2c_error_with_phase(e, slv_addr, SccbPhase::Data)),
            }
        }
        Err(CameraError::I2cError)
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
        // Use a portable 1-byte read rather than a zero-length write, which
        // embedded-hal 1.0 drivers are not required to support consistently.
        let mut byte = [0u8; 1];
        for attempt in 0..2u8 {
            match self.i2c.read(slv_addr, &mut byte) {
                Ok(()) => return Ok(()),
                Err(e) if attempt == 0 && is_arbitration_loss(&e) => continue,
                Err(e) => return Err(map_i2c_error(e, slv_addr)),
            }
        }
        Err(CameraError::I2cError)
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
        fail_once_err: Option<ErrorKind>,
        read_data: u8,
        tx_calls: usize,
        read_calls: usize,
        written: [u8; 32],
        written_len: usize,
    }

    impl MockI2c {
        const fn new(read_data: u8) -> Self {
            Self {
                err: None,
                fail_once_err: None,
                read_data,
                tx_calls: 0,
                read_calls: 0,
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
            if let Some(err) = self.fail_once_err.take() {
                return Err(MockI2cError(err));
            }
            if let Some(err) = self.err {
                return Err(MockI2cError(err));
            }
            for op in operations {
                match op {
                    Operation::Read(buf) => {
                        self.read_calls += 1;
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

    #[test]
    fn test_m200_probe_read_phase_and_arbitration_retry() {
        let mut adapter = EmbeddedHalSccb::new(MockI2c::new(0x5A), MockDelay);

        // Portable probe uses a 1-byte read, not a zero-length write.
        assert_eq!(adapter.probe(0x30), Ok(()));
        assert_eq!(adapter.i2c().read_calls, 1);

        // Register-address write NACK reports SccbPhase::RegAddr.
        adapter.i2c_mut().err = Some(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Data));
        assert_eq!(
            adapter.read_reg8(0x30, 0x10),
            Err(CameraError::SccbNack {
                slv_addr: 0x30,
                phase: SccbPhase::RegAddr,
            })
        );
        assert_eq!(
            adapter.read_reg16(0x3C, 0x300A),
            Err(CameraError::SccbNack {
                slv_addr: 0x3C,
                phase: SccbPhase::RegAddr,
            })
        );

        // Transient ArbitrationLoss recovers on retry for read and probe.
        adapter.i2c_mut().err = None;
        adapter.i2c_mut().fail_once_err = Some(ErrorKind::ArbitrationLoss);
        assert_eq!(adapter.probe(0x30), Ok(()));
        adapter.i2c_mut().fail_once_err = Some(ErrorKind::ArbitrationLoss);
        assert_eq!(adapter.read_reg8(0x30, 0x10), Ok(0x5A));
        adapter.i2c_mut().fail_once_err = Some(ErrorKind::ArbitrationLoss);
        assert_eq!(adapter.read_reg16(0x3C, 0x300A), Ok(0x5A));
    }
}
