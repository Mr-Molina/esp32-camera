//! Safe `#![no_std]` SCCB (Serial Camera Control Bus) abstraction, `embedded-hal` 1.0 adapter,
//! and zero-heap [`MockSccbBus`] for deterministic host and target sensor testing.

#![no_std]
#![forbid(unsafe_code)]

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;
use esp32_cam_types::CameraError;

/// Hardware-agnostic SCCB / I2C register bus trait for camera sensors.
///
/// Supports both 8-bit register addressing (e.g. `OV2640`, `OV7725`, `GC0308`, `SC030IOT`)
/// and 16-bit big-endian register addressing (e.g. `OV5640`, `OV3660`, `OV3640`, `NT99141`,
/// `HM1055`, `HM0360`, `MEGA_CCM`).
pub trait SccbBus {
    /// Reads an 8-bit value from an 8-bit register address on `slv_addr` (`SCCB_Read`).
    fn read_reg8(&mut self, slv_addr: u8, reg: u8) -> Result<u8, CameraError>;

    /// Writes an 8-bit value to an 8-bit register address on `slv_addr` (`SCCB_Write`).
    fn write_reg8(&mut self, slv_addr: u8, reg: u8, data: u8) -> Result<(), CameraError>;

    /// Performs a read-modify-write on an 8-bit register using `mask`.
    fn write_reg8_mask(
        &mut self,
        slv_addr: u8,
        reg: u8,
        val: u8,
        mask: u8,
    ) -> Result<(), CameraError> {
        let current = self.read_reg8(slv_addr, reg)?;
        let next = (current & !mask) | (val & mask);
        self.write_reg8(slv_addr, reg, next)
    }

    /// Reads an 8-bit value from a 16-bit register address on `slv_addr` (`SCCB_Read16`).
    fn read_reg16(&mut self, slv_addr: u8, reg: u16) -> Result<u8, CameraError>;

    /// Writes an 8-bit value to a 16-bit register address on `slv_addr` (`SCCB_Write16`).
    fn write_reg16(&mut self, slv_addr: u8, reg: u16, data: u8) -> Result<(), CameraError>;

    /// Performs a read-modify-write on a 16-bit register using `mask`.
    fn write_reg16_mask(
        &mut self,
        slv_addr: u8,
        reg: u16,
        val: u8,
        mask: u8,
    ) -> Result<(), CameraError> {
        let current = self.read_reg16(slv_addr, reg)?;
        let next = (current & !mask) | (val & mask);
        self.write_reg16(slv_addr, reg, next)
    }

    /// Reads a 16-bit big-endian value from a 16-bit register address (`SCCB_Read_Addr16_Val16`).
    fn read_reg16_val16(&mut self, slv_addr: u8, reg: u16) -> Result<u16, CameraError> {
        if slv_addr == 0 || slv_addr >= 0x80 {
            return Err(CameraError::InvalidArg);
        }
        let next_reg = reg.checked_add(1).ok_or(CameraError::InvalidArg)?;
        let hi = self.read_reg16(slv_addr, reg)? as u16;
        let lo = self.read_reg16(slv_addr, next_reg)? as u16;
        Ok((hi << 8) | lo)
    }

    /// Writes a 16-bit big-endian value to a 16-bit register address (`SCCB_Write_Addr16_Val16`).
    fn write_reg16_val16(&mut self, slv_addr: u8, reg: u16, data: u16) -> Result<(), CameraError> {
        if slv_addr == 0 || slv_addr >= 0x80 {
            return Err(CameraError::InvalidArg);
        }
        let next_reg = reg.checked_add(1).ok_or(CameraError::InvalidArg)?;
        self.write_reg16(slv_addr, reg, (data >> 8) as u8)?;
        self.write_reg16(slv_addr, next_reg, (data & 0xFF) as u8)
    }

    /// Probes whether a device responds at the 7-bit `slv_addr` (`SCCB_Probe`).
    fn probe(&mut self, slv_addr: u8) -> Result<(), CameraError> {
        if slv_addr == 0 || slv_addr >= 0x80 {
            return Err(CameraError::InvalidArg);
        }
        let _ = self.read_reg8(slv_addr, 0x00)?;
        Ok(())
    }

    /// Delays execution for `ms` milliseconds.
    fn delay_ms(&mut self, ms: u32);
}

/// `embedded-hal` 1.0 adapter implementing [`SccbBus`] over any `I2c` and `DelayNs` peripheral.
///
/// Follows the two-phase SCCB read protocol from `driver/sccb.c` (separate write of register
/// address followed by a STOP condition and a subsequent read transaction).
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
            .map_err(|_| CameraError::SccbError)?;
        self.i2c
            .read(slv_addr, &mut data)
            .map_err(|_| CameraError::SccbError)?;
        Ok(data[0])
    }

    fn write_reg8(&mut self, slv_addr: u8, reg: u8, data: u8) -> Result<(), CameraError> {
        if slv_addr == 0 || slv_addr >= 0x80 {
            return Err(CameraError::InvalidArg);
        }
        self.i2c
            .write(slv_addr, &[reg, data])
            .map_err(|_| CameraError::SccbError)
    }

    fn read_reg16(&mut self, slv_addr: u8, reg: u16) -> Result<u8, CameraError> {
        if slv_addr == 0 || slv_addr >= 0x80 {
            return Err(CameraError::InvalidArg);
        }
        let reg_bytes = reg.to_be_bytes();
        let mut data = [0u8; 1];
        self.i2c
            .write(slv_addr, &reg_bytes)
            .map_err(|_| CameraError::SccbError)?;
        self.i2c
            .read(slv_addr, &mut data)
            .map_err(|_| CameraError::SccbError)?;
        Ok(data[0])
    }

    fn write_reg16(&mut self, slv_addr: u8, reg: u16, data: u8) -> Result<(), CameraError> {
        if slv_addr == 0 || slv_addr >= 0x80 {
            return Err(CameraError::InvalidArg);
        }
        let reg_bytes = reg.to_be_bytes();
        self.i2c
            .write(slv_addr, &[reg_bytes[0], reg_bytes[1], data])
            .map_err(|_| CameraError::SccbError)
    }

    fn read_reg16_val16(&mut self, slv_addr: u8, reg: u16) -> Result<u16, CameraError> {
        if slv_addr == 0 || slv_addr >= 0x80 {
            return Err(CameraError::InvalidArg);
        }
        let reg_bytes = reg.to_be_bytes();
        let mut data = [0u8; 2];
        self.i2c
            .write(slv_addr, &reg_bytes)
            .map_err(|_| CameraError::SccbError)?;
        self.i2c
            .read(slv_addr, &mut data)
            .map_err(|_| CameraError::SccbError)?;
        Ok(u16::from_be_bytes(data))
    }

    fn write_reg16_val16(&mut self, slv_addr: u8, reg: u16, data: u16) -> Result<(), CameraError> {
        if slv_addr == 0 || slv_addr >= 0x80 {
            return Err(CameraError::InvalidArg);
        }
        let reg_bytes = reg.to_be_bytes();
        let data_bytes = data.to_be_bytes();
        self.i2c
            .write(
                slv_addr,
                &[reg_bytes[0], reg_bytes[1], data_bytes[0], data_bytes[1]],
            )
            .map_err(|_| CameraError::SccbError)
    }

    fn probe(&mut self, slv_addr: u8) -> Result<(), CameraError> {
        if slv_addr == 0 || slv_addr >= 0x80 {
            return Err(CameraError::InvalidArg);
        }
        self.i2c
            .write(slv_addr, &[])
            .map_err(|_| CameraError::SccbError)
    }

    fn delay_ms(&mut self, ms: u32) {
        self.delay.delay_ms(ms);
    }
}

/// Recorded transaction entry in [`MockSccbBus`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SccbTransaction {
    Read8 { slv_addr: u8, reg: u8, val: u8 },
    Write8 { slv_addr: u8, reg: u8, val: u8 },
    Read16 { slv_addr: u8, reg: u16, val: u8 },
    Write16 { slv_addr: u8, reg: u16, val: u8 },
    DelayMs(u32),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct RegEntry {
    slv_addr: u8,
    reg: u16,
    val: u8,
    occupied: bool,
}

/// Maximum mock register store capacity for [`MockSccbBus`].
pub const MAX_MOCK_REGS: usize = 1024;

/// Detailed SCCB / I2C bus error conditions.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SccbError {
    /// Sensor/slave device did not acknowledge address or command (NACK / missing ACK).
    AckMissing,
}

impl core::fmt::Display for SccbError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::AckMissing => write!(f, "SCCB acknowledge missing (NACK)"),
        }
    }
}

impl core::error::Error for SccbError {}

/// Deterministic, zero-heap `#![no_std]` mock SCCB bus for unit testing sensor drivers.
#[derive(Debug, Clone)]
pub struct MockSccbBus<const CAP: usize = MAX_MOCK_REGS> {
    regs: [RegEntry; CAP],
    reg_len: usize,
    tx_log: [SccbTransaction; CAP],
    tx_len: usize,
    total_delay_ms: u32,
    forced_error: Option<CameraError>,
    strict_ack: bool,
    device_addrs: [u8; 16],
    device_addrs_len: usize,
}

impl<const CAP: usize> Default for MockSccbBus<CAP> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const CAP: usize> MockSccbBus<CAP> {
    /// Creates an empty [`MockSccbBus`].
    pub const fn new() -> Self {
        Self {
            regs: [RegEntry {
                slv_addr: 0,
                reg: 0,
                val: 0,
                occupied: false,
            }; CAP],
            reg_len: 0,
            tx_log: [SccbTransaction::DelayMs(0); CAP],
            tx_len: 0,
            total_delay_ms: 0,
            forced_error: None,
            strict_ack: false,
            device_addrs: [0; 16],
            device_addrs_len: 0,
        }
    }

    /// Configures the mock bus to return `err` on all subsequent bus operations, or clears the error if `None`.
    pub fn set_error(&mut self, err: Option<CameraError>) {
        self.forced_error = err;
    }

    /// Convenience helper to toggle simulated SCCB bus failure.
    pub fn set_fail(&mut self, fail: bool) {
        self.forced_error = if fail {
            Some(CameraError::SccbError)
        } else {
            None
        };
    }

    /// Sets strict ACK verification mode on `probe`.
    ///
    /// When `strict_ack` is enabled (or when [`Self::register_device_addr`] has been used),
    /// [`SccbBus::probe`] will return [`CameraError::SccbError`] if no registers or registered
    /// device addresses exist for `addr`.
    pub fn set_strict_ack(&mut self, strict: bool) {
        self.strict_ack = strict;
    }

    /// Builder method to enable or disable strict ACK mode.
    pub const fn with_strict_ack(mut self, strict: bool) -> Self {
        self.strict_ack = strict;
        self
    }

    /// Returns whether strict ACK mode is active.
    pub const fn strict_ack(&self) -> bool {
        self.strict_ack
    }

    /// Explicitly registers a 7-bit slave device address as active on the mock bus.
    pub fn register_device_addr(&mut self, addr: u8) {
        if addr > 0 && addr < 0x80 {
            let mut i = 0;
            while i < self.device_addrs_len {
                if self.device_addrs[i] == addr {
                    return;
                }
                i += 1;
            }
            if self.device_addrs_len < self.device_addrs.len() {
                self.device_addrs[self.device_addrs_len] = addr;
                self.device_addrs_len += 1;
            }
        }
    }

    /// Checks if any register or explicit device registration exists for `slv_addr`.
    pub fn has_device(&self, slv_addr: u8) -> bool {
        let mut i = 0;
        while i < self.device_addrs_len {
            if self.device_addrs[i] == slv_addr {
                return true;
            }
            i += 1;
        }
        let mut i = 0;
        while i < self.reg_len {
            let entry = &self.regs[i];
            if entry.occupied && entry.slv_addr == slv_addr {
                return true;
            }
            i += 1;
        }
        false
    }

    /// Probes whether a device responds at `slv_addr` returning a detailed [`SccbError`].
    pub fn probe_strict(&mut self, slv_addr: u8) -> Result<(), SccbError> {
        if slv_addr == 0 || slv_addr >= 0x80 || !self.has_device(slv_addr) || self.forced_error.is_some() {
            return Err(SccbError::AckMissing);
        }
        Ok(())
    }

    /// Pre-populates an 8-bit register value without recording a bus transaction.
    pub fn preset_reg8(&mut self, slv_addr: u8, reg: u8, val: u8) {
        let _ = self.store_reg(slv_addr, reg as u16, val);
    }

    /// Pre-populates a 16-bit register value without recording a bus transaction.
    pub fn preset_reg16(&mut self, slv_addr: u8, reg: u16, val: u8) {
        let _ = self.store_reg(slv_addr, reg, val);
    }

    /// Alias for [`Self::preset_reg8`].
    pub fn set_reg8(&mut self, slv_addr: u8, reg: u8, val: u8) {
        let _ = self.store_reg(slv_addr, reg as u16, val);
    }

    /// Alias for [`Self::preset_reg16`].
    pub fn set_reg16(&mut self, slv_addr: u8, reg: u16, val: u8) {
        let _ = self.store_reg(slv_addr, reg, val);
    }

    /// Inspects the current value of an 8-bit register (`0` if never written).
    pub fn get_reg8(&self, slv_addr: u8, reg: u8) -> u8 {
        self.peek_reg8(slv_addr, reg).unwrap_or(0)
    }

    /// Inspects the current value of a 16-bit register (`0` if never written).
    pub fn get_reg16(&self, slv_addr: u8, reg: u16) -> u8 {
        self.peek_reg16(slv_addr, reg).unwrap_or(0)
    }

    /// Inspects an 8-bit register, returning `Some(val)` if present in the store.
    pub fn peek_reg8(&self, slv_addr: u8, reg: u8) -> Option<u8> {
        self.lookup_reg(slv_addr, reg as u16)
    }

    /// Inspects a 16-bit register, returning `Some(val)` if present in the store.
    pub fn peek_reg16(&self, slv_addr: u8, reg: u16) -> Option<u8> {
        self.lookup_reg(slv_addr, reg)
    }

    /// Returns the recorded slice of transactions (up to `CAP`).
    pub fn transactions(&self) -> &[SccbTransaction] {
        &self.tx_log[..self.tx_len]
    }

    /// Returns the number of recorded transactions in the log.
    pub const fn transaction_count(&self) -> usize {
        self.tx_len
    }

    /// Clears the recorded transaction log without clearing register state.
    pub fn clear_transactions(&mut self) {
        self.tx_len = 0;
    }

    /// Returns total accumulated delay in milliseconds passed to `delay_ms`.
    pub const fn total_delay_ms(&self) -> u32 {
        self.total_delay_ms
    }

    fn lookup_reg(&self, slv_addr: u8, reg: u16) -> Option<u8> {
        let mut i = 0;
        while i < self.reg_len {
            let entry = &self.regs[i];
            if entry.occupied && entry.slv_addr == slv_addr && entry.reg == reg {
                return Some(entry.val);
            }
            i += 1;
        }
        None
    }

    fn store_reg(&mut self, slv_addr: u8, reg: u16, val: u8) -> Result<(), CameraError> {
        if CAP == 0 {
            return Err(CameraError::SccbError);
        }
        let mut i = 0;
        while i < self.reg_len {
            if self.regs[i].occupied && self.regs[i].slv_addr == slv_addr && self.regs[i].reg == reg
            {
                self.regs[i].val = val;
                return Ok(());
            }
            i += 1;
        }
        if self.reg_len < CAP {
            self.regs[self.reg_len] = RegEntry {
                slv_addr,
                reg,
                val,
                occupied: true,
            };
            self.reg_len += 1;
            Ok(())
        } else {
            Err(CameraError::SccbError)
        }
    }

    fn push_tx(&mut self, tx: SccbTransaction) {
        if CAP == 0 {
            return;
        }
        if self.tx_len < CAP {
            self.tx_log[self.tx_len] = tx;
            self.tx_len += 1;
        } else {
            self.tx_log.copy_within(1..CAP, 0);
            self.tx_log[CAP - 1] = tx;
        }
    }
}

impl<const CAP: usize> SccbBus for MockSccbBus<CAP> {
    fn read_reg8(&mut self, slv_addr: u8, reg: u8) -> Result<u8, CameraError> {
        if slv_addr == 0 || slv_addr >= 0x80 {
            return Err(CameraError::InvalidArg);
        }
        if let Some(err) = self.forced_error {
            return Err(err);
        }
        let val = self.lookup_reg(slv_addr, reg as u16).unwrap_or(0);
        self.push_tx(SccbTransaction::Read8 { slv_addr, reg, val });
        Ok(val)
    }

    fn write_reg8(&mut self, slv_addr: u8, reg: u8, data: u8) -> Result<(), CameraError> {
        if slv_addr == 0 || slv_addr >= 0x80 {
            return Err(CameraError::InvalidArg);
        }
        if let Some(err) = self.forced_error {
            return Err(err);
        }
        self.store_reg(slv_addr, reg as u16, data)?;
        self.push_tx(SccbTransaction::Write8 {
            slv_addr,
            reg,
            val: data,
        });
        Ok(())
    }

    fn read_reg16(&mut self, slv_addr: u8, reg: u16) -> Result<u8, CameraError> {
        if slv_addr == 0 || slv_addr >= 0x80 {
            return Err(CameraError::InvalidArg);
        }
        if let Some(err) = self.forced_error {
            return Err(err);
        }
        let val = self.lookup_reg(slv_addr, reg).unwrap_or(0);
        self.push_tx(SccbTransaction::Read16 { slv_addr, reg, val });
        Ok(val)
    }

    fn write_reg16(&mut self, slv_addr: u8, reg: u16, data: u8) -> Result<(), CameraError> {
        if slv_addr == 0 || slv_addr >= 0x80 {
            return Err(CameraError::InvalidArg);
        }
        if let Some(err) = self.forced_error {
            return Err(err);
        }
        self.store_reg(slv_addr, reg, data)?;
        self.push_tx(SccbTransaction::Write16 {
            slv_addr,
            reg,
            val: data,
        });
        Ok(())
    }

    fn probe(&mut self, slv_addr: u8) -> Result<(), CameraError> {
        if slv_addr == 0 || slv_addr >= 0x80 {
            return Err(CameraError::InvalidArg);
        }
        if let Some(err) = self.forced_error {
            return Err(err);
        }
        if (self.strict_ack || self.device_addrs_len > 0) && !self.has_device(slv_addr) {
            return Err(CameraError::SccbError);
        }
        Ok(())
    }

    fn delay_ms(&mut self, ms: u32) {
        self.total_delay_ms = self.total_delay_ms.saturating_add(ms);
        self.push_tx(SccbTransaction::DelayMs(ms));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use esp32_cam_types::CameraSccbAddr;

    #[test]
    fn test_mock_sccb_reg8_and_mask() {
        let mut bus: MockSccbBus<64> = MockSccbBus::new();
        bus.preset_reg8(CameraSccbAddr::OV2640, 0x0A, 0x26);
        assert_eq!(bus.read_reg8(CameraSccbAddr::OV2640, 0x0A), Ok(0x26));

        assert_eq!(bus.write_reg8(CameraSccbAddr::OV2640, 0x12, 0b1010_0000), Ok(()));
        assert_eq!(
            bus.write_reg8_mask(CameraSccbAddr::OV2640, 0x12, 0b0000_0101, 0b0000_1111),
            Ok(())
        );
        assert_eq!(bus.get_reg8(CameraSccbAddr::OV2640, 0x12), 0b1010_0101);
    }

    #[test]
    fn test_mock_sccb_reg16_and_val16() {
        let mut bus: MockSccbBus<64> = MockSccbBus::new();
        assert_eq!(bus.write_reg16(CameraSccbAddr::OV5640, 0x300A, 0x56), Ok(()));
        assert_eq!(bus.write_reg16(CameraSccbAddr::OV5640, 0x300B, 0x40), Ok(()));
        assert_eq!(bus.read_reg16(CameraSccbAddr::OV5640, 0x300A), Ok(0x56));
        assert_eq!(bus.read_reg16(CameraSccbAddr::OV5640, 0x300B), Ok(0x40));
        assert_eq!(
            bus.read_reg16_val16(CameraSccbAddr::OV5640, 0x300A),
            Ok(0x5640)
        );

        assert_eq!(
            bus.write_reg16_mask(CameraSccbAddr::OV5640, 0x3820, 0x06, 0x06),
            Ok(())
        );
        assert_eq!(bus.get_reg16(CameraSccbAddr::OV5640, 0x3820), 0x06);

        bus.delay_ms(10);
        bus.delay_ms(25);
        assert_eq!(bus.total_delay_ms(), 35);
    }

    #[test]
    fn test_mock_sccb_error_propagation() {
        let mut bus: MockSccbBus<16> = MockSccbBus::new();
        bus.set_fail(true);
        assert_eq!(
            bus.read_reg8(CameraSccbAddr::OV2640, 0x0A),
            Err(CameraError::SccbError)
        );
        assert_eq!(
            bus.write_reg16(CameraSccbAddr::OV5640, 0x3008, 0x82),
            Err(CameraError::SccbError)
        );
        bus.set_fail(false);
        assert_eq!(bus.write_reg8(CameraSccbAddr::OV2640, 0xFF, 0x01), Ok(()));
    }

    #[test]
    fn test_mock_sccb_address_validation_and_overflow() {
        let mut bus: MockSccbBus<2> = MockSccbBus::new();

        // 7-bit slave address validation: 0 and >= 0x80 must fail
        assert_eq!(bus.probe(0x00), Err(CameraError::InvalidArg));
        assert_eq!(bus.probe(0x80), Err(CameraError::InvalidArg));
        assert_eq!(bus.probe(0xFF), Err(CameraError::InvalidArg));
        assert_eq!(bus.probe(0x30), Ok(()));

        assert_eq!(bus.read_reg8(0x00, 0x00), Err(CameraError::InvalidArg));
        assert_eq!(bus.read_reg8(0x80, 0x00), Err(CameraError::InvalidArg));
        assert_eq!(bus.write_reg8(0x00, 0x00, 0x12), Err(CameraError::InvalidArg));
        assert_eq!(bus.write_reg8(0x80, 0x00, 0x12), Err(CameraError::InvalidArg));

        assert_eq!(bus.read_reg16(0x00, 0x3000), Err(CameraError::InvalidArg));
        assert_eq!(bus.read_reg16(0x80, 0x3000), Err(CameraError::InvalidArg));
        assert_eq!(bus.write_reg16(0x00, 0x3000, 0x12), Err(CameraError::InvalidArg));
        assert_eq!(bus.write_reg16(0x80, 0x3000, 0x12), Err(CameraError::InvalidArg));

        // 16-bit register address checked_add overflow
        assert_eq!(
            bus.read_reg16_val16(CameraSccbAddr::OV5640, 0xFFFF),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(
            bus.write_reg16_val16(CameraSccbAddr::OV5640, 0xFFFF, 0x1234),
            Err(CameraError::InvalidArg)
        );

        // Capacity overflow returns Err(CameraError::SccbError) when full
        assert_eq!(bus.write_reg8(0x30, 0x01, 0xAA), Ok(()));
        assert_eq!(bus.write_reg8(0x30, 0x02, 0xBB), Ok(()));
        assert_eq!(bus.write_reg8(0x30, 0x03, 0xCC), Err(CameraError::SccbError));
        // Overwriting existing slot succeeds
        assert_eq!(bus.write_reg8(0x30, 0x01, 0xFF), Ok(()));
        assert_eq!(bus.read_reg8(0x30, 0x01), Ok(0xFF));
    }

    #[test]
    fn test_mock_sccb_strict_ack_missing() {
        let mut bus: MockSccbBus<64> = MockSccbBus::new();
        bus.set_strict_ack(true);
        bus.preset_reg8(CameraSccbAddr::OV2640, 0x0A, 0x26);

        // Populated address succeeds probe
        assert_eq!(bus.probe(CameraSccbAddr::OV2640), Ok(()));

        // Unpopulated slave address returns SccbError in strict mode
        assert_eq!(bus.probe(CameraSccbAddr::OV7725), Err(CameraError::SccbError));
        assert_eq!(
            bus.probe_strict(CameraSccbAddr::OV7725),
            Err(SccbError::AckMissing)
        );

        // After registering device address, probe succeeds
        bus.register_device_addr(CameraSccbAddr::OV7725);
        assert_eq!(bus.probe(CameraSccbAddr::OV7725), Ok(()));
        assert_eq!(bus.probe_strict(CameraSccbAddr::OV7725), Ok(()));

        // With strict_ack disabled and no registered devices, unpopulated probe succeeds for backward compat
        let mut loose_bus: MockSccbBus<64> = MockSccbBus::new();
        assert_eq!(loose_bus.probe(CameraSccbAddr::OV7725), Ok(()));
    }
}
