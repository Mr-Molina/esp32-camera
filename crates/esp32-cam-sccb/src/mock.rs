//! Deterministic `#![no_std]` mock SCCB bus for unit testing sensor drivers.

use core::fmt;
use esp32_cam_types::CameraError;

use crate::bus::{is_valid_sccb_addr, SccbBus};

/// Recorded transaction entry in [`MockSccbBus`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SccbTransaction {
    Read8 { slv_addr: u8, reg: u8, val: u8 },
    Write8 { slv_addr: u8, reg: u8, val: u8 },
    Read16 { slv_addr: u8, reg: u16, val: u8 },
    Write16 { slv_addr: u8, reg: u16, val: u8 },
    DelayMs(u32),
}

/// Alias for [`SccbTransaction`].
pub type SccbLogEntry = SccbTransaction;

/// Alias for [`SccbTransaction`].
pub type SccbOp = SccbTransaction;

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
    /// Sensor/slave device did not acknowledge address or command (NACK).
    AckMissing,
}

impl fmt::Display for SccbError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::AckMissing => write!(f, "SCCB acknowledge missing (NACK)"),
        }
    }
}

impl core::error::Error for SccbError {}

/// Deterministic, zero-heap mock SCCB bus for sensor driver testing.
#[derive(Debug, Clone)]
pub struct MockSccbBus<const CAP: usize = MAX_MOCK_REGS> {
    regs: [RegEntry; CAP],
    reg_len: usize,
    tx_log: [SccbTransaction; CAP],
    tx_len: usize,
    total_delay_ms: u32,
    forced_error: Option<CameraError>,
    strict_ack: bool,
    device_mask: [u64; 2],
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
            device_mask: [0; 2],
        }
    }

    const fn set_device_bit(&mut self, addr: u8) {
        if is_valid_sccb_addr(addr) {
            if addr < 64 {
                self.device_mask[0] |= 1u64 << addr;
            } else {
                self.device_mask[1] |= 1u64 << (addr - 64);
            }
        }
    }

    /// Configures mock bus to return `err` on all subsequent operations.
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

    /// Sets strict ACK verification mode on `probe` and read/write ops.
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

    /// Registers a 7-bit slave device address as active on the mock bus.
    pub fn register_device_addr(&mut self, addr: u8) {
        if is_valid_sccb_addr(addr) {
            self.set_device_bit(addr);
        }
    }

    /// Checks if a register or device registration exists for `slv_addr` in
    /// O(1) time.
    pub const fn has_device(&self, slv_addr: u8) -> bool {
        if !is_valid_sccb_addr(slv_addr) {
            return false;
        }
        if slv_addr < 64 {
            (self.device_mask[0] & (1u64 << slv_addr)) != 0
        } else {
            (self.device_mask[1] & (1u64 << (slv_addr - 64))) != 0
        }
    }

    /// Probes device at `slv_addr`, returning detailed [`SccbError`].
    pub fn probe_strict(&mut self, slv_addr: u8) -> Result<(), SccbError> {
        if !is_valid_sccb_addr(slv_addr)
            || !self.has_device(slv_addr)
            || self.forced_error.is_some()
        {
            return Err(SccbError::AckMissing);
        }
        Ok(())
    }

    /// Pre-populates an 8-bit register value without recording transactions.
    pub fn preset_reg8(&mut self, slv_addr: u8, reg: u8, val: u8) {
        let _ = self.store_reg(slv_addr, reg as u16, val);
    }

    /// Pre-populates a 16-bit register value without recording transactions.
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

    /// Inspects current value of an 8-bit register (`0` if never written).
    pub fn reg8(&self, slv_addr: u8, reg: u8) -> u8 {
        self.peek_reg8(slv_addr, reg).unwrap_or(0)
    }

    /// Inspects current value of a 16-bit register (`0` if never written).
    pub fn reg16(&self, slv_addr: u8, reg: u16) -> u8 {
        self.peek_reg16(slv_addr, reg).unwrap_or(0)
    }

    /// Inspects an 8-bit register, returning `Some(val)` if present in store.
    pub fn peek_reg8(&self, slv_addr: u8, reg: u8) -> Option<u8> {
        self.lookup_reg(slv_addr, reg as u16)
    }

    /// Inspects a 16-bit register, returning `Some(val)` if present in store.
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
            return Err(CameraError::NoMem);
        }
        let mut i = 0;
        while i < self.reg_len {
            if self.regs[i].occupied && self.regs[i].slv_addr == slv_addr && self.regs[i].reg == reg
            {
                self.regs[i].val = val;
                self.set_device_bit(slv_addr);
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
            self.set_device_bit(slv_addr);
            Ok(())
        } else {
            Err(CameraError::NoMem)
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
        if !is_valid_sccb_addr(slv_addr) {
            return Err(CameraError::InvalidArg);
        }
        if self.strict_ack && !self.has_device(slv_addr) {
            return Err(CameraError::SccbNack {
                slv_addr,
                phase: "device_addr",
            });
        }
        if let Some(err) = self.forced_error {
            return Err(err);
        }
        let val = self.lookup_reg(slv_addr, reg as u16).unwrap_or(0);
        self.push_tx(SccbTransaction::Read8 { slv_addr, reg, val });
        Ok(val)
    }

    fn write_reg8(&mut self, slv_addr: u8, reg: u8, data: u8) -> Result<(), CameraError> {
        if !is_valid_sccb_addr(slv_addr) {
            return Err(CameraError::InvalidArg);
        }
        if self.strict_ack && !self.has_device(slv_addr) {
            return Err(CameraError::SccbNack {
                slv_addr,
                phase: "device_addr",
            });
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
        if !is_valid_sccb_addr(slv_addr) {
            return Err(CameraError::InvalidArg);
        }
        if self.strict_ack && !self.has_device(slv_addr) {
            return Err(CameraError::SccbNack {
                slv_addr,
                phase: "device_addr",
            });
        }
        if let Some(err) = self.forced_error {
            return Err(err);
        }
        let val = self.lookup_reg(slv_addr, reg).unwrap_or(0);
        self.push_tx(SccbTransaction::Read16 { slv_addr, reg, val });
        Ok(val)
    }

    fn write_reg16(&mut self, slv_addr: u8, reg: u16, data: u8) -> Result<(), CameraError> {
        if !is_valid_sccb_addr(slv_addr) {
            return Err(CameraError::InvalidArg);
        }
        if self.strict_ack && !self.has_device(slv_addr) {
            return Err(CameraError::SccbNack {
                slv_addr,
                phase: "device_addr",
            });
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
        if !is_valid_sccb_addr(slv_addr) {
            return Err(CameraError::InvalidArg);
        }
        if let Some(err) = self.forced_error {
            return Err(err);
        }
        if (self.strict_ack || self.device_mask[0] != 0 || self.device_mask[1] != 0)
            && !self.has_device(slv_addr)
        {
            return Err(CameraError::SccbNack {
                slv_addr,
                phase: "device_addr",
            });
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

        assert_eq!(
            bus.write_reg8(CameraSccbAddr::OV2640, 0x12, 0b1010_0000),
            Ok(())
        );
        assert_eq!(
            bus.write_reg8_mask(CameraSccbAddr::OV2640, 0x12, 0b0000_0101, 0b0000_1111),
            Ok(())
        );
        assert_eq!(bus.reg8(CameraSccbAddr::OV2640, 0x12), 0b1010_0101);
    }

    #[test]
    fn test_mock_sccb_reg16_and_val16() {
        let mut bus: MockSccbBus<64> = MockSccbBus::new();
        assert_eq!(
            bus.write_reg16(CameraSccbAddr::OV5640, 0x300A, 0x56),
            Ok(())
        );
        assert_eq!(
            bus.write_reg16(CameraSccbAddr::OV5640, 0x300B, 0x40),
            Ok(())
        );
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
        assert_eq!(bus.reg16(CameraSccbAddr::OV5640, 0x3820), 0x06);

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

        // 7-bit slave address validation: reserved (0x00..=0x07, 0x78..=0x7F)
        // and >= 0x80 must fail.
        assert_eq!(bus.probe(0x00), Err(CameraError::InvalidArg));
        assert_eq!(bus.probe(0x07), Err(CameraError::InvalidArg));
        assert_eq!(bus.probe(0x78), Err(CameraError::InvalidArg));
        assert_eq!(bus.probe(0x80), Err(CameraError::InvalidArg));
        assert_eq!(bus.probe(0xFF), Err(CameraError::InvalidArg));
        assert_eq!(bus.probe(0x30), Ok(()));

        assert_eq!(bus.read_reg8(0x00, 0x00), Err(CameraError::InvalidArg));
        assert_eq!(bus.read_reg8(0x07, 0x00), Err(CameraError::InvalidArg));
        assert_eq!(bus.read_reg8(0x78, 0x00), Err(CameraError::InvalidArg));
        assert_eq!(bus.read_reg8(0x80, 0x00), Err(CameraError::InvalidArg));
        assert_eq!(
            bus.write_reg8(0x00, 0x00, 0x12),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(
            bus.write_reg8(0x07, 0x00, 0x12),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(
            bus.write_reg8(0x78, 0x00, 0x12),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(
            bus.write_reg8(0x80, 0x00, 0x12),
            Err(CameraError::InvalidArg)
        );

        assert_eq!(bus.read_reg16(0x00, 0x3000), Err(CameraError::InvalidArg));
        assert_eq!(bus.read_reg16(0x07, 0x3000), Err(CameraError::InvalidArg));
        assert_eq!(bus.read_reg16(0x78, 0x3000), Err(CameraError::InvalidArg));
        assert_eq!(bus.read_reg16(0x80, 0x3000), Err(CameraError::InvalidArg));
        assert_eq!(
            bus.write_reg16(0x00, 0x3000, 0x12),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(
            bus.write_reg16(0x07, 0x3000, 0x12),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(
            bus.write_reg16(0x78, 0x3000, 0x12),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(
            bus.write_reg16(0x80, 0x3000, 0x12),
            Err(CameraError::InvalidArg)
        );

        // write_reg8_mask with mask == 0 and slv_addr == 0 must return
        // InvalidArg (SCCB-002).
        assert_eq!(
            bus.write_reg8_mask(0x00, 0x01, 0xAA, 0x00),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(
            bus.write_reg8_mask(0x07, 0x01, 0xAA, 0x00),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(
            bus.write_reg8_mask(0x78, 0x01, 0xAA, 0x00),
            Err(CameraError::InvalidArg)
        );

        // 16-bit register address checked_add overflow
        assert_eq!(
            bus.read_reg16_val16(CameraSccbAddr::OV5640, 0xFFFF),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(
            bus.write_reg16_val16(CameraSccbAddr::OV5640, 0xFFFF, 0x1234),
            Err(CameraError::InvalidArg)
        );

        // Capacity overflow returns Err(CameraError::NoMem) when reg_len == CAP
        assert_eq!(bus.write_reg8(0x30, 0x01, 0xAA), Ok(()));
        assert_eq!(bus.write_reg8(0x30, 0x02, 0xBB), Ok(()));
        assert_eq!(bus.write_reg8(0x30, 0x03, 0xCC), Err(CameraError::NoMem));
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

        // Unpopulated slave address returns SccbNack in strict mode
        assert_eq!(
            bus.probe(CameraSccbAddr::OV7725),
            Err(CameraError::SccbNack {
                slv_addr: CameraSccbAddr::OV7725,
                phase: "device_addr",
            })
        );
        assert_eq!(
            bus.probe_strict(CameraSccbAddr::OV7725),
            Err(SccbError::AckMissing)
        );

        // After registering device address, probe succeeds
        bus.register_device_addr(CameraSccbAddr::OV7725);
        assert_eq!(bus.probe(CameraSccbAddr::OV7725), Ok(()));
        assert_eq!(bus.probe_strict(CameraSccbAddr::OV7725), Ok(()));

        // With strict_ack disabled, unpopulated probe succeeds for compat
        let mut loose_bus: MockSccbBus<64> = MockSccbBus::new();
        assert_eq!(loose_bus.probe(CameraSccbAddr::OV7725), Ok(()));
    }

    #[test]
    fn test_mock_sccb_strict_ack_rejects_unregistered_rw() {
        let mut bus: MockSccbBus<64> = MockSccbBus::new();
        bus.set_strict_ack(true);

        assert_eq!(
            bus.read_reg8(0x30, 0x0A),
            Err(CameraError::SccbNack {
                slv_addr: 0x30,
                phase: "device_addr",
            })
        );
        assert_eq!(
            bus.write_reg8(0x30, 0x0A, 0x12),
            Err(CameraError::SccbNack {
                slv_addr: 0x30,
                phase: "device_addr",
            })
        );
        assert_eq!(
            bus.read_reg16(0x3C, 0x300A),
            Err(CameraError::SccbNack {
                slv_addr: 0x3C,
                phase: "device_addr",
            })
        );
        assert_eq!(
            bus.write_reg16(0x3C, 0x300A, 0x56),
            Err(CameraError::SccbNack {
                slv_addr: 0x3C,
                phase: "device_addr",
            })
        );

        // After registering device address, reads and writes succeed
        bus.register_device_addr(0x30);
        assert_eq!(bus.write_reg8(0x30, 0x0A, 0x26), Ok(()));
        assert_eq!(bus.read_reg8(0x30, 0x0A), Ok(0x26));
    }
}
