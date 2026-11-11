//! Deterministic `#![no_std]` mock SCCB bus for unit testing sensor drivers.

use core::fmt;

use esp32_cam_types::{CameraError, SccbPhase};

use crate::bus::{is_valid_sccb_addr, SccbBus};

/// Register address width in [`MockSccbBus`].
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum RegWidth {
    /// 8-bit register address space (`read_reg8` / `write_reg8`).
    #[default]
    Bits8,
    /// 16-bit register address space (`read_reg16` / `write_reg16`).
    Bits16,
}

/// Read policy when an unpopulated register is read from [`MockSccbBus`].
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum UnsetReadPolicy {
    /// Return `0` when reading an unset register (default compatibility mode).
    #[default]
    Zero,
    /// Return `Err(CameraError::SccbError)` when reading an unset register.
    Error,
}

/// Recorded transaction entry in [`MockSccbBus`].
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SccbTransaction {
    /// 8-bit register read (`read_reg8`).
    Read8 { slv_addr: u8, reg: u8, val: u8 },
    /// Single 8-bit register write (`write_reg8`).
    Write8 { slv_addr: u8, reg: u8, val: u8 },
    /// Sequential 8-bit burst write (`write_reg8_seq`).
    Write8Seq {
        slv_addr: u8,
        start_reg: u8,
        len: u16,
    },
    /// 16-bit register read (`read_reg16`).
    Read16 { slv_addr: u8, reg: u16, val: u8 },
    /// Single 16-bit register write (`write_reg16`).
    Write16 { slv_addr: u8, reg: u16, val: u8 },
    /// Sequential 16-bit burst write (`write_reg16_seq`).
    Write16Seq {
        slv_addr: u8,
        start_reg: u16,
        len: u16,
    },
    /// Millisecond delay (`delay_ms`).
    DelayMs(u32),
    /// Failed bus operation (`SCCB-05`).
    Failed {
        slv_addr: u8,
        reg: u16,
        err: CameraError,
    },
}

/// Alias for [`SccbTransaction`].
pub type SccbLogEntry = SccbTransaction;

/// Alias for [`SccbTransaction`].
pub type SccbOp = SccbTransaction;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct RegEntry {
    slv_addr: u8,
    width: RegWidth,
    bank: u8,
    reg: u16,
    val: u8,
    occupied: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct BankTracker {
    slv_addr: u8,
    bank_reg: u8,
    active_bank: u8,
    enabled: bool,
}

/// Maximum mock register store capacity for [`MockSccbBus`].
pub const MAX_MOCK_REGS: usize = 1024;

const MAX_BANK_TRACKERS: usize = 4;

/// Detailed SCCB / I2C bus error conditions.
#[non_exhaustive]
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
    dropped_tx_count: usize,
    total_reg_writes: usize,
    total_delay_ms: u32,
    forced_error: Option<CameraError>,
    ops_until_fail: Option<usize>,
    fail_at_err: CameraError,
    strict_ack: bool,
    unset_read_policy: UnsetReadPolicy,
    device_mask: [u64; 2],
    bank_trackers: [BankTracker; MAX_BANK_TRACKERS],
}

impl<const CAP: usize> Default for MockSccbBus<CAP> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const CAP: usize> MockSccbBus<CAP> {
    /// Creates an empty [`MockSccbBus`].
    #[must_use]
    pub const fn new() -> Self {
        Self {
            regs: [RegEntry {
                slv_addr: 0,
                width: RegWidth::Bits8,
                bank: 0,
                reg: 0,
                val: 0,
                occupied: false,
            }; CAP],
            reg_len: 0,
            tx_log: [SccbTransaction::DelayMs(0); CAP],
            tx_len: 0,
            dropped_tx_count: 0,
            total_reg_writes: 0,
            total_delay_ms: 0,
            forced_error: None,
            ops_until_fail: None,
            fail_at_err: CameraError::SccbError,
            strict_ack: false,
            unset_read_policy: UnsetReadPolicy::Zero,
            device_mask: [0; 2],
            bank_trackers: [BankTracker {
                slv_addr: 0,
                bank_reg: 0xFF,
                active_bank: 0,
                enabled: false,
            }; MAX_BANK_TRACKERS],
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

    /// Arms a one-shot [`CameraError::SccbError`] on the `nth_op`-th bus
    /// operation (`1`-indexed; `0` clears pending fault injection).
    pub fn fail_at(&mut self, nth_op: usize) {
        self.fail_at_with(nth_op, CameraError::SccbError);
    }

    /// Arms a one-shot `err` on the `nth_op`-th bus operation (`1`-indexed;
    /// `0` clears pending fault injection).
    pub fn fail_at_with(&mut self, nth_op: usize, err: CameraError) {
        if nth_op == 0 {
            self.ops_until_fail = None;
        } else {
            self.ops_until_fail = Some(nth_op);
            self.fail_at_err = err;
        }
    }

    /// Sets strict ACK verification mode on `probe` and read/write ops.
    pub fn set_strict_ack(&mut self, strict: bool) {
        self.strict_ack = strict;
    }

    /// Builder method to enable or disable strict ACK mode.
    #[must_use]
    pub const fn with_strict_ack(mut self, strict: bool) -> Self {
        self.strict_ack = strict;
        self
    }

    /// Returns whether strict ACK mode is active.
    #[must_use]
    pub const fn strict_ack(&self) -> bool {
        self.strict_ack
    }

    /// Sets the read policy when reading an unset register (`SCCB-06`).
    pub fn set_unset_read_policy(&mut self, policy: UnsetReadPolicy) {
        self.unset_read_policy = policy;
    }

    /// Builder method to configure [`UnsetReadPolicy`].
    #[must_use]
    pub const fn with_unset_read_policy(mut self, policy: UnsetReadPolicy) -> Self {
        self.unset_read_policy = policy;
        self
    }

    /// Returns the active [`UnsetReadPolicy`].
    #[must_use]
    pub const fn unset_read_policy(&self) -> UnsetReadPolicy {
        self.unset_read_policy
    }

    /// Enables register-bank tracking for an 8-bit sensor at `slv_addr` using
    /// `bank_reg` (for example `0xFF` on `OV2640`).
    pub fn enable_bank_select(&mut self, slv_addr: u8, bank_reg: u8) {
        let mut idx = 0;
        while idx < MAX_BANK_TRACKERS {
            if self.bank_trackers[idx].enabled && self.bank_trackers[idx].slv_addr == slv_addr {
                self.bank_trackers[idx].bank_reg = bank_reg;
                return;
            }
            idx += 1;
        }
        let mut slot = 0;
        while slot < MAX_BANK_TRACKERS {
            if !self.bank_trackers[slot].enabled {
                self.bank_trackers[slot] = BankTracker {
                    slv_addr,
                    bank_reg,
                    active_bank: 0,
                    enabled: true,
                };
                return;
            }
            slot += 1;
        }
    }

    /// Builder method to enable register-bank tracking on `slv_addr`.
    #[must_use]
    pub const fn with_bank_select(mut self, slv_addr: u8, bank_reg: u8) -> Self {
        let mut idx = 0;
        while idx < MAX_BANK_TRACKERS {
            if self.bank_trackers[idx].enabled && self.bank_trackers[idx].slv_addr == slv_addr {
                self.bank_trackers[idx].bank_reg = bank_reg;
                return self;
            }
            idx += 1;
        }
        let mut slot = 0;
        while slot < MAX_BANK_TRACKERS {
            if !self.bank_trackers[slot].enabled {
                self.bank_trackers[slot] = BankTracker {
                    slv_addr,
                    bank_reg,
                    active_bank: 0,
                    enabled: true,
                };
                return self;
            }
            slot += 1;
        }
        self
    }

    /// Returns the currently selected bank for `slv_addr` (`0` if unbanked).
    #[must_use]
    pub const fn current_bank(&self, slv_addr: u8) -> u8 {
        let mut idx = 0;
        while idx < MAX_BANK_TRACKERS {
            if self.bank_trackers[idx].enabled && self.bank_trackers[idx].slv_addr == slv_addr {
                return self.bank_trackers[idx].active_bank;
            }
            idx += 1;
        }
        0
    }

    /// Registers a 7-bit slave device address as active on the mock bus.
    pub fn register_device_addr(&mut self, addr: u8) {
        if is_valid_sccb_addr(addr) {
            self.set_device_bit(addr);
        }
    }

    /// Checks if a register or device registration exists for `slv_addr` in
    /// O(1) time.
    #[must_use]
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
        let _ = self.store_reg8_bank_aware(slv_addr, reg, val);
    }

    /// Pre-populates an 8-bit register value in an explicit `bank` without
    /// recording transactions or changing [`Self::current_bank`].
    pub fn preset_reg8_banked(&mut self, slv_addr: u8, bank: u8, reg: u8, val: u8) {
        let _ = self.store_reg(slv_addr, RegWidth::Bits8, bank, u16::from(reg), val);
    }

    /// Pre-populates a 16-bit register value without recording transactions.
    pub fn preset_reg16(&mut self, slv_addr: u8, reg: u16, val: u8) {
        let _ = self.store_reg(slv_addr, RegWidth::Bits16, 0, reg, val);
    }

    /// Alias for [`Self::preset_reg8`].
    pub fn set_reg8(&mut self, slv_addr: u8, reg: u8, val: u8) {
        self.preset_reg8(slv_addr, reg, val);
    }

    /// Alias for [`Self::preset_reg16`].
    pub fn set_reg16(&mut self, slv_addr: u8, reg: u16, val: u8) {
        self.preset_reg16(slv_addr, reg, val);
    }

    /// Inspects current value of an 8-bit register in the active bank (`0` if
    /// never written).
    #[must_use]
    pub fn reg8(&self, slv_addr: u8, reg: u8) -> u8 {
        self.peek_reg8(slv_addr, reg).unwrap_or(0)
    }

    /// Inspects current value of an 8-bit register in `bank` (`0` if never
    /// written).
    #[must_use]
    pub fn reg8_banked(&self, slv_addr: u8, bank: u8, reg: u8) -> u8 {
        self.peek_reg8_banked(slv_addr, bank, reg).unwrap_or(0)
    }

    /// Inspects current value of a 16-bit register (`0` if never written).
    #[must_use]
    pub fn reg16(&self, slv_addr: u8, reg: u16) -> u8 {
        self.peek_reg16(slv_addr, reg).unwrap_or(0)
    }

    /// Inspects an 8-bit register in the active bank, returning `Some(val)` if
    /// present in store.
    #[must_use]
    pub fn peek_reg8(&self, slv_addr: u8, reg: u8) -> Option<u8> {
        let bank = self.resolve_reg8_bank(slv_addr, reg);
        self.lookup_reg(slv_addr, RegWidth::Bits8, bank, u16::from(reg))
    }

    /// Inspects an 8-bit register in `bank`, returning `Some(val)` if present.
    #[must_use]
    pub fn peek_reg8_banked(&self, slv_addr: u8, bank: u8, reg: u8) -> Option<u8> {
        self.lookup_reg(slv_addr, RegWidth::Bits8, bank, u16::from(reg))
    }

    /// Inspects a 16-bit register, returning `Some(val)` if present in store.
    #[must_use]
    pub fn peek_reg16(&self, slv_addr: u8, reg: u16) -> Option<u8> {
        self.lookup_reg(slv_addr, RegWidth::Bits16, 0, reg)
    }

    /// Returns the recorded slice of transactions (up to `CAP`).
    #[must_use]
    pub fn transactions(&self) -> &[SccbTransaction] {
        &self.tx_log[..self.tx_len]
    }

    /// Returns the number of recorded transactions in the log.
    #[must_use]
    pub const fn transaction_count(&self) -> usize {
        self.tx_len
    }

    /// Returns the number of transactions dropped because `tx_log` reached
    /// `CAP` (`SCCB-05`).
    #[must_use]
    pub const fn dropped_count(&self) -> usize {
        self.dropped_tx_count
    }

    /// Returns total logical register bytes written since creation or the last
    /// [`Self::clear_transactions`] call (`+1` per single write, `+len` per
    /// sequential burst write).
    #[must_use]
    pub const fn register_write_count(&self) -> usize {
        self.total_reg_writes
    }

    /// Asserts that recorded transactions match `expected` and zero entries
    /// were dropped due to log capacity overflow.
    ///
    /// # Panics
    /// Panics if `dropped_count() > 0` or `transactions() != expected`.
    #[track_caller]
    pub fn assert_transactions(&self, expected: &[SccbTransaction]) {
        assert_eq!(
            self.dropped_tx_count, 0,
            "MockSccbBus dropped {} transaction(s) beyond CAP={CAP}",
            self.dropped_tx_count
        );
        assert_eq!(
            self.transactions(),
            expected,
            "MockSccbBus transaction sequence mismatch"
        );
    }

    /// Clears the recorded transaction log and counters without clearing
    /// register state.
    pub fn clear_transactions(&mut self) {
        self.tx_len = 0;
        self.dropped_tx_count = 0;
        self.total_reg_writes = 0;
    }

    /// Returns total accumulated delay in milliseconds passed to `delay_ms`.
    #[must_use]
    pub const fn total_delay_ms(&self) -> u32 {
        self.total_delay_ms
    }

    const fn resolve_reg8_bank(&self, slv_addr: u8, reg: u8) -> u8 {
        let mut idx = 0;
        while idx < MAX_BANK_TRACKERS {
            if self.bank_trackers[idx].enabled && self.bank_trackers[idx].slv_addr == slv_addr {
                if reg == self.bank_trackers[idx].bank_reg {
                    return 0;
                }
                return self.bank_trackers[idx].active_bank;
            }
            idx += 1;
        }
        0
    }

    fn store_reg8_bank_aware(&mut self, slv_addr: u8, reg: u8, val: u8) -> Result<(), CameraError> {
        let mut tracker_idx = None;
        let mut idx = 0;
        while idx < MAX_BANK_TRACKERS {
            if self.bank_trackers[idx].enabled && self.bank_trackers[idx].slv_addr == slv_addr {
                tracker_idx = Some(idx);
                break;
            }
            idx += 1;
        }
        if let Some(slot) = tracker_idx {
            if reg == self.bank_trackers[slot].bank_reg {
                self.store_reg(slv_addr, RegWidth::Bits8, 0, u16::from(reg), val)?;
                self.bank_trackers[slot].active_bank = val;
                return Ok(());
            }
            let bank = self.bank_trackers[slot].active_bank;
            return self.store_reg(slv_addr, RegWidth::Bits8, bank, u16::from(reg), val);
        }
        self.store_reg(slv_addr, RegWidth::Bits8, 0, u16::from(reg), val)
    }

    fn lookup_reg(&self, slv_addr: u8, width: RegWidth, bank: u8, reg: u16) -> Option<u8> {
        let mut i = 0;
        while i < self.reg_len {
            let entry = &self.regs[i];
            if entry.occupied
                && entry.slv_addr == slv_addr
                && entry.width == width
                && entry.bank == bank
                && entry.reg == reg
            {
                return Some(entry.val);
            }
            i += 1;
        }
        None
    }

    fn store_reg(
        &mut self,
        slv_addr: u8,
        width: RegWidth,
        bank: u8,
        reg: u16,
        val: u8,
    ) -> Result<(), CameraError> {
        if CAP == 0 {
            return Err(CameraError::NoMem);
        }
        let mut i = 0;
        while i < self.reg_len {
            let entry = &self.regs[i];
            if entry.occupied
                && entry.slv_addr == slv_addr
                && entry.width == width
                && entry.bank == bank
                && entry.reg == reg
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
                width,
                bank,
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
            self.dropped_tx_count += 1;
            return;
        }
        if self.tx_len < CAP {
            self.tx_log[self.tx_len] = tx;
            self.tx_len += 1;
        } else {
            self.dropped_tx_count += 1;
        }
    }

    fn fail_op(&mut self, slv_addr: u8, reg: u16, err: CameraError) -> CameraError {
        self.push_tx(SccbTransaction::Failed { slv_addr, reg, err });
        err
    }

    fn check_rw_preconditions(&mut self, slv_addr: u8, reg: u16) -> Result<(), CameraError> {
        if !is_valid_sccb_addr(slv_addr) {
            return Err(self.fail_op(slv_addr, reg, CameraError::InvalidArg));
        }
        if let Some(rem) = self.ops_until_fail {
            if rem <= 1 {
                self.ops_until_fail = None;
                let err = self.fail_at_err;
                return Err(self.fail_op(slv_addr, reg, err));
            }
            self.ops_until_fail = Some(rem - 1);
        }
        if self.strict_ack && !self.has_device(slv_addr) {
            let err = CameraError::SccbNack {
                slv_addr,
                phase: SccbPhase::DeviceAddr,
            };
            return Err(self.fail_op(slv_addr, reg, err));
        }
        if let Some(err) = self.forced_error {
            return Err(self.fail_op(slv_addr, reg, err));
        }
        Ok(())
    }
}

impl<const CAP: usize> SccbBus for MockSccbBus<CAP> {
    fn read_reg8(&mut self, slv_addr: u8, reg: u8) -> Result<u8, CameraError> {
        self.check_rw_preconditions(slv_addr, u16::from(reg))?;
        let bank = self.resolve_reg8_bank(slv_addr, reg);
        let val = match self.lookup_reg(slv_addr, RegWidth::Bits8, bank, u16::from(reg)) {
            Some(v) => v,
            None => match self.unset_read_policy {
                UnsetReadPolicy::Zero => 0,
                UnsetReadPolicy::Error => {
                    return Err(self.fail_op(slv_addr, u16::from(reg), CameraError::SccbError));
                }
            },
        };
        self.push_tx(SccbTransaction::Read8 { slv_addr, reg, val });
        Ok(val)
    }

    fn write_reg8(&mut self, slv_addr: u8, reg: u8, data: u8) -> Result<(), CameraError> {
        self.check_rw_preconditions(slv_addr, u16::from(reg))?;
        if let Err(err) = self.store_reg8_bank_aware(slv_addr, reg, data) {
            return Err(self.fail_op(slv_addr, u16::from(reg), err));
        }
        self.total_reg_writes += 1;
        self.push_tx(SccbTransaction::Write8 {
            slv_addr,
            reg,
            val: data,
        });
        Ok(())
    }

    fn write_reg8_seq(
        &mut self,
        slv_addr: u8,
        start_reg: u8,
        data: &[u8],
    ) -> Result<(), CameraError> {
        if !is_valid_sccb_addr(slv_addr) {
            return Err(self.fail_op(slv_addr, u16::from(start_reg), CameraError::InvalidArg));
        }
        if data.is_empty() {
            return Ok(());
        }
        let Ok(last_off) = u8::try_from(data.len() - 1) else {
            return Err(self.fail_op(slv_addr, u16::from(start_reg), CameraError::InvalidArg));
        };
        if start_reg.checked_add(last_off).is_none() {
            return Err(self.fail_op(slv_addr, u16::from(start_reg), CameraError::InvalidArg));
        }
        self.check_rw_preconditions(slv_addr, u16::from(start_reg))?;
        for (idx, &byte) in data.iter().enumerate() {
            let reg = start_reg + (idx as u8);
            if let Err(err) = self.store_reg8_bank_aware(slv_addr, reg, byte) {
                return Err(self.fail_op(slv_addr, u16::from(reg), err));
            }
        }
        self.total_reg_writes += data.len();
        self.push_tx(SccbTransaction::Write8Seq {
            slv_addr,
            start_reg,
            len: data.len() as u16,
        });
        Ok(())
    }

    fn read_reg16(&mut self, slv_addr: u8, reg: u16) -> Result<u8, CameraError> {
        self.check_rw_preconditions(slv_addr, reg)?;
        let val = match self.lookup_reg(slv_addr, RegWidth::Bits16, 0, reg) {
            Some(v) => v,
            None => match self.unset_read_policy {
                UnsetReadPolicy::Zero => 0,
                UnsetReadPolicy::Error => {
                    return Err(self.fail_op(slv_addr, reg, CameraError::SccbError));
                }
            },
        };
        self.push_tx(SccbTransaction::Read16 { slv_addr, reg, val });
        Ok(val)
    }

    fn write_reg16(&mut self, slv_addr: u8, reg: u16, data: u8) -> Result<(), CameraError> {
        self.check_rw_preconditions(slv_addr, reg)?;
        if let Err(err) = self.store_reg(slv_addr, RegWidth::Bits16, 0, reg, data) {
            return Err(self.fail_op(slv_addr, reg, err));
        }
        self.total_reg_writes += 1;
        self.push_tx(SccbTransaction::Write16 {
            slv_addr,
            reg,
            val: data,
        });
        Ok(())
    }

    fn write_reg16_seq(
        &mut self,
        slv_addr: u8,
        start_reg: u16,
        data: &[u8],
    ) -> Result<(), CameraError> {
        if !is_valid_sccb_addr(slv_addr) {
            return Err(self.fail_op(slv_addr, start_reg, CameraError::InvalidArg));
        }
        if data.is_empty() {
            return Ok(());
        }
        let Ok(last_off) = u16::try_from(data.len() - 1) else {
            return Err(self.fail_op(slv_addr, start_reg, CameraError::InvalidArg));
        };
        if start_reg.checked_add(last_off).is_none() {
            return Err(self.fail_op(slv_addr, start_reg, CameraError::InvalidArg));
        }
        self.check_rw_preconditions(slv_addr, start_reg)?;
        for (idx, &byte) in data.iter().enumerate() {
            let reg = start_reg + (idx as u16);
            if let Err(err) = self.store_reg(slv_addr, RegWidth::Bits16, 0, reg, byte) {
                return Err(self.fail_op(slv_addr, reg, err));
            }
        }
        self.total_reg_writes += data.len();
        self.push_tx(SccbTransaction::Write16Seq {
            slv_addr,
            start_reg,
            len: data.len() as u16,
        });
        Ok(())
    }

    fn probe(&mut self, slv_addr: u8) -> Result<(), CameraError> {
        if !is_valid_sccb_addr(slv_addr) {
            return Err(self.fail_op(slv_addr, 0, CameraError::InvalidArg));
        }
        if let Some(rem) = self.ops_until_fail {
            if rem <= 1 {
                self.ops_until_fail = None;
                let err = self.fail_at_err;
                return Err(self.fail_op(slv_addr, 0, err));
            }
            self.ops_until_fail = Some(rem - 1);
        }
        if let Some(err) = self.forced_error {
            return Err(self.fail_op(slv_addr, 0, err));
        }
        if (self.strict_ack || self.device_mask[0] != 0 || self.device_mask[1] != 0)
            && !self.has_device(slv_addr)
        {
            let err = CameraError::SccbNack {
                slv_addr,
                phase: SccbPhase::DeviceAddr,
            };
            return Err(self.fail_op(slv_addr, 0, err));
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

        assert_eq!(
            bus.read_reg16_val16(CameraSccbAddr::OV5640, 0xFFFF),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(
            bus.write_reg16_val16(CameraSccbAddr::OV5640, 0xFFFF, 0x1234),
            Err(CameraError::InvalidArg)
        );

        assert_eq!(bus.write_reg8(0x30, 0x01, 0xAA), Ok(()));
        assert_eq!(bus.write_reg8(0x30, 0x02, 0xBB), Ok(()));
        assert_eq!(bus.write_reg8(0x30, 0x03, 0xCC), Err(CameraError::NoMem));
        assert_eq!(bus.write_reg8(0x30, 0x01, 0xFF), Ok(()));
        assert_eq!(bus.read_reg8(0x30, 0x01), Ok(0xFF));
    }

    #[test]
    fn test_mock_sccb_strict_ack_missing() {
        let mut bus: MockSccbBus<64> = MockSccbBus::new();
        bus.set_strict_ack(true);
        bus.preset_reg8(CameraSccbAddr::OV2640, 0x0A, 0x26);

        assert_eq!(bus.probe(CameraSccbAddr::OV2640), Ok(()));
        assert_eq!(
            bus.probe(CameraSccbAddr::OV7725),
            Err(CameraError::SccbNack {
                slv_addr: CameraSccbAddr::OV7725,
                phase: SccbPhase::DeviceAddr,
            })
        );
        assert_eq!(
            bus.probe_strict(CameraSccbAddr::OV7725),
            Err(SccbError::AckMissing)
        );

        bus.register_device_addr(CameraSccbAddr::OV7725);
        assert_eq!(bus.probe(CameraSccbAddr::OV7725), Ok(()));
        assert_eq!(bus.probe_strict(CameraSccbAddr::OV7725), Ok(()));

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
                phase: SccbPhase::DeviceAddr,
            })
        );
        assert_eq!(
            bus.write_reg8(0x30, 0x0A, 0x12),
            Err(CameraError::SccbNack {
                slv_addr: 0x30,
                phase: SccbPhase::DeviceAddr,
            })
        );
        assert_eq!(
            bus.read_reg16(0x3C, 0x300A),
            Err(CameraError::SccbNack {
                slv_addr: 0x3C,
                phase: SccbPhase::DeviceAddr,
            })
        );
        assert_eq!(
            bus.write_reg16(0x3C, 0x300A, 0x56),
            Err(CameraError::SccbNack {
                slv_addr: 0x3C,
                phase: SccbPhase::DeviceAddr,
            })
        );

        bus.register_device_addr(0x30);
        assert_eq!(bus.write_reg8(0x30, 0x0A, 0x26), Ok(()));
        assert_eq!(bus.read_reg8(0x30, 0x0A), Ok(0x26));
    }

    #[test]
    fn test_sccb05_dropped_count_fail_at_and_assert_transactions() {
        let mut bus: MockSccbBus<3> = MockSccbBus::new();
        bus.fail_at(3);

        assert_eq!(bus.write_reg8(0x30, 0x01, 0x11), Ok(()));
        assert_eq!(bus.write_reg8(0x30, 0x02, 0x22), Ok(()));
        assert_eq!(
            bus.write_reg8(0x30, 0x03, 0x33),
            Err(CameraError::SccbError)
        );
        assert_eq!(bus.dropped_count(), 0);
        bus.assert_transactions(&[
            SccbTransaction::Write8 {
                slv_addr: 0x30,
                reg: 0x01,
                val: 0x11,
            },
            SccbTransaction::Write8 {
                slv_addr: 0x30,
                reg: 0x02,
                val: 0x22,
            },
            SccbTransaction::Failed {
                slv_addr: 0x30,
                reg: 0x03,
                err: CameraError::SccbError,
            },
        ]);

        // Fourth log entry overflows CAP=3 without evicting early history.
        bus.delay_ms(5);
        assert_eq!(bus.dropped_count(), 1);
        assert_eq!(bus.transaction_count(), 3);

        bus.clear_transactions();
        assert_eq!(bus.dropped_count(), 0);
        assert_eq!(bus.transaction_count(), 0);
    }

    #[test]
    fn test_sccb06_banked_registers_width_separation_and_unset_policy() {
        let mut bus: MockSccbBus<32> = MockSccbBus::new()
            .with_bank_select(CameraSccbAddr::OV2640, 0xFF)
            .with_unset_read_policy(UnsetReadPolicy::Error);

        // Unset reads fail in strict unset policy mode and log Failed.
        assert_eq!(
            bus.read_reg8(CameraSccbAddr::OV2640, 0x44),
            Err(CameraError::SccbError)
        );
        bus.clear_transactions();

        // Bank 0 (DSP) vs Bank 1 (Sensor) register 0x44 hold distinct values.
        assert_eq!(bus.write_reg8(CameraSccbAddr::OV2640, 0xFF, 0x00), Ok(()));
        assert_eq!(bus.write_reg8(CameraSccbAddr::OV2640, 0x44, 0x0C), Ok(()));
        assert_eq!(bus.write_reg8(CameraSccbAddr::OV2640, 0xFF, 0x01), Ok(()));
        assert_eq!(bus.write_reg8(CameraSccbAddr::OV2640, 0x44, 0x7E), Ok(()));

        assert_eq!(bus.reg8_banked(CameraSccbAddr::OV2640, 0, 0x44), 0x0C);
        assert_eq!(bus.reg8_banked(CameraSccbAddr::OV2640, 1, 0x44), 0x7E);
        assert_eq!(bus.read_reg8(CameraSccbAddr::OV2640, 0x44), Ok(0x7E));
        assert_eq!(bus.write_reg8(CameraSccbAddr::OV2640, 0xFF, 0x00), Ok(()));
        assert_eq!(bus.read_reg8(CameraSccbAddr::OV2640, 0x44), Ok(0x0C));

        // 8-bit 0x44 and 16-bit 0x0044 never alias.
        assert_eq!(
            bus.write_reg16(CameraSccbAddr::OV2640, 0x0044, 0x99),
            Ok(())
        );
        assert_eq!(bus.read_reg16(CameraSccbAddr::OV2640, 0x0044), Ok(0x99));
        assert_eq!(bus.read_reg8(CameraSccbAddr::OV2640, 0x44), Ok(0x0C));
    }

    #[test]
    fn test_sccb02_seq_writes_and_register_write_count() {
        let mut bus: MockSccbBus<32> = MockSccbBus::new();
        assert_eq!(bus.write_reg8_seq(0x30, 0x10, &[0xAA, 0xBB, 0xCC]), Ok(()));
        assert_eq!(bus.write_reg16_seq(0x3C, 0x8000, &[0x01, 0x02]), Ok(()));
        assert_eq!(bus.register_write_count(), 5);
        assert_eq!(bus.reg8(0x30, 0x10), 0xAA);
        assert_eq!(bus.reg8(0x30, 0x11), 0xBB);
        assert_eq!(bus.reg8(0x30, 0x12), 0xCC);
        assert_eq!(bus.reg16(0x3C, 0x8000), 0x01);
        assert_eq!(bus.reg16(0x3C, 0x8001), 0x02);
        bus.assert_transactions(&[
            SccbTransaction::Write8Seq {
                slv_addr: 0x30,
                start_reg: 0x10,
                len: 3,
            },
            SccbTransaction::Write16Seq {
                slv_addr: 0x3C,
                start_reg: 0x8000,
                len: 2,
            },
        ]);
    }
}
