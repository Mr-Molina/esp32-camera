//! Hardware-agnostic SCCB / I2C register bus trait for camera sensors.

use esp32_cam_types::CameraError;

/// OmniVision 16-bit group-hold control register (`SRB CTRL`).
pub const OV_GROUP_HOLD_REG: u16 = 0x3212;

const MAX_BULK_COALESCE_BYTES: usize = 64;

/// Returns true if `addr` is a valid 7-bit SCCB / I2C slave address
/// (`0x08..=0x77`).
///
/// Addresses `0x00..=0x07` and `0x78..=0x7F` are reserved by the I2C
/// specification.
#[must_use]
#[inline]
pub const fn is_valid_sccb_addr(addr: u8) -> bool {
    matches!(addr, 0x08..=0x77)
}

/// Hardware-agnostic SCCB / I2C register bus trait for camera sensors.
///
/// Supports both 8-bit register addressing (e.g. `OV2640`, `OV7725`, `GC0308`)
/// and 16-bit big-endian register addressing (e.g. `OV5640`, `OV3660`).
pub trait SccbBus {
    /// Reads an 8-bit value from an 8-bit register address on `slv_addr`.
    fn read_reg8(&mut self, slv_addr: u8, reg: u8) -> Result<u8, CameraError>;

    /// Writes an 8-bit value to an 8-bit register address on `slv_addr`.
    fn write_reg8(&mut self, slv_addr: u8, reg: u8, data: u8) -> Result<(), CameraError>;

    /// Performs a read-modify-write on an 8-bit register using `mask`.
    ///
    /// Always writes `next` when `mask != 0` (even if `next == current`) so
    /// write-side-effect registers (triggers, strobes, bank-sensitive latches)
    /// always fire, matching upstream C `SCCB_Write` semantics. For plain
    /// storage registers where skipping redundant writes is safe, call
    /// [`Self::write_reg8_mask_if_changed`].
    fn write_reg8_mask(
        &mut self,
        slv_addr: u8,
        reg: u8,
        val: u8,
        mask: u8,
    ) -> Result<(), CameraError> {
        if !is_valid_sccb_addr(slv_addr) {
            return Err(CameraError::InvalidArg);
        }
        if mask == 0 {
            return Ok(());
        }
        if mask == 0xFF {
            return self.write_reg8(slv_addr, reg, val);
        }
        let current = self.read_reg8(slv_addr, reg)?;
        let next = (current & !mask) | (val & mask);
        self.write_reg8(slv_addr, reg, next)
    }

    /// Performs a read-modify-write on a plain storage 8-bit register,
    /// skipping the write transaction when the masked bits are already equal.
    fn write_reg8_mask_if_changed(
        &mut self,
        slv_addr: u8,
        reg: u8,
        val: u8,
        mask: u8,
    ) -> Result<(), CameraError> {
        if !is_valid_sccb_addr(slv_addr) {
            return Err(CameraError::InvalidArg);
        }
        if mask == 0 {
            return Ok(());
        }
        if mask == 0xFF {
            return self.write_reg8(slv_addr, reg, val);
        }
        let current = self.read_reg8(slv_addr, reg)?;
        let next = (current & !mask) | (val & mask);
        if next == current {
            return Ok(());
        }
        self.write_reg8(slv_addr, reg, next)
    }

    /// Reads an 8-bit value from a 16-bit register address on `slv_addr`.
    fn read_reg16(&mut self, slv_addr: u8, reg: u16) -> Result<u8, CameraError>;

    /// Writes an 8-bit value to a 16-bit register address on `slv_addr`.
    fn write_reg16(&mut self, slv_addr: u8, reg: u16, data: u8) -> Result<(), CameraError>;

    /// Performs a read-modify-write on a 16-bit register using `mask`.
    ///
    /// Always writes `next` when `mask != 0` (even if `next == current`) so
    /// write-side-effect registers such as group-hold launch (`0x3212`) fire
    /// unconditionally. Use [`Self::write_reg16_mask_if_changed`] only for
    /// side-effect-free storage registers.
    fn write_reg16_mask(
        &mut self,
        slv_addr: u8,
        reg: u16,
        val: u8,
        mask: u8,
    ) -> Result<(), CameraError> {
        if !is_valid_sccb_addr(slv_addr) {
            return Err(CameraError::InvalidArg);
        }
        if mask == 0 {
            return Ok(());
        }
        if mask == 0xFF {
            return self.write_reg16(slv_addr, reg, val);
        }
        let current = self.read_reg16(slv_addr, reg)?;
        let next = (current & !mask) | (val & mask);
        self.write_reg16(slv_addr, reg, next)
    }

    /// Performs a read-modify-write on a plain storage 16-bit register,
    /// skipping the write transaction when the masked bits are already equal.
    fn write_reg16_mask_if_changed(
        &mut self,
        slv_addr: u8,
        reg: u16,
        val: u8,
        mask: u8,
    ) -> Result<(), CameraError> {
        if !is_valid_sccb_addr(slv_addr) {
            return Err(CameraError::InvalidArg);
        }
        if mask == 0 {
            return Ok(());
        }
        if mask == 0xFF {
            return self.write_reg16(slv_addr, reg, val);
        }
        let current = self.read_reg16(slv_addr, reg)?;
        let next = (current & !mask) | (val & mask);
        if next == current {
            return Ok(());
        }
        self.write_reg16(slv_addr, reg, next)
    }

    /// Writes `data` sequentially starting at 8-bit register `start_reg`.
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
        for (idx, &byte) in data.iter().enumerate() {
            self.write_reg8(slv_addr, start_reg + (idx as u8), byte)?;
        }
        Ok(())
    }

    /// Writes `data` sequentially starting at 16-bit register `start_reg`.
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
        for (idx, &byte) in data.iter().enumerate() {
            self.write_reg16(slv_addr, start_reg + (idx as u16), byte)?;
        }
        Ok(())
    }

    /// Reads a 16-bit big-endian value across `reg` and `reg + 1`.
    ///
    /// # Note on Atomicity
    /// SCCB reads one 8-bit register per bus phase, so reading `reg` and
    /// `reg + 1` is **not** hardware-atomic across frame boundaries unless the
    /// sensor latches high/low byte pairs internally.
    fn read_reg16_val16(&mut self, slv_addr: u8, reg: u16) -> Result<u16, CameraError> {
        if !is_valid_sccb_addr(slv_addr) {
            return Err(CameraError::InvalidArg);
        }
        let next_reg = reg.checked_add(1).ok_or(CameraError::InvalidArg)?;
        let hi = u16::from(self.read_reg16(slv_addr, reg)?);
        let lo = u16::from(self.read_reg16(slv_addr, next_reg)?);
        Ok((hi << 8) | lo)
    }

    /// Writes a 16-bit big-endian value across `reg` and `reg + 1` using
    /// [`Self::write_reg16_seq`].
    ///
    /// # Note on Atomicity
    /// Even when transmitted in a single sequential I2C burst, the sensor may
    /// latch `reg` and `reg + 1` across a VSYNC boundary unless wrapped in a
    /// sensor group-hold transaction such as [`with_ov_group_hold`].
    fn write_reg16_val16(&mut self, slv_addr: u8, reg: u16, data: u16) -> Result<(), CameraError> {
        self.write_reg16_seq(slv_addr, reg, &data.to_be_bytes())
    }

    /// Writes multiple 8-bit register address/value pairs in sequence,
    /// coalescing contiguous ascending addresses into [`Self::write_reg8_seq`].
    fn write_reg8_bulk(&mut self, addr: u8, pairs: &[(u8, u8)]) -> Result<(), CameraError> {
        if !is_valid_sccb_addr(addr) {
            return Err(CameraError::InvalidArg);
        }
        let mut idx = 0;
        while idx < pairs.len() {
            let (start_reg, first_val) = pairs[idx];
            let mut buf = [0u8; MAX_BULK_COALESCE_BYTES];
            buf[0] = first_val;
            let mut run_len = 1usize;
            while idx + run_len < pairs.len() && run_len < MAX_BULK_COALESCE_BYTES {
                let prev_reg = pairs[idx + run_len - 1].0;
                let (cand_reg, cand_val) = pairs[idx + run_len];
                if prev_reg.checked_add(1) == Some(cand_reg) {
                    buf[run_len] = cand_val;
                    run_len += 1;
                } else {
                    break;
                }
            }
            if run_len == 1 {
                self.write_reg8(addr, start_reg, first_val)?;
            } else {
                self.write_reg8_seq(addr, start_reg, &buf[..run_len])?;
            }
            idx += run_len;
        }
        Ok(())
    }

    /// Writes multiple 16-bit register address/value pairs in sequence,
    /// coalescing contiguous ascending addresses into [`Self::write_reg16_seq`].
    fn write_reg16_bulk(&mut self, addr: u8, pairs: &[(u16, u8)]) -> Result<(), CameraError> {
        if !is_valid_sccb_addr(addr) {
            return Err(CameraError::InvalidArg);
        }
        let mut idx = 0;
        while idx < pairs.len() {
            let (start_reg, first_val) = pairs[idx];
            let mut buf = [0u8; MAX_BULK_COALESCE_BYTES];
            buf[0] = first_val;
            let mut run_len = 1usize;
            while idx + run_len < pairs.len() && run_len < MAX_BULK_COALESCE_BYTES {
                let prev_reg = pairs[idx + run_len - 1].0;
                let (cand_reg, cand_val) = pairs[idx + run_len];
                if prev_reg.checked_add(1) == Some(cand_reg) {
                    buf[run_len] = cand_val;
                    run_len += 1;
                } else {
                    break;
                }
            }
            if run_len == 1 {
                self.write_reg16(addr, start_reg, first_val)?;
            } else {
                self.write_reg16_seq(addr, start_reg, &buf[..run_len])?;
            }
            idx += run_len;
        }
        Ok(())
    }

    /// Probes whether a device responds at the 7-bit `slv_addr`.
    fn probe(&mut self, slv_addr: u8) -> Result<(), CameraError> {
        if !is_valid_sccb_addr(slv_addr) {
            return Err(CameraError::InvalidArg);
        }
        let _ = self.read_reg8(slv_addr, 0x00)?;
        Ok(())
    }

    /// Delays execution for `ms` milliseconds.
    fn delay_ms(&mut self, ms: u32);
}

/// Executes `f` inside an OmniVision 16-bit group-hold transaction (`0x3212`).
///
/// Writes `0x00 | group_id` to start group hold, executes `f(bus)`, then writes
/// `0x10 | group_id` (group end) followed by `0xA0 | group_id` (group launch)
/// so multi-register exposure/gain/timing pairs latch atomically at frame end.
///
/// # Errors
/// Returns [`CameraError::InvalidArg`] if `slv_addr` is outside `0x08..=0x77`
/// or `group_id > 3`, or propagates any [`CameraError`] from `bus` or `f`.
pub fn with_ov_group_hold<B, R, F>(
    bus: &mut B,
    slv_addr: u8,
    group_id: u8,
    f: F,
) -> Result<R, CameraError>
where
    B: SccbBus + ?Sized,
    F: FnOnce(&mut B) -> Result<R, CameraError>,
{
    if !is_valid_sccb_addr(slv_addr) || group_id > 0x03 {
        return Err(CameraError::InvalidArg);
    }
    bus.write_reg16(slv_addr, OV_GROUP_HOLD_REG, group_id)?;
    let out = f(bus)?;
    bus.write_reg16(slv_addr, OV_GROUP_HOLD_REG, 0x10 | group_id)?;
    bus.write_reg16(slv_addr, OV_GROUP_HOLD_REG, 0xA0 | group_id)?;
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    struct SpyBus {
        reads: usize,
        writes: usize,
        seq8_calls: usize,
        seq16_calls: usize,
        reg_val: u8,
        log16: [(u16, u8); 8],
        log16_len: usize,
    }

    impl SpyBus {
        const fn new(reg_val: u8) -> Self {
            Self {
                reads: 0,
                writes: 0,
                seq8_calls: 0,
                seq16_calls: 0,
                reg_val,
                log16: [(0, 0); 8],
                log16_len: 0,
            }
        }
    }

    impl SccbBus for SpyBus {
        fn read_reg8(&mut self, _slv_addr: u8, _reg: u8) -> Result<u8, CameraError> {
            self.reads += 1;
            Ok(self.reg_val)
        }

        fn write_reg8(&mut self, _slv_addr: u8, _reg: u8, data: u8) -> Result<(), CameraError> {
            self.writes += 1;
            self.reg_val = data;
            Ok(())
        }

        fn read_reg16(&mut self, _slv_addr: u8, _reg: u16) -> Result<u8, CameraError> {
            self.reads += 1;
            Ok(self.reg_val)
        }

        fn write_reg16(&mut self, _slv_addr: u8, reg: u16, data: u8) -> Result<(), CameraError> {
            self.writes += 1;
            self.reg_val = data;
            if self.log16_len < self.log16.len() {
                self.log16[self.log16_len] = (reg, data);
                self.log16_len += 1;
            }
            Ok(())
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
            self.seq8_calls += 1;
            for (idx, &byte) in data.iter().enumerate() {
                self.write_reg8(slv_addr, start_reg + (idx as u8), byte)?;
            }
            Ok(())
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
            self.seq16_calls += 1;
            for (idx, &byte) in data.iter().enumerate() {
                self.write_reg16(slv_addr, start_reg + (idx as u16), byte)?;
            }
            Ok(())
        }

        fn delay_ms(&mut self, _ms: u32) {}
    }

    #[test]
    fn test_write_reg8_mask_fast_paths_and_if_changed() {
        let mut bus = SpyBus::new(0x55);

        assert_eq!(bus.write_reg8_mask(0x30, 0x10, 0xAA, 0x00), Ok(()));
        assert_eq!(bus.reads, 0, "mask 0 must not read");
        assert_eq!(bus.writes, 0, "mask 0 must not write");

        assert_eq!(bus.write_reg8_mask(0x30, 0x10, 0xAA, 0xFF), Ok(()));
        assert_eq!(bus.reads, 0, "mask 0xFF must skip read");
        assert_eq!(bus.writes, 1, "mask 0xFF must write directly");
        assert_eq!(bus.reg_val, 0xAA);

        // SCCB-01: write_reg8_mask ALWAYS writes even when next == current.
        bus.reads = 0;
        bus.writes = 0;
        assert_eq!(bus.write_reg8_mask(0x30, 0x10, 0x0A, 0x0F), Ok(()));
        assert_eq!(bus.reads, 1, "partial mask must read once");
        assert_eq!(bus.writes, 1, "SCCB-01 masked write must always write");
        assert_eq!(bus.reg_val, 0xAA);

        // Opt-in write_reg8_mask_if_changed skips write when unchanged.
        bus.reads = 0;
        bus.writes = 0;
        assert_eq!(
            bus.write_reg8_mask_if_changed(0x30, 0x10, 0x0A, 0x0F),
            Ok(())
        );
        assert_eq!(bus.reads, 1);
        assert_eq!(bus.writes, 0, "if_changed must skip redundant write");

        bus.reads = 0;
        bus.writes = 0;
        assert_eq!(
            bus.write_reg8_mask_if_changed(0x30, 0x10, 0x05, 0x0F),
            Ok(())
        );
        assert_eq!(bus.reads, 1);
        assert_eq!(bus.writes, 1);
        assert_eq!(bus.reg_val, 0xA5);
    }

    #[test]
    fn test_write_reg16_mask_fast_paths_and_if_changed() {
        let mut bus = SpyBus::new(0x55);

        assert_eq!(bus.write_reg16_mask(0x30, 0x1000, 0xAA, 0x00), Ok(()));
        assert_eq!(bus.reads, 0);
        assert_eq!(bus.writes, 0);

        assert_eq!(bus.write_reg16_mask(0x30, 0x1000, 0xAA, 0xFF), Ok(()));
        assert_eq!(bus.reads, 0);
        assert_eq!(bus.writes, 1);
        assert_eq!(bus.reg_val, 0xAA);

        // SCCB-01: write_reg16_mask ALWAYS writes even when next == current.
        bus.reads = 0;
        bus.writes = 0;
        assert_eq!(bus.write_reg16_mask(0x30, 0x1000, 0x0A, 0x0F), Ok(()));
        assert_eq!(bus.reads, 1);
        assert_eq!(
            bus.writes, 1,
            "SCCB-01 16-bit masked write must always write"
        );

        // Opt-in write_reg16_mask_if_changed skips write when unchanged.
        bus.reads = 0;
        bus.writes = 0;
        assert_eq!(
            bus.write_reg16_mask_if_changed(0x30, 0x1000, 0x0A, 0x0F),
            Ok(())
        );
        assert_eq!(bus.reads, 1);
        assert_eq!(bus.writes, 0);

        bus.reads = 0;
        bus.writes = 0;
        assert_eq!(
            bus.write_reg16_mask_if_changed(0x30, 0x1000, 0x05, 0x0F),
            Ok(())
        );
        assert_eq!(bus.reads, 1);
        assert_eq!(bus.writes, 1);
        assert_eq!(bus.reg_val, 0xA5);
    }

    #[test]
    fn test_bulk_coalescing_and_seq_validation() {
        let mut bus = SpyBus::new(0);
        let pairs8 = [(0x01, 0x10), (0x02, 0x20), (0x03, 0x30), (0x10, 0x40)];
        assert_eq!(bus.write_reg8_bulk(0x30, &pairs8), Ok(()));
        assert_eq!(bus.writes, 4);
        assert_eq!(
            bus.seq8_calls, 1,
            "0x01..=0x03 must coalesce into 1 seq8 call"
        );

        let pairs16 = [(0x1001, 0x10), (0x1002, 0x20), (0x2000, 0x99)];
        assert_eq!(bus.write_reg16_bulk(0x30, &pairs16), Ok(()));
        assert_eq!(bus.writes, 7);
        assert_eq!(
            bus.seq16_calls, 1,
            "0x1001..=0x1002 must coalesce into 1 seq16 call"
        );

        assert_eq!(
            bus.write_reg8_seq(0x30, 0xFF, &[1, 2]),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(
            bus.write_reg16_seq(0x30, 0xFFFF, &[1, 2]),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(bus.write_reg16_seq(0x30, 0xFFFF, &[]), Ok(()));
        assert_eq!(
            bus.write_reg8_bulk(0x00, &pairs8),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(
            bus.write_reg16_bulk(0x78, &pairs16),
            Err(CameraError::InvalidArg)
        );
    }

    #[test]
    fn test_ov_group_hold_wraps_16bit_pair() {
        let mut bus = SpyBus::new(0);
        let res = with_ov_group_hold(&mut bus, 0x3C, 1, |b| {
            b.write_reg16_val16(0x3C, 0x3500, 0x1234)
        });
        assert_eq!(res, Ok(()));
        assert_eq!(
            bus.seq16_calls, 1,
            "write_reg16_val16 must use write_reg16_seq"
        );
        assert_eq!(
            &bus.log16[..bus.log16_len],
            &[
                (OV_GROUP_HOLD_REG, 0x01),
                (0x3500, 0x12),
                (0x3501, 0x34),
                (OV_GROUP_HOLD_REG, 0x11),
                (OV_GROUP_HOLD_REG, 0xA1),
            ]
        );
        assert_eq!(
            with_ov_group_hold(&mut bus, 0x3C, 4, |_| Ok(())),
            Err(CameraError::InvalidArg)
        );
    }

    #[test]
    fn test_is_valid_sccb_addr_and_mask_invalid_address() {
        assert!(!is_valid_sccb_addr(0x00));
        assert!(!is_valid_sccb_addr(0x07));
        assert!(is_valid_sccb_addr(0x08));
        assert!(is_valid_sccb_addr(0x30));
        assert!(is_valid_sccb_addr(0x77));
        assert!(!is_valid_sccb_addr(0x78));
        assert!(!is_valid_sccb_addr(0xFF));

        let mut bus = SpyBus::new(0x55);
        assert_eq!(
            bus.write_reg8_mask(0x00, 0x10, 0xAA, 0x00),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(
            bus.write_reg16_mask(0x78, 0x1000, 0xAA, 0x00),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(bus.reads, 0);
        assert_eq!(bus.writes, 0);
    }
}
