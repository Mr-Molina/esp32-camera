//! Hardware-agnostic SCCB / I2C register bus trait for camera sensors.

use esp32_cam_types::CameraError;

/// Returns true if `addr` is a valid 7-bit SCCB / I2C slave address
/// (0x08..=0x77).
///
/// Addresses 0x00..=0x07 and 0x78..=0x7F are reserved by the I2C specification.
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
        if next == current {
            return Ok(());
        }
        self.write_reg16(slv_addr, reg, next)
    }

    /// Reads a 16-bit big-endian value from a 16-bit register address.
    fn read_reg16_val16(&mut self, slv_addr: u8, reg: u16) -> Result<u16, CameraError> {
        if !is_valid_sccb_addr(slv_addr) {
            return Err(CameraError::InvalidArg);
        }
        let next_reg = reg.checked_add(1).ok_or(CameraError::InvalidArg)?;
        let hi = self.read_reg16(slv_addr, reg)? as u16;
        let lo = self.read_reg16(slv_addr, next_reg)? as u16;
        Ok((hi << 8) | lo)
    }

    /// Writes a 16-bit big-endian value to a 16-bit register address.
    fn write_reg16_val16(&mut self, slv_addr: u8, reg: u16, data: u16) -> Result<(), CameraError> {
        if !is_valid_sccb_addr(slv_addr) {
            return Err(CameraError::InvalidArg);
        }
        let next_reg = reg.checked_add(1).ok_or(CameraError::InvalidArg)?;
        self.write_reg16(slv_addr, reg, (data >> 8) as u8)?;
        self.write_reg16(slv_addr, next_reg, (data & 0xFF) as u8)
    }

    /// Writes multiple 8-bit register address/value pairs in sequence.
    fn write_reg8_bulk(&mut self, addr: u8, pairs: &[(u8, u8)]) -> Result<(), CameraError> {
        if !is_valid_sccb_addr(addr) {
            return Err(CameraError::InvalidArg);
        }
        for &(reg, data) in pairs {
            self.write_reg8(addr, reg, data)?;
        }
        Ok(())
    }

    /// Writes multiple 16-bit register address/value pairs in sequence.
    fn write_reg16_bulk(&mut self, addr: u8, pairs: &[(u16, u8)]) -> Result<(), CameraError> {
        if !is_valid_sccb_addr(addr) {
            return Err(CameraError::InvalidArg);
        }
        for &(reg, data) in pairs {
            self.write_reg16(addr, reg, data)?;
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

#[cfg(test)]
mod tests {
    use super::*;

    struct SpyBus {
        reads: usize,
        writes: usize,
        reg_val: u8,
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
        fn write_reg16(&mut self, _slv_addr: u8, _reg: u16, data: u8) -> Result<(), CameraError> {
            self.writes += 1;
            self.reg_val = data;
            Ok(())
        }
        fn delay_ms(&mut self, _ms: u32) {}
    }

    #[test]
    fn test_write_reg8_mask_fast_paths() {
        let mut bus = SpyBus {
            reads: 0,
            writes: 0,
            reg_val: 0x55,
        };

        // mask == 0: fast path (no read, no write)
        assert_eq!(bus.write_reg8_mask(0x30, 0x10, 0xAA, 0x00), Ok(()));
        assert_eq!(bus.reads, 0);
        assert_eq!(bus.writes, 0);

        // mask == 0xFF: direct write without read
        assert_eq!(bus.write_reg8_mask(0x30, 0x10, 0xAA, 0xFF), Ok(()));
        assert_eq!(bus.reads, 0);
        assert_eq!(bus.writes, 1);
        assert_eq!(bus.reg_val, 0xAA);

        // next == current: reads once, no write
        bus.reads = 0;
        bus.writes = 0;
        assert_eq!(bus.write_reg8_mask(0x30, 0x10, 0x0A, 0x0F), Ok(()));
        assert_eq!(bus.reads, 1);
        assert_eq!(bus.writes, 0);

        // modified bits: reads once, writes once
        bus.reads = 0;
        bus.writes = 0;
        assert_eq!(bus.write_reg8_mask(0x30, 0x10, 0x05, 0x0F), Ok(()));
        assert_eq!(bus.reads, 1);
        assert_eq!(bus.writes, 1);
        assert_eq!(bus.reg_val, 0xA5);
    }

    #[test]
    fn test_write_reg16_mask_fast_paths() {
        let mut bus = SpyBus {
            reads: 0,
            writes: 0,
            reg_val: 0x55,
        };

        // mask == 0: fast path (no read, no write)
        assert_eq!(bus.write_reg16_mask(0x30, 0x1000, 0xAA, 0x00), Ok(()));
        assert_eq!(bus.reads, 0);
        assert_eq!(bus.writes, 0);

        // mask == 0xFF: direct write without read
        assert_eq!(bus.write_reg16_mask(0x30, 0x1000, 0xAA, 0xFF), Ok(()));
        assert_eq!(bus.reads, 0);
        assert_eq!(bus.writes, 1);
        assert_eq!(bus.reg_val, 0xAA);

        // next == current: reads once, no write
        bus.reads = 0;
        bus.writes = 0;
        assert_eq!(bus.write_reg16_mask(0x30, 0x1000, 0x0A, 0x0F), Ok(()));
        assert_eq!(bus.reads, 1);
        assert_eq!(bus.writes, 0);

        // modified bits: reads once, writes once
        bus.reads = 0;
        bus.writes = 0;
        assert_eq!(bus.write_reg16_mask(0x30, 0x1000, 0x05, 0x0F), Ok(()));
        assert_eq!(bus.reads, 1);
        assert_eq!(bus.writes, 1);
        assert_eq!(bus.reg_val, 0xA5);
    }

    #[test]
    fn test_bulk_writes() {
        let mut bus = SpyBus {
            reads: 0,
            writes: 0,
            reg_val: 0,
        };
        let pairs8 = [(0x01, 0x10), (0x02, 0x20), (0x03, 0x30)];
        assert_eq!(bus.write_reg8_bulk(0x30, &pairs8), Ok(()));
        assert_eq!(bus.writes, 3);

        let pairs16 = [(0x1001, 0x10), (0x1002, 0x20)];
        assert_eq!(bus.write_reg16_bulk(0x30, &pairs16), Ok(()));
        assert_eq!(bus.writes, 5);

        assert_eq!(
            bus.write_reg8_bulk(0x00, &pairs8),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(
            bus.write_reg8_bulk(0x07, &pairs8),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(
            bus.write_reg16_bulk(0x78, &pairs16),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(
            bus.write_reg16_bulk(0x80, &pairs16),
            Err(CameraError::InvalidArg)
        );
    }

    #[test]
    fn test_is_valid_sccb_addr() {
        assert!(!is_valid_sccb_addr(0x00));
        assert!(!is_valid_sccb_addr(0x07));
        assert!(is_valid_sccb_addr(0x08));
        assert!(is_valid_sccb_addr(0x30));
        assert!(is_valid_sccb_addr(0x77));
        assert!(!is_valid_sccb_addr(0x78));
        assert!(!is_valid_sccb_addr(0x7F));
        assert!(!is_valid_sccb_addr(0x80));
        assert!(!is_valid_sccb_addr(0xFF));
    }

    #[test]
    fn test_write_reg_mask_invalid_address() {
        let mut bus = SpyBus {
            reads: 0,
            writes: 0,
            reg_val: 0x55,
        };

        // mask == 0 with invalid address must return InvalidArg (SCCB-002)
        assert_eq!(
            bus.write_reg8_mask(0x00, 0x10, 0xAA, 0x00),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(
            bus.write_reg8_mask(0x07, 0x10, 0xAA, 0x00),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(
            bus.write_reg8_mask(0x78, 0x10, 0xAA, 0x00),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(bus.reads, 0);
        assert_eq!(bus.writes, 0);

        // write_reg16_mask with mask == 0 and invalid address
        assert_eq!(
            bus.write_reg16_mask(0x00, 0x1000, 0xAA, 0x00),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(
            bus.write_reg16_mask(0x07, 0x1000, 0xAA, 0x00),
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
