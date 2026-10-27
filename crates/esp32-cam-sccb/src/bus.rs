//! Hardware-agnostic SCCB / I2C register bus trait for camera sensors.

use esp32_cam_types::CameraError;

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
        let current = self.read_reg8(slv_addr, reg)?;
        let next = (current & !mask) | (val & mask);
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
        let current = self.read_reg16(slv_addr, reg)?;
        let next = (current & !mask) | (val & mask);
        self.write_reg16(slv_addr, reg, next)
    }

    /// Reads a 16-bit big-endian value from a 16-bit register address.
    fn read_reg16_val16(&mut self, slv_addr: u8, reg: u16) -> Result<u16, CameraError> {
        if slv_addr == 0 || slv_addr >= 0x80 {
            return Err(CameraError::InvalidArg);
        }
        let next_reg = reg.checked_add(1).ok_or(CameraError::InvalidArg)?;
        let hi = self.read_reg16(slv_addr, reg)? as u16;
        let lo = self.read_reg16(slv_addr, next_reg)? as u16;
        Ok((hi << 8) | lo)
    }

    /// Writes a 16-bit big-endian value to a 16-bit register address.
    fn write_reg16_val16(&mut self, slv_addr: u8, reg: u16, data: u16) -> Result<(), CameraError> {
        if slv_addr == 0 || slv_addr >= 0x80 {
            return Err(CameraError::InvalidArg);
        }
        let next_reg = reg.checked_add(1).ok_or(CameraError::InvalidArg)?;
        self.write_reg16(slv_addr, reg, (data >> 8) as u8)?;
        self.write_reg16(slv_addr, next_reg, (data & 0xFF) as u8)
    }

    /// Probes whether a device responds at the 7-bit `slv_addr`.
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
