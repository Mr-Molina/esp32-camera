//! `embedded-hal` 1.0 adapter implementing [`SccbBus`] over I2C and Delay.

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;
use esp32_cam_types::CameraError;

use crate::bus::SccbBus;

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
