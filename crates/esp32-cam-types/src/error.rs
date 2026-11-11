//! Unified camera driver error types and result definitions.

use core::fmt;

/// SCCB / I2C bus transfer phase where a slave NACK occurred.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SccbPhase {
    /// Slave did not acknowledge its 7-bit device address byte.
    DeviceAddr = 0,
    /// Slave did not acknowledge the register address byte(s).
    RegAddr = 1,
    /// Slave did not acknowledge a data byte during transfer.
    Data = 2,
    /// Error detected while issuing the bus STOP condition.
    Stop = 3,
    /// Hardware did not report which transfer phase NACKed.
    Unknown = 4,
}

impl SccbPhase {
    /// Returns the static identifier string for this SCCB transfer phase.
    #[must_use]
    #[inline]
    pub const fn as_str(&self) -> &'static str {
        match self {
            Self::DeviceAddr => "device_addr",
            Self::RegAddr => "reg_addr",
            Self::Data => "data",
            Self::Stop => "stop",
            Self::Unknown => "unknown",
        }
    }
}

impl fmt::Display for SccbPhase {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// Unified error type for the `esp32-camera` Rust workspace.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CameraError {
    /// Camera sensor was not detected on the SCCB/I2C bus.
    NotDetected,
    /// Failed to configure the requested frame size.
    FailedToSetFrameSize,
    /// Failed to configure the requested pixel format.
    FailedToSetOutFormat,
    /// Operation or control is not supported by this sensor.
    NotSupported,
    /// Invalid argument or out-of-range parameter.
    InvalidArg,
    /// Driver or peripheral is in an invalid state for operation.
    InvalidState,
    /// Insufficient memory or caller buffer capacity.
    NoMem,
    /// Destination buffer is too small to hold the output.
    BufferTooSmall,
    /// Arithmetic overflow detected during calculation.
    Overflow,
    /// Underlying SCCB / I2C bus transaction failed.
    SccbError,
    /// Slave did not acknowledge the transfer on the SCCB/I2C bus.
    SccbNack { slv_addr: u8, phase: SccbPhase },
    /// Underlying I2C error.
    I2cError,
    /// Operation timed out waiting for hardware or sensor status.
    Timeout,
    /// Captured frame or image data is corrupted.
    CorruptedFrame,
    /// Invalid or unsupported image format.
    InvalidFormat,
    /// Image encoder/decoder failure.
    CodecError,
    /// Low-level peripheral or hardware configuration error.
    HardwareError,
    /// DMA descriptor ring is corrupted or misaligned.
    DmaRingCorrupted,
}

const _: () = assert!(core::mem::size_of::<CameraError>() <= 4);

impl CameraError {
    /// Returns a static description string for this error variant.
    #[must_use]
    #[inline]
    pub const fn as_str(&self) -> &'static str {
        match self {
            Self::NotDetected => "camera sensor not detected",
            Self::FailedToSetFrameSize => "failed to set frame size",
            Self::FailedToSetOutFormat => "failed to set output pixel format",
            Self::NotSupported => "operation not supported by sensor",
            Self::InvalidArg => "invalid argument",
            Self::InvalidState => "invalid driver state",
            Self::NoMem => "out of memory",
            Self::BufferTooSmall => "destination buffer too small",
            Self::Overflow => "integer overflow",
            Self::SccbError => "SCCB bus transaction error",
            Self::SccbNack { .. } => "SCCB NACK",
            Self::I2cError => "I2C bus error",
            Self::Timeout => "operation timed out",
            Self::CorruptedFrame => "corrupted frame data",
            Self::InvalidFormat => "invalid pixel or image format",
            Self::CodecError => "image codec error",
            Self::HardwareError => "hardware peripheral error",
            Self::DmaRingCorrupted => "DMA descriptor ring corrupted",
        }
    }
}

impl fmt::Display for CameraError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::SccbNack { slv_addr, phase } => write!(
                f,
                "SCCB NACK on device 0x{slv_addr:02X} during {}",
                phase.as_str()
            ),
            other => f.write_str(other.as_str()),
        }
    }
}

impl core::error::Error for CameraError {}

#[cfg(test)]
mod tests {
    extern crate alloc;

    use alloc::format;
    use core::mem::size_of;

    use super::*;

    #[test]
    fn test_camera_error_size_and_equality() {
        assert!(
            size_of::<SccbPhase>() == 1,
            "SccbPhase must be 1 byte: {}",
            size_of::<SccbPhase>()
        );
        assert!(
            size_of::<CameraError>() <= 4,
            "CameraError must fit in a 32-bit register: {}",
            size_of::<CameraError>()
        );

        let err = CameraError::SccbNack {
            slv_addr: 0x30,
            phase: SccbPhase::DeviceAddr,
        };
        assert_eq!(
            err,
            CameraError::SccbNack {
                slv_addr: 0x30,
                phase: SccbPhase::DeviceAddr,
            }
        );
        assert_ne!(
            err,
            CameraError::SccbNack {
                slv_addr: 0x30,
                phase: SccbPhase::Data,
            }
        );
        assert_ne!(err, CameraError::SccbError);
    }

    #[test]
    fn test_sccb_phase_as_str_and_display() {
        let cases = [
            (SccbPhase::DeviceAddr, "device_addr"),
            (SccbPhase::RegAddr, "reg_addr"),
            (SccbPhase::Data, "data"),
            (SccbPhase::Stop, "stop"),
            (SccbPhase::Unknown, "unknown"),
        ];
        for (phase, expected) in cases {
            assert_eq!(phase.as_str(), expected);
            assert_eq!(format!("{phase}"), expected);
        }

        const CONST_PHASE: &str = SccbPhase::RegAddr.as_str();
        assert_eq!(CONST_PHASE, "reg_addr");
    }

    #[test]
    fn test_camera_error_as_str_and_display() {
        let unit_cases = [
            (CameraError::NotDetected, "camera sensor not detected"),
            (
                CameraError::FailedToSetFrameSize,
                "failed to set frame size",
            ),
            (
                CameraError::FailedToSetOutFormat,
                "failed to set output pixel format",
            ),
            (
                CameraError::NotSupported,
                "operation not supported by sensor",
            ),
            (CameraError::InvalidArg, "invalid argument"),
            (CameraError::InvalidState, "invalid driver state"),
            (CameraError::NoMem, "out of memory"),
            (CameraError::BufferTooSmall, "destination buffer too small"),
            (CameraError::Overflow, "integer overflow"),
            (CameraError::SccbError, "SCCB bus transaction error"),
            (CameraError::I2cError, "I2C bus error"),
            (CameraError::Timeout, "operation timed out"),
            (CameraError::CorruptedFrame, "corrupted frame data"),
            (CameraError::InvalidFormat, "invalid pixel or image format"),
            (CameraError::CodecError, "image codec error"),
            (CameraError::HardwareError, "hardware peripheral error"),
            (
                CameraError::DmaRingCorrupted,
                "DMA descriptor ring corrupted",
            ),
        ];

        for (err, expected) in unit_cases {
            assert_eq!(err.as_str(), expected);
            assert_eq!(format!("{err}"), expected);
        }

        let nack = CameraError::SccbNack {
            slv_addr: 0x3C,
            phase: SccbPhase::DeviceAddr,
        };
        assert_eq!(nack.as_str(), "SCCB NACK");
        assert_eq!(
            format!("{nack}"),
            "SCCB NACK on device 0x3C during device_addr"
        );

        const CONST_ERR: &str = CameraError::Timeout.as_str();
        assert_eq!(CONST_ERR, "operation timed out");
    }
}
