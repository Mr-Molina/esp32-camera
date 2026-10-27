//! Unified camera driver error types and result definitions.

use core::fmt;

/// Unified error type for the `esp32-camera` Rust workspace.
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
    SccbNack { slv_addr: u8, phase: &'static str },
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

impl fmt::Display for CameraError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NotDetected => write!(f, "camera sensor not detected"),
            Self::FailedToSetFrameSize => write!(f, "failed to set frame size"),
            Self::FailedToSetOutFormat => {
                write!(f, "failed to set output pixel format")
            }
            Self::NotSupported => write!(f, "operation not supported by sensor"),
            Self::InvalidArg => write!(f, "invalid argument"),
            Self::InvalidState => write!(f, "invalid driver state"),
            Self::NoMem => write!(f, "out of memory"),
            Self::BufferTooSmall => write!(f, "destination buffer too small"),
            Self::Overflow => write!(f, "integer overflow"),
            Self::SccbError => write!(f, "SCCB bus transaction error"),
            Self::SccbNack { slv_addr, phase } => {
                write!(f, "SCCB NACK on device 0x{slv_addr:02X} during {phase}")
            }
            Self::I2cError => write!(f, "I2C bus error"),
            Self::Timeout => write!(f, "operation timed out"),
            Self::CorruptedFrame => write!(f, "corrupted frame data"),
            Self::InvalidFormat => write!(f, "invalid pixel or image format"),
            Self::CodecError => write!(f, "image codec error"),
            Self::HardwareError => write!(f, "hardware peripheral error"),
            Self::DmaRingCorrupted => write!(f, "DMA descriptor ring corrupted"),
        }
    }
}

impl core::error::Error for CameraError {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_camera_error_equality_and_nack() {
        let err = CameraError::SccbNack {
            slv_addr: 0x30,
            phase: "device_addr",
        };
        assert_eq!(
            err,
            CameraError::SccbNack {
                slv_addr: 0x30,
                phase: "device_addr",
            }
        );
        assert_ne!(err, CameraError::SccbError);
    }
}
