//! Pure-Rust `#![forbid(unsafe_code)]` I2S0 DMA element unpacking filters and sampling mode
//! selection logic, replacing `target/esp32/ll_cam.c:63-224` and `571-619`.

#![forbid(unsafe_code)]

use esp32_cam_types::{CameraError, CameraPid, PixelFormat};

/// ESP32 I2S0 camera FIFO sampling mode (`i2s_sampling_mode_t` in `target/esp32/ll_cam.c`).
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum I2sSamplingMode {
    /// Camera sends `s1, s2, s3, s4, ...`; FIFO receives `00 s1 00 s2, 00 s2 00 s3, ...` (`SM_0A0B_0B0C = 0`).
    Sm0A0B0B0C = 0,
    /// Camera sends `s1, s2, s3, s4, ...`; FIFO receives `00 s1 00 s2, 00 s3 00 s4, ...` (`SM_0A0B_0C0D = 1`).
    Sm0A0B0C0D = 1,
    /// Camera sends `s1, s2, s3, s4, ...`; FIFO receives `00 s1 00 00, 00 s2 00 00, ...` (`SM_0A00_0B00 = 3`).
    Sm0A000B00 = 3,
}

#[allow(non_upper_case_globals)]
impl I2sSamplingMode {
    pub const SM_0A0B_0B0C: Self = Self::Sm0A0B0B0C;
    pub const SM_0A0B_0C0D: Self = Self::Sm0A0B0C0D;
    pub const SM_0A00_0B00: Self = Self::Sm0A000B00;

    /// Returns the number of DMA buffer bytes consumed per camera sample (`ll_cam_bytes_per_sample`).
    #[inline]
    pub const fn bytes_per_sample(self) -> usize {
        match self {
            Self::Sm0A000B00 => 4,
            Self::Sm0A0B0B0C => 4,
            Self::Sm0A0B0C0D => 2,
        }
    }
}

/// 32-bit I2S0 DMA FIFO element (`dma_elem_t` in `target/esp32/ll_cam.c:63-71`).
///
/// On little-endian targets (Xtensa ESP32 and x86_64 host):
/// - byte `0`: `sample2`
/// - byte `1`: `unused2`
/// - byte `2`: `sample1`
/// - byte `3`: `unused1`
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Hash)]
pub struct DmaElement(pub [u8; 4]);

impl DmaElement {
    /// Constructs a [`DmaElement`] from `sample1` (byte 2) and `sample2` (byte 0).
    #[inline]
    pub const fn new(sample1: u8, sample2: u8) -> Self {
        Self([sample2, 0, sample1, 0])
    }

    /// Constructs a [`DmaElement`] from raw little-endian bytes.
    #[inline]
    pub const fn from_bytes(bytes: [u8; 4]) -> Self {
        Self(bytes)
    }

    /// Constructs a [`DmaElement`] from a 32-bit little-endian word.
    #[inline]
    pub const fn from_u32(val: u32) -> Self {
        Self(val.to_le_bytes())
    }

    /// Returns the underlying 32-bit little-endian word.
    #[inline]
    pub const fn to_u32(self) -> u32 {
        u32::from_le_bytes(self.0)
    }

    /// Returns `sample1` (bits `16..23`, byte index 2).
    #[inline]
    pub const fn sample1(self) -> u8 {
        self.0[2]
    }

    /// Returns `sample2` (bits `0..7`, byte index 0).
    #[inline]
    pub const fn sample2(self) -> u8 {
        self.0[0]
    }

    /// Returns `unused1` (bits `24..31`, byte index 3).
    #[inline]
    pub const fn unused1(self) -> u8 {
        self.0[3]
    }

    /// Returns `unused2` (bits `8..15`, byte index 1).
    #[inline]
    pub const fn unused2(self) -> u8 {
        self.0[1]
    }
}

/// Function pointer signature for DMA buffer unpacking filters (`dma_filter_t`).
pub type DmaFilterFn = fn(&mut [u8], &[u8]) -> usize;

/// Discriminant identifying the active DMA unpacking filter.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum DmaFilterKind {
    Jpeg,
    Grayscale,
    GrayscaleHighspeed,
    Yuyv,
    YuyvHighspeed,
}

impl DmaFilterKind {
    /// Returns the filter function pointer for this [`DmaFilterKind`].
    #[inline]
    pub const fn as_fn(self) -> DmaFilterFn {
        match self {
            Self::Jpeg => filter_jpeg,
            Self::Grayscale => filter_grayscale,
            Self::GrayscaleHighspeed => filter_grayscale_highspeed,
            Self::Yuyv => filter_yuyv,
            Self::YuyvHighspeed => filter_yuyv_highspeed,
        }
    }

    /// Applies this filter directly to `src`, writing unpacked bytes into `dst`.
    #[inline]
    pub fn apply(self, dst: &mut [u8], src: &[u8]) -> usize {
        (self.as_fn())(dst, src)
    }
}

/// Computed I2S0 sampling mode and pixel byte ratios (`ll_cam_set_sample_mode`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SampleModeConfig {
    pub sampling_mode: I2sSamplingMode,
    pub filter: DmaFilterKind,
    pub in_bytes_per_pixel: u8,
    pub fb_bytes_per_pixel: u8,
}

/// Unpacks JPEG DMA elements by extracting `sample1` from each 4-byte word (`ll_cam_dma_filter_jpeg`).
///
/// Unrolls 4 elements per iteration matching `target/esp32/ll_cam.c:107-122`.
#[inline]
pub fn filter_jpeg(dst: &mut [u8], src: &[u8]) -> usize {
    let elements = (src.len() / 4).min(dst.len());
    let src_valid = &src[..elements * 4];
    let dst_valid = &mut dst[..elements];

    let (src_chunks, rem_in) = src_valid.as_chunks::<16>();
    let (dst_chunks, rem_out) = dst_valid.as_chunks_mut::<4>();
    for (c_out, c_in) in dst_chunks.iter_mut().zip(src_chunks.iter()) {
        *c_out = [c_in[2], c_in[6], c_in[10], c_in[14]];
    }

    let (rem_in_words, _) = rem_in.as_chunks::<4>();
    for (b_out, c_in) in rem_out.iter_mut().zip(rem_in_words.iter()) {
        *b_out = c_in[2];
    }

    elements
}

/// Unpacks standard-speed grayscale DMA elements by extracting `sample1` (`ll_cam_dma_filter_grayscale`).
///
/// Matches `target/esp32/ll_cam.c:124-139`.
#[inline]
pub fn filter_grayscale(dst: &mut [u8], src: &[u8]) -> usize {
    filter_jpeg(dst, src)
}

/// Unpacks high-speed grayscale DMA elements (`ll_cam_dma_filter_grayscale_highspeed`).
///
/// Extracts `sample1` from every even-indexed [`DmaElement`] (`0, 2, 4, 6`), with special
/// handling for line remainders (`rem > 0`, `rem >= 3`, `rem >= 5`, and `rem >= 7`),
/// matching `target/esp32/ll_cam.c:141-167`.
#[inline]
pub fn filter_grayscale_highspeed(dst: &mut [u8], src: &[u8]) -> usize {
    let elements = src.len() / 4;
    let end = elements / 8;

    let mut i = 0;
    let mut out_bytes = 0;
    while i < end {
        if out_bytes + 4 > dst.len() {
            break;
        }
        let base_in = i * 32;
        dst[out_bytes] = src[base_in + 2]; // dma_el[0].sample1
        dst[out_bytes + 1] = src[base_in + 10]; // dma_el[2].sample1
        dst[out_bytes + 2] = src[base_in + 18]; // dma_el[4].sample1
        dst[out_bytes + 3] = src[base_in + 26]; // dma_el[6].sample1
        out_bytes += 4;
        i += 1;
    }

    let rem = elements - i * 8;
    let tail_base = i * 32;
    if rem > 0 && out_bytes < dst.len() {
        dst[out_bytes] = src[tail_base + 2]; // dma_el[0].sample1
        out_bytes += 1;
        if rem >= 3 && out_bytes < dst.len() {
            dst[out_bytes] = src[tail_base + 10]; // dma_el[2].sample1
            out_bytes += 1;
            if rem >= 5 && out_bytes < dst.len() {
                dst[out_bytes] = src[tail_base + 18]; // dma_el[4].sample1
                out_bytes += 1;
                if rem >= 7 && out_bytes < dst.len() {
                    dst[out_bytes] = src[tail_base + 26]; // dma_el[6].sample1
                    out_bytes += 1;
                }
            }
        }
    }

    out_bytes
}

/// Unpacks standard-speed YUYV / RGB565 DMA elements (`ll_cam_dma_filter_yuyv`).
///
/// Extracts `[sample1, sample2]` from each [`DmaElement`] (2 output bytes per 4-byte element),
/// matching `target/esp32/ll_cam.c:169-188`.
#[inline]
pub fn filter_yuyv(dst: &mut [u8], src: &[u8]) -> usize {
    let max_elements = dst.len() / 2;
    let elements = (src.len() / 4).min(max_elements);
    let end = elements / 4;

    let mut i = 0;
    while i < end {
        let base_in = i * 16;
        let base_out = i * 8;
        dst[base_out] = src[base_in + 2]; // dma_el[0].sample1 (y0)
        dst[base_out + 1] = src[base_in]; // dma_el[0].sample2 (u)
        dst[base_out + 2] = src[base_in + 6]; // dma_el[1].sample1 (y1)
        dst[base_out + 3] = src[base_in + 4]; // dma_el[1].sample2 (v)

        dst[base_out + 4] = src[base_in + 10]; // dma_el[2].sample1 (y0)
        dst[base_out + 5] = src[base_in + 8]; // dma_el[2].sample2 (u)
        dst[base_out + 6] = src[base_in + 14]; // dma_el[3].sample1 (y1)
        dst[base_out + 7] = src[base_in + 12]; // dma_el[3].sample2 (v)
        i += 1;
    }

    let mut rem_idx = end * 4;
    while rem_idx < elements {
        let base_in = rem_idx * 4;
        let base_out = rem_idx * 2;
        dst[base_out] = src[base_in + 2];
        dst[base_out + 1] = src[base_in];
        rem_idx += 1;
    }

    elements * 2
}

/// Unpacks high-speed YUYV / RGB565 DMA elements (`ll_cam_dma_filter_yuyv_highspeed`).
///
/// Extracts `sample1` from each of 8 consecutive [`DmaElement`]s, with special remainder handling
/// (`rem == 1`, `rem == 2`, `rem == 3` where `dst[3] = dma_el[2].sample2`, and `rem >= 4`
/// where `dst[3] = dma_el[3].sample1`), matching `target/esp32/ll_cam.c:190-224`.
#[inline]
pub fn filter_yuyv_highspeed(dst: &mut [u8], src: &[u8]) -> usize {
    let elements = src.len() / 4;
    let end = elements / 8;

    let mut i = 0;
    let mut out_bytes = 0;
    while i < end {
        if out_bytes + 8 > dst.len() {
            break;
        }
        let base_in = i * 32;
        dst[out_bytes] = src[base_in + 2]; // dma_el[0].sample1 (y0)
        dst[out_bytes + 1] = src[base_in + 6]; // dma_el[1].sample1 (u)
        dst[out_bytes + 2] = src[base_in + 10]; // dma_el[2].sample1 (y1)
        dst[out_bytes + 3] = src[base_in + 14]; // dma_el[3].sample1 (v)
        dst[out_bytes + 4] = src[base_in + 18]; // dma_el[4].sample1 (y0)
        dst[out_bytes + 5] = src[base_in + 22]; // dma_el[5].sample1 (u)
        dst[out_bytes + 6] = src[base_in + 26]; // dma_el[6].sample1 (y1)
        dst[out_bytes + 7] = src[base_in + 30]; // dma_el[7].sample1 (v)
        out_bytes += 8;
        i += 1;
    }

    let rem = elements - i * 8;
    let tail_base = i * 32;
    if rem > 0 && out_bytes < dst.len() {
        dst[out_bytes] = src[tail_base + 2]; // dma_el[0].sample1 (y0)
        out_bytes += 1;
        if rem >= 2 && out_bytes < dst.len() {
            dst[out_bytes] = src[tail_base + 6]; // dma_el[1].sample1 (u)
            out_bytes += 1;
        }
        if rem >= 3 && out_bytes < dst.len() {
            dst[out_bytes] = src[tail_base + 10]; // dma_el[2].sample1 (y1)
            out_bytes += 1;
            if out_bytes < dst.len() {
                dst[out_bytes] = if rem >= 4 {
                    src[tail_base + 14] // dma_el[3].sample1 (v)
                } else {
                    src[tail_base + 8] // dma_el[2].sample2 (v)
                };
                out_bytes += 1;
            }
        }
        if rem >= 5 && out_bytes < dst.len() {
            dst[out_bytes] = src[tail_base + 18]; // dma_el[4].sample1 (y0)
            out_bytes += 1;
            if rem >= 6 && out_bytes < dst.len() {
                dst[out_bytes] = src[tail_base + 22]; // dma_el[5].sample1 (u)
                out_bytes += 1;
            }
            if rem >= 7 && out_bytes < dst.len() {
                dst[out_bytes] = src[tail_base + 26]; // dma_el[6].sample1 (y1)
                out_bytes += 1;
                if out_bytes < dst.len() {
                    dst[out_bytes] = if rem >= 8 {
                        src[tail_base + 30] // dma_el[7].sample1 (v)
                    } else {
                        src[tail_base + 24] // dma_el[6].sample2 (v)
                    };
                    out_bytes += 1;
                }
            }
        }
    }

    out_bytes
}

/// Returns `true` if the sensor outputs native 1-byte Y8 in `PIXFORMAT_GRAYSCALE` mode on ESP32.
#[inline]
const fn is_native_y8_grayscale_sensor(sensor_pid: u16) -> bool {
    sensor_pid == CameraPid::OV3660.as_u16()
        || sensor_pid == CameraPid::OV3640.as_u16()
        || sensor_pid == CameraPid::OV5640.as_u16()
        || sensor_pid == CameraPid::NT99141.as_u16()
        || sensor_pid == CameraPid::SC031GS.as_u16()
        || sensor_pid == CameraPid::BF20A6.as_u16()
        || sensor_pid == CameraPid::GC0308.as_u16()
}

/// Selects the I2S0 sampling mode, DMA filter, and bytes-per-pixel ratios for the given
/// `pix_format`, `xclk_freq_hz`, and `sensor_pid`, matching `ll_cam_set_sample_mode`
/// in `target/esp32/ll_cam.c:571-619`.
pub const fn select_sample_mode(
    pix_format: PixelFormat,
    xclk_freq_hz: u32,
    sensor_pid: u16,
) -> Result<SampleModeConfig, CameraError> {
    match pix_format {
        PixelFormat::Grayscale => {
            if is_native_y8_grayscale_sensor(sensor_pid) {
                let (sampling_mode, filter) = if xclk_freq_hz > 10_000_000 {
                    (I2sSamplingMode::Sm0A000B00, DmaFilterKind::YuyvHighspeed)
                } else {
                    (I2sSamplingMode::Sm0A0B0C0D, DmaFilterKind::Yuyv)
                };
                Ok(SampleModeConfig {
                    sampling_mode,
                    filter,
                    in_bytes_per_pixel: 1,
                    fb_bytes_per_pixel: 1,
                })
            } else {
                let (sampling_mode, filter) =
                    if xclk_freq_hz > 10_000_000 && sensor_pid != CameraPid::OV7725.as_u16() {
                        (
                            I2sSamplingMode::Sm0A000B00,
                            DmaFilterKind::GrayscaleHighspeed,
                        )
                    } else {
                        (I2sSamplingMode::Sm0A0B0C0D, DmaFilterKind::Grayscale)
                    };
                Ok(SampleModeConfig {
                    sampling_mode,
                    filter,
                    in_bytes_per_pixel: 2,
                    fb_bytes_per_pixel: 1,
                })
            }
        }
        PixelFormat::Yuv422 | PixelFormat::Rgb565 => {
            let (sampling_mode, filter) =
                if xclk_freq_hz > 10_000_000 && sensor_pid != CameraPid::OV7725.as_u16() {
                    let mode = if sensor_pid == CameraPid::OV7670.as_u16() {
                        I2sSamplingMode::Sm0A0B0B0C
                    } else {
                        I2sSamplingMode::Sm0A000B00
                    };
                    (mode, DmaFilterKind::YuyvHighspeed)
                } else {
                    (I2sSamplingMode::Sm0A0B0C0D, DmaFilterKind::Yuyv)
                };
            Ok(SampleModeConfig {
                sampling_mode,
                filter,
                in_bytes_per_pixel: 2,
                fb_bytes_per_pixel: 2,
            })
        }
        PixelFormat::Jpeg => Ok(SampleModeConfig {
            sampling_mode: I2sSamplingMode::Sm0A000B00,
            filter: DmaFilterKind::Jpeg,
            in_bytes_per_pixel: 1,
            fb_bytes_per_pixel: 1,
        }),
        _ => Err(CameraError::NotSupported),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dma_element_layout_and_bytes_per_sample() {
        let el = DmaElement::new(0xAA, 0x55);
        assert_eq!(el.0, [0x55, 0x00, 0xAA, 0x00]);
        assert_eq!(el.sample1(), 0xAA);
        assert_eq!(el.sample2(), 0x55);
        assert_eq!(el.unused1(), 0x00);
        assert_eq!(el.unused2(), 0x00);
        assert_eq!(DmaElement::from_u32(el.to_u32()), el);

        assert_eq!(I2sSamplingMode::Sm0A000B00.bytes_per_sample(), 4);
        assert_eq!(I2sSamplingMode::Sm0A0B0B0C.bytes_per_sample(), 4);
        assert_eq!(I2sSamplingMode::Sm0A0B0C0D.bytes_per_sample(), 2);
    }

    #[test]
    fn test_filter_jpeg_and_grayscale() {
        let mut src = [0u8; 32];
        for i in 0..8 {
            let el = DmaElement::new(0x10 + i as u8, 0x80 + i as u8);
            src[i * 4..i * 4 + 4].copy_from_slice(&el.0);
        }
        let mut dst = [0u8; 8];
        let written = filter_jpeg(&mut dst, &src);
        assert_eq!(written, 8);
        assert_eq!(dst, [0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17]);

        let mut dst_gray = [0u8; 8];
        assert_eq!(filter_grayscale(&mut dst_gray, &src), 8);
        assert_eq!(dst_gray, dst);
    }

    #[test]
    fn test_filter_grayscale_highspeed_with_remainders() {
        let mut src = [0u8; 44]; // 11 elements = 8 + 3 remainder
        for i in 0..11 {
            let el = DmaElement::new((i as u8 + 1) * 3, 0xFF);
            src[i * 4..i * 4 + 4].copy_from_slice(&el.0);
        }
        let mut dst = [0u8; 16];
        let n = filter_grayscale_highspeed(&mut dst, &src);
        // 8 elements -> indices 0, 2, 4, 6 -> values 3, 9, 15, 21
        // 3 rem elements -> indices 8, 10 -> values 27, 33
        assert_eq!(n, 6);
        assert_eq!(&dst[..6], &[3, 9, 15, 21, 27, 33]);

        // Remainder 5 (indices 8, 10, 12) and Remainder 7 (indices 8, 10, 12, 14) (LL-009)
        let mut src15 = [0u8; 60]; // 15 elements = 8 + 7 remainder
        for i in 0..15 {
            let el = DmaElement::new((i as u8 + 1) * 2, 0xFF);
            src15[i * 4..i * 4 + 4].copy_from_slice(&el.0);
        }
        let mut dst15 = [0u8; 16];
        // Test rem = 5 with 13 elements (52 bytes)
        let n13 = filter_grayscale_highspeed(&mut dst15, &src15[..52]);
        assert_eq!(n13, 7); // 4 + 3 = 7
        assert_eq!(&dst15[..7], &[2, 6, 10, 14, 18, 22, 26]);

        // Test rem = 7 with 15 elements (60 bytes)
        let n15 = filter_grayscale_highspeed(&mut dst15, &src15);
        assert_eq!(n15, 8); // 4 + 4 = 8
        assert_eq!(&dst15[..8], &[2, 6, 10, 14, 18, 22, 26, 30]);

        // Test loop break with partial dst buffer (LL-009)
        let mut dst_small = [0u8; 6];
        let n_small = filter_grayscale_highspeed(&mut dst_small, &src15);
        assert_eq!(n_small, 6);
        assert_eq!(&dst_small, &[2, 6, 10, 14, 18, 22]);
    }

    #[test]
    fn test_filter_yuyv_and_yuyv_highspeed() {
        let mut src = [0u8; 16]; // 4 elements
        for i in 0..4 {
            let el = DmaElement::new(0x20 + i as u8, 0x30 + i as u8);
            src[i * 4..i * 4 + 4].copy_from_slice(&el.0);
        }
        let mut dst = [0u8; 8];
        let n = filter_yuyv(&mut dst, &src);
        assert_eq!(n, 8);
        assert_eq!(dst, [0x20, 0x30, 0x21, 0x31, 0x22, 0x32, 0x23, 0x33]);

        // High-speed YUYV: 11 elements (8 + 3 remainder)
        let mut hs_src = [0u8; 44];
        for i in 0..11 {
            let el = DmaElement::new(0x40 + i as u8, 0x90 + i as u8);
            hs_src[i * 4..i * 4 + 4].copy_from_slice(&el.0);
        }
        let mut hs_dst = [0u8; 16];
        let hs_n = filter_yuyv_highspeed(&mut hs_dst, &hs_src);
        // First 8 elements -> sample1 of 0..8 -> 0x40..=0x47
        // Remainder 3 elements -> el[8].sample1 (0x48), el[9].sample1 (0x49), el[10].sample1 (0x4A), el[10].sample2 (0x9A)
        assert_eq!(hs_n, 12);
        assert_eq!(
            &hs_dst[..12],
            &[
                0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x9A
            ]
        );

        // High-speed YUYV: 12 elements (8 + 4 remainder) (LL-009)
        let mut hs_src12 = [0u8; 48];
        for i in 0..12 {
            let el = DmaElement::new(0x40 + i as u8, 0x90 + i as u8);
            hs_src12[i * 4..i * 4 + 4].copy_from_slice(&el.0);
        }
        let mut hs_dst12 = [0u8; 16];
        let hs_n12 = filter_yuyv_highspeed(&mut hs_dst12, &hs_src12);
        // Remainder 4 elements -> el[8..11].sample1: 0x48, 0x49, 0x4A, 0x4B
        assert_eq!(hs_n12, 12);
        assert_eq!(
            &hs_dst12[..12],
            &[
                0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x4B
            ]
        );

        // Test loop break with partial dst buffer (LL-009)
        let mut hs_dst_small = [0u8; 10];
        let hs_n_small = filter_yuyv_highspeed(&mut hs_dst_small, &hs_src12);
        assert_eq!(hs_n_small, 10);
        assert_eq!(
            &hs_dst_small,
            &[0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49]
        );
    }

    #[test]
    fn test_select_sample_mode_all_branches() {
        // Grayscale on native Y8 sensor (OV5640) at 20MHz vs 10MHz
        let cfg = select_sample_mode(PixelFormat::Grayscale, 20_000_000, CameraPid::OV5640.as_u16())
            .expect("valid sample mode");
        assert_eq!(cfg.sampling_mode, I2sSamplingMode::Sm0A000B00);
        assert_eq!(cfg.filter, DmaFilterKind::YuyvHighspeed);
        assert_eq!(cfg.in_bytes_per_pixel, 1);
        assert_eq!(cfg.fb_bytes_per_pixel, 1);

        let cfg_slow =
            select_sample_mode(PixelFormat::Grayscale, 10_000_000, CameraPid::OV5640.as_u16())
                .expect("valid sample mode");
        assert_eq!(cfg_slow.sampling_mode, I2sSamplingMode::Sm0A0B0C0D);
        assert_eq!(cfg_slow.filter, DmaFilterKind::Yuyv);

        // Grayscale on YU/YV sensor (OV2640 vs OV7725) at 20MHz
        let cfg_ov2640 =
            select_sample_mode(PixelFormat::Grayscale, 20_000_000, CameraPid::OV2640.as_u16())
                .expect("valid sample mode");
        assert_eq!(cfg_ov2640.sampling_mode, I2sSamplingMode::Sm0A000B00);
        assert_eq!(cfg_ov2640.filter, DmaFilterKind::GrayscaleHighspeed);
        assert_eq!(cfg_ov2640.in_bytes_per_pixel, 2);
        assert_eq!(cfg_ov2640.fb_bytes_per_pixel, 1);

        let cfg_ov7725 =
            select_sample_mode(PixelFormat::Grayscale, 20_000_000, CameraPid::OV7725.as_u16())
                .expect("valid sample mode");
        assert_eq!(cfg_ov7725.sampling_mode, I2sSamplingMode::Sm0A0B0C0D);
        assert_eq!(cfg_ov7725.filter, DmaFilterKind::Grayscale);

        // RGB565 on OV7670 at 20MHz uses Sm0A0B0B0C
        let cfg_ov7670 =
            select_sample_mode(PixelFormat::Rgb565, 20_000_000, CameraPid::OV7670.as_u16())
                .expect("valid sample mode");
        assert_eq!(cfg_ov7670.sampling_mode, I2sSamplingMode::Sm0A0B0B0C);
        assert_eq!(cfg_ov7670.filter, DmaFilterKind::YuyvHighspeed);

        // JPEG mode
        let cfg_jpeg = select_sample_mode(PixelFormat::Jpeg, 20_000_000, CameraPid::OV2640.as_u16())
            .expect("valid sample mode");
        assert_eq!(cfg_jpeg.sampling_mode, I2sSamplingMode::Sm0A000B00);
        assert_eq!(cfg_jpeg.filter, DmaFilterKind::Jpeg);
        assert_eq!(cfg_jpeg.in_bytes_per_pixel, 1);
        assert_eq!(cfg_jpeg.fb_bytes_per_pixel, 1);

        // Unsupported format
        assert_eq!(
            select_sample_mode(PixelFormat::Rgb888, 20_000_000, CameraPid::OV2640.as_u16()),
            Err(CameraError::NotSupported)
        );
    }
}
