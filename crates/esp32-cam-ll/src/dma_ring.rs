//! ESP32 ROM `lldesc_t` DMA descriptor layout, circular ring initialization, and RGB/JPEG
//! DMA buffer sizing algorithms (`target/esp32/ll_cam.c:470-559`, `driver/cam_hal.c:465-482`).

use core::ptr;
use esp32_cam_types::CameraError;

use crate::dma_filter::I2sSamplingMode;

/// Maximum buffer capacity per single ESP32/S2/S3 DMA descriptor node (`LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE`).
pub const LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE: usize = 4092;

/// Default maximum DMA ping-pong buffer size in bytes (`CONFIG_CAMERA_DMA_BUFFER_SIZE_MAX`).
pub const DEFAULT_CAMERA_DMA_BUFFER_SIZE_MAX: usize = 32768;

/// Maximum supported uncompressed image size in bytes (`4 MiB`, `ll_cam.c:481`).
pub const MAX_DMA_IMAGE_SIZE: usize = 4 * 1024 * 1024;

/// Hardware DMA linked-list descriptor matching ESP32/ESP32-S2/ESP32-S3 ROM `lldesc_t`.
///
/// Bitfield packing within `size_length_flags` (little-endian 32-bit word):
/// - bits `0..=11` (`12` bits): `size` (buffer capacity, max `4095`)
/// - bits `12..=23` (`12` bits): `length` (transferred byte count, max `4095`)
/// - bits `24..=28` (`5` bits): `offset` (software/reserved offset, max `31`)
/// - bit `29` (`1` bit): `sosf` (start of sub-frame)
/// - bit `30` (`1` bit): `eof` (end of frame)
/// - bit `31` (`1` bit): `owner` (`1` = DMA controller, `0` = CPU software)
#[repr(C, align(4))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Lldesc {
    pub size_length_flags: u32,
    pub buf: *const u8,
    pub qe_stqe_next: *const Lldesc,
}

impl Default for Lldesc {
    fn default() -> Self {
        Self::empty()
    }
}

impl Lldesc {
    /// Bit mask for the 12-bit `size` field (`bits 0..=11`).
    pub const SIZE_MASK: u32 = 0x0000_0FFF;
    /// Bit shift for the 12-bit `length` field (`bits 12..=23`).
    pub const LENGTH_SHIFT: u32 = 12;
    /// Bit mask for the 12-bit `length` field (`bits 12..=23`).
    pub const LENGTH_MASK: u32 = 0x00FF_F000;
    /// Bit shift for the 5-bit `offset` field (`bits 24..=28`).
    pub const OFFSET_SHIFT: u32 = 24;
    /// Bit mask for the 5-bit `offset` field (`bits 24..=28`).
    pub const OFFSET_MASK: u32 = 0x1F00_0000;
    /// Bit flag for `sosf` (`bit 29`).
    pub const SOSF_BIT: u32 = 1 << 29;
    /// Bit flag for `eof` (`bit 30`).
    pub const EOF_BIT: u32 = 1 << 30;
    /// Bit flag for `owner` (`bit 31`, `1` = DMA hardware).
    pub const OWNER_BIT: u32 = 1 << 31;

    /// Packs `size`, `length`, `offset`, `sosf`, `eof`, and `owner` into the 32-bit header word.
    #[inline]
    pub const fn pack_flags(
        size: u16,
        length: u16,
        offset: u8,
        sosf: bool,
        eof: bool,
        owner: bool,
    ) -> u32 {
        let s = if size > 0x0FFF { 0x0FFF } else { size as u32 };
        let l = (if length > 0x0FFF { 0x0FFF } else { length as u32 }) << Self::LENGTH_SHIFT;
        let o = (if offset > 0x1F { 0x1F } else { offset as u32 }) << Self::OFFSET_SHIFT;
        let sf = if sosf { Self::SOSF_BIT } else { 0 };
        let ef = if eof { Self::EOF_BIT } else { 0 };
        let ow = if owner { Self::OWNER_BIT } else { 0 };
        s | l | o | sf | ef | ow
    }

    /// Creates a zeroed descriptor with null `buf` and `qe_stqe_next` pointers.
    #[inline]
    pub const fn empty() -> Self {
        Self {
            size_length_flags: 0,
            buf: ptr::null(),
            qe_stqe_next: ptr::null(),
        }
    }

    /// Returns the 12-bit `size` field (`0..=4095`).
    #[inline]
    pub const fn size(&self) -> u16 {
        (self.size_length_flags & Self::SIZE_MASK) as u16
    }

    /// Sets the 12-bit `size` field (`0..=4095`), saturating at `4095`.
    #[inline]
    pub fn set_size(&mut self, size: u16) {
        let s = size.min(0x0FFF) as u32;
        self.size_length_flags = (self.size_length_flags & !Self::SIZE_MASK) | s;
    }

    /// Returns the 12-bit `length` field (`0..=4095`).
    #[inline]
    pub const fn length(&self) -> u16 {
        ((self.size_length_flags & Self::LENGTH_MASK) >> Self::LENGTH_SHIFT) as u16
    }

    /// Sets the 12-bit `length` field (`0..=4095`), saturating at `4095`.
    #[inline]
    pub fn set_length(&mut self, length: u16) {
        let l = (length.min(0x0FFF) as u32) << Self::LENGTH_SHIFT;
        self.size_length_flags = (self.size_length_flags & !Self::LENGTH_MASK) | l;
    }

    /// Returns the 5-bit `offset` field (`0..=31`).
    #[inline]
    pub const fn offset(&self) -> u8 {
        ((self.size_length_flags & Self::OFFSET_MASK) >> Self::OFFSET_SHIFT) as u8
    }

    /// Sets the 5-bit `offset` field (`0..=31`), saturating at `31`.
    #[inline]
    pub fn set_offset(&mut self, offset: u8) {
        let o = (offset.min(0x1F) as u32) << Self::OFFSET_SHIFT;
        self.size_length_flags = (self.size_length_flags & !Self::OFFSET_MASK) | o;
    }

    /// Returns the `sosf` (start of sub-frame) flag (`bit 29`).
    #[inline]
    pub const fn sosf(&self) -> bool {
        (self.size_length_flags & Self::SOSF_BIT) != 0
    }

    /// Sets the `sosf` (start of sub-frame) flag (`bit 29`).
    #[inline]
    pub fn set_sosf(&mut self, sosf: bool) {
        if sosf {
            self.size_length_flags |= Self::SOSF_BIT;
        } else {
            self.size_length_flags &= !Self::SOSF_BIT;
        }
    }

    /// Returns the `eof` (end of frame) flag (`bit 30`).
    #[inline]
    pub const fn eof(&self) -> bool {
        (self.size_length_flags & Self::EOF_BIT) != 0
    }

    /// Sets the `eof` (end of frame) flag (`bit 30`).
    #[inline]
    pub fn set_eof(&mut self, eof: bool) {
        if eof {
            self.size_length_flags |= Self::EOF_BIT;
        } else {
            self.size_length_flags &= !Self::EOF_BIT;
        }
    }

    /// Returns the `owner` flag (`bit 31`, `true` = owned by DMA hardware).
    #[inline]
    pub const fn owner(&self) -> bool {
        (self.size_length_flags & Self::OWNER_BIT) != 0
    }

    /// Sets the `owner` flag (`bit 31`, `true` = owned by DMA hardware).
    #[inline]
    pub fn set_owner(&mut self, owner: bool) {
        if owner {
            self.size_length_flags |= Self::OWNER_BIT;
        } else {
            self.size_length_flags &= !Self::OWNER_BIT;
        }
    }

    /// Returns the 20-bit physical descriptor address mask (`addr & 0xFFFFF`) used by `I2S0.in_link.addr`
    /// and `GDMA.channel[n].in.link.addr`.
    #[inline]
    pub fn link_addr_20bit(&self) -> u32 {
        (ptr::from_ref(self) as usize as u32) & 0x000F_FFFF
    }
}

/// Computed DMA buffer, half-buffer, and descriptor node dimensions (`cam_obj_t` DMA fields).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DmaSizeConfig {
    pub dma_bytes_per_item: usize,
    pub dma_buffer_size: usize,
    pub dma_half_buffer_size: usize,
    pub dma_half_buffer_cnt: usize,
    pub dma_node_buffer_size: usize,
    pub dma_node_cnt: usize,
    pub lines_per_node: usize,
    pub nodes_per_line: usize,
    pub lines_per_half_buffer: usize,
    pub image_size: usize,
}

/// Computes RGB/YUV/Grayscale DMA node, half-buffer, and full-buffer sizes (`ll_cam_calc_rgb_dma`
/// in `target/esp32/ll_cam.c:470-545`), enforcing zero-division and 4 MiB bounds (`DMA-101`..`105`).
pub fn calc_rgb_dma_sizes(
    width: u16,
    height: u16,
    in_bytes_per_pixel: u8,
    dma_bytes_per_item: usize,
    dma_buffer_size_max: usize,
) -> Result<DmaSizeConfig, CameraError> {
    if width == 0
        || height == 0
        || in_bytes_per_pixel == 0
        || dma_bytes_per_item == 0
        || dma_buffer_size_max == 0
    {
        return Err(CameraError::InvalidArg);
    }

    let width_usize = width as usize;
    let height_usize = height as usize;
    let bpp_usize = in_bytes_per_pixel as usize;

    let dma_half_buffer_max = dma_buffer_size_max / 2 / dma_bytes_per_item;
    let dma_buffer_max = 2 * dma_half_buffer_max;
    let node_max = LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE / dma_bytes_per_item;

    if dma_half_buffer_max == 0 || node_max == 0 {
        return Err(CameraError::InvalidArg);
    }

    let line_width = width_usize
        .checked_mul(bpp_usize)
        .ok_or(CameraError::Overflow)?;
    let image_size = height_usize
        .checked_mul(line_width)
        .ok_or(CameraError::Overflow)?;

    if image_size > MAX_DMA_IMAGE_SIZE || line_width > dma_half_buffer_max {
        return Err(CameraError::InvalidArg);
    }

    let mut best_node_samples = 0usize;
    let mut nodes_per_line = 1usize;
    let mut lines_per_node = 1usize;

    if line_width >= node_max {
        let mut d = node_max;
        while d > 0 {
            if (d * dma_bytes_per_item).is_multiple_of(4) && line_width.is_multiple_of(d) {
                best_node_samples = d;
                nodes_per_line = line_width / best_node_samples;
                break;
            }
            d -= 1;
        }
    } else {
        let mut d = node_max;
        while d > 0 {
            if (d * dma_bytes_per_item).is_multiple_of(4) && d.is_multiple_of(line_width) {
                let mut lpn = d / line_width;
                while lpn > 1 && !height_usize.is_multiple_of(lpn) {
                    lpn -= 1;
                }
                let cand_samples = lpn * line_width;
                if (cand_samples * dma_bytes_per_item).is_multiple_of(4) {
                    best_node_samples = cand_samples;
                    lines_per_node = lpn;
                    break;
                }
            }
            d -= 1;
        }
    }

    if best_node_samples == 0 {
        return Err(CameraError::InvalidArg);
    }
    let node_size = best_node_samples;

    let dma_half_buffer_min = node_size
        .checked_mul(nodes_per_line)
        .ok_or(CameraError::Overflow)?;
    if dma_half_buffer_min == 0 {
        return Err(CameraError::InvalidArg);
    }

    let mut dma_half_buffer = (dma_half_buffer_max / dma_half_buffer_min) * dma_half_buffer_min;
    let mut lines_per_half_buffer = dma_half_buffer / line_width;

    while lines_per_half_buffer > 1
        && dma_half_buffer >= dma_half_buffer_min + line_width
        && !height_usize.is_multiple_of(lines_per_half_buffer)
    {
        dma_half_buffer -= dma_half_buffer_min;
        lines_per_half_buffer = dma_half_buffer / line_width;
    }

    if dma_half_buffer == 0 {
        return Err(CameraError::InvalidArg);
    }

    let dma_buffer_items = (dma_buffer_max / dma_half_buffer) * dma_half_buffer;
    if dma_buffer_items == 0 {
        return Err(CameraError::InvalidArg);
    }

    let dma_buffer_size = dma_buffer_items
        .checked_mul(dma_bytes_per_item)
        .ok_or(CameraError::Overflow)?;
    let dma_half_buffer_size = dma_half_buffer
        .checked_mul(dma_bytes_per_item)
        .ok_or(CameraError::Overflow)?;
    let dma_node_buffer_size = node_size
        .checked_mul(dma_bytes_per_item)
        .ok_or(CameraError::Overflow)?;

    if dma_half_buffer_size == 0 || dma_node_buffer_size == 0 {
        return Err(CameraError::InvalidArg);
    }

    let dma_half_buffer_cnt = dma_buffer_size / dma_half_buffer_size;
    let dma_node_cnt = dma_buffer_size / dma_node_buffer_size;

    Ok(DmaSizeConfig {
        dma_bytes_per_item,
        dma_buffer_size,
        dma_half_buffer_size,
        dma_half_buffer_cnt,
        dma_node_buffer_size,
        dma_node_cnt,
        lines_per_node,
        nodes_per_line,
        lines_per_half_buffer,
        image_size,
    })
}

/// Computes DMA buffer geometry for either JPEG or uncompressed modes on ESP32 (`ll_cam_dma_sizes`
/// in `target/esp32/ll_cam.c:547-559`).
pub fn calc_dma_sizes(
    jpeg_mode: bool,
    sampling_mode: I2sSamplingMode,
    width: u16,
    height: u16,
    in_bytes_per_pixel: u8,
    dma_buffer_size_max: usize,
) -> Result<DmaSizeConfig, CameraError> {
    if width == 0 || height == 0 || dma_buffer_size_max < 4096 {
        return Err(CameraError::InvalidArg);
    }
    let dma_bytes_per_item = sampling_mode.bytes_per_sample();
    if jpeg_mode {
        let dma_half_buffer_cnt = 8;
        let dma_node_buffer_size = 2048;
        let dma_half_buffer_size = (dma_node_buffer_size * 2).min(dma_buffer_size_max / 2);
        let dma_buffer_size = dma_half_buffer_cnt * dma_half_buffer_size;
        let dma_node_cnt = dma_buffer_size / dma_node_buffer_size;
        let image_size = (width as usize).saturating_mul(height as usize) / 5;
        Ok(DmaSizeConfig {
            dma_bytes_per_item,
            dma_buffer_size,
            dma_half_buffer_size,
            dma_half_buffer_cnt,
            dma_node_buffer_size,
            dma_node_cnt,
            lines_per_node: 1,
            nodes_per_line: 1,
            lines_per_half_buffer: 1,
            image_size,
        })
    } else {
        calc_rgb_dma_sizes(
            width,
            height,
            in_bytes_per_pixel,
            dma_bytes_per_item,
            dma_buffer_size_max,
        )
    }
}

/// Initializes a circular linked-list ring of [`Lldesc`] descriptors over `buffer`, matching
/// `allocate_dma_descriptors` in `driver/cam_hal.c:465-482`.
///
/// Validates that:
/// - `descriptors` is non-empty,
/// - `node_size` is non-zero, `<= LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE` (`4092`), and 4-byte aligned,
/// - `buffer.len() >= descriptors.len() * node_size`.
pub fn build_circular_ring(
    descriptors: &mut [Lldesc],
    buffer: &[u8],
    node_size: usize,
) -> Result<(), CameraError> {
    let count = descriptors.len();
    if count == 0
        || node_size == 0
        || node_size > LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE
        || !node_size.is_multiple_of(4)
    {
        return Err(CameraError::InvalidArg);
    }

    let required_bytes = count.checked_mul(node_size).ok_or(CameraError::Overflow)?;
    if buffer.len() < required_bytes {
        return Err(CameraError::BufferTooSmall);
    }

    let buf_base = buffer.as_ptr();
    let desc_base = descriptors.as_ptr();

    let mut x = 0;
    while x < count {
        let next_idx = (x + 1) % count;
        let byte_offset = x * node_size;

        // SAFETY:
        // 1. Alignment & Non-null: `buf_base` is derived from a valid slice `buffer: &[u8]` (1-byte aligned)
        //    and `desc_base` is derived from `descriptors: &mut [Lldesc]` (4-byte aligned via `#[repr(C, align(4))]`).
        // 2. Memory layout: `Lldesc` is `#[repr(C, align(4))]` matching hardware `lldesc_t`.
        // 3. Provenance & Bounds: `byte_offset = x * node_size < count * node_size <= buffer.len()`, and
        //    `next_idx < count == descriptors.len()`, so both `.add(...)` operations stay strictly within
        //    their respective allocated objects.
        // 4. Aliasing: Raw pointers `chunk_ptr` and `next_desc_ptr` are stored passively in `Lldesc` fields
        //    for hardware DMA consumption and are not dereferenced while `descriptors` is mutably borrowed.
        let (chunk_ptr, next_desc_ptr) = unsafe {
            (buf_base.add(byte_offset), desc_base.add(next_idx))
        };

        descriptors[x] = Lldesc {
            size_length_flags: Lldesc::pack_flags(node_size as u16, 0, 0, false, false, true),
            buf: chunk_ptr,
            qe_stqe_next: next_desc_ptr,
        };
        x += 1;
    }

    Ok(())
}

/// Verifies that a slice of [`Lldesc`] descriptors forms a valid circular ring over `buffer`
/// with each node having `size == expected_node_size`, `length == 0`, `owner == true`, `eof == false`,
/// `sosf == false`, and `offset == 0`.
pub fn verify_circular_ring(
    descriptors: &[Lldesc],
    buffer: &[u8],
    expected_node_size: usize,
) -> Result<(), CameraError> {
    let count = descriptors.len();
    if count == 0
        || expected_node_size == 0
        || expected_node_size > LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE
        || !expected_node_size.is_multiple_of(4)
    {
        return Err(CameraError::InvalidArg);
    }
    let required = count
        .checked_mul(expected_node_size)
        .ok_or(CameraError::Overflow)?;
    if buffer.len() < required {
        return Err(CameraError::BufferTooSmall);
    }

    let buf_base = buffer.as_ptr();
    let desc_base = descriptors.as_ptr();

    let mut x = 0;
    while x < count {
        let d = &descriptors[x];
        if d.size() as usize != expected_node_size
            || d.length() != 0
            || !d.owner()
            || d.eof()
            || d.sosf()
            || d.offset() != 0
        {
            return Err(CameraError::InvalidState);
        }
        let next_idx = (x + 1) % count;

        // SAFETY:
        // 1. Alignment & Non-null: `buf_base` and `desc_base` come from live slice references.
        // 2. Provenance & Bounds: `x * expected_node_size < required <= buffer.len()` and `next_idx < count`,
        //    so pointer arithmetic remains within the allocated slice bounds.
        let (expected_buf, expected_next) = unsafe {
            (
                buf_base.add(x * expected_node_size),
                desc_base.add(next_idx),
            )
        };

        if !ptr::eq(d.buf, expected_buf) || !ptr::eq(d.qe_stqe_next, expected_next) {
            return Err(CameraError::InvalidState);
        }
        x += 1;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::mem::align_of;

    #[test]
    fn test_lldesc_bitfield_packing_and_alignment() {
        assert!(align_of::<Lldesc>() >= 4);

        let mut desc = Lldesc::empty();
        desc.set_size(2048);
        desc.set_length(1024);
        desc.set_offset(17);
        desc.set_sosf(true);
        desc.set_eof(false);
        desc.set_owner(true);

        assert_eq!(desc.size(), 2048);
        assert_eq!(desc.length(), 1024);
        assert_eq!(desc.offset(), 17);
        assert!(desc.sosf());
        assert!(!desc.eof());
        assert!(desc.owner());

        desc.set_eof(true);
        desc.set_owner(false);
        assert!(desc.eof());
        assert!(!desc.owner());

        // Saturation tests for size, length, and offset (LL-006)
        desc.set_size(5000);
        assert_eq!(desc.size(), 4095);
        desc.set_length(5000);
        assert_eq!(desc.length(), 4095);
        desc.set_offset(50);
        assert_eq!(desc.offset(), 31);

        let saturated_flags = Lldesc::pack_flags(6000, 7000, 100, true, true, true);
        assert_eq!(saturated_flags & Lldesc::SIZE_MASK, 0x0FFF);
        assert_eq!(
            (saturated_flags & Lldesc::LENGTH_MASK) >> Lldesc::LENGTH_SHIFT,
            0x0FFF
        );
        assert_eq!(
            (saturated_flags & Lldesc::OFFSET_MASK) >> Lldesc::OFFSET_SHIFT,
            0x1F
        );
    }

    #[test]
    fn test_circular_ring_builder_and_verifier() {
        let buf = [0u8; 4096];
        let mut ring = [Lldesc::empty(); 4];

        assert_eq!(build_circular_ring(&mut ring, &buf, 1024), Ok(()));
        assert_eq!(verify_circular_ring(&ring, &buf, 1024), Ok(()));

        // Verify circular wrap-around from last descriptor back to index 0
        assert!(ptr::eq(ring[3].qe_stqe_next, &ring[0]));
        assert!(ptr::eq(ring[0].buf, buf.as_ptr()));

        // Reject invalid parameters (LL-007)
        assert_eq!(
            build_circular_ring(&mut ring, &buf, 0),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(
            build_circular_ring(&mut ring, &buf, 4093),
            Err(CameraError::InvalidArg)
        );
        // Reject non-multiple of 4
        assert_eq!(
            build_circular_ring(&mut ring, &buf, 1023),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(
            verify_circular_ring(&ring, &buf, 1023),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(
            build_circular_ring(&mut ring, &buf[..2048], 1024),
            Err(CameraError::BufferTooSmall)
        );

        // Reject corrupt descriptor flags in verifier
        ring[1].set_eof(true);
        assert_eq!(
            verify_circular_ring(&ring, &buf, 1024),
            Err(CameraError::InvalidState)
        );
        ring[1].set_eof(false);
        ring[2].set_sosf(true);
        assert_eq!(
            verify_circular_ring(&ring, &buf, 1024),
            Err(CameraError::InvalidState)
        );
        ring[2].set_sosf(false);
        ring[0].set_offset(1);
        assert_eq!(
            verify_circular_ring(&ring, &buf, 1024),
            Err(CameraError::InvalidState)
        );
    }

    #[test]
    fn test_calc_dma_sizes_jpeg_and_rgb_and_guards() {
        // JPEG mode
        let jpeg_cfg = calc_dma_sizes(
            true,
            I2sSamplingMode::Sm0A000B00,
            640,
            480,
            1,
            DEFAULT_CAMERA_DMA_BUFFER_SIZE_MAX,
        )
        .expect("valid jpeg dma config");
        assert_eq!(jpeg_cfg.dma_half_buffer_cnt, 8);
        assert_eq!(jpeg_cfg.dma_node_buffer_size, 2048);
        assert_eq!(jpeg_cfg.dma_half_buffer_size, 4096);
        assert_eq!(jpeg_cfg.dma_buffer_size, 32768);
        assert_eq!(jpeg_cfg.dma_node_cnt, 16);

        // JPEG clamping to dma_buffer_size_max / 2 (LL-008)
        let jpeg_clamped = calc_dma_sizes(
            true,
            I2sSamplingMode::Sm0A000B00,
            640,
            480,
            1,
            4096,
        )
        .expect("valid clamped jpeg config");
        assert_eq!(jpeg_clamped.dma_half_buffer_size, 2048);
        assert_eq!(jpeg_clamped.dma_buffer_size, 8 * 2048);

        // Reject width == 0, height == 0, or dma_buffer_size_max < 4096 in JPEG mode
        assert_eq!(
            calc_dma_sizes(true, I2sSamplingMode::Sm0A000B00, 0, 480, 1, 32768),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(
            calc_dma_sizes(true, I2sSamplingMode::Sm0A000B00, 640, 0, 1, 32768),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(
            calc_dma_sizes(true, I2sSamplingMode::Sm0A000B00, 640, 480, 1, 4095),
            Err(CameraError::InvalidArg)
        );

        // QVGA RGB565 (320x240, 2 bpp, 2 bytes per item in Sm0A0B0C0D)
        let qvga_cfg = calc_dma_sizes(
            false,
            I2sSamplingMode::Sm0A0B0C0D,
            320,
            240,
            2,
            DEFAULT_CAMERA_DMA_BUFFER_SIZE_MAX,
        )
        .expect("valid qvga rgb565 dma config");
        assert_eq!(qvga_cfg.image_size, 320 * 240 * 2);
        assert!(qvga_cfg.dma_node_buffer_size <= LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE);
        assert_eq!(qvga_cfg.dma_buffer_size % qvga_cfg.dma_half_buffer_size, 0);
        assert_eq!(qvga_cfg.dma_buffer_size % qvga_cfg.dma_node_buffer_size, 0);

        // Zero-division guards (DMA-101..105)
        assert_eq!(
            calc_rgb_dma_sizes(0, 240, 2, 2, DEFAULT_CAMERA_DMA_BUFFER_SIZE_MAX),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(
            calc_rgb_dma_sizes(320, 0, 2, 2, DEFAULT_CAMERA_DMA_BUFFER_SIZE_MAX),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(
            calc_rgb_dma_sizes(320, 240, 0, 2, DEFAULT_CAMERA_DMA_BUFFER_SIZE_MAX),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(
            calc_rgb_dma_sizes(320, 240, 2, 0, DEFAULT_CAMERA_DMA_BUFFER_SIZE_MAX),
            Err(CameraError::InvalidArg)
        );

        // Exceeds 4 MiB guard
        assert_eq!(
            calc_rgb_dma_sizes(2592, 1944, 2, 2, DEFAULT_CAMERA_DMA_BUFFER_SIZE_MAX),
            Err(CameraError::InvalidArg)
        );
    }

    #[test]
    fn test_calc_rgb_dma_sizes_enforces_4_byte_aligned_node_buffer() {
        // When width * bpp and height cannot form any 4-byte-aligned DMA node buffer,
        // calc_rgb_dma_sizes must reject with InvalidArg instead of producing an unaligned node_size (LOGIC-008).
        // e.g. width = 3, height = 1, in_bytes_per_pixel = 1, dma_bytes_per_item = 1 (line_width = 3, height = 1)
        assert_eq!(
            calc_rgb_dma_sizes(3, 1, 1, 1, 8192),
            Err(CameraError::InvalidArg)
        );

        // e.g. line_width >= node_max with prime unaligned width (e.g. 4093)
        assert_eq!(
            calc_rgb_dma_sizes(4093, 1, 1, 1, 32768),
            Err(CameraError::InvalidArg)
        );

        // When a valid 4-byte-aligned configuration is provided, dma_node_buffer_size must be a multiple of 4
        // and build_circular_ring must accept it.
        let cfg = calc_rgb_dma_sizes(4, 1, 1, 1, 8192).expect("valid aligned rgb dma config");
        assert_eq!(cfg.dma_node_buffer_size % 4, 0);

        let mut ring = [Lldesc::empty(); 2];
        let buf = [0u8; 8192];
        assert_eq!(
            build_circular_ring(
                &mut ring,
                &buf[..2 * cfg.dma_node_buffer_size],
                cfg.dma_node_buffer_size
            ),
            Ok(())
        );
    }
}
