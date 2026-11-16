//! Application image parsing, checksumming, and SHA-256 verification.
//!
//! Provides `#![no_std]` zero-heap verification of ESP32 `0xE9` application
//! images from SPI flash, enforcing bootloader-reserved SRAM isolation.

use crate::rom_boundary::AlignedBounce64;
use crate::rom_boundary::FlashError;
use crate::rom_boundary::FlashIo;

/// Magic byte identifying the start of an ESP32 application image (`0xE9`).
pub const ESP_IMAGE_HEADER_MAGIC: u8 = 0xE9;

/// Magic byte seed for the streaming XOR checksum (`0xEF`).
pub const ESP_CHECKSUM_MAGIC: u8 = 0xEF;

/// Maximum number of segments allowed in a single application image.
pub const MAX_SEGMENTS: usize = 16;

/// Start of bootloader-reserved DRAM (inclusive).
pub const BOOTLOADER_DRAM_START: u32 = 0x3FFF_0000;

/// End of bootloader-reserved DRAM (exclusive).
pub const BOOTLOADER_DRAM_END: u32 = 0x3FFF_8000;

/// Start of bootloader-reserved IRAM (inclusive).
pub const BOOTLOADER_IRAM_START: u32 = 0x4007_8000;

/// End of bootloader-reserved IRAM (exclusive).
pub const BOOTLOADER_IRAM_END: u32 = 0x4008_0000;

/// Lower bound of memory-mapped flash DROM virtual address space.
pub const SOC_DROM_LOW: u32 = 0x3F40_0000;

/// Upper bound of memory-mapped flash DROM virtual address space.
pub const SOC_DROM_HIGH: u32 = 0x3F80_0000;

/// Lower bound of memory-mapped flash IROM virtual address space.
pub const SOC_IROM_LOW: u32 = 0x400D_0000;

/// Upper bound of memory-mapped flash IROM virtual address space.
pub const SOC_IROM_HIGH: u32 = 0x4040_0000;

/// SHA-256 initial hash values (FIPS 180-4).
const H0: [u32; 8] = [
    0x6a09_e667,
    0xbb67_ae85,
    0x3c6e_f372,
    0xa54f_f53a,
    0x510e_527f,
    0x9b05_688c,
    0x1f83_d9ab,
    0x5be0_cd19,
];

/// SHA-256 round constants (FIPS 180-4).
const K: [u32; 64] = [
    0x428a_2f98,
    0x7137_4491,
    0xb5c0_fbcf,
    0xe9b5_dba5,
    0x3956_c25b,
    0x59f1_11f1,
    0x923f_82a4,
    0xab1c_5ed5,
    0xd807_aa98,
    0x1283_5b01,
    0x2431_85be,
    0x550c_7dc3,
    0x72be_5d74,
    0x80de_b1fe,
    0x9bdc_06a7,
    0xc19b_f174,
    0xe49b_69c1,
    0xefbe_4786,
    0x0fc1_9dc6,
    0x240c_a1cc,
    0x2de9_2c6f,
    0x4a74_84aa,
    0x5cb0_a9dc,
    0x76f9_88da,
    0x983e_5152,
    0xa831_c66d,
    0xb003_27c8,
    0xbf59_7fc7,
    0xc6e0_0bf3,
    0xd5a7_9147,
    0x06ca_6351,
    0x1429_2967,
    0x27b7_0a85,
    0x2e1b_2138,
    0x4d2c_6dfc,
    0x5338_0d13,
    0x650a_7354,
    0x766a_0abb,
    0x81c2_c92e,
    0x9272_2c85,
    0xa2bf_e8a1,
    0xa81a_664b,
    0xc24b_8b70,
    0xc76c_51a3,
    0xd192_e819,
    0xd699_0624,
    0xf40e_3585,
    0x106a_a070,
    0x19a4_c116,
    0x1e37_6c08,
    0x2748_774c,
    0x34b0_bcb5,
    0x391c_0cb3,
    0x4ed8_aa4a,
    0x5b9c_ca4f,
    0x682e_6ff3,
    0x748f_82ee,
    0x78a5_636f,
    0x84c8_7814,
    0x8cc7_0208,
    0x90be_fffa,
    0xa450_6ceb,
    0xbef9_a3f7,
    0xc671_78f2,
];

#[inline]
const fn ch(x: u32, y: u32, z: u32) -> u32 {
    (x & y) ^ (!x & z)
}

#[inline]
const fn maj(x: u32, y: u32, z: u32) -> u32 {
    (x & y) ^ (x & z) ^ (y & z)
}

#[inline]
const fn big_sigma0(x: u32) -> u32 {
    x.rotate_right(2) ^ x.rotate_right(13) ^ x.rotate_right(22)
}

#[inline]
const fn big_sigma1(x: u32) -> u32 {
    x.rotate_right(6) ^ x.rotate_right(11) ^ x.rotate_right(25)
}

#[inline]
const fn small_sigma0(x: u32) -> u32 {
    x.rotate_right(7) ^ x.rotate_right(18) ^ (x >> 3)
}

#[inline]
const fn small_sigma1(x: u32) -> u32 {
    x.rotate_right(17) ^ x.rotate_right(19) ^ (x >> 10)
}

/// Pure `#![no_std]` streaming SHA-256 state machine.
#[derive(Debug, Clone)]
pub struct Sha256 {
    state: [u32; 8],
    block: [u8; 64],
    block_len: usize,
    total_len: u64,
}

impl Sha256 {
    /// Creates a new SHA-256 instance with standard initial state.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            state: H0,
            block: [0u8; 64],
            block_len: 0,
            total_len: 0,
        }
    }

    /// Feeds arbitrary data bytes into the hasher.
    pub fn update(&mut self, data: &[u8]) {
        self.total_len = self.total_len.saturating_add(data.len() as u64);
        let mut offset = 0;
        while offset < data.len() {
            let available = 64 - self.block_len;
            let to_copy = core::cmp::min(available, data.len() - offset);
            self.block[self.block_len..self.block_len + to_copy]
                .copy_from_slice(&data[offset..offset + to_copy]);
            self.block_len += to_copy;
            offset += to_copy;
            if self.block_len == 64 {
                self.process_block();
                self.block_len = 0;
            }
        }
    }

    /// Finalizes the SHA-256 calculation and returns the 32-byte digest.
    #[must_use]
    pub fn finalize(mut self) -> [u8; 32] {
        let bit_len = self.total_len.wrapping_mul(8);
        self.block[self.block_len] = 0x80;
        self.block_len += 1;
        if self.block_len > 56 {
            for b in &mut self.block[self.block_len..64] {
                *b = 0;
            }
            self.process_block();
            self.block_len = 0;
        }
        for b in &mut self.block[self.block_len..56] {
            *b = 0;
        }
        self.block[56..64].copy_from_slice(&bit_len.to_be_bytes());
        self.process_block();

        let mut out = [0u8; 32];
        for (i, word) in self.state.iter().enumerate() {
            out[i * 4..(i + 1) * 4].copy_from_slice(&word.to_be_bytes());
        }
        out
    }

    fn process_block(&mut self) {
        let mut w = [0u32; 64];
        for (i, word) in w.iter_mut().enumerate().take(16) {
            let offset = i * 4;
            *word = u32::from_be_bytes([
                self.block[offset],
                self.block[offset + 1],
                self.block[offset + 2],
                self.block[offset + 3],
            ]);
        }
        for i in 16..64 {
            let s0 = small_sigma0(w[i - 15]);
            let s1 = small_sigma1(w[i - 2]);
            w[i] = w[i - 16]
                .wrapping_add(s0)
                .wrapping_add(w[i - 7])
                .wrapping_add(s1);
        }

        let mut a = self.state[0];
        let mut b = self.state[1];
        let mut c = self.state[2];
        let mut d = self.state[3];
        let mut e = self.state[4];
        let mut f = self.state[5];
        let mut g = self.state[6];
        let mut h = self.state[7];

        for i in 0..64 {
            let t1 = h
                .wrapping_add(big_sigma1(e))
                .wrapping_add(ch(e, f, g))
                .wrapping_add(K[i])
                .wrapping_add(w[i]);
            let t2 = big_sigma0(a).wrapping_add(maj(a, b, c));
            h = g;
            g = f;
            f = e;
            e = d.wrapping_add(t1);
            d = c;
            c = b;
            b = a;
            a = t1.wrapping_add(t2);
        }

        self.state[0] = self.state[0].wrapping_add(a);
        self.state[1] = self.state[1].wrapping_add(b);
        self.state[2] = self.state[2].wrapping_add(c);
        self.state[3] = self.state[3].wrapping_add(d);
        self.state[4] = self.state[4].wrapping_add(e);
        self.state[5] = self.state[5].wrapping_add(f);
        self.state[6] = self.state[6].wrapping_add(g);
        self.state[7] = self.state[7].wrapping_add(h);
    }
}

impl Default for Sha256 {
    fn default() -> Self {
        Self::new()
    }
}

/// Target memory classification for an application segment.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SegmentKind {
    /// Flash data mapped into DROM virtual address space.
    DromMap,
    /// Flash instructions mapped into IROM virtual address space.
    IromMap,
    /// RAM segment loaded directly into target SRAM memory.
    RamLoad,
    /// RTC memory segment preserved or skipped during standard load.
    RtcSkip,
}

/// Metadata describing a validated segment within an application image.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VerifiedSegment {
    /// Offset within flash where segment payload begins.
    pub flash_offset: u32,
    /// Target load address in memory.
    pub load_addr: u32,
    /// Length of segment data in bytes.
    pub data_len: u32,
    /// Target memory classification.
    pub kind: SegmentKind,
}

/// Summary of an entire verified application image ready for booting.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct VerifiedImage {
    /// Entry point address in memory where CPU begins execution.
    pub entry_addr: u32,
    /// Number of valid segments in the image.
    pub segment_count: usize,
    /// Verified segment descriptors.
    pub segments: [Option<VerifiedSegment>; MAX_SEGMENTS],
    /// Flash offset immediately following the entire image.
    pub image_end_offset: u32,
}

/// Errors encountered during application image verification.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImageVerifyError {
    /// Flash I/O error occurred while reading image.
    Flash(FlashError),
    /// Header magic byte mismatch (expected `0xE9`).
    BadMagic,
    /// Segment count outside valid range `1..=16`.
    InvalidSegmentCount,
    /// Segment length is not 4-byte aligned.
    UnalignedSegmentLength,
    /// Segment target address overlaps bootloader-reserved SRAM.
    BootloaderOverlap,
    /// Image or segment exceeds partition boundary.
    OutOfBounds,
    /// Segment XOR checksum mismatch.
    ChecksumMismatch,
    /// Appended SHA-256 digest mismatch.
    HashMismatch,
}

impl From<FlashError> for ImageVerifyError {
    fn from(err: FlashError) -> Self {
        Self::Flash(err)
    }
}

fn classify_and_check_overlap(
    load_addr: u32,
    data_len: u32,
) -> Result<SegmentKind, ImageVerifyError> {
    let seg_end = load_addr
        .checked_add(data_len)
        .ok_or(ImageVerifyError::OutOfBounds)?;

    // MAP_PADDING segments (load_addr == 0) align flash mappings and are
    // hashed/checksummed without writing to RAM address zero.
    if load_addr == 0 {
        return Ok(SegmentKind::RtcSkip);
    }

    // Check DROM
    if load_addr >= SOC_DROM_LOW && seg_end <= SOC_DROM_HIGH {
        return Ok(SegmentKind::DromMap);
    }
    // Check IROM
    if load_addr >= SOC_IROM_LOW && seg_end <= SOC_IROM_HIGH {
        return Ok(SegmentKind::IromMap);
    }
    // Check RTC
    if (load_addr >= 0x3FF8_0000 && seg_end <= 0x3FF8_2000)
        || (load_addr >= 0x400C_0000 && seg_end <= 0x400C_2000)
        || (load_addr >= 0x5000_0000 && seg_end <= 0x5000_2000)
    {
        return Ok(SegmentKind::RtcSkip);
    }

    // Check Bootloader SRAM Overlap:
    let overlaps_dram = load_addr < BOOTLOADER_DRAM_END && seg_end > BOOTLOADER_DRAM_START;
    let overlaps_iram = load_addr < BOOTLOADER_IRAM_END && seg_end > BOOTLOADER_IRAM_START;
    if overlaps_dram || overlaps_iram {
        return Err(ImageVerifyError::BootloaderOverlap);
    }

    Ok(SegmentKind::RamLoad)
}

/// Verifies an ESP32 application image from flash, validating headers,
/// checksum, optional SHA-256, and bootloader memory isolation.
#[allow(clippy::manual_is_multiple_of)]
pub fn verify_app_image<F: FlashIo>(
    flash: &mut F,
    part_offset: u32,
    part_size: u32,
) -> Result<VerifiedImage, ImageVerifyError> {
    if part_size < 24 {
        return Err(ImageVerifyError::OutOfBounds);
    }

    let mut bounce = AlignedBounce64::new();
    flash.read(part_offset, &mut bounce.bytes[..24])?;
    let header = &bounce.bytes[..24];

    if header[0] != ESP_IMAGE_HEADER_MAGIC {
        return Err(ImageVerifyError::BadMagic);
    }

    let segment_count = header[1] as usize;
    if segment_count == 0 || segment_count > MAX_SEGMENTS {
        return Err(ImageVerifyError::InvalidSegmentCount);
    }

    let entry_addr = u32::from_le_bytes([header[4], header[5], header[6], header[7]]);
    let hash_appended = header[23] == 1;

    let mut sha256 = Sha256::new();
    sha256.update(header);
    let mut xor_checksum: u8 = ESP_CHECKSUM_MAGIC;

    let mut segments: [Option<VerifiedSegment>; MAX_SEGMENTS] = [None; MAX_SEGMENTS];
    let mut rel_cursor: u32 = 24;

    for slot in segments.iter_mut().take(segment_count) {
        let seg_hdr_end = rel_cursor
            .checked_add(8)
            .ok_or(ImageVerifyError::OutOfBounds)?;
        if seg_hdr_end > part_size {
            return Err(ImageVerifyError::OutOfBounds);
        }

        let hdr_flash_addr = part_offset
            .checked_add(rel_cursor)
            .ok_or(ImageVerifyError::OutOfBounds)?;
        flash.read(hdr_flash_addr, &mut bounce.bytes[..8])?;
        sha256.update(&bounce.bytes[..8]);

        let load_addr = u32::from_le_bytes([
            bounce.bytes[0],
            bounce.bytes[1],
            bounce.bytes[2],
            bounce.bytes[3],
        ]);
        let data_len = u32::from_le_bytes([
            bounce.bytes[4],
            bounce.bytes[5],
            bounce.bytes[6],
            bounce.bytes[7],
        ]);
        rel_cursor = seg_hdr_end;

        if data_len % 4 != 0 {
            return Err(ImageVerifyError::UnalignedSegmentLength);
        }

        let seg_payload_end = rel_cursor
            .checked_add(data_len)
            .ok_or(ImageVerifyError::OutOfBounds)?;
        if seg_payload_end > part_size {
            return Err(ImageVerifyError::OutOfBounds);
        }

        let kind = classify_and_check_overlap(load_addr, data_len)?;
        let flash_offset = part_offset
            .checked_add(rel_cursor)
            .ok_or(ImageVerifyError::OutOfBounds)?;
        *slot = Some(VerifiedSegment {
            flash_offset,
            load_addr,
            data_len,
            kind,
        });

        let mut remaining = data_len;
        while remaining > 0 {
            let chunk_size = core::cmp::min(remaining as usize, bounce.bytes.len());
            let chunk_addr = part_offset
                .checked_add(rel_cursor)
                .ok_or(ImageVerifyError::OutOfBounds)?;
            flash.read(chunk_addr, &mut bounce.bytes[..chunk_size])?;
            let chunk = &bounce.bytes[..chunk_size];
            for &b in chunk {
                xor_checksum ^= b;
            }
            sha256.update(chunk);
            rel_cursor = rel_cursor
                .checked_add(chunk_size as u32)
                .ok_or(ImageVerifyError::OutOfBounds)?;
            remaining -= chunk_size as u32;
        }
    }

    let target_end = (rel_cursor.saturating_add(16)) & !15;
    let pad_len = target_end.saturating_sub(rel_cursor);
    if target_end > part_size {
        return Err(ImageVerifyError::OutOfBounds);
    }

    let pad_flash_addr = part_offset
        .checked_add(rel_cursor)
        .ok_or(ImageVerifyError::OutOfBounds)?;
    flash.read(pad_flash_addr, &mut bounce.bytes[..pad_len as usize])?;
    let pad_bytes = &bounce.bytes[..pad_len as usize];

    let actual_checksum = pad_bytes[pad_len as usize - 1];
    if actual_checksum != xor_checksum {
        return Err(ImageVerifyError::ChecksumMismatch);
    }

    sha256.update(pad_bytes);
    rel_cursor = target_end;

    if hash_appended {
        let hash_end = rel_cursor
            .checked_add(32)
            .ok_or(ImageVerifyError::OutOfBounds)?;
        if hash_end > part_size {
            return Err(ImageVerifyError::OutOfBounds);
        }
        let hash_flash_addr = part_offset
            .checked_add(rel_cursor)
            .ok_or(ImageVerifyError::OutOfBounds)?;
        flash.read(hash_flash_addr, &mut bounce.bytes[..32])?;
        let expected_hash = &bounce.bytes[..32];
        let computed_hash = sha256.finalize();
        if computed_hash != expected_hash {
            return Err(ImageVerifyError::HashMismatch);
        }
        rel_cursor = hash_end;
    }

    let image_end_offset = part_offset
        .checked_add(rel_cursor)
        .ok_or(ImageVerifyError::OutOfBounds)?;

    Ok(VerifiedImage {
        entry_addr,
        segment_count,
        segments,
        image_end_offset,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rom_boundary::MockFlash;

    #[test]
    fn test_sha256_nist_vector_abc() {
        let mut hasher = Sha256::new();
        hasher.update(b"abc");
        let digest = hasher.finalize();
        let expected = [
            0xba, 0x78, 0x16, 0xbf, 0x8f, 0x01, 0xcf, 0xea, 0x41, 0x41, 0x40, 0xde, 0x5d, 0xae,
            0x22, 0x23, 0xb0, 0x03, 0x61, 0xa3, 0x96, 0x17, 0x7a, 0x9c, 0xb4, 0x10, 0xff, 0x61,
            0xf2, 0x00, 0x15, 0xad,
        ];
        assert_eq!(digest, expected);
    }

    fn build_test_image(
        entry_addr: u32,
        hash_appended: bool,
        segments: &[(u32, &[u8])],
    ) -> ([u8; 512], usize) {
        let mut buf = [0u8; 512];
        buf[0] = ESP_IMAGE_HEADER_MAGIC;
        buf[1] = segments.len() as u8;
        buf[4..8].copy_from_slice(&entry_addr.to_le_bytes());
        buf[23] = if hash_appended { 1 } else { 0 };

        let mut sha = Sha256::new();
        sha.update(&buf[..24]);
        let mut xor_chk = ESP_CHECKSUM_MAGIC;
        let mut cursor = 24;

        for &(load_addr, payload) in segments {
            buf[cursor..cursor + 4].copy_from_slice(&load_addr.to_le_bytes());
            buf[cursor + 4..cursor + 8].copy_from_slice(&(payload.len() as u32).to_le_bytes());
            sha.update(&buf[cursor..cursor + 8]);
            cursor += 8;

            buf[cursor..cursor + payload.len()].copy_from_slice(payload);
            for &b in payload {
                xor_chk ^= b;
            }
            sha.update(payload);
            cursor += payload.len();
        }

        let target_end = (cursor + 16) & !15;
        let pad_len = target_end - cursor;
        for i in 0..pad_len - 1 {
            buf[cursor + i] = 0;
        }
        buf[target_end - 1] = xor_chk;
        sha.update(&buf[cursor..target_end]);
        cursor = target_end;

        if hash_appended {
            let digest = sha.finalize();
            buf[cursor..cursor + 32].copy_from_slice(&digest);
            cursor += 32;
        }

        (buf, cursor)
    }

    #[test]
    fn test_valid_image_with_hash() {
        let drom_payload = [0x11u8; 64];
        let iram_payload = [0x22u8; 64];
        let dram_payload = [0x33u8; 64];

        let (image_bytes, total_len) = build_test_image(
            0x4008_0000,
            true,
            &[
                (0x3F40_0020, &drom_payload),
                (0x4008_0000, &iram_payload),
                (0x3FFB_0000, &dram_payload),
            ],
        );

        let mut flash = MockFlash::<2>::new();
        flash
            .write_raw(0, &image_bytes[..total_len])
            .expect("write_raw failed");

        let verified = verify_app_image(&mut flash, 0, 8192).expect("verification should succeed");
        assert_eq!(verified.entry_addr, 0x4008_0000);
        assert_eq!(verified.segment_count, 3);
        assert_eq!(verified.image_end_offset, total_len as u32);

        let s0 = verified.segments[0].expect("seg 0 present");
        assert_eq!(s0.kind, SegmentKind::DromMap);
        assert_eq!(s0.load_addr, 0x3F40_0020);
        assert_eq!(s0.data_len, 64);
        assert_eq!(s0.flash_offset, 32);

        let s1 = verified.segments[1].expect("seg 1 present");
        assert_eq!(s1.kind, SegmentKind::RamLoad);
        assert_eq!(s1.load_addr, 0x4008_0000);
        assert_eq!(s1.data_len, 64);
        assert_eq!(s1.flash_offset, 104);

        let s2 = verified.segments[2].expect("seg 2 present");
        assert_eq!(s2.kind, SegmentKind::RamLoad);
        assert_eq!(s2.load_addr, 0x3FFB_0000);
        assert_eq!(s2.data_len, 64);
        assert_eq!(s2.flash_offset, 176);
    }

    #[test]
    fn test_corrupted_payload_checksum_mismatch() {
        let drom_payload = [0x11u8; 64];
        let (mut image_bytes, total_len) =
            build_test_image(0x4008_0000, true, &[(0x3F40_0020, &drom_payload)]);

        // Corrupt one payload byte
        image_bytes[35] ^= 0xFF;

        let mut flash = MockFlash::<2>::new();
        flash
            .write_raw(0, &image_bytes[..total_len])
            .expect("write_raw failed");

        let err = verify_app_image(&mut flash, 0, 8192).expect_err("should fail");
        assert_eq!(err, ImageVerifyError::ChecksumMismatch);
    }

    #[test]
    fn test_corrupted_hash_mismatch() {
        let drom_payload = [0x11u8; 64];
        let (mut image_bytes, total_len) =
            build_test_image(0x4008_0000, true, &[(0x3F40_0020, &drom_payload)]);

        // Corrupt the last byte of the SHA-256 hash
        image_bytes[total_len - 1] ^= 0xFF;

        let mut flash = MockFlash::<2>::new();
        flash
            .write_raw(0, &image_bytes[..total_len])
            .expect("write_raw failed");

        let err = verify_app_image(&mut flash, 0, 8192).expect_err("should fail");
        assert_eq!(err, ImageVerifyError::HashMismatch);
    }

    #[test]
    fn test_bootloader_overlap_rejection() {
        let payload = [0xAAu8; 64];

        // Case 1: Overlaps Bootloader IRAM (0x4007_8000..0x4008_0000)
        let (image1, len1) = build_test_image(0x4008_0000, false, &[(0x4007_8100, &payload)]);
        let mut flash1 = MockFlash::<2>::new();
        flash1
            .write_raw(0, &image1[..len1])
            .expect("write_raw failed");
        let err1 = verify_app_image(&mut flash1, 0, 8192).expect_err("should reject IRAM overlap");
        assert_eq!(err1, ImageVerifyError::BootloaderOverlap);

        // Case 2: Overlaps Bootloader DRAM (0x3FFF_0000..0x3FFF_8000)
        let (image2, len2) = build_test_image(0x4008_0000, false, &[(0x3FFF_0100, &payload)]);
        let mut flash2 = MockFlash::<2>::new();
        flash2
            .write_raw(0, &image2[..len2])
            .expect("write_raw failed");
        let err2 = verify_app_image(&mut flash2, 0, 8192).expect_err("should reject DRAM overlap");
        assert_eq!(err2, ImageVerifyError::BootloaderOverlap);
    }

    #[test]
    fn test_padding_segment_returns_rtc_skip() {
        let pad_payload = [0x00u8; 64];
        let drom_payload = [0x11u8; 64];

        let (image_bytes, total_len) = build_test_image(
            0x4008_0000,
            true,
            &[(0, &pad_payload), (0x3F40_0020, &drom_payload)],
        );

        let mut flash = MockFlash::<2>::new();
        flash
            .write_raw(0, &image_bytes[..total_len])
            .expect("write_raw failed");

        let verified = verify_app_image(&mut flash, 0, 8192).expect("verification should succeed");
        assert_eq!(verified.segment_count, 2);

        let s0 = verified.segments[0].expect("seg 0 present");
        assert_eq!(s0.kind, SegmentKind::RtcSkip);
        assert_eq!(s0.load_addr, 0);
        assert_eq!(s0.data_len, 64);
        assert_eq!(s0.flash_offset, 32);

        let s1 = verified.segments[1].expect("seg 1 present");
        assert_eq!(s1.kind, SegmentKind::DromMap);
        assert_eq!(s1.load_addr, 0x3F40_0020);
        assert_eq!(s1.data_len, 64);
        assert_eq!(s1.flash_offset, 104);
    }
}
