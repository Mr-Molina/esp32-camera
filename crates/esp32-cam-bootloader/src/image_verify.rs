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

/// Start of SRAM1 IRAM alias mapping bootloader DRAM (inclusive).
pub const BOOTLOADER_DRAM_IRAM_ALIAS_START: u32 = 0x400A_8000;

/// End of SRAM1 IRAM alias mapping bootloader DRAM (exclusive).
pub const BOOTLOADER_DRAM_IRAM_ALIAS_END: u32 = 0x400B_0000;

/// Lower bound of allowlisted application DRAM0 (inclusive).
pub const SOC_APP_DRAM_LOW: u32 = 0x3FFA_E000;

/// Upper bound of allowlisted application DRAM0 (exclusive).
pub const SOC_APP_DRAM_HIGH: u32 = BOOTLOADER_DRAM_START;

/// Lower bound of allowlisted application IRAM0 (inclusive).
pub const SOC_APP_IRAM_LOW: u32 = BOOTLOADER_IRAM_END;

/// Upper bound of allowlisted application IRAM0 (exclusive).
pub const SOC_APP_IRAM_HIGH: u32 = 0x400A_0000;

/// Lower bound of memory-mapped flash DROM virtual address space.
pub const SOC_DROM_LOW: u32 = 0x3F40_0000;

/// Upper bound of memory-mapped flash DROM virtual address space.
pub const SOC_DROM_HIGH: u32 = 0x3F80_0000;

/// Lower bound of memory-mapped flash IROM virtual address space.
pub const SOC_IROM_LOW: u32 = 0x400D_0000;

/// Upper bound of memory-mapped flash IROM virtual address space.
pub const SOC_IROM_HIGH: u32 = 0x4040_0000;

/// Lower bound of RTC fast DRAM (inclusive).
pub const SOC_RTC_DRAM_LOW: u32 = 0x3FF8_0000;

/// Upper bound of RTC fast DRAM (exclusive).
pub const SOC_RTC_DRAM_HIGH: u32 = 0x3FF8_2000;

/// Lower bound of RTC fast IRAM (inclusive).
pub const SOC_RTC_IRAM_LOW: u32 = 0x400C_0000;

/// Upper bound of RTC fast IRAM (exclusive).
pub const SOC_RTC_IRAM_HIGH: u32 = 0x400C_2000;

/// Lower bound of RTC slow RAM (inclusive).
pub const SOC_RTC_SLOW_LOW: u32 = 0x5000_0000;

/// Upper bound of RTC slow RAM (exclusive).
pub const SOC_RTC_SLOW_HIGH: u32 = 0x5000_2000;

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
    flash_offset: u32,
    load_addr: u32,
    data_len: u32,
    kind: SegmentKind,
}

impl VerifiedSegment {
    /// Returns the offset within flash where the segment payload begins.
    #[inline]
    #[must_use]
    pub const fn flash_offset(&self) -> u32 {
        self.flash_offset
    }

    /// Returns the target load address in memory.
    #[inline]
    #[must_use]
    pub const fn load_addr(&self) -> u32 {
        self.load_addr
    }

    /// Returns the length of segment data in bytes.
    #[inline]
    #[must_use]
    pub const fn data_len(&self) -> u32 {
        self.data_len
    }

    /// Returns the target memory classification.
    #[inline]
    #[must_use]
    pub const fn kind(&self) -> SegmentKind {
        self.kind
    }

    #[cfg(test)]
    #[must_use]
    pub(crate) const fn new_for_test(
        flash_offset: u32,
        load_addr: u32,
        data_len: u32,
        kind: SegmentKind,
    ) -> Self {
        Self {
            flash_offset,
            load_addr,
            data_len,
            kind,
        }
    }
}

/// Summary of an entire verified application image ready for booting.
///
/// Fields are private so callers cannot forge or mutate an already-verified
/// image descriptor outside [`verify_app_image`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct VerifiedImage {
    entry_addr: u32,
    segment_count: usize,
    segments: [Option<VerifiedSegment>; MAX_SEGMENTS],
    image_end_offset: u32,
}

impl VerifiedImage {
    /// Returns the verified entry point address in executable memory.
    #[inline]
    #[must_use]
    pub const fn entry_addr(&self) -> u32 {
        self.entry_addr
    }

    /// Returns the number of valid segments in the image.
    #[inline]
    #[must_use]
    pub const fn segment_count(&self) -> usize {
        self.segment_count
    }

    /// Returns the verified segment descriptors.
    #[inline]
    #[must_use]
    pub const fn segments(&self) -> &[Option<VerifiedSegment>; MAX_SEGMENTS] {
        &self.segments
    }

    /// Returns the flash offset immediately following the entire image.
    #[inline]
    #[must_use]
    pub const fn image_end_offset(&self) -> u32 {
        self.image_end_offset
    }

    #[cfg(test)]
    #[must_use]
    pub(crate) const fn new_for_test(
        entry_addr: u32,
        segment_count: usize,
        segments: [Option<VerifiedSegment>; MAX_SEGMENTS],
        image_end_offset: u32,
    ) -> Self {
        Self {
            entry_addr,
            segment_count,
            segments,
            image_end_offset,
        }
    }
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
    /// Segment load address is not 4-byte aligned or fails MMU page parity.
    UnalignedLoadAddress,
    /// Segment target address overlaps bootloader-reserved SRAM or its alias.
    BootloaderOverlap,
    /// Two segments overlap in target RAM or 64 KiB MMU virtual page space.
    SegmentOverlap,
    /// Image or segment exceeds partition or allowlisted memory bounds.
    OutOfBounds,
    /// Segment XOR checksum mismatch.
    ChecksumMismatch,
    /// Image header does not mandate appended SHA-256 (`header[23] != 1`).
    MissingHash,
    /// Appended SHA-256 digest mismatch.
    HashMismatch,
    /// Non-zero alignment padding or non-erased trailing unverified bytes.
    TrailingBytes,
    /// Entry address is zero or outside verified executable segments.
    InvalidEntryAddress,
}

impl From<FlashError> for ImageVerifyError {
    fn from(err: FlashError) -> Self {
        Self::Flash(err)
    }
}

#[inline]
const fn span_within(start: u32, end: u32, low: u32, high: u32) -> bool {
    if start == end {
        start >= low && start < high
    } else {
        start >= low && end <= high
    }
}

#[inline]
const fn span_overlaps(start: u32, end: u32, low: u32, high: u32) -> bool {
    if start == end {
        start >= low && start < high
    } else {
        start < high && end > low
    }
}

/// Returns `true` if `[addr, addr + len)` lies strictly inside allowlisted
/// ESP32 application DRAM0 or IRAM0, outside bootloader SRAM and MMIO.
#[inline]
#[must_use]
pub const fn is_valid_app_ram_span(addr: u32, len: u32) -> bool {
    let Some(end) = addr.checked_add(len) else {
        return false;
    };
    span_within(addr, end, SOC_APP_DRAM_LOW, SOC_APP_DRAM_HIGH)
        || span_within(addr, end, SOC_APP_IRAM_LOW, SOC_APP_IRAM_HIGH)
}

/// Returns `true` if `addr` is a non-zero address inside allowlisted
/// executable ESP32 memory (application IRAM0 or flash-mapped IROM).
#[inline]
#[must_use]
pub const fn is_valid_executable_addr(addr: u32) -> bool {
    if addr == 0 {
        return false;
    }
    (addr >= SOC_APP_IRAM_LOW && addr < SOC_APP_IRAM_HIGH)
        || (addr >= SOC_IROM_LOW && addr < SOC_IROM_HIGH)
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

    if (load_addr & 3) != 0 {
        return Err(ImageVerifyError::UnalignedLoadAddress);
    }

    // Reject overlap with bootloader SRAM and its SRAM1 IRAM alias first so
    // callers distinguish bootloader isolation faults from generic OOB.
    if span_overlaps(
        load_addr,
        seg_end,
        BOOTLOADER_DRAM_START,
        BOOTLOADER_DRAM_END,
    ) || span_overlaps(
        load_addr,
        seg_end,
        BOOTLOADER_IRAM_START,
        BOOTLOADER_IRAM_END,
    ) || span_overlaps(
        load_addr,
        seg_end,
        BOOTLOADER_DRAM_IRAM_ALIAS_START,
        BOOTLOADER_DRAM_IRAM_ALIAS_END,
    ) {
        return Err(ImageVerifyError::BootloaderOverlap);
    }

    // Strict allowlist: DROM
    if span_within(load_addr, seg_end, SOC_DROM_LOW, SOC_DROM_HIGH) {
        return Ok(SegmentKind::DromMap);
    }
    // Strict allowlist: IROM
    if span_within(load_addr, seg_end, SOC_IROM_LOW, SOC_IROM_HIGH) {
        return Ok(SegmentKind::IromMap);
    }
    // Strict allowlist: RTC fast/slow RAM
    if span_within(load_addr, seg_end, SOC_RTC_DRAM_LOW, SOC_RTC_DRAM_HIGH)
        || span_within(load_addr, seg_end, SOC_RTC_IRAM_LOW, SOC_RTC_IRAM_HIGH)
        || span_within(load_addr, seg_end, SOC_RTC_SLOW_LOW, SOC_RTC_SLOW_HIGH)
    {
        return Ok(SegmentKind::RtcSkip);
    }
    // Strict allowlist: application DRAM0 and IRAM0 SRAM
    if is_valid_app_ram_span(load_addr, data_len) {
        return Ok(SegmentKind::RamLoad);
    }

    Err(ImageVerifyError::OutOfBounds)
}

#[inline]
const fn mmu_page_vspan(load_addr: u32, data_len: u32) -> Option<(u64, u64)> {
    const PAGE_SIZE: u64 = 0x1_0000;
    const PAGE_MASK: u64 = PAGE_SIZE - 1;
    let v_aligned = (load_addr as u64) & !PAGE_MASK;
    if data_len == 0 {
        return Some((v_aligned, v_aligned));
    }
    let off = (load_addr as u64) & PAGE_MASK;
    let pages = (off + (data_len as u64) + PAGE_MASK) >> 16;
    Some((v_aligned, v_aligned + pages * PAGE_SIZE))
}

fn check_segment_pairwise_overlap(
    existing: &[Option<VerifiedSegment>],
    candidate: &VerifiedSegment,
) -> Result<(), ImageVerifyError> {
    let cand_end = candidate
        .load_addr
        .checked_add(candidate.data_len)
        .ok_or(ImageVerifyError::OutOfBounds)?;

    for prev in existing.iter().flatten() {
        match (prev.kind, candidate.kind) {
            (SegmentKind::RamLoad, SegmentKind::RamLoad) => {
                let prev_end = prev
                    .load_addr
                    .checked_add(prev.data_len)
                    .ok_or(ImageVerifyError::OutOfBounds)?;
                if span_overlaps(candidate.load_addr, cand_end, prev.load_addr, prev_end) {
                    return Err(ImageVerifyError::SegmentOverlap);
                }
            }
            (
                SegmentKind::DromMap | SegmentKind::IromMap,
                SegmentKind::DromMap | SegmentKind::IromMap,
            ) => {
                if let (Some((a_lo, a_hi)), Some((b_lo, b_hi))) = (
                    mmu_page_vspan(prev.load_addr, prev.data_len),
                    mmu_page_vspan(candidate.load_addr, candidate.data_len),
                ) {
                    if a_lo < b_hi && b_lo < a_hi {
                        return Err(ImageVerifyError::SegmentOverlap);
                    }
                }
            }
            _ => {}
        }
    }
    Ok(())
}

fn entry_in_verified_executable_segment(
    entry_addr: u32,
    segments: &[Option<VerifiedSegment>; MAX_SEGMENTS],
) -> bool {
    if !is_valid_executable_addr(entry_addr) {
        return false;
    }
    for seg in segments.iter().flatten() {
        let is_exec_seg = match seg.kind {
            SegmentKind::IromMap => true,
            SegmentKind::RamLoad => seg.load_addr >= SOC_APP_IRAM_LOW,
            SegmentKind::DromMap | SegmentKind::RtcSkip => false,
        };
        if !is_exec_seg || seg.data_len == 0 {
            continue;
        }
        if let Some(seg_end) = seg.load_addr.checked_add(seg.data_len) {
            if entry_addr >= seg.load_addr && entry_addr < seg_end {
                return true;
            }
        }
    }
    false
}

/// Verifies an ESP32 application image from flash, enforcing strict ESP32
/// memory allowlists, mandatory SHA-256 verification, executable entry point
/// bounds, inter-segment isolation, and clean trailing flash state.
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

    // Mandatory SHA-256 verification: reject images without hash_appended == 1.
    if header[23] != 1 {
        return Err(ImageVerifyError::MissingHash);
    }

    let entry_addr = u32::from_le_bytes([header[4], header[5], header[6], header[7]]);

    let mut sha256 = Sha256::new();
    sha256.update(header);
    let mut xor_checksum: u8 = ESP_CHECKSUM_MAGIC;

    let mut segments: [Option<VerifiedSegment>; MAX_SEGMENTS] = [None; MAX_SEGMENTS];
    let mut rel_cursor: u32 = 24;

    for seg_idx in 0..segment_count {
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

        if (data_len & 3) != 0 {
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

        // Enforce 64 KiB intra-page offset parity for MMU-mapped segments.
        if matches!(kind, SegmentKind::DromMap | SegmentKind::IromMap)
            && (load_addr & 0xFFFF) != (flash_offset & 0xFFFF)
        {
            return Err(ImageVerifyError::UnalignedLoadAddress);
        }

        let candidate = VerifiedSegment {
            flash_offset,
            load_addr,
            data_len,
            kind,
        };
        check_segment_pairwise_overlap(&segments[..seg_idx], &candidate)?;
        segments[seg_idx] = Some(candidate);

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

    // Pad to the next 16-byte boundary (always 1..=16 bytes, no underflow).
    let pad_len = 16 - (rel_cursor & 15);
    let target_end = rel_cursor
        .checked_add(pad_len)
        .ok_or(ImageVerifyError::OutOfBounds)?;
    if target_end > part_size {
        return Err(ImageVerifyError::OutOfBounds);
    }

    let pad_flash_addr = part_offset
        .checked_add(rel_cursor)
        .ok_or(ImageVerifyError::OutOfBounds)?;
    flash.read(pad_flash_addr, &mut bounce.bytes[..pad_len as usize])?;
    let pad_bytes = &bounce.bytes[..pad_len as usize];

    for &pad_byte in &pad_bytes[..pad_len as usize - 1] {
        if pad_byte != 0 {
            return Err(ImageVerifyError::TrailingBytes);
        }
    }

    let actual_checksum = pad_bytes[pad_len as usize - 1];
    if actual_checksum != xor_checksum {
        return Err(ImageVerifyError::ChecksumMismatch);
    }

    sha256.update(pad_bytes);
    rel_cursor = target_end;

    // Mandatory 32-byte SHA-256 verification.
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

    // Reject non-erased trailing unverified bytes immediately following the
    // SHA-256 trailer up to the next 4 KiB sector boundary (clamped to
    // `part_size`).
    if rel_cursor < part_size {
        const SECTOR_BYTES: u32 = 4096;
        let next_sector_boundary = rel_cursor
            .checked_add(SECTOR_BYTES)
            .map_or(part_size, |v| (v & !(SECTOR_BYTES - 1)).min(part_size));
        let mut tail_cursor = rel_cursor;
        while tail_cursor < next_sector_boundary {
            let chunk_len = core::cmp::min(
                (next_sector_boundary - tail_cursor) as usize,
                bounce.bytes.len(),
            );
            let tail_addr = part_offset
                .checked_add(tail_cursor)
                .ok_or(ImageVerifyError::OutOfBounds)?;
            flash.read(tail_addr, &mut bounce.bytes[..chunk_len])?;
            for &b in &bounce.bytes[..chunk_len] {
                if b != 0xFF {
                    return Err(ImageVerifyError::TrailingBytes);
                }
            }
            tail_cursor = tail_cursor
                .checked_add(chunk_len as u32)
                .ok_or(ImageVerifyError::OutOfBounds)?;
        }
    }

    if !entry_in_verified_executable_segment(entry_addr, &segments) {
        return Err(ImageVerifyError::InvalidEntryAddress);
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
        assert_eq!(verified.entry_addr(), 0x4008_0000);
        assert_eq!(verified.segment_count(), 3);
        assert_eq!(verified.image_end_offset(), total_len as u32);

        let s0 = verified.segments()[0].expect("seg 0 present");
        assert_eq!(s0.kind(), SegmentKind::DromMap);
        assert_eq!(s0.load_addr(), 0x3F40_0020);
        assert_eq!(s0.data_len(), 64);
        assert_eq!(s0.flash_offset(), 32);

        let s1 = verified.segments()[1].expect("seg 1 present");
        assert_eq!(s1.kind(), SegmentKind::RamLoad);
        assert_eq!(s1.load_addr(), 0x4008_0000);
        assert_eq!(s1.data_len(), 64);
        assert_eq!(s1.flash_offset(), 104);

        let s2 = verified.segments()[2].expect("seg 2 present");
        assert_eq!(s2.kind(), SegmentKind::RamLoad);
        assert_eq!(s2.load_addr(), 0x3FFB_0000);
        assert_eq!(s2.data_len(), 64);
        assert_eq!(s2.flash_offset(), 176);
    }

    #[test]
    fn test_corrupted_payload_checksum_mismatch() {
        let drom_payload = [0x11u8; 64];
        let iram_payload = [0x22u8; 64];
        let (mut image_bytes, total_len) = build_test_image(
            0x4008_0000,
            true,
            &[(0x3F40_0020, &drom_payload), (0x4008_0000, &iram_payload)],
        );

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
        let iram_payload = [0x22u8; 64];
        let (mut image_bytes, total_len) = build_test_image(
            0x4008_0000,
            true,
            &[(0x3F40_0020, &drom_payload), (0x4008_0000, &iram_payload)],
        );

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
        let (image1, len1) = build_test_image(0x4008_0000, true, &[(0x4007_8100, &payload)]);
        let mut flash1 = MockFlash::<2>::new();
        flash1
            .write_raw(0, &image1[..len1])
            .expect("write_raw failed");
        let err1 = verify_app_image(&mut flash1, 0, 8192).expect_err("should reject IRAM overlap");
        assert_eq!(err1, ImageVerifyError::BootloaderOverlap);

        // Case 2: Overlaps Bootloader DRAM (0x3FFF_0000..0x3FFF_8000)
        let (image2, len2) = build_test_image(0x4008_0000, true, &[(0x3FFF_0100, &payload)]);
        let mut flash2 = MockFlash::<2>::new();
        flash2
            .write_raw(0, &image2[..len2])
            .expect("write_raw failed");
        let err2 = verify_app_image(&mut flash2, 0, 8192).expect_err("should reject DRAM overlap");
        assert_eq!(err2, ImageVerifyError::BootloaderOverlap);

        // Case 3: Overlaps SRAM1 IRAM alias of bootloader DRAM
        // (0x400A_8000..0x400B_0000)
        let (image3, len3) = build_test_image(0x4008_0000, true, &[(0x400A_8100, &payload)]);
        let mut flash3 = MockFlash::<2>::new();
        flash3
            .write_raw(0, &image3[..len3])
            .expect("write_raw failed");
        let err3 = verify_app_image(&mut flash3, 0, 8192).expect_err("should reject alias overlap");
        assert_eq!(err3, ImageVerifyError::BootloaderOverlap);
    }

    #[test]
    fn test_padding_segment_returns_rtc_skip() {
        let pad_payload = [0x00u8; 64];
        let drom_payload = [0x11u8; 64];
        let iram_payload = [0x22u8; 64];

        // seg0 ends at offset 24+8+64 = 96; seg1 header is 8 bytes -> payload
        // at flash offset 104 (0x68), matching 0x3F40_0068 page offset.
        let (image_bytes, total_len) = build_test_image(
            0x4008_0000,
            true,
            &[
                (0, &pad_payload),
                (0x3F40_0068, &drom_payload),
                (0x4008_0000, &iram_payload),
            ],
        );

        let mut flash = MockFlash::<2>::new();
        flash
            .write_raw(0, &image_bytes[..total_len])
            .expect("write_raw failed");

        let verified = verify_app_image(&mut flash, 0, 8192).expect("verification should succeed");
        assert_eq!(verified.segment_count(), 3);

        let s0 = verified.segments()[0].expect("seg 0 present");
        assert_eq!(s0.kind(), SegmentKind::RtcSkip);
        assert_eq!(s0.load_addr(), 0);
        assert_eq!(s0.data_len(), 64);
        assert_eq!(s0.flash_offset(), 32);

        let s1 = verified.segments()[1].expect("seg 1 present");
        assert_eq!(s1.kind(), SegmentKind::DromMap);
        assert_eq!(s1.load_addr(), 0x3F40_0068);
        assert_eq!(s1.data_len(), 64);
        assert_eq!(s1.flash_offset(), 104);
    }

    #[test]
    fn test_rejects_mmio_rom_unmapped_and_unaligned_segments() {
        let payload = [0x5Au8; 64];

        // MMIO peripheral register space (DPORT 0x3FF0_0000)
        let (img_mmio, len_mmio) = build_test_image(0x4008_0000, true, &[(0x3FF0_0040, &payload)]);
        let mut flash = MockFlash::<2>::new();
        flash.write_raw(0, &img_mmio[..len_mmio]).expect("write");
        assert_eq!(
            verify_app_image(&mut flash, 0, 8192),
            Err(ImageVerifyError::OutOfBounds)
        );

        // Mask ROM space (0x4000_0000)
        let (img_rom, len_rom) = build_test_image(0x4008_0000, true, &[(0x4000_0100, &payload)]);
        let mut flash_rom = MockFlash::<2>::new();
        flash_rom.write_raw(0, &img_rom[..len_rom]).expect("write");
        assert_eq!(
            verify_app_image(&mut flash_rom, 0, 8192),
            Err(ImageVerifyError::OutOfBounds)
        );

        // ROM static stack/BSS space (0x3FFF_8000..0x4000_0000)
        let (img_stack, len_stack) =
            build_test_image(0x4008_0000, true, &[(0x3FFF_8100, &payload)]);
        let mut flash_stack = MockFlash::<2>::new();
        flash_stack
            .write_raw(0, &img_stack[..len_stack])
            .expect("write");
        assert_eq!(
            verify_app_image(&mut flash_stack, 0, 8192),
            Err(ImageVerifyError::OutOfBounds)
        );

        // Unaligned RAM load address
        let (img_unaligned, len_u) =
            build_test_image(0x4008_0000, true, &[(0x4008_0002, &payload)]);
        let mut flash_u = MockFlash::<2>::new();
        flash_u
            .write_raw(0, &img_unaligned[..len_u])
            .expect("write");
        assert_eq!(
            verify_app_image(&mut flash_u, 0, 8192),
            Err(ImageVerifyError::UnalignedLoadAddress)
        );
    }

    #[test]
    fn test_rejects_missing_sha256_and_trailing_unverified_bytes() {
        let iram_payload = [0x22u8; 64];

        // Case 1: header[23] == 0 (missing mandatory SHA-256)
        let (img_no_hash, len_nh) =
            build_test_image(0x4008_0000, false, &[(0x4008_0000, &iram_payload)]);
        let mut flash_nh = MockFlash::<2>::new();
        flash_nh
            .write_raw(0, &img_no_hash[..len_nh])
            .expect("write");
        assert_eq!(
            verify_app_image(&mut flash_nh, 0, 8192),
            Err(ImageVerifyError::MissingHash)
        );

        // Case 2: header[23] == 2 (non-boolean downgrade value)
        let (mut img_bad_flag, len_bf) =
            build_test_image(0x4008_0000, true, &[(0x4008_0000, &iram_payload)]);
        img_bad_flag[23] = 2;
        let mut flash_bf = MockFlash::<2>::new();
        flash_bf
            .write_raw(0, &img_bad_flag[..len_bf])
            .expect("write");
        assert_eq!(
            verify_app_image(&mut flash_bf, 0, 8192),
            Err(ImageVerifyError::MissingHash)
        );

        // Case 3: trailing non-0xFF byte appended after valid SHA-256 trailer
        let (img_valid, len_v) =
            build_test_image(0x4008_0000, true, &[(0x4008_0000, &iram_payload)]);
        let mut flash_tail = MockFlash::<2>::new();
        flash_tail.write_raw(0, &img_valid[..len_v]).expect("write");
        flash_tail
            .write_raw(len_v as u32, &[0xAA])
            .expect("write trailing byte");
        assert_eq!(
            verify_app_image(&mut flash_tail, 0, 8192),
            Err(ImageVerifyError::TrailingBytes)
        );
    }

    #[test]
    fn test_rejects_invalid_entry_addr_and_overlapping_segments() {
        let dram_payload = [0x33u8; 64];
        let iram_payload = [0x22u8; 64];

        // Case 1: entry_addr == 0
        let (img_zero, len_z) = build_test_image(0, true, &[(0x4008_0000, &iram_payload)]);
        let mut flash_z = MockFlash::<2>::new();
        flash_z.write_raw(0, &img_zero[..len_z]).expect("write");
        assert_eq!(
            verify_app_image(&mut flash_z, 0, 8192),
            Err(ImageVerifyError::InvalidEntryAddress)
        );

        // Case 2: entry_addr in DRAM (non-executable)
        let (img_dram, len_d) =
            build_test_image(0x3FFB_0000, true, &[(0x3FFB_0000, &dram_payload)]);
        let mut flash_d = MockFlash::<2>::new();
        flash_d.write_raw(0, &img_dram[..len_d]).expect("write");
        assert_eq!(
            verify_app_image(&mut flash_d, 0, 8192),
            Err(ImageVerifyError::InvalidEntryAddress)
        );

        // Case 3: entry_addr in IRAM range but outside loaded segment
        let (img_outside, len_o) =
            build_test_image(0x4008_1000, true, &[(0x4008_0000, &iram_payload)]);
        let mut flash_o = MockFlash::<2>::new();
        flash_o.write_raw(0, &img_outside[..len_o]).expect("write");
        assert_eq!(
            verify_app_image(&mut flash_o, 0, 8192),
            Err(ImageVerifyError::InvalidEntryAddress)
        );

        // Case 4: pairwise overlapping RamLoad segments
        let (img_ov, len_ov) = build_test_image(
            0x4008_0000,
            true,
            &[(0x4008_0000, &iram_payload), (0x4008_0020, &iram_payload)],
        );
        let mut flash_ov = MockFlash::<2>::new();
        flash_ov.write_raw(0, &img_ov[..len_ov]).expect("write");
        assert_eq!(
            verify_app_image(&mut flash_ov, 0, 8192),
            Err(ImageVerifyError::SegmentOverlap)
        );

        // Case 5: DROM segment violating 64 KiB intra-page parity
        let (img_par, len_par) = build_test_image(
            0x4008_0000,
            true,
            &[(0x3F40_0040, &dram_payload), (0x4008_0000, &iram_payload)],
        );
        let mut flash_par = MockFlash::<2>::new();
        flash_par.write_raw(0, &img_par[..len_par]).expect("write");
        assert_eq!(
            verify_app_image(&mut flash_par, 0, 8192),
            Err(ImageVerifyError::UnalignedLoadAddress)
        );
    }
}
