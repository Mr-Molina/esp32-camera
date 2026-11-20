//! Partition table parser with RFC 1321 MD5 validation for ESP32.
//!
//! Parses the 2nd-stage bootloader partition table at flash offset `0x8000`,
//! verifies MD5 checksum trailers, and extracts application and data partition
//! layouts without dynamic memory allocation.

use crate::rom_boundary::{AlignedBounce64, FlashError, FlashIo};

/// Flash offset of the primary partition table (`0x8000`).
pub const PARTITION_TABLE_OFFSET: u32 = 0x8000;

/// Maximum size of the partition table region (`0xC00` = 3072 bytes,
/// 96 entries max).
pub const PARTITION_TABLE_MAX_BYTES: usize = 0xC00;

/// Partition entry magic number (`0x50AA`, `[0xAA, 0x50]` little-endian).
pub const PARTITION_MAGIC: u16 = 0x50AA;

/// Partition MD5 trailer magic number (`0xEBEB`, `[0xEB, 0xEB]`).
pub const PARTITION_MD5_MAGIC: u16 = 0xEBEB;

/// Application partition type (`0x00`).
pub const PART_TYPE_APP: u8 = 0x00;

/// Data partition type (`0x01`).
pub const PART_TYPE_DATA: u8 = 0x01;

/// Factory application subtype (`0x00`).
pub const PART_SUBTYPE_FACTORY: u8 = 0x00;

/// Minimum OTA application subtype (`0x10`, `ota_0`).
pub const PART_SUBTYPE_OTA_MIN: u8 = 0x10;

/// Maximum OTA application subtype (`0x1F`, `ota_15`).
pub const PART_SUBTYPE_OTA_MAX: u8 = 0x1F;

/// OTA selection data subtype (`0x00`).
pub const PART_SUBTYPE_DATA_OTA: u8 = 0x00;

/// Total SPI flash capacity for ESP32-WROVER 4 MiB modules (`0x0040_0000`).
pub const FLASH_SIZE_BYTES: u32 = 4 * 1024 * 1024;

/// Minimum allowed flash offset for any partition entry (`0x9000`).
///
/// Protects the 2nd-stage bootloader (`0x1000`) and partition table (`0x8000`).
pub const MIN_PARTITION_OFFSET: u32 = 0x9000;

/// Required 64 KiB alignment for ESP32 MMU-mapped APP partitions (`0x10000`).
pub const APP_PARTITION_ALIGN: u32 = 0x1_0000;

/// Required 4 KiB sector alignment for `otadata` partitions (`0x1000`).
pub const OTADATA_PARTITION_ALIGN: u32 = 0x1000;

/// Minimum required size for dual-sector `otadata` partitions (`0x2000`).
pub const OTADATA_MIN_SIZE: u32 = 0x2000;

/// Maximum number of 32-byte entries in the partition table region.
const MAX_PARTITION_ENTRIES: usize = PARTITION_TABLE_MAX_BYTES / 32;

/// Errors encountered while reading and parsing partition tables.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PartitionError {
    /// Flash I/O error occurred during read.
    Flash(FlashError),
    /// Partition table header or entry structure is invalid.
    InvalidTable,
    /// Partition table MD5 checksum verification failed.
    Md5Mismatch,
    /// Requested partition was not found.
    NotFound,
}

impl core::fmt::Display for PartitionError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Flash(err) => write!(f, "Flash I/O error: {err:?}"),
            Self::InvalidTable => write!(f, "Invalid partition table structure"),
            Self::Md5Mismatch => write!(f, "Partition table MD5 checksum mismatch"),
            Self::NotFound => write!(f, "Partition not found"),
        }
    }
}

impl From<FlashError> for PartitionError {
    #[inline]
    fn from(err: FlashError) -> Self {
        Self::Flash(err)
    }
}

/// Linear flash span defined by offset and size.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PartitionSpan {
    /// Starting byte address in SPI flash.
    pub offset: u32,
    /// Total length in bytes.
    pub size: u32,
}

/// Parsed partition layout discovered in flash.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PartitionLayout {
    /// Optional OTA data sector pair (`otadata`).
    pub otadata: Option<PartitionSpan>,
    /// Optional factory application partition.
    pub factory: Option<PartitionSpan>,
    /// Discovered OTA application slots (`ota_0` through `ota_15`).
    pub ota_slots: [Option<PartitionSpan>; 16],
    /// Total count of valid OTA slots present (`0..=16`).
    pub ota_count: u8,
}

impl PartitionLayout {
    /// Creates an empty [`PartitionLayout`] with no partitions registered.
    #[must_use]
    pub const fn empty() -> Self {
        Self {
            otadata: None,
            factory: None,
            ota_slots: [None; 16],
            ota_count: 0,
        }
    }
}

impl Default for PartitionLayout {
    fn default() -> Self {
        Self::empty()
    }
}

// RFC 1321 sine constants table K[0..63]
const MD5_K: [u32; 64] = [
    0xd76a_a478,
    0xe8c7_b756,
    0x2420_70db,
    0xc1bd_ceee,
    0xf57c_0faf,
    0x4787_c62a,
    0xa830_4613,
    0xfd46_9501,
    0x6980_98d8,
    0x8b44_f7af,
    0xffff_5bb1,
    0x895c_d7be,
    0x6b90_1122,
    0xfd98_7193,
    0xa679_438e,
    0x49b4_0821,
    0xf61e_2562,
    0xc040_b340,
    0x265e_5a51,
    0xe9b6_c7aa,
    0xd62f_105d,
    0x0244_1453,
    0xd8a1_e681,
    0xe7d3_fbc8,
    0x21e1_cde6,
    0xc337_07d6,
    0xf4d5_0d87,
    0x455a_14ed,
    0xa9e3_e905,
    0xfcef_a3f8,
    0x676f_02d9,
    0x8d2a_4c8a,
    0xfffa_3942,
    0x8771_f681,
    0x6d9d_6122,
    0xfde5_380c,
    0xa4be_ea44,
    0x4bde_cfa9,
    0xf6bb_4b60,
    0xbebf_bc70,
    0x289b_7ec6,
    0xeaa1_27fa,
    0xd4ef_3085,
    0x0488_1d05,
    0xd9d4_d039,
    0xe6db_99e5,
    0x1fa2_7cf8,
    0xc4ac_5665,
    0xf429_2244,
    0x432a_ff97,
    0xab94_23a7,
    0xfc93_a039,
    0x655b_59c3,
    0x8f0c_cc92,
    0xffef_f47d,
    0x8584_5dd1,
    0x6fa8_7e4f,
    0xfe2c_e6e0,
    0xa301_4314,
    0x4e08_11a1,
    0xf753_7e82,
    0xbd3a_f235,
    0x2ad7_d2bb,
    0xeb86_d391,
];

// RFC 1321 per-round shift amounts S[0..63]
const MD5_S: [u32; 64] = [
    7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22, 5, 9, 14, 20, 5, 9, 14, 20, 5, 9,
    14, 20, 5, 9, 14, 20, 4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 6, 10, 15,
    21, 6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21,
];

/// Stack-only RFC 1321 MD5 streaming state hasher.
#[derive(Debug, Clone)]
pub struct Md5Hasher {
    state: [u32; 4],
    byte_count: u64,
    buffer: [u8; 64],
    buf_len: usize,
}

impl Md5Hasher {
    /// Creates a new zero-allocation [`Md5Hasher`].
    #[must_use]
    pub const fn new() -> Self {
        Self {
            state: [0x6745_2301, 0xEFCD_AB89, 0x98BA_DCFE, 0x1032_5476],
            byte_count: 0,
            buffer: [0u8; 64],
            buf_len: 0,
        }
    }

    /// Feeds a chunk of bytes into the hasher.
    pub fn update(&mut self, mut data: &[u8]) {
        while !data.is_empty() {
            let available = 64 - self.buf_len;
            let take = if data.len() < available {
                data.len()
            } else {
                available
            };
            let mut i = 0;
            while i < take {
                self.buffer[self.buf_len + i] = data[i];
                i += 1;
            }
            self.buf_len += take;
            self.byte_count = self.byte_count.saturating_add(take as u64);
            data = &data[take..];

            if self.buf_len == 64 {
                self.process_block();
                self.buf_len = 0;
            }
        }
    }

    /// Finalizes hashing and returns the 16-byte MD5 digest.
    #[must_use]
    pub fn finalize(mut self) -> [u8; 16] {
        let bit_len = self.byte_count.wrapping_mul(8);
        self.buffer[self.buf_len] = 0x80;
        self.buf_len += 1;

        if self.buf_len > 56 {
            let mut i = self.buf_len;
            while i < 64 {
                self.buffer[i] = 0;
                i += 1;
            }
            self.process_block();
            self.buf_len = 0;
        }

        let mut i = self.buf_len;
        while i < 56 {
            self.buffer[i] = 0;
            i += 1;
        }

        let len_bytes = bit_len.to_le_bytes();
        let mut j = 0;
        while j < 8 {
            self.buffer[56 + j] = len_bytes[j];
            j += 1;
        }
        self.process_block();

        let mut out = [0u8; 16];
        let mut k = 0;
        while k < 4 {
            let word = self.state[k].to_le_bytes();
            out[k * 4] = word[0];
            out[k * 4 + 1] = word[1];
            out[k * 4 + 2] = word[2];
            out[k * 4 + 3] = word[3];
            k += 1;
        }
        out
    }

    fn process_block(&mut self) {
        let mut m = [0u32; 16];
        let mut j = 0;
        while j < 16 {
            let offset = j * 4;
            m[j] = u32::from_le_bytes([
                self.buffer[offset],
                self.buffer[offset + 1],
                self.buffer[offset + 2],
                self.buffer[offset + 3],
            ]);
            j += 1;
        }

        let mut a = self.state[0];
        let mut b = self.state[1];
        let mut c = self.state[2];
        let mut d = self.state[3];

        let mut i = 0;
        while i < 64 {
            let (f, g) = if i < 16 {
                ((b & c) | ((!b) & d), i)
            } else if i < 32 {
                ((d & b) | ((!d) & c), (5 * i + 1) % 16)
            } else if i < 48 {
                (b ^ c ^ d, (3 * i + 5) % 16)
            } else {
                (c ^ (b | (!d)), (7 * i) % 16)
            };

            let temp = d;
            d = c;
            c = b;
            let sum = a.wrapping_add(f).wrapping_add(MD5_K[i]).wrapping_add(m[g]);
            let rotated = sum.rotate_left(MD5_S[i]);
            b = b.wrapping_add(rotated);
            a = temp;
            i += 1;
        }

        self.state[0] = self.state[0].wrapping_add(a);
        self.state[1] = self.state[1].wrapping_add(b);
        self.state[2] = self.state[2].wrapping_add(c);
        self.state[3] = self.state[3].wrapping_add(d);
    }
}

impl Default for Md5Hasher {
    fn default() -> Self {
        Self::new()
    }
}

/// Computes the 16-byte RFC 1321 MD5 digest of `data` on the stack.
#[must_use]
pub fn md5_digest(data: &[u8]) -> [u8; 16] {
    let mut hasher = Md5Hasher::new();
    hasher.update(data);
    hasher.finalize()
}

/// Encodes an ESP-IDF partition table MD5 trailer entry.
///
/// Bytes `0..2` contain `0xEBEB`, bytes `2..16` contain `0xFF` padding,
/// and bytes `16..32` contain the MD5 digest over `entries_bytes`.
#[must_use]
pub fn encode_md5_entry(entries_bytes: &[u8]) -> [u8; 32] {
    let mut entry = [0xFFu8; 32];
    let magic = PARTITION_MD5_MAGIC.to_le_bytes();
    entry[0] = magic[0];
    entry[1] = magic[1];
    let digest = md5_digest(entries_bytes);
    let mut i = 0;
    while i < 16 {
        entry[16 + i] = digest[i];
        i += 1;
    }
    entry
}

/// Reads and parses the flash partition table starting at `0x8000`.
///
/// Verifies the `0x50AA` magic on each entry, enforces offset/size/alignment,
/// flash-bounds, duplicate-subtype, overlap, and contiguous-OTA invariants,
/// and requires a valid `0xEBEB` MD5 trailer with 14-byte `0xFF` padding.
pub fn parse_partition_table<F: FlashIo>(flash: &mut F) -> Result<PartitionLayout, PartitionError> {
    let mut bounce = AlignedBounce64::new();
    let mut md5 = Md5Hasher::new();
    let mut layout = PartitionLayout::empty();
    let mut app_found = false;
    let mut md5_verified = false;
    let mut spans = [(0u32, 0u32); MAX_PARTITION_ENTRIES];
    let mut seen_types = [(0u8, 0u8); MAX_PARTITION_ENTRIES];
    let mut span_count = 0usize;

    let mut i = 0;
    while i < MAX_PARTITION_ENTRIES {
        let chunk_offset = (i * 32) as u32;
        let read_addr = PARTITION_TABLE_OFFSET
            .checked_add(chunk_offset)
            .ok_or(PartitionError::InvalidTable)?;

        flash.read(read_addr, &mut bounce.bytes[..32])?;
        let entry = &bounce.bytes[..32];

        // Check for MD5 marker (0xEBEB) with 14-byte 0xFF padding
        if entry[0] == 0xEB && entry[1] == 0xEB {
            if entry[2..16] != [0xFFu8; 14] {
                return Err(PartitionError::InvalidTable);
            }
            let actual = md5.finalize();
            if entry[16..32] != actual {
                return Err(PartitionError::Md5Mismatch);
            }
            md5_verified = true;
            break;
        }

        // Check for end of table marker (0xFFFF)
        if entry[0] == 0xFF && entry[1] == 0xFF {
            break;
        }

        let magic = u16::from_le_bytes([entry[0], entry[1]]);
        if magic != PARTITION_MAGIC {
            return Err(PartitionError::InvalidTable);
        }

        md5.update(entry);

        let ptype = entry[2];
        let subtype = entry[3];
        let offset = u32::from_le_bytes([entry[4], entry[5], entry[6], entry[7]]);
        let size = u32::from_le_bytes([entry[8], entry[9], entry[10], entry[11]]);

        // Reject zero-sized, below-0x9000, overflowing, or out-of-flash spans
        if size == 0 || offset < MIN_PARTITION_OFFSET {
            return Err(PartitionError::InvalidTable);
        }
        let end = offset
            .checked_add(size)
            .ok_or(PartitionError::InvalidTable)?;
        if end > FLASH_SIZE_BYTES {
            return Err(PartitionError::InvalidTable);
        }

        // Enforce alignment and size invariants for APP and otadata entries
        if ptype == PART_TYPE_APP && (offset & (APP_PARTITION_ALIGN - 1)) != 0 {
            return Err(PartitionError::InvalidTable);
        }
        if ptype == PART_TYPE_DATA
            && subtype == PART_SUBTYPE_DATA_OTA
            && ((offset & (OTADATA_PARTITION_ALIGN - 1)) != 0 || size < OTADATA_MIN_SIZE)
        {
            return Err(PartitionError::InvalidTable);
        }

        // Reject duplicate (type, subtype) entries and overlapping flash spans
        let mut prev = 0;
        while prev < span_count {
            if seen_types[prev] == (ptype, subtype) {
                return Err(PartitionError::InvalidTable);
            }
            let (prev_start, prev_end) = spans[prev];
            if offset < prev_end && prev_start < end {
                return Err(PartitionError::InvalidTable);
            }
            prev += 1;
        }
        spans[span_count] = (offset, end);
        seen_types[span_count] = (ptype, subtype);
        span_count += 1;

        let span = PartitionSpan { offset, size };

        if ptype == PART_TYPE_DATA && subtype == PART_SUBTYPE_DATA_OTA {
            layout.otadata = Some(span);
        } else if ptype == PART_TYPE_APP && subtype == PART_SUBTYPE_FACTORY {
            layout.factory = Some(span);
            app_found = true;
        } else if ptype == PART_TYPE_APP
            && (PART_SUBTYPE_OTA_MIN..=PART_SUBTYPE_OTA_MAX).contains(&subtype)
        {
            let slot = (subtype - PART_SUBTYPE_OTA_MIN) as usize;
            if slot < 16 {
                layout.ota_slots[slot] = Some(span);
                let next_count = (slot + 1) as u8;
                if next_count > layout.ota_count {
                    layout.ota_count = next_count;
                }
                app_found = true;
            }
        }

        i += 1;
    }

    if !app_found {
        return Err(PartitionError::InvalidTable);
    }

    if !md5_verified {
        return Err(PartitionError::Md5Mismatch);
    }

    // Require contiguous OTA slots 0..ota_count so modulo selection is sound
    let mut slot = 0;
    while slot < layout.ota_count as usize {
        if layout.ota_slots[slot].is_none() {
            return Err(PartitionError::InvalidTable);
        }
        slot += 1;
    }

    Ok(layout)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rom_boundary::MockFlash;

    fn make_entry(ptype: u8, subtype: u8, offset: u32, size: u32) -> [u8; 32] {
        let mut entry = [0xFFu8; 32];
        let magic = PARTITION_MAGIC.to_le_bytes();
        entry[0] = magic[0];
        entry[1] = magic[1];
        entry[2] = ptype;
        entry[3] = subtype;
        let off_b = offset.to_le_bytes();
        entry[4] = off_b[0];
        entry[5] = off_b[1];
        entry[6] = off_b[2];
        entry[7] = off_b[3];
        let sz_b = size.to_le_bytes();
        entry[8] = sz_b[0];
        entry[9] = sz_b[1];
        entry[10] = sz_b[2];
        entry[11] = sz_b[3];
        entry
    }

    #[test]
    fn md5_standard_test_vectors() {
        assert_eq!(
            md5_digest(b""),
            [
                0xd4, 0x1d, 0x8c, 0xd9, 0x8f, 0x00, 0xb2, 0x04, 0xe9, 0x80, 0x09, 0x98, 0xec, 0xf8,
                0x42, 0x7e
            ]
        );
        assert_eq!(
            md5_digest(b"a"),
            [
                0x0c, 0xc1, 0x75, 0xb9, 0xc0, 0xf1, 0xb6, 0xa8, 0x31, 0xc3, 0x99, 0xe2, 0x69, 0x77,
                0x26, 0x61
            ]
        );
        assert_eq!(
            md5_digest(b"abc"),
            [
                0x90, 0x01, 0x50, 0x98, 0x3c, 0xd2, 0x4f, 0xb0, 0xd6, 0x96, 0x3f, 0x7d, 0x28, 0xe1,
                0x7f, 0x72
            ]
        );
        assert_eq!(
            md5_digest(b"message digest"),
            [
                0xf9, 0x6b, 0x69, 0x7d, 0x7c, 0xb7, 0x93, 0x8d, 0x52, 0x5a, 0x2f, 0x31, 0xaa, 0xf1,
                0x61, 0xd0
            ]
        );
    }

    #[test]
    fn parse_realistic_partition_table_with_md5() {
        let mut mock = MockFlash::<9>::new();
        let mut raw = [0xFFu8; 128];

        let e_otadata = make_entry(PART_TYPE_DATA, PART_SUBTYPE_DATA_OTA, 0x17000, 0x2000);
        let e_ota0 = make_entry(PART_TYPE_APP, PART_SUBTYPE_OTA_MIN, 0x20000, 0x1B0000);
        let e_ota1 = make_entry(PART_TYPE_APP, PART_SUBTYPE_OTA_MIN + 1, 0x1D0000, 0x1B0000);

        raw[0..32].copy_from_slice(&e_otadata);
        raw[32..64].copy_from_slice(&e_ota0);
        raw[64..96].copy_from_slice(&e_ota1);

        let trailer = encode_md5_entry(&raw[..96]);
        raw[96..128].copy_from_slice(&trailer);

        mock.write_raw(PARTITION_TABLE_OFFSET, &raw)
            .expect("flash write");

        let layout = parse_partition_table(&mut mock).expect("valid parse");
        assert_eq!(
            layout.otadata,
            Some(PartitionSpan {
                offset: 0x17000,
                size: 0x2000,
            })
        );
        assert_eq!(layout.ota_count, 2);
        assert_eq!(
            layout.ota_slots[0],
            Some(PartitionSpan {
                offset: 0x20000,
                size: 0x1B0000,
            })
        );
        assert_eq!(
            layout.ota_slots[1],
            Some(PartitionSpan {
                offset: 0x1D0000,
                size: 0x1B0000,
            })
        );
        assert_eq!(layout.factory, None);
    }

    #[test]
    fn corrupted_md5_returns_mismatch_error() {
        let mut mock = MockFlash::<9>::new();
        let mut raw = [0xFFu8; 128];

        let e_ota0 = make_entry(PART_TYPE_APP, PART_SUBTYPE_OTA_MIN, 0x20000, 0x1B0000);
        raw[0..32].copy_from_slice(&e_ota0);

        let mut trailer = encode_md5_entry(&raw[..32]);
        trailer[16] ^= 0xFF; // corrupt digest
        raw[32..64].copy_from_slice(&trailer);

        mock.write_raw(PARTITION_TABLE_OFFSET, &raw)
            .expect("flash write");
        let res = parse_partition_table(&mut mock);
        assert_eq!(res, Err(PartitionError::Md5Mismatch));
    }

    #[test]
    fn empty_table_returns_invalid_table() {
        let mut mock = MockFlash::<9>::new();
        let res = parse_partition_table(&mut mock);
        assert_eq!(res, Err(PartitionError::InvalidTable));
    }

    fn write_entries_with_md5(mock: &mut MockFlash<9>, entries: &[[u8; 32]]) {
        let mut raw = [0xFFu8; 256];
        let used = entries.len() * 32;
        for (idx, entry) in entries.iter().enumerate() {
            raw[idx * 32..(idx + 1) * 32].copy_from_slice(entry);
        }
        let trailer = encode_md5_entry(&raw[..used]);
        raw[used..used + 32].copy_from_slice(&trailer);
        mock.write_raw(PARTITION_TABLE_OFFSET, &raw[..used + 32])
            .expect("flash write");
    }

    #[test]
    fn missing_md5_trailer_is_rejected() {
        let mut mock = MockFlash::<9>::new();
        let e_ota0 = make_entry(PART_TYPE_APP, PART_SUBTYPE_OTA_MIN, 0x20000, 0x100000);
        mock.write_raw(PARTITION_TABLE_OFFSET, &e_ota0)
            .expect("flash write");
        assert_eq!(
            parse_partition_table(&mut mock),
            Err(PartitionError::Md5Mismatch)
        );
    }

    #[test]
    fn md5_trailer_with_non_ff_padding_is_rejected() {
        let mut mock = MockFlash::<9>::new();
        let mut raw = [0xFFu8; 64];
        let e_ota0 = make_entry(PART_TYPE_APP, PART_SUBTYPE_OTA_MIN, 0x20000, 0x100000);
        raw[0..32].copy_from_slice(&e_ota0);
        let mut trailer = encode_md5_entry(&raw[..32]);
        trailer[5] = 0x00; // corrupt 14-byte 0xFF padding
        raw[32..64].copy_from_slice(&trailer);
        mock.write_raw(PARTITION_TABLE_OFFSET, &raw)
            .expect("flash write");
        assert_eq!(
            parse_partition_table(&mut mock),
            Err(PartitionError::InvalidTable)
        );
    }

    #[test]
    fn rejects_zero_size_overflow_and_out_of_flash() {
        let mut mock = MockFlash::<9>::new();
        let zero = make_entry(PART_TYPE_APP, PART_SUBTYPE_OTA_MIN, 0x20000, 0);
        write_entries_with_md5(&mut mock, &[zero]);
        assert_eq!(
            parse_partition_table(&mut mock),
            Err(PartitionError::InvalidTable)
        );

        let mut mock = MockFlash::<9>::new();
        let wrap = make_entry(PART_TYPE_APP, PART_SUBTYPE_OTA_MIN, 0xFFFF_0000, 0x2_0000);
        write_entries_with_md5(&mut mock, &[wrap]);
        assert_eq!(
            parse_partition_table(&mut mock),
            Err(PartitionError::InvalidTable)
        );

        let mut mock = MockFlash::<9>::new();
        let oob = make_entry(PART_TYPE_APP, PART_SUBTYPE_OTA_MIN, 0x380000, 0x100000);
        write_entries_with_md5(&mut mock, &[oob]);
        assert_eq!(
            parse_partition_table(&mut mock),
            Err(PartitionError::InvalidTable)
        );
    }

    #[test]
    fn rejects_offset_below_0x9000_and_unaligned_app_or_otadata() {
        let mut mock = MockFlash::<9>::new();
        let below = make_entry(PART_TYPE_DATA, PART_SUBTYPE_DATA_OTA, 0x1000, 0x2000);
        let ota0 = make_entry(PART_TYPE_APP, PART_SUBTYPE_OTA_MIN, 0x20000, 0x100000);
        write_entries_with_md5(&mut mock, &[below, ota0]);
        assert_eq!(
            parse_partition_table(&mut mock),
            Err(PartitionError::InvalidTable)
        );

        let mut mock = MockFlash::<9>::new();
        let unaligned_app = make_entry(PART_TYPE_APP, PART_SUBTYPE_OTA_MIN, 0x18000, 0x100000);
        write_entries_with_md5(&mut mock, &[unaligned_app]);
        assert_eq!(
            parse_partition_table(&mut mock),
            Err(PartitionError::InvalidTable)
        );

        let mut mock = MockFlash::<9>::new();
        let unaligned_ota = make_entry(PART_TYPE_DATA, PART_SUBTYPE_DATA_OTA, 0x9800, 0x2000);
        write_entries_with_md5(&mut mock, &[unaligned_ota, ota0]);
        assert_eq!(
            parse_partition_table(&mut mock),
            Err(PartitionError::InvalidTable)
        );

        let mut mock = MockFlash::<9>::new();
        let small_ota = make_entry(PART_TYPE_DATA, PART_SUBTYPE_DATA_OTA, 0x9000, 0x1000);
        write_entries_with_md5(&mut mock, &[small_ota, ota0]);
        assert_eq!(
            parse_partition_table(&mut mock),
            Err(PartitionError::InvalidTable)
        );
    }

    #[test]
    fn rejects_duplicate_subtypes_overlapping_spans_and_sparse_ota_slots() {
        let mut mock = MockFlash::<9>::new();
        let ota0_a = make_entry(PART_TYPE_APP, PART_SUBTYPE_OTA_MIN, 0x20000, 0x100000);
        let ota0_b = make_entry(PART_TYPE_APP, PART_SUBTYPE_OTA_MIN, 0x120000, 0x100000);
        write_entries_with_md5(&mut mock, &[ota0_a, ota0_b]);
        assert_eq!(
            parse_partition_table(&mut mock),
            Err(PartitionError::InvalidTable)
        );

        let mut mock = MockFlash::<9>::new();
        let ota1_overlap = make_entry(PART_TYPE_APP, PART_SUBTYPE_OTA_MIN + 1, 0x100000, 0x100000);
        write_entries_with_md5(&mut mock, &[ota0_a, ota1_overlap]);
        assert_eq!(
            parse_partition_table(&mut mock),
            Err(PartitionError::InvalidTable)
        );

        let mut mock = MockFlash::<9>::new();
        let ota2_gap = make_entry(PART_TYPE_APP, PART_SUBTYPE_OTA_MIN + 2, 0x120000, 0x100000);
        write_entries_with_md5(&mut mock, &[ota0_a, ota2_gap]);
        assert_eq!(
            parse_partition_table(&mut mock),
            Err(PartitionError::InvalidTable)
        );
    }
}
