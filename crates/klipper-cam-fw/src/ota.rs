//! Partition layout, constant-time OTA token verification, ESP32 firmware
//! image header validation (`0xE9` magic), and `otadata` IEEE 802.3 CRC32
//! rollback state machine.
//!
//! Replaces the legacy OTA subsystem with native Rust modules matching
//! `partitions_rust_ota.csv`.

#![forbid(unsafe_code)]

use core::fmt;

/// Flash offset of the `nvs` partition (`0x11000`).
pub const NVS_OFFSET: u32 = 0x0001_1000;
/// Size of the `nvs` partition (`0x6000` = 24,576 bytes).
pub const NVS_SIZE: u32 = 0x0000_6000;

/// Flash offset of the `otadata` partition (`0x17000`).
pub const OTADATA_OFFSET: u32 = 0x0001_7000;
/// Size of the `otadata` partition (`0x2000` = 8,192 bytes, 2 x 4KB sectors).
pub const OTADATA_SIZE: u32 = 0x0000_2000;
/// Flash offset of `otadata` sector 0 (`0x17000`).
pub const OTADATA_SECTOR_0_OFFSET: u32 = 0x0001_7000;
/// Flash offset of `otadata` sector 1 (`0x18000`).
pub const OTADATA_SECTOR_1_OFFSET: u32 = 0x0001_8000;
/// SPI NOR flash sector size (`4096` bytes).
pub const FLASH_SECTOR_SIZE: u32 = 0x0000_1000;

/// Flash offset of the `phy_init` partition (`0x19000`).
pub const PHY_INIT_OFFSET: u32 = 0x0001_9000;
/// Size of the `phy_init` partition (`0x1000` = 4,096 bytes).
pub const PHY_INIT_SIZE: u32 = 0x0000_1000;

/// Flash offset of the `ota_0` application partition (`0x20000`).
pub const OTA_0_OFFSET: u32 = 0x0002_0000;
/// Size of the `ota_0` application partition (`0x1B0000` = 1,769,472 bytes).
pub const OTA_0_SIZE: u32 = 0x001B_0000;

/// Flash offset of the `ota_1` application partition (`0x1D0000`).
pub const OTA_1_OFFSET: u32 = 0x001D_0000;
/// Size of the `ota_1` application partition (`0x1B0000` = 1,769,472 bytes).
pub const OTA_1_SIZE: u32 = 0x001B_0000;

/// Maximum firmware image size per OTA slot (`0x1B0000` = 1,769,472 bytes).
pub const OTA_PARTITION_MAX_SIZE: usize = 0x001B_0000;

/// Total SPI flash capacity on ESP32-WROVER-DEV (`4 MB` = `0x400000` bytes).
pub const FLASH_TOTAL_SIZE: u32 = 0x0040_0000;

/// Mandatory magic byte at offset `0` of every valid ESP32 application
/// image (`esp_image_header_t`).
pub const ESP_IMAGE_HEADER_MAGIC: u8 = 0xE9;

/// Minimum valid segment count in an ESP32 application image header (`1`).
pub const ESP_IMAGE_MIN_SEGMENTS: u8 = 1;

/// Maximum valid segment count in an ESP32 application image header (`16`).
pub const ESP_IMAGE_MAX_SEGMENTS: u8 = 16;

/// Size in bytes of the fixed `esp_image_header_t` prefix (`24` bytes).
pub const ESP_IMAGE_HEADER_LEN: usize = 24;

/// `esp_image_header_t::chip_id` value for the original ESP32
/// (`ESP_CHIP_ID_ESP32`). Any other value (e.g. `0x0009` for ESP32-S3) is
/// rejected by [`validate_ota_image_header`] (`FW-NET-18`).
pub const ESP_IMAGE_CHIP_ID_ESP32: u16 = 0x0000;

/// ESP32 internal instruction RAM (`SOC_IRAM_LOW..SOC_IRAM_HIGH`), a valid
/// application entry-point region.
pub const ESP32_IRAM_ENTRY_RANGE: core::ops::Range<u32> = 0x4007_0000..0x400A_0000;

/// ESP32 flash-mapped instruction region (`SOC_IROM_LOW..SOC_IROM_HIGH`), a
/// valid application entry-point region.
pub const ESP32_IROM_ENTRY_RANGE: core::ops::Range<u32> = 0x400D_0000..0x4040_0000;

/// Length of the SHA-256 digest ESP-IDF appends after the checksum block when
/// `esp_image_header_t::hash_appended == 1`.
pub const ESP_IMAGE_DIGEST_LEN: usize = 32;

/// Maximum chunk size read per socket receive step during OTA
/// (`1024` bytes, `klipper_cam.c`).
pub const OTA_CHUNK_SIZE: usize = 1024;

/// Maximum consecutive socket timeout retries during OTA upload
/// (`5`, `APP-007`).
pub const OTA_MAX_TIMEOUT_RETRIES: u8 = 5;

/// Partition major type (`esp_partition_type_t`).
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PartitionType {
    /// Application partition (`0x00`).
    App = 0x00,
    /// Data partition (`0x01`).
    Data = 0x01,
}

/// Partition subtype (`esp_partition_subtype_t`).
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PartitionSubType {
    /// OTA selection data partition (`0x00`).
    Ota = 0x00,
    /// PHY initialization data partition (`0x01`).
    Phy = 0x01,
    /// Non-volatile storage partition (`0x02`).
    Nvs = 0x02,
    /// OTA slot 0 application (`0x10`).
    Ota0 = 0x10,
    /// OTA slot 1 application (`0x11`).
    Ota1 = 0x11,
}

/// Dual-slot OTA application partition identifier (`ota_0` or `ota_1`).
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum OtaSlot {
    /// Primary OTA slot (`ota_0` @ `0x20000`).
    #[default]
    Ota0 = 0,
    /// Secondary OTA slot (`ota_1` @ `0x1D0000`).
    Ota1 = 1,
}

impl OtaSlot {
    /// Returns the partition label (`"ota_0"` or `"ota_1"`).
    #[inline]
    pub const fn label(self) -> &'static str {
        match self {
            Self::Ota0 => "ota_0",
            Self::Ota1 => "ota_1",
        }
    }

    /// Returns the opposite partition slot (`Ota0` <-> `Ota1`).
    #[inline]
    pub const fn opposite(self) -> Self {
        match self {
            Self::Ota0 => Self::Ota1,
            Self::Ota1 => Self::Ota0,
        }
    }

    /// Returns the passive target slot to write during an OTA update.
    #[inline]
    pub const fn next_slot(self) -> Self {
        self.opposite()
    }

    /// Returns the static [`PartitionEntry`] for this slot.
    #[inline]
    pub const fn partition(self) -> &'static PartitionEntry {
        match self {
            Self::Ota0 => &PARTITION_TABLE[3],
            Self::Ota1 => &PARTITION_TABLE[4],
        }
    }

    /// Resolves an [`OtaSlot`] from its string label (`"ota_0"` or `"ota_1"`).
    pub fn from_label(label: &str) -> Option<Self> {
        match label {
            "ota_0" => Some(Self::Ota0),
            "ota_1" => Some(Self::Ota1),
            _ => None,
        }
    }

    /// Maps an ESP-IDF 1-based `ota_seq` counter to the corresponding [`OtaSlot`].
    ///
    /// In ESP-IDF `bootloader_common_get_active_otadata`:
    /// `(ota_seq - 1) % num_ota_partitions` selects the slot
    /// (`1, 3, 5 -> ota_0`; `2, 4, 6 -> ota_1`).
    #[inline]
    pub const fn from_seq(ota_seq: u32) -> Option<Self> {
        if ota_seq == 0 || ota_seq == u32::MAX {
            return None;
        }
        if ((ota_seq - 1) & 1) == 0 {
            Some(Self::Ota0)
        } else {
            Some(Self::Ota1)
        }
    }
}

/// Static flash partition descriptor matching
/// `partitions_rust_ota.csv`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct PartitionEntry {
    pub label: &'static str,
    pub part_type: PartitionType,
    pub subtype: PartitionSubType,
    pub offset: u32,
    pub size: u32,
}

impl PartitionEntry {
    /// Returns the exclusive end address (`offset + size`) of this partition.
    #[inline]
    pub const fn end_offset(&self) -> u32 {
        self.offset + self.size
    }
}

/// Complete 5-entry partition table matching
/// `partitions_rust_ota.csv`:
/// - `nvs`      @ `0x011000` (`0x006000`)
/// - `otadata`  @ `0x017000` (`0x002000`)
/// - `phy_init` @ `0x019000` (`0x001000`)
/// - `ota_0`    @ `0x020000` (`0x1B0000` = `1,769,472` bytes)
/// - `ota_1`    @ `0x1D0000` (`0x1B0000` = `1,769,472` bytes)
pub const PARTITION_TABLE: [PartitionEntry; 5] = [
    PartitionEntry {
        label: "nvs",
        part_type: PartitionType::Data,
        subtype: PartitionSubType::Nvs,
        offset: NVS_OFFSET,
        size: NVS_SIZE,
    },
    PartitionEntry {
        label: "otadata",
        part_type: PartitionType::Data,
        subtype: PartitionSubType::Ota,
        offset: OTADATA_OFFSET,
        size: OTADATA_SIZE,
    },
    PartitionEntry {
        label: "phy_init",
        part_type: PartitionType::Data,
        subtype: PartitionSubType::Phy,
        offset: PHY_INIT_OFFSET,
        size: PHY_INIT_SIZE,
    },
    PartitionEntry {
        label: "ota_0",
        part_type: PartitionType::App,
        subtype: PartitionSubType::Ota0,
        offset: OTA_0_OFFSET,
        size: OTA_0_SIZE,
    },
    PartitionEntry {
        label: "ota_1",
        part_type: PartitionType::App,
        subtype: PartitionSubType::Ota1,
        offset: OTA_1_OFFSET,
        size: OTA_1_SIZE,
    },
];

/// Looks up a partition in [`PARTITION_TABLE`] by label.
pub fn find_partition_by_label(label: &str) -> Option<&'static PartitionEntry> {
    let mut i = 0;
    while i < PARTITION_TABLE.len() {
        if PARTITION_TABLE[i].label.as_bytes() == label.as_bytes() {
            return Some(&PARTITION_TABLE[i]);
        }
        i += 1;
    }
    None
}

/// Minimum safe flash offset after the ESP32 bootloader (`0x1000`) and
/// partition table (`0x8000..0x9000`) (`M-135` / `C1-037`).
pub const PARTITION_MIN_OFFSET: u32 = 0x0000_9000;

/// Mandatory 64KB MMU alignment for ESP32 application (`ota_0` / `ota_1`)
/// partitions (`M-135` / `C1-037`).
pub const APP_PARTITION_ALIGN: u32 = 0x0001_0000;

/// Verifies that all entries in [`PARTITION_TABLE`] are 4KB-aligned, above
/// the bootloader/partition-table region (`>= 0x9000`), 64KB-aligned for
/// application partitions, have non-empty unique labels, are strictly ordered,
/// non-overlapping, and fit within the 4MB SPI flash (`0x400000`).
pub const fn verify_partition_table_integrity() -> bool {
    let mut i = 0;
    while i < PARTITION_TABLE.len() {
        let entry = PARTITION_TABLE[i];
        if entry.label.is_empty() {
            return false;
        }
        if entry.offset < PARTITION_MIN_OFFSET {
            return false;
        }
        if (entry.offset & 0xFFF) != 0 || (entry.size & 0xFFF) != 0 || entry.size == 0 {
            return false;
        }
        if matches!(entry.part_type, PartitionType::App)
            && (entry.offset & (APP_PARTITION_ALIGN - 1)) != 0
        {
            return false;
        }
        let end = match entry.offset.checked_add(entry.size) {
            Some(e) => e,
            None => return false,
        };
        if end > FLASH_TOTAL_SIZE {
            return false;
        }
        if i + 1 < PARTITION_TABLE.len() {
            let next = PARTITION_TABLE[i + 1];
            if end > next.offset {
                return false;
            }
        }
        let mut j = i + 1;
        while j < PARTITION_TABLE.len() {
            if const_str_eq(entry.label, PARTITION_TABLE[j].label) {
                return false;
            }
            j += 1;
        }
        i += 1;
    }
    true
}

/// Const-compatible string equality helper for partition table checks.
const fn const_str_eq(a: &str, b: &str) -> bool {
    let ab = a.as_bytes();
    let bb = b.as_bytes();
    if ab.len() != bb.len() {
        return false;
    }
    let mut i = 0;
    while i < ab.len() {
        if ab[i] != bb[i] {
            return false;
        }
        i += 1;
    }
    true
}

/// Constant-time OTA authentication token comparator (`FW-010` / `APP-002` /
/// `SEC-170` / `M-030`).
///
/// Uses `#[inline(never)]` and `core::hint::black_box` optimization barriers
/// across `max(provided.len(), expected.len())` with branchless byte masking
/// to prevent LLVM short-circuiting or `memcmp` lowering.
/// Rejects empty `provided` or empty `expected` tokens unconditionally.
#[inline(never)]
pub fn verify_ota_token(provided: &[u8], expected: &[u8]) -> bool {
    if expected.is_empty() || provided.is_empty() {
        return false;
    }
    let mut diff: usize = core::hint::black_box(provided.len() ^ expected.len());
    let max_len = if provided.len() > expected.len() {
        provided.len()
    } else {
        expected.len()
    };
    let mut i: usize = 0;
    while i < max_len {
        let p_mask = ((i < provided.len()) as usize).wrapping_neg();
        let e_mask = ((i < expected.len()) as usize).wrapping_neg();
        let p_byte = core::hint::black_box(provided[i & p_mask]) & (p_mask as u8);
        let e_byte = core::hint::black_box(expected[i & e_mask]) & (e_mask as u8);
        diff = core::hint::black_box(diff | ((p_byte ^ e_byte) as usize));
        i += 1;
    }
    core::hint::black_box(diff) == 0
}

/// 16-entry nibble lookup table for IEEE 802.3 CRC-32 (polynomial
/// `0xEDB8_8320`, `PERF-170`).
pub const CRC32_NIBBLE_LUT: [u32; 16] = [
    0x0000_0000,
    0x1DB7_1064,
    0x3B6E_20C8,
    0x26D9_30AC,
    0x76DC_4190,
    0x6B6B_51F4,
    0x4DB2_6158,
    0x5005_713C,
    0xEDB8_8320,
    0xF00F_9344,
    0xD6D6_A3E8,
    0xCB61_B38C,
    0x9B64_C2B0,
    0x86D3_D2D4,
    0xA00A_E278,
    0xBDBD_F21C,
];

/// Computes the standard IEEE 802.3 CRC-32 checksum over `data`
/// (polynomial `0xEDB88320`, initial value `0xFFFFFFFF`, final XOR
/// `0xFFFFFFFF`, `PERF-170`).
pub const fn crc32_ieee(data: &[u8]) -> u32 {
    let mut crc: u32 = 0xFFFF_FFFF;
    let mut i: usize = 0;
    while i < data.len() {
        crc ^= data[i] as u32;
        crc = (crc >> 4) ^ CRC32_NIBBLE_LUT[(crc & 0x0F) as usize];
        crc = (crc >> 4) ^ CRC32_NIBBLE_LUT[(crc & 0x0F) as usize];
        i += 1;
    }
    !crc
}

/// Computes the ESP-IDF / ESP32 ROM `esp_rom_crc32_le(u32::MAX, data)`
/// checksum used by `bootloader_common_ota_select_crc` (`init = 0x0000_0000`,
/// reflected polynomial `0xEDB8_8320`, final XOR `0xFFFF_FFFF`, `PERF-170`).
pub const fn crc32_esp_ota(data: &[u8]) -> u32 {
    let mut crc: u32 = 0x0000_0000;
    let mut i: usize = 0;
    while i < data.len() {
        crc ^= data[i] as u32;
        crc = (crc >> 4) ^ CRC32_NIBBLE_LUT[(crc & 0x0F) as usize];
        crc = (crc >> 4) ^ CRC32_NIBBLE_LUT[(crc & 0x0F) as usize];
        i += 1;
    }
    !crc
}

/// OTA image lifecycle states stored in `otadata` (`esp_ota_img_states_t`).
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum OtaImgState {
    /// Monitor the first boot; bootloader transitions this to
    /// `PendingVerify` (`0x0`).
    New = 0x0,
    /// First boot after OTA; requires `mark_app_valid_cancel_rollback()`
    /// before next reboot (`0x1`).
    PendingVerify = 0x1,
    /// Image confirmed operational; rollback cancelled (`0x2`).
    Valid = 0x2,
    /// Image rolled back or rejected (`0x3`).
    Invalid = 0x3,
    /// Image verification aborted (`0x4`).
    Aborted = 0x4,
    /// Erased / uninitialized state (`0xFFFFFFFF`).
    Undefined = 0xFFFF_FFFF,
}

impl OtaImgState {
    /// Converts a raw 32-bit `ota_state` field into an [`OtaImgState`].
    /// Maps unknown values (`0x5..=0xFFFF_FFFE`) to [`Self::Invalid`] (`OTA-002`).
    #[inline]
    pub const fn from_u32(val: u32) -> Self {
        match val {
            0x0 => Self::New,
            0x1 => Self::PendingVerify,
            0x2 => Self::Valid,
            0x3 => Self::Invalid,
            0x4 => Self::Aborted,
            0xFFFF_FFFF => Self::Undefined,
            _ => Self::Invalid,
        }
    }
}

/// 32-byte `otadata` sector selection record (`esp_ota_select_entry_t`,
/// `EMB-003`).
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct OtaDataSelect {
    /// Monotonically increasing sequence counter (`1..u32::MAX - 1`).
    pub ota_seq: u32,
    /// Reserved 20-byte label/padding array (`0xFF` filled).
    pub seq_label: [u8; 20],
    /// Raw [`OtaImgState`] value.
    pub ota_state: u32,
    /// ESP-IDF `bootloader_common_ota_select_crc` (`crc32_esp_ota`) over
    /// `ota_seq` in little-endian byte order (`M-136` / `C1-038`).
    pub crc: u32,
}

impl OtaDataSelect {
    /// Size in bytes of an [`OtaDataSelect`] structure (`32` bytes).
    pub const SIZE: usize = 32;

    /// Returns an erased (`0xFF`-filled) [`OtaDataSelect`] matching
    /// blank NOR flash.
    #[inline]
    pub const fn erased() -> Self {
        Self {
            ota_seq: u32::MAX,
            seq_label: [0xFF; 20],
            ota_state: OtaImgState::Undefined as u32,
            crc: u32::MAX,
        }
    }

    /// Creates a new [`OtaDataSelect`] with validated `ota_seq`
    /// (`1..=u32::MAX - 1`), `state`, and valid ESP-IDF `otadata` CRC-32,
    /// returning `None` if `ota_seq` is `0` or `u32::MAX` (`M-136`).
    #[inline]
    pub const fn try_new(ota_seq: u32, state: OtaImgState) -> Option<Self> {
        if ota_seq == 0 || ota_seq == u32::MAX {
            return None;
        }
        let crc = crc32_esp_ota(&ota_seq.to_le_bytes());
        Some(Self {
            ota_seq,
            seq_label: [0xFF; 20],
            ota_state: state as u32,
            crc,
        })
    }

    /// Creates a new [`OtaDataSelect`] with `ota_seq`, `state`, and
    /// valid ESP-IDF `otadata` CRC-32.
    ///
    /// # Panics
    ///
    /// Panics if `ota_seq` is `0` or `u32::MAX` (`M-136` / `C1-038`).
    #[inline]
    pub const fn new(ota_seq: u32, state: OtaImgState) -> Self {
        assert!(
            ota_seq != 0 && ota_seq != u32::MAX,
            "ota_seq must be in 1..u32::MAX"
        );
        let crc = crc32_esp_ota(&ota_seq.to_le_bytes());
        Self {
            ota_seq,
            seq_label: [0xFF; 20],
            ota_state: state as u32,
            crc,
        }
    }

    /// Calculates the ESP-IDF `bootloader_common_ota_select_crc` over the
    /// little-endian representation of `ota_seq`.
    #[inline]
    pub const fn calculate_crc(&self) -> u32 {
        crc32_esp_ota(&self.ota_seq.to_le_bytes())
    }

    /// Returns `true` if `ota_seq` is initialized (`!= 0` and
    /// `!= u32::MAX`) and `crc` matches.
    #[inline]
    pub const fn is_crc_valid(&self) -> bool {
        self.ota_seq != 0 && self.ota_seq != u32::MAX && self.crc == self.calculate_crc()
    }

    /// Returns the parsed [`OtaImgState`] of this record.
    #[inline]
    pub const fn state(&self) -> OtaImgState {
        OtaImgState::from_u32(self.ota_state)
    }

    /// Returns `true` if this record has a valid CRC and is in a bootable state
    /// (`New`, `PendingVerify`, `Valid`, or `Undefined` for legacy bootloaders).
    #[inline]
    pub const fn is_bootable(&self) -> bool {
        if !self.is_crc_valid() {
            return false;
        }
        matches!(
            self.state(),
            OtaImgState::New
                | OtaImgState::PendingVerify
                | OtaImgState::Valid
                | OtaImgState::Undefined
        )
    }

    /// Returns the [`OtaSlot`] selected by `self.ota_seq` if valid.
    #[inline]
    pub const fn slot(&self) -> Option<OtaSlot> {
        if !self.is_crc_valid() {
            return None;
        }
        OtaSlot::from_seq(self.ota_seq)
    }

    /// Marks this `otadata` record as [`OtaImgState::Valid`]
    /// (`esp_ota_mark_app_valid_cancel_rollback`, `EMB-003`).
    pub fn mark_app_valid_cancel_rollback(&mut self) -> Result<bool, OtaError> {
        if !self.is_crc_valid() {
            return Err(OtaError::InvalidOtaDataCrc);
        }
        match self.state() {
            OtaImgState::PendingVerify | OtaImgState::New | OtaImgState::Undefined => {
                self.ota_state = OtaImgState::Valid as u32;
                self.crc = self.calculate_crc();
                Ok(true)
            }
            OtaImgState::Valid => Ok(false),
            OtaImgState::Invalid | OtaImgState::Aborted => Err(OtaError::RollbackStateInvalid),
        }
    }

    /// Serializes this record verbatim into a 32-byte little-endian array
    /// (`M-032` / `C1-011`: preserves `New` vs `PendingVerify` without silent
    /// rewriting).
    pub const fn to_bytes(&self) -> [u8; 32] {
        let mut out = [0xFFu8; 32];
        let seq_b = self.ota_seq.to_le_bytes();
        out[0] = seq_b[0];
        out[1] = seq_b[1];
        out[2] = seq_b[2];
        out[3] = seq_b[3];

        let mut i = 0;
        while i < 20 {
            out[4 + i] = self.seq_label[i];
            i += 1;
        }

        let st_b = self.ota_state.to_le_bytes();
        out[24] = st_b[0];
        out[25] = st_b[1];
        out[26] = st_b[2];
        out[27] = st_b[3];

        let crc_b = self.crc.to_le_bytes();
        out[28] = crc_b[0];
        out[29] = crc_b[1];
        out[30] = crc_b[2];
        out[31] = crc_b[3];
        out
    }

    /// Deserializes a 32-byte little-endian array verbatim into an
    /// [`OtaDataSelect`] (`M-032` / `C1-011`: preserves `New` vs
    /// `PendingVerify` without silent rewriting).
    pub const fn from_bytes(bytes: &[u8; 32]) -> Self {
        let ota_seq = u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
        let mut seq_label = [0u8; 20];
        let mut i = 0;
        while i < 20 {
            seq_label[i] = bytes[4 + i];
            i += 1;
        }
        let ota_state = u32::from_le_bytes([bytes[24], bytes[25], bytes[26], bytes[27]]);
        let crc = u32::from_le_bytes([bytes[28], bytes[29], bytes[30], bytes[31]]);
        Self {
            ota_seq,
            seq_label,
            ota_state,
            crc,
        }
    }
}

impl Default for OtaDataSelect {
    #[inline]
    fn default() -> Self {
        Self::new(1, OtaImgState::Valid)
    }
}

/// 4-byte-aligned 32-byte buffer for `otadata` NOR flash reads and
/// writes (`esp-storage` alignment requirement).
#[repr(C, align(4))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AlignedOtaBuf(pub [u8; 32]);

impl Default for AlignedOtaBuf {
    #[inline]
    fn default() -> Self {
        Self([0xFF; 32])
    }
}

/// 4-byte-aligned 1024-byte chunk buffer for OTA application partition
/// flash writes (`esp-storage` alignment requirement).
#[repr(C, align(4))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AlignedFlashChunk(pub [u8; OTA_CHUNK_SIZE]);

impl Default for AlignedFlashChunk {
    #[inline]
    fn default() -> Self {
        Self([0xFF; OTA_CHUNK_SIZE])
    }
}

/// Multicore parking hooks used by [`Core1PauseGuard`] before SPI flash cache
/// disable.
pub trait Core1ParkControl {
    /// Parks Core 1 in IRAM and returns `true` once paused, or `false` on
    /// timeout.
    fn pause_core1(&self) -> bool;
    /// Resumes Core 1 execution and returns `true` once resumed, or `false` on
    /// timeout.
    fn resume_core1(&self) -> bool;
}

/// RAII guard that pauses Core 1 on creation and resumes it on explicit
/// [`Self::resume`] or drop (`FW-CORE-01` / `M-019` / `C1-014`).
///
/// Must not be nested on the same `Core1ParkControl` instance; prefer
/// [`Self::with_core1_paused`] or explicit [`Self::resume`] so resume
/// failures are propagated as `Err(OtaError::FlashWriteError)`.
#[must_use = "Core1PauseGuard resumes Core 1 immediately if dropped"]
pub struct Core1PauseGuard<'a, P: Core1ParkControl> {
    controller: &'a P,
    resumed: bool,
}

impl<'a, P: Core1ParkControl> Core1PauseGuard<'a, P> {
    /// Pauses Core 1, returning [`OtaError::FlashWriteError`] if parking times
    /// out (`M-019`). Rollback of any partial depth increment is the
    /// responsibility of [`Core1ParkControl::pause_core1`], so a failed nested
    /// pause never unparks an outer guard.
    pub fn new(controller: &'a P) -> Result<Self, OtaError> {
        if controller.pause_core1() {
            Ok(Self {
                controller,
                resumed: false,
            })
        } else {
            Err(OtaError::FlashWriteError)
        }
    }

    /// Explicitly resumes Core 1 and propagates any resume timeout as
    /// [`OtaError::FlashWriteError`] (`M-019`).
    pub fn resume(mut self) -> Result<(), OtaError> {
        self.resumed = true;
        if self.controller.resume_core1() {
            Ok(())
        } else {
            Err(OtaError::FlashWriteError)
        }
    }

    /// Executes `f` while Core 1 is parked, resuming Core 1 afterward and
    /// propagating both operation errors and resume failures (`M-019`).
    pub fn with_core1_paused<R>(
        controller: &'a P,
        f: impl FnOnce() -> Result<R, OtaError>,
    ) -> Result<R, OtaError> {
        let guard = Self::new(controller)?;
        let op_res = f();
        let resume_res = guard.resume();
        match (op_res, resume_res) {
            (Ok(val), Ok(())) => Ok(val),
            (Err(err), _) => Err(err),
            (Ok(_), Err(err)) => Err(err),
        }
    }
}

impl<P: Core1ParkControl> Drop for Core1PauseGuard<'_, P> {
    fn drop(&mut self) {
        if !self.resumed {
            let _ = self.controller.resume_core1();
        }
    }
}

/// [`OtaFlashStorage`] wrapper that acquires a [`Core1PauseGuard`] on every
/// `read_aligned`, `erase_range`, and `write_aligned` call (`FW-CORE-01`).
///
/// Inner `flash` and `park` fields are encapsulated (`M-034` / `C1-013`) so
/// callers cannot bypass Core 1 parking during active flash operations.
pub struct GuardedFlashStorage<F, P: Core1ParkControl> {
    flash: F,
    park: P,
}

impl<F, P: Core1ParkControl> GuardedFlashStorage<F, P> {
    #[must_use]
    pub const fn new(flash: F, park: P) -> Self {
        Self { flash, park }
    }

    /// Returns a shared reference to the parking controller metadata.
    #[must_use]
    pub const fn park(&self) -> &P {
        &self.park
    }

    /// Consumes the guard wrapper and returns the underlying `(flash, park)`.
    #[must_use]
    pub fn into_inner(self) -> (F, P) {
        (self.flash, self.park)
    }
}

impl<F: OtaFlashStorage, P: Core1ParkControl> OtaFlashStorage for GuardedFlashStorage<F, P> {
    fn read_aligned(&mut self, offset: u32, buf: &mut [u8]) -> Result<(), OtaError> {
        let guard = Core1PauseGuard::new(&self.park)?;
        let res = self.flash.read_aligned(offset, buf);
        let resume_res = guard.resume();
        res.and(resume_res)
    }

    fn erase_range(&mut self, from: u32, to: u32) -> Result<(), OtaError> {
        let guard = Core1PauseGuard::new(&self.park)?;
        let res = self.flash.erase_range(from, to);
        let resume_res = guard.resume();
        res.and(resume_res)
    }

    fn write_aligned(&mut self, offset: u32, data: &[u8]) -> Result<(), OtaError> {
        let guard = Core1PauseGuard::new(&self.park)?;
        let res = self.flash.write_aligned(offset, data);
        let resume_res = guard.resume();
        res.and(resume_res)
    }

    fn has_persistent_otadata(&self) -> bool {
        self.flash.has_persistent_otadata()
    }

    fn supports_readback(&self) -> bool {
        self.flash.supports_readback()
    }
}

/// Hardware abstraction trait for SPI NOR flash `otadata` and OTA
/// partition reads, erases, and writes.
///
/// Designed to wrap `esp_storage::FlashStorage` on Xtensa targets while
/// enabling deterministic `#![no_std]` mock NOR flash verification on the
/// host.
pub trait OtaFlashStorage {
    /// Reads `buf.len()` bytes starting at `offset` (`offset` and
    /// `buf.len()` must be 4-byte aligned).
    fn read_aligned(&mut self, offset: u32, buf: &mut [u8]) -> Result<(), OtaError>;

    /// Erases the flash range `[from, to)` (`from` and `to` must be
    /// 4096-byte sector aligned).
    fn erase_range(&mut self, from: u32, to: u32) -> Result<(), OtaError>;

    /// Writes `data` starting at `offset` (`offset` and `data.len()`
    /// must be 4-byte aligned).
    fn write_aligned(&mut self, offset: u32, data: &[u8]) -> Result<(), OtaError>;

    /// Returns `true` if this storage backend persists real `otadata` across
    /// reboots.
    fn has_persistent_otadata(&self) -> bool {
        true
    }

    /// Returns `true` if [`OtaFlashStorage::read_aligned`] returns the bytes
    /// actually programmed, so the OTA write path can read the image back and
    /// re-hash it before committing `otadata` (`FW-NET-18`).
    fn supports_readback(&self) -> bool {
        true
    }
}

/// No-op [`OtaFlashStorage`] implementation for in-memory HTTP/OTA state
/// machine operation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct NoOpFlashStorage;

impl OtaFlashStorage for NoOpFlashStorage {
    #[inline]
    fn read_aligned(&mut self, _offset: u32, _buf: &mut [u8]) -> Result<(), OtaError> {
        Ok(())
    }

    #[inline]
    fn erase_range(&mut self, _from: u32, _to: u32) -> Result<(), OtaError> {
        Ok(())
    }

    #[inline]
    fn write_aligned(&mut self, _offset: u32, _data: &[u8]) -> Result<(), OtaError> {
        Ok(())
    }

    #[inline]
    fn has_persistent_otadata(&self) -> bool {
        false
    }

    #[inline]
    fn supports_readback(&self) -> bool {
        false
    }
}

/// Deterministic `#![no_std]` SPI NOR flash simulator covering
/// `otadata` (`0x17000..0x19000`) and the leading `APP_WINDOW` bytes of
/// `ota_0` (`0x20000`) and `ota_1` (`0x1D0000`).
///
/// Enforces hardware NOR flash invariants:
/// - `read_aligned` and `write_aligned` require 4-byte aligned `offset` and
///   buffer length.
/// - `erase_range` requires 4096-byte (`0x1000`) sector-aligned `from` and
///   `to` with `from < to`.
/// - `erase_range` resets bytes to `0xFF`, and `write_aligned` programs
///   bits via `dst &= src`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MockNorFlash<const APP_WINDOW: usize = 8192> {
    pub otadata_s0: [u8; FLASH_SECTOR_SIZE as usize],
    pub otadata_s1: [u8; FLASH_SECTOR_SIZE as usize],
    pub ota_0: [u8; APP_WINDOW],
    pub ota_1: [u8; APP_WINDOW],
    pub erase_count: u32,
    pub write_count: u32,
    pub last_erased_range: Option<(u32, u32)>,
    pub fail_writes: bool,
}

impl<const APP_WINDOW: usize> MockNorFlash<APP_WINDOW> {
    /// Creates a blank (`0xFF`-erased) [`MockNorFlash`] instance.
    pub const fn new() -> Self {
        Self {
            otadata_s0: [0xFF; FLASH_SECTOR_SIZE as usize],
            otadata_s1: [0xFF; FLASH_SECTOR_SIZE as usize],
            ota_0: [0xFF; APP_WINDOW],
            ota_1: [0xFF; APP_WINDOW],
            erase_count: 0,
            write_count: 0,
            last_erased_range: None,
            fail_writes: false,
        }
    }

    /// Creates a [`MockNorFlash`] initialized with `ota_0` active
    /// (`ota_seq = 1`, `Valid`) in `otadata` sector 0.
    pub const fn new_booted_ota0() -> Self {
        let mut flash = Self::new();
        let bytes = OtaDataSelect::new(1, OtaImgState::Valid).to_bytes();
        let mut i = 0;
        while i < 32 {
            flash.otadata_s0[i] = bytes[i];
            i += 1;
        }
        flash
    }

    fn region_mut(&mut self, offset: u32, len: usize) -> Result<&mut [u8], OtaError> {
        let len_u32 = u32::try_from(len).map_err(|_| OtaError::FlashWriteError)?;
        let end = offset
            .checked_add(len_u32)
            .ok_or(OtaError::FlashWriteError)?;
        if offset >= OTADATA_SECTOR_0_OFFSET && end <= OTADATA_SECTOR_0_OFFSET + FLASH_SECTOR_SIZE {
            let start = (offset - OTADATA_SECTOR_0_OFFSET) as usize;
            Ok(&mut self.otadata_s0[start..start + len])
        } else if offset >= OTADATA_SECTOR_1_OFFSET
            && end <= OTADATA_SECTOR_1_OFFSET + FLASH_SECTOR_SIZE
        {
            let start = (offset - OTADATA_SECTOR_1_OFFSET) as usize;
            Ok(&mut self.otadata_s1[start..start + len])
        } else if offset >= OTA_0_OFFSET && end <= OTA_0_OFFSET + (APP_WINDOW as u32) {
            let start = (offset - OTA_0_OFFSET) as usize;
            Ok(&mut self.ota_0[start..start + len])
        } else if offset >= OTA_1_OFFSET && end <= OTA_1_OFFSET + (APP_WINDOW as u32) {
            let start = (offset - OTA_1_OFFSET) as usize;
            Ok(&mut self.ota_1[start..start + len])
        } else {
            Err(OtaError::FlashWriteError)
        }
    }
}

impl<const APP_WINDOW: usize> Default for MockNorFlash<APP_WINDOW> {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

impl<const APP_WINDOW: usize> OtaFlashStorage for MockNorFlash<APP_WINDOW> {
    fn read_aligned(&mut self, offset: u32, buf: &mut [u8]) -> Result<(), OtaError> {
        if (offset & 3) != 0 || (buf.len() & 3) != 0 {
            return Err(OtaError::FlashWriteError);
        }
        let len = buf.len();
        let slice = self.region_mut(offset, len)?;
        buf.copy_from_slice(slice);
        Ok(())
    }

    fn erase_range(&mut self, from: u32, to: u32) -> Result<(), OtaError> {
        if self.fail_writes
            || (from & (FLASH_SECTOR_SIZE - 1)) != 0
            || (to & (FLASH_SECTOR_SIZE - 1)) != 0
            || from >= to
        {
            return Err(OtaError::FlashWriteError);
        }
        let len = (to - from) as usize;
        let slice = self.region_mut(from, len)?;
        slice.fill(0xFF);
        self.erase_count = self.erase_count.saturating_add(1);
        self.last_erased_range = Some((from, to));
        Ok(())
    }

    fn write_aligned(&mut self, offset: u32, data: &[u8]) -> Result<(), OtaError> {
        if self.fail_writes || (offset & 3) != 0 || (data.len() & 3) != 0 {
            return Err(OtaError::FlashWriteError);
        }
        let slice = self.region_mut(offset, data.len())?;
        for (dst, &src) in slice.iter_mut().zip(data.iter()) {
            *dst &= src;
        }
        self.write_count = self.write_count.saturating_add(1);
        Ok(())
    }
}

/// Returns `true` if `seq_a` is newer than `seq_b` under ESP-IDF bootloader
/// unsigned sequence ordering over valid sequences `1..=u32::MAX - 1`
/// (`M-033` / `C1-012`).
///
/// Forms a strict weak ordering:
/// - Any valid sequence (`1..=u32::MAX - 1`) is greater than an invalid
///   sequence (`0` or `u32::MAX`).
/// - Between two valid sequences, plain unsigned `seq_a > seq_b` is used,
///   matching ESP-IDF `bootloader_common_get_active_otadata`.
/// - Two invalid sequences are incomparable (`false`).
#[inline]
pub const fn is_seq_greater(seq_a: u32, seq_b: u32) -> bool {
    let a_valid = seq_a != 0 && seq_a != u32::MAX;
    let b_valid = seq_b != 0 && seq_b != u32::MAX;
    match (a_valid, b_valid) {
        (true, true) => seq_a > seq_b,
        (true, false) => true,
        (false, _) => false,
    }
}

/// Typed `otadata` flash sector selector (`0x17000` vs `0x18000`,
/// `M-137` / `C1-039`).
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum OtaDataSector {
    /// First 4KB `otadata` sector at `0x17000`.
    Sector0 = 0,
    /// Second 4KB `otadata` sector at `0x18000`.
    Sector1 = 1,
}

impl OtaDataSector {
    /// Returns the SPI flash byte offset of this `otadata` sector.
    #[inline]
    pub const fn offset(self) -> u32 {
        match self {
            Self::Sector0 => OTADATA_SECTOR_0_OFFSET,
            Self::Sector1 => OTADATA_SECTOR_1_OFFSET,
        }
    }

    /// Returns the numeric sector index (`0` or `1`).
    #[inline]
    pub const fn index(self) -> usize {
        self as usize
    }

    /// Returns the opposite `otadata` sector.
    #[inline]
    pub const fn opposite(self) -> Self {
        match self {
            Self::Sector0 => Self::Sector1,
            Self::Sector1 => Self::Sector0,
        }
    }

    /// Converts a raw numeric sector index (`0` or `1`) to an [`OtaDataSector`].
    #[inline]
    pub const fn from_index(index: usize) -> Option<Self> {
        match index {
            0 => Some(Self::Sector0),
            1 => Some(Self::Sector1),
            _ => None,
        }
    }
}

/// Stateful streaming OTA write session enforcing partition bounds, header
/// validation, sequential writes, running SHA-256, and full flash readback
/// integrity verification before `otadata` commit (`M-035` / `C1-015`).
pub struct OtaWriteSession {
    target_slot: OtaSlot,
    expected_len: usize,
    bytes_written: usize,
    partition_offset: u32,
    partition_size: u32,
    header_buf: [u8; ESP_IMAGE_HEADER_LEN],
    header_len: usize,
    header_validated: bool,
    hasher: Sha256,
}

impl OtaWriteSession {
    /// Returns the passive target [`OtaSlot`] being written.
    #[inline]
    pub const fn target_slot(&self) -> OtaSlot {
        self.target_slot
    }

    /// Returns the expected total firmware image length in bytes.
    #[inline]
    pub const fn expected_len(&self) -> usize {
        self.expected_len
    }

    /// Returns the number of firmware payload bytes written so far.
    #[inline]
    pub const fn bytes_written(&self) -> usize {
        self.bytes_written
    }

    /// Writes the next sequential chunk of firmware bytes into the erased
    /// passive partition, padding any final non-4-byte-aligned tail with `0xFF`.
    pub fn write_chunk<F: OtaFlashStorage>(
        &mut self,
        flash: &mut F,
        chunk: &[u8],
    ) -> Result<(), OtaError> {
        if chunk.is_empty() {
            return Ok(());
        }
        let next_written =
            self.bytes_written
                .checked_add(chunk.len())
                .ok_or(OtaError::ImageTooLarge {
                    image_size: usize::MAX,
                    partition_size: self.partition_size as usize,
                })?;
        if next_written > self.expected_len {
            return Err(OtaError::ImageTooLarge {
                image_size: next_written,
                partition_size: self.expected_len,
            });
        }
        // Only the final chunk may have a length not divisible by 4.
        if (chunk.len() & 3) != 0 && next_written != self.expected_len {
            return Err(OtaError::FlashWriteError);
        }

        if !self.header_validated {
            let need = ESP_IMAGE_HEADER_LEN - self.header_len;
            let take = chunk.len().min(need);
            self.header_buf[self.header_len..self.header_len + take]
                .copy_from_slice(&chunk[..take]);
            self.header_len += take;
            if self.header_len == ESP_IMAGE_HEADER_LEN {
                validate_ota_image_header(
                    &self.header_buf,
                    self.expected_len,
                    self.partition_size as usize,
                )?;
                self.header_validated = true;
            }
        }

        let mut buf = AlignedFlashChunk::default();
        let mut off = 0usize;
        while off < chunk.len() {
            let step = (chunk.len() - off).min(OTA_CHUNK_SIZE);
            buf.0[..step].copy_from_slice(&chunk[off..off + step]);
            let aligned_step = (step + 3) & !3;
            let mut pad = step;
            while pad < aligned_step {
                buf.0[pad] = 0xFF;
                pad += 1;
            }
            let rel_off =
                u32::try_from(self.bytes_written).map_err(|_| OtaError::FlashWriteError)?;
            let write_end = rel_off
                .checked_add(aligned_step as u32)
                .ok_or(OtaError::FlashWriteError)?;
            if write_end > self.partition_size {
                return Err(OtaError::ImageTooLarge {
                    image_size: write_end as usize,
                    partition_size: self.partition_size as usize,
                });
            }
            let flash_addr = self
                .partition_offset
                .checked_add(rel_off)
                .ok_or(OtaError::FlashWriteError)?;
            flash.write_aligned(flash_addr, &buf.0[..aligned_step])?;
            self.hasher.update(&chunk[off..off + step]);
            self.bytes_written += step;
            off += step;
        }
        Ok(())
    }

    /// Finalizes the streaming OTA session: requires exact byte count,
    /// validates full image header/segments/XOR checksum/SHA-256 via flash
    /// readback (`FW-NET-18`), and commits `otadata` (`M-035`).
    pub fn finish<F: OtaFlashStorage>(
        self,
        mgr: &mut OtaDataManager,
        flash: &mut F,
    ) -> Result<OtaDataSelect, OtaError> {
        if self.bytes_written != self.expected_len {
            return Err(OtaError::IncompletePayload {
                received: self.bytes_written,
                expected: self.expected_len,
            });
        }
        if !self.header_validated {
            return Err(OtaError::ImageTooSmall);
        }
        if self.target_slot != mgr.select_inactive_slot()
            || mgr.pending_reboot_slot().is_some()
            || mgr.has_pending_rollback()
        {
            return Err(OtaError::RollbackStateInvalid);
        }
        let written_digest = self.hasher.finalize();
        if flash.supports_readback() {
            let mut chunk = AlignedFlashChunk::default();
            let readback = OtaDataManager::readback_verify_image(
                flash,
                self.partition_offset,
                self.expected_len,
                &mut chunk,
            )?;
            if readback != written_digest {
                return Err(OtaError::ReadbackMismatch);
            }
        }
        mgr.commit_update(flash, self.target_slot)
    }
}

/// Dual-sector `otadata` state machine (`0x17000` and `0x18000`).
///
/// Tracks immutable hardware `running_slot` and a `pending_reboot_slot` latch
/// (`M-004` / `C1-004`) so next-boot `otadata` commits never flip the target
/// OTA partition onto the live XIP-executing slot before reboot.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OtaDataManager {
    pub sector_0: OtaDataSelect,
    pub sector_1: OtaDataSelect,
    running_slot: OtaSlot,
    pending_reboot_slot: Option<OtaSlot>,
}

impl OtaDataManager {
    /// Computes the winning bootable [`OtaSlot`] from two raw sectors.
    #[inline]
    const fn compute_active_slot(sector_0: &OtaDataSelect, sector_1: &OtaDataSelect) -> OtaSlot {
        let s0_ok = sector_0.is_bootable();
        let s1_ok = sector_1.is_bootable();
        let winning_seq = match (s0_ok, s1_ok) {
            (true, true) => {
                if is_seq_greater(sector_0.ota_seq, sector_1.ota_seq) {
                    sector_0.ota_seq
                } else {
                    sector_1.ota_seq
                }
            }
            (true, false) => sector_0.ota_seq,
            (false, true) => sector_1.ota_seq,
            (false, false) => return OtaSlot::Ota0,
        };
        match OtaSlot::from_seq(winning_seq) {
            Some(slot) => slot,
            None => OtaSlot::Ota0,
        }
    }

    /// Creates an [`OtaDataManager`] booted into `ota_0` (`ota_seq = 1`, `Valid`).
    #[inline]
    pub const fn new() -> Self {
        Self::new_booted_ota0()
    }

    /// Creates an [`OtaDataManager`] booted into `ota_0` (`ota_seq = 1`, `Valid`).
    #[inline]
    pub const fn new_booted_ota0() -> Self {
        Self {
            sector_0: OtaDataSelect::new(1, OtaImgState::Valid),
            sector_1: OtaDataSelect::erased(),
            running_slot: OtaSlot::Ota0,
            pending_reboot_slot: None,
        }
    }

    /// Creates an [`OtaDataManager`] from two raw `otadata` sector entries,
    /// latching `running_slot` from the winning bootable sector.
    #[inline]
    pub const fn from_sectors(sector_0: OtaDataSelect, sector_1: OtaDataSelect) -> Self {
        let running_slot = Self::compute_active_slot(&sector_0, &sector_1);
        Self {
            sector_0,
            sector_1,
            running_slot,
            pending_reboot_slot: None,
        }
    }

    /// Returns the immutable hardware partition slot currently executing code
    /// (`M-004` / `C1-004`).
    #[inline]
    pub const fn running_slot(&self) -> OtaSlot {
        self.running_slot
    }

    /// Returns the newly committed target slot awaiting a device reboot, if any
    /// (`M-004` / `C1-004`).
    #[inline]
    pub const fn pending_reboot_slot(&self) -> Option<OtaSlot> {
        self.pending_reboot_slot
    }

    /// Records the currently executing hardware MMU slot without fabricating
    /// or overwriting `otadata` sector state (`M-039` / `C1-019`).
    #[inline]
    pub fn set_running_slot_from_mmu(&mut self, slot: OtaSlot) {
        self.running_slot = slot;
    }

    /// Reads both `otadata` sectors (`0x17000` and `0x18000`) from persistent
    /// `flash` using 4-byte-aligned [`AlignedOtaBuf`] buffers.
    ///
    /// Rejects non-persistent backends (`!flash.has_persistent_otadata()`,
    /// `M-137`). If neither sector in flash contains a valid CRC-32 (e.g.
    /// blank factory flash), initializes to [`Self::new_booted_ota0`].
    pub fn load_from_flash<F: OtaFlashStorage>(flash: &mut F) -> Result<Self, OtaError> {
        if !flash.has_persistent_otadata() {
            return Err(OtaError::FlashWriteError);
        }
        let mut buf0 = AlignedOtaBuf::default();
        let mut buf1 = AlignedOtaBuf::default();
        flash.read_aligned(OTADATA_SECTOR_0_OFFSET, &mut buf0.0)?;
        flash.read_aligned(OTADATA_SECTOR_1_OFFSET, &mut buf1.0)?;
        let sector_0 = OtaDataSelect::from_bytes(&buf0.0);
        let sector_1 = OtaDataSelect::from_bytes(&buf1.0);
        if !sector_0.is_crc_valid() && !sector_1.is_crc_valid() {
            Ok(Self::new_booted_ota0())
        } else {
            Ok(Self::from_sectors(sector_0, sector_1))
        }
    }

    /// Returns the typed inactive/older `otadata` flash sector, preferring
    /// non-bootable (erased, aborted, invalid, or corrupt) sectors first so
    /// post-rollback OTAs never overwrite the sole valid fallback sector
    /// (`M-003` / `C1-003`).
    #[inline]
    pub const fn select_inactive_sector(&self) -> OtaDataSector {
        let s0_boot = self.sector_0.is_bootable();
        let s1_boot = self.sector_1.is_bootable();
        match (s0_boot, s1_boot) {
            (true, true) => {
                if is_seq_greater(self.sector_0.ota_seq, self.sector_1.ota_seq) {
                    OtaDataSector::Sector1
                } else {
                    OtaDataSector::Sector0
                }
            }
            (true, false) => OtaDataSector::Sector1,
            (false, true) => OtaDataSector::Sector0,
            (false, false) => OtaDataSector::Sector0,
        }
    }

    /// Returns the index of the inactive/older `otadata` flash sector (`0` for
    /// `0x17000`, `1` for `0x18000`) (`M-003` / `C1-003`).
    #[inline]
    pub const fn select_inactive_otadata_sector(&self) -> usize {
        self.select_inactive_sector().index()
    }

    /// Returns the index of the active `otadata` flash sector (`0` for
    /// `0x17000`, `1` for `0x18000`).
    #[inline]
    pub const fn active_otadata_sector(&self) -> usize {
        self.select_inactive_sector().opposite().index()
    }

    /// Programs `entry` into `sector` in two stages (`0..28` before the final
    /// 4-byte `crc` commit word at `28..32`, `M-136` / `C1-038`) and performs
    /// 32-byte readback verification when `flash.supports_readback()` is true
    /// (`M-137` / `C1-039`).
    fn write_entry_to_sector_with_verify<F: OtaFlashStorage>(
        flash: &mut F,
        sector: OtaDataSector,
        entry: &OtaDataSelect,
    ) -> Result<(), OtaError> {
        let sector_offset = sector.offset();
        let expected = AlignedOtaBuf(entry.to_bytes());
        flash.erase_range(sector_offset, sector_offset + FLASH_SECTOR_SIZE)?;
        // Stage 1: write ota_seq, seq_label, and ota_state before CRC so a
        // power loss leaves crc == 0xFFFF_FFFF (detectable torn write).
        flash.write_aligned(sector_offset, &expected.0[..28])?;
        // Stage 2: commit the 4-byte CRC word last.
        flash.write_aligned(sector_offset + 28, &expected.0[28..32])?;
        if flash.supports_readback() {
            let mut verify = AlignedOtaBuf::default();
            flash.read_aligned(sector_offset, &mut verify.0)?;
            if verify.0 != expected.0 {
                return Err(OtaError::ReadbackMismatch);
            }
        }
        Ok(())
    }

    /// Erases the 4KB `otadata` `sector` and writes its 32-byte
    /// [`OtaDataSelect`] entry to `flash` with two-stage commit and readback
    /// verification (`M-136` / `M-137`).
    pub fn sync_sector_entry_to_flash<F: OtaFlashStorage>(
        &self,
        flash: &mut F,
        sector: OtaDataSector,
    ) -> Result<(), OtaError> {
        let entry = match sector {
            OtaDataSector::Sector0 => &self.sector_0,
            OtaDataSector::Sector1 => &self.sector_1,
        };
        Self::write_entry_to_sector_with_verify(flash, sector, entry)
    }

    /// Erases the 4KB `otadata` sector (`sector_index`: `0` for `0x17000`,
    /// `1` for `0x18000`) and writes its 32-byte [`OtaDataSelect`] entry
    /// to `flash`.
    pub fn sync_otadata_sector_to_flash<F: OtaFlashStorage>(
        &self,
        flash: &mut F,
        sector_index: usize,
    ) -> Result<(), OtaError> {
        let sector = OtaDataSector::from_index(sector_index).ok_or(OtaError::InvalidOtaDataCrc)?;
        self.sync_sector_entry_to_flash(flash, sector)
    }

    /// Syncs the `otadata` sector corresponding to `slot` to `flash`
    /// (`M-137` / `C1-039`: routes to the sector actually holding `slot`
    /// rather than assuming `Ota0 => sector_0` / `Ota1 => sector_1`).
    pub fn sync_slot_to_flash<F: OtaFlashStorage>(
        &self,
        flash: &mut F,
        slot: OtaSlot,
    ) -> Result<(), OtaError> {
        let s0_match = self.sector_0.is_bootable() && self.sector_0.slot() == Some(slot);
        let s1_match = self.sector_1.is_bootable() && self.sector_1.slot() == Some(slot);
        let sector = match (s0_match, s1_match) {
            (true, true) => {
                if is_seq_greater(self.sector_0.ota_seq, self.sector_1.ota_seq) {
                    OtaDataSector::Sector0
                } else {
                    OtaDataSector::Sector1
                }
            }
            (true, false) => OtaDataSector::Sector0,
            (false, true) => OtaDataSector::Sector1,
            (false, false) => {
                if self.sector_0.is_crc_valid() && self.sector_0.slot() == Some(slot) {
                    OtaDataSector::Sector0
                } else if self.sector_1.is_crc_valid() && self.sector_1.slot() == Some(slot) {
                    OtaDataSector::Sector1
                } else {
                    match slot {
                        OtaSlot::Ota0 => OtaDataSector::Sector0,
                        OtaSlot::Ota1 => OtaDataSector::Sector1,
                    }
                }
            }
        };
        self.sync_sector_entry_to_flash(flash, sector)
    }

    /// Alias for [`Self::sync_slot_to_flash`].
    #[inline]
    pub fn sync_sector_to_flash<F: OtaFlashStorage>(
        &self,
        flash: &mut F,
        slot: OtaSlot,
    ) -> Result<(), OtaError> {
        self.sync_slot_to_flash(flash, slot)
    }

    /// Marks the currently running OTA slot as [`OtaImgState::Valid`]
    /// (`EMB-003`) and, if the state transitioned from `PendingVerify`/`New`/
    /// `Undefined`, persists the updated `otadata` sector to `flash`.
    ///
    /// Hardened against:
    /// - Premature validation of an unbooted committed slot (`M-004`)
    /// - RAM/flash desync on write failure (`M-036`)
    /// - Zero-bootable-sector power-loss bricking during active-sector erase
    ///   (`M-037` / `C1-017`)
    pub fn cancel_rollback_in_flash<F: OtaFlashStorage>(
        &mut self,
        flash: &mut F,
    ) -> Result<bool, OtaError> {
        if self.pending_reboot_slot.is_some() {
            return Err(OtaError::RollbackStateInvalid);
        }
        let mut staged = *self;
        let s0_ok = staged.sector_0.is_bootable();
        let s1_ok = staged.sector_1.is_bootable();
        let (active_sec, other_bootable) = match (s0_ok, s1_ok) {
            (true, true) => {
                if is_seq_greater(staged.sector_0.ota_seq, staged.sector_1.ota_seq) {
                    (OtaDataSector::Sector0, true)
                } else {
                    (OtaDataSector::Sector1, true)
                }
            }
            (true, false) => (OtaDataSector::Sector0, false),
            (false, true) => (OtaDataSector::Sector1, false),
            (false, false) => return Err(OtaError::InvalidOtaDataCrc),
        };
        let target_entry = match active_sec {
            OtaDataSector::Sector0 => &mut staged.sector_0,
            OtaDataSector::Sector1 => &mut staged.sector_1,
        };
        if target_entry.slot() != Some(self.running_slot) {
            return Err(OtaError::RollbackStateInvalid);
        }
        let orig_entry = *target_entry;
        let prev_state = orig_entry.state();
        let changed = target_entry.mark_app_valid_cancel_rollback()?;
        if changed {
            let updated_entry = *target_entry;
            if flash.has_persistent_otadata() {
                if prev_state == OtaImgState::Undefined {
                    // NOR 0xFFFF_FFFF -> 0x0000_0002 is strictly 1->0 bits;
                    // program in place with zero erase window.
                    let state_bytes = (OtaImgState::Valid as u32).to_le_bytes();
                    flash.write_aligned(active_sec.offset() + 24, &state_bytes)?;
                    if flash.supports_readback() {
                        let mut verify = AlignedOtaBuf::default();
                        flash.read_aligned(active_sec.offset(), &mut verify.0)?;
                        if verify.0 != updated_entry.to_bytes() {
                            return Err(OtaError::ReadbackMismatch);
                        }
                    }
                } else if !other_bootable {
                    let backup_sec = active_sec.opposite();
                    if updated_entry.ota_seq <= u32::MAX - 3 {
                        // M-037: Atomically write `(ota_seq + 2, Valid)` (same
                        // slot parity, strictly higher seq) into `backup_sec`
                        // without erasing the sole bootable `active_sec`.
                        let next_seq = updated_entry.ota_seq + 2;
                        let advanced = OtaDataSelect::try_new(next_seq, OtaImgState::Valid)
                            .ok_or(OtaError::SequenceOverflow)?;
                        Self::write_entry_to_sector_with_verify(flash, backup_sec, &advanced)?;
                        match active_sec {
                            OtaDataSector::Sector0 => {
                                staged.sector_0 = orig_entry;
                                staged.sector_1 = advanced;
                            }
                            OtaDataSector::Sector1 => {
                                staged.sector_1 = orig_entry;
                                staged.sector_0 = advanced;
                            }
                        }
                    } else {
                        // Sequence near u32::MAX: stage backup copy before
                        // rewriting active sector; equal `(ota_seq, Valid)`
                        // recovery is permitted in `resync_from_flash_for_ota`.
                        Self::write_entry_to_sector_with_verify(flash, backup_sec, &updated_entry)?;
                        Self::write_entry_to_sector_with_verify(flash, active_sec, &updated_entry)?;
                        flash.erase_range(
                            backup_sec.offset(),
                            backup_sec.offset() + FLASH_SECTOR_SIZE,
                        )?;
                    }
                } else {
                    // Opposite sector is a valid bootable fallback; safe to
                    // erase and two-stage rewrite the active sector.
                    Self::write_entry_to_sector_with_verify(flash, active_sec, &updated_entry)?;
                }
            }
            *self = staged;
        }
        Ok(changed)
    }

    /// Prepares the inactive partition for receiving OTA chunks by resyncing
    /// `otadata` from flash, rejecting unconfirmed/pending-reboot states
    /// (`M-004` / `M-005`), validating `image_size`, and erasing the 4KB-aligned
    /// target range (`M-035`).
    pub fn begin_ota<F: OtaFlashStorage>(
        &mut self,
        flash: &mut F,
        image_size: usize,
    ) -> Result<OtaSlot, OtaError> {
        if self.pending_reboot_slot.is_some() {
            return Err(OtaError::RollbackStateInvalid);
        }
        self.resync_from_flash_for_ota(flash)?;
        if self.has_pending_rollback() {
            return Err(OtaError::RollbackStateInvalid);
        }
        let target_slot = self.select_inactive_slot();
        let partition = target_slot.partition();
        let max_size = if (partition.size as usize) < OTA_PARTITION_MAX_SIZE {
            partition.size as usize
        } else {
            OTA_PARTITION_MAX_SIZE
        };
        if image_size == 0 {
            return Err(OtaError::MissingContentLength);
        }
        if image_size < ESP_IMAGE_HEADER_LEN {
            return Err(OtaError::ImageTooSmall);
        }
        if image_size > max_size {
            return Err(OtaError::ImageTooLarge {
                image_size,
                partition_size: max_size,
            });
        }
        let image_u32 = u32::try_from(image_size).map_err(|_| OtaError::ImageTooLarge {
            image_size,
            partition_size: max_size,
        })?;
        let erase_len = image_u32
            .checked_add(0xFFF)
            .ok_or(OtaError::ImageTooLarge {
                image_size,
                partition_size: max_size,
            })?
            & !0xFFF;
        if erase_len > partition.size {
            return Err(OtaError::ImageTooLarge {
                image_size,
                partition_size: max_size,
            });
        }
        let erase_end = partition
            .offset
            .checked_add(erase_len)
            .ok_or(OtaError::FlashWriteError)?;
        flash.erase_range(partition.offset, erase_end)?;
        Ok(target_slot)
    }

    /// Starts a stateful [`OtaWriteSession`] that tracks byte count, header
    /// validation, running SHA-256, and flash readback before commit (`M-035`).
    pub fn begin_ota_session<F: OtaFlashStorage>(
        &mut self,
        flash: &mut F,
        image_size: usize,
    ) -> Result<OtaWriteSession, OtaError> {
        let target_slot = self.begin_ota(flash, image_size)?;
        let partition = target_slot.partition();
        Ok(OtaWriteSession {
            target_slot,
            expected_len: image_size,
            bytes_written: 0,
            partition_offset: partition.offset,
            partition_size: partition.size,
            header_buf: [0u8; ESP_IMAGE_HEADER_LEN],
            header_len: 0,
            header_validated: false,
            hasher: Sha256::new(),
        })
    }

    /// Writes a 4-byte-aligned chunk of OTA firmware into the inactive flash
    /// partition after checking slot safety, alignment, and partition bounds
    /// (`M-035` / `C1-015`).
    pub fn write_chunk<F: OtaFlashStorage>(
        &self,
        flash: &mut F,
        slot: OtaSlot,
        offset: u32,
        chunk: &[u8],
    ) -> Result<(), OtaError> {
        if slot != self.select_inactive_slot()
            || self.pending_reboot_slot.is_some()
            || self.has_pending_rollback()
        {
            return Err(OtaError::RollbackStateInvalid);
        }
        if (offset & 3) != 0 || (chunk.len() & 3) != 0 {
            return Err(OtaError::FlashWriteError);
        }
        let partition = slot.partition();
        let chunk_len = u32::try_from(chunk.len()).map_err(|_| OtaError::ImageTooLarge {
            image_size: chunk.len(),
            partition_size: partition.size as usize,
        })?;
        let end_rel = offset
            .checked_add(chunk_len)
            .ok_or(OtaError::ImageTooLarge {
                image_size: usize::MAX,
                partition_size: partition.size as usize,
            })?;
        if end_rel > partition.size {
            return Err(OtaError::ImageTooLarge {
                image_size: end_rel as usize,
                partition_size: partition.size as usize,
            });
        }
        let flash_addr = partition
            .offset
            .checked_add(offset)
            .ok_or(OtaError::FlashWriteError)?;
        flash.write_aligned(flash_addr, chunk)
    }

    /// Finalizes an OTA update after all chunks are written, walking the
    /// flashed ESP32 segment headers and executing full
    /// [`Self::readback_verify_image`] (header, segments, 8-bit XOR checksum,
    /// and mandatory appended SHA-256) before committing `otadata` (`M-035`).
    pub fn finish_ota<F: OtaFlashStorage>(
        &mut self,
        flash: &mut F,
        slot: OtaSlot,
    ) -> Result<OtaDataSelect, OtaError> {
        if slot != self.select_inactive_slot()
            || self.pending_reboot_slot.is_some()
            || self.has_pending_rollback()
        {
            return Err(OtaError::RollbackStateInvalid);
        }
        if !flash.supports_readback() {
            return Err(OtaError::FlashWriteError);
        }
        let partition = slot.partition();
        let max_size = if (partition.size as usize) < OTA_PARTITION_MAX_SIZE {
            partition.size as usize
        } else {
            OTA_PARTITION_MAX_SIZE
        };
        let base = partition.offset;

        let mut hdr_buf = AlignedOtaBuf::default();
        flash.read_aligned(base, &mut hdr_buf.0)?;
        let header = validate_ota_image_header(
            &hdr_buf.0[..ESP_IMAGE_HEADER_LEN],
            ESP_IMAGE_HEADER_LEN,
            max_size,
        )?;

        let mut chunk = AlignedFlashChunk::default();
        let mut offset = ESP_IMAGE_HEADER_LEN;
        let mut seg_idx = 0u8;
        let mut seg_hdr = [0u8; 8];
        while seg_idx < header.segment_count {
            if offset.checked_add(8).ok_or(OtaError::ImageTooSmall)? > max_size {
                return Err(OtaError::ImageTooSmall);
            }
            Self::read_exact_unaligned(flash, base, offset, &mut seg_hdr, &mut chunk)?;
            let data_len =
                u32::from_le_bytes([seg_hdr[4], seg_hdr[5], seg_hdr[6], seg_hdr[7]]) as usize;
            offset = offset
                .checked_add(8)
                .and_then(|o| o.checked_add(data_len))
                .ok_or(OtaError::ImageTooSmall)?;
            if offset > max_size {
                return Err(OtaError::ImageTooLarge {
                    image_size: offset,
                    partition_size: max_size,
                });
            }
            seg_idx += 1;
        }

        let aligned_end = (offset.checked_add(16).ok_or(OtaError::ImageTooSmall)?) & !15;
        let total_len = aligned_end
            .checked_add(ESP_IMAGE_DIGEST_LEN)
            .ok_or(OtaError::ImageTooSmall)?;
        if total_len > max_size {
            return Err(OtaError::ImageTooLarge {
                image_size: total_len,
                partition_size: max_size,
            });
        }
        Self::readback_verify_image(flash, base, total_len, &mut chunk)?;
        self.commit_update(flash, slot)
    }

    /// Commits an update to the older/inactive `otadata` sector in flash,
    /// deferring RAM mutation until flash writes + readback succeed (`M-036`)
    /// and persisting factory-blank `sector_0` first if needed (`M-037`).
    pub fn commit_update<F: OtaFlashStorage>(
        &mut self,
        flash: &mut F,
        target_slot: OtaSlot,
    ) -> Result<OtaDataSelect, OtaError> {
        if self.pending_reboot_slot.is_some()
            || self.has_pending_rollback()
            || target_slot == self.running_slot
        {
            return Err(OtaError::RollbackStateInvalid);
        }
        let mut staged = *self;
        let inactive_sector = staged.select_inactive_sector();

        // M-037 / C1-017: On factory-blank flash where `sector_0 = (1, Valid)`
        // was synthesized only in RAM, persist `sector_0` before writing the
        // first `Ota1` update into `sector_1` so rollback always has a valid
        // on-flash fallback record.
        if flash.has_persistent_otadata()
            && inactive_sector == OtaDataSector::Sector1
            && staged.sector_0.is_bootable()
        {
            let mut raw0 = AlignedOtaBuf::default();
            let mut raw1 = AlignedOtaBuf::default();
            flash.read_aligned(OTADATA_SECTOR_0_OFFSET, &mut raw0.0)?;
            flash.read_aligned(OTADATA_SECTOR_1_OFFSET, &mut raw1.0)?;
            if raw0.0 == [0xFF; 32] && raw1.0 == [0xFF; 32] {
                staged.sync_sector_entry_to_flash(flash, OtaDataSector::Sector0)?;
            }
        }

        let entry = staged.set_boot_slot(target_slot)?;
        staged.sync_sector_entry_to_flash(flash, inactive_sector)?;
        *self = staged;
        Ok(entry)
    }

    /// Rolls back an unconfirmed (`PendingVerify` / `New` / `Undefined`)
    /// running slot to the previous bootable partition, deferring RAM mutation
    /// until flash write + readback succeed (`M-036`).
    pub fn rollback<F: OtaFlashStorage>(&mut self, flash: &mut F) -> Result<OtaSlot, OtaError> {
        if !self.has_pending_rollback() || self.pending_reboot_slot.is_some() {
            return Err(OtaError::RollbackStateInvalid);
        }
        let mut staged = *self;
        let s0_ok = staged.sector_0.is_bootable();
        let s1_ok = staged.sector_1.is_bootable();
        match (s0_ok, s1_ok) {
            (true, true) => {
                let active_sec = if is_seq_greater(staged.sector_0.ota_seq, staged.sector_1.ota_seq)
                {
                    staged.sector_0.ota_state = OtaImgState::Aborted as u32;
                    staged.sector_0.crc = staged.sector_0.calculate_crc();
                    OtaDataSector::Sector0
                } else {
                    staged.sector_1.ota_state = OtaImgState::Aborted as u32;
                    staged.sector_1.crc = staged.sector_1.calculate_crc();
                    OtaDataSector::Sector1
                };
                staged.sync_sector_entry_to_flash(flash, active_sec)?;
                *self = staged;
                Ok(self.active_slot())
            }
            _ => Err(OtaError::RollbackStateInvalid),
        }
    }

    /// Resynchronizes this manager's memory state against real flash before a
    /// rollback cancellation check (`M-039` / `C1-019`).
    pub fn resync_from_flash_for_rollback<F: OtaFlashStorage>(
        &mut self,
        flash: &mut F,
    ) -> Result<(), OtaError> {
        if !flash.has_persistent_otadata() {
            return Err(OtaError::FlashWriteError);
        }
        if self.pending_reboot_slot.is_some() {
            return Err(OtaError::RollbackStateInvalid);
        }
        let mut buf0 = AlignedOtaBuf::default();
        let mut buf1 = AlignedOtaBuf::default();
        flash.read_aligned(OTADATA_SECTOR_0_OFFSET, &mut buf0.0)?;
        flash.read_aligned(OTADATA_SECTOR_1_OFFSET, &mut buf1.0)?;

        let s0_erased = buf0.0 == [0xFF; 32];
        let s1_erased = buf1.0 == [0xFF; 32];
        if s0_erased && s1_erased {
            if self.running_slot != OtaSlot::Ota0 {
                return Err(OtaError::RollbackStateInvalid);
            }
            self.sector_0 = OtaDataSelect::new(1, OtaImgState::Valid);
            self.sector_1 = OtaDataSelect::erased();
            return Ok(());
        }

        let s0 = OtaDataSelect::from_bytes(&buf0.0);
        let s1 = OtaDataSelect::from_bytes(&buf1.0);
        if !s0.is_bootable() && !s1.is_bootable() {
            return Err(OtaError::InvalidOtaDataCrc);
        }
        let flash_active = Self::compute_active_slot(&s0, &s1);
        if flash_active != self.running_slot {
            return Err(OtaError::RollbackStateInvalid);
        }
        self.sector_0 = s0;
        self.sector_1 = s1;
        Ok(())
    }

    /// Resynchronizes this manager's memory state against real flash before an
    /// OTA partition write begins (`FW-NET-12`, `M-004`, `M-005`, `M-037`,
    /// `M-038`).
    ///
    /// Allows recovery when:
    /// - one sector is valid, bootable, confirmed (`!has_pending_rollback()`),
    ///   and matches `running_slot`, even if the inactive sector is erased,
    ///   torn, or has a corrupt CRC (`M-038`);
    /// - both sectors hold identical `(ota_seq, Valid)` matching
    ///   `running_slot` after an interrupted backup-copy cancel (`M-037`).
    ///
    /// Rejects states where neither sector is bootable (`InvalidOtaDataCrc` /
    /// `RollbackStateInvalid`), unconfirmed `PendingVerify` running slots
    /// (`M-005`), and already-committed pending-reboot states (`M-004`).
    pub fn resync_from_flash_for_ota<F: OtaFlashStorage>(
        &mut self,
        flash: &mut F,
    ) -> Result<(), OtaError> {
        if self.pending_reboot_slot.is_some() {
            return Err(OtaError::RollbackStateInvalid);
        }
        if !flash.has_persistent_otadata() {
            if self.has_pending_rollback() {
                return Err(OtaError::RollbackStateInvalid);
            }
            return Ok(());
        }
        let mut buf0 = AlignedOtaBuf::default();
        let mut buf1 = AlignedOtaBuf::default();
        flash.read_aligned(OTADATA_SECTOR_0_OFFSET, &mut buf0.0)?;
        flash.read_aligned(OTADATA_SECTOR_1_OFFSET, &mut buf1.0)?;

        let s0_erased = buf0.0 == [0xFF; 32];
        let s1_erased = buf1.0 == [0xFF; 32];
        if s0_erased && s1_erased {
            if self.running_slot != OtaSlot::Ota0 {
                return Err(OtaError::RollbackStateInvalid);
            }
            self.sector_0 = OtaDataSelect::new(1, OtaImgState::Valid);
            self.sector_1 = OtaDataSelect::erased();
            return Ok(());
        }

        let s0 = OtaDataSelect::from_bytes(&buf0.0);
        let s1 = OtaDataSelect::from_bytes(&buf1.0);

        if !s0.is_bootable() && !s1.is_bootable() {
            let s0_bad_crc = !s0_erased && !s0.is_crc_valid();
            let s1_bad_crc = !s1_erased && !s1.is_crc_valid();
            if s0_bad_crc || s1_bad_crc {
                return Err(OtaError::InvalidOtaDataCrc);
            }
            return Err(OtaError::RollbackStateInvalid);
        }

        let flash_mgr = Self::from_sectors(s0, s1);
        let equal_seq = s0.is_crc_valid() && s1.is_crc_valid() && s0.ota_seq == s1.ota_seq;
        let equal_valid_running = equal_seq
            && s0.state() == OtaImgState::Valid
            && s1.state() == OtaImgState::Valid
            && s0.slot() == Some(self.running_slot);
        if (equal_seq && !equal_valid_running)
            || flash_mgr.active_slot() != self.running_slot
            || flash_mgr.has_pending_rollback()
        {
            return Err(OtaError::RollbackStateInvalid);
        }
        self.sector_0 = s0;
        self.sector_1 = s1;
        Ok(())
    }

    /// Validates an ESP32 OTA firmware `payload`, resyncs against `flash`
    /// (`FW-NET-12`), enforces confirmed running slot and single-commit latch
    /// (`M-004` / `M-005`), erases the 4KB-rounded range in the passive
    /// [`OtaSlot`] partition, writes the image in 1024-byte 4-byte-aligned
    /// chunks (padding any trailing `1..=3` bytes with `0xFF`), verifies
    /// readback SHA-256 (`FW-NET-18`), and commits `otadata`.
    pub fn write_ota_image_to_flash<F: OtaFlashStorage>(
        &mut self,
        flash: &mut F,
        payload: &[u8],
    ) -> Result<OtaSlot, OtaError> {
        if self.pending_reboot_slot.is_some() {
            return Err(OtaError::RollbackStateInvalid);
        }
        validate_ota_image_header(payload, payload.len(), OTA_PARTITION_MAX_SIZE)?;
        validate_ota_image_integrity(payload)?;

        self.resync_from_flash_for_ota(flash)?;
        if self.has_pending_rollback() {
            return Err(OtaError::RollbackStateInvalid);
        }

        let target_slot = self.select_inactive_slot();
        let partition = target_slot.partition();
        let max_size = if (partition.size as usize) < OTA_PARTITION_MAX_SIZE {
            partition.size as usize
        } else {
            OTA_PARTITION_MAX_SIZE
        };
        if payload.len() > max_size {
            return Err(OtaError::ImageTooLarge {
                image_size: payload.len(),
                partition_size: max_size,
            });
        }

        let payload_u32 = u32::try_from(payload.len()).map_err(|_| OtaError::ImageTooLarge {
            image_size: payload.len(),
            partition_size: max_size,
        })?;
        let erase_len = payload_u32
            .checked_add(0xFFF)
            .ok_or(OtaError::ImageTooLarge {
                image_size: payload.len(),
                partition_size: max_size,
            })?
            & !0xFFF;
        if erase_len > partition.size {
            return Err(OtaError::ImageTooLarge {
                image_size: payload.len(),
                partition_size: max_size,
            });
        }
        let erase_end = partition
            .offset
            .checked_add(erase_len)
            .ok_or(OtaError::FlashWriteError)?;
        flash.erase_range(partition.offset, erase_end)?;

        let mut written_hash = Sha256::new();
        let mut chunk = AlignedFlashChunk::default();
        let mut received = 0usize;
        while received < payload.len() {
            let remaining = payload.len() - received;
            let step = if remaining > OTA_CHUNK_SIZE {
                OTA_CHUNK_SIZE
            } else {
                remaining
            };
            chunk.0[..step].copy_from_slice(&payload[received..received + step]);
            let aligned_step = (step + 3) & !3;
            let mut pad = step;
            while pad < aligned_step {
                chunk.0[pad] = 0xFF;
                pad += 1;
            }
            written_hash.update(&chunk.0[..step]);
            let write_addr = partition
                .offset
                .checked_add(received as u32)
                .ok_or(OtaError::FlashWriteError)?;
            flash.write_aligned(write_addr, &chunk.0[..aligned_step])?;
            received += step;
        }

        if flash.supports_readback() {
            let expected = written_hash.finalize();
            let actual = Self::readback_digest(flash, partition.offset, payload.len(), &mut chunk)?;
            if actual != expected {
                return Err(OtaError::ReadbackMismatch);
            }
        }

        self.commit_update(flash, target_slot)?;
        Ok(target_slot)
    }

    /// Reads `len` bytes back from `base` in [`OTA_CHUNK_SIZE`] 4-byte-aligned
    /// chunks and returns their SHA-256 (`FW-NET-18` read-back before
    /// `commit_update`).
    fn readback_digest<F: OtaFlashStorage>(
        flash: &mut F,
        base: u32,
        len: usize,
        chunk: &mut AlignedFlashChunk,
    ) -> Result<[u8; 32], OtaError> {
        let mut hasher = Sha256::new();
        let mut off = 0usize;
        while off < len {
            let step = (len - off).min(OTA_CHUNK_SIZE);
            let aligned_step = (step + 3) & !3;
            let addr = base
                .checked_add(off as u32)
                .ok_or(OtaError::FlashWriteError)?;
            flash.read_aligned(addr, &mut chunk.0[..aligned_step])?;
            hasher.update(&chunk.0[..step]);
            off += step;
        }
        Ok(hasher.finalize())
    }

    /// Reads back a flashed image from `base` (`len` bytes), verifies its
    /// header, segment structure, 8-bit XOR checksum, exact length, and
    /// mandatory appended SHA-256 digest (`M-028` / `M-035`), and returns the
    /// full-image SHA-256 digest for comparison against the streamed hash.
    fn readback_verify_image<F: OtaFlashStorage>(
        flash: &mut F,
        base: u32,
        len: usize,
        chunk: &mut AlignedFlashChunk,
    ) -> Result<[u8; 32], OtaError> {
        if len < ESP_IMAGE_HEADER_LEN + ESP_IMAGE_DIGEST_LEN {
            return Err(OtaError::ImageTooSmall);
        }
        let mut hdr_buf = AlignedOtaBuf::default();
        flash.read_aligned(base, &mut hdr_buf.0)?;
        let header = validate_ota_image_header(
            &hdr_buf.0[..ESP_IMAGE_HEADER_LEN],
            len,
            OTA_PARTITION_MAX_SIZE,
        )?;
        if hdr_buf.0[23] != 1 {
            return Err(OtaError::InvalidDigestPlacement);
        }

        // Walk segments from flash to verify XOR checksum & exact aligned_end.
        let mut offset = ESP_IMAGE_HEADER_LEN;
        let mut chk = 0xEFu8;
        let mut seg_idx = 0u8;
        let mut seg_hdr = [0u8; 8];
        while seg_idx < header.segment_count {
            if offset.checked_add(8).ok_or(OtaError::ImageTooSmall)? > len {
                return Err(OtaError::ImageTooSmall);
            }
            Self::read_exact_unaligned(flash, base, offset, &mut seg_hdr, chunk)?;
            let load_addr = u32::from_le_bytes([seg_hdr[0], seg_hdr[1], seg_hdr[2], seg_hdr[3]]);
            let data_len =
                u32::from_le_bytes([seg_hdr[4], seg_hdr[5], seg_hdr[6], seg_hdr[7]]) as usize;
            if data_len > 0 && (load_addr & 3) != 0 {
                return Err(OtaError::InvalidEntryAddr(load_addr));
            }
            offset += 8;
            let end_seg = offset
                .checked_add(data_len)
                .ok_or(OtaError::ImageTooSmall)?;
            if end_seg > len {
                return Err(OtaError::ImageTooSmall);
            }
            let mut pos = offset;
            while pos < end_seg {
                let take = (end_seg - pos).min(OTA_CHUNK_SIZE);
                let mut tmp = AlignedFlashChunk::default();
                Self::read_exact_unaligned(flash, base, pos, &mut tmp.0[..take], chunk)?;
                let mut b = 0;
                while b < take {
                    chk ^= tmp.0[b];
                    b += 1;
                }
                pos += take;
            }
            offset = end_seg;
            seg_idx += 1;
        }

        let aligned_end = (offset.checked_add(16).ok_or(OtaError::ImageTooSmall)?) & !15;
        if aligned_end.checked_add(ESP_IMAGE_DIGEST_LEN) != Some(len) {
            return Err(OtaError::InvalidDigestPlacement);
        }
        let checksum_offset = aligned_end.checked_sub(1).ok_or(OtaError::ImageTooSmall)?;
        let mut chk_byte = [0u8; 1];
        Self::read_exact_unaligned(flash, base, checksum_offset, &mut chk_byte, chunk)?;
        if chk_byte[0] != chk {
            return Err(OtaError::InvalidChecksum);
        }

        let mut img_hasher = Sha256::new();
        let mut full_hasher = Sha256::new();
        let mut off = 0usize;
        while off < aligned_end {
            let step = (aligned_end - off).min(OTA_CHUNK_SIZE);
            let aligned_step = (step + 3) & !3;
            let addr = base
                .checked_add(off as u32)
                .ok_or(OtaError::FlashWriteError)?;
            flash.read_aligned(addr, &mut chunk.0[..aligned_step])?;
            img_hasher.update(&chunk.0[..step]);
            full_hasher.update(&chunk.0[..step]);
            off += step;
        }
        let mut expected_digest = [0u8; ESP_IMAGE_DIGEST_LEN];
        Self::read_exact_unaligned(flash, base, aligned_end, &mut expected_digest, chunk)?;
        if img_hasher.finalize() != expected_digest {
            return Err(OtaError::InvalidSha256);
        }
        full_hasher.update(&expected_digest);
        Ok(full_hasher.finalize())
    }

    /// Reads an arbitrary byte slice from flash using 4-byte-aligned reads.
    fn read_exact_unaligned<F: OtaFlashStorage>(
        flash: &mut F,
        base: u32,
        rel_offset: usize,
        out: &mut [u8],
        scratch: &mut AlignedFlashChunk,
    ) -> Result<(), OtaError> {
        let mut done = 0usize;
        while done < out.len() {
            let cur = rel_offset + done;
            let aligned_cur = cur & !3;
            let lead = cur - aligned_cur;
            let max_copy = (OTA_CHUNK_SIZE - lead).min(out.len() - done);
            let read_len = ((lead + max_copy) + 3) & !3;
            let addr = base
                .checked_add(aligned_cur as u32)
                .ok_or(OtaError::FlashWriteError)?;
            flash.read_aligned(addr, &mut scratch.0[..read_len])?;
            out[done..done + max_copy].copy_from_slice(&scratch.0[lead..lead + max_copy]);
            done += max_copy;
        }
        Ok(())
    }

    /// Returns `true` when at least one sector is bootable and the winning
    /// active sector's state is `PendingVerify`, `New`, or `Undefined`.
    #[inline]
    pub const fn has_pending_rollback(&self) -> bool {
        let s0_ok = self.sector_0.is_bootable();
        let s1_ok = self.sector_1.is_bootable();
        let active_state = match (s0_ok, s1_ok) {
            (true, true) => {
                if is_seq_greater(self.sector_0.ota_seq, self.sector_1.ota_seq) {
                    self.sector_0.state()
                } else {
                    self.sector_1.state()
                }
            }
            (true, false) => self.sector_0.state(),
            (false, true) => self.sector_1.state(),
            (false, false) => return false,
        };
        matches!(
            active_state,
            OtaImgState::PendingVerify | OtaImgState::New | OtaImgState::Undefined
        )
    }

    /// Returns the winning next-boot [`OtaSlot`] according to ESP-IDF
    /// bootloader selection rules.
    ///
    /// If neither sector contains a valid bootable record, defaults to
    /// [`OtaSlot::Ota0`] (factory fallback).
    pub const fn active_slot(&self) -> OtaSlot {
        Self::compute_active_slot(&self.sector_0, &self.sector_1)
    }

    /// Returns the inactive target slot derived strictly from the immutable
    /// `running_slot.opposite()` (`M-004` / `C1-004`), so committing an update
    /// never flips the target slot onto the live XIP-executing partition.
    #[inline]
    pub const fn select_inactive_slot(&self) -> OtaSlot {
        self.running_slot.opposite()
    }

    /// Returns the passive [`OtaSlot`] targeted by the next OTA update.
    #[inline]
    pub const fn next_update_slot(&self) -> OtaSlot {
        self.select_inactive_slot()
    }

    /// Updates `otadata` to boot into `target_slot` on the next restart, setting
    /// its state to [`OtaImgState::PendingVerify`] with a newly calculated
    /// ESP-IDF `otadata` CRC-32.
    ///
    /// Enforces:
    /// - Single-commit pending-reboot latch & non-XIP target (`M-004`)
    /// - Confirmed running slot (`M-005`)
    /// - Inactive sector selection via `is_bootable()` with `next_seq` strictly
    ///   above the maximum valid-CRC `ota_seq` across both sectors (`M-003`)
    /// - Fail-safe [`OtaError::SequenceOverflow`] near `u32::MAX` (`M-033`)
    pub fn set_boot_slot(&mut self, target_slot: OtaSlot) -> Result<OtaDataSelect, OtaError> {
        if self.pending_reboot_slot.is_some()
            || self.has_pending_rollback()
            || target_slot == self.running_slot
        {
            return Err(OtaError::RollbackStateInvalid);
        }
        let current_max = match (self.sector_0.is_crc_valid(), self.sector_1.is_crc_valid()) {
            (true, true) => {
                if is_seq_greater(self.sector_0.ota_seq, self.sector_1.ota_seq) {
                    self.sector_0.ota_seq
                } else {
                    self.sector_1.ota_seq
                }
            }
            (true, false) => self.sector_0.ota_seq,
            (false, true) => self.sector_1.ota_seq,
            (false, false) => 0,
        };

        let mut next_seq = current_max
            .checked_add(1)
            .ok_or(OtaError::SequenceOverflow)?;
        if next_seq == u32::MAX {
            return Err(OtaError::SequenceOverflow);
        }
        if OtaSlot::from_seq(next_seq) != Some(target_slot) {
            next_seq = next_seq.checked_add(1).ok_or(OtaError::SequenceOverflow)?;
            if next_seq == u32::MAX {
                return Err(OtaError::SequenceOverflow);
            }
        }

        let entry = OtaDataSelect::try_new(next_seq, OtaImgState::PendingVerify)
            .ok_or(OtaError::SequenceOverflow)?;
        let inactive_sector = self.select_inactive_sector();
        match inactive_sector {
            OtaDataSector::Sector0 => self.sector_0 = entry,
            OtaDataSector::Sector1 => self.sector_1 = entry,
        }
        self.pending_reboot_slot = Some(target_slot);
        Ok(entry)
    }

    /// Cancels rollback on the currently running OTA slot in RAM
    /// (`esp_ota_mark_app_valid_cancel_rollback`, `EMB-003` / `M-004`).
    pub fn mark_app_valid_cancel_rollback(&mut self) -> Result<bool, OtaError> {
        if self.pending_reboot_slot.is_some() {
            return Err(OtaError::RollbackStateInvalid);
        }
        let s0_ok = self.sector_0.is_bootable();
        let s1_ok = self.sector_1.is_bootable();
        let target = match (s0_ok, s1_ok) {
            (true, true) => {
                if is_seq_greater(self.sector_0.ota_seq, self.sector_1.ota_seq) {
                    &mut self.sector_0
                } else {
                    &mut self.sector_1
                }
            }
            (true, false) => &mut self.sector_0,
            (false, true) => &mut self.sector_1,
            (false, false) => return Err(OtaError::InvalidOtaDataCrc),
        };
        if target.slot() != Some(self.running_slot) {
            return Err(OtaError::RollbackStateInvalid);
        }
        target.mark_app_valid_cancel_rollback()
    }
}

impl Default for OtaDataManager {
    #[inline]
    fn default() -> Self {
        Self::new_booted_ota0()
    }
}

/// Parsed ESP32 firmware image header (`esp_image_header_t`, `OTA-002` /
/// `APP-008`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct EspImageHeader {
    /// Magic byte (`0xE9`).
    pub magic: u8,
    /// Number of memory segments (`1..=16`).
    pub segment_count: u8,
    /// SPI flash interface mode (`0..=5`).
    pub spi_mode: u8,
    /// Packed SPI speed (low nibble) and flash size (high nibble).
    pub spi_speed_size: u8,
    /// Entry point address (`u32`, little-endian).
    pub entry_addr: u32,
}

impl EspImageHeader {
    /// Constructs a canonical valid 24-byte ESP32 application header
    /// buffer for testing or staging.
    pub const fn minimal_valid_header_bytes(segment_count: u8, entry_addr: u32) -> [u8; 24] {
        let ep = entry_addr.to_le_bytes();
        let mut hdr = [0u8; 24];
        hdr[0] = ESP_IMAGE_HEADER_MAGIC;
        hdr[1] = segment_count;
        hdr[2] = 0x02; // DIO
        hdr[3] = 0x20; // 4MB, 40MHz
        hdr[4] = ep[0];
        hdr[5] = ep[1];
        hdr[6] = ep[2];
        hdr[7] = ep[3];
        hdr[23] = 0x01; // append_digest = 1
        hdr
    }

    /// Constructs a valid 96-byte single-segment ESP32 application image
    /// (64-byte aligned image + mandatory 32-byte appended SHA-256 digest,
    /// `M-028`).
    pub const fn minimal_valid_image_96() -> [u8; 96] {
        let mut base = [0u8; 64];
        base[0] = ESP_IMAGE_HEADER_MAGIC;
        base[1] = 1;
        base[2] = 0x02; // DIO
        base[3] = 0x20; // 4MB, 40MHz
        base[4] = 0x00;
        base[5] = 0x04;
        base[6] = 0x08;
        base[7] = 0x40; // Entry point 0x4008_0400
        base[23] = 0x01; // Mandatory appended SHA-256 digest (M-028)
                         // Segment 0 header
        base[24] = 0x00;
        base[25] = 0x04;
        base[26] = 0x08;
        base[27] = 0x40; // Load address 0x4008_0400
        base[28] = 16;
        base[29] = 0;
        base[30] = 0;
        base[31] = 0; // Length 16 bytes
        let mut chk = 0xEFu8;
        let mut i = 0usize;
        while i < 16 {
            let val = (i as u8) ^ 0x5A;
            base[32 + i] = val;
            chk ^= val;
            i += 1;
        }
        base[63] = chk;
        let digest = sha256_digest(&base);
        let mut img = [0u8; 96];
        let mut j = 0usize;
        while j < 64 {
            img[j] = base[j];
            j += 1;
        }
        let mut k = 0usize;
        while k < 32 {
            img[64 + k] = digest[k];
            k += 1;
        }
        img
    }

    /// Constructs a valid minimal single-segment ESP32 application image with
    /// valid XOR checksum and mandatory appended 32-byte SHA-256 (`96` bytes,
    /// `M-028`).
    #[inline]
    pub const fn minimal_valid_image_64() -> [u8; 96] {
        Self::minimal_valid_image_96()
    }
}

/// Validates an ESP32 OTA image prefix and total `Content-Length`
/// against target partition bounds (`OTA-002`, `APP-008`, `M-028`, `M-029`).
///
/// Checks:
/// 1. `total_image_len >= ESP_IMAGE_HEADER_LEN` (`24`) and
///    `image_prefix.len() >= ESP_IMAGE_HEADER_LEN` (`24`)
/// 2. `total_image_len <= max_partition_size` (`0x1B0000`)
/// 3. `image_prefix[0] == ESP_IMAGE_HEADER_MAGIC` (`0xE9`)
/// 4. `segment_count` in `1..=16`
///    (`ESP_IMAGE_MIN_SEGMENTS..=ESP_IMAGE_MAX_SEGMENTS`)
/// 5. `spi_mode <= 5` (`M-029` / `C1-008`)
/// 6. `chip_id` (bytes `12..14`) equals [`ESP_IMAGE_CHIP_ID_ESP32`]
/// 7. `entry_addr` in ESP32 IRAM ([`ESP32_IRAM_ENTRY_RANGE`]) or IROM
///    ([`ESP32_IROM_ENTRY_RANGE`]); DROM/DRAM entry points are rejected
///    (`FW-NET-18`)
/// 8. `image_prefix[23] == 1` (mandatory SHA-256 digest appended, `M-028`)
pub fn validate_ota_image_header(
    image_prefix: &[u8],
    total_image_len: usize,
    max_partition_size: usize,
) -> Result<EspImageHeader, OtaError> {
    if total_image_len == 0 {
        return Err(OtaError::MissingContentLength);
    }
    if total_image_len < ESP_IMAGE_HEADER_LEN || image_prefix.len() < ESP_IMAGE_HEADER_LEN {
        return Err(OtaError::ImageTooSmall);
    }
    if total_image_len > max_partition_size {
        return Err(OtaError::ImageTooLarge {
            image_size: total_image_len,
            partition_size: max_partition_size,
        });
    }

    let magic = image_prefix[0];
    if magic != ESP_IMAGE_HEADER_MAGIC {
        return Err(OtaError::InvalidMagicByte(magic));
    }

    let segment_count = image_prefix[1];
    if !(ESP_IMAGE_MIN_SEGMENTS..=ESP_IMAGE_MAX_SEGMENTS).contains(&segment_count) {
        return Err(OtaError::InvalidSegmentCount(segment_count));
    }

    let spi_mode = image_prefix[2];
    if spi_mode > 5 {
        return Err(OtaError::InvalidSpiMode(spi_mode));
    }
    let spi_speed_size = image_prefix[3];
    let entry_addr = u32::from_le_bytes([
        image_prefix[4],
        image_prefix[5],
        image_prefix[6],
        image_prefix[7],
    ]);

    let chip_id = u16::from_le_bytes([image_prefix[12], image_prefix[13]]);
    if chip_id != ESP_IMAGE_CHIP_ID_ESP32 {
        return Err(OtaError::InvalidChipId(chip_id));
    }

    let is_valid_entry = ESP32_IRAM_ENTRY_RANGE.contains(&entry_addr)
        || ESP32_IROM_ENTRY_RANGE.contains(&entry_addr);
    if !is_valid_entry {
        return Err(OtaError::InvalidEntryAddr(entry_addr));
    }

    if image_prefix[23] != 1 {
        return Err(OtaError::InvalidDigestPlacement);
    }

    Ok(EspImageHeader {
        magic,
        segment_count,
        spi_mode,
        spi_speed_size,
        entry_addr,
    })
}

/// Summary returned by [`validate_ota_image_integrity`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EspImageHeaderSummary {
    /// Parsed 24-byte image header.
    pub header: EspImageHeader,
    /// Number of segments validated.
    pub segment_count: u8,
    /// Entry address.
    pub entry_addr: u32,
    /// Total data length across all segments.
    pub total_segment_len: usize,
    /// Computed 8-bit XOR checksum byte (seeded with `0xEF`).
    pub checksum: u8,
    /// Whether a trailing SHA-256 digest was present and verified.
    pub hash_appended: bool,
}

/// Pure `#![no_std]` SHA-256 state and digest computation.
pub struct Sha256 {
    state: [u32; 8],
    buffer: [u8; 64],
    buf_len: usize,
    total_len: u64,
}

impl Sha256 {
    /// Creates a new SHA-256 hasher initialized to standard initial constants.
    pub const fn new() -> Self {
        Self {
            state: [
                0x6a09_e667,
                0xbb67_ae85,
                0x3c6e_f372,
                0xa54f_f53a,
                0x510e_527f,
                0x9b05_688c,
                0x1f83_d9ab,
                0x5be0_cd19,
            ],
            buffer: [0u8; 64],
            buf_len: 0,
            total_len: 0,
        }
    }

    /// Appends `data` bytes to the hash state.
    pub const fn update(&mut self, data: &[u8]) {
        self.total_len = self.total_len.saturating_add(data.len() as u64);
        let mut i = 0;
        while i < data.len() {
            self.buffer[self.buf_len] = data[i];
            self.buf_len += 1;
            if self.buf_len == 64 {
                self.process_block();
                self.buf_len = 0;
            }
            i += 1;
        }
    }

    /// Consumes the hasher and returns the computed 32-byte SHA-256 digest.
    pub const fn finalize(mut self) -> [u8; 32] {
        let bit_len = self.total_len.wrapping_mul(8);
        self.buffer[self.buf_len] = 0x80;
        self.buf_len += 1;

        if self.buf_len > 56 {
            while self.buf_len < 64 {
                self.buffer[self.buf_len] = 0;
                self.buf_len += 1;
            }
            self.process_block();
            self.buf_len = 0;
        }

        while self.buf_len < 56 {
            self.buffer[self.buf_len] = 0;
            self.buf_len += 1;
        }

        let len_bytes = bit_len.to_be_bytes();
        let mut j = 0;
        while j < 8 {
            self.buffer[56 + j] = len_bytes[j];
            j += 1;
        }
        self.process_block();

        let mut out = [0u8; 32];
        let mut k = 0;
        while k < 8 {
            let bytes = self.state[k].to_be_bytes();
            out[k * 4] = bytes[0];
            out[k * 4 + 1] = bytes[1];
            out[k * 4 + 2] = bytes[2];
            out[k * 4 + 3] = bytes[3];
            k += 1;
        }
        out
    }

    const fn process_block(&mut self) {
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

        let mut w = [0u32; 64];
        let mut t = 0;
        while t < 16 {
            w[t] = u32::from_be_bytes([
                self.buffer[t * 4],
                self.buffer[t * 4 + 1],
                self.buffer[t * 4 + 2],
                self.buffer[t * 4 + 3],
            ]);
            t += 1;
        }
        while t < 64 {
            let s0 = w[t - 15].rotate_right(7) ^ w[t - 15].rotate_right(18) ^ (w[t - 15] >> 3);
            let s1 = w[t - 2].rotate_right(17) ^ w[t - 2].rotate_right(19) ^ (w[t - 2] >> 10);
            w[t] = w[t - 16]
                .wrapping_add(s0)
                .wrapping_add(w[t - 7])
                .wrapping_add(s1);
            t += 1;
        }

        let mut a = self.state[0];
        let mut b = self.state[1];
        let mut c = self.state[2];
        let mut d = self.state[3];
        let mut e = self.state[4];
        let mut f = self.state[5];
        let mut g = self.state[6];
        let mut h = self.state[7];

        let mut i = 0;
        while i < 64 {
            let s1 = e.rotate_right(6) ^ e.rotate_right(11) ^ e.rotate_right(25);
            let ch = (e & f) ^ ((!e) & g);
            let temp1 = h
                .wrapping_add(s1)
                .wrapping_add(ch)
                .wrapping_add(K[i])
                .wrapping_add(w[i]);
            let s0 = a.rotate_right(2) ^ a.rotate_right(13) ^ a.rotate_right(22);
            let maj = (a & b) ^ (a & c) ^ (b & c);
            let temp2 = s0.wrapping_add(maj);

            h = g;
            g = f;
            f = e;
            e = d.wrapping_add(temp1);
            d = c;
            c = b;
            b = a;
            a = temp1.wrapping_add(temp2);
            i += 1;
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
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

/// Computes the 32-byte SHA-256 digest of `data` (`#![no_std]`,
/// zero allocation, `const fn`).
pub const fn sha256_digest(data: &[u8]) -> [u8; 32] {
    let mut h = Sha256::new();
    h.update(data);
    h.finalize()
}

/// Validates the full integrity of an ESP32 firmware application image.
///
/// 1. Validates the 24-byte header via [`validate_ota_image_header`] (including
///    mandatory `image[23] == 1` SHA-256 flag, `M-028`).
/// 2. Iterates over each of the `segment_count` segments starting at offset 24,
///    verifying 4-byte `load_addr` alignment and that each segment's header
///    (8 bytes) and payload lie within `image` (`M-029`).
/// 3. Computes the ESP-IDF 8-bit XOR checksum (seeded with `0xEF`) over all
///    segment payloads and verifies the byte at boundary `aligned_end - 1`.
/// 4. Unconditionally requires the 32-byte SHA-256 digest at exactly
///    `aligned_end..aligned_end + 32 == image.len()` (rejecting unverified
///    trailing bytes and `image[23] != 1`, `M-028` / `M-029`).
pub fn validate_ota_image_integrity(image: &[u8]) -> Result<EspImageHeaderSummary, OtaError> {
    let header = validate_ota_image_header(image, image.len(), OTA_PARTITION_MAX_SIZE)?;

    let mut offset = ESP_IMAGE_HEADER_LEN;
    let mut chk = 0xEFu8;
    let mut total_seg_len = 0usize;

    let mut seg_idx = 0u8;
    while seg_idx < header.segment_count {
        if offset.checked_add(8).ok_or(OtaError::ImageTooSmall)? > image.len() {
            return Err(OtaError::ImageTooSmall);
        }
        let load_addr = u32::from_le_bytes([
            image[offset],
            image[offset + 1],
            image[offset + 2],
            image[offset + 3],
        ]);
        let data_len = u32::from_le_bytes([
            image[offset + 4],
            image[offset + 5],
            image[offset + 6],
            image[offset + 7],
        ]) as usize;
        if data_len > 0 && (load_addr & 3) != 0 {
            return Err(OtaError::InvalidEntryAddr(load_addr));
        }
        offset += 8;

        let end_seg = offset
            .checked_add(data_len)
            .ok_or(OtaError::ImageTooSmall)?;
        if end_seg > image.len() {
            return Err(OtaError::ImageTooSmall);
        }

        let mut b_idx = offset;
        while b_idx < end_seg {
            chk ^= image[b_idx];
            b_idx += 1;
        }

        offset = end_seg;
        total_seg_len = total_seg_len.saturating_add(data_len);
        seg_idx += 1;
    }

    let aligned_end = (offset.checked_add(16).ok_or(OtaError::ImageTooSmall)?) & !15;
    let checksum_offset = aligned_end.checked_sub(1).ok_or(OtaError::ImageTooSmall)?;
    if image.len() <= checksum_offset {
        return Err(OtaError::ImageTooSmall);
    }

    if image[checksum_offset] != chk {
        return Err(OtaError::InvalidChecksum);
    }

    if image[23] != 1 {
        return Err(OtaError::InvalidDigestPlacement);
    }
    // The digest must sit immediately after the checksum block and end
    // the image exactly (`FW-NET-18` / `M-028` / `M-029`); no trailing bytes.
    if aligned_end.checked_add(ESP_IMAGE_DIGEST_LEN) != Some(image.len()) {
        return Err(OtaError::InvalidDigestPlacement);
    }
    let expected_hash = &image[aligned_end..];
    let computed_hash = sha256_digest(&image[..aligned_end]);
    if computed_hash != expected_hash {
        return Err(OtaError::InvalidSha256);
    }

    Ok(EspImageHeaderSummary {
        header,
        segment_count: header.segment_count,
        entry_addr: header.entry_addr,
        total_segment_len: total_seg_len,
        checksum: chk,
        hash_appended: true,
    })
}

/// Errors returned by the OTA authentication, header validation, and
/// `otadata` state machine.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum OtaError {
    /// Missing or invalid `X-OTA-Token` header (`401 Unauthorized`).
    Unauthorized,
    /// No passive OTA partition available (`500 Internal Server Error`).
    NoPassivePartition,
    /// Missing or non-positive `Content-Length` header (`400 Bad Request`).
    MissingContentLength,
    /// Image size exceeds target OTA partition capacity
    /// (`400 Bad Request`, `APP-008`).
    ImageTooLarge {
        image_size: usize,
        partition_size: usize,
    },
    /// Firmware payload is smaller than minimal header or segment length
    /// (`400 Bad Request`).
    ImageTooSmall,
    /// Firmware payload is empty or too short to inspect header
    /// (`400 Bad Request`).
    HeaderTooShort,
    /// First byte of firmware image is not `0xE9` (`400 Bad Request`, `OTA-002`).
    InvalidMagicByte(u8),
    /// Segment count in ESP32 image header is outside `1..=16`
    /// (`400 Bad Request`, `OTA-002`).
    InvalidSegmentCount(u8),
    /// SPI flash interface mode in ESP32 image header is outside `0..=5`
    /// (`400 Bad Request`, `M-029`).
    InvalidSpiMode(u8),
    /// Entry address outside ESP32 executable memory regions (`400 Bad Request`).
    InvalidEntryAddr(u32),
    /// Image `chip_id` is not ESP32 (`0x0000`), e.g. an ESP32-S3 build
    /// (`400 Bad Request`, `FW-NET-18`).
    InvalidChipId(u16),
    /// `hash_appended` is set but the digest does not end the image exactly at
    /// `aligned_end + 32` (`400 Bad Request`, `FW-NET-18`).
    InvalidDigestPlacement,
    /// ESP-IDF 8-bit XOR checksum mismatch (`400 Bad Request`).
    InvalidChecksum,
    /// Appended SHA-256 digest mismatch (`400 Bad Request`).
    InvalidSha256,
    /// Number of received bytes did not match `Content-Length` (`400 Bad Request`).
    IncompletePayload { received: usize, expected: usize },
    /// Socket read error or retry limit exceeded
    /// (`500 Internal Server Error`, `APP-007`).
    SocketReadTimeout,
    /// Underlying flash erase/write failure (`500 Internal Server Error`).
    FlashWriteError,
    /// The image read back from flash after writing does not hash to the
    /// received payload; `otadata` is left unchanged
    /// (`500 Internal Server Error`, `FW-NET-18`).
    ReadbackMismatch,
    /// `otadata` CRC-32 validation failed (`500 Internal Server Error`).
    InvalidOtaDataCrc,
    /// Current slot is in `Invalid` or `Aborted` state and cannot cancel rollback.
    RollbackStateInvalid,
    /// `ota_seq` counter overflowed `u32::MAX`.
    SequenceOverflow,
}

impl fmt::Display for OtaError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Unauthorized => write!(f, "Unauthorized: Invalid or missing X-OTA-Token"),
            Self::NoPassivePartition => write!(f, "No passive OTA partition found"),
            Self::MissingContentLength => write!(f, "Content-Length header required"),
            Self::ImageTooLarge {
                image_size,
                partition_size,
            } => write!(
                f,
                "OTA image size ({}) exceeds target partition size ({})",
                image_size, partition_size
            ),
            Self::ImageTooSmall => write!(f, "Firmware image validation failed: image too small"),
            Self::HeaderTooShort => write!(f, "Firmware image validation failed"),
            Self::InvalidMagicByte(b) => {
                write!(
                    f,
                    "Firmware image validation failed: invalid magic 0x{:02X}",
                    b
                )
            }
            Self::InvalidSegmentCount(c) => {
                write!(
                    f,
                    "Firmware image validation failed: invalid segments {}",
                    c
                )
            }
            Self::InvalidSpiMode(m) => {
                write!(
                    f,
                    "Firmware image validation failed: invalid spi_mode {}",
                    m
                )
            }
            Self::InvalidEntryAddr(addr) => {
                write!(
                    f,
                    "Firmware image validation failed: invalid entry 0x{:08X}",
                    addr
                )
            }
            Self::InvalidChipId(id) => {
                write!(
                    f,
                    "Firmware image validation failed: chip_id 0x{:04X} is not ESP32",
                    id
                )
            }
            Self::InvalidDigestPlacement => {
                write!(
                    f,
                    "Firmware image validation failed: misplaced SHA-256 digest"
                )
            }
            Self::InvalidChecksum => {
                write!(f, "Firmware image validation failed: checksum mismatch")
            }
            Self::InvalidSha256 => write!(f, "Firmware image validation failed: SHA-256 mismatch"),
            Self::IncompletePayload { received, expected } => write!(
                f,
                "OTA socket read error: received {} of {} bytes",
                received, expected
            ),
            Self::SocketReadTimeout => write!(f, "OTA socket read error or timeout"),
            Self::FlashWriteError => write!(f, "esp_ota_write failed"),
            Self::ReadbackMismatch => write!(f, "OTA flash read-back verification failed"),
            Self::InvalidOtaDataCrc => write!(f, "Invalid otadata CRC32"),
            Self::RollbackStateInvalid => write!(f, "Cannot mark invalid/aborted slot valid"),
            Self::SequenceOverflow => write!(f, "OTA sequence counter overflow"),
        }
    }
}

#[cfg(test)]
pub fn make_valid_esp_image(hash_appended: bool) -> alloc::vec::Vec<u8> {
    make_valid_esp_image_with_len(16, hash_appended)
}

#[cfg(test)]
pub fn make_valid_esp_image_with_len(
    payload_len: usize,
    hash_appended: bool,
) -> alloc::vec::Vec<u8> {
    let seg_offset = 24 + 8;
    let end_of_seg = seg_offset + payload_len;
    let checksum_offset = end_of_seg | 15;
    let base_len = checksum_offset + 1;
    let mut img = alloc::vec![0u8; base_len];
    img[0] = ESP_IMAGE_HEADER_MAGIC;
    img[1] = 1; // 1 segment
    img[2] = 0x02; // DIO
    img[3] = 0x20; // 4MB, 40MHz
    img[4..8].copy_from_slice(&0x4008_0400u32.to_le_bytes());
    if hash_appended {
        img[23] = 1;
    }
    // Segment 0 header
    img[24..28].copy_from_slice(&0x4008_0400u32.to_le_bytes());
    img[28..32].copy_from_slice(&(payload_len as u32).to_le_bytes());
    // Fill payload
    let mut chk = 0xEFu8;
    for (idx, b) in img[seg_offset..end_of_seg].iter_mut().enumerate() {
        let val = (idx as u8) ^ 0x5A;
        *b = val;
        chk ^= val;
    }
    img[checksum_offset] = chk;
    if hash_appended {
        let digest = sha256_digest(&img[..base_len]);
        img.extend_from_slice(&digest);
    }
    img
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_partition_table_matches_csv() {
        assert!(verify_partition_table_integrity());
        assert_eq!(PARTITION_TABLE.len(), 5);

        let csv = include_str!("../../../partitions_rust_ota.csv");
        for line in csv.lines() {
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            let mut parts = line.split(',').map(|s| s.trim());
            let name = parts.next().expect("partition name");
            let _part_type = parts.next().expect("partition type");
            let _sub_type = parts.next().expect("partition subtype");
            let offset_str = parts.next().expect("offset");
            let size_str = parts.next().expect("size");

            let offset = if let Some(hex) = offset_str
                .strip_prefix("0x")
                .or_else(|| offset_str.strip_prefix("0X"))
            {
                u32::from_str_radix(hex, 16).expect("hex offset")
            } else {
                offset_str.parse::<u32>().expect("dec offset")
            };
            let size = if let Some(hex) = size_str
                .strip_prefix("0x")
                .or_else(|| size_str.strip_prefix("0X"))
            {
                u32::from_str_radix(hex, 16).expect("hex size")
            } else {
                size_str.parse::<u32>().expect("dec size")
            };

            let entry = find_partition_by_label(name)
                .unwrap_or_else(|| panic!("Partition {} not found", name));
            assert_eq!(entry.offset, offset, "Partition {} offset mismatch", name);
            assert_eq!(entry.size, size, "Partition {} size mismatch", name);
        }
    }

    #[test]
    fn test_constant_time_verify_ota_token() {
        let expected = b"klipper-cam-ota-2026";
        assert!(verify_ota_token(b"klipper-cam-ota-2026", expected));
        assert!(!verify_ota_token(b"klipper-cam-ota-2025", expected));
        assert!(!verify_ota_token(b"klipper-cam-ota-2026-extra", expected));
        assert!(!verify_ota_token(b"klipper", expected));
        assert!(!verify_ota_token(b"", expected));
        assert!(!verify_ota_token(b"klipper-cam-ota-2026", b""));
        assert!(!verify_ota_token(b"", b""));
    }

    #[test]
    fn test_is_seq_greater_strict_weak_ordering() {
        assert!(!is_seq_greater(1, 1));
        assert!(!is_seq_greater(u32::MAX, u32::MAX));
        assert!(is_seq_greater(2, 1));
        assert!(!is_seq_greater(1, 2));
        assert!(is_seq_greater(1, u32::MAX));
        assert!(!is_seq_greater(u32::MAX, 1));
    }

    #[test]
    fn test_ieee_802_3_crc32_and_otadata_select() {
        // Standard IEEE 802.3 check value for ASCII "123456789"
        assert_eq!(crc32_ieee(b"123456789"), 0xCBF4_3926);

        let erased = OtaDataSelect::erased();
        assert!(!erased.is_crc_valid());
        assert!(!erased.is_bootable());
        assert_eq!(erased.slot(), None);

        let mut s0 = OtaDataSelect::new(1, OtaImgState::PendingVerify);
        assert_eq!(core::mem::size_of::<OtaDataSelect>(), 32);
        assert!(s0.is_crc_valid());
        assert!(s0.is_bootable());
        assert_eq!(s0.slot(), Some(OtaSlot::Ota0));
        assert_eq!(s0.state(), OtaImgState::PendingVerify);

        // M-032: Verbatim round-trip serialization for both PendingVerify (1)
        // and New (0) with ESP-IDF bootloader_common_ota_select_crc parity.
        let raw = s0.to_bytes();
        assert_eq!(&raw[24..28], &[1, 0, 0, 0]);
        assert_eq!(&raw[28..32], &[154, 152, 67, 71]);
        assert_eq!(OtaDataSelect::from_bytes(&raw), s0);

        let s_new = OtaDataSelect::new(2, OtaImgState::New);
        let s_new_raw = s_new.to_bytes();
        assert_eq!(&s_new_raw[24..32], &[0, 0, 0, 0, 116, 55, 246, 85]);
        assert_eq!(OtaDataSelect::from_bytes(&s_new_raw), s_new);
        assert_eq!(
            OtaDataSelect::from_bytes(&s_new_raw).state(),
            OtaImgState::New
        );

        // Rollback cancellation transition: PendingVerify -> Valid
        let res = s0.mark_app_valid_cancel_rollback();
        assert_eq!(res, Ok(true));
        assert_eq!(s0.state(), OtaImgState::Valid);
        assert!(s0.is_crc_valid());

        // Second call is idempotent
        assert_eq!(s0.mark_app_valid_cancel_rollback(), Ok(false));
    }

    #[test]
    fn test_otadata_inactive_slot_selection_and_ping_pong_commit() {
        let mut flash: MockNorFlash<8192> = MockNorFlash::new();
        // sector_0 holds seq=2 (Ota1), sector_1 holds seq=1 (Ota0)
        let s0 = OtaDataSelect::new(2, OtaImgState::Valid);
        let s1 = OtaDataSelect::new(1, OtaImgState::Valid);
        let mut mgr = OtaDataManager::from_sectors(s0, s1);

        assert_eq!(mgr.running_slot(), OtaSlot::Ota1);
        assert_eq!(mgr.active_slot(), OtaSlot::Ota1);
        assert_eq!(mgr.select_inactive_slot(), OtaSlot::Ota0);
        assert_eq!(mgr.select_inactive_otadata_sector(), 1);

        // Commit update targeting the inactive slot (Ota0)
        let target_slot = mgr.select_inactive_slot();
        let entry = mgr
            .commit_update(&mut flash, target_slot)
            .expect("commit update to inactive sector");

        assert_eq!(entry.ota_seq, 3);
        assert_eq!(entry.slot(), Some(OtaSlot::Ota0));
        assert_eq!(entry.state(), OtaImgState::PendingVerify);

        // Verify sector_1 was updated with seq=3, while sector_0 remains
        // untouched at seq=2
        assert_eq!(mgr.sector_0.ota_seq, 2);
        assert_eq!(mgr.sector_0.state(), OtaImgState::Valid);
        assert_eq!(mgr.sector_1.ota_seq, 3);
        assert_eq!(mgr.sector_1.state(), OtaImgState::PendingVerify);

        // M-004: After commit, next-boot active_slot is Ota0, while running_slot
        // stays Ota1, select_inactive_slot stays Ota0 (never XIP self-erase),
        // and pending_reboot_slot latches Some(Ota0).
        assert_eq!(mgr.active_slot(), OtaSlot::Ota0);
        assert_eq!(mgr.running_slot(), OtaSlot::Ota1);
        assert_eq!(mgr.select_inactive_slot(), OtaSlot::Ota0);
        assert_eq!(mgr.pending_reboot_slot(), Some(OtaSlot::Ota0));
        assert_eq!(
            mgr.commit_update(&mut flash, OtaSlot::Ota0),
            Err(OtaError::RollbackStateInvalid)
        );

        // Flash sector 1 (0x18000) was written with the new entry
        let mut buf1 = AlignedOtaBuf::default();
        flash
            .read_aligned(OTADATA_SECTOR_1_OFFSET, &mut buf1.0)
            .expect("read sector 1");
        assert_eq!(OtaDataSelect::from_bytes(&buf1.0), entry);
    }

    #[test]
    fn test_otadata_manager_slot_rotation_and_rollback_cancel() {
        let mut mgr = OtaDataManager::new_booted_ota0();
        assert_eq!(mgr.active_slot(), OtaSlot::Ota0);
        assert_eq!(mgr.next_update_slot(), OtaSlot::Ota1);

        // Flash to ota_1
        let e1 = mgr.set_boot_slot(OtaSlot::Ota1);
        assert!(e1.is_ok());
        assert_eq!(mgr.active_slot(), OtaSlot::Ota1);
        assert_eq!(mgr.running_slot(), OtaSlot::Ota0);
        assert_eq!(mgr.next_update_slot(), OtaSlot::Ota1);
        assert_eq!(mgr.sector_1.state(), OtaImgState::PendingVerify);

        // Cannot validate unbooted Ota1 before reboot (M-004)
        assert_eq!(
            mgr.mark_app_valid_cancel_rollback(),
            Err(OtaError::RollbackStateInvalid)
        );

        // Simulate reboot into Ota1
        let mut booted = OtaDataManager::from_sectors(mgr.sector_0, mgr.sector_1);
        assert_eq!(booted.running_slot(), OtaSlot::Ota1);
        assert_eq!(booted.next_update_slot(), OtaSlot::Ota0);

        // Confirm valid after Wi-Fi GOT_IP + HTTP
        assert_eq!(booted.mark_app_valid_cancel_rollback(), Ok(true));
        assert_eq!(booted.sector_1.state(), OtaImgState::Valid);

        // Next OTA back to ota_0
        let e2 = booted.set_boot_slot(OtaSlot::Ota0);
        assert!(e2.is_ok());
        assert_eq!(booted.active_slot(), OtaSlot::Ota0);
        assert_eq!(booted.sector_0.ota_seq, 3);
        assert_eq!(booted.sector_0.state(), OtaImgState::PendingVerify);
    }

    #[test]
    fn test_esp_image_header_validation() {
        let valid_hdr = EspImageHeader::minimal_valid_header_bytes(3, 0x4008_0400);
        let parsed = validate_ota_image_header(&valid_hdr, 512_000, OTA_PARTITION_MAX_SIZE);
        assert!(parsed.is_ok());
        if let Ok(hdr) = parsed {
            assert_eq!(hdr.magic, 0xE9);
            assert_eq!(hdr.segment_count, 3);
            assert_eq!(hdr.entry_addr, 0x4008_0400);
        }

        // Header < 24 bytes
        assert_eq!(
            validate_ota_image_header(&valid_hdr[..23], 1024, OTA_PARTITION_MAX_SIZE),
            Err(OtaError::ImageTooSmall)
        );

        // Bad magic byte (0x00 instead of 0xE9)
        let mut bad_magic = valid_hdr;
        bad_magic[0] = 0x00;
        assert_eq!(
            validate_ota_image_header(&bad_magic, 1024, OTA_PARTITION_MAX_SIZE),
            Err(OtaError::InvalidMagicByte(0x00))
        );

        // Invalid segment count (0 and 17)
        let mut zero_seg = valid_hdr;
        zero_seg[1] = 0;
        assert_eq!(
            validate_ota_image_header(&zero_seg, 1024, OTA_PARTITION_MAX_SIZE),
            Err(OtaError::InvalidSegmentCount(0))
        );
        let mut seventeen_seg = valid_hdr;
        seventeen_seg[1] = 17;
        assert_eq!(
            validate_ota_image_header(&seventeen_seg, 1024, OTA_PARTITION_MAX_SIZE),
            Err(OtaError::InvalidSegmentCount(17))
        );

        // Invalid entry address
        let mut bad_entry = valid_hdr;
        bad_entry[4..8].copy_from_slice(&0x1000_0000u32.to_le_bytes());
        assert_eq!(
            validate_ota_image_header(&bad_entry, 1024, OTA_PARTITION_MAX_SIZE),
            Err(OtaError::InvalidEntryAddr(0x1000_0000))
        );

        // Exceeds partition size (0x1B0000 + 1)
        assert_eq!(
            validate_ota_image_header(
                &valid_hdr,
                OTA_PARTITION_MAX_SIZE + 1,
                OTA_PARTITION_MAX_SIZE,
            ),
            Err(OtaError::ImageTooLarge {
                image_size: OTA_PARTITION_MAX_SIZE + 1,
                partition_size: OTA_PARTITION_MAX_SIZE,
            })
        );

        // Zero Content-Length
        assert_eq!(
            validate_ota_image_header(&valid_hdr, 0, OTA_PARTITION_MAX_SIZE),
            Err(OtaError::MissingContentLength)
        );
    }

    #[test]
    fn test_validate_ota_image_integrity() {
        // M-028: Images without mandatory SHA-256 (image[23] != 1) are rejected.
        let img_no_hash = make_valid_esp_image(false);
        assert_eq!(
            validate_ota_image_integrity(&img_no_hash),
            Err(OtaError::InvalidDigestPlacement)
        );
        let mut bad_hash_flag = make_valid_esp_image(true);
        bad_hash_flag[23] = 2;
        assert_eq!(
            validate_ota_image_integrity(&bad_hash_flag),
            Err(OtaError::InvalidDigestPlacement)
        );

        // Valid image with appended SHA-256
        let img_hash = make_valid_esp_image(true);
        let summary_hash = validate_ota_image_integrity(&img_hash);
        assert!(summary_hash.is_ok());
        if let Ok(sum) = summary_hash {
            assert_eq!(sum.segment_count, 1);
            assert_eq!(sum.entry_addr, 0x4008_0400);
            assert!(sum.hash_appended);
        }

        // Const minimal_valid_image_96() / minimal_valid_image_64() passes full check.
        let min96 = EspImageHeader::minimal_valid_image_96();
        assert!(validate_ota_image_integrity(&min96).is_ok());

        // M-029: Invalid spi_mode (> 5) and unaligned segment load_addr rejected.
        let mut bad_spi = img_hash.clone();
        bad_spi[2] = 6;
        assert_eq!(
            validate_ota_image_integrity(&bad_spi),
            Err(OtaError::InvalidSpiMode(6))
        );
        let mut unaligned_seg = img_hash.clone();
        unaligned_seg[24..28].copy_from_slice(&0x4008_0401u32.to_le_bytes());
        assert_eq!(
            validate_ota_image_integrity(&unaligned_seg),
            Err(OtaError::InvalidEntryAddr(0x4008_0401))
        );

        // Corrupted checksum byte
        let mut corrupted_chk = img_hash.clone();
        let chk_idx = 63;
        corrupted_chk[chk_idx] ^= 0xFF;
        assert_eq!(
            validate_ota_image_integrity(&corrupted_chk),
            Err(OtaError::InvalidChecksum)
        );

        // Corrupted payload byte -> checksum mismatch
        let mut corrupted_payload = img_hash.clone();
        corrupted_payload[35] ^= 0x01;
        assert_eq!(
            validate_ota_image_integrity(&corrupted_payload),
            Err(OtaError::InvalidChecksum)
        );

        // Corrupted appended SHA-256
        let mut corrupted_sha = img_hash.clone();
        let last_idx = corrupted_sha.len() - 1;
        corrupted_sha[last_idx] ^= 0x55;
        assert_eq!(
            validate_ota_image_integrity(&corrupted_sha),
            Err(OtaError::InvalidSha256)
        );

        // Truncated image
        assert_eq!(
            validate_ota_image_integrity(&img_hash[..40]),
            Err(OtaError::ImageTooSmall)
        );
    }

    #[test]
    fn test_set_boot_slot_wraparound() {
        // M-033: Near u32::MAX must fail-safe with SequenceOverflow instead of
        // wrapping to 1/2 and diverging from ESP-IDF unsigned bootloader order.
        let mut mgr = OtaDataManager::from_sectors(
            OtaDataSelect::new(u32::MAX - 2, OtaImgState::Valid),
            OtaDataSelect::erased(),
        );
        // u32::MAX - 2 (Ota0) -> u32::MAX - 1 (Ota1) is the last valid seq.
        let last = mgr.set_boot_slot(OtaSlot::Ota1).expect("last valid seq");
        assert_eq!(last.ota_seq, u32::MAX - 1);
        assert_eq!(last.slot(), Some(OtaSlot::Ota1));

        // Any advance past u32::MAX - 1 overflows instead of wrapping.
        let mut mgr_ota0_wrap = OtaDataManager::from_sectors(
            OtaDataSelect::erased(),
            OtaDataSelect::new(u32::MAX - 1, OtaImgState::Valid),
        );
        assert_eq!(
            mgr_ota0_wrap.set_boot_slot(OtaSlot::Ota0),
            Err(OtaError::SequenceOverflow)
        );
    }

    #[test]
    fn test_ota_flash_storage_load_write_and_cancel_rollback() {
        let mut flash: MockNorFlash<8192> = MockNorFlash::new();
        assert_eq!(core::mem::align_of::<AlignedOtaBuf>(), 4);
        assert_eq!(core::mem::align_of::<AlignedFlashChunk>(), 4);

        // 1. Blank factory flash initializes to ota_0 (ota_seq = 1, Valid)
        let mut mgr = OtaDataManager::load_from_flash(&mut flash).expect("load blank flash");
        assert_eq!(mgr.active_slot(), OtaSlot::Ota0);
        assert_eq!(mgr.next_update_slot(), OtaSlot::Ota1);
        mgr.sync_slot_to_flash(&mut flash, OtaSlot::Ota0)
            .expect("sync initial ota_0 sector");

        // 2. Build a valid firmware image with multi-chunk data + SHA-256
        let image = make_valid_esp_image_with_len(980, true);
        let img_len = image.len();

        let slot = mgr
            .write_ota_image_to_flash(&mut flash, &image)
            .expect("write OTA image to ota_1");
        assert_eq!(slot, OtaSlot::Ota1);
        assert_eq!(mgr.active_slot(), OtaSlot::Ota1);
        assert_eq!(mgr.sector_1.state(), OtaImgState::PendingVerify);

        // Verify ota_1 flash contents
        assert_eq!(&flash.ota_1[..img_len], &image[..]);

        // 3. Simulate reboot: reload OtaDataManager from flash and verify
        //    PendingVerify state on ota_1
        let mut booted_mgr =
            OtaDataManager::load_from_flash(&mut flash).expect("reload otadata after reboot");
        assert_eq!(booted_mgr.active_slot(), OtaSlot::Ota1);
        assert_eq!(booted_mgr.sector_1.ota_seq, 2);
        assert_eq!(booted_mgr.sector_1.state(), OtaImgState::PendingVerify);

        // 4. Wi-Fi GOT_IP cancels rollback and persists Valid state to flash
        let changed = booted_mgr
            .cancel_rollback_in_flash(&mut flash)
            .expect("cancel rollback in flash");
        assert!(changed);
        assert_eq!(booted_mgr.sector_1.state(), OtaImgState::Valid);

        // Second call is idempotent and does not rewrite flash
        let writes_before = flash.write_count;
        let changed_again = booted_mgr
            .cancel_rollback_in_flash(&mut flash)
            .expect("idempotent cancel rollback");
        assert!(!changed_again);
        assert_eq!(flash.write_count, writes_before);

        // Reload again from flash to confirm Valid state was persisted to
        // sector 1 (0x18000)
        let reloaded = OtaDataManager::load_from_flash(&mut flash).expect("reload after Valid");
        assert_eq!(reloaded.active_slot(), OtaSlot::Ota1);
        assert_eq!(reloaded.sector_1.state(), OtaImgState::Valid);
    }

    // ---- WP-09 / FW-NET-18: OTA image header and write-path hardening ----

    #[test]
    fn test_wp09_rejects_non_esp32_chip_id() {
        // ESP32-S3 images carry `chip_id = 0x0009` at header bytes 12..14.
        let mut s3 = make_valid_esp_image(true);
        s3[12..14].copy_from_slice(&0x0009u16.to_le_bytes());
        assert_eq!(
            validate_ota_image_header(&s3, s3.len(), OTA_PARTITION_MAX_SIZE),
            Err(OtaError::InvalidChipId(0x0009))
        );
        assert_eq!(
            validate_ota_image_integrity(&s3),
            Err(OtaError::InvalidChipId(0x0009))
        );

        let mut flash: MockNorFlash<8192> = MockNorFlash::new_booted_ota0();
        let before = flash.clone();
        let mut mgr = OtaDataManager::load_from_flash(&mut flash).expect("load");
        assert_eq!(
            mgr.write_ota_image_to_flash(&mut flash, &s3),
            Err(OtaError::InvalidChipId(0x0009))
        );
        assert_eq!(flash, before, "rejected image must not touch flash");

        // chip_id == 0 (ESP32) is accepted.
        let esp32 = make_valid_esp_image(true);
        assert!(validate_ota_image_integrity(&esp32).is_ok());
    }

    #[test]
    fn test_wp09_entry_point_must_be_iram_or_irom() {
        let accepted = [
            0x4007_0000u32, // IRAM start
            0x4008_0400,    // typical call_start_cpu0 in IRAM
            0x4009_FFFC,    // IRAM end - 4
            0x400D_0000,    // IROM start
            0x400D_1234,    // typical IROM entry
            0x403F_FFFC,    // IROM end - 4
        ];
        for addr in accepted {
            let hdr = EspImageHeader::minimal_valid_header_bytes(1, addr);
            assert!(
                validate_ota_image_header(&hdr, 1024, OTA_PARTITION_MAX_SIZE).is_ok(),
                "entry 0x{addr:08X} must be accepted"
            );
        }
        let rejected = [
            0x3F40_0020u32, // DROM (data, not executable)
            0x3FFB_0000,    // DRAM
            0x4006_FFFC,    // below IRAM
            0x400A_0000,    // IRAM end (exclusive)
            0x400C_0000,    // RTC fast / between IRAM and IROM
            0x4040_0000,    // IROM end (exclusive)
            0x4080_0000,    // outside the ESP32 map
        ];
        for addr in rejected {
            let hdr = EspImageHeader::minimal_valid_header_bytes(1, addr);
            assert_eq!(
                validate_ota_image_header(&hdr, 1024, OTA_PARTITION_MAX_SIZE),
                Err(OtaError::InvalidEntryAddr(addr)),
                "entry 0x{addr:08X} must be rejected"
            );
        }
    }

    #[test]
    fn test_wp09_misplaced_digest_rejected() {
        let good = make_valid_esp_image(true);
        let aligned_end = good.len() - 32;
        assert!(validate_ota_image_integrity(&good).is_ok());

        // Trailing bytes after a correctly placed digest.
        let mut trailing = good.clone();
        trailing.extend_from_slice(&[0xFF; 16]);
        assert_eq!(
            validate_ota_image_integrity(&trailing),
            Err(OtaError::InvalidDigestPlacement)
        );

        // Padding between checksum and digest, with the digest re-computed
        // over everything before it and placed at the very end.
        let mut padded = good[..aligned_end].to_vec();
        padded.extend_from_slice(&[0xFF; 16]);
        let digest = sha256_digest(&padded);
        padded.extend_from_slice(&digest);
        assert_eq!(
            validate_ota_image_integrity(&padded),
            Err(OtaError::InvalidDigestPlacement)
        );

        // `hash_appended` set but no digest present at all.
        let missing = good[..aligned_end].to_vec();
        assert_eq!(
            validate_ota_image_integrity(&missing),
            Err(OtaError::InvalidDigestPlacement)
        );
    }

    /// Flash wrapper that flips one bit in the target app partition after
    /// the write that covers `flip_offset`, simulating a marginal NOR cell.
    struct BitFlipFlash {
        inner: MockNorFlash<8192>,
        flip_offset: u32,
        flipped: bool,
    }

    impl OtaFlashStorage for BitFlipFlash {
        fn read_aligned(&mut self, offset: u32, buf: &mut [u8]) -> Result<(), OtaError> {
            self.inner.read_aligned(offset, buf)
        }

        fn erase_range(&mut self, from: u32, to: u32) -> Result<(), OtaError> {
            self.inner.erase_range(from, to)
        }

        fn write_aligned(&mut self, offset: u32, data: &[u8]) -> Result<(), OtaError> {
            self.inner.write_aligned(offset, data)?;
            let end = offset + data.len() as u32;
            if !self.flipped && (offset..end).contains(&self.flip_offset) {
                let idx = (self.flip_offset - OTA_1_OFFSET) as usize;
                self.inner.ota_1[idx] ^= 0x04;
                self.flipped = true;
            }
            Ok(())
        }
    }

    #[test]
    fn test_wp09_readback_bitflip_leaves_otadata_unchanged() {
        let mut flash = BitFlipFlash {
            inner: MockNorFlash::new_booted_ota0(),
            flip_offset: OTA_1_OFFSET + 1500,
            flipped: false,
        };
        let mut mgr = OtaDataManager::load_from_flash(&mut flash).expect("load");
        let otadata_s0 = flash.inner.otadata_s0;
        let otadata_s1 = flash.inner.otadata_s1;
        let mgr_before = mgr;

        let image = make_valid_esp_image_with_len(2000, true);
        assert!(validate_ota_image_integrity(&image).is_ok());
        assert_eq!(
            mgr.write_ota_image_to_flash(&mut flash, &image),
            Err(OtaError::ReadbackMismatch)
        );
        assert!(flash.flipped, "fault must have been injected");
        assert_eq!(flash.inner.otadata_s0, otadata_s0);
        assert_eq!(flash.inner.otadata_s1, otadata_s1);
        assert_eq!(mgr, mgr_before, "in-memory otadata must stay unchanged");
        assert_eq!(mgr.active_slot(), OtaSlot::Ota0);

        // Without the fault the same image commits.
        let mut clean: MockNorFlash<8192> = MockNorFlash::new_booted_ota0();
        let mut mgr2 = OtaDataManager::load_from_flash(&mut clean).expect("load");
        assert_eq!(
            mgr2.write_ota_image_to_flash(&mut clean, &image),
            Ok(OtaSlot::Ota1)
        );
        assert_eq!(&clean.ota_1[..image.len()], &image[..]);
    }

    #[test]
    fn test_wp09_noop_flash_skips_readback() {
        assert!(!NoOpFlashStorage.supports_readback());
        assert!(MockNorFlash::<8192>::new().supports_readback());
        let mut mgr = OtaDataManager::new_booted_ota0();
        let image = make_valid_esp_image(true);
        assert_eq!(
            mgr.write_ota_image_to_flash(&mut NoOpFlashStorage, &image),
            Ok(OtaSlot::Ota1)
        );
    }

    #[test]
    fn test_m003_post_rollback_preserves_sole_valid_sector_and_advances_seq() {
        let mut flash: MockNorFlash<8192> = MockNorFlash::new();
        let s0 = OtaDataSelect::new(1, OtaImgState::Valid);
        let s1 = OtaDataSelect::new(2, OtaImgState::Aborted);
        flash.otadata_s0[..32].copy_from_slice(&s0.to_bytes());
        flash.otadata_s1[..32].copy_from_slice(&s1.to_bytes());

        let mut mgr = OtaDataManager::load_from_flash(&mut flash).expect("load");
        assert_eq!(mgr.running_slot(), OtaSlot::Ota0);
        // Must select aborted sector 1, NEVER sole valid sector 0.
        assert_eq!(mgr.select_inactive_otadata_sector(), 1);

        let image = make_valid_esp_image(true);
        let slot = mgr
            .write_ota_image_to_flash(&mut flash, &image)
            .expect("post-rollback OTA succeeds");
        assert_eq!(slot, OtaSlot::Ota1);
        assert_eq!(
            mgr.sector_0, s0,
            "sole valid fallback sector 0 must remain intact"
        );
        assert_eq!(
            mgr.sector_1.ota_seq, 4,
            "next_seq must advance above aborted seq 2"
        );
        assert_eq!(mgr.sector_1.state(), OtaImgState::PendingVerify);

        let mut raw0 = [0u8; 32];
        raw0.copy_from_slice(&flash.otadata_s0[..32]);
        assert_eq!(OtaDataSelect::from_bytes(&raw0), s0);
    }

    #[test]
    fn test_m004_pending_reboot_blocks_second_ota_and_premature_cancel() {
        let mut flash: MockNorFlash<8192> = MockNorFlash::new_booted_ota0();
        let mut mgr = OtaDataManager::load_from_flash(&mut flash).expect("load");
        let image = make_valid_esp_image(true);

        let slot = mgr
            .write_ota_image_to_flash(&mut flash, &image)
            .expect("first OTA to Ota1");
        assert_eq!(slot, OtaSlot::Ota1);
        assert_eq!(mgr.running_slot(), OtaSlot::Ota0);
        assert_eq!(mgr.select_inactive_slot(), OtaSlot::Ota1);
        assert_eq!(mgr.pending_reboot_slot(), Some(OtaSlot::Ota1));

        let erases = flash.erase_count;
        // Second OTA before reboot must be blocked (never erases live XIP Ota0).
        assert_eq!(
            mgr.write_ota_image_to_flash(&mut flash, &image),
            Err(OtaError::RollbackStateInvalid)
        );
        assert_eq!(
            mgr.begin_ota(&mut flash, image.len()),
            Err(OtaError::RollbackStateInvalid)
        );
        assert_eq!(flash.erase_count, erases);

        // Premature cancel_rollback_in_flash before reboot must be blocked.
        assert_eq!(
            mgr.cancel_rollback_in_flash(&mut flash),
            Err(OtaError::RollbackStateInvalid)
        );
    }

    #[test]
    fn test_m005_blocks_ota_while_running_slot_pending_verify() {
        let mut flash: MockNorFlash<8192> = MockNorFlash::new();
        flash.otadata_s0[..32]
            .copy_from_slice(&OtaDataSelect::new(1, OtaImgState::Valid).to_bytes());
        flash.otadata_s1[..32]
            .copy_from_slice(&OtaDataSelect::new(2, OtaImgState::PendingVerify).to_bytes());

        let mut mgr = OtaDataManager::load_from_flash(&mut flash).expect("load");
        assert!(mgr.has_pending_rollback());
        let erases = flash.erase_count;
        let image = make_valid_esp_image(true);

        assert_eq!(
            mgr.write_ota_image_to_flash(&mut flash, &image),
            Err(OtaError::RollbackStateInvalid)
        );
        assert_eq!(
            mgr.begin_ota(&mut flash, image.len()),
            Err(OtaError::RollbackStateInvalid)
        );
        assert_eq!(
            mgr.write_chunk(&mut flash, OtaSlot::Ota0, 0, &[0xFF; 4]),
            Err(OtaError::RollbackStateInvalid)
        );
        assert_eq!(
            flash.erase_count, erases,
            "passive rollback slot must not be erased"
        );
    }

    #[test]
    fn test_m019_m034_core1_pause_guard_timeout_cleanup_and_resume_error() {
        use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

        struct FaultPark {
            pause_ok: AtomicBool,
            resume_ok: AtomicBool,
            resumes: AtomicU32,
        }

        impl Core1ParkControl for FaultPark {
            fn pause_core1(&self) -> bool {
                self.pause_ok.load(Ordering::SeqCst)
            }
            fn resume_core1(&self) -> bool {
                self.resumes.fetch_add(1, Ordering::SeqCst);
                self.resume_ok.load(Ordering::SeqCst)
            }
        }

        // M-019: Pause failure MUST NOT call resume_core1() (rollback is
        // pause_core1()'s contract so a failed nested pause never unparks an
        // outer guard).
        let park_timeout = FaultPark {
            pause_ok: AtomicBool::new(false),
            resume_ok: AtomicBool::new(true),
            resumes: AtomicU32::new(0),
        };
        assert!(Core1PauseGuard::new(&park_timeout).is_err());
        assert_eq!(park_timeout.resumes.load(Ordering::SeqCst), 0);

        // Nested failure preserves outer guard depth without unparking Core 1.
        struct DepthPark {
            depth: AtomicU32,
            fail_next_pause: AtomicBool,
        }
        impl Core1ParkControl for DepthPark {
            fn pause_core1(&self) -> bool {
                if self.fail_next_pause.swap(false, Ordering::SeqCst) {
                    return false;
                }
                self.depth.fetch_add(1, Ordering::SeqCst);
                true
            }
            fn resume_core1(&self) -> bool {
                self.depth.fetch_sub(1, Ordering::SeqCst);
                true
            }
        }
        let depth_park = DepthPark {
            depth: AtomicU32::new(0),
            fail_next_pause: AtomicBool::new(false),
        };
        let outer = Core1PauseGuard::new(&depth_park).expect("outer pause");
        assert_eq!(depth_park.depth.load(Ordering::SeqCst), 1);
        depth_park.fail_next_pause.store(true, Ordering::SeqCst);
        assert!(Core1PauseGuard::new(&depth_park).is_err());
        assert_eq!(
            depth_park.depth.load(Ordering::SeqCst),
            1,
            "failed inner guard must NOT decrement outer guard depth"
        );
        outer.resume().expect("outer resume");
        assert_eq!(depth_park.depth.load(Ordering::SeqCst), 0);

        // Resume failure propagates Err(FlashWriteError) via GuardedFlash.
        let park_resume_fail = FaultPark {
            pause_ok: AtomicBool::new(true),
            resume_ok: AtomicBool::new(false),
            resumes: AtomicU32::new(0),
        };
        let mut guarded =
            GuardedFlashStorage::new(MockNorFlash::<8192>::new_booted_ota0(), park_resume_fail);
        let mut buf = AlignedOtaBuf::default();
        assert_eq!(
            guarded.read_aligned(OTADATA_SECTOR_0_OFFSET, &mut buf.0),
            Err(OtaError::FlashWriteError)
        );
        assert_eq!(guarded.park().resumes.load(Ordering::SeqCst), 1);
        let (_flash, _park) = guarded.into_inner();
    }

    #[test]
    fn test_m033_is_seq_greater_strict_weak_ordering_no_cycles() {
        let vals = [0u32, 1, 5, 100, u32::MAX - 5, u32::MAX - 1, u32::MAX];
        for &a in &vals {
            assert!(!is_seq_greater(a, a), "irreflexive failed for {a}");
            for &b in &vals {
                if is_seq_greater(a, b) {
                    assert!(!is_seq_greater(b, a), "asymmetric failed for ({a}, {b})");
                }
                for &c in &vals {
                    if is_seq_greater(a, b) && is_seq_greater(b, c) {
                        assert!(
                            is_seq_greater(a, c),
                            "transitive failed for ({a}, {b}, {c})"
                        );
                    }
                }
            }
        }
        assert!(is_seq_greater(u32::MAX - 5, 100));
        assert!(is_seq_greater(100, 5));
        assert!(is_seq_greater(u32::MAX - 5, 5));
    }

    #[test]
    fn test_m035_ota_write_session_and_bounds_checks() {
        let mut flash: MockNorFlash<8192> = MockNorFlash::new_booted_ota0();
        let mut mgr = OtaDataManager::load_from_flash(&mut flash).expect("load");
        let image = make_valid_esp_image_with_len(128, true);

        // Out-of-partition and active-slot write_chunk rejected.
        assert_eq!(
            mgr.write_chunk(&mut flash, OtaSlot::Ota0, 0, &[0; 4]),
            Err(OtaError::RollbackStateInvalid)
        );
        assert!(matches!(
            mgr.write_chunk(&mut flash, OtaSlot::Ota1, OTA_1_SIZE - 4, &[0; 8]),
            Err(OtaError::ImageTooLarge { .. })
        ));
        assert_eq!(
            mgr.write_chunk(&mut flash, OtaSlot::Ota1, 2, &[0; 4]),
            Err(OtaError::FlashWriteError)
        );

        // Stateful OtaWriteSession streams chunks, verifies readback, commits.
        let mut session = mgr
            .begin_ota_session(&mut flash, image.len())
            .expect("begin session");
        assert_eq!(session.target_slot(), OtaSlot::Ota1);
        session
            .write_chunk(&mut flash, &image[..64])
            .expect("first chunk");
        session
            .write_chunk(&mut flash, &image[64..])
            .expect("final chunk");
        let entry = session
            .finish(&mut mgr, &mut flash)
            .expect("finish verified session");
        assert_eq!(entry.ota_seq, 2);
        assert_eq!(mgr.pending_reboot_slot(), Some(OtaSlot::Ota1));

        // M-035: Direct `finish_ota` rejects a valid 24-byte header + blank or
        // corrupt body without mutating otadata, and accepts a valid full image.
        let mut flash_bad: MockNorFlash<8192> = MockNorFlash::new_booted_ota0();
        let mut mgr_bad = OtaDataManager::load_from_flash(&mut flash_bad).expect("load");
        let otadata_before = (flash_bad.otadata_s0, flash_bad.otadata_s1);
        let slot_bad = mgr_bad
            .begin_ota(&mut flash_bad, image.len())
            .expect("begin_ota");
        // Write only the valid 24-byte header + 0xFF blank body.
        mgr_bad
            .write_chunk(&mut flash_bad, slot_bad, 0, &image[..24])
            .expect("write header only");
        assert!(
            mgr_bad.finish_ota(&mut flash_bad, slot_bad).is_err(),
            "finish_ota must reject header-only/blank body"
        );
        assert_eq!((flash_bad.otadata_s0, flash_bad.otadata_s1), otadata_before);
        assert_eq!(mgr_bad.pending_reboot_slot(), None);

        // Write full image with a flipped payload byte -> SHA-256/XOR mismatch.
        let mut corrupt = image.clone();
        corrupt[32] ^= 0xFF;
        let slot_bad2 = mgr_bad
            .begin_ota(&mut flash_bad, corrupt.len())
            .expect("begin_ota corrupt");
        mgr_bad
            .write_chunk(&mut flash_bad, slot_bad2, 0, &corrupt)
            .expect("write corrupt");
        assert!(
            mgr_bad.finish_ota(&mut flash_bad, slot_bad2).is_err(),
            "finish_ota must reject corrupt body"
        );
        assert_eq!((flash_bad.otadata_s0, flash_bad.otadata_s1), otadata_before);

        // Write intact image -> finish_ota verifies and commits.
        let slot_ok = mgr_bad
            .begin_ota(&mut flash_bad, image.len())
            .expect("begin_ota ok");
        mgr_bad
            .write_chunk(&mut flash_bad, slot_ok, 0, &image)
            .expect("write valid image");
        let committed = mgr_bad
            .finish_ota(&mut flash_bad, slot_ok)
            .expect("finish_ota valid");
        assert_eq!(committed.ota_seq, 2);
        assert_eq!(mgr_bad.pending_reboot_slot(), Some(OtaSlot::Ota1));
    }

    #[test]
    fn test_m036_m037_deferred_ram_mutation_and_blank_sector0_persistence() {
        // 1. Factory-blank flash persists sector_0 before writing sector_1.
        let mut blank_flash: MockNorFlash<8192> = MockNorFlash::new();
        let mut mgr = OtaDataManager::load_from_flash(&mut blank_flash).expect("load blank");
        let image = make_valid_esp_image(true);
        mgr.write_ota_image_to_flash(&mut blank_flash, &image)
            .expect("commit from blank flash");
        let mut raw0 = [0u8; 32];
        raw0.copy_from_slice(&blank_flash.otadata_s0[..32]);
        let s0_flash = OtaDataSelect::from_bytes(&raw0);
        assert_eq!(s0_flash.ota_seq, 1);
        assert_eq!(s0_flash.state(), OtaImgState::Valid);

        // 2. Flash error on commit_update leaves RAM unchanged (M-036).
        let mut fail_flash: MockNorFlash<8192> = MockNorFlash::new_booted_ota0();
        let mut mgr2 = OtaDataManager::load_from_flash(&mut fail_flash).expect("load");
        let snapshot = mgr2;
        fail_flash.fail_writes = true;
        assert!(mgr2.commit_update(&mut fail_flash, OtaSlot::Ota1).is_err());
        assert_eq!(mgr2, snapshot);

        // 3. rollback() rejected on already-Valid slot (M-036).
        fail_flash.fail_writes = false;
        assert_eq!(
            mgr2.rollback(&mut fail_flash),
            Err(OtaError::RollbackStateInvalid)
        );

        // 4. Undefined -> Valid programs in-place with zero erases (M-037).
        let mut undef_flash: MockNorFlash<8192> = MockNorFlash::new();
        undef_flash.otadata_s0[..32]
            .copy_from_slice(&OtaDataSelect::new(1, OtaImgState::Undefined).to_bytes());
        let mut mgr3 = OtaDataManager::load_from_flash(&mut undef_flash).expect("load undef");
        let erases_before = undef_flash.erase_count;
        assert_eq!(mgr3.cancel_rollback_in_flash(&mut undef_flash), Ok(true));
        assert_eq!(undef_flash.erase_count, erases_before);
        let mut raw_u = [0u8; 32];
        raw_u.copy_from_slice(&undef_flash.otadata_s0[..32]);
        assert_eq!(
            OtaDataSelect::from_bytes(&raw_u).state(),
            OtaImgState::Valid
        );
    }

    #[test]
    fn test_m037_atomic_sole_sector_cancel_and_dual_equal_valid_recovery() {
        // 1. Sole bootable sector `(2, PendingVerify)` in sector 1 with sector 0
        // erased: `cancel_rollback_in_flash` writes `(4, Valid)` atomically to
        // sector 0 without erasing sector 1.
        let mut flash: MockNorFlash<8192> = MockNorFlash::new();
        let orig_s1 = OtaDataSelect::new(2, OtaImgState::PendingVerify);
        flash.otadata_s1[..32].copy_from_slice(&orig_s1.to_bytes());
        let mut mgr = OtaDataManager::load_from_flash(&mut flash).expect("load sole s1");
        let erases_before = flash.erase_count;
        assert_eq!(mgr.cancel_rollback_in_flash(&mut flash), Ok(true));
        // Only sector 0 was erased/written; sector 1 was never erased.
        assert_eq!(flash.erase_count, erases_before + 1);
        let mut raw0 = [0u8; 32];
        let mut raw1 = [0u8; 32];
        raw0.copy_from_slice(&flash.otadata_s0[..32]);
        raw1.copy_from_slice(&flash.otadata_s1[..32]);
        let s0_after = OtaDataSelect::from_bytes(&raw0);
        let s1_after = OtaDataSelect::from_bytes(&raw1);
        assert_eq!(s0_after.ota_seq, 4);
        assert_eq!(s0_after.state(), OtaImgState::Valid);
        assert_eq!(s1_after, orig_s1, "sole active sector 1 must not be erased");
        assert!(!mgr.has_pending_rollback());
        assert_eq!(mgr.active_slot(), OtaSlot::Ota1);
        // Second cancel is idempotent with zero writes.
        let writes_before = flash.write_count;
        assert_eq!(mgr.cancel_rollback_in_flash(&mut flash), Ok(false));
        assert_eq!(flash.write_count, writes_before);

        // 2. Dual-equal `(ota_seq, Valid)` sectors matching `running_slot`
        // (e.g. power loss during fallback cancel near u32::MAX) must NOT
        // permanently lock out OTA.
        let mut eq_flash: MockNorFlash<8192> = MockNorFlash::new();
        let dup = OtaDataSelect::new(5, OtaImgState::Valid).to_bytes();
        eq_flash.otadata_s0[..32].copy_from_slice(&dup);
        eq_flash.otadata_s1[..32].copy_from_slice(&dup);
        let mut eq_mgr = OtaDataManager::load_from_flash(&mut eq_flash).expect("load dual equal");
        assert_eq!(eq_mgr.running_slot(), OtaSlot::Ota0);
        let image = make_valid_esp_image(true);
        let next_slot = eq_mgr
            .write_ota_image_to_flash(&mut eq_flash, &image)
            .expect("OTA must recover from dual-equal (seq, Valid) sectors");
        assert_eq!(next_slot, OtaSlot::Ota1);
        assert_eq!(eq_mgr.active_slot(), OtaSlot::Ota1);
        assert_eq!(eq_mgr.running_slot(), OtaSlot::Ota0);
    }

    #[test]
    fn test_m038_torn_inactive_sector_recovery() {
        // 1. Torn write with erased CRC (0xFFFF_FFFF).
        let mut flash: MockNorFlash<8192> = MockNorFlash::new_booted_ota0();
        let partial = OtaDataSelect::new(2, OtaImgState::PendingVerify).to_bytes();
        flash.otadata_s1[..28].copy_from_slice(&partial[..28]);

        let mut mgr = OtaDataManager::load_from_flash(&mut flash).expect("load");
        let image = make_valid_esp_image(true);
        let slot = mgr
            .write_ota_image_to_flash(&mut flash, &image)
            .expect("OTA must recover from torn inactive sector");
        assert_eq!(slot, OtaSlot::Ota1);
        let mut raw1 = [0u8; 32];
        raw1.copy_from_slice(&flash.otadata_s1[..32]);
        let committed = OtaDataSelect::from_bytes(&raw1);
        assert!(committed.is_crc_valid());
        assert_eq!(committed.ota_seq, 2);

        // 2. Brownout during Stage-2 CRC programming (`crc = 0xFFFF_FF9A`,
        // `ota_state = 1`) in inactive sector while active sector is valid &
        // confirmed matching `running_slot` must also recover cleanly.
        let mut flash_crc: MockNorFlash<8192> = MockNorFlash::new_booted_ota0();
        let mut partial_crc = OtaDataSelect::new(2, OtaImgState::PendingVerify).to_bytes();
        partial_crc[28..32].copy_from_slice(&0xFFFF_FF9Au32.to_le_bytes());
        flash_crc.otadata_s1[..32].copy_from_slice(&partial_crc);

        let mut mgr_crc = OtaDataManager::load_from_flash(&mut flash_crc).expect("load");
        let slot_crc = mgr_crc
            .write_ota_image_to_flash(&mut flash_crc, &image)
            .expect("OTA must recover from partial-CRC corrupt inactive sector");
        assert_eq!(slot_crc, OtaSlot::Ota1);

        // 3. When NEITHER sector is bootable (both corrupt), OTA refuses with
        // InvalidOtaDataCrc without erasing flash.
        let mut flash_both_bad: MockNorFlash<8192> = MockNorFlash::new();
        flash_both_bad.otadata_s0[..32].copy_from_slice(&partial_crc);
        flash_both_bad.otadata_s1[..32].copy_from_slice(&partial_crc);
        let mut mgr_bad = OtaDataManager::new_booted_ota0();
        let erases_before = flash_both_bad.erase_count;
        assert_eq!(
            mgr_bad.write_ota_image_to_flash(&mut flash_both_bad, &image),
            Err(OtaError::InvalidOtaDataCrc)
        );
        assert_eq!(flash_both_bad.erase_count, erases_before);
    }

    #[test]
    fn test_m136_m137_otadata_try_new_and_storage_parity() {
        assert_eq!(OtaDataSelect::try_new(0, OtaImgState::Valid), None);
        assert_eq!(OtaDataSelect::try_new(u32::MAX, OtaImgState::Valid), None);
        assert!(OtaDataSelect::try_new(1, OtaImgState::Valid).is_some());
        assert_eq!(
            OtaDataManager::load_from_flash(&mut NoOpFlashStorage),
            Err(OtaError::FlashWriteError)
        );
    }
}
