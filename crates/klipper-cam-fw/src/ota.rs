//! Partition layout, constant-time OTA token verification, ESP32 firmware image
//! header validation (`0xE9` magic), and `otadata` IEEE 802.3 CRC32 rollback state machine.
//!
//! Replaces the OTA subsystem in `examples/camera_example/main/klipper_cam.c`,
//! `examples/camera_example/main/ota_minimal.c`, and `examples/camera_example/partitions.csv`.

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

/// Mandatory magic byte at offset `0` of every valid ESP32 application image (`esp_image_header_t`).
pub const ESP_IMAGE_HEADER_MAGIC: u8 = 0xE9;

/// Minimum valid segment count in an ESP32 application image header (`1`).
pub const ESP_IMAGE_MIN_SEGMENTS: u8 = 1;

/// Maximum valid segment count in an ESP32 application image header (`16`).
pub const ESP_IMAGE_MAX_SEGMENTS: u8 = 16;

/// Size in bytes of the fixed `esp_image_header_t` prefix (`24` bytes).
pub const ESP_IMAGE_HEADER_LEN: usize = 24;

/// Maximum chunk size read per socket receive step during OTA (`1024` bytes, `klipper_cam.c`).
pub const OTA_CHUNK_SIZE: usize = 1024;

/// Maximum consecutive socket timeout retries during OTA upload (`5`, `APP-007`).
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

    /// Returns the passive target slot to write during an OTA update.
    #[inline]
    pub const fn next_slot(self) -> Self {
        match self {
            Self::Ota0 => Self::Ota1,
            Self::Ota1 => Self::Ota0,
        }
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
    /// `(ota_seq - 1) % num_ota_partitions` selects the slot (`1, 3, 5 -> ota_0`; `2, 4, 6 -> ota_1`).
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

/// Static flash partition descriptor matching `examples/camera_example/partitions.csv`.
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

/// Complete 5-entry partition table matching `examples/camera_example/partitions.csv`:
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

/// Verifies that all entries in [`PARTITION_TABLE`] are 4KB-aligned, strictly ordered,
/// non-overlapping, and fit within the 4MB SPI flash (`0x400000`).
pub const fn verify_partition_table_integrity() -> bool {
    let mut i = 0;
    while i < PARTITION_TABLE.len() {
        let entry = PARTITION_TABLE[i];
        if (entry.offset & 0xFFF) != 0 || (entry.size & 0xFFF) != 0 || entry.size == 0 {
            return false;
        }
        let end = entry.offset.saturating_add(entry.size);
        if end > FLASH_TOTAL_SIZE {
            return false;
        }
        if i + 1 < PARTITION_TABLE.len() {
            let next = PARTITION_TABLE[i + 1];
            if end > next.offset {
                return false;
            }
        }
        i += 1;
    }
    true
}

/// Constant-time OTA authentication token comparator (`FW-010` / `APP-002`).
///
/// Uses a bitwise XOR accumulator across all bytes of `expected` without branching
/// early on byte mismatches or length mismatches, preventing timing side-channel
/// recovery of `X-OTA-Token` over Wi-Fi.
///
/// Rejects empty `provided` or empty `expected` tokens unconditionally.
#[inline]
pub const fn verify_ota_token(provided: &[u8], expected: &[u8]) -> bool {
    if expected.is_empty() {
        return false;
    }
    let mut diff: usize = provided.len() ^ expected.len();
    let mut i: usize = 0;
    while i < expected.len() {
        let p_byte = if i < provided.len() { provided[i] } else { 0 };
        diff |= (p_byte ^ expected[i]) as usize;
        i += 1;
    }
    diff == 0 && !provided.is_empty()
}

/// Computes the standard IEEE 802.3 CRC-32 checksum over `data`
/// (polynomial `0xEDB88320`, initial value `0xFFFFFFFF`, final XOR `0xFFFFFFFF`).
pub const fn crc32_ieee(data: &[u8]) -> u32 {
    let mut crc: u32 = 0xFFFF_FFFF;
    let mut i: usize = 0;
    while i < data.len() {
        crc ^= data[i] as u32;
        let mut bit: u8 = 0;
        while bit < 8 {
            if (crc & 1) != 0 {
                crc = (crc >> 1) ^ 0xEDB8_8320;
            } else {
                crc >>= 1;
            }
            bit += 1;
        }
        i += 1;
    }
    !crc
}

/// Computes the ESP-IDF / ESP32 ROM `esp_rom_crc32_le(u32::MAX, data)` checksum used by
/// `bootloader_common_ota_select_crc` (`init = 0x0000_0000`, reflected polynomial `0xEDB8_8320`,
/// final XOR `0xFFFF_FFFF`).
pub const fn crc32_esp_ota(data: &[u8]) -> u32 {
    let mut crc: u32 = 0x0000_0000;
    let mut i: usize = 0;
    while i < data.len() {
        crc ^= data[i] as u32;
        let mut bit: u8 = 0;
        while bit < 8 {
            if (crc & 1) != 0 {
                crc = (crc >> 1) ^ 0xEDB8_8320;
            } else {
                crc >>= 1;
            }
            bit += 1;
        }
        i += 1;
    }
    !crc
}

/// OTA image lifecycle states stored in `otadata` (`esp_ota_img_states_t`).
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum OtaImgState {
    /// Monitor the first boot; bootloader transitions this to `PendingVerify` (`0x0`).
    New = 0x0,
    /// First boot after OTA; requires `mark_app_valid_cancel_rollback()` before next reboot (`0x1`).
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

/// 32-byte `otadata` sector selection record (`esp_ota_select_entry_t`, `EMB-003`).
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct OtaDataSelect {
    /// Monotonically increasing sequence counter (`1..u32::MAX - 1`).
    pub ota_seq: u32,
    /// Reserved 20-byte label/padding array (`0xFF` filled).
    pub seq_label: [u8; 20],
    /// Raw [`OtaImgState`] value.
    pub ota_state: u32,
    /// IEEE 802.3 CRC-32 of `ota_seq` in little-endian byte order.
    pub crc: u32,
}

impl OtaDataSelect {
    /// Size in bytes of an [`OtaDataSelect`] structure (`32` bytes).
    pub const SIZE: usize = 32;

    /// Returns an erased (`0xFF`-filled) [`OtaDataSelect`] matching blank NOR flash.
    #[inline]
    pub const fn erased() -> Self {
        Self {
            ota_seq: u32::MAX,
            seq_label: [0xFF; 20],
            ota_state: OtaImgState::Undefined as u32,
            crc: u32::MAX,
        }
    }

    /// Creates a new [`OtaDataSelect`] with `ota_seq`, `state`, and valid ESP-IDF `otadata` CRC-32.
    #[inline]
    pub const fn new(ota_seq: u32, state: OtaImgState) -> Self {
        let crc = crc32_esp_ota(&ota_seq.to_le_bytes());
        Self {
            ota_seq,
            seq_label: [0xFF; 20],
            ota_state: state as u32,
            crc,
        }
    }

    /// Calculates the ESP-IDF `bootloader_common_ota_select_crc` over the little-endian representation of `ota_seq`.
    #[inline]
    pub const fn calculate_crc(&self) -> u32 {
        crc32_esp_ota(&self.ota_seq.to_le_bytes())
    }

    /// Returns `true` if `ota_seq` is initialized (`!= 0` and `!= u32::MAX`) and `crc` matches.
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
        !matches!(self.state(), OtaImgState::Invalid | OtaImgState::Aborted)
    }

    /// Returns the [`OtaSlot`] selected by `self.ota_seq` if valid.
    #[inline]
    pub const fn slot(&self) -> Option<OtaSlot> {
        if !self.is_crc_valid() {
            return None;
        }
        OtaSlot::from_seq(self.ota_seq)
    }

    /// Marks this `otadata` record as [`OtaImgState::Valid`] (`esp_ota_mark_app_valid_cancel_rollback`, `EMB-003`).
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

    /// Serializes this record into a 32-byte little-endian array.
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

        let flash_state = if self.ota_state == OtaImgState::PendingVerify as u32 {
            OtaImgState::New as u32
        } else {
            self.ota_state
        };
        let st_b = flash_state.to_le_bytes();
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

    /// Deserializes a 32-byte little-endian array into an [`OtaDataSelect`].
    pub const fn from_bytes(bytes: &[u8; 32]) -> Self {
        let ota_seq = u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
        let mut seq_label = [0u8; 20];
        let mut i = 0;
        while i < 20 {
            seq_label[i] = bytes[4 + i];
            i += 1;
        }
        let raw_state = u32::from_le_bytes([bytes[24], bytes[25], bytes[26], bytes[27]]);
        let ota_state = if ota_seq != u32::MAX && raw_state == OtaImgState::New as u32 {
            OtaImgState::PendingVerify as u32
        } else {
            raw_state
        };
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

/// 4-byte-aligned 32-byte buffer for `otadata` NOR flash reads and writes (`esp-storage` alignment requirement).
#[repr(C, align(4))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AlignedOtaBuf(pub [u8; 32]);

impl Default for AlignedOtaBuf {
    #[inline]
    fn default() -> Self {
        Self([0xFF; 32])
    }
}

/// 4-byte-aligned 1024-byte chunk buffer for OTA application partition flash writes (`esp-storage` alignment requirement).
#[repr(C, align(4))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AlignedFlashChunk(pub [u8; OTA_CHUNK_SIZE]);

impl Default for AlignedFlashChunk {
    #[inline]
    fn default() -> Self {
        Self([0xFF; OTA_CHUNK_SIZE])
    }
}

/// Hardware abstraction trait for SPI NOR flash `otadata` and OTA partition reads, erases, and writes.
///
/// Designed to wrap `esp_storage::FlashStorage` on Xtensa targets while enabling
/// deterministic `#![no_std]` mock NOR flash verification on the host.
pub trait OtaFlashStorage {
    /// Reads `buf.len()` bytes starting at `offset` (`offset` and `buf.len()` must be 4-byte aligned).
    fn read_aligned(&mut self, offset: u32, buf: &mut [u8]) -> Result<(), OtaError>;

    /// Erases the flash range `[from, to)` (`from` and `to` must be 4096-byte sector aligned).
    fn erase_range(&mut self, from: u32, to: u32) -> Result<(), OtaError>;

    /// Writes `data` starting at `offset` (`offset` and `data.len()` must be 4-byte aligned).
    fn write_aligned(&mut self, offset: u32, data: &[u8]) -> Result<(), OtaError>;
}

/// No-op [`OtaFlashStorage`] implementation for in-memory HTTP/OTA state machine operation.
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
}

/// Deterministic `#![no_std]` SPI NOR flash simulator covering `otadata` (`0x17000..0x19000`)
/// and the leading `APP_WINDOW` bytes of `ota_0` (`0x20000`) and `ota_1` (`0x1D0000`).
///
/// Enforces hardware NOR flash invariants:
/// - `read_aligned` and `write_aligned` require 4-byte aligned `offset` and buffer length.
/// - `erase_range` requires 4096-byte (`0x1000`) sector-aligned `from` and `to` with `from < to`.
/// - `erase_range` resets bytes to `0xFF`, and `write_aligned` programs bits via `dst &= src`.
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

    /// Creates a [`MockNorFlash`] initialized with `ota_0` active (`ota_seq = 1`, `Valid`) in `otadata` sector 0.
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
        let end = offset
            .checked_add(len as u32)
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

/// Dual-sector `otadata` state machine (`0x17000` and `0x18000`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OtaDataManager {
    pub sector_0: OtaDataSelect,
    pub sector_1: OtaDataSelect,
}

impl OtaDataManager {
    /// Creates an [`OtaDataManager`] booted into `ota_0` (`ota_seq = 1`, `Valid`).
    #[inline]
    pub const fn new_booted_ota0() -> Self {
        Self {
            sector_0: OtaDataSelect::new(1, OtaImgState::Valid),
            sector_1: OtaDataSelect::erased(),
        }
    }

    /// Creates an [`OtaDataManager`] from two raw `otadata` sector entries.
    #[inline]
    pub const fn from_sectors(sector_0: OtaDataSelect, sector_1: OtaDataSelect) -> Self {
        Self { sector_0, sector_1 }
    }

    /// Reads both `otadata` sectors (`0x17000` and `0x18000`) from `flash` using 4-byte-aligned
    /// [`AlignedOtaBuf`] buffers.
    ///
    /// If neither sector in flash contains a valid CRC-32 (e.g. blank factory flash),
    /// initializes to [`Self::new_booted_ota0`].
    pub fn load_from_flash<F: OtaFlashStorage>(flash: &mut F) -> Result<Self, OtaError> {
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

    /// Erases the 4KB `otadata` sector corresponding to `slot` (`0x17000` for `Ota0`,
    /// `0x18000` for `Ota1`) and writes its 32-byte [`OtaDataSelect`] entry to `flash`.
    pub fn sync_slot_to_flash<F: OtaFlashStorage>(
        &self,
        flash: &mut F,
        slot: OtaSlot,
    ) -> Result<(), OtaError> {
        let (sector_offset, entry) = match slot {
            OtaSlot::Ota0 => (OTADATA_SECTOR_0_OFFSET, &self.sector_0),
            OtaSlot::Ota1 => (OTADATA_SECTOR_1_OFFSET, &self.sector_1),
        };
        flash.erase_range(sector_offset, sector_offset + FLASH_SECTOR_SIZE)?;
        let aligned = AlignedOtaBuf(entry.to_bytes());
        flash.write_aligned(sector_offset, &aligned.0)
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

    /// Marks the currently active OTA slot as [`OtaImgState::Valid`] (`EMB-003`) and,
    /// if the state transitioned from `PendingVerify`/`New`, persists the updated `otadata`
    /// sector to `flash`.
    pub fn cancel_rollback_in_flash<F: OtaFlashStorage>(
        &mut self,
        flash: &mut F,
    ) -> Result<bool, OtaError> {
        let s0_ok = self.sector_0.is_bootable();
        let s1_ok = self.sector_1.is_bootable();
        let (mutated_slot, changed) = match (s0_ok, s1_ok) {
            (true, true) => {
                if self.sector_0.ota_seq >= self.sector_1.ota_seq {
                    (OtaSlot::Ota0, self.sector_0.mark_app_valid_cancel_rollback()?)
                } else {
                    (OtaSlot::Ota1, self.sector_1.mark_app_valid_cancel_rollback()?)
                }
            }
            (true, false) => (OtaSlot::Ota0, self.sector_0.mark_app_valid_cancel_rollback()?),
            (false, true) => (OtaSlot::Ota1, self.sector_1.mark_app_valid_cancel_rollback()?),
            (false, false) => return Err(OtaError::InvalidOtaDataCrc),
        };
        if changed {
            self.sync_slot_to_flash(flash, mutated_slot)?;
        }
        Ok(changed)
    }

    /// Validates an ESP32 OTA firmware `payload`, erases the 4KB-rounded range in the passive
    /// [`OtaSlot`] partition, writes the image in 1024-byte 4-byte-aligned chunks (padding any
    /// trailing `1..=3` bytes with `0xFF`), updates `otadata` to [`OtaImgState::PendingVerify`],
    /// and syncs the updated `otadata` sector to `flash`.
    pub fn write_ota_image_to_flash<F: OtaFlashStorage>(
        &mut self,
        flash: &mut F,
        payload: &[u8],
    ) -> Result<OtaSlot, OtaError> {
        let target_slot = self.next_update_slot();
        let partition = target_slot.partition();
        let max_size = if (partition.size as usize) < OTA_PARTITION_MAX_SIZE {
            partition.size as usize
        } else {
            OTA_PARTITION_MAX_SIZE
        };

        validate_ota_image_header(payload, payload.len(), max_size)?;
        validate_ota_image_integrity(payload)?;

        let erase_len = ((payload.len() as u32) + 0xFFF) & !0xFFF;
        flash.erase_range(
            partition.offset,
            partition.offset.saturating_add(erase_len),
        )?;

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
            flash.write_aligned(
                partition.offset.saturating_add(received as u32),
                &chunk.0[..aligned_step],
            )?;
            received += step;
        }

        self.set_boot_slot(target_slot)?;
        self.sync_slot_to_flash(flash, target_slot)?;
        Ok(target_slot)
    }

    /// Returns the currently active [`OtaSlot`] according to ESP-IDF bootloader selection rules.
    ///
    /// If neither sector contains a valid bootable record, defaults to [`OtaSlot::Ota0`] (factory fallback).
    pub const fn active_slot(&self) -> OtaSlot {
        let s0_ok = self.sector_0.is_bootable();
        let s1_ok = self.sector_1.is_bootable();
        let winning_seq = match (s0_ok, s1_ok) {
            (true, true) => {
                if self.sector_0.ota_seq >= self.sector_1.ota_seq {
                    self.sector_0.ota_seq
                } else {
                    self.sector_1.ota_seq
                }
            }
            (true, false) => self.sector_0.ota_seq,
            (false, true) => self.sector_1.ota_seq,
            (false, false) => return OtaSlot::Ota0,
        };
        match OtaSlot::from_seq(winning_seq) {
            Some(slot) => slot,
            None => OtaSlot::Ota0,
        }
    }

    /// Returns the passive [`OtaSlot`] targeted by the next OTA update.
    #[inline]
    pub const fn next_update_slot(&self) -> OtaSlot {
        self.active_slot().next_slot()
    }

    /// Updates `otadata` to boot into `target_slot` on the next restart, setting its state
    /// to [`OtaImgState::PendingVerify`] with a newly calculated IEEE 802.3 CRC-32.
    pub fn set_boot_slot(&mut self, target_slot: OtaSlot) -> Result<OtaDataSelect, OtaError> {
        let current_max = match (self.sector_0.is_crc_valid(), self.sector_1.is_crc_valid()) {
            (true, true) => {
                if self.sector_0.ota_seq >= self.sector_1.ota_seq {
                    self.sector_0.ota_seq
                } else {
                    self.sector_1.ota_seq
                }
            }
            (true, false) => self.sector_0.ota_seq,
            (false, true) => self.sector_1.ota_seq,
            (false, false) => 0,
        };

        let next_seq = if current_max >= u32::MAX - 3 {
            match target_slot {
                OtaSlot::Ota0 => 1,
                OtaSlot::Ota1 => 2,
            }
        } else {
            let mut seq = current_max.checked_add(1).ok_or(OtaError::SequenceOverflow)?;
            if OtaSlot::from_seq(seq) != Some(target_slot) {
                seq = seq.checked_add(1).ok_or(OtaError::SequenceOverflow)?;
            }
            seq
        };

        let entry = OtaDataSelect::new(next_seq, OtaImgState::PendingVerify);
        match target_slot {
            OtaSlot::Ota0 => self.sector_0 = entry,
            OtaSlot::Ota1 => self.sector_1 = entry,
        }
        Ok(entry)
    }

    /// Cancels rollback on the currently running OTA slot (`esp_ota_mark_app_valid_cancel_rollback`, `EMB-003`).
    pub fn mark_app_valid_cancel_rollback(&mut self) -> Result<bool, OtaError> {
        let s0_ok = self.sector_0.is_bootable();
        let s1_ok = self.sector_1.is_bootable();
        match (s0_ok, s1_ok) {
            (true, true) => {
                if self.sector_0.ota_seq >= self.sector_1.ota_seq {
                    self.sector_0.mark_app_valid_cancel_rollback()
                } else {
                    self.sector_1.mark_app_valid_cancel_rollback()
                }
            }
            (true, false) => self.sector_0.mark_app_valid_cancel_rollback(),
            (false, true) => self.sector_1.mark_app_valid_cancel_rollback(),
            (false, false) => Err(OtaError::InvalidOtaDataCrc),
        }
    }
}

impl Default for OtaDataManager {
    #[inline]
    fn default() -> Self {
        Self::new_booted_ota0()
    }
}

/// Parsed ESP32 firmware image header (`esp_image_header_t`, `OTA-002` / `APP-008`).
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
    /// Constructs a canonical valid 24-byte ESP32 application header buffer for testing or staging.
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

    /// Constructs a valid 64-byte single-segment ESP32 application image with valid XOR checksum.
    pub const fn minimal_valid_image_64() -> [u8; 64] {
        let mut img = [0u8; 64];
        img[0] = ESP_IMAGE_HEADER_MAGIC;
        img[1] = 1;
        img[2] = 0x02; // DIO
        img[3] = 0x20; // 4MB, 40MHz
        img[4] = 0x00;
        img[5] = 0x04;
        img[6] = 0x08;
        img[7] = 0x40; // Entry point 0x4008_0400
        // Segment 0 header
        img[24] = 0x00;
        img[25] = 0x04;
        img[26] = 0x08;
        img[27] = 0x40; // Load address 0x4008_0400
        img[28] = 16;
        img[29] = 0;
        img[30] = 0;
        img[31] = 0; // Length 16 bytes
        let mut chk = 0xEFu8;
        let mut i = 0usize;
        while i < 16 {
            let val = (i as u8) ^ 0x5A;
            img[32 + i] = val;
            chk ^= val;
            i += 1;
        }
        img[63] = chk;
        img
    }
}

/// Validates an ESP32 OTA image prefix and total `Content-Length` against target partition bounds (`OTA-002`, `APP-008`).
///
/// Checks:
/// 1. `total_image_len >= ESP_IMAGE_HEADER_LEN` (`24`) and `image_prefix.len() >= ESP_IMAGE_HEADER_LEN` (`24`)
/// 2. `total_image_len <= max_partition_size` (`0x1B0000`)
/// 3. `image_prefix[0] == ESP_IMAGE_HEADER_MAGIC` (`0xE9`)
/// 4. `segment_count` in `1..=16` (`ESP_IMAGE_MIN_SEGMENTS..=ESP_IMAGE_MAX_SEGMENTS`)
/// 5. `entry_addr` in ESP32 executable memory regions (`0x4008_0000..0x4080_0000` or `0x3F40_0000..0x3F80_0000`)
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
    let spi_speed_size = image_prefix[3];
    let entry_addr = u32::from_le_bytes([
        image_prefix[4],
        image_prefix[5],
        image_prefix[6],
        image_prefix[7],
    ]);

    let is_valid_entry = (0x4008_0000..0x4080_0000).contains(&entry_addr)
        || (0x3F40_0000..0x3F80_0000).contains(&entry_addr);
    if !is_valid_entry {
        return Err(OtaError::InvalidEntryAddr(entry_addr));
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
                0x6a09_e667, 0xbb67_ae85, 0x3c6e_f372, 0xa54f_f53a,
                0x510e_527f, 0x9b05_688c, 0x1f83_d9ab, 0x5be0_cd19,
            ],
            buffer: [0u8; 64],
            buf_len: 0,
            total_len: 0,
        }
    }

    /// Appends `data` bytes to the hash state.
    pub fn update(&mut self, data: &[u8]) {
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
    pub fn finalize(mut self) -> [u8; 32] {
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

    fn process_block(&mut self) {
        const K: [u32; 64] = [
            0x428a_2f98, 0x7137_4491, 0xb5c0_fbcf, 0xe9b5_dba5,
            0x3956_c25b, 0x59f1_11f1, 0x923f_82a4, 0xab1c_5ed5,
            0xd807_aa98, 0x1283_5b01, 0x2431_85be, 0x550c_7dc3,
            0x72be_5d74, 0x80de_b1fe, 0x9bdc_06a7, 0xc19b_f174,
            0xe49b_69c1, 0xefbe_4786, 0x0fc1_9dc6, 0x240c_a1cc,
            0x2de9_2c6f, 0x4a74_84aa, 0x5cb0_a9dc, 0x76f9_88da,
            0x983e_5152, 0xa831_c66d, 0xb003_27c8, 0xbf59_7fc7,
            0xc6e0_0bf3, 0xd5a7_9147, 0x06ca_6351, 0x1429_2967,
            0x27b7_0a85, 0x2e1b_2138, 0x4d2c_6dfc, 0x5338_0d13,
            0x650a_7354, 0x766a_0abb, 0x81c2_c92e, 0x9272_2c85,
            0xa2bf_e8a1, 0xa81a_664b, 0xc24b_8b70, 0xc76c_51a3,
            0xd192_e819, 0xd699_0624, 0xf40e_3585, 0x106a_a070,
            0x19a4_c116, 0x1e37_6c08, 0x2748_774c, 0x34b0_bcb5,
            0x391c_0cb3, 0x4ed8_aa4a, 0x5b9c_ca4f, 0x682e_6ff3,
            0x748f_82ee, 0x78a5_636f, 0x84c8_7814, 0x8cc7_0208,
            0x90be_fffa, 0xa450_6ceb, 0xbef9_a3f7, 0xc671_78f2,
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

/// Computes the 32-byte SHA-256 digest of `data` (`#![no_std]`, zero allocation).
pub fn sha256_digest(data: &[u8]) -> [u8; 32] {
    let mut h = Sha256::new();
    h.update(data);
    h.finalize()
}

/// Validates the full integrity of an ESP32 firmware application image.
///
/// 1. Validates the 24-byte header via [`validate_ota_image_header`].
/// 2. Iterates over each of the `segment_count` segments starting at offset 24,
///    verifying that each segment's header (8 bytes) and payload lie within `image`.
/// 3. Computes the ESP-IDF 8-bit XOR checksum (seeded with `0xEF`) over all segment payloads
///    and verifies the byte at 16-byte boundary `((offset + 15) & !15) | 15`.
/// 4. If `image[23] == 1` (`hash_appended`), validates the trailing 32-byte SHA-256 digest
///    over `image[..image.len() - 32]`.
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
        let _load_addr = u32::from_le_bytes([
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
        offset += 8;

        let end_seg = offset.checked_add(data_len).ok_or(OtaError::ImageTooSmall)?;
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

    let checksum_offset = offset | 15;
    if image.len() <= checksum_offset {
        return Err(OtaError::ImageTooSmall);
    }

    if image[checksum_offset] != chk {
        return Err(OtaError::InvalidChecksum);
    }

    let hash_appended = image[23] == 1;
    if hash_appended {
        let min_required = checksum_offset.checked_add(33).ok_or(OtaError::ImageTooSmall)?;
        if image.len() < min_required {
            return Err(OtaError::ImageTooSmall);
        }
        let expected_hash = &image[image.len() - 32..];
        let computed_hash = sha256_digest(&image[..image.len() - 32]);
        if computed_hash != expected_hash {
            return Err(OtaError::InvalidSha256);
        }
    }

    Ok(EspImageHeaderSummary {
        header,
        segment_count: header.segment_count,
        entry_addr: header.entry_addr,
        total_segment_len: total_seg_len,
        checksum: chk,
        hash_appended,
    })
}

/// Errors returned by the OTA authentication, header validation, and `otadata` state machine.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum OtaError {
    /// Missing or invalid `X-OTA-Token` header (`401 Unauthorized`).
    Unauthorized,
    /// No passive OTA partition available (`500 Internal Server Error`).
    NoPassivePartition,
    /// Missing or non-positive `Content-Length` header (`400 Bad Request`).
    MissingContentLength,
    /// Image size exceeds target OTA partition capacity (`400 Bad Request`, `APP-008`).
    ImageTooLarge {
        image_size: usize,
        partition_size: usize,
    },
    /// Firmware payload is smaller than minimal header or segment length (`400 Bad Request`).
    ImageTooSmall,
    /// Firmware payload is empty or too short to inspect header (`400 Bad Request`).
    HeaderTooShort,
    /// First byte of firmware image is not `0xE9` (`400 Bad Request`, `OTA-002`).
    InvalidMagicByte(u8),
    /// Segment count in ESP32 image header is outside `1..=16` (`400 Bad Request`, `OTA-002`).
    InvalidSegmentCount(u8),
    /// Entry address outside ESP32 executable memory regions (`400 Bad Request`).
    InvalidEntryAddr(u32),
    /// ESP-IDF 8-bit XOR checksum mismatch (`400 Bad Request`).
    InvalidChecksum,
    /// Appended SHA-256 digest mismatch (`400 Bad Request`).
    InvalidSha256,
    /// Number of received bytes did not match `Content-Length` (`400 Bad Request`).
    IncompletePayload { received: usize, expected: usize },
    /// Socket read error or retry limit exceeded (`500 Internal Server Error`, `APP-007`).
    SocketReadTimeout,
    /// Underlying flash erase/write failure (`500 Internal Server Error`).
    FlashWriteError,
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
                write!(f, "Firmware image validation failed: invalid magic 0x{:02X}", b)
            }
            Self::InvalidSegmentCount(c) => {
                write!(f, "Firmware image validation failed: invalid segments {}", c)
            }
            Self::InvalidEntryAddr(addr) => {
                write!(f, "Firmware image validation failed: invalid entry 0x{:08X}", addr)
            }
            Self::InvalidChecksum => write!(f, "Firmware image validation failed: checksum mismatch"),
            Self::InvalidSha256 => write!(f, "Firmware image validation failed: SHA-256 mismatch"),
            Self::IncompletePayload { received, expected } => write!(
                f,
                "OTA socket read error: received {} of {} bytes",
                received, expected
            ),
            Self::SocketReadTimeout => write!(f, "OTA socket read error or timeout"),
            Self::FlashWriteError => write!(f, "esp_ota_write failed"),
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

        // Round-trip serialization and ESP-IDF bootloader_common_ota_select_crc parity
        let raw = s0.to_bytes();
        assert_eq!(&raw[24..28], &[0, 0, 0, 0]);
        assert_eq!(&raw[28..32], &[154, 152, 67, 71]);
        let s1_raw = OtaDataSelect::new(2, OtaImgState::PendingVerify).to_bytes();
        assert_eq!(&s1_raw[24..32], &[0, 0, 0, 0, 116, 55, 246, 85]);
        let decoded = OtaDataSelect::from_bytes(&raw);
        assert_eq!(decoded, s0);

        // Rollback cancellation transition: PendingVerify -> Valid
        let res = s0.mark_app_valid_cancel_rollback();
        assert_eq!(res, Ok(true));
        assert_eq!(s0.state(), OtaImgState::Valid);
        assert!(s0.is_crc_valid());

        // Second call is idempotent
        assert_eq!(s0.mark_app_valid_cancel_rollback(), Ok(false));
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
        assert_eq!(mgr.next_update_slot(), OtaSlot::Ota0);
        assert_eq!(mgr.sector_1.state(), OtaImgState::PendingVerify);

        // Confirm valid after Wi-Fi GOT_IP
        assert_eq!(mgr.mark_app_valid_cancel_rollback(), Ok(true));
        assert_eq!(mgr.sector_1.state(), OtaImgState::Valid);

        // Next OTA back to ota_0
        let e2 = mgr.set_boot_slot(OtaSlot::Ota0);
        assert!(e2.is_ok());
        assert_eq!(mgr.active_slot(), OtaSlot::Ota0);
        assert_eq!(mgr.sector_0.ota_seq, 3);
        assert_eq!(mgr.sector_0.state(), OtaImgState::PendingVerify);
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
            validate_ota_image_header(&valid_hdr, OTA_PARTITION_MAX_SIZE + 1, OTA_PARTITION_MAX_SIZE),
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
        // Valid image without appended SHA-256
        let img_no_hash = make_valid_esp_image(false);
        let summary_no_hash = validate_ota_image_integrity(&img_no_hash);
        assert!(summary_no_hash.is_ok());
        if let Ok(sum) = summary_no_hash {
            assert_eq!(sum.segment_count, 1);
            assert_eq!(sum.entry_addr, 0x4008_0400);
            assert!(!sum.hash_appended);
        }

        // Valid image with appended SHA-256
        let img_hash = make_valid_esp_image(true);
        let summary_hash = validate_ota_image_integrity(&img_hash);
        assert!(summary_hash.is_ok());
        if let Ok(sum) = summary_hash {
            assert!(sum.hash_appended);
        }

        // Corrupted checksum byte
        let mut corrupted_chk = img_no_hash.clone();
        let chk_idx = 63;
        corrupted_chk[chk_idx] ^= 0xFF;
        assert_eq!(
            validate_ota_image_integrity(&corrupted_chk),
            Err(OtaError::InvalidChecksum)
        );

        // Corrupted payload byte -> checksum mismatch
        let mut corrupted_payload = img_no_hash.clone();
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
            validate_ota_image_integrity(&img_no_hash[..40]),
            Err(OtaError::ImageTooSmall)
        );
    }

    #[test]
    fn test_set_boot_slot_wraparound() {
        let mut mgr = OtaDataManager::new_booted_ota0();
        mgr.sector_0.ota_seq = u32::MAX - 2;
        mgr.sector_0.crc = mgr.sector_0.calculate_crc();

        let res = mgr.set_boot_slot(OtaSlot::Ota1);
        assert!(res.is_ok());
        assert_eq!(mgr.sector_1.ota_seq, 2);
        assert_eq!(mgr.sector_1.slot(), Some(OtaSlot::Ota1));

        let res0 = mgr.set_boot_slot(OtaSlot::Ota0);
        assert!(res0.is_ok());
        assert_eq!(mgr.sector_0.ota_seq, 1);
        assert_eq!(mgr.sector_0.slot(), Some(OtaSlot::Ota0));
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

        // 2. Build a valid firmware image with multi-chunk data
        let image = make_valid_esp_image_with_len(980, false);
        let img_len = image.len();

        let slot = mgr
            .write_ota_image_to_flash(&mut flash, &image)
            .expect("write OTA image to ota_1");
        assert_eq!(slot, OtaSlot::Ota1);
        assert_eq!(mgr.active_slot(), OtaSlot::Ota1);
        assert_eq!(mgr.sector_1.state(), OtaImgState::PendingVerify);

        // Verify ota_1 flash contents
        assert_eq!(&flash.ota_1[..img_len], &image[..]);

        // 3. Simulate reboot: reload OtaDataManager from flash and verify PendingVerify state on ota_1
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

        // Reload again from flash to confirm Valid state was persisted to sector 1 (0x18000)
        let reloaded = OtaDataManager::load_from_flash(&mut flash).expect("reload after Valid");
        assert_eq!(reloaded.active_slot(), OtaSlot::Ota1);
        assert_eq!(reloaded.sector_1.state(), OtaImgState::Valid);
    }
}
