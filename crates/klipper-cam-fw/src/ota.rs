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
    #[inline]
    pub const fn from_u32(val: u32) -> Self {
        match val {
            0x0 => Self::New,
            0x1 => Self::PendingVerify,
            0x2 => Self::Valid,
            0x3 => Self::Invalid,
            0x4 => Self::Aborted,
            _ => Self::Undefined,
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

    /// Creates a new [`OtaDataSelect`] with `ota_seq`, `state`, and valid IEEE 802.3 CRC-32.
    #[inline]
    pub const fn new(ota_seq: u32, state: OtaImgState) -> Self {
        let crc = crc32_ieee(&ota_seq.to_le_bytes());
        Self {
            ota_seq,
            seq_label: [0xFF; 20],
            ota_state: state as u32,
            crc,
        }
    }

    /// Calculates the IEEE 802.3 CRC-32 over the little-endian representation of `ota_seq`.
    #[inline]
    pub const fn calculate_crc(&self) -> u32 {
        crc32_ieee(&self.ota_seq.to_le_bytes())
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

    /// Deserializes a 32-byte little-endian array into an [`OtaDataSelect`].
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
        let active = self.active_slot();
        let changed = self.mark_app_valid_cancel_rollback()?;
        if changed {
            self.sync_slot_to_flash(flash, active)?;
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

        let mut next_seq = current_max.checked_add(1).ok_or(OtaError::SequenceOverflow)?;
        if OtaSlot::from_seq(next_seq) != Some(target_slot) {
            next_seq = next_seq.checked_add(1).ok_or(OtaError::SequenceOverflow)?;
        }

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
}

/// Validates an ESP32 OTA image prefix and total `Content-Length` against target partition bounds (`OTA-002`, `APP-008`).
///
/// Checks:
/// 1. `total_image_len > 0`
/// 2. `total_image_len <= max_partition_size` (`0x1B0000`)
/// 3. `image_prefix[0] == ESP_IMAGE_HEADER_MAGIC` (`0xE9`)
/// 4. `segment_count` in `1..=16` (`ESP_IMAGE_MIN_SEGMENTS..=ESP_IMAGE_MAX_SEGMENTS`)
pub fn validate_ota_image_header(
    image_prefix: &[u8],
    total_image_len: usize,
    max_partition_size: usize,
) -> Result<EspImageHeader, OtaError> {
    if total_image_len == 0 {
        return Err(OtaError::MissingContentLength);
    }
    if total_image_len > max_partition_size {
        return Err(OtaError::ImageTooLarge {
            image_size: total_image_len,
            partition_size: max_partition_size,
        });
    }
    if image_prefix.is_empty() {
        return Err(OtaError::HeaderTooShort);
    }
    let magic = image_prefix[0];
    if magic != ESP_IMAGE_HEADER_MAGIC {
        return Err(OtaError::InvalidMagicByte(magic));
    }

    let segment_count = if image_prefix.len() >= 2 {
        let count = image_prefix[1];
        if !(ESP_IMAGE_MIN_SEGMENTS..=ESP_IMAGE_MAX_SEGMENTS).contains(&count) {
            return Err(OtaError::InvalidSegmentCount(count));
        }
        count
    } else {
        1
    };

    let spi_mode = if image_prefix.len() >= 3 {
        image_prefix[2]
    } else {
        0
    };
    let spi_speed_size = if image_prefix.len() >= 4 {
        image_prefix[3]
    } else {
        0
    };
    let entry_addr = if image_prefix.len() >= 8 {
        u32::from_le_bytes([
            image_prefix[4],
            image_prefix[5],
            image_prefix[6],
            image_prefix[7],
        ])
    } else {
        0x4008_0000
    };

    Ok(EspImageHeader {
        magic,
        segment_count,
        spi_mode,
        spi_speed_size,
        entry_addr,
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
    /// Firmware payload is empty or too short to inspect header (`400 Bad Request`).
    HeaderTooShort,
    /// First byte of firmware image is not `0xE9` (`400 Bad Request`, `OTA-002`).
    InvalidMagicByte(u8),
    /// Segment count in ESP32 image header is outside `1..=16` (`400 Bad Request`, `OTA-002`).
    InvalidSegmentCount(u8),
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
            Self::HeaderTooShort => write!(f, "Firmware image validation failed"),
            Self::InvalidMagicByte(b) => {
                write!(f, "Firmware image validation failed: invalid magic 0x{:02X}", b)
            }
            Self::InvalidSegmentCount(c) => {
                write!(f, "Firmware image validation failed: invalid segments {}", c)
            }
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
mod tests {
    use super::*;

    #[test]
    fn test_partition_table_matches_csv() {
        assert!(verify_partition_table_integrity());
        assert_eq!(PARTITION_TABLE.len(), 5);

        let nvs = find_partition_by_label("nvs");
        assert!(nvs.is_some());
        if let Some(p) = nvs {
            assert_eq!(p.offset, 0x11000);
            assert_eq!(p.size, 0x6000);
        }

        let otadata = find_partition_by_label("otadata");
        assert!(otadata.is_some());
        if let Some(p) = otadata {
            assert_eq!(p.offset, 0x17000);
            assert_eq!(p.size, 0x2000);
        }

        let phy = find_partition_by_label("phy_init");
        assert!(phy.is_some());
        if let Some(p) = phy {
            assert_eq!(p.offset, 0x19000);
            assert_eq!(p.size, 0x1000);
        }

        let ota0 = find_partition_by_label("ota_0");
        assert!(ota0.is_some());
        if let Some(p) = ota0 {
            assert_eq!(p.offset, 0x20000);
            assert_eq!(p.size, 0x1B0000);
            assert_eq!(p.size, 1_769_472);
            assert_eq!(p.end_offset(), 0x1D0000);
        }

        let ota1 = find_partition_by_label("ota_1");
        assert!(ota1.is_some());
        if let Some(p) = ota1 {
            assert_eq!(p.offset, 0x1D0000);
            assert_eq!(p.size, 0x1B0000);
            assert_eq!(p.size, 1_769_472);
            assert_eq!(p.end_offset(), 0x380000);
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

        // Round-trip serialization
        let raw = s0.to_bytes();
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

        // Bad magic byte (0x00 instead of 0xE9)
        let bad_magic = [0x00, 0x03, 0x02, 0x20];
        assert_eq!(
            validate_ota_image_header(&bad_magic, 1024, OTA_PARTITION_MAX_SIZE),
            Err(OtaError::InvalidMagicByte(0x00))
        );

        // Invalid segment count (0 and 17)
        let zero_seg = [0xE9, 0x00, 0x02, 0x20];
        assert_eq!(
            validate_ota_image_header(&zero_seg, 1024, OTA_PARTITION_MAX_SIZE),
            Err(OtaError::InvalidSegmentCount(0))
        );
        let seventeen_seg = [0xE9, 17, 0x02, 0x20];
        assert_eq!(
            validate_ota_image_header(&seventeen_seg, 1024, OTA_PARTITION_MAX_SIZE),
            Err(OtaError::InvalidSegmentCount(17))
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

        // 2. Build a 1029-byte firmware image (1024 + 5 bytes -> tests multi-chunk + 3-byte 0xFF alignment padding)
        let mut image = [0x5Au8; 1029];
        let hdr = EspImageHeader::minimal_valid_header_bytes(4, 0x4008_0400);
        image[..24].copy_from_slice(&hdr);

        let slot = mgr
            .write_ota_image_to_flash(&mut flash, &image)
            .expect("write OTA image to ota_1");
        assert_eq!(slot, OtaSlot::Ota1);
        assert_eq!(mgr.active_slot(), OtaSlot::Ota1);
        assert_eq!(mgr.sector_1.state(), OtaImgState::PendingVerify);

        // Verify ota_1 flash contents and 0xFF trailing padding up to 1032-byte 4-byte boundary
        assert_eq!(&flash.ota_1[..1029], &image[..]);
        assert_eq!(&flash.ota_1[1029..1032], &[0xFF, 0xFF, 0xFF]);

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
