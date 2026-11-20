//! OTADATA dual-sector selection and rollback state machine.
//!
//! Evaluates dual-sector OTA selection records, computes IEEE 802.3 CRC32
//! checksums matching ESP-IDF `bootloader_common_ota_select_crc`, and enforces
//! NOR-safe `New -> PendingVerify` and `PendingVerify -> Aborted` state
//! transitions.

use crate::partitions::{
    PartitionLayout, PartitionSpan, FLASH_SIZE_BYTES, MIN_PARTITION_OFFSET, OTADATA_MIN_SIZE,
};
use crate::rom_boundary::{AlignedBounce64, FlashError, FlashIo, FLASH_SECTOR_SIZE};

/// Reflected polynomial used for ESP32 OTADATA CRC32 calculation.
const CRC32_POLYNOMIAL_REFLECTED: u32 = 0xEDB8_8320;

/// Maximum number of boot candidates returned by OTADATA selection
/// (16 OTA + 1 factory).
pub const MAX_BOOT_CANDIDATES: usize = 17;

const _: () = assert!(MAX_BOOT_CANDIDATES >= 17);

/// Computes the ESP-IDF `bootloader_common_ota_select_crc` over `ota_seq`.
///
/// Uses `init = 0x0000_0000`, reflected polynomial `0xEDB8_8320`, and final
/// bitwise NOT (`!crc`) across the 4 little-endian bytes of `ota_seq`.
#[must_use]
pub const fn crc32_esp_ota(ota_seq: u32) -> u32 {
    let bytes = ota_seq.to_le_bytes();
    let mut crc: u32 = 0x0000_0000;
    let mut i = 0;
    while i < 4 {
        crc ^= bytes[i] as u32;
        let mut bit = 0;
        while bit < 8 {
            if (crc & 1) != 0 {
                crc = (crc >> 1) ^ CRC32_POLYNOMIAL_REFLECTED;
            } else {
                crc >>= 1;
            }
            bit += 1;
        }
        i += 1;
    }
    !crc
}

/// Lifecycle state of an OTA application image stored in `otadata`.
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OtaImgState {
    /// Newly flashed image awaiting initial boot observation (`0x0000_0000`).
    New = 0,
    /// Bootloader transitioned state awaiting runtime confirmation (`0x0000_0001`).
    PendingVerify = 1,
    /// Runtime confirmed operational state (`0x0000_0002`).
    Valid = 2,
    /// Explicitly invalidated image (`0x0000_0003`).
    Invalid = 3,
    /// Image verification failed or aborted on boot (`0x0000_0004`).
    Aborted = 4,
    /// Erased or uninitialized state (`0xFFFF_FFFF`).
    Undefined = 0xFFFF_FFFF,
}

impl OtaImgState {
    /// Converts a raw 32-bit state field into an [`OtaImgState`].
    ///
    /// Unknown values default to [`Self::Undefined`].
    #[inline]
    #[must_use]
    pub const fn from_u32(val: u32) -> Self {
        match val {
            0 => Self::New,
            1 => Self::PendingVerify,
            2 => Self::Valid,
            3 => Self::Invalid,
            4 => Self::Aborted,
            _ => Self::Undefined,
        }
    }
}

/// 32-byte dual-sector OTA selection entry (`esp_ota_select_entry_t`).
#[repr(C, align(4))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OtaSelectEntry {
    /// Monotonically increasing sequence number (`1..=u32::MAX - 1`).
    pub ota_seq: u32,
    /// Label or padding bytes.
    pub seq_label: [u8; 20],
    /// Current image state as raw `u32`.
    pub ota_state: u32,
    /// CRC32 over `ota_seq`.
    pub crc: u32,
}

impl OtaSelectEntry {
    /// Creates a new [`OtaSelectEntry`] with sequence number, state,
    /// and computed CRC32.
    #[must_use]
    pub const fn new(ota_seq: u32, state: OtaImgState) -> Self {
        Self {
            ota_seq,
            seq_label: [0xFFu8; 20],
            ota_state: state as u32,
            crc: crc32_esp_ota(ota_seq),
        }
    }

    /// Deserializes a 32-byte array into an [`OtaSelectEntry`].
    #[must_use]
    pub const fn from_bytes(bytes: [u8; 32]) -> Self {
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

    /// Serializes this entry into a 32-byte array for flash storage.
    #[must_use]
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

    /// Returns the parsed [`OtaImgState`].
    #[inline]
    #[must_use]
    pub const fn state(&self) -> OtaImgState {
        OtaImgState::from_u32(self.ota_state)
    }

    /// Returns `true` if `ota_seq` is initialized and `crc` matches
    /// `crc32_esp_ota(ota_seq)`.
    #[inline]
    #[must_use]
    pub const fn is_valid(&self) -> bool {
        self.ota_seq != 0 && self.ota_seq != u32::MAX && self.crc == crc32_esp_ota(self.ota_seq)
    }

    /// Returns `true` if this entry has a valid CRC and is not in
    /// `Invalid` or `Aborted` state.
    #[inline]
    #[must_use]
    pub const fn is_bootable(&self) -> bool {
        if !self.is_valid() {
            return false;
        }
        let st = self.state();
        !matches!(st, OtaImgState::Invalid | OtaImgState::Aborted)
    }
}

/// Errors occurring during OTADATA reading, parsing, or state mutation.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OtaSelectError {
    /// Low-level SPI flash failure.
    Flash(FlashError),
    /// Partition table does not contain valid or sufficient bootable partitions.
    InvalidOtaData,
}

impl core::fmt::Display for OtaSelectError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Flash(err) => write!(f, "Flash I/O error: {err:?}"),
            Self::InvalidOtaData => write!(f, "Invalid or missing OTADATA configuration"),
        }
    }
}

impl From<FlashError> for OtaSelectError {
    #[inline]
    fn from(err: FlashError) -> Self {
        Self::Flash(err)
    }
}

/// Boot target candidate describing an application partition to boot.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BootCandidate {
    /// Slot index (`0..15` for OTA slots, `0` for factory).
    pub slot_index: usize,
    /// Partition offset and size span.
    pub span: PartitionSpan,
    /// `true` if this candidate points to an OTA slot, `false` for factory.
    pub is_ota: bool,
}

#[derive(Clone, Copy)]
enum ActiveCopy {
    Copy0,
    Copy1,
}

/// Modular serial-number comparison (`a > b` modulo 2^32, RFC 1982 half-space)
/// so wrapped `ota_seq` (`u32::MAX - 1 -> 1`) orders `1` newer than
/// `u32::MAX - 1`.
#[inline]
const fn is_seq_newer(a: u32, b: u32) -> bool {
    a != b && a.wrapping_sub(b) < 0x8000_0000
}

#[inline]
const fn is_seq_newer_or_eq(a: u32, b: u32) -> bool {
    a.wrapping_sub(b) < 0x8000_0000
}

#[inline]
const fn is_excluded_state(entry: &OtaSelectEntry) -> bool {
    entry.is_valid() && matches!(entry.state(), OtaImgState::Aborted | OtaImgState::Invalid)
}

fn slot_for_seq(ota_seq: u32, ota_count: u8) -> Result<usize, OtaSelectError> {
    if ota_count == 0 || (ota_count as usize) > 16 {
        return Err(OtaSelectError::InvalidOtaData);
    }
    let zero_based = ota_seq
        .checked_sub(1)
        .ok_or(OtaSelectError::InvalidOtaData)?;
    Ok((zero_based % u32::from(ota_count)) as usize)
}

fn push_candidate(
    candidates: &mut [Option<BootCandidate>; MAX_BOOT_CANDIDATES],
    idx: &mut usize,
    cand: BootCandidate,
) {
    if *idx < candidates.len() {
        candidates[*idx] = Some(cand);
        *idx += 1;
    }
}

fn pick_active(e0: &OtaSelectEntry, e1: &OtaSelectEntry) -> Option<ActiveCopy> {
    let b0 = e0.is_bootable();
    let b1 = e1.is_bootable();
    match (b0, b1) {
        (true, true) => {
            if is_seq_newer_or_eq(e0.ota_seq, e1.ota_seq) {
                Some(ActiveCopy::Copy0)
            } else {
                Some(ActiveCopy::Copy1)
            }
        }
        (true, false) => Some(ActiveCopy::Copy0),
        (false, true) => Some(ActiveCopy::Copy1),
        (false, false) => None,
    }
}

fn build_ota_candidates(
    layout: &PartitionLayout,
    ota_seq: u32,
    excluded_mask: u16,
) -> Result<[Option<BootCandidate>; MAX_BOOT_CANDIDATES], OtaSelectError> {
    let mut candidates = [None; MAX_BOOT_CANDIDATES];
    if layout.ota_count == 0 {
        return build_fallback_candidates(layout, excluded_mask);
    }
    let ota_count = layout.ota_count as usize;
    if ota_count > 16 {
        return Err(OtaSelectError::InvalidOtaData);
    }
    let active_slot = slot_for_seq(ota_seq, layout.ota_count)?;
    let mut idx = 0;

    // 1. Active slot (skipped if marked aborted/invalid)
    if (excluded_mask & (1u16 << active_slot)) == 0 {
        if let Some(span) = layout.ota_slots[active_slot] {
            push_candidate(
                &mut candidates,
                &mut idx,
                BootCandidate {
                    slot_index: active_slot,
                    span,
                    is_ota: true,
                },
            );
        }
    }

    // 2. Remaining OTA slots in round-robin order, skipping excluded slots
    let mut step = 1;
    while step < ota_count {
        let slot = (active_slot + step) % ota_count;
        if (excluded_mask & (1u16 << slot)) == 0 {
            if let Some(span) = layout.ota_slots[slot] {
                push_candidate(
                    &mut candidates,
                    &mut idx,
                    BootCandidate {
                        slot_index: slot,
                        span,
                        is_ota: true,
                    },
                );
            }
        }
        step += 1;
    }

    // 3. Factory fallback if present
    if let Some(factory) = layout.factory {
        push_candidate(
            &mut candidates,
            &mut idx,
            BootCandidate {
                slot_index: 0,
                span: factory,
                is_ota: false,
            },
        );
    }

    if idx == 0 {
        return Err(OtaSelectError::InvalidOtaData);
    }

    Ok(candidates)
}

fn build_fallback_candidates(
    layout: &PartitionLayout,
    excluded_mask: u16,
) -> Result<[Option<BootCandidate>; MAX_BOOT_CANDIDATES], OtaSelectError> {
    let mut candidates = [None; MAX_BOOT_CANDIDATES];
    let ota_count = layout.ota_count as usize;
    if ota_count > 16 {
        return Err(OtaSelectError::InvalidOtaData);
    }
    let mut idx = 0;

    if let Some(factory) = layout.factory {
        push_candidate(
            &mut candidates,
            &mut idx,
            BootCandidate {
                slot_index: 0,
                span: factory,
                is_ota: false,
            },
        );
    }

    let mut slot = 0;
    while slot < ota_count {
        if (excluded_mask & (1u16 << slot)) == 0 {
            if let Some(span) = layout.ota_slots[slot] {
                push_candidate(
                    &mut candidates,
                    &mut idx,
                    BootCandidate {
                        slot_index: slot,
                        span,
                        is_ota: true,
                    },
                );
            }
        }
        slot += 1;
    }

    if idx == 0 {
        return Err(OtaSelectError::InvalidOtaData);
    }

    Ok(candidates)
}

fn write_otadata_sector<F: FlashIo>(
    flash: &mut F,
    offset: u32,
    entry: &OtaSelectEntry,
) -> Result<(), OtaSelectError> {
    let sector_size = FLASH_SECTOR_SIZE as u32;
    if offset < MIN_PARTITION_OFFSET || (offset & (sector_size - 1)) != 0 {
        return Err(OtaSelectError::InvalidOtaData);
    }
    let end = offset
        .checked_add(sector_size)
        .ok_or(OtaSelectError::InvalidOtaData)?;
    if end > FLASH_SIZE_BYTES {
        return Err(OtaSelectError::InvalidOtaData);
    }
    let sec_idx = offset / sector_size;
    flash.erase_sector(sec_idx)?;
    flash.write(offset, &entry.to_bytes())?;
    Ok(())
}

fn collect_excluded_mask(
    entry0: &OtaSelectEntry,
    entry1: &OtaSelectEntry,
    ota_count: u8,
) -> Result<u16, OtaSelectError> {
    let mut mask = 0u16;
    if ota_count == 0 {
        return Ok(0);
    }
    if (ota_count as usize) > 16 {
        return Err(OtaSelectError::InvalidOtaData);
    }
    if is_excluded_state(entry0) {
        let s0 = slot_for_seq(entry0.ota_seq, ota_count)?;
        let superseded = entry1.is_bootable()
            && slot_for_seq(entry1.ota_seq, ota_count).ok() == Some(s0)
            && is_seq_newer(entry1.ota_seq, entry0.ota_seq);
        if !superseded {
            mask |= 1u16 << s0;
        }
    }
    if is_excluded_state(entry1) {
        let s1 = slot_for_seq(entry1.ota_seq, ota_count)?;
        let superseded = entry0.is_bootable()
            && slot_for_seq(entry0.ota_seq, ota_count).ok() == Some(s1)
            && is_seq_newer(entry0.ota_seq, entry1.ota_seq);
        if !superseded {
            mask |= 1u16 << s1;
        }
    }
    Ok(mask)
}

/// Evaluates dual-sector OTADATA, performs rollback state updates,
/// and returns boot candidates.
///
/// Sector 0 resides at `otadata.offset`, and Sector 1 resides at
/// `otadata.offset + 0x1000`. Rejects `otadata` offsets below `0x9000`,
/// unaligned to 4 KiB, smaller than `0x2000`, or exceeding 4 MiB flash.
/// When rolling back from `PendingVerify` to a `New` alternate copy, only
/// the aborted sector is erased/written in this boot so the alternate copy
/// remains intact against power loss.
pub fn select_and_update_otadata<F: FlashIo>(
    flash: &mut F,
    layout: &PartitionLayout,
) -> Result<[Option<BootCandidate>; MAX_BOOT_CANDIDATES], OtaSelectError> {
    let sector_size = FLASH_SECTOR_SIZE as u32;
    let otadata = match layout.otadata {
        Some(span) => {
            if span.size < OTADATA_MIN_SIZE
                || span.offset < MIN_PARTITION_OFFSET
                || (span.offset & (sector_size - 1)) != 0
            {
                return Err(OtaSelectError::InvalidOtaData);
            }
            let end = span
                .offset
                .checked_add(span.size)
                .ok_or(OtaSelectError::InvalidOtaData)?;
            if end > FLASH_SIZE_BYTES {
                return Err(OtaSelectError::InvalidOtaData);
            }
            span
        }
        None => return build_fallback_candidates(layout, 0),
    };

    let sec0_offset = otadata.offset;
    let sec1_offset = otadata
        .offset
        .checked_add(sector_size)
        .ok_or(OtaSelectError::InvalidOtaData)?;
    let sec1_end = sec1_offset
        .checked_add(sector_size)
        .ok_or(OtaSelectError::InvalidOtaData)?;
    if sec1_end > FLASH_SIZE_BYTES {
        return Err(OtaSelectError::InvalidOtaData);
    }

    let mut bounce = AlignedBounce64::new();

    // Read Sector 0
    flash.read(sec0_offset, &mut bounce.bytes[..32])?;
    let mut e0_bytes = [0u8; 32];
    let mut k = 0;
    while k < 32 {
        e0_bytes[k] = bounce.bytes[k];
        k += 1;
    }
    let entry0 = OtaSelectEntry::from_bytes(e0_bytes);

    // Read Sector 1
    flash.read(sec1_offset, &mut bounce.bytes[..32])?;
    let mut e1_bytes = [0u8; 32];
    let mut j = 0;
    while j < 32 {
        e1_bytes[j] = bounce.bytes[j];
        j += 1;
    }
    let entry1 = OtaSelectEntry::from_bytes(e1_bytes);

    let mut excluded_mask = collect_excluded_mask(&entry0, &entry1, layout.ota_count)?;

    let active_pick = pick_active(&entry0, &entry1);
    let (active_entry, active_offset, alt_entry, alt_offset) = match active_pick {
        Some(ActiveCopy::Copy0) => (entry0, sec0_offset, entry1, sec1_offset),
        Some(ActiveCopy::Copy1) => (entry1, sec1_offset, entry0, sec0_offset),
        None => return build_fallback_candidates(layout, excluded_mask),
    };

    match active_entry.state() {
        OtaImgState::PendingVerify => {
            // Active image crashed before confirmation -> mark Aborted
            let mut aborted_entry = active_entry;
            aborted_entry.ota_state = OtaImgState::Aborted as u32;
            write_otadata_sector(flash, active_offset, &aborted_entry)?;

            if layout.ota_count > 0 {
                let aborted_slot = slot_for_seq(active_entry.ota_seq, layout.ota_count)?;
                excluded_mask |= 1u16 << aborted_slot;
            }

            // Re-evaluate alternate copy
            if alt_entry.is_bootable() {
                match alt_entry.state() {
                    OtaImgState::New => {
                        // Defer New -> PendingVerify to avoid erasing both
                        // OTADATA sectors in one boot (brownout safety).
                        build_ota_candidates(layout, alt_entry.ota_seq, excluded_mask)
                    }
                    OtaImgState::PendingVerify => {
                        let mut alt_aborted = alt_entry;
                        alt_aborted.ota_state = OtaImgState::Aborted as u32;
                        write_otadata_sector(flash, alt_offset, &alt_aborted)?;
                        if layout.ota_count > 0 {
                            let alt_slot = slot_for_seq(alt_entry.ota_seq, layout.ota_count)?;
                            excluded_mask |= 1u16 << alt_slot;
                        }
                        build_fallback_candidates(layout, excluded_mask)
                    }
                    _ => build_ota_candidates(layout, alt_entry.ota_seq, excluded_mask),
                }
            } else {
                build_fallback_candidates(layout, excluded_mask)
            }
        }
        OtaImgState::New => {
            // First boot of new OTA image -> Transition to PendingVerify
            let mut new_pv = active_entry;
            new_pv.ota_state = OtaImgState::PendingVerify as u32;
            write_otadata_sector(flash, active_offset, &new_pv)?;
            build_ota_candidates(layout, active_entry.ota_seq, excluded_mask)
        }
        _ => build_ota_candidates(layout, active_entry.ota_seq, excluded_mask),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rom_boundary::MockFlash;

    #[test]
    fn crc32_esp_ota_matches_expected() {
        let expected = crc32_esp_ota(1);
        let bytes = 1u32.to_le_bytes();
        let mut crc: u32 = 0;
        let mut i = 0;
        while i < 4 {
            crc ^= bytes[i] as u32;
            let mut bit = 0;
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
        assert_eq!(expected, !crc);
    }

    const TEST_OTADATA_OFFSET: u32 = 0x9000;

    fn test_layout(factory: Option<PartitionSpan>) -> PartitionLayout {
        PartitionLayout {
            otadata: Some(PartitionSpan {
                offset: TEST_OTADATA_OFFSET,
                size: 0x2000,
            }),
            factory,
            ota_slots: {
                let mut s = [None; 16];
                s[0] = Some(PartitionSpan {
                    offset: 0x20000,
                    size: 0x100000,
                });
                s[1] = Some(PartitionSpan {
                    offset: 0x120000,
                    size: 0x100000,
                });
                s
            },
            ota_count: 2,
        }
    }

    struct CountingFlash<const SLOTS: usize> {
        inner: MockFlash<SLOTS>,
        erases: usize,
    }

    impl<const SLOTS: usize> CountingFlash<SLOTS> {
        const fn new() -> Self {
            Self {
                inner: MockFlash::new(),
                erases: 0,
            }
        }
    }

    impl<const SLOTS: usize> FlashIo for CountingFlash<SLOTS> {
        fn read(&mut self, offset: u32, buf: &mut [u8]) -> Result<(), FlashError> {
            self.inner.read(offset, buf)
        }

        fn write(&mut self, offset: u32, data: &[u8]) -> Result<(), FlashError> {
            self.inner.write(offset, data)
        }

        fn erase_sector(&mut self, sector: u32) -> Result<(), FlashError> {
            self.erases += 1;
            self.inner.erase_sector(sector)
        }
    }

    #[test]
    fn empty_otadata_falls_back_to_ota_slots() {
        let mut mock = MockFlash::<12>::new();
        let layout = test_layout(None);

        let candidates = select_and_update_otadata(&mut mock, &layout).expect("select");
        assert_eq!(
            candidates[0],
            Some(BootCandidate {
                slot_index: 0,
                span: PartitionSpan {
                    offset: 0x20000,
                    size: 0x100000,
                },
                is_ota: true,
            })
        );
        assert_eq!(
            candidates[1],
            Some(BootCandidate {
                slot_index: 1,
                span: PartitionSpan {
                    offset: 0x120000,
                    size: 0x100000,
                },
                is_ota: true,
            })
        );
    }

    #[test]
    fn new_transitions_to_pending_verify_in_flash() {
        let mut mock = MockFlash::<12>::new();
        let entry0 = OtaSelectEntry::new(1, OtaImgState::New);
        mock.write_raw(TEST_OTADATA_OFFSET, &entry0.to_bytes())
            .expect("write entry");

        let layout = test_layout(None);

        let candidates = select_and_update_otadata(&mut mock, &layout).expect("select");
        assert_eq!(
            candidates[0],
            Some(BootCandidate {
                slot_index: 0,
                span: PartitionSpan {
                    offset: 0x20000,
                    size: 0x100000,
                },
                is_ota: true,
            })
        );

        // Verify flash state updated to PendingVerify
        let mut bounce = AlignedBounce64::new();
        mock.read(TEST_OTADATA_OFFSET, &mut bounce.bytes[..32])
            .expect("read");
        let mut bytes = [0u8; 32];
        bytes.copy_from_slice(&bounce.bytes[..32]);
        let readback = OtaSelectEntry::from_bytes(bytes);
        assert_eq!(readback.state(), OtaImgState::PendingVerify);
    }

    #[test]
    fn pending_verify_aborts_and_falls_back_to_alt_copy() {
        let mut mock = MockFlash::<12>::new();
        let entry0 = OtaSelectEntry::new(2, OtaImgState::PendingVerify);
        let entry1 = OtaSelectEntry::new(1, OtaImgState::Valid);
        mock.write_raw(TEST_OTADATA_OFFSET, &entry0.to_bytes())
            .expect("write e0");
        mock.write_raw(TEST_OTADATA_OFFSET + 0x1000, &entry1.to_bytes())
            .expect("write e1");

        let layout = test_layout(None);

        let candidates = select_and_update_otadata(&mut mock, &layout).expect("select");
        // entry0 was slot 1 (seq 2 -> (2-1)%2 = 1); candidate 0 must be slot 0
        assert_eq!(
            candidates[0],
            Some(BootCandidate {
                slot_index: 0,
                span: PartitionSpan {
                    offset: 0x20000,
                    size: 0x100000,
                },
                is_ota: true,
            })
        );
        assert_eq!(candidates[1], None);

        // Verify Sector 0 is marked Aborted
        let mut bounce = AlignedBounce64::new();
        mock.read(TEST_OTADATA_OFFSET, &mut bounce.bytes[..32])
            .expect("read");
        let mut bytes = [0u8; 32];
        bytes.copy_from_slice(&bounce.bytes[..32]);
        let readback = OtaSelectEntry::from_bytes(bytes);
        assert_eq!(readback.state(), OtaImgState::Aborted);
    }

    #[test]
    fn rejects_otadata_below_0x9000_unaligned_or_overflow_without_erasing() {
        for bad_span in [
            PartitionSpan {
                offset: 0x1000,
                size: 0x2000,
            },
            PartitionSpan {
                offset: 0x8000,
                size: 0x2000,
            },
            PartitionSpan {
                offset: 0x9800,
                size: 0x2000,
            },
            PartitionSpan {
                offset: 0x9000,
                size: 0x1000,
            },
            PartitionSpan {
                offset: 0xFFFF_F000,
                size: 0x2000,
            },
            PartitionSpan {
                offset: 0x3FF000,
                size: 0x2000,
            },
        ] {
            let mut flash = CountingFlash::<12>::new();
            let mut layout = test_layout(None);
            layout.otadata = Some(bad_span);
            assert_eq!(
                select_and_update_otadata(&mut flash, &layout),
                Err(OtaSelectError::InvalidOtaData)
            );
            assert_eq!(flash.erases, 0);
        }
    }

    #[test]
    fn active_slot_excluded_when_aborted_and_zero_candidates_errors() {
        let mut mock = MockFlash::<12>::new();
        // Both records map to slot 0: seq=3 (PendingVerify) aborts, seq=1 (Valid)
        // also maps to slot 0 and must not re-select the just-aborted slot 0.
        let e0 = OtaSelectEntry::new(3, OtaImgState::PendingVerify);
        let e1 = OtaSelectEntry::new(1, OtaImgState::Valid);
        mock.write_raw(TEST_OTADATA_OFFSET, &e0.to_bytes())
            .expect("write e0");
        mock.write_raw(TEST_OTADATA_OFFSET + 0x1000, &e1.to_bytes())
            .expect("write e1");

        let layout = test_layout(None);
        let cands = select_and_update_otadata(&mut mock, &layout).expect("fallback to slot 1");
        assert_eq!(cands[0].map(|c| c.slot_index), Some(1));
        assert_eq!(cands[1], None);

        // Single-slot layout with slot 0 aborted and no factory -> InvalidOtaData
        let mut single_layout = test_layout(None);
        single_layout.ota_slots[1] = None;
        single_layout.ota_count = 1;
        let res = build_ota_candidates(&single_layout, 1, 1u16 << 0);
        assert_eq!(res, Err(OtaSelectError::InvalidOtaData));
    }

    #[test]
    fn both_pending_verify_aborts_and_excludes_both_slots() {
        let mut mock = MockFlash::<12>::new();
        let e0 = OtaSelectEntry::new(2, OtaImgState::PendingVerify);
        let e1 = OtaSelectEntry::new(1, OtaImgState::PendingVerify);
        mock.write_raw(TEST_OTADATA_OFFSET, &e0.to_bytes())
            .expect("write e0");
        mock.write_raw(TEST_OTADATA_OFFSET + 0x1000, &e1.to_bytes())
            .expect("write e1");

        let factory = PartitionSpan {
            offset: 0x10000,
            size: 0x10000,
        };
        let layout = test_layout(Some(factory));
        let cands = select_and_update_otadata(&mut mock, &layout).expect("factory fallback");
        assert_eq!(
            cands[0],
            Some(BootCandidate {
                slot_index: 0,
                span: factory,
                is_ota: false,
            })
        );
        assert_eq!(cands[1], None);

        // Without factory, zero bootable candidates -> InvalidOtaData
        mock.write_raw(TEST_OTADATA_OFFSET, &e0.to_bytes())
            .expect("write e0");
        mock.write_raw(TEST_OTADATA_OFFSET + 0x1000, &e1.to_bytes())
            .expect("write e1");
        let no_factory = test_layout(None);
        assert_eq!(
            select_and_update_otadata(&mut mock, &no_factory),
            Err(OtaSelectError::InvalidOtaData)
        );
    }

    #[test]
    fn both_aborted_and_valid_with_aborted_alt_preserve_exclusions() {
        // Case A: Both sectors Aborted -> neither OTA slot is returned
        let mut mock = MockFlash::<12>::new();
        let a0 = OtaSelectEntry::new(2, OtaImgState::Aborted);
        let a1 = OtaSelectEntry::new(1, OtaImgState::Aborted);
        mock.write_raw(TEST_OTADATA_OFFSET, &a0.to_bytes())
            .expect("write a0");
        mock.write_raw(TEST_OTADATA_OFFSET + 0x1000, &a1.to_bytes())
            .expect("write a1");
        assert_eq!(
            select_and_update_otadata(&mut mock, &test_layout(None)),
            Err(OtaSelectError::InvalidOtaData)
        );

        // Case B: Active Valid (slot 0), Alt Aborted (slot 1) -> slot 1 excluded
        let mut mock = MockFlash::<12>::new();
        let v0 = OtaSelectEntry::new(3, OtaImgState::Valid);
        let a1 = OtaSelectEntry::new(2, OtaImgState::Aborted);
        mock.write_raw(TEST_OTADATA_OFFSET, &v0.to_bytes())
            .expect("write v0");
        mock.write_raw(TEST_OTADATA_OFFSET + 0x1000, &a1.to_bytes())
            .expect("write a1");
        let cands = select_and_update_otadata(&mut mock, &test_layout(None)).expect("slot 0 only");
        assert_eq!(cands[0].map(|c| c.slot_index), Some(0));
        assert_eq!(cands[1], None);
    }

    #[test]
    fn pending_verify_with_new_alt_erases_only_one_sector() {
        let mut flash = CountingFlash::<12>::new();
        let e0 = OtaSelectEntry::new(2, OtaImgState::PendingVerify);
        let e1 = OtaSelectEntry::new(1, OtaImgState::New);
        flash
            .inner
            .write_raw(TEST_OTADATA_OFFSET, &e0.to_bytes())
            .expect("write e0");
        flash
            .inner
            .write_raw(TEST_OTADATA_OFFSET + 0x1000, &e1.to_bytes())
            .expect("write e1");

        let cands = select_and_update_otadata(&mut flash, &test_layout(None)).expect("select alt");
        // Only the aborted sector is erased; alt sector stays intact New
        assert_eq!(flash.erases, 1);
        assert_eq!(cands[0].map(|c| c.slot_index), Some(0));
        assert_eq!(cands[1], None);

        let mut bounce = AlignedBounce64::new();
        flash
            .read(TEST_OTADATA_OFFSET, &mut bounce.bytes[..32])
            .expect("read s0");
        let mut raw0 = [0u8; 32];
        raw0.copy_from_slice(&bounce.bytes[..32]);
        assert_eq!(
            OtaSelectEntry::from_bytes(raw0).state(),
            OtaImgState::Aborted
        );

        flash
            .read(TEST_OTADATA_OFFSET + 0x1000, &mut bounce.bytes[..32])
            .expect("read s1");
        let mut raw1 = [0u8; 32];
        raw1.copy_from_slice(&bounce.bytes[..32]);
        assert_eq!(OtaSelectEntry::from_bytes(raw1).state(), OtaImgState::New);
    }

    #[test]
    fn pick_active_handles_ota_seq_wraparound_and_slot_for_seq_checked() {
        let wrapped_new = OtaSelectEntry::new(1, OtaImgState::Valid);
        let pre_wrap = OtaSelectEntry::new(u32::MAX - 1, OtaImgState::Valid);
        assert!(matches!(
            pick_active(&wrapped_new, &pre_wrap),
            Some(ActiveCopy::Copy0)
        ));
        assert!(matches!(
            pick_active(&pre_wrap, &wrapped_new),
            Some(ActiveCopy::Copy1)
        ));
        assert_eq!(slot_for_seq(0, 2), Err(OtaSelectError::InvalidOtaData));
        assert_eq!(slot_for_seq(1, 0), Err(OtaSelectError::InvalidOtaData));
    }
}
