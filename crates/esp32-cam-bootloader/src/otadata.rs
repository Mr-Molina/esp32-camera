//! OTADATA dual-sector selection and rollback state machine.
//!
//! Evaluates dual-sector OTA selection records, computes IEEE 802.3 CRC32
//! checksums matching ESP-IDF `bootloader_common_ota_select_crc`, and enforces
//! NOR-safe `New -> PendingVerify` and `PendingVerify -> Aborted` state
//! transitions.

use crate::partitions::{PartitionLayout, PartitionSpan};
use crate::rom_boundary::{AlignedBounce64, FlashError, FlashIo, FLASH_SECTOR_SIZE};

/// Reflected polynomial used for ESP32 OTADATA CRC32 calculation.
const CRC32_POLYNOMIAL_REFLECTED: u32 = 0xEDB8_8320;

/// Maximum number of boot candidates returned by OTADATA selection
/// (16 OTA + 1 factory).
pub const MAX_BOOT_CANDIDATES: usize = 17;

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

fn pick_active(e0: &OtaSelectEntry, e1: &OtaSelectEntry) -> Option<ActiveCopy> {
    let b0 = e0.is_bootable();
    let b1 = e1.is_bootable();
    match (b0, b1) {
        (true, true) => {
            if e0.ota_seq >= e1.ota_seq {
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
    aborted_slot: Option<usize>,
) -> Result<[Option<BootCandidate>; MAX_BOOT_CANDIDATES], OtaSelectError> {
    let mut candidates = [None; MAX_BOOT_CANDIDATES];
    if layout.ota_count == 0 {
        return build_fallback_candidates(layout, aborted_slot);
    }
    let ota_count = layout.ota_count as usize;
    let active_slot = ((ota_seq - 1) % (layout.ota_count as u32)) as usize;
    let mut idx = 0;

    // 1. Active slot
    if let Some(span) = layout.ota_slots[active_slot] {
        candidates[idx] = Some(BootCandidate {
            slot_index: active_slot,
            span,
            is_ota: true,
        });
        idx += 1;
    }

    // 2. Remaining OTA slots in round-robin order, skipping aborted slot
    let mut step = 1;
    while step < ota_count {
        let slot = (active_slot + step) % ota_count;
        if Some(slot) != aborted_slot {
            if let Some(span) = layout.ota_slots[slot] {
                candidates[idx] = Some(BootCandidate {
                    slot_index: slot,
                    span,
                    is_ota: true,
                });
                idx += 1;
            }
        }
        step += 1;
    }

    // 3. Factory fallback if present
    if let Some(factory) = layout.factory {
        candidates[idx] = Some(BootCandidate {
            slot_index: 0,
            span: factory,
            is_ota: false,
        });
    }

    Ok(candidates)
}

fn build_fallback_candidates(
    layout: &PartitionLayout,
    aborted_slot: Option<usize>,
) -> Result<[Option<BootCandidate>; MAX_BOOT_CANDIDATES], OtaSelectError> {
    let mut candidates = [None; MAX_BOOT_CANDIDATES];
    let mut idx = 0;

    if let Some(factory) = layout.factory {
        candidates[idx] = Some(BootCandidate {
            slot_index: 0,
            span: factory,
            is_ota: false,
        });
        idx += 1;
    }

    let mut slot = 0;
    while slot < layout.ota_count as usize {
        if Some(slot) != aborted_slot {
            if let Some(span) = layout.ota_slots[slot] {
                candidates[idx] = Some(BootCandidate {
                    slot_index: slot,
                    span,
                    is_ota: true,
                });
                idx += 1;
            }
        }
        slot += 1;
    }

    if idx == 0 {
        return Err(OtaSelectError::InvalidOtaData);
    }

    Ok(candidates)
}

/// Evaluates dual-sector OTADATA, performs rollback state updates,
/// and returns boot candidates.
///
/// Sector 0 resides at `otadata.offset`, and Sector 1 resides at
/// `otadata.offset + 0x1000`. If the active entry is in `New` state,
/// it is updated to `PendingVerify` in flash. If the active entry is in
/// `PendingVerify` state, it is updated to `Aborted` in flash and fallback
/// candidates are evaluated.
pub fn select_and_update_otadata<F: FlashIo>(
    flash: &mut F,
    layout: &PartitionLayout,
) -> Result<[Option<BootCandidate>; MAX_BOOT_CANDIDATES], OtaSelectError> {
    let otadata = match layout.otadata {
        Some(span) if span.size >= 0x2000 => span,
        _ => return build_fallback_candidates(layout, None),
    };

    let sec0_offset = otadata.offset;
    let sec1_offset = otadata.offset.saturating_add(FLASH_SECTOR_SIZE as u32);

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

    let active_pick = pick_active(&entry0, &entry1);
    let (active_entry, active_offset, alt_entry, alt_offset) = match active_pick {
        Some(ActiveCopy::Copy0) => (entry0, sec0_offset, entry1, sec1_offset),
        Some(ActiveCopy::Copy1) => (entry1, sec1_offset, entry0, sec0_offset),
        None => return build_fallback_candidates(layout, None),
    };

    match active_entry.state() {
        OtaImgState::PendingVerify => {
            // Active image crashed or watchdog-reset before confirmation -> Mark Aborted
            let mut aborted_entry = active_entry;
            aborted_entry.ota_state = OtaImgState::Aborted as u32;
            let sec_idx = active_offset / (FLASH_SECTOR_SIZE as u32);
            flash.erase_sector(sec_idx)?;
            flash.write(active_offset, &aborted_entry.to_bytes())?;

            let aborted_slot = if layout.ota_count > 0 {
                Some(((active_entry.ota_seq - 1) % (layout.ota_count as u32)) as usize)
            } else {
                None
            };

            // Re-evaluate alternate copy
            if alt_entry.is_bootable() {
                match alt_entry.state() {
                    OtaImgState::New => {
                        let mut new_pv = alt_entry;
                        new_pv.ota_state = OtaImgState::PendingVerify as u32;
                        let alt_sec = alt_offset / (FLASH_SECTOR_SIZE as u32);
                        flash.erase_sector(alt_sec)?;
                        flash.write(alt_offset, &new_pv.to_bytes())?;
                        build_ota_candidates(layout, alt_entry.ota_seq, aborted_slot)
                    }
                    OtaImgState::PendingVerify => {
                        let mut alt_aborted = alt_entry;
                        alt_aborted.ota_state = OtaImgState::Aborted as u32;
                        let alt_sec = alt_offset / (FLASH_SECTOR_SIZE as u32);
                        flash.erase_sector(alt_sec)?;
                        flash.write(alt_offset, &alt_aborted.to_bytes())?;
                        build_fallback_candidates(layout, aborted_slot)
                    }
                    _ => build_ota_candidates(layout, alt_entry.ota_seq, aborted_slot),
                }
            } else {
                build_fallback_candidates(layout, aborted_slot)
            }
        }
        OtaImgState::New => {
            // First boot of new OTA image -> Transition to PendingVerify
            let mut new_pv = active_entry;
            new_pv.ota_state = OtaImgState::PendingVerify as u32;
            let sec_idx = active_offset / (FLASH_SECTOR_SIZE as u32);
            flash.erase_sector(sec_idx)?;
            flash.write(active_offset, &new_pv.to_bytes())?;
            build_ota_candidates(layout, active_entry.ota_seq, None)
        }
        _ => build_ota_candidates(layout, active_entry.ota_seq, None),
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

    #[test]
    fn empty_otadata_falls_back_to_ota_slots() {
        let mut mock = MockFlash::<4>::new();
        let layout = PartitionLayout {
            otadata: Some(PartitionSpan {
                offset: 0x0000,
                size: 0x2000,
            }),
            factory: None,
            ota_slots: {
                let mut s = [None; 16];
                s[0] = Some(PartitionSpan {
                    offset: 0x2000,
                    size: 0x1000,
                });
                s[1] = Some(PartitionSpan {
                    offset: 0x3000,
                    size: 0x1000,
                });
                s
            },
            ota_count: 2,
        };

        let candidates = select_and_update_otadata(&mut mock, &layout).expect("select");
        assert_eq!(
            candidates[0],
            Some(BootCandidate {
                slot_index: 0,
                span: PartitionSpan {
                    offset: 0x2000,
                    size: 0x1000,
                },
                is_ota: true,
            })
        );
        assert_eq!(
            candidates[1],
            Some(BootCandidate {
                slot_index: 1,
                span: PartitionSpan {
                    offset: 0x3000,
                    size: 0x1000,
                },
                is_ota: true,
            })
        );
    }

    #[test]
    fn new_transitions_to_pending_verify_in_flash() {
        let mut mock = MockFlash::<4>::new();
        let entry0 = OtaSelectEntry::new(1, OtaImgState::New);
        mock.write_raw(0x0000, &entry0.to_bytes())
            .expect("write entry");

        let layout = PartitionLayout {
            otadata: Some(PartitionSpan {
                offset: 0x0000,
                size: 0x2000,
            }),
            factory: None,
            ota_slots: {
                let mut s = [None; 16];
                s[0] = Some(PartitionSpan {
                    offset: 0x2000,
                    size: 0x1000,
                });
                s[1] = Some(PartitionSpan {
                    offset: 0x3000,
                    size: 0x1000,
                });
                s
            },
            ota_count: 2,
        };

        let candidates = select_and_update_otadata(&mut mock, &layout).expect("select");
        assert_eq!(
            candidates[0],
            Some(BootCandidate {
                slot_index: 0,
                span: PartitionSpan {
                    offset: 0x2000,
                    size: 0x1000,
                },
                is_ota: true,
            })
        );

        // Verify flash state updated to PendingVerify
        let mut bounce = AlignedBounce64::new();
        mock.read(0x0000, &mut bounce.bytes[..32]).expect("read");
        let mut bytes = [0u8; 32];
        bytes.copy_from_slice(&bounce.bytes[..32]);
        let readback = OtaSelectEntry::from_bytes(bytes);
        assert_eq!(readback.state(), OtaImgState::PendingVerify);
    }

    #[test]
    fn pending_verify_aborts_and_falls_back_to_alt_copy() {
        let mut mock = MockFlash::<4>::new();
        let entry0 = OtaSelectEntry::new(2, OtaImgState::PendingVerify);
        let entry1 = OtaSelectEntry::new(1, OtaImgState::Valid);
        mock.write_raw(0x0000, &entry0.to_bytes())
            .expect("write e0");
        mock.write_raw(0x1000, &entry1.to_bytes())
            .expect("write e1");

        let layout = PartitionLayout {
            otadata: Some(PartitionSpan {
                offset: 0x0000,
                size: 0x2000,
            }),
            factory: None,
            ota_slots: {
                let mut s = [None; 16];
                s[0] = Some(PartitionSpan {
                    offset: 0x2000,
                    size: 0x1000,
                });
                s[1] = Some(PartitionSpan {
                    offset: 0x3000,
                    size: 0x1000,
                });
                s
            },
            ota_count: 2,
        };

        let candidates = select_and_update_otadata(&mut mock, &layout).expect("select");
        // entry0 was for slot 1 (seq 2 -> (2-1)%2 = 1). Since it was aborted,
        // candidate 0 should be slot 0!
        assert_eq!(
            candidates[0],
            Some(BootCandidate {
                slot_index: 0,
                span: PartitionSpan {
                    offset: 0x2000,
                    size: 0x1000,
                },
                is_ota: true,
            })
        );

        // Verify Sector 0 is marked Aborted
        let mut bounce = AlignedBounce64::new();
        mock.read(0x0000, &mut bounce.bytes[..32]).expect("read");
        let mut bytes = [0u8; 32];
        bytes.copy_from_slice(&bounce.bytes[..32]);
        let readback = OtaSelectEntry::from_bytes(bytes);
        assert_eq!(readback.state(), OtaImgState::Aborted);
    }
}
