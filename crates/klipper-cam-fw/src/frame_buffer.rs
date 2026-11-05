//! Zero-`unsafe` `#![no_std]` double-buffered JPEG frame manager.
//!
//! Backs two PSRAM / DRAM slots (`slot_a` and `slot_b`) for conflict-free
//! concurrent Port 80 `/snapshot`, Port 81 `/stream`, and background `I2S0`
//! DMA frame commits.

#![forbid(unsafe_code)]

use esp32_cam_types::CameraError;

/// Zero-`unsafe` `#![no_std]` double-buffered JPEG frame manager backing two
/// PSRAM/DRAM slots (`slot_a` and `slot_b`) for conflict-free concurrent
/// Port 80 `/snapshot`, Port 81 `/stream`, and background `I2S0` DMA frame
/// commits.
#[derive(Debug, PartialEq, Eq)]
pub struct PingPongFrameBuffer<'a> {
    slot_a: &'a mut [u8],
    slot_b: &'a mut [u8],
    len_a: usize,
    len_b: usize,
    active_is_b: bool,
    frame_seq: u64,
}

impl<'a> PingPongFrameBuffer<'a> {
    /// Initializes a double-buffered [`PingPongFrameBuffer`] backed by `slot_a`
    /// and `slot_b`, seeding both slots with `initial_frame` at sequence
    /// number `1`.
    pub fn new(
        slot_a: &'a mut [u8],
        slot_b: &'a mut [u8],
        initial_frame: &[u8],
    ) -> Result<Self, CameraError> {
        let init_len = initial_frame.len();
        if init_len == 0 {
            return Err(CameraError::InvalidArg);
        }
        if slot_a.len() < init_len || slot_b.len() < init_len {
            return Err(CameraError::BufferTooSmall);
        }
        if slot_a.len() != slot_b.len() {
            return Err(CameraError::InvalidArg);
        }
        slot_a[..init_len].copy_from_slice(initial_frame);
        Ok(Self {
            slot_a,
            slot_b,
            len_a: init_len,
            len_b: 0,
            active_is_b: false,
            frame_seq: 1,
        })
    }

    /// Returns a shared slice of the currently active front-buffer JPEG frame.
    #[inline]
    pub fn active_frame(&self) -> &[u8] {
        if self.active_is_b {
            &self.slot_b[..self.len_b]
        } else {
            &self.slot_a[..self.len_a]
        }
    }

    /// Returns the byte length of the currently active front-buffer JPEG frame.
    #[inline]
    pub const fn active_len(&self) -> usize {
        if self.active_is_b {
            self.len_b
        } else {
            self.len_a
        }
    }

    /// Returns the monotonically increasing sequence number of the active frame.
    #[inline]
    pub const fn frame_seq(&self) -> u64 {
        self.frame_seq
    }

    /// Returns the active frame generation sequence counter (`LOGIC-143`).
    #[inline]
    pub const fn active_generation(&self) -> u64 {
        self.frame_seq
    }

    /// Returns `0` when `slot_a` is the active front buffer, or `1` when `slot_b`
    /// is active.
    #[inline]
    pub const fn active_slot_index(&self) -> u8 {
        if self.active_is_b {
            1
        } else {
            0
        }
    }

    /// Splits the borrow across the disjoint back-buffer (`&mut [u8]`) and active
    /// front-buffer (`&[u8]`), enabling callers to stage or DMA-extract a new
    /// frame into the inactive slot while concurrently reading the active frame.
    #[inline]
    pub fn inactive_slot_and_active_frame(&mut self) -> (&mut [u8], &[u8]) {
        if self.active_is_b {
            (&mut *self.slot_a, &self.slot_b[..self.len_b])
        } else {
            (&mut *self.slot_b, &self.slot_a[..self.len_a])
        }
    }

    /// Commits `new_len` bytes previously written into the inactive back
    /// buffer, atomically flipping the active slot and incrementing
    /// `frame_seq`.
    pub fn commit_inactive(&mut self, new_len: usize) -> Result<u64, CameraError> {
        if new_len == 0 {
            return Err(CameraError::InvalidArg);
        }
        if self.active_is_b {
            if new_len > self.slot_a.len() {
                return Err(CameraError::BufferTooSmall);
            }
            self.len_a = new_len;
        } else {
            if new_len > self.slot_b.len() {
                return Err(CameraError::BufferTooSmall);
            }
            self.len_b = new_len;
        }
        self.active_is_b = !self.active_is_b;
        self.frame_seq = self.frame_seq.wrapping_add(1);
        Ok(self.frame_seq)
    }

    /// Copies `new_frame` into the inactive back buffer and commits it as the
    /// new active frame.
    pub fn write_and_commit_inactive(&mut self, new_frame: &[u8]) -> Result<u64, CameraError> {
        if new_frame.is_empty() {
            return Err(CameraError::InvalidArg);
        }
        let new_len = new_frame.len();
        if self.active_is_b {
            if new_len > self.slot_a.len() {
                return Err(CameraError::BufferTooSmall);
            }
            self.slot_a[..new_len].copy_from_slice(new_frame);
        } else {
            if new_len > self.slot_b.len() {
                return Err(CameraError::BufferTooSmall);
            }
            self.slot_b[..new_len].copy_from_slice(new_frame);
        }
        self.commit_inactive(new_len)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ping_pong_frame_buffer_init_and_alternating_commits() {
        let mut slot_a = [0u8; 64];
        let mut slot_b = [0u8; 64];
        let frame1 = [0xFF, 0xD8, 0xFF, 0xE0, 0x01, 0xFF, 0xD9];
        let frame2 = [0xFF, 0xD8, 0xFF, 0xE0, 0x02, 0x03, 0x04, 0xFF, 0xD9];
        let frame3 = [
            0xFF, 0xD8, 0xFF, 0xE0, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0xD9,
        ];

        let mut pp = PingPongFrameBuffer::new(&mut slot_a, &mut slot_b, &frame1)
            .expect("initialize PingPongFrameBuffer");
        assert_eq!(pp.frame_seq(), 1);
        assert_eq!(pp.active_slot_index(), 0);
        assert_eq!(pp.active_len(), frame1.len());
        assert_eq!(pp.active_frame(), &frame1);

        // Commit frame2 via write_and_commit_inactive -> flips to slot_b
        // (index 1), seq 2.
        let seq2 = pp
            .write_and_commit_inactive(&frame2)
            .expect("commit frame2 to inactive slot_b");
        assert_eq!(seq2, 2);
        assert_eq!(pp.frame_seq(), 2);
        assert_eq!(pp.active_slot_index(), 1);
        assert_eq!(pp.active_len(), frame2.len());
        assert_eq!(pp.active_frame(), &frame2);

        // Disjoint borrow: read active_frame (frame2 in slot_b) while staging
        // frame3 into slot_a.
        {
            let (inactive_slot, active_frame) = pp.inactive_slot_and_active_frame();
            assert_eq!(active_frame, &frame2);
            assert_eq!(inactive_slot.len(), 64);
            // Verify slot_a still held frame1 prefix prior to overwrite
            assert_eq!(&inactive_slot[..frame1.len()], &frame1);
            inactive_slot[..frame3.len()].copy_from_slice(&frame3);
            // Active frame remains untouched while inactive slot is mutated
            assert_eq!(active_frame, &frame2);
        }

        let seq3 = pp
            .commit_inactive(frame3.len())
            .expect("commit staged frame3 in slot_a");
        assert_eq!(seq3, 3);
        assert_eq!(pp.frame_seq(), 3);
        assert_eq!(pp.active_slot_index(), 0);
        assert_eq!(pp.active_len(), frame3.len());
        assert_eq!(pp.active_frame(), &frame3);

        // Another disjoint borrow cycle when slot_a (index 0) is active ->
        // writes into slot_b.
        {
            let (inactive_slot, active_frame) = pp.inactive_slot_and_active_frame();
            assert_eq!(active_frame, &frame3);
            assert_eq!(&inactive_slot[..frame2.len()], &frame2);
            inactive_slot[..frame1.len()].copy_from_slice(&frame1);
        }
        let seq4 = pp
            .commit_inactive(frame1.len())
            .expect("commit frame1 in slot_b");
        assert_eq!(seq4, 4);
        assert_eq!(pp.active_slot_index(), 1);
        assert_eq!(pp.active_frame(), &frame1);
    }

    #[test]
    fn test_ping_pong_frame_buffer_error_handling_preserves_active_frame() {
        let mut small_a = [0u8; 4];
        let mut small_b = [0u8; 8];
        let init = [0xFF, 0xD8, 0x01, 0x02, 0xFF, 0xD9];

        // Empty initial frame rejected
        assert_eq!(
            PingPongFrameBuffer::new(&mut small_a, &mut small_b, &[]),
            Err(CameraError::InvalidArg)
        );
        // Oversized initial frame for slot_a rejected
        assert_eq!(
            PingPongFrameBuffer::new(&mut small_a, &mut small_b, &init),
            Err(CameraError::BufferTooSmall)
        );

        let mut slot_a = [0u8; 8];
        let mut slot_b = [0u8; 12];
        // Unequal slot lengths rejected with InvalidArg (PERF-142)
        assert_eq!(
            PingPongFrameBuffer::new(&mut slot_a, &mut slot_b, &init),
            Err(CameraError::InvalidArg)
        );

        let mut slot_a = [0u8; 4];
        let mut slot_b = [0u8; 4];
        // Oversized initial frame for both slots rejected
        assert_eq!(
            PingPongFrameBuffer::new(&mut slot_a, &mut slot_b, &init),
            Err(CameraError::BufferTooSmall)
        );

        let mut slot_a = [0u8; 8];
        let mut slot_b = [0u8; 8];
        let mut pp = PingPongFrameBuffer::new(&mut slot_a, &mut slot_b, &init).expect("valid init");
        assert_eq!(pp.active_frame(), &init);
        assert_eq!(pp.frame_seq(), 1);
        assert_eq!(pp.active_generation(), 1);
        assert_eq!(pp.active_slot_index(), 0);

        // Empty write_and_commit_inactive rejected; active frame & seq preserved
        assert_eq!(
            pp.write_and_commit_inactive(&[]),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(pp.active_frame(), &init);
        assert_eq!(pp.frame_seq(), 1);
        assert_eq!(pp.active_slot_index(), 0);

        // Oversized write_and_commit_inactive rejected; active frame & seq
        // preserved.
        let oversized = [0xAA; 9];
        assert_eq!(
            pp.write_and_commit_inactive(&oversized),
            Err(CameraError::BufferTooSmall)
        );
        assert_eq!(pp.active_frame(), &init);
        assert_eq!(pp.frame_seq(), 1);
        assert_eq!(pp.active_slot_index(), 0);

        // Zero-length or oversized commit_inactive rejected; active frame & seq
        // preserved.
        assert_eq!(pp.commit_inactive(0), Err(CameraError::InvalidArg));
        assert_eq!(pp.commit_inactive(9), Err(CameraError::BufferTooSmall));
        assert_eq!(pp.active_frame(), &init);
        assert_eq!(pp.frame_seq(), 1);
        assert_eq!(pp.active_slot_index(), 0);
    }
}
