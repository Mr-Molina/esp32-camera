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

/// Zero-`unsafe` `#![no_std]` triple-buffered JPEG frame manager.
///
/// Backs three PSRAM/DRAM slots (`read_idx`, `ready_idx`, `write_idx`) for
/// race-free concurrent HTTP streaming (active read slot), HTTP snapshots
/// (ready slot), and background DMA frame capture (write slot).
pub struct TripleFrameBuffer<'a> {
    slots: [&'a mut [u8]; 3],
    lens: [usize; 3],
    read_idx: usize,
    ready_idx: usize,
    write_idx: usize,
    has_new_ready: bool,
    frame_seq: u64,
}

impl<'a> PartialEq for TripleFrameBuffer<'a> {
    fn eq(&self, other: &Self) -> bool {
        self.lens == other.lens
            && self.read_idx == other.read_idx
            && self.ready_idx == other.ready_idx
            && self.write_idx == other.write_idx
            && self.has_new_ready == other.has_new_ready
            && self.frame_seq == other.frame_seq
    }
}

impl<'a> Eq for TripleFrameBuffer<'a> {}

impl<'a> core::fmt::Debug for TripleFrameBuffer<'a> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("TripleFrameBuffer")
            .field("lens", &self.lens)
            .field("read_idx", &self.read_idx)
            .field("ready_idx", &self.ready_idx)
            .field("write_idx", &self.write_idx)
            .field("has_new_ready", &self.has_new_ready)
            .field("frame_seq", &self.frame_seq)
            .finish()
    }
}

impl<'a> TripleFrameBuffer<'a> {
    /// Initializes a triple-buffered [`TripleFrameBuffer`] backed by three slots.
    ///
    /// Copies `initial_frame` into `slot_a` and `slot_b`.
    ///
    /// # Errors
    ///
    /// Returns [`CameraError::InvalidArg`] if `initial_frame` is empty.
    /// Returns [`CameraError::BufferTooSmall`] if any slot is smaller
    /// than `initial_frame`.
    pub fn new(
        slot_a: &'a mut [u8],
        slot_b: &'a mut [u8],
        slot_c: &'a mut [u8],
        initial_frame: &[u8],
    ) -> Result<Self, CameraError> {
        let init_len = initial_frame.len();
        if init_len == 0 {
            return Err(CameraError::InvalidArg);
        }
        if slot_a.len() < init_len || slot_b.len() < init_len || slot_c.len() < init_len {
            return Err(CameraError::BufferTooSmall);
        }
        slot_a[..init_len].copy_from_slice(initial_frame);
        slot_b[..init_len].copy_from_slice(initial_frame);
        Ok(Self {
            slots: [slot_a, slot_b, slot_c],
            lens: [init_len, init_len, 0],
            read_idx: 0,
            ready_idx: 1,
            write_idx: 2,
            has_new_ready: false,
            frame_seq: 1,
        })
    }

    /// Returns a shared slice of the currently active front-buffer JPEG frame.
    #[must_use]
    pub fn active_frame(&self) -> &[u8] {
        &self.slots[self.read_idx][..self.lens[self.read_idx]]
    }

    /// Returns the byte length of the currently active front-buffer JPEG frame.
    #[must_use]
    pub const fn active_len(&self) -> usize {
        self.lens[self.read_idx]
    }

    /// Returns index of the active front-buffer slot (`0..=2`).
    #[must_use]
    pub const fn active_slot_index(&self) -> u8 {
        self.read_idx as u8
    }

    /// Returns index of the ready slot (`0..=2`).
    #[must_use]
    pub const fn ready_slot_index(&self) -> u8 {
        self.ready_idx as u8
    }

    /// Returns index of the write slot (`0..=2`).
    #[must_use]
    pub const fn write_slot_index(&self) -> u8 {
        self.write_idx as u8
    }

    /// Returns the monotonically increasing sequence number of the active frame.
    #[must_use]
    pub const fn frame_seq(&self) -> u64 {
        self.frame_seq
    }

    /// Returns `true` if a newer committed frame is waiting in the ready slot.
    #[must_use]
    pub const fn has_new_ready(&self) -> bool {
        self.has_new_ready
    }

    /// Returns the latest committed frame for instantaneous snapshot delivery.
    ///
    /// If a newer frame is ready, returns the ready slot without waiting for
    /// stream reader promotion; otherwise returns the active stream frame.
    #[must_use]
    pub fn latest_snapshot_frame(&self) -> &[u8] {
        if self.has_new_ready {
            &self.slots[self.ready_idx][..self.lens[self.ready_idx]]
        } else {
            self.active_frame()
        }
    }

    /// Provides exclusive mutable access to the current staging/write slot.
    pub fn write_slot_mut(&mut self) -> &mut [u8] {
        self.slots[self.write_idx]
    }

    /// Commits `new_len` bytes written into the staging/write slot.
    ///
    /// Swaps the write and ready slots, sets `has_new_ready = true`, and
    /// increments `frame_seq`.
    ///
    /// # Errors
    ///
    /// Returns [`CameraError::InvalidArg`] if `new_len == 0` or exceeds
    /// the slot capacity.
    pub fn commit_write_slot(&mut self, new_len: usize) -> Result<u64, CameraError> {
        if new_len == 0 || new_len > self.slots[self.write_idx].len() {
            return Err(CameraError::InvalidArg);
        }
        self.lens[self.write_idx] = new_len;
        core::mem::swap(&mut self.write_idx, &mut self.ready_idx);
        self.has_new_ready = true;
        self.frame_seq = self.frame_seq.wrapping_add(1);
        Ok(self.frame_seq)
    }

    /// Promotes the ready slot to become the new active read slot.
    ///
    /// Returns `true` if a new ready frame was promoted, or `false` if no new
    /// ready frame was available.
    pub fn promote_ready_to_active(&mut self) -> bool {
        if self.has_new_ready {
            core::mem::swap(&mut self.read_idx, &mut self.ready_idx);
            self.has_new_ready = false;
            true
        } else {
            false
        }
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

    #[test]
    fn test_triple_frame_buffer_permutation_and_snapshot_isolation() {
        let mut slot_a = [0u8; 64];
        let mut slot_b = [0u8; 64];
        let mut slot_c = [0u8; 64];
        let init_frame = [0xFF, 0xD8, 0x01, 0xFF, 0xD9];

        let mut tfb = TripleFrameBuffer::new(&mut slot_a, &mut slot_b, &mut slot_c, &init_frame)
            .expect("initialize TripleFrameBuffer");

        assert_eq!(tfb.frame_seq(), 1);
        assert_eq!(tfb.active_slot_index(), 0);
        assert_eq!(tfb.ready_slot_index(), 1);
        assert_eq!(tfb.write_slot_index(), 2);
        assert!(!tfb.has_new_ready());
        assert_eq!(tfb.active_frame(), &init_frame);
        assert_eq!(tfb.latest_snapshot_frame(), &init_frame);

        // Verify permutation invariant across 5 consecutive commit_write_slot
        // while active read slot remains pinned at slot 0.
        for i in 2..=6u64 {
            let chunk_data = [i as u8; 16];
            tfb.write_slot_mut()[..chunk_data.len()].copy_from_slice(&chunk_data);
            let seq = tfb
                .commit_write_slot(chunk_data.len())
                .expect("commit write slot");
            assert_eq!(seq, i);
            assert_eq!(tfb.frame_seq(), i);
            assert!(tfb.has_new_ready());

            // Active frame remains pinned to initial frame at index 0
            assert_eq!(tfb.active_slot_index(), 0);
            assert_eq!(tfb.active_frame(), &init_frame);

            // Latest snapshot immediately reflects newest committed frame
            assert_eq!(tfb.latest_snapshot_frame(), &chunk_data);

            // Verify {read, ready, write} is always a permutation of {0, 1, 2}
            let r = tfb.active_slot_index();
            let y = tfb.ready_slot_index();
            let w = tfb.write_slot_index();
            assert_ne!(r, y);
            assert_ne!(r, w);
            assert_ne!(y, w);
            assert!(r <= 2 && y <= 2 && w <= 2);
        }

        // Active frame is still initial frame
        assert_eq!(tfb.active_frame(), &init_frame);

        // Promote ready to active
        let promoted = tfb.promote_ready_to_active();
        assert!(promoted);
        assert!(!tfb.has_new_ready());
        // Active frame is now the last committed chunk (value 6)
        assert_eq!(tfb.active_frame(), &[6u8; 16]);

        // Second promote returns false when has_new_ready is false
        let second_promoted = tfb.promote_ready_to_active();
        assert!(!second_promoted);
    }

    #[test]
    fn test_triple_frame_buffer_error_handling() {
        let mut slot_a = [0u8; 16];
        let mut slot_b = [0u8; 16];
        let mut slot_c = [0u8; 16];
        let init = [0xFF, 0xD8, 0xAA, 0xFF, 0xD9];

        // Empty init frame rejected
        assert_eq!(
            TripleFrameBuffer::new(&mut slot_a, &mut slot_b, &mut slot_c, &[]),
            Err(CameraError::InvalidArg)
        );

        // Buffer too small rejected
        let mut tiny = [0u8; 2];
        assert_eq!(
            TripleFrameBuffer::new(&mut tiny, &mut slot_b, &mut slot_c, &init),
            Err(CameraError::BufferTooSmall)
        );

        let mut tfb = TripleFrameBuffer::new(&mut slot_a, &mut slot_b, &mut slot_c, &init)
            .expect("valid init");

        // Commit zero length rejected
        assert_eq!(tfb.commit_write_slot(0), Err(CameraError::InvalidArg));
        // Commit oversized length rejected
        assert_eq!(tfb.commit_write_slot(17), Err(CameraError::InvalidArg));
    }
}
