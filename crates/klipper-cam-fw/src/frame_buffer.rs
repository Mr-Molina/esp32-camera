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

/// Move-only reader pin token issued by [`TripleFrameBuffer`].
///
/// While a [`PinnedFrameSlot`] is live, [`TripleFrameBuffer`] refuses to
/// recycle that slot into `write_idx` or swap it out during promotion,
/// guaranteeing sound zero-`unsafe` scoped borrows via
/// [`TripleFrameBuffer::slot_bytes`] (`M-018` / `C3-002`).
#[derive(Debug, PartialEq, Eq)]
pub struct PinnedFrameSlot {
    slot_idx: usize,
    len: usize,
    seq: u64,
}

impl PinnedFrameSlot {
    /// Returns the pinned slot index (`0..=2`).
    #[inline]
    #[must_use]
    pub const fn slot_index(&self) -> u8 {
        self.slot_idx as u8
    }

    /// Returns the valid JPEG payload length in bytes for this pinned frame.
    #[inline]
    #[must_use]
    pub const fn len(&self) -> usize {
        self.len
    }

    /// Returns `true` if the pinned frame has zero bytes.
    #[inline]
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Returns the frame sequence number recorded when this slot was written.
    #[inline]
    #[must_use]
    pub const fn frame_seq(&self) -> u64 {
        self.seq
    }
}

/// Zero-`unsafe` `#![no_std]` triple-buffered JPEG frame manager.
///
/// Backs three PSRAM/DRAM slots (`read_idx`, `ready_idx`, `write_idx`) for
/// race-free concurrent HTTP streaming (active read slot), HTTP snapshots
/// (ready slot), and background DMA frame capture (write slot), with per-slot
/// sequence tracking (`M-169`) and reader pinning refcounts (`M-018`).
pub struct TripleFrameBuffer<'a> {
    slots: [&'a mut [u8]; 3],
    lens: [usize; 3],
    seqs: [u64; 3],
    pin_counts: [u32; 3],
    read_idx: usize,
    ready_idx: usize,
    write_idx: usize,
    has_new_ready: bool,
    latest_seq: u64,
}

impl<'a> PartialEq for TripleFrameBuffer<'a> {
    fn eq(&self, other: &Self) -> bool {
        self.lens == other.lens
            && self.seqs == other.seqs
            && self.pin_counts == other.pin_counts
            && self.read_idx == other.read_idx
            && self.ready_idx == other.ready_idx
            && self.write_idx == other.write_idx
            && self.has_new_ready == other.has_new_ready
            && self.latest_seq == other.latest_seq
            && self.slots[0][..self.lens[0]] == other.slots[0][..other.lens[0]]
            && self.slots[1][..self.lens[1]] == other.slots[1][..other.lens[1]]
            && self.slots[2][..self.lens[2]] == other.slots[2][..other.lens[2]]
    }
}

impl<'a> Eq for TripleFrameBuffer<'a> {}

impl<'a> core::fmt::Debug for TripleFrameBuffer<'a> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("TripleFrameBuffer")
            .field("lens", &self.lens)
            .field("seqs", &self.seqs)
            .field("pin_counts", &self.pin_counts)
            .field("read_idx", &self.read_idx)
            .field("ready_idx", &self.ready_idx)
            .field("write_idx", &self.write_idx)
            .field("has_new_ready", &self.has_new_ready)
            .field("latest_seq", &self.latest_seq)
            .finish()
    }
}

impl<'a> TripleFrameBuffer<'a> {
    /// Initializes a triple-buffered [`TripleFrameBuffer`] backed by three
    /// equal-length slots.
    ///
    /// Copies `initial_frame` into `slot_a` and `slot_b`.
    ///
    /// # Errors
    ///
    /// Returns [`CameraError::InvalidArg`] if `initial_frame` is empty or the
    /// three slots do not have identical capacities (`M-169`).
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
        if slot_a.len() != slot_b.len() || slot_b.len() != slot_c.len() {
            return Err(CameraError::InvalidArg);
        }
        slot_a[..init_len].copy_from_slice(initial_frame);
        slot_b[..init_len].copy_from_slice(initial_frame);
        Ok(Self {
            slots: [slot_a, slot_b, slot_c],
            lens: [init_len, init_len, 0],
            seqs: [1, 1, 0],
            pin_counts: [0, 0, 0],
            read_idx: 0,
            ready_idx: 1,
            write_idx: 2,
            has_new_ready: false,
            latest_seq: 1,
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

    /// Returns the sequence number of the active front-buffer frame (`M-169`).
    #[must_use]
    pub const fn frame_seq(&self) -> u64 {
        self.seqs[self.read_idx]
    }

    /// Returns the sequence number of the active front-buffer frame (`M-169`).
    #[must_use]
    pub const fn active_frame_seq(&self) -> u64 {
        self.seqs[self.read_idx]
    }

    /// Returns the monotonically increasing sequence number of the latest
    /// committed frame (`M-169`).
    #[must_use]
    pub const fn latest_frame_seq(&self) -> u64 {
        self.latest_seq
    }

    /// Returns `true` if a newer committed frame is waiting in the ready slot.
    #[must_use]
    pub const fn has_new_ready(&self) -> bool {
        self.has_new_ready
    }

    /// Returns the active reader pin refcount for `slot_idx` (`0..=2`).
    #[must_use]
    pub const fn pin_count(&self, slot_idx: usize) -> u32 {
        if slot_idx < 3 {
            self.pin_counts[slot_idx]
        } else {
            0
        }
    }

    /// Returns `true` if `slot_idx` currently has one or more pinned readers.
    #[must_use]
    pub const fn is_slot_pinned(&self, slot_idx: usize) -> bool {
        self.pin_count(slot_idx) > 0
    }

    /// Pins the currently active read slot, returning a move-only
    /// [`PinnedFrameSlot`] token that protects the slot from promotion or
    /// write recycling until [`Self::unpin_slot`] is called (`M-018`).
    #[must_use]
    pub fn pin_active_slot(&mut self) -> PinnedFrameSlot {
        let idx = self.read_idx;
        self.pin_counts[idx] = self.pin_counts[idx].saturating_add(1);
        PinnedFrameSlot {
            slot_idx: idx,
            len: self.lens[idx],
            seq: self.seqs[idx],
        }
    }

    /// Pins the latest committed snapshot slot (`ready_idx` if a newer frame is
    /// ready, else `read_idx`), protecting it from recycling until
    /// [`Self::unpin_slot`] is called (`M-018`).
    #[must_use]
    pub fn pin_latest_snapshot_slot(&mut self) -> PinnedFrameSlot {
        let idx = if self.has_new_ready {
            self.ready_idx
        } else {
            self.read_idx
        };
        self.pin_counts[idx] = self.pin_counts[idx].saturating_add(1);
        PinnedFrameSlot {
            slot_idx: idx,
            len: self.lens[idx],
            seq: self.seqs[idx],
        }
    }

    /// Releases a previously acquired [`PinnedFrameSlot`] reader pin (`M-018`).
    pub fn unpin_slot(&mut self, token: PinnedFrameSlot) {
        if token.slot_idx < 3 {
            self.pin_counts[token.slot_idx] = self.pin_counts[token.slot_idx].saturating_sub(1);
        }
    }

    /// Returns a scoped immutable slice for a live [`PinnedFrameSlot`] token.
    #[must_use]
    pub fn slot_bytes(&self, token: &PinnedFrameSlot) -> &[u8] {
        let slot = &self.slots[token.slot_idx];
        let clamped = token.len.min(slot.len());
        &slot[..clamped]
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
    /// increments `latest_seq`. Refuses to recycle `ready_idx` into `write_idx`
    /// while `ready_idx` is pinned by an active reader (`M-018`).
    ///
    /// # Errors
    ///
    /// Returns [`CameraError::InvalidArg`] if `new_len == 0` or exceeds
    /// the slot capacity.
    /// Returns [`CameraError::InvalidState`] if `ready_idx` or `write_idx` is
    /// currently pinned.
    pub fn commit_write_slot(&mut self, new_len: usize) -> Result<u64, CameraError> {
        if new_len == 0 || new_len > self.slots[self.write_idx].len() {
            return Err(CameraError::InvalidArg);
        }
        if self.pin_counts[self.ready_idx] != 0 || self.pin_counts[self.write_idx] != 0 {
            return Err(CameraError::InvalidState);
        }
        let next_seq = self.latest_seq.wrapping_add(1);
        self.lens[self.write_idx] = new_len;
        self.seqs[self.write_idx] = next_seq;
        self.latest_seq = next_seq;
        core::mem::swap(&mut self.write_idx, &mut self.ready_idx);
        self.has_new_ready = true;
        Ok(next_seq)
    }

    /// Promotes the ready slot to become the new active read slot.
    ///
    /// Returns `true` if a new ready frame was promoted, or `false` if no new
    /// ready frame was available or if the current active read slot is pinned
    /// by a reader (`M-018`).
    pub fn promote_ready_to_active(&mut self) -> bool {
        if self.has_new_ready && self.pin_counts[self.read_idx] == 0 {
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
            assert_eq!(tfb.latest_frame_seq(), i);
            assert_eq!(tfb.active_frame_seq(), 1);
            assert_eq!(tfb.frame_seq(), 1);
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
        assert_eq!(tfb.frame_seq(), 1);

        // Promote ready to active
        let promoted = tfb.promote_ready_to_active();
        assert!(promoted);
        assert!(!tfb.has_new_ready());
        // Active frame is now the last committed chunk (value 6)
        assert_eq!(tfb.active_frame(), &[6u8; 16]);
        assert_eq!(tfb.frame_seq(), 6);
        assert_eq!(tfb.active_frame_seq(), 6);

        // Second promote returns false when has_new_ready is false
        let second_promoted = tfb.promote_ready_to_active();
        assert!(!second_promoted);
    }

    #[test]
    fn test_triple_frame_buffer_pinning_blocks_recycling_and_promotion() {
        let mut slot_a = [0u8; 64];
        let mut slot_b = [0u8; 64];
        let mut slot_c = [0u8; 64];
        let init_frame = [0xFF, 0xD8, 0x01, 0xFF, 0xD9];

        let mut tfb = TripleFrameBuffer::new(&mut slot_a, &mut slot_b, &mut slot_c, &init_frame)
            .expect("initialize TripleFrameBuffer");

        // Pin active slot (slot 0) as a streaming reader.
        let active_pin = tfb.pin_active_slot();
        assert_eq!(active_pin.slot_index(), 0);
        assert_eq!(active_pin.frame_seq(), 1);
        assert_eq!(tfb.slot_bytes(&active_pin), &init_frame);
        assert!(tfb.is_slot_pinned(0));

        // Writes into write_idx / ready_idx still succeed while read_idx is pinned.
        let frame2 = [0xAA; 12];
        tfb.write_slot_mut()[..frame2.len()].copy_from_slice(&frame2);
        assert_eq!(tfb.commit_write_slot(frame2.len()), Ok(2));

        // Promotion is blocked while read_idx is pinned.
        assert!(!tfb.promote_ready_to_active());
        assert_eq!(tfb.slot_bytes(&active_pin), &init_frame);

        // Pin latest snapshot slot (ready_idx) -> blocks commit recycling ready_idx.
        let snap_pin = tfb.pin_latest_snapshot_slot();
        assert_eq!(snap_pin.frame_seq(), 2);
        assert_eq!(tfb.slot_bytes(&snap_pin), &frame2);
        let frame3 = [0xBB; 10];
        tfb.write_slot_mut()[..frame3.len()].copy_from_slice(&frame3);
        assert_eq!(
            tfb.commit_write_slot(frame3.len()),
            Err(CameraError::InvalidState)
        );
        assert_eq!(tfb.slot_bytes(&snap_pin), &frame2);

        // Unpin snapshot slot -> commit now succeeds.
        tfb.unpin_slot(snap_pin);
        assert_eq!(tfb.commit_write_slot(frame3.len()), Ok(3));
        assert_eq!(tfb.latest_snapshot_frame(), &frame3);

        // Unpin active slot -> promotion now succeeds.
        tfb.unpin_slot(active_pin);
        assert!(!tfb.is_slot_pinned(0));
        assert!(tfb.promote_ready_to_active());
        assert_eq!(tfb.active_frame(), &frame3);
        assert_eq!(tfb.active_frame_seq(), 3);
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

        // Uneven slot capacities rejected (M-169)
        let mut wider = [0u8; 24];
        assert_eq!(
            TripleFrameBuffer::new(&mut slot_a, &mut wider, &mut slot_c, &init),
            Err(CameraError::InvalidArg)
        );

        // PartialEq compares slot contents, not only indices/lengths (M-169)
        let mut a1 = [0u8; 16];
        let mut b1 = [0u8; 16];
        let mut c1 = [0u8; 16];
        let mut a2 = [0u8; 16];
        let mut b2 = [0u8; 16];
        let mut c2 = [0u8; 16];
        let init_alt = [0xFF, 0xD8, 0xBB, 0xFF, 0xD9];
        let tfb1 = TripleFrameBuffer::new(&mut a1, &mut b1, &mut c1, &init).expect("tfb1");
        let tfb2 = TripleFrameBuffer::new(&mut a2, &mut b2, &mut c2, &init_alt).expect("tfb2");
        assert_ne!(tfb1, tfb2);

        let mut tfb = TripleFrameBuffer::new(&mut slot_a, &mut slot_b, &mut slot_c, &init)
            .expect("valid init");

        // Commit zero length rejected
        assert_eq!(tfb.commit_write_slot(0), Err(CameraError::InvalidArg));
        // Commit oversized length rejected
        assert_eq!(tfb.commit_write_slot(17), Err(CameraError::InvalidArg));
    }
}
