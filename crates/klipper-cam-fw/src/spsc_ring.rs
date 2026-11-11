//! Lock-free Single-Producer Single-Consumer (SPSC) ring buffer for Core 1
//! JPEG chunk streaming.
//!
//! Provides bounded, non-blocking chunk queues between Core 0 / DMA frame
//! capture and Core 1 HTTP MJPEG streamer tasks.

// JUSTIFICATION: Lock-free SPSC inter-core queue requires UnsafeCell & Sync.
#![allow(unsafe_code)]

use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicU32, AtomicUsize, Ordering};

/// Maximum payload bytes carried by each chunk slot in the SPSC ring.
pub const JPEG_SPSC_CHUNK_BYTES: usize = 1024;

/// SPSC frame flag: Start-of-Frame (first chunk of a frame).
pub const SPSC_FLAG_SOF: u8 = 0x01;

/// SPSC frame flag: End-of-Frame (final chunk of a frame).
pub const SPSC_FLAG_EOF: u8 = 0x02;

/// SPSC frame flag: Abort / drop indicator for an interrupted frame.
pub const SPSC_FLAG_ABORT: u8 = 0x04;

/// Slot storing a single chunk of JPEG stream data in the SPSC ring.
#[repr(C, align(4))]
pub struct JpegSpscSlot {
    /// Payload storage.
    pub data: [u8; JPEG_SPSC_CHUNK_BYTES],
    /// Valid payload length in `data`.
    pub len: u16,
    /// Frame boundary and status flags (`SOF`, `EOF`, `ABORT`).
    pub flags: u8,
    /// Explicit padding to maintain 4-byte boundary.
    pub pad: u8,
}

impl JpegSpscSlot {
    /// Zero-initialized constant instance placed in `.bss` (NOBITS).
    pub const ZERO: Self = Self {
        data: [0; JPEG_SPSC_CHUNK_BYTES],
        len: 0,
        flags: 0,
        pad: 0,
    };
}

/// Bounded lock-free SPSC ring buffer for inter-core JPEG streaming.
pub struct Core1JpegSpsc<const SLOTS: usize> {
    head: AtomicUsize,
    tail: AtomicUsize,
    frames_pushed: AtomicU32,
    frames_dropped: AtomicU32,
    slots: [UnsafeCell<JpegSpscSlot>; SLOTS],
}

// JUSTIFICATION: Dual-core SPSC queue synchronizes via acquire/release atomics.
// SAFETY:
// 1. Alignment & Non-null: All internal slots are aligned to 4 bytes in array.
// 2. Memory layout & Bounds: Array length is fixed at `SLOTS`.
// 3. Provenance & Initialization: All slots zero-initialized at const creation.
// 4. Disjoint access & Aliasing: Producer only accesses slot `head`; consumer
//    only accesses slot `tail`. When `head == tail`, queue is empty.
// 5. Atomic synchronization: Head and tail updates use Release stores and
//    Acquire loads to ensure memory ordering between cores.
// 6. Panic freedom & no_std safety: Const assertions guarantee power-of-two.
unsafe impl<const SLOTS: usize> Sync for Core1JpegSpsc<SLOTS> {}

impl<const SLOTS: usize> Core1JpegSpsc<SLOTS> {
    /// Creates a new `Core1JpegSpsc` ring buffer.
    ///
    /// # Panics
    ///
    /// Panics at compile time if `SLOTS < 4` or `SLOTS` is not a power of two.
    #[must_use]
    pub const fn new() -> Self {
        assert!(
            SLOTS >= 4 && SLOTS.is_power_of_two(),
            "SLOTS must be a power of two >= 4"
        );
        Self {
            head: AtomicUsize::new(0),
            tail: AtomicUsize::new(0),
            frames_pushed: AtomicU32::new(0),
            frames_dropped: AtomicU32::new(0),
            slots: [const { UnsafeCell::new(JpegSpscSlot::ZERO) }; SLOTS],
        }
    }

    /// Pushes a single chunk into the ring buffer.
    ///
    /// Returns `true` if successfully enqueued, or `false` if the chunk exceeds
    /// `JPEG_SPSC_CHUNK_BYTES` or the ring is full.
    ///
    /// # Safety
    ///
    /// Must only be called by the designated single producer thread / core.
    // JUSTIFICATION: Lock-free single-producer single-consumer ring queue.
    // SAFETY:
    // 1. Alignment & Non-null: `self.slots[head]` is an inline array element,
    //    guaranteed 4-byte aligned and non-null by `#[repr(C, align(4))]`.
    // 2. Memory layout & Bounds: `head < SLOTS` is guaranteed by `head & mask`,
    //    so the slice index never exceeds `SLOTS`.
    // 3. Provenance & Initialization: `self.slots` is initialized in `.bss` or
    //    via `JpegSpscSlot::ZERO` and remains valid for lifetime of `self`.
    // 4. Disjoint access & Aliasing: The producer exclusively accesses the slot
    //    at index `head` because `next_head != tail` ensures consumer is not
    //    reading `self.slots[head]`.
    // 5. Atomic synchronization: Producer writes slot data before updating
    //    `self.head` with `Ordering::Release`, synchronizing with consumer
    //    `Ordering::Acquire`.
    // 6. Panic freedom & no_std safety: Slicing is bounded by `chunk.len() <=
    //    JPEG_SPSC_CHUNK_BYTES`, preventing panics.
    pub unsafe fn push_chunk(&self, chunk: &[u8], flags: u8) -> bool {
        if chunk.len() > JPEG_SPSC_CHUNK_BYTES {
            return false;
        }
        let mask = SLOTS - 1;
        let head = self.head.load(Ordering::Relaxed);
        let next_head = (head + 1) & mask;
        if next_head == self.tail.load(Ordering::Acquire) {
            return false;
        }
        let slot_ptr = self.slots[head].get();
        let chunk_len = chunk.len();
        // SAFETY: Exclusive producer access guaranteed by head != tail.
        unsafe {
            let slot = &mut *slot_ptr;
            slot.data[..chunk_len].copy_from_slice(chunk);
            slot.len = chunk_len as u16;
            slot.flags = flags;
        }
        self.head.store(next_head, Ordering::Release);
        true
    }

    /// Pops the next available chunk and passes its slice and flags to `f`.
    ///
    /// Returns `Some(res)` if a chunk was consumed, or `None` if empty.
    ///
    /// # Safety
    ///
    /// Must only be called by the designated single consumer thread / core.
    // JUSTIFICATION: Lock-free single-producer single-consumer ring queue.
    // SAFETY:
    // 1. Alignment & Non-null: `self.slots[tail]` is an inline array element,
    //    guaranteed 4-byte aligned and non-null by `#[repr(C, align(4))]`.
    // 2. Memory layout & Bounds: `tail < SLOTS` is guaranteed by `tail & mask`,
    //    so the slice index never exceeds `SLOTS`.
    // 3. Provenance & Initialization: Slot data was committed by producer
    //    prior to `head` store with `Ordering::Release`.
    // 4. Disjoint access & Aliasing: Consumer exclusively reads the slot at
    //    index `tail` because producer cannot overwrite it until `tail` advances.
    // 5. Atomic synchronization: Consumer checks `head` via `Ordering::Acquire`
    //    before reading slot, and stores `tail` with `Ordering::Release`.
    // 6. Panic freedom & no_std safety: `len` is checked against chunk capacity
    //    ensuring slice bounds are within `JPEG_SPSC_CHUNK_BYTES`.
    pub unsafe fn pop_with<R>(&self, f: impl FnOnce(&[u8], u8) -> R) -> Option<R> {
        let tail = self.tail.load(Ordering::Relaxed);
        if tail == self.head.load(Ordering::Acquire) {
            return None;
        }
        let slot_ptr = self.slots[tail].get();
        // SAFETY: Exclusive consumer access guaranteed while tail != head.
        let res = unsafe {
            let slot = &*slot_ptr;
            let len = usize::from(slot.len);
            let clamped_len = if len <= JPEG_SPSC_CHUNK_BYTES {
                len
            } else {
                JPEG_SPSC_CHUNK_BYTES
            };
            f(&slot.data[..clamped_len], slot.flags)
        };
        let mask = SLOTS - 1;
        let next_tail = (tail + 1) & mask;
        self.tail.store(next_tail, Ordering::Release);
        Some(res)
    }

    /// Resets the consumer index to match current head, discarding pending items.
    // JUSTIFICATION: Fast drain of obsolete JPEG chunks on stream abort.
    // SAFETY: Consumer advances `tail` to current `head` via atomic operations.
    pub fn reset_consumer(&self) {
        let head = self.head.load(Ordering::Acquire);
        self.tail.store(head, Ordering::Release);
    }

    /// Increments the total count of successfully queued full frames.
    pub fn record_frame_pushed(&self) {
        self.frames_pushed.fetch_add(1, Ordering::Relaxed);
    }

    /// Increments the total count of dropped or aborted frames.
    pub fn record_frame_dropped(&self) {
        self.frames_dropped.fetch_add(1, Ordering::Relaxed);
    }

    /// Returns the total number of frames pushed.
    #[must_use]
    pub fn frames_pushed(&self) -> u32 {
        self.frames_pushed.load(Ordering::Relaxed)
    }

    /// Returns the total number of frames dropped.
    #[must_use]
    pub fn frames_dropped(&self) -> u32 {
        self.frames_dropped.load(Ordering::Relaxed)
    }

    /// Returns `true` if the ring has no pending chunks.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.tail.load(Ordering::Relaxed) == self.head.load(Ordering::Acquire)
    }

    /// Returns the maximum chunk capacity of this ring (`SLOTS - 1`).
    #[must_use]
    pub const fn capacity(&self) -> usize {
        SLOTS - 1
    }
}

impl<const SLOTS: usize> Default for Core1JpegSpsc<SLOTS> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const SLOTS: usize> core::fmt::Debug for Core1JpegSpsc<SLOTS> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Core1JpegSpsc")
            .field("head", &self.head.load(Ordering::Relaxed))
            .field("tail", &self.tail.load(Ordering::Relaxed))
            .field("frames_pushed", &self.frames_pushed.load(Ordering::Relaxed))
            .field(
                "frames_dropped",
                &self.frames_dropped.load(Ordering::Relaxed),
            )
            .field("capacity", &(SLOTS - 1))
            .finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_spsc_ring_wraparound_25_chunks() {
        let ring = Core1JpegSpsc::<8>::new();
        assert_eq!(ring.capacity(), 7);
        assert!(ring.is_empty());

        for i in 0..25u8 {
            let chunk = [i; 32];
            let flags = if i == 0 {
                SPSC_FLAG_SOF
            } else if i == 24 {
                SPSC_FLAG_EOF
            } else {
                0
            };
            // SAFETY: Single thread in test context acts as exclusive producer.
            let ok = unsafe { ring.push_chunk(&chunk, flags) };
            assert!(ok, "push should succeed for chunk {i}");

            // SAFETY: Single thread in test context acts as exclusive consumer.
            let popped = unsafe {
                ring.pop_with(|data, f| {
                    assert_eq!(data, &chunk[..]);
                    assert_eq!(f, flags);
                    data[0]
                })
            };
            assert_eq!(popped, Some(i));
        }
        assert!(ring.is_empty());
    }

    #[test]
    fn test_spsc_ring_full_backpressure_and_counters() {
        let ring = Core1JpegSpsc::<8>::new();
        let payload = [0x5A; 128];

        for i in 0..7 {
            // SAFETY: Single producer in test context.
            let ok = unsafe { ring.push_chunk(&payload, 0) };
            assert!(ok, "slot {i} should push");
        }
        assert!(!ring.is_empty());

        // Ring is full (capacity is 7 for SLOTS=8)
        // SAFETY: Single producer in test context.
        let pushed_overflow = unsafe { ring.push_chunk(&payload, 0) };
        assert!(!pushed_overflow, "ring should reject when full");

        ring.record_frame_dropped();
        assert_eq!(ring.frames_dropped(), 1);

        // SAFETY: Drain consumer
        for _ in 0..7 {
            // SAFETY: Single consumer in test context.
            let res = unsafe { ring.pop_with(|_data, _flags| ()) };
            assert!(res.is_some());
        }
        assert!(ring.is_empty());

        ring.record_frame_pushed();
        assert_eq!(ring.frames_pushed(), 1);
    }

    #[test]
    fn test_spsc_ring_abort_marker_and_reset_consumer() {
        let ring = Core1JpegSpsc::<8>::new();
        let empty_chunk: [u8; 0] = [];

        // SAFETY: Single producer pushes zero-length abort marker.
        let ok = unsafe { ring.push_chunk(&empty_chunk, SPSC_FLAG_ABORT) };
        assert!(ok);

        // SAFETY: Single consumer inspects abort marker.
        let seen_abort = unsafe {
            ring.pop_with(|data, flags| {
                assert_eq!(data.len(), 0);
                (flags & SPSC_FLAG_ABORT) != 0
            })
        };
        assert_eq!(seen_abort, Some(true));
        assert!(ring.is_empty());

        // Test reset_consumer
        for _ in 0..3 {
            // SAFETY: Single producer in test context.
            let _ = unsafe { ring.push_chunk(&[0x11; 16], 0) };
        }
        assert!(!ring.is_empty());
        // Single consumer in test context.
        ring.reset_consumer();
        assert!(ring.is_empty());
    }
}
