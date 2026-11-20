//! Static/PSRAM-backed `#![forbid(unsafe_code)]` frame pool
//! (`FramePool<COUNT, BUF_CAP>`) and RAII [`FrameGuard`] making
//! `esp_camera_fb_return` memory leaks impossible by construction.

#![forbid(unsafe_code)]

use core::ops::Deref;
use core::sync::atomic::{AtomicU8, AtomicUsize, Ordering};
use esp32_cam_types::{CameraError, GrabMode, PixelFormat};

/// A 4-byte aligned byte buffer for DMA controllers and hardware peripherals
/// (`Invariant 68`).
#[repr(C, align(4))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AlignedBuf<const N: usize>(pub [u8; N]);

impl<const N: usize> AlignedBuf<N> {
    /// Creates a new zero-initialized 4-byte aligned buffer.
    #[inline]
    pub const fn new() -> Self {
        Self([0u8; N])
    }

    /// Returns the buffer as a byte slice.
    #[inline]
    pub fn as_slice(&self) -> &[u8] {
        &self.0
    }

    /// Returns the buffer as a mutable byte slice.
    #[inline]
    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        &mut self.0
    }
}

impl<const N: usize> Default for AlignedBuf<N> {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> Deref for AlignedBuf<N> {
    type Target = [u8];

    #[inline]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<const N: usize> core::ops::DerefMut for AlignedBuf<N> {
    #[inline]
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl<const N: usize> AsRef<[u8]> for AlignedBuf<N> {
    #[inline]
    fn as_ref(&self) -> &[u8] {
        &self.0
    }
}

impl<const N: usize> AsMut<[u8]> for AlignedBuf<N> {
    #[inline]
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}

/// Slot state: buffer is free and available for DMA / capture write
/// (`en == 1` in `cam_hal.c`).
pub const SLOT_FREE: u8 = 0;

/// Slot state: buffer holds a completed, verified frame waiting to be taken by
/// the consumer.
pub const SLOT_READY: u8 = 1;

/// Slot state: buffer is currently leased out to a live [`FrameGuard`] and
/// cannot be overwritten.
pub const SLOT_LEASED: u8 = 2;

/// Internal slot state: buffer has been atomically reserved via CAS for an
/// in-flight producer write and is neither free nor readable (`M-118`).
const SLOT_WRITING: u8 = 3;

/// RAII cleanup guard for an in-flight slot write (`M-118`).
///
/// If the producer closure returns `Err`, reports a length exceeding `BUF_CAP`,
/// or unwinds via panic before `committed` is set, dropping this guard zeroes
/// the partially written buffer, resets `meta_len` to `0`, and returns the
/// slot to [`SLOT_FREE`].
struct WriteSlotGuard<'a> {
    state: &'a AtomicU8,
    buf: &'a mut [u8],
    meta_len: &'a mut usize,
    committed: bool,
}

impl Drop for WriteSlotGuard<'_> {
    #[inline]
    fn drop(&mut self) {
        if !self.committed {
            self.buf.fill(0);
            *self.meta_len = 0;
            self.state.store(SLOT_FREE, Ordering::Release);
        }
    }
}

/// Metadata associated with a captured frame buffer slot (`camera_fb_t`
/// fields).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct FrameMetadata {
    pub len: usize,
    pub width: u16,
    pub height: u16,
    pub format: PixelFormat,
    pub timestamp_us: u64,
    pub sequence: u64,
}

/// RAII guard for a leased camera frame buffer (`camera_fb_t`).
///
/// Implements [`Drop`] to automatically transition the slot state back to
/// [`SLOT_FREE`], guaranteeing at compile time that frame buffers are returned
/// to [`FramePool`] when dropped.
#[derive(Debug)]
pub struct FrameGuard<'a> {
    data: &'a mut [u8],
    len: usize,
    meta: FrameMetadata,
    slot_index: usize,
    state_flag: &'a AtomicU8,
}

impl<'a> FrameGuard<'a> {
    /// Returns the valid captured frame payload slice (`&[u8]` of length
    /// `self.len()`).
    #[inline]
    pub fn as_slice(&self) -> &[u8] {
        &self.data[..self.len]
    }

    /// Returns a mutable slice of the valid captured frame payload
    /// (`&mut [u8]` of length `self.len()`).
    #[inline]
    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        &mut self.data[..self.len]
    }

    /// Returns the length of the valid frame data in bytes (`camera_fb_t::len`).
    #[inline]
    pub const fn len(&self) -> usize {
        self.len
    }

    /// Returns `true` if the frame payload has zero bytes.
    #[inline]
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Truncates or updates the valid length of this frame (clamped to
    /// underlying capacity).
    ///
    /// Zeroes out newly exposed slack buffer space when expanding length to
    /// prevent uninitialized memory leakage (`SEC-051`).
    #[inline]
    pub fn set_len(&mut self, new_len: usize) {
        let clamped = new_len.min(self.data.len());
        if clamped > self.len {
            self.data[self.len..clamped].fill(0);
        }
        self.len = clamped;
        self.meta.len = self.len;
    }

    /// Returns the frame width in pixels (`camera_fb_t::width`).
    #[inline]
    pub const fn width(&self) -> u16 {
        self.meta.width
    }

    /// Returns the frame height in pixels (`camera_fb_t::height`).
    #[inline]
    pub const fn height(&self) -> u16 {
        self.meta.height
    }

    /// Returns the [`PixelFormat`] of the captured frame (`camera_fb_t::format`).
    #[inline]
    pub const fn format(&self) -> PixelFormat {
        self.meta.format
    }

    /// Returns the capture timestamp in microseconds (`camera_fb_t::timestamp`).
    #[inline]
    pub const fn timestamp_us(&self) -> u64 {
        self.meta.timestamp_us
    }

    /// Returns the monotonic frame sequence number.
    #[inline]
    pub const fn sequence(&self) -> u64 {
        self.meta.sequence
    }

    /// Returns the pool slot index (`0..COUNT`) backing this frame.
    #[inline]
    pub const fn slot_index(&self) -> usize {
        self.slot_index
    }
}

impl Deref for FrameGuard<'_> {
    type Target = [u8];

    #[inline]
    fn deref(&self) -> &Self::Target {
        self.as_slice()
    }
}

impl Drop for FrameGuard<'_> {
    #[inline]
    fn drop(&mut self) {
        self.data.fill(0);
        self.state_flag.store(SLOT_FREE, Ordering::Release);
    }
}

/// Static/PSRAM-backed multi-slot frame buffer pool
/// (`FramePool<COUNT, BUF_CAP>`).
///
/// Supports both [`GrabMode::WhenEmpty`] (preserves earliest unread frames;
/// drops incoming frames when full) and [`GrabMode::Latest`] (evicts older
/// unleased [`SLOT_READY`] frames so the consumer always receives the freshest
/// frame).
#[derive(Debug)]
pub struct FramePool<const COUNT: usize, const BUF_CAP: usize> {
    buffers: [AlignedBuf<BUF_CAP>; COUNT],
    meta: [FrameMetadata; COUNT],
    states: [AtomicU8; COUNT],
    grab_mode: GrabMode,
    next_seq: u64,
    write_cursor: AtomicUsize,
}

impl<const COUNT: usize, const BUF_CAP: usize> Default for FramePool<COUNT, BUF_CAP> {
    fn default() -> Self {
        Self::new(GrabMode::WhenEmpty)
    }
}

impl<const COUNT: usize, const BUF_CAP: usize> FramePool<COUNT, BUF_CAP> {
    /// Creates a new [`FramePool`] with all `COUNT` slots initialized to
    /// [`SLOT_FREE`].
    pub fn new(grab_mode: GrabMode) -> Self {
        Self {
            buffers: [AlignedBuf::new(); COUNT],
            meta: [FrameMetadata {
                len: 0,
                width: 0,
                height: 0,
                format: PixelFormat::Rgb565,
                timestamp_us: 0,
                sequence: 0,
            }; COUNT],
            states: core::array::from_fn(|_| AtomicU8::new(SLOT_FREE)),
            grab_mode,
            next_seq: 1,
            write_cursor: AtomicUsize::new(0),
        }
    }

    /// Returns the configured [`GrabMode`].
    #[inline]
    pub const fn grab_mode(&self) -> GrabMode {
        self.grab_mode
    }

    /// Sets the [`GrabMode`] policy (`WhenEmpty` vs `Latest`).
    #[inline]
    pub fn set_grab_mode(&mut self, grab_mode: GrabMode) {
        self.grab_mode = grab_mode;
    }

    /// Returns the number of slots currently in [`SLOT_FREE`] state.
    pub fn free_count(&self) -> usize {
        let mut n = 0;
        let mut i = 0;
        while i < COUNT {
            if self.states[i].load(Ordering::Acquire) == SLOT_FREE {
                n += 1;
            }
            i += 1;
        }
        n
    }

    /// Returns the number of completed frames currently waiting in
    /// [`SLOT_READY`] state (`esp_camera_available_frames`).
    pub fn available_frames(&self) -> usize {
        let mut n = 0;
        let mut i = 0;
        while i < COUNT {
            if self.states[i].load(Ordering::Acquire) == SLOT_READY {
                n += 1;
            }
            i += 1;
        }
        n
    }

    /// Resets all slots (including any leaked [`SLOT_LEASED`] slots) back to
    /// [`SLOT_FREE`] and clears metadata and buffers (`esp_camera_return_all`
    /// / `cam_give_all`, `M-118`).
    ///
    /// Taking `&mut self` statically guarantees no live [`FrameGuard`] borrows
    /// this pool, so resetting [`SLOT_LEASED`] slots safely recovers from
    /// `core::mem::forget`.
    pub fn return_all(&mut self) {
        let mut i = 0;
        while i < COUNT {
            self.buffers[i].0.fill(0);
            self.meta[i].len = 0;
            self.states[i].store(SLOT_FREE, Ordering::Release);
            i += 1;
        }
    }

    /// Returns a slice of the valid captured frame payload in slot `idx`
    /// (`..meta.len`), or `None` if out of bounds or not in [`SLOT_READY`]
    /// (`M-197`).
    #[inline]
    pub fn raw_buffer(&self, idx: usize) -> Option<&[u8]> {
        if idx < COUNT && self.states[idx].load(Ordering::Acquire) == SLOT_READY {
            let len = self.meta[idx].len.min(BUF_CAP);
            Some(&self.buffers[idx].0[..len])
        } else {
            None
        }
    }

    /// Returns a mutable slice to the raw backing buffer of slot `idx`, or
    /// `None` if out of bounds or not in [`SLOT_FREE`] (`LOGIC-054`, `M-197`).
    #[inline]
    pub fn raw_buffer_mut(&mut self, idx: usize) -> Option<&mut [u8]> {
        if idx < COUNT && self.states[idx].load(Ordering::Acquire) == SLOT_FREE {
            Some(&mut self.buffers[idx].0)
        } else {
            None
        }
    }

    /// Finds a target slot index for writing a new frame according to
    /// `self.grab_mode` (`LOGIC-052`, `M-118`).
    ///
    /// Starts from `self.write_cursor % COUNT` to prevent slots `2..COUNT`
    /// from starving in [`GrabMode::Latest`], and only advances `write_cursor`
    /// when a slot is available (never on [`CameraError::NoMem`]).
    pub fn find_target_slot(&self) -> Result<usize, CameraError> {
        if COUNT == 0 || BUF_CAP == 0 {
            return Err(CameraError::NoMem);
        }

        let start = self.write_cursor.load(Ordering::Relaxed) % COUNT;

        // First look for any SLOT_FREE buffer starting from write_cursor
        for offset in 0..COUNT {
            let i = (start + offset) % COUNT;
            if self.states[i].load(Ordering::Acquire) == SLOT_FREE {
                self.write_cursor.fetch_add(1, Ordering::Relaxed);
                return Ok(i);
            }
        }

        match self.grab_mode {
            GrabMode::WhenEmpty => Err(CameraError::NoMem),
            GrabMode::Latest => {
                // In Latest mode, if no slot is free, identify ONLY the oldest
                // ready slot. Never evict a SLOT_LEASED or SLOT_WRITING slot.
                let mut oldest: Option<(u64, usize)> = None;
                for offset in 0..COUNT {
                    let j = (start + offset) % COUNT;
                    if self.states[j].load(Ordering::Acquire) == SLOT_READY {
                        let seq = self.meta[j].sequence;
                        match oldest {
                            None => oldest = Some((seq, j)),
                            Some((best_seq, _)) if seq < best_seq => {
                                oldest = Some((seq, j));
                            }
                            _ => {}
                        }
                    }
                }
                if let Some((_, idx)) = oldest {
                    self.write_cursor.fetch_add(1, Ordering::Relaxed);
                    Ok(idx)
                } else {
                    Err(CameraError::NoMem)
                }
            }
        }
    }

    /// Atomically reserves a write slot by transitioning it via CAS to
    /// [`SLOT_WRITING`] without advancing `write_cursor` on
    /// [`CameraError::NoMem`] (`M-118`).
    fn reserve_write_slot(&self) -> Result<usize, CameraError> {
        if COUNT == 0 || BUF_CAP == 0 {
            return Err(CameraError::NoMem);
        }

        let start = self.write_cursor.load(Ordering::Relaxed) % COUNT;

        // 1. Atomically claim any SLOT_FREE buffer starting from write_cursor.
        for offset in 0..COUNT {
            let i = (start + offset) % COUNT;
            if self.states[i]
                .compare_exchange(SLOT_FREE, SLOT_WRITING, Ordering::AcqRel, Ordering::Acquire)
                .is_ok()
            {
                self.write_cursor.fetch_add(1, Ordering::Relaxed);
                return Ok(i);
            }
        }

        // 2. In Latest mode, atomically reclaim the oldest SLOT_READY slot.
        match self.grab_mode {
            GrabMode::WhenEmpty => Err(CameraError::NoMem),
            GrabMode::Latest => loop {
                let mut oldest: Option<(u64, usize)> = None;
                for offset in 0..COUNT {
                    let j = (start + offset) % COUNT;
                    if self.states[j].load(Ordering::Acquire) == SLOT_READY {
                        let seq = self.meta[j].sequence;
                        match oldest {
                            None => oldest = Some((seq, j)),
                            Some((best_seq, _)) if seq < best_seq => {
                                oldest = Some((seq, j));
                            }
                            _ => {}
                        }
                    }
                }
                let Some((_, idx)) = oldest else {
                    return Err(CameraError::NoMem);
                };
                if self.states[idx]
                    .compare_exchange(
                        SLOT_READY,
                        SLOT_WRITING,
                        Ordering::AcqRel,
                        Ordering::Acquire,
                    )
                    .is_ok()
                {
                    self.write_cursor.fetch_add(1, Ordering::Relaxed);
                    return Ok(idx);
                }
            },
        }
    }

    /// Zeroes and frees all [`SLOT_READY`] slots other than `keep_idx` for
    /// [`GrabMode::Latest`].
    fn drain_other_ready_slots(&mut self, keep_idx: usize) {
        for j in 0..COUNT {
            if j != keep_idx
                && self.states[j]
                    .compare_exchange(
                        SLOT_READY,
                        SLOT_WRITING,
                        Ordering::AcqRel,
                        Ordering::Acquire,
                    )
                    .is_ok()
            {
                self.buffers[j].0.fill(0);
                self.meta[j].len = 0;
                self.states[j].store(SLOT_FREE, Ordering::Release);
            }
        }
    }

    /// Pushes a complete frame payload into the pool according to
    /// [`GrabMode`], returning the assigned slot index.
    pub fn push_frame(
        &mut self,
        payload: &[u8],
        width: u16,
        height: u16,
        format: PixelFormat,
        timestamp_us: u64,
    ) -> Result<usize, CameraError> {
        if payload.len() > BUF_CAP {
            return Err(CameraError::BufferTooSmall);
        }
        let idx = self.reserve_write_slot()?;

        self.buffers[idx].0[..payload.len()].copy_from_slice(payload);
        self.buffers[idx].0[payload.len()..].fill(0);

        // Only after payload is successfully written, drain older ready slots
        // in GrabMode::Latest
        if self.grab_mode == GrabMode::Latest {
            self.drain_other_ready_slots(idx);
        }

        let seq = self.next_seq;
        self.next_seq = self.next_seq.wrapping_add(1);

        self.meta[idx] = FrameMetadata {
            len: payload.len(),
            width,
            height,
            format,
            timestamp_us,
            sequence: seq,
        };
        self.states[idx].store(SLOT_READY, Ordering::Release);
        Ok(idx)
    }

    /// Writes a frame in-place using a producer closure
    /// `writer(&mut [u8]) -> Result<usize, CameraError>`, transitions the slot
    /// to [`SLOT_LEASED`], and immediately returns the RAII [`FrameGuard`].
    pub fn write_and_lease<F>(
        &mut self,
        width: u16,
        height: u16,
        format: PixelFormat,
        timestamp_us: u64,
        writer: F,
    ) -> Result<FrameGuard<'_>, CameraError>
    where
        F: FnOnce(&mut [u8]) -> Result<usize, CameraError>,
    {
        let idx = self.reserve_write_slot()?;

        // Guard transitions slot back to SLOT_FREE and zeroes the buffer on
        // error, oversize length, or panic unwind (M-118).
        let mut write_guard = WriteSlotGuard {
            state: &self.states[idx],
            buf: &mut self.buffers[idx].0,
            meta_len: &mut self.meta[idx].len,
            committed: false,
        };

        let written_len = writer(write_guard.buf)?;
        if written_len > BUF_CAP {
            return Err(CameraError::BufferTooSmall);
        }
        write_guard.buf[written_len..].fill(0);
        write_guard.committed = true;
        drop(write_guard);

        // Only after writer succeeds and validates capacity, drain any older
        // SLOT_READY slots in GrabMode::Latest
        if self.grab_mode == GrabMode::Latest {
            self.drain_other_ready_slots(idx);
        }

        let seq = self.next_seq;
        self.next_seq = self.next_seq.wrapping_add(1);

        let meta = FrameMetadata {
            len: written_len,
            width,
            height,
            format,
            timestamp_us,
            sequence: seq,
        };
        self.meta[idx] = meta;
        self.states[idx].store(SLOT_LEASED, Ordering::Release);

        Ok(FrameGuard {
            data: &mut self.buffers[idx].0,
            len: written_len,
            meta,
            slot_index: idx,
            state_flag: &self.states[idx],
        })
    }

    /// Takes the next available [`SLOT_READY`] frame from the pool and returns
    /// an RAII [`FrameGuard`].
    ///
    /// - In [`GrabMode::WhenEmpty`], returns the earliest ready frame (FIFO by
    ///   `sequence`).
    /// - In [`GrabMode::Latest`], returns the newest ready frame (LIFO by
    ///   `sequence`) and releases any older ready frames.
    pub fn take(&mut self) -> Result<FrameGuard<'_>, CameraError> {
        let mut chosen_idx = None;
        match self.grab_mode {
            GrabMode::WhenEmpty => {
                let mut oldest: Option<(u64, usize)> = None;
                for i in 0..COUNT {
                    if self.states[i].load(Ordering::Acquire) == SLOT_READY {
                        let seq = self.meta[i].sequence;
                        match oldest {
                            None => oldest = Some((seq, i)),
                            Some((best_seq, _)) if seq < best_seq => {
                                oldest = Some((seq, i));
                            }
                            _ => {}
                        }
                    }
                }
                chosen_idx = oldest.map(|(_, idx)| idx);
            }
            GrabMode::Latest => {
                let mut newest: Option<(u64, usize)> = None;
                for i in 0..COUNT {
                    if self.states[i].load(Ordering::Acquire) == SLOT_READY {
                        let seq = self.meta[i].sequence;
                        match newest {
                            None => newest = Some((seq, i)),
                            Some((best_seq, _)) if seq >= best_seq => {
                                newest = Some((seq, i));
                            }
                            _ => {}
                        }
                    }
                }
                if let Some((_, win)) = newest {
                    chosen_idx = Some(win);
                    self.drain_other_ready_slots(win);
                }
            }
        }

        let idx = chosen_idx.ok_or(CameraError::Timeout)?;
        let meta = self.meta[idx];
        self.states[idx]
            .compare_exchange(SLOT_READY, SLOT_LEASED, Ordering::AcqRel, Ordering::Acquire)
            .map_err(|_| CameraError::Timeout)?;

        Ok(FrameGuard {
            data: &mut self.buffers[idx].0,
            len: meta.len,
            meta,
            slot_index: idx,
            state_flag: &self.states[idx],
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_raii_frame_guard_drop_releases_slot() {
        let mut pool: FramePool<2, 64> = FramePool::new(GrabMode::WhenEmpty);
        assert_eq!(pool.free_count(), 2);
        assert_eq!(pool.available_frames(), 0);

        pool.push_frame(&[0xFF, 0xD8, 0xFF, 0xD9], 320, 240, PixelFormat::Jpeg, 1000)
            .expect("push frame 1");
        assert_eq!(pool.free_count(), 1);
        assert_eq!(pool.available_frames(), 1);

        {
            let guard = pool.take().expect("take frame 1");
            assert_eq!(guard.as_slice(), &[0xFF, 0xD8, 0xFF, 0xD9]);
            assert_eq!(guard.width(), 320);
            assert_eq!(guard.height(), 240);
            assert_eq!(guard.format(), PixelFormat::Jpeg);
            assert_eq!(guard.timestamp_us(), 1000);
            assert_eq!(guard.slot_index(), 0);
        }
        // Once `guard` drops out of scope, its slot is automatically returned
        // to `SLOT_FREE`!
        assert_eq!(pool.free_count(), 2);
        assert_eq!(pool.available_frames(), 0);
    }

    #[test]
    fn test_frame_pool_backing_buffers_4_byte_aligned() {
        let mut pool: FramePool<3, 17> = FramePool::new(GrabMode::WhenEmpty);
        for i in 0..3 {
            let buf = pool.raw_buffer_mut(i).expect("free slot exists");
            assert_eq!(
                (buf.as_ptr() as usize) % 4,
                0,
                "Slot {} raw buffer {:p} is not 4-byte aligned",
                i,
                buf.as_ptr()
            );
        }

        // Verify write_and_lease receives 4-byte aligned buffer across slots
        for _ in 0..3 {
            let guard = pool
                .write_and_lease(160, 120, PixelFormat::Grayscale, 100, |buf| {
                    assert_eq!(
                        (buf.as_ptr() as usize) % 4,
                        0,
                        "Writer buffer {:p} is not 4-byte aligned",
                        buf.as_ptr()
                    );
                    Ok(4)
                })
                .expect("write_and_lease slot");
            assert_eq!(
                (guard.as_slice().as_ptr() as usize) % 4,
                0,
                "FrameGuard slice {:p} is not 4-byte aligned",
                guard.as_slice().as_ptr()
            );
        }
    }

    #[test]
    fn test_grab_mode_when_empty_vs_latest() {
        // 1. GrabMode::WhenEmpty preserves earliest frames and rejects overflow
        let mut pool_empty: FramePool<2, 16> = FramePool::new(GrabMode::WhenEmpty);
        assert!(pool_empty
            .push_frame(&[1, 1], 160, 120, PixelFormat::Grayscale, 10)
            .is_ok());
        assert!(pool_empty
            .push_frame(&[2, 2], 160, 120, PixelFormat::Grayscale, 20)
            .is_ok());
        assert_eq!(
            pool_empty.push_frame(&[3, 3], 160, 120, PixelFormat::Grayscale, 30),
            Err(CameraError::NoMem)
        );

        {
            let g1 = pool_empty.take().expect("first frame");
            assert_eq!(g1.as_slice(), &[1, 1]);
        }
        {
            let g2 = pool_empty.take().expect("second frame");
            assert_eq!(g2.as_slice(), &[2, 2]);
        }

        // 2. GrabMode::Latest overwrites older unleased frames so consumer gets
        // the freshest
        let mut pool_latest: FramePool<2, 16> = FramePool::new(GrabMode::Latest);
        assert!(pool_latest
            .push_frame(&[10, 10], 160, 120, PixelFormat::Grayscale, 100)
            .is_ok());
        assert!(pool_latest
            .push_frame(&[20, 20], 160, 120, PixelFormat::Grayscale, 200)
            .is_ok());
        assert!(pool_latest
            .push_frame(&[30, 30], 160, 120, PixelFormat::Grayscale, 300)
            .is_ok());

        let latest = pool_latest.take().expect("latest frame");
        assert_eq!(latest.as_slice(), &[30, 30]);
        assert_eq!(latest.timestamp_us(), 300);
    }

    #[test]
    fn test_hal_005_write_and_lease_writer_failure_preserves_ready_slots() {
        let mut pool: FramePool<2, 16> = FramePool::new(GrabMode::Latest);
        pool.push_frame(&[42, 42], 160, 120, PixelFormat::Grayscale, 100)
            .expect("push valid ready frame");
        assert_eq!(pool.available_frames(), 1);

        // A failed writer must NOT wipe existing ready frames before writing
        // succeeds!
        let write_res = pool.write_and_lease(160, 120, PixelFormat::Grayscale, 200, |_buf| {
            Err(CameraError::CorruptedFrame)
        });
        assert_eq!(write_res.err(), Some(CameraError::CorruptedFrame));

        // Ready frame 1 is still intact and can be taken
        assert_eq!(pool.available_frames(), 1);
        let guard = pool.take().expect("take existing ready frame");
        assert_eq!(guard.as_slice(), &[42, 42]);
    }

    #[test]
    fn test_hal_005_u64_max_sequence_handling() {
        let mut pool: FramePool<2, 16> = FramePool::new(GrabMode::WhenEmpty);
        // Force sequence number to u64::MAX
        pool.next_seq = u64::MAX;
        pool.push_frame(&[99, 99], 160, 120, PixelFormat::Grayscale, 10)
            .expect("push u64::MAX sequence frame");
        assert_eq!(pool.available_frames(), 1);

        let guard = pool.take().expect("take u64::MAX sequence frame");
        assert_eq!(guard.sequence(), u64::MAX);
        assert_eq!(guard.as_slice(), &[99, 99]);
    }

    #[test]
    fn test_write_and_lease_error_in_latest_mode_marks_slot_free() {
        let mut pool: FramePool<1, 16> = FramePool::new(GrabMode::Latest);
        pool.push_frame(&[1, 1], 160, 120, PixelFormat::Grayscale, 100)
            .expect("push frame 1");
        assert_eq!(pool.available_frames(), 1);
        assert_eq!(pool.free_count(), 0);

        // In Latest mode with no free slots, write_and_lease reclaims the
        // oldest ready slot (slot 0). If writer fails after partially writing,
        // slot 0 MUST become SLOT_FREE with len 0, and must NOT be left in
        // SLOT_READY with corrupted data.
        let res = pool.write_and_lease(160, 120, PixelFormat::Grayscale, 300, |buf| {
            buf[0] = 0xDE;
            buf[1] = 0xAD;
            Err(CameraError::CorruptedFrame)
        });
        assert_eq!(res.err(), Some(CameraError::CorruptedFrame));

        // Slot 0 was freed; no frames are ready and corrupted data cannot be
        // taken.
        assert_eq!(pool.available_frames(), 0);
        assert_eq!(pool.free_count(), 1);
        assert_eq!(pool.take().err(), Some(CameraError::Timeout));
    }

    #[test]
    fn test_sec_051_set_len_zeroes_slack_space() {
        let mut pool: FramePool<1, 16> = FramePool::new(GrabMode::WhenEmpty);
        let mut guard = pool
            .write_and_lease(160, 120, PixelFormat::Grayscale, 10, |buf| {
                buf[..8].fill(0xAA);
                Ok(4)
            })
            .expect("lease");
        assert_eq!(guard.len(), 4);
        guard.set_len(8);
        assert_eq!(guard.len(), 8);
        // Bytes 4..8 must have been zeroed by set_len
        assert_eq!(&guard.as_slice()[4..8], &[0, 0, 0, 0]);
    }

    #[test]
    fn test_logic_054_raw_buffer_mut_none_when_leased() {
        let mut pool: FramePool<2, 16> = FramePool::new(GrabMode::WhenEmpty);
        pool.states[0].store(SLOT_LEASED, Ordering::Release);
        assert!(pool.raw_buffer_mut(0).is_none());
        assert!(pool.raw_buffer_mut(1).is_some());

        // M-197: SLOT_READY and SLOT_WRITING must also reject raw_buffer_mut.
        pool.states[0].store(SLOT_READY, Ordering::Release);
        assert!(pool.raw_buffer_mut(0).is_none());
        pool.states[0].store(SLOT_WRITING, Ordering::Release);
        assert!(pool.raw_buffer_mut(0).is_none());

        pool.states[0].store(SLOT_FREE, Ordering::Release);
        assert!(pool.raw_buffer_mut(0).is_some());
    }

    #[test]
    fn test_logic_052_find_target_slot_round_robin() {
        let mut pool: FramePool<4, 16> = FramePool::new(GrabMode::Latest);
        let s0 = pool
            .push_frame(&[1], 10, 10, PixelFormat::Grayscale, 1)
            .expect("push_frame");
        let s1 = pool
            .push_frame(&[2], 10, 10, PixelFormat::Grayscale, 2)
            .expect("push_frame");
        assert_ne!(s0, s1);
    }

    #[test]
    fn test_m118_return_all_recovers_mem_forget_leased_slot_and_zeroes() {
        let mut pool: FramePool<2, 16> = FramePool::new(GrabMode::WhenEmpty);
        pool.push_frame(&[0xAA, 0xBB, 0xCC, 0xDD], 160, 120, PixelFormat::Jpeg, 10)
            .expect("push frame");

        // Simulate a leaked FrameGuard via core::mem::forget.
        let leaked = pool.take().expect("take frame");
        assert_eq!(leaked.slot_index(), 0);
        core::mem::forget(leaked);

        // Slot 0 is stuck in SLOT_LEASED before return_all().
        assert_eq!(pool.free_count(), 1);
        assert_eq!(pool.states[0].load(Ordering::Acquire), SLOT_LEASED);

        // return_all(&mut self) statically proves zero live guards exist and
        // MUST reclaim SLOT_LEASED slots and zero buffer residue.
        pool.return_all();
        assert_eq!(pool.free_count(), 2);
        assert_eq!(pool.available_frames(), 0);
        assert_eq!(pool.states[0].load(Ordering::Acquire), SLOT_FREE);
        assert_eq!(pool.meta[0].len, 0);
        assert_eq!(pool.buffers[0].0, [0u8; 16]);
    }

    #[test]
    fn test_m118_write_slot_guard_drop_clears_slot_on_unwind() {
        let mut pool: FramePool<1, 16> = FramePool::new(GrabMode::Latest);
        pool.push_frame(&[0x11, 0x22, 0x33, 0x44], 160, 120, PixelFormat::Jpeg, 10)
            .expect("push ready frame");
        assert_eq!(pool.available_frames(), 1);

        // Atomically reserve the only slot (evicting SLOT_READY ->
        // SLOT_WRITING), simulate partial write, and drop WriteSlotGuard
        // uncommitted (exact unwind path).
        let idx = pool.reserve_write_slot().expect("reserve latest slot");
        assert_eq!(idx, 0);
        assert_eq!(pool.states[0].load(Ordering::Acquire), SLOT_WRITING);
        {
            let guard = WriteSlotGuard {
                state: &pool.states[0],
                buf: &mut pool.buffers[0].0,
                meta_len: &mut pool.meta[0].len,
                committed: false,
            };
            guard.buf[0] = 0xDE;
            guard.buf[1] = 0xAD;
            // Drop without setting committed = true.
        }

        assert_eq!(pool.states[0].load(Ordering::Acquire), SLOT_FREE);
        assert_eq!(pool.available_frames(), 0);
        assert_eq!(pool.free_count(), 1);
        assert_eq!(pool.meta[0].len, 0);
        assert_eq!(pool.buffers[0].0, [0u8; 16]);
    }

    #[test]
    fn test_m118_cas_reservation_and_no_cursor_advance_on_nomem() {
        let mut pool: FramePool<2, 16> = FramePool::new(GrabMode::WhenEmpty);
        pool.push_frame(&[1, 2], 10, 10, PixelFormat::Grayscale, 1)
            .expect("push 0");
        pool.push_frame(&[3, 4], 10, 10, PixelFormat::Grayscale, 2)
            .expect("push 1");

        let cursor_before = pool.write_cursor.load(Ordering::Relaxed);
        assert_eq!(pool.find_target_slot(), Err(CameraError::NoMem));
        assert_eq!(
            pool.write_cursor.load(Ordering::Relaxed),
            cursor_before,
            "find_target_slot must not advance write_cursor on NoMem"
        );
        assert_eq!(
            pool.push_frame(&[5, 6], 10, 10, PixelFormat::Grayscale, 3),
            Err(CameraError::NoMem)
        );
        assert_eq!(
            pool.write_cursor.load(Ordering::Relaxed),
            cursor_before,
            "push_frame must not advance write_cursor on NoMem"
        );

        // Verify CAS reservation prevents double-claiming the same slot.
        pool.return_all();
        let r0 = pool.reserve_write_slot().expect("first CAS claim");
        let r1 = pool.reserve_write_slot().expect("second CAS claim");
        assert_ne!(r0, r1);
        assert_eq!(pool.reserve_write_slot(), Err(CameraError::NoMem));
        assert_eq!(pool.states[r0].load(Ordering::Acquire), SLOT_WRITING);
        assert_eq!(pool.states[r1].load(Ordering::Acquire), SLOT_WRITING);
        pool.return_all();
    }

    #[test]
    fn test_m197_frame_guard_drop_zeroizes_and_raw_buffer_bounds() {
        let mut pool: FramePool<2, 16> = FramePool::new(GrabMode::WhenEmpty);

        // Free slots must not be exposed by raw_buffer().
        assert!(pool.raw_buffer(0).is_none());

        let idx = pool
            .push_frame(&[0xFE, 0xED, 0xFA, 0xCE], 160, 120, PixelFormat::Jpeg, 50)
            .expect("push frame");
        assert_eq!(pool.raw_buffer(idx), Some(&[0xFE, 0xED, 0xFA, 0xCE][..]));
        // Ready slot must reject raw_buffer_mut().
        assert!(pool.raw_buffer_mut(idx).is_none());

        {
            let guard = pool.take().expect("lease ready frame");
            assert_eq!(guard.as_slice(), &[0xFE, 0xED, 0xFA, 0xCE]);
        }

        // After FrameGuard drops, buffer bytes MUST be zeroed (SEC-051) and
        // raw_buffer() must return None because slot is SLOT_FREE.
        assert!(pool.raw_buffer(idx).is_none());
        assert_eq!(pool.buffers[idx].0, [0u8; 16]);
    }
}
