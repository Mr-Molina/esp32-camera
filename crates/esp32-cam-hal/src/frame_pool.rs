//! Static/PSRAM-backed `#![forbid(unsafe_code)]` frame pool (`FramePool<COUNT, BUF_CAP>`) and
//! RAII [`FrameGuard`] making `esp_camera_fb_return` memory leaks impossible by construction.

#![forbid(unsafe_code)]

use core::ops::Deref;
use core::sync::atomic::{AtomicU8, Ordering};
use esp32_cam_types::{CameraError, GrabMode, PixelFormat};

/// Slot state: buffer is free and available for DMA / capture write (`en == 1` in `cam_hal.c`).
pub const SLOT_FREE: u8 = 0;

/// Slot state: buffer holds a completed, verified frame waiting to be taken by the consumer.
pub const SLOT_READY: u8 = 1;

/// Slot state: buffer is currently leased out to a live [`FrameGuard`] and cannot be overwritten.
pub const SLOT_LEASED: u8 = 2;

/// Metadata associated with a captured frame buffer slot (`camera_fb_t` fields).
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
/// Implements [`Drop`] to automatically transition the slot state back to [`SLOT_FREE`],
/// guaranteeing at compile time that frame buffers are returned to [`FramePool`] when dropped.
#[derive(Debug)]
pub struct FrameGuard<'a> {
    data: &'a mut [u8],
    len: usize,
    meta: FrameMetadata,
    slot_index: usize,
    state_flag: &'a AtomicU8,
}

impl<'a> FrameGuard<'a> {
    /// Returns the valid captured frame payload slice (`&[u8]` of length `self.len()`).
    #[inline]
    pub fn as_slice(&self) -> &[u8] {
        &self.data[..self.len]
    }

    /// Returns a mutable slice of the valid captured frame payload (`&mut [u8]` of length `self.len()`).
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

    /// Truncates or updates the valid length of this frame (clamped to underlying capacity).
    #[inline]
    pub fn set_len(&mut self, new_len: usize) {
        self.len = new_len.min(self.data.len());
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
        self.state_flag.store(SLOT_FREE, Ordering::Release);
    }
}

/// Static/PSRAM-backed multi-slot frame buffer pool (`FramePool<COUNT, BUF_CAP>`).
///
/// Supports both [`GrabMode::WhenEmpty`] (preserves earliest unread frames; drops incoming frames
/// when full) and [`GrabMode::Latest`] (evicts older unleased [`SLOT_READY`] frames so the consumer
/// always receives the freshest frame).
#[derive(Debug)]
pub struct FramePool<const COUNT: usize, const BUF_CAP: usize> {
    buffers: [[u8; BUF_CAP]; COUNT],
    meta: [FrameMetadata; COUNT],
    states: [AtomicU8; COUNT],
    grab_mode: GrabMode,
    next_seq: u64,
}

impl<const COUNT: usize, const BUF_CAP: usize> Default for FramePool<COUNT, BUF_CAP> {
    fn default() -> Self {
        Self::new(GrabMode::WhenEmpty)
    }
}

impl<const COUNT: usize, const BUF_CAP: usize> FramePool<COUNT, BUF_CAP> {
    /// Creates a new [`FramePool`] with all `COUNT` slots initialized to [`SLOT_FREE`].
    pub fn new(grab_mode: GrabMode) -> Self {
        Self {
            buffers: [[0u8; BUF_CAP]; COUNT],
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

    /// Returns the number of completed frames currently waiting in [`SLOT_READY`] state
    /// (`esp_camera_available_frames`).
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

    /// Resets all unleased slots back to [`SLOT_FREE`] (`esp_camera_return_all` / `cam_give_all`).
    pub fn return_all(&mut self) {
        let mut i = 0;
        while i < COUNT {
            if self.states[i].load(Ordering::Acquire) != SLOT_LEASED {
                self.states[i].store(SLOT_FREE, Ordering::Release);
            }
            i += 1;
        }
    }

    /// Selects a slot index for writing a new frame according to `self.grab_mode`.
    fn select_write_slot(&mut self) -> Result<usize, CameraError> {
        if COUNT == 0 || BUF_CAP == 0 {
            return Err(CameraError::NoMem);
        }

        // First look for any SLOT_FREE buffer
        for i in 0..COUNT {
            if self.states[i].load(Ordering::Acquire) == SLOT_FREE {
                return Ok(i);
            }
        }

        match self.grab_mode {
            GrabMode::WhenEmpty => Err(CameraError::NoMem),
            GrabMode::Latest => {
                // In Latest mode, if no slot is free, reclaim ONLY the oldest SLOT_READY slot.
                // Existing ready slots are not wiped here; older ready slots are only drained after
                // write successfully finishes. Never evict a SLOT_LEASED slot.
                let mut oldest: Option<(u64, usize)> = None;
                for j in 0..COUNT {
                    if self.states[j].load(Ordering::Acquire) == SLOT_READY {
                        let seq = self.meta[j].sequence;
                        match oldest {
                            None => oldest = Some((seq, j)),
                            Some((best_seq, _)) if seq < best_seq => oldest = Some((seq, j)),
                            _ => {}
                        }
                    }
                }
                if let Some((_, idx)) = oldest {
                    Ok(idx)
                } else {
                    Err(CameraError::NoMem)
                }
            }
        }
    }

    /// Pushes a complete frame payload into the pool according to [`GrabMode`], returning the
    /// assigned slot index.
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
        let idx = self.select_write_slot()?;

        self.buffers[idx][..payload.len()].copy_from_slice(payload);

        // Only after payload is successfully written, drain older ready slots in GrabMode::Latest
        if self.grab_mode == GrabMode::Latest {
            for j in 0..COUNT {
                if j != idx && self.states[j].load(Ordering::Acquire) == SLOT_READY {
                    self.states[j].store(SLOT_FREE, Ordering::Release);
                }
            }
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

    /// Writes a frame in-place using a producer closure `writer(&mut [u8]) -> Result<usize, CameraError>`,
    /// transitions the slot to [`SLOT_LEASED`], and immediately returns the RAII [`FrameGuard`].
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
        let idx = self.select_write_slot()?;
        let written_len = match writer(&mut self.buffers[idx]) {
            Ok(len) => len,
            Err(e) => {
                self.meta[idx].len = 0;
                self.states[idx].store(SLOT_FREE, Ordering::Release);
                return Err(e);
            }
        };
        if written_len > BUF_CAP {
            self.meta[idx].len = 0;
            self.states[idx].store(SLOT_FREE, Ordering::Release);
            return Err(CameraError::BufferTooSmall);
        }

        // Only after writer succeeds and validates capacity, drain any older SLOT_READY slots in GrabMode::Latest
        if self.grab_mode == GrabMode::Latest {
            for j in 0..COUNT {
                if j != idx && self.states[j].load(Ordering::Acquire) == SLOT_READY {
                    self.states[j].store(SLOT_FREE, Ordering::Release);
                }
            }
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
            data: &mut self.buffers[idx],
            len: written_len,
            meta,
            slot_index: idx,
            state_flag: &self.states[idx],
        })
    }

    /// Takes the next available [`SLOT_READY`] frame from the pool and returns an RAII [`FrameGuard`].
    ///
    /// - In [`GrabMode::WhenEmpty`], returns the earliest ready frame (FIFO by `sequence`).
    /// - In [`GrabMode::Latest`], returns the newest ready frame (LIFO by `sequence`) and releases
    ///   any older ready frames.
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
                            Some((best_seq, _)) if seq < best_seq => oldest = Some((seq, i)),
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
                            Some((best_seq, _)) if seq >= best_seq => newest = Some((seq, i)),
                            _ => {}
                        }
                    }
                }
                if let Some((_, win)) = newest {
                    chosen_idx = Some(win);
                    for j in 0..COUNT {
                        if j != win && self.states[j].load(Ordering::Acquire) == SLOT_READY {
                            self.states[j].store(SLOT_FREE, Ordering::Release);
                        }
                    }
                }
            }
        }

        let idx = chosen_idx.ok_or(CameraError::Timeout)?;
        let meta = self.meta[idx];
        self.states[idx].store(SLOT_LEASED, Ordering::Release);

        Ok(FrameGuard {
            data: &mut self.buffers[idx],
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
        // Once `guard` drops out of scope, its slot is automatically returned to `SLOT_FREE`!
        assert_eq!(pool.free_count(), 2);
        assert_eq!(pool.available_frames(), 0);
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

        // 2. GrabMode::Latest overwrites older unleased frames so consumer gets the freshest
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

        // A failed writer must NOT wipe existing ready frames before writing succeeds!
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

        // In Latest mode with no free slots, write_and_lease reclaims the oldest ready slot (slot 0).
        // If writer fails after partially writing, slot 0 MUST become SLOT_FREE with len 0,
        // and must NOT be left in SLOT_READY with corrupted data.
        let res = pool.write_and_lease(160, 120, PixelFormat::Grayscale, 300, |buf| {
            buf[0] = 0xDE;
            buf[1] = 0xAD;
            Err(CameraError::CorruptedFrame)
        });
        assert_eq!(res.err(), Some(CameraError::CorruptedFrame));

        // Slot 0 was freed; no frames are ready and corrupted data cannot be taken.
        assert_eq!(pool.available_frames(), 0);
        assert_eq!(pool.free_count(), 1);
        assert_eq!(pool.take().err(), Some(CameraError::Timeout));
    }
}
