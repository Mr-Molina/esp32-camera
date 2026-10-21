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
        let mut free_idx = None;
        let mut i = 0;
        while i < COUNT {
            if self.states[i].load(Ordering::Acquire) == SLOT_FREE {
                free_idx = Some(i);
                break;
            }
            i += 1;
        }

        match self.grab_mode {
            GrabMode::WhenEmpty => free_idx.ok_or(CameraError::NoMem),
            GrabMode::Latest => {
                if let Some(idx) = free_idx {
                    // In Latest mode, invalidate older ready slots so the new frame is the sole latest ready frame
                    let mut j = 0;
                    while j < COUNT {
                        if self.states[j].load(Ordering::Acquire) == SLOT_READY {
                            self.states[j].store(SLOT_FREE, Ordering::Release);
                        }
                        j += 1;
                    }
                    Ok(idx)
                } else {
                    // Evict the oldest SLOT_READY slot (never evict a SLOT_LEASED slot)
                    let mut oldest_idx = None;
                    let mut oldest_seq = u64::MAX;
                    let mut j = 0;
                    while j < COUNT {
                        if self.states[j].load(Ordering::Acquire) == SLOT_READY
                            && self.meta[j].sequence < oldest_seq
                        {
                            oldest_seq = self.meta[j].sequence;
                            oldest_idx = Some(j);
                        }
                        j += 1;
                    }
                    if let Some(idx) = oldest_idx {
                        // Also mark any other READY slots free
                        let mut k = 0;
                        while k < COUNT {
                            if self.states[k].load(Ordering::Acquire) == SLOT_READY {
                                self.states[k].store(SLOT_FREE, Ordering::Release);
                            }
                            k += 1;
                        }
                        Ok(idx)
                    } else {
                        Err(CameraError::NoMem)
                    }
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
        let seq = self.next_seq;
        self.next_seq = self.next_seq.wrapping_add(1);

        self.buffers[idx][..payload.len()].copy_from_slice(payload);
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
        let written_len = writer(&mut self.buffers[idx])?;
        if written_len > BUF_CAP {
            return Err(CameraError::BufferTooSmall);
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
                let mut best_seq = u64::MAX;
                let mut i = 0;
                while i < COUNT {
                    if self.states[i].load(Ordering::Acquire) == SLOT_READY
                        && self.meta[i].sequence < best_seq
                    {
                        best_seq = self.meta[i].sequence;
                        chosen_idx = Some(i);
                    }
                    i += 1;
                }
            }
            GrabMode::Latest => {
                let mut best_seq = 0u64;
                let mut i = 0;
                while i < COUNT {
                    if self.states[i].load(Ordering::Acquire) == SLOT_READY
                        && self.meta[i].sequence >= best_seq
                    {
                        best_seq = self.meta[i].sequence;
                        chosen_idx = Some(i);
                    }
                    i += 1;
                }
                if let Some(win) = chosen_idx {
                    let mut j = 0;
                    while j < COUNT {
                        if j != win && self.states[j].load(Ordering::Acquire) == SLOT_READY {
                            self.states[j].store(SLOT_FREE, Ordering::Release);
                        }
                        j += 1;
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
}
