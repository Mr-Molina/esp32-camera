//! Shared DMA half-buffer drain for the polled ESP32 I2S0 capture (WP-06).
//!
//! Every capture path (the ll polled capture and both firmware capture loops)
//! consumes completed DMA half-buffers through [`HalfDrain`], so the
//! ownership, overrun and poisoning rules live in one place:
//!
//! - **FW-CORE-02 / LL-15.** A half is consumed only when the DMA reported it
//!   complete (`in_eof_des_addr`) and, with [`DrainGate::OwnerWriteback`],
//!   its last descriptor has been written back to CPU ownership.
//! - **LL-06.** A consumed half is filled with [`DMA_POISON_BYTE`] before it is
//!   re-armed, so bytes the DMA has not rewritten can never carry a stale JPEG
//!   marker from an earlier frame.
//! - **LL-07.** Halves completed since the last drain are counted from the
//!   `in_eof_des_addr` delta. More than `half_cnt - 1` pending halves means
//!   the ring lapped and fails the frame with [`CameraError::Overflow`];
//!   `IN_DSCR_ERR` / `IN_DSCR_EMPTY` abort the frame. The drain never passes
//!   the EOF descriptor.
//! - **LL-05.** At the end of a frame the current, partially written half is
//!   processed unconditionally by [`HalfDrain::drain_tail_half`].
//! - **CONV-10.** [`find_jpeg_eoi_tail`] bounds the frame-level EOI scan to the
//!   last chunk plus a margin.

use core::ops::ControlFlow;

use esp32_cam_types::CameraError;

use crate::dma_filter::DmaFilterKind;
use crate::dma_ring::{Lldesc, LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE};
use crate::esp32_i2s::{
    append_unpacked_jpeg_chunk, find_jpeg_eoi, I2S_INLINK_ADDR_MASK, I2S_IN_DSCR_EMPTY_INT_RAW,
    I2S_IN_DSCR_ERR_INT_RAW, I2S_IN_SUC_EOF_INT_BIT, JPEG_DMA_NODE_SIZE,
    JPEG_UNPACKED_HALF_BUFFER_SIZE,
};

/// Maximum number of halves one [`HalfDrain::drain_ready_halves`] call
/// consumes, so a single call has a bounded run time.
pub const MAX_DRAIN_HALVES: usize = 8;

/// Fill byte written over a consumed half before it is re-armed (LL-06).
///
/// `0x00` is not a JPEG marker byte, so neither `FF D8` nor `FF D9` can be
/// formed from poison alone, and a consumed-but-unwritten half shows up as a
/// zero run.
pub const DMA_POISON_BYTE: u8 = 0x00;

/// Margin added to the last chunk length by [`find_jpeg_eoi_tail`]: one more
/// unpacked half-buffer chunk plus one byte for an EOI straddling the chunk
/// boundary.
pub const JPEG_EOI_TAIL_MARGIN: usize = JPEG_UNPACKED_HALF_BUFFER_SIZE + 1;

/// Access to the DMA status registers a drain needs.
///
/// Implemented by [`crate::I2s0DmaStatus`] for the real (or shadow) I2S0
/// register block, and by mocks in host tests.
pub trait DmaStatus {
    /// Reads the raw interrupt status (`I2S_INT_RAW_REG`).
    fn int_raw(&mut self) -> u32;
    /// Clears the given raw interrupt bits (`I2S_INT_CLR_REG`, write-1-to-clear).
    fn clear_int(&mut self, mask: u32);
    /// Reads the address of the descriptor that carried the last EOF
    /// (`I2S_IN_EOF_DES_ADDR_REG`).
    fn eof_desc_addr(&mut self) -> u32;
    /// Orders DMA-visible memory accesses (`memw` on Xtensa). Called after a
    /// new EOF is observed and before SRAM reads, and between poisoning a half
    /// and re-arming its descriptors.
    fn barrier(&mut self) {
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    }
}

/// Callbacks a polled capture loop makes into its caller (LL-15), so the
/// firmware can service the network and watchdog while the ll loop runs.
pub trait CaptureHooks {
    /// Called periodically while the capture loop is waiting for DMA progress.
    fn idle(&mut self);
    /// Returns a monotonic timestamp in microseconds, used for diagnostics.
    fn now_us(&mut self) -> u64;
}

/// [`CaptureHooks`] that do nothing and report time `0`.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct NoCaptureHooks;

impl CaptureHooks for NoCaptureHooks {
    #[inline]
    fn idle(&mut self) {}

    #[inline]
    fn now_us(&mut self) -> u64 {
        0
    }
}

/// How [`HalfDrain::drain_ready_halves`] decides that a pending half may be
/// consumed, in addition to the `in_eof_des_addr` position.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum DrainGate {
    /// Also require the half's last descriptor to be written back with
    /// `owner == 0`. Relies on the DMA writing back descriptors (LL Open
    /// question 1).
    OwnerWriteback,
    /// Trust the `in_eof_des_addr` position alone.
    EofPosition,
}

/// State of the shared half-buffer drain over a circular ring of
/// `2 * half_cnt` descriptors of `node_size` bytes each (two descriptors per
/// half-buffer).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct HalfDrain {
    half_cnt: usize,
    node_size: usize,
    filter: DmaFilterKind,
    gate: DrainGate,
    half_idx: usize,
    pending: usize,
    last_eof_half: usize,
    int_raw_seen: u32,
    halves_drained: u32,
}

impl HalfDrain {
    /// Creates a drain for `half_cnt` half-buffers of `2 * node_size` DMA bytes.
    ///
    /// Returns [`CameraError::InvalidArg`] unless `half_cnt >= 2` and
    /// `node_size` is non-zero, a multiple of 4 and at most
    /// [`LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE`].
    pub fn new(
        half_cnt: usize,
        node_size: usize,
        filter: DmaFilterKind,
        gate: DrainGate,
    ) -> Result<Self, CameraError> {
        if half_cnt < 2
            || node_size == 0
            || node_size > LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE
            || !node_size.is_multiple_of(4)
        {
            return Err(CameraError::InvalidArg);
        }
        Ok(Self {
            half_cnt,
            node_size,
            filter,
            gate,
            half_idx: 0,
            pending: 0,
            last_eof_half: half_cnt - 1,
            int_raw_seen: 0,
            halves_drained: 0,
        })
    }

    /// Resets the drain for a new DMA start at descriptor 0. Call it whenever
    /// the ring is rebuilt and the DMA is (re)started.
    pub fn reset(&mut self) {
        self.half_idx = 0;
        self.pending = 0;
        self.last_eof_half = self.half_cnt - 1;
        self.int_raw_seen = 0;
        self.halves_drained = 0;
    }

    /// Index of the next half-buffer to consume.
    #[inline]
    pub const fn half_idx(&self) -> usize {
        self.half_idx
    }

    /// Halves reported complete by the DMA and not consumed yet.
    #[inline]
    pub const fn pending(&self) -> usize {
        self.pending
    }

    /// Number of half-buffers.
    #[inline]
    pub const fn half_cnt(&self) -> usize {
        self.half_cnt
    }

    /// DMA bytes per half-buffer (`2 * node_size`).
    #[inline]
    pub const fn half_bytes(&self) -> usize {
        2 * self.node_size
    }

    /// Bitwise OR of every raw interrupt status read since the last reset.
    #[inline]
    pub const fn int_raw_seen(&self) -> u32 {
        self.int_raw_seen
    }

    /// Halves consumed since the last reset (completed and tail halves).
    #[inline]
    pub const fn halves_drained(&self) -> u32 {
        self.halves_drained
    }

    fn check_ring(&self, descriptors: &[Lldesc], dma_buf: &[u8]) -> Result<(), CameraError> {
        let need_buf = self
            .half_cnt
            .checked_mul(self.half_bytes())
            .ok_or(CameraError::Overflow)?;
        if descriptors.len() < 2 * self.half_cnt || dma_buf.len() < need_buf {
            return Err(CameraError::InvalidArg);
        }
        Ok(())
    }

    /// Maps an `in_eof_des_addr` value to the half it completes. Only the
    /// last descriptor of a half can carry the EOF (`rx_eof_num` is one
    /// half-buffer), so any other address means a broken ring.
    fn half_of_eof_addr(&self, descriptors: &[Lldesc], addr: u32) -> Option<usize> {
        let mut h = 0usize;
        while h < self.half_cnt {
            if descriptors[2 * h + 1].link_addr_20bit() == addr {
                return Some(h);
            }
            h += 1;
        }
        None
    }

    /// Reads the DMA status and accounts newly completed halves (LL-07).
    ///
    /// - `IN_DSCR_ERR` fails with [`CameraError::DmaRingCorrupted`] and
    ///   `IN_DSCR_EMPTY` with [`CameraError::Overflow`] (frame abort).
    /// - On `IN_SUC_EOF` the bit is cleared *before* `in_eof_des_addr` is
    ///   read, so an EOF landing in between is never lost: it re-raises the
    ///   bit and the address already covers it.
    /// - The completed-half count advances by the EOF position delta, modulo
    ///   the ring. More than `half_cnt - 1` pending halves means the DMA is
    ///   overwriting the oldest unconsumed half: [`CameraError::Overflow`].
    ///   (A lap of exactly `half_cnt` halves between two observations leaves
    ///   the position unchanged and cannot be told apart from a re-raised
    ///   bit; the owner gate and the poll rate cover that case.)
    ///
    /// Returns the number of halves newly reported complete.
    pub fn observe<S: DmaStatus>(
        &mut self,
        descriptors: &[Lldesc],
        status: &mut S,
    ) -> Result<usize, CameraError> {
        if descriptors.len() < 2 * self.half_cnt {
            return Err(CameraError::InvalidArg);
        }
        let raw = status.int_raw();
        self.int_raw_seen |= raw;

        let err = raw & (I2S_IN_DSCR_ERR_INT_RAW | I2S_IN_DSCR_EMPTY_INT_RAW);
        if err != 0 {
            status.clear_int(err);
            return Err(if (err & I2S_IN_DSCR_ERR_INT_RAW) != 0 {
                CameraError::DmaRingCorrupted
            } else {
                CameraError::Overflow
            });
        }
        if (raw & I2S_IN_SUC_EOF_INT_BIT) == 0 {
            return Ok(0);
        }

        status.clear_int(I2S_IN_SUC_EOF_INT_BIT);
        let addr = status.eof_desc_addr() & I2S_INLINK_ADDR_MASK;
        // Order the status reads before the SRAM reads of the halves.
        status.barrier();

        let h = self
            .half_of_eof_addr(descriptors, addr)
            .ok_or(CameraError::DmaRingCorrupted)?;
        let delta = (h + self.half_cnt - self.last_eof_half) % self.half_cnt;
        self.last_eof_half = h;
        self.pending += delta;
        if self.pending > self.half_cnt - 1 {
            return Err(CameraError::Overflow);
        }
        Ok(delta)
    }

    /// Filters the current half into `scratch`, poisons it (LL-06) and
    /// re-arms both of its descriptors with one packed volatile store each.
    fn consume_half<S: DmaStatus>(
        &mut self,
        descriptors: &mut [Lldesc],
        dma_buf: &mut [u8],
        status: &mut S,
        scratch: &mut [u8],
    ) -> usize {
        let h = self.half_idx;
        let half_bytes = self.half_bytes();
        let off = h * half_bytes;
        let half = &mut dma_buf[off..off + half_bytes];
        let n = self.filter.apply(scratch, half);
        half.fill(DMA_POISON_BYTE);
        // The poison must reach SRAM before the DMA may write the half again.
        status.barrier();
        let flags = Lldesc::pack_flags(self.node_size as u16, 0, 0, false, false, true);
        descriptors[2 * h].store_flags(flags);
        descriptors[2 * h + 1].store_flags(flags);
        self.half_idx = (h + 1) % self.half_cnt;
        self.halves_drained = self.halves_drained.wrapping_add(1);
        n
    }

    /// Consumes every half that is complete, oldest first (FW-CORE-02,
    /// LL-15).
    ///
    /// Calls [`Self::observe`] first, then consumes at most
    /// `min(half_cnt, MAX_DRAIN_HALVES)` halves, stopping at the EOF
    /// descriptor (`pending == 0`) and, with [`DrainGate::OwnerWriteback`], at
    /// the first half whose last descriptor is still DMA-owned. Each half is
    /// filtered into `scratch`, poisoned, re-armed, and only then handed to
    /// `sink`; [`ControlFlow::Break`] from the sink stops the drain after that
    /// half.
    ///
    /// Returns the number of halves consumed. Fails with the errors of
    /// [`Self::observe`] (overflow, descriptor errors), or with
    /// [`CameraError::InvalidArg`] when the ring has fewer than
    /// `2 * half_cnt` descriptors or `dma_buf` holds fewer than `half_cnt`
    /// halves.
    pub fn drain_ready_halves<S, F>(
        &mut self,
        descriptors: &mut [Lldesc],
        dma_buf: &mut [u8],
        status: &mut S,
        scratch: &mut [u8],
        mut sink: F,
    ) -> Result<usize, CameraError>
    where
        S: DmaStatus,
        F: FnMut(&[u8]) -> ControlFlow<()>,
    {
        self.check_ring(descriptors, dma_buf)?;
        self.observe(descriptors, status)?;

        let limit = self.half_cnt.min(MAX_DRAIN_HALVES);
        let mut drained = 0usize;
        while drained < limit && self.pending > 0 {
            if self.gate == DrainGate::OwnerWriteback && descriptors[2 * self.half_idx + 1].owner()
            {
                break;
            }
            let n = self.consume_half(descriptors, dma_buf, status, scratch);
            self.pending -= 1;
            drained += 1;
            if sink(&scratch[..n]).is_break() {
                break;
            }
        }
        Ok(drained)
    }

    /// Consumes the current half unconditionally (LL-05): filters it into
    /// `scratch`, poisons it and re-arms it.
    ///
    /// Call this only at the end of a frame, after the DMA has been stopped
    /// and [`Self::drain_ready_halves`] has consumed the completed halves.
    /// Bytes the DMA did not write read back as [`DMA_POISON_BYTE`].
    ///
    /// Returns the number of filtered bytes written to `scratch`.
    pub fn drain_tail_half<S: DmaStatus>(
        &mut self,
        descriptors: &mut [Lldesc],
        dma_buf: &mut [u8],
        status: &mut S,
        scratch: &mut [u8],
    ) -> Result<usize, CameraError> {
        self.check_ring(descriptors, dma_buf)?;
        status.barrier();
        let n = self.consume_half(descriptors, dma_buf, status, scratch);
        self.pending = self.pending.saturating_sub(1);
        Ok(n)
    }

    /// Filters the current half into `scratch` without consuming, poisoning
    /// or re-arming it. Because consumed halves are poisoned, any JPEG marker
    /// found here was written during the current DMA run.
    pub fn peek_current_half(
        &self,
        dma_buf: &[u8],
        scratch: &mut [u8],
    ) -> Result<usize, CameraError> {
        let half_bytes = self.half_bytes();
        let off = self.half_idx * half_bytes;
        // LL-03: read only what the DMA wrote before this point.
        crate::esp32_i2s::dma_fence(core::sync::atomic::Ordering::Acquire);
        let half = dma_buf
            .get(off..off + half_bytes)
            .ok_or(CameraError::InvalidArg)?;
        Ok(self.filter.apply(scratch, half))
    }
}

/// Start of the bounded EOI probe window for a frame of `frame_len` bytes
/// whose last chunk is at most `last_chunk_len` bytes (CONV-10): the last
/// chunk plus [`JPEG_EOI_TAIL_MARGIN`].
#[inline]
#[must_use]
pub const fn jpeg_eoi_tail_start(frame_len: usize, last_chunk_len: usize) -> usize {
    frame_len.saturating_sub(last_chunk_len.saturating_add(JPEG_EOI_TAIL_MARGIN))
}

/// Finds the last JPEG EOI marker (`FF D9`) in the tail of `frame` (CONV-10).
///
/// Scans backward over `frame[jpeg_eoi_tail_start(frame.len(),
/// last_chunk_len)..]` only, so a frame without an EOI costs at most that
/// window instead of the whole frame. Returns the offset of the `0xFF` byte.
#[must_use]
pub fn find_jpeg_eoi_tail(frame: &[u8], last_chunk_len: usize) -> Option<usize> {
    find_jpeg_eoi_tail_by(frame.len(), last_chunk_len, |i| frame[i])
}

fn find_jpeg_eoi_tail_by<F: FnMut(usize) -> u8>(
    len: usize,
    last_chunk_len: usize,
    mut at: F,
) -> Option<usize> {
    let start = jpeg_eoi_tail_start(len, last_chunk_len);
    if len < start + 2 {
        return None;
    }
    let mut i = len - 2;
    loop {
        if at(i + 1) == 0xD9 && at(i) == 0xFF {
            return Some(i);
        }
        if i == start {
            return None;
        }
        i -= 1;
    }
}

/// Outcome of one [`PolledJpegCapture::poll`] step.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CaptureStep {
    /// Keep polling.
    Pending,
    /// A complete `SOI..=EOI` frame of this many bytes is in `jpeg_out`. Stop
    /// the DMA.
    Complete(usize),
    /// The frame ended (debounced VSYNC end, or an idle peek saw the EOI).
    /// Stop the DMA, then call [`PolledJpegCapture::finish`].
    FrameEnded,
    /// The frame failed. Stop the DMA.
    Failed(CameraError),
}

/// Diagnostics gathered by [`PolledJpegCapture`] (LL-15).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct CaptureDiag {
    /// Halves consumed, including the tail half.
    pub halves_drained: u32,
    /// Bitwise OR of every raw interrupt status read.
    pub int_raw_seen: u32,
    /// Idle peeks of the current half.
    pub idle_peeks: u32,
    /// [`CaptureHooks::now_us`] when the first half was consumed.
    pub first_half_us: Option<u64>,
    /// [`CaptureHooks::now_us`] when the frame end was detected.
    pub frame_end_us: Option<u64>,
}

/// Host-testable state machine behind the polled JPEG capture.
#[derive(Debug, Clone)]
pub struct PolledJpegCapture {
    drain: HalfDrain,
    chunk: [u8; JPEG_UNPACKED_HALF_BUFFER_SIZE],
    written: usize,
    soi_found: bool,
    prev_tail: [u8; 2],
    prev_tail_len: usize,
    seen_active_video: bool,
    idle_polls: u32,
    diag: CaptureDiag,
}

impl PolledJpegCapture {
    /// Idle polls (no half consumed) between peeks at the current half.
    pub const IDLE_PEEK_POLLS: u32 = 2048;
    /// Idle polls between [`CaptureHooks::idle`] calls.
    pub const IDLE_HOOK_POLLS: u32 = 64;

    /// Creates a capture over `half_cnt` JPEG half-buffers
    /// ([`JPEG_DMA_NODE_SIZE`] bytes per descriptor).
    pub fn new(
        half_cnt: usize,
        filter: DmaFilterKind,
        gate: DrainGate,
    ) -> Result<Self, CameraError> {
        let drain = HalfDrain::new(half_cnt, JPEG_DMA_NODE_SIZE, filter, gate)?;
        Ok(Self {
            drain,
            chunk: [0; JPEG_UNPACKED_HALF_BUFFER_SIZE],
            written: 0,
            soi_found: false,
            prev_tail: [0; 2],
            prev_tail_len: 0,
            seen_active_video: false,
            idle_polls: 0,
            diag: CaptureDiag::default(),
        })
    }

    /// Diagnostics gathered so far.
    #[inline]
    pub fn diag(&self) -> CaptureDiag {
        let mut d = self.diag;
        d.halves_drained = self.drain.halves_drained();
        d.int_raw_seen = self.drain.int_raw_seen();
        d
    }

    /// Drains completed halves into `jpeg_out`. Returns the number of halves
    /// consumed and, once the EOI was appended, the frame length.
    fn drain_into<S: DmaStatus>(
        &mut self,
        descriptors: &mut [Lldesc],
        dma_buf: &mut [u8],
        status: &mut S,
        jpeg_out: &mut [u8],
    ) -> Result<(usize, Option<usize>), CameraError> {
        let Self {
            drain,
            chunk,
            written,
            soi_found,
            prev_tail,
            prev_tail_len,
            ..
        } = self;
        let mut outcome: Result<Option<usize>, CameraError> = Ok(None);
        let n =
            drain.drain_ready_halves(descriptors, dma_buf, status, chunk.as_mut_slice(), |c| {
                match append_unpacked_jpeg_chunk(
                    c,
                    jpeg_out,
                    written,
                    soi_found,
                    prev_tail,
                    prev_tail_len,
                ) {
                    Ok(None) => ControlFlow::Continue(()),
                    done => {
                        outcome = done;
                        ControlFlow::Break(())
                    }
                }
            })?;
        Ok((n, outcome?))
    }

    fn note_progress<H: CaptureHooks>(&mut self, hooks: &mut H) {
        self.idle_polls = 0;
        if self.diag.first_half_us.is_none() {
            self.diag.first_half_us = Some(hooks.now_us());
        }
    }

    /// Runs one poll step: drains completed halves into `jpeg_out`, then
    /// checks for the end of the frame.
    ///
    /// - `vsync_active` returns `true` while the sensor is inside the frame
    ///   (VSYNC at its active level). The frame ends after active video was
    ///   seen and four consecutive samples read inactive (LL-05 debounce).
    /// - Every [`Self::IDLE_HOOK_POLLS`] polls without progress,
    ///   [`CaptureHooks::idle`] is called. Every [`Self::IDLE_PEEK_POLLS`]
    ///   such polls the current half is peeked; an EOI there ends the frame
    ///   early.
    ///
    /// On [`CaptureStep::FrameEnded`] the caller stops the DMA and calls
    /// [`Self::finish`]; on [`CaptureStep::Complete`] or
    /// [`CaptureStep::Failed`] it stops the DMA.
    pub fn poll<S, H, V>(
        &mut self,
        descriptors: &mut [Lldesc],
        dma_buf: &mut [u8],
        status: &mut S,
        jpeg_out: &mut [u8],
        vsync_active: &mut V,
        hooks: &mut H,
    ) -> CaptureStep
    where
        S: DmaStatus,
        H: CaptureHooks,
        V: FnMut() -> bool,
    {
        if vsync_active() {
            self.seen_active_video = true;
        }

        match self.drain_into(descriptors, dma_buf, status, jpeg_out) {
            Err(e) => return CaptureStep::Failed(e),
            Ok((_, Some(len))) => {
                self.note_progress(hooks);
                return CaptureStep::Complete(len);
            }
            Ok((n, None)) if n > 0 => {
                self.note_progress(hooks);
                return CaptureStep::Pending;
            }
            Ok(_) => {}
        }

        self.idle_polls = self.idle_polls.wrapping_add(1);
        if self.idle_polls.is_multiple_of(Self::IDLE_HOOK_POLLS) {
            hooks.idle();
        }

        // LL-05: a debounced VSYNC end finishes the frame whatever the
        // current half's descriptor says; poisoning rules out stale markers.
        let vsync_ended = self.seen_active_video
            && !vsync_active()
            && !vsync_active()
            && !vsync_active()
            && !vsync_active();
        if vsync_ended {
            self.diag.frame_end_us = Some(hooks.now_us());
            return CaptureStep::FrameEnded;
        }

        if self.idle_polls >= Self::IDLE_PEEK_POLLS {
            self.idle_polls = 0;
            self.diag.idle_peeks = self.diag.idle_peeks.wrapping_add(1);
            let n = match self.drain.peek_current_half(dma_buf, &mut self.chunk) {
                Ok(n) => n,
                Err(e) => return CaptureStep::Failed(e),
            };
            let peeked = &self.chunk[..n];
            let boundary_eoi = self.written > 0
                && jpeg_out.get(self.written - 1) == Some(&0xFF)
                && peeked.first() == Some(&0xD9);
            if boundary_eoi || find_jpeg_eoi(peeked).is_some() {
                self.diag.frame_end_us = Some(hooks.now_us());
                return CaptureStep::FrameEnded;
            }
        }
        CaptureStep::Pending
    }

    /// Completes a frame after [`CaptureStep::FrameEnded`] and a DMA stop.
    ///
    /// Halves the EOF position reported complete but the owner gate held back
    /// are consumed first, then the current, partially written half is
    /// consumed unconditionally (LL-05). Returns the `SOI..=EOI` length, or
    /// [`CameraError::CorruptedFrame`] when no EOI follows the SOI.
    pub fn finish<S: DmaStatus>(
        &mut self,
        descriptors: &mut [Lldesc],
        dma_buf: &mut [u8],
        status: &mut S,
        jpeg_out: &mut [u8],
    ) -> Result<usize, CameraError> {
        let Self {
            drain,
            chunk,
            written,
            soi_found,
            prev_tail,
            prev_tail_len,
            ..
        } = self;
        loop {
            let more = drain.pending() > 0;
            let n = drain.drain_tail_half(descriptors, dma_buf, status, chunk.as_mut_slice())?;
            if let Some(len) = append_unpacked_jpeg_chunk(
                &chunk[..n],
                jpeg_out,
                written,
                soi_found,
                prev_tail,
                prev_tail_len,
            )? {
                return Ok(len);
            }
            if !more {
                return Err(CameraError::CorruptedFrame);
            }
        }
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use crate::dma_filter::DmaElement;
    use crate::dma_ring::build_circular_ring;

    const HALVES: usize = 8;
    const NODE: usize = JPEG_DMA_NODE_SIZE;
    const HALF: usize = 2 * NODE;
    const CHUNK: usize = HALF / 4;

    #[derive(Default)]
    struct MockDma {
        int_raw: u32,
        eof_addr: u32,
        barriers: u32,
    }

    impl DmaStatus for MockDma {
        fn int_raw(&mut self) -> u32 {
            self.int_raw
        }
        fn clear_int(&mut self, mask: u32) {
            self.int_raw &= !mask;
        }
        fn eof_desc_addr(&mut self) -> u32 {
            self.eof_addr
        }
        fn barrier(&mut self) {
            self.barriers += 1;
        }
    }

    #[repr(C, align(4))]
    struct Ring<const N: usize, const B: usize> {
        desc: [Lldesc; N],
        buf: [u8; B],
    }

    type Ring8 = Ring<{ 2 * HALVES }, { HALVES * HALF }>;

    impl<const N: usize, const B: usize> Ring<N, B> {
        fn new() -> Self {
            let mut r = Self {
                desc: [Lldesc::EMPTY; N],
                buf: [DMA_POISON_BYTE; B],
            };
            r.build();
            r
        }

        /// Re-initializes the descriptors (as every capture start does) but
        /// keeps the buffer contents, like the real static ring.
        fn build(&mut self) {
            build_circular_ring(&mut self.desc, &mut self.buf, NODE).expect("ring builds");
        }

        /// The DMA writes unpacked JPEG `bytes` into half `h` at unpacked
        /// offset `at`.
        fn dma_write(&mut self, h: usize, at: usize, bytes: &[u8]) {
            for (i, &b) in bytes.iter().enumerate() {
                let off = h * HALF + (at + i) * 4;
                self.buf[off..off + 4].copy_from_slice(&DmaElement::new(b, 0).0);
            }
        }

        /// Fills half `h` completely with unpacked byte `b`.
        fn dma_fill(&mut self, h: usize, b: u8) {
            self.dma_write(h, 0, &[b; CHUNK]);
        }

        /// The DMA completes half `h`: optional owner write-back on both
        /// descriptors, EOF on the last one, `IN_SUC_EOF` raised with the
        /// EOF descriptor address.
        fn dma_complete(&mut self, h: usize, dma: &mut MockDma, writeback: bool) {
            if writeback {
                for d in [2 * h, 2 * h + 1] {
                    self.desc[d].set_length(NODE as u16);
                    self.desc[d].set_owner(false);
                }
            }
            self.desc[2 * h + 1].set_eof(true);
            dma.eof_addr = self.desc[2 * h + 1].link_addr_20bit();
            dma.int_raw |= I2S_IN_SUC_EOF_INT_BIT;
        }

        fn half(&self, h: usize) -> &[u8] {
            &self.buf[h * HALF..(h + 1) * HALF]
        }
    }

    fn rearmed_flags() -> u32 {
        Lldesc::pack_flags(NODE as u16, 0, 0, false, false, true)
    }

    fn drain8(gate: DrainGate) -> HalfDrain {
        HalfDrain::new(HALVES, NODE, DmaFilterKind::Jpeg, gate).expect("drain")
    }

    /// Drains and records the first byte of every chunk handed to the sink.
    fn drain_collect(
        drain: &mut HalfDrain,
        ring: &mut Ring8,
        dma: &mut MockDma,
    ) -> (Result<usize, CameraError>, [u8; 16], usize) {
        let mut scratch = [0u8; CHUNK];
        let mut seen = [0u8; 16];
        let mut n = 0usize;
        let r = drain.drain_ready_halves(&mut ring.desc, &mut ring.buf, dma, &mut scratch, |c| {
            assert_eq!(c.len(), CHUNK);
            seen[n] = c[0];
            n += 1;
            ControlFlow::Continue(())
        });
        (r, seen, n)
    }

    #[test]
    fn new_rejects_invalid_geometry() {
        let j = DmaFilterKind::Jpeg;
        let g = DrainGate::OwnerWriteback;
        assert_eq!(HalfDrain::new(1, NODE, j, g), Err(CameraError::InvalidArg));
        assert_eq!(HalfDrain::new(8, 0, j, g), Err(CameraError::InvalidArg));
        assert_eq!(HalfDrain::new(8, 2046, j, g), Err(CameraError::InvalidArg));
        assert_eq!(HalfDrain::new(8, 4096, j, g), Err(CameraError::InvalidArg));
        let d = HalfDrain::new(8, NODE, j, g).expect("valid");
        assert_eq!(d.half_cnt(), 8);
        assert_eq!(d.half_bytes(), HALF);
        assert_eq!(d.half_idx(), 0);
        assert_eq!(d.pending(), 0);
    }

    #[test]
    fn drain_rejects_short_ring() {
        let mut drain = drain8(DrainGate::OwnerWriteback);
        let mut ring = Ring8::new();
        let mut dma = MockDma::default();
        let mut scratch = [0u8; CHUNK];
        let r = drain.drain_ready_halves(
            &mut ring.desc[..14],
            &mut ring.buf,
            &mut dma,
            &mut scratch,
            |_| ControlFlow::Continue(()),
        );
        assert_eq!(r, Err(CameraError::InvalidArg));
        let r = drain.drain_ready_halves(
            &mut ring.desc,
            &mut ring.buf[..7 * HALF],
            &mut dma,
            &mut scratch,
            |_| ControlFlow::Continue(()),
        );
        assert_eq!(r, Err(CameraError::InvalidArg));
    }

    /// FW-CORE-02 acceptance: halves 0 and 1 done (two EOF events), half 2
    /// still DMA-owned, so half 2 is not consumed.
    #[test]
    fn halves_0_and_1_done_half_2_dma_owned_is_not_consumed() {
        let mut drain = drain8(DrainGate::OwnerWriteback);
        let mut ring = Ring8::new();
        let mut dma = MockDma::default();

        ring.dma_fill(0, 0xA0);
        ring.dma_complete(0, &mut dma, true);
        ring.dma_fill(1, 0xA1);
        ring.dma_complete(1, &mut dma, true);
        // Half 2 is being filled by the DMA right now: data present, owner=1.
        ring.dma_write(2, 0, &[0xA2; 100]);

        let (r, seen, n) = drain_collect(&mut drain, &mut ring, &mut dma);
        assert_eq!(r, Ok(2));
        assert_eq!(n, 2);
        assert_eq!(&seen[..2], &[0xA0, 0xA1]);
        assert_eq!(drain.half_idx(), 2);
        assert_eq!(drain.pending(), 0);
        assert_eq!(dma.int_raw & I2S_IN_SUC_EOF_INT_BIT, 0, "EOF bit cleared");

        // Consumed halves are poisoned and re-armed with one packed word.
        assert!(ring.half(0).iter().all(|&b| b == DMA_POISON_BYTE));
        assert!(ring.half(1).iter().all(|&b| b == DMA_POISON_BYTE));
        for d in 0..4 {
            assert_eq!(ring.desc[d].flags(), rearmed_flags());
        }
        // Half 2 is untouched.
        assert_eq!(ring.half(2)[2], 0xA2);
        assert!(ring.desc[4].owner() && ring.desc[5].owner());

        // A stale / spurious EOF bit with the same address consumes nothing.
        dma.int_raw |= I2S_IN_SUC_EOF_INT_BIT;
        let (r, _, n) = drain_collect(&mut drain, &mut ring, &mut dma);
        assert_eq!((r, n), (Ok(0), 0));
        assert_eq!(drain.half_idx(), 2);
    }

    /// FW-CORE-02 / LL-07: even if the EOF position claims half 2 complete,
    /// the owner gate holds while its last descriptor is still DMA-owned.
    #[test]
    fn owner_gate_holds_pending_half_until_written_back() {
        let mut drain = drain8(DrainGate::OwnerWriteback);
        let mut ring = Ring8::new();
        let mut dma = MockDma::default();
        ring.dma_complete(0, &mut dma, true);
        ring.dma_complete(1, &mut dma, true);
        ring.dma_complete(2, &mut dma, false);

        let (r, _, n) = drain_collect(&mut drain, &mut ring, &mut dma);
        assert_eq!((r, n), (Ok(2), 2));
        assert_eq!(drain.pending(), 1);
        assert_eq!(drain.half_idx(), 2);

        // Write-back arrives later; the pending half drains without a new EOF.
        ring.desc[5].set_owner(false);
        let (r, _, n) = drain_collect(&mut drain, &mut ring, &mut dma);
        assert_eq!((r, n), (Ok(1), 1));
        assert_eq!(drain.pending(), 0);
        assert_eq!(drain.half_idx(), 3);
    }

    /// LL-07: with every descriptor CPU-owned, the drain still stops at the
    /// EOF descriptor.
    #[test]
    fn drain_never_passes_the_eof_descriptor() {
        let mut drain = drain8(DrainGate::EofPosition);
        let mut ring = Ring8::new();
        let mut dma = MockDma::default();
        for d in ring.desc.iter_mut() {
            d.set_owner(false);
        }
        ring.dma_complete(0, &mut dma, true);
        ring.dma_complete(1, &mut dma, true);

        let (r, _, n) = drain_collect(&mut drain, &mut ring, &mut dma);
        assert_eq!((r, n), (Ok(2), 2));
        assert_eq!(drain.half_idx(), 2);

        // No EOF yet: nothing is consumed even though owner bits say CPU.
        let (r, _, n) = drain_collect(&mut drain, &mut ring, &mut dma);
        assert_eq!((r, n), (Ok(0), 0));
    }

    /// LL-07: more than `half_cnt - 1` pending halves fails with `Overflow`,
    /// counted across calls from the `in_eof_des_addr` delta.
    #[test]
    fn overflow_when_more_than_half_cnt_minus_1_pending() {
        let mut drain = drain8(DrainGate::OwnerWriteback);
        let mut ring = Ring8::new();
        let mut dma = MockDma::default();

        // Seven halves complete but none written back: 7 pending is legal.
        ring.dma_complete(6, &mut dma, false);
        let (r, _, n) = drain_collect(&mut drain, &mut ring, &mut dma);
        assert_eq!((r, n), (Ok(0), 0));
        assert_eq!(drain.pending(), HALVES - 1);

        // The eighth completes: the DMA is now overwriting half 0.
        ring.dma_complete(7, &mut dma, false);
        let (r, _, n) = drain_collect(&mut drain, &mut ring, &mut dma);
        assert_eq!(r, Err(CameraError::Overflow));
        assert_eq!(n, 0);
    }

    /// LL-07: a single jump past `half_cnt - 1` overflows too, and the wrap
    /// of the EOF position is counted modulo the ring.
    #[test]
    fn overflow_is_detected_across_a_ring_wrap() {
        let mut drain = drain8(DrainGate::EofPosition);
        let mut ring = Ring8::new();
        let mut dma = MockDma::default();
        for h in 0..6 {
            ring.dma_complete(h, &mut dma, true);
        }
        let (r, _, n) = drain_collect(&mut drain, &mut ring, &mut dma);
        assert_eq!((r, n), (Ok(6), 6));
        assert_eq!(drain.half_idx(), 6);

        // Halves 6, 7, 0 complete (wrap): 3 pending, fine.
        ring.dma_complete(0, &mut dma, true);
        assert_eq!(drain.observe(&ring.desc, &mut dma), Ok(3));
        // Then 1..=5 complete: 8 pending.
        ring.dma_complete(5, &mut dma, true);
        assert_eq!(
            drain.observe(&ring.desc, &mut dma),
            Err(CameraError::Overflow)
        );
    }

    #[test]
    fn descriptor_errors_abort_the_frame() {
        let mut drain = drain8(DrainGate::OwnerWriteback);
        let mut ring = Ring8::new();
        let mut dma = MockDma {
            int_raw: I2S_IN_DSCR_ERR_INT_RAW,
            ..MockDma::default()
        };
        let (r, _, _) = drain_collect(&mut drain, &mut ring, &mut dma);
        assert_eq!(r, Err(CameraError::DmaRingCorrupted));
        assert_eq!(dma.int_raw, 0, "error bit cleared");

        dma.int_raw = I2S_IN_DSCR_EMPTY_INT_RAW | I2S_IN_SUC_EOF_INT_BIT;
        let (r, _, _) = drain_collect(&mut drain, &mut ring, &mut dma);
        assert_eq!(r, Err(CameraError::Overflow));
        assert_eq!(
            drain.int_raw_seen() & I2S_IN_DSCR_ERR_INT_RAW,
            I2S_IN_DSCR_ERR_INT_RAW
        );
    }

    #[test]
    fn eof_on_a_mid_half_descriptor_is_a_ring_error() {
        let mut drain = drain8(DrainGate::OwnerWriteback);
        let ring = Ring8::new();
        let mut dma = MockDma {
            int_raw: I2S_IN_SUC_EOF_INT_BIT,
            eof_addr: ring.desc[2].link_addr_20bit(),
            ..MockDma::default()
        };
        assert_eq!(
            drain.observe(&ring.desc, &mut dma),
            Err(CameraError::DmaRingCorrupted)
        );
    }

    #[test]
    fn drain_is_bounded_to_max_drain_halves_per_call() {
        type Ring16 = Ring<32, { 16 * HALF }>;
        let mut ring = Ring16::new();
        let mut drain =
            HalfDrain::new(16, NODE, DmaFilterKind::Jpeg, DrainGate::OwnerWriteback).expect("d");
        let mut dma = MockDma::default();
        for h in 0..12 {
            for d in [2 * h, 2 * h + 1] {
                ring.desc[d].set_owner(false);
            }
        }
        ring.dma_complete(11, &mut dma, true);
        let mut scratch = [0u8; CHUNK];
        let r = drain.drain_ready_halves(
            &mut ring.desc,
            &mut ring.buf,
            &mut dma,
            &mut scratch,
            |_| ControlFlow::Continue(()),
        );
        assert_eq!(r, Ok(MAX_DRAIN_HALVES));
        assert_eq!(drain.pending(), 12 - MAX_DRAIN_HALVES);
        let r = drain.drain_ready_halves(
            &mut ring.desc,
            &mut ring.buf,
            &mut dma,
            &mut scratch,
            |_| ControlFlow::Continue(()),
        );
        assert_eq!(r, Ok(12 - MAX_DRAIN_HALVES));
        assert_eq!(drain.half_idx(), 12);
    }

    #[test]
    fn sink_break_stops_after_the_current_half() {
        let mut drain = drain8(DrainGate::OwnerWriteback);
        let mut ring = Ring8::new();
        let mut dma = MockDma::default();
        for h in 0..3 {
            ring.dma_fill(h, 0xB0 + h as u8);
            ring.dma_complete(h, &mut dma, true);
        }
        let mut scratch = [0u8; CHUNK];
        let r = drain.drain_ready_halves(
            &mut ring.desc,
            &mut ring.buf,
            &mut dma,
            &mut scratch,
            |_| ControlFlow::Break(()),
        );
        assert_eq!(r, Ok(1));
        assert_eq!(drain.half_idx(), 1);
        assert_eq!(drain.pending(), 2);
        // The consumed half was re-armed before the sink ran; the others not.
        assert_eq!(ring.desc[1].flags(), rearmed_flags());
        assert!(!ring.desc[3].owner());
    }

    /// LL-05: the tail half is consumed whatever its descriptor says, then
    /// poisoned and re-armed.
    #[test]
    fn tail_half_is_consumed_unconditionally_and_poisoned() {
        let mut drain = drain8(DrainGate::OwnerWriteback);
        let mut ring = Ring8::new();
        let mut dma = MockDma::default();
        ring.dma_write(0, 0, &[0xFF, 0xD8, 0xFF, 0xD9]);
        assert!(ring.desc[0].owner() && ring.desc[1].owner());

        let mut scratch = [0u8; CHUNK];
        let n = drain
            .drain_tail_half(&mut ring.desc, &mut ring.buf, &mut dma, &mut scratch)
            .expect("tail");
        assert_eq!(n, CHUNK);
        assert_eq!(&scratch[..4], &[0xFF, 0xD8, 0xFF, 0xD9]);
        assert!(scratch[4..].iter().all(|&b| b == DMA_POISON_BYTE));
        assert!(ring.half(0).iter().all(|&b| b == DMA_POISON_BYTE));
        assert_eq!(ring.desc[0].flags(), rearmed_flags());
        assert_eq!(drain.half_idx(), 1);
        assert_eq!(drain.halves_drained(), 1);
        assert!(dma.barriers >= 1, "poison ordered before re-arm");
    }

    #[test]
    fn peek_does_not_consume() {
        let mut ring = Ring8::new();
        let drain = drain8(DrainGate::OwnerWriteback);
        ring.dma_write(0, 0, &[0x11, 0x22]);
        let mut scratch = [0u8; CHUNK];
        assert_eq!(drain.peek_current_half(&ring.buf, &mut scratch), Ok(CHUNK));
        assert_eq!(&scratch[..2], &[0x11, 0x22]);
        assert_eq!(ring.half(0)[2], 0x11, "peek leaves the half intact");
        assert_eq!(drain.half_idx(), 0);
    }

    #[test]
    fn reset_restarts_at_half_0() {
        let mut drain = drain8(DrainGate::EofPosition);
        let mut ring = Ring8::new();
        let mut dma = MockDma::default();
        ring.dma_complete(2, &mut dma, true);
        let (r, _, _) = drain_collect(&mut drain, &mut ring, &mut dma);
        assert_eq!(r, Ok(3));
        drain.reset();
        assert_eq!((drain.half_idx(), drain.pending()), (0, 0));
        assert_eq!((drain.int_raw_seen(), drain.halves_drained()), (0, 0));
        // After a restart the first EOF at half 0 counts as one half.
        ring.build();
        ring.dma_complete(0, &mut dma, true);
        assert_eq!(drain.observe(&ring.desc, &mut dma), Ok(1));
    }

    // ---- CONV-10: bounded EOI scan ----

    #[test]
    fn eoi_tail_window_is_last_chunk_plus_margin() {
        assert_eq!(
            jpeg_eoi_tail_start(100_000, 1024),
            100_000 - 1024 - JPEG_EOI_TAIL_MARGIN
        );
        assert_eq!(jpeg_eoi_tail_start(500, 1024), 0);
        assert_eq!(jpeg_eoi_tail_start(0, 0), 0);
        assert_eq!(jpeg_eoi_tail_start(10, usize::MAX), 0);
    }

    /// CONV-10 acceptance: a frame with no EOI touches only a bounded number
    /// of bytes.
    #[test]
    fn frame_without_eoi_touches_a_bounded_number_of_bytes() {
        let len = 128 * 1024;
        let mut touched = 0usize;
        let mut lowest = usize::MAX;
        let r = find_jpeg_eoi_tail_by(len, 1024, |i| {
            touched += 1;
            lowest = lowest.min(i);
            0x5A
        });
        assert_eq!(r, None);
        let window = 1024 + JPEG_EOI_TAIL_MARGIN;
        assert!(lowest >= len - window, "lowest touched {lowest}");
        assert!(touched <= 2 * window, "touched {touched}");
    }

    #[test]
    fn eoi_tail_finds_the_marker_in_the_last_chunk_and_across_the_boundary() {
        let mut frame = [0x5Au8; 8192];
        frame[0] = 0xFF;
        frame[1] = 0xD8;
        frame[8000] = 0xFF;
        frame[8001] = 0xD9;
        assert_eq!(find_jpeg_eoi_tail(&frame, 1024), Some(8000));
        assert_eq!(find_jpeg_eoi_tail(&frame[..8002], 1024), Some(8000));

        // Marker straddling the last-chunk boundary.
        let mut frame = [0x5Au8; 4096];
        frame[3071] = 0xFF;
        frame[3072] = 0xD9;
        assert_eq!(find_jpeg_eoi_tail(&frame, 1024), Some(3071));
    }

    #[test]
    fn eoi_tail_ignores_markers_before_the_window() {
        let mut frame = [0x5Au8; 16384];
        frame[10] = 0xFF;
        frame[11] = 0xD9;
        assert_eq!(find_jpeg_eoi_tail(&frame, 1024), None);
        assert_eq!(find_jpeg_eoi_tail(&[0xFF], 1024), None);
        assert_eq!(find_jpeg_eoi_tail(&[], 1024), None);
        assert_eq!(find_jpeg_eoi_tail(&[0xFF, 0xD9], 0), Some(0));
    }

    // ---- PolledJpegCapture ----

    #[derive(Default)]
    struct TestHooks {
        idle_calls: u32,
        now: u64,
    }

    impl CaptureHooks for TestHooks {
        fn idle(&mut self) {
            self.idle_calls += 1;
        }
        fn now_us(&mut self) -> u64 {
            self.now += 10;
            self.now
        }
    }

    struct Harness {
        ring: Ring8,
        dma: MockDma,
        hooks: TestHooks,
        out: [u8; 16 * 1024],
    }

    impl Harness {
        fn new() -> Self {
            Self {
                ring: Ring8::new(),
                dma: MockDma::default(),
                hooks: TestHooks::default(),
                out: [0; 16 * 1024],
            }
        }

        fn capture() -> PolledJpegCapture {
            PolledJpegCapture::new(HALVES, DmaFilterKind::Jpeg, DrainGate::OwnerWriteback)
                .expect("capture")
        }

        fn poll(&mut self, cap: &mut PolledJpegCapture, vsync: bool) -> CaptureStep {
            cap.poll(
                &mut self.ring.desc,
                &mut self.ring.buf,
                &mut self.dma,
                &mut self.out,
                &mut || vsync,
                &mut self.hooks,
            )
        }

        fn finish(&mut self, cap: &mut PolledJpegCapture) -> Result<usize, CameraError> {
            cap.finish(
                &mut self.ring.desc,
                &mut self.ring.buf,
                &mut self.dma,
                &mut self.out,
            )
        }

        /// Streams one frame of unpacked `bytes` from half 0 through the DMA
        /// model, polling after each completed half, then ends VSYNC and
        /// finishes. Mirrors a fresh DMA start (ring rebuilt, buffer kept).
        fn run_frame(&mut self, bytes: &[u8]) -> Result<usize, CameraError> {
            self.ring.build();
            self.dma.int_raw = 0;
            let mut cap = Self::capture();
            // The loop is already polling when the frame starts (VSYNC active).
            assert_eq!(self.poll(&mut cap, true), CaptureStep::Pending);
            let mut h = 0usize;
            let mut off = 0usize;
            while off < bytes.len() {
                let n = (bytes.len() - off).min(CHUNK);
                self.ring.dma_write(h, 0, &bytes[off..off + n]);
                off += n;
                if n == CHUNK {
                    self.ring.dma_complete(h, &mut self.dma, true);
                    match self.poll(&mut cap, true) {
                        CaptureStep::Pending => {}
                        CaptureStep::Complete(len) => return Ok(len),
                        CaptureStep::Failed(e) => return Err(e),
                        CaptureStep::FrameEnded => break,
                    }
                    h = (h + 1) % HALVES;
                }
            }
            // VSYNC ends.
            for _ in 0..16 {
                match self.poll(&mut cap, false) {
                    CaptureStep::Pending => continue,
                    CaptureStep::Complete(len) => return Ok(len),
                    CaptureStep::Failed(e) => return Err(e),
                    CaptureStep::FrameEnded => return self.finish(&mut cap),
                }
            }
            panic!("VSYNC end was not detected");
        }
    }

    /// Builds an unpacked JPEG-like stream: SOI, `len - 4` filler bytes and,
    /// when `eoi`, a trailing EOI (otherwise filler).
    fn jpeg(len: usize, eoi: bool, fill: u8) -> [u8; 8192] {
        let mut b = [fill; 8192];
        b[0] = 0xFF;
        b[1] = 0xD8;
        b[2] = 0xFF;
        b[3] = 0xE0;
        if eoi {
            b[len - 2] = 0xFF;
            b[len - 1] = 0xD9;
        }
        b
    }

    #[test]
    fn capture_returns_exact_length_for_a_multi_half_frame() {
        let mut h = Harness::new();
        let f = jpeg(2500, true, 0x33);
        assert_eq!(h.run_frame(&f[..2500]), Ok(2500));
        assert_eq!(&h.out[..4], &[0xFF, 0xD8, 0xFF, 0xE0]);
        assert_eq!(&h.out[2498..2500], &[0xFF, 0xD9]);
    }

    /// LL-05: the JPEG ends inside the first 2048 DMA bytes of the current
    /// half (the old `has_new_data` gate stayed false); VSYNC end processes
    /// the half unconditionally.
    #[test]
    fn frame_ending_early_in_a_half_completes_on_vsync_end() {
        let mut h = Harness::new();
        let f = jpeg(1024 + 300, true, 0x44);
        assert_eq!(h.run_frame(&f[..1324]), Ok(1324));
    }

    /// LL-06 acceptance: a stale old frame in the ring yields the correct
    /// length or `CorruptedFrame`, never a spliced frame.
    #[test]
    fn stale_old_frame_in_the_ring_is_never_spliced() {
        let mut h = Harness::new();
        // Frame 1: 950 bytes, EOI at 948 inside the tail of half 0.
        let f1 = jpeg(950, true, 0x61);
        assert_eq!(h.run_frame(&f1[..950]), Ok(950));

        // Frame 2: 300 bytes, no EOI. Without poisoning the tail would still
        // hold frame 1's EOI at 948 and return a spliced 950-byte frame.
        let f2 = jpeg(300, false, 0x62);
        assert_eq!(h.run_frame(&f2[..300]), Err(CameraError::CorruptedFrame));

        // Frame 3: 300 bytes with EOI: exact length.
        let f3 = jpeg(300, true, 0x63);
        assert_eq!(h.run_frame(&f3[..300]), Ok(300));
    }

    /// LL-06: same, with the stale EOI in a later half that was drained as a
    /// complete half in the earlier frame.
    #[test]
    fn stale_eoi_in_a_later_half_is_never_spliced() {
        let mut h = Harness::new();
        // Frame 1 is exactly two halves; its EOI (at 2046) sits at the end of
        // half 1, which completes with an EOF and is drained as a full half.
        let f1 = jpeg(2048, true, 0x71);
        assert_eq!(h.run_frame(&f1[..2048]), Ok(2048));
        // Frame 2 fills half 0 and part of half 1. Without poisoning, half 1
        // would still hold frame 1's EOI and splice a 2048-byte frame.
        let f2 = jpeg(1500, false, 0x72);
        assert_eq!(h.run_frame(&f2[..1500]), Err(CameraError::CorruptedFrame));
        let f3 = jpeg(1500, true, 0x73);
        assert_eq!(h.run_frame(&f3[..1500]), Ok(1500));
    }

    #[test]
    fn idle_peek_ends_the_frame_when_the_eoi_is_already_written() {
        let mut h = Harness::new();
        let mut cap = Harness::capture();
        let f = jpeg(1200, true, 0x55);
        h.ring.dma_write(0, 0, &f[..CHUNK]);
        h.ring.dma_complete(0, &mut h.dma, true);
        assert_eq!(h.poll(&mut cap, true), CaptureStep::Pending);
        h.ring.dma_write(1, 0, &f[CHUNK..1200]);

        // VSYNC stays active (no end); the idle peek finds the EOI.
        let mut step = CaptureStep::Pending;
        for _ in 0..PolledJpegCapture::IDLE_PEEK_POLLS {
            step = h.poll(&mut cap, true);
            if step != CaptureStep::Pending {
                break;
            }
        }
        assert_eq!(step, CaptureStep::FrameEnded);
        assert_eq!(h.finish(&mut cap), Ok(1200));
        let d = cap.diag();
        assert_eq!(d.idle_peeks, 1);
        assert_eq!(d.halves_drained, 2);
        assert!(d.first_half_us.is_some() && d.frame_end_us.is_some());
        assert!(h.hooks.idle_calls >= PolledJpegCapture::IDLE_PEEK_POLLS / 64 - 1);
    }

    #[test]
    fn capture_reports_ring_overflow() {
        let mut h = Harness::new();
        let mut cap = Harness::capture();
        // Seven halves complete but not yet written back: legal, nothing drains.
        h.ring.dma_complete(6, &mut h.dma, false);
        assert_eq!(h.poll(&mut cap, true), CaptureStep::Pending);
        // The eighth completes before the CPU consumed any: the ring lapped.
        h.ring.dma_complete(7, &mut h.dma, false);
        assert_eq!(
            h.poll(&mut cap, true),
            CaptureStep::Failed(CameraError::Overflow)
        );
    }

    #[test]
    fn capture_buffer_too_small_fails() {
        let mut h = Harness::new();
        let mut cap = Harness::capture();
        let f = jpeg(1024, false, 0x67);
        h.ring.dma_write(0, 0, &f[..CHUNK]);
        h.ring.dma_complete(0, &mut h.dma, true);
        let step = cap.poll(
            &mut h.ring.desc,
            &mut h.ring.buf,
            &mut h.dma,
            &mut h.out[..100],
            &mut || true,
            &mut h.hooks,
        );
        assert_eq!(step, CaptureStep::Failed(CameraError::BufferTooSmall));
    }
}
