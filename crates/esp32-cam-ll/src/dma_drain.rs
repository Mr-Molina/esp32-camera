//! Pure-Rust DMA half-buffer drain routines, ring poisoning, and flow-control.

use core::ptr;
use core::sync::atomic::{compiler_fence, Ordering};

use esp32_cam_types::CameraError;

use crate::dma_filter::DmaFilterKind;
use crate::dma_ring::Lldesc;
use crate::esp32_i2s::{
    I2S_INLINK_ADDR_MASK, JPEG_DMA_HALF_BUFFER_SIZE, JPEG_DMA_NODE_SIZE,
    JPEG_UNPACKED_HALF_BUFFER_SIZE,
};

/// Non-marker poison byte (`0xAA`) written into drained DMA half-buffers
/// (`LL-06`) so stale `0xFF`/`0xD8`/`0xD9` markers cannot survive frames.
pub const DMA_RING_POISON_BYTE: u8 = 0xAA;

/// Maximum halves drained in a single poll burst (`LL-15`).
pub const MAX_DRAIN_HALVES_PER_BURST: usize = 8;

/// Fills `dma_buf` with [`DMA_RING_POISON_BYTE`] (`LL-06`).
#[inline]
pub fn poison_dma_ring_buf(dma_buf: &mut [u8]) {
    dma_buf.fill(DMA_RING_POISON_BYTE);
}

/// Callbacks invoked by polled capture loops while waiting on DMA (`LL-15`).
pub trait CaptureHooks {
    /// Invoked once per poll spin while waiting for DMA activity.
    fn idle(&mut self) {}

    /// Monotonic timestamp in microseconds (`0` when unused).
    fn now_us(&mut self) -> u64 {
        0
    }
}

/// Default no-op [`CaptureHooks`] implementation.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct NoOpCaptureHooks;

impl CaptureHooks for NoOpCaptureHooks {}

/// Sink flow-control directive returned per unpacked half-buffer (`LL-15`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DrainControl {
    /// Continue draining remaining ready halves.
    Continue,
    /// Stop draining immediately (e.g., EOI found or frame aborted).
    Stop,
}

/// Summary returned by [`drain_ready_halves`].
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DrainSummary {
    /// Number of half-buffers unpacked and re-armed during this call.
    pub halves_drained: usize,
    /// `true` if the sink returned [`DrainControl::Stop`].
    pub stopped_by_sink: bool,
}

/// Identifies which half-buffer index contains the descriptor whose address
/// matches `eof_desc_addr`.
///
/// Returns `None` if `eof_desc_addr == 0` or `descriptors.len() < 2`.
#[must_use]
pub fn eof_half_from_desc_addr(descriptors: &[Lldesc], eof_desc_addr: u32) -> Option<usize> {
    if eof_desc_addr == 0 || descriptors.len() < 2 {
        return None;
    }
    let addr = eof_desc_addr & I2S_INLINK_ADDR_MASK;
    let half_cnt = descriptors.len() / 2;
    for h in 0..half_cnt {
        let desc0_addr =
            (ptr::from_ref(&descriptors[2 * h]) as usize as u32) & I2S_INLINK_ADDR_MASK;
        let desc1_addr =
            (ptr::from_ref(&descriptors[2 * h + 1]) as usize as u32) & I2S_INLINK_ADDR_MASK;
        if desc0_addr == addr || desc1_addr == addr {
            return Some(h);
        }
    }
    None
}

/// Drains CPU-owned DMA half-buffers, unpacking them with `filter` and passing
/// them to `sink`.
///
/// Returns a [`DrainSummary`] indicating the count of half-buffers drained
/// and whether `sink` requested a stop via [`DrainControl::Stop`].
pub fn drain_ready_halves<F>(
    descriptors: &mut [Lldesc],
    dma_buf: &mut [u8],
    half_idx: &mut usize,
    eof_half: Option<usize>,
    filter: DmaFilterKind,
    mut sink: F,
) -> Result<DrainSummary, CameraError>
where
    F: FnMut(&[u8]) -> Result<DrainControl, CameraError>,
{
    let desc_len = descriptors.len();
    if desc_len < 2 || !desc_len.is_multiple_of(2) {
        return Err(CameraError::InvalidArg);
    }
    let half_buf_cnt = desc_len / 2;
    let required_dma_len = half_buf_cnt
        .checked_mul(JPEG_DMA_HALF_BUFFER_SIZE)
        .ok_or(CameraError::Overflow)?;
    if dma_buf.len() < required_dma_len || *half_idx >= half_buf_cnt {
        return Err(CameraError::InvalidArg);
    }

    // LL-07: Overflow check when EOF was signaled in hardware ring.
    if let Some(target_h) = eof_half {
        if target_h >= half_buf_cnt {
            return Err(CameraError::InvalidArg);
        }
        let pending = (target_h + half_buf_cnt - *half_idx) % half_buf_cnt + 1;
        if half_buf_cnt >= 2 && pending > half_buf_cnt - 1 {
            return Err(CameraError::Overflow);
        }
    }

    let mut halves_drained = 0usize;
    let max_drain = half_buf_cnt.min(MAX_DRAIN_HALVES_PER_BURST);

    for _ in 0..max_drain {
        let cur = *half_idx;
        let desc1_ptr = ptr::from_mut(&mut descriptors[2 * cur + 1]);

        // JUSTIFICATION: Volatile read of descriptor size_length_flags word
        // written asynchronously by DMA hardware.
        // SAFETY:
        // 1. Alignment & Non-null: `desc1_ptr` is derived from valid slice ref.
        // 2. Memory layout: Reads a 32-bit `size_length_flags` field.
        // 3. Provenance: Sourced directly from caller-borrowed mutable slice.
        // 4. Aliasing: Volatile read prevents compiler caching descriptor word.
        // 5. Volatile semantics: Observes DMA controller write to OWNER bit.
        // 6. Panic safety: Pointer dereference cannot panic or unwind.
        let flags = unsafe { ptr::read_volatile(ptr::addr_of!((*desc1_ptr).size_length_flags)) };

        if (flags & Lldesc::OWNER_BIT) != 0 {
            break;
        }

        compiler_fence(Ordering::Acquire);

        let half_off = cur * JPEG_DMA_HALF_BUFFER_SIZE;
        let mut unpacked = [0u8; JPEG_UNPACKED_HALF_BUFFER_SIZE];
        let written = filter.apply(
            &mut unpacked,
            &dma_buf[half_off..half_off + JPEG_DMA_HALF_BUFFER_SIZE],
        );

        // LL-06: Poison consumed half-buffer so stale markers cannot survive.
        dma_buf[half_off..half_off + JPEG_DMA_HALF_BUFFER_SIZE].fill(DMA_RING_POISON_BYTE);

        compiler_fence(Ordering::Release);

        let rearm_flags = Lldesc::pack_flags(JPEG_DMA_NODE_SIZE as u16, 0, 0, false, false, true);

        let desc0_flags_ptr = ptr::addr_of_mut!(descriptors[2 * cur].size_length_flags);
        let desc1_flags_ptr = ptr::addr_of_mut!(descriptors[2 * cur + 1].size_length_flags);

        // JUSTIFICATION: Volatile re-arming of descriptors 0 and 1 for DMA.
        // SAFETY:
        // 1. Alignment & Non-null: Pointers derived from valid in-bounds slice.
        // 2. Memory layout: Writes a 32-bit flags word to `size_length_flags`.
        // 3. Provenance: Sourced directly from caller-borrowed mutable slice.
        // 4. Aliasing: Unique mutable borrow of slice guarantees no aliasing.
        // 5. Volatile semantics: Hardware immediately observes OWNER bit write.
        // 6. Panic safety: Pointer writes cannot panic or unwind.
        unsafe {
            ptr::write_volatile(desc0_flags_ptr, rearm_flags);
            ptr::write_volatile(desc1_flags_ptr, rearm_flags);
        }

        *half_idx = (cur + 1) % half_buf_cnt;
        halves_drained += 1;

        if sink(&unpacked[..written])? == DrainControl::Stop {
            return Ok(DrainSummary {
                halves_drained,
                stopped_by_sink: true,
            });
        }

        // Never drain past the EOF descriptor
        if eof_half == Some(cur) {
            break;
        }
    }

    Ok(DrainSummary {
        halves_drained,
        stopped_by_sink: false,
    })
}

/// Unconditionally drains the single trailing half-buffer at `*half_idx`
/// (`LL-05`).
///
/// Unpacks the current half regardless of `OWNER_BIT`, poisons that half with
/// [`DMA_RING_POISON_BYTE`], re-arms both descriptors, advances `*half_idx`,
/// and returns the sink result.
pub fn drain_trailing_half_unconditional<F>(
    descriptors: &mut [Lldesc],
    dma_buf: &mut [u8],
    half_idx: &mut usize,
    filter: DmaFilterKind,
    mut sink: F,
) -> Result<DrainControl, CameraError>
where
    F: FnMut(&[u8]) -> Result<DrainControl, CameraError>,
{
    let desc_len = descriptors.len();
    if desc_len < 2 || !desc_len.is_multiple_of(2) {
        return Err(CameraError::InvalidArg);
    }
    let half_buf_cnt = desc_len / 2;
    let required_dma_len = half_buf_cnt
        .checked_mul(JPEG_DMA_HALF_BUFFER_SIZE)
        .ok_or(CameraError::Overflow)?;
    if dma_buf.len() < required_dma_len || *half_idx >= half_buf_cnt {
        return Err(CameraError::InvalidArg);
    }

    let cur = *half_idx;
    let half_off = cur * JPEG_DMA_HALF_BUFFER_SIZE;
    let mut unpacked = [0u8; JPEG_UNPACKED_HALF_BUFFER_SIZE];
    let written = filter.apply(
        &mut unpacked,
        &dma_buf[half_off..half_off + JPEG_DMA_HALF_BUFFER_SIZE],
    );

    dma_buf[half_off..half_off + JPEG_DMA_HALF_BUFFER_SIZE].fill(DMA_RING_POISON_BYTE);

    compiler_fence(Ordering::Release);

    let rearm_flags = Lldesc::pack_flags(JPEG_DMA_NODE_SIZE as u16, 0, 0, false, false, true);

    let desc0_flags_ptr = ptr::addr_of_mut!(descriptors[2 * cur].size_length_flags);
    let desc1_flags_ptr = ptr::addr_of_mut!(descriptors[2 * cur + 1].size_length_flags);

    // JUSTIFICATION: Volatile re-arming of descriptors 0 and 1 for DMA.
    // SAFETY:
    // 1. Alignment & Non-null: Pointers derived from valid in-bounds slice.
    // 2. Memory layout: Writes a 32-bit flags word to `size_length_flags`.
    // 3. Provenance: Sourced directly from caller-borrowed mutable slice.
    // 4. Aliasing: Unique mutable borrow of slice guarantees no aliasing.
    // 5. Volatile semantics: Hardware immediately observes OWNER bit write.
    // 6. Panic safety: Pointer writes cannot panic or unwind.
    unsafe {
        ptr::write_volatile(desc0_flags_ptr, rearm_flags);
        ptr::write_volatile(desc1_flags_ptr, rearm_flags);
    }

    *half_idx = (cur + 1) % half_buf_cnt;

    sink(&unpacked[..written])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_drain_ready_halves_partial_ownership() {
        // 3 half-buffers = 6 descriptors
        let mut descriptors = [Lldesc::default(); 6];
        let mut dma_buf = [0u8; 3 * JPEG_DMA_HALF_BUFFER_SIZE];

        // Halves 0 and 1 CPU-owned (OWNER_BIT = 0)
        descriptors[0].size_length_flags =
            Lldesc::pack_flags(JPEG_DMA_NODE_SIZE as u16, 100, 0, false, false, false);
        descriptors[1].size_length_flags =
            Lldesc::pack_flags(JPEG_DMA_NODE_SIZE as u16, 100, 0, false, false, false);
        descriptors[2].size_length_flags =
            Lldesc::pack_flags(JPEG_DMA_NODE_SIZE as u16, 100, 0, false, false, false);
        descriptors[3].size_length_flags =
            Lldesc::pack_flags(JPEG_DMA_NODE_SIZE as u16, 100, 0, false, false, false);

        // Half 2 DMA-owned (OWNER_BIT = 1)
        descriptors[4].size_length_flags =
            Lldesc::pack_flags(JPEG_DMA_NODE_SIZE as u16, 0, 0, false, false, true);
        descriptors[5].size_length_flags =
            Lldesc::pack_flags(JPEG_DMA_NODE_SIZE as u16, 0, 0, false, false, true);

        // Populate sample data in halves 0 and 1
        dma_buf[..2 * JPEG_DMA_HALF_BUFFER_SIZE].fill(0x55);
        dma_buf[2 * JPEG_DMA_HALF_BUFFER_SIZE..].fill(0x77);

        let mut half_idx = 0usize;
        let mut collected_chunks = 0usize;

        let summary = drain_ready_halves(
            &mut descriptors,
            &mut dma_buf,
            &mut half_idx,
            None,
            DmaFilterKind::Jpeg,
            |_chunk| {
                collected_chunks += 1;
                Ok(DrainControl::Continue)
            },
        )
        .expect("drain should succeed");

        assert_eq!(summary.halves_drained, 2);
        assert!(!summary.stopped_by_sink);
        assert_eq!(collected_chunks, 2);
        assert_eq!(half_idx, 2);

        // Halves 0 and 1 poisoned with 0xAA and re-armed with OWNER_BIT
        assert!(dma_buf[..2 * JPEG_DMA_HALF_BUFFER_SIZE]
            .iter()
            .all(|&b| b == DMA_RING_POISON_BYTE));
        assert_ne!(descriptors[0].size_length_flags & Lldesc::OWNER_BIT, 0);
        assert_ne!(descriptors[1].size_length_flags & Lldesc::OWNER_BIT, 0);
        assert_ne!(descriptors[2].size_length_flags & Lldesc::OWNER_BIT, 0);
        assert_ne!(descriptors[3].size_length_flags & Lldesc::OWNER_BIT, 0);

        // Half 2 was not consumed (preserved 0x77)
        assert_eq!(dma_buf[2 * JPEG_DMA_HALF_BUFFER_SIZE], 0x77);
    }

    #[test]
    fn test_drain_ready_halves_overflow_and_eof_cutoff() {
        let mut descriptors = [Lldesc::default(); 6];
        let mut dma_buf = [0u8; 3 * JPEG_DMA_HALF_BUFFER_SIZE];

        // Set all halves CPU-owned
        for desc in &mut descriptors {
            desc.size_length_flags =
                Lldesc::pack_flags(JPEG_DMA_NODE_SIZE as u16, 100, 0, false, false, false);
        }

        let mut half_idx = 0usize;

        // LL-07 overflow: 3 halves, target_h = 2, half_idx = 0 -> pending = 3 > 2 -> Overflow
        let res = drain_ready_halves(
            &mut descriptors,
            &mut dma_buf,
            &mut half_idx,
            Some(2),
            DmaFilterKind::Jpeg,
            |_| Ok(DrainControl::Continue),
        );
        assert_eq!(res, Err(CameraError::Overflow));

        // eof_half = Some(0): drains half 0 and stops even if half 1 is CPU-owned
        half_idx = 0;
        let summary = drain_ready_halves(
            &mut descriptors,
            &mut dma_buf,
            &mut half_idx,
            Some(0),
            DmaFilterKind::Jpeg,
            |_| Ok(DrainControl::Continue),
        )
        .expect("drain should succeed");

        assert_eq!(summary.halves_drained, 1);
        assert_eq!(half_idx, 1);
        assert_eq!(descriptors[2].size_length_flags & Lldesc::OWNER_BIT, 0);
    }

    #[test]
    fn test_poison_ring_and_trailing_half_unconditional() {
        let mut descriptors = [Lldesc::default(); 4];
        let mut dma_buf = [0u8; 2 * JPEG_DMA_HALF_BUFFER_SIZE];

        // Stale EOI in DMA buffer
        dma_buf[10] = 0xFF;
        dma_buf[11] = 0xD9;

        // LL-06: poison ring
        poison_dma_ring_buf(&mut dma_buf);
        assert!(dma_buf.iter().all(|&b| b == DMA_RING_POISON_BYTE));

        // Mark half 0 as DMA-owned to verify unconditional drain bypasses OWNER_BIT
        descriptors[0].size_length_flags =
            Lldesc::pack_flags(JPEG_DMA_NODE_SIZE as u16, 0, 0, false, false, true);
        descriptors[1].size_length_flags =
            Lldesc::pack_flags(JPEG_DMA_NODE_SIZE as u16, 0, 0, false, false, true);

        let mut half_idx = 0usize;
        let mut collected = [0u8; JPEG_UNPACKED_HALF_BUFFER_SIZE];
        let mut collected_len = 0usize;

        let ctrl = drain_trailing_half_unconditional(
            &mut descriptors,
            &mut dma_buf,
            &mut half_idx,
            DmaFilterKind::Jpeg,
            |chunk| {
                collected[..chunk.len()].copy_from_slice(chunk);
                collected_len = chunk.len();
                Ok(DrainControl::Stop)
            },
        )
        .expect("trailing half drain succeeds");

        assert_eq!(ctrl, DrainControl::Stop);
        assert_eq!(half_idx, 1);

        // Descriptors re-armed
        assert_ne!(descriptors[0].size_length_flags & Lldesc::OWNER_BIT, 0);
        assert_ne!(descriptors[1].size_length_flags & Lldesc::OWNER_BIT, 0);

        // No stale EOI (0xFF, 0xD9) in collected output
        assert!(!collected[..collected_len]
            .windows(2)
            .any(|w| w == [0xFF, 0xD9]));
    }
}
