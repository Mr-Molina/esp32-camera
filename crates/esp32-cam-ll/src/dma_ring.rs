//! ESP32 ROM `lldesc_t` DMA descriptor layout, the checked [`DmaRing`]
//! (LL-01), circular ring initialization, and RGB/JPEG DMA buffer sizing
//! algorithms (`target/esp32/ll_cam.c:470-559`, `driver/cam_hal.c:465-482`).

use core::cell::UnsafeCell;
use core::fmt;
use core::ptr;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use esp32_cam_types::CameraError;

use crate::dma_filter::I2sSamplingMode;

/// Maximum buffer capacity per single ESP32/S2/S3 DMA descriptor node
/// (`LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE`).
pub const LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE: usize = 4092;

/// Default maximum DMA ping-pong buffer size in bytes
/// (`CONFIG_CAMERA_DMA_BUFFER_SIZE_MAX`).
pub const DEFAULT_CAMERA_DMA_BUFFER_SIZE_MAX: usize = 32768;

/// Maximum supported uncompressed image size in bytes (`4 MiB`,
/// `ll_cam.c:481`).
pub const MAX_DMA_IMAGE_SIZE: usize = 4 * 1024 * 1024;

/// First byte of the ESP32 internal SRAM the I2S DMA can reach
/// (`0x3FFA_E000`, DRAM; ESP32 TRM "Embedded Memory").
pub const DMA_CAPABLE_START: usize = 0x3FFA_E000;

/// Last byte (inclusive) of the DMA-capable internal SRAM window
/// (`0x3FFF_FFFF`). External PSRAM (`0x3F80_0000..`) is not DMA-capable.
pub const DMA_CAPABLE_END: usize = 0x3FFF_FFFF;

/// Size of one [`Lldesc`] in bytes (`lldesc_t` is three 32-bit words on the
/// target).
pub const DESC_BYTES: usize = core::mem::size_of::<Lldesc>();

/// Hardware DMA linked-list descriptor matching ESP32/ESP32-S2/ESP32-S3 ROM
/// `lldesc_t`.
///
/// Bitfield packing within `size_length_flags` (little-endian 32-bit word):
/// - bits `0..=11` (`12` bits): `size` (buffer capacity, max `4095`)
/// - bits `12..=23` (`12` bits): `length` (transferred byte count, max `4095`)
/// - bits `24..=28` (`5` bits): `offset` (software/reserved offset, max `31`)
/// - bit `29` (`1` bit): `sosf` (start of sub-frame)
/// - bit `30` (`1` bit): `eof` (end of frame)
/// - bit `31` (`1` bit): `owner` (`1` = DMA controller, `0` = CPU software)
///
/// The header word is written back by the DMA engine (owner, length, eof)
/// while the CPU holds references to the descriptor, so it is an
/// [`AtomicU32`] (LL-03): every accessor is a single load or store, and the
/// setters take `&self`. Loads are `Acquire` and stores `Release`, which on
/// Xtensa adds a `memw` on the DMA side of each access.
///
/// `buf` is `*mut u8`: the DMA writes through it (LL-02).
#[repr(C, align(4))]
pub struct Lldesc {
    pub size_length_flags: AtomicU32,
    pub buf: *mut u8,
    pub qe_stqe_next: *const Lldesc,
}

// `lldesc_t` layout: the header word first, then the two pointers.
const _: () = assert!(core::mem::offset_of!(Lldesc, size_length_flags) == 0);
#[cfg(target_pointer_width = "32")]
const _: () = assert!(
    core::mem::size_of::<Lldesc>() == 12
        && core::mem::align_of::<Lldesc>() == 4
        && core::mem::offset_of!(Lldesc, buf) == 4
);

impl Default for Lldesc {
    fn default() -> Self {
        Self::empty()
    }
}

impl fmt::Debug for Lldesc {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Lldesc")
            .field("size_length_flags", &format_args!("{:#010x}", self.flags()))
            .field("buf", &self.buf)
            .field("qe_stqe_next", &self.qe_stqe_next)
            .finish()
    }
}

impl Lldesc {
    /// Bit mask for the 12-bit `size` field (`bits 0..=11`).
    pub const SIZE_MASK: u32 = 0x0000_0FFF;
    /// Bit shift for the 12-bit `length` field (`bits 12..=23`).
    pub const LENGTH_SHIFT: u32 = 12;
    /// Bit mask for the 12-bit `length` field (`bits 12..=23`).
    pub const LENGTH_MASK: u32 = 0x00FF_F000;
    /// Bit shift for the 5-bit `offset` field (`bits 24..=28`).
    pub const OFFSET_SHIFT: u32 = 24;
    /// Bit mask for the 5-bit `offset` field (`bits 24..=28`).
    pub const OFFSET_MASK: u32 = 0x1F00_0000;
    /// Bit flag for `sosf` (`bit 29`).
    pub const SOSF_BIT: u32 = 1 << 29;
    /// Bit flag for `eof` (`bit 30`).
    pub const EOF_BIT: u32 = 1 << 30;
    /// Bit flag for `owner` (`bit 31`, `1` = DMA hardware).
    pub const OWNER_BIT: u32 = 1 << 31;

    /// A zeroed descriptor with null `buf` and `qe_stqe_next`, usable in
    /// array repeat expressions (`[Lldesc::EMPTY; 16]`). Every use is a
    /// fresh descriptor, which is the point.
    #[allow(
        clippy::declare_interior_mutable_const,
        reason = "array-repeat initializer; each use must be a new descriptor"
    )]
    pub const EMPTY: Self = Self::empty();

    /// Packs `size`, `length`, `offset`, `sosf`, `eof`, and `owner` into the 32-bit
    /// header word.
    #[inline]
    pub const fn pack_flags(
        size: u16,
        length: u16,
        offset: u8,
        sosf: bool,
        eof: bool,
        owner: bool,
    ) -> u32 {
        let s = if size > 0x0FFF { 0x0FFF } else { size as u32 };
        let l = (if length > 0x0FFF {
            0x0FFF
        } else {
            length as u32
        }) << Self::LENGTH_SHIFT;
        let o = (if offset > 0x1F { 0x1F } else { offset as u32 }) << Self::OFFSET_SHIFT;
        let sf = if sosf { Self::SOSF_BIT } else { 0 };
        let ef = if eof { Self::EOF_BIT } else { 0 };
        let ow = if owner { Self::OWNER_BIT } else { 0 };
        s | l | o | sf | ef | ow
    }

    /// Creates a zeroed descriptor with null `buf` and `qe_stqe_next` pointers.
    #[inline]
    pub const fn empty() -> Self {
        Self {
            size_length_flags: AtomicU32::new(0),
            buf: ptr::null_mut(),
            qe_stqe_next: ptr::null(),
        }
    }

    /// Reads the whole header word with one `Acquire` load.
    #[inline]
    pub fn flags(&self) -> u32 {
        self.size_length_flags.load(Ordering::Acquire)
    }

    /// Replaces the whole header word (`size`, `length`, `offset`, `sosf`,
    /// `eof`, `owner`) with one `Release` store, e.g. a value from
    /// [`Self::pack_flags`].
    ///
    /// Re-arming a descriptor this way is a single bus write, so the DMA never
    /// observes a partially updated header, and the `Release` ordering keeps
    /// earlier buffer writes (poisoning) before it (LL-03).
    #[inline]
    pub fn store_flags(&self, flags: u32) {
        self.size_length_flags.store(flags, Ordering::Release);
    }

    /// Replaces the bits of `mask` with `bits`: one load and one packed
    /// store. Only for a CPU-owned descriptor (the DMA is stopped or does not
    /// own it); use [`Self::store_flags`] to re-arm a descriptor the DMA may
    /// write back concurrently.
    #[inline]
    fn update_flags(&self, mask: u32, bits: u32) {
        let cur = self.flags();
        self.store_flags((cur & !mask) | (bits & mask));
    }

    /// Returns the 12-bit `size` field (`0..=4095`).
    #[inline]
    pub fn size(&self) -> u16 {
        (self.flags() & Self::SIZE_MASK) as u16
    }

    /// Sets the 12-bit `size` field (`0..=4095`), saturating at `4095`.
    /// CPU-owned descriptors only (see [`Self::store_flags`]).
    #[inline]
    pub fn set_size(&self, size: u16) {
        self.update_flags(Self::SIZE_MASK, size.min(0x0FFF) as u32);
    }

    /// Returns the 12-bit `length` field (`0..=4095`).
    #[inline]
    pub fn length(&self) -> u16 {
        ((self.flags() & Self::LENGTH_MASK) >> Self::LENGTH_SHIFT) as u16
    }

    /// Sets the 12-bit `length` field (`0..=4095`), saturating at `4095`.
    /// CPU-owned descriptors only (see [`Self::store_flags`]).
    #[inline]
    pub fn set_length(&self, length: u16) {
        self.update_flags(
            Self::LENGTH_MASK,
            (length.min(0x0FFF) as u32) << Self::LENGTH_SHIFT,
        );
    }

    /// Returns the 5-bit `offset` field (`0..=31`).
    #[inline]
    pub fn offset(&self) -> u8 {
        ((self.flags() & Self::OFFSET_MASK) >> Self::OFFSET_SHIFT) as u8
    }

    /// Sets the 5-bit `offset` field (`0..=31`), saturating at `31`.
    /// CPU-owned descriptors only (see [`Self::store_flags`]).
    #[inline]
    pub fn set_offset(&self, offset: u8) {
        self.update_flags(
            Self::OFFSET_MASK,
            (offset.min(0x1F) as u32) << Self::OFFSET_SHIFT,
        );
    }

    /// Returns the `sosf` (start of sub-frame) flag (`bit 29`).
    #[inline]
    pub fn sosf(&self) -> bool {
        (self.flags() & Self::SOSF_BIT) != 0
    }

    /// Sets the `sosf` (start of sub-frame) flag (`bit 29`).
    /// CPU-owned descriptors only (see [`Self::store_flags`]).
    #[inline]
    pub fn set_sosf(&self, sosf: bool) {
        self.update_flags(Self::SOSF_BIT, if sosf { Self::SOSF_BIT } else { 0 });
    }

    /// Returns the `eof` (end of frame) flag (`bit 30`).
    #[inline]
    pub fn eof(&self) -> bool {
        (self.flags() & Self::EOF_BIT) != 0
    }

    /// Sets the `eof` (end of frame) flag (`bit 30`).
    /// CPU-owned descriptors only (see [`Self::store_flags`]).
    #[inline]
    pub fn set_eof(&self, eof: bool) {
        self.update_flags(Self::EOF_BIT, if eof { Self::EOF_BIT } else { 0 });
    }

    /// Returns the `owner` flag (`bit 31`, `true` = owned by DMA hardware).
    #[inline]
    pub fn owner(&self) -> bool {
        (self.flags() & Self::OWNER_BIT) != 0
    }

    /// Sets the `owner` flag (`bit 31`, `true` = owned by DMA hardware).
    /// CPU-owned descriptors only (see [`Self::store_flags`]).
    #[inline]
    pub fn set_owner(&self, owner: bool) {
        self.update_flags(Self::OWNER_BIT, if owner { Self::OWNER_BIT } else { 0 });
    }

    /// Returns the 20-bit physical descriptor address (`addr & 0x000F_FFFF`) used
    /// by ESP32 `I2S0.in_link.addr`.
    ///
    /// # Hardware Addressing
    /// Note that ESP32 I2S DMA uses a 20-bit link address (`addr & 0x000F_FFFF`),
    /// where internal SRAM descriptors reside in `0x3FF8_0000..=0x3FFF_FFFF`, and
    /// the hardware automatically assumes the high address bits. In contrast,
    /// ESP32-S3 GDMA uses a full 32-bit descriptor address.
    #[inline]
    pub fn link_addr_20bit(&self) -> u32 {
        (ptr::from_ref(self) as usize as u32) & 0x000F_FFFF
    }
}

/// Returns `true` if `len > 0` bytes at `addr` lie entirely inside the ESP32
/// DMA-capable internal SRAM window
/// ([`DMA_CAPABLE_START`]`..=`[`DMA_CAPABLE_END`]).
#[inline]
#[must_use]
pub const fn is_dma_capable(addr: usize, len: usize) -> bool {
    if len == 0 || addr < DMA_CAPABLE_START {
        return false;
    }
    match addr.checked_add(len - 1) {
        Some(last) => last <= DMA_CAPABLE_END,
        None => false,
    }
}

/// Addresses and geometry of a descriptor ring, as checked by
/// [`validate_dma_ring`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DmaRingLayout {
    /// Address of the first [`Lldesc`].
    pub desc_addr: usize,
    /// Number of descriptors (one per node).
    pub desc_count: usize,
    /// Address of the DMA buffer.
    pub buf_addr: usize,
    /// Length of the DMA buffer in bytes (at least `desc_count * node_size`).
    pub buf_len: usize,
    /// Bytes per descriptor node.
    pub node_size: usize,
}

/// Validates a descriptor ring layout (LL-01). A pure function on addresses,
/// so it is unit-tested on the host with target addresses.
///
/// - [`CameraError::InvalidArg`] unless `desc_count >= 2`, `node_size` is
///   non-zero, a multiple of 4 and at most [`LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE`]
///   (`4092`), and both the descriptors and the buffer are 4-byte aligned.
/// - [`CameraError::BufferTooSmall`] when `buf_len < desc_count * node_size`.
/// - With `require_dma_window`, [`CameraError::InvalidArg`] unless the
///   descriptors and the `desc_count * node_size` bytes of buffer the ring
///   uses all lie in the DMA-capable window (no PSRAM, no peripheral space).
///   [`DmaRing::new`] sets it on Xtensa.
/// - Address arithmetic that would overflow is [`CameraError::InvalidArg`].
pub const fn validate_dma_ring(
    layout: &DmaRingLayout,
    require_dma_window: bool,
) -> Result<(), CameraError> {
    let l = layout;
    if l.desc_count < 2
        || l.node_size == 0
        || l.node_size > LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE
        || !l.node_size.is_multiple_of(4)
        || !l.desc_addr.is_multiple_of(4)
        || !l.buf_addr.is_multiple_of(4)
    {
        return Err(CameraError::InvalidArg);
    }
    let Some(ring_bytes) = l.desc_count.checked_mul(l.node_size) else {
        return Err(CameraError::Overflow);
    };
    let Some(desc_bytes) = l.desc_count.checked_mul(DESC_BYTES) else {
        return Err(CameraError::Overflow);
    };
    if l.buf_len < ring_bytes {
        return Err(CameraError::BufferTooSmall);
    }
    if l.buf_addr.checked_add(ring_bytes).is_none() || l.desc_addr.checked_add(desc_bytes).is_none()
    {
        return Err(CameraError::InvalidArg);
    }
    if require_dma_window
        && !(is_dma_capable(l.desc_addr, desc_bytes) && is_dma_capable(l.buf_addr, ring_bytes))
    {
        return Err(CameraError::InvalidArg);
    }
    Ok(())
}

/// A circular descriptor ring over a DMA buffer, built by a checked
/// constructor (LL-01).
///
/// [`DmaRing::new`] validates alignment, node size and (on Xtensa) that the
/// descriptors and the buffer lie in DMA-capable internal SRAM, then links
/// the ring. [`crate::Esp32I2sCamera::start`] borrows a `DmaRing<'static>`
/// mutably for the lifetime of the returned [`crate::RunningCapture`], so
/// the CPU cannot touch, move or drop the ring while the DMA writes into it.
///
/// If a `RunningCapture` is leaked (`mem::forget`), the ring stays *armed*:
/// every accessor and every later start fail with
/// [`CameraError::InvalidState`], because the DMA may still be writing.
#[derive(Debug)]
pub struct DmaRing<'a> {
    descriptors: &'a mut [Lldesc],
    buffer: &'a mut [u8],
    node_size: usize,
    armed: bool,
}

impl<'a> DmaRing<'a> {
    /// Validates the layout ([`validate_dma_ring`], with the DMA window check
    /// on Xtensa only, so host tests can build rings over host memory) and
    /// links a circular ring of `descriptors.len()` nodes of `node_size`
    /// bytes over the start of `buffer`. Every descriptor is DMA-owned with
    /// `length == 0`. The ring covers exactly `descriptors.len() * node_size`
    /// bytes; any rest of `buffer` is not part of it.
    pub fn new(
        descriptors: &'a mut [Lldesc],
        buffer: &'a mut [u8],
        node_size: usize,
    ) -> Result<Self, CameraError> {
        let layout = DmaRingLayout {
            desc_addr: descriptors.as_ptr() as usize,
            desc_count: descriptors.len(),
            buf_addr: buffer.as_ptr() as usize,
            buf_len: buffer.len(),
            node_size,
        };
        validate_dma_ring(&layout, cfg!(target_arch = "xtensa"))?;
        let ring_bytes = layout.desc_count * node_size;
        let buffer = &mut buffer[..ring_bytes];
        build_circular_ring(descriptors, buffer, node_size)?;
        Ok(Self {
            descriptors,
            buffer,
            node_size,
            armed: false,
        })
    }

    /// Number of descriptors (nodes).
    #[inline]
    pub fn desc_count(&self) -> usize {
        self.descriptors.len()
    }

    /// Bytes per node.
    #[inline]
    pub const fn node_size(&self) -> usize {
        self.node_size
    }

    /// Bytes of DMA buffer in the ring (`desc_count * node_size`).
    #[inline]
    pub fn len_bytes(&self) -> usize {
        self.buffer.len()
    }

    /// 20-bit `in_link` address of the first descriptor.
    #[inline]
    pub fn first_desc_addr(&self) -> u32 {
        self.descriptors[0].link_addr_20bit()
    }

    /// `true` while a [`crate::RunningCapture`] over this ring is live, or
    /// after one was leaked.
    #[inline]
    pub const fn is_armed(&self) -> bool {
        self.armed
    }

    /// The descriptors, for inspection. [`CameraError::InvalidState`] while
    /// the ring is armed.
    pub fn descriptors(&self) -> Result<&[Lldesc], CameraError> {
        if self.armed {
            return Err(CameraError::InvalidState);
        }
        Ok(self.descriptors)
    }

    /// The descriptors and the buffer, e.g. for
    /// [`crate::HalfDrain::drain_tail_half`] after the DMA was stopped.
    /// [`CameraError::InvalidState`] while the ring is armed.
    pub fn parts_mut(&mut self) -> Result<(&mut [Lldesc], &mut [u8]), CameraError> {
        if self.armed {
            return Err(CameraError::InvalidState);
        }
        Ok((self.descriptors, self.buffer))
    }

    /// Fills the whole ring buffer with [`crate::DMA_POISON_BYTE`] (LL-06).
    pub fn poison(&mut self) -> Result<(), CameraError> {
        let (_, buf) = self.parts_mut()?;
        buf.fill(crate::dma_drain::DMA_POISON_BYTE);
        Ok(())
    }

    /// Re-links the ring and hands every descriptor back to the DMA
    /// (`owner = 1`, `length = 0`, no `eof`), as before each start.
    pub fn rearm(&mut self) -> Result<(), CameraError> {
        let node_size = self.node_size;
        let (desc, buf) = self.parts_mut()?;
        build_circular_ring(desc, buf, node_size).map(|_| ())
    }

    /// Splits the ring for a [`crate::RunningCapture`]: arms it and returns
    /// raw pointers to the descriptors and the buffer plus the armed flag.
    pub(crate) fn arm(&mut self) -> Result<RingParts<'_>, CameraError> {
        if self.armed {
            return Err(CameraError::InvalidState);
        }
        self.armed = true;
        Ok(RingParts {
            descriptors: ptr::NonNull::from(&mut *self.descriptors),
            buffer: ptr::NonNull::from(&mut *self.buffer),
            node_size: self.node_size,
            armed: &mut self.armed,
        })
    }
}

/// Raw view of an armed [`DmaRing`], held by [`crate::RunningCapture`].
#[derive(Debug)]
pub(crate) struct RingParts<'r> {
    pub(crate) descriptors: ptr::NonNull<[Lldesc]>,
    pub(crate) buffer: ptr::NonNull<[u8]>,
    pub(crate) node_size: usize,
    pub(crate) armed: &'r mut bool,
}

#[repr(C, align(4))]
struct AlignedBytes<const N: usize>([u8; N]);

/// `'static` storage for one [`DmaRing`] of `NODES` descriptors over a
/// 4-byte-aligned `BYTES`-byte buffer, handed out at most once.
///
/// Declare it as a `static` so it lands in internal SRAM (`.bss`), which is
/// DMA-capable, and take the ring with [`Self::take_ring`]:
///
/// ```
/// use esp32_cam_ll::DmaRingStorage;
/// static RING: DmaRingStorage<4, 512> = DmaRingStorage::new();
/// let ring = RING.take_ring(128).expect("first take");
/// assert_eq!(ring.len_bytes(), 512);
/// assert!(RING.take_ring(128).is_err());
/// ```
pub struct DmaRingStorage<const NODES: usize, const BYTES: usize> {
    taken: AtomicBool,
    descriptors: UnsafeCell<[Lldesc; NODES]>,
    buffer: UnsafeCell<AlignedBytes<BYTES>>,
}

// JUSTIFICATION: The cells are only reached through `take_ring`, which hands
// out the single `&'static mut` view after winning the `taken` flag.
// SAFETY:
// 1. Alignment & Non-null: Not applicable; no pointer is created here.
// 2. Memory layout: `UnsafeCell<T>` has the layout of `T`.
// 3. Provenance & Lifetime: The storage is used as a `static`.
// 4. Exclusive aliasing: `take_ring` swaps `taken` to `true` with `AcqRel`
//    before creating the only mutable references, so at most one thread (or
//    core) ever obtains them; no shared reference to the cells exists.
// 5. Volatile semantics: Not applicable.
// 6. Panic safety: Not applicable.
unsafe impl<const NODES: usize, const BYTES: usize> Sync for DmaRingStorage<NODES, BYTES> {}

impl<const NODES: usize, const BYTES: usize> Default for DmaRingStorage<NODES, BYTES> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const NODES: usize, const BYTES: usize> fmt::Debug for DmaRingStorage<NODES, BYTES> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("DmaRingStorage")
            .field("nodes", &NODES)
            .field("bytes", &BYTES)
            .field("taken", &self.taken.load(Ordering::Relaxed))
            .finish()
    }
}

impl<const NODES: usize, const BYTES: usize> DmaRingStorage<NODES, BYTES> {
    /// Creates zeroed, untaken storage.
    pub const fn new() -> Self {
        Self {
            taken: AtomicBool::new(false),
            descriptors: UnsafeCell::new([Lldesc::EMPTY; NODES]),
            buffer: UnsafeCell::new(AlignedBytes([0; BYTES])),
        }
    }

    /// Builds the [`DmaRing`] over this storage with `node_size`-byte nodes.
    ///
    /// The first call claims the storage for good, even when the ring
    /// geometry is then rejected; later calls fail with
    /// [`CameraError::InvalidState`].
    pub fn take_ring(&'static self, node_size: usize) -> Result<DmaRing<'static>, CameraError> {
        if self.taken.swap(true, Ordering::AcqRel) {
            return Err(CameraError::InvalidState);
        }
        // JUSTIFICATION: Hands out the one mutable view of the storage.
        // SAFETY:
        // 1. Alignment & Non-null: `UnsafeCell::get` of a `static` is non-null
        //    and aligned for `[Lldesc; NODES]` and `AlignedBytes<BYTES>`.
        // 2. Memory layout: The cells hold initialized arrays.
        // 3. Provenance & Lifetime: `self` is `&'static`, so the references
        //    are valid for `'static`.
        // 4. Exclusive aliasing: This call won the `taken` swap above, so no
        //    other reference to either cell exists or will ever be created.
        // 5. Volatile semantics: Not applicable.
        // 6. Panic safety: Creating references cannot unwind.
        let (descriptors, buffer) =
            unsafe { (&mut *self.descriptors.get(), &mut (*self.buffer.get()).0) };
        DmaRing::new(descriptors, buffer, node_size)
    }
}

/// Computed DMA buffer, half-buffer, and descriptor node dimensions
/// (`cam_obj_t` DMA fields).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DmaSizeConfig {
    pub dma_bytes_per_item: usize,
    pub dma_buffer_size: usize,
    pub dma_half_buffer_size: usize,
    pub dma_half_buffer_cnt: usize,
    pub dma_node_buffer_size: usize,
    pub dma_node_cnt: usize,
    pub lines_per_node: usize,
    pub nodes_per_line: usize,
    pub lines_per_half_buffer: usize,
    pub image_size: usize,
}

/// Computes RGB/YUV/Grayscale DMA node, half-buffer, and full-buffer sizes
/// (`ll_cam_calc_rgb_dma`
/// in `target/esp32/ll_cam.c:470-545`), enforcing zero-division and 4 MiB
/// bounds (`DMA-101`..`105`).
pub fn calc_rgb_dma_sizes(
    width: u16,
    height: u16,
    in_bytes_per_pixel: u8,
    dma_bytes_per_item: usize,
    dma_buffer_size_max: usize,
) -> Result<DmaSizeConfig, CameraError> {
    if width == 0
        || height == 0
        || in_bytes_per_pixel == 0
        || dma_bytes_per_item == 0
        || dma_buffer_size_max == 0
    {
        return Err(CameraError::InvalidArg);
    }

    let width_usize = width as usize;
    let height_usize = height as usize;
    let bpp_usize = in_bytes_per_pixel as usize;

    let dma_half_buffer_max = dma_buffer_size_max / 2 / dma_bytes_per_item;
    let dma_buffer_max = 2 * dma_half_buffer_max;
    let node_cap = if dma_buffer_size_max < 4096 {
        (dma_buffer_size_max.clamp(512, 4092) & !3).min(LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE)
    } else {
        LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE
    };
    let node_max = node_cap / dma_bytes_per_item;

    if dma_half_buffer_max == 0 || node_max == 0 {
        return Err(CameraError::InvalidArg);
    }

    let line_width = width_usize
        .checked_mul(bpp_usize)
        .ok_or(CameraError::Overflow)?;
    let image_size = height_usize
        .checked_mul(line_width)
        .ok_or(CameraError::Overflow)?;

    if image_size > MAX_DMA_IMAGE_SIZE || line_width > dma_half_buffer_max {
        return Err(CameraError::InvalidArg);
    }

    let mut best_node_samples = 0usize;
    let mut nodes_per_line = 1usize;
    let mut lines_per_node = 1usize;

    if line_width >= node_max {
        let mut d = node_max;
        while d > 0 {
            if (d * dma_bytes_per_item).is_multiple_of(4) && line_width.is_multiple_of(d) {
                best_node_samples = d;
                nodes_per_line = line_width / best_node_samples;
                break;
            }
            d -= 1;
        }
    } else {
        let mut d = node_max;
        while d > 0 {
            if (d * dma_bytes_per_item).is_multiple_of(4) && d.is_multiple_of(line_width) {
                let mut lpn = d / line_width;
                while lpn > 1 && !height_usize.is_multiple_of(lpn) {
                    lpn -= 1;
                }
                let cand_samples = lpn * line_width;
                if (cand_samples * dma_bytes_per_item).is_multiple_of(4) {
                    best_node_samples = cand_samples;
                    lines_per_node = lpn;
                    break;
                }
            }
            d -= 1;
        }
    }

    if best_node_samples == 0 {
        return Err(CameraError::InvalidArg);
    }
    let node_size = best_node_samples;

    let dma_half_buffer_min = node_size
        .checked_mul(nodes_per_line)
        .ok_or(CameraError::Overflow)?;
    if dma_half_buffer_min == 0 {
        return Err(CameraError::InvalidArg);
    }

    let mut dma_half_buffer = (dma_half_buffer_max / dma_half_buffer_min) * dma_half_buffer_min;
    let mut lines_per_half_buffer = dma_half_buffer / line_width;

    while lines_per_half_buffer > 1
        && dma_half_buffer >= dma_half_buffer_min + line_width
        && !height_usize.is_multiple_of(lines_per_half_buffer)
    {
        dma_half_buffer -= dma_half_buffer_min;
        lines_per_half_buffer = dma_half_buffer / line_width;
    }

    if dma_half_buffer == 0 {
        return Err(CameraError::InvalidArg);
    }

    let dma_buffer_items = (dma_buffer_max / dma_half_buffer) * dma_half_buffer;
    if dma_buffer_items == 0 {
        return Err(CameraError::InvalidArg);
    }

    let dma_buffer_size = dma_buffer_items
        .checked_mul(dma_bytes_per_item)
        .ok_or(CameraError::Overflow)?;
    let dma_half_buffer_size = dma_half_buffer
        .checked_mul(dma_bytes_per_item)
        .ok_or(CameraError::Overflow)?;
    let dma_node_buffer_size = node_size
        .checked_mul(dma_bytes_per_item)
        .ok_or(CameraError::Overflow)?;

    if dma_half_buffer_size == 0 || dma_node_buffer_size == 0 {
        return Err(CameraError::InvalidArg);
    }

    let dma_half_buffer_cnt = dma_buffer_size / dma_half_buffer_size;
    let dma_node_cnt = dma_buffer_size / dma_node_buffer_size;

    Ok(DmaSizeConfig {
        dma_bytes_per_item,
        dma_buffer_size,
        dma_half_buffer_size,
        dma_half_buffer_cnt,
        dma_node_buffer_size,
        dma_node_cnt,
        lines_per_node,
        nodes_per_line,
        lines_per_half_buffer,
        image_size,
    })
}

/// Computes DMA buffer geometry for either JPEG or uncompressed modes on ESP32
/// (`ll_cam_dma_sizes`
/// in `target/esp32/ll_cam.c:547-559`).
pub fn calc_dma_sizes(
    jpeg_mode: bool,
    sampling_mode: I2sSamplingMode,
    width: u16,
    height: u16,
    in_bytes_per_pixel: u8,
    dma_buffer_size_max: usize,
) -> Result<DmaSizeConfig, CameraError> {
    if width == 0 || height == 0 || dma_buffer_size_max < 512 {
        return Err(CameraError::InvalidArg);
    }
    let dma_bytes_per_item = sampling_mode.bytes_per_sample();
    if jpeg_mode {
        let max_node_cap = if dma_buffer_size_max < 4096 {
            dma_buffer_size_max.clamp(512, 4092) & !3
        } else {
            2048
        };
        let dma_half_buffer_cnt = 8;
        let dma_half_buffer_size = ((dma_buffer_size_max / 2) & !3).max(4);
        let limit = 2048.min(max_node_cap);
        let mut node_size = if limit == 0 {
            4
        } else {
            1usize << (usize::BITS - 1 - limit.leading_zeros())
        };
        while node_size > 0 {
            if dma_half_buffer_size.is_multiple_of(node_size) {
                break;
            }
            node_size >>= 1;
        }
        let dma_node_buffer_size = node_size.max(4);
        let dma_buffer_size = dma_half_buffer_cnt * dma_half_buffer_size;
        let dma_node_cnt = dma_buffer_size
            .checked_div(dma_node_buffer_size)
            .ok_or(CameraError::InvalidArg)?;
        let image_size = (width as usize).saturating_mul(height as usize) / 5;
        Ok(DmaSizeConfig {
            dma_bytes_per_item,
            dma_buffer_size,
            dma_half_buffer_size,
            dma_half_buffer_cnt,
            dma_node_buffer_size,
            dma_node_cnt,
            lines_per_node: 1,
            nodes_per_line: 1,
            lines_per_half_buffer: 1,
            image_size,
        })
    } else {
        calc_rgb_dma_sizes(
            width,
            height,
            in_bytes_per_pixel,
            dma_bytes_per_item,
            dma_buffer_size_max,
        )
    }
}

/// Initializes a circular linked-list ring of [`Lldesc`] descriptors over
/// the mutable `buffer` the DMA writes into (`allocate_dma_descriptors` in
/// `driver/cam_hal.c:465-482`), returning the descriptor count.
///
/// LL-02: this is the only builder; it takes `&mut [u8]`, so the `buf`
/// pointers it stores carry write provenance. Prefer [`DmaRing::new`], which
/// also checks the DMA window on Xtensa.
///
/// Validates that:
/// - `descriptors.len() >= 2`,
/// - `node_size` is non-zero, `<= LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE` (`4092`),
///   and a multiple of 4,
/// - `descriptors` and `buffer` are 4-byte aligned,
/// - `buffer.len() >= descriptors.len() * node_size`.
pub fn build_circular_ring(
    descriptors: &mut [Lldesc],
    buffer: &mut [u8],
    node_size: usize,
) -> Result<usize, CameraError> {
    let count = descriptors.len();
    validate_dma_ring(
        &DmaRingLayout {
            desc_addr: descriptors.as_ptr() as usize,
            desc_count: count,
            buf_addr: buffer.as_ptr() as usize,
            buf_len: buffer.len(),
            node_size,
        },
        false,
    )?;

    let buf_base = buffer.as_mut_ptr();
    let desc_base = descriptors.as_mut_ptr();
    let flags = Lldesc::pack_flags(node_size as u16, 0, 0, false, false, true);

    let mut x = 0;
    while x < count {
        let next_idx = (x + 1) % count;
        let byte_offset = x * node_size;

        // JUSTIFICATION: Initializing descriptor linked list and slice offsets.
        // SAFETY:
        // 1. Alignment & Non-null: `buf_base` is derived from a live
        //    `&mut [u8]` and `desc_base` from `descriptors: &mut [Lldesc]`;
        //    both were checked 4-byte aligned above.
        // 2. Memory layout: `Lldesc` is `#[repr(C, align(4))]`.
        // 3. Provenance & Bounds: `byte_offset = x * node_size < count *
        //    node_size <= buffer.len()`, and `next_idx < count ==
        //    descriptors.len()`, staying within the allocations.
        // 4. Aliasing: The pointers are only stored in descriptor fields for
        //    the DMA engine; nothing is dereferenced here.
        // 5. Volatile semantics: Not applicable (address arithmetic only).
        // 6. Panic safety: Pointer arithmetic cannot unwind.
        let (chunk_ptr, next_desc_ptr) =
            unsafe { (buf_base.add(byte_offset), desc_base.add(next_idx)) };

        descriptors[x] = Lldesc {
            size_length_flags: AtomicU32::new(flags),
            buf: chunk_ptr,
            qe_stqe_next: next_desc_ptr,
        };
        x += 1;
    }

    Ok(count)
}

/// Verifies that a slice of [`Lldesc`] descriptors forms a valid circular ring
/// over `buffer`
/// with each node having `size == expected_node_size`, `length == 0`, `owner ==
/// true`, `eof == false`,
/// `sosf == false`, and `offset == 0`.
pub fn verify_circular_ring(
    descriptors: &[Lldesc],
    buffer: &[u8],
    expected_node_size: usize,
) -> Result<(), CameraError> {
    let count = descriptors.len();
    if count < 2
        || expected_node_size == 0
        || expected_node_size > LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE
        || !expected_node_size.is_multiple_of(4)
    {
        return Err(CameraError::InvalidArg);
    }
    let buf_base = buffer.as_ptr();
    let desc_base = descriptors.as_ptr();

    if ((buf_base as usize) & 0x3 != 0) || ((desc_base as usize) & 0x3 != 0) {
        return Err(CameraError::DmaRingCorrupted);
    }

    let required = count
        .checked_mul(expected_node_size)
        .ok_or(CameraError::Overflow)?;
    if buffer.len() < required {
        return Err(CameraError::BufferTooSmall);
    }

    let mut x = 0;
    while x < count {
        let d = &descriptors[x];
        if d.size() as usize != expected_node_size
            || d.length() != 0
            || !d.owner()
            || d.eof()
            || d.sosf()
            || d.offset() != 0
        {
            return Err(CameraError::InvalidState);
        }
        if ((d.buf as usize) & 0x3 != 0) || ((d.qe_stqe_next as usize) & 0x3 != 0) {
            return Err(CameraError::DmaRingCorrupted);
        }
        let next_idx = (x + 1) % count;

        // JUSTIFICATION: Verifying descriptor pointer continuity and alignment.
        // SAFETY:
        // 1. Alignment & Non-null: `buf_base` and `desc_base` come from live slice
        //    references.
        // 2. Provenance & Bounds: `x * expected_node_size < required <= buffer.len()`
        //    and `next_idx < count`,
        //    so pointer arithmetic remains within the allocated slice bounds.
        let (expected_buf, expected_next) = unsafe {
            (
                buf_base.add(x * expected_node_size),
                desc_base.add(next_idx),
            )
        };

        if !ptr::eq(d.buf, expected_buf) || !ptr::eq(d.qe_stqe_next, expected_next) {
            return Err(CameraError::InvalidState);
        }
        x += 1;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::mem::align_of;

    #[test]
    fn test_lldesc_bitfield_packing_and_alignment() {
        assert!(align_of::<Lldesc>() >= 4);

        let desc = Lldesc::empty();
        desc.set_size(2048);
        desc.set_length(1024);
        desc.set_offset(17);
        desc.set_sosf(true);
        desc.set_eof(false);
        desc.set_owner(true);

        assert_eq!(desc.size(), 2048);
        assert_eq!(desc.length(), 1024);
        assert_eq!(desc.offset(), 17);
        assert!(desc.sosf());
        assert!(!desc.eof());
        assert!(desc.owner());

        desc.set_eof(true);
        desc.set_owner(false);
        assert!(desc.eof());
        assert!(!desc.owner());

        // Saturation tests for size, length, and offset (LL-006)
        desc.set_size(5000);
        assert_eq!(desc.size(), 4095);
        desc.set_length(5000);
        assert_eq!(desc.length(), 4095);
        desc.set_offset(50);
        assert_eq!(desc.offset(), 31);

        let saturated_flags = Lldesc::pack_flags(6000, 7000, 100, true, true, true);
        assert_eq!(saturated_flags & Lldesc::SIZE_MASK, 0x0FFF);
        assert_eq!(
            (saturated_flags & Lldesc::LENGTH_MASK) >> Lldesc::LENGTH_SHIFT,
            0x0FFF
        );
        assert_eq!(
            (saturated_flags & Lldesc::OFFSET_MASK) >> Lldesc::OFFSET_SHIFT,
            0x1F
        );
    }

    #[repr(C, align(4))]
    struct AlignedBuffer<const N: usize>([u8; N]);

    #[test]
    fn test_circular_ring_builder_and_verifier() {
        let mut buf = AlignedBuffer([0u8; 4096]);
        let mut ring = [Lldesc::EMPTY; 4];

        // LL-02: the only builder takes `&mut [u8]` (SEC-011).
        assert_eq!(build_circular_ring(&mut ring, &mut buf.0, 1024), Ok(4));
        assert_eq!(verify_circular_ring(&ring, &buf.0, 1024), Ok(()));

        // Verify circular wrap-around from last descriptor back to index 0
        assert!(ptr::eq(ring[3].qe_stqe_next, &ring[0]));
        assert!(ptr::eq(ring[0].buf, buf.0.as_ptr()));

        // Reject count < 2 (LOGIC-013)
        let mut single_desc = [Lldesc::EMPTY; 1];
        assert_eq!(
            build_circular_ring(&mut single_desc, &mut buf.0, 1024),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(
            verify_circular_ring(&single_desc, &buf.0, 1024),
            Err(CameraError::InvalidArg)
        );

        // Misaligned buffer tests
        assert_eq!(
            build_circular_ring(&mut ring, &mut buf.0[1..], 1024),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(
            verify_circular_ring(&ring, &buf.0[1..], 1024),
            Err(CameraError::DmaRingCorrupted)
        );

        // Reject invalid parameters (LL-007)
        assert_eq!(
            build_circular_ring(&mut ring, &mut buf.0, 0),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(
            build_circular_ring(&mut ring, &mut buf.0, 4093),
            Err(CameraError::InvalidArg)
        );
        // Reject non-multiple of 4
        assert_eq!(
            build_circular_ring(&mut ring, &mut buf.0, 1023),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(
            verify_circular_ring(&ring, &buf.0, 1023),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(
            build_circular_ring(&mut ring, &mut buf.0[..2048], 1024),
            Err(CameraError::BufferTooSmall)
        );
        // Restore a valid ring for the verifier checks below.
        assert_eq!(build_circular_ring(&mut ring, &mut buf.0, 1024), Ok(4));

        // Reject corrupt descriptor flags in verifier
        ring[1].set_eof(true);
        assert_eq!(
            verify_circular_ring(&ring, &buf.0, 1024),
            Err(CameraError::InvalidState)
        );
        ring[1].set_eof(false);
        ring[2].set_sosf(true);
        assert_eq!(
            verify_circular_ring(&ring, &buf.0, 1024),
            Err(CameraError::InvalidState)
        );
        ring[2].set_sosf(false);
        ring[0].set_offset(1);
        assert_eq!(
            verify_circular_ring(&ring, &buf.0, 1024),
            Err(CameraError::InvalidState)
        );
    }

    /// A valid internal-SRAM layout: 16 descriptors at `0x3FFB_0000` and a
    /// 32 KiB buffer at `0x3FFC_0000` with 2048-byte nodes.
    const fn sram_layout() -> DmaRingLayout {
        DmaRingLayout {
            desc_addr: 0x3FFB_0000,
            desc_count: 16,
            buf_addr: 0x3FFC_0000,
            buf_len: 32768,
            node_size: 2048,
        }
    }

    #[test]
    fn validator_accepts_an_internal_sram_ring() {
        assert_eq!(validate_dma_ring(&sram_layout(), true), Ok(()));
        // The window bounds themselves are inclusive.
        let edge = DmaRingLayout {
            desc_addr: DMA_CAPABLE_START,
            desc_count: 2,
            buf_addr: DMA_CAPABLE_END + 1 - 2 * 4092,
            buf_len: 2 * 4092,
            node_size: 4092,
        };
        assert_eq!(validate_dma_ring(&edge, true), Ok(()));
    }

    #[test]
    fn validator_rejects_a_psram_buffer() {
        let psram = DmaRingLayout {
            buf_addr: 0x3F80_0000,
            ..sram_layout()
        };
        assert_eq!(
            validate_dma_ring(&psram, true),
            Err(CameraError::InvalidArg)
        );
        // Without the window check (host rings) the same layout passes.
        assert_eq!(validate_dma_ring(&psram, false), Ok(()));
    }

    #[test]
    fn validator_rejects_psram_descriptors() {
        let psram = DmaRingLayout {
            desc_addr: 0x3F80_1000,
            ..sram_layout()
        };
        assert_eq!(
            validate_dma_ring(&psram, true),
            Err(CameraError::InvalidArg)
        );
    }

    #[test]
    fn validator_rejects_rings_leaving_the_dma_window() {
        // Starts one word below the window.
        let below = DmaRingLayout {
            buf_addr: DMA_CAPABLE_START - 4,
            ..sram_layout()
        };
        assert_eq!(
            validate_dma_ring(&below, true),
            Err(CameraError::InvalidArg)
        );
        // Starts inside the window but runs past its end.
        let straddle = DmaRingLayout {
            buf_addr: 0x3FFF_C000,
            ..sram_layout()
        };
        assert_eq!(
            validate_dma_ring(&straddle, true),
            Err(CameraError::InvalidArg)
        );
        // Descriptors straddling the end of the window.
        let desc_straddle = DmaRingLayout {
            desc_addr: DMA_CAPABLE_END + 1 - 8 * DESC_BYTES,
            ..sram_layout()
        };
        assert_eq!(
            validate_dma_ring(&desc_straddle, true),
            Err(CameraError::InvalidArg)
        );
        // Peripheral space (I2S0 itself) is not DMA-capable memory.
        let mmio = DmaRingLayout {
            desc_addr: 0x3FF4_F000,
            ..sram_layout()
        };
        assert_eq!(validate_dma_ring(&mmio, true), Err(CameraError::InvalidArg));
        // Address arithmetic that would wrap is rejected, not wrapped.
        let wrap = DmaRingLayout {
            buf_addr: usize::MAX - 3,
            ..sram_layout()
        };
        assert_eq!(
            validate_dma_ring(&wrap, false),
            Err(CameraError::InvalidArg)
        );
    }

    #[test]
    fn validator_rejects_misaligned_rings() {
        for (desc_addr, buf_addr) in [(0x3FFB_0002, 0x3FFC_0000), (0x3FFB_0000, 0x3FFC_0001)] {
            let l = DmaRingLayout {
                desc_addr,
                buf_addr,
                ..sram_layout()
            };
            assert_eq!(validate_dma_ring(&l, true), Err(CameraError::InvalidArg));
            assert_eq!(validate_dma_ring(&l, false), Err(CameraError::InvalidArg));
        }
    }

    #[test]
    fn validator_rejects_bad_node_sizes_and_counts() {
        for node_size in [0, 2, 1022, 4093, 4094, 4096] {
            let l = DmaRingLayout {
                node_size,
                buf_len: 16 * 4096,
                ..sram_layout()
            };
            assert_eq!(
                validate_dma_ring(&l, false),
                Err(CameraError::InvalidArg),
                "node_size {node_size}"
            );
        }
        let single = DmaRingLayout {
            desc_count: 1,
            ..sram_layout()
        };
        assert_eq!(
            validate_dma_ring(&single, false),
            Err(CameraError::InvalidArg)
        );
        let short = DmaRingLayout {
            buf_len: 32764,
            ..sram_layout()
        };
        assert_eq!(
            validate_dma_ring(&short, false),
            Err(CameraError::BufferTooSmall)
        );
    }

    #[test]
    fn is_dma_capable_checks_the_whole_range() {
        assert!(is_dma_capable(0x3FFA_E000, 4));
        assert!(is_dma_capable(0x3FFF_FFFC, 4));
        assert!(!is_dma_capable(0x3FFF_FFFC, 8));
        assert!(!is_dma_capable(0x3FFA_DFFC, 4));
        assert!(!is_dma_capable(0x3F80_0000, 4));
        assert!(!is_dma_capable(0x3FFB_0000, 0));
    }

    #[test]
    fn dma_ring_rejects_a_misaligned_host_buffer() {
        let mut desc = [Lldesc::EMPTY; 4];
        let mut buf = AlignedBuffer([0u8; 4100]);
        assert!(matches!(
            DmaRing::new(&mut desc, &mut buf.0[1..], 1024),
            Err(CameraError::InvalidArg)
        ));
        assert!(matches!(
            DmaRing::new(&mut desc, &mut buf.0[..4096], 1023),
            Err(CameraError::InvalidArg)
        ));
        assert!(matches!(
            DmaRing::new(&mut desc, &mut buf.0[..2048], 1024),
            Err(CameraError::BufferTooSmall)
        ));
    }

    #[test]
    fn dma_ring_builds_a_writable_circular_ring() {
        let mut desc = [Lldesc::EMPTY; 4];
        let mut buf = AlignedBuffer([0u8; 4100]);
        let mut ring = DmaRing::new(&mut desc, &mut buf.0, 1024).expect("ring");
        assert_eq!(ring.desc_count(), 4);
        assert_eq!(ring.node_size(), 1024);
        assert_eq!(ring.len_bytes(), 4096);
        {
            let (d, b) = ring.parts_mut().expect("idle ring");
            assert_eq!(b.len(), 4096, "the ring covers exactly count * node_size");
            assert_eq!(verify_circular_ring(d, b, 1024), Ok(()));
            assert_eq!(d[0].link_addr_20bit(), ring_first_addr(d));
        }
        // LL-02: the DMA writes through `desc.buf`. Do the same from the CPU:
        // the pointer must carry write provenance for the buffer.
        {
            let (d, _) = ring.parts_mut().expect("idle ring");
            let p = d[1].buf;
            // JUSTIFICATION: Simulates the DMA engine writing into node 1.
            // SAFETY:
            // 1. Alignment & Non-null: `p` is node 1's start inside `buf`.
            // 2. Memory layout: `u8` has no alignment requirement.
            // 3. Provenance & Lifetime: `p` was derived from the `&mut [u8]`
            //    the ring borrows, which is live.
            // 4. Aliasing: No reference to the buffer is live here; `d`
            //    borrows only the descriptors.
            // 5. Volatile semantics: A plain write, as the test reads it back.
            // 6. Panic safety: A byte store cannot unwind.
            unsafe { p.add(3).write(0xAB) };
        }
        let (_, b) = ring.parts_mut().expect("idle ring");
        assert_eq!(b[1024 + 3], 0xAB);

        ring.poison().expect("idle ring");
        let (d, b) = ring.parts_mut().expect("idle ring");
        assert!(b.iter().all(|&x| x == crate::dma_drain::DMA_POISON_BYTE));
        d[2].store_flags(0);
        ring.rearm().expect("idle ring");
        let (d, b) = ring.parts_mut().expect("idle ring");
        assert_eq!(verify_circular_ring(d, b, 1024), Ok(()));
    }

    fn ring_first_addr(d: &[Lldesc]) -> u32 {
        (ptr::from_ref(&d[0]) as usize as u32) & 0x000F_FFFF
    }

    #[test]
    fn lldesc_header_is_one_shared_word() {
        assert_eq!(core::mem::size_of::<AtomicU32>(), 4);
        assert_eq!(core::mem::offset_of!(Lldesc, size_length_flags), 0);
        let d = Lldesc::EMPTY;
        // Setters take `&self`: the header is shared with the DMA engine.
        d.store_flags(Lldesc::pack_flags(2048, 7, 0, false, true, false));
        assert_eq!(
            d.flags(),
            Lldesc::pack_flags(2048, 7, 0, false, true, false)
        );
        d.set_owner(true);
        assert!(d.owner());
        assert_eq!(d.length(), 7);
        assert!(d.eof());
    }

    #[test]
    fn test_calc_dma_sizes_jpeg_and_rgb_and_guards() {
        // JPEG mode
        let jpeg_cfg = calc_dma_sizes(
            true,
            I2sSamplingMode::Sm0A000B00,
            640,
            480,
            1,
            DEFAULT_CAMERA_DMA_BUFFER_SIZE_MAX,
        )
        .expect("valid jpeg dma config");
        assert_eq!(jpeg_cfg.dma_half_buffer_cnt, 8);
        assert_eq!(jpeg_cfg.dma_node_buffer_size, 2048);
        assert_eq!(jpeg_cfg.dma_half_buffer_size, 16384);
        assert_eq!(jpeg_cfg.dma_buffer_size, 8 * 16384);
        assert_eq!(jpeg_cfg.dma_node_cnt, (8 * 16384) / 2048);

        // JPEG clamping to dma_buffer_size_max / 2 (LL-008)
        let jpeg_clamped = calc_dma_sizes(true, I2sSamplingMode::Sm0A000B00, 640, 480, 1, 4096)
            .expect("valid clamped jpeg config");
        assert_eq!(jpeg_clamped.dma_half_buffer_size, 2048);
        assert_eq!(jpeg_clamped.dma_buffer_size, 8 * 2048);

        // Reject width == 0, height == 0, or dma_buffer_size_max < 512 in JPEG mode
        assert_eq!(
            calc_dma_sizes(true, I2sSamplingMode::Sm0A000B00, 0, 480, 1, 32768),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(
            calc_dma_sizes(true, I2sSamplingMode::Sm0A000B00, 640, 0, 1, 32768),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(
            calc_dma_sizes(true, I2sSamplingMode::Sm0A000B00, 640, 480, 1, 511),
            Err(CameraError::InvalidArg)
        );

        let sub_4k_cfg = calc_dma_sizes(true, I2sSamplingMode::Sm0A000B00, 640, 480, 1, 2048)
            .expect("valid sub-4k jpeg config");
        assert!(sub_4k_cfg.dma_node_buffer_size <= 2048);

        // QVGA RGB565 (320x240, 2 bpp, 2 bytes per item in Sm0A0B0C0D)
        let qvga_cfg = calc_dma_sizes(
            false,
            I2sSamplingMode::Sm0A0B0C0D,
            320,
            240,
            2,
            DEFAULT_CAMERA_DMA_BUFFER_SIZE_MAX,
        )
        .expect("valid qvga rgb565 dma config");
        assert_eq!(qvga_cfg.image_size, 320 * 240 * 2);
        assert!(qvga_cfg.dma_node_buffer_size <= LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE);
        assert_eq!(qvga_cfg.dma_buffer_size % qvga_cfg.dma_half_buffer_size, 0);
        assert_eq!(qvga_cfg.dma_buffer_size % qvga_cfg.dma_node_buffer_size, 0);

        // Zero-division guards (DMA-101..105)
        assert_eq!(
            calc_rgb_dma_sizes(0, 240, 2, 2, DEFAULT_CAMERA_DMA_BUFFER_SIZE_MAX),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(
            calc_rgb_dma_sizes(320, 0, 2, 2, DEFAULT_CAMERA_DMA_BUFFER_SIZE_MAX),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(
            calc_rgb_dma_sizes(320, 240, 0, 2, DEFAULT_CAMERA_DMA_BUFFER_SIZE_MAX),
            Err(CameraError::InvalidArg)
        );
        assert_eq!(
            calc_rgb_dma_sizes(320, 240, 2, 0, DEFAULT_CAMERA_DMA_BUFFER_SIZE_MAX),
            Err(CameraError::InvalidArg)
        );

        // Exceeds 4 MiB guard
        assert_eq!(
            calc_rgb_dma_sizes(2592, 1944, 2, 2, DEFAULT_CAMERA_DMA_BUFFER_SIZE_MAX),
            Err(CameraError::InvalidArg)
        );
    }

    #[test]
    fn test_calc_rgb_dma_sizes_enforces_4_byte_aligned_node_buffer() {
        // When width * bpp and height cannot form any 4-byte-aligned DMA node buffer,
        // calc_rgb_dma_sizes must reject with InvalidArg instead of producing an
        // unaligned node_size (LOGIC-008).
        // e.g. width = 3, height = 1, in_bytes_per_pixel = 1, dma_bytes_per_item = 1
        // (line_width = 3, height = 1)
        assert_eq!(
            calc_rgb_dma_sizes(3, 1, 1, 1, 8192),
            Err(CameraError::InvalidArg)
        );

        // e.g. line_width >= node_max with prime unaligned width (e.g. 4093)
        assert_eq!(
            calc_rgb_dma_sizes(4093, 1, 1, 1, 32768),
            Err(CameraError::InvalidArg)
        );

        // When a valid 4-byte-aligned configuration is provided, dma_node_buffer_size
        // must be a multiple of 4
        // and build_circular_ring must accept it.
        let cfg = calc_rgb_dma_sizes(4, 1, 1, 1, 8192).expect("valid aligned rgb dma config");
        assert_eq!(cfg.dma_node_buffer_size % 4, 0);

        let mut ring = [Lldesc::EMPTY; 2];
        let mut buf = AlignedBuffer([0u8; 8192]);
        assert_eq!(
            build_circular_ring(
                &mut ring,
                &mut buf.0[..2 * cfg.dma_node_buffer_size],
                cfg.dma_node_buffer_size
            ),
            Ok(2)
        );
    }
}
