//! Wi-Fi MAC DMA descriptor ring and TX slot buffer management.
//!
//! Implements hardware DMA ring buffers for 802.11 frame reception and
//! transmission on the ESP32 Wi-Fi MAC. All descriptors and buffers are
//! 4-byte aligned as required by the Xtensa DMA controller and Miri invariants,
//! use volatile descriptor accesses with Acquire/Release/`memw` barriers, and
//! store DMA-shared state in `UnsafeCell` with `PhantomPinned`.

use core::cell::UnsafeCell;
use core::fmt;
use core::marker::PhantomPinned;
use core::pin::Pin;
use core::ptr;
use core::sync::atomic::fence;
use core::sync::atomic::Ordering;

/// Length in bytes of the hardware `rx_ctrl` descriptor prepended to
/// each received 802.11 MPDU buffer by the MAC DMA engine.
pub const RX_CTRL_LEN: usize = 28;

/// Maximum 4-byte aligned DMA buffer length representable in the 12-bit
/// `lldesc_t` `size` and `length` fields (`0xFFC = 4092`).
pub const MAX_DMA_BUF_LEN: usize = 0xFFC;

/// Issues an Acquire memory fence (plus Xtensa `memw`) after observing a
/// hardware-to-software ownership handoff (`OWNER_HW == 0`).
#[inline]
fn dma_acquire_barrier() {
    fence(Ordering::Acquire);
    #[cfg(target_arch = "xtensa")]
    {
        // JUSTIFICATION: Xtensa requires `memw` to order descriptor ownership
        // reads before CPU reads of DMA-written RX payload buffers.
        // SAFETY: `memw` only drains/orders processor memory operations.
        unsafe {
            core::arch::asm!("memw", options(nostack, preserves_flags));
        }
    }
}

/// Issues a Release memory fence (plus Xtensa `memw`) before publishing
/// software-to-hardware ownership (`OWNER_HW = 1`).
#[inline]
fn dma_release_barrier() {
    #[cfg(target_arch = "xtensa")]
    {
        // JUSTIFICATION: Xtensa requires `memw` to drain CPU buffer and
        // descriptor writes to DRAM before publishing OWNER_HW to DMA.
        // SAFETY: `memw` only drains/orders processor memory operations.
        unsafe {
            core::arch::asm!("memw", options(nostack, preserves_flags));
        }
    }
    fence(Ordering::Release);
}

/// Validates a physical DMA address and converts it to a 32-bit hardware
/// descriptor pointer.
///
/// On Xtensa targets, enforces non-truncating `u32::try_from`, 4-byte
/// alignment, and ESP32 internal DMA DRAM (`0x3FFA_E000..=0x3FFF_FFFF`).
/// On host targets, enforces non-null and 4-byte alignment.
#[inline]
fn validate_dma_ptr(addr: usize) -> Result<u32, DmaRingError> {
    if addr == 0 {
        return Err(DmaRingError::InvalidDmaAddress);
    }
    if (addr & 0x3) != 0 {
        return Err(DmaRingError::InvalidAlignment);
    }

    #[cfg(target_arch = "xtensa")]
    {
        let addr32 = u32::try_from(addr).map_err(|_| DmaRingError::InvalidDmaAddress)?;
        if !crate::regs::is_valid_xtensa_dma_addr(addr32) {
            return Err(DmaRingError::InvalidDmaAddress);
        }
        Ok(addr32)
    }
    #[cfg(not(target_arch = "xtensa"))]
    {
        let addr32 = addr as u32;
        if addr32 == 0 {
            return Err(DmaRingError::InvalidDmaAddress);
        }
        Ok(addr32)
    }
}

/// Errors returned by DMA ring and slot operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DmaRingError {
    /// Provided buffer or slice is too short (or empty).
    BufferTooShort,
    /// MPDU payload or buffer capacity exceeds the 12-bit DMA limit (`0xFFC`).
    BufferTooLarge,
    /// The RX ring has no completed frames available for consumption.
    RingEmpty,
    /// The TX slot is currently in-flight and cannot accept a new transmission.
    TxBusy,
    /// The ring capacity `N` is zero or invalid.
    InvalidRingSize,
    /// The ring has not been armed with hardware links.
    NotArmed,
    /// DMA buffer length or pointer is not 4-byte aligned.
    InvalidAlignment,
    /// DMA address is null or outside ESP32 internal DRAM bounds.
    InvalidDmaAddress,
    /// Cannot re-arm the ring while descriptors are still owned by hardware DMA.
    RingBusy,
}

impl fmt::Display for DmaRingError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::BufferTooShort => f.write_str("buffer too short"),
            Self::BufferTooLarge => f.write_str("payload or buffer exceeds DMA capacity"),
            Self::RingEmpty => f.write_str("no completed RX descriptors available"),
            Self::TxBusy => f.write_str("TX slot is busy"),
            Self::InvalidRingSize => f.write_str("invalid ring descriptor count"),
            Self::NotArmed => f.write_str("DMA ring is not armed"),
            Self::InvalidAlignment => f.write_str("DMA buffer or address is not 4-byte aligned"),
            Self::InvalidDmaAddress => f.write_str("DMA address outside valid internal DRAM"),
            Self::RingBusy => f.write_str("DMA ring has descriptors owned by hardware"),
        }
    }
}

/// Parsed metadata from the 28-byte hardware `rx_ctrl` header prepended to
/// every received frame buffer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WifiRxCtrl {
    /// Received signal strength indication in dBm (-128..=127).
    pub rssi: i8,
    /// Hardware PHY rate code.
    pub rate: u8,
    /// Physical layer signal mode (0 = 11b, 1 = 11g, 2 = 11n).
    pub sig_mode: u8,
    /// Modulation and Coding Scheme index (0..=7).
    pub mcs: u8,
    /// Channel width (false = 20 MHz, true = 40 MHz).
    pub cwb: bool,
    /// A-MPDU aggregation indicator.
    pub aggregation: bool,
    /// Short Guard Interval flag.
    pub sgi: bool,
    /// Measured RF noise floor in dBm.
    pub noise_floor: i8,
    /// A-MPDU subframe count.
    pub ampdu_cnt: u8,
    /// Primary radio channel index.
    pub channel: u8,
    /// Secondary radio channel index.
    pub secondary_channel: u8,
    /// Hardware TSF timestamp in microseconds.
    pub timestamp_us: u32,
    /// Antenna path used for reception (0 or 1).
    pub ant: u8,
    /// Physical layer signal length in bytes.
    pub sig_len: u16,
    /// Receiver hardware state byte.
    pub rx_state: u8,
}

impl WifiRxCtrl {
    /// Parses a `WifiRxCtrl` header from an array of seven 32-bit words (28 bytes).
    ///
    /// The bitfield layout corresponds to the reverse-engineered ESP32 `libpp.a`
    /// hardware specification:
    /// - `w0[7:0]`: `rssi` (signed `i8`)
    /// - `w0[12:8]`: `rate` (5 bits)
    /// - `w0[15:14]`: `sig_mode` (2 bits)
    /// - `w1[6:0]`: `mcs` (7 bits)
    /// - `w1[7]`: `cwb` (1 bit)
    /// - `w1[27]`: `aggregation` (1 bit)
    /// - `w1[31]`: `sgi` (1 bit)
    /// - `w2[7:0]`: `noise_floor` (signed `i8`)
    /// - `w2[15:8]`: `ampdu_cnt` (8 bits)
    /// - `w2[19:16]`: `channel` (4 bits)
    /// - `w2[23:20]`: `secondary_channel` (4 bits)
    /// - `w3`: `timestamp_us` (32 bits)
    /// - `w5[31]`: `ant` (1 bit)
    /// - `w6[11:0]`: `sig_len` (12 bits)
    /// - `w6[31:24]`: `rx_state` (8 bits)
    #[must_use]
    pub const fn parse(words: &[u32; 7]) -> Self {
        let w0 = words[0];
        let w1 = words[1];
        let w2 = words[2];
        let w3 = words[3];
        let w5 = words[5];
        let w6 = words[6];

        let rssi = (w0 & 0xFF) as u8 as i8;
        let rate = ((w0 >> 8) & 0x1F) as u8;
        let sig_mode = ((w0 >> 14) & 0x3) as u8;

        let mcs = (w1 & 0x7F) as u8;
        let cwb = (w1 & (1 << 7)) != 0;
        let aggregation = (w1 & (1 << 27)) != 0;
        let sgi = (w1 & (1 << 31)) != 0;

        let noise_floor = (w2 & 0xFF) as u8 as i8;
        let ampdu_cnt = ((w2 >> 8) & 0xFF) as u8;
        let channel = ((w2 >> 16) & 0xF) as u8;
        let secondary_channel = ((w2 >> 20) & 0xF) as u8;

        let timestamp_us = w3;
        let ant = ((w5 >> 31) & 1) as u8;

        let sig_len = (w6 & 0xFFF) as u16;
        let rx_state = ((w6 >> 24) & 0xFF) as u8;

        Self {
            rssi,
            rate,
            sig_mode,
            mcs,
            cwb,
            aggregation,
            sgi,
            noise_floor,
            ampdu_cnt,
            channel,
            secondary_channel,
            timestamp_us,
            ant,
            sig_len,
            rx_state,
        }
    }

    /// Parses a `WifiRxCtrl` header from a byte slice of at least
    /// [`RX_CTRL_LEN`] bytes.
    pub fn parse_slice(bytes: &[u8]) -> Result<Self, DmaRingError> {
        if bytes.len() < RX_CTRL_LEN {
            return Err(DmaRingError::BufferTooShort);
        }

        let mut words = [0u32; 7];
        let mut i = 0;
        while i < 7 {
            let offset = i * 4;
            let b = [
                bytes[offset],
                bytes[offset + 1],
                bytes[offset + 2],
                bytes[offset + 3],
            ];
            words[i] = u32::from_le_bytes(b);
            i += 1;
        }

        Ok(Self::parse(&words))
    }
}

/// 4-byte aligned hardware RX DMA descriptor (12 bytes).
///
/// Matches the ESP32 Wi-Fi MAC DMA descriptor memory layout:
/// - `word0`: Hardware status and ownership flags.
///   - Bit 31: Owner (1 = Hardware/DMA, 0 = Software/CPU). On ESP32 Wi-Fi RX
///     silicon, hardware leaves bit 31 set to `1` after writing a frame and
///     signals completion via bit 30 (`EOF_BIT` / `has_data`).
///   - Bit 30: EOF (`has_data`, End of Frame — set to `1` on RX completion).
///   - Bit 29: SOF (Start of Frame — left `0` by ESP32 Wi-Fi RX silicon).
///   - Bits 23..12: `length`, received DMA length in bytes (4-byte padded).
///   - Bits 11..0: `size`, buffer capacity in bytes.
/// - `buf_ptr`: Physical address of 4-byte aligned payload buffer.
/// - `next_desc`: Physical address of next descriptor in circular ring.
#[repr(C, align(4))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WifiRxDmaDesc {
    /// Control and status word.
    pub word0: u32,
    /// Physical 32-bit address of data buffer.
    pub buf_ptr: u32,
    /// Physical 32-bit address of next descriptor.
    pub next_desc: u32,
}

impl WifiRxDmaDesc {
    /// Hardware ownership bit (`1 << 31`).
    pub const OWNER_HW: u32 = 1 << 31;
    /// End-of-frame flag bit (`1 << 30`).
    pub const EOF_BIT: u32 = 1 << 30;
    /// Start-of-frame flag bit (`1 << 29`).
    pub const SOF_BIT: u32 = 1 << 29;

    /// Creates an empty descriptor owned by software.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            word0: 0,
            buf_ptr: 0,
            next_desc: 0,
        }
    }

    #[inline]
    fn read_word0_volatile(&self) -> u32 {
        // JUSTIFICATION: DMA descriptors are mutated asynchronously by the
        // ESP32 SLC DMA engine; volatile read prevents compiler hoisting.
        // SAFETY: `self.word0` is a valid, 4-byte aligned `u32` owned by `self`.
        unsafe { ptr::read_volatile(ptr::addr_of!(self.word0)) }
    }

    #[inline]
    fn write_word0_volatile(&mut self, val: u32) {
        // JUSTIFICATION: Publishing descriptor control bits to hardware DMA
        // requires a volatile store so the write cannot be elided or reordered.
        // SAFETY: `self.word0` is a valid, 4-byte aligned `u32` uniquely borrowed.
        unsafe {
            ptr::write_volatile(ptr::addr_of_mut!(self.word0), val);
        }
    }

    /// Returns `true` if the descriptor is currently owned by hardware/DMA.
    #[inline]
    #[must_use]
    pub fn is_owner_hw(&self) -> bool {
        (self.read_word0_volatile() & Self::OWNER_HW) != 0
    }

    /// Returns `true` if the descriptor is currently owned by software/CPU.
    #[inline]
    #[must_use]
    pub fn is_owner_sw(&self) -> bool {
        !self.is_owner_hw()
    }

    /// Returns `true` if this descriptor contains the end of a received frame.
    #[inline]
    #[must_use]
    pub fn is_eof(&self) -> bool {
        (self.read_word0_volatile() & Self::EOF_BIT) != 0
    }

    /// Returns `true` if this descriptor contains the start of a received frame.
    #[inline]
    #[must_use]
    pub fn is_sof(&self) -> bool {
        (self.read_word0_volatile() & Self::SOF_BIT) != 0
    }

    /// Returns the received DMA length from the `lldesc_t` `length` field,
    /// bits [23:12]. Hardware pads this up to a 4-byte boundary.
    #[inline]
    #[must_use]
    pub fn payload_len(&self) -> u16 {
        ((self.read_word0_volatile() >> 12) & 0xFFF) as u16
    }

    /// Returns the buffer capacity from the `lldesc_t` `size` field,
    /// bits [11:0].
    #[inline]
    #[must_use]
    pub fn buffer_cap(&self) -> u16 {
        (self.read_word0_volatile() & 0xFFF) as u16
    }

    /// Sets ownership to hardware/DMA with a Release barrier.
    #[inline]
    pub fn set_owner_hw(&mut self) {
        let cur = self.read_word0_volatile();
        dma_release_barrier();
        self.write_word0_volatile(cur | Self::OWNER_HW);
    }

    /// Sets ownership to software/CPU with an Acquire barrier.
    #[inline]
    pub fn set_owner_sw(&mut self) {
        let cur = self.read_word0_volatile();
        self.write_word0_volatile(cur & !Self::OWNER_HW);
        dma_acquire_barrier();
    }

    /// Prepares and arms this descriptor for hardware DMA reception with
    /// the given validated 4-byte aligned capacity (`RX_CTRL_LEN..=0xFFC`).
    ///
    /// Matches vendor `wDev_AppendRxBlocks`: the capacity goes in both the
    /// `size` field (bits [11:0]) and the `length` field (bits [23:12]).
    #[inline]
    pub fn arm(&mut self, capacity: u16) -> Result<(), DmaRingError> {
        let cap_usize = usize::from(capacity);
        if cap_usize < RX_CTRL_LEN {
            return Err(DmaRingError::BufferTooShort);
        }
        if cap_usize > MAX_DMA_BUF_LEN {
            return Err(DmaRingError::BufferTooLarge);
        }
        if (cap_usize & 0x3) != 0 {
            return Err(DmaRingError::InvalidAlignment);
        }

        let cap = u32::from(capacity);
        dma_release_barrier();
        self.write_word0_volatile(Self::OWNER_HW | (cap << 12) | cap);
        Ok(())
    }
}

impl Default for WifiRxDmaDesc {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

/// 4-byte aligned hardware TX DMA descriptor (12 bytes).
///
/// Matches the ESP32 Wi-Fi MAC TX DMA descriptor memory layout:
/// - `ctrl`: Control word (`0xC000_0000 | (len << 12) | cap`), the ROM
///   `lldesc_t` layout.
///   - Bit 31: Owner (1 = Hardware/DMA, 0 = Software/CPU).
///   - Bit 30: EOF (End of Frame).
///   - Bit 29: SOSF (clear for single-buffer vendor TX).
///   - Bits 23..12: `length`, packet payload length in bytes.
///   - Bits 11..0: `size`, buffer capacity in bytes.
/// - `buf_ptr`: Physical address of 4-byte aligned payload buffer.
/// - `next_desc`: Physical address of next descriptor (0 for single frame).
#[repr(C, align(4))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WifiTxDmaDesc {
    /// Control and status word.
    pub ctrl: u32,
    /// Physical 32-bit address of data buffer.
    pub buf_ptr: u32,
    /// Physical 32-bit address of next descriptor.
    pub next_desc: u32,
}

impl WifiTxDmaDesc {
    /// Combined control bits: Owner (bit 31) and EOF (bit 30). SOSF
    /// (bit 29) stays clear, as in single-buffer vendor TX.
    pub const CTRL_FLAGS: u32 = 0xC000_0000;

    /// Creates an empty TX descriptor owned by software.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            ctrl: 0,
            buf_ptr: 0,
            next_desc: 0,
        }
    }

    #[inline]
    fn read_ctrl_volatile(&self) -> u32 {
        // JUSTIFICATION: TX DMA descriptors are cleared asynchronously by
        // hardware; volatile read prevents compiler hoisting in poll loops.
        // SAFETY: `self.ctrl` is a valid, 4-byte aligned `u32` owned by `self`.
        unsafe { ptr::read_volatile(ptr::addr_of!(self.ctrl)) }
    }

    /// Returns `true` if the descriptor is currently owned by hardware/DMA.
    #[inline]
    #[must_use]
    pub fn is_owner_hw(&self) -> bool {
        (self.read_ctrl_volatile() & 0x8000_0000) != 0
    }

    /// Returns `true` if the descriptor is currently owned by software/CPU.
    #[inline]
    #[must_use]
    pub fn is_owner_sw(&self) -> bool {
        !self.is_owner_hw()
    }

    /// Returns the configured payload length from the `lldesc_t` `length`
    /// field, bits [23:12].
    #[inline]
    #[must_use]
    pub fn payload_len(&self) -> u16 {
        ((self.read_ctrl_volatile() >> 12) & 0xFFF) as u16
    }

    /// Returns the buffer capacity from the `lldesc_t` `size` field,
    /// bits [11:0].
    #[inline]
    #[must_use]
    pub fn buffer_cap(&self) -> u16 {
        (self.read_ctrl_volatile() & 0xFFF) as u16
    }

    /// Prepares the TX descriptor for single-frame hardware transmission.
    ///
    /// Writes `buf_ptr` and `next_desc` first, issues a Release/`memw` barrier,
    /// and publishes `ctrl` (containing `OWNER_HW`) last so DMA never observes
    /// a half-initialized descriptor.
    #[inline]
    pub fn prepare(&mut self, buf_addr: u32, cap: u16, len: u16) -> Result<(), DmaRingError> {
        let cap_usize = usize::from(cap);
        let len_usize = usize::from(len);
        if cap_usize == 0 || len_usize == 0 {
            return Err(DmaRingError::BufferTooShort);
        }
        if cap_usize > MAX_DMA_BUF_LEN || len_usize > cap_usize {
            return Err(DmaRingError::BufferTooLarge);
        }
        if buf_addr == 0 {
            return Err(DmaRingError::InvalidDmaAddress);
        }
        if (buf_addr & 0x3) != 0 {
            return Err(DmaRingError::InvalidAlignment);
        }
        #[cfg(target_arch = "xtensa")]
        if !crate::regs::is_valid_xtensa_dma_addr(buf_addr) {
            return Err(DmaRingError::InvalidDmaAddress);
        }

        let len_bits = u32::from(len) << 12;
        let cap_bits = u32::from(cap);
        let ctrl_word = Self::CTRL_FLAGS | len_bits | cap_bits;

        // JUSTIFICATION: Hardware TX DMA reads `buf_ptr` and `next_desc`
        // immediately upon seeing `OWNER_HW` in `ctrl`; volatile writes ordered
        // by a Release barrier guarantee pointer visibility before handoff.
        // SAFETY: Fields belong to uniquely borrowed `self`, 4-byte aligned.
        unsafe {
            ptr::write_volatile(ptr::addr_of_mut!(self.buf_ptr), buf_addr);
            ptr::write_volatile(ptr::addr_of_mut!(self.next_desc), 0);
        }
        dma_release_barrier();
        // JUSTIFICATION: Final volatile publish of TX descriptor control word
        // with OWNER_HW after payload and pointer writes are ordered.
        // SAFETY: `self.ctrl` is a valid 4-byte aligned field of `self`.
        unsafe {
            ptr::write_volatile(ptr::addr_of_mut!(self.ctrl), ctrl_word);
        }
        Ok(())
    }
}

impl Default for WifiTxDmaDesc {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

/// 4-byte aligned payload buffer wrapper.
#[repr(C, align(4))]
#[derive(Clone, Copy)]
pub struct DmaBuffer<const LEN: usize>(pub [u8; LEN]);

impl<const LEN: usize> DmaBuffer<LEN> {
    /// Creates a zeroed 4-byte aligned DMA buffer.
    #[must_use]
    pub const fn new() -> Self {
        Self([0u8; LEN])
    }

    /// Returns the buffer as an immutable byte slice.
    #[inline]
    #[must_use]
    pub fn as_slice(&self) -> &[u8] {
        &self.0
    }

    /// Returns the buffer as a mutable byte slice.
    #[inline]
    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        &mut self.0
    }
}

impl<const LEN: usize> Default for DmaBuffer<LEN> {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

/// Outcome of inspecting or popping the current head descriptor of
/// [`WifiRxRing`].
///
/// Explicitly separates an empty ring (`OWNER_HW == 1 && EOF_BIT == 0`, or
/// unarmed) from a malformed descriptor (`M-071` / `M-089`) so callers always
/// recycle malformed slots instead of wedging the RX ring.
#[derive(Debug, PartialEq, Eq)]
pub enum RxPop<'a> {
    /// Head descriptor is still armed and waiting for hardware DMA
    /// (`OWNER_HW == 1 && EOF_BIT == 0`), or ring is unarmed/empty.
    Empty,
    /// Head descriptor failed integrity checks (`EOF_BIT == 0` with
    /// `OWNER_HW == 0`, out-of-range `desc_len`, or invalid `rx_ctrl` header).
    /// Caller MUST call [`WifiRxRing::recycle_current`].
    Malformed,
    /// Valid received frame with parsed hardware `rx_ctrl` and payload slice.
    Frame(WifiRxCtrl, &'a [u8]),
}

/// Circular hardware RX DMA ring holding `N` descriptors and `N` aligned
/// buffers.
///
/// DMA-shared descriptor and buffer storage is wrapped in `UnsafeCell` and
/// marked `!Unpin` (`PhantomPinned`) to prevent accidental relocation after
/// hardware pointer arming.
#[repr(C, align(4))]
pub struct WifiRxRing<const N: usize, const BUF_LEN: usize> {
    descs: UnsafeCell<[WifiRxDmaDesc; N]>,
    buffers: UnsafeCell<[DmaBuffer<BUF_LEN>; N]>,
    head: usize,
    armed: bool,
    _pin: PhantomPinned,
}

impl<const N: usize, const BUF_LEN: usize> WifiRxRing<N, BUF_LEN> {
    /// Creates an unlinked, zeroed RX DMA ring.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            descs: UnsafeCell::new([WifiRxDmaDesc::new(); N]),
            buffers: UnsafeCell::new([DmaBuffer::new(); N]),
            head: 0,
            armed: false,
            _pin: PhantomPinned,
        }
    }

    #[inline]
    fn desc_ptr(&self, idx: usize) -> *mut WifiRxDmaDesc {
        // JUSTIFICATION: Accessing individual DMA descriptors via raw pointer
        // derived from `UnsafeCell` avoids creating Rust references over
        // hardware-owned slots.
        // SAFETY: Caller has verified `idx < N`.
        unsafe { self.descs.get().cast::<WifiRxDmaDesc>().add(idx) }
    }

    #[inline]
    fn read_desc_word0(&self, idx: usize) -> u32 {
        let p = self.desc_ptr(idx);
        // JUSTIFICATION: Descriptor `word0` is asynchronously updated by DMA;
        // volatile read prevents compiler hoisting.
        // SAFETY: `idx < N` is verified by callers and `word0` is aligned.
        unsafe { ptr::read_volatile(ptr::addr_of!((*p).word0)) }
    }

    #[inline]
    fn desc_is_owner_hw(&self, idx: usize) -> bool {
        (self.read_desc_word0(idx) & WifiRxDmaDesc::OWNER_HW) != 0
    }

    /// Returns `true` if descriptor `idx` is armed (`OWNER_HW == 1`) and still
    /// waiting for hardware RX completion (`EOF_BIT == 0` / `has_data == 0`).
    #[inline]
    fn desc_is_hw_armed(&self, idx: usize) -> bool {
        let word0 = self.read_desc_word0(idx);
        (word0 & WifiRxDmaDesc::OWNER_HW) != 0 && (word0 & WifiRxDmaDesc::EOF_BIT) == 0
    }

    /// Arms a pinned circular descriptor ring in-place, linking descriptor
    /// addresses and assigning buffer pointers.
    pub fn arm_ring_pinned(self: Pin<&mut Self>) -> Result<(), DmaRingError> {
        // JUSTIFICATION: `arm_ring` initializes internal self-referential DMA
        // pointers in-place; `self: Pin<&mut Self>` enforces address stability.
        // SAFETY: `arm_ring` never moves `self` out of its pinned location.
        let this = unsafe { self.get_unchecked_mut() };
        // JUSTIFICATION: Pinning guarantees `this` will not be relocated while
        // hardware DMA holds pointers to its descriptors and buffers.
        // SAFETY: Caller pinned `self` via `Pin<&mut Self>`.
        unsafe { this.arm_ring() }
    }

    /// Arms the circular descriptor ring in-place, linking descriptor addresses
    /// and assigning buffer pointers.
    ///
    /// Prefer [`Self::arm_ring_pinned`] when holding a `Pin<&mut Self>`.
    ///
    /// # Safety
    ///
    /// This method stores self-referential physical pointers (`buf_ptr` and
    /// `next_desc`) into hardware DMA descriptors. The caller MUST guarantee
    /// that `self` remains pinned at a fixed memory address (never moved or
    /// dropped while armed) for as long as hardware RX DMA is enabled.
    // JUSTIFICATION: Hardware RX DMA requires self-referential physical ring
    // addresses; exposing an explicit `unsafe` entry point lets pinned static
    // drivers arm in-place when `Pin<&mut Self>` cannot cross trait bounds.
    pub unsafe fn arm_ring(&mut self) -> Result<(), DmaRingError> {
        if N == 0 {
            return Err(DmaRingError::InvalidRingSize);
        }
        if BUF_LEN < RX_CTRL_LEN {
            return Err(DmaRingError::BufferTooShort);
        }
        if BUF_LEN > MAX_DMA_BUF_LEN {
            return Err(DmaRingError::BufferTooLarge);
        }
        if (BUF_LEN & 0x3) != 0 {
            return Err(DmaRingError::InvalidAlignment);
        }

        // Prevent re-arming over descriptors currently owned by live DMA.
        if self.armed {
            let mut idx = 0;
            while idx < N {
                if self.desc_is_owner_hw(idx) {
                    return Err(DmaRingError::RingBusy);
                }
                idx += 1;
            }
        }

        let descs_base = self.descs.get() as usize;
        let desc_stride = core::mem::size_of::<WifiRxDmaDesc>();
        let cap_u32 = BUF_LEN as u32;
        let armed_word0 = WifiRxDmaDesc::OWNER_HW | (cap_u32 << 12) | cap_u32;

        // Validate all descriptor and buffer addresses before mutating state.
        let mut v = 0;
        while v < N {
            let desc_addr = descs_base
                .checked_add(v * desc_stride)
                .ok_or(DmaRingError::InvalidDmaAddress)?;
            let _ = validate_dma_ptr(desc_addr)?;

            // JUSTIFICATION: Computing the physical address of buffer slot `v`
            // from `UnsafeCell` for pre-arm address validation.
            // SAFETY: `v < N` is strictly within `[DmaBuffer<BUF_LEN>; N]`.
            let buf_ptr = unsafe { self.buffers.get().cast::<DmaBuffer<BUF_LEN>>().add(v) };
            let _ = validate_dma_ptr(buf_ptr as usize)?;
            v += 1;
        }

        let mut i = 0;
        while i < N {
            let next_idx = (i + 1) % N;
            let next_desc_addr = validate_dma_ptr(descs_base + next_idx * desc_stride)?;
            // JUSTIFICATION: Accessing buffer slot `i` via `UnsafeCell` raw
            // pointer while all slots are SW-owned (checked above).
            // SAFETY: `i < N`; no hardware DMA owns slot `i`.
            let buf_slot = unsafe { self.buffers.get().cast::<DmaBuffer<BUF_LEN>>().add(i) };
            let buf_addr = validate_dma_ptr(buf_slot as usize)?;

            // Scrub rx_ctrl header prefix before exposing buffer to DMA.
            // JUSTIFICATION: Volatile byte-zeroing ensures stale rx_ctrl words
            // cannot survive in memory before DMA ownership handoff.
            // SAFETY: `RX_CTRL_LEN <= BUF_LEN` was verified; `buf_slot` valid.
            unsafe {
                let bptr = buf_slot.cast::<u8>();
                let mut b = 0;
                while b < RX_CTRL_LEN {
                    ptr::write_volatile(bptr.add(b), 0);
                    b += 1;
                }
            }

            let dptr = self.desc_ptr(i);
            // JUSTIFICATION: Write DMA pointer fields before publishing OWNER.
            // SAFETY: `dptr` points to valid, 4-byte aligned descriptor `i`.
            unsafe {
                ptr::write_volatile(ptr::addr_of_mut!((*dptr).buf_ptr), buf_addr);
                ptr::write_volatile(ptr::addr_of_mut!((*dptr).next_desc), next_desc_addr);
            }
            dma_release_barrier();
            // JUSTIFICATION: Publish OWNER_HW and capacity after pointers and
            // header scrub are ordered by Release barrier.
            // SAFETY: `dptr` points to valid, 4-byte aligned descriptor `i`.
            unsafe {
                ptr::write_volatile(ptr::addr_of_mut!((*dptr).word0), armed_word0);
            }
            i += 1;
        }

        self.head = 0;
        self.armed = true;
        Ok(())
    }

    /// Returns `true` if the ring has been armed.
    #[inline]
    #[must_use]
    pub const fn is_armed(&self) -> bool {
        self.armed
    }

    /// Returns the current consumer head index.
    #[inline]
    #[must_use]
    pub const fn head_index(&self) -> usize {
        self.head
    }

    /// Returns the physical 32-bit address of the first descriptor
    /// (for `RX_BASE_DSCR_REG`).
    #[inline]
    #[must_use]
    pub fn base_desc_addr(&self) -> u32 {
        self.descs.get() as usize as u32
    }

    /// Returns a volatile by-value snapshot of the descriptor at the current
    /// consumer head, or `None` if `N == 0` or `head >= N`.
    #[inline]
    #[must_use]
    pub fn current_desc(&self) -> Option<WifiRxDmaDesc> {
        self.desc(self.head)
    }

    /// Returns a mutable reference to the descriptor at the current consumer
    /// head if within bounds and currently owned by software.
    #[inline]
    pub fn current_desc_mut(&mut self) -> Option<&mut WifiRxDmaDesc> {
        self.desc_mut(self.head)
    }

    /// Returns a volatile by-value snapshot of a specific descriptor if index
    /// is within bounds, avoiding creating a shared Rust reference over
    /// hardware-mutated DMA descriptor memory (`M-014`).
    #[must_use]
    pub fn desc(&self, idx: usize) -> Option<WifiRxDmaDesc> {
        if idx < N {
            let dptr = self.desc_ptr(idx);
            // JUSTIFICATION: Hardware DMA mutates RX descriptors concurrently;
            // field-wise volatile reads capture a snapshot without creating a
            // Rust `&WifiRxDmaDesc` reference over live DMA memory.
            // SAFETY: `idx < N`; `dptr` is valid and 4-byte aligned.
            let snap = unsafe {
                WifiRxDmaDesc {
                    word0: ptr::read_volatile(ptr::addr_of!((*dptr).word0)),
                    buf_ptr: ptr::read_volatile(ptr::addr_of!((*dptr).buf_ptr)),
                    next_desc: ptr::read_volatile(ptr::addr_of!((*dptr).next_desc)),
                }
            };
            Some(snap)
        } else {
            None
        }
    }

    /// Returns a mutable reference to a specific descriptor if index is
    /// within bounds and the descriptor is not armed and waiting for HW
    /// (`OWNER_HW == 0 || EOF_BIT == 1`).
    pub fn desc_mut(&mut self, idx: usize) -> Option<&mut WifiRxDmaDesc> {
        if idx >= N {
            return None;
        }
        if self.armed && self.desc_is_hw_armed(idx) {
            return None;
        }
        // JUSTIFICATION: Mutable access is gated on `!desc_is_hw_armed(idx)`,
        // ensuring hardware DMA is not actively writing descriptor `idx`.
        // SAFETY: `idx < N`, `&mut self` guarantees exclusive CPU access, and
        // either the ring is unarmed, `OWNER_HW == 0`, or hardware signaled
        // completion via `EOF_BIT == 1` (`has_data`).
        Some(unsafe { &mut *self.desc_ptr(idx) })
    }

    /// Returns an immutable reference to the raw buffer at `idx` if within
    /// bounds and not armed and waiting for hardware
    /// (`OWNER_HW == 0 || EOF_BIT == 1`).
    #[must_use]
    pub fn raw_buffer(&self, idx: usize) -> Option<&[u8]> {
        if idx >= N {
            return None;
        }
        if self.armed && self.desc_is_hw_armed(idx) {
            return None;
        }
        dma_acquire_barrier();
        // JUSTIFICATION: Immutable buffer read is gated on hardware completion
        // (`EOF_BIT == 1` / `has_data`) or `OWNER_HW == 0` plus Acquire barrier.
        // SAFETY: `idx < N` and DMA will not overwrite slot `idx` until re-armed.
        Some(unsafe { &(*self.buffers.get().cast::<DmaBuffer<BUF_LEN>>().add(idx)).0 })
    }

    /// Returns a mutable reference to the raw buffer at `idx` if within bounds
    /// and not armed and waiting for hardware (`OWNER_HW == 0 || EOF_BIT == 1`).
    pub fn raw_buffer_mut(&mut self, idx: usize) -> Option<&mut [u8]> {
        if idx >= N {
            return None;
        }
        if self.armed && self.desc_is_hw_armed(idx) {
            return None;
        }
        dma_acquire_barrier();
        // JUSTIFICATION: Mutable buffer access is gated on `!desc_is_hw_armed(idx)`
        // so hardware DMA cannot concurrently alias this buffer.
        // SAFETY: `idx < N`, `&mut self` ensures exclusive CPU access, and
        // either `OWNER_HW == 0` or `EOF_BIT == 1` (`has_data`) guarantees DMA
        // is not actively writing slot `idx`.
        Some(unsafe { &mut (*self.buffers.get().cast::<DmaBuffer<BUF_LEN>>().add(idx)).0 })
    }

    /// Inspects the frame at the current head without consuming or recycling it.
    ///
    /// Returns [`RxPop::Empty`] if the ring is not armed, `N == 0`, or the
    /// current descriptor is still armed and waiting for hardware DMA
    /// (`EOF_BIT == 0 && OWNER_HW == 1`). Returns [`RxPop::Malformed`] when
    /// `EOF_BIT == 0` with `OWNER_HW == 0` (malformed mock descriptor), or when
    /// `EOF_BIT == 1` (`has_data`) fails `desc_len` or `rx_ctrl` validation so
    /// the caller can call [`Self::recycle_current`] without wedging the ring.
    ///
    /// Note: Real ESP32 Wi-Fi RX silicon leaves `OWNER_HW = 1` and `SOF_BIT = 0`
    /// upon frame completion, signaling completion via `EOF_BIT = 1` (`has_data`).
    #[must_use]
    pub fn peek_received(&self) -> RxPop<'_> {
        if !self.armed || N == 0 || self.head >= N || BUF_LEN < RX_CTRL_LEN {
            return RxPop::Empty;
        }

        let word0 = self.read_desc_word0(self.head);
        if (word0 & WifiRxDmaDesc::EOF_BIT) == 0 {
            if (word0 & WifiRxDmaDesc::OWNER_HW) != 0 {
                return RxPop::Empty;
            }
            return RxPop::Malformed;
        }

        // Order hardware completion check (`EOF_BIT == 1` / `has_data`)
        // before reading DMA-written RX header and payload bytes.
        dma_acquire_barrier();

        let desc_len = ((word0 >> 12) & 0xFFF) as usize;
        if !(RX_CTRL_LEN..=BUF_LEN).contains(&desc_len) {
            return RxPop::Malformed;
        }

        // JUSTIFICATION: Hardware signaled RX frame completion via
        // `EOF_BIT == 1` (`has_data`) and an Acquire barrier executed before
        // borrowing the payload buffer.
        // SAFETY: `self.head < N` and DMA will not overwrite this completed
        // slot until `recycle_current` clears `EOF_BIT` back to armed state.
        let raw_buf: &[u8] = unsafe {
            &(*self
                .buffers
                .get()
                .cast::<DmaBuffer<BUF_LEN>>()
                .add(self.head))
            .0
        };

        let Ok(ctrl) = WifiRxCtrl::parse_slice(&raw_buf[..RX_CTRL_LEN]) else {
            return RxPop::Malformed;
        };

        // Bound `sig_len` strictly to the DMA-written payload bytes
        // (`desc_len - RX_CTRL_LEN`) to prevent stale tail memory disclosure.
        let dma_payload_len = desc_len - RX_CTRL_LEN;
        let sig_len = usize::from(ctrl.sig_len);
        let payload_len = if sig_len > 0 && sig_len <= dma_payload_len {
            sig_len
        } else {
            dma_payload_len
        };

        let payload_end = RX_CTRL_LEN + payload_len;
        let payload = &raw_buf[RX_CTRL_LEN..payload_end];
        RxPop::Frame(ctrl, payload)
    }

    /// Reads the frame at the current head without recycling it.
    ///
    /// On [`RxPop::Frame`] or [`RxPop::Malformed`], the caller MUST call
    /// [`Self::recycle_current`] to return the descriptor to hardware ownership
    /// and advance the ring.
    pub fn pop_received(&mut self) -> RxPop<'_> {
        self.peek_received()
    }

    /// Zeroes consumed header/payload bytes, re-arms the current head
    /// descriptor back to hardware DMA (`OWNER_HW = 1`, `EOF_BIT = 0`) with a
    /// Release barrier, and advances the head pointer to the next slot in the
    /// ring.
    pub fn recycle_current(&mut self) -> Result<usize, DmaRingError> {
        if !self.armed {
            return Err(DmaRingError::NotArmed);
        }
        if N == 0 || self.head >= N {
            return Err(DmaRingError::InvalidRingSize);
        }
        if BUF_LEN < RX_CTRL_LEN {
            return Err(DmaRingError::BufferTooShort);
        }
        if BUF_LEN > MAX_DMA_BUF_LEN {
            return Err(DmaRingError::BufferTooLarge);
        }
        if (BUF_LEN & 0x3) != 0 {
            return Err(DmaRingError::InvalidAlignment);
        }

        let curr = self.head;
        let word0 = self.read_desc_word0(curr);
        if (word0 & WifiRxDmaDesc::EOF_BIT) == 0 && (word0 & WifiRxDmaDesc::OWNER_HW) != 0 {
            return Err(DmaRingError::RingEmpty);
        }

        // Scrub the consumed region (at least RX_CTRL_LEN) to prevent stale
        // frame disclosure if a future descriptor reports a short length.
        let prev_len = ((word0 >> 12) & 0xFFF) as usize;
        let scrub_len = prev_len.max(RX_CTRL_LEN).min(BUF_LEN);
        // JUSTIFICATION: Volatile scrubbing prevents dead-store elimination
        // of buffer sanitization before re-arming the slot for DMA.
        // SAFETY: `curr < N`, `scrub_len <= BUF_LEN`, and slot `curr` is
        // either hardware-completed (`EOF_BIT == 1` / `has_data`) or SW-owned
        // (`OWNER_HW == 0`), so DMA will not write until `EOF_BIT` is cleared.
        unsafe {
            let buf_ptr = self
                .buffers
                .get()
                .cast::<DmaBuffer<BUF_LEN>>()
                .add(curr)
                .cast::<u8>();
            let mut b = 0;
            while b < scrub_len {
                ptr::write_volatile(buf_ptr.add(b), 0);
                b += 1;
            }
        }

        let cap = BUF_LEN as u32;
        let armed_word0 = WifiRxDmaDesc::OWNER_HW | (cap << 12) | cap;
        dma_release_barrier();
        let dptr = self.desc_ptr(curr);
        // JUSTIFICATION: Volatile publish of `OWNER_HW` with `EOF_BIT = 0`
        // re-arms slot `curr` for DMA strictly after scrubbing and Release
        // barrier.
        // SAFETY: `curr < N` and `dptr` is a valid, 4-byte aligned descriptor.
        unsafe {
            ptr::write_volatile(ptr::addr_of_mut!((*dptr).word0), armed_word0);
        }

        self.head = (self.head + 1) % N;
        Ok(curr)
    }
}

impl<const N: usize, const BUF_LEN: usize> Default for WifiRxRing<N, BUF_LEN> {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

/// Hardware TX DMA slot holding one descriptor and an aligned packet buffer.
///
/// DMA-shared storage is wrapped in `UnsafeCell` and marked `!Unpin`
/// (`PhantomPinned`) to prevent accidental relocation while a TX is in flight.
#[repr(C, align(4))]
pub struct WifiTxSlot<const BUF_LEN: usize> {
    desc: UnsafeCell<WifiTxDmaDesc>,
    buffer: UnsafeCell<DmaBuffer<BUF_LEN>>,
    in_flight: bool,
    _pin: PhantomPinned,
}

impl<const BUF_LEN: usize> WifiTxSlot<BUF_LEN> {
    /// Creates an idle TX DMA slot.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            desc: UnsafeCell::new(WifiTxDmaDesc::new()),
            buffer: UnsafeCell::new(DmaBuffer::new()),
            in_flight: false,
            _pin: PhantomPinned,
        }
    }

    #[inline]
    fn desc_is_owner_hw(&self) -> bool {
        let dptr = self.desc.get();
        // JUSTIFICATION: TX descriptor `ctrl` is cleared asynchronously by
        // hardware; volatile read prevents compiler hoisting.
        // SAFETY: `dptr` points to the valid 4-byte aligned `WifiTxDmaDesc`.
        let ctrl = unsafe { ptr::read_volatile(ptr::addr_of!((*dptr).ctrl)) };
        (ctrl & 0x8000_0000) != 0
    }

    /// Prepares a pinned TX slot for transmission.
    pub fn tx_prepare_pinned(self: Pin<&mut Self>, mpdu: &[u8]) -> Result<u32, DmaRingError> {
        // JUSTIFICATION: `tx_prepare` writes payload and arms descriptor
        // in-place; `self: Pin<&mut Self>` enforces address stability.
        // SAFETY: `tx_prepare` never relocates `self`.
        let this = unsafe { self.get_unchecked_mut() };
        // JUSTIFICATION: Pinning guarantees `this` stays at a stable physical
        // address while the armed TX descriptor is owned by DMA.
        // SAFETY: Caller pinned `self` via `Pin<&mut Self>`.
        unsafe { this.tx_prepare(mpdu) }
    }

    /// Prepares an MPDU frame for transmission.
    ///
    /// Copies `mpdu` into the aligned buffer, writes descriptor pointers,
    /// executes a Release/`memw` barrier, publishes `OWNER_HW | EOF` last,
    /// marks the slot in-flight, and returns the physical descriptor address
    /// to trigger in the MAC TX queue register.
    ///
    /// Prefer [`Self::tx_prepare_pinned`] when holding a `Pin<&mut Self>`.
    ///
    /// # Safety
    ///
    /// This method writes the physical address of `self.buffer` into `self.desc`
    /// and returns the physical address of `self.desc` for hardware TX DMA.
    /// The caller MUST guarantee that `self` remains pinned at a fixed memory
    /// address while the transmission is in flight.
    // JUSTIFICATION: Hardware TX DMA requires stable physical descriptor and
    // buffer addresses; `unsafe` enforces the pinned-address contract.
    pub unsafe fn tx_prepare(&mut self, mpdu: &[u8]) -> Result<u32, DmaRingError> {
        if BUF_LEN == 0 {
            return Err(DmaRingError::BufferTooShort);
        }
        if BUF_LEN > MAX_DMA_BUF_LEN {
            return Err(DmaRingError::BufferTooLarge);
        }
        if (BUF_LEN & 0x3) != 0 {
            return Err(DmaRingError::InvalidAlignment);
        }

        if self.in_flight {
            if self.desc_is_owner_hw() {
                return Err(DmaRingError::TxBusy);
            }
            dma_acquire_barrier();
            self.in_flight = false;
        } else if self.desc_is_owner_hw() {
            return Err(DmaRingError::TxBusy);
        }

        let len = mpdu.len();
        if len == 0 {
            return Err(DmaRingError::BufferTooShort);
        }
        let Some(frame_len) = len.checked_add(4) else {
            return Err(DmaRingError::BufferTooLarge);
        };
        if frame_len > BUF_LEN {
            return Err(DmaRingError::BufferTooLarge);
        }
        let frame_len_u16 = u16::try_from(frame_len).map_err(|_| DmaRingError::BufferTooLarge)?;

        let buf_addr = validate_dma_ptr(self.buffer.get() as usize)?;
        let desc_addr = validate_dma_ptr(self.desc.get() as usize)?;

        // 1. Copy payload into aligned DMA buffer FIRST (while SW owns slot),
        //    and zero the 4 trailing FCS bytes so hardware can fill FCS.
        // JUSTIFICATION: Exclusive SW ownership is verified above (`OWNER==0`);
        // raw copy/zero avoids creating a mutable reference over DMA storage.
        // SAFETY: `frame_len = len + 4 <= BUF_LEN`, non-overlapping buffers.
        unsafe {
            let dst = self.buffer.get().cast::<u8>();
            ptr::copy_nonoverlapping(mpdu.as_ptr(), dst, len);
            ptr::write_bytes(dst.add(len), 0, 4);
        }

        // 2. Write `buf_ptr` and `next_desc`, issue Release/`memw`, then
        // publish `ctrl` LAST. Both `size` and `length` match `frame_len`.
        // JUSTIFICATION: Descriptor is SW-owned; `prepare` enforces write
        // ordering and Release barrier before publishing `OWNER_HW`.
        // SAFETY: `self.desc` is uniquely owned and `OWNER_HW == 0`.
        let desc = unsafe { &mut *self.desc.get() };
        desc.prepare(buf_addr, frame_len_u16, frame_len_u16)?;
        self.in_flight = true;

        Ok(desc_addr)
    }

    /// Checks if the TX slot is ready to accept a new packet.
    pub fn is_ready(&mut self) -> bool {
        if self.in_flight && !self.desc_is_owner_hw() {
            dma_acquire_barrier();
            self.in_flight = false;
        }
        !self.in_flight
    }

    /// Returns `true` if a transmission is currently in flight.
    #[inline]
    #[must_use]
    pub const fn is_in_flight(&self) -> bool {
        self.in_flight
    }

    /// Marks the transmission completed after hardware TX has finished or been
    /// aborted and drained (`TX_ACTIVE == 0`).
    ///
    /// # Safety
    ///
    /// Caller MUST have verified that hardware TX DMA is no longer reading
    /// this slot's descriptor or payload buffer (`TX_DONE` observed or abort
    /// drain confirmed `TX_ACTIVE == 0`).
    // JUSTIFICATION: Reclaiming TX slot ownership requires caller proof that
    // hardware TX DMA has finished or drained cleanly (`M-014` / `M-074`).
    #[inline]
    pub(crate) unsafe fn mark_completed(&mut self) {
        let dptr = self.desc.get();
        // JUSTIFICATION: Reclaiming TX descriptor ownership after LMAC confirms
        // TX completion or drain requires a volatile clear + Acquire barrier.
        // SAFETY: `dptr` points to the valid, 4-byte aligned TX descriptor and
        // caller verified hardware TX DMA is quiescent.
        unsafe {
            let ctrl = ptr::read_volatile(ptr::addr_of!((*dptr).ctrl));
            ptr::write_volatile(ptr::addr_of_mut!((*dptr).ctrl), ctrl & !0x8000_0000);
        }
        dma_acquire_barrier();
        self.in_flight = false;
    }

    /// Returns a volatile by-value snapshot of the internal TX descriptor,
    /// avoiding forming a shared Rust reference over hardware-mutated DMA
    /// descriptor memory (`M-014`).
    #[inline]
    #[must_use]
    pub fn desc(&self) -> WifiTxDmaDesc {
        let dptr = self.desc.get();
        // JUSTIFICATION: Hardware TX DMA clears descriptor control bits
        // concurrently; field-wise volatile reads capture a snapshot without
        // forming a `&WifiTxDmaDesc` reference over live DMA memory.
        // SAFETY: `dptr` points to the valid 4-byte aligned TX descriptor.
        unsafe {
            WifiTxDmaDesc {
                ctrl: ptr::read_volatile(ptr::addr_of!((*dptr).ctrl)),
                buf_ptr: ptr::read_volatile(ptr::addr_of!((*dptr).buf_ptr)),
                next_desc: ptr::read_volatile(ptr::addr_of!((*dptr).next_desc)),
            }
        }
    }

    /// Returns a mutable reference to the internal TX descriptor if owned by
    /// software (not in flight / not HW-owned).
    #[inline]
    pub fn desc_mut(&mut self) -> Option<&mut WifiTxDmaDesc> {
        if self.in_flight || self.desc_is_owner_hw() {
            return None;
        }
        // JUSTIFICATION: Mutable access is gated on SW ownership.
        // SAFETY: `&mut self` and `OWNER_HW == 0` guarantee exclusive access.
        Some(unsafe { &mut *self.desc.get() })
    }

    /// Returns the physical 32-bit address of the TX descriptor.
    #[inline]
    #[must_use]
    pub fn desc_addr(&self) -> u32 {
        self.desc.get() as usize as u32
    }

    /// Returns an immutable slice of the packet buffer.
    #[inline]
    #[must_use]
    pub fn buffer_slice(&self) -> &[u8] {
        // JUSTIFICATION: Shared read of TX buffer contents.
        // SAFETY: `self.buffer` is valid and initialized.
        unsafe { (*self.buffer.get()).as_slice() }
    }

    /// Returns a mutable slice of the packet buffer if owned by software
    /// (not in flight / not HW-owned).
    #[inline]
    pub fn buffer_mut_slice(&mut self) -> Option<&mut [u8]> {
        if self.in_flight || self.desc_is_owner_hw() {
            return None;
        }
        // JUSTIFICATION: Mutable buffer access is gated on SW ownership.
        // SAFETY: `&mut self` and `OWNER_HW == 0` prevent DMA aliasing.
        Some(unsafe { (*self.buffer.get()).as_mut_slice() })
    }
}

impl<const BUF_LEN: usize> Default for WifiTxSlot<BUF_LEN> {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Simulates the ESP32 Wi-Fi MAC DMA engine writing a frame into slot `idx`
    /// and transferring ownership back to software (`OWNER_HW = 0`) with the
    /// specified `word0_flags` (`SOF_BIT | EOF_BIT` for single-buffer RX).
    fn simulate_hw_rx<const N: usize, const BUF_LEN: usize>(
        ring: &WifiRxRing<N, BUF_LEN>,
        idx: usize,
        word0_flags: u32,
        sig_len: u32,
        desc_len: u32,
        rssi_byte: u8,
        body: &[u8],
    ) {
        assert!(idx < N);
        // JUSTIFICATION: Test harness simulating hardware DMA write into a
        // hardware-owned slot through `UnsafeCell`.
        // SAFETY: `idx < N`; test is single-threaded and mimics hardware DMA.
        unsafe {
            let b = &mut (*ring.buffers.get().cast::<DmaBuffer<BUF_LEN>>().add(idx)).0;
            let mut words = [0u32; 7];
            words[0] = u32::from(rssi_byte);
            words[6] = sig_len;
            for (w_idx, &val) in words.iter().enumerate() {
                let off = w_idx * 4;
                b[off..off + 4].copy_from_slice(&val.to_le_bytes());
            }
            let copy_len = body.len().min(BUF_LEN.saturating_sub(RX_CTRL_LEN));
            b[RX_CTRL_LEN..RX_CTRL_LEN + copy_len].copy_from_slice(&body[..copy_len]);

            let dptr = ring.desc_ptr(idx);
            let word0 = (word0_flags & !WifiRxDmaDesc::OWNER_HW)
                | ((desc_len & 0xFFF) << 12)
                | ((BUF_LEN as u32) & 0xFFF);
            dma_release_barrier();
            ptr::write_volatile(ptr::addr_of_mut!((*dptr).word0), word0);
        }
    }

    #[test]
    fn test_struct_sizes_and_alignments() {
        assert_eq!(core::mem::size_of::<WifiRxDmaDesc>(), 12);
        assert_eq!(core::mem::align_of::<WifiRxDmaDesc>(), 4);

        assert_eq!(core::mem::size_of::<WifiTxDmaDesc>(), 12);
        assert_eq!(core::mem::align_of::<WifiTxDmaDesc>(), 4);

        assert_eq!(core::mem::size_of::<DmaBuffer<1600>>(), 1600);
        assert_eq!(core::mem::align_of::<DmaBuffer<1600>>(), 4);
    }

    #[test]
    fn test_rx_ctrl_parse() {
        let mut words = [0u32; 7];
        words[0] = 0x0000_8C00 | (0xC4 & 0xFF);
        words[1] = 0x8800_0087;
        words[2] = 0x0016_04A1;
        words[3] = 1_234_567;
        words[4] = 0;
        words[5] = 0x8000_0000;
        words[6] = 0x0100_01F4;

        let ctrl = WifiRxCtrl::parse(&words);
        assert_eq!(ctrl.rssi, -60);
        assert_eq!(ctrl.rate, 12);
        assert_eq!(ctrl.sig_mode, 2);
        assert_eq!(ctrl.mcs, 7);
        assert!(ctrl.cwb);
        assert!(ctrl.aggregation);
        assert!(ctrl.sgi);
        assert_eq!(ctrl.noise_floor, -95);
        assert_eq!(ctrl.ampdu_cnt, 4);
        assert_eq!(ctrl.channel, 6);
        assert_eq!(ctrl.secondary_channel, 1);
        assert_eq!(ctrl.timestamp_us, 1_234_567);
        assert_eq!(ctrl.ant, 1);
        assert_eq!(ctrl.sig_len, 500);
        assert_eq!(ctrl.rx_state, 0x01);
    }

    #[test]
    fn test_rx_ring_lifecycle_and_recycling_zeroes_buffer() {
        let mut ring = WifiRxRing::<4, 256>::new();
        assert!(!ring.is_armed());

        // JUSTIFICATION: Local test ring is never moved after arming.
        // SAFETY: `ring` stays at a fixed stack address for the test lifetime.
        assert_eq!(unsafe { ring.arm_ring() }, Ok(()));
        assert!(ring.is_armed());

        // Armed HW-owned slots must deny mutable CPU accessors (M-014).
        assert!(ring.desc_mut(0).is_none());
        assert!(ring.raw_buffer_mut(0).is_none());
        assert!(ring.current_desc_mut().is_none());
        // JUSTIFICATION: Re-arm rejection test on fixed local ring.
        // SAFETY: `ring` stays pinned in place.
        assert_eq!(unsafe { ring.arm_ring() }, Err(DmaRingError::RingBusy));

        for i in 0..4 {
            let desc = ring.desc(i);
            assert!(desc.is_some());
            if let Some(d) = desc {
                assert!(d.is_owner_hw());
                assert_eq!(d.buffer_cap(), 256);
                assert_ne!(d.buf_ptr, 0);
                assert_ne!(d.next_desc, 0);
            }
        }

        assert_eq!(ring.peek_received(), RxPop::Empty);
        assert_eq!(ring.recycle_current(), Err(DmaRingError::RingEmpty));

        simulate_hw_rx(
            &ring,
            0,
            WifiRxDmaDesc::SOF_BIT | WifiRxDmaDesc::EOF_BIT,
            4,
            32,
            0xC4,
            &[0x08, 0x00, 0x12, 0x34],
        );

        let rx = ring.pop_received();
        assert!(matches!(rx, RxPop::Frame(_, _)));
        if let RxPop::Frame(ctrl, payload) = rx {
            assert_eq!(ctrl.rssi, -60);
            assert_eq!(payload, &[0x08, 0x00, 0x12, 0x34]);
        }

        assert_eq!(ring.recycle_current(), Ok(0));
        assert_eq!(ring.head_index(), 1);

        if let Some(d) = ring.desc(0) {
            assert!(d.is_owner_hw());
        }

        // Verify slot 0 was zeroed on recycle via raw pointer inspection.
        // JUSTIFICATION: Test verifies recycle scrubbed prior payload bytes.
        // SAFETY: Single-threaded test inspection of slot 0 buffer.
        unsafe {
            let b = &(*ring.buffers.get().cast::<DmaBuffer<256>>()).0;
            assert_eq!(&b[..32], &[0u8; 32]);
        }
    }

    #[test]
    fn test_rx_ring_rejects_invalid_n_and_buf_len_over_0xffc() {
        let mut zero_n = WifiRxRing::<0, 256>::new();
        // JUSTIFICATION: Testing error path of arm_ring on local stack ring.
        // SAFETY: `zero_n` is never moved while armed (and arm fails).
        let zero_res = unsafe { zero_n.arm_ring() };
        assert_eq!(zero_res, Err(DmaRingError::InvalidRingSize));
        // M-163: must return None/Empty without panicking when N == 0.
        assert!(zero_n.current_desc().is_none());
        assert!(zero_n.current_desc_mut().is_none());
        assert_eq!(zero_n.peek_received(), RxPop::Empty);

        let mut too_small = WifiRxRing::<2, 24>::new();
        // JUSTIFICATION: Testing error path of arm_ring on local stack ring.
        // SAFETY: `too_small` is never moved.
        let small_res = unsafe { too_small.arm_ring() };
        assert_eq!(small_res, Err(DmaRingError::BufferTooShort));

        let mut unaligned = WifiRxRing::<2, 30>::new();
        // JUSTIFICATION: Testing error path of arm_ring on local stack ring.
        // SAFETY: `unaligned` is never moved.
        let unaligned_res = unsafe { unaligned.arm_ring() };
        assert_eq!(unaligned_res, Err(DmaRingError::InvalidAlignment));

        // M-070: Reject BUF_LEN > 0xFFC instead of 12-bit truncation.
        let mut trunc_4096 = WifiRxRing::<2, 4096>::new();
        // JUSTIFICATION: Testing error path of arm_ring on local stack ring.
        // SAFETY: `trunc_4096` is never moved.
        let trunc_res = unsafe { trunc_4096.arm_ring() };
        assert_eq!(trunc_res, Err(DmaRingError::BufferTooLarge));

        let mut unaligned_max = WifiRxRing::<2, 0xFFE>::new();
        // JUSTIFICATION: Testing error path of arm_ring on local stack ring.
        // SAFETY: `unaligned_max` is never moved.
        let max_res = unsafe { unaligned_max.arm_ring() };
        assert_eq!(max_res, Err(DmaRingError::BufferTooLarge));

        let mut tx_4096 = WifiTxSlot::<4096>::new();
        // JUSTIFICATION: Testing error path of tx_prepare on local stack slot.
        // SAFETY: `tx_4096` is never moved.
        let tx_res = unsafe { tx_4096.tx_prepare(&[1, 2, 3, 4]) };
        assert_eq!(tx_res, Err(DmaRingError::BufferTooLarge));
    }

    #[test]
    fn test_tx_slot_prepare_and_completion() {
        let mut slot = WifiTxSlot::<256>::new();
        assert!(slot.is_ready());
        assert!(!slot.is_in_flight());

        // M-070: empty payload rejected.
        // JUSTIFICATION: Testing tx_prepare validation on fixed local slot.
        // SAFETY: `slot` is never moved during this test.
        let empty_res = unsafe { slot.tx_prepare(&[]) };
        assert_eq!(empty_res, Err(DmaRingError::BufferTooShort));

        // Pre-seed stale bytes in the FCS tail to verify zeroing.
        if let Some(buf) = slot.buffer_mut_slice() {
            buf[5..9].copy_from_slice(&[0xFF; 4]);
        }

        let payload = [0xAA, 0xBB, 0xCC, 0xDD, 0xEE];
        // JUSTIFICATION: Preparing local stack slot for unit test inspection.
        // SAFETY: `slot` stays pinned in place and no real DMA runs.
        let prep_res = unsafe { slot.tx_prepare(&payload) };
        assert!(prep_res.is_ok());
        if let Ok(desc_addr) = prep_res {
            assert_eq!(desc_addr, slot.desc_addr());
        }

        assert!(slot.is_in_flight());
        assert!(!slot.is_ready());
        // M-014: mutable accessors denied while in flight.
        assert!(slot.desc_mut().is_none());
        assert!(slot.buffer_mut_slice().is_none());

        let d = slot.desc();
        assert!(d.is_owner_hw());
        assert_eq!(d.payload_len(), 9);
        assert_eq!(d.buffer_cap(), 9);
        assert_eq!(d.ctrl, 0xC000_0000 | (9 << 12) | 9);
        assert_eq!(&slot.buffer_slice()[..5], &payload);
        assert_eq!(&slot.buffer_slice()[5..9], &[0u8; 4]);

        // JUSTIFICATION: Verifying TxBusy guard on in-flight slot.
        // SAFETY: `slot` stays pinned in place.
        let busy_res = unsafe { slot.tx_prepare(&payload) };
        assert_eq!(busy_res, Err(DmaRingError::TxBusy));

        // JUSTIFICATION: Simulated TX completion in host unit test.
        // SAFETY: No hardware DMA is active in unit tests.
        unsafe {
            slot.mark_completed();
        }
        assert!(!slot.is_in_flight());
        assert!(slot.is_ready());
        assert!(slot.desc_mut().is_some());
        assert!(slot.buffer_mut_slice().is_some());

        // 253 + 4 = 257 > 256 must fail with BufferTooLarge.
        let boundary_oversized = [0u8; 253];
        // JUSTIFICATION: Testing FCS boundary overflow on fixed slot.
        // SAFETY: `slot` stays pinned in place.
        let boundary_res = unsafe { slot.tx_prepare(&boundary_oversized) };
        assert_eq!(boundary_res, Err(DmaRingError::BufferTooLarge));

        let oversized = [0u8; 300];
        // JUSTIFICATION: Testing oversized payload rejection on fixed slot.
        // SAFETY: `slot` stays pinned in place.
        let over_res = unsafe { slot.tx_prepare(&oversized) };
        assert_eq!(over_res, Err(DmaRingError::BufferTooLarge));
    }

    #[test]
    fn test_rx_desc_arm_validates_capacity() {
        let mut d = WifiRxDmaDesc::new();
        assert_eq!(d.arm(1600), Ok(()));
        assert_eq!(d.word0, 0x8000_0000 | (1600 << 12) | 1600);
        assert_eq!(d.buffer_cap(), 1600);
        assert!(d.is_owner_hw());
        assert!(!d.is_eof());

        assert_eq!(d.arm(4096), Err(DmaRingError::BufferTooLarge));
        assert_eq!(d.arm(16), Err(DmaRingError::BufferTooShort));
        assert_eq!(d.arm(30), Err(DmaRingError::InvalidAlignment));
    }

    #[test]
    fn test_rx_desc_reads_length_from_bits_23_12_and_size_from_bits_11_0() {
        let mut d = WifiRxDmaDesc::new();
        d.word0 = WifiRxDmaDesc::EOF_BIT | (0x123 << 12) | 0x640;
        assert_eq!(d.payload_len(), 0x123);
        assert_eq!(d.buffer_cap(), 0x640);
    }

    #[test]
    fn test_tx_desc_prepare_uses_lldesc_bit_layout_and_validates() {
        let mut d = WifiTxDmaDesc::new();
        assert_eq!(d.prepare(0x3FFB_0000, 45, 45), Ok(()));
        assert_eq!(d.ctrl, 0xC000_0000 | (45 << 12) | 45);
        assert_eq!(d.ctrl & (1 << 29), 0);
        assert_eq!(d.payload_len(), 45);
        assert_eq!(d.buffer_cap(), 45);
        assert_eq!(d.buf_ptr, 0x3FFB_0000);
        assert_eq!(d.next_desc, 0);

        assert_eq!(
            d.prepare(0x3FFB_0000, 1600, 0),
            Err(DmaRingError::BufferTooShort)
        );
        assert_eq!(
            d.prepare(0x3FFB_0000, 1600, 1601),
            Err(DmaRingError::BufferTooLarge)
        );
        assert_eq!(
            d.prepare(0x3FFB_0000, 4096, 42),
            Err(DmaRingError::BufferTooLarge)
        );
        assert_eq!(
            d.prepare(0x3FFB_0002, 1600, 42),
            Err(DmaRingError::InvalidAlignment)
        );
    }

    #[test]
    fn test_peek_received_prefers_sig_len_over_padded_descriptor_length() {
        let mut ring = WifiRxRing::<2, 256>::new();
        // JUSTIFICATION: Stack-pinned local test ring never moved.
        // SAFETY: `ring` stays at a fixed address for the test lifetime.
        assert_eq!(unsafe { ring.arm_ring() }, Ok(()));

        simulate_hw_rx(
            &ring,
            0,
            WifiRxDmaDesc::SOF_BIT | WifiRxDmaDesc::EOF_BIT,
            6,
            36,
            0xC4,
            &[1, 2, 3, 4, 5, 6, 0xEE, 0xEE],
        );

        let rx = ring.peek_received();
        assert!(matches!(rx, RxPop::Frame(_, _)));
        if let RxPop::Frame(ctrl, payload) = rx {
            assert_eq!(ctrl.sig_len, 6);
            assert_eq!(payload, &[1, 2, 3, 4, 5, 6]);
        }
    }

    #[test]
    fn test_peek_received_falls_back_to_descriptor_length_when_sig_len_is_zero() {
        let mut ring = WifiRxRing::<2, 256>::new();
        // JUSTIFICATION: Stack-pinned local test ring never moved.
        // SAFETY: `ring` stays at a fixed address for the test lifetime.
        assert_eq!(unsafe { ring.arm_ring() }, Ok(()));

        simulate_hw_rx(
            &ring,
            0,
            WifiRxDmaDesc::SOF_BIT | WifiRxDmaDesc::EOF_BIT,
            0,
            28 + 8,
            0xC4,
            &[1, 2, 3, 4, 5, 6, 7, 8],
        );

        let rx = ring.peek_received();
        assert!(matches!(rx, RxPop::Frame(_, _)));
        if let RxPop::Frame(_, payload) = rx {
            assert_eq!(payload, &[1, 2, 3, 4, 5, 6, 7, 8]);
        }
    }

    #[test]
    fn test_peek_received_caps_sig_len_to_dma_payload_len_not_full_desc_len() {
        let mut ring = WifiRxRing::<2, 256>::new();
        // JUSTIFICATION: Stack-pinned local test ring never moved.
        // SAFETY: `ring` stays at a fixed address for the test lifetime.
        assert_eq!(unsafe { ring.arm_ring() }, Ok(()));

        // M-071: DMA wrote 36 bytes (28 rx_ctrl + 8 payload).
        // sig_len == 36 exceeds dma_payload_len (8) so it must fall back to 8.
        simulate_hw_rx(
            &ring,
            0,
            WifiRxDmaDesc::SOF_BIT | WifiRxDmaDesc::EOF_BIT,
            36,
            36,
            0xC4,
            &[9; 8],
        );

        let rx = ring.peek_received();
        assert!(matches!(rx, RxPop::Frame(_, _)));
        if let RxPop::Frame(_, payload) = rx {
            assert_eq!(payload, &[9; 8]);
        }
    }

    #[test]
    fn test_peek_received_rejects_short_desc_len_and_missing_eof() {
        let mut ring = WifiRxRing::<2, 256>::new();
        // JUSTIFICATION: Stack-pinned local test ring never moved.
        // SAFETY: `ring` stays at a fixed address for the test lifetime.
        assert_eq!(unsafe { ring.arm_ring() }, Ok(()));

        // Armed HW-owned ring reports Empty (distinct from Malformed).
        assert_eq!(ring.peek_received(), RxPop::Empty);

        // M-071/M-089: desc_len < RX_CTRL_LEN must return Malformed, not Empty.
        simulate_hw_rx(
            &ring,
            0,
            WifiRxDmaDesc::SOF_BIT | WifiRxDmaDesc::EOF_BIT,
            0xFFF,
            0,
            0xC4,
            &[9; 8],
        );
        assert_eq!(ring.peek_received(), RxPop::Malformed);
        assert_eq!(ring.recycle_current(), Ok(0));

        // Real ESP32 Wi-Fi silicon leaves SOF_BIT == 0 and sets EOF_BIT == 1
        // (`has_data`) upon frame completion; this must be accepted as Frame.
        simulate_hw_rx(&ring, 1, WifiRxDmaDesc::EOF_BIT, 8, 36, 0xC4, &[7; 8]);
        assert!(matches!(
            ring.peek_received(),
            RxPop::Frame(_, payload) if payload == [7; 8]
        ));
        assert_eq!(ring.recycle_current(), Ok(1));

        // Missing EOF with OWNER_HW == 0 (malformed mock) must return
        // Malformed.
        simulate_hw_rx(&ring, 0, WifiRxDmaDesc::SOF_BIT, 8, 36, 0xC4, &[5; 8]);
        assert_eq!(ring.peek_received(), RxPop::Malformed);
        assert_eq!(ring.recycle_current(), Ok(0));
        assert_eq!(ring.peek_received(), RxPop::Empty);
    }

    #[test]
    fn test_peek_received_accepts_real_silicon_word0_and_recycles_to_armed() {
        let mut ring = WifiRxRing::<2, 1600>::new();
        // JUSTIFICATION: Stack-pinned local test ring never moved.
        // SAFETY: `ring` stays at a fixed address for the test lifetime.
        assert_eq!(unsafe { ring.arm_ring() }, Ok(()));

        let expected_payload = [0xA5u8; 56];
        // JUSTIFICATION: Test harness simulates real ESP32 Wi-Fi RX silicon
        // DMA write where OWNER_HW (bit 31) stays 1, EOF_BIT (bit 30) is set
        // to 1 (`has_data`), and SOF_BIT (bit 29) stays 0.
        // SAFETY: `ring` is pinned on stack and single-threaded test owns it.
        unsafe {
            let b = &mut (*ring.buffers.get().cast::<DmaBuffer<1600>>()).0;
            let mut words = [0u32; 7];
            words[0] = 0xC4;
            words[6] = 56;
            for (w_idx, &val) in words.iter().enumerate() {
                let off = w_idx * 4;
                b[off..off + 4].copy_from_slice(&val.to_le_bytes());
            }
            b[RX_CTRL_LEN..RX_CTRL_LEN + 56].copy_from_slice(&expected_payload);

            let dptr = ring.desc_ptr(0);
            let silicon_word0 = 0xC000_0000 | (84 << 12) | (1600 & 0xFFF);
            assert_eq!(
                silicon_word0 & WifiRxDmaDesc::OWNER_HW,
                WifiRxDmaDesc::OWNER_HW
            );
            assert_eq!(
                silicon_word0 & WifiRxDmaDesc::EOF_BIT,
                WifiRxDmaDesc::EOF_BIT
            );
            assert_eq!(silicon_word0 & WifiRxDmaDesc::SOF_BIT, 0);
            dma_release_barrier();
            ptr::write_volatile(ptr::addr_of_mut!((*dptr).word0), silicon_word0);
        }

        let rx = ring.peek_received();
        assert!(matches!(rx, RxPop::Frame(_, _)));
        if let RxPop::Frame(ctrl, payload) = rx {
            assert_eq!(ctrl.rssi, -60);
            assert_eq!(ctrl.sig_len, 56);
            assert_eq!(payload, &expected_payload);
        }

        assert!(ring.raw_buffer(0).is_some());
        assert_eq!(ring.recycle_current(), Ok(0));
        assert_eq!(ring.head_index(), 1);

        let desc_opt = ring.desc(0);
        assert!(desc_opt.is_some());
        if let Some(recycled) = desc_opt {
            assert_eq!(recycled.word0, 0x8064_0640);
            assert_eq!(recycled.word0 & WifiRxDmaDesc::EOF_BIT, 0);
            assert!(recycled.is_owner_hw());
        }
        assert_eq!(ring.peek_received(), RxPop::Empty);
    }

    #[test]
    fn test_pinned_arm_and_tx_prepare_work_under_miri() {
        let mut ring = WifiRxRing::<2, 256>::new();
        // JUSTIFICATION: Stack-pinned local test value never moved after Pin.
        // SAFETY: `ring` is shadowed and never moved before drop.
        let pinned_ring = unsafe { Pin::new_unchecked(&mut ring) };
        assert_eq!(pinned_ring.arm_ring_pinned(), Ok(()));

        let mut slot = WifiTxSlot::<256>::new();
        // JUSTIFICATION: Stack-pinned local test value never moved after Pin.
        // SAFETY: `slot` is shadowed and never moved before drop.
        let pinned_slot = unsafe { Pin::new_unchecked(&mut slot) };
        assert!(pinned_slot.tx_prepare_pinned(&[1, 2, 3, 4]).is_ok());
    }
}
