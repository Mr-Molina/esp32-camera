//! Wi-Fi MAC DMA descriptor ring and TX slot buffer management.
//!
//! Implements hardware DMA ring buffers for 802.11 frame reception and
//! transmission on the ESP32 Wi-Fi MAC. All descriptors and buffers are
//! 4-byte aligned as required by the Xtensa DMA controller and Miri invariants.

use core::fmt;

/// Length in bytes of the hardware `rx_ctrl` descriptor prepended to
/// each received 802.11 MPDU buffer by the MAC DMA engine.
pub const RX_CTRL_LEN: usize = 28;

/// Errors returned by DMA ring and slot operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DmaRingError {
    /// Provided buffer or slice is too short.
    BufferTooShort,
    /// Provided MPDU payload exceeds buffer capacity.
    BufferTooLarge,
    /// The RX ring has no completed frames available for consumption.
    RingEmpty,
    /// The TX slot is currently in-flight and cannot accept a new transmission.
    TxBusy,
    /// The ring capacity `N` is zero or invalid.
    InvalidRingSize,
    /// The ring has not been armed with hardware links.
    NotArmed,
}

impl fmt::Display for DmaRingError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::BufferTooShort => f.write_str("buffer too short"),
            Self::BufferTooLarge => f.write_str("payload exceeds buffer capacity"),
            Self::RingEmpty => f.write_str("no completed RX descriptors available"),
            Self::TxBusy => f.write_str("TX slot is busy"),
            Self::InvalidRingSize => f.write_str("invalid ring descriptor count"),
            Self::NotArmed => f.write_str("DMA ring is not armed"),
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
///   - Bit 31: Owner (1 = Hardware/DMA, 0 = Software/CPU).
///   - Bit 30: EOF (End of Frame).
///   - Bit 29: SOF (Start of Frame).
///   - Bits 23..12: Buffer capacity in bytes.
///   - Bits 11..0: Received frame payload length in bytes.
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

    /// Returns `true` if the descriptor is currently owned by hardware/DMA.
    #[inline]
    #[must_use]
    pub const fn is_owner_hw(&self) -> bool {
        (self.word0 & Self::OWNER_HW) != 0
    }

    /// Returns `true` if the descriptor is currently owned by software/CPU.
    #[inline]
    #[must_use]
    pub const fn is_owner_sw(&self) -> bool {
        !self.is_owner_hw()
    }

    /// Returns `true` if this descriptor contains the end of a received frame.
    #[inline]
    #[must_use]
    pub const fn is_eof(&self) -> bool {
        (self.word0 & Self::EOF_BIT) != 0
    }

    /// Returns `true` if this descriptor contains the start of a received frame.
    #[inline]
    #[must_use]
    pub const fn is_sof(&self) -> bool {
        (self.word0 & Self::SOF_BIT) != 0
    }

    /// Returns the received payload length from bits [11:0].
    #[inline]
    #[must_use]
    pub const fn payload_len(&self) -> u16 {
        (self.word0 & 0xFFF) as u16
    }

    /// Returns the buffer capacity from bits [23:12].
    #[inline]
    #[must_use]
    pub const fn buffer_cap(&self) -> u16 {
        ((self.word0 >> 12) & 0xFFF) as u16
    }

    /// Sets ownership to hardware/DMA.
    #[inline]
    pub fn set_owner_hw(&mut self) {
        self.word0 |= Self::OWNER_HW;
    }

    /// Sets ownership to software/CPU.
    #[inline]
    pub fn set_owner_sw(&mut self) {
        self.word0 &= !Self::OWNER_HW;
    }

    /// Prepares and arms this descriptor for hardware DMA reception with
    /// the given capacity.
    #[inline]
    pub fn arm(&mut self, capacity: u16) {
        let cap_field = ((capacity as u32) & 0xFFF) << 12;
        self.word0 = Self::OWNER_HW | cap_field;
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
/// - `ctrl`: Control word (`0xE000_0000 | (cap << 12) | len`).
///   - Bit 31: Owner (1 = Hardware/DMA, 0 = Software/CPU).
///   - Bit 30: EOF (End of Frame).
///   - Bit 29: SOF (Start of Frame).
///   - Bits 23..12: Buffer capacity in bytes.
///   - Bits 11..0: Packet payload length in bytes.
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
    /// Combined control bits: Owner (bit 31), EOF (bit 30), SOF (bit 29).
    pub const CTRL_FLAGS: u32 = 0xE000_0000;

    /// Creates an empty TX descriptor owned by software.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            ctrl: 0,
            buf_ptr: 0,
            next_desc: 0,
        }
    }

    /// Returns `true` if the descriptor is currently owned by hardware/DMA.
    #[inline]
    #[must_use]
    pub const fn is_owner_hw(&self) -> bool {
        (self.ctrl & 0x8000_0000) != 0
    }

    /// Returns `true` if the descriptor is currently owned by software/CPU.
    #[inline]
    #[must_use]
    pub const fn is_owner_sw(&self) -> bool {
        !self.is_owner_hw()
    }

    /// Returns the configured payload length from bits [11:0].
    #[inline]
    #[must_use]
    pub const fn payload_len(&self) -> u16 {
        (self.ctrl & 0xFFF) as u16
    }

    /// Returns the buffer capacity from bits [23:12].
    #[inline]
    #[must_use]
    pub const fn buffer_cap(&self) -> u16 {
        ((self.ctrl >> 12) & 0xFFF) as u16
    }

    /// Prepares the TX descriptor for single-frame hardware transmission.
    #[inline]
    pub fn prepare(&mut self, buf_addr: u32, cap: u16, len: u16) {
        let cap_bits = ((cap as u32) & 0xFFF) << 12;
        let len_bits = (len as u32) & 0xFFF;
        self.ctrl = Self::CTRL_FLAGS | cap_bits | len_bits;
        self.buf_ptr = buf_addr;
        self.next_desc = 0;
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

/// Circular hardware RX DMA ring holding `N` descriptors and `N` aligned
/// buffers.
///
/// Ensures memory alignment (`align(4)`) for hardware DMA and Miri checks.
#[repr(C, align(4))]
pub struct WifiRxRing<const N: usize, const BUF_LEN: usize> {
    descs: [WifiRxDmaDesc; N],
    buffers: [DmaBuffer<BUF_LEN>; N],
    head: usize,
    armed: bool,
}

impl<const N: usize, const BUF_LEN: usize> WifiRxRing<N, BUF_LEN> {
    /// Creates an unlinked, zeroed RX DMA ring.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            descs: [WifiRxDmaDesc::new(); N],
            buffers: [DmaBuffer::new(); N],
            head: 0,
            armed: false,
        }
    }

    /// Arms the circular descriptor ring in-place, linking descriptor addresses
    /// and assigning buffer pointers.
    ///
    /// # Safety Invariant
    /// The `WifiRxRing` must not be moved in memory after this function is called,
    /// as the internal descriptor pointers reference its current location.
    pub fn arm_ring(&mut self) -> Result<(), DmaRingError> {
        if N == 0 {
            return Err(DmaRingError::InvalidRingSize);
        }

        let descs_base = self.descs.as_ptr() as usize;
        let desc_stride = core::mem::size_of::<WifiRxDmaDesc>();

        let mut i = 0;
        while i < N {
            let next_idx = (i + 1) % N;
            let next_desc_addr = (descs_base + next_idx * desc_stride) as u32;
            let buf_addr = self.buffers[i].0.as_ptr() as usize as u32;

            self.descs[i].buf_ptr = buf_addr;
            self.descs[i].next_desc = next_desc_addr;
            self.descs[i].arm(BUF_LEN as u16);
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
        self.descs.as_ptr() as usize as u32
    }

    /// Returns a reference to the descriptor at the current consumer head.
    #[inline]
    #[must_use]
    pub fn current_desc(&self) -> &WifiRxDmaDesc {
        &self.descs[self.head]
    }

    /// Returns a mutable reference to the descriptor at the current consumer head.
    #[inline]
    pub fn current_desc_mut(&mut self) -> &mut WifiRxDmaDesc {
        &mut self.descs[self.head]
    }

    /// Returns a reference to a specific descriptor if index is within bounds.
    #[must_use]
    pub fn desc(&self, idx: usize) -> Option<&WifiRxDmaDesc> {
        if idx < N {
            Some(&self.descs[idx])
        } else {
            None
        }
    }

    /// Returns a mutable reference to a specific descriptor if index is
    /// within bounds.
    pub fn desc_mut(&mut self, idx: usize) -> Option<&mut WifiRxDmaDesc> {
        if idx < N {
            Some(&mut self.descs[idx])
        } else {
            None
        }
    }

    /// Returns an immutable reference to the raw buffer at `idx` if within bounds.
    #[must_use]
    pub fn raw_buffer(&self, idx: usize) -> Option<&[u8]> {
        if idx < N {
            Some(&self.buffers[idx].0)
        } else {
            None
        }
    }

    /// Returns a mutable reference to the raw buffer at `idx` if within bounds.
    pub fn raw_buffer_mut(&mut self, idx: usize) -> Option<&mut [u8]> {
        if idx < N {
            Some(&mut self.buffers[idx].0)
        } else {
            None
        }
    }

    /// Inspects the frame at the current head without consuming or recycling it.
    ///
    /// Returns `None` if the ring is not armed or the current descriptor is
    /// still owned by hardware DMA.
    #[must_use]
    pub fn peek_received(&self) -> Option<(WifiRxCtrl, &[u8])> {
        if !self.armed {
            return None;
        }

        let desc = &self.descs[self.head];
        if desc.is_owner_hw() {
            return None;
        }

        let raw_buf = &self.buffers[self.head].0;
        let ctrl = WifiRxCtrl::parse_slice(raw_buf).ok()?;

        let desc_len = desc.payload_len() as usize;
        let payload_len = if desc_len >= RX_CTRL_LEN {
            desc_len - RX_CTRL_LEN
        } else if ctrl.sig_len > 0 {
            (ctrl.sig_len as usize).min(BUF_LEN.saturating_sub(RX_CTRL_LEN))
        } else {
            0
        };

        let payload_end = (RX_CTRL_LEN + payload_len).min(BUF_LEN);
        let payload = &raw_buf[RX_CTRL_LEN..payload_end];
        Some((ctrl, payload))
    }

    /// Reads the frame at the current head without recycling it.
    ///
    /// The caller must subsequently call [`Self::recycle_current`] to return the
    /// descriptor to hardware ownership and advance the ring.
    pub fn pop_received(&mut self) -> Option<(WifiRxCtrl, &[u8])> {
        self.peek_received()
    }

    /// Returns ownership of the current head descriptor back to hardware DMA
    /// and advances the head pointer to the next slot in the ring.
    pub fn recycle_current(&mut self) -> Result<usize, DmaRingError> {
        if !self.armed {
            return Err(DmaRingError::NotArmed);
        }

        let curr = self.head;
        if self.descs[curr].is_owner_hw() {
            return Err(DmaRingError::RingEmpty);
        }

        self.descs[curr].arm(BUF_LEN as u16);
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
#[repr(C, align(4))]
pub struct WifiTxSlot<const BUF_LEN: usize> {
    desc: WifiTxDmaDesc,
    buffer: DmaBuffer<BUF_LEN>,
    in_flight: bool,
}

impl<const BUF_LEN: usize> WifiTxSlot<BUF_LEN> {
    /// Creates an idle TX DMA slot.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            desc: WifiTxDmaDesc::new(),
            buffer: DmaBuffer::new(),
            in_flight: false,
        }
    }

    /// Prepares an MPDU frame for transmission.
    ///
    /// Copies `mpdu` into the aligned buffer, sets up the TX descriptor,
    /// marks the slot in-flight, and returns the physical descriptor address
    /// to trigger in the MAC TX queue register.
    pub fn tx_prepare(&mut self, mpdu: &[u8]) -> Result<u32, DmaRingError> {
        if self.in_flight {
            if self.desc.is_owner_hw() {
                return Err(DmaRingError::TxBusy);
            }
            self.in_flight = false;
        }

        let len = mpdu.len();
        if len > BUF_LEN {
            return Err(DmaRingError::BufferTooLarge);
        }

        self.buffer.0[..len].copy_from_slice(mpdu);
        let buf_addr = self.buffer.0.as_ptr() as usize as u32;
        self.desc.prepare(buf_addr, BUF_LEN as u16, len as u16);
        self.in_flight = true;

        Ok(&self.desc as *const WifiTxDmaDesc as usize as u32)
    }

    /// Checks if the TX slot is ready to accept a new packet.
    pub fn is_ready(&mut self) -> bool {
        if self.in_flight && self.desc.is_owner_sw() {
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

    /// Marks the transmission completed by hardware.
    #[inline]
    pub fn mark_completed(&mut self) {
        self.in_flight = false;
        self.desc.ctrl &= !0x8000_0000;
    }

    /// Returns a reference to the internal TX descriptor.
    #[inline]
    #[must_use]
    pub const fn desc(&self) -> &WifiTxDmaDesc {
        &self.desc
    }

    /// Returns a mutable reference to the internal TX descriptor.
    #[inline]
    pub fn desc_mut(&mut self) -> &mut WifiTxDmaDesc {
        &mut self.desc
    }

    /// Returns the physical 32-bit address of the TX descriptor.
    #[inline]
    #[must_use]
    pub fn desc_addr(&self) -> u32 {
        &self.desc as *const WifiTxDmaDesc as usize as u32
    }

    /// Returns an immutable slice of the packet buffer.
    #[inline]
    #[must_use]
    pub fn buffer_slice(&self) -> &[u8] {
        self.buffer.as_slice()
    }

    /// Returns a mutable slice of the packet buffer.
    #[inline]
    pub fn buffer_mut_slice(&mut self) -> &mut [u8] {
        self.buffer.as_mut_slice()
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
        // w0: rssi = -60 (0xC4), rate = 12 (0x0C), sig_mode = 2 (11n)
        words[0] = 0x0000_8C00 | (0xC4 & 0xFF);
        // w1: mcs = 7 (0x07), cwb = 1 (bit 7), agg = 1 (bit 27), sgi = 1 (bit 31)
        words[1] = 0x8800_0087;
        // w2: noise_floor = -95 (0xA1), ampdu_cnt = 4, channel = 6, sec_chan = 1
        words[2] = 0x0016_04A1;
        // w3: timestamp_us = 1_234_567
        words[3] = 1_234_567;
        // w4: reserved
        words[4] = 0;
        // w5: ant = 1 (bit 31)
        words[5] = 0x8000_0000;
        // w6: sig_len = 500 (0x1F4), rx_state = 0x01
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
    fn test_rx_ring_lifecycle_and_recycling() {
        let mut ring = WifiRxRing::<4, 256>::new();
        assert!(!ring.is_armed());

        let arm_res = ring.arm_ring();
        assert!(arm_res.is_ok());
        assert!(ring.is_armed());

        // Check circular descriptor links
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

        // Initially no received frames
        assert!(ring.peek_received().is_none());
        assert_eq!(ring.recycle_current(), Err(DmaRingError::RingEmpty));

        // Simulate HW reception in slot 0
        let buf = ring.raw_buffer_mut(0);
        assert!(buf.is_some());
        if let Some(b) = buf {
            // Write rx_ctrl header (28 bytes)
            let mut words = [0u32; 7];
            words[0] = 0xC4; // rssi = -60
            words[6] = 4; // sig_len = 4 bytes payload
            for (w_idx, &val) in words.iter().enumerate() {
                let off = w_idx * 4;
                b[off..off + 4].copy_from_slice(&val.to_le_bytes());
            }
            // 4 bytes MPDU payload at offset 28
            b[28..32].copy_from_slice(&[0x08, 0x00, 0x12, 0x34]);
        }

        let desc0 = ring.desc_mut(0);
        assert!(desc0.is_some());
        if let Some(d) = desc0 {
            // HW sets SW ownership, EOF, and length (28 + 4 = 32 bytes)
            d.word0 = WifiRxDmaDesc::EOF_BIT | ((256 & 0xFFF) << 12) | 32;
        }

        // Now pop_received should see frame
        let rx = ring.pop_received();
        assert!(rx.is_some());
        if let Some((ctrl, payload)) = rx {
            assert_eq!(ctrl.rssi, -60);
            assert_eq!(payload, &[0x08, 0x00, 0x12, 0x34]);
        }

        // Recycle slot 0
        let recycled = ring.recycle_current();
        assert_eq!(recycled, Ok(0));
        assert_eq!(ring.head_index(), 1);

        // Slot 0 should be re-armed to HW ownership
        let d0_recheck = ring.desc(0);
        assert!(d0_recheck.is_some());
        if let Some(d) = d0_recheck {
            assert!(d.is_owner_hw());
        }
    }

    #[test]
    fn test_tx_slot_prepare_and_completion() {
        let mut slot = WifiTxSlot::<256>::new();
        assert!(slot.is_ready());
        assert!(!slot.is_in_flight());

        let payload = [0xAA, 0xBB, 0xCC, 0xDD, 0xEE];
        let prep_res = slot.tx_prepare(&payload);
        assert!(prep_res.is_ok());
        if let Ok(desc_addr) = prep_res {
            assert_eq!(desc_addr, slot.desc_addr());
        }

        assert!(slot.is_in_flight());
        assert!(!slot.is_ready());

        let d = slot.desc();
        assert!(d.is_owner_hw());
        assert_eq!(d.payload_len(), 5);
        assert_eq!(d.buffer_cap(), 256);
        assert_eq!(&slot.buffer_slice()[..5], &payload);

        // In-flight slot returns TxBusy
        let busy_res = slot.tx_prepare(&payload);
        assert_eq!(busy_res, Err(DmaRingError::TxBusy));

        // Mark completed
        slot.mark_completed();
        assert!(!slot.is_in_flight());
        assert!(slot.is_ready());

        // Test buffer overflow
        let oversized = [0u8; 300];
        assert_eq!(
            slot.tx_prepare(&oversized),
            Err(DmaRingError::BufferTooLarge)
        );
    }
}
