//! Memory Management Unit (MMU) mapping, RAM loading, and boot handoff.
//!
//! Provides `#![no_std]` zero-heap routines to load application RAM segments,
//! configure DPORT flash cache MMU pages, unmask cache isolation, and jump
//! to the verified application entry point.

// JUSTIFICATION: Hardware Mask ROM and DPORT MMIO registers on Xtensa target
// require unsafe FFI and volatile pointer operations.

use crate::image_verify::SegmentKind;
use crate::image_verify::VerifiedImage;
use crate::rom_boundary::AlignedBounce64;
use crate::rom_boundary::CacheControl;
use crate::rom_boundary::FlashError;
use crate::rom_boundary::FlashIo;

/// PRO CPU cache control register.
pub const DPORT_PRO_CACHE_CTRL_REG: u32 = 0x3FF0_0040;

/// PRO CPU cache control register 1 (contains flash cache mask bits).
pub const DPORT_PRO_CACHE_CTRL1_REG: u32 = 0x3FF0_0044;

/// APP CPU cache control register.
pub const DPORT_APP_CACHE_CTRL_REG: u32 = 0x3FF0_0058;

/// APP CPU cache control register 1 (contains flash cache mask bits).
pub const DPORT_APP_CACHE_CTRL1_REG: u32 = 0x3FF0_005C;

/// APP CPU cache MMU invalid access interrupt clear bit.
pub const DPORT_APP_CACHE_MMU_IA_CLR: u32 = 1 << 13;

/// Base address of PRO CPU Flash MMU page table.
pub const DPORT_PRO_FLASH_MMU_TABLE: u32 = 0x3FF1_0000;

/// Base address of APP CPU Flash MMU page table.
pub const DPORT_APP_FLASH_MMU_TABLE: u32 = 0x3FF1_2000;

/// Mask bits for flash cache isolation in `DPORT_PRO_CACHE_CTRL1_REG`.
///
/// Corresponds to bits `0..=2` (IRAM0, IRAM1, IROM0) and bit `4` (DROM0).
pub const DPORT_CACHE_MASK_FLASH_BITS: u32 = 0x0000_0017;

/// MMU page granularity in bytes (64 KiB).
pub const MMU_PAGE_SIZE: u32 = 0x1_0000;

/// MMU mapping plan calculated for a virtual memory segment.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MmuMappingPlan {
    /// 64 KiB page-aligned virtual address.
    pub vaddr_aligned: u32,
    /// 64 KiB page-aligned physical flash address.
    pub paddr_aligned: u32,
    /// Number of contiguous 64 KiB pages required.
    pub page_count: u32,
}

/// Computes the 64 KiB page-aligned MMU mapping parameters for a segment.
#[must_use]
pub const fn compute_mmu_mapping(flash_offset: u32, vaddr: u32, len: u32) -> MmuMappingPlan {
    let vaddr_aligned = vaddr & !(MMU_PAGE_SIZE - 1);
    let paddr_aligned = flash_offset & !(MMU_PAGE_SIZE - 1);
    let offset_in_page = vaddr & (MMU_PAGE_SIZE - 1);
    let page_count = if len == 0 {
        0
    } else {
        (offset_in_page + len + MMU_PAGE_SIZE - 1) >> 16
    };
    MmuMappingPlan {
        vaddr_aligned,
        paddr_aligned,
        page_count,
    }
}

/// Abstraction for writing loaded segment bytes to destination RAM.
pub trait RamSink {
    /// Writes `bytes` to RAM destination address `addr`.
    fn write_ram(&mut self, addr: u32, bytes: &[u8]);
}

/// Abstraction for clearing DPORT flash cache isolation masks.
pub trait CacheMaskControl {
    /// Unmasks flash cache bits in DPORT cache control registers.
    fn unmask_flash_cache(&mut self);
}

/// Errors occurring during RAM loading, MMU mapping, or boot jump preparation.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BootJumpError {
    /// Flash I/O error occurred while copying segment to RAM.
    Flash(FlashError),
    /// MMU mapping failed via cache controller.
    MmuMapFailed,
}

impl From<FlashError> for BootJumpError {
    fn from(err: FlashError) -> Self {
        Self::Flash(err)
    }
}

/// Copies RAM segments to memory, configures MMU mappings for flash segments,
/// and unmasks the flash cache controller.
pub fn load_ram_and_map_mmu<F: FlashIo, C: CacheControl, R: RamSink, D: CacheMaskControl>(
    flash: &mut F,
    cache: &mut C,
    ram: &mut R,
    dport: &mut D,
    image: &VerifiedImage,
) -> Result<(), BootJumpError> {
    let mut bounce = AlignedBounce64::new();
    for seg in image.segments.iter().flatten() {
        if seg.kind == SegmentKind::RamLoad {
            let mut remaining = seg.data_len;
            let mut curr_flash = seg.flash_offset;
            let mut curr_dst = seg.load_addr;
            while remaining > 0 {
                let chunk_len = core::cmp::min(remaining as usize, bounce.bytes.len());
                flash.read(curr_flash, &mut bounce.bytes[..chunk_len])?;
                ram.write_ram(curr_dst, &bounce.bytes[..chunk_len]);
                curr_flash = curr_flash.saturating_add(chunk_len as u32);
                curr_dst = curr_dst.saturating_add(chunk_len as u32);
                remaining -= chunk_len as u32;
            }
        }
    }

    cache.disable_and_flush();

    #[cfg(target_arch = "xtensa")]
    // JUSTIFICATION: Mask ROM MMU initialization requires ROM FFI call and
    // clearing APP MMU invalid access bit in DPORT_APP_CACHE_CTRL1_REG.
    // SAFETY: ROM symbol mmu_init is provided by ESP32 ROM for CPUs 0 and 1;
    // DPORT_APP_CACHE_CTRL1_REG is a valid MMIO register on ESP32.
    unsafe {
        mmu_init(0);
        let app_ctrl1 = DPORT_APP_CACHE_CTRL1_REG as *mut u32;
        app_ctrl1.write_volatile(app_ctrl1.read_volatile() | DPORT_APP_CACHE_MMU_IA_CLR);
        mmu_init(1);
        app_ctrl1.write_volatile(app_ctrl1.read_volatile() & !DPORT_APP_CACHE_MMU_IA_CLR);
    }

    for seg in image.segments.iter().flatten() {
        if seg.kind == SegmentKind::DromMap || seg.kind == SegmentKind::IromMap {
            let plan = compute_mmu_mapping(seg.flash_offset, seg.load_addr, seg.data_len);
            if plan.page_count > 0 {
                if !cache.mmu_map_pages(0, plan.vaddr_aligned, plan.paddr_aligned, plan.page_count)
                {
                    return Err(BootJumpError::MmuMapFailed);
                }
                if !cache.mmu_map_pages(1, plan.vaddr_aligned, plan.paddr_aligned, plan.page_count)
                {
                    return Err(BootJumpError::MmuMapFailed);
                }
            }
        }
    }

    dport.unmask_flash_cache();
    cache.enable_cache(0);
    cache.flush_uart0();

    Ok(())
}

#[cfg(target_arch = "xtensa")]
extern "C" {
    fn mmu_init(cpu: u32);
}

/// Writes up to `4 - lane_offset` bytes into the 32-bit little-endian word
/// at `word_ptr` using a single 32-bit volatile read-modify-write.
///
/// Preserves unpatched byte lanes via a bitwise mask, returning the number
/// of bytes actually written.
///
/// # Safety
///
/// `word_ptr` must be 4-byte aligned and point to valid, readable and writable
/// 32-bit memory. `lane_offset` must be strictly less than 4.
// JUSTIFICATION: Subword write requires a single 32-bit volatile read and
// write (l32i + s32i) to prevent unaligned bus faults on Xtensa hardware.
// SAFETY: Caller guarantees `word_ptr` is 4-byte aligned and points to valid,
// initialized memory.
pub(crate) unsafe fn write_partial_word_le(
    word_ptr: *mut u32,
    lane_offset: usize,
    bytes: &[u8],
) -> usize {
    if lane_offset >= 4 || bytes.is_empty() {
        return 0;
    }
    let count = core::cmp::min(4 - lane_offset, bytes.len());
    let mut mask = 0_u32;
    let mut patch = 0_u32;
    let mut i = 0;
    while i < count {
        let shift = (lane_offset + i) * 8;
        mask |= 0xFF_u32 << shift;
        patch |= (bytes[i] as u32) << shift;
        i += 1;
    }
    // JUSTIFICATION: Subword RMW requires 32-bit volatile read and write.
    // SAFETY: Caller guarantees `word_ptr` is 4-byte aligned and writable.
    unsafe {
        let prev = word_ptr.read_volatile();
        word_ptr.write_volatile((prev & !mask) | patch);
    }
    count
}

/// Writes arbitrary byte slices to 32-bit word-aligned memory using 32-bit
/// volatile stores and subword read-modify-write for trailing bytes.
///
/// # Safety
///
/// Caller must ensure `dst_words` is 4-byte aligned and points to valid,
/// writable memory of at least `bytes.len()` bytes (rounded up to 4 bytes
/// if a partial trailing word is present).
// JUSTIFICATION: Hardware RAM writes require 32-bit aligned volatile store
// sequences to avoid unaligned bus faults on Xtensa architecture.
// SAFETY: Caller guarantees `dst_words` is 4-byte aligned and writable for the
// byte span covered by `bytes`.
pub unsafe fn write_ram_words_32(dst_words: *mut u32, bytes: &[u8]) {
    if bytes.is_empty() {
        return;
    }
    let full_words = bytes.len() / 4;
    let mut i = 0;
    while i < full_words {
        let chunk_start = i * 4;
        let word_bytes: [u8; 4] = [
            bytes[chunk_start],
            bytes[chunk_start + 1],
            bytes[chunk_start + 2],
            bytes[chunk_start + 3],
        ];
        let val = u32::from_le_bytes(word_bytes);
        // JUSTIFICATION: Volatile 32-bit store (s32i) avoids bus fault.
        // SAFETY: Caller guarantees dst_words.add(i) is in-bounds and writable.
        unsafe {
            dst_words.add(i).write_volatile(val);
        }
        i += 1;
    }
    let remainder = bytes.len() % 4;
    if remainder > 0 {
        // JUSTIFICATION: Subword tail requires 32-bit volatile RMW.
        // SAFETY: Caller guarantees dst_words.add(full_words) is writable.
        unsafe {
            write_partial_word_le(dst_words.add(full_words), 0, &bytes[full_words * 4..]);
        }
    }
}

/// RAM sink for Xtensa target that writes verified image bytes to SRAM.
#[cfg(target_arch = "xtensa")]
#[derive(Debug, Default, Clone, Copy)]
pub struct XtensaRamSink;

#[cfg(target_arch = "xtensa")]
impl RamSink for XtensaRamSink {
    fn write_ram(&mut self, addr: u32, bytes: &[u8]) {
        if bytes.is_empty() {
            return;
        }
        let lane = (addr as usize) & 3;
        let mut aligned_word = (addr as usize & !3) as *mut u32;
        let mut remaining_bytes = bytes;
        if lane != 0 {
            // JUSTIFICATION: Unaligned leading bytes require 32-bit volatile
            // read-modify-write at the enclosing word boundary.
            // SAFETY: Destination addresses are verified against bootloader
            // SRAM overlap, and aligned_word points to valid writable SRAM.
            let written = unsafe { write_partial_word_le(aligned_word, lane, remaining_bytes) };
            // JUSTIFICATION: Advancing word pointer past partially written word.
            // SAFETY: Pointer is advanced by 1 word within verified SRAM region.
            aligned_word = unsafe { aligned_word.add(1) };
            remaining_bytes = &remaining_bytes[written..];
        }
        if !remaining_bytes.is_empty() {
            // JUSTIFICATION: Aligned segment payload writes require 32-bit
            // volatile store sequences.
            // SAFETY: Destination addresses are verified against bootloader
            // SRAM overlap and aligned_word is 4-byte aligned.
            unsafe {
                write_ram_words_32(aligned_word, remaining_bytes);
            }
        }
    }
}

/// DPORT cache mask controller for Xtensa hardware.
#[cfg(target_arch = "xtensa")]
#[derive(Debug, Default, Clone, Copy)]
pub struct XtensaDportMask;

#[cfg(target_arch = "xtensa")]
impl CacheMaskControl for XtensaDportMask {
    fn unmask_flash_cache(&mut self) {
        // JUSTIFICATION: Modifying hardware DPORT MMIO to unmask flash cache.
        // SAFETY: DPORT cache control register addresses are valid ESP32 MMIO.
        unsafe {
            let pro_reg = DPORT_PRO_CACHE_CTRL1_REG as *mut u32;
            let app_reg = DPORT_APP_CACHE_CTRL1_REG as *mut u32;
            pro_reg.write_volatile(pro_reg.read_volatile() & !DPORT_CACHE_MASK_FLASH_BITS);
            app_reg.write_volatile(app_reg.read_volatile() & !DPORT_CACHE_MASK_FLASH_BITS);
        }
    }
}

/// Hands off control to the application entry point.
///
/// # Safety
///
/// The entry address must point to a valid Xtensa Windowed ABI entry point.
// JUSTIFICATION: Target handoff requires direct machine jump to entry point.
// SAFETY: The entry address is validated during image verification.
#[cfg(target_arch = "xtensa")]
pub unsafe fn jump_to_entry(entry_addr: u32) -> ! {
    // JUSTIFICATION: Target handoff requires transmuting the verified entry
    // address into a diverging extern "C" function pointer.
    // SAFETY: Caller guarantees `entry_addr` points to a validated Xtensa
    // Windowed ABI entry point inside a verified application segment.
    let entry_fn: extern "C" fn() -> ! = unsafe { core::mem::transmute(entry_addr as usize) };
    entry_fn()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::image_verify::VerifiedSegment;
    use crate::image_verify::MAX_SEGMENTS;
    use crate::rom_boundary::MockFlash;

    #[test]
    fn test_compute_mmu_mapping_aligned_and_unaligned() {
        // Aligned 64 KiB page
        let plan_aligned = compute_mmu_mapping(0x20000, 0x3F40_0000, 0x1_0000);
        assert_eq!(plan_aligned.vaddr_aligned, 0x3F40_0000);
        assert_eq!(plan_aligned.paddr_aligned, 0x20000);
        assert_eq!(plan_aligned.page_count, 1);

        // Unaligned by +0x20 bytes crossing page boundary
        let plan_unaligned = compute_mmu_mapping(0x20020, 0x3F40_0020, 0x1_0000);
        assert_eq!(plan_unaligned.vaddr_aligned, 0x3F40_0000);
        assert_eq!(plan_unaligned.paddr_aligned, 0x20000);
        assert_eq!(plan_unaligned.page_count, 2);

        // Zero length
        let plan_zero = compute_mmu_mapping(0x20000, 0x3F40_0000, 0);
        assert_eq!(plan_zero.page_count, 0);
    }

    struct TestRamSink {
        written_bytes: [u8; 128],
        write_len: usize,
        last_addr: u32,
    }

    impl TestRamSink {
        fn new() -> Self {
            Self {
                written_bytes: [0u8; 128],
                write_len: 0,
                last_addr: 0,
            }
        }
    }

    impl RamSink for TestRamSink {
        fn write_ram(&mut self, addr: u32, bytes: &[u8]) {
            self.last_addr = addr;
            for &b in bytes {
                if self.write_len < self.written_bytes.len() {
                    self.written_bytes[self.write_len] = b;
                    self.write_len += 1;
                }
            }
        }
    }

    struct TestCacheMask {
        unmasked: bool,
    }

    impl TestCacheMask {
        const fn new() -> Self {
            Self { unmasked: false }
        }
    }

    impl CacheMaskControl for TestCacheMask {
        fn unmask_flash_cache(&mut self) {
            self.unmasked = true;
        }
    }

    #[derive(Default)]
    struct RecordingCache {
        disabled_and_flushed: bool,
        map_calls: usize,
        mapped_vaddr: [u32; 4],
        mapped_paddr: [u32; 4],
        mapped_pages: [u32; 4],
        enabled_cpu: Option<u32>,
        uart_flushed: bool,
    }

    impl CacheControl for RecordingCache {
        fn disable_and_flush(&mut self) {
            self.disabled_and_flushed = true;
        }

        fn mmu_map_pages(&mut self, _cpu: u32, vaddr: u32, paddr: u32, page_count: u32) -> bool {
            if self.map_calls < 4 {
                self.mapped_vaddr[self.map_calls] = vaddr;
                self.mapped_paddr[self.map_calls] = paddr;
                self.mapped_pages[self.map_calls] = page_count;
                self.map_calls += 1;
            }
            true
        }

        fn enable_cache(&mut self, cpu: u32) {
            self.enabled_cpu = Some(cpu);
        }

        fn flush_uart0(&mut self) {
            self.uart_flushed = true;
        }

        fn delay_us(&mut self, _us: u32) {}
    }

    #[test]
    fn test_load_ram_and_map_mmu_full_sequence() {
        let mut flash = MockFlash::<2>::new();
        let ram_payload = [0x55u8; 64];
        flash
            .write_raw(0x100, &ram_payload)
            .expect("write_raw failed");

        let mut segments: [Option<VerifiedSegment>; MAX_SEGMENTS] = [None; MAX_SEGMENTS];
        segments[0] = Some(VerifiedSegment {
            flash_offset: 0x100,
            load_addr: 0x4008_0000,
            data_len: 64,
            kind: SegmentKind::RamLoad,
        });
        segments[1] = Some(VerifiedSegment {
            flash_offset: 0x20020,
            load_addr: 0x3F40_0020,
            data_len: 0x8000,
            kind: SegmentKind::DromMap,
        });

        let image = VerifiedImage {
            entry_addr: 0x4008_0000,
            segment_count: 2,
            segments,
            image_end_offset: 0x30000,
        };

        let mut cache = RecordingCache::default();
        let mut ram = TestRamSink::new();
        let mut dport = TestCacheMask::new();

        load_ram_and_map_mmu(&mut flash, &mut cache, &mut ram, &mut dport, &image)
            .expect("load_ram_and_map_mmu should succeed");

        // Verify RAM loading
        assert_eq!(ram.write_len, 64);
        assert_eq!(&ram.written_bytes[..64], &ram_payload[..]);

        // Verify Cache flush & MMU map
        assert!(cache.disabled_and_flushed);
        assert_eq!(cache.map_calls, 2); // CPU 0 and CPU 1
        assert_eq!(cache.mapped_vaddr[0], 0x3F40_0000);
        assert_eq!(cache.mapped_paddr[0], 0x20000);
        assert_eq!(cache.mapped_pages[0], 1);

        // Verify DPORT cache unmask and re-enable
        assert!(dport.unmasked);
        assert_eq!(cache.enabled_cpu, Some(0));
        assert!(cache.uart_flushed);
    }

    #[test]
    fn test_write_ram_words_32_short_8byte_iram_tail() {
        let mut mem = [0xDEAD_BEEFu32; 8];
        let payload: [u8; 8] = [1, 2, 3, 4, 5, 6, 7, 8];
        // JUSTIFICATION: Test verifies 32-bit word-aligned RAM writing.
        // SAFETY: `mem` contains 8 u32 words (32 bytes), payload is 8 bytes.
        unsafe {
            write_ram_words_32(mem.as_mut_ptr(), &payload);
        }
        assert_eq!(mem[0], u32::from_le_bytes([1, 2, 3, 4]));
        assert_eq!(mem[1], u32::from_le_bytes([5, 6, 7, 8]));
        for &word in &mem[2..8] {
            assert_eq!(word, 0xDEAD_BEEF);
        }
    }

    #[test]
    fn test_write_ram_words_32_full_64_and_subword_rmw() {
        // 64-byte write (16 full 32-bit words).
        let mut mem_64 = [0xDEAD_BEEFu32; 20];
        let payload_64 = [0xA5u8; 64];
        // JUSTIFICATION: Test verifies 64-byte aligned volatile word writes.
        // SAFETY: `mem_64` has 20 words (80 bytes); payload is 64 bytes.
        unsafe {
            write_ram_words_32(mem_64.as_mut_ptr(), &payload_64);
        }
        for &word in &mem_64[..16] {
            assert_eq!(word, 0xA5A5_A5A5);
        }
        for &word in &mem_64[16..20] {
            assert_eq!(word, 0xDEAD_BEEF);
        }

        // 12-byte write (3 full 32-bit words).
        let mut mem_12 = [0xDEAD_BEEFu32; 4];
        let payload_12: [u8; 12] = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12];
        // JUSTIFICATION: Test verifies 12-byte aligned volatile word writes.
        // SAFETY: `mem_12` has 4 words (16 bytes); payload is 12 bytes.
        unsafe {
            write_ram_words_32(mem_12.as_mut_ptr(), &payload_12);
        }
        assert_eq!(mem_12[0], u32::from_le_bytes([1, 2, 3, 4]));
        assert_eq!(mem_12[1], u32::from_le_bytes([5, 6, 7, 8]));
        assert_eq!(mem_12[2], u32::from_le_bytes([9, 10, 11, 12]));
        assert_eq!(mem_12[3], 0xDEAD_BEEF);

        // 10-byte write (2 full words + 2-byte tail RMW preserving upper 16 bits).
        let mut mem_10 = [0xDEAD_BEEFu32; 4];
        let payload_10: [u8; 10] = [1, 2, 3, 4, 5, 6, 7, 8, 0x34, 0x12];
        // JUSTIFICATION: Test verifies subword RMW preserving untouched lanes.
        // SAFETY: `mem_10` has 4 words (16 bytes); payload is 10 bytes.
        unsafe {
            write_ram_words_32(mem_10.as_mut_ptr(), &payload_10);
        }
        assert_eq!(mem_10[0], u32::from_le_bytes([1, 2, 3, 4]));
        assert_eq!(mem_10[1], u32::from_le_bytes([5, 6, 7, 8]));
        // Word 2: lower 16 bits are [0x34, 0x12], upper 16 bits preserved from
        // 0xDEAD_BEEF (0xDEAD).
        assert_eq!(mem_10[2], 0xDEAD_1234);
        assert_eq!(mem_10[3], 0xDEAD_BEEF);
    }

    #[test]
    fn test_load_ram_72_byte_segment_with_8_byte_tail() {
        let mut flash = MockFlash::<2>::new();
        let mut ram_payload = [0x55u8; 72];
        for (idx, b) in ram_payload.iter_mut().enumerate() {
            *b = (idx & 0xFF) as u8;
        }
        flash
            .write_raw(0x100, &ram_payload)
            .expect("write_raw failed");

        let mut segments: [Option<VerifiedSegment>; MAX_SEGMENTS] = [None; MAX_SEGMENTS];
        segments[0] = Some(VerifiedSegment {
            flash_offset: 0x100,
            load_addr: 0x4008_0000,
            data_len: 72,
            kind: SegmentKind::RamLoad,
        });

        let image = VerifiedImage {
            entry_addr: 0x4008_0000,
            segment_count: 1,
            segments,
            image_end_offset: 0x30000,
        };

        let mut cache = RecordingCache::default();
        let mut ram = TestRamSink::new();
        let mut dport = TestCacheMask::new();

        load_ram_and_map_mmu(&mut flash, &mut cache, &mut ram, &mut dport, &image)
            .expect("load_ram_and_map_mmu should succeed");

        assert_eq!(ram.write_len, 72);
        assert_eq!(&ram.written_bytes[..72], &ram_payload[..]);
        assert_eq!(ram.last_addr, 0x4008_0040);
    }

    #[test]
    fn test_write_partial_word_le_unaligned_lanes() {
        // lane_offset = 1, 2 bytes into 0xDEAD_BEEF
        let mut word = 0xDEAD_BEEFu32;
        let slice1 = &[0x11, 0x22];
        let ptr1 = &mut word as *mut u32;
        // JUSTIFICATION: Test verifies subword RMW at lane offset 1.
        // SAFETY: Pointer is valid aligned stack allocation.
        let written = unsafe { write_partial_word_le(ptr1, 1, slice1) };
        assert_eq!(written, 2);
        // byte 0: 0xEF, byte 1: 0x11, byte 2: 0x22, byte 3: 0xDE -> 0xDE22_11EF
        assert_eq!(word, 0xDE22_11EF);

        // lane_offset = 2, 3 bytes (should cap at 2 bytes since 4 - 2 = 2)
        let mut word2 = 0xDEAD_BEEFu32;
        let slice2 = &[0x33, 0x44, 0x55];
        let ptr2 = &mut word2 as *mut u32;
        // JUSTIFICATION: Test verifies subword RMW with count truncation.
        // SAFETY: Pointer is valid aligned stack allocation.
        let written2 = unsafe { write_partial_word_le(ptr2, 2, slice2) };
        assert_eq!(written2, 2);
        // byte 0: 0xEF, byte 1: 0xBE, byte 2: 0x33, byte 3: 0x44 -> 0x4433_BEEF
        assert_eq!(word2, 0x4433_BEEF);

        // lane_offset = 3, 1 byte
        let mut word3 = 0xDEAD_BEEFu32;
        let slice3 = &[0x77];
        let ptr3 = &mut word3 as *mut u32;
        // JUSTIFICATION: Test verifies subword RMW at final byte lane.
        // SAFETY: Pointer is valid aligned stack allocation.
        let written3 = unsafe { write_partial_word_le(ptr3, 3, slice3) };
        assert_eq!(written3, 1);
        // byte 0: 0xEF, byte 1: 0xBE, byte 2: 0xAD, byte 3: 0x77 -> 0x77AD_BEEF
        assert_eq!(word3, 0x77AD_BEEF);

        // lane_offset >= 4 edge case: returns 0 and does not modify
        let mut word4 = 0xDEAD_BEEFu32;
        let slice4 = &[0x88];
        let ptr4 = &mut word4 as *mut u32;
        // JUSTIFICATION: Test verifies out-of-range lane offset safety.
        // SAFETY: Pointer is valid aligned stack allocation.
        let written4 = unsafe { write_partial_word_le(ptr4, 4, slice4) };
        assert_eq!(written4, 0);
        assert_eq!(word4, 0xDEAD_BEEF);
    }
}
