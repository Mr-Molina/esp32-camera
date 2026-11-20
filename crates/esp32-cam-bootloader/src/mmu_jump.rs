//! Memory Management Unit (MMU) mapping, RAM loading, and boot handoff.
//!
//! Provides `#![no_std]` zero-heap routines to load application RAM segments,
//! configure DPORT flash cache MMU pages, unmask cache isolation, and jump
//! to the verified application entry point.

// JUSTIFICATION: Hardware Mask ROM and DPORT MMIO registers on Xtensa target
// require unsafe FFI and volatile pointer operations.

#[cfg(target_arch = "xtensa")]
use core::sync::atomic::AtomicBool;
#[cfg(target_arch = "xtensa")]
use core::sync::atomic::Ordering;

use crate::image_verify::is_valid_app_ram_span;
use crate::image_verify::is_valid_executable_addr;
use crate::image_verify::SegmentKind;
use crate::image_verify::VerifiedImage;
use crate::image_verify::MAX_SEGMENTS;
use crate::image_verify::SOC_APP_IRAM_LOW;
use crate::image_verify::SOC_DROM_HIGH;
use crate::image_verify::SOC_DROM_LOW;
use crate::image_verify::SOC_IROM_HIGH;
use crate::image_verify::SOC_IROM_LOW;
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

/// Maximum ESP32-WROVER SPI flash size in bytes (4 MiB).
pub const MAX_MMU_FLASH_BYTES: u64 = 4 * 1024 * 1024;

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

/// Errors occurring during RAM loading, MMU mapping, or boot jump preparation.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BootJumpError {
    /// Flash I/O error occurred while copying segment to RAM.
    Flash(FlashError),
    /// MMU mapping failed via cache controller.
    MmuMapFailed,
    /// MMU mapping parameters overflow, break page parity, or exceed bounds.
    MmuMappingInvalid,
    /// RAM write destination fails alignment, overflow, or SRAM allowlist.
    InvalidRamWrite,
    /// Entry point address is zero or outside verified executable segments.
    InvalidEntryAddress,
}

impl From<FlashError> for BootJumpError {
    fn from(err: FlashError) -> Self {
        Self::Flash(err)
    }
}

/// Validates a destination RAM write span against 4-byte alignment,
/// non-null/non-wrap arithmetic, and the ESP32 application SRAM allowlist.
#[inline]
pub const fn validate_ram_write_span(addr: u32, len: usize) -> Result<(), BootJumpError> {
    if len == 0 {
        return Ok(());
    }
    if addr == 0 || (addr & 3) != 0 || (len & 3) != 0 || len > u32::MAX as usize {
        return Err(BootJumpError::InvalidRamWrite);
    }
    let len_u32 = len as u32;
    if !is_valid_app_ram_span(addr, len_u32) {
        return Err(BootJumpError::InvalidRamWrite);
    }
    Ok(())
}

/// Computes the 64 KiB page-aligned MMU mapping parameters for a segment.
///
/// Enforces `u64` overflow-free arithmetic, 64 KiB intra-page offset parity
/// between `flash_offset` and `vaddr`, DROM/IROM virtual bounds, and the 4 MiB
/// physical flash ceiling.
pub const fn compute_mmu_mapping(
    flash_offset: u32,
    vaddr: u32,
    len: u32,
) -> Result<MmuMappingPlan, BootJumpError> {
    let page_mask = MMU_PAGE_SIZE - 1;
    let offset_in_page = vaddr & page_mask;
    if offset_in_page != (flash_offset & page_mask) {
        return Err(BootJumpError::MmuMappingInvalid);
    }

    let vaddr_u64 = vaddr as u64;
    let flash_u64 = flash_offset as u64;
    let len_u64 = len as u64;
    let v_end = vaddr_u64 + len_u64;
    let p_end = flash_u64 + len_u64;

    let drom_lo = SOC_DROM_LOW as u64;
    let drom_hi = SOC_DROM_HIGH as u64;
    let irom_lo = SOC_IROM_LOW as u64;
    let irom_hi = SOC_IROM_HIGH as u64;

    let region_hi = if vaddr_u64 >= drom_lo && v_end <= drom_hi && (len > 0 || vaddr_u64 < drom_hi)
    {
        drom_hi
    } else if vaddr_u64 >= irom_lo && v_end <= irom_hi && (len > 0 || vaddr_u64 < irom_hi) {
        irom_hi
    } else {
        return Err(BootJumpError::MmuMappingInvalid);
    };

    if p_end > MAX_MMU_FLASH_BYTES || (len == 0 && flash_u64 >= MAX_MMU_FLASH_BYTES) {
        return Err(BootJumpError::MmuMappingInvalid);
    }

    let vaddr_aligned = vaddr & !page_mask;
    let paddr_aligned = flash_offset & !page_mask;
    let page_count_u64 = if len == 0 {
        0
    } else {
        let page_size_u64 = MMU_PAGE_SIZE as u64;
        ((offset_in_page as u64) + len_u64 + page_size_u64 - 1) >> 16
    };

    let mapped_bytes = page_count_u64 * (MMU_PAGE_SIZE as u64);
    if (vaddr_aligned as u64) + mapped_bytes > region_hi
        || (paddr_aligned as u64) + mapped_bytes > MAX_MMU_FLASH_BYTES
    {
        return Err(BootJumpError::MmuMappingInvalid);
    }

    Ok(MmuMappingPlan {
        vaddr_aligned,
        paddr_aligned,
        page_count: page_count_u64 as u32,
    })
}

/// Abstraction for writing loaded segment bytes to destination RAM.
pub trait RamSink {
    /// Writes `bytes` to RAM destination address `addr`.
    fn write_ram(&mut self, addr: u32, bytes: &[u8]) -> Result<(), BootJumpError>;
}

/// Abstraction for controlling DPORT flash cache isolation and MMU IA clear
/// registers.
pub trait CacheMaskControl {
    /// Sets or clears `DPORT_APP_CACHE_MMU_IA_CLR` around APP MMU reset.
    fn set_app_mmu_ia_clr(&mut self, enabled: bool);

    /// Unmasks flash cache bits in DPORT cache control registers.
    fn unmask_flash_cache(&mut self);
}

fn verify_image_entry_for_load(image: &VerifiedImage) -> Result<(), BootJumpError> {
    let entry = image.entry_addr();
    if !is_valid_executable_addr(entry) {
        return Err(BootJumpError::InvalidEntryAddress);
    }
    for seg in image.segments().iter().flatten() {
        let is_exec = match seg.kind() {
            SegmentKind::IromMap => true,
            SegmentKind::RamLoad => seg.load_addr() >= SOC_APP_IRAM_LOW,
            SegmentKind::DromMap | SegmentKind::RtcSkip => false,
        };
        if !is_exec || seg.data_len() == 0 {
            continue;
        }
        if let Some(seg_end) = seg.load_addr().checked_add(seg.data_len()) {
            if entry >= seg.load_addr() && entry < seg_end {
                return Ok(());
            }
        }
    }
    Err(BootJumpError::InvalidEntryAddress)
}

/// Copies RAM segments to memory, configures MMU mappings for flash segments,
/// and unmasks the flash cache controller.
///
/// Pre-validates entry bounds, RAM write spans, and all MMU plans before
/// mutating cache, RAM, or MMU state. Disables flash cache prior to SPI flash
/// reads and flushes APP CPU cache after MMU programming.
pub fn load_ram_and_map_mmu<F: FlashIo, C: CacheControl, R: RamSink, D: CacheMaskControl>(
    flash: &mut F,
    cache: &mut C,
    ram: &mut R,
    dport: &mut D,
    image: &VerifiedImage,
) -> Result<(), BootJumpError> {
    verify_image_entry_for_load(image)?;

    let mut mmu_plans: [Option<MmuMappingPlan>; MAX_SEGMENTS] = [None; MAX_SEGMENTS];
    let mut plan_count: usize = 0;

    for seg in image.segments().iter().flatten() {
        match seg.kind() {
            SegmentKind::RamLoad => {
                validate_ram_write_span(seg.load_addr(), seg.data_len() as usize)?;
                let _flash_end = seg
                    .flash_offset()
                    .checked_add(seg.data_len())
                    .ok_or(BootJumpError::InvalidRamWrite)?;
            }
            SegmentKind::DromMap | SegmentKind::IromMap => {
                let plan =
                    compute_mmu_mapping(seg.flash_offset(), seg.load_addr(), seg.data_len())?;
                if plan.page_count > 0 {
                    if plan_count >= MAX_SEGMENTS {
                        return Err(BootJumpError::MmuMappingInvalid);
                    }
                    mmu_plans[plan_count] = Some(plan);
                    plan_count += 1;
                }
            }
            SegmentKind::RtcSkip => {}
        }
    }

    cache.disable_and_flush();

    let mut bounce = AlignedBounce64::new();
    for seg in image.segments().iter().flatten() {
        if seg.kind() == SegmentKind::RamLoad {
            let mut remaining = seg.data_len();
            let mut curr_flash = seg.flash_offset();
            let mut curr_dst = seg.load_addr();
            while remaining > 0 {
                let chunk_len = core::cmp::min(remaining as usize, bounce.bytes.len());
                flash.read(curr_flash, &mut bounce.bytes[..chunk_len])?;
                ram.write_ram(curr_dst, &bounce.bytes[..chunk_len])?;
                remaining -= chunk_len as u32;
                if remaining > 0 {
                    curr_flash = curr_flash
                        .checked_add(chunk_len as u32)
                        .ok_or(BootJumpError::InvalidRamWrite)?;
                    curr_dst = curr_dst
                        .checked_add(chunk_len as u32)
                        .ok_or(BootJumpError::InvalidRamWrite)?;
                }
            }
        }
    }

    cache.mmu_init(0);
    dport.set_app_mmu_ia_clr(true);
    cache.mmu_init(1);
    dport.set_app_mmu_ia_clr(false);

    for plan in mmu_plans.iter().take(plan_count).flatten() {
        if !cache.mmu_map_pages(0, plan.vaddr_aligned, plan.paddr_aligned, plan.page_count) {
            return Err(BootJumpError::MmuMapFailed);
        }
        if !cache.mmu_map_pages(1, plan.vaddr_aligned, plan.paddr_aligned, plan.page_count) {
            return Err(BootJumpError::MmuMapFailed);
        }
    }

    cache.flush_cache(1);
    dport.unmask_flash_cache();
    cache.enable_cache(0);
    cache.flush_uart0();

    Ok(())
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

#[cfg(target_arch = "xtensa")]
static XTENSA_RAM_SINK_TAKEN: AtomicBool = AtomicBool::new(false);

#[cfg(target_arch = "xtensa")]
static XTENSA_DPORT_MASK_TAKEN: AtomicBool = AtomicBool::new(false);

/// RAM sink singleton for Xtensa target that writes verified bytes to SRAM.
#[cfg(target_arch = "xtensa")]
#[derive(Debug)]
pub struct XtensaRamSink {
    _private: (),
}

#[cfg(target_arch = "xtensa")]
impl XtensaRamSink {
    /// Claims exclusive ownership of the Xtensa RAM sink singleton.
    ///
    /// Returns `None` if the singleton has already been claimed.
    #[must_use]
    pub fn take() -> Option<Self> {
        if XTENSA_RAM_SINK_TAKEN
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
            .is_ok()
        {
            Some(Self { _private: () })
        } else {
            None
        }
    }
}

#[cfg(target_arch = "xtensa")]
impl RamSink for XtensaRamSink {
    fn write_ram(&mut self, addr: u32, bytes: &[u8]) -> Result<(), BootJumpError> {
        validate_ram_write_span(addr, bytes.len())?;
        if bytes.is_empty() {
            return Ok(());
        }
        let aligned_word = (addr as usize) as *mut u32;
        // JUSTIFICATION: Aligned segment payload writes require 32-bit
        // volatile store sequences on Xtensa SRAM.
        // SAFETY: `validate_ram_write_span` verified `addr != 0`, 4-byte
        // alignment of both `addr` and `bytes.len()`, no `u32` wrap, and
        // strict membership in allowlisted application DRAM0/IRAM0.
        unsafe {
            write_ram_words_32(aligned_word, bytes);
        }
        Ok(())
    }
}

/// DPORT cache mask controller singleton for Xtensa hardware.
#[cfg(target_arch = "xtensa")]
#[derive(Debug)]
pub struct XtensaDportMask {
    _private: (),
}

#[cfg(target_arch = "xtensa")]
impl XtensaDportMask {
    /// Claims exclusive ownership of the Xtensa DPORT mask singleton.
    ///
    /// Returns `None` if the singleton has already been claimed.
    #[must_use]
    pub fn take() -> Option<Self> {
        if XTENSA_DPORT_MASK_TAKEN
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
            .is_ok()
        {
            Some(Self { _private: () })
        } else {
            None
        }
    }
}

#[cfg(target_arch = "xtensa")]
impl CacheMaskControl for XtensaDportMask {
    fn set_app_mmu_ia_clr(&mut self, enabled: bool) {
        // JUSTIFICATION: Toggling APP MMU invalid-access clear bit in DPORT
        // hardware MMIO register around `mmu_init(1)`.
        // SAFETY: `DPORT_APP_CACHE_CTRL1_REG` is a valid 4-byte-aligned
        // ESP32 MMIO register owned exclusively by this singleton.
        unsafe {
            let app_ctrl1 = DPORT_APP_CACHE_CTRL1_REG as *mut u32;
            let cur = app_ctrl1.read_volatile();
            let next = if enabled {
                cur | DPORT_APP_CACHE_MMU_IA_CLR
            } else {
                cur & !DPORT_APP_CACHE_MMU_IA_CLR
            };
            app_ctrl1.write_volatile(next);
        }
    }

    fn unmask_flash_cache(&mut self) {
        // JUSTIFICATION: Modifying hardware DPORT MMIO to unmask flash cache.
        // SAFETY: DPORT cache control register addresses are valid ESP32 MMIO
        // owned exclusively by this singleton.
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
/// The entry address must point to a valid Xtensa Windowed ABI entry point
/// inside a verified executable segment.
// JUSTIFICATION: Target handoff requires direct machine jump to entry point.
// SAFETY: Entry address is validated both during image verification and
// against `is_valid_executable_addr` immediately before transmuting.
#[cfg(target_arch = "xtensa")]
#[allow(clippy::empty_loop)]
pub unsafe fn jump_to_entry(entry_addr: u32) -> ! {
    if !is_valid_executable_addr(entry_addr) {
        let mut wdt = crate::wdt::XtensaWdtMmio;
        crate::wdt::trigger_sw_reset(&mut wdt);
        loop {}
    }
    // JUSTIFICATION: Target handoff requires transmuting the verified entry
    // address into a diverging extern "C" function pointer.
    // SAFETY: `entry_addr` is verified non-zero and inside executable ESP32
    // IRAM0/IROM, and caller guarantees valid Xtensa Windowed ABI code there.
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
        let plan_aligned =
            compute_mmu_mapping(0x20000, 0x3F40_0000, 0x1_0000).expect("aligned mapping");
        assert_eq!(plan_aligned.vaddr_aligned, 0x3F40_0000);
        assert_eq!(plan_aligned.paddr_aligned, 0x20000);
        assert_eq!(plan_aligned.page_count, 1);

        // Unaligned by +0x20 bytes crossing page boundary
        let plan_unaligned =
            compute_mmu_mapping(0x20020, 0x3F40_0020, 0x1_0000).expect("unaligned mapping");
        assert_eq!(plan_unaligned.vaddr_aligned, 0x3F40_0000);
        assert_eq!(plan_unaligned.paddr_aligned, 0x20000);
        assert_eq!(plan_unaligned.page_count, 2);

        // Zero length
        let plan_zero = compute_mmu_mapping(0x20000, 0x3F40_0000, 0).expect("zero-length mapping");
        assert_eq!(plan_zero.page_count, 0);

        // Intra-page parity mismatch rejected
        assert_eq!(
            compute_mmu_mapping(0x20020, 0x3F40_0040, 0x1000),
            Err(BootJumpError::MmuMappingInvalid)
        );

        // u32 overflow rejected without wrapping
        assert_eq!(
            compute_mmu_mapping(0x20000, 0x3F40_0000, u32::MAX),
            Err(BootJumpError::MmuMappingInvalid)
        );

        // Exceeds DROM upper bound rejected
        assert_eq!(
            compute_mmu_mapping(0x20000, 0x3F7F_0000, 0x2_0000),
            Err(BootJumpError::MmuMappingInvalid)
        );

        // Exceeds 4 MiB flash bound rejected
        assert_eq!(
            compute_mmu_mapping(0x3F_0000, 0x3F40_0000, 0x2_0000),
            Err(BootJumpError::MmuMappingInvalid)
        );

        // Virtual address outside DROM/IROM rejected
        assert_eq!(
            compute_mmu_mapping(0x20000, 0x4008_0000, 0x1000),
            Err(BootJumpError::MmuMappingInvalid)
        );
    }

    #[test]
    fn test_validate_ram_write_span_hardening() {
        assert_eq!(validate_ram_write_span(0x4008_0000, 64), Ok(()));
        assert_eq!(validate_ram_write_span(0x3FFB_0000, 64), Ok(()));
        assert_eq!(validate_ram_write_span(0x4008_0000, 0), Ok(()));

        // Unaligned address or length
        assert_eq!(
            validate_ram_write_span(0x4008_0001, 64),
            Err(BootJumpError::InvalidRamWrite)
        );
        assert_eq!(
            validate_ram_write_span(0x4008_0000, 62),
            Err(BootJumpError::InvalidRamWrite)
        );

        // NULL / wrap-to-NULL
        assert_eq!(
            validate_ram_write_span(0, 4),
            Err(BootJumpError::InvalidRamWrite)
        );
        assert_eq!(
            validate_ram_write_span(0xFFFF_FFFC, 8),
            Err(BootJumpError::InvalidRamWrite)
        );

        // MMIO & bootloader SRAM rejected
        assert_eq!(
            validate_ram_write_span(0x3FF0_0040, 4),
            Err(BootJumpError::InvalidRamWrite)
        );
        assert_eq!(
            validate_ram_write_span(0x3FFF_0000, 4),
            Err(BootJumpError::InvalidRamWrite)
        );
        assert_eq!(
            validate_ram_write_span(0x4007_8000, 4),
            Err(BootJumpError::InvalidRamWrite)
        );
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
        fn write_ram(&mut self, addr: u32, bytes: &[u8]) -> Result<(), BootJumpError> {
            validate_ram_write_span(addr, bytes.len())?;
            self.last_addr = addr;
            for &b in bytes {
                if self.write_len < self.written_bytes.len() {
                    self.written_bytes[self.write_len] = b;
                    self.write_len += 1;
                }
            }
            Ok(())
        }
    }

    struct TestCacheMask {
        unmasked: bool,
        ia_clr_events: [bool; 4],
        ia_clr_count: usize,
    }

    impl TestCacheMask {
        const fn new() -> Self {
            Self {
                unmasked: false,
                ia_clr_events: [false; 4],
                ia_clr_count: 0,
            }
        }
    }

    impl CacheMaskControl for TestCacheMask {
        fn set_app_mmu_ia_clr(&mut self, enabled: bool) {
            if self.ia_clr_count < self.ia_clr_events.len() {
                self.ia_clr_events[self.ia_clr_count] = enabled;
                self.ia_clr_count += 1;
            }
        }

        fn unmask_flash_cache(&mut self) {
            self.unmasked = true;
        }
    }

    #[derive(Default)]
    struct RecordingCache {
        disabled_and_flushed: bool,
        mmu_inits: [u32; 4],
        mmu_init_count: usize,
        map_calls: usize,
        mapped_vaddr: [u32; 4],
        mapped_paddr: [u32; 4],
        mapped_pages: [u32; 4],
        flushed_cpus: [u32; 4],
        flush_count: usize,
        enabled_cpu: Option<u32>,
        uart_flushed: bool,
    }

    impl CacheControl for RecordingCache {
        fn disable_and_flush(&mut self) {
            self.disabled_and_flushed = true;
        }

        fn mmu_init(&mut self, cpu: u32) {
            if self.mmu_init_count < self.mmu_inits.len() {
                self.mmu_inits[self.mmu_init_count] = cpu;
                self.mmu_init_count += 1;
            }
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

        fn flush_cache(&mut self, cpu: u32) {
            if self.flush_count < self.flushed_cpus.len() {
                self.flushed_cpus[self.flush_count] = cpu;
                self.flush_count += 1;
            }
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
        segments[0] = Some(VerifiedSegment::new_for_test(
            0x100,
            0x4008_0000,
            64,
            SegmentKind::RamLoad,
        ));
        segments[1] = Some(VerifiedSegment::new_for_test(
            0x20020,
            0x3F40_0020,
            0x8000,
            SegmentKind::DromMap,
        ));

        let image = VerifiedImage::new_for_test(0x4008_0000, 2, segments, 0x30000);

        let mut cache = RecordingCache::default();
        let mut ram = TestRamSink::new();
        let mut dport = TestCacheMask::new();

        load_ram_and_map_mmu(&mut flash, &mut cache, &mut ram, &mut dport, &image)
            .expect("load_ram_and_map_mmu should succeed");

        // Verify RAM loading
        assert_eq!(ram.write_len, 64);
        assert_eq!(&ram.written_bytes[..64], &ram_payload[..]);

        // Verify Cache flush, MMU init parity, APP IA clear, and MMU map
        assert!(cache.disabled_and_flushed);
        assert_eq!(cache.mmu_init_count, 2);
        assert_eq!(&cache.mmu_inits[..2], &[0, 1]);
        assert_eq!(dport.ia_clr_count, 2);
        assert_eq!(&dport.ia_clr_events[..2], &[true, false]);
        assert_eq!(cache.map_calls, 2); // CPU 0 and CPU 1
        assert_eq!(cache.mapped_vaddr[0], 0x3F40_0000);
        assert_eq!(cache.mapped_paddr[0], 0x20000);
        assert_eq!(cache.mapped_pages[0], 1);
        assert_eq!(cache.flush_count, 1);
        assert_eq!(cache.flushed_cpus[0], 1);

        // Verify DPORT cache unmask and re-enable
        assert!(dport.unmasked);
        assert_eq!(cache.enabled_cpu, Some(0));
        assert!(cache.uart_flushed);
    }

    #[test]
    fn test_load_ram_prevalidates_before_mutating_state() {
        let mut flash = MockFlash::<2>::new();
        let ram_payload = [0x55u8; 64];
        flash.write_raw(0x100, &ram_payload).expect("write_raw");

        // Valid RAM segment paired with invalid MMU segment (parity mismatch)
        let mut segments: [Option<VerifiedSegment>; MAX_SEGMENTS] = [None; MAX_SEGMENTS];
        segments[0] = Some(VerifiedSegment::new_for_test(
            0x100,
            0x4008_0000,
            64,
            SegmentKind::RamLoad,
        ));
        segments[1] = Some(VerifiedSegment::new_for_test(
            0x20020,
            0x3F40_0040,
            0x8000,
            SegmentKind::DromMap,
        ));
        let image = VerifiedImage::new_for_test(0x4008_0000, 2, segments, 0x30000);

        let mut cache = RecordingCache::default();
        let mut ram = TestRamSink::new();
        let mut dport = TestCacheMask::new();

        let err = load_ram_and_map_mmu(&mut flash, &mut cache, &mut ram, &mut dport, &image)
            .expect_err("must reject invalid MMU plan before mutating RAM");
        assert_eq!(err, BootJumpError::MmuMappingInvalid);
        assert_eq!(ram.write_len, 0);
        assert!(!cache.disabled_and_flushed);
        assert!(!dport.unmasked);
    }

    #[test]
    fn test_load_ram_rejects_zero_entry_addr() {
        let mut flash = MockFlash::<2>::new();
        let mut segments: [Option<VerifiedSegment>; MAX_SEGMENTS] = [None; MAX_SEGMENTS];
        segments[0] = Some(VerifiedSegment::new_for_test(
            0x100,
            0x4008_0000,
            64,
            SegmentKind::RamLoad,
        ));
        let image = VerifiedImage::new_for_test(0, 1, segments, 0x200);

        let mut cache = RecordingCache::default();
        let mut ram = TestRamSink::new();
        let mut dport = TestCacheMask::new();

        let err = load_ram_and_map_mmu(&mut flash, &mut cache, &mut ram, &mut dport, &image)
            .expect_err("must reject zero entry address");
        assert_eq!(err, BootJumpError::InvalidEntryAddress);
        assert_eq!(ram.write_len, 0);
        assert!(!cache.disabled_and_flushed);
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
        segments[0] = Some(VerifiedSegment::new_for_test(
            0x100,
            0x4008_0000,
            72,
            SegmentKind::RamLoad,
        ));

        let image = VerifiedImage::new_for_test(0x4008_0000, 1, segments, 0x30000);

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
