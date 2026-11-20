//! Mask ROM flash and cache boundary abstraction.
//!
//! Provides hardware-independent interfaces ([`FlashIo`], [`CacheControl`])
//! along with `#![no_std]` [`MockFlash`] for unit/Miri testing and
//! Xtensa Mask ROM implementations ([`RomFlash`], [`RomCache`]).

// JUSTIFICATION: Hardware Mask ROM bindings for ESP32 Xtensa target require
// unsafe FFI.

/// Size of a standard SPI flash sector in bytes (4 KiB).
pub const FLASH_SECTOR_SIZE: usize = 4096;

/// Size of a standard SPI NOR flash page in bytes (256 B).
pub const FLASH_PAGE_SIZE: usize = 256;

/// Maximum hardware `SPI_FLASH_PP` FIFO payload per transfer (32 B).
pub const FLASH_PP_MAX_CHUNK: usize = 32;

/// Computes the next NOR page-program sub-chunk length (`<= 32` bytes)
/// without crossing a 256-byte flash page boundary.
#[inline]
#[must_use]
pub const fn next_program_chunk_len(offset: u32, remaining: usize) -> usize {
    let page_rem = FLASH_PAGE_SIZE - ((offset as usize) & (FLASH_PAGE_SIZE - 1));
    let max_chunk = if page_rem < FLASH_PP_MAX_CHUNK {
        page_rem
    } else {
        FLASH_PP_MAX_CHUNK
    };
    if remaining < max_chunk {
        remaining
    } else {
        max_chunk
    }
}

/// Flash operational errors.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlashError {
    /// Read operation failed in hardware.
    ReadFailed,
    /// Write operation failed in hardware.
    WriteFailed,
    /// Sector erase operation failed in hardware.
    EraseFailed,
    /// Requested length or offset violates 4-byte alignment requirement.
    UnalignedLength,
    /// Requested address range exceeds flash capacity.
    OutOfBounds,
}

/// 4-byte aligned sector buffer holding a full 4096-byte sector.
#[repr(C, align(4))]
#[derive(Clone, Copy)]
pub struct AlignedSectorBuf {
    /// Raw sector payload bytes.
    pub bytes: [u8; FLASH_SECTOR_SIZE],
}

impl AlignedSectorBuf {
    /// Creates a zero-initialized sector buffer aligned to 4 bytes.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            bytes: [0u8; FLASH_SECTOR_SIZE],
        }
    }
}

impl Default for AlignedSectorBuf {
    fn default() -> Self {
        Self::new()
    }
}

/// 4-byte aligned 64-byte bounce buffer for safe DMA/ROM SPI transfers.
#[repr(C, align(4))]
#[derive(Clone, Copy)]
pub struct AlignedBounce64 {
    /// Raw bounce buffer payload bytes.
    pub bytes: [u8; 64],
}

impl AlignedBounce64 {
    /// Creates a zero-initialized 64-byte bounce buffer aligned to 4 bytes.
    #[must_use]
    pub const fn new() -> Self {
        Self { bytes: [0u8; 64] }
    }
}

impl Default for AlignedBounce64 {
    fn default() -> Self {
        Self::new()
    }
}

/// SPI flash read/write/erase contract.
pub trait FlashIo {
    /// Reads bytes from `offset` into `buf`.
    fn read(&mut self, offset: u32, buf: &mut [u8]) -> Result<(), FlashError>;

    /// Writes `data` starting at `offset`.
    fn write(&mut self, offset: u32, data: &[u8]) -> Result<(), FlashError>;

    /// Erases the specified sector index to `0xFF`.
    fn erase_sector(&mut self, sector: u32) -> Result<(), FlashError>;
}

/// Minimum flash byte offset permitted for bootloader write/erase operations.
///
/// Protects sectors `0..9`: the 2nd-stage bootloader (`0x1000..0x8000`) and
/// the partition table (`0x8000..0x9000`).
pub const MIN_WRITABLE_FLASH_OFFSET: u32 = 0x9000;

/// Minimum 4 KiB flash sector index permitted for bootloader erase operations.
pub const MIN_WRITABLE_FLASH_SECTOR: u32 = MIN_WRITABLE_FLASH_OFFSET / (FLASH_SECTOR_SIZE as u32);

/// ESP32 CPU core identifier accepted by Mask ROM cache and MMU routines.
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CpuId {
    /// PRO CPU (core 0).
    Pro = 0,
    /// APP CPU (core 1).
    App = 1,
}

impl CpuId {
    /// Converts a raw CPU index into a validated [`CpuId`].
    #[inline]
    #[must_use]
    pub const fn try_from_u32(cpu: u32) -> Option<Self> {
        match cpu {
            0 => Some(Self::Pro),
            1 => Some(Self::App),
            _ => None,
        }
    }

    /// Returns the raw hardware CPU index (`0` or `1`).
    #[inline]
    #[must_use]
    pub const fn as_u32(self) -> u32 {
        self as u32
    }
}

/// Returns `true` if `cpu` is a valid ESP32 CPU index (`0` or `1`).
#[inline]
#[must_use]
pub const fn is_valid_cpu_id(cpu: u32) -> bool {
    CpuId::try_from_u32(cpu).is_some()
}

/// Validates flash read offset, length, and 4-byte aligned read ceiling
/// against `chip_size` before issuing SPI flash I/O.
#[inline]
pub const fn validate_rom_read_bounds(
    offset: u32,
    len: usize,
    chip_size: u32,
) -> Result<(), FlashError> {
    if chip_size == 0 || len > u32::MAX as usize {
        return Err(FlashError::OutOfBounds);
    }
    let len_u32 = len as u32;
    let Some(end) = offset.checked_add(len_u32) else {
        return Err(FlashError::OutOfBounds);
    };
    if end > chip_size {
        return Err(FlashError::OutOfBounds);
    }
    if len_u32 > 0 {
        let Some(aligned_end_raw) = end.checked_add(3) else {
            return Err(FlashError::OutOfBounds);
        };
        let aligned_end = aligned_end_raw & !3;
        if aligned_end > chip_size {
            return Err(FlashError::OutOfBounds);
        }
    }
    Ok(())
}

/// Validates flash write alignment, protected bootloader/partition-table
/// floor, overflow safety, and `chip_size` ceiling before SPI flash I/O.
#[inline]
pub const fn validate_rom_write_bounds(
    offset: u32,
    len: usize,
    chip_size: u32,
) -> Result<(), FlashError> {
    if (offset & 3) != 0 || (len & 3) != 0 {
        return Err(FlashError::UnalignedLength);
    }
    if chip_size == 0 || len > u32::MAX as usize {
        return Err(FlashError::OutOfBounds);
    }
    if offset < MIN_WRITABLE_FLASH_OFFSET {
        return Err(FlashError::OutOfBounds);
    }
    let len_u32 = len as u32;
    let Some(end) = offset.checked_add(len_u32) else {
        return Err(FlashError::OutOfBounds);
    };
    if end > chip_size {
        return Err(FlashError::OutOfBounds);
    }
    Ok(())
}

/// Validates flash sector index against protected bootloader/partition-table
/// sectors and `chip_size` before issuing SPI flash sector erase.
#[inline]
pub const fn validate_rom_erase_sector(sector: u32, chip_size: u32) -> Result<(), FlashError> {
    if sector < MIN_WRITABLE_FLASH_SECTOR {
        return Err(FlashError::OutOfBounds);
    }
    let sector_count = chip_size / (FLASH_SECTOR_SIZE as u32);
    if sector >= sector_count {
        return Err(FlashError::OutOfBounds);
    }
    Ok(())
}

/// Cache disable, flush, MMU mapping, and UART synchronization contract.
pub trait CacheControl {
    /// Disables and flushes cache across PRO and APP CPUs.
    fn disable_and_flush(&mut self);

    /// Initializes the Flash MMU table for the specified CPU (`0` or `1`).
    fn mmu_init(&mut self, cpu: u32);

    /// Maps virtual address to physical flash pages in 64 KiB increments.
    fn mmu_map_pages(&mut self, cpu: u32, vaddr: u32, paddr: u32, page_count: u32) -> bool;

    /// Flushes cache lines for the specified CPU (`0` or `1`).
    fn flush_cache(&mut self, cpu: u32);

    /// Re-enables cache for the specified CPU (`0` or `1`).
    fn enable_cache(&mut self, cpu: u32);

    /// Drains any pending UART0 transmission FIFO.
    fn flush_uart0(&mut self);

    /// Delays execution by busy-waiting for `us` microseconds.
    fn delay_us(&mut self, us: u32);
}

/// Pure `#![no_std]` in-memory mock flash for unit tests and Miri verification.
pub struct MockFlash<const SLOTS: usize> {
    sectors: [[u8; FLASH_SECTOR_SIZE]; SLOTS],
}

impl<const SLOTS: usize> MockFlash<SLOTS> {
    /// Creates an erased [`MockFlash`] with all sectors initialized to `0xFF`.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            sectors: [[0xFFu8; FLASH_SECTOR_SIZE]; SLOTS],
        }
    }

    /// Directly overwrites flash contents without NOR-AND bit clearing.
    ///
    /// Intended for setting up test fixtures.
    pub fn write_raw(&mut self, offset: u32, data: &[u8]) -> Result<(), FlashError> {
        let total_bytes = SLOTS.saturating_mul(FLASH_SECTOR_SIZE);
        let start = offset as usize;
        let end = start
            .checked_add(data.len())
            .ok_or(FlashError::OutOfBounds)?;
        if end > total_bytes {
            return Err(FlashError::OutOfBounds);
        }
        for (i, &byte) in data.iter().enumerate() {
            let curr = start + i;
            let sector = curr / FLASH_SECTOR_SIZE;
            let sec_offset = curr % FLASH_SECTOR_SIZE;
            self.sectors[sector][sec_offset] = byte;
        }
        Ok(())
    }
}

impl<const SLOTS: usize> Default for MockFlash<SLOTS> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const SLOTS: usize> FlashIo for MockFlash<SLOTS> {
    fn read(&mut self, offset: u32, buf: &mut [u8]) -> Result<(), FlashError> {
        let total_bytes = SLOTS.saturating_mul(FLASH_SECTOR_SIZE);
        let start = offset as usize;
        let end = start
            .checked_add(buf.len())
            .ok_or(FlashError::OutOfBounds)?;
        if end > total_bytes {
            return Err(FlashError::OutOfBounds);
        }
        for (i, byte) in buf.iter_mut().enumerate() {
            let curr = start + i;
            let sector = curr / FLASH_SECTOR_SIZE;
            let sec_offset = curr % FLASH_SECTOR_SIZE;
            *byte = self.sectors[sector][sec_offset];
        }
        Ok(())
    }

    fn write(&mut self, offset: u32, data: &[u8]) -> Result<(), FlashError> {
        let total_bytes = SLOTS.saturating_mul(FLASH_SECTOR_SIZE);
        let start = offset as usize;
        let end = start
            .checked_add(data.len())
            .ok_or(FlashError::OutOfBounds)?;
        if end > total_bytes {
            return Err(FlashError::OutOfBounds);
        }
        for (i, &byte) in data.iter().enumerate() {
            let curr = start + i;
            let sector = curr / FLASH_SECTOR_SIZE;
            let sec_offset = curr % FLASH_SECTOR_SIZE;
            self.sectors[sector][sec_offset] &= byte;
        }
        Ok(())
    }

    fn erase_sector(&mut self, sector: u32) -> Result<(), FlashError> {
        let sec_idx = sector as usize;
        if sec_idx >= SLOTS {
            return Err(FlashError::OutOfBounds);
        }
        self.sectors[sec_idx].fill(0xFF);
        Ok(())
    }
}

/// In-memory mock cache controller for unit tests and simulation.
#[derive(Debug, Default, Clone, Copy)]
pub struct MockCache;

impl MockCache {
    /// Creates a new [`MockCache`].
    #[must_use]
    pub const fn new() -> Self {
        Self
    }
}

impl CacheControl for MockCache {
    fn disable_and_flush(&mut self) {}

    fn mmu_init(&mut self, _cpu: u32) {}

    fn mmu_map_pages(&mut self, cpu: u32, _vaddr: u32, _paddr: u32, _page_count: u32) -> bool {
        is_valid_cpu_id(cpu)
    }

    fn flush_cache(&mut self, _cpu: u32) {}

    fn enable_cache(&mut self, _cpu: u32) {}

    fn flush_uart0(&mut self) {}

    fn delay_us(&mut self, _us: u32) {}
}

#[cfg(target_arch = "xtensa")]
use core::sync::atomic::AtomicBool;
#[cfg(target_arch = "xtensa")]
use core::sync::atomic::Ordering;

#[cfg(target_arch = "xtensa")]
use crate::clock_spi_psram::RomSpiflashChip;

#[cfg(target_arch = "xtensa")]
static ROM_FLASH_TAKEN: AtomicBool = AtomicBool::new(false);

#[cfg(target_arch = "xtensa")]
static ROM_CACHE_TAKEN: AtomicBool = AtomicBool::new(false);

/// Maximum poll iterations when waiting for SPI NOR WIP bit to clear.
///
/// With a 10 us delay per poll, 50,000 iterations caps wait time at ~500 ms.
#[cfg(target_arch = "xtensa")]
const FLASH_WIP_MAX_POLLS: u32 = 50_000;

/// Microsecond delay between RDSR (0x05) WIP polls.
#[cfg(target_arch = "xtensa")]
const FLASH_WIP_POLL_DELAY_US: u32 = 10;

#[cfg(target_arch = "xtensa")]
// JUSTIFICATION: Mask ROM hardware functions for ESP32 Xtensa target
// require FFI linkage.
// SAFETY: Hardware functions match ESP32 ROM symbol addresses in linkall.x
// and ESP-IDF `esp32/rom/{spi_flash,cache,uart,ets_sys}.h` C signatures.
extern "C" {
    static mut g_rom_flashchip: RomSpiflashChip;
    fn esp_rom_spiflash_wait_idle(spi: *mut RomSpiflashChip) -> i32;
    fn esp_rom_spiflash_read_user_cmd(status: *mut u32, cmd: u8) -> i32;

    fn mmu_init(cpu: u32);
    fn Cache_Read_Disable_rom(cpu: u32);
    fn Cache_Flush_rom(cpu: u32);
    fn cache_flash_mmu_set_rom(
        cpu: u32,
        pid: u32,
        vaddr: u32,
        paddr: u32,
        psize: u32,
        page_count: u32,
    ) -> u32;
    fn Cache_Read_Enable_rom(cpu: u32);
    fn uart_tx_flush(uart_no: u8) -> i32;
    fn ets_delay_us(us: u32);

    fn esp_rom_spiflash_read(src_addr: u32, dest: *mut u32, len: i32) -> i32;
    fn esp_rom_spiflash_write(dest_addr: u32, src: *const u32, len: i32) -> i32;
    fn esp_rom_spiflash_erase_sector(sector_num: u32) -> i32;
}

#[cfg(target_arch = "xtensa")]
fn rom_flash_chip_size() -> u32 {
    // JUSTIFICATION: Reading flash capacity from ESP32 ROM chip descriptor.
    // SAFETY: `g_rom_flashchip` is initialized by Mask ROM and flash attach.
    unsafe { core::ptr::read_volatile(core::ptr::addr_of!(g_rom_flashchip.chip_size)) }
}

#[cfg(target_arch = "xtensa")]
fn wait_flash_wip_clear(err: FlashError) -> Result<(), FlashError> {
    // JUSTIFICATION: Mask ROM SPI flash wait idle checks hardware controller
    // status via global ROM chip descriptor.
    // SAFETY: `g_rom_flashchip` is defined in ROM and initialized during boot.
    let idle_res = unsafe { esp_rom_spiflash_wait_idle(core::ptr::addr_of_mut!(g_rom_flashchip)) };
    if idle_res != 0 {
        return Err(err);
    }

    let mut status = 0x01_u32;
    for _ in 0..FLASH_WIP_MAX_POLLS {
        // JUSTIFICATION: Mask ROM read user command queries RDSR (0x05) to
        // check WIP bit.
        // SAFETY: Pointer to local stack variable `status` is valid for
        // 32-bit read.
        let cmd_res = unsafe { esp_rom_spiflash_read_user_cmd(&mut status, 0x05) };
        if cmd_res != 0 {
            return Err(err);
        }
        if (status & 0x01) == 0 {
            return Ok(());
        }
        // JUSTIFICATION: Bounded delay prevents tight-loop WDT starvation.
        // SAFETY: Mask ROM `ets_delay_us` accepts any `u32` microsecond count.
        unsafe {
            ets_delay_us(FLASH_WIP_POLL_DELAY_US);
        }
    }
    Err(err)
}

/// Xtensa Mask ROM SPI flash singleton driver.
#[cfg(target_arch = "xtensa")]
#[derive(Debug)]
pub struct RomFlash {
    _private: (),
}

#[cfg(target_arch = "xtensa")]
impl RomFlash {
    /// Claims exclusive ownership of the Mask ROM SPI flash singleton.
    ///
    /// Returns `None` if the singleton has already been claimed.
    #[must_use]
    pub fn take() -> Option<Self> {
        if ROM_FLASH_TAKEN
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
impl FlashIo for RomFlash {
    fn read(&mut self, mut offset: u32, mut buf: &mut [u8]) -> Result<(), FlashError> {
        let chip_size = rom_flash_chip_size();
        validate_rom_read_bounds(offset, buf.len(), chip_size)?;
        if buf.is_empty() {
            return Ok(());
        }
        wait_flash_wip_clear(FlashError::ReadFailed)?;
        let mut bounce = AlignedBounce64::new();
        while !buf.is_empty() {
            let aligned_addr = offset & !3;
            let misalign = (offset & 3) as usize;
            let max_payload = 64 - misalign;
            let chunk_len = buf.len().min(max_payload);
            let flash_read_len_usize = (misalign + chunk_len + 3) & !3;
            let flash_read_len =
                i32::try_from(flash_read_len_usize).map_err(|_| FlashError::OutOfBounds)?;

            // JUSTIFICATION: Mask ROM SPI flash read requires a 4-byte aligned
            // destination buffer.
            // SAFETY: `bounce.bytes` is 4-byte aligned by `#[repr(C, align(4))]`,
            // `flash_read_len <= 64`, and span was validated against `chip_size`.
            let res = unsafe {
                esp_rom_spiflash_read(
                    aligned_addr,
                    bounce.bytes.as_mut_ptr() as *mut u32,
                    flash_read_len,
                )
            };
            if res != 0 {
                return Err(FlashError::ReadFailed);
            }

            buf[..chunk_len].copy_from_slice(&bounce.bytes[misalign..misalign + chunk_len]);
            buf = &mut buf[chunk_len..];
            if !buf.is_empty() {
                offset = offset
                    .checked_add(chunk_len as u32)
                    .ok_or(FlashError::OutOfBounds)?;
            }
        }
        Ok(())
    }

    fn write(&mut self, mut offset: u32, mut data: &[u8]) -> Result<(), FlashError> {
        let chip_size = rom_flash_chip_size();
        validate_rom_write_bounds(offset, data.len(), chip_size)?;
        if data.is_empty() {
            return Ok(());
        }

        let mut bounce = AlignedBounce64::new();
        while !data.is_empty() {
            let chunk_len = next_program_chunk_len(offset, data.len());
            let chunk_len_i32 = i32::try_from(chunk_len).map_err(|_| FlashError::OutOfBounds)?;
            bounce.bytes[..chunk_len].copy_from_slice(&data[..chunk_len]);

            wait_flash_wip_clear(FlashError::WriteFailed)?;

            // JUSTIFICATION: Mask ROM SPI flash write requires 4-byte aligned
            // source buffer and 4-byte aligned length.
            // SAFETY: `bounce.bytes` is 4-byte aligned by `#[repr(C, align(4))]`,
            // `chunk_len % 4 == 0`, and span was validated against `chip_size`.
            let res = unsafe {
                esp_rom_spiflash_write(offset, bounce.bytes.as_ptr() as *const u32, chunk_len_i32)
            };
            if res != 0 {
                return Err(FlashError::WriteFailed);
            }

            wait_flash_wip_clear(FlashError::WriteFailed)?;

            data = &data[chunk_len..];
            if !data.is_empty() {
                offset = offset
                    .checked_add(chunk_len as u32)
                    .ok_or(FlashError::OutOfBounds)?;
            }
        }
        Ok(())
    }

    fn erase_sector(&mut self, sector: u32) -> Result<(), FlashError> {
        let chip_size = rom_flash_chip_size();
        validate_rom_erase_sector(sector, chip_size)?;
        wait_flash_wip_clear(FlashError::EraseFailed)?;
        // JUSTIFICATION: Calling Mask ROM SPI flash erase function for
        // sector erasure.
        // SAFETY: `sector` was validated against protected sectors and
        // `g_rom_flashchip.chip_size`.
        let res = unsafe { esp_rom_spiflash_erase_sector(sector) };
        if res != 0 {
            return Err(FlashError::EraseFailed);
        }
        wait_flash_wip_clear(FlashError::EraseFailed)?;
        Ok(())
    }
}

/// Xtensa Mask ROM cache and MMU singleton controller.
#[cfg(target_arch = "xtensa")]
#[derive(Debug)]
pub struct RomCache {
    _private: (),
}

#[cfg(target_arch = "xtensa")]
impl RomCache {
    /// Claims exclusive ownership of the Mask ROM cache singleton.
    ///
    /// Returns `None` if the singleton has already been claimed.
    #[must_use]
    pub fn take() -> Option<Self> {
        if ROM_CACHE_TAKEN
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
impl CacheControl for RomCache {
    fn disable_and_flush(&mut self) {
        // JUSTIFICATION: ESP32 dual-core cache disable and flush required
        // before SPI flash reads and MMU re-mapping.
        // SAFETY: Calling Cache_Read_Disable_rom and Cache_Flush_rom for
        // validated CPU IDs 0 (PRO) and 1 (APP).
        unsafe {
            Cache_Read_Disable_rom(CpuId::Pro.as_u32());
            Cache_Read_Disable_rom(CpuId::App.as_u32());
            Cache_Flush_rom(CpuId::Pro.as_u32());
            Cache_Flush_rom(CpuId::App.as_u32());
        }
    }

    fn mmu_init(&mut self, cpu: u32) {
        let Some(cpu_id) = CpuId::try_from_u32(cpu) else {
            return;
        };
        // JUSTIFICATION: ESP32 Mask ROM MMU table reset for the target CPU.
        // SAFETY: `cpu_id` is validated to be 0 (PRO) or 1 (APP).
        unsafe {
            mmu_init(cpu_id.as_u32());
        }
    }

    fn mmu_map_pages(&mut self, cpu: u32, vaddr: u32, paddr: u32, page_count: u32) -> bool {
        let Some(cpu_id) = CpuId::try_from_u32(cpu) else {
            return false;
        };
        let cpu_raw = cpu_id.as_u32();
        // JUSTIFICATION: ESP32 Mask ROM MMU configuration to map virtual
        // address to physical flash address.
        // SAFETY: `cpu_raw` is validated (0 or 1), PID is 0, page size 64 KiB.
        let ret = unsafe { cache_flash_mmu_set_rom(cpu_raw, 0, vaddr, paddr, 64, page_count) };
        ret == 0
    }

    fn flush_cache(&mut self, cpu: u32) {
        let Some(cpu_id) = CpuId::try_from_u32(cpu) else {
            return;
        };
        // JUSTIFICATION: ESP32 Mask ROM cache flush after MMU update.
        // SAFETY: `cpu_id` is validated to be 0 (PRO) or 1 (APP).
        unsafe {
            Cache_Flush_rom(cpu_id.as_u32());
        }
    }

    fn enable_cache(&mut self, cpu: u32) {
        let Some(cpu_id) = CpuId::try_from_u32(cpu) else {
            return;
        };
        // JUSTIFICATION: ESP32 Mask ROM cache enable after MMU re-mapping.
        // SAFETY: `cpu_id` is validated to be 0 (PRO) or 1 (APP).
        unsafe {
            Cache_Flush_rom(cpu_id.as_u32());
            Cache_Read_Enable_rom(cpu_id.as_u32());
        }
    }

    fn flush_uart0(&mut self) {
        // JUSTIFICATION: ESP32 Mask ROM UART flush to drain output before
        // boot transitions.
        // SAFETY: Calling uart_tx_flush with valid UART ID 0.
        let _status: i32 = unsafe { uart_tx_flush(0) };
    }

    fn delay_us(&mut self, us: u32) {
        // JUSTIFICATION: Mask ROM microsecond busy wait for hardware settling.
        // SAFETY: Calling ets_delay_us with duration in microseconds.
        unsafe {
            ets_delay_us(us);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_aligned_buffer_alignments() {
        assert_eq!(core::mem::align_of::<AlignedSectorBuf>(), 4);
        assert_eq!(core::mem::align_of::<AlignedBounce64>(), 4);
        let sec = AlignedSectorBuf::default();
        let bounce = AlignedBounce64::default();
        assert_eq!(sec.bytes.len(), FLASH_SECTOR_SIZE);
        assert_eq!(bounce.bytes.len(), 64);
    }

    #[test]
    fn test_mock_flash_erase_and_raw_write() {
        let mut flash = MockFlash::<4>::new();
        let mut buf = [0u8; 8];

        flash.read(0, &mut buf).expect("read initial sector");
        assert_eq!(buf, [0xFF; 8]);

        flash
            .write_raw(0, &[0x12, 0x34, 0x56, 0x78])
            .expect("write raw");
        flash.read(0, &mut buf[..4]).expect("read back");
        assert_eq!(&buf[..4], &[0x12, 0x34, 0x56, 0x78]);
    }

    #[test]
    fn test_mock_flash_nor_clearing() {
        let mut flash = MockFlash::<4>::new();
        // Erased state is 0xFF.
        // Write 0b1100_0011 (0xC3) -> bits become 0xC3.
        flash.write(0, &[0xC3]).expect("first write");
        let mut read_byte = [0u8; 1];
        flash.read(0, &mut read_byte).expect("read");
        assert_eq!(read_byte[0], 0xC3);

        // Writing 0b1010_1010 (0xAA) should AND with 0b1100_0011 -> 0b1000_0010 (0x82).
        flash.write(0, &[0xAA]).expect("second write");
        flash.read(0, &mut read_byte).expect("read");
        assert_eq!(read_byte[0], 0xC3 & 0xAA);
    }

    #[test]
    fn test_mock_flash_erase_sector() {
        let mut flash = MockFlash::<4>::new();
        flash
            .write_raw(0, &[0x00, 0x00, 0x00, 0x00])
            .expect("write raw");
        let mut read_bytes = [0u8; 4];
        flash.read(0, &mut read_bytes).expect("read");
        assert_eq!(read_bytes, [0x00; 4]);

        flash.erase_sector(0).expect("erase sector 0");
        flash.read(0, &mut read_bytes).expect("read after erase");
        assert_eq!(read_bytes, [0xFF; 4]);
    }

    #[test]
    fn test_mock_flash_bounds() {
        let mut flash = MockFlash::<2>::new();
        let mut buf = [0u8; 1];

        // Total capacity is 2 * 4096 = 8192 bytes.
        let out_of_bounds_offset = 8192;
        assert_eq!(
            flash.read(out_of_bounds_offset, &mut buf),
            Err(FlashError::OutOfBounds)
        );
        assert_eq!(
            flash.write(out_of_bounds_offset, &[0xAA]),
            Err(FlashError::OutOfBounds)
        );
        assert_eq!(flash.erase_sector(2), Err(FlashError::OutOfBounds));
    }

    #[test]
    fn test_mock_cache() {
        let mut cache = MockCache::new();
        cache.disable_and_flush();
        cache.mmu_init(0);
        cache.mmu_init(1);
        assert!(cache.mmu_map_pages(0, 0x3F40_0000, 0x20000, 1));
        assert!(cache.mmu_map_pages(1, 0x3F40_0000, 0x20000, 1));
        assert!(!cache.mmu_map_pages(2, 0x3F40_0000, 0x20000, 1));
        cache.flush_cache(1);
        cache.enable_cache(0);
        cache.flush_uart0();
        cache.delay_us(10);
    }

    #[test]
    fn test_cpu_id_validation() {
        assert_eq!(CpuId::try_from_u32(0), Some(CpuId::Pro));
        assert_eq!(CpuId::try_from_u32(1), Some(CpuId::App));
        assert_eq!(CpuId::try_from_u32(2), None);
        assert_eq!(CpuId::try_from_u32(u32::MAX), None);
        assert!(is_valid_cpu_id(0));
        assert!(is_valid_cpu_id(1));
        assert!(!is_valid_cpu_id(2));
    }

    #[test]
    fn test_rom_flash_bounds_validators() {
        const CHIP_4M: u32 = 4 * 1024 * 1024;

        // Read validation: partition table at 0x8000 allowed, overflow/OOB rejected.
        assert_eq!(validate_rom_read_bounds(0x8000, 32, CHIP_4M), Ok(()));
        assert_eq!(validate_rom_read_bounds(CHIP_4M - 4, 4, CHIP_4M), Ok(()));
        assert_eq!(
            validate_rom_read_bounds(CHIP_4M - 2, 4, CHIP_4M),
            Err(FlashError::OutOfBounds)
        );
        assert_eq!(
            validate_rom_read_bounds(u32::MAX - 2, 4, CHIP_4M),
            Err(FlashError::OutOfBounds)
        );
        assert_eq!(
            validate_rom_read_bounds(0x8000, 32, 0),
            Err(FlashError::OutOfBounds)
        );

        // Write validation: protects < 0x9000, alignment, overflow, chip_size.
        assert_eq!(validate_rom_write_bounds(0x9000, 32, CHIP_4M), Ok(()));
        assert_eq!(
            validate_rom_write_bounds(0x1000, 32, CHIP_4M),
            Err(FlashError::OutOfBounds)
        );
        assert_eq!(
            validate_rom_write_bounds(0x8000, 32, CHIP_4M),
            Err(FlashError::OutOfBounds)
        );
        assert_eq!(
            validate_rom_write_bounds(0x9002, 32, CHIP_4M),
            Err(FlashError::UnalignedLength)
        );
        assert_eq!(
            validate_rom_write_bounds(0x9000, 30, CHIP_4M),
            Err(FlashError::UnalignedLength)
        );
        assert_eq!(
            validate_rom_write_bounds(u32::MAX - 3, 8, CHIP_4M),
            Err(FlashError::OutOfBounds)
        );

        // Erase validation: protects sectors 0..9 and rejects >= chip_size/4K.
        assert_eq!(validate_rom_erase_sector(9, CHIP_4M), Ok(()));
        assert_eq!(validate_rom_erase_sector(1023, CHIP_4M), Ok(()));
        assert_eq!(
            validate_rom_erase_sector(0, CHIP_4M),
            Err(FlashError::OutOfBounds)
        );
        assert_eq!(
            validate_rom_erase_sector(8, CHIP_4M),
            Err(FlashError::OutOfBounds)
        );
        assert_eq!(
            validate_rom_erase_sector(1024, CHIP_4M),
            Err(FlashError::OutOfBounds)
        );
    }

    #[test]
    fn test_next_program_chunk_len_bounds_and_page_split() {
        assert_eq!(next_program_chunk_len(0x18000, 32), 32);
        assert_eq!(next_program_chunk_len(0x18000, 64), 32);
        assert_eq!(next_program_chunk_len(0x180F0, 32), 16);
        assert_eq!(next_program_chunk_len(0x18100, 16), 16);
        assert_eq!(next_program_chunk_len(0x18000, 0), 0);
    }
}
