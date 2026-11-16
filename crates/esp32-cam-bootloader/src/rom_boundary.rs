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

/// Cache disable, flush, MMU mapping, and UART synchronization contract.
pub trait CacheControl {
    /// Disables and flushes cache across PRO and APP CPUs.
    fn disable_and_flush(&mut self);

    /// Maps virtual address to physical flash pages in 64 KiB increments.
    fn mmu_map_pages(&mut self, cpu: u32, vaddr: u32, paddr: u32, page_count: u32) -> bool;

    /// Re-enables cache for the specified CPU.
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

    fn mmu_map_pages(&mut self, _cpu: u32, _vaddr: u32, _paddr: u32, _page_count: u32) -> bool {
        true
    }

    fn enable_cache(&mut self, _cpu: u32) {}

    fn flush_uart0(&mut self) {}

    fn delay_us(&mut self, _us: u32) {}
}

#[cfg(target_arch = "xtensa")]
use crate::clock_spi_psram::RomSpiflashChip;

#[cfg(target_arch = "xtensa")]
// JUSTIFICATION: Mask ROM hardware functions for ESP32 Xtensa target
// require FFI linkage.
// SAFETY: Hardware functions match ESP32 ROM symbol addresses in linkall.x.
extern "C" {
    static mut g_rom_flashchip: RomSpiflashChip;
    fn esp_rom_spiflash_wait_idle(spi: *mut RomSpiflashChip) -> i32;
    fn esp_rom_spiflash_read_user_cmd(status: *mut u32, cmd: u8) -> i32;

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
    fn uart_tx_flush(uart_no: u8);
    fn ets_delay_us(us: u32);

    fn esp_rom_spiflash_read(src_addr: u32, dest: *mut u32, len: u32) -> i32;
    fn esp_rom_spiflash_write(dest_addr: u32, src: *const u32, len: u32) -> i32;
    fn esp_rom_spiflash_erase_sector(sector_num: u32) -> i32;
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
    for _ in 0..10_000_000 {
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
    }
    Err(err)
}

/// Xtensa Mask ROM SPI flash driver.
#[cfg(target_arch = "xtensa")]
#[derive(Debug, Default, Clone, Copy)]
pub struct RomFlash;

#[cfg(target_arch = "xtensa")]
impl RomFlash {
    /// Creates a new [`RomFlash`] interface.
    #[must_use]
    pub const fn new() -> Self {
        Self
    }
}

#[cfg(target_arch = "xtensa")]
impl FlashIo for RomFlash {
    fn read(&mut self, mut offset: u32, mut buf: &mut [u8]) -> Result<(), FlashError> {
        wait_flash_wip_clear(FlashError::ReadFailed)?;
        let mut bounce = AlignedBounce64::new();
        while !buf.is_empty() {
            let aligned_addr = offset & !3;
            let misalign = (offset & 3) as usize;
            let max_payload = 64 - misalign;
            let chunk_len = buf.len().min(max_payload);
            let flash_read_len = ((misalign + chunk_len + 3) & !3) as u32;

            // JUSTIFICATION: Mask ROM SPI flash read requires a 4-byte aligned
            // destination buffer.
            // SAFETY: `bounce.bytes` is 4-byte aligned by `#[repr(C, align(4))]`,
            // and `flash_read_len <= 64`.
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
            offset = offset
                .checked_add(chunk_len as u32)
                .ok_or(FlashError::OutOfBounds)?;
            buf = &mut buf[chunk_len..];
        }
        Ok(())
    }

    #[allow(clippy::manual_is_multiple_of)]
    fn write(&mut self, mut offset: u32, mut data: &[u8]) -> Result<(), FlashError> {
        if offset % 4 != 0 || data.len() % 4 != 0 {
            return Err(FlashError::UnalignedLength);
        }

        let mut bounce = AlignedBounce64::new();
        while !data.is_empty() {
            let chunk_len = next_program_chunk_len(offset, data.len());
            bounce.bytes[..chunk_len].copy_from_slice(&data[..chunk_len]);

            wait_flash_wip_clear(FlashError::WriteFailed)?;

            // JUSTIFICATION: Mask ROM SPI flash write requires 4-byte aligned
            // source buffer and 4-byte aligned length.
            // SAFETY: `bounce.bytes` is 4-byte aligned by `#[repr(C, align(4))]`,
            // and `chunk_len % 4 == 0` because chunks are multiples of 4.
            let res = unsafe {
                esp_rom_spiflash_write(
                    offset,
                    bounce.bytes.as_ptr() as *const u32,
                    chunk_len as u32,
                )
            };
            if res != 0 {
                return Err(FlashError::WriteFailed);
            }

            wait_flash_wip_clear(FlashError::WriteFailed)?;

            offset = offset
                .checked_add(chunk_len as u32)
                .ok_or(FlashError::OutOfBounds)?;
            data = &data[chunk_len..];
        }
        Ok(())
    }

    fn erase_sector(&mut self, sector: u32) -> Result<(), FlashError> {
        wait_flash_wip_clear(FlashError::EraseFailed)?;
        // JUSTIFICATION: Calling Mask ROM SPI flash erase function for
        // sector erasure.
        // SAFETY: `sector` is a 32-bit sector index accepted by ROM function.
        let res = unsafe { esp_rom_spiflash_erase_sector(sector) };
        if res != 0 {
            return Err(FlashError::EraseFailed);
        }
        wait_flash_wip_clear(FlashError::EraseFailed)?;
        Ok(())
    }
}

/// Xtensa Mask ROM cache and MMU controller.
#[cfg(target_arch = "xtensa")]
#[derive(Debug, Default, Clone, Copy)]
pub struct RomCache;

#[cfg(target_arch = "xtensa")]
impl RomCache {
    /// Creates a new [`RomCache`] interface.
    #[must_use]
    pub const fn new() -> Self {
        Self
    }
}

#[cfg(target_arch = "xtensa")]
impl CacheControl for RomCache {
    fn disable_and_flush(&mut self) {
        // JUSTIFICATION: ESP32 dual-core cache disable and flush required
        // before MMU re-mapping.
        // SAFETY: Calling Cache_Read_Disable_rom and Cache_Flush_rom for
        // CPU 0 (PRO) and CPU 1 (APP).
        unsafe {
            Cache_Read_Disable_rom(0);
            Cache_Read_Disable_rom(1);
            Cache_Flush_rom(0);
            Cache_Flush_rom(1);
        }
    }

    fn mmu_map_pages(&mut self, cpu: u32, vaddr: u32, paddr: u32, page_count: u32) -> bool {
        // JUSTIFICATION: ESP32 Mask ROM MMU configuration to map virtual
        // address to physical flash address.
        // SAFETY: Calling cache_flash_mmu_set_rom with valid CPU ID (0 or 1),
        // PID 0, and 64KB page size.
        let ret = unsafe { cache_flash_mmu_set_rom(cpu, 0, vaddr, paddr, 64, page_count) };
        ret == 0
    }

    fn enable_cache(&mut self, cpu: u32) {
        // JUSTIFICATION: ESP32 Mask ROM cache enable after MMU re-mapping.
        // SAFETY: Calling Cache_Flush_rom and Cache_Read_Enable_rom on
        // the requested CPU.
        unsafe {
            Cache_Flush_rom(cpu);
            Cache_Read_Enable_rom(cpu);
        }
    }

    fn flush_uart0(&mut self) {
        // JUSTIFICATION: ESP32 Mask ROM UART flush to drain output before
        // boot transitions.
        // SAFETY: Calling uart_tx_flush with valid UART ID 0.
        unsafe {
            uart_tx_flush(0);
        }
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
        assert!(cache.mmu_map_pages(0, 0x3F40_0000, 0x20000, 1));
        cache.enable_cache(0);
        cache.flush_uart0();
        cache.delay_us(10);
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
