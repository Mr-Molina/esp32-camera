//! Pure-Rust 2nd-stage bootloader for ESP32-WROVER.

#![no_std]

pub mod boot;
pub mod clock_spi_psram;
pub mod gpio16;
pub mod image_verify;
pub mod mmu_jump;
pub mod otadata;
pub mod partitions;
pub mod rom_boundary;
pub mod wdt;

#[cfg(target_arch = "xtensa")]
pub use boot::run_xtensa_boot;
pub use boot::{prepare_boot_image, BootError};
#[cfg(target_arch = "xtensa")]
pub use clock_spi_psram::{attach_flash_4mb_dio, XtensaClockMmio};
pub use clock_spi_psram::{configure_ticks, uart0_clkdiv_for_115200, ClockMmio};
pub use gpio16::{init_gpio16_psram_cs, release_psram_cs_hold, GpioMmio};
#[cfg(target_arch = "xtensa")]
pub use gpio16::{init_gpio16_xtensa, XtensaGpioMmio};
pub use image_verify::{
    verify_app_image, ImageVerifyError, SegmentKind, VerifiedImage, VerifiedSegment,
};
pub use mmu_jump::{
    compute_mmu_mapping, load_ram_and_map_mmu, BootJumpError, CacheMaskControl, MmuMappingPlan,
    RamSink,
};
#[cfg(target_arch = "xtensa")]
pub use mmu_jump::{jump_to_entry, XtensaDportMask, XtensaRamSink};
pub use otadata::{
    crc32_esp_ota, select_and_update_otadata, BootCandidate, OtaImgState, OtaSelectEntry,
    OtaSelectError, MAX_BOOT_CANDIDATES,
};
pub use partitions::{
    encode_md5_entry, md5_digest, parse_partition_table, Md5Hasher, PartitionError,
    PartitionLayout, PartitionSpan,
};
pub use rom_boundary::{
    AlignedBounce64, AlignedSectorBuf, CacheControl, FlashError, FlashIo, MockCache, MockFlash,
    FLASH_SECTOR_SIZE,
};
#[cfg(target_arch = "xtensa")]
pub use rom_boundary::{RomCache, RomFlash};
#[cfg(target_arch = "xtensa")]
pub use wdt::XtensaWdtMmio;
pub use wdt::{configure_watchdogs_and_isolation, record_boot_stage, trigger_sw_reset, WdtMmio};
