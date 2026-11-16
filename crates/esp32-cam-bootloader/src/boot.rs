//! Pure-Rust 2nd-stage bootloader orchestration for ESP32-WROVER.
//!
//! Coordinates early GPIO 16 PSRAM CS# pull-up, watchdog/isolation setup,
//! system tick configuration, partition table validation, OTADATA slot
//! selection and rollback, image verification, RAM loading, MMU mapping, and
//! final Xtensa entry handoff.

use crate::clock_spi_psram::configure_ticks_apb;
use crate::clock_spi_psram::ClockMmio;
#[cfg(target_arch = "xtensa")]
use crate::clock_spi_psram::{attach_flash_4mb_dio, configure_bbpll_240m_apb_80m, XtensaClockMmio};
use crate::gpio16::init_gpio16_psram_cs;
use crate::gpio16::GpioMmio;
#[cfg(target_arch = "xtensa")]
use crate::gpio16::{init_gpio16_xtensa, XtensaGpioMmio};
use crate::image_verify::verify_app_image;
use crate::mmu_jump::load_ram_and_map_mmu;
use crate::mmu_jump::CacheMaskControl;
use crate::mmu_jump::RamSink;
#[cfg(target_arch = "xtensa")]
use crate::mmu_jump::{jump_to_entry, XtensaDportMask, XtensaRamSink};
use crate::otadata::select_and_update_otadata;
use crate::otadata::OtaSelectError;
use crate::partitions::parse_partition_table;
use crate::partitions::PartitionError;
use crate::rom_boundary::CacheControl;
use crate::rom_boundary::FlashIo;
#[cfg(target_arch = "xtensa")]
use crate::rom_boundary::{RomCache, RomFlash};
use crate::wdt::configure_watchdogs_and_isolation;
use crate::wdt::record_boot_stage;
use crate::wdt::WdtMmio;
#[cfg(target_arch = "xtensa")]
use crate::wdt::{trigger_sw_reset, XtensaWdtMmio};

/// Breadcrumb recorded in `RTC_CNTL_STORE7_REG` after early hardware setup.
pub const BOOT_STAGE_EARLY_HW: u32 = 0xB007_0001;

/// Breadcrumb recorded after partition table parsing succeeds.
pub const BOOT_STAGE_PARTITION_OK: u32 = 0xB007_0002;

/// Breadcrumb recorded after OTADATA candidate selection succeeds.
pub const BOOT_STAGE_OTADATA_OK: u32 = 0xB007_0003;

/// Breadcrumb recorded after RAM loading and MMU mapping succeed.
pub const BOOT_STAGE_IMAGE_LOADED: u32 = 0xB007_0004;

/// Breadcrumb recorded immediately prior to jumping to application entry.
pub const BOOT_STAGE_JUMP_ENTRY: u32 = 0xB007_0005;

/// Errors returned during 2nd-stage bootloader preparation.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BootError {
    /// Partition table read, magic check, or MD5 verification failed.
    Partition(PartitionError),
    /// OTADATA evaluation or fallback selection failed.
    OtaSelect(OtaSelectError),
    /// No candidate partition passed image verification and MMU mapping.
    NoBootableImage,
    /// Early clock or flash attach hardware initialization failed.
    HardwareInit,
}

impl From<PartitionError> for BootError {
    #[inline]
    fn from(err: PartitionError) -> Self {
        Self::Partition(err)
    }
}

impl From<OtaSelectError> for BootError {
    #[inline]
    fn from(err: OtaSelectError) -> Self {
        Self::OtaSelect(err)
    }
}

/// Orchestrates the 2nd-stage bootloader sequence and returns the verified
/// application entry point address.
///
/// # Errors
///
/// Returns [`BootError::Partition`] if the partition table is invalid,
/// [`BootError::OtaSelect`] if OTADATA yields no candidate slots, or
/// [`BootError::NoBootableImage`] if every candidate fails verification or
/// MMU mapping.
pub fn prepare_boot_image<G, W, K, F, C, R, D>(
    gpio: &mut G,
    wdt: &mut W,
    clk: &mut K,
    flash: &mut F,
    cache: &mut C,
    ram: &mut R,
    dport: &mut D,
) -> Result<u32, BootError>
where
    G: GpioMmio,
    W: WdtMmio,
    K: ClockMmio,
    F: FlashIo,
    C: CacheControl,
    R: RamSink,
    D: CacheMaskControl,
{
    init_gpio16_psram_cs(gpio);
    configure_watchdogs_and_isolation(wdt);
    record_boot_stage(wdt, BOOT_STAGE_EARLY_HW);

    configure_ticks_apb(clk, 40, 80, 80);

    let layout = parse_partition_table(flash)?;
    record_boot_stage(wdt, BOOT_STAGE_PARTITION_OK);

    let candidates = select_and_update_otadata(flash, &layout)?;
    record_boot_stage(wdt, BOOT_STAGE_OTADATA_OK);

    for slot in candidates {
        let Some(cand) = slot else {
            continue;
        };
        let Ok(verified) = verify_app_image(flash, cand.span.offset, cand.span.size) else {
            continue;
        };
        if load_ram_and_map_mmu(flash, cache, ram, dport, &verified).is_ok() {
            record_boot_stage(wdt, BOOT_STAGE_IMAGE_LOADED);
            return Ok(verified.entry_addr);
        }
    }

    Err(BootError::NoBootableImage)
}

/// Executes the full Xtensa hardware 2nd-stage boot sequence and jumps to the
/// verified application image, or triggers a software reset on error.
#[cfg(target_arch = "xtensa")]
#[allow(clippy::empty_loop)]
pub fn run_xtensa_boot() -> ! {
    init_gpio16_xtensa();

    let mut gpio = XtensaGpioMmio;
    let mut wdt = XtensaWdtMmio;
    let mut clk = XtensaClockMmio;
    let mut flash = RomFlash::new();
    let mut cache = RomCache::new();
    cache.flush_uart0();
    configure_bbpll_240m_apb_80m();
    let mut ram = XtensaRamSink;
    let mut dport = XtensaDportMask;

    if attach_flash_4mb_dio().is_err() {
        cache.flush_uart0();
        trigger_sw_reset(&mut wdt);
        loop {}
    }

    match prepare_boot_image(
        &mut gpio, &mut wdt, &mut clk, &mut flash, &mut cache, &mut ram, &mut dport,
    ) {
        Ok(entry_addr) => {
            record_boot_stage(&mut wdt, BOOT_STAGE_JUMP_ENTRY);
            // JUSTIFICATION: Final bootloader handoff requires a direct machine
            // jump to the verified application entry point.
            // SAFETY: `prepare_boot_image` verified the image headers, XOR
            // checksum, SHA-256 digest, RAM load bounds, and MMU mappings.
            unsafe { jump_to_entry(entry_addr) }
        }
        Err(_) => {
            cache.flush_uart0();
            trigger_sw_reset(&mut wdt);
            loop {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gpio16::GPIO_ENABLE_W1TS_REG;
    use crate::gpio16::IO_MUX_GPIO16_REG;
    use crate::image_verify::Sha256;
    use crate::image_verify::ESP_CHECKSUM_MAGIC;
    use crate::image_verify::ESP_IMAGE_HEADER_MAGIC;
    use crate::otadata::OtaImgState;
    use crate::otadata::OtaSelectEntry;
    use crate::partitions::encode_md5_entry;
    use crate::partitions::PARTITION_MAGIC;
    use crate::partitions::PARTITION_TABLE_OFFSET;
    use crate::partitions::PART_SUBTYPE_DATA_OTA;
    use crate::partitions::PART_SUBTYPE_OTA_MIN;
    use crate::partitions::PART_TYPE_APP;
    use crate::partitions::PART_TYPE_DATA;
    use crate::rom_boundary::AlignedBounce64;
    use crate::rom_boundary::FlashError;
    use crate::rom_boundary::MockCache;
    use crate::rom_boundary::MockFlash;
    use crate::rom_boundary::FLASH_SECTOR_SIZE;
    use crate::wdt::RTC_CNTL_STORE7_REG;

    const OTADATA_OFFSET: u32 = 0x9000;
    const OTADATA_SIZE: u32 = 0x2000;
    const OTA0_OFFSET: u32 = 0xB000;
    const OTA0_SIZE: u32 = 0x2000;
    const OTA1_OFFSET: u32 = 0xD000;
    const OTA1_SIZE: u32 = 0x2000;

    #[repr(C, align(4))]
    struct AlignedTestImage {
        bytes: [u8; 512],
    }

    struct MockGpio {
        writes: usize,
        saw_pullup: bool,
        saw_enable: bool,
    }

    impl MockGpio {
        const fn new() -> Self {
            Self {
                writes: 0,
                saw_pullup: false,
                saw_enable: false,
            }
        }
    }

    impl GpioMmio for MockGpio {
        fn read_u32(&self, _addr: u32) -> u32 {
            0
        }

        fn write_u32(&mut self, addr: u32, val: u32) {
            self.writes += 1;
            if addr == IO_MUX_GPIO16_REG && val == 0x2B00 {
                self.saw_pullup = true;
            }
            if addr == GPIO_ENABLE_W1TS_REG && val == (1 << 16) {
                self.saw_enable = true;
            }
        }
    }

    struct MockWdt {
        last_stage: u32,
        writes: usize,
    }

    impl MockWdt {
        const fn new() -> Self {
            Self {
                last_stage: 0,
                writes: 0,
            }
        }
    }

    impl WdtMmio for MockWdt {
        fn read_u32(&self, _addr: u32) -> u32 {
            0
        }

        fn write_u32(&mut self, addr: u32, val: u32) {
            self.writes += 1;
            if addr == RTC_CNTL_STORE7_REG {
                self.last_stage = val;
            }
        }
    }

    struct MockClock {
        writes: usize,
    }

    impl MockClock {
        const fn new() -> Self {
            Self { writes: 0 }
        }
    }

    impl ClockMmio for MockClock {
        fn read_u32(&self, _addr: u32) -> u32 {
            0
        }

        fn write_u32(&mut self, _addr: u32, _val: u32) {
            self.writes += 1;
        }
    }

    #[repr(C, align(4))]
    struct MockRamSink {
        data: [u8; 128],
        written_len: usize,
        last_addr: u32,
    }

    impl MockRamSink {
        const fn new() -> Self {
            Self {
                data: [0u8; 128],
                written_len: 0,
                last_addr: 0,
            }
        }
    }

    impl RamSink for MockRamSink {
        fn write_ram(&mut self, addr: u32, bytes: &[u8]) {
            self.last_addr = addr;
            for &b in bytes {
                if self.written_len < self.data.len() {
                    self.data[self.written_len] = b;
                    self.written_len += 1;
                }
            }
        }
    }

    struct MockDportMask {
        unmasked: bool,
    }

    impl MockDportMask {
        const fn new() -> Self {
            Self { unmasked: false }
        }
    }

    impl CacheMaskControl for MockDportMask {
        fn unmask_flash_cache(&mut self) {
            self.unmasked = true;
        }
    }

    fn make_part_entry(ptype: u8, subtype: u8, offset: u32, size: u32) -> [u8; 32] {
        let mut entry = [0xFFu8; 32];
        entry[0..2].copy_from_slice(&PARTITION_MAGIC.to_le_bytes());
        entry[2] = ptype;
        entry[3] = subtype;
        entry[4..8].copy_from_slice(&offset.to_le_bytes());
        entry[8..12].copy_from_slice(&size.to_le_bytes());
        entry
    }

    fn write_standard_partitions(flash: &mut MockFlash<18>) {
        let mut raw = [0xFFu8; 128];
        let e_otadata = make_part_entry(
            PART_TYPE_DATA,
            PART_SUBTYPE_DATA_OTA,
            OTADATA_OFFSET,
            OTADATA_SIZE,
        );
        let e_ota0 = make_part_entry(PART_TYPE_APP, PART_SUBTYPE_OTA_MIN, OTA0_OFFSET, OTA0_SIZE);
        let e_ota1 = make_part_entry(
            PART_TYPE_APP,
            PART_SUBTYPE_OTA_MIN + 1,
            OTA1_OFFSET,
            OTA1_SIZE,
        );
        raw[0..32].copy_from_slice(&e_otadata);
        raw[32..64].copy_from_slice(&e_ota0);
        raw[64..96].copy_from_slice(&e_ota1);
        let md5 = encode_md5_entry(&raw[..96]);
        raw[96..128].copy_from_slice(&md5);

        flash
            .write_raw(PARTITION_TABLE_OFFSET, &raw)
            .expect("partition table write");
    }

    fn build_app_image(entry_addr: u32, ram_fill: u8) -> (AlignedTestImage, usize) {
        let drom_payload = [0x11u8; 32];
        let ram_payload = [ram_fill; 32];
        let segments: [(u32, &[u8]); 2] =
            [(0x3F40_0020, &drom_payload), (0x4008_0000, &ram_payload)];

        let mut img = AlignedTestImage { bytes: [0u8; 512] };
        img.bytes[0] = ESP_IMAGE_HEADER_MAGIC;
        img.bytes[1] = segments.len() as u8;
        img.bytes[4..8].copy_from_slice(&entry_addr.to_le_bytes());
        img.bytes[23] = 1;

        let mut sha = Sha256::new();
        sha.update(&img.bytes[..24]);
        let mut xor_chk = ESP_CHECKSUM_MAGIC;
        let mut cursor = 24;

        for &(load_addr, payload) in &segments {
            img.bytes[cursor..cursor + 4].copy_from_slice(&load_addr.to_le_bytes());
            img.bytes[cursor + 4..cursor + 8]
                .copy_from_slice(&(payload.len() as u32).to_le_bytes());
            sha.update(&img.bytes[cursor..cursor + 8]);
            cursor += 8;

            img.bytes[cursor..cursor + payload.len()].copy_from_slice(payload);
            for &b in payload {
                xor_chk ^= b;
            }
            sha.update(payload);
            cursor += payload.len();
        }

        let target_end = (cursor + 16) & !15;
        let pad_len = target_end - cursor;
        for b in &mut img.bytes[cursor..cursor + pad_len - 1] {
            *b = 0;
        }
        img.bytes[target_end - 1] = xor_chk;
        sha.update(&img.bytes[cursor..target_end]);
        cursor = target_end;

        let digest = sha.finalize();
        img.bytes[cursor..cursor + 32].copy_from_slice(&digest);
        cursor += 32;

        (img, cursor)
    }

    fn read_otadata_entry<F: FlashIo>(flash: &mut F, offset: u32) -> OtaSelectEntry {
        let mut bounce = AlignedBounce64::new();
        flash
            .read(offset, &mut bounce.bytes[..32])
            .expect("read otadata");
        let mut raw = [0u8; 32];
        raw.copy_from_slice(&bounce.bytes[..32]);
        OtaSelectEntry::from_bytes(raw)
    }

    #[test]
    fn valid_ota0_transitions_new_to_pending_verify_and_returns_entry() {
        let mut flash = MockFlash::<18>::new();
        write_standard_partitions(&mut flash);

        let entry0 = OtaSelectEntry::new(1, OtaImgState::New);
        flash
            .write_raw(OTADATA_OFFSET, &entry0.to_bytes())
            .expect("write otadata0");

        let (img0, len0) = build_app_image(0x4008_1000, 0xA1);
        let (img1, len1) = build_app_image(0x4008_2000, 0xB2);
        flash
            .write_raw(OTA0_OFFSET, &img0.bytes[..len0])
            .expect("write ota0");
        flash
            .write_raw(OTA1_OFFSET, &img1.bytes[..len1])
            .expect("write ota1");

        let mut gpio = MockGpio::new();
        let mut wdt = MockWdt::new();
        let mut clk = MockClock::new();
        let mut cache = MockCache::new();
        let mut ram = MockRamSink::new();
        let mut dport = MockDportMask::new();

        let entry = prepare_boot_image(
            &mut gpio, &mut wdt, &mut clk, &mut flash, &mut cache, &mut ram, &mut dport,
        )
        .expect("boot preparation succeeds");

        assert_eq!(entry, 0x4008_1000);
        assert!(gpio.saw_pullup && gpio.saw_enable);
        assert_eq!(clk.writes, 3);
        assert_eq!(wdt.last_stage, BOOT_STAGE_IMAGE_LOADED);
        assert!(dport.unmasked);
        assert_eq!(ram.written_len, 32);
        assert_eq!(&ram.data[..32], &[0xA1; 32]);

        let updated = read_otadata_entry(&mut flash, OTADATA_OFFSET);
        assert_eq!(updated.state(), OtaImgState::PendingVerify);
    }

    #[test]
    fn corrupt_ota0_falls_back_to_valid_ota1() {
        let mut flash = MockFlash::<18>::new();
        write_standard_partitions(&mut flash);

        let entry0 = OtaSelectEntry::new(1, OtaImgState::Valid);
        flash
            .write_raw(OTADATA_OFFSET, &entry0.to_bytes())
            .expect("write otadata0");

        let (mut img0, len0) = build_app_image(0x4008_1000, 0xA1);
        img0.bytes[40] ^= 0xFF; // corrupt segment payload
        let (img1, len1) = build_app_image(0x4008_2000, 0xB2);
        flash
            .write_raw(OTA0_OFFSET, &img0.bytes[..len0])
            .expect("write ota0");
        flash
            .write_raw(OTA1_OFFSET, &img1.bytes[..len1])
            .expect("write ota1");

        let mut gpio = MockGpio::new();
        let mut wdt = MockWdt::new();
        let mut clk = MockClock::new();
        let mut cache = MockCache::new();
        let mut ram = MockRamSink::new();
        let mut dport = MockDportMask::new();

        let entry = prepare_boot_image(
            &mut gpio, &mut wdt, &mut clk, &mut flash, &mut cache, &mut ram, &mut dport,
        )
        .expect("fallback to ota1 succeeds");

        assert_eq!(entry, 0x4008_2000);
        assert_eq!(ram.written_len, 32);
        assert_eq!(&ram.data[..32], &[0xB2; 32]);
        assert!(dport.unmasked);
    }

    #[test]
    fn pending_verify_rollback_aborts_ota0_and_selects_ota1() {
        let mut flash = MockFlash::<18>::new();
        write_standard_partitions(&mut flash);

        // seq=3 maps to ota_0 ((3-1)%2 = 0); seq=2 maps to ota_1 ((2-1)%2 = 1).
        let entry0 = OtaSelectEntry::new(3, OtaImgState::PendingVerify);
        let entry1 = OtaSelectEntry::new(2, OtaImgState::Valid);
        flash
            .write_raw(OTADATA_OFFSET, &entry0.to_bytes())
            .expect("write otadata0");
        flash
            .write_raw(OTADATA_OFFSET + 0x1000, &entry1.to_bytes())
            .expect("write otadata1");

        let (img0, len0) = build_app_image(0x4008_1000, 0xA1);
        let (img1, len1) = build_app_image(0x4008_2000, 0xB2);
        flash
            .write_raw(OTA0_OFFSET, &img0.bytes[..len0])
            .expect("write ota0");
        flash
            .write_raw(OTA1_OFFSET, &img1.bytes[..len1])
            .expect("write ota1");

        let mut gpio = MockGpio::new();
        let mut wdt = MockWdt::new();
        let mut clk = MockClock::new();
        let mut cache = MockCache::new();
        let mut ram = MockRamSink::new();
        let mut dport = MockDportMask::new();

        let entry = prepare_boot_image(
            &mut gpio, &mut wdt, &mut clk, &mut flash, &mut cache, &mut ram, &mut dport,
        )
        .expect("rollback selects ota1");

        assert_eq!(entry, 0x4008_2000);
        assert_eq!(&ram.data[..32], &[0xB2; 32]);

        let updated0 = read_otadata_entry(&mut flash, OTADATA_OFFSET);
        assert_eq!(updated0.state(), OtaImgState::Aborted);
    }

    #[test]
    fn all_candidates_corrupt_returns_no_bootable_image() {
        let mut flash = MockFlash::<18>::new();
        write_standard_partitions(&mut flash);

        let (mut img0, len0) = build_app_image(0x4008_1000, 0xA1);
        let (mut img1, len1) = build_app_image(0x4008_2000, 0xB2);
        img0.bytes[0] = 0x00;
        img1.bytes[0] = 0x00;
        flash
            .write_raw(OTA0_OFFSET, &img0.bytes[..len0])
            .expect("write ota0");
        flash
            .write_raw(OTA1_OFFSET, &img1.bytes[..len1])
            .expect("write ota1");

        let mut gpio = MockGpio::new();
        let mut wdt = MockWdt::new();
        let mut clk = MockClock::new();
        let mut cache = MockCache::new();
        let mut ram = MockRamSink::new();
        let mut dport = MockDportMask::new();

        let err = prepare_boot_image(
            &mut gpio, &mut wdt, &mut clk, &mut flash, &mut cache, &mut ram, &mut dport,
        )
        .expect_err("no valid image must fail");

        assert_eq!(err, BootError::NoBootableImage);
    }

    const REALISTIC_OTADATA_OFFSET: u32 = 0x17000;
    const REALISTIC_OTA0_OFFSET: u32 = 0x20000;
    const REALISTIC_OTA1_OFFSET: u32 = 0x1D0000;

    #[repr(C, align(4))]
    struct SparseSector {
        sector: u32,
        data: [u8; FLASH_SECTOR_SIZE],
    }

    struct SparseFlash<const N: usize> {
        sectors: [SparseSector; N],
        count: usize,
    }

    impl<const N: usize> SparseFlash<N> {
        fn new() -> Self {
            const EMPTY: SparseSector = SparseSector {
                sector: u32::MAX,
                data: [0xFF; FLASH_SECTOR_SIZE],
            };
            Self {
                sectors: [EMPTY; N],
                count: 0,
            }
        }

        fn get_or_create_sector(&mut self, sector: u32) -> &mut [u8; FLASH_SECTOR_SIZE] {
            for i in 0..self.count {
                if self.sectors[i].sector == sector {
                    return &mut self.sectors[i].data;
                }
            }
            assert!(self.count < N, "SparseFlash capacity exceeded");
            let idx = self.count;
            self.sectors[idx].sector = sector;
            self.sectors[idx].data = [0xFF; FLASH_SECTOR_SIZE];
            self.count += 1;
            &mut self.sectors[idx].data
        }

        fn find_sector(&self, sector: u32) -> Option<&[u8; FLASH_SECTOR_SIZE]> {
            for i in 0..self.count {
                if self.sectors[i].sector == sector {
                    return Some(&self.sectors[i].data);
                }
            }
            None
        }

        fn write_raw(&mut self, offset: u32, data: &[u8]) {
            for (i, &b) in data.iter().enumerate() {
                let curr = (offset as usize) + i;
                let sec = (curr / FLASH_SECTOR_SIZE) as u32;
                let sec_off = curr % FLASH_SECTOR_SIZE;
                let buf = self.get_or_create_sector(sec);
                buf[sec_off] = b;
            }
        }
    }

    impl<const N: usize> FlashIo for SparseFlash<N> {
        fn read(&mut self, offset: u32, buf: &mut [u8]) -> Result<(), FlashError> {
            for (i, b) in buf.iter_mut().enumerate() {
                let curr = (offset as usize) + i;
                let sec = (curr / FLASH_SECTOR_SIZE) as u32;
                let sec_off = curr % FLASH_SECTOR_SIZE;
                if let Some(s) = self.find_sector(sec) {
                    *b = s[sec_off];
                } else {
                    *b = 0xFF;
                }
            }
            Ok(())
        }

        fn write(&mut self, offset: u32, data: &[u8]) -> Result<(), FlashError> {
            for (i, &b) in data.iter().enumerate() {
                let curr = (offset as usize) + i;
                let sec = (curr / FLASH_SECTOR_SIZE) as u32;
                let sec_off = curr % FLASH_SECTOR_SIZE;
                let buf = self.get_or_create_sector(sec);
                buf[sec_off] &= b;
            }
            Ok(())
        }

        fn erase_sector(&mut self, sector: u32) -> Result<(), FlashError> {
            let buf = self.get_or_create_sector(sector);
            buf.fill(0xFF);
            Ok(())
        }
    }

    fn write_realistic_5_entry_partitions<F: FlashIo>(flash: &mut F) {
        let mut raw = [0xFFu8; 192];
        let e_nvs = make_part_entry(PART_TYPE_DATA, 0x02, 0x11000, 0x6000);
        let e_otadata = make_part_entry(
            PART_TYPE_DATA,
            PART_SUBTYPE_DATA_OTA,
            REALISTIC_OTADATA_OFFSET,
            0x2000,
        );
        let e_phy = make_part_entry(PART_TYPE_DATA, 0x01, 0x19000, 0x1000);
        let e_ota0 = make_part_entry(
            PART_TYPE_APP,
            PART_SUBTYPE_OTA_MIN,
            REALISTIC_OTA0_OFFSET,
            0x1B0000,
        );
        let e_ota1 = make_part_entry(
            PART_TYPE_APP,
            PART_SUBTYPE_OTA_MIN + 1,
            REALISTIC_OTA1_OFFSET,
            0x1B0000,
        );
        raw[0..32].copy_from_slice(&e_nvs);
        raw[32..64].copy_from_slice(&e_otadata);
        raw[64..96].copy_from_slice(&e_phy);
        raw[96..128].copy_from_slice(&e_ota0);
        raw[128..160].copy_from_slice(&e_ota1);
        let md5 = encode_md5_entry(&raw[..160]);
        raw[160..192].copy_from_slice(&md5);

        flash
            .write(PARTITION_TABLE_OFFSET, &raw)
            .expect("write realistic partitions");
    }

    #[test]
    fn realistic_5_entry_partition_table_lifecycle_and_rollback() {
        let mut flash = SparseFlash::<8>::new();
        write_realistic_5_entry_partitions(&mut flash);

        let (img0, len0) = build_app_image(0x4008_1000, 0xA1);
        let (img1, len1) = build_app_image(0x4008_2000, 0xB2);
        flash.write_raw(REALISTIC_OTA0_OFFSET, &img0.bytes[..len0]);
        flash.write_raw(REALISTIC_OTA1_OFFSET, &img1.bytes[..len1]);

        // Phase 1: Blank otadata falls back to ota_0
        let mut gpio = MockGpio::new();
        let mut wdt = MockWdt::new();
        let mut clk = MockClock::new();
        let mut cache = MockCache::new();
        let mut ram = MockRamSink::new();
        let mut dport = MockDportMask::new();

        let entry = prepare_boot_image(
            &mut gpio, &mut wdt, &mut clk, &mut flash, &mut cache, &mut ram, &mut dport,
        )
        .expect("blank otadata boot succeeds");

        assert_eq!(entry, 0x4008_1000);
        assert_eq!(&ram.data[..32], &[0xA1; 32]);
        assert_eq!(wdt.last_stage, BOOT_STAGE_IMAGE_LOADED);

        // Phase 2: seq=2 selects ota_1
        let entry_seq2 = OtaSelectEntry::new(2, OtaImgState::Valid);
        flash.write_raw(REALISTIC_OTADATA_OFFSET, &entry_seq2.to_bytes());

        let mut gpio = MockGpio::new();
        let mut wdt = MockWdt::new();
        let mut clk = MockClock::new();
        let mut cache = MockCache::new();
        let mut ram = MockRamSink::new();
        let mut dport = MockDportMask::new();

        let entry = prepare_boot_image(
            &mut gpio, &mut wdt, &mut clk, &mut flash, &mut cache, &mut ram, &mut dport,
        )
        .expect("seq=2 selects ota_1");

        assert_eq!(entry, 0x4008_2000);
        assert_eq!(&ram.data[..32], &[0xB2; 32]);

        // Phase 3: PendingVerify on seq=3 rolls back to ota_1, aborting ota_0
        // seq=3 -> ota_0 ((3-1)%2 = 0); seq=2 -> ota_1 ((2-1)%2 = 1).
        let entry_pv = OtaSelectEntry::new(3, OtaImgState::PendingVerify);
        let entry_prev = OtaSelectEntry::new(2, OtaImgState::Valid);
        flash.write_raw(REALISTIC_OTADATA_OFFSET, &entry_pv.to_bytes());
        flash.write_raw(REALISTIC_OTADATA_OFFSET + 0x1000, &entry_prev.to_bytes());

        let mut gpio = MockGpio::new();
        let mut wdt = MockWdt::new();
        let mut clk = MockClock::new();
        let mut cache = MockCache::new();
        let mut ram = MockRamSink::new();
        let mut dport = MockDportMask::new();

        let entry = prepare_boot_image(
            &mut gpio, &mut wdt, &mut clk, &mut flash, &mut cache, &mut ram, &mut dport,
        )
        .expect("pending_verify rollback selects ota_1");

        assert_eq!(entry, 0x4008_2000);
        assert_eq!(&ram.data[..32], &[0xB2; 32]);

        let updated0 = read_otadata_entry(&mut flash, REALISTIC_OTADATA_OFFSET);
        assert_eq!(updated0.state(), OtaImgState::Aborted);
    }
}
