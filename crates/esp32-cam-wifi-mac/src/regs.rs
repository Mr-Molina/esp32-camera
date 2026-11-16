//! Hardware MMIO register definitions and PHY rate constants.
//!
//! All constants correspond to reverse-engineered MMIO registers.

// --- BSSID Filter Table Registers (Slots 0..=2) ---

/// Base address of BSSID filter address low registers (slots 0..=2).
pub const BSSID_FILTER_BASE_LOW: u32 = 0x3FF7_3000;
/// Base address of BSSID filter address high registers (slots 0..=2).
pub const BSSID_FILTER_BASE_HIGH: u32 = 0x3FF7_3004;
/// Base address of BSSID filter mask low registers (slots 0..=2).
pub const BSSID_MASK_BASE_LOW: u32 = 0x3FF7_3020;
/// Base address of BSSID filter mask high / control registers (slots 0..=2).
pub const BSSID_MASK_BASE_HIGH_CTRL: u32 = 0x3FF7_3024;
/// Stride between successive BSSID filter entries.
pub const BSSID_FILTER_STRIDE: u32 = 8;
/// Total number of hardware BSSID filter slots.
pub const BSSID_FILTER_SLOT_COUNT: usize = 3;
/// Filter enable bit in `BSSID_MASK_BASE_HIGH_CTRL` (`1 << 16`).
pub const BSSID_BIT_ENABLE: u32 = 1 << 16;

/// Returns the MMIO address for BSSID filter slot low-word register.
#[inline]
pub const fn bssid_filter_low_reg(slot: usize) -> u32 {
    BSSID_FILTER_BASE_LOW + (slot as u32) * BSSID_FILTER_STRIDE
}

/// Returns the MMIO address for BSSID filter slot high-word register.
#[inline]
pub const fn bssid_filter_high_reg(slot: usize) -> u32 {
    BSSID_FILTER_BASE_HIGH + (slot as u32) * BSSID_FILTER_STRIDE
}

/// Returns the MMIO address for BSSID filter slot mask-low register.
#[inline]
pub const fn bssid_mask_low_reg(slot: usize) -> u32 {
    BSSID_MASK_BASE_LOW + (slot as u32) * BSSID_FILTER_STRIDE
}

/// Returns the MMIO address for BSSID filter slot mask-high control register.
#[inline]
pub const fn bssid_mask_high_ctrl_reg(slot: usize) -> u32 {
    BSSID_MASK_BASE_HIGH_CTRL + (slot as u32) * BSSID_FILTER_STRIDE
}

// --- STA MAC Address Table Registers (Slots 0..=2) ---

/// Base address of STA MAC address low registers (slots 0..=2).
pub const STA_MAC_BASE_LOW: u32 = 0x3FF7_3040;
/// Base address of STA MAC address high registers (slots 0..=2).
pub const STA_MAC_BASE_HIGH: u32 = 0x3FF7_3044;
/// Base address of STA MAC address mask low registers (slots 0..=2).
pub const STA_MAC_MASK_BASE_LOW: u32 = 0x3FF7_3060;
/// Base address of STA MAC address mask high / control registers (slots 0..=2).
pub const STA_MAC_MASK_BASE_HIGH_CTRL: u32 = 0x3FF7_3064;
/// Stride between successive STA MAC table entries.
pub const STA_MAC_STRIDE: u32 = 8;
/// Total number of hardware STA MAC address slots.
pub const STA_MAC_SLOT_COUNT: usize = 3;
/// Table enable bit in `STA_MAC_MASK_BASE_HIGH_CTRL` (`1 << 16`).
pub const STA_MAC_BIT_ENABLE: u32 = 1 << 16;

/// Returns the MMIO address for STA MAC address slot low-word register.
#[inline]
pub const fn sta_mac_low_reg(slot: usize) -> u32 {
    STA_MAC_BASE_LOW + (slot as u32) * STA_MAC_STRIDE
}

/// Returns the MMIO address for STA MAC address slot high-word register.
#[inline]
pub const fn sta_mac_high_reg(slot: usize) -> u32 {
    STA_MAC_BASE_HIGH + (slot as u32) * STA_MAC_STRIDE
}

/// Returns the MMIO address for STA MAC address slot mask-low register.
#[inline]
pub const fn sta_mac_mask_low_reg(slot: usize) -> u32 {
    STA_MAC_MASK_BASE_LOW + (slot as u32) * STA_MAC_STRIDE
}

/// Returns the MMIO address for STA MAC slot mask-high control register.
#[inline]
pub const fn sta_mac_mask_high_ctrl_reg(slot: usize) -> u32 {
    STA_MAC_MASK_BASE_HIGH_CTRL + (slot as u32) * STA_MAC_STRIDE
}

// --- RX Policy & DMA Registers ---

/// Base address of RX policy registers (slots 0..=2).
pub const RX_POLICY_BASE_REG: u32 = 0x3FF7_30D8;
/// RX policy bit 1 (`1 << 1`). Vendor `hal_mac_rx_set_policy` never writes
/// it, so the driver leaves it as found; kept for tests and diagnostics.
pub const RX_POLICY_BIT_BSSID_CHECK: u32 = 1 << 1;
/// Promiscuous bypass mask for RX policy (`0x110`).
pub const RX_POLICY_PROMISC_BYPASS: u32 = 0x110;

/// Returns the MMIO address for RX policy slot register.
#[inline]
pub const fn rx_policy_reg(slot: usize) -> u32 {
    RX_POLICY_BASE_REG + (slot as u32) * 4
}

/// RX configuration register.
pub const MAC_RX_REG_3080: u32 = 0x3FF7_3080;
/// RX DMA control register.
pub const RX_CTRL_REG: u32 = 0x3FF7_3084;
/// RX DMA descriptor reload trigger bit (`1`).
pub const RX_CTRL_BIT_RELOAD: u32 = 1;
/// RX DMA controller enable bit (`1 << 31`).
pub const RX_CTRL_BIT_ENABLE: u32 = 1 << 31;
/// RX DMA base descriptor register.
pub const RX_BASE_DSCR_REG: u32 = 0x3FF7_3088;
/// RX DMA next descriptor register.
pub const RX_NEXT_DSCR_REG: u32 = 0x3FF7_308C;
/// RX DMA last descriptor register.
pub const RX_LAST_DSCR_REG: u32 = 0x3FF7_3090;

/// MAC RX configuration register 0x309C.
pub const MAC_RX_REG_309C: u32 = 0x3FF7_309C;
/// MAC RX configuration register 0x30A4.
pub const MAC_RX_REG_30A4: u32 = 0x3FF7_30A4;
/// MAC RX configuration register 0x30F8.
pub const MAC_RX_REG_30F8: u32 = 0x3FF7_30F8;
/// MAC RX configuration register 0x30FC.
pub const MAC_RX_REG_30FC: u32 = 0x3FF7_30FC;
/// MAC RX configuration register 0x3100.
pub const MAC_RX_REG_3100: u32 = 0x3FF7_3100;
/// MAC RX configuration register 0x3104.
pub const MAC_RX_REG_3104: u32 = 0x3FF7_3104;
/// MAC RX configuration register 0x310C.
pub const MAC_RX_REG_310C: u32 = 0x3FF7_310C;
/// MAC RX configuration register 0x3114.
pub const MAC_RX_REG_3114: u32 = 0x3FF7_3114;
/// MAC RX configuration register 0x3118.
pub const MAC_RX_REG_3118: u32 = 0x3FF7_3118;
/// MAC RX configuration register 0x311C.
pub const MAC_RX_REG_311C: u32 = 0x3FF7_311C;
/// MAC RX configuration register 0x3120.
pub const MAC_RX_REG_3120: u32 = 0x3FF7_3120;
/// MAC RX configuration register 0x3124.
pub const MAC_RX_REG_3124: u32 = 0x3FF7_3124;
/// MAC RX configuration register 0x3148.
pub const MAC_RX_REG_3148: u32 = 0x3FF7_3148;
/// MAC RX configuration register 0x314C.
pub const MAC_RX_REG_314C: u32 = 0x3FF7_314C;
/// MAC RX configuration register 0x3158.
pub const MAC_RX_REG_3158: u32 = 0x3FF7_3158;
/// MAC RX configuration register 0x3164.
pub const MAC_RX_REG_3164: u32 = 0x3FF7_3164;
/// MAC RX configuration register 0x3284.
pub const MAC_RX_REG_3284: u32 = 0x3FF7_3284;
/// MAC RX configuration register 0x3288.
pub const MAC_RX_REG_3288: u32 = 0x3FF7_3288;

// --- Interrupts, Clock, and Reset Registers ---

/// Wi-Fi MAC primary interrupt enable register.
pub const INT_ENABLE_REG: u32 = 0x3FF7_3C40;
/// Operational interrupt enable mask for normal STA operation (`0x01E8_39E0`).
pub const INT_ENABLE_OPERATIONAL: u32 = 0x01E8_39E0;
/// Wi-Fi MAC secondary interrupt enable register.
pub const INT_ENABLE_SEC_REG: u32 = 0x3FF7_3C44;
/// Wi-Fi MAC interrupt status register.
pub const INT_STATUS_REG: u32 = 0x3FF7_3C48;
/// Wi-Fi MAC interrupt clear register.
pub const INT_CLEAR_REG: u32 = 0x3FF7_3C4C;

/// Interrupt clear/status bit: Watchdog panic (`1 << 11`).
pub const INT_PANIC_WDT: u32 = 1 << 11;
/// Interrupt clear/status bit: Composite RX interrupt mask (`0x0100_0024`).
pub const INT_RX_MASK: u32 = 0x0100_0024;
/// Interrupt clear/status bit: Transmission completed (`1 << 7`).
pub const INT_TX_DONE: u32 = 1 << 7;
/// Interrupt clear/status bit: Transmission collision (`1 << 8`).
pub const INT_TX_COLL: u32 = 1 << 8;
/// Interrupt clear/status bit: Transmission timeout (`1 << 19`).
pub const INT_TX_TIMEOUT: u32 = 1 << 19;

/// MAC control register 0x3C04 (transceiver enable / power control).
pub const MAC_CTRL_REG_3C04: u32 = 0x3FF7_3C04;
/// Mask for transceiver disable bits in `MAC_CTRL_REG_3C04` (`[7:4]`).
pub const MAC_CTRL_TRANSCEIVER_DISABLE_MASK: u32 = 0x0000_00F0;

/// MAC initialization register 0x3C1C.
pub const MAC_INIT_REG_3C1C: u32 = 0x3FF7_3C1C;
/// MAC initialization register 0x3C20.
pub const MAC_INIT_REG_3C20: u32 = 0x3FF7_3C20;
/// MAC initialization register 0x3C24.
pub const MAC_INIT_REG_3C24: u32 = 0x3FF7_3C24;
/// MAC initialization register 0x3C54.
pub const MAC_INIT_REG_3C54: u32 = 0x3FF7_3C54;
/// MAC initialization register 0x3C58.
pub const MAC_INIT_REG_3C58: u32 = 0x3FF7_3C58;
/// MAC initialization register 0x3C5C.
pub const MAC_INIT_REG_3C5C: u32 = 0x3FF7_3C5C;
/// MAC initialization register 0x3C6C.
pub const MAC_INIT_REG_3C6C: u32 = 0x3FF7_3C6C;
/// MAC initialization register 0x3C74.
pub const MAC_INIT_REG_3C74: u32 = 0x3FF7_3C74;
/// MAC initialization register 0x3C78.
pub const MAC_INIT_REG_3C78: u32 = 0x3FF7_3C78;
/// MAC initialization register 0x3C88.
pub const MAC_INIT_REG_3C88: u32 = 0x3FF7_3C88;
/// MAC initialization register 0x3CAC.
pub const MAC_INIT_REG_3CAC: u32 = 0x3FF7_3CAC;

/// DPORT Wi-Fi clock enable register (`soc/dport_reg.h`).
pub const DPORT_WIFI_CLK_EN_REG: u32 = 0x3FF0_00CC;
/// Wi-Fi clock enable bitmask (`soc/dport_reg.h`).
pub const DPORT_WIFI_CLK_WIFI_EN_M: u32 = 0x0000_0406;
/// Wi-Fi and Bluetooth common clock enable bitmask (`soc/dport_reg.h`).
pub const DPORT_WIFI_CLK_WIFI_BT_COMMON_M: u32 = 0x0000_03C9;
/// DPORT core reset enable register (`soc/dport_reg.h`).
pub const DPORT_CORE_RST_EN_REG: u32 = 0x3FF0_00D0;
/// Wi-Fi MAC core reset bit (`1 << 2`, `soc/dport_reg.h`).
pub const DPORT_WIFIMAC_RST: u32 = 1 << 2;

/// MAC clock gating and reset register (`0x3FF7_3CB8`).
pub const MAC_CLK_RST_REG: u32 = 0x3FF7_3CB8;
/// Mask to un-gate MAC clocks (`0x17FF`).
pub const MAC_CLK_RST_MASK_UNGATE: u32 = 0x17FF;
/// Transmitter active status bit (`1 << 13`).
pub const MAC_CLK_RST_BIT_TX_ACTIVE: u32 = 1 << 13;

/// MAC initialization register 0x3D24.
pub const MAC_INIT_REG_3D24: u32 = 0x3FF7_3D24;
/// MAC initialization register 0x3D78.
pub const MAC_INIT_REG_3D78: u32 = 0x3FF7_3D78;

/// Auto-ACK rate table initialization registers (`0x3FF7_3400..=0x3FF7_341C`).
pub const AUTOACK_RATE_INIT_REGS: [u32; 8] = [
    0x3FF7_3400,
    0x3FF7_3404,
    0x3FF7_3408,
    0x3FF7_340C,
    0x3FF7_3410,
    0x3FF7_3414,
    0x3FF7_3418,
    0x3FF7_341C,
];

// --- TX Slot 0 & Transmission Status Registers ---

/// TX EDCA timeout register.
pub const TX_TIMEOUT_EDCA_REG: u32 = 0x3FF7_3D1C;
/// TX PLCP0 and trigger register.
pub const TX_PLCP0_TRIG_REG: u32 = 0x3FF7_3D20;
/// Trigger bits for TX initiation (`0xC000_0000`).
pub const TX_PLCP0_TRIGGER_BITS: u32 = 0xC000_0000;
/// Mode flag for 802.11n HT transmission (`0x0020_0000`).
pub const TX_PLCP0_MODE_HT: u32 = 0x0020_0000;
/// Mode flag for legacy 802.11a/b/g transmission (`0x0060_0000`).
pub const TX_PLCP0_MODE_LEGACY: u32 = 0x0060_0000;
/// Wait ACK flag for unicast transmission (`0x0100_0000`).
pub const TX_PLCP0_WAIT_ACK: u32 = 0x0100_0000;
/// No-ACK flag for broadcast, multicast, and probe request transmission (`3 << 24`).
pub const TX_PLCP0_NO_ACK: u32 = 0x0300_0000;

/// TX PLCP1 register.
pub const TX_PLCP1_REG: u32 = 0x3FF7_4258;
/// TX PLCP2 register.
pub const TX_PLCP2_REG: u32 = 0x3FF7_425C;
/// TX HT-SIG1 register.
pub const TX_HTSIG1_REG: u32 = 0x3FF7_4260;
/// TX HT-SIG2 register.
pub const TX_HTSIG2_REG: u32 = 0x3FF7_4264;
/// Fixed `TX_HTSIG1_REG` bits [26:24]: HT-SIG2 smoothing | not-sounding |
/// reserved.
pub const TX_HTSIG1_FIXED: u32 = 0x0700_0000;
/// `TX_HTSIG1_REG` bit 31: HT-SIG2 short guard interval.
///
/// Provisional: derived from the 802.11n HT-SIG2 field order (short GI is
/// HT-SIG2 bit 7, with HT-SIG2 bits 2:0 at register bits [26:24]) and not
/// yet confirmed against the vendor `mac_tx_set_htsig` disassembly. Dormant
/// while the data rate ladder holds no HT rates.
pub const TX_HTSIG1_SHORT_GI: u32 = 0x8000_0000;
/// Fixed `TX_HTSIG2_REG` bits above the PSDU length in [15:0].
pub const TX_HTSIG2_FIXED: u32 = 0x0005_0000;
/// TX duration register.
pub const TX_DURATION_REG: u32 = 0x3FF7_4268;
/// TX PMD parameter register.
pub const TX_PMD_REG: u32 = 0x3FF7_4270;

/// TX transmission error status register.
pub const TX_ERR_STATUS_REG: u32 = 0x3FF7_3CC0;
/// TX transmission error clear register.
pub const TX_ERR_CLEAR_REG: u32 = 0x3FF7_3CBC;
/// TX transmission done status register.
pub const TX_DONE_STATUS_REG: u32 = 0x3FF7_3CC8;
/// TX transmission done clear register.
pub const TX_DONE_CLEAR_REG: u32 = 0x3FF7_3CC4;

// --- Hardware Crypto Disable & Peripheral Registers ---

/// Start address of hardware crypto engine disable range.
pub const HW_CRYPTO_DISABLE_START: u32 = 0x3FF7_3800;
/// End address of hardware crypto engine disable range.
pub const HW_CRYPTO_DISABLE_END: u32 = 0x3FF7_3814;
/// Key table base register in MAC MMIO space.
pub const KEY_TABLE_BASE_REG: u32 = 0x3FF7_4400;
/// Total number of hardware crypto key table slots.
pub const KEY_SLOT_COUNT: usize = 32;
/// Stride between consecutive hardware key slots (`0x28` bytes).
pub const KEY_SLOT_STRIDE: u32 = 0x28;

/// Low-rate disable mask register 0x6001_C860.
pub const MAC_LOW_RATE_DISABLE_REG_6860: u32 = 0x6001_C860;
/// Low-rate disable mask register 0x6001_C87C.
pub const MAC_LOW_RATE_DISABLE_REG_687C: u32 = 0x6001_C87C;

/// Hardware RNG register (DPORT alias `0x3FF7_5144` for `0x6003_5144`).
pub const WDEV_RND_REG: u32 = 0x3FF7_5144;
/// eFuse Block 0 MAC address low word register (`DR_REG_EFUSE_BASE + 0x4`).
pub const EFUSE_BLK0_RDATA1_REG: u32 = 0x3FF5_A004;
/// eFuse Block 0 MAC address high word register (`DR_REG_EFUSE_BASE + 0x8`).
pub const EFUSE_BLK0_RDATA2_REG: u32 = 0x3FF5_A008;

// --- PHY Transmission Rates ---

/// IEEE 802.11 PHY transmission rates matching ESP-IDF `wifi_phy_rate_t`.
#[non_exhaustive]
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WifiPhyRate {
    /// 1 Mbps Long preamble (802.11b).
    Rate1mL = 0x00,
    /// 2 Mbps Long preamble (802.11b).
    Rate2mL = 0x01,
    /// 5.5 Mbps Long preamble (802.11b).
    Rate5m5L = 0x02,
    /// 11 Mbps Long preamble (802.11b).
    Rate11mL = 0x03,
    /// 2 Mbps Short preamble (802.11b).
    Rate2mS = 0x05,
    /// 5.5 Mbps Short preamble (802.11b).
    Rate5m5S = 0x06,
    /// 11 Mbps Short preamble (802.11b).
    Rate11mS = 0x07,
    /// 48 Mbps (802.11g).
    Rate48m = 0x08,
    /// 24 Mbps (802.11g).
    Rate24m = 0x09,
    /// 12 Mbps (802.11g).
    Rate12m = 0x0A,
    /// 6 Mbps (802.11g).
    Rate6m = 0x0B,
    /// 54 Mbps (802.11g).
    Rate54m = 0x0C,
    /// 36 Mbps (802.11g).
    Rate36m = 0x0D,
    /// 18 Mbps (802.11g).
    Rate18m = 0x0E,
    /// 9 Mbps (802.11g).
    Rate9m = 0x0F,
    /// MCS0 Long guard interval (802.11n).
    Mcs0Lgi = 0x10,
    /// MCS1 Long guard interval (802.11n).
    Mcs1Lgi = 0x11,
    /// MCS2 Long guard interval (802.11n).
    Mcs2Lgi = 0x12,
    /// MCS3 Long guard interval (802.11n).
    Mcs3Lgi = 0x13,
    /// MCS4 Long guard interval (802.11n).
    Mcs4Lgi = 0x14,
    /// MCS5 Long guard interval (802.11n).
    Mcs5Lgi = 0x15,
    /// MCS6 Long guard interval (802.11n).
    Mcs6Lgi = 0x16,
    /// MCS7 Long guard interval (802.11n).
    Mcs7Lgi = 0x17,
    /// MCS0 Short guard interval (802.11n).
    Mcs0Sgi = 0x18,
    /// MCS1 Short guard interval (802.11n).
    Mcs1Sgi = 0x19,
    /// MCS2 Short guard interval (802.11n).
    Mcs2Sgi = 0x1A,
    /// MCS3 Short guard interval (802.11n).
    Mcs3Sgi = 0x1B,
    /// MCS4 Short guard interval (802.11n).
    Mcs4Sgi = 0x1C,
    /// MCS5 Short guard interval (802.11n).
    Mcs5Sgi = 0x1D,
    /// MCS6 Short guard interval (802.11n).
    Mcs6Sgi = 0x1E,
    /// MCS7 Short guard interval (802.11n).
    Mcs7Sgi = 0x1F,
}

impl WifiPhyRate {
    /// Returns `true` if this PHY rate is an 802.11n High Throughput (HT) rate.
    #[inline]
    pub const fn is_ht(self) -> bool {
        matches!(self as u8, 0x10..=0x1F)
    }

    /// Returns the raw hardware rate code.
    #[inline]
    pub const fn hw_code(self) -> u8 {
        self as u8
    }
}
