//! Low-level MAC (LMAC) MMIO driver, register sequencing, and bus abstraction.
//!
//! Provides the [`WifiMmioBus`] trait, [`HardwareWifiMmioBus`] for direct
//! Xtensa/ESP32 MMIO operations, [`MockWifiMmioBus`] for testing without
//! hardware, and [`LmacHal`] implementing reverse-engineered LMAC hardware
//! initialization, BSSID filtering, interrupt handling, and TX sequencing.

use crate::regs;
use crate::regs::WifiPhyRate;
use core::fmt;

/// Errors returned by LMAC hardware operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LmacError {
    /// MAC reset timed out waiting for register 0x3D24 bit 0 ready signal.
    MacResetTimeout,
    /// Transmission payload length was zero or exceeded maximum 1600 bytes.
    InvalidPayloadLength,
    /// Hardware transmission error status was asserted.
    TxHardwareError,
    /// Hardware transmission completed but peer ACK was not received.
    TxAckFailed(u32),
    /// Transmission poll loop exceeded maximum iterations without completion.
    TxTimeout,
}

impl fmt::Display for LmacError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::MacResetTimeout => f.write_str("MAC reset timed out waiting for 3D24 ready bit"),
            Self::InvalidPayloadLength => {
                f.write_str("invalid TX payload length (must be 1..=1600)")
            }
            Self::TxHardwareError => f.write_str("hardware error during transmission"),
            Self::TxAckFailed(status) => {
                write!(f, "transmission ACK failed with status code {status}")
            }
            Self::TxTimeout => f.write_str("transmission timed out waiting for completion"),
        }
    }
}

/// Decoded status flags from MAC interrupt status register.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct LmacIrqEvents {
    /// One or more RX DMA descriptors completed and frame is available.
    pub rx_ready: bool,
    /// Frame transmission completed.
    pub tx_done: bool,
    /// Transmission collision detected.
    pub tx_collision: bool,
    /// Transmission watchdog timeout occurred.
    pub tx_timeout: bool,
    /// Wi-Fi MAC panic watchdog interrupt asserted.
    pub panic_wdt: bool,
}

/// Abstract bus interface for reading and writing Wi-Fi MAC MMIO registers.
pub trait WifiMmioBus {
    /// Reads a 32-bit register from the given MMIO address.
    fn read_reg(&self, addr: u32) -> u32;

    /// Writes a 32-bit value to the given MMIO address.
    fn write_reg(&mut self, addr: u32, val: u32);

    /// Delays execution for approximately `us` microseconds.
    fn delay_us(&mut self, us: u32);

    /// Issues a hardware memory write barrier / fence.
    fn mem_barrier(&mut self);
}

/// Direct hardware MMIO bus implementation for ESP32 silicon.
#[derive(Debug, Default, Clone, Copy)]
pub struct HardwareWifiMmioBus;

impl HardwareWifiMmioBus {
    /// Creates a new direct [`HardwareWifiMmioBus`].
    #[inline]
    #[must_use]
    pub const fn new() -> Self {
        Self
    }
}

impl WifiMmioBus for HardwareWifiMmioBus {
    #[inline]
    fn read_reg(&self, addr: u32) -> u32 {
        // JUSTIFICATION: Hardware MMIO register access requires volatile read.
        // SAFETY: The provided address is a valid 4-byte aligned ESP32 MMIO register.
        unsafe { core::ptr::read_volatile(addr as *const u32) }
    }

    #[inline]
    fn write_reg(&mut self, addr: u32, val: u32) {
        // JUSTIFICATION: Hardware MMIO register access requires volatile write.
        // SAFETY: The provided address is a valid 4-byte aligned ESP32 MMIO register.
        unsafe { core::ptr::write_volatile(addr as *mut u32, val) }
    }

    #[inline]
    fn delay_us(&mut self, us: u32) {
        let spins = us.saturating_mul(240);
        for _ in 0..spins {
            core::hint::spin_loop();
        }
    }

    #[inline]
    fn mem_barrier(&mut self) {
        #[cfg(target_arch = "xtensa")]
        {
            // JUSTIFICATION: Xtensa bus ordering requires `memw` barrier instruction.
            // SAFETY: `memw` drains processor write buffers and has no memory operand.
            unsafe {
                core::arch::asm!("memw", options(nostack, preserves_flags));
            }
        }
        #[cfg(not(target_arch = "xtensa"))]
        {
            core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
        }
    }
}

/// Mock MMIO bus for testing LMAC register sequences without hardware.
///
/// Stores up to 512 unique 32-bit register addresses and their values.
/// Fully `#![no_std]`, zero heap allocation, and zero `unsafe`.
#[derive(Debug, Clone)]
pub struct MockWifiMmioBus {
    regs: [(u32, u32); 512],
    count: usize,
    /// When `true`, reading `MAC_INIT_REG_3D24` returns bit 0 set.
    pub auto_init_3d24: bool,
    /// When `true`, reading `TX_DONE_STATUS_REG` returns bit 0 set.
    pub auto_tx_done: bool,
    /// ACK status value returned in `TX_PMD_REG` bits [15:12] when
    /// `auto_tx_done` is true.
    pub auto_tx_status: u32,
    /// Count of write operations recorded by the mock bus.
    pub write_count: usize,
}

impl Default for MockWifiMmioBus {
    fn default() -> Self {
        Self::new()
    }
}

impl MockWifiMmioBus {
    /// Creates an empty [`MockWifiMmioBus`].
    #[must_use]
    pub const fn new() -> Self {
        Self {
            regs: [(0, 0); 512],
            count: 0,
            auto_init_3d24: false,
            auto_tx_done: false,
            auto_tx_status: 0,
            write_count: 0,
        }
    }

    /// Returns the current value of the register at `addr`, or 0 if unwritten.
    #[must_use]
    pub fn reg(&self, addr: u32) -> u32 {
        let mut idx = 0;
        while idx < self.count {
            if self.regs[idx].0 == addr {
                return self.regs[idx].1;
            }
            idx += 1;
        }
        0
    }

    /// Sets or updates the stored register value at `addr`.
    pub fn set_reg(&mut self, addr: u32, val: u32) {
        let mut idx = 0;
        while idx < self.count {
            if self.regs[idx].0 == addr {
                self.regs[idx].1 = val;
                return;
            }
            idx += 1;
        }
        if self.count < self.regs.len() {
            self.regs[self.count] = (addr, val);
            self.count += 1;
        }
    }
}

impl WifiMmioBus for MockWifiMmioBus {
    fn read_reg(&self, addr: u32) -> u32 {
        let mut val = self.reg(addr);
        if addr == regs::MAC_INIT_REG_3D24 && self.auto_init_3d24 && (val & 2) != 0 {
            val |= 1;
        }
        if addr == regs::TX_DONE_STATUS_REG && self.auto_tx_done {
            val |= 1;
        }
        if addr == regs::TX_PMD_REG && self.auto_tx_done {
            val = (val & !0xF000) | ((self.auto_tx_status & 0x0F) << 12);
        }
        val
    }

    fn write_reg(&mut self, addr: u32, val: u32) {
        self.write_count = self.write_count.saturating_add(1);
        self.set_reg(addr, val);
    }

    fn delay_us(&mut self, _us: u32) {}

    fn mem_barrier(&mut self) {}
}

/// Hardware Abstraction Layer for the ESP32 Wi-Fi Low-Level MAC (LMAC).
pub struct LmacHal<B: WifiMmioBus> {
    bus: B,
}

impl<B: WifiMmioBus> LmacHal<B> {
    /// Creates a new [`LmacHal`] wrapping the provided bus interface.
    #[inline]
    pub const fn new(bus: B) -> Self {
        Self { bus }
    }

    /// Returns an immutable reference to the underlying bus.
    #[inline]
    pub const fn bus(&self) -> &B {
        &self.bus
    }

    /// Returns a mutable reference to the underlying bus.
    #[inline]
    pub fn bus_mut(&mut self) -> &mut B {
        &mut self.bus
    }

    /// Consumes the HAL wrapper and returns the underlying bus.
    #[inline]
    pub fn into_bus(self) -> B {
        self.bus
    }

    /// Initializes MAC clocks, resets core logic, programs operational registers,
    /// clears crypto key slots, configures STA MAC address, and arms RX DMA.
    pub fn init_mac(&mut self, sta_mac: [u8; 6], rx_ring_base_addr: u32) -> Result<(), LmacError> {
        // 1. hal_mac_init: un-gate MAC clocks
        let r_clk = self.bus.read_reg(regs::MAC_CLK_RST_REG);
        self.bus.write_reg(
            regs::MAC_CLK_RST_REG,
            r_clk & !regs::MAC_CLK_RST_MASK_UNGATE,
        );
        self.bus.mem_barrier();

        // 2. hal_init: assert reset bit, wait for MAC ready bit
        let r3d24 = self.bus.read_reg(regs::MAC_INIT_REG_3D24);
        self.bus.write_reg(regs::MAC_INIT_REG_3D24, r3d24 | 2);
        self.bus.mem_barrier();

        #[cfg(target_arch = "xtensa")]
        const MAX_READY_POLLS: u32 = 1_000_000;
        #[cfg(not(target_arch = "xtensa"))]
        const MAX_READY_POLLS: u32 = 10_000;

        let mut ready = false;
        let mut poll_count = 0;
        while poll_count < MAX_READY_POLLS {
            if (self.bus.read_reg(regs::MAC_INIT_REG_3D24) & 1) != 0 {
                ready = true;
                break;
            }
            core::hint::spin_loop();
            poll_count += 1;
        }
        if !ready {
            return Err(LmacError::MacResetTimeout);
        }

        self.bus.write_reg(regs::INT_ENABLE_REG, 0);
        self.bus.write_reg(regs::INT_CLEAR_REG, 0xFFFF_FFFF);
        self.bus.mem_barrier();

        // 3. mac_txrx_init
        let r3c6c = self.bus.read_reg(regs::MAC_INIT_REG_3C6C);
        self.bus
            .write_reg(regs::MAC_INIT_REG_3C6C, r3c6c | 0x8080_2100);

        let mut pol_slot = 0;
        while pol_slot < 4 {
            let p_reg = regs::rx_policy_reg(pol_slot);
            let r_pol = self.bus.read_reg(p_reg);
            self.bus.write_reg(p_reg, (r_pol & !0x20) | 0x40);
            pol_slot += 1;
        }

        let r3c74 = self.bus.read_reg(regs::MAC_INIT_REG_3C74);
        self.bus.write_reg(regs::MAC_INIT_REG_3C74, r3c74 | 8);

        let r30f8 = self.bus.read_reg(regs::MAC_RX_REG_30F8);
        self.bus.write_reg(regs::MAC_RX_REG_30F8, r30f8 & 0xFFFF);

        let r30fc = self.bus.read_reg(regs::MAC_RX_REG_30FC);
        self.bus.write_reg(regs::MAC_RX_REG_30FC, r30fc & 0xFFFF);

        let r3100 = self.bus.read_reg(regs::MAC_RX_REG_3100);
        self.bus.write_reg(regs::MAC_RX_REG_3100, r3100 & 0xFFFF);

        let r3104 = self.bus.read_reg(regs::MAC_RX_REG_3104);
        self.bus.write_reg(regs::MAC_RX_REG_3104, r3104 & 0xFFFF);

        let r30f8 = self.bus.read_reg(regs::MAC_RX_REG_30F8);
        self.bus
            .write_reg(regs::MAC_RX_REG_30F8, r30f8 | 0x0500_0000);

        let r30fc = self.bus.read_reg(regs::MAC_RX_REG_30FC);
        self.bus
            .write_reg(regs::MAC_RX_REG_30FC, r30fc | 0x0500_0000);

        let r310c = self.bus.read_reg(regs::MAC_RX_REG_310C);
        self.bus.write_reg(regs::MAC_RX_REG_310C, r310c & !0xF0);

        let r3114 = self.bus.read_reg(regs::MAC_RX_REG_3114);
        self.bus
            .write_reg(regs::MAC_RX_REG_3114, (r3114 & 0xF00F_FFFF) | 0x81B0_0000);

        let r3c78 = self.bus.read_reg(regs::MAC_INIT_REG_3C78);
        self.bus.write_reg(regs::MAC_INIT_REG_3C78, r3c78 | 3);

        let r3d78 = self.bus.read_reg(regs::MAC_INIT_REG_3D78);
        self.bus.write_reg(regs::MAC_INIT_REG_3D78, r3d78 & !0xF0);

        let r3c54 = self.bus.read_reg(regs::MAC_INIT_REG_3C54);
        self.bus
            .write_reg(regs::MAC_INIT_REG_3C54, (r3c54 & 0xFFFF_0003) | 0x0404);

        let r3c1c = self.bus.read_reg(regs::MAC_INIT_REG_3C1C);
        self.bus
            .write_reg(regs::MAC_INIT_REG_3C1C, (r3c1c & 0xFFFF_F000) | 0xC000_00F0);

        let r3c20 = self.bus.read_reg(regs::MAC_INIT_REG_3C20);
        self.bus
            .write_reg(regs::MAC_INIT_REG_3C20, (r3c20 & 0xFFFF_F000) | 0xF0);

        let r3c24 = self.bus.read_reg(regs::MAC_INIT_REG_3C24);
        self.bus
            .write_reg(regs::MAC_INIT_REG_3C24, (r3c24 & 0xFFFF_F000) | 0xF0);

        let r3cac = self.bus.read_reg(regs::MAC_INIT_REG_3CAC);
        self.bus
            .write_reg(regs::MAC_INIT_REG_3CAC, (r3cac & !0xF0) | 0x40);

        let r3c5c = self.bus.read_reg(regs::MAC_INIT_REG_3C5C);
        self.bus
            .write_reg(regs::MAC_INIT_REG_3C5C, r3c5c | 0xFFFF_0000);

        let r3c88 = self.bus.read_reg(regs::MAC_INIT_REG_3C88);
        self.bus
            .write_reg(regs::MAC_INIT_REG_3C88, r3c88 & 0xF0FF_FFFF);

        let r3288 = self.bus.read_reg(regs::MAC_RX_REG_3288);
        self.bus.write_reg(regs::MAC_RX_REG_3288, r3288 | 2);

        let r_rx_ctrl = self.bus.read_reg(regs::RX_CTRL_REG);
        self.bus
            .write_reg(regs::RX_CTRL_REG, r_rx_ctrl & 0x7FFF_FFFF);

        let mut slot = 0;
        while slot < 4 {
            let p_reg = regs::rx_policy_reg(slot);
            let val = self.bus.read_reg(p_reg);
            self.bus.write_reg(p_reg, (val | 5) & 0xFFFF_F6FF);
            slot += 1;
        }

        // RX SRAM window & ring
        self.bus.write_reg(regs::MAC_RX_REG_311C, 0x3FFA_E000);
        self.bus.write_reg(regs::MAC_RX_REG_3124, 0x3FFA_E000);
        self.bus.write_reg(regs::MAC_RX_REG_3118, 0x400A_0000);
        self.bus.write_reg(regs::MAC_RX_REG_3120, 0x400A_0000);
        self.bus
            .write_reg(regs::RX_BASE_DSCR_REG, rx_ring_base_addr);

        let r3080 = self.bus.read_reg(regs::MAC_RX_REG_3080);
        self.bus
            .write_reg(regs::MAC_RX_REG_3080, r3080 & 0xDFFF_FF00);

        self.bus.write_reg(regs::MAC_RX_REG_314C, 0x3000);
        self.bus.write_reg(regs::MAC_RX_REG_3158, 0);
        self.bus.write_reg(regs::MAC_RX_REG_3164, 0x000C);

        let r3148 = self.bus.read_reg(regs::MAC_RX_REG_3148);
        self.bus.write_reg(regs::MAC_RX_REG_3148, r3148 | 0x0900);

        let r309c = self.bus.read_reg(regs::MAC_RX_REG_309C);
        self.bus
            .write_reg(regs::MAC_RX_REG_309C, r309c | 0x0800_0000);

        // 4. hal_mac_rate_autoack_init
        self.bus.write_reg(regs::AUTOACK_RATE_INIT_REGS[1], 0);
        self.bus.write_reg(regs::AUTOACK_RATE_INIT_REGS[0], 0);
        self.bus
            .write_reg(regs::AUTOACK_RATE_INIT_REGS[3], 0x0302_0100);
        self.bus
            .write_reg(regs::AUTOACK_RATE_INIT_REGS[2], 0x0706_0500);
        self.bus
            .write_reg(regs::AUTOACK_RATE_INIT_REGS[5], 0x0B0B_0B0B);
        self.bus
            .write_reg(regs::AUTOACK_RATE_INIT_REGS[4], 0x0B0B_0B0B);
        self.bus
            .write_reg(regs::AUTOACK_RATE_INIT_REGS[7], 0x090B_0B0B);
        self.bus
            .write_reg(regs::AUTOACK_RATE_INIT_REGS[6], 0x0909_0909);
        self.disable_low_rate();

        // 5. hal_attenna_init
        let r_plcp2 = self.bus.read_reg(regs::TX_PLCP2_REG);
        self.bus
            .write_reg(regs::TX_PLCP2_REG, (r_plcp2 & !0x1F) | 0x20);

        let r3284 = self.bus.read_reg(regs::MAC_RX_REG_3284);
        self.bus
            .write_reg(regs::MAC_RX_REG_3284, (r3284 & !0x07) | 0x20);

        // 6. hal_crypto_init (pure-Rust CCMP passthrough: 0x30000 bypass on
        // STA 0x3FF7_3800 and AP 0x3FF7_3804; 0 on 0x3FF7_3808..=0x3FF7_3814)
        let mut crypto_addr = regs::HW_CRYPTO_DISABLE_START;
        while crypto_addr <= regs::HW_CRYPTO_DISABLE_END {
            let val = if crypto_addr <= regs::HW_CRYPTO_DISABLE_START + 4 {
                0x0003_0000
            } else {
                0
            };
            self.bus.write_reg(crypto_addr, val);
            crypto_addr += 4;
        }

        let mut key_slot = 0;
        while key_slot < regs::KEY_SLOT_COUNT {
            let slot_base = regs::KEY_TABLE_BASE_REG + (key_slot as u32) * regs::KEY_SLOT_STRIDE;
            let mut word_idx = 0;
            while word_idx < 10 {
                self.bus.write_reg(slot_base + word_idx * 4, 0);
                word_idx += 1;
            }
            key_slot += 1;
        }

        // 7. Clear BSSID filter slots 0..=2
        let mut bssid_idx = 0;
        while bssid_idx < regs::BSSID_FILTER_SLOT_COUNT {
            let ctrl = regs::bssid_mask_high_ctrl_reg(bssid_idx);
            let val = self.bus.read_reg(ctrl);
            self.bus.write_reg(ctrl, val & !regs::BSSID_BIT_ENABLE);
            bssid_idx += 1;
        }

        // 8. Program STA MAC address in hardware slot 0
        let mac_low = u32::from_le_bytes([sta_mac[0], sta_mac[1], sta_mac[2], sta_mac[3]]);
        let mac_high = u32::from(sta_mac[4]) | (u32::from(sta_mac[5]) << 8);
        self.bus.write_reg(regs::sta_mac_low_reg(0), mac_low);
        self.bus.write_reg(regs::sta_mac_high_reg(0), mac_high);
        self.bus
            .write_reg(regs::sta_mac_mask_low_reg(0), 0xFFFF_FFFF);
        self.bus
            .write_reg(regs::sta_mac_mask_high_ctrl_reg(0), 0x0001_FFFF);

        // 9. Operational interrupt and RX DMA arming
        let r3c04 = self.bus.read_reg(regs::MAC_CTRL_REG_3C04);
        self.bus.write_reg(
            regs::MAC_CTRL_REG_3C04,
            r3c04 & !regs::MAC_CTRL_TRANSCEIVER_DISABLE_MASK,
        );
        self.bus.mem_barrier();

        self.bus
            .write_reg(regs::INT_ENABLE_REG, regs::INT_ENABLE_OPERATIONAL);

        let r309c_arm = self.bus.read_reg(regs::MAC_RX_REG_309C);
        self.bus
            .write_reg(regs::MAC_RX_REG_309C, (r309c_arm & !0xFF) | 2);

        let rx_ctrl_arm = self.bus.read_reg(regs::RX_CTRL_REG);
        self.bus
            .write_reg(regs::RX_CTRL_REG, rx_ctrl_arm | regs::RX_CTRL_BIT_ENABLE);

        Ok(())
    }

    /// Deinitializes Wi-Fi MAC, disabling interrupts, halting RX DMA, and
    /// gating clocks.
    pub fn deinit_mac(&mut self) {
        let r3c04 = self.bus.read_reg(regs::MAC_CTRL_REG_3C04);
        self.bus.write_reg(
            regs::MAC_CTRL_REG_3C04,
            r3c04 | regs::MAC_CTRL_TRANSCEIVER_DISABLE_MASK,
        );
        self.bus.write_reg(regs::INT_ENABLE_REG, 0);
        self.bus.write_reg(regs::INT_ENABLE_SEC_REG, 0);

        let r_rx = self.bus.read_reg(regs::RX_CTRL_REG);
        self.bus
            .write_reg(regs::RX_CTRL_REG, r_rx & !regs::RX_CTRL_BIT_ENABLE);

        let r_clk = self.bus.read_reg(regs::MAC_CLK_RST_REG);
        self.bus
            .write_reg(regs::MAC_CLK_RST_REG, r_clk | regs::MAC_CLK_RST_MASK_UNGATE);

        self.bus.delay_us(32);

        let mut busy = true;
        let mut poll = 0;
        while poll < 200 {
            let clk_stat = self.bus.read_reg(regs::MAC_CLK_RST_REG);
            if (clk_stat & regs::MAC_CLK_RST_BIT_TX_ACTIVE) == 0 {
                busy = false;
                break;
            }
            self.bus.delay_us(1);
            poll += 1;
        }

        if busy {
            let r3c58 = self.bus.read_reg(regs::MAC_INIT_REG_3C58);
            self.bus
                .write_reg(regs::MAC_INIT_REG_3C58, (r3c58 & !0x0003_0000) | (2 << 16));
            self.bus.delay_us(5);
            let r3c58_clr = self.bus.read_reg(regs::MAC_INIT_REG_3C58);
            self.bus
                .write_reg(regs::MAC_INIT_REG_3C58, r3c58_clr & !0x0003_0000);
        }
    }

    /// Configures RX policy for passive scanning (promiscuous BSSID bypass).
    ///
    /// Vendor `hal_mac_rx_set_policy` only toggles `0x110` in the policy
    /// register; bit 1 is never written and is preserved as found.
    pub fn set_scan_rx_policy(&mut self) {
        let ctrl_reg = regs::bssid_mask_high_ctrl_reg(0);
        let val = self.bus.read_reg(ctrl_reg);
        self.bus.write_reg(ctrl_reg, val & !regs::BSSID_BIT_ENABLE);

        let pol_reg = regs::RX_POLICY_BASE_REG;
        let pol = self.bus.read_reg(pol_reg);
        self.bus
            .write_reg(pol_reg, pol | regs::RX_POLICY_PROMISC_BYPASS);
    }

    /// Sets the hardware BSSID filter address in slot 0 and enforces
    /// BSSID checking through the BSSID mask enable bit (bit 16 of
    /// `0x3FF7_3024`) and by clearing the promiscuous bypass (`0x110`).
    pub fn set_associated_bssid(&mut self, bssid: [u8; 6]) {
        let ctrl_reg = regs::bssid_mask_high_ctrl_reg(0);
        let ctrl = self.bus.read_reg(ctrl_reg);
        self.bus.write_reg(ctrl_reg, ctrl & !regs::BSSID_BIT_ENABLE);

        let low = u32::from_le_bytes([bssid[0], bssid[1], bssid[2], bssid[3]]);
        self.bus.write_reg(regs::bssid_filter_low_reg(0), low);

        let high_cur = self.bus.read_reg(regs::bssid_filter_high_reg(0));
        let high_val = (high_cur & 0xFFFF_0000) | u32::from(bssid[4]) | (u32::from(bssid[5]) << 8);
        self.bus.write_reg(regs::bssid_filter_high_reg(0), high_val);

        self.bus.write_reg(regs::bssid_mask_low_reg(0), 0xFFFF_FFFF);
        self.bus
            .write_reg(regs::bssid_mask_high_ctrl_reg(0), 0x0001_FFFF);

        let pol_reg = regs::RX_POLICY_BASE_REG;
        let pol = self.bus.read_reg(pol_reg);
        self.bus
            .write_reg(pol_reg, pol & !regs::RX_POLICY_PROMISC_BYPASS);
    }

    /// Reads the factory MAC address from ESP32 eFuse Block 0.
    #[must_use]
    pub fn read_efuse_sta_mac(&self) -> [u8; 6] {
        let w1 = self.bus.read_reg(regs::EFUSE_BLK0_RDATA1_REG);
        let w2 = self.bus.read_reg(regs::EFUSE_BLK0_RDATA2_REG);
        [
            (w2 >> 8) as u8,
            w2 as u8,
            (w1 >> 24) as u8,
            (w1 >> 16) as u8,
            (w1 >> 8) as u8,
            w1 as u8,
        ]
    }

    /// Reads a 32-bit hardware random number from the WDEV RNG peripheral.
    #[must_use]
    pub fn hw_random_u32(&self) -> u32 {
        self.bus.read_reg(regs::WDEV_RND_REG)
    }

    /// Disables 802.11b low-rate transmissions in MAC rate control tables.
    pub fn disable_low_rate(&mut self) {
        self.bus
            .write_reg(regs::AUTOACK_RATE_INIT_REGS[7], 0x090B_0B0B);
        self.bus
            .write_reg(regs::AUTOACK_RATE_INIT_REGS[6], 0x0909_0909);

        let r6860 = self.bus.read_reg(regs::MAC_LOW_RATE_DISABLE_REG_6860);
        self.bus
            .write_reg(regs::MAC_LOW_RATE_DISABLE_REG_6860, r6860 & 0xFFFF_F3FF);

        let r687c = self.bus.read_reg(regs::MAC_LOW_RATE_DISABLE_REG_687C);
        self.bus
            .write_reg(regs::MAC_LOW_RATE_DISABLE_REG_687C, r687c & 0xFFFF_F7FF);
    }

    /// Enables 802.11b 1/2 Mbps CCK low-rate transmissions in MAC tables.
    pub fn enable_low_rate(&mut self) {
        self.bus
            .write_reg(regs::AUTOACK_RATE_INIT_REGS[7], 0x0B0B_0B0B);
        self.bus
            .write_reg(regs::AUTOACK_RATE_INIT_REGS[6], 0x0B0B_0B0B);

        let r6860 = self.bus.read_reg(regs::MAC_LOW_RATE_DISABLE_REG_6860);
        self.bus
            .write_reg(regs::MAC_LOW_RATE_DISABLE_REG_6860, r6860 | 0x0000_0C00);

        let r687c = self.bus.read_reg(regs::MAC_LOW_RATE_DISABLE_REG_687C);
        self.bus
            .write_reg(regs::MAC_LOW_RATE_DISABLE_REG_687C, r687c | 0x0000_0800);
    }

    /// Reads the interrupt status register, clears asserted interrupts (W1C),
    /// and returns decoded [`LmacIrqEvents`].
    pub fn poll_and_clear_irqs(&mut self) -> LmacIrqEvents {
        let status = self.bus.read_reg(regs::INT_STATUS_REG);
        self.bus.write_reg(regs::INT_CLEAR_REG, status);
        LmacIrqEvents {
            rx_ready: (status & regs::INT_RX_MASK) != 0,
            tx_done: (status & regs::INT_TX_DONE) != 0,
            tx_collision: (status & regs::INT_TX_COLL) != 0,
            tx_timeout: (status & regs::INT_TX_TIMEOUT) != 0,
            panic_wdt: (status & regs::INT_PANIC_WDT) != 0,
        }
    }

    /// Signals the RX DMA controller to reload descriptor ring pointers.
    pub fn reload_rx_dma(&mut self) {
        let r = self.bus.read_reg(regs::RX_CTRL_REG);
        self.bus
            .write_reg(regs::RX_CTRL_REG, r | regs::RX_CTRL_BIT_RELOAD);
    }

    /// Returns the raw hardware RX end-state byte from register 0x30A4.
    #[must_use]
    pub fn rx_end_state(&self) -> u8 {
        (self.bus.read_reg(regs::MAC_RX_REG_30A4) & 0xFF) as u8
    }

    /// Submits a frame for transmission and polls until completion, error,
    /// or timeout.
    pub fn submit_and_poll_tx(
        &mut self,
        desc_addr: u32,
        mpdu_len: usize,
        rate: WifiPhyRate,
        dur_us: u16,
        wait_ack: bool,
        max_polls: u32,
    ) -> Result<(), LmacError> {
        if mpdu_len == 0 || mpdu_len > 1600 {
            return Err(LmacError::InvalidPayloadLength);
        }

        // 1. Program EDCA transmission timeout
        self.bus.write_reg(
            regs::TX_TIMEOUT_EDCA_REG,
            (10 & 0xFFF) | ((0x0F & 0x3FF) << 12) | ((7 & 0x0F) << 24),
        );

        // 2. Program PLCP1 parameters.
        // Hardware TX_PLCP1_REG bits [11:0] require the PSDU length, which
        // includes the 4-byte hardware-appended 802.11 FCS (MPDU length + 4).
        let psdu_len = (mpdu_len as u32).saturating_add(4);
        let is_ht_u32 = if rate.is_ht() { 1u32 } else { 0u32 };
        let plcp1 = ((is_ht_u32 << 9) << 16)
            | (u32::from(rate.hw_code() & 0x1F) << 12)
            | (psdu_len & 0xFFF);
        self.bus.write_reg(regs::TX_PLCP1_REG, plcp1);

        // 3. Program PLCP2 antenna configuration
        self.bus.write_reg(regs::TX_PLCP2_REG, 0x20);

        // 4. Program transmission duration
        self.bus.write_reg(
            regs::TX_DURATION_REG,
            (u32::from(dur_us) << 16) | u32::from(dur_us),
        );

        // 5. If 802.11n HT rate, program HT-SIG1 and HT-SIG2 (vendor
        // `mac_tx_set_htsig` parity).
        // TX_HTSIG1_REG: MCS in [6:0], PSDU length (MPDU length + 4-byte
        // 802.11 FCS) in [23:8], and the low byte of the 802.11n HT-SIG2
        // field in [31:24]: smoothing | not-sounding | reserved = 0x07, with
        // short GI as HT-SIG2 bit 7 (register bit 31).
        // TX_HTSIG2_REG: PSDU length in [15:0] under the fixed 0x0005_0000.
        if rate.is_ht() {
            let short_gi = if (rate.hw_code() & 0x08) != 0 {
                regs::TX_HTSIG1_SHORT_GI
            } else {
                0
            };
            let htsig1 = regs::TX_HTSIG1_FIXED
                | short_gi
                | ((psdu_len & 0xFFFF) << 8)
                | u32::from(rate.hw_code() & 0x07);
            self.bus.write_reg(regs::TX_HTSIG1_REG, htsig1);

            let htsig2 = (psdu_len & 0xFFFF) | regs::TX_HTSIG2_FIXED;
            self.bus.write_reg(regs::TX_HTSIG2_REG, htsig2);
        }

        // 6. Build PLCP0 base command word
        let mode = if rate.is_ht() {
            regs::TX_PLCP0_MODE_HT
        } else {
            regs::TX_PLCP0_MODE_LEGACY
        };
        let ack = if wait_ack {
            regs::TX_PLCP0_WAIT_ACK
        } else {
            regs::TX_PLCP0_NO_ACK
        };
        let base = (desc_addr & 0x000F_FFFF) | mode | ack;

        // Write descriptor base address, barrier, then assert trigger bits
        self.bus.write_reg(regs::TX_PLCP0_TRIG_REG, base);
        self.bus.mem_barrier();
        self.bus
            .write_reg(regs::TX_PLCP0_TRIG_REG, base | regs::TX_PLCP0_TRIGGER_BITS);

        // 7. Poll for completion, error, or loop exhaustion
        let mut poll = 0;
        while poll < max_polls {
            let err_status = self.bus.read_reg(regs::TX_ERR_STATUS_REG);
            if (err_status & 0x0001_0001) != 0 {
                let r_err_clr = self.bus.read_reg(regs::TX_ERR_CLEAR_REG);
                self.bus.write_reg(
                    regs::TX_ERR_CLEAR_REG,
                    (r_err_clr & 0xF800_F800) | (err_status & 0x0001_0001),
                );
                let r_trig = self.bus.read_reg(regs::TX_PLCP0_TRIG_REG);
                self.bus.write_reg(
                    regs::TX_PLCP0_TRIG_REG,
                    r_trig & !regs::TX_PLCP0_TRIGGER_BITS,
                );
                return Err(LmacError::TxHardwareError);
            }

            let done_status = self.bus.read_reg(regs::TX_DONE_STATUS_REG);
            if (done_status & 1) != 0 {
                let pmd = self.bus.read_reg(regs::TX_PMD_REG);
                let status = (pmd >> 12) & 0x0F;
                let r_done_clr = self.bus.read_reg(regs::TX_DONE_CLEAR_REG);
                self.bus.write_reg(regs::TX_DONE_CLEAR_REG, r_done_clr | 1);
                if status == 0 {
                    return Ok(());
                }
                return Err(LmacError::TxAckFailed(status));
            }

            self.bus.delay_us(1);
            poll += 1;
        }

        // Timeout: abort trigger and return error
        let r_trig = self.bus.read_reg(regs::TX_PLCP0_TRIG_REG);
        self.bus.write_reg(
            regs::TX_PLCP0_TRIG_REG,
            r_trig & !regs::TX_PLCP0_TRIGGER_BITS,
        );
        Err(LmacError::TxTimeout)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_init_mac_success() {
        let mut mock = MockWifiMmioBus::new();
        mock.auto_init_3d24 = true;
        mock.set_reg(regs::MAC_CTRL_REG_3C04, 0xFF);
        let mut hal = LmacHal::new(mock);

        let mac = [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC];
        let ring_addr = 0x3FFE_0000;
        let res = hal.init_mac(mac, ring_addr);
        assert_eq!(res, Ok(()));

        let bus = hal.bus();
        // Check STA MAC slot 0 low and high registers
        assert_eq!(bus.reg(regs::sta_mac_low_reg(0)), 0x7856_3412);
        assert_eq!(bus.reg(regs::sta_mac_high_reg(0)), 0xBC9A);
        assert_eq!(bus.reg(regs::sta_mac_mask_low_reg(0)), 0xFFFF_FFFF);
        assert_eq!(bus.reg(regs::sta_mac_mask_high_ctrl_reg(0)), 0x0001_FFFF);

        // Check RX base descriptor
        assert_eq!(bus.reg(regs::RX_BASE_DSCR_REG), ring_addr);

        // Check operational interrupt enable
        assert_eq!(bus.reg(regs::INT_ENABLE_REG), regs::INT_ENABLE_OPERATIONAL);

        // Check transceiver control bits [7:4] cleared
        assert_eq!(bus.reg(regs::MAC_CTRL_REG_3C04), 0x0F);

        // Check crypto bypass (0x30000 on 0x3800/0x3804; 0 on 0x3808..=0x3814)
        assert_eq!(bus.reg(regs::HW_CRYPTO_DISABLE_START), 0x0003_0000);
        assert_eq!(bus.reg(regs::HW_CRYPTO_DISABLE_START + 4), 0x0003_0000);
        assert_eq!(bus.reg(regs::HW_CRYPTO_DISABLE_START + 8), 0);
        assert_eq!(bus.reg(regs::HW_CRYPTO_DISABLE_END), 0);

        // Check key slot 0 zeroed
        assert_eq!(bus.reg(regs::KEY_TABLE_BASE_REG), 0);
    }

    #[test]
    fn test_init_mac_timeout_when_3d24_never_ready() {
        let mock = MockWifiMmioBus::new();
        // auto_init_3d24 is false, 3D24 remains 0
        let mut hal = LmacHal::new(mock);

        let mac = [0x00; 6];
        let res = hal.init_mac(mac, 0);
        assert_eq!(res, Err(LmacError::MacResetTimeout));
    }

    #[test]
    fn test_deinit_mac_sequence() {
        let mut mock = MockWifiMmioBus::new();
        mock.set_reg(regs::MAC_CLK_RST_REG, 0);
        let mut hal = LmacHal::new(mock);

        hal.deinit_mac();
        assert_eq!(hal.bus().reg(regs::INT_ENABLE_REG), 0);
        assert_eq!(
            hal.bus().reg(regs::MAC_CLK_RST_REG) & regs::MAC_CLK_RST_MASK_UNGATE,
            regs::MAC_CLK_RST_MASK_UNGATE
        );
    }

    #[test]
    fn test_rx_policy_scan_vs_associated_bssid() {
        let mock = MockWifiMmioBus::new();
        let mut hal = LmacHal::new(mock);

        hal.set_scan_rx_policy();
        let scan_pol = hal.bus().reg(regs::RX_POLICY_BASE_REG);
        assert_eq!(scan_pol & regs::RX_POLICY_BIT_BSSID_CHECK, 0);
        assert_eq!(
            scan_pol & regs::RX_POLICY_PROMISC_BYPASS,
            regs::RX_POLICY_PROMISC_BYPASS
        );

        let bssid = [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF];
        hal.set_associated_bssid(bssid);
        let assoc_pol = hal.bus().reg(regs::RX_POLICY_BASE_REG);
        // Vendor hal_mac_rx_set_policy only toggles 0x110; it never sets
        // bit 1.
        assert_eq!(assoc_pol & regs::RX_POLICY_BIT_BSSID_CHECK, 0);
        assert_eq!(assoc_pol & regs::RX_POLICY_PROMISC_BYPASS, 0);

        assert_eq!(hal.bus().reg(regs::bssid_filter_low_reg(0)), 0xDDCC_BBAA);
        assert_eq!(hal.bus().reg(regs::bssid_filter_high_reg(0)), 0xFFEE);
        assert_eq!(hal.bus().reg(regs::bssid_mask_low_reg(0)), 0xFFFF_FFFF);
        assert_eq!(
            hal.bus().reg(regs::bssid_mask_high_ctrl_reg(0)),
            0x0001_FFFF
        );
    }

    #[test]
    fn test_rx_policy_toggles_only_promisc_bypass_bits() {
        let mut mock = MockWifiMmioBus::new();
        mock.set_reg(regs::RX_POLICY_BASE_REG, 0x8000_0002);
        let mut hal = LmacHal::new(mock);

        hal.set_scan_rx_policy();
        assert_eq!(hal.bus().reg(regs::RX_POLICY_BASE_REG), 0x8000_0112);

        hal.set_associated_bssid([0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF]);
        assert_eq!(hal.bus().reg(regs::RX_POLICY_BASE_REG), 0x8000_0002);
    }

    #[test]
    fn test_submit_and_poll_tx_ht_writes_vendor_htsig_words() {
        let mut mock = MockWifiMmioBus::new();
        mock.auto_tx_done = true;
        mock.auto_tx_status = 0;
        let mut hal = LmacHal::new(mock);

        let res = hal.submit_and_poll_tx(0x1000, 128, WifiPhyRate::Mcs7Lgi, 40, true, 100);
        assert_eq!(res, Ok(()));

        // mac_tx_set_htsig parity: HT-SIG2 smoothing | not-sounding |
        // reserved (0x07) sits in HTSIG1 [26:24] above HT length [23:8] and
        // MCS [6:0]; HTSIG2 carries the PSDU length under 0x0005_0000.
        let psdu_len = 132u32;
        assert_eq!(
            hal.bus().reg(regs::TX_HTSIG1_REG),
            0x0700_0000 | (psdu_len << 8) | 7
        );
        assert_eq!(hal.bus().reg(regs::TX_HTSIG2_REG), psdu_len | 0x0005_0000);
    }

    #[test]
    fn test_submit_and_poll_tx_ht_short_gi_sets_htsig1_bit_31() {
        let mut mock = MockWifiMmioBus::new();
        mock.auto_tx_done = true;
        mock.auto_tx_status = 0;
        let mut hal = LmacHal::new(mock);

        let res = hal.submit_and_poll_tx(0x1000, 128, WifiPhyRate::Mcs7Sgi, 40, true, 100);
        assert_eq!(res, Ok(()));

        let psdu_len = 132u32;
        assert_eq!(
            hal.bus().reg(regs::TX_HTSIG1_REG),
            0x8700_0000 | (psdu_len << 8) | 7
        );
        // Short GI must not corrupt the length field in HTSIG2.
        assert_eq!(hal.bus().reg(regs::TX_HTSIG2_REG), psdu_len | 0x0005_0000);
    }

    #[test]
    fn test_poll_and_clear_irqs_w1c() {
        let mut mock = MockWifiMmioBus::new();
        let test_status = regs::INT_RX_MASK | regs::INT_TX_DONE | regs::INT_PANIC_WDT;
        mock.set_reg(regs::INT_STATUS_REG, test_status);
        let mut hal = LmacHal::new(mock);

        let events = hal.poll_and_clear_irqs();
        assert!(events.rx_ready);
        assert!(events.tx_done);
        assert!(!events.tx_collision);
        assert!(!events.tx_timeout);
        assert!(events.panic_wdt);

        // Confirm W1C write to INT_CLEAR_REG
        assert_eq!(hal.bus().reg(regs::INT_CLEAR_REG), test_status);
    }

    #[test]
    fn test_efuse_mac_and_rng() {
        let mut mock = MockWifiMmioBus::new();
        // EFUSE word 1: 0x1122_3344 (LE: bytes [44, 33, 22, 11])
        // EFUSE word 2: 0x0000_AABB
        mock.set_reg(regs::EFUSE_BLK0_RDATA1_REG, 0x1122_3344);
        mock.set_reg(regs::EFUSE_BLK0_RDATA2_REG, 0x0000_AABB);
        mock.set_reg(regs::WDEV_RND_REG, 0xDEAD_BEEF);
        let hal = LmacHal::new(mock);

        let mac = hal.read_efuse_sta_mac();
        assert_eq!(mac, [0xAA, 0xBB, 0x11, 0x22, 0x33, 0x44]);
        assert_eq!(hal.hw_random_u32(), 0xDEAD_BEEF);
    }

    #[test]
    fn test_submit_and_poll_tx_success() {
        let mut mock = MockWifiMmioBus::new();
        mock.auto_tx_done = true;
        mock.auto_tx_status = 0;
        let mut hal = LmacHal::new(mock);

        let mpdu_len = 128;
        let res = hal.submit_and_poll_tx(0x1000, mpdu_len, WifiPhyRate::Mcs7Lgi, 40, true, 100);
        assert_eq!(res, Ok(()));

        let bus = hal.bus();
        // Hardware TX_PLCP1_REG bits [11:0] and TX_HTSIG1_REG bits [23:8]
        // expect PSDU length (MPDU length + 4-byte 802.11 FCS).
        let expected_psdu_len = (mpdu_len as u32).saturating_add(4);
        let plcp1 = bus.reg(regs::TX_PLCP1_REG);
        assert_eq!(plcp1 & 0xFFF, expected_psdu_len & 0xFFF);

        let htsig1 = bus.reg(regs::TX_HTSIG1_REG);
        assert_eq!((htsig1 >> 8) & 0xFFFF, expected_psdu_len & 0xFFFF);
    }

    #[test]
    fn test_submit_and_poll_tx_ack_fail() {
        let mut mock = MockWifiMmioBus::new();
        mock.auto_tx_done = true;
        mock.auto_tx_status = 5;
        let mut hal = LmacHal::new(mock);

        let res = hal.submit_and_poll_tx(0x1000, 128, WifiPhyRate::Rate11mS, 40, true, 100);
        assert_eq!(res, Err(LmacError::TxAckFailed(5)));
    }

    #[test]
    fn test_submit_and_poll_tx_hw_error() {
        let mut mock = MockWifiMmioBus::new();
        mock.set_reg(regs::TX_ERR_STATUS_REG, 1);
        let mut hal = LmacHal::new(mock);

        let res = hal.submit_and_poll_tx(0x1000, 128, WifiPhyRate::Rate54m, 40, true, 100);
        assert_eq!(res, Err(LmacError::TxHardwareError));
    }

    #[test]
    fn test_submit_and_poll_tx_timeout() {
        let mock = MockWifiMmioBus::new();
        let mut hal = LmacHal::new(mock);

        let res = hal.submit_and_poll_tx(0x1000, 128, WifiPhyRate::Rate1mL, 40, false, 10);
        assert_eq!(res, Err(LmacError::TxTimeout));
    }

    #[test]
    fn test_submit_and_poll_tx_broadcast_sets_no_ack_bits() {
        let mut mock = MockWifiMmioBus::new();
        mock.auto_tx_done = true;
        mock.auto_tx_status = 0;
        let mut hal = LmacHal::new(mock);

        // Broadcast Probe Request: legacy 1 Mbps, no ACK expected.
        let res = hal.submit_and_poll_tx(0x1000, 64, WifiPhyRate::Rate1mL, 0, false, 100);
        assert_eq!(res, Ok(()));

        // Bits [25:24] must be 3 (No-ACK). 0 selects hardware RTS/CTS, which
        // times out against a broadcast DA and latches TX_PMD status 4.
        let plcp0 = hal.bus().reg(regs::TX_PLCP0_TRIG_REG);
        assert_eq!(
            plcp0 & 0x0360_0000,
            regs::TX_PLCP0_NO_ACK | regs::TX_PLCP0_MODE_LEGACY
        );
        assert_eq!(regs::TX_PLCP0_NO_ACK, 0x0300_0000);
    }

    #[test]
    fn test_submit_and_poll_tx_unicast_sets_wait_ack_bits() {
        let mut mock = MockWifiMmioBus::new();
        mock.auto_tx_done = true;
        mock.auto_tx_status = 0;
        let mut hal = LmacHal::new(mock);

        let res = hal.submit_and_poll_tx(0x1000, 64, WifiPhyRate::Rate1mL, 314, true, 100);
        assert_eq!(res, Ok(()));

        let plcp0 = hal.bus().reg(regs::TX_PLCP0_TRIG_REG);
        assert_eq!(
            plcp0 & 0x0360_0000,
            regs::TX_PLCP0_WAIT_ACK | regs::TX_PLCP0_MODE_LEGACY
        );
    }

    #[test]
    fn test_submit_and_poll_tx_invalid_len() {
        let mock = MockWifiMmioBus::new();
        let mut hal = LmacHal::new(mock);

        assert_eq!(
            hal.submit_and_poll_tx(0, 0, WifiPhyRate::Rate1mL, 0, false, 10),
            Err(LmacError::InvalidPayloadLength)
        );
        assert_eq!(
            hal.submit_and_poll_tx(0, 1601, WifiPhyRate::Rate1mL, 0, false, 10),
            Err(LmacError::InvalidPayloadLength)
        );
    }

    #[test]
    fn test_init_mac_writes_3d24_reset_bit_before_ready_poll() {
        let mut mock = MockWifiMmioBus::new();
        mock.auto_init_3d24 = true;
        let mut hal = LmacHal::new(mock);

        let mac = [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC];
        let ring_addr = 0x3FFE_0000;
        let res = hal.init_mac(mac, ring_addr);
        assert_eq!(res, Ok(()));
        assert_ne!(hal.bus().reg(regs::MAC_INIT_REG_3D24) & 2, 0);
    }

    #[test]
    fn test_tx_collision_and_timeout_error_bits() {
        let mut mock = MockWifiMmioBus::new();
        mock.set_reg(regs::TX_ERR_STATUS_REG, 1 << 16);
        let mut hal = LmacHal::new(mock);

        let res = hal.submit_and_poll_tx(0x1000, 128, WifiPhyRate::Rate54m, 40, true, 100);
        assert_eq!(res, Err(LmacError::TxHardwareError));
        assert_eq!(hal.bus().reg(regs::TX_ERR_CLEAR_REG) & (1 << 16), 1 << 16);
    }

    #[test]
    fn test_hal_crypto_init_passthrough_registers() {
        let mut mock = MockWifiMmioBus::new();
        mock.auto_init_3d24 = true;
        // Seed stale values across 0x3FF7_3800..=0x3FF7_3814 to check overwrite.
        let mut offset = 0u32;
        while offset <= 20 {
            mock.set_reg(regs::HW_CRYPTO_DISABLE_START + offset, 0xDEAD_BEEF);
            offset += 4;
        }
        let mut hal = LmacHal::new(mock);

        assert_eq!(hal.init_mac([0x02; 6], 0x3FFE_0000), Ok(()));

        let expected = [0x0003_0000u32, 0x0003_0000, 0, 0, 0, 0];
        for (idx, &exp) in expected.iter().enumerate() {
            let addr = regs::HW_CRYPTO_DISABLE_START + (idx as u32) * 4;
            assert_eq!(hal.bus().reg(addr), exp);
        }
    }

    #[test]
    fn test_enable_low_rate_restores_11b_cck_bits() {
        let mut mock = MockWifiMmioBus::new();
        mock.auto_init_3d24 = true;
        let mut hal = LmacHal::new(mock);

        assert_eq!(hal.init_mac([0x02; 6], 0x3FFE_0000), Ok(()));
        assert_eq!(hal.bus().reg(regs::AUTOACK_RATE_INIT_REGS[7]), 0x090B_0B0B);

        hal.enable_low_rate();
        assert_eq!(hal.bus().reg(regs::AUTOACK_RATE_INIT_REGS[7]), 0x0B0B_0B0B);
        assert_eq!(hal.bus().reg(regs::AUTOACK_RATE_INIT_REGS[6]), 0x0B0B_0B0B);
        assert_eq!(
            hal.bus().reg(regs::MAC_LOW_RATE_DISABLE_REG_6860) & 0x0000_0C00,
            0x0000_0C00
        );
        assert_eq!(
            hal.bus().reg(regs::MAC_LOW_RATE_DISABLE_REG_687C) & 0x0000_0800,
            0x0000_0800
        );
    }
}
