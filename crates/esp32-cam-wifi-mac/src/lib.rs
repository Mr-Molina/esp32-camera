//! Pure-Rust ESP32 802.11 STA MAC, LMAC DMA, and smoltcp PHY adapter.

#![no_std]
#![cfg_attr(target_arch = "xtensa", feature(asm_experimental_arch))]

pub mod dma_ring;
pub mod frames_80211;
pub mod lmac_mmio;
pub mod phy_bridge;
pub mod rate_ctrl;
pub mod regs;
pub mod smoltcp_phy;
pub mod sta_fsm;

pub use crate::dma_ring::DmaRingError;
pub use crate::dma_ring::WifiRxCtrl;
pub use crate::dma_ring::WifiRxRing;
pub use crate::dma_ring::WifiTxSlot;

pub use crate::lmac_mmio::HardwareWifiMmioBus;
pub use crate::lmac_mmio::LmacError;
pub use crate::lmac_mmio::LmacHal;
pub use crate::lmac_mmio::LmacIrqEvents;
pub use crate::lmac_mmio::MockWifiMmioBus;
pub use crate::lmac_mmio::WifiMmioBus;

pub use crate::phy_bridge::PhyBridgeError;

pub use crate::rate_ctrl::estimate_tx_duration_us;
pub use crate::rate_ctrl::RateController;

pub use crate::regs::WifiPhyRate;

pub use crate::smoltcp_phy::DriverError;
pub use crate::smoltcp_phy::Esp32StaWifiDriver;
pub use crate::smoltcp_phy::StaRxToken;
pub use crate::smoltcp_phy::StaTxToken;

pub use crate::sta_fsm::ScanCandidate;
pub use crate::sta_fsm::StaConnectConfig;
pub use crate::sta_fsm::StaError;
pub use crate::sta_fsm::StaMacCore;
pub use crate::sta_fsm::StaState;
pub use crate::sta_fsm::StaTxSink;
