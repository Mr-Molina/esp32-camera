//! Wi-Fi rate control, rate table definitions, and TX duration estimation.
//!
//! Provides hardware PHY rate definitions matching ESP-IDF `wifi_phy_rate_t`,
//! translation to PLCP parameters, physical layer duration estimation for
//! NAV/MAC duration registers, and a rate controller for Station (STA) link
//! rate adaptation.

use crate::regs::WifiPhyRate;

/// Hardware PLCP transmission parameters corresponding to a [`WifiPhyRate`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TxPlcpRate {
    /// True if transmission uses 802.11n High Throughput (HT) mode.
    pub is_ht: bool,
    /// Hardware rate bits programmed into MAC registers / descriptors.
    pub hw_rate_bits: u8,
    /// True if transmission uses Short Guard Interval (400 ns).
    pub short_gi: bool,
    /// True if transmission uses 802.11b short preamble.
    pub is_short_preamble: bool,
    /// Data rate represented as twice the Mbps value (e.g. 1 Mbps = 2).
    pub mbps_x2: u16,
}

impl WifiPhyRate {
    /// Translates this PHY rate into hardware PLCP parameters.
    #[must_use]
    pub const fn to_plcp(self) -> TxPlcpRate {
        match self {
            Self::Rate1mL => TxPlcpRate {
                is_ht: false,
                hw_rate_bits: 0x00,
                short_gi: false,
                is_short_preamble: false,
                mbps_x2: 2,
            },
            Self::Rate2mL => TxPlcpRate {
                is_ht: false,
                hw_rate_bits: 0x01,
                short_gi: false,
                is_short_preamble: false,
                mbps_x2: 4,
            },
            Self::Rate5m5L => TxPlcpRate {
                is_ht: false,
                hw_rate_bits: 0x02,
                short_gi: false,
                is_short_preamble: false,
                mbps_x2: 11,
            },
            Self::Rate11mL => TxPlcpRate {
                is_ht: false,
                hw_rate_bits: 0x03,
                short_gi: false,
                is_short_preamble: false,
                mbps_x2: 22,
            },
            Self::Rate2mS => TxPlcpRate {
                is_ht: false,
                hw_rate_bits: 0x01,
                short_gi: false,
                is_short_preamble: true,
                mbps_x2: 4,
            },
            Self::Rate5m5S => TxPlcpRate {
                is_ht: false,
                hw_rate_bits: 0x02,
                short_gi: false,
                is_short_preamble: true,
                mbps_x2: 11,
            },
            Self::Rate11mS => TxPlcpRate {
                is_ht: false,
                hw_rate_bits: 0x03,
                short_gi: false,
                is_short_preamble: true,
                mbps_x2: 22,
            },
            Self::Rate6m => TxPlcpRate {
                is_ht: false,
                hw_rate_bits: 0x04,
                short_gi: false,
                is_short_preamble: false,
                mbps_x2: 12,
            },
            Self::Rate9m => TxPlcpRate {
                is_ht: false,
                hw_rate_bits: 0x05,
                short_gi: false,
                is_short_preamble: false,
                mbps_x2: 18,
            },
            Self::Rate12m => TxPlcpRate {
                is_ht: false,
                hw_rate_bits: 0x06,
                short_gi: false,
                is_short_preamble: false,
                mbps_x2: 24,
            },
            Self::Rate18m => TxPlcpRate {
                is_ht: false,
                hw_rate_bits: 0x07,
                short_gi: false,
                is_short_preamble: false,
                mbps_x2: 36,
            },
            Self::Rate24m => TxPlcpRate {
                is_ht: false,
                hw_rate_bits: 0x08,
                short_gi: false,
                is_short_preamble: false,
                mbps_x2: 48,
            },
            Self::Rate36m => TxPlcpRate {
                is_ht: false,
                hw_rate_bits: 0x09,
                short_gi: false,
                is_short_preamble: false,
                mbps_x2: 72,
            },
            Self::Rate48m => TxPlcpRate {
                is_ht: false,
                hw_rate_bits: 0x0A,
                short_gi: false,
                is_short_preamble: false,
                mbps_x2: 96,
            },
            Self::Rate54m => TxPlcpRate {
                is_ht: false,
                hw_rate_bits: 0x0B,
                short_gi: false,
                is_short_preamble: false,
                mbps_x2: 108,
            },
            Self::Mcs0Lgi => TxPlcpRate {
                is_ht: true,
                hw_rate_bits: 0x10,
                short_gi: false,
                is_short_preamble: false,
                mbps_x2: 13,
            },
            Self::Mcs1Lgi => TxPlcpRate {
                is_ht: true,
                hw_rate_bits: 0x11,
                short_gi: false,
                is_short_preamble: false,
                mbps_x2: 26,
            },
            Self::Mcs2Lgi => TxPlcpRate {
                is_ht: true,
                hw_rate_bits: 0x12,
                short_gi: false,
                is_short_preamble: false,
                mbps_x2: 39,
            },
            Self::Mcs3Lgi => TxPlcpRate {
                is_ht: true,
                hw_rate_bits: 0x13,
                short_gi: false,
                is_short_preamble: false,
                mbps_x2: 52,
            },
            Self::Mcs4Lgi => TxPlcpRate {
                is_ht: true,
                hw_rate_bits: 0x14,
                short_gi: false,
                is_short_preamble: false,
                mbps_x2: 78,
            },
            Self::Mcs5Lgi => TxPlcpRate {
                is_ht: true,
                hw_rate_bits: 0x15,
                short_gi: false,
                is_short_preamble: false,
                mbps_x2: 104,
            },
            Self::Mcs6Lgi => TxPlcpRate {
                is_ht: true,
                hw_rate_bits: 0x16,
                short_gi: false,
                is_short_preamble: false,
                mbps_x2: 117,
            },
            Self::Mcs7Lgi => TxPlcpRate {
                is_ht: true,
                hw_rate_bits: 0x17,
                short_gi: false,
                is_short_preamble: false,
                mbps_x2: 130,
            },
            Self::Mcs0Sgi => TxPlcpRate {
                is_ht: true,
                hw_rate_bits: 0x10,
                short_gi: true,
                is_short_preamble: false,
                mbps_x2: 14,
            },
            Self::Mcs1Sgi => TxPlcpRate {
                is_ht: true,
                hw_rate_bits: 0x11,
                short_gi: true,
                is_short_preamble: false,
                mbps_x2: 29,
            },
            Self::Mcs2Sgi => TxPlcpRate {
                is_ht: true,
                hw_rate_bits: 0x12,
                short_gi: true,
                is_short_preamble: false,
                mbps_x2: 43,
            },
            Self::Mcs3Sgi => TxPlcpRate {
                is_ht: true,
                hw_rate_bits: 0x13,
                short_gi: true,
                is_short_preamble: false,
                mbps_x2: 58,
            },
            Self::Mcs4Sgi => TxPlcpRate {
                is_ht: true,
                hw_rate_bits: 0x14,
                short_gi: true,
                is_short_preamble: false,
                mbps_x2: 87,
            },
            Self::Mcs5Sgi => TxPlcpRate {
                is_ht: true,
                hw_rate_bits: 0x15,
                short_gi: true,
                is_short_preamble: false,
                mbps_x2: 116,
            },
            Self::Mcs6Sgi => TxPlcpRate {
                is_ht: true,
                hw_rate_bits: 0x16,
                short_gi: true,
                is_short_preamble: false,
                mbps_x2: 130,
            },
            Self::Mcs7Sgi => TxPlcpRate {
                is_ht: true,
                hw_rate_bits: 0x17,
                short_gi: true,
                is_short_preamble: false,
                mbps_x2: 144,
            },
        }
    }
}

/// Calculates the physical transmission duration in microseconds.
///
/// Used to populate the hardware TX duration word register (`0x3FF7_4268`).
#[must_use]
pub fn estimate_tx_duration_us(mpdu_len: usize, rate: WifiPhyRate) -> u16 {
    let plcp = rate.to_plcp();
    let payload_bits = (mpdu_len as u32).saturating_mul(8);

    if plcp.is_ht {
        // HT20 mode: Preamble is 36 us (L-STF, L-LTF, L-SIG, HT-SIG, HT-STF, HT-LTF).
        // N_DBPS per MCS rate index (MCS0..MCS7):
        let n_dbps: u32 = match plcp.hw_rate_bits & 0x07 {
            0 => 26,
            1 => 52,
            2 => 78,
            3 => 104,
            4 => 156,
            5 => 208,
            6 => 234,
            _ => 260,
        };
        // 16 service bits + payload + 6 tail bits
        let total_bits = payload_bits.saturating_add(22);
        let n_sym = total_bits.saturating_add(n_dbps - 1) / n_dbps;

        // Symbol time: 4 us for LGI, 3.6 us for SGI
        let sym_time_ns: u32 = if plcp.short_gi { 3600 } else { 4000 };
        let data_duration_us = (n_sym.saturating_mul(sym_time_ns).saturating_add(999)) / 1000;
        let total = 36u32.saturating_add(data_duration_us);
        total.min(u16::MAX as u32) as u16
    } else {
        match rate {
            WifiPhyRate::Rate1mL
            | WifiPhyRate::Rate2mL
            | WifiPhyRate::Rate5m5L
            | WifiPhyRate::Rate11mL
            | WifiPhyRate::Rate2mS
            | WifiPhyRate::Rate5m5S
            | WifiPhyRate::Rate11mS => {
                let preamble_us: u32 = if plcp.is_short_preamble { 96 } else { 192 };
                let mbps_x2 = plcp.mbps_x2 as u32;
                let data_us =
                    (payload_bits.saturating_mul(2).saturating_add(mbps_x2 - 1)) / mbps_x2;
                preamble_us.saturating_add(data_us).min(u16::MAX as u32) as u16
            }
            _ => {
                // Legacy 802.11a/g OFDM: Preamble is 20 us (16 us preamble + 4 us signal).
                // N_DBPS is 4 * Mbps
                let mbps = (plcp.mbps_x2 as u32) / 2;
                let n_dbps = mbps.saturating_mul(4);
                let total_bits = payload_bits.saturating_add(22);
                let n_sym = total_bits.saturating_add(n_dbps - 1) / n_dbps;
                let total = 20u32.saturating_add(n_sym.saturating_mul(4));
                total.min(u16::MAX as u32) as u16
            }
        }
    }
}

/// Ordered ladder of data rates for Station rate adaptation.
const RATE_LADDER: [WifiPhyRate; 12] = [
    WifiPhyRate::Rate1mL,
    WifiPhyRate::Rate2mL,
    WifiPhyRate::Rate5m5L,
    WifiPhyRate::Rate11mL,
    WifiPhyRate::Rate6m,
    WifiPhyRate::Rate12m,
    WifiPhyRate::Rate24m,
    WifiPhyRate::Mcs2Lgi,
    WifiPhyRate::Mcs3Lgi,
    WifiPhyRate::Mcs4Lgi,
    WifiPhyRate::Mcs5Lgi,
    WifiPhyRate::Mcs7Lgi,
];

/// Adaptive rate controller tracking transmission successes and failures.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RateController {
    current_index: usize,
    consecutive_success: u16,
    consecutive_failures: u16,
}

impl Default for RateController {
    fn default() -> Self {
        Self::new()
    }
}

impl RateController {
    /// Consecutive successes required to step up transmission rate.
    pub const SUCCESS_STEP_UP_THRESHOLD: u16 = 10;
    /// Consecutive failures that trigger a rate step down.
    pub const FAILURE_STEP_DOWN_THRESHOLD: u16 = 2;

    /// Creates a new rate controller starting at a robust default rate (MCS3).
    #[must_use]
    pub const fn new() -> Self {
        Self {
            current_index: 8, // Mcs3Lgi
            consecutive_success: 0,
            consecutive_failures: 0,
        }
    }

    /// Returns the active data rate for normal payload transmissions.
    #[must_use]
    pub fn current_data_rate(&self) -> WifiPhyRate {
        RATE_LADDER[self.current_index]
    }

    /// Returns a robust management frame rate (e.g. 1 Mbps for CCK).
    #[must_use]
    pub const fn mgmt_rate(&self) -> WifiPhyRate {
        WifiPhyRate::Rate1mL
    }

    /// Records a successful frame transmission.
    pub fn on_tx_success(&mut self) {
        self.consecutive_failures = 0;
        self.consecutive_success = self.consecutive_success.saturating_add(1);
        if self.consecutive_success >= Self::SUCCESS_STEP_UP_THRESHOLD {
            self.consecutive_success = 0;
            if self.current_index + 1 < RATE_LADDER.len() {
                self.current_index += 1;
            }
        }
    }

    /// Records a failed frame transmission (e.g. ACK timeout or excessive retries).
    pub fn on_tx_failure(&mut self) {
        self.consecutive_success = 0;
        self.consecutive_failures = self.consecutive_failures.saturating_add(1);
        if self.consecutive_failures >= Self::FAILURE_STEP_DOWN_THRESHOLD {
            self.consecutive_failures = 0;
            if self.current_index > 0 {
                self.current_index -= 1;
            }
        }
    }

    /// Resets the rate controller back to the initial default state.
    pub fn reset(&mut self) {
        self.current_index = 8;
        self.consecutive_success = 0;
        self.consecutive_failures = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_to_plcp_parameters() {
        let p_1m = WifiPhyRate::Rate1mL.to_plcp();
        assert!(!p_1m.is_ht);
        assert_eq!(p_1m.hw_rate_bits, 0x00);
        assert_eq!(p_1m.mbps_x2, 2);

        let p_6m = WifiPhyRate::Rate6m.to_plcp();
        assert!(!p_6m.is_ht);
        assert_eq!(p_6m.hw_rate_bits, 0x04);
        assert_eq!(p_6m.mbps_x2, 12);

        let p_mcs7_sgi = WifiPhyRate::Mcs7Sgi.to_plcp();
        assert!(p_mcs7_sgi.is_ht);
        assert!(p_mcs7_sgi.short_gi);
        assert_eq!(p_mcs7_sgi.hw_rate_bits, 0x17);
    }

    #[test]
    fn test_tx_duration_estimation() {
        let dur_1m = estimate_tx_duration_us(100, WifiPhyRate::Rate1mL);
        assert!(dur_1m > 192);

        let dur_54m = estimate_tx_duration_us(1000, WifiPhyRate::Rate54m);
        assert!(dur_54m > 20 && dur_54m < 500);

        let dur_mcs7 = estimate_tx_duration_us(1000, WifiPhyRate::Mcs7Lgi);
        assert!(dur_mcs7 > 36 && dur_mcs7 < dur_54m * 2);
    }

    #[test]
    fn test_rate_controller_transitions() {
        let mut rc = RateController::new();
        assert_eq!(rc.current_data_rate(), WifiPhyRate::Mcs3Lgi);

        // 2 failures trigger a step down
        rc.on_tx_failure();
        assert_eq!(rc.current_data_rate(), WifiPhyRate::Mcs3Lgi);
        rc.on_tx_failure();
        assert_eq!(rc.current_data_rate(), WifiPhyRate::Mcs2Lgi);

        // 10 successes trigger a step up
        for _ in 0..10 {
            rc.on_tx_success();
        }
        assert_eq!(rc.current_data_rate(), WifiPhyRate::Mcs3Lgi);

        rc.reset();
        assert_eq!(rc.current_data_rate(), WifiPhyRate::Mcs3Lgi);
    }
}
