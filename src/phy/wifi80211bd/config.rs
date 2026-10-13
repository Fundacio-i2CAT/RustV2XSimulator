//! IEEE 802.11bd Configuration Options.

use serde::{Deserialize, Serialize};
use crate::phy::wifi80211bd::consts_80211bd::{Mcs80211bd, MidamblePeriod};
use crate::phy::wifi80211bd::doppler_model::DEFAULT_CARRIER_FREQ_HZ;

/// Configuration options for IEEE 802.11bd Next Generation V2X.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Wifi80211bdConfig {
    /// Channel bandwidth in MHz (10.0 for standard ITS, 20.0 for channel bonding)
    pub bandwidth_mhz: f64,
    /// Center carrier frequency in Hz (default 5.890 GHz)
    pub carrier_freq_hz: f64,
    /// Active Modulation and Coding Scheme (MCS 0 to MCS 9)
    pub mcs: Mcs80211bd,
    /// Midamble periodicity for vehicular Doppler channel estimation
    pub midamble_period: MidamblePeriod,
    /// Dual Carrier Modulation (DCM) frequency diversity flag (applicable to MCS 0)
    pub use_dcm: bool,
    /// Transmit power in dBm (default 23.0 dBm = 200 mW per ETSI EN 302 571)
    pub tx_power_dbm: f64,
    /// Clear Channel Assessment (CCA) threshold in dBm
    pub cca_threshold_dbm: f64,
    /// Receiver power sensitivity cutoff in dBm
    pub min_power_dbm: f64,
    /// Pure ALOHA mode bypass (for testing/benchmarks)
    pub is_aloha: bool,
}

impl Default for Wifi80211bdConfig {
    fn default() -> Self {
        Self {
            bandwidth_mhz: 10.0,
            carrier_freq_hz: DEFAULT_CARRIER_FREQ_HZ,
            mcs: Mcs80211bd::Mcs2Qpsk3_4, // 9 Mbps default (equivalent to 802.11p 6 Mbps range but higher throughput)
            midamble_period: MidamblePeriod::Every8Symbols,
            use_dcm: false,
            tx_power_dbm: 23.0,
            cca_threshold_dbm: -85.0,
            min_power_dbm: -89.0,
            is_aloha: false,
        }
    }
}

impl Wifi80211bdConfig {
    /// Create a high-throughput 20 MHz configuration (e.g. for raw sensor data sharing / platooning).
    pub fn wideband_20mhz() -> Self {
        Self {
            bandwidth_mhz: 20.0,
            carrier_freq_hz: DEFAULT_CARRIER_FREQ_HZ,
            mcs: Mcs80211bd::Mcs4Qam16_3_4, // 36 Mbps in 20 MHz
            midamble_period: MidamblePeriod::Every8Symbols,
            use_dcm: false,
            tx_power_dbm: 23.0,
            cca_threshold_dbm: -85.0,
            min_power_dbm: -86.0,
            is_aloha: false,
        }
    }

    /// Create an ultra-reliable long-range configuration (MCS 0 with DCM).
    pub fn ultra_reliable_dcm() -> Self {
        Self {
            bandwidth_mhz: 10.0,
            carrier_freq_hz: DEFAULT_CARRIER_FREQ_HZ,
            mcs: Mcs80211bd::Mcs0Bpsk1_2,
            midamble_period: MidamblePeriod::Every4Symbols,
            use_dcm: true,
            tx_power_dbm: 23.0,
            cca_threshold_dbm: -85.0,
            min_power_dbm: -89.0,
            is_aloha: false,
        }
    }
}
