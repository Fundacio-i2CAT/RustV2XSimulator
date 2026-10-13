//! C-V2X PC5 Mode 4 Configuration Parameters.
//!
//! Standards-compliant configuration per 3GPP Release 14 (LTE-V2X Sidelink):
//! - 3GPP TS 36.213: Physical layer procedures (Sensing-Based SPS)
//! - 3GPP TS 36.321: Medium Access Control (MAC) protocol specification
//! - 3GPP TS 36.211: Physical channels and modulation

use serde::{Deserialize, Serialize};

/// C-V2X PC5 Sidelink Mode 4 Configuration.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Cv2xConfig {
    /// Channel carrier bandwidth in MHz (10.0 MHz or 20.0 MHz, default 10.0 MHz)
    pub bandwidth_mhz: f64,
    /// Number of Resource Blocks (RBs) per Sub-channel (typically 10 RBs = 1.8 MHz)
    pub subchannel_size_rbs: usize,
    /// Carrier center frequency in Hertz (default 5.905 GHz, 3GPP Band 47)
    pub carrier_frequency_hz: f64,
    /// Sidelink transmit power in dBm (default 23.0 dBm = 200 mW)
    pub tx_power_dbm: f64,
    /// Sliding sensing window length in subframes (1,000 subframes = 1.0 second per TS 36.213)
    pub sensing_window_subframes: usize,
    /// Minimum selection window offset T1 in milliseconds (1 <= T1 <= 4 ms, default 4 ms)
    pub t1_min_ms: u64,
    /// Maximum selection window offset T2 in milliseconds (20 <= T2 <= 100 ms, default 100 ms)
    pub t2_max_ms: u64,
    /// Nominal Resource Reservation Interval (RRI / P_rsvp) in milliseconds (default 100 ms for 10 Hz CAM)
    pub p_rsvp_ms: u64,
    /// Probability of keeping resource upon counter expiration (P_keep in [0.2, 0.8], default 0.4)
    pub prob_resource_keep: f64,
    /// Minimum resource reselection counter C_resel (default 5)
    pub c_resel_min: u32,
    /// Maximum resource reselection counter C_resel (default 15)
    pub c_resel_max: u32,
    /// Initial PSSCH-RSRP threshold in dBm for candidate resource exclusion (default -110.0 dBm)
    pub rsrp_threshold_dbm: f64,
    /// Modulation and Coding Scheme (MCS) index for PSSCH transport block (default 7: QPSK rate 0.58)
    pub pssch_mcs: u8,
}

impl Default for Cv2xConfig {
    fn default() -> Self {
        Self {
            bandwidth_mhz: 10.0, // Standard 10 MHz channel
            subchannel_size_rbs: 10, // 10 RBs per sub-channel
            carrier_frequency_hz: 5.905e9, // 3GPP Band 47
            tx_power_dbm: 23.0, // 200 mW
            sensing_window_subframes: 1000, // 1000 ms sliding window
            t1_min_ms: 4, // 4 ms lower selection bound
            t2_max_ms: 100, // 100 ms packet latency deadline
            p_rsvp_ms: 100, // 100 ms nominal SPS reservation (10 Hz CAM)
            prob_resource_keep: 0.4, // P_keep = 0.4
            c_resel_min: 5,
            c_resel_max: 15,
            rsrp_threshold_dbm: -110.0,
            pssch_mcs: 7, // Standard QPSK sidelink default
        }
    }
}

impl Cv2xConfig {
    /// Return the total number of Resource Blocks (RBs) for the configured channel bandwidth.
    /// - 10 MHz: 50 RBs
    /// - 20 MHz: 100 RBs
    #[inline]
    pub fn total_resource_blocks(&self) -> usize {
        if self.bandwidth_mhz >= 19.0 {
            100
        } else {
            50
        }
    }

    /// Return the number of active sub-channels available in the frequency domain.
    ///
    /// For 10 MHz with 10 RBs per sub-channel: 50 / 10 = 5 sub-channels.
    /// For 20 MHz with 10 RBs per sub-channel: 100 / 10 = 10 sub-channels.
    #[inline]
    pub fn num_subchannels(&self) -> usize {
        self.total_resource_blocks() / self.subchannel_size_rbs.max(1)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_10mhz_channel_parameters() {
        let cfg = Cv2xConfig::default();
        assert_eq!(cfg.total_resource_blocks(), 50);
        assert_eq!(cfg.num_subchannels(), 5);
    }

    #[test]
    fn test_20mhz_channel_parameters() {
        let mut cfg = Cv2xConfig::default();
        cfg.bandwidth_mhz = 20.0;
        assert_eq!(cfg.total_resource_blocks(), 100);
        assert_eq!(cfg.num_subchannels(), 10);
    }
}
