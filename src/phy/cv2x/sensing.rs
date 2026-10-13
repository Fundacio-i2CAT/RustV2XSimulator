//! 3GPP Sidelink Sensing Window Manager.
//!
//! Maintains historical spectrum measurements over the 1,000 ms sliding sensing window
//! per 3GPP TS 36.213 Section 14.1.1.6:
//! - Tracks decoded SCIs from nearby transmitters (RSRP and reservation intervals)
//! - Tracks S-RSSI energy across all sub-channels
//! - Tracks half-duplex transmission history (subframes where the node itself transmitted)
//! - Computes Sidelink Channel Busy Ratio (CBR_SL) over the last 100 subframes per TS 36.321

use std::collections::{HashMap, VecDeque};
use crate::phy::cv2x::config::Cv2xConfig;
use crate::phy::cv2x::resource_grid::CandidateResource;
use crate::phy::cv2x::sci::SidelinkControlInfo;

/// Sidelink measurement record for a received SCI transmission.
#[derive(Debug, Clone)]
pub struct SciMeasurement {
    pub sci: SidelinkControlInfo,
    /// Measured Reference Signal Received Power in dBm on PSSCH DMRS
    pub rsrp_dbm: f64,
    /// Measured Sidelink Received Signal Strength Indicator in dBm
    pub rssi_dbm: f64,
}

/// Sidelink sensing manager maintaining the 1,000 ms sliding observation buffer.
#[derive(Debug, Clone)]
pub struct SidelinkSensingManager {
    pub config: Cv2xConfig,
    /// Subframes during which the local vehicle was transmitting (half-duplex exclusion)
    pub local_tx_history: VecDeque<CandidateResource>,
    /// Historical SCIs decoded from surrounding vehicles, indexed by subframe
    pub received_scis: HashMap<u64, Vec<SciMeasurement>>,
    /// S-RSSI energy map in dBm: (subframe, subchannel) -> rssi_dbm
    pub rssi_history: HashMap<CandidateResource, f64>,
}

impl SidelinkSensingManager {
    pub fn new(config: Cv2xConfig) -> Self {
        Self {
            config,
            local_tx_history: VecDeque::new(),
            received_scis: HashMap::new(),
            rssi_history: HashMap::new(),
        }
    }

    /// Record that the local station transmitted on (subframe, subchannel).
    pub fn record_local_tx(&mut self, subframe: u64, subchannel: usize) {
        self.local_tx_history.push_back(CandidateResource::new(subframe, subchannel));
    }

    /// Record a successfully decoded SCI from another vehicle with its measured RSRP and RSSI.
    pub fn record_received_sci(&mut self, sci: SidelinkControlInfo, rsrp_dbm: f64, rssi_dbm: f64) {
        let entry = self.received_scis.entry(sci.tx_subframe).or_default();
        entry.push(SciMeasurement { sci, rsrp_dbm, rssi_dbm });
        self.rssi_history.insert(CandidateResource::new(sci.tx_subframe, sci.subchannel_index), rssi_dbm);
    }

    /// Record background S-RSSI for a sub-channel in a given subframe.
    pub fn record_subchannel_rssi(&mut self, subframe: u64, subchannel: usize, rssi_dbm: f64) {
        self.rssi_history.insert(CandidateResource::new(subframe, subchannel), rssi_dbm);
    }

    /// Check if the local station was transmitting at subframe `sf` (half-duplex restriction).
    #[inline]
    pub fn was_transmitting_at(&self, sf: u64) -> bool {
        self.local_tx_history.iter().any(|csr| csr.subframe == sf)
    }

    /// Purge historical measurements older than the 1,000 subframe sensing window.
    pub fn purge_older_than(&mut self, min_subframe: u64) {
        while let Some(front) = self.local_tx_history.front() {
            if front.subframe < min_subframe {
                self.local_tx_history.pop_front();
            } else {
                break;
            }
        }

        self.received_scis.retain(|&sf, _| sf >= min_subframe);
        self.rssi_history.retain(|csr, _| csr.subframe >= min_subframe);
    }

    /// Compute Sidelink Channel Busy Ratio (CBR_SL) over the previous 100 subframes [now - 100, now - 1]
    /// per 3GPP TS 36.321 Section 5.14.1.1.
    ///
    /// Defined as the fraction of sub-channels whose S-RSSI exceeds the busy threshold (-94.0 dBm).
    pub fn evaluate_cbr(&self, current_subframe: u64) -> f64 {
        if current_subframe < 100 {
            return 0.0;
        }

        let start_sf = current_subframe - 100;
        let end_sf = current_subframe - 1;
        let num_sc = self.config.num_subchannels();
        let total_subchannel_slots = 100 * num_sc;
        let busy_threshold_dbm = -94.0;

        let mut busy_count = 0usize;
        for sf in start_sf..=end_sf {
            for sc in 0..num_sc {
                let csr = CandidateResource::new(sf, sc);
                let rssi = self.rssi_history.get(&csr).copied().unwrap_or(-115.0);
                if rssi >= busy_threshold_dbm {
                    busy_count += 1;
                }
            }
        }

        (busy_count as f64) / (total_subchannel_slots as f64).max(1.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sensing_cbr_evaluation() {
        let cfg = Cv2xConfig::default();
        let mut sensing = SidelinkSensingManager::new(cfg);

        // Mark 50 out of 500 subchannels as busy
        for sf in 100..150 {
            sensing.record_subchannel_rssi(sf, 0, -85.0); // busy
        }

        let cbr = sensing.evaluate_cbr(200);
        // 50 busy out of 500 total = 0.10 CBR
        assert!((cbr - 0.10).abs() < 1e-3);
    }
}
