//! 3GPP Release 16/17 Sidelink Sensing Manager.
//!
//! Standards Reference: 3GPP TS 38.214 Section 8.1.4 & TS 38.213 Section 16.6
//! Parity Reference: 5G-LENA (`nr-sl-ue-mac.cc: RemoveOldSensingData`, `DoReceiveSensingData`)

use std::collections::{HashMap, HashSet};
use super::config::NrV2xConfig;
use super::resource_grid::NrResourceGrid;
use super::sci::SciFormat1A;

/// Decoded Sidelink Control Information with measured physical signal strength.
#[derive(Debug, Clone)]
pub struct NrSciMeasurement {
    pub sci: SciFormat1A,
    pub rsrp_dbm: f64,
    pub rssi_dbm: f64,
    pub rx_slot: u64,
}

/// Sidelink sensing manager maintaining sliding observation window and channel measurements.
#[derive(Debug, Clone)]
pub struct NrSensingManager {
    pub config: NrV2xConfig,
    pub grid: NrResourceGrid,
    pub sensing_window_slots: u64,
    /// History of received SCIs indexed by reception slot
    pub received_scis: HashMap<u64, Vec<NrSciMeasurement>>,
    /// Record of slots where local UE was transmitting (half-duplex blindness)
    pub local_tx_slots: HashSet<u64>,
    /// Sidelink Received Signal Strength Indicator (S-RSSI) per (slot, subchannel)
    pub rssi_history: HashMap<(u64, u8), f64>,
}

impl NrSensingManager {
    pub fn new(config: NrV2xConfig) -> Self {
        let grid = NrResourceGrid::new(
            config.numerology,
            config.num_subchannels,
            config.subchannel_size_rbs,
        );
        let sensing_window_slots = grid.millis_to_slots(config.sensing_window_ms);
        Self {
            config,
            grid,
            sensing_window_slots,
            received_scis: HashMap::new(),
            local_tx_slots: HashSet::new(),
            rssi_history: HashMap::new(),
        }
    }

    /// Record newly demodulated Stage-1 SCI and its measured RSRP/RSSI.
    pub fn record_received_sci(
        &mut self,
        rx_slot: u64,
        sci: SciFormat1A,
        rsrp_dbm: f64,
        rssi_dbm: f64,
    ) {
        // Record SCI
        self.received_scis
            .entry(rx_slot)
            .or_default()
            .push(NrSciMeasurement {
                sci,
                rsrp_dbm,
                rssi_dbm,
                rx_slot,
            });

        // Record RSSI across occupied subchannels
        for sc in sci.start_subchannel..(sci.start_subchannel + sci.num_subchannels) {
            self.rssi_history.insert((rx_slot, sc), rssi_dbm);
        }
    }

    /// Record a local transmission for half-duplex self-exclusion.
    pub fn record_local_tx(&mut self, slot: u64) {
        self.local_tx_slots.insert(slot);
    }

    /// Check if the local UE was transmitting at `slot`.
    pub fn was_transmitting_at(&self, slot: u64) -> bool {
        self.local_tx_slots.contains(&slot)
    }

    /// Prune sensing data older than the sliding sensing window [current_slot - T0, current_slot].
    /// Direct parity with 5G-LENA `RemoveOldSensingData`.
    pub fn prune_old_sensing_data(&mut self, current_slot: u64) {
        let cutoff_slot = current_slot.saturating_sub(self.sensing_window_slots);

        self.received_scis.retain(|&slot, _| slot >= cutoff_slot);
        self.local_tx_slots.retain(|&slot| slot >= cutoff_slot);
        self.rssi_history.retain(|&(slot, _), _| slot >= cutoff_slot);
    }

    /// Record background energy measurement on a subchannel
    pub fn record_subchannel_rssi(&mut self, slot: u64, subchannel: u8, rssi_dbm: f64) {
        self.rssi_history.insert((slot, subchannel), rssi_dbm);
    }

    /// Calculate Channel Busy Ratio (CBR) over the past 100 ms (TS 38.215 Section 5.1.27).
    pub fn evaluate_cbr(&self, current_slot: u64) -> f64 {
        let cbr_slots = self.grid.millis_to_slots(100);
        let start_slot = current_slot.saturating_sub(cbr_slots);
        let mut busy_count = 0;
        let mut total_count = 0;

        for slot in start_slot..=current_slot {
            for sc in 0..self.config.num_subchannels {
                total_count += 1;
                let rssi = self.rssi_history.get(&(slot, sc as u8)).copied().unwrap_or(-115.0);
                if rssi >= -85.0 {
                    // 3GPP CBR threshold: -85 dBm
                    busy_count += 1;
                }
            }
        }

        if total_count == 0 {
            0.0
        } else {
            (busy_count as f64) / (total_count as f64)
        }
    }
}
