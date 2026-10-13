//! 2D Time-Frequency Sidelink Resource Grid and Bandwidth Part (BWP) Pool.
//!
//! Standards Reference: 3GPP TS 38.211 Section 4 & TS 38.214 Section 8.1.4

use serde::{Deserialize, Serialize};
use super::consts_nrv2x::NrNumerology;
use crate::common::SimTime;

/// Candidate Single-Slot Resource (CSR) in the NR Sidelink time-frequency grid.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct NrResource {
    /// Slot index in the simulation timeline
    pub slot: u64,
    /// Starting subchannel index (0 to num_subchannels - 1)
    pub start_subchannel: u8,
    /// Number of contiguous subchannels occupied (default 1)
    pub num_subchannels: u8,
}

impl NrResource {
    pub fn new(slot: u64, start_subchannel: u8, num_subchannels: u8) -> Self {
        Self {
            slot,
            start_subchannel,
            num_subchannels,
        }
    }

    /// Check if this resource overlaps in frequency with another resource in the same slot
    pub fn overlaps_frequency(&self, other_start: u8, other_length: u8) -> bool {
        let self_end = self.start_subchannel + self.num_subchannels;
        let other_end = other_start + other_length;
        !(self_end <= other_start || other_end <= self.start_subchannel)
    }
}

/// Sidelink Bandwidth Part (SL-BWP) Resource Pool representation.
#[derive(Debug, Clone)]
pub struct NrResourceGrid {
    pub numerology: NrNumerology,
    pub num_subchannels: usize,
    pub subchannel_size_rbs: usize,
}

impl NrResourceGrid {
    pub fn new(numerology: NrNumerology, num_subchannels: usize, subchannel_size_rbs: usize) -> Self {
        Self {
            numerology,
            num_subchannels,
            subchannel_size_rbs,
        }
    }

    /// Convert continuous simulation time to slot index
    pub fn sim_time_to_slot(&self, time: SimTime) -> u64 {
        let slot_ticks = self.numerology.slot_duration().ticks();
        time.ticks() / slot_ticks
    }

    /// Convert slot index to continuous simulation start timestamp
    pub fn slot_to_sim_time(&self, slot: u64) -> SimTime {
        let slot_ticks = self.numerology.slot_duration().ticks();
        SimTime::from_ticks(slot * slot_ticks)
    }

    /// Convert millisecond interval to slots based on active numerology
    pub fn millis_to_slots(&self, millis: u64) -> u64 {
        let slots_per_ms = self.numerology.slots_per_subframe() as u64;
        millis * slots_per_ms
    }

    /// Enumerate all candidate single-slot resources within selection window [current_slot + T1, current_slot + T2]
    pub fn candidate_resources_in_window(
        &self,
        current_slot: u64,
        t1_slots: u64,
        t2_slots: u64,
        req_subchannels: u8,
    ) -> Vec<NrResource> {
        let start_slot = current_slot + t1_slots;
        let end_slot = current_slot + t2_slots;
        let mut candidates = Vec::new();

        if (req_subchannels as usize) > self.num_subchannels {
            return candidates;
        }

        let max_start_sc = self.num_subchannels - (req_subchannels as usize);

        for slot in start_slot..=end_slot {
            for sc in 0..=max_start_sc {
                candidates.push(NrResource::new(slot, sc as u8, req_subchannels));
            }
        }

        candidates
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_grid_slot_conversions() {
        let grid = NrResourceGrid::new(NrNumerology::Mu1_30kHz, 5, 10);
        // mu = 1 -> 500 us per slot
        assert_eq!(grid.sim_time_to_slot(SimTime::from_millis(1)), 2);
        assert_eq!(grid.sim_time_to_slot(SimTime::from_millis(10)), 20);
        assert_eq!(grid.slot_to_sim_time(20), SimTime::from_millis(10));
    }

    #[test]
    fn test_resource_frequency_overlap() {
        let res1 = NrResource::new(10, 0, 2); // occupies subchannel 0, 1
        assert!(res1.overlaps_frequency(1, 1)); // subchannel 1 overlaps!
        assert!(!res1.overlaps_frequency(2, 2)); // subchannel 2, 3 does not overlap
    }
}
