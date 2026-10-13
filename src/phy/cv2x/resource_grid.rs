//! 3GPP LTE-V2X Sidelink Time-Frequency Resource Grid.
//!
//! Subdivides the wireless medium into discrete 2D resources:
//! - Time domain: 1 ms Subframes (TTIs), synchronized via GNSS (GPS/Galileo).
//! - Frequency domain: Sub-channels, each containing `N_RB` Resource Blocks (180 kHz each).

use serde::{Deserialize, Serialize};
use crate::common::SimTime;

/// Candidate Single-Subframe Resource (CSR) in the 2D Sidelink resource grid.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct CandidateResource {
    /// Subframe index (1 subframe = 1 ms)
    pub subframe: u64,
    /// Sub-channel index (0 .. num_subchannels - 1)
    pub subchannel: usize,
}

impl CandidateResource {
    pub fn new(subframe: u64, subchannel: usize) -> Self {
        Self { subframe, subchannel }
    }

    /// Convert a discrete simulation timestamp into the corresponding subframe index.
    #[inline]
    pub fn subframe_from_time(time: SimTime) -> u64 {
        time.to_millis()
    }

    /// Convert a subframe index back to a simulation timestamp at the subframe start boundary.
    #[inline]
    pub fn time_from_subframe(subframe: u64) -> SimTime {
        SimTime::from_millis(subframe)
    }
}

/// Sidelink Time-Frequency Resource Grid coordinate and iterator helper.
#[derive(Debug, Clone)]
pub struct SidelinkResourceGrid {
    pub num_subchannels: usize,
    pub subchannel_size_rbs: usize,
}

impl SidelinkResourceGrid {
    pub fn new(num_subchannels: usize, subchannel_size_rbs: usize) -> Self {
        Self {
            num_subchannels,
            subchannel_size_rbs,
        }
    }

    /// Enumerate all Candidate Single-Subframe Resources (CSRs) within selection window [t + t1, t + t2].
    pub fn candidate_resources_in_window(
        &self,
        current_subframe: u64,
        t1_ms: u64,
        t2_ms: u64,
    ) -> Vec<CandidateResource> {
        let start_subframe = current_subframe + t1_ms;
        let end_subframe = current_subframe + t2_ms;
        let num_subframes = (end_subframe - start_subframe + 1) as usize;
        let mut csrs = Vec::with_capacity(num_subframes * self.num_subchannels);

        for sf in start_subframe..=end_subframe {
            for sc in 0..self.num_subchannels {
                csrs.push(CandidateResource::new(sf, sc));
            }
        }

        csrs
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_grid_window_enumeration() {
        let grid = SidelinkResourceGrid::new(5, 10);
        let csrs = grid.candidate_resources_in_window(100, 4, 10);
        // Window length: 104 to 110 = 7 subframes * 5 subchannels = 35 CSRs
        assert_eq!(csrs.len(), 35);
        assert_eq!(csrs[0].subframe, 104);
        assert_eq!(csrs.last().unwrap().subframe, 110);
    }
}
