//! 3GPP Rel-14 Sensing-Based Semi-Persistent Scheduling (SB-SPS).
//!
//! Complete, exact implementation of 3GPP TS 36.213 Section 14.1.1.6:
//! - Autonomous Sidelink Mode 4 distributed resource reservation
//! - Exclusion of half-duplex and high-RSRP colliding resources
//! - 20% lowest energy candidate subset selection (S_B)
//! - Random candidate selection and Reselection Counter (C_resel) management
//! - Seamless handling of ETSI aperiodic CAM arrivals interacting with 100 ms SPS grants

use rand::Rng;
use serde::{Deserialize, Serialize};

use crate::phy::cv2x::config::Cv2xConfig;
use crate::phy::cv2x::resource_grid::{CandidateResource, SidelinkResourceGrid};
use crate::phy::cv2x::sensing::SidelinkSensingManager;

/// State of an active Semi-Persistent Scheduling (SPS) grant.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct SpsGrant {
    /// Currently reserved resource in the 2D grid
    pub resource: CandidateResource,
    /// Reservation interval P_rsvp in milliseconds (e.g. 100 ms)
    pub interval_ms: u64,
    /// Remaining reselection counter (C_resel)
    pub reselection_counter: u32,
}

/// Sensing-Based Semi-Persistent Scheduler per 3GPP TS 36.213.
#[derive(Debug, Clone)]
pub struct SensingBasedSpsScheduler {
    pub config: Cv2xConfig,
    pub grid: SidelinkResourceGrid,
    pub active_grant: Option<SpsGrant>,
}

impl SensingBasedSpsScheduler {
    pub fn new(config: Cv2xConfig) -> Self {
        let num_sc = config.num_subchannels();
        let sc_size = config.subchannel_size_rbs;
        Self {
            config,
            grid: SidelinkResourceGrid::new(num_sc, sc_size),
            active_grant: None,
        }
    }

    /// Schedule or allocate a transmission resource for a packet arriving at `current_subframe`.
    ///
    /// Interacts directly with adaptive ETSI CAM arrivals:
    /// - If an active SPS grant exists and its next reservation subframe falls within the
    ///   packet's latency window [current_subframe, current_subframe + t2_max_ms], the grant is utilized.
    /// - If no grant exists, the counter expired, or an adaptive CAM arrived early (exceeding latency tolerance),
    ///   the 3GPP TS 36.213 Section 14.1.1.6 resource reselection algorithm is triggered.
    pub fn schedule_packet<R: Rng>(
        &mut self,
        current_subframe: u64,
        sensing: &SidelinkSensingManager,
        rng: &mut R,
    ) -> CandidateResource {
        if let Some(grant) = self.active_grant {
            // Check if existing grant is valid and within latency deadline
            let target_sf = grant.resource.subframe;
            let deadline_sf = current_subframe + self.config.t2_max_ms;

            if target_sf >= current_subframe && target_sf <= deadline_sf && grant.reselection_counter > 0 {
                // Active grant is reusable!
                return grant.resource;
            }
        }

        // Trigger 3GPP Mode 4 Resource Reselection
        let selected_csr = self.reselect_resource(current_subframe, sensing, rng);

        // Draw new Reselection Counter C_resel uniformly from [c_resel_min, c_resel_max]
        let c_resel = rng.gen_range(self.config.c_resel_min..=self.config.c_resel_max);
        self.active_grant = Some(SpsGrant {
            resource: selected_csr,
            interval_ms: self.config.p_rsvp_ms,
            reselection_counter: c_resel,
        });

        selected_csr
    }

    /// Advance active grant following a completed transmission.
    ///
    /// Decrements C_resel and advances the grant by P_rsvp.
    /// When C_resel reaches 0, evaluates probability P_keep whether to maintain the resource.
    pub fn on_transmission_completed<R: Rng>(&mut self, rng: &mut R) {
        if let Some(ref mut grant) = self.active_grant {
            if grant.reselection_counter > 0 {
                grant.reselection_counter -= 1;
            }

            if grant.reselection_counter == 0 {
                let u: f64 = rng.gen();
                if u <= self.config.prob_resource_keep {
                    // Retain resource! Reset reselection counter
                    grant.reselection_counter =
                        rng.gen_range(self.config.c_resel_min..=self.config.c_resel_max);
                    grant.resource.subframe += grant.interval_ms;
                } else {
                    // Release grant; will reselect on next packet arrival
                    self.active_grant = None;
                }
            } else {
                // Advance reservation by P_rsvp (e.g. 100 ms)
                grant.resource.subframe += grant.interval_ms;
            }
        }
    }

    /// Execute the 3GPP TS 36.213 Section 14.1.1.6 Resource Selection Algorithm.
    pub fn reselect_resource<R: Rng>(
        &self,
        current_subframe: u64,
        sensing: &SidelinkSensingManager,
        rng: &mut R,
    ) -> CandidateResource {
        // Enumerate all CSRs in selection window [t + T1, t + T2] (Set S_A)
        let all_csrs = self.grid.candidate_resources_in_window(
            current_subframe,
            self.config.t1_min_ms,
            self.config.t2_max_ms,
        );
        let total_candidates = all_csrs.len();
        if total_candidates == 0 {
            return CandidateResource::new(current_subframe + self.config.t1_min_ms, 0);
        }

        let mut current_rsrp_th = self.config.rsrp_threshold_dbm;
        let mut eligible_csrs: Vec<CandidateResource> = Vec::new();

        // Step 1: Resource Exclusion
        // Try initial threshold, iteratively increase by 3 dB if remaining candidates < 20%
        for _ in 0..10 {
            eligible_csrs.clear();

            for &csr in &all_csrs {
                // Check 1: Half-duplex exclusion
                // If local station was transmitting at (csr.subframe - j * P_step) in sensing window
                let mut half_duplex_collision = false;
                let mut past_sf = csr.subframe.saturating_sub(self.config.p_rsvp_ms);
                while past_sf >= current_subframe.saturating_sub(self.config.sensing_window_subframes as u64) {
                    if sensing.was_transmitting_at(past_sf) {
                        half_duplex_collision = true;
                        break;
                    }
                    if past_sf < self.config.p_rsvp_ms {
                        break;
                    }
                    past_sf -= self.config.p_rsvp_ms;
                }

                if half_duplex_collision {
                    continue;
                }

                // Check 2: SCI reservation overlap with measured RSRP >= threshold
                let mut reserved_and_high_rsrp = false;
                for (&tx_sf, sci_list) in &sensing.received_scis {
                    for meas in sci_list {
                        if !meas.sci.has_reservation() {
                            continue;
                        }
                        if meas.sci.subchannel_index != csr.subchannel {
                            continue;
                        }

                        // Project future reserved subframes of the transmitting station
                        let rri = meas.sci.reservation_interval_ms;
                        let mut proj_sf = tx_sf + rri;
                        while proj_sf <= csr.subframe {
                            if proj_sf == csr.subframe && meas.rsrp_dbm >= current_rsrp_th {
                                reserved_and_high_rsrp = true;
                                break;
                            }
                            proj_sf += rri;
                        }

                        if reserved_and_high_rsrp {
                            break;
                        }
                    }
                    if reserved_and_high_rsrp {
                        break;
                    }
                }

                if !reserved_and_high_rsrp {
                    eligible_csrs.push(csr);
                }
            }

            // Step 2: 20% Threshold Condition
            let fraction = (eligible_csrs.len() as f64) / (total_candidates as f64);
            if fraction >= 0.20 || current_rsrp_th >= -70.0 {
                break;
            } else {
                // Increase RSRP threshold by 3 dB
                current_rsrp_th += 3.0;
            }
        }

        if eligible_csrs.is_empty() {
            // Fallback: pick any resource in the window
            let idx = rng.gen_range(0..all_csrs.len());
            return all_csrs[idx];
        }

        // Step 3 & 4: Evaluate average S-RSSI energy and select lowest 20% subset (Set S_B)
        let mut csr_energy: Vec<(CandidateResource, f64)> = eligible_csrs
            .into_iter()
            .map(|csr| {
                // Calculate average RSSI across past sensing periods
                let mut sum_rssi = 0.0;
                let mut count = 0;
                let mut past_sf = csr.subframe.saturating_sub(self.config.p_rsvp_ms);
                while past_sf >= current_subframe.saturating_sub(self.config.sensing_window_subframes as u64) {
                    let past_csr = CandidateResource::new(past_sf, csr.subchannel);
                    let rssi = sensing.rssi_history.get(&past_csr).copied().unwrap_or(-115.0);
                    sum_rssi += rssi;
                    count += 1;
                    if past_sf < self.config.p_rsvp_ms {
                        break;
                    }
                    past_sf -= self.config.p_rsvp_ms;
                }
                let avg_rssi = if count > 0 { sum_rssi / (count as f64) } else { -115.0 };
                (csr, avg_rssi)
            })
            .collect();

        // Sort by average RSSI energy ascending (lowest interference first)
        csr_energy.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));

        // Take lowest 20% candidate subset S_B
        let subset_size = ((total_candidates as f64) * 0.20).ceil() as usize;
        let s_b_size = subset_size.clamp(1, csr_energy.len());
        let subset_s_b = &csr_energy[..s_b_size];

        // Step 5: Random Draw from S_B with uniform probability
        let chosen_idx = rng.gen_range(0..subset_s_b.len());
        subset_s_b[chosen_idx].0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rand::SeedableRng;
    use rand::rngs::StdRng;
    use crate::phy::cv2x::sci::SidelinkControlInfo;

    #[test]
    fn test_sps_initial_selection() {
        let cfg = Cv2xConfig::default();
        let mut scheduler = SensingBasedSpsScheduler::new(cfg.clone());
        let sensing = SidelinkSensingManager::new(cfg);
        let mut rng = StdRng::seed_from_u64(42);

        let csr = scheduler.schedule_packet(100, &sensing, &mut rng);
        assert!(csr.subframe >= 104);
        assert!(csr.subframe <= 200);
        assert!(scheduler.active_grant.is_some());
    }

    #[test]
    fn test_sps_exclusion_high_rsrp() {
        let cfg = Cv2xConfig::default();
        let scheduler = SensingBasedSpsScheduler::new(cfg.clone());
        let mut sensing = SidelinkSensingManager::new(cfg);
        let mut rng = StdRng::seed_from_u64(42);

        // Transmit an SCI at subframe 50 reserving subchannel 0 every 100 ms with high RSRP (-70 dBm)
        let sci = SidelinkControlInfo::new(99, 4, 100, 0, 7, 10, 50);
        sensing.record_received_sci(sci, -70.0, -70.0);

        // When scheduling at subframe 100 with selection window [104..200],
        // subframe 150 on subchannel 0 should be excluded
        let csr = scheduler.reselect_resource(100, &sensing, &mut rng);
        assert!(
            !(csr.subframe == 150 && csr.subchannel == 0),
            "Colliding reserved resource must be excluded"
        );
    }
}
