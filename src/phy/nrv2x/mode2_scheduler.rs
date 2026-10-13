//! 3GPP Release 16/17 Sidelink Mode 2 Autonomous Scheduler.
//!
//! Standards Reference: 3GPP TS 38.213 Section 16.6 (UE Procedure for Sidelink Resource Allocation Mode 2)
//! Parity Reference: 5G-LENA (`nr-sl-ue-mac.cc: SchedNrSlTriggerReq`, `ExcludeReservedResources`)

use rand::Rng;
use serde::{Deserialize, Serialize};

use super::config::NrV2xConfig;
use super::resource_grid::{NrResource, NrResourceGrid};
use super::sensing::NrSensingManager;

/// State of an allocated Sidelink Mode 2 Semi-Persistent Scheduling (SPS) or Dynamic grant.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct NrMode2Grant {
    /// Allocated candidate single-slot resource
    pub resource: NrResource,
    /// Reservation interval in slots (e.g. 200 slots = 100 ms at mu=1)
    pub interval_slots: u64,
    /// Reselection counter C_resel
    pub reselection_counter: u32,
    /// Priority of the packet using this grant (0 to 7)
    pub priority: u8,
    /// Slot at which the grant was initially selected (for re-evaluation tracking)
    pub selected_at_slot: u64,
}

/// Sidelink Mode 2 Autonomous MAC Scheduler per 3GPP TS 38.213 Section 16.6.
#[derive(Debug, Clone)]
pub struct NrMode2Scheduler {
    pub config: NrV2xConfig,
    pub grid: NrResourceGrid,
    pub active_grant: Option<NrMode2Grant>,
}

impl NrMode2Scheduler {
    pub fn new(config: NrV2xConfig) -> Self {
        let grid = NrResourceGrid::new(
            config.numerology,
            config.num_subchannels,
            config.subchannel_size_rbs,
        );
        Self {
            config,
            grid,
            active_grant: None,
        }
    }

    /// Schedule or allocate a transmission resource for an incoming packet arriving at `current_slot`.
    ///
    /// Implements Release 16 Mode 2 selection, SPS reuse, and Re-evaluation ($T_3$ lookahead).
    pub fn schedule_packet<R: Rng>(
        &mut self,
        current_slot: u64,
        priority: u8,
        req_subchannels: u8,
        sensing: &NrSensingManager,
        rng: &mut R,
    ) -> NrResource {
        // 1. Check if active grant can be reused
        if let Some(grant) = self.active_grant {
            let target_slot = grant.resource.slot;
            let deadline_slot = current_slot + self.config.t2_max_slots;

            if target_slot >= current_slot && target_slot <= deadline_slot && grant.reselection_counter > 0 {
                // Perform Release 16 Re-evaluation Check at slot m - T3
                if self.config.enable_reevaluation && current_slot + self.config.t3_reeval_slots >= target_slot {
                    let is_safe = self.check_reevaluation(current_slot, &grant, sensing);
                    if !is_safe {
                        // Collision detected prior to transmission! Drop and reselect immediately
                        let new_res = self.reselect_resource(current_slot, priority, req_subchannels, sensing, rng);
                        let c_resel = rng.gen_range(self.config.c_resel_min..=self.config.c_resel_max);
                        self.active_grant = Some(NrMode2Grant {
                            resource: new_res,
                            interval_slots: self.grid.millis_to_slots(self.config.p_rsvp_ms),
                            reselection_counter: c_resel,
                            priority,
                            selected_at_slot: current_slot,
                        });
                        return new_res;
                    }
                }

                // Grant is valid and verified!
                return grant.resource;
            }
        }

        // 2. Trigger TS 38.213 Section 16.6.1 Resource Reselection
        let selected_resource = self.reselect_resource(current_slot, priority, req_subchannels, sensing, rng);

        // Draw new Reselection Counter C_resel
        let c_resel = rng.gen_range(self.config.c_resel_min..=self.config.c_resel_max);
        let interval_slots = self.grid.millis_to_slots(self.config.p_rsvp_ms);

        self.active_grant = Some(NrMode2Grant {
            resource: selected_resource,
            interval_slots,
            reselection_counter: c_resel,
            priority,
            selected_at_slot: current_slot,
        });

        selected_resource
    }

    /// Advance active grant following transmission completion.
    /// Evaluates probability P_keep whether to maintain the resource or release it.
    pub fn on_transmission_completed<R: Rng>(&mut self, rng: &mut R) {
        if let Some(ref mut grant) = self.active_grant {
            if grant.reselection_counter > 0 {
                grant.reselection_counter -= 1;
            }

            if grant.reselection_counter == 0 {
                let u: f64 = rng.gen();
                if u <= self.config.prob_resource_keep {
                    // Maintain grant
                    grant.reselection_counter =
                        rng.gen_range(self.config.c_resel_min..=self.config.c_resel_max);
                    grant.resource.slot += grant.interval_slots;
                } else {
                    // Release grant
                    self.active_grant = None;
                }
            } else {
                // Advance to next periodic slot
                grant.resource.slot += grant.interval_slots;
            }
        }
    }

    /// Execute the 3GPP TS 38.213 Section 16.6.1 Mode 2 Resource Selection Algorithm.
    ///
    /// Direct parity with 5G-LENA `nr-sl-ue-mac.cc` (lines 510-640).
    pub fn reselect_resource<R: Rng>(
        &self,
        current_slot: u64,
        priority: u8,
        req_subchannels: u8,
        sensing: &NrSensingManager,
        rng: &mut R,
    ) -> NrResource {
        // Enumerate all candidate single-slot resources within selection window [n + T1, n + T2]
        let all_candidates = self.grid.candidate_resources_in_window(
            current_slot,
            self.config.t1_min_slots,
            self.config.t2_max_slots,
            req_subchannels,
        );

        let total_candidates = all_candidates.len();
        if total_candidates == 0 {
            return NrResource::new(current_slot + self.config.t1_min_slots, 0, req_subchannels);
        }

        let p_rsvp_slots = self.grid.millis_to_slots(self.config.p_rsvp_ms);
        let mut current_rsrp_th = self.config.rsrp_threshold_dbm;
        let mut eligible_candidates: Vec<NrResource> = Vec::new();
        let target_ratio = self.config.min_candidate_pct / 100.0;

        // Step 1: Iterative exclusion with +3 dB threshold backoff until >= X% remain
        for _ in 0..10 {
            eligible_candidates.clear();

            for &cand in &all_candidates {
                // Check 1: Half-duplex self-exclusion
                let mut half_duplex_collision = false;
                if p_rsvp_slots > 0 {
                    let mut past_slot = cand.slot.saturating_sub(p_rsvp_slots);
                    let sensing_start = current_slot.saturating_sub(sensing.sensing_window_slots);
                    while past_slot >= sensing_start {
                        if sensing.was_transmitting_at(past_slot) {
                            half_duplex_collision = true;
                            break;
                        }
                        if past_slot < p_rsvp_slots {
                            break;
                        }
                        past_slot -= p_rsvp_slots;
                    }
                }

                if half_duplex_collision {
                    continue;
                }

                // Check 2: Exclude based on sensed SCIs and RSRP threshold
                let mut reserved_and_high_rsrp = false;
                for (&tx_slot, sci_list) in &sensing.received_scis {
                    for meas in sci_list {
                        let rri_ms = meas.sci.reservation_period_ms as u64;
                        if rri_ms == 0 {
                            continue;
                        }
                        let rri_slots = self.grid.millis_to_slots(rri_ms);
                        if rri_slots == 0 {
                            continue;
                        }

                        // Project periodic reservations of the sender
                        let mut proj_slot = tx_slot + rri_slots;
                        while proj_slot <= cand.slot {
                            if proj_slot == cand.slot {
                                // Overlaps in time! Check RSRP and frequency overlap
                                if meas.rsrp_dbm >= current_rsrp_th {
                                    if cand.overlaps_frequency(
                                        meas.sci.start_subchannel,
                                        meas.sci.num_subchannels,
                                    ) {
                                        // Release 16 Pre-emption check:
                                        // If local packet has higher priority (lower numerical index)
                                        // than the interfering packet, do NOT exclude (pre-empt)!
                                        let is_preempted = self.config.enable_preemption
                                            && (priority < meas.sci.priority);

                                        if !is_preempted {
                                            reserved_and_high_rsrp = true;
                                            break;
                                        }
                                    }
                                }
                            }
                            proj_slot += rri_slots;
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
                    eligible_candidates.push(cand);
                }
            }

            // Step 2: Verify X% candidate pool condition
            let remaining_ratio = (eligible_candidates.len() as f64) / (total_candidates as f64);
            if remaining_ratio >= target_ratio || current_rsrp_th >= -70.0 {
                break;
            } else {
                current_rsrp_th += 3.0; // Increase threshold by 3 dB
            }
        }

        if eligible_candidates.is_empty() {
            let idx = rng.gen_range(0..all_candidates.len());
            return all_candidates[idx];
        }

        // Step 3: Energy ranking (S-RSSI) to select lowest X% subset (Set S_B)
        let mut cand_energy: Vec<(NrResource, f64)> = eligible_candidates
            .into_iter()
            .map(|cand| {
                let mut sum_rssi = 0.0;
                let mut count = 0;
                if p_rsvp_slots > 0 {
                    let mut past_slot = cand.slot.saturating_sub(p_rsvp_slots);
                    let sensing_start = current_slot.saturating_sub(sensing.sensing_window_slots);
                    while past_slot >= sensing_start {
                        let rssi = sensing
                            .rssi_history
                            .get(&(past_slot, cand.start_subchannel))
                            .copied()
                            .unwrap_or(-115.0);
                        sum_rssi += rssi;
                        count += 1;
                        if past_slot < p_rsvp_slots {
                            break;
                        }
                        past_slot -= p_rsvp_slots;
                    }
                }
                let avg_rssi = if count > 0 { sum_rssi / (count as f64) } else { -115.0 };
                (cand, avg_rssi)
            })
            .collect();

        cand_energy.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));

        let subset_size = ((total_candidates as f64) * target_ratio).ceil() as usize;
        let s_b_size = subset_size.clamp(1, cand_energy.len());
        let s_b = &cand_energy[..s_b_size];

        // Step 4: Uniform random draw from lowest energy pool S_B
        let chosen_idx = rng.gen_range(0..s_b.len());
        s_b[chosen_idx].0
    }

    /// 3GPP Release 16 Re-evaluation Check ($T_3$).
    ///
    /// Evaluates if new SCIs received between initial grant selection and $m - T_3$
    /// conflict with the scheduled transmission. Returns true if grant is safe, false if conflict occurs.
    pub fn check_reevaluation(
        &self,
        current_slot: u64,
        grant: &NrMode2Grant,
        sensing: &NrSensingManager,
    ) -> bool {
        let target_slot = grant.resource.slot;

        // Check SCIs received after the grant was selected
        for (&rx_slot, sci_list) in &sensing.received_scis {
            if rx_slot > grant.selected_at_slot && rx_slot <= current_slot {
                for meas in sci_list {
                    let mut is_conflict = false;

                    // Case 1: Direct reservation via gap_retx1_slots
                    if meas.sci.gap_retx1_slots > 0
                        && (rx_slot + meas.sci.gap_retx1_slots as u64) == target_slot
                    {
                        is_conflict = true;
                    }

                    // Case 2: Periodic reservation via reservation_period_ms
                    let rri_ms = meas.sci.reservation_period_ms as u64;
                    if rri_ms > 0 {
                        let rri_slots = self.grid.millis_to_slots(rri_ms);
                        if rri_slots > 0 {
                            let mut proj_slot = rx_slot + rri_slots;
                            while proj_slot <= target_slot {
                                if proj_slot == target_slot {
                                    is_conflict = true;
                                    break;
                                }
                                proj_slot += rri_slots;
                            }
                        }
                    }

                    if is_conflict && meas.rsrp_dbm >= self.config.rsrp_threshold_dbm {
                        if grant.resource.overlaps_frequency(
                            meas.sci.start_subchannel,
                            meas.sci.num_subchannels,
                        ) {
                            // Conflict detected!
                            return false;
                        }
                    }
                }
            }
        }

        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rand::SeedableRng;
    use rand::rngs::StdRng;
    use super::super::sci::SciFormat1A;

    #[test]
    fn test_mode2_initial_selection() {
        let cfg = NrV2xConfig::default();
        let mut sched = NrMode2Scheduler::new(cfg.clone());
        let sensing = NrSensingManager::new(cfg);
        let mut rng = StdRng::seed_from_u64(42);

        let res = sched.schedule_packet(10, 3, 1, &sensing, &mut rng);
        assert!(res.slot >= 12); // T1 = 2
        assert!(res.slot <= 42); // T2 = 32
        assert!(sched.active_grant.is_some());
    }

    #[test]
    fn test_mode2_reevaluation_drop() {
        let mut cfg = NrV2xConfig::default();
        cfg.t1_min_slots = 10;
        cfg.t2_max_slots = 40;
        let mut sched = NrMode2Scheduler::new(cfg.clone());
        let mut sensing = NrSensingManager::new(cfg);
        let mut rng = StdRng::seed_from_u64(42);

        let res = sched.schedule_packet(10, 3, 1, &sensing, &mut rng);
        let target_slot = res.slot;

        let mut sci = SciFormat1A::new(1, 0, res.start_subchannel, 1, 5, 10);
        sci.gap_retx1_slots = (target_slot - 15) as u8;
        sensing.record_received_sci(15, sci, -70.0, -70.0);

        let check_slot = target_slot - 2;
        let is_safe = sched.check_reevaluation(check_slot, &sched.active_grant.unwrap(), &sensing);
        assert!(!is_safe, "Re-evaluation must detect high-RSRP reservation conflict");
    }
}
