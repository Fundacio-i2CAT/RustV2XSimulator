//! Physical Layer Radio Medium and parallel link evaluation via Rayon.

use rand::rngs::StdRng;
use rand::SeedableRng;
use rayon::prelude::*;
use serde::{Deserialize, Serialize};

use crate::common::SimTime;
use crate::geometry::polygon::BuildingMap;
use crate::geometry::segment::Point2D;
use crate::phy::air_packet::AirPacket;
use crate::phy::interference_helper::{dbm_to_watts, CumulativeInterferenceTracker, SignalEvent};
use crate::phy::knife_edge::FresnelKnifeEdge;
use crate::phy::path_loss::{DualSlopePathLoss, LegacyPolynomialPathLoss};
use crate::scheduler::event::ActorId;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum PhyModelType {
    DualSlope,
    LegacyPoly,
}

#[derive(Debug, Clone)]
pub struct ReceptionResult {
    pub receiver_id: ActorId,
    pub is_success: bool,
    pub rx_power_dbm: f64,
    pub sinr_db: f64,
    pub is_los: bool,
    pub distance_m: f64,
    pub delivery_delay: SimTime,
}

pub struct RadioMedium {
    pub model_type: PhyModelType,
    pub dual_slope: DualSlopePathLoss,
    pub legacy_poly: LegacyPolynomialPathLoss,
    pub knife_edge: FresnelKnifeEdge,
    pub interference_tracker: CumulativeInterferenceTracker,
    pub tx_power_dbm: f64,
    pub building_attenuation_db_per_m: f64,
    pub max_building_attenuation_db: f64,
    pub active_signals: Vec<SignalEvent>,
    pub packet_history: Vec<(SimTime, Point2D, usize)>,
}

impl Default for RadioMedium {
    fn default() -> Self {
        Self::new(PhyModelType::DualSlope, 23.0)
    }
}

impl RadioMedium {
    pub fn new(model_type: PhyModelType, tx_power_dbm: f64) -> Self {
        Self {
            model_type,
            dual_slope: DualSlopePathLoss::default(),
            legacy_poly: LegacyPolynomialPathLoss::default(),
            knife_edge: FresnelKnifeEdge::default(),
            interference_tracker: CumulativeInterferenceTracker::default(),
            tx_power_dbm,
            building_attenuation_db_per_m: 0.85,
            max_building_attenuation_db: 45.0,
            active_signals: Vec::new(),
            packet_history: Vec::new(),
        }
    }

    /// Prune signals that have already completed before `current_time`.
    pub fn purge_completed_signals(&mut self, current_time: SimTime) {
        self.active_signals.retain(|s| s.end_time > current_time);
        self.packet_history.retain(|(t, _, _)| {
            current_time >= *t
                && (current_time - *t).to_secs_f64() <= self.legacy_poly.congestion_window_s
        });
    }

    /// Register a newly transmitted air packet.
    pub fn register_transmission(&mut self, packet: &AirPacket) {
        self.register_transmission_with_pos(packet, Point2D::default());
    }

    /// Register a newly transmitted air packet with its 2D geographic coordinate.
    pub fn register_transmission_with_pos(&mut self, packet: &AirPacket, sender_pos: Point2D) {
        let signal = SignalEvent {
            signal_id: packet.packet_id,
            rx_power_dbm: packet.tx_power_dbm,
            rx_power_watts: dbm_to_watts(packet.tx_power_dbm),
            start_time: packet.start_time,
            end_time: packet.end_time,
            is_desired: false,
        };
        self.active_signals.push(signal);
        self.packet_history
            .push((packet.start_time, sender_pos, packet.payload_bytes.len()));
    }

    /// Evaluate reception across all candidate receivers in parallel using Rayon.
    pub fn evaluate_broadcast(
        &self,
        packet: &AirPacket,
        sender_pos: Point2D,
        receivers: &[(ActorId, Point2D, f64)], // (id, pos, distance)
        buildings: &BuildingMap,
        sim_seed: u64,
    ) -> Vec<ReceptionResult> {
        let active_signals_snapshot = self.active_signals.clone();
        let packet_history_snapshot = self.packet_history.clone();
        let payload_len = packet.payload_bytes.len();

        receivers
            .par_iter()
            .enumerate()
            .map(|(idx, &(rx_id, rx_pos, distance_m))| {
                // Deterministic per-receiver PRNG
                let mut rng = StdRng::seed_from_u64(
                    sim_seed ^ ((packet.packet_id << 32) | (rx_id as u64)) ^ (idx as u64),
                );

                // 1. Evaluate geometric building obstruction
                let obs = buildings.evaluate_obstruction(
                    &sender_pos,
                    &rx_pos,
                    self.building_attenuation_db_per_m,
                    self.max_building_attenuation_db,
                );

                // 2. Compute delivery delay
                let delivery_delay = if self.model_type == PhyModelType::LegacyPoly
                    && self.legacy_poly.use_gaussian_delay
                {
                    let normal = rand_distr::Normal::new(
                        self.legacy_poly.mean_delay_s,
                        self.legacy_poly.std_dev_delay_s,
                    )
                    .unwrap_or_else(|_| rand_distr::Normal::new(0.02, 0.005).unwrap());
                    let delay_s = rand_distr::Distribution::sample(&normal, &mut rng)
                        .max(self.legacy_poly.min_delay_s);
                    SimTime::from_secs_f64(delay_s)
                } else {
                    packet.duration + SimTime::from_micros(10)
                };

                // 3. Compute path loss according to configured PHY model
                let (rx_power_dbm, is_success_legacy) = match self.model_type {
                    PhyModelType::DualSlope => {
                        let base_pl_db = self
                            .dual_slope
                            .calculate_path_loss_db(distance_m, obs.is_los, &mut rng);

                        // Diffraction attenuation if obstructed and corner is detected
                        let diff_db = if !obs.is_los {
                            if let Some(ref corner_pt) = obs.closest_corner_point {
                                self.knife_edge.calculate_loss_db(
                                    &sender_pos,
                                    &rx_pos,
                                    corner_pt,
                                    obs.closest_corner_dist_m,
                                )
                            } else {
                                0.0
                            }
                        } else {
                            0.0
                        };

                        let total_attenuation_db =
                            base_pl_db + obs.penetration_attenuation_db + diff_db;
                        let p_rx = packet.tx_power_dbm - total_attenuation_db;
                        (p_rx, true)
                    }
                    PhyModelType::LegacyPoly => {
                        // Spatio-temporal congestion calculation within 200m radius of sender_pos over last 1.0s
                        let nearby_bytes: usize = packet_history_snapshot
                            .iter()
                            .filter(|(t, pos, _)| {
                                (packet.start_time >= *t)
                                    && (packet.start_time - *t).to_secs_f64()
                                        <= self.legacy_poly.congestion_window_s
                                    && sender_pos.distance_to(pos)
                                        <= self.legacy_poly.congestion_radius_m
                            })
                            .map(|(_, _, len)| *len)
                            .sum();

                        let bytes_on_air = (nearby_bytes.max(payload_len)) as f64;
                        let p_deliver = self.legacy_poly.delivery_probability(
                            distance_m,
                            bytes_on_air,
                            obs.is_los,
                        );
                        let rand_u: f64 = rand::Rng::gen(&mut rng);
                        let legacy_ok = rand_u < p_deliver;
                        let p_rx = if legacy_ok { -75.0 } else { -105.0 };
                        (p_rx, legacy_ok)
                    }
                };

                // 4. Interference and SINR evaluation (for DualSlope)
                if self.model_type == PhyModelType::LegacyPoly {
                    ReceptionResult {
                        receiver_id: rx_id,
                        is_success: is_success_legacy,
                        rx_power_dbm,
                        sinr_db: if is_success_legacy { 15.0 } else { -5.0 },
                        is_los: obs.is_los,
                        distance_m,
                        delivery_delay,
                    }
                } else {
                    let desired_sig = SignalEvent {
                        signal_id: packet.packet_id,
                        rx_power_dbm,
                        rx_power_watts: dbm_to_watts(rx_power_dbm),
                        start_time: packet.start_time,
                        end_time: packet.end_time,
                        is_desired: true,
                    };

                    let (is_success, sinr_db, _) = self.interference_tracker.evaluate_reception(
                        &desired_sig,
                        &active_signals_snapshot,
                        payload_len,
                        &mut rng,
                    );

                    ReceptionResult {
                        receiver_id: rx_id,
                        is_success,
                        rx_power_dbm,
                        sinr_db,
                        is_los: obs.is_los,
                        distance_m,
                        delivery_delay,
                    }
                }
            })
            .collect()
    }
}
