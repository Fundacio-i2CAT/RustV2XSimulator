//! Krauß-Safe Closed-Loop Actuation Pipeline.
//!
//! Enforces collision safety under emergency braking by calculating the Krauß
//! safe boundary velocity and applying ballistic sub-step velocity injection.
//!
//! Supports:
//! - Sub-step ballistic mode: immediate deceleration onset at sub-step timestamp t_k + delta_t.
//! - Quantized 100 ms mode (ablation A7): deceleration onset quantized to the next full step t_{k+1}.

use crate::common::SimTime;
use crate::mobility::vehicle_state::VehicleKinematics;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ActuationMode {
    /// Sub-step ballistic velocity injection immediately upon packet arrival.
    KraussSubstep,
    /// Delay deceleration onset until the next 100 ms mobility timestep boundary.
    Quantized,
}

#[derive(Debug, Clone)]
pub struct ActuationResult {
    pub injected_velocity: f64,
    pub safe_velocity: f64,
    pub effective_acceleration: f64,
    pub gap_to_leader: f64,
    pub is_collision: bool,
    pub reaction_latency_ms: f64,
}

#[derive(Debug, Clone)]
pub struct KraussSafeActuator {
    pub mode: ActuationMode,
    pub b_max: f64,     // Maximum braking deceleration (e.g. 6.0 - 8.0 m/s^2)
    pub tau_react: f64, // Driver/system mechanical reaction time (e.g. 0.1 s)
    pub g_min: f64,     // Minimum standstill safety gap (e.g. 2.0 - 2.5 m)
    pub collision_count: u32,
}

impl Default for KraussSafeActuator {
    fn default() -> Self {
        Self::new(ActuationMode::KraussSubstep, 6.0, 0.1, 2.5)
    }
}

impl KraussSafeActuator {
    pub fn new(mode: ActuationMode, b_max: f64, tau_react: f64, g_min: f64) -> Self {
        Self {
            mode,
            b_max,
            tau_react,
            g_min,
            collision_count: 0,
        }
    }

    /// Compute Krauß safe boundary velocity:
    ///
    /// v_safe(t_{k+1}) = -b_max * tau + sqrt((b_max * tau)^2 + v_lead^2 + 2 * b_max * (g - g_min))
    pub fn compute_krauss_safe_velocity(&self, v_lead: f64, gap: f64) -> f64 {
        let b_tau = self.b_max * self.tau_react;
        let effective_gap = (gap - self.g_min).max(0.0);
        let radicand = b_tau * b_tau + v_lead * v_lead + 2.0 * self.b_max * effective_gap;
        let v_safe = -b_tau + radicand.max(0.0).sqrt();
        v_safe.max(0.0)
    }

    /// Execute the closed-loop actuation pipeline when an EEBL message is decoded
    /// at delivery time `t_delivery` within step [t_k, t_k + dt_mob].
    pub fn actuate_eebl_reaction(
        &mut self,
        ego: &mut VehicleKinematics,
        leader_speed: f64,
        leader_gap: f64,
        t_hazard_detected: SimTime,
        t_delivery: SimTime,
        dt_mob_s: f64,
    ) -> ActuationResult {
        // Compute Krauß safe velocity boundary
        let v_safe = self.compute_krauss_safe_velocity(leader_speed, leader_gap);

        // Required emergency deceleration
        let a_req = -self.b_max;

        let v_injected = match self.mode {
            ActuationMode::KraussSubstep => {
                // Ballistic sub-step velocity projection:
                // delta_t is the intra-step arrival time within the mobility step
                let delta_t_s = (t_delivery.to_secs_f64() % dt_mob_s).max(0.0);
                let remaining_time_s = (dt_mob_s - delta_t_s).max(0.0);

                let v_ballistic = (ego.speed + a_req * remaining_time_s).max(0.0);
                v_ballistic.min(v_safe)
            }
            ActuationMode::Quantized => {
                // Delay braking until next mobility step boundary (full dt_mob delay)
                v_safe.min(ego.speed)
            }
        };

        let effective_accel = (v_injected - ego.speed) / dt_mob_s.max(1e-4);
        ego.speed = v_injected;
        ego.acceleration = effective_accel;
        ego.is_braking_hazard = true;

        let is_collision = leader_gap < self.g_min;
        if is_collision {
            self.collision_count += 1;
        }

        let reaction_latency_ms = if t_delivery >= t_hazard_detected {
            (t_delivery - t_hazard_detected).to_secs_f64() * 1000.0
        } else {
            0.0
        };

        ActuationResult {
            injected_velocity: v_injected,
            safe_velocity: v_safe,
            effective_acceleration: effective_accel,
            gap_to_leader: leader_gap,
            is_collision,
            reaction_latency_ms,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_krauss_safe_velocity_formula() {
        let actuator = KraussSafeActuator::new(ActuationMode::KraussSubstep, 6.0, 0.1, 2.5);
        // Standstill leader (v_lead = 0) with gap = 20m:
        // v_safe = -0.6 + sqrt(0.36 + 0 + 2*6*(20 - 2.5)) = -0.6 + sqrt(0.36 + 210) ~= 13.9 m/s
        let v_safe = actuator.compute_krauss_safe_velocity(0.0, 20.0);
        assert!((v_safe - 13.9).abs() < 0.5);

        // At minimal gap (2.5m), safe speed is 0
        let v_min = actuator.compute_krauss_safe_velocity(0.0, 2.5);
        assert_eq!(v_min, 0.0);
    }
}
