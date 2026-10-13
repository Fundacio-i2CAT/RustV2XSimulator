//! Path loss models: Standard Dual-Slope Log-Distance and Legacy 10th-Degree Polynomial.

use rand::distributions::Distribution;
use rand::Rng;
use rand_distr::Normal;
use serde::{Deserialize, Serialize};

pub const SPEED_OF_LIGHT: f64 = 299_792_458.0;

/// Dual-Slope Log-Distance Path Loss model with log-normal shadow fading.
///
/// Parameters based on vehicular 5.9 GHz measurements:
/// - Breakpoint distance: 120.0 m
/// - Exponent before breakpoint (gamma1): 2.05
/// - Exponent after breakpoint (gamma2): 3.90
/// - Shadow fading std-dev: LoS = 3.1 dB, NLoS = 8.3 dB
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DualSlopePathLoss {
    pub carrier_frequency_hz: f64,
    pub d_break: f64,
    pub gamma1: f64,
    pub gamma2: f64,
    pub sigma_los: f64,
    pub sigma_nlos: f64,
    pub pl_0: f64,
}

impl Default for DualSlopePathLoss {
    fn default() -> Self {
        Self::new(5.9e9, 120.0, 2.05, 3.90, 3.1, 8.3)
    }
}

impl DualSlopePathLoss {
    pub fn new(
        carrier_frequency_hz: f64,
        d_break: f64,
        gamma1: f64,
        gamma2: f64,
        sigma_los: f64,
        sigma_nlos: f64,
    ) -> Self {
        // Free-space path loss at d_0 = 1.0 m: 20*log10(4*pi*d0*fc / c)
        let pl_0 = 20.0
            * (4.0 * std::f64::consts::PI * 1.0 * carrier_frequency_hz / SPEED_OF_LIGHT).log10();
        Self {
            carrier_frequency_hz,
            d_break,
            gamma1,
            gamma2,
            sigma_los,
            sigma_nlos,
            pl_0,
        }
    }

    /// Calculate deterministic path loss in dB at distance `distance_m`.
    pub fn deterministic_path_loss_db(&self, distance_m: f64) -> f64 {
        let d = distance_m.max(1.0);
        if d <= self.d_break {
            self.pl_0 + 10.0 * self.gamma1 * d.log10()
        } else {
            let pl_break = self.pl_0 + 10.0 * self.gamma1 * self.d_break.log10();
            pl_break + 10.0 * self.gamma2 * (d / self.d_break).log10()
        }
    }

    /// Calculate total path loss in dB including log-normal shadow fading.
    pub fn calculate_path_loss_db<R: Rng>(
        &self,
        distance_m: f64,
        is_los: bool,
        rng: &mut R,
    ) -> f64 {
        let det_pl = self.deterministic_path_loss_db(distance_m);
        let sigma = if is_los {
            self.sigma_los
        } else {
            self.sigma_nlos
        };

        let fading = if sigma > 0.0 {
            let normal = Normal::new(0.0, sigma).unwrap();
            normal.sample(rng)
        } else {
            0.0
        };

        (det_pl + fading).max(0.0)
    }
}

/// Legacy 10th-degree polynomial path loss and logistic congestion model.
///
/// Ported from commit 6885a2f3 / pyv2xsimulator baseline:
/// - Fitted 10th-degree polynomial with 350 m cutoff.
/// - Spatio-temporal logistic congestion (1.0 s window, 200 m radius).
/// - Configurable Gaussian delivery delay.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LegacyPolynomialPathLoss {
    pub max_distance_m: f64,
    pub enable_clamping: bool,
    pub logistic_k: f64,
    pub logistic_b0: f64,
    pub congestion_radius_m: f64,
    pub congestion_window_s: f64,
    pub use_gaussian_delay: bool,
    pub mean_delay_s: f64,
    pub std_dev_delay_s: f64,
    pub min_delay_s: f64,
}

impl Default for LegacyPolynomialPathLoss {
    fn default() -> Self {
        Self {
            max_distance_m: 350.0,
            enable_clamping: true,
            logistic_k: 0.0005,
            logistic_b0: 20_000.0,
            congestion_radius_m: 200.0,
            congestion_window_s: 1.0,
            use_gaussian_delay: true,
            mean_delay_s: 0.02,
            std_dev_delay_s: 0.01,
            min_delay_s: 0.002,
        }
    }
}

impl LegacyPolynomialPathLoss {
    const C10: f64 = 1.111_823_869_040_061e-25;
    const C9: f64 = 3.597_485_384_820_973_5e-23;
    const C8: f64 = -8.038_884_092_343_818e-21;
    const C7: f64 = -9.158_429_982_705_657e-18;
    const C6: f64 = -2.132_059_001_989_335e-15;
    const C5: f64 = 8.192_060_372_243_259e-13;
    const C4: f64 = 4.764_017_738_147_641e-10;
    const C3: f64 = -1.596_566_671_433_574_4e-7;
    const C2: f64 = 1.867_541_672_239_932e-5;
    const C1: f64 = -0.000_498_485_400_857_115_6;
    const C0: f64 = 0.0;

    /// Evaluate 10th-degree polynomial probability of packet loss at distance `d`.
    ///
    /// Distances beyond `max_distance_m` (default 350 m) return 1.0 (100% loss)
    /// exactly matching commit 6885a2f3.
    pub fn distance_loss_prob(&self, d: f64) -> f64 {
        if d > self.max_distance_m {
            return 1.0;
        }

        let d2 = d * d;
        let d3 = d2 * d;
        let d4 = d3 * d;
        let d5 = d4 * d;
        let d6 = d5 * d;
        let d7 = d6 * d;
        let d8 = d7 * d;
        let d9 = d8 * d;
        let d10 = d9 * d;

        let raw_loss = Self::C10 * d10
            + Self::C9 * d9
            + Self::C8 * d8
            + Self::C7 * d7
            + Self::C6 * d6
            + Self::C5 * d5
            + Self::C4 * d4
            + Self::C3 * d3
            + Self::C2 * d2
            + Self::C1 * d
            + Self::C0;

        if self.enable_clamping {
            raw_loss.clamp(0.0, 1.0)
        } else {
            raw_loss
        }
    }

    /// Evaluate logistic congestion loss probability based on local bytes on air.
    pub fn congestion_loss_prob(&self, bytes_on_air_200m: f64) -> f64 {
        let z = self.logistic_k * (bytes_on_air_200m - self.logistic_b0);
        1.0 / (1.0 + (-z).exp())
    }

    /// Total delivery probability under legacy model.
    pub fn delivery_probability(
        &self,
        distance_m: f64,
        bytes_on_air_200m: f64,
        is_los: bool,
    ) -> f64 {
        if !is_los || distance_m > self.max_distance_m {
            return 0.0;
        }
        let p_loss_dist = self.distance_loss_prob(distance_m);
        let p_loss_cong = self.congestion_loss_prob(bytes_on_air_200m);
        (1.0 - p_loss_dist) * (1.0 - p_loss_cong)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dual_slope_continuity() {
        let model = DualSlopePathLoss::default();
        let pl_before = model.deterministic_path_loss_db(119.99);
        let pl_after = model.deterministic_path_loss_db(120.01);
        assert!((pl_after - pl_before).abs() < 0.01);
        assert!(model.deterministic_path_loss_db(300.0) > model.deterministic_path_loss_db(100.0));
    }

    #[test]
    fn test_polynomial_clamping_prevents_negative() {
        let unclamped = LegacyPolynomialPathLoss {
            enable_clamping: false,
            ..Default::default()
        };
        let clamped = LegacyPolynomialPathLoss {
            enable_clamping: true,
            ..Default::default()
        };

        // At d = 10m, C1 < 0 causes raw polynomial to dip below 0
        let raw = unclamped.distance_loss_prob(10.0);
        let fixed = clamped.distance_loss_prob(10.0);
        assert!(raw < 0.0);
        assert_eq!(fixed, 0.0);
    }

    #[test]
    fn test_polynomial_distance_cutoff() {
        let model = LegacyPolynomialPathLoss::default();
        assert_eq!(model.distance_loss_prob(350.01), 1.0);
        assert_eq!(model.distance_loss_prob(500.0), 1.0);
        assert!(model.distance_loss_prob(100.0) < 1.0);
        assert_eq!(model.delivery_probability(351.0, 0.0, true), 0.0);
    }
}
