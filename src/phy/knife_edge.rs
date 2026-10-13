//! ITU-R P.526 Knife-Edge Corner Diffraction Model.
//!
//! Computes continuous diffraction attenuation around occluding building corners.

use crate::geometry::segment::Point2D;
use crate::phy::path_loss::SPEED_OF_LIGHT;

/// Knife-edge diffraction calculator for 5.9 GHz V2X.
#[derive(Debug, Clone)]
pub struct FresnelKnifeEdge {
    pub carrier_frequency_hz: f64,
    pub wavelength_m: f64,
}

impl Default for FresnelKnifeEdge {
    fn default() -> Self {
        Self::new(5.9e9)
    }
}

impl FresnelKnifeEdge {
    pub fn new(carrier_frequency_hz: f64) -> Self {
        let wavelength_m = SPEED_OF_LIGHT / carrier_frequency_hz;
        Self {
            carrier_frequency_hz,
            wavelength_m,
        }
    }

    /// Compute Fresnel-Kirchhoff dimensionless parameter nu.
    ///
    /// * `tx`: Transmitter 2D location
    /// * `rx`: Receiver 2D location
    /// * `corner`: Closest occluding building corner vertex
    /// * `clearance_h`: Perpendicular distance from corner to line tx-rx (positive if obstructing)
    pub fn compute_nu(
        &self,
        tx: &Point2D,
        rx: &Point2D,
        corner: &Point2D,
        clearance_h: f64,
    ) -> f64 {
        let d1 = tx.distance_to(corner).max(1.0);
        let d2 = rx.distance_to(corner).max(1.0);

        clearance_h * (2.0 * (d1 + d2) / (self.wavelength_m * d1 * d2)).sqrt()
    }

    /// Calculate knife-edge diffraction loss J(nu) in dB.
    ///
    /// ITU-R P.526 standard approximation:
    /// J(nu) = 6.9 + 20*log10(sqrt((nu - 0.1)^2 + 1) + nu - 0.1) for nu > -0.78
    /// J(nu) = 0 for nu <= -0.78
    pub fn diffraction_loss_db(&self, nu: f64) -> f64 {
        if nu <= -0.78 {
            0.0
        } else {
            let term = ((nu - 0.1).powi(2) + 1.0).sqrt() + nu - 0.1;
            if term > 0.0 {
                let j = 6.9 + 20.0 * term.log10();
                j.clamp(0.0, 45.0)
            } else {
                0.0
            }
        }
    }

    /// Convenience calculation from geometry.
    pub fn calculate_loss_db(
        &self,
        tx: &Point2D,
        rx: &Point2D,
        corner: &Point2D,
        corner_dist_to_ray: f64,
    ) -> f64 {
        let nu = self.compute_nu(tx, rx, corner, corner_dist_to_ray);
        self.diffraction_loss_db(nu)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_knife_edge_attenuation() {
        let diff = FresnelKnifeEdge::default();
        // At grazing incidence nu = 0, J(0) ~= 6.0 dB
        let j_0 = diff.diffraction_loss_db(0.0);
        assert!((j_0 - 6.0).abs() < 1.0);

        // Deep shadow nu = 2.0 -> significant loss
        let j_deep = diff.diffraction_loss_db(2.0);
        assert!(j_deep > 15.0);

        // Well clear of obstacle nu = -1.0 -> 0 dB
        let j_clear = diff.diffraction_loss_db(-1.0);
        assert_eq!(j_clear, 0.0);
    }
}
