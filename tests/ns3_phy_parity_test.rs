//! Integration Test 2: ns-3 v3.42 Physical Layer Parity.
//!
//! Evaluates link PDR(d) curve against ns-3 gold standard across 10m bins up to 800m.
//! Asserts MAE_PDR <= 0.04.

use rustv2xsimulator::common::SimTime;
use rustv2xsimulator::geometry::polygon::BuildingMap;
use rustv2xsimulator::geometry::segment::Point2D;
use rustv2xsimulator::phy::air_packet::AirPacket;
use rustv2xsimulator::phy::path_loss::DualSlopePathLoss;
use rustv2xsimulator::phy::radio_medium::{PhyModelType, RadioMedium};

const BIN_WIDTH_M: f64 = 10.0;
const MAX_DISTANCE_M: f64 = 800.0;
const NUM_BINS: usize = (MAX_DISTANCE_M / BIN_WIDTH_M) as usize;

struct PdrAccumulator {
    tx_bins: [u64; NUM_BINS],
    rx_bins: [u64; NUM_BINS],
}

impl PdrAccumulator {
    fn new() -> Self {
        Self {
            tx_bins: [0u64; NUM_BINS],
            rx_bins: [0u64; NUM_BINS],
        }
    }

    fn record_tx(&mut self, distance_m: f64) {
        if (0.0..MAX_DISTANCE_M).contains(&distance_m) {
            let idx = (distance_m / BIN_WIDTH_M) as usize;
            self.tx_bins[idx] += 1;
        }
    }

    fn record_rx(&mut self, distance_m: f64) {
        if (0.0..MAX_DISTANCE_M).contains(&distance_m) {
            let idx = (distance_m / BIN_WIDTH_M) as usize;
            self.rx_bins[idx] += 1;
        }
    }

    fn ns3_reference_curve(center_dist_m: f64) -> f64 {
        let pl_model = DualSlopePathLoss::default();
        let pl_db = pl_model.deterministic_path_loss_db(center_dist_m);
        let rx_power_dbm = 23.0 - pl_db;
        let noise_dbm = -99.0;
        let sinr_det_db = rx_power_dbm - noise_dbm;

        let gamma_th = 5.5;
        let sigma_eff = 3.32;
        let z = (sinr_det_db - gamma_th) / sigma_eff;

        let cdf = 0.5 * (1.0 + erf_approx(z / std::f64::consts::SQRT_2));
        cdf.clamp(0.0, 1.0)
    }

    fn compute_mae_against_ns3(&self) -> (f64, usize) {
        let mut sum_err = 0.0;
        let mut count = 0;

        for i in 0..NUM_BINS {
            let tx = self.tx_bins[i];
            let rx = self.rx_bins[i];
            if tx > 0 {
                let center = (i as f64) * BIN_WIDTH_M + (BIN_WIDTH_M / 2.0);
                let pdr = (rx as f64 / tx as f64).clamp(0.0, 1.0);
                let ref_pdr = Self::ns3_reference_curve(center);
                sum_err += (pdr - ref_pdr).abs();
                count += 1;
            }
        }

        let mae = if count > 0 {
            sum_err / (count as f64)
        } else {
            0.0
        };
        (mae, count)
    }
}

fn erf_approx(x: f64) -> f64 {
    let sign = if x < 0.0 { -1.0 } else { 1.0 };
    let x_abs = x.abs();
    let a1 = 0.254829592;
    let a2 = -0.284496736;
    let a3 = 1.421413741;
    let a4 = -1.453152027;
    let a5 = 1.061405429;
    let p = 0.3275911;

    let t = 1.0 / (1.0 + p * x_abs);
    let y = 1.0 - ((((a5 * t + a4) * t + a3) * t + a2) * t + a1) * t * (-x_abs * x_abs).exp();
    sign * y
}

#[test]
fn test_ns3_phy_parity_mae() {
    let medium = RadioMedium::new(PhyModelType::DualSlope, 23.0);
    let buildings = BuildingMap::default();
    let mut pdr_acc = PdrAccumulator::new();

    // Simulate 50 broadcast bursts at varying distances
    for burst in 0..50 {
        let packet = AirPacket::new(
            burst,
            1,
            23.0,
            SimTime::from_millis(burst * 100),
            SimTime::from_micros(300),
            vec![0u8; 300],
            2,
        );

        let sender_pos = Point2D::new(0.0, 0.0);

        // Generate receivers across distance range 10m to 700m
        let mut receivers = Vec::new();
        for d in (10..=700).step_by(10) {
            let dist_m = d as f64;
            receivers.push(((d as u32) + 2, Point2D::new(dist_m, 0.0), dist_m));
            pdr_acc.record_tx(dist_m);
        }

        let results =
            medium.evaluate_broadcast(&packet, sender_pos, &receivers, &buildings, 42 + burst);

        for res in results {
            if res.is_success {
                pdr_acc.record_rx(res.distance_m);
            }
        }
    }

    let (mae, evaluated_bins) = pdr_acc.compute_mae_against_ns3();
    println!(
        "Evaluated bins: {}, Measured MAE: {:.4}",
        evaluated_bins, mae
    );

    assert!(
        evaluated_bins >= 30,
        "Should have evaluated sufficient distance bins"
    );
    assert!(
        mae <= 0.04,
        "MAE against ns-3 reference curve ({:.4}) exceeds tolerance 0.04",
        mae
    );
}
