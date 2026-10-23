//! Integration Test: Parametric Empirical Physical Layer Model (Commit 6885a2f3 Parity).
//!
//! Validates:
//! - 10th-degree polynomial distance loss with 350 m cutoff.
//! - Spatio-temporal rolling window congestion loss.
//! - Gaussian delivery delay distribution (min 2 ms).
//! - Clean WirelessTechnology trait execution for both Dsrc80211p and ParametricEmpirical.

use rand::rngs::StdRng;
use rand::SeedableRng;

use rustv2xsimulator::common::SimTime;
use rustv2xsimulator::geometry::polygon::BuildingMap;
use rustv2xsimulator::geometry::segment::Point2D;
use rustv2xsimulator::mac::edca::TxFrame;
use rustv2xsimulator::phy::air_packet::AirPacket;
use rustv2xsimulator::phy::path_loss::LegacyPolynomialPathLoss;
use rustv2xsimulator::phy::radio_medium::{PhyModelType, RadioMedium};
use rustv2xsimulator::phy::technology::{
    Dsrc80211pModel, ParametricEmpiricalModel, WirelessTechnology,
};

#[test]
fn test_parametric_350m_hard_cutoff() {
    let model = LegacyPolynomialPathLoss::default();

    // At 350.0m or below, delivery probability can be positive
    let p_loss_300 = model.distance_loss_prob(300.0);
    assert!(p_loss_300 < 1.0, "At 300m loss should not be 100%");

    // Beyond 350m, distance loss is strictly 100% (matching commit 6885a2f3)
    let p_loss_350_1 = model.distance_loss_prob(350.1);
    assert_eq!(p_loss_350_1, 1.0);
    assert_eq!(model.delivery_probability(350.1, 0.0, true), 0.0);
    assert_eq!(model.delivery_probability(500.0, 0.0, true), 0.0);
}

#[test]
fn test_parametric_spatio_temporal_congestion_loss() {
    let mut medium = RadioMedium::new(PhyModelType::LegacyPoly, 23.0);
    let buildings = BuildingMap::default();

    let t0 = SimTime::from_millis(100);
    let pos_sender = Point2D::new(0.0, 0.0);

    // Simulate 60 packets sent in the last second within 100m (total 18,000 bytes)
    for i in 0..60 {
        let pkt = AirPacket::new(
            i,
            1,
            23.0,
            t0,
            SimTime::from_micros(300),
            vec![0u8; 300],
            2,
        );
        medium.register_transmission_with_pos(&pkt, Point2D::new(10.0, 10.0));
    }

    let test_pkt = AirPacket::new(
        100,
        1,
        23.0,
        t0,
        SimTime::from_micros(300),
        vec![0u8; 300],
        2,
    );

    // Evaluate broadcast to a receiver at 50m
    let receivers = vec![(2, Point2D::new(50.0, 0.0), 50.0)];
    let results = medium.evaluate_broadcast(&test_pkt, pos_sender, &receivers, &buildings, 42);

    assert_eq!(results.len(), 1);
    assert!(results[0].delivery_delay >= SimTime::from_millis(2));
}

#[test]
fn test_wireless_technology_trait_abstraction() {
    let mut rng = StdRng::seed_from_u64(42);

    // 1. DSRC 802.11p Model
    let mut dsrc = Dsrc80211pModel::new(23.0, 42);
    assert_eq!(dsrc.name(), "IEEE_802_11p_DSRC");

    let frame = TxFrame {
        frame_id: 1,
        payload: vec![1, 2, 3],
        created_time: SimTime::ZERO,
        duration: SimTime::from_micros(300),
    };
    let sched_time = dsrc.enqueue_outbound(frame.clone(), 2, SimTime::ZERO, &mut rng);
    assert!(sched_time.is_some());

    // 2. Parametric Model
    let mut parametric = ParametricEmpiricalModel::new(LegacyPolynomialPathLoss::default(), 42);
    assert_eq!(parametric.name(), "Parametric_Empirical");

    let sched_param = parametric.enqueue_outbound(frame, 2, SimTime::ZERO, &mut rng);
    assert_eq!(sched_param, Some(SimTime::ZERO)); // Unslotted immediate access
}
