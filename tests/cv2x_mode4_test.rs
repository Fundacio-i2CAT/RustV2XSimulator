//! Integration Test: 3GPP Rel-14 C-V2X PC5 Mode 4 (LTE Sidelink).
//!
//! Verifies:
//! 1. Standard 10 MHz (50 RBs) and configurable 20 MHz (100 RBs) resource grid partition.
//! 2. Sensing-Based Semi-Persistent Scheduling (SB-SPS) per 3GPP TS 36.213 Section 14.1.1.6.
//! 3. Interaction between ETSI aperiodic CAM arrivals and 100 ms SPS reservation grants.
//! 4. Exclusion of colliding high-RSRP resources.
//! 5. Physical layer dual decoding (PSCCH SCI + PSSCH TB).

use rand::SeedableRng;
use rand::rngs::StdRng;

use rustv2xsimulator::common::SimTime;
use rustv2xsimulator::geometry::polygon::BuildingMap;
use rustv2xsimulator::geometry::segment::Point2D;
use rustv2xsimulator::mac::edca::TxFrame;
use rustv2xsimulator::phy::air_packet::AirPacket;
use rustv2xsimulator::phy::cv2x::config::Cv2xConfig;
use rustv2xsimulator::phy::cv2x::phy_bler::SidelinkPhyModel;
use rustv2xsimulator::phy::cv2x::sb_sps::SensingBasedSpsScheduler;
use rustv2xsimulator::phy::cv2x::sci::SidelinkControlInfo;
use rustv2xsimulator::phy::cv2x::sensing::SidelinkSensingManager;
use rustv2xsimulator::phy::technology::{Cv2xMode4Model, WirelessTechnology};

#[test]
fn test_cv2x_channel_bandwidth_configuration() {
    // 10 MHz standard
    let mut cfg10 = Cv2xConfig::default();
    cfg10.bandwidth_mhz = 10.0;
    cfg10.subchannel_size_rbs = 10;
    assert_eq!(cfg10.total_resource_blocks(), 50);
    assert_eq!(cfg10.num_subchannels(), 5);

    // 20 MHz wideband
    let mut cfg20 = Cv2xConfig::default();
    cfg20.bandwidth_mhz = 20.0;
    cfg20.subchannel_size_rbs = 10;
    assert_eq!(cfg20.total_resource_blocks(), 100);
    assert_eq!(cfg20.num_subchannels(), 10);
}

#[test]
fn test_cv2x_adaptive_cam_sps_interaction() {
    let cfg = Cv2xConfig::default();
    let mut scheduler = SensingBasedSpsScheduler::new(cfg.clone());
    let sensing = SidelinkSensingManager::new(cfg);
    let mut rng = StdRng::seed_from_u64(12345);

    // Initial CAM generated at t = 100 ms
    let csr1 = scheduler.schedule_packet(100, &sensing, &mut rng);
    assert!(csr1.subframe >= 104);
    assert!(csr1.subframe <= 200);

    // Grant is registered with P_rsvp = 100 ms
    assert!(scheduler.active_grant.is_some());
    let initial_grant = scheduler.active_grant.unwrap();
    assert_eq!(initial_grant.interval_ms, 100);

    // Simulate completion of transmission 1
    scheduler.on_transmission_completed(&mut rng);
    let updated_grant = scheduler.active_grant.unwrap();
    // Next reserved subframe has advanced by exactly 100 ms!
    assert_eq!(updated_grant.resource.subframe, csr1.subframe + 100);

    // Next adaptive CAM arrives at t = 200 ms (nominally 100 ms later):
    // The scheduler should REUSE the active SPS grant rather than running random reselection!
    let csr2 = scheduler.schedule_packet(200, &sensing, &mut rng);
    assert_eq!(csr2.subframe, csr1.subframe + 100);
    assert_eq!(csr2.subchannel, csr1.subchannel);
}

#[test]
fn test_cv2x_sb_sps_sensing_exclusion() {
    let cfg = Cv2xConfig::default();
    let scheduler = SensingBasedSpsScheduler::new(cfg.clone());
    let mut sensing = SidelinkSensingManager::new(cfg);
    let mut rng = StdRng::seed_from_u64(42);

    // Node 99 transmits at subframe 10 reserving subchannel 1 every 100 ms with high RSRP
    let sci = SidelinkControlInfo::new(99, 4, 100, 1, 7, 10, 10);
    sensing.record_received_sci(sci, -75.0, -75.0);

    // Local node requests resource at subframe 100 (window [104..200]).
    // Subframe 110 on subchannel 1 (10 + 100 = 110) must be excluded from selection!
    for _ in 0..20 {
        let chosen = scheduler.reselect_resource(100, &sensing, &mut rng);
        assert!(
            !(chosen.subframe == 110 && chosen.subchannel == 1),
            "High-RSRP reserved resource (110, 1) must be excluded"
        );
    }
}

#[test]
fn test_cv2x_sidelink_dual_decoding() {
    let mut rng = StdRng::seed_from_u64(999);

    // Very good channel (SINR = 20 dB): both SCI and TB succeed
    let res_good = SidelinkPhyModel::evaluate_reception(20.0, 300, 7, &mut rng);
    assert!(res_good.is_success);
    assert!(res_good.sci_success);
    assert!(res_good.tb_success);

    // Intermediate channel (SINR = 0 dB): SCI succeeds (demod threshold -2.5 dB), but TB fails (demod threshold ~3.5 dB)
    let res_mid = SidelinkPhyModel::evaluate_reception(0.0, 300, 7, &mut rng);
    assert!(!res_mid.is_success);
    assert!(res_mid.sci_success); // SCI successfully decoded
    assert!(!res_mid.tb_success); // Transport block corrupted

    // Very bad channel (SINR = -10 dB): both fail
    let res_bad = SidelinkPhyModel::evaluate_reception(-10.0, 300, 7, &mut rng);
    assert!(!res_bad.is_success);
    assert!(!res_bad.sci_success);
}

#[test]
fn test_cv2x_wireless_technology_trait_integration() {
    let cfg = Cv2xConfig::default();
    let mut model = Cv2xMode4Model::new(cfg, 42);
    let mut rng = StdRng::seed_from_u64(101);

    assert_eq!(model.name(), "3GPP_C-V2X_PC5_Mode4");

    // Enqueue an outbound frame at t = 100 ms
    let frame = TxFrame {
        frame_id: 1,
        payload: vec![0u8; 300],
        created_time: SimTime::from_millis(100),
        duration: SimTime::from_millis(1),
    };

    let scheduled_time = model.enqueue_outbound(frame, 2, SimTime::from_millis(100), &mut rng);
    assert!(scheduled_time.is_some());
    let t_tx = scheduled_time.unwrap();
    // Must be aligned to a subframe boundary >= 104 ms
    assert!(t_tx >= SimTime::from_millis(104));
    assert!(t_tx <= SimTime::from_millis(200));

    // Evaluate transmission across two receivers (one close at 50m, one far at 600m)
    let sender_pos = Point2D::new(0.0, 0.0);
    let rx1 = (1, Point2D::new(50.0, 0.0), 50.0);
    let rx2 = (2, Point2D::new(600.0, 0.0), 600.0);
    let receivers = vec![rx1, rx2];
    let buildings = BuildingMap::default();

    let air_pkt = AirPacket::new(
        1,
        0,
        23.0,
        t_tx,
        SimTime::from_millis(1),
        vec![0u8; 300],
        2,
    );

    let results = model.evaluate_transmission(
        &air_pkt,
        sender_pos,
        &receivers,
        &buildings,
        t_tx,
        &mut rng,
    );

    assert_eq!(results.len(), 2);
    // Close receiver (50m) should have high SINR and decode successfully
    let r1 = results.iter().find(|r| r.receiver_id == 1).unwrap();
    assert!(r1.is_success);
    assert!(r1.sinr_db > 10.0);
}
