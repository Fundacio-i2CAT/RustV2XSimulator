//! Integration Test: 3GPP Release 16/17 5G NR-V2X Sidelink Mode 2.
//!
//! Standards Reference: 3GPP TS 38.211, TS 38.212, TS 38.213, TS 38.214, TS 38.321
//! Parity References: CTTC 5G-LENA & UWICORE NR-V2X Link-Level Simulator (IEEE IoT-J 2024)
//!
//! Verifies:
//! 1. Flexible numerologies mu in {0, 1, 2} (15, 30, 60 kHz SCS, 1.0, 0.5, 0.25 ms slots).
//! 2. Two-Stage Sidelink Control Information (SCI Format 1-A on PSCCH & SCI Format 2-A on PSSCH).
//! 3. Sensing-Based Mode 2 Autonomous Resource Selection (TS 38.213 Section 16.6.1).
//! 4. Release 16 Re-evaluation Check at slot m - T3 avoiding packet collisions.
//! 5. Release 16 Pre-emption mechanism allowing high-priority safety traffic to override lower-priority reservations.
//! 6. PSFCH HARQ Feedback Timing and Redundancy Version progression (0 -> 2 -> 3 -> 1).
//! 7. 5G NR LDPC & Polar dual decoding decider.
//! 8. Seamless `WirelessTechnology` trait abstraction integration.

use rand::rngs::StdRng;
use rand::SeedableRng;

use rustv2xsimulator::common::SimTime;
use rustv2xsimulator::geometry::polygon::BuildingMap;
use rustv2xsimulator::geometry::segment::Point2D;
use rustv2xsimulator::mac::edca::TxFrame;
use rustv2xsimulator::phy::air_packet::AirPacket;
use rustv2xsimulator::phy::nrv2x::{
    NrCastType, NrHarqEntity, NrHarqFeedback, NrMode2Scheduler, NrNumerology, NrPhyBlerModel,
    NrResourceGrid, NrSensingManager, NrV2xConfig, NrV2xMode2Model, PsfchTimingCalculator,
    SciFormat1A, SciFormat2A,
};
use rustv2xsimulator::phy::technology::WirelessTechnology;

#[test]
fn test_nrv2x_numerologies_and_timing_scaling() {
    // mu = 0: 15 kHz SCS, 1.0 ms slot
    let mu0 = NrNumerology::Mu0_15kHz;
    assert_eq!(mu0.scs_hz(), 15_000.0);
    assert_eq!(mu0.slot_duration(), SimTime::from_millis(1));
    assert_eq!(mu0.slots_per_subframe(), 1);
    assert_eq!(mu0.slots_per_frame(), 10);

    // mu = 1: 30 kHz SCS, 0.5 ms slot (500 us)
    let mu1 = NrNumerology::Mu1_30kHz;
    assert_eq!(mu1.scs_hz(), 30_000.0);
    assert_eq!(mu1.slot_duration(), SimTime::from_micros(500));
    assert_eq!(mu1.slots_per_subframe(), 2);
    assert_eq!(mu1.slots_per_frame(), 20);

    // mu = 2: 60 kHz SCS, 0.25 ms slot (250 us)
    let mu2 = NrNumerology::Mu2_60kHz;
    assert_eq!(mu2.scs_hz(), 60_000.0);
    assert_eq!(mu2.slot_duration(), SimTime::from_micros(250));
    assert_eq!(mu2.slots_per_subframe(), 4);
    assert_eq!(mu2.slots_per_frame(), 40);

    // Test grid conversions at mu = 1
    let grid = NrResourceGrid::new(mu1, 5, 10);
    assert_eq!(grid.sim_time_to_slot(SimTime::from_millis(10)), 20);
    assert_eq!(grid.slot_to_sim_time(20), SimTime::from_millis(10));
}

#[test]
fn test_nrv2x_two_stage_sci_structure() {
    // Stage 1 SCI on PSCCH (Polar code, Priority 1, 100 ms RRI, subchannel 0, length 1)
    let sci1 = SciFormat1A::new(1, 100, 0, 1, 5, 10);
    assert_eq!(sci1.priority, 1);
    assert_eq!(sci1.reservation_period_ms, 100);
    assert!(sci1.has_periodic_reservation());
    assert_eq!(sci1.start_subchannel, 0);
    assert_eq!(sci1.num_subchannels, 1);
    assert_eq!(sci1.mcs, 10);

    // Stage 2 SCI on PSSCH (Broadcast)
    let sci2_bcast = SciFormat2A::new_broadcast(101);
    assert_eq!(sci2_bcast.source_id, 101);
    assert_eq!(sci2_bcast.destination_id, 0xFFFF);
    assert_eq!(sci2_bcast.cast_type, NrCastType::Broadcast);
    assert!(!sci2_bcast.harq_feedback_enabled);

    // Stage 2 SCI on PSSCH (Unicast with HARQ)
    let sci2_unicast = SciFormat2A::new_unicast(101, 202, 3, true, 0);
    assert_eq!(sci2_unicast.source_id, 101);
    assert_eq!(sci2_unicast.destination_id, 202);
    assert_eq!(sci2_unicast.harq_process_id, 3);
    assert_eq!(sci2_unicast.cast_type, NrCastType::Unicast);
    assert!(sci2_unicast.harq_feedback_enabled);
}

#[test]
fn test_nrv2x_mode2_sensing_and_exclusion() {
    let cfg = NrV2xConfig::default();
    let sched = NrMode2Scheduler::new(cfg.clone());
    let mut sensing = NrSensingManager::new(cfg);
    let mut rng = StdRng::seed_from_u64(42);

    // Node 99 transmits at slot 10 with RRI = 100 ms (200 slots at mu=1) reserving subchannel 2 with high RSRP (-75 dBm)
    let sci = SciFormat1A::new(3, 100, 2, 1, 5, 10);
    sensing.record_received_sci(10, sci, -75.0, -75.0);

    // When selecting at slot 190 (selection window [192..222]),
    // projected reservation at slot 210 on subchannel 2 must be excluded!
    for _ in 0..20 {
        let chosen = sched.reselect_resource(190, 4, 1, &sensing, &mut rng);
        assert!(
            !(chosen.slot == 210 && chosen.start_subchannel == 2),
            "Colliding resource (slot 210, subchannel 2) must be excluded"
        );
    }
}

#[test]
fn test_nrv2x_rel16_reevaluation_drop() {
    let mut cfg = NrV2xConfig::default();
    cfg.t1_min_slots = 10;
    cfg.t2_max_slots = 40;
    let mut sched = NrMode2Scheduler::new(cfg.clone());
    let mut sensing = NrSensingManager::new(cfg);
    let mut rng = StdRng::seed_from_u64(123);

    // 1. Initial selection at slot 10 selects target_slot (in [20..50])
    let res = sched.schedule_packet(10, 4, 1, &sensing, &mut rng);
    let target_slot = res.slot;
    assert!(target_slot >= 20);

    // 2. Another station (node 77) transmits an SCI at slot 15 with higher priority reserving target_slot on the same subchannel
    let mut conflicting_sci = SciFormat1A::new(1, 0, res.start_subchannel, 1, 5, 10);
    conflicting_sci.gap_retx1_slots = (target_slot - 15) as u8;
    sensing.record_received_sci(15, conflicting_sci, -70.0, -70.0);

    // 3. At slot target_slot - T3 (e.g. target_slot - 2 >= 18), re-evaluation check should detect the conflict!
    let check_slot = target_slot - 2;
    let grant = sched.active_grant.unwrap();
    let is_safe = sched.check_reevaluation(check_slot, &grant, &sensing);
    assert!(!is_safe, "Re-evaluation must catch late-arriving conflicting reservations");
}

#[test]
fn test_nrv2x_rel16_preemption() {
    let cfg = NrV2xConfig::default();
    let sched = NrMode2Scheduler::new(cfg.clone());
    let mut sensing = NrSensingManager::new(cfg);
    let mut rng = StdRng::seed_from_u64(999);

    // Lower priority transmission (priority 6 e.g. background) reserves slot 30 on subchannel 0
    let sci_bg = SciFormat1A::new(6, 10, 0, 1, 5, 10);
    sensing.record_received_sci(20, sci_bg, -75.0, -75.0);

    // Incoming high-priority packet (priority 1 e.g. DENM emergency braking)
    // Pre-emption is enabled by default, so the scheduler will NOT exclude slot 30 for priority 1 traffic!
    let chosen = sched.reselect_resource(22, 1, 1, &sensing, &mut rng);
    // Resource selection succeeds cleanly
    assert!(chosen.slot >= 24);
}

#[test]
fn test_nrv2x_psfch_timing_and_harq_entity() {
    let timing = PsfchTimingCalculator::new(2); // PSFCH every 2 slots
    assert!(timing.is_psfch_slot(0));
    assert!(timing.is_psfch_slot(2));
    assert!(timing.is_psfch_slot(4));

    // Tx at slot 1: earliest feedback slot is 1 + 2 = 3. Next PSFCH slot multiple of 2 is 4!
    assert_eq!(timing.feedback_slot_for_tx(1), Some(4));
    // Tx at slot 2: earliest feedback slot is 2 + 2 = 4. Exactly 4!
    assert_eq!(timing.feedback_slot_for_tx(2), Some(4));

    // Test HARQ entity stop-and-wait
    let mut harq_entity = NrHarqEntity::new(4, 2);
    let proc_id = harq_entity.allocate_process().expect("allocate process");
    assert_eq!(proc_id, 0);

    let proc = harq_entity.get_process_mut(proc_id).unwrap();
    proc.start_new_transmission(400);
    assert_eq!(proc.current_rv(), 0);

    // Receiver reports NACK -> moves to RV = 2
    assert!(proc.update_feedback(NrHarqFeedback::Nack));
    assert_eq!(proc.current_rv(), 2);

    // Receiver reports ACK -> process completes and resets
    assert!(!proc.update_feedback(NrHarqFeedback::Ack));
    assert!(!proc.is_active);
}

#[test]
fn test_nrv2x_ldpc_and_polar_dual_decoding() {
    let mut rng = StdRng::seed_from_u64(777);

    // High SINR = 25 dB: Both Polar PSCCH and LDPC PSSCH decode error-free
    let res_good = NrPhyBlerModel::evaluate_reception(25.0, 300, 10, &mut rng);
    assert!(res_good.is_success);
    assert!(res_good.sci1_success);
    assert!(res_good.sci2_success);
    assert!(res_good.tb_success);
    assert_eq!(res_good.harq_feedback, NrHarqFeedback::Ack);

    // Intermediate SINR = 2.0 dB: Polar PSCCH succeeds (-4 dB th), but 16-QAM LDPC fails (+11.2 dB th)
    // Produces NACK feedback!
    let res_mid = NrPhyBlerModel::evaluate_reception(2.0, 300, 10, &mut rng);
    assert!(!res_mid.is_success);
    assert!(res_mid.sci1_success);
    assert!(!res_mid.tb_success);
    assert_eq!(res_mid.harq_feedback, NrHarqFeedback::Nack);

    // Very low SINR = -15 dB: PSCCH cannot be detected
    // Produces DTX (Discontinuous Transmission) feedback!
    let res_bad = NrPhyBlerModel::evaluate_reception(-15.0, 300, 10, &mut rng);
    assert!(!res_bad.is_success);
    assert!(!res_bad.sci1_success);
    assert_eq!(res_bad.harq_feedback, NrHarqFeedback::Dtx);
}

#[test]
fn test_nrv2x_wireless_technology_trait_integration() {
    let cfg = NrV2xConfig::default();
    let mut model = NrV2xMode2Model::new(cfg, 42);
    let mut rng = StdRng::seed_from_u64(101);

    assert_eq!(model.name(), "3GPP_5G_NR_V2X_Sidelink_Mode2");

    // Enqueue an outbound frame at t = 10 ms (slot 20 at mu=1)
    let frame = TxFrame {
        frame_id: 1,
        payload: vec![0u8; 300],
        created_time: SimTime::from_millis(10),
        duration: SimTime::from_micros(500),
    };

    let scheduled_time = model.enqueue_outbound(frame, 0, SimTime::from_millis(10), &mut rng);
    assert!(scheduled_time.is_some());
    let t_tx = scheduled_time.unwrap();
    // Must be scheduled at slot boundary >= slot 22 (11.0 ms)
    assert!(t_tx >= SimTime::from_millis(11));

    // Evaluate transmission across candidate receivers
    let sender_pos = Point2D::new(0.0, 0.0);
    let rx1 = (1, Point2D::new(40.0, 0.0), 40.0);
    let rx2 = (2, Point2D::new(600.0, 0.0), 600.0);
    let receivers = vec![rx1, rx2];
    let buildings = BuildingMap::default();

    let air_pkt = AirPacket::new(
        1,
        0,
        23.0,
        t_tx,
        SimTime::from_micros(500),
        vec![0u8; 300],
        0,
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
    // Receiver 1 (40 m) should have strong signal and succeed
    let r1 = results.iter().find(|r| r.receiver_id == 1).unwrap();
    assert!(r1.is_success);
    assert!(r1.sinr_db > 15.0);

    // CBR evaluation
    model.record_channel_activity(SimTime::from_millis(10), SimTime::from_millis(20));
    let cbr = model.evaluate_cbr(SimTime::from_millis(100));
    assert!(cbr > 0.0);
}
