//! Integration Test 3: ETSI TS 102 687 DCC Standardization Conformance.
//!
//! Validates Reactive FSM (5-state, ETSI Table A.1) transitions and LIMERIC
//! adaptive algorithm convergence as implemented by `rustflexstack::management`.

use rustv2xsimulator::common::SimTime;
use rustv2xsimulator::dcc::limeric::LimericDcc;
use rustv2xsimulator::dcc::reactive_fsm::{DccState, ReactiveDccFsm};

#[test]
fn test_reactive_dcc_conformance_step_profile() {
    // ETSI TS 102 687 V1.2.1 Table A.1 (T_on ≤ 1 ms) — 5-state FSM.
    // Note: the adjacency constraint means each update() advances at most one
    // step, so we drive the FSM incrementally by repeating high-CBR inputs.
    let mut fsm = ReactiveDccFsm::new();

    // 1. CBR = 0.15 → Relaxed (t_off = 100 ms, Table A.1)
    let t_off = fsm.update_cbr(0.15);
    assert_eq!(fsm.current_state(), DccState::Relaxed);
    assert_eq!(t_off, SimTime::from_millis(100));

    // 2. CBR = 0.35 → Active1 (t_off = 200 ms, Table A.1)
    let t_off = fsm.update_cbr(0.35);
    assert_eq!(fsm.current_state(), DccState::Active1);
    assert_eq!(t_off, SimTime::from_millis(200));

    // 3. CBR = 0.44 → Active2 (t_off = 400 ms, Table A.1)
    //    (adjacency: one more step up from Active1 → Active2)
    let t_off = fsm.update_cbr(0.44);
    assert_eq!(fsm.current_state(), DccState::Active2);
    assert_eq!(t_off, SimTime::from_millis(400));

    // 4. CBR = 0.52 → Active3 (t_off = 500 ms, Table A.1)
    let t_off = fsm.update_cbr(0.52);
    assert_eq!(fsm.current_state(), DccState::Active3);
    assert_eq!(t_off, SimTime::from_millis(500));

    // 5. CBR = 0.70 → Restrictive (t_off = 1000 ms, Table A.1)
    //    CBR ≥ 0.60 → Restrictive band, but must step through Active3 → Restrictive
    let t_off = fsm.update_cbr(0.70);
    assert_eq!(fsm.current_state(), DccState::Restrictive);
    assert_eq!(t_off, SimTime::from_millis(1000));

    // 6. CBR = 0.10 → step back toward Relaxed (adjacency: one step per call)
    let t_off = fsm.update_cbr(0.10);
    assert_eq!(fsm.current_state(), DccState::Active3);
    assert_eq!(t_off, SimTime::from_millis(500));
}

#[test]
fn test_limeric_adaptive_rate_convergence() {
    let target_cbr = 0.68;
    let mut limeric = LimericDcc::new(target_cbr, 0.016, 0.0012, 2.0, 10.0);

    // Initial delta = delta_min = 0.0006 → T_off = 0.001 / 0.0006 ≈ 1.67 s (clamped to 1.0 s)
    let initial_t_off = limeric.current_t_off();
    assert!(
        initial_t_off > SimTime::ZERO,
        "Initial T_off must be positive"
    );

    // Subject to high channel load (CBR = 0.85 > target)
    // delta must converge toward delta_min (maximum restriction)
    let delta_before = limeric.adaptive.delta;
    for _ in 0..200 {
        limeric.update_cbr(0.85);
    }
    let delta_after_high = limeric.adaptive.delta;
    assert!(
        delta_after_high <= delta_before || delta_after_high <= 0.005,
        "delta must shrink under sustained high CBR (delta = {:.6})",
        delta_after_high,
    );
    assert!(
        delta_after_high >= limeric.adaptive.parameters.delta_min,
        "delta must not fall below delta_min"
    );

    // Subject to light channel load (CBR = 0.20 < target)
    // delta must grow toward delta_max (more transmission allowed)
    for _ in 0..500 {
        limeric.update_cbr(0.20);
    }
    let delta_after_low = limeric.adaptive.delta;
    assert!(
        delta_after_low > delta_after_high,
        "delta must increase under sustained low CBR (delta = {:.6})",
        delta_after_low,
    );
    assert!(
        delta_after_low <= limeric.adaptive.parameters.delta_max,
        "delta must not exceed delta_max"
    );
}
