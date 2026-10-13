//! Integration Test 1: Causality Invariance & Strict Temporal Monotonicity.
//!
//! Asserts zero retrograde DES events (t_{k+1} >= t_k) over a 60-second run.

use rustv2xsimulator::common::SimTime;
use rustv2xsimulator::engine::{EngineConfig, SimulationEngine};
use rustv2xsimulator::geometry::polygon::BuildingMap;
use rustv2xsimulator::mobility::trajectory_source::SyntheticMobilityType;
use rustv2xsimulator::scheduler::event::EventPayload;

#[test]
fn test_causality_invariance_over_60s() {
    let config = EngineConfig {
        scenario_name: "test_causality".to_string(),
        duration_s: 60.0,
        mobility_step_s: 0.05,
        num_nodes: 50,
        seed: 42,
        workload: "cam".to_string(),
        mobility_type: SyntheticMobilityType::Urban,
        output_dir: "results/test_causality/".to_string(),
        enable_pcap: false,
        ..Default::default()
    };

    let buildings = BuildingMap::default();
    let mut engine = SimulationEngine::new(config, buildings).expect("create engine");

    let mut prev_time = SimTime::ZERO;
    let mut total_events = 0u64;

    while let Ok(Some(event)) = engine.step() {
        assert!(
            event.timestamp >= prev_time,
            "Causality violation! Event #{} at {} is earlier than prev_time {}",
            event.id,
            event.timestamp,
            prev_time
        );
        prev_time = event.timestamp;
        total_events += 1;

        // Ensure attempting to schedule a retroactive event fails immediately
        if prev_time > SimTime::from_millis(10) {
            let retroactive_t = prev_time - SimTime::from_millis(5);
            let res =
                engine
                    .scheduler
                    .schedule(retroactive_t, 0, 1, EventPayload::Timer { token: 99 });
            assert!(
                res.is_err(),
                "Retroactive event must be rejected by scheduler"
            );
        }
    }

    assert!(
        total_events > 100,
        "Should have processed substantial events"
    );
    assert_eq!(engine.scheduler.current_time(), prev_time);
}
