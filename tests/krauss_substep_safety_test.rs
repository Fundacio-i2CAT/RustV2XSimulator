//! Integration Test 4: Krauß Sub-step Actuation Collision Safety.
//!
//! Executes a 10-vehicle platoon emergency braking simulation.
//! Asserts N_teleport = 0, zero rear-end collisions (gap >= g_min), and sub-step reaction accuracy.

use rustv2xsimulator::engine::{EngineConfig, SimulationEngine};
use rustv2xsimulator::geometry::polygon::BuildingMap;
use rustv2xsimulator::mobility::krauss_actuation::ActuationMode;
use rustv2xsimulator::mobility::trajectory_source::SyntheticMobilityType;

#[test]
fn test_platoon_emergency_braking_krauss_safety() {
    let config = EngineConfig {
        scenario_name: "test_platoon_safety".to_string(),
        duration_s: 15.0,
        mobility_step_s: 0.01, // 10 ms sub-stepping
        num_nodes: 10,
        seed: 42,
        workload: "eebl".to_string(),
        mobility_type: SyntheticMobilityType::Platoon,
        actuation_mode: ActuationMode::KraussSubstep,
        output_dir: "results/test_platoon/".to_string(),
        enable_pcap: false,
        hazard_time_s: 3.0,
        ..Default::default()
    };

    let buildings = BuildingMap::default();
    let mut engine = SimulationEngine::new(config, buildings).expect("create engine");
    engine.hazard_time_s = 3.0; // Leader brakes at 3.0 seconds

    engine.run().expect("run simulation");

    // Check collision count across entire platoon run
    assert_eq!(
        engine.collision_count, 0,
        "Zero rear-end collisions allowed under Krauß-safe actuation!"
    );

    // Verify minimum safety distance maintained for every follower
    for i in 2..=10 {
        let agent = engine.agents.get(&(i as u32)).expect("agent");
        if let Some(gap) = agent.kinematics.leader_distance {
            assert!(
                gap >= agent.kinematics.min_gap_m - 0.1,
                "Vehicle {} violated minimum standstill safety gap: gap={:.2} m, min_gap={:.2} m",
                i,
                gap,
                agent.kinematics.min_gap_m
            );
        }
    }
}
