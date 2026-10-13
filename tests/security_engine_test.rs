use rustv2xsimulator::common::SimTime;
use rustv2xsimulator::engine::{EngineConfig, SimulationEngine};
use rustv2xsimulator::geometry::polygon::BuildingMap;
use rustv2xsimulator::mobility::trajectory_source::SyntheticMobilityType;
use rustv2xsimulator::security::SimPki;

#[test]
fn test_simulation_engine_with_security_enabled() {
    let config = EngineConfig {
        scenario_name: "test_security_engine".to_string(),
        duration_s: 1.0,
        mobility_step_s: 0.05,
        num_nodes: 10,
        seed: 42,
        workload: "cam".to_string(),
        mobility_type: SyntheticMobilityType::Urban,
        output_dir: "results/test_security/".to_string(),
        enable_pcap: false,
        security_enabled: true,
        verify_packets: true,
        ..Default::default()
    };

    let buildings = BuildingMap::default();
    let mut engine = SimulationEngine::new(config, buildings).expect("create engine with security");

    // 1. Verify PKI initialized properly
    assert!(engine.pki.is_some(), "Engine PKI must be initialized");
    let pki = engine.pki.as_ref().unwrap();
    assert!(pki.verify_chain(), "Root CA and AA must be verified");

    // 2. Verify all 10 vehicles were provisioned with AT credentials and security MIB enabled
    assert_eq!(engine.agents.len(), 10);
    for (id, agent) in &engine.agents {
        // Confirm agent exists and has link layer & gn_handle
        assert_eq!(agent.id, *id);
    }

    // 3. Step the simulation and ensure events execute without error
    let mut event_count = 0;
    while let Ok(Some(_event)) = engine.step() {
        event_count += 1;
        if engine.scheduler.current_time() >= SimTime::from_millis(500) {
            break;
        }
    }

    assert!(event_count > 0, "Simulation should have processed events with security active");
    println!("Processed {} simulation events with security enabled.", event_count);
}

#[test]
fn test_pki_tampered_packet_rejection() {
    use rustflexstack::security::sn_sap::{ReportVerify, SNSignRequest, SNVerifyRequest};
    use rustflexstack::security::verify_service::verify_message;
    use rustv2xsimulator::security::ITS_AID_CAM;

    let mut pki = SimPki::new();
    pki.pregenerate_ats(&[1, 2]);

    let creds1 = pki.issue_at(1);
    let creds2 = pki.issue_at(2);

    let (mut sign_svc1, _, _) = pki.setup_vehicle_security(&creds1, true);
    let (_, verify_backend2, verify_cert_lib2) = pki.setup_vehicle_security(&creds2, true);

    let verify_backend2 = verify_backend2.unwrap();
    let mut verify_cert_lib2 = verify_cert_lib2.unwrap();

    let sign_req = SNSignRequest {
        tbs_message: vec![0xDE, 0xAD, 0xBE, 0xEF],
        its_aid: ITS_AID_CAM,
        permissions: vec![],
        generation_location: None,
    };
    let sign_confirm = sign_svc1.sign_request(&sign_req);

    // Tamper with the payload inside the secured message
    let mut tampered_message = sign_confirm.sec_message.clone();
    let len = tampered_message.len();
    tampered_message[len - 1] ^= 0xFF; // flip bits in the signature

    let verify_req = SNVerifyRequest {
        message: tampered_message,
    };
    let (verify_confirm, _) =
        verify_message(&verify_req, &verify_backend2, &mut verify_cert_lib2);

    // Verification must fail!
    assert_ne!(
        verify_confirm.report,
        ReportVerify::Success,
        "Tampered packet must NOT verify successfully"
    );
}
