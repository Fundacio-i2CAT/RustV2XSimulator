//! Integration Test: ETSI C-ITS Feature Validation.
//!
//! Validates:
//! 1. CAM LowFrequencyContainer cadence (first CAM, then every >= 500 ms).
//! 2. Security signer choice (full certificate initially, digest within 1.0 s, certificate after > 1.0 s).
//! 3. 10 Hz CAM generation for active/moving vehicles.

use rustflexstack::btp::router::Router as BTPRouter;
use rustflexstack::facilities::ca_basic_service::cam_coder::CamCoder;
use rustflexstack::facilities::ca_basic_service::cam_transmission::{
    CAMTransmissionManagement, VehicleData,
};
use rustflexstack::facilities::location_service::GpsFix;
use rustflexstack::geonet::mib::Mib;
use rustflexstack::security::certificate::decode_ieee1609_dot2_data;
use rustflexstack::security::security_asn::ieee1609_dot2::{Ieee1609Dot2Content, SignerIdentifier};
use rustflexstack::security::sn_sap::SNSignRequest;

use rustv2xsimulator::cits::cam::{summarize_cam, CamTriggerEvaluator};
use rustv2xsimulator::common::SimTime;
use rustv2xsimulator::geometry::segment::Point2D;
use rustv2xsimulator::security::SimPki;

#[test]
fn test_cam_low_frequency_container_cadence() {
    let (btp_handle, _) = BTPRouter::spawn(Mib::new());
    let coder = CamCoder::new();
    let vd = VehicleData {
        station_id: 1,
        station_type: 5,
        ..VehicleData::default()
    };
    let ctm = CAMTransmissionManagement::new(btp_handle, coder, vd, None);
    ctm.start();

    let fix = GpsFix {
        latitude: 41.3879,
        longitude: 2.1699,
        altitude_m: 50.0,
        speed_mps: 10.0,
        heading_deg: 90.0,
        pai: true,
    };

    // 1. First CAM (t = 0 ms): MUST include LowFrequencyContainer
    let cam0 = ctm.generate_and_send_cam(&fix, 0, 1).expect("generate CAM 0");
    let summary0 = summarize_cam(&cam0);
    assert!(
        summary0.has_low_frequency_container,
        "First CAM must contain LowFrequencyContainer"
    );

    // 2. Intermediate CAMs (t = 100 ms .. 400 ms at 10 Hz): MUST NOT include LowFrequencyContainer
    for t_ms in [100, 200, 300, 400] {
        let cam = ctm.generate_and_send_cam(&fix, t_ms, 1).expect("generate CAM");
        let summary = summarize_cam(&cam);
        assert!(
            !summary.has_low_frequency_container,
            "Intermediate CAM at t={} ms must NOT contain LowFrequencyContainer",
            t_ms
        );
    }

    // 3. CAM at t = 500 ms (>= 500 ms elapsed since last LF): MUST include LowFrequencyContainer
    let cam500 = ctm.generate_and_send_cam(&fix, 500, 1).expect("generate CAM 500");
    let summary500 = summarize_cam(&cam500);
    assert!(
        summary500.has_low_frequency_container,
        "CAM at t=500 ms must contain LowFrequencyContainer"
    );

    // 4. CAM at t = 600 ms (< 500 ms elapsed since t=500 ms): MUST NOT include LowFrequencyContainer
    let cam600 = ctm.generate_and_send_cam(&fix, 600, 1).expect("generate CAM 600");
    let summary600 = summarize_cam(&cam600);
    assert!(
        !summary600.has_low_frequency_container,
        "CAM at t=600 ms must NOT contain LowFrequencyContainer"
    );
}

#[test]
fn test_cam_security_certificate_vs_digest() {
    let mut pki = SimPki::new();
    let creds = pki.issue_at(42);
    let (mut sign_svc, _, _) = pki.setup_vehicle_security(&creds, true);

    let req = SNSignRequest {
        tbs_message: vec![0xCA, 0x01, 0x02, 0x03],
        its_aid: 36, // CAM
        permissions: vec![],
        generation_location: None,
    };

    // 1. First CAM at t = 100.0s: MUST use full certificate
    let confirm1 = sign_svc.sign_cam_at(&req, Some(100.0));
    let dot2_1 = decode_ieee1609_dot2_data(&confirm1.sec_message);
    if let Ieee1609Dot2Content::signedData(sd) = dot2_1.content {
        assert!(
            matches!(sd.signer, SignerIdentifier::certificate(_)),
            "First CAM must attach full certificate"
        );
    } else {
        panic!("Expected signedData");
    }

    // 2. Next CAM at t = 100.1s (100 ms later, 10 Hz): MUST use digest (<= 1.0s)
    let confirm2 = sign_svc.sign_cam_at(&req, Some(100.1));
    let dot2_2 = decode_ieee1609_dot2_data(&confirm2.sec_message);
    if let Ieee1609Dot2Content::signedData(sd) = dot2_2.content {
        assert!(
            matches!(sd.signer, SignerIdentifier::digest(_)),
            "CAM within 1.0s must attach 8-byte digest"
        );
    } else {
        panic!("Expected signedData");
    }

    // 3. CAM at t = 100.5s: STILL digest
    let confirm3 = sign_svc.sign_cam_at(&req, Some(100.5));
    let dot2_3 = decode_ieee1609_dot2_data(&confirm3.sec_message);
    if let Ieee1609Dot2Content::signedData(sd) = dot2_3.content {
        assert!(
            matches!(sd.signer, SignerIdentifier::digest(_)),
            "CAM at 100.5s must attach 8-byte digest"
        );
    } else {
        panic!("Expected signedData");
    }

    // 4. CAM at t = 101.05s (> 1.0s elapsed): MUST attach full certificate again!
    let confirm4 = sign_svc.sign_cam_at(&req, Some(101.05));
    let dot2_4 = decode_ieee1609_dot2_data(&confirm4.sec_message);
    if let Ieee1609Dot2Content::signedData(sd) = dot2_4.content {
        assert!(
            matches!(sd.signer, SignerIdentifier::certificate(_)),
            "CAM after > 1.0s must attach full certificate again"
        );
    } else {
        panic!("Expected signedData");
    }
}

#[test]
fn test_cam_10hz_kinematic_generation() {
    let mut eval = CamTriggerEvaluator::new(1, true);

    let speed = 15.0; // Moving at 15 m/s
    let heading = 0.0;
    let mut pos = Point2D::new(0.0, 0.0);

    // t = 0 ms -> first CAM
    assert!(
        eval.should_generate(SimTime::from_millis(0), pos, speed, heading),
        "Initial step must trigger CAM"
    );

    // t = 50 ms -> elapsed < t_min (100 ms) -> false
    pos.x += 0.75;
    assert!(
        !eval.should_generate(SimTime::from_millis(50), pos, speed, heading),
        "Step at 50 ms must not trigger (respects 100 ms T_min / 10 Hz)"
    );

    // t = 100 ms -> elapsed = 100 ms, moving vehicle -> true (10 Hz rate)
    pos.x += 0.75;
    assert!(
        eval.should_generate(SimTime::from_millis(100), pos, speed, heading),
        "Step at 100 ms must trigger CAM at 10 Hz"
    );

    // t = 150 ms -> false
    pos.x += 0.75;
    assert!(
        !eval.should_generate(SimTime::from_millis(150), pos, speed, heading),
        "Step at 150 ms must not trigger"
    );

    // t = 200 ms -> true (10 Hz rate)
    pos.x += 0.75;
    assert!(
        eval.should_generate(SimTime::from_millis(200), pos, speed, heading),
        "Step at 200 ms must trigger CAM at 10 Hz"
    );
}

