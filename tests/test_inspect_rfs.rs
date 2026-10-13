use rustflexstack::btp::router::Router as BTPRouter;
use rustflexstack::facilities::ca_basic_service::{CooperativeAwarenessBasicService, VehicleData};
use rustflexstack::facilities::decentralized_environmental_notification_service::{
    DecentralizedEnvironmentalNotificationService, VehicleData as DenmVehData,
};
use rustflexstack::facilities::local_dynamic_map::LdmFacility;
use rustflexstack::facilities::location_service::{GpsFix, LocationService};
use rustflexstack::geonet::basic_header::{BasicHeader, BasicNH};
use rustflexstack::geonet::gn_address::{GNAddress, M, MID, ST};
use rustflexstack::geonet::mib::Mib;
use rustflexstack::geonet::position_vector::{LongPositionVector, Tst};
use rustv2xsimulator::cits::geonet::GNRouter;

use rustv2xsimulator::cits::cam::{build_cam_btp_request, summarize_cam};
use rustv2xsimulator::cits::denm::create_eebl_request;
use rustv2xsimulator::cits::link_layer::SimulatorLinkLayer;
use rustv2xsimulator::common::{SimTime, Wgs84Coordinate};
use rustv2xsimulator::security::{GnSecurity, SimPki};

#[test]
fn test_rustflexstack_full_stack_roundtrip() {
    let mac_a: [u8; 6] = [0x02, 0x00, 0x00, 0x00, 0x00, 0x01];
    let mac_b: [u8; 6] = [0x02, 0x00, 0x00, 0x00, 0x00, 0x02];

    // ── Vehicle A Setup ──
    let mut mib_a = Mib::new();
    mib_a.itsGnLocalGnAddr = GNAddress::new(M::GnMulticast, ST::PassengerCar, MID::new(mac_a));
    mib_a.itsGnBeaconServiceRetransmitTimer = 0;

    let (gn_handle_a, gn_to_ll_rx_a, gn_to_btp_rx_a) =
        GNRouter::spawn(mib_a, None, None, None);
    let (btp_handle_a, btp_to_gn_rx_a) = BTPRouter::spawn(mib_a);
    let link_layer_a = SimulatorLinkLayer::new(
        1,
        gn_to_ll_rx_a,
        gn_handle_a.clone(),
        gn_to_btp_rx_a,
        btp_handle_a.clone(),
        btp_to_gn_rx_a,
        false,
    );
    let ldm_a = LdmFacility::new(413_851_000, 21_734_000, 2000.0);
    let mut loc_svc_a = LocationService::new();
    let gps_rx_a = loc_svc_a.subscribe();
    let veh_data_a = VehicleData {
        station_id: 1,
        station_type: 5,
        ..VehicleData::default()
    };
    let (ca_svc_a, _cam_rx_a) = CooperativeAwarenessBasicService::new(
        btp_handle_a.clone(),
        veh_data_a.clone(),
        Some(ldm_a),
    );
    ca_svc_a.start(gps_rx_a);

    let (den_svc_a, _denm_rx_a) = DecentralizedEnvironmentalNotificationService::new(
        btp_handle_a.clone(),
        DenmVehData {
            station_id: 1,
            station_type: 5,
        },
    );

    // ── Vehicle B Setup ──
    let mut mib_b = Mib::new();
    mib_b.itsGnLocalGnAddr = GNAddress::new(M::GnMulticast, ST::PassengerCar, MID::new(mac_b));
    mib_b.itsGnBeaconServiceRetransmitTimer = 0;

    let (gn_handle_b, gn_to_ll_rx_b, gn_to_btp_rx_b) =
        GNRouter::spawn(mib_b, None, None, None);
    let (btp_handle_b, btp_to_gn_rx_b) = BTPRouter::spawn(mib_b);
    let link_layer_b = SimulatorLinkLayer::new(
        2,
        gn_to_ll_rx_b,
        gn_handle_b.clone(),
        gn_to_btp_rx_b,
        btp_handle_b.clone(),
        btp_to_gn_rx_b,
        false,
    );
    let ldm_b = LdmFacility::new(413_851_000, 21_734_000, 2000.0);
    let mut loc_svc_b = LocationService::new();
    let gps_rx_b = loc_svc_b.subscribe();
    let veh_data_b = VehicleData {
        station_id: 2,
        station_type: 5,
        ..VehicleData::default()
    };
    let (ca_svc_b, cam_rx_b) =
        CooperativeAwarenessBasicService::new(btp_handle_b.clone(), veh_data_b, Some(ldm_b));
    ca_svc_b.start(gps_rx_b);

    let (_den_svc_b, denm_rx_b) = DecentralizedEnvironmentalNotificationService::new(
        btp_handle_b.clone(),
        DenmVehData {
            station_id: 2,
            station_type: 5,
        },
    );

    // 1. Trigger a CAM transmission from Vehicle A
    let fix_a = GpsFix {
        latitude: 41.3851,
        longitude: 2.1734,
        altitude_m: 10.0,
        speed_mps: 15.0,
        heading_deg: 90.0,
        pai: true,
    };
    loc_svc_a.publish(fix_a);

    let req = build_cam_btp_request(&veh_data_a, &fix_a, 0.0).expect("build cam request");
    btp_handle_a.send_btp_data_request(req);

    // Allow actor thread to process
    std::thread::sleep(std::time::Duration::from_millis(50));
    link_layer_a.drain_bridges();
    std::thread::sleep(std::time::Duration::from_millis(50));

    let frames_a = link_layer_a.drain_outbound_frames(SimTime::from_millis(100));
    assert!(
        !frames_a.is_empty(),
        "Vehicle A should have generated at least 1 outbound GeoNetworking frame"
    );

    let raw_gn_pkt = &frames_a[0].1.payload;
    println!("Drained GeoNetworking packet: {} bytes", raw_gn_pkt.len());

    // 2. Deliver over the simulated channel to Vehicle B
    link_layer_b.inject_rx(raw_gn_pkt);
    for _ in 0..10 {
        link_layer_b.drain_bridges();
        std::thread::sleep(std::time::Duration::from_millis(10));
    }

    // 3. Verify Vehicle B decoded the CAM
    let received_cam = cam_rx_b.recv_timeout(std::time::Duration::from_millis(500));
    assert!(received_cam.is_ok(), "Vehicle B must receive decoded CAM!");
    let cam = received_cam.unwrap();
    let summary = summarize_cam(&cam);
    assert_eq!(summary.station_id, 1);
    println!(
        "Vehicle B received CAM from Station ID: {}",
        summary.station_id
    );

    // 4. Trigger a DENM transmission from Vehicle A
    let coord_a = Wgs84Coordinate::new(41.3851, 2.1734, 10.0);
    let den_req = create_eebl_request(&coord_a, 15.0, 90.0);
    den_svc_a.send_single_denm(&den_req);

    // IMPORTANT: Update Vehicle B's ego position vector to Barcelona coordinates
    // so it lies inside the DENM geo-broadcast circle (radius 1000m centred at
    // 41.3851° N, 2.1734° E). Without this, the GN router's gn_geometric_function_f
    // returns < 0 for the default (0,0) location and does NOT deliver the packet.
    let lpv_b = LongPositionVector {
        gn_addr: GNAddress::new(M::GnMulticast, ST::PassengerCar, MID::new(mac_b)),
        tst: Tst::set_in_normal_timestamp_milliseconds(100),
        // 41.3852° N, 2.1735° E — 11 m from event origin, well inside 1 km circle.
        latitude: ((41.3852_f64 * 1e7).round() as u32),
        longitude: ((2.1735_f64 * 1e7).round() as u32),
        pai: true,
        s: 1500, // 15 m/s → 1500 cm/s
        h: 900,  // 90.0° → 900 tenths
    };
    link_layer_b.gn_handle().update_position_vector(lpv_b);

    std::thread::sleep(std::time::Duration::from_millis(50));
    link_layer_a.drain_bridges();
    std::thread::sleep(std::time::Duration::from_millis(50));

    let denm_frames = link_layer_a.drain_outbound_frames(SimTime::from_millis(200));
    assert!(
        !denm_frames.is_empty(),
        "Vehicle A should have generated outbound DENM frame"
    );

    println!("Drained {} frame(s) after DENM trigger", denm_frames.len());
    for (_, frame) in &denm_frames {
        link_layer_b.inject_rx(&frame.payload);
    }
    for _ in 0..15 {
        link_layer_b.drain_bridges();
        std::thread::sleep(std::time::Duration::from_millis(10));
    }

    let received_denm = denm_rx_b.recv_timeout(std::time::Duration::from_millis(500));
    assert!(
        received_denm.is_ok(),
        "Vehicle B must receive decoded DENM!"
    );
    let denm = received_denm.unwrap();
    assert_eq!(denm.header.station_id.0, 1);
    println!(
        "Vehicle B received DENM from Station ID: {}",
        denm.header.station_id.0
    );
}

#[test]
fn test_rustflexstack_secured_roundtrip() {
    let mac_a: [u8; 6] = [0x02, 0x00, 0x00, 0x00, 0x00, 0x01];
    let mac_b: [u8; 6] = [0x02, 0x00, 0x00, 0x00, 0x00, 0x02];

    // ── PKI Hierarchy Setup (Root CA → AA → ATs) ──
    let mut pki = SimPki::new();
    pki.pregenerate_ats(&[1, 2]);
    let creds_a = pki.issue_at(1);
    let creds_b = pki.issue_at(2);

    // ── Vehicle A Setup (Secured) ──
    let mut mib_a = Mib::new();
    mib_a.itsGnLocalGnAddr = GNAddress::new(M::GnMulticast, ST::PassengerCar, MID::new(mac_a));
    mib_a.itsGnBeaconServiceRetransmitTimer = 0;
    mib_a.itsGnSecurity = GnSecurity::Enabled;

    let (sign_svc_a, verify_backend_a, verify_cert_lib_a) =
        pki.setup_vehicle_security(&creds_a, true);
    let (gn_handle_a, gn_to_ll_rx_a, gn_to_btp_rx_a) =
        GNRouter::spawn(mib_a, Some(sign_svc_a), verify_backend_a, verify_cert_lib_a);
    let (btp_handle_a, btp_to_gn_rx_a) = BTPRouter::spawn(mib_a);
    let link_layer_a = SimulatorLinkLayer::new(
        1,
        gn_to_ll_rx_a,
        gn_handle_a.clone(),
        gn_to_btp_rx_a,
        btp_handle_a.clone(),
        btp_to_gn_rx_a,
        false,
    );
    let ldm_a = LdmFacility::new(413_851_000, 21_734_000, 2000.0);
    let mut loc_svc_a = LocationService::new();
    let gps_rx_a = loc_svc_a.subscribe();
    let veh_data_a = VehicleData {
        station_id: 1,
        station_type: 5,
        ..VehicleData::default()
    };
    let (ca_svc_a, _cam_rx_a) = CooperativeAwarenessBasicService::new(
        btp_handle_a.clone(),
        veh_data_a.clone(),
        Some(ldm_a),
    );
    ca_svc_a.start(gps_rx_a);

    let (den_svc_a, _denm_rx_a) = DecentralizedEnvironmentalNotificationService::new(
        btp_handle_a.clone(),
        DenmVehData {
            station_id: 1,
            station_type: 5,
        },
    );

    // ── Vehicle B Setup (Secured Verification) ──
    let mut mib_b = Mib::new();
    mib_b.itsGnLocalGnAddr = GNAddress::new(M::GnMulticast, ST::PassengerCar, MID::new(mac_b));
    mib_b.itsGnBeaconServiceRetransmitTimer = 0;
    mib_b.itsGnSecurity = GnSecurity::Enabled;

    let (sign_svc_b, verify_backend_b, verify_cert_lib_b) =
        pki.setup_vehicle_security(&creds_b, true);
    let (gn_handle_b, gn_to_ll_rx_b, gn_to_btp_rx_b) =
        GNRouter::spawn(mib_b, Some(sign_svc_b), verify_backend_b, verify_cert_lib_b);
    let (btp_handle_b, btp_to_gn_rx_b) = BTPRouter::spawn(mib_b);
    let link_layer_b = SimulatorLinkLayer::new(
        2,
        gn_to_ll_rx_b,
        gn_handle_b.clone(),
        gn_to_btp_rx_b,
        btp_handle_b.clone(),
        btp_to_gn_rx_b,
        false,
    );
    let ldm_b = LdmFacility::new(413_851_000, 21_734_000, 2000.0);
    let mut loc_svc_b = LocationService::new();
    let gps_rx_b = loc_svc_b.subscribe();
    let veh_data_b = VehicleData {
        station_id: 2,
        station_type: 5,
        ..VehicleData::default()
    };
    let (ca_svc_b, cam_rx_b) =
        CooperativeAwarenessBasicService::new(btp_handle_b.clone(), veh_data_b, Some(ldm_b));
    ca_svc_b.start(gps_rx_b);

    let (_den_svc_b, denm_rx_b) = DecentralizedEnvironmentalNotificationService::new(
        btp_handle_b.clone(),
        DenmVehData {
            station_id: 2,
            station_type: 5,
        },
    );

    // 1. Trigger a secured CAM transmission from Vehicle A
    let fix_a = GpsFix {
        latitude: 41.3851,
        longitude: 2.1734,
        altitude_m: 10.0,
        speed_mps: 15.0,
        heading_deg: 90.0,
        pai: true,
    };
    loc_svc_a.publish(fix_a);

    let req = build_cam_btp_request(&veh_data_a, &fix_a, 0.0).expect("build cam request");
    btp_handle_a.send_btp_data_request(req);

    // Allow actor thread to process signing
    std::thread::sleep(std::time::Duration::from_millis(50));
    link_layer_a.drain_bridges();
    std::thread::sleep(std::time::Duration::from_millis(50));

    let frames_a = link_layer_a.drain_outbound_frames(SimTime::from_millis(100));
    assert!(
        !frames_a.is_empty(),
        "Vehicle A should have generated at least 1 outbound GeoNetworking frame"
    );

    let raw_gn_pkt = &frames_a[0].1.payload;
    println!("Drained Secured GeoNetworking packet: {} bytes", raw_gn_pkt.len());

    // Verify that packet header specifies SecuredPacket
    assert!(raw_gn_pkt.len() >= 4);
    let bh = BasicHeader::decode([raw_gn_pkt[0], raw_gn_pkt[1], raw_gn_pkt[2], raw_gn_pkt[3]]);
    assert!(
        matches!(bh.nh, BasicNH::SecuredPacket),
        "Outbound GeoNetworking packet next header must be SecuredPacket (0x02)"
    );

    // 2. Deliver over the simulated channel to Vehicle B
    link_layer_b.inject_rx(raw_gn_pkt);
    for _ in 0..10 {
        link_layer_b.drain_bridges();
        std::thread::sleep(std::time::Duration::from_millis(10));
    }

    // 3. Verify Vehicle B successfully verified and decoded the CAM
    let received_cam = cam_rx_b.recv_timeout(std::time::Duration::from_millis(500));
    assert!(received_cam.is_ok(), "Vehicle B must verify signature and receive decoded CAM!");
    let cam = received_cam.unwrap();
    let summary = summarize_cam(&cam);
    assert_eq!(summary.station_id, 1);
    println!(
        "Vehicle B verified & received CAM from Station ID: {}",
        summary.station_id
    );

    // 4. Trigger a secured DENM transmission from Vehicle A
    let coord_a = Wgs84Coordinate::new(41.3851, 2.1734, 10.0);
    let den_req = create_eebl_request(&coord_a, 15.0, 90.0);
    den_svc_a.send_single_denm(&den_req);

    // Update Vehicle B's position vector so it lies inside the DENM broadcast circle
    let lpv_b = LongPositionVector {
        gn_addr: GNAddress::new(M::GnMulticast, ST::PassengerCar, MID::new(mac_b)),
        tst: Tst::set_in_normal_timestamp_milliseconds(100),
        latitude: ((41.3852_f64 * 1e7).round() as u32),
        longitude: ((2.1735_f64 * 1e7).round() as u32),
        pai: true,
        s: 1500,
        h: 900,
    };
    link_layer_b.gn_handle().update_position_vector(lpv_b);

    std::thread::sleep(std::time::Duration::from_millis(50));
    link_layer_a.drain_bridges();
    std::thread::sleep(std::time::Duration::from_millis(50));

    let denm_frames = link_layer_a.drain_outbound_frames(SimTime::from_millis(200));
    assert!(
        !denm_frames.is_empty(),
        "Vehicle A should have generated outbound secured DENM frame"
    );

    let raw_denm_pkt = &denm_frames[0].1.payload;
    assert!(raw_denm_pkt.len() >= 4);
    let denm_bh = BasicHeader::decode([raw_denm_pkt[0], raw_denm_pkt[1], raw_denm_pkt[2], raw_denm_pkt[3]]);
    assert!(
        matches!(denm_bh.nh, BasicNH::SecuredPacket),
        "Outbound DENM packet next header must be SecuredPacket (0x02)"
    );

    for (_, frame) in &denm_frames {
        link_layer_b.inject_rx(&frame.payload);
    }
    for _ in 0..15 {
        link_layer_b.drain_bridges();
        std::thread::sleep(std::time::Duration::from_millis(10));
    }

    let received_denm = denm_rx_b.recv_timeout(std::time::Duration::from_millis(500));
    assert!(
        received_denm.is_ok(),
        "Vehicle B must verify signature and receive decoded DENM!"
    );
    let denm = received_denm.unwrap();
    assert_eq!(denm.header.station_id.0, 1);
    println!(
        "Vehicle B verified & received DENM from Station ID: {}",
        denm.header.station_id.0
    );
}
