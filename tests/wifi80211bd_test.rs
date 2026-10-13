//! Integration Test: IEEE 802.11bd Next Generation V2X (NGV).
//!
//! Standards Reference: IEEE Std 802.11bd-2022
//!
//! Verifies:
//! 1. Standard 10 MHz and bonded 20 MHz channel modes (MCS 0 to MCS 9).
//! 2. Mixed-format PPDU duration computation with periodic midambles.
//! 3. L-SIG duration spoofing for legacy IEEE 802.11p backward compatibility.
//! 4. LDPC coding gain: ~2.5 dB advantage over 802.11p BCC at identical payload and MCS.
//! 5. Dual Carrier Modulation (DCM): ~2.0 dB frequency diversity gain at low SNR.
//! 6. Vehicular Doppler mitigation: midambles eliminate high-speed Doppler error floors at 220 km/h.
//! 7. Seamless `WirelessTechnology` trait abstraction integration.

use rand::rngs::StdRng;
use rand::SeedableRng;

use rustv2xsimulator::common::SimTime;
use rustv2xsimulator::geometry::polygon::BuildingMap;
use rustv2xsimulator::geometry::segment::Point2D;
use rustv2xsimulator::mac::edca::TxFrame;
use rustv2xsimulator::phy::air_packet::AirPacket;
use rustv2xsimulator::phy::consts_80211p::{compute_duration_80211p, Mcs80211p};
use rustv2xsimulator::phy::decider_80211p::Decider80211p;
use rustv2xsimulator::phy::technology::{Wifi80211bdModel, WirelessTechnology};
use rustv2xsimulator::phy::wifi80211bd::config::Wifi80211bdConfig;
use rustv2xsimulator::phy::wifi80211bd::consts_80211bd::{
    compute_duration_80211bd, compute_lsig_length_spoofed, Mcs80211bd, MidamblePeriod,
};
use rustv2xsimulator::phy::wifi80211bd::decider_80211bd::Decider80211bd;

#[test]
fn test_80211bd_bandwidth_and_mcs_scaling() {
    // 10 MHz channel bitrates
    assert_eq!(Mcs80211bd::Mcs0Bpsk1_2.bitrate_10mhz_bps(false), 3_000_000);
    assert_eq!(Mcs80211bd::Mcs1Qpsk1_2.bitrate_10mhz_bps(false), 6_000_000);
    assert_eq!(Mcs80211bd::Mcs2Qpsk3_4.bitrate_10mhz_bps(false), 9_000_000);
    assert_eq!(Mcs80211bd::Mcs6Qam64_3_4.bitrate_10mhz_bps(false), 27_000_000);
    assert_eq!(Mcs80211bd::Mcs8Qam256_3_4.bitrate_10mhz_bps(false), 36_000_000);
    assert_eq!(Mcs80211bd::Mcs9Qam256_5_6.bitrate_10mhz_bps(false), 40_000_000);

    // 20 MHz bonded channel bitrates (exact 2x throughput)
    assert_eq!(Mcs80211bd::Mcs0Bpsk1_2.bitrate_20mhz_bps(false), 6_000_000);
    assert_eq!(Mcs80211bd::Mcs1Qpsk1_2.bitrate_20mhz_bps(false), 12_000_000);
    assert_eq!(Mcs80211bd::Mcs8Qam256_3_4.bitrate_20mhz_bps(false), 72_000_000);
    assert_eq!(Mcs80211bd::Mcs9Qam256_5_6.bitrate_20mhz_bps(false), 80_000_000);
}

#[test]
fn test_80211bd_ppdu_duration_and_lsig_spoofing() {
    let payload_bytes = 300;
    let mcs = Mcs80211bd::Mcs2Qpsk3_4; // 9 Mbps in 10 MHz

    // Without midambles:
    let dur_no_mid = compute_duration_80211bd(payload_bytes, mcs, 10.0, MidamblePeriod::None, false);

    // With midambles every 4 symbols:
    let dur_mid4 = compute_duration_80211bd(payload_bytes, mcs, 10.0, MidamblePeriod::Every4Symbols, false);

    assert!(dur_mid4 > dur_no_mid);
    // Overhead should be modest (typically ~15-20%)
    let overhead_pct = ((dur_mid4.to_micros() - dur_no_mid.to_micros()) as f64) / (dur_no_mid.to_micros() as f64);
    assert!(overhead_pct < 0.25, "Midamble overhead should not exceed 25%");

    // Verify L-SIG spoofing: legacy station calculating duration from spoofed length
    // should arrive at a duration matching or slightly exceeding dur_mid4
    let spoofed_length = compute_lsig_length_spoofed(dur_mid4, 10.0);
    let legacy_dur = compute_duration_80211p(spoofed_length as usize, Mcs80211p::BpskR1_2);
    assert!(
        legacy_dur.to_micros() >= dur_mid4.to_micros().saturating_sub(8),
        "Legacy 802.11p node must compute equal or protective duration: legacy={}us, actual={}us",
        legacy_dur.to_micros(),
        dur_mid4.to_micros()
    );
}

#[test]
fn test_ldpc_coding_gain_vs_80211p_bcc() {
    let mut rng = StdRng::seed_from_u64(12345);
    let decider_11p = Decider80211p::default();
    let decider_11bd = Decider80211bd::new(10.0, 5.890e9);

    // At SINR = 4.8 dB, for a 300-byte CAM at QPSK 1/2:
    // - 802.11p (BCC Viterbi) requires ~6.5 - 7.0 dB, so at 4.8 dB it fails almost all packets.
    // - 802.11bd (LDPC) threshold is 4.0 dB, so at 4.8 dB it decodes successfully.
    let rx_power = -75.0; // Well above sensitivity
    let sinr = 4.8;
    let payload_len = 300;

    let mut success_11p = 0;
    let mut success_11bd = 0;
    let trials = 100;

    for _ in 0..trials {
        let res_11p = decider_11p.evaluate_packet(
            rx_power,
            sinr,
            payload_len,
            Mcs80211p::QpskR1_2,
            &mut rng,
        );
        if res_11p.is_success {
            success_11p += 1;
        }

        let res_11bd = decider_11bd.evaluate_packet(
            rx_power,
            sinr,
            payload_len,
            Mcs80211bd::Mcs1Qpsk1_2,
            MidamblePeriod::Every8Symbols,
            false,
            0.0, // Stationary (isolate coding gain)
            &mut rng,
        );
        if res_11bd.is_success {
            success_11bd += 1;
        }
    }

    assert!(
        success_11bd >= 80,
        "802.11bd LDPC must decode reliably at 4.8 dB: got {}/{}",
        success_11bd,
        trials
    );
    assert!(
        success_11p <= 20,
        "802.11p BCC must struggle at 4.8 dB due to higher SNR threshold: got {}/{}",
        success_11p,
        trials
    );
}

#[test]
fn test_dual_carrier_modulation_frequency_diversity_gain() {
    let mut rng = StdRng::seed_from_u64(999);
    let decider = Decider80211bd::new(10.0, 5.890e9);

    // Test at low SINR = 0.0 dB for 300-byte payload:
    // Standard MCS 0 (threshold 1.5 dB) is 1.5 dB below threshold (fails).
    // DCM MCS 0 (threshold -0.5 dB) is 0.5 dB above threshold (succeeds).
    let mut wins_std = 0;
    let mut wins_dcm = 0;
    let trials = 50;

    for _ in 0..trials {
        let res_std = decider.evaluate_packet(
            -80.0,
            0.0,
            300,
            Mcs80211bd::Mcs0Bpsk1_2,
            MidamblePeriod::Every8Symbols,
            false, // DCM disabled
            0.0,
            &mut rng,
        );
        if res_std.is_success {
            wins_std += 1;
        }

        let res_dcm = decider.evaluate_packet(
            -80.0,
            0.0,
            300,
            Mcs80211bd::Mcs0Bpsk1_2,
            MidamblePeriod::Every8Symbols,
            true, // DCM enabled
            0.0,
            &mut rng,
        );
        if res_dcm.is_success {
            wins_dcm += 1;
        }
    }

    assert!(wins_dcm >= 40, "DCM must succeed at 0.0 dB: got {}/{}", wins_dcm, trials);
    assert!(wins_std <= 10, "Standard MCS 0 must fail at 0.0 dB: got {}/{}", wins_std, trials);
}

#[test]
fn test_high_speed_highway_doppler_mitigation_with_midambles() {
    let mut rng = StdRng::seed_from_u64(777);
    let decider = Decider80211bd::new(10.0, 5.890e9);

    // High speed highway closing scenario:
    // Relative speed 220 km/h = 61.1 m/s (Doppler spread ~1200 Hz)
    let v_rel = 61.1;
    let rx_power = -68.0;
    let raw_sinr = 22.0; // High raw SINR (would easily succeed if stationary)
    let payload_len = 600; // 600-byte message (longer packet = longer channel aging)

    // Evaluate without midambles (simulating 802.11p single-preamble channel estimation)
    let mut success_no_mid = 0;
    // Evaluate with 802.11bd midambles every 4 symbols
    let mut success_mid4 = 0;
    let trials = 50;

    for _ in 0..trials {
        let res_no_mid = decider.evaluate_packet(
            rx_power,
            raw_sinr,
            payload_len,
            Mcs80211bd::Mcs2Qpsk3_4,
            MidamblePeriod::None,
            false,
            v_rel,
            &mut rng,
        );
        if res_no_mid.is_success {
            success_no_mid += 1;
        }

        let res_mid4 = decider.evaluate_packet(
            rx_power,
            raw_sinr,
            payload_len,
            Mcs80211bd::Mcs2Qpsk3_4,
            MidamblePeriod::Every4Symbols,
            false,
            v_rel,
            &mut rng,
        );
        if res_mid4.is_success {
            success_mid4 += 1;
        }
    }

    assert!(
        success_mid4 >= 45,
        "Midambles (Every4Symbols) must preserve high packet reception at 220 km/h: got {}/{}",
        success_mid4,
        trials
    );
    assert!(
        success_no_mid <= 10,
        "Without midambles, high-speed Doppler error floor must cause heavy packet drop: got {}/{}",
        success_no_mid,
        trials
    );
}

#[test]
fn test_80211bd_wireless_technology_trait_integration() {
    let mut model = Wifi80211bdModel::new(Wifi80211bdConfig::default(), 42);
    let mut rng = StdRng::seed_from_u64(100);

    assert_eq!(model.name(), "IEEE_802_11bd_NGV");

    // Enqueue frame
    let frame = TxFrame {
        frame_id: 1,
        payload: vec![0u8; 300],
        created_time: SimTime::from_millis(10),
        duration: SimTime::from_micros(376),
    };
    let tx_time = model.enqueue_outbound(frame, 0, SimTime::from_millis(10), &mut rng);
    assert!(tx_time.is_some());
    assert!(tx_time.unwrap() >= SimTime::from_millis(10));

    // Evaluate broadcast
    let pkt = AirPacket::new(
        1,
        10,
        23.0,
        SimTime::from_millis(10),
        SimTime::from_micros(376),
        vec![0u8; 300],
        0,
    );
    let sender_pos = Point2D::new(0.0, 0.0);
    let receivers = vec![
        (1, Point2D::new(50.0, 0.0), 50.0),
        (2, Point2D::new(100.0, 0.0), 100.0),
    ];
    let buildings = BuildingMap::default();

    let results = model.evaluate_transmission(
        &pkt,
        sender_pos,
        &receivers,
        &buildings,
        SimTime::from_millis(10),
        &mut rng,
    );
    assert_eq!(results.len(), 2);
    assert!(results[0].is_success);

    // Channel activity and CBR
    model.record_channel_activity(SimTime::from_millis(10), SimTime::from_millis(20));
    let cbr = model.evaluate_cbr(SimTime::from_millis(100));
    assert!(cbr > 0.0);
}
