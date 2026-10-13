//! Microbenchmark for Physical Layer Chunked SINR and Rayon Link Evaluation.

use criterion::{black_box, criterion_group, criterion_main, Criterion};
use rand::rngs::StdRng;
use rand::SeedableRng;

use rustv2xsimulator::common::SimTime;
use rustv2xsimulator::geometry::polygon::BuildingMap;
use rustv2xsimulator::geometry::segment::Point2D;
use rustv2xsimulator::phy::air_packet::AirPacket;
use rustv2xsimulator::phy::interference_helper::{
    dbm_to_watts, CumulativeInterferenceTracker, SignalEvent,
};
use rustv2xsimulator::phy::radio_medium::{PhyModelType, RadioMedium};

fn bench_chunked_sinr_helper(c: &mut Criterion) {
    let tracker = CumulativeInterferenceTracker::default();
    let mut rng = StdRng::seed_from_u64(42);

    let t0 = SimTime::ZERO;
    let t_end = SimTime::from_micros(300);

    let desired = SignalEvent {
        signal_id: 1,
        rx_power_dbm: -75.0,
        rx_power_watts: dbm_to_watts(-75.0),
        start_time: t0,
        end_time: t_end,
        is_desired: true,
    };

    // 10 concurrent interfering transmissions
    let mut interferers = Vec::new();
    for i in 0..10 {
        let t_s = SimTime::from_micros(i * 25);
        let t_e = SimTime::from_micros(i * 25 + 250);
        interferers.push(SignalEvent {
            signal_id: i + 2,
            rx_power_dbm: -82.0 + (i as f64),
            rx_power_watts: dbm_to_watts(-82.0 + (i as f64)),
            start_time: t_s,
            end_time: t_e,
            is_desired: false,
        });
    }

    c.bench_function("chunked_sinr_10_interferers", |b| {
        b.iter(|| {
            let res = tracker.evaluate_reception(
                black_box(&desired),
                black_box(&interferers),
                300,
                &mut rng,
            );
            black_box(res);
        });
    });
}

fn bench_rayon_broadcast_evaluation(c: &mut Criterion) {
    let medium = RadioMedium::new(PhyModelType::DualSlope, 23.0);
    let buildings = BuildingMap::default();

    let packet = AirPacket::new(
        1,
        1,
        23.0,
        SimTime::ZERO,
        SimTime::from_micros(300),
        vec![0u8; 300],
        2,
    );

    let sender_pos = Point2D::new(0.0, 0.0);
    let mut receivers = Vec::with_capacity(500);
    for i in 0..500 {
        let d = (i as f64) * 1.5 + 5.0;
        receivers.push(((i + 2) as u32, Point2D::new(d, 0.0), d));
    }

    c.bench_function("rayon_broadcast_500_receivers", |b| {
        b.iter(|| {
            let results = medium.evaluate_broadcast(
                black_box(&packet),
                black_box(sender_pos),
                black_box(&receivers),
                black_box(&buildings),
                42,
            );
            black_box(results);
        });
    });
}

criterion_group!(
    benches,
    bench_chunked_sinr_helper,
    bench_rayon_broadcast_evaluation
);
criterion_main!(benches);
