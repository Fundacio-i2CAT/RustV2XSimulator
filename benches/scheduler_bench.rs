//! Microbenchmark for Discrete-Event Scheduler and CalendarQueue.

use criterion::{black_box, criterion_group, criterion_main, Criterion};
use rustv2xsimulator::common::SimTime;
use rustv2xsimulator::scheduler::calendar::CalendarQueue;
use rustv2xsimulator::scheduler::des::DiscreteEventScheduler;
use rustv2xsimulator::scheduler::event::EventPayload;

fn bench_calendar_push_pop(c: &mut Criterion) {
    c.bench_function("calendar_queue_10k_events", |b| {
        b.iter(|| {
            let mut cal = CalendarQueue::with_capacity(10_000);
            for i in 0..10_000 {
                let t = SimTime::from_micros(i * 10);
                cal.schedule(t, 0, (i % 100) as u32, EventPayload::Timer { token: i });
            }
            let mut popped = 0;
            while let Some(e) = cal.pop() {
                black_box(e);
                popped += 1;
            }
            assert_eq!(popped, 10_000);
        });
    });
}

fn bench_des_step_throughput(c: &mut Criterion) {
    c.bench_function("des_step_throughput", |b| {
        b.iter(|| {
            let mut des = DiscreteEventScheduler::new();
            for i in 0..10_000 {
                let t = SimTime::from_micros(i * 10);
                des.schedule(t, 0, (i % 50) as u32, EventPayload::Timer { token: i })
                    .unwrap();
            }
            while let Ok(Some(ev)) = des.step() {
                black_box(ev);
            }
        });
    });
}

criterion_group!(benches, bench_calendar_push_pop, bench_des_step_throughput);
criterion_main!(benches);
