//! Decoupled Wall-Clock Profiler.
//!
//! Provides hardware-timestamped profiling via `_rdtsc` CPU instruction timers
//! (with an `Instant` fallback) with non-overlapping attribution into:
//! - T_mob: Microscopic mobility updates and coordinate ingestion
//! - T_stack: ETSI C-ITS serialization, trigger evaluations, BTP/GeoNet routing
//! - T_chan: Spatial indexing, Rayon ray-casting, chunked SINR, DES calendar operations
//!
//! Exposes isolated network throughput:
//! Theta_net = |E_net| / (T_stack + T_chan)

use serde::{Deserialize, Serialize};
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Instant;

#[inline(always)]
pub fn read_tsc() -> u64 {
    #[cfg(target_arch = "x86_64")]
    {
        unsafe { core::arch::x86_64::_rdtsc() }
    }
    #[cfg(not(target_arch = "x86_64"))]
    {
        // Fallback: nanoseconds as ticks
        lazy_static_start_time().elapsed().as_nanos() as u64
    }
}

#[cfg(not(target_arch = "x86_64"))]
fn lazy_static_start_time() -> Instant {
    static START: std::sync::OnceLock<Instant> = std::sync::OnceLock::new();
    *START.get_or_init(Instant::now)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProfileCategory {
    Mobility,
    Stack,
    Channel,
}

#[derive(Debug, Default)]
pub struct DecoupledWallClockProfiler {
    t_mob_nanos: AtomicU64,
    t_stack_nanos: AtomicU64,
    t_chan_nanos: AtomicU64,
    net_events_count: AtomicU64,
    total_events_count: AtomicU64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProfilerSummary {
    pub t_mob_seconds: f64,
    pub t_stack_seconds: f64,
    pub t_chan_seconds: f64,
    pub t_total_seconds: f64,
    pub t_mob_percent: f64,
    pub t_stack_percent: f64,
    pub t_chan_percent: f64,
    pub net_events_count: u64,
    pub total_events_count: u64,
    pub theta_net_events_per_sec: f64,
    pub overall_events_per_sec: f64,
}

impl DecoupledWallClockProfiler {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn start_timer(&self, category: ProfileCategory) -> ScopedProfileGuard<'_> {
        ScopedProfileGuard {
            profiler: self,
            category,
            start_instant: Instant::now(),
        }
    }

    pub fn record_nanos(&self, category: ProfileCategory, nanos: u64) {
        match category {
            ProfileCategory::Mobility => {
                self.t_mob_nanos.fetch_add(nanos, Ordering::Relaxed);
            }
            ProfileCategory::Stack => {
                self.t_stack_nanos.fetch_add(nanos, Ordering::Relaxed);
            }
            ProfileCategory::Channel => {
                self.t_chan_nanos.fetch_add(nanos, Ordering::Relaxed);
            }
        }
    }

    pub fn record_net_event(&self) {
        self.net_events_count.fetch_add(1, Ordering::Relaxed);
        self.total_events_count.fetch_add(1, Ordering::Relaxed);
    }

    pub fn record_event(&self) {
        self.total_events_count.fetch_add(1, Ordering::Relaxed);
    }

    pub fn summary(&self) -> ProfilerSummary {
        let t_mob_ns = self.t_mob_nanos.load(Ordering::Relaxed);
        let t_stack_ns = self.t_stack_nanos.load(Ordering::Relaxed);
        let t_chan_ns = self.t_chan_nanos.load(Ordering::Relaxed);
        let total_ns = t_mob_ns + t_stack_ns + t_chan_ns;

        let t_mob_s = t_mob_ns as f64 * 1e-9;
        let t_stack_s = t_stack_ns as f64 * 1e-9;
        let t_chan_s = t_chan_ns as f64 * 1e-9;
        let t_total_s = total_ns as f64 * 1e-9;

        let total_denom = if total_ns > 0 { total_ns as f64 } else { 1.0 };
        let t_mob_percent = (t_mob_ns as f64 / total_denom) * 100.0;
        let t_stack_percent = (t_stack_ns as f64 / total_denom) * 100.0;
        let t_chan_percent = (t_chan_ns as f64 / total_denom) * 100.0;

        let net_events = self.net_events_count.load(Ordering::Relaxed);
        let total_events = self.total_events_count.load(Ordering::Relaxed);

        let net_denom_s = t_stack_s + t_chan_s;
        let theta_net = if net_denom_s > 0.0 {
            net_events as f64 / net_denom_s
        } else {
            0.0
        };

        let overall_throughput = if t_total_s > 0.0 {
            total_events as f64 / t_total_s
        } else {
            0.0
        };

        ProfilerSummary {
            t_mob_seconds: t_mob_s,
            t_stack_seconds: t_stack_s,
            t_chan_seconds: t_chan_s,
            t_total_seconds: t_total_s,
            t_mob_percent,
            t_stack_percent,
            t_chan_percent,
            net_events_count: net_events,
            total_events_count: total_events,
            theta_net_events_per_sec: theta_net,
            overall_events_per_sec: overall_throughput,
        }
    }
}

pub struct ScopedProfileGuard<'a> {
    profiler: &'a DecoupledWallClockProfiler,
    category: ProfileCategory,
    start_instant: Instant,
}

impl<'a> Drop for ScopedProfileGuard<'a> {
    fn drop(&mut self) {
        let elapsed = self.start_instant.elapsed().as_nanos() as u64;
        self.profiler.record_nanos(self.category, elapsed);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_profiler_timing() {
        let profiler = DecoupledWallClockProfiler::new();
        {
            let _guard = profiler.start_timer(ProfileCategory::Mobility);
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
        profiler.record_net_event();
        let summary = profiler.summary();
        assert!(summary.t_mob_seconds > 0.0);
        assert_eq!(summary.net_events_count, 1);
    }
}
