//! Channel Busy Ratio (CBR) measurement over a sliding evaluation window.
//!
//! Standardized evaluation window: T_cbr = 100.0 ms.

use crate::common::SimTime;
use std::collections::VecDeque;

#[derive(Debug, Clone)]
struct BusyInterval {
    start: SimTime,
    end: SimTime,
}

#[derive(Debug, Clone)]
pub struct ChannelMonitor {
    pub window_duration: SimTime,
    intervals: VecDeque<BusyInterval>,
    current_busy_start: Option<SimTime>,
    last_cbr: f64,
}

impl Default for ChannelMonitor {
    fn default() -> Self {
        Self::new(SimTime::from_millis(100))
    }
}

impl ChannelMonitor {
    pub fn new(window_duration: SimTime) -> Self {
        Self {
            window_duration,
            intervals: VecDeque::new(),
            current_busy_start: None,
            last_cbr: 0.0,
        }
    }

    /// Mark the channel as entering busy state at timestamp `now`.
    pub fn on_channel_busy_start(&mut self, now: SimTime) {
        if self.current_busy_start.is_none() {
            self.current_busy_start = Some(now);
        }
    }

    /// Mark the channel as returning to idle state at timestamp `now`.
    pub fn on_channel_busy_end(&mut self, now: SimTime) {
        if let Some(start) = self.current_busy_start.take() {
            if now > start {
                self.intervals.push_back(BusyInterval { start, end: now });
            }
        }
    }

    /// Record a complete busy interval [start, end].
    pub fn record_busy_interval(&mut self, start: SimTime, end: SimTime) {
        if end > start {
            self.intervals.push_back(BusyInterval { start, end });
        }
    }

    /// Calculate the current Channel Busy Ratio (CBR) over [now - window_duration, now].
    pub fn evaluate_cbr(&mut self, now: SimTime) -> f64 {
        let window_start = if now > self.window_duration {
            now - self.window_duration
        } else {
            SimTime::ZERO
        };

        // Prune intervals that ended before window_start
        while let Some(front) = self.intervals.front() {
            if front.end <= window_start {
                self.intervals.pop_front();
            } else {
                break;
            }
        }

        // Collect and clamp intervals that fall within [window_start, now]
        let mut clamped_intervals: Vec<(u64, u64)> = Vec::with_capacity(self.intervals.len() + 1);
        for intv in &self.intervals {
            let clamped_start = intv.start.max(window_start);
            let clamped_end = intv.end.min(now);
            if clamped_end > clamped_start {
                clamped_intervals.push((clamped_start.ticks(), clamped_end.ticks()));
            }
        }

        // Include ongoing busy interval if active
        if let Some(curr_start) = self.current_busy_start {
            let clamped_start = curr_start.max(window_start);
            if now > clamped_start {
                clamped_intervals.push((clamped_start.ticks(), now.ticks()));
            }
        }

        if clamped_intervals.is_empty() {
            self.last_cbr = 0.0;
            return 0.0;
        }

        // Sort by start tick and merge overlapping intervals to calculate the true union
        clamped_intervals.sort_unstable_by_key(|&(s, _)| s);
        let mut total_busy_ticks = 0u64;
        let mut cur_start = clamped_intervals[0].0;
        let mut cur_end = clamped_intervals[0].1;

        for &(s, e) in &clamped_intervals[1..] {
            if s <= cur_end {
                cur_end = cur_end.max(e);
            } else {
                total_busy_ticks += cur_end.saturating_sub(cur_start);
                cur_start = s;
                cur_end = e;
            }
        }
        total_busy_ticks += cur_end.saturating_sub(cur_start);

        let window_ticks = self.window_duration.ticks().max(1);
        let cbr = (total_busy_ticks as f64 / window_ticks as f64).clamp(0.0, 1.0);
        self.last_cbr = cbr;
        cbr
    }

    #[inline]
    pub fn last_cbr(&self) -> f64 {
        self.last_cbr
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cbr_calculation() {
        let mut monitor = ChannelMonitor::new(SimTime::from_millis(100));

        // 50 ms busy out of 100 ms window
        monitor.record_busy_interval(SimTime::from_millis(20), SimTime::from_millis(70));
        let cbr = monitor.evaluate_cbr(SimTime::from_millis(100));
        assert!((cbr - 0.50).abs() < 1e-4);
    }

    #[test]
    fn test_cbr_overlapping_intervals_no_double_count() {
        let mut monitor = ChannelMonitor::new(SimTime::from_millis(100));

        // Overlapping intervals: [10..50] and [30..70] => merged [10..70] = 60 ms
        monitor.record_busy_interval(SimTime::from_millis(10), SimTime::from_millis(50));
        monitor.record_busy_interval(SimTime::from_millis(30), SimTime::from_millis(70));
        let cbr = monitor.evaluate_cbr(SimTime::from_millis(100));
        assert!((cbr - 0.60).abs() < 1e-4);
    }
}
