//! High-resolution simulation time with 100-nanosecond tick quantization.
//!
//! Resolution: 1 tick = 100 nanoseconds = 1e-7 seconds.
//! 1 second = 10,000,000 ticks.
//!
//! A 64-bit unsigned integer can represent over 58,494 years of simulation time
//! without overflow.

use serde::{Deserialize, Serialize};
use std::fmt;
use std::ops::{Add, AddAssign, Sub, SubAssign};

/// Number of simulation ticks per second (100-ns resolution: 10 MHz).
pub const TICKS_PER_SECOND: u64 = 10_000_000;
/// Number of ticks per millisecond.
pub const TICKS_PER_MILLIS: u64 = 10_000;
/// Number of ticks per microsecond.
pub const TICKS_PER_MICROS: u64 = 10;
/// Number of nanoseconds per tick (100 ns).
pub const NANOS_PER_TICK: u64 = 100;

#[derive(
    Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default, Serialize, Deserialize,
)]
pub struct SimTime(u64);

impl SimTime {
    pub const ZERO: SimTime = SimTime(0);
    pub const MAX: SimTime = SimTime(u64::MAX);

    #[inline]
    pub const fn from_ticks(ticks: u64) -> Self {
        SimTime(ticks)
    }

    #[inline]
    pub const fn ticks(&self) -> u64 {
        self.0
    }

    #[inline]
    pub fn from_secs_f64(secs: f64) -> Self {
        if secs <= 0.0 {
            SimTime::ZERO
        } else {
            let ticks = (secs * (TICKS_PER_SECOND as f64)).round() as u64;
            SimTime(ticks)
        }
    }

    #[inline]
    pub fn to_secs_f64(&self) -> f64 {
        self.0 as f64 / (TICKS_PER_SECOND as f64)
    }

    #[inline]
    pub const fn from_millis(ms: u64) -> Self {
        SimTime(ms * TICKS_PER_MILLIS)
    }

    #[inline]
    pub const fn to_millis(&self) -> u64 {
        self.0 / TICKS_PER_MILLIS
    }

    #[inline]
    pub const fn from_micros(us: u64) -> Self {
        SimTime(us * TICKS_PER_MICROS)
    }

    #[inline]
    pub const fn to_micros(&self) -> u64 {
        self.0 / TICKS_PER_MICROS
    }

    #[inline]
    pub const fn from_nanos(ns: u64) -> Self {
        SimTime(ns / NANOS_PER_TICK)
    }

    #[inline]
    pub const fn to_nanos(&self) -> u64 {
        self.0 * NANOS_PER_TICK
    }
}

impl Add for SimTime {
    type Output = SimTime;
    #[inline]
    fn add(self, rhs: Self) -> Self::Output {
        SimTime(self.0.checked_add(rhs.0).expect("SimTime overflow"))
    }
}

impl AddAssign for SimTime {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        self.0 = self.0.checked_add(rhs.0).expect("SimTime overflow");
    }
}

impl Sub for SimTime {
    type Output = SimTime;
    #[inline]
    fn sub(self, rhs: Self) -> Self::Output {
        SimTime(self.0.checked_sub(rhs.0).expect("SimTime underflow"))
    }
}

impl SubAssign for SimTime {
    #[inline]
    fn sub_assign(&mut self, rhs: Self) {
        self.0 = self.0.checked_sub(rhs.0).expect("SimTime underflow");
    }
}

impl fmt::Display for SimTime {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:.6}s ({} ticks)", self.to_secs_f64(), self.0)
    }
}

impl From<f64> for SimTime {
    fn from(secs: f64) -> Self {
        SimTime::from_secs_f64(secs)
    }
}

impl From<SimTime> for f64 {
    fn from(t: SimTime) -> Self {
        t.to_secs_f64()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sim_time_conversions() {
        let t = SimTime::from_secs_f64(1.5);
        assert_eq!(t.ticks(), 15_000_000);
        assert_eq!(t.to_millis(), 1500);
        assert_eq!(t.to_micros(), 1_500_000);
        assert_eq!(t.to_nanos(), 1_500_000_000);
        assert!((t.to_secs_f64() - 1.5).abs() < 1e-9);

        let t_us = SimTime::from_micros(13); // 13 us IEEE 802.11p slot time
        assert_eq!(t_us.ticks(), 130);
    }

    #[test]
    fn test_sim_time_arithmetic() {
        let t1 = SimTime::from_millis(100);
        let t2 = SimTime::from_millis(50);
        assert_eq!((t1 + t2).to_millis(), 150);
        assert_eq!((t1 - t2).to_millis(), 50);
    }
}
