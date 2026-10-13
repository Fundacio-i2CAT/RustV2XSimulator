//! ETSI TS 102 687 Reactive DCC FSM — backed by `rustflexstack`.
//!
//! This module wraps [`rustflexstack::management::DccReactive`] to provide the
//! exact ETSI TS 102 687 V1.2.1 (2018-04) clause 5.3 / Annex A reactive FSM,
//! replacing the previous hand-rolled implementation.
//!
//! The public API is kept backward-compatible: `update_cbr(cbr) -> SimTime`
//! works as before, so `engine.rs` needs no changes.

pub use rustflexstack::management::{DccError, DccReactive, DccReactiveOutput, DccState};

use crate::common::SimTime;

/// Type alias kept for call-site backward compatibility.
pub type ReactiveState = DccState;

/// Adapter that drives [`DccReactive`] from `rustflexstack` and exposes a
/// simulator-friendly `update_cbr(cbr) -> SimTime` interface.
///
/// The underlying `DccReactive::update` returns a [`DccReactiveOutput`] that
/// includes the standard `t_off_ms` field from Annex A Table A.1 or A.2,
/// which this adapter converts to a [`SimTime`].
#[derive(Debug, Clone)]
pub struct ReactiveDccFsm {
    /// The canonical ETSI reactive DCC state machine from `rustflexstack`.
    pub inner: DccReactive,
    /// Current inter-packet gap in simulation time.
    pub current_t_off: SimTime,
}

impl Default for ReactiveDccFsm {
    fn default() -> Self {
        Self::new()
    }
}

impl ReactiveDccFsm {
    /// Create a new reactive DCC FSM using **Table A.1** (T_on ≤ 1 ms).
    ///
    /// This is the most common profile for IEEE 802.11p / ITS-G5 at 1 ms OFDM
    /// burst duration and matches the original simulator behaviour.
    pub fn new() -> Self {
        Self {
            inner: DccReactive::new(1000), // T_on_max = 1000 µs → Table A.1
            current_t_off: SimTime::from_millis(100), // Relaxed = 100 ms (T_off, Table A.1)
        }
    }

    /// Expose the current DCC state for metrics / logging.
    #[inline]
    pub fn current_state(&self) -> DccState {
        self.inner.state
    }

    /// Evaluate the reactive FSM with the current Channel Busy Ratio.
    ///
    /// Calls `DccReactive::update(cbr)`, which enforces the adjacency constraint
    /// (at most one state change per evaluation), then converts `t_off_ms` from
    /// the returned [`DccReactiveOutput`] to a [`SimTime`].
    ///
    /// On validation error (CBR outside [0, 1]) the method clamps the CBR to the
    /// nearest valid boundary to remain non-panicking in the simulation loop.
    pub fn update_cbr(&mut self, cbr: f64) -> SimTime {
        let clamped = cbr.clamp(0.0, 1.0);
        match self.inner.update(clamped) {
            Ok(out) => {
                self.current_t_off = SimTime::from_millis(out.t_off_ms.round() as u64);
            }
            Err(_) => {
                // Shouldn't happen after clamping, but be defensive
            }
        }
        self.current_t_off
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_reactive_fsm_starts_relaxed() {
        let fsm = ReactiveDccFsm::new();
        assert_eq!(fsm.current_state(), DccState::Relaxed);
    }

    #[test]
    fn test_reactive_fsm_high_cbr_escalates() {
        let mut fsm = ReactiveDccFsm::new();
        // Drive toward Restrictive step-by-step (adjacency constraint)
        for _ in 0..5 {
            fsm.update_cbr(0.85);
        }
        assert_eq!(fsm.current_state(), DccState::Restrictive);
    }

    #[test]
    fn test_reactive_fsm_low_cbr_relaxes() {
        let mut fsm = ReactiveDccFsm::new();
        // Drive to Restrictive
        for _ in 0..5 {
            fsm.update_cbr(0.85);
        }
        // Then recover
        for _ in 0..5 {
            fsm.update_cbr(0.05);
        }
        assert_eq!(fsm.current_state(), DccState::Relaxed);
    }

    #[test]
    fn test_reactive_fsm_t_off_table_a1_relaxed() {
        let mut fsm = ReactiveDccFsm::new();
        let t = fsm.update_cbr(0.10);
        // Table A.1 Relaxed → t_off = 100 ms
        assert_eq!(t, SimTime::from_millis(100));
    }

    #[test]
    fn test_reactive_fsm_t_off_table_a1_active1() {
        let mut fsm = ReactiveDccFsm::new();
        let t = fsm.update_cbr(0.35);
        // Table A.1 Active1 → t_off = 200 ms
        assert_eq!(t, SimTime::from_millis(200));
    }
}
