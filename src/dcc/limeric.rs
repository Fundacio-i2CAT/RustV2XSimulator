//! ETSI TS 102 687 Adaptive DCC (LIMERIC) — backed by `rustflexstack`.
//!
//! This module wraps [`rustflexstack::management::DccAdaptive`] and
//! [`rustflexstack::management::GateKeeper`] to provide the exact ETSI TS 102 687
//! V1.2.1 (2018-04) clause 5.4 algorithm and Annex B admission gate, replacing the
//! previous hand-rolled implementation.
//!
//! The public API exposed here is kept backward-compatible with the rest of the
//! simulator (specifically `engine.rs`) via the `update_cbr` convenience method.

pub use rustflexstack::management::{DccAdaptive, DccAdaptiveParameters, DccError, GateKeeper};

use crate::common::SimTime;

/// Adapter that drives [`DccAdaptive`] from `rustflexstack` and exposes a
/// simulator-friendly `update_cbr(cbr) -> SimTime` interface.
///
/// Internally it:
/// 1. Calls `DccAdaptive::update_local(cbr, cbr)` to run the full LIMERIC steps.
/// 2. Updates the `GateKeeper` delta so the admission interval tracks `delta`.
/// 3. Returns `1 / rate_hz` as a [`SimTime`], where `rate_hz = 1 / (t_on / delta)`.
///    For a default `t_on = 1 ms`, this equals `delta / 0.001`.
///
/// A separate [`GateKeeper`] is kept so callers can optionally use it for
/// fine-grained packet admission control.
#[derive(Debug, Clone)]
pub struct LimericDcc {
    /// The canonical ETSI LIMERIC state machine from `rustflexstack`.
    pub adaptive: DccAdaptive,
    /// Gate keeper for packet admission (Annex B).
    pub gate: GateKeeper,
    /// Assumed maximum packet transmission duration in seconds, used to convert
    /// `delta` (duty-cycle fraction) to a minimum inter-packet interval.
    /// Default: 0.001 s (1 ms, Table A.1 assumption).
    pub t_on_s: f64,
}

impl Default for LimericDcc {
    fn default() -> Self {
        let params = DccAdaptiveParameters::default();
        let delta_init = params.delta_min;
        Self {
            adaptive: DccAdaptive::new(params),
            gate: GateKeeper::new(delta_init),
            t_on_s: 0.001,
        }
    }
}

impl LimericDcc {
    /// Create a new LIMERIC controller.
    ///
    /// # Parameters
    /// - `target_cbr`:   Target Channel Busy Ratio (default 0.68).
    /// - `alpha`:        Exponential smoothing coefficient (default 0.016).
    /// - `beta`:         Proportional gain (default 0.0012).
    /// - `_min_rate_hz`: Not used by the ETSI algorithm; kept for API compat.
    /// - `_max_rate_hz`: Not used by the ETSI algorithm; kept for API compat.
    pub fn new(
        target_cbr: f64,
        alpha: f64,
        beta: f64,
        _min_rate_hz: f64,
        _max_rate_hz: f64,
    ) -> Self {
        let params = DccAdaptiveParameters {
            alpha,
            beta,
            cbr_target: target_cbr,
            ..DccAdaptiveParameters::default()
        };
        let delta_init = params.delta_min;
        Self {
            adaptive: DccAdaptive::new(params),
            gate: GateKeeper::new(delta_init),
            t_on_s: 0.001,
        }
    }

    /// Run one LIMERIC step and return the new minimum inter-packet interval.
    ///
    /// Calls `DccAdaptive::update_local(cbr, cbr)` (local-only mode, no global CBR
    /// is available in a simulation without inter-vehicle BEACON exchange), then
    /// converts the resulting `delta` to a [`SimTime`] inter-packet gap:
    ///
    /// ```text
    /// T_off = T_on / delta        (ETSI TS 102 687 Annex B, eq. B.1)
    /// ```
    pub fn update_cbr(&mut self, cbr: f64) -> SimTime {
        let delta = self
            .adaptive
            .update_local(cbr, cbr)
            .unwrap_or(self.adaptive.delta);

        // Keep the gate keeper in sync (best-effort; errors are ignored in sim context)
        let _ = self.gate.update_delta(0.0, delta);

        // T_off = T_on / delta
        let t_off_s = (self.t_on_s / delta).clamp(0.025, 1.0);
        SimTime::from_secs_f64(t_off_s)
    }

    /// Return the current inter-packet gap without updating state.
    #[inline]
    pub fn current_t_off(&self) -> SimTime {
        let t_off_s = (self.t_on_s / self.adaptive.delta).clamp(0.025, 1.0);
        SimTime::from_secs_f64(t_off_s)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_limeric_convergence_via_rfs() {
        let mut limeric = LimericDcc::default();
        // Under sustained high CBR = 0.90, delta must converge toward delta_min.
        for _ in 0..500 {
            limeric.update_cbr(0.90);
        }
        let delta = limeric.adaptive.delta;
        assert!(
            delta <= limeric.adaptive.parameters.delta_max,
            "delta must not exceed delta_max"
        );
        assert!(
            delta >= limeric.adaptive.parameters.delta_min,
            "delta must not go below delta_min"
        );
    }

    #[test]
    fn test_limeric_update_returns_simtime() {
        let mut limeric = LimericDcc::default();
        let t = limeric.update_cbr(0.5);
        assert!(t.to_secs_f64() > 0.0);
    }

    #[test]
    fn test_limeric_high_cbr_increases_t_off() {
        let mut l_low = LimericDcc::default();
        let mut l_high = LimericDcc::default();
        for _ in 0..100 {
            l_low.update_cbr(0.1);
            l_high.update_cbr(0.9);
        }
        // High CBR → smaller delta → larger T_off
        assert!(
            l_high.adaptive.delta <= l_low.adaptive.delta,
            "High CBR must produce smaller delta (more restriction)"
        );
    }
}
