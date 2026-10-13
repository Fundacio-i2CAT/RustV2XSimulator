//! Decentralized Congestion Control (DCC) configuration and profile types.

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DccMode {
    /// Mode 1: DCC disabled (raw kinematic triggers for baseline isolation).
    Disabled,
    /// Mode 2: Standardized ETSI TS 102 687 Reactive 7-state FSM.
    Reactive,
    /// Mode 2: Standardized ETSI TS 102 687 Adaptive LIMERIC linear rate controller.
    Limeric,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DccConfig {
    pub mode: DccMode,
    pub target_cbr: f64,
    pub alpha: f64,
    pub beta: f64,
    pub min_rate_hz: f64,
    pub max_rate_hz: f64,
}

impl Default for DccConfig {
    fn default() -> Self {
        Self {
            mode: DccMode::Reactive,
            target_cbr: 0.68,
            alpha: 0.016,
            beta: 0.0012,
            min_rate_hz: 2.0,
            max_rate_hz: 10.0,
        }
    }
}
