//! Decentralized Congestion Control (DCC) — ETSI TS 102 687.
//!
//! All DCC algorithms are now backed by the official
//! [`rustflexstack::management`] crate, which provides a rigorously
//! standards-conformant implementation of:
//! - **Adaptive DCC / LIMERIC** – clause 5.4 & Annex B ([`DccAdaptive`], [`GateKeeper`])
//! - **Reactive DCC FSM**      – clause 5.3 & Annex A ([`DccReactive`])
//!
//! The sub-modules in this directory provide thin simulator adapters that
//! preserve the existing call-site API (`update_cbr(cbr) -> SimTime`).

pub mod dcc_profile;
pub mod limeric;
pub mod reactive_fsm;

// ── Simulator-level config types ──────────────────────────────────────────────
pub use dcc_profile::{DccConfig, DccMode};

// ── Adapter types (wrap rustflexstack implementations) ───────────────────────
pub use limeric::LimericDcc;
pub use reactive_fsm::ReactiveDccFsm;

// ── Re-export rustflexstack management layer for direct use ───────────────────
pub use rustflexstack::management::{
    DccAdaptive, DccAdaptiveParameters, DccError, DccReactive, DccReactiveOutput, DccState,
    GateKeeper,
};

// Backward-compat alias: the old 7-state ReactiveState is now DccState (5-state).
pub use DccState as ReactiveState;
