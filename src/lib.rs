//! # rustv2xsimulator
//!
//! A High-Performance Discrete-Event Simulation Framework for ETSI C-ITS in Rust.
//!
//! ## Modular architecture:
//! - [`common`] — High-resolution `SimTime` (100-ns ticks), geodetic coordinates, profiler.
//! - [`scheduler`] — Discrete-event calendar queue and strict causality scheduler.
//! - [`geometry`] — 2D computational geometry, OSM building polygons, AABB, and KD-tree index.
//! - [`phy`] — Dual-slope log-distance path loss, knife-edge diffraction, chunked SINR, and parallel radio medium.
//! - [`mac`] — IEEE 802.11p EDCA 4 Access Categories and Channel Busy Ratio (CBR) monitor.
//! - [`dcc`] — ETSI TS 102 687 Decentralized Congestion Control (Reactive FSM & Adaptive LIMERIC).
//! - [`cits`] — Standard ETSI C-ITS stack: CAM, DENM, BTP, GeoNetworking, and LDM.
//! - [`mobility`] — Microscopic vehicle mobility and Krauß-safe closed-loop ballistic actuation.
//! - [`pcap`] — Standard libpcap file export with Ethernet II / GeoNetworking framing.
//! - [`engine`] — Unified discrete-event simulation engine orchestrator.
//!
//! ## Quick Start with Prelude:
//! ```rust,no_run
//! use rustv2xsimulator::prelude::*;
//!
//! let config = EngineConfig::default();
//! let buildings = BuildingMap::default();
//! let mut engine = SimulationEngine::new(config, buildings).expect("Failed to initialize engine");
//! engine.run().expect("Simulation failed");
//! ```

pub mod cits;
pub mod common;
pub mod dcc;
pub mod engine;
pub mod geometry;
pub mod mac;
pub mod mobility;
pub mod pcap;
pub mod phy;
pub mod scheduler;
pub mod security;

// Re-export core types at crate root for ease of use
pub use common::{
    DecoupledWallClockProfiler, GeodeticTransform, ProfileCategory, ProfilerSummary, SimError,
    SimResult, SimTime, Wgs84Coordinate,
};
pub use dcc::{DccConfig, DccMode};
pub use engine::{EngineConfig, SimulationEngine, VehicleAgent};
pub use geometry::{BuildingMap, Point2D, SpatialIndex};
pub use mobility::{ActuationMode, SyntheticMobilityType, TrajectorySource, VehicleKinematics};
pub use phy::{PhyModelType, ReceptionResult, WirelessTechnology, WirelessTechnologyType};
pub use scheduler::{CalendarQueue, DiscreteEventScheduler, Event, EventContext, EventPayload};

/// Convenience prelude module re-exporting the most commonly used types and traits.
pub mod prelude {
    pub use crate::common::{
        DecoupledWallClockProfiler, GeodeticTransform, ProfileCategory, ProfilerSummary, SimError,
        SimResult, SimTime, Wgs84Coordinate,
    };
    pub use crate::dcc::{DccConfig, DccMode};
    pub use crate::engine::{EngineConfig, SimulationEngine, VehicleAgent};
    pub use crate::geometry::{BuildingMap, Point2D, SpatialIndex};
    pub use crate::mobility::{
        ActuationMode, SyntheticMobilityType, TrajectorySource, VehicleKinematics,
    };
    pub use crate::phy::{
        PhyModelType, ReceptionResult, WirelessTechnology, WirelessTechnologyType,
    };
    pub use crate::scheduler::{
        CalendarQueue, DiscreteEventScheduler, Event, EventContext, EventPayload,
    };
}
