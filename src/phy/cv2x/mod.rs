//! 3GPP Release 14 Cellular-V2X (C-V2X) PC5 Sidelink Mode 4 Implementation.
//!
//! Standards-compliant distributed vehicular communications:
//! - 3GPP TS 36.213: Physical Layer Procedures (Sensing-Based Semi-Persistent Scheduling)
//! - 3GPP TS 36.321: MAC Protocol Specification (Sidelink scheduling & CBR/CR)
//! - 3GPP TS 36.211 / 36.212: Physical Channels (PSCCH / PSSCH) and SCI Format 1

pub mod config;
pub mod phy_bler;
pub mod resource_grid;
pub mod sb_sps;
pub mod sci;
pub mod sensing;

pub use config::Cv2xConfig;
pub use phy_bler::{SidelinkPhyModel, SidelinkReceptionResult};
pub use resource_grid::{CandidateResource, SidelinkResourceGrid};
pub use sb_sps::{SensingBasedSpsScheduler, SpsGrant};
pub use sci::SidelinkControlInfo;
pub use sensing::{SciMeasurement, SidelinkSensingManager};
