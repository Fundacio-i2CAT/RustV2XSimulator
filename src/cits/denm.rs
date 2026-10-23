//! ETSI Decentralized Environmental Notification (DEN) Service via `rustflexstack`.
//!
//! Re-exports and integrates standard ETSI EN 302 637-3 DENM from `rustflexstack`.

pub use rustflexstack::facilities::decentralized_environmental_notification_service::{
    denm_bindings::etsi_its_cdd::DangerousSituationSubCauseCode, denm_coder::*, DENRequest,
    DecentralizedEnvironmentalNotificationService, VehicleData,
};
