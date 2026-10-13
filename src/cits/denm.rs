//! ETSI Decentralized Environmental Notification (DEN) Service via `rustflexstack`.
//!
//! Re-exports and integrates standard ETSI EN 302 637-3 DENM from `rustflexstack`.

pub use rustflexstack::facilities::decentralized_environmental_notification_service::{
    denm_bindings::etsi_its_cdd::DangerousSituationSubCauseCode, denm_coder::*, DENRequest,
    DecentralizedEnvironmentalNotificationService, VehicleData,
};

use crate::common::Wgs84Coordinate;

pub const CAUSE_CODE_EMERGENCY_BRAKE: u8 = 99;
pub const SUBCAUSE_CODE_EEBL: u8 = 1;

/// Helper to create a standard EEBL emergency braking hazard request.
pub fn create_eebl_request(
    coord: &Wgs84Coordinate,
    speed_m_s: f64,
    heading_deg: f64,
) -> DENRequest {
    DENRequest {
        event_latitude: coord.latitude,
        event_longitude: coord.longitude,
        event_altitude_m: coord.altitude,
        cause_code: CauseCodeChoice::dangerousSituation99(DangerousSituationSubCauseCode(1)),
        information_quality: 7,
        event_speed_raw: ((speed_m_s * 100.0).round() as u16).min(16382),
        event_heading_raw: ((heading_deg.rem_euclid(360.0) * 10.0).round() as u16).min(3600),
        denm_interval_ms: 100,
        time_period_ms: 5000,
        relevance_radius_m: 1000,
    }
}
