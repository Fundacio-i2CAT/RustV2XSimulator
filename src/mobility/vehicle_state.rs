//! Vehicle kinematic state definitions.

use crate::common::Wgs84Coordinate;
use crate::geometry::segment::Point2D;
use crate::scheduler::event::ActorId;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VehicleKinematics {
    pub id: ActorId,
    pub position: Point2D,
    pub altitude: f64,
    pub speed: f64,
    pub heading_deg: f64,
    pub acceleration: f64,
    pub lane_id: u32,
    pub leader_id: Option<ActorId>,
    pub leader_distance: Option<f64>,
    pub leader_speed: Option<f64>,
    pub length_m: f64,
    pub min_gap_m: f64,
    pub max_decel: f64,
    pub reaction_time_s: f64,
    pub is_braking_hazard: bool,
}

impl VehicleKinematics {
    pub fn new(id: ActorId, position: Point2D, speed: f64, heading_deg: f64) -> Self {
        Self {
            id,
            position,
            altitude: 0.0,
            speed,
            heading_deg,
            acceleration: 0.0,
            lane_id: 0,
            leader_id: None,
            leader_distance: None,
            leader_speed: None,
            length_m: 5.0,
            min_gap_m: 2.5,
            max_decel: 6.0,
            reaction_time_s: 0.1,
            is_braking_hazard: false,
        }
    }

    pub fn to_wgs84(&self, orig_lat: f64, orig_lon: f64) -> Wgs84Coordinate {
        let lat = orig_lat + (self.position.y / 111_139.0);
        let lon = orig_lon + (self.position.x / (111_139.0 * orig_lat.to_radians().cos()));
        Wgs84Coordinate::new(lat, lon, self.altitude)
    }
}
