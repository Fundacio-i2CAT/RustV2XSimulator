//! Native Synthetic Mobility Generators and Zero-IPC Trajectory Replay.
//!
//! Provides deterministic multi-lane highway, urban Manhattan grid, and
//! car-following platoon generation to execute fully self-contained simulations
//! without external SUMO runtime dependencies.

use rand::rngs::StdRng;
use rand::Rng;
use rand::SeedableRng;
use serde::{Deserialize, Serialize};

use crate::geometry::segment::Point2D;
use crate::mobility::vehicle_state::VehicleKinematics;
use crate::scheduler::event::ActorId;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SyntheticMobilityType {
    Sumo,
    Highway,
    Urban,
    Platoon,
    Replay,
}

pub trait TrajectorySource: Send + Sync {
    fn step(&mut self, dt_s: f64) -> Vec<VehicleKinematics>;
    fn get_vehicles(&self) -> &[VehicleKinematics];
    fn get_vehicle_mut(&mut self, id: ActorId) -> Option<&mut VehicleKinematics>;
}

/// Deterministic multi-lane highway mobility generator.
#[derive(Debug, Clone)]
pub struct HighwayMobilityGenerator {
    pub vehicles: Vec<VehicleKinematics>,
    pub highway_length_m: f64,
    pub num_lanes: u32,
    pub lane_width_m: f64,
}

impl HighwayMobilityGenerator {
    pub fn new(num_vehicles: usize, length_m: f64, num_lanes: u32, seed: u64) -> Self {
        let mut rng = StdRng::seed_from_u64(seed);
        let lane_width = 3.75;
        let mut vehicles = Vec::with_capacity(num_vehicles);

        let spacing = length_m / (num_vehicles.max(1) as f64);
        for i in 0..num_vehicles {
            let id = (i + 1) as ActorId;
            let lane = (i as u32) % num_lanes;
            let x = (i as f64) * spacing + rng.gen_range(-2.0..2.0);
            let y = (lane as f64) * lane_width;
            let speed = rng.gen_range(22.0..30.0); // 80 - 110 km/h

            let mut vk = VehicleKinematics::new(id, Point2D::new(x, y), speed, 0.0);
            vk.lane_id = lane;
            vehicles.push(vk);
        }

        Self {
            vehicles,
            highway_length_m: length_m,
            num_lanes,
            lane_width_m: lane_width,
        }
    }
}

impl TrajectorySource for HighwayMobilityGenerator {
    fn step(&mut self, dt_s: f64) -> Vec<VehicleKinematics> {
        for vk in &mut self.vehicles {
            vk.position.x += vk.speed * dt_s;
            if vk.position.x > self.highway_length_m {
                vk.position.x = 0.0;
            }
        }
        self.vehicles.clone()
    }

    fn get_vehicles(&self) -> &[VehicleKinematics] {
        &self.vehicles
    }

    fn get_vehicle_mut(&mut self, id: ActorId) -> Option<&mut VehicleKinematics> {
        self.vehicles.iter_mut().find(|v| v.id == id)
    }
}

/// Linear platoon mobility generator.
#[derive(Debug, Clone)]
pub struct PlatoonMobilityGenerator {
    pub vehicles: Vec<VehicleKinematics>,
    pub initial_speed: f64,
}

impl PlatoonMobilityGenerator {
    pub fn new(num_vehicles: usize, initial_speed: f64, initial_gap: f64) -> Self {
        let mut vehicles = Vec::with_capacity(num_vehicles);
        let vehicle_len = 5.0;

        for i in 0..num_vehicles {
            let id = (i + 1) as ActorId;
            let x = -(i as f64) * (vehicle_len + initial_gap);
            let mut vk = VehicleKinematics::new(id, Point2D::new(x, 0.0), initial_speed, 0.0);
            vk.length_m = vehicle_len;

            if i > 0 {
                vk.leader_id = Some(i as ActorId);
                vk.leader_distance = Some(initial_gap);
                vk.leader_speed = Some(initial_speed);
            }
            vehicles.push(vk);
        }

        Self {
            vehicles,
            initial_speed,
        }
    }
}

impl TrajectorySource for PlatoonMobilityGenerator {
    fn step(&mut self, dt_s: f64) -> Vec<VehicleKinematics> {
        for vk in &mut self.vehicles {
            vk.position.x += vk.speed * dt_s;
        }
        self.vehicles.clone()
    }

    fn get_vehicles(&self) -> &[VehicleKinematics] {
        &self.vehicles
    }

    fn get_vehicle_mut(&mut self, id: ActorId) -> Option<&mut VehicleKinematics> {
        self.vehicles.iter_mut().find(|v| v.id == id)
    }
}

/// Manhattan urban grid synthetic mobility generator.
#[derive(Debug, Clone)]
pub struct UrbanGridMobilityGenerator {
    pub vehicles: Vec<VehicleKinematics>,
    pub grid_size_m: f64,
    pub block_size_m: f64,
}

impl UrbanGridMobilityGenerator {
    pub fn new(num_vehicles: usize, grid_size_m: f64, block_size_m: f64, seed: u64) -> Self {
        let mut rng = StdRng::seed_from_u64(seed);
        let mut vehicles = Vec::with_capacity(num_vehicles);

        for i in 0..num_vehicles {
            let id = (i + 1) as ActorId;
            let is_horizontal: bool = rng.gen();
            let block_idx = rng.gen_range(0..=(grid_size_m / block_size_m) as u32);
            let pos_coord = (block_idx as f64) * block_size_m;
            let along_coord = rng.gen_range(0.0..grid_size_m);

            let (pos, heading) = if is_horizontal {
                let h = if rng.gen() { 0.0 } else { 180.0 };
                (Point2D::new(along_coord, pos_coord), h)
            } else {
                let h = if rng.gen() { 90.0 } else { 270.0 };
                (Point2D::new(pos_coord, along_coord), h)
            };

            let speed = rng.gen_range(8.0..14.0); // 30 - 50 km/h urban
            vehicles.push(VehicleKinematics::new(id, pos, speed, heading));
        }

        Self {
            vehicles,
            grid_size_m,
            block_size_m,
        }
    }
}

impl TrajectorySource for UrbanGridMobilityGenerator {
    fn step(&mut self, dt_s: f64) -> Vec<VehicleKinematics> {
        for v in &mut self.vehicles {
            let rad = v.heading_deg.to_radians();
            let dx = v.speed * dt_s * rad.cos();
            let dy = v.speed * dt_s * rad.sin();

            v.position.x = (v.position.x + dx).rem_euclid(self.grid_size_m);
            v.position.y = (v.position.y + dy).rem_euclid(self.grid_size_m);
        }
        self.vehicles.clone()
    }

    fn get_vehicles(&self) -> &[VehicleKinematics] {
        &self.vehicles
    }

    fn get_vehicle_mut(&mut self, id: ActorId) -> Option<&mut VehicleKinematics> {
        self.vehicles.iter_mut().find(|v| v.id == id)
    }
}
