//! SUMO Interface and TraCI client adapter.
//!
//! Provides parsing for SUMO network XML (.net.xml) and route files (.rou.xml),
//! and TCP TraCI client for driving vehicles directly from SUMO or SUMO-GUI.

use std::collections::HashMap;
use std::fs::File;
use std::io::BufReader;
use std::path::{Path, PathBuf};
use std::process::{Child, Command};
use std::thread;
use std::time::Duration;

use log::{info, warn};
use quick_xml::events::Event as XmlEvent;
use quick_xml::reader::Reader as XmlReader;
use sumo_traci::TraciClient;

use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};

use crate::common::{GeodeticTransform, SimError, SimResult};
use crate::geometry::aabb::AABB;
use crate::geometry::segment::Point2D;
use crate::mobility::trajectory_source::TrajectorySource;
use crate::mobility::vehicle_state::VehicleKinematics;
use crate::scheduler::event::ActorId;

#[derive(Debug, Clone, Default)]
pub struct SumoNetworkInfo {
    pub boundary: AABB,
    pub geodetic: GeodeticTransform,
    pub edges: Vec<String>,
}

impl SumoNetworkInfo {
    /// Ingest boundary and projection parameters from SUMO `.net.xml`.
    pub fn parse_from_net_xml<P: AsRef<Path>>(path: P) -> SimResult<Self> {
        let file = File::open(path.as_ref())
            .map_err(|e| SimError::IoError(format!("Failed to open net XML: {}", e)))?;
        let mut reader = XmlReader::from_reader(BufReader::new(file));
        reader.config_mut().trim_text(true);

        let mut boundary = AABB::new(0.0, 0.0, 2000.0, 2000.0);
        let mut geodetic = GeodeticTransform::default();
        let mut edges = Vec::new();
        let mut buf = Vec::new();

        loop {
            match reader.read_event_into(&mut buf) {
                Ok(XmlEvent::Start(ref e)) | Ok(XmlEvent::Empty(ref e)) => {
                    match e.name().as_ref() {
                        b"location" => {
                            for attr in e.attributes().flatten() {
                                match attr.key.as_ref() {
                                    b"convBoundary" => {
                                        let s = String::from_utf8_lossy(&attr.value);
                                        let coords: Vec<f64> = s
                                            .split(',')
                                            .filter_map(|c| c.parse::<f64>().ok())
                                            .collect();
                                        if coords.len() == 4 {
                                            boundary = AABB::new(
                                                coords[0], coords[1], coords[2], coords[3],
                                            );
                                        }
                                    }
                                    b"netOffset" => {
                                        let s = String::from_utf8_lossy(&attr.value);
                                        let offsets: Vec<f64> = s
                                            .split(',')
                                            .filter_map(|c| c.parse::<f64>().ok())
                                            .collect();
                                        if offsets.len() == 2 {
                                            geodetic.net_offset_x = offsets[0];
                                            geodetic.net_offset_y = offsets[1];
                                        }
                                    }
                                    _ => {}
                                }
                            }
                        }
                        b"edge" => {
                            for attr in e.attributes().flatten() {
                                if attr.key.as_ref() == b"id" {
                                    edges.push(String::from_utf8_lossy(&attr.value).into_owned());
                                }
                            }
                        }
                        _ => {}
                    }
                }
                Ok(XmlEvent::Eof) => break,
                Err(e) => return Err(SimError::XmlParseError(format!("XML error: {}", e))),
                _ => {}
            }
            buf.clear();
        }

        Ok(Self {
            boundary,
            geodetic,
            edges,
        })
    }
}

/// Configuration for launching or connecting to SUMO via TraCI.
#[derive(Debug, Clone)]
pub struct SumoMobilityConfig {
    pub sumo_binary: String,
    pub sumocfg_path: PathBuf,
    pub traci_host: String,
    pub traci_port: u16,
    pub gui: bool,
    pub num_vehicles: usize,
    pub step_length_s: f64,
    pub seed: u64,
}

impl Default for SumoMobilityConfig {
    fn default() -> Self {
        Self {
            sumo_binary: "/usr/bin/sumo".to_string(),
            sumocfg_path: PathBuf::from("scenarios/diagonal/osm.sumocfg"),
            traci_host: "127.0.0.1".to_string(),
            traci_port: 8813,
            gui: false,
            num_vehicles: 5,
            step_length_s: 0.1,
            seed: 42,
        }
    }
}

/// TraCI-connected SUMO mobility source.
pub struct SumoMobilitySource {
    client: TraciClient,
    child_process: Option<Child>,
    vehicles: Vec<VehicleKinematics>,
    veh_id_to_actor: HashMap<String, ActorId>,
    next_actor_id: ActorId,
    pub config: SumoMobilityConfig,
}

impl SumoMobilitySource {
    /// Launch SUMO (or SUMO-GUI) and establish TraCI connection.
    pub fn new(config: SumoMobilityConfig) -> SimResult<Self> {
        if !config.sumocfg_path.exists() {
            return Err(SimError::MobilityError(format!(
                "SUMO config file not found: {}",
                config.sumocfg_path.display()
            )));
        }

        info!(
            "Starting SUMO ({}) on config: {}, GUI: {}",
            config.sumo_binary,
            config.sumocfg_path.display(),
            config.gui
        );

        let abs_sumocfg = config
            .sumocfg_path
            .canonicalize()
            .unwrap_or_else(|_| config.sumocfg_path.clone());

        let mut cmd = Command::new(&config.sumo_binary);
        if let Some(parent) = abs_sumocfg.parent() {
            cmd.current_dir(parent);
        }
        cmd.arg("-c")
            .arg(&abs_sumocfg)
            .arg("--remote-port")
            .arg(config.traci_port.to_string())
            .arg("--quit-on-end");

        if config.step_length_s > 0.0 {
            cmd.arg("--step-length")
                .arg(format!("{:.6}", config.step_length_s));
        }

        if config.gui {
            cmd.arg("--start");
        } else {
            cmd.arg("--no-step-log");
        }

        let child = cmd.spawn().map_err(|e| {
            SimError::IoError(format!(
                "Failed to spawn SUMO binary '{}': {}",
                config.sumo_binary, e
            ))
        })?;

        // Give SUMO a moment to initialize the TCP server with retry loop
        let mut client_res = None;
        for _ in 0..25 {
            match TraciClient::connect(&config.traci_host, config.traci_port) {
                Ok(c) => {
                    client_res = Some(c);
                    break;
                }
                Err(_) => {
                    thread::sleep(Duration::from_millis(200));
                }
            }
        }

        let mut client = client_res.ok_or_else(|| {
            SimError::MobilityError(format!(
                "TraCI connect to {}:{} timed out after 5s",
                config.traci_host, config.traci_port
            ))
        })?;

        client
            .set_order(1)
            .map_err(|e| SimError::MobilityError(format!("TraCI set_order failed: {:?}", e)))?;

        let mut veh_id_to_actor: HashMap<String, ActorId> = HashMap::new();
        let mut next_actor_id = 1;

        // If requested, spawn vehicles randomly across available routes
        let client_ptr: *mut TraciClient = &mut client;
        let route_ids = unsafe { &*client_ptr }
            .route
            .get_id_list(unsafe { &mut *client_ptr })
            .unwrap_or_default();

        if !route_ids.is_empty() && config.num_vehicles > 0 {
            info!(
                "Populating {} vehicles randomly across {} routes in SUMO (seed: {})...",
                config.num_vehicles,
                route_ids.len(),
                config.seed
            );
            let mut rng = StdRng::seed_from_u64(config.seed);
            for i in 0..config.num_vehicles {
                let veh_id = format!("veh_{}", i + 1);
                let route_idx = rng.gen_range(0..route_ids.len());
                let route_id = &route_ids[route_idx];
                let res = unsafe { &mut *client_ptr }.vehicle.add(
                    unsafe { &mut *client_ptr },
                    &veh_id,
                    route_id,
                    "DEFAULT_VEHTYPE",
                    "-1".to_string(), // depart = now
                    "best",
                    "0",
                    "max",
                    "current",
                    "max",
                    "current",
                    "",
                    "",
                    "",
                    0,
                    0,
                );
                if let Err(e) = res {
                    warn!("Could not add vehicle {}: {:?}", veh_id, e);
                }
            }
        }

        // Step once so vehicles are placed on roads
        let _ = client.simulation_step(0.0);

        // Fetch initial active vehicles
        let id_list = unsafe { &*client_ptr }
            .vehicle
            .get_id_list(unsafe { &mut *client_ptr })
            .unwrap_or_default();

        let mut vehicles = Vec::with_capacity(id_list.len());
        for v_str in &id_list {
            let actor_id = next_actor_id;
            next_actor_id += 1;
            veh_id_to_actor.insert(v_str.clone(), actor_id);

            let (pos_x, pos_y) = match unsafe { &*client_ptr }.vehicle.get_position(unsafe { &mut *client_ptr }, v_str) {
                Ok(p) => (p.x, p.y),
                Err(_) => (0.0, 0.0),
            };
            let spd = unsafe { &*client_ptr }
                .vehicle
                .get_speed(unsafe { &mut *client_ptr }, v_str)
                .unwrap_or(0.0);
            let angle = unsafe { &*client_ptr }
                .vehicle
                .get_angle(unsafe { &mut *client_ptr }, v_str)
                .unwrap_or(0.0);
            let accel = unsafe { &*client_ptr }
                .vehicle
                .get_acceleration(unsafe { &mut *client_ptr }, v_str)
                .unwrap_or(0.0);

            let mut vk = VehicleKinematics::new(actor_id, Point2D::new(pos_x, pos_y), spd, angle);
            vk.acceleration = accel;
            vehicles.push(vk);
        }

        info!("SUMO initialized with {} active vehicles", vehicles.len());

        Ok(Self {
            client,
            child_process: Some(child),
            vehicles,
            veh_id_to_actor,
            next_actor_id,
            config,
        })
    }
}

impl TrajectorySource for SumoMobilitySource {
    fn step(&mut self, _dt_s: f64) -> Vec<VehicleKinematics> {
        let client_ptr: *mut TraciClient = &mut self.client;
        if let Err(e) = self.client.simulation_step(0.0) {
            warn!("SUMO simulation_step error: {:?}", e);
            return self.vehicles.clone();
        }

        let id_list = unsafe { &*client_ptr }
            .vehicle
            .get_id_list(unsafe { &mut *client_ptr })
            .unwrap_or_default();

        let mut active_vehicles = Vec::with_capacity(id_list.len());

        for v_str in &id_list {
            let actor_id = if let Some(&id) = self.veh_id_to_actor.get(v_str) {
                id
            } else {
                let new_id = self.next_actor_id;
                self.next_actor_id += 1;
                self.veh_id_to_actor.insert(v_str.clone(), new_id);
                new_id
            };

            let (pos_x, pos_y) = match unsafe { &*client_ptr }.vehicle.get_position(unsafe { &mut *client_ptr }, v_str) {
                Ok(p) => (p.x, p.y),
                Err(_) => (0.0, 0.0),
            };
            let spd = unsafe { &*client_ptr }
                .vehicle
                .get_speed(unsafe { &mut *client_ptr }, v_str)
                .unwrap_or(0.0);
            let angle = unsafe { &*client_ptr }
                .vehicle
                .get_angle(unsafe { &mut *client_ptr }, v_str)
                .unwrap_or(0.0);
            let accel = unsafe { &*client_ptr }
                .vehicle
                .get_acceleration(unsafe { &mut *client_ptr }, v_str)
                .unwrap_or(0.0);

            let mut vk = VehicleKinematics::new(actor_id, Point2D::new(pos_x, pos_y), spd, angle);
            vk.acceleration = accel;
            active_vehicles.push(vk);
        }

        self.vehicles = active_vehicles;
        self.vehicles.clone()
    }

    fn get_vehicles(&self) -> &[VehicleKinematics] {
        &self.vehicles
    }

    fn get_vehicle_mut(&mut self, id: ActorId) -> Option<&mut VehicleKinematics> {
        self.vehicles.iter_mut().find(|v| v.id == id)
    }
}

impl Drop for SumoMobilitySource {
    fn drop(&mut self) {
        let _ = self.client.close();
        if let Some(mut child) = self.child_process.take() {
            let _ = child.kill();
            let _ = child.wait();
        }
    }
}

/// Backward compatibility stub
pub struct SumoInterface {
    pub network: SumoNetworkInfo,
    pub vehicles: Vec<VehicleKinematics>,
}

impl SumoInterface {
    pub fn new(network: SumoNetworkInfo) -> Self {
        Self {
            network,
            vehicles: Vec::new(),
        }
    }

    pub fn set_vehicles(&mut self, vehicles: Vec<VehicleKinematics>) {
        self.vehicles = vehicles;
    }
}

impl TrajectorySource for SumoInterface {
    fn step(&mut self, dt_s: f64) -> Vec<VehicleKinematics> {
        for v in &mut self.vehicles {
            let rad = v.heading_deg.to_radians();
            v.position.x += v.speed * dt_s * rad.cos();
            v.position.y += v.speed * dt_s * rad.sin();
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
