//! Central Discrete-Event Vehicular Simulation Engine.
//!
//! Orchestrates microscopic mobility, ETSI C-ITS protocols via `rustflexstack`,
//! EDCA MAC contention, Rayon-parallel physical medium evaluation,
//! and telemetry.

use std::collections::HashMap;
use std::path::{Path, PathBuf};

use log::info;
use rand::rngs::StdRng;
use rand::SeedableRng;
use serde::{Deserialize, Serialize};

use crate::cits::btp::{BTPRouter, BTPRouterHandle};
use crate::cits::cam::{
    summarize_cam, Cam, CamTriggerEvaluator, CooperativeAwarenessBasicService,
    CAMTransmissionManagement, GpsFix, LocationService, VehicleData,
};
use rustflexstack::geonet::basic_header::{BasicHeader, BasicNH};
use rustflexstack::security::certificate::decode_ieee1609_dot2_data;
use rustflexstack::security::security_asn::ieee1609_dot2::{Ieee1609Dot2Content, SignerIdentifier};
use crate::cits::denm::{
    DecentralizedEnvironmentalNotificationService, Denm, VehicleData as DenmVehData,
};
use crate::cits::geonet::{
    create_long_position_vector, GNAddress, GNRouter, Mib, RouterHandle, M, MID, ST,
};
use crate::cits::ldm::{LdmFacility, LdmHandle};
use crate::cits::link_layer::SimulatorLinkLayer;
use crate::common::{
    DecoupledWallClockProfiler, GeodeticTransform, ProfileCategory, SimResult, SimTime,
};
use crate::dcc::dcc_profile::{DccConfig, DccMode};
use crate::dcc::limeric::LimericDcc;
use crate::dcc::reactive_fsm::ReactiveDccFsm;
use crate::geometry::kd_tree::{NodeEntry, SpatialIndex};
use crate::geometry::polygon::BuildingMap;
use crate::mac::channel_monitor::ChannelMonitor;
use crate::mac::edca::{AccessCategory, EdcaChannelAccess};
use crate::mobility::sumo_interface::{SumoMobilityConfig, SumoMobilitySource};
use crate::mobility::trajectory_source::{
    HighwayMobilityGenerator, PlatoonMobilityGenerator, SyntheticMobilityType, TrajectorySource,
    UrbanGridMobilityGenerator,
};
use crate::mobility::vehicle_state::VehicleKinematics;
use crate::pcap::writer::PcapWriter;
use crate::phy::air_packet::AirPacket;
use crate::phy::consts_80211p::{compute_duration_80211p, Mcs80211p};
use crate::phy::cv2x::config::Cv2xConfig;
use crate::phy::nrv2x::config::NrV2xConfig;
use crate::phy::path_loss::LegacyPolynomialPathLoss;
use crate::phy::radio_medium::{PhyModelType, RadioMedium};
use crate::phy::technology::{
    Cv2xMode4Model, Dsrc80211pModel, NrV2xMode2Model, ParametricEmpiricalModel, Wifi80211bdModel,
    WirelessTechnology, WirelessTechnologyType,
};
use crate::phy::wifi80211bd::config::Wifi80211bdConfig;
use crate::phy::wifi80211bd::consts_80211bd::{compute_duration_80211bd, Mcs80211bd, MidamblePeriod};
use crate::scheduler::des::DiscreteEventScheduler;
use crate::scheduler::event::{ActorId, Event, EventPayload};
use crate::security::{GnSecurity, SimPki};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EngineConfig {
    pub scenario_name: String,
    pub scenario_dir: String,
    pub sumocfg_path: String,
    pub sumo_binary: String,
    pub traci_host: String,
    pub traci_port: u16,
    pub gui: bool,
    pub duration_s: f64,
    pub mobility_step_s: f64,
    pub num_nodes: usize,
    pub seed: u64,
    pub workload: String, // "cam", "cps"
    pub mobility_type: SyntheticMobilityType,
    pub technology: WirelessTechnologyType,
    pub phy_model: PhyModelType,
    pub parametric_phy: LegacyPolynomialPathLoss,
    pub dcc_config: DccConfig,
    pub output_dir: String,
    pub enable_pcap: bool,
    pub security_enabled: bool,
    pub verify_packets: bool,
    #[serde(default)]
    pub mcs: Option<u8>,
}

impl Default for EngineConfig {
    fn default() -> Self {
        Self {
            scenario_name: "diagonal".to_string(),
            scenario_dir: "scenarios/diagonal".to_string(),
            sumocfg_path: "scenarios/diagonal/osm.sumocfg".to_string(),
            sumo_binary: "/usr/bin/sumo".to_string(),
            traci_host: "127.0.0.1".to_string(),
            traci_port: 8813,
            gui: false,
            duration_s: 30.0,
            mobility_step_s: 0.1, // 100 ms step
            num_nodes: 5,
            seed: 42,
            workload: "cam".to_string(),
            mobility_type: SyntheticMobilityType::Sumo,
            technology: WirelessTechnologyType::Dsrc80211p,
            phy_model: PhyModelType::DualSlope,
            parametric_phy: LegacyPolynomialPathLoss::default(),
            dcc_config: DccConfig::default(),
            output_dir: "results/output/".to_string(),
            enable_pcap: false,
            security_enabled: false,
            verify_packets: false,
            mcs: None,
        }
    }
}

pub struct VehicleAgent {
    pub id: ActorId,
    pub mac_addr: [u8; 6],
    pub kinematics: VehicleKinematics,
    pub gn_handle: RouterHandle,
    pub btp_handle: BTPRouterHandle,
    pub location_service: LocationService,
    pub cam_trigger: CamTriggerEvaluator,
    pub cam_transmission: CAMTransmissionManagement,
    pub cam_rx: std::sync::mpsc::Receiver<Cam>,
    pub den_service: DecentralizedEnvironmentalNotificationService,
    pub denm_rx: std::sync::mpsc::Receiver<Denm>,
    pub ldm: LdmHandle,
    pub link_layer: SimulatorLinkLayer,
    pub mac: EdcaChannelAccess,
    pub channel_monitor: ChannelMonitor,
    pub tech: Box<dyn WirelessTechnology>,
    pub dcc_reactive: Option<ReactiveDccFsm>,
    pub dcc_limeric: Option<LimericDcc>,
    pub last_cbr: f64,
}

pub struct SimulationEngine {
    pub config: EngineConfig,
    pub scheduler: DiscreteEventScheduler,
    pub profiler: DecoupledWallClockProfiler,
    pub trajectory: Box<dyn TrajectorySource>,
    pub spatial_index: SpatialIndex,
    pub buildings: BuildingMap,
    pub radio_medium: RadioMedium,
    pub agents: HashMap<ActorId, VehicleAgent>,
    pub pcap_writer: Option<PcapWriter>,
    pub geodetic: GeodeticTransform,
    pub rng: StdRng,
    pub next_packet_id: u64,
    pub pki: Option<SimPki>,
}

impl SimulationEngine {
    pub fn new(config: EngineConfig, buildings: BuildingMap) -> SimResult<Self> {
        let max_duration = SimTime::from_secs_f64(config.duration_s);
        let scheduler = DiscreteEventScheduler::with_duration(max_duration);
        let profiler = DecoupledWallClockProfiler::new();

        let rng = StdRng::seed_from_u64(config.seed);

        // Initialize trajectory source
        let trajectory: Box<dyn TrajectorySource> = match config.mobility_type {
            SyntheticMobilityType::Sumo => {
                let sumocfg = if !config.sumocfg_path.is_empty() {
                    PathBuf::from(&config.sumocfg_path)
                } else {
                    let default_path = PathBuf::from(&config.scenario_dir).join("osm.sumocfg");
                    if default_path.exists() {
                        default_path
                    } else {
                        // Scan for any *.sumocfg in scenario_dir
                        let mut found = None;
                        if let Ok(entries) = std::fs::read_dir(&config.scenario_dir) {
                            for entry in entries.flatten() {
                                let path = entry.path();
                                if path.extension().and_then(|s| s.to_str()) == Some("sumocfg") {
                                    found = Some(path);
                                    break;
                                }
                            }
                        }
                        found.unwrap_or(default_path)
                    }
                };
                let sumo_bin = if config.gui && (config.sumo_binary == "/usr/bin/sumo" || config.sumo_binary == "sumo") {
                    "/usr/bin/sumo-gui".to_string()
                } else {
                    config.sumo_binary.clone()
                };
                let sumo_cfg = SumoMobilityConfig {
                    sumo_binary: sumo_bin,
                    sumocfg_path: sumocfg,
                    traci_host: config.traci_host.clone(),
                    traci_port: config.traci_port,
                    gui: config.gui,
                    num_vehicles: config.num_nodes,
                    step_length_s: config.mobility_step_s,
                    seed: config.seed,
                };
                Box::new(SumoMobilitySource::new(sumo_cfg)?)
            }
            SyntheticMobilityType::Highway => Box::new(HighwayMobilityGenerator::new(
                config.num_nodes,
                2000.0,
                3,
                config.seed,
            )),
            SyntheticMobilityType::Urban => Box::new(UrbanGridMobilityGenerator::new(
                config.num_nodes,
                1500.0,
                100.0,
                config.seed,
            )),
            SyntheticMobilityType::Platoon => Box::new(PlatoonMobilityGenerator::new(
                config.num_nodes,
                25.0, // 90 km/h
                20.0, // 20 m initial gap
            )),
            SyntheticMobilityType::Replay => Box::new(HighwayMobilityGenerator::new(
                config.num_nodes,
                2000.0,
                3,
                config.seed,
            )),
        };

        let spatial_index = SpatialIndex::new();
        let mut radio_medium = RadioMedium::new(config.phy_model, 23.0);
        radio_medium.legacy_poly = config.parametric_phy.clone();

        // Open PCAP writer if enabled
        let pcap_writer = if config.enable_pcap {
            let pcap_path = Path::new(&config.output_dir).join("simulation.pcap");
            Some(PcapWriter::open(pcap_path)?)
        } else {
            None
        };

        let geodetic = GeodeticTransform::default();

        let pki = if config.security_enabled {
            Some(SimPki::new())
        } else {
            None
        };

        let mut engine = Self {
            config,
            scheduler,
            profiler,
            trajectory,
            spatial_index,
            buildings,
            radio_medium,
            agents: HashMap::new(),
            pcap_writer,
            geodetic,
            rng,
            next_packet_id: 1,
            pki,
        };

        engine.init_agents();
        engine.init_initial_events()?;

        Ok(engine)
    }

    fn init_agents(&mut self) {
        let initial_vehicles = self.trajectory.get_vehicles().to_vec();
        let mut node_entries = Vec::with_capacity(initial_vehicles.len());

        if self.config.security_enabled {
            let ids: Vec<u32> = initial_vehicles.iter().map(|vk| vk.id).collect();
            if let Some(ref mut pki) = self.pki {
                pki.pregenerate_ats(&ids);
            }
        }

        for vk in initial_vehicles {
            let id = vk.id;
            node_entries.push(NodeEntry::new(id, vk.position));
            self.create_and_insert_agent(vk);
        }

        self.spatial_index.rebuild(node_entries);
    }

    pub fn create_and_insert_agent(&mut self, vk: VehicleKinematics) {
        let id = vk.id;
        let mac: [u8; 6] = [
            0x02,
            0x00,
            ((id >> 24) & 0xFF) as u8,
            ((id >> 16) & 0xFF) as u8,
            ((id >> 8) & 0xFF) as u8,
            (id & 0xFF) as u8,
        ];

        let mut mib = Mib::new();
        mib.itsGnLocalGnAddr = GNAddress::new(M::GnMulticast, ST::PassengerCar, MID::new(mac));
        mib.itsGnBeaconServiceRetransmitTimer = 0;
        if self.config.security_enabled {
            mib.itsGnSecurity = GnSecurity::Enabled;
        }

        let (sign_service, verify_backend, verify_cert_library) = if self.config.security_enabled {
            let pki = self.pki.as_mut().unwrap();
            let creds = pki.issue_at(id);
            let (s, vb, vcl) = pki.setup_vehicle_security(&creds, self.config.verify_packets);
            (Some(s), vb, vcl)
        } else {
            (None, None, None)
        };

        let (gn_handle, gn_to_ll_rx, gn_to_btp_rx) =
            GNRouter::spawn(mib, sign_service, verify_backend, verify_cert_library);
        let (btp_handle, btp_to_gn_rx) = BTPRouter::spawn(mib);

        let link_layer = SimulatorLinkLayer::new(
            id,
            gn_to_ll_rx,
            gn_handle.clone(),
            gn_to_btp_rx,
            btp_handle.clone(),
            btp_to_gn_rx,
        );

        let coord = vk.to_wgs84(self.geodetic.orig_lat, self.geodetic.orig_lon);
        let lat_etsi =
            ((coord.latitude * 1e7).round() as i64).clamp(-900_000_000, 900_000_000) as i32;
        let lon_etsi = ((coord.longitude * 1e7).round() as i64)
            .clamp(-1_800_000_000, 1_800_000_000) as i32;
        let ldm = LdmFacility::new(lat_etsi, lon_etsi, 2000.0);

        let location_service = LocationService::new();
        let cam_trigger = CamTriggerEvaluator::new(id, true);

        let vehicle_data = VehicleData {
            station_id: id,
            station_type: 5,
            ..VehicleData::default()
        };
        let (ca_svc, cam_rx) = CooperativeAwarenessBasicService::new(
            btp_handle.clone(),
            vehicle_data,
            Some(ldm.clone()),
        );
        let (_dummy_tx, dummy_rx) = std::sync::mpsc::channel();
        ca_svc.start(dummy_rx);
        let cam_transmission = ca_svc.cam_transmission_management.clone();
        cam_transmission.start();

        let den_veh_data = DenmVehData {
            station_id: id,
            station_type: 5,
        };
        let (den_service, denm_rx) = DecentralizedEnvironmentalNotificationService::new(
            btp_handle.clone(),
            den_veh_data,
        );

        let mac_access = EdcaChannelAccess::new(-85.0);
        let channel_monitor = ChannelMonitor::default();

        let dcc_reactive = match self.config.dcc_config.mode {
            DccMode::Reactive => Some(ReactiveDccFsm::new()),
            _ => None,
        };

        let dcc_limeric = match self.config.dcc_config.mode {
            DccMode::Limeric => Some(LimericDcc::new(
                self.config.dcc_config.target_cbr,
                self.config.dcc_config.alpha,
                self.config.dcc_config.beta,
                self.config.dcc_config.min_rate_hz,
                self.config.dcc_config.max_rate_hz,
            )),
            _ => None,
        };

        let tech: Box<dyn WirelessTechnology> = match self.config.technology {
            WirelessTechnologyType::Dsrc80211p => {
                let mcs_11p = self.config.mcs.map(Mcs80211p::from_index).unwrap_or_default();
                Box::new(Dsrc80211pModel::with_mcs(23.0, self.config.seed + (id as u64), mcs_11p))
            }
            WirelessTechnologyType::Wifi80211bd => {
                let mcs_11bd = self.config.mcs.map(Mcs80211bd::from_index).unwrap_or(Mcs80211bd::Mcs2Qpsk3_4);
                Box::new(Wifi80211bdModel::new(
                    Wifi80211bdConfig {
                        tx_power_dbm: 23.0,
                        mcs: mcs_11bd,
                        ..Default::default()
                    },
                    self.config.seed + (id as u64),
                ))
            }
            WirelessTechnologyType::Cv2xMode4 => {
                let pssch_mcs = self.config.mcs.unwrap_or(7).min(28);
                Box::new(Cv2xMode4Model::new(
                    Cv2xConfig {
                        tx_power_dbm: 23.0,
                        pssch_mcs,
                        ..Default::default()
                    },
                    self.config.seed + (id as u64),
                ))
            }
            WirelessTechnologyType::NrV2xMode2 => {
                let pssch_mcs = self.config.mcs.unwrap_or(10).min(28);
                Box::new(NrV2xMode2Model::new(
                    NrV2xConfig {
                        tx_power_dbm: 23.0,
                        pssch_mcs,
                        ..Default::default()
                    },
                    self.config.seed + (id as u64),
                ))
            }
            WirelessTechnologyType::ParametricEmpirical => {
                Box::new(ParametricEmpiricalModel::new(
                    self.config.parametric_phy.clone(),
                    self.config.seed + (id as u64),
                ))
            }
            _ => {
                let mcs_11p = self.config.mcs.map(Mcs80211p::from_index).unwrap_or_default();
                Box::new(Dsrc80211pModel::with_mcs(23.0, self.config.seed + (id as u64), mcs_11p))
            }
        };

        let agent = VehicleAgent {
            id,
            mac_addr: mac,
            kinematics: vk,
            gn_handle,
            btp_handle,
            location_service,
            cam_trigger,
            cam_transmission,
            cam_rx,
            den_service,
            denm_rx,
            ldm,
            link_layer,
            mac: mac_access,
            channel_monitor,
            tech,
            dcc_reactive,
            dcc_limeric,
            last_cbr: 0.0,
        };

        self.agents.insert(id, agent);
    }

    fn init_initial_events(&mut self) -> SimResult<()> {
        // Schedule periodic mobility stepping
        self.scheduler.schedule(
            SimTime::ZERO,
            1,
            0,
            EventPayload::MobilityStep { step_index: 0 },
        )?;

        // Schedule periodic CBR monitoring (every 100 ms)
        for id in self.agents.keys() {
            self.scheduler.schedule(
                SimTime::from_millis(100),
                5,
                *id,
                EventPayload::CbrUpdate { node_id: *id },
            )?;
        }

        Ok(())
    }

    /// Advance simulation by executing a single discrete event.
    pub fn step(&mut self) -> SimResult<Option<Event>> {
        if let Some(event) = self.scheduler.step()? {
            self.profiler.record_event();

            match event.payload {
                EventPayload::MobilityStep { step_index } => {
                    self.handle_mobility_step(step_index)?;
                }
                EventPayload::MacTxStart {
                    packet_id,
                    sender_id,
                    access_category,
                    ref payload_bytes,
                } => {
                    self.handle_mac_tx(
                        packet_id,
                        sender_id,
                        access_category,
                        payload_bytes.clone(),
                    )?;
                }
                EventPayload::PhyRxDelivery {
                    packet_id,
                    sender_id,
                    receiver_id,
                    ref payload_bytes,
                    rx_power_dbm,
                    sinr_db,
                    is_success,
                } => {
                    self.handle_phy_rx(
                        packet_id,
                        sender_id,
                        receiver_id,
                        payload_bytes.clone(),
                        rx_power_dbm,
                        sinr_db,
                        is_success,
                    )?;
                }
                EventPayload::CbrUpdate { node_id } => {
                    self.handle_cbr_update(node_id)?;
                }
                _ => {}
            }

            Ok(Some(event))
        } else {
            Ok(None)
        }
    }

    /// Advance simulation to completion.
    pub fn run(&mut self) -> SimResult<()> {
        let mut last_log_time_s = 0;

        while let Some(_event) = self.step()? {
            let now_s = self.scheduler.current_time().to_secs_f64().floor() as u64;
            if now_s > last_log_time_s {
                last_log_time_s = now_s;
                let active_nodes = self.agents.len();
                let avg_cbr: f64 = if active_nodes > 0 {
                    self.agents.values().map(|a| a.last_cbr).sum::<f64>() / active_nodes as f64
                } else {
                    0.0
                };
                info!(
                    "[Sim t={:4.1}s / {:4.1}s] Active Nodes: {:3} | Avg CBR: {:5.2}% | Events: {}",
                    now_s as f64,
                    self.config.duration_s,
                    active_nodes,
                    avg_cbr * 100.0,
                    self.profiler.summary().total_events_count
                );
            }
        }

        // Finalize PCAP
        if let Some(ref mut pcap) = self.pcap_writer {
            pcap.close()?;
        }

        Ok(())
    }

    fn handle_mobility_step(&mut self, step_index: u64) -> SimResult<()> {
        let now = self.scheduler.current_time();
        let dt_s = self.config.mobility_step_s;

        for (id, agent) in &self.agents {
            if let Some(veh) = self.trajectory.get_vehicle_mut(*id) {
                veh.speed = agent.kinematics.speed;
                veh.acceleration = agent.kinematics.acceleration;
            }
        }

        // 1. Advance microscopic vehicle kinematics
        let updated_kinematics = {
            let _guard = self.profiler.start_timer(ProfileCategory::Mobility);
            self.trajectory.step(dt_s)
        };

        // 2. Update agent states and rebuild KD-Tree
        let mut entries = Vec::with_capacity(updated_kinematics.len());
        for vk in updated_kinematics {
            let id = vk.id;
            entries.push(NodeEntry::new(id, vk.position));
            if let Some(agent) = self.agents.get_mut(&id) {
                agent.kinematics = vk;
            } else {
                self.create_and_insert_agent(vk);
                self.scheduler.schedule(
                    now + SimTime::from_millis(100),
                    5,
                    id,
                    EventPayload::CbrUpdate { node_id: id },
                )?;
            }
        }
        self.spatial_index.rebuild(entries);

        // 3. Evaluate C-ITS Triggers & Queue Transmissions
        {
            let _guard = self.profiler.start_timer(ProfileCategory::Stack);

            // Phase 1: Application triggers & generation across all active agents
            for agent in self.agents.values_mut() {
                let ego_coord = agent
                    .kinematics
                    .to_wgs84(self.geodetic.orig_lat, self.geodetic.orig_lon);

                // Update GN router ego position vector
                let lpv = create_long_position_vector(
                    agent.id,
                    agent.mac_addr,
                    now,
                    &ego_coord,
                    agent.kinematics.speed,
                    agent.kinematics.heading_deg,
                );
                agent.gn_handle.update_position_vector(lpv);

                // Publish GPS fix to location service
                let fix = GpsFix {
                    latitude: ego_coord.latitude,
                    longitude: ego_coord.longitude,
                    altitude_m: ego_coord.altitude,
                    speed_mps: agent.kinematics.speed,
                    heading_deg: agent.kinematics.heading_deg,
                    pai: true,
                };
                agent.location_service.publish(fix);

                // Evaluate dynamic CAM generation triggers
                let should_cam = agent.cam_trigger.should_generate(
                    now,
                    agent.kinematics.position,
                    agent.kinematics.speed,
                    agent.kinematics.heading_deg,
                );
                if should_cam {
                    let _ = agent.cam_transmission.generate_and_send_cam(
                        &fix,
                        now.to_millis(),
                        1,
                    );
                }
            }

            // Phase 2: Synchronous multi-stage TX bridge draining loop.
            // Pumps cross-channel bridges 10 times across all active agents with micro-sleeps.
            // Gives background BTP and GeoNetworking worker threads ~1.5 ms of wall-clock time
            // to serialize ASN.1 and route packets into gn_to_ll_rx in the CURRENT simulation tick.
            for _ in 0..10 {
                for agent in self.agents.values_mut() {
                    agent.link_layer.drain_bridges();
                }
                std::thread::sleep(std::time::Duration::from_micros(150));
            }
            for agent in self.agents.values_mut() {
                agent.link_layer.drain_bridges();
            }

            // Phase 3: Drain outbound link-layer frames and enqueue into MAC in the same tick
            let mut outbound_ready = Vec::new();
            for agent in self.agents.values_mut() {
                let frames = agent.link_layer.drain_outbound_frames(now);
                for (ac, frame) in frames {
                    // For CAM BestEffort traffic, discard any stale unsent frames per ETSI TS 103 900
                    if ac == AccessCategory::BestEffort {
                        agent.mac.queues[ac as usize].buffer.clear();
                    }
                    let payload_copy = frame.payload.clone();
                    if let Some(tx_time) = agent.tech.enqueue_outbound(frame, ac as u8, now, &mut self.rng) {
                        outbound_ready.push((agent.id, ac as u8, payload_copy, tx_time));
                    }
                }
            }

            // Schedule physical transmissions with technology-specific medium access delay
            for (sender_id, ac, payload, tx_time) in outbound_ready {
                let pkt_id = self.next_packet_id;
                self.next_packet_id += 1;

                self.scheduler.schedule(
                    tx_time,
                    0,
                    sender_id,
                    EventPayload::MacTxStart {
                        packet_id: pkt_id,
                        sender_id,
                        access_category: ac,
                        payload_bytes: payload,
                    },
                )?;
            }
        }

        // Schedule next periodic mobility step
        let next_step_time = now + SimTime::from_secs_f64(dt_s);
        self.scheduler.schedule(
            next_step_time,
            1,
            0,
            EventPayload::MobilityStep {
                step_index: step_index + 1,
            },
        )?;

        Ok(())
    }

    fn handle_mac_tx(
        &mut self,
        packet_id: u64,
        sender_id: ActorId,
        access_category: u8,
        payload_bytes: Vec<u8>,
    ) -> SimResult<()> {
        let now = self.scheduler.current_time();
        self.profiler.record_net_event();

        let _guard = self.profiler.start_timer(ProfileCategory::Channel);

        let sender_pos = match self.agents.get(&sender_id) {
            Some(a) => a.kinematics.position,
            None => return Ok(()),
        };

        // Compute frame transmission duration based on wireless technology
        let duration = match self.config.technology {
            WirelessTechnologyType::Dsrc80211p | WirelessTechnologyType::ParametricEmpirical => {
                let mcs_11p = self.config.mcs.map(Mcs80211p::from_index).unwrap_or_default();
                compute_duration_80211p(payload_bytes.len(), mcs_11p)
            }
            WirelessTechnologyType::Wifi80211bd => {
                let mcs_11bd = self.config.mcs.map(Mcs80211bd::from_index).unwrap_or(Mcs80211bd::Mcs2Qpsk3_4);
                compute_duration_80211bd(
                    payload_bytes.len(),
                    mcs_11bd,
                    10.0,
                    MidamblePeriod::Every8Symbols,
                    false,
                )
            }
            WirelessTechnologyType::Cv2xMode4 => SimTime::from_micros(1000),
            WirelessTechnologyType::NrV2xMode2 => SimTime::from_micros(500),
            _ => SimTime::from_micros(300),
        };
        let air_pkt = AirPacket::new(
            packet_id,
            sender_id,
            self.radio_medium.tx_power_dbm,
            now,
            duration,
            payload_bytes.clone(),
            access_category,
        );

        // Record channel busy interval on transmitter
        if let Some(sender_agent) = self.agents.get_mut(&sender_id) {
            sender_agent.tech.record_channel_activity(now, now + duration);
            sender_agent
                .channel_monitor
                .record_busy_interval(now, now + duration);
        }

        // Register in physical radio medium with position
        self.radio_medium.purge_completed_signals(now);
        self.radio_medium.register_transmission_with_pos(&air_pkt, sender_pos);

        // payload_bytes is already the full GeoNetworking packet generated by GNRouter!
        // Write to PCAP trace if enabled
        if let Some(ref mut pcap) = self.pcap_writer {
            pcap.write_frame(now, sender_id, &payload_bytes)?;
        }

        // Query candidate receivers within 800m via KD-tree
        let candidate_receivers = self
            .spatial_index
            .query_radius(sender_pos, 800.0, sender_id);

        for (rx_id, _, _dist_m) in &candidate_receivers {
            if let Some(rx_agent) = self.agents.get_mut(rx_id) {
                rx_agent.tech.record_channel_activity(now, now + duration);
                rx_agent
                    .channel_monitor
                    .record_busy_interval(now, now + duration);
            }
        }

        // Technology-specific physical layer link evaluation
        let results = if let Some(sender_agent) = self.agents.get_mut(&sender_id) {
            sender_agent.tech.evaluate_transmission(
                &air_pkt,
                sender_pos,
                &candidate_receivers,
                &self.buildings,
                now,
                &mut self.rng,
            )
        } else {
            self.radio_medium.evaluate_broadcast(
                &air_pkt,
                sender_pos,
                &candidate_receivers,
                &self.buildings,
                self.config.seed,
            )
        };

        // Schedule Rx delivery events at receivers (delivery delay from PHY model)
        for res in results {
            let delivery_time = now + res.delivery_delay;
            self.scheduler.schedule(
                delivery_time,
                2,
                res.receiver_id,
                EventPayload::PhyRxDelivery {
                    packet_id,
                    sender_id,
                    receiver_id: res.receiver_id,
                    payload_bytes: payload_bytes.clone(),
                    rx_power_dbm: res.rx_power_dbm,
                    sinr_db: res.sinr_db,
                    is_success: res.is_success,
                },
            )?;
        }

        Ok(())
    }

    #[allow(clippy::too_many_arguments)]
    fn handle_phy_rx(
        &mut self,
        _packet_id: u64,
        sender_id: ActorId,
        receiver_id: ActorId,
        payload_bytes: Vec<u8>,
        _rx_power_dbm: f64,
        _sinr_db: f64,
        is_success: bool,
    ) -> SimResult<()> {
        let _now = self.scheduler.current_time();
        self.profiler.record_net_event();

        let sender_pos = self.agents.get(&sender_id).map(|a| a.kinematics.position);
        let receiver_pos = self.agents.get(&receiver_id).map(|a| a.kinematics.position);

        let _distance_m = match (sender_pos, receiver_pos) {
            (Some(p1), Some(p2)) => p1.distance_to(&p2),
            _ => 100.0,
        };

        if !is_success {
            return Ok(());
        }

        let _guard = self.profiler.start_timer(ProfileCategory::Stack);

        // Check security info from packet header if secured
        let sec_tag = if payload_bytes.len() >= 4 {
            let bh = BasicHeader::decode([
                payload_bytes[0],
                payload_bytes[1],
                payload_bytes[2],
                payload_bytes[3],
            ]);
            if matches!(bh.nh, BasicNH::SecuredPacket) {
                let dot2 = decode_ieee1609_dot2_data(&payload_bytes[4..]);
                if let Ieee1609Dot2Content::signedData(sd) = dot2.content {
                    match sd.signer {
                        SignerIdentifier::certificate(_) => " [SEC:certificate]",
                        SignerIdentifier::digest(_) => " [SEC:digest]",
                        _ => " [SEC:other]",
                    }
                } else {
                    " [SEC:unknown]"
                }
            } else {
                ""
            }
        } else {
            ""
        };

        // Inject decoded GeoNetworking frame into receiver's link layer
        if let Some(agent) = self.agents.get_mut(&receiver_id) {
            agent.link_layer.inject_rx(&payload_bytes);
            for _ in 0..10 {
                agent.link_layer.drain_bridges();
                std::thread::sleep(std::time::Duration::from_micros(150));
            }
            agent.link_layer.drain_bridges();

            // Process received CAMs (LDM is automatically updated by CAMReceptionManagement)
            while let Ok(cam) = agent.cam_rx.try_recv() {
                let summary = summarize_cam(&cam);
                let lf_tag = if summary.has_low_frequency_container {
                    " [LF]"
                } else {
                    ""
                };
                info!(
                    "[CAM RX] node={} <- sender={} lat={:.5} lon={:.5} speed={:.1}m/s{}{}",
                    receiver_id,
                    summary.station_id,
                    summary.latitude,
                    summary.longitude,
                    summary.speed_m_s,
                    lf_tag,
                    sec_tag
                );
            }

            // Drain received DENMs
            while let Ok(denm) = agent.denm_rx.try_recv() {
                info!(
                    "[DENM RX] node={} <- sender={}",
                    receiver_id, denm.header.station_id.0
                );
            }
        }

        Ok(())
    }

    fn handle_cbr_update(&mut self, node_id: ActorId) -> SimResult<()> {
        let now = self.scheduler.current_time();

        if let Some(agent) = self.agents.get_mut(&node_id) {
            let cbr = agent.tech.evaluate_cbr(now);
            agent.last_cbr = cbr;

            // Feed CBR to DCC rate controller and update CAM generation interval
            let t_off = if let Some(ref mut fsm) = agent.dcc_reactive {
                Some(fsm.update_cbr(cbr))
            } else {
                agent.dcc_limeric.as_mut().map(|limeric| limeric.update_cbr(cbr))
            };

            if let Some(t_off) = t_off {
                agent.cam_trigger.t_min = t_off;
            }
        }

        // Schedule next CBR update (every 100 ms)
        let next_update = now + SimTime::from_millis(100);
        self.scheduler
            .schedule(next_update, 5, node_id, EventPayload::CbrUpdate { node_id })?;

        Ok(())
    }
}
