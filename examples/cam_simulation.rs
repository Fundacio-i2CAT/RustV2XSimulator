//! `cam_simulation` — Complete CAM simulation example using `rustv2xsimulator`.
//!
//! Demonstrates running `rustv2xsimulator` as a library: connects to or launches SUMO / SUMO-GUI
//! via TraCI (or uses synthetic mobility), equips each vehicle with an ETSI C-ITS stack
//! (GeoNetworking + BTP + CAM) with DCC and Security, and executes the discrete-event simulation.

use std::fs;
use std::path::Path;
use clap::Parser;

use rustv2xsimulator::common::SimResult;
use rustv2xsimulator::dcc::dcc_profile::{DccConfig, DccMode};
use rustv2xsimulator::engine::{EngineConfig, SimulationEngine};
use rustv2xsimulator::geometry::polygon::BuildingMap;
use rustv2xsimulator::mobility::trajectory_source::SyntheticMobilityType;
use rustv2xsimulator::phy::radio_medium::PhyModelType;
use rustv2xsimulator::phy::WirelessTechnologyType;

#[derive(Parser, Debug)]
#[command(
    name = "cam_simulation",
    about = "SUMO + ETSI C-ITS CAM Simulation Example using rustv2xsimulator"
)]
struct ExampleArgs {
    /// Path to scenario TOML configuration file
    #[arg(short = 'c', long)]
    config: Option<String>,

    /// Scenario name (diagonal / highway / little)
    #[arg(long, default_value = "diagonal")]
    scenario: String,

    /// Directory containing osm.sumocfg & osm.poly.xml
    #[arg(long, alias = "scenario-dir", default_value = "scenarios/diagonal")]
    scenario_dir: String,

    /// Direct path to .sumocfg file
    #[arg(long)]
    sumocfg: Option<String>,

    /// SUMO executable path
    #[arg(long, alias = "sumo-binary", default_value = "/usr/bin/sumo")]
    sumo_binary: String,

    /// Launch SUMO GUI (sumo-gui) instead of command-line sumo
    #[arg(long, default_value_t = false)]
    gui: bool,

    /// TraCI server host
    #[arg(long, alias = "traci-host", default_value = "127.0.0.1")]
    traci_host: String,

    /// TraCI server port
    #[arg(long, alias = "traci-port", default_value_t = 8813)]
    traci_port: u16,

    /// Simulation workload type: "cam"
    #[arg(short = 'w', long, default_value = "cam")]
    workload: String,

    /// Number of vehicles to simulate
    #[arg(short = 'n', long, alias = "num-vehicles", default_value_t = 5)]
    nodes: usize,

    /// Simulation duration in seconds
    #[arg(short = 'd', long, default_value_t = 30.0)]
    duration: f64,

    /// Wireless communication technology: "11p", "11bd", "cv2x", "nrv2x", "parametric"
    #[arg(long, default_value = "11p")]
    technology: String,

    /// Physical layer path loss model: "dualslope", "legacy_poly"
    #[arg(long, default_value = "dualslope")]
    phy_model: String,

    /// Modulation and Coding Scheme (MCS) index override (e.g. 0-7 for 11p, 0-9 for 11bd, 0-28 for cv2x/nrv2x)
    #[arg(long)]
    mcs: Option<u8>,

    /// Generate Ethernet II PCAP packet capture (EtherType 0x8947)
    #[arg(long, alias = "generate-pcap", default_value_t = false)]
    enable_pcap: bool,

    /// Output directory for simulation CSV, JSON, and PCAP traces
    #[arg(short = 'o', long, default_value = "results/cam_example/")]
    output_dir: String,

    /// Mobility source: "sumo", "urban", "highway", "platoon"
    #[arg(long, default_value = "sumo")]
    mobility: String,

    /// Simulation PRNG seed
    #[arg(long, default_value_t = 42)]
    seed: u64,

    /// Disable ETSI TS 103 097 security layer (signing and verification)
    #[arg(long, default_value_t = false)]
    no_security: bool,

    /// Decentralized Congestion Control mode: "off", "reactive", "limeric"
    #[arg(long, default_value = "reactive")]
    dcc_mode: String,
}

fn main() -> SimResult<()> {
    let _ = env_logger::try_init();
    let args = ExampleArgs::parse();

    let mut engine_cfg = EngineConfig::default();

    // Determine scenario directory and poly file
    let scenario_dir_path = Path::new(&args.scenario_dir);
    let mut poly_file = if scenario_dir_path.join("osm.poly.xml").exists() {
        scenario_dir_path
            .join("osm.poly.xml")
            .to_string_lossy()
            .to_string()
    } else {
        "scenarios/diagonal/osm.poly.xml".to_string()
    };

    if let Some(ref cfg_path) = args.config {
        if Path::new(cfg_path).exists() {
            let content = fs::read_to_string(cfg_path).unwrap_or_default();
            if let Ok(toml_val) = toml::from_str::<toml::Value>(&content) {
                if let Some(scenario) = toml_val.get("scenario") {
                    if let Some(poly) = scenario.get("poly_file").and_then(|v| v.as_str()) {
                        poly_file = poly.to_string();
                    }
                }
            }
        }
    }

    // Apply CLI overrides
    engine_cfg.scenario_name = args.scenario;
    engine_cfg.scenario_dir = args.scenario_dir;
    engine_cfg.sumocfg_path = args.sumocfg.unwrap_or_default();
    engine_cfg.sumo_binary = args.sumo_binary;
    engine_cfg.traci_host = args.traci_host;
    engine_cfg.traci_port = args.traci_port;
    engine_cfg.gui = args.gui;

    engine_cfg.duration_s = args.duration;
    engine_cfg.num_nodes = args.nodes;
    engine_cfg.workload = args.workload;
    engine_cfg.output_dir = args.output_dir;
    engine_cfg.enable_pcap = args.enable_pcap;
    engine_cfg.seed = args.seed;
    engine_cfg.security_enabled = !args.no_security;
    engine_cfg.verify_packets = !args.no_security;

    engine_cfg.mobility_type = match args.mobility.as_str() {
        "highway" => SyntheticMobilityType::Highway,
        "platoon" => SyntheticMobilityType::Platoon,
        "urban" => SyntheticMobilityType::Urban,
        _ => SyntheticMobilityType::Sumo,
    };

    engine_cfg.phy_model = match args.phy_model.as_str() {
        "legacy_poly" | "parametric" => PhyModelType::LegacyPoly,
        _ => PhyModelType::DualSlope,
    };

    engine_cfg.technology = match args.technology.to_lowercase().as_str() {
        "11p" | "dsrc" | "80211p" => WirelessTechnologyType::Dsrc80211p,
        "11bd" | "80211bd" | "ngv" => WirelessTechnologyType::Wifi80211bd,
        "cv2x" | "c-v2x" | "mode4" | "lte-v2x" => WirelessTechnologyType::Cv2xMode4,
        "nrv2x" | "nr-v2x" | "mode2" | "5g-v2x" => WirelessTechnologyType::NrV2xMode2,
        "parametric" => WirelessTechnologyType::ParametricEmpirical,
        other => {
            eprintln!("Warning: unknown technology '{}', defaulting to 11p", other);
            WirelessTechnologyType::Dsrc80211p
        }
    };
    engine_cfg.mcs = args.mcs;

    let dcc_m = match args.dcc_mode.as_str() {
        "off" => DccMode::Disabled,
        "limeric" => DccMode::Limeric,
        _ => DccMode::Reactive,
    };
    engine_cfg.dcc_config = DccConfig {
        mode: dcc_m,
        ..Default::default()
    };

    println!("============================================================");
    println!("  rustv2xsimulator — CAM Simulation with SUMO, DCC & Security");
    println!("============================================================");
    println!("  Scenario       : {}", engine_cfg.scenario_name);
    println!("  Scenario Dir   : {}", engine_cfg.scenario_dir);
    println!("  Mobility       : {:?}", engine_cfg.mobility_type);
    println!("  Vehicles       : {}", engine_cfg.num_nodes);
    println!("  Duration       : {} s", engine_cfg.duration_s);
    println!("  Technology     : {:?}", engine_cfg.technology);
    if let Some(m) = engine_cfg.mcs {
        println!("  MCS Override   : {}", m);
    } else {
        println!("  MCS            : Default");
    }
    println!("  GUI Enabled    : {}", engine_cfg.gui);
    println!("  Security       : {}", engine_cfg.security_enabled);
    println!("  DCC Mode       : {:?}", engine_cfg.dcc_config.mode);
    println!("  PCAP Capture   : {}", engine_cfg.enable_pcap);
    println!("  Output Dir     : {}", engine_cfg.output_dir);
    println!("============================================================");

    // Load building polygon map
    let buildings = if Path::new(&poly_file).exists() {
        BuildingMap::from_osm_poly_xml(&poly_file).unwrap_or_default()
    } else {
        BuildingMap::default()
    };

    let mut engine = SimulationEngine::new(engine_cfg, buildings)?;
    engine.run()?;

    let summary = engine.profiler.summary();
    println!("============================================================");
    println!("  rustv2xsimulator Simulation Completed Successfully");
    println!("============================================================");
    println!("  Processed Events   : {}", summary.total_events_count);
    println!("  Wall-Clock Time    : {:.3} s", summary.t_total_seconds);
    println!(
        "  Overall Throughput : {:.0} events/s",
        summary.overall_events_per_sec
    );
    println!(
        "  Network Throughput : {:.0} events/s",
        summary.theta_net_events_per_sec
    );
    println!(
        "  Time Attribution   : Mob: {:.1}% | Stack: {:.1}% | Chan: {:.1}%",
        summary.t_mob_percent, summary.t_stack_percent, summary.t_chan_percent
    );
    println!("  Output Directory   : {}", engine.config.output_dir);
    println!("============================================================");

    Ok(())
}
