# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.3.0] - 2026-10-23

### Removed
- **Monaco & CARISSMA Simulation Environments**:
  - Removed `config/env3_monaco.toml` and `config/env4_carissma.toml`.

## [0.2.0] - 2026-10-23

### Removed
- **Krauß-Safe Actuation & EEBL Pipeline**:
  - Removed `src/mobility/krauss_actuation.rs` and `tests/krauss_substep_safety_test.rs`.
  - Removed `ActuationMode`, `ActuationResult`, and `KraussSafeActuator` from public API.
  - Removed `KraussBrakeActuation` from `EventPayload`.
  - Removed `create_eebl_request` and EEBL emergency braking hazard constants from `src/cits/denm.rs`.
  - Removed car-following and hazard fields (`is_braking_hazard`, `min_gap_m`, `max_decel`, `reaction_time_s`) from `VehicleKinematics`.
  - Removed `set_hazard_time_s` from `TrajectorySource` trait and cleaned `PlatoonMobilityGenerator`.
  - Removed `actuator`, `hazard_time_s`, `collision_count`, and EEBL packet interception logic from `SimulationEngine`.
  - Removed `--actuation-mode` CLI flag and `[actuation]` configuration sections.
- **Paper Ablation Flags**:
  - Removed `is_aloha_mac` from `EngineConfig`, `EdcaChannelAccess`, `Dsrc80211pModel`, and `Wifi80211bdModel`.
  - Removed `is_all_pairs_geometry` from `EngineConfig` (engine always utilizes $O(\log N)$ 2D k-d tree spatial indexing).
  - Removed `is_threaded_stack` from `EngineConfig` and `SimulatorLinkLayer`.
  - Removed rigid fixed-rate CAM override `cam_static_10hz`; standard dynamic kinematic triggers are always enabled.
- **Paper Folder**:
  - Removed `paper/` directory and draft assets from repository.

### Changed
- Standardized `SpatialIndex` default instantiation in `SimulationEngine`.
- Simplified `PlatoonMobilityGenerator` to represent uniform vehicular platooning without ad-hoc emergency braking triggers.
- Updated `.gitignore` to ensure all context directories (`aoibenchmark/`, `RustFlexstack/`, `ignore/`) are properly ignored.

## [0.1.0] - 2026-10-13

### Added
- Core discrete-event simulation engine (`SimulationEngine`, `DiscreteEventScheduler`) operating with 100-nanosecond tick resolution (`SimTime`).
- Priority min-heap calendar queue with deterministic FIFO tie-breaking and sub-microsecond event advance.
- Integration with ETSI C-ITS standard protocol stack via `rustflexstack` v0.3.4 (GeoNetworking EN 302 636-4-1, BTP EN 302 636-5-1, CAM TS 103 900, DENM TS 103 898, Security TS 103 097).
- Pluggable `WirelessTechnology` trait abstraction supporting:
  - **IEEE 802.11p DSRC**: EDCA 4 Access Categories, Dual-Slope log-distance path loss, ITU-R P.526 knife-edge diffraction, and ns-3 v3.42 NIST chunked cumulative SINR tracking.
  - **IEEE 802.11bd Next-Gen V2X**: Mixed PPDU framing, Doppler-tracking midambles for high-speed highway mobility (up to 220 km/h), QC-LDPC forward error correction, and Dual Carrier Modulation (DCM).
  - **3GPP C-V2X PC5 Mode 4**: 2D time-frequency subchannel resource grid with Sensing-Based Semi-Persistent Scheduling (SB-SPS) and dual PSCCH/PSSCH BLER decoding.
  - **3GPP 5G NR-V2X Sidelink Mode 2**: Flexible numerologies ($\mu \in \{0, 1, 2\}$, 15/30/60 kHz SCS), Two-Stage SCI (Format 1-A & 2-A), Release 16 $T_3$ re-evaluation lookahead, priority pre-emption, and Stop-and-Wait PSFCH HARQ feedback.
  - **Parametric Empirical Model**: Fast 10th-degree polynomial distance loss, logistic spatio-temporal congestion, and Gaussian delay.
- Microscopic vehicular mobility via Eclipse SUMO TraCI client with sub-stepping Krauß collision avoidance actuation.
- Built-in zero-IPC synthetic mobility generators (highway, platoon, urban grid).
- 2D spatial indexing with k-d tree radius filtering for candidate receiver evaluation.
- Direct libpcap packet capture generation (`EtherType 0x8947`) for Wireshark inspection.
- Dedicated standalone simulation example (`examples/cam_simulation.rs`).
- Ergonomic library prelude (`rustv2xsimulator::prelude::*`) for downstream research crates.
