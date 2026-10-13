# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

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
