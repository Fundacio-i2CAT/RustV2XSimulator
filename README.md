# RustV2XSimulator

<img src="https://raw.githubusercontent.com/Fundacio-i2CAT/FlexStack/refs/heads/master/doc/img/i2cat_logo.png" alt="i2CAT Logo" width="200"/>

![Rust](https://img.shields.io/badge/rust-1.85%2B-orange)
![License](https://img.shields.io/badge/license-AGPL--3.0-blue)
[![Documentation](https://img.shields.io/badge/docs-GitHub%20Pages-blue.svg)](https://fundacio-i2cat.github.io/RustV2XSimulator/)

# Short description

**RustV2XSimulator** is a high-performance, modular discrete-event simulation (DES) framework for vehicular communications (V2X) and Cooperative Intelligent Transport Systems (C-ITS) natively implemented in Rust. Its aim is to provide a standard-compliant, memory-safe, and computationally ultra-fast alternative to monolithic simulation environments (such as ns-3 and OMNeT++/Veins) for research in connected and automated mobility.

Operating with a deterministic 100-nanosecond calendar queue scheduler (`SimTime`), it bridges microscopic vehicular mobility (via Eclipse SUMO TraCI or synthetic high-density traffic generators) with standard-compliant ETSI C-ITS protocol stacks ([`rustflexstack`](https://crates.io/crates/rustflexstack)) and multi-technology wireless communications:

- **IEEE 802.11p DSRC**: EDCA 4 Access Categories (`AC_VO`, `AC_VI`, `AC_BE`, `AC_BK`), dual-slope log-distance path loss ($d_{\mathrm{break}} = 120\,\mathrm{m}$), ITU-R P.526 knife-edge corner diffraction, Rayon-parallel broadcast evaluation, and ns-3 v3.42 piecewise chunked cumulative SINR tracking with 4.0 dB capture effect.
- **IEEE 802.11bd Next-Gen V2X**: Mixed PPDU framing, Doppler-tracking midambles for high-speed highway mobility (up to 220 km/h), QC-LDPC forward error correction, and Dual Carrier Modulation (DCM).
- **3GPP C-V2X PC5 Mode 4**: 2D time-frequency subchannel resource grid with Sensing-Based Semi-Persistent Scheduling (SB-SPS) and dual PSCCH/PSSCH BLER decoding.
- **3GPP 5G NR-V2X Sidelink Mode 2**: Flexible numerologies ($\mu \in \{0, 1, 2\}$, 15/30/60 kHz SCS), Two-Stage SCI (Format 1-A & Format 2-A), Release 16 $T_3$ re-evaluation lookahead, priority pre-emption, and Stop-and-Wait PSFCH closed-loop HARQ feedback.
- **Parametric Empirical Model**: Fast 10th-degree polynomial distance loss, logistic spatio-temporal congestion calculation, and Gaussian delay for large-scale macroscopic studies.
- **Wireshark libpcap Trace Export**: Direct export to `.pcap` capture files containing raw Ethernet II frames with EtherType `0x8947` (ETSI GeoNetworking), dissectible in Wireshark.

# Documentation

Extensive documentation is available in the [`docs/`](docs/) directory and online at [fundacio-i2cat.github.io/RustV2XSimulator](https://fundacio-i2cat.github.io/RustV2XSimulator/).

Local documentation can be served using MkDocs:

```bash
mkdocs serve
```

Library API documentation can be generated locally by running:

```bash
cargo doc --open
```

# Pre-requisites

## Supported Operating Systems

This library can run on any system that supports Rust 1.85 or higher, including Linux (Ubuntu 20.04/22.04/24.04 recommended), macOS, and Windows.

Microscopic vehicular traffic simulation with TraCI requires **Eclipse SUMO** ($\ge 1.18$). Alternatively, built-in synthetic mobility models (highways, platoons, urban Manhattan grids) run headlessly without external dependencies.

Cross-compilation for embedded or non-x86 targets (e.g. `aarch64-unknown-linux-gnu`) is fully supported.

## Dependencies

All dependencies are managed by Cargo and declared in `Cargo.toml`:

| Crate | Version | Purpose |
|-------|---------|---------|
| `rayon` | 1.10 | Multi-core parallel physical broadcast link evaluation |
| `crossbeam` / `crossbeam-channel` | 0.8 / 0.5 | Multi-threaded simulation channels and concurrent queues |
| `rustflexstack` | 0.3.4 | ETSI C-ITS protocol stack (GeoNetworking, BTP, CAM, DENM, Security) |
| `rasn` | 0.26 | ASN.1 UPER encoding and decoding for C-ITS messages |
| `sumo-traci` (`traci-rs`) | 0.2 | Eclipse SUMO TraCI client protocol interface |
| `kd-tree` / `ordered-float` | 0.6 / 4.2 | 2D spatial indexing for $O(\log N)$ neighbor radius queries |
| `utm` | 0.1 | UTM coordinate projections and WGS-84 geodetic transformations |
| `rand` / `rand_distr` | 0.8 / 0.4 | PRNG and standard statistical distributions (Gaussian, Log-Normal) |
| `serde` / `serde_json` / `toml` | 1.0 / 1.0 / 0.8 | Scenario configuration serialization and parsing |
| `quick-xml` | 0.36 | Fast streaming parser for SUMO `osm.poly.xml` building shapes |
| `byteorder` / `zstd` | 1.5 / 0.13 | Binary PCAP stream writing and trace compression |
| `clap` | 4.5 | Command-line argument parsing for examples |
| `log` / `env_logger` | 0.4 / 0.11 | Configurable logging facade and terminal output |

## Build tools

Requires a stable Rust toolchain. Install via [rustup](https://rustup.rs):

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

To build the library:

```bash
cargo build --release
```

To run all unit and integration tests:

```bash
cargo test --all-targets --all-features
```

To run Criterion microbenchmarks:

```bash
cargo bench
```

To build and run examples:

```bash
# Option A: Headless SUMO with 10 vehicles exchanging CAMs (with DCC & Security)
cargo run --release --example cam_simulation -- \
    --scenario diagonal \
    --scenario-dir scenarios/diagonal \
    --nodes 10 \
    --duration 30.0 \
    --enable-pcap

# Option B: Watch live in SUMO-GUI
cargo run --release --example cam_simulation -- \
    --gui \
    --scenario diagonal \
    --scenario-dir scenarios/diagonal \
    --nodes 10 \
    --duration 30.0

# Option C: 5G NR-V2X Sidelink Mode 2 with custom MCS
cargo run --release --example cam_simulation -- \
    --scenario diagonal \
    --technology nrv2x \
    --mcs 10 \
    --nodes 10 \
    --duration 30.0
```

### Cross-compilation for aarch64

1. Add the target:
   ```bash
   rustup target add aarch64-unknown-linux-gnu
   ```
2. Install the cross-linker (Debian/Ubuntu):
   ```bash
   sudo apt install gcc-aarch64-linux-gnu
   ```
3. Configure the linker in `.cargo/config.toml`:
   ```toml
   [target.aarch64-unknown-linux-gnu]
   linker = "aarch64-linux-gnu-gcc"
   ```
4. Build:
   ```bash
   cargo build --release --target aarch64-unknown-linux-gnu
   ```

Alternatively, use [`cross`](https://github.com/cross-rs/cross) for a Docker-based, zero-configuration cross-compilation:

```bash
cargo install cross
cross build --release --target aarch64-unknown-linux-gnu
```

## Known Limitations

- Eclipse SUMO integration requires `sumo` (or `sumo-gui`) installed and accessible in the system PATH when using TraCI mobility mode (`--mobility sumo`).
- Generating large PCAP packet captures over long simulation runs can produce multi-gigabyte trace files on disk.

# Installation

Add the library to your `Cargo.toml`:

```toml
[dependencies]
rustv2xsimulator = { git = "https://github.com/Fundacio-i2CAT/RustV2XSimulator" }
rustflexstack = "0.3.4"
```

Once published on [crates.io](https://crates.io):

```toml
[dependencies]
rustv2xsimulator = "0.1"
rustflexstack = "0.3.4"
```

## Developers

- Jordi Marias-i-Parella (jordi.marias@i2cat.net)

# Source

This code has been developed within the following research and innovation project:

- **CERTAIN** (Grant Agreement No. 101203230) – Funded under the Horizon Europe programme, focusing on resilient and continuous safety assurance methodology for connected, cooperative and automated mobility (CCAM).

# Copyright

This code has been developed by Fundació Privada Internet i Innovació Digital a Catalunya (i2CAT).

i2CAT is a **non-profit research and innovation centre that** promotes mission-driven knowledge to solve business challenges, co-create solutions with a transformative impact, empower citizens through open and participative digital social innovation with territorial capillarity, and promote pioneering and strategic initiatives. i2CAT **aims to transfer** research project results to private companies in order to create social and economic impact via the out-licensing of intellectual property and the creation of spin-offs. Find more information of i2CAT projects and IP rights at https://i2cat.net/tech-transfer/

# License

This code is licensed under the terms of the AGPL. Information about the license can be located at https://www.gnu.org/licenses/agpl-3.0.html.

Please, refer to `rustv2xsimulator` as a dependence of your works.

If you find that this license doesn't fit with your requirements regarding the use, distribution or redistribution of our code for your specific work, please, don't hesitate to contact the intellectual property managers in i2CAT at the following address: techtransfer@i2cat.net. Also, in the following page you'll find more information about the current commercialization status or other licensees: Under Development.

# Attributions

Attributions of Third Party Components of this work:

- `ns-3` Network Simulator - Physical layer chunked cumulative SINR tracking and NIST error rate parity - https://www.nsnam.org - GPL-2.0 / NIST
- `CTTC 5G-LENA` - 3GPP 5G NR Sidelink physical and Mode 2 MAC models - https://5g-lena.cttc.es - GPL-2.0
- `UWICORE Laboratory (UMH)` - Link-level analysis and NR-V2X simulation datasets - https://uwicore.umh.es - GPL-3.0
- `Eclipse SUMO` - Microscopic road traffic simulation engine and TraCI protocol - https://eclipse.dev/sumo - EPL-2.0 / GPL-2.0+
- `Veins & Artery` - Discrete-event vehicular network coupling architectures - https://veins.car2x.org - GPL-2.0
- `rustflexstack` Version 0.3.4 - Imported Rust crate - https://crates.io/crates/rustflexstack - AGPL-3.0
- `rayon` Version 1.10 - Imported Rust crate - https://github.com/rayon-rs/rayon - MIT / Apache-2.0 license
- `crossbeam` Version 0.8 - Imported Rust crate - https://github.com/crossbeam-rs/crossbeam - MIT / Apache-2.0 license
- `rasn` Version 0.26 - Imported Rust crate - https://github.com/XAMPPRocky/rasn - MIT license
- `sumo-traci` (`traci-rs`) Version 0.2 - Imported Rust crate - https://github.com/m-ou-se/traci-rs - MIT / Apache-2.0 license
- `kd-tree` Version 0.6 - Imported Rust crate - https://github.com/mrhooray/kdtree-rs - MIT license
- `utm` Version 0.1 - Imported Rust crate - https://github.com/AnErrup/utm - MIT / Apache-2.0 license
- `rand` Version 0.8 - Imported Rust crate - https://github.com/rust-random/rand - MIT / Apache-2.0 license
- `serde` Version 1.0 - Imported Rust crate - https://github.com/serde-rs/serde - MIT / Apache-2.0 license
- `toml` Version 0.8 - Imported Rust crate - https://github.com/toml-rs/toml - MIT / Apache-2.0 license
- `quick-xml` Version 0.36 - Imported Rust crate - https://github.com/tafia/quick-xml - MIT license
- `clap` Version 4.5 - Imported Rust crate - https://github.com/clap-rs/clap - MIT / Apache-2.0 license
