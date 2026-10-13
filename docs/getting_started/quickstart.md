# Quickstart Guide

This guide walks you through running your first V2X simulation and inspecting the generated results.

---

## 1. Running the CAM Simulation Example

The repository includes a ready-to-run example (`examples/cam_simulation.rs`) that:

1. Launches Eclipse SUMO using the `scenarios/diagonal/` scenario.
2. Equips each simulated vehicle with an ETSI C-ITS stack (`rustflexstack`) generating Cooperative Awareness Messages (CAM).
3. Transmits messages over the wireless medium using IEEE 802.11p EDCA MAC and Rayon-parallel physical layer evaluation.
4. Dynamically regulates transmission frequency using Decentralized Congestion Control (DCC).
5. Exports an Ethernet II packet capture (`simulation.pcap`) with EtherType `0x8947` (GeoNetworking).

Execute the example with:

```bash
cargo run --release --example cam_simulation -- \
    --num-vehicles 10 \
    --scenario diagonal \
    --scenario-dir scenarios/diagonal \
    --duration 30.0 \
    --generate-pcap
```

### Visualizing Vehicle Movement in SUMO GUI

To watch the vehicles driving in real time in the SUMO graphical interface, add the `--gui` flag:

```bash
cargo run --release --example cam_simulation -- \
    --num-vehicles 10 \
    --scenario diagonal \
    --scenario-dir scenarios/diagonal \
    --duration 30.0 \
    --gui
```

---

## 2. Choosing Wireless Models in the CAM Example

The `cam_simulation` example allows you to select any of the five supported wireless communication technologies using the `--technology` flag, along with optional modulation overrides via `--mcs`:

### IEEE 802.11p DSRC (Default)
Operates with EDCA 4 Access Categories, NIST error tables, and chunked cumulative SINR:
```bash
cargo run --release --example cam_simulation -- \
    --technology 11p \
    --num-vehicles 15 \
    --duration 30.0
```

### IEEE 802.11bd Next-Gen V2X (NGV)
Enables 802.11bd mixed PPDU framing, midambles for Doppler tracking at highway speeds, and QC-LDPC coding gain:
```bash
cargo run --release --example cam_simulation -- \
    --technology 11bd \
    --mcs 4 \
    --num-vehicles 15 \
    --duration 30.0
```

### 3GPP C-V2X PC5 Mode 4
Enables LTE-V2X Sensing-Based Semi-Persistent Scheduling (SB-SPS) over a 2D subchannel resource grid with dual PSCCH/PSSCH BLER decider:
```bash
cargo run --release --example cam_simulation -- \
    --technology cv2x \
    --mcs 7 \
    --num-vehicles 15 \
    --duration 30.0
```

### 3GPP 5G NR-V2X Sidelink Mode 2
Enables 5G NR-V2X Mode 2 autonomous sensing, Release 16 $T_3$ Re-evaluation lookahead, Two-Stage SCI, and PSFCH closed-loop HARQ:
```bash
cargo run --release --example cam_simulation -- \
    --technology nrv2x \
    --mcs 10 \
    --num-vehicles 15 \
    --duration 30.0
```

### Fast Parametric Empirical Model
Uses the 10th-degree distance loss polynomial and spatio-temporal logistic congestion calculation for ultra-fast macroscopic benchmarks:
```bash
cargo run --release --example cam_simulation -- \
    --technology parametric \
    --num-vehicles 15 \
    --duration 30.0
```

---

## 3. Inspecting the Generated PCAP Trace

If `--generate-pcap` or `--enable-pcap` is passed, the simulator records every transmitted packet to a standard `.pcap` capture file.

Open the capture in Wireshark:

```bash
wireshark results/cam_example/simulation.pcap
```

In Wireshark:

- The link layer is encapsulated in Ethernet II with protocol **`0x8947`** (GeoNetworking).
- Wireshark's built-in dissector will automatically decode:
  - **GeoNetworking** headers (Basic, Common, Extended Single-Hop Broadcast).
  - **BTP-B** (Basic Transport Protocol) destination port `2001` (CAM).
  - **ETSI CAM** ASN.1 UPER payloads (station ID, generation timestamp, latitude, longitude, speed, heading).

---

## 4. Using Configuration Files

Instead of specifying long command-line options, you can run simulations using declarative TOML configuration files:

```bash
cargo run --release --example cam_simulation -- --config config/env1_diagonal.toml
```

See the [TOML Configuration Reference](../configuration/toml_reference.md) for details on all available fields.
