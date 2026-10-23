# Command-Line Interface (CLI) Reference

`rustv2xsimulator` is architected natively as an open-source library crate. For direct command-line execution, benchmarking, and visualization, the repository provides the full-featured **`cam_simulation`** example binary (`examples/cam_simulation.rs`).

---

## 1. Running Simulations via `cam_simulation`

```bash
cargo run --release --example cam_simulation -- [OPTIONS]
```

### Options Reference

| Flag / Option | Type | Default | Description |
| :--- | :---: | :---: | :--- |
| `-c, --config <FILE>` | String | `None` | Path to scenario TOML configuration file |
| `--scenario <NAME>` | String | `"diagonal"` | Scenario name (`diagonal`, `highway`, `little`, `trinitat`) |
| `--scenario-dir <DIR>` | String | `"scenarios/diagonal"` | Directory containing `osm.sumocfg` & `osm.poly.xml` |
| `--sumocfg <FILE>` | String | `None` | Explicit path to `.sumocfg` file |
| `--sumo-binary <PATH>` | String | `"/usr/bin/sumo"` | SUMO binary path (`sumo` or `sumo-gui`) |
| `--gui` | Flag | `false` | Launch SUMO GUI window for live visual simulation |
| `--traci-host <HOST>` | String | `"127.0.0.1"` | TraCI socket server host |
| `--traci-port <PORT>` | Integer | `8813` | TraCI socket server port |
| `-w, --workload <TYPE>` | String | `"cam"` | Traffic workload: `"cam"` |
| `-n, --nodes <N>` | Integer | `5` | Number of vehicles to simulate (alias `--num-vehicles`) |
| `-d, --duration <SECS>` | Float | `30.0` | Total simulation duration in seconds |
| `--technology <TECH>` | String | `"11p"` | Wireless technology: `"11p"`, `"11bd"`, `"cv2x"`, `"nrv2x"`, or `"parametric"` |
| `--phy-model <MODEL>` | String | `"dualslope"` | Wireless physical model: `"dualslope"` or `"legacy_poly"` |
| `--mcs <INDEX>` | Integer | `None` | MCS index override: 0..7 (11p), 0..9 (11bd), 0..28 (cv2x/nrv2x) |
| `--dcc-mode <MODE>` | String | `"reactive"` | DCC mode: `"off"`, `"reactive"` (ETSI TS 102 687), or `"limeric"` |
| `--mobility <SOURCE>` | String | `"sumo"` | Mobility source: `"sumo"`, `"urban"`, `"highway"`, or `"platoon"` |
| `--seed <U64>` | Integer | `42` | PRNG seed for deterministic reproducibility |
| `--enable-pcap` | Flag | `false` | Generate Ethernet II PCAP packet trace (`simulation.pcap`, alias `--generate-pcap`) |
| `-o, --output-dir <DIR>` | String | `"results/cam_example/"` | Output directory for simulation traces |
| `--no-security` | Flag | `false` | Disable ETSI TS 103 097 security signing and verification |

---

## 2. Common CLI Invocations

### Headless CAM Simulation with PCAP Capture
```bash
cargo run --release --example cam_simulation -- \
    --scenario diagonal \
    --nodes 10 \
    --duration 30.0 \
    --enable-pcap
```

### Live Visualization with SUMO-GUI
```bash
cargo run --release --example cam_simulation -- \
    --scenario diagonal \
    --gui \
    --nodes 10 \
    --duration 30.0
```

### 5G NR-V2X Sidelink Mode 2 with Custom MCS
```bash
cargo run --release --example cam_simulation -- \
    --technology nrv2x \
    --mcs 10 \
    --nodes 15 \
    --duration 30.0
```

### Fast Synthetic Highway Platoon without SUMO
```bash
cargo run --release --example cam_simulation -- \
    --mobility platoon \
    --nodes 10 \
    --duration 20.0
```

### Using Declarative TOML Scenarios
```bash
cargo run --release --example cam_simulation -- \
    --config config/env1_diagonal.toml
```
