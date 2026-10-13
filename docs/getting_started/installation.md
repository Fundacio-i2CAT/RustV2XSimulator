# Installation & Prerequisites

This page guides you through installing the prerequisites and building **`rustv2xsimulator`**.

---

## 1. System Requirements

`rustv2xsimulator` is supported on Linux (recommended: Ubuntu 20.04/22.04/24.04), macOS, and Windows.

| Dependency | Minimum Version | Recommended | Purpose |
| :--- | :---: | :---: | :--- |
| **Rust Toolchain** | $\ge 1.75$ | Stable (latest) | Core compiler (`rustc`) and package manager (`cargo`) |
| **Eclipse SUMO** | $\ge 1.18$ | $\ge 1.19$ | Microscopic road traffic mobility simulation |
| **libpcap** | any recent | `libpcap-dev` | Wireshark `.pcap` packet trace capture export |
| **Wireshark / tshark** | optional | latest | Offline packet inspection for GeoNetworking & CAM/DENM |

---

## 2. Installing Dependencies

### On Ubuntu / Debian

```bash
# 1. Update system packages
sudo apt-get update

# 2. Install build essentials and libpcap
sudo apt-get install -y build-essential libpcap-dev pkg-config cmake

# 3. Install Eclipse SUMO and SUMO-GUI
sudo add-apt-repository -y ppa:sumo/stable
sudo apt-get update
sudo apt-get install -y sumo sumo-tools sumo-gui

# 4. Verify SUMO installation
sumo --version
```

### On macOS

```bash
# Install Homebrew dependencies
brew install sumo libpcap pkg-config
```

### Installing Rust via Rustup

If you do not have Rust installed:

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
source "$HOME/.cargo/env"
rustc --version
```

---

## 3. Cloning and Compiling

Clone the repository:

```bash
git clone https://github.com/Fundacio-i2CAT/RustV2XSimulator.git
cd RustV2XSimulator
```

Build the release binary:

```bash
cargo build --release
```

Run the built-in test suite to verify physical layer parity, DCC state machine conformance, and discrete-event scheduler causality:

```bash
cargo test --release
```

All integration tests should pass:

- `test_ns3_phy_parity_mae`: Verifies PDR vs. distance curve against ns-3 with $\text{MAE} \le 0.04$.
- `test_reactive_dcc_conformance_step_profile`: Verifies ETSI TS 102 687 Table A.1 state transitions.
- `test_causality_invariance`: Verifies monotonic discrete-event ordering under multi-threading.
