# TOML Configuration Reference

Simulation scenarios can be fully configured using declarative `.toml` files located in `config/`.

This document details all configuration tables and fields.

---

## 1. Example Configuration

```toml
[scenario]
name = "diagonal"
net_file = "scenarios/diagonal/osm_v2.net.xml"
poly_file = "scenarios/diagonal/osm.poly.xml"

[simulation]
duration = 60.0
time_step = 0.1
seed = 42
workload = "cam" # "cam", "eebl", "cam_static_10hz"

[mobility]
mode = "sumo" # "sumo", "highway", "platoon", "urban"
nodes = 15

[phy]
model = "dualslope" # "dualslope" (802.11p), "wifi80211bd", "cv2x", "nr_v2x", "legacy_poly"
carrier_frequency_hz = 5.9e9
bandwidth_hz = 10.0e6
tx_power_dbm = 23.0
noise_floor_dbm = -99.0
d_break = 120.0
gamma1 = 2.05
gamma2 = 3.90
sigma_los = 3.1
sigma_nlos = 8.3
capture_threshold_db = 4.0
building_attenuation_db_per_m = 0.85
max_building_attenuation_db = 45.0

[mac]
mode = "edca" # "edca" or "aloha"
cca_threshold_dbm = -85.0
slot_time_us = 13.0
sifs_us = 32.0
cbr_window_s = 0.100

[dcc]
mode = "reactive" # "off", "reactive", "limeric"
target_cbr = 0.68
alpha = 0.016
beta = 0.0012
min_rate_hz = 2.0
max_rate_hz = 10.0

[actuation]
mode = "krauss_substep" # "krauss_substep" or "quantized"
max_decel = 6.0
reaction_time_s = 0.1
min_gap_m = 2.5

[metrics]
output_dir = "results/diagonal/"
export_csv = true
export_json = true
enable_pcap = true
```

---

## 2. Table-by-Table Reference

### `[scenario]`
| Key | Type | Default | Description |
| :--- | :---: | :---: | :--- |
| `name` | String | `"default"` | Scenario name identifier |
| `net_file` | String | `""` | Path to SUMO road network XML (`.net.xml`) |
| `poly_file` | String | `""` | Path to building polygon XML (`.poly.xml`) used for obstacle attenuation and knife-edge diffraction |

### `[simulation]`
| Key | Type | Default | Description |
| :--- | :---: | :---: | :--- |
| `duration` | Float | `30.0` | Total simulation duration in seconds |
| `time_step` | Float | `0.1` | Mobility stepping resolution in seconds |
| `seed` | Integer | `42` | PRNG seed for deterministic reproducibility |
| `workload` | String | `"cam"` | Traffic workload: `"cam"`, `"eebl"`, or `"cam_static_10hz"` |

### `[phy]`
| Key | Type | Default | Description |
| :--- | :---: | :---: | :--- |
| `model` | String | `"dualslope"` | PHY model: `"dualslope"` (802.11p DSRC), `"wifi80211bd"`, `"cv2x"`, `"nr_v2x"`, or `"legacy_poly"` (Parametric) |
| `carrier_frequency_hz` | Float | `5.9e9` | Center carrier frequency (Hz) |
| `bandwidth_hz` | Float | `10.0e6` | Channel bandwidth (Hz) |
| `tx_power_dbm` | Float | `23.0` | Transmit power ($23\,\text{dBm} = 200\,\text{mW}$) |
| `noise_floor_dbm` | Float | `-99.0` | Thermal noise floor ($N_0$) |
| `d_break` | Float | `120.0` | Breakpoint distance in meters |
| `gamma1` | Float | `2.05` | Path loss exponent before breakpoint |
| `gamma2` | Float | `3.90` | Path loss exponent after breakpoint |
| `sigma_los` | Float | `3.1` | Log-normal shadow fading standard deviation for LoS (dB) |
| `sigma_nlos` | Float | `8.3` | Log-normal shadow fading standard deviation for NLoS (dB) |
| `capture_threshold_db` | Float | `4.0` | Receiver capture threshold ($\gamma_{cap}$ in dB) |
| `building_attenuation_db_per_m`| Float | `0.85` | Concrete wall penetration attenuation (dB/meter) |
| `max_building_attenuation_db` | Float | `45.0` | Maximum concrete penetration attenuation cap (dB) |

### `[mac]`
| Key | Type | Default | Description |
| :--- | :---: | :---: | :--- |
| `mode` | String | `"edca"` | MAC protocol: `"edca"` (IEEE 802.11p) or `"aloha"` (unslotted) |
| `cca_threshold_dbm` | Float | `-85.0` | Clear Channel Assessment (CCA) threshold in dBm |
| `slot_time_us` | Float | `13.0` | Slot duration ($\sigma$) in microseconds |
| `sifs_us` | Float | `32.0` | Short Inter-Frame Space duration in microseconds |
| `cbr_window_s` | Float | `0.100` | Sliding window for Channel Busy Ratio calculation (100 ms) |

### `[dcc]`
| Key | Type | Default | Description |
| :--- | :---: | :---: | :--- |
| `mode` | String | `"reactive"`| DCC profile: `"off"`, `"reactive"` (ETSI TS 102 687 FSM), or `"limeric"` |
| `target_cbr` | Float | `0.68` | Target channel busy ratio for LIMERIC ($68\%$) |
| `alpha` | Float | `0.016` | LIMERIC adaptation rate parameter $\alpha$ |
| `beta` | Float | `0.0012` | LIMERIC gain parameter $\beta$ |
| `min_rate_hz` | Float | `2.0` | Minimum message generation rate under heavy congestion ($2\,\text{Hz} = 500\,\text{ms}$) |
| `max_rate_hz` | Float | `10.0` | Maximum message generation rate under light channel load ($10\,\text{Hz} = 100\,\text{ms}$) |
