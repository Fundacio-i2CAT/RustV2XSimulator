# V2X Standards Modulation and Coding Schemes (MCS) & Bandwidth Analysis

This document provides a comprehensive technical reference for the Modulation and Coding Schemes (MCS), channel bandwidths, symbol timings, Forward Error Correction (FEC), and physical layer decider parameters across all four wireless standards supported by `rustv2xsimulator` and `aoi_benchmark`.

---

## 1. High-Level Standards Overview

| Standard | Channel Bandwidth(s) | Carrier Freq (Band) | Multiple Access / MAC | Forward Error Correction | Active Default in Simulator | Default Physical Bitrate |
|---|:---:|:---:|---|---|:---:|:---:|
| **IEEE 802.11p (DSRC / ITS-G5)** | 10 MHz | 5.86–5.92 GHz (Ch 172–184) | CSMA/CA (EDCA 4 ACs) | Punctured Convolutional ($K=7$) | **MCS 2** (QPSK 1/2) | **6.0 Mbps** |
| **IEEE 802.11bd (NGV)** | 10 MHz, 20 MHz | 5.890 GHz (ITS Band) | Enhanced EDCA | Quasi-Cyclic LDPC (QC-LDPC) | **MCS 2** (QPSK 3/4) | **9.0 Mbps** (10 MHz) / **18.0 Mbps** (20 MHz) |
| **3GPP C-V2X PC5 Mode 4** | 10 MHz, 20 MHz | 5.905 GHz (Band 47) | Sensing-Based SPS (Rel-14) | Turbo Coding | **MCS 7** (QPSK rate 0.58) | Transport Block (~1.16 bits/RE) |
| **3GPP 5G NR-V2X Mode 2** | 10, 20, 40 MHz | 5.890 GHz (Band n47) | Sidelink Mode 2 (Rel-16/17) | Polar (PSCCH) + NR-LDPC (PSSCH) | **MCS 10** (16-QAM, 340/1024) | Transport Block (1.3281 bits/RE) |

---

## 2. IEEE 802.11p (DSRC / ITS-G5)

Implemented in `src/phy/consts_80211p.rs` and `src/phy/decider_80211p.rs` with timing parameters from IEEE 802.11-2012 Table 17-14 and IEEE 1609.4-2010.

### 2.1 Physical Layer Parameters
- **Channel Bandwidth**: **10 MHz** (channel spacing)
- **Carrier Center Frequency**: Configurable via `DsrcChannel`:
  - Channel 172: Critical Safety (5.86 GHz)
  - Channel 174: Service Channel 1 (5.87 GHz)
  - Channel 176: Service Channel 2 (5.88 GHz)
  - Channel 178: Control Channel CCH / ETSI G5-CCH (5.89 GHz) - **Default**
  - Channel 180: Service Channel 3 (5.90 GHz)
  - Channel 182: Service Channel 4 (5.91 GHz)
  - Channel 184: High Power Public Safety (5.92 GHz)
- **Subcarriers**: 64 total (48 data subcarriers, 4 pilot subcarriers, 12 guard/null subcarriers)
- **Subcarrier Spacing ($\Delta f$)**: $156.25\,\text{kHz}$
- **OFDM Symbol Duration ($T_{\text{sym}}$)**: $8.0\,\mu\text{s}$ (including $1.6\,\mu\text{s}$ Guard Interval)
- **PLCP Preamble & Header**:
  - Preamble: $32.0\,\mu\text{s}$ (10 short + 2 long training symbols)
  - PLCP Signal field: $8.0\,\mu\text{s}$ (24 bits transmitted at 3.0 Mbps BPSK 1/2)
  - Total PHY Framing Overhead: $40.0\,\mu\text{s}$
- **Air Duration Equation**:

  $$T_{\text{air}}(L) = 40\,\mu\text{s} + \left\lceil \frac{16 + 8 \times L_{\text{bytes}} + 6}{N_{\text{DBPS}}} \right\rceil \times 8\,\mu\text{s}$$

  Implemented in `compute_duration_10mhz` and `compute_duration_80211p`.

### 2.2 Standard 802.11p MCS Table (8 Levels)

| MCS Enum (`Mcs80211p`) | Index | Modulation | Code Rate ($R$) | Puncturing $b$-value | Data Bits / Symbol ($N_{\text{DBPS}}$) | Bitrate (10 MHz) | ns-3 / NIST 50% Transition | Typical Operational Profile |
|---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|---|
| `BpskR1_2` | 0 | BPSK | 1/2 | 1 | 24 | **3.0 Mbps** | ~2.5 dB | PLCP Signal field / Long-range broadcast |
| `BpskR3_4` | 1 | BPSK | 3/4 | 3 | 36 | **4.5 Mbps** | ~4.0 dB | Robust emergency warning |
| `QpskR1_2` | 2 | QPSK | 1/2 | 1 | 48 | **6.0 Mbps** | ~5.5 dB | **Standard C-ITS Default (CAM / DENM)** |
| `QpskR3_4` | 3 | QPSK | 3/4 | 3 | 72 | **9.0 Mbps** | ~8.0 dB | Enhanced throughput safety broadcast |
| `Qam16R1_2` | 4 | 16-QAM | 1/2 | 1 | 96 | **12.0 Mbps** | ~11.0 dB | Collective Perception Messages (CPM) |
| `Qam16R3_4` | 5 | 16-QAM | 3/4 | 3 | 144 | **18.0 Mbps** | ~14.5 dB | Maneuver Coordination / Platooning |
| `Qam64R2_3` | 6 | 64-QAM | 2/3 | 2 | 192 | **24.0 Mbps** | ~18.0 dB | High-rate sensor telemetry |
| `Qam64R3_4` | 7 | 64-QAM | 3/4 | 3 | 216 | **27.0 Mbps** | ~20.0 dB | Maximum 802.11p peak throughput |

---

## 3. IEEE 802.11bd (Next Generation V2X / NGV)

Implemented in `src/phy/wifi80211bd/consts_80211bd.rs`, `config.rs`, and `decider_80211bd.rs` per IEEE Std 802.11bd-2022.

### 3.1 Physical Layer Parameters
- **Channel Bandwidths**:
  - **10 MHz** (standard vehicular ITS band)
  - **20 MHz** (channel bonding for high throughput)
- **Carrier Center Frequency**: $5.890\,\text{GHz}$ (default)
- **Symbol Duration ($T_{\text{sym}}$)**:
  - 10 MHz: $8.0\,\mu\text{s}$
  - 20 MHz: $4.0\,\mu\text{s}$ ($312.5\,\text{kHz}$ subcarrier spacing)
- **Preamble & Framing (Mixed-Format NGV PPDU)**:
  - 10 MHz: $72.0\,\mu\text{s}$ total (L-STF $16\mu\text{s}$, L-LTF $16\mu\text{s}$, L-SIG $8\mu\text{s}$, R-SIG $8\mu\text{s}$, NGV-SIG $8\mu\text{s}$, NGV-STF $8\mu\text{s}$, NGV-LTF $8\mu\text{s}$)
  - 20 MHz: $36.0\,\mu\text{s}$ total
  - **Backward Compatibility**: L-SIG duration spoofing (`compute_lsig_length_spoofed`) forces legacy 802.11p stations to defer channel access during the full 802.11bd transmission.
- **Midamble Periodicity (`MidamblePeriod`)**:
  - `None`: 0 midambles (vulnerable to high Doppler spread)
  - `Every4Symbols`: Maximum resilience for relative speeds $> 200\,\text{km/h}$
  - `Every8Symbols`: **Default** balance of tracking and overhead
  - `Every16Symbols`: Low overhead for moderate speeds
- **Coding**: Quasi-Cyclic Low-Density Parity-Check (QC-LDPC), providing $+2.5$ to $+3.0\,\text{dB}$ SNR advantage over convolutional codes.
- **Air Duration Equation**:

  $$T_{\text{air}} = T_{\text{preamble}} + (N_{\text{data\_sym}} + N_{\text{midamble\_sym}}) \times T_{\text{sym}}$$

  Implemented in `compute_duration_80211bd`.

### 3.2 Standard 802.11bd MCS Table (10 Levels + DCM)

| MCS Enum (`Mcs80211bd`) | Index | Modulation | Code Rate ($R$) | Constellation Order ($Q_m$) | $N_{\text{DBPS}}$ (10 MHz) | Bitrate (10 MHz) | Bitrate (20 MHz) | 1% PER Threshold | Primary Use Case |
|---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|---|
| `Mcs0Bpsk1_2 (DCM)` | 0 | BPSK (DCM) | 1/2 | 1 | 12 | **1.5 Mbps** | **3.0 Mbps** | **-0.5 dB** | Ultra-reliable emergency beaconing / deep NLOS |
| `Mcs0Bpsk1_2` | 0 | BPSK | 1/2 | 1 | 24 | **3.0 Mbps** | **6.0 Mbps** | **+1.5 dB** | Extended-range warning messages |
| `Mcs1Qpsk1_2` | 1 | QPSK | 1/2 | 2 | 48 | **6.0 Mbps** | **12.0 Mbps** | **+4.0 dB** | Standard CAM / DENM broadcast |
| `Mcs2Qpsk3_4` | 2 | QPSK | 3/4 | 2 | 72 | **9.0 Mbps** | **18.0 Mbps** | **+6.8 dB** | **Recommended Simulator Default** |
| `Mcs3Qam16_1_2` | 3 | 16-QAM | 1/2 | 4 | 96 | **12.0 Mbps** | **24.0 Mbps** | **+10.2 dB** | Collective Perception Messages (CPM) |
| `Mcs4Qam16_3_4` | 4 | 16-QAM | 3/4 | 4 | 144 | **18.0 Mbps** | **36.0 Mbps** | **+13.8 dB** | Maneuver Coordination (MCM) / Platooning |
| `Mcs5Qam64_2_3` | 5 | 64-QAM | 2/3 | 6 | 192 | **24.0 Mbps** | **48.0 Mbps** | **+17.5 dB** | High-density urban sensor sharing |
| `Mcs6Qam64_3_4` | 6 | 64-QAM | 3/4 | 6 | 216 | **27.0 Mbps** | **54.0 Mbps** | **+19.2 dB** | High-throughput platoon telemetry |
| `Mcs7Qam64_5_6` | 7 | 64-QAM | 5/6 | 6 | 240 | **30.0 Mbps** | **60.0 Mbps** | **+20.8 dB** | Raw camera/radar point cloud streams |
| `Mcs8Qam256_3_4` | 8 | 256-QAM | 3/4 | 8 | 288 | **36.0 Mbps** | **72.0 Mbps** | **+24.5 dB** | Ultra-high throughput LOS sensor fusion |
| `Mcs9Qam256_5_6` | 9 | 256-QAM | 5/6 | 8 | 320 | **40.0 Mbps** | **80.0 Mbps** | **+26.2 dB** | Peak throughput (point-to-point file transfer) |

---

## 4. 3GPP C-V2X PC5 Mode 4 (LTE-V2X Rel-14/15)

Implemented in `src/phy/cv2x/config.rs`, `src/phy/cv2x/phy_bler.rs`, and `src/phy/cv2x/sb_sps.rs` per 3GPP TS 36.213 and TR 36.885.

### 4.1 Resource Grid & MAC Configuration
- **Bandwidth Options**:
  - **10 MHz**: 50 Resource Blocks (RBs) $\to$ **5 sub-channels** of 10 RBs each
  - **20 MHz**: 100 Resource Blocks (RBs) $\to$ **10 sub-channels** of 10 RBs each
- **Sub-channel Granularity**: 10 RBs ($1.8\,\text{MHz}$)
- **Carrier Center Frequency**: $5.905\,\text{GHz}$ (3GPP Band 47)
- **Time Transmission Interval (TTI)**: 1.0 ms subframe (14 SC-FDMA symbols)
- **Sensing-Based Semi-Persistent Scheduling (SB-SPS)**:
  - Sensing Window: $1000\,\text{ms}$ (1,000 subframes)
  - Selection Window: $[t + T_1, t + T_2] = [t + 4\,\text{ms}, t + 100\,\text{ms}]$
  - Reservation Period ($P_{\text{rsvp}}$): $100\,\text{ms}$ (10 Hz periodic beaconing)
  - Reselection Counter ($C_{\text{resel}}$): Uniform random $\mathcal{U}[5, 15]$
  - Probability of Keep ($P_{\text{keep}}$): $0.4$ (configurable $0.2 \dots 0.8$)
  - Candidate Exclusion Threshold: $\text{Th}_{\text{RSRP}} = -110.0\,\text{dBm}$ (backs off by $+3\,\text{dB}$ if $< 20\%$ remain)

### 4.2 Multiplexed Physical Channels & Dual Decoding
1. **PSCCH (Physical Sidelink Control Channel)**:
   - Occupies 2 RBs.
   - Carries Sidelink Control Information (**SCI Format 1**): Priority (PPPP), $P_{\text{rsvp}}$, frequency allocation, and MCS.
   - Demodulated with QPSK convolutional coding; sigmoidal error transition centered at **$\text{SINR} \approx -2.5\,\text{dB}$**.
2. **PSSCH (Physical Sidelink Shared Channel)**:
   - Occupies remaining 8 RBs in the allocated subchannel.
   - Carries the Transport Block (CAM / DENM).
   - Demodulated with Turbo coding across **MCS 0 to MCS 28**.
3. **Dual Decoding Condition**: Reception succeeds if and only if **both PSCCH (SCI) and PSSCH (TB) decode error-free**.

### 4.3 C-V2X Sidelink MCS Table (3GPP Rel-14 / TR 36.885)

| MCS Index | Modulation | Code Rate ($R$) | Approx. Spectral Eff. | 50% BLER Center SINR | Operational Role in V2X |
|:---:|:---:|:---:|:---:|:---:|---|
| **0** | QPSK | ~0.15 | ~0.30 bits/RE | $-4.9\text{ dB}$ | Extreme coverage / deep shadowing fallback |
| **1–3** | QPSK | 0.20 – 0.35 | 0.40 – 0.70 bits/RE | $-3.7\text{ dB}$ to $-1.3\text{ dB}$ | Extended-range warning messages |
| **4–6** | QPSK | 0.40 – 0.52 | 0.80 – 1.04 bits/RE | $-0.1\text{ dB}$ to $+2.3\text{ dB}$ | Dense urban vehicular beaconing |
| **7** | **QPSK** | **~0.58** | **~1.16 bits/RE** | **$+3.5\text{ dB}$** | **Standard 3GPP Rel-14 Default (`pssch_mcs = 7`)** |
| **8–10** | QPSK | 0.65 – 0.80 | 1.30 – 1.60 bits/RE | $+4.7\text{ dB}$ to $+7.1\text{ dB}$ | Peak QPSK transport blocks |
| **11–15** | 16-QAM | 0.37 – 0.55 | 1.48 – 2.20 bits/RE | $+8.3\text{ dB}$ to $+13.1\text{ dB}$ | Collective Perception Messages (CPM) |
| **16–20** | 16-QAM | 0.60 – 0.82 | 2.40 – 3.28 bits/RE | $+14.3\text{ dB}$ to $+19.1\text{ dB}$ | Platooning coordination |
| **21–28** | 64-QAM | 0.56 – 0.92 | 3.36 – 5.52 bits/RE | $+20.3\text{ dB}$ to $+28.7\text{ dB}$ | High-throughput localized sensor sharing |

*Center SINR is parameterized as $\gamma_{th} = 3.5 + (\text{mcs} - 7) \times 1.2\,\text{dB}$ in `SidelinkPhyModel::pssch_tb_bler`.*

---

## 5. 3GPP 5G NR-V2X Sidelink Mode 2 (Rel-16/17)

Implemented in `src/phy/nrv2x/consts_nrv2x.rs`, `config.rs`, `model.rs`, and `ldpc_bler.rs` per 3GPP TS 38.211, TS 38.213 Sec 16, and TS 38.214 Table 5.1.3.1-1.

### 5.1 Flexible Numerologies (`NrNumerology`)

| Numerology ($\mu$) | Subcarrier Spacing ($\Delta f$) | Slot Duration | Symbols / Slot | Slots / 1 ms Subframe | Operational Profile |
|:---:|:---:|:---:|:---:|:---:|---|
| **$\mu = 0$** | $15\,\text{kHz}$ | $1.0\,\text{ms}$ | 14 | 1 | Wide macro coverage / rural |
| **$\mu = 1$** | $30\,\text{kHz}$ | $0.5\,\text{ms}$ ($500\,\mu\text{s}$) | 14 | 2 | **Standard Default for Band n47 @ 5.890 GHz** |
| **$\mu = 2$** | $60\,\text{kHz}$ | $0.25\,\text{ms}$ ($250\,\mu\text{s}$) | 14 | 4 | High-speed highway & URLLC platooning |

### 5.2 Physical Layer Channels & Mode 2 Enhancements
- **Bandwidth**: 10 MHz, 20 MHz (**Default**), or 40 MHz.
- **Sub-channel Configuration**: $10, 12, 15, 20, 25$ RBs per sub-channel (default: 10 RBs, 5 active sub-channels per slot).
- **Two-Stage Sidelink Control Information (2-Stage SCI)**:
  - **Stage 1 (SCI Format 1-A)**: Carried over PSCCH (2 or 3 symbols); Polar coded with CRC-24C; signals priority (3 bits), frequency RIV, time allocation, $P_{\text{rsvp}}$, and MCS. Demodulation threshold $\approx -4.0\,\text{dB}$ SINR.
  - **Stage 2 (SCI Format 2-A)**: Multiplexed in PSSCH; Polar coded; carries Source/Destination L2 IDs, HARQ process ID, NDI, and Cast Type.
- **PSSCH Data**: Transport block encoded via **5G NR LDPC Base Graph 1/2**.
- **Release 16 Re-evaluation**: Lookahead sensing check at slot $m - T_3$ ($T_3 = 2$ slots) to drop conflicting grants before transmission.
- **Release 16 Pre-emption**: Priority overrides allow safety packets to pre-empt lower-priority reservations.
- **PSFCH HARQ**: Closed-loop ACK/NACK feedback channel with up to 3 retransmissions cycling through RV $0 \to 2 \to 3 \to 1$.

### 5.3 Full 3GPP TS 38.214 Table 5.1.3.1-1 (MCS 0 to 28)

Implemented in `NR_MCS_TABLE_1` with calibrated 10% BLER thresholds against UWICORE / 5G-LENA under TDL-B / TDL-D vehicular fading profiles:

| MCS Index | Modulation | Code Rate ($R \times 1024$) | Effective Code Rate ($R$) | Spectral Efficiency (bits/RE) | 10% BLER Threshold ($\gamma_{th}$) | Operational Profile |
|:---:|:---:|:---:|:---:|:---:|:---:|---|
| **0** | QPSK | 120 | 0.117 | 0.2344 | **-1.8 dB** | Deep NLOS / Extended radio horizon |
| **1** | QPSK | 157 | 0.153 | 0.3066 | -0.6 dB | Long-range safety warning |
| **2** | QPSK | 193 | 0.188 | 0.3770 | +0.7 dB | High-reliability broadcast |
| **3** | QPSK | 251 | 0.245 | 0.4902 | +1.9 dB | Robust CAM transmission |
| **4** | **QPSK** | **308** | **0.301** | **0.6016** | **+3.2 dB** | **Preset: `urllc_platooning`** |
| **5** | QPSK | 379 | 0.370 | 0.7402 | +4.6 dB | Moderate-density safety traffic |
| **6** | QPSK | 449 | 0.438 | 0.8770 | +6.0 dB | Standard safety beaconing |
| **7** | QPSK | 526 | 0.514 | 1.0273 | +7.3 dB | Standard CAM exchange |
| **8** | QPSK | 602 | 0.588 | 1.1758 | +8.6 dB | High-rate QPSK beaconing |
| **9** | QPSK | 679 | 0.663 | 1.3262 | +9.9 dB | Peak QPSK throughput |
| **10** | **16-QAM** | **340** | **0.332** | **1.3281** | **+11.2 dB** | **Simulator Default (`pssch_mcs = 10`)** |
| **11** | 16-QAM | 378 | 0.369 | 1.4766 | +12.4 dB | Balanced CAM / DENM |
| **12** | 16-QAM | 434 | 0.424 | 1.6953 | +13.6 dB | Maneuver Coordination (MCM) |
| **13** | 16-QAM | 490 | 0.479 | 1.9141 | +14.8 dB | Collective Perception Messages (CPM) |
| **14** | 16-QAM | 553 | 0.540 | 2.1602 | +16.0 dB | Sensor sharing |
| **15** | 16-QAM | 616 | 0.602 | 2.4063 | +17.3 dB | Sensor sharing |
| **16** | 16-QAM | 658 | 0.643 | 2.5703 | +18.5 dB | Peak 16-QAM throughput |
| **17** | 64-QAM | 438 | 0.428 | 2.5664 | +19.6 dB | High-rate cooperative perception |
| **18** | 64-QAM | 466 | 0.455 | 2.7305 | +20.8 dB | Dense cooperative perception |
| **19** | 64-QAM | 517 | 0.505 | 3.0293 | +21.9 dB | Video streaming / Sensor fusion |
| **20** | 64-QAM | 567 | 0.554 | 3.3223 | +23.1 dB | Radar point-cloud sharing |
| **21** | 64-QAM | 616 | 0.602 | 3.6094 | +24.2 dB | Radar point-cloud sharing |
| **22** | 64-QAM | 666 | 0.650 | 3.9023 | +25.4 dB | Raw camera feed |
| **23** | 64-QAM | 719 | 0.702 | 4.2129 | +26.5 dB | High-throughput platoon telemetry |
| **24** | 64-QAM | 772 | 0.754 | 4.5234 | +27.7 dB | High-throughput platoon telemetry |
| **25** | 64-QAM | 822 | 0.803 | 4.8164 | +28.8 dB | Inter-vehicle file transfer |
| **26** | 64-QAM | 873 | 0.853 | 5.1152 | +29.9 dB | Inter-vehicle file transfer |
| **27** | 64-QAM | 910 | 0.889 | 5.3320 | +31.0 dB | Very high SNR near-field |
| **28** | 64-QAM | 948 | 0.926 | 5.5547 | **+32.0 dB** | Peak 5G NR Sidelink throughput |

---

## 6. Summary of Simulator Presets and Code Locations

| Technology | CLI Flag | Config Struct | Default MCS | Primary Code References |
|---|---|---|---|---|
| **IEEE 802.11p** | `--technology 11p` | `Dsrc80211pModel` | `Mcs80211p::QpskR1_2` (MCS 2, 6 Mbps) | `consts_80211p.rs`, `edca.rs`, `decider_80211p.rs` |
| **IEEE 802.11bd** | `--technology 11bd` | `Wifi80211bdConfig` | `Mcs80211bd::Mcs2Qpsk3_4` (MCS 2, 9 Mbps) | `consts_80211bd.rs`, `model.rs`, `decider_80211bd.rs` |
| **C-V2X Mode 4** | `--technology cv2x` | `Cv2xConfig` | `pssch_mcs = 7` (QPSK rate 0.58) | `config.rs`, `sb_sps.rs`, `phy_bler.rs` |
| **NR-V2X Mode 2** | `--technology nrv2x` | `NrV2xConfig` | `pssch_mcs = 10` (16-QAM rate 340/1024) | `consts_nrv2x.rs`, `model.rs`, `ldpc_bler.rs` |
