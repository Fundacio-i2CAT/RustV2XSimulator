# Congestion, Simultaneous Transmissions, and Discrete-Event Scheduling in RustV2XSimulator

## Executive Summary

This document provides a comprehensive, mathematically formalized analysis of how **congestion-related losses**, **simultaneous transmissions (packet collisions)**, and **discrete-event scheduling (DES)** operate within **`rustv2xsimulator`**.

Specifically, this treatise addresses:

1. **The Mechanisms of Congestion-Related Losses**: Cross-layer dynamics from Decentralized Congestion Control (DCC) and MAC contention down to Physical Layer SINR degradation.
2. **Simultaneous Transmissions by Nearby ITS Stations**: How simultaneous channel access, interference superposition, chunked SINR tracking, and receiver capture ($\gamma_{\mathrm{cap}} = 4.0\,\mathrm{dB}$) are simulated.
3. **Discrete-Event Simulation (DES) Event Lifecycle**: The exact sequence of events (`MobilityStep`, `MacTxStart`, `PhyRxDelivery`) scheduled in the $O(1)$ Calendar Queue.
4. **Transmission Delay vs. Propagation Delay**: How packet air duration ($T_{\mathrm{trans}}$) and propagation delay ($\tau_{\mathrm{prop}}$) are evaluated across IEEE 802.11p, IEEE 802.11bd, C-V2X Mode 4, and 5G NR-V2X Mode 2.

---

## 1. Mechanisms of Congestion-Related Packet Losses

In vehicular ad-hoc networks (VANETs), congestion-induced packet loss is not a single phenomenon; it occurs across three distinct protocol layers:

```
+----------------------------------------------------------------------------------+
| 1. FACILITIES & DCC (ETSI TS 102 687 / TS 103 900)                                |
|    - Channel Busy Ratio (CBR) monitoring                                         |
|    - Reactive 7-state FSM throttles T_off from 60 ms to 460 ms                   |
|    - Dual-regime LIMERIC dynamically limits CAM generation rate to CBR* = 0.68   |
+----------------------------------------------------------------------------------+
                                        |
+----------------------------------------------------------------------------------+
| 2. MAC LAYER (IEEE 802.11p EDCA / CSMA/CA / 3GPP Mode 2 Sidelink)                |
|    - Carrier Sense Multiple Access: CCA energy threshold -85.0 dBm              |
|    - Contention Window backoff countdown: slots freeze when channel is busy       |
|    - Uncoordinated broadcast (NO ACKs, NO RTS/CTS) leads to simultaneous air slot |
+----------------------------------------------------------------------------------+
                                        |
+----------------------------------------------------------------------------------+
| 3. PHYSICAL MEDIUM & SINR CHUNKING (RadioMedium / InterferenceHelper)            |
|    - Multiple overlapping signals sum linearly in linear Watts                   |
|    - SINR = P_rx,desired / (I_chunk + N_0) drops                                  |
|    - Destructive collision if SIR < 4.0 dB; else PER curve decides survival      |
+----------------------------------------------------------------------------------+
```

### 1.1 Facility & Congestion Control Throttling
Before a packet even reaches the wireless medium, the **Decentralized Congestion Control (DCC)** subsystem monitors the **Channel Busy Ratio (CBR)**:

$$CBR = \frac{T_{\mathrm{busy}}}{T_{\mathrm{window}}}$$

where $T_{\mathrm{window}} = 100\,\mathrm{ms}$.

- Implemented in [`ChannelMonitor`](../../src/mac/channel_monitor.rs) and [`ReactiveDccFsm`](../../src/dcc/reactive_fsm.rs).
- When node density increases, $CBR$ crosses standard threshold gates ($CBR > 0.3, 0.4, 0.5, \dots$).
- Under severe congestion, CAM generation is suppressed (intervals expand from $100\,\mathrm{ms}$ up to $1000\,\mathrm{ms}$), resulting in **staleness / Age of Information (AoI) degradation** rather than dropped physical packets.

### 1.2 MAC Layer Contention & The Broadcast Vulnerability
In vehicular safety communications (CAM and DENM), transmissions are **unacknowledged single-hop broadcasts (SHB)**:

- **No RTS/CTS Exchange**: Hidden terminals cannot be mitigated with reservation handshakes.
- **No Positive Acknowledgements (ACKs)**: Transmitters cannot detect whether their broadcast collided on air.
- If two vehicles draw the same random backoff slot $k \in [0, CW_{\mathrm{min}}]$ or are out of carrier-sense range of one another (hidden terminals), both stations will transition to the transmitting state at the same instant in time.

---

## 2. Simultaneous Transmissions by Nearby ITS Stations

When two vehicles (Station $A$ and Station $B$) transmit overlapping packets in the vicinity of a receiving station $C$, the simulator evaluates the reception using **Piecewise-Constant SINR Chunking** and **Receiver Capture Effects**.

Implemented in [`CumulativeInterferenceTracker::evaluate_reception`](../../src/phy/interference_helper.rs), mirroring the gold-standard architecture of **ns-3 v3.42 `InterferenceHelper`**.

```
Time Axis --->

Station A (Desired):  [====================================================]
                      t_start                                              t_end

Station B (Interf 1):           [==============================]
                                t_B_start                      t_B_end

Station D (Interf 2):                                    [========================]
                                                         t_D_start                t_D_end

Slices / Chunks:      | Chunk 1 |           Chunk 2            |  Chunk 3  | Chunk 4  |
Interference:          I = 0     I = P_B                        I = P_B+P_D I = P_D
```

### 2.1 Temporal Chunk Slicing Algorithm
Let the desired packet from Station $A$ have active reception window $[t_{\mathrm{start}}, t_{\mathrm{end}}]$ with received signal power $P_A$ (in Watts).

1. **Boundary Collection**:
   The engine collects all timestamps within $[t_{\mathrm{start}}, t_{\mathrm{end}}]$ where any concurrent interfering signal begins or ends:

   $$\mathcal{T} = \{t_{\mathrm{start}}, t_{\mathrm{end}}\} \cup \{t_{\mathrm{start}, j}, t_{\mathrm{end}, j} \mid [t_{\mathrm{start}, j}, t_{\mathrm{end}, j}] \cap [t_{\mathrm{start}}, t_{\mathrm{end}}] \neq \emptyset\}$$

2. **Chunk Division**:
   Sorting and deduplicating $\mathcal{T}$ yields $M$ non-overlapping sub-intervals $[t_k, t_{k+1}]$.
3. **Chunk Interference Summation**:
   In each sub-interval $k$, the total interfering power $I_k$ is the sum in linear Watts of all signals active at the chunk midpoint $t_{\mathrm{mid}} = (t_k + t_{k+1})/2$:

   $$I_k = \sum_{j \in \text{interferers active at } t_{\mathrm{mid}}} P_j$$

4. **Chunked SINR Evaluation**:

   $$\mathrm{SINR}_k = \frac{P_A}{I_k + N_0}$$

   where thermal noise floor $N_0 = -99.0\,\mathrm{dBm} \approx 1.2589 \times 10^{-13}\,\mathrm{W}$.

### 2.2 Receiver Capture Effect ($\gamma_{\mathrm{cap}} = 4.0\,\mathrm{dB}$)
In physical wireless receivers, when an interfering packet collides with an ongoing reception, the receiver's phase-locked loop (PLL) and automatic gain control (AGC) may remain locked to the desired signal if the desired signal dominates the interference:

- **Signal-to-Interference Ratio**:

  $$\mathrm{SIR}_k = 10 \log_{10}\left( \frac{P_A}{I_k} \right)$$

- **Destructive Collision**:
  If $\mathrm{SIR}_k < 4.0\,\mathrm{dB}$ and $\mathrm{SINR}_k < 6.0\,\mathrm{dB}$, capture fails completely. The receiver loses synchronization:

  $$\mathrm{PSR}_k = 0.0 \implies \mathrm{Overall\ PSR} = 0.0$$

  The packet is immediately marked as collided and lost.

- **Successful Capture**:
  If Station $A$ is substantially closer to the receiver than Station $B$ such that $\mathrm{SIR}_k \ge 4.0\,\mathrm{dB}$, the receiver successfully captures Station $A$'s frame despite Station $B$'s simultaneous transmission.

### 2.3 Analytical Packet Error Rate (PER) Decider
If capture succeeds across all chunks, each chunk's success rate is evaluated using the IEEE 802.11p analytical PER curve (NIST error rate model for QPSK 1/2 with convolutional coding):

$$\mathrm{PSR}_{\mathrm{overall}} = \prod_{k=1}^M \left( 1 - \mathrm{PER}_k(\mathrm{SINR}_k) \right)$$

A uniform random draw $u \sim \mathcal{U}(0, 1)$ decides whether the packet is decoded error-free ($u < \mathrm{PSR}_{\mathrm{overall}}$).

---

## 3. How Sending and Receiving Events are Scheduled in DES

The simulation engine coordinates events via an $O(1)$ amortized **Calendar Queue** discrete-event scheduler ([`DiscreteEventScheduler`](../../src/scheduler/scheduler.rs)).

### 3.1 Strict Event Ordering
Every event possesses three ordering criteria:

1. `timestamp: SimTime`: Simulation clock represented in **100-nanosecond integer ticks** (preventing floating-point drift).
2. `priority: u8`: Integer priority tie-breaker ($0 = \text{highest}$, $255 = \text{lowest}$).
   - **Priority 0**: `MacTxStart` (Medium transmission starts)
   - **Priority 1**: `MobilityStep` (Vehicle kinematics updates)
   - **Priority 2**: `PhyRxDelivery` (Packet deliveries at receivers)
3. `sequence: u64`: Monotonic sequential counter guaranteeing deterministic FIFO execution for co-temporal events.

### 3.2 End-to-End Event Lifecycle

```
[Simulation Clock: t_now]
       |
       v
1. MobilityStep (Priority 1)
   - TraCI or synthetic kinematics updates vehicle positions.
   - Triggers CAM generation checks (|Delta theta| >= 4 deg, Delta d >= 4 m).
   - Facilities generate payload bytes and pass to GeoNetworking router.
       |
       v
2. Outbound Enqueue & EDCA Contention
   - Node calls tech.enqueue_outbound(frame, ac, t_now).
   - Contention delay is computed:
       Delta t_backoff = AIFS[ac] + rand(0, CW_min) * T_slot
   - Schedules EventPayload::MacTxStart at:
       t_tx = t_now + Delta t_backoff   (Priority 0)
       |
       v
[Simulation Clock Advances to: t_tx]
       |
       v
3. MacTxStart Handler (Priority 0)
   - Computes packet transmission air time: duration = compute_duration_80211p(L, mcs).
   - Registers AirPacket in RadioMedium as active on air for [t_tx, t_tx + duration].
   - Queries spatial k-d tree for all candidate receivers within 800m.
   - Evaluates path loss, building polygons, and overlapping active signals.
   - For every candidate receiver, schedules EventPayload::PhyRxDelivery at:
       t_delivery = t_tx + delivery_delay   (Priority 2)
       |
       v
[Simulation Clock Advances to: t_delivery]
       |
       v
4. PhyRxDelivery Handler (Priority 2)
   - If is_success == true:
       Passes payload to receiver's BTP/GeoNetworking stack.
       Updates receiver's Local Dynamic Map (LDM).
   - If is_success == false:
       Discards packet, records telemetry drop metrics.
```

---

## 4. Transmission Delay vs. Propagation Delay

A critical question in simulation modeling is whether **transmission delay** and **propagation delay** are explicitly accounted for:

### 4.1 Transmission Delay ($T_{\mathrm{trans}}$): Microsecond-Level Modeling
**Yes, transmission delay is strictly and rigorously modeled based on PHY standard specifications.**

For **IEEE 802.11p** (10 MHz channel, implemented in [`compute_duration_80211p`](../../src/phy/consts_80211p.rs#L195)):

$$T_{\mathrm{trans}} = T_{\mathrm{preamble}} + T_{\mathrm{signal}} + \left\lceil \frac{N_{\mathrm{service}} + 8 L + N_{\mathrm{tail}}}{N_{\mathrm{DBPS}}} \right\rceil \times T_{\mathrm{sym}}$$

where:

- $T_{\mathrm{preamble}} = 32\,\mu\mathrm{s}$ (PLCP preamble: short and long training symbols).
- $T_{\mathrm{signal}} = 8\,\mu\mathrm{s}$ (PLCP signal field: Rate and Length, BPSK 1/2).
- $N_{\mathrm{service}} = 16\,\text{bits}$, $N_{\mathrm{tail}} = 6\,\text{bits}$.
- $T_{\mathrm{sym}} = 8\,\mu\mathrm{s}$ (OFDM symbol interval in 10 MHz channel).
- $N_{\mathrm{DBPS}}$: Data bits per OFDM symbol (e.g. $48\,\text{bits}$ for 6 Mbps QPSK 1/2).

**Concrete Example (300-byte CAM)**:

$$N_{\mathrm{bits}} = 16 + (300 \times 8) + 6 = 2422\,\text{bits}$$

$$N_{\mathrm{sym}} = \left\lceil \frac{2422}{48} \right\rceil = 51\,\text{symbols}$$

$$T_{\mathrm{trans}} = 32\,\mu\mathrm{s} + 8\,\mu\mathrm{s} + (51 \times 8\,\mu\mathrm{s}) = 448\,\mu\mathrm{s}$$

For other technologies:

- **IEEE 802.11bd**: Explicitly accounts for periodic midambles ($T_{\mathrm{mid}} = 8\,\mu\mathrm{s}$ every 8 symbols) and 2x DCM repetition.
- **3GPP C-V2X PC5 Mode 4**: Slot duration is exactly $1.0\,\mathrm{ms}$ ($1000\,\mu\mathrm{s}$).
- **3GPP 5G NR-V2X Mode 2**: Slot duration is exactly $0.5\,\mathrm{ms}$ ($500\,\mu\mathrm{s}$ for $\mu=1$, 30 kHz SCS).

During this entire time window $[t_{\mathrm{tx}}, t_{\mathrm{tx}} + T_{\mathrm{trans}}]$, the channel is occupied and causes interference against any concurrent transmission.

### 4.2 Propagation Delay ($\tau_{\mathrm{prop}}$): Physical Values vs. Implementation
In free space, electromagnetic waves travel at the speed of light $c \approx 3 \times 10^8\,\mathrm{m/s}$:

$$\tau_{\mathrm{prop}} = \frac{d}{c}$$

Over vehicular communication distances:

- $d = 30\,\mathrm{m} \implies \tau_{\mathrm{prop}} = 0.10\,\mu\mathrm{s}$ ($100\,\text{ns}$, exactly 1 SimTime tick).
- $d = 150\,\mathrm{m} \implies \tau_{\mathrm{prop}} = 0.50\,\mu\mathrm{s}$ ($500\,\text{ns}$).
- $d = 300\,\mathrm{m} \implies \tau_{\mathrm{prop}} = 1.00\,\mu\mathrm{s}$.
- $d = 800\,\mathrm{m} \implies \tau_{\mathrm{prop}} = 2.67\,\mu\mathrm{s}$.

#### How `rustv2xsimulator` Evaluates Delivery Timing:
In [`RadioMedium::evaluate_broadcast`](../../src/phy/radio_medium.rs#L141):
```rust
let delivery_delay = packet.duration + SimTime::from_micros(10);
```
- The simulator adds a fixed **$10\,\mu\mathrm{s}$ guard/turnaround offset** to the packet's air transmission duration.
- This $10\,\mu\mathrm{s}$ offset represents:
  1. Physical propagation time ($0.1\,\mu\mathrm{s} - 2.7\,\mu\mathrm{s}$).
  2. Receiver RF front-end AGC settling and preamble synchronization.
  3. MAC layer processing and CRC verification turnaround.
- Notice that because $T_{\mathrm{trans}} \approx 448\,\mu\mathrm{s}$, the transmission duration is **168× to 4480× larger** than the physical propagation delay $\tau_{\mathrm{prop}}$. Adding the calibrated $10\,\mu\mathrm{s}$ turnaround avoids hundreds of thousands of floating-point divisions while maintaining realistic reception ordering in the discrete-event scheduler.
- Under the ablation study parametric model (`LegacyPoly`), delivery delay is drawn from a Gaussian distribution ($\mu = 20\,\mathrm{ms}, \sigma = 10\,\mathrm{ms}$).

---

## 5. Summary Matrix: Congestion, Collision, and DES Architecture

| Dimension / Mechanism | Modeled in `rustv2xsimulator` | Mathematical / Algorithmic Basis | File Implementation |
| :--- | :---: | :--- | :--- |
| **CBR Congestion Monitoring** | **Yes** | Rolling 100 ms busy-time integration: $CBR = \frac{T_{\mathrm{busy}}}{100\,\mathrm{ms}}$ | [`ChannelMonitor`](../../src/mac/channel_monitor.rs) |
| **DCC Transmission Throttling** | **Yes** | ETSI TS 102 687 Table A.1 Reactive FSM ($60 - 460\,\mathrm{ms}$) & LIMERIC ($CBR^* = 0.68$) | [`ReactiveDccFsm`](../../src/dcc/reactive_fsm.rs) / [`LimericDcc`](../../src/dcc/limeric.rs) |
| **Carrier Sense Multiple Access** | **Yes** | CCA threshold $-85\,\mathrm{dBm}$, backoff countdown freeze/resume, AIFSN/CWmin | [`EdcaChannelAccess`](../../src/mac/edca.rs) |
| **Simultaneous On-Air Overlap** | **Yes** | Active signal registry with start/end timestamps and transmitter coordinates | [`RadioMedium`](../../src/phy/radio_medium.rs) |
| **Interference Superposition** | **Yes** | Linear power summation: $I_{\mathrm{chunk}} = \sum P_{\mathrm{rx}, j}$ in Watts | [`CumulativeInterferenceTracker`](../../src/phy/interference_helper.rs) |
| **Receiver Capture Effect** | **Yes** | $\mathrm{SIR} \ge 4.0\,\mathrm{dB}$ captures frame; $\mathrm{SIR} < 4.0\,\mathrm{dB} \wedge \mathrm{SINR} < 6.0\,\mathrm{dB}$ destroys frame | [`CumulativeInterferenceTracker`](../../src/phy/interference_helper.rs) |
| **Frame Decoding Probability** | **Yes** | Chunked $\mathrm{PSR} = \prod (1 - \mathrm{PER}_k)$ evaluated on NIST 802.11p error curve | [`compute_80211p_coded_per`](../../src/phy/interference_helper.rs) |
| **Transmission Air Time ($T_{\mathrm{trans}}$)**| **Yes** | Exact symbol and preamble duration ($448\,\mu\mathrm{s}$ for 300 bytes QPSK 1/2) | [`compute_duration_80211p`](../../src/phy/consts_80211p.rs) |
| **Propagation Delay ($\tau_{\mathrm{prop}}$)** | **Approximated** | Modeled via calibrated $10\,\mu\mathrm{s}$ turnaround offset ($T_{\mathrm{trans}} + 10\,\mu\mathrm{s}$) | [`RadioMedium`](../../src/phy/radio_medium.rs#L141) |
| **DES Scheduler Structure** | **Yes** | Multi-bucket $O(1)$ amortized Calendar Queue, integer 100 ns ticks | [`CalendarQueue`](../../src/scheduler/calendar_queue.rs) / [`DiscreteEventScheduler`](../../src/scheduler/scheduler.rs) |
