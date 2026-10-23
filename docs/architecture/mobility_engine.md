# Vehicular Mobility & SUMO TraCI Integration

This document covers how vehicle motion and microscopic traffic simulation are modeled in **`rustv2xsimulator`**.

---

## 1. Mobility Architecture

Vehicular mobility can be driven by either:

1. **Eclipse SUMO (Simulation of Urban MObility)** via the TraCI socket protocol.
2. **Built-in Synthetic Mobility Generators** (Highways, Platoons, Urban Grids) for fast headless benchmarking without external dependencies.

```mermaid
flowchart LR
    subgraph MobilitySources ["Mobility Sources"]
        SUMO["Eclipse SUMO (TraCI Client)"]
        HighwayGen["Synthetic Highway Generator"]
        PlatoonGen["Synthetic Platoon Generator"]
        UrbanGen["Synthetic Urban Grid Generator"]
    end

    subgraph TrajectorySource ["TrajectorySource Trait"]
        Step["step(dt: f64) -> Vec<VehicleKinematics>"]
    end

    subgraph Engine ["Simulation Engine"]
        KDTree["2D Spatial KD-Tree Rebuild"]
        Agents["Vehicle Agents Kinematics Update"]
    end

    SUMO & HighwayGen & PlatoonGen & UrbanGen --> Step
    Step --> KDTree & Agents
```

---

## 2. Eclipse SUMO TraCI Client

The simulator embeds a TraCI client that communicates with SUMO or SUMO-GUI over a TCP socket (default port `8813`):

- **Stepping**: The simulation engine steps SUMO by `mobility_step_s` (typically $0.1\,\text{s} = 100\,\text{ms}$).
- **State Ingestion**: At every step, the engine fetches:
  - 2D Cartesian coordinates $(x, y)$ in SUMO metric coordinates.
  - Vehicle speed ($v$ in m/s) and acceleration ($a$ in $\text{m/s}^2$).
  - Heading angle ($\theta$ in degrees from North).
  - Leader vehicle ID and longitudinal headway gap.
- **WGS84 Projection**: Uses [`GeodeticTransform`](../../src/common/geo.rs) to accurately map SUMO local metric coordinates to global WGS84 GPS latitude and longitude ($10^{-7}$ degree precision required by ETSI CAM).

---

## 3. Synthetic Mobility Generators

For rapid benchmarking and continuous integration (CI) where SUMO is not installed, the simulator provides built-in trajectory generators:

- **`SyntheticMobilityType::Highway`**: Multi-lane unidirectional highway with Poisson arrivals and Krauß car-following.
- **`SyntheticMobilityType::Platoon`**: Tight-headway vehicular platoon ($d_{gap} \in [5, 15]\,\text{m}$) ideal for evaluating Age of Information (AoI) and safety beacon latency.
- **`SyntheticMobilityType::Urban`**: Manhattan grid with orthogonal streets and traffic-light intersections.
