# Parametric Empirical Wireless Model (Legacy)

The **Parametric Empirical Wireless Model** revives the fast, closed-form empirical channel formulation originally ported from `pyv2xsimulator` in commit `6885a2f3`.

It is designed for rapid execution, large-scale macro simulations, analytical comparisons, and ablation studies.

---

## 1. Physical Layer: Empirical Distance Loss

Rather than computing path loss in dB and calculating cumulative interference, the empirical model evaluates delivery probability using a **10th-degree polynomial** fitted to measured 802.11p vehicular packet delivery curves.

Implemented in [`LegacyPolynomialPathLoss`](../../src/phy/path_loss.rs):

$$P_{loss, dist}(d) = \sum_{k=0}^{10} C_k d^k$$

### Polynomial Coefficients
```rust
const C10: f64 = 1.111_823_869_040_061e-25;
const C9:  f64 = 3.597_485_384_820_973_5e-23;
const C8:  f64 = -8.038_884_092_343_818e-21;
const C7:  f64 = -9.158_429_982_705_657e-18;
const C6:  f64 = -2.132_059_001_989_335e-15;
const C5:  f64 = 8.192_060_372_243_259e-13;
const C4:  f64 = 4.764_017_738_147_641e-10;
const C3:  f64 = -1.596_566_671_433_574_4e-7;
const C2:  f64 = 1.867_541_672_239_932e-5;
const C1:  f64 = -0.000_498_485_400_857_115_6;
const C0:  f64 = 0.0;
```

### 350-Meter Cutoff & Boundary Clamping
In commit `6885a2f3`:

- If distance $d > 350.0\,\text{m}$, packet loss is strictly 100%: $P_{loss, dist} = 1.0$.
- For $d \le 350.0\,\text{m}$, the raw polynomial is clamped to $[0.0, 1.0]$ to prevent negative probabilities at close range ($d < 20\,\text{m}$).

---

## 2. Spatio-Temporal Logistic Congestion Loss

Medium congestion is evaluated using a logistic function over the volume of bytes transmitted within a local spatio-temporal neighborhood:

1. **Neighborhood Window**:
   - Packets active in the medium within the last **1.0 second**.
   - Within a **200-meter radius** of the transmitting vehicle.
2. **Bytes on Air**:

   $$x = N_{packets} \times \bar{L}_{bytes}$$

3. **Logistic Loss Probability**:

   $$P_{loss, cong}(x) = \frac{1}{1 + \exp\Big(-k \cdot (x - b_0)\Big)}$$

   - Parameter $k = 0.0005$ (steepness of the congestion curve).
   - Parameter $b_0 = 20\,000.0$ (midpoint threshold where congestion loss reaches 50%).

---

## 3. Line-of-Sight (LoS) Check

Building occlusion is evaluated via 2D ray intersection against building polygons:

- **Binary occlusion**: If any building edge intersects the line of sight between transmitter and receiver, the packet is **dropped unconditionally**:

  $$\text{If } \neg\text{LoS} \implies P_{deliver} = 0.0$$

---

## 4. Total Delivery Decision

A packet is delivered if and only if it survives distance loss, line-of-sight obstruction, and congestion loss:

$$P_{deliver} = (1 - P_{loss, dist}(d)) \times (1 - P_{loss, cong}(x)) \quad \text{if LoS, else } 0.0$$

A uniform random variable $u \sim \mathcal{U}(0, 1)$ determines whether the transmission is successful ($u < P_{deliver}$).

---

## 5. Gaussian Propagation Delay

If the packet is successfully delivered, its delivery timestamp is drawn from a Gaussian distribution:

$$\text{Delay} \sim \mathcal{N}(\mu, \sigma^2) \quad \text{clamped to } \ge 2\,\text{ms}$$

- Mean delay: $\mu = 0.02\,\text{s} = 20\,\text{ms}$.
- Standard deviation: $\sigma = 0.01\,\text{s} = 10\,\text{ms}$.

---

## 6. How to Enable the Parametric Model

Run with the CLI flag:

```bash
cargo run --release --example cam_simulation -- --technology parametric
```

Or configure in your TOML scenario:

```toml
[phy]
model = "legacy_poly"
```
