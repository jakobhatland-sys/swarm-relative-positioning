# Relative Positioning in UAV Swarms

Estimation of relative and absolute position in a swarm of 64 UAVs by fusing UWB range measurements, RTK anchors and barometric altitude in a centralized (iterated) Extended Kalman Filter.

**Research question:** How few anchors and how little connectivity can the swarm tolerate before relative and absolute positioning accuracy break down?

![UWB network for k = 4 and k = 8](figures/uwb_network.png)

## Method

| Component | Model |
|---|---|
| Swarm | 64 UAVs, 8×8 grid, 10 m spacing, ~50 m altitude |
| Motion | Common velocity 2 m/s + individual Gauss-Markov deviation |
| UWB | Range to k nearest neighbours, σ = 0.1 m |
| Anchors | RTK position, σ = 0.02 m, placed by farthest point sampling |
| Barometer | Altitude for all UAVs, σ = 0.3 m (optional) |
| Estimator | Centralized EKF/IEKF, state [p, v] per UAV (384 states), constant-velocity model, Joseph form |
| Evaluation | Absolute RMS, relative RMS (after Kabsch alignment), ANEES, Monte Carlo |

Convergence is defined as relative RMS < 0.3 m (3σ_UWB). RMS and ANEES are reported as the median over converged runs.

## Results

### 1. Filter verification
With 16 anchors and k = 8, the filter converges to 7 cm RMS with ANEES 0.97 (95 % interval [0.91, 1.09]) for initial errors ≤ 1 m. At 2 m initial error it diverges. The implementation is correct, and the convergence basin lies between 1 and 2 m.

### 2. Anchors and connectivity (EKF, 1 m initial error, M = 10)

| Anchors | k | Conv. | Abs. RMS [m] | Rel. RMS [m] | ANEES |
|---:|---:|---:|---:|---:|---:|
| 1 | 4 | 0/10 | – | – | – |
| 2 | 4 | 0/10 | – | – | – |
| 4 | 4 | 0/10 | – | – | – |
| 8 | 4 | 0/10 | – | – | – |
| 16 | 4 | 1/10 | 0.187 | 0.183 | 1.09 |
| 1 | 6 | 4/10 | 5.803 | 0.190 | 39.57 |
| 2 | 6 | 2/10 | 3.815 | 0.211 | 26.77 |
| 4 | 6 | 5/10 | 0.205 | 0.163 | 1.03 |
| 8 | 6 | 4/10 | 0.156 | 0.135 | 1.02 |
| 16 | 6 | 10/10 | 0.098 | 0.094 | 1.00 |
| 1 | 8 | 6/10 | 6.256 | 0.119 | 47.75 |
| 2 | 8 | 9/10 | 1.356 | 0.115 | 1.20 |
| 4 | 8 | 7/10 | 0.109 | 0.093 | 0.91 |
| 8 | 8 | 10/10 | 0.094 | 0.088 | 1.01 |
| 16 | 8 | 10/10 | 0.075 | 0.074 | 1.00 |

### 3. EKF vs. IEKF (8 anchors, M = 5)

| Initial error | k | EKF conv. | IEKF conv. | Rel. RMS IEKF [m] |
|---:|---:|---:|---:|---:|
| 1 m | 4 | 0/5 | 0/5 | – |
| 1 m | 6 | 1/5 | 4/5 | 0.111 |
| 1 m | 8 | 5/5 | 5/5 | 0.095 |
| 2 m | 4 | 0/5 | 0/5 | – |
| 2 m | 6 | 0/5 | 3/5 | 0.110 |
| 2 m | 8 | 1/5 | 5/5 | 0.095 |

### 4. Barometer (IEKF, 8 anchors, 1 m initial error, M = 5)

| k | Barometer | Conv. | Rel. RMS [m] | ANEES |
|---:|---|---:|---:|---:|
| 4 | no | 0/5 | – | – |
| 4 | σ = 0.3 m | 5/5 | 0.171 | 1.01 |
| 6 | no | 4/5 | 0.111 | 0.93 |
| 6 | σ = 0.3 m | 5/5 | 0.100 | 1.02 |

## Findings

1. **Relative geometry is independent of anchors.** When converged, relative RMS is 7–21 cm regardless of anchor count. It is determined by the UWB network.
2. **Absolute position requires ≥ 4 anchors.** With 1 anchor, rotation about the anchor is unobservable (~6 m error). With 2 anchors, rotation about the axis through them is unobservable (~1.4 m). With ≥ 4 anchors, absolute RMS ≈ relative RMS.
3. **Spurious observability.** With 1–2 anchors, ANEES is 27–48. Linearization gives the filter information in unobservable directions, and P becomes overconfident.
4. **Connectivity governs robustness.** k = 8 is robust, k = 6 needs many anchors, and k = 4 fails.
5. **IEKF enlarges the convergence basin.** The convergence rate increases, especially at 2 m initial error, while accuracy is unchanged.
6. **k = 4 fails even from perfect initialization.** The problem is observability, not initialization.
7. **Root cause is planar geometry.** The swarm spans ~70 m horizontally but only ~4 m vertically, so ∂ρ/∂z ≈ 0 for nearly all UWB links. Adding a barometer gives k = 4 5/5 convergence with ANEES ≈ 1, reducing the connectivity requirement from d+1 = 4 to 3.

**Preliminary answer:** relative positioning requires 0–1 anchors; absolute positioning requires ≥ 4 anchors. Without a barometer, k ≥ 8 is needed for robust convergence; with a barometer, k = 4 is sufficient.

## Limitations

- The filter is initialized with true position plus noise; no initialization algorithm (e.g. MDS).
- Neighbour sets are selected from true positions.
- Barometer modelled with white noise, without slowly varying bias.
- UWB without outliers (NLOS) or packet loss.
- Centralized estimator; not scalable to real-time onboard use.
- M = 5–10 gives large uncertainty in convergence rates (~±30 percentage points).

## Future work

- Verify rigidity theory: k = 2 and k = 3 with barometer.
- Increase Monte Carlo runs to M ≥ 30.
- MDS initialization and robust UWB model (NLOS, Huber loss).
- Decentralized estimator (covariance intersection) compared against the centralized reference.
- Field validation with 2–4 RTK receivers and UWB modules.

## Structure

```
src/swarm/        simulation.py, sensors.py, estimator.py, metrics.py
tests/            unit tests (measurement model, Jacobian, IEKF, Kabsch)
experiments/      anchor_sweep.py, compare_iekf.py, compare_baro.py, diagnose.py, plot_network.py
figures/          generated figures
```

## Usage

```powershell
py -3.15 -m venv .venv
.venv\Scripts\Activate.ps1
pip install -r requirements.txt
python -m pytest
python experiments\anchor_sweep.py
```