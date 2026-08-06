# Tuning guide

This guide is the procedure actually used to reach the defaults in
`config/nmpc_params.yaml`. The numbers here were derived from the model, cross-checked
against SITL, and reproduced by the benchmarks; where a claim rests on a measurement we
did not make, it says so.

---

## 0. Before you touch a weight

Nine times out of ten, "the MPC won't track" is a wrong mass or a wrong thrust
coefficient, not a bad weight. Do this first, in order:

1. **Verify the airframe parameters** (`params/*_calibration.yaml`). The mass goes into the
   dynamics *and* the flatness map; a 20% mass error is a 20% tracking error that no gain
   fixes. The thrust coefficient goes into the allocation map. Both are validated at
   startup (`QuadrotorDynamics::isValid`), but validation only catches absurd values.
2. **Verify the hover thrust.** Command a hover, read `normalised_thrust` off `~/status`,
   and check it sits near PX4's `MPC_THR_HOVER`. If it does not, the thrust map is wrong
   and no amount of weight tuning will fix it. This is the single cheapest diagnostic in
   the whole stack.
3. **Verify the frame conventions with a yaw-only test.** Command a 90° yaw step and
   check the sign. If yaw goes the wrong way, stop — every other result in this session is
   suspect.

## 1. Weight structure

**Diagonal weights.** The cost is a weighted sum of squared residual components, and the
residual components are already decoupled (position, velocity, quaternion error, rates,
inputs) by the model structure. Off-diagonal terms would buy coupling between, e.g., a
position error and a rate penalty — which in this problem is what the *horizon* already
provides. Keep the diagonal; add off-diagonal only if a specific coupling shows up in
practice.

**Bryson-rule starting point**: $Q_{ii} = 1/\Delta_{i,\max}^2$, where $\Delta_{i,\max}$ is
the maximum acceptable deviation of residual component $i$. The defaults in
`config/nmpc_params.yaml` were generated this way:

| Residual | $\Delta_{\max}$ | $1/\Delta_{\max}^2$ | YAML value |
| --- | --- | --- | --- |
| position xy | 0.07 m | 204 | 200.0 |
| position z | 0.05 m | 400 | 400.0 |
| velocity xy | 0.32 m/s | 9.8 | 10.0 |
| velocity z | 0.22 m/s | 20.7 | 20.0 |
| quaternion xy | 0.14 rad | 51 | 50.0 |
| quaternion z | 0.22 rad | 20.7 | 20.0 |
| body rates | 1.0 rad/s | 1.0 | 1.0 |
| input | 1.4 N | 0.51 | 0.5 |

These are *starting points*; the $\Delta_{\max}$ values are engineering judgements about
acceptable tracking error, not solver outputs.

**Why `q_diag[qw] = 0`**: with $\|q\| = 1$ the scalar part is a function of the vector
part, so weighting $q_w$ double-counts the error — the shortest-arc error quaternion
$q_{\mathrm{ref}}^{-1}\otimes q$ already carries the full attitude error in its vector
part.

## 2. Tuning order

Tune in this order and re-verify after each step; changing two things at once tells you
nothing.

1. **Altitude** — z position and z velocity weights, hovering only. Get a clean, non-
   oscillating step to 1 m before anything else. The vertical channel is the most
   independent (it barely couples into yaw), so it is the cheapest to converge.
2. **Lateral position** — x/y weights. Step response target: ~15% overshoot, then reduce
   the position weights a little until the overshoot is gone. Watch the *settling* time,
   not just the overshoot.
3. **Attitude** — quaternion weights. Symptom of too low: sluggish, lagging tracks (the
   solver is willing to tilt slowly). Symptom of too high: the solver fights PX4's inner
   loop, visible as a ~5 Hz oscillation on the rate command. Halve/double, don't nudge.
4. **Body rates** — usually the smallest weights; raise only to damp visible rate chatter
   at the 100 Hz loop. If you need more than ~3× the default to stop chatter, look for a
   structural cause (PX4 rate gains, latency) instead.
5. **Input** `r_diag` — raise to smooth the command, at the cost of tracking. The trade is
   monotone and visible: at `r = 0.5` the thrust command is crisp; at `r = 4` the commands
   smooth out but the RMS tracking error roughly doubles on the figure-8. The benchmark
   (`analysis/solve_time_benchmark.py`) does not exercise this; the tracking analysis
   notebook does.
6. **Terminal weights** — start at 2× the stage weights (the default). Symptom of too low:
   the tail of the predicted trajectory curls away from the reference near the horizon
   end (visible in the acados terminal residual in `~/status`). Symptom of too high:
   the first half of the horizon underperforms because the solver "saves" effort for the
   terminal.

## 3. Horizon selection

The experiment, run in SITL at fixed amplitude/period:

- Sweep $N \in \{10, 20, 30, 40\}$ at fixed $T_f = 1.0$ s (so $dt$ varies: 100/50/33/25 ms).
- Sweep $T_f \in \{0.5, 1.0, 2.0\}$ s at fixed $dt = 50$ ms (so $N$ varies: 10/20/40).
- Plot tracking RMS (from `analysis/tracking_error_analysis.ipynb`) and p99 solve time
  (from `analysis/solve_time_benchmark.py`) against each.

The conclusion: **$N = 20$, $T_f = 1.0$ s** is the knee. Shorter $T_f$ (< 0.5 s) cannot
see the position-dynamics consequences of a tilt (the tilt mode of the X500 is
$\omega_\mathrm{tilt} \approx 12.6$ rad/s, period ~0.5 s — a 0.5 s horizon is one tilt
period and the MPC plans into its own transient). Longer $T_f$ (> 2 s) adds stages with
negligible tracking gain: the tail of the prediction is dominated by the terminal weight
anyway. Larger $N$ at fixed $T_f$ costs quadratic-ish solve time (dense condensing block)
for sub-millimetre tracking gain. The trade-off is not "more is better": it is
"enough horizon to see the slow mode, fine enough $dt$ to resolve the fast one".

## 4. Diagnosing common failures

| Symptom | Likely cause | Check |
| --- | --- | --- |
| Steady vertical offset | wrong `px4_hover_thrust` or `THR_MDL_FAC` | `normalised_thrust` at hover vs `MPC_THR_HOVER` |
| Track lags on fast segments | unmodelled rotor drag, or `latency_compensation_s` underestimated | error vs commanded-acceleration plot in the tracking notebook |
| ~5 Hz oscillation | attitude weight too high vs PX4 inner loop | reduce quaternion weights 3× |
| Solver hits `MAX_ITERATIONS` | infeasible reference, or too-tight soft-constraint slack | `~/status` cost value + reference feasibility check |
| Occasional 5 ms solve spike | memory allocation in the loop, or CPU frequency scaling | `perf`, and check the CPU governor |
| Drift only in yaw | ENU/NED yaw sign error | yaw-only test (§0.3) |
| Consistent heading offset in flight, fine in SITL | magnetometer calibration or EKF yaw bias | EKF status flags, mag calibration |

> This table is the single most useful thing in this document. Add every failure you
> actually hit, with the fix, not just the diagnosis.

## 5. System identification

1. **Mass**: a scale, with the battery you will fly. This is a measurement, not a guess —
   the value in `params/x500_calibration.yaml` (2.0 kg) is the advertised X500 dry mass
   plus battery; re-measure on your own build.
2. **Hover thrust**: from a real hover log — the `normalised_thrust` of a settled hover
   times the maximum per-rotor thrust. Flip `verified: true` only for values you have
   actually measured or traced to a cited source.
3. **Rotor drag coefficient** (the `drag_coeff` in the calibration file): fly a
   constant-velocity segment in each body axis (a straight, level pass), and fit
   $D$ so that $R(\bm F_b - D R^{\mathsf T}\bm v) + m\bm g = m\ddot{\bm p}$ holds with
   $\ddot{\bm p} \approx 0$. The fit from a handful of passes has wide error bars (the
   drag is a few percent of thrust at these speeds); quote the confidence, and note that
   the flatness map *ignores* this term by design (see `differential_flatness.tex`).
4. **Actuation latency** (the number that goes into `latency_compensation_s`): command a
   step input and cross-correlate the commanded and measured body-rate signals. The
   dominant term is the PX4→actuator→rate-sensor loop; measure it end-to-end, don't add
   the individual budgets.

## 6. Timescale separation

The NMPC runs at 100 Hz (10 ms); PX4's rate loop runs at ~1 kHz (1 ms), and its attitude
loop at 250–500 Hz. A 10:1 ratio is defensible: the MPC shapes the *slow* states
(position, velocity, attitude error) and lets the inner loop handle the fast rates. An
attitude-setpoint interface (thrust + quaternion) is therefore the right contract for the
X500.

**Where this breaks down**: the Crazyflie 2.1. Its attitude dynamics are much faster
relative to the loop (smaller inertia, aggressive inner gains), so a 100 Hz attitude
interface sits close to the Nyquist rate of the attitude response and the loop feels
sluggish or rings. For that frame, the guide's recommendation is to switch to **body-rate
setpoints** (the adapter contract of §4 in the bring-up guide), which moves the fast
attitude loop inside PX4/Crazyswarm where it belongs.

## 7. Reproducing the README numbers

Every number in the README is produced by these exact steps (no hand-tuned exceptions):

1. **Solve-time histogram**: `python3 analysis/solve_time_benchmark.py
   --scenario hover --runs 2000` on the target compute; p99 < 5 ms. The plot title
   records the CPU model, horizon, and acados commit so the number is reproducible.
2. **Tracking error**: `ros2 launch uav_mpc figure8.launch.py record:=true
   aggressive:=false` in SITL, then run `analysis/tracking_error_analysis.ipynb` against
   the bag. RMS < 0.25 m, peak < 0.5 m on the default (2 m/8 s) figure-8.
3. **Hover assert**: `ros2 launch uav_mpc sitl.launch.py` (or `figure8.launch.py
   aggressive:=false laps:=1`) with recording, then `python3 scripts/assert_hover.py
   --max-rms 0.25 <bag>`; exit 0 is the CI gate in `.github/workflows/docker_smoke_test.yml`.
4. **Hardware**: the same commands, on the target compute, with the *measured* solve time
   recorded in the bring-up log. A 0.4 ms desktop number means nothing on a Raspberry Pi.

Config: `config/nmpc_params.yaml` as committed; hardware: Holybro X500 v2 / Pixhawk 6C /
PX4 v1.16+ in SITL. If you cannot reproduce a README number with these steps, that is a
bug — report it.
