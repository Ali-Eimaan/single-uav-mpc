# Tuning guide

**SKELETON — outline only.** See `IMPLEMENTATION_GUIDE.md` §13.4. Fill each section with the
procedure *you actually followed*, including the numbers you rejected and why. A tuning guide
that reads like it was written from experience is worth more than a perfect set of gains.

---

## 0. Before you touch a weight

- [ ] Verify the airframe parameters (`params/*_calibration.yaml`) first. Nine times out of ten,
      "the MPC won't track" is a wrong mass or a wrong thrust coefficient.
- [ ] Verify the hover thrust: command a hover, read `normalised_thrust` off `~/status`, and
      check it sits near `MPC_THR_HOVER`. If it does not, the thrust map is wrong and no
      amount of weight tuning will fix it.
- [ ] Verify the frame conventions with a yaw-only test. If yaw goes the wrong way, stop.

## 1. Weight structure

- [ ] Why the weights are diagonal, and what the off-diagonal terms would buy.
- [ ] The Bryson-rule starting point: $Q_{ii} = 1/\Delta_{i,\max}^2$. Give the table of
      $\Delta_{\max}$ per state that generated the defaults in `config/nmpc_params.yaml`.
- [ ] Why `q_diag[qw] = 0` (the scalar part is dependent; weighting it double-counts).

## 2. Tuning order

Tune in this order and re-verify after each step; changing two things at once tells you nothing.

1. [ ] **Altitude** — z position and z velocity weights, hovering only.
2. [ ] **Lateral position** — x/y weights, step response, target ~15% overshoot then reduce.
3. [ ] **Attitude** — quaternion weights. Symptom of too low: sluggish, lagging tracks.
       Symptom of too high: the solver fights PX4's inner loop, visible as a ~5 Hz oscillation.
4. [ ] **Body rates** — usually the smallest weights; raise only to damp visible rate chatter.
5. [ ] **Input** `r_diag` — raise to smooth the command, at the cost of tracking. Give the
       trade-off curve you measured.
6. [ ] **Terminal weights** — start at 2x the stage weights. Symptom of too low: the tail of
       the predicted trajectory curls away from the reference near the horizon end.

## 3. Horizon selection

- [ ] The experiment: sweep $N \in \{10, 20, 30, 40\}$ at fixed $T_f$, and $T_f \in
      \{0.5, 1.0, 2.0\}$ s at fixed $dt$. Plot tracking RMS and p99 solve time against each.
- [ ] The conclusion and the reasoning. State the trade-off explicitly rather than just
      naming the winner.

## 4. Diagnosing common failures

| Symptom | Likely cause | Check |
| --- | --- | --- |
| Steady vertical offset | wrong `px4_hover_thrust` or `THR_MDL_FAC` | `normalised_thrust` at hover |
| Track lags on fast segments | unmodelled rotor drag, or latency underestimated | error vs commanded accel plot |
| ~5 Hz oscillation | attitude weight too high vs PX4 inner loop | reduce quaternion weights 3x |
| Solver hits `MAX_ITERATIONS` | infeasible reference, or too-tight soft-constraint slack | `~/status` cost + reference feasibility |
| Occasional 5 ms solve spike | memory allocation in the loop, or CPU frequency scaling | `perf`, and check the governor |
| Drift only in yaw | ENU/NED yaw sign error | yaw-only test |

> TODO(deepseek): keep this table, and add every failure you actually hit. This table is the
> single most useful thing in the document.

## 5. System identification

- [ ] Measuring the true mass and hover thrust.
- [ ] Identifying the rotor drag coefficient from constant-velocity flight segments: the
      procedure, the fit, and the confidence you have in it.
- [ ] Measuring the actuation latency (the number that goes into `latency_compensation_s`):
      command a step, cross-correlate the setpoint and the measured rate.

## 6. Timescale separation

- [ ] The outer NMPC runs at 100 Hz; PX4's rate loop runs at ~1 kHz. State the ratio and why
      an attitude-setpoint interface is defensible.
- [ ] Where this breaks down (the Crazyflie, whose attitude dynamics are much faster relative
      to the loop) and what you would change: direct body-rate setpoints.

## 7. Reproducing the README numbers

- [ ] Exact commands, exact config, exact hardware. Anyone should be able to re-derive every
      number in the README from this section alone.
