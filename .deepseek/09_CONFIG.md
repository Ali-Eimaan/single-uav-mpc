# §9 · Configuration and parameters

**Governs:** `uav_mpc/config/*.yaml`, `uav_mpc/params/*_calibration.yaml`, and the parameter
declaration/validation code in `nmpc_node.cpp`
**Milestone:** M6 (with the node)

---

## 9.1 Parameter declaration

Every key in `config/nmpc_params.yaml` is declared with a `ParameterDescriptor` carrying a
description, a `FloatingPointRange`/`IntegerRange` where meaningful, and `read_only = true` for
the structural ones (`horizon_steps`, `horizon_time`, `attitude_rep`, `airframe_params_path`).

`onParameterUpdate` accepts weights and trajectory parameters live; it rejects structural
changes with `result.reason = "horizon_steps is baked into the generated solver; re-run codegen
and restart"`. A rejection with a useful reason is worth ten with "invalid parameter".

Validate: `control_rate_hz > 0`; `state_timeout_s >= 2/control_rate_hz`;
`q_diag.size() == 12`, `r_diag.size() == 4`, `q_terminal_diag.size() == 12`; all weights ≥ 0;
`q_terminal_diag[i] >= q_diag[i]` (warn, do not reject).

The weight vectors are **12 entries, not 13** — they weight the cost residual, in which the
attitude appears as a 3-vector error. See [06_SOLVER.md §6.3](06_SOLVER.md).

## 9.2 Airframe calibration files

`params/x500_calibration.yaml` and `params/crazyflie21_calibration.yaml` carry `verified: false`.
**Flip that flag only for values you have measured yourself or traced to a cited source**, and
record the source in a trailing comment on each number.

Sources to check against:

| File | Cross-check against |
| --- | --- |
| `x500_calibration.yaml` | the PX4 Gazebo `x500` model SDF (mass, inertia, rotor positions, motor constants), the Holybro X500 v2 datasheet (arm length, prop, motor kV) |
| `crazyflie21_calibration.yaml` | the Bitcraze CF2.1 published specification, and the cited system-identification literature |

This is rule 4 in [00_RULES.md](00_RULES.md) and it matters more here than anywhere else in the
repository. An advisor evaluating this work **will** check an inertia tensor against the
datasheet.

## 9.3 `px4_overrides.yaml`

PX4 parameters that must be set for offboard attitude control to behave as the controller
assumes. Applied to SITL by `sitl.launch.py` (see [08_LAUNCH.md §8.2](08_LAUNCH.md)) and used as
the hardware checklist in `docs/HARDWARE_BRINGUP.md`.

Two consistency requirements that are easy to break and hard to notice:

- `MPC_THR_HOVER` here MUST equal `px4_hover_thrust` in `nmpc_params.yaml`. If they disagree,
  the vehicle holds a steady altitude offset and every weight you tune afterwards is
  compensating for a units bug.
- `THR_MDL_FAC` MUST be 0 unless `normaliseThrust()` implements the quadratic inversion
  ([07_NODE.md §7.6](07_NODE.md)).

Verify each parameter name against the pinned PX4 version before trusting it — parameter names
do get renamed across releases.

## 9.4 `trajectory_params.yaml`

Mirrors `TrajectoryParams` and `msg/TrajectorySpec.msg` one-to-one. When you add a field to one,
add it to all three; the service and action interfaces both carry `TrajectorySpec`.

The commented "aggressive preset" at the bottom is what produces `media/figure8.gif`. Keep the
two presets in this file rather than hard-coding demo numbers into a launch file.
