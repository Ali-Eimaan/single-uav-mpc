# §16 · Conventions and known traps

**Read before M1. Re-read whenever something behaves inexplicably.** Most of the expensive bugs
in a quadrotor controller are on this page.

---

## Frames

World **ENU**, body **FLU**, everywhere except at the PX4 message boundary. Every conversion
lives in `quadrotor_dynamics.cpp`. If you find yourself writing a sign flip anywhere else, you
have found a bug, not a shortcut.

PX4 speaks NED/FRD. The conversions are in
[04_DYNAMICS.md §4.4](04_DYNAMICS.md); they are involutive, so one implementation serves both
directions.

## Quaternions

Hamilton, `(w, x, y, z)`, body-to-world.

**Eigen's `Quaterniond` constructor takes `(w, x, y, z)` but its `.coeffs()` returns
`(x, y, z, w)`.** This mismatch has cost more quadrotor-hours than any other single line of
code. Never `memcpy` a quaternion; never pass `.coeffs().data()` to something expecting wxyz.

Error quaternions use the shortest arc: negate the vector part when `w < 0`. Skipping this gives
you the unwinding phenomenon ([06_SOLVER.md §6.3](06_SOLVER.md)).

## Units

SI throughout. Thrust in **newtons** in every internal interface; normalisation to PX4's `[0,1]`
happens exactly once, in `normaliseThrust()`.

## Time

`rclcpp::Time` internally; **microseconds** at the PX4 boundary; seconds in trajectory
parameters. Trajectory time is measured from `trajectory_start_time_`, not wall clock.

Passing nanoseconds to PX4 does not error — the setpoints are silently discarded.

## Angles

Radians internally. **Wrap** yaw *errors* to `[−π, π]`; **unwrap** yaw *references* across
samples. These are different operations and both are needed — wrapping a reference produces a
commanded 360°/dt yaw rate at the discontinuity.

## Logging

Nothing unthrottled in the control loop. `RCLCPP_*_THROTTLE` with a 1 s period. A `printf` at
100 Hz is a latency bug, not a debugging aid.

## Allocation

Nothing dynamic in the control loop after the first tick. Reserve every buffer in
`on_configure` / `initialise`.

---

## The failures you will actually hit

| Symptom | First thing to check |
| --- | --- |
| **No messages arriving in SITL** | QoS profile ([07_NODE.md §7.2](07_NODE.md)), then `px4_msgs` vs PX4 version (risk V4). Check both before debugging anything else — this is the single most likely first failure. |
| Vehicle flips on takeoff | allocation matrix signs ([04_DYNAMICS.md §4.3](04_DYNAMICS.md)), not the controller |
| Steady altitude offset | `px4_hover_thrust` vs `MPC_THR_HOVER`, or `THR_MDL_FAC ≠ 0` |
| Reference "looks smooth" but rates spike | ramp-in product rule ([05_TRAJECTORY.md §5.2](05_TRAJECTORY.md)) |
| Velocities scaled by segment duration | minimum-snap chain rule ([05_TRAJECTORY.md §5.3](05_TRAJECTORY.md)) |
| Drift only in yaw | ENU/NED yaw sign |
| Solver infeasible in flight | a hard state constraint that should be soft ([06_SOLVER.md §6.4](06_SOLVER.md)) |
| Occasional multi-ms solve spike | allocation in the loop, or CPU frequency scaling |
| Predicted tail curls off the reference | terminal weight below the stage weight |

## A note on debugging order

When something is wrong in SITL, the cost of checking goes: conventions (minutes) → unit tests
(minutes) → the C++/CasADi agreement test (minutes) → SITL logs (hours). Work in that order. The
tests specified in [10_TESTS.md](10_TESTS.md) exist precisely so that the cheap checks are
available before you need them.
