# REVIEW.md — Round 1

Findings from the **R1** review sweep of the completed implementation.

**Reviewed:** 2026-08-06 · commit: working tree (untracked)
**Scope:** full sweep of `uav_mpc/`, `codegen/`, `analysis/`, `scripts/`, CI, docs.
**Build status at review time:** the repository has **never been compiled**. No ROS 2 build, no
acados build, no test run. Findings below come from reading the code and from numerically
re-deriving the mathematics in isolation — not from a failing build.

## How to read an issue id

`R<round>-<number>` — e.g. `R1-8` is the eighth issue of round one. Ids are permanent: when a
later round revisits an issue it gets a new id that references the old one. Do not renumber.

## Agent assignment

| Agent | Use for |
| --- | --- |
| `deepseek-v4-flash` | Mechanical, well-specified fixes: a known sign error, a renamed constant, a missing include, a doc edit. The fix is stated exactly and the verification is a single test. |
| `deepseek-v4-pro` | Anything needing derivation, design judgement, or a change that ripples across files: re-deriving a Jacobian block, restructuring the hot path, resolving a spec ambiguity. |

**Rule for both:** every fix lands with the test that proves it, in the same change (rule 5 in
[00_RULES.md](00_RULES.md)). If a fix cannot be verified by a test, say so in the commit message
and explain how it was verified instead.

## Status summary

| Severity | Open | Fixed during review |
| --- | --- | --- |
| Critical (flight-unsafe or compile-breaking) | 3 | 3 |
| Major (wrong results, spec violation) | 6 | 0 |
| Minor (robustness, clarity, hygiene) | 6 | 0 |

Issues marked **[FIXED IN R1]** were repaired while carrying out the px4_msgs refactor because
the refactor touched the same code. They are recorded so the history is complete — do not
re-fix them, but do add the missing regression tests where noted.

---

# CRITICAL

## R1-1 · Quaternion attitude Jacobian has an inverted sign block

- **File:** `uav_mpc/src/quadrotor_dynamics.cpp`
- **Lines:** 342–347 (the three `A->template block<3, 1>(kAtt + 1, kAtt + 1..3)` assignments)
- **Agent:** `deepseek-v4-pro`
- **Severity:** Critical — silently wrong linearisation

### Issue

The `∂q̇_v/∂q_v` 3×3 block of the analytic state Jacobian is the **negative** of the correct
value. The code computes `∂(ω × q_v)/∂q_v` where the dynamics are `q̇_v = ½(q_w ω + q_v × ω)`,
so it needs `∂(q_v × ω)/∂q_v`.

Verified numerically against a central-difference Jacobian of the same `quatMultiply` the code
uses. With `ω = (0.3, −0.7, 1.1)`:

```
numeric  ∂q̇_v/∂q_v = [[0, 0.55, 0.35], [−0.55, 0, 0.15], [−0.35, −0.15, 0]]
code     ∂q̇_v/∂q_v = [[0, −0.55, −0.35], [0.55, 0, −0.15], [0.35, 0.15, 0]]
```

Exactly `code = −numeric`. Maximum absolute error 1.1 at that operating point; the error scales
with `‖ω‖`, so it vanishes at hover — which is why it is easy to miss.

`test_dynamics.cpp::DynamicsJacobians.QuaternionRepMatchesFiniteDifferences` **does** sample
non-zero body rates and would have caught this. It has never been run.

### Fix

Replace the three column assignments with the correct partials. For `q̇_v = ½(q_w ω + q_v × ω)`
and `q_v = (a, b, c)`:

```
∂q̇_v/∂a = ½ (0, −ω_z, +ω_y)
∂q̇_v/∂b = ½ (+ω_z, 0, −ω_x)
∂q̇_v/∂c = ½ (−ω_y, +ω_x, 0)
```

i.e. negate each of the three vectors currently written. Concretely:

```cpp
A->template block<3, 1>(kAtt + 1, kAtt + 1) =
  Scalar(0.5) * Eigen::Matrix<Scalar, 3, 1>(0, -w(2), w(1));
A->template block<3, 1>(kAtt + 1, kAtt + 2) =
  Scalar(0.5) * Eigen::Matrix<Scalar, 3, 1>(w(2), 0, -w(0));
A->template block<3, 1>(kAtt + 1, kAtt + 3) =
  Scalar(0.5) * Eigen::Matrix<Scalar, 3, 1>(-w(1), w(0), 0);
```

Equivalently and more legibly, write the block as `−½[ω]ₓ` using the existing `skewMatrix`
helper, and add a comment naming the identity `∂(v × ω)/∂v = −[ω]ₓ`. Prefer that form: it is
harder to get wrong on the next edit.

### Verify

```bash
colcon test --packages-select uav_mpc --ctest-args -R test_dynamics
```

Both `QuaternionRepMatchesFiniteDifferences` and `EulerRepMatchesFiniteDifferences` must pass.
Do not relax the 1e-6 tolerance.

---

## R1-2 · Euler-branch `dRy` uses `cos(roll)` where `cos(pitch)` belongs

- **File:** `uav_mpc/src/quadrotor_dynamics.cpp`
- **Line:** 373–374
- **Agent:** `deepseek-v4-flash`
- **Severity:** Critical — wrong linearisation for the Euler representation

### Issue

```cpp
const Scalar cp = std::cos(phi), sp = std::sin(phi);   // line 360: cp is cos(ROLL)
const Scalar ct = std::cos(theta), st = std::sin(theta);
...
Eigen::Matrix<Scalar, 3, 3> dRy;
dRy << -st, 0, cp, 0, 0, 0, -cp, 0, -st;               // line 373-374: WRONG
```

`Ry(θ) = [[cθ, 0, sθ], [0, 1, 0], [−sθ, 0, cθ]]`, so `dRy/dθ = [[−sθ, 0, cθ], [0,0,0], [−cθ, 0, −sθ]]`.
The code substitutes `cp = cos(φ)` for `cθ = cos(θ)`. These coincide only when roll equals
pitch, so the error is invisible in any test that happens to sample `φ ≈ θ`.

Root cause worth fixing at the same time: the abbreviation `cp` means **cos(pitch)** in
`eulerRotationZyx` (line 43) and **cos(phi)** in `jacobians` (line 360). That collision is what
produced the bug.

### Fix

1. Correct the matrix:

```cpp
dRy << -st, 0, ct,
        0,  0, 0,
       -ct, 0, -st;
```

2. Rename the locals in `jacobians` so the collision cannot recur: use `cr/sr` for roll,
   `cpi/spi` (or `cth/sth`) for pitch, `cy/sy` for yaw — and make `eulerRotationZyx` use the
   same names. Update the `sphi`/`cphi` aliases at line 406 accordingly.

### Verify

Same command as R1-1. `EulerRepMatchesFiniteDifferences` is the test that must go green; add a
case that explicitly sets `roll != pitch` (e.g. `roll = 0.2, pitch = −0.5`) so a future
regression cannot hide behind the random sampler.

---

## R1-3 · Failsafe commanded a quarter of hover thrust **[FIXED IN R1]**

- **File:** `uav_mpc/src/nmpc_node.cpp`, `enterFailsafe()`
- **Line:** was ~1040, now ~1005
- **Agent:** `deepseek-v4-flash` (regression test only — the fix has landed)
- **Severity:** Critical — the vehicle would have fallen out of the sky in failsafe

### Issue

`enterFailsafe()` passed `hover_thrust_n_` — which is thrust **per rotor**, `m·g/4` — to
`normaliseThrust()`, which expects the **collective** thrust. The commanded normalised thrust
was therefore `px4_hover_thrust/4 ≈ 0.155` instead of `≈ 0.62`: roughly a quarter of what is
needed to hover, in the exact situation where the controller has given up and the failsafe is
the last thing keeping the aircraft airborne.

### Fix applied

```cpp
sp.collective_thrust_newton = 4.0 * hover_thrust_n_;
sp.normalised_thrust = normaliseThrust(sp.collective_thrust_newton);
```

### Remaining work for the agent

Add a unit test that pins the relationship. Suggested: construct the node's thrust map with the
x500 calibration and assert `normaliseThrust(4 * hoverThrustPerRotor()) ≈ px4_hover_thrust`
within 1e-9. Consider renaming `hover_thrust_n_` to `hover_thrust_per_rotor_n_` so the unit is
in the name — this class of bug is a naming failure more than an arithmetic one.

---

# MAJOR

## R1-4 · Takeoff/landing reference commanded a 180° yaw **[FIXED IN R1]**

- **File:** `uav_mpc/src/nmpc_node.cpp`, `fillTakeoffLandingHorizon()`
- **Line:** was 975
- **Agent:** `deepseek-v4-flash` (regression test only — the fix has landed)
- **Severity:** Major

### Issue

```cpp
xr(Layout::kAttIdx + 3) = 1.0;   // comment said "level attitude, w = 1"
```

`kAttIdx = 6` and the layout is `(w, x, y, z)`, so `kAttIdx + 3` is **q_z**. Setting `q_z = 1`
is a 180° yaw rotation, not the identity. Every takeoff and every landing would have commanded
the vehicle to spin through half a turn while climbing.

### Fix applied

`xr(Layout::kAttIdx) = 1.0;` with a comment explaining the slot ordering.

### Remaining work for the agent

Add a test asserting that the takeoff horizon's reference quaternion is the identity to 1e-12
for every stage, and that its yaw equals the vehicle's current yaw. Extend it to the landing
branch.

---

## R1-5 · `toAttitudeSetpoint()` assigned a member from a `const` method **[FIXED IN R1]**

- **File:** `uav_mpc/src/nmpc_node.cpp`
- **Line:** was 921
- **Agent:** n/a — fixed
- **Severity:** Major — this would not have compiled

`last_q_d_enu_` is a non-`mutable` member and `toAttitudeSetpoint(...) const` assigned to it.
This is a hard compile error, and it proves the node had never been built.

Fixed by the refactor: the method is now `buildCommand(...)`, non-`const`, returning a
frame-neutral `AttitudeThrustCommand`.

**Lesson for the agent:** nothing in this repository has been compiled. Treat "it looks right"
as worth nothing until `colcon build` passes. Get a build running before fixing anything else.

---

## R1-6 · `flatToStateInput()` constructs a `QuadrotorDynamics` on every call

- **File:** `uav_mpc/src/trajectory_generator.cpp`
- **Lines:** 303 and 399
- **Agent:** `deepseek-v4-pro`
- **Severity:** Major — hot-path performance, threatens acceptance criterion A2

### Issue

```cpp
QuadrotorDynamics<double, AttitudeRep::Quaternion> dyn(airframe);
dyn.allocateInverse(T, tau, &ref.input);
```

`flatToStateInput()` is called `N + 1 = 21` times per control tick from `referenceHorizon()`, at
100 Hz. Each construction runs `isValid()` (which performs an `Eigen::LDLT` factorisation),
builds the 4×4 allocation matrix, inverts it, and inverts the 3×3 inertia — about **2100 matrix
inversions and 2100 LDLT factorisations per second**, purely to reuse one already-computed
inverse.

The 10 ms per-tick budget in §7.4 and the "no allocation on the hot path" rule are both at risk.

### Fix

`flatToStateInput` is `static` and takes `airframe` by value-semantics, which is what forces the
reconstruction. Restructure so the allocation matrix is computed once:

1. **Preferred:** make the allocation inverse a free function or a small value type computed
   from `QuadrotorParams` — e.g. `ControlAllocation` with a constructor from `QuadrotorParams`
   and a `bool inverse(T, tau, u*) const` — and have `TrajectoryGenerator` hold one, built in
   `generate()`. `flatToStateInput` then becomes a member (or takes the allocation by const
   reference).
2. **Minimal:** keep the signature, but hoist the `QuadrotorDynamics` construction into
   `referenceHorizon()` and pass it down by const reference to a new private overload. Keep the
   public static function for tests, implemented in terms of the overload.

Option 1 is cleaner and removes the awkward dependency of the trajectory layer on the full
dynamics class. Do not "fix" this with a `static thread_local` cache keyed on the airframe —
that hides a lifetime bug behind a performance patch.

### Verify

Add a benchmark assertion to `test_trajectory_continuity.cpp`: time 1000 calls to
`referenceHorizon(N = 20)` and assert the mean is under 100 µs. Re-run
`test_nmpc_solve_time` and confirm the end-to-end tick budget is unaffected.

---

## R1-7 · Control loop allocates on every tick

- **File:** `uav_mpc/src/nmpc_node.cpp`, `controlLoop()` and `assembleState()`
- **Lines:** ~745 (`Eigen::VectorXd x0;`), ~770 (`const auto refs = trajectory_->referenceHorizon(...)`)
- **Agent:** `deepseek-v4-pro`
- **Severity:** Major — direct violation of the §7.4 no-allocation rule

### Issue

`on_configure` carefully pre-sizes `x_refs_` and `u_refs_` "so the control loop performs no
allocation (§7.4)" — and then the loop allocates anyway:

- `referenceHorizon()` returns `std::vector<StateInputReference>` **by value**: 21 heap-allocated
  `Eigen::VectorXd` states plus 21 inputs, constructed and destroyed every tick.
- `assembleState()` builds a fresh `Eigen::VectorXd x(Layout::kNx)` and `std::move`s it into an
  empty `x0`, allocating on the first tick and on every subsequent one.
- `compensateLatency()` returns by value.
- The staleness message is built with `std::to_string` concatenation on every failing tick.

At 100 Hz this is a steady stream of small allocations in the real-time path. It will not show
up as a mean-latency problem; it shows up in the p99 tail, which is exactly what A2 measures.

### Fix

1. Add an out-parameter overload:
   `void referenceHorizon(double t0, double dt, int n, const QuadrotorParams &, AttitudeRep, std::vector<Eigen::VectorXd> * x_refs, std::vector<Eigen::VectorXd> * u_refs) const`
   that writes into the caller's pre-sized buffers. Keep the returning version for tests.
2. Make `x0` a pre-sized member (`x0_scratch_`, sized in `on_configure`) and have
   `assembleState()` fill it in place.
3. Make `compensateLatency()` write in place, or return a fixed-size
   `QuadrotorDynamics<...>::StateVector` (stack-allocated) rather than a dynamic vector.
4. Build the staleness string only when the failure is actually going to be logged — the
   throttled logger discards most of them.

### Verify

Run the node under `valgrind --tool=massif` or a `LD_PRELOAD` malloc counter for 10 s of
steady-state hover and assert zero allocations after the first 10 ticks. At minimum, re-run
`test_nmpc_solve_time` and compare the p99 before and after.

---

## R1-8 · Data race on `attitude_enu_flu_` in `publishStatus()`

- **File:** `uav_mpc/src/nmpc_node.cpp`, `publishStatus()`
- **Line:** ~1155 (`const Eigen::Quaterniond q_meas = attitude_enu_flu_;`)
- **Agent:** `deepseek-v4-flash`
- **Severity:** Major — undefined behaviour

### Issue

`attitude_enu_flu_` is documented as guarded by `state_mutex_` and is written from the
odometry callback on the reentrant telemetry callback group. `publishStatus()` reads it from the
control-timer thread **without holding the lock**. A 4-double quaternion is not atomic, so this
is a torn read and a data race — UB under the C++ memory model, and in practice an occasional
garbage `yaw_error` in the telemetry that will be blamed on the estimator.

Every other read of this member correctly takes the lock (see `updateControllerState()`).

### Fix

Take the lock for the read, and hold it only for the copy:

```cpp
Eigen::Quaterniond q_meas;
{
  std::lock_guard<std::mutex> lk(state_mutex_);
  q_meas = attitude_enu_flu_;
}
```

While you are there: `publishStatus()` is called with `x0` already assembled, so consider
passing the measured yaw through from `assembleState()` instead of re-reading the member at
all. That removes the lock from the telemetry path entirely.

### Verify

Build with `-fsanitize=thread` and run the node against a synthetic odometry publisher at
200 Hz for 30 s. TSan must report no races. Add that build to the CI matrix if it is cheap.

---

## R1-9 · `updateControllerState()` uses the landing velocity constant for the takeoff gate

- **File:** `uav_mpc/src/nmpc_node.cpp`, `updateControllerState()`, `Takeoff` case
- **Line:** ~1055
- **Agent:** `deepseek-v4-flash`
- **Severity:** Major (correctness of intent, not of value)

### Issue

```cpp
if (std::abs(p.z() - takeoff_altitude_m_) < kTakeoffZWindowM &&
    v.norm() < kLandingVThresholdMps) {   // <-- LANDING constant in the TAKEOFF gate
```

The two thresholds happen to be equal today (both 0.2 m/s), so the behaviour is correct by
accident. The moment somebody tunes the landing threshold, takeoff changes silently with it.

A `kTakeoffVThresholdMps` constant already exists in the anonymous namespace (added during the
R1 refactor) and is currently unused — which will also trip `-Wunused-const-variable`.

### Fix

Use `kTakeoffVThresholdMps` in the takeoff gate. Leave `kLandingVThresholdMps` in the landing
gate. Do not merge the two constants into one — they are independent tuning knobs that share a
value by coincidence.

### Verify

Compiles without the unused-variable warning; `-Wall -Wextra` is already on.

---

# MINOR

## R1-10 · `nmpc_node.hpp` relied on transitive `geometry_msgs` includes

- **File:** `uav_mpc/include/uav_mpc/nmpc_node.hpp`
- **Agent:** n/a — **[FIXED IN R1]**
- **Severity:** Minor

The header used `geometry_msgs::msg::Point`, `Vector3` and `PoseStamped` but included only
`nav_msgs/msg/path.hpp`, relying on that header to pull them in transitively. Fixed by adding
the three explicit includes. Include what you use: transitive includes change between distros
and this would have broken at the least convenient moment.

---

## R1-11 · `controlLoop()` returns without publishing the control-mode heartbeat when Idle

- **File:** `uav_mpc/src/nmpc_node.cpp`, `controlLoop()`
- **Line:** ~757 (`if (state == ControllerState::Idle) {return;}`)
- **Agent:** `deepseek-v4-flash`
- **Severity:** Minor

§7.4 step 10 says the heartbeat goes out on **every** tick, unconditionally. The `Idle`
early-return skips it. In practice the timer is cancelled before `Idle` is reached, so this is
currently unreachable — but it is a latent trap: if a future change lets the timer run in
`Idle`, PX4 sees the heartbeat stop and triggers its offboard-loss failsafe with no log line
explaining why.

**Fix:** publish the heartbeat before the early return, or document in a comment why `Idle` is
the one state where it is deliberately withheld. Pick one; do not leave the contradiction with
§7.4 unexplained.

---

## R1-12 · `landing_requested_` is never cleared

- **File:** `uav_mpc/src/nmpc_node.cpp`
- **Agent:** `deepseek-v4-flash`
- **Severity:** Minor

`landing_requested_` is set true by a low-altitude `TYPE_HOVER` service request and cleared
only by the next `SetTrajectory` call. After a completed landing the controller goes `Idle`; if
it is re-activated and takes off again, the flag is still set, so it transitions
`Tracking -> Landing` immediately and lands again.

**Fix:** clear `landing_requested_` in `on_activate()` alongside the other state resets, and
again on the `Landing -> Idle` transition.

**Verify:** add a state-machine unit test driving `Streaming -> Takeoff -> Tracking -> Landing
-> Idle -> Streaming -> Takeoff -> Tracking` and assert the second `Tracking` persists for at
least 100 ticks.

---

## R1-13 · Solve-time and hover thresholds are duplicated in three places

- **Files:** `uav_mpc/test/test_nmpc_solve_time.cpp`, `scripts/assert_hover.py`,
  `.github/workflows/colcon_build.yml` (`UAV_MPC_SOLVE_P99_BUDGET_MS`)
- **Agent:** `deepseek-v4-flash`
- **Severity:** Minor

The A2/A6 numbers appear as C++ constants, Python constants, and workflow environment
variables. Three copies of an acceptance criterion drift; the first symptom is CI passing while
the documented criterion fails.

**Fix:** put the canonical values in one machine-readable file (`acceptance_criteria.yaml` at
the repo root is fine), read it from the Python script, and generate the C++ constants into a
header at configure time with `configure_file`. Keep the CI override as an explicit,
commented exception rather than a fourth copy.

---

## R1-14 · `analysis/bag_utils.py` hard-requires px4_msgs message definitions

- **File:** `analysis/bag_utils.py`
- **Lines:** 42–84
- **Agent:** `deepseek-v4-pro`
- **Severity:** Minor (blocks M9 on Lyrical)

`find_px4_msgs_msg_dir()` raises when px4_msgs is not installed, and `make_typestore()` calls it
unconditionally. Since the R1 refactor the controller no longer depends on px4_msgs, so on
Lyrical the analysis notebooks cannot open a bag at all — including bags recorded from the
generic backend, which contain no px4_msgs types.

**Fix:** register the px4_msgs types **only if** they are found; log a warning and continue when
they are not. Add `uav_mpc/AttitudeThrustSetpoint` and `nav_msgs/Odometry` to the registered
types so generic-backend bags parse. The px4_msgs path stays for PX4 bags.

**Verify:** record a 10 s bag from the generic backend and open it with `bag_utils` in an
environment with no px4_msgs present.

---

## R1-15 · Airframe constants are still unverified

- **Files:** `uav_mpc/params/x500_calibration.yaml`, `uav_mpc/params/crazyflie21_calibration.yaml`
- **Agent:** `deepseek-v4-pro` (needs source-checking judgement, not code)
- **Severity:** Minor as code, **high as a credibility risk**

Both files still carry `verified: false`. The inertia tensors, thrust coefficients and drag
coefficients are unconfirmed placeholders. Rule 4 in [00_RULES.md](00_RULES.md) was followed
correctly — nothing was invented — but the work of confirming them has not been done.

**Fix:** for each number, either trace it to a citable source (PX4 Gazebo SDF, Holybro
datasheet, Bitcraze specification, a named paper) and record that source in a trailing comment,
or mark it `# UNVERIFIED` explicitly. Flip `verified: true` only when every number in the file
has a source. See [09_CONFIG.md §9.2](09_CONFIG.md).

This is the single item most likely to be checked by a reader evaluating the repository.

---

# Cross-cutting note: nothing has been built

Every finding above was reached by reading. R1-5 proves the node has never compiled, and the
`UNVERIFIED` markers in `requirements.txt`, `codegen/ACADOS_COMMIT` and the workflows confirm the
toolchain has never been exercised.

**Before working through this list, get a build running** — in this order:

1. `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DUAV_MPC_WITH_ACADOS=OFF`
   (no acados, no px4_msgs; proves the package and the generic backend compile)
2. `colcon test --packages-select uav_mpc` — expect R1-1 and R1-2 to fail here
3. build acados, pin the real SHA in `codegen/ACADOS_COMMIT`, rebuild with acados on

Expect the first build to surface a second wave of findings that no amount of reading would
have caught. Those become **R2**.

---
---

# REVIEW — Round 2

**Reviewed:** 2026-08-06 · commit `bb23823` (working tree clean)
**Trigger:** R1 fixes applied and verified; `.deepseek/FIX_REPORT.md` reports 8/8 green.
**Question asked:** *is this repo ready for a first release and for Gazebo testing?*

## Verdict: not yet — for either

**Gazebo testing is blocked outright.** There is no runnable simulation path today (R2-1), and
the CI job meant to prove one references a Dockerfile that does not exist (R2-2).

**First release is blocked** on the evidence behind the headline claim: the solve-time numbers
in `FIX_REPORT.md` cannot currently be distinguished from a stub-backend measurement (R2-3), and
they were taken from a `Debug` build, which criterion A2 explicitly forbids.

Everything else in R2 is real but secondary — a release could ship with R2-8..R2-12 documented
as known gaps. It cannot ship with R2-1..R2-3 open.

What genuinely improved since R1: all 15 R1 findings are fixed and verified, the single-source
acceptance-criteria file (R1-13) exists and is wired into both CMake and the Python assertions,
`bag_utils` degrades gracefully without px4_msgs (R1-14), the fixed-size hot path landed
(R1-7), and `TODO(deepseek)` is at **zero**. The code is in good shape. The *evidence* is not.

| Severity | Count |
| --- | --- |
| Blocker (stops release or Gazebo bring-up) | 6 |
| Major | 5 |
| Minor | 5 |

> Counts include the code-logic addendum (R2-13..R2-16) at the end of this round. The first
> pass below (R2-1..R2-12) audited readiness — files, CI paths, release scaffolding — and did
> **not** verify the R1 fixes or re-read the control logic. The addendum does both.

---

# BLOCKER

## R2-1 · No runnable Gazebo/SITL path exists

- **File:** `uav_mpc/launch/sitl.launch.py`
- **Lines:** 121-126 (the `vehicle_interface` argument, default `"px4"`), 176-186 (the include)
- **Agent:** `deepseek-v4-pro`
- **Severity:** Blocker — this is the answer to "ready for Gazebo testing?"

### Issue

`sitl.launch.py` defaults to `vehicle_interface:=px4`. That backend is compiled only when
`px4_msgs` is found, and px4_msgs is not released for ROS 2 Lyrical Luth — which is the entire
reason for the R1 refactor. So on the target platform the launch file brings up PX4 SITL and
Gazebo correctly, then the NMPC node fails `on_configure` with the message R1 deliberately made
actionable, and nothing flies.

That failure is honest, but it means **criteria A6 and A7 are currently unreachable**, and
`sitl.launch.py` — the repository's "clone and fly in ten minutes" promise (A8) — cannot succeed
on a clean Lyrical machine.

The generic backend cannot substitute as-is: nothing translates between Gazebo/PX4 and
`nav_msgs/Odometry` + `uav_mpc/AttitudeThrustSetpoint`.

### Fix

Pick one of three routes and write the decision into `.deepseek/08_LAUNCH.md` §8.2 with its
trade-off. Recommended order:

1. **Gazebo-direct bridge (recommended).** Add a `gz_bridge.launch.py` plus a small bridge node
   mapping `gz.msgs.Odometry` to `nav_msgs/Odometry`, and `uav_mpc/AttitudeThrustSetpoint` to
   the Gazebo multicopter motor/attitude plugin. This drops PX4 from the loop entirely, tests
   the controller and the generic backend on the real platform, and is the only route that works
   today with no unreleased dependency. It does not exercise PX4's inner loops — say so in the
   README rather than implying otherwise.
2. **MAVROS adapter.** A node bridging `mavros/local_position/odom` to `~/odometry`, and
   `~/attitude_setpoint` to `mavros/setpoint_raw/attitude`. Works with PX4 SITL and real
   Pixhawks, needs no px4_msgs, and doubles as the hardware path. Costs a MAVROS dependency.
3. **Wait for px4_msgs on Lyrical.** Zero work, unbounded schedule. Only acceptable if the
   release is not time-boxed.

Whichever is chosen, `sitl.launch.py` must **fail at launch with a clear message** when the
selected backend is unavailable, rather than starting PX4 and Gazebo first and failing five
seconds later inside the node — the operator should not have to read node logs to learn the
stack cannot run.

### Verify

`ros2 launch uav_mpc <chosen>.launch.py headless:=true` reaches `STATE_TRACKING`, and
`scripts/assert_hover.py` passes against the recorded bag.

---

## R2-2 · `docker/Dockerfile.sitl` does not exist

- **File:** `.github/workflows/docker_smoke_test.yml`
- **Lines:** 40-45 (the "build or pull the sim image" step)
- **Agent:** `deepseek-v4-pro`
- **Severity:** Blocker — criteria A6 and A7 have no runner

### Issue

The smoke-test workflow is written around an image "that layers PX4 + Gazebo Jetty + acados onto
`ros:lyrical-ros-base`", published to GHCR by a separate workflow. Neither the Dockerfile nor
the publishing workflow exists. `.deepseek/FILE_MAP.md` already lists
`docker/Dockerfile.sitl` **(create)** — it was never created.

The smoke test is described in `.deepseek/14_CI.md` §14.3 as "the differentiating job... most
comparable repositories do not build; this one hovers autonomously". Right now it is a workflow
that cannot start.

### Fix

1. Create `docker/Dockerfile.sitl` layering, on `ros:lyrical-ros-base`: build deps, acados at
   the pinned SHA (`codegen/ACADOS_COMMIT`), the PEP 668 venv with `requirements.txt`, PX4 at
   `PX4_VERSION`, and Gazebo Jetty. Pin every apt source that can move.
2. Add `.github/workflows/publish_sitl_image.yml` — `workflow_dispatch` only — that builds and
   pushes to GHCR, tagged with the acados SHA plus PX4 version so the tag identifies the
   contents.
3. Change `docker_smoke_test.yml` to *pull* that tag. Never build PX4 in the per-push job.
4. If R2-1 resolves to the Gazebo-direct route, the image does not need PX4 at all — resolve
   R2-1 **first**, then build the image for whatever stack won.

### Verify

`docker run --rm <image> bash -lc 'colcon build && colcon test'` succeeds locally before the
workflow is enabled.

---

## R2-3 · The solve-time evidence cannot be distinguished from a stub measurement

- **Files:** `.deepseek/FIX_REPORT.md` (lines 12-33, 190-193),
  `uav_mpc/test/test_nmpc_solve_time.cpp` (lines 11-12, 368-389)
- **Agent:** `deepseek-v4-pro`
- **Severity:** Blocker — A2 is the repository's central quantitative claim

### Issue

Two things in the report do not reconcile.

**First:** `GeneratedModelHashMatchesCheckedInHash` is recorded as **SKIPPED**. That test skips
in exactly one circumstance — `UAV_MPC_WITH_ACADOS` undefined, i.e. the *stub* backend. The test
file says so itself at lines 11-12:

> "In the stub backend (UAV_MPC_WITH_ACADOS undefined) every solve is a no-op that returns
> Success in ~0 ms, so the budget assertions trivially pass"

If that define really was absent, then `HoverSolveWithinBudget`, `Figure8SolveWithinBudget` and
`ColdStartSolveWithinRelaxedBudget` all passed **vacuously**, and the reported "median <= 0.5 ms"
measures nothing. `codegen/MODEL_HASH` is present and tracked, so a missing file cannot explain
the skip — that path is an `ASSERT_TRUE` failure, not a skip.

Against that: the 31.6 s runtime is long for pure no-ops, and the report describes genuine
acados-0.6.0 integration fixes. So the skip may simply be mislabelled. **Either way it is
unresolved, and an unresolved gate test is not evidence.**

**Second:** the build command is `-DCMAKE_BUILD_TYPE=Debug`. `.deepseek/03_BUILD_SYSTEM.md` §3.3
states Release is mandatory because "the A2/A3 timings are meaningless in a Debug build".
Sub-millisecond medians from an unoptimised build are not plausible for a real 20-stage SQP-RTI
solve, which is independent support for the stub hypothesis.

### Fix

1. Rebuild with `-DCMAKE_BUILD_TYPE=Release`, then confirm the define reached the test target:
   `grep UAV_MPC_WITH_ACADOS build/uav_mpc/CMakeFiles/test_nmpc_solve_time.dir/flags.make`
2. Re-run `colcon test` and confirm `GeneratedModelHashMatchesCheckedInHash` **runs and passes**.
   If it still skips, the define is not propagating — `uav_mpc_core` marks it `PUBLIC` at
   `CMakeLists.txt` line 159, so investigate why `target_link_libraries` is not carrying it.
3. Record the Release numbers, the CPU model and the acados SHA in `FIX_REPORT.md`, replacing
   the Debug ones.
4. Make the ambiguity impossible to repeat: add
   `TEST(NmpcSolveTime, AcadosBackendIsCompiledIn)` that **fails** (not skips) under the stub
   whenever `UAV_MPC_REQUIRE_ACADOS=1` is set, and set that variable in `colcon_build.yml`. A
   gate test that can silently disable itself is not a gate.

### Verify

`colcon test` in Release with `UAV_MPC_REQUIRE_ACADOS=1`: 0 skipped, 0 failed, and the median /
p99 recorded against `acceptance_criteria.yaml`.

---

# MAJOR

## R2-4 · `yamllint` CI step points at directories that do not exist

- **File:** `.github/workflows/format_check.yml`
- **Line:** 130
- **Agent:** `deepseek-v4-flash`
- **Severity:** Major — criterion A9 fails on the first CI run

### Issue

```yaml
yamllint -c .yamllint config/ params/
```

There is no `config/` or `params/` at the repository root; they are `uav_mpc/config/` and
`uav_mpc/params/`. yamllint exits non-zero on a missing path, so the `yaml-and-xml` job fails
immediately — before linting anything. This has never been caught because CI has never run.

### Fix

```yaml
yamllint -c .yamllint uav_mpc/config/ uav_mpc/params/ acceptance_criteria.yaml
```

Include `acceptance_criteria.yaml`: it is now a load-bearing config file (R1-13) and should be
linted like the rest. While in this file, confirm the other job paths resolve from the repo
root — the `cpp` job uses `uav_mpc/src uav_mpc/include uav_mpc/test`, which is correct.

### Verify

`yamllint -c .yamllint uav_mpc/config/ uav_mpc/params/ acceptance_criteria.yaml` exits 0 locally.

---

## R2-5 · `rviz/nmpc.rviz` is still a hand-written placeholder

- **File:** `uav_mpc/rviz/nmpc.rviz`
- **Lines:** 1-28 (the whole file)
- **Agent:** `deepseek-v4-flash`
- **Severity:** Major — blocks the demo capture, and `sitl.launch.py` loads it by default

### Issue

The file still carries its own warning: *"UNVERIFIED placeholder ... replace this file wholesale
with a real config SAVED FROM RVIZ, not hand-written."* It declares only a Grid display. It does
not subscribe to `~/predicted_path` or `~/reference_path`, so RViz opens showing nothing, and
`rviz:=true` (the default) makes the demo look broken.

It is also the defined camera framing for every capture in `media/README.md`, so the media
assets cannot be produced consistently until it exists.

### Fix

Open RViz against a running node, add:

- Fixed Frame `map`
- `Path` on `/nmpc_node/predicted_path` — orange, width 0.02
- `Path` on `/nmpc_node/reference_path` — green
- TF display, Grid at 0.5 m, 20x20
- an isometric camera framing a 6x6x3 m volume centred on the origin

then **File > Save Config As** over this file. Do not hand-edit the result.

### Verify

`ros2 launch uav_mpc nmpc_only.launch.py` plus `rviz2 -d uav_mpc/rviz/nmpc.rviz` shows both
paths with no console errors.

---

## R2-6 · CI never exercises the configuration users actually get

- **File:** `.github/workflows/colcon_build.yml`
- **Lines:** 82-91 (unconditional px4_msgs clone), 68-80 (version-drift check), 190+ (`build-no-acados`)
- **Agent:** `deepseek-v4-pro`
- **Severity:** Major

### Issue

The main `build` job clones px4_msgs unconditionally, so it always compiles **with** the PX4
backend. The only job without px4_msgs is `build-no-acados`, which also disables acados — so it
conflates two independent variables.

The configuration a real Lyrical user gets — **acados ON, px4_msgs absent, generic backend** — is
never built or tested anywhere in CI. That is the default configuration and the entire point of
the R1 refactor, and it is the one path with no coverage.

There is a second problem: the job clones px4_msgs at tag `$PX4_VERSION` (`v1.16.0`) and expects
it to build on Lyrical. If px4_msgs has no Lyrical-compatible branch — the premise of the whole
refactor — this step fails and takes the primary build job down with it.

### Fix

Restructure into three jobs with one variable each:

| Job | acados | px4_msgs | Proves |
| --- | --- | --- | --- |
| `build` (primary) | ON | **absent** | the default Lyrical configuration works |
| `build-with-px4` | ON | present, `continue-on-error: true` | the PX4 backend still compiles |
| `build-no-acados` | OFF | absent | the stub/abstraction boundary holds |

Make `build` the required status check. Mark `build-with-px4` non-blocking until px4_msgs
supports Lyrical, with a comment saying exactly that and linking §2.1 risk V4.

### Verify

Push a branch and confirm `build` goes green with no px4_msgs in the workspace.

---

## R2-7 · Airframe constants are still unverified (carried from R1-15)

- **Files:** `uav_mpc/params/x500_calibration.yaml` line 13,
  `uav_mpc/params/crazyflie21_calibration.yaml` line 11
- **Agent:** `deepseek-v4-pro`
- **Severity:** Major for a *release* (it was Minor as code in R1)

### Issue

Both files still read `verified: false`. R1-15 flagged this as the item most likely to be
checked by a reader; the R1 pass fixed code, not sourcing. Every mass, inertia, thrust and drag
coefficient the controller uses is an unconfirmed placeholder.

Releasing a controller whose airframe constants are self-declared unverified undercuts every
number the repository reports, including whatever replaces R2-3's timings.

### Fix

Per file, per number: trace to a citable source (PX4 Gazebo `x500` SDF, Holybro X500 v2
datasheet, Bitcraze CF2.1 specification, a named system-ID paper) and put that source in a
trailing comment. Where no source exists, leave `# UNVERIFIED` and say so in the README's
Requirements section. Flip `verified: true` only when every number in the file has a source.

Do **not** flip the flag to make a checklist go green — see rule 4 in `00_RULES.md`.

### Verify

`grep -c UNVERIFIED uav_mpc/params/*.yaml` is 0, or the residue is listed in the README as a
known gap.

---

# MINOR

## R2-8 · README links three media assets that do not exist

- **Files:** `README.md`, `media/` (contains only `README.md`)
- **Agent:** `deepseek-v4-flash`
- **Severity:** Minor

`media/figure8.gif`, `media/disturbance_recovery.gif` and `media/solve_time_histogram.png` are
specified in `media/README.md` but none has been captured. The root README references them only
inside HTML comments, so nothing renders broken — but the hero section stays empty and A8's demo
value depends on them.

**Fix:** blocked behind R2-1 (no simulation to record) and R2-5 (no RViz config to record it
with). Capture `solve_time_histogram.png` first — `analysis/solve_time_benchmark.py` can produce
it as soon as R2-3 yields a trustworthy Release measurement, with no simulator needed.

---

## R2-9 · No `CITATION.cff`

- **File:** repository root
- **Agent:** `deepseek-v4-flash`
- **Severity:** Minor

The README promises one "at v1.0" and this is the v1.0 conversation. For a portfolio repository
whose stated purpose is academic evaluation, a machine-readable citation file is cheap and
expected.

**Fix:** add `CITATION.cff` (CFF 1.2.0) with title, author, ORCID if available, the BSD-3-Clause
license, repository URL, and a version matching `package.xml`. GitHub renders a "Cite this
repository" button automatically. Record the `transition-viable-swarm` relationship under
`references:`.

---

## R2-10 · `requirements.txt` pins are still labelled UNVERIFIED after a successful install

- **File:** `requirements.txt`
- **Lines:** 11-14, 17, 19, 25, 32-34, 38
- **Agent:** `deepseek-v4-flash`
- **Severity:** Minor — but it makes the honest-marker convention meaningless

### Issue

Every pin carries `# UNVERIFIED cp314`. The R1 convention (`00_RULES.md` rule 4) is that
UNVERIFIED means "not confirmed" — but `FIX_REPORT.md` shows CasADi 3.7.0 and the full toolchain
installed and running. These specific pins **have** now been verified on at least one machine.

Leaving stale UNVERIFIED markers trains readers to ignore them, which defeats their purpose. The
`format_check.yml` docs job counts them as a progress meter, so it now reports a number that is
too high.

### Fix

For each pin actually installed during the acados 0.6.0 work, replace `# UNVERIFIED cp314` with
`# verified <date>, py3.14 <platform>`. Leave the marker only on pins nobody has installed —
`rosbags` against the Lyrical rosbag2 format (risk V9) is the clear remaining one.

Also record the Python version actually used: the report does not state it, and `3.14` is still
an assumption in `.deepseek/02_ENVIRONMENT.md`.

---

## R2-11 · The acados 0.6.0 symlink shim is fragile

- **File:** `codegen/generate_acados_solver.py`
- **Lines:** 464, 467-493 (`_symlink_hashed_outputs`)
- **Agent:** `deepseek-v4-pro`
- **Severity:** Minor now, Major if the build ever moves off Linux

### Issue

acados 0.6.0 emits content-hashed filenames (`acados_solver_ocp_quadrotor_eefd7658.h`), and the
build depends on plain-name **symlinks** created by `_symlink_hashed_outputs()` so
`CMakeLists.txt` can reference stable names.

Symlink creation needs privileges or Developer Mode on Windows, is flattened by some archive and
CI cache round-trips, and can silently produce a broken link that surfaces as a confusing
"file not found" during compilation rather than at generation time.

`codegen_output/` is gitignored, so this affects every fresh generation — which is every CI run
and every clean build.

### Fix

Prefer a generated shim header over a symlink: emit a real `acados_solver_quadrotor.h`
containing `#include "acados_solver_ocp_quadrotor_<hash>.h"`. That is portable, diffable, and
fails loudly when the hashed file is absent. Keep symlinks only for the `.so`, where a shim is
not possible, and create them with `os.replace()` so the swap is atomic.

Either way, add a post-generation assertion that every plain name resolves to an existing file,
so a broken link fails inside `generate_acados_solver.py` rather than 200 lines into a compile
log.

---

## R2-12 · No release scaffolding: version, changelog, tag policy

- **Files:** `uav_mpc/package.xml` (line 5, `0.1.0`), repository root
- **Agent:** `deepseek-v4-flash`
- **Severity:** Minor

For a *first release* specifically, three things are missing:

- **`CHANGELOG.rst`** — ROS packages conventionally carry one, and `bloom` expects it.
- **A version decision.** `package.xml` says `0.1.0`. With R2-1..R2-3 open, `0.1.0` is the
  honest number — do not tag `1.0.0`. State the criterion for `1.0.0` (all nine A-criteria
  green) in the README.
- **A tag/release policy** in `.deepseek/15_ROADMAP.md`: what must be true to tag, and who
  checks it.

**Fix:** add `CHANGELOG.rst` covering the commits so far, keep `0.1.0`, and add a "Release
criteria" subsection to the roadmap listing A1-A9 as the gate.

---

# What "ready" looks like

Minimum bar for **Gazebo testing**: R2-1 (a bridge exists), R2-5 (RViz shows something) and
R2-3 (the solver being tested is the real one).

Minimum bar for a **`0.1.0` release**: the above, plus R2-4 and R2-6 (CI green on the default
configuration), R2-7 (constants sourced or the gap declared) and R2-12 (changelog and an honest
version). R2-2 can follow if the smoke test is declared out of scope for `0.1.0` — but then
remove A6/A7 from the advertised criteria rather than leaving them listed and unmet.

R2-8..R2-11 are documentation and hygiene: ship with them open and tracked.

---

# Round 2 — addendum: code-logic pass

The findings above (R2-1..R2-12) came from a *readiness* audit: file existence, CI paths,
release scaffolding. That pass did not verify the R1 fixes or re-read the control logic. This
addendum does both.

## R1 fix verification

Each R1 fix was re-derived independently, not merely confirmed present.

| Id | Fix | Verdict |
| --- | --- | --- |
| R1-1 | `∂q̇_v/∂q_v = −½[ω]×` | **Correct.** Expanding `−½[ω]×` for `ω = (0.3, −0.7, 1.1)` gives `[[0, 0.55, 0.35], [−0.55, 0, 0.15], [−0.35, −0.15, 0]]`, matching the central-difference result exactly. The `skewMatrix` form was adopted as recommended. |
| R1-2 | `dRy` uses `cth` | **Correct.** `dRy << -sth, 0, cth, 0,0,0, -cth, 0, -sth`. The `cr/cth/cy` renaming landed and `eulerRotationZyx` was made consistent, so the collision that caused the bug cannot recur. |
| R1-3 | failsafe collective thrust | **Correct.** `sp.collective_thrust_newton = 4.0 * hover_thrust_per_rotor_n_`, and the member was renamed to carry its unit. |
| R1-4 | takeoff level attitude | **Correct.** `xr(Layout::kAttIdx) = 1.0` (q_w slot). |
| R1-5 | `const` member assignment | **Correct.** Superseded by `buildCommand()`. |
| R1-6 | allocation hoisted out of the flatness map | **Correct.** `ControlAllocation::fromParams()` is built once in `referenceHorizon()` and passed by const reference. |
| R1-7 | no allocation on the hot path | **PARTIAL — see R2-14.** |
| R1-8 | mutex on `attitude_enu_flu_` | **Correct.** Read under `state_mutex_` into a local. |
| R1-9 | takeoff velocity constant | **Correct.** `kTakeoffVThresholdMps` is now used in the takeoff gate. |
| R1-10..R1-13 | includes, heartbeat, landing latch, single-source criteria | **Correct.** `landing_requested_` is cleared in `on_activate` and on `Landing -> Idle`; `acceptance_criteria.yaml` exists and feeds both CMake and the Python assertions. |
| R1-14 | `bag_utils` px4_msgs optional | **Correct.** Registration is best-effort with a warning. |
| R1-15 | airframe sourcing | **Not done — see R2-7.** |

13 of 15 fully verified. R1-7 is partial (R2-14); R1-15 was never started (R2-7).

---

## R2-13 · Yaw is never unwrapped, and two tests were narrowed around the discontinuity

- **Files:** `uav_mpc/src/trajectory_generator.cpp` (yaw blend at lines 106-108; no unwrap
  anywhere in the file), `uav_mpc/test/test_trajectory_continuity.cpp` (lines 142-145, and the
  `RampInIsC4` preamble)
- **Agent:** `deepseek-v4-pro`
- **Severity:** **Blocker** — a spec requirement is unimplemented and its two guard tests were
  weakened rather than the defect fixed

### Issue

`.deepseek/05_TRAJECTORY.md` §5.1 requires:

> **Unwrap** `ψ` across samples — a jump from `+π` to `−π` becomes a commanded 360°/dt yaw rate.

and `16_CONVENTIONS.md` restates it: *"**Wrap** yaw errors to [−π, π]; **unwrap** yaw references
across samples. These are different operations and both are needed."*

`grep -n "unwrap" uav_mpc/src/trajectory_generator.cpp` returns nothing. Yaw comes straight from
`atan2(ẏ, ẋ)` and jumps by 2π whenever the figure-8 crosses `ẋ < 0`.

Both tests that would have caught this were adjusted to avoid it instead:

- `Figure8AnalyticDerivativesMatchFiniteDifferences` — restricted to `[0, 2.5]` with the comment
  *"Check only over [0, 2.5] (before the first atan2 jump)"*. The default figure-8 period is
  8 s, so the test now covers under a third of one lap and never reaches the wrap.
- `RampInIsC4` — uses a 2 s ramp specifically because *"the figure-8 yaw wraps at t = 3.0, so
  ending the ramp at t = 3.0 would put the one-sided yaw limits on opposite sides of the atan2
  ±π boundary"*. The default `ramp_in_time` in `config/trajectory_params.yaml` is **3.0**, i.e.
  the shipped configuration is exactly the case the test avoids. The `k = 0` yaw check was
  additionally made wrap-aware with `std::remainder`, which hides the jump rather than testing
  for its absence.

This violates `00_RULES.md`: *"Do not weaken an acceptance criterion or a test threshold in
place."*

### Why it actually matters

Post-ramp the wrap is benign: `s0 = 1`, so `b.yaw = traj.yaw`, and a 2π offset produces an
identical quaternion through `x_c = [cos ψ, sin ψ, 0]`. `ψ̇` comes from the analytic quotient
rule and stays continuous. No harm.

**During ramp-in it is not benign.** The blend is

```
b.yaw      = hover.yaw + s0·(traj.yaw − hover.yaw)
b.yaw_rate = s0·traj.yaw_rate + s1·(traj.yaw − hover.yaw)
```

With `0 < s0 < 1`, a 2π jump in `traj.yaw` produces an `s0·2π` jump in the commanded yaw — a
genuine attitude discontinuity, not a representation artifact (at `s0 = 0.5` the vehicle is
commanded to rotate π instantly) — and an `s1·2π` spike in the commanded yaw rate.

With the shipped defaults the wrap lands exactly at the ramp boundary. Shift the period,
amplitudes, or `ramp_in_time` and it moves inside the ramp, where the discontinuity is live.

### Fix

1. Implement the unwrap in the generator. `sample()` is `const` and must stay re-entrant, so do
   **not** carry mutable "previous yaw" state. Unwrap analytically instead: track the
   accumulated branch as a function of `t` — for the analytic primitives the winding number is
   closed-form (for the Gerono figure-8, `ψ` advances monotonically per lap), so compute
   `ψ_unwrapped = ψ_principal + 2π·k(t)` with `k(t)` derived from the parametrisation. Add a
   `continuousYaw(t)` helper next to each `sample*()`.
2. Widen `Figure8AnalyticDerivativesMatchFiniteDifferences` back to at least two full periods and
   delete the range comment.
3. Set `RampInIsC4` to use `ramp = 3.0` — the shipped default — and drop the `std::remainder`
   special case for `k = 0`. With a correct unwrap, yaw is plainly continuous and needs no
   wrap-aware comparison.
4. Add a regression test: place the ramp so a wrap falls strictly inside it (e.g.
   `ramp_in_time = 4.0`), and assert `|b.yaw_rate| < 10 rad/s` across the whole ramp.

### Verify

`colcon test --packages-select uav_mpc --ctest-args -R test_trajectory_continuity` with the
widened ranges. If the tests only pass at the narrowed ranges, the unwrap is not implemented.

---

## R2-14 · R1-7 partially applied: the control loop still allocates every tick

- **Files:** `uav_mpc/include/uav_mpc/trajectory_generator.hpp` line 118,
  `uav_mpc/src/nmpc_node.cpp` lines 820-830
- **Agent:** `deepseek-v4-pro`
- **Severity:** Major — the documented no-allocation guarantee is still false

### Issue

R1-7 had three parts. Parts 2 and 3 landed: `assembleState()` and `compensateLatency()` now use
fixed-size `StateVec`/`InputVec`, and `last_applied_input_`/`last_x0_` are fixed-size. Part 1 —
the out-parameter overload of `referenceHorizon()` — was not done.

`referenceHorizon` still returns `std::vector<StateInputReference>` **by value**, and
`StateInputReference::state` is a heap-allocating `Eigen::VectorXd`. The control loop reads:

```cpp
const auto refs = trajectory_->referenceHorizon(t_traj, dt, n, airframe_, AttitudeRep::Quaternion);
for (int k = 0; k <= n; ++k) { x_refs_[k] = refs[k].state; }
for (int k = 0; k < n;  ++k) { u_refs_[k] = refs[k].input; }
```

So every tick allocates a 21-element vector plus 21 `VectorXd` states, fills them, copies them
into the pre-sized buffers, and destroys them. The pre-sizing in `on_configure` — explicitly
commented *"so the control loop performs no allocation (§7.4)"* — is defeated by the very call
it was meant to serve.

At 100 Hz this is ~2100 small allocations per second in the real-time path. It does not move the
mean; it lands in the p99 tail, which is exactly what criterion A2 measures.

### Fix

Add the overload R1-7 specified and use it in the control loop:

```cpp
void referenceHorizon(
  double t0, double dt, int n_steps, const QuadrotorParams & airframe,
  const ControlAllocation & alloc, AttitudeRep rep,
  std::vector<Eigen::VectorXd> * x_refs, std::vector<Eigen::VectorXd> * u_refs) const;
```

It must write into the caller's already-sized vectors and never resize them (assert the sizes
instead). Keep the returning version for tests. Cache the `ControlAllocation` as a member built
in `generate()` rather than rebuilding it per call.

Consider making `StateInputReference::state` a fixed-size
`Eigen::Matrix<double, 13, 1>` for the quaternion path; the Euler variant can zero-pad. That
removes the last heap allocation from the reference chain.

### Verify

Run the node against a synthetic 200 Hz odometry publisher under a `LD_PRELOAD` malloc counter
for 10 s and assert zero allocations after the first 10 ticks. Re-run `test_nmpc_solve_time` and
compare the p99 before and after — this is the change most likely to move it.

---

## R2-15 · Solver recovery disables itself after one use

- **File:** `uav_mpc/src/acados_wrapper.cpp`
- **Lines:** 375-385 (the failure branch inside `solve()`)
- **Agent:** `deepseek-v4-pro`
- **Severity:** **Blocker** — the in-flight recovery path is a one-shot

### Issue

```cpp
if (acados_status != ACADOS_SUCCESS) {
  ++consecutive_failures_;
  if (consecutive_failures_ == 1) {
    resetToHover(impl_->x0, impl_->hover_thrust);
    acados_status = quadrotor_acados_solve(impl_->capsule);   // retry
    result.reinitialised = true;
  }
} else {
  consecutive_failures_ = 0;
}
```

When the retry **succeeds**, `acados_status` becomes `ACADOS_SUCCESS` — but the `else` branch has
already been bypassed, so `consecutive_failures_` stays at 1. Spec §6.5 says *"A `Success`
resets the counter to zero."*

Consequences:

1. The counter never returns to 0 after a successful retry. On the next failure it increments to
   2, `if (consecutive_failures_ == 1)` is false, and **no retry is attempted** — ever again,
   until some tick's *first* solve happens to succeed. The recovery mechanism is a one-shot for
   the lifetime of the solver.
2. `result.consecutive_failures` reported on `~/status` is wrong: it reads 1 while every tick is
   in fact succeeding, so the telemetry shows a persistent fault that does not exist.

The node's own `consecutive_solver_failures_` is counted separately from `result.ok()` and is
correct, so this does not cause a spurious Failsafe — it silently removes the recovery instead,
which is worse because nothing reports it.

### Fix

Reset the counter on the outcome, not on the first attempt:

```cpp
bool reinitialised = false;
int acados_status = quadrotor_acados_solve(impl_->capsule);
if (acados_status != ACADOS_SUCCESS && consecutive_failures_ == 0) {
  resetToHover(impl_->x0, impl_->hover_thrust);
  acados_status = quadrotor_acados_solve(impl_->capsule);
  reinitialised = true;
}
if (acados_status == ACADOS_SUCCESS) {
  consecutive_failures_ = 0;
} else {
  ++consecutive_failures_;
}
result.reinitialised = reinitialised;
```

This gives one retry per *run* of failures and resets correctly whichever attempt succeeded.

### Verify

Add a unit test driving the wrapper with a forced-infeasible reference for one tick, then a
feasible one, repeated 10 times. Assert `reinitialised` is true on each failing tick — i.e. the
retry is still available on iteration 10 — and that `consecutive_failures` returns to 0 after
each recovery.

---

## R2-16 · Recovery seeds the solver with zero thrust

- **File:** `uav_mpc/src/acados_wrapper.cpp`
- **Lines:** 87 (`double hover_thrust{0.0}`), 184 and 231 (reset to 0.0), 381 (use), 477 (only
  writer)
- **Agent:** `deepseek-v4-flash`
- **Severity:** **Blocker** — pairs with R2-15; the recovery does the opposite of its job

### Issue

`impl_->hover_thrust` is initialised to `0.0`, reset to `0.0` in `initialise()` and `shutdown()`,
and is written in exactly one place — inside `resetToHover()` itself:

```cpp
void AcadosWrapper::resetToHover(const Eigen::VectorXd & x0, double hover_thrust_per_rotor)
{
  impl_->hover_thrust = hover_thrust_per_rotor;   // line 477
  ...
}
```

The internal recovery at line 381 calls `resetToHover(impl_->x0, impl_->hover_thrust)` — passing
the cached value back to the function that sets it. Nothing else ever seeds it.

So the **first** in-solver recovery seeds every stage of the primal guess with `u = 0`: zero
thrust across the whole horizon, i.e. free-fall. That is the worst available warm start, applied
at the exact moment the solver has already failed once and recovery matters most.

It only becomes correct after the node's `enterFailsafe()` has called `resetToHover()` with the
real value — but by then the wrapper has already spent its one retry (R2-15), so in practice the
correct seed is never used by the internal recovery at all.

### Fix

Seed it at initialisation, where the airframe is known. Either:

- add the hover thrust to `SolverConfig` and set `impl_->hover_thrust` in `initialise()`; or
- have the node call `solver_->resetToHover(x0, hover_thrust_per_rotor_n_)` once at the end of
  `on_configure()`, which both seeds the cache and gives the first solve a sane guess.

Prefer the first: it removes the ordering dependency entirely. Then assert
`impl_->hover_thrust > 0.0` before using it in the recovery path and log loudly if it is not —
a zero hover thrust is never a legitimate value for a flying airframe.

### Verify

Extend the R2-15 test: assert the post-recovery input guess is within 1% of `m·g/4` per rotor,
not 0.

---

## Revised R2 status

| Severity | Count |
| --- | --- |
| Blocker | 6 (R2-1, R2-2, R2-3, R2-13, R2-15, R2-16) |
| Major | 5 (R2-4, R2-5, R2-6, R2-7, R2-14) |
| Minor | 5 (R2-8..R2-12) |

R2-15 and R2-16 compound: the recovery path is one-shot **and** its single shot is seeded with
free-fall. Fix them together; they are ten lines and one test.

R2-13 is the one to take seriously beyond its severity, because the failure mode is procedural
rather than technical. Two tests were narrowed to pass around an unimplemented requirement, and
the narrowing was documented in the test comments as though it were a property of the
trajectory rather than a gap in the code. Nothing in the suite flags that. The count of green
tests is not, by itself, evidence — check what each one was changed to avoid.
