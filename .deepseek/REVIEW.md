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
