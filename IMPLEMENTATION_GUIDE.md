# IMPLEMENTATION_GUIDE.md

Specification for implementing the `uav-mpc` skeleton.

**Audience:** an implementing model (`deepseek-v4-flash`) filling in the stubs.
**Scope:** every file in this repository that contains a `TODO(deepseek)` marker.
**Authority:** where this guide and a code comment disagree, this guide wins — but say so in
the commit message rather than silently diverging.

---

## §0 · How to use this guide

Rules, in priority order:

1. **One file per change.** Implement a file completely, build it, then move on. Do not open
   six files and leave all of them half-finished.
2. **Do not change public signatures.** The headers in `uav_mpc/include/` and the
   `.msg`/`.srv`/`.action` definitions are the contract. If a signature is genuinely wrong,
   stop and say so instead of quietly editing it — several files depend on each one.
3. **Delete the `TODO(deepseek)` comment when you implement it.** A leftover TODO on
   implemented code is a lie that costs the next reader ten minutes. `format_check.yml`
   counts them as a progress meter.
4. **Never invent a physical constant.** If you do not have a source for a number, leave the
   placeholder and add `# UNVERIFIED` next to it. A fabricated inertia tensor discredits the
   whole repository.
5. **Every claim needs a check.** If you implement something whose correctness is not obvious
   (a Jacobian, a frame conversion, an allocation matrix), add the test that proves it in the
   same change.
6. **Build order matters.** Follow §15. The dependency chain is real: nothing above it
   compiles until the layer below does.
7. **If a step is blocked** (missing acados, missing PX4, a decision this guide does not
   settle), implement everything that is not blocked, then report exactly what is blocked and
   why. Do not stub around it silently.

Symbols used below: **MUST** = required for correctness or safety. **SHOULD** = strong
default, deviate only with a stated reason.

---

## §1 · What is being built, and what "done" means

A nonlinear MPC that tracks aggressive quadrotor trajectories at 100 Hz:

```
trajectory_generator ──flat outputs──► flatness map ──(x_ref, u_ref)──► acados SQP-RTI
                                                                              │
     PX4 (uXRCE-DDS) ──odometry──► nmpc_node ─────────────────────────────────┘
                                       │
                                       └──► VehicleAttitudeSetpoint ──► PX4 inner loops
```

### Acceptance criteria (the whole project)

| # | Criterion | Verified by |
| --- | --- | --- |
| A1 | Clean-container build succeeds, Release | `colcon_build.yml` |
| A2 | p99 NMPC solve time < 2 ms, median < 1 ms | `test_nmpc_solve_time.cpp` |
| A3 | Generated solver matches the committed model hash | `test_acados_codegen.py` |
| A4 | CasADi model and C++ dynamics agree to 1e-9 | `test_acados_codegen.py` |
| A5 | References are C⁴; flatness map consistent with the dynamics | `test_trajectory_continuity.cpp` |
| A6 | SITL: 10 s autonomous hover, RMS error < 0.15 m, zero solver failures | `docker_smoke_test.yml` |
| A7 | SITL: 2 laps of a figure-8, RMS error < 0.25 m | `docker_smoke_test.yml` |
| A8 | `sitl.launch.py` flies from a clean clone in one command | manual, before each release |
| A9 | All linters clean | `format_check.yml` |

If a criterion cannot be met, **change the criterion in this file with a written reason**.
Do not weaken a test in place.

---

## §2 · Environment

Pinned, and pinned in one place each:

| Component | Version | Pinned in |
| --- | --- | --- |
| Ubuntu | 24.04 | workflow `runs-on` |
| ROS 2 | Jazzy | container image |
| PX4 | see `PX4_VERSION` | `.github/workflows/docker_smoke_test.yml` |
| `px4_msgs` | tag matching `PX4_VERSION` | `colcon_build.yml` |
| Gazebo | Harmonic | `docker_smoke_test.yml` |
| acados | commit in `codegen/ACADOS_COMMIT` | that file |
| CasADi / numpy / … | exact `==` pins | `requirements.txt` |
| Eigen | 3.4 (system) | `package.xml` |

**`px4_msgs` must match the PX4 version.** A mismatched message definition produces no error
— it produces silence, or worse, fields read at the wrong offset. When you touch either
version, touch both.

Local setup:

```bash
mkdir -p ~/ws/src && cd ~/ws/src && git clone <this-repo> uav-mpc
```

```bash
python3 -m venv ~/.venvs/uavmpc && . ~/.venvs/uavmpc/bin/activate && pip install -r ~/ws/src/uav-mpc/requirements.txt
```

```bash
export ACADOS_SOURCE_DIR=$HOME/acados && export LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib:$LD_LIBRARY_PATH
```

---

## §3 · Build system — `uav_mpc/CMakeLists.txt`

### 3.1 acados discovery

```cmake
if(UAV_MPC_WITH_ACADOS)
  if(NOT DEFINED ENV{ACADOS_SOURCE_DIR})
    message(FATAL_ERROR "ACADOS_SOURCE_DIR is not set. Either export it, or configure with "
                        "-DUAV_MPC_WITH_ACADOS=OFF to build the stub backend.")
  endif()
endif()
```

Include directories MUST be, in order:
`$ENV{ACADOS_SOURCE_DIR}/include`, `.../include/blasfeo/include`, `.../include/hpipm/include`,
and the generated tree `${CMAKE_CURRENT_SOURCE_DIR}/../codegen/codegen_output/c_generated_code`.

Import three shared libraries as `IMPORTED` targets: `libacados.so`, `libblasfeo.so`,
`libhpipm.so`, plus the generated `libacados_ocp_solver_quadrotor.so`.

### 3.2 Codegen freshness

Add a custom command that re-runs codegen when the model is newer than the output:

- **DEPENDS:** `codegen/quadrotor_model.py`, `codegen/generate_acados_solver.py`,
  `uav_mpc/config/nmpc_params.yaml`, the selected `params/*_calibration.yaml`
- **OUTPUT:** `codegen_output/c_generated_code/acados_solver_quadrotor.c`
- **COMMAND:** `python3 ${CMAKE_CURRENT_SOURCE_DIR}/../codegen/generate_acados_solver.py …`

This MUST NOT run in CI (CI uses `--check-only` and fails on drift). Guard it with
`if(NOT DEFINED ENV{CI})`.

### 3.3 Rules

- `UAV_MPC_WITH_ACADOS` is a `target_compile_definitions` on `${PROJECT_NAME}_core`
  **PRIVATE** only. If it leaks `PUBLIC`, consumers need acados headers and the stub CI job
  fails — which is exactly what that job is for.
- Release MUST be `-O2` or better. A3/A2 timings are meaningless in a Debug build.
- `-Wconversion` is on deliberately. Fix the warnings; do not silence them.

---

## §4 · `quadrotor_dynamics.{hpp,cpp}`

### 4.1 `QuadrotorParams::fromYaml`

Required keys (all mandatory except `gravity`, which defaults to 9.80665), read from the
`airframe:` root of `params/*_calibration.yaml`:

`name, mass, inertia.{ixx,iyy,izz,ixy,ixz,iyz}, arm_length, thrust_coeff, torque_coeff,
min_thrust_per_rotor, max_thrust_per_rotor, rotor_time_constant, drag_coeff[3]`

On a missing key: `throw std::runtime_error("quadrotor params: missing key 'airframe.mass' in " + path)`.
Name the key and the file. Do not default silently.

`isValid()` MUST check, and put the failing condition in `*why`:

- `mass > 0`
- inertia symmetric, and positive-definite (`Eigen::LDLT` succeeds and all diagonal entries of D > 0)
- `thrust_coeff > 0`, `torque_coeff > 0`, `arm_length > 0`
- `0 <= min_thrust_per_rotor < max_thrust_per_rotor`
- `4 * max_thrust_per_rotor > mass * gravity` (thrust-to-weight > 1)

### 4.2 Continuous dynamics

World **ENU**, body **FLU**, gravity acts along `-z_world`.

State (quaternion, `nx = 13`): `x = [p(3), v(3), q(4, wxyz), ω(3)]`, `u = [T₁..T₄]` in newtons.

```
ṗ = v
v̇ = (1/m) · ( R(q)·[0,0,ΣTᵢ]ᵀ − R(q)·D·R(q)ᵀ·(v − v_wind) ) + [0,0,−g]
q̇ = ½ · q ⊗ [0, ω]
ω̇ = J⁻¹ · ( τ − ω × (J·ω) )
```

with `D = diag(drag_coeff)`. Note the drag term: the drag is **linear in body-frame velocity**,
so it is rotated into the body frame, scaled, and rotated back. Applying `D` directly to the
world velocity is a different (wrong) model.

Euler variant (`nx = 12`) replaces `q̇` with `Θ̇ = T(Θ)·ω`,

```
T(Θ) = [[1, sinφ·tanθ,  cosφ·tanθ],
        [0, cosφ,      −sinφ     ],
        [0, sinφ/cosθ,  cosφ/cosθ]]
```

singular at `θ = ±π/2`. The Euler branch is for analysis only — **the flight solver MUST use
the quaternion branch**.

`step()` is classical RK4 with a single step, then `q ← q/‖q‖` when `Rep == Quaternion`.

### 4.3 Control allocation (PX4 quad-X)

Motor numbering is PX4's, **not** the intuitive clockwise order. In body FLU with
`d = arm_length/√2`:

| Motor | Position (x, y) | Spin | Reaction torque |
| --- | --- | --- | --- |
| 1 | front-right (+d, −d) | CCW | −z |
| 2 | rear-left (−d, +d) | CCW | −z |
| 3 | front-left (+d, +d) | CW | +z |
| 4 | rear-right (−d, −d) | CW | +z |

A thrust `Tᵢ` at `rᵢ = (xᵢ, yᵢ, 0)` produces `τ = rᵢ × (0,0,Tᵢ) = (yᵢTᵢ, −xᵢTᵢ, 0)`. With
`c = torque_coeff / thrust_coeff`:

```
[ T  ]   [  1    1    1    1 ] [T₁]
[ τx ] = [ −d   +d   +d   −d ] [T₂]
[ τy ]   [ −d   +d   −d   +d ] [T₃]
[ τz ]   [ −c   −c   +c   +c ] [T₄]
```

`buildAllocationMatrix()` builds this once and caches `inertia_inv_`.
`allocateInverse()` applies `allocation_.inverse()` (precompute it too), then clamps each
rotor into `[min_thrust_per_rotor, max_thrust_per_rotor]` and returns `false` if any component
was clamped.

> **Verify this table before trusting it.** Cross-check against the PX4 quad-X mixer in the
> pinned PX4 version and against the Gazebo model SDF, and record what you checked in a
> comment. A wrong sign here produces a vehicle that flips on takeoff in SITL and looks like
> "the MPC is unstable".

### 4.4 Frame conversions

```
ENU → NED:  (x, y, z) → ( y,  x, −z)        (involutive: NED → ENU is the same map)
FRD → FLU rates: (ωx, ωy, ωz) → (ωx, −ωy, −ωz)   (also involutive)
```

Quaternions: with `q_a = (w=0, x=√2/2, y=√2/2, z=0)` (180° about the ENU (1,1,0)/√2 axis) and
`q_b = (w=0, x=1, y=0, z=0)` (180° about body x),

```
q_ned_frd = q_a ⊗ q_enu_flu ⊗ q_b
```

and the same expression converts back (both fixed rotations are self-inverse). Implement one
function and call it from the other, so they cannot drift apart.

**Test these first.** Add `test/test_frame_conversions.cpp`: round-trip 1000 random
quaternions and vectors to 1e-12, and check three hand-computed cases (level, 90° yaw,
30° roll). Frame bugs are the most expensive bugs in this repository and the cheapest to test.

### 4.5 Jacobians

Analytic `A = ∂f/∂x`, `B = ∂f/∂u`. Verify against central differences with step `1e-6` and
relative tolerance `1e-6` in `test/test_dynamics.cpp`. These are not used by acados (which has
its own AD) — they exist so the model can be checked without acados present.

---

## §5 · `trajectory_generator.{hpp,cpp}`

### 5.1 Analytic primitives

All derivatives up to snap MUST be closed-form. **Do not finite-difference inside the
generator** — the flatness map differentiates again, and numerical noise becomes commanded
body-rate noise.

Figure-8 (Gerono), `ω = 2π/period`:

```
x(t) = Aₓ·sin(ωt)
y(t) = A_y·sin(ωt)·cos(ωt) = (A_y/2)·sin(2ωt)
z(t) = altitude + A_z·sin(ωt)
```

Differentiate symbolically four times; each `d/dt` brings down a factor of `ω` (or `2ω`).

Yaw when `yaw_follows_velocity`: `ψ = atan2(ẏ, ẋ)`, `ψ̇` and `ψ̈` from the quotient rule —

```
ψ̇  = (ẋÿ − ẏẍ) / (ẋ² + ẏ²)
ψ̈  = (ẋy⃛ − ẏx⃛)/(ẋ² + ẏ²) − 2(ẋÿ − ẏẍ)(ẋẍ + ẏÿ)/(ẋ² + ẏ²)²
```

Guard `ẋ² + ẏ² < 1e-6` by holding the previous yaw. **Unwrap** `ψ` across samples — a jump
from `+π` to `−π` becomes a commanded 360°/dt yaw rate.

### 5.2 Ramp-in

`rampScale(t, T_ramp, k)` returns the `k`-th time derivative of `S₄(t/T_ramp)`, where `S₄` is
the degree-9 smoothstep — the unique 9th-order polynomial with `S₄(0)=0`, `S₄(1)=1`, and
derivatives 1–4 vanishing at both ends:

```
S₄(u) = 70u⁹ − 315u⁸ + 540u⁷ − 420u⁶ + 126u⁵
```

Verify `S₄(0)=0`, `S₄(1)=1`, and `S₄⁽ᵏ⁾(0) = S₄⁽ᵏ⁾(1) = 0` for `k = 1..4` in a unit test rather
than trusting the coefficients printed here.

The blend is `σ(t) = σ_hover + S₄(t/T)·(σ_traj(t) − σ_hover)`; the derivatives follow from the
**product rule**, so `sample()` needs `S₄⁽ᵏ⁾` for `k = 0..4`. Forgetting this is the single
most common bug in this file — it produces a reference that looks smooth in position and has a
step in jerk.

### 5.3 Minimum snap

Per axis (x, y, z, ψ), per segment: a 7th-order polynomial on normalised `τ ∈ [0,1]`,
minimising `∫₀¹ (d⁴p/dτ⁴)² dτ`.

Hessian entries for `i, j ≥ 4` (zero otherwise):

```
Q_ij = [i!/(i−4)!] · [j!/(j−4)!] / (i + j − 7)
```

scaled by `1/T⁷` when mapping back to real time.

Equality constraints:

- position at each waypoint (both sides of every interior boundary)
- continuity of derivatives 1–4 at every interior boundary
- derivatives 1–4 zero at the start and the end

Solve the KKT system directly:

```
[ Q  Aᵀ ] [ c ]   [ 0 ]
[ A  0  ] [ λ ] = [ b ]
```

with Eigen's dense `LDLT` (fall back to `FullPivLU` if the factorisation reports failure, and
log which was used). Waypoint counts are small (< 30), so dense is fine — say so in a comment
so a reader knows it was a decision.

Chain rule when evaluating: `dᵏp/dtᵏ = (dᵏp/dτᵏ)/Tᵏ`. Getting this wrong scales your
velocities by the segment duration and is invisible for `T = 1 s`.

### 5.4 Time allocation

When `segment_times` is empty, for each segment of length `dᵢ`:

```
Tᵢ = 1.2 · max( dᵢ/v_max , √(2dᵢ/a_max) )
```

Floor at 0.1 s. The 1.2 factor is slack for the cornering the straight-line estimate ignores.

### 5.5 Differential flatness map

Given `(a, j, s, ψ, ψ̇, ψ̈)` and the airframe, in world ENU with `g = 9.80665`:

```
t   = a + [0,0,g]ᵀ                        required specific force
z_b = t/‖t‖                               body z-axis (world coords)
T   = m·‖t‖                               collective thrust [N]
x_c = [cos ψ, sin ψ, 0]ᵀ
y_b = (z_b × x_c)/‖z_b × x_c‖
x_b = y_b × z_b
R   = [x_b  y_b  z_b]                     body → world
```

Body rates from the jerk projection:

```
Ṫ   = m·(z_b · j)
h_ω = (m/T)·( j − (z_b · j)·z_b )
ω_x = −h_ω · y_b
ω_y =  h_ω · x_b
ω_z = ( ψ̇·(x_c · x_b) + ω_y·(y_c · z_b) ) / ‖y_c × z_b‖,   y_c = [−sin ψ, cos ψ, 0]ᵀ
```

Angular acceleration from the snap projection follows by differentiating the same relation;
derive it in `docs/derivations/differential_flatness.tex` and implement what you derived.
Then `τ = J·ω̇ + ω × (J·ω)` and `u_ref = allocateInverse(T, τ)`.

**Degenerate cases MUST be handled explicitly:**

- `‖t‖ < 1e-3` (free fall): the attitude is undefined. Hold the previous attitude, zero the
  rates, set `T = 0`, and log once at WARN.
- `‖z_b × x_c‖ < 1e-6` (yaw axis aligned with body z): perturb `ψ` by `1e-4` and recompute.

**Verification (do this, it is cheap and it catches everything):** compute `ω` a second way, by
finite-differencing `R(t)` and extracting `[ω]× = Rᵀ Ṙ`, and assert agreement to 1e-5. If the
projection formulas above have a sign error, this test finds it in seconds; SITL would take
you a day.

---

## §6 · `acados_wrapper.{hpp,cpp}`

### 6.1 Boundary rule

`acados_wrapper.cpp` is the **only** file that may include an acados header. Everything acados
touches is behind the PIMPL. `build-no-acados` in CI enforces it. When you need something from
acados elsewhere, widen the wrapper's interface rather than including the header.

### 6.2 Solver configuration (all of these are decisions — do not "improve" them casually)

| Setting | Value | Why |
| --- | --- | --- |
| `N` | 20 | with `Tf = 1 s` gives `dt = 50 ms` |
| `Tf` | 1.0 s | ≈ one figure-8 quarter-lap at the demo speed |
| `nlp_solver_type` | `SQP_RTI` | real-time iteration; one QP per control tick |
| `nlp_solver_max_iter` | 1 | that is what RTI means |
| `qp_solver` | `PARTIAL_CONDENSING_HPIPM` | best measured fit at this horizon |
| `qp_solver_cond_N` | 5 | condensing block size; sweep it in §12 and record the result |
| `hessian_approx` | `GAUSS_NEWTON` | least-squares cost; exact Hessian is not worth the time |
| `integrator_type` | `ERK`, 4 stages, 1 step | explicit is enough for a non-stiff quadrotor at 50 ms |
| `qp_solver_iter_max` | 50 | bounded, so a bad QP cannot blow the budget |
| `globalization` | `FIXED_STEP` | line search costs time RTI does not have |
| `hpipm_mode` | `SPEED` | |
| `print_level` | 0 | printing from a 100 Hz loop is a latency bug |

### 6.3 Cost

`NONLINEAR_LS`, because the attitude error is not affine in the state.

```
y   = [ p(3), v(3), vec(q_ref⁻¹ ⊗ q)(3), ω(3), u(4) ]     ny   = 16
y_e = [ p(3), v(3), vec(q_ref⁻¹ ⊗ q)(3), ω(3) ]           ny_e = 12
```

`q_ref` therefore enters through the **online parameter vector**, not `yref`:
`p = [wind(3), mass_scale, q_ref(4)]`, `np = 8`. `yref` carries the position/velocity/rate
reference and `u_ref`, with the attitude-error block held at zero.

Sign convention: take the vector part of the error quaternion, negated if `w < 0`, so the cost
always drives along the shortest arc. Without this you get the unwinding phenomenon — the
vehicle takes the long way round a large yaw error, which looks spectacular and is a bug.

> Rejected alternative, recorded so it is not re-litigated: `LINEAR_LS` weighting the raw
> quaternion components. It penalises the wrong quantity near large attitude errors and cannot
> express the shortest-arc convention. Do not switch to it for the sake of a faster solve.

### 6.4 Constraints

- **Hard**, per rotor: `lbu = min_thrust_per_rotor`, `ubu = max_thrust_per_rotor`, `idxbu = [0,1,2,3]`.
  Always feasible by construction, so these cannot cause infeasibility.
- **Soft**, body rates: `|ω| ≤ 6 rad/s` with an L2 slack penalty (`Zl = Zu = 1e2`,
  `zl = zu = 1e1`). Hard state constraints under RTI go infeasible in flight the first time a
  disturbance pushes you outside the set, and then you are on the recovery path at 100 Hz.
- `x₀` via the initial-state equality (`lbx_0 = ubx_0 = x₀`), set every tick.
- No position constraints in v0.1. CBF-based ones are the thesis contribution and belong in
  the downstream repository.

### 6.5 Failure recovery

`solve()` returns, never throws. On a non-zero acados status:

1. **First failure:** `resetToHover(x₀, hoverThrustPerRotor())`, solve exactly once more, set
   `reinitialised = true`. If the retry succeeds, report `Success` with the flag set.
2. **Failures 2..max−1:** report the status; the node holds the previous input.
3. **`consecutive_failures >= max_consecutive_failures`:** report; the node enters `Failsafe`.

A `Success` resets the counter to zero. `solve()` MUST also enforce
`solve_time_budget_ms` — if the measured wall time exceeds it, report `Timeout` even when
acados returned success, so the budget breach shows up in the logs rather than as a missed
deadline you never see.

### 6.6 Hot-path rules

No allocation inside `solve()`, `setStageReference()`, `setInitialState()`, or
`optimalInput()`. All scratch buffers are sized once in `initialise()`. Use
`Eigen::Map<Eigen::VectorXd>` over the existing buffers rather than constructing vectors.

---

## §7 · `nmpc_node.{hpp,cpp}`

### 7.1 Lifecycle

| Transition | Does | On failure |
| --- | --- | --- |
| `on_configure` | load params, airframe, model, trajectory, solver; create subs (inactive pubs), service, param callback | `FAILURE` + one clear `RCLCPP_ERROR` naming the cause |
| `on_activate` | activate pubs, create the 100 Hz timer, state → `Streaming` | `FAILURE` |
| `on_deactivate` | **cancel + reset the timer first**, then deactivate pubs, state → `Idle` | — |
| `on_cleanup` | release solver, generator, model, pubs/subs | — |
| `on_shutdown` | stop the timer and publishing. **Do not disarm.** | — |

The timer-first ordering in `on_deactivate` is not stylistic: a timer callback that runs
against a deactivated publisher is undefined behaviour.

`on_shutdown` does not disarm because PX4's own offboard-loss failsafe is the safer authority
and is already configured (`COM_OF_LOSS_T`, `COM_OBL_RC_ACT`). A companion computer deciding
to disarm a flying vehicle is how you break a frame.

### 7.2 QoS

PX4 publishes with a specific profile; a mismatched subscription silently receives nothing:

```cpp
rclcpp::QoS px4_qos(rclcpp::KeepLast(5));
px4_qos.best_effort().durability_volatile();
```

Use it for **every** `/fmu/out/*` subscription and `/fmu/in/*` publisher. If topics look dead,
check this before anything else.

### 7.3 State machine

| From | To | Condition |
| --- | --- | --- |
| `Idle` | `Streaming` | node activated |
| `Streaming` | `Takeoff` | armed ∧ offboard ∧ ≥ 20 OffboardControlMode published ∧ state fresh |
| `Takeoff` | `Tracking` | `|z − takeoff_altitude| < 0.1 m` ∧ `‖v‖ < 0.2 m/s` |
| `Tracking` | `Landing` | landing requested |
| `Landing` | `Idle` | `z < 0.15 m` ∧ `‖v‖ < 0.2 m/s` → disarm if `auto_arm` |
| any | `Failsafe` | state stale, or `consecutive_failures ≥ max` |
| `Failsafe` | `Streaming` | state fresh again ∧ one solve succeeded |
| `Tracking` | `Streaming` | offboard lost (pilot took over) — log at ERROR |

`Streaming` exists because PX4 refuses to enter offboard mode unless setpoints are already
arriving. Publish hover setpoints there.

### 7.4 Control loop — fixed ordering

```
 1. now = get_clock()->now();  measure the period since the last tick
 2. if (!assembleState(&x0, &why))            → enterFailsafe(why); publishOffboardControlMode(); return
 3. updateControllerState()
 4. x0 = compensateLatency(x0, latency_compensation_s_)
 5. refs = trajectory_->referenceHorizon(t_traj, dt, N, airframe_, rep)
 6. solver_->setInitialState(x0); solver_->setReferenceHorizon(...); solver_->setParameters(p)
 7. result = solver_->solve()
 8. if (result.ok())  u0 = solver_->optimalInput();  last_applied_input_ = u0
    else              hold last_applied_input_  (and count the failure)
 9. pub_attitude_setpoint_->publish(toAttitudeSetpoint(u0, solver_->predictedState(1)))
10. publishOffboardControlMode()      ← EVERY tick, unconditionally, including in Failsafe
11. publishStatus(result, x0); publishVisualisation()
```

Budget: 10 ms total. Warn (throttled, 1 Hz) above `log_solve_time_warn_ms`.
**No dynamic allocation after the first tick** — pre-size every buffer in `on_configure`.
`publishVisualisation` is skipped when there are no subscribers.

### 7.5 Message conventions

- **Timestamps to PX4 are microseconds:** `get_clock()->now().nanoseconds() / 1000`.
  Nanoseconds here means PX4 silently discards your setpoints.
- `VehicleAttitude::q` is `(w, x, y, z)` in NED/FRD.
- `VehicleCommand` requires `target_system = 1`, `target_component = 1`, `source_system = 1`,
  `source_component = 1`, `from_external = true`. Missing `from_external` = ignored command.
- Offboard mode: `VEHICLE_CMD_DO_SET_MODE` (176) with `param1 = 1`, `param2 = 6`.
  Arm: `VEHICLE_CMD_COMPONENT_ARM_DISARM` (400) with `param1 = 1`.
- Reject `VehicleLocalPosition` unless `xy_valid ∧ z_valid ∧ v_xy_valid ∧ v_z_valid`.

> `VehicleAttitudeSetpoint` field names changed across PX4 releases (the `roll_body` /
> `pitch_body` / `yaw_body` members were removed). **Read the message definition in the pinned
> `px4_msgs`** rather than trusting any example, including this one.

### 7.6 Attitude setpoint

```cpp
model_->allocate(u0, &T, &tau);                      // collective thrust [N]
q_d_enu = quaternion slice of x_pred_1;              // the optimiser's intended attitude
msg.q_d = quatEnuFluToNedFrd(q_d_enu);               // (w, x, y, z)
msg.thrust_body = {0.f, 0.f, -normaliseThrust(T)};   // NEGATIVE z — PX4 is FRD
msg.yaw_sp_move_rate = predicted yaw rate;
msg.timestamp = now_us;
```

Taking the attitude from the **predicted state at stage 1** rather than reconstructing it from
`u0` is deliberate: it is what the optimiser actually intends one step ahead, and it already
accounts for the rate dynamics.

`normaliseThrust`: `u = px4_hover_thrust · T/(m·g)`, clamped to `[0.05, 0.95]`. This is correct
only when `THR_MDL_FAC = 0`. If the frame sets it non-zero, invert PX4's quadratic instead —
and cross-check against `config/px4_overrides.yaml`, which must agree with
`nmpc_params.yaml:px4_hover_thrust`.

### 7.7 Latency compensation

`x0 ← model_->step(x0, last_applied_input_, latency_s)`, skipped on the first tick and when
`latency_s <= 0`. `latency_compensation_s` is the **measured** sensor-to-actuation delay
(procedure in `TUNING_GUIDE.md` §5), not a guess. Over-compensating is destabilising — start
at 0 and increase.

### 7.8 `main.cpp`

`rclcpp::init` → `NodeOptions().use_intra_process_comms(true)` → `MultiThreadedExecutor` with
2 threads → add `node->get_node_base_interface()` → spin → shutdown.

Optional `--rt-priority N`: attempt `SCHED_FIFO` on the control thread; if it fails (no
`CAP_SYS_NICE`), log a warning and **continue**. Never make real-time scheduling a hard
requirement for the demo to run.

---

## §8 · Launch files

### 8.1 `nmpc_only.launch.py`

Arguments: `airframe` (`x500`|`crazyflie21`), `nmpc_config`, `trajectory_config`, `namespace`,
`auto_arm` (false), `auto_activate` (true), `log_level` (info), `use_sim_time` (false).

Lifecycle sequencing MUST use events, not timers:

```python
EmitEvent(ChangeState(… TRANSITION_CONFIGURE))                     # on ProcessStart
RegisterEventHandler(OnStateTransition(goal_state="inactive",
                                       entities=[EmitEvent(ChangeState(… TRANSITION_ACTIVATE))]))
```

A `TimerAction` that "usually works" is a race, and a reviewer reading the launch file will
recognise which one you wrote.

### 8.2 `sitl.launch.py`

Order: PX4 SITL+Gazebo → uXRCE agent → NMPC → RViz.

- PX4 via `ExecuteProcess` with `additional_env = {PX4_SYS_AUTOSTART: "4001",
  PX4_GZ_MODEL: model, PX4_GZ_WORLD: world, HEADLESS: …}` and
  `cwd = <px4_dir>/build/px4_sitl_default`.
- If `px4_dir` does not exist, fail immediately with the exact `git clone` + `make` commands
  in the message. This is the file that has to work for a stranger.
- Agent: `MicroXRCEAgent udp4 -p 8888`, started after PX4 (event handler where possible).
- NMPC included after a bounded delay; document why the delay exists.
- **Apply `config/px4_overrides.yaml`.** Write the `param set` lines into the PX4 startup
  extras file before launching (`$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/px4-rc.simulator`
  or the documented equivalent for the pinned version). Skipping this silently invalidates the
  thrust map.
- `on_shutdown` handler that SIGINTs PX4 and the agent — orphaned `px4` processes hold port
  8888 and the next launch fails mysteriously.

### 8.3 `figure8.launch.py`

Includes 8.2 with `trajectory:=hover, auto_arm:=true`, then commands the figure-8 once the
controller reports `STATE_TRACKING` on `~/status` (with a bounded timer as a fallback), and
optionally records the bag listed in the file's docstring.

### 8.4 `hardware.launch.py`

`auto_arm` defaults **false** and `auto_activate` defaults **false**. These defaults are a
safety property; do not "improve the UX" by flipping them.

Crazyflie adapter contract: subscribe `/nmpc_node/status`, take `attitude_setpoint` +
`collective_thrust_newton`, convert to the `crazyflie_ros2` setpoint (roll/pitch in degrees,
yaw rate in deg/s, thrust as a 16-bit PWM via the calibrated map in
`params/crazyflie21_calibration.yaml:hover_pwm`), publish at 100 Hz, and **latch to zero
thrust if the NMPC status goes stale for 100 ms**.

---

## §9 · Configuration

### 9.1 Parameter declaration

Every key in `config/nmpc_params.yaml` is declared with a `ParameterDescriptor` carrying a
description, a `FloatingPointRange`/`IntegerRange` where meaningful, and `read_only = true`
for the structural ones (`horizon_steps`, `horizon_time`, `attitude_rep`, `airframe_params_path`).

`onParameterUpdate` accepts weights and trajectory parameters live; it rejects structural
changes with `result.reason = "horizon_steps is baked into the generated solver; re-run
codegen and restart"`. A rejection with a useful reason is worth ten with "invalid parameter".

Validate: `control_rate_hz > 0`; `state_timeout_s >= 2/control_rate_hz`;
`q_diag.size() == 12`, `r_diag.size() == 4`, `q_terminal_diag.size() == 12`; all weights ≥ 0;
`q_terminal_diag[i] >= q_diag[i]` (warn, do not reject).

---

## §10 · Tests

### 10.1 `test_nmpc_solve_time.cpp`

- Release build only. Warm up with 100 discarded solves before timing anything — first-touch
  page faults and CPU frequency ramp otherwise contaminate the left tail.
- Measure with `std::chrono::steady_clock` around `solve()`, **not** acados' self-report.
- Report min/median/mean/p90/p95/p99/p99.9/max via `RecordProperty`, and assert on the p99.
- Thresholds: hover and figure-8 → median ≤ 1 ms, p99 ≤ 2 ms. Cold start → p99 ≤ 10 ms.
- The hash test compares `AcadosWrapper::generatedModelHash()` against `codegen/MODEL_HASH`
  and prints the regeneration command on failure.

> On shared GitHub runners these timings are noisy. Either relax the CI threshold and keep the
> strict one for a self-hosted job, or mark the assertion informational **in the workflow** —
> whichever you choose, write the decision into `colcon_build.yml`. Do not disable the test.

### 10.2 `test_trajectory_continuity.cpp`

Analytic vs finite-difference derivatives (rel. tol 1e-5); ramp-in C⁴; minimum-snap C⁴ and
waypoint interpolation to 1e-9; the flatness/dynamics consistency check of §5.5; and the
infeasible-trajectory case (3 m at 1.5 s on the Crazyflie must be *reported*, not accepted).

### 10.3 `test_acados_codegen.py`

The model-agreement test needs C++ values from Python. Use the **helper-binary** route: build
`dynamics_probe` under `BUILD_TESTING`, which reads `(x, u)` rows on stdin and writes `f(x,u)`
rows on stdout. Simpler than a pybind module and it also gives you a manual debugging tool.

Determinism: two codegen runs into two temp dirs must be byte-identical. If they are not, fix
the source of non-determinism (`strip_nondeterminism` in §11.2) rather than loosening the test.

---

## §11 · Codegen

### 11.1 `quadrotor_model.py`

The CasADi model MUST be the same equations as §4.2. `AirframeConstants.from_yaml()` reads the
same `params/*_calibration.yaml` the C++ reads — one source of truth, no transcribed numbers.

`model.name = "quadrotor"` is load-bearing: the generated symbols, the header name
`acados_solver_quadrotor.h`, and the capsule type all derive from it.

`model_hash()` must be deterministic across runs and machines: hash the `str()` of the SX
graph plus the constants dict serialised with sorted keys and fixed-precision float formatting
(`f"{v:.17g}"`). No `id()`, no unsorted dict iteration, no `repr()` of floats.

### 11.2 `generate_acados_solver.py`

`--check-only` regenerates into a temp dir, diffs against the committed tree, and exits
non-zero on any difference with a readable summary. That is the mode CI runs.

Emits `codegen/MODEL_HASH` (bare digest), `codegen_output/include/model_hash.h`
(`#define UAV_MPC_MODEL_HASH "…"`), and `GENERATION_PROVENANCE.txt` (acados commit,
CasADi version, input file hashes — **no timestamps**, they break determinism).

`strip_nondeterminism()` rewrites acados' generation-date lines and MUST assert that its
substitution matched, so a future acados that stops emitting them fails loudly instead of
silently no-oping.

---

## §12 · Analysis

### 12.1 `solve_time_benchmark.py`

Report **both** the synthetic and the from-bag numbers in the README. Publishing only the
clean synthetic figure is the kind of thing a reviewer catches, and it costs more credibility
than the slower honest number ever would.

The plot title MUST name the CPU, the horizon, and the acados commit. A solve-time histogram
without the hardware named conveys nothing.

Exit non-zero when the p99 exceeds budget, so the script doubles as a CI gate.

### 12.2 `tracking_error_analysis.ipynb`

Plot the reference the controller was actually given, not the ideal analytic curve. State the
disturbance condition in every title. Never crop a transient without saying so in the caption.

The PX4 and ROS clocks differ — pick one time base explicitly, align with `merge_asof`, and
write down which you chose.

### 12.3 `disturbance_sweep.ipynb`

Grid: wind ∈ {0,2,4,6,8} m/s × direction ∈ {0,45,90}° × gust ∈ {off,on} × 3 seeds. Cache every
run; a sweep that cannot resume is a sweep you will run once and never again. Report mean ± std
across repeats. Include the **failure boundary** plot — where the controller breaks is more
interesting than where it works.

---

## §13 · Documentation

| File | Requirement |
| --- | --- |
| §13.1 `quadrotor_se3_dynamics.tex` | Full Newton–Euler derivation, symbol table with units, quad-X allocation figure, hover linearisation justifying `dt` and `Tf`, geometric-control baseline |
| §13.2 `differential_flatness.tex` | The map with **all intermediate algebra**, degenerate cases, the feasibility inequality worked for the 3 m/5 s figure-8 on the X500, and an equation → code-function table |
| §13.3 `nmpc_formulation.tex` | The OCP as generated, the Bryson weight table, the RTI statement **with its hypotheses**, and a section on what this formulation does *not* guarantee |
| §13.4 `TUNING_GUIDE.md` | The tuning order, and the failure table — extend it with every failure you actually hit |
| §13.5 `HARDWARE_BRINGUP.md` | Status line per section (`written`/`bench-tested`/`flown`), kept accurate |

On §13.3's last section and §13.5's status lines: stating the limits of your own work is not a
weakness in this context, it is the signal. Claiming stability you have not established, or
implying flights you have not flown, is the one failure mode this repository cannot recover
from.

---

## §14 · CI

### 14.1 `colcon_build.yml`

Cache pip, the acados build, and `build/`+`install/`, keyed on the lockfiles and
`codegen/ACADOS_COMMIT`. Run `generate_acados_solver.py --check-only` **before** the build so
stale generated code fails rather than being silently regenerated. `-DCMAKE_BUILD_TYPE=Release`
is mandatory. Upload test XML and the percentile JSON as artifacts.

The `build-no-acados` job exists to catch an acados header leaking out of the wrapper. If it
starts failing, fix the leak — do not add the include path to that job.

### 14.2 `format_check.yml`

Under two minutes. Put flake8 settings in a committed `setup.cfg`, never inline flags, or the
local and CI runs will drift.

### 14.3 `docker_smoke_test.yml`

The differentiating job. Build the sim image in a separate manually-triggered workflow, publish
to GHCR, and have this job only pull — building PX4 on every push makes this 40 minutes and
nobody will wait.

Assertions live in `scripts/assert_hover.py` so they can be run locally:
`STATE_TRACKING` within 20 s; RMS error < 0.15 m; max error < 0.30 m; zero solver failures;
p99 solve < 5 ms (relaxed — a CI runner is not a flight computer); altitude never below half
the target. Print every metric even on success. Upload the bag, the ulog and the node logs
with `if: always()`.

Budget real effort for determinism here. A smoke test that fails 30% of the time trains
everyone to ignore red, which is worse than not having one.

---

## §15 · Implementation order

Each milestone is independently verifiable. Do not start one before the previous one passes.

| # | Milestone | Files | Done when |
| --- | --- | --- | --- |
| M1 | Frames and dynamics | `quadrotor_dynamics.*`, `test_frame_conversions.cpp`, `test_dynamics.cpp` | round-trips exact, Jacobians match finite differences |
| M2 | Symbolic model | `codegen/quadrotor_model.py`, `dynamics_probe` | CasADi and C++ agree to 1e-9 |
| M3 | Solver generation | `generate_acados_solver.py` | deterministic output, hash written, hover solves in Python |
| M4 | Trajectories | `trajectory_generator.*`, `test_trajectory_continuity.cpp` | C⁴ everywhere, flatness consistent with the dynamics |
| M5 | Solver wrapper | `acados_wrapper.cpp`, `test_nmpc_solve_time.cpp` | A2 met |
| M6 | Node | `nmpc_node.cpp`, `main.cpp` | hovers in SITL |
| M7 | Launch | all four launch files | one-command SITL flight |
| M8 | CI | three workflows | A1, A6, A9 green |
| M9 | Analysis + media | `analysis/`, `media/` | README numbers and GIFs exist |
| M10 | Docs | `docs/` | derivations complete and matching the code |

M1–M3 need no ROS at all. M1–M5 need no PX4. Get as far as M5 before fighting a simulator.

---

## §16 · Conventions and known traps

**Frames.** World ENU, body FLU, everywhere except at the PX4 message boundary. Every
conversion lives in `quadrotor_dynamics.cpp`. If you find yourself writing a sign flip
anywhere else, you have found a bug, not a shortcut.

**Quaternions.** Hamilton, `(w, x, y, z)`, body-to-world. Eigen's `Quaterniond` constructor
takes `(w, x, y, z)` but its `.coeffs()` returns `(x, y, z, w)`. This mismatch has cost more
quadrotor-hours than any other single line of code. Never `memcpy` a quaternion.

**Units.** SI throughout. Thrust in newtons in every internal interface; normalisation to
PX4's `[0,1]` happens once, in `normaliseThrust()`.

**Time.** `rclcpp::Time` internally; microseconds at the PX4 boundary; seconds in trajectory
parameters. Trajectory time is measured from `trajectory_start_time_`, not wall clock.

**Angles.** Radians internally. Wrap yaw errors to `[−π, π]`; unwrap yaw *references* across
samples. These are different operations and both are needed.

**Logging.** Nothing unthrottled in the control loop. `RCLCPP_*_THROTTLE` with a 1 s period.

**Allocation.** Nothing dynamic in the control loop after the first tick. Reserve in
`on_configure`.

**The single most likely failure** when you first run in SITL: no messages arriving, because
the QoS is wrong (§7.2) or `px4_msgs` does not match PX4 (§2). Check those two before
debugging anything else.

---

## §17 · Definition of done

A file is done when: every `TODO(deepseek)` in it is either implemented and deleted, or
converted into a specific issue with a reason; it builds with `-Wall -Wextra -Wpedantic
-Wconversion` clean; its tests pass; and any behaviour a reader would not predict from the
signature is documented in a comment.

The repository is done when all of §1's acceptance criteria hold, the README's numbers are
reproducible from `analysis/` on the hardware named in the plot titles, and
`docs/HARDWARE_BRINGUP.md` accurately states what has and has not been flown.
