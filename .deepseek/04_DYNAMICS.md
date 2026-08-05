# §4 · `quadrotor_dynamics.{hpp,cpp}`

**Governs:** `uav_mpc/include/uav_mpc/quadrotor_dynamics.hpp`,
`uav_mpc/src/quadrotor_dynamics.cpp`
**Prerequisites:** [16_CONVENTIONS.md](16_CONVENTIONS.md)
**Milestone:** M1 — the first thing you implement. Needs no ROS, no PX4, no acados.
**Done when:** frame round-trips are exact and the analytic Jacobians match central differences.

---

## 4.1 `QuadrotorParams::fromYaml`

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

## 4.2 Continuous dynamics

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

Branch on the representation with `if constexpr (Rep == AttitudeRep::Quaternion)`, not a
runtime check — the layout constants come from `StateLayout<Rep>` and must fold away.

## 4.3 Control allocation (PX4 quad-X)

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
> "the MPC is unstable" — you will debug the controller for a day before suspecting the mixer.

## 4.4 Frame conversions

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

**Test these first.** Add `test/test_frame_conversions.cpp`: round-trip 1000 random quaternions
and vectors to 1e-12, and check three hand-computed cases (level, 90° yaw, 30° roll). Frame bugs
are the most expensive bugs in this repository and the cheapest to test.

## 4.5 Jacobians

Analytic `A = ∂f/∂x`, `B = ∂f/∂u`. Verify against central differences with step `1e-6` and
relative tolerance `1e-6` in `test/test_dynamics.cpp`. These are not used by acados (which has
its own AD) — they exist so the model can be checked without acados present.

## 4.6 Explicit instantiations

The header declares `extern template` for `<double, Quaternion>` and `<double, Euler>`; the
`.cpp` defines them, along with the four free quaternion helpers. Adding a new instantiation
means adding it in both places or the link fails with an unhelpful message.
