# §5 · `trajectory_generator.{hpp,cpp}`

**Governs:** `uav_mpc/include/uav_mpc/trajectory_generator.hpp`,
`uav_mpc/src/trajectory_generator.cpp`
**Prerequisites:** [04_DYNAMICS.md](04_DYNAMICS.md) (the flatness map needs the allocation)
**Milestone:** M4
**Done when:** references are C⁴ everywhere and the flatness map is consistent with the
dynamics — `test_trajectory_continuity.cpp` green.

---

## 5.1 Analytic primitives

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

Guard `ẋ² + ẏ² < 1e-6` by holding the previous yaw. **Unwrap** `ψ` across samples — a jump from
`+π` to `−π` becomes a commanded 360°/dt yaw rate.

## 5.2 Ramp-in

`rampScale(t, T_ramp, k)` returns the `k`-th time derivative of `S₄(t/T_ramp)`, where `S₄` is
the degree-9 smoothstep — the unique 9th-order polynomial with `S₄(0)=0`, `S₄(1)=1`, and
derivatives 1–4 vanishing at both ends:

```
S₄(u) = 70u⁹ − 315u⁸ + 540u⁷ − 420u⁶ + 126u⁵
```

Verify `S₄(0)=0`, `S₄(1)=1`, and `S₄⁽ᵏ⁾(0) = S₄⁽ᵏ⁾(1) = 0` for `k = 1..4` in a unit test rather
than trusting the coefficients printed here.

The blend is `σ(t) = σ_hover + S₄(t/T)·(σ_traj(t) − σ_hover)`; the derivatives follow from the
**product rule**, so `sample()` needs `S₄⁽ᵏ⁾` for `k = 0..4`. Forgetting this is the single most
common bug in this file — it produces a reference that looks smooth in position and has a step
in jerk.

## 5.3 Minimum snap

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
log which was used). Waypoint counts are small (< 30), so dense is fine — say so in a comment so
a reader knows it was a decision.

Chain rule when evaluating: `dᵏp/dtᵏ = (dᵏp/dτᵏ)/Tᵏ`. Getting this wrong scales your velocities
by the segment duration and is invisible for `T = 1 s`.

## 5.4 Time allocation

When `segment_times` is empty, for each segment of length `dᵢ`:

```
Tᵢ = 1.2 · max( dᵢ/v_max , √(2dᵢ/a_max) )
```

Floor at 0.1 s. The 1.2 factor is slack for the cornering the straight-line estimate ignores.

## 5.5 Differential flatness map

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
projection formulas above have a sign error, this test finds it in seconds; SITL would take you
a day.

## 5.6 Hot-path note

`sampleHorizon()` and `referenceHorizon()` are called every control tick. `reserve()` the output
vectors; do not allocate per sample. The node pre-sizes its buffers in `on_configure` — see
[07_NODE.md §7.4](07_NODE.md).
