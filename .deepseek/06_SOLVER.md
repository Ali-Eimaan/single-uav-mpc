# §6 · `acados_wrapper.{hpp,cpp}`

**Governs:** `uav_mpc/include/uav_mpc/acados_wrapper.hpp`, `uav_mpc/src/acados_wrapper.cpp`
**Prerequisites:** [11_CODEGEN.md](11_CODEGEN.md) — the solver must exist before you wrap it
**Milestone:** M5
**Done when:** acceptance criterion A2 is met (p99 < 2 ms, median < 1 ms, Release build).

---

## 6.1 Boundary rule

`acados_wrapper.cpp` is the **only** file that may include an acados header. Everything acados
touches is behind the PIMPL (`struct AcadosWrapper::Impl`). The `build-no-acados` CI job
enforces this. When you need something from acados elsewhere, widen the wrapper's interface
rather than including the header.

Guard every acados include with `#ifdef UAV_MPC_WITH_ACADOS`. In the stub build,
`initialise()` fills `nx_`/`nu_` from the config and returns `true`, so unit tests that never
solve still link.

## 6.2 Solver configuration (all of these are decisions — do not "improve" them casually)

| Setting | Value | Why |
| --- | --- | --- |
| `N` | 20 | with `Tf = 1 s` gives `dt = 50 ms` |
| `Tf` | 1.0 s | ≈ one figure-8 quarter-lap at the demo speed |
| `nlp_solver_type` | `SQP_RTI` | real-time iteration; one QP per control tick |
| `nlp_solver_max_iter` | 1 | that is what RTI means |
| `qp_solver` | `PARTIAL_CONDENSING_HPIPM` | best measured fit at this horizon |
| `qp_solver_cond_N` | 5 | condensing block size; sweep it (see [12_ANALYSIS.md](12_ANALYSIS.md)) and record the result |
| `hessian_approx` | `GAUSS_NEWTON` | least-squares cost; exact Hessian is not worth the time |
| `integrator_type` | `ERK`, 4 stages, 1 step | explicit is enough for a non-stiff quadrotor at 50 ms |
| `qp_solver_iter_max` | 50 | bounded, so a bad QP cannot blow the budget |
| `globalization` | `FIXED_STEP` | line search costs time RTI does not have |
| `hpipm_mode` | `SPEED` | |
| `print_level` | 0 | printing from a 100 Hz loop is a latency bug |

## 6.3 Cost

`NONLINEAR_LS`, because the attitude error is not affine in the state.

```
y   = [ p(3), v(3), vec(q_ref⁻¹ ⊗ q)(3), ω(3), u(4) ]     ny   = 16
y_e = [ p(3), v(3), vec(q_ref⁻¹ ⊗ q)(3), ω(3) ]           ny_e = 12
```

`q_ref` therefore enters through the **online parameter vector**, not `yref`:
`p = [wind(3), mass_scale, q_ref(4)]`, `np = 8`. `yref` carries the position/velocity/rate
reference and `u_ref`, with the attitude-error block held at zero.

Because the residual is 12-dimensional on the state side, the weight vectors in
`config/nmpc_params.yaml` are **12 entries, not `nx`**:
`[p(3) | v(3) | attitude-error(3) | ω(3)]`. `W` is 16×16, `W_e` is 12×12.

Sign convention: take the vector part of the error quaternion, negated if `w < 0`, so the cost
always drives along the shortest arc. Without this you get the unwinding phenomenon — the
vehicle takes the long way round a large yaw error, which looks spectacular and is a bug.

> Rejected alternative, recorded so it is not re-litigated: `LINEAR_LS` weighting the raw
> quaternion components. It penalises the wrong quantity near large attitude errors and cannot
> express the shortest-arc convention. Do not switch to it for the sake of a faster solve.

`setWeights()` validates that the diagonals are non-negative and that `Q_N ≥ Q` elementwise — a
terminal weight smaller than the stage weight is the classic cause of horizon-end drift.

## 6.4 Constraints

- **Hard**, per rotor: `lbu = min_thrust_per_rotor`, `ubu = max_thrust_per_rotor`,
  `idxbu = [0,1,2,3]`. Always feasible by construction, so these cannot cause infeasibility.
- **Soft**, body rates: `|ω| ≤ 6 rad/s` with an L2 slack penalty (`Zl = Zu = 1e2`,
  `zl = zu = 1e1`). Hard state constraints under RTI go infeasible in flight the first time a
  disturbance pushes you outside the set, and then you are on the recovery path at 100 Hz.
- `x₀` via the initial-state equality (`lbx_0 = ubx_0 = x₀`), set every tick.
- No position constraints in v0.1. CBF-based ones are the thesis contribution and belong in the
  downstream repository.

## 6.5 Failure recovery

`solve()` returns, never throws. On a non-zero acados status:

1. **First failure:** `resetToHover(x₀, hoverThrustPerRotor())`, solve exactly once more, set
   `reinitialised = true`. If the retry succeeds, report `Success` with the flag set.
2. **Failures 2..max−1:** report the status; the node holds the previous input.
3. **`consecutive_failures >= max_consecutive_failures`:** report; the node enters `Failsafe`.

A `Success` resets the counter to zero. `solve()` MUST also enforce `solve_time_budget_ms` — if
the measured wall time exceeds it, report `Timeout` even when acados returned success, so the
budget breach shows up in the logs rather than as a missed deadline you never see.

## 6.6 Hot-path rules

No allocation inside `solve()`, `setStageReference()`, `setInitialState()`, or
`optimalInput()`. All scratch buffers are sized once in `initialise()`. Use
`Eigen::Map<Eigen::VectorXd>` over the existing buffers rather than constructing vectors.

## 6.7 Initialisation checks

`initialise()` returns `false` with a filled `*error` (it does not throw, so the lifecycle node
can report `FAILURE` from `on_configure`). It MUST hard-fail when:

- `config.horizon_steps != QUADROTOR_N` or `|Tf − codegen Tf| > 1e-9`
- `generatedModelHash() != ` the digest in `codegen/MODEL_HASH`

The hash check is what stops a stale solver being flown against a changed model. Do not make it
a warning.
