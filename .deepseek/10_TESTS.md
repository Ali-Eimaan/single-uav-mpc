# §10 · Tests

**Governs:** everything under `uav_mpc/test/`
**Milestone:** written alongside each milestone, never afterwards (rule 5 in
[00_RULES.md](00_RULES.md))

Existing skeletons: `test_nmpc_solve_time.cpp`, `test_trajectory_continuity.cpp`,
`test_acados_codegen.py`. Two more are required and do not exist yet: `test_dynamics.cpp` and
`test_frame_conversions.cpp` (both at M1, see [04_DYNAMICS.md](04_DYNAMICS.md)).

---

## 10.1 `test_nmpc_solve_time.cpp`

This is the gate test for criterion A2 — the single quantitative claim the repository makes.
Keep it honest:

- Release build only. Warm up with 100 discarded solves before timing anything — first-touch
  page faults and CPU frequency ramp otherwise contaminate the left tail.
- Measure with `std::chrono::steady_clock` around `solve()`, **not** acados' self-report.
- Report min/median/mean/p90/p95/p99/p99.9/max via `RecordProperty`, and assert on the **p99**,
  not the mean — a 100 Hz loop that misses once per second is broken.
- Thresholds: hover and figure-8 → median ≤ 1 ms, p99 ≤ 2 ms. Cold start → p99 ≤ 10 ms.
- Advance the reference by `dt` each sample so the warm start sees a genuinely moving target.
- The hash test compares `AcadosWrapper::generatedModelHash()` against `codegen/MODEL_HASH` and
  prints the regeneration command on failure.

> On shared GitHub runners these timings are noisy. Either relax the CI threshold and keep the
> strict one for a self-hosted job, or mark the assertion informational **in the workflow** —
> whichever you choose, write the decision into `colcon_build.yml`. Do not disable the test.

Path resolution for the airframe YAML: use `ament_index_cpp` or a compile definition set in
`CMakeLists.txt`. Never hard-code an absolute path.

## 10.2 `test_trajectory_continuity.cpp`

Covers criterion A5:

- analytic vs finite-difference derivatives, figure-8 and lemniscate (rel. tol 1e-5)
- ramp-in is C⁴ at both boundaries
- minimum-snap segments C⁴, waypoints interpolated to 1e-9
- supplied `segment_times` respected
- **the flatness/dynamics consistency check** of [05_TRAJECTORY.md §5.5](05_TRAJECTORY.md) —
  this is the test that catches a wrong allocation matrix or a swapped body axis
- an infeasible trajectory (3 m at 1.5 s on the Crazyflie) is *reported*, not silently accepted

## 10.3 `test_acados_codegen.py`

The model-agreement test (criterion A4) needs C++ values from Python. Use the **helper-binary**
route: build `dynamics_probe` under `BUILD_TESTING`, which reads `(x, u)` rows on stdin and
writes `f(x,u)` rows on stdout. Simpler than a pybind module, and it doubles as a manual
debugging tool.

Determinism: two codegen runs into two temp dirs must be byte-identical. If they are not, fix
the source of non-determinism (`strip_nondeterminism`, see
[11_CODEGEN.md §11.2](11_CODEGEN.md)) rather than loosening the test.

Also asserts: OCP dimensions match `nmpc_params.yaml`; the solver options are the real-time set
from [06_SOLVER.md §6.2](06_SOLVER.md); and (marked `slow`) the generated solver holds a hover
in a closed-loop Python simulation.

## 10.4 New tests you must add

| File | Milestone | Asserts |
| --- | --- | --- |
| `test_frame_conversions.cpp` | M1 | ENU↔NED and FLU↔FRD round-trip to 1e-12 over 1000 random samples; three hand-computed cases (level, 90° yaw, 30° roll) |
| `test_dynamics.cpp` | M1 | analytic Jacobians vs central differences (step 1e-6, rel. tol 1e-6); RK4 energy sanity in the drag-free, torque-free case; quaternion norm preserved after `step()` |

Register both in `CMakeLists.txt` with `ament_add_gtest` and link `uav_mpc_core`.

## 10.5 SITL assertions

`scripts/assert_hover.py` holds the criterion A6/A7 thresholds and is called by
`docker_smoke_test.yml`. It lives in `scripts/` rather than inline in the workflow so it can be
run locally against a bag from a failed CI run. See [14_CI.md §14.3](14_CI.md).
