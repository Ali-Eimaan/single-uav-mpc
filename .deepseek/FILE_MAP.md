# File map

Every skeleton file in the repository, and the document that specifies it. Use this when you
have a file open and need its spec.

---

## ROS 2 package — `uav_mpc/`

| File | Spec | Milestone |
| --- | --- | --- |
| `CMakeLists.txt` | [03_BUILD_SYSTEM.md](03_BUILD_SYSTEM.md) | throughout |
| `package.xml` | [03_BUILD_SYSTEM.md §3.5](03_BUILD_SYSTEM.md) | throughout |
| `include/uav_mpc/quadrotor_dynamics.hpp` | [04_DYNAMICS.md](04_DYNAMICS.md) | M1 |
| `src/quadrotor_dynamics.cpp` | [04_DYNAMICS.md](04_DYNAMICS.md) | M1 |
| `include/uav_mpc/trajectory_generator.hpp` | [05_TRAJECTORY.md](05_TRAJECTORY.md) | M4 |
| `src/trajectory_generator.cpp` | [05_TRAJECTORY.md](05_TRAJECTORY.md) | M4 |
| `include/uav_mpc/acados_wrapper.hpp` | [06_SOLVER.md](06_SOLVER.md) | M5 |
| `src/acados_wrapper.cpp` | [06_SOLVER.md](06_SOLVER.md) | M5 |
| `include/uav_mpc/nmpc_node.hpp` | [07_NODE.md](07_NODE.md) | M6 |
| `include/uav_mpc/vehicle_interface.hpp` | [07_NODE.md §7.10](07_NODE.md) | M6 |
| `src/vehicle_interface.cpp` (generic backend + factory) | [07_NODE.md §7.10.1](07_NODE.md) | M6 |
| `src/vehicle_interface_px4.cpp` (optional PX4 backend) | [07_NODE.md §7.11](07_NODE.md) | M6 |
| `src/nmpc_node.cpp` | [07_NODE.md](07_NODE.md) | M6 |
| `src/main.cpp` | [07_NODE.md §7.8](07_NODE.md) | M6 |

### Interfaces (no TODOs — these are the contract, do not change them)

| File | Consumed by |
| --- | --- |
| `msg/SolverDiagnostics.msg` | `NmpcStatus`, `assert_hover.py`, both notebooks |
| `msg/TrajectorySpec.msg` | `SetTrajectory.srv`, `FollowTrajectory.action` |
| `msg/NmpcStatus.msg` | `~/status`, all analysis |
| `msg/AttitudeThrustSetpoint.msg` | generic backend output; any downstream adapter |
| `srv/SetTrajectory.srv` | `figure8.launch.py`, runtime trajectory swap |
| `action/FollowTrajectory.action` | not wired in v0.1; generated for downstream use |

### Launch — [08_LAUNCH.md](08_LAUNCH.md), M7

| File | Section |
| --- | --- |
| `launch/nmpc_only.launch.py` | §8.1 |
| `launch/sitl.launch.py` | §8.2 |
| `launch/figure8.launch.py` | §8.3 |
| `launch/hardware.launch.py` | §8.4 |

### Configuration — [09_CONFIG.md](09_CONFIG.md), M6

| File | Section |
| --- | --- |
| `config/nmpc_params.yaml` | §9.1 |
| `config/trajectory_params.yaml` | §9.4 |
| `config/px4_overrides.yaml` | §9.3 |
| `params/x500_calibration.yaml` | §9.2 |
| `params/crazyflie21_calibration.yaml` | §9.2 |
| `rviz/nmpc.rviz` | [12_ANALYSIS.md §12.5](12_ANALYSIS.md) — replace with a config saved from RViz, never hand-edited |

### Tests — [10_TESTS.md](10_TESTS.md)

| File | Section | Milestone |
| --- | --- | --- |
| `test/test_nmpc_solve_time.cpp` | §10.1 | M5 |
| `test/test_trajectory_continuity.cpp` | §10.2 | M4 |
| `test/test_acados_codegen.py` | §10.3 | M2–M3 |
| `test/test_frame_conversions.cpp` **(create)** | §10.4 | M1 |
| `test/test_dynamics.cpp` **(create)** | §10.4 | M1 |
| `test/dynamics_probe.cpp` **(create)** | §10.3 | M2 |

## Codegen — [11_CODEGEN.md](11_CODEGEN.md)

| File | Section | Milestone |
| --- | --- | --- |
| `codegen/quadrotor_model.py` | §11.1 | M2 |
| `codegen/generate_acados_solver.py` | §11.2 | M3 |
| `codegen/ACADOS_COMMIT` | §11.3 | M3 |

## Analysis and scripts — [12_ANALYSIS.md](12_ANALYSIS.md), M9

| File | Section |
| --- | --- |
| `analysis/solve_time_benchmark.py` | §12.1 |
| `analysis/tracking_error_analysis.ipynb` | §12.2 |
| `analysis/disturbance_sweep.ipynb` | §12.3 |
| `scripts/assert_hover.py` | §12.4 (used by CI §14.3) |
| `media/README.md` | §12.5 |

## Documentation — [13_DOCS.md](13_DOCS.md), M10

| File | Section |
| --- | --- |
| `docs/DERIVATION.md` | index |
| `docs/derivations/quadrotor_se3_dynamics.tex` | §13.1 |
| `docs/derivations/differential_flatness.tex` | §13.2 |
| `docs/derivations/nmpc_formulation.tex` | §13.3 |
| `docs/TUNING_GUIDE.md` | §13.4 |
| `docs/HARDWARE_BRINGUP.md` | §13.5 |

## CI — [14_CI.md](14_CI.md), M8

| File | Section |
| --- | --- |
| `.github/workflows/colcon_build.yml` | §14.1 |
| `.github/workflows/format_check.yml` | §14.2 |
| `.github/workflows/docker_smoke_test.yml` | §14.3 |
| `docker/Dockerfile.sitl` **(create)** | §14.3 |

## Repository root

| File | Notes |
| --- | --- |
| `README.md` | fill the results table from `analysis/`; add badges only once the workflow is green |
| `requirements.txt` | resolve every `UNVERIFIED` cp314 pin — [02_ENVIRONMENT.md §2.1](02_ENVIRONMENT.md) V7 |
| `setup.cfg` | flake8 + pytest config; do not duplicate its settings into the workflow |
| `.clang-format`, `.gitignore`, `LICENSE` | complete, no work needed |
