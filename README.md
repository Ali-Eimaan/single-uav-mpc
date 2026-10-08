# single-uav-mpc

Real-time nonlinear MPC for a quadrotor on ROS 2 Lyrical Luth, using an acados-generated
SQP-RTI solver. Speaks **standard ROS 2 messages** by default; PX4/uXRCE-DDS is an optional
backend.

[![colcon build](https://github.com/Ali-Eimaan/single-uav-mpc/actions/workflows/colcon_build.yml/badge.svg)](https://github.com/Ali-Eimaan/single-uav-mpc/actions/workflows/colcon_build.yml)
[![format check](https://github.com/Ali-Eimaan/single-uav-mpc/actions/workflows/format_check.yml/badge.svg)](https://github.com/Ali-Eimaan/single-uav-mpc/actions/workflows/format_check.yml)

[![ROS 2 Lyrical Luth](https://img.shields.io/badge/ROS%202-Lyrical%20Luth-22314E?logo=ros&logoColor=white)](https://docs.ros.org/)
[![Ubuntu 26.04](https://img.shields.io/badge/Ubuntu-26.04%20LTS-E95420?logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus&logoColor=white)](https://en.cppreference.com/w/cpp/17)
[![Python 3.14](https://img.shields.io/badge/Python-3.14-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![acados 0.6.0](https://img.shields.io/badge/acados-0.6.0-2E7D32)](https://github.com/acados/acados)
[![Eigen 3.4](https://img.shields.io/badge/Eigen-3.4-8E44AD)](https://eigen.tuxfamily.org/)
[![License: BSD-3-Clause](https://img.shields.io/badge/License-BSD--3--Clause-blue.svg)](LICENSE)
[![version 0.1.0](https://img.shields.io/badge/version-0.1.0-lightgrey)](uav_mpc/package.xml)
[![status: pre-release](https://img.shields.io/badge/status-pre--release-orange)]

> **Status: pre-release (`0.1.0`). Builds clean and passes its full suite on the target
> platform; not yet flown, and not yet runnable in simulation.**
>
> Verified by an actual containerised build (`ros:lyrical-ros-base`, Ubuntu 26.04, GCC 15.2,
> Release): **85 tests, 0 failures, 0 skipped** with acados, and **0 failures** in the
> no-acados stub configuration. Zero compiler warnings under
> `-Wall -Wextra -Wpedantic -Wshadow -Wconversion`. Criterion A2 is met with ~4x margin
> (see Results).
>
> All CI lint and build gates now pass locally in that container: cpplint, uncrustify, black,
> flake8, yamllint and xmllint — six for six.
>
> Known gaps: the airframe constants in `uav_mpc/params/` are still marked `verified: false` —
> traceable to published sources, but not measured. There is no Gazebo bridge yet, so the SITL
> smoke test is manual-trigger-only rather than permanently red, and criteria A6/A7 are openly
> unmet.
>
> Numbers marked *TBM* are unfilled by design rather than estimated. `1.0.0` is tagged when
> every acceptance criterion in `acceptance_criteria.yaml` is green in CI.

---

## What this is

- A nonlinear MPC that tracks aggressive quadrotor trajectories at 100 Hz, solved with an
  acados SQP-RTI solver generated from a CasADi model (`N = 20`, `Tf = 1 s`, `dt = 50 ms`).
- A ROS 2 lifecycle node that is **autopilot-agnostic**: the vehicle interface is a runtime
  choice, so the controller does not depend on `px4_msgs`.
- Minimum-snap and closed-form analytic trajectory generation through the differential-flatness
  map, producing a full state+input reference for the OCP.

State `x = [p(3), v(3), q(4, wxyz), ω(3)]` (world ENU, body FLU), input
`u = [T₁..T₄]` per-rotor thrust in newtons.

## Vehicle backends

`px4_msgs` is **not** a dependency — it is not released for ROS 2 Lyrical Luth. The backend is
selected at runtime with the `vehicle_interface` parameter.

| | `generic` (default) | `px4` |
| --- | --- | --- |
| Requires `px4_msgs` | no | yes, at build time |
| Available on Lyrical Luth | **yes** | not until `px4_msgs` releases |
| Odometry in | `~/odometry` — `nav_msgs/Odometry` | `/fmu/out/vehicle_local_position`, `…/vehicle_attitude`, `…/vehicle_angular_velocity` |
| Command out | `~/attitude_setpoint` — `uav_mpc/AttitudeThrustSetpoint` | `/fmu/in/vehicle_attitude_setpoint` |
| Arm / offboard handshake | none — assumes authority | full `VehicleCommand` handshake |

The generic backend is a pass-through: `nav_msgs/Odometry` carries pose in the world frame and
twist in the body frame, which is exactly what the controller wants once the world frame is ENU
and the body frame FLU (REP-103/105). Anything that publishes odometry — a mocap bridge, an
EKF, a simulator — can drive it, and any adapter can consume the setpoint.

The PX4 backend is complete and retained in `src/vehicle_interface_px4.cpp`; CMake compiles it
automatically if `px4_msgs` is in the workspace. Selecting `vehicle_interface:=px4` without it
fails `on_configure` with instructions rather than degrading silently.

> **Safety note for `generic`:** there is no autopilot to ask for permission, so the controller
> reports itself armed and authorised as soon as it is activated. Whatever consumes
> `~/attitude_setpoint` owns the arming interlock and the kill switch.

## Quick start

Build (no `px4_msgs` needed):

```bash
mkdir -p ~/ws/src && cd ~/ws/src && git clone https://github.com/Ali-Eimaan/single-uav-mpc
```

```bash
python3 -m venv --system-site-packages ~/.venvs/uavmpc && . ~/.venvs/uavmpc/bin/activate && pip install -r ~/ws/src/single-uav-mpc/requirements.txt
```

```bash
export ACADOS_SOURCE_DIR=$HOME/acados && export LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib:$LD_LIBRARY_PATH
```

```bash
cd ~/ws && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && . install/setup.bash
```

Run against any odometry source:

```bash
ros2 launch uav_mpc nmpc_only.launch.py vehicle_interface:=generic airframe:=x500
```

Run the test suite:

```bash
colcon test --packages-select uav_mpc && colcon test-result --verbose --all
```

PX4 SITL — **does not work on Lyrical Luth today.** It needs `px4_msgs` in the workspace, which
has no Lyrical release, so the node will refuse to configure. Tracked as
; a Gazebo bridge for the generic backend is the planned fix.

```bash
ros2 launch uav_mpc sitl.launch.py headless:=false trajectory:=figure8
```

Build without acados — core library, tests and the stub solver backend only
([full guide](docs/ACADOS.md)):

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DUAV_MPC_WITH_ACADOS=OFF
```

## Results

Measured in a `ros:lyrical-ros-base` container (Ubuntu 26.04, GCC 15.2), Release build, real
acados `503364817` — **not** the stub backend: `GeneratedModelHashMatchesCheckedInHash` runs and
passes, which is only possible when the generated solver is linked. 10 000 warm-started solves
per scenario on a 12th Gen Intel Core i5-12500H.

| Metric | Value | Conditions |
| --- | --- | --- |
| Median solve time, hover | **0.218 ms** | N=20, Tf=1 s, acados `503364817`, i5-12500H |
| p99 solve time, hover | **0.469 ms** | budget 2.0 ms (A2) — ~4x margin |
| Median solve time, figure-8 | **0.318 ms** | 3 m amplitude, 5 s period |
| p99 solve time, figure-8 | **0.729 ms** | budget 2.0 ms (A2) |
| p99 solve time, cold start | **0.515 ms** | no warm start, budget 10 ms |
| RMS tracking error, figure-8 | *TBM* | needs a simulator — blocked on  |
| SITL hover RMS error | *TBM* | target < 0.15 m (A6) — blocked on  |

The **max** solve time is 5.27 ms. That exceeds the production `solve_time_budget_ms` of 5.0 ms,
which the wrapper currently stamps as `Timeout` and the node counts toward its failsafe
threshold. Fix that before running under simulator load.

These are synthetic (solver-driven) numbers. The from-flight numbers still need a vehicle.

Regenerate with:

```bash
colcon test --packages-select uav_mpc --ctest-args -R test_nmpc_solve_time
```

## Repository layout

| Path | Contents |
| --- | --- |
| [`uav_mpc/`](uav_mpc/) | The ROS 2 package: node, vehicle backends, dynamics, trajectory generation, solver wrapper |
| [`codegen/`](codegen/) | CasADi model and the acados solver generator |
| [`analysis/`](analysis/) | Benchmarks and the notebooks producing the plots above |
| [`scripts/`](scripts/) | `assert_hover.py` — the SITL acceptance thresholds, runnable locally |
| [`docs/`](docs/) | Derivations (LaTeX), tuning guide, hardware bring-up |
| [`media/`](media/) | README assets (not yet captured) |

## Documentation

- [Derivations](docs/DERIVATION.md) — dynamics, differential flatness, NMPC formulation
- [Tuning guide](docs/TUNING_GUIDE.md) — weight selection and the failure table
- [acados guide](docs/ACADOS.md) — what acados does, building with and without it
- [Hardware bring-up](docs/HARDWARE_BRINGUP.md) — what has and has not been flown
- Implementation specification — the spec this code is built against
- Review — open defects from the R1 sweep

## Requirements

Ubuntu 26.04 LTS · ROS 2 Lyrical Luth · Python 3.14 · Eigen 3.4 · acados (commit pinned in
[`codegen/ACADOS_COMMIT`](codegen/ACADOS_COMMIT)).

Optional: `px4_msgs` + PX4 + Gazebo Jetty, for the PX4 backend and SITL.

> These versions were confirmed by building and testing in a `ros:lyrical-ros-base` container
> (Ubuntu 26.04, GCC 15.2, CMake 4.2.3, Python 3.14.4). `px4_msgs` remains unreleased for
> Lyrical Luth, which is why it is optional.

## Citation

This is the per-agent controller underlying the distributed MPC-CBF work in
`transition-viable-swarm`. A `CITATION.cff` will be added at v1.0.

## License

BSD-3-Clause — see [LICENSE](LICENSE).
