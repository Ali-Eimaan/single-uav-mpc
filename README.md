# uav-mpc

Real-time nonlinear MPC for a quadrotor on ROS 2 Lyrical Luth, using an acados-generated
SQP-RTI solver. Speaks **standard ROS 2 messages** by default; PX4/uXRCE-DDS is an optional
backend.

<!-- Badges are intentionally absent until the workflows pass. Do not add one before the
     workflow is green — a red badge on the landing page is worse than none.
[![colcon build](https://github.com/Ali-Eimaan/uav-mpc/actions/workflows/colcon_build.yml/badge.svg)](...)
-->

> **Status: implemented, not yet verified.** All subsystems are written and unit-tested on
> paper, but **nothing in this repository has been compiled or flown yet** — acados has not been
> built here and no ROS 2 build has run. Numbers marked *TBM* (to be measured) are unfilled by
> design rather than estimated. Known defects are tracked in
> [`.deepseek/REVIEW.md`](.deepseek/REVIEW.md).

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
mkdir -p ~/ws/src && cd ~/ws/src && git clone https://github.com/Ali-Eimaan/uav-mpc.git
```

```bash
python3 -m venv --system-site-packages ~/.venvs/uavmpc && . ~/.venvs/uavmpc/bin/activate && pip install -r ~/ws/src/uav-mpc/requirements.txt
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

PX4 SITL (needs `px4_msgs` in the workspace and a PX4 checkout):

```bash
ros2 launch uav_mpc sitl.launch.py headless:=false trajectory:=figure8
```

Build without acados — core library, tests and the stub solver backend only:

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DUAV_MPC_WITH_ACADOS=OFF
```

## Results

Nothing has been measured yet. These rows are the deliverable of milestone M9
([`.deepseek/12_ANALYSIS.md`](.deepseek/12_ANALYSIS.md)); each must name the CPU it was measured
on and the git SHA that produced it, and report the from-bag number next to the synthetic one.

| Metric | Value | Conditions |
| --- | --- | --- |
| Median solve time | *TBM* | CPU, horizon, acados commit |
| p99 solve time | *TBM* | target: < 2 ms (A2) |
| RMS tracking error, figure-8 | *TBM* | amplitude, period, wind |
| SITL hover RMS error | *TBM* | target: < 0.15 m (A6) |

Regenerate with:

```bash
python analysis/solve_time_benchmark.py --samples 10000 --out media/solve_time_histogram.png
```

## Repository layout

| Path | Contents |
| --- | --- |
| [`.deepseek/`](.deepseek/README.md) | Implementation specification, split by subsystem, plus [`REVIEW.md`](.deepseek/REVIEW.md) |
| [`uav_mpc/`](uav_mpc/) | The ROS 2 package: node, vehicle backends, dynamics, trajectory generation, solver wrapper |
| [`codegen/`](codegen/) | CasADi model and the acados solver generator |
| [`analysis/`](analysis/) | Benchmarks and the notebooks producing the plots above |
| [`scripts/`](scripts/) | `assert_hover.py` — the SITL acceptance thresholds, runnable locally |
| [`docs/`](docs/) | Derivations (LaTeX), tuning guide, hardware bring-up |
| [`media/`](media/) | README assets (not yet captured) |

## Documentation

- [Derivations](docs/DERIVATION.md) — dynamics, differential flatness, NMPC formulation
- [Tuning guide](docs/TUNING_GUIDE.md) — weight selection and the failure table
- [Hardware bring-up](docs/HARDWARE_BRINGUP.md) — what has and has not been flown
- [Implementation specification](.deepseek/README.md) — the spec this code is built against
- [Review](.deepseek/REVIEW.md) — open defects from the R1 sweep

## Requirements

Ubuntu 26.04 LTS · ROS 2 Lyrical Luth · Python 3.14 · Eigen 3.4 · acados (commit pinned in
[`codegen/ACADOS_COMMIT`](codegen/ACADOS_COMMIT)).

Optional: `px4_msgs` + PX4 + Gazebo Jetty, for the PX4 backend and SITL.

> Several of these pins are still unconfirmed against the Lyrical Luth package set — see the
> version risk register in
> [`.deepseek/02_ENVIRONMENT.md` §2.1](.deepseek/02_ENVIRONMENT.md).

## Citation

This is the per-agent controller underlying the distributed MPC-CBF work in
`transition-viable-swarm`. A `CITATION.cff` will be added at v1.0.

## License

BSD-3-Clause — see [LICENSE](LICENSE).
