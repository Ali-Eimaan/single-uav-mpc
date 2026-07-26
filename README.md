# uav-mpc

Real-time NMPC for a 12-state quadrotor in ROS 2 Jazzy + PX4 SITL + Gazebo Harmonic, using
acados-generated C code, tracking aggressive trajectories.

<!-- TODO(deepseek): badges once the workflows are green. Do not add a badge before the
     workflow passes — a red badge on the landing page is worse than none.
[![colcon build](https://github.com/Ali-Eimaan/uav-mpc/actions/workflows/colcon_build.yml/badge.svg)](...)
[![format check](https://github.com/Ali-Eimaan/uav-mpc/actions/workflows/format_check.yml/badge.svg)](...)
[![smoke test](https://github.com/Ali-Eimaan/uav-mpc/actions/workflows/docker_smoke_test.yml/badge.svg)](...)
-->

> **Status: skeleton.** The structure, interfaces and documentation outline are in place; the
> implementation is not. See [`IMPLEMENTATION_GUIDE.md`](IMPLEMENTATION_GUIDE.md) for the
> build order and the specification of every stub.

<!-- TODO(deepseek): media/figure8.gif goes here, above the fold. This is the first thing a
     reader sees; it decides whether they scroll. -->

---

## Quick start

```bash
# TODO(deepseek): this block is the repo's central promise — clone, build, fly in ten minutes.
# It must work verbatim on a clean Ubuntu 24.04 machine. Test it in a fresh container before
# every release, and keep it to this many lines.
```

## What this is

- A nonlinear MPC that tracks aggressive quadrotor trajectories at 100 Hz, solved with an
  acados SQP-RTI solver generated from a CasADi model.
- A ROS 2 Jazzy lifecycle node that speaks PX4's uXRCE-DDS interface, in SITL or on hardware.
- Minimum-snap and analytic trajectory generation through the differential-flatness map.

## Results

<!-- TODO(deepseek): fill from analysis/. Every number needs the hardware it was measured on
     and the git SHA that produced it. Report the from-bag numbers alongside the synthetic
     ones — see analysis/solve_time_benchmark.py. -->

| Metric | Value | Conditions |
| --- | --- | --- |
| Median solve time | _TODO_ | _CPU, horizon, acados commit_ |
| p99 solve time | _TODO_ | |
| RMS tracking error, figure-8 | _TODO_ | _amplitude, period, wind_ |

## Repository layout

| Path | Contents |
| --- | --- |
| [`uav_mpc/`](uav_mpc/) | The ROS 2 package: node, dynamics, trajectory generation, solver wrapper |
| [`codegen/`](codegen/) | CasADi model and the acados solver generator |
| [`analysis/`](analysis/) | Benchmarks and the notebooks producing the plots above |
| [`docs/`](docs/) | Derivations (LaTeX), tuning guide, hardware bring-up |
| [`media/`](media/) | README assets |

## Documentation

- [Derivations](docs/DERIVATION.md) — dynamics, flatness, NMPC formulation
- [Tuning guide](docs/TUNING_GUIDE.md)
- [Hardware bring-up](docs/HARDWARE_BRINGUP.md)
- [Implementation guide](IMPLEMENTATION_GUIDE.md) — the spec this skeleton is built against

## Requirements

<!-- TODO(deepseek): exact versions, pinned. ROS 2 Jazzy, Ubuntu 24.04, PX4 (version from
     .github/workflows/docker_smoke_test.yml), Gazebo Harmonic, acados (commit from
     codegen/ACADOS_COMMIT), Eigen 3.4, Python 3.12. -->

## Citation

<!-- TODO(deepseek): CITATION.cff plus the one-line note that this is the per-agent controller
     underlying the distributed MPC-CBF work in `transition-viable-swarm`. -->

## License

MIT — see [LICENSE](LICENSE).
