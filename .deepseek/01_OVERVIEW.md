# §1 · What is being built

A nonlinear MPC that tracks aggressive quadrotor trajectories at 100 Hz, running on ROS 2 and
commanding PX4 over uXRCE-DDS.

```
trajectory_generator ──flat outputs──► flatness map ──(x_ref, u_ref)──► acados SQP-RTI
                                                                              │
     PX4 (uXRCE-DDS) ──odometry──► nmpc_node ─────────────────────────────────┘
                                       │
                                       └──► VehicleAttitudeSetpoint ──► PX4 inner loops
```

## 1.1 Why this repository exists

It is a portfolio repository for a robotics PhD application, and it is **the first repository a
hardware-equipped advisor opens**. That single fact drives most of the engineering decisions in
these documents:

- **If someone can clone, build, and fly a figure-8 in ten minutes, it passes their first
  filter. If anything is broken, it fails.** `sitl.launch.py` working from a clean clone
  (criterion A8) matters more than any individual feature.
- **Every number must be defensible.** A solve-time plot without the CPU named, an inertia
  tensor without a source, or a bring-up guide implying flights that never happened — each of
  these costs more credibility than the missing capability would have.
- **Stating limits is a strength here, not a weakness.** The sections in the derivations that
  say what the formulation does *not* guarantee are load-bearing.

Downstream, this per-agent NMPC is the building block for the distributed MPC-CBF controller in
the author's `transition-viable-swarm` work. The OCP structure and the flatness-based reference
carry over; the centralised solve does not.

## 1.2 Deliverables beyond the code

These are part of "done", not extras:

| Deliverable | Produced by | Notes |
| --- | --- | --- |
| `media/figure8.gif` | `figure8.launch.py` + capture | Gazebo view with a solve-time side panel |
| `media/disturbance_recovery.gif` | `analysis/disturbance_sweep.ipynb` | gust step and recovery |
| `media/solve_time_histogram.png` | `analysis/solve_time_benchmark.py` | median and p99 marked |
| README results table | `analysis/` | every row names its hardware and git SHA |

The three CI workflows are themselves a deliverable — see [14_CI.md](14_CI.md). The smoke test
that autonomously hovers in CI is the strongest single signal in the repository, because most
comparable repositories do not build at all.

---

## §1.3 Acceptance criteria

The whole project is done when all nine hold.

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

**If a criterion cannot be met, change the criterion in this file with a written reason.**
Do not weaken a test in place — a threshold quietly relaxed to make CI green is a lie told to
every future reader.

## 1.4 Design decisions already made

These are settled. They are recorded here so they are not re-litigated mid-implementation;
each is justified in its own document.

| Decision | Value | Where |
| --- | --- | --- |
| State | `x = [p(3), v(3), q(4,wxyz), ω(3)]`, `nx = 13` | [04_DYNAMICS.md §4.2](04_DYNAMICS.md) |
| Input | `u = [T₁..T₄]`, per-rotor thrust in newtons | [04_DYNAMICS.md §4.3](04_DYNAMICS.md) |
| World / body frame | ENU / FLU, converted only at the PX4 boundary | [16_CONVENTIONS.md](16_CONVENTIONS.md) |
| Horizon | `N = 20`, `Tf = 1.0 s`, `dt = 50 ms` | [06_SOLVER.md §6.2](06_SOLVER.md) |
| Solver | acados SQP-RTI + partial-condensing HPIPM | [06_SOLVER.md §6.2](06_SOLVER.md) |
| Cost | `NONLINEAR_LS`, 12-dim state residual, `q_ref` in the parameter vector | [06_SOLVER.md §6.3](06_SOLVER.md) |
| PX4 interface | `VehicleAttitudeSetpoint` at 100 Hz | [07_NODE.md §7.6](07_NODE.md) |
| ROS node type | lifecycle node, also available as a component | [07_NODE.md §7.1](07_NODE.md) |
