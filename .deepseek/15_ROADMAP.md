# §15 · Implementation order · §17 · Definition of done

---

## §15 · Milestones

Each milestone is independently verifiable. **Do not start one before the previous one passes.**

| # | Milestone | Files | Spec | Done when |
| --- | --- | --- | --- | --- |
| M1 | Frames and dynamics | `quadrotor_dynamics.*`, `test_frame_conversions.cpp`, `test_dynamics.cpp` | [04](04_DYNAMICS.md) | round-trips exact, Jacobians match finite differences |
| M2 | Symbolic model | `codegen/quadrotor_model.py`, `dynamics_probe` | [11](11_CODEGEN.md) | CasADi and C++ agree to 1e-9 (A4) |
| M3 | Solver generation | `generate_acados_solver.py` | [11](11_CODEGEN.md) | deterministic output, hash written, hover solves in Python (A3) |
| M4 | Trajectories | `trajectory_generator.*`, `test_trajectory_continuity.cpp` | [05](05_TRAJECTORY.md) | C⁴ everywhere, flatness consistent with the dynamics (A5) |
| M5 | Solver wrapper | `acados_wrapper.cpp`, `test_nmpc_solve_time.cpp` | [06](06_SOLVER.md) | A2 met |
| M6 | Node | `nmpc_node.cpp`, `main.cpp`, parameter code | [07](07_NODE.md), [09](09_CONFIG.md) | hovers in SITL |
| M7 | Launch | all four launch files | [08](08_LAUNCH.md) | one-command SITL flight (A8) |
| M8 | CI | three workflows | [14](14_CI.md) | A1, A6, A9 green |
| M9 | Analysis + media | `analysis/`, `scripts/`, `media/` | [12](12_ANALYSIS.md) | README numbers and GIFs exist |
| M10 | Docs | `docs/` | [13](13_DOCS.md) | derivations complete and matching the code |

### Why this order

**M1–M3 need no ROS at all. M1–M5 need no PX4.** Get as far as M5 before fighting a simulator —
debugging a frame convention inside a running SITL stack costs an order of magnitude more than
debugging it in a unit test.

M2 before M4 is deliberate: the flatness map in M4 calls the allocation from M1 and is validated
against the model from M2. Building trajectories against an unvalidated model means finding out
at M6 that both were wrong.

M8 after M7 rather than early: CI that runs before there is anything to run produces noise, and
noisy CI gets ignored.

### Parallelism

M9 and M10 are independent of each other and can follow M8 in either order. Everything else is a
strict chain.

---

## §17 · Definition of done

### Per file

A file is done when:

- every `TODO(deepseek)` in it is implemented and the marker **deleted**, or converted into a
  specific written issue with a reason
- it builds with `-Wall -Wextra -Wpedantic -Wconversion` clean
- its tests pass, in a Release build
- any behaviour a reader would not predict from the signature is documented in a comment

### Per repository

The repository is done when:

- all nine acceptance criteria in [01_OVERVIEW.md §1.3](01_OVERVIEW.md) hold
- the README's numbers are reproducible from `analysis/` on the hardware named in the plot
  titles
- `docs/HARDWARE_BRINGUP.md` accurately states what has and has not been flown
- every `verified: false` in `uav_mpc/params/` is either flipped to `true` with a cited source,
  or still `false` and honestly labelled
- no `UNVERIFIED` marker remains in `requirements.txt`, `codegen/ACADOS_COMMIT`, or the
  workflows without an accompanying note explaining why it could not be resolved

### Progress check

```bash
grep -rn "TODO(deepseek)" --exclude-dir=.git --exclude-dir=.deepseek . | wc -l
```

```bash
grep -rn "UNVERIFIED" --exclude-dir=.git --exclude-dir=.deepseek .
```

---

## §18 · Release criteria

The project is pre-release at version `0.1.0` (see `uav_mpc/package.xml`). Do **not** tag
`1.0.0` until the gate below holds — keeping `0.1.0` while R2-1..R2-3 are open is the honest
number.

**Gate for `1.0.0`:** all nine acceptance criteria (A1-A9) in
[01_OVERVIEW.md §1.3](01_OVERVIEW.md) are green, in CI, on the hardware named in the README.

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

**Who checks it:** the maintainer runs the release checklist below; CI runs A1-A9 automatically.
**What happens on each tag:** a release commit bumps the version in `uav_mpc/package.xml`,
updates `CHANGELOG.rst`, and tags `v<version>`. `bloom` releases are out of scope until a ROS
distro ships `px4_msgs` (the `generic` backend is the default and needs no `px4_msgs`).

### Release checklist (run before every tag)

1. A1-A9 all green in CI on the README's named hardware.
2. `media/figure8.gif`, `media/disturbance_recovery.gif`, `media/solve_time_histogram.png`
   exist and are reproducible from `analysis/` (R2-8).
3. `CITATION.cff` version matches `uav_mpc/package.xml` (R2-9).
4. `CHANGELOG.rst` is current (R2-12).

If any criterion cannot be met, change the criterion in `01_OVERVIEW.md §1.3` with a written
reason — do not weaken a test in place.

Both should trend to zero. The second one reaching zero matters more than the first — an
unimplemented function is visible, an unverified constant is not.
