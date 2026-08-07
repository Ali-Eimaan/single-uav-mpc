# acados: what it does, and how to run with or without it

`uav_mpc` has **two solver backends**, chosen at *build* time by the CMake option
`UAV_MPC_WITH_ACADOS`:

| | acados backend (`ON`, default) | stub backend (`OFF`) |
| --- | --- | --- |
| Solves the OCP | **yes** — real SQP-RTI | **no** — every solve is a no-op |
| Extra dependency | acados + CasADi + a code-generation step | none beyond ROS 2 and Eigen |
| Build time | ~15 min first time (acados itself) | ~40 s |
| Can fly | **yes** | **no** |
| Intended for | everything real | linting, unit tests, CI on machines without acados |

> **The stub backend does not compute anything.** `AcadosWrapper::solve()` returns
> `SolverStatus::Success` immediately without touching the problem, and `optimalInput()`
> returns zeros. It exists so that the dynamics, trajectory generation, frame conversions and
> the ROS layer can be compiled and unit-tested on a machine that has no acados. **Never fly
> it, and never quote a solve time measured against it** — see the safety note in §5.

---

## 1. What acados actually provides

The controller solves this optimal control problem every 10 ms:

```
min  Σ_{k=0..N-1} ‖y(x_k,u_k) − y_ref_k‖²_W  +  ‖y_N(x_N) − y_ref_N‖²_{W_N}
s.t. x_0 = x̂,  x_{k+1} = F(x_k, u_k),  u ∈ [u_min, u_max],  |ω| ≤ ω_max (soft)
```

with `N = 20`, `Tf = 1 s` (so `dt = 50 ms`), 13 states and 4 inputs.

acados contributes three things:

1. **Code generation.** `codegen/quadrotor_model.py` describes the dynamics symbolically in
   CasADi. `codegen/generate_acados_solver.py` turns that into self-contained C — the
   integrator, the cost residuals, and all their derivatives — with no runtime symbolic
   machinery left.
2. **The SQP-RTI scheme.** One real-time iteration per control tick: a single QP, no
   convergence loop, bounded work per tick. That is what makes a 100 Hz hard deadline feasible.
3. **The QP solver.** `PARTIAL_CONDENSING_HPIPM` with `qp_solver_cond_N = 5`.

Measured on the reference machine (Ubuntu 26.04, GCC 15.2, i5-12500H, Release): median
**0.218 ms**, p99 **0.469 ms** per solve. The acceptance budget is 2.0 ms p99 — roughly 4×
margin. See the README results table.

### What is NOT acados

These are plain C++ and work identically in both backends:

- `quadrotor_dynamics.cpp` — the rigid-body model, allocation matrix and frame conversions
- `trajectory_generator.cpp` — minimum-snap and analytic trajectories, the flatness map
- `nmpc_node.cpp` / `vehicle_interface*.cpp` — the ROS 2 lifecycle node and vehicle backends

So a stub build still exercises ~80% of the codebase. It simply has no optimiser.

---

## 2. Building WITH acados

### 2.1 Prerequisites

| Component | Version | Note |
| --- | --- | --- |
| acados | commit in [`codegen/ACADOS_COMMIT`](../codegen/ACADOS_COMMIT) | pinned; the build refuses to run against a different commit |
| CasADi | pinned in [`requirements.txt`](../requirements.txt) | 3.7.2, verified with cp314 wheels |
| Python | 3.14 | the system interpreter on Ubuntu 26.04 |

### 2.2 Build acados

```bash
git clone --recursive https://github.com/acados/acados.git ~/acados
```

```bash
cd ~/acados && git checkout $(grep -oE '[0-9a-f]{40}' /path/to/uav-mpc/codegen/ACADOS_COMMIT) && git submodule update --init --recursive
```

**Configure with qpOASES OFF.** This is not optional on Ubuntu 26.04:

```bash
cmake -S ~/acados -B ~/acados/build -DACADOS_WITH_QPOASES=OFF -DACADOS_WITH_OSQP=OFF -DACADOS_WITH_HPIPM=ON -DACADOS_INSTALL_DIR=$HOME/acados -DCMAKE_BUILD_TYPE=Release
```

```bash
cmake --build ~/acados/build --target install -j$(nproc)
```

> **Why `-DACADOS_WITH_QPOASES=OFF`.** acados bundles qpOASES, whose C predates C23. GCC 15
> (the Ubuntu 26.04 default) rejects it:
> `error: passing argument 1 of 'ConstraintsCPY' from incompatible pointer type`.
> The build fails outright. We never use qpOASES — the OCP selects
> `PARTIAL_CONDENSING_HPIPM` — so turning it off costs nothing.

### 2.3 Environment

Both variables are required, in every shell that builds or runs the package:

```bash
export ACADOS_SOURCE_DIR=$HOME/acados && export LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib:$LD_LIBRARY_PATH
```

Add them to your shell profile. `LD_LIBRARY_PATH` is the one people forget; the symptom is
`OSError: libhpipm.so: cannot open shared object file` at test time.

### 2.4 Python toolchain

Ubuntu 26.04 enforces PEP 668, so a bare `pip install` into the system interpreter is refused.
Use a venv that can still see the ROS 2 Python modules:

```bash
python3 -m venv --system-site-packages ~/.venvs/uavmpc && . ~/.venvs/uavmpc/bin/activate && pip install -r requirements.txt
```

`acados_template` is installed from the acados tree, **not** PyPI — the PyPI package will not
match your pinned commit:

```bash
pip install -e $ACADOS_SOURCE_DIR/interfaces/acados_template
```

### 2.5 Generate the solver

```bash
python codegen/generate_acados_solver.py
```

This writes `codegen/codegen_output/` (gitignored — it is reproducible), plus
`codegen/MODEL_HASH`, the fingerprint of the model the solver was generated from.

CMake re-runs this automatically when `quadrotor_model.py`, `generate_acados_solver.py` or
`config/nmpc_params.yaml` changes, so you rarely invoke it by hand.

### 2.6 Build and test

```bash
colcon build --packages-select uav_mpc --cmake-args -DCMAKE_BUILD_TYPE=Release
```

```bash
colcon test --packages-select uav_mpc && colcon test-result --all
```

Expect **85 tests, 0 failures, 0 skipped**.

> Release matters. `-DCMAKE_BUILD_TYPE=Debug` produces solve times that are not comparable to
> anything, and the A2 acceptance criterion is meaningless there.

---

## 3. Building WITHOUT acados

One flag. No acados, no CasADi, no code generation, no `ACADOS_SOURCE_DIR`:

```bash
colcon build --packages-select uav_mpc --cmake-args -DCMAKE_BUILD_TYPE=Release -DUAV_MPC_WITH_ACADOS=OFF
```

```bash
colcon test --packages-select uav_mpc && colcon test-result --all
```

Expect **85 tests, 0 failures, 8 skipped**. The 8 skips are correct and self-describing:

| Skipped | Why |
| --- | --- |
| 7 × `test_acados_codegen.py` | the whole module needs CasADi + acados_template + the native libraries |
| 1 × `GeneratedModelHashMatchesCheckedInHash` | `generatedModelHash()` is empty by design in the stub |

Everything else — dynamics, Jacobians, frame conversions, trajectory continuity, the flatness
map, the node state machine — runs and must pass. If any of those fail, it is a real defect,
not a missing dependency.

### 3.1 When the stub is the right choice

- **CI lint / unit jobs** that only need the package to compile (`build-no-acados`).
- **Reviewing or editing** the dynamics, trajectory or ROS layer without a 15-minute acados
  build in the loop.
- **A machine where acados will not build** — while you sort that out, the rest of the package
  is still testable.

### 3.2 When it is NOT

Anything involving actual control. The node will start, accept odometry, run its state machine
and publish setpoints — but the setpoints are meaningless, because `optimalInput()` returns
zeros. See §5.

---

## 4. Telling which backend you built

Three independent checks, cheapest first.

**At configure time**, CMake prints its decision:

```
-- px4_msgs NOT found — building with the generic vehicle backend only
```

**At compile time**, the `UAV_MPC_WITH_ACADOS` macro is what actually switches the code:

```bash
grep -r UAV_MPC_WITH_ACADOS build/uav_mpc/CMakeFiles/uav_mpc_core.dir/flags.make
```

**At test time** — the reliable one. `GeneratedModelHashMatchesCheckedInHash` **runs** in an
acados build and **skips** in a stub build:

```bash
colcon test-result --all --verbose | grep -A2 GeneratedModelHash
```

If that test skipped, you are on the stub. Any solve-time number from that run is measuring
nothing.

---

## 5. Safety

**Never arm a vehicle against a stub build.** The node cannot tell you it is not solving:
`solve()` reports `Success`, `~/status` shows a plausible-looking `SolverDiagnostics`, and the
attitude setpoint stream looks alive. The only outward sign is that the commanded thrust is
zero and the vehicle does not respond.

Before any flight or simulation run, confirm the hash test ran (§4). It is one command, and it
is the only check that cannot be faked by a nominal-looking status topic.

---

## 6. Troubleshooting

| Symptom | Cause | Fix |
| --- | --- | --- |
| `error: passing argument 1 of 'ConstraintsCPY' from incompatible pointer type` | qpOASES vs GCC 15 | rebuild acados with `-DACADOS_WITH_QPOASES=OFF` (§2.2) |
| `CMake Error: ACADOS_SOURCE_DIR is not set` | env missing | §2.3, or build the stub with `-DUAV_MPC_WITH_ACADOS=OFF` |
| `OSError: libhpipm.so: cannot open shared object file` | `LD_LIBRARY_PATH` missing | §2.3. The Python tests now detect this and skip with this message rather than failing obscurely |
| `test_acados_codegen` all skipped, but acados IS built | `acados_template` not installed, or installed from PyPI instead of your tree | §2.4 |
| `GeneratedModelHashMatchesCheckedInHash` **fails** (not skips) | the generated solver is stale relative to the model | re-run `python codegen/generate_acados_solver.py`; never fly a stale solver |
| `externally-managed-environment` from pip | PEP 668 on Ubuntu 26.04 | use the venv in §2.4; do **not** use `--break-system-packages` |
| Solve times ~0 ms and suspiciously perfect | you are on the stub | §4 |

---

## 7. Related documents

- [`docs/derivations/nmpc_formulation.tex`](derivations/nmpc_formulation.tex) — the OCP as
  generated: cost, constraints and every solver option, with the reasoning for each
- [`docs/derivations/quadrotor_se3_dynamics.tex`](derivations/quadrotor_se3_dynamics.tex) —
  the model the CasADi code generates from
- [`docs/TUNING_GUIDE.md`](TUNING_GUIDE.md) — weight selection and the failure table
- [`codegen/ACADOS_COMMIT`](../codegen/ACADOS_COMMIT) — the pinned acados revision
- [`acceptance_criteria.yaml`](../acceptance_criteria.yaml) — the solve-time budgets this
  document quotes
