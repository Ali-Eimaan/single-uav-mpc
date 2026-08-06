# §3 · Build system — `uav_mpc/CMakeLists.txt`

**Governs:** `uav_mpc/CMakeLists.txt`, `uav_mpc/package.xml`
**Prerequisites:** [02_ENVIRONMENT.md](02_ENVIRONMENT.md)
**Done when:** `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release` succeeds both with and
without acados present.

---

## 3.1 acados discovery

```cmake
if(UAV_MPC_WITH_ACADOS)
  if(NOT DEFINED ENV{ACADOS_SOURCE_DIR})
    message(FATAL_ERROR "ACADOS_SOURCE_DIR is not set. Either export it, or configure with "
                        "-DUAV_MPC_WITH_ACADOS=OFF to build the stub backend.")
  endif()
endif()
```

Include directories MUST be, in order:
`$ENV{ACADOS_SOURCE_DIR}/include`, `.../include/blasfeo/include`, `.../include/hpipm/include`,
and the generated tree `${CMAKE_CURRENT_SOURCE_DIR}/../codegen/codegen_output/c_generated_code`.

Import three shared libraries as `IMPORTED` targets: `libacados.so`, `libblasfeo.so`,
`libhpipm.so`, plus the generated `libacados_ocp_solver_quadrotor.so`.

## 3.2 Codegen freshness

Add a custom command that re-runs codegen when the model is newer than the output:

- **DEPENDS:** `codegen/quadrotor_model.py`, `codegen/generate_acados_solver.py`,
  `uav_mpc/config/nmpc_params.yaml`, the selected `params/*_calibration.yaml`
- **OUTPUT:** `codegen_output/c_generated_code/acados_solver_quadrotor.c`
- **COMMAND:** `python3 ${CMAKE_CURRENT_SOURCE_DIR}/../codegen/generate_acados_solver.py …`

This MUST NOT run in CI (CI uses `--check-only` and fails on drift). Guard it with
`if(NOT DEFINED ENV{CI})`.

## 3.3 Rules

- `UAV_MPC_WITH_ACADOS` is a `target_compile_definitions` on `${PROJECT_NAME}_core`
  **PRIVATE** only. If it leaks `PUBLIC`, consumers need acados headers and the stub CI job
  fails — which is exactly what that job is for. See [06_SOLVER.md §6.1](06_SOLVER.md).
- Release MUST be `-O2` or better. The A2/A3 timings are meaningless in a Debug build.
- `-Wconversion` is on deliberately. Fix the warnings; do not silence them.

## 3.4 Targets

The skeleton already declares the target graph; keep it:

| Target | Kind | Contains |
| --- | --- | --- |
| `uav_mpc` (interfaces) | `rosidl_generate_interfaces` | the two msgs, srv, action |
| `uav_mpc_core` | SHARED lib | dynamics, trajectory, acados wrapper |
| `nmpc_component` | SHARED lib + registered component | the lifecycle node |
| `nmpc_node` | executable | `main.cpp`, links `nmpc_component` |
| `dynamics_probe` | test-only executable | added at M2, see [10_TESTS.md §10.3](10_TESTS.md) |

Interface generation order matters: `rosidl_generate_interfaces` must list
`SolverDiagnostics.msg` and `TrajectorySpec.msg` before the files that depend on them.
Link the node against its own generated types via `rosidl_get_typesupport_target`.

## 3.4.1 px4_msgs is optional, never required

`find_package(px4_msgs QUIET)` gates only the `UAV_MPC_WITH_PX4_MSGS` compile definition on
`nmpc_component`. The node target is built **unconditionally** — a build without px4_msgs is a
fully working controller using the generic backend, not a degraded one.

`src/vehicle_interface_px4.cpp` is listed unconditionally in the target sources; it wraps its
whole body in `#ifdef UAV_MPC_WITH_PX4_MSGS`, so without the define it compiles to an empty
translation unit. Do not move it into a conditional `if()` block — the unconditional listing is
what keeps the file compiling (and lint-clean) on every build.

## 3.5 `package.xml`

`<license>BSD-3-Clause</license>`, format 3, `<member_of_group>rosidl_interface_packages</member_of_group>`.
Every dependency the code includes must be declared — a missing `<depend>` builds fine on your
machine and fails in the clean container, which is the whole point of A1.
