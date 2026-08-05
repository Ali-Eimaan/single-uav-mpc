# §11 · Codegen

**Governs:** `codegen/quadrotor_model.py`, `codegen/generate_acados_solver.py`,
`codegen/ACADOS_COMMIT`
**Prerequisites:** [04_DYNAMICS.md](04_DYNAMICS.md) — the C++ model is the reference
**Milestones:** M2 (model), M3 (solver generation)
**Done when:** criteria A3 and A4 hold.

---

## 11.1 `quadrotor_model.py`

The CasADi model MUST be the same equations as [04_DYNAMICS.md §4.2](04_DYNAMICS.md).
`AirframeConstants.from_yaml()` reads the same `params/*_calibration.yaml` the C++ reads — one
source of truth, no transcribed numbers.

Layout, restated so it cannot drift:

```
x = [p(3), v(3), q(4, wxyz), ω(3)]     nx = 13   (quaternion; Euler variant nx = 12)
u = [T1, T2, T3, T4]                    nu = 4    per-rotor thrust [N]
p = [wind(3), mass_scale, q_ref(4)]     np = 8
```

`model.name = "quadrotor"` is load-bearing: the generated symbols, the header name
`acados_solver_quadrotor.h`, and the capsule type all derive from it. Changing it means changing
the include in `acados_wrapper.cpp` and the library name in `CMakeLists.txt`.

`model_hash()` must be deterministic across runs and machines: hash the `str()` of the SX graph
plus the constants dict serialised with sorted keys and fixed-precision float formatting
(`f"{v:.17g}"`). No `id()`, no unsorted dict iteration, no `repr()` of floats.

Write the kinematics out explicitly rather than using CasADi's quaternion helpers, so the
derivation in `docs/derivations/` matches the code line for line.

## 11.2 `generate_acados_solver.py`

`--check-only` regenerates into a temp dir, diffs against the committed tree, and exits non-zero
on any difference with a readable summary. That is the mode CI runs — a stale committed solver
must fail the build, not be silently regenerated.

Emits:

| Output | Contents |
| --- | --- |
| `codegen/MODEL_HASH` | the bare hex digest, newline-terminated |
| `codegen_output/include/model_hash.h` | `#define UAV_MPC_MODEL_HASH "…"` |
| `codegen_output/GENERATION_PROVENANCE.txt` | acados commit, CasADi version, input file hashes — **no timestamps**, they break determinism |

`strip_nondeterminism()` rewrites acados' generation-date lines and MUST assert that its
substitution matched, so a future acados that stops emitting them fails loudly instead of
silently no-oping.

The OCP settings themselves are specified in [06_SOLVER.md §6.2–6.4](06_SOLVER.md) — build them
from there, not from acados' examples.

## 11.3 `ACADOS_COMMIT`

Pin the full 40-character SHA of the acados commit you actually generate and test against, plus
the release tag it corresponds to. Never pin a branch name — reproducibility is the entire point
of the file.

`generate_acados_solver.py` refuses to run against a different commit unless
`--allow-acados-mismatch` is passed. Keep that refusal; it is what stops "works on my machine"
solver differences.

> Verify acados builds against the Ubuntu 26.04 toolchain before pinning — risk V8 in
> [02_ENVIRONMENT.md §2.1](02_ENVIRONMENT.md).

## 11.4 The generated tree

`codegen/codegen_output/` is gitignored except `include/*.h`. The generated C is reproducible
from this directory, so it is not tracked; the small public headers are kept for IDE resolution
and so `acados_wrapper.cpp` parses without a build.
