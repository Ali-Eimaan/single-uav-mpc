# `.deepseek/` — implementation specification

**You are the implementing model.** This directory is your complete instruction set for turning
the `uav-mpc` skeleton into working code. Everything you need is here; nothing outside this
directory instructs you.

**Scope:** every file in the repository containing a `TODO(deepseek)` marker.
**Authority:** where a document here and a code comment disagree, the document wins — say so in
the commit message rather than silently diverging.

---

## Read in this order

| Read first | Document | Covers |
| --- | --- | --- |
| 1 | [00_RULES.md](00_RULES.md) | How to work. Non-negotiable. Read before touching a file. |
| 2 | [01_OVERVIEW.md](01_OVERVIEW.md) | What is being built, why it exists, acceptance criteria A1–A9 |
| 3 | [02_ENVIRONMENT.md](02_ENVIRONMENT.md) | Ubuntu 26.04 / ROS 2 Lyrical Luth, pinned versions, **version risk register** |
| 4 | [15_ROADMAP.md](15_ROADMAP.md) | Milestones M1–M10 in dependency order, definition of done |
| 5 | [16_CONVENTIONS.md](16_CONVENTIONS.md) | Frames, quaternions, units, time, traps. Re-read when confused. |

Then work through the milestones, opening the subsystem document for each:

| Document | Implements | Milestone |
| --- | --- | --- |
| [03_BUILD_SYSTEM.md](03_BUILD_SYSTEM.md) | `uav_mpc/CMakeLists.txt`, `package.xml` | throughout |
| [04_DYNAMICS.md](04_DYNAMICS.md) | `quadrotor_dynamics.{hpp,cpp}` | M1 |
| [11_CODEGEN.md](11_CODEGEN.md) | `codegen/quadrotor_model.py`, `generate_acados_solver.py` | M2, M3 |
| [05_TRAJECTORY.md](05_TRAJECTORY.md) | `trajectory_generator.{hpp,cpp}` | M4 |
| [06_SOLVER.md](06_SOLVER.md) | `acados_wrapper.{hpp,cpp}` | M5 |
| [07_NODE.md](07_NODE.md) | `nmpc_node.{hpp,cpp}`, `main.cpp` | M6 |
| [08_LAUNCH.md](08_LAUNCH.md) | the four launch files | M7 |
| [09_CONFIG.md](09_CONFIG.md) | `config/`, `params/`, parameter declaration | M6 |
| [10_TESTS.md](10_TESTS.md) | everything under `uav_mpc/test/` | with each milestone |
| [14_CI.md](14_CI.md) | `.github/workflows/` | M8 |
| [12_ANALYSIS.md](12_ANALYSIS.md) | `analysis/`, `scripts/` | M9 |
| [13_DOCS.md](13_DOCS.md) | `docs/` derivations and guides | M10 |

[FILE_MAP.md](FILE_MAP.md) — every skeleton file in the repository mapped to the document that
specifies it. Use it when you have a file and need its spec.

---

## Section numbering

Each document owns a section number that matches its filename prefix: `04_DYNAMICS.md` contains
§4, `07_NODE.md` contains §7, and so on. A cross-reference written `§7.6` therefore always
means "section 7.6, which lives in `07_NODE.md`". This holds everywhere, including in the
`TODO(deepseek)` comments inside the skeleton source files.

`15_ROADMAP.md` carries both §15 (implementation order) and §17 (definition of done).

## Conventions used in these documents

- **MUST** — required for correctness or safety. Deviating is a bug.
- **SHOULD** — strong default. Deviate only with a stated reason in a code comment.
- **UNVERIFIED** — a value or assumption that has not been confirmed. Verify before relying on
  it, and update the document with what you found.

## Before you start

Run this to see the work remaining:

```bash
grep -rn "TODO(deepseek)" --exclude-dir=.git --exclude-dir=.deepseek .
```

Every one of those markers is specified somewhere in this directory. If you find one that is
not, that is a gap in the spec — report it rather than guessing.
