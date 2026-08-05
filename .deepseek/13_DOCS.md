# §13 · Documentation

**Governs:** `docs/DERIVATION.md`, `docs/derivations/*.tex`, `docs/TUNING_GUIDE.md`,
`docs/HARDWARE_BRINGUP.md`
**Milestone:** M10
**Done when:** the derivations are complete and match the code, equation for equation.

These are not an afterthought. For the audience described in
[01_OVERVIEW.md §1.1](01_OVERVIEW.md), the derivations are the difference between "this person
integrated a library" and "this person understands the system".

---

| § | File | Requirement |
| --- | --- | --- |
| 13.1 | `derivations/quadrotor_se3_dynamics.tex` | Full Newton–Euler derivation, symbol table with units, quad-X allocation figure, hover linearisation justifying `dt` and `Tf`, geometric-control baseline |
| 13.2 | `derivations/differential_flatness.tex` | The map with **all intermediate algebra**, degenerate cases, the feasibility inequality worked for the 3 m/5 s figure-8 on the X500, and an equation → code-function table |
| 13.3 | `derivations/nmpc_formulation.tex` | The OCP as generated, the Bryson weight table, the RTI statement **with its hypotheses**, and a section on what this formulation does *not* guarantee |
| 13.4 | `TUNING_GUIDE.md` | The tuning order, and the failure table — extend it with every failure you actually hit |
| 13.5 | `HARDWARE_BRINGUP.md` | A status line per section (`written` / `bench-tested` / `flown`), kept accurate |

## 13.6 Two rules that matter more than completeness

**State the limits.** §13.3's closing section and §13.5's status lines are load-bearing.
Claiming stability you have not established, or implying flights you have not flown, is the one
failure mode this repository cannot recover from. A candidate who writes "no terminal set,
therefore no nominal stability certificate" reads as a researcher; one who omits it reads as
someone who did not know to ask.

**Keep the equation→code table current.** `differential_flatness.tex` ends with a table mapping
each equation number to the function and line that implements it. A stale table is worse than
none. If you change either side, change both in the same commit.

## 13.7 Cross-references

`docs/DERIVATION.md` is the readable index; the `.tex` files carry the detail. The `.tex`
headers name the source files they are the authority for:

- `quadrotor_se3_dynamics.tex` → `uav_mpc/src/quadrotor_dynamics.cpp` + `codegen/quadrotor_model.py`
- `differential_flatness.tex` → `TrajectoryGenerator::flatToStateInput()`
- `nmpc_formulation.tex` → `codegen/generate_acados_solver.py` + `uav_mpc/src/acados_wrapper.cpp`

If the code and a derivation disagree, the derivation is wrong until proven otherwise — but they
must not disagree in a commit.

## 13.8 References to include

At minimum, with full bibliographic entries in a real `refs.bib`:

- Mellinger & Kumar (ICRA 2011) — minimum snap, differential flatness
- Lee, Leok & McClamroch (CDC 2010) — geometric tracking control on SE(3)
- Verschueren et al. (2021) — acados
- Diehl et al. (2005) — real-time iteration
- Faessler, Falanga & Scaramuzza (RA-L 2017) — rotor drag and flatness
- Richter, Bry & Roy (2013) — unconstrained minimum-snap reformulation
