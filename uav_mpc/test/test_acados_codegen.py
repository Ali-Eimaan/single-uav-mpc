"""Verifies that the checked-in solver is exactly what codegen/ produces from the current
model and config.

A stale generated solver that still builds is the most dangerous failure mode in this repo:
the code flies, but not the model you documented. Every test here either detects that drift
or guards the real-time solver configuration against a debug-settings commit.

Run standalone:  pytest uav_mpc/test/test_acados_codegen.py -v
Run via colcon:  colcon test --packages-select uav_mpc

These tests need casadi + acados_template + a built acados; without them the module skips
(importorskip), mirroring the stub-backend CI path where the codegen checks cannot run.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import re
import subprocess
import sys

import numpy as np
import pytest

# --- module-level setup: make `codegen` importable and skip when its deps are missing ----
_HERE = Path(__file__).resolve().parent  # uav_mpc/test
_UAV_MPC = _HERE.parent  # uav_mpc
_REPO_ROOT = _UAV_MPC.parent  # repository root

for _p in (_REPO_ROOT, _REPO_ROOT / "codegen"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

# this module must remain COLLECTABLE even when its dependencies are absent.
#
# The obvious spellings — `pytest.importorskip(...)` at module scope, or
# `pytest.skip(..., allow_module_level=True)` — abort collection. pytest 9 then exits with
# code 4 ("found no collectors"), ctest sees a non-zero exit, and the test is reported as a
# FAILURE even though the xunit file correctly says "skipped". That is precisely what the
# `build-no-acados` CI job does, so that job fails on a machine without acados.
#
# Instead: probe the dependencies, keep the module importable, and let `pytestmark` mark every
# test skipped. Collection succeeds, pytest exits 0, and ctest reports a clean skip.
#
# Two independent things are probed, because they fail independently:
#   1. the Python packages (casadi, acados_template)
#   2. the acados NATIVE libraries — a machine can have acados_template pip-installed while
#      libacados.so is missing from the loader path, which otherwise surfaces deep inside a
#      test as a bare "OSError: libhpipm.so: cannot open shared object file".
_SKIP_REASON = None

try:
    import casadi  # noqa: F401  (imported for the module)
except ImportError as _exc:  # pragma: no cover - env dependent
    _SKIP_REASON = f"casadi is not installed ({_exc})"

if _SKIP_REASON is None:
    try:
        import acados_template  # noqa: F401
    except ImportError as _exc:  # pragma: no cover - env dependent
        _SKIP_REASON = f"acados_template is not installed ({_exc})"

if _SKIP_REASON is None:
    try:
        import ctypes

        ctypes.CDLL("libacados.so")
    except OSError as _exc:  # pragma: no cover - env dependent
        _SKIP_REASON = (
            f"acados_template imports but the acados native libraries are not loadable ({_exc}). "
            "Build acados and put its lib/ on LD_LIBRARY_PATH: "
            "export ACADOS_SOURCE_DIR=<acados>; "
            "export LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib:$LD_LIBRARY_PATH"
        )

if _SKIP_REASON is None:
    from codegen import generate_acados_solver  # noqa: E402
    from codegen import quadrotor_model  # noqa: E402
else:  # pragma: no cover - env dependent
    generate_acados_solver = None
    quadrotor_model = None

# Applies to every test in this module; harmless (empty) when the dependencies are present.
pytestmark = pytest.mark.skipif(_SKIP_REASON is not None, reason=_SKIP_REASON or "")


# --------------------------------------------------------------------------------------- fixtures


@pytest.fixture(scope="module")
def repo_root() -> Path:
    """Absolute path to the repository root (three levels up from this test file)."""
    assert _REPO_ROOT.is_dir(), f"repo root {_REPO_ROOT} is not a directory"
    for required in ("codegen", "uav_mpc"):
        assert (_REPO_ROOT / required).is_dir(), f"repo root {_REPO_ROOT} is missing {required}/"
    return _REPO_ROOT


def _airframe_path(repo_root: Path) -> Path:
    return repo_root / "uav_mpc/params/x500_calibration.yaml"


def _config_path(repo_root: Path) -> Path:
    return repo_root / "uav_mpc/config/nmpc_params.yaml"


def _generated_dir(repo_root: Path) -> Path:
    """The committed generated solver tree (codegen/codegen_output/c_generated_code)."""
    return repo_root / "codegen/codegen_output/c_generated_code"


def _generated_json(repo_root: Path) -> dict:
    """The committed acados OCP JSON; fails with the regeneration command when missing."""
    path = repo_root / "codegen/codegen_output/c_generated_code/acados_ocp_quadrotor.json"
    if not path.is_file():
        pytest.fail(
            "generated solver missing — run:\n"
            f"  python codegen/generate_acados_solver.py --airframe {_airframe_path(repo_root)} "
            f"--config {_config_path(repo_root)} --output codegen/codegen_output"
        )
    return json.loads(path.read_text())


def _constants(repo_root: Path) -> quadrotor_model.AirframeConstants:
    return quadrotor_model.AirframeConstants.from_yaml(_airframe_path(repo_root))


def _config(repo_root: Path) -> dict:
    return generate_acados_solver.load_nmpc_config(_config_path(repo_root))


def _find_dynamics_probe(repo_root: Path) -> Path | None:
    """Locate the C++ helper binary built by CMakeLists.txt under BUILD_TESTING.

    colcon builds it into build/uav_mpc/dynamics_probe; allow an env override for other
    layouts (plain cmake, CI runners with a separate build dir).
    """
    override = os.environ.get("UAV_MPC_DYNAMICS_PROBE")
    if override:
        path = Path(override)
        if path.is_file():
            return path
    candidates = [
        repo_root / "build/uav_mpc/dynamics_probe",
        repo_root / "build/dynamics_probe",
        repo_root / "install/uav_mpc/lib/uav_mpc/dynamics_probe",
    ]
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    hits = sorted(repo_root.glob("build/**/dynamics_probe"))
    if hits:
        return hits[0]
    return None


def _default_params() -> np.ndarray:
    """Online parameters matching the generated default: zero wind, mass_scale = 1,
    identity q_ref."""
    return np.array([0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0])


# --------------------------------------------------------------------------------------- tests


def test_casadi_model_builds(repo_root):
    """quadrotor_model.py must produce a well-formed AcadosModel."""
    constants = _constants(repo_root)
    model = quadrotor_model.export_quadrotor_model(constants, "quaternion")

    assert (
        model.name == "quadrotor"
    ), f"model name must be 'quadrotor' (generated symbols derive from it), got {model.name!r}"
    assert model.x.shape == (13, 1), f"x.shape == {model.x.shape}, expected (13, 1)"
    assert model.u.shape == (4, 1), f"u.shape == {model.u.shape}, expected (4, 1)"
    assert model.p.shape == (8, 1), f"p.shape == {model.p.shape}, expected (8, 1)"

    # f_expl must depend on all three symbols ...
    assert casadi.depends_on(model.f_expl_expr, model.x), "f_expl must depend on x"
    assert casadi.depends_on(model.f_expl_expr, model.u), "f_expl must depend on u"
    assert casadi.depends_on(
        model.f_expl_expr, model.p
    ), "f_expl must depend on p (wind + mass_scale enter through the drag term)"
    # ... and on nothing else: every free variable must be a component of x, u or p.
    free_vars = casadi.symvar(model.f_expl_expr)
    assert len(free_vars) > 0, "f_expl has no free variables — model is constant?"
    for sv in free_vars:
        assert (
            casadi.depends_on(sv, model.x)
            or casadi.depends_on(sv, model.u)
            or casadi.depends_on(sv, model.p)
        ), f"free variable {sv.name()} lies outside x/u/p — model has an undeclared input"


def test_symbolic_dynamics_match_cpp(repo_root):
    """The CasADi model and the C++ QuadrotorDynamics must be the same equations.

    The single most valuable test in the repo. If the two models drift, every solve-time
    number and every tracking plot is measuring the wrong thing.
    """
    probe = _find_dynamics_probe(repo_root)
    if probe is None:
        pytest.skip(
            "dynamics_probe binary not found — build tests first "
            "(colcon build --packages-select uav_mpc --cmake-args -DBUILD_TESTING=ON)"
        )

    constants = _constants(repo_root)
    model = quadrotor_model.export_quadrotor_model(constants, "quaternion")
    f_casadi = casadi.Function("f", [model.x, model.u, model.p], [model.f_expl_expr])
    p = _default_params()

    rng = np.random.RandomState(42)  # deterministic across runs
    n_samples = 200
    rows = []
    expected = []
    for _ in range(n_samples):
        pos = rng.uniform(-3.0, 3.0, 3)
        vel = rng.uniform(-5.0, 5.0, 3)
        quat = rng.randn(4)
        quat /= np.linalg.norm(quat)
        omega = rng.uniform(-4.0, 4.0, 3)
        u = rng.uniform(constants.min_thrust_per_rotor, constants.max_thrust_per_rotor, 4)
        x = np.concatenate([pos, vel, quat, omega])
        rows.append(" ".join(f"{v:.17g}" for v in np.concatenate([x, u])))
        expected.append(np.asarray(f_casadi(x, u, p)).reshape(-1))

    result = subprocess.run(
        [str(probe), str(_airframe_path(repo_root))],
        input="\n".join(rows) + "\n",
        capture_output=True,
        text=True,
        check=False,
    )
    assert (
        result.returncode == 0
    ), f"dynamics_probe failed ({result.returncode}): {result.stderr[:2000]}"

    lines = [ln for ln in result.stdout.splitlines() if ln.strip()]
    assert (
        len(lines) == n_samples
    ), f"dynamics_probe returned {len(lines)} rows, expected {n_samples}"

    worst_abs = 0.0
    worst_rel = 0.0
    for casadi_row, cpp_line in zip(expected, lines):
        cpp = np.array([float(v) for v in cpp_line.split()])
        assert (
            cpp.shape == casadi_row.shape
        ), f"shape mismatch: casadi {casadi_row.shape} vs cpp {cpp.shape}"
        diff = np.abs(cpp - casadi_row)
        worst_abs = max(worst_abs, float(diff.max()))
        with np.errstate(divide="ignore", invalid="ignore"):
            rel = diff / np.maximum(np.abs(casadi_row), 1e-12)
            worst_rel = max(worst_rel, float(rel.max()))
    assert worst_abs < 1e-9, (
        f"CasADi vs C++ dynamics drift: max abs diff {worst_abs:.3e} (limit 1e-9). "
        "quadrotor_model.py and quadrotor_dynamics.cpp must agree; fix both in one commit."
    )
    assert (
        worst_rel < 1e-9
    ), f"CasADi vs C++ dynamics drift: max rel diff {worst_rel:.3e} (limit 1e-9)"


def test_generated_code_is_up_to_date(repo_root):
    """The committed MODEL_HASH must match a fresh regeneration of the current model."""
    constants = _constants(repo_root)
    fresh = quadrotor_model.model_hash(constants, "quaternion")

    hash_file = repo_root / "codegen/MODEL_HASH"
    if not hash_file.is_file():
        pytest.fail(
            "codegen/MODEL_HASH is missing — generate the solver and commit the hash:\n"
            f"  python codegen/generate_acados_solver.py --airframe {_airframe_path(repo_root)} "
            f"--config {_config_path(repo_root)} --output codegen/codegen_output"
        )
    recorded = hash_file.read_text().strip()

    assert recorded == fresh, (
        "generated solver is stale: codegen/MODEL_HASH records "
        + recorded[:12]
        + " but the current model hashes to "
        + fresh[:12]
        + ". Regenerate with:\n"
        f"  python codegen/generate_acados_solver.py --airframe {_airframe_path(repo_root)} "
        f"--config {_config_path(repo_root)} --output codegen/codegen_output"
    )


def test_codegen_is_deterministic(tmp_path, repo_root):
    """Two runs of codegen must produce byte-identical C.

    Non-determinism (timestamps, dict ordering) must be eliminated at the source
    (strip_nondeterminism), not tolerated here.

    UNVERIFIED: AcadosOcp.to_dict() converts AcadosCodeGenOptions via its raw
    __dict__, which MAY embed the absolute code_export_directory path in the
    .json. The two runs below use different absolute paths (tmp_path/run1 vs
    run2), so if that path leaks into the JSON this test fails. If it does,
    FIX THE SOURCE per §10.3: extend strip_nondeterminism to normalise the
    export-dir path in JSON — do not loosen this test. (Cannot be verified
    until acados is installed; codegen/ACADOS_COMMIT is still a placeholder.)
    """
    constants = _constants(repo_root)
    cfg = _config(repo_root)
    model = quadrotor_model.export_quadrotor_model(constants, "quaternion")

    def run_once(tag: str) -> Path:
        out = tmp_path / tag
        ocp = generate_acados_solver.build_ocp(model, constants, cfg)
        ocp.code_export_directory = str(out / "c_generated_code")
        ocp.json_file = str(out / "acados_ocp_quadrotor.json")
        generate_acados_solver._generate(ocp, out, build=False)
        generate_acados_solver.strip_nondeterminism(out)
        generate_acados_solver.write_provenance(
            out,
            "0000000000000000000000000000000000000000",
            quadrotor_model.model_hash(constants, "quaternion"),
            _airframe_path(repo_root),
            _config_path(repo_root),
        )
        return out

    run1 = run_once("run1")
    run2 = run_once("run2")

    text_suffixes = {".c", ".h", ".json", ".txt"}
    files1 = {
        p.relative_to(run1) for p in run1.rglob("*") if p.is_file() and p.suffix in text_suffixes
    }
    files2 = {
        p.relative_to(run2) for p in run2.rglob("*") if p.is_file() and p.suffix in text_suffixes
    }
    assert files1 == files2, f"codegen is non-deterministic — file sets differ: {files1 ^ files2}"
    for rel in sorted(files1):
        a = (run1 / rel).read_bytes()
        b = (run2 / rel).read_bytes()
        assert a == b, f"codegen is non-deterministic — {rel} differs between two runs"


def test_ocp_dimensions_match_config(repo_root):
    """N, Tf, nx, nu in the generated code must match config/nmpc_params.yaml."""
    cfg = _config(repo_root)
    ocp_json = _generated_json(repo_root)

    dims = ocp_json.get("dims", {})
    n_json = dims.get("N")
    nx_json = dims.get("nx")
    nu_json = dims.get("nu")
    np_json = dims.get("np")
    assert n_json == int(
        cfg["horizon_steps"]
    ), f"N mismatch: generated {n_json} vs config {cfg['horizon_steps']}"
    assert nu_json == 4, f"nu mismatch: generated {nu_json} vs expected 4"

    nx_expected = 13  # quaternion attitude representation
    assert nx_json == nx_expected, f"nx mismatch: generated {nx_json} vs expected {nx_expected}"
    assert np_json == 8, f"np mismatch: generated {np_json} vs expected 8 (wind, mass_scale, q_ref)"

    tf = ocp_json.get("solver_options", {}).get("tf")
    if tf is None:  # defensive: some acados versions carry tf in dims
        tf = dims.get("tf")
    assert (
        tf is not None and abs(float(tf) - float(cfg["horizon_time"])) < 1e-9
    ), f"tf mismatch: generated {tf} vs config {cfg['horizon_time']}"

    # Header defines must agree too (generated C macros are the load-bearing interface).
    header = _generated_dir(repo_root) / "acados_solver_quadrotor.h"
    if header.is_file():
        text = header.read_text()
        for macro, expected in (("NX", nx_expected), ("NU", 4), ("NP", 8)):
            # acados < 0.6.0:  #define QUADROTOR_NX 13
            # acados >= 0.6.0: #define OCP_QUADROTOR_<HASH>_NX 13
            m = re.search(rf"#define\s+\S*{macro}\s+(\d+)", text)
            assert m, f"{macro} not found in {header}"
            assert (
                int(m.group(1)) == expected
            ), f"{macro} mismatch: header defines {m.group(1)}, expected {expected}"


def test_solver_options_are_realtime(repo_root):
    """Guards the RTI configuration against an accidental commit of a debug setting."""
    ocp_json = _generated_json(repo_root)
    so = ocp_json.get("solver_options", {})

    realtime_settings = {
        "nlp_solver_type": "SQP_RTI",
        "qp_solver": "PARTIAL_CONDENSING_HPIPM",
        "integrator_type": "ERK",
        "hessian_approx": "GAUSS_NEWTON",
        "globalization": "FIXED_STEP",
        "hpipm_mode": "SPEED",
    }
    for key, expected in realtime_settings.items():
        actual = so.get(key)
        assert actual == expected, (
            f"solver option {key}: generated {actual!r}, expected {expected!r} — "
            "a debug configuration was committed"
        )

    iter_max = so.get("qp_solver_iter_max")
    assert (
        isinstance(iter_max, int) and 0 < iter_max <= 100
    ), f"qp_solver_iter_max = {iter_max} is outside (0, 100]"
    assert (
        so.get("nlp_solver_max_iter") == 1
    ), "nlp_solver_max_iter must be 1 (single SQP iteration per sample, RTI)"
    num_stages = so.get("sim_method_num_stages")
    # acados >= 0.6.0: per-stage list; earlier: scalar
    if isinstance(num_stages, list):
        assert all(
            s == 4 for s in num_stages
        ), f"sim_method_num_stages: all entries must be 4 (RK4), got {num_stages}"
    else:
        assert num_stages == 4, "sim_method_num_stages must be 4 (RK4)"


@pytest.mark.slow
def test_generated_solver_solves_hover(repo_root):
    """End-to-end: the generated solver, driven from Python, must hold a hover.

    Uses the generated C solver via acados_template with a Python RK4 integration of the
    same CasADi model as the plant — 200 closed-loop steps (10 s). The position must
    converge to the reference within 1e-2 m and every solve must return status 0.
    """
    from acados_template import AcadosOcpSolver

    constants = _constants(repo_root)
    cfg = _config(repo_root)
    model = quadrotor_model.export_quadrotor_model(constants, "quaternion")
    ocp = generate_acados_solver.build_ocp(model, constants, cfg)

    generated = _generated_dir(repo_root)
    if not (generated / "acados_solver_quadrotor.c").is_file():
        pytest.fail(
            "generated solver missing — run:\n"
            f"  python codegen/generate_acados_solver.py --airframe {_airframe_path(repo_root)} "
            f"--config {_config_path(repo_root)} --output codegen/codegen_output"
        )

    # Point the formulation at the committed solver and load it *without* regenerating.
    # `check_reuse_possible=False` guarantees this test never writes into the repo.
    ocp.code_export_directory = str(generated)
    solver = AcadosOcpSolver(
        ocp, build=False, generate=False, check_reuse_possible=False, verbose=False
    )

    n = int(cfg["horizon_steps"])  # 20
    dt = float(cfg["horizon_time"]) / n  # 50 ms
    nx = model.x.shape[0]
    ny = 16
    ny_e = 12

    pref = np.array([1.0, 1.0, 1.5])
    hover_thrust = constants.mass * constants.gravity / 4.0  # 4.9033 N per rotor
    p = _default_params()

    # yref: [p; v; e_q; omega; u], yref_e: [p; v; e_q; omega].
    yref = np.zeros(ny)
    yref[:3] = pref
    yref[3:6] = 0.0
    yref[6:9] = 0.0  # identity q_ref => zero attitude error at reference
    yref[9:12] = 0.0
    yref[12:16] = hover_thrust
    yref_e = np.zeros(ny_e)
    yref_e[:3] = pref

    f_plant = casadi.Function("plant", [model.x, model.u, model.p], [model.f_expl_expr])

    def plant_step(x: np.ndarray, u: np.ndarray) -> np.ndarray:
        x = np.asarray(x, dtype=float).reshape(-1)
        k1 = np.asarray(f_plant(x, u, p)).reshape(-1)
        k2 = np.asarray(f_plant(x + 0.5 * dt * k1, u, p)).reshape(-1)
        k3 = np.asarray(f_plant(x + 0.5 * dt * k2, u, p)).reshape(-1)
        k4 = np.asarray(f_plant(x + dt * k3, u, p)).reshape(-1)
        x_new = x + dt / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        quat = x_new[6:10]
        x_new[6:10] = quat / np.linalg.norm(quat)
        return x_new

    def set_initial_state(state: np.ndarray) -> None:
        state = np.asarray(state, dtype=float).reshape(-1)
        # UNVERIFIED: canonical acados usage is constraints_set(0, "lbx_0", ...)
        # rather than the generic set(); the fallback below covers both until the
        # pinned acados is available to check. Same for solver.set(n, "yref", ...)
        # below — the terminal reference may need cost_set(n, "yref_e", ...).
        try:
            solver.set(0, "lbx_0", state)
            solver.set(0, "ubx_0", state)
        except Exception:
            solver.set(0, "lbx", state)
            solver.set(0, "ubx", state)

    # Start 0.1 m off the reference so the test measures convergence, not a no-op.
    x = np.zeros(nx)
    x[:3] = pref - 0.1
    x[3:6] = 0.0
    x[6] = 1.0  # level attitude
    x[10:13] = 0.0
    set_initial_state(x)

    statuses = []
    for _ in range(200):
        for stage in range(n):
            solver.set(stage, "yref", yref)
        solver.set(n, "yref", yref_e)  # terminal cost reference (yref_e at N)
        for stage in range(n + 1):
            solver.set(stage, "p", p)
        set_initial_state(x)
        statuses.append(solver.solve())
        u0 = np.asarray(solver.get(0, "u")).reshape(-1)
        x = plant_step(x, u0)
        # RTI shift: reuse the previous solution as the next warm start (§6.5).
        for stage in range(n - 1):
            solver.set(stage, "x", np.asarray(solver.get(stage + 1, "x")).reshape(-1))
            solver.set(stage, "u", np.asarray(solver.get(stage + 1, "u")).reshape(-1))
        solver.set(n - 1, "x", x)
        solver.set(n - 1, "u", hover_thrust * np.ones(4))

    err = float(np.linalg.norm(x[:3] - pref))
    assert err < 1e-2, f"closed-loop hover diverged: final position error {err:.4e} m (limit 1e-2)"
    failures = [i for i, s in enumerate(statuses) if s != 0]
    assert (
        not failures
    ), f"non-zero solver status at steps {failures[:10]}... ({len(failures)} of 200)"
