"""Generates the acados SQP-RTI C solver into codegen/codegen_output/.
 and 06_SOLVER.md §6.2–6.4.

    python codegen/generate_acados_solver.py \
        --airframe uav_mpc/params/x500_calibration.yaml \
        --config   uav_mpc/config/nmpc_params.yaml \
        --output   codegen/codegen_output

Called manually, by the CMake custom command when the model is newer than the generated code,
and by CI (--check-only regenerates into a temp dir and diffs, exiting non-zero on drift).
Output MUST be deterministic — test_acados_codegen.py diffs two runs.

Requires ACADOS_SOURCE_DIR in the environment and acados_template installed from that tree
(see requirements.txt). The pinned acados commit lives in codegen/ACADOS_COMMIT; the script
refuses to run against a different commit unless --allow-acados-mismatch is passed.
"""

from __future__ import annotations

import argparse
import hashlib
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import warnings

import yaml

# Suppress acados_template deprecation warnings during code generation.
# The acados 0.6.0 API is used throughout; these are known low-severity warnings.
warnings.filterwarnings("ignore", category=DeprecationWarning)
warnings.filterwarnings("ignore", category=UserWarning)

# noqa: E402 — the warning filters above must be installed before quadrotor_model imports
# casadi, or acados' DeprecationWarnings escape to stderr and pollute --check-only diffs.
from quadrotor_model import (  # noqa: E402
    AirframeConstants,
    export_quadrotor_model,
    model_hash,
)

HERE = Path(__file__).resolve().parent
ACADOS_COMMIT_FILE = HERE / "ACADOS_COMMIT"

# Known nondeterminism sources in the acados-generated tree (UNVERIFIED against the pinned
# acados — confirm the exact emission points when ACADOS_COMMIT is pinned, see §11.2):
#   * a "generated on <date>" comment in acados_solver_quadrotor.c
#   * a "time_stamp" field in the emitted JSON
DATE_RE = re.compile(rb"generated on\s+\d{4}-\d{2}-\d{2}[ T]\d{2}:\d{2}:\d{2}")
JSON_TIME_STAMP_RE = re.compile(rb'"time_stamp"\s*:\s*"?\d+(\.\d+)?"?')
GENERATED_ON_RE = re.compile(rb"\d{4}-\d{2}-\d{2}[ T]\d{2}:\d{2}:\d{2}")
CODE_EXPORT_DIR_RE = re.compile(rb'"code_export_directory"\s*:\s*"[^"]*"')
JSON_FILE_RE = re.compile(rb'"json_file"\s*:\s*"[^"]*"')
# acados >= 0.6.0 includes a top-level "hash" that is computed from the OCP
# configuration (including path-dependent fields); normalise it to a constant
# after stripping path-dependent strings so two runs are byte-identical.
HASH_RE = re.compile(rb'"hash"\s*:\s*"[0-9a-fA-F]{32}"')


def load_nmpc_config(path: Path) -> dict:
    """Load `config/nmpc_params.yaml`, unwrapping the ROS 2 param-file wrapper.

    The node loads this file as a ROS 2 parameter file, so it carries the standard
    `/**:` / `ros__parameters:` wrapper; codegen needs the flat mapping underneath.
    A flat YAML (no wrapper) is also accepted, so the helper is safe with both forms.
    """
    try:
        doc = yaml.safe_load(path.read_text())
    except OSError as exc:
        raise RuntimeError(f"cannot read config {path}: {exc}") from exc
    except yaml.YAMLError as exc:
        raise RuntimeError(f"invalid YAML in config {path}: {exc}") from exc
    if not isinstance(doc, dict):
        raise RuntimeError(f"config {path} must be a YAML mapping")
    block = doc.get("/**")
    if isinstance(block, dict) and isinstance(block.get("ros__parameters"), dict):
        return block["ros__parameters"]
    return doc


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--airframe", default=str(HERE.parent / "uav_mpc/params/x500_calibration.yaml")
    )
    parser.add_argument("--config", default=str(HERE.parent / "uav_mpc/config/nmpc_params.yaml"))
    parser.add_argument("--output", default=str(HERE / "codegen_output"))
    parser.add_argument("--attitude-rep", default="quaternion", choices=["quaternion", "euler"])
    parser.add_argument(
        "--allow-acados-mismatch",
        action="store_true",
        help="proceed even if the installed acados does not match codegen/ACADOS_COMMIT",
    )
    parser.add_argument(
        "--check-only",
        action="store_true",
        help="regenerate into a temp dir and diff against the committed tree; exit 1 on drift",
    )
    return parser.parse_args()


def check_acados_version(allow_mismatch: bool) -> str:
    """Verify the installed acados matches codegen/ACADOS_COMMIT; return its commit SHA."""
    source_dir = os.environ.get("ACADOS_SOURCE_DIR")
    if not source_dir:
        raise RuntimeError(
            "ACADOS_SOURCE_DIR is not set. Install acados from source and export "
            "ACADOS_SOURCE_DIR."
        )
    result = subprocess.run(
        ["git", "-C", source_dir, "rev-parse", "HEAD"], capture_output=True, text=True, check=False
    )
    if result.returncode != 0:
        raise RuntimeError(f"cannot resolve acados HEAD in {source_dir}: {result.stderr.strip()}")

    installed = result.stdout.strip()
    pinned = None
    for line in ACADOS_COMMIT_FILE.read_text().splitlines():
        line = line.strip()
        if re.fullmatch(r"[0-9a-f]{40}", line):
            pinned = line
            break
    if pinned is None:
        raise RuntimeError(
            f"{ACADOS_COMMIT_FILE} does not contain a 40-char commit SHA; pin acados before "
            f"generating."
        )

    if installed != pinned:
        msg = (
            f"installed acados {installed} != pinned {pinned} (see {ACADOS_COMMIT_FILE}). "
            f"Check out the pinned commit and reinstall acados_template:\n"
            f"  git -C {source_dir} checkout {pinned}\n"
            f"  pip install -e {source_dir}/interfaces/acados_template\n"
            f"or pass --allow-acados-mismatch to proceed anyway."
        )
        if not allow_mismatch:
            raise RuntimeError(msg)
        print(f"WARNING: {msg}", file=sys.stderr)
    return installed


def quat_error_vec(q, q_ref):
    """Vector part of the shortest-arc error quaternion q_ref^-1 (x) q, Hamilton wxyz.

    The vector part is negated when the scalar part is negative (avoids the unwinding
    phenomenon, §6.3). The sign is a piecewise-smooth function of the state; Gauss-Newton
    only needs the residual and its (almost everywhere smooth) derivative.
    """
    import casadi as cs

    qw, qx, qy, qz = q[0], q[1], q[2], q[3]
    rw, rx, ry, rz = q_ref[0], q_ref[1], q_ref[2], q_ref[3]
    e_w = rw * qw + rx * qx + ry * qy + rz * qz
    e_x = rw * qx - rx * qw - ry * qz + rz * qy
    e_y = rw * qy + rx * qz - ry * qw - rz * qx
    e_z = rw * qz - rx * qy + ry * qx - rz * qw
    s = cs.if_else(e_w >= 0, 1.0, -1.0)
    return cs.vertcat(s * e_x, s * e_y, s * e_z)


def build_ocp(model, constants: AirframeConstants, config: dict):
    """Assemble the `AcadosOcp` from the settings of §6.2–6.4 (decisions, not defaults)."""
    from acados_template import AcadosOcp
    import casadi as cs
    import numpy as np

    ocp = AcadosOcp()
    ocp.model = model

    nx = model.x.shape[0]
    nu = model.u.shape[0]

    # --- dimensions --------------------------------------------------------------
    ocp.dims.N = int(config["horizon_steps"])  # 20
    ocp.solver_options.tf = float(config["horizon_time"])  # 1.0 s -> dt = 50 ms

    # --- cost: NONLINEAR_LS (§6.3) ------------------------------------------------
    q = model.x[6:10]
    omega = model.x[10:13]
    q_ref = model.p[4:8]
    y_expr = cs.vertcat(
        model.x[0:3],  # p
        model.x[3:6],  # v
        quat_error_vec(q, q_ref),  # shortest-arc attitude error
        omega,
        model.u,  # u
    )
    y_expr_e = cs.vertcat(
        model.x[0:3],
        model.x[3:6],
        quat_error_vec(q, q_ref),
        omega,
    )
    ny = y_expr.shape[0]  # 16
    ny_e = y_expr_e.shape[0]  # 12

    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.cost.cost_type_e = "NONLINEAR_LS"
    # acados >= 0.6.0: cost_y_expr lives on model, not cost.
    ocp.model.cost_y_expr = y_expr
    ocp.model.cost_y_expr_e = y_expr_e

    q_diag = np.array(config["q_diag"], dtype=float)  # 12 entries
    r_diag = np.array(config["r_diag"], dtype=float)  # 4 entries
    q_terminal_diag = np.array(config["q_terminal_diag"], dtype=float)
    assert q_diag.shape == (12,), f"q_diag must have 12 entries, got {q_diag.shape}"
    assert r_diag.shape == (4,), f"r_diag must have 4 entries, got {r_diag.shape}"
    assert q_terminal_diag.shape == (
        12,
    ), f"q_terminal_diag must have 12 entries, got {q_terminal_diag.shape}"

    W = np.diag(np.concatenate([q_diag, r_diag]))  # 16x16
    W_e = np.diag(q_terminal_diag)  # 12x12
    # Path stages 0..N-1 (loop is still needed for the per-stage indexing).
    # acados 0.6.0: copy_path_cost_to_stage_0() sets cost_type_0, y_expr_0,
    # W_0, yref_0 from the path cost automatically.
    for k in range(ocp.dims.N):
        ocp.cost.W = W
        ocp.cost.yref = np.zeros(ny)
    ocp.cost.W_e = W_e
    ocp.cost.yref_e = np.zeros(ny_e)

    # --- constraints (§6.4) --------------------------------------------------------
    min_thrust = constants.min_thrust_per_rotor
    max_thrust = constants.max_thrust_per_rotor

    rate_idx = 10 if nx == 13 else 9
    omega_max = 6.0
    # Soft L2 slack on body-rate bounds.
    zl_rate = np.ones(3) * 1e1
    zu_rate = np.ones(3) * 1e1
    Zl_rate = np.ones(3) * 1e2
    Zu_rate = np.ones(3) * 1e2

    lbx = -omega_max * np.ones(3)
    ubx = +omega_max * np.ones(3)
    rate_idxs = np.arange(rate_idx, rate_idx + 3)

    for k in range(ocp.dims.N):
        ocp.constraints.idxbu = np.arange(nu)
        ocp.constraints.lbu = min_thrust * np.ones(nu)
        ocp.constraints.ubu = max_thrust * np.ones(nu)
        # Body-rate soft bounds only.
        ocp.constraints.idxbx = rate_idxs
        ocp.constraints.lbx = lbx
        ocp.constraints.ubx = ubx
        ocp.constraints.idxsbx = np.arange(3)
        ocp.cost.zl = zl_rate
        ocp.cost.zu = zu_rate
        ocp.cost.Zl = Zl_rate
        ocp.cost.Zu = Zu_rate
    # acados >= 0.6.0: initial stage (nsbx not supported at stage 0).
    ocp.constraints.idxbx_0 = np.arange(nx)
    ocp.constraints.lbx_0 = np.zeros(nx)
    ocp.constraints.ubx_0 = np.zeros(nx)
    ocp.constraints.idxbu_0 = np.arange(nu)
    ocp.constraints.lbu_0 = min_thrust * np.ones(nu)
    ocp.constraints.ubu_0 = max_thrust * np.ones(nu)
    ocp.constraints.idxsbx_0 = np.array([], dtype=int)
    ocp.cost.zl_0 = np.zeros(0)
    ocp.cost.zu_0 = np.zeros(0)
    ocp.cost.Zl_0 = np.zeros(0)
    ocp.cost.Zu_0 = np.zeros(0)
    ocp.constraints.idxbx_e = rate_idxs
    ocp.constraints.lbx_e = lbx
    ocp.constraints.ubx_e = ubx
    ocp.constraints.idxsbx_e = np.arange(3)
    ocp.cost.zl_e = zl_rate
    ocp.cost.zu_e = zu_rate
    ocp.cost.Zl_e = Zl_rate
    ocp.cost.Zu_e = Zu_rate

    # Default online parameters: zero wind, mass_scale = 1, identity q_ref.
    ocp.parameter_values = np.array([0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0])

    # --- solver options (§6.2) ------------------------------------------------------
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.qp_solver_cond_N = 5
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 1
    ocp.solver_options.qp_solver_iter_max = 50
    ocp.solver_options.nlp_solver_max_iter = 1
    ocp.solver_options.globalization = "FIXED_STEP"
    ocp.solver_options.print_level = 0
    ocp.solver_options.hpipm_mode = "SPEED"

    return ocp


def write_provenance(
    output_dir: Path,
    acados_commit: str,
    model_hash_value: str,
    airframe_path: Path,
    config_path: Path,
) -> None:
    """Emit MODEL_HASH, include/model_hash.h and GENERATION_PROVENANCE.txt (no timestamps)."""
    include_dir = output_dir / "include"
    include_dir.mkdir(parents=True, exist_ok=True)

    # codegen/MODEL_HASH — bare hex digest, newline-terminated.
    (output_dir.parent / "MODEL_HASH").write_text(model_hash_value + "\n")

    # <output>/include/model_hash.h
    (include_dir / "model_hash.h").write_text(
        "#ifndef UAV_MPC__MODEL_HASH_H_\n"
        "#define UAV_MPC__MODEL_HASH_H_\n"
        f'#define UAV_MPC_MODEL_HASH "{model_hash_value}"\n'
        "#endif  // UAV_MPC__MODEL_HASH_H_\n"
    )

    # <output>/GENERATION_PROVENANCE.txt
    def sha256_of(path: Path) -> str:
        return hashlib.sha256(Path(path).read_bytes()).hexdigest()

    import casadi as cs

    provenance = (
        f"acados_commit: {acados_commit}\n"
        f"casadi_version: {cs.__version__}\n"
        f"airframe_file: {airframe_path}\n"
        f"airframe_sha256: {sha256_of(airframe_path)}\n"
        f"config_file: {config_path}\n"
        f"config_sha256: {sha256_of(config_path)}\n"
        f"model_hash: {model_hash_value}\n"
        f"generator: {Path(__file__).name}\n"
        f"generator_sha256: {sha256_of(__file__)}\n"
    )
    (output_dir / "GENERATION_PROVENANCE.txt").write_text(provenance)


def strip_nondeterminism(output_dir: Path) -> None:
    """Remove generation timestamps from the emitted C/JSON so two runs are byte-identical.

    The substitutions are narrow and the post-condition is asserted: if any date-like string
    survives, a future acados that emits a different format fails loudly instead of silently
    no-op'ing (§11.2).
    """
    generated = output_dir / "c_generated_code"
    if not generated.is_dir():
        raise FileNotFoundError(f"{generated} does not exist")

    replaced = 0
    for path in sorted(generated.rglob("*")):
        if not path.is_file():
            continue
        if path.suffix in (".c", ".h", ".json", ".txt"):
            data = path.read_bytes()
            new_data, n1 = DATE_RE.subn(b"generated on 1970-01-01 00:00:00", data)
            new_data, n2 = JSON_TIME_STAMP_RE.subn(b'"time_stamp": 0', new_data)
            new_data, n3 = GENERATED_ON_RE.subn(b"1970-01-01 00:00:00", new_data)
            # acados >= 0.5.6 embeds the code_export_directory absolute path in
            # JSON; normalise it so two runs to different temp dirs are identical.
            new_data, n4 = CODE_EXPORT_DIR_RE.subn(
                b'"code_export_directory": "/codegen/c_generated_code"', new_data
            )
            new_data, n5 = JSON_FILE_RE.subn(
                b'"json_file": "/codegen/acados_ocp_quadrotor.json"', new_data
            )
            # acados >= 0.6.0 stores a top-level hash that includes
            # path-dependent fields — re-compute after normalising them.
            new_data, n6 = HASH_RE.subn(b'"hash": "00000000000000000000000000000000"', new_data)
            replaced += n1 + n2 + n3 + n4 + n5 + n6
            if new_data != data:
                path.write_bytes(new_data)

    # Post-condition: the generated tree must contain no date-like strings at all.
    leftovers = []
    for path in sorted(generated.rglob("*")):
        if path.is_file() and path.suffix in (".c", ".h", ".json", ".txt"):
            if GENERATED_ON_RE.search(path.read_bytes()):
                leftovers.append(str(path))
    if leftovers:
        raise RuntimeError(
            "strip_nondeterminism: date-like strings remain in generated files: "
            + ", ".join(leftovers)
        )


def _diff_trees(committed: Path, fresh: Path) -> str:
    """Recursive text diff of two trees; returns '' when identical.

    Build artifacts (.so/.o/… and anything under a build/ dir) are ignored — the committed
    tree was built (build=True) while --check-only regenerates with build=False, so those
    files legitimately differ and would be noise.
    """

    def relevant(path: Path):
        rel = path.relative_to(committed) if path.is_relative_to(committed) else path
        rel_str = str(rel).replace("\\", "/")
        if "/build/" in rel_str or rel_str.startswith("build/"):
            return False
        return path.suffix not in (".so", ".o", ".a", ".dylib", ".dll", ".exe")

    lines = []
    committed_files = {
        p.relative_to(committed) for p in committed.rglob("*") if p.is_file() and relevant(p)
    }
    fresh_files = {p.relative_to(fresh) for p in fresh.rglob("*") if p.is_file() and relevant(p)}
    for rel in sorted(committed_files - fresh_files):
        lines.append(f"only in committed: {rel}")
    for rel in sorted(fresh_files - committed_files):
        lines.append(f"only in fresh:     {rel}")
    for rel in sorted(committed_files & fresh_files):
        a = (committed / rel).read_bytes()
        b = (fresh / rel).read_bytes()
        if a != b:
            lines.append(f"differs: {rel} (committed {len(a)} B vs fresh {len(b)} B)")
    return "\n".join(lines)


def main() -> int:
    args = parse_args()

    output_dir = Path(args.output).resolve()
    airframe_path = Path(args.airframe).resolve()
    config_path = Path(args.config).resolve()

    acados_commit = check_acados_version(args.allow_acados_mismatch)

    constants = AirframeConstants.from_yaml(airframe_path)
    config = load_nmpc_config(config_path)
    model = export_quadrotor_model(constants, args.attitude_rep)
    hash_value = model_hash(constants, args.attitude_rep)

    if args.check_only:
        with tempfile.TemporaryDirectory(prefix="uav_mpc_codegen_check_") as tmp:
            tmp_out = Path(tmp) / "codegen_output"
            ocp = build_ocp(model, constants, config)
            ocp.code_export_directory = str(tmp_out / "c_generated_code")
            ocp.json_file = str(tmp_out / "acados_ocp_quadrotor.json")
            _generate(ocp, tmp_out, build=False)
            strip_nondeterminism(tmp_out)
            write_provenance(tmp_out, acados_commit, hash_value, airframe_path, config_path)
            diff = _diff_trees(output_dir, tmp_out)
            # MODEL_HASH lives one level above the generated tree (codegen/MODEL_HASH),
            # so compare it separately.
            committed_hash = output_dir.parent / "MODEL_HASH"
            if committed_hash.is_file() and committed_hash.read_text().strip() != hash_value:
                diff += (
                    "\ndiffers: MODEL_HASH ("
                    f"committed {committed_hash.read_text().strip()} vs fresh {hash_value})"
                )
        if diff:
            print("codegen drift detected — regenerate with:")
            print(
                f"  python {Path(__file__).name} --airframe {args.airframe} "
                f"--config {args.config} --output {args.output}"
            )
            print("differences:")
            print(diff)
            return 1
        print("codegen is up to date.")
        return 0

    ocp = build_ocp(model, constants, config)
    ocp.code_gen_options.code_export_directory = str(output_dir / "c_generated_code")
    ocp.code_gen_options.json_file = str(output_dir / "acados_ocp_quadrotor.json")
    _generate(ocp, output_dir, build=True)
    strip_nondeterminism(output_dir)
    write_provenance(output_dir, acados_commit, hash_value, airframe_path, config_path)
    print(f"generated solver in {output_dir}")
    print(f"MODEL_HASH={hash_value}")
    return 0


def _generate(ocp, output_dir: Path, build: bool) -> None:
    """Run acados_template code generation (and optionally the C build)."""
    from acados_template import AcadosOcpSolver

    output_dir.mkdir(parents=True, exist_ok=True)
    AcadosOcpSolver(ocp, generate=True, build=build)
    shutil.rmtree(output_dir / "build", ignore_errors=True)
    _symlink_hashed_outputs(output_dir / "c_generated_code")


def _symlink_hashed_outputs(gen_dir: Path) -> None:
    """Create plain-name symlinks to the hashed generated files.

    acados_template appends a content hash to every generated file (e.g.
    ``acados_solver_ocp_quadrotor_eefd7658.h``). The consuming C++ code
    and CMakeLists.txt expect stable names without the hash suffix so they
    don't need updating every time the model changes.
    """
    import glob as _glob
    import re as _re

    patterns = {
        "acados_solver_quadrotor": "acados_solver_ocp_quadrotor_*",
        "libacados_ocp_solver_quadrotor": "libacados_ocp_solver_ocp_quadrotor_*",
    }
    hash_str = ""
    for plain, pattern in patterns.items():
        for ext in (".h", ".c", ".so"):
            matches = sorted(_glob.glob(str(gen_dir / (pattern + ext)), root_dir=str(gen_dir)))
            if matches:
                hashed = gen_dir / matches[0]
                link = gen_dir / (plain + ext)
                if link.is_symlink() or link.exists():
                    link.unlink()
                link.symlink_to(hashed.name)
                print(f"  symlink: {link.name} -> {hashed.name}")
                if not hash_str and ext == ".h":
                    # Extract hash from e.g. acados_solver_ocp_quadrotor_<hash>.h
                    m = _re.search(r"ocp_quadrotor_([0-9a-f]+)\.h", hashed.name)
                    if m:
                        hash_str = m.group(1)

    # acados >= 0.6.0 uses OCP_<MODEL>_<HASH>_NX etc; older code references
    # QUADROTOR_NX etc. Provide compatibility aliases.
    if hash_str:
        compat_header = gen_dir / "acados_solver_compat.h"
        suffix = f"OCP_QUADROTOR_{hash_str.upper()}_"
        compat_header.write_text(
            f"""\
// Auto-generated compatibility aliases for acados >= 0.6.0 naming convention.
// Generated by codegen/generate_acados_solver.py — DO NOT EDIT.
#ifndef ACADOS_SOLVER_COMPAT_H_
#define ACADOS_SOLVER_COMPAT_H_

#include "acados_solver_quadrotor.h"

// Dimension macros
#define QUADROTOR_NX  ({suffix}NX)
#define QUADROTOR_NU  ({suffix}NU)
#define QUADROTOR_NP  ({suffix}NP)
#define QUADROTOR_N   ({suffix}N)
#define QUADROTOR_TF  (({suffix}N) * 0.05)  /* N * Ts; Ts defaults to 0.05 */
#define QUADROTOR_NBX ({suffix}NBX)
#define QUADROTOR_NBU ({suffix}NBU)
#define QUADROTOR_NY  ({suffix}NY)
#define QUADROTOR_NYN ({suffix}NYN)

// Function and type aliases — acados 0.6.0 hashes every entry point.
#define quadrotor_solver_capsule       ocp_quadrotor_{hash_str}_solver_capsule
#define quadrotor_acados_create_capsule  ocp_quadrotor_{hash_str}_acados_create_capsule
#define quadrotor_acados_free_capsule    ocp_quadrotor_{hash_str}_acados_free_capsule
#define quadrotor_acados_create          ocp_quadrotor_{hash_str}_acados_create
#define quadrotor_acados_solve           ocp_quadrotor_{hash_str}_acados_solve
#define quadrotor_acados_free            ocp_quadrotor_{hash_str}_acados_free
#define quadrotor_acados_update_params   ocp_quadrotor_{hash_str}_acados_update_params
#define quadrotor_acados_get_nlp_config  ocp_quadrotor_{hash_str}_acados_get_nlp_config
#define quadrotor_acados_get_nlp_dims    ocp_quadrotor_{hash_str}_acados_get_nlp_dims
#define quadrotor_acados_get_nlp_in      ocp_quadrotor_{hash_str}_acados_get_nlp_in
#define quadrotor_acados_get_nlp_out     ocp_quadrotor_{hash_str}_acados_get_nlp_out
#define quadrotor_acados_get_nlp_solver  ocp_quadrotor_{hash_str}_acados_get_nlp_solver
#define quadrotor_acados_get_nlp_opts    ocp_quadrotor_{hash_str}_acados_get_nlp_opts
#define quadrotor_acados_print_stats     ocp_quadrotor_{hash_str}_acados_print_stats
#define quadrotor_acados_reset           ocp_quadrotor_{hash_str}_acados_reset

#endif  // ACADOS_SOLVER_COMPAT_H_
"""
        )
        print(f"  written: {compat_header.name}")


if __name__ == "__main__":
    raise SystemExit(main())
