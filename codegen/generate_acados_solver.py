"""SKELETON — no implementation. See IMPLEMENTATION_GUIDE.md §11.2.

Generates the acados SQP-RTI C solver into codegen/codegen_output/.

    python codegen/generate_acados_solver.py \
        --airframe uav_mpc/params/x500_calibration.yaml \
        --config   uav_mpc/config/nmpc_params.yaml \
        --output   codegen/codegen_output

Called manually, by the CMake custom command when the model is newer than the generated code,
and by CI. Output MUST be deterministic — test_acados_codegen.py diffs two runs.

Requires ACADOS_SOURCE_DIR in the environment and acados_template installed from that tree
(see requirements.txt). The pinned acados commit lives in codegen/ACADOS_COMMIT; the script
refuses to run against a different commit unless --allow-acados-mismatch is passed.
"""

from __future__ import annotations

import argparse


def parse_args() -> argparse.Namespace:
    """TODO(deepseek): --airframe, --config, --output, --attitude-rep, --allow-acados-mismatch,
    --check-only (regenerate into a temp dir and diff, exit non-zero on drift; this is what CI
    runs)."""
    raise NotImplementedError


def check_acados_version(allow_mismatch: bool) -> str:
    """Verify the installed acados matches codegen/ACADOS_COMMIT.

    TODO(deepseek): `git -C $ACADOS_SOURCE_DIR rev-parse HEAD`, compare, and either raise with
    the exact checkout command or warn. Return the commit hash for the provenance header.
    """
    raise NotImplementedError


def build_ocp(model, constants, config: dict):
    """Assemble the `AcadosOcp`.

    TODO(deepseek): the settings of §6.2 — every one of these is a decision, not a default:

      dims:      N  = config["horizon_steps"]   (20)
                 Tf = config["horizon_time"]    (1.0 s)  => dt = 50 ms

      cost:      cost_type   = "NONLINEAR_LS"  (LINEAR_LS cannot express the quaternion error)
                 cost_type_e = "NONLINEAR_LS"
                 y_expr   = vertcat(p, v, quat_error_vec(q, q_ref), omega, u)
                 y_expr_e = vertcat(p, v, quat_error_vec(q, q_ref), omega)
                 W, W_e   = diag from the YAML weights; yref set at runtime
                 NOTE: with a NONLINEAR_LS quaternion-error residual, q_ref must enter through
                 the online parameter vector — extend np accordingly and document it. §6.3
                 explains the alternative (LINEAR_LS on the vector part) and why it is rejected.

      constraints:
                 lbu = min_thrust_per_rotor, ubu = max_thrust_per_rotor  (per rotor, hard)
                 idxbu = [0, 1, 2, 3]
                 lbx/ubx on the body rates (|omega| <= 6 rad/s) as SOFT constraints with an
                 L2 slack penalty — hard state constraints cause RTI infeasibility in flight
                 x0 handled by the initial-state equality (set at runtime)

      solver:    nlp_solver_type          = "SQP_RTI"
                 qp_solver                = "PARTIAL_CONDENSING_HPIPM"
                 qp_solver_cond_N         = 5
                 hessian_approx           = "GAUSS_NEWTON"
                 integrator_type          = "ERK", sim_method_num_stages = 4,
                                            sim_method_num_steps = 1
                 qp_solver_iter_max       = 50
                 nlp_solver_max_iter      = 1
                 globalization             = "FIXED_STEP"
                 print_level              = 0
                 hpipm_mode               = "SPEED"

      codegen:   code_export_directory = <output>/c_generated_code
                 json_file             = <output>/acados_ocp_quadrotor.json
    """
    raise NotImplementedError


def write_provenance(output_dir, acados_commit: str, model_hash_value: str) -> None:
    """Write MODEL_HASH and a provenance header next to the generated code.

    TODO(deepseek): emit
      codegen/MODEL_HASH                       — the bare hex digest, newline-terminated
      <output>/include/model_hash.h            — #define UAV_MPC_MODEL_HASH "<digest>"
      <output>/GENERATION_PROVENANCE.txt       — acados commit, casadi version, airframe file,
                                                 config file, and the hashes of both inputs.
                                                 NO timestamps (they break determinism).
    """
    raise NotImplementedError


def strip_nondeterminism(output_dir) -> None:
    """Remove generation timestamps from the emitted C so two runs are byte-identical.

    TODO(deepseek): acados writes a date into some headers. Rewrite those lines. Keep the
    substitution narrow and assert that it matched, so a future acados version that stops
    emitting the line does not silently no-op.
    """
    raise NotImplementedError


def main() -> int:
    """TODO(deepseek): parse args -> check acados -> load airframe + config -> build model ->
    build ocp -> AcadosOcpSolver(ocp, generate=True, build=True) -> strip_nondeterminism ->
    write_provenance. In --check-only mode, generate into a temp dir and diff against the
    committed tree, returning 1 on any difference with a readable diff summary."""
    raise NotImplementedError


if __name__ == "__main__":
    raise SystemExit(main())
