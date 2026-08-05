"""SKELETON — no implementation. See .deepseek/11_CODEGEN.md §11.1.

CasADi symbolic model of the quadrotor, exported as an `AcadosModel`.

THE INVARIANT: these equations must be identical to
uav_mpc/src/quadrotor_dynamics.cpp. test_acados_codegen.py enforces it numerically.
When you change one, change both in the same commit.

State  (quaternion, nx = 13):  x = [p(3), v(3), q(4, wxyz), omega(3)]     world ENU, body FLU
State  (euler,      nx = 12):  x = [p(3), v(3), rpy(3),     omega(3)]
Input  (nu = 4):               u = [T1, T2, T3, T4]  per-rotor thrust [N]
Params (np = 8):               p = [wind(3), mass_scale, q_ref(4, wxyz)]

`q_ref` lives in the parameter vector because the NONLINEAR_LS cost residual contains the
error quaternion q_ref^-1 (x) q, which is a function of both the state and the reference —
acados' `yref` mechanism alone cannot express it. See .deepseek/06_SOLVER.md §6.3.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class AirframeConstants:
    """Numeric constants baked into the generated C.

    Anything in here requires a re-run of codegen to change — keep it to true structural
    constants. Runtime-tunable quantities belong in the online parameter vector `p`.

    TODO(deepseek): fields mirroring uav_mpc::QuadrotorParams
    (mass, ixx/iyy/izz, arm_length, thrust_coeff, torque_coeff, drag_coeff, gravity,
    min/max thrust per rotor), plus a `from_yaml(path)` classmethod that reads
    uav_mpc/params/<frame>_calibration.yaml so there is ONE source of truth.
    """


def allocation_matrix(constants: AirframeConstants):
    """Return the 4x4 quad-X allocation matrix mapping rotor thrusts to [T, tau_x, tau_y, tau_z].

    TODO(deepseek): must be numerically identical to
    QuadrotorDynamics::buildAllocationMatrix(). See §4.3 for the row/column convention and the
    motor numbering diagram — PX4's quad-X numbering is NOT the intuitive clockwise order.
    """
    raise NotImplementedError


def quaternion_kinematics(q, omega):
    """qdot = 0.5 * q (x) [0, omega], Hamilton convention, wxyz ordering.

    TODO(deepseek): implement with casadi.vertcat. Do not use casadi's built-in quaternion
    helpers — spell it out so the derivation in docs/ matches the code line for line.
    """
    raise NotImplementedError


def euler_kinematics(rpy, omega):
    """ZYX Euler rates from body rates.

    TODO(deepseek): the T(rpy) matrix. Note the 1/cos(pitch) singularity — this branch is for
    analysis and comparison only, never for the flight solver.
    """
    raise NotImplementedError


def rotation_from_quaternion(q):
    """Body-to-world rotation matrix from a wxyz quaternion.

    TODO(deepseek): implement symbolically; do not normalise here (the OCP keeps ||q|| = 1 via
    the dynamics plus the norm penalty in the cost).
    """
    raise NotImplementedError


def export_quadrotor_model(constants: AirframeConstants, attitude_rep: str = "quaternion"):
    """Build and return the `AcadosModel`.

    TODO(deepseek):
      1. declare SX symbols x, u, p and xdot with the layout documented at the top of this file
      2. build f_expl:
             pdot     = v
             vdot     = (1/(mass_scale*m)) * (R @ [0,0,sum(T)] - D_body_applied) + [0,0,-g]
                        with rotor drag D applied in the BODY frame:
                        R @ (diag(drag) @ (R.T @ (v - wind)))
             qdot     = quaternion_kinematics(q, omega)
             omegadot = J^-1 @ (tau - cross(omega, J @ omega))
      3. f_impl = xdot - f_expl
      4. model.name = "quadrotor"   (the generated symbols depend on this — do not change it
         without updating acados_wrapper.cpp's include and CMakeLists)
      5. set model.x/xdot/u/p/f_expl_expr/f_impl_expr
      6. attach the constraint expression for the quaternion norm if §6.3 selects the soft
         constraint variant
    Returns the model. Raises ValueError on an unknown attitude_rep.
    """
    raise NotImplementedError


def model_hash(constants: AirframeConstants, attitude_rep: str = "quaternion") -> str:
    """Stable SHA-256 over the model expressions and structural constants.

    TODO(deepseek): serialise the CasADi expressions with `str()` on the SX graph plus the
    sorted constants dict, hash it, return the hex digest. Must be deterministic across runs
    and machines — no `id()`, no dict iteration order, no floats formatted with repr().
    Written to codegen/MODEL_HASH and compiled into the generated code as
    UAV_MPC_MODEL_HASH.
    """
    raise NotImplementedError


if __name__ == "__main__":
    # TODO(deepseek): tiny self-check — build the model, print the dimensions and the hash.
    raise SystemExit("quadrotor_model.py not implemented")
