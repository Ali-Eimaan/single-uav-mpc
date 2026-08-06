"""CasADi symbolic model of the quadrotor, exported as an `AcadosModel`.
See .deepseek/11_CODEGEN.md §11.1.

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

from dataclasses import dataclass, field
import hashlib
from pathlib import Path

import casadi as cs
import numpy as np
import yaml

from acados_template import AcadosModel


@dataclass
class AirframeConstants:
    """Numeric constants baked into the generated C.

    Anything in here requires a re-run of codegen to change — keep it to true structural
    constants. Runtime-tunable quantities belong in the online parameter vector `p`.

    Fields mirror uav_mpc::QuadrotorParams (include/uav_mpc/quadrotor_dynamics.hpp) one
    for one. `from_yaml()` reads the same params/*_calibration.yaml the C++ reads, so there
    is ONE source of truth.
    """

    frame_name: str = ""
    mass: float = 0.0                       # [kg]
    inertia: np.ndarray = field(default_factory=lambda: np.zeros((3, 3)))  # [kg m^2]
    arm_length: float = 0.0                 # [m]
    thrust_coeff: float = 0.0               # [N/(rad/s)^2]
    torque_coeff: float = 0.0               # [N m/(rad/s)^2]
    rotor_time_constant: float = 0.0        # [s]
    drag_coeff: np.ndarray = field(default_factory=lambda: np.zeros(3))    # [N s/m] body frame
    gravity: float = 9.80665                # [m/s^2]
    min_thrust_per_rotor: float = 0.0       # [N]
    max_thrust_per_rotor: float = 0.0       # [N]

    @classmethod
    def from_yaml(cls, yaml_path: str | Path) -> "AirframeConstants":
        """Load the same airframe calibration YAML the C++ node loads.

        Raises ValueError with the key *and* file named (matching the C++ error strings), so a
        mismatch between the two parsers shows up as drift the tests catch rather than a
        silent default.
        """
        path = Path(yaml_path)
        try:
            doc = yaml.safe_load(path.read_text())
        except OSError as exc:
            raise ValueError(f"airframe params: cannot read {path}: {exc}") from exc
        except yaml.YAMLError as exc:
            raise ValueError(f"airframe params: invalid YAML in {path}: {exc}") from exc

        if not isinstance(doc, dict) or "airframe" not in doc:
            raise ValueError(f"airframe params: missing key 'airframe' in {path}")
        af = doc["airframe"]
        if not isinstance(af, dict):
            raise ValueError(f"airframe params: 'airframe' must be a mapping in {path}")

        def req(key: str):
            if key not in af:
                raise ValueError(f"airframe params: missing key 'airframe.{key}' in {path}")
            return af[key]

        frame_name = str(req("name"))
        mass = float(req("mass"))
        arm_length = float(req("arm_length"))
        thrust_coeff = float(req("thrust_coeff"))
        torque_coeff = float(req("torque_coeff"))
        rotor_time_constant = float(req("rotor_time_constant"))
        min_thrust = float(req("min_thrust_per_rotor"))
        max_thrust = float(req("max_thrust_per_rotor"))

        # inertia: accepted as a map (ixx/iyy/izz/ixy/ixz/iyz), a 3-vector (diagonal), or a
        # 9-vector (row-major) — identical to QuadrotorParams::fromYaml.
        inertia_raw = req("inertia")
        inertia = np.zeros((3, 3))
        if isinstance(inertia_raw, dict):
            diag_keys = {"ixx": (0, 0), "iyy": (1, 1), "izz": (2, 2)}
            offdiag = {"ixy": (0, 1), "ixz": (0, 2), "iyz": (1, 2)}
            for key, (r, c) in diag_keys.items():
                if key in inertia_raw:
                    inertia[r, c] = float(inertia_raw[key])
            for key, (r, c) in offdiag.items():
                if key in inertia_raw:
                    val = float(inertia_raw[key])
                    inertia[r, c] = val
                    inertia[c, r] = val
        else:
            seq = [float(v) for v in inertia_raw]
            if len(seq) == 3:
                np.fill_diagonal(inertia, seq)
            elif len(seq) == 9:
                inertia = np.array(seq).reshape((3, 3))
            else:
                raise ValueError(
                    f"airframe params: 'airframe.inertia' must be a 3- or 9-element "
                    f"sequence in {path}")

        drag_raw = req("drag_coeff")
        drag = np.array([float(v) for v in drag_raw])
        if drag.shape != (3,):
            raise ValueError(
                f"airframe params: 'airframe.drag_coeff' must have 3 entries in {path}")

        gravity = float(af["gravity"]) if "gravity" in af else 9.80665

        return cls(
            frame_name=frame_name,
            mass=mass,
            inertia=inertia,
            arm_length=arm_length,
            thrust_coeff=thrust_coeff,
            torque_coeff=torque_coeff,
            rotor_time_constant=rotor_time_constant,
            drag_coeff=drag,
            gravity=gravity,
            min_thrust_per_rotor=min_thrust,
            max_thrust_per_rotor=max_thrust,
        )


def allocation_matrix(constants: AirframeConstants) -> np.ndarray:
    """Return the 4x4 quad-X allocation matrix mapping rotor thrusts to [T, tau_x, tau_y, tau_z].

    Must be numerically identical to QuadrotorDynamics::buildAllocationMatrix()
    (uav_mpc/src/quadrotor_dynamics.cpp). Layout per §4.3 — PX4's quad-X convention:

        row 0: T     = T1 + T2 + T3 + T4
        row 1: tau_x = -d T1 + d T2 + d T3 - d T4
        row 2: tau_y = -d T1 + d T2 - d T3 + d T4
        row 3: tau_z = -c T1 - c T2 + c T3 + c T4

    Motor order: 1 front-right CCW, 2 rear-left CCW, 3 front-left CW, 4 rear-right CW.
    d = arm_length / sqrt(2), c = torque_coeff / thrust_coeff.
    """
    d = constants.arm_length / np.sqrt(2.0)
    c = (constants.torque_coeff / constants.thrust_coeff
         if constants.thrust_coeff > 0.0 else 0.0)
    return np.array([
        [1.0, 1.0, 1.0, 1.0],
        [-d, +d, +d, -d],
        [-d, +d, -d, +d],
        [-c, -c, +c, +c],
    ])


def quaternion_kinematics(q, omega):
    """qdot = 0.5 * q (x) [0, omega], Hamilton convention, wxyz ordering.

    Spelled out component-wise so docs/derivations/ matches line for line; deliberately NOT
    using casadi's built-in quaternion helpers.
    """
    qw, qx, qy, qz = q[0], q[1], q[2], q[3]
    wx, wy, wz = omega[0], omega[1], omega[2]
    return cs.vertcat(
        -0.5 * (qx * wx + qy * wy + qz * wz),
        +0.5 * (qw * wx + qy * wz - qz * wy),
        +0.5 * (qw * wy + qz * wx - qx * wz),
        +0.5 * (qw * wz + qx * wy - qy * wx),
    )


def euler_kinematics(rpy, omega):
    """ZYX Euler rates from body rates: rpy_dot = T(rpy) @ omega.

    T = [[1, s_phi s_theta/c_theta, c_phi s_theta/c_theta],
         [0, c_phi,                 -s_phi],
         [0, s_phi/c_theta,          c_phi/c_theta]]

    Note the 1/cos(pitch) singularity — this branch is for analysis and comparison only,
    never for the flight solver.
    """
    phi, theta, _ = rpy[0], rpy[1], rpy[2]
    s_phi, c_phi = cs.sin(phi), cs.cos(phi)
    s_th, c_th = cs.sin(theta), cs.cos(theta)
    one_over_cth = 1.0 / c_th
    return cs.vertcat(
        omega[0] + omega[1] * s_phi * s_th * one_over_cth + omega[2] * c_phi * s_th * one_over_cth,
        omega[1] * c_phi - omega[2] * s_phi,
        omega[1] * s_phi * one_over_cth + omega[2] * c_phi * one_over_cth,
    )


def rotation_from_quaternion(q):
    """Body-to-world rotation matrix from a wxyz quaternion, explicit component form.

    Not normalised here — the OCP keeps ||q|| = 1 via the dynamics plus the re-normalisation
    in the C++ integrator (uav_mpc/src/quadrotor_dynamics.cpp step()).
    """
    qw, qx, qy, qz = q[0], q[1], q[2], q[3]
    return cs.blockcat([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
        [2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx)],
        [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx * qx + qy * qy)],
    ])


def _rotation_from_euler(rpy):
    """Body-to-world rotation from ZYX (yaw-pitch-roll) Euler angles; matches the C++ helper."""
    phi, theta, psi = rpy[0], rpy[1], rpy[2]
    cp, sp = cs.cos(phi), cs.sin(phi)
    ct, st = cs.cos(theta), cs.sin(theta)
    cy, sy = cs.cos(psi), cs.sin(psi)
    return cs.blockcat([
        [cy * ct, cy * st * sp - sy * cp, cy * st * cp + sy * sp],
        [sy * ct, sy * st * sp + cy * cp, sy * st * cp - cy * sp],
        [-st, ct * sp, ct * cp],
    ])


def export_quadrotor_model(constants: AirframeConstants, attitude_rep: str = "quaternion"):
    """Build and return the `AcadosModel`.

    Raises ValueError on an unknown attitude_rep.
    """
    if attitude_rep not in ("quaternion", "euler"):
        raise ValueError(f"unknown attitude_rep '{attitude_rep}' (expected 'quaternion'|'euler')")

    nx = 13 if attitude_rep == "quaternion" else 12
    nu = 4
    np_ = 8

    x = cs.SX.sym("x", nx)
    u = cs.SX.sym("u", nu)
    p = cs.SX.sym("p", np_)
    xdot = cs.SX.sym("xdot", nx)

    # --- slice the layout ---------------------------------------------------------------
    v = x[3:6]
    omega = x[10:13]
    if attitude_rep == "quaternion":
        q = x[6:10]
        R = rotation_from_quaternion(q)
    else:
        rpy = x[6:9]
        R = _rotation_from_euler(rpy)

    # --- online parameters --------------------------------------------------------------
    wind = p[0:3]
    mass_scale = p[3]
    # p[4:8] = q_ref — not used by the dynamics, only by the cost residual in
    # generate_acados_solver.py. Kept here so the layout is defined in exactly one place.

    # --- forces and torques -------------------------------------------------------------
    F_b = cs.vertcat(0.0, 0.0, cs.sum1(u))       # collective thrust, body +z
    tau = allocation_matrix(constants) @ u        # [T; tau_x; tau_y; tau_z]
    tau_b = tau[1:4]

    m = constants.mass
    J = constants.inertia
    g = constants.gravity
    drag_diag = constants.drag_coeff

    # Rotor drag is linear in the BODY-frame airspeed; wind is a world-frame disturbance, so
    # the relative velocity enters as R^T (v - wind).
    drag_body = R @ cs.diag(drag_diag) @ (R.T @ (v - wind))

    # --- dynamics ----------------------------------------------------------------------
    p_dot = v
    v_dot = (1.0 / (mass_scale * m)) * (R @ F_b - drag_body) + cs.vertcat(0.0, 0.0, -g)
    if attitude_rep == "quaternion":
        q_dot = quaternion_kinematics(q, omega)
    else:
        q_dot = euler_kinematics(rpy, omega)
    omega_dot = cs.solve(J, tau_b - cs.cross(omega, J @ omega))

    f_expl = cs.vertcat(p_dot, v_dot, q_dot, omega_dot)
    f_impl = xdot - f_expl

    model = AcadosModel()
    model.name = "quadrotor"          # load-bearing: generated symbols derive from it (§11.1)
    model.x = x
    model.xdot = xdot
    model.u = u
    model.p = p
    model.f_expl_expr = f_expl
    model.f_impl_expr = f_impl

    # No quaternion-norm constraint is attached: q_dot = 0.5 q (x) [0, omega] preserves ||q||
    # for the continuous dynamics and the C++ step() re-normalises after RK4; a soft norm
    # constraint would fight the shortest-arc attitude cost of §6.3.

    return model


def _format_constants(constants: AirframeConstants) -> str:
    """Deterministic fixed-precision serialisation of the structural constants."""
    parts = []
    for key in sorted(vars(constants)):
        value = getattr(constants, key)
        if isinstance(value, np.ndarray):
            entries = ",".join(f"{v:.17g}" for v in value.reshape(-1))
            parts.append(f"{key}=[{entries}]")
        else:
            parts.append(f"{key}={value:.17g}")
    return ";".join(parts)


def model_hash(constants: AirframeConstants, attitude_rep: str = "quaternion") -> str:
    """Stable SHA-256 over the model expressions and structural constants.

    Deterministic across runs and machines: `str()` of the SX graph (no id(), no repr() of
    floats) plus the constants serialised with sorted keys and `f"{v:.17g}"` precision.
    Written to codegen/MODEL_HASH and compiled into the generated code as UAV_MPC_MODEL_HASH.
    """
    model = export_quadrotor_model(constants, attitude_rep)
    payload = (
        f"attitude_rep={attitude_rep}\n"
        f"f_expl={str(model.f_expl_expr)}\n"
        f"f_impl={str(model.f_impl_expr)}\n"
        f"constants={_format_constants(constants)}"
    )
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Self-check the symbolic quadrotor model.")
    parser.add_argument(
        "--yaml",
        default=str(Path(__file__).resolve().parents[1] / "uav_mpc/params/x500_calibration.yaml"),
        help="airframe calibration YAML",
    )
    parser.add_argument("--attitude-rep", default="quaternion", choices=["quaternion", "euler"])
    args = parser.parse_args()

    consts = AirframeConstants.from_yaml(args.yaml)
    model = export_quadrotor_model(consts, args.attitude_rep)
    print(f"frame_name   : {consts.frame_name}")
    print(f"name         : {model.name}")
    print(f"nx/nu/np     : {model.x.shape[0]}/{model.u.shape[0]}/{model.p.shape[0]}")
    print(f"attitude_rep : {args.attitude_rep}")
    print(f"MODEL_HASH   : {model_hash(consts, args.attitude_rep)}")
