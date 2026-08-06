// Copyright (c) 2026 Ali-Eimaan.
// SPDX-License-Identifier: BSD-3-Clause
//
// Implementation of the templated rigid-body dynamics declared in
// include/uav_mpc/quadrotor_dynamics.hpp. See .deepseek/04_DYNAMICS.md §4.

#include "uav_mpc/quadrotor_dynamics.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <functional>
#include <stdexcept>
#include <string>

namespace uav_mpc
{

namespace
{

/// Skew-symmetric matrix [a]_x such that [a]_x b == a x b.
template<typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> skewMatrix(
  const Eigen::MatrixBase<Derived> & a)
{
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
  const auto & a0 = a(0);
  const auto & a1 = a(1);
  const auto & a2 = a(2);
  Eigen::Matrix<typename Derived::Scalar, 3, 3> S;
  S << 0, -a2, a1,
       a2, 0, -a0,
       -a1, a0, 0;
  return S;
}

/// Body-to-world rotation from ZYX (yaw-pitch-roll) Euler angles, R = Rz(psi) Ry(theta) Rx(phi).
template<typename Scalar>
Eigen::Matrix<Scalar, 3, 3> eulerRotationZyx(const Eigen::Matrix<Scalar, 3, 1> & rpy)
{
  const Scalar phi = rpy(0), theta = rpy(1), psi = rpy(2);
  const Scalar cp = std::cos(theta), sp = std::sin(theta);
  const Scalar cr = std::cos(phi), sr = std::sin(phi);
  const Scalar cy = std::cos(psi), sy = std::sin(psi);
  Eigen::Matrix<Scalar, 3, 3> R;
  R(0, 0) = cy * cp; R(0, 1) = cy * sp * sr - sy * cr; R(0, 2) = cy * sp * cr + sy * sr;
  R(1, 0) = sy * cp; R(1, 1) = sy * sp * sr + cy * cr; R(1, 2) = sy * sp * cr - cy * sr;
  R(2, 0) = -sp;     R(2, 1) = cp * sr;                 R(2, 2) = cp * cr;
  return R;
}

/// Read a required double from a YAML node, throwing a message that names the key and the file.
std::function<double(const YAML::Node &, const char *)> requireDouble(const std::string & path)
{
  return [&path](const YAML::Node & node, const char * key) -> double {
    const YAML::Node n = node[key];
    if (!n || !n.IsDefined()) {
      throw std::runtime_error(
        "quadrotor params: missing key 'airframe." + std::string(key) + "' in " + path);
    }
    return n.as<double>();
  };
}

}  // namespace

// ================================================================================================
// QuadrotorParams
// ================================================================================================

QuadrotorParams QuadrotorParams::fromYaml(const std::string & yaml_path)
{
  YAML::Node doc = YAML::LoadFile(yaml_path);
  const YAML::Node root = doc["airframe"];
  if (!root || !root.IsDefined()) {
    throw std::runtime_error(
      "quadrotor params: missing key 'airframe' in " + yaml_path);
  }

  auto getd = requireDouble(yaml_path);

  QuadrotorParams p;
  if (root["name"] && root["name"].IsDefined()) {
    p.frame_name = root["name"].as<std::string>();
  } else {
    throw std::runtime_error(
      "quadrotor params: missing key 'airframe.name' in " + yaml_path);
  }
  p.mass = getd(root, "mass");
  p.arm_length = getd(root, "arm_length");
  p.thrust_coeff = getd(root, "thrust_coeff");
  p.torque_coeff = getd(root, "torque_coeff");
  p.min_thrust_per_rotor = getd(root, "min_thrust_per_rotor");
  p.max_thrust_per_rotor = getd(root, "max_thrust_per_rotor");
  p.rotor_time_constant = getd(root, "rotor_time_constant");
  if (root["gravity"] && root["gravity"].IsDefined()) {
    p.gravity = root["gravity"].as<double>();
  }
  // else: keep the 9.80665 default.

  const YAML::Node inertia = root["inertia"];
  if (!inertia || !inertia.IsDefined()) {
    throw std::runtime_error(
      "quadrotor params: missing key 'airframe.inertia' in " + yaml_path);
  }
  if (inertia.IsMap()) {
    // Named-entry form used by params/*_calibration.yaml: ixx, iyy, izz, ixy, ixz, iyz.
    p.inertia(0, 0) = getd(inertia, "ixx");
    p.inertia(1, 1) = getd(inertia, "iyy");
    p.inertia(2, 2) = getd(inertia, "izz");
    p.inertia(0, 1) = p.inertia(1, 0) = getd(inertia, "ixy");
    p.inertia(0, 2) = p.inertia(2, 0) = getd(inertia, "ixz");
    p.inertia(1, 2) = p.inertia(2, 1) = getd(inertia, "iyz");
  } else if (inertia.IsSequence() && inertia.size() == 3) {
    // Diagonal shorthand: [ixx, iyy, izz].
    p.inertia(0, 0) = inertia[0].as<double>();
    p.inertia(1, 1) = inertia[1].as<double>();
    p.inertia(2, 2) = inertia[2].as<double>();
  } else if (inertia.IsSequence() && inertia.size() == 9) {
    // Full row-major 3x3.
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        p.inertia(r, c) = inertia[3 * r + c].as<double>();
      }
    }
  } else {
    throw std::runtime_error(
      "quadrotor params: 'airframe.inertia' must be a map (ixx/iyy/izz/...), a 3-vector "
      "(diagonal) or a 9-vector (row-major) in " + yaml_path);
  }

  const YAML::Node drag = root["drag_coeff"];
  if (!drag || !drag.IsDefined() || !drag.IsSequence() || drag.size() != 3) {
    throw std::runtime_error(
      "quadrotor params: missing key 'airframe.drag_coeff' (3-vector) in " + yaml_path);
  }
  for (int i = 0; i < 3; ++i) {
    p.drag_coeff[i] = drag[i].as<double>();
  }

  std::string why;
  if (!p.isValid(&why)) {
    throw std::runtime_error(
      "quadrotor params: invalid airframe in " + yaml_path + ": " + why);
  }
  return p;
}

bool QuadrotorParams::isValid(std::string * why) const
{
  auto fail = [&why](const std::string & msg) {
    if (why != nullptr) {*why = msg;}
    return false;
  };

  if (!(mass > 0.0)) {return fail("mass must be > 0");}
  if (!inertia.isApprox(inertia.transpose())) {return fail("inertia must be symmetric");}
  Eigen::LDLT<Eigen::Matrix3d> ldlt(inertia);
  if (ldlt.info() != Eigen::Success) {return fail("inertia LDLT factorisation failed");}
  if (!(ldlt.vectorD().array() > 0.0).all()) {return fail("inertia must be positive definite");}
  if (!(thrust_coeff > 0.0)) {return fail("thrust_coeff must be > 0");}
  if (!(torque_coeff > 0.0)) {return fail("torque_coeff must be > 0");}
  if (!(arm_length > 0.0)) {return fail("arm_length must be > 0");}
  if (!(min_thrust_per_rotor >= 0.0 && min_thrust_per_rotor < max_thrust_per_rotor)) {
    return fail("require 0 <= min_thrust_per_rotor < max_thrust_per_rotor");
  }
  if (!(4.0 * max_thrust_per_rotor > mass * gravity)) {
    return fail("thrust-to-weight ratio must exceed 1 (4 * max_thrust_per_rotor > mass * g)");
  }
  return true;
}

// ================================================================================================
// QuadrotorDynamics
// ================================================================================================

template<typename Scalar, AttitudeRep Rep>
QuadrotorDynamics<Scalar, Rep>::QuadrotorDynamics(const QuadrotorParams & params)
: params_(params)
{
  if (!params_.isValid()) {
    throw std::invalid_argument("QuadrotorDynamics: invalid QuadrotorParams");
  }
  buildAllocationMatrix();
}

template<typename Scalar, AttitudeRep Rep>
void QuadrotorDynamics<Scalar, Rep>::buildAllocationMatrix()
{
  // PX4 quad-X allocation (§4.3). Motor numbering is PX4's, NOT the intuitive clockwise order:
  //   1 front-right (+d,-d) CCW | 2 rear-left (-d,+d) CCW | 3 front-left (+d,+d) CW |
  //   4 rear-right (-d,-d) CW,  with d = arm_length/sqrt(2), c = torque_coeff/thrust_coeff.
  //
  // VERIFY before trusting (see §4.3): cross-check this table against the pinned PX4 quad-X
  // mixer and the Gazebo model SDF. A wrong sign here flips the vehicle on takeoff and looks
  // like "the MPC is unstable".
  const Scalar d = params_.arm_length / std::sqrt(Scalar(2));
  const Scalar c = params_.torque_coeff / params_.thrust_coeff;
  allocation_ <<  Scalar(1),  Scalar(1),  Scalar(1),  Scalar(1),
                 -d,  d,  d, -d,
                 -d,  d, -d,  d,
                 -c, -c,  c,  c;
  allocation_inverse_ = allocation_.inverse();
  inertia_inv_ = params_.inertia.template cast<Scalar>().inverse();
}

template<typename Scalar, AttitudeRep Rep>
typename QuadrotorDynamics<Scalar, Rep>::StateVector
QuadrotorDynamics<Scalar, Rep>::f(const StateVector & x, const InputVector & u) const
{
  // Continuous-time dynamics of §4.2 (world ENU, body FLU, gravity along -z_world):
  //   p_dot     = v
  //   v_dot     = (1/m) * R * (F_b - D * R^T * v) + g_w      (drag linear in BODY velocity)
  //   q_dot     = 0.5 * q (x) [0, omega]                     (or T(rpy) * omega for Euler)
  //   omega_dot = J^-1 * (tau - omega x (J * omega))
  StateVector xd = StateVector::Zero();

  constexpr int kPos = StateLayout<Rep>::kPosIdx;
  constexpr int kVel = StateLayout<Rep>::kVelIdx;
  constexpr int kAtt = StateLayout<Rep>::kAttIdx;
  constexpr int kRate = StateLayout<Rep>::kRateIdx;

  const Eigen::Matrix<Scalar, 3, 1> v = x.template segment<3>(kVel);
  const Eigen::Matrix<Scalar, 3, 1> w = x.template segment<3>(kRate);
  const Eigen::Matrix<Scalar, 3, 3> R = rotationMatrix(x);
  const Eigen::Matrix<Scalar, 3, 1> g_w(0, 0, -params_.gravity);

  // Translational.
  xd.template segment<3>(kPos) = v;
  const Scalar sumT = u.sum();
  const Eigen::Matrix<Scalar, 3, 1> F_b(0, 0, sumT);
  const Eigen::Matrix<Scalar, 3, 1> D = params_.drag_coeff.template cast<Scalar>();
  xd.template segment<3>(kVel) =
    (Scalar(1) / params_.mass) * (R * (F_b - D.cwiseProduct(R.transpose() * v))) + g_w;

  // Attitude kinematics.
  if constexpr (Rep == AttitudeRep::Quaternion) {
    const Eigen::Matrix<Scalar, 4, 1> q = x.template segment<4>(kAtt);
    const Eigen::Quaternion<Scalar> quat(q(0), q(1), q(2), q(3));
    const Eigen::Quaternion<Scalar> omega_q(Scalar(0), w(0), w(1), w(2));
    Eigen::Quaternion<Scalar> qd;
    qd.coeffs() = Scalar(0.5) * quatMultiply(quat, omega_q).coeffs();
    xd.template segment<4>(kAtt) << qd.w(), qd.x(), qd.y(), qd.z();
  } else {
    // ZYX Euler rates: Theta_dot = T(Theta) * omega, singular at pitch = +/-pi/2.
    const Eigen::Matrix<Scalar, 3, 1> rpy = x.template segment<3>(kAtt);
    const Scalar phi = rpy(0), theta = rpy(1);
    const Scalar sphi = std::sin(phi), cphi = std::cos(phi);
    const Scalar st = std::sin(theta), ct = std::cos(theta);
    Eigen::Matrix<Scalar, 3, 3> T;
    T << Scalar(1), sphi * st / ct, cphi * st / ct,
         Scalar(0), cphi, -sphi,
         Scalar(0), sphi / ct, cphi / ct;
    xd.template segment<3>(kAtt) = T * w;
  }

  // Rotational.
  const Eigen::Matrix<Scalar, 3, 1> tau = allocation_.template block<3, 4>(1, 0) * u;
  const Eigen::Matrix<Scalar, 3, 1> Jw = params_.inertia.template cast<Scalar>() * w;
  xd.template segment<3>(kRate) = inertia_inv_ * (tau - w.cross(Jw));

  return xd;
}

template<typename Scalar, AttitudeRep Rep>
typename QuadrotorDynamics<Scalar, Rep>::StateVector
QuadrotorDynamics<Scalar, Rep>::step(
  const StateVector & x, const InputVector & u, Scalar dt) const
{
  // Classical RK4, single step. The exact quaternion flow preserves ||q|| but the RK4
  // discretisation does not, so re-normalise on exit (Rep == Quaternion).
  const StateVector k1 = f(x, u);
  const StateVector k2 = f(x + (Scalar(0.5) * dt) * k1, u);
  const StateVector k3 = f(x + (Scalar(0.5) * dt) * k2, u);
  const StateVector k4 = f(x + dt * k3, u);
  StateVector x1 = x + (dt / Scalar(6)) * (k1 + Scalar(2) * k2 + Scalar(2) * k3 + k4);
  if constexpr (Rep == AttitudeRep::Quaternion) {
    constexpr int kAtt = StateLayout<Rep>::kAttIdx;
    x1.template segment<4>(kAtt).normalize();
  }
  return x1;
}

template<typename Scalar, AttitudeRep Rep>
void QuadrotorDynamics<Scalar, Rep>::jacobians(
  const StateVector & x, const InputVector & u, StateJacobian * A, InputJacobian * B) const
{
  // Analytic df/dx and df/du. Verified against central differences in test/test_dynamics.cpp
  // (step 1e-6, relative tolerance 1e-6). Not used by acados (which has its own AD).
  A->setZero();
  B->setZero();

  constexpr int kPos = StateLayout<Rep>::kPosIdx;
  constexpr int kVel = StateLayout<Rep>::kVelIdx;
  constexpr int kAtt = StateLayout<Rep>::kAttIdx;
  constexpr int kRate = StateLayout<Rep>::kRateIdx;

  const Scalar m = params_.mass;
  const Eigen::Matrix<Scalar, 3, 1> v = x.template segment<3>(kVel);
  const Eigen::Matrix<Scalar, 3, 1> w = x.template segment<3>(kRate);
  const Eigen::Matrix<Scalar, 3, 3> R = rotationMatrix(x);
  const Eigen::Matrix<Scalar, 3, 3> I3 = Eigen::Matrix<Scalar, 3, 3>::Identity();
  const Eigen::Matrix<Scalar, 3, 1> D = params_.drag_coeff.template cast<Scalar>();
  const Scalar sumT = u.sum();
  const Eigen::Matrix<Scalar, 3, 1> F_b(0, 0, sumT);
  const Eigen::Matrix<Scalar, 3, 1> t = F_b - D.cwiseProduct(R.transpose() * v);  // F_b - D R^T v
  const Eigen::Matrix<Scalar, 3, 3> J = params_.inertia.template cast<Scalar>();

  // p_dot = v.
  A->template block<3, 3>(kPos, kVel) = I3;

  // v_dot wrt v: -(1/m) R D R^T.
  A->template block<3, 3>(kVel, kVel) = -(Scalar(1) / m) * R * D.asDiagonal() * R.transpose();

  if constexpr (Rep == AttitudeRep::Quaternion) {
    // v_dot wrt q. With G = R * (F_b - D R^T v):
    //   dG/dq_i = (dR/dq_i) t - R D (dR/dq_i)^T v
    const Scalar qw = x(kAtt);
    const Eigen::Matrix<Scalar, 3, 1> qv = x.template segment<3>(kAtt + 1);

    const Eigen::Matrix<Scalar, 3, 3> dRdw = Scalar(2) * qw * I3 + Scalar(2) * skewMatrix(qv);
    A->template block<3, 1>(kVel, kAtt) =
      (Scalar(1) / m) * (dRdw * t - R * (D.cwiseProduct(dRdw.transpose() * v)));

    for (int j = 0; j < 3; ++j) {
      Eigen::Matrix<Scalar, 3, 1> ej = Eigen::Matrix<Scalar, 3, 1>::Zero();
      ej(j) = Scalar(1);
      const Eigen::Matrix<Scalar, 3, 3> dRdq =
        -Scalar(2) * qv(j) * I3 + Scalar(2) * (ej * qv.transpose() + qv * ej.transpose()) +
        Scalar(2) * qw * skewMatrix(ej);
      A->template block<3, 1>(kVel, kAtt + 1 + j) =
        (Scalar(1) / m) * (dRdq * t - R * (D.cwiseProduct(dRdq.transpose() * v)));
    }

    // q_dot wrt q:  f_q = 0.5 * [-qv.w ; qw*w + qv x w].
    A->operator()(kAtt, kAtt) = Scalar(0);
    A->operator()(kAtt, kAtt + 1) = -Scalar(0.5) * w(0);
    A->operator()(kAtt, kAtt + 2) = -Scalar(0.5) * w(1);
    A->operator()(kAtt, kAtt + 3) = -Scalar(0.5) * w(2);
    A->template block<3, 1>(kAtt + 1, kAtt) = Scalar(0.5) * w;
    A->template block<3, 1>(kAtt + 1, kAtt + 1) =
      Scalar(0.5) * Eigen::Matrix<Scalar, 3, 1>(0, w(2), -w(1));
    A->template block<3, 1>(kAtt + 1, kAtt + 2) =
      Scalar(0.5) * Eigen::Matrix<Scalar, 3, 1>(-w(2), 0, w(0));
    A->template block<3, 1>(kAtt + 1, kAtt + 3) =
      Scalar(0.5) * Eigen::Matrix<Scalar, 3, 1>(w(1), -w(0), 0);

    // q_dot wrt omega:  0.5 * L(q)[:, 1:4], L(q) the left-multiplication matrix of q.
    Eigen::Matrix<Scalar, 4, 3> dqdw;
    dqdw << -qv(0), -qv(1), -qv(2),
            qw, -qv(2), qv(1),
            qv(2), qw, -qv(0),
            -qv(1), qv(0), qw;
    A->template block<4, 3>(kAtt, kRate) = Scalar(0.5) * dqdw;
  } else {
    // v_dot wrt rpy for the Euler branch. R = Rz Ry Rx, dR = dRz Ry Rx + Rz dRy Rx + Rz Ry dRx.
    const Eigen::Matrix<Scalar, 3, 1> rpy = x.template segment<3>(kAtt);
    const Scalar phi = rpy(0), theta = rpy(1), psi = rpy(2);
    const Scalar cp = std::cos(phi), sp = std::sin(phi);
    const Scalar ct = std::cos(theta), st = std::sin(theta);
    const Scalar cy = std::cos(psi), sy = std::sin(psi);

    Eigen::Matrix<Scalar, 3, 3> Rx;
    Rx << Scalar(1), 0, 0, 0, cp, -sp, 0, sp, cp;
    Eigen::Matrix<Scalar, 3, 3> Ry;
    Ry << ct, 0, st, 0, Scalar(1), 0, -st, 0, ct;
    Eigen::Matrix<Scalar, 3, 3> Rz;
    Rz << cy, -sy, 0, sy, cy, 0, 0, 0, Scalar(1);

    Eigen::Matrix<Scalar, 3, 3> dRx;
    dRx << 0, 0, 0, 0, -sp, -cp, 0, cp, -sp;
    Eigen::Matrix<Scalar, 3, 3> dRy;
    dRy << -st, 0, cp, 0, 0, 0, -cp, 0, -st;
    Eigen::Matrix<Scalar, 3, 3> dRz;
    dRz << -sy, -cy, 0, cy, -sy, 0, 0, 0, 0;

    const Eigen::Matrix<Scalar, 3, 3> dR_dphi = Rz * Ry * dRx;
    const Eigen::Matrix<Scalar, 3, 3> dR_dtheta = Rz * dRy * Rx;
    const Eigen::Matrix<Scalar, 3, 3> dR_dpsi = dRz * Ry * Rx;

    A->template block<3, 1>(kVel, kAtt + 0) =
      (Scalar(1) / m) * (dR_dphi * t - R * (D.cwiseProduct(dR_dphi.transpose() * v)));
    A->template block<3, 1>(kVel, kAtt + 1) =
      (Scalar(1) / m) * (dR_dtheta * t - R * (D.cwiseProduct(dR_dtheta.transpose() * v)));
    A->template block<3, 1>(kVel, kAtt + 2) =
      (Scalar(1) / m) * (dR_dpsi * t - R * (D.cwiseProduct(dR_dpsi.transpose() * v)));

    // rpy_dot wrt rpy:  M_ik = sum_j dT_ij/drpy_k * w_j.
    const Scalar ct2 = ct * ct;
    Eigen::Matrix<Scalar, 3, 3> dT_dphi;
    dT_dphi << 0, cp * st / ct, -sp * st / ct,
               0, -sp, -cp,
               0, cp / ct, -sp / ct;
    Eigen::Matrix<Scalar, 3, 3> dT_dtheta;
    dT_dtheta << 0, sp / ct2, cp / ct2,
                 0, 0, 0,
                 0, sp * st / ct2, cp * st / ct2;
    Eigen::Matrix<Scalar, 3, 3> M;
    M.col(0) = dT_dphi * w;
    M.col(1) = dT_dtheta * w;
    M.col(2) = Eigen::Matrix<Scalar, 3, 1>::Zero();
    A->template block<3, 3>(kAtt, kAtt) = M;

    // rpy_dot wrt omega: T(rpy).
    const Scalar sphi = sp, cphi = cp;  // aliases for readability
    Eigen::Matrix<Scalar, 3, 3> T;
    T << Scalar(1), sphi * st / ct, cphi * st / ct,
         Scalar(0), cphi, -sphi,
         Scalar(0), sphi / ct, cphi / ct;
    A->template block<3, 3>(kAtt, kRate) = T;
  }

  // omega_dot wrt omega:  J^-1 ([J w]_x - [w]_x J).
  A->template block<3, 3>(kRate, kRate) =
    inertia_inv_ * (skewMatrix(J * w) - skewMatrix(w) * J);

  // --- df/du ----------------------------------------------------------------------------------
  // v_dot wrt u: (1/m) R * [0 0 0 0; 0 0 0 0; 1 1 1 1] = (1/m) R.col(2) * ones(1,4).
  B->template block<3, 4>(kVel, 0) =
    (Scalar(1) / m) * (R.col(2) * Eigen::Matrix<Scalar, 1, 4>::Ones());
  // omega_dot wrt u: J^-1 * allocation_torque_rows.
  B->template block<3, 4>(kRate, 0) = inertia_inv_ * allocation_.template block<3, 4>(1, 0);
}

template<typename Scalar, AttitudeRep Rep>
void QuadrotorDynamics<Scalar, Rep>::allocate(
  const InputVector & u, Scalar * collective_thrust,
  Eigen::Matrix<Scalar, 3, 1> * torque) const
{
  const Eigen::Matrix<Scalar, 4, 1> wrench = allocation_ * u;
  if (collective_thrust != nullptr) {*collective_thrust = wrench(0);}
  if (torque != nullptr) {*torque = wrench.template segment<3>(1);}
}

template<typename Scalar, AttitudeRep Rep>
bool QuadrotorDynamics<Scalar, Rep>::allocateInverse(
  Scalar collective_thrust, const Eigen::Matrix<Scalar, 3, 1> & torque, InputVector * u) const
{
  Eigen::Matrix<Scalar, 4, 1> wrench;
  wrench << collective_thrust, torque;
  Eigen::Matrix<Scalar, 4, 1> u_raw = allocation_inverse_ * wrench;
  bool clamped = false;
  for (int i = 0; i < 4; ++i) {
    const Scalar lo = params_.min_thrust_per_rotor;
    const Scalar hi = params_.max_thrust_per_rotor;
    if (u_raw(i) < lo) {u_raw(i) = lo; clamped = true;}
    if (u_raw(i) > hi) {u_raw(i) = hi; clamped = true;}
  }
  if (u != nullptr) {*u = u_raw;}
  return !clamped;
}

template<typename Scalar, AttitudeRep Rep>
Scalar QuadrotorDynamics<Scalar, Rep>::hoverThrustPerRotor() const
{
  return Scalar(params_.mass * params_.gravity / 4.0);
}

template<typename Scalar, AttitudeRep Rep>
Eigen::Matrix<Scalar, 3, 3>
QuadrotorDynamics<Scalar, Rep>::rotationMatrix(const StateVector & x) const
{
  if constexpr (Rep == AttitudeRep::Quaternion) {
    const Eigen::Matrix<Scalar, 4, 1> q = x.template segment<4>(StateLayout<Rep>::kAttIdx);
    const Scalar w = q(0), qx = q(1), qy = q(2), qz = q(3);
    const Scalar ww = w * w, xx = qx * qx, yy = qy * qy, zz = qz * qz;
    Eigen::Matrix<Scalar, 3, 3> R;
    R(0, 0) = ww + xx - yy - zz; R(0, 1) = Scalar(2) * (qx * qy - w * qz);
    R(0, 2) = Scalar(2) * (qx * qz + w * qy);
    R(1, 0) = Scalar(2) * (qx * qy + w * qz); R(1, 1) = ww - xx + yy - zz;
    R(1, 2) = Scalar(2) * (qy * qz - w * qx);
    R(2, 0) = Scalar(2) * (qx * qz - w * qy); R(2, 1) = Scalar(2) * (qy * qz + w * qx);
    R(2, 2) = ww - xx - yy + zz;
    return R;
  } else {
    const Eigen::Matrix<Scalar, 3, 1> rpy = x.template segment<3>(StateLayout<Rep>::kAttIdx);
    return eulerRotationZyx(rpy);
  }
}

// ================================================================================================
// Free helpers
// ================================================================================================

template<typename Scalar>
Eigen::Quaternion<Scalar> quatMultiply(
  const Eigen::Quaternion<Scalar> & a, const Eigen::Quaternion<Scalar> & b)
{
  // Hamilton product, spelled out in (w, x, y, z) to avoid Eigen's (x, y, z, w) coeffs layout.
  return Eigen::Quaternion<Scalar>(
    a.w() * b.w() - a.x() * b.x() - a.y() * b.y() - a.z() * b.z(),
    a.w() * b.x() + a.x() * b.w() + a.y() * b.z() - a.z() * b.y(),
    a.w() * b.y() - a.x() * b.z() + a.y() * b.w() + a.z() * b.x(),
    a.w() * b.z() + a.x() * b.y() - a.y() * b.x() + a.z() * b.w());
}

template<typename Scalar>
Eigen::Quaternion<Scalar> quatError(
  const Eigen::Quaternion<Scalar> & q, const Eigen::Quaternion<Scalar> & q_ref)
{
  // Shortest-arc error: q_err = q_ref^-1 (x) q. Negate so w >= 0 (avoids the unwinding
  // phenomenon — the cost must always drive along the shortest arc).
  Eigen::Quaternion<Scalar> err = quatMultiply(q_ref.conjugate(), q);
  if (err.w() < Scalar(0)) {
    err.coeffs() = -err.coeffs();
  }
  return err;
}

template<typename Scalar>
Eigen::Matrix<Scalar, 3, 1> quatToEulerZyx(const Eigen::Quaternion<Scalar> & q)
{
  // ZYX (yaw-pitch-roll) extraction, guarding the asin argument against |arg| > 1.
  const Scalar w = q.w(), x = q.x(), y = q.y(), z = q.z();
  const Scalar roll = std::atan2(
    Scalar(2) * (w * x + y * z), Scalar(1) - Scalar(2) * (x * x + y * y));
  const Scalar sin_pitch = std::clamp(Scalar(2) * (w * y - z * x), Scalar(-1), Scalar(1));
  const Scalar pitch = std::asin(sin_pitch);
  const Scalar yaw = std::atan2(
    Scalar(2) * (w * z + x * y), Scalar(1) - Scalar(2) * (y * y + z * z));
  return Eigen::Matrix<Scalar, 3, 1>(roll, pitch, yaw);
}

template<typename Scalar>
Eigen::Quaternion<Scalar> eulerZyxToQuat(const Eigen::Matrix<Scalar, 3, 1> & rpy)
{
  const Scalar cr = std::cos(rpy(0) / Scalar(2)), sr = std::sin(rpy(0) / Scalar(2));
  const Scalar cp = std::cos(rpy(1) / Scalar(2)), sp = std::sin(rpy(1) / Scalar(2));
  const Scalar cy = std::cos(rpy(2) / Scalar(2)), sy = std::sin(rpy(2) / Scalar(2));
  return Eigen::Quaternion<Scalar>(
    cr * cp * cy + sr * sp * sy,
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy);
}

Eigen::Vector3d enuToNed(const Eigen::Vector3d & v_enu)
{
  // ENU -> NED: (x, y, z) -> (y, x, -z). Involutive: NED -> ENU is the same map.
  return Eigen::Vector3d(v_enu.y(), v_enu.x(), -v_enu.z());
}

Eigen::Vector3d nedToEnu(const Eigen::Vector3d & v_ned)
{
  return enuToNed(v_ned);
}

Eigen::Quaterniond quatEnuFluToNedFrd(const Eigen::Quaterniond & q_enu_flu)
{
  // q_ned_frd = q_a (x) q_enu_flu (x) q_b, with the two fixed 180-degree rotations:
  //   q_a = 180 deg about ENU (1,1,0)/sqrt(2)   (world frame change)
  //   q_b = 180 deg about body x                (body frame change)
  // Both are self-inverse, so the same expression converts back.
  const Eigen::Quaterniond q_a(0.0, std::sqrt(2.0) / 2.0, std::sqrt(2.0) / 2.0, 0.0);
  const Eigen::Quaterniond q_b(0.0, 1.0, 0.0, 0.0);
  return quatMultiply(quatMultiply(q_a, q_enu_flu), q_b);
}

Eigen::Quaterniond quatNedFrdToEnuFlu(const Eigen::Quaterniond & q_ned_frd)
{
  return quatEnuFluToNedFrd(q_ned_frd);
}

// ================================================================================================
// Explicit instantiations
// ================================================================================================

template class QuadrotorDynamics<double, AttitudeRep::Quaternion>;
template class QuadrotorDynamics<double, AttitudeRep::Euler>;

template Eigen::Quaterniond quatMultiply<double>(
  const Eigen::Quaterniond &, const Eigen::Quaterniond &);
template Eigen::Quaterniond quatError<double>(
  const Eigen::Quaterniond &, const Eigen::Quaterniond &);
template Eigen::Vector3d quatToEulerZyx<double>(const Eigen::Quaterniond &);
template Eigen::Quaterniond eulerZyxToQuat<double>(const Eigen::Vector3d &);

}  // namespace uav_mpc
