// Copyright (c) 2026 Ali-Eimaan.
// SPDX-License-Identifier: BSD-3-Clause
//
// Declarations for the templated 12/13-state quadrotor rigid-body dynamics on SE(3).
// See .deepseek/04_DYNAMICS.md §4. The same equations are mirrored symbolically in
// codegen/quadrotor_model.py; the C++ version here is the *ground truth* used by the
// simulation-free unit tests and by the internal forward-integration used for one-step
// state prediction (latency compensation).

#ifndef UAV_MPC__QUADROTOR_DYNAMICS_HPP_
#define UAV_MPC__QUADROTOR_DYNAMICS_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>
#include <string>

namespace uav_mpc
{

/// Attitude parameterisation selected at compile time.
enum class AttitudeRep
{
  Euler,      ///< ZYX (yaw-pitch-roll) Tait-Bryan angles; nx = 12. Singular at pitch = +/- pi/2.
  Quaternion  ///< Hamilton (w, x, y, z), body-to-world; nx = 13. Norm enforced by projection.
};

/// Physical + aerodynamic parameters of one airframe. Populated from params/*_calibration.yaml.
struct QuadrotorParams
{
  double mass{0.0};                             ///< [kg] total take-off mass
  Eigen::Matrix3d inertia{Eigen::Matrix3d::Zero()};  ///< [kg m^2] body-frame inertia tensor
  double arm_length{0.0};                       ///< [m] motor axis to CoG, in-plane
  double thrust_coeff{0.0};                     ///< [N/(rad/s)^2] k_f, T_i = k_f * omega_i^2
  double torque_coeff{0.0};                     ///< [N m/(rad/s)^2] k_m, yaw drag torque
  double rotor_time_constant{0.0};              ///< [s] first-order motor lag (used for one-step lag comp)
  Eigen::Vector3d drag_coeff{Eigen::Vector3d::Zero()};  ///< [N s/m] linear body-frame rotor drag
  double gravity{9.80665};                      ///< [m/s^2]
  double max_thrust_per_rotor{0.0};             ///< [N] saturation, single rotor
  double min_thrust_per_rotor{0.0};             ///< [N] usually > 0 to keep rotors spinning
  std::string frame_name{};                     ///< e.g. "x500_v2", "crazyflie21"

  /// Load from a flat ROS parameter map or a YAML node. Throws std::runtime_error if a key
  /// is missing or a value is non-physical (mass <= 0, non-SPD inertia, ...).
  static QuadrotorParams fromYaml(const std::string & yaml_path);

  /// True iff every field is physically admissible. Cheap; called on every reconfigure.
  bool isValid(std::string * why = nullptr) const;
};

/// Compile-time state/input layout for a given attitude representation.
template<AttitudeRep Rep>
struct StateLayout;

template<>
struct StateLayout<AttitudeRep::Euler>
{
  static constexpr int kNx = 12;   ///< [p(3) v(3) rpy(3) omega(3)]
  static constexpr int kNu = 4;    ///< [T1 T2 T3 T4] individual rotor thrusts [N]
  static constexpr int kPosIdx = 0;
  static constexpr int kVelIdx = 3;
  static constexpr int kAttIdx = 6;
  static constexpr int kAttSize = 3;
  static constexpr int kRateIdx = 9;
};

template<>
struct StateLayout<AttitudeRep::Quaternion>
{
  static constexpr int kNx = 13;   ///< [p(3) v(3) q(4, wxyz) omega(3)]
  static constexpr int kNu = 4;
  static constexpr int kPosIdx = 0;
  static constexpr int kVelIdx = 3;
  static constexpr int kAttIdx = 6;
  static constexpr int kAttSize = 4;
  static constexpr int kRateIdx = 10;
};

/// Rigid-body quadrotor model. World frame is ENU (ROS convention); body frame is FLU.
/// PX4 talks NED/FRD — all conversion happens in nmpc_node, never here.
///
/// Continuous-time dynamics (see docs/derivations/quadrotor_se3_dynamics.tex):
///   p_dot     = v
///   v_dot     = (1/m) * R(q) * (F_b - D * R(q)^T * v) + g_w
///   q_dot     = 0.5 * q (x) [0, omega]          (or Euler-rate kinematics for Rep = Euler)
///   omega_dot = J^-1 * (tau_b - omega x (J * omega))
/// with F_b = [0, 0, sum(T_i)]^T and tau_b from the fixed allocation matrix.
template<typename Scalar, AttitudeRep Rep = AttitudeRep::Quaternion>
class QuadrotorDynamics
{
public:
  static constexpr int kNx = StateLayout<Rep>::kNx;
  static constexpr int kNu = StateLayout<Rep>::kNu;

  using StateVector = Eigen::Matrix<Scalar, kNx, 1>;
  using InputVector = Eigen::Matrix<Scalar, kNu, 1>;
  using StateJacobian = Eigen::Matrix<Scalar, kNx, kNx>;
  using InputJacobian = Eigen::Matrix<Scalar, kNx, kNu>;

  explicit QuadrotorDynamics(const QuadrotorParams & params);

  /// Continuous-time state derivative x_dot = f(x, u). No allocation, no exceptions.
  StateVector f(const StateVector & x, const InputVector & u) const;

  /// Explicit RK4 discretisation over dt. Re-normalises the quaternion on exit when
  /// Rep == Quaternion. Used for latency compensation and for the SIL "truth" integrator.
  StateVector step(const StateVector & x, const InputVector & u, Scalar dt) const;

  /// Analytic Jacobians of f. Only needed for the C++-side EKF-free sanity tests; acados gets
  /// its own derivatives from CasADi AD. Implement by hand and unit-test against finite
  /// differences (tolerance 1e-6 relative).
  void jacobians(
    const StateVector & x, const InputVector & u, StateJacobian * A, InputJacobian * B) const;

  /// Control allocation: individual rotor thrusts -> (collective thrust [N], body torque [N m]).
  /// Layout is the PX4 "quad X" convention; see the mixer table in the guide (§4.3).
  void allocate(const InputVector & u, Scalar * collective_thrust, Eigen::Matrix<Scalar, 3, 1> * torque)
  const;

  /// Inverse allocation, clamped to [min_thrust_per_rotor, max_thrust_per_rotor].
  /// Returns false if the request was infeasible and had to be clamped.
  bool allocateInverse(
    Scalar collective_thrust, const Eigen::Matrix<Scalar, 3, 1> & torque, InputVector * u) const;

  /// Hover thrust per rotor, m*g/4. Used to seed the solver and to normalise PX4 thrust.
  Scalar hoverThrustPerRotor() const;

  /// Rotation body->world for the attitude slice of x.
  Eigen::Matrix<Scalar, 3, 3> rotationMatrix(const StateVector & x) const;

  const QuadrotorParams & params() const {return params_;}

private:
  QuadrotorParams params_;
  Eigen::Matrix<Scalar, 4, 4> allocation_;      ///< [T; tau_x; tau_y; tau_z] = allocation_ * u
  Eigen::Matrix<Scalar, 4, 4> allocation_inverse_;  ///< cached inverse of allocation_ (see §4.3)
  Eigen::Matrix<Scalar, 3, 3> inertia_inv_;

  /// Builds allocation_ (PX4 quad-X, §4.3) and inertia_inv_ from params_.
  void buildAllocationMatrix();
};

// ---------------------------------------------------------------------------------------------
// Free helpers shared with trajectory_generator (differential flatness map).
// ---------------------------------------------------------------------------------------------

/// Hamilton quaternion product, (w, x, y, z) ordering.
template<typename Scalar>
Eigen::Quaternion<Scalar> quatMultiply(
  const Eigen::Quaternion<Scalar> & a, const Eigen::Quaternion<Scalar> & b);

/// Shortest-arc error quaternion q_err = q_ref^-1 (x) q, with the sign convention that the
/// scalar part is non-negative (avoids the unwinding phenomenon).
template<typename Scalar>
Eigen::Quaternion<Scalar> quatError(
  const Eigen::Quaternion<Scalar> & q, const Eigen::Quaternion<Scalar> & q_ref);

/// ZYX Euler (roll, pitch, yaw) <-> quaternion, matching the StateLayout ordering.
template<typename Scalar>
Eigen::Matrix<Scalar, 3, 1> quatToEulerZyx(const Eigen::Quaternion<Scalar> & q);

template<typename Scalar>
Eigen::Quaternion<Scalar> eulerZyxToQuat(const Eigen::Matrix<Scalar, 3, 1> & rpy);

/// ENU(world)/FLU(body) <-> NED(world)/FRD(body) frame conversions for the PX4 boundary.
Eigen::Vector3d enuToNed(const Eigen::Vector3d & v_enu);
Eigen::Vector3d nedToEnu(const Eigen::Vector3d & v_ned);
Eigen::Quaterniond quatEnuFluToNedFrd(const Eigen::Quaterniond & q_enu_flu);
Eigen::Quaterniond quatNedFrdToEnuFlu(const Eigen::Quaterniond & q_ned_frd);

// Explicit instantiations provided in quadrotor_dynamics.cpp:
extern template class QuadrotorDynamics<double, AttitudeRep::Quaternion>;
extern template class QuadrotorDynamics<double, AttitudeRep::Euler>;

// The free helper templates are also explicitly instantiated for double in
// quadrotor_dynamics.cpp so other translation units (e.g. trajectory_generator.cpp) can use
// them without dragging the template definitions into every TU.
extern template Eigen::Quaterniond quatMultiply<double>(
  const Eigen::Quaterniond &, const Eigen::Quaterniond &);
extern template Eigen::Quaterniond quatError<double>(
  const Eigen::Quaterniond &, const Eigen::Quaterniond &);
extern template Eigen::Vector3d quatToEulerZyx<double>(const Eigen::Quaterniond &);
extern template Eigen::Quaterniond eulerZyxToQuat<double>(const Eigen::Vector3d &);

}  // namespace uav_mpc

#endif  // UAV_MPC__QUADROTOR_DYNAMICS_HPP_
