// Copyright (c) 2026 Ali-Eimaan.
// SPDX-License-Identifier: BSD-3-Clause
//
// See .deepseek/05_TRAJECTORY.md §5.
//
// Minimum-snap polynomial trajectory generation (Mellinger & Kumar, ICRA 2011) plus closed-form
// analytic primitives (figure-8 / lemniscate / circle). All derivatives up to snap are analytic
// — never finite-differenced inside the generator, because the flatness map differentiates
// again and numerical noise would become commanded body-rate noise.

#include "uav_mpc/trajectory_generator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace uav_mpc
{

namespace
{

/// pi and 2*pi; M_PI is not guaranteed with -std=c++17 -Wpedantic.
constexpr double kPi = 3.141592653589793238462643383279502884;  // std::numbers::pi (C++20) unavailable
constexpr double kTwoPi = 2.0 * kPi;

/// Falling factorial k!/(k-d)! = k*(k-1)*...*(k-d+1); 1 for d == 0.
double fallingFactorial(int k, int d)
{
  double v = 1.0;
  for (int i = 0; i < d; ++i) {
    v *= static_cast<double>(k - i);
  }
  return v;
}

/// Binomial coefficient table for n, k <= 4.
double binom(int n, int k)
{
  static constexpr double C[5][5] = {
    {1, 0, 0, 0, 0},
    {1, 1, 0, 0, 0},
    {1, 2, 1, 0, 0},
    {1, 3, 3, 1, 0},
    {1, 4, 6, 4, 1},
  };
  if (n < 0 || n > 4 || k < 0 || k > n) {
    return 0.0;
  }
  return C[n][k];
}

/// Value and first four derivatives of one scalar function of time.
struct AxisDerivs
{
  double f0{0.0};
  double f1{0.0};
  double f2{0.0};
  double f3{0.0};
  double f4{0.0};
};

/// Exact (not finite-difference) quotient rule applied recursively: f = n/d, and
/// f^(k) = (n^(k) - sum_{i=1..k} C(k,i) d^(i) f^(k-i)) / d. Used for the Bernoulli lemniscate
/// rational parametrisation.
AxisDerivs quotientRule(const AxisDerivs & n, const AxisDerivs & d)
{
  AxisDerivs f;
  f.f0 = n.f0 / d.f0;
  f.f1 = (n.f1 - binom(1, 1) * d.f1 * f.f0) / d.f0;
  f.f2 = (n.f2 - (binom(2, 1) * d.f1 * f.f1 + binom(2, 2) * d.f2 * f.f0)) / d.f0;
  f.f3 = (n.f3 -
    (binom(3, 1) * d.f1 * f.f2 + binom(3, 2) * d.f2 * f.f1 +
      binom(3, 3) * d.f3 * f.f0)) /
    d.f0;
  f.f4 = (n.f4 -
    (binom(4, 1) * d.f1 * f.f3 + binom(4, 2) * d.f2 * f.f2 +
      binom(4, 3) * d.f3 * f.f1 + binom(4, 4) * d.f4 * f.f0)) /
    d.f0;
  return f;
}

/// Blends `hover` into `traj` with the C^4 ramp: b = hover + S(u)*(traj - hover).
/// s_k are the ramp k-th time derivatives (rampScale at the given t). Product rule:
///   b^(k) = sum_{i=0..k-1} C(k,i) s_i * traj^(k-i) + s_k * (traj^0 - hover^0)
/// since hover has zero derivatives of order >= 1.
FlatState blendHoverToTraj(
  const FlatState & hover, const FlatState & traj,
  double s0, double s1, double s2, double s3, double s4)
{
  const Eigen::Vector3d dpos = traj.position - hover.position;
  FlatState b;
  b.position = hover.position + s0 * dpos;
  b.velocity = s0 * traj.velocity + s1 * dpos;
  b.acceleration = s0 * traj.acceleration + 2.0 * s1 * traj.velocity + s2 * dpos;
  b.jerk = s0 * traj.jerk + 3.0 * s1 * traj.acceleration + 3.0 * s2 * traj.velocity + s3 * dpos;
  b.snap = s0 * traj.snap + 4.0 * s1 * traj.jerk + 6.0 * s2 * traj.acceleration +
    4.0 * s3 * traj.velocity + s4 * dpos;
  b.yaw = hover.yaw + s0 * (traj.yaw - hover.yaw);
  b.yaw_rate = s0 * traj.yaw_rate + s1 * (traj.yaw - hover.yaw);
  b.yaw_accel = s0 * traj.yaw_accel + 2.0 * s1 * traj.yaw_rate + s2 * (traj.yaw - hover.yaw);
  b.t = traj.t;
  return b;
}

/// Yaw and its derivatives from the velocity direction (§5.1). Returns false and sets
/// yaw = fixed_yaw (all derivatives zero) when the horizontal velocity is degenerate —
/// stateless fallback: the primitives never reach ||v_xy|| = 0 except at isolated instants.
bool yawFromVelocity(
  double x_dot, double y_dot, double x_ddot, double y_ddot,
  double x_dddot, double y_dddot, double fixed_yaw,
  double * yaw, double * yaw_rate, double * yaw_accel)
{
  const double r2 = x_dot * x_dot + y_dot * y_dot;
  if (r2 < 1e-6) {
    *yaw = fixed_yaw;
    *yaw_rate = 0.0;
    *yaw_accel = 0.0;
    return false;
  }
  const double num1 = x_dot * y_ddot - y_dot * x_ddot;
  *yaw = std::atan2(y_dot, x_dot);
  *yaw_rate = num1 / r2;
  const double num2 = x_dot * y_dddot - y_dot * x_dddot;
  *yaw_accel = num2 / r2 - 2.0 * num1 * (x_dot * x_ddot + y_dot * y_ddot) / (r2 * r2);
  return true;
}

}  // namespace

TrajectoryGenerator::TrajectoryGenerator(const TrajectoryParams & params)
: params_(params)
{
  generate();
}

void TrajectoryGenerator::generate()
{
  if (params_.altitude <= 0.0) {
    throw std::invalid_argument("trajectory: altitude must be > 0");
  }
  if (params_.max_velocity <= 0.0) {
    throw std::invalid_argument("trajectory: max_velocity must be > 0");
  }
  if (params_.max_acceleration <= 0.0) {
    throw std::invalid_argument("trajectory: max_acceleration must be > 0");
  }

  switch (params_.type) {
    case TrajectoryType::Hover:
    case TrajectoryType::Step:
      // No segments needed; sample() is fully analytic.
      segments_.clear();
      break;
    case TrajectoryType::Figure8:
    case TrajectoryType::Lemniscate:
    case TrajectoryType::Circle:
      if (params_.period <= 0.0) {
        throw std::invalid_argument("trajectory: period must be > 0 for periodic types");
      }
      segments_.clear();
      break;
    case TrajectoryType::Waypoints:
      if (params_.waypoints.size() < 2) {
        throw std::invalid_argument("trajectory: waypoints requires at least 2 points");
      }
      if (!params_.segment_times.empty() &&
        params_.segment_times.size() != params_.waypoints.size() - 1)
      {
        throw std::invalid_argument(
          "trajectory: segment_times size must equal waypoints size - 1");
      }
      buildMinimumSnap();
      break;
  }
}

void TrajectoryGenerator::setParams(const TrajectoryParams & params)
{
  params_ = params;
  generate();
}

FlatState TrajectoryGenerator::sample(double t) const
{
  // Per-type primitive evaluation; periodic types wrap internally, so this is
  // the absolute-time value used for the ramp as well.
  FlatState traj;
  switch (params_.type) {
    case TrajectoryType::Hover:
      return sampleHover(t);
    case TrajectoryType::Figure8:
      traj = sampleFigure8(t);
      break;
    case TrajectoryType::Lemniscate:
      traj = sampleLemniscate(t);
      break;
    case TrajectoryType::Circle:
      traj = sampleCircle(t);
      break;
    case TrajectoryType::Step:
      traj = sampleStep(t);
      break;
    case TrajectoryType::Waypoints: {
      double total = duration();
      double tc = std::min(std::max(t, 0.0), total);
      double acc = 0.0;
      for (std::size_t i = 0; i < segments_.size(); ++i) {
        const double seg_end = acc + segments_[i].duration;
        if (tc <= seg_end || i + 1 == segments_.size()) {
          const double tau = (tc - acc) / segments_[i].duration;
          FlatState s = evaluateSegment(i, tau);
          s.t = t;
          return s;
        }
        acc = seg_end;
      }
      // unreachable: at least one segment exists for Waypoints
      return FlatState{};
    }
  }

  // C^4 ramp-in from hover: only the orbit types blend, so a step-jump at t = 0 does not
  // reach the flatness map. Waypoints carry their own boundary conditions and are exempt.
  if (params_.ramp_in_time > 0.0) {
    const double s0 = rampScale(t, params_.ramp_in_time, 0);
    const double s1 = rampScale(t, params_.ramp_in_time, 1);
    const double s2 = rampScale(t, params_.ramp_in_time, 2);
    const double s3 = rampScale(t, params_.ramp_in_time, 3);
    const double s4 = rampScale(t, params_.ramp_in_time, 4);
    return blendHoverToTraj(sampleHover(t), traj, s0, s1, s2, s3, s4);
  }
  return traj;
}

std::vector<FlatState> TrajectoryGenerator::sampleHorizon(
  double t0, double dt, int n_steps) const
{
  std::vector<FlatState> out;
  out.reserve(static_cast<std::size_t>(n_steps) + 1);
  for (int i = 0; i <= n_steps; ++i) {
    out.push_back(sample(t0 + static_cast<double>(i) * dt));
  }
  return out;
}

StateInputReference TrajectoryGenerator::flatToStateInput(
  const FlatState & flat, const QuadrotorParams & airframe, AttitudeRep rep)
{
  using Vec3 = Eigen::Vector3d;

  const double g = airframe.gravity;
  const double m = airframe.mass;

  StateInputReference ref;
  ref.state.resize(rep == AttitudeRep::Quaternion ? 13 : 12);

  const Vec3 t_vec = flat.acceleration + Vec3(0.0, 0.0, g);
  const double norm_t = t_vec.norm();

  if (norm_t < 1e-3) {
    // Free-fall / zero-acceleration: the attitude is undefined. Hold the level attitude at the
    // commanded yaw, zero the rates and set T = 0 (§5.5). Logged once.
    static std::once_flag flag;
    std::call_once(flag, []() {
      std::fprintf(
        stderr,
        "[trajectory_generator] WARN: flatness map degenerate (||a + g e_z|| < 1e-3); "
        "holding level attitude, T = 0\n");
    });
    const double psi = flat.yaw;
    const Vec3 x_b(std::cos(psi), std::sin(psi), 0.0);
    const Vec3 y_b(-std::sin(psi), std::cos(psi), 0.0);
    const Vec3 z_b(0.0, 0.0, 1.0);
    Eigen::Matrix3d R;
    R.col(0) = x_b;
    R.col(1) = y_b;
    R.col(2) = z_b;
    const Vec3 omega = Vec3::Zero();
    if (rep == AttitudeRep::Quaternion) {
      const Eigen::Quaterniond q(R);
      ref.state.segment<3>(0) = flat.position;
      ref.state.segment<3>(3) = flat.velocity;
      ref.state[6] = q.w();
      ref.state[7] = q.x();
      ref.state[8] = q.y();
      ref.state[9] = q.z();
      ref.state.segment<3>(10) = omega;
    } else {
      const Eigen::Quaterniond q(R);
      const Eigen::Vector3d rpy = quatToEulerZyx(q);
      ref.state.segment<3>(0) = flat.position;
      ref.state.segment<3>(3) = flat.velocity;
      ref.state.segment<3>(6) = rpy;
      ref.state.segment<3>(9) = omega;
    }
    QuadrotorDynamics<double, AttitudeRep::Quaternion> dyn(airframe);
    dyn.allocateInverse(0.0, Vec3::Zero(), &ref.input);  // clamps to [min, max]
    return ref;
  }

  // --- body axes (§5.5) -------------------------------------------------------------------
  const Vec3 z_b = t_vec / norm_t;
  const double T = m * norm_t;

  double psi = flat.yaw;
  Vec3 x_c(std::cos(psi), std::sin(psi), 0.0);
  Vec3 y_c(-std::sin(psi), std::cos(psi), 0.0);

  Vec3 n = z_b.cross(x_c);
  if (n.norm() < 1e-6) {
    // Yaw axis aligned with the body z-axis: perturb psi and recompute (§5.5).
    psi += 1e-4;
    x_c = Vec3(std::cos(psi), std::sin(psi), 0.0);
    y_c = Vec3(-std::sin(psi), std::cos(psi), 0.0);
    n = z_b.cross(x_c);
  }
  const Vec3 y_b = n.normalized();
  const Vec3 x_b = y_b.cross(z_b);

  Eigen::Matrix3d R;
  R.col(0) = x_b;
  R.col(1) = y_b;
  R.col(2) = z_b;

  // --- body rates from the jerk projection (§5.5) -----------------------------------------
  const double T_dot = m * z_b.dot(flat.jerk);
  const Vec3 h_w = (m / T) * (flat.jerk - (z_b.dot(flat.jerk)) * z_b);
  const double w_x = -h_w.dot(y_b);
  const double w_y = h_w.dot(x_b);
  const double denom = y_c.cross(z_b).norm();
  const double w_z = (flat.yaw_rate * (x_c.dot(x_b)) + w_y * (y_c.dot(z_b))) /
    std::max(denom, 1e-9);
  const Vec3 omega(w_x, w_y, w_z);

  // --- angular acceleration from the snap projection ---------------------------------------
  // z_b' = h_w (world frame); everything else follows by the product rule. [omega_dot]x =
  // R'^T R' + R^T R''  (derivative of [omega]x = R^T R').
  const Vec3 z_b_prime = h_w;
  const Vec3 h_w_prime = (-m * T_dot / (T * T)) * (flat.jerk - (z_b.dot(flat.jerk)) * z_b) +
    (m / T) * (flat.snap - ((h_w.dot(flat.jerk) + z_b.dot(flat.snap)) * z_b +
    (z_b.dot(flat.jerk)) * h_w));
  const Vec3 z_b_pp = h_w_prime;

  // x_c' = psi_dot * y_c,  x_c'' = psi_ddot * y_c - psi_dot^2 * x_c (and y_c analogues),
  // inlined below to keep the cross products explicit.
  const Vec3 n_prime = z_b_prime.cross(x_c) + flat.yaw_rate * (z_b.cross(y_c));
  const Vec3 n_pp = z_b_pp.cross(x_c) + 2.0 * flat.yaw_rate * (z_b_prime.cross(y_c)) +
    flat.yaw_accel * (z_b.cross(y_c)) - flat.yaw_rate * flat.yaw_rate * (z_b.cross(x_c));
  const double n_norm = n.norm();
  const double n_norm_prime = n.dot(n_prime) / n_norm;
  const double n_norm_pp = (n_prime.dot(n_prime) + n.dot(n_pp) - n_norm_prime * n_norm_prime) /
    n_norm;
  const Vec3 y_b_prime = (n_prime - n_norm_prime * y_b) / n_norm;
  const Vec3 y_b_pp = (n_pp - n_norm_pp * y_b - n_norm_prime * y_b_prime) / n_norm;

  const Vec3 x_b_prime = y_b_prime.cross(z_b) + y_b.cross(z_b_prime);
  const Vec3 x_b_pp = y_b_pp.cross(z_b) + 2.0 * (y_b_prime.cross(z_b_prime)) + y_b.cross(z_b_pp);

  Eigen::Matrix3d R_prime;
  R_prime.col(0) = x_b_prime;
  R_prime.col(1) = y_b_prime;
  R_prime.col(2) = z_b_prime;
  Eigen::Matrix3d R_pp;
  R_pp.col(0) = x_b_pp;
  R_pp.col(1) = y_b_pp;
  R_pp.col(2) = z_b_pp;

  const Eigen::Matrix3d omega_dot_hat = R_prime.transpose() * R_prime + R.transpose() * R_pp;
  const Vec3 omega_dot(omega_dot_hat(2, 1), omega_dot_hat(0, 2), omega_dot_hat(1, 0));

  const Vec3 tau = airframe.inertia * omega_dot + omega.cross(airframe.inertia * omega);

  // --- state + input reference --------------------------------------------------------------
  if (rep == AttitudeRep::Quaternion) {
    const Eigen::Quaterniond q(R);
    ref.state.segment<3>(0) = flat.position;
    ref.state.segment<3>(3) = flat.velocity;
    ref.state[6] = q.w();
    ref.state[7] = q.x();
    ref.state[8] = q.y();
    ref.state[9] = q.z();
    ref.state.segment<3>(10) = omega;
  } else {
    const Eigen::Quaterniond q(R);
    const Eigen::Vector3d rpy = quatToEulerZyx(q);
    ref.state.segment<3>(0) = flat.position;
    ref.state.segment<3>(3) = flat.velocity;
    ref.state.segment<3>(6) = rpy;
    ref.state.segment<3>(9) = omega;
  }

  QuadrotorDynamics<double, AttitudeRep::Quaternion> dyn(airframe);
  dyn.allocateInverse(T, tau, &ref.input);  // clamped to [min, max]
  return ref;
}

std::vector<StateInputReference> TrajectoryGenerator::referenceHorizon(
  double t0, double dt, int n_steps, const QuadrotorParams & airframe, AttitudeRep rep) const
{
  std::vector<StateInputReference> out;
  out.reserve(static_cast<std::size_t>(n_steps) + 1);
  const std::vector<FlatState> flats = sampleHorizon(t0, dt, n_steps);
  for (const FlatState & f : flats) {
    out.push_back(flatToStateInput(f, airframe, rep));
  }
  return out;
}

double TrajectoryGenerator::duration() const
{
  if (params_.type == TrajectoryType::Waypoints) {
    double total = 0.0;
    for (const PolynomialSegment & s : segments_) {
      total += s.duration;
    }
    return total;
  }
  return std::numeric_limits<double>::infinity();
}

bool TrajectoryGenerator::isDynamicallyFeasible(
  const QuadrotorParams & airframe, std::string * report) const
{
  constexpr double kSampleHz = 200.0;
  constexpr double kTol = 1e-3;

  double span = duration();
  if (std::isinf(span)) {
    // Periodic types: sweep one full lap plus the ramp-in so the blend is covered too.
    span = (params_.period > 0.0) ? params_.period + params_.ramp_in_time : 1.0;
  }

  double max_speed = 0.0;
  double max_accel = 0.0;
  double max_thrust_per_rotor = 0.0;
  const double min_thrust = airframe.min_thrust_per_rotor;
  const double max_thrust = airframe.max_thrust_per_rotor;

  const int n = static_cast<int>(std::ceil(span * kSampleHz));
  for (int i = 0; i <= n; ++i) {
    const double t = (static_cast<double>(i) / static_cast<double>(n)) * span;
    const FlatState s = sample(t);
    const double speed = s.velocity.norm();
    const double accel = s.acceleration.norm();
    max_speed = std::max(max_speed, speed);
    max_accel = std::max(max_accel, accel);
    // Required collective thrust, per rotor (ignoring the torque trim).
    const double per_rotor = airframe.mass * (s.acceleration + Eigen::Vector3d(0.0, 0.0, airframe.gravity)).norm() / 4.0;
    max_thrust_per_rotor = std::max(max_thrust_per_rotor, per_rotor);
  }

  const bool speed_ok = max_speed <= params_.max_velocity * (1.0 + kTol);
  const bool accel_ok = max_accel <= params_.max_acceleration * (1.0 + kTol);
  const bool thrust_ok = max_thrust_per_rotor >= min_thrust * (1.0 - kTol) &&
    max_thrust_per_rotor <= max_thrust * (1.0 + kTol);

  if (report != nullptr) {
    std::ostringstream os;
    os << "max speed " << max_speed << " m/s (limit " << params_.max_velocity << ")"
       << (speed_ok ? " OK" : " VIOLATION") << "; "
       << "max accel " << max_accel << " m/s^2 (limit " << params_.max_acceleration << ")"
       << (accel_ok ? " OK" : " VIOLATION") << "; "
       << "max per-rotor thrust " << max_thrust_per_rotor << " N ([" << min_thrust << ", "
       << max_thrust << "])" << (thrust_ok ? " OK" : " VIOLATION");
    *report = os.str();
  }
  return speed_ok && accel_ok && thrust_ok;
}

double TrajectoryGenerator::maxDerivativeJump(int derivative_order) const
{
  if (segments_.size() < 2 || derivative_order < 0 || derivative_order > 4) {
    return 0.0;
  }
  double worst = 0.0;
  for (std::size_t i = 0; i + 1 < segments_.size(); ++i) {
    const FlatState l = evaluateSegment(i, 1.0);
    const FlatState r = evaluateSegment(i + 1, 0.0);
    double d = 0.0;
    switch (derivative_order) {
      case 0:
        d = (l.position - r.position).norm();
        d = std::max(d, std::abs(l.yaw - r.yaw));
        break;
      case 1:
        d = (l.velocity - r.velocity).norm();
        d = std::max(d, std::abs(l.yaw_rate - r.yaw_rate));
        break;
      case 2:
        d = (l.acceleration - r.acceleration).norm();
        d = std::max(d, std::abs(l.yaw_accel - r.yaw_accel));
        break;
      case 3:
        d = (l.jerk - r.jerk).norm();
        break;
      case 4:
        d = (l.snap - r.snap).norm();
        break;
    }
    worst = std::max(worst, d);
  }
  return worst;
}

// ------------------------------------------------------------------------------------------------
// Analytic primitives
// ------------------------------------------------------------------------------------------------

FlatState TrajectoryGenerator::sampleFigure8(double t) const
{
  const double period = params_.period;
  double tt = std::fmod(t, period);
  if (tt < 0.0) {
    tt += period;
  }

  const double w = kTwoPi / period;
  const double wt = w * tt;
  const double sin_wt = std::sin(wt);
  const double cos_wt = std::cos(wt);
  const double sin_2wt = std::sin(2.0 * wt);
  const double cos_2wt = std::cos(2.0 * wt);

  // x = Ax*sin(wt)
  const double x = params_.amplitude_x * sin_wt;
  const double x_d = params_.amplitude_x * w * cos_wt;
  const double x_dd = -params_.amplitude_x * w * w * sin_wt;
  const double x_ddd = -params_.amplitude_x * w * w * w * cos_wt;
  const double x_dddd = params_.amplitude_x * w * w * w * w * sin_wt;
  // y = (Ay/2)*sin(2wt)
  const double y = 0.5 * params_.amplitude_y * sin_2wt;
  const double y_d = params_.amplitude_y * w * cos_2wt;
  const double y_dd = -2.0 * params_.amplitude_y * w * w * sin_2wt;
  const double y_ddd = -4.0 * params_.amplitude_y * w * w * w * cos_2wt;
  const double y_dddd = 8.0 * params_.amplitude_y * w * w * w * w * sin_2wt;
  // z = altitude + Az*sin(wt)
  const double z = params_.altitude + params_.amplitude_z * sin_wt;
  const double z_d = params_.amplitude_z * w * cos_wt;
  const double z_dd = -params_.amplitude_z * w * w * sin_wt;
  const double z_ddd = -params_.amplitude_z * w * w * w * cos_wt;
  const double z_dddd = params_.amplitude_z * w * w * w * w * sin_wt;

  FlatState s;
  s.position = params_.center + Eigen::Vector3d(x, y, z);
  s.velocity = Eigen::Vector3d(x_d, y_d, z_d);
  s.acceleration = Eigen::Vector3d(x_dd, y_dd, z_dd);
  s.jerk = Eigen::Vector3d(x_ddd, y_ddd, z_ddd);
  s.snap = Eigen::Vector3d(x_dddd, y_dddd, z_dddd);
  s.t = t;

  if (params_.yaw_follows_velocity) {
    yawFromVelocity(x_d, y_d, x_dd, y_dd, x_ddd, y_ddd, params_.fixed_yaw,
      &s.yaw, &s.yaw_rate, &s.yaw_accel);
  } else {
    s.yaw = params_.fixed_yaw;
  }
  return s;
}

FlatState TrajectoryGenerator::sampleLemniscate(double t) const
{
  const double period = params_.period;
  double tt = std::fmod(t, period);
  if (tt < 0.0) {
    tt += period;
  }
  const double w = kTwoPi / period;
  const double wt = w * tt;
  const double sin_wt = std::sin(wt);
  const double cos_wt = std::cos(wt);
  const double sin_2wt = std::sin(2.0 * wt);
  const double cos_2wt = std::cos(2.0 * wt);

  // Bernoulli lemniscate, rational parametrisation x = Ax*cos/(1+sin^2), y = Ay*sin*cos/(1+sin^2).
  // Derivatives are exact (recursive quotient rule), never finite-differenced.
  const double ax = params_.amplitude_x;
  const double ay = params_.amplitude_y;
  AxisDerivs nx{ax * cos_wt, -ax * sin_wt, -ax * cos_wt, ax * sin_wt, ax * cos_wt};
  AxisDerivs ny{ay * sin_wt * cos_wt, ay * cos_2wt, -2.0 * ay * sin_2wt, -4.0 * ay * cos_2wt,
    8.0 * ay * sin_2wt};
  AxisDerivs d{1.0 + sin_wt * sin_wt, sin_2wt, 2.0 * cos_2wt, -4.0 * sin_2wt, -8.0 * cos_2wt};
  const AxisDerivs fx = quotientRule(nx, d);
  const AxisDerivs fy = quotientRule(ny, d);

  const double z = params_.altitude + params_.amplitude_z * sin_wt;
  const double z_d = params_.amplitude_z * w * cos_wt;
  const double z_dd = -params_.amplitude_z * w * w * sin_wt;
  const double z_ddd = -params_.amplitude_z * w * w * w * cos_wt;
  const double z_dddd = params_.amplitude_z * w * w * w * w * sin_wt;

  FlatState s;
  s.position = params_.center + Eigen::Vector3d(fx.f0, fy.f0, z);
  s.velocity = Eigen::Vector3d(fx.f1, fy.f1, z_d);
  s.acceleration = Eigen::Vector3d(fx.f2, fy.f2, z_dd);
  s.jerk = Eigen::Vector3d(fx.f3, fy.f3, z_ddd);
  s.snap = Eigen::Vector3d(fx.f4, fy.f4, z_dddd);
  s.t = t;

  if (params_.yaw_follows_velocity) {
    yawFromVelocity(fx.f1, fy.f1, fx.f2, fy.f2, fx.f3, fy.f3, params_.fixed_yaw,
      &s.yaw, &s.yaw_rate, &s.yaw_accel);
  } else {
    s.yaw = params_.fixed_yaw;
  }
  return s;
}

FlatState TrajectoryGenerator::sampleCircle(double t) const
{
  const double period = params_.period;
  double tt = std::fmod(t, period);
  if (tt < 0.0) {
    tt += period;
  }
  const double w = kTwoPi / period;
  const double wt = w * tt;
  const double sin_wt = std::sin(wt);
  const double cos_wt = std::cos(wt);

  // x = cx + Ax*cos(wt), y = cy + Ay*sin(wt)
  const double x = params_.amplitude_x * cos_wt;
  const double x_d = -params_.amplitude_x * w * sin_wt;
  const double x_dd = -params_.amplitude_x * w * w * cos_wt;
  const double x_ddd = params_.amplitude_x * w * w * w * sin_wt;
  const double x_dddd = params_.amplitude_x * w * w * w * w * cos_wt;
  const double y = params_.amplitude_y * sin_wt;
  const double y_d = params_.amplitude_y * w * cos_wt;
  const double y_dd = -params_.amplitude_y * w * w * sin_wt;
  const double y_ddd = -params_.amplitude_y * w * w * w * cos_wt;
  const double y_dddd = params_.amplitude_y * w * w * w * w * sin_wt;

  FlatState s;
  s.position = params_.center + Eigen::Vector3d(x, y, params_.altitude);
  s.velocity = Eigen::Vector3d(x_d, y_d, 0.0);
  s.acceleration = Eigen::Vector3d(x_dd, y_dd, 0.0);
  s.jerk = Eigen::Vector3d(x_ddd, y_ddd, 0.0);
  s.snap = Eigen::Vector3d(x_dddd, y_dddd, 0.0);
  s.t = t;

  if (params_.yaw_follows_velocity) {
    yawFromVelocity(x_d, y_d, x_dd, y_dd, x_ddd, y_ddd, params_.fixed_yaw,
      &s.yaw, &s.yaw_rate, &s.yaw_accel);
  } else {
    s.yaw = params_.fixed_yaw;
  }
  return s;
}

FlatState TrajectoryGenerator::sampleHover(double t) const
{
  FlatState s;
  s.position = params_.center + Eigen::Vector3d(0.0, 0.0, params_.altitude);
  s.yaw = params_.fixed_yaw;
  s.t = t;
  return s;
}

FlatState TrajectoryGenerator::sampleStep(double t) const
{
  // The step target; the C^4 smoothing happens in sample() via the ramp blend.
  FlatState s;
  s.position = params_.center +
    Eigen::Vector3d(params_.amplitude_x, params_.amplitude_y, params_.altitude + params_.amplitude_z);
  s.yaw = params_.fixed_yaw;
  s.t = t;
  return s;
}

// ------------------------------------------------------------------------------------------------
// Minimum snap
// ------------------------------------------------------------------------------------------------

void TrajectoryGenerator::buildMinimumSnap()
{
  // Per axis (x, y, z, yaw), per segment: a 7th-order polynomial on the normalised domain
  // tau in [0, 1] minimising int_0^1 (d^4 p/dtau^4)^2 dtau. The waypoint counts here are
  // small (< 30), so a dense KKT solve is a deliberate choice — no banded structure is
  // exploited (§5.3).
  constexpr int kOrder = PolynomialSegment::kOrder;
  const int M = static_cast<int>(params_.waypoints.size()) - 1;
  const int n_coeff = (kOrder + 1) * M;  // 8 coefficients per segment

  std::vector<double> times = params_.segment_times;
  if (times.empty()) {
    times = allocateSegmentTimes();
  }
  if (times.size() != static_cast<std::size_t>(M)) {
    throw std::invalid_argument(
      "trajectory: segment_times size must equal waypoints size - 1");
  }

  // Per-segment Hessian: Q_ij = ff(i,4)*ff(j,4)/(i+j-7) for i,j >= 4, else 0.
  Eigen::MatrixXd q_local(kOrder + 1, kOrder + 1);
  for (int i = 0; i <= kOrder; ++i) {
    for (int j = 0; j <= kOrder; ++j) {
      if (i >= 4 && j >= 4) {
        q_local(i, j) = fallingFactorial(i, 4) * fallingFactorial(j, 4) /
          static_cast<double>(i + j - 7);
      } else {
        q_local(i, j) = 0.0;
      }
    }
  }
  // Scale each segment block by 1/T^7 so the objective is real-time snap, not normalised snap.
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(n_coeff, n_coeff);
  for (int s = 0; s < M; ++s) {
    const double inv_t7 = 1.0 / std::pow(times[static_cast<std::size_t>(s)], 7.0);
    Q.block(s * (kOrder + 1), s * (kOrder + 1), kOrder + 1, kOrder + 1) = inv_t7 * q_local;
  }

  // --- equality constraints ------------------------------------------------------------
  // Position at every waypoint (both sides of interior boundaries), C^4 continuity at each
  // interior boundary, and derivatives 1..4 zero at both ends.
  struct Row
  {
    Eigen::RowVectorXd vec;
    int wp_index{-1};  // waypoint index for a position constraint, else -1 (rhs = 0)
  };
  std::vector<Row> rows;
  auto addPos = [&](int seg, double tau, int wp_index) {
    Row r;
    r.vec = Eigen::RowVectorXd::Zero(n_coeff);
    double tau_k = 1.0;
    for (int k = 0; k <= kOrder; ++k) {
      r.vec[seg * (kOrder + 1) + k] = tau_k;
      tau_k *= tau;
    }
    r.wp_index = wp_index;
    rows.push_back(std::move(r));
  };
  auto addDerivRow = [&](int seg, double tau, int d) {
    Row r;
    r.vec = Eigen::RowVectorXd::Zero(n_coeff);
    for (int k = d; k <= kOrder; ++k) {
      r.vec[seg * (kOrder + 1) + k] = fallingFactorial(k, d) * std::pow(tau, k - d);
    }
    rows.push_back(std::move(r));
  };

  // w_0 at the start of segment 0, w_M at the end of segment M-1, and every interior
  // waypoint pinned by both neighbouring segments.
  addPos(0, 0.0, 0);
  for (int i = 1; i < M; ++i) {
    addPos(i - 1, 1.0, i);
    addPos(i, 0.0, i);
  }
  addPos(M - 1, 1.0, M);

  // C^4 continuity at every interior boundary:
  //   d^k/dtau^k(seg i-1 @ tau=1) - d^k/dtau^k(seg i @ tau=0) = 0.
  for (int i = 1; i < M; ++i) {
    for (int d = 1; d <= 4; ++d) {
      Row r;
      r.vec = Eigen::RowVectorXd::Zero(n_coeff);
      for (int k = d; k <= kOrder; ++k) {
        r.vec[(i - 1) * (kOrder + 1) + k] += fallingFactorial(k, d);  // tau = 1
        r.vec[i * (kOrder + 1) + k] -= fallingFactorial(k, d);        // tau = 0: only k == d
      }
      rows.push_back(std::move(r));
    }
  }

  // Boundary derivatives 1..4 zero at the start and the end.
  for (int d = 1; d <= 4; ++d) {
    addDerivRow(0, 0.0, d);
    addDerivRow(M - 1, 1.0, d);
  }

  // --- assemble the KKT system -----------------------------------------------------------
  const int n_cons = static_cast<int>(rows.size());
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_cons, n_coeff);
  for (int i = 0; i < n_cons; ++i) {
    A.row(i) = rows[i].vec;
  }

  Eigen::MatrixXd KKT = Eigen::MatrixXd::Zero(n_coeff + n_cons, n_coeff + n_cons);
  KKT.topLeftCorner(n_coeff, n_coeff) = Q;
  KKT.topRightCorner(n_coeff, n_cons) = A.transpose();
  KKT.bottomLeftCorner(n_cons, n_coeff) = A;

  // Per-axis RHS: position constraints take the waypoint value of that axis (yaw holds
  // fixed_yaw at every waypoint); every other row has rhs = 0.
  Eigen::MatrixXd rhs = Eigen::MatrixXd::Zero(n_coeff + n_cons, 4);
  for (int i = 0; i < n_cons; ++i) {
    if (rows[i].wp_index >= 0) {
      const Eigen::Vector3d & p =
        params_.waypoints[static_cast<std::size_t>(rows[i].wp_index)];
      rhs(n_coeff + i, 0) = p.x();
      rhs(n_coeff + i, 1) = p.y();
      rhs(n_coeff + i, 2) = p.z();
      rhs(n_coeff + i, 3) = params_.fixed_yaw;
    }
  }

  Eigen::MatrixXd sol;
  Eigen::LDLT<Eigen::MatrixXd> ldlt(KKT);
  if (ldlt.info() == Eigen::Success) {
    sol = ldlt.solve(rhs).topRows(n_coeff);
  } else {
    // Singular / near-singular factorisation: fall back to the dense LU and log it (§5.3).
    std::fprintf(
      stderr, "[trajectory_generator] WARN: LDLT failed, falling back to FullPivLU\n");
    sol = KKT.fullPivLu().solve(rhs).topRows(n_coeff);
  }

  // --- store the segments ---------------------------------------------------------------
  segments_.clear();
  segments_.reserve(static_cast<std::size_t>(M));
  for (int s = 0; s < M; ++s) {
    PolynomialSegment ps;
    ps.duration = times[static_cast<std::size_t>(s)];
    for (int axis = 0; axis < 4; ++axis) {
      ps.coeffs.col(axis) = sol.block(s * (kOrder + 1), axis, kOrder + 1, 1);
    }
    segments_.push_back(ps);
  }
}

std::vector<double> TrajectoryGenerator::allocateSegmentTimes() const
{
  // T_i = 1.2 * max(d_i/v_max, sqrt(2 d_i / a_max)), floored at 0.1 s (§5.4).
  std::vector<double> times;
  const auto & wp = params_.waypoints;
  for (std::size_t i = 0; i + 1 < wp.size(); ++i) {
    const double d = (wp[i + 1] - wp[i]).norm();
    const double t_vel = d / params_.max_velocity;
    const double t_acc = std::sqrt(2.0 * d / params_.max_acceleration);
    times.push_back(std::max(1.2 * std::max(t_vel, t_acc), 0.1));
  }
  return times;
}

FlatState TrajectoryGenerator::evaluateSegment(std::size_t idx, double tau) const
{
  const PolynomialSegment & seg = segments_[idx];
  const double T = seg.duration;
  const double tc = std::min(std::max(tau, 0.0), 1.0);

  FlatState s;
  Eigen::Vector3d pos, vel, acc, jrk, snp;
  double yaw = 0.0, yaw_d = 0.0, yaw_dd = 0.0;
  for (int axis = 0; axis < 4; ++axis) {
    double p = 0.0, v = 0.0, a = 0.0, j = 0.0, s4 = 0.0;
    double tau_k = 1.0;
    for (int k = 0; k <= PolynomialSegment::kOrder; ++k) {
      const double c = seg.coeffs(k, axis);
      p += c * tau_k;
      if (k >= 1) {
        v += fallingFactorial(k, 1) * c * (k >= 1 ? std::pow(tc, k - 1) : 0.0);
      }
      if (k >= 2) {
        a += fallingFactorial(k, 2) * c * (k >= 2 ? std::pow(tc, k - 2) : 0.0);
      }
      if (k >= 3) {
        j += fallingFactorial(k, 3) * c * (k >= 3 ? std::pow(tc, k - 3) : 0.0);
      }
      if (k >= 4) {
        s4 += fallingFactorial(k, 4) * c * (k >= 4 ? std::pow(tc, k - 4) : 0.0);
      }
      tau_k *= tc;
    }
    // Chain rule: d^k/dt^k = d^k/dtau^k / T^k (§5.3).
    const double inv_t = 1.0 / T;
    v *= inv_t;
    a *= inv_t * inv_t;
    j *= inv_t * inv_t * inv_t;
    s4 *= inv_t * inv_t * inv_t * inv_t;
    if (axis == 0) {
      pos.x() = p; vel.x() = v; acc.x() = a; jrk.x() = j; snp.x() = s4;
    } else if (axis == 1) {
      pos.y() = p; vel.y() = v; acc.y() = a; jrk.y() = j; snp.y() = s4;
    } else if (axis == 2) {
      pos.z() = p; vel.z() = v; acc.z() = a; jrk.z() = j; snp.z() = s4;
    } else {
      yaw = p; yaw_d = v; yaw_dd = a;
    }
  }
  s.position = pos;
  s.velocity = vel;
  s.acceleration = acc;
  s.jerk = jrk;
  s.snap = snp;
  s.yaw = yaw;
  s.yaw_rate = yaw_d;
  s.yaw_accel = yaw_dd;
  return s;
}

double TrajectoryGenerator::rampScale(double t, double ramp_time, int derivative_order)
{
  if (ramp_time <= 0.0) {
    return derivative_order == 0 ? 1.0 : 0.0;
  }
  const double u = std::min(std::max(t / ramp_time, 0.0), 1.0);
  const double u2 = u * u;
  const double u3 = u2 * u;
  const double u4 = u3 * u;
  const double u5 = u4 * u;
  const double u6 = u5 * u;
  const double u7 = u6 * u;
  const double u8 = u7 * u;
  const double u9 = u8 * u;

  double s = 0.0;
  switch (derivative_order) {
    case 0:
      // S4(u) = 70u^9 - 315u^8 + 540u^7 - 420u^6 + 126u^5
      s = 70.0 * u9 - 315.0 * u8 + 540.0 * u7 - 420.0 * u6 + 126.0 * u5;
      break;
    case 1:
      s = 630.0 * u8 - 2520.0 * u7 + 3780.0 * u6 - 2520.0 * u5 + 630.0 * u4;
      break;
    case 2:
      s = 5040.0 * u7 - 17640.0 * u6 + 22680.0 * u5 - 12600.0 * u4 + 2520.0 * u3;
      break;
    case 3:
      s = 35280.0 * u6 - 105840.0 * u5 + 113400.0 * u4 - 50400.0 * u3 + 7560.0 * u2;
      break;
    case 4:
      s = 211680.0 * u5 - 529200.0 * u4 + 453600.0 * u3 - 151200.0 * u2 + 15120.0 * u;
      break;
    default:
      s = 0.0;
      break;
  }
  // Chain rule: d^k/dt^k = S4^(k)(u) / ramp_time^k.
  return s / std::pow(ramp_time, derivative_order);
}

}  // namespace uav_mpc
