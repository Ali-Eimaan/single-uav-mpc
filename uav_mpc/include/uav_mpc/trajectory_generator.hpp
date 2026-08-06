// Copyright (c) 2026 Ali-Eimaan.
// SPDX-License-Identifier: BSD-3-Clause
//
// See .deepseek/05_TRAJECTORY.md §5.
//
// Minimum-snap polynomial trajectory generation (Mellinger & Kumar, ICRA 2011) plus closed-form
// analytic primitives (figure-8 / lemniscate / circle). Produces a flat-output reference
// (position, yaw and derivatives) and maps it through the differential-flatness map to a full
// state+input reference for the NMPC stage cost.

#ifndef UAV_MPC__TRAJECTORY_GENERATOR_HPP_
#define UAV_MPC__TRAJECTORY_GENERATOR_HPP_

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

#include "uav_mpc/quadrotor_dynamics.hpp"

namespace uav_mpc
{

/// Flat outputs sigma = (x, y, z, psi) and their derivatives up to snap / yaw-acceleration.
struct FlatState
{
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
  Eigen::Vector3d jerk{Eigen::Vector3d::Zero()};
  Eigen::Vector3d snap{Eigen::Vector3d::Zero()};
  double yaw{0.0};
  double yaw_rate{0.0};
  double yaw_accel{0.0};
  double t{0.0};  ///< [s] time since trajectory start
};

/// Full reference for one NMPC stage: y_ref = [x_ref(nx); u_ref(nu)].
struct StateInputReference
{
  Eigen::VectorXd state;  ///< nx x 1, layout matching the active AttitudeRep
  Eigen::Vector4d input;  ///< per-rotor thrusts [N] at the reference (feed-forward)
};

enum class TrajectoryType
{
  Hover,        ///< hold a single setpoint
  Figure8,      ///< Gerono lemniscate: x = A sin(wt), y = B sin(wt) cos(wt)
  Lemniscate,   ///< Bernoulli lemniscate (tighter cusps, higher peak jerk)
  Circle,
  Waypoints,    ///< minimum-snap through a user list
  Step          ///< position step, for disturbance/step-response tests
};

/// Tuning knobs; mirrors config/trajectory_params.yaml.
struct TrajectoryParams
{
  TrajectoryType type{TrajectoryType::Hover};
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};
  double amplitude_x{1.0};      ///< [m]
  double amplitude_y{1.0};      ///< [m]
  double amplitude_z{0.0};      ///< [m] vertical bob amplitude
  double period{8.0};           ///< [s] one full lap
  double altitude{1.5};         ///< [m] nominal AGL
  bool yaw_follows_velocity{true};
  double fixed_yaw{0.0};        ///< [rad] used when yaw_follows_velocity == false
  double ramp_in_time{3.0};     ///< [s] cosine blend from hover into the periodic orbit
  std::vector<Eigen::Vector3d> waypoints{};
  std::vector<double> segment_times{};   ///< empty => allocate by the heuristic in §5.4
  double max_velocity{5.0};     ///< [m/s] used for time allocation + feasibility check
  double max_acceleration{8.0};  ///< [m/s^2]
};

/// One minimum-snap segment: a 7th-order polynomial per axis on a normalised [0, 1] domain.
struct PolynomialSegment
{
  static constexpr int kOrder = 7;             ///< snap-minimising => order 2*4-1
  Eigen::Matrix<double, kOrder + 1, 4> coeffs{  ///< columns: x, y, z, yaw
    Eigen::Matrix<double, kOrder + 1, 4>::Zero()};
  double duration{0.0};                        ///< [s]
};

/// Generates and samples references. Thread-compat: sample*() are const and re-entrant; a
/// regenerate must not run concurrently with sampling (the node holds a mutex).
class TrajectoryGenerator
{
public:
  explicit TrajectoryGenerator(const TrajectoryParams & params);

  /// Rebuild the internal representation. For analytic types this is O(1); for Waypoints it
  /// solves the banded QP / closed-form linear system of §5.3. Throws on infeasible input.
  void generate();

  /// Update parameters and regenerate.
  void setParams(const TrajectoryParams & params);

  /// Flat outputs at absolute trajectory time t [s]. Clamps to [0, duration()] for
  /// non-periodic types; wraps for periodic ones.
  FlatState sample(double t) const;

  /// Sample the whole prediction horizon in one call: t0, t0+dt, ..., t0+N*dt.
  /// Returns N+1 entries. This is what nmpc_node calls every control tick.
  std::vector<FlatState> sampleHorizon(double t0, double dt, int n_steps) const;

  /// Differential-flatness map: flat outputs -> (state, input) reference.
  /// See docs/derivations/differential_flatness.tex. Needs the airframe to convert the
  /// required body wrench into per-rotor thrusts.
  static StateInputReference flatToStateInput(
    const FlatState & flat, const QuadrotorParams & airframe, AttitudeRep rep);

  /// Non-allocating overload: pre-built {ControlAllocation} avoids constructing
  /// {QuadrotorDynamics} (with its LDLT factorisation) for every horizon point.  REVIEW R1-6.
  static StateInputReference flatToStateInput(
    const FlatState & flat, const QuadrotorParams & airframe,
    const ControlAllocation & alloc, AttitudeRep rep);

  /// Convenience: horizon of full references, ready to memcpy into the acados yref buffers.
  std::vector<StateInputReference> referenceHorizon(
    double t0, double dt, int n_steps, const QuadrotorParams & airframe, AttitudeRep rep) const;

  /// [s] total duration; std::numeric_limits<double>::infinity() for periodic types.
  double duration() const;

  /// True if every sampled point respects max_velocity / max_acceleration and the thrust
  /// envelope of `airframe`. Called once after generate(); logged as a warning, not fatal.
  bool isDynamicallyFeasible(
    const QuadrotorParams & airframe,
    std::string * report = nullptr) const;

  /// C^4 continuity check across all segment boundaries; used by
  /// test/test_trajectory_continuity.cpp.
  double maxDerivativeJump(int derivative_order) const;

  const TrajectoryParams & params() const {return params_;}

private:
  TrajectoryParams params_;
  std::vector<PolynomialSegment> segments_;

  // --- analytic primitives -------------------------------------------------------------------
  FlatState sampleFigure8(double t) const;
  FlatState sampleLemniscate(double t) const;
  FlatState sampleCircle(double t) const;
  FlatState sampleHover(double t) const;
  FlatState sampleStep(double t) const;

  // --- minimum snap --------------------------------------------------------------------------
  // Builds the QP and solves for `segments_` (dense KKT via Eigen LDLT, FullPivLU fallback).
  void buildMinimumSnap();
  // Heuristic time allocation when segment_times is empty (§5.4).
  std::vector<double> allocateSegmentTimes() const;
  // Evaluates segment `idx` at local time `tau` in [0, 1] (chain rule /T^k applied).
  FlatState evaluateSegment(std::size_t idx, double tau) const;

  /// Smooth 0->1 blend applied during the first `ramp_in_time` seconds so the vehicle does not
  /// step-jump onto the orbit. Must be C^4 (use a 9th-order smoothstep) or the flatness map
  /// produces a jerk spike.
  static double rampScale(double t, double ramp_time, int derivative_order);
};

}  // namespace uav_mpc

#endif  // UAV_MPC__TRAJECTORY_GENERATOR_HPP_
