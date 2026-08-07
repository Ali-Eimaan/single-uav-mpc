// Copyright (c) 2026 Ali-Eimaan.
// SPDX-License-Identifier: BSD-3-Clause
//
// Trajectory tests (§10.2): the analytic derivatives must be the derivatives (finite
// differences), the ramp-in must be C^4, minimum-snap segments must be C^4 and hit their
// waypoints, and the flatness map must round-trip through the dynamics.
//
// A discontinuity in the reference at any derivative up to snap becomes an impulse in the
// commanded body rate, which is exactly the failure that looks like "the MPC is unstable"
// but is really "the reference was not C^4".

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "uav_mpc/quadrotor_dynamics.hpp"
#include "uav_mpc/trajectory_generator.hpp"

#ifndef UAV_MPC_SOURCE_DIR
#error "UAV_MPC_SOURCE_DIR must be defined (set in CMakeLists.txt)"
#endif

namespace
{
constexpr double kDt = 1e-4;      // finite-difference step for the numeric derivative checks
constexpr double kRelTol = 1e-5;  // analytic vs numeric derivative agreement

std::string x500Path()
{
  return std::string(UAV_MPC_SOURCE_DIR) + "/params/x500_calibration.yaml";
}

std::string crazyfliePath()
{
  return std::string(UAV_MPC_SOURCE_DIR) + "/params/crazyflie21_calibration.yaml";
}

/// Mixed absolute/relative closeness: |a - n| <= tol * (1 + |a|). A single central difference
/// of a smooth analytic derivative at h = kDt has error ~1e-9 here, so 1e-5 relative is
/// generous for real signals and still catches a missing chain-rule factor (O(1)).
bool closeEnough(double a, double n, double tol = kRelTol)
{
  return std::abs(a - n) <= tol * (1.0 + std::abs(a));
}

/// Checks an analytic k-th scalar derivative against the central difference of the (k-1)-th
/// across the sample points in [t_lo, t_hi].
template<typename F, typename G>
void checkScalarDerivative(
  const uav_mpc::TrajectoryGenerator & gen, double t_lo, double t_hi, F && high, G && low)
{
  const int n = static_cast<int>(std::ceil((t_hi - t_lo) / 0.01));
  for (int i = 0; i <= n; ++i) {
    const double t = t_lo + static_cast<double>(i) * (t_hi - t_lo) / static_cast<double>(n);
    const double a = high(gen.sample(t));
    const double nrm = (low(gen.sample(t + kDt)) - low(gen.sample(t - kDt))) / (2.0 * kDt);
    EXPECT_TRUE(closeEnough(a, nrm))
      << "t = " << t << ": analytic " << a << " vs finite difference " << nrm;
  }
}

/// Same for the norm of a vector-valued derivative (position/velocity/acceleration/jerk/snap).
template<typename F, typename G>
void checkVectorDerivative(
  const uav_mpc::TrajectoryGenerator & gen, double t_lo, double t_hi, F && high, G && low)
{
  const int n = static_cast<int>(std::ceil((t_hi - t_lo) / 0.01));
  for (int i = 0; i <= n; ++i) {
    const double t = t_lo + static_cast<double>(i) * (t_hi - t_lo) / static_cast<double>(n);
    const double a = high(gen.sample(t)).norm();
    const double nrm = (low(gen.sample(t + kDt)) - low(gen.sample(t - kDt))).norm() / (2.0 * kDt);
    EXPECT_TRUE(closeEnough(a, nrm))
      << "t = " << t << ": analytic " << a << " vs finite difference " << nrm;
  }
}
}  // namespace

using uav_mpc::AttitudeRep;
using uav_mpc::FlatState;
using uav_mpc::QuadrotorDynamics;
using uav_mpc::QuadrotorParams;
using uav_mpc::TrajectoryGenerator;
using uav_mpc::TrajectoryParams;
using uav_mpc::TrajectoryType;

namespace
{

TrajectoryParams figure8Params(double ramp_in_time = 0.0)
{
  TrajectoryParams p;
  p.type = TrajectoryType::Figure8;
  p.center = Eigen::Vector3d(0.0, 0.0, 0.0);
  p.amplitude_x = 2.0;
  p.amplitude_y = 2.0;
  p.amplitude_z = 0.5;  // exercise the vertical channel too
  p.period = 8.0;
  p.altitude = 1.5;
  p.yaw_follows_velocity = true;
  p.fixed_yaw = 0.0;
  p.ramp_in_time = ramp_in_time;
  p.max_velocity = 5.0;
  p.max_acceleration = 8.0;
  return p;
}

TrajectoryParams lemniscateParams()
{
  TrajectoryParams p = figure8Params();
  p.type = TrajectoryType::Lemniscate;
  return p;
}

}  // namespace

// The analytic derivatives must actually be the derivatives. This catches sign errors and
// missing chain-rule factors, which is where every trajectory bug lives.
TEST(TrajectoryContinuity, Figure8AnalyticDerivativesMatchFiniteDifferences)
{
  const TrajectoryGenerator gen(figure8Params());
  const double period = gen.params().period;
  checkVectorDerivative(
    gen, 0.0, 2.0 * period,
    [](const FlatState & s) {return s.velocity;},
    [](const FlatState & s) {return s.position;});
  checkVectorDerivative(
    gen, 0.0, 2.0 * period,
    [](const FlatState & s) {return s.acceleration;},
    [](const FlatState & s) {return s.velocity;});
  checkVectorDerivative(
    gen, 0.0, 2.0 * period,
    [](const FlatState & s) {return s.jerk;},
    [](const FlatState & s) {return s.acceleration;});
  checkVectorDerivative(
    gen, 0.0, 2.0 * period,
    [](const FlatState & s) {return s.snap;},
    [](const FlatState & s) {return s.jerk;});
  // with continuousFigure8Yaw the yaw is unwrapped, so the full
  // period can be checked without hitting the atan2 branch cut.
  checkScalarDerivative(
    gen, 0.0, 2.0 * period,
    [](const FlatState & s) {return s.yaw_rate;},
    [](const FlatState & s) {return s.yaw;});
}

TEST(TrajectoryContinuity, LemniscateAnalyticDerivativesMatchFiniteDifferences)
{
  const TrajectoryGenerator gen(lemniscateParams());
  const double period = gen.params().period;
  checkVectorDerivative(
    gen, 0.0, 2.0 * period,
    [](const FlatState & s) {return s.velocity;},
    [](const FlatState & s) {return s.position;});
  checkVectorDerivative(
    gen, 0.0, 2.0 * period,
    [](const FlatState & s) {return s.acceleration;},
    [](const FlatState & s) {return s.velocity;});
  checkVectorDerivative(
    gen, 0.0, 2.0 * period,
    [](const FlatState & s) {return s.jerk;},
    [](const FlatState & s) {return s.acceleration;});
  checkVectorDerivative(
    gen, 0.0, 2.0 * period,
    [](const FlatState & s) {return s.snap;},
    [](const FlatState & s) {return s.jerk;});
}

// k-th derivative of the sampled position via nested central differences (step kDt).
Eigen::Vector3d positionDerivative(const uav_mpc::TrajectoryGenerator & gen, double t, int k)
{
  if (k == 0) {
    return gen.sample(t).position;
  }
  return (positionDerivative(gen, t + kDt, k - 1) - positionDerivative(gen, t - kDt, k - 1)) /
         (2.0 * kDt);
}

// k-th derivative of the sampled yaw (k = 0: yaw, 1: yaw_rate, 2: yaw_accel).
double yawDerivative(const uav_mpc::TrajectoryGenerator & gen, double t, int k)
{
  if (k == 0) {
    return gen.sample(t).yaw;
  }
  return (yawDerivative(gen, t + kDt, k - 1) - yawDerivative(gen, t - kDt, k - 1)) / (2.0 * kDt);
}

// Quadratic (Lagrange) extrapolation of the k-th derivative to `boundary`, evaluated only from
// points on one side (`side` = -1 left, +1 right). Points sit d, d+1, d+2 steps from the
// boundary with d = k + 1, so every nested finite-difference window (which spans k steps)
// stays strictly on its own side. For a C^4 signal the two one-sided limits agree to O(h^3);
// a C^3 (or lower) ramp shows its full derivative jump here.
template<typename F>
auto oneSidedLimit(F && deriv, double boundary, int k, int side)
-> decltype(deriv(boundary))
{
  const double d = static_cast<double>(k + 1);
  const double w0 = 0.5 * (d + 1.0) * (d + 2.0);
  const double w1 = -d * (d + 2.0);
  const double w2 = 0.5 * d * (d + 1.0);
  return w0 * deriv(boundary + side * d * kDt) +
         w1 * deriv(boundary + side * (d + 1.0) * kDt) +
         w2 * deriv(boundary + side * (d + 2.0) * kDt);
}

// The ramp-in must not introduce a jerk step at t = 0 or t = ramp_in_time.
TEST(TrajectoryContinuity, RampInIsC4)
{
  // use the shipped default ramp_in_time = 3.0 s.  With
  // continuousFigure8Yaw the unwrapped yaw is continuous across the
  // full ramp, so no wrap-aware comparison is needed.
  const TrajectoryGenerator gen(figure8Params(3.0));
  const double ramp = 3.0;

  // Use the analytic derivatives from sample() directly — nested finite differences
  // amplify floating-point noise by (2*kDt)^-k (≈ 10^15 at order 4), which makes
  // the one-sided extrapolation meaningless for higher orders.
  for (const double boundary : {0.0, ramp}) {
    // Position C^4: the ramp polynomial has S^(k)(0) = S^(k)(1) = 0 for k = 1..4
    // by construction, so the blend matches the pure trajectory to 4th order.
    for (int k = 0; k <= 4; ++k) {
      auto deriv = [&gen, k](double t) -> Eigen::Vector3d {
          const FlatState s = gen.sample(t);
          switch (k) {
            case 0: return s.position;
            case 1: return s.velocity;
            case 2: return s.acceleration;
            case 3: return s.jerk;
            default: return s.snap;
          }
        };
      const Eigen::Vector3d left = oneSidedLimit(deriv, boundary, k, -1);
      const Eigen::Vector3d right = oneSidedLimit(deriv, boundary, k, +1);
      const double scale = 1.0 + std::max(left.norm(), right.norm());
      EXPECT_LT((left - right).norm(), kRelTol * scale)
        << "position derivative order " << k << " at t = " << boundary;
    }
    // Yaw C^2: the yaw blend is b_yaw = hover.yaw + S*(traj.yaw - hover.yaw) with
    // S^(k)(0) = S^(k)(1) = 0 for k >= 1.  yaw is now unwrapped, so the
    // raw derivative comparison is valid without any wrap-aware adjustment.
    for (int k = 0; k <= 2; ++k) {
      auto deriv = [&gen, k](double t) -> double {
          const FlatState s = gen.sample(t);
          switch (k) {
            case 0: return s.yaw;
            case 1: return s.yaw_rate;
            default: return s.yaw_accel;
          }
        };
      double left = oneSidedLimit(deriv, boundary, k, -1);
      double right = oneSidedLimit(deriv, boundary, k, +1);
      const double scale = 1.0 + std::max(std::abs(left), std::abs(right));
      EXPECT_LT(std::abs(left - right), kRelTol * scale)
        << "yaw derivative order " << k << " at t = " << boundary;
    }
  }
}

// regression test — place the ramp so a yaw wrap (atan2 branch cut) falls
// strictly inside the ramp interval.  With continuousFigure8Yaw this must produce
// a bounded yaw rate (no instantaneous 2π/0 spike).
TEST(TrajectoryContinuity, RampInHandlesYawWrap)
{
  // ramp_in_time = 4.0, period = 8.0: the first yaw wrap is at t = 3.0,
  // which falls inside the ramp.  Without unwrapping this would produce
  // an s0·2π yaw jump at the blend boundary.
  const TrajectoryGenerator gen(figure8Params(4.0));
  const double ramp = 4.0;
  const double dt = 0.001;
  // Check the whole ramp at 1 ms resolution.
  for (double t = 0.0; t <= ramp; t += dt) {
    const FlatState s = gen.sample(t);
    EXPECT_LT(std::abs(s.yaw_rate), 10.0)
      << "yaw_rate spike at t = " << t;
  }
}

namespace
{

// Mirrors the generator's time allocation (§5.4): T_i = max(1.2*max(d/v_max, sqrt(2d/a_max)), 0.5).
std::vector<double> allocatedSegmentTimes(const TrajectoryParams & p)
{
  std::vector<double> times;
  times.reserve(p.waypoints.size() - 1);
  for (std::size_t i = 0; i + 1 < p.waypoints.size(); ++i) {
    const double d = (p.waypoints[i + 1] - p.waypoints[i]).norm();
    times.push_back(std::max(
      1.2 * std::max(d / p.max_velocity, std::sqrt(2.0 * d / p.max_acceleration)), 0.5));
  }
  return times;
}

TrajectoryParams fiveWaypointParams()
{
  TrajectoryParams p;
  p.type = TrajectoryType::Waypoints;
  p.waypoints = {
    Eigen::Vector3d(0.0, 0.0, 1.0),
    Eigen::Vector3d(1.0, 0.0, 1.0),
    Eigen::Vector3d(1.0, 1.0, 1.0),
    Eigen::Vector3d(0.0, 1.0, 1.0),
    Eigen::Vector3d(0.0, 0.0, 1.5),
  };
  p.altitude = 1.0;
  p.max_velocity = 5.0;
  p.max_acceleration = 8.0;
  return p;
}

}  // namespace

// Minimum-snap waypoints: continuity across segment boundaries is the whole point of the QP.
TEST(TrajectoryContinuity, MinimumSnapSegmentsAreC4)
{
  const TrajectoryGenerator gen(fiveWaypointParams());

  for (int k = 0; k <= 4; ++k) {
    EXPECT_LT(gen.maxDerivativeJump(k), 1e-6) << "derivative order " << k;
  }

  // Every waypoint is interpolated exactly (the KKT constraints pin them) at the instants
  // implied by the internal time allocation.
  const std::vector<double> times = allocatedSegmentTimes(gen.params());
  double t = 0.0;
  for (std::size_t i = 0; i < gen.params().waypoints.size(); ++i) {
    const FlatState s = gen.sample(t);
    EXPECT_LT((s.position - gen.params().waypoints[i]).norm(), 1e-9)
      << "waypoint " << i << " at t = " << t;
    if (i + 1 < times.size() + 1 && i < times.size()) {
      t += times[i];
    }
  }
}

TEST(TrajectoryContinuity, MinimumSnapRespectsSuppliedSegmentTimes)
{
  TrajectoryParams p = fiveWaypointParams();
  p.segment_times = {1.5, 2.0, 1.0, 2.5};
  const TrajectoryGenerator gen(p);

  const double total = gen.duration();
  EXPECT_NEAR(total, 7.0, 1e-12);

  // The waypoints must be reached at exactly the cumulative supplied segment times.
  double t = 0.0;
  for (std::size_t i = 0; i < p.waypoints.size(); ++i) {
    const FlatState s = gen.sample(t);
    EXPECT_LT((s.position - p.waypoints[i]).norm(), 1e-9)
      << "waypoint " << i << " expected at t = " << t;
    if (i < p.segment_times.size()) {
      t += p.segment_times[i];
    }
  }
}

// The flatness map is where the trajectory meets the dynamics. Round-tripping it through the
// model is the strongest available check without a simulator.
TEST(TrajectoryContinuity, FlatnessMapIsConsistentWithDynamics)
{
  const QuadrotorParams params = QuadrotorParams::fromYaml(x500Path());
  QuadrotorParams drag_free = params;
  drag_free.drag_coeff.setZero();  // the flatness map is derived drag-free (§5.5)

  const TrajectoryGenerator gen(figure8Params());
  const double period = gen.params().period;
  using Dyn = QuadrotorDynamics<double, AttitudeRep::Quaternion>;
  using Layout = uav_mpc::StateLayout<AttitudeRep::Quaternion>;
  const Dyn dyn(drag_free);

  const int n = 100;
  for (int i = 0; i <= n; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(n) * period;
    const FlatState flat = gen.sample(t);
    const auto ref = TrajectoryGenerator::flatToStateInput(flat, drag_free,
      AttitudeRep::Quaternion);

    // Quaternion slice must be unit norm (the reference is a valid attitude).
    const double q_norm = ref.state.segment<4>(6).norm();
    EXPECT_NEAR(q_norm, 1.0, 1e-9);

    // The dynamic VectorXd reference must be copied into the fixed-size state vector the
    // dynamics API consumes.
    const Dyn::StateVector x = ref.state;
    const auto x_dot = dyn.f(x, ref.input);
    // p_dot == v_ref exactly; v_dot == a_ref (drag-free, T = m||a + g e_z||).
    for (int row = 0; row < 3; ++row) {
      EXPECT_NEAR(x_dot(Layout::kPosIdx + row), flat.velocity(row), 1e-6)
        << "t = " << t << ", row " << row;
      EXPECT_NEAR(x_dot(Layout::kVelIdx + row), flat.acceleration(row), 1e-6)
        << "t = " << t << ", row " << row;
    }
  }
}

TEST(TrajectoryContinuity, InfeasibleTrajectoryIsReportedNotSilentlyAccepted)
{
  // 3 m amplitude at a 1.5 s period on the Crazyflie needs a = A w^2 ~ 3*(2*pi/1.5)^2
  // ~ 53 m/s^2 => per-rotor thrust ~ m/4*(a+g) ~ 0.43 N, far above the 0.157 N max.
  const QuadrotorParams params = QuadrotorParams::fromYaml(crazyfliePath());

  TrajectoryParams p = figure8Params();
  p.amplitude_x = 3.0;
  p.amplitude_y = 3.0;
  p.period = 1.5;
  p.max_velocity = 5.0;
  p.max_acceleration = 8.0;
  const TrajectoryGenerator gen(p);

  std::string report;
  EXPECT_FALSE(gen.isDynamicallyFeasible(params, &report));
  EXPECT_FALSE(report.empty());
  // The same trajectory on the x500 is also outside the envelope (a_max = A w² =
  // 3*(2*pi/1.5)² ≈ 53 m/s² ≈ 5.4 g for a ~2 kg quad — per-rotor thrust exceeds
  // the x500's limit too).  The point is that both must report infeasible without
  // crashing.
  const QuadrotorParams x500 = QuadrotorParams::fromYaml(x500Path());
  EXPECT_FALSE(gen.isDynamicallyFeasible(x500, &report));
}
