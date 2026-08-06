// Copyright (c) 2026 Ali-Eimaan.
// SPDX-License-Identifier: BSD-3-Clause
//
// Frame conversion tests (§10.4): ENU<->NED and FLU<->FRD round-trips to 1e-12 over 1000
// random samples, plus three hand-computed cases (level, 90° yaw, 30° roll).

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <random>

#include "uav_mpc/quadrotor_dynamics.hpp"

namespace uav_mpc
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

/// World rotation matrix P such that v_ned = P * v_enu: (x, y, z)_enu -> (y, x, -z)_ned.
Eigen::Matrix3d enuToNedMatrix()
{
  Eigen::Matrix3d P;
  P << 0.0, 1.0, 0.0,
    1.0, 0.0, 0.0,
    0.0, 0.0, -1.0;
  return P;
}

/// Random rotation quaternion from three uniform Euler angles.
Eigen::Quaterniond randomQuat(std::mt19937 & rng, std::uniform_real_distribution<double> & ang)
{
  return Eigen::Quaterniond(Eigen::AngleAxisd(ang(rng), Eigen::Vector3d::UnitZ())) *
         Eigen::Quaterniond(Eigen::AngleAxisd(ang(rng), Eigen::Vector3d::UnitY())) *
         Eigen::Quaterniond(Eigen::AngleAxisd(ang(rng), Eigen::Vector3d::UnitX()));
}

}  // namespace

TEST(FrameConversions, EnuToNedHandCase)
{
  // (x, y, z)_ENU -> (y, x, -z)_NED.
  const Eigen::Vector3d v_enu(1.0, 2.0, 3.0);
  const Eigen::Vector3d v_ned = enuToNed(v_enu);
  EXPECT_NEAR(v_ned.x(), 2.0, 1e-15);
  EXPECT_NEAR(v_ned.y(), 1.0, 1e-15);
  EXPECT_NEAR(v_ned.z(), -3.0, 1e-15);
}

TEST(FrameConversions, EnuNedRoundTrip1000)
{
  std::mt19937 rng(1234);
  std::uniform_real_distribution<double> dist(-100.0, 100.0);
  for (int i = 0; i < 1000; ++i) {
    const Eigen::Vector3d v(dist(rng), dist(rng), dist(rng));
    const Eigen::Vector3d round = nedToEnu(enuToNed(v));
    EXPECT_LT((round - v).norm(), 1e-12);
  }
}

TEST(FrameConversions, QuaternionLevelHandCase)
{
  // Level ENU-FLU vehicle: q_ned_frd = q_a (x) q_b = (-sqrt(2)/2, 0, 0, -sqrt(2)/2), i.e. a
  // 180° turn about z_ned (the famous "level in ENU/FLU == yaw 180° in NED/FRD" offset is
  // actually 90° of yaw here: R(q) = Rz(pi/2) in NED for the x/y swap).
  const Eigen::Quaterniond q_enu_flu(1.0, 0.0, 0.0, 0.0);
  const Eigen::Quaterniond q_ned_frd = quatEnuFluToNedFrd(q_enu_flu);
  EXPECT_NEAR(q_ned_frd.w(), -std::sqrt(2.0) / 2.0, 1e-12);
  EXPECT_NEAR(q_ned_frd.x(), 0.0, 1e-12);
  EXPECT_NEAR(q_ned_frd.y(), 0.0, 1e-12);
  EXPECT_NEAR(q_ned_frd.z(), -std::sqrt(2.0) / 2.0, 1e-12);
}

TEST(FrameConversions, QuaternionYaw90HandCase)
{
  // 90° yaw in ENU-FLU (vehicle faces north): identity attitude in NED-FRD.
  const Eigen::Quaterniond q_enu_flu(std::sqrt(2.0) / 2.0, 0.0, 0.0, std::sqrt(2.0) / 2.0);
  const Eigen::Quaterniond q_ned_frd = quatEnuFluToNedFrd(q_enu_flu);
  EXPECT_NEAR(q_ned_frd.w(), -1.0, 1e-12);
  EXPECT_NEAR(q_ned_frd.x(), 0.0, 1e-12);
  EXPECT_NEAR(q_ned_frd.y(), 0.0, 1e-12);
  EXPECT_NEAR(q_ned_frd.z(), 0.0, 1e-12);
}

TEST(FrameConversions, QuaternionRoll30HandCase)
{
  // 30° roll about x_flu. R(q') = P * Rx(30°) * diag(1,-1,-1) =
  //   [[0, -sqrt(3)/2, 1/2], [1, 0, 0], [0, 1/2, sqrt(3)/2]].
  const double c15 = std::cos(15.0 * kPi / 180.0);
  const double s15 = std::sin(15.0 * kPi / 180.0);
  const Eigen::Quaterniond q_enu_flu(c15, s15, 0.0, 0.0);
  const Eigen::Quaterniond q_ned_frd = quatEnuFluToNedFrd(q_enu_flu);
  Eigen::Matrix3d expected;
  expected << 0.0, -std::sqrt(3.0) / 2.0, 0.5,
              1.0, 0.0, 0.0,
              0.0, 0.5, std::sqrt(3.0) / 2.0;
  EXPECT_LT((q_ned_frd.toRotationMatrix() - expected).norm(), 1e-12);
}

TEST(FrameConversions, QuaternionTransformLaw1000)
{
  // The defining property: R(quatEnuFluToNedFrd(q)) == P * R(q) * diag(1,-1,-1) (world change
  // on the left, body-frame change on the right). Catches a wrong q_a/q_b pair immediately.
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> ang(-kPi, kPi);
  const Eigen::Matrix3d P = enuToNedMatrix();
  Eigen::Matrix3d Rb = Eigen::Matrix3d::Identity();
  Rb(1, 1) = -1.0;
  Rb(2, 2) = -1.0;
  for (int i = 0; i < 1000; ++i) {
    const Eigen::Quaterniond q = randomQuat(rng, ang);
    const Eigen::Quaterniond q2 = quatEnuFluToNedFrd(q);
    const Eigen::Matrix3d expected = P * q.toRotationMatrix() * Rb;
    EXPECT_LT((q2.toRotationMatrix() - expected).norm(), 1e-12);
  }
}

TEST(FrameConversions, QuaternionRoundTrip1000)
{
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> ang(-kPi, kPi);
  for (int i = 0; i < 1000; ++i) {
    const Eigen::Quaterniond q = randomQuat(rng, ang);
    const Eigen::Quaterniond back = quatNedFrdToEnuFlu(quatEnuFluToNedFrd(q));
    // q and -q are the same rotation, so compare via |dot| == 1.
    const double dot = std::abs(q.dot(back));
    EXPECT_NEAR(dot, 1.0, 1e-12);
  }
}

}  // namespace uav_mpc
