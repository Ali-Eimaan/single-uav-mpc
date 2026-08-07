// Copyright (c) 2026 Ali-Eimaan.
// SPDX-License-Identifier: BSD-3-Clause
//
// Dynamics tests (§10.4): analytic Jacobians vs central differences (step 1e-6, rel. tol
// 1e-6), RK4 energy sanity in the drag-free, torque-free case, quaternion norm preserved
// after step(), plus fromYaml/isValid/allocate/hover-thrust sanity on the x500 calibration.

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <fstream>
#include <random>
#include <string>

#include "uav_mpc/quadrotor_dynamics.hpp"

#ifndef UAV_MPC_SOURCE_DIR
#error "UAV_MPC_SOURCE_DIR must be defined (set in CMakeLists.txt)"
#endif

namespace uav_mpc
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

std::string x500Path()
{
  return std::string(UAV_MPC_SOURCE_DIR) + "/params/x500_calibration.yaml";
}

/// Random state for the given attitude rep with a normalised quaternion slice.
template<AttitudeRep Rep>
typename QuadrotorDynamics<double, Rep>::StateVector randomState(std::mt19937 & rng)
{
  using Dyn = QuadrotorDynamics<double, Rep>;
  using StateVec = typename Dyn::StateVector;
  typename Dyn::StateVector x = StateVec::Zero();
  std::uniform_real_distribution<double> pos(-2.0, 2.0);
  std::uniform_real_distribution<double> vel(-3.0, 3.0);
  std::uniform_real_distribution<double> rate(-1.5, 1.5);
  std::uniform_real_distribution<double> ang(-0.6, 0.6);
  for (int i = 0; i < 3; ++i) {
    x(i) = pos(rng);
    x(3 + i) = vel(rng);
  }
  if constexpr (Rep == AttitudeRep::Quaternion) {
    const Eigen::Quaterniond q =
      Eigen::Quaterniond(Eigen::AngleAxisd(ang(rng), Eigen::Vector3d::UnitZ())) *
      Eigen::Quaterniond(Eigen::AngleAxisd(ang(rng), Eigen::Vector3d::UnitY())) *
      Eigen::Quaterniond(Eigen::AngleAxisd(ang(rng), Eigen::Vector3d::UnitX()));
    x.template segment<4>(6) << q.w(), q.x(), q.y(), q.z();
    for (int i = 0; i < 3; ++i) {
      x(10 + i) = rate(rng);
    }
  } else {
    for (int i = 0; i < 3; ++i) {
      x(6 + i) = ang(rng);
      x(9 + i) = rate(rng);
    }
  }
  return x;
}

/// Central-difference Jacobians with step h.
template<AttitudeRep Rep>
void centralDifferenceJacobians(
  const QuadrotorDynamics<double, Rep> & dyn,
  const typename QuadrotorDynamics<double, Rep>::StateVector & x,
  const typename QuadrotorDynamics<double, Rep>::InputVector & u,
  typename QuadrotorDynamics<double, Rep>::StateJacobian * A_num,
  typename QuadrotorDynamics<double, Rep>::InputJacobian * B_num)
{
  using Dyn = QuadrotorDynamics<double, Rep>;
  constexpr int Nx = Dyn::kNx;
  constexpr int Nu = Dyn::kNu;
  const double h = 1e-6;
  A_num->setZero();
  for (int j = 0; j < Nx; ++j) {
    typename Dyn::StateVector xp = x;
    typename Dyn::StateVector xm = x;
    xp(j) += h;
    xm(j) -= h;
    A_num->col(j) = (dyn.f(xp, u) - dyn.f(xm, u)) / (2.0 * h);
  }
  B_num->setZero();
  for (int j = 0; j < Nu; ++j) {
    typename Dyn::InputVector up = u;
    typename Dyn::InputVector um = u;
    up(j) += h;
    um(j) -= h;
    B_num->col(j) = (dyn.f(x, up) - dyn.f(x, um)) / (2.0 * h);
  }
}

}  // namespace

// -----------------------------------------------------------------------------------------------
// Jacobians vs central differences
// -----------------------------------------------------------------------------------------------

template<AttitudeRep Rep>
void checkJacobiansOnce(std::mt19937 & rng)
{
  using Dyn = QuadrotorDynamics<double, Rep>;
  constexpr int Nx = Dyn::kNx;
  constexpr int Nu = Dyn::kNu;
  using SV = typename Dyn::StateVector;
  using UV = typename Dyn::InputVector;

  const QuadrotorParams params = QuadrotorParams::fromYaml(x500Path());
  const Dyn dyn(params);

  std::uniform_real_distribution<double> thrust(0.5, 5.0);
  UV u;
  for (int i = 0; i < Nu; ++i) {
    u(i) = thrust(rng);
  }

  const SV x = randomState<Rep>(rng);

  typename Dyn::StateJacobian A;
  typename Dyn::InputJacobian B;
  dyn.jacobians(x, u, &A, &B);

  typename Dyn::StateJacobian A_num;
  typename Dyn::InputJacobian B_num;
  centralDifferenceJacobians<Rep>(dyn, x, u, &A_num, &B_num);

  for (int i = 0; i < Nx; ++i) {
    for (int j = 0; j < Nx; ++j) {
      const double rel = std::abs(A(i, j) - A_num(i, j)) / (1.0 + std::abs(A(i, j)));
      EXPECT_LT(rel, 1e-6) << "A(" << i << "," << j << ") analytic " << A(i, j)
                           << " vs fd " << A_num(i, j);
    }
  }
  for (int i = 0; i < Nx; ++i) {
    for (int j = 0; j < Nu; ++j) {
      const double rel = std::abs(B(i, j) - B_num(i, j)) / (1.0 + std::abs(B(i, j)));
      EXPECT_LT(rel, 1e-6) << "B(" << i << "," << j << ") analytic " << B(i, j)
                           << " vs fd " << B_num(i, j);
    }
  }
}

TEST(DynamicsJacobians, QuaternionRepMatchesFiniteDifferences)
{
  std::mt19937 rng(101);
  for (int trial = 0; trial < 10; ++trial) {
    checkJacobiansOnce<AttitudeRep::Quaternion>(rng);
                                                                                             }
}

TEST(DynamicsJacobians, EulerRepMatchesFiniteDifferences)
{
  std::mt19937 rng(202);
  for (int trial = 0; trial < 10; ++trial) {
    checkJacobiansOnce<AttitudeRep::Euler>(rng);
                                                                                        }
}

/// regression: the analytic dRy/d(theta) once used cos(roll) where cos(pitch)
/// belongs. The random sampler draws roll and pitch independently, so a collision is only
/// guaranteed to be exercised when roll != pitch — pin a state with roll = 0.2, pitch = -0.5
/// (both comfortably inside the singular-free envelope) and diff the Jacobian against central
/// differences with the full 1e-6 relative tolerance.
TEST(DynamicsJacobians, EulerRepRollDiffersFromPitchMatchesFiniteDifferences)
{
  using Dyn = QuadrotorDynamics<double, AttitudeRep::Euler>;
  using SV = Dyn::StateVector;
  using UV = Dyn::InputVector;

  const QuadrotorParams params = QuadrotorParams::fromYaml(x500Path());
  const Dyn dyn(params);

  SV x = SV::Zero();
  x(0) = 0.8; x(1) = -1.2; x(2) = 2.5;            // position
  x(3) = 1.1; x(4) = -0.6; x(5) = 0.3;            // velocity
  x(6) = 0.2; x(7) = -0.5; x(8) = 0.4;            // roll != pitch, yaw = 0.4
  x(9) = 0.7; x(10) = -0.9; x(11) = 0.5;          // body rates

  UV u;
  u << 1.2, 1.4, 1.1, 1.3;                          // nonzero thrust, nonzero torque

  typename Dyn::StateJacobian A;
  typename Dyn::InputJacobian B;
  dyn.jacobians(x, u, &A, &B);

  typename Dyn::StateJacobian A_num;
  typename Dyn::InputJacobian B_num;
  centralDifferenceJacobians<AttitudeRep::Euler>(dyn, x, u, &A_num, &B_num);

  for (int i = 0; i < Dyn::kNx; ++i) {
    for (int j = 0; j < Dyn::kNx; ++j) {
      const double rel = std::abs(A(i, j) - A_num(i, j)) / (1.0 + std::abs(A(i, j)));
      EXPECT_LT(rel, 1e-6) << "A(" << i << "," << j << ") analytic " << A(i, j)
                           << " vs fd " << A_num(i, j);
    }
  }
  for (int i = 0; i < Dyn::kNx; ++i) {
    for (int j = 0; j < Dyn::kNu; ++j) {
      const double rel = std::abs(B(i, j) - B_num(i, j)) / (1.0 + std::abs(B(i, j)));
      EXPECT_LT(rel, 1e-6) << "B(" << i << "," << j << ") analytic " << B(i, j)
                           << " vs fd " << B_num(i, j);
    }
  }
}

// -----------------------------------------------------------------------------------------------
// RK4 energy sanity (drag-free, torque-free)
// -----------------------------------------------------------------------------------------------

TEST(DynamicsStep, EnergyConservedDragFreeTorqueFree)
{
  QuadrotorParams params = QuadrotorParams::fromYaml(x500Path());
  params.drag_coeff.setZero();  // drag-free
  params.rotor_time_constant = 0.0;

  QuadrotorDynamics<double, AttitudeRep::Quaternion> dyn(params);

  const double g = params.gravity;
  const double z0 = 10.0;
  const Eigen::Vector3d v0(1.0, 0.0, 0.0);

  using QuatDyn = QuadrotorDynamics<double, AttitudeRep::Quaternion>;
  using QuatState = QuatDyn::StateVector;
  QuatState x = QuatState::Zero();
  x(2) = z0;
  x.segment<3>(3) = v0;
  x(6) = 1.0;  // identity quaternion
  const Eigen::Vector4d u = Eigen::Vector4d::Zero();  // no thrust, no torque

  const double dt = 0.01;
  const int n_steps = 100;
  const double E0 = 0.5 * v0.squaredNorm() + g * z0;

  for (int k = 0; k < n_steps; ++k) {
    x = dyn.step(x, u, dt);
  }

  const double t_end = dt * static_cast<double>(n_steps);
  const Eigen::Vector3d v_end = x.segment<3>(3);
  const double z_end = x(2);
  const double E_end = 0.5 * v_end.squaredNorm() + g * z_end;

  EXPECT_NEAR(E_end, E0, 1e-9);
  // Exact ballistic trajectory: v(t) = v0 + g t, z(t) = z0 + v0z t - 0.5 g t^2.
  EXPECT_NEAR(v_end.x(), 1.0, 1e-9);
  EXPECT_NEAR(v_end.y(), 0.0, 1e-9);
  EXPECT_NEAR(v_end.z(), -g * t_end, 1e-9);
  EXPECT_NEAR(z_end, z0 - 0.5 * g * t_end * t_end, 1e-9);
  // Quaternion stays the identity (zero rates, no torque).
  EXPECT_NEAR(x(6), 1.0, 1e-12);
  EXPECT_NEAR(x.segment<3>(7).norm(), 0.0, 1e-12);
}

TEST(DynamicsStep, QuaternionNormPreserved)
{
  const QuadrotorParams params = QuadrotorParams::fromYaml(x500Path());
  QuadrotorDynamics<double, AttitudeRep::Quaternion> dyn(params);

  std::mt19937 rng(303);
  std::uniform_real_distribution<double> dist(-1.0, 1.0);
  std::uniform_real_distribution<double> thrust(0.5, 5.0);

  for (int trial = 0; trial < 20; ++trial) {
    auto x = randomState<AttitudeRep::Quaternion>(rng);
    Eigen::Matrix<double, 4, 1> u;
    for (int i = 0; i < 4; ++i) {
      u(i) = thrust(rng);
    }
    for (int k = 0; k < 50; ++k) {
      x = dyn.step(x, u, 0.01);
    }
    EXPECT_NEAR(x.segment<4>(6).norm(), 1.0, 1e-12);
  }
}

// -----------------------------------------------------------------------------------------------
// Params, allocation, helpers
// -----------------------------------------------------------------------------------------------

TEST(DynamicsParams, FromYamlLoadsX500)
{
  const QuadrotorParams p = QuadrotorParams::fromYaml(x500Path());
  EXPECT_EQ(p.frame_name, "x500_v2");
  EXPECT_DOUBLE_EQ(p.mass, 2.0);
  EXPECT_DOUBLE_EQ(p.arm_length, 0.25);
  EXPECT_DOUBLE_EQ(p.thrust_coeff, 8.54858e-06);
  EXPECT_DOUBLE_EQ(p.torque_coeff, 1.3677728e-07);
  EXPECT_DOUBLE_EQ(p.max_thrust_per_rotor, 8.55);
  EXPECT_DOUBLE_EQ(p.min_thrust_per_rotor, 0.5);
  EXPECT_DOUBLE_EQ(p.gravity, 9.80665);
  EXPECT_TRUE(p.isValid());
}

TEST(DynamicsParams, MissingKeyThrowsNamedMessage)
{
  const std::string tmp = std::string(testing::TempDir()) + "/uav_mpc_missing_key.yaml";
  {
    std::ofstream f(tmp);
    f << "airframe:\n  name: \"x\"\n";
  }
  EXPECT_THROW(
    {
      try {
        QuadrotorParams::fromYaml(tmp);
      } catch (const std::runtime_error & e) {
        const std::string msg = e.what();
        EXPECT_NE(msg.find("airframe.mass"), std::string::npos) << msg;
        EXPECT_NE(msg.find(tmp), std::string::npos) << msg;
        throw;
      }
    }, std::runtime_error);
}

TEST(DynamicsParams, InvalidParamsRejected)
{
  QuadrotorParams p = QuadrotorParams::fromYaml(x500Path());
  p.mass = -1.0;
  std::string why;
  EXPECT_FALSE(p.isValid(&why));
  EXPECT_FALSE(why.empty());

  p = QuadrotorParams::fromYaml(x500Path());
  p.inertia(0, 0) = -p.inertia(0, 0);  // not positive definite
  EXPECT_FALSE(p.isValid(&why));

  p = QuadrotorParams::fromYaml(x500Path());
  p.max_thrust_per_rotor = p.mass * p.gravity / 8.0;  // thrust-to-weight <= 1
  EXPECT_FALSE(p.isValid(&why));
}

TEST(DynamicsAllocation, RoundTripWithinLimits)
{
  const QuadrotorParams params = QuadrotorParams::fromYaml(x500Path());
  QuadrotorDynamics<double, AttitudeRep::Quaternion> dyn(params);

  Eigen::Matrix<double, 4, 1> u;
  u << 1.0, 2.0, 3.0, 4.0;  // all inside [0.5, 8.55]
  double T = 0.0;
  Eigen::Matrix<double, 3, 1> tau;
  dyn.allocate(u, &T, &tau);

  Eigen::Matrix<double, 4, 1> u2;
  const bool feasible = dyn.allocateInverse(T, tau, &u2);
  EXPECT_TRUE(feasible);
  EXPECT_LT((u2 - u).norm(), 1e-12);
}

TEST(DynamicsAllocation, HoverThrust)
{
  const QuadrotorParams params = QuadrotorParams::fromYaml(x500Path());
  QuadrotorDynamics<double, AttitudeRep::Quaternion> dyn(params);
  EXPECT_NEAR(dyn.hoverThrustPerRotor(), params.mass * params.gravity / 4.0, 1e-12);
  // Hover inputs map to zero torque and exactly weight.
  Eigen::Matrix<double, 4, 1> u = Eigen::Matrix<double, 4, 1>::Constant(
    dyn.hoverThrustPerRotor());
  double T = 0.0;
  Eigen::Matrix<double, 3, 1> tau;
  dyn.allocate(u, &T, &tau);
  EXPECT_NEAR(T, params.mass * params.gravity, 1e-12);
  EXPECT_LT(tau.norm(), 1e-12);
}

TEST(DynamicsRotation, RotationMatrixOrthonormalAndConsistent)
{
  const QuadrotorParams params = QuadrotorParams::fromYaml(x500Path());
  QuadrotorDynamics<double, AttitudeRep::Quaternion> dyn_q(params);
  QuadrotorDynamics<double, AttitudeRep::Euler> dyn_e(params);

  std::mt19937 rng(404);
  const auto xq = randomState<AttitudeRep::Quaternion>(rng);
  const Eigen::Matrix3d Rq = dyn_q.rotationMatrix(xq);
  EXPECT_LT((Rq.transpose() * Rq - Eigen::Matrix3d::Identity()).norm(), 1e-12);
  EXPECT_NEAR(Rq.determinant(), 1.0, 1e-12);

  // Quaternion and Euler rotations agree for the same physical attitude.
  const Eigen::Quaterniond q(xq(6), xq(7), xq(8), xq(9));
  const auto rpy = quatToEulerZyx(q);
  using EulerDyn = QuadrotorDynamics<double, AttitudeRep::Euler>;
  using EulerState = EulerDyn::StateVector;
  EulerState xe = EulerState::Zero();
  xe.segment<3>(6) = rpy;
  EXPECT_LT((dyn_e.rotationMatrix(xe) - Rq).norm(), 1e-12);
}

TEST(DynamicsHelpers, EulerQuaternionRoundTrip)
{
  std::mt19937 rng(505);
  std::uniform_real_distribution<double> ang(-0.6, 0.6);
  for (int i = 0; i < 100; ++i) {
    const Eigen::Vector3d rpy(ang(rng), ang(rng), ang(rng));
    const Eigen::Quaterniond q = eulerZyxToQuat(rpy);
    const Eigen::Vector3d rpy2 = quatToEulerZyx(q);
    EXPECT_LT((rpy2 - rpy).norm(), 1e-12);
  }
}

}  // namespace uav_mpc
