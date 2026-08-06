// Copyright (c) 2026 Ali-Eimaan.
// SPDX-License-Identifier: BSD-3-Clause
//
// Node-level regression tests for REVIEW R1-3, R1-4 and R1-12:
//   - the failsafe collective thrust (4 * per-rotor hover) maps to exactly `px4_hover_thrust`
//   - the takeoff/landing reference horizon commands a LEVEL attitude — identity quaternion
//     in slot order (w, x, y, z) — for every stage, with yaw equal to the vehicle's yaw
//   - the Landing -> Idle transition clears the `landing_requested_` latch, so a second
//     mission (Streaming -> Takeoff -> Tracking) persists instead of re-descending
//
// The fixture is a friend of NmpcNode (see nmpc_node.hpp) so it can drive the private state
// machine and helpers directly: no executor, no backend, no acados. `rclcpp::init` is still
// required to construct a node, hence the local main().

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "uav_mpc/nmpc_node.hpp"
#include "uav_mpc/quadrotor_dynamics.hpp"

#ifndef UAV_MPC_SOURCE_DIR
#error "UAV_MPC_SOURCE_DIR must be defined (set in CMakeLists.txt)"
#endif

namespace uav_mpc
{

namespace
{

std::string x500Path()
{
  return std::string(UAV_MPC_SOURCE_DIR) + "/params/x500_calibration.yaml";
}

}  // namespace

/// Friend fixture (declared in nmpc_node.hpp). Each test gets a fresh, unconfigured node whose
/// members the tests need (airframe, model, thrust map, reference buffers) are set up in SetUp.
class NmpcNodeTest : public ::testing::Test
{
protected:
  NmpcNodeTest() : node_(rclcpp::NodeOptions()) {}

  void SetUp() override
  {
    node_.airframe_ = QuadrotorParams::fromYaml(x500Path());
    node_.px4_hover_thrust_ = 0.62;   // the node's default from config/nmpc_params.yaml
    node_.model_ = std::make_unique<QuadrotorDynamics<double, AttitudeRep::Quaternion>>(
      node_.airframe_);
    using Layout = StateLayout<AttitudeRep::Quaternion>;
    node_.x_refs_.assign(21, Eigen::VectorXd::Zero(Layout::kNx));
    node_.u_refs_.assign(20, Eigen::VectorXd::Zero(4));
  }

  NmpcNode node_;

  // The test bodies live HERE (not in TEST_F): gtest's TEST_F generates a class DERIVED from
  // the fixture, and friendship is not inherited — only members of NmpcNodeTest itself can
  // touch NmpcNode's private members.

  // R1-3: enterFailsafe() publishes a COLLECTIVE thrust of 4 * (m g / 4) = m g — full hover
  // weight — and normaliseThrust() must map that back to exactly `px4_hover_thrust_` (linear
  // map, THR_MDL_FAC == 0). Before R1-3 the per-rotor value was passed in directly, which
  // normalised to px4_hover_thrust_ / 4: the vehicle would have dropped out of the sky.
  void failsafeCollectiveThrustMapsToPx4HoverThrust()
  {
    const double per_rotor = node_.model_->hoverThrustPerRotor();
    EXPECT_NEAR(per_rotor, node_.airframe_.mass * node_.airframe_.gravity / 4.0, 1e-12);

    const double collective = 4.0 * per_rotor;   // what enterFailsafe() publishes
    const double u = node_.normaliseThrust(collective);

    EXPECT_NEAR(u, node_.px4_hover_thrust_, 1e-9);
    // Sanity: the pre-fix mistake (per-rotor passed to the collective map) gives 1/4 of the
    // commanded thrust — exactly the failure R1-3 fixed.
    EXPECT_NEAR(node_.normaliseThrust(per_rotor), node_.px4_hover_thrust_ / 4.0, 1e-9);
  }

  // R1-4: fillTakeoffLandingHorizon() writes the reference quaternion in (w, x, y, z) order,
  // so the identity puts 1.0 in the FIRST slot. Before R1-4 the code wrote kAttIdx + 3
  // (qz = 1), i.e. a 180-degree yaw reference — the vehicle would have spun around on takeoff.
  // The vehicle state here is level (yaw 0), so "identity" and "yaw equals the vehicle's yaw"
  // coincide and both assertions are meaningful.
  void takeoffLandingHorizonCommandsLevelAttitude()
  {
    using Layout = StateLayout<AttitudeRep::Quaternion>;
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(Layout::kNx);
    x0(Layout::kPosIdx) = 0.5;
    x0(Layout::kPosIdx + 1) = -0.25;
    x0(Layout::kPosIdx + 2) = 0.3;
    x0(Layout::kAttIdx) = 1.0;   // level vehicle, yaw 0

    const double dt = 0.05;
    const int n = 20;
    const double hover = node_.airframe_.mass * node_.airframe_.gravity / 4.0;

    // --- takeoff branch --------------------------------------------------------------------
    node_.fillTakeoffLandingHorizon(ControllerState::Takeoff, x0, dt, n, hover);
    for (int k = 0; k <= n; ++k) {
      const Eigen::VectorXd & xr = node_.x_refs_[static_cast<std::size_t>(k)];
      const Eigen::Quaterniond q_ref(xr(Layout::kAttIdx), xr(Layout::kAttIdx + 1),
        xr(Layout::kAttIdx + 2), xr(Layout::kAttIdx + 3));
      EXPECT_NEAR(q_ref.w(), 1.0, 1e-12) << "takeoff stage " << k;
      EXPECT_NEAR(q_ref.x(), 0.0, 1e-12) << "takeoff stage " << k;
      EXPECT_NEAR(q_ref.y(), 0.0, 1e-12) << "takeoff stage " << k;
      EXPECT_NEAR(q_ref.z(), 0.0, 1e-12) << "takeoff stage " << k;
      const double ref_yaw = std::atan2(2.0 * (q_ref.w() * q_ref.z()),
        1.0 - 2.0 * q_ref.z() * q_ref.z());
      EXPECT_NEAR(ref_yaw, 0.0, 1e-12) << "takeoff stage " << k;   // == vehicle yaw (level)
      // The ramp keeps the current xy and only moves z upward.
      EXPECT_NEAR(xr(Layout::kPosIdx), 0.5, 1e-12) << "takeoff stage " << k;
      EXPECT_NEAR(xr(Layout::kPosIdx + 1), -0.25, 1e-12) << "takeoff stage " << k;
      EXPECT_GE(xr(Layout::kPosIdx + 2), 0.3 - 1e-12) << "takeoff stage " << k;
    }
    for (int k = 0; k < n; ++k) {
      EXPECT_NEAR(node_.u_refs_[static_cast<std::size_t>(k)](0), hover, 1e-12)
        << "takeoff stage " << k;
    }

    // --- landing branch --------------------------------------------------------------------
    node_.fillTakeoffLandingHorizon(ControllerState::Landing, x0, dt, n, hover);
    for (int k = 0; k <= n; ++k) {
      const Eigen::VectorXd & xr = node_.x_refs_[static_cast<std::size_t>(k)];
      const Eigen::Quaterniond q_ref(xr(Layout::kAttIdx), xr(Layout::kAttIdx + 1),
        xr(Layout::kAttIdx + 2), xr(Layout::kAttIdx + 3));
      EXPECT_NEAR(q_ref.w(), 1.0, 1e-12) << "landing stage " << k;
      EXPECT_NEAR(q_ref.x(), 0.0, 1e-12) << "landing stage " << k;
      EXPECT_NEAR(q_ref.y(), 0.0, 1e-12) << "landing stage " << k;
      EXPECT_NEAR(q_ref.z(), 0.0, 1e-12) << "landing stage " << k;
      EXPECT_LE(xr(Layout::kPosIdx + 2), 0.3 + 1e-12) << "landing stage " << k;  // descends
    }
  }

  // R1-12: Sequence Streaming -> Takeoff -> Tracking -> Landing -> Idle, then a second mission
  // (re-activation, exactly what on_activate() resets) -> Takeoff -> Tracking. Before R1-12 the
  // latch stayed true after landing, so the second Tracking instantly re-entered Landing and the
  // vehicle would have descended again instead of flying the new mission.
  void landingClearsRequestedLatchAndSecondMissionPersists()
  {
    // --- first mission: stream -> takeoff -> track -----------------------------------------
    node_.controller_state_.store(ControllerState::Streaming);
    node_.offboard_active_.store(true);
    node_.armed_.store(true);
    node_.updateControllerState();
    ASSERT_EQ(node_.controller_state_.load(), ControllerState::Takeoff);

    {
      std::lock_guard<std::mutex> lk(node_.state_mutex_);
      node_.position_enu_.z() = node_.takeoff_altitude_m_;
      node_.velocity_enu_.setZero();
    }
    node_.updateControllerState();
    ASSERT_EQ(node_.controller_state_.load(), ControllerState::Tracking);

    // --- request landing, descend, land -----------------------------------------------------
    node_.landing_requested_.store(true);
    node_.updateControllerState();
    ASSERT_EQ(node_.controller_state_.load(), ControllerState::Landing);
    EXPECT_TRUE(node_.landing_requested_.load());   // still latched while descending

    {
      std::lock_guard<std::mutex> lk(node_.state_mutex_);
      node_.position_enu_.z() = 0.0;   // below kLandingZThresholdM (0.15 m)
      node_.velocity_enu_.setZero();
    }
    node_.updateControllerState();
    ASSERT_EQ(node_.controller_state_.load(), ControllerState::Idle);
    EXPECT_FALSE(node_.landing_requested_.load());   // REVIEW R1-12: latch cleared

    // --- second mission: re-activation resets state exactly like on_activate() --------------
    node_.controller_state_.store(ControllerState::Streaming);
    node_.offboard_active_.store(true);
    node_.armed_.store(true);
    node_.updateControllerState();
    ASSERT_EQ(node_.controller_state_.load(), ControllerState::Takeoff);

    {
      std::lock_guard<std::mutex> lk(node_.state_mutex_);
      node_.position_enu_.z() = node_.takeoff_altitude_m_;
      node_.velocity_enu_.setZero();
    }
    node_.updateControllerState();
    ASSERT_EQ(node_.controller_state_.load(), ControllerState::Tracking);

    // A stale landing_requested_ would flip Tracking -> Landing on the very next tick; the
    // second mission must persist for at least 100 ticks.
    for (int i = 0; i < 100; ++i) {
      node_.updateControllerState();
      ASSERT_EQ(node_.controller_state_.load(), ControllerState::Tracking) << "tick " << i;
    }
    EXPECT_FALSE(node_.landing_requested_.load());
  }
};

// -----------------------------------------------------------------------------------------------
// Test entry points — thin wrappers; the logic lives in the fixture methods above because
// friendship does not extend to gtest's generated derived classes.
// -----------------------------------------------------------------------------------------------

TEST_F(NmpcNodeTest, FailsafeCollectiveThrustMapsToPx4HoverThrust)
{
  failsafeCollectiveThrustMapsToPx4HoverThrust();
}

TEST_F(NmpcNodeTest, TakeoffLandingHorizonCommandsLevelAttitude)
{
  takeoffLandingHorizonCommandsLevelAttitude();
}

TEST_F(NmpcNodeTest, LandingClearsRequestedLatchAndSecondMissionPersists)
{
  landingClearsRequestedLatchAndSecondMissionPersists();
}

}  // namespace uav_mpc

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
