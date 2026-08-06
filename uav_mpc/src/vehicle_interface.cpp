// Copyright (c) 2026 Ali-Eimaan.
// SPDX-License-Identifier: BSD-3-Clause
//
// Generic (autopilot-agnostic) vehicle backend + the backend factory.
// See .deepseek/07_NODE.md §7.10.
//
// This translation unit uses STANDARD ROS 2 messages only. It builds on any distro, with or
// without px4_msgs, and is the default backend.

#include "uav_mpc/vehicle_interface.hpp"

#include <algorithm>
#include <cctype>
#include <memory>
#include <string>
#include <utility>

#include "nav_msgs/msg/odometry.hpp"
#include "uav_mpc/msg/attitude_thrust_setpoint.hpp"

namespace uav_mpc
{

namespace
{

std::string toLower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
  return s;
}

}  // namespace

// ================================================================================================
// GenericVehicleInterface
// ================================================================================================

/// Autopilot-agnostic backend.
///
///   IN : ~/odometry            nav_msgs/msg/Odometry
///        Pose in the world frame, twist in the BODY frame — this is what REP-105 and the
///        nav_msgs/Odometry documentation specify, and what a mocap bridge or an EKF publishes.
///        We take the world frame to be ENU and the body frame FLU (REP-103), so no conversion
///        is needed: this backend is a pass-through by construction.
///
///   OUT: ~/attitude_setpoint   uav_mpc/msg/AttitudeThrustSetpoint  (ENU/FLU)
///
/// There is no arming or mode handshake here — there is no autopilot to ask. The backend
/// therefore reports `armed = offboard_active = true` as soon as it is activated, and
/// `hasAutopilotHandshake()` returns false so the node knows the flags are nominal rather than
/// measured. Whatever consumes ~/attitude_setpoint owns the actual safety interlocks.
class GenericVehicleInterface : public VehicleInterface
{
public:
  std::string name() const override {return "generic";}

  bool setup(
    rclcpp_lifecycle::LifecycleNode * node,
    rclcpp::CallbackGroup::SharedPtr callback_group,
    std::string * error) override
  {
    if (node == nullptr) {
      if (error != nullptr) {*error = "GenericVehicleInterface::setup: node is null";}
      return false;
    }
    node_ = node;

    // Reliable + volatile: a generic source (mocap bridge, EKF, sim) is a normal ROS 2
    // publisher, unlike PX4's BestEffort uXRCE-DDS bridge.
    rclcpp::QoS qos(rclcpp::KeepLast(5));
    qos.reliable().durability_volatile();

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = std::move(callback_group);

    sub_odometry_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "~/odometry", qos,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {this->onOdometry(msg);},
      sub_options);

    pub_setpoint_ = node_->create_publisher<uav_mpc::msg::AttitudeThrustSetpoint>(
      "~/attitude_setpoint", qos);

    RCLCPP_INFO(
      node_->get_logger(),
      "generic vehicle interface: subscribing ~/odometry (nav_msgs/Odometry, world ENU, "
      "body-frame twist), publishing ~/attitude_setpoint (uav_mpc/AttitudeThrustSetpoint)");
    return true;
  }

  void activate() override
  {
    if (pub_setpoint_) {pub_setpoint_->on_activate();}
    // No autopilot to arm: report authority immediately so the state machine can progress.
    VehicleStatusFlags flags;
    flags.armed = true;
    flags.offboard_active = true;
    emitStatus(flags);
  }

  void deactivate() override
  {
    if (pub_setpoint_) {pub_setpoint_->on_deactivate();}
    VehicleStatusFlags flags;   // armed = offboard = false
    emitStatus(flags);
  }

  void teardown() override
  {
    sub_odometry_.reset();
    pub_setpoint_.reset();
    node_ = nullptr;
  }

  void publishSetpoint(const AttitudeThrustCommand & cmd) override
  {
    if (!pub_setpoint_ || node_ == nullptr) {return;}
    uav_mpc::msg::AttitudeThrustSetpoint msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = "map";
    msg.attitude.w = cmd.attitude_enu_flu.w();
    msg.attitude.x = cmd.attitude_enu_flu.x();
    msg.attitude.y = cmd.attitude_enu_flu.y();
    msg.attitude.z = cmd.attitude_enu_flu.z();
    msg.collective_thrust_newton = cmd.collective_thrust_newton;
    msg.normalised_thrust = cmd.normalised_thrust;
    msg.yaw_rate_feedforward = cmd.yaw_rate_feedforward;
    for (int i = 0; i < 4; ++i) {
      msg.rotor_thrust_newton[i] = cmd.rotor_thrust_newton(i);
    }
    pub_setpoint_->publish(msg);
  }

  bool hasAutopilotHandshake() const override {return false;}

private:
  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    VehicleOdometry odom;

    odom.position_enu = Eigen::Vector3d(
      msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    odom.attitude_enu_flu = Eigen::Quaterniond(
      msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    odom.attitude_enu_flu.normalize();

    // nav_msgs/Odometry twist is expressed in the CHILD (body) frame. The controller wants
    // world-frame velocity, so rotate it. Getting this wrong is invisible at hover and shows
    // up as a heading-dependent tracking error — see .deepseek/16_CONVENTIONS.md.
    const Eigen::Vector3d v_body(
      msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    odom.velocity_enu = odom.attitude_enu_flu * v_body;

    odom.body_rates_flu = Eigen::Vector3d(
      msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);

    odom.has_position = true;
    odom.has_velocity = true;
    odom.has_attitude = true;
    odom.has_rates = true;
    emitOdometry(odom);
  }

  rclcpp_lifecycle::LifecycleNode * node_{nullptr};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
  rclcpp_lifecycle::LifecyclePublisher<uav_mpc::msg::AttitudeThrustSetpoint>::SharedPtr
    pub_setpoint_;
};

// ================================================================================================
// Factory
// ================================================================================================

#ifdef UAV_MPC_WITH_PX4_MSGS
/// Defined in src/vehicle_interface_px4.cpp, which is only compiled when px4_msgs is found.
std::unique_ptr<VehicleInterface> makePx4VehicleInterface();
#endif

bool px4InterfaceAvailable()
{
#ifdef UAV_MPC_WITH_PX4_MSGS
  return true;
#else
  return false;
#endif
}

std::unique_ptr<VehicleInterface> makeVehicleInterface(
  const std::string & kind, std::string * error)
{
  const std::string k = toLower(kind);

  if (k == "generic") {
    return std::make_unique<GenericVehicleInterface>();
  }

  if (k == "px4") {
#ifdef UAV_MPC_WITH_PX4_MSGS
    return makePx4VehicleInterface();
#else
    if (error != nullptr) {
      *error =
        "vehicle_interface is 'px4' but this build has no PX4 backend: px4_msgs was not found "
        "at configure time. px4_msgs is not released for ROS 2 Lyrical Luth yet. Either "
        "(a) use vehicle_interface:='generic', which needs no px4_msgs, or "
        "(b) install px4_msgs into the workspace "
        "(git clone https://github.com/PX4/px4_msgs.git <ws>/src/px4_msgs) and rebuild — "
        "CMake picks it up automatically.";
    }
    return nullptr;
#endif
  }

  if (error != nullptr) {
    *error = "unknown vehicle_interface '" + kind + "' (expected 'generic' or 'px4')";
  }
  return nullptr;
}

}  // namespace uav_mpc
