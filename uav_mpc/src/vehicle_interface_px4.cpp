// Copyright (c) 2026 Ali-Eimaan.
// SPDX-License-Identifier: BSD-3-Clause
//
// PX4 / uXRCE-DDS vehicle backend. See .deepseek/07_NODE.md §7.10-§7.11.
//
// ============================================================================================
// THIS FILE IS COMPILED ONLY WHEN px4_msgs IS AVAILABLE.
//
// px4_msgs is not released for ROS 2 Lyrical Luth at the time of writing, so `uav_mpc` does not
// depend on it. CMake probes for it with find_package(px4_msgs QUIET); when it is found the
// UAV_MPC_WITH_PX4_MSGS option turns on and this translation unit joins the build. When it is
// absent, everything else still builds and the generic backend is used instead.
//
// The implementation below is COMPLETE and is deliberately retained — do not delete it to
// "clean up" the px4_msgs removal. To enable it:
//     git clone https://github.com/PX4/px4_msgs.git <ros2_ws>/src/px4_msgs
//     cd <ros2_ws> && colcon build
// then run with `vehicle_interface:=px4`.
//
// VERIFICATION STATUS: UNVERIFIED. This code has never been compiled — px4_msgs is not
// installed here. Every field name and constant below must be checked against the message
// definitions in the pinned px4_msgs release before it is trusted. `VehicleAttitudeSetpoint`
// in particular changed shape across PX4 releases (roll_body/pitch_body/yaw_body removed).
// ============================================================================================

#include "uav_mpc/vehicle_interface.hpp"

#ifdef UAV_MPC_WITH_PX4_MSGS

#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_angular_velocity.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/vehicle_attitude_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"

#include "uav_mpc/quadrotor_dynamics.hpp"   // frame conversions live here, and only here

namespace uav_mpc
{

namespace
{

/// PX4 QoS: BestEffort + Volatile, KeepLast(5). A mismatched profile silently receives
/// nothing — this is the single most likely first failure in SITL (§16).
rclcpp::QoS px4Qos()
{
  rclcpp::QoS qos(rclcpp::KeepLast(5));
  qos.best_effort().durability_volatile();
  return qos;
}

/// PX4 timestamps are MICROSECONDS. Nanoseconds here makes PX4 silently drop the setpoint.
std::uint64_t nowUs(rclcpp_lifecycle::LifecycleNode * node)
{
  return static_cast<std::uint64_t>(node->get_clock()->now().nanoseconds() / 1000);
}

constexpr int kOffboardPreStreamCount = 20;  ///< PX4 wants setpoints BEFORE accepting offboard

}  // namespace

class Px4VehicleInterface : public VehicleInterface
{
public:
  std::string name() const override {return "px4";}

  bool setup(
    rclcpp_lifecycle::LifecycleNode * node,
    rclcpp::CallbackGroup::SharedPtr callback_group,
    std::string * error) override
  {
    if (node == nullptr) {
      if (error != nullptr) {*error = "Px4VehicleInterface::setup: node is null";}
      return false;
    }
    node_ = node;

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = std::move(callback_group);

    sub_local_position_ = node_->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", px4Qos(),
      [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr m) {onLocalPosition(m);},
      sub_options);
    sub_attitude_ = node_->create_subscription<px4_msgs::msg::VehicleAttitude>(
      "/fmu/out/vehicle_attitude", px4Qos(),
      [this](const px4_msgs::msg::VehicleAttitude::SharedPtr m) {onAttitude(m);},
      sub_options);
    sub_angular_velocity_ = node_->create_subscription<px4_msgs::msg::VehicleAngularVelocity>(
      "/fmu/out/vehicle_angular_velocity", px4Qos(),
      [this](const px4_msgs::msg::VehicleAngularVelocity::SharedPtr m) {onAngularVelocity(m);},
      sub_options);
    sub_vehicle_status_ = node_->create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status", px4Qos(),
      [this](const px4_msgs::msg::VehicleStatus::SharedPtr m) {onVehicleStatus(m);},
      sub_options);

    pub_attitude_setpoint_ = node_->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
      "/fmu/in/vehicle_attitude_setpoint", px4Qos());
    pub_offboard_mode_ = node_->create_publisher<px4_msgs::msg::OffboardControlMode>(
      "/fmu/in/offboard_control_mode", px4Qos());
    pub_command_ = node_->create_publisher<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", px4Qos());

    RCLCPP_INFO(node_->get_logger(), "px4 vehicle interface: uXRCE-DDS topics under /fmu/*");
    return true;
  }

  void activate() override
  {
    if (pub_attitude_setpoint_) {pub_attitude_setpoint_->on_activate();}
    if (pub_offboard_mode_) {pub_offboard_mode_->on_activate();}
    if (pub_command_) {pub_command_->on_activate();}
    stream_count_ = 0;
  }

  void deactivate() override
  {
    if (pub_attitude_setpoint_) {pub_attitude_setpoint_->on_deactivate();}
    if (pub_offboard_mode_) {pub_offboard_mode_->on_deactivate();}
    if (pub_command_) {pub_command_->on_deactivate();}
  }

  void teardown() override
  {
    sub_local_position_.reset();
    sub_attitude_.reset();
    sub_angular_velocity_.reset();
    sub_vehicle_status_.reset();
    pub_attitude_setpoint_.reset();
    pub_offboard_mode_.reset();
    pub_command_.reset();
    node_ = nullptr;
  }

  // --- outputs ----------------------------------------------------------------------------

  void publishSetpoint(const AttitudeThrustCommand & cmd) override
  {
    if (!pub_attitude_setpoint_ || node_ == nullptr) {return;}
    px4_msgs::msg::VehicleAttitudeSetpoint msg{};

    const Eigen::Quaterniond q_ned_frd = quatEnuFluToNedFrd(cmd.attitude_enu_flu);
    msg.q_d[0] = static_cast<float>(q_ned_frd.w());
    msg.q_d[1] = static_cast<float>(q_ned_frd.x());
    msg.q_d[2] = static_cast<float>(q_ned_frd.y());
    msg.q_d[3] = static_cast<float>(q_ned_frd.z());

    // PX4 is FRD: positive thrust acts on NEGATIVE body z.
    msg.thrust_body[0] = 0.0f;
    msg.thrust_body[1] = 0.0f;
    msg.thrust_body[2] = static_cast<float>(-cmd.normalised_thrust);

    // ENU/FLU yaw rate and NED/FRD yaw rate differ in sign (the z axis flips).
    msg.yaw_sp_move_rate = static_cast<float>(-cmd.yaw_rate_feedforward);

    msg.timestamp = nowUs(node_);
    pub_attitude_setpoint_->publish(msg);
  }

  /// Only the `attitude` channel is requested: the NMPC outputs attitude + normalised thrust
  /// and PX4's attitude/rate loops close the inner loop.
  void publishControlMode() override
  {
    if (!pub_offboard_mode_ || node_ == nullptr) {return;}
    px4_msgs::msg::OffboardControlMode msg{};
    msg.timestamp = nowUs(node_);
    msg.position = false;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = true;
    msg.body_rate = false;
    ++stream_count_;
    pub_offboard_mode_->publish(msg);
  }

  void requestOffboard() override
  {
    // PX4 needs a warm setpoint stream before it will accept the mode switch.
    if (stream_count_ < kOffboardPreStreamCount) {return;}
    if (offboard_active_) {return;}
    auto cmd = baseCommand();
    cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
    cmd.param1 = 1.0f;   // custom mode enabled
    cmd.param2 = 6.0f;   // PX4 custom main mode: OFFBOARD
    pub_command_->publish(cmd);
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
      "requesting OFFBOARD mode");
  }

  void requestArm() override
  {
    if (armed_) {return;}
    auto cmd = baseCommand();
    cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    cmd.param1 = 1.0f;
    pub_command_->publish(cmd);
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "requesting ARM");
  }

  void requestDisarm() override
  {
    auto cmd = baseCommand();
    cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    cmd.param1 = 0.0f;
    pub_command_->publish(cmd);
    RCLCPP_INFO(node_->get_logger(), "requesting DISARM");
  }

  bool hasAutopilotHandshake() const override {return true;}
  int controlModeStreamCount() const override {return stream_count_;}

private:
  /// VehicleCommand requires target/source system+component == 1 and from_external == true,
  /// or PX4 ignores the command outright (§7.5).
  px4_msgs::msg::VehicleCommand baseCommand() const
  {
    px4_msgs::msg::VehicleCommand cmd{};
    cmd.timestamp = nowUs(node_);
    cmd.target_system = 1;
    cmd.target_component = 1;
    cmd.source_system = 1;
    cmd.source_component = 1;
    cmd.from_external = true;
    return cmd;
  }

  // --- inputs -----------------------------------------------------------------------------

  /// VehicleLocalPosition is NED. The *_valid flags gate everything: PX4 publishes the message
  /// continuously, with some estimates invalid during startup.
  void onLocalPosition(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    if (!msg->xy_valid || !msg->z_valid || !msg->v_xy_valid || !msg->v_z_valid) {
      return;   // incomplete estimate — the node's staleness check will catch a sustained gap
    }
    VehicleOdometry odom;
    odom.position_enu = nedToEnu(Eigen::Vector3d(msg->x, msg->y, msg->z));
    odom.velocity_enu = nedToEnu(Eigen::Vector3d(msg->vx, msg->vy, msg->vz));
    odom.has_position = true;
    odom.has_velocity = true;
    emitOdometry(odom);
  }

  /// VehicleAttitude::q is (w, x, y, z) in NED/FRD.
  void onAttitude(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
  {
    VehicleOdometry odom;
    const Eigen::Quaterniond q_ned_frd(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
    odom.attitude_enu_flu = quatNedFrdToEnuFlu(q_ned_frd);
    odom.has_attitude = true;
    emitOdometry(odom);
  }

  /// FRD -> FLU flips the lateral axes: (x, -y, -z).
  void onAngularVelocity(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr msg)
  {
    VehicleOdometry odom;
    odom.body_rates_flu = Eigen::Vector3d(msg->xyz[0], -msg->xyz[1], -msg->xyz[2]);
    odom.has_rates = true;
    emitOdometry(odom);
  }

  /// Constants live on the message (ARMING_STATE_ARMED, NAVIGATION_STATE_OFFBOARD); verify
  /// against the pinned px4_msgs version — these names have moved between releases.
  void onVehicleStatus(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
  {
    armed_ = (msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
    offboard_active_ =
      (msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD);
    VehicleStatusFlags flags;
    flags.armed = armed_;
    flags.offboard_active = offboard_active_;
    emitStatus(flags);
  }

  rclcpp_lifecycle::LifecycleNode * node_{nullptr};

  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_local_position_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr sub_attitude_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAngularVelocity>::SharedPtr sub_angular_velocity_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr sub_vehicle_status_;

  rclcpp_lifecycle::LifecyclePublisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr
    pub_attitude_setpoint_;
  rclcpp_lifecycle::LifecyclePublisher<px4_msgs::msg::OffboardControlMode>::SharedPtr
    pub_offboard_mode_;
  rclcpp_lifecycle::LifecyclePublisher<px4_msgs::msg::VehicleCommand>::SharedPtr pub_command_;

  std::atomic<bool> armed_{false};
  std::atomic<bool> offboard_active_{false};
  std::atomic<int> stream_count_{0};
};

std::unique_ptr<VehicleInterface> makePx4VehicleInterface()
{
  return std::make_unique<Px4VehicleInterface>();
}

}  // namespace uav_mpc

#endif  // UAV_MPC_WITH_PX4_MSGS
