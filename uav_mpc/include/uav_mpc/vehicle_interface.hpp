// Copyright (c) 2026 Ali-Eimaan.
// SPDX-License-Identifier: BSD-3-Clause
//
// Autopilot abstraction. See .deepseek/07_NODE.md §7.10.
//
// WHY THIS EXISTS: px4_msgs is not released for ROS 2 Lyrical Luth, so the controller must not
// depend on it. Everything autopilot-specific lives behind this interface:
//
//   * GenericVehicleInterface — standard ROS 2 messages only (nav_msgs/Odometry in,
//     uav_mpc/AttitudeThrustSetpoint out). Always available. This is the DEFAULT.
//   * Px4VehicleInterface     — px4_msgs/uXRCE-DDS. Compiled ONLY when px4_msgs is present
//     (CMake option UAV_MPC_WITH_PX4_MSGS, auto-detected). The implementation is complete and
//     preserved; it is simply not built when the package is missing.
//
// The backend is chosen at RUNTIME by the `vehicle_interface` parameter ("generic" | "px4").
// Requesting "px4" from a build without px4_msgs fails on_configure with an actionable message
// rather than silently degrading.
//
// NOTHING in this header may include a px4_msgs header — that is the whole point. The PX4
// types are confined to src/vehicle_interface_px4.cpp behind the PIMPL.

#ifndef UAV_MPC__VEHICLE_INTERFACE_HPP_
#define UAV_MPC__VEHICLE_INTERFACE_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace uav_mpc
{

/// One odometry sample, already converted into the controller's conventions:
/// world ENU, body FLU, Hamilton quaternion. Backends own their frame conversions.
struct VehicleOdometry
{
  Eigen::Vector3d position_enu{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_enu{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond attitude_enu_flu{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d body_rates_flu{Eigen::Vector3d::Zero()};

  /// Which fields this sample actually carries. A backend that delivers pose and twist in one
  /// message sets all four; PX4 delivers them on three separate topics and sets them
  /// independently.
  bool has_position{false};
  bool has_velocity{false};
  bool has_attitude{false};
  bool has_rates{false};
};

/// Arming / control-authority state reported by the autopilot.
struct VehicleStatusFlags
{
  bool armed{false};
  bool offboard_active{false};
};

/// The controller's output, frame-neutral. Mirrors msg/AttitudeThrustSetpoint.msg.
struct AttitudeThrustCommand
{
  Eigen::Quaterniond attitude_enu_flu{Eigen::Quaterniond::Identity()};
  double collective_thrust_newton{0.0};
  double normalised_thrust{0.0};      ///< [0, 1]
  double yaw_rate_feedforward{0.0};   ///< [rad/s]
  Eigen::Vector4d rotor_thrust_newton{Eigen::Vector4d::Zero()};
};

/// Abstract autopilot backend. All methods are called from the node; `publishSetpoint` and
/// `publishControlMode` run inside the 100 Hz control loop and must not allocate or block.
class VehicleInterface
{
public:
  using OdometryCallback = std::function<void (const VehicleOdometry &)>;
  using StatusCallback = std::function<void (const VehicleStatusFlags &)>;

  virtual ~VehicleInterface() = default;

  /// Backend id, for logging: "generic" or "px4".
  virtual std::string name() const = 0;

  /// Create subscriptions and publishers on `node`. Returns false with `*error` filled rather
  /// than throwing, so the lifecycle node can report FAILURE from on_configure().
  virtual bool setup(
    rclcpp_lifecycle::LifecycleNode * node,
    rclcpp::CallbackGroup::SharedPtr callback_group,
    std::string * error) = 0;

  virtual void activate() = 0;
  virtual void deactivate() = 0;
  virtual void teardown() = 0;

  void setOdometryCallback(OdometryCallback cb) {odometry_cb_ = std::move(cb);}
  void setStatusCallback(StatusCallback cb) {status_cb_ = std::move(cb);}

  /// Send one attitude+thrust command to the vehicle.
  virtual void publishSetpoint(const AttitudeThrustCommand & cmd) = 0;

  /// Heartbeat the autopilot needs to accept offboard commands. PX4 requires this at >= 2 Hz
  /// before AND during offboard; the generic backend has no such concept and no-ops.
  virtual void publishControlMode() {}

  /// Mode / arming requests. Only meaningful for backends with an autopilot handshake.
  virtual void requestOffboard() {}
  virtual void requestArm() {}
  virtual void requestDisarm() {}

  /// True when the backend has a real arm/offboard handshake. When false the node treats the
  /// vehicle as always-authorised, because there is no autopilot to ask — see §7.10.
  virtual bool hasAutopilotHandshake() const {return false;}

  /// Number of control-mode heartbeats published so far (PX4 needs a warm stream before it
  /// will accept the offboard mode switch).
  virtual int controlModeStreamCount() const {return 0;}

protected:
  /// Backends call these to hand data to the node.
  void emitOdometry(const VehicleOdometry & odom) const {if (odometry_cb_) {odometry_cb_(odom);}}
  void emitStatus(const VehicleStatusFlags & status) const {if (status_cb_) {status_cb_(status);}}

private:
  OdometryCallback odometry_cb_;
  StatusCallback status_cb_;
};

/// True when this build contains the PX4 backend (i.e. px4_msgs was found at configure time).
bool px4InterfaceAvailable();

/// Construct a backend by name. `kind` is "generic" or "px4" (case-insensitive).
/// Returns nullptr with `*error` filled when the name is unknown, or when "px4" is requested
/// from a build that does not contain it.
std::unique_ptr<VehicleInterface> makeVehicleInterface(
  const std::string & kind, std::string * error);

}  // namespace uav_mpc

#endif  // UAV_MPC__VEHICLE_INTERFACE_HPP_
