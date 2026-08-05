// Copyright (c) 2026 Ali-Eimaan.
// SPDX-License-Identifier: BSD-3-Clause
//
// SKELETON — declarations only. See .deepseek/07_NODE.md §7.
//
// ROS 2 lifecycle node running the NMPC at 100 Hz against PX4 over uXRCE-DDS.
//
//   IN : /fmu/out/vehicle_local_position      (px4_msgs::msg::VehicleLocalPosition, NED)
//        /fmu/out/vehicle_attitude            (px4_msgs::msg::VehicleAttitude, NED/FRD)
//        /fmu/out/vehicle_angular_velocity    (px4_msgs::msg::VehicleAngularVelocity, FRD)
//        /fmu/out/vehicle_status              (px4_msgs::msg::VehicleStatus)
//   OUT: /fmu/in/offboard_control_mode        (px4_msgs::msg::OffboardControlMode)  @ 100 Hz
//        /fmu/in/vehicle_attitude_setpoint    (px4_msgs::msg::VehicleAttitudeSetpoint)
//        /fmu/in/vehicle_command              (px4_msgs::msg::VehicleCommand)  arm / mode switch
//        ~/status                             (uav_mpc::msg::NmpcStatus)
//        ~/predicted_path                     (nav_msgs::msg::Path, ENU, for RViz)
//        ~/reference_path                     (nav_msgs::msg::Path, ENU, for RViz)

#ifndef UAV_MPC__NMPC_NODE_HPP_
#define UAV_MPC__NMPC_NODE_HPP_

#include <Eigen/Dense>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_angular_velocity.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/vehicle_attitude_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"

#include "uav_mpc/acados_wrapper.hpp"
#include "uav_mpc/quadrotor_dynamics.hpp"
#include "uav_mpc/trajectory_generator.hpp"
#include "uav_mpc/msg/nmpc_status.hpp"
#include "uav_mpc/srv/set_trajectory.hpp"

namespace uav_mpc
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/// Internal mission state machine. Independent of the PX4 nav state; the node only ever
/// *requests* offboard and arming, PX4 remains the authority.
enum class ControllerState
{
  Idle,          ///< configured, not publishing setpoints
  Streaming,     ///< publishing hover setpoints to satisfy PX4's pre-offboard stream requirement
  Takeoff,       ///< NMPC tracking a vertical ramp to `takeoff_altitude`
  Tracking,      ///< NMPC tracking the active trajectory
  Landing,       ///< NMPC tracking a descent ramp
  Failsafe       ///< solver gave up; streaming level-hover attitude and warning loudly
};

class NmpcNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit NmpcNode(const rclcpp::NodeOptions & options);
  ~NmpcNode() override;

  // --- lifecycle transitions -----------------------------------------------------------------
  // TODO(deepseek): declare params, load airframe + solver config, build the solver & generator.
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  // TODO(deepseek): activate publishers, start the 100 Hz timer, arm the state machine.
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  // TODO(deepseek): stop the timer, deactivate publishers, hand control back to PX4.
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  // TODO(deepseek): release the solver and subscriptions.
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  // TODO(deepseek): best-effort safe stop.
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  // --- parameters ----------------------------------------------------------------------------
  // TODO(deepseek): implement — declares every parameter in config/nmpc_params.yaml with
  // descriptors, ranges and read-only flags.
  void declareParameters();
  // TODO(deepseek): implement — reads declared params into the members below; validates.
  bool loadParameters(std::string * error);
  // TODO(deepseek): implement — on-the-fly reconfiguration of weights and trajectory params.
  rcl_interfaces::msg::SetParametersResult onParameterUpdate(
    const std::vector<rclcpp::Parameter> & params);

  // --- subscriptions -------------------------------------------------------------------------
  // TODO(deepseek): implement — cache NED position/velocity, convert to ENU, stamp arrival time.
  void onLocalPosition(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
  // TODO(deepseek): implement — cache attitude, convert NED/FRD -> ENU/FLU.
  void onAttitude(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
  // TODO(deepseek): implement — cache body rates, FRD -> FLU.
  void onAngularVelocity(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr msg);
  // TODO(deepseek): implement — track arming state + nav state, detect loss of offboard.
  void onVehicleStatus(const px4_msgs::msg::VehicleStatus::SharedPtr msg);

  // --- services ------------------------------------------------------------------------------
  // TODO(deepseek): implement — swap the active trajectory at runtime, rejecting the request
  // unless the vehicle is in Tracking or Streaming and the new trajectory starts near the
  // current position.
  void onSetTrajectory(
    const std::shared_ptr<uav_mpc::srv::SetTrajectory::Request> request,
    std::shared_ptr<uav_mpc::srv::SetTrajectory::Response> response);

  // --- control loop --------------------------------------------------------------------------
  /// 100 Hz timer body. Ordering is fixed (§7.4):
  ///   1. staleness check -> Failsafe on timeout
  ///   2. assemble x0 (+ latency compensation)
  ///   3. sample the reference horizon
  ///   4. push into acados, solve, measure
  ///   5. map u0 -> attitude setpoint, publish
  ///   6. publish OffboardControlMode, status, RViz paths
  // TODO(deepseek): implement
  void controlLoop();

  /// Build the solver state vector from the cached PX4 messages, in the active AttitudeRep.
  /// Returns false if any source is stale beyond `state_timeout_`.
  // TODO(deepseek): implement
  bool assembleState(Eigen::VectorXd * x0, std::string * why_stale);

  /// Forward-integrate x0 by the measured sensor+actuator latency using the last applied input,
  /// so the OCP starts from where the vehicle will be when the command lands.
  // TODO(deepseek): implement
  Eigen::VectorXd compensateLatency(const Eigen::VectorXd & x0, double latency_s) const;

  /// u0 [per-rotor thrust, N] -> PX4 VehicleAttitudeSetpoint (q_d in NED/FRD, normalised
  /// thrust_body[2] in [-1, 0]). Uses the desired body z-axis implied by the predicted
  /// acceleration at stage 1, per §7.6.
  // TODO(deepseek): implement
  px4_msgs::msg::VehicleAttitudeSetpoint toAttitudeSetpoint(
    const Eigen::VectorXd & u0, const Eigen::VectorXd & x_pred_1) const;

  /// Normalised thrust for PX4 from collective thrust [N], using the hover-thrust calibration
  /// (`px4_hover_thrust_` from config/px4_overrides.yaml) and a linear/quadratic mapping.
  // TODO(deepseek): implement
  double normaliseThrust(double collective_thrust_newton) const;

  // --- PX4 handshake -------------------------------------------------------------------------
  // TODO(deepseek): implement — must be published at >= 2 Hz *before* and during offboard.
  void publishOffboardControlMode();
  // TODO(deepseek): implement — VEHICLE_CMD_DO_SET_MODE to offboard.
  void requestOffboardMode();
  // TODO(deepseek): implement — VEHICLE_CMD_COMPONENT_ARM_DISARM. Gated on `auto_arm_` param.
  void requestArm();
  // TODO(deepseek): implement
  void requestDisarm();

  // --- state machine -------------------------------------------------------------------------
  // TODO(deepseek): implement — the transition table of §7.3.
  void updateControllerState();
  // TODO(deepseek): implement — level attitude, hover thrust, loud throttled warning.
  void enterFailsafe(const std::string & reason);

  // --- telemetry -----------------------------------------------------------------------------
  // TODO(deepseek): implement
  void publishStatus(const SolveResult & result, const Eigen::VectorXd & x0);
  // TODO(deepseek): implement — predicted + reference paths in ENU for RViz.
  void publishVisualisation();

  // --- publishers / subscribers ---------------------------------------------------------------
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_local_position_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr sub_attitude_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAngularVelocity>::SharedPtr sub_angular_velocity_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr sub_vehicle_status_;

  rclcpp_lifecycle::LifecyclePublisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr
    pub_attitude_setpoint_;
  rclcpp_lifecycle::LifecyclePublisher<px4_msgs::msg::OffboardControlMode>::SharedPtr
    pub_offboard_mode_;
  rclcpp_lifecycle::LifecyclePublisher<px4_msgs::msg::VehicleCommand>::SharedPtr pub_command_;
  rclcpp_lifecycle::LifecyclePublisher<uav_mpc::msg::NmpcStatus>::SharedPtr pub_status_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr pub_predicted_path_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr pub_reference_path_;

  rclcpp::Service<uav_mpc::srv::SetTrajectory>::SharedPtr srv_set_trajectory_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::CallbackGroup::SharedPtr control_callback_group_;   ///< MutuallyExclusive, real-time
  rclcpp::CallbackGroup::SharedPtr telemetry_callback_group_; ///< Reentrant, best-effort
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // --- owned components ------------------------------------------------------------------------
  std::unique_ptr<AcadosWrapper> solver_;
  std::unique_ptr<TrajectoryGenerator> trajectory_;
  std::unique_ptr<QuadrotorDynamics<double, AttitudeRep::Quaternion>> model_;
  QuadrotorParams airframe_{};
  SolverConfig solver_config_{};

  // --- cached state (guarded by state_mutex_) ---------------------------------------------------
  mutable std::mutex state_mutex_;
  Eigen::Vector3d position_enu_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_enu_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond attitude_enu_flu_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d body_rates_flu_{Eigen::Vector3d::Zero()};
  rclcpp::Time last_position_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_attitude_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_rates_stamp_{0, 0, RCL_ROS_TIME};
  bool position_valid_{false};

  std::atomic<ControllerState> controller_state_{ControllerState::Idle};
  std::atomic<bool> armed_{false};
  std::atomic<bool> offboard_active_{false};
  rclcpp::Time trajectory_start_time_{0, 0, RCL_ROS_TIME};
  Eigen::VectorXd last_applied_input_{};
  int offboard_stream_counter_{0};
  int consecutive_solver_failures_{0};

  // --- parameters (mirrors config/nmpc_params.yaml) ---------------------------------------------
  double control_rate_hz_{100.0};
  double state_timeout_s_{0.1};
  double latency_compensation_s_{0.02};
  double takeoff_altitude_m_{1.5};
  double px4_hover_thrust_{0.5};
  bool auto_arm_{false};
  bool publish_visualisation_{true};
  std::string airframe_params_path_{};
};

}  // namespace uav_mpc

#endif  // UAV_MPC__NMPC_NODE_HPP_
