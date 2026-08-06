// Copyright (c) 2026 Ali-Eimaan.
// SPDX-License-Identifier: BSD-3-Clause
//
// ROS 2 lifecycle node running the NMPC at 100 Hz against PX4 over uXRCE-DDS.
// See .deepseek/07_NODE.md §7.
//
// [px4_msgs] This header and nmpc_node.cpp depend on the px4_msgs package
// (https://github.com/PX4/px4_msgs), which is NOT installed in this environment.
// The node targets in CMakeLists.txt are gated on px4_msgs_FOUND, so the core library
// and tests build without it. To build the node, first install px4_msgs:
//   git clone https://github.com/PX4/px4_msgs.git <ros2_ws>/src/px4_msgs
//   cd <ros2_ws> && colcon build --packages-select px4_msgs
// Every place that needs px4_msgs is marked with a `[px4_msgs]` comment.
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
  /// Load params + airframe + model + trajectory + solver; create subs, pubs, service, param cb.
  /// Returns FAILURE (never throws) with one clear RCLCPP_ERROR naming the cause.
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /// Activate publishers, create the 100 Hz timer, state -> Streaming.
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /// Cancel + reset the timer FIRST, then deactivate pubs, state -> Idle.
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /// Release solver, generator, model, pubs/subs.
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /// Best-effort safe stop: stop timer + publishing. Does NOT disarm (PX4 failsafe is authority).
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  // --- parameters ----------------------------------------------------------------------------
  /// Declares every key of config/nmpc_params.yaml with descriptors, ranges, read-only flags.
  void declareParameters();
  /// Reads declared params into the members below; validates. Returns false + reason on error.
  bool loadParameters(std::string * error);
  /// Live weight + trajectory reconfiguration; rejects structural changes with a reason string.
  rcl_interfaces::msg::SetParametersResult onParameterUpdate(
    const std::vector<rclcpp::Parameter> & params);

  // --- subscriptions -------------------------------------------------------------------------
  /// Caches NED position/velocity, converts to ENU, stamps arrival time. Rejects invalid flags.
  void onLocalPosition(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
  /// Caches attitude; converts NED/FRD -> ENU/FLU (msg->q is w,x,y,z NED/FRD).
  void onAttitude(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
  /// Caches body rates; FRD -> FLU is (x, -y, -z).
  void onAngularVelocity(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr msg);
  /// Tracks arming state + nav state, detects loss of offboard.
  void onVehicleStatus(const px4_msgs::msg::VehicleStatus::SharedPtr msg);

  // --- services ------------------------------------------------------------------------------
  /// Swaps the active trajectory at runtime; rejects unless in Tracking/Streaming and the new
  /// trajectory starts within `max_jump_on_switch` of the current position.
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
  void controlLoop();

  /// Build the solver state vector from the cached PX4 messages, in the active AttitudeRep.
  /// Returns false if any source is stale beyond `state_timeout_`.
  bool assembleState(Eigen::VectorXd * x0, std::string * why_stale);

  /// Forward-integrate x0 by the measured sensor+actuator latency using the last applied input,
  /// so the OCP starts from where the vehicle will be when the command lands.
  Eigen::VectorXd compensateLatency(const Eigen::VectorXd & x0, double latency_s) const;

  /// u0 [per-rotor thrust, N] -> PX4 VehicleAttitudeSetpoint (q_d in NED/FRD, normalised
  /// thrust_body[2] in [-1, 0]). Uses the desired body z-axis implied by the predicted
  /// acceleration at stage 1, per §7.6.
  px4_msgs::msg::VehicleAttitudeSetpoint toAttitudeSetpoint(
    const Eigen::VectorXd & u0, const Eigen::VectorXd & x_pred_1) const;

  /// Normalised thrust for PX4 from collective thrust [N], using the hover-thrust calibration
  /// (`px4_hover_thrust_` from config/px4_overrides.yaml) and a linear/quadratic mapping.
  double normaliseThrust(double collective_thrust_newton) const;

  // --- PX4 handshake -------------------------------------------------------------------------
  /// Must be published at >= 2 Hz *before* and during offboard (every control tick).
  void publishOffboardControlMode();
  /// VEHICLE_CMD_DO_SET_MODE (176) to offboard: param1 = 1, param2 = 6.
  void requestOffboardMode();
  /// VEHICLE_CMD_COMPONENT_ARM_DISARM (400), param1 = 1. Gated on `auto_arm_` param.
  void requestArm();
  /// param1 = 0. Only from Landing with |z| < 0.15 m and |v| < 0.2 m/s.
  void requestDisarm();

  // --- state machine -------------------------------------------------------------------------
  /// The transition table of §7.3.
  void updateControllerState();
  /// Level attitude, hover thrust, loud throttled warning, solver guess reset.
  void enterFailsafe(const std::string & reason);

  // --- telemetry -----------------------------------------------------------------------------
  /// Fills NmpcStatus (solve info, tracking error, applied command, timing).
  void publishStatus(const SolveResult & result, const Eigen::VectorXd & x0);
  /// Predicted + reference nav_msgs::Path in frame "map" (ENU). Skipped when nobody subscribes.
  void publishVisualisation();

  /// Overrides the reference horizon with a vertical position ramp for Takeoff/Landing (§7.3).
  void fillTakeoffLandingHorizon(
    ControllerState state, const Eigen::VectorXd & x0, double dt, int n, double hover_thrust_n);

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
  TrajectoryParams trajectory_params_{};  ///< last accepted trajectory (service + param updates)

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
  std::atomic<bool> landing_requested_{false};
  rclcpp::Time trajectory_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_tick_time_{0, 0, RCL_ROS_TIME};
  Eigen::VectorXd last_applied_input_{};
  Eigen::VectorXd last_x0_{};      ///< last assembled state, for failsafe resetToHover
  Eigen::Quaterniond last_q_d_enu_{Eigen::Quaterniond::Identity()};  ///< last sent setpoint attitude
  int offboard_stream_counter_{0};
  int consecutive_solver_failures_{0};
  double hover_thrust_n_{0.0};   ///< [N] per-rotor hover thrust, m*g/4 — solver seed + takeoff ref

  // --- reference buffers (pre-sized in on_configure; NO allocation inside the control loop) -----
  std::vector<Eigen::VectorXd> x_refs_;   ///< N+1 states
  std::vector<Eigen::VectorXd> u_refs_;   ///< N inputs

  // --- parameters (mirrors config/nmpc_params.yaml) ---------------------------------------------
  double control_rate_hz_{100.0};
  double state_timeout_s_{0.1};
  double latency_compensation_s_{0.02};
  double takeoff_altitude_m_{1.5};
  double takeoff_speed_mps_{0.5};
  double landing_speed_mps_{0.3};
  double max_jump_on_switch_m_{1.0};
  double log_solve_time_warn_ms_{2.0};
  double status_publish_rate_hz_{100.0};
  double px4_hover_thrust_{0.5};
  bool auto_arm_{false};
  bool publish_visualisation_{true};
  std::string airframe_params_path_{};

  // --- per-tick telemetry cache -----------------------------------------------------------------
  double loop_period_ms_{0.0};
  double loop_duration_ms_{0.0};
  double state_age_ms_{0.0};

  /// Guards trajectory_ swap (service callback) vs sampling (control loop).
  mutable std::mutex trajectory_mutex_;
};

}  // namespace uav_mpc

#endif  // UAV_MPC__NMPC_NODE_HPP_
