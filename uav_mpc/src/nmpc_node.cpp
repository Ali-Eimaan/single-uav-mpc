// Copyright (c) 2026 Ali-Eimaan.
// SPDX-License-Identifier: BSD-3-Clause
//
// ROS 2 lifecycle node running the NMPC at 100 Hz against PX4 over uXRCE-DDS.
// See .deepseek/07_NODE.md §7.
//
// [px4_msgs] This file depends on px4_msgs (https://github.com/PX4/px4_msgs), which is NOT
// installed in this environment — the node targets in CMakeLists.txt are gated on
// px4_msgs_FOUND, so the core library and tests build without it. To build the node, install
// px4_msgs first (see the comment in CMakeLists.txt), then re-run colcon. Every place below
// that needs px4_msgs is marked with a `[px4_msgs]` comment.
//
// VERIFICATION STATUS: syntax-verified against the .deepseek spec, NOT compile-verified —
// px4_msgs is absent and no build has been run. Treat every px4_msgs field name/constant as
// UNVERIFIED until the node compiles against the pinned px4_msgs release.

#include "uav_mpc/nmpc_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "rclcpp_components/register_node_macro.hpp"
#include "uav_mpc/msg/solver_diagnostics.hpp"

namespace uav_mpc
{

namespace
{

constexpr int kOffboardPreStreamCount = 20;  ///< PX4 wants setpoints BEFORE it accepts offboard
constexpr double kMinAltitudeM = 0.3;        ///< SetTrajectory validation bounds
constexpr double kMaxAltitudeM = 10.0;
constexpr double kLandingZThresholdM = 0.15;
constexpr double kLandingVThresholdMps = 0.2;
constexpr double kTakeoffZWindowM = 0.1;

/// PX4 QoS: BestEffort + Volatile, KeepLast(5). A mismatched profile silently receives nothing.
rclcpp::QoS px4Qos()
{
  rclcpp::QoS qos(rclcpp::KeepLast(5));
  qos.best_effort().durability_volatile();
  return qos;
}

/// Timestamps to PX4 are microseconds — nanoseconds makes PX4 silently drop the setpoint (§7.5).
std::uint64_t nowUs(const rclcpp::Clock & clock)
{
  return static_cast<std::uint64_t>(clock.now().nanoseconds() / 1000);
}

Eigen::Vector3d toEigen(const geometry_msgs::msg::Point & p)
{
  return Eigen::Vector3d(p.x, p.y, p.z);
}

geometry_msgs::msg::Vector3 toVector3(const Eigen::Vector3d & v)
{
  geometry_msgs::msg::Vector3 out;
  out.x = v.x();
  out.y = v.y();
  out.z = v.z();
  return out;
}

/// Wrap an angle to [-pi, pi].
double wrapPi(double a)
{
  while (a > M_PI) {a -= 2.0 * M_PI;}
  while (a < -M_PI) {a += 2.0 * M_PI;}
  return a;
}

}  // namespace

NmpcNode::NmpcNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("nmpc_node", options)
{
  // Declaration only. Solver/airframe/trajectory allocation belongs in on_configure so the
  // node can be re-configured (and recovered) without a process restart.
  declareParameters();
}

NmpcNode::~NmpcNode() = default;

// ================================================================================================
// Lifecycle
// ================================================================================================

CallbackReturn NmpcNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  // Callback groups first: the subscriptions/services created below are bound to them.
  if (!control_callback_group_) {
    control_callback_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  }
  if (!telemetry_callback_group_) {
    telemetry_callback_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
  }

  // --- parameters ---------------------------------------------------------------------------
  std::string error;
  if (!loadParameters(&error)) {
    RCLCPP_ERROR(get_logger(), "on_configure: %s", error.c_str());
    return CallbackReturn::FAILURE;
  }

  // --- airframe + dynamics model ------------------------------------------------------------
  if (airframe_params_path_.empty()) {
    RCLCPP_ERROR(get_logger(), "on_configure: airframe_params_path is empty — set it in the "
                 "launch file (or pass the calibration yaml directly)");
    return CallbackReturn::FAILURE;
  }
  try {
    airframe_ = QuadrotorParams::fromYaml(airframe_params_path_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "on_configure: failed to load airframe '%s': %s",
      airframe_params_path_.c_str(), e.what());
    return CallbackReturn::FAILURE;
  }
  std::string why;
  if (!airframe_.isValid(&why)) {
    RCLCPP_ERROR(get_logger(), "on_configure: airframe invalid: %s", why.c_str());
    return CallbackReturn::FAILURE;
  }
  model_ = std::make_unique<QuadrotorDynamics<double, AttitudeRep::Quaternion>>(airframe_);
  hover_thrust_n_ = model_->hoverThrustPerRotor();

  // --- trajectory ---------------------------------------------------------------------------
  try {
    trajectory_ = std::make_unique<TrajectoryGenerator>(trajectory_params_);
    trajectory_->generate();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "on_configure: trajectory generation failed: %s", e.what());
    return CallbackReturn::FAILURE;
  }

  // --- solver -------------------------------------------------------------------------------
  solver_ = std::make_unique<AcadosWrapper>();
  if (!solver_->initialise(solver_config_, &error)) {
    RCLCPP_ERROR(get_logger(), "on_configure: solver initialisation failed: %s", error.c_str());
    return CallbackReturn::FAILURE;
  }
  // Pre-size the reference buffers NOW so the control loop performs no allocation (§7.4).
  const int n = solver_config_.horizon_steps;
  const int nx = solver_->nx();
  x_refs_.assign(static_cast<std::size_t>(n) + 1, Eigen::VectorXd(nx));
  u_refs_.assign(static_cast<std::size_t>(n), Eigen::VectorXd(solver_->nu()));
  last_applied_input_ = Eigen::VectorXd::Zero(solver_->nu());
  last_x0_ = Eigen::VectorXd::Zero(nx);
  // Online parameters stay constant in this version: no wind estimate, nominal mass, level
  // quaternion reference for the geometric attitude error.
  Eigen::VectorXd p = Eigen::VectorXd::Zero(solver_->np());
  p(3) = 1.0;   // mass_scale
  p(4) = 1.0;   // q_ref.w
  solver_->setParameters(p);

  // --- subscriptions [px4_msgs] --------------------------------------------------------------
  // BestEffort + Volatile QoS is mandatory for every /fmu/* topic (§7.2).
  sub_local_position_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
    "/fmu/out/vehicle_local_position", px4Qos(),
    std::bind(&NmpcNode::onLocalPosition, this, std::placeholders::_1),
    telemetry_callback_group_);
  sub_attitude_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
    "/fmu/out/vehicle_attitude", px4Qos(),
    std::bind(&NmpcNode::onAttitude, this, std::placeholders::_1),
    telemetry_callback_group_);
  sub_angular_velocity_ = create_subscription<px4_msgs::msg::VehicleAngularVelocity>(
    "/fmu/out/vehicle_angular_velocity", px4Qos(),
    std::bind(&NmpcNode::onAngularVelocity, this, std::placeholders::_1),
    telemetry_callback_group_);
  sub_vehicle_status_ = create_subscription<px4_msgs::msg::VehicleStatus>(
    "/fmu/out/vehicle_status", px4Qos(),
    std::bind(&NmpcNode::onVehicleStatus, this, std::placeholders::_1),
    telemetry_callback_group_);

  // --- publishers (created inactive; activated on on_activate) --------------------------------
  pub_attitude_setpoint_ = create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
    "/fmu/in/vehicle_attitude_setpoint", px4Qos());
  pub_offboard_mode_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
    "/fmu/in/offboard_control_mode", px4Qos());
  pub_command_ = create_publisher<px4_msgs::msg::VehicleCommand>(
    "/fmu/in/vehicle_command", px4Qos());
  pub_status_ = create_publisher<uav_mpc::msg::NmpcStatus>("~/status", rclcpp::QoS(10));
  pub_predicted_path_ = create_publisher<nav_msgs::msg::Path>(
    "~/predicted_path", rclcpp::QoS(10));
  pub_reference_path_ = create_publisher<nav_msgs::msg::Path>(
    "~/reference_path", rclcpp::QoS(10));

  // --- service -------------------------------------------------------------------------------
  srv_set_trajectory_ = create_service<uav_mpc::srv::SetTrajectory>(
    "~/set_trajectory",
    std::bind(&NmpcNode::onSetTrajectory, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, telemetry_callback_group_);

  // --- parameter callback ---------------------------------------------------------------------
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&NmpcNode::onParameterUpdate, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "configured: airframe=%s N=%d Tf=%.2f nx=%d nu=%d np=%d hover=%.4f N",
    airframe_.frame_name.c_str(), solver_config_.horizon_steps, solver_config_.horizon_time,
    solver_->nx(), solver_->nu(), solver_->np(), hover_thrust_n_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn NmpcNode::on_activate(const rclcpp_lifecycle::State & state)
{
  // The base implementation activates the node's lifecycle publishers.
  rclcpp_lifecycle::LifecycleNode::on_activate(state);

  const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
  control_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&NmpcNode::controlLoop, this), control_callback_group_);

  // Reset the state machine + counters; the trajectory clock starts at activation.
  controller_state_.store(ControllerState::Streaming);
  armed_.store(false);
  offboard_active_.store(false);
  landing_requested_.store(false);
  offboard_stream_counter_ = 0;
  consecutive_solver_failures_ = 0;
  last_tick_time_ = this->now();
  trajectory_start_time_ = this->now();

  RCLCPP_INFO(get_logger(), "activated: streaming hover setpoints at %.1f Hz",
    control_rate_hz_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn NmpcNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  // Cancel + reset the timer FIRST so no callback runs against deactivated publishers (§7.1).
  if (control_timer_) {
    control_timer_->cancel();
    control_timer_.reset();
  }
  rclcpp_lifecycle::LifecycleNode::on_deactivate(state);
  controller_state_.store(ControllerState::Idle);
  RCLCPP_INFO(get_logger(), "deactivated: setpoint stream stopped, control handed to PX4");
  return CallbackReturn::SUCCESS;
}

CallbackReturn NmpcNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  control_timer_.reset();
  solver_.reset();
  trajectory_.reset();
  model_.reset();
  sub_local_position_.reset();
  sub_attitude_.reset();
  sub_angular_velocity_.reset();
  sub_vehicle_status_.reset();
  pub_attitude_setpoint_.reset();
  pub_offboard_mode_.reset();
  pub_command_.reset();
  pub_status_.reset();
  pub_predicted_path_.reset();
  pub_reference_path_.reset();
  srv_set_trajectory_.reset();
  param_callback_handle_.reset();
  control_callback_group_.reset();
  telemetry_callback_group_.reset();
  RCLCPP_INFO(get_logger(), "cleaned up");
  return CallbackReturn::SUCCESS;
}

CallbackReturn NmpcNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  // Best-effort: stop the timer and publishing. Deliberately NOT arming/disarming — PX4's own
  // offboard-loss failsafe (COM_OF_LOSS_T / COM_OBL_RC_ACT) is the safer authority (§7.1).
  if (control_timer_) {control_timer_->cancel(); control_timer_.reset();}
  controller_state_.store(ControllerState::Idle);
  RCLCPP_INFO(get_logger(), "shutdown: setpoint stream stopped (PX4 failsafe remains armed)");
  return CallbackReturn::SUCCESS;
}

// ================================================================================================
// Parameters
// ================================================================================================

namespace
{

rcl_interfaces::msg::ParameterDescriptor descriptor(
  const std::string & description, bool read_only = false,
  double lo = 0.0, double hi = 0.0, bool has_range = false)
{
  rcl_interfaces::msg::ParameterDescriptor d;
  d.description = description;
  d.read_only = read_only;
  if (has_range) {
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = lo;
    range.to_value = hi;
    d.floating_point_range.push_back(range);
  }
  return d;
}

}  // namespace

void NmpcNode::declareParameters()
{
  // --- loop timing -------------------------------------------------------------------------
  declare_parameter("control_rate_hz", 100.0,
    descriptor("NMPC + setpoint publish rate [Hz]", false, 10.0, 1000.0, true));
  declare_parameter("state_timeout_s", 0.1,
    descriptor("odometry older than this => Failsafe [s]", false, 0.01, 5.0, true));
  declare_parameter("latency_compensation_s", 0.02,
    descriptor("forward-integrate x0 by this before solving [s]", false, 0.0, 1.0, true));

  // --- OCP structure (read-only: baked into the generated solver) -----------------------------
  declare_parameter("horizon_steps", 20, descriptor("OCP horizon N (codegen)", true));
  declare_parameter("horizon_time", 1.0, descriptor("OCP horizon Tf [s] (codegen)", true));
  declare_parameter("attitude_rep", "quaternion",
    descriptor("attitude parameterisation (codegen)", true));

  // --- cost weights --------------------------------------------------------------------------
  declare_parameter("q_diag", std::vector<double>{200.0, 200.0, 400.0, 10.0, 10.0, 20.0,
      50.0, 50.0, 20.0, 1.0, 1.0, 1.0},
    descriptor("12 stage weights [p v att_err omega]"));
  declare_parameter("r_diag", std::vector<double>{0.5, 0.5, 0.5, 0.5},
    descriptor("4 input weights"));
  declare_parameter("q_terminal_diag",
    std::vector<double>{400.0, 400.0, 800.0, 20.0, 20.0, 40.0, 100.0, 100.0, 40.0, 2.0, 2.0, 2.0},
    descriptor("12 terminal weights"));

  // --- solver behaviour ----------------------------------------------------------------------
  declare_parameter("max_sqp_iterations", 1,
    descriptor("SQP-RTI iterations (codegen)" , true));
  declare_parameter("solve_time_budget_ms", 5.0,
    descriptor("wall-clock solve budget [ms]; exceeded => Timeout", false, 0.1, 100.0, true));
  declare_parameter("max_consecutive_failures", 5,
    descriptor("consecutive solver failures before Failsafe", false, 1.0, 100.0, true));
  declare_parameter("warm_start", true, descriptor("use previous solution as initial guess"));
  declare_parameter("shift_on_warm_start", true,
    descriptor("shift the previous solution one stage forward"));

  // --- airframe + PX4 interface ---------------------------------------------------------------
  declare_parameter("airframe_params_path", "",
    descriptor("absolute path to params/*_calibration.yaml (launch fills this in)", true));
  declare_parameter("px4_hover_thrust", 0.62,
    descriptor("PX4 MPC_THR_HOVER — MUST match config/px4_overrides.yaml", false, 0.0, 1.0, true));
  declare_parameter("auto_arm", false, descriptor("arm via VehicleCommand on takeoff (SITL/CI)"));

  // --- mission -------------------------------------------------------------------------------
  declare_parameter("takeoff_altitude_m", 1.5,
    descriptor("target AGL altitude after takeoff [m]", false, 0.3, 10.0, true));
  declare_parameter("takeoff_speed_mps", 0.5,
    descriptor("vertical climb rate during takeoff [m/s]", false, 0.1, 3.0, true));
  declare_parameter("landing_speed_mps", 0.3,
    descriptor("vertical descent rate during landing [m/s]", false, 0.05, 2.0, true));
  declare_parameter("max_jump_on_switch_m", 1.0,
    descriptor("max distance between the vehicle and a new trajectory's start", false, 0.0, 10.0, true));
  declare_parameter("publish_visualisation", true,
    descriptor("publish ~/predicted_path and ~/reference_path for RViz"));
  declare_parameter("status_publish_rate_hz", 100.0,
    descriptor("~/status publish rate [Hz]", false, 1.0, 500.0, true));
  declare_parameter("log_solve_time_warn_ms", 2.0,
    descriptor("log a throttled warning when a control tick exceeds this [ms]",
      false, 0.0, 100.0, true));

  // --- trajectory (mirrors config/trajectory_params.yaml) ---------------------------------------
  declare_parameter("trajectory.type", "figure8",
    descriptor("hover | figure8 | lemniscate | circle | waypoints | step"));
  declare_parameter("trajectory.center", std::vector<double>{0.0, 0.0, 0.0},
    descriptor("[m] ENU"));
  declare_parameter("trajectory.altitude", 1.5,
    descriptor("[m] AGL", false, 0.0, 20.0, true));
  declare_parameter("trajectory.amplitude_x", 2.0, descriptor("[m]"));
  declare_parameter("trajectory.amplitude_y", 2.0, descriptor("[m]"));
  declare_parameter("trajectory.amplitude_z", 0.0, descriptor("[m]"));
  declare_parameter("trajectory.period", 8.0, descriptor("[s]", false, 0.1, 3600.0, true));
  declare_parameter("trajectory.ramp_in_time", 3.0, descriptor("[s]"));
  declare_parameter("trajectory.yaw_follows_velocity", true, descriptor(""));
  declare_parameter("trajectory.fixed_yaw", 0.0, descriptor("[rad]"));
  declare_parameter("trajectory.max_velocity", 5.0, descriptor("[m/s]"));
  declare_parameter("trajectory.max_acceleration", 8.0, descriptor("[m/s^2]"));
}

bool NmpcNode::loadParameters(std::string * error)
{
  control_rate_hz_ = get_parameter("control_rate_hz").as_double();
  state_timeout_s_ = get_parameter("state_timeout_s").as_double();
  latency_compensation_s_ = get_parameter("latency_compensation_s").as_double();
  takeoff_altitude_m_ = get_parameter("takeoff_altitude_m").as_double();
  takeoff_speed_mps_ = get_parameter("takeoff_speed_mps").as_double();
  landing_speed_mps_ = get_parameter("landing_speed_mps").as_double();
  max_jump_on_switch_m_ = get_parameter("max_jump_on_switch_m").as_double();
  px4_hover_thrust_ = get_parameter("px4_hover_thrust").as_double();
  auto_arm_ = get_parameter("auto_arm").as_bool();
  publish_visualisation_ = get_parameter("publish_visualisation").as_bool();
  status_publish_rate_hz_ = get_parameter("status_publish_rate_hz").as_double();
  log_solve_time_warn_ms_ = get_parameter("log_solve_time_warn_ms").as_double();
  airframe_params_path_ = get_parameter("airframe_params_path").as_string();

  // --- solver config -------------------------------------------------------------------------
  solver_config_.horizon_steps = get_parameter("horizon_steps").as_int();
  solver_config_.horizon_time = get_parameter("horizon_time").as_double();
  solver_config_.max_sqp_iterations = get_parameter("max_sqp_iterations").as_int();
  solver_config_.solve_time_budget_ms = get_parameter("solve_time_budget_ms").as_double();
  solver_config_.max_consecutive_failures = get_parameter("max_consecutive_failures").as_int();
  solver_config_.warm_start = get_parameter("warm_start").as_bool();
  solver_config_.shift_on_warm_start = get_parameter("shift_on_warm_start").as_bool();
  const auto q = get_parameter("q_diag").as_double_array();
  const auto r = get_parameter("r_diag").as_double_array();
  const auto qe = get_parameter("q_terminal_diag").as_double_array();
  solver_config_.q_diag = Eigen::Map<const Eigen::VectorXd>(q.data(), q.size());
  solver_config_.r_diag = Eigen::Map<const Eigen::VectorXd>(r.data(), r.size());
  solver_config_.q_terminal_diag = Eigen::Map<const Eigen::VectorXd>(qe.data(), qe.size());

  // --- trajectory config (config/trajectory_params.yaml values arrive as parameters) ----------
  trajectory_params_.type = TrajectoryType::Hover;
  const std::string ttype = get_parameter("trajectory.type").as_string();
  if (ttype == "hover") {trajectory_params_.type = TrajectoryType::Hover;}
  else if (ttype == "figure8") {trajectory_params_.type = TrajectoryType::Figure8;}
  else if (ttype == "lemniscate") {trajectory_params_.type = TrajectoryType::Lemniscate;}
  else if (ttype == "circle") {trajectory_params_.type = TrajectoryType::Circle;}
  else if (ttype == "waypoints") {trajectory_params_.type = TrajectoryType::Waypoints;}
  else if (ttype == "step") {trajectory_params_.type = TrajectoryType::Step;}
  else {
    if (error) {*error = "unknown trajectory.type '" + ttype + "'";}
    return false;
  }
  const auto c = get_parameter("trajectory.center").as_double_array();
  if (c.size() == 3) {trajectory_params_.center = Eigen::Vector3d(c[0], c[1], c[2]);}
  trajectory_params_.altitude = get_parameter("trajectory.altitude").as_double();
  trajectory_params_.amplitude_x = get_parameter("trajectory.amplitude_x").as_double();
  trajectory_params_.amplitude_y = get_parameter("trajectory.amplitude_y").as_double();
  trajectory_params_.amplitude_z = get_parameter("trajectory.amplitude_z").as_double();
  trajectory_params_.period = get_parameter("trajectory.period").as_double();
  trajectory_params_.ramp_in_time = get_parameter("trajectory.ramp_in_time").as_double();
  trajectory_params_.yaw_follows_velocity = get_parameter("trajectory.yaw_follows_velocity").as_bool();
  trajectory_params_.fixed_yaw = get_parameter("trajectory.fixed_yaw").as_double();
  trajectory_params_.max_velocity = get_parameter("trajectory.max_velocity").as_double();
  trajectory_params_.max_acceleration = get_parameter("trajectory.max_acceleration").as_double();

  // --- validation (§9.1) ----------------------------------------------------------------------
  if (control_rate_hz_ <= 0.0) {
    if (error) {*error = "control_rate_hz must be > 0";}
    return false;
  }
  if (state_timeout_s_ < 2.0 / control_rate_hz_) {
    if (error) {*error = "state_timeout_s must be >= 2/control_rate_hz";}
    return false;
  }
  if (solver_config_.q_diag.size() != 12 || solver_config_.r_diag.size() != 4 ||
      solver_config_.q_terminal_diag.size() != 12) {
    if (error) {*error = "weight vector sizes must be 12/4/12 (they weight the cost residual)";}
    return false;
  }
  for (Eigen::Index i = 0; i < solver_config_.q_diag.size(); ++i) {
    if (solver_config_.q_diag(i) < 0.0 || solver_config_.q_terminal_diag(i) < 0.0) {
      if (error) {*error = "weights must be non-negative";}
      return false;
    }
  }
  for (double w : solver_config_.r_diag) {
    if (w < 0.0) {
      if (error) {*error = "weights must be non-negative";}
      return false;
    }
  }
  // q_terminal_diag[i] < q_diag[i] is suspicious — warn, do not reject (§9.1).
  for (Eigen::Index i = 0; i < solver_config_.q_diag.size(); ++i) {
    if (solver_config_.q_terminal_diag(i) < solver_config_.q_diag(i)) {
      RCLCPP_WARN(get_logger(),
        "q_terminal_diag[%ld] < q_diag[%ld] — terminal cost weaker than stage cost", i, i);
    }
  }
  if (airframe_params_path_.empty()) {
    if (error) {*error = "airframe_params_path is empty";}
    return false;
  }
  return true;
}

rcl_interfaces::msg::SetParametersResult NmpcNode::onParameterUpdate(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  // Structural keys are baked into the generated solver; a useful rejection beats ten
  // with "invalid parameter" (§9.1).
  const std::vector<std::string> read_only = {
    "horizon_steps", "horizon_time", "attitude_rep", "airframe_params_path",
    "max_sqp_iterations"};
  for (const auto & p : params) {
    for (const auto & ro : read_only) {
      if (p.get_name() == ro) {
        result.successful = false;
        result.reason = ro + " is baked into the generated solver; re-run codegen and restart";
        return result;
      }
    }
  }

  // Weights: apply live to the solver.
  Eigen::VectorXd q_diag, r_diag, q_terminal_diag;
  bool have_weights = false;
  for (const auto & p : params) {
    if (p.get_name() == "q_diag") {
      const auto v = p.as_double_array();
      q_diag = Eigen::Map<const Eigen::VectorXd>(v.data(), v.size());
      have_weights = true;
    } else if (p.get_name() == "r_diag") {
      const auto v = p.as_double_array();
      r_diag = Eigen::Map<const Eigen::VectorXd>(v.data(), v.size());
      have_weights = true;
    } else if (p.get_name() == "q_terminal_diag") {
      const auto v = p.as_double_array();
      q_terminal_diag = Eigen::Map<const Eigen::VectorXd>(v.data(), v.size());
      have_weights = true;
    }
  }
  if (have_weights && solver_ && solver_->isInitialised()) {
    if (!solver_->setWeights(q_diag, r_diag, q_terminal_diag)) {
      result.successful = false;
      result.reason = "weight update rejected (negative weight or terminal < stage)";
      return result;
    }
  }

  // Trajectory params: rebuild and swap live (accepted only when the vehicle is safely in
  // Streaming/Tracking — the same gate as the service).
  TrajectoryParams tp = trajectory_params_;
  bool have_trajectory = false;
  for (const auto & p : params) {
    const std::string & n = p.get_name();
    if (n == "trajectory.type") {
      const std::string t = p.as_string();
      if (t == "hover") {tp.type = TrajectoryType::Hover;}
      else if (t == "figure8") {tp.type = TrajectoryType::Figure8;}
      else if (t == "lemniscate") {tp.type = TrajectoryType::Lemniscate;}
      else if (t == "circle") {tp.type = TrajectoryType::Circle;}
      else if (t == "waypoints") {tp.type = TrajectoryType::Waypoints;}
      else if (t == "step") {tp.type = TrajectoryType::Step;}
      else {
        result.successful = false;
        result.reason = "unknown trajectory.type '" + t + "'";
        return result;
      }
      have_trajectory = true;
    } else if (n == "trajectory.center") {
      const auto v = p.as_double_array();
      if (v.size() == 3) {tp.center = Eigen::Vector3d(v[0], v[1], v[2]);}
      have_trajectory = true;
    } else if (n == "trajectory.altitude") {tp.altitude = p.as_double(); have_trajectory = true;}
    else if (n == "trajectory.amplitude_x") {tp.amplitude_x = p.as_double(); have_trajectory = true;}
    else if (n == "trajectory.amplitude_y") {tp.amplitude_y = p.as_double(); have_trajectory = true;}
    else if (n == "trajectory.amplitude_z") {tp.amplitude_z = p.as_double(); have_trajectory = true;}
    else if (n == "trajectory.period") {tp.period = p.as_double(); have_trajectory = true;}
    else if (n == "trajectory.ramp_in_time") {tp.ramp_in_time = p.as_double(); have_trajectory = true;}
    else if (n == "trajectory.yaw_follows_velocity") {
      tp.yaw_follows_velocity = p.as_bool(); have_trajectory = true;
    } else if (n == "trajectory.fixed_yaw") {tp.fixed_yaw = p.as_double(); have_trajectory = true;}
    else if (n == "trajectory.max_velocity") {tp.max_velocity = p.as_double(); have_trajectory = true;}
    else if (n == "trajectory.max_acceleration") {
      tp.max_acceleration = p.as_double(); have_trajectory = true;
    }
  }
  if (have_trajectory && trajectory_) {
    const ControllerState s = controller_state_.load();
    if (s != ControllerState::Tracking && s != ControllerState::Streaming) {
      result.successful = false;
      result.reason = "trajectory changes are only accepted in Tracking or Streaming";
      return result;
    }
    try {
      std::lock_guard<std::mutex> lk(trajectory_mutex_);
      trajectory_->setParams(tp);
      trajectory_->generate();
      trajectory_params_ = tp;
    } catch (const std::exception & e) {
      result.successful = false;
      result.reason = std::string("trajectory rejected: ") + e.what();
      return result;
    }
    trajectory_start_time_ = this->now();
  }

  // Accept plain scalar/timing params that do not need reconfiguration.
  for (const auto & p : params) {
    if (p.get_name() == "control_rate_hz") {control_rate_hz_ = p.as_double();}
    else if (p.get_name() == "state_timeout_s") {state_timeout_s_ = p.as_double();}
    else if (p.get_name() == "latency_compensation_s") {latency_compensation_s_ = p.as_double();}
    else if (p.get_name() == "takeoff_altitude_m") {takeoff_altitude_m_ = p.as_double();}
    else if (p.get_name() == "px4_hover_thrust") {px4_hover_thrust_ = p.as_double();}
    else if (p.get_name() == "max_jump_on_switch_m") {max_jump_on_switch_m_ = p.as_double();}
    else if (p.get_name() == "publish_visualisation") {publish_visualisation_ = p.as_bool();}
    else if (p.get_name() == "log_solve_time_warn_ms") {log_solve_time_warn_ms_ = p.as_double();}
  }
  return result;
}

// ================================================================================================
// Subscriptions
// ================================================================================================

// [px4_msgs] VehicleLocalPosition is NED. The *_valid flags gate everything: PX4 publishes
// the message continuously, with some estimates invalid during startup.
void NmpcNode::onLocalPosition(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
  if (!msg->xy_valid || !msg->z_valid || !msg->v_xy_valid || !msg->v_z_valid) {
    return;  // incomplete estimate — keep the previous sample; staleness check will catch a gap
  }
  const Eigen::Vector3d p_ned(msg->x, msg->y, msg->z);
  const Eigen::Vector3d v_ned(msg->vx, msg->vy, msg->vz);
  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    position_enu_ = nedToEnu(p_ned);
    velocity_enu_ = nedToEnu(v_ned);
    position_valid_ = true;
    last_position_stamp_ = this->now();
  }
}

// [px4_msgs] VehicleAttitude::q is (w, x, y, z) in NED/FRD (§7.5).
void NmpcNode::onAttitude(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
{
  const Eigen::Quaterniond q_ned_frd(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    attitude_enu_flu_ = quatNedFrdToEnuFlu(q_ned_frd);
    last_attitude_stamp_ = this->now();
  }
}

// [px4_msgs] FRD -> FLU is (x, -y, -z) — body rates flip sign in the lateral axes.
void NmpcNode::onAngularVelocity(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr msg)
{
  const Eigen::Vector3d w_frd(msg->xyz[0], msg->xyz[1], msg->xyz[2]);
  const Eigen::Vector3d w_flu(w_frd.x(), -w_frd.y(), -w_frd.z());
  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    body_rates_flu_ = w_flu;
    last_rates_stamp_ = this->now();
  }
}

// [px4_msgs] Constants live on the message (ARMING_STATE_ARMED, NAVIGATION_STATE_OFFBOARD);
// verify against the pinned px4_msgs version — names have moved between releases.
void NmpcNode::onVehicleStatus(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
  armed_.store(msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
  const bool was_offboard = offboard_active_.load();
  offboard_active_.store(
    msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD);
  // Pilot took over while we were tracking: drop to Streaming and log loudly (§7.3).
  if (was_offboard && !offboard_active_.load() &&
      controller_state_.load() == ControllerState::Tracking) {
    RCLCPP_ERROR(get_logger(), "offboard lost while tracking — pilot has taken over");
    controller_state_.store(ControllerState::Streaming);
  }
}

// ================================================================================================
// Services
// ================================================================================================

// [px4_msgs] No px4_msgs types here — pure uav_mpc interfaces. Rejected unless the controller
// is in Tracking/Streaming and the new trajectory starts within max_jump_on_switch of the
// current position. Sending a HOVER request with altitude <= 0.5 m requests a landing (§7.3).
void NmpcNode::onSetTrajectory(
  const std::shared_ptr<uav_mpc::srv::SetTrajectory::Request> request,
  std::shared_ptr<uav_mpc::srv::SetTrajectory::Response> response)
{
  const auto & spec = request->spec;
  response->success = false;

  // --- validate -------------------------------------------------------------------------------
  TrajectoryParams tp;
  switch (spec.type) {
    case uav_mpc::msg::TrajectorySpec::TYPE_HOVER: tp.type = TrajectoryType::Hover; break;
    case uav_mpc::msg::TrajectorySpec::TYPE_FIGURE8: tp.type = TrajectoryType::Figure8; break;
    case uav_mpc::msg::TrajectorySpec::TYPE_LEMNISCATE: tp.type = TrajectoryType::Lemniscate; break;
    case uav_mpc::msg::TrajectorySpec::TYPE_CIRCLE: tp.type = TrajectoryType::Circle; break;
    case uav_mpc::msg::TrajectorySpec::TYPE_WAYPOINTS: tp.type = TrajectoryType::Waypoints; break;
    case uav_mpc::msg::TrajectorySpec::TYPE_STEP: tp.type = TrajectoryType::Step; break;
    default:
      response->message = "unknown trajectory type";
      return;
  }
  if (spec.type != uav_mpc::msg::TrajectorySpec::TYPE_HOVER &&
      spec.type != uav_mpc::msg::TrajectorySpec::TYPE_STEP && spec.period <= 0.0) {
    response->message = "period must be > 0 for periodic types";
    return;
  }
  if (spec.altitude < kMinAltitudeM || spec.altitude > kMaxAltitudeM) {
    response->message = "altitude outside [0.3, 10] m";
    return;
  }
  tp.center = toEigen(spec.center);
  tp.center.z() = spec.altitude;  // hover targets sit at `altitude`, not `center.z`
  tp.altitude = spec.altitude;
  tp.amplitude_x = spec.amplitude_x;
  tp.amplitude_y = spec.amplitude_y;
  tp.amplitude_z = spec.amplitude_z;
  tp.period = spec.period;
  tp.yaw_follows_velocity = spec.yaw_follows_velocity;
  tp.fixed_yaw = spec.fixed_yaw;
  tp.ramp_in_time = spec.ramp_in_time;
  tp.max_velocity = spec.max_velocity;
  tp.max_acceleration = spec.max_acceleration;
  for (const auto & w : spec.waypoints) {tp.waypoints.push_back(toEigen(w));}
  tp.segment_times.assign(spec.segment_times.begin(), spec.segment_times.end());

  // --- gate on controller state ----------------------------------------------------------------
  const ControllerState s = controller_state_.load();
  if (s != ControllerState::Tracking && s != ControllerState::Streaming) {
    response->message = "controller is not in Tracking or Streaming";
    return;
  }

  // --- jump guard: the new trajectory must start near the vehicle -------------------------------
  try {
    TrajectoryGenerator probe(tp);   // service callback: allocation is fine here
    probe.generate();
    const FlatState start = probe.sample(0.0);
    Eigen::Vector3d p;
    {
      std::lock_guard<std::mutex> lk(state_mutex_);
      p = position_enu_;
    }
    const double jump = (start.position - p).norm();
    if (jump > max_jump_on_switch_m_) {
      response->message = "trajectory starts " + std::to_string(jump) +
        " m from the vehicle (limit " + std::to_string(max_jump_on_switch_m_) + " m)";
      return;
    }
    // --- commit --------------------------------------------------------------------------------
    {
      std::lock_guard<std::mutex> lk(trajectory_mutex_);
      trajectory_->setParams(tp);
      trajectory_->generate();
      trajectory_params_ = tp;
    }
    if (request->restart_clock) {trajectory_start_time_ = this->now();}
    landing_requested_.store(
      spec.type == uav_mpc::msg::TrajectorySpec::TYPE_HOVER && spec.altitude <= 0.5);

    response->success = true;
    response->expected_duration = trajectory_->duration();
    std::string report;
    response->dynamically_feasible = trajectory_->isDynamicallyFeasible(airframe_, &report);
    if (!response->dynamically_feasible) {
      RCLCPP_WARN(get_logger(), "accepted trajectory is not dynamically feasible: %s",
        report.c_str());
    }
    RCLCPP_INFO(get_logger(), "trajectory switched to type=%d at (%.2f, %.2f, %.2f)",
      spec.type, tp.center.x(), tp.center.y(), tp.center.z());
  } catch (const std::exception & e) {
    response->message = std::string("trajectory rejected: ") + e.what();
  }
}

// ================================================================================================
// Control loop
// ================================================================================================

void NmpcNode::controlLoop()
{
  // --- 0. timing ----------------------------------------------------------------------------
  const rclcpp::Time now = this->now();
  loop_period_ms_ = (last_tick_time_.nanoseconds() == 0)
    ? 0.0 : (now - last_tick_time_).seconds() * 1e3;
  last_tick_time_ = now;
  const auto t_start = std::chrono::steady_clock::now();

  // --- 1. staleness check -> Failsafe ---------------------------------------------------------
  Eigen::VectorXd x0;
  std::string why;
  if (!assembleState(&x0, &why)) {
    enterFailsafe(why);
    publishOffboardControlMode();   // EVERY tick, unconditionally, even in Failsafe (§7.4)
    return;
  }
  last_x0_ = x0;

  // --- 2. state machine -----------------------------------------------------------------------
  updateControllerState();
  const ControllerState state = controller_state_.load();
  if (state == ControllerState::Idle) {return;}  // deactivated between ticks: stay silent

  // --- 3. latency compensation ----------------------------------------------------------------
  x0 = compensateLatency(x0, latency_compensation_s_);

  // --- 4. reference horizon --------------------------------------------------------------------
  const int n = solver_config_.horizon_steps;
  const double dt = solver_config_.horizon_time / static_cast<double>(n);
  if (state == ControllerState::Takeoff || state == ControllerState::Landing) {
    fillTakeoffLandingHorizon(state, x0, dt, n, hover_thrust_n_);
  } else {
    std::lock_guard<std::mutex> lk(trajectory_mutex_);
    const double t_traj = (now - trajectory_start_time_).seconds();
    const auto refs = trajectory_->referenceHorizon(
      t_traj, dt, n, airframe_, AttitudeRep::Quaternion);
    for (int k = 0; k <= n; ++k) {x_refs_[static_cast<std::size_t>(k)] = refs[k].state;}
    for (int k = 0; k < n; ++k) {u_refs_[static_cast<std::size_t>(k)] = refs[k].input;}
  }

  // --- 5. push into acados + solve ---------------------------------------------------------------
  solver_->setInitialState(x0);
  solver_->setReferenceHorizon(x_refs_, u_refs_);
  const SolveResult result = solver_->solve();

  // Failure accounting (§6.5): the wrapper tracks consecutive_failures; we own the Failsafe
  // decision on the last one.
  if (result.ok()) {
    consecutive_solver_failures_ = 0;
  } else {
    ++consecutive_solver_failures_;
    RCLCPP_WARN_THROTTLE(get_logger(), get_clock(), 1000,
      "solver failure #%d (status %d)", consecutive_solver_failures_,
      static_cast<int>(result.status));
    if (consecutive_solver_failures_ >= solver_config_.max_consecutive_failures) {
      enterFailsafe("max consecutive solver failures reached");
    }
  }

  // --- 6. map u0 -> attitude setpoint, publish ---------------------------------------------------
  if (result.ok() && state != ControllerState::Failsafe) {
    const Eigen::VectorXd u0 = solver_->optimalInput();
    last_applied_input_ = u0;
    const Eigen::VectorXd x_pred_1 = solver_->predictedState(1);
    pub_attitude_setpoint_->publish(toAttitudeSetpoint(u0, x_pred_1));
  } else {
    // Hold the previous input (spec §6.5): do not publish a fresh (possibly garbage) command.
    if (result.ok() && state == ControllerState::Failsafe) {
      // A successful solve while in Failsafe is the recovery condition (§7.3).
      controller_state_.store(ControllerState::Streaming);
      RCLCPP_INFO(get_logger(), "recovered from failsafe — back to streaming");
    }
  }

  // --- 7. telemetry -------------------------------------------------------------------------------
  publishOffboardControlMode();
  publishStatus(result, x0);
  publishVisualisation();

  // --- 8. budget -----------------------------------------------------------------------------------
  const auto t_end = std::chrono::steady_clock::now();
  loop_duration_ms_ = std::chrono::duration<double, std::milli>(t_end - t_start).count();
  if (loop_duration_ms_ > log_solve_time_warn_ms_) {
    RCLCPP_WARN_THROTTLE(get_logger(), get_clock(), 1000,
      "control tick took %.2f ms (warn threshold %.2f ms)", loop_duration_ms_,
      log_solve_time_warn_ms_);
  }
}

bool NmpcNode::assembleState(Eigen::VectorXd * x0, std::string * why_stale)
{
  using Layout = StateLayout<AttitudeRep::Quaternion>;
  const rclcpp::Time now = this->now();
  std::lock_guard<std::mutex> lk(state_mutex_);

  const double age_pos = (now - last_position_stamp_).seconds();
  const double age_att = (now - last_attitude_stamp_).seconds();
  const double age_rates = (now - last_rates_stamp_).seconds();
  state_age_ms_ = std::max({age_pos, age_att, age_rates}) * 1e3;

  if (!position_valid_ || age_pos > state_timeout_s_ || age_att > state_timeout_s_ ||
      age_rates > state_timeout_s_) {
    if (why_stale) {
      *why_stale = "state stale: pos_age=" + std::to_string(age_pos) +
        " att_age=" + std::to_string(age_att) + " rates_age=" + std::to_string(age_rates) +
        " s (timeout " + std::to_string(state_timeout_s_) + " s)";
    }
    return false;
  }

  Eigen::VectorXd x(Layout::kNx);
  x.segment<3>(Layout::kPosIdx) = position_enu_;
  x.segment<3>(Layout::kVelIdx) = velocity_enu_;
  x.segment<4>(Layout::kAttIdx) <<
    attitude_enu_flu_.w(), attitude_enu_flu_.x(), attitude_enu_flu_.y(), attitude_enu_flu_.z();
  x.segment<3>(Layout::kRateIdx) = body_rates_flu_;
  *x0 = std::move(x);
  return true;
}

Eigen::VectorXd NmpcNode::compensateLatency(
  const Eigen::VectorXd & x0, double latency_s) const
{
  using Dyn = QuadrotorDynamics<double, AttitudeRep::Quaternion>;
  if (latency_s <= 0.0 || last_applied_input_.size() != Dyn::kNu) {return x0;}
  const Dyn::StateVector xs = x0;            // fixed-size copy (Eigen asserts on mismatch)
  const Dyn::InputVector us = last_applied_input_;
  const Dyn::StateVector x_pred = model_->step(xs, us, latency_s);
  return x_pred;
}

// [px4_msgs] VehicleAttitudeSetpoint field names differ across PX4 releases (roll_body /
// pitch_body / yaw_body were removed in favour of q_d). Verify against the pinned px4_msgs
// message definition before trusting this — §7.5.
px4_msgs::msg::VehicleAttitudeSetpoint NmpcNode::toAttitudeSetpoint(
  const Eigen::VectorXd & u0, const Eigen::VectorXd & x_pred_1) const
{
  px4_msgs::msg::VehicleAttitudeSetpoint msg{};

  // Collective thrust [N] from the per-rotor command (PX4 quad-X allocation, §4.3).
  double T = 0.0;
  Eigen::Vector3d tau;
  model_->allocate(u0, &T, &tau);

  // The optimiser's intended attitude at stage 1 — already accounts for rate dynamics (§7.6).
  using Layout = StateLayout<AttitudeRep::Quaternion>;
  Eigen::Quaterniond q_d_enu(x_pred_1(Layout::kAttIdx), x_pred_1(Layout::kAttIdx + 1),
    x_pred_1(Layout::kAttIdx + 2), x_pred_1(Layout::kAttIdx + 3));
  q_d_enu.normalize();
  last_q_d_enu_ = q_d_enu;
  const Eigen::Quaterniond q_d_ned_frd = quatEnuFluToNedFrd(q_d_enu);

  // [px4_msgs] q_d is (w, x, y, z) floats.
  msg.q_d[0] = static_cast<float>(q_d_ned_frd.w());
  msg.q_d[1] = static_cast<float>(q_d_ned_frd.x());
  msg.q_d[2] = static_cast<float>(q_d_ned_frd.y());
  msg.q_d[3] = static_cast<float>(q_d_ned_frd.z());

  // PX4 is FRD: positive thrust on NEGATIVE z. thrust_body is normalised [0, 1].
  msg.thrust_body[0] = 0.0f;
  msg.thrust_body[1] = 0.0f;
  msg.thrust_body[2] = static_cast<float>(-normaliseThrust(T));

  // Predicted body-z rate at stage 1 (rates are the last three states).
  msg.yaw_sp_move_rate = static_cast<float>(x_pred_1(Layout::kRateIdx + 2));

  msg.timestamp = nowUs(*get_clock());
  return msg;
}

double NmpcNode::normaliseThrust(double collective_thrust_newton) const
{
  // Linear map: u = px4_hover_thrust * T / (m g). Valid while THR_MDL_FAC == 0
  // (config/px4_overrides.yaml). If a frame sets THR_MDL_FAC != 0, invert PX4's quadratic
  // thrust curve here instead (§7.6, §9.3).
  const double u = px4_hover_thrust_ * collective_thrust_newton /
    (airframe_.mass * airframe_.gravity);
  return std::clamp(u, 0.05, 0.95);
}

void NmpcNode::fillTakeoffLandingHorizon(
  ControllerState state, const Eigen::VectorXd & x0, double dt, int n, double hover_thrust_n)
{
  using Layout = StateLayout<AttitudeRep::Quaternion>;
  const double z_start = x0(Layout::kPosIdx + 2);
  const double z_end = (state == ControllerState::Takeoff)
    ? takeoff_altitude_m_ : kLandingZThresholdM;
  const double speed = (state == ControllerState::Takeoff)
    ? takeoff_speed_mps_ : landing_speed_mps_;

  for (int k = 0; k <= n; ++k) {
    const double t_k = static_cast<double>(k) * dt;
    double z_k = z_start + speed * t_k;
    if (state == ControllerState::Takeoff) {
      z_k = std::min(z_k, z_end);
    } else {
      z_k = std::max(z_k, z_end);
    }
    Eigen::VectorXd & xr = x_refs_[static_cast<std::size_t>(k)];
    xr.setZero();
    xr(Layout::kPosIdx) = x0(Layout::kPosIdx);          // keep current xy
    xr(Layout::kPosIdx + 1) = x0(Layout::kPosIdx + 1);
    xr(Layout::kPosIdx + 2) = z_k;
    xr(Layout::kAttIdx + 3) = 1.0;                      // level attitude, w = 1
    if (k < n) {
      u_refs_[static_cast<std::size_t>(k)].setConstant(hover_thrust_n);
    }
  }
}

// ================================================================================================
// PX4 handshake
// ================================================================================================

// [px4_msgs] OffboardControlMode: only the `attitude` channel is requested — the NMPC outputs
// attitude + normalised thrust, and PX4's attitude/rate loops close the inner loop.
void NmpcNode::publishOffboardControlMode()
{
  px4_msgs::msg::OffboardControlMode msg{};
  msg.timestamp = nowUs(*get_clock());
  msg.position = false;
  msg.velocity = false;
  msg.acceleration = false;
  msg.attitude = true;
  msg.body_rate = false;
  ++offboard_stream_counter_;
  pub_offboard_mode_->publish(msg);
}

// [px4_msgs] VehicleCommand: target/source system+component MUST all be 1 and from_external
// MUST be true or PX4 ignores the command (§7.5).
void NmpcNode::requestOffboardMode()
{
  if (offboard_stream_counter_ < kOffboardPreStreamCount) {return;}  // PX4 needs a warm stream
  if (offboard_active_.load()) {return;}

  px4_msgs::msg::VehicleCommand cmd{};
  cmd.timestamp = nowUs(*get_clock());
  cmd.target_system = 1;
  cmd.target_component = 1;
  cmd.source_system = 1;
  cmd.source_component = 1;
  cmd.from_external = true;
  cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
  cmd.param1 = 1.0f;  // main mode: OFFBOARD
  cmd.param2 = 6.0f;  // sub mode: offboard (mode 6)
  pub_command_->publish(cmd);
  RCLCPP_INFO_THROTTLE(get_logger(), get_clock(), 2000, "requesting OFFBOARD mode");
}

// [px4_msgs] VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, param1 = 1. No-op unless the
// auto_arm param is set (SITL/CI only — never auto-arm on real hardware).
void NmpcNode::requestArm()
{
  if (!auto_arm_) {return;}
  if (armed_.load()) {return;}

  px4_msgs::msg::VehicleCommand cmd{};
  cmd.timestamp = nowUs(*get_clock());
  cmd.target_system = 1;
  cmd.target_component = 1;
  cmd.source_system = 1;
  cmd.source_component = 1;
  cmd.from_external = true;
  cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
  cmd.param1 = 1.0f;  // arm
  pub_command_->publish(cmd);
  RCLCPP_INFO_THROTTLE(get_logger(), get_clock(), 2000, "requesting ARM (auto_arm)");
}

void NmpcNode::requestDisarm()
{
  if (!auto_arm_) {return;}

  px4_msgs::msg::VehicleCommand cmd{};
  cmd.timestamp = nowUs(*get_clock());
  cmd.target_system = 1;
  cmd.target_component = 1;
  cmd.source_system = 1;
  cmd.source_component = 1;
  cmd.from_external = true;
  cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
  cmd.param1 = 0.0f;  // disarm
  pub_command_->publish(cmd);
  RCLCPP_INFO(get_logger(), "requesting DISARM");
}

// ================================================================================================
// State machine
// ================================================================================================

void NmpcNode::updateControllerState()
{
  const ControllerState s = controller_state_.load();
  switch (s) {
    case ControllerState::Idle:
      return;  // timer should not be running, but be safe

    case ControllerState::Streaming: {
      requestOffboardMode();
      if (offboard_active_.load() && !armed_.load()) {requestArm();}
      if (offboard_active_.load() && armed_.load()) {
        controller_state_.store(ControllerState::Takeoff);
        RCLCPP_INFO(get_logger(), "armed + offboard: taking off to %.2f m",
          takeoff_altitude_m_);
      }
      break;
    }

    case ControllerState::Takeoff: {
      Eigen::Vector3d p, v;
      {
        std::lock_guard<std::mutex> lk(state_mutex_);
        p = position_enu_;
        v = velocity_enu_;
      }
      if (std::abs(p.z() - takeoff_altitude_m_) < kTakeoffZWindowM && v.norm() < kLandingVThresholdMps) {
        controller_state_.store(ControllerState::Tracking);
        RCLCPP_INFO(get_logger(), "takeoff complete — tracking the active trajectory");
      }
      break;
    }

    case ControllerState::Tracking: {
      if (landing_requested_.load()) {
        controller_state_.store(ControllerState::Landing);
        RCLCPP_INFO(get_logger(), "landing requested — descending at %.2f m/s",
          landing_speed_mps_);
      } else if (!offboard_active_.load()) {
        // handled loudly in onVehicleStatus; here just make sure we do not keep tracking.
        controller_state_.store(ControllerState::Streaming);
      }
      break;
    }

    case ControllerState::Landing: {
      Eigen::Vector3d p, v;
      {
        std::lock_guard<std::mutex> lk(state_mutex_);
        p = position_enu_;
        v = velocity_enu_;
      }
      if (p.z() < kLandingZThresholdM && v.norm() < kLandingVThresholdMps) {
        controller_state_.store(ControllerState::Idle);
        requestDisarm();
        RCLCPP_INFO(get_logger(), "landed — controller idle");
      }
      break;
    }

    case ControllerState::Failsafe:
      break;  // recovery is decided in controlLoop on the first successful solve
  }
}

void NmpcNode::enterFailsafe(const std::string & reason)
{
  controller_state_.store(ControllerState::Failsafe);
  RCLCPP_ERROR_THROTTLE(get_logger(), get_clock(), 1000, "FAILSAFE: %s", reason.c_str());

  // Reset the solver guess so a recovery solve has a sane starting point.
  if (solver_ && solver_->isInitialised() && last_x0_.size() == solver_->nx()) {
    solver_->resetToHover(last_x0_, hover_thrust_n_);
  }

  // Level attitude + hover thrust, straight to PX4 (bypasses the NMPC entirely).
  px4_msgs::msg::VehicleAttitudeSetpoint sp{};
  sp.q_d[0] = 1.0f;  // identity (level) in both ENU and NED
  sp.thrust_body[2] = static_cast<float>(-normaliseThrust(hover_thrust_n_));
  sp.timestamp = nowUs(*get_clock());
  pub_attitude_setpoint_->publish(sp);
}

// ================================================================================================
// Telemetry
// ================================================================================================

void NmpcNode::publishStatus(const SolveResult & result, const Eigen::VectorXd & x0)
{
  uav_mpc::msg::NmpcStatus msg{};
  msg.header.stamp = this->now();
  msg.header.frame_id = "map";

  using Layout = StateLayout<AttitudeRep::Quaternion>;
  msg.controller_state = static_cast<std::uint8_t>(controller_state_.load());
  msg.armed = armed_.load();
  msg.offboard_active = offboard_active_.load();

  // --- solver diagnostics ---------------------------------------------------------------------
  msg.solver.status = static_cast<std::uint8_t>(result.status);
  msg.solver.solve_time_ms = result.solve_time_ms;
  msg.solver.wall_time_ms = result.wall_time_ms;
  msg.solver.sqp_iterations = result.sqp_iterations;
  msg.solver.kkt_residual = result.kkt_residual;
  msg.solver.cost = result.cost;
  msg.solver.consecutive_failures = result.consecutive_failures;
  msg.solver.reinitialised = result.reinitialised;

  // --- tracking error vs the first reference stage, ENU ----------------------------------------
  if (!x_refs_.empty() && x0.size() >= 6) {
    const Eigen::Vector3d ref_p = x_refs_.front().segment<3>(Layout::kPosIdx);
    const Eigen::Vector3d ref_v = x_refs_.front().segment<3>(Layout::kVelIdx);
    const Eigen::Vector3d p_err = ref_p - x0.segment<3>(Layout::kPosIdx);
    const Eigen::Vector3d v_err = ref_v - x0.segment<3>(Layout::kVelIdx);
    msg.position_error = toVector3(p_err);
    msg.velocity_error = toVector3(v_err);
    msg.position_error_norm = p_err.norm();
    // Reference yaw from the reference quaternion (w,x,y,z) at stage 0.
    const double qw = x_refs_.front()(Layout::kAttIdx);
    const double qz = x_refs_.front()(Layout::kAttIdx + 3);
    const double ref_yaw = std::atan2(2.0 * (qw * qz), 1.0 - 2.0 * qz * qz);
    const Eigen::Quaterniond q_meas = attitude_enu_flu_;
    const double meas_yaw = std::atan2(
      2.0 * (q_meas.w() * q_meas.z()), 1.0 - 2.0 * q_meas.z() * q_meas.z());
    msg.yaw_error = wrapPi(ref_yaw - meas_yaw);
  }

  // --- applied command -------------------------------------------------------------------------
  if (last_applied_input_.size() == 4) {
    for (int i = 0; i < 4; ++i) {
      msg.rotor_thrust_setpoint[i] = last_applied_input_(i);
    }
    double T = 0.0;
    Eigen::Vector3d tau;
    model_->allocate(last_applied_input_, &T, &tau);
    msg.collective_thrust_newton = T;
    msg.normalised_thrust = normaliseThrust(T);
  }
  msg.attitude_setpoint.w = last_q_d_enu_.w();
  msg.attitude_setpoint.x = last_q_d_enu_.x();
  msg.attitude_setpoint.y = last_q_d_enu_.y();
  msg.attitude_setpoint.z = last_q_d_enu_.z();

  // --- timing -----------------------------------------------------------------------------------
  msg.loop_period_ms = loop_period_ms_;
  msg.loop_duration_ms = loop_duration_ms_;
  msg.state_age_ms = state_age_ms_;

  pub_status_->publish(msg);
}

void NmpcNode::publishVisualisation()
{
  if (!publish_visualisation_) {return;}
  if (pub_predicted_path_->get_subscription_count() == 0 &&
      pub_reference_path_->get_subscription_count() == 0) {
    return;  // nobody is watching; skip the allocation (§7.4)
  }

  using Layout = StateLayout<AttitudeRep::Quaternion>;
  const rclcpp::Time now = this->now();

  nav_msgs::msg::Path predicted;
  predicted.header.frame_id = "map";
  predicted.header.stamp = now;
  if (solver_ && solver_->isInitialised()) {
    const auto traj = solver_->predictedTrajectory();
    for (const auto & x : traj) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = predicted.header;
      ps.pose.position.x = x(Layout::kPosIdx);
      ps.pose.position.y = x(Layout::kPosIdx + 1);
      ps.pose.position.z = x(Layout::kPosIdx + 2);
      ps.pose.orientation.w = 1.0;
      predicted.poses.push_back(std::move(ps));
    }
  }
  pub_predicted_path_->publish(predicted);

  nav_msgs::msg::Path reference;
  reference.header.frame_id = "map";
  reference.header.stamp = now;
  for (const auto & x : x_refs_) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = reference.header;
    ps.pose.position.x = x(Layout::kPosIdx);
    ps.pose.position.y = x(Layout::kPosIdx + 1);
    ps.pose.position.z = x(Layout::kPosIdx + 2);
    ps.pose.orientation.w = 1.0;
    reference.poses.push_back(std::move(ps));
  }
  pub_reference_path_->publish(reference);
}

}  // namespace uav_mpc

RCLCPP_COMPONENTS_REGISTER_NODE(uav_mpc::NmpcNode)
