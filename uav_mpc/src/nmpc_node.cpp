// Copyright (c) 2026 Ali-Eimaan. MIT License.
//
// SKELETON — no implementation. See IMPLEMENTATION_GUIDE.md §7.

#include "uav_mpc/nmpc_node.hpp"

#include <memory>
#include <string>
#include <vector>

#include "rclcpp_components/register_node_macro.hpp"

namespace uav_mpc
{

NmpcNode::NmpcNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("nmpc_node", options)
{
  // TODO(deepseek): declareParameters() only. No allocation of the solver here — that belongs
  // in on_configure so the node can be re-configured without a restart.
}

NmpcNode::~NmpcNode() = default;

// ================================================================================================
// Lifecycle
// ================================================================================================

CallbackReturn NmpcNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  // TODO(deepseek):
  //   loadParameters(); airframe_ = QuadrotorParams::fromYaml(airframe_params_path_)
  //   model_ = make_unique<QuadrotorDynamics<double, Quaternion>>(airframe_)
  //   trajectory_ = make_unique<TrajectoryGenerator>(params from config/trajectory_params.yaml)
  //   solver_ = make_unique<AcadosWrapper>(); solver_->initialise(solver_config_, &err)
  //   create subscriptions with the PX4 QoS profile (§7.2: BestEffort, KeepLast(5), Volatile)
  //   create publishers (still inactive), the SetTrajectory service, the parameter callback
  // Return FAILURE (not ERROR) with a clear RCLCPP_ERROR on any invalid config.
  return CallbackReturn::FAILURE;
}

CallbackReturn NmpcNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  // TODO(deepseek): LifecycleNode::on_activate(state) for the base publishers, then create the
  // control timer at control_rate_hz_ on control_callback_group_, reset the state machine to
  // Streaming, zero the counters, and record trajectory_start_time_.
  return CallbackReturn::FAILURE;
}

CallbackReturn NmpcNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  // TODO(deepseek): cancel + reset the timer FIRST (so no callback runs against dead
  // publishers), then deactivate publishers, then set controller_state_ = Idle.
  return CallbackReturn::FAILURE;
}

CallbackReturn NmpcNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  // TODO(deepseek): reset solver_, trajectory_, model_, all pubs/subs/services.
  return CallbackReturn::FAILURE;
}

CallbackReturn NmpcNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  // TODO(deepseek): best-effort — stop the timer, stop publishing. Do NOT disarm here;
  // PX4's own failsafe handles offboard signal loss and is the safer authority.
  return CallbackReturn::FAILURE;
}

// ================================================================================================
// Parameters
// ================================================================================================

void NmpcNode::declareParameters()
{
  // TODO(deepseek): declare every key of config/nmpc_params.yaml with a
  // rcl_interfaces::msg::ParameterDescriptor: description, floating-point range, and
  // read_only = true for the structural ones (horizon_steps, horizon_time, attitude_rep).
}

bool NmpcNode::loadParameters(std::string * /*error*/)
{
  // TODO(deepseek): read into the members; validate control_rate_hz_ > 0,
  // state_timeout_s_ >= 2 / control_rate_hz_, weight vector lengths == nx/nu.
  return false;
}

rcl_interfaces::msg::SetParametersResult NmpcNode::onParameterUpdate(
  const std::vector<rclcpp::Parameter> & /*params*/)
{
  // TODO(deepseek): accept weight + trajectory changes live (call solver_->setWeights() /
  // trajectory_->setParams()); reject structural ones with a reason string.
  return rcl_interfaces::msg::SetParametersResult{};
}

// ================================================================================================
// Subscriptions
// ================================================================================================

void NmpcNode::onLocalPosition(const px4_msgs::msg::VehicleLocalPosition::SharedPtr /*msg*/)
{
  // TODO(deepseek): reject unless msg->xy_valid && msg->z_valid && msg->v_xy_valid &&
  // msg->v_z_valid. Convert NED -> ENU, take the lock, store, stamp with now().
}

void NmpcNode::onAttitude(const px4_msgs::msg::VehicleAttitude::SharedPtr /*msg*/)
{
  // TODO(deepseek): msg->q is (w, x, y, z) NED/FRD -> quatNedFrdToEnuFlu().
}

void NmpcNode::onAngularVelocity(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr /*msg*/)
{
  // TODO(deepseek): FRD -> FLU is (x, -y, -z).
}

void NmpcNode::onVehicleStatus(const px4_msgs::msg::VehicleStatus::SharedPtr /*msg*/)
{
  // TODO(deepseek): armed_ = (arming_state == ARMING_STATE_ARMED);
  // offboard_active_ = (nav_state == NAVIGATION_STATE_OFFBOARD). If offboard was active and
  // is not any more while we are in Tracking, log an error and drop to Streaming — the pilot
  // has taken over.
}

// ================================================================================================
// Services
// ================================================================================================

void NmpcNode::onSetTrajectory(
  const std::shared_ptr<uav_mpc::srv::SetTrajectory::Request> /*request*/,
  std::shared_ptr<uav_mpc::srv::SetTrajectory::Response> /*response*/)
{
  // TODO(deepseek): validate the request (known type, period > 0, altitude within
  // [0.3, 10] m), check the first sample is within `max_jump_on_switch` of the current
  // position, then swap under the trajectory mutex and reset trajectory_start_time_.
}

// ================================================================================================
// Control loop
// ================================================================================================

void NmpcNode::controlLoop()
{
  // TODO(deepseek): the fixed six-step ordering documented in nmpc_node.hpp and §7.4.
  // Hard requirements:
  //   - publishOffboardControlMode() runs on EVERY tick, even in Failsafe, even before arming
  //   - no dynamic allocation after the first tick (pre-size every buffer in on_configure)
  //   - total callback budget 10 ms at 100 Hz; log a throttled warning above 5 ms
}

bool NmpcNode::assembleState(Eigen::VectorXd * /*x0*/, std::string * /*why_stale*/)
{
  // TODO(deepseek): lock state_mutex_, check each stamp against state_timeout_s_, pack
  // [p_enu, v_enu, q_enu_flu(w,x,y,z), omega_flu].
  return false;
}

Eigen::VectorXd NmpcNode::compensateLatency(
  const Eigen::VectorXd & /*x0*/, double /*latency_s*/) const
{
  // TODO(deepseek): model_->step(x0, last_applied_input_, latency_s). Skip when latency_s <= 0
  // or last_applied_input_ is empty (first tick).
  return Eigen::VectorXd{};
}

px4_msgs::msg::VehicleAttitudeSetpoint NmpcNode::toAttitudeSetpoint(
  const Eigen::VectorXd & /*u0*/, const Eigen::VectorXd & /*x_pred_1*/) const
{
  // TODO(deepseek): §7.6.
  //   collective thrust from model_->allocate(u0, ...)
  //   desired attitude = the quaternion slice of x_pred_1 (already the optimiser's intent),
  //     converted ENU/FLU -> NED/FRD
  //   thrust_body = {0, 0, -normaliseThrust(T)}   (PX4 wants a NEGATIVE z thrust)
  //   yaw_sp_move_rate from the predicted yaw rate
  //   timestamp in MICROSECONDS from get_clock()->now().nanoseconds() / 1000
  return px4_msgs::msg::VehicleAttitudeSetpoint{};
}

double NmpcNode::normaliseThrust(double /*collective_thrust_newton*/) const
{
  // TODO(deepseek): u_norm = px4_hover_thrust_ * T / (m * g), clamped to [0.05, 0.95].
  // The linear map is correct for PX4's default thrust curve when THR_MDL_FAC = 0; if the
  // airframe sets THR_MDL_FAC != 0, invert the quadratic instead (see config/px4_overrides.yaml).
  return 0.0;
}

// ================================================================================================
// PX4 handshake
// ================================================================================================

void NmpcNode::publishOffboardControlMode()
{
  // TODO(deepseek): position/velocity/acceleration = false, attitude = true, body_rate = false.
  // Timestamp in microseconds.
}

void NmpcNode::requestOffboardMode()
{
  // TODO(deepseek): VehicleCommand VEHICLE_CMD_DO_SET_MODE, param1 = 1, param2 = 6.
  // Only after >= 20 OffboardControlMode messages have been streamed (offboard_stream_counter_).
}

void NmpcNode::requestArm()
{
  // TODO(deepseek): VEHICLE_CMD_COMPONENT_ARM_DISARM, param1 = 1. No-op unless auto_arm_.
}

void NmpcNode::requestDisarm()
{
  // TODO(deepseek): param1 = 0. Only from Landing with |z| < 0.15 m and |v| < 0.2 m/s.
}

// ================================================================================================
// State machine
// ================================================================================================

void NmpcNode::updateControllerState()
{
  // TODO(deepseek): the transition table of §7.3.
}

void NmpcNode::enterFailsafe(const std::string & /*reason*/)
{
  // TODO(deepseek): set the state, log RCLCPP_ERROR_THROTTLE (1 s), publish a level-attitude
  // setpoint at hover thrust, and reset the solver guess so a recovery is possible.
}

// ================================================================================================
// Telemetry
// ================================================================================================

void NmpcNode::publishStatus(const SolveResult & /*result*/, const Eigen::VectorXd & /*x0*/)
{
  // TODO(deepseek): fill NmpcStatus (solve time, status, iterations, cost, position and yaw
  // error against the current reference, controller state as a string).
}

void NmpcNode::publishVisualisation()
{
  // TODO(deepseek): predicted + reference nav_msgs::Path in frame "map" (ENU). Skip entirely
  // when publish_visualisation_ is false or nobody is subscribed
  // (count_subscribers()/get_subscription_count()).
}

}  // namespace uav_mpc

RCLCPP_COMPONENTS_REGISTER_NODE(uav_mpc::NmpcNode)
