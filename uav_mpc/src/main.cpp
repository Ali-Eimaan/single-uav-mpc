// Copyright (c) 2026 Ali-Eimaan.
// SPDX-License-Identifier: BSD-3-Clause
//
// Standalone entry point. The node is ALSO available as a composable component
// (RCLCPP_COMPONENTS_REGISTER_NODE in nmpc_node.cpp) so it can be loaded into a container
// alongside the micro-XRCE bridge for zero-copy intra-process transport.
//
// §7.8: rclcpp::init -> NodeOptions(use_intra_process_comms=true) -> MultiThreadedExecutor
// with 2 threads (one for the MutuallyExclusive control group, one for the Reentrant
// telemetry group) -> spin -> shutdown. Optional --rt-priority N raises the process to
// SCHED_FIFO (needs CAP_SYS_NICE); failure is a warning, not a fatal error.
//
// VERIFICATION STATUS: syntax-verified against the .deepseek spec, NOT compile-verified —
// depends on nmpc_node.cpp which is gated on px4_msgs_FOUND (px4_msgs absent here).

#include <sched.h>

#include <cerrno>
#include <cstring>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "uav_mpc/nmpc_node.hpp"

namespace
{

/// Try to become a real-time process. Returns false (after logging) when the platform does
/// not allow it — the node keeps running best-effort rather than dying (§7.8).
bool tryRealtimePriority(int priority)
{
  struct sched_param param {};
  param.sched_priority = priority;
  if (sched_setscheduler(0, SCHED_FIFO, &param) != 0) {
    RCLCPP_WARN(rclcpp::get_logger("main"),
      "sched_setscheduler(SCHED_FIFO, %d) failed: %s — continuing without RT priority "
      "(run with sudo / set CAP_SYS_NICE on the binary to enable)",
      priority, std::strerror(errno));
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("main"), "raised to SCHED_FIFO priority %d", priority);
  return true;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // --rt-priority N is consumed here so the node itself never sees it.
  int rt_priority = 0;
  for (int i = 1; i < argc - 1; ++i) {
    if (std::string(argv[i]) == "--rt-priority") {
      try {
        rt_priority = std::stoi(argv[i + 1]);
      } catch (const std::exception &) {
        RCLCPP_WARN(rclcpp::get_logger("main"), "ignoring malformed --rt-priority value");
      }
      break;
    }
  }
  if (rt_priority > 0) {tryRealtimePriority(rt_priority);}

  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  auto node = std::make_shared<uav_mpc::NmpcNode>(options);

  // Two threads: the MutuallyExclusive control group can starve the Reentrant telemetry
  // group without blocking it (they never share a callback).
  // Lyrical Luth renamed ExecutorArgs to ExecutorOptions.
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 2);
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
