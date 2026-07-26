// Copyright (c) 2026 Ali-Eimaan. MIT License.
//
// SKELETON — no implementation. See IMPLEMENTATION_GUIDE.md §7.8.
//
// Standalone entry point. The node is ALSO available as a composable component
// (RCLCPP_COMPONENTS_REGISTER_NODE in nmpc_node.cpp) so it can be loaded into a container
// alongside the micro-XRCE bridge for zero-copy intra-process transport.

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "uav_mpc/nmpc_node.hpp"

int main(int argc, char ** argv)
{
  // TODO(deepseek):
  //   rclcpp::init(argc, argv)
  //   NodeOptions with use_intra_process_comms(true)
  //   MultiThreadedExecutor with 2 threads (control group + telemetry group)
  //   add the node's base interface, spin, shutdown
  //   optionally: raise the control thread to SCHED_FIFO when `--rt-priority N` is passed and
  //   the process has CAP_SYS_NICE; log a warning and continue if it does not (§7.8).
  (void)argc;
  (void)argv;
  return 0;
}
