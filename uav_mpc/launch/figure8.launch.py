"""The README demo. See .deepseek/08_LAUNCH.md §8.3.

Includes sitl.launch.py, then commands a figure-8 once the vehicle is hovering, and records a
rosbag of everything the analysis notebooks need.

    ros2 launch uav_mpc figure8.launch.py record:=true aggressive:=false

Sequencing: a TimerAction(takeoff_wait) is the bounded fallback; the primary trigger is an
OpaqueFunction that waits for the controller to report STATE_TRACKING on ~/status before
calling /nmpc_node/set_trajectory.
"""

import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource

PACKAGE = "uav_mpc"
STATE_TRACKING = 3  # uav_mpc/msg/NmpcStatus.controller_state


def _command_figure8(context) -> list:
    """Wait for STATE_TRACKING (bounded), then command the figure-8 via the service.

    Runs in the launch process; rclpy is already initialised by launch_ros, so we reuse the
    default context and destroy our node before returning.
    """
    import rclpy
    from rclpy.node import Node as RclpyNode

    cfg = context.launch_configurations
    aggressive = cfg.get("aggressive", "false") == "true"
    laps = int(cfg.get("laps", "3"))
    record = cfg.get("record", "false") == "true"
    takeoff_wait = float(cfg.get("takeoff_wait", "12.0"))

    amplitude = 3.0 if aggressive else 2.0
    period = 5.0 if aggressive else 8.0
    max_velocity = 6.0 if aggressive else 5.0
    max_acceleration = 9.0 if aggressive else 8.0

    if not rclpy.ok():
        rclpy.init(args=[])
    node = RclpyNode("figure8_commander")

    # --- wait for STATE_TRACKING, bounded by takeoff_wait -------------------------------------
    from uav_mpc.msg import NmpcStatus

    statuses = {"state": None}

    def _cb(msg):
        statuses["state"] = msg.controller_state

    sub = node.create_subscription(NmpcStatus, "/nmpc_node/status", _cb, 10)
    deadline = time.monotonic() + takeoff_wait
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if statuses["state"] == STATE_TRACKING:
            print(f"[figure8.launch.py] controller reports STATE_TRACKING after "
                  f"{takeoff_wait - (deadline - time.monotonic()):.1f}s")
            break
    else:
        print(f"[figure8.launch.py] WARNING: never saw STATE_TRACKING within {takeoff_wait}s — "
              f"commanding the figure-8 anyway (bounded fallback)")
    node.destroy_subscription(sub)

    # --- command the figure-8 ------------------------------------------------------------------
    from uav_mpc.srv import SetTrajectory

    spec = {
        "spec": {
            "type": 1,  # TYPE_FIGURE8
            "center": {"x": 0.0, "y": 0.0, "z": 0.0},
            "amplitude_x": amplitude,
            "amplitude_y": amplitude,
            "amplitude_z": 0.0,
            "period": period,
            "altitude": 1.5,
            "yaw_follows_velocity": True,
            "fixed_yaw": 0.0,
            "ramp_in_time": 3.0,
            "max_velocity": max_velocity,
            "max_acceleration": max_acceleration,
            "waypoints": [],
            "segment_times": [],
        },
        "restart_clock": True,
    }
    client = node.create_client(SetTrajectory, "/nmpc_node/set_trajectory")
    if not client.wait_for_service(timeout_sec=5.0):
        print("[figure8.launch.py] ERROR: /nmpc_node/set_trajectory service unavailable")
        node.destroy_node()
        return []
    req = SetTrajectory.Request()
    req.spec.type = spec["spec"]["type"]
    req.spec.center.x = 0.0
    req.spec.center.y = 0.0
    req.spec.center.z = 0.0
    req.spec.amplitude_x = amplitude
    req.spec.amplitude_y = amplitude
    req.spec.amplitude_z = 0.0
    req.spec.period = period
    req.spec.altitude = 1.5
    req.spec.yaw_follows_velocity = True
    req.spec.fixed_yaw = 0.0
    req.spec.ramp_in_time = 3.0
    req.spec.max_velocity = max_velocity
    req.spec.max_acceleration = max_acceleration
    req.restart_clock = True
    fut = client.call_async(req)
    while not fut.done():
        rclpy.spin_once(node, timeout_sec=0.1)
    resp = fut.result()
    node.destroy_node()

    if resp is None or not resp.success:
        print(f"[figure8.launch.py] ERROR: set_trajectory rejected: {resp.message if resp else 'no response'}")
        return []
    print(f"[figure8.launch.py] figure-8 commanded ({amplitude} m / {period} s lap, "
          f"expected duration {resp.expected_duration}s, "
          f"dynamically feasible: {resp.dynamically_feasible})")

    # --- optional bag recording ----------------------------------------------------------------
    if not record:
        return []
    out_dir = os.path.abspath(os.path.join(
        "analysis", "output", f"figure8_{time.strftime('%Y%m%d_%H%M%S')}"))
    duration = takeoff_wait + laps * period + 10.0
    bag = ExecuteProcess(
        cmd=[
            "ros2", "bag", "record", "-o", out_dir, "--duration", str(int(duration)),
            "/nmpc_node/status",
            "/nmpc_node/reference_path",   # §12.2: plot the reference actually given, not the analytic curve
            "/fmu/out/vehicle_local_position",
            "/fmu/out/vehicle_attitude",
            "/fmu/in/vehicle_attitude_setpoint",
        ],
        name="figure8_bag",
        output="screen",
    )
    print(f"[figure8.launch.py] recording {laps} laps (~{int(duration)}s) into {out_dir}")
    return [bag]


def generate_launch_description() -> LaunchDescription:
    """Build the launch description."""
    share_dir = get_package_share_directory(PACKAGE)

    declared_arguments = [
        DeclareLaunchArgument("aggressive", default_value="false",
                              description="3 m / 5 s preset (~6 m/s, ~35 deg bank)"),
        DeclareLaunchArgument("record", default_value="false",
                              description="ros2 bag record into analysis/output/figure8_<ts>"),
        DeclareLaunchArgument("laps", default_value="3",
                              description="figure-8 laps (used for the bag duration)"),
        DeclareLaunchArgument("takeoff_wait", default_value="12.0",
                              description="bounded fallback wait for arm + takeoff [s]"),
        DeclareLaunchArgument("headless", default_value="false",
                              description="forwarded to sitl.launch.py (HEADLESS=1 for PX4)"),
        DeclareLaunchArgument("rviz", default_value="false",
                              description="forwarded to sitl.launch.py (GUI off by default)"),
    ]

    sitl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share_dir, "launch", "sitl.launch.py")),
        launch_arguments={
            "trajectory": "hover",   # boot hovering; the figure-8 is commanded below
            "auto_arm": "true",
            "airframe": "x500",
            "headless": LaunchConfiguration("headless"),
            "rviz": LaunchConfiguration("rviz"),
        },
    )

    commander = OpaqueFunction(function=_command_figure8)

    return LaunchDescription(declared_arguments + [sitl, commander])
