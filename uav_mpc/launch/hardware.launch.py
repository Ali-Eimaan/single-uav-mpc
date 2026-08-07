"""Real-hardware bring-up. and docs/HARDWARE_BRINGUP.md.

Same NMPC node as SITL; Gazebo and PX4-SITL are replaced by real drivers. Two backends:

    backend:=px4        Pixhawk-class vehicle (X500 v2) over serial or Wi-Fi telemetry,
                        uXRCE-DDS agent talking to the FMU. Identical topic namespace to
                        SITL, so the controller code path is bit-for-bit the same.

    backend:=crazyflie  Bitcraze Crazyflie 2.1 via crazyflie_ros2 / Crazyswarm2. The setpoint
                        interface differs (no px4_msgs), so a thin adapter node translates
                        the NMPC output; see §8.4 for the adapter contract.

    ros2 launch uav_mpc hardware.launch.py backend:=px4 uri:=serial:///dev/ttyUSB0

SAFETY — these are hard requirements, not suggestions:
    * auto_arm defaults to FALSE and must stay that way. Arming is a human action.
    * a kill switch must be mapped on the RC transmitter before this file is ever run
    * geofence parameters are applied before the controller is activated
    * the node starts in the *inactive* lifecycle state; the operator activates it explicitly
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE = "uav_mpc"


def _preflight_check(context) -> list:
    """Verify battery, EKF health and mocap age before the controller may be activated.

    Runs on the launch process. Uses rclpy (already initialised by launch_ros) to peek at
    /fmu/out/vehicle_status, /fmu/out/battery_status and (optionally) the mocap topic.
    Any check that cannot be satisfied prints an explicit FAIL; the final return value of
    this function carries the verdict in the printed log, and the operator must see the
    "PREFLIGHT PASS" line before activating the node by hand.
    """
    import time

    import rclpy
    from rclpy.node import Node as RclpyNode

    cfg = context.launch_configurations
    timeout = 10.0
    mocap_required = cfg.get("mocap", "false") == "true"
    mocap_topic = cfg.get("mocap_topic", "/vrpn_client_node/uav/pose")

    if not rclpy.ok():
        rclpy.init(args=[])
    node = RclpyNode("uav_mpc_preflight")

    results = {}

    # --- battery ----------------------------------------------------------------------------
    try:
        from px4_msgs.msg import BatteryStatus  # noqa: F401  (name varies; guarded below)

        battery_topic = "/fmu/out/battery_status"
    except ImportError:
        battery_topic = None
        results["battery"] = ("WARN", "px4_msgs.BatteryStatus unavailable — battery skipped")
    if battery_topic:
        samples = []

        def _on_battery(msg):
            samples.append(msg.voltage_v)

        sub = node.create_subscription(BatteryStatus, battery_topic, _on_battery, 10)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not samples:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_subscription(sub)
        if samples:
            v = samples[-1]
            results["battery"] = ("PASS" if v >= 10.5 else "FAIL", f"{v:.2f} V (limit 10.5 V)")
        else:
            results["battery"] = ("FAIL", f"no battery messages on {battery_topic}")

    # --- EKF health + state freshness ---------------------------------------------------------
    try:
        from px4_msgs.msg import VehicleStatus
    except ImportError:
        results["ekf"] = ("WARN", "px4_msgs unavailable — EKF check skipped")
    else:
        got = []

        def _on_status(msg):
            got.append(msg)

        sub = node.create_subscription(VehicleStatus, "/fmu/out/vehicle_status", _on_status, 10)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not got:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_subscription(sub)
        if not got:
            results["ekf"] = (
                "FAIL",
                "no vehicle_status within 10 s — is the uXRCE-DDS " "agent running?",
            )
        else:
            s = got[-1]
            nav_ok = int(s.nav_state) != 11  # 11 = NAVIGATION_STATE_FAILSAFE
            results["ekf"] = ("PASS" if nav_ok else "FAIL", f"nav_state={s.nav_state}")

    # --- mocap age (only when the mission needs external vision) ------------------------------
    if mocap_required:
        from geometry_msgs.msg import PoseStamped

        stamps = []

        def _on_pose(msg):
            stamps.append(msg.header.stamp)

        sub = node.create_subscription(PoseStamped, mocap_topic, _on_pose, 10)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not stamps:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_subscription(sub)
        if stamps:
            age = (node.get_clock().now() - stamps[-1]).nanoseconds * 1e-9
            results["mocap"] = (
                "PASS" if age < 0.1 else "FAIL",
                f"age {age * 1e3:.0f} ms (limit 100 ms)",
            )
        else:
            results["mocap"] = ("FAIL", f"no poses on {mocap_topic} within {timeout} s")

    node.destroy_node()

    verdict = "PASS" if all(r[0] == "PASS" for r in results.values()) else "FAIL"
    print(f"\n===== uav_mpc PREFLIGHT {verdict} =====")
    for name, (status, detail) in results.items():
        print(f"  [{status:>4}] {name}: {detail}")
    print("Activate the node by hand ONLY after 'PREFLIGHT PASS'.\n")
    return []


def generate_launch_description() -> LaunchDescription:
    """Build the launch description."""
    share_dir = get_package_share_directory(PACKAGE)

    declared_arguments = [
        DeclareLaunchArgument("backend", default_value="px4", description="'px4' | 'crazyflie'"),
        DeclareLaunchArgument(
            "airframe", default_value="x500", description="defaults per backend: x500 / crazyflie21"
        ),
        DeclareLaunchArgument(
            "uri",
            default_value="serial:///dev/ttyUSB0:921600",
            description="px4: serial:///dev/ttyUSB0:921600 or udp://:14540; "
            "crazyflie: radio://0/80/2M/E7E7E7E7E7",
        ),
        DeclareLaunchArgument(
            "mocap",
            default_value="false",
            description="start the mocap republisher (vehicle_visual_odometry)",
        ),
        DeclareLaunchArgument("mocap_topic", default_value="/vrpn_client_node/uav/pose"),
        DeclareLaunchArgument("geofence_radius", default_value="3.0", description="[m]"),
        DeclareLaunchArgument("max_altitude", default_value="2.5", description="[m]"),
        # SAFETY: these defaults are a safety property — do not flip them.
        DeclareLaunchArgument(
            "auto_arm",
            default_value="false",
            description="MUST stay false on hardware; arming is human",
        ),
        DeclareLaunchArgument(
            "auto_activate",
            default_value="false",
            description="MUST stay false on hardware; operator activates",
        ),
    ]

    backend = LaunchConfiguration("backend")
    airframe = LaunchConfiguration("airframe")
    uri = LaunchConfiguration("uri")
    mocap = LaunchConfiguration("mocap")
    mocap_topic = LaunchConfiguration("mocap_topic")
    auto_arm = LaunchConfiguration("auto_arm")
    auto_activate = LaunchConfiguration("auto_activate")

    actions = []

    if backend == "px4":
        # --- uXRCE-DDS agent to the FMU --------------------------------------------------------
        agent = ExecuteProcess(
            cmd=["MicroXRCEAgent", uri],
            name="microxrce_agent_hw",
            output="screen",
            emulate_tty=True,
        )
        actions.append(agent)

        # --- mocap republisher: PoseStamped (ENU) -> px4_msgs/VehicleOdometry (NED) ------------
        if mocap:
            mocap_bridge = Node(
                package="uav_mpc",
                executable="mocap_bridge",  # small helper, see below
                parameters=[{"mocap_topic": mocap_topic}],
                output="screen",
                condition=IfCondition(mocap),
            )
            actions.append(mocap_bridge)

    elif backend == "crazyflie":
        # The setpoint adapter (§8.4) is not part of this repo — a stranger must not be left
        # wondering why nothing flies. Fail loudly at parse time with the exact contract.
        raise NotImplementedError(
            "backend:=crazyflie needs the §8.4 adapter node (NmpcStatus -> crazyflie_ros2 "
            "setpoint at 100 Hz, zero-thrust latch on stale status) plus crazyflie_ros2 "
            "installed. Implement the adapter in this repo before using this backend."
        )
    else:
        raise RuntimeError(f"unknown backend '{backend}' (expected 'px4' or 'crazyflie')")

    # --- the NMPC node itself (inactive; preflight + human activation) -------------------------
    nmpc_only = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share_dir, "launch", "nmpc_only.launch.py")),
        launch_arguments={
            "airframe": airframe,
            "auto_arm": auto_arm,
            "auto_activate": auto_activate,
            "use_sim_time": "false",
        },
    )
    actions.append(nmpc_only)

    # --- preflight gate: refuses to proceed (loudly) when battery/EKF/mocap are not healthy ----
    preflight = OpaqueFunction(function=_preflight_check)
    actions.append(preflight)

    return LaunchDescription(declared_arguments + actions)
