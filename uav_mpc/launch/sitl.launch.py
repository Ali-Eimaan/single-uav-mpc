"""The one-command demo. See .deepseek/08_LAUNCH.md §8.2.

Brings up, in dependency order:
    1. PX4 SITL + Gazebo Jetty (gz_x500 model) with config/px4_overrides.yaml applied
    2. MicroXRCEAgent udp4 -p 8888
    3. the NMPC lifecycle node (via nmpc_only.launch.py)
    4. RViz2 with rviz/nmpc.rviz

    ros2 launch uav_mpc sitl.launch.py world:=default headless:=false

This file is the "clone and fly in 10 minutes" promise from the README. If it is fragile,
the repo fails its primary purpose — prefer explicit checks and loud error messages over
clever automation.
"""

import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.actions import Shutdown
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.event_handlers import OnProcessStart

PACKAGE = "uav_mpc"


def _fail_immediately(px4_dir: str) -> None:
    """A stranger must be able to fix the failure from the message alone (§8.2)."""
    if not os.path.isdir(px4_dir):
        raise RuntimeError(
            f"PX4-Autopilot not found at '{px4_dir}'. Clone and build it first:\n"
            f"  git clone --recursive https://github.com/PX4/PX4-Autopilot.git {px4_dir}\n"
            f"  cd {px4_dir}\n"
            f"  make px4_sitl gz_x500\n"
            f"Then re-run this launch file. (Override with px4_dir:=<path> or "
            f"export PX4_DIR=<path>.)")
    build_dir = os.path.join(px4_dir, "build", "px4_sitl_default")
    px4_bin = os.path.join(build_dir, "bin", "px4")
    if not os.path.isfile(px4_bin):
        raise RuntimeError(
            f"PX4 SITL binary missing at '{px4_bin}'. Run `make px4_sitl gz_x500` inside "
            f"'{px4_dir}' first.")


def _apply_px4_overrides(context) -> list:
    """Write config/px4_overrides.yaml into the PX4 startup extras file (spec §8.2).

    The params are read by PX4 at boot, so this MUST run before the PX4 process starts.
    The file is the documented ROMFS extras for the pinned PX4 version; we write only the
    lines that are not already present, so repeated launches are idempotent.
    """
    import yaml

    px4_dir = context.launch_configurations["px4_dir"]
    share_dir = get_package_share_directory(PACKAGE)
    overrides_path = os.path.join(share_dir, "config", "px4_overrides.yaml")
    with open(overrides_path, "r", encoding="utf-8") as f:
        doc = yaml.safe_load(f)
    params = (doc or {}).get("px4_parameters", {})
    if not params:
        raise RuntimeError(
            f"config/px4_overrides.yaml has no 'px4_parameters:' section — refusing to "
            f"continue; the thrust map depends on these values.")

    candidates = [
        os.path.join(px4_dir, "ROMFS", "px4fmu_common", "init.d-posix", "px4-rc.simulator"),
        os.path.join(px4_dir, "ROMFS", "px4fmu_common", "init.d-posix", "rcS"),
    ]
    extras = next((p for p in candidates if os.path.isfile(p)), None)
    if extras is None:
        raise RuntimeError(
            f"could not find the PX4 startup extras file under '{px4_dir}' "
            f"(looked in {', '.join(candidates)}). Update launch/sitl.launch.py for the "
            f"pinned PX4 version.")

    with open(extras, "r", encoding="utf-8") as f:
        existing = f.read()
    marker = "# ---- uav_mpc overrides (config/px4_overrides.yaml) ----"
    if marker not in existing:
        lines = [marker]
        for name, value in params.items():
            lines.append(f"param set {name} {value}")
        lines.append("# ---- end uav_mpc overrides ----")
        with open(extras, "a", encoding="utf-8") as f:
            f.write("\n" + "\n".join(lines) + "\n")
        print(f"[sitl.launch.py] applied {len(params)} PX4 params from px4_overrides.yaml "
              f"to {extras}")
    else:
        print("[sitl.launch.py] PX4 overrides already applied — skipping")
    return []


def generate_launch_description() -> LaunchDescription:
    """Build the launch description."""
    share_dir = get_package_share_directory(PACKAGE)

    px4_dir_default = os.path.expanduser(os.environ.get("PX4_DIR", "~/PX4-Autopilot"))
    _fail_immediately(px4_dir_default)  # parse-time, before anything starts

    declared_arguments = [
        DeclareLaunchArgument("px4_dir", default_value=px4_dir_default,
                              description="path to the PX4-Autopilot checkout"),
        DeclareLaunchArgument("model", default_value="gz_x500"),
        DeclareLaunchArgument("world", default_value="default"),
        DeclareLaunchArgument("headless", default_value="false",
                              description="set HEADLESS=1 in the PX4 environment"),
        DeclareLaunchArgument("agent_port", default_value="8888"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("auto_arm", default_value="true",
                              description="SITL only — hardware.launch.py defaults to false"),
        DeclareLaunchArgument("trajectory", default_value="hover",
                              description="boot trajectory type, passed to the NMPC node"),
        DeclareLaunchArgument("airframe", default_value="x500"),
        DeclareLaunchArgument(
            "vehicle_interface", default_value="px4",
            description="backend for the NMPC node. SITL is PX4, so this defaults to 'px4', "
                        "which requires a workspace containing px4_msgs."),
    ]

    model = LaunchConfiguration("model")
    world = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")
    agent_port = LaunchConfiguration("agent_port")
    rviz_enabled = LaunchConfiguration("rviz")
    auto_arm = LaunchConfiguration("auto_arm")
    vehicle_interface = LaunchConfiguration("vehicle_interface")
    trajectory = LaunchConfiguration("trajectory")
    airframe = LaunchConfiguration("airframe")
    px4_dir = LaunchConfiguration("px4_dir")

    # ------------------------------------------------------------------ PX4 ---------------------
    # Apply the parameter overrides FIRST (an OpaqueFunction runs in order), then boot PX4;
    # the extras file is read at boot so ordering matters (§8.2).
    apply_overrides = OpaqueFunction(function=_apply_px4_overrides)

    px4 = ExecuteProcess(
        cmd=[os.path.join(px4_dir_default, "build", "px4_sitl_default", "bin", "px4")],
        additional_env={
            "PX4_SYS_AUTOSTART": "4001",
            "PX4_GZ_MODEL": model,
            "PX4_GZ_WORLD": world,
            "HEADLESS": PythonExpression(["'1' if '", headless, "' == 'true' else '0'"]),
        },
        cwd=os.path.join(px4_dir_default, "build", "px4_sitl_default"),
        name="px4",
        output="screen",
        emulate_tty=True,
    )

    # ------------------------------------------------------------------ uXRCE agent -------------
    # Deterministic: starts the moment the px4 process has spawned (not a blind sleep).
    agent = ExecuteProcess(
        cmd=["MicroXRCEAgent", "udp4", "-p", agent_port],
        name="microxrce_agent",
        output="screen",
        emulate_tty=True,
    )
    agent_after_px4 = RegisterEventHandler(
        OnProcessStart(target_action=px4, on_start=[agent]))

    # ------------------------------------------------------------------ NMPC node ---------------
    # Include after the agent is up. A bounded TimerAction is acceptable HERE (documented):
    # the agent prints "UDP server up" asynchronously and PX4 takes ~5 s to publish
    # vehicle_status; the NMPC node idles in Streaming until offboard + armed, so a slightly
    # early start is harmless — only the *agent* must be up before setpoints flow.
    nmpc_only = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share_dir, "launch", "nmpc_only.launch.py")),
        launch_arguments={
            "airframe": airframe,
            "auto_arm": auto_arm,
            "trajectory": trajectory,
            "use_sim_time": "true",
            # SITL IS PX4, so this stack needs the px4 backend. It is only available when the
            # workspace contains px4_msgs — on Lyrical Luth it does not, and the node will fail
            # on_configure with instructions. That failure is deliberate and honest: a PX4 SITL
            # run genuinely cannot work through the generic backend without an adapter.
            "vehicle_interface": vehicle_interface,
        },
    )
    nmpc_delayed = TimerAction(period=5.0, actions=[nmpc_only])

    # ------------------------------------------------------------------ RViz --------------------
    rviz = Node(
        package="rviz2", executable="rviz2",
        arguments=["-d", os.path.join(share_dir, "rviz", "nmpc.rviz")],
        output="screen",
        condition=IfCondition(rviz_enabled),
    )

    # ------------------------------------------------------------------ shutdown ----------------
    # If PX4 dies, tear the whole launch down — orphaned px4 processes hold UDP port 8888 and
    # the next launch fails mysteriously (§8.2). launch SIGINTs every process on Shutdown.
    px4_exit_shutdown = RegisterEventHandler(
        OnProcessExit(target_action=px4, on_exit=[Shutdown()]))

    return LaunchDescription(
        declared_arguments
        + [apply_overrides, px4, agent_after_px4, nmpc_delayed, rviz, px4_exit_shutdown]
    )
