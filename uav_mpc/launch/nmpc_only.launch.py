"""Launches ONLY the NMPC lifecycle node. See .deepseek/08_LAUNCH.md §8.1.

Assumes PX4 (SITL or hardware) and the uXRCE-DDS agent are already running. This is the
building block every other launch file includes.

    ros2 launch uav_mpc nmpc_only.launch.py airframe:=x500 auto_arm:=false

Lifecycle sequencing uses EVENTS (OnProcessStart -> CONFIGURE, OnStateTransition
inactive -> ACTIVATE), never TimerAction races — a timer that "usually works" is a race.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnProcessStart
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition
from rcl_interfaces.msg import ParameterType

PACKAGE = "uav_mpc"
NODE_NAME = "nmpc_node"


def generate_launch_description() -> LaunchDescription:
    """Build the launch description."""
    share_dir = get_package_share_directory(PACKAGE)

    # ------------------------------------------------------------------ arguments --------------
    declared_arguments = [
        DeclareLaunchArgument(
            "airframe", default_value="x500",
            description="'x500' | 'crazyflie21' -> selects params/<airframe>_calibration.yaml"),
        DeclareLaunchArgument(
            "nmpc_config",
            default_value=PathJoinSubstitution(
                [share_dir, "config", "nmpc_params.yaml"]),
            description="full path override for config/nmpc_params.yaml"),
        DeclareLaunchArgument(
            "trajectory_config",
            default_value=PathJoinSubstitution(
                [share_dir, "config", "trajectory_params.yaml"]),
            description="full path override for config/trajectory_params.yaml"),
        DeclareLaunchArgument(
            "namespace", default_value="",
            description="namespace for the node's OWN topics (PX4 /fmu/* topics stay absolute)"),
        DeclareLaunchArgument(
            "auto_arm", default_value="false",
            description="arm via VehicleCommand once offboard is active (SITL/CI only)"),
        DeclareLaunchArgument(
            "auto_activate", default_value="true",
            description="emit the lifecycle configure+activate transitions"),
        DeclareLaunchArgument("log_level", default_value="info",
                              description="ROS log level for the node"),
        DeclareLaunchArgument("use_sim_time", default_value="false",
                              description="use /clock for timestamps (SITL/bag replay)"),
        # Extension consumed by sitl.launch.py / figure8.launch.py: the trajectory TYPE the
        # node boots with (overrides `trajectory.type` inside trajectory_config).
        DeclareLaunchArgument(
            "trajectory", default_value="figure8",
            description="boot trajectory type: hover | figure8 | lemniscate | circle | "
                        "waypoints | step"),
    ]

    airframe = LaunchConfiguration("airframe")
    nmpc_config = LaunchConfiguration("nmpc_config")
    trajectory_config = LaunchConfiguration("trajectory_config")
    namespace = LaunchConfiguration("namespace")
    auto_arm = LaunchConfiguration("auto_arm")
    auto_activate = LaunchConfiguration("auto_activate")
    log_level = LaunchConfiguration("log_level")
    use_sim_time = LaunchConfiguration("use_sim_time")
    trajectory = LaunchConfiguration("trajectory")

    # ------------------------------------------------------------------ node -------------------
    node = LifecycleNode(
        package=PACKAGE,
        executable="nmpc_node",
        name=NODE_NAME,
        namespace=namespace,
        parameters=[
            nmpc_config,
            trajectory_config,
            {
                "airframe_params_path": PathJoinSubstitution(
                    [share_dir, "params", [airframe, "_calibration.yaml"]]),
                "auto_arm": ParameterValue(auto_arm, value_type=ParameterType.PARAMETER_BOOL),
                "use_sim_time": ParameterValue(
                    use_sim_time, value_type=ParameterType.PARAMETER_BOOL),
                "trajectory.type": trajectory,
            },
        ],
        arguments=["--ros-args", "--log-level", log_level],
        output="screen",
        emulate_tty=True,
    )

    # ------------------------------------------------------------------ lifecycle events -----
    # CONFIGURE as soon as the process is up; then ACTIVATE when it reports "inactive".
    # Event-driven, so there is no fixed sleep to get wrong (spec §8.1).
    configure_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=node,
            on_start=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(node),
                    transition_id=Transition.TRANSITION_CONFIGURE)),
            ]),
        condition=IfCondition(auto_activate),
    )

    activate_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            goal_state="inactive",
            entities=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(node),
                    transition_id=Transition.TRANSITION_ACTIVATE)),
            ]),
        condition=IfCondition(auto_activate),
    )

    return LaunchDescription(declared_arguments + [node, configure_handler, activate_handler])
