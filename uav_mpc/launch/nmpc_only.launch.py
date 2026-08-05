"""SKELETON — launches ONLY the NMPC lifecycle node. See .deepseek/08_LAUNCH.md §8.1.

Assumes PX4 (SITL or hardware) and the uXRCE-DDS agent are already running. This is the
building block every other launch file includes.

    ros2 launch uav_mpc nmpc_only.launch.py airframe:=x500 auto_arm:=false
"""

from launch import LaunchDescription


def generate_launch_description() -> LaunchDescription:
    """Build the launch description.

    TODO(deepseek): implement.

    Declared arguments (all with defaults, all documented):
        airframe         "x500" | "crazyflie21"  -> selects params/<airframe>_calibration.yaml
        nmpc_config      full path override for config/nmpc_params.yaml
        trajectory_config full path override for config/trajectory_params.yaml
        namespace        default "" (PX4 topics are absolute either way)
        auto_arm         bool, default false
        auto_activate    bool, default true — emit the lifecycle configure+activate transitions
        log_level        default "info"
        use_sim_time     bool, default false

    Nodes:
        LifecycleNode(package="uav_mpc", executable="nmpc_node", name="nmpc_node")
          parameters = [nmpc_config, trajectory_config,
                        {"airframe_params_path": <resolved>, "auto_arm": ..., "use_sim_time": ...}]
          output = "screen", emulate_tty = True

    Lifecycle sequencing (only when auto_activate):
        EmitEvent(ChangeState -> TRANSITION_CONFIGURE) on ProcessStart of the node, then
        RegisterEventHandler(OnStateTransition: inactive -> EmitEvent(TRANSITION_ACTIVATE)).
        Do NOT use a TimerAction to fake the ordering — the event handler is the correct tool
        and a reviewer will notice the difference.
    """
    raise NotImplementedError("nmpc_only.launch.py not implemented")
