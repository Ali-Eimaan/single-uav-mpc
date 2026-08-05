"""SKELETON — the README demo. See .deepseek/08_LAUNCH.md §8.3.

Includes sitl.launch.py, then commands a figure-8 once the vehicle is hovering, and records a
rosbag of everything the analysis notebooks need.

    ros2 launch uav_mpc figure8.launch.py record:=true aggressive:=false
"""

from launch import LaunchDescription


def generate_launch_description() -> LaunchDescription:
    """Build the launch description.

    TODO(deepseek): implement.

    Declared arguments:
        aggressive   bool, default false — swaps in the 3 m / 5 s preset from
                     config/trajectory_params.yaml (~6 m/s, ~35 deg bank)
        record       bool, default false — start `ros2 bag record` on
                     /nmpc_node/status, /fmu/out/vehicle_local_position,
                     /fmu/out/vehicle_attitude, /fmu/in/vehicle_attitude_setpoint
                     into analysis/output/figure8_<timestamp>
        laps         default 3
        takeoff_wait default 12.0 [s] — time allowed for arm + takeoff before the figure-8
                     is commanded

    Sequencing:
        IncludeLaunchDescription(sitl.launch.py, launch_arguments={"trajectory": "hover",
                                                                   "auto_arm": "true"})
        TimerAction(takeoff_wait) -> ExecuteProcess calling
            `ros2 service call /nmpc_node/set_trajectory uav_mpc/srv/SetTrajectory "{...}"`
        Prefer an OpaqueFunction that waits for the controller to report STATE_TRACKING on
        ~/status over a blind timer — but keep the timer as a bounded fallback.
    """
    raise NotImplementedError("figure8.launch.py not implemented")
