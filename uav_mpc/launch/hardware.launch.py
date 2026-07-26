"""SKELETON — real-hardware bring-up. See IMPLEMENTATION_GUIDE.md §8.4 and
docs/HARDWARE_BRINGUP.md.

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

from launch import LaunchDescription


def generate_launch_description() -> LaunchDescription:
    """Build the launch description.

    TODO(deepseek): implement.

    Declared arguments:
        backend          "px4" | "crazyflie", default "px4"
        airframe         default follows backend ("x500" / "crazyflie21")
        uri              px4: "serial:///dev/ttyUSB0:921600" or "udp://:14540"
                         crazyflie: "radio://0/80/2M/E7E7E7E7E7"
        mocap            bool, default false — start the mocap bridge and the external-vision
                         republisher (/fmu/in/vehicle_visual_odometry)
        mocap_topic      default "/vrpn_client_node/uav/pose"
        geofence_radius  default 3.0 [m]
        max_altitude     default 2.5 [m]
        auto_activate    default FALSE — see the safety note above

    Nodes (backend == "px4"):
        ExecuteProcess MicroXRCEAgent for the chosen transport (serial vs udp)
        IncludeLaunchDescription(nmpc_only.launch.py, auto_arm:=false, auto_activate:=false)
        optional mocap client + a small republisher converting PoseStamped ->
            px4_msgs/VehicleOdometry in NED

    Nodes (backend == "crazyflie"):
        crazyflie_ros2 server with the uri
        IncludeLaunchDescription(nmpc_only.launch.py, ...)
        the adapter node of §8.4 (NmpcStatus/attitude setpoint -> crazyflie setpoint)

    Always:
        a `preflight_check` action or ExecuteProcess that verifies battery voltage, EKF health
        and mocap age before allowing activation, and refuses otherwise.
    """
    raise NotImplementedError("hardware.launch.py not implemented")
