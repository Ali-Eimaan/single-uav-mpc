# Declarative description of a reference trajectory. Shared by srv/SetTrajectory.srv and
# action/FollowTrajectory.action, and mirrors uav_mpc::TrajectoryParams 1:1.

uint8 TYPE_HOVER=0
uint8 TYPE_FIGURE8=1
uint8 TYPE_LEMNISCATE=2
uint8 TYPE_CIRCLE=3
uint8 TYPE_WAYPOINTS=4
uint8 TYPE_STEP=5
uint8 type

geometry_msgs/Point center
float64 amplitude_x              # [m]
float64 amplitude_y              # [m]
float64 amplitude_z              # [m] vertical bob
float64 period                   # [s] one lap; must be > 0 for periodic types
float64 altitude                 # [m] AGL
bool yaw_follows_velocity
float64 fixed_yaw                # [rad], used when yaw_follows_velocity is false
float64 ramp_in_time             # [s] C^4 blend from hover onto the orbit
float64 max_velocity             # [m/s] feasibility bound + time allocation
float64 max_acceleration         # [m/s^2]
geometry_msgs/Point[] waypoints  # TYPE_WAYPOINTS only
float64[] segment_times          # [s] optional; empty => heuristic allocation
