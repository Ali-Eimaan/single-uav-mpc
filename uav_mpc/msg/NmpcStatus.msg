# Per-tick controller telemetry. Published on ~/status at the control rate.
# Everything here is what the analysis/ notebooks consume from a rosbag.

std_msgs/Header header

# --- controller state machine ---
uint8 STATE_IDLE=0
uint8 STATE_STREAMING=1
uint8 STATE_TAKEOFF=2
uint8 STATE_TRACKING=3
uint8 STATE_LANDING=4
uint8 STATE_FAILSAFE=5
uint8 controller_state

bool armed
bool offboard_active

# --- solver ---
SolverDiagnostics solver

# --- tracking error, ENU ---
geometry_msgs/Vector3 position_error      # reference - measured [m]
geometry_msgs/Vector3 velocity_error      # [m/s]
float64 yaw_error                         # wrapped to [-pi, pi] [rad]
float64 position_error_norm               # [m]

# --- applied command ---
float64[4] rotor_thrust_setpoint          # per-rotor [N], the solver's u0
float64 collective_thrust_newton          # [N]
float64 normalised_thrust                 # what PX4 received, [0, 1]
geometry_msgs/Quaternion attitude_setpoint  # ENU/FLU (converted to NED/FRD on the wire to PX4)

# --- timing ---
float64 loop_period_ms                    # measured wall time between ticks
float64 loop_duration_ms                  # measured wall time inside this tick
float64 state_age_ms                      # age of the newest odometry sample used
