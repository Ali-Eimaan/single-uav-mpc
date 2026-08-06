# Frame-neutral attitude + thrust command produced by the NMPC.
#
# This is the GENERIC output of the controller — it carries no autopilot-specific types, so it
# works without px4_msgs. The PX4 backend converts this into px4_msgs/VehicleAttitudeSetpoint
# internally; the generic backend publishes it as-is for any adapter to consume.
#
# Conventions (see .deepseek/16_CONVENTIONS.md):
#   attitude  — world ENU, body FLU, Hamilton (w, x, y, z). NOT NED/FRD.
#   thrust    — newtons in `collective_thrust_newton`; the normalised value is provided too so
#               an adapter does not have to know the airframe mass.

std_msgs/Header header

# Desired attitude, world-ENU / body-FLU.
geometry_msgs/Quaternion attitude

# Collective thrust along the body z-axis.
float64 collective_thrust_newton   # [N] physical, airframe-independent
float64 normalised_thrust          # [0, 1] using the configured hover-thrust calibration

# Feed-forward yaw rate about the body z-axis, from the optimiser's stage-1 prediction.
float64 yaw_rate_feedforward       # [rad/s]

# Per-rotor thrusts the solver actually produced, before allocation to an attitude command.
# Present so a direct-actuator adapter can bypass the attitude loop entirely.
float64[4] rotor_thrust_newton     # [N]
