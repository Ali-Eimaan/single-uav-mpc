# One acados solve. Mirrors uav_mpc::SolveResult.

uint8 STATUS_SUCCESS=0
uint8 STATUS_MAX_ITERATIONS=1
uint8 STATUS_MIN_STEP=2
uint8 STATUS_QP_FAILURE=3
uint8 STATUS_NAN=4
uint8 STATUS_NOT_INITIALISED=5
uint8 STATUS_TIMEOUT=6      # RETIRED — never published. A missed wall-clock deadline is
                            # reported by `deadline_missed` below, not as a status.
uint8 status

float64 solve_time_ms       # acados-reported CPU time
float64 wall_time_ms        # measured around the solve call
int32 sqp_iterations
float64 kkt_residual
float64 cost
int32 consecutive_failures
bool reinitialised          # solver guess was reset this tick

# Wall-clock deadline, reported INDEPENDENTLY of `status`.
#
# `wall_time_ms` measures the host (scheduler, page faults, a busy core), not acados. A late
# solve is still a valid solve and its command is applied; this flag exists so a sustained
# overrun is visible without ever being mistaken for a solver failure.
bool deadline_missed
int32 consecutive_deadline_misses
