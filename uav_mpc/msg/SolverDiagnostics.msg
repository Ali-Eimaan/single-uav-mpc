# One acados solve. Mirrors uav_mpc::SolveResult.

uint8 STATUS_SUCCESS=0
uint8 STATUS_MAX_ITERATIONS=1
uint8 STATUS_MIN_STEP=2
uint8 STATUS_QP_FAILURE=3
uint8 STATUS_NAN=4
uint8 STATUS_NOT_INITIALISED=5
uint8 STATUS_TIMEOUT=6
uint8 status

float64 solve_time_ms       # acados-reported CPU time
float64 wall_time_ms        # measured around the solve call
int32 sqp_iterations
float64 kkt_residual
float64 cost
int32 consecutive_failures
bool reinitialised          # solver guess was reset this tick
