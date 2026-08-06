// Copyright (c) 2026 Ali-Eimaan.
// SPDX-License-Identifier: BSD-3-Clause
//
// SKELETON — declarations only. See .deepseek/06_SOLVER.md §6.
//
// RAII wrapper around the acados-generated OCP solver. Owns the capsule, the nlp config, dims,
// in/out structs and the opts. Nothing outside this file may include acados headers, so the
// rest of the package stays compilable (and unit-testable) without acados installed —
// build the package with -DUAV_MPC_WITH_ACADOS=OFF to get the stub backend.

#ifndef UAV_MPC__ACADOS_WRAPPER_HPP_
#define UAV_MPC__ACADOS_WRAPPER_HPP_

#include <Eigen/Dense>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace uav_mpc
{

/// Return status of one solve. Mirrors the acados ACADOS_* codes, plus wrapper-level states.
enum class SolverStatus
{
  Success = 0,
  MaxIterations,
  MinStepLength,
  QpFailure,       ///< QP subproblem infeasible / HPIPM failure
  NanDetected,
  NotInitialised,
  Timeout          ///< exceeded the wall-clock budget enforced by the wrapper
};

/// Everything nmpc_node needs to know about a solve, for logging and for /nmpc/status.
struct SolveResult
{
  SolverStatus status{SolverStatus::NotInitialised};
  double solve_time_ms{0.0};      ///< acados-reported total CPU time
  double wall_time_ms{0.0};       ///< measured around the solve() call
  int sqp_iterations{0};
  double kkt_residual{0.0};
  double cost{0.0};
  int consecutive_failures{0};    ///< reset on the first Success
  bool reinitialised{false};      ///< true if the wrapper re-initialised the solver this tick

  bool ok() const {return status == SolverStatus::Success;}
};

/// Runtime-tunable solver settings. Anything not listed here is baked into the generated code
/// by codegen/generate_acados_solver.py and requires a rebuild to change.
struct SolverConfig
{
  int horizon_steps{20};                ///< N — must equal the codegen value; checked at init
  double horizon_time{1.0};             ///< Tf [s] — must equal the codegen value
  // Weights apply to the COST RESIDUAL, not to the raw state: the attitude enters as a
  // 3-vector error, so the state residual is 12-dimensional even when nx == 13.
  Eigen::VectorXd q_diag{};             ///< 12 stage weights [p(3) v(3) att_err(3) omega(3)]
  Eigen::VectorXd r_diag{};             ///< 4 input weights (per-rotor thrust deviation)
  Eigen::VectorXd q_terminal_diag{};    ///< 12 terminal weights, same layout as q_diag
  int max_sqp_iterations{1};            ///< RTI => 1
  double solve_time_budget_ms{5.0};     ///< abort + fall back beyond this
  int max_consecutive_failures{5};      ///< beyond this, nmpc_node aborts to PX4 failsafe
  bool warm_start{true};
  bool shift_on_warm_start{true};       ///< shift the previous solution one stage forward
  ///< per-rotor hover thrust [N]; seeds recovery guess (R2-16)
  double hover_thrust_per_rotor{0.0};
};

/// Non-copyable, movable RAII handle on the generated solver.
class AcadosWrapper
{
public:
  AcadosWrapper();
  ~AcadosWrapper();
  AcadosWrapper(const AcadosWrapper &) = delete;
  AcadosWrapper & operator=(const AcadosWrapper &) = delete;
  AcadosWrapper(AcadosWrapper &&) noexcept;
  AcadosWrapper & operator=(AcadosWrapper &&) noexcept;

  /// Allocate the capsule and create the solver. Verifies that the generated dimensions match
  /// `config` and that the model hash in the generated code matches codegen/MODEL_HASH.
  /// Returns false (and fills `error`) instead of throwing, so the lifecycle node can report
  /// FAILURE from on_configure().
  bool initialise(const SolverConfig & config, std::string * error = nullptr);

  /// Free the capsule. Safe to call twice. Called from on_cleanup()/destructor.
  void shutdown();

  bool isInitialised() const {return initialised_;}

  // --- problem data --------------------------------------------------------------------------

  /// Set lbx_0 = ubx_0 = x0 (the initial-state equality constraint).
  void setInitialState(const Eigen::VectorXd & x0);

  /// Stage references, stage in [0, N-1]: yref = [x_ref; u_ref] of size nx+nu.
  void setStageReference(int stage, const Eigen::VectorXd & x_ref, const Eigen::VectorXd & u_ref);

  /// Terminal reference: yref_e = x_ref, size nx.
  void setTerminalReference(const Eigen::VectorXd & x_ref);

  /// Convenience for the whole horizon; sizes must be N (inputs) and N+1 (states).
  void setReferenceHorizon(
    const std::vector<Eigen::VectorXd> & x_refs, const std::vector<Eigen::VectorXd> & u_refs);

  /// Online parameters p (per stage) — used for the wind/disturbance estimate and the
  /// time-varying mass if enabled in the model. Size must equal np from codegen.
  void setParameters(const Eigen::VectorXd & p);

  /// Update the diagonal cost weights at runtime (W, W_e). Cheap enough for a param callback,
  /// too slow for every tick. Returns false (and leaves the weights unchanged) if any
  /// diagonal is negative or the terminal weight is smaller than the stage weight elementwise.
  bool setWeights(
    const Eigen::VectorXd & q_diag, const Eigen::VectorXd & r_diag,
    const Eigen::VectorXd & q_terminal_diag);

  // --- solve ---------------------------------------------------------------------------------

  /// One RTI solve. Never throws. On failure applies the recovery policy of §6.5:
  ///   1st failure  -> re-solve once from a hover-initialised guess
  ///   Nth failure  -> report and let the caller fall back
  SolveResult solve();

  /// First optimal input u_0 [N per rotor]. Valid only after an ok() solve.
  Eigen::VectorXd optimalInput() const;

  /// Predicted state at `stage` in [0, N]. Used for the RViz predicted-path marker.
  Eigen::VectorXd predictedState(int stage) const;

  /// Whole predicted state trajectory, N+1 entries.
  std::vector<Eigen::VectorXd> predictedTrajectory() const;

  /// Reset every stage of the primal guess to a hover trim about `x0`, and zero the duals.
  /// Called on infeasibility and whenever the controller is (re-)activated.
  void resetToHover(const Eigen::VectorXd & x0, double hover_thrust_per_rotor);

  /// Shift the previous solution one stage forward (x_k <- x_{k+1}) as the warm start.
  void shiftWarmStart();

  // --- introspection -------------------------------------------------------------------------

  int nx() const {return nx_;}
  int nu() const {return nu_;}
  int np() const {return np_;}
  int horizonSteps() const {return config_.horizon_steps;}
  const SolverConfig & config() const {return config_;}

  /// Model hash baked into the generated code; compared against codegen/MODEL_HASH so a stale
  /// solver can never be silently flown.
  static std::string generatedModelHash();

private:
  /// PIMPL: hides all acados C types from every other translation unit.
  struct Impl;
  std::unique_ptr<Impl> impl_;

  SolverConfig config_{};
  bool initialised_{false};
  int nx_{0};
  int nu_{0};
  int np_{0};
  int consecutive_failures_{0};
};

}  // namespace uav_mpc

#endif  // UAV_MPC__ACADOS_WRAPPER_HPP_
