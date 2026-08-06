// Copyright (c) 2026 Ali-Eimaan.
// SPDX-License-Identifier: BSD-3-Clause
//
// RAII wrapper around the acados-generated OCP solver. See .deepseek/06_SOLVER.md §6.
//
// This is the ONLY translation unit allowed to include acados headers. Everything acados
// touches is behind the PIMPL (struct AcadosWrapper::Impl). The build-no-acados CI job
// enforces the boundary: build with -DUAV_MPC_WITH_ACADOS=OFF to get the stub backend, in
// which initialise() fills nx_/nu_ from the config and returns true so unit tests that
// never solve still link.
//
// Hot-path rules (§6.6): no allocation inside solve(), setStageReference(),
// setInitialState(), or optimalInput(). Scratch buffers are sized once in initialise() and
// reused; Eigen::Map / .data() are used over those buffers instead of constructing vectors.
//
// VERIFICATION STATUS: syntax-verified against the .deepseek spec, NOT compile-verified —
// acados is not built in this environment (codegen/ACADOS_COMMIT is an all-zeros
// UNVERIFIED placeholder), so the AcadosWrapper::Impl paths and the generated solver calls
// below are unchecked. The stub backend (UAV_MPC_WITH_ACADOS=OFF) builds without acados.

#include "uav_mpc/acados_wrapper.hpp"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <string>
#include <vector>

#include "uav_mpc/quadrotor_dynamics.hpp"  // StateLayout, for the recovery-path hover guess

#ifdef UAV_MPC_WITH_ACADOS
#ifndef UAV_MPC_CODEGEN_DIR
#error \
  "UAV_MPC_CODEGEN_DIR must be defined (set in CMakeLists.txt) so the " \
  "model-hash check can read codegen/MODEL_HASH"
#endif
// clang-format off
#include "acados/utils/print.h"
#include "acados/utils/types.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_solver_compat.h"   // NOLINT(build/include_subdir)
#include "acados_solver_quadrotor.h"   // NOLINT(build/include_subdir)
#include "model_hash.h"             // NOLINT(build/include_subdir)
// clang-format on
#endif

namespace uav_mpc
{

namespace
{

/// Trims whitespace from both ends of a string. Used by the model-hash check.
inline std::string trim(const std::string & s)
{
  const auto start = s.find_first_not_of(" \t\r\n");
  if (start == std::string::npos) {return {};}
  const auto end = s.find_last_not_of(" \t\r\n");
  return s.substr(start, end - start + 1);
}

/// Stub-build layout defaults. The real values come from the generated code (QUADROTOR_NX
/// etc.); the stub only needs sizes that keep tests linking.
constexpr int kStubNx = 13;  // quaternion layout
constexpr int kStubNu = 4;
constexpr int kStubNp = 8;   // [wind(3) mass_scale(1) q_ref(4)]

/// Codegen defaults for the structural checks in initialise(). The generated solver bakes in
/// N = 20, Tf = 1.0 (§6.2); the wrapper refuses to run against anything else.
constexpr int kCodegenHorizon = 20;
constexpr double kCodegenTf = 1.0;

/// Residual dimensions (§6.3): y = [p v e_q omega u] (16), y_e = [p v e_q omega] (12).
constexpr int kStageResidual = 16;
constexpr int kTerminalResidual = 12;

}  // namespace

/// PIMPL body. Holds the raw acados handles; empty when built without acados.
struct AcadosWrapper::Impl
{
#ifdef UAV_MPC_WITH_ACADOS
  quadrotor_solver_capsule * capsule{nullptr};
  ocp_nlp_config * nlp_config{nullptr};
  ocp_nlp_dims * nlp_dims{nullptr};
  ocp_nlp_in * nlp_in{nullptr};
  ocp_nlp_out * nlp_out{nullptr};
  ocp_nlp_solver * nlp_solver{nullptr};
  void * nlp_opts{nullptr};
  int lam_size{0};  ///< per-stage multiplier size, queried from nlp_dims at initialise
#endif
  std::vector<double> yref_buffer;    ///< scratch, sized max(ny, ny_e), reused every tick
  std::vector<double> x_buffer;       ///< scratch, sized nx
  std::vector<double> u_buffer;       ///< scratch, sized nu
  std::vector<double> zero_buffer;    ///< scratch for zeroing lam/pi and slop-free guesses
  Eigen::VectorXd x0;                 ///< last initial state (recovery bookkeeping)
  double hover_thrust{0.0};           ///< last hover thrust passed to resetToHover()
};

AcadosWrapper::AcadosWrapper()
: impl_(std::make_unique<Impl>())
{
}

AcadosWrapper::~AcadosWrapper()
{
  shutdown();  // idempotent, must not throw
}

AcadosWrapper::AcadosWrapper(AcadosWrapper &&) noexcept = default;
AcadosWrapper & AcadosWrapper::operator=(AcadosWrapper &&) noexcept = default;

bool AcadosWrapper::initialise(const SolverConfig & config, std::string * error)
{
  shutdown();  // idempotent; makes re-configure safe

  auto fail = [error](const std::string & msg) {
      if (error != nullptr) {*error = msg;}
      return false;
    };

  // Structural checks run in BOTH builds so a config/codegen mismatch is caught even on the
  // stub (where the "generated" values are the known codegen defaults).
  if (config.horizon_steps != kCodegenHorizon) {
    return fail("config.horizon_steps = " + std::to_string(config.horizon_steps) +
      " does not match the generated solver (N = " + std::to_string(kCodegenHorizon) +
      "); re-run codegen/generate_acados_solver.py or fix nmpc_params.yaml");
  }
  if (std::abs(config.horizon_time - kCodegenTf) > 1e-9) {
    return fail("config.horizon_time = " + std::to_string(config.horizon_time) +
      " does not match the generated solver (Tf = " + std::to_string(kCodegenTf) + ")");
  }
  if (config.max_sqp_iterations < 1) {
    return fail("max_sqp_iterations must be >= 1");
  }

#ifdef UAV_MPC_WITH_ACADOS
  impl_->capsule = quadrotor_acados_create_capsule();
  if (impl_->capsule == nullptr) {
    return fail("quadrotor_acados_create_capsule() returned NULL");
  }
  const int create_status = quadrotor_acados_create(impl_->capsule);
  if (create_status != ACADOS_SUCCESS) {
    shutdown();
    return fail("quadrotor_acados_create() failed with status " +
      std::to_string(create_status));
  }

  impl_->nlp_config = quadrotor_acados_get_nlp_config(impl_->capsule);
  impl_->nlp_dims = quadrotor_acados_get_nlp_dims(impl_->capsule);
  impl_->nlp_in = quadrotor_acados_get_nlp_in(impl_->capsule);
  impl_->nlp_out = quadrotor_acados_get_nlp_out(impl_->capsule);
  impl_->nlp_solver = quadrotor_acados_get_nlp_solver(impl_->capsule);
  impl_->nlp_opts = quadrotor_acados_get_nlp_opts(impl_->capsule);

  nx_ = QUADROTOR_NX;
  nu_ = QUADROTOR_NU;
  np_ = QUADROTOR_NP;
  if (QUADROTOR_N != config.horizon_steps ||
    std::abs(QUADROTOR_TF - config.horizon_time) > 1e-9)
  {
    shutdown();
    return fail("generated solver dimensions (N = " + std::to_string(QUADROTOR_N) +
      ", Tf = " + std::to_string(QUADROTOR_TF) + ") disagree with config");
  }

  // The hash check is what stops a stale solver being flown against a changed model (§6.7).
  // Do not make it a warning.
  const std::string expected = generatedModelHash();
  std::ifstream hash_file(std::string(UAV_MPC_CODEGEN_DIR) + "/MODEL_HASH");
  std::string recorded;
  std::getline(hash_file, recorded);
  if (!hash_file || trim(recorded) != expected) {
    shutdown();
    return fail("model hash mismatch: generated code hashes to '" + expected +
      "' but codegen/MODEL_HASH records '" + trim(recorded) +
      "'. Re-run codegen/generate_acados_solver.py (--check-only in CI) before flying.");
  }

  impl_->lam_size = ocp_nlp_dims_get_from_attr(impl_->nlp_config, impl_->nlp_dims,
    impl_->nlp_out, 0, "lam");
  if (impl_->lam_size <= 0) {impl_->lam_size = nx_ + nu_ + 8;}  // defensive fallback
#else
  nx_ = kStubNx;
  nu_ = kStubNu;
  np_ = kStubNp;
#endif

  // Scratch buffers, sized once (§6.6). Never resized on the hot path.
  impl_->yref_buffer.assign(static_cast<std::size_t>(kStageResidual), 0.0);
  impl_->x_buffer.assign(static_cast<std::size_t>(nx_), 0.0);
  impl_->u_buffer.assign(static_cast<std::size_t>(nu_), 0.0);
  impl_->zero_buffer.assign(static_cast<std::size_t>(nx_ + nu_ + 8), 0.0);
  impl_->x0 = Eigen::VectorXd::Zero(nx_);
  impl_->hover_thrust = config.hover_thrust_per_rotor;

  config_ = config;
  initialised_ = true;

  // Apply the cost weights from config (empty diagonals leave the codegen defaults in place).
  if (config.q_diag.size() > 0 || config.r_diag.size() > 0 ||
    config.q_terminal_diag.size() > 0)
  {
    if (!setWeights(config.q_diag, config.r_diag, config.q_terminal_diag)) {
      shutdown();
      return fail("invalid cost weights (see setWeights validation)");
    }
  }

#ifdef UAV_MPC_WITH_ACADOS
  // Seed the online parameter vector: mass_scale and q_ref.w must be 1.0 so the
  // solver is well-posed from the first call (§4). Zero wind, zero attitude error.
  Eigen::VectorXd p0 = Eigen::VectorXd::Zero(np_);
  p0(3) = 1.0;   // mass_scale — dynamics divide by this, so 0 → NaN
  p0(4) = 1.0;   // q_ref.w — quaternion reference scalar part
  for (int stage = 0; stage < QUADROTOR_N; ++stage) {
    quadrotor_acados_update_params(impl_->capsule, stage,
      const_cast<double *>(p0.data()), np_);
  }
#endif

  return true;
}

void AcadosWrapper::shutdown()
{
  if (!impl_) {return;}  // moved-from state
#ifdef UAV_MPC_WITH_ACADOS
  if (impl_->capsule != nullptr) {
    quadrotor_acados_free(impl_->capsule);
    quadrotor_acados_free_capsule(impl_->capsule);
    impl_->capsule = nullptr;
  }
  impl_->nlp_config = nullptr;
  impl_->nlp_dims = nullptr;
  impl_->nlp_in = nullptr;
  impl_->nlp_out = nullptr;
  impl_->nlp_solver = nullptr;
  impl_->nlp_opts = nullptr;
#endif
  impl_->yref_buffer.clear();
  impl_->x_buffer.clear();
  impl_->u_buffer.clear();
  impl_->zero_buffer.clear();
  impl_->x0 = Eigen::VectorXd{};
  impl_->hover_thrust = 0.0;
  initialised_ = false;
}

void AcadosWrapper::setInitialState(const Eigen::VectorXd & x0)
{
  if (!initialised_ || x0.size() != nx_) {return;}
#ifdef UAV_MPC_WITH_ACADOS
  // Initial-state equality: lbx_0 = ubx_0 = x0 (§6.4).
  // acados 0.6.0: stage-0 bounds use "lbx"/"ubx", not "lbx_0"/"ubx_0".
  ocp_nlp_constraints_model_set(impl_->nlp_config, impl_->nlp_dims,
    impl_->nlp_in, impl_->nlp_out, 0, "lbx", const_cast<double *>(x0.data()));
  ocp_nlp_constraints_model_set(impl_->nlp_config, impl_->nlp_dims,
    impl_->nlp_in, impl_->nlp_out, 0, "ubx", const_cast<double *>(x0.data()));
#endif
  impl_->x0 = x0;  // same size as nx_ => no reallocation on the hot path
}

void AcadosWrapper::setStageReference(
  int stage, const Eigen::VectorXd & x_ref, const Eigen::VectorXd & u_ref)
{
  if (!initialised_ || stage < 0 || stage >= config_.horizon_steps) {return;}
  if (x_ref.size() != nx_ || u_ref.size() != nu_) {return;}
  // yref = [p(3) v(3) e_q(3)=0 omega(3) u(4)]; the attitude-error block is always zero
  // because q_ref enters through the online parameters (§6.3).
  Eigen::Map<Eigen::VectorXd>(impl_->yref_buffer.data(), kStageResidual).setZero();
  impl_->yref_buffer[0] = x_ref[0];
  impl_->yref_buffer[1] = x_ref[1];
  impl_->yref_buffer[2] = x_ref[2];
  impl_->yref_buffer[3] = x_ref[3];
  impl_->yref_buffer[4] = x_ref[4];
  impl_->yref_buffer[5] = x_ref[5];
  // 6..8 stay zero (attitude error)
  impl_->yref_buffer[9] = x_ref[nx_ - 3];
  impl_->yref_buffer[10] = x_ref[nx_ - 2];
  impl_->yref_buffer[11] = x_ref[nx_ - 1];
  impl_->yref_buffer[12] = u_ref[0];
  impl_->yref_buffer[13] = u_ref[1];
  impl_->yref_buffer[14] = u_ref[2];
  impl_->yref_buffer[15] = u_ref[3];
#ifdef UAV_MPC_WITH_ACADOS
  ocp_nlp_cost_model_set(impl_->nlp_config, impl_->nlp_dims,
    impl_->nlp_in, stage, "yref", impl_->yref_buffer.data());
#endif
}

void AcadosWrapper::setTerminalReference(const Eigen::VectorXd & x_ref)
{
  if (!initialised_ || x_ref.size() != nx_) {return;}
  // yref_e = [p(3) v(3) e_q(3)=0 omega(3)], size ny_e = 12.
  Eigen::Map<Eigen::VectorXd>(impl_->yref_buffer.data(), kTerminalResidual).setZero();
  impl_->yref_buffer[0] = x_ref[0];
  impl_->yref_buffer[1] = x_ref[1];
  impl_->yref_buffer[2] = x_ref[2];
  impl_->yref_buffer[3] = x_ref[3];
  impl_->yref_buffer[4] = x_ref[4];
  impl_->yref_buffer[5] = x_ref[5];
  // 6..8 stay zero (attitude error)
  impl_->yref_buffer[9] = x_ref[nx_ - 3];
  impl_->yref_buffer[10] = x_ref[nx_ - 2];
  impl_->yref_buffer[11] = x_ref[nx_ - 1];
#ifdef UAV_MPC_WITH_ACADOS
  ocp_nlp_cost_model_set(impl_->nlp_config, impl_->nlp_dims,
    impl_->nlp_in, config_.horizon_steps, "yref",
    impl_->yref_buffer.data());
#endif
}

void AcadosWrapper::setReferenceHorizon(
  const std::vector<Eigen::VectorXd> & x_refs, const std::vector<Eigen::VectorXd> & u_refs)
{
  if (!initialised_) {return;}
  if (x_refs.size() != static_cast<std::size_t>(config_.horizon_steps + 1) ||
    u_refs.size() != static_cast<std::size_t>(config_.horizon_steps))
  {
    return;
  }
  for (int stage = 0; stage < config_.horizon_steps; ++stage) {
    setStageReference(stage, x_refs[static_cast<std::size_t>(stage)],
      u_refs[static_cast<std::size_t>(stage)]);
  }
  setTerminalReference(x_refs[static_cast<std::size_t>(config_.horizon_steps)]);
}

void AcadosWrapper::setParameters(const Eigen::VectorXd & p)
{
  if (!initialised_ || p.size() != np_) {return;}
#ifdef UAV_MPC_WITH_ACADOS
  for (int stage = 0; stage < config_.horizon_steps; ++stage) {
    quadrotor_acados_update_params(impl_->capsule, stage,
      const_cast<double *>(p.data()), np_);
  }
#endif
}

bool AcadosWrapper::setWeights(
  const Eigen::VectorXd & q_diag, const Eigen::VectorXd & r_diag,
  const Eigen::VectorXd & q_terminal_diag)
{
  if (!initialised_) {return false;}
  if (q_diag.size() != kTerminalResidual || r_diag.size() != nu_ ||
    q_terminal_diag.size() != kTerminalResidual)
  {
    return false;
  }
  // Non-negative diagonals; terminal weight must not be smaller than the stage weight
  // elementwise (a smaller terminal weight is the classic cause of horizon-end drift, §6.3).
  for (int i = 0; i < kTerminalResidual; ++i) {
    if (q_diag[i] < 0.0 || q_terminal_diag[i] < 0.0) {return false;}
    if (q_terminal_diag[i] < q_diag[i]) {return false;}
  }
  for (int i = 0; i < nu_; ++i) {
    if (r_diag[i] < 0.0) {return false;}
  }

  // Dense diagonal W (16x16) and W_e (12x12), row-major (acados convention).
  std::vector<double> w(static_cast<std::size_t>(kStageResidual * kStageResidual), 0.0);
  for (int i = 0; i < kTerminalResidual; ++i) {
    w[static_cast<std::size_t>(i * kStageResidual + i)] = q_diag[i];
  }
  for (int i = 0; i < nu_; ++i) {
    const int col = kTerminalResidual + i;
    w[static_cast<std::size_t>(col * kStageResidual + col)] = r_diag[i];
  }
  std::vector<double> w_e(static_cast<std::size_t>(kTerminalResidual * kTerminalResidual), 0.0);
  for (int i = 0; i < kTerminalResidual; ++i) {
    w_e[static_cast<std::size_t>(i * kTerminalResidual + i)] = q_terminal_diag[i];
  }

#ifdef UAV_MPC_WITH_ACADOS
  for (int stage = 0; stage < config_.horizon_steps; ++stage) {
    ocp_nlp_cost_model_set(impl_->nlp_config, impl_->nlp_dims,
      impl_->nlp_in, stage, "W", w.data());
  }
  ocp_nlp_cost_model_set(impl_->nlp_config, impl_->nlp_dims,
    impl_->nlp_in, config_.horizon_steps, "W", w_e.data());
#endif
  return true;
}

SolveResult AcadosWrapper::solve()
{
  SolveResult result;
  if (!initialised_) {
    result.status = SolverStatus::NotInitialised;
    result.consecutive_failures = consecutive_failures_;
    return result;
  }

#ifdef UAV_MPC_WITH_ACADOS
  const auto t0 = std::chrono::steady_clock::now();

  int acados_status = quadrotor_acados_solve(impl_->capsule);

  // Failure recovery (§6.5): first failure -> re-solve once from a hover-initialised guess.
  if (acados_status != ACADOS_SUCCESS) {
    ++consecutive_failures_;
    if (consecutive_failures_ == 1) {
      // R2-16: refuse to recover from zero thrust (free-fall guess). When
      // hover_thrust_per_rotor is 0, the solver has never been told what thrust
      // to expect — the retry would be a free-fall dive. Warn and skip.
      if (impl_->hover_thrust > 0.0) {
        resetToHover(impl_->x0, impl_->hover_thrust);
        acados_status = quadrotor_acados_solve(impl_->capsule);
        result.reinitialised = true;
      }
    }
  }

  // Reset the failure counter when the solver succeeds, whether it was the
  // first attempt or the recovery retry (§6.5 bugfix).
  if (acados_status == ACADOS_SUCCESS) {
    consecutive_failures_ = 0;
  }

  const auto t1 = std::chrono::steady_clock::now();
  result.wall_time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

  double time_tot{0.0};
  double cost{0.0};
  double kkt{0.0};
  int sqp_iter{0};
  ocp_nlp_get(impl_->nlp_solver, "time_tot", &time_tot);
  ocp_nlp_get(impl_->nlp_solver, "sqp_iter", &sqp_iter);
  ocp_nlp_get(impl_->nlp_solver, "cost_value", &cost);
  // acados 0.6.0: "residuals" was split into res_stat/res_eq/res_ineq/res_comp.
  // res_stat is the stationarity residual inf-norm (closest to the old combined KKT).
  ocp_nlp_get(impl_->nlp_solver, "res_stat", &kkt);
  result.solve_time_ms = time_tot * 1e3;
  result.sqp_iterations = sqp_iter;
  result.cost = cost;
  result.kkt_residual = kkt;
  result.consecutive_failures = consecutive_failures_;

  switch (acados_status) {
    case ACADOS_SUCCESS:
      result.status = SolverStatus::Success;
      break;
    case ACADOS_MAXITER:
      result.status = SolverStatus::MaxIterations;
      break;
    case ACADOS_MINSTEP:
      result.status = SolverStatus::MinStepLength;
      break;
    case ACADOS_QP_FAILURE:
    case ACADOS_INFEASIBLE:
      result.status = SolverStatus::QpFailure;
      break;
    case ACADOS_NAN_DETECTED:
      result.status = SolverStatus::NanDetected;
      break;
    default:
      result.status = SolverStatus::QpFailure;
      break;
  }

  // Wall-clock budget (§6.5): report Timeout even if acados returned success, so a missed
  // deadline shows up in the logs instead of silently stretching the control period.
  if (result.wall_time_ms > config_.solve_time_budget_ms) {
    result.status = SolverStatus::Timeout;
  }
#else
  // Stub backend: nothing to solve, nothing to time.
  result.status = SolverStatus::Success;
  result.consecutive_failures = 0;
#endif
  return result;
}

Eigen::VectorXd AcadosWrapper::optimalInput() const
{
  if (!initialised_) {return Eigen::VectorXd::Zero(nu_);}
#ifdef UAV_MPC_WITH_ACADOS
  ocp_nlp_out_get(impl_->nlp_config, impl_->nlp_dims,
    impl_->nlp_out, 0, "u", impl_->u_buffer.data());
#endif
  return Eigen::Map<const Eigen::VectorXd>(impl_->u_buffer.data(), nu_);
}

Eigen::VectorXd AcadosWrapper::predictedState(int stage) const
{
  if (!initialised_) {return Eigen::VectorXd::Zero(nx_);}
#ifdef UAV_MPC_WITH_ACADOS
  const int clamped = std::max(0, std::min(stage, config_.horizon_steps));
  ocp_nlp_out_get(impl_->nlp_config, impl_->nlp_dims,
    impl_->nlp_out, clamped, "x", impl_->x_buffer.data());
#else
  (void)stage;
#endif
  return Eigen::Map<const Eigen::VectorXd>(impl_->x_buffer.data(), nx_);
}

std::vector<Eigen::VectorXd> AcadosWrapper::predictedTrajectory() const
{
  std::vector<Eigen::VectorXd> traj;
  if (!initialised_) {return traj;}
  traj.reserve(static_cast<std::size_t>(config_.horizon_steps + 1));
  for (int stage = 0; stage <= config_.horizon_steps; ++stage) {
    traj.push_back(predictedState(stage));
  }
  return traj;
}

void AcadosWrapper::resetToHover(const Eigen::VectorXd & x0, double hover_thrust_per_rotor)
{
  if (!initialised_ || x0.size() != nx_) {return;}
  impl_->x0 = x0;
  impl_->hover_thrust = hover_thrust_per_rotor;
#ifdef UAV_MPC_WITH_ACADOS
  // Hover guess: keep x0's position/attitude, zero velocity and body rates; thrust at hover.
  Eigen::Map<Eigen::VectorXd> hover_x(impl_->x_buffer.data(), nx_);
  hover_x = x0;
  hover_x.segment(StateLayout<AttitudeRep::Quaternion>::kVelIdx, 3).setZero();
  hover_x.tail(3).setZero();  // body rates are the last three states in both layouts
  Eigen::Map<Eigen::VectorXd> hover_u(impl_->u_buffer.data(), nu_);
  hover_u.setConstant(hover_thrust_per_rotor);

  for (int stage = 0; stage <= config_.horizon_steps; ++stage) {
    ocp_nlp_out_set(impl_->nlp_config, impl_->nlp_dims,
      impl_->nlp_out, impl_->nlp_in, stage, "x", hover_x.data());
    if (stage < config_.horizon_steps) {
      ocp_nlp_out_set(impl_->nlp_config, impl_->nlp_dims,
        impl_->nlp_out, impl_->nlp_in, stage, "u", hover_u.data());
    }
    ocp_nlp_out_set(impl_->nlp_config, impl_->nlp_dims,
      impl_->nlp_out, impl_->nlp_in, stage, "lam", impl_->zero_buffer.data());
    ocp_nlp_out_set(impl_->nlp_config, impl_->nlp_dims,
      impl_->nlp_out, impl_->nlp_in, stage, "pi", impl_->zero_buffer.data());
  }
#endif
}

void AcadosWrapper::shiftWarmStart()
{
  if (!initialised_ || !config_.shift_on_warm_start) {return;}
#ifdef UAV_MPC_WITH_ACADOS
  // x_k <- x_{k+1} for k in [0, N-1]; x_N unchanged. u_k <- u_{k+1} for k in [0, N-2].
  for (int stage = 0; stage < config_.horizon_steps; ++stage) {
    ocp_nlp_out_get(impl_->nlp_config, impl_->nlp_dims,
      impl_->nlp_out, stage + 1, "x", impl_->x_buffer.data());
    ocp_nlp_out_set(impl_->nlp_config, impl_->nlp_dims,
      impl_->nlp_out, impl_->nlp_in, stage, "x", impl_->x_buffer.data());
  }
  for (int stage = 0; stage + 1 < config_.horizon_steps; ++stage) {
    ocp_nlp_out_get(impl_->nlp_config, impl_->nlp_dims,
      impl_->nlp_out, stage + 1, "u", impl_->u_buffer.data());
    ocp_nlp_out_set(impl_->nlp_config, impl_->nlp_dims,
      impl_->nlp_out, impl_->nlp_in, stage, "u", impl_->u_buffer.data());
  }
#endif
}

std::string AcadosWrapper::generatedModelHash()
{
#ifdef UAV_MPC_WITH_ACADOS
#ifdef UAV_MPC_MODEL_HASH
  return std::string(UAV_MPC_MODEL_HASH);
#else
  return std::string();
#endif
#else
  return std::string();
#endif
}

}  // namespace uav_mpc
