// Copyright (c) 2026 Ali-Eimaan.
// SPDX-License-Identifier: BSD-3-Clause
//
// THE gate test. CI fails if the NMPC solve regresses past the real-time budget, which is the
// single claim this repo is making. Keep it honest:
//   - Release build only (the CMake test target inherits CMAKE_BUILD_TYPE)
//   - measured with steady_clock around the solve, not acados' self-report
//   - reported as median / p95 / p99 / max over N samples, never a single mean
//   - the threshold is on the p99, because a 100 Hz loop that misses once per second is broken
//
// See .deepseek/10_TESTS.md §10.1. In the stub backend (UAV_MPC_WITH_ACADOS undefined) every
// solve is a no-op that returns Success in ~0 ms, so the budget assertions trivially pass —
// they only have teeth when the generated acados solver is linked in.

#include <gtest/gtest.h>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "uav_mpc/acados_wrapper.hpp"
#include "uav_mpc/quadrotor_dynamics.hpp"
#include "uav_mpc/trajectory_generator.hpp"

#ifndef UAV_MPC_SOURCE_DIR
#error "UAV_MPC_SOURCE_DIR must be defined (set in CMakeLists.txt)"
#endif
#ifndef UAV_MPC_CODEGEN_DIR
#error "UAV_MPC_CODEGEN_DIR must be defined (set in CMakeLists.txt)"
#endif

// REVIEW R1-13: the budgets are the A2 acceptance criteria, generated at configure time from
// acceptance_criteria.yaml (the single source of truth). Do not edit them here.
#include "acceptance_criteria.h"  // NOLINT(build/include_subdir)

namespace
{

/// Budget at 100 Hz. The solve must fit with room for the rest of the callback.
constexpr double kP99BudgetMs = uav_mpc::acceptance::kSolveP99BudgetMs;
constexpr double kMedianBudgetMs = uav_mpc::acceptance::kSolveMedianBudgetMs;
constexpr int kSamples = 10000;
constexpr int kWarmUpSolves = 100;

/// Budgets may be relaxed via environment variables for noisy shared CI runners — see the
/// "solve-time noise" decision documented in .github/workflows/colcon_build.yml. Local runs
/// keep the strict defaults above, which are the repo's real-time claim.
double budgetMs(const char * var, double def)
{
  const char * v = std::getenv(var);
  return (v == nullptr) ? def : std::strtod(v, nullptr);
}

/// Wall-clock timeout for the test environment, to prevent OS scheduling jitter from
/// producing false Timeout statuses.  This is independent of the performance budget
/// (kP99BudgetMs / kMedianBudgetMs), which still gates solve speed.  Defaults to 100 ms
/// — high enough that only a genuinely stuck solver triggers it.
constexpr double kTestWallTimeoutMs = 100.0;

/// Loads params/x500_calibration.yaml relative to the test binary (UAV_MPC_SOURCE_DIR is a
/// compile definition set in CMakeLists.txt — never hard-code a path).
uav_mpc::QuadrotorParams loadX500()
{
  const std::string path = std::string(UAV_MPC_SOURCE_DIR) + "/params/x500_calibration.yaml";
  return uav_mpc::QuadrotorParams::fromYaml(path);
}

/// Converts a YAML sequence node into an Eigen vector.
Eigen::VectorXd yamlVector(const YAML::Node & node)
{
  Eigen::VectorXd v(static_cast<Eigen::Index>(node.size()));
  for (std::size_t i = 0; i < node.size(); ++i) {
    v(static_cast<Eigen::Index>(i)) = node[i].as<double>();
  }
  return v;
}

/// Builds an initialised solver with the nominal config from config/nmpc_params.yaml.
uav_mpc::AcadosWrapper makeSolver()
{
  const std::string path = std::string(UAV_MPC_SOURCE_DIR) + "/config/nmpc_params.yaml";
  const YAML::Node doc = YAML::LoadFile(path);
  const YAML::Node ros = doc["/**"]["ros__parameters"];

  uav_mpc::SolverConfig cfg;
  cfg.horizon_steps = ros["horizon_steps"].as<int>();
  cfg.horizon_time = ros["horizon_time"].as<double>();
  cfg.q_diag = yamlVector(ros["q_diag"]);
  cfg.r_diag = yamlVector(ros["r_diag"]);
  cfg.q_terminal_diag = yamlVector(ros["q_terminal_diag"]);
  cfg.max_sqp_iterations = ros["max_sqp_iterations"].as<int>();
  // Use a generous wall-clock timeout for tests: OS scheduling jitter on shared CI
  // runners can stretch wall time well past the acados self-reported solve time, and
  // a Timeout status here would be a false positive.  The real-time performance
  // gates are the p99/median checks below, which use kP99BudgetMs/kMedianBudgetMs.
  cfg.solve_time_budget_ms = budgetMs("UAV_MPC_SOLVE_WALL_TIMEOUT_MS", kTestWallTimeoutMs);
  cfg.max_consecutive_failures = ros["max_consecutive_failures"].as<int>();
  cfg.warm_start = ros["warm_start"].as<bool>();
  cfg.shift_on_warm_start = ros["shift_on_warm_start"].as<bool>();

  // REVIEW R2-16: seed hover thrust so the in-solver recovery path is never
  // disabled by a zero-per-rotor thrust.  The x500 calibration is the default
  // test airframe; the value gets overwritten by the first resetToHover call
  // in the test anyway, so it is only a safety net.
  {
    const auto ap = loadX500();
    cfg.hover_thrust_per_rotor = ap.mass * ap.gravity / 4.0;
  }

  uav_mpc::AcadosWrapper solver;
  std::string error;
  if (!solver.initialise(cfg, &error)) {
    throw std::runtime_error("makeSolver(): solver initialisation failed: " + error);
  }
  return solver;  // move (AcadosWrapper is movable, not copyable)
}

/// Quaternion-layout hover state at (x, y, z): level attitude, zero velocity/rates.
Eigen::VectorXd hoverState(const Eigen::Vector3d & p)
{
  Eigen::VectorXd x(13);
  x << p.x(), p.y(), p.z(), 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  return x;
}

/// Small random perturbation on position/velocity only — the warm start is exercised
/// realistically without throwing the solver off a cliff.
void perturbState(Eigen::VectorXd * x, std::mt19937 & rng)
{
  std::normal_distribution<double> pos(0.0, 0.02);
  std::normal_distribution<double> vel(0.0, 0.05);
  for (int i = 0; i < 3; ++i) {
    (*x)(i) += pos(rng);
    (*x)(3 + i) += vel(rng);
  }
}

/// Percentile of an already-sorted vector (linear interpolation, p in [0, 1]).
double percentile(const std::vector<double> & sorted, double p)
{
  if (sorted.empty()) {return 0.0;}
  const double idx = p * static_cast<double>(sorted.size() - 1);
  const std::size_t lo = static_cast<std::size_t>(std::floor(idx));
  const std::size_t hi = static_cast<std::size_t>(std::ceil(idx));
  const double frac = idx - static_cast<double>(lo);
  return sorted[lo] * (1.0 - frac) + sorted[hi] * frac;
}

/// Records the full percentile table under `prefix_*` so CI logs are useful.
void reportPercentiles(const std::string & prefix, const std::vector<double> & times)
{
  std::vector<double> sorted = times;
  std::sort(sorted.begin(), sorted.end());

  const double min = sorted.front();
  const double max = sorted.back();
  const double mean =
    std::accumulate(sorted.begin(), sorted.end(), 0.0) / static_cast<double>(sorted.size());
  const double median = sorted.size() % 2 == 1 ?
    sorted[sorted.size() / 2] :
    0.5 * (sorted[sorted.size() / 2 - 1] + sorted[sorted.size() / 2]);
  const double p90 = percentile(sorted, 0.90);
  const double p95 = percentile(sorted, 0.95);
  const double p99 = percentile(sorted, 0.99);
  const double p999 = percentile(sorted, 0.999);

  testing::Test::RecordProperty((prefix + "_min_ms").c_str(), min);
  testing::Test::RecordProperty((prefix + "_median_ms").c_str(), median);
  testing::Test::RecordProperty((prefix + "_mean_ms").c_str(), mean);
  testing::Test::RecordProperty((prefix + "_p90_ms").c_str(), p90);
  testing::Test::RecordProperty((prefix + "_p95_ms").c_str(), p95);
  testing::Test::RecordProperty((prefix + "_p99_ms").c_str(), p99);
  testing::Test::RecordProperty((prefix + "_p99_9_ms").c_str(), p999);
  testing::Test::RecordProperty((prefix + "_max_ms").c_str(), max);
}

}  // namespace

// Hover at the origin: the easiest problem. If this is slow, nothing else matters.
TEST(NmpcSolveTime, HoverSolveWithinBudget)
{
  uav_mpc::AcadosWrapper solver;
  ASSERT_NO_THROW(solver = makeSolver());

  const uav_mpc::QuadrotorParams airframe = loadX500();
  const uav_mpc::QuadrotorDynamics<double, uav_mpc::AttitudeRep::Quaternion> dyn(airframe);
  const double hover_thrust = dyn.hoverThrustPerRotor();
  const int n = solver.horizonSteps();

  const Eigen::Vector3d center(0.0, 0.0, 1.5);
  const Eigen::VectorXd x_hover = hoverState(center);
  Eigen::VectorXd u_hover(4);
  u_hover.setConstant(hover_thrust);

  std::vector<Eigen::VectorXd> x_refs(static_cast<std::size_t>(n) + 1, x_hover);
  std::vector<Eigen::VectorXd> u_refs(static_cast<std::size_t>(n), u_hover);
  solver.setReferenceHorizon(x_refs, u_refs);

  std::mt19937 rng(42);

  // Warm up: 100 discarded solves so caches, TLB and the codegen'd initialisations settle.
  Eigen::VectorXd x0 = x_hover;
  perturbState(&x0, rng);
  solver.setInitialState(x0);
  solver.resetToHover(x0, hover_thrust);
  for (int i = 0; i < kWarmUpSolves; ++i) {
    const uav_mpc::SolveResult r = solver.solve();
    ASSERT_EQ(r.status, uav_mpc::SolverStatus::Success);
    solver.shiftWarmStart();
  }

  std::vector<double> times;
  times.reserve(static_cast<std::size_t>(kSamples));
  bool all_ok = true;
  for (int i = 0; i < kSamples; ++i) {
    x0 = x_hover;  // reset so perturbState does not random-walk (§5.2)
    perturbState(&x0, rng);
    solver.setInitialState(x0);

    const auto t0 = std::chrono::steady_clock::now();
    const uav_mpc::SolveResult r = solver.solve();
    const auto t1 = std::chrono::steady_clock::now();
    times.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());

    all_ok = all_ok && (r.status == uav_mpc::SolverStatus::Success);
    solver.shiftWarmStart();
  }

  reportPercentiles("hover", times);
  std::vector<double> sorted = times;
  std::sort(sorted.begin(), sorted.end());
  const double median = sorted.size() % 2 == 1 ?
    sorted[sorted.size() / 2] :
    0.5 * (sorted[sorted.size() / 2 - 1] + sorted[sorted.size() / 2]);

  const double median_budget = budgetMs("UAV_MPC_SOLVE_MEDIAN_BUDGET_MS", kMedianBudgetMs);
  const double p99_budget = budgetMs("UAV_MPC_SOLVE_P99_BUDGET_MS", kP99BudgetMs);
  EXPECT_TRUE(all_ok) << "at least one hover solve returned a non-Success status";
  EXPECT_LE(median, median_budget)
    << "median hover solve exceeded the " << median_budget << " ms budget";
  EXPECT_LE(percentile(sorted, 0.99), p99_budget)
    << "p99 hover solve exceeded the " << p99_budget << " ms budget";
}

// Aggressive figure-8: the case the README claims. Same budget applies.
TEST(NmpcSolveTime, Figure8SolveWithinBudget)
{
  uav_mpc::AcadosWrapper solver;
  ASSERT_NO_THROW(solver = makeSolver());

  const uav_mpc::QuadrotorParams airframe = loadX500();
  const int n = solver.horizonSteps();
  const double dt = solver.config().horizon_time / static_cast<double>(n);

  // 3 m amplitude, 5 s period — the aggressive preset from the README.
  uav_mpc::TrajectoryParams tp;
  tp.type = uav_mpc::TrajectoryType::Figure8;
  tp.amplitude_x = 3.0;
  tp.amplitude_y = 3.0;
  tp.period = 5.0;
  tp.altitude = 1.5;
  tp.ramp_in_time = 3.0;
  tp.yaw_follows_velocity = true;
  uav_mpc::TrajectoryGenerator gen(tp);

  // REVIEW R2-16: seed a hover warm start and hover thrust so the first solve is not
  // cold and the in-solver recovery path can retry from a sane guess if needed.
  {
    const uav_mpc::QuadrotorDynamics<double, uav_mpc::AttitudeRep::Quaternion> dyn(airframe);
    const double hover_thrust = dyn.hoverThrustPerRotor();
    const Eigen::VectorXd x_hover = hoverState(Eigen::Vector3d(0.0, 0.0, tp.altitude));
    solver.resetToHover(x_hover, hover_thrust);
  }

  std::mt19937 rng(7);
  double t0 = 0.0;
  std::vector<double> times;
  times.reserve(static_cast<std::size_t>(kSamples));

  auto solve_at = [&](bool record) {
      const auto refs = gen.referenceHorizon(
      t0, dt, n, airframe, uav_mpc::AttitudeRep::Quaternion);
      std::vector<Eigen::VectorXd> x_refs, u_refs;
      x_refs.reserve(refs.size());
      u_refs.reserve(refs.size() - 1);
      for (std::size_t k = 0; k < refs.size(); ++k) {
        x_refs.push_back(refs[k].state);
        if (k + 1 < refs.size()) {u_refs.push_back(refs[k].input);}
      }
      solver.setReferenceHorizon(x_refs, u_refs);

      Eigen::VectorXd x0 = refs.front().state;
      perturbState(&x0, rng);
      solver.setInitialState(x0);

      const auto s0 = std::chrono::steady_clock::now();
      const uav_mpc::SolveResult r = solver.solve();
      const auto s1 = std::chrono::steady_clock::now();
      if (record) {times.push_back(std::chrono::duration<double, std::milli>(s1 - s0).count());}
      solver.shiftWarmStart();
      t0 += dt;
      return r;
    };

  // Warm up at t = 0 (hover blend still active), then let the orbit ramp in.
  for (int i = 0; i < kWarmUpSolves; ++i) {
    const uav_mpc::SolveResult r = solve_at(false);
    ASSERT_EQ(r.status, uav_mpc::SolverStatus::Success);
  }

  bool all_ok = true;
  for (int i = 0; i < kSamples; ++i) {
    const uav_mpc::SolveResult r = solve_at(true);
    all_ok = all_ok && (r.status == uav_mpc::SolverStatus::Success);
  }

  reportPercentiles("figure8", times);
  std::vector<double> sorted = times;
  std::sort(sorted.begin(), sorted.end());
  const double median = sorted.size() % 2 == 1 ?
    sorted[sorted.size() / 2] :
    0.5 * (sorted[sorted.size() / 2 - 1] + sorted[sorted.size() / 2]);

  EXPECT_TRUE(all_ok) << "at least one figure-8 solve returned a non-Success status";
  EXPECT_LE(median, budgetMs("UAV_MPC_SOLVE_MEDIAN_BUDGET_MS", kMedianBudgetMs))
    << "median figure-8 solve exceeded the 1 ms budget";
  EXPECT_LE(percentile(sorted, 0.99), budgetMs("UAV_MPC_SOLVE_P99_BUDGET_MS", kP99BudgetMs))
    << "p99 figure-8 solve exceeded the 2 ms budget";
}

// Recovery from a bad initial guess must not blow the budget either — this is the path taken
// after an infeasibility, in flight.
TEST(NmpcSolveTime, ColdStartSolveWithinRelaxedBudget)
{
  uav_mpc::AcadosWrapper solver;
  ASSERT_NO_THROW(solver = makeSolver());

  const uav_mpc::QuadrotorParams airframe = loadX500();
  const uav_mpc::QuadrotorDynamics<double, uav_mpc::AttitudeRep::Quaternion> dyn(airframe);
  const double hover_thrust = dyn.hoverThrustPerRotor();
  const int n = solver.horizonSteps();

  const Eigen::Vector3d center(0.0, 0.0, 1.5);
  const Eigen::VectorXd x_hover = hoverState(center);
  Eigen::VectorXd u_hover(4);
  u_hover.setConstant(hover_thrust);
  std::vector<Eigen::VectorXd> x_refs(static_cast<std::size_t>(n) + 1, x_hover);
  std::vector<Eigen::VectorXd> u_refs(static_cast<std::size_t>(n), u_hover);
  solver.setReferenceHorizon(x_refs, u_refs);

  std::mt19937 rng(1234);

  // Warm up once from hover, then discard the guess on every sample with resetToHover().
  Eigen::VectorXd x0 = x_hover;
  perturbState(&x0, rng);
  solver.setInitialState(x0);
  solver.resetToHover(x0, hover_thrust);
  ASSERT_EQ(solver.solve().status, uav_mpc::SolverStatus::Success);

  std::vector<double> times;
  times.reserve(static_cast<std::size_t>(kSamples));
  bool all_ok = true;
  for (int i = 0; i < kSamples; ++i) {
    x0 = x_hover;  // reset so perturbState does not random-walk (§5.2)
    perturbState(&x0, rng);
    // No warm start is available: blow away the primal/dual guess before every solve.
    solver.resetToHover(x0, hover_thrust);
    solver.setInitialState(x0);

    const auto t0 = std::chrono::steady_clock::now();
    const uav_mpc::SolveResult r = solver.solve();
    const auto t1 = std::chrono::steady_clock::now();
    times.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());

    all_ok = all_ok && (r.status == uav_mpc::SolverStatus::Success);
  }

  reportPercentiles("coldstart", times);
  std::vector<double> sorted = times;
  std::sort(sorted.begin(), sorted.end());

  // Relaxed budget: 5x the normal p99. Documented in docs/TUNING_GUIDE.md, not just here.
  constexpr double kColdStartP99BudgetMs = 5.0 * kP99BudgetMs;
  EXPECT_TRUE(all_ok) << "at least one cold-start solve returned a non-Success status";
  EXPECT_LE(percentile(sorted, 0.99), kColdStartP99BudgetMs)
    << "p99 cold-start solve exceeded the 10 ms budget";
}

// Guards against a silently mis-generated solver being flown.
TEST(NmpcSolveTime, GeneratedModelHashMatchesCheckedInHash)
{
#ifdef UAV_MPC_WITH_ACADOS
  const std::string hash_file = std::string(UAV_MPC_CODEGEN_DIR) + "/MODEL_HASH";
  std::ifstream f(hash_file);
  ASSERT_TRUE(f.good()) << "cannot open " << hash_file;
  std::string expected;
  std::getline(f, expected);
  // MODEL_HASH is a bare hex string, newline-terminated; trim any trailing whitespace.
  while (!expected.empty() && std::isspace(static_cast<unsigned char>(expected.back()))) {
    expected.pop_back();
  }

  const std::string actual = uav_mpc::AcadosWrapper::generatedModelHash();
  EXPECT_EQ(actual, expected)
    << "The generated solver does not match codegen/MODEL_HASH. The model source "
       "changed since the solver was generated — re-run codegen/generate_acados_solver.py "
       "and commit the fresh solver + hash. NEVER fly a stale solver.";
#else
  GTEST_SKIP() << "stub backend: generatedModelHash() is empty by design";
#endif
}
