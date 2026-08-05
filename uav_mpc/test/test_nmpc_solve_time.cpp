// Copyright (c) 2026 Ali-Eimaan.
// SPDX-License-Identifier: BSD-3-Clause
//
// SKELETON — test bodies not implemented. See .deepseek/10_TESTS.md §10.1.
//
// THE gate test. CI fails if the NMPC solve regresses past the real-time budget, which is the
// single claim this repo is making. Keep it honest:
//   - Release build only (the CMake test target inherits CMAKE_BUILD_TYPE)
//   - measured with steady_clock around the solve, not acados' self-report
//   - reported as median / p95 / p99 / max over N samples, never a single mean
//   - the threshold is on the p99, because a 100 Hz loop that misses once per second is broken

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <vector>

#include "uav_mpc/acados_wrapper.hpp"
#include "uav_mpc/quadrotor_dynamics.hpp"
#include "uav_mpc/trajectory_generator.hpp"

namespace
{

/// Budget at 100 Hz. The solve must fit with room for the rest of the callback.
constexpr double kP99BudgetMs = 2.0;
constexpr double kMedianBudgetMs = 1.0;
constexpr int kSamples = 10000;

/// Loads params/x500_calibration.yaml relative to the test binary.
// TODO(deepseek): implement — resolve the path via ament_index_cpp or a compile definition
// set in CMakeLists; do NOT hard-code an absolute path.
uav_mpc::QuadrotorParams loadX500();

/// Builds an initialised solver with the nominal config from config/nmpc_params.yaml.
// TODO(deepseek): implement
uav_mpc::AcadosWrapper makeSolver();

}  // namespace

// Hover at the origin: the easiest problem. If this is slow, nothing else matters.
TEST(NmpcSolveTime, HoverSolveWithinBudget)
{
  // TODO(deepseek): initialise the solver, set a hover reference across the horizon, warm it up
  // with 100 discarded solves, then time kSamples solves from a perturbed x0 (small random
  // position/velocity noise, so the warm start is exercised realistically).
  // ASSERT every solve returned SolverStatus::Success.
  // EXPECT median <= kMedianBudgetMs and p99 <= kP99BudgetMs.
  // Print the full percentile table with RecordProperty() so CI logs are useful.
  GTEST_SKIP() << "not implemented";
}

// Aggressive figure-8: the case the README claims. Same budget applies.
TEST(NmpcSolveTime, Figure8SolveWithinBudget)
{
  // TODO(deepseek): 3 m amplitude, 5 s period; step the reference horizon forward by dt each
  // sample so the warm start sees a genuinely moving target.
  GTEST_SKIP() << "not implemented";
}

// Recovery from a bad initial guess must not blow the budget either — this is the path taken
// after an infeasibility, in flight.
TEST(NmpcSolveTime, ColdStartSolveWithinRelaxedBudget)
{
  // TODO(deepseek): resetToHover() before every solve so no warm start is available.
  // Relaxed budget: p99 <= 5 * kP99BudgetMs. Document the number in the guide, not just here.
  GTEST_SKIP() << "not implemented";
}

// Guards against a silently mis-generated solver being flown.
TEST(NmpcSolveTime, GeneratedModelHashMatchesCheckedInHash)
{
  // TODO(deepseek): compare AcadosWrapper::generatedModelHash() with the contents of
  // codegen/MODEL_HASH. Fail with a message telling the developer to re-run codegen.
  GTEST_SKIP() << "not implemented";
}
