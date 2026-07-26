"""SKELETON — no implementation. See IMPLEMENTATION_GUIDE.md §12.1.

Produces media/solve_time_histogram.png, the README's central quantitative claim.

    python analysis/solve_time_benchmark.py --samples 10000 --out media/solve_time_histogram.png
    python analysis/solve_time_benchmark.py --from-bag analysis/output/figure8_2026xxxx

Two modes:
  synthetic  drive the generated solver directly from Python across a sweep of operating
             points — clean numbers, no ROS, reproducible in CI
  from-bag   read /nmpc_node/status out of a recorded flight — the honest numbers, including
             everything the ROS layer costs

Report BOTH in the README. Reporting only the synthetic number is the kind of thing a
reviewer catches, and it costs more credibility than the slower figure ever would.
"""

from __future__ import annotations

import argparse


def parse_args() -> argparse.Namespace:
    """TODO(deepseek): --samples, --out, --from-bag, --horizon, --scenario
    {hover,figure8,coldstart,all}, --json (dump the raw percentiles alongside the PNG)."""
    raise NotImplementedError


def benchmark_synthetic(scenario: str, samples: int, horizon: int) -> "list[float]":
    """Time `samples` solves of the generated solver.

    TODO(deepseek):
      - use acados_template.AcadosOcpSolver on codegen/codegen_output
      - warm up with 100 discarded solves (first-touch page faults and CPU frequency ramp
        otherwise contaminate the left tail)
      - measure with time.perf_counter_ns around solver.solve()
      - advance the reference by dt each iteration so the warm start is realistic
      - pin to a single core if psutil is available, and record whether it succeeded
      - return wall times in milliseconds
    """
    raise NotImplementedError


def load_from_bag(bag_path: str) -> "dict[str, list[float]]":
    """Extract solve times, loop durations and status codes from a rosbag.

    TODO(deepseek): read with `rosbags` (no sourced ROS env needed). Topic
    /nmpc_node/status, fields solver.wall_time_ms, solver.solve_time_ms, loop_duration_ms,
    solver.status. Drop the first 2 s (startup transient) and say so in the plot caption.
    """
    raise NotImplementedError


def percentiles(times_ms: "list[float]") -> "dict[str, float]":
    """TODO(deepseek): min, median, mean, p90, p95, p99, p99.9, max, plus the count and the
    fraction over the 2 ms budget. The over-budget fraction is the number that actually
    matters for a 100 Hz loop."""
    raise NotImplementedError


def plot_histogram(results: dict, out_path: str) -> None:
    """Render the README figure.

    TODO(deepseek): log-x histogram, one panel per scenario. Mark the median and p99 with
    labelled vertical lines and the 2 ms budget with a shaded region. Title states the CPU
    model, the horizon, and the acados commit — a solve-time plot without the hardware named
    is worthless. Save at dpi=150 with bbox_inches="tight".
    """
    raise NotImplementedError


def main() -> int:
    """TODO(deepseek): dispatch on the mode, print the percentile table to stdout as markdown
    (so it can be pasted into the README), write the PNG, and exit non-zero if p99 exceeds the
    budget — that makes this script usable as a CI gate too."""
    raise NotImplementedError


if __name__ == "__main__":
    raise SystemExit(main())
