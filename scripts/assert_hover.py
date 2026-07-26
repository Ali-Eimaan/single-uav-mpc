"""SKELETON — no implementation. See IMPLEMENTATION_GUIDE.md §14.3.

Post-flight assertions for the SITL smoke test. Lives in scripts/ rather than inside the
workflow so the same checks can be run locally against a bag from a failed CI run:

    python scripts/assert_hover.py analysis/output/ci_hover_bag --window 10

Exit code 0 = all assertions passed. Non-zero = at least one failed. Every metric is printed
regardless of the outcome — the numbers in the job log are the evidence, not the green tick.
"""

from __future__ import annotations

import argparse

# Thresholds. Deliberately looser than the unit tests: a shared CI runner is not a flight
# computer. Changing any of these is changing an acceptance criterion — update
# IMPLEMENTATION_GUIDE.md §1 in the same commit.
MAX_TIME_TO_TRACKING_S = 20.0
MAX_RMS_POSITION_ERROR_M = 0.15
MAX_PEAK_POSITION_ERROR_M = 0.30
MAX_SOLVER_FAILURES = 0
MAX_P99_SOLVE_MS = 5.0
MIN_ALTITUDE_FRACTION = 0.5  # of takeoff_altitude, at any point after takeoff


def parse_args() -> argparse.Namespace:
    """TODO(deepseek): bag_path (positional), --window (steady-state seconds to score),
    --takeoff-altitude, --json (dump the metrics for the workflow summary)."""
    raise NotImplementedError


def load_metrics(bag_path: str, window_s: float, takeoff_altitude: float) -> dict:
    """Read the bag and compute every metric named above.

    TODO(deepseek): read /nmpc_node/status and /fmu/out/vehicle_local_position with `rosbags`.
    Locate the first sample where controller_state == STATE_TRACKING; score the `window_s`
    seconds that follow. Return a plain dict of metric name -> value so the caller can print
    and serialise it without knowing what was measured.
    """
    raise NotImplementedError


def check(metrics: dict) -> "list[str]":
    """Return the list of human-readable failures; empty means everything passed.

    TODO(deepseek): one message per violated threshold, each naming the measured value, the
    threshold, and the constant above that encodes it.
    """
    raise NotImplementedError


def main() -> int:
    """TODO(deepseek): load -> print the metric table -> check -> print failures -> return
    0 or 1. Print the table even when loading succeeded but assertions failed; a bare
    'assertion failed' with no numbers wastes the one artifact you get from a CI failure."""
    raise NotImplementedError


if __name__ == "__main__":
    raise SystemExit(main())
