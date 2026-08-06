"""Post-flight assertions for the SITL smoke test (§14.3). Lives in scripts/ rather than
inside the workflow so the same checks can be run locally against a bag from a failed CI run:

    python scripts/assert_hover.py analysis/output/ci_hover_bag --window 10

Exit code 0 = all assertions passed. Non-zero = at least one failed. Every metric is printed
regardless of the outcome — the numbers in the job log are the evidence, not the green tick.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "analysis"))  # bag_utils lives next to the notebooks
from bag_utils import read_topics, require_topic  # noqa: E402

# Thresholds. Deliberately looser than the unit tests: a shared CI runner is not a flight
# computer. Changing any of these is changing an acceptance criterion — update
# .deepseek/01_OVERVIEW.md §1 in the same commit.
MAX_TIME_TO_TRACKING_S = 20.0
MAX_RMS_POSITION_ERROR_M = 0.15
MAX_PEAK_POSITION_ERROR_M = 0.30
MAX_SOLVER_FAILURES = 0
MAX_P99_SOLVE_MS = 5.0
MIN_ALTITUDE_FRACTION = 0.5  # of takeoff_altitude, at any point after takeoff

STATE_TRACKING = 3  # uav_mpc/msg/NmpcStatus.controller_state
STATUS_SUCCESS = 0  # uav_mpc/msg/SolverDiagnostics.status


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_path", help="rosbag2 directory recorded by the smoke test")
    parser.add_argument(
        "--window", type=float, default=10.0,
        help="steady-state seconds after STATE_TRACKING to score")
    parser.add_argument(
        "--takeoff-altitude", type=float, default=1.5,
        help="commanded takeoff altitude [m] AGL; the MIN_ALTITUDE_FRACTION check is "
             "relative to this")
    parser.add_argument(
        "--max-rms", type=float, default=MAX_RMS_POSITION_ERROR_M,
        help="RMS position error limit [m] (defaults to MAX_RMS_POSITION_ERROR_M); the "
             "figure-8 job relaxes this")
    parser.add_argument(
        "--json", metavar="PATH", default=None,
        help="dump the metrics as JSON (workflow summary consumes this)")
    return parser.parse_args()


def _percentile(values: list[float], q: float) -> float:
    if not values:
        return math.nan
    return float(np.percentile(values, q))


def load_metrics(bag_path: str, window_s: float, takeoff_altitude: float) -> dict:
    """Read the bag and compute every metric named above.

    /nmpc_node/status is the controller's own error and solver telemetry; altitude comes
    from /fmu/out/vehicle_local_position (NED — negated to AGL). Both are aligned on the
    bag record timestamp (one time base, §12). The window starts at the first STATE_TRACKING
    sample and runs `window_s` seconds on the record clock.
    """
    bag = Path(bag_path)
    data = read_topics(
        bag, ("/nmpc_node/status", "/fmu/out/vehicle_local_position"))
    require_topic(data, "/nmpc_node/status", bag)
    require_topic(data, "/fmu/out/vehicle_local_position", bag)

    status = data["/nmpc_node/status"]
    local = data["/fmu/out/vehicle_local_position"]
    t0 = status[0][0]
    tracking = [(t, m) for t, m in status if m.controller_state == STATE_TRACKING]

    metrics: dict = {
        "bag_path": str(bag),
        "takeoff_altitude": takeoff_altitude,
        "window_s": window_s,
        "n_status_samples": len(status),
        "n_local_samples": len(local),
        "tracking_started": bool(tracking),
    }
    if not tracking:
        metrics.update({
            "time_to_tracking_s": math.inf,
            "rms_position_error_m": math.nan,
            "peak_position_error_m": math.nan,
            "solver_failure_count": math.nan,
            "p99_solve_time_ms": math.nan,
            "min_altitude_fraction": math.nan,
            "scored_duration_s": 0.0,
        })
        return metrics

    t_track = tracking[0][0]
    metrics["time_to_tracking_s"] = (t_track - t0) / 1e9
    t_end = t_track + window_s * 1e9
    win_status = [(t, m) for t, m in status if t_track <= t <= t_end]
    win_local = [(t, m) for t, m in local if t_track <= t <= t_end]
    metrics["scored_duration_s"] = (win_status[-1][0] - t_track) / 1e9 if win_status else 0.0

    errors = [m.position_error_norm for _, m in win_status]
    failures = [m.solver.status for _, m in win_status]
    solve_times = [m.solver.wall_time_ms for _, m in win_status]
    metrics["rms_position_error_m"] = (
        float(np.sqrt(np.mean(np.square(errors)))) if errors else math.nan)
    metrics["peak_position_error_m"] = float(max(errors)) if errors else math.nan
    metrics["solver_failure_count"] = (
        int(sum(1 for s in failures if s != STATUS_SUCCESS)) if failures else math.nan)
    metrics["p99_solve_time_ms"] = _percentile(solve_times, 99.0)

    # Altitude AGL = -z (PX4 local position is NED, z down). Fraction of the commanded
    # takeoff altitude, minimum over the scored window.
    altitudes = [-m.z for _, m in win_local]
    if altitudes and takeoff_altitude > 0.0:
        metrics["min_altitude_fraction"] = min(altitudes) / takeoff_altitude
    else:
        metrics["min_altitude_fraction"] = math.nan
    return metrics


def check(metrics: dict) -> list[str]:
    """Return the list of human-readable failures; empty means everything passed.

    One message per violated threshold, each naming the measured value, the threshold, and
    the constant above that encodes it.
    """
    failures: list[str] = []
    if not metrics["tracking_started"]:
        return [
            f"vehicle never reached STATE_TRACKING "
            f"(time_to_tracking_s = inf > MAX_TIME_TO_TRACKING_S = {MAX_TIME_TO_TRACKING_S})"]

    def cmp(name: str, value: float, limit: float, const: str, unit: str, op: str) -> None:
        if math.isnan(value):
            failures.append(f"{name} = NaN (scored window empty?): no data to compare "
                            f"against {const} = {limit} {unit}")
            return
        if op == "<=" and not value <= limit:
            failures.append(
                f"{name} = {value:.3f} {unit} > {const} = {limit} {unit}")
        elif op == ">=" and not value >= limit:
            failures.append(
                f"{name} = {value:.3f} {unit} < {const} = {limit} {unit}")

    cmp("time_to_tracking_s", metrics["time_to_tracking_s"],
        MAX_TIME_TO_TRACKING_S, "MAX_TIME_TO_TRACKING_S", "s", "<=")
    rms_limit = metrics["rms_limit"]
    const = "MAX_RMS_POSITION_ERROR_M"
    if rms_limit != MAX_RMS_POSITION_ERROR_M:
        const += f" (overridden by --max-rms={rms_limit})"
    cmp("rms_position_error_m", metrics["rms_position_error_m"],
        rms_limit, const, "m", "<=")
    cmp("peak_position_error_m", metrics["peak_position_error_m"],
        MAX_PEAK_POSITION_ERROR_M, "MAX_PEAK_POSITION_ERROR_M", "m", "<=")
    cmp("solver_failure_count", float(metrics["solver_failure_count"]),
        MAX_SOLVER_FAILURES, "MAX_SOLVER_FAILURES", "", "<=")
    cmp("p99_solve_time_ms", metrics["p99_solve_time_ms"],
        MAX_P99_SOLVE_MS, "MAX_P99_SOLVE_MS", "ms", "<=")
    cmp("min_altitude_fraction", metrics["min_altitude_fraction"],
        MIN_ALTITUDE_FRACTION, "MIN_ALTITUDE_FRACTION", "", ">=")
    return failures


def _print_table(metrics: dict, failures: list[str]) -> None:
    rows = [
        ("time_to_tracking_s", "s", None),
        ("rms_position_error_m", "m", metrics.get("rms_limit")),
        ("peak_position_error_m", "m", None),
        ("solver_failure_count", "", None),
        ("p99_solve_time_ms", "ms", None),
        ("min_altitude_fraction", "", None),
        ("scored_duration_s", "s", None),
        ("n_status_samples", "", None),
        ("n_local_samples", "", None),
    ]
    print(f"assert_hover: bag {metrics['bag_path']} "
          f"(tracking_started={metrics['tracking_started']})")
    print(f"{'metric':<24} {'value':>12} {'limit':>10}  pass")
    for key, unit, limit in rows:
        value = metrics.get(key, math.nan)
        if isinstance(value, float) and math.isnan(value):
            rendered, ok = "nan", "?"
        elif isinstance(value, float) and math.isinf(value):
            rendered, ok = "inf", "?"
        else:
            rendered = f"{value:.4f}".rstrip("0").rstrip(".") if isinstance(value, float) \
                else str(value)
            if limit is not None and not math.isnan(metrics.get(key, math.nan)):
                ok = "yes" if value <= limit else "NO"
            else:
                ok = ""
        print(f"{key:<24} {rendered:>12} {str(limit):>10}  {ok}")
    if failures:
        print("\nfailures:")
        for f in failures:
            print(f"  FAIL: {f}")


def _json_safe(value):
    if isinstance(value, float) and (math.isnan(value) or math.isinf(value)):
        return None
    return value


def main() -> int:
    args = parse_args()
    try:
        metrics = load_metrics(args.bag_path, args.window, args.takeoff_altitude)
    except (FileNotFoundError, RuntimeError) as exc:
        print(f"assert_hover: {exc}", file=sys.stderr)
        return 2
    metrics["rms_limit"] = args.max_rms
    failures = check(metrics)
    _print_table(metrics, failures)

    if args.json:
        out = {k: _json_safe(v) for k, v in metrics.items()}
        out["failures"] = failures
        Path(args.json).write_text(json.dumps(out, indent=2))
        print(f"assert_hover: metrics written to {args.json}")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
