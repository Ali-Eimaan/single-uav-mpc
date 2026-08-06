"""Produces media/solve_time_histogram.png, the README's central quantitative claim (§12.1).

    python analysis/solve_time_benchmark.py --samples 10000 --out media/solve_time_histogram.png
    python analysis/solve_time_benchmark.py --from-bag analysis/output/figure8_2026xxxx

Two modes:
  synthetic  drive the generated solver directly from Python across a sweep of operating
             points — clean numbers, no ROS, reproducible in CI
  from-bag   read /nmpc_node/status out of a recorded flight — the honest numbers, including
             everything the ROS layer costs

Report BOTH in the README. Reporting only the synthetic number is the kind of thing a
reviewer catches, and it costs more credibility than the slower figure ever would.

Method notes that change the answer (§12.1): 100 discarded warm-up solves; pin to a single
core if psutil is available (recorded in the output); measure with time.perf_counter_ns;
advance the reference by dt each iteration so the warm start is realistic; from-bag mode
drops the first 2 s of startup transient (stated in the plot caption).

Exit non-zero when the p99 exceeds the budget — the script doubles as a CI gate.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import platform
import sys
import time
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent
GENERATED_DIR = REPO_ROOT / "codegen" / "codegen_output" / "c_generated_code"
SOLVER_NAME = "quadrotor"          # model.name in codegen/quadrotor_model.py
ACADOS_COMMIT_FILE = REPO_ROOT / "codegen" / "ACADOS_COMMIT"

# Figure-8 primitive defaults (must match launch/figure8.launch.py defaults).
DEFAULT_FIGURE8 = dict(amplitude_x=2.0, amplitude_y=2.0, period=8.0, altitude=1.5)


def load_config(path: Path) -> dict:
    """Read a ROS 2 param-style YAML (or a flat YAML) and return the flat parameter dict."""
    doc = yaml.safe_load(path.read_text())
    if not isinstance(doc, dict):
        raise RuntimeError(f"config {path} must be a YAML mapping")
    block = doc.get("/**")
    if isinstance(block, dict) and isinstance(block.get("ros__parameters"), dict):
        return block["ros__parameters"]
    return doc


def cpu_model() -> str:
    try:
        for line in Path("/proc/cpuinfo").read_text().splitlines():
            if line.lower().startswith("model name"):
                return line.split(":", 1)[1].strip()
    except OSError:
        pass
    return platform.processor() or platform.machine()


def acados_commit() -> str:
    try:
        for line in ACADOS_COMMIT_FILE.read_text().splitlines():
            line = line.strip()
            if len(line) == 40 and all(c in "0123456789abcdef" for c in line.lower()):
                return line
    except OSError:
        pass
    return "unpinned"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=int, default=10000, help="solves per scenario")
    parser.add_argument("--out", default=str(REPO_ROOT / "media" / "solve_time_histogram.png"))
    parser.add_argument(
        "--from-bag", metavar="BAG", default=None,
        help="measure from a recorded flight instead of synthetic solves")
    parser.add_argument(
        "--horizon", type=int, default=None,
        help="horizon steps for the plot title and reference generation (default: from "
             "nmpc_params.yaml; the generated solver's N is fixed by codegen)")
    parser.add_argument(
        "--scenario", choices=["hover", "figure8", "coldstart", "all"], default="hover",
        help="synthetic operating point (ignored in --from-bag mode)")
    parser.add_argument(
        "--budget-ms", type=float, default=2.0,
        help="p99 budget; exit code is non-zero when any scenario exceeds it")
    parser.add_argument(
        "--json", metavar="PATH", default=None,
        help="dump percentiles and metadata as JSON alongside the PNG")
    return parser.parse_args()


def figure8(t: float, p: dict) -> tuple[np.ndarray, np.ndarray]:
    """Figure-8 position and velocity, matching trajectory_generator.cpp sampleFigure8.

    x = Ax sin(wt), y = (Ay/2) sin(2wt), z = altitude.
    """
    w = 2.0 * math.pi / p["period"]
    wt = w * t
    x = p["amplitude_x"] * math.sin(wt)
    y = 0.5 * p["amplitude_y"] * math.sin(2.0 * wt)
    z = p["altitude"]
    vx = p["amplitude_x"] * w * math.cos(wt)
    vy = p["amplitude_y"] * w * math.cos(2.0 * wt)
    return np.array([x, y, z]), np.array([vx, vy, 0.0])


def yaw_quat_wxyz(yaw: float) -> np.ndarray:
    return np.array([math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)])


def benchmark_synthetic(scenario: str, samples: int, horizon: int) -> tuple[list[float], bool]:
    """Time `samples` solves of the generated solver (wall time in ms, perf_counter_ns).

    - acados_template.AcadosOcpSolver on codegen/codegen_output/c_generated_code
    - 100 discarded warm-up solves (first-touch page faults and CPU frequency ramp would
      otherwise contaminate the left tail)
    - pin to a single core if psutil is available; the flag is returned so it lands in the
      JSON metadata
    - the reference is advanced by dt each iteration so the warm start is realistic
    - `coldstart` constructs a fresh solver every iteration (the wrapper's reinitialise
      path); the others reuse one solver.
    """
    try:
        from acados_template import AcadosOcpSolver
    except ImportError as exc:
        raise SystemExit(
            "acados_template is not importable — install it from the pinned acados tree "
            "(see .deepseek/02_ENVIRONMENT.md), and run codegen/generate_acados_solver.py "
            "first so the solver exists.") from exc
    if not (GENERATED_DIR / f"acados_solver_{SOLVER_NAME}.json").is_file():
        raise SystemExit(
            f"generated solver not found in {GENERATED_DIR} — run "
            "codegen/generate_acados_solver.py first.")

    nmpc = load_config(REPO_ROOT / "uav_mpc" / "config" / "nmpc_params.yaml")
    traj = load_config(REPO_ROOT / "uav_mpc" / "config" / "trajectory_params.yaml")
    traj = traj.get("trajectory", {})
    fig8 = {**DEFAULT_FIGURE8, **{k: traj[k] for k in DEFAULT_FIGURE8 if k in traj}}

    n = nmpc["horizon_steps"]          # 20 — the generated solver's N
    tf = float(nmpc["horizon_time"])   # 1.0 s
    dt_h = tf / n
    dt_ctrl = 1.0 / float(nmpc["control_rate_hz"])
    g = 9.80665
    airframe = load_config(
        REPO_ROOT / "uav_mpc" / "params" / "x500_calibration.yaml")["airframe"]
    t_hover = airframe["mass"] * g / 4.0  # per-rotor trim thrust

    # Reference is encoded as stage yref (position/velocity/rates/input) + the q_ref online
    # parameter (attitude enters the NONLINEAR_LS residual through the error quaternion).
    def stage_refs(t_phase: float):
        if scenario == "figure8":
            p_ref, v_ref = figure8(t_phase, fig8)
            psi = math.atan2(v_ref[1], v_ref[0]) if fig8["period"] > 0 else 0.0
        else:  # hover and coldstart share the same hover trim point
            p_ref, v_ref = np.array([0.0, 0.0, fig8["altitude"]]), np.zeros(3)
            psi = 0.0
        yref = np.concatenate([p_ref, v_ref, np.zeros(3), np.zeros(3),
                               t_hover * np.ones(4)])
        yref_e = np.concatenate([p_ref, v_ref, np.zeros(3), np.zeros(3)])
        p = np.concatenate([np.zeros(3), [1.0], yaw_quat_wxyz(psi)])  # wind, mass_scale, q_ref
        return yref, yref_e, p

    times_ms: list[float] = []
    pinned = _pin_to_core()
    print(f"[benchmark] scenario={scenario} samples={samples} horizon={n} "
          f"core_pinned={pinned}", file=sys.stderr)

    def run_one(t_phase: float):
        solver.set(0, "x", np.zeros(13))
        for k in range(n + 1):
            yref, yref_e, p = stage_refs(t_phase + k * dt_h)
            if k < n:
                solver.cost_set(k, "yref", yref)
                solver.parameter_set(k, "p", p)
            else:
                solver.cost_set(n, "yref", yref_e)
                solver.parameter_set(n, "p", p)
        t0 = time.perf_counter_ns()
        solver.solve()
        return (time.perf_counter_ns() - t0) / 1e6

    for i in range(samples + 100):
        if scenario == "coldstart":
            # Fresh solver every iteration: construction + first solve, like a reinit.
            solver = AcadosOcpSolver(SOLVER_NAME, str(GENERATED_DIR), build=False,
                                     verbose=False)
        elif i == 0:
            solver = AcadosOcpSolver(SOLVER_NAME, str(GENERATED_DIR), build=False,
                                     verbose=False)
        dt_ms = run_one(i * dt_ctrl)
        if i >= 100:  # discard the warm-up solves
            times_ms.append(dt_ms)
    return times_ms, pinned


def _pin_to_core() -> bool:
    try:
        import psutil
        psutil.Process().cpu_affinity([0])  # Linux; core 0
        return True
    except Exception:
        return False


def load_from_bag(bag_path: str) -> dict:
    """Extract solve times, loop durations and status codes from a rosbag.

    Reads /nmpc_node/status with `rosbags` (no sourced ROS env needed), aligned on the bag
    record timestamp. The first 2 s of startup transient are dropped — the plot caption
    says so.
    """
    sys.path.insert(0, str(HERE))  # bag_utils lives in analysis/
    from bag_utils import read_topics, require_topic
    data = read_topics(Path(bag_path), ("/nmpc_node/status",))
    require_topic(data, "/nmpc_node/status", Path(bag_path))
    status = data["/nmpc_node/status"]
    t0 = status[0][0]
    drop_ns = 2.0 * 1e9
    kept = [(t, m) for t, m in status if t - t0 >= drop_ns]
    return {
        "wall_time_ms": [m.solver.wall_time_ms for _, m in kept],
        "solve_time_ms": [m.solver.solve_time_ms for _, m in kept],
        "loop_duration_ms": [m.loop_duration_ms for _, m in kept],
        "status": [m.solver.status for _, m in kept],
        "startup_dropped_s": 2.0,
        "source": str(bag_path),
        "kept_samples": len(kept),
    }


def percentiles(times_ms: list[float], budget_ms: float) -> dict:
    """min, median, mean, p90, p95, p99, p99.9, max, count, and the over-budget fraction.

    The over-budget fraction is the number that actually matters for a 100 Hz loop.
    """
    a = np.asarray(times_ms, dtype=float)
    if a.size == 0:
        raise ValueError("no solve times to summarise")
    out = {"count": int(a.size)}
    for key, q in (("min", 0), ("median", 50), ("p90", 90), ("p95", 95),
                   ("p99", 99), ("p999", 99.9), ("max", 100)):
        out[key] = float(np.percentile(a, q))
    out["mean"] = float(np.mean(a))
    out["over_budget_fraction"] = float(np.mean(a > budget_ms))
    return out


def plot_histogram(results: dict, out_path: Path, budget_ms: float, horizon: int) -> None:
    """Render the README figure: log-x histogram, one panel per scenario.

    Median and p99 are marked with labelled vertical lines, the budget with a shaded
    region. The title names the CPU model, the horizon, and the acados commit — a
    solve-time plot without the hardware named is worthless.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    n_panels = len(results)
    fig, axes = plt.subplots(1, n_panels, figsize=(5.5 * n_panels, 4.0), squeeze=False)
    for ax, (label, times) in zip(axes[0], results.items()):
        a = np.asarray(times, dtype=float)
        if a.size == 0:
            ax.text(0.5, 0.5, "no data", ha="center", va="center", transform=ax.transAxes)
            continue
        bins = np.logspace(np.log10(max(a.min() * 0.7, 1e-3)),
                           np.log10(a.max() * 1.3), 80)
        ax.hist(a, bins=bins, color="#1f77b4", alpha=0.85)
        ax.set_xscale("log")
        ax.set_xlabel("solve time [ms]")
        ax.set_ylabel("count")
        median = float(np.median(a))
        p99 = float(np.percentile(a, 99))
        ax.axvline(median, color="k", ls="--", lw=1.2)
        ax.axvline(p99, color="crimson", ls=":", lw=1.4)
        ax.axvspan(0, budget_ms, color="green", alpha=0.12)
        ax.text(median, ax.get_ylim()[1] * 0.95, f"median {median:.2f}",
                rotation=90, va="top", fontsize=8)
        ax.text(p99, ax.get_ylim()[1] * 0.75, f"p99 {p99:.2f}",
                rotation=90, va="top", fontsize=8, color="crimson")
        ax.set_title(label)
        if label == "from-bag":
            ax.text(0.02, 0.02,
                    "first 2 s dropped (startup transient)", transform=ax.transAxes,
                    fontsize=7, color="dimgray")

    fig.suptitle(f"solve time — {cpu_model()} · horizon {horizon} steps · "
                 f"acados {acados_commit()}", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    print(f"[benchmark] wrote {out_path}", file=sys.stderr)


def _md_table(scenario: str, pct: dict, budget_ms: float) -> str:
    hdr = ("| scenario | n | min | median | mean | p90 | p95 | p99 | p99.9 | max | "
           "over 2 ms |")
    sep = "|---|---|---|---|---|---|---|---|---|---|---|"
    over = f"{pct['over_budget_fraction'] * 100:.2f}%"
    row = (f"| {scenario} | {pct['count']} | {pct['min']:.3f} | {pct['median']:.3f} "
           f"| {pct['mean']:.3f} | {pct['p90']:.3f} | {pct['p95']:.3f} | {pct['p99']:.3f} "
           f"| {pct['p999']:.3f} | {pct['max']:.3f} | {over} |")
    return "\n".join([hdr, sep, row])


def main() -> int:
    args = parse_args()
    nmpc = load_config(REPO_ROOT / "uav_mpc" / "config" / "nmpc_params.yaml")
    horizon = args.horizon or int(nmpc["horizon_steps"])
    budget = args.budget_ms

    results: dict[str, list[float]] = {}
    tables: list[str] = []
    meta = {"cpu": cpu_model(), "horizon": horizon, "acados_commit": acados_commit(),
            "budget_ms": budget, "core_pinned": None}
    pcts: dict[str, dict] = {}

    if args.from_bag:
        bag_data = load_from_bag(args.from_bag)
        results["from-bag"] = bag_data["wall_time_ms"]
        meta["core_pinned"] = None
        meta.update({k: v for k, v in bag_data.items() if k != "wall_time_ms"})
        pcts["from-bag"] = percentiles(results["from-bag"], budget)
        tables.append(_md_table(f"from-bag ({bag_data['source']})", pcts["from-bag"], budget))
        meta["from_bag"] = {k: pcts["from-bag"][k] for k in
                            ("p99", "median", "mean", "over_budget_fraction")}
    else:
        scenarios = (["hover", "figure8", "coldstart"] if args.scenario == "all"
                     else [args.scenario])
        for s in scenarios:
            times, pinned = benchmark_synthetic(s, args.samples, horizon)
            results[s] = times
            meta["core_pinned"] = meta["core_pinned"] if meta["core_pinned"] is not None \
                else pinned
            pcts[s] = percentiles(results[s], budget)
            tables.append(_md_table(s, pcts[s], budget))

    print("\n".join(tables))

    plot_histogram(results, Path(args.out), budget, horizon)

    if args.json:
        payload = {**meta,
                   "scenarios": {s: pcts[s] for s in pcts},
                   "p99_over_budget": {s: pcts[s]["p99"] > budget for s in pcts}}
        Path(args.json).write_text(json.dumps(payload, indent=2))
        print(f"[benchmark] wrote {args.json}", file=sys.stderr)

    exceeded = [s for s in pcts if pcts[s]["p99"] > budget]
    if exceeded:
        print(f"[benchmark] p99 exceeds {budget} ms budget in: {', '.join(exceeded)}",
              file=sys.stderr)
        return 1
    print(f"[benchmark] p99 within {budget} ms budget for all scenarios")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
