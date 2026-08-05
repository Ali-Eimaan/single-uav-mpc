# §12 · Analysis and media

**Governs:** `analysis/solve_time_benchmark.py`, `analysis/tracking_error_analysis.ipynb`,
`analysis/disturbance_sweep.ipynb`, `scripts/assert_hover.py`, `media/`
**Milestone:** M9
**Done when:** every number in the README is reproducible by running something in `analysis/`.

---

## 12.1 `solve_time_benchmark.py`

Report **both** the synthetic and the from-bag numbers in the README. Publishing only the clean
synthetic figure is the kind of thing a reviewer catches, and it costs more credibility than the
slower honest number ever would.

The plot title MUST name the CPU, the horizon, and the acados commit. A solve-time histogram
without the hardware named conveys nothing.

Method notes that change the answer:

- warm up with 100 discarded solves; pin to a single core if `psutil` is available, and record
  whether pinning succeeded
- measure with `time.perf_counter_ns`
- advance the reference by `dt` each iteration
- from-bag mode: drop the first 2 s of startup transient, and say so in the caption

Exit non-zero when the p99 exceeds budget, so the script doubles as a CI gate.

## 12.2 `tracking_error_analysis.ipynb`

Plot the reference the controller was actually given, not the ideal analytic curve. State the
disturbance condition in every title. Never crop a transient without saying so in the caption.

The PX4 and ROS clocks differ — pick one time base explicitly, align with `merge_asof`, and
write down which you chose.

The most informative figure in this notebook is error vs commanded acceleration: it separates a
model deficiency (error grows with acceleration) from a tuning or latency issue (error roughly
constant).

Output a markdown summary table with `df.to_markdown()` for pasting into the README.

## 12.3 `disturbance_sweep.ipynb`

Grid: wind ∈ {0,2,4,6,8} m/s × direction ∈ {0,45,90}° × gust ∈ {off,on} × 3 seeds.

Cache every run to `analysis/output/sweep/` with a deterministic id derived from its parameters;
a sweep that cannot resume is a sweep you will run once and never again. Treat a crashed or
timed-out trial as a recorded FAILURE row, not an exception — a sweep that dies on trial 47 of
90 is useless.

Report mean ± std across the 3 repeats; a single run per cell would not be publishable.

Include the **failure boundary** plot — the wind speed at which the solver first reports an
infeasibility or the vehicle leaves the tube. Where the controller breaks is more interesting
than where it works, and showing it reads as research rather than demo.

This notebook is slow (hours) and is deliberately **not** part of CI.

## 12.4 `scripts/assert_hover.py`

Holds the A6/A7 thresholds as named constants at the top of the file. Prints every metric
regardless of outcome — the numbers in the CI job log are the evidence, not the green tick.

Changing any threshold there is changing an acceptance criterion: update
[01_OVERVIEW.md §1.3](01_OVERVIEW.md) in the same commit.

## 12.5 `media/`

Three assets, all regenerable — see `media/README.md` for the table. Rules:

- keep GIFs under 8 MB
- record at the camera framing defined in `uav_mpc/rviz/nmpc.rviz`, so captures are comparable
  across versions
- burn the git SHA into a corner of every capture
- document the exact `ffmpeg`/`gifski` command line once the first capture is made

The root README links to all three. Produce them or remove the links before the repository is
made public — broken image links are worse than no images.
