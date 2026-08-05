# media/

**SKELETON — the assets do not exist yet.** The root README links to all three; produce them
before making the repository public, or remove the links. A README with broken image links is
worse than a README with no images.

| File | Produced by | Content |
| --- | --- | --- |
| `figure8.gif` | `launch/figure8.launch.py` + screen capture | Gazebo Jetty view of the figure-8, with a side panel showing per-step solve time. 10-15 s, looping, under 8 MB. |
| `disturbance_recovery.gif` | `analysis/disturbance_sweep.ipynb` §6 | Step gust at 6 m/s and the recovery, with the position-error trace overlaid. |
| `solve_time_histogram.png` | `analysis/solve_time_benchmark.py` | Log-x histogram, median and p99 marked, CPU model and acados commit in the title. |

## Capture notes

> TODO(deepseek): fill in the exact recipe once the first capture is made — recorder, frame
> rate, crop, palette settings for the GIF, and the `ffmpeg`/`gifski` command line. The point
> is that these are regenerable, not one-off screen recordings nobody can reproduce.

- Keep GIFs under 8 MB; GitHub will render but throttle larger ones.
- Record at the RViz/Gazebo camera framing defined in `uav_mpc/rviz/nmpc.rviz`.
- Burn the git SHA into a corner of the capture. When someone asks "which version is this?"
  six months from now, the answer should be in the image.
