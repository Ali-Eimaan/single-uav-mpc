# §8 · Launch files

**Governs:** `uav_mpc/launch/{nmpc_only,sitl,figure8,hardware}.launch.py`
**Prerequisites:** [07_NODE.md](07_NODE.md)
**Milestone:** M7
**Done when:** `ros2 launch uav_mpc sitl.launch.py` flies from a clean clone (criterion A8).

> Check the Lyrical launch lifecycle-event API first — see
> [02_ENVIRONMENT.md §2.2](02_ENVIRONMENT.md). If the API moved, the pattern below still holds.

---

## 8.1 `nmpc_only.launch.py`

The building block every other launch file includes. Assumes PX4 and the uXRCE-DDS agent are
already running.

Arguments: `airframe` (`x500`|`crazyflie21`), `nmpc_config`, `trajectory_config`, `namespace`,
`auto_arm` (false), `auto_activate` (true), `log_level` (info), `use_sim_time` (false).

Lifecycle sequencing MUST use events, not timers:

```python
EmitEvent(ChangeState(… TRANSITION_CONFIGURE))                     # on ProcessStart
RegisterEventHandler(OnStateTransition(goal_state="inactive",
                                       entities=[EmitEvent(ChangeState(… TRANSITION_ACTIVATE))]))
```

A `TimerAction` that "usually works" is a race, and a reviewer reading the launch file will
recognise which one you wrote.

## 8.2 `sitl.launch.py`

Order: PX4 SITL+Gazebo → uXRCE agent → NMPC → RViz.

- PX4 via `ExecuteProcess` with `additional_env = {PX4_SYS_AUTOSTART: "4001",
  PX4_GZ_MODEL: model, PX4_GZ_WORLD: world, HEADLESS: …}` and
  `cwd = <px4_dir>/build/px4_sitl_default`.
- If `px4_dir` does not exist, fail immediately with the exact `git clone` + `make` commands in
  the message. **This is the file that has to work for a stranger** — prefer explicit checks and
  loud errors over clever automation.
- Agent: `MicroXRCEAgent udp4 -p 8888`, started after PX4 (event handler where possible).
- NMPC included after a bounded delay; document why the delay exists.
- **Apply `config/px4_overrides.yaml`.** Write the `param set` lines into the PX4 startup extras
  file before launching (`$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/px4-rc.simulator` or the
  documented equivalent for the pinned version). Skipping this silently invalidates the thrust
  map, and the symptom is a steady altitude offset you will blame on tuning.
- `on_shutdown` handler that SIGINTs PX4 and the agent — orphaned `px4` processes hold port 8888
  and the next launch fails mysteriously.

## 8.3 `figure8.launch.py`

Includes 8.2 with `trajectory:=hover, auto_arm:=true`, then commands the figure-8 once the
controller reports `STATE_TRACKING` on `~/status` (with a bounded timer as a fallback), and
optionally records the bag listed in the file's docstring.

Arguments: `aggressive` (false — swaps in the 3 m / 5 s preset), `record` (false), `laps` (3),
`takeoff_wait` (12.0 s).

## 8.4 `hardware.launch.py`

`auto_arm` defaults **false** and `auto_activate` defaults **false**. These defaults are a
safety property; do not "improve the UX" by flipping them. Arming is a human action.

Backends:

- `backend:=px4` — Pixhawk-class vehicle over serial or Wi-Fi, uXRCE-DDS agent to the FMU.
  Identical topic namespace to SITL, so the controller code path is bit-for-bit the same.
- `backend:=crazyflie` — Crazyflie 2.1 via `crazyflie_ros2` / Crazyswarm2, through an adapter.

**Crazyflie adapter contract:** subscribe `/nmpc_node/status`, take `attitude_setpoint` +
`collective_thrust_newton`, convert to the `crazyflie_ros2` setpoint (roll/pitch in degrees,
yaw rate in deg/s, thrust as a 16-bit PWM via the calibrated map in
`params/crazyflie21_calibration.yaml:hover_pwm`), publish at 100 Hz, and **latch to zero thrust
if the NMPC status goes stale for 100 ms**.

Also required: geofence parameters applied before activation, and a `preflight_check` step that
verifies battery voltage, EKF health and mocap age and refuses activation otherwise.
