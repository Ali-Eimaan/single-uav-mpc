# Hardware bring-up

This document describes how this controller *would be* deployed on real hardware. Where a
step has not been performed on a physical vehicle, it says so explicitly. An advisor
reading a bring-up guide that quietly implies untaken flights will discount the entire
repository; one that says "procedure written, not yet flown" reads as rigorous.

**Status key**: `written` = procedure documented, not exercised on hardware;
`bench-tested` = exercised on the bench (props off, tethered); `flown` = flown.

---

## 1. Safety first

**Status: written**

1. **Kill switch mapped and tested on the bench with props off** before every session.
   Test it at least once per session, and test the *switch* not just the button.
2. **Geofence**: set `GF_MAX_HOR_DIST`, `GF_MAX_VER_DIST`, `GF_ACTION` to a return/land
   action. Do not rely on the pilot for horizontal containment during autonomous tracking.
3. **Offboard-loss behaviour**: set `COM_OF_LOSS_T` (timeout) and `COM_OBL_RC_ACT` (fallback
   action). Never fly with `COM_RCL_EXCEPT` set outside SITL — the RC link is your escape
   hatch.
4. **Props off for every first run of anything.** All of them, every time.
5. **Battery, netting/tether, and a second person whose only job is the kill switch.** The
   spotter does not hold a controller; the spotter holds the switch.

## 2. Hardware checklist

**Status: written**

- **Vehicle**: Holybro X500 v2, Pixhawk 6C, PX4 v1.16+ (the version pinned in
  `docker_smoke_test.yml`). Flashing a different PX4 than SITL was tested against is a
  classic source of "it worked in sim".
- **Companion**: the compute you will actually use, with the measured solve time on *that*
  CPU. A 0.4 ms desktop number means nothing on a Raspberry Pi. Re-run
  `analysis/solve_time_benchmark.py` on the target and record the p99 here before flying.
- **Positioning**: mocap (indoor) or RTK GPS (outdoor). Record the latency you measured —
  it feeds `latency_compensation_s` and the tracking-error budget.
- **Link**: uXRCE-DDS over serial (921600) or Wi-Fi. Measure the round-trip and check it
  against `latency_compensation_s`. The NMPC node publishes
  `/fmu/in/vehicle_attitude_setpoint`; verify it arrives with a fresh timestamp before
  arming.

## 3. Step-by-step bring-up (X500 / PX4)

**Status: written**

1. **Bench, props off.** Run `hardware.launch.py` with the node left inactive. Verify all
   four PX4 topics arrive and `~/status` shows a fresh state age.
2. **Bench, props off, armed.** Activate the node. Verify the attitude setpoint tracks
   hand-held vehicle rotations sensibly (the controller reacts as you tilt it, the right
   way). Check `normalised_thrust` is near hover. *This is the yaw-sign test of the tuning
   guide §0 — do it here, on the bench.*
3. **Tethered hover.** Hover only (`TYPE_HOVER`), 30 s. Land. Read the log: position RMS,
   solver status, rate activity. Fix any fault at the tether, not in free flight.
4. **Free hover**, then a 0.5 m step, then a 1 m step. Watch the 1 m step: settle time
   should be ~1–2 s with no overshoot beyond 15%.
5. **Slow figure-8** (2 m, 12 s period). Verify tracking, then shorten the period in steps
   (12 → 8 → 5 s), re-verifying at each step. Do not skip to the target period.
6. **Target figure-8**. Record (rosbag + PX4 ulog), run
   `analysis/tracking_error_analysis.ipynb`, publish the plot.

**Abort criteria — written down before flying, not after**: solve-time p99 above the
benchmark budget, any solver infeasibility in flight, position error beyond 0.5 m, or link
loss. Any one of these → kill switch, land, review the log.

## 3.5 Choosing the vehicle backend

`px4_msgs` is not released for ROS 2 Lyrical Luth, so the controller ships with two backends
(`.deepseek/07_NODE.md` §7.10). Pick before you power anything on:

| Situation | Setting |
| --- | --- |
| Pixhawk + PX4, workspace has px4_msgs | `vehicle_interface:=px4` |
| Pixhawk + PX4, no px4_msgs available | `vehicle_interface:=generic` + a MAVROS/uXRCE adapter you write and test on the bench first |
| Crazyflie 2.1 | `vehicle_interface:=generic` (§4) |
| Mocap-driven bench rig, any airframe | `vehicle_interface:=generic` |

**Safety consequence of `generic`:** it has no arm/offboard handshake. The controller assumes
it always has authority and begins publishing setpoints the moment it is activated. With the
`px4` backend, PX4 refuses commands until it is armed and in offboard mode — that refusal is a
safety layer you lose with `generic`. Whatever consumes `~/attitude_setpoint` must therefore
implement:

- [ ] an arming interlock the pilot controls
- [ ] a setpoint-staleness watchdog that latches thrust to zero (100 ms is the value used by the
      Crazyflie adapter in §4)
- [ ] the kill switch, independent of this software

Do not fly `generic` on a real vehicle until those three exist and have been bench-tested with
props off.

## 4. Crazyflie 2.1 backend

**Status: written**

- **Differences from PX4**: no `px4_msgs`, no EKF you control, a much lighter airframe with
  faster attitude dynamics (see the timescale discussion in the tuning guide §6).
- **Adapter node contract** (`.deepseek/08_LAUNCH.md` §8.4): the NMPC publishes the same
  interface (attitude setpoint + thrust, diagnostics); `crazyflie_ros2` expects its own
  message types, so an adapter translates. The thrust-to-PWM map is a separate calibration
  step — the Crazyflie's motor map is highly nonlinear, and the hover setpoint must be
  verified on the bench with a scale.
- **Mocap**: Crazyswarm2 provides the pose stream; feed it in place of
  `/fmu/out/vehicle_local_position`.
- **The honest caveat**: at CF scale the 100 Hz attitude-setpoint interface is marginal
  (attitude response is much faster relative to the loop). The guide's recommendation is
  to switch to **body-rate setpoints** for this frame, which moves the fast attitude loop
  inside Crazyswarm where it belongs.

## 5. Calibration procedure

**Status: written**

1. **Mass**: a scale, with the battery you will fly. Re-measure on your own build; do not
   trust the advertised dry mass.
2. **Inertia**: bifilar pendulum procedure, or cite the source you are trusting instead —
   and say which. The X500 values in `params/x500_calibration.yaml` are from the
   manufacturer's CAD (with the source cited in the file comment); they are a good prior,
   not a measurement.
3. **Thrust coefficient**: static thrust stand, thrust vs commanded PWM, fit and residual.
   This is the number that makes `normalised_thrust` sit at hover — the cheapest possible
   validation is a hover log (§0.2 of the tuning guide).
4. **Hover thrust**: from a real hover log (`normalised_thrust` at settled hover × max
   thrust).
5. **Update `params/<frame>_calibration.yaml` and flip `verified: true` only for values
   you have actually measured or traced to a cited source.** A calibration file where
   everything is `verified: true` without a flight log is a lie.

## 6. Pre-flight checklist

**Status: written**

Print this. Run it in order. No step requires judgment.

- [ ] Battery voltage ≥ 3.7 V/cell resting
- [ ] EKF health flags all green (in `~/status` and on the GCS)
- [ ] Mocap/GPS age < 100 ms and stable (no jumps in the last 30 s)
- [ ] Solve-time sanity: p99 from `analysis/solve_time_benchmark.py` on *this* compute is
      within budget (record the number on the log)
- [ ] Geofence active: `GF_ACTION` set, distances match the flying area
- [ ] Kill switch tested this session, props off, switch position confirmed
- [ ] RC link healthy, `COM_RCL_EXCEPT` not set
- [ ] Props on only after all of the above
- [ ] Second person on the kill switch, briefed

## 7. Post-flight

**Status: written**

1. Pull the PX4 ulog and the rosbag; run `analysis/tracking_error_analysis.ipynb`.
2. Record the flight in the log table below (date, config hash, git SHA, outcome). The git
   SHA column is what makes a result reproducible.

| Date | Config hash | git SHA | Outcome |
| --- | --- | --- | --- |
| — | — | — | (first flight to be recorded here) |
