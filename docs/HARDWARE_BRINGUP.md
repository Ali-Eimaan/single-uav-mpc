# Hardware bring-up

**SKELETON — outline only.** See `.deepseek/13_DOCS.md` §13.5.

> This document describes how this controller *would be* deployed on real hardware. Where a
> step has not been performed on a physical vehicle, say so in that step. An advisor reading
> a bring-up guide that quietly implies untaken flights will discount the entire repository;
> one that says "procedure written, not yet flown" reads as rigorous.
>
> TODO(deepseek): mark each section with a status line — `Status: written / bench-tested /
> flown` — and keep it accurate.

---

## 1. Safety first

- [ ] Kill switch mapped and **tested on the bench with props off** before every session.
- [ ] Geofence: `GF_MAX_HOR_DIST`, `GF_MAX_VER_DIST`, `GF_ACTION`.
- [ ] Offboard-loss behaviour: `COM_OF_LOSS_T`, `COM_OBL_RC_ACT`. Never fly with
      `COM_RCL_EXCEPT` set outside SITL.
- [ ] Props off for every first run of anything. All of them.
- [ ] Battery, netting/tether, and a second person whose only job is the kill switch.

## 2. Hardware checklist

- [ ] Vehicle: Holybro X500 v2, Pixhawk 6C, PX4 v1.16+ (the version pinned in
      `docker_smoke_test.yml`; flashing a different one than SITL was tested against is a
      classic source of "it worked in sim").
- [ ] Companion: the compute you will actually use, with the measured solve time on *that*
      CPU — a 0.4 ms desktop number means nothing on a Raspberry Pi. Re-run
      `analysis/solve_time_benchmark.py` on the target and record it here.
- [ ] Positioning: mocap (indoor) or RTK GPS (outdoor). State the latency you measured.
- [ ] Link: uXRCE-DDS over serial (921600) or Wi-Fi. Measure the round-trip and check it
      against `latency_compensation_s`.

## 3. Step-by-step bring-up (X500 / PX4)

1. [ ] **Bench, props off.** Run `hardware.launch.py` with the node left inactive. Verify all
       four PX4 topics arrive and `~/status` shows a fresh state age.
2. [ ] **Bench, props off, armed.** Activate the node. Verify the attitude setpoint tracks
       hand-held vehicle rotations sensibly. Check `normalised_thrust` is near hover.
3. [ ] **Tethered hover.** Hover only (`TYPE_HOVER`), 30 s. Land. Read the log.
4. [ ] **Free hover**, then a 0.5 m step, then a 1 m step.
5. [ ] **Slow figure-8** (2 m, 12 s period). Verify tracking, then shorten the period in steps.
6. [ ] **Target figure-8**. Record, analyse, publish the plot.

Abort criteria — write them down before flying, not after: solve time p99 above budget,
any solver infeasibility in flight, position error beyond 0.5 m, or link loss.

## 4. Crazyflie 2.1 backend

- [ ] Differences from PX4: no `px4_msgs`, no EKF you control, a much lighter airframe with
      faster attitude dynamics.
- [ ] The adapter node contract (`.deepseek/08_LAUNCH.md` §8.4): what the NMPC publishes and
      what `crazyflie_ros2` expects, including the thrust-to-PWM map and its calibration.
- [ ] Mocap setup with Crazyswarm2.
- [ ] The honest caveat: at CF scale the 100 Hz attitude-setpoint interface is marginal; the
      guide's recommendation is to switch to body-rate setpoints for this frame.

## 5. Calibration procedure

- [ ] Mass: scale, with the battery you will fly.
- [ ] Inertia: bifilar pendulum procedure, or cite the source you are trusting instead — and
      say which.
- [ ] Thrust coefficient: static thrust stand, thrust vs commanded PWM, fit and residual.
- [ ] Hover thrust: from a real hover log.
- [ ] Update `params/<frame>_calibration.yaml` and flip `verified: true` only for values you
      have actually measured or traced to a cited source.

## 6. Pre-flight checklist

> TODO(deepseek): a literal checklist to run before every flight — printable, ordered, with
> no step that requires judgment. Include battery voltage, EKF health flags, mocap age,
> solve-time sanity, geofence active, kill switch tested.

## 7. Post-flight

- [ ] Pull the PX4 ulog and the rosbag; run `analysis/tracking_error_analysis.ipynb`.
- [ ] Record what changed since the last flight in a flight log table (date, config hash, git
      SHA, outcome). The git SHA column is what makes a result reproducible.
