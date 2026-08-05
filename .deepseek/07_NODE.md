# §7 · `nmpc_node.{hpp,cpp}` and `main.cpp`

**Governs:** `uav_mpc/include/uav_mpc/nmpc_node.hpp`, `uav_mpc/src/nmpc_node.cpp`,
`uav_mpc/src/main.cpp`
**Prerequisites:** [04_DYNAMICS.md](04_DYNAMICS.md), [05_TRAJECTORY.md](05_TRAJECTORY.md),
[06_SOLVER.md](06_SOLVER.md), [09_CONFIG.md](09_CONFIG.md)
**Milestone:** M6
**Done when:** the vehicle hovers autonomously in SITL (criterion A6).

Topic map:

```
IN : /fmu/out/vehicle_local_position     VehicleLocalPosition   (NED)
     /fmu/out/vehicle_attitude           VehicleAttitude        (NED/FRD)
     /fmu/out/vehicle_angular_velocity   VehicleAngularVelocity (FRD)
     /fmu/out/vehicle_status             VehicleStatus
OUT: /fmu/in/offboard_control_mode       OffboardControlMode    @ 100 Hz, unconditional
     /fmu/in/vehicle_attitude_setpoint   VehicleAttitudeSetpoint
     /fmu/in/vehicle_command             VehicleCommand         (arm / mode switch)
     ~/status                            uav_mpc/NmpcStatus
     ~/predicted_path, ~/reference_path  nav_msgs/Path          (ENU, RViz)
```

---

## 7.1 Lifecycle

| Transition | Does | On failure |
| --- | --- | --- |
| `on_configure` | load params, airframe, model, trajectory, solver; create subs (inactive pubs), service, param callback | `FAILURE` + one clear `RCLCPP_ERROR` naming the cause |
| `on_activate` | activate pubs, create the 100 Hz timer, state → `Streaming` | `FAILURE` |
| `on_deactivate` | **cancel + reset the timer first**, then deactivate pubs, state → `Idle` | — |
| `on_cleanup` | release solver, generator, model, pubs/subs | — |
| `on_shutdown` | stop the timer and publishing. **Do not disarm.** | — |

The timer-first ordering in `on_deactivate` is not stylistic: a timer callback that runs against
a deactivated publisher is undefined behaviour.

`on_shutdown` does not disarm because PX4's own offboard-loss failsafe is the safer authority
and is already configured (`COM_OF_LOSS_T`, `COM_OBL_RC_ACT`). A companion computer deciding to
disarm a flying vehicle is how you break a frame.

> Check the Lyrical `rclcpp_lifecycle` API before writing these — see
> [02_ENVIRONMENT.md §2.2](02_ENVIRONMENT.md).

## 7.2 QoS

PX4 publishes with a specific profile; a mismatched subscription silently receives nothing:

```cpp
rclcpp::QoS px4_qos(rclcpp::KeepLast(5));
px4_qos.best_effort().durability_volatile();
```

Use it for **every** `/fmu/out/*` subscription and `/fmu/in/*` publisher. If topics look dead,
check this before anything else.

## 7.3 State machine

| From | To | Condition |
| --- | --- | --- |
| `Idle` | `Streaming` | node activated |
| `Streaming` | `Takeoff` | armed ∧ offboard ∧ ≥ 20 OffboardControlMode published ∧ state fresh |
| `Takeoff` | `Tracking` | `|z − takeoff_altitude| < 0.1 m` ∧ `‖v‖ < 0.2 m/s` |
| `Tracking` | `Landing` | landing requested |
| `Landing` | `Idle` | `z < 0.15 m` ∧ `‖v‖ < 0.2 m/s` → disarm if `auto_arm` |
| any | `Failsafe` | state stale, or `consecutive_failures ≥ max` |
| `Failsafe` | `Streaming` | state fresh again ∧ one solve succeeded |
| `Tracking` | `Streaming` | offboard lost (pilot took over) — log at ERROR |

`Streaming` exists because PX4 refuses to enter offboard mode unless setpoints are already
arriving. Publish hover setpoints there.

## 7.4 Control loop — fixed ordering

```
 1. now = get_clock()->now();  measure the period since the last tick
 2. if (!assembleState(&x0, &why))            → enterFailsafe(why); publishOffboardControlMode(); return
 3. updateControllerState()
 4. x0 = compensateLatency(x0, latency_compensation_s_)
 5. refs = trajectory_->referenceHorizon(t_traj, dt, N, airframe_, rep)
 6. solver_->setInitialState(x0); solver_->setReferenceHorizon(...); solver_->setParameters(p)
 7. result = solver_->solve()
 8. if (result.ok())  u0 = solver_->optimalInput();  last_applied_input_ = u0
    else              hold last_applied_input_  (and count the failure)
 9. pub_attitude_setpoint_->publish(toAttitudeSetpoint(u0, solver_->predictedState(1)))
10. publishOffboardControlMode()      ← EVERY tick, unconditionally, including in Failsafe
11. publishStatus(result, x0); publishVisualisation()
```

Budget: 10 ms total. Warn (throttled, 1 Hz) above `log_solve_time_warn_ms`.
**No dynamic allocation after the first tick** — pre-size every buffer in `on_configure`.
`publishVisualisation` is skipped when there are no subscribers.

## 7.5 Message conventions

- **Timestamps to PX4 are microseconds:** `get_clock()->now().nanoseconds() / 1000`.
  Nanoseconds here means PX4 silently discards your setpoints.
- `VehicleAttitude::q` is `(w, x, y, z)` in NED/FRD.
- `VehicleCommand` requires `target_system = 1`, `target_component = 1`, `source_system = 1`,
  `source_component = 1`, `from_external = true`. Missing `from_external` = ignored command.
- Offboard mode: `VEHICLE_CMD_DO_SET_MODE` (176) with `param1 = 1`, `param2 = 6`.
  Arm: `VEHICLE_CMD_COMPONENT_ARM_DISARM` (400) with `param1 = 1`.
- Reject `VehicleLocalPosition` unless `xy_valid ∧ z_valid ∧ v_xy_valid ∧ v_z_valid`.

> `VehicleAttitudeSetpoint` field names changed across PX4 releases (the `roll_body` /
> `pitch_body` / `yaw_body` members were removed). **Read the message definition in the pinned
> `px4_msgs`** rather than trusting any example, including this one.

## 7.6 Attitude setpoint

```cpp
model_->allocate(u0, &T, &tau);                      // collective thrust [N]
q_d_enu = quaternion slice of x_pred_1;              // the optimiser's intended attitude
msg.q_d = quatEnuFluToNedFrd(q_d_enu);               // (w, x, y, z)
msg.thrust_body = {0.f, 0.f, -normaliseThrust(T)};   // NEGATIVE z — PX4 is FRD
msg.yaw_sp_move_rate = predicted yaw rate;
msg.timestamp = now_us;
```

Taking the attitude from the **predicted state at stage 1** rather than reconstructing it from
`u0` is deliberate: it is what the optimiser actually intends one step ahead, and it already
accounts for the rate dynamics.

`normaliseThrust`: `u = px4_hover_thrust · T/(m·g)`, clamped to `[0.05, 0.95]`. This is correct
only when `THR_MDL_FAC = 0`. If the frame sets it non-zero, invert PX4's quadratic instead — and
cross-check against `config/px4_overrides.yaml`, which must agree with
`nmpc_params.yaml:px4_hover_thrust`.

## 7.7 Latency compensation

`x0 ← model_->step(x0, last_applied_input_, latency_s)`, skipped on the first tick and when
`latency_s <= 0`. `latency_compensation_s` is the **measured** sensor-to-actuation delay
(procedure in `docs/TUNING_GUIDE.md` §5), not a guess. Over-compensating is destabilising —
start at 0 and increase.

## 7.8 `main.cpp`

`rclcpp::init` → `NodeOptions().use_intra_process_comms(true)` → `MultiThreadedExecutor` with
2 threads → add `node->get_node_base_interface()` → spin → shutdown.

Optional `--rt-priority N`: attempt `SCHED_FIFO` on the control thread; if it fails (no
`CAP_SYS_NICE`), log a warning and **continue**. Never make real-time scheduling a hard
requirement for the demo to run.

## 7.9 Threading

Two callback groups: `control_callback_group_` (MutuallyExclusive — the timer) and
`telemetry_callback_group_` (Reentrant — subscriptions and the service). Cached state is guarded
by `state_mutex_`; hold it only to copy in/out, never across a solve.
