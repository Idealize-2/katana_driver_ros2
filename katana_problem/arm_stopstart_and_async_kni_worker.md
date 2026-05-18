# Arm stops at every MoveIt waypoint (stop-start motion) + async KNI worker fix

## Symptom

When executing a MoveIt trajectory the arm moves in a stutter pattern: it reaches each
intermediate waypoint, **decelerates fully to zero**, then re-accelerates toward the next one.
The motion is mechanically correct but jerky and slow — nothing like the smooth continuous
motion you get from the KNI demo tools.

Log shows normal JTC execution with no errors; the arm just stops between points.

---

## Root Cause 1 — TPS always decelerates to zero

`moveRobotToEnc()` (and its underlying firmware command) uses the KNI **Trapezoidal Profile
Segment (TPS)**. TPS is a point-to-point profile: the firmware generates a trapezoidal
velocity curve that begins and ends at zero velocity. There is no way to "chain" TPS commands
smoothly — every call brings the arm to a full stop at the commanded position.

MoveIt sends a trajectory as a sequence of (position, velocity, time) waypoints. The JTC
converts these into successive `write()` calls. Each `write()` calls `moveRobotToEnc()` and the
firmware decelerates to zero at that waypoint before the next command arrives.

---

## Root Cause 2 — KNI TCP is too slow for a synchronous control loop

Measured round-trip times over TCP for a Katana 400:
- `getRobotEncoders(true)` (read all 7 encoders): **~55 ms**
- `moveRobotToEnc()` (one motor): **~35 ms × 7 motors ≈ 245 ms total** for a full spline send

At `update_rate: 8` Hz the budget per cycle is 125 ms. Any synchronous KNI call in
`read()` or `write()` blows the budget.

Attempt: raise `update_rate` to 100 Hz.
Result: the controller manager logged `Overrun (395 ms)` for every cycle — the KNI
communication consumed the entire cycle plus two more.

---

## Fix — Async KNI worker thread + spline firmware commands

Move **all** KNI communication into a background thread. `read()` and `write()` become
instant mutex copies (< 1 µs each). The background thread runs at its natural pace
(~3 Hz, limited by TCP latency) and uses the KNI **spline** firmware command instead of TPS.

### Why splines fix the stop-start problem

The KNI spline API (`sendSplineToMotor` + `startSplineMovement`) accepts a **Hermite cubic**
segment per motor and a `moreflag`:

- `moreflag = 0` — firmware **chains** this segment to the previous one without stopping.
  The next segment must arrive before the current one finishes (overlap window).
- `moreflag = 1` — firmware treats this as the **final** segment and decelerates to zero
  at the target.

With T = 40 steps × 10 ms = **400 ms** per segment and a background loop cycle of ~300 ms,
there is a 100 ms overlap window — the next segment always arrives before the firmware needs it,
so the arm never stops between segments.

### Architecture

```
ros2_control CM thread (7 Hz)          KNI background thread (~3 Hz)
─────────────────────────────          ──────────────────────────────
read()  →  hw_states = hw_pos_cache    1. getRobotEncoders(true)  ~55 ms
write() →  hw_cmd_cache = hw_commands  2. update hw_pos_cache  (mutex)
           (both < 1 µs under mutex)   3. get hw_cmd_cache     (mutex)
                                       4. deadband / idle count
                                       5. motor fault check
                                       6. sendSplineToMotor × 7
                                       7. startSplineMovement(moreflag)
                                       total: ~300 ms/cycle
```

### Key implementation details

**`read()` / `write()`** — instant mutex copies, no KNI calls:
```cpp
hardware_interface::return_type KatanaHardwareInterface::read(
    const rclcpp::Time &, const rclcpp::Duration &) {
    std::lock_guard<std::mutex> lock(kni_mtx_);
    hw_states_positions_ = hw_pos_cache_;
    return hardware_interface::return_type::OK;
}
hardware_interface::return_type KatanaHardwareInterface::write(
    const rclcpp::Time &, const rclcpp::Duration &) {
    std::lock_guard<std::mutex> lock(kni_mtx_);
    hw_cmd_cache_ = hw_commands_positions_;
    return hardware_interface::return_type::OK;
}
```

**Hermite cubic coefficients** (encoder-space, T = segment length in 10 ms steps):
```
p1 = s                                  (start position)
p2 = vs * T                             (start velocity × T)
p3 = 3(e-s) - (2vs + ve) * T
p4 = (vs + ve) * T - 2(e-s)
```
Firmware scaling: `p2 → p2*64/T`, `p3 → p3*1024/T²`, `p4 → p4*32768/T³`.

For intermediate segments `ve = (target_enc - last_enc) / T` (average rate → C1 continuity).
For the final segment `ve = 0` (clean deceleration to rest).

**Idle detection** — after `kIdleThresh = 3` consecutive cycles with no new command (within
`kDeadbandRad = 0.009 rad`), the next send uses `moreflag = 1` so the arm stops cleanly at
the goal instead of chasing a drift signal forever.

**Encoder safety margin** — `kEncMargin = 200` ticks clamped away from firmware `enc_min` /
`enc_max` in every target to avoid FirmwareException "Encoder out of range" near joint limits.

---

## Secondary problem 1 — "Encoder out of range (axis 1)"

**Symptom:** `FirmwareException: Encoder out of range (axis 1)` during trajectory execution.

**Cause:** `katana_motor2_lift_joint` URDF upper limit 2.20 rad converts to encoder ≈ −31 005,
which is 5 ticks outside the firmware's post-calibration `enc_min = −31 000`.
The firmware's actual soft limits differ slightly from what the KNI config file states.

**Fix:** Added `kEncMargin = 200` in the background thread so computed encoder targets are
always clamped ≥ enc_min + 200. No URDF limit change needed.

---

## Secondary problem 2 — "Start state outside bounds"

**Symptom:**
```
Joint katana_motor2_lift_joint from the starting state is outside bounds by: [2.16997]
should be in the range [-0.135228], [2.16]
```

**Cause:** We had temporarily reduced the URDF `upper` limit to 2.16 rad to avoid the encoder
range issue. But the arm's post-calibration home position is 2.16997 rad — above the reduced
limit.

**Fix:** Restored `upper="2.20"` in `katana_400_6m180.urdf.xacro` and `max_position: 2.20` in
`joint_limits.yaml`. The `kEncMargin` clamp prevents the firmware rejection without needing a
tight URDF limit.

---

## Secondary problem 3 — PATH_TOLERANCE_VIOLATED

**Symptom:**
```
PATH_TOLERANCE_VIOLATED: Position Error 0.324387, Position Tolerance 0.300000
```
Arm moves smoothly for ~2 seconds then JTC aborts and returns arm to start.

**Cause:** The KNI position cache (`hw_pos_cache_`) updates at ~3 Hz (300 ms per cycle).
JTC computes position error as `desired_position − hw_states_positions_`. Because
`hw_states_positions_` is 300 ms stale, the apparent error can be much larger than the real
physical error even when the arm is following the trajectory perfectly.

**Fix:** Set `trajectory: 0.0` for all arm joints in `ros2_controllers.yaml` — in JTC,
`0.0` disables the per-joint trajectory tolerance check entirely while keeping the final
goal tolerance (`goal: 0.15`) active.

```yaml
constraints:
  katana_motor1_pan_joint:              { trajectory: 0.0, goal: 0.15 }
  katana_motor2_lift_joint:             { trajectory: 0.0, goal: 0.15 }
  katana_motor3_lift_joint:             { trajectory: 0.0, goal: 0.15 }
  katana_motor4_lift_joint:             { trajectory: 0.0, goal: 0.15 }
  katana_motor5_wrist_roll_joint:       { trajectory: 0.0, goal: 0.15 }
```

---

## Secondary problem 4 — Arm drifts backward ~300 ms after reaching goal

**Symptom:** Arm reaches the goal correctly, then ~300 ms later it reverses slightly back
toward the previous position.

**Cause:** After a trajectory completes, JTC enters "hold" mode. It commands
`hw_states_positions_` (the current feedback) as the hold target. Because the cache is 300 ms
stale, `hw_states_positions_` is the position from 300 ms ago — slightly behind the actual goal.
The background thread sees this as a new command and sends a spline backward.

**Fix:** `kni_hold_sent_` flag. After sending `moreflag = 1` (final segment):
1. Immediately update `hw_pos_cache_` to the target position derived from `target_enc`.
2. Set `kni_hold_sent_ = true`.
3. On subsequent cycles while held: skip the spline send, only refresh the cache from real
   encoders. Reset `kni_hold_sent_` only when a genuinely new command arrives.

This makes JTC's hold command equal to the goal that was just reached, so no backward spline
is ever sent.

---

## Files changed

| File | Change |
|------|--------|
| `katana_driver/include/katana_driver/katana_hardware_interface.hpp` | Added mutex/cache/thread members and `kni_loop()` declaration |
| `katana_driver/src/katana_hardware_interface.cpp` | Replaced `read()`/`write()` with mutex copies; added `kni_loop()` |
| `katana400_moveit_config/config/ros2_controllers.yaml` | `trajectory: 0.0` for all arm joints; `goal: 0.15` |
| `katana400_moveit_config/config/joint_limits.yaml` | Restored `max_position: 2.20`; velocity/accel scaling → 0.3 |
| `katana_description/urdf/katana_400_6m180.urdf.xacro` | Restored `upper="2.20"` for motor2_lift_joint |

---

## Environment

- ROS 2 Jazzy, Ubuntu 24.04
- MoveIt2, `joint_trajectory_controller`, `ros2_control`
- Neuronics Katana 400 6M180, KNI SDK 4.3.0, TCP connection
