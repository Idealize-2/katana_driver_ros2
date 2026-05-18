# Intermittent backward drift ~300 ms after reaching goal + spline loop explanation

## Symptom

The arm reaches the goal position correctly, then ~300 ms later it briefly reverses
a small amount before settling. The drift is intermittent — it does not happen on
every move, only sometimes.

---

## How the spline movement loop works

The KNI firmware supports two kinds of position command:

### TPS (Trapezoidal Profile Segment) — `moveRobotToEnc()`
Generates a trapezoidal velocity profile that **always starts and ends at zero**.
Every call brings the arm to a full stop at the commanded position.
This is why using `moveRobotToEnc` directly produces stop-start motion at waypoints.

### Spline firmware command — `sendSplineToMotor()` + `startSplineMovement()`
Sends a **Hermite cubic** polynomial segment per motor, then fires all of them
simultaneously with `startSplineMovement()`. The key parameter is `moreflag`:

| `moreflag` | Meaning |
|------------|---------|
| `0` | **Chain**: firmware expects another segment to arrive before this one ends. Arm does NOT decelerate to zero — it transitions smoothly into the next segment. |
| `1` | **Final**: this is the last segment. Firmware decelerates arm to zero at the target. |

### Chaining loop (how continuous motion is achieved)

```
KNI thread iteration 1 (~300 ms):
  read encoders (55 ms)
  compute Hermite segment targeting waypoint A
  send to firmware (moreflag=0)     ← segment takes 400 ms to execute
  ← returns after 245 ms

KNI thread iteration 2 (~300 ms):
  read encoders
  compute Hermite segment targeting waypoint B
  send to firmware (moreflag=0)     ← arrives 300 ms into segment 1 (100 ms overlap)
  ← firmware queues segment 2 without stopping the arm

KNI thread iteration 3 (~300 ms):
  command stable for kIdleThresh cycles → moreflag=1
  compute final Hermite segment targeting goal G
  send to firmware (moreflag=1)
  ← firmware decelerates arm cleanly to G
```

The **100 ms overlap window** (T=400 ms segment duration minus ~300 ms loop cycle) is
what ensures the firmware always has the next segment queued before the current one
completes — the arm never stops between segments.

### Hermite cubic coefficients (encoder-space, T = steps)

```
p1 = start_enc
p2 = start_vel × T
p3 = 3(end-start) − (2·start_vel + end_vel) × T
p4 = (start_vel + end_vel) × T − 2(end-start)
```

Firmware encoding:
- `p2 → round(64 × p2 / T)`
- `p3 → round(1024 × p3 / T²)`
- `p4 → round(32768 × p4 / T³)`

**Velocity continuity**: For intermediate segments `end_vel = (target_enc − last_enc) / T`
(average rate of the segment). The next segment starts at this same velocity (`start_vel = end_vel`),
creating C1 continuity. Final segment uses `end_vel = 0` for clean deceleration.

**Idle detection**: `kIdleThresh = 3` cycles of command stable within `kDeadbandRad = 0.009 rad`
→ arm switches to `moreflag=1`. This means the arm sends up to 900 ms of `moreflag=0`
segments after JTC stops commanding (the trajectory is done) before the final hold segment fires.

---

## Root Cause of the intermittent backward drift

### Setup

After the arm reaches the goal and `moreflag=1` is sent, `kni_loop()` must:
1. Update `hw_pos_cache_` to the goal position (so JTC's hold command = goal).
2. Set `kni_hold_sent_ = true` (so subsequent iterations skip the spline send).

### The Race Window

In the original code, both actions happened **after** `startSplineMovement()` returned:

```
sendSplineToMotor × 7     ← ~245 ms (35 ms/motor × 7)
startSplineMovement()     ← arm starts executing final segment
                                         ↑ JTC can read here
  kni_hold_sent_ = true
  hw_pos_cache_ = goal    ← only updated NOW (~245 ms after send began)
```

`sendSplineToMotor()` is a per-motor sequential call — it takes ~35 ms per motor,
~245 ms total. During those 245 ms, JTC runs at 7 Hz (reads every ~143 ms).
JTC can read `hw_states_positions_` = the stale pre-goal cache **one or two times**
before `hw_pos_cache_` is finally updated.

When the trajectory ends JTC switches to "hold" mode and latches
`hw_states_positions_` as its hold target. If that latch happens during the 245 ms
race window, JTC commands the stale (pre-goal) position into `hw_cmd_cache_`.

On the next `kni_loop()` iteration:
- `cmds` = stale position (from JTC hold)
- `kni_last_cmd_` = goal position (set when the last changed=true fired)
- `|cmds − kni_last_cmd_| > kDeadbandRad` → `changed = true`
- `kni_hold_sent_ = false` — arm is commanded backward to the stale position

This produces the intermittent ~300 ms backward drift.

### Why it is intermittent

The race only triggers when JTC's 7 Hz read cycle coincides with the 245 ms send window.
Whether or not that happens depends on the exact timing offset between the JTC read timer
and the KNI loop. On most moves the window is missed; occasionally it is hit.

---

## Fix

Move the `hw_pos_cache_` → goal update to **before** the `sendSplineToMotor` loop,
closing the race window to zero:

```cpp
// Pre-update cache to goal BEFORE the 245ms send so JTC's hold command
// is already at the goal position — closes the race window.
if (is_last) {
  std::lock_guard<std::mutex> lk(kni_mtx_);
  for (int i = 0; i < mc && i < static_cast<int>(hw_pos_cache_.size()); ++i)
    hw_pos_cache_[i] = encoderToRad(i, kni_target_enc_[i]);
}

for (int i = 0; i < mc; ++i) { sendSplineToMotor(...); }
katana_->startSplineMovement(true, moreflag);

// Step 9
kni_last_enc_ = kni_target_enc_;
kni_last_vel_ = kni_ve_;
if (is_last) kni_hold_sent_ = true;
```

By the time JTC next reads, `hw_pos_cache_` already shows the goal. JTC's hold command
= goal = `kni_last_cmd_`, so `changed = false`, `kni_hold_sent_` stays true, and no
backward spline is ever sent.

### Why this is safe

- `is_last` only becomes true after `kIdleThresh = 3` cycles of stable command (≥ 900 ms
  with no new trajectory waypoints). The trajectory is definitely finished by then.
- The arm WILL reach `kni_target_enc_` within one 400 ms spline segment — pre-declaring
  arrival is accurate within the `goal: 0.15 rad` tolerance already configured in JTC.
- If a `sendSplineToMotor` call throws after the cache is pre-set, the exception handler
  sleeps 100 ms and continues — same behaviour as before.

---

## Files changed

| File | Change |
|------|--------|
| `katana_driver/src/katana_hardware_interface.cpp` | Moved `hw_pos_cache_` → goal update from after `startSplineMovement` to before `sendSplineToMotor` loop |

---

## Environment

- ROS 2 Jazzy, Ubuntu 24.04
- Neuronics Katana 400 6M180, KNI SDK 4.3.0, TCP connection
- Async KNI worker thread architecture (see `problem/arm_stopstart_and_async_kni_worker.md`)
