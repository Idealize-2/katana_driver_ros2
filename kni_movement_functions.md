# KNI SDK — Movement-Related Functions

Source headers: `install/kni/include/`

Inheritance chain (bottom → top):
```
CMotBase  (kmlMotBase.h)   — single motor
    └── CKatBase  (kmlBase.h)      — robot base
            └── CKatana  (kmlExt.h)       — extended arm
                    └── CikBase  (ikBase.h)        — inverse kinematics
                            └── CLMBase  (lmBase.h)        — linear movement  ← used by driver
```

---

## CMotBase — single motor (`KNI/kmlMotBase.h`)

| Function | Description |
|---|---|
| `mov(int tar, bool wait, int tol, long timeout)` | Move motor to encoder target |
| `inc(int dif, bool wait, int tol, long timeout)` | Increment motor by encoder delta |
| `dec(int dif, bool wait, int tol, long timeout)` | Decrement motor by encoder delta |
| `movDegrees(double tar, bool wait, int tol, long timeout)` | Move motor to target in degrees |
| `incDegrees(double dif, bool wait, int tol, long timeout)` | Increment motor by degrees |
| `decDegrees(double dif, bool wait, int tol, long timeout)` | Decrement motor by degrees |
| `waitForMotor(int tar, int tol, short mode, int timeout)` | Block until motor reaches position |
| `sendSpline(short targetPos, short duration, short p1, short p2, short p3, short p4)` | Send one spline segment to this motor |
| `setSpeedLimits(short posVel, short negVel)` | Set per-motor velocity limits |
| `setSpeedLimit(short vel)` | Set symmetric velocity limit |
| `setAccelerationLimit(short acc)` | Set per-motor acceleration limit |

---

## CKatBase — robot base (`KNI/kmlBase.h`)

| Function | Description |
|---|---|
| `startSplineMovement(int exactflag, int moreflag)` | Fire loaded spline segments (`moreflag`: 0=chain more, 1=last/single, 2=load only) |
| `setAndStartPolyMovement(vector<short> poly, int exactflag, int moreflag)` | Load polynomials for all motors and start |
| `waitFor(TMotStsFlg status, int timeout, bool gripper)` | Block until all motors reach a status flag |
| `flushMoveBuffers()` | Clear queued move commands on all motors |
| `unBlock()` | Clear crash/fault flags so movement can resume |
| `enableCrashLimits()` | Enable firmware crash detection |
| `disableCrashLimits()` | Disable firmware crash detection |
| `setCrashLimit(long idx, int limit)` | Set crash error threshold per motor |
| `setPositionCollisionLimit(long idx, int limit)` | Set position-based collision limit per motor |
| `setSpeedCollisionLimit(long idx, int limit)` | Set speed-based collision limit per motor |

---

## CKatana — extended arm (`KNI/kmlExt.h`)

| Function | Description |
|---|---|
| `calibrate()` | Run full calibration sequence (drives all joints to mechanical stops) |
| `calibrate(long idx, TMotCLB, TMotSCP, TMotDYL)` | Calibrate a single motor |
| `searchMechStop(long idx, TSearchDir, TMotSCP, TMotDYL)` | Drive one motor until it hits its mechanical stop |
| `mov(long idx, int tar, bool wait, int tol, long timeout)` | Move one motor by index to encoder target |
| `inc(long idx, int dif, bool wait, int tol, long timeout)` | Increment one motor by encoder delta |
| `dec(long idx, int dif, bool wait, int tol, long timeout)` | Decrement one motor by encoder delta |
| `movDegrees(long idx, double tar, bool wait, int tol, long timeout)` | Move one motor to target in degrees |
| `incDegrees(long idx, double dif, bool wait, int tol, long timeout)` | Increment one motor by degrees |
| `decDegrees(long idx, double dif, bool wait, int tol, long timeout)` | Decrement one motor by degrees |
| `moveMotorToEnc(short num, int enc, bool wait, int tol, int timeout)` | Move one motor to encoder position |
| `moveMotorByEnc(short num, int enc, bool wait, int timeout)` | Move one motor by encoder delta |
| `moveMotorTo(short num, double rad, bool wait, int timeout)` | Move one motor to radian target |
| `moveMotorBy(short num, double rad, bool wait, int timeout)` | Move one motor by radian delta |
| `moveRobotToEnc(vector<int> enc, bool wait, int tol, int timeout)` | Move all motors to encoder targets (TPS path — decelerates to zero between waypoints) |
| `moveRobotToEnc4D(vector<int> target, int vel, int acc, int tol)` | Move all motors to encoders with explicit velocity/acceleration |
| `openGripper(bool wait, int timeout)` | Open gripper to configured `openEncoders` |
| `closeGripper(bool wait, int timeout)` | Close gripper to configured `closeEncoders` |
| `setGripperParameters(bool isPresent, int openEnc, int closeEnc)` | Configure gripper encoder targets |
| `freezeRobot()` | Hold all motors at current position |
| `freezeMotor(short num)` | Hold one motor at current position |
| `switchRobotOn()` | Enable all motors (power on) |
| `switchRobotOff()` | Disable all motors (arm goes limp) |
| `switchMotorOn(short num)` | Enable one motor |
| `switchMotorOff(short num)` | Disable one motor |
| `sendSplineToMotor(short num, short targetPos, short duration, short p1, short p2, short p3, short p4)` | Load one spline segment onto one motor (`duration` in 10 ms units) |
| `startSplineMovement(bool exactflag, int moreflag)` | Fire loaded spline segments |
| `setAndStartPolyMovement(vector<short> poly, bool exactflag, int moreflag)` | Load + fire poly segments for all motors |
| `waitForMotor(short num, int enc, int tol, short mode, int timeout)` | Block until one motor reaches position |
| `waitFor(TMotStsFlg status, int timeout)` | Block until all motors reach a status flag |
| `setMotorVelocityLimit(short num, short vel)` | Set velocity limit for one motor |
| `setRobotVelocityLimit(short vel)` | Set velocity limit for all motors |
| `setMotorAccelerationLimit(short num, short acc)` | Set acceleration limit for one motor |
| `setRobotAccelerationLimit(short acc)` | Set acceleration limit for all motors |

---

## CikBase — inverse kinematics (`KNI_InvKin/ikBase.h`)

| Function | Description |
|---|---|
| `IKCalculate(X, Y, Z, Al, Be, Ga, iter)` | Solve IK from current robot encoders (does TCP communication) |
| `IKCalculate(X, Y, Z, Al, Be, Ga, iter, actualPos)` | Solve IK from provided encoder vector (no TCP) |
| `IKGoto(X, Y, Z, Al, Be, Ga, bool wait, int tol, long timeout)` | Move to Cartesian pose via IK (**deprecated** — use `moveRobotTo`) |
| `moveRobotTo(double x, y, z, phi, theta, psi, bool wait, int timeout)` | Move to Cartesian pose via IK |
| `moveRobotTo(vector<double> coords, bool wait, int timeout)` | Same, takes a 6-element vector |
| `getCoordinates(x, y, z, phi, theta, psi, bool refresh)` | Read current Cartesian position (forward kinematics) |
| `getCoordinatesFromEncoders(vector<double>& pose, vector<int>& encs)` | Forward kinematics from a given encoder vector |
| `DKApos(double* position)` | Forward kinematics (**deprecated** — use `getCoordinates`) |
| `setTcpOffset(double xoff, yoff, zoff, psioff)` | Set TCP offset from flange in metres / radians |

---

## CLMBase — linear movement (`KNI_LM/lmBase.h`)

| Function | Description |
|---|---|
| `movLM(X, Y, Z, Al, Be, Ga, bool exactflag, double vmax, bool wait, int tol, long timeout)` | Linear move to Cartesian pose at constant TCP velocity (mm/s) |
| `movLM2P(X1..Ga1, X2..Ga2, bool exactflag, double vmax, bool wait, int tol, long timeout)` | Linear move from explicit start pose to end pose |
| `movP2P(X1..Ps1, X2..Ps2, bool exactflag, double vmax, bool wait, long timeout)` | Joint-space point-to-point move using splines (TCP path not straight) |
| `moveRobotLinearTo(double x, y, z, phi, theta, psi, bool wait, int timeout)` | High-level linear move to Cartesian pose |
| `moveRobotLinearTo(vector<double> coords, bool wait, int timeout)` | Same, takes a 6-element vector |
| `moveRobotTo(double x, y, z, phi, theta, psi, bool wait, int timeout)` | Overrides `CikBase::moveRobotTo` (uses default tolerance) |
| `moveRobotTo(vector<double> coords, bool wait, int timeout)` | Same, takes a 6-element vector |
| `setMaximumLinearVelocity(double vmax)` | Cap TCP speed for linear moves (mm/s) |
| `getMaximumLinearVelocity()` | Read current TCP speed cap |
| `setActivatePositionController(bool activate)` | Whether to re-engage position hold after a linear move |
| `getActivatePositionController()` | Read current position-controller setting |

---

## moreflag values (spline chaining)

| Value | Meaning |
|---|---|
| `0` | Start moving, more segments will follow — firmware chains without stopping |
| `1` | This is the last (or only) segment — firmware decelerates to a stop |
| `2` | Load segment into buffer but do not start moving yet |

## TMotStsFlg status flags

| Flag | Value | Meaning |
|---|---|---|
| `MSF_MECHSTOP` | 1 | Mechanical stop reached |
| `MSF_MAXPOS` | 2 | Maximum position reached |
| `MSF_MINPOS` | 4 | Minimum position reached / calibrating |
| `MSF_DESPOS` | 8 | In desired position / holding |
| `MSF_NORMOPSTAT` | 16 | Trying to follow target / moving |
| `MSF_MOTCRASHED` | 40 | Motor crashed / collision |
| `MSF_NLINMOV` | 88 | Non-linear (poly) move finished |
| `MSF_LINMOV` | 152 | Moving poly, buffer full |
| `MSF_NOTVALID` | 128 | Motor data not valid |

---

## Driver usage note

`katana_hardware_interface.cpp` uses the spline path exclusively:

```
sendSplineToMotor(motor, targetEnc, kSplineT=40, p1..p4)   // per motor
startSplineMovement(exactflag=true, moreflag)               // fire
```

`moveRobotToEnc` / `moveMotorToEnc` (TPS path) are **not used** — they decelerate to
zero before accepting the next target, causing visible stop-start motion at every waypoint.
