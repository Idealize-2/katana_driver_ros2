# katana_moveit_ikfast_plugin — Not Ported to ROS 2

## Why IKFast was not brought forward

IKFast is an analytical IK solver generator built inside OpenRAVE (~2008–2010).
It produces exact closed-form solutions in microseconds and was the best option available
at the time — nothing numerical came close in speed or reliability.

However, the gap has closed. Modern numerical solvers like **TRAC-IK** and **pick_ik**
achieve near-analytical reliability through hybrid strategies (combining Jacobian pseudoinverse
with random restarts and constraint relaxation). For a 5-DOF arm like the Katana 400, the
practical difference in planning success rate and solve time is negligible.

Meanwhile, IKFast's only path to a new solver remains OpenRAVE — an unmaintained project
frozen in Python 2, abandoned by its author when he moved to commercial work. The algorithm
is mathematically complete and was never improved because it did not need to be. The toolchain
around it simply decayed.

## What to use instead

Both are ROS 2 native, install via `apt`, and require only a one-line change in `kinematics.yaml`:

```yaml
# drop-in for KDL, much more reliable near singularities
kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin

# or: modern gradient-descent solver, actively maintained for ROS 2
kinematics_solver: pick_ik/PickIkPlugin
```

The current configuration uses KDL, which is sufficient. If planning failures appear near
workspace boundaries, switch to TRAC-IK first.
