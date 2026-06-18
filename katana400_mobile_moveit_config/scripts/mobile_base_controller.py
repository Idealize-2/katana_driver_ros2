#!/usr/bin/env python3
"""
Bridge: MoveIt FollowJointTrajectory → diff_drive_controller cmd_vel.

MoveIt plans for base_planar_joint (virtual planar joint, SRDF) and sends
FollowJointTrajectory goals containing the 3 variable names:
  joint_names = ['base_planar_joint/x', 'base_planar_joint/y', 'base_planar_joint/theta']

This node converts those waypoints into TwistStamped cmd_vel commands, using the
odom→base_footprint TF for closed-loop feedback.
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import TwistStamped
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException


def _yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _angle_diff(a, b):
    """Signed difference (a − b) wrapped to [−π, π]."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


class MobileBaseController(Node):
    def __init__(self):
        super().__init__('mobile_base_controller')
        cb_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/base_controller/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=cb_group,
        )
        self._cmd_vel_pub = self.create_publisher(
            TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.declare_parameter('goal_tolerance_xy',    0.05)   # m
        self.declare_parameter('goal_tolerance_theta', 0.15)   # rad (~8.6 deg)
        self.declare_parameter('kp_linear',  0.6)
        self.declare_parameter('kp_angular', 1.2)
        self.declare_parameter('max_linear_vel',  0.6)         # m/s
        self.declare_parameter('max_angular_vel', 1.0)         # rad/s
        self.declare_parameter('min_angular_vel', 0.1)        # rad/s — minimum to break static friction
        self.declare_parameter('control_rate_hz', 20.0)

        self.get_logger().info('MobileBaseController ready on /base_controller/follow_joint_trajectory')

    def goal_callback(self, _goal):
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _get_pose(self):
        """Return (x, y, theta) from odom→base_footprint TF, or None if unavailable."""
        try:
            t = self._tf_buffer.lookup_transform(
                'odom', 'base_footprint', rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            theta = _yaw_from_quat(t.transform.rotation)
            return x, y, theta
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def execute_callback(self, goal_handle):
        traj   = goal_handle.request.trajectory
        jnames = list(traj.joint_names)
        self.get_logger().info(
            f'Base goal: {len(traj.points)} waypoints, joints={jnames}')

        # Empty trajectory = MoveIt contacting all controllers for a plan that doesn't
        # involve the base (e.g. arm-only execution). Succeed immediately as a no-op.
        if not jnames or not traj.points:
            goal_handle.succeed()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return result

        # MoveIt encodes a planar joint's 3 DOF as separate variable names.
        try:
            ix = jnames.index('base_planar_joint/x')
            iy = jnames.index('base_planar_joint/y')
            it = jnames.index('base_planar_joint/theta')
        except ValueError:
            self.get_logger().error(
                f'Expected base_planar_joint/x|y|theta in joint_names; got: {jnames}')
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        tol_xy  = self.get_parameter('goal_tolerance_xy').value
        tol_t   = self.get_parameter('goal_tolerance_theta').value
        kp_l    = self.get_parameter('kp_linear').value
        kp_a    = self.get_parameter('kp_angular').value
        max_l   = self.get_parameter('max_linear_vel').value
        max_a   = self.get_parameter('max_angular_vel').value
        dt      = 1.0 / self.get_parameter('control_rate_hz').value

        for point in traj.points:
            tx = point.positions[ix]
            ty = point.positions[iy]
            tt = point.positions[it]

            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self._stop()
                    goal_handle.canceled()
                    return FollowJointTrajectory.Result()

                pose = self._get_pose()
                if pose is None:
                    time.sleep(dt)
                    continue

                cx, cy, ct = pose
                dx    = tx - cx
                dy    = ty - cy
                dist  = math.hypot(dx, dy)
                dhead = _angle_diff(math.atan2(dy, dx), ct)   # heading toward goal
                dfin  = _angle_diff(tt, ct)                   # final orientation error

                if dist < tol_xy and abs(dfin) < tol_t:
                    break

                cmd = TwistStamped()
                cmd.header.frame_id = 'base_footprint'
                cmd.header.stamp = self.get_clock().now().to_msg()
                if dist > tol_xy:
                    cmd.twist.linear.x  = max(-max_l, min(max_l, kp_l * dist * math.cos(dhead)))
                    cmd.twist.angular.z = max(-max_a, min(max_a, kp_a * dhead))
                else:
                    cmd.twist.angular.z = max(-max_a, min(max_a, kp_a * dfin))

                self.get_logger().info(
                    f'pose=({cx:.2f},{cy:.2f},{math.degrees(ct):.1f}°) '
                    f'target=({tx:.2f},{ty:.2f},{math.degrees(tt):.1f}°) '
                    f'dist={dist:.3f} dhead={math.degrees(dhead):.1f}° dfin={math.degrees(dfin):.1f}° '
                    f'→ lin={cmd.twist.linear.x:.3f} ang={cmd.twist.angular.z:.3f}'
                )
                self._cmd_vel_pub.publish(cmd)
                time.sleep(dt)  # wall-clock rate (feedback-based so timing imprecision is fine)

        self._stop()
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

    def _stop(self):
        self._cmd_vel_pub.publish(TwistStamped())


def main(args=None):
    rclpy.init(args=args)
    node = MobileBaseController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
