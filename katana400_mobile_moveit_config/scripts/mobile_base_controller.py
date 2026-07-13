#!/usr/bin/env python3
"""
Bridge: MoveIt FollowJointTrajectory → Nav2 NavigateThroughPoses.

Forwards all MoveIt trajectory waypoints (base_planar_joint/x,y,theta) to Nav2's
NavigateThroughPoses action. Nav2 passes through intermediate poses and stops
precisely at the final one using the RPP controller with odom feedback.

Architecture:
  MoveIt → FollowJointTrajectory (base_planar_joint/x,y,theta)
         → mobile_base_controller (this node)
         → NavigateThroughPoses → Nav2 (RPP, odom frame)
         → /diff_drive_controller/cmd_vel (TwistStamped)
"""
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from control_msgs.action import FollowJointTrajectory
from nav2_msgs.action import NavigateThroughPoses
from geometry_msgs.msg import PoseStamped, Quaternion
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException


def _yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _yaw_to_quat(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


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
        self._nav2_client = ActionClient(
            self, NavigateThroughPoses, '/navigate_through_poses',
            callback_group=cb_group,
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.get_logger().info('MobileBaseController ready (Nav2 path-following mode)')

    def goal_callback(self, _goal):
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _get_pose(self):
        try:
            t = self._tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
            return (t.transform.translation.x,
                    t.transform.translation.y,
                    _yaw_from_quat(t.transform.rotation))
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def execute_callback(self, goal_handle):
        traj = goal_handle.request.trajectory
        jnames = list(traj.joint_names)
        self.get_logger().info(
            f'Base goal: {len(traj.points)} waypoints, joints={jnames}')

        if not jnames or not traj.points:
            goal_handle.succeed()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return result

        try:
            ix = jnames.index('base_planar_joint/x')
            iy = jnames.index('base_planar_joint/y')
            it = jnames.index('base_planar_joint/theta')
        except ValueError:
            self.get_logger().error(f'Unexpected joint names: {jnames}')
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        # Downsample: every 3rd point keeps ~65 waypoints from 195; always include last.
        step = 3
        sampled = list(range(0, len(traj.points), step))
        if (len(traj.points) - 1) not in sampled:
            sampled.append(len(traj.points) - 1)

        now = self.get_clock().now().to_msg()
        poses = []
        for i in sampled:
            pt = traj.points[i]
            ps = PoseStamped()
            ps.header.frame_id = 'odom'
            ps.header.stamp = now
            ps.pose.position.x = pt.positions[ix]
            ps.pose.position.y = pt.positions[iy]
            ps.pose.orientation = _yaw_to_quat(pt.positions[it])
            poses.append(ps)

        final = traj.points[-1]
        gx = final.positions[ix]
        gy = final.positions[iy]
        gt = final.positions[it]

        self.get_logger().info(
            f'Forwarding {len(poses)}/{len(traj.points)} waypoints to Nav2, '
            f'final=({gx:.3f}, {gy:.3f}, {math.degrees(gt):.1f}°)')

        if not self._nav2_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('Nav2 /navigate_through_poses not available after 15s')
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        nav_goal = NavigateThroughPoses.Goal()
        nav_goal.poses = poses

        done = threading.Event()
        nav2_gh_holder = [None]
        nav2_result = [None]

        def on_goal_response(future):
            gh = future.result()
            nav2_gh_holder[0] = gh
            if not gh.accepted:
                self.get_logger().error('Nav2 rejected the goal')
                done.set()
                return
            gh.get_result_async().add_done_callback(on_result)

        def on_result(future):
            nav2_result[0] = future.result()
            done.set()

        self._nav2_client.send_goal_async(nav_goal).add_done_callback(on_goal_response)

        while not done.wait(timeout=0.5):
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Cancel requested — cancelling Nav2 goal')
                nav2_gh = nav2_gh_holder[0]
                if nav2_gh:
                    nav2_gh.cancel_goal_async()
                done.wait(timeout=3.0)
                goal_handle.canceled()
                return FollowJointTrajectory.Result()

        pose = self._get_pose()
        if pose:
            self.get_logger().info(
                f'Nav2 done. final=({pose[0]:.3f}, {pose[1]:.3f}, {math.degrees(pose[2]):.1f}°)'
                f' target=({gx:.3f}, {gy:.3f}, {math.degrees(gt):.1f}°)')

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result


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
