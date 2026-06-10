#!/usr/bin/env python3
"""Publishes a TEXT_VIEW_FACING marker in RViz2 showing the EEF pose (x,y,z,roll,pitch,yaw)."""

import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros


def _quat_to_rpy(x, y, z, w):
    roll  = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin (max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw   = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


class EefMarkerNode(Node):
    BASE_FRAME = "katana_base_link"
    EEF_FRAME  = "katana_gripper_link"
    TOPIC      = "/katana/eef_marker"
    RATE_HZ    = 10.0
    TEXT_SCALE = 0.04   # metres — marker text height in RViz2

    def __init__(self):
        super().__init__("eef_marker")
        self._pub = self.create_publisher(MarkerArray, self.TOPIC, 10)
        self._tf  = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tf, self)
        self.create_timer(1.0 / self.RATE_HZ, self._tick)

    def _tick(self):
        try:
            tf = self._tf.lookup_transform(
                self.BASE_FRAME,
                self.EEF_FRAME,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return

        t = tf.transform.translation
        q = tf.transform.rotation
        roll, pitch, yaw = _quat_to_rpy(q.x, q.y, q.z, q.w)

        rd, pd, yd = (math.degrees(a) for a in (roll, pitch, yaw))
        text = (
            f"x:{t.x:.3f}\n"
            f"y:{t.y:.3f}\n"
            f"z:{t.z:.3f}\n"
            f"r:{rd:.1f}°\n"
            f"p:{pd:.1f}°\n"
            f"y:{yd:.1f}°"
        )

        m = Marker()
        m.header.frame_id = self.EEF_FRAME
        m.header.stamp    = rclpy.time.Time().to_msg()  # 0 = use latest TF, avoids blink
        m.ns              = "eef_pose"
        m.id              = 0
        m.type            = Marker.TEXT_VIEW_FACING
        m.action          = Marker.ADD
        # float slightly above the gripper so it doesn't clip the mesh
        m.pose.position.z = 0.06
        m.pose.orientation.w = 1.0
        m.scale.z         = self.TEXT_SCALE
        m.color.r         = 1.0
        m.color.g         = 1.0
        m.color.b         = 0.2
        m.color.a         = 1.0
        m.text            = text

        arr = MarkerArray()
        arr.markers.append(m)
        self._pub.publish(arr)


def main():
    rclpy.init()
    node = EefMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
