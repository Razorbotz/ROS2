#!/usr/bin/env python3
"""
Publishes map -> odom by combining:
  - map -> tag36h11:N        (static, known tag position)
  - camera_optical_frame -> tag36h11:N  (detected by apriltag_ros)
  - odom -> ... -> camera_optical_frame (URDF + odometry)

Result: map -> odom, which closes the TF tree.

No external dependencies beyond rclpy, tf2_ros, and numpy.
"""

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion
import numpy as np


def quaternion_to_matrix(q):
    """Convert geometry_msgs Quaternion to a 4x4 homogeneous matrix."""
    x, y, z, w = q.x, q.y, q.z, q.w
    n = w*w + x*x + y*y + z*z
    s = 2.0 / n if n > 0.0 else 0.0

    wx = s * w * x; wy = s * w * y; wz = s * w * z
    xx = s * x * x; xy = s * x * y; xz = s * x * z
    yy = s * y * y; yz = s * y * z; zz = s * z * z

    return np.array([
        [1-(yy+zz),    xy-wz,    xz+wy, 0],
        [   xy+wz, 1-(xx+zz),    yz-wx, 0],
        [   xz-wy,    yz+wx, 1-(xx+yy), 0],
        [       0,        0,        0,   1],
    ])


def matrix_to_quaternion(m):
    """Extract a quaternion from a 4x4 homogeneous matrix."""
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m[2, 1] - m[1, 2]) * s
        y = (m[0, 2] - m[2, 0]) * s
        z = (m[1, 0] - m[0, 1]) * s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return (x, y, z, w)


def tf_to_matrix(tf):
    """Convert a geometry_msgs Transform to a 4x4 matrix."""
    mat = quaternion_to_matrix(tf.rotation)
    mat[0, 3] = tf.translation.x
    mat[1, 3] = tf.translation.y
    mat[2, 3] = tf.translation.z
    return mat


def matrix_to_tf(mat):
    """Convert a 4x4 matrix to a geometry_msgs Transform."""
    q = matrix_to_quaternion(mat)
    t = Transform()
    t.translation = Vector3(x=float(mat[0, 3]), y=float(mat[1, 3]), z=float(mat[2, 3]))
    t.rotation = Quaternion(x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3]))
    return t


class AprilTagLocalization(Node):
    def __init__(self):
        super().__init__('apriltag_localization')

        # --- Parameters ---
        self.declare_parameter('tag_frame', 'tag36h11:7')
        self.declare_parameter('camera_frame', 'camera_optical_frame')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('publish_rate', 10.0)

        self.tag_frame = self.get_parameter('tag_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        rate = self.get_parameter('publish_rate').value

        # --- TF ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(1.0 / rate, self.update)
        self.get_logger().info(
            f'Localizing: {self.map_frame} -> {self.odom_frame} '
            f'via tag "{self.tag_frame}" seen from "{self.camera_frame}"'
        )

    def update(self):
        try:
            # map -> tag (static, known)
            map_to_tag = self.tf_buffer.lookup_transform(
                self.map_frame, self.tag_frame, rclpy.time.Time()
            )
            # camera -> tag (detected by apriltag_ros)
            cam_to_tag = self.tf_buffer.lookup_transform(
                self.camera_frame, self.tag_frame, rclpy.time.Time()
            )
            # odom -> camera (from URDF chain + odometry)
            odom_to_cam = self.tf_buffer.lookup_transform(
                self.odom_frame, self.camera_frame, rclpy.time.Time()
            )
        except Exception:
            return

        # Convert to 4x4 matrices
        M_map_tag = tf_to_matrix(map_to_tag.transform)
        M_cam_tag = tf_to_matrix(cam_to_tag.transform)
        M_odom_cam = tf_to_matrix(odom_to_cam.transform)

        # map -> camera = map -> tag  @  (camera -> tag)^-1
        M_map_cam = M_map_tag @ np.linalg.inv(M_cam_tag)

        # map -> odom = map -> camera  @  (odom -> camera)^-1
        M_map_odom = M_map_cam @ np.linalg.inv(M_odom_cam)

        # Publish
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        t.transform = matrix_to_tf(M_map_odom)
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = AprilTagLocalization()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()