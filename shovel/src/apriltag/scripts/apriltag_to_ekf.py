#!/usr/bin/env python3
"""AprilTag → EKF pose translator.

Looks up detected tag frames from the TF tree (published by apriltag_ros),
applies the known tag positions in the arena, and publishes the robot's
pose in the map frame for the EKF.

Transform chain:
  M_map_base = M_map_tag @ inv(M_base_tag)

Where:
  M_map_tag  — Known position of the tag in the arena (hardcoded below)
  M_base_tag — Looked up from TF: base_link → tag36h11:<id>
               (apriltag_ros publishes camera → tag, robot_state_publisher
                provides base_link → camera, so TF can chain them)
"""
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import math


def quaternion_to_matrix(q):
    """Convert a geometry_msgs Quaternion to a 4x4 homogeneous rotation matrix."""
    x, y, z, w = q.x, q.y, q.z, q.w
    n = w*w + x*x + y*y + z*z
    s = 2.0 / n if n > 0.0 else 0.0
    wx = s*w*x; wy = s*w*y; wz = s*w*z
    xx = s*x*x; xy = s*x*y; xz = s*x*z
    yy = s*y*y; yz = s*y*z; zz = s*z*z
    return np.array([
        [1-(yy+zz), xy-wz, xz+wy, 0],
        [xy+wz, 1-(xx+zz), yz-wx, 0],
        [xz-wy, yz+wx, 1-(xx+yy), 0],
        [0, 0, 0, 1],
    ])


def matrix_to_quaternion(m):
    """Convert a 3x3 or 4x4 rotation matrix to (x, y, z, w) quaternion.

    Uses the Shepperd method which correctly handles all rotation cases
    by selecting the numerically stable branch based on the largest
    diagonal element.
    """
    # The previous implementation only handled trace > 0 and one fallback
    # case (z-dominant). This failed when m[0,0] or m[1,1] was the largest
    # diagonal — producing wrong quaternions that caused orientation jumps.
    trace = m[0, 0] + m[1, 1] + m[2, 2]

    if trace > 0:
        s = 2.0 * math.sqrt(trace + 1.0)
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s

    # Normalize to unit quaternion
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if norm > 0:
        x /= norm; y /= norm; z /= norm; w /= norm

    return (x, y, z, w)


def pose_from_sdf(x, y, z, roll, pitch, yaw):
    """Build a 4x4 transform from Gazebo SDF pose values.

    Gazebo SDF <pose> is: x y z roll pitch yaw
    Rotation order: ZYX (yaw then pitch then roll), same as ROS convention.
    This produces the exact same transform Gazebo uses internally.
    """
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    M = np.eye(4)
    M[0, 0] = cy * cp
    M[0, 1] = cy * sp * sr - sy * cr
    M[0, 2] = cy * sp * cr + sy * sr
    M[1, 0] = sy * cp
    M[1, 1] = sy * sp * sr + cy * cr
    M[1, 2] = sy * sp * cr - cy * sr
    M[2, 0] = -sp
    M[2, 1] = cp * sr
    M[2, 2] = cp * cr

    M[0, 3] = x
    M[1, 3] = y
    M[2, 3] = z
    return M


class TagToEKF(Node):
    def __init__(self):
        super().__init__('apriltag_to_ekf')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')

        self.base_frame = self.get_parameter('base_frame').value
        self.map_frame = self.get_parameter('map_frame').value

        # --- THE ARENA MAP ---
        # Tag poses taken directly from the Gazebo world SDF.
        # Format: pose_from_sdf(x, y, z, roll, pitch, yaw)
        # These must match the <pose> values in the .world file exactly.
        self.KNOWN_TAGS = {
            # <pose>3.4 1.8 0.4 0 -1.58 0</pose>
            'tag36h11:7':  pose_from_sdf(3.4, 1.8, 0.4, 0.0, -1.58, 0.0),
            # <pose>2.5 2.5 0.4 1.58 1.58 0</pose>
            'tag36h11:11': pose_from_sdf(2.5, 2.5, 0.4, 1.58, 1.58, 0.0),
        }

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/apriltag_pose', 10)
        self.timer = self.create_timer(0.1, self.update)

        self.last_published_tag = None
        self.get_logger().info(
            f"Translator initialized. Tracking {len(self.KNOWN_TAGS)} tags: "
            f"{list(self.KNOWN_TAGS.keys())}")

    def update(self):
        for tag_frame, M_map_tag in self.KNOWN_TAGS.items():
            try:
                now = rclpy.time.Time()
                timeout = rclpy.duration.Duration(seconds=0.1)

                # Look up base_link → tag via the camera chain:
                #   base_link → zed2i_base_link → zed2i_left_optical_frame → tag
                # TF can resolve the full chain automatically.
                base_to_tag = self.tf_buffer.lookup_transform(
                    self.base_frame, tag_frame, now, timeout)
            except Exception:
                continue

            # Build 4x4 matrix from the TF lookup
            M_base_tag = quaternion_to_matrix(base_to_tag.transform.rotation)
            M_base_tag[0, 3] = base_to_tag.transform.translation.x
            M_base_tag[1, 3] = base_to_tag.transform.translation.y
            M_base_tag[2, 3] = base_to_tag.transform.translation.z

            # The key equation: robot pose in map = tag pose in map * inv(tag pose in base)
            M_map_base = M_map_tag @ np.linalg.inv(M_base_tag)

            q = matrix_to_quaternion(M_map_base)

            # Sanity check: reject if the result has NaN (can happen if
            # the detection is degenerate)
            if any(math.isnan(v) for v in [M_map_base[0, 3], M_map_base[1, 3], *q]):
                self.get_logger().warn(f"NaN in pose from {tag_frame}, skipping")
                continue

            # Publish
            msg = PoseWithCovarianceStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.map_frame
            msg.pose.pose.position.x = float(M_map_base[0, 3])
            msg.pose.pose.position.y = float(M_map_base[1, 3])
            msg.pose.pose.position.z = 0.0  # 2D navigation

            msg.pose.pose.orientation.x = float(q[0])
            msg.pose.pose.orientation.y = float(q[1])
            msg.pose.pose.orientation.z = float(q[2])
            msg.pose.pose.orientation.w = float(q[3])

            cov = np.zeros((6, 6))
            # Lower = more trust. We trust XY and yaw, distrust Z/roll/pitch.
            np.fill_diagonal(cov, [0.05, 0.05, 999.0, 999.0, 999.0, 0.1])
            msg.pose.covariance = cov.flatten().tolist()

            self.pose_pub.publish(msg)

            if self.last_published_tag != tag_frame:
                self.get_logger().info(
                    f"Publishing pose from {tag_frame}: "
                    f"x={M_map_base[0,3]:.2f} y={M_map_base[1,3]:.2f} "
                    f"yaw={math.atan2(2*(q[3]*q[2]+q[0]*q[1]), 1-2*(q[1]**2+q[2]**2))*180/math.pi:.1f} deg")
                self.last_published_tag = tag_frame

            # Use first visible tag, don't double-publish in one cycle
            return


def main():
    rclpy.init()
    node = TagToEKF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()