#!/usr/bin/env python3
from __future__ import annotations

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster

from scipy.spatial.transform import Rotation as R


def quat_to_mat(x: float, y: float, z: float, w: float) -> np.ndarray:
    M = np.eye(4)
    M[0:3, 0:3] = R.from_quat([x, y, z, w]).as_matrix()
    return M


def mat_to_quat(M: np.ndarray):
    # returns x,y,z,w
    return R.from_matrix(M[0:3, 0:3]).as_quat()


def pose_to_mat(pose) -> np.ndarray:
    q = pose.orientation
    p = pose.position
    M = quat_to_mat(q.x, q.y, q.z, q.w)
    M[0, 3] = p.x
    M[1, 3] = p.y
    M[2, 3] = p.z
    return M


def tf_to_mat(tf: TransformStamped) -> np.ndarray:
    q = tf.transform.rotation
    p = tf.transform.translation
    M = quat_to_mat(q.x, q.y, q.z, q.w)
    M[0, 3] = p.x
    M[1, 3] = p.y
    M[2, 3] = p.z
    return M


def inv_T(M: np.ndarray) -> np.ndarray:
    Rm = M[0:3, 0:3]
    t = M[0:3, 3]
    Rt = Rm.T
    Minv = np.eye(4)
    Minv[0:3, 0:3] = Rt
    Minv[0:3, 3] = -(Rt @ t)
    return Minv


def mat_to_tf(M: np.ndarray, parent: str, child: str, stamp_msg) -> TransformStamped:
    t = TransformStamped()
    t.header.stamp = stamp_msg
    t.header.frame_id = parent
    t.child_frame_id = child

    t.transform.translation.x = float(M[0, 3])
    t.transform.translation.y = float(M[1, 3])
    t.transform.translation.z = float(M[2, 3])

    q = mat_to_quat(M)  # x,y,z,w
    t.transform.rotation.x = float(q[0])
    t.transform.rotation.y = float(q[1])
    t.transform.rotation.z = float(q[2])
    t.transform.rotation.w = float(q[3])
    return t


class ArucoPoseLocalization(Node):
    def __init__(self):
        super().__init__("aruco_pose_localization")

        # Frames
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("known_marker_frame", "wall_marker_7")

        # Pose input
        self.declare_parameter("pose_topic", "/aruco_single/pose")

        # Robustness
        self.declare_parameter("max_jump_m", 1.0)
        self.declare_parameter("ema_alpha", 0.35)
        self.declare_parameter("min_z_m", -5.0)
        self.declare_parameter("max_z_m", 5.0)

        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.known_marker_frame = self.get_parameter("known_marker_frame").value
        self.pose_topic = self.get_parameter("pose_topic").value

        self.max_jump = float(self.get_parameter("max_jump_m").value)
        self.alpha = float(self.get_parameter("ema_alpha").value)
        self.min_z = float(self.get_parameter("min_z_m").value)
        self.max_z = float(self.get_parameter("max_z_m").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.last_xyz = None

        self.sub = self.create_subscription(PoseStamped, self.pose_topic, self._pose_cb, 10)

        self.get_logger().info(
            f"Listening to {self.pose_topic}. Publishing TF {self.map_frame} -> {self.base_frame} "
            f"using known marker {self.known_marker_frame}."
        )

    def _pose_cb(self, msg: PoseStamped):
        cam_frame = msg.header.frame_id.strip() if msg.header.frame_id else ""
        if not cam_frame:
            self.get_logger().warn("Aruco pose has empty header.frame_id; cannot use.")
            return

        now = self.get_clock().now()

        # Lookup:
        #   map -> marker (known static)
        #   base -> cam   (URDF static via robot_state_publisher)
        try:
            T_map_marker = self.tf_buffer.lookup_transform(
                self.map_frame, self.known_marker_frame, Time()
            )
            T_base_cam = self.tf_buffer.lookup_transform(
                self.base_frame, cam_frame, Time()
            )
        except Exception as e:
            self.get_logger().debug(f"TF lookup failed: {e}")
            return

        M_map_marker = tf_to_mat(T_map_marker)
        M_base_cam = tf_to_mat(T_base_cam)

        # Aruco gives marker pose expressed in camera frame: cam -> marker
        M_cam_marker = pose_to_mat(msg.pose)

        # T_map_base = T_map_marker * inv(T_cam_marker) * inv(T_base_cam)
        M_map_base = M_map_marker @ inv_T(M_cam_marker) @ inv_T(M_base_cam)

        # sanity z
        z = float(M_map_base[2, 3])
        if z < self.min_z or z > self.max_z:
            self.get_logger().warn(f"Rejecting pose (z={z:.3f} out of bounds).")
            return

        # jump reject + EMA translation
        xyz = (float(M_map_base[0, 3]), float(M_map_base[1, 3]), float(M_map_base[2, 3]))
        if self.last_xyz is not None:
            dx = xyz[0] - self.last_xyz[0]
            dy = xyz[1] - self.last_xyz[1]
            dz = xyz[2] - self.last_xyz[2]
            jump = math.sqrt(dx * dx + dy * dy + dz * dz)
            if jump > self.max_jump:
                self.get_logger().warn(f"Rejecting jump {jump:.2f} m")
                return

            M_map_base[0, 3] = self.alpha * self.last_xyz[0] + (1.0 - self.alpha) * xyz[0]
            M_map_base[1, 3] = self.alpha * self.last_xyz[1] + (1.0 - self.alpha) * xyz[1]
            M_map_base[2, 3] = self.alpha * self.last_xyz[2] + (1.0 - self.alpha) * xyz[2]

        self.last_xyz = (float(M_map_base[0, 3]), float(M_map_base[1, 3]), float(M_map_base[2, 3]))

        out_tf = mat_to_tf(M_map_base, self.map_frame, self.base_frame, now.to_msg())
        self.tf_broadcaster.sendTransform(out_tf)


def main():
    rclpy.init()
    node = ArucoPoseLocalization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
