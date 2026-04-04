#!/usr/bin/env python3
"""
Publishes visualization_msgs/MarkerArray for known AprilTag positions.

Subscribes to /tf and /tf_static to read tag frames (map -> tag36h11:N)
and publishes colored square markers so you can see them in Foxglove/RViz.

Also listens for detected tag transforms (camera -> tag) and publishes
those as a separate colored marker so you can compare known vs detected.
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
from tf2_ros import Buffer, TransformListener
import numpy as np


TAG_SIZE = 0.3  # meters, matching your world file


def quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    n = w*w + x*x + y*y + z*z
    s = 2.0 / n if n > 0.0 else 0.0
    wx = s*w*x; wy = s*w*y; wz = s*w*z
    xx = s*x*x; xy = s*x*y; xz = s*x*z
    yy = s*y*y; yz = s*y*z; zz = s*z*z
    return np.array([
        [1-(yy+zz), xy-wz, xz+wy],
        [xy+wz, 1-(xx+zz), yz-wx],
        [xz-wy, yz+wx, 1-(xx+yy)],
    ])


class TagVisualizer(Node):
    def __init__(self):
        super().__init__('tag_visualizer')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('tag_frames', ['tag36h11:7', 'tag36h11:11'])
        self.declare_parameter('publish_rate', 2.0)

        self.map_frame = self.get_parameter('map_frame').value
        self.tag_frames = self.get_parameter('tag_frames').value
        rate = self.get_parameter('publish_rate').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.marker_pub = self.create_publisher(
            MarkerArray, '/apriltag_markers', 10
        )

        self.timer = self.create_timer(1.0 / rate, self.publish_markers)
        self.get_logger().info(
            f'Visualizing tags {self.tag_frames} in frame "{self.map_frame}"'
        )

    def make_marker(self, tag_frame, marker_id, r, g, b, ns='known_tags'):
        """Try to look up map -> tag_frame and return a Marker, or None."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, tag_frame, rclpy.time.Time()
            )
        except Exception:
            return None

        m = Marker()
        m.header.frame_id = self.map_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = marker_id
        m.type = Marker.CUBE
        m.action = Marker.ADD

        m.pose.position.x = tf.transform.translation.x
        m.pose.position.y = tf.transform.translation.y
        m.pose.position.z = tf.transform.translation.z
        m.pose.orientation = tf.transform.rotation

        # Flat square oriented with the tag face
        m.scale.x = TAG_SIZE
        m.scale.y = TAG_SIZE
        m.scale.z = 0.005

        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = 0.85

        m.lifetime = Duration(sec=1, nanosec=0)

        # Add text label above the tag
        label = Marker()
        label.header = m.header
        label.ns = ns + '_labels'
        label.id = marker_id
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = tf.transform.translation.x
        label.pose.position.y = tf.transform.translation.y
        label.pose.position.z = tf.transform.translation.z + 0.25
        label.scale.z = 0.12
        label.color.r = 1.0
        label.color.g = 1.0
        label.color.b = 1.0
        label.color.a = 1.0
        label.text = tag_frame
        label.lifetime = Duration(sec=1, nanosec=0)

        return [m, label]

    def publish_markers(self):
        markers = MarkerArray()

        for i, tag_frame in enumerate(self.tag_frames):
            # Known position (from static TF) — green
            result = self.make_marker(tag_frame, i, 0.0, 0.8, 0.2, 'known_tags')
            if result:
                markers.markers.extend(result)

        if markers.markers:
            self.marker_pub.publish(markers)


def main():
    rclpy.init()
    node = TagVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()