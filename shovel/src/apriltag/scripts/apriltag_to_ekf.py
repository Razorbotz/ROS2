#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import math

def quaternion_to_matrix(q):
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
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m[2, 1] - m[1, 2]) * s
        y = (m[0, 2] - m[2, 0]) * s
        z = (m[1, 0] - m[0, 1]) * s
    else:
        s = 2.0 * math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return (x, y, z, w)

def create_wall_tag(x, y, z, facing_yaw_deg):
    """
    Automatically generates a perfect AprilTag-compliant rotation matrix.
    facing_yaw_deg: The direction the tag is looking into the arena room.
    """
    rad = math.radians(facing_yaw_deg)
    
    # Tag X is Right (90 degrees offset from where it's looking)
    rad_x = rad + math.pi/2
    # Tag Z is into the wall (180 degrees offset from where it's looking)
    rad_z = rad + math.pi
    
    M = np.eye(4)
    M[0, 0] = math.cos(rad_x); M[0, 1] = 0.0;  M[0, 2] = math.cos(rad_z)
    M[1, 0] = math.sin(rad_x); M[1, 1] = 0.0;  M[1, 2] = math.sin(rad_z)
    M[2, 0] = 0.0;             M[2, 1] = -1.0; M[2, 2] = 0.0 # Y strictly points Down
    
    M[0, 3] = x; M[1, 3] = y; M[2, 3] = z
    return M

class TagToEKF(Node):
    def __init__(self):
        super().__init__('apriltag_to_ekf')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        
        self.base_frame = self.get_parameter('base_frame').value
        self.map_frame = self.get_parameter('map_frame').value

        # --- THE ARENA MAP ---
        self.KNOWN_TAGS = {
            'tag36h11:7': create_wall_tag(x=3.4, y=1.8, z=0.4, facing_yaw_deg=180.0),
            'tag36h11:11': create_wall_tag(x=2.5, y=2.5, z=0.4, facing_yaw_deg=0.0)
        }
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/apriltag_pose', 10)
        self.timer = self.create_timer(0.1, self.update)
        
        self.get_logger().info(f"Translator Initialized. Tracking {len(self.KNOWN_TAGS)} tags.")

    def update(self):
        for tag_frame, M_map_tag in self.KNOWN_TAGS.items():
            try:
                now = rclpy.time.Time()
                timeout = rclpy.duration.Duration(seconds=0.1)
                
                # Look up base_link -> tag via the camera
                base_to_tag = self.tf_buffer.lookup_transform(
                    self.base_frame, tag_frame, now, timeout)
            except Exception:
                continue 

            # Matrix Math
            M_base_tag = quaternion_to_matrix(base_to_tag.transform.rotation)
            M_base_tag[0, 3] = base_to_tag.transform.translation.x
            M_base_tag[1, 3] = base_to_tag.transform.translation.y
            M_base_tag[2, 3] = base_to_tag.transform.translation.z
            
            M_map_base = M_map_tag @ np.linalg.inv(M_base_tag)
            q = matrix_to_quaternion(M_map_base)

            # Package and Publish
            msg = PoseWithCovarianceStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.map_frame
            msg.pose.pose.position.x = float(M_map_base[0, 3])
            msg.pose.pose.position.y = float(M_map_base[1, 3])
            # Force Z to 0 to keep the covariance matrix happy
            msg.pose.pose.position.z = 0.0 
            
            msg.pose.pose.orientation.x = float(q[0])
            msg.pose.pose.orientation.y = float(q[1])
            msg.pose.pose.orientation.z = float(q[2])
            msg.pose.pose.orientation.w = float(q[3])

            cov = np.zeros((6, 6))
            # Lower number = Higher trust. We trust XY, we distrust Z/Roll/Pitch.
            np.fill_diagonal(cov, [0.05, 0.05, 999.0, 999.0, 999.0, 0.1])
            msg.pose.covariance = cov.flatten().tolist()

            self.pose_pub.publish(msg)

def main():
    rclpy.init()
    node = TagToEKF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()