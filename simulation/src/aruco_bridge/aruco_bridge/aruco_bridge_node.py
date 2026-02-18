import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from messages.msg import ZedPosition
import math
from tf_transformations import euler_from_quaternion

class SimZedBridge(Node):
    def __init__(self):
        super().__init__('sim_zed_bridge')
        
        self.pub = self.create_publisher(ZedPosition, '/zed_position', 10)
        
        # Ground truth odom from p3d plugin
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        
        # ArUco detection
        self.create_subscription(PoseStamped, '/aruco_single/pose', self.aruco_cb, 10)
        
        self.latest_aruco = None
        self.aruco_visible = False
        self.aruco_initialized = False
        self.aruco_timeout = 1.0  # seconds
        self.last_aruco_time = None

    def aruco_cb(self, msg: PoseStamped):
        self.latest_aruco = msg
        self.aruco_visible = True
        self.aruco_initialized = True
        self.last_aruco_time = self.get_clock().now()

    def odom_cb(self, msg: Odometry):
        # Check aruco timeout
        if self.last_aruco_time:
            elapsed = (self.get_clock().now() - self.last_aruco_time).nanoseconds / 1e9
            if elapsed > self.aruco_timeout:
                self.aruco_visible = False

        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        vel = msg.twist.twist.linear
        
        roll, pitch, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])

        zp = ZedPosition()
        # Position mapping: Gazebo (X-fwd, Y-left, Z-up) → ZED LEFT_HANDED_Y_UP (X-right, Y-up, Z-fwd)
        zp.x = float(pos.y + 3.44)          # ZED X (right) = Gazebo -Y
        zp.y = float(pos.z)           # ZED Y (up) = Gazebo Z  
        zp.z = float(pos.x + 2.5)           # ZED Z (forward) = Gazebo X

        # Euler mapping (all in degrees)
        # Gazebo: roll=X-rot, pitch=Y-rot, yaw=Z-rot (heading)
        # ZED LEFT_HANDED_Y_UP: roll=fwd/back tilt, pitch=heading, yaw=side tilt
        gz_roll, gz_pitch, gz_yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        zp.roll = float(math.degrees(gz_pitch))     # Forward/back tilt
        zp.pitch = float(math.degrees(-gz_yaw))     # Heading (negated for left-handed)
        zp.yaw = float(math.degrees(gz_roll))       # Side tilt

        zp.x_vel = float(vel.x)
        zp.y_vel = float(vel.y)
        zp.z_vel = float(vel.z)
        # p3d doesn't give linear acceleration directly; set to 0
        zp.x_acc = 0.0
        zp.y_acc = 0.0
        zp.z_acc = 0.0
        
        zp.aruco_visible = self.aruco_visible
        zp.aruco_initialized = self.aruco_initialized
        
        if self.latest_aruco and self.aruco_visible:
            ao = self.latest_aruco.pose.orientation
            ar, ap, ay = euler_from_quaternion([ao.x, ao.y, ao.z, ao.w])
            zp.aruco_roll = float(math.degrees(ar))
            zp.aruco_pitch = float(math.degrees(ap))
            zp.aruco_yaw = float(math.degrees(ay))

        self.pub.publish(zp)

def main(args=None):
    rclpy.init(args=args)
    node = SimZedBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()