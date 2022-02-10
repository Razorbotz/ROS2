import rclpy
from rclpy.node import Node
import odrive
from odrive.enums import *
import time
from std_msgs.msg import Float32
import fibre.utils
import fibre.remote_object

class ExcavationNode(Node):
	def __init__(self):
				self.get_logger().info("init")
				super().__init__('excavation_node')
				self.get_logger().info("odriveObjects")
				self.calibrated = False
				self.findODriveObjects()
				self.get_logger().info("Found ODriveObjects")
				self.calibrate()
				self.setRequestedState()
				self.get_logger().info("elevationSubscription")
				self.excavationElevationSubscription = self.create_subscription(Float32, 'excavationElevation', self.excavationElevationCallback, 10)
				self.get_logger().info("augerSubscription")
				self.excavationAugerSubscription = self.create_subscription(Float32, 'excavationAuger', self.excavationAugerCallback, 10)
				self.get_logger().info("angleSubscription")
				self.excavationAngleSubscription = self.create_subscription(Float32, 'excavationAngle', self.excavationAngleCallback, 10)

	def findODriveObjects(self):
		self.get_logger().info("findOdriveObjects start")
		self.odrv0 = odrive.find_any(path = "usb", serial_number = "20773881304E")
		self.odrv1 = odrive.find_any(path = "usb", serial_number = "206736A1424D")

	##################################################################################
	#Note: This setup relies on the motors being in the following positions:         #
	#The first three motors are position controlled.  The motors of odrv0 are assumed#
	#to be the motors that control elevation of the assembly and are mirrored.       #
	#The first motor of odrv1 is assumed to control the angle of the assembly and is #
	#also position controlled.  The final motor is assumed to control the drill and  #
	#is currently configured to run sensorlessly because we do not have the final    #
	#encoder needed to run it with sensors.						 #
	##################################################################################
	def setRequestedState(self):
		self.get_logger().info("setRequestedState start")
		self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
		self.odrv0.axis0.controller.config.control_mode = 3
		self.odrv0.axis0.controller.input_pos = 0
		self.odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
		self.odrv0.axis1.controller.config.control_mode = 3
		self.odrv0.axis1.controller.input_pos = 0
		self.odrv1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
		self.odrv1.axis0.controller.config.control_mode = 2
		self.odrv1.axis0.controller.input_vel = 0
		self.odrv1.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
		self.odrv1.axis1.controller.config.control_mode = 3
		self.odrv1.axis1.controller.input_pos = 0
		self.calibrated = True

	def excavationElevationCallback(self, msg):
		if self.calibrated:
			self.odrv0.axis0.controller.input_pos = msg.data
			self.odrv0.axis1.controller.input_pos = msg.data
			self.get_logger().info("excavationElevationCallback: msg.data: " + str(msg.data))
			self.get_logger().info("input_pos: " + str(self.odrv0.axis0.controller.input_pos))
			self.get_logger().info("input_pos: " + str(self.odrv0.axis1.controller.input_pos))

	def excavationAugerCallback(self, msg):
		if self.calibrated:
			self.odrv1.axis1.controller.input_vel = msg.data * 20
			self.get_logger().info("excavationAugerCallback: msg.data: " + str(msg.data))
			self.get_logger().info("input_vel: " + str(self.odrv1.axis0.controller.input_vel))

	def excavationAngleCallback(self, msg):
		if self.calibrated:
			self.odrv1.axis0.controller.input_pos = msg.data
			self.get_logger().info("excavationAngleCallback: msg.data: " + str(msg.data))
			self.get_logger().info("input_pos: " + str(self.odrv1.axis1.controller.input_pos))

	def calibrate(self):
		self.get_logger().info("calibrate start")
		self.odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
		self.odrv1.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
		while self.odrv0.axis0.current_state != AXIS_STATE_IDLE | self.odrv1.axis0.current_state != AXIS_STATE_IDLE:
			time.sleep(0.1)
		self.odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
		self.odrv1.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
		while self.odrv0.axis1.current_state != AXIS_STATE_IDLE | self.odrv1.axis1.current_state != AXIS_STATE_IDLE:
			time.sleep(0.1)
		self.get_logger().info("Calibrated")


def main(args=None):
	rclpy.init(args=args)
	excavation_node = ExcavationNode()
	rclpy.spin(excavation_node)

	excavation_node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
    main()
