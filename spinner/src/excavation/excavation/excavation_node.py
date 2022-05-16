import rclpy
from rclpy.node import Node
import odrive
from odrive.enums import *
import time
from std_msgs.msg import Float32
from std_msgs.msg import String
from std_msgs.msg import Empty
import fibre.utils

# ToDo:
# error handling


class ExcavationNode(Node):
        def __init__(self):
                super().__init__('excavation_node')
                self.get_logger().info("odriveObjects")
                self.calibrated = False
                self.GO = False
                self.findODriveObjects()
                self.get_logger().info("Found ODriveObjects")
                self.calibrate()
                self.setRequestedState()
                self.get_logger().info("excavationDrumSubscription")
                self.excavationDrumSubscription = self.create_subscription(Float32, 'excavationDrum', self.excavationDrumCallback, 10)
                self.get_logger().info("excavationArmSubscription")
                self.excavationArmSubscription = self.create_subscription(Float32, 'excavationArm', self.excavationArmCallback, 10)
                self.stopSubscription = self.create_subscription(Empty, 'STOP', self.stopCallback, 1)
                self.goSubscription = self.create_subscription(Empty, 'GO', self.goCallback, 1)
                self.get_logger().info('HERE')

        def errorChecking(self):
                if self.odrv0.error != 0:
                        self.get_logger().info("ERROR: odrv0 has error state: ", self.odrv0.error)
                        self.calibrate0()
                if self.odrv1.error != 0:
                        self.get_logger().info("ERROR: odrv1 has error state: ", self.odrv1.error)
                        self.calibrate1()

        def calibrate0(self):
                self.get_logger().info("Calibrating odrv0 from error state")
                self.odrv0.clear_errors()
                self.odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
                while self.odrv0.axis0.current_state != AXIS_STATE_IDLE:
	                time.sleep(0.1)
                self.odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
                while self.odrv0.axis1.current_state != AXIS_STATE_IDLE:
                        time.sleep(0.1)
                self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

        def calibrate1(self):
                self.get_logger().info("Calibrating odrv1 from error state")
                self.odrv1.clear_errors()
                self.odrv1.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
                while self.odrv1.axis0.current_state != AXIS_STATE_IDLE:
                        time.sleep(0.1)
                self.odrv1.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
                while self.odrv1.axis1.current_state != AXIS_STATE_IDLE:
                        time.sleep(0.1)
                self.get_logger().info("Calibrated odrv1 from error state.  Current error state: ", self.odrv1.error)
                self.odrv1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

        # each odrv object contains two motors
        def findODriveObjects(self):
                self.get_logger().info("findOdriveObjects start")
                # odrv0 = drum
                self.odrv0 = odrive.find_any(path = "usb", serial_number = "2083367F424D")
                # odrv1 = arm
                self.odrv1 = odrive.find_any(path = "usb", serial_number = "20773881304E")
                # self.odrv0 = odrive.find_any(path = "usb", serial_number = "2083367F424D")

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
                #self.odrv0.axis1.controller.config.axis_to_mirror = 0
                #self.odrv0.axis1.controller.config.input_mode = 7
                #self.odrv0.axis0.controller.config.control_mode = 2
                #self.odrv0.axis0.controller.input_vel = 0
                #self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
                self.odrv1.axis1.controller.config.axis_to_mirror = 0
                self.odrv1.axis1.controller.config.input_mode = 7
                self.odrv1.axis0.controller.config.control_mode = 2
                self.odrv1.axis0.controller.input_vel = 0
                self.odrv1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
                self.odrv0.axis1.controller.config.control_mode = 2
                self.odrv0.axis1.controller.input_vel = 0
                self.odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
                self.calibrated = True

        def excavationDrumCallback(self, msg):
                if self.calibrated and self.GO:
                        newSpeed = msg.data * 144
                        oldSpeed = self.odrv0.axis1.controller.input_vel
                        if abs(newSpeed - oldSpeed) > 5:
                                if newSpeed > oldSpeed:
                                        self.odrv0.axis1.controller.input_vel = oldSpeed + 5
                                else:
                                        self.odrv0.axis1.controller.input_vel = oldSpeed - 5
                        else:
                                self.odrv0.axis1.controller.input_vel = msg.data * 144
                        self.get_logger().info("excavationDrumCallback: msg.data: " + str(msg.data))
                        self.get_logger().info("input_vel: " + str(self.odrv0.axis1.controller.input_vel))

        def excavationArmCallback(self, msg):
                if self.calibrated and self.GO:
                        newSpeed = msg.data * 20
                        oldSpeed = self.odrv1.axis1.controller.input_vel
                        if abs(newSpeed - oldSpeed) > 2:
                                if newSpeed > oldSpeed:
                                        self.odrv1.axis0.controller.input_vel = oldSpeed + 2
                                else:
                                        self.odrv1.axis0.controller.input_vel = oldSpeed - 2
                        else:
                                self.odrv1.axis0.controller.input_vel = msg.data * 20
                        self.get_logger().info("excavationArmCallback: msg.data: " + str(msg.data))
                        self.get_logger().info("input_vel: " + str(self.odrv1.axis0.controller.input_vel))

        def stopCallback(self, msg):
                self.GO = False
                self.odrv0.axis0.controller.input_vel = 0
                self.odrv1.axis1.controller.input_vel = 0

        def goCallback(self, msg):
                self.GO = True

        def calibrate(self):
                self.get_logger().info("calibrate start")
#                self.odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
#                while self.odrv0.axis0.current_state != AXIS_STATE_IDLE:
#                        time.sleep(0.1)
                self.odrv1.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
                while self.odrv1.axis0.current_state != AXIS_STATE_IDLE:
                        time.sleep(0.1)
                self.odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
                while self.odrv1.axis0.current_state != AXIS_STATE_IDLE:
                        time.sleep(0.1)
                self.odrv1.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
                while self.odrv1.axis1.current_state != AXIS_STATE_IDLE:
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
