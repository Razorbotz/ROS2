import rclpy
from rclpy.node import Node
import odrive
from odrive.enums import *
import time
from std_msgs.msg import Float32
from std_msgs.msg import String
from std_msgs.msg import Empty
import fibre.utils

## @file
# @brief Node controlling excavation motors
# 
# This node listens for several topics, and sends the information
# to the ODrive excavation motors.  The topics this node subscribes to
# are as follows:
#
# \li \b excavationDrum
# \li \b excavationArm
# \li \b STOP
# \li \b GO
# 
# To read more about the nodes that publish these topics
# \see logic_node.cpp
# \see communication_node.cpp

class ExcavationNode(Node):
        def __init__(self):
                """! Initializes the program, sets subscriptions"""
                super().__init__('excavation_node')
                self.get_logger().info("odriveObjects")
                self.calibrated = False
                self.GO = False
                self.errorState = False
                self.restarted = False
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

        def errorChecking(self, odrv, num):
                """! Checks motors for errors and corrects any errors found
                
                @param odr ODrive object being checked for errors
                @param num Number of the ODrive object, used for printing to the screen
                """
                if odrv.error != 0:
                        self.get_logger().info("SYSTEM LEVEL ERROR: odrv%d has error state: %d", num, odrv.error)
                        self.errorState = True
                        self.calibrate(self, odrv, num)
                elif odrv.axis0.error != 0:
                        self.get_logger().info("axis0 level ERROR: odrv%d has error state: %d", num, odrv.axis0.error)
                        self.errorState = True
                        self.calibrate(self, odrv, num)
                elif odrv.axis0.motor.error != 0:
                        self.get_logger().info("axis0.motor ERROR: odrv%d has error state: %d", num, odrv.axis0.error)
                        self.errorState = True
                        self.calibrate(self, odrv, num)
                elif odrv.axis1.error != 0:
                        self.get_logger().info("axis1 ERROR: odrv%d has error state: %d", num, odrv.axis1.error)
                        self.errorState = True
                        self.calibrate(self, odrv, num)
                elif odrv.axis1.motor.error != 0:
                        self.get_logger().info("axis1.motor ERROR: odrv%d has error state: %d", num, odrv.axis0.error)
                        self.errorState = True
                        self.calibrate(self, odrv, num)
        
        def calibrate(self, odrv, num):
                """! Calibrates the ODrive object specified
                
                @param odr ODrive object being calibrated
                @param num Number of the ODrive object, used for printing to the screen
                """
                self.get_logger().info("Calibrating odrv%d from error state", num)
                odrv.clear_errors()
                odrv.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
                while odrv.axis0.current_state != AXIS_STATE_IDLE:
                        time.sleep(0.1)
                odrv.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
                while odrv.axis1.current_state != AXIS_STATE_IDLE:
                        time.sleep(0.1)
                odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
                self.errorState = False


#        def calibrate0(self):
#                self.get_logger().info("Calibrating odrv0 from error state")
#                self.odrv0.clear_errors()
#                self.odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
#                while self.odrv0.axis0.current_state != AXIS_STATE_IDLE:
#                        time.sleep(0.1)
#                self.odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
#                while self.odrv0.axis1.current_state != AXIS_STATE_IDLE:
#                        time.sleep(0.1)
#                self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

#        def calibrate1(self):
#                self.get_logger().info("Calibrating odrv1 from error state")
#                self.odrv1.clear_errors()
#                self.odrv1.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
#                while self.odrv1.axis0.current_state != AXIS_STATE_IDLE:
#                        time.sleep(0.1)
#                self.odrv1.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
#                while self.odrv1.axis1.current_state != AXIS_STATE_IDLE:
#                        time.sleep(0.1)
#                self.get_logger().info("Calibrated odrv1 from error state.  Current error state: ", self.odrv1.error)
#                self.odrv1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

        # each odrv object contains two motors
        def findODriveObjects(self):
                """! Function to find the ODrive boards

                The ODrive boards must be set to the following: odrv0 = 2083367F424D and
                odrv1 = 20773881304E.  If the boards are inverted, the callbacks will be
                incorrect for the actions.                
                """
                self.get_logger().info("findOdriveObjects start")
                # odrv0 = drum
                self.odrv0 = odrive.find_any(path = "usb", serial_number = "2083367F424D")
                # odrv1 = arm
                self.odrv1 = odrive.find_any(path = "usb", serial_number = "20773881304E")

        ##################################################################################
        # The first two motors are set up to run the drum while the second is set up to  #
        # control the angle of the arm.  On both boards, axis0 is the primary axis while #
        # axis1 mirrors it.  Currently both boards are controlled with velocity control  #
        # but odrv1 might be better served doing position control to better allow it to  #
        # be automated.                                                                  #
        ##################################################################################
        def setRequestedState(self):
                """! Sets motors in correct states
                
                The motors must be calibrated and then set to the correct states to run 
                correctly.  For axis1 on both odrv0 and odrv1, the motor is configured to 
                mirror axis 0, then put into input mode 7, which is mirrored input.  This
                allows the motor to exactly mirror the output of axis 0 and prevents the
                motors from opposing each other.  For both ODrive objects, axis 0 is set 
                to control mode 2, velocity control, and initialized with a velocity of 0.
                They are then set to AXIS_STATE_CLOSED_LOOP_CONTROL, which allows the motors
                to run.
                """
                self.get_logger().info("setRequestedState start")
                self.odrv0.axis1.controller.config.axis_to_mirror = 0
                self.odrv0.axis1.controller.config.input_mode = 7
                self.odrv0.axis0.controller.config.control_mode = 2
                self.odrv0.axis0.controller.input_vel = 0
                self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
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
                """! Excavation drum callback
                
                The callback function triggered when the node receives a message 
                of topic excavationDrum.  The ODrive motor has a max RPM of 8640
                or 144 rev/sec.  The input_vel command takes a float as input and
                sets the speed to that number of rev/sec, thus is upper bounded by
                the motor maximum of 144.  Additionally, the motor has a tendency
                to throw errors if the new input_vel greatly differs from the
                previous command (ie > ~10), so there is logic to ensure that any
                greatly differing command is handled gracefully.  The expected
                input ranges from -1.0 to 1.0.
                @param msg The message containing the speed data, ranges from -1.0 to 1.0
                """
                if self.calibrated and self.GO and not self.errorState:
                        newSpeed = msg.data * 144
                        oldSpeed = self.odrv0.axis0.controller.input_vel
                        if abs(newSpeed - oldSpeed) > 5:
                                if newSpeed > oldSpeed:
                                        self.odrv0.axis0.controller.input_vel = oldSpeed + 5
                                else:
                                        self.odrv0.axis0.controller.input_vel = oldSpeed - 5
                        else:
                                self.odrv0.axis0.controller.input_vel = msg.data * 144
                        self.get_logger().info("excavationDrumCallback: msg.data: " + str(msg.data))
                        self.get_logger().info("input_vel: " + str(self.odrv0.axis0.controller.input_vel))

        def excavationArmCallback(self, msg):
                """! Excavation arm callback
                
                The callback function triggered when the node receives a message 
                of topic excavationArm.  The ODrive motor has a max RPM of 8640
                or 144 rev/sec.  The input_vel command takes a float as input and
                sets the speed to that number of rev/sec, thus is upper bounded by
                the motor maximum of 144.  Additionally, the motor has a tendency
                to throw errors if the new input_vel greatly differs from the
                previous command (ie > ~10), so there is logic to ensure that any
                greatly differing command is handled gracefully.  The expected 
                input ranges from -1.0 to 1.0.
                @param msg The message containing the speed data, ranges from -1.0 to 1.0
                """
                if self.calibrated and self.GO and not self.errorState:
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
                """! Stop callback function
                
                The function triggered when the node receives a message with the name
                STOP, which is sent when the user releases the trigger.  The GO 
                variable is set to False and the input_vel of both boards are set to
                zero.
                """
                self.GO = False
                self.odrv0.axis0.controller.input_vel = 0
                self.odrv1.axis1.controller.input_vel = 0

        def goCallback(self, msg):
                """! Go callback function
                
                The function triggered when the node receives a message with the name
                GO, which is sent when the user presses the trigger.  The GO variable 
                is set to True, which allows the motors to be set to non-zero 
                velocities.
                """
                self.GO = True

        def calibrate(self):
                """! Initial motor calibration
                
                Goes through all motors and sets them to the full calibration sequence, then
                waits until the motor is finished with the calibration before moving on to
                the next motor.  If two motors are calibrating simultaneously on the same
                board, the current draw will shutdown the Jetson.
                """
                self.get_logger().info("calibrate start")
                self.odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
                while self.odrv0.axis0.current_state != AXIS_STATE_IDLE:
                        time.sleep(0.1)
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
