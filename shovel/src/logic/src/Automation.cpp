#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>

#include "logic/Automation.hpp"
#include "logic/AutomationTypes.hpp"

/** @file
 *
 * @brief Defines functions used in Automation.hpp and AutomationTypes.hpp
 * 
 * These functions are used to calculate wheel speed and three dimensional positioning of the bot.
 * */
 
 
/** @brief Sets publisher node for left and right wheel motor speed
 * 
 * This function sets the driveLeftSpeedPublisher and driveRightSpeedPublisher.
 * @param rclcpp::Node::SharedPtr node
 * @return void
 * */
void Automation::setNode(rclcpp::Node::SharedPtr node){
    this->node=node;
    driveLeftSpeedPublisher= this->node->create_publisher<std_msgs::msg::Float32>("drive_left_speed",1);
    driveRightSpeedPublisher= this->node->create_publisher<std_msgs::msg::Float32>("drive_right_speed",1);
    armSpeedPublisher= this->node->create_publisher<std_msgs::msg::Float32>("arm_speed",1);
    bucketSpeedPublisher= this->node->create_publisher<std_msgs::msg::Float32>("bucket_speed",1);
    goPublisher = this->node->create_publisher<std_msgs::msg::Empty>("GO", 1);
    stopPublisher = this->node->create_publisher<std_msgs::msg::Empty>("STOP",1);
    autonomyOutPublisher = this->node->create_publisher<messages::msg::AutonomyOut>("autonomy_out",1);
    talon14PositionPublisher = this->node->create_publisher<std_msgs::msg::Int32>("talon_14_position",1);
    talon15PositionPublisher = this->node->create_publisher<std_msgs::msg::Int32>("talon_15_position",1);
    talon16PositionPublisher = this->node->create_publisher<std_msgs::msg::Int32>("talon_16_position",1);
    talon17PositionPublisher = this->node->create_publisher<std_msgs::msg::Int32>("talon_17_position",1);
}


/** @brief Sets axis orientations and calls EulerAngles.
 *
 * Sets position for x, y, z axes and w (homogenous vertex), and then 
 * calls toEulerAngles to convert the positions to Euler angles to orient 
 * the bot in 3d space.
 * @param Position
 * @return void
 * */
void Automation::setPosition(Position position){
    this->position = position;
    this->orientationQuaternion.x=position.ox;
    this->orientationQuaternion.y=position.oy;
    this->orientationQuaternion.z=position.oz;
    this->orientationQuaternion.w=position.ow;
    this->orientation=toEulerAngles(this->orientationQuaternion);
    if(prevX == 0.0 && prevY == 0.0 && prevZ == 0.0){
        prevX = position.x;
        prevY = position.y;
        prevZ = position.z;
    }
    else{
        deltaX = prevX - position.x;
        deltaY = prevY - position.y;
        deltaZ = prevZ - position.z;
        prevX = position.x;
        prevY = position.y;
        prevZ = position.z;
    }
    //RCLCPP_INFO(this->node->get_logger(), "Roll: %f, Pitch: %f, Yaw: %f", this->orientation.roll, this->orientation.pitch, this->orientation.yaw);
}


/** @brief Assigns/publishes left/right motorspeeds
 * 
 * This function assigns the speed of the left and right motors and then publishes them.
 * @param left, right
 * @return void
 * */
void Automation::changeSpeed(float left, float right){
    if(currentLeftSpeed==left && currentRightSpeed==right) return;
    currentLeftSpeed=left;
    currentRightSpeed=right;
    std_msgs::msg::Float32 speedLeft;
    std_msgs::msg::Float32 speedRight;
    speedLeft.data=left;
    speedRight.data=right;
    driveLeftSpeedPublisher->publish(speedLeft);
    driveRightSpeedPublisher->publish(speedRight);
}


void Automation::setBucketSpeed(float speed){
    std_msgs::msg::Float32 Speed;
    Speed.data = speed;
    bucketSpeedPublisher->publish(Speed);
}


void Automation::setArmSpeed(float speed){
    std_msgs::msg::Float32 Speed;
    Speed.data = speed;
    armSpeedPublisher->publish(Speed);
}


/** @brief Converts raw x,y,z-axis data to Euler angles to orient the bot in 3d space.
 *
 * This function takes in the x,y,z-axis coordinates established in setPosition and 
 * converts them to Euler angles. Euler angles describe the orientation of a body to a fixed coordinate 
 * system using each axes angle from its respective origin to the coordinate itself. 
 * This measurment of a coordinates angle from the origin in each individual axii indicates the 
 * rotation of the robot in the 3d reference system i.e, pitch, roll, and yaw.
 * @param q
 * @return angles
 * */
EulerAngles Automation::toEulerAngles(Quaternion q) {
    EulerAngles angles;

     // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp) * 180 / M_PI;

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp) * 180 / M_PI; // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp) * 180 / M_PI;

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp) * 180 / M_PI;

    return angles;
}


/*
Function to publish a ROS2 message of type empty with the topic name
of GO, which will enable the motors to turn on.
*/
void Automation::setGo(){
   std_msgs::msg::Empty empty;
   goPublisher->publish(empty);
}


/*
Function to publish a ROS2 message of type empty with the topic name
STOP, which will disable the motors.
*/
void Automation::setStop(){
    std_msgs::msg::Empty empty;
    stopPublisher->publish(empty);
}

void Automation::setIdle(){
    robotState = ROBOT_IDLE;
}

/*
Function to set Linear1 to the received Linear motor values.
*/
void Automation::setLinear1(const messages::msg::LinearOut::SharedPtr linearOut){
    this->linear1.speed = linearOut->speed;
    this->linear1.atMax = linearOut->at_max;
    this->linear1.atMin = linearOut->at_min;
    this->linear1.error = linearOut->error;
    this->linear1.distance = linearOut->distance;
    this->linear1.stroke = linearOut->stroke;
    this->linear1.extensionSpeed = linearOut->extension_speed;
    this->linear1.timeToExtend = linearOut->time_to_extend;
    this->linear1.potentiometer = linearOut->potentiometer;
    this->linear1.sensorless = linearOut->sensorless;
}


/*
Function to set Linear2 to the received Linear motor values.
*/
void Automation::setLinear2(const messages::msg::LinearOut::SharedPtr linearOut){
    this->linear2.speed = linearOut->speed;
    this->linear2.atMax = linearOut->at_max;
    this->linear2.atMin = linearOut->at_min;
    this->linear2.error = linearOut->error;
    this->linear2.distance = linearOut->distance;
    this->linear2.stroke = linearOut->stroke;
    this->linear2.extensionSpeed = linearOut->extension_speed;
    this->linear2.timeToExtend = linearOut->time_to_extend;
    this->linear2.potentiometer = linearOut->potentiometer;
    this->linear2.sensorless = linearOut->sensorless;
}


/*
Function to set Linear3 to the received Linear motor values.
*/
void Automation::setLinear3(const messages::msg::LinearOut::SharedPtr linearOut){
    this->linear3.speed = linearOut->speed;
    this->linear3.atMax = linearOut->at_max;
    this->linear3.atMin = linearOut->at_min;
    this->linear3.error = linearOut->error;
    this->linear3.distance = linearOut->distance;
    this->linear3.stroke = linearOut->stroke;
    this->linear3.extensionSpeed = linearOut->extension_speed;
    this->linear3.timeToExtend = linearOut->time_to_extend;
    this->linear3.potentiometer = linearOut->potentiometer;
    this->linear3.sensorless = linearOut->sensorless;
}


/*
Function to set Linear4 to the received Linear motor values.
*/
void Automation::setLinear4(const messages::msg::LinearOut::SharedPtr linearOut){
    this->linear4.speed = linearOut->speed;
    this->linear4.atMax = linearOut->at_max;
    this->linear4.atMin = linearOut->at_min;
    this->linear4.error = linearOut->error;
    this->linear4.distance = linearOut->distance;
    this->linear4.stroke = linearOut->stroke;
    this->linear4.extensionSpeed = linearOut->extension_speed;
    this->linear4.timeToExtend = linearOut->time_to_extend;
    this->linear4.potentiometer = linearOut->potentiometer;
    this->linear4.sensorless = linearOut->sensorless;
}


/*
Function to set Talon1 to the received Talon motor values.
*/
void Automation::setTalon1(const messages::msg::TalonOut::SharedPtr talonOut){
    this->talon1.busVoltage = talonOut->bus_voltage;
    this->talon1.outputCurrent = talonOut->output_current;
    this->talon1.outputVoltage = talonOut->output_voltage;
    this->talon1.outputPercentage = talonOut->output_percent;
    if(talonOut->output_current > this->talon1.maxCurrent){
        this->talon1.maxCurrent = talonOut->output_current;
    }
    this->talon1.sensorValue = talonOut->sensor_position;
}


/*
Function to set Talon2 to the received Talon motor values.
*/
void Automation::setTalon2(const messages::msg::TalonOut::SharedPtr talonOut){
    this->talon2.busVoltage = talonOut->bus_voltage;
    this->talon2.outputCurrent = talonOut->output_current;
    this->talon2.outputVoltage = talonOut->output_voltage;
    this->talon2.outputPercentage = talonOut->output_percent;
    if(talonOut->output_current > this->talon2.maxCurrent){
        this->talon2.maxCurrent = talonOut->output_current;
    }
    this->talon2.sensorValue = talonOut->sensor_position;
}


/*
Function to set Talon3 to the received Talon motor values.
*/
void Automation::setTalon3(const messages::msg::TalonOut::SharedPtr talonOut){
    this->talon3.busVoltage = talonOut->bus_voltage;
    this->talon3.outputCurrent = talonOut->output_current;
    this->talon3.outputVoltage = talonOut->output_voltage;
    this->talon3.outputPercentage = talonOut->output_percent;
    if(talonOut->output_current > this->talon3.maxCurrent){
        this->talon3.maxCurrent = talonOut->output_current;
    }
    this->talon3.sensorValue = talonOut->sensor_position;
}


/*
Function to set Talon4 to the received Talon motor values.
*/
void Automation::setTalon4(const messages::msg::TalonOut::SharedPtr talonOut){
    this->talon4.busVoltage = talonOut->bus_voltage;
    this->talon4.outputCurrent = talonOut->output_current;
    this->talon4.outputVoltage = talonOut->output_voltage;
    this->talon4.outputPercentage = talonOut->output_percent;
    if(talonOut->output_current > this->talon4.maxCurrent){
        this->talon4.maxCurrent = talonOut->output_current;
    }
    this->talon4.sensorValue = talonOut->sensor_position;
}


/*
Function to set Falcon1 to the received Falcon motor values.
*/
void Automation::setFalcon1(const messages::msg::FalconOut::SharedPtr falconOut){
    this->falcon1.busVoltage = falconOut->bus_voltage;
    this->falcon1.outputCurrent = falconOut->output_current;
    this->falcon1.outputVoltage = falconOut->output_voltage;
    this->falcon1.outputPercentage = falconOut->output_percent;
}


/*
Function to set Falcon2 to the received Falcon motor values.
*/
void Automation::setFalcon2(const messages::msg::FalconOut::SharedPtr falconOut){
    this->falcon2.busVoltage = falconOut->bus_voltage;
    this->falcon2.outputCurrent = falconOut->output_current;
    this->falcon2.outputVoltage = falconOut->output_voltage;
    this->falcon2.outputPercentage = falconOut->output_percent;
}


/*
Function to set Falcon3 to the received Falcon motor values.
*/
void Automation::setFalcon3(const messages::msg::FalconOut::SharedPtr falconOut){
    this->falcon3.busVoltage = falconOut->bus_voltage;
    this->falcon3.outputCurrent = falconOut->output_current;
    this->falcon3.outputVoltage = falconOut->output_voltage;
    this->falcon3.outputPercentage = falconOut->output_percent;
}


/*
Function to set Falcon4 to the received Falcon motor values.
*/
void Automation::setFalcon4(const messages::msg::FalconOut::SharedPtr falconOut){
    this->falcon4.busVoltage = falconOut->bus_voltage;
    this->falcon4.outputCurrent = falconOut->output_current;
    this->falcon4.outputVoltage = falconOut->output_voltage;
    this->falcon4.outputPercentage = falconOut->output_percent;
}


/*
* Function that checks if the linear actuator has a PotentiometerError
* or an ActuatorNotMovingError.
* @param linear - Linear actuator object
* @return True if PotentiometerError or ActuatorNotMovingError
*/
bool Automation::checkErrors(Linear linear){
    if(linear.error == "PotentiometerError" || linear.error == "ActuatorNotMovingError"){
        return true;
    }
    else{
        return false;
    }
}


/*
Function that will set the angle that the robot should be facing
to travel to the destination. This should be between the values of
[-180, 180].
*/
void Automation::setDestAngle(float degrees){
    if(degrees < -180){
        this->destAngle = degrees + 360;
    }
    else if (degrees > 180){
        this->destAngle = degrees - 360;
    }
    else{
        this->destAngle = degrees;
    }
}

float Automation::getAngle(){
    float x = this->destX - this->position.x;
    float y = this->destZ - this->position.z;
    float angle = std::atan2(y, x) * 180 / M_PI;
    if(angle < -90){
        angle += 180;
    }
    if(angle > 90){
        angle -= 180;
    }
    if(x > 0){
	    angle = 90 - angle;
    }
    else{
    	angle = -90 - angle;
    }
    RCLCPP_INFO(this->node->get_logger(), "Dest x: %f, Position.x: %f, x: %f", this->destX, this->position.x, x);
    RCLCPP_INFO(this->node->get_logger(), "Dest z: %f, Position.z: %f, y: %f", this->destZ, this->position.z, y);
    RCLCPP_INFO(this->node->get_logger(), "Angle: %f", angle);
    if(angle > 180){
        angle -= 360;
    }
    return angle;
}


float Automation::getAngle2(){
    // The robot rotates around a point that's offset from the camera
    // This takes that offset into account
    float offset_from_center = 0.35;
    float center_x = this->position.x - offset_from_center * std::cos(this->position.pitch);
    float center_y = this->position.z - offset_from_center * std::sin(this->position.pitch);

    float x = this->destX - center_x;
    float y = this->destZ - center_y;
    float angle = std::atan2(y, x) * 180 / M_PI;
    if(angle < -90){
        angle += 180;
    }
    if(angle > 90){
        angle -= 180;
    }
    if(x > 0){
	    angle = 90 - angle;
    }
    else{
    	angle = -90 - angle;
    }
    RCLCPP_INFO(this->node->get_logger(), "Dest x: %f, Position.x: %f, x: %f", this->destX, this->position.x, x);
    RCLCPP_INFO(this->node->get_logger(), "Dest z: %f, Position.z: %f, y: %f", this->destZ, this->position.z, y);
    RCLCPP_INFO(this->node->get_logger(), "Angle: %f", angle);
    if(angle > 180){
        angle -= 360;
    }
    // Dot product between heading vector and vector to target
    // Testing to determine when the robot should back up vs turn around
    double heading_x = std::cos(this->position.pitch);
    double heading_y = std::sin(this->position.pitch);
    double dot_product = x * heading_x + y * heading_y;

    if (dot_product < 0) {
        RCLCPP_INFO(this->node->get_logger(), "Target is behind the robot. Consider reversing.");
    }
    return angle;
}


/*

*/
float Automation::getAngleDiff(){
    return Automation::getAngleDiff(position.pitch);
}


/*
If result < 0, turn left. Else, turn right.
*/
float Automation::getAngleDiff(float degrees){
    float result = this->destAngle - degrees;
    if(result < -180){
        result += 360;
    }
    if(result > 180){
        result -= 360;
    }
    return result;
}


/*
Function to check whether the current angle of the robot is within an
acceptable range. The logic when the values are at the edge of the range
gets a little bit fuzzy. 

The expected use of this function is that the function will detect if the
robot has gone outside of the expected angle.
*/
// place up near your helpers:
static float normalizeAngle(float x) {
    // bring any θ into (–180, +180]
    while (x >  180.0f) x -= 360.0f;
    while (x <= -180.0f) x += 360.0f;
    return x;
}


/*
Function to get the angle that the robot should be facing to
travel to the destination. This should be between the values of
[-180, 180]. 
The atan2 function assumes that the zero angle location is to the right,
while the arena has the zero position to the bottom, which is why the 
angle has 90 added to it.
*/
bool Automation::checkAngle(bool reverse){
    // 1) compute signed error in [–180, +180]

    float error = 0.0;
    if(reverse)
        error = normalizeAngle(this->destAngle - position.pitch + 180);
    else
        error = normalizeAngle(this->destAngle - position.pitch);

    // 2) magnitude
    float absErr = std::abs(error);

    // 3) pick spin speed based on how far off we are
    float speed;
    if      (absErr > 20.0f) speed = 0.3f;
    else if (absErr > 10.0f) speed = 0.2f;
    else if (absErr >  5.0f) speed = 0.15f;
    else                      speed = 0.1f;

    // 4) if we’re still outside our dead-band, spin; otherwise we’re aligned
    if (absErr > angleThresh) {
        if (error < 0.0f) {
        // negative error ⇒ need to turn “left”
        changeSpeed(speed, -speed);
        }
        else {
            // positive error ⇒ turn “right”
            changeSpeed(-speed, speed);
        }
        
        return false;
    }

    // 5) we’re within the dead-band—stop turning
    changeSpeed(0.0f, 0.0f);
    return true;
}


// Testing some logic that will make more gradual turns to the robot
bool Automation::checkAngle2(){
    // 1) compute signed error in [–180, +180]
    float error = normalizeAngle(this->destAngle - position.pitch);
    float absErr = std::abs(error);

    // 2) check if we're aligned
    if (absErr <= angleThresh) {
        // Aligned
        return true;
    }

    // 3) scale a turn factor based on angle error (max 1.0)
    float turnStrength;
    if      (absErr > 30.0f) turnStrength = 1.0f;
    else if (absErr > 15.0f) turnStrength = 0.7f;
    else if (absErr > 5.0f)  turnStrength = 0.4f;
    else                    turnStrength = 0.2f;

    // 4) direction of the turn
    float direction = (error < 0.0f) ? -1.0f : 1.0f;

    // 5) set wheel speeds for curved motion
    float baseSpeed = 0.15f;  // Forward motion
    float leftSpeed  = baseSpeed - direction * turnStrength * baseSpeed;
    float rightSpeed = baseSpeed + direction * turnStrength * baseSpeed;

    // Clamp speeds if needed
    leftSpeed  = std::clamp(leftSpeed,  -0.3f, 0.3f);
    rightSpeed = std::clamp(rightSpeed, -0.3f, 0.3f);

    changeSpeed(leftSpeed, rightSpeed);
    return false;
}



/*
Function that checks the distance between the robot's current position
and the destination. The greater the distance returned, the further out
the robot is from the target location. 
*/
int Automation::checkDistance(float thresh){
    float dist = sqrt(pow(position.z - this->destZ, 2) + pow(position.x - this->destX, 2));
    RCLCPP_INFO(this->node->get_logger(), "Distance: %f", dist); 
    // Check to see if the distance is increasing. If the distance
    // is decreasing as expected, the diff should be greater than zero.
    float diff = prevDist - dist;
    prevDist = dist;
    if(diff < 0)
        return -1;
    
    if(dist < thresh)
        return 0;
    if(dist < 2*thresh)
        return 1;
    if(dist < 3*thresh)
        return 2;
    return 3;
    
}


// Testing merge of checkAngle and checkDistance
bool Automation::driveToTarget(float closeThresh) {
    float camTheta = this->position.pitch * (M_PI / 180.0f);

    // Compute robot rotation center (camera is offset forward)
    float centerX = this->position.x - 0.35 * std::cos(camTheta);
    float centerZ = this->position.z - 0.35 * std::sin(camTheta);

    // Vector from center to target
    float dx = this->destX - centerX;
    float dz = this->destZ - centerZ;
    float dist = std::sqrt(dx * dx + dz * dz);

    float diff = prevDist - dist;
    prevDist = dist;

    if (diff < 0) {
        // We're getting farther — maybe stuck or overshooting
        setDestAngle(getAngle());
        return false;
    }

    // Arrival check
    if (dist < closeThresh) {
        changeSpeed(0.0f, 0.0f);
        return true;
    }

    // Heading vector from camera
    float hx = std::cos(camTheta);
    float hz = std::sin(camTheta);

    // Dot product tells if target is in front or behind
    float dot = dx * hx + dz * hz;
    bool reverse = (dot < 0);

    // Desired angle from center to target
    float desiredAngle = std::atan2(dz, dx) * 180 / M_PI;

    // If reversing, rotate 180° to face away from target
    if (reverse) {
        desiredAngle = normalizeAngle(desiredAngle + 180);
    }

    float angleError = normalizeAngle(desiredAngle - this->position.pitch);  // Still in degrees
    float absErr = std::abs(angleError);

    // Base forward/reverse speed based on distance
    float baseSpeed;
    if      (dist < 2 * closeThresh) baseSpeed = 0.1f;
    else if (dist < 3 * closeThresh) baseSpeed = 0.15f;
    else                             baseSpeed = 0.25f;

    // Reverse speed is negative
    if (reverse) baseSpeed = -baseSpeed;

    float leftSpeed = baseSpeed;
    float rightSpeed = baseSpeed;

    // Thresholds
    const float spinThreshold = 30.0f;
    const float angleThresh   = 5.0f;

    if (absErr > spinThreshold) {
        // Spin in place
        float spinSpeed = (absErr > 60.0f) ? 0.3f : 0.2f;
        if (angleError < 0.0f) {
            leftSpeed = -spinSpeed;
            rightSpeed = spinSpeed;
        } else {
            leftSpeed = spinSpeed;
            rightSpeed = -spinSpeed;
        }
    } else if (absErr > angleThresh) {
        // Gradual turning while moving
        float turnStrength;
        if      (absErr > 15.0f) turnStrength = 0.7f;
        else if (absErr > 5.0f)  turnStrength = 0.4f;
        else                    turnStrength = 0.2f;

        float direction = (angleError < 0.0f) ? -1.0f : 1.0f;

        leftSpeed  = baseSpeed - direction * turnStrength * std::abs(baseSpeed);
        rightSpeed = baseSpeed + direction * turnStrength * std::abs(baseSpeed);

        // Clamp speeds
        leftSpeed  = std::clamp(leftSpeed,  -0.3f, 0.3f);
        rightSpeed = std::clamp(rightSpeed, -0.3f, 0.3f);
    }

    changeSpeed(leftSpeed, rightSpeed);
    return false;
}


/*
Function to set the X coordinate of the destination. 
TODO: Double check that this is correct
*/
void Automation::setDestX(float meters){
    this->search.destY = int(std::ceil(meters * 10));
    this->destX = meters;
}


void Automation::setDestZ(float meters){
    this->search.destX = this->search.Row - int(std::ceil(meters * 10));
    this->destZ = meters;
}


void Automation::publishAutonomyOut(std::string robotStateString, std::string excavationStateString, std::string errorStateString, std::string diagnosticsStateString, std::string tiltStateString, std::string dumpStateString, std::string bucketState, std::string armsState){
    messages::msg::AutonomyOut aOut;
    aOut.robot_state = robotStateString;
    aOut.excavation_state = excavationStateString;
    aOut.error_state = errorStateString;
    aOut.diagnostics_state = diagnosticsStateString;
    aOut.tilt_state = tiltStateString;
    aOut.dump_state = dumpStateString;
    aOut.bucket_state = bucketState;
    aOut.arms_state = armsState;
    aOut.dest_x = this->destX;
    aOut.dest_z = this->destZ;
    autonomyOutPublisher->publish(aOut);
}


/*
Function to set the start time at some point. This value can be 
used to calculate how much time something has been running, which
can be used to replace a while loop with an if statement instead.
*/
void Automation::setStartTime(std::chrono::time_point<std::chrono::high_resolution_clock> StartTime){
    this->startTime = StartTime;
}


std::chrono::time_point<std::chrono::high_resolution_clock> Automation::getStartTime(){
    return this->startTime;
}


/*
Sets the value of the runSensorlessly variable. If this is true, the
robot will continue to run if any potentiometers on the excavation
system fails.
*/
void Automation::setRunSensorlessly(bool value){
    this->runSensorlessly = value;
}


/*
Function to set the start position of the robot given the current
position in meters. These values are transformed into decimeters,
then transformed to map it into the 2D array.
*/
void Automation::setStartPosition(float x, float z){
    this->search.startX = this->search.Row - int(std::ceil(z * 10));
    this->search.startY = int(std::ceil(x  * 10));
}


/*
Function to set the destination position of the robot given the current
position in meters. These values are transformed into decimeters,
then transformed to map it into the 2D array.
*/
void Automation::setDestPosition(float x, float z){
    this->search.destX = this->search.Row - int(std::ceil(z * 10));
    this->search.destY = int(std::ceil(x * 10));
    this->destX = x;
    this->destZ = z;
}


/*
TODO: Rename this to be more descriptive
*/
bool Automation::getPosition(){
    if(this->currentPath.empty()){
        return false;
    }
    else{
        std::pair<int, int> current = this->currentPath.top();
        this->currentPath.pop();
        this->destZ = (this->search.Row - current.first) / 10.0;
        this->destX = current.second / 10.0;
        return true;
    }
}


void Automation::addPointToStack(float x, float z){
    int X = this->search.Row - int(std::ceil(z * 10));
    int Y = int(std::ceil(x  * 10));
    this->search.addPointToStack(X, Y);
}


/*
Function to run the A-Star algorithm. The includeHoles parameter
tells the search algorithm whether or not holes should be included
in the path or if they must be avoided.
*/
void Automation::aStar(bool includeHoles){
    this->currentPath = this->search.getSimplifiedPath(this->search.aStar(includeHoles)) ;
    this->currentPath.pop();
}


void Automation::aStarStack(bool includeHoles, bool simplify){
    this->currentPath = this->search.aStar(this->search.points, includeHoles, simplify);
    this->currentPath.pop();
}


// TODO: Look into changing this from thresh parameter over to
// (int value, int thresh) to allow for clearer use of this function
/*
Function to check the current position of the arm relative
to the target position of the arm based on the value of the
potentiometer. The return values are described below.

Potentiometer <= target - thresh, return 0

target - thresh < Potentiometer < target + thresh, return 1

target + thresh < Potentiometer, return 2
@param thresh - Value of threshold to check if potentiometer is within
+/- the thresh value of target
*/
int Automation::checkArmPosition(int thresh){
    if(linear1.potentiometer <= target1 - thresh){
        return 0;
    }
    if(linear1.potentiometer > (target1 - thresh) && linear1.potentiometer < (target1 + thresh)){
        return 1;
    }
    if(linear1.potentiometer >= (target1 + thresh)){
        return 2;
    }
    return -1;
}


/*
Function to check the current position of the bucket relative
to the target position of the arm based on the value of the
potentiometer. The return values are described below.

Potentiometer <= target - thresh, return 0

target - thresh < Potentiometer < target + thresh, return 1

target + thresh < Potentiometer, return 2
@param thresh - Value of threshold to check if potentiometer is within
+/- the thresh value of target
*/
int Automation::checkBucketPosition(int thresh){
    if(linear3.potentiometer <= target3 - thresh){
        return 0;
    }
    if(linear3.potentiometer > (target3 - thresh) && linear3.potentiometer < (target3 + thresh)){
        return 1;
    }
    if(linear3.potentiometer >= (target3 + thresh)){
        return 2;
    }
    return -1;
}


/*
This function is designed to move the arms to a specific point. It
currently uses a while loop to run for a specified amount of time,
which is bad practice because it will block other thread executions.
This should probably be rewritten to use an if statement instead.
*/
void Automation::setArmPosition(int potent){
    target1 = potent;
    std_msgs::msg::Int32 position;
    position.data = potent;
    talon14PositionPublisher->publish(position);
    talon15PositionPublisher->publish(position);
}


/*
This function is designed to move the bucket to a specific point. It
currently uses a while loop to run for a specified amount of time,
which is bad practice because it will block other thread executions.
This should probably be rewritten to use an if statement instead.
*/
void Automation::setBucketPosition(int potent){
    if(potent > 700)
        potent = 700;
    target3 = potent;
    std_msgs::msg::Int32 position;
    position.data = potent;
    talon16PositionPublisher->publish(position);
    talon17PositionPublisher->publish(position);
}


void Automation::setMap(std::string mapUsed){
    if(mapUsed == "NASA"){
        this->search.setRowCol(NASA.height, NASA.width);
    }
    if(mapUsed == "UCF_1"){
        this->search.setRowCol(UCF_1.height, UCF_1.width);
    }
    if(mapUsed == "UCF_2"){
        this->search.setRowCol(UCF_2.height, UCF_2.width);
    }
    if(mapUsed == "lab"){
        this->search.setRowCol(lab.height, lab.width);
    }
    this->search.initializeMap(this->robotWidth);
}


void Automation::setTurnLeft(bool TurnLeft){
    this->turnLeft = TurnLeft;
}

void Automation::centering(int xCounter, int zCounter){ // centering function to align with dump location
 	
    float originX = 3.5; //top left x of the target dump zone
    float originZ = 0.7; //top left z of the target dump zone
    float zChange = 0.3; //meters of change in the Z axis per row change
    float zAlign = 0.5; //meters south of the origin for aligning bucket to dump site
    float bucketOffset = .508; //offset for aligning to the front of the bucket

    if(!centeringSecond){
        setStartPosition(position.x, position.z);
        setDestPosition((originX + ((xCounter*BUCKET_WIDTH / 10) + (BUCKET_WIDTH/20))), originZ + zAlign); // calculate angle 1
        setDestAngle(getAngle());
    }
    else{
        setStartPosition(position.x, position.z);
        setDestPosition((originX + ((xCounter*BUCKET_WIDTH / 10) + (BUCKET_WIDTH/20))), originZ + (zCounter*zChange + bucketOffset)); // calculate angle 2
        setDestAngle(getAngle());
    }
}


enum Automation::TiltState Automation::checkOrientation(){
    if(position.roll < -TILT_THRESH){
        if(position.yaw > TILT_THRESH){
            RCLCPP_INFO(this->node->get_logger(), "FRONT RIGHT");
            return TILT_FRONT_RIGHT;
        }
        else if(position.yaw < -TILT_THRESH){
            RCLCPP_INFO(this->node->get_logger(), "FRONT LEFT");
            return TILT_FRONT_LEFT;
        }
        else{
            if(position.roll < -TIP_THRESH){
                RCLCPP_INFO(this->node->get_logger(), "TIP FRONT");
                return TIP_FRONT;
            }
            RCLCPP_INFO(this->node->get_logger(), "FRONT");
            return TILT_FRONT;
        }
    }
    else if(position.roll > TILT_THRESH){
        if(position.yaw > TILT_THRESH){
            RCLCPP_INFO(this->node->get_logger(), "BACK RIGHT");
            return TILT_BACK_RIGHT;
        }
        else if(position.yaw < -TILT_THRESH){
            RCLCPP_INFO(this->node->get_logger(), "BACK LEFT");
            return TILT_BACK_LEFT;
        }
        else{
            if(position.roll > TIP_THRESH){
                RCLCPP_INFO(this->node->get_logger(), "TIP BACK");
                return TIP_BACK;
            }
            RCLCPP_INFO(this->node->get_logger(), "BACK");
            return TILT_BACK;
        }
    }
    else{
        if(position.yaw > TILT_THRESH){
            RCLCPP_INFO(this->node->get_logger(), "RIGHT"); 
            return TILT_RIGHT;
        }
        else if(position.yaw < -TILT_THRESH){
            RCLCPP_INFO(this->node->get_logger(), "LEFT");
            return TILT_LEFT;
        }
    }
    return TILT_LEVEL;
}


void Automation::setLevelBucket(){
    if(!levelBucket)
        return;
    int currentArm = this->talon1.sensorValue;
    int currentBucket = this->talon3.sensorValue;
    float target = currentArm * (ARM_DEGREES / ARM_TRAVEL) * (BUCKET_TRAVEL / BUCKET_DEGREES) - position.roll * (BUCKET_TRAVEL / BUCKET_DEGREES) - 50;
    int bucketTarget = (int)target;
    RCLCPP_INFO(this->node->get_logger(), "bucketTarget: %d", bucketTarget);
    RCLCPP_INFO(this->node->get_logger(), "currentBucket: %d", currentBucket);
    setBucketPosition(bucketTarget);
}


void Automation::setLevelArms(){
    if(!levelArms)
        return;
    int currentArm = this->talon1.sensorValue;
    float target = 400 + position.roll * (ARM_TRAVEL / ARM_DEGREES);
    if(target < 40.0)
        target = 40.0;
    int armTarget = (int)target;
    RCLCPP_INFO(this->node->get_logger(), "armTarget: %d", armTarget);
    RCLCPP_INFO(this->node->get_logger(), "currentArm: %d", currentArm);
    setArmPosition(armTarget);
}


void Automation::startBucketLevel(){
    levelBucket = true;
}


void Automation::startArmsLevel(){
    levelArms = true;
}


void Automation::stopBucketLevel(){
    levelBucket = false;
}


void Automation::stopArmsLevel(){
    levelArms = false;
}
