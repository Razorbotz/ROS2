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
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

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


/*
Function to get the angle that the robot should be facing to
travel to the destination. This should be between the values of
[-180, 180]. 
NOTE: This has not been confirmed to work and needs to be tested
significantly more to ensure that the results are corret.
*/
float Automation::getAngle(){
    float y = this->search.destX - this->search.startX;
    float x = this->search.destY - this->search.startY;
    float angle = std::atan2(y, x) * 180 / M_PI;
    angle += 90;
    if(angle > 180){
        angle -= 360;
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
int Automation::checkAngle(){
    if(abs(this->destAngle) > 177){
        if(std::abs(position.pitch) < std::abs(this->destAngle) + 2 && std::abs(position.pitch) > std::abs(this->destAngle) - 2){
            return 1;
        }
    }
    else{
        if(position.pitch < this->destAngle + 2 && position.pitch > this->destAngle - 2){
            return 1;
        }
    }
    float angle = getAngleDiff(position.pitch);
    if(angle < 0){
        return 1;
    }
    return 0;
}


/*
Function that checks the distance between the robot's current position
and the destination. The greater the distance returned, the further out
the robot is from the target location. 
*/
int Automation::checkDistance(){
    float currentZ = (this->search.Row / 10.0) - position.z;
    float currentX = position.x + this->xOffset;
    float dist = sqrt(pow(currentZ - (this->search.destX / 10.0), 2) + pow(currentX - (this->search.destY / 10.0), 2));
    if(dist == 0)
        return 0;
    if(dist < 0.5)
        return 1;
    if(dist < 1)
        return 2;
    return 3;
    
}


/*
Function to set the X coordinate of the destination. 

TODO: Explain why this sets the destY coordinate
*/
void Automation::setDestX(float meters){
    this->search.destY = meters;
}


void Automation::setDestZ(float meters){
    this->search.destX = meters;
}


void Automation::publishAutonomyOut(std::string robotStateString, std::string excavationStateString, std::string errorStateString, std::string diagnosticsStateString, std::string tiltStateString){
    messages::msg::AutonomyOut aOut;
    aOut.robot_state = robotStateString;
    aOut.excavation_state = excavationStateString;
    aOut.error_state = errorStateString;
    aOut.diagnostics_state = diagnosticsStateString;
    aOut.tilt_state = tiltStateString;
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


void Automation::setBackupStartTime(std::chrono::time_point<std::chrono::high_resolution_clock> StartTime){
    this->startBackupTime = StartTime;
}


std::chrono::time_point<std::chrono::high_resolution_clock> Automation::getStartTime(){
    return this->startTime;
}


std::chrono::time_point<std::chrono::high_resolution_clock> Automation::getBackupStartTime(){
    return this->startBackupTime;
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
void Automation::setStartPositionM(float x, float y){
    this->search.startX = this->search.Row - int(std::ceil(x * 10));
    this->search.startY = int(std::ceil((y + this->xOffset) * 10));
}


void Automation::setStartPosition(int x, int y){
    this->search.startX = x;
    this->search.startY = y;
}


/*
Function to set the destination position of the robot given the current
position in meters. These values are transformed into decimeters,
then transformed to map it into the 2D array.
*/
void Automation::setDestPositionM(float x, float y){
    this->search.destX = this->search.Row - int(std::ceil(x * 10));
    this->search.destY = int(std::ceil((y + this->xOffset) * 10));
}


void Automation::setDestPosition(int x, int y){
    this->search.destX = x;
    this->search.destY = y;
}


/*
Function to run the A-Star algorithm. The includeHoles parameter
tells the search algorithm whether or not holes should be included
in the path or if they must be avoided.
*/
void Automation::aStar(bool includeHoles){
    this->currentPath = this->search.aStar(includeHoles);
}


void Automation::aStar(std::stack<Coord> points, bool includeHoles, bool simplify){
    this->currentPath = this->search.aStar(points, includeHoles, simplify);
}


/*
* Function to set the target position of the arms.
* @param potent - Desired int value of potentiometer
*/
void Automation::setArmTarget(int potent){
    target1 = potent;
}


/*
* Function to set the target position of the bucket.
* @param potent - Desired int value of potentiometer
*/
void Automation::setBucketTarget(int potent){
    target3 = potent;
}


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
    setArmTarget(potent);
    int current = this->talon1.sensorValue;
    float timeToRun = abs(current - potent) * (linear1.timeToExtend / 900.0) * 1000;
    RCLCPP_INFO(this->node->get_logger(), "Arm potent: %d", potent);
    RCLCPP_INFO(this->node->get_logger(), "Arm timeToExtend: %f", linear1.timeToExtend);
    RCLCPP_INFO(this->node->get_logger(), "Arm timeToRuN: %f", timeToRun);
    if(potent > current){
        setArmSpeed(1.0);
    }
    else{
        setArmSpeed(-1.0);
    }
    auto start = std::chrono::high_resolution_clock::now();
    while(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-start).count() < timeToRun){}
    setArmSpeed(0.0);
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
    setBucketTarget(potent);
    int current = this->talon3.sensorValue;
    float timeToRun = abs(current - potent) * (linear3.timeToExtend / 900.0) * 1000;
    RCLCPP_INFO(this->node->get_logger(), "Bucket potent: %d", potent);
    RCLCPP_INFO(this->node->get_logger(), "Bucket timeToExtend: %f", linear3.timeToExtend);
    RCLCPP_INFO(this->node->get_logger(), "Bucket timeToRuN: %f", timeToRun);
    if(potent > current){
        setBucketSpeed(1.0);
    }
    else{
        setBucketSpeed(-1.0);
    }
    auto start = std::chrono::high_resolution_clock::now();
    while(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-start).count() < timeToRun){}
    setBucketSpeed(0.0);
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


void Automation::setxOffset(float XOffset){
    this->xOffset = XOffset;
}


void Automation::setTurnLeft(bool TurnLeft){
    this->turnLeft = TurnLeft;
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
    int currentArm = this->talon1.sensorValue;
    int currentBucket = this->talon3.sensorValue;
    float target = currentArm * (ARM_DEGREES / ARM_TRAVEL) * (BUCKET_TRAVEL / BUCKET_DEGREES) + position.roll * (BUCKET_TRAVEL / BUCKET_DEGREES);
    int bucketTarget = (int)target;
    if(std::abs(target - currentBucket) < 5)
        return;
    setBucketPosition(bucketTarget);
}


void Automation::setLevelArms(){
    int currentArm = this->talon1.sensorValue;
    float target = 400 + position.roll * (ARM_DEGREES / ARM_TRAVEL);
    if(target < 40.0)
        target = 40.0;
    int armTarget = (int)target;
    if(std::abs(target - currentArm) < 5)
        return;
    setArmPosition(armTarget);
}