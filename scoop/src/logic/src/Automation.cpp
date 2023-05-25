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
    goPublisher = this->node->create_publisher<std_msgs::msg::Empty>("GO", 1);
    stopPublisher = this->node->create_publisher<std_msgs::msg::Empty>("STOP",1);
    shoulderPublisher = this->node->create_publisher<std_msgs::msg::Float32>("shoulder_speed",1);
    dumpPublisher = this->node->create_publisher<std_msgs::msg::Float32>("dump_speed",1);
    neoPublisher = this->node->create_publisher<std_msgs::msg::Float32>("neo_speed",1);
    stepperPublisher = this->node->create_publisher<std_msgs::msg::Float32>("stepper_speed",1);
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
}


/** @brief Assigns/publishes left/right motorspeeds
 * 
 * This function assigns the speed of the left and right motors and then publishes them.
 * @param left, right
 * @return void
 * */
void Automation::changeSpeed(float left, float right){
    RCLCPP_INFO(this->node->get_logger(), "Current left: %f, Current right: %f", currentLeftSpeed, currentRightSpeed);
    RCLCPP_INFO(this->node->get_logger(), "New left: %f, New right: %f", left, right);
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

void Automation::setGo(){
   std_msgs::msg::Empty empty;
   goPublisher->publish(empty);
}

void Automation::setStop(){
    std_msgs::msg::Empty empty;
    stopPublisher->publish(empty);
}

void Automation::stopActuators(){
    std_msgs::msg::Float32 Speed;
    Speed.data = 0.0;
    shoulderPublisher->publish(Speed);
    dumpPublisher->publish(Speed);
    stepperPublisher->publish(Speed);
}

void Automation::setLinear1(const messages::msg::LinearOut::SharedPtr linearOut){
    this->linear1.speed = linearOut->speed;
    this->linear1.atMax = linearOut->at_max;
    this->linear1.atMin = linearOut->at_min;
    this->linear1.error = linearOut->error;
}

void Automation::setLinear2(const messages::msg::LinearOut::SharedPtr linearOut){
    this->linear2.speed = linearOut->speed;
    this->linear2.atMax = linearOut->at_max;
    this->linear2.atMin = linearOut->at_min;
    this->linear2.error = linearOut->error;
}

void Automation::setLinear3(const messages::msg::LinearOut::SharedPtr linearOut){
    this->linear3.speed = linearOut->speed;
    this->linear3.atMax = linearOut->at_max;
    this->linear3.atMin = linearOut->at_min;
    this->linear3.error = linearOut->error;
}

void Automation::setShoulderSpeed(float speed){
    std_msgs::msg::Float32 Speed;
    Speed.data = speed;
    shoulderPublisher->publish(Speed);
}

void Automation::setDumpSpeed(float speed){
    std_msgs::msg::Float32 Speed;
    Speed.data = speed;
    dumpPublisher->publish(Speed);
}

bool Automation::checkErrors(Linear linear){
    if(linear.error == "PotentiometerError" || linear.error == "ActuatorNotMovingError" || linear.error == "ConnectionError"){
        return true;
    }
    else{
        return false;
    }
}

void Automation::setNeoSpeed(float speed){
    std_msgs::msg::Float32 Speed;
    Speed.data = speed;
    neoPublisher->publish(Speed);
}

void Automation::setStepperSpeed(float speed){
    std_msgs::msg::Float32 Speed;
    Speed.data = speed;
    stepperPublisher->publish(Speed);
}

void Automation::setDestAngle(float degrees){
    if(degrees > 180){
        this->destAngle = degrees - 360;
    }
    else{
        this->destAngle = degrees;
    }
}

void Automation::setDestDistance(float meters){
    this->destDistance = meters;
}

void Automation::publishAutonomyOut(std::string robotStateString, std::string excavationStateString, std::string errorStateString, std::string dumpStateString){
    messages::msg::AutonomyOut aOut;
    aOut.robot_state = robotStateString;
    aOut.excavation_state = excavationStateString;
    aOut.error_state = errorStateString;
    aOut.dump_state = dumpStateString;
    autonomyOutPublisher->publish(aOut);
}

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

void Automation::setRunSensorlessly(bool value){
    this->runSensorlessly = value;
}