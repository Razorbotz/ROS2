#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/empty.hpp>
#include <messages/msg/key_state.hpp>

#include <messages/msg/zed_position.hpp>
#include "messages/msg/linear_status.hpp"
#include "messages/msg/talon_status.hpp"
#include "messages/msg/button_state.hpp"
#include "utils/utils.hpp"
#include "excavation_core.hpp"

rclcpp::Node::SharedPtr nodeHandle;
using std::placeholders::_1;

std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > talon14Publisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > talon15Publisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > talon16Publisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > talon17Publisher;

std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Int32_<std::allocator<void> >, std::allocator<void> > > talon14PositionPublisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Int32_<std::allocator<void> >, std::allocator<void> > > talon15PositionPublisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Int32_<std::allocator<void> >, std::allocator<void> > > talon16PositionPublisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Int32_<std::allocator<void> >, std::allocator<void> > > talon17PositionPublisher;

std::shared_ptr<rclcpp::Publisher<messages::msg::LinearStatus>> linearStatus1Publisher;
std::shared_ptr<rclcpp::Publisher<messages::msg::LinearStatus>> linearStatus2Publisher;
std::shared_ptr<rclcpp::Publisher<messages::msg::LinearStatus>> linearStatus3Publisher;
std::shared_ptr<rclcpp::Publisher<messages::msg::LinearStatus>> linearStatus4Publisher;

messages::msg::LinearStatus linearStatus1;
messages::msg::LinearStatus linearStatus2;
messages::msg::LinearStatus linearStatus3;
messages::msg::LinearStatus linearStatus4;

/** @file
 * @brief Node to control excavation motors
 * 
 * This node receives information intended to control the
 * linear actuators, then modifies the data to synchronize the
 * two linear actuators attached to the excavation assembly. 
 * This node also sends information about the linear actuators
 * back to the client-side GUI to integrate information about the
 * position data and error state. This node subscribes to the 
 * following topics:
 * \li \b potentiometer_data_1
 * \li \b potentiometer_data_2
 * \li \b potentiometer_data_3
 * \li \b potentiometer_data_4
 * \li \b automationGo
 * 
 * 
 * This node publishes the following topics:
 * \li \b talon_14_speed
 * \li \b talon_15_speed
 * \li \b talon_16_speed
 * \li \b talon_17_speed
 * \li \b linearStatus1
 * \li \b linearStatus2
 * \li \b linearStatus3
 * \li \b linearStatus4
 * 
 */

bool single_arm = false;
bool level_bucket = false;
bool level_arm = false;
float roll = 0.0;

// Number of degrees of travel of arm from fully retracted to fully extended
#define ARM_DEGREES 60.0
// Change in potentiometer value from fully retracted to fully extended
#define ARM_TRAVEL 945.0

#define BUCKET_DEGREES 120.0
#define BUCKET_TRAVEL 945.0

float armSetpoint = 400;         // pot value when user stops input
float bucketSetpoint = 0;        // pot value when user stops input
float pitchAtArmCapture = 0.0;   // pitch when arm setpoint was captured
float pitchAtBucketCapture = 0.0;// pitch when bucket setpoint was captured
bool prevLevelArm = false;
bool prevLevelBucket = false;
float filtered_pitch = 0.0;
bool first_pitch_received = false;
const float PITCH_ALPHA = 0.15;

// Global state
bool automationGo = false;
bool run = false;
float currentArmSpeed = 0.0f;
float currentBucketSpeed = 0.0f;

// Robot configuration flags
enum RobotMode {
    MODE_4_ACTUATOR,  // 2 arm, 2 bucket
    MODE_3_ACTUATOR,  // 2 arm, 1 bucket
    MODE_2_ACTUATOR   // 1 arm, 1 bucket
};

RobotMode robotMode    = MODE_4_ACTUATOR;
bool armHasPair        = true;   // 14 & 15
bool bucketHasPair     = true;   // 16 & 17

core::LinearActuator linear1(14,  9.8f, 0.85f, 11.5f);
core::LinearActuator linear2(15,  9.8f, 0.89f, 11.0f);
core::LinearActuator linear3(16, 11.8f, 0.69f,  8.5f);
core::LinearActuator linear4(17, 11.8f, 0.69f,  8.5f);

void goCallback(std_msgs::msg::Empty::SharedPtr empty){
    run = true;
}


void stopCallback(std_msgs::msg::Empty::SharedPtr empty){
    run = false;
}


/** @brief Function to publish the speeds of the first pair of 
 * actuators to Talons 14 and 15. 
 * 
 * The function creates two new Float32 messages, sets the data to
 * the value of the linear object speed, then publishes the data.
 * @return void
 * */
void publishSpeedsArm() {
    std_msgs::msg::Float32 speed1, speed2;
    speed1.data = linear1.speed;
    talon14Publisher->publish(speed1);
    linear1.previousSpeed = linear1.speed;

    if (robotMode == MODE_4_ACTUATOR || robotMode == MODE_3_ACTUATOR) {
        speed2.data = linear2.speed;
        talon15Publisher->publish(speed2);
        linear2.previousSpeed = linear2.speed;
    }

    //RCLCPP_INFO(nodeHandle->get_logger(), "Arm Speeds: %f, %f", linear1.speed, linear2.speed);
}


/** @brief Function to publish the speeds of the second pair of 
 * actuators to Talons 16 and 17. 
 * 
 * The function creates two new Float32 messages, sets the data to
 * the value of the linear object speed, then publishes the data.
 * @return void
 * */
void publishSpeedsBucket() {
    std_msgs::msg::Float32 speed3, speed4;
    speed3.data = linear3.speed;
    talon16Publisher->publish(speed3);
    linear3.previousSpeed = linear3.speed;

    if (robotMode == MODE_4_ACTUATOR) {
        speed4.data = linear4.speed;
        talon17Publisher->publish(speed4);
        linear4.previousSpeed = linear4.speed;
    }

    //RCLCPP_INFO(nodeHandle->get_logger(), "Bucket Speeds: %f, %f", linear3.speed, linear4.speed);
}


/** @brief Callback function for the automationGo topic. 
 * 
 * This function sets the automationGo value to the value
 * in the message.
 * @param msg - ROS2 message containing automationGo value
 * @return void
 * */
void automationGoCallback(const std_msgs::msg::Bool::SharedPtr msg){
    automationGo = msg->data;
}


void handleSyncErrors(core::LinearActuator* a, core::LinearActuator* b, float currentSpeed) {
    bool speedsChanged = core::enforceSyncLimits(a, b, currentSpeed);
    if (speedsChanged) {
        if (a->motorNumber == 14) {
            publishSpeedsArm();
        }
        else {
            publishSpeedsBucket();
        }
    }
}

/** @brief Callback function for potentiometer 1
 * 
 * The function sets the error state of linear actuator 1, then
 * processes the potentiometer data and sets the sync errors if the 
 * node is not running in sensorless mode. If the node is in sensorless
 * mode, the potentiometer data is ignored and nothing happens when the 
 * data is received.
 * @param msg - ROS2 message containing the value of the potentiomter
 * @return void
 * */
void potentiometer1Callback(const messages::msg::TalonStatus::SharedPtr msg){
    bool error = core::updateActuatorFromSensor(msg->sensor_position, msg->max_current, &linear1, run);
    if (error) {
        RCLCPP_INFO(nodeHandle->get_logger(), "EXCAVATION ERROR: %s", getErrorString(linear1.error));
    }
    if (armHasPair && !linear1.sensorless && !linear2.sensorless) {
        handleSyncErrors(&linear1, &linear2, currentArmSpeed);
    }
}


/** @brief Callback function for potentiometer 2.
 * 
 * The function sets the error state of linear actuator 2, then
 * processes the potentiometer data and sets the sync errors if the 
 * node is not running in sensorless mode. If the node is in sensorless
 * mode, the potentiometer data is ignored and nothing happens when the 
 * data is received.
 * @param msg - ROS2 message containing the value of the potentiomter
 * @return void
 * */
void potentiometer2Callback(const messages::msg::TalonStatus::SharedPtr msg){
    if (robotMode == MODE_2_ACTUATOR) {
        return;
    }

    bool error = core::updateActuatorFromSensor(msg->sensor_position, msg->max_current, &linear2, run);
    if (error) {
        RCLCPP_INFO(nodeHandle->get_logger(), "EXCAVATION ERROR: %s", getErrorString(linear2.error));
    }
    if (armHasPair && !linear1.sensorless && !linear2.sensorless) {
        handleSyncErrors(&linear1, &linear2, currentArmSpeed);
    }
}


/** @brief Callback function for potentiometer 3.
 * 
 * The function sets the error state of linear actuator 3, then
 * processes the potentiometer data and sets the sync errors if the 
 * node is not running in sensorless mode. If the node is in sensorless
 * mode, the potentiometer data is ignored and nothing happens when the 
 * data is received.
 * @param msg - ROS2 message containing the value of the potentiomter
 * @return void
 * */
void potentiometer3Callback(const messages::msg::TalonStatus::SharedPtr msg){
    bool error = core::updateActuatorFromSensor(msg->sensor_position, msg->max_current, &linear3, run);
    if (error) {
        RCLCPP_INFO(nodeHandle->get_logger(), "EXCAVATION ERROR: %s", getErrorString(linear3.error));
    }
    if (bucketHasPair &&!linear3.sensorless && !linear4.sensorless) {
        handleSyncErrors(&linear3, &linear4, currentBucketSpeed);
    }
}


/** @brief Callback function for potentiometer 4.
 * 
 * The function sets the error state of linear actuator 4, then
 * processes the potentiometer data and sets the sync errors if the 
 * node is not running in sensorless mode. If the node is in sensorless
 * mode, the potentiometer data is ignored and nothing happens when the 
 * data is received.
 * @param msg - ROS2 message containing the value of the potentiomter
 * @return void
 * */
void potentiometer4Callback(const messages::msg::TalonStatus::SharedPtr msg){
    if (robotMode != MODE_4_ACTUATOR) {
        return;
    }
    bool error = core::updateActuatorFromSensor(msg->sensor_position, msg->max_current, &linear4, run);
    if (error) {
        RCLCPP_INFO(nodeHandle->get_logger(), "EXCAVATION ERROR: %s", getErrorString(linear4.error));
    }
    if (bucketHasPair &&!linear3.sensorless && !linear4.sensorless) {
        handleSyncErrors(&linear3, &linear4, currentBucketSpeed);
    }
}


/** @brief Callback function for the armSpeed topic. 
 * 
 * This function sets the currentArmSpeed variable to the value contained in the 
 * speed->data. The speeds are set using the setSpeeds function and published
 * using the publishSpeeds function.
 * @param speed - ROS2 message containing speed value for the arm linear actuators
 * @return void
 * */
void armSpeedCallback(const std_msgs::msg::Float32::SharedPtr speed){
    currentArmSpeed = speed->data;

    if (std::abs(currentArmSpeed) > 0.05f) {
        prevLevelArm = false;
    }

    if (armHasPair) {
        core::setSpeedsPair(&linear1, &linear2, currentArmSpeed, automationGo);
    }
    else {
        linear1.speed = currentArmSpeed;
        core::setSpeedAtEnd(&linear1, currentArmSpeed);
    }
    publishSpeedsArm();
}


/** @brief Callback function for the bucketSpeed topic. 
 * 
 * This function sets the currentBucketSpeed variable to the value contained in the 
 * speed->data. The speeds are set using the setSpeeds2 function and published
 * using the publishSpeeds2 function.
 * @param speed - ROS2 message containing speed value for the arm linear actuators
 * @return void
 * */
void bucketSpeedCallback(const std_msgs::msg::Float32::SharedPtr speed){
    currentBucketSpeed = speed->data;

    if (std::abs(currentBucketSpeed) > 0.05f) {
        prevLevelBucket = false;
    }

    if (bucketHasPair) {
        core::setSpeedsPair(&linear3, &linear4, currentBucketSpeed, automationGo);
    }
    else {
        linear3.speed = currentBucketSpeed;
        core::setSpeedAtEnd(&linear3, currentBucketSpeed);
    }
    publishSpeedsBucket();
}

/** @brief Function to get the LinearStatus values
 * 
 * This function sets the values of the LinearStatus message
 * with the values from the linear actuator. 
 * @param *linearStatus - Pointer for the LinearStatus object
 * @param *linear - Pointer for the linear actuator
 * @return void
 * */
void getLinearStatus(messages::msg::LinearStatus *linearStatus, core::LinearActuator *linear){
    linearStatus->motor_number = linear->motorNumber;
    linearStatus->speed = linear->speed;
    linearStatus->potentiometer = linear->potentiometer;
    linearStatus->time_without_change = linear->timeWithoutChange;
    linearStatus->max = linear->max;
    linearStatus->min = linear->min;
    linearStatus->error = getErrorString(linear->error);
    linearStatus->at_min = linear->atMin;
    linearStatus->at_max = linear->atMax;
    linearStatus->distance = linear->distance;
    linearStatus->sensorless = linear->sensorless;
    linearStatus->stroke = linear->stroke;
    linearStatus->extension_speed = linear->extensionSpeed;
    linearStatus->time_to_extend = linear->timeToExtend;
}

/** @brief Function to update the estimated position of the motors when sensorless
 * mode is enabled.  
 * 
 * This function estimates the position of the motors by using the current speed, 
 * the extension speed of each motor, and the time elapsed in milliseconds. The delta
 * position is calculated by multiplying the speed by the extension speed in inches / sec
 * and number of seconds, then adds this value to the previous value. The positions of
 * all four linear actuators are calculated and the motor speeds are adjusted using the
 * setSpeedDistance and setSpeedDistance2 functions. 
 * @param speed - ROS2 message containing speed value for the arm linear actuators
 * @return void
 * */
void updateMotorPositions(int millis){
    // ARM
    core::updateMotorPosition(millis, &linear1, run);
    if (robotMode == MODE_4_ACTUATOR || robotMode == MODE_3_ACTUATOR)
        core::updateMotorPosition(millis, &linear2, run);

    if (armHasPair &&
        (linear1.sensorless || linear2.sensorless)) {
        core::setSpeedsDistancePair(&linear1, &linear2, currentArmSpeed);
    }

    // BUCKET
    core::updateMotorPosition(millis, &linear3, run);
    if (robotMode == MODE_4_ACTUATOR)
        core::updateMotorPosition(millis, &linear4, run);

    if (bucketHasPair &&
        (linear3.sensorless || linear4.sensorless)) {
        core::setSpeedsDistancePair(&linear3, &linear4, currentBucketSpeed);
    }
}

void zedPositionCallback(const messages::msg::ZedPosition::SharedPtr zedPosition){
    float raw_pitch = zedPosition->roll; 
    //RCLCPP_INFO(nodeHandle->get_logger(), "raw_pitch: %f", raw_pitch);

    if (!first_pitch_received) {
        filtered_pitch = raw_pitch;
        first_pitch_received = true;
    }
    else {
        filtered_pitch = (PITCH_ALPHA * raw_pitch) + ((1.0 - PITCH_ALPHA) * filtered_pitch);
    }
    //RCLCPP_INFO(nodeHandle->get_logger(), "filtered_pitch: %f", filtered_pitch);

    // --- ARM LEVELING ---
    if (level_arm) {
        if (std::abs(currentArmSpeed) > 0.05f) {
            prevLevelArm = false; 
        } 
        else {
            if (!prevLevelArm && linear1.initialized) {
                armSetpoint = linear1.potentiometer;
                pitchAtArmCapture = filtered_pitch;
                prevLevelArm = true; // Mark as successfully latched
            }

            if (prevLevelArm) {
                float pitch_delta = filtered_pitch - pitchAtArmCapture;
                float target = armSetpoint + pitch_delta * (ARM_TRAVEL / ARM_DEGREES);
                
                if(target < 40.0) target = 40.0;
                if(target > 980.0) target = 980.0; 
                
                //RCLCPP_INFO(nodeHandle->get_logger(), "Arm target: %f", target);
                std_msgs::msg::Int32 position;
                position.data = (int)target;
                talon14PositionPublisher->publish(position);
                if (armHasPair) talon15PositionPublisher->publish(position);
            }
        }
    }
    else {
        prevLevelArm = false; 
    }

    // --- BUCKET LEVELING ---
    if (level_bucket) {
        if (std::abs(currentBucketSpeed) > 0.05f) {
            prevLevelBucket = false;
        }
        else {
            if (!prevLevelBucket && linear3.initialized) {
                bucketSetpoint = linear3.potentiometer;
                pitchAtBucketCapture = filtered_pitch;
                prevLevelBucket = true; // Mark as successfully latched
            }

            if (prevLevelBucket) {
                float pitch_delta = filtered_pitch - pitchAtBucketCapture;
                float target = bucketSetpoint + pitch_delta * (BUCKET_TRAVEL / BUCKET_DEGREES);
                
                if(target > 900.0) target = 900.0;
                if(target < 20.0) target = 20.0; 
                RCLCPP_INFO(nodeHandle->get_logger(), "Bucket target: %f", target);
                
                std_msgs::msg::Int32 position;
                position.data = (int)target;
                talon16PositionPublisher->publish(position);
                if (bucketHasPair) talon17PositionPublisher->publish(position);
            }
        }
    }
    else {
        prevLevelBucket = false;
    }
}

/** @brief Callback function for joystick buttons
 * 
 * This function is called when the node receives a
 * topic with the name joystick_button.  
 * @param buttonState \see ButtonState.msg
 * @return void
 * */
void joystickButtonCallback(const messages::msg::ButtonState::SharedPtr buttonState){
    std::cout << "Button " << buttonState->joystick << " " << buttonState->button << " " << buttonState->state << std::endl;
    std_msgs::msg::Float32 armSpeed;
    std_msgs::msg::Float32 bucketSpeed;
    std_msgs::msg::Bool state; 

    switch (buttonState->button) { 
        case 2:
            RCLCPP_INFO(nodeHandle->get_logger(), "Button 3");
            if(buttonState->state == 1){
                level_arm = !level_arm;
            }
            break;
        case 3:
            RCLCPP_INFO(nodeHandle->get_logger(), "Button 4");
            if(buttonState->state == 1){
                level_bucket = !level_bucket;
            }
            break;
    }
}


int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    nodeHandle = rclcpp::Node::make_shared("excavation");

    single_arm = utils::getParameter<bool>(nodeHandle, "single_arm", false);
    std::string robot_mode_str = utils::getParameter<std::string>(nodeHandle, "actuator_mode", "2_actuator");

    if (robot_mode_str == "4_actuator") {
        robotMode   = MODE_4_ACTUATOR;
        armHasPair  = true;
        bucketHasPair = true;
    }
    else if (robot_mode_str == "3_actuator") {
        robotMode   = MODE_3_ACTUATOR;
        armHasPair  = true;
        bucketHasPair = false;  // only linear3 on bucket
    }
    else {
        robotMode   = MODE_2_ACTUATOR;
        armHasPair  = false;    // only linear1
        bucketHasPair = false;  // only linear3
    }

    auto automationGoSubscriber = nodeHandle->create_subscription<std_msgs::msg::Bool>("automationGo",1,automationGoCallback);
    auto stopSubscriber = nodeHandle->create_subscription<std_msgs::msg::Empty>("STOP",1,stopCallback);
    auto goSubscriber = nodeHandle->create_subscription<std_msgs::msg::Empty>("GO",1,goCallback);

    auto armSpeedSubscriber = nodeHandle->create_subscription<std_msgs::msg::Float32>("arm_speed",1,armSpeedCallback);
    auto bucketSpeedSubscriber = nodeHandle->create_subscription<std_msgs::msg::Float32>("bucket_speed",1,bucketSpeedCallback);

    auto talon1Subscriber = nodeHandle->create_subscription<messages::msg::TalonStatus>("talon_14_info",1,potentiometer1Callback);
    auto talon2Subscriber = nodeHandle->create_subscription<messages::msg::TalonStatus>("talon_15_info",1,potentiometer2Callback);
    auto talon3Subscriber = nodeHandle->create_subscription<messages::msg::TalonStatus>("talon_16_info",1,potentiometer3Callback);
    auto talon4Subscriber = nodeHandle->create_subscription<messages::msg::TalonStatus>("talon_17_info",1,potentiometer4Callback);

    auto zedPositionSubscriber= nodeHandle->create_subscription<messages::msg::ZedPosition>("zed_position",1,zedPositionCallback);
    auto joystickButtonSubscriber= nodeHandle->create_subscription<messages::msg::ButtonState>("joystick_button",1,joystickButtonCallback);

    talon14Publisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("talon_14_speed",1);
    talon15Publisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("talon_15_speed",1);
    talon16Publisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("talon_16_speed",1);
    talon17Publisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("talon_17_speed",1);
    talon14PositionPublisher = nodeHandle->create_publisher<std_msgs::msg::Int32>("talon_14_position",1);
    talon15PositionPublisher = nodeHandle->create_publisher<std_msgs::msg::Int32>("talon_15_position",1);
    talon16PositionPublisher = nodeHandle->create_publisher<std_msgs::msg::Int32>("talon_16_position",1);
    talon17PositionPublisher = nodeHandle->create_publisher<std_msgs::msg::Int32>("talon_17_position",1);

    linearStatus1Publisher = nodeHandle->create_publisher<messages::msg::LinearStatus>("linearStatus1",1);
    linearStatus2Publisher = nodeHandle->create_publisher<messages::msg::LinearStatus>("linearStatus2",1);
    linearStatus3Publisher = nodeHandle->create_publisher<messages::msg::LinearStatus>("linearStatus3",1);
    linearStatus4Publisher = nodeHandle->create_publisher<messages::msg::LinearStatus>("linearStatus4",1);

    auto start = std::chrono::high_resolution_clock::now();
    auto finish = std::chrono::high_resolution_clock::now();
    rclcpp::Rate rate(60);
    while(rclcpp::ok()){
        finish = std::chrono::high_resolution_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(finish - start).count();
        if(elapsed_ms > 33){
            updateMotorPositions(elapsed_ms);
            getLinearStatus(&linearStatus1, &linear1);
            linearStatus1Publisher->publish(linearStatus1);

            if (robotMode == MODE_4_ACTUATOR || robotMode == MODE_3_ACTUATOR) {
                getLinearStatus(&linearStatus2, &linear2);
                linearStatus2Publisher->publish(linearStatus2);
            }
            
            getLinearStatus(&linearStatus3, &linear3);
            linearStatus3Publisher->publish(linearStatus3);

            if (robotMode == MODE_4_ACTUATOR) {
                getLinearStatus(&linearStatus4, &linear4);
                linearStatus4Publisher->publish(linearStatus4);
            }
            start = std::chrono::high_resolution_clock::now();
        }
        rate.sleep();
        rclcpp:spin_some(nodeHandle);
    }
}
