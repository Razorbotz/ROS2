#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/empty.hpp>
#include <messages/msg/key_state.hpp>

#include "messages/msg/linear_status.hpp"
#include "messages/msg/talon_status.hpp"
#include "utils/utils.hpp"

rclcpp::Node::SharedPtr nodeHandle;
using std::placeholders::_1;

std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > talon14Publisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > talon15Publisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > talon16Publisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > talon17Publisher;

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


enum Error {
    ActuatorsSyncError,
    ActuatorNotMovingError,
    PotentiometerError,
    None
};

bool single_arm = false;

std::map<Error, const char*> errorMap = {{ActuatorsSyncError, "ActuatorsSyncError"},
    {ActuatorNotMovingError, "ActuatorNotMovingError"},
    {PotentiometerError, "PotentiometerError"},
    {None, "None"}};

// Range where pot is floating/disconnected
constexpr int POT_FLOAT_LOW = 100;
constexpr int POT_FLOAT_HIGH = 110;

// Approx valid working range: ~30 - 980 (span ~950)
constexpr int POT_RAW_MIN_VALID = 30;
constexpr int POT_RAW_MAX_VALID = 980;
constexpr int POT_RAW_RANGE = POT_RAW_MAX_VALID - POT_RAW_MIN_VALID;

// Noise and thresholds
constexpr int NOISE_THRESH = 2;
constexpr int NO_MOVEMENT_LIMIT = 15;

// Distance sync thresholds (inches)
float distThresh1 = 0.05f;
float distThresh2 = 0.10f;
float distThresh3 = 0.15f;

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

struct LinearActuator{
    int motorNumber = 0;
    float speed = 0.0;              // Speed variable of linear actuator
    int potentiometer = 0;          // Potentiometer reading
    int timeWithoutChange = 0;      // Number of potentiometer values received without change when speed > 0
    int max = 0;                    // Max potentiometer value
    int min = 1024;                 // Min potentiometer value
    Error error = None;             // Error state of the actuator
    bool atMin = false;             // Bool value of if actuator is at min extension
    bool atMax = false;             // Bool value of if actuator is at max extension
    float stroke = 11.8;            // Length of stroke of the actuator
    float distance = 0.0;           // Distance extended
    float extensionSpeed = 0.0;     // Speed of extension in in/sec
    float timeToExtend = 0.0;       // Time to fully extend actuator
    bool sensorless = false;        // Running without sensor
    float maxCurrent = 0.0;         
    bool initialized = false;
    float previousSpeed = 0.0;
    int previousPotent = 0;
    //float lowerDistance = 0.0;
   // float upperDistance = 0.0;
    //float lowerSpeed = 0.0;
    //float upperSpeed = 0.0;
    LinearActuator(int motor, float strokeLength, float ExtensionSpeed, float TimeToExtend)
        : motorNumber(motor), stroke(strokeLength), extensionSpeed(ExtensionSpeed), timeToExtend(TimeToExtend) {}
};

LinearActuator linear1(14,  9.8f, 0.85f, 11.5f);
LinearActuator linear2(15,  9.8f, 0.89f, 11.0f);
LinearActuator linear3(16, 11.8f, 0.69f,  8.5f);
LinearActuator linear4(17, 11.8f, 0.69f,  8.5f);

inline bool isFloatValue(int v) {
    return v >= POT_FLOAT_LOW && v <= POT_FLOAT_HIGH;
}

inline bool isRealValue(int v) {
    return v >= POT_RAW_MIN_VALID && v <= POT_RAW_MAX_VALID;
}

void goCallback(std_msgs::msg::Empty::SharedPtr empty){
    run = true;
}


void stopCallback(std_msgs::msg::Empty::SharedPtr empty){
    run = false;
}

/** @brief Function to sync the linear actuators. 
 * 
 * The sync function works by checking if the currentSpeed is
 * greater than zero. If the speed is greater than zero, the val
 * checks which actuator is more extended and sets the speed of
 * the actuator to a lower value if the diff is greater than the 
 * thresh values.  If the value is less than zero, the val checks 
 * which actuator is less extended and sets the speed of the 
 * actuator to a lower value.
 * @return void
 * */
/*
    val truth table:
    if Current Speed > 0:                   If actuators are extending
        if linear1.pot >= linear2.pot:      If linear1 is further extended, use first value in ternary operators below
            val = true
        else:
            val = false
    else:                                   If actuators are retracting
        if linear1.pot < linear2.pot:       If linear1 is further retracted, use first value in ternary operators below
            val = true
        else:
            val = false
    */
void sync(LinearActuator* a, LinearActuator* b, float currentSpeed) {
    float diff = std::abs(a->potentiometer - b->potentiometer);
    bool aIsAhead = (currentSpeed > 0)
                    ? (a->potentiometer >= b->potentiometer)
                    : (a->potentiometer <  b->potentiometer);

    float scale = 950.0f / a->stroke;

    if (diff > scale / 6.0f) {
        if (aIsAhead) a->speed = 0.0f;
        else          b->speed = 0.0f;
    }
    else if (diff > scale / 9.0f) {
        if (aIsAhead) a->speed *= 0.5f;
        else          b->speed *= 0.5f;
    }
    else if (diff > scale / 12.0f) {
        if (aIsAhead) a->speed *= 0.9f;
        else          b->speed *= 0.9f;
    }
    else {
        a->speed = currentSpeed;
        b->speed = currentSpeed;
    }
}


/** @brief Function to sync the linear actuators when using the distance 
 * calculated from the time running. 
 * 
 * The sync function works by checking if the currentSpeed is
 * greater than zero. If the speed is greater than zero, the val
 * checks which actuator is more extended and sets the speed of
 * the actuator to a lower value if the diff is greater than the 
 * thresh values.  If the value is less than zero, the val checks 
 * which actuator is less extended and sets the speed of the 
 * actuator to a lower value. 
 * @return void
 * */
void syncDistance(LinearActuator* a, LinearActuator* b, float currentSpeed) {
    float diff = std::abs(a->distance - b->distance);
    bool aIsAhead = (currentSpeed > 0)
                    ? (a->distance >= b->distance)
                    : (a->distance <  b->distance);

    if (diff > distThresh3) {
        if (aIsAhead) a->speed = 0.0f;
        else          b->speed = 0.0f;

        if (!a->sensorless) a->error = ActuatorsSyncError;
        if (!b->sensorless) b->error = ActuatorsSyncError;
    }
    else if (diff > distThresh2) {
        if (aIsAhead) a->speed *= 0.5f;
        else          b->speed *= 0.5f;

        if (!a->sensorless && a->error == ActuatorsSyncError) a->error = None;
        if (!b->sensorless && b->error == ActuatorsSyncError) b->error = None;
    }
    else if (diff > distThresh1) {
        if (aIsAhead) a->speed *= 0.9f;
        else          b->speed *= 0.9f;

        if (!a->sensorless && a->error == ActuatorsSyncError) a->error = None;
        if (!b->sensorless && b->error == ActuatorsSyncError) b->error = None;
    }
    else {
        a->speed = currentSpeed;
        b->speed = currentSpeed;
        if (!a->sensorless && a->error == ActuatorsSyncError) a->error = None;
        if (!b->sensorless && b->error == ActuatorsSyncError) b->error = None;
    }
}


void setSpeedAtEnd(LinearActuator* a, float currentSpeed) {
    if ((a->atMax && currentSpeed > 0) || (a->atMin && currentSpeed < 0))
        a->speed = 0.0f;
}


/** @brief Function that sets the speeds of the first pair of linear
 * actuators, then syncs the motors. 
 * 
 * The setSpeed function checks if the linear actuators are at the min
 * or max, then sets the speed to 0.0 if either are true.
 * @return void
 * */
void setSpeedsPair(LinearActuator* a, LinearActuator* b, float currentSpeed) {
    if (!automationGo) {
        a->speed = currentSpeed;
        b->speed = currentSpeed;
    } else {
        if (a->error != PotentiometerError && b->error != PotentiometerError) {
            a->speed = currentSpeed;
            b->speed = currentSpeed;
        }
    }

    if (a->error != PotentiometerError && b->error != PotentiometerError) {
        sync(a, b, currentSpeed);
        setSpeedAtEnd(a, currentSpeed);
        setSpeedAtEnd(b, currentSpeed);
    }
}


/** @brief Function that sets the speeds of the first pair of linear
 * actuators, then syncs the motors.
 * 
 * The setSpeed function checks if the linear actuators are at the min
 * or max, then sets the speed to 0.0 if either are true. The values are
 * published if either is not zero or the currentSpeed is not zero.
 * @return void
 * */
void setSpeedsDistancePair(LinearActuator* a, LinearActuator* b, float currentSpeed) {
    a->speed = currentSpeed;
    b->speed = currentSpeed;
    syncDistance(a, b, currentSpeed);
    setSpeedAtEnd(a, currentSpeed);
    setSpeedAtEnd(b, currentSpeed);
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

    RCLCPP_INFO(nodeHandle->get_logger(), "Arm Speeds: %f, %f", linear1.speed, linear2.speed);
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

    RCLCPP_INFO(nodeHandle->get_logger(), "Bucket Speeds: %f, %f", linear3.speed, linear4.speed);
}


/** @brief Function to set potentiometer error.
 * 
 * This function is used to set the value of the error
 * of the linear object.  If the potentiometer is equal
 * to 1024, which is the value that occurs when the
 * potentiometer is disconnected from the Arduino. Refer
 * to the ErrorState state diagram for more information.
 * @param potentData - Int value of potentiometer
 * @param *linear - Pointer to linear object
 * @return void
 * */
void setPotentiometerError(int potentData, LinearActuator* linear) {
    // 1024 or higher means completely disconnected
    if (potentData > 1024) {
        linear->error = PotentiometerError;
        linear->sensorless = true;
        RCLCPP_INFO(nodeHandle->get_logger(), "EXCAVATION ERROR: PotentiometerError");
    }

    // If we return to a “non-floating” region, clear pot error
    if (potentData > POT_FLOAT_HIGH || potentData < POT_FLOAT_LOW) {
        if (linear->error == PotentiometerError) {
            linear->error = None;
        }
    }
}


/** @brief Function to process potentiometer data. 
 * 
 * This function processes the passed potentiometer data
 * and adjusts the passed linear values accordingly. First
 * the function sets the min and max values if the new data
 * is beyond the previous limits. Next, the function checks
 * if the value is within a threshold of the previous value
 * that is stored in the linear->potentiometer variable. If
 * the value is within this threshold, it's assumed that 
 * the actuator isn't moving. If the speed isn't equal to
 * zero, ie the actuator should be moving, the timeWithoutChange
 * variable gets increased. If the timeWithoutChange is greater than 5,
 * the function checks if the actuator is at the min or max
 * positions and sets the corresponding values to true if
 * it is.  If the data is outside of the threshold, the 
 * actuator is moving as intended and is not at the min or
 * max positions.
 * 
 * NOTE: If the potentiometer is disconnected, the values fall to
 * between 100 and 110.
 * @param potentData - Int value of potentiometer
 * @param *linear - Pointer to linear object
 * @return void
 * */
void processPotentiometerData(int potentData, LinearActuator* linear) {
    // Track observed min/max within valid range
    if (potentData < linear->min) linear->min = potentData;
    if (potentData > linear->max) linear->max = potentData;

    // Initialization: we don't know if connected while in 100–110
    if (!linear->initialized) {
        if (!isFloatValue(potentData) && isRealValue(potentData)) {
            linear->initialized = true;
        }
    }

    // If initialized and value jumps into 100–110 or sticks there while moving:
    if (linear->initialized && isFloatValue(potentData)) {
        if (std::abs(linear->potentiometer - potentData) > 50) {
            linear->sensorless = true;
            linear->error = PotentiometerError;
        }
    }

    if (isRealValue(potentData)) {
        linear->distance = linear->stroke * (static_cast<float>(potentData - POT_RAW_MIN_VALID) / POT_RAW_RANGE);
    }

    // Not-moving detection
    if (linear->potentiometer >= potentData - NOISE_THRESH && linear->potentiometer <= potentData + NOISE_THRESH) {
        if (linear->speed != 0.0f && run) {
            linear->timeWithoutChange += 1;
            if (linear->timeWithoutChange >= NO_MOVEMENT_LIMIT) {
                if (isFloatValue(linear->potentiometer) && !linear->initialized) {
                    linear->sensorless = true;
                    linear->error = PotentiometerError;
                }
                else if (linear->max > 800 && linear->speed > 0.0f && potentData >= linear->max - 20) {
                    linear->atMax = true;
                    linear->timeWithoutChange = 0;
                }
                else if (linear->min < 200 && linear->speed < 0.0f && potentData <= linear->min + 20) {
                    linear->atMin = true;
                    linear->timeWithoutChange = 0;
                }
                else {
                    if (linear->error == None || linear->error == ActuatorsSyncError) {
                        if(linear->initialized && isFloatValue(potentData)){
                            linear->sensorless = true;
                            linear->error = PotentiometerError;
                            RCLCPP_INFO(nodeHandle->get_logger(), "EXCAVATION ERROR: PotentiometerError");
                        }
                        else{
                            linear->error = ActuatorNotMovingError;
                            RCLCPP_INFO(nodeHandle->get_logger(), "EXCAVATION ERROR: ActuatorNotMovingError");
                        }
                    }
                }
            }
        }
    }
    else {
        linear->timeWithoutChange = 0;
        if (linear->error == ActuatorNotMovingError)
            linear->error = None;

        if (linear->atMax && linear->speed < 0.0f) linear->atMax = false;
        if (linear->atMin && linear->speed > 0.0f) linear->atMin = false;
    }

    linear->potentiometer = potentData;

    // Special case for bucket actuators (16, 17)
    if (linear->motorNumber == 16 || linear->motorNumber == 17) {
        if (potentData > 700) {
            linear->atMax = true;
        }
        else{
            linear->atMax = false;
        }
    }
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


/** @brief Function that checks if the linear actuators are out of sync
 * then sets the error state to the correct one
 * 
 * The function checks if the difference between the potentiometers is 
 * greater than the thresh1 value, then checks if the error state is 
 * None. If the error is None, the error is set to ActuatorsSyncError, 
 * which indicates that the actuators are out of sync.
 * @return void
 * */
void setSyncErrors(LinearActuator* a, LinearActuator* b, float currentSpeed) {
    float diff = std::abs(a->potentiometer - b->potentiometer);
    float thresh = (950.0f / a->stroke) / 6.0f;

    if (diff > thresh) {
        if (isFloatValue(a->potentiometer) && !a->initialized) {
            a->error = PotentiometerError;
            a->sensorless = true;
        }
        if (isFloatValue(b->potentiometer) && !b->initialized) {
            b->error = PotentiometerError;
            b->sensorless = true;
        }
        if (a->error == None) a->error = ActuatorsSyncError;
        if (b->error == None) b->error = ActuatorsSyncError;
    }
    else {
        if (a->error == ActuatorsSyncError) a->error = None;
        if (b->error == ActuatorsSyncError) b->error = None;
    }

    if (a->error != PotentiometerError && b->error != PotentiometerError) {
        sync(a, b, currentSpeed);
        if (a->speed != a->previousSpeed || b->speed != b->previousSpeed) {
            if (a->motorNumber == 14) {
                publishSpeedsArm();
            }
            else {
                publishSpeedsBucket();
            }
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
    linear1.maxCurrent = msg->max_current;
    if (!linear1.sensorless) {
        setPotentiometerError(msg->sensor_position, &linear1);
        if (linear1.error != PotentiometerError) {
            processPotentiometerData(msg->sensor_position, &linear1);
            if (armHasPair &&
                !linear1.sensorless && !linear2.sensorless) {
                setSyncErrors(&linear1, &linear2, currentArmSpeed);
            }
        }
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

    linear2.maxCurrent = msg->max_current;
    if (!linear2.sensorless) {
        setPotentiometerError(msg->sensor_position, &linear2);
        if (linear2.error != PotentiometerError) {
            processPotentiometerData(msg->sensor_position, &linear2);
            if (armHasPair &&
                !linear1.sensorless && !linear2.sensorless) {
                setSyncErrors(&linear1, &linear2, currentArmSpeed);
            }
        }
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
    linear3.maxCurrent = msg->max_current;
    if (!linear3.sensorless) {
        setPotentiometerError(msg->sensor_position, &linear3);
        if (linear3.error != PotentiometerError) {
            processPotentiometerData(msg->sensor_position, &linear3);
            if (bucketHasPair &&
                !linear3.sensorless && !linear4.sensorless) {
                setSyncErrors(&linear3, &linear4, currentBucketSpeed);
            }
        }
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
    linear4.maxCurrent = msg->max_current;
    if (!linear4.sensorless) {
        setPotentiometerError(msg->sensor_position, &linear4);
        if (linear4.error != PotentiometerError) {
            processPotentiometerData(msg->sensor_position, &linear4);
            if (bucketHasPair &&
                !linear3.sensorless && !linear4.sensorless) {
                setSyncErrors(&linear3, &linear4, currentBucketSpeed);
            }
        }
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

    if (armHasPair) {
        setSpeedsPair(&linear1, &linear2, currentArmSpeed);
    }
    else {
        linear1.speed = currentArmSpeed;
        setSpeedAtEnd(&linear1, currentArmSpeed);
        
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

    if (bucketHasPair) {
        setSpeedsPair(&linear3, &linear4, currentBucketSpeed);
    }
    else {
        linear3.speed = currentBucketSpeed;
        setSpeedAtEnd(&linear3, currentBucketSpeed);
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
void getLinearStatus(messages::msg::LinearStatus *linearStatus, LinearActuator *linear){
    linearStatus->motor_number = linear->motorNumber;
    linearStatus->speed = linear->speed;
    linearStatus->potentiometer = linear->potentiometer;
    linearStatus->time_without_change = linear->timeWithoutChange;
    linearStatus->max = linear->max;
    linearStatus->min = linear->min;
    linearStatus->error = errorMap.at(linear->error);
    linearStatus->at_min = linear->atMin;
    linearStatus->at_max = linear->atMax;
    linearStatus->distance = linear->distance;
    linearStatus->sensorless = linear->sensorless;
    linearStatus->stroke = linear->stroke;
    linearStatus->extension_speed = linear->extensionSpeed;
    linearStatus->time_to_extend = linear->timeToExtend;
}


/*
Extending:
Upper += Time * upperSpeed;
Est += Time * (lowerSpeed + (expMaxCurr - current) * (upperSpeed - lowerSpeed))
Lower += Time * lowerSpeed;

Retracting:
Upper -= Time * lowerSpeed;
Est -= Time * (lowerSpeed + (expMaxCurr - current) * (upperSpeed - lowerSpeed))
Lower -= Time * upperSpeed;

The upper and lower speeds are given by the datasheet.
The estimate relies on using the current to estimate load and speed of the motor
to get a better estimate of what the current position is. 
These upper and lower estimates should bound the possible positions for the
actuator based on the max and min speeds of the motor. These bounds will
grow smaller when the actuator reaches end of travel in either direction.
*/
void updateMotorPosition(int millis, LinearActuator *linear){
    if(run){
        linear->distance = linear->speed * linear->extensionSpeed * (millis / 1000.0) + linear->distance;
    }
    if(linear->distance > linear->stroke){
        linear->distance = linear->stroke;
        linear->atMax = true;
    }
    else if(linear->distance < 0.0){
        linear->distance = 0.0;
        linear->atMin = true;
    }
    else{
        linear->atMin = false;
        linear->atMax = false;
    }
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
    updateMotorPosition(millis, &linear1);
    if (robotMode == MODE_4_ACTUATOR || robotMode == MODE_3_ACTUATOR)
        updateMotorPosition(millis, &linear2);

    if (armHasPair &&
        (linear1.sensorless || linear2.sensorless)) {
        setSpeedsDistancePair(&linear1, &linear2, currentArmSpeed);
    }

    // BUCKET
    updateMotorPosition(millis, &linear3);
    if (robotMode == MODE_4_ACTUATOR)
        updateMotorPosition(millis, &linear4);

    if (bucketHasPair &&
        (linear3.sensorless || linear4.sensorless)) {
        setSpeedsDistancePair(&linear3, &linear4, currentBucketSpeed);
    }
}


int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    nodeHandle = rclcpp::Node::make_shared("excavation");

    single_arm = utils::getParameter<bool>(nodeHandle, "single_arm", false);
    std::string robot_mode_str = utils::getParameter<std::string>(nodeHandle, "actuator_mode", "4_actuator");

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

    talon14Publisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("talon_14_speed",1);
    talon15Publisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("talon_15_speed",1);
    talon16Publisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("talon_16_speed",1);
    talon17Publisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("talon_17_speed",1);
    
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
