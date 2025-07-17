#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/empty.hpp>
#include <messages/msg/key_state.hpp>

#include "messages/msg/linear_status.hpp"
#include "messages/msg/talon_status.hpp"

rclcpp::Node::SharedPtr nodeHandle;

std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > talon14Publisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > talon15Publisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > talon16Publisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > talon17Publisher;

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
};


LinearActuator linear1{14, 0.0, 0, 0, 0, 1024, None, false, false, 9.8, 0.0, 0.85, 11.5, false, 0.0, false, 0.0, 0};
LinearActuator linear2{15, 0.0, 0, 0, 0, 1024, None, false, false, 9.8, 0.0, 0.89, 11.0, false, 0.0, false, 0.0, 0};
LinearActuator linear3{16, 0.0, 0, 0, 0, 1024, None, false, false, 11.8, 0.0, 0.69, 8.5, false, 0.0, false, 0.0, 0};
LinearActuator linear4{17, 0.0, 0, 0, 0, 1024, None, false, false, 11.8, 0.0, 0.69, 8.5, false, 0.0, false, 0.0, 0};

float currentArmSpeed = 0.0;
float currentBucketSpeed = 0.0;
float distThresh1 = 0.05;
float distThresh2 = 0.10;
float distThresh3 = 0.15;
int noiseThresh = 2;

bool automationGo = false;
bool run = false;


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
void sync(LinearActuator *linear1, LinearActuator *linear2, float currentSpeed){
    float diff = abs(linear1->potentiometer - linear2->potentiometer);
    bool val = (currentSpeed > 0) ? (linear1->potentiometer >= linear2->potentiometer) : (linear1->potentiometer < linear2->potentiometer);
    
    if (diff > ((950 / linear1->stroke)/6)){
        if(val){
            linear1->speed = 0;
        }
        else{
            linear2->speed = 0;
        }
    }
    else if (diff > ((950 / linear1->stroke)/9)){
        if(val){
            linear1->speed *= 0.5;
        }
        else{
            linear2->speed *= 0.5;
        }
    }
    else if (diff > ((950 / linear1->stroke)/12)){
        if(val){
            linear1->speed *= 0.9;
        }
        else{
            linear2->speed *= 0.9;
        }
    }
    else{
        linear1->speed = currentSpeed;
        linear2->speed = currentSpeed;
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
void syncDistance(LinearActuator *linear1, LinearActuator *linear2, float currentSpeed){
    float diff = abs(linear1->distance - linear2->distance);
    bool val = (currentSpeed > 0) ? (linear1->distance >= linear2->distance) : (linear1->distance < linear2->distance);
    if (diff > distThresh3) {
        if(val){
            linear1->speed = 0;
        }
        else{
            linear2->speed = 0;
        }
        if(!linear1->sensorless)
            linear1->error = ActuatorsSyncError;
        if(!linear2->sensorless)
            linear2->error = ActuatorsSyncError;
    }
    else if (diff > distThresh2){
        if(val){
            linear1->speed *= 0.5;
        }
        else{
            linear2->speed *= 0.5;
        }
        if(!linear1->sensorless)
            if(linear1->error == ActuatorsSyncError)
                linear1->error = None;
        if(!linear2->sensorless)
            if(linear2->error == ActuatorsSyncError)
                linear2->error = None;
    }
    else if (diff > distThresh1) {
        if(val){
            linear1->speed *= 0.9;
        }
        else{
            linear2->speed *= 0.9;
        }
        if(!linear1->sensorless)
            if(linear1->error == ActuatorsSyncError)
                linear1->error = None;
        if(!linear2->sensorless)
            if(linear2->error == ActuatorsSyncError)
                linear2->error = None;
    }
    else{
        linear1->speed = currentSpeed;
	    linear2->speed = currentSpeed;
        if(!linear1->sensorless)
            if(linear1->error == ActuatorsSyncError)
                linear1->error = None;
        if(!linear2->sensorless)
            if(linear2->error == ActuatorsSyncError)
                linear2->error = None;
    }
}


void setSpeedAtEnd(LinearActuator *linear1, LinearActuator *linear2, float currentSpeed){
    if((linear1->atMax && currentSpeed > 0) || (linear1->atMin && currentSpeed < 0)){
        linear1->speed = 0.0;
    }
    if((linear2->atMax && currentSpeed > 0) || (linear2->atMin && currentSpeed < 0)){
        linear2->speed = 0.0;
    }
}


/** @brief Function that sets the speeds of the first pair of linear
 * actuators, then syncs the motors. 
 * 
 * The setSpeed function checks if the linear actuators are at the min
 * or max, then sets the speed to 0.0 if either are true.
 * @return void
 * */
void setSpeeds(LinearActuator *linear1, LinearActuator *linear2, float currentSpeed){
    if(!automationGo){
        linear1->speed = currentSpeed;
        linear2->speed = currentSpeed;
    }
    else{
        if(linear1->error != PotentiometerError && linear2->error != PotentiometerError){
            linear1->speed = currentSpeed;
            linear2->speed = currentSpeed;
        }
    }
    if(linear1->error != PotentiometerError && linear2->error != PotentiometerError){
        sync(linear1, linear2, currentSpeed);
        setSpeedAtEnd(linear1, linear2, currentSpeed);
    }
}


/** @brief Function to publish the speeds of the first pair of 
 * actuators to Talons 14 and 15. 
 * 
 * The function creates two new Float32 messages, sets the data to
 * the value of the linear object speed, then publishes the data.
 * @return void
 * */
void publishSpeeds(){
    std_msgs::msg::Float32 speed1;
    speed1.data = linear1.speed;
    talon14Publisher->publish(speed1);
    linear1.previousSpeed = linear1.speed;
    std_msgs::msg::Float32 speed2;
    speed2.data = linear2.speed;
    talon15Publisher->publish(speed2);
    linear2.previousSpeed = linear2.speed;
}


/** @brief Function to publish the speeds of the second pair of 
 * actuators to Talons 16 and 17. 
 * 
 * The function creates two new Float32 messages, sets the data to
 * the value of the linear object speed, then publishes the data.
 * @return void
 * */
void publishSpeeds2(){
    std_msgs::msg::Float32 speed1;
    speed1.data = linear3.speed;
    talon16Publisher->publish(speed1);
    linear3.previousSpeed = linear3.speed;
    std_msgs::msg::Float32 speed2;
    speed2.data = linear4.speed;
    talon17Publisher->publish(speed2);
    linear4.previousSpeed = linear4.speed;
}


/** @brief Function that sets the speeds of the first pair of linear
 * actuators, then syncs the motors.
 * 
 * The setSpeed function checks if the linear actuators are at the min
 * or max, then sets the speed to 0.0 if either are true. The values are
 * published if either is not zero or the currentSpeed is not zero.
 * @return void
 * */
void setSpeedsDistance(LinearActuator *linear1, LinearActuator *linear2, float currentSpeed){
    linear1->speed = currentSpeed;
    linear2->speed = currentSpeed;
    syncDistance(linear1, linear2, currentSpeed);
    setSpeedAtEnd(linear1, linear2, currentSpeed);
    if(linear1->speed != linear1->previousSpeed || linear2->speed != linear2->previousSpeed){
        if(linear1->motorNumber == 14){
            publishSpeeds();
        }
        else{
            publishSpeeds2();
        }
    }
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
void setPotentiometerError(int potentData, LinearActuator *linear){
    if(potentData > 1024){
        linear->error = PotentiometerError;
        linear->sensorless = true;
        RCLCPP_INFO(nodeHandle->get_logger(),"EXCAVATION ERROR: PotentiometerError");
    }
    if(potentData > 110 || potentData < 100){
        if(linear->error == PotentiometerError){
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
 * between 100 and 110. If the 
 * @param potentData - Int value of potentiometer
 * @param *linear - Pointer to linear object
 * @return void
 * */
void processPotentiometerData(int potentData, LinearActuator *linear){
    if(potentData < linear->min){
        linear->min = potentData;
    }

    if(potentData > linear->max){
        linear->max = potentData;
    }
    if(potentData > 110 || potentData < 100){
        linear->initialized = true;
        linear->distance = linear->stroke * (potentData / 950);
    }
    if(abs(linear->potentiometer - potentData) > 50 && (potentData >= 100 && potentData <= 110) && linear->initialized){
        linear->sensorless = true;
	    linear->error = PotentiometerError;
    }
    if(linear->potentiometer >= potentData - noiseThresh && linear->potentiometer <= potentData + noiseThresh){
        if(linear->speed != 0.0 && run){
            linear->timeWithoutChange += 1;
            if(linear->timeWithoutChange >= 15){
                if(linear->potentiometer >= 100 && linear->potentiometer <= 110 && !linear->initialized){
                    linear->sensorless = true;
                    linear->error = PotentiometerError;
                }
                else if(linear->max > 800 && linear->speed > 0.0 && potentData >= linear->max - 20){
                    linear->atMax = true;
                    linear->timeWithoutChange = 0;
                }
                else if(linear->min < 200 && linear->speed < 0.0 && potentData <= linear->min + 20){
                    linear->atMin = true;
                    linear->timeWithoutChange = 0;
                }
                else{
                    if(linear->error == None || linear->error == ActuatorsSyncError){
                        linear->error = ActuatorNotMovingError;
                        RCLCPP_INFO(nodeHandle->get_logger(),"EXCAVATION ERROR: ActuatorNotMovingError");
                    }
                }
            }
        }
    }
    else{
        linear->timeWithoutChange = 0;
        if(linear->error == ActuatorNotMovingError){
            linear->error = None;
        }
        if(linear->atMax){
            if(linear->speed < 0.0){
                linear->atMax = false;
            }
        }
        else{
            linear->atMax = false;
        }
        if(linear->atMin){
            if(linear->speed > 0.0){
                linear->atMin = false;
            }
        }
        else{
            linear->atMin = false;
        }
    }
    linear->potentiometer = potentData;
    if(linear->motorNumber == 16 || linear->motorNumber == 17){
        if(potentData > 700){
            linear->atMax = true;
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
void setSyncErrors(LinearActuator *linear1, LinearActuator *linear2, float currentSpeed){
    if(abs(linear1->potentiometer - linear2->potentiometer) > ((950 / linear1->stroke) / 6)){
        if(linear1->potentiometer >= 100 && linear1->potentiometer <= 110 && !linear1->initialized){
            linear1->error = PotentiometerError;
            linear1->sensorless = true;
        }
        if(linear2->potentiometer >= 100 && linear2->potentiometer <= 110 && !linear2->initialized){
            linear2->error = PotentiometerError;
            linear2->sensorless = true;
        }
        if(linear1->error == None){
            linear1->error = ActuatorsSyncError;
        }
        if(linear2->error == None){
            linear2->error = ActuatorsSyncError;
        }
    }
    else{
        if(linear1->error == ActuatorsSyncError){
            linear1->error = None;
        }
        if(linear2->error == ActuatorsSyncError){
            linear2->error = None;
        }
    }
    if(linear1->error != PotentiometerError && linear2->error != PotentiometerError){
        sync(linear1, linear2, currentSpeed);
        if(linear1->speed != linear1->previousSpeed || linear2->speed != linear2->previousSpeed){
            if(linear1->motorNumber == 14){
                publishSpeeds();
            }
            else{
                publishSpeeds2();
            }
        }
    }
}

/*
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
    if(!linear1.sensorless){
        setPotentiometerError(msg->sensor_position, &linear1);

        if(linear1.error != PotentiometerError){
            processPotentiometerData(msg->sensor_position, &linear1);
            if(!single_arm){
                if(!linear1.sensorless && !linear2.sensorless){
                    setSyncErrors(&linear1, &linear2, currentArmSpeed);
                }
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
    linear2.maxCurrent = msg->max_current;
    if(!linear2.sensorless){
        setPotentiometerError(msg->sensor_position, &linear2);

        if(linear2.error != PotentiometerError){
            processPotentiometerData(msg->sensor_position, &linear2);
            if(!single_arm){
                if(!linear1.sensorless && !linear2.sensorless){
                    setSyncErrors(&linear1, &linear2, currentArmSpeed);
                }
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
    if(!linear3.sensorless){
        setPotentiometerError(msg->sensor_position, &linear3);

        if(linear3.error != PotentiometerError){
            processPotentiometerData(msg->sensor_position, &linear3);
            if(!single_arm){
                if(!linear3.sensorless && !linear4.sensorless){
                    setSyncErrors(&linear3, &linear4, currentBucketSpeed);
                }
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
    linear4.maxCurrent = msg->max_current;
    if(!linear4.sensorless){
        setPotentiometerError(msg->sensor_position, &linear4);

        if(linear4.error != PotentiometerError){
            processPotentiometerData(msg->sensor_position, &linear4);
            if(!single_arm){
                if(!linear3.sensorless && !linear4.sensorless){
                    setSyncErrors(&linear3, &linear4, currentBucketSpeed);
                }
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
    RCLCPP_INFO(nodeHandle->get_logger(),"currentArmSpeed: %f", currentArmSpeed);
    if(!single_arm){
        setSpeeds(&linear1, &linear2, currentArmSpeed);
    }
    else{
        linear1.speed = speed->data;
    }
    publishSpeeds();
    RCLCPP_INFO(nodeHandle->get_logger(),"Arm speeds: %f, %f", linear1.speed, linear2.speed);
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
    RCLCPP_INFO(nodeHandle->get_logger(),"currentSpeed: %f", currentBucketSpeed);
    if(!single_arm){
        setSpeeds(&linear3, &linear4, currentBucketSpeed);
    }
    else{
        linear3.speed = speed->data;
    }
    publishSpeeds2();
    RCLCPP_INFO(nodeHandle->get_logger(),"Bucket speeds: %f, %f", linear3.speed, linear4.speed);

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
    updateMotorPosition(millis, &linear1);
    //RCLCPP_INFO(nodeHandle->get_logger(), "Linear 1 Distance: %f", linear1.distance);

    updateMotorPosition(millis, &linear2);
    //RCLCPP_INFO(nodeHandle->get_logger(), "Linear 2 Distance: %f", linear2.distance);
    
    if(linear1.sensorless || linear2.sensorless)
        setSpeedsDistance(&linear1, &linear2, currentArmSpeed);
    
    updateMotorPosition(millis, &linear3);
    //RCLCPP_INFO(nodeHandle->get_logger(), "Linear 3 Distance: %f", linear3.distance);

    updateMotorPosition(millis, &linear4);
    //RCLCPP_INFO(nodeHandle->get_logger(), "Linear 4 Distance: %f", linear4.distance);
    
    if(linear3.sensorless || linear4.sensorless)
        setSpeedsDistance(&linear3, &linear4, currentBucketSpeed);
}


/** @brief Function to get the value of the specified parameter
 * 
 * Function that takes a string as a parameter containing the
 * name of the parameter that is being parsed from the launch
 * file and the initial value of the parameter as inputs, then
 * gets the parameter, casts it as the desired type, displays 
 * the value of the parameter on the command line and the log 
 * file, then returns the parsed value of the parameter.
 * @param parametername String of the name of the parameter
 * @param initialValue Initial value of the parameter
 * @return value Value of the parameter
 * */
template <typename T>
T getParameter(std::string parameterName, T initialValue){
	nodeHandle->declare_parameter<T>(parameterName, initialValue);
	rclcpp::Parameter param = nodeHandle->get_parameter(parameterName);
	T value = param.template get_value<T>();
	std::cout << parameterName << ": " << value << std::endl;
	RCLCPP_INFO(nodeHandle->get_logger(), param.value_to_string().c_str());
	return value;
}

template <typename T>
T getParameter(const std::string& parameterName, const char* initialValue){
	return getParameter<T>(parameterName, std::string(initialValue));
}


int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    nodeHandle = rclcpp::Node::make_shared("excavation");

    single_arm = getParameter<bool>("single_arm", false);

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
    
    messages::msg::LinearStatus linearStatus1;
    messages::msg::LinearStatus linearStatus2;
    messages::msg::LinearStatus linearStatus3;
    messages::msg::LinearStatus linearStatus4;

    auto linearStatus1Publisher = nodeHandle->create_publisher<messages::msg::LinearStatus>("linearStatus1",1);
    auto linearStatus2Publisher = nodeHandle->create_publisher<messages::msg::LinearStatus>("linearStatus2",1);
    auto linearStatus3Publisher = nodeHandle->create_publisher<messages::msg::LinearStatus>("linearStatus3",1);
    auto linearStatus4Publisher = nodeHandle->create_publisher<messages::msg::LinearStatus>("linearStatus4",1);

    auto start = std::chrono::high_resolution_clock::now();
    auto finish = std::chrono::high_resolution_clock::now();
    rclcpp::Rate rate(60);
    while(rclcpp::ok()){
        finish = std::chrono::high_resolution_clock::now();
        if(std::chrono::duration_cast<std::chrono::milliseconds>(finish-start).count() > 33){
            updateMotorPositions(std::chrono::duration_cast<std::chrono::milliseconds>(finish-start).count() );
            getLinearStatus(&linearStatus1, &linear1);
            linearStatus1Publisher->publish(linearStatus1);

            if(!single_arm){
                getLinearStatus(&linearStatus2, &linear2);
                linearStatus2Publisher->publish(linearStatus2);
            }
            
            getLinearStatus(&linearStatus3, &linear3);
            linearStatus3Publisher->publish(linearStatus3);

            if(!single_arm){
                getLinearStatus(&linearStatus4, &linear4);
                linearStatus4Publisher->publish(linearStatus4);
            }
            start = std::chrono::high_resolution_clock::now();
        }
        rate.sleep();
        rclcpp:spin_some(nodeHandle);
    }
}
