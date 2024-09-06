#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/empty.hpp>

#include "messages/msg/linear_out.hpp"
#include "messages/msg/talon_out.hpp"

rclcpp::Node::SharedPtr nodeHandle;

std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > talon14Publisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > talon15Publisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > talon16Publisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > talon17Publisher;

messages::msg::LinearOut linearOut1;
messages::msg::LinearOut linearOut2;
messages::msg::LinearOut linearOut3;
messages::msg::LinearOut linearOut4;

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
 * \li \b linearOut1
 * \li \b linearOut2
 * \li \b linearOut3
 * \li \b linearOut4
 * 
 */


enum Error {
    ActuatorsSyncError,
    ActuatorNotMovingError,
    PotentiometerError,
    MotorConnectionError,
    None
};


std::map<Error, const char*> errorMap = {{ActuatorsSyncError, "ActuatorsSyncError"},
    {ActuatorNotMovingError, "ActuatorNotMovingError"},
    {PotentiometerError, "PotentiometerError"},
    {MotorConnectionError, "MotorConnectionError"},
    {None, "None"}};


struct LinearActuator{
    int motorNumber = 0;
    float speed = 0.0;              // Speed variable of linear actuator
    int potentiometer = 0;          // Potentiometer reading
    int timeWithoutChange = 0;      // Number of potentiometer values received without change when speed > 0
    int max = 0;                    // Max potentiometer value
    int min = 1024;                 // Min potentiometer value
    Error error = None;  // Error state of the actuator
    bool atMin = false;             // Bool value of if actuator is at min extension
    bool atMax = false;             // Bool value of if actuator is at max extension
    float stroke = 11.8;            // Length of stroke of the actuator
    float distance = 0.0;           // Distance extended
    float extensionSpeed = 0.0;     // Speed of extension in in/sec
    float timeToExtend = 0.0;       // Time to fully extend actuator
    bool sensorless = false;
    float maxCurrent = 0.0;
    bool initialized = false;
    float previous = 0.0;
};


LinearActuator linear1{14, 0.0, 0, 0, 0, 1024, None, false, false, 9.8, 0.0, 0.85, 11.5, false, 0.0, false, 0.0};
LinearActuator linear2{15, 0.0, 0, 0, 0, 1024, None, false, false, 9.8, 0.0, 0.89, 11.0, false, 0.0, false, 0.0};
LinearActuator linear3{16, 0.0, 0, 0, 0, 1024, None, false, false, 5.9, 0.0, 0.69, 8.5, false, 0.0, false, 0.0};
LinearActuator linear4{17, 0.0, 0, 0, 0, 1024, None, false, false, 5.9, 0.0, 0.69, 8.5, false, 0.0, false, 0.0};

float currentSpeed = 0.0;
float currentSpeed2 = 0.0;
float distThresh1 = 0.05;
float distThresh2 = 0.10;
float distThresh3 = 0.15;

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
    linear1.previous = linear1.speed;
    std_msgs::msg::Float32 speed2;
    speed2.data = linear2.speed;
    talon15Publisher->publish(speed2);
    linear2.previous = linear2.speed;
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
    linear3.previous = linear3.speed;
    std_msgs::msg::Float32 speed2;
    speed2.data = linear4.speed;
    talon17Publisher->publish(speed2);
    linear4.previous = linear4.speed;
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
    if(linear1->speed != linear1->previous || linear2->speed != linear2->previous){
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
    }
    if(abs(linear->potentiometer - potentData) > 50 && (potentData >= 100 && potentData <= 110)){
        linear->sensorless = true;
	    linear->error = PotentiometerError;
    }
    if(linear->potentiometer >= potentData - 5 && linear->potentiometer <= potentData + 5){
        if(linear->speed != 0.0 && run){
            linear->timeWithoutChange += 1;
            if(linear->timeWithoutChange >= 3){
                if(linear->potentiometer >= 100 && linear->potentiometer <= 110){
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
        if(linear1->speed != linear1->previous || linear2->speed != linear2->previous){
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
void potentiometer1Callback(const messages::msg::TalonOut::SharedPtr msg){
    linear1.maxCurrent = msg->max_current;
    if(!linear1.sensorless){
        setPotentiometerError(msg->sensor_position, &linear1);

        if(linear1.error != PotentiometerError){
            processPotentiometerData(msg->sensor_position, &linear1);
            if(!linear1.sensorless && !linear2.sensorless){
                setSyncErrors(&linear1, &linear2, currentSpeed);
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
void potentiometer2Callback(const messages::msg::TalonOut::SharedPtr msg){
    linear2.maxCurrent = msg->max_current;
    if(!linear2.sensorless){
        setPotentiometerError(msg->sensor_position, &linear2);

        if(linear2.error != PotentiometerError){
            processPotentiometerData(msg->sensor_position, &linear2);
            if(!linear1.sensorless && !linear2.sensorless){
                setSyncErrors(&linear1, &linear2, currentSpeed);
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
void potentiometer3Callback(const messages::msg::TalonOut::SharedPtr msg){
    linear3.maxCurrent = msg->max_current;
    if(!linear3.sensorless){
        setPotentiometerError(msg->sensor_position, &linear3);

        if(linear3.error != PotentiometerError){
            processPotentiometerData(msg->sensor_position, &linear3);
            if(!linear3.sensorless && !linear4.sensorless){
                setSyncErrors(&linear3, &linear4, currentSpeed2);
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
void potentiometer4Callback(const messages::msg::TalonOut::SharedPtr msg){
    linear4.maxCurrent = msg->max_current;
    if(!linear4.sensorless){
        setPotentiometerError(msg->sensor_position, &linear4);

        if(linear4.error != PotentiometerError){
            processPotentiometerData(msg->sensor_position, &linear4);
            if(!linear3.sensorless && !linear4.sensorless){
                setSyncErrors(&linear3, &linear4, currentSpeed2);
            }
        }
    }
}


/** @brief Callback function for the armSpeed topic. 
 * 
 * This function sets the currentSpeed variable to the value contained in the 
 * speed->data. The speeds are set using the setSpeeds function and published
 * using the publishSpeeds function.
 * @param speed - ROS2 message containing speed value for the arm linear actuators
 * @return void
 * */
void armSpeedCallback(const std_msgs::msg::Float32::SharedPtr speed){
    currentSpeed = speed->data;
    RCLCPP_INFO(nodeHandle->get_logger(),"currentSpeed: %f", currentSpeed);
    setSpeeds(&linear1, &linear2, currentSpeed);
    publishSpeeds();
    RCLCPP_INFO(nodeHandle->get_logger(),"Arm speeds: %f, %f", linear1.speed, linear2.speed);
}


/** @brief Callback function for the bucketSpeed topic. 
 * 
 * This function sets the currentSpeed2 variable to the value contained in the 
 * speed->data. The speeds are set using the setSpeeds2 function and published
 * using the publishSpeeds2 function.
 * @param speed - ROS2 message containing speed value for the arm linear actuators
 * @return void
 * */
void bucketSpeedCallback(const std_msgs::msg::Float32::SharedPtr speed){
    currentSpeed2 = speed->data;
    RCLCPP_INFO(nodeHandle->get_logger(),"currentSpeed: %f", currentSpeed2);
    setSpeeds(&linear3, &linear4, currentSpeed2);
    publishSpeeds2();
    RCLCPP_INFO(nodeHandle->get_logger(),"Bucket speeds: %f, %f", linear3.speed, linear4.speed);

}


/** @brief Function to get the LinearOut values
 * 
 * This function sets the values of the LinearOut message
 * with the values from the linear actuator. 
 * @param *linearOut - Pointer for the LinearOut object
 * @param *linear - Pointer for the linear actuator
 * @return void
 * */
void getLinearOut(messages::msg::LinearOut *linearOut, LinearActuator *linear){
    linearOut->motor_number = linear->motorNumber;
    linearOut->speed = linear->speed;
    linearOut->potentiometer = linear->potentiometer;
    linearOut->time_without_change = linear->timeWithoutChange;
    linearOut->max = linear->max;
    linearOut->min = linear->min;
    linearOut->error = errorMap.at(linear->error);
    linearOut->at_min = linear->atMin;
    linearOut->at_max = linear->atMax;
    linearOut->distance = linear->distance;
    linearOut->sensorless = linear->sensorless;
    linearOut->stroke = linear->stroke;
    linearOut->extension_speed = linear->extensionSpeed;
    linearOut->time_to_extend = linear->timeToExtend;
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
        setSpeedsDistance(&linear1, &linear2, currentSpeed);
    
    updateMotorPosition(millis, &linear3);
    //RCLCPP_INFO(nodeHandle->get_logger(), "Linear 3 Distance: %f", linear3.distance);

    updateMotorPosition(millis, &linear4);
    //RCLCPP_INFO(nodeHandle->get_logger(), "Linear 4 Distance: %f", linear4.distance);
    
    if(linear3.sensorless || linear4.sensorless)
        setSpeedsDistance(&linear3, &linear4, currentSpeed2);
}


int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    nodeHandle = rclcpp::Node::make_shared("excavation");

    auto automationGoSubscriber = nodeHandle->create_subscription<std_msgs::msg::Bool>("automationGo",1,automationGoCallback);
    auto stopSubscriber = nodeHandle->create_subscription<std_msgs::msg::Empty>("STOP",1,stopCallback);
    auto goSubscriber = nodeHandle->create_subscription<std_msgs::msg::Empty>("GO",1,goCallback);

    auto armSpeedSubscriber = nodeHandle->create_subscription<std_msgs::msg::Float32>("arm_speed",1,armSpeedCallback);
    auto bucketSpeedSubscriber = nodeHandle->create_subscription<std_msgs::msg::Float32>("bucket_speed",1,bucketSpeedCallback);

    auto talon1Subscriber = nodeHandle->create_subscription<messages::msg::TalonOut>("talon_14_info",1,potentiometer1Callback);
    auto talon2Subscriber = nodeHandle->create_subscription<messages::msg::TalonOut>("talon_15_info",1,potentiometer2Callback);
    auto talon3Subscriber = nodeHandle->create_subscription<messages::msg::TalonOut>("talon_16_info",1,potentiometer3Callback);
    auto talon4Subscriber = nodeHandle->create_subscription<messages::msg::TalonOut>("talon_17_info",1,potentiometer4Callback);

    talon14Publisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("talon_14_speed",1);
    talon15Publisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("talon_15_speed",1);
    talon16Publisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("talon_16_speed",1);
    talon17Publisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("talon_17_speed",1);
    
    messages::msg::LinearOut linearOut1;
    messages::msg::LinearOut linearOut2;
    messages::msg::LinearOut linearOut3;
    messages::msg::LinearOut linearOut4;

    auto linearOut1Publisher = nodeHandle->create_publisher<messages::msg::LinearOut>("linearOut1",1);
    auto linearOut2Publisher = nodeHandle->create_publisher<messages::msg::LinearOut>("linearOut2",1);
    auto linearOut3Publisher = nodeHandle->create_publisher<messages::msg::LinearOut>("linearOut3",1);
    auto linearOut4Publisher = nodeHandle->create_publisher<messages::msg::LinearOut>("linearOut4",1);

    auto start = std::chrono::high_resolution_clock::now();
    auto finish = std::chrono::high_resolution_clock::now();
    rclcpp::Rate rate(20);
    while(rclcpp::ok()){
        finish = std::chrono::high_resolution_clock::now();
        if(std::chrono::duration_cast<std::chrono::milliseconds>(finish-start).count() > 33){
            updateMotorPositions(std::chrono::duration_cast<std::chrono::milliseconds>(finish-start).count() );
            getLinearOut(&linearOut1, &linear1);
            linearOut1Publisher->publish(linearOut1);

            getLinearOut(&linearOut2, &linear2);
            linearOut2Publisher->publish(linearOut2);

            getLinearOut(&linearOut3, &linear3);
            linearOut3Publisher->publish(linearOut3);

            getLinearOut(&linearOut4, &linear4);
            linearOut4Publisher->publish(linearOut4);
            start = std::chrono::high_resolution_clock::now();
        }
        rate.sleep();
        rclcpp:spin_some(nodeHandle);
    }
}
