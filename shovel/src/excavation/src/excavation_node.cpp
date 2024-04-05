#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/empty.hpp>

#include "messages/msg/linear_out.hpp"

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
 * \li \b sensorless
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
    ConnectionError,
    None
};


std::map<Error, const char*> errorMap = {{ActuatorsSyncError, "ActuatorsSyncError"},
    {ActuatorNotMovingError, "ActuatorNotMovingError"},
    {PotentiometerError, "PotentiometerError"},
    {ConnectionError, "ConnectionError"},
    {None, "None"}};


struct LinearActuator{
    int motorNumber = 0;
    float speed = 0.0;              // Speed variable of linear actuator
    int potentiometer = 0;          // Potentiometer reading
    int timeWithoutChange = 0;      // Number of potentiometer values received without change when speed > 0
    int max = 0;                    // Max potentiometer value
    int min = 1024;                 // Min potentiometer value
    Error error = ConnectionError;  // Error state of the actuator
    bool run = true;                 
    bool atMin = false;             // Bool value of if actuator is at min extension
    bool atMax = false;             // Bool value of if actuator is at max extension
    float stroke = 11.8;            // Length of stroke of the actuator
    float distance = 0.0;           // Distance extended
    float extensionSpeed = 0.0;     // Speed of extension in in/sec
    float timeToExtend = 0.0;       // Time to fully extend actuator
};


LinearActuator linear1{14, 0.0, 0, 0, 0, 1024, ConnectionError, true, false, false, 5.9, 0.0, 0.69, 8.5};
LinearActuator linear2{15, 0.0, 0, 0, 0, 1024, ConnectionError, true, false, false, 5.9, 0.0, 0.69, 8.5};
LinearActuator linear3{16, 0.0, 0, 0, 0, 1024, ConnectionError, true, false, false, 9.8, 0.0, 0.85, 11.5};
LinearActuator linear4{17, 0.0, 0, 0, 0, 1024, ConnectionError, true, false, false, 9.8, 0.0, 0.89, 11.0};

float currentSpeed = 0.0;
float currentSpeed2 = 0.0;
int thresh1 = 15;
int thresh2 = 30;
int thresh3 = 45;
float distThresh1 = 0.1;
float distThresh2 = 0.2;
float distThresh3 = 0.3;

bool automationGo = false;
bool sensorless = false;


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
void sync(){
    float diff = abs(linear1.potentiometer - linear2.potentiometer);
    // Might change this from ternary to if statements to improve readability
    bool val = (currentSpeed > 0) ? (linear1.potentiometer > linear2.potentiometer) : (linear1.potentiometer < linear2.potentiometer);
    if (diff > thresh3){
        (val) ? linear1.speed = 0 : linear2.speed = 0;
    }
    else if (diff > thresh2){
        (val) ? linear1.speed *= 0.5 : linear2.speed *= 0.5;
    }
    else if (diff > thresh1){
        (val) ? linear1.speed *= 0.9 : linear2.speed *= 0.9;
    }
    else{
        linear1.speed = currentSpeed;
        linear2.speed = currentSpeed;
    }
}


/** @brief Function to sync the second pair of linear actuators. 
 * 
 * The sync function works by checking if the currentSpeed2 is
 * greater than zero. If the speed is greater than zero, the val
 * checks which actuator is more extended and sets the speed of
 * the actuator to a lower value if the diff is greater than the 
 * thresh values.  If the value is less than zero, the val checks 
 * which actuator is less extended and sets the speed of the 
 * actuator to a lower value. This was added to eliminate memory
 * issues arising from earlier functions that used pointers and 
 * had unexpected behavior. Future updates may remove this.
 * @return void
 * */
void sync2(){
    float diff = abs(linear3.potentiometer - linear4.potentiometer);
    // Might change this from ternary to if statements to improve readability
    bool val = (currentSpeed2 > 0) ? (linear3.potentiometer > linear4.potentiometer) : (linear3.potentiometer < linear4.potentiometer);
    if (diff > thresh3){
        (val) ? linear3.speed = 0 : linear4.speed = 0;
    }
    else if (diff > thresh2){
        (val) ? linear3.speed *= 0.5 : linear4.speed *= 0.5;
    }
    else if (diff > thresh1){
        (val) ? linear3.speed *= 0.9 : linear4.speed *= 0.9;
    }
    else{
        linear3.speed = currentSpeed2;
        linear4.speed = currentSpeed2;
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
void syncDistance(){
    float diff = abs(linear1.distance - linear2.distance);
    bool val = (currentSpeed > 0) ? (linear1.distance > linear2.distance) : (linear1.distance < linear2.distance);
    if (diff > distThresh3) {
        (val) ? linear1.speed = 0 : linear2.speed = 0;
    }
    else if (diff > distThresh2){
        (val) ? linear1.speed *= 0.5 : linear2.speed *= 0.5;
    }
    else if (diff > distThresh1) {
        (val) ? linear1.speed *= 0.9 : linear2.speed *= 0.9;
    }
    else{
        linear1.speed = currentSpeed;
	    linear2.speed = currentSpeed;
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
void syncDistance2(){
    float diff = abs(linear3.distance - linear4.distance);
    bool val = (currentSpeed > 0) ? (linear3.distance > linear4.distance) : (linear3.distance < linear4.distance);
    if (diff > distThresh3) {
        (val) ? linear3.speed = 0 : linear4.speed = 0;
    }
    else if (diff > distThresh2){
        (val) ? linear3.speed *= 0.5 : linear4.speed *= 0.5;
    }
    else if (diff > distThresh1){
        (val) ? linear3.speed *= 0.9 : linear4.speed *= 0.9;
    }
    else{
        linear3.speed = currentSpeed2;
	    linear4.speed = currentSpeed2;
    }
}


void setSpeedAtEnd(){
    if((linear1.atMax && currentSpeed > 0) || (linear1.atMin && currentSpeed < 0)){
        linear1.speed = 0.0;
    }
    if((linear2.atMax && currentSpeed > 0) || (linear2.atMin && currentSpeed < 0)){
        linear2.speed = 0.0;
    }
}


void setSpeedAtEnd2(){
    if((linear3.atMax && currentSpeed2 > 0) || (linear3.atMin && currentSpeed2 < 0)){
        linear3.speed = 0.0;
    }
    if((linear4.atMax && currentSpeed2 > 0) || (linear4.atMin && currentSpeed2 < 0)){
        linear4.speed = 0.0;
    }
}


/** @brief Function that sets the speeds of the first pair of linear
 * actuators, then syncs the motors. 
 * 
 * The setSpeed function checks if the linear actuators are at the min
 * or max, then sets the speed to 0.0 if either are true.
 * @return void
 * */
void setSpeeds(){
    if(!automationGo){
        linear1.speed = currentSpeed;
        linear2.speed = currentSpeed;
    }
    else{
        if(linear1.error != ConnectionError && linear1.error != PotentiometerError && linear2.error != ConnectionError && linear2.error != PotentiometerError){
            linear1.speed = currentSpeed;
            linear2.speed = currentSpeed;
        }
    }
    if(linear1.error != ConnectionError && linear1.error != PotentiometerError && linear2.error != ConnectionError && linear2.error != PotentiometerError){
        sync();
        setSpeedAtEnd();
    }
}


/** @brief Function that sets the speeds of the second pair of linear
 * actuators, then syncs the motors. 
 * 
 * The setSpeed function checks if the linear actuators are at the min
 * or max, then sets the speed to 0.0 if either are true.
 * @return void
 * */
void setSpeeds2(){
    if(!automationGo){
        linear3.speed = currentSpeed2;
        linear4.speed = currentSpeed2;
    }
    else{
        if(linear3.error != ConnectionError && linear3.error != PotentiometerError && linear4.error != ConnectionError && linear4.error != PotentiometerError){
            linear3.speed = currentSpeed2;
            linear4.speed = currentSpeed2;
        }
    }
    if(linear3.error != ConnectionError && linear3.error != PotentiometerError && linear4.error != ConnectionError && linear4.error != PotentiometerError){
        sync2();
        setSpeedAtEnd2();
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
    std_msgs::msg::Float32 speed2;
    speed2.data = linear2.speed;
    talon15Publisher->publish(speed2);
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
    std_msgs::msg::Float32 speed2;
    speed2.data = linear4.speed;
    talon17Publisher->publish(speed2);
}


/** @brief Function that sets the speeds of the first pair of linear
 * actuators, then syncs the motors.
 * 
 * The setSpeed function checks if the linear actuators are at the min
 * or max, then sets the speed to 0.0 if either are true. The values are
 * published if either is not zero or the currentSpeed is not zero.
 * @return void
 * */
void setSpeedsDistance(){
    syncDistance();
    setSpeedAtEnd();
    if(linear1.speed != 0.0 || linear2.speed != 0.0 || currentSpeed != 0.0){
        publishSpeeds();
    }
}


/** @brief Function that sets the speeds of the second pair of linear
 * actuators, then syncs the motors.
 * 
 * The setSpeed function checks if the linear actuators are at the min
 * or max, then sets the speed to 0.0 if either are true. The values are
 * published if either is not zero or the currentSpeed2 is not zero.
 * @return void
 * */
void setSpeedsDistance2(){
    syncDistance2();
    setSpeedAtEnd2();
    if(linear3.speed != 0 || linear4.speed != 0 || currentSpeed2 != 0.0){
        publishSpeeds2();
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
    if(potentData == -1){
        linear->error = ConnectionError;
    }
    else{
        if(linear->error == ConnectionError){
            linear->error = None;
        }
    }
    if(potentData == 1024){
        linear->error = PotentiometerError;
        RCLCPP_INFO(nodeHandle->get_logger(),"EXCAVATION ERROR: PotentiometerError");
    }
    else{
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

    if(linear->potentiometer >= potentData - 10 && linear->potentiometer <= potentData + 10){
        if(linear->speed != 0.0){
            linear->timeWithoutChange += 1;
            if(linear->timeWithoutChange >= 5){
                if(linear->max > 800 && linear->speed > 0.0 && potentData >= linear->max - 20){
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
void setSyncErrors(){
    if(abs(linear1.potentiometer - linear2.potentiometer) > thresh1){
        if(linear1.error == None){
            linear1.error = ActuatorsSyncError;
        }
        if(linear2.error == None){
            linear2.error = ActuatorsSyncError;
        }
    }
    else{
        if(linear1.error == ActuatorsSyncError){
            linear1.error = None;
        }
        if(linear2.error == ActuatorsSyncError){
            linear2.error = None;
        }
    }
    sync();
    if(linear1.speed != 0 || linear2.speed != 0){
        publishSpeeds();
    }
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
void setSyncErrors2(){
    if(abs(linear3.potentiometer - linear4.potentiometer) > thresh1){
        if(linear3.error == None){
            linear3.error = ActuatorsSyncError;
        }
        if(linear4.error == None){
            linear4.error = ActuatorsSyncError;
        }
    }
    else{
        if(linear3.error == ActuatorsSyncError){
            linear3.error = None;
        }
        if(linear4.error == ActuatorsSyncError){
            linear4.error = None;
        }
    }
    sync2();
    if(linear3.speed != 0 || linear4.speed != 0){
        publishSpeeds2();
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
void potentiometer1Callback(const std_msgs::msg::Int32::SharedPtr msg){
    if(!sensorless){
        setPotentiometerError(msg->data, &linear1);

        if(linear1.error != ConnectionError && linear1.error != PotentiometerError && linear2.error != ConnectionError && linear2.error != PotentiometerError){
            processPotentiometerData(msg->data, &linear1);
            setSyncErrors();
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
void potentiometer2Callback(const std_msgs::msg::Int32::SharedPtr msg){
    if(!sensorless){
        setPotentiometerError(msg->data, &linear2);

        if(linear1.error != ConnectionError && linear1.error != PotentiometerError && linear2.error != ConnectionError && linear2.error != PotentiometerError){
            processPotentiometerData(msg->data, &linear2);
            setSyncErrors();
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
void potentiometer3Callback(const std_msgs::msg::Int32::SharedPtr msg){
    if(!sensorless){
        setPotentiometerError(msg->data, &linear3);

        if(linear3.error != ConnectionError && linear3.error != PotentiometerError){
            processPotentiometerData(msg->data, &linear3);
            setSyncErrors2();
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
void potentiometer4Callback(const std_msgs::msg::Int32::SharedPtr msg){
    if(!sensorless){
        setPotentiometerError(msg->data, &linear4);

        if(linear4.error != ConnectionError && linear4.error != PotentiometerError){
            processPotentiometerData(msg->data, &linear4);
            setSyncErrors2();
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
    setSpeeds();
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
    setSpeeds2();
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
    linearOut->run = linear->run;
    linearOut->at_min = linear->atMin;
    linearOut->at_max = linear->atMax;
    linearOut->distance = linear->distance;
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
    linear1.distance = linear1.speed * linear1.extensionSpeed * (millis / 1000.0) + linear1.distance;
    if(linear1.distance > linear1.stroke){
        linear1.distance = linear1.stroke;
        linear1.atMax = true;
    }
    else if(linear1.distance < 0.0){
        linear1.distance = 0.0;
        linear1.atMin = true;
    }
    else{
        linear1.atMin = false;
        linear1.atMax = false;
    }
    RCLCPP_INFO(nodeHandle->get_logger(), "Linear 1 Distance: %f", linear1.distance);

    linear2.distance = linear2.speed * linear2.extensionSpeed * (millis / 1000.0) + linear2.distance;
    if(linear2.distance > linear2.stroke){
        linear2.distance = linear2.stroke;
        linear2.atMax = true;
    }
    else if(linear2.distance < 0.0){
        linear2.distance = 0.0;
        linear2.atMin = true;
    }
    else{
        linear2.atMin = false;
        linear2.atMax = false;
    }
    RCLCPP_INFO(nodeHandle->get_logger(), "Linear 1 Distance: %f", linear1.distance);
    setSpeedsDistance();

    linear3.distance = linear3.speed * linear3.extensionSpeed * (millis / 1000.0) + linear3.distance;
    if(linear3.distance > linear3.stroke){
        linear3.distance = linear3.stroke;
        linear3.atMax = true;
    }
    else if(linear3.distance < 0.0){
        linear3.distance = 0.0;
        linear3.atMin = true;
    }
    else{
        linear3.atMin = false;
        linear3.atMax = false;
    }
    RCLCPP_INFO(nodeHandle->get_logger(), "Linear 1 Distance: %f", linear1.distance);

    linear4.distance = linear4.speed * linear4.extensionSpeed * (millis / 1000.0) + linear4.distance;
    if(linear4.distance > linear4.stroke){
        linear4.distance = linear4.stroke;
        linear4.atMax = true;
    }
    else if(linear4.distance < 0.0){
        linear4.distance = 0.0;
        linear4.atMin = true;
    }
    else{
        linear4.atMin = false;
        linear4.atMax = false;
    }
    RCLCPP_INFO(nodeHandle->get_logger(), "Linear 1 Distance: %f", linear1.distance);
    setSpeedsDistance2();
}


/** @brief Callback function for the sensorless topic. 
 * 
 * This function sets the sensorless value to the opposite value when
 * this message is received by the node.
 * @param msg - ROS2 message containing sensorless value
 * @return void
 * */
void sensorlessCallback(const std_msgs::msg::Empty::SharedPtr empty){
    sensorless = !sensorless;
    RCLCPP_INFO(nodeHandle->get_logger(), "Excavation sensorless mode: %d", sensorless);
}


int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    nodeHandle = rclcpp::Node::make_shared("excavation");

    auto automationGoSubscriber = nodeHandle->create_subscription<std_msgs::msg::Bool>("automationGo",1,automationGoCallback);
    auto sensorlessSubscriber = nodeHandle->create_subscription<std_msgs::msg::Empty>("sensorless",1,sensorlessCallback);

    auto armSpeedSubscriber = nodeHandle->create_subscription<std_msgs::msg::Float32>("arm_speed",1,armSpeedCallback);
    auto bucketSpeedSubscriber = nodeHandle->create_subscription<std_msgs::msg::Float32>("bucket_speed",1,bucketSpeedCallback);

    auto potentiometerDataSubscriber1 = nodeHandle->create_subscription<std_msgs::msg::Int32>("potentiometer_1_data",1,potentiometer1Callback);
    auto potentiometerDataSubscriber2 = nodeHandle->create_subscription<std_msgs::msg::Int32>("potentiometer_2_data",1,potentiometer2Callback);
    auto potentiometerDataSubscriber3 = nodeHandle->create_subscription<std_msgs::msg::Int32>("potentiometer_3_data",1,potentiometer3Callback);
    auto potentiometerDataSubscriber4 = nodeHandle->create_subscription<std_msgs::msg::Int32>("potentiometer_4_data",1,potentiometer4Callback);


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
    int count = 0;
    while(rclcpp::ok()){
        finish = std::chrono::high_resolution_clock::now();
        if(std::chrono::duration_cast<std::chrono::milliseconds>(finish-start).count() > 100){
            if(sensorless)
                updateMotorPositions(std::chrono::duration_cast<std::chrono::milliseconds>(finish-start).count() );
            count += 1;
            start = std::chrono::high_resolution_clock::now();
        }
        if(count > 10){
            getLinearOut(&linearOut1, &linear1);
            linearOut1Publisher->publish(linearOut1);

            getLinearOut(&linearOut2, &linear2);
            linearOut2Publisher->publish(linearOut2);

            getLinearOut(&linearOut3, &linear3);
            linearOut3Publisher->publish(linearOut3);

            getLinearOut(&linearOut4, &linear4);
            linearOut4Publisher->publish(linearOut4);
            count = 0;
        }
        rclcpp:spin_some(nodeHandle);
    }
}
