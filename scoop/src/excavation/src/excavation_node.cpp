#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>

#include "messages/msg/linear_out.hpp"

rclcpp::Node::SharedPtr nodeHandle;

std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > talon14Publisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > talon15Publisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > talon16Publisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > talon17Publisher;

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
 * \li \b potentiometer_data
 * \li \b shoulder_speed
 * \li \b dump_speed
 * \li \b automationGo
 * 
 * 
 * This node publishes the following topics:
 * \li \b talon_14_speed
 * \li \b talon_15_speed
 * \li \b talon_16_speed
 * \li \b linearOut1
 * \li \b linearOut2
 * \li \b linearOut3
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
    float speed = 0.0;
    int potentiometer = 0;
    int timeWithoutChange = 0;
    int max = 0;
    int min = 1024;
    int count = 0;
    Error error = ConnectionError;
    bool run = true;
    bool atMin = false;
    bool atMax = false;
    float stroke = 11.8;
    float extended = 0.0;
};


LinearActuator linear1;
LinearActuator linear2;
LinearActuator linear3;

float currentSpeed = 0;
int thresh1 = 60;
int thresh2 = 120;
int thresh3 = 180;

bool automationGo = false;


/** @brief Function to sync the linear actuators
 * 
 * The sync function works by checking if the currentSpeed is
 * greater than zero. If the speed is greater than zero, the val
 * checks which actuator is more extended and sets the speed of
 * the actuator to a lower value if the diff is greater than the 
 * thresh values.  If the value is less than zero, the val checks 
 * which actuator is less extended and sets the speed of the 
 * actuator to a lower value. The function then publishes the 
 * updated speed.
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


/** @brief Shoulder Callback Function
 * 
 * Callback function triggered when the node receives
 * a topic with the topic name of shoulder_speed. This
 * function checks if the automationGo value is true; if
 * it is false, the linear actuators are set to the speed 
 * value, even if the linear actuators are in an error state.
 * It is assumed that the user is watching the motors and is
 * intending to override the error checking logic. If the 
 * automationGo value is true, the speeds are only set if the
 * actuators don't have a PotentiometerError, then syncs the
 * actuators. To better understand the logic, please take a
 * look at the Excavation Speed Flow Diagram.
 * @param speed
 * @return void
 * */
void shoulderCallback(const std_msgs::msg::Float32::SharedPtr speed){
    currentSpeed = speed->data;
    RCLCPP_INFO(nodeHandle->get_logger(),"currentSpeed: %f", currentSpeed);
    if(!automationGo){
        linear1.speed = currentSpeed;
        linear2.speed = currentSpeed;
    }
    else{
        if(linear1.error != ConnectionError && linear1.error != PotentiometerError && linear2.error != PotentiometerError){
            linear1.speed = currentSpeed;
            linear2.speed = currentSpeed;
        }
    }
    if(linear1.error != ConnectionError && linear1.error != PotentiometerError && linear2.error != PotentiometerError){
        sync();
        if(linear1.atMax && currentSpeed > 0){
            linear1.speed = 0.0;
        }
        if(linear2.atMax && currentSpeed > 0){
            linear2.speed = 0.0;
        }
        if(linear1.atMin && currentSpeed < 0){
            linear1.speed = 0.0;
        }
        if(linear2.atMin && currentSpeed < 0){
            linear2.speed = 0.0;
        }
    }
    std_msgs::msg::Float32 speed1;
    speed1.data = linear1.speed;    
    talon14Publisher->publish(speed1);
    std_msgs::msg::Float32 speed2;
    speed2.data = linear2.speed;
    talon15Publisher->publish(speed2);
    RCLCPP_INFO(nodeHandle->get_logger(),"Shoulder speeds: %f, %f", linear1.speed, linear2.speed);
}


/** @brief Function to set potentiometer error
 * 
 * Thsi function is used to set the value of the error
 * of the linear object.  If the potentiometer is equal
 * to 1024, which is the value that occurs when the
 * potentiometer is disconnected from the Arduino. Refer
 * to the ErrorState state diagram for more information.
 * @param potentData - Int value of potentiometer
 * @param *linear - Pointer to linear object
 * @return void
 * */
void setPotentiometerError(int potentData, LinearActuator *linear){
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


/** @brief Function to process potentiometer data
 * 
 * This function processes the passed potentiometer data
 * and adjusts the passed linear values accordingly. First
 * the function sets the min and max values if the new data
 * is beyond the previous limits. Next, the function checks
 * if the value is within a threshold of the previous value
 * that is stored in the linear->potentiometer variable. If
 * the value is within this threshold, it's assumed that 
 * the actuator isn't moving. If the speed isn't equal to
 * zero, ie the actuator should be moving, the count
 * variable gets increased. If the count is greater than 5,
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
            linear->count += 1;
            if(linear->count >= 5){
                if(linear->max > 800 && linear->speed > 0.0 && potentData >= linear->max - 20){
                    linear->atMax = true;
                    linear->count = 0;
                }
                else if(linear->min < 200 && linear->speed < 0.0 && potentData <= linear->min + 20){
                    linear->atMin = true;
                    linear->count = 0;
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
        linear->count = 0;
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



/** @brief Callback function for the potentiometer topic
 * 
 * This function receives the potentiometer topic and processes the
 * information. If any of the values are equal to -1, the Arduino
 * node isn't reading the data correctly and any data that is sent
 * should be ignored, which is denoted by raising the ConnectionError
 * error. If the values received are not equal to -1, the data is 
 * being read and there is no ConnectionError. Next, the values
 * are sent to the setPotentiometerError function to ensure that the
 * received values are valid. If there is no connection error and the
 * potentiometers are reading correctly, the first two linear actuators
 * are compared to check if there is a synchronization error and are
 * then synced. 
 * @param potent - Array of ints containing potentiometer information
 * @return void
 * */
void potentiometerCallback(const std_msgs::msg::Int16MultiArray::SharedPtr potent){
    RCLCPP_INFO(nodeHandle->get_logger(),"Potentiometer %d %d %d", potent->data[0], potent->data[1], potent->data[2]);
    
    if(potent->data[0] == -1 || potent->data[1] == -1){
        linear1.error = ConnectionError;
        linear2.error = ConnectionError;
        linear3.error = ConnectionError;
        RCLCPP_INFO(nodeHandle->get_logger(),"EXCAVATION ERROR: ConnectionError");
    }
    else{
        if(linear1.error == ConnectionError){
            linear1.error = None;
            linear2.error = None;
            linear3.error = None;
        }
    }

    setPotentiometerError(potent->data[0], &linear1);
    setPotentiometerError(potent->data[1], &linear2);
    setPotentiometerError(potent->data[2], &linear3);

    if(linear1.error != ConnectionError && linear1.error != PotentiometerError && linear2.error != PotentiometerError){
        processPotentiometerData(potent->data[0], &linear1);
        processPotentiometerData(potent->data[1], &linear2);

        if(abs(potent->data[0] - potent->data[1]) > thresh1){
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
    }

    if(linear3.error != ConnectionError && linear3.error != PotentiometerError){
        processPotentiometerData(potent->data[2], &linear3);
    }
}



/** @brief Callback function for the dump_speed topic
 * 
 * 
 * @param speed - ROS2 message containing the speed data
 * @return void
 * */
void dumpSpeedCallback(const std_msgs::msg::Float32::SharedPtr speed){
    if(!automationGo){
        std_msgs::msg::Float32 speed1;
        speed1.data = speed->data;    
        talon16Publisher->publish(speed1);
        linear3.speed = speed->data;
    }
}


/** @brief Callback function for the automationGo topic
 * 
 * This function sets the automationGo value to the value
 * in the message.
 * @param msg - ROS2 message containing automationGo value
 * @return void
 * */
void automationGoCallback(const std_msgs::msg::Bool::SharedPtr msg){
    automationGo = msg->data;
}


/** @brief Callback function for the ladder_speed topic
 * 
 * This function publishes the speed topic to the Talon 
 * that controls the height of the bucket ladder.
 * @param speed - ROS2 message containing the speed data
 * @return void
 * */
void dumpBinSpeedCallback(const std_msgs::msg::Float32::SharedPtr speed){
    std_msgs::msg::Float32 speed1;
    speed1.data = speed->data;    
    talon17Publisher->publish(speed1);
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
    linearOut->speed = linear->speed;
    linearOut->potentiometer = linear->potentiometer;
    linearOut->time_without_change = linear->timeWithoutChange;
    linearOut->max = linear->max;
    linearOut->min = linear->min;
    linearOut->error = errorMap.at(linear->error);
    linearOut->run = linear->run;
    linearOut->at_min = linear->atMin;
    linearOut->at_max = linear->atMax;
}


int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    nodeHandle = rclcpp::Node::make_shared("excavation");

    auto potentiometerSubscriber = nodeHandle->create_subscription<std_msgs::msg::Int16MultiArray>("potentiometer_data",1, potentiometerCallback);
    auto shoulderSubscriber = nodeHandle->create_subscription<std_msgs::msg::Float32>("shoulder_speed",1,shoulderCallback);
    auto dumpSpeedSubscriber = nodeHandle->create_subscription<std_msgs::msg::Float32>("dump_speed",1,dumpSpeedCallback);
    auto automationGoSubscriber = nodeHandle->create_subscription<std_msgs::msg::Bool>("automationGo",1,automationGoCallback);
    auto dumpBinSubscriber = nodeHandle->create_subscription<std_msgs::msg::Float32>("dump_bin_speed",1,dumpBinSpeedCallback);

    talon14Publisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("talon_14_speed",1);
    talon15Publisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("talon_15_speed",1);
    talon16Publisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("talon_16_speed",1);
    talon17Publisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("talon_17_speed",1);
    
    messages::msg::LinearOut linearOut1;
    messages::msg::LinearOut linearOut2;
    messages::msg::LinearOut linearOut3;

    auto linearOut1Publisher = nodeHandle->create_publisher<messages::msg::LinearOut>("linearOut1",1);
    auto linearOut2Publisher = nodeHandle->create_publisher<messages::msg::LinearOut>("linearOut2",1);
    auto linearOut3Publisher = nodeHandle->create_publisher<messages::msg::LinearOut>("linearOut3",1);

    auto start = std::chrono::high_resolution_clock::now();
    while(rclcpp::ok()){
        auto finish = std::chrono::high_resolution_clock::now();
        if(std::chrono::duration_cast<std::chrono::seconds>(finish-start).count() > 2){
            getLinearOut(&linearOut1, &linear1);
            linearOut1Publisher->publish(linearOut1);

            getLinearOut(&linearOut2, &linear2);
            linearOut2Publisher->publish(linearOut2);

            getLinearOut(&linearOut3, &linear3);
            linearOut3Publisher->publish(linearOut3);
        }
        rclcpp:spin_some(nodeHandle);
    }
}