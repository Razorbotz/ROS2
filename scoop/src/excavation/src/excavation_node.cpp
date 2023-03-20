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


void sync(){
    float diff = abs(linear1.potentiometer - linear2.potentiometer);
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
    std_msgs::msg::Float32 speed1;
    speed1.data = linear1.speed;    
    talon14Publisher->publish(speed1);
    std_msgs::msg::Float32 speed2;
    speed2.data = linear2.speed;
    talon15Publisher->publish(speed2);
    RCLCPP_INFO(nodeHandle->get_logger(),"Sync: %f, %f", linear1.speed, linear2.speed);
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
        sync();
    }
    else{
        std_msgs::msg::Float32 speed1;
        speed1.data = linear1.speed;    
        talon14Publisher->publish(speed1);
        std_msgs::msg::Float32 speed2;
        speed2.data = linear2.speed;
        talon15Publisher->publish(speed2);
    }
}

void setPotentiometerError(int potentData, LinearActuator *linear){
    if(potentData == 1024){
        linear->error = PotentiometerError;
    }
    else{
        if(linear->error == PotentiometerError){
            linear->error = None;
        }
    }
}

void processPotentiometerData(int potentData, LinearActuator *linear){
    if(potentData < linear->min){
        linear->min = potentData;
    }

    if(potentData > linear->max){
        linear->max = potentData;
    }

    if(linear->potentiometer >= potentData - 20 && linear->potentiometer <= potentData + 20){
        if(linear->speed != 0.0){
            linear->count += 1;
            if(linear->count >= 5){
                if(potentData >= linear->max - 30){
                    linear->atMax = true;
                }
                else if(potentData <= linear->min + 30){
                    linear->atMin = true;
                }
                else{
                    if(linear->error == None){
                        linear->error = ActuatorNotMovingError;
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
        linear->atMax = false;
        linear->atMin = false;
    }
    linear->potentiometer = potentData;
}


void potentiometerCallback(const std_msgs::msg::Int16MultiArray::SharedPtr potent){
    RCLCPP_INFO(nodeHandle->get_logger(),"Potentiometer %d %d %d", potent->data[0], potent->data[1], potent->data[2]);
    
    if(potent->data[0] == -1 || potent->data[1] == -1){
        linear1.error = ConnectionError;
        linear2.error = ConnectionError;
        linear3.error = ConnectionError;
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


void dumpSpeedCallback(const std_msgs::msg::Float32::SharedPtr speed){
    if(!automationGo){
        std_msgs::msg::Float32 speed1;
        speed1.data = speed->data;    
        talon16Publisher->publish(speed1);
    }
}


void automationGoCallback(const std_msgs::msg::Bool::SharedPtr msg){
    automationGo = msg->data;
}


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

    talon14Publisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("talon_14_speed",1);
    talon15Publisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("talon_15_speed",1);
    talon16Publisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("talon_16_speed",1);
    
    messages::msg::LinearOut linearOut1;
    messages::msg::LinearOut linearOut2;
    messages::msg::LinearOut linearOut3;

    auto linearOut1Publisher = nodeHandle->create_publisher<messages::msg::LinearOut>("linearOut1",1);
    auto linearOut2Publisher = nodeHandle->create_publisher<messages::msg::LinearOut>("linearOut2",1);
    auto linearOut3Publisher = nodeHandle->create_publisher<messages::msg::LinearOut>("linearOut3",1);

    auto start = std::chrono::high_resolution_clock::now();
    while(rclcpp::ok()){
        auto finish = std::chrono::high_resolution_clock::now();
        if(std::chrono::duration_cast<std::chrono::nanoseconds>(finish-start).count() > 250000000){
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