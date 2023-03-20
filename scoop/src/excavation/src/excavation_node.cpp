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
}


// TODO: Break this into smaller functions
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

    if(potent->data[0] == 1024){
        linear1.error = PotentiometerError;
    }
    else{
        if(linear1.error == PotentiometerError){
            linear1.error = None;
        }
    }

    if(potent->data[1] == 1024){
        linear2.error = PotentiometerError;
    }
    else{
        if(linear2.error == PotentiometerError){
            linear2.error = None;
        }
    }

    if(potent->data[2] == 1024){
        linear3.error = PotentiometerError;
    }
    else{
        if(linear3.error == PotentiometerError){
            linear3.error = None;
        }
    }
    
    if(linear1.error != ConnectionError && linear1.error != PotentiometerError && linear2.error != PotentiometerError){
        if(potent->data[0] < linear1.min){
            linear1.min = potent->data[0];
        }
        if(potent->data[1] < linear2.min){
            linear2.min = potent->data[1];
        }

        if(potent->data[0] > linear1.max){
            linear1.max = potent->data[0];
        }
        if(potent->data[1] > linear2.max){
            linear2.max = potent->data[1];
        }

        if(linear1.potentiometer >= potent->data[0] - 20 && linear1.potentiometer <= potent->data[0] + 20){
            if(linear1.speed != 0.0){
                linear1.count += 1;
                if(linear1.count >= 5){
                    if(potent->data[0] >= linear1.max - 30){
                        linear1.atMax = true;
                    }
                    else if(potent->data[0] <= linear1.min + 30){
                        linear1.atMin = true;
                    }
                    else{
                        if(linear1.error == None){
                            linear1.error = ActuatorNotMovingError;
                        }
                    }
                }   
            } 
        }
        else{
            linear1.count = 0;
            if(linear1.error == ActuatorNotMovingError){
                linear1.error = None;
            }
            linear1.atMax = false;
            linear1.atMin = false;
        }

        if(linear2.potentiometer >= potent->data[1] - 20 && linear2.potentiometer <= potent->data[1] + 20){
            if(linear2.speed != 0.0){
                linear2.count += 1;
                if(linear2.count >= 5){
                    if(potent->data[1] >= linear2.max - 30){
                        linear2.atMax = true;
                    }
                    else if(potent->data[1] <= linear2.min + 30){
                        linear2.atMin = true;
                    }
                    else{
                        if(linear2.error == None){
                            linear2.error = ActuatorNotMovingError;
                        }
                    }
                }   
            } 
        }
        else{
            linear2.count = 0;
            if(linear2.error == ActuatorNotMovingError){
                linear2.error = None;
            }
            linear2.atMax = false;
            linear2.atMin = false;
        }

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
        linear1.potentiometer = potent->data[0];
        linear2.potentiometer = potent->data[1];
        sync();
    }
    // Get potentiometer 3 data
    if(linear3.error != ConnectionError && linear3.error != PotentiometerError){
        if(potent->data[2] < linear3.min){
            linear3.min = potent->data[2];
        }
        if(potent->data[2] > linear3.max){
            linear3.max = potent->data[2];
        }
        if(linear3.potentiometer >= potent->data[2] - 20 && linear3.potentiometer <= potent->data[2] + 20){
            if(linear3.speed != 0.0){
                linear3.count += 1;
                if(linear3.count >= 5){
                    if(potent->data[2] >= linear3.max - 30){
                        linear3.atMax = true;
                    }
                    else if(potent->data[2] <= linear3.min + 30){
                        linear3.atMin = true;
                    }
                    else{
                        if(linear3.error == None){
                            linear3.error = ActuatorNotMovingError;
                        }
                    }
                }   
            } 
        }
        else{
            linear3.count = 0;
            if(linear3.error == ActuatorNotMovingError){
                linear3.error = None;
            }
            linear3.atMax = false;
            linear3.atMin = false;
        }
        linear3.potentiometer = potent->data[2];
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
            linearOut1.speed = linear1.speed;
            linearOut1.potentiometer = linear1.potentiometer;
            linearOut1.time_without_change = linear1.timeWithoutChange;
            linearOut1.max = linear1.max;
            linearOut1.min = linear1.min;
            linearOut1.error = errorMap.at(linear1.error);
            linearOut1.run = linear1.run;
            linearOut1.at_min = linear1.atMin;
            linearOut1.at_max = linear1.atMax;
            linearOut1Publisher->publish(linearOut1);

            linearOut2.speed = linear2.speed;
            linearOut2.potentiometer = linear2.potentiometer;
            linearOut2.time_without_change = linear2.timeWithoutChange;
            linearOut2.max = linear2.max;
            linearOut2.min = linear2.min;
            linearOut2.error = errorMap.at(linear2.error);
            linearOut2.run = linear2.run;
            linearOut2.at_min = linear2.atMin;
            linearOut2.at_max = linear2.atMax;
            linearOut2Publisher->publish(linearOut2);

            linearOut3.speed = linear3.speed;
            linearOut3.potentiometer = linear3.potentiometer;
            linearOut3.time_without_change = linear3.timeWithoutChange;
            linearOut3.max = linear3.max;
            linearOut3.min = linear3.min;
            linearOut3.error = errorMap.at(linear3.error);
            linearOut3.run = linear3.run;
            linearOut3.at_min = linear3.atMin;
            linearOut3.at_max = linear3.atMax;
            linearOut3Publisher->publish(linearOut3);
        }
        rclcpp:spin_some(nodeHandle);
    }
}