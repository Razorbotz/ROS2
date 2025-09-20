#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/empty.hpp>

#include <messages/msg/button_state.hpp>
#include <messages/msg/hat_state.hpp>
#include <messages/msg/axis_state.hpp>
#include <messages/msg/key_state.hpp>
#include <messages/msg/zed_position.hpp>
#include <messages/msg/autonomy_status.hpp>
#include <messages/msg/talon_status.hpp>
#include <messages/msg/falcon_status.hpp>
#include <messages/msg/linear_status.hpp>

#include "logic/Automation1.hpp"
#include "logic/AutomationTypes.hpp"
#include "utils/utils.hpp"


/*
Added subscribers to excavation, logic, falcon, and talon nodes
listening for specific keypresses to kill the node to ensure that
the system is resilient to failures of individual nodes. The kill
keys are as follows:
0 - communication node
1 - logic node
2-5 - talon nodes
6-9 - falcon nodes
; - excavation node
Ideal behavior for node failure:
Communication node - System should stop all motors within 200ms
Logic node - System should stop all motors within 200 ms
Talon node - System shouldn't allow the other actuator to extend
    beyond the safety thresholds
Falcon node - System should continue as expected
    NOTE: This is dependent on testing. It might need to shutdown
    if the testing shows the system is no longer able to move on 
    only three motors
Excavation node - System should be able to drive, but excavation 
    motors should shutdown.
Video streaming node - Critical systems should not be impacted.
Zed tracking node - Critical systems should not be impacted.

*/

/** @file
 * @brief Node handling logic for robot
 * 
 * This node receives information published by the communication node,
 * wraps the information into topics, then publishes the topics.  
 * The topics that the node subscribes to are as follows:
 * \li \b joystick_axis
 * \li \b joystick_button
 * \li \b joystick_hat
 * \li \b key
 * \li \b zed_position
 * 
 * To read more about the communication node
 * \see communication_node.cpp
 * 
 * The topics that are being published are as follows:
 * \li \b drive_left_speed
 * \li \b drive_right_speed
 * \li \b arm_speed
 * \li \b bucket_speed
 * 
 * To read more about the nodes that subscribe to this one
 * \see talon_node.cpp
 * \see excavation_node.py
 * \see falcon_node.cpp
 * 
 * */

rclcpp::Node::SharedPtr nodeHandle;
std_msgs::msg::Empty heartbeat;

float joystick1Roll=0;
float joystick1Pitch=0;
float joystick1Yaw=0;
float joystick1Throttle=0;

float maxSpeed=0.4;

bool automationGo=false;
bool excavationGo = false;
bool printData = false;
bool zedInit = false;
bool useSpeed = false;
bool useController = false;

Automation* automation;

std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > driveLeftSpeedPublisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > driveRightSpeedPublisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > userLeftSpeedPublisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > userRightSpeedPublisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > armSpeedPublisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > bucketSpeedPublisher;

std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool_<std::allocator<void> >, std::allocator<void> > > automationGoPublisher;
std::chrono::time_point<std::chrono::high_resolution_clock> zedPrevious;
std::chrono::time_point<std::chrono::high_resolution_clock> falcon1Previous;
std::chrono::time_point<std::chrono::high_resolution_clock> falcon2Previous;
std::chrono::time_point<std::chrono::high_resolution_clock> falcon3Previous;
std::chrono::time_point<std::chrono::high_resolution_clock> falcon4Previous;

std::chrono::time_point<std::chrono::high_resolution_clock> talon1Previous;
std::chrono::time_point<std::chrono::high_resolution_clock> talon2Previous;
std::chrono::time_point<std::chrono::high_resolution_clock> talon3Previous;
std::chrono::time_point<std::chrono::high_resolution_clock> talon4Previous;


/** @brief Callback function for the keys
 * 
 * This function is called when the node receives a
 * topic with the name key_state.  It currently prints
 * the key pressed to the screen and inverts  
 * @arg automationGo if the key is 's'.  This function
 * also increases the max speed multiplier when the '+'
 * key is pressed and decreases it when the '-' key is 
 * pressed.
 * @param keyState \see KeyState.msg
 * @return void
 * */
void keyCallback(const messages::msg::KeyState::SharedPtr keyState){
    std::cout << "Key " << keyState->key << " " << keyState->state << std::endl;

    if(keyState->key==115 && keyState->state==1){
        automationGo= !automationGo;
        std_msgs::msg::Bool msg;
        msg.data = automationGo;
        automationGoPublisher->publish(msg);
        if(automationGo){
            automation->setGo();
        }
        else{
            automation->setStop();
        }
        RCLCPP_INFO(nodeHandle->get_logger(), "Automation invert.  Current state: %d", automationGo);
    }
    if(keyState->key==100 && keyState->state==1){
        automation->setDiagnostics();
    }
    if(keyState->key==101 && keyState->state==1){
        automation->setExcavate();
    }
    if(keyState->key==97 && keyState->state==1){
        automation->startAutonomy();
    }
    if(keyState->key==68 && keyState->state==1){
        automation->startAutonomy();
    }
    if(keyState->key == 107 && keyState->state == 1){
        automationGo = false;
        automation->setStop();
        automation->setIdle();
    }
    if(keyState->key == 108 && keyState->state == 1){
        automation->setLevel();
    }
    if(keyState->key == 48 && keyState->state==1){
        return;
    }
}


/** @brief Callback function for the zedPosition
 * 
 * This function is called when the node receives a
 * topic with the name zed_position.  This function
 * extracts the information and calls the setPosition
 * function from the automation problem.  
 * \see .Automation.cpp
 * @param zedPosition \see ZedPosition.msg
 * @return void
 * */
void zedPositionCallback(const messages::msg::ZedPosition::SharedPtr zedPosition){
    Position position;
    position.x=zedPosition->x;	
    position.y=zedPosition->y;	
    position.z=zedPosition->z;	
    position.ox=zedPosition->ox;	
    position.oy=zedPosition->oy;	
    position.oz=zedPosition->oz;	
    position.ow=zedPosition->ow;	
    position.roll=zedPosition->roll;
    position.pitch=zedPosition->pitch;
    position.yaw=zedPosition->yaw;
    position.aruco_roll=zedPosition->aruco_roll;
    position.aruco_pitch=zedPosition->aruco_pitch;
    position.aruco_yaw=zedPosition->aruco_yaw;
    position.arucoVisible=zedPosition->aruco_visible;
    position.x_acc = zedPosition->x_acc;
    position.y_acc = zedPosition->y_acc;
    position.z_acc = zedPosition->z_acc;
    position.x_vel = zedPosition->x_vel;
    position.y_vel = zedPosition->y_vel;
    position.z_vel = zedPosition->z_vel;
    position.arucoInitialized = zedPosition->aruco_initialized;

    if(!zedInit)
        zedInit = true;
    
    automation->setPosition(position);
	zedPrevious = std::chrono::high_resolution_clock::now();
}


/** @brief Callback function for the LinearOut topic
 * 
 * @param LinearStatus 
 */
void linearStatus1Callback(const messages::msg::LinearStatus::SharedPtr linearStatus){
    automation->setLinear1(linearStatus);
}


/** @brief Callback function for the linearStatus topic
 * 
 * @param LinearStatus 
 */
void linearStatus2Callback(const messages::msg::LinearStatus::SharedPtr linearStatus){
    automation->setLinear2(linearStatus);
}


/** @brief Callback function for the linearStatus topic
 * 
 * @param LinearStatus 
 */
void linearStatus3Callback(const messages::msg::LinearStatus::SharedPtr linearStatus){
    automation->setLinear3(linearStatus);
}


/** @brief Callback function for the linearStatus topic
 * 
 * @param LinearStatus 
 */
void linearStatus4Callback(const messages::msg::LinearStatus::SharedPtr linearStatus){
    automation->setLinear4(linearStatus);
}


void talon1Callback(const messages::msg::TalonStatus::SharedPtr talonStatus){
    automation->setTalon1(talonStatus);
    talon1Previous = std::chrono::high_resolution_clock::now();
}

void talon2Callback(const messages::msg::TalonStatus::SharedPtr talonStatus){
    automation->setTalon2(talonStatus);
    talon2Previous = std::chrono::high_resolution_clock::now();
}

void talon3Callback(const messages::msg::TalonStatus::SharedPtr talonStatus){
    automation->setTalon3(talonStatus);
    talon3Previous = std::chrono::high_resolution_clock::now();
}

void talon4Callback(const messages::msg::TalonStatus::SharedPtr talonStatus){
    automation->setTalon4(talonStatus);
    talon4Previous = std::chrono::high_resolution_clock::now();
}


void falcon1Callback(const messages::msg::FalconStatus::SharedPtr falconOut){
    automation->setFalcon1(falconOut);
    falcon1Previous = std::chrono::high_resolution_clock::now();
}

void falcon2Callback(const messages::msg::FalconStatus::SharedPtr falconOut){
    automation->setFalcon2(falconOut);
    falcon2Previous = std::chrono::high_resolution_clock::now();
}

void falcon3Callback(const messages::msg::FalconStatus::SharedPtr falconOut){
    automation->setFalcon3(falconOut);
    falcon3Previous = std::chrono::high_resolution_clock::now();
}

void falcon4Callback(const messages::msg::FalconStatus::SharedPtr falconOut){
    automation->setFalcon4(falconOut);
    falcon4Previous = std::chrono::high_resolution_clock::now();
}


bool checkTimes(){
    auto finish = std::chrono::high_resolution_clock::now();
    if(std::chrono::duration_cast<std::chrono::milliseconds>(finish-zedPrevious).count() > 1000 || !zedInit){
        RCLCPP_INFO(nodeHandle->get_logger(), "ERROR: Zed hasn't updated in time.");
        return false;
    }
    if(std::chrono::duration_cast<std::chrono::milliseconds>(finish-talon1Previous).count() > 150){
        RCLCPP_INFO(nodeHandle->get_logger(), "ERROR: Talon1 hasn't updated in time.");
        return false;
    }
    if(std::chrono::duration_cast<std::chrono::milliseconds>(finish-talon3Previous).count() > 150){
        RCLCPP_INFO(nodeHandle->get_logger(), "ERROR: Talon3 hasn't updated in time.");
        return false;
    }
    if(std::chrono::duration_cast<std::chrono::milliseconds>(finish-falcon1Previous).count() > 200){
        RCLCPP_INFO(nodeHandle->get_logger(), "ERROR: Falcon1 hasn't updated in time.");
        return false;
    }
    if(std::chrono::duration_cast<std::chrono::milliseconds>(finish-falcon2Previous).count() > 200){
        RCLCPP_INFO(nodeHandle->get_logger(), "ERROR: Falcon2 hasn't updated in time.");
        return false;
    }
    if(std::chrono::duration_cast<std::chrono::milliseconds>(finish-falcon3Previous).count() > 200){
        RCLCPP_INFO(nodeHandle->get_logger(), "ERROR: Falcon3 hasn't updated in time.");
        return false;
    }
    if(std::chrono::duration_cast<std::chrono::milliseconds>(finish-falcon4Previous).count() > 200){
        RCLCPP_INFO(nodeHandle->get_logger(), "ERROR: Falcon4 hasn't updated in time.");
        return false;
    }
    return true;
}


int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    nodeHandle = rclcpp::Node::make_shared("autonomy");

    std::string mapUsed = utils::getParameter<std::string>(nodeHandle, "map", "NASA");
    bool turnLeft = utils::getParameter<bool>(nodeHandle, "turnLeft", false);
    printData = utils::getParameter<bool>(nodeHandle, "print_data", false);
    automation = new Automation1();
    automation->setMap(mapUsed);
    automation->setTurnLeft(turnLeft);
    automation->setNode(nodeHandle);

    auto zedPositionSubscriber= nodeHandle->create_subscription<messages::msg::ZedPosition>("zed_position",1,zedPositionCallback);
    
    auto linearStatus1Subscriber = nodeHandle->create_subscription<messages::msg::LinearStatus>("linearStatus1",1,linearStatus1Callback);
    auto linearStatus2Subscriber = nodeHandle->create_subscription<messages::msg::LinearStatus>("linearStatus2",1,linearStatus2Callback);
    auto linearStatus3Subscriber = nodeHandle->create_subscription<messages::msg::LinearStatus>("linearStatus3",1,linearStatus3Callback);
    auto linearStatus4Subscriber = nodeHandle->create_subscription<messages::msg::LinearStatus>("linearStatus4",1,linearStatus4Callback);
    
    auto talon1Subscriber = nodeHandle->create_subscription<messages::msg::TalonStatus>("talon_14_info",1,talon1Callback);
    auto talon2Subscriber = nodeHandle->create_subscription<messages::msg::TalonStatus>("talon_15_info",1,talon2Callback);
    auto talon3Subscriber = nodeHandle->create_subscription<messages::msg::TalonStatus>("talon_16_info",1,talon3Callback);
    auto talon4Subscriber = nodeHandle->create_subscription<messages::msg::TalonStatus>("talon_17_info",1,talon4Callback);
    
    auto falcon1Subscriber = nodeHandle->create_subscription<messages::msg::FalconStatus>("talon_10_info",1,falcon1Callback);
    auto falcon2Subscriber = nodeHandle->create_subscription<messages::msg::FalconStatus>("talon_11_info",1,falcon2Callback);
    auto falcon3Subscriber = nodeHandle->create_subscription<messages::msg::FalconStatus>("talon_12_info",1,falcon3Callback);
    auto falcon4Subscriber = nodeHandle->create_subscription<messages::msg::FalconStatus>("talon_13_info",1,falcon4Callback);

    driveLeftSpeedPublisher= nodeHandle->create_publisher<std_msgs::msg::Float32>("drive_left_speed",1);
    driveRightSpeedPublisher= nodeHandle->create_publisher<std_msgs::msg::Float32>("drive_right_speed",1);
    armSpeedPublisher= nodeHandle->create_publisher<std_msgs::msg::Float32>("arm_speed",1);
    bucketSpeedPublisher= nodeHandle->create_publisher<std_msgs::msg::Float32>("bucket_speed",1);

    rclcpp::Rate rate(30);
    while(rclcpp::ok()){
		auto finish = std::chrono::high_resolution_clock::now();
        if(automationGo){
            automation->automate();
        }
        automation->publishAutomationStatus();
        rclcpp::spin_some(nodeHandle);
        rate.sleep();
    }
    rclcpp::shutdown();
}
