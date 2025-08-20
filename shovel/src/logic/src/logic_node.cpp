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
#include <messages/msg/autonomy_out.hpp>
#include <messages/msg/talon_out.hpp>
#include <messages/msg/falcon_out.hpp>
#include <messages/msg/linear_out.hpp>
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

float joystick2Roll=0;
float joystick2Pitch=0;
float joystick2Yaw=0;
float joystick2Throttle=0;

float maxSpeed=0.4;

bool excavationGo = false;
bool printData = false;
bool zedInit = false;
bool useSpeed = false;
bool useController = false;
bool twoControllers = false;
bool useAltJoystick = false;

std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > driveLeftSpeedPublisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > driveRightSpeedPublisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > userLeftSpeedPublisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > userRightSpeedPublisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > armSpeedPublisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > bucketSpeedPublisher;


/** @brief Function to initialize the motors to zero
 * 
 * This function is called on start of the node and
 * sends a message with a zero message to all motors
 * to ensure that the motors are not set to an old 
 * value.
 * @return void
 * */
void initSetSpeed(){
    std_msgs::msg::Float32 speed;
    speed.data = 0.0;
    
    driveLeftSpeedPublisher->publish(speed);
    driveRightSpeedPublisher->publish(speed);
    armSpeedPublisher->publish(speed);
    bucketSpeedPublisher->publish(speed);
    //RCLCPP_INFO(nodeHandle->get_logger(), "Set init motor speeds to 0.0");
}

/** @brief Function to update speed of the wheels
 * 
 * This function is called by the joystickAxisCallback
 * to update the speed to the wheels.  It uses the data 
 * published to compute the speed of the wheels based on
 * the joystick information and limited by the maxSpeed.
 * @return void
 * */
void updateSpeed(){
    std_msgs::msg::Float32 speedLeft;
    std_msgs::msg::Float32 speedRight;
    
    //Linear transformation of cordinate planes
    speedLeft.data  = (joystick1Pitch + joystick1Roll);
    speedRight.data = (joystick1Pitch - joystick1Roll);
    
    //multiplied the output speed by the "throttle" value and a limiting precentage
    speedLeft.data  = speedLeft.data * maxSpeed;
    speedRight.data = speedRight.data * maxSpeed;
    
    if(printData)
        RCLCPP_INFO(nodeHandle->get_logger(),"speed left=%f right=%f  pitch=%f roll=%f", speedLeft.data,  speedRight.data, joystick1Pitch, joystick1Roll);

    if(useSpeed){
        driveLeftSpeedPublisher->publish(speedLeft);
        driveRightSpeedPublisher->publish(speedRight);
    }
    else{
        userLeftSpeedPublisher->publish(speedLeft);
        userRightSpeedPublisher->publish(speedRight);
    }
    
}

/** @brief Function to stop drive train motors
 * 
 * This function is called when toggle excavation 
 * button is pressed and switches to the excavation
 * state.  It publishes a speed of 0.0 for both the
 * right and left wheels.
 * @return void
 * */
void stopSpeed(){
    std_msgs::msg::Float32 speed;
    speed.data = 0.0;
    userLeftSpeedPublisher->publish(speed);
    userRightSpeedPublisher->publish(speed);
}

/**
 * @brief  Function to transform the joystick input
 * 
 * This function is called to transform the joystick input 
 * into a slightly modified version.  Because the joystick
 * doesn't go to exactly zero, there will always be data
 * published to the robot, causing it to drift slightly.  
 * This function zeroes the input in the range
 * [-deadZone, deadZone] to prevent this.
 * 
 * @param info - Float value of the joystick axis
 * @param deadZone - Float value of the deadzone of the joystick
 * @return float 
 */
float transformJoystickInfo(float info, float deadZone){
    float transformed = (fabs(info) < deadZone) ? 0.0 : info;
    transformed = (transformed > 0) ? transformed - deadZone : transformed;
    transformed = (transformed < 0) ? transformed + deadZone : transformed;
    return transformed;
}


/** @brief Callback function for joystick axis topic
 * 
 * This function is called when the node receives the
 * topic joystick_axis, then converts the joystick
 * input using a series of linear transformations 
 * before calling updateSpeed() or updateExcavation()
 * depending on the current state.
 * @param axisState \see AxisState.msg
 * @return void
 * */
void joystickAxisCallback(const messages::msg::AxisState::SharedPtr axisState){
    //RCLCPP_INFO(nodeHandle->get_logger(),"Axis %d %d %f", axisState->joystick, axisState->axis, axisState->state);
    //RCLCPP_INFO(nodeHandle->get_logger(),"Axis %d %f %f %f %f", axisState->joystick, axisState->state0, axisState->state1, axisState->state2, axisState->state3);
    float deadZone = 0.1;
    if(!useAltJoystick){
        if(axisState->joystick == 0){
            if(axisState->axis==0){
                joystick1Roll = transformJoystickInfo(-axisState->state, deadZone);
                updateSpeed();
            }
            else if(axisState->axis==1){
                joystick1Pitch = transformJoystickInfo(axisState->state, deadZone);
                updateSpeed();
            }
            else if(axisState->axis==2){
                joystick1Yaw = transformJoystickInfo(axisState->state, deadZone);
                if(useController){
                    std_msgs::msg::Float32 armSpeed;
                    armSpeed.data = joystick1Yaw;
                    armSpeedPublisher->publish(armSpeed);
                }
                else{
                    if(!twoControllers){
                        std_msgs::msg::Float32 bucketSpeed;
                        bucketSpeed.data = joystick1Yaw;
                        bucketSpeedPublisher->publish(bucketSpeed);
                    }
                }
            }
            else if(axisState->axis==3){
                joystick1Throttle = axisState->state/2 + 0.5;
                joystick1Throttle = transformJoystickInfo(joystick1Throttle, deadZone);
                if(useController){
                    std_msgs::msg::Float32 bucketSpeed;
                    bucketSpeed.data = axisState->state;
                    bucketSpeedPublisher->publish(bucketSpeed);
                }
            }
        }
        else if(axisState->joystick == 1){
            if(!twoControllers){
                twoControllers = true;
            }
            if(axisState->axis==0){
                joystick2Roll = transformJoystickInfo(axisState->state, deadZone);
                std_msgs::msg::Float32 bucketSpeed;
                bucketSpeed.data = joystick2Roll;
                bucketSpeedPublisher->publish(bucketSpeed);
            }
            else if(axisState->axis==1){
                joystick2Pitch = transformJoystickInfo(-axisState->state, deadZone);
                std_msgs::msg::Float32 armSpeed;
                armSpeed.data = joystick2Pitch;
                armSpeedPublisher->publish(armSpeed);
            }
            else if(axisState->axis==2){
                joystick2Yaw = transformJoystickInfo(axisState->state, deadZone);
            }
            else if(axisState->axis==3){
                joystick2Throttle = axisState->state/2 + 0.5;
                joystick2Throttle = transformJoystickInfo(joystick2Throttle, deadZone);
            }
        }
    }
    else{
        if(axisState->joystick == 0){
            if(axisState->axis==0){
                joystick1Roll = transformJoystickInfo(-axisState->state, deadZone);
                std_msgs::msg::Float32 speedLeft;
                speedLeft.data = joystick1Roll;
                speedLeft.data  = speedLeft.data * maxSpeed;
                if(useSpeed){
                    driveLeftSpeedPublisher->publish(speedLeft);
                }
                else{
                    userLeftSpeedPublisher->publish(speedLeft);
                }
            }
            else if(axisState->axis==1){
                joystick1Pitch = transformJoystickInfo(axisState->state, deadZone);
                std_msgs::msg::Float32 armSpeed;
                armSpeed.data = joystick1Pitch;
                armSpeedPublisher->publish(armSpeed);
            }
            else if(axisState->axis==2){
                joystick1Yaw = transformJoystickInfo(axisState->state, deadZone);
                
            }
            else if(axisState->axis==3){
                joystick1Throttle = axisState->state/2 + 0.5;
                joystick1Throttle = transformJoystickInfo(joystick1Throttle, deadZone);
            }
        }
        else if(axisState->joystick == 1){
            if(!twoControllers){
                twoControllers = true;
            }
            if(axisState->axis==0){
                joystick2Roll = transformJoystickInfo(-axisState->state, deadZone);
                std_msgs::msg::Float32 speedRight;
                speedRight.data = joystick1Roll;
                speedRight.data  = speedRight.data * maxSpeed;
                if(useSpeed){
                    driveRightSpeedPublisher->publish(speedRight);
                }
                else{
                    userRightSpeedPublisher->publish(speedRight);
                }
            }
            else if(axisState->axis==1){
                joystick2Pitch = transformJoystickInfo(axisState->state, deadZone);
                std_msgs::msg::Float32 bucketSpeed;
                bucketSpeed.data = joystick2Roll;
                bucketSpeedPublisher->publish(bucketSpeed);
            }
            else if(axisState->axis==2){
                joystick2Yaw = transformJoystickInfo(axisState->state, deadZone);
            }
            else if(axisState->axis==3){
                joystick2Throttle = axisState->state/2 + 0.5;
                joystick2Throttle = transformJoystickInfo(joystick2Throttle, deadZone);
            }
        }
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

        case 0:
            break;
        case 1:
            break;
        case 2:
            if(printData)
                RCLCPP_INFO(nodeHandle->get_logger(), "Button 3");
            break;
        case 3:
            if(printData)
                RCLCPP_INFO(nodeHandle->get_logger(), "Button 4");
            break;
        case 4:
            if(printData)
                RCLCPP_INFO(nodeHandle->get_logger(), "Button 5");
            break;
        case 5:
            if(printData)
                RCLCPP_INFO(nodeHandle->get_logger(), "Button 6");
            break;
        case 6:
            if(printData)
                RCLCPP_INFO(nodeHandle->get_logger(), "Button 7");
            break;
        case 7:
            if(printData)
                RCLCPP_INFO(nodeHandle->get_logger(), "Button 8");
            break;
        case 8:
            break;
        case 9:
            break;
        case 10:
            break;
        case 11:
            break;
    }
}


/** @brief Callback function for joystick hat
 * 
 * This function is called when the node receives a
 * topic with the name joystick_hat.  It publishes
 * the dump bin speed based on the hat.
 * @param hatState \see HatState.msg
 * @return void
 * */
void joystickHatCallback(const messages::msg::HatState::SharedPtr hatState){
    std::cout << "Hat " << (int)hatState->joystick << " " << (int)hatState->hat << " " << (int)hatState->state << std::endl;
    std_msgs::msg::Float32 speed;
    if((int)hatState->state == 1 ){
        speed.data = 1.0;
    }
    if((int)hatState->state == 4 ){
        speed.data = -1.0;
    }
    if((int)hatState->state == 0 ){
        speed.data = 0.0;
    }

}

/** @brief Function to update max speed multiplier
 * 
 * This function is called when the keyCallback
 * function receives either a '+' or '-' keypress
 * event.  A delta speed value is then passed to
 * this function to change the value of the max
 * speed multiplier dynamically, instead of the 
 * user recompiling the code and reexecuting it.
 * @param deltaSpeed - Amount to change max speed
 * @return void
 * */
void updateMaxSpeed(float deltaSpeed){
    maxSpeed += deltaSpeed;
    if(maxSpeed <= 0){
        maxSpeed = 0.1;
    }
    if(maxSpeed > 1){
        maxSpeed = 1;
    }
    if(printData)
        RCLCPP_INFO(nodeHandle->get_logger(), "maxSpeed: %f", maxSpeed);
}


/** @brief Callback function for the keys
 * 
 * This function is called when the node receives a
 * topic with the name key_state.  It currently prints
 * the key pressed to the screen. This function
 * also increases the max speed multiplier when the '+'
 * key is pressed and decreases it when the '-' key is 
 * pressed.
 * @param keyState \see KeyState.msg
 * @return void
 * */
void keyCallback(const messages::msg::KeyState::SharedPtr keyState){
    std::cout << "Key " << keyState->key << " " << keyState->state << std::endl;
    if(keyState->key==43 && keyState->state==1){
        updateMaxSpeed(0.2);
    }
    if(keyState->key==45 && keyState->state==1){
        updateMaxSpeed(-0.2);
    }
    if(keyState->key == 48 && keyState->state==1){
        return;
    }
    if(keyState->key==83 && keyState->state == 1){
        useSpeed = true;
    }
    if(keyState->key==2){
        useController = true;
    }
    if(keyState->key==3){
        useAltJoystick = true;
    }
}

int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    nodeHandle = rclcpp::Node::make_shared("logic");

    printData = utils::getParameter<bool>(nodeHandle, "print_data", false);

    auto joystickAxisSubscriber= nodeHandle->create_subscription<messages::msg::AxisState>("joystick_axis",1,joystickAxisCallback);
    auto joystickButtonSubscriber= nodeHandle->create_subscription<messages::msg::ButtonState>("joystick_button",1,joystickButtonCallback);
    auto joystickHatSubscriber= nodeHandle->create_subscription<messages::msg::HatState>("joystick_hat",1,joystickHatCallback);
    auto keySubscriber= nodeHandle->create_subscription<messages::msg::KeyState>("key",1,keyCallback);
    
    driveLeftSpeedPublisher= nodeHandle->create_publisher<std_msgs::msg::Float32>("drive_left_speed",1);
    driveRightSpeedPublisher= nodeHandle->create_publisher<std_msgs::msg::Float32>("drive_right_speed",1);
    armSpeedPublisher= nodeHandle->create_publisher<std_msgs::msg::Float32>("arm_speed",1);
    bucketSpeedPublisher= nodeHandle->create_publisher<std_msgs::msg::Float32>("bucket_speed",1);
    userLeftSpeedPublisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("user_left_speed",1);
    userRightSpeedPublisher= nodeHandle->create_publisher<std_msgs::msg::Float32>("user_right_speed",1);

    auto logicHeartbeatPublisher = nodeHandle->create_publisher<std_msgs::msg::Empty>("logic_heartbeat",1);

    initSetSpeed();

    rclcpp::Rate rate(30);
    while(rclcpp::ok()){
		auto finish = std::chrono::high_resolution_clock::now();
        rclcpp::spin_some(nodeHandle);
        logicHeartbeatPublisher->publish(heartbeat);
        rate.sleep();
    }
    rclcpp::shutdown();
}
