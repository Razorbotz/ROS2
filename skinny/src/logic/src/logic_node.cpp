#include <math.h>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <messages/msg/button_state.hpp>
#include <messages/msg/hat_state.hpp>
#include <messages/msg/axis_state.hpp>
#include <messages/msg/key_state.hpp>
#include <messages/msg/zed_position.hpp>

#include "logic/Automation1.hpp"
#include "logic/AutomationTypes.hpp"

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
 * \li \b drive_state
 * 
 * To read more about the nodes that subscribe to this one
 * \see talon_node.cpp
 * 
 * This node uses ternary statements in the joystickAxisCallback
 * function.  To read more about how these operate, please read
 * \see http://www.cplusplus.com/articles/1AUq5Di1/
 * 
 * */

rclcpp::Node::SharedPtr nodeHandle;

float joystick1Roll=0;
float joystick1Pitch=0;
float joystick1Yaw=0;
float joystick1Throttle=1;

bool automationGo=false;

Automation* automation=new Automation1();

std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > driveLeftSpeedPublisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > driveRightSpeedPublisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool_<std::allocator<void> >, std::allocator<void> > > driveStatePublisher;

/** @brief Function to update speed of the wheels
 * 
 * This function is called on the node startup to 
 * initialize the wheels to a speed of zero to
 * prevent the random wheel rotations that would
 * occur at certain times.  
 * @return void
 * */
void initSetSpeed(){

    std_msgs::msg::Float32 speed;
    speed.data = 0.0;

    driveLeftSpeedPublisher->publish(speed);
    driveRightSpeedPublisher->publish(speed);
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
    float maxSpeed = 0.4;
    
    //Linear transformation of cordinate planes
    speedLeft.data  = (joystick1Pitch + joystick1Roll);
    speedRight.data = (joystick1Pitch - joystick1Roll);
    
    //multiplied the output speed by the "throttle" value and a limiting precentage
    speedLeft.data  = speedLeft.data * maxSpeed;
    speedRight.data = speedRight.data * maxSpeed;
    
    RCLCPP_INFO(nodeHandle->get_logger(),"speed left=%f right=%f  pitch=%f roll=%f", speedLeft.data,  speedRight.data, joystick1Pitch, joystick1Roll);

    driveLeftSpeedPublisher->publish(speedLeft);
    driveRightSpeedPublisher->publish(speedRight);
}

/** @brief Callback function for joystick axis topic
 * 
 * This function is called when the node receives the
 * topic joystick_axis, then converts the joystick
 * input using a series of linear transformations 
 * before calling UpdateSpeed()
 * @param axisState \see AxisState.msg
 * @return void
 * */
void joystickAxisCallback(const messages::msg::AxisState::SharedPtr axisState){
    RCLCPP_INFO(nodeHandle->get_logger(),"Button %d %d %f", axisState->joystick, axisState->axis, axisState->state);
    float deadZone = 0.1;
    joystick1Roll = -axisState->state0;
    joystick1Roll = (fabs(joystick1Roll)<deadZone)? 0.0 : joystick1Roll;
    joystick1Roll = (joystick1Roll>0)?joystick1Roll-deadZone:joystick1Roll;
    joystick1Roll = (joystick1Roll<0)?joystick1Roll+deadZone:joystick1Roll;
    joystick1Pitch = axisState->state1;
    joystick1Pitch = (fabs(joystick1Pitch)<deadZone)? 0.0 : joystick1Pitch;
    joystick1Pitch = (joystick1Pitch>0)?joystick1Pitch-deadZone:joystick1Pitch;
    joystick1Pitch = (joystick1Pitch<0)?joystick1Pitch+deadZone:joystick1Pitch;
    joystick1Yaw = axisState->state2;
    updateSpeed();
    //std_msgs::msg::Float32 auger;
/*    
    if(axisState->axis==0){
        joystick1Roll = -axisState->state;
        joystick1Roll = (fabs(joystick1Roll)<deadZone)? 0.0 : joystick1Roll;
	    joystick1Roll = (joystick1Roll>0)?joystick1Roll-deadZone:joystick1Roll;
	    joystick1Roll = (joystick1Roll<0)?joystick1Roll+deadZone:joystick1Roll;
        updateSpeed();
    }else if(axisState->axis==1){
        joystick1Pitch = axisState->state;
        joystick1Pitch = (fabs(joystick1Pitch)<deadZone)? 0.0 : joystick1Pitch;
	    joystick1Pitch = (joystick1Pitch>0)?joystick1Pitch-deadZone:joystick1Pitch;
	    joystick1Pitch = (joystick1Pitch<0)?joystick1Pitch+deadZone:joystick1Pitch;
        updateSpeed();
    }else if(axisState->axis==2){
        joystick1Yaw = axisState->state;
    }else if(axisState->axis==3){
    }
*/
}

/** @brief Callback function for joystick buttons
 * 
 * This function is called when the node receives a
 * topic with the name joystick_button.  It currently
 * prints the button pressed to the screen.
 * @param buttonState \see ButtonState.msg
 * @return void
 * */
void joystickButtonCallback(const messages::msg::ButtonState::SharedPtr buttonState){
    std::cout << "Button " << buttonState->joystick << " " << buttonState->button << " " << buttonState->state << std::endl;

    switch (buttonState->button) {

        case 0: //ESTOP
//            DO NOT USE!!!
            break;
        case 1: //toggles driving and digging
            break;
        case 2:
            break;
        case 3:
            if (buttonState->state) {
            }else{
            }
            break;
        case 4:
            if(buttonState->state) {
            }else {
            }
            break;
        case 5: 
            if(buttonState->state) {
            }else {
            }
            break;
        case 6: 
            break;
        case 7:
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
 * topic with the name joystick_hat.  It currently
 * prints the hat state to the screen.
 * @param hatState \see HatState.msg
 * @return void
 * */
void joystickHatCallback(const messages::msg::HatState::SharedPtr hatState){
    std::cout << "Hat " << hatState->joystick << " " << hatState->hat << " " << hatState->state << std::endl;
}

/** @brief Callback function for the keys
 * 
 * This function is called when the node receives a
 * topic with the name key_state.  It currently prints
 * the key pressed to the screen and inverts  
 * @arg automationGo if the key is 's'.
 * @param keyState \see KeyState.msg
 * @return void
 * */
void keyCallback(const messages::msg::KeyState::SharedPtr keyState){
    std::cout << "Key " << keyState->key << " " << keyState->state << std::endl;

    if(keyState->key==115 && keyState->state==1){
        automationGo= !automationGo;
    }
}

/** @brief Callback function for the zedPosition
 * 
 * This function is caled when the node receives a
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
    position.arucoVisible=zedPosition->aruco_visible;	
    automation->setPosition(position);
}


int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    nodeHandle = rclcpp::Node::make_shared("logic");
    automation->setNode(nodeHandle);

    auto joystickAxisSubscriber= nodeHandle->create_subscription<messages::msg::AxisState>("joystick_axis",1,joystickAxisCallback);
    auto joystickButtonSubscriber= nodeHandle->create_subscription<messages::msg::ButtonState>("joystick_button",1,joystickButtonCallback);
    auto joystickHatSubscriber= nodeHandle->create_subscription<messages::msg::HatState>("joystick_hat",1,joystickHatCallback);
    auto keySubscriber= nodeHandle->create_subscription<messages::msg::KeyState>("key",1,keyCallback);
    auto zedPositionSubscriber= nodeHandle->create_subscription<messages::msg::ZedPosition>("zed_position",1,zedPositionCallback);
    
    driveLeftSpeedPublisher= nodeHandle->create_publisher<std_msgs::msg::Float32>("drive_left_speed",1);
    driveRightSpeedPublisher= nodeHandle->create_publisher<std_msgs::msg::Float32>("drive_right_speed",1);
    driveStatePublisher= nodeHandle->create_publisher<std_msgs::msg::Bool>("drive_state",1);

    initSetSpeed();

    rclcpp::Rate rate(20);

    while(rclcpp::ok()){
	if(automationGo) automation->automate();
        rclcpp::spin_some(nodeHandle);
	rate.sleep();
    }
}