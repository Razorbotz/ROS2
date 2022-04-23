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
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > dumpBinSpeedPublisher;
//std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool_<std::allocator<void> >, std::allocator<void> > > driveStatePublisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > excavationArmPublisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > excavationDrumPublisher;

void initSetSpeed(){
    std_msgs::msg::Float32 speed;
    speed.data = 0.0;
    
    driveLeftSpeedPublisher->publish(speed);
    driveRightSpeedPublisher->publish(speed);
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
    //RCLCPP_INFO(nodeHandle->get_logger(),"Axis %d %d %f", axisState->joystick, axisState->axis, axisState->state);
    //RCLCPP_INFO(nodeHandle->get_logger(),"Axis %d %f %f %f", axisState->joystick, axisState->state0, axisState->state1, axisState->state2);
    /*
    float deadZone = 0.1;
    //std_msgs::msg::Float32 auger;
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
    */
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
    if(axisState->axis==0){
        joystick1Roll = -axisState->state;
        updateSpeed();
    }else if(axisState->axis==1){
        joystick1Pitch = axisState->state;
        updateSpeed();
    }else if(axisState->axis==2){
        joystick1Yaw = axisState->state;
    }else if(axisState->axis==3){
    }
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
    std_msgs::msg::Float32 speed;

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
            RCLCPP_INFO(nodeHandle->get_logger(), "Button 7");
            speed.data = 1;
            excavationArmPublisher->publish(speed);
            break;
        case 7:
            RCLCPP_INFO(nodeHandle->get_logger(), "Button 8");
            speed.data = -1;
            excavationArmPublisher->publish(speed);
            break;
        case 8:
            RCLCPP_INFO(nodeHandle->get_logger(), "Button 9");
            speed.data = 1;
            excavationDrumPublisher->publish(speed);
            break;
        case 9:
            RCLCPP_INFO(nodeHandle->get_logger(), "Button 10");
            speed.data = -1;
            excavationDrumPublisher->publish(speed);
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
    //std::cout << "Hat " << (int)hatState->joystick << " " << (int)hatState->hat << " " << (int)hatState->state << std::endl;

    std_msgs::msg::Float32 dumpSpeed;
    if((int)hatState->state == 1 ){
	dumpSpeed.data=0.2;
        dumpBinSpeedPublisher->publish(dumpSpeed);
    }
    if((int)hatState->state == 4 ){
	dumpSpeed.data=-0.2;
        dumpBinSpeedPublisher->publish(dumpSpeed);
    }
    if((int)hatState->state == 0 ){
	dumpSpeed.data=0.0;
        dumpBinSpeedPublisher->publish(dumpSpeed);
    }
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
    double yawRadians=automation->orientation.roll;

    double facingUnitX=-sin(yawRadians);
    double facingUnitZ=cos(yawRadians);
    double directionX=-5-position.x;
    double directionZ=2-position.z;

    double theta = acos((facingUnitX*directionX + facingUnitZ*directionZ)/(sqrt(directionX*directionX + directionZ*directionZ)))*180/M_PI;
    double yaw = yawRadians * 180/M_PI;
    double deltaYaw = theta-yaw;
    double yawTolerance=5;
    std::cout << "roll:" << automation->orientation.roll*180/M_PI << ", pitch:" << automation->orientation.pitch*180/M_PI << ", yaw" << automation->orientation.yaw*180/ M_PI << "   "
              << "   \tx:" << position.x << " y: " << position.y << " z:" << position.z
              << "   \tox:" << position.ox << "  oy:" << position.oy << " oz:" << position.oz << "  ow:" << position.ow
              << "   \tfUX:" << facingUnitX << " fUZ:" << facingUnitZ << "   yaw:" << yaw << " dYaw:" << deltaYaw << " theta:" << theta
              << "   \tvisible:" << position.arucoVisible << std::endl;
    RCLCPP_INFO(nodeHandle->get_logger(), "roll: %f, pitch: %f, yaw: %f", automation->orientation.roll*180/M_PI, automation->orientation.pitch*180/M_PI, automation->orientation.yaw*180/ M_PI);
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
    //driveStatePublisher= nodeHandle->create_publisher<std_msgs::msg::Bool>("drive_state",1);
    dumpBinSpeedPublisher= nodeHandle->create_publisher<std_msgs::msg::Float32>("dump_bin_speed",1);
    excavationArmPublisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("excavationArm",1);
    excavationDrumPublisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("excavationDrum",1);
 
    initSetSpeed();

    rclcpp::Rate rate(20);
    while(rclcpp::ok()){
	if(automationGo) automation->automate();
        rclcpp::spin_some(nodeHandle);
	rate.sleep();
    }
}
