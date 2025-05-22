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

#include "logic/Automation1.hpp"
#include "logic/AutomationTypes.hpp"


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

float maxSpeed=0.8;

bool automationGo=false;
bool excavationGo = false;
bool printData = false;
bool zedInit = false;

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

    userLeftSpeedPublisher->publish(speedLeft);
    userRightSpeedPublisher->publish(speedRight);
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
    }
    else if(axisState->axis==3){
        joystick1Throttle = axisState->state/2 + 0.5;
        joystick1Throttle = transformJoystickInfo(joystick1Throttle, deadZone);
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
        case 1: //toggles driving and digging
            automation->dumpMacro();
            break;
        case 2:
            if(buttonState->state){ 
                armSpeed.data = -1.0; 
            }
            else{
                armSpeed.data = 0.0;
            }
            armSpeedPublisher->publish(armSpeed);
            automation->stopArmsLevel();
            if(printData)
                RCLCPP_INFO(nodeHandle->get_logger(), "Button 3");
            break;
        case 3:
            if(buttonState->state){
                bucketSpeed.data = -1.0;
            }
            else{
                bucketSpeed.data = 0.0;
            }
            bucketSpeedPublisher->publish(bucketSpeed);
            automation->stopBucketLevel();
            if(printData)
                RCLCPP_INFO(nodeHandle->get_logger(), "Button 4");
            break;
        case 4:
            if(buttonState->state){
                armSpeed.data = 1.0;
            }
            else{
                armSpeed.data = 0.0;
            }
            armSpeedPublisher->publish(armSpeed);
            automation->stopArmsLevel();
            if(printData)
                RCLCPP_INFO(nodeHandle->get_logger(), "Button 5");
            break;
        case 5:
            if(buttonState->state){
                bucketSpeed.data = 1.0;
            }
            else{
                bucketSpeed.data = 0.0;
            }
            bucketSpeedPublisher->publish(bucketSpeed);
            automation->stopBucketLevel();
            if(printData)
                RCLCPP_INFO(nodeHandle->get_logger(), "Button 6");
            break;
        case 6:
            automation->startArmsLevel();
            if(printData)
                RCLCPP_INFO(nodeHandle->get_logger(), "Button 7");
            break;
        case 7:
            automation->startBucketLevel();
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
    if(keyState->key==97 && keyState->state==1){
        automation->startAutonomy();
    }
    if(keyState->key==43 && keyState->state==1){
        updateMaxSpeed(0.1);
    }
    if(keyState->key==45 && keyState->state==1){
        updateMaxSpeed(-0.1);
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
 * @param linearOut 
 */
void linearOut1Callback(const messages::msg::LinearOut::SharedPtr linearOut){
    automation->setLinear1(linearOut);
}


/** @brief Callback function for the LinearOut topic
 * 
 * @param linearOut 
 */
void linearOut2Callback(const messages::msg::LinearOut::SharedPtr linearOut){
    automation->setLinear2(linearOut);
}


/** @brief Callback function for the LinearOut topic
 * 
 * @param linearOut 
 */
void linearOut3Callback(const messages::msg::LinearOut::SharedPtr linearOut){
    automation->setLinear3(linearOut);
}


/** @brief Callback function for the LinearOut topic
 * 
 * @param linearOut 
 */
void linearOut4Callback(const messages::msg::LinearOut::SharedPtr linearOut){
    automation->setLinear4(linearOut);
}


void talon1Callback(const messages::msg::TalonOut::SharedPtr talonOut){
    automation->setTalon1(talonOut);
    talon1Previous = std::chrono::high_resolution_clock::now();
}

void talon2Callback(const messages::msg::TalonOut::SharedPtr talonOut){
    automation->setTalon2(talonOut);
    talon2Previous = std::chrono::high_resolution_clock::now();
}

void talon3Callback(const messages::msg::TalonOut::SharedPtr talonOut){
    automation->setTalon3(talonOut);
    talon3Previous = std::chrono::high_resolution_clock::now();
}

void talon4Callback(const messages::msg::TalonOut::SharedPtr talonOut){
    automation->setTalon4(talonOut);
    talon4Previous = std::chrono::high_resolution_clock::now();
}


void falcon1Callback(const messages::msg::FalconOut::SharedPtr falconOut){
    automation->setFalcon1(falconOut);
    falcon1Previous = std::chrono::high_resolution_clock::now();
}

void falcon2Callback(const messages::msg::FalconOut::SharedPtr falconOut){
    automation->setFalcon2(falconOut);
    falcon2Previous = std::chrono::high_resolution_clock::now();
}

void falcon3Callback(const messages::msg::FalconOut::SharedPtr falconOut){
    automation->setFalcon3(falconOut);
    falcon3Previous = std::chrono::high_resolution_clock::now();
}

void falcon4Callback(const messages::msg::FalconOut::SharedPtr falconOut){
    automation->setFalcon4(falconOut);
    falcon4Previous = std::chrono::high_resolution_clock::now();
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
    nodeHandle = rclcpp::Node::make_shared("logic");

    std::string mapUsed = getParameter<std::string>("map", "NASA");
    bool turnLeft = getParameter<bool>("turnLeft", false);
    printData = getParameter<bool>("print_data", false);
    automation = new Automation1();
    automation->setMap(mapUsed);
    automation->setTurnLeft(turnLeft);
    automation->setNode(nodeHandle);

    auto joystickAxisSubscriber= nodeHandle->create_subscription<messages::msg::AxisState>("joystick_axis",1,joystickAxisCallback);
    auto joystickButtonSubscriber= nodeHandle->create_subscription<messages::msg::ButtonState>("joystick_button",1,joystickButtonCallback);
    auto joystickHatSubscriber= nodeHandle->create_subscription<messages::msg::HatState>("joystick_hat",1,joystickHatCallback);
    auto keySubscriber= nodeHandle->create_subscription<messages::msg::KeyState>("key",1,keyCallback);
    auto zedPositionSubscriber= nodeHandle->create_subscription<messages::msg::ZedPosition>("zed_position",1,zedPositionCallback);
    
    auto linearOut1Subscriber = nodeHandle->create_subscription<messages::msg::LinearOut>("linearOut1",1,linearOut1Callback);
    auto linearOut2Subscriber = nodeHandle->create_subscription<messages::msg::LinearOut>("linearOut2",1,linearOut2Callback);
    auto linearOut3Subscriber = nodeHandle->create_subscription<messages::msg::LinearOut>("linearOut3",1,linearOut3Callback);
    auto linearOut4Subscriber = nodeHandle->create_subscription<messages::msg::LinearOut>("linearOut4",1,linearOut4Callback);
    
    auto talon1Subscriber = nodeHandle->create_subscription<messages::msg::TalonOut>("talon_14_info",1,talon1Callback);
    auto talon2Subscriber = nodeHandle->create_subscription<messages::msg::TalonOut>("talon_15_info",1,talon2Callback);
    auto talon3Subscriber = nodeHandle->create_subscription<messages::msg::TalonOut>("talon_16_info",1,talon3Callback);
    auto talon4Subscriber = nodeHandle->create_subscription<messages::msg::TalonOut>("talon_17_info",1,talon4Callback);
    
    auto falcon1Subscriber = nodeHandle->create_subscription<messages::msg::FalconOut>("talon_10_info",1,falcon1Callback);
    auto falcon2Subscriber = nodeHandle->create_subscription<messages::msg::FalconOut>("talon_11_info",1,falcon2Callback);
    auto falcon3Subscriber = nodeHandle->create_subscription<messages::msg::FalconOut>("talon_12_info",1,falcon3Callback);
    auto falcon4Subscriber = nodeHandle->create_subscription<messages::msg::FalconOut>("talon_13_info",1,falcon4Callback);

    driveLeftSpeedPublisher= nodeHandle->create_publisher<std_msgs::msg::Float32>("drive_left_speed",1);
    driveRightSpeedPublisher= nodeHandle->create_publisher<std_msgs::msg::Float32>("drive_right_speed",1);
    armSpeedPublisher= nodeHandle->create_publisher<std_msgs::msg::Float32>("arm_speed",1);
    bucketSpeedPublisher= nodeHandle->create_publisher<std_msgs::msg::Float32>("bucket_speed",1);
    automationGoPublisher = nodeHandle->create_publisher<std_msgs::msg::Bool>("automationGo",1);
    userLeftSpeedPublisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("user_left_speed",1);
    userRightSpeedPublisher= nodeHandle->create_publisher<std_msgs::msg::Float32>("user_right_speed",1);

    auto logicHeartbeatPublisher = nodeHandle->create_publisher<std_msgs::msg::Empty>("logic_heartbeat",1);

    initSetSpeed();

    rclcpp::Rate rate(30);
    while(rclcpp::ok()){
		auto finish = std::chrono::high_resolution_clock::now();
        if(automationGo){
            if(checkTimes()){
                automation->automate();
            }
        }
        automation->publishAutomationOut();
        rclcpp::spin_some(nodeHandle);
        logicHeartbeatPublisher->publish(heartbeat);
        rate.sleep();
    }
    rclcpp::shutdown();
}
