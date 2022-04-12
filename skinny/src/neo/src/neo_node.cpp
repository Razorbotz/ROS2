#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <typeinfo>


#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <linux/if_packet.h>
#include <thread>
#include <chrono>
#include <linux/reboot.h>
#include <sys/reboot.h>


#include <rclcpp/rclcpp.hpp>
//#include <rclcpp/console.h>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/empty.hpp>

#include <rev/CANSparkMax.h>

/** @file
 * @brief Node controlling one Neo motor 
 * 
 * This node receives information published by the logic node,
 * then transforms the data received into movement by the motor
 * controlled by the Neo instance.  The topics that the node
 * subscribes to are as follows:
 * \li \b speed_topic
 * \li \b STOP
 * \li \b GO
 * 
 * The \b speed_topic topic is either \b drive_left_speed
 * or \b drive_right_speed as defined in the parameters set in
 * the launch file.  To read more about the logic node or the
 * launch file
 * \see logic_node.cpp
 * \see launch.py
 * 
 * The topics being published are as follows:
 * \li \b info_topic
 * 
 * This string has the general form neo_{motorNumber}_info and
 * is defined by the user in the launch file.  To read more about
 * the launch file,
 * \see launch.py
 * 
 * */

rclcpp::Node::SharedPtr nodeHandle;
//bool GO=false;
bool GO = true;

/** @brief STOP Callback
 * 
 * Callback function triggered when the node receives
 * a topic with the topic name of STOP.  This function
 * sets a boolean value GO to false, which prevents the
 * robot from moving.
 * @param empty
 * @return void
 * */
void stopCallback(std_msgs::msg::Empty::SharedPtr empty){
	RCLCPP_INFO(nodeHandle->get_logger(),"STOP");
	GO=false;
} 

/** @brief GO Callback
 * 
 * Callback function triggered when the node receives
 * a topic with the topic name of GO.  This function
 * sets a boolean value GO to true, which allows the
 * robot to drive.
 * @param empty
 * @return void
 * */
void goCallback(std_msgs::msg::Empty::SharedPtr empty){
	RCLCPP_INFO(nodeHandle->get_logger(),"GO");
	GO=true;
}

bool useVelocity=false;
int velocityMultiplier=0;
int testSpeed=0;
TalonSRX* talonSRX;

/** @brief Speed Callback Function
 * 
 * Callback function triggered when the node receives
 * a topic with the topic name of drive_left_speed or
 * drive_right_speed.  This function takes the data
 * from the topic and sets the motor to the speed
 * specified.
 * @param speed
 * @return void
 * */
void speedCallback(const std_msgs::msg::Float32::SharedPtr speed){
//	RCLCPP_INFO(nodeHandle->get_logger(),"---------->>> %f ", speed->data);
	//std::cout << "---------->>>  " << speed->data << std::endl;

	if(useVelocity){
        	talonSRX->Set(ControlMode::Velocity, int(speed->data*velocityMultiplier));
		//talonSRX->Set(ControlMode::Velocity, testSpeed);
	}else{
        	talonSRX->Set(ControlMode::PercentOutput, speed->data);
	}
}

/** @brief String parameter function
 * 
 * Function that takes a string as a parameter containing the
 * name of the parameter that is being parsed from the launch
 * file and the initial value of the parameter as inputs, then
 * gets the parameter, casts it as a string, displays the value
 * of the parameter on the command line and the log file, then
 * returns the parsed value of the parameter.
 * @param parametername String of the name of the parameter
 * @param initialValue Initial value of the parameter
 * @return value Value of the parameter
 * */
template <typename T>
T getParameter(std::string parameterName, std::string initialValue){
	nodeHandle->declare_parameter<T>(parameterName, initialValue);
	rclcpp::Parameter param = nodeHandle->get_parameter(parameterName);
	T value = param.as_string();
	std::cout << parameterName << ": " << value << std::endl;
	std::string output = parameterName + ": " + value;
	RCLCPP_INFO(nodeHandle->get_logger(), output);
	return value;
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
T getParameter(std::string parameterName, int initialValue){
	nodeHandle->declare_parameter<T>(parameterName, initialValue);
	rclcpp::Parameter param = nodeHandle->get_parameter(parameterName);
	T value;
	switch(typeid(value).name()){
		case typeid(int).name():
			value = param.as_int();
			break;
		case typeid(double).name():
			value = param.as_double();
			break;
		case typeid(bool).name():
			value = param.as_bool();
			break;
	}
	std::cout << parameterName << ": " << value << std::endl;
	std::string output = parameterName + ": " + std::to_string(value);
	RCLCPP_INFO(nodeHandle->get_logger(), output);
	return value;
}

int main(int argc, char** argv){
	rclcpp::init(argc,argv);
	nodeHandle = rclcpp::Node::make_shared("neo");
	RCLCPP_INFO(nodeHandle->get_logger(),"Starting neo");
	int motorNumber = getParameter<int>("motor_number", 1);
	std::string infoTopic = getParameter<std::string>("info_topic", "unset");
	std::string speedTopic = getParameter<std::string>("speed_topic", "unset");
	bool invertMotor = getParameter<bool>("invert_motor", 0);
	useVelocity = getParameter<bool>("use_velocity", 0);
	velocityMultiplier = getParameter<int>("velocity_multiplier", 0);
	testSpeed = getParameter<int>("test_speed", 0);

	rev::CANSparkMax sparkMax{motorNumber, rev::CANSparkMax::MotorType::kBrushless};
	sparkMax.RestoreFactoryDefaults();

	auto speedSubscriber=nodeHandle->create_subscription<std_msgs::msg::Float32(speedTopic.c_str(),1,speedCallback);
	auto stopSubscriber=nodeHandle->create_subscription<std_msgs::msg::Empty>("STOP",1,stopCallback);
	auto goSubscriber=nodeHandle->create_subscription<std_msgs::msg::Empty("GO", 1, goCallback);
	RCLCPP_INFO(nodeHandle->get_logger(), "Set subscribers");

	RCLCPP_INFO(nodeHandle->get_logger(),"Created NEO");
	while(rclcpp::ok()){
		rclcpp::spin_some(nodeHandle);
	}
}
