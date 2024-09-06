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

#define Phoenix_No_WPI // remove WPI dependencies
#include <ctre/Phoenix.h>
#include <ctre/phoenix/platform/Platform.h>
#include <ctre/phoenix/unmanaged/Unmanaged.h>
#include <ctre/phoenix/cci/Unmanaged_CCI.h>
#include <ctre/phoenix/cci/Diagnostics_CCI.h>

#include "messages/msg/falcon_out.hpp"

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

/** @file
 * @brief Node controlling one Talon motor 
 * 
 * This node receives information published by the logic node,
 * then transforms the data received into movement by the motor
 * controlled by the Talon instance.  The topics that the node
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
 * This string has the general form talon_{motorNumber}_info and
 * is defined by the user in the launch file.  To read more about
 * the launch file,
 * \see launch.py
 * 
 * */


rclcpp::Node::SharedPtr nodeHandle;
bool GO=false;
std::chrono::time_point<std::chrono::high_resolution_clock> commPrevious;
TalonFX* talonFX;
bool useVelocity=false;

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
	if(useVelocity){
		talonFX->Set(ControlMode::Velocity, 0);
	}
	else{
		talonFX->Set(ControlMode::PercentOutput, 0.0);
	}
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

void commHeartbeatCallback(std_msgs::msg::Empty::SharedPtr empty){
	commPrevious = std::chrono::high_resolution_clock::now();
}

int velocityMultiplier=0;
int testSpeed=0;

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
	RCLCPP_INFO(nodeHandle->get_logger(),"---------->>> %f ", speed->data);
	//std::cout << "---------->>>  " << speed->data << std::endl;

	if(useVelocity){
		talonFX->Set(ControlMode::Velocity, int(speed->data*velocityMultiplier));
	}
	else{
		talonFX->Set(ControlMode::PercentOutput, speed->data);
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
	RCLCPP_INFO(nodeHandle->get_logger(), output.c_str());
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
	if(typeid(value).name() == typeid(int).name())
		value = param.as_int();
	if(typeid(value).name() == typeid(double).name())
		value = param.as_double();
	if(typeid(value).name() == typeid(bool).name())
		value = param.as_bool();
	std::cout << parameterName << ": " << value << std::endl;
	std::string output = parameterName + ": " + std::to_string(value);
	RCLCPP_INFO(nodeHandle->get_logger(), output.c_str());
	return value;
}

int main(int argc,char** argv){
	rclcpp::init(argc,argv);
	nodeHandle = rclcpp::Node::make_shared("talon");

	RCLCPP_INFO(nodeHandle->get_logger(),"Starting talon");
	//int success;

	int motorNumber = getParameter<int>("motor_number", 1);
	int portNumber = getParameter<int>("diagnostics_port", 1);
	c_Phoenix_Diagnostics_Create1(portNumber);
	std::string infoTopic = getParameter<std::string>("info_topic", "unset");
	std::string speedTopic = getParameter<std::string>("speed_topic", "unset");
	bool invertMotor = getParameter<bool>("invert_motor", 0);
	useVelocity = getParameter<bool>("use_velocity", 0);
	velocityMultiplier = getParameter<int>("velocity_multiplier", 0);
	testSpeed = getParameter<int>("test_speed", 0);
	double kP = getParameter<double>("kP", 1);
	double kI = getParameter<double>("kI", 0);
	double kD = getParameter<double>("kD", 0);
	double kF = getParameter<double>("kF", 0);

	ctre::phoenix::platform::can::SetCANInterface("can0");
	RCLCPP_INFO(nodeHandle->get_logger(),"Opened CAN interface");

	int kTimeoutMs=30;
	int kPIDLoopIdx=0;
	talonFX=new TalonFX(motorNumber);
	RCLCPP_INFO(nodeHandle->get_logger(),"created talon instance");

	if(invertMotor){
		talonFX->SetInverted(TalonFXInvertType::CounterClockwise);
	}
	else{
		talonFX->SetInverted(TalonFXInvertType::Clockwise);
	}
	RCLCPP_INFO(nodeHandle->get_logger(),"here 1");

	talonFX->SelectProfileSlot(0,0);
	talonFX->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, kTimeoutMs);
	talonFX->ConfigClosedloopRamp(2);
	talonFX->ConfigNominalOutputForward(0, kTimeoutMs);
	talonFX->ConfigNominalOutputReverse(0, kTimeoutMs);
	talonFX->ConfigPeakOutputForward(1, kTimeoutMs);
	talonFX->ConfigPeakOutputReverse(-1, kTimeoutMs);
	talonFX->Config_kF(kPIDLoopIdx, kF, kTimeoutMs);
	talonFX->Config_kP(kPIDLoopIdx, kP, kTimeoutMs);
	talonFX->Config_kI(kPIDLoopIdx, kI, kTimeoutMs);
	talonFX->Config_kD(kPIDLoopIdx, kD, kTimeoutMs);
	talonFX->ConfigAllowableClosedloopError(kPIDLoopIdx,0,kTimeoutMs);

	talonFX->Set(ControlMode::PercentOutput, 0);
	talonFX->Set(ControlMode::Velocity, 0);

	RCLCPP_INFO(nodeHandle->get_logger(),"configured talon");

	TalonFXConfiguration allConfigs;

	messages::msg::FalconOut falconOut;
	auto falconOutPublisher=nodeHandle->create_publisher<messages::msg::FalconOut>(infoTopic.c_str(),1);
	auto speedSubscriber=nodeHandle->create_subscription<std_msgs::msg::Float32>(speedTopic.c_str(),1,speedCallback);

	auto stopSubscriber=nodeHandle->create_subscription<std_msgs::msg::Empty>("STOP",1,stopCallback);
	auto goSubscriber=nodeHandle->create_subscription<std_msgs::msg::Empty>("GO",1,goCallback);
	auto commHeartbeatSubscriber = nodeHandle->create_subscription<std_msgs::msg::Empty>("comm_heartbeat",1,commHeartbeatCallback);
	
	RCLCPP_INFO(nodeHandle->get_logger(),"set subscribers");

	rclcpp::Rate rate(20);
	auto start = std::chrono::high_resolution_clock::now();
	while(rclcpp::ok()){
		if(GO)ctre::phoenix::unmanaged::FeedEnable(100);
		auto finish = std::chrono::high_resolution_clock::now();

		if(std::chrono::duration_cast<std::chrono::nanoseconds>(finish-start).count() > 250000000){
			int deviceID=talonFX->GetDeviceID();
			double busVoltage=talonFX->GetBusVoltage();
			double outputCurrent=talonFX->GetOutputCurrent();
			bool isInverted=talonFX->GetInverted();
			double motorOutputVoltage=talonFX->GetMotorOutputVoltage();
			double motorOutputPercent=talonFX->GetMotorOutputPercent();
			double temperature=talonFX->GetTemperature();
			int sensorPosition0=talonFX->GetSelectedSensorPosition(0);
			int sensorVelocity0=talonFX->GetSelectedSensorVelocity(0);
			int closedLoopError0=talonFX->GetClosedLoopError(0);
			double integralAccumulator0=talonFX->GetIntegralAccumulator(0);
			double errorDerivative0=talonFX->GetErrorDerivative(0);
		
			falconOut.device_id=deviceID;	
			falconOut.bus_voltage=busVoltage;
			falconOut.output_current=outputCurrent;
			falconOut.output_voltage=motorOutputVoltage;
			falconOut.output_percent=motorOutputPercent;
			falconOut.temperature=temperature;
			falconOut.sensor_position=sensorPosition0;
			falconOut.sensor_velocity=sensorVelocity0;
			falconOut.closed_loop_error=closedLoopError0;
			falconOut.integral_accumulator=integralAccumulator0;
			falconOut.error_derivative=errorDerivative0;

			falconOutPublisher->publish(falconOut);
			start = std::chrono::high_resolution_clock::now();

		}

		if(std::chrono::duration_cast<std::chrono::milliseconds>(finish-commPrevious).count() > 100){
			talonFX->Set(ControlMode::PercentOutput, 0.0);
			GO = false;
		}
		rate.sleep();
		rclcpp::spin_some(nodeHandle);
	}
}


