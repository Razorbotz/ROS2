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
#include <messages/msg/key_state.hpp>

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
bool TEMP_DISABLE = false;
float Speed = 0.0;
bool error = false;

// Operating modes:
// 0 - Normal
// 1 - Critical
// 2 - Emergency 
int op_mode = 0;
int killKey = 0;
bool printData = false;
int errorCounter = 0;

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
	if(printData)
		RCLCPP_INFO(nodeHandle->get_logger(),"STOP");
	GO=false;
	talonFX->Set(ControlMode::PercentOutput, 0.0);
	Speed = 0.0;
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
	if(printData)
		RCLCPP_INFO(nodeHandle->get_logger(),"GO");
	GO=true;
}

void commHeartbeatCallback(std_msgs::msg::Empty::SharedPtr empty){
	commPrevious = std::chrono::high_resolution_clock::now();
}

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
	if(printData)
		RCLCPP_INFO(nodeHandle->get_logger(),"---------->>> %f ", speed->data);
	//std::cout << "---------->>>  " << speed->data << std::endl;
	//double targetVelocity_RPM = 6000 * speed->data; 
	//talonFX->Set(ControlMode::Velocity, targetVelocity_RPM * 2048 / 600.0);
	talonFX->Set(ControlMode::PercentOutput, speed->data);
	Speed = speed->data;
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


void checkTemperature(double temperature){
	switch(op_mode){
		case 0:
			temperature > 70 ? TEMP_DISABLE = true : TEMP_DISABLE = false;
			break;
		case 1:
			temperature > 80 ? TEMP_DISABLE = true : TEMP_DISABLE = false;
			break;
		case 2:
			temperature > 90 ? TEMP_DISABLE = true : TEMP_DISABLE = false;
			break;
	}
}

void keyCallback(const messages::msg::KeyState::SharedPtr keyState){
    if(printData)
		std::cout << "Key " << keyState->key << " " << keyState->state << std::endl;
    if(keyState->key==killKey && keyState->state==1){
        return;
    }
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
	bool invertMotor = getParameter<bool>("invert_motor", false);
	double kP = getParameter<double>("kP", 1.0);
	double kI = getParameter<double>("kI", 0.0);
	double kD = getParameter<double>("kD", 0.0);
	double kF = getParameter<double>("kF", 0.0);
	int publishingDelay = getParameter<int>("publishing_delay", 0);
	killKey = getParameter<int>("kill_key", 0);
	op_mode = getParameter<int>("op_mode", 0);
	printData = getParameter<bool>("print_data", false);
	std::string can_interface = getParameter<std::string>("can_interface", "can0");

	ctre::phoenix::platform::can::SetCANInterface(can_interface.c_str());
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
	talonFX->SelectProfileSlot(0,0);
	talonFX->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, kTimeoutMs);
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

	RCLCPP_INFO(nodeHandle->get_logger(),"configured falcon");

	TalonFXConfiguration allConfigs;

	ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration supplyLimitConfig;
    supplyLimitConfig.enable = true;
    supplyLimitConfig.currentLimit = 70.0;
    supplyLimitConfig.triggerThresholdCurrent = 75.0;
    supplyLimitConfig.triggerThresholdTime = 0.1; 
	talonFX->ConfigSupplyCurrentLimit(supplyLimitConfig, kTimeoutMs);

	messages::msg::FalconOut falconOut;
	auto falconOutPublisher=nodeHandle->create_publisher<messages::msg::FalconOut>(infoTopic.c_str(),1);
	auto speedSubscriber=nodeHandle->create_subscription<std_msgs::msg::Float32>(speedTopic.c_str(),1,speedCallback);

	auto stopSubscriber=nodeHandle->create_subscription<std_msgs::msg::Empty>("STOP",1,stopCallback);
	auto goSubscriber=nodeHandle->create_subscription<std_msgs::msg::Empty>("GO",1,goCallback);
	auto commHeartbeatSubscriber = nodeHandle->create_subscription<std_msgs::msg::Empty>("comm_heartbeat",1,commHeartbeatCallback);
	auto keySubscriber= nodeHandle->create_subscription<messages::msg::KeyState>("key",1,keyCallback);

	RCLCPP_INFO(nodeHandle->get_logger(),"set subscribers");

	rclcpp::Rate rate(20);
	auto start = std::chrono::high_resolution_clock::now();
	float maxCurrent = 0.0;
	double busVoltage = 0.0;
	while(rclcpp::ok()){
		if(GO)ctre::phoenix::unmanaged::FeedEnable(100);
		auto finish = std::chrono::high_resolution_clock::now();

		if(std::chrono::duration_cast<std::chrono::milliseconds>(finish-start).count() > publishingDelay){
			int deviceID=talonFX->GetDeviceID();
			busVoltage=talonFX->GetBusVoltage();
			double outputCurrent=talonFX->GetOutputCurrent();
			bool isInverted=talonFX->GetInverted();
			double motorOutputVoltage=talonFX->GetMotorOutputVoltage();
			double motorOutputPercent=talonFX->GetMotorOutputPercent();
			if(Speed > 0.1 && motorOutputPercent == 0.0){
				errorCounter++;
				if(errorCounter > 5){
					RCLCPP_INFO(nodeHandle->get_logger(), "Falcon %d ERROR", deviceID);
					error = true;
				}
			}
			else{
				if(motorOutputPercent != 0.0)
					errorCounter = 0;
			}
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
			falconOut.temp_disable = TEMP_DISABLE;
			falconOut.error = error;
			if(outputCurrent > maxCurrent){
				maxCurrent = outputCurrent;
			}
			falconOut.max_current = maxCurrent;
			falconOutPublisher->publish(falconOut);
			start = std::chrono::high_resolution_clock::now();
			checkTemperature(temperature);
		}

		if(std::chrono::duration_cast<std::chrono::milliseconds>(finish-commPrevious).count() > 100 ||  TEMP_DISABLE){
			if(TEMP_DISABLE){
				RCLCPP_INFO(nodeHandle->get_logger(),"Temp Disable");
			}
			talonFX->Set(ControlMode::PercentOutput, 0.0);
			GO = false;
		}
		rate.sleep();
		rclcpp::spin_some(nodeHandle);
	}
}


