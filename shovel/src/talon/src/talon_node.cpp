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
#include <cstdlib>

#include <rclcpp/rclcpp.hpp>
//#include <rclcpp/console.h>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <messages/msg/key_state.hpp>

#define Phoenix_No_WPI // remove WPI dependencies
#include <ctre/Phoenix.h>
#include <ctre/phoenix/platform/Platform.h>
#include <ctre/phoenix/unmanaged/Unmanaged.h>
#include <ctre/phoenix/cci/Unmanaged_CCI.h>
#include <ctre/phoenix/cci/Diagnostics_CCI.h>

#include "messages/msg/talon_out.hpp"

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
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String_<std::allocator<void> >, std::allocator<void> > > resetPublisher;
bool GO=false;
std::chrono::time_point<std::chrono::high_resolution_clock> commPrevious;
std::chrono::time_point<std::chrono::high_resolution_clock> logicPrevious;
bool printData = false;
std::string resetString = "";

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

void logicHeartbeatCallback(std_msgs::msg::Empty::SharedPtr empty){
	logicPrevious = std::chrono::high_resolution_clock::now();
}

TalonSRX* talonSRX;
bool TEMP_DISABLE = false;

// Operating modes:
// 0 - Normal
// 1 - Critical
// 2 - Emergency 
int op_mode = 0;
int killKey = 0;

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
	talonSRX->Set(ControlMode::PercentOutput, speed->data);
}

void positionCallback(const std_msgs::msg::Int32::SharedPtr position){
	if(printData)
		RCLCPP_INFO(nodeHandle->get_logger(),"Position---------->>> %d ", position->data);
	//std::cout << "---------->>>  " << speed->data << std::endl;
	talonSRX->Set(ControlMode::Position, position->data);
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
	if(keyState->key == 98 && keyState->state==1){
		std_msgs::msg::String reset;
		reset.data = resetString;
		resetPublisher->publish(reset);
	}
}


int main(int argc,char** argv){
	rclcpp::init(argc,argv);
	nodeHandle = rclcpp::Node::make_shared("talon");

	RCLCPP_INFO(nodeHandle->get_logger(),"Starting talon");
	//int success;

	int motorNumber = getParameter<int>("motor_number", 1);
	int portNumber = getParameter<int>("diagnostics_port", 1);
	//c_SetPhoenixDiagnosticsStartTime(-1); //Disables the Phoenix Diagnostics server, but does not allow the Talons to run
	c_Phoenix_Diagnostics_Create1(portNumber);  //Creates a Phoenix Diagnostics server with the port specified
	
	std::string infoTopic = getParameter<std::string>("info_topic", "unset");
	std::string potentiometerTopic = getParameter<std::string>("potentiometer_topic", "unset");
	std::string speedTopic = getParameter<std::string>("speed_topic", "unset");
	std::string positionTopic = getParameter<std::string>("position_topic", "unset");

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
	resetString = getParameter<std::string>("reset_topic", "1");

	ctre::phoenix::platform::can::SetCANInterface(can_interface.c_str());
	RCLCPP_INFO(nodeHandle->get_logger(),"Opened CAN interface");

	int kTimeoutMs=30;
	int kPIDLoopIdx=0;
	//int kSlotIdx=0;
	talonSRX=new TalonSRX(motorNumber);
	RCLCPP_INFO(nodeHandle->get_logger(),"created talon instance");

	talonSRX->SetInverted(invertMotor);
	talonSRX->SelectProfileSlot(0,0);
	talonSRX->ConfigSelectedFeedbackSensor(FeedbackDevice::Analog, 0, kTimeoutMs);
	talonSRX->SetSensorPhase(true);
	talonSRX->ConfigClosedloopRamp(2);
	talonSRX->ConfigNominalOutputForward(0, kTimeoutMs);
	talonSRX->ConfigNominalOutputReverse(0, kTimeoutMs);
	talonSRX->ConfigPeakOutputForward(1, kTimeoutMs);
	talonSRX->ConfigPeakOutputReverse(-1, kTimeoutMs);
	talonSRX->Config_kF(kPIDLoopIdx, kF, kTimeoutMs);
	talonSRX->Config_kP(kPIDLoopIdx, kP, kTimeoutMs);
	talonSRX->Config_kI(kPIDLoopIdx, kI, kTimeoutMs);
	talonSRX->Config_kD(kPIDLoopIdx, kD, kTimeoutMs);
	talonSRX->ConfigAllowableClosedloopError(kPIDLoopIdx,0,kTimeoutMs);

	talonSRX->Set(ControlMode::PercentOutput, 0);
	talonSRX->Set(ControlMode::Position, 500);
	//talonSRX->SetFeedbackDevice(FeedbackDevice.AnalogPotentiometer);
	talonSRX->SetStatusFramePeriod(StatusFrame::Status_2_Feedback0_, 10, 10);

	RCLCPP_INFO(nodeHandle->get_logger(),"configured talon");

	TalonSRXConfiguration allConfigs;

	messages::msg::TalonOut talonOut;
	auto talonOutPublisher=nodeHandle->create_publisher<messages::msg::TalonOut>(infoTopic.c_str(),1);
	auto potentiometerPublisher=nodeHandle->create_publisher<std_msgs::msg::Int32>(potentiometerTopic.c_str(),1);
	auto speedSubscriber=nodeHandle->create_subscription<std_msgs::msg::Float32>(speedTopic.c_str(),1,speedCallback);
	auto positionSubscriber=nodeHandle->create_subscription<std_msgs::msg::Int32>(positionTopic.c_str(),1,positionCallback);
	resetPublisher=nodeHandle->create_publisher<std_msgs::msg::String>("reset_topic",1);

	auto stopSubscriber=nodeHandle->create_subscription<std_msgs::msg::Empty>("STOP",1,stopCallback);
	auto goSubscriber=nodeHandle->create_subscription<std_msgs::msg::Empty>("GO",1,goCallback);
	auto commHeartbeatSubscriber = nodeHandle->create_subscription<std_msgs::msg::Empty>("comm_heartbeat",1,commHeartbeatCallback);
	auto logicHeartbeatSubscriber = nodeHandle->create_subscription<std_msgs::msg::Empty>("logic_heartbeat",1,logicHeartbeatCallback);
	auto keySubscriber= nodeHandle->create_subscription<messages::msg::KeyState>("key",1,keyCallback);
	
	RCLCPP_INFO(nodeHandle->get_logger(),"set subscribers");

	rclcpp::Rate rate(30);
	auto start2 = std::chrono::high_resolution_clock::now();
	auto start = std::chrono::high_resolution_clock::now();
	float maxCurrent = 0.0;
	double busVoltage = 0.0;

	while(rclcpp::ok()){
		if(GO)ctre::phoenix::unmanaged::FeedEnable(100);
		auto finish = std::chrono::high_resolution_clock::now();

		if(std::chrono::duration_cast<std::chrono::milliseconds>(finish-start).count() > publishingDelay){

			int deviceID=talonSRX->GetDeviceID();
			busVoltage=talonSRX->GetBusVoltage();
			double outputCurrent=talonSRX->GetOutputCurrent();
			bool isInverted=talonSRX->GetInverted();
			double motorOutputVoltage=talonSRX->GetMotorOutputVoltage();
			double motorOutputPercent=talonSRX->GetMotorOutputPercent();
			double temperature=talonSRX->GetTemperature();				
			int sensorPosition0=talonSRX->GetSelectedSensorPosition(0);
			int sensorVelocity0=talonSRX->GetSelectedSensorVelocity(0);
			int closedLoopError0=talonSRX->GetClosedLoopError(0);
			double integralAccumulator0=talonSRX->GetIntegralAccumulator(0);
			double errorDerivative0=talonSRX->GetErrorDerivative(0);
		
			talonOut.device_id=deviceID;	
			talonOut.bus_voltage=busVoltage;
			talonOut.output_current=outputCurrent;
			talonOut.output_voltage=motorOutputVoltage;
			talonOut.output_percent=motorOutputPercent;
			talonOut.temperature=temperature;
			talonOut.sensor_position=sensorPosition0;
			talonOut.sensor_velocity=sensorVelocity0;
			talonOut.closed_loop_error=closedLoopError0;
			talonOut.integral_accumulator=integralAccumulator0;
			talonOut.error_derivative=errorDerivative0;
			talonOut.temp_disable = TEMP_DISABLE;
			if(outputCurrent > maxCurrent){
				maxCurrent = outputCurrent;
			}
			talonOut.max_current = maxCurrent;
			talonOutPublisher->publish(talonOut);
			checkTemperature(temperature);
        	start = std::chrono::high_resolution_clock::now();
		}
		if(std::chrono::duration_cast<std::chrono::milliseconds>(finish-commPrevious).count() > 100 || TEMP_DISABLE
		||	std::chrono::duration_cast<std::chrono::milliseconds>(finish-logicPrevious).count() > 100 ){
			if(TEMP_DISABLE){
				RCLCPP_INFO(nodeHandle->get_logger(),"Temp Disable");
			}
			talonSRX->Set(ControlMode::PercentOutput, 0.0);
			GO = false;
		}
		rate.sleep();
		rclcpp::spin_some(nodeHandle);
	}
}

