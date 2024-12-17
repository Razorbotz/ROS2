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

#include "common/CommonFuncs.h"
#include "messages/msg/falcon_out.hpp"
#include <fstream>

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
bool TEMP_DISABLE = false;
bool VOLT_DISABLE = false;
bool UPDATE_SPEED = false;
bool DISABLE_GO = false;
double speedIncrease = 0.05;
int speedTiming = 100;
float currentSpeed = 0.0;
float expectedSpeed = 0.0;

// Operating modes:
// 0 - Normal
// 1 - Critical
// 2 - Emergency
int op_mode = 0;

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
	DISABLE_GO=true;
	expectedSpeed = 0.0;
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
	UPDATE_SPEED = true;
	expectedSpeed = speed->data;
}


/** @brief Function that updates the current speed closer to the expected
 * 
 * When the motors receive a large change in speed, the amperage needed
 * is very high and can trip the overcurrent protection on the Falcons.
 * The overcurrent protection can only be reset by physically unplugging
 * the motor, so it disables the motor until after the end of the run. By
 * limiting the change in the speed, the current is kept below the threshold
 * and allows for higher speeds to be used.
 */
void updateSpeed(){
	float diff = expectedSpeed - currentSpeed;
	if(std::abs(diff) > speedIncrease){
		if(expectedSpeed < currentSpeed){
			currentSpeed -= speedIncrease;
		}
		else{
			currentSpeed += speedIncrease;
		}
	}
	else{
		currentSpeed = expectedSpeed;
		UPDATE_SPEED = false;
	}
	if(DISABLE_GO && currentSpeed == 0.0){
		DISABLE_GO = false;
		GO = false;
	}
	if(useVelocity){
		talonFX->Set(ControlMode::Velocity, int(currentSpeed*velocityMultiplier));
	}
	else{
		talonFX->Set(ControlMode::PercentOutput, currentSpeed);
	}
}


int main(int argc,char** argv){
	rclcpp::init(argc,argv);
	nodeHandle = rclcpp::Node::make_shared("talon");

	RCLCPP_INFO(nodeHandle->get_logger(),"Starting talon");
	//int success;

	int motorNumber = getParameter<int>("motor_number", 1, &nodeHandle);
	int portNumber = getParameter<int>("diagnostics_port", 1, &nodeHandle);
	c_Phoenix_Diagnostics_Create1(portNumber);
	std::string infoTopic = getParameter<std::string>("info_topic", "unset", &nodeHandle);
	std::string speedTopic = getParameter<std::string>("speed_topic", "unset", &nodeHandle);
	bool invertMotor = getParameter<bool>("invert_motor", 0, &nodeHandle);
	useVelocity = getParameter<bool>("use_velocity", 0, &nodeHandle);
	velocityMultiplier = getParameter<int>("velocity_multiplier", 0, &nodeHandle);
	testSpeed = getParameter<int>("test_speed", 0, &nodeHandle);
	double kP = getParameter<double>("kP", 1, &nodeHandle);
	double kI = getParameter<double>("kI", 0, &nodeHandle);
	double kD = getParameter<double>("kD", 0, &nodeHandle);
	double kF = getParameter<double>("kF", 0, &nodeHandle);
	int publishingDelay = getParameter<int>("publishing_delay", 0, &nodeHandle);
	speedIncrease = getParameter<double>("speed_increase", 0.05, &nodeHandle);
	speedTiming = getParameter<int>("speed_timing", 100, &nodeHandle);
	std::string fileToWrite = getParameter<std::string>("file_name", "unset", &nodeHandle);

	std::ofstream outfile(fileToWrite + "_" + std::to_string(speedIncrease) + "_" + std::to_string(speedTiming) + ".txt");
	int counter = 0;

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
	talonFX->SetStatusFramePeriod(Status_1_General_, 255, 0);
	talonFX->SetStatusFramePeriod(Status_2_Feedback0_, 255, 0);
	talonFX->SetStatusFramePeriod(Status_4_AinTempVbat_, 255, 0);
	talonFX->SetStatusFramePeriod(Status_6_Misc_, 255, 0);
	talonFX->SetStatusFramePeriod(Status_7_CommStatus_, 255, 0);
	talonFX->SetStatusFramePeriod(Status_9_MotProfBuffer_, 255, 0);
	talonFX->SetStatusFramePeriod(Status_10_Targets_, 255, 0);
	talonFX->SetStatusFramePeriod(Status_12_Feedback1_, 255, 0);
	talonFX->SetStatusFramePeriod(Status_13_Base_PIDF0_, 255, 0);
	talonFX->SetStatusFramePeriod(Status_14_Turn_PIDF1_, 255, 0);

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
	auto start2 = std::chrono::high_resolution_clock::now();
	float maxCurrent = 0.0;
	bool WRITE = true;
	while(rclcpp::ok()){
		if(GO)ctre::phoenix::unmanaged::FeedEnable(100);
		auto finish = std::chrono::high_resolution_clock::now();

		if(std::chrono::duration_cast<std::chrono::milliseconds>(finish-start).count() > publishingDelay){
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
			falconOut.temp_disable = TEMP_DISABLE;
			falconOut.volt_disable = VOLT_DISABLE;
			if(outputCurrent > maxCurrent){
				maxCurrent = outputCurrent;
			}
			falconOut.max_current = maxCurrent;
			falconOutPublisher->publish(falconOut);
			start = std::chrono::high_resolution_clock::now();
			checkTemperature(temperature, op_mode, &TEMP_DISABLE);
			checkVoltage(busVoltage, motorOutputPercent, op_mode, &VOLT_DISABLE);
			if(counter < 7500){
				outfile << motorOutputPercent << ", " << outputCurrent << '\n';
				counter += 1;
			}
			else{
				if(WRITE){
					outfile.close();
					WRITE = false;
				}
				RCLCPP_INFO(nodeHandle->get_logger(),"DONE");
			}
		}
		if(UPDATE_SPEED && std::chrono::duration_cast<std::chrono::milliseconds>(finish-start2).count() > speedTiming){
			updateSpeed();
			start2 = std::chrono::high_resolution_clock::now();
		}

		if(std::chrono::duration_cast<std::chrono::milliseconds>(finish-commPrevious).count() > 100){
			talonFX->Set(ControlMode::PercentOutput, 0.0);
			GO = false;
		}
		rate.sleep();
		rclcpp::spin_some(nodeHandle);
	}
}


