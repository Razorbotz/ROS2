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
TalonFX* frontFalconFX;
TalonFX* backFalconFX;

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
        	backFalconFX->Set(ControlMode::Velocity, int(speed->data*velocityMultiplier));s
		//talonSRX->Set(ControlMode::Velocity, testSpeed);
	}else{
        	backFalconFX->Set(ControlMode::PercentOutput, speed->data);
	}
}


int main(int argc,char** argv){
	rclcpp::init(argc,argv);
	nodeHandle = rclcpp::Node::make_shared("falcon");

	RCLCPP_INFO(nodeHandle->get_logger(),"Starting falcon");
	//int success;

	//int motorNumber=0;
	//success=nodeHandleP.getParam("motor_number", motorNumber);
	//nodeHandle->declare_parameter<int>("motor_number",1);
	//rclcpp::Parameter motorNumberParameter = nodeHandle->get_parameter("motor_number");
	//int motorNumber = motorNumberParameter.as_int();
	//std::cout << "motor_number: " << motorNumber << std::endl;
	//RCLCPP_INFO(nodeHandle->get_logger(),"motorNumber: %d", motorNumber);
	nodeHandle->declare_parameter<int>("motor_number", 10);
	rclcpp::Parameter motorNumberParameter = nodeHandle->get_parameter("motor_number");
	int motorNumber = motorNumberParameter.as_int();
	std::cout << "motor_number: " << motorNumber << std::endl;
	RCLCPP_INFO(nodeHandle->get_logger(), "motorNumber: %d", motorNumber);


	nodeHandle->declare_parameter<int>("diagnostics_port",1);
	rclcpp::Parameter portNumberParameter = nodeHandle->get_parameter("diagnostics_port");
	int portNumber = portNumberParameter.as_int();
	std::cout << "diagnostics_port: " << portNumber <<std::endl;
	RCLCPP_INFO(nodeHandle->get_logger(), "diagnosticsPort: %d", portNumber);
	//c_SetPhoenixDiagnosticsStartTime(-1); //Disables the Phoenix Diagnostics server, but does not allow the falcons to run
	c_Phoenix_Diagnostics_Create1(portNumber);  //Creates a Phoenix Diagnostics server with the port specified
	
	//std::string infoTopic;
	//success=nodeHandleP.getParam("info_topic", infoTopic);
	nodeHandle->declare_parameter<std::string>("info_topic","unset");
        rclcpp::Parameter infoTopicParameter = nodeHandle->get_parameter("info_topic");
        std::string infoTopic = infoTopicParameter.as_string();
	std::cout << "info_topic: " << infoTopic << std::endl;
	RCLCPP_INFO(nodeHandle->get_logger(),"infoTopic: %s",infoTopic.c_str());

	//std::string speedTopic;
	//success=nodeHandleP.getParam("speed_topic", speedTopic);
	nodeHandle->declare_parameter<std::string>("speed_topic","unset");
        rclcpp::Parameter speedTopicParameter = nodeHandle->get_parameter("speed_topic");
        std::string speedTopic = speedTopicParameter.as_string();
	std::cout << "speed_topic: " << speedTopic << std::endl;
	RCLCPP_INFO(nodeHandle->get_logger(),"speedTopic: %s",speedTopic.c_str());

	//bool invertMotor=false;
	//success=nodeHandleP.getParam("invert_motor", invertMotor);
	nodeHandle->declare_parameter<bool>("invert_motor",false);
        rclcpp::Parameter invertMotorParameter = nodeHandle->get_parameter("invert_motor");
        bool invertMotor = invertMotorParameter.as_bool();
	std::cout << "invert_motor: " << invertMotor << std::endl;
	RCLCPP_INFO(nodeHandle->get_logger(),"invertMotor: %d",invertMotor);

	//success=nodeHandleP.getParam("use_velocity", useVelocity);
	nodeHandle->declare_parameter<bool>("use_velocity",false);
        rclcpp::Parameter useVelocityParameter = nodeHandle->get_parameter("use_velocity");
        useVelocity = useVelocityParameter.as_bool();
	std::cout << "use_velocity: " << useVelocity << std::endl;
	RCLCPP_INFO(nodeHandle->get_logger(),"useVelocity: %d",useVelocity);

	//success=nodeHandleP.getParam("velocity_multiplier", velocityMultiplier);
	nodeHandle->declare_parameter<int>("velocity_multiplier",0);
        rclcpp::Parameter velocityMultiplierParameter = nodeHandle->get_parameter("velocity_multiplier");
        velocityMultiplier = velocityMultiplierParameter.as_int();
	std::cout << "velocity_multiplier: " << velocityMultiplier << std::endl;
	RCLCPP_INFO(nodeHandle->get_logger(),"velocityMultiplier: %d",velocityMultiplier);

	//success=nodeHandleP.getParam("test_speed", testSpeed);
	nodeHandle->declare_parameter<int>("test_speed",0);
        rclcpp::Parameter testSpeedParameter = nodeHandle->get_parameter("test_speed");
        testSpeed = testSpeedParameter.as_int();
	std::cout << "test_speed: " << testSpeed << std::endl;
	RCLCPP_INFO(nodeHandle->get_logger(),"testSpeed: %d",testSpeed);

	//double kP=0;
	//success=nodeHandleP.getParam("kP", kP);
	nodeHandle->declare_parameter<double>("kP",1);
        rclcpp::Parameter kPParameter = nodeHandle->get_parameter("kP");
        double kP= kPParameter.as_double();
	std::cout << "kP: " << kP << std::endl;
	RCLCPP_INFO(nodeHandle->get_logger(),"kP: %f",kP);

	//double kI=0;
	//success=nodeHandleP.getParam("kI", kI);
	nodeHandle->declare_parameter<double>("kI",0);
        rclcpp::Parameter kIParameter = nodeHandle->get_parameter("kI");
        double kI= kIParameter.as_double();
	std::cout << "kI: " << kI << std::endl;
	RCLCPP_INFO(nodeHandle->get_logger(),"kI: %f",kI);

	//double kD=0;
	//success=nodeHandleP.getParam("kD", kD);
	nodeHandle->declare_parameter<double>("kD",0);
        rclcpp::Parameter kDParameter = nodeHandle->get_parameter("kD");
        double kD= kDParameter.as_double();
	std::cout << "kD: " << kD << std::endl;
	RCLCPP_INFO(nodeHandle->get_logger(),"kD: %f",kD);

	//double kF=0;
	//success=nodeHandleP.getParam("kF", kF);
	nodeHandle->declare_parameter<double>("kF",0);
        rclcpp::Parameter kFParameter = nodeHandle->get_parameter("kF");
        double kF= kFParameter.as_double();
	std::cout << "kF: " << kF << std::endl;
	RCLCPP_INFO(nodeHandle->get_logger(),"kF: %f",kF);

        ctre::phoenix::platform::can::SetCANInterface("can0");
	RCLCPP_INFO(nodeHandle->get_logger(),"Opened CAN interface");

	int kTimeoutMs=30;
	int kPIDLoopIdx=0;
	//int kSlotIdx=0;
	backFalconFX = new TalonFX(motorNumber);
	RCLCPP_INFO(nodeHandle->get_logger(),"created talon instance");

	backFalconFX->SetInverted(invertMotor);
	RCLCPP_INFO(nodeHandle->get_logger(),"here 1");

	backFalconFX->SelectProfileSlot(0,0);
	backFalconFX->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, kTimeoutMs);
	//talonSRX->SetSensorPhase(false);
	backFalconFX->ConfigClosedloopRamp(2);
	backFalconFX->ConfigNominalOutputForward(0, kTimeoutMs);
	backFalconFX->ConfigNominalOutputReverse(0, kTimeoutMs);
	backFalconFX->ConfigPeakOutputForward(1, kTimeoutMs);
	backFalconFX->ConfigPeakOutputReverse(-1, kTimeoutMs);
	backFalconFX->Config_kF(kPIDLoopIdx, kF, kTimeoutMs);
	backFalconFX->Config_kP(kPIDLoopIdx, kP, kTimeoutMs);
	backFalconFX->Config_kI(kPIDLoopIdx, kI, kTimeoutMs);
	backFalconFX->Config_kD(kPIDLoopIdx, kD, kTimeoutMs);
	backFalconFX->ConfigAllowableClosedloopError(kPIDLoopIdx,0,kTimeoutMs);

    backFalconFX->Set(ControlMode::PercentOutput, 0);
    backFalconFX->Set(ControlMode::Velocity, 0);

	frontFalconFX.Follow(backFalconFX);
	RCLCPP_INFO(nodeHandle->get_logger(),"configured talon");

	//TalonSRXPIDSetConfiguration pid;
	TalonFXConfiguration allConfigs;
	//StatusFrame statusFrame;

	messages::msg::FalconOut falconOut;
	//ros::Publisher talonOutPublisher=nodeHandle.advertise<messages::TalonOut>(infoTopic.c_str(),1);
	auto falconOutPublisher=nodeHandle->create_publisher<messages::msg::TalonOut>(infoTopic.c_str(),1);
	//ros::Subscriber speedSubscriber=nodeHandle.subscribe(speedTopic.c_str(),1,speedCallback);
	auto speedSubscriber=nodeHandle->create_subscription<std_msgs::msg::Float32>(speedTopic.c_str(),1,speedCallback);

	//ros::Subscriber stopSubscriber=nodeHandle.subscribe("STOP",1,stopCallback); 
	auto stopSubscriber=nodeHandle->create_subscription<std_msgs::msg::Empty>("STOP",1,stopCallback);
	//ros::Subscriber goSubscriber=nodeHandle.subscribe("GO",1,goCallback); 
	auto goSubscriber=nodeHandle->create_subscription<std_msgs::msg::Empty>("GO",1,goCallback);
	RCLCPP_INFO(nodeHandle->get_logger(),"set subscribers");

	rclcpp::Rate rate(20);
	int count=0;
        auto start = std::chrono::high_resolution_clock::now();
        while(rclcpp::ok()){
		if(GO)ctre::phoenix::unmanaged::FeedEnable(100);
		auto finish = std::chrono::high_resolution_clock::now();

		if(std::chrono::duration_cast<std::chrono::nanoseconds>(finish-start).count() > 250000000){
			int deviceID=backFalconFX->GetDeviceID();
			double busVoltage=backFalconFX->GetBusVoltage();
			double outputCurrent=backFalconFX->GetOutputCurrent();
			bool isInverted=backFalconFX->GetInverted();
			double motorOutputVoltage=backFalconFX->GetMotorOutputVoltage();
			double motorOutputPercent=backFalconFX->GetMotorOutputPercent();
			double temperature=backFalconFX->GetTemperature();
			int sensorPosition0=backFalconFX->GetSelectedSensorPosition(0);
			int sensorVelocity0=backFalconFX->GetSelectedSensorVelocity(0);
			int closedLoopError0=backFalconFX->GetClosedLoopError(0);
			double integralAccumulator0=backFalconFX->GetIntegralAccumulator(0);
			double errorDerivative0=backFalconFX->GetErrorDerivative(0);
		
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

			falconOutPublisher->publish(talonOut);
        	start = std::chrono::high_resolution_clock::now();
		}

		if(count++>200 && GO){
			std::cout <<"V=" << backFalconFX->GetSelectedSensorVelocity(kPIDLoopIdx) <<"  "
				<< "  E=" << backFalconFX->GetClosedLoopError(kPIDLoopIdx) 
				<< "  IA=" << backFalconFX->GetIntegralAccumulator(kPIDLoopIdx)
				<< "  ED=" << backFalconFX->GetErrorDerivative(kPIDLoopIdx) 
				<< std::endl;
			count=0;
		}
        rate.sleep();
		rclcpp::spin_some(nodeHandle);
        }
}


