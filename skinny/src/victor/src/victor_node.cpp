#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <unistd.h>

#define Phoenix_No_WPI // remove WPI dependencies
#include <ctre/Phoenix.h>
#include <ctre/phoenix/platform/Platform.h>
#include <ctre/phoenix/unmanaged/Unmanaged.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "messages/msg/victor_out.hpp"

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

bool GO = false;
void stopCallback(std_msgs::Empty empty) {
	std::cout << "STOP" << std::endl;
	GO = false;
}
void goCallback(std_msgs::Empty empty) {
	std::cout << "GO" << std::endl;
	GO = true;
}

VictorSPX* victorSPX;
void speedCallback(const std_msgs::Float32 speed) {
	std::cout << speed.data << std::endl;
	victorSPX->Set(ControlMode::PercentOutput, speed.data);
}

int main(int argc, char** argv) {
	std::cout << "Starting talon 3" << std::endl;
	ros::init(argc, argv, "talon_node");
	ros::NodeHandle nodeHandleP("~");
	ros::NodeHandle nodeHandle;

	int success;

	std::string message;
	success = nodeHandleP.getParam("message", message);
	std::cout << success << "message " << message << std::endl;

	int motorNumber = 0;
	success = nodeHandleP.getParam("motor_number", motorNumber);
	std::cout << success << "motor_number: " << motorNumber << std::endl;

	std::string infoTopic;
	success = nodeHandleP.getParam("info_topic", infoTopic);
	std::cout << success << "info_topic: " << infoTopic << std::endl;

	std::string speedTopic;
	success = nodeHandleP.getParam("speed_topic", speedTopic);
	std::cout << success << "speed_topic: " << speedTopic << std::endl;

	bool invertMotor = false;
	success = nodeHandleP.getParam("invert_motor", invertMotor);
	std::cout << success << "invert_motor: " << invertMotor << std::endl;

	ctre::phoenix::platform::can::SetCANInterface("can0");

	victorSPX = new VictorSPX(motorNumber);
	victorSPX->SetInverted(invertMotor);
	victorSPX->Set(ControlMode::PercentOutput, 0);
	StatusFrame statusFrame;

	messages::VictorOut victorOut;
	ros::Publisher victorOutPublisher = nodeHandle.advertise<messages::VictorOut>(infoTopic.c_str(), 1);
	ros::Subscriber speedSubscriber = nodeHandle.subscribe(speedTopic.c_str(), 1, speedCallback);

	ros::Subscriber stopSubscriber = nodeHandle.subscribe("STOP", 1, stopCallback);
	ros::Subscriber goSubscriber = nodeHandle.subscribe("GO", 1, goCallback);
	auto start = std::chrono::high_resolution_clock::now();
	while(ros::ok()) {
		if(GO) ctre::phoenix::unmanaged::FeedEnable(100);

		auto finish = std::chrono::high_resolution_clock::now();
		if(std::chrono::duration_cast<std::chrono::nanoseconds>(finish - start).count() > 250000000) {
			int deviceID = victorSPX->GetDeviceID();
			double busVoltage = victorSPX->GetBusVoltage();
			bool isInverted = victorSPX->GetInverted();
			double motorOutputVoltage = victorSPX->GetMotorOutputVoltage();
			double motorOutputPercent = victorSPX->GetMotorOutputPercent();

			victorOut.deviceID = deviceID;
			victorOut.busVoltage = busVoltage;
			victorOut.outputVoltage = motorOutputVoltage;
			victorOut.outputPercent = motorOutputPercent;

			victorOutPublisher.publish(victorOut);
			start = std::chrono::high_resolution_clock::now();
		}

		usleep(20);
		ros::spinOnce();
	}
}
