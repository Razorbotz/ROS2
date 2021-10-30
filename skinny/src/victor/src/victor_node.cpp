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
void stopCallback(std_msgs::msg::Empty::SharedPtr) {
	std::cout << "STOP" << std::endl;
	GO = false;
}
void goCallback(std_msgs::msg::Empty::SharedPtr) {
	std::cout << "GO" << std::endl;
	GO = true;
}

VictorSPX* victorSPX;
void speedCallback(const std_msgs::msg::Float32::SharedPtr speed) {
	std::cout << speed->data << std::endl;
	victorSPX->Set(ControlMode::PercentOutput, speed->data);
}

int main(int argc, char** argv) {
	std::cout << "Starting victor" << std::endl;
	rclcpp::init(argc, argv);
	auto nodeHandle = rclcpp::Node::make_shared("victor");

	int success;

	std::string message;
	success = nodeHandle->get_parameter("message", message);
	std::cout << success << "message " << message << std::endl;

	int motorNumber = 0;
	success = nodeHandle->get_parameter("motor_number", motorNumber);
	std::cout << success << "motor_number: " << motorNumber << std::endl;

	std::string infoTopic;
	success = nodeHandle->get_parameter("info_topic", infoTopic);
	std::cout << success << "info_topic: " << infoTopic << std::endl;

	std::string speedTopic;
	success = nodeHandle->get_parameter("speed_topic", speedTopic);
	std::cout << success << "speed_topic: " << speedTopic << std::endl;

	bool invertMotor = false;
	success = nodeHandle->get_parameter("invert_motor", invertMotor);
	std::cout << success << "invert_motor: " << invertMotor << std::endl;

	ctre::phoenix::platform::can::SetCANInterface("can0");

	victorSPX = new VictorSPX(motorNumber);
	victorSPX->SetInverted(invertMotor);
	victorSPX->Set(ControlMode::PercentOutput, 0);
	//StatusFrame statusFrame;

	messages::msg::VictorOut victorOut;
	auto victorOutPublisher = nodeHandle->create_publisher<messages::msg::VictorOut>(infoTopic.c_str(), 1);
	auto speedSubscriber = nodeHandle->create_subscription<std_msgs::msg::Float32>(speedTopic.c_str(), 1, speedCallback);

	auto stopSubscriber = nodeHandle->create_subscription<std_msgs::msg::Empty>("STOP", 1, stopCallback);
	auto goSubscriber = nodeHandle->create_subscription<std_msgs::msg::Empty>("GO", 1, goCallback);
	auto start = std::chrono::high_resolution_clock::now();
	while(rclcpp::ok()) {
		if(GO) ctre::phoenix::unmanaged::FeedEnable(100);

		auto finish = std::chrono::high_resolution_clock::now();
		if(std::chrono::duration_cast<std::chrono::nanoseconds>(finish - start).count() > 250000000) {
			int deviceID = victorSPX->GetDeviceID();
			double busVoltage = victorSPX->GetBusVoltage();
			//bool isInverted = victorSPX->GetInverted();
			double motorOutputVoltage = victorSPX->GetMotorOutputVoltage();
			double motorOutputPercent = victorSPX->GetMotorOutputPercent();

			victorOut.device_id = deviceID;
			victorOut.bus_voltage = busVoltage;
			victorOut.output_voltage = motorOutputVoltage;
			victorOut.output_percent = motorOutputPercent;

			victorOutPublisher->publish(victorOut);
			start = std::chrono::high_resolution_clock::now();
		}

		usleep(20);
		rclcpp::spin_some(nodeHandle);
	}
}
