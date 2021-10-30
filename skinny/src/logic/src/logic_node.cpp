#include <cmath>
#include <math.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <messages/msg/axis_state.hpp>
#include <messages/msg/button_state.hpp>
#include <messages/msg/hat_state.hpp>
#include <messages/msg/key_state.hpp>
#include <messages/msg/zed_position.hpp>

#include "logic/Automation1.hpp"
#include "logic/AutomationTypes.hpp"

rclcpp::Node::SharedPtr nodeHandle;

float joystick1Roll = 0;
float joystick1Pitch = 0;
float joystick1Yaw = 0;
float joystick1Throttle = 1;
float excavationElevation = 0;
float excavationAuger = 0;
float excavationAngle = 0;
bool reverseAuger = false;

bool automationGo = false;

Automation* automation = new Automation1();

std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void>>, std::allocator<void>>> driveLeftSpeedPublisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void>>, std::allocator<void>>> driveRightSpeedPublisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool_<std::allocator<void>>, std::allocator<void>>> driveStatePublisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void>>, std::allocator<void>>> excavationElevationPublisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void>>, std::allocator<void>>> excavationAugerPublisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void>>, std::allocator<void>>> excavationAnglePublisher;

void updateSpeed() {
	std_msgs::msg::Float32 speedLeft;
	std_msgs::msg::Float32 speedRight;
	float maxSpeed = 0.6;

	//Linear transformation of cordinate planes
	speedLeft.data = (joystick1Pitch + joystick1Roll);
	speedRight.data = (joystick1Pitch - joystick1Roll);

	//multiplied the output speed by the "throttle" value and a limiting precentage
	speedLeft.data = speedLeft.data * maxSpeed;
	speedRight.data = speedRight.data * maxSpeed;

	RCLCPP_INFO(nodeHandle->get_logger(), "speed left=%f right=%f  pitch=%f roll=%f", speedLeft.data, speedRight.data, joystick1Pitch, joystick1Roll);

	driveLeftSpeedPublisher->publish(speedLeft);
	driveRightSpeedPublisher->publish(speedRight);
}

void joystickAxisCallback(const messages::msg::AxisState::SharedPtr axisState) {
	RCLCPP_INFO(nodeHandle->get_logger(), "Button %d %d %f", axisState->joystick, axisState->axis, axisState->state);
	float deadZone = 0.1;
	std_msgs::msg::Float32 auger;
	if(axisState->axis == 0) {
		joystick1Roll = -axisState->state;
		joystick1Roll = (fabs(joystick1Roll) < deadZone) ? 0.0 : joystick1Roll;
		joystick1Roll = (joystick1Roll > 0) ? joystick1Roll - deadZone : joystick1Roll;
		joystick1Roll = (joystick1Roll < 0) ? joystick1Roll + deadZone : joystick1Roll;
		updateSpeed();
	} else if(axisState->axis == 1) {
		joystick1Pitch = axisState->state;
		joystick1Pitch = (fabs(joystick1Pitch) < deadZone) ? 0.0 : joystick1Pitch;
		joystick1Pitch = (joystick1Pitch > 0) ? joystick1Pitch - deadZone : joystick1Pitch;
		joystick1Pitch = (joystick1Pitch < 0) ? joystick1Pitch + deadZone : joystick1Pitch;
		updateSpeed();
	} else if(axisState->axis == 2) {
		joystick1Yaw = axisState->state;
	} else if(axisState->axis == 3) {
		joystick1Throttle = axisState->state / 2 + 0.5;
		float augerSpeed = (reverseAuger) ? -joystick1Throttle : joystick1Throttle;
		std::cout << "auger speed: " << augerSpeed << std::endl;
		auger.data = augerSpeed;
		excavationAugerPublisher->publish(auger);
	}
}

void joystickButtonCallback(const messages::msg::ButtonState::SharedPtr buttonState) {
	std::cout << "Button " << buttonState->joystick << " " << buttonState->button << " " << buttonState->state << std::endl;
	std_msgs::msg::Bool but7;
	std_msgs::msg::Bool but8;
	std_msgs::msg::Bool driveState;
	std_msgs::msg::Float32 elevation;
	std_msgs::msg::Float32 angle;

	switch(buttonState->button) {
		case 0: //ESTOP
				//            DO NOT USE!!!
			break;
		case 1: //toggles driving and digging
			break;
		case 2:
			break;
		case 3:
			if(buttonState->state) {
			} else {
			}
			break;
		case 4:
			if(buttonState->state) {
			} else {
			}
			break;
		case 5:
			if(buttonState->state) {
			} else {
			}
			break;
		case 6:
			reverseAuger = true;
			break;
		case 7:
			reverseAuger = false;
			break;
		case 8:
			excavationAngle += 50;
			angle.data = excavationAngle;
			std::cout << "excavationAngle: " << excavationAngle << std::endl;
			excavationAnglePublisher->publish(angle);
			break;
		case 9:
			excavationAngle -= 50;
			angle.data = excavationAngle;
			std::cout << "excavationAngle: " << excavationAngle << std::endl;
			excavationAnglePublisher->publish(angle);
			break;
		case 10:
			excavationElevation += 50;
			elevation.data = excavationElevation;
			std::cout << "excavationElevation: " << excavationElevation << std::endl;
			excavationElevationPublisher->publish(elevation);
			break;
		case 11:
			excavationElevation -= 50;
			elevation.data = excavationElevation;
			std::cout << "excavationElevation: " << excavationElevation << std::endl;
			excavationElevationPublisher->publish(elevation);
			break;
	}
}

void joystickHatCallback(const messages::msg::HatState::SharedPtr hatState) {
	std::cout << "Hat " << hatState->joystick << " " << hatState->hat << " " << hatState->state << std::endl;
}

void keyCallback(const messages::msg::KeyState::SharedPtr keyState) {
	std::cout << "Key " << keyState->key << " " << keyState->state << std::endl;

	if(keyState->key == 115 && keyState->state == 1) {
		automationGo = !automationGo;
	}
}

void zedPositionCallback(const messages::msg::ZedPosition::SharedPtr zedPosition) {
	Position position;
	position.x = zedPosition->x;
	position.y = zedPosition->y;
	position.z = zedPosition->z;
	position.ox = zedPosition->ox;
	position.oy = zedPosition->oy;
	position.oz = zedPosition->oz;
	position.ow = zedPosition->ow;
	position.arucoVisible = zedPosition->aruco_visible;
	automation->setPosition(position);
}

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	nodeHandle = rclcpp::Node::make_shared("logic");
	automation->setNode(nodeHandle);

	auto joystickAxisSubscriber = nodeHandle->create_subscription<messages::msg::AxisState>("joystick_axis", 1, joystickAxisCallback);
	auto joystickButtonSubscriber = nodeHandle->create_subscription<messages::msg::ButtonState>("joystick_button", 1, joystickButtonCallback);
	auto joystickHatSubscriber = nodeHandle->create_subscription<messages::msg::HatState>("joystick_hat", 1, joystickHatCallback);
	auto keySubscriber = nodeHandle->create_subscription<messages::msg::KeyState>("key", 1, keyCallback);
	auto zedPositionSubscriber = nodeHandle->create_subscription<messages::msg::ZedPosition>("zed_position", 1, zedPositionCallback);

	driveLeftSpeedPublisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("drive_left_speed", 1);
	driveRightSpeedPublisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("drive_right_speed", 1);
	driveStatePublisher = nodeHandle->create_publisher<std_msgs::msg::Bool>("drive_state", 1);
	excavationElevationPublisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("excavationElevation", 1);
	excavationAugerPublisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("excavationAuger", 1);
	excavationAnglePublisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("excavationAngle", 1);

	rclcpp::Rate rate(20);

	while(rclcpp::ok()) {
		if(automationGo) automation->automate();
		rclcpp::spin_some(nodeHandle);
		rate.sleep();
	}
}
