#pragma once
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>

namespace utils {

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
	T getParameter(rclcpp::Node::SharedPtr nodeHandle, std::string parameterName, T initialValue){
		nodeHandle->declare_parameter<T>(parameterName, initialValue);
		rclcpp::Parameter param = nodeHandle->get_parameter(parameterName);
		T value = param.template get_value<T>();
		std::cout << parameterName << ": " << value << std::endl;
		RCLCPP_INFO(nodeHandle->get_logger(), param.value_to_string().c_str());
		return value;
	}

	template <typename T>
	T getParameter(rclcpp::Node::SharedPtr nodeHandle, const std::string& parameterName, const char* initialValue){
		return getParameter<T>(nodeHandle, parameterName, std::string(initialValue));
	}

	void set_speed_frame(struct can_frame* frame, int motorNumber, float speed);
}
