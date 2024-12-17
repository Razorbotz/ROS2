#include <stdio.h>
#include <string>
#include <stdio.h>
#include <rclcpp/rclcpp.hpp>
#include "common/CommonFuncs.h"


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
T getParameter(std::string parameterName, std::string initialValue, rclcpp::Node::SharedPtr *nodeHandle){
	*nodeHandle->declare_parameter<T>(parameterName, initialValue);
	rclcpp::Parameter param = *nodeHandle->get_parameter(parameterName);
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
T getParameter(std::string parameterName, int initialValue, rclcpp::Node::SharedPtr *nodeHandle){
	*nodeHandle->declare_parameter<T>(parameterName, initialValue);
	rclcpp::Parameter param = *nodeHandle->get_parameter(parameterName);
	T value;
	if(typeid(value).name() == typeid(int).name())
		value = param.as_int();
	if(typeid(value).name() == typeid(double).name())
		value = param.as_double();
	if(typeid(value).name() == typeid(bool).name())
		value = param.as_bool();
	std::cout << parameterName << ": " << value << std::endl;
	std::string output = parameterName + ": " + std::to_string(value);
	RCLCPP_INFO(*nodeHandle->get_logger(), output.c_str());
	return value;
}


void checkTemperature(double temperature, int op_mode, bool *TEMP_DISABLE){
	switch(op_mode){
		case 0:
			temperature > 70 ? *TEMP_DISABLE = true : *TEMP_DISABLE = false;
			break;
		case 1:
			temperature > 80 ? *TEMP_DISABLE = true : *TEMP_DISABLE = false;
			break;
		case 2:
			temperature > 90 ? *TEMP_DISABLE = true : *TEMP_DISABLE = false;
			break;
	}
}


void checkVoltage(double voltage, double speed, int op_mode, bool *VOLT_DISABLE){
	if(speed > 0){
		switch(op_mode){
			case 0:
				voltage < 15 ? *VOLT_DISABLE = true : *VOLT_DISABLE = false;
				break;
			case 1:
				voltage < 14.4 ? *VOLT_DISABLE = true : *VOLT_DISABLE = false;
				break;
			case 2:
				voltage < 13 ? *VOLT_DISABLE = true : *VOLT_DISABLE = false;
				break;
		}
	}
	else{
		switch(op_mode){
			case 0:
				voltage < 15.4 ? *VOLT_DISABLE = true : *VOLT_DISABLE = false;
				break;
			case 1:
				voltage < 15 ? *VOLT_DISABLE = true : *VOLT_DISABLE = false;
				break;
			case 2:
				voltage < 14 ? *VOLT_DISABLE = true : *VOLT_DISABLE = false;
				break;
		}
	}
}