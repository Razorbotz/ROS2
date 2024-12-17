#ifndef COMMONFUNCS_H   // To make sure you don't declare the function more than once by including the header multiple times.
#define COMMONFUNCS_H
    #include <string>
    #include <rclcpp/rclcpp.hpp>
    #include <stdio.h>

    template <typename T>
    T getParameter(std::string parameterName, int initialValue, rclcpp::Node::SharedPtr *nodeHandle);
    template <typename T>
    T getParameter(std::string parameterName, std::string initialValue, rclcpp::Node::SharedPtr *nodeHandle);
    void checkTemperature(double temperature, int op_mode, bool *TEMP_DISABLE);
    void checkVoltage(double voltage, double speed, int op_mode, bool *VOLT_DISABLE);
    
#endif