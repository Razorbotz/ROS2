#ifndef COMMONFUNCS_H   // To make sure you don't declare the function more than once by including the header multiple times.
#define COMMONFUNCS_H
    #include <string>
    #include <rclcpp/rclcpp.hpp>

    template <typename T>
    T getParameter(std::string parameterName, int initialValue);
    template <typename T>
    T getParameter(std::string parameterName, std::string initialValue);
    void checkTemperature(double temperature, int op_mode, bool *TEMP_DISABLE);
    void checkVoltage(double voltage, double speed, int op_mode, bool *VOLT_DISABLE);
    
#endif