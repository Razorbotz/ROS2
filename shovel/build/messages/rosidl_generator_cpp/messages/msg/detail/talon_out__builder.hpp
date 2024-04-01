// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from messages:msg/TalonOut.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__MSG__DETAIL__TALON_OUT__BUILDER_HPP_
#define MESSAGES__MSG__DETAIL__TALON_OUT__BUILDER_HPP_

#include "messages/msg/detail/talon_out__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace messages
{

namespace msg
{

namespace builder
{

class Init_TalonOut_potentiometer
{
public:
  explicit Init_TalonOut_potentiometer(::messages::msg::TalonOut & msg)
  : msg_(msg)
  {}
  ::messages::msg::TalonOut potentiometer(::messages::msg::TalonOut::_potentiometer_type arg)
  {
    msg_.potentiometer = std::move(arg);
    return std::move(msg_);
  }

private:
  ::messages::msg::TalonOut msg_;
};

class Init_TalonOut_error_derivative
{
public:
  explicit Init_TalonOut_error_derivative(::messages::msg::TalonOut & msg)
  : msg_(msg)
  {}
  Init_TalonOut_potentiometer error_derivative(::messages::msg::TalonOut::_error_derivative_type arg)
  {
    msg_.error_derivative = std::move(arg);
    return Init_TalonOut_potentiometer(msg_);
  }

private:
  ::messages::msg::TalonOut msg_;
};

class Init_TalonOut_integral_accumulator
{
public:
  explicit Init_TalonOut_integral_accumulator(::messages::msg::TalonOut & msg)
  : msg_(msg)
  {}
  Init_TalonOut_error_derivative integral_accumulator(::messages::msg::TalonOut::_integral_accumulator_type arg)
  {
    msg_.integral_accumulator = std::move(arg);
    return Init_TalonOut_error_derivative(msg_);
  }

private:
  ::messages::msg::TalonOut msg_;
};

class Init_TalonOut_closed_loop_error
{
public:
  explicit Init_TalonOut_closed_loop_error(::messages::msg::TalonOut & msg)
  : msg_(msg)
  {}
  Init_TalonOut_integral_accumulator closed_loop_error(::messages::msg::TalonOut::_closed_loop_error_type arg)
  {
    msg_.closed_loop_error = std::move(arg);
    return Init_TalonOut_integral_accumulator(msg_);
  }

private:
  ::messages::msg::TalonOut msg_;
};

class Init_TalonOut_sensor_velocity
{
public:
  explicit Init_TalonOut_sensor_velocity(::messages::msg::TalonOut & msg)
  : msg_(msg)
  {}
  Init_TalonOut_closed_loop_error sensor_velocity(::messages::msg::TalonOut::_sensor_velocity_type arg)
  {
    msg_.sensor_velocity = std::move(arg);
    return Init_TalonOut_closed_loop_error(msg_);
  }

private:
  ::messages::msg::TalonOut msg_;
};

class Init_TalonOut_sensor_position
{
public:
  explicit Init_TalonOut_sensor_position(::messages::msg::TalonOut & msg)
  : msg_(msg)
  {}
  Init_TalonOut_sensor_velocity sensor_position(::messages::msg::TalonOut::_sensor_position_type arg)
  {
    msg_.sensor_position = std::move(arg);
    return Init_TalonOut_sensor_velocity(msg_);
  }

private:
  ::messages::msg::TalonOut msg_;
};

class Init_TalonOut_temperature
{
public:
  explicit Init_TalonOut_temperature(::messages::msg::TalonOut & msg)
  : msg_(msg)
  {}
  Init_TalonOut_sensor_position temperature(::messages::msg::TalonOut::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return Init_TalonOut_sensor_position(msg_);
  }

private:
  ::messages::msg::TalonOut msg_;
};

class Init_TalonOut_output_percent
{
public:
  explicit Init_TalonOut_output_percent(::messages::msg::TalonOut & msg)
  : msg_(msg)
  {}
  Init_TalonOut_temperature output_percent(::messages::msg::TalonOut::_output_percent_type arg)
  {
    msg_.output_percent = std::move(arg);
    return Init_TalonOut_temperature(msg_);
  }

private:
  ::messages::msg::TalonOut msg_;
};

class Init_TalonOut_output_voltage
{
public:
  explicit Init_TalonOut_output_voltage(::messages::msg::TalonOut & msg)
  : msg_(msg)
  {}
  Init_TalonOut_output_percent output_voltage(::messages::msg::TalonOut::_output_voltage_type arg)
  {
    msg_.output_voltage = std::move(arg);
    return Init_TalonOut_output_percent(msg_);
  }

private:
  ::messages::msg::TalonOut msg_;
};

class Init_TalonOut_output_current
{
public:
  explicit Init_TalonOut_output_current(::messages::msg::TalonOut & msg)
  : msg_(msg)
  {}
  Init_TalonOut_output_voltage output_current(::messages::msg::TalonOut::_output_current_type arg)
  {
    msg_.output_current = std::move(arg);
    return Init_TalonOut_output_voltage(msg_);
  }

private:
  ::messages::msg::TalonOut msg_;
};

class Init_TalonOut_bus_voltage
{
public:
  explicit Init_TalonOut_bus_voltage(::messages::msg::TalonOut & msg)
  : msg_(msg)
  {}
  Init_TalonOut_output_current bus_voltage(::messages::msg::TalonOut::_bus_voltage_type arg)
  {
    msg_.bus_voltage = std::move(arg);
    return Init_TalonOut_output_current(msg_);
  }

private:
  ::messages::msg::TalonOut msg_;
};

class Init_TalonOut_device_id
{
public:
  Init_TalonOut_device_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TalonOut_bus_voltage device_id(::messages::msg::TalonOut::_device_id_type arg)
  {
    msg_.device_id = std::move(arg);
    return Init_TalonOut_bus_voltage(msg_);
  }

private:
  ::messages::msg::TalonOut msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::messages::msg::TalonOut>()
{
  return messages::msg::builder::Init_TalonOut_device_id();
}

}  // namespace messages

#endif  // MESSAGES__MSG__DETAIL__TALON_OUT__BUILDER_HPP_
