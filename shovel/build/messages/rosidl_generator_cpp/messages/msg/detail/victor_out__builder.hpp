// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from messages:msg/VictorOut.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__MSG__DETAIL__VICTOR_OUT__BUILDER_HPP_
#define MESSAGES__MSG__DETAIL__VICTOR_OUT__BUILDER_HPP_

#include "messages/msg/detail/victor_out__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace messages
{

namespace msg
{

namespace builder
{

class Init_VictorOut_output_percent
{
public:
  explicit Init_VictorOut_output_percent(::messages::msg::VictorOut & msg)
  : msg_(msg)
  {}
  ::messages::msg::VictorOut output_percent(::messages::msg::VictorOut::_output_percent_type arg)
  {
    msg_.output_percent = std::move(arg);
    return std::move(msg_);
  }

private:
  ::messages::msg::VictorOut msg_;
};

class Init_VictorOut_output_voltage
{
public:
  explicit Init_VictorOut_output_voltage(::messages::msg::VictorOut & msg)
  : msg_(msg)
  {}
  Init_VictorOut_output_percent output_voltage(::messages::msg::VictorOut::_output_voltage_type arg)
  {
    msg_.output_voltage = std::move(arg);
    return Init_VictorOut_output_percent(msg_);
  }

private:
  ::messages::msg::VictorOut msg_;
};

class Init_VictorOut_bus_voltage
{
public:
  explicit Init_VictorOut_bus_voltage(::messages::msg::VictorOut & msg)
  : msg_(msg)
  {}
  Init_VictorOut_output_voltage bus_voltage(::messages::msg::VictorOut::_bus_voltage_type arg)
  {
    msg_.bus_voltage = std::move(arg);
    return Init_VictorOut_output_voltage(msg_);
  }

private:
  ::messages::msg::VictorOut msg_;
};

class Init_VictorOut_device_id
{
public:
  Init_VictorOut_device_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VictorOut_bus_voltage device_id(::messages::msg::VictorOut::_device_id_type arg)
  {
    msg_.device_id = std::move(arg);
    return Init_VictorOut_bus_voltage(msg_);
  }

private:
  ::messages::msg::VictorOut msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::messages::msg::VictorOut>()
{
  return messages::msg::builder::Init_VictorOut_device_id();
}

}  // namespace messages

#endif  // MESSAGES__MSG__DETAIL__VICTOR_OUT__BUILDER_HPP_
