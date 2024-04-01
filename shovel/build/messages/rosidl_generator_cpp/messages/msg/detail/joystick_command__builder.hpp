// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from messages:msg/JoystickCommand.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__MSG__DETAIL__JOYSTICK_COMMAND__BUILDER_HPP_
#define MESSAGES__MSG__DETAIL__JOYSTICK_COMMAND__BUILDER_HPP_

#include "messages/msg/detail/joystick_command__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace messages
{

namespace msg
{

namespace builder
{

class Init_JoystickCommand_value
{
public:
  explicit Init_JoystickCommand_value(::messages::msg::JoystickCommand & msg)
  : msg_(msg)
  {}
  ::messages::msg::JoystickCommand value(::messages::msg::JoystickCommand::_value_type arg)
  {
    msg_.value = std::move(arg);
    return std::move(msg_);
  }

private:
  ::messages::msg::JoystickCommand msg_;
};

class Init_JoystickCommand_element
{
public:
  explicit Init_JoystickCommand_element(::messages::msg::JoystickCommand & msg)
  : msg_(msg)
  {}
  Init_JoystickCommand_value element(::messages::msg::JoystickCommand::_element_type arg)
  {
    msg_.element = std::move(arg);
    return Init_JoystickCommand_value(msg_);
  }

private:
  ::messages::msg::JoystickCommand msg_;
};

class Init_JoystickCommand_number
{
public:
  Init_JoystickCommand_number()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_JoystickCommand_element number(::messages::msg::JoystickCommand::_number_type arg)
  {
    msg_.number = std::move(arg);
    return Init_JoystickCommand_element(msg_);
  }

private:
  ::messages::msg::JoystickCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::messages::msg::JoystickCommand>()
{
  return messages::msg::builder::Init_JoystickCommand_number();
}

}  // namespace messages

#endif  // MESSAGES__MSG__DETAIL__JOYSTICK_COMMAND__BUILDER_HPP_
