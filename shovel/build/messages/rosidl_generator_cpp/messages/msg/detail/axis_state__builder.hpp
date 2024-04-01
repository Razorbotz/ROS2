// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from messages:msg/AxisState.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__MSG__DETAIL__AXIS_STATE__BUILDER_HPP_
#define MESSAGES__MSG__DETAIL__AXIS_STATE__BUILDER_HPP_

#include "messages/msg/detail/axis_state__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace messages
{

namespace msg
{

namespace builder
{

class Init_AxisState_state
{
public:
  explicit Init_AxisState_state(::messages::msg::AxisState & msg)
  : msg_(msg)
  {}
  ::messages::msg::AxisState state(::messages::msg::AxisState::_state_type arg)
  {
    msg_.state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::messages::msg::AxisState msg_;
};

class Init_AxisState_axis
{
public:
  explicit Init_AxisState_axis(::messages::msg::AxisState & msg)
  : msg_(msg)
  {}
  Init_AxisState_state axis(::messages::msg::AxisState::_axis_type arg)
  {
    msg_.axis = std::move(arg);
    return Init_AxisState_state(msg_);
  }

private:
  ::messages::msg::AxisState msg_;
};

class Init_AxisState_joystick
{
public:
  Init_AxisState_joystick()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AxisState_axis joystick(::messages::msg::AxisState::_joystick_type arg)
  {
    msg_.joystick = std::move(arg);
    return Init_AxisState_axis(msg_);
  }

private:
  ::messages::msg::AxisState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::messages::msg::AxisState>()
{
  return messages::msg::builder::Init_AxisState_joystick();
}

}  // namespace messages

#endif  // MESSAGES__MSG__DETAIL__AXIS_STATE__BUILDER_HPP_
