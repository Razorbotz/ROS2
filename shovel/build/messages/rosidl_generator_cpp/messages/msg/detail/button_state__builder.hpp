// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from messages:msg/ButtonState.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__MSG__DETAIL__BUTTON_STATE__BUILDER_HPP_
#define MESSAGES__MSG__DETAIL__BUTTON_STATE__BUILDER_HPP_

#include "messages/msg/detail/button_state__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace messages
{

namespace msg
{

namespace builder
{

class Init_ButtonState_state
{
public:
  explicit Init_ButtonState_state(::messages::msg::ButtonState & msg)
  : msg_(msg)
  {}
  ::messages::msg::ButtonState state(::messages::msg::ButtonState::_state_type arg)
  {
    msg_.state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::messages::msg::ButtonState msg_;
};

class Init_ButtonState_button
{
public:
  explicit Init_ButtonState_button(::messages::msg::ButtonState & msg)
  : msg_(msg)
  {}
  Init_ButtonState_state button(::messages::msg::ButtonState::_button_type arg)
  {
    msg_.button = std::move(arg);
    return Init_ButtonState_state(msg_);
  }

private:
  ::messages::msg::ButtonState msg_;
};

class Init_ButtonState_joystick
{
public:
  Init_ButtonState_joystick()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ButtonState_button joystick(::messages::msg::ButtonState::_joystick_type arg)
  {
    msg_.joystick = std::move(arg);
    return Init_ButtonState_button(msg_);
  }

private:
  ::messages::msg::ButtonState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::messages::msg::ButtonState>()
{
  return messages::msg::builder::Init_ButtonState_joystick();
}

}  // namespace messages

#endif  // MESSAGES__MSG__DETAIL__BUTTON_STATE__BUILDER_HPP_
