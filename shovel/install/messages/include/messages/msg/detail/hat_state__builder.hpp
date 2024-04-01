// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from messages:msg/HatState.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__MSG__DETAIL__HAT_STATE__BUILDER_HPP_
#define MESSAGES__MSG__DETAIL__HAT_STATE__BUILDER_HPP_

#include "messages/msg/detail/hat_state__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace messages
{

namespace msg
{

namespace builder
{

class Init_HatState_state
{
public:
  explicit Init_HatState_state(::messages::msg::HatState & msg)
  : msg_(msg)
  {}
  ::messages::msg::HatState state(::messages::msg::HatState::_state_type arg)
  {
    msg_.state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::messages::msg::HatState msg_;
};

class Init_HatState_hat
{
public:
  explicit Init_HatState_hat(::messages::msg::HatState & msg)
  : msg_(msg)
  {}
  Init_HatState_state hat(::messages::msg::HatState::_hat_type arg)
  {
    msg_.hat = std::move(arg);
    return Init_HatState_state(msg_);
  }

private:
  ::messages::msg::HatState msg_;
};

class Init_HatState_joystick
{
public:
  Init_HatState_joystick()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HatState_hat joystick(::messages::msg::HatState::_joystick_type arg)
  {
    msg_.joystick = std::move(arg);
    return Init_HatState_hat(msg_);
  }

private:
  ::messages::msg::HatState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::messages::msg::HatState>()
{
  return messages::msg::builder::Init_HatState_joystick();
}

}  // namespace messages

#endif  // MESSAGES__MSG__DETAIL__HAT_STATE__BUILDER_HPP_
