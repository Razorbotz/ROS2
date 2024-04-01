// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from messages:msg/KeyState.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__MSG__DETAIL__KEY_STATE__BUILDER_HPP_
#define MESSAGES__MSG__DETAIL__KEY_STATE__BUILDER_HPP_

#include "messages/msg/detail/key_state__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace messages
{

namespace msg
{

namespace builder
{

class Init_KeyState_state
{
public:
  explicit Init_KeyState_state(::messages::msg::KeyState & msg)
  : msg_(msg)
  {}
  ::messages::msg::KeyState state(::messages::msg::KeyState::_state_type arg)
  {
    msg_.state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::messages::msg::KeyState msg_;
};

class Init_KeyState_key
{
public:
  Init_KeyState_key()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_KeyState_state key(::messages::msg::KeyState::_key_type arg)
  {
    msg_.key = std::move(arg);
    return Init_KeyState_state(msg_);
  }

private:
  ::messages::msg::KeyState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::messages::msg::KeyState>()
{
  return messages::msg::builder::Init_KeyState_key();
}

}  // namespace messages

#endif  // MESSAGES__MSG__DETAIL__KEY_STATE__BUILDER_HPP_
