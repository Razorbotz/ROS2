// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from messages:msg/AutonomyOut.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__MSG__DETAIL__AUTONOMY_OUT__BUILDER_HPP_
#define MESSAGES__MSG__DETAIL__AUTONOMY_OUT__BUILDER_HPP_

#include "messages/msg/detail/autonomy_out__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace messages
{

namespace msg
{

namespace builder
{

class Init_AutonomyOut_dump_state
{
public:
  explicit Init_AutonomyOut_dump_state(::messages::msg::AutonomyOut & msg)
  : msg_(msg)
  {}
  ::messages::msg::AutonomyOut dump_state(::messages::msg::AutonomyOut::_dump_state_type arg)
  {
    msg_.dump_state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::messages::msg::AutonomyOut msg_;
};

class Init_AutonomyOut_error_state
{
public:
  explicit Init_AutonomyOut_error_state(::messages::msg::AutonomyOut & msg)
  : msg_(msg)
  {}
  Init_AutonomyOut_dump_state error_state(::messages::msg::AutonomyOut::_error_state_type arg)
  {
    msg_.error_state = std::move(arg);
    return Init_AutonomyOut_dump_state(msg_);
  }

private:
  ::messages::msg::AutonomyOut msg_;
};

class Init_AutonomyOut_excavation_state
{
public:
  explicit Init_AutonomyOut_excavation_state(::messages::msg::AutonomyOut & msg)
  : msg_(msg)
  {}
  Init_AutonomyOut_error_state excavation_state(::messages::msg::AutonomyOut::_excavation_state_type arg)
  {
    msg_.excavation_state = std::move(arg);
    return Init_AutonomyOut_error_state(msg_);
  }

private:
  ::messages::msg::AutonomyOut msg_;
};

class Init_AutonomyOut_robot_state
{
public:
  Init_AutonomyOut_robot_state()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AutonomyOut_excavation_state robot_state(::messages::msg::AutonomyOut::_robot_state_type arg)
  {
    msg_.robot_state = std::move(arg);
    return Init_AutonomyOut_excavation_state(msg_);
  }

private:
  ::messages::msg::AutonomyOut msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::messages::msg::AutonomyOut>()
{
  return messages::msg::builder::Init_AutonomyOut_robot_state();
}

}  // namespace messages

#endif  // MESSAGES__MSG__DETAIL__AUTONOMY_OUT__BUILDER_HPP_
