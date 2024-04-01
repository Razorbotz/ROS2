// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from messages:msg/LinearOut.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__MSG__DETAIL__LINEAR_OUT__BUILDER_HPP_
#define MESSAGES__MSG__DETAIL__LINEAR_OUT__BUILDER_HPP_

#include "messages/msg/detail/linear_out__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace messages
{

namespace msg
{

namespace builder
{

class Init_LinearOut_at_max
{
public:
  explicit Init_LinearOut_at_max(::messages::msg::LinearOut & msg)
  : msg_(msg)
  {}
  ::messages::msg::LinearOut at_max(::messages::msg::LinearOut::_at_max_type arg)
  {
    msg_.at_max = std::move(arg);
    return std::move(msg_);
  }

private:
  ::messages::msg::LinearOut msg_;
};

class Init_LinearOut_at_min
{
public:
  explicit Init_LinearOut_at_min(::messages::msg::LinearOut & msg)
  : msg_(msg)
  {}
  Init_LinearOut_at_max at_min(::messages::msg::LinearOut::_at_min_type arg)
  {
    msg_.at_min = std::move(arg);
    return Init_LinearOut_at_max(msg_);
  }

private:
  ::messages::msg::LinearOut msg_;
};

class Init_LinearOut_run
{
public:
  explicit Init_LinearOut_run(::messages::msg::LinearOut & msg)
  : msg_(msg)
  {}
  Init_LinearOut_at_min run(::messages::msg::LinearOut::_run_type arg)
  {
    msg_.run = std::move(arg);
    return Init_LinearOut_at_min(msg_);
  }

private:
  ::messages::msg::LinearOut msg_;
};

class Init_LinearOut_error
{
public:
  explicit Init_LinearOut_error(::messages::msg::LinearOut & msg)
  : msg_(msg)
  {}
  Init_LinearOut_run error(::messages::msg::LinearOut::_error_type arg)
  {
    msg_.error = std::move(arg);
    return Init_LinearOut_run(msg_);
  }

private:
  ::messages::msg::LinearOut msg_;
};

class Init_LinearOut_min
{
public:
  explicit Init_LinearOut_min(::messages::msg::LinearOut & msg)
  : msg_(msg)
  {}
  Init_LinearOut_error min(::messages::msg::LinearOut::_min_type arg)
  {
    msg_.min = std::move(arg);
    return Init_LinearOut_error(msg_);
  }

private:
  ::messages::msg::LinearOut msg_;
};

class Init_LinearOut_max
{
public:
  explicit Init_LinearOut_max(::messages::msg::LinearOut & msg)
  : msg_(msg)
  {}
  Init_LinearOut_min max(::messages::msg::LinearOut::_max_type arg)
  {
    msg_.max = std::move(arg);
    return Init_LinearOut_min(msg_);
  }

private:
  ::messages::msg::LinearOut msg_;
};

class Init_LinearOut_time_without_change
{
public:
  explicit Init_LinearOut_time_without_change(::messages::msg::LinearOut & msg)
  : msg_(msg)
  {}
  Init_LinearOut_max time_without_change(::messages::msg::LinearOut::_time_without_change_type arg)
  {
    msg_.time_without_change = std::move(arg);
    return Init_LinearOut_max(msg_);
  }

private:
  ::messages::msg::LinearOut msg_;
};

class Init_LinearOut_potentiometer
{
public:
  explicit Init_LinearOut_potentiometer(::messages::msg::LinearOut & msg)
  : msg_(msg)
  {}
  Init_LinearOut_time_without_change potentiometer(::messages::msg::LinearOut::_potentiometer_type arg)
  {
    msg_.potentiometer = std::move(arg);
    return Init_LinearOut_time_without_change(msg_);
  }

private:
  ::messages::msg::LinearOut msg_;
};

class Init_LinearOut_speed
{
public:
  Init_LinearOut_speed()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LinearOut_potentiometer speed(::messages::msg::LinearOut::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_LinearOut_potentiometer(msg_);
  }

private:
  ::messages::msg::LinearOut msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::messages::msg::LinearOut>()
{
  return messages::msg::builder::Init_LinearOut_speed();
}

}  // namespace messages

#endif  // MESSAGES__MSG__DETAIL__LINEAR_OUT__BUILDER_HPP_
