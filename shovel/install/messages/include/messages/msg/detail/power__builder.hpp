// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from messages:msg/Power.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__MSG__DETAIL__POWER__BUILDER_HPP_
#define MESSAGES__MSG__DETAIL__POWER__BUILDER_HPP_

#include "messages/msg/detail/power__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace messages
{

namespace msg
{

namespace builder
{

class Init_Power_current15
{
public:
  explicit Init_Power_current15(::messages::msg::Power & msg)
  : msg_(msg)
  {}
  ::messages::msg::Power current15(::messages::msg::Power::_current15_type arg)
  {
    msg_.current15 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::messages::msg::Power msg_;
};

class Init_Power_current14
{
public:
  explicit Init_Power_current14(::messages::msg::Power & msg)
  : msg_(msg)
  {}
  Init_Power_current15 current14(::messages::msg::Power::_current14_type arg)
  {
    msg_.current14 = std::move(arg);
    return Init_Power_current15(msg_);
  }

private:
  ::messages::msg::Power msg_;
};

class Init_Power_current13
{
public:
  explicit Init_Power_current13(::messages::msg::Power & msg)
  : msg_(msg)
  {}
  Init_Power_current14 current13(::messages::msg::Power::_current13_type arg)
  {
    msg_.current13 = std::move(arg);
    return Init_Power_current14(msg_);
  }

private:
  ::messages::msg::Power msg_;
};

class Init_Power_current12
{
public:
  explicit Init_Power_current12(::messages::msg::Power & msg)
  : msg_(msg)
  {}
  Init_Power_current13 current12(::messages::msg::Power::_current12_type arg)
  {
    msg_.current12 = std::move(arg);
    return Init_Power_current13(msg_);
  }

private:
  ::messages::msg::Power msg_;
};

class Init_Power_current11
{
public:
  explicit Init_Power_current11(::messages::msg::Power & msg)
  : msg_(msg)
  {}
  Init_Power_current12 current11(::messages::msg::Power::_current11_type arg)
  {
    msg_.current11 = std::move(arg);
    return Init_Power_current12(msg_);
  }

private:
  ::messages::msg::Power msg_;
};

class Init_Power_current10
{
public:
  explicit Init_Power_current10(::messages::msg::Power & msg)
  : msg_(msg)
  {}
  Init_Power_current11 current10(::messages::msg::Power::_current10_type arg)
  {
    msg_.current10 = std::move(arg);
    return Init_Power_current11(msg_);
  }

private:
  ::messages::msg::Power msg_;
};

class Init_Power_current9
{
public:
  explicit Init_Power_current9(::messages::msg::Power & msg)
  : msg_(msg)
  {}
  Init_Power_current10 current9(::messages::msg::Power::_current9_type arg)
  {
    msg_.current9 = std::move(arg);
    return Init_Power_current10(msg_);
  }

private:
  ::messages::msg::Power msg_;
};

class Init_Power_current8
{
public:
  explicit Init_Power_current8(::messages::msg::Power & msg)
  : msg_(msg)
  {}
  Init_Power_current9 current8(::messages::msg::Power::_current8_type arg)
  {
    msg_.current8 = std::move(arg);
    return Init_Power_current9(msg_);
  }

private:
  ::messages::msg::Power msg_;
};

class Init_Power_current7
{
public:
  explicit Init_Power_current7(::messages::msg::Power & msg)
  : msg_(msg)
  {}
  Init_Power_current8 current7(::messages::msg::Power::_current7_type arg)
  {
    msg_.current7 = std::move(arg);
    return Init_Power_current8(msg_);
  }

private:
  ::messages::msg::Power msg_;
};

class Init_Power_current6
{
public:
  explicit Init_Power_current6(::messages::msg::Power & msg)
  : msg_(msg)
  {}
  Init_Power_current7 current6(::messages::msg::Power::_current6_type arg)
  {
    msg_.current6 = std::move(arg);
    return Init_Power_current7(msg_);
  }

private:
  ::messages::msg::Power msg_;
};

class Init_Power_current5
{
public:
  explicit Init_Power_current5(::messages::msg::Power & msg)
  : msg_(msg)
  {}
  Init_Power_current6 current5(::messages::msg::Power::_current5_type arg)
  {
    msg_.current5 = std::move(arg);
    return Init_Power_current6(msg_);
  }

private:
  ::messages::msg::Power msg_;
};

class Init_Power_current4
{
public:
  explicit Init_Power_current4(::messages::msg::Power & msg)
  : msg_(msg)
  {}
  Init_Power_current5 current4(::messages::msg::Power::_current4_type arg)
  {
    msg_.current4 = std::move(arg);
    return Init_Power_current5(msg_);
  }

private:
  ::messages::msg::Power msg_;
};

class Init_Power_current3
{
public:
  explicit Init_Power_current3(::messages::msg::Power & msg)
  : msg_(msg)
  {}
  Init_Power_current4 current3(::messages::msg::Power::_current3_type arg)
  {
    msg_.current3 = std::move(arg);
    return Init_Power_current4(msg_);
  }

private:
  ::messages::msg::Power msg_;
};

class Init_Power_current2
{
public:
  explicit Init_Power_current2(::messages::msg::Power & msg)
  : msg_(msg)
  {}
  Init_Power_current3 current2(::messages::msg::Power::_current2_type arg)
  {
    msg_.current2 = std::move(arg);
    return Init_Power_current3(msg_);
  }

private:
  ::messages::msg::Power msg_;
};

class Init_Power_current1
{
public:
  explicit Init_Power_current1(::messages::msg::Power & msg)
  : msg_(msg)
  {}
  Init_Power_current2 current1(::messages::msg::Power::_current1_type arg)
  {
    msg_.current1 = std::move(arg);
    return Init_Power_current2(msg_);
  }

private:
  ::messages::msg::Power msg_;
};

class Init_Power_current0
{
public:
  explicit Init_Power_current0(::messages::msg::Power & msg)
  : msg_(msg)
  {}
  Init_Power_current1 current0(::messages::msg::Power::_current0_type arg)
  {
    msg_.current0 = std::move(arg);
    return Init_Power_current1(msg_);
  }

private:
  ::messages::msg::Power msg_;
};

class Init_Power_temperature
{
public:
  explicit Init_Power_temperature(::messages::msg::Power & msg)
  : msg_(msg)
  {}
  Init_Power_current0 temperature(::messages::msg::Power::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return Init_Power_current0(msg_);
  }

private:
  ::messages::msg::Power msg_;
};

class Init_Power_voltage
{
public:
  Init_Power_voltage()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Power_temperature voltage(::messages::msg::Power::_voltage_type arg)
  {
    msg_.voltage = std::move(arg);
    return Init_Power_temperature(msg_);
  }

private:
  ::messages::msg::Power msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::messages::msg::Power>()
{
  return messages::msg::builder::Init_Power_voltage();
}

}  // namespace messages

#endif  // MESSAGES__MSG__DETAIL__POWER__BUILDER_HPP_
