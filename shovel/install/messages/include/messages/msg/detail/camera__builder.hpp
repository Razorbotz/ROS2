// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from messages:msg/Camera.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__MSG__DETAIL__CAMERA__BUILDER_HPP_
#define MESSAGES__MSG__DETAIL__CAMERA__BUILDER_HPP_

#include "messages/msg/detail/camera__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace messages
{

namespace msg
{

namespace builder
{

class Init_Camera_address
{
public:
  explicit Init_Camera_address(::messages::msg::Camera & msg)
  : msg_(msg)
  {}
  ::messages::msg::Camera address(::messages::msg::Camera::_address_type arg)
  {
    msg_.address = std::move(arg);
    return std::move(msg_);
  }

private:
  ::messages::msg::Camera msg_;
};

class Init_Camera_port
{
public:
  Init_Camera_port()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Camera_address port(::messages::msg::Camera::_port_type arg)
  {
    msg_.port = std::move(arg);
    return Init_Camera_address(msg_);
  }

private:
  ::messages::msg::Camera msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::messages::msg::Camera>()
{
  return messages::msg::builder::Init_Camera_port();
}

}  // namespace messages

#endif  // MESSAGES__MSG__DETAIL__CAMERA__BUILDER_HPP_
