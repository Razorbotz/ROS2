// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from messages:msg/Camera.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__MSG__DETAIL__CAMERA__TRAITS_HPP_
#define MESSAGES__MSG__DETAIL__CAMERA__TRAITS_HPP_

#include "messages/msg/detail/camera__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const messages::msg::Camera & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: port
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "port: ";
    value_to_yaml(msg.port, out);
    out << "\n";
  }

  // member: address
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "address: ";
    value_to_yaml(msg.address, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const messages::msg::Camera & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<messages::msg::Camera>()
{
  return "messages::msg::Camera";
}

template<>
inline const char * name<messages::msg::Camera>()
{
  return "messages/msg/Camera";
}

template<>
struct has_fixed_size<messages::msg::Camera>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<messages::msg::Camera>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<messages::msg::Camera>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MESSAGES__MSG__DETAIL__CAMERA__TRAITS_HPP_
