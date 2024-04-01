// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from messages:msg/AxisState.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__MSG__DETAIL__AXIS_STATE__TRAITS_HPP_
#define MESSAGES__MSG__DETAIL__AXIS_STATE__TRAITS_HPP_

#include "messages/msg/detail/axis_state__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const messages::msg::AxisState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: joystick
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "joystick: ";
    value_to_yaml(msg.joystick, out);
    out << "\n";
  }

  // member: axis
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "axis: ";
    value_to_yaml(msg.axis, out);
    out << "\n";
  }

  // member: state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state: ";
    value_to_yaml(msg.state, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const messages::msg::AxisState & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<messages::msg::AxisState>()
{
  return "messages::msg::AxisState";
}

template<>
inline const char * name<messages::msg::AxisState>()
{
  return "messages/msg/AxisState";
}

template<>
struct has_fixed_size<messages::msg::AxisState>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<messages::msg::AxisState>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<messages::msg::AxisState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MESSAGES__MSG__DETAIL__AXIS_STATE__TRAITS_HPP_
