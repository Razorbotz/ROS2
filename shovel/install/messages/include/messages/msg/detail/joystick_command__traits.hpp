// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from messages:msg/JoystickCommand.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__MSG__DETAIL__JOYSTICK_COMMAND__TRAITS_HPP_
#define MESSAGES__MSG__DETAIL__JOYSTICK_COMMAND__TRAITS_HPP_

#include "messages/msg/detail/joystick_command__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const messages::msg::JoystickCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: number
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "number: ";
    value_to_yaml(msg.number, out);
    out << "\n";
  }

  // member: element
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "element: ";
    value_to_yaml(msg.element, out);
    out << "\n";
  }

  // member: value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "value: ";
    value_to_yaml(msg.value, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const messages::msg::JoystickCommand & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<messages::msg::JoystickCommand>()
{
  return "messages::msg::JoystickCommand";
}

template<>
inline const char * name<messages::msg::JoystickCommand>()
{
  return "messages/msg/JoystickCommand";
}

template<>
struct has_fixed_size<messages::msg::JoystickCommand>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<messages::msg::JoystickCommand>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<messages::msg::JoystickCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MESSAGES__MSG__DETAIL__JOYSTICK_COMMAND__TRAITS_HPP_
