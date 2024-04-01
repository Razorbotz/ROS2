// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from messages:msg/KeyState.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__MSG__DETAIL__KEY_STATE__TRAITS_HPP_
#define MESSAGES__MSG__DETAIL__KEY_STATE__TRAITS_HPP_

#include "messages/msg/detail/key_state__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const messages::msg::KeyState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: key
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "key: ";
    value_to_yaml(msg.key, out);
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

inline std::string to_yaml(const messages::msg::KeyState & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<messages::msg::KeyState>()
{
  return "messages::msg::KeyState";
}

template<>
inline const char * name<messages::msg::KeyState>()
{
  return "messages/msg/KeyState";
}

template<>
struct has_fixed_size<messages::msg::KeyState>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<messages::msg::KeyState>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<messages::msg::KeyState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MESSAGES__MSG__DETAIL__KEY_STATE__TRAITS_HPP_
