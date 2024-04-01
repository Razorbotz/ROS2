// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from messages:msg/AutonomyOut.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__MSG__DETAIL__AUTONOMY_OUT__TRAITS_HPP_
#define MESSAGES__MSG__DETAIL__AUTONOMY_OUT__TRAITS_HPP_

#include "messages/msg/detail/autonomy_out__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const messages::msg::AutonomyOut & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: robot_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_state: ";
    value_to_yaml(msg.robot_state, out);
    out << "\n";
  }

  // member: excavation_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "excavation_state: ";
    value_to_yaml(msg.excavation_state, out);
    out << "\n";
  }

  // member: error_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error_state: ";
    value_to_yaml(msg.error_state, out);
    out << "\n";
  }

  // member: dump_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dump_state: ";
    value_to_yaml(msg.dump_state, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const messages::msg::AutonomyOut & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<messages::msg::AutonomyOut>()
{
  return "messages::msg::AutonomyOut";
}

template<>
inline const char * name<messages::msg::AutonomyOut>()
{
  return "messages/msg/AutonomyOut";
}

template<>
struct has_fixed_size<messages::msg::AutonomyOut>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<messages::msg::AutonomyOut>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<messages::msg::AutonomyOut>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MESSAGES__MSG__DETAIL__AUTONOMY_OUT__TRAITS_HPP_
