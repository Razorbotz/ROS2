// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from messages:msg/LinearOut.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__MSG__DETAIL__LINEAR_OUT__TRAITS_HPP_
#define MESSAGES__MSG__DETAIL__LINEAR_OUT__TRAITS_HPP_

#include "messages/msg/detail/linear_out__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const messages::msg::LinearOut & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed: ";
    value_to_yaml(msg.speed, out);
    out << "\n";
  }

  // member: potentiometer
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "potentiometer: ";
    value_to_yaml(msg.potentiometer, out);
    out << "\n";
  }

  // member: time_without_change
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time_without_change: ";
    value_to_yaml(msg.time_without_change, out);
    out << "\n";
  }

  // member: max
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max: ";
    value_to_yaml(msg.max, out);
    out << "\n";
  }

  // member: min
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "min: ";
    value_to_yaml(msg.min, out);
    out << "\n";
  }

  // member: error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error: ";
    value_to_yaml(msg.error, out);
    out << "\n";
  }

  // member: run
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "run: ";
    value_to_yaml(msg.run, out);
    out << "\n";
  }

  // member: at_min
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "at_min: ";
    value_to_yaml(msg.at_min, out);
    out << "\n";
  }

  // member: at_max
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "at_max: ";
    value_to_yaml(msg.at_max, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const messages::msg::LinearOut & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<messages::msg::LinearOut>()
{
  return "messages::msg::LinearOut";
}

template<>
inline const char * name<messages::msg::LinearOut>()
{
  return "messages/msg/LinearOut";
}

template<>
struct has_fixed_size<messages::msg::LinearOut>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<messages::msg::LinearOut>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<messages::msg::LinearOut>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MESSAGES__MSG__DETAIL__LINEAR_OUT__TRAITS_HPP_
