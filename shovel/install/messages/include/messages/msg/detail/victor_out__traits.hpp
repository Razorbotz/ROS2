// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from messages:msg/VictorOut.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__MSG__DETAIL__VICTOR_OUT__TRAITS_HPP_
#define MESSAGES__MSG__DETAIL__VICTOR_OUT__TRAITS_HPP_

#include "messages/msg/detail/victor_out__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const messages::msg::VictorOut & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: device_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "device_id: ";
    value_to_yaml(msg.device_id, out);
    out << "\n";
  }

  // member: bus_voltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bus_voltage: ";
    value_to_yaml(msg.bus_voltage, out);
    out << "\n";
  }

  // member: output_voltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "output_voltage: ";
    value_to_yaml(msg.output_voltage, out);
    out << "\n";
  }

  // member: output_percent
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "output_percent: ";
    value_to_yaml(msg.output_percent, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const messages::msg::VictorOut & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<messages::msg::VictorOut>()
{
  return "messages::msg::VictorOut";
}

template<>
inline const char * name<messages::msg::VictorOut>()
{
  return "messages/msg/VictorOut";
}

template<>
struct has_fixed_size<messages::msg::VictorOut>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<messages::msg::VictorOut>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<messages::msg::VictorOut>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MESSAGES__MSG__DETAIL__VICTOR_OUT__TRAITS_HPP_
