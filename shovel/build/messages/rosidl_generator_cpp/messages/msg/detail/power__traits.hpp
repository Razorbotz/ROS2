// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from messages:msg/Power.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__MSG__DETAIL__POWER__TRAITS_HPP_
#define MESSAGES__MSG__DETAIL__POWER__TRAITS_HPP_

#include "messages/msg/detail/power__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const messages::msg::Power & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: voltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "voltage: ";
    value_to_yaml(msg.voltage, out);
    out << "\n";
  }

  // member: temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "temperature: ";
    value_to_yaml(msg.temperature, out);
    out << "\n";
  }

  // member: current0
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current0: ";
    value_to_yaml(msg.current0, out);
    out << "\n";
  }

  // member: current1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current1: ";
    value_to_yaml(msg.current1, out);
    out << "\n";
  }

  // member: current2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current2: ";
    value_to_yaml(msg.current2, out);
    out << "\n";
  }

  // member: current3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current3: ";
    value_to_yaml(msg.current3, out);
    out << "\n";
  }

  // member: current4
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current4: ";
    value_to_yaml(msg.current4, out);
    out << "\n";
  }

  // member: current5
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current5: ";
    value_to_yaml(msg.current5, out);
    out << "\n";
  }

  // member: current6
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current6: ";
    value_to_yaml(msg.current6, out);
    out << "\n";
  }

  // member: current7
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current7: ";
    value_to_yaml(msg.current7, out);
    out << "\n";
  }

  // member: current8
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current8: ";
    value_to_yaml(msg.current8, out);
    out << "\n";
  }

  // member: current9
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current9: ";
    value_to_yaml(msg.current9, out);
    out << "\n";
  }

  // member: current10
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current10: ";
    value_to_yaml(msg.current10, out);
    out << "\n";
  }

  // member: current11
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current11: ";
    value_to_yaml(msg.current11, out);
    out << "\n";
  }

  // member: current12
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current12: ";
    value_to_yaml(msg.current12, out);
    out << "\n";
  }

  // member: current13
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current13: ";
    value_to_yaml(msg.current13, out);
    out << "\n";
  }

  // member: current14
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current14: ";
    value_to_yaml(msg.current14, out);
    out << "\n";
  }

  // member: current15
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current15: ";
    value_to_yaml(msg.current15, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const messages::msg::Power & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<messages::msg::Power>()
{
  return "messages::msg::Power";
}

template<>
inline const char * name<messages::msg::Power>()
{
  return "messages/msg/Power";
}

template<>
struct has_fixed_size<messages::msg::Power>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<messages::msg::Power>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<messages::msg::Power>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MESSAGES__MSG__DETAIL__POWER__TRAITS_HPP_
