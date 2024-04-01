// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from messages:msg/TalonOut.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__MSG__DETAIL__TALON_OUT__TRAITS_HPP_
#define MESSAGES__MSG__DETAIL__TALON_OUT__TRAITS_HPP_

#include "messages/msg/detail/talon_out__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const messages::msg::TalonOut & msg,
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

  // member: output_current
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "output_current: ";
    value_to_yaml(msg.output_current, out);
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

  // member: temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "temperature: ";
    value_to_yaml(msg.temperature, out);
    out << "\n";
  }

  // member: sensor_position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sensor_position: ";
    value_to_yaml(msg.sensor_position, out);
    out << "\n";
  }

  // member: sensor_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sensor_velocity: ";
    value_to_yaml(msg.sensor_velocity, out);
    out << "\n";
  }

  // member: closed_loop_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "closed_loop_error: ";
    value_to_yaml(msg.closed_loop_error, out);
    out << "\n";
  }

  // member: integral_accumulator
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "integral_accumulator: ";
    value_to_yaml(msg.integral_accumulator, out);
    out << "\n";
  }

  // member: error_derivative
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error_derivative: ";
    value_to_yaml(msg.error_derivative, out);
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const messages::msg::TalonOut & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<messages::msg::TalonOut>()
{
  return "messages::msg::TalonOut";
}

template<>
inline const char * name<messages::msg::TalonOut>()
{
  return "messages/msg/TalonOut";
}

template<>
struct has_fixed_size<messages::msg::TalonOut>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<messages::msg::TalonOut>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<messages::msg::TalonOut>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MESSAGES__MSG__DETAIL__TALON_OUT__TRAITS_HPP_
