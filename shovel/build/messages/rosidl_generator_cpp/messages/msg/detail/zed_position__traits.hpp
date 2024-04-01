// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from messages:msg/ZedPosition.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__MSG__DETAIL__ZED_POSITION__TRAITS_HPP_
#define MESSAGES__MSG__DETAIL__ZED_POSITION__TRAITS_HPP_

#include "messages/msg/detail/zed_position__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const messages::msg::ZedPosition & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z: ";
    value_to_yaml(msg.z, out);
    out << "\n";
  }

  // member: ox
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ox: ";
    value_to_yaml(msg.ox, out);
    out << "\n";
  }

  // member: oy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "oy: ";
    value_to_yaml(msg.oy, out);
    out << "\n";
  }

  // member: oz
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "oz: ";
    value_to_yaml(msg.oz, out);
    out << "\n";
  }

  // member: ow
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ow: ";
    value_to_yaml(msg.ow, out);
    out << "\n";
  }

  // member: aruco_visible
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "aruco_visible: ";
    value_to_yaml(msg.aruco_visible, out);
    out << "\n";
  }

  // member: pitch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch: ";
    value_to_yaml(msg.pitch, out);
    out << "\n";
  }

  // member: yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw: ";
    value_to_yaml(msg.yaw, out);
    out << "\n";
  }

  // member: roll
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "roll: ";
    value_to_yaml(msg.roll, out);
    out << "\n";
  }

  // member: aruco_pitch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "aruco_pitch: ";
    value_to_yaml(msg.aruco_pitch, out);
    out << "\n";
  }

  // member: aruco_yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "aruco_yaw: ";
    value_to_yaml(msg.aruco_yaw, out);
    out << "\n";
  }

  // member: aruco_roll
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "aruco_roll: ";
    value_to_yaml(msg.aruco_roll, out);
    out << "\n";
  }

  // member: distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance: ";
    value_to_yaml(msg.distance, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const messages::msg::ZedPosition & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<messages::msg::ZedPosition>()
{
  return "messages::msg::ZedPosition";
}

template<>
inline const char * name<messages::msg::ZedPosition>()
{
  return "messages/msg/ZedPosition";
}

template<>
struct has_fixed_size<messages::msg::ZedPosition>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<messages::msg::ZedPosition>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<messages::msg::ZedPosition>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MESSAGES__MSG__DETAIL__ZED_POSITION__TRAITS_HPP_
