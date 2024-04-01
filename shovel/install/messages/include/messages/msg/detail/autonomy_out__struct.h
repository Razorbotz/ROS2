// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from messages:msg/AutonomyOut.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__MSG__DETAIL__AUTONOMY_OUT__STRUCT_H_
#define MESSAGES__MSG__DETAIL__AUTONOMY_OUT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'robot_state'
// Member 'excavation_state'
// Member 'error_state'
// Member 'dump_state'
#include "rosidl_runtime_c/string.h"

// Struct defined in msg/AutonomyOut in the package messages.
typedef struct messages__msg__AutonomyOut
{
  rosidl_runtime_c__String robot_state;
  rosidl_runtime_c__String excavation_state;
  rosidl_runtime_c__String error_state;
  rosidl_runtime_c__String dump_state;
} messages__msg__AutonomyOut;

// Struct for a sequence of messages__msg__AutonomyOut.
typedef struct messages__msg__AutonomyOut__Sequence
{
  messages__msg__AutonomyOut * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} messages__msg__AutonomyOut__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MESSAGES__MSG__DETAIL__AUTONOMY_OUT__STRUCT_H_
