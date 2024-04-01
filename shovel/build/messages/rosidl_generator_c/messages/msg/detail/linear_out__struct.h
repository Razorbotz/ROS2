// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from messages:msg/LinearOut.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__MSG__DETAIL__LINEAR_OUT__STRUCT_H_
#define MESSAGES__MSG__DETAIL__LINEAR_OUT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'error'
#include "rosidl_runtime_c/string.h"

// Struct defined in msg/LinearOut in the package messages.
typedef struct messages__msg__LinearOut
{
  float speed;
  int32_t potentiometer;
  int32_t time_without_change;
  int32_t max;
  int32_t min;
  rosidl_runtime_c__String error;
  bool run;
  bool at_min;
  bool at_max;
} messages__msg__LinearOut;

// Struct for a sequence of messages__msg__LinearOut.
typedef struct messages__msg__LinearOut__Sequence
{
  messages__msg__LinearOut * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} messages__msg__LinearOut__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MESSAGES__MSG__DETAIL__LINEAR_OUT__STRUCT_H_
