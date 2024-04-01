// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from messages:msg/AxisState.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__MSG__DETAIL__AXIS_STATE__STRUCT_H_
#define MESSAGES__MSG__DETAIL__AXIS_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/AxisState in the package messages.
typedef struct messages__msg__AxisState
{
  uint8_t joystick;
  float axis;
  float state;
} messages__msg__AxisState;

// Struct for a sequence of messages__msg__AxisState.
typedef struct messages__msg__AxisState__Sequence
{
  messages__msg__AxisState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} messages__msg__AxisState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MESSAGES__MSG__DETAIL__AXIS_STATE__STRUCT_H_
