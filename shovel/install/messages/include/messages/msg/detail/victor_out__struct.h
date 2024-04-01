// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from messages:msg/VictorOut.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__MSG__DETAIL__VICTOR_OUT__STRUCT_H_
#define MESSAGES__MSG__DETAIL__VICTOR_OUT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/VictorOut in the package messages.
typedef struct messages__msg__VictorOut
{
  int32_t device_id;
  float bus_voltage;
  float output_voltage;
  float output_percent;
} messages__msg__VictorOut;

// Struct for a sequence of messages__msg__VictorOut.
typedef struct messages__msg__VictorOut__Sequence
{
  messages__msg__VictorOut * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} messages__msg__VictorOut__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MESSAGES__MSG__DETAIL__VICTOR_OUT__STRUCT_H_
