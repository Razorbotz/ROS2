// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from messages:msg/TalonOut.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__MSG__DETAIL__TALON_OUT__STRUCT_H_
#define MESSAGES__MSG__DETAIL__TALON_OUT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/TalonOut in the package messages.
typedef struct messages__msg__TalonOut
{
  int32_t device_id;
  float bus_voltage;
  float output_current;
  float output_voltage;
  float output_percent;
  float temperature;
  int32_t sensor_position;
  int32_t sensor_velocity;
  int32_t closed_loop_error;
  int32_t integral_accumulator;
  int32_t error_derivative;
  int32_t potentiometer;
} messages__msg__TalonOut;

// Struct for a sequence of messages__msg__TalonOut.
typedef struct messages__msg__TalonOut__Sequence
{
  messages__msg__TalonOut * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} messages__msg__TalonOut__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MESSAGES__MSG__DETAIL__TALON_OUT__STRUCT_H_
