// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from messages:msg/Power.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__MSG__DETAIL__POWER__STRUCT_H_
#define MESSAGES__MSG__DETAIL__POWER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/Power in the package messages.
typedef struct messages__msg__Power
{
  float voltage;
  float temperature;
  float current0;
  float current1;
  float current2;
  float current3;
  float current4;
  float current5;
  float current6;
  float current7;
  float current8;
  float current9;
  float current10;
  float current11;
  float current12;
  float current13;
  float current14;
  float current15;
} messages__msg__Power;

// Struct for a sequence of messages__msg__Power.
typedef struct messages__msg__Power__Sequence
{
  messages__msg__Power * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} messages__msg__Power__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MESSAGES__MSG__DETAIL__POWER__STRUCT_H_
