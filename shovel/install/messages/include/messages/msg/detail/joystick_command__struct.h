// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from messages:msg/JoystickCommand.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__MSG__DETAIL__JOYSTICK_COMMAND__STRUCT_H_
#define MESSAGES__MSG__DETAIL__JOYSTICK_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/JoystickCommand in the package messages.
typedef struct messages__msg__JoystickCommand
{
  int8_t number;
  int8_t element;
  float value;
} messages__msg__JoystickCommand;

// Struct for a sequence of messages__msg__JoystickCommand.
typedef struct messages__msg__JoystickCommand__Sequence
{
  messages__msg__JoystickCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} messages__msg__JoystickCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MESSAGES__MSG__DETAIL__JOYSTICK_COMMAND__STRUCT_H_
