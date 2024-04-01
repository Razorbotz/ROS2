// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from messages:msg/ButtonState.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__MSG__DETAIL__BUTTON_STATE__STRUCT_H_
#define MESSAGES__MSG__DETAIL__BUTTON_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/ButtonState in the package messages.
typedef struct messages__msg__ButtonState
{
  uint8_t joystick;
  uint8_t button;
  uint8_t state;
} messages__msg__ButtonState;

// Struct for a sequence of messages__msg__ButtonState.
typedef struct messages__msg__ButtonState__Sequence
{
  messages__msg__ButtonState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} messages__msg__ButtonState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MESSAGES__MSG__DETAIL__BUTTON_STATE__STRUCT_H_
