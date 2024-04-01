// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from messages:msg/Camera.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__MSG__DETAIL__CAMERA__STRUCT_H_
#define MESSAGES__MSG__DETAIL__CAMERA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'address'
#include "rosidl_runtime_c/string.h"

// Struct defined in msg/Camera in the package messages.
typedef struct messages__msg__Camera
{
  int32_t port;
  rosidl_runtime_c__String address;
} messages__msg__Camera;

// Struct for a sequence of messages__msg__Camera.
typedef struct messages__msg__Camera__Sequence
{
  messages__msg__Camera * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} messages__msg__Camera__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MESSAGES__MSG__DETAIL__CAMERA__STRUCT_H_
