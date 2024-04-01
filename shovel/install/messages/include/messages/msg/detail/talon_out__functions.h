// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from messages:msg/TalonOut.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__MSG__DETAIL__TALON_OUT__FUNCTIONS_H_
#define MESSAGES__MSG__DETAIL__TALON_OUT__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "messages/msg/rosidl_generator_c__visibility_control.h"

#include "messages/msg/detail/talon_out__struct.h"

/// Initialize msg/TalonOut message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * messages__msg__TalonOut
 * )) before or use
 * messages__msg__TalonOut__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_messages
bool
messages__msg__TalonOut__init(messages__msg__TalonOut * msg);

/// Finalize msg/TalonOut message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_messages
void
messages__msg__TalonOut__fini(messages__msg__TalonOut * msg);

/// Create msg/TalonOut message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * messages__msg__TalonOut__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_messages
messages__msg__TalonOut *
messages__msg__TalonOut__create();

/// Destroy msg/TalonOut message.
/**
 * It calls
 * messages__msg__TalonOut__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_messages
void
messages__msg__TalonOut__destroy(messages__msg__TalonOut * msg);

/// Check for msg/TalonOut message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_messages
bool
messages__msg__TalonOut__are_equal(const messages__msg__TalonOut * lhs, const messages__msg__TalonOut * rhs);

/// Copy a msg/TalonOut message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_messages
bool
messages__msg__TalonOut__copy(
  const messages__msg__TalonOut * input,
  messages__msg__TalonOut * output);

/// Initialize array of msg/TalonOut messages.
/**
 * It allocates the memory for the number of elements and calls
 * messages__msg__TalonOut__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_messages
bool
messages__msg__TalonOut__Sequence__init(messages__msg__TalonOut__Sequence * array, size_t size);

/// Finalize array of msg/TalonOut messages.
/**
 * It calls
 * messages__msg__TalonOut__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_messages
void
messages__msg__TalonOut__Sequence__fini(messages__msg__TalonOut__Sequence * array);

/// Create array of msg/TalonOut messages.
/**
 * It allocates the memory for the array and calls
 * messages__msg__TalonOut__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_messages
messages__msg__TalonOut__Sequence *
messages__msg__TalonOut__Sequence__create(size_t size);

/// Destroy array of msg/TalonOut messages.
/**
 * It calls
 * messages__msg__TalonOut__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_messages
void
messages__msg__TalonOut__Sequence__destroy(messages__msg__TalonOut__Sequence * array);

/// Check for msg/TalonOut message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_messages
bool
messages__msg__TalonOut__Sequence__are_equal(const messages__msg__TalonOut__Sequence * lhs, const messages__msg__TalonOut__Sequence * rhs);

/// Copy an array of msg/TalonOut messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_messages
bool
messages__msg__TalonOut__Sequence__copy(
  const messages__msg__TalonOut__Sequence * input,
  messages__msg__TalonOut__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // MESSAGES__MSG__DETAIL__TALON_OUT__FUNCTIONS_H_
