// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from messages:msg/LinearOut.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__MSG__DETAIL__LINEAR_OUT__FUNCTIONS_H_
#define MESSAGES__MSG__DETAIL__LINEAR_OUT__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "messages/msg/rosidl_generator_c__visibility_control.h"

#include "messages/msg/detail/linear_out__struct.h"

/// Initialize msg/LinearOut message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * messages__msg__LinearOut
 * )) before or use
 * messages__msg__LinearOut__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_messages
bool
messages__msg__LinearOut__init(messages__msg__LinearOut * msg);

/// Finalize msg/LinearOut message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_messages
void
messages__msg__LinearOut__fini(messages__msg__LinearOut * msg);

/// Create msg/LinearOut message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * messages__msg__LinearOut__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_messages
messages__msg__LinearOut *
messages__msg__LinearOut__create();

/// Destroy msg/LinearOut message.
/**
 * It calls
 * messages__msg__LinearOut__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_messages
void
messages__msg__LinearOut__destroy(messages__msg__LinearOut * msg);

/// Check for msg/LinearOut message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_messages
bool
messages__msg__LinearOut__are_equal(const messages__msg__LinearOut * lhs, const messages__msg__LinearOut * rhs);

/// Copy a msg/LinearOut message.
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
messages__msg__LinearOut__copy(
  const messages__msg__LinearOut * input,
  messages__msg__LinearOut * output);

/// Initialize array of msg/LinearOut messages.
/**
 * It allocates the memory for the number of elements and calls
 * messages__msg__LinearOut__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_messages
bool
messages__msg__LinearOut__Sequence__init(messages__msg__LinearOut__Sequence * array, size_t size);

/// Finalize array of msg/LinearOut messages.
/**
 * It calls
 * messages__msg__LinearOut__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_messages
void
messages__msg__LinearOut__Sequence__fini(messages__msg__LinearOut__Sequence * array);

/// Create array of msg/LinearOut messages.
/**
 * It allocates the memory for the array and calls
 * messages__msg__LinearOut__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_messages
messages__msg__LinearOut__Sequence *
messages__msg__LinearOut__Sequence__create(size_t size);

/// Destroy array of msg/LinearOut messages.
/**
 * It calls
 * messages__msg__LinearOut__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_messages
void
messages__msg__LinearOut__Sequence__destroy(messages__msg__LinearOut__Sequence * array);

/// Check for msg/LinearOut message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_messages
bool
messages__msg__LinearOut__Sequence__are_equal(const messages__msg__LinearOut__Sequence * lhs, const messages__msg__LinearOut__Sequence * rhs);

/// Copy an array of msg/LinearOut messages.
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
messages__msg__LinearOut__Sequence__copy(
  const messages__msg__LinearOut__Sequence * input,
  messages__msg__LinearOut__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // MESSAGES__MSG__DETAIL__LINEAR_OUT__FUNCTIONS_H_
