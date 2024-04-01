// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from messages:msg/AutonomyOut.idl
// generated code does not contain a copyright notice
#include "messages/msg/detail/autonomy_out__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `robot_state`
// Member `excavation_state`
// Member `error_state`
// Member `dump_state`
#include "rosidl_runtime_c/string_functions.h"

bool
messages__msg__AutonomyOut__init(messages__msg__AutonomyOut * msg)
{
  if (!msg) {
    return false;
  }
  // robot_state
  if (!rosidl_runtime_c__String__init(&msg->robot_state)) {
    messages__msg__AutonomyOut__fini(msg);
    return false;
  }
  // excavation_state
  if (!rosidl_runtime_c__String__init(&msg->excavation_state)) {
    messages__msg__AutonomyOut__fini(msg);
    return false;
  }
  // error_state
  if (!rosidl_runtime_c__String__init(&msg->error_state)) {
    messages__msg__AutonomyOut__fini(msg);
    return false;
  }
  // dump_state
  if (!rosidl_runtime_c__String__init(&msg->dump_state)) {
    messages__msg__AutonomyOut__fini(msg);
    return false;
  }
  return true;
}

void
messages__msg__AutonomyOut__fini(messages__msg__AutonomyOut * msg)
{
  if (!msg) {
    return;
  }
  // robot_state
  rosidl_runtime_c__String__fini(&msg->robot_state);
  // excavation_state
  rosidl_runtime_c__String__fini(&msg->excavation_state);
  // error_state
  rosidl_runtime_c__String__fini(&msg->error_state);
  // dump_state
  rosidl_runtime_c__String__fini(&msg->dump_state);
}

bool
messages__msg__AutonomyOut__are_equal(const messages__msg__AutonomyOut * lhs, const messages__msg__AutonomyOut * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // robot_state
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->robot_state), &(rhs->robot_state)))
  {
    return false;
  }
  // excavation_state
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->excavation_state), &(rhs->excavation_state)))
  {
    return false;
  }
  // error_state
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->error_state), &(rhs->error_state)))
  {
    return false;
  }
  // dump_state
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->dump_state), &(rhs->dump_state)))
  {
    return false;
  }
  return true;
}

bool
messages__msg__AutonomyOut__copy(
  const messages__msg__AutonomyOut * input,
  messages__msg__AutonomyOut * output)
{
  if (!input || !output) {
    return false;
  }
  // robot_state
  if (!rosidl_runtime_c__String__copy(
      &(input->robot_state), &(output->robot_state)))
  {
    return false;
  }
  // excavation_state
  if (!rosidl_runtime_c__String__copy(
      &(input->excavation_state), &(output->excavation_state)))
  {
    return false;
  }
  // error_state
  if (!rosidl_runtime_c__String__copy(
      &(input->error_state), &(output->error_state)))
  {
    return false;
  }
  // dump_state
  if (!rosidl_runtime_c__String__copy(
      &(input->dump_state), &(output->dump_state)))
  {
    return false;
  }
  return true;
}

messages__msg__AutonomyOut *
messages__msg__AutonomyOut__create()
{
  messages__msg__AutonomyOut * msg = (messages__msg__AutonomyOut *)malloc(sizeof(messages__msg__AutonomyOut));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(messages__msg__AutonomyOut));
  bool success = messages__msg__AutonomyOut__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
messages__msg__AutonomyOut__destroy(messages__msg__AutonomyOut * msg)
{
  if (msg) {
    messages__msg__AutonomyOut__fini(msg);
  }
  free(msg);
}


bool
messages__msg__AutonomyOut__Sequence__init(messages__msg__AutonomyOut__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  messages__msg__AutonomyOut * data = NULL;
  if (size) {
    data = (messages__msg__AutonomyOut *)calloc(size, sizeof(messages__msg__AutonomyOut));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = messages__msg__AutonomyOut__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        messages__msg__AutonomyOut__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
messages__msg__AutonomyOut__Sequence__fini(messages__msg__AutonomyOut__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      messages__msg__AutonomyOut__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

messages__msg__AutonomyOut__Sequence *
messages__msg__AutonomyOut__Sequence__create(size_t size)
{
  messages__msg__AutonomyOut__Sequence * array = (messages__msg__AutonomyOut__Sequence *)malloc(sizeof(messages__msg__AutonomyOut__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = messages__msg__AutonomyOut__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
messages__msg__AutonomyOut__Sequence__destroy(messages__msg__AutonomyOut__Sequence * array)
{
  if (array) {
    messages__msg__AutonomyOut__Sequence__fini(array);
  }
  free(array);
}

bool
messages__msg__AutonomyOut__Sequence__are_equal(const messages__msg__AutonomyOut__Sequence * lhs, const messages__msg__AutonomyOut__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!messages__msg__AutonomyOut__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
messages__msg__AutonomyOut__Sequence__copy(
  const messages__msg__AutonomyOut__Sequence * input,
  messages__msg__AutonomyOut__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(messages__msg__AutonomyOut);
    messages__msg__AutonomyOut * data =
      (messages__msg__AutonomyOut *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!messages__msg__AutonomyOut__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          messages__msg__AutonomyOut__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!messages__msg__AutonomyOut__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
