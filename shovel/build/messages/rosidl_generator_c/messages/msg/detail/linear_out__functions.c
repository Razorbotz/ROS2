// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from messages:msg/LinearOut.idl
// generated code does not contain a copyright notice
#include "messages/msg/detail/linear_out__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `error`
#include "rosidl_runtime_c/string_functions.h"

bool
messages__msg__LinearOut__init(messages__msg__LinearOut * msg)
{
  if (!msg) {
    return false;
  }
  // speed
  // potentiometer
  // time_without_change
  // max
  // min
  // error
  if (!rosidl_runtime_c__String__init(&msg->error)) {
    messages__msg__LinearOut__fini(msg);
    return false;
  }
  // run
  // at_min
  // at_max
  return true;
}

void
messages__msg__LinearOut__fini(messages__msg__LinearOut * msg)
{
  if (!msg) {
    return;
  }
  // speed
  // potentiometer
  // time_without_change
  // max
  // min
  // error
  rosidl_runtime_c__String__fini(&msg->error);
  // run
  // at_min
  // at_max
}

bool
messages__msg__LinearOut__are_equal(const messages__msg__LinearOut * lhs, const messages__msg__LinearOut * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // speed
  if (lhs->speed != rhs->speed) {
    return false;
  }
  // potentiometer
  if (lhs->potentiometer != rhs->potentiometer) {
    return false;
  }
  // time_without_change
  if (lhs->time_without_change != rhs->time_without_change) {
    return false;
  }
  // max
  if (lhs->max != rhs->max) {
    return false;
  }
  // min
  if (lhs->min != rhs->min) {
    return false;
  }
  // error
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->error), &(rhs->error)))
  {
    return false;
  }
  // run
  if (lhs->run != rhs->run) {
    return false;
  }
  // at_min
  if (lhs->at_min != rhs->at_min) {
    return false;
  }
  // at_max
  if (lhs->at_max != rhs->at_max) {
    return false;
  }
  return true;
}

bool
messages__msg__LinearOut__copy(
  const messages__msg__LinearOut * input,
  messages__msg__LinearOut * output)
{
  if (!input || !output) {
    return false;
  }
  // speed
  output->speed = input->speed;
  // potentiometer
  output->potentiometer = input->potentiometer;
  // time_without_change
  output->time_without_change = input->time_without_change;
  // max
  output->max = input->max;
  // min
  output->min = input->min;
  // error
  if (!rosidl_runtime_c__String__copy(
      &(input->error), &(output->error)))
  {
    return false;
  }
  // run
  output->run = input->run;
  // at_min
  output->at_min = input->at_min;
  // at_max
  output->at_max = input->at_max;
  return true;
}

messages__msg__LinearOut *
messages__msg__LinearOut__create()
{
  messages__msg__LinearOut * msg = (messages__msg__LinearOut *)malloc(sizeof(messages__msg__LinearOut));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(messages__msg__LinearOut));
  bool success = messages__msg__LinearOut__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
messages__msg__LinearOut__destroy(messages__msg__LinearOut * msg)
{
  if (msg) {
    messages__msg__LinearOut__fini(msg);
  }
  free(msg);
}


bool
messages__msg__LinearOut__Sequence__init(messages__msg__LinearOut__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  messages__msg__LinearOut * data = NULL;
  if (size) {
    data = (messages__msg__LinearOut *)calloc(size, sizeof(messages__msg__LinearOut));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = messages__msg__LinearOut__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        messages__msg__LinearOut__fini(&data[i - 1]);
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
messages__msg__LinearOut__Sequence__fini(messages__msg__LinearOut__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      messages__msg__LinearOut__fini(&array->data[i]);
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

messages__msg__LinearOut__Sequence *
messages__msg__LinearOut__Sequence__create(size_t size)
{
  messages__msg__LinearOut__Sequence * array = (messages__msg__LinearOut__Sequence *)malloc(sizeof(messages__msg__LinearOut__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = messages__msg__LinearOut__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
messages__msg__LinearOut__Sequence__destroy(messages__msg__LinearOut__Sequence * array)
{
  if (array) {
    messages__msg__LinearOut__Sequence__fini(array);
  }
  free(array);
}

bool
messages__msg__LinearOut__Sequence__are_equal(const messages__msg__LinearOut__Sequence * lhs, const messages__msg__LinearOut__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!messages__msg__LinearOut__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
messages__msg__LinearOut__Sequence__copy(
  const messages__msg__LinearOut__Sequence * input,
  messages__msg__LinearOut__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(messages__msg__LinearOut);
    messages__msg__LinearOut * data =
      (messages__msg__LinearOut *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!messages__msg__LinearOut__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          messages__msg__LinearOut__fini(&data[i]);
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
    if (!messages__msg__LinearOut__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
