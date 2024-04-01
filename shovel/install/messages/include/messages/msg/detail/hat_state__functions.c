// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from messages:msg/HatState.idl
// generated code does not contain a copyright notice
#include "messages/msg/detail/hat_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
messages__msg__HatState__init(messages__msg__HatState * msg)
{
  if (!msg) {
    return false;
  }
  // joystick
  // hat
  // state
  return true;
}

void
messages__msg__HatState__fini(messages__msg__HatState * msg)
{
  if (!msg) {
    return;
  }
  // joystick
  // hat
  // state
}

bool
messages__msg__HatState__are_equal(const messages__msg__HatState * lhs, const messages__msg__HatState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // joystick
  if (lhs->joystick != rhs->joystick) {
    return false;
  }
  // hat
  if (lhs->hat != rhs->hat) {
    return false;
  }
  // state
  if (lhs->state != rhs->state) {
    return false;
  }
  return true;
}

bool
messages__msg__HatState__copy(
  const messages__msg__HatState * input,
  messages__msg__HatState * output)
{
  if (!input || !output) {
    return false;
  }
  // joystick
  output->joystick = input->joystick;
  // hat
  output->hat = input->hat;
  // state
  output->state = input->state;
  return true;
}

messages__msg__HatState *
messages__msg__HatState__create()
{
  messages__msg__HatState * msg = (messages__msg__HatState *)malloc(sizeof(messages__msg__HatState));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(messages__msg__HatState));
  bool success = messages__msg__HatState__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
messages__msg__HatState__destroy(messages__msg__HatState * msg)
{
  if (msg) {
    messages__msg__HatState__fini(msg);
  }
  free(msg);
}


bool
messages__msg__HatState__Sequence__init(messages__msg__HatState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  messages__msg__HatState * data = NULL;
  if (size) {
    data = (messages__msg__HatState *)calloc(size, sizeof(messages__msg__HatState));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = messages__msg__HatState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        messages__msg__HatState__fini(&data[i - 1]);
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
messages__msg__HatState__Sequence__fini(messages__msg__HatState__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      messages__msg__HatState__fini(&array->data[i]);
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

messages__msg__HatState__Sequence *
messages__msg__HatState__Sequence__create(size_t size)
{
  messages__msg__HatState__Sequence * array = (messages__msg__HatState__Sequence *)malloc(sizeof(messages__msg__HatState__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = messages__msg__HatState__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
messages__msg__HatState__Sequence__destroy(messages__msg__HatState__Sequence * array)
{
  if (array) {
    messages__msg__HatState__Sequence__fini(array);
  }
  free(array);
}

bool
messages__msg__HatState__Sequence__are_equal(const messages__msg__HatState__Sequence * lhs, const messages__msg__HatState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!messages__msg__HatState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
messages__msg__HatState__Sequence__copy(
  const messages__msg__HatState__Sequence * input,
  messages__msg__HatState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(messages__msg__HatState);
    messages__msg__HatState * data =
      (messages__msg__HatState *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!messages__msg__HatState__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          messages__msg__HatState__fini(&data[i]);
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
    if (!messages__msg__HatState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
