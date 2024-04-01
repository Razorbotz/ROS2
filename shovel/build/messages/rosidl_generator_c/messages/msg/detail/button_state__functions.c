// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from messages:msg/ButtonState.idl
// generated code does not contain a copyright notice
#include "messages/msg/detail/button_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
messages__msg__ButtonState__init(messages__msg__ButtonState * msg)
{
  if (!msg) {
    return false;
  }
  // joystick
  // button
  // state
  return true;
}

void
messages__msg__ButtonState__fini(messages__msg__ButtonState * msg)
{
  if (!msg) {
    return;
  }
  // joystick
  // button
  // state
}

bool
messages__msg__ButtonState__are_equal(const messages__msg__ButtonState * lhs, const messages__msg__ButtonState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // joystick
  if (lhs->joystick != rhs->joystick) {
    return false;
  }
  // button
  if (lhs->button != rhs->button) {
    return false;
  }
  // state
  if (lhs->state != rhs->state) {
    return false;
  }
  return true;
}

bool
messages__msg__ButtonState__copy(
  const messages__msg__ButtonState * input,
  messages__msg__ButtonState * output)
{
  if (!input || !output) {
    return false;
  }
  // joystick
  output->joystick = input->joystick;
  // button
  output->button = input->button;
  // state
  output->state = input->state;
  return true;
}

messages__msg__ButtonState *
messages__msg__ButtonState__create()
{
  messages__msg__ButtonState * msg = (messages__msg__ButtonState *)malloc(sizeof(messages__msg__ButtonState));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(messages__msg__ButtonState));
  bool success = messages__msg__ButtonState__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
messages__msg__ButtonState__destroy(messages__msg__ButtonState * msg)
{
  if (msg) {
    messages__msg__ButtonState__fini(msg);
  }
  free(msg);
}


bool
messages__msg__ButtonState__Sequence__init(messages__msg__ButtonState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  messages__msg__ButtonState * data = NULL;
  if (size) {
    data = (messages__msg__ButtonState *)calloc(size, sizeof(messages__msg__ButtonState));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = messages__msg__ButtonState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        messages__msg__ButtonState__fini(&data[i - 1]);
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
messages__msg__ButtonState__Sequence__fini(messages__msg__ButtonState__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      messages__msg__ButtonState__fini(&array->data[i]);
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

messages__msg__ButtonState__Sequence *
messages__msg__ButtonState__Sequence__create(size_t size)
{
  messages__msg__ButtonState__Sequence * array = (messages__msg__ButtonState__Sequence *)malloc(sizeof(messages__msg__ButtonState__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = messages__msg__ButtonState__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
messages__msg__ButtonState__Sequence__destroy(messages__msg__ButtonState__Sequence * array)
{
  if (array) {
    messages__msg__ButtonState__Sequence__fini(array);
  }
  free(array);
}

bool
messages__msg__ButtonState__Sequence__are_equal(const messages__msg__ButtonState__Sequence * lhs, const messages__msg__ButtonState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!messages__msg__ButtonState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
messages__msg__ButtonState__Sequence__copy(
  const messages__msg__ButtonState__Sequence * input,
  messages__msg__ButtonState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(messages__msg__ButtonState);
    messages__msg__ButtonState * data =
      (messages__msg__ButtonState *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!messages__msg__ButtonState__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          messages__msg__ButtonState__fini(&data[i]);
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
    if (!messages__msg__ButtonState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
