// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from messages:msg/JoystickCommand.idl
// generated code does not contain a copyright notice
#include "messages/msg/detail/joystick_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
messages__msg__JoystickCommand__init(messages__msg__JoystickCommand * msg)
{
  if (!msg) {
    return false;
  }
  // number
  // element
  // value
  return true;
}

void
messages__msg__JoystickCommand__fini(messages__msg__JoystickCommand * msg)
{
  if (!msg) {
    return;
  }
  // number
  // element
  // value
}

bool
messages__msg__JoystickCommand__are_equal(const messages__msg__JoystickCommand * lhs, const messages__msg__JoystickCommand * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // number
  if (lhs->number != rhs->number) {
    return false;
  }
  // element
  if (lhs->element != rhs->element) {
    return false;
  }
  // value
  if (lhs->value != rhs->value) {
    return false;
  }
  return true;
}

bool
messages__msg__JoystickCommand__copy(
  const messages__msg__JoystickCommand * input,
  messages__msg__JoystickCommand * output)
{
  if (!input || !output) {
    return false;
  }
  // number
  output->number = input->number;
  // element
  output->element = input->element;
  // value
  output->value = input->value;
  return true;
}

messages__msg__JoystickCommand *
messages__msg__JoystickCommand__create()
{
  messages__msg__JoystickCommand * msg = (messages__msg__JoystickCommand *)malloc(sizeof(messages__msg__JoystickCommand));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(messages__msg__JoystickCommand));
  bool success = messages__msg__JoystickCommand__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
messages__msg__JoystickCommand__destroy(messages__msg__JoystickCommand * msg)
{
  if (msg) {
    messages__msg__JoystickCommand__fini(msg);
  }
  free(msg);
}


bool
messages__msg__JoystickCommand__Sequence__init(messages__msg__JoystickCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  messages__msg__JoystickCommand * data = NULL;
  if (size) {
    data = (messages__msg__JoystickCommand *)calloc(size, sizeof(messages__msg__JoystickCommand));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = messages__msg__JoystickCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        messages__msg__JoystickCommand__fini(&data[i - 1]);
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
messages__msg__JoystickCommand__Sequence__fini(messages__msg__JoystickCommand__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      messages__msg__JoystickCommand__fini(&array->data[i]);
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

messages__msg__JoystickCommand__Sequence *
messages__msg__JoystickCommand__Sequence__create(size_t size)
{
  messages__msg__JoystickCommand__Sequence * array = (messages__msg__JoystickCommand__Sequence *)malloc(sizeof(messages__msg__JoystickCommand__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = messages__msg__JoystickCommand__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
messages__msg__JoystickCommand__Sequence__destroy(messages__msg__JoystickCommand__Sequence * array)
{
  if (array) {
    messages__msg__JoystickCommand__Sequence__fini(array);
  }
  free(array);
}

bool
messages__msg__JoystickCommand__Sequence__are_equal(const messages__msg__JoystickCommand__Sequence * lhs, const messages__msg__JoystickCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!messages__msg__JoystickCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
messages__msg__JoystickCommand__Sequence__copy(
  const messages__msg__JoystickCommand__Sequence * input,
  messages__msg__JoystickCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(messages__msg__JoystickCommand);
    messages__msg__JoystickCommand * data =
      (messages__msg__JoystickCommand *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!messages__msg__JoystickCommand__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          messages__msg__JoystickCommand__fini(&data[i]);
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
    if (!messages__msg__JoystickCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
