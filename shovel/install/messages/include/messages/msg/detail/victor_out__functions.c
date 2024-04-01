// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from messages:msg/VictorOut.idl
// generated code does not contain a copyright notice
#include "messages/msg/detail/victor_out__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
messages__msg__VictorOut__init(messages__msg__VictorOut * msg)
{
  if (!msg) {
    return false;
  }
  // device_id
  // bus_voltage
  // output_voltage
  // output_percent
  return true;
}

void
messages__msg__VictorOut__fini(messages__msg__VictorOut * msg)
{
  if (!msg) {
    return;
  }
  // device_id
  // bus_voltage
  // output_voltage
  // output_percent
}

bool
messages__msg__VictorOut__are_equal(const messages__msg__VictorOut * lhs, const messages__msg__VictorOut * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // device_id
  if (lhs->device_id != rhs->device_id) {
    return false;
  }
  // bus_voltage
  if (lhs->bus_voltage != rhs->bus_voltage) {
    return false;
  }
  // output_voltage
  if (lhs->output_voltage != rhs->output_voltage) {
    return false;
  }
  // output_percent
  if (lhs->output_percent != rhs->output_percent) {
    return false;
  }
  return true;
}

bool
messages__msg__VictorOut__copy(
  const messages__msg__VictorOut * input,
  messages__msg__VictorOut * output)
{
  if (!input || !output) {
    return false;
  }
  // device_id
  output->device_id = input->device_id;
  // bus_voltage
  output->bus_voltage = input->bus_voltage;
  // output_voltage
  output->output_voltage = input->output_voltage;
  // output_percent
  output->output_percent = input->output_percent;
  return true;
}

messages__msg__VictorOut *
messages__msg__VictorOut__create()
{
  messages__msg__VictorOut * msg = (messages__msg__VictorOut *)malloc(sizeof(messages__msg__VictorOut));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(messages__msg__VictorOut));
  bool success = messages__msg__VictorOut__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
messages__msg__VictorOut__destroy(messages__msg__VictorOut * msg)
{
  if (msg) {
    messages__msg__VictorOut__fini(msg);
  }
  free(msg);
}


bool
messages__msg__VictorOut__Sequence__init(messages__msg__VictorOut__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  messages__msg__VictorOut * data = NULL;
  if (size) {
    data = (messages__msg__VictorOut *)calloc(size, sizeof(messages__msg__VictorOut));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = messages__msg__VictorOut__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        messages__msg__VictorOut__fini(&data[i - 1]);
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
messages__msg__VictorOut__Sequence__fini(messages__msg__VictorOut__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      messages__msg__VictorOut__fini(&array->data[i]);
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

messages__msg__VictorOut__Sequence *
messages__msg__VictorOut__Sequence__create(size_t size)
{
  messages__msg__VictorOut__Sequence * array = (messages__msg__VictorOut__Sequence *)malloc(sizeof(messages__msg__VictorOut__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = messages__msg__VictorOut__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
messages__msg__VictorOut__Sequence__destroy(messages__msg__VictorOut__Sequence * array)
{
  if (array) {
    messages__msg__VictorOut__Sequence__fini(array);
  }
  free(array);
}

bool
messages__msg__VictorOut__Sequence__are_equal(const messages__msg__VictorOut__Sequence * lhs, const messages__msg__VictorOut__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!messages__msg__VictorOut__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
messages__msg__VictorOut__Sequence__copy(
  const messages__msg__VictorOut__Sequence * input,
  messages__msg__VictorOut__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(messages__msg__VictorOut);
    messages__msg__VictorOut * data =
      (messages__msg__VictorOut *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!messages__msg__VictorOut__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          messages__msg__VictorOut__fini(&data[i]);
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
    if (!messages__msg__VictorOut__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
