// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from messages:msg/Power.idl
// generated code does not contain a copyright notice
#include "messages/msg/detail/power__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
messages__msg__Power__init(messages__msg__Power * msg)
{
  if (!msg) {
    return false;
  }
  // voltage
  // temperature
  // current0
  // current1
  // current2
  // current3
  // current4
  // current5
  // current6
  // current7
  // current8
  // current9
  // current10
  // current11
  // current12
  // current13
  // current14
  // current15
  return true;
}

void
messages__msg__Power__fini(messages__msg__Power * msg)
{
  if (!msg) {
    return;
  }
  // voltage
  // temperature
  // current0
  // current1
  // current2
  // current3
  // current4
  // current5
  // current6
  // current7
  // current8
  // current9
  // current10
  // current11
  // current12
  // current13
  // current14
  // current15
}

bool
messages__msg__Power__are_equal(const messages__msg__Power * lhs, const messages__msg__Power * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // voltage
  if (lhs->voltage != rhs->voltage) {
    return false;
  }
  // temperature
  if (lhs->temperature != rhs->temperature) {
    return false;
  }
  // current0
  if (lhs->current0 != rhs->current0) {
    return false;
  }
  // current1
  if (lhs->current1 != rhs->current1) {
    return false;
  }
  // current2
  if (lhs->current2 != rhs->current2) {
    return false;
  }
  // current3
  if (lhs->current3 != rhs->current3) {
    return false;
  }
  // current4
  if (lhs->current4 != rhs->current4) {
    return false;
  }
  // current5
  if (lhs->current5 != rhs->current5) {
    return false;
  }
  // current6
  if (lhs->current6 != rhs->current6) {
    return false;
  }
  // current7
  if (lhs->current7 != rhs->current7) {
    return false;
  }
  // current8
  if (lhs->current8 != rhs->current8) {
    return false;
  }
  // current9
  if (lhs->current9 != rhs->current9) {
    return false;
  }
  // current10
  if (lhs->current10 != rhs->current10) {
    return false;
  }
  // current11
  if (lhs->current11 != rhs->current11) {
    return false;
  }
  // current12
  if (lhs->current12 != rhs->current12) {
    return false;
  }
  // current13
  if (lhs->current13 != rhs->current13) {
    return false;
  }
  // current14
  if (lhs->current14 != rhs->current14) {
    return false;
  }
  // current15
  if (lhs->current15 != rhs->current15) {
    return false;
  }
  return true;
}

bool
messages__msg__Power__copy(
  const messages__msg__Power * input,
  messages__msg__Power * output)
{
  if (!input || !output) {
    return false;
  }
  // voltage
  output->voltage = input->voltage;
  // temperature
  output->temperature = input->temperature;
  // current0
  output->current0 = input->current0;
  // current1
  output->current1 = input->current1;
  // current2
  output->current2 = input->current2;
  // current3
  output->current3 = input->current3;
  // current4
  output->current4 = input->current4;
  // current5
  output->current5 = input->current5;
  // current6
  output->current6 = input->current6;
  // current7
  output->current7 = input->current7;
  // current8
  output->current8 = input->current8;
  // current9
  output->current9 = input->current9;
  // current10
  output->current10 = input->current10;
  // current11
  output->current11 = input->current11;
  // current12
  output->current12 = input->current12;
  // current13
  output->current13 = input->current13;
  // current14
  output->current14 = input->current14;
  // current15
  output->current15 = input->current15;
  return true;
}

messages__msg__Power *
messages__msg__Power__create()
{
  messages__msg__Power * msg = (messages__msg__Power *)malloc(sizeof(messages__msg__Power));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(messages__msg__Power));
  bool success = messages__msg__Power__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
messages__msg__Power__destroy(messages__msg__Power * msg)
{
  if (msg) {
    messages__msg__Power__fini(msg);
  }
  free(msg);
}


bool
messages__msg__Power__Sequence__init(messages__msg__Power__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  messages__msg__Power * data = NULL;
  if (size) {
    data = (messages__msg__Power *)calloc(size, sizeof(messages__msg__Power));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = messages__msg__Power__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        messages__msg__Power__fini(&data[i - 1]);
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
messages__msg__Power__Sequence__fini(messages__msg__Power__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      messages__msg__Power__fini(&array->data[i]);
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

messages__msg__Power__Sequence *
messages__msg__Power__Sequence__create(size_t size)
{
  messages__msg__Power__Sequence * array = (messages__msg__Power__Sequence *)malloc(sizeof(messages__msg__Power__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = messages__msg__Power__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
messages__msg__Power__Sequence__destroy(messages__msg__Power__Sequence * array)
{
  if (array) {
    messages__msg__Power__Sequence__fini(array);
  }
  free(array);
}

bool
messages__msg__Power__Sequence__are_equal(const messages__msg__Power__Sequence * lhs, const messages__msg__Power__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!messages__msg__Power__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
messages__msg__Power__Sequence__copy(
  const messages__msg__Power__Sequence * input,
  messages__msg__Power__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(messages__msg__Power);
    messages__msg__Power * data =
      (messages__msg__Power *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!messages__msg__Power__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          messages__msg__Power__fini(&data[i]);
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
    if (!messages__msg__Power__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
