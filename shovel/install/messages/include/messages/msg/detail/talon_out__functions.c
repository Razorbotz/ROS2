// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from messages:msg/TalonOut.idl
// generated code does not contain a copyright notice
#include "messages/msg/detail/talon_out__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
messages__msg__TalonOut__init(messages__msg__TalonOut * msg)
{
  if (!msg) {
    return false;
  }
  // device_id
  // bus_voltage
  // output_current
  // output_voltage
  // output_percent
  // temperature
  // sensor_position
  // sensor_velocity
  // closed_loop_error
  // integral_accumulator
  // error_derivative
  // potentiometer
  return true;
}

void
messages__msg__TalonOut__fini(messages__msg__TalonOut * msg)
{
  if (!msg) {
    return;
  }
  // device_id
  // bus_voltage
  // output_current
  // output_voltage
  // output_percent
  // temperature
  // sensor_position
  // sensor_velocity
  // closed_loop_error
  // integral_accumulator
  // error_derivative
  // potentiometer
}

bool
messages__msg__TalonOut__are_equal(const messages__msg__TalonOut * lhs, const messages__msg__TalonOut * rhs)
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
  // output_current
  if (lhs->output_current != rhs->output_current) {
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
  // temperature
  if (lhs->temperature != rhs->temperature) {
    return false;
  }
  // sensor_position
  if (lhs->sensor_position != rhs->sensor_position) {
    return false;
  }
  // sensor_velocity
  if (lhs->sensor_velocity != rhs->sensor_velocity) {
    return false;
  }
  // closed_loop_error
  if (lhs->closed_loop_error != rhs->closed_loop_error) {
    return false;
  }
  // integral_accumulator
  if (lhs->integral_accumulator != rhs->integral_accumulator) {
    return false;
  }
  // error_derivative
  if (lhs->error_derivative != rhs->error_derivative) {
    return false;
  }
  // potentiometer
  if (lhs->potentiometer != rhs->potentiometer) {
    return false;
  }
  return true;
}

bool
messages__msg__TalonOut__copy(
  const messages__msg__TalonOut * input,
  messages__msg__TalonOut * output)
{
  if (!input || !output) {
    return false;
  }
  // device_id
  output->device_id = input->device_id;
  // bus_voltage
  output->bus_voltage = input->bus_voltage;
  // output_current
  output->output_current = input->output_current;
  // output_voltage
  output->output_voltage = input->output_voltage;
  // output_percent
  output->output_percent = input->output_percent;
  // temperature
  output->temperature = input->temperature;
  // sensor_position
  output->sensor_position = input->sensor_position;
  // sensor_velocity
  output->sensor_velocity = input->sensor_velocity;
  // closed_loop_error
  output->closed_loop_error = input->closed_loop_error;
  // integral_accumulator
  output->integral_accumulator = input->integral_accumulator;
  // error_derivative
  output->error_derivative = input->error_derivative;
  // potentiometer
  output->potentiometer = input->potentiometer;
  return true;
}

messages__msg__TalonOut *
messages__msg__TalonOut__create()
{
  messages__msg__TalonOut * msg = (messages__msg__TalonOut *)malloc(sizeof(messages__msg__TalonOut));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(messages__msg__TalonOut));
  bool success = messages__msg__TalonOut__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
messages__msg__TalonOut__destroy(messages__msg__TalonOut * msg)
{
  if (msg) {
    messages__msg__TalonOut__fini(msg);
  }
  free(msg);
}


bool
messages__msg__TalonOut__Sequence__init(messages__msg__TalonOut__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  messages__msg__TalonOut * data = NULL;
  if (size) {
    data = (messages__msg__TalonOut *)calloc(size, sizeof(messages__msg__TalonOut));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = messages__msg__TalonOut__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        messages__msg__TalonOut__fini(&data[i - 1]);
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
messages__msg__TalonOut__Sequence__fini(messages__msg__TalonOut__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      messages__msg__TalonOut__fini(&array->data[i]);
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

messages__msg__TalonOut__Sequence *
messages__msg__TalonOut__Sequence__create(size_t size)
{
  messages__msg__TalonOut__Sequence * array = (messages__msg__TalonOut__Sequence *)malloc(sizeof(messages__msg__TalonOut__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = messages__msg__TalonOut__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
messages__msg__TalonOut__Sequence__destroy(messages__msg__TalonOut__Sequence * array)
{
  if (array) {
    messages__msg__TalonOut__Sequence__fini(array);
  }
  free(array);
}

bool
messages__msg__TalonOut__Sequence__are_equal(const messages__msg__TalonOut__Sequence * lhs, const messages__msg__TalonOut__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!messages__msg__TalonOut__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
messages__msg__TalonOut__Sequence__copy(
  const messages__msg__TalonOut__Sequence * input,
  messages__msg__TalonOut__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(messages__msg__TalonOut);
    messages__msg__TalonOut * data =
      (messages__msg__TalonOut *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!messages__msg__TalonOut__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          messages__msg__TalonOut__fini(&data[i]);
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
    if (!messages__msg__TalonOut__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
