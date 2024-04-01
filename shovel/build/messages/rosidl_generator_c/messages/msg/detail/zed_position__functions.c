// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from messages:msg/ZedPosition.idl
// generated code does not contain a copyright notice
#include "messages/msg/detail/zed_position__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
messages__msg__ZedPosition__init(messages__msg__ZedPosition * msg)
{
  if (!msg) {
    return false;
  }
  // x
  // y
  // z
  // ox
  // oy
  // oz
  // ow
  // aruco_visible
  // pitch
  // yaw
  // roll
  // aruco_pitch
  // aruco_yaw
  // aruco_roll
  // distance
  return true;
}

void
messages__msg__ZedPosition__fini(messages__msg__ZedPosition * msg)
{
  if (!msg) {
    return;
  }
  // x
  // y
  // z
  // ox
  // oy
  // oz
  // ow
  // aruco_visible
  // pitch
  // yaw
  // roll
  // aruco_pitch
  // aruco_yaw
  // aruco_roll
  // distance
}

bool
messages__msg__ZedPosition__are_equal(const messages__msg__ZedPosition * lhs, const messages__msg__ZedPosition * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  // z
  if (lhs->z != rhs->z) {
    return false;
  }
  // ox
  if (lhs->ox != rhs->ox) {
    return false;
  }
  // oy
  if (lhs->oy != rhs->oy) {
    return false;
  }
  // oz
  if (lhs->oz != rhs->oz) {
    return false;
  }
  // ow
  if (lhs->ow != rhs->ow) {
    return false;
  }
  // aruco_visible
  if (lhs->aruco_visible != rhs->aruco_visible) {
    return false;
  }
  // pitch
  if (lhs->pitch != rhs->pitch) {
    return false;
  }
  // yaw
  if (lhs->yaw != rhs->yaw) {
    return false;
  }
  // roll
  if (lhs->roll != rhs->roll) {
    return false;
  }
  // aruco_pitch
  if (lhs->aruco_pitch != rhs->aruco_pitch) {
    return false;
  }
  // aruco_yaw
  if (lhs->aruco_yaw != rhs->aruco_yaw) {
    return false;
  }
  // aruco_roll
  if (lhs->aruco_roll != rhs->aruco_roll) {
    return false;
  }
  // distance
  if (lhs->distance != rhs->distance) {
    return false;
  }
  return true;
}

bool
messages__msg__ZedPosition__copy(
  const messages__msg__ZedPosition * input,
  messages__msg__ZedPosition * output)
{
  if (!input || !output) {
    return false;
  }
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // z
  output->z = input->z;
  // ox
  output->ox = input->ox;
  // oy
  output->oy = input->oy;
  // oz
  output->oz = input->oz;
  // ow
  output->ow = input->ow;
  // aruco_visible
  output->aruco_visible = input->aruco_visible;
  // pitch
  output->pitch = input->pitch;
  // yaw
  output->yaw = input->yaw;
  // roll
  output->roll = input->roll;
  // aruco_pitch
  output->aruco_pitch = input->aruco_pitch;
  // aruco_yaw
  output->aruco_yaw = input->aruco_yaw;
  // aruco_roll
  output->aruco_roll = input->aruco_roll;
  // distance
  output->distance = input->distance;
  return true;
}

messages__msg__ZedPosition *
messages__msg__ZedPosition__create()
{
  messages__msg__ZedPosition * msg = (messages__msg__ZedPosition *)malloc(sizeof(messages__msg__ZedPosition));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(messages__msg__ZedPosition));
  bool success = messages__msg__ZedPosition__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
messages__msg__ZedPosition__destroy(messages__msg__ZedPosition * msg)
{
  if (msg) {
    messages__msg__ZedPosition__fini(msg);
  }
  free(msg);
}


bool
messages__msg__ZedPosition__Sequence__init(messages__msg__ZedPosition__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  messages__msg__ZedPosition * data = NULL;
  if (size) {
    data = (messages__msg__ZedPosition *)calloc(size, sizeof(messages__msg__ZedPosition));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = messages__msg__ZedPosition__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        messages__msg__ZedPosition__fini(&data[i - 1]);
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
messages__msg__ZedPosition__Sequence__fini(messages__msg__ZedPosition__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      messages__msg__ZedPosition__fini(&array->data[i]);
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

messages__msg__ZedPosition__Sequence *
messages__msg__ZedPosition__Sequence__create(size_t size)
{
  messages__msg__ZedPosition__Sequence * array = (messages__msg__ZedPosition__Sequence *)malloc(sizeof(messages__msg__ZedPosition__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = messages__msg__ZedPosition__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
messages__msg__ZedPosition__Sequence__destroy(messages__msg__ZedPosition__Sequence * array)
{
  if (array) {
    messages__msg__ZedPosition__Sequence__fini(array);
  }
  free(array);
}

bool
messages__msg__ZedPosition__Sequence__are_equal(const messages__msg__ZedPosition__Sequence * lhs, const messages__msg__ZedPosition__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!messages__msg__ZedPosition__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
messages__msg__ZedPosition__Sequence__copy(
  const messages__msg__ZedPosition__Sequence * input,
  messages__msg__ZedPosition__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(messages__msg__ZedPosition);
    messages__msg__ZedPosition * data =
      (messages__msg__ZedPosition *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!messages__msg__ZedPosition__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          messages__msg__ZedPosition__fini(&data[i]);
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
    if (!messages__msg__ZedPosition__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
