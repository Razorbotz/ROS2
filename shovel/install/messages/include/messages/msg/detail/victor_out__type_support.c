// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from messages:msg/VictorOut.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "messages/msg/detail/victor_out__rosidl_typesupport_introspection_c.h"
#include "messages/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "messages/msg/detail/victor_out__functions.h"
#include "messages/msg/detail/victor_out__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void VictorOut__rosidl_typesupport_introspection_c__VictorOut_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  messages__msg__VictorOut__init(message_memory);
}

void VictorOut__rosidl_typesupport_introspection_c__VictorOut_fini_function(void * message_memory)
{
  messages__msg__VictorOut__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember VictorOut__rosidl_typesupport_introspection_c__VictorOut_message_member_array[4] = {
  {
    "device_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(messages__msg__VictorOut, device_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "bus_voltage",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(messages__msg__VictorOut, bus_voltage),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "output_voltage",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(messages__msg__VictorOut, output_voltage),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "output_percent",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(messages__msg__VictorOut, output_percent),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers VictorOut__rosidl_typesupport_introspection_c__VictorOut_message_members = {
  "messages__msg",  // message namespace
  "VictorOut",  // message name
  4,  // number of fields
  sizeof(messages__msg__VictorOut),
  VictorOut__rosidl_typesupport_introspection_c__VictorOut_message_member_array,  // message members
  VictorOut__rosidl_typesupport_introspection_c__VictorOut_init_function,  // function to initialize message memory (memory has to be allocated)
  VictorOut__rosidl_typesupport_introspection_c__VictorOut_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t VictorOut__rosidl_typesupport_introspection_c__VictorOut_message_type_support_handle = {
  0,
  &VictorOut__rosidl_typesupport_introspection_c__VictorOut_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_messages
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, messages, msg, VictorOut)() {
  if (!VictorOut__rosidl_typesupport_introspection_c__VictorOut_message_type_support_handle.typesupport_identifier) {
    VictorOut__rosidl_typesupport_introspection_c__VictorOut_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &VictorOut__rosidl_typesupport_introspection_c__VictorOut_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
