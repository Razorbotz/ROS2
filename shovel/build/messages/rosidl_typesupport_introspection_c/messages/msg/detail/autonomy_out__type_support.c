// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from messages:msg/AutonomyOut.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "messages/msg/detail/autonomy_out__rosidl_typesupport_introspection_c.h"
#include "messages/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "messages/msg/detail/autonomy_out__functions.h"
#include "messages/msg/detail/autonomy_out__struct.h"


// Include directives for member types
// Member `robot_state`
// Member `excavation_state`
// Member `error_state`
// Member `dump_state`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void AutonomyOut__rosidl_typesupport_introspection_c__AutonomyOut_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  messages__msg__AutonomyOut__init(message_memory);
}

void AutonomyOut__rosidl_typesupport_introspection_c__AutonomyOut_fini_function(void * message_memory)
{
  messages__msg__AutonomyOut__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember AutonomyOut__rosidl_typesupport_introspection_c__AutonomyOut_message_member_array[4] = {
  {
    "robot_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(messages__msg__AutonomyOut, robot_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "excavation_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(messages__msg__AutonomyOut, excavation_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "error_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(messages__msg__AutonomyOut, error_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "dump_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(messages__msg__AutonomyOut, dump_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers AutonomyOut__rosidl_typesupport_introspection_c__AutonomyOut_message_members = {
  "messages__msg",  // message namespace
  "AutonomyOut",  // message name
  4,  // number of fields
  sizeof(messages__msg__AutonomyOut),
  AutonomyOut__rosidl_typesupport_introspection_c__AutonomyOut_message_member_array,  // message members
  AutonomyOut__rosidl_typesupport_introspection_c__AutonomyOut_init_function,  // function to initialize message memory (memory has to be allocated)
  AutonomyOut__rosidl_typesupport_introspection_c__AutonomyOut_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t AutonomyOut__rosidl_typesupport_introspection_c__AutonomyOut_message_type_support_handle = {
  0,
  &AutonomyOut__rosidl_typesupport_introspection_c__AutonomyOut_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_messages
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, messages, msg, AutonomyOut)() {
  if (!AutonomyOut__rosidl_typesupport_introspection_c__AutonomyOut_message_type_support_handle.typesupport_identifier) {
    AutonomyOut__rosidl_typesupport_introspection_c__AutonomyOut_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &AutonomyOut__rosidl_typesupport_introspection_c__AutonomyOut_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
