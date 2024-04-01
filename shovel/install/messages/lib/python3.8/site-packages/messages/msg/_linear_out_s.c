// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from messages:msg/LinearOut.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "messages/msg/detail/linear_out__struct.h"
#include "messages/msg/detail/linear_out__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool messages__msg__linear_out__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[35];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("messages.msg._linear_out.LinearOut", full_classname_dest, 34) == 0);
  }
  messages__msg__LinearOut * ros_message = _ros_message;
  {  // speed
    PyObject * field = PyObject_GetAttrString(_pymsg, "speed");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->speed = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // potentiometer
    PyObject * field = PyObject_GetAttrString(_pymsg, "potentiometer");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->potentiometer = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // time_without_change
    PyObject * field = PyObject_GetAttrString(_pymsg, "time_without_change");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->time_without_change = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // max
    PyObject * field = PyObject_GetAttrString(_pymsg, "max");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->max = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // min
    PyObject * field = PyObject_GetAttrString(_pymsg, "min");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->min = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // error
    PyObject * field = PyObject_GetAttrString(_pymsg, "error");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->error, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // run
    PyObject * field = PyObject_GetAttrString(_pymsg, "run");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->run = (Py_True == field);
    Py_DECREF(field);
  }
  {  // at_min
    PyObject * field = PyObject_GetAttrString(_pymsg, "at_min");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->at_min = (Py_True == field);
    Py_DECREF(field);
  }
  {  // at_max
    PyObject * field = PyObject_GetAttrString(_pymsg, "at_max");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->at_max = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * messages__msg__linear_out__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of LinearOut */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("messages.msg._linear_out");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "LinearOut");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  messages__msg__LinearOut * ros_message = (messages__msg__LinearOut *)raw_ros_message;
  {  // speed
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->speed);
    {
      int rc = PyObject_SetAttrString(_pymessage, "speed", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // potentiometer
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->potentiometer);
    {
      int rc = PyObject_SetAttrString(_pymessage, "potentiometer", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // time_without_change
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->time_without_change);
    {
      int rc = PyObject_SetAttrString(_pymessage, "time_without_change", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // max
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->max);
    {
      int rc = PyObject_SetAttrString(_pymessage, "max", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // min
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->min);
    {
      int rc = PyObject_SetAttrString(_pymessage, "min", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // error
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->error.data,
      strlen(ros_message->error.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "error", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // run
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->run ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "run", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // at_min
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->at_min ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "at_min", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // at_max
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->at_max ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "at_max", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
