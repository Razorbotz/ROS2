// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from messages:msg/TalonOut.idl
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
#include "messages/msg/detail/talon_out__struct.h"
#include "messages/msg/detail/talon_out__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool messages__msg__talon_out__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[33];
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
    assert(strncmp("messages.msg._talon_out.TalonOut", full_classname_dest, 32) == 0);
  }
  messages__msg__TalonOut * ros_message = _ros_message;
  {  // device_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "device_id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->device_id = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // bus_voltage
    PyObject * field = PyObject_GetAttrString(_pymsg, "bus_voltage");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->bus_voltage = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // output_current
    PyObject * field = PyObject_GetAttrString(_pymsg, "output_current");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->output_current = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // output_voltage
    PyObject * field = PyObject_GetAttrString(_pymsg, "output_voltage");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->output_voltage = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // output_percent
    PyObject * field = PyObject_GetAttrString(_pymsg, "output_percent");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->output_percent = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // temperature
    PyObject * field = PyObject_GetAttrString(_pymsg, "temperature");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->temperature = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // sensor_position
    PyObject * field = PyObject_GetAttrString(_pymsg, "sensor_position");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->sensor_position = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // sensor_velocity
    PyObject * field = PyObject_GetAttrString(_pymsg, "sensor_velocity");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->sensor_velocity = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // closed_loop_error
    PyObject * field = PyObject_GetAttrString(_pymsg, "closed_loop_error");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->closed_loop_error = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // integral_accumulator
    PyObject * field = PyObject_GetAttrString(_pymsg, "integral_accumulator");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->integral_accumulator = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // error_derivative
    PyObject * field = PyObject_GetAttrString(_pymsg, "error_derivative");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->error_derivative = (int32_t)PyLong_AsLong(field);
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

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * messages__msg__talon_out__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of TalonOut */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("messages.msg._talon_out");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "TalonOut");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  messages__msg__TalonOut * ros_message = (messages__msg__TalonOut *)raw_ros_message;
  {  // device_id
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->device_id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "device_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // bus_voltage
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->bus_voltage);
    {
      int rc = PyObject_SetAttrString(_pymessage, "bus_voltage", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // output_current
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->output_current);
    {
      int rc = PyObject_SetAttrString(_pymessage, "output_current", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // output_voltage
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->output_voltage);
    {
      int rc = PyObject_SetAttrString(_pymessage, "output_voltage", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // output_percent
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->output_percent);
    {
      int rc = PyObject_SetAttrString(_pymessage, "output_percent", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // temperature
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->temperature);
    {
      int rc = PyObject_SetAttrString(_pymessage, "temperature", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // sensor_position
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->sensor_position);
    {
      int rc = PyObject_SetAttrString(_pymessage, "sensor_position", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // sensor_velocity
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->sensor_velocity);
    {
      int rc = PyObject_SetAttrString(_pymessage, "sensor_velocity", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // closed_loop_error
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->closed_loop_error);
    {
      int rc = PyObject_SetAttrString(_pymessage, "closed_loop_error", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // integral_accumulator
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->integral_accumulator);
    {
      int rc = PyObject_SetAttrString(_pymessage, "integral_accumulator", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // error_derivative
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->error_derivative);
    {
      int rc = PyObject_SetAttrString(_pymessage, "error_derivative", field);
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

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
