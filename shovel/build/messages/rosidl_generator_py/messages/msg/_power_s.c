// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from messages:msg/Power.idl
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
#include "messages/msg/detail/power__struct.h"
#include "messages/msg/detail/power__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool messages__msg__power__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[26];
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
    assert(strncmp("messages.msg._power.Power", full_classname_dest, 25) == 0);
  }
  messages__msg__Power * ros_message = _ros_message;
  {  // voltage
    PyObject * field = PyObject_GetAttrString(_pymsg, "voltage");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->voltage = (float)PyFloat_AS_DOUBLE(field);
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
  {  // current0
    PyObject * field = PyObject_GetAttrString(_pymsg, "current0");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->current0 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // current1
    PyObject * field = PyObject_GetAttrString(_pymsg, "current1");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->current1 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // current2
    PyObject * field = PyObject_GetAttrString(_pymsg, "current2");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->current2 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // current3
    PyObject * field = PyObject_GetAttrString(_pymsg, "current3");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->current3 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // current4
    PyObject * field = PyObject_GetAttrString(_pymsg, "current4");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->current4 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // current5
    PyObject * field = PyObject_GetAttrString(_pymsg, "current5");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->current5 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // current6
    PyObject * field = PyObject_GetAttrString(_pymsg, "current6");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->current6 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // current7
    PyObject * field = PyObject_GetAttrString(_pymsg, "current7");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->current7 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // current8
    PyObject * field = PyObject_GetAttrString(_pymsg, "current8");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->current8 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // current9
    PyObject * field = PyObject_GetAttrString(_pymsg, "current9");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->current9 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // current10
    PyObject * field = PyObject_GetAttrString(_pymsg, "current10");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->current10 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // current11
    PyObject * field = PyObject_GetAttrString(_pymsg, "current11");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->current11 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // current12
    PyObject * field = PyObject_GetAttrString(_pymsg, "current12");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->current12 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // current13
    PyObject * field = PyObject_GetAttrString(_pymsg, "current13");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->current13 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // current14
    PyObject * field = PyObject_GetAttrString(_pymsg, "current14");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->current14 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // current15
    PyObject * field = PyObject_GetAttrString(_pymsg, "current15");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->current15 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * messages__msg__power__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Power */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("messages.msg._power");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Power");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  messages__msg__Power * ros_message = (messages__msg__Power *)raw_ros_message;
  {  // voltage
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->voltage);
    {
      int rc = PyObject_SetAttrString(_pymessage, "voltage", field);
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
  {  // current0
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->current0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "current0", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // current1
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->current1);
    {
      int rc = PyObject_SetAttrString(_pymessage, "current1", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // current2
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->current2);
    {
      int rc = PyObject_SetAttrString(_pymessage, "current2", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // current3
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->current3);
    {
      int rc = PyObject_SetAttrString(_pymessage, "current3", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // current4
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->current4);
    {
      int rc = PyObject_SetAttrString(_pymessage, "current4", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // current5
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->current5);
    {
      int rc = PyObject_SetAttrString(_pymessage, "current5", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // current6
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->current6);
    {
      int rc = PyObject_SetAttrString(_pymessage, "current6", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // current7
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->current7);
    {
      int rc = PyObject_SetAttrString(_pymessage, "current7", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // current8
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->current8);
    {
      int rc = PyObject_SetAttrString(_pymessage, "current8", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // current9
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->current9);
    {
      int rc = PyObject_SetAttrString(_pymessage, "current9", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // current10
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->current10);
    {
      int rc = PyObject_SetAttrString(_pymessage, "current10", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // current11
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->current11);
    {
      int rc = PyObject_SetAttrString(_pymessage, "current11", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // current12
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->current12);
    {
      int rc = PyObject_SetAttrString(_pymessage, "current12", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // current13
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->current13);
    {
      int rc = PyObject_SetAttrString(_pymessage, "current13", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // current14
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->current14);
    {
      int rc = PyObject_SetAttrString(_pymessage, "current14", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // current15
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->current15);
    {
      int rc = PyObject_SetAttrString(_pymessage, "current15", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
