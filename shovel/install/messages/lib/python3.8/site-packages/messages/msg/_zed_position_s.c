// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from messages:msg/ZedPosition.idl
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
#include "messages/msg/detail/zed_position__struct.h"
#include "messages/msg/detail/zed_position__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool messages__msg__zed_position__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[39];
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
    assert(strncmp("messages.msg._zed_position.ZedPosition", full_classname_dest, 38) == 0);
  }
  messages__msg__ZedPosition * ros_message = _ros_message;
  {  // x
    PyObject * field = PyObject_GetAttrString(_pymsg, "x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->x = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // y
    PyObject * field = PyObject_GetAttrString(_pymsg, "y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->y = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // z
    PyObject * field = PyObject_GetAttrString(_pymsg, "z");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->z = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // ox
    PyObject * field = PyObject_GetAttrString(_pymsg, "ox");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->ox = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // oy
    PyObject * field = PyObject_GetAttrString(_pymsg, "oy");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->oy = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // oz
    PyObject * field = PyObject_GetAttrString(_pymsg, "oz");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->oz = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // ow
    PyObject * field = PyObject_GetAttrString(_pymsg, "ow");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->ow = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // aruco_visible
    PyObject * field = PyObject_GetAttrString(_pymsg, "aruco_visible");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->aruco_visible = (Py_True == field);
    Py_DECREF(field);
  }
  {  // pitch
    PyObject * field = PyObject_GetAttrString(_pymsg, "pitch");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->pitch = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // yaw
    PyObject * field = PyObject_GetAttrString(_pymsg, "yaw");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->yaw = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // roll
    PyObject * field = PyObject_GetAttrString(_pymsg, "roll");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->roll = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // aruco_pitch
    PyObject * field = PyObject_GetAttrString(_pymsg, "aruco_pitch");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->aruco_pitch = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // aruco_yaw
    PyObject * field = PyObject_GetAttrString(_pymsg, "aruco_yaw");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->aruco_yaw = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // aruco_roll
    PyObject * field = PyObject_GetAttrString(_pymsg, "aruco_roll");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->aruco_roll = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // distance
    PyObject * field = PyObject_GetAttrString(_pymsg, "distance");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->distance = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * messages__msg__zed_position__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of ZedPosition */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("messages.msg._zed_position");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "ZedPosition");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  messages__msg__ZedPosition * ros_message = (messages__msg__ZedPosition *)raw_ros_message;
  {  // x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // z
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->z);
    {
      int rc = PyObject_SetAttrString(_pymessage, "z", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ox
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->ox);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ox", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // oy
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->oy);
    {
      int rc = PyObject_SetAttrString(_pymessage, "oy", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // oz
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->oz);
    {
      int rc = PyObject_SetAttrString(_pymessage, "oz", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ow
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->ow);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ow", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // aruco_visible
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->aruco_visible ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "aruco_visible", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pitch
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->pitch);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pitch", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // yaw
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->yaw);
    {
      int rc = PyObject_SetAttrString(_pymessage, "yaw", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // roll
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->roll);
    {
      int rc = PyObject_SetAttrString(_pymessage, "roll", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // aruco_pitch
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->aruco_pitch);
    {
      int rc = PyObject_SetAttrString(_pymessage, "aruco_pitch", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // aruco_yaw
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->aruco_yaw);
    {
      int rc = PyObject_SetAttrString(_pymessage, "aruco_yaw", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // aruco_roll
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->aruco_roll);
    {
      int rc = PyObject_SetAttrString(_pymessage, "aruco_roll", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // distance
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->distance);
    {
      int rc = PyObject_SetAttrString(_pymessage, "distance", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
