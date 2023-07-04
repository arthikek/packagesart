// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from raspimouse_msgs:msg/Leds.idl
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
#include "raspimouse_msgs/msg/detail/leds__struct.h"
#include "raspimouse_msgs/msg/detail/leds__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool raspimouse_msgs__msg__leds__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[31];
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
    assert(strncmp("raspimouse_msgs.msg._leds.Leds", full_classname_dest, 30) == 0);
  }
  raspimouse_msgs__msg__Leds * ros_message = _ros_message;
  {  // led0
    PyObject * field = PyObject_GetAttrString(_pymsg, "led0");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->led0 = (Py_True == field);
    Py_DECREF(field);
  }
  {  // led1
    PyObject * field = PyObject_GetAttrString(_pymsg, "led1");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->led1 = (Py_True == field);
    Py_DECREF(field);
  }
  {  // led2
    PyObject * field = PyObject_GetAttrString(_pymsg, "led2");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->led2 = (Py_True == field);
    Py_DECREF(field);
  }
  {  // led3
    PyObject * field = PyObject_GetAttrString(_pymsg, "led3");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->led3 = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * raspimouse_msgs__msg__leds__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Leds */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("raspimouse_msgs.msg._leds");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Leds");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  raspimouse_msgs__msg__Leds * ros_message = (raspimouse_msgs__msg__Leds *)raw_ros_message;
  {  // led0
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->led0 ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "led0", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // led1
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->led1 ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "led1", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // led2
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->led2 ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "led2", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // led3
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->led3 ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "led3", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
