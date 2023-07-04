// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from raspimouse_msgs:msg/LightSensors.idl
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
#include "raspimouse_msgs/msg/detail/light_sensors__struct.h"
#include "raspimouse_msgs/msg/detail/light_sensors__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool raspimouse_msgs__msg__light_sensors__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[48];
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
    assert(strncmp("raspimouse_msgs.msg._light_sensors.LightSensors", full_classname_dest, 47) == 0);
  }
  raspimouse_msgs__msg__LightSensors * ros_message = _ros_message;
  {  // forward_r
    PyObject * field = PyObject_GetAttrString(_pymsg, "forward_r");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->forward_r = (int16_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // forward_l
    PyObject * field = PyObject_GetAttrString(_pymsg, "forward_l");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->forward_l = (int16_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // left
    PyObject * field = PyObject_GetAttrString(_pymsg, "left");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->left = (int16_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // right
    PyObject * field = PyObject_GetAttrString(_pymsg, "right");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->right = (int16_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * raspimouse_msgs__msg__light_sensors__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of LightSensors */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("raspimouse_msgs.msg._light_sensors");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "LightSensors");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  raspimouse_msgs__msg__LightSensors * ros_message = (raspimouse_msgs__msg__LightSensors *)raw_ros_message;
  {  // forward_r
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->forward_r);
    {
      int rc = PyObject_SetAttrString(_pymessage, "forward_r", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // forward_l
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->forward_l);
    {
      int rc = PyObject_SetAttrString(_pymessage, "forward_l", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // left
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->left);
    {
      int rc = PyObject_SetAttrString(_pymessage, "left", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // right
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->right);
    {
      int rc = PyObject_SetAttrString(_pymessage, "right", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
