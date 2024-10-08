// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from dynamixel_sdk_custom_interfaces:msg/SetVelocityDual.idl
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
#include "dynamixel_sdk_custom_interfaces/msg/detail/set_velocity_dual__struct.h"
#include "dynamixel_sdk_custom_interfaces/msg/detail/set_velocity_dual__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool dynamixel_sdk_custom_interfaces__msg__set_velocity_dual__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[71];
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
    assert(strncmp("dynamixel_sdk_custom_interfaces.msg._set_velocity_dual.SetVelocityDual", full_classname_dest, 70) == 0);
  }
  dynamixel_sdk_custom_interfaces__msg__SetVelocityDual * ros_message = _ros_message;
  {  // motorspeed1
    PyObject * field = PyObject_GetAttrString(_pymsg, "motorspeed1");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->motorspeed1 = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // motorspeed2
    PyObject * field = PyObject_GetAttrString(_pymsg, "motorspeed2");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->motorspeed2 = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * dynamixel_sdk_custom_interfaces__msg__set_velocity_dual__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of SetVelocityDual */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("dynamixel_sdk_custom_interfaces.msg._set_velocity_dual");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "SetVelocityDual");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  dynamixel_sdk_custom_interfaces__msg__SetVelocityDual * ros_message = (dynamixel_sdk_custom_interfaces__msg__SetVelocityDual *)raw_ros_message;
  {  // motorspeed1
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->motorspeed1);
    {
      int rc = PyObject_SetAttrString(_pymessage, "motorspeed1", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // motorspeed2
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->motorspeed2);
    {
      int rc = PyObject_SetAttrString(_pymessage, "motorspeed2", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
