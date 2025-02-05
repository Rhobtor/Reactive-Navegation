// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from hector_gazebo_plugins:srv/SetReferenceGeoPose.idl
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
#include "hector_gazebo_plugins/srv/detail/set_reference_geo_pose__struct.h"
#include "hector_gazebo_plugins/srv/detail/set_reference_geo_pose__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool geographic_msgs__msg__geo_pose__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geographic_msgs__msg__geo_pose__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool hector_gazebo_plugins__srv__set_reference_geo_pose__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[78];
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
    assert(strncmp("hector_gazebo_plugins.srv._set_reference_geo_pose.SetReferenceGeoPose_Request", full_classname_dest, 77) == 0);
  }
  hector_gazebo_plugins__srv__SetReferenceGeoPose_Request * ros_message = _ros_message;
  {  // geo_pose
    PyObject * field = PyObject_GetAttrString(_pymsg, "geo_pose");
    if (!field) {
      return false;
    }
    if (!geographic_msgs__msg__geo_pose__convert_from_py(field, &ros_message->geo_pose)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * hector_gazebo_plugins__srv__set_reference_geo_pose__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of SetReferenceGeoPose_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("hector_gazebo_plugins.srv._set_reference_geo_pose");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "SetReferenceGeoPose_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  hector_gazebo_plugins__srv__SetReferenceGeoPose_Request * ros_message = (hector_gazebo_plugins__srv__SetReferenceGeoPose_Request *)raw_ros_message;
  {  // geo_pose
    PyObject * field = NULL;
    field = geographic_msgs__msg__geo_pose__convert_to_py(&ros_message->geo_pose);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "geo_pose", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
// already included above
// #include <Python.h>
// already included above
// #include <stdbool.h>
// already included above
// #include "numpy/ndarrayobject.h"
// already included above
// #include "rosidl_runtime_c/visibility_control.h"
// already included above
// #include "hector_gazebo_plugins/srv/detail/set_reference_geo_pose__struct.h"
// already included above
// #include "hector_gazebo_plugins/srv/detail/set_reference_geo_pose__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool hector_gazebo_plugins__srv__set_reference_geo_pose__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[79];
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
    assert(strncmp("hector_gazebo_plugins.srv._set_reference_geo_pose.SetReferenceGeoPose_Response", full_classname_dest, 78) == 0);
  }
  hector_gazebo_plugins__srv__SetReferenceGeoPose_Response * ros_message = _ros_message;
  ros_message->structure_needs_at_least_one_member = 0;

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * hector_gazebo_plugins__srv__set_reference_geo_pose__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of SetReferenceGeoPose_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("hector_gazebo_plugins.srv._set_reference_geo_pose");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "SetReferenceGeoPose_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  (void)raw_ros_message;

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
