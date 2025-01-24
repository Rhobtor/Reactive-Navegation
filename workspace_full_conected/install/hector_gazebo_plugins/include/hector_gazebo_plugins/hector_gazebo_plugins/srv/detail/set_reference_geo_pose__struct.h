// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hector_gazebo_plugins:srv/SetReferenceGeoPose.idl
// generated code does not contain a copyright notice

#ifndef HECTOR_GAZEBO_PLUGINS__SRV__DETAIL__SET_REFERENCE_GEO_POSE__STRUCT_H_
#define HECTOR_GAZEBO_PLUGINS__SRV__DETAIL__SET_REFERENCE_GEO_POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'geo_pose'
#include "geographic_msgs/msg/detail/geo_pose__struct.h"

/// Struct defined in srv/SetReferenceGeoPose in the package hector_gazebo_plugins.
typedef struct hector_gazebo_plugins__srv__SetReferenceGeoPose_Request
{
  geographic_msgs__msg__GeoPose geo_pose;
} hector_gazebo_plugins__srv__SetReferenceGeoPose_Request;

// Struct for a sequence of hector_gazebo_plugins__srv__SetReferenceGeoPose_Request.
typedef struct hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__Sequence
{
  hector_gazebo_plugins__srv__SetReferenceGeoPose_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/SetReferenceGeoPose in the package hector_gazebo_plugins.
typedef struct hector_gazebo_plugins__srv__SetReferenceGeoPose_Response
{
  uint8_t structure_needs_at_least_one_member;
} hector_gazebo_plugins__srv__SetReferenceGeoPose_Response;

// Struct for a sequence of hector_gazebo_plugins__srv__SetReferenceGeoPose_Response.
typedef struct hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__Sequence
{
  hector_gazebo_plugins__srv__SetReferenceGeoPose_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HECTOR_GAZEBO_PLUGINS__SRV__DETAIL__SET_REFERENCE_GEO_POSE__STRUCT_H_
