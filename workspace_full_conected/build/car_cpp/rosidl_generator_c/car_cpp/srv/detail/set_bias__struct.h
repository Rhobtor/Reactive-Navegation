// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from car_cpp:srv/SetBias.idl
// generated code does not contain a copyright notice

#ifndef CAR_CPP__SRV__DETAIL__SET_BIAS__STRUCT_H_
#define CAR_CPP__SRV__DETAIL__SET_BIAS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'bias'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in srv/SetBias in the package car_cpp.
typedef struct car_cpp__srv__SetBias_Request
{
  geometry_msgs__msg__Vector3 bias;
} car_cpp__srv__SetBias_Request;

// Struct for a sequence of car_cpp__srv__SetBias_Request.
typedef struct car_cpp__srv__SetBias_Request__Sequence
{
  car_cpp__srv__SetBias_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} car_cpp__srv__SetBias_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/SetBias in the package car_cpp.
typedef struct car_cpp__srv__SetBias_Response
{
  uint8_t structure_needs_at_least_one_member;
} car_cpp__srv__SetBias_Response;

// Struct for a sequence of car_cpp__srv__SetBias_Response.
typedef struct car_cpp__srv__SetBias_Response__Sequence
{
  car_cpp__srv__SetBias_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} car_cpp__srv__SetBias_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CAR_CPP__SRV__DETAIL__SET_BIAS__STRUCT_H_
