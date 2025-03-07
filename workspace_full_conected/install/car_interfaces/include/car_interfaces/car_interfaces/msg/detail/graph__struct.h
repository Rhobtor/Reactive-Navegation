// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from car_interfaces:msg/Graph.idl
// generated code does not contain a copyright notice

#ifndef CAR_INTERFACES__MSG__DETAIL__GRAPH__STRUCT_H_
#define CAR_INTERFACES__MSG__DETAIL__GRAPH__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'nodes'
#include "geometry_msgs/msg/detail/pose_array__struct.h"
// Member 'edges'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/Graph in the package car_interfaces.
/**
  * Graph.msg
 */
typedef struct car_interfaces__msg__Graph
{
  geometry_msgs__msg__PoseArray nodes;
  /// Se interpretan en pares: [nodo0, nodo1, nodo2, nodo3, ...]
  rosidl_runtime_c__int32__Sequence edges;
} car_interfaces__msg__Graph;

// Struct for a sequence of car_interfaces__msg__Graph.
typedef struct car_interfaces__msg__Graph__Sequence
{
  car_interfaces__msg__Graph * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} car_interfaces__msg__Graph__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CAR_INTERFACES__MSG__DETAIL__GRAPH__STRUCT_H_
