// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from car_interfaces:msg/Graph.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "car_interfaces/msg/detail/graph__rosidl_typesupport_introspection_c.h"
#include "car_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "car_interfaces/msg/detail/graph__functions.h"
#include "car_interfaces/msg/detail/graph__struct.h"


// Include directives for member types
// Member `nodes`
#include "geometry_msgs/msg/pose_array.h"
// Member `nodes`
#include "geometry_msgs/msg/detail/pose_array__rosidl_typesupport_introspection_c.h"
// Member `edges`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void car_interfaces__msg__Graph__rosidl_typesupport_introspection_c__Graph_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  car_interfaces__msg__Graph__init(message_memory);
}

void car_interfaces__msg__Graph__rosidl_typesupport_introspection_c__Graph_fini_function(void * message_memory)
{
  car_interfaces__msg__Graph__fini(message_memory);
}

size_t car_interfaces__msg__Graph__rosidl_typesupport_introspection_c__size_function__Graph__edges(
  const void * untyped_member)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return member->size;
}

const void * car_interfaces__msg__Graph__rosidl_typesupport_introspection_c__get_const_function__Graph__edges(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * car_interfaces__msg__Graph__rosidl_typesupport_introspection_c__get_function__Graph__edges(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void car_interfaces__msg__Graph__rosidl_typesupport_introspection_c__fetch_function__Graph__edges(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    car_interfaces__msg__Graph__rosidl_typesupport_introspection_c__get_const_function__Graph__edges(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void car_interfaces__msg__Graph__rosidl_typesupport_introspection_c__assign_function__Graph__edges(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    car_interfaces__msg__Graph__rosidl_typesupport_introspection_c__get_function__Graph__edges(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

bool car_interfaces__msg__Graph__rosidl_typesupport_introspection_c__resize_function__Graph__edges(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  rosidl_runtime_c__int32__Sequence__fini(member);
  return rosidl_runtime_c__int32__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember car_interfaces__msg__Graph__rosidl_typesupport_introspection_c__Graph_message_member_array[2] = {
  {
    "nodes",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(car_interfaces__msg__Graph, nodes),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "edges",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(car_interfaces__msg__Graph, edges),  // bytes offset in struct
    NULL,  // default value
    car_interfaces__msg__Graph__rosidl_typesupport_introspection_c__size_function__Graph__edges,  // size() function pointer
    car_interfaces__msg__Graph__rosidl_typesupport_introspection_c__get_const_function__Graph__edges,  // get_const(index) function pointer
    car_interfaces__msg__Graph__rosidl_typesupport_introspection_c__get_function__Graph__edges,  // get(index) function pointer
    car_interfaces__msg__Graph__rosidl_typesupport_introspection_c__fetch_function__Graph__edges,  // fetch(index, &value) function pointer
    car_interfaces__msg__Graph__rosidl_typesupport_introspection_c__assign_function__Graph__edges,  // assign(index, value) function pointer
    car_interfaces__msg__Graph__rosidl_typesupport_introspection_c__resize_function__Graph__edges  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers car_interfaces__msg__Graph__rosidl_typesupport_introspection_c__Graph_message_members = {
  "car_interfaces__msg",  // message namespace
  "Graph",  // message name
  2,  // number of fields
  sizeof(car_interfaces__msg__Graph),
  car_interfaces__msg__Graph__rosidl_typesupport_introspection_c__Graph_message_member_array,  // message members
  car_interfaces__msg__Graph__rosidl_typesupport_introspection_c__Graph_init_function,  // function to initialize message memory (memory has to be allocated)
  car_interfaces__msg__Graph__rosidl_typesupport_introspection_c__Graph_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t car_interfaces__msg__Graph__rosidl_typesupport_introspection_c__Graph_message_type_support_handle = {
  0,
  &car_interfaces__msg__Graph__rosidl_typesupport_introspection_c__Graph_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_car_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, car_interfaces, msg, Graph)() {
  car_interfaces__msg__Graph__rosidl_typesupport_introspection_c__Graph_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseArray)();
  if (!car_interfaces__msg__Graph__rosidl_typesupport_introspection_c__Graph_message_type_support_handle.typesupport_identifier) {
    car_interfaces__msg__Graph__rosidl_typesupport_introspection_c__Graph_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &car_interfaces__msg__Graph__rosidl_typesupport_introspection_c__Graph_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
