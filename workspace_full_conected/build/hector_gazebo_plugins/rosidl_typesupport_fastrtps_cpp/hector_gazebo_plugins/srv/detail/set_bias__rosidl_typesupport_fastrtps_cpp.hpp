// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from hector_gazebo_plugins:srv/SetBias.idl
// generated code does not contain a copyright notice

#ifndef HECTOR_GAZEBO_PLUGINS__SRV__DETAIL__SET_BIAS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define HECTOR_GAZEBO_PLUGINS__SRV__DETAIL__SET_BIAS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "hector_gazebo_plugins/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "hector_gazebo_plugins/srv/detail/set_bias__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace hector_gazebo_plugins
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hector_gazebo_plugins
cdr_serialize(
  const hector_gazebo_plugins::srv::SetBias_Request & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hector_gazebo_plugins
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  hector_gazebo_plugins::srv::SetBias_Request & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hector_gazebo_plugins
get_serialized_size(
  const hector_gazebo_plugins::srv::SetBias_Request & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hector_gazebo_plugins
max_serialized_size_SetBias_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace hector_gazebo_plugins

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hector_gazebo_plugins
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, hector_gazebo_plugins, srv, SetBias_Request)();

#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "hector_gazebo_plugins/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
// already included above
// #include "hector_gazebo_plugins/srv/detail/set_bias__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// already included above
// #include "fastcdr/Cdr.h"

namespace hector_gazebo_plugins
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hector_gazebo_plugins
cdr_serialize(
  const hector_gazebo_plugins::srv::SetBias_Response & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hector_gazebo_plugins
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  hector_gazebo_plugins::srv::SetBias_Response & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hector_gazebo_plugins
get_serialized_size(
  const hector_gazebo_plugins::srv::SetBias_Response & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hector_gazebo_plugins
max_serialized_size_SetBias_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace hector_gazebo_plugins

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hector_gazebo_plugins
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, hector_gazebo_plugins, srv, SetBias_Response)();

#ifdef __cplusplus
}
#endif

#include "rmw/types.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "hector_gazebo_plugins/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hector_gazebo_plugins
const rosidl_service_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, hector_gazebo_plugins, srv, SetBias)();

#ifdef __cplusplus
}
#endif

#endif  // HECTOR_GAZEBO_PLUGINS__SRV__DETAIL__SET_BIAS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
