// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from hector_gazebo_plugins:srv/SetReferenceGeoPose.idl
// generated code does not contain a copyright notice
#include "hector_gazebo_plugins/srv/detail/set_reference_geo_pose__rosidl_typesupport_fastrtps_cpp.hpp"
#include "hector_gazebo_plugins/srv/detail/set_reference_geo_pose__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace geographic_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const geographic_msgs::msg::GeoPose &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  geographic_msgs::msg::GeoPose &);
size_t get_serialized_size(
  const geographic_msgs::msg::GeoPose &,
  size_t current_alignment);
size_t
max_serialized_size_GeoPose(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace geographic_msgs


namespace hector_gazebo_plugins
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hector_gazebo_plugins
cdr_serialize(
  const hector_gazebo_plugins::srv::SetReferenceGeoPose_Request & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: geo_pose
  geographic_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.geo_pose,
    cdr);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hector_gazebo_plugins
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  hector_gazebo_plugins::srv::SetReferenceGeoPose_Request & ros_message)
{
  // Member: geo_pose
  geographic_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.geo_pose);

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hector_gazebo_plugins
get_serialized_size(
  const hector_gazebo_plugins::srv::SetReferenceGeoPose_Request & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: geo_pose

  current_alignment +=
    geographic_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.geo_pose, current_alignment);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hector_gazebo_plugins
max_serialized_size_SetReferenceGeoPose_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: geo_pose
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        geographic_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_GeoPose(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = hector_gazebo_plugins::srv::SetReferenceGeoPose_Request;
    is_plain =
      (
      offsetof(DataType, geo_pose) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _SetReferenceGeoPose_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const hector_gazebo_plugins::srv::SetReferenceGeoPose_Request *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _SetReferenceGeoPose_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<hector_gazebo_plugins::srv::SetReferenceGeoPose_Request *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _SetReferenceGeoPose_Request__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const hector_gazebo_plugins::srv::SetReferenceGeoPose_Request *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _SetReferenceGeoPose_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_SetReferenceGeoPose_Request(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _SetReferenceGeoPose_Request__callbacks = {
  "hector_gazebo_plugins::srv",
  "SetReferenceGeoPose_Request",
  _SetReferenceGeoPose_Request__cdr_serialize,
  _SetReferenceGeoPose_Request__cdr_deserialize,
  _SetReferenceGeoPose_Request__get_serialized_size,
  _SetReferenceGeoPose_Request__max_serialized_size
};

static rosidl_message_type_support_t _SetReferenceGeoPose_Request__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_SetReferenceGeoPose_Request__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace hector_gazebo_plugins

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_hector_gazebo_plugins
const rosidl_message_type_support_t *
get_message_type_support_handle<hector_gazebo_plugins::srv::SetReferenceGeoPose_Request>()
{
  return &hector_gazebo_plugins::srv::typesupport_fastrtps_cpp::_SetReferenceGeoPose_Request__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, hector_gazebo_plugins, srv, SetReferenceGeoPose_Request)() {
  return &hector_gazebo_plugins::srv::typesupport_fastrtps_cpp::_SetReferenceGeoPose_Request__handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include <limits>
// already included above
// #include <stdexcept>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
// already included above
// #include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace hector_gazebo_plugins
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hector_gazebo_plugins
cdr_serialize(
  const hector_gazebo_plugins::srv::SetReferenceGeoPose_Response & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: structure_needs_at_least_one_member
  cdr << ros_message.structure_needs_at_least_one_member;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hector_gazebo_plugins
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  hector_gazebo_plugins::srv::SetReferenceGeoPose_Response & ros_message)
{
  // Member: structure_needs_at_least_one_member
  cdr >> ros_message.structure_needs_at_least_one_member;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hector_gazebo_plugins
get_serialized_size(
  const hector_gazebo_plugins::srv::SetReferenceGeoPose_Response & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: structure_needs_at_least_one_member
  {
    size_t item_size = sizeof(ros_message.structure_needs_at_least_one_member);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hector_gazebo_plugins
max_serialized_size_SetReferenceGeoPose_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: structure_needs_at_least_one_member
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = hector_gazebo_plugins::srv::SetReferenceGeoPose_Response;
    is_plain =
      (
      offsetof(DataType, structure_needs_at_least_one_member) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _SetReferenceGeoPose_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const hector_gazebo_plugins::srv::SetReferenceGeoPose_Response *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _SetReferenceGeoPose_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<hector_gazebo_plugins::srv::SetReferenceGeoPose_Response *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _SetReferenceGeoPose_Response__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const hector_gazebo_plugins::srv::SetReferenceGeoPose_Response *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _SetReferenceGeoPose_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_SetReferenceGeoPose_Response(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _SetReferenceGeoPose_Response__callbacks = {
  "hector_gazebo_plugins::srv",
  "SetReferenceGeoPose_Response",
  _SetReferenceGeoPose_Response__cdr_serialize,
  _SetReferenceGeoPose_Response__cdr_deserialize,
  _SetReferenceGeoPose_Response__get_serialized_size,
  _SetReferenceGeoPose_Response__max_serialized_size
};

static rosidl_message_type_support_t _SetReferenceGeoPose_Response__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_SetReferenceGeoPose_Response__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace hector_gazebo_plugins

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_hector_gazebo_plugins
const rosidl_message_type_support_t *
get_message_type_support_handle<hector_gazebo_plugins::srv::SetReferenceGeoPose_Response>()
{
  return &hector_gazebo_plugins::srv::typesupport_fastrtps_cpp::_SetReferenceGeoPose_Response__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, hector_gazebo_plugins, srv, SetReferenceGeoPose_Response)() {
  return &hector_gazebo_plugins::srv::typesupport_fastrtps_cpp::_SetReferenceGeoPose_Response__handle;
}

#ifdef __cplusplus
}
#endif

#include "rmw/error_handling.h"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/service_type_support_decl.hpp"

namespace hector_gazebo_plugins
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

static service_type_support_callbacks_t _SetReferenceGeoPose__callbacks = {
  "hector_gazebo_plugins::srv",
  "SetReferenceGeoPose",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, hector_gazebo_plugins, srv, SetReferenceGeoPose_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, hector_gazebo_plugins, srv, SetReferenceGeoPose_Response)(),
};

static rosidl_service_type_support_t _SetReferenceGeoPose__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_SetReferenceGeoPose__callbacks,
  get_service_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace hector_gazebo_plugins

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_hector_gazebo_plugins
const rosidl_service_type_support_t *
get_service_type_support_handle<hector_gazebo_plugins::srv::SetReferenceGeoPose>()
{
  return &hector_gazebo_plugins::srv::typesupport_fastrtps_cpp::_SetReferenceGeoPose__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, hector_gazebo_plugins, srv, SetReferenceGeoPose)() {
  return &hector_gazebo_plugins::srv::typesupport_fastrtps_cpp::_SetReferenceGeoPose__handle;
}

#ifdef __cplusplus
}
#endif
