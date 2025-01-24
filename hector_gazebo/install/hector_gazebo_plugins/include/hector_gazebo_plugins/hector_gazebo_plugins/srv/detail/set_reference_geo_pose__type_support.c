// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from hector_gazebo_plugins:srv/SetReferenceGeoPose.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "hector_gazebo_plugins/srv/detail/set_reference_geo_pose__rosidl_typesupport_introspection_c.h"
#include "hector_gazebo_plugins/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "hector_gazebo_plugins/srv/detail/set_reference_geo_pose__functions.h"
#include "hector_gazebo_plugins/srv/detail/set_reference_geo_pose__struct.h"


// Include directives for member types
// Member `geo_pose`
#include "geographic_msgs/msg/geo_pose.h"
// Member `geo_pose`
#include "geographic_msgs/msg/detail/geo_pose__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__rosidl_typesupport_introspection_c__SetReferenceGeoPose_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__init(message_memory);
}

void hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__rosidl_typesupport_introspection_c__SetReferenceGeoPose_Request_fini_function(void * message_memory)
{
  hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__rosidl_typesupport_introspection_c__SetReferenceGeoPose_Request_message_member_array[1] = {
  {
    "geo_pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hector_gazebo_plugins__srv__SetReferenceGeoPose_Request, geo_pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__rosidl_typesupport_introspection_c__SetReferenceGeoPose_Request_message_members = {
  "hector_gazebo_plugins__srv",  // message namespace
  "SetReferenceGeoPose_Request",  // message name
  1,  // number of fields
  sizeof(hector_gazebo_plugins__srv__SetReferenceGeoPose_Request),
  hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__rosidl_typesupport_introspection_c__SetReferenceGeoPose_Request_message_member_array,  // message members
  hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__rosidl_typesupport_introspection_c__SetReferenceGeoPose_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__rosidl_typesupport_introspection_c__SetReferenceGeoPose_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__rosidl_typesupport_introspection_c__SetReferenceGeoPose_Request_message_type_support_handle = {
  0,
  &hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__rosidl_typesupport_introspection_c__SetReferenceGeoPose_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hector_gazebo_plugins
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hector_gazebo_plugins, srv, SetReferenceGeoPose_Request)() {
  hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__rosidl_typesupport_introspection_c__SetReferenceGeoPose_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geographic_msgs, msg, GeoPose)();
  if (!hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__rosidl_typesupport_introspection_c__SetReferenceGeoPose_Request_message_type_support_handle.typesupport_identifier) {
    hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__rosidl_typesupport_introspection_c__SetReferenceGeoPose_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__rosidl_typesupport_introspection_c__SetReferenceGeoPose_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "hector_gazebo_plugins/srv/detail/set_reference_geo_pose__rosidl_typesupport_introspection_c.h"
// already included above
// #include "hector_gazebo_plugins/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "hector_gazebo_plugins/srv/detail/set_reference_geo_pose__functions.h"
// already included above
// #include "hector_gazebo_plugins/srv/detail/set_reference_geo_pose__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__rosidl_typesupport_introspection_c__SetReferenceGeoPose_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__init(message_memory);
}

void hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__rosidl_typesupport_introspection_c__SetReferenceGeoPose_Response_fini_function(void * message_memory)
{
  hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__rosidl_typesupport_introspection_c__SetReferenceGeoPose_Response_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hector_gazebo_plugins__srv__SetReferenceGeoPose_Response, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__rosidl_typesupport_introspection_c__SetReferenceGeoPose_Response_message_members = {
  "hector_gazebo_plugins__srv",  // message namespace
  "SetReferenceGeoPose_Response",  // message name
  1,  // number of fields
  sizeof(hector_gazebo_plugins__srv__SetReferenceGeoPose_Response),
  hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__rosidl_typesupport_introspection_c__SetReferenceGeoPose_Response_message_member_array,  // message members
  hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__rosidl_typesupport_introspection_c__SetReferenceGeoPose_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__rosidl_typesupport_introspection_c__SetReferenceGeoPose_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__rosidl_typesupport_introspection_c__SetReferenceGeoPose_Response_message_type_support_handle = {
  0,
  &hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__rosidl_typesupport_introspection_c__SetReferenceGeoPose_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hector_gazebo_plugins
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hector_gazebo_plugins, srv, SetReferenceGeoPose_Response)() {
  if (!hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__rosidl_typesupport_introspection_c__SetReferenceGeoPose_Response_message_type_support_handle.typesupport_identifier) {
    hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__rosidl_typesupport_introspection_c__SetReferenceGeoPose_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__rosidl_typesupport_introspection_c__SetReferenceGeoPose_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "hector_gazebo_plugins/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "hector_gazebo_plugins/srv/detail/set_reference_geo_pose__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers hector_gazebo_plugins__srv__detail__set_reference_geo_pose__rosidl_typesupport_introspection_c__SetReferenceGeoPose_service_members = {
  "hector_gazebo_plugins__srv",  // service namespace
  "SetReferenceGeoPose",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // hector_gazebo_plugins__srv__detail__set_reference_geo_pose__rosidl_typesupport_introspection_c__SetReferenceGeoPose_Request_message_type_support_handle,
  NULL  // response message
  // hector_gazebo_plugins__srv__detail__set_reference_geo_pose__rosidl_typesupport_introspection_c__SetReferenceGeoPose_Response_message_type_support_handle
};

static rosidl_service_type_support_t hector_gazebo_plugins__srv__detail__set_reference_geo_pose__rosidl_typesupport_introspection_c__SetReferenceGeoPose_service_type_support_handle = {
  0,
  &hector_gazebo_plugins__srv__detail__set_reference_geo_pose__rosidl_typesupport_introspection_c__SetReferenceGeoPose_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hector_gazebo_plugins, srv, SetReferenceGeoPose_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hector_gazebo_plugins, srv, SetReferenceGeoPose_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hector_gazebo_plugins
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hector_gazebo_plugins, srv, SetReferenceGeoPose)() {
  if (!hector_gazebo_plugins__srv__detail__set_reference_geo_pose__rosidl_typesupport_introspection_c__SetReferenceGeoPose_service_type_support_handle.typesupport_identifier) {
    hector_gazebo_plugins__srv__detail__set_reference_geo_pose__rosidl_typesupport_introspection_c__SetReferenceGeoPose_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)hector_gazebo_plugins__srv__detail__set_reference_geo_pose__rosidl_typesupport_introspection_c__SetReferenceGeoPose_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hector_gazebo_plugins, srv, SetReferenceGeoPose_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hector_gazebo_plugins, srv, SetReferenceGeoPose_Response)()->data;
  }

  return &hector_gazebo_plugins__srv__detail__set_reference_geo_pose__rosidl_typesupport_introspection_c__SetReferenceGeoPose_service_type_support_handle;
}
