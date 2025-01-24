// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hector_gazebo_plugins:srv/SetReferenceGeoPose.idl
// generated code does not contain a copyright notice

#ifndef HECTOR_GAZEBO_PLUGINS__SRV__DETAIL__SET_REFERENCE_GEO_POSE__TRAITS_HPP_
#define HECTOR_GAZEBO_PLUGINS__SRV__DETAIL__SET_REFERENCE_GEO_POSE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "hector_gazebo_plugins/srv/detail/set_reference_geo_pose__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'geo_pose'
#include "geographic_msgs/msg/detail/geo_pose__traits.hpp"

namespace hector_gazebo_plugins
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetReferenceGeoPose_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: geo_pose
  {
    out << "geo_pose: ";
    to_flow_style_yaml(msg.geo_pose, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetReferenceGeoPose_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: geo_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "geo_pose:\n";
    to_block_style_yaml(msg.geo_pose, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetReferenceGeoPose_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace hector_gazebo_plugins

namespace rosidl_generator_traits
{

[[deprecated("use hector_gazebo_plugins::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const hector_gazebo_plugins::srv::SetReferenceGeoPose_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  hector_gazebo_plugins::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use hector_gazebo_plugins::srv::to_yaml() instead")]]
inline std::string to_yaml(const hector_gazebo_plugins::srv::SetReferenceGeoPose_Request & msg)
{
  return hector_gazebo_plugins::srv::to_yaml(msg);
}

template<>
inline const char * data_type<hector_gazebo_plugins::srv::SetReferenceGeoPose_Request>()
{
  return "hector_gazebo_plugins::srv::SetReferenceGeoPose_Request";
}

template<>
inline const char * name<hector_gazebo_plugins::srv::SetReferenceGeoPose_Request>()
{
  return "hector_gazebo_plugins/srv/SetReferenceGeoPose_Request";
}

template<>
struct has_fixed_size<hector_gazebo_plugins::srv::SetReferenceGeoPose_Request>
  : std::integral_constant<bool, has_fixed_size<geographic_msgs::msg::GeoPose>::value> {};

template<>
struct has_bounded_size<hector_gazebo_plugins::srv::SetReferenceGeoPose_Request>
  : std::integral_constant<bool, has_bounded_size<geographic_msgs::msg::GeoPose>::value> {};

template<>
struct is_message<hector_gazebo_plugins::srv::SetReferenceGeoPose_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace hector_gazebo_plugins
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetReferenceGeoPose_Response & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetReferenceGeoPose_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetReferenceGeoPose_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace hector_gazebo_plugins

namespace rosidl_generator_traits
{

[[deprecated("use hector_gazebo_plugins::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const hector_gazebo_plugins::srv::SetReferenceGeoPose_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  hector_gazebo_plugins::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use hector_gazebo_plugins::srv::to_yaml() instead")]]
inline std::string to_yaml(const hector_gazebo_plugins::srv::SetReferenceGeoPose_Response & msg)
{
  return hector_gazebo_plugins::srv::to_yaml(msg);
}

template<>
inline const char * data_type<hector_gazebo_plugins::srv::SetReferenceGeoPose_Response>()
{
  return "hector_gazebo_plugins::srv::SetReferenceGeoPose_Response";
}

template<>
inline const char * name<hector_gazebo_plugins::srv::SetReferenceGeoPose_Response>()
{
  return "hector_gazebo_plugins/srv/SetReferenceGeoPose_Response";
}

template<>
struct has_fixed_size<hector_gazebo_plugins::srv::SetReferenceGeoPose_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<hector_gazebo_plugins::srv::SetReferenceGeoPose_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<hector_gazebo_plugins::srv::SetReferenceGeoPose_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<hector_gazebo_plugins::srv::SetReferenceGeoPose>()
{
  return "hector_gazebo_plugins::srv::SetReferenceGeoPose";
}

template<>
inline const char * name<hector_gazebo_plugins::srv::SetReferenceGeoPose>()
{
  return "hector_gazebo_plugins/srv/SetReferenceGeoPose";
}

template<>
struct has_fixed_size<hector_gazebo_plugins::srv::SetReferenceGeoPose>
  : std::integral_constant<
    bool,
    has_fixed_size<hector_gazebo_plugins::srv::SetReferenceGeoPose_Request>::value &&
    has_fixed_size<hector_gazebo_plugins::srv::SetReferenceGeoPose_Response>::value
  >
{
};

template<>
struct has_bounded_size<hector_gazebo_plugins::srv::SetReferenceGeoPose>
  : std::integral_constant<
    bool,
    has_bounded_size<hector_gazebo_plugins::srv::SetReferenceGeoPose_Request>::value &&
    has_bounded_size<hector_gazebo_plugins::srv::SetReferenceGeoPose_Response>::value
  >
{
};

template<>
struct is_service<hector_gazebo_plugins::srv::SetReferenceGeoPose>
  : std::true_type
{
};

template<>
struct is_service_request<hector_gazebo_plugins::srv::SetReferenceGeoPose_Request>
  : std::true_type
{
};

template<>
struct is_service_response<hector_gazebo_plugins::srv::SetReferenceGeoPose_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // HECTOR_GAZEBO_PLUGINS__SRV__DETAIL__SET_REFERENCE_GEO_POSE__TRAITS_HPP_
