// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from car_cpp:srv/SetReferenceGeoPose.idl
// generated code does not contain a copyright notice

#ifndef CAR_CPP__SRV__DETAIL__SET_REFERENCE_GEO_POSE__TRAITS_HPP_
#define CAR_CPP__SRV__DETAIL__SET_REFERENCE_GEO_POSE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "car_cpp/srv/detail/set_reference_geo_pose__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'geo_pose'
#include "geographic_msgs/msg/detail/geo_pose__traits.hpp"

namespace car_cpp
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

}  // namespace car_cpp

namespace rosidl_generator_traits
{

[[deprecated("use car_cpp::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const car_cpp::srv::SetReferenceGeoPose_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  car_cpp::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use car_cpp::srv::to_yaml() instead")]]
inline std::string to_yaml(const car_cpp::srv::SetReferenceGeoPose_Request & msg)
{
  return car_cpp::srv::to_yaml(msg);
}

template<>
inline const char * data_type<car_cpp::srv::SetReferenceGeoPose_Request>()
{
  return "car_cpp::srv::SetReferenceGeoPose_Request";
}

template<>
inline const char * name<car_cpp::srv::SetReferenceGeoPose_Request>()
{
  return "car_cpp/srv/SetReferenceGeoPose_Request";
}

template<>
struct has_fixed_size<car_cpp::srv::SetReferenceGeoPose_Request>
  : std::integral_constant<bool, has_fixed_size<geographic_msgs::msg::GeoPose>::value> {};

template<>
struct has_bounded_size<car_cpp::srv::SetReferenceGeoPose_Request>
  : std::integral_constant<bool, has_bounded_size<geographic_msgs::msg::GeoPose>::value> {};

template<>
struct is_message<car_cpp::srv::SetReferenceGeoPose_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace car_cpp
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

}  // namespace car_cpp

namespace rosidl_generator_traits
{

[[deprecated("use car_cpp::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const car_cpp::srv::SetReferenceGeoPose_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  car_cpp::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use car_cpp::srv::to_yaml() instead")]]
inline std::string to_yaml(const car_cpp::srv::SetReferenceGeoPose_Response & msg)
{
  return car_cpp::srv::to_yaml(msg);
}

template<>
inline const char * data_type<car_cpp::srv::SetReferenceGeoPose_Response>()
{
  return "car_cpp::srv::SetReferenceGeoPose_Response";
}

template<>
inline const char * name<car_cpp::srv::SetReferenceGeoPose_Response>()
{
  return "car_cpp/srv/SetReferenceGeoPose_Response";
}

template<>
struct has_fixed_size<car_cpp::srv::SetReferenceGeoPose_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<car_cpp::srv::SetReferenceGeoPose_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<car_cpp::srv::SetReferenceGeoPose_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<car_cpp::srv::SetReferenceGeoPose>()
{
  return "car_cpp::srv::SetReferenceGeoPose";
}

template<>
inline const char * name<car_cpp::srv::SetReferenceGeoPose>()
{
  return "car_cpp/srv/SetReferenceGeoPose";
}

template<>
struct has_fixed_size<car_cpp::srv::SetReferenceGeoPose>
  : std::integral_constant<
    bool,
    has_fixed_size<car_cpp::srv::SetReferenceGeoPose_Request>::value &&
    has_fixed_size<car_cpp::srv::SetReferenceGeoPose_Response>::value
  >
{
};

template<>
struct has_bounded_size<car_cpp::srv::SetReferenceGeoPose>
  : std::integral_constant<
    bool,
    has_bounded_size<car_cpp::srv::SetReferenceGeoPose_Request>::value &&
    has_bounded_size<car_cpp::srv::SetReferenceGeoPose_Response>::value
  >
{
};

template<>
struct is_service<car_cpp::srv::SetReferenceGeoPose>
  : std::true_type
{
};

template<>
struct is_service_request<car_cpp::srv::SetReferenceGeoPose_Request>
  : std::true_type
{
};

template<>
struct is_service_response<car_cpp::srv::SetReferenceGeoPose_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CAR_CPP__SRV__DETAIL__SET_REFERENCE_GEO_POSE__TRAITS_HPP_
