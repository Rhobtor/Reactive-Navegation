// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hector_gazebo_plugins:srv/SetBias.idl
// generated code does not contain a copyright notice

#ifndef HECTOR_GAZEBO_PLUGINS__SRV__DETAIL__SET_BIAS__TRAITS_HPP_
#define HECTOR_GAZEBO_PLUGINS__SRV__DETAIL__SET_BIAS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "hector_gazebo_plugins/srv/detail/set_bias__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'bias'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace hector_gazebo_plugins
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetBias_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: bias
  {
    out << "bias: ";
    to_flow_style_yaml(msg.bias, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetBias_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: bias
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bias:\n";
    to_block_style_yaml(msg.bias, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetBias_Request & msg, bool use_flow_style = false)
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
  const hector_gazebo_plugins::srv::SetBias_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  hector_gazebo_plugins::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use hector_gazebo_plugins::srv::to_yaml() instead")]]
inline std::string to_yaml(const hector_gazebo_plugins::srv::SetBias_Request & msg)
{
  return hector_gazebo_plugins::srv::to_yaml(msg);
}

template<>
inline const char * data_type<hector_gazebo_plugins::srv::SetBias_Request>()
{
  return "hector_gazebo_plugins::srv::SetBias_Request";
}

template<>
inline const char * name<hector_gazebo_plugins::srv::SetBias_Request>()
{
  return "hector_gazebo_plugins/srv/SetBias_Request";
}

template<>
struct has_fixed_size<hector_gazebo_plugins::srv::SetBias_Request>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Vector3>::value> {};

template<>
struct has_bounded_size<hector_gazebo_plugins::srv::SetBias_Request>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Vector3>::value> {};

template<>
struct is_message<hector_gazebo_plugins::srv::SetBias_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace hector_gazebo_plugins
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetBias_Response & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetBias_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetBias_Response & msg, bool use_flow_style = false)
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
  const hector_gazebo_plugins::srv::SetBias_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  hector_gazebo_plugins::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use hector_gazebo_plugins::srv::to_yaml() instead")]]
inline std::string to_yaml(const hector_gazebo_plugins::srv::SetBias_Response & msg)
{
  return hector_gazebo_plugins::srv::to_yaml(msg);
}

template<>
inline const char * data_type<hector_gazebo_plugins::srv::SetBias_Response>()
{
  return "hector_gazebo_plugins::srv::SetBias_Response";
}

template<>
inline const char * name<hector_gazebo_plugins::srv::SetBias_Response>()
{
  return "hector_gazebo_plugins/srv/SetBias_Response";
}

template<>
struct has_fixed_size<hector_gazebo_plugins::srv::SetBias_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<hector_gazebo_plugins::srv::SetBias_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<hector_gazebo_plugins::srv::SetBias_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<hector_gazebo_plugins::srv::SetBias>()
{
  return "hector_gazebo_plugins::srv::SetBias";
}

template<>
inline const char * name<hector_gazebo_plugins::srv::SetBias>()
{
  return "hector_gazebo_plugins/srv/SetBias";
}

template<>
struct has_fixed_size<hector_gazebo_plugins::srv::SetBias>
  : std::integral_constant<
    bool,
    has_fixed_size<hector_gazebo_plugins::srv::SetBias_Request>::value &&
    has_fixed_size<hector_gazebo_plugins::srv::SetBias_Response>::value
  >
{
};

template<>
struct has_bounded_size<hector_gazebo_plugins::srv::SetBias>
  : std::integral_constant<
    bool,
    has_bounded_size<hector_gazebo_plugins::srv::SetBias_Request>::value &&
    has_bounded_size<hector_gazebo_plugins::srv::SetBias_Response>::value
  >
{
};

template<>
struct is_service<hector_gazebo_plugins::srv::SetBias>
  : std::true_type
{
};

template<>
struct is_service_request<hector_gazebo_plugins::srv::SetBias_Request>
  : std::true_type
{
};

template<>
struct is_service_response<hector_gazebo_plugins::srv::SetBias_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // HECTOR_GAZEBO_PLUGINS__SRV__DETAIL__SET_BIAS__TRAITS_HPP_
