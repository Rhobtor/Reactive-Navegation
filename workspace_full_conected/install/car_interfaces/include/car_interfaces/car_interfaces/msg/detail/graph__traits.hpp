// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from car_interfaces:msg/Graph.idl
// generated code does not contain a copyright notice

#ifndef CAR_INTERFACES__MSG__DETAIL__GRAPH__TRAITS_HPP_
#define CAR_INTERFACES__MSG__DETAIL__GRAPH__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "car_interfaces/msg/detail/graph__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'nodes'
#include "geometry_msgs/msg/detail/pose_array__traits.hpp"

namespace car_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Graph & msg,
  std::ostream & out)
{
  out << "{";
  // member: nodes
  {
    out << "nodes: ";
    to_flow_style_yaml(msg.nodes, out);
    out << ", ";
  }

  // member: edges
  {
    if (msg.edges.size() == 0) {
      out << "edges: []";
    } else {
      out << "edges: [";
      size_t pending_items = msg.edges.size();
      for (auto item : msg.edges) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Graph & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: nodes
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "nodes:\n";
    to_block_style_yaml(msg.nodes, out, indentation + 2);
  }

  // member: edges
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.edges.size() == 0) {
      out << "edges: []\n";
    } else {
      out << "edges:\n";
      for (auto item : msg.edges) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Graph & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace car_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use car_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const car_interfaces::msg::Graph & msg,
  std::ostream & out, size_t indentation = 0)
{
  car_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use car_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const car_interfaces::msg::Graph & msg)
{
  return car_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<car_interfaces::msg::Graph>()
{
  return "car_interfaces::msg::Graph";
}

template<>
inline const char * name<car_interfaces::msg::Graph>()
{
  return "car_interfaces/msg/Graph";
}

template<>
struct has_fixed_size<car_interfaces::msg::Graph>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<car_interfaces::msg::Graph>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<car_interfaces::msg::Graph>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CAR_INTERFACES__MSG__DETAIL__GRAPH__TRAITS_HPP_
