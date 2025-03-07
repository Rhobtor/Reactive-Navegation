// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from car_interfaces:msg/Graph.idl
// generated code does not contain a copyright notice

#ifndef CAR_INTERFACES__MSG__DETAIL__GRAPH__BUILDER_HPP_
#define CAR_INTERFACES__MSG__DETAIL__GRAPH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "car_interfaces/msg/detail/graph__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace car_interfaces
{

namespace msg
{

namespace builder
{

class Init_Graph_edges
{
public:
  explicit Init_Graph_edges(::car_interfaces::msg::Graph & msg)
  : msg_(msg)
  {}
  ::car_interfaces::msg::Graph edges(::car_interfaces::msg::Graph::_edges_type arg)
  {
    msg_.edges = std::move(arg);
    return std::move(msg_);
  }

private:
  ::car_interfaces::msg::Graph msg_;
};

class Init_Graph_nodes
{
public:
  Init_Graph_nodes()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Graph_edges nodes(::car_interfaces::msg::Graph::_nodes_type arg)
  {
    msg_.nodes = std::move(arg);
    return Init_Graph_edges(msg_);
  }

private:
  ::car_interfaces::msg::Graph msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::car_interfaces::msg::Graph>()
{
  return car_interfaces::msg::builder::Init_Graph_nodes();
}

}  // namespace car_interfaces

#endif  // CAR_INTERFACES__MSG__DETAIL__GRAPH__BUILDER_HPP_
