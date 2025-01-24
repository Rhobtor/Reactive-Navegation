// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hector_gazebo_plugins:srv/SetBias.idl
// generated code does not contain a copyright notice

#ifndef HECTOR_GAZEBO_PLUGINS__SRV__DETAIL__SET_BIAS__BUILDER_HPP_
#define HECTOR_GAZEBO_PLUGINS__SRV__DETAIL__SET_BIAS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "hector_gazebo_plugins/srv/detail/set_bias__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace hector_gazebo_plugins
{

namespace srv
{

namespace builder
{

class Init_SetBias_Request_bias
{
public:
  Init_SetBias_Request_bias()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::hector_gazebo_plugins::srv::SetBias_Request bias(::hector_gazebo_plugins::srv::SetBias_Request::_bias_type arg)
  {
    msg_.bias = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hector_gazebo_plugins::srv::SetBias_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::hector_gazebo_plugins::srv::SetBias_Request>()
{
  return hector_gazebo_plugins::srv::builder::Init_SetBias_Request_bias();
}

}  // namespace hector_gazebo_plugins


namespace hector_gazebo_plugins
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::hector_gazebo_plugins::srv::SetBias_Response>()
{
  return ::hector_gazebo_plugins::srv::SetBias_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace hector_gazebo_plugins

#endif  // HECTOR_GAZEBO_PLUGINS__SRV__DETAIL__SET_BIAS__BUILDER_HPP_
