// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hector_gazebo_plugins:srv/SetReferenceGeoPose.idl
// generated code does not contain a copyright notice

#ifndef HECTOR_GAZEBO_PLUGINS__SRV__DETAIL__SET_REFERENCE_GEO_POSE__BUILDER_HPP_
#define HECTOR_GAZEBO_PLUGINS__SRV__DETAIL__SET_REFERENCE_GEO_POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "hector_gazebo_plugins/srv/detail/set_reference_geo_pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace hector_gazebo_plugins
{

namespace srv
{

namespace builder
{

class Init_SetReferenceGeoPose_Request_geo_pose
{
public:
  Init_SetReferenceGeoPose_Request_geo_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::hector_gazebo_plugins::srv::SetReferenceGeoPose_Request geo_pose(::hector_gazebo_plugins::srv::SetReferenceGeoPose_Request::_geo_pose_type arg)
  {
    msg_.geo_pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hector_gazebo_plugins::srv::SetReferenceGeoPose_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::hector_gazebo_plugins::srv::SetReferenceGeoPose_Request>()
{
  return hector_gazebo_plugins::srv::builder::Init_SetReferenceGeoPose_Request_geo_pose();
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
auto build<::hector_gazebo_plugins::srv::SetReferenceGeoPose_Response>()
{
  return ::hector_gazebo_plugins::srv::SetReferenceGeoPose_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace hector_gazebo_plugins

#endif  // HECTOR_GAZEBO_PLUGINS__SRV__DETAIL__SET_REFERENCE_GEO_POSE__BUILDER_HPP_
