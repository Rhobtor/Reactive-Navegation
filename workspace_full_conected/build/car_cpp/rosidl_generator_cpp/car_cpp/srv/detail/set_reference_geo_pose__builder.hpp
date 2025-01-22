// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from car_cpp:srv/SetReferenceGeoPose.idl
// generated code does not contain a copyright notice

#ifndef CAR_CPP__SRV__DETAIL__SET_REFERENCE_GEO_POSE__BUILDER_HPP_
#define CAR_CPP__SRV__DETAIL__SET_REFERENCE_GEO_POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "car_cpp/srv/detail/set_reference_geo_pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace car_cpp
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
  ::car_cpp::srv::SetReferenceGeoPose_Request geo_pose(::car_cpp::srv::SetReferenceGeoPose_Request::_geo_pose_type arg)
  {
    msg_.geo_pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::car_cpp::srv::SetReferenceGeoPose_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::car_cpp::srv::SetReferenceGeoPose_Request>()
{
  return car_cpp::srv::builder::Init_SetReferenceGeoPose_Request_geo_pose();
}

}  // namespace car_cpp


namespace car_cpp
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::car_cpp::srv::SetReferenceGeoPose_Response>()
{
  return ::car_cpp::srv::SetReferenceGeoPose_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace car_cpp

#endif  // CAR_CPP__SRV__DETAIL__SET_REFERENCE_GEO_POSE__BUILDER_HPP_
