// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from car_cpp:srv/SetBias.idl
// generated code does not contain a copyright notice

#ifndef CAR_CPP__SRV__DETAIL__SET_BIAS__BUILDER_HPP_
#define CAR_CPP__SRV__DETAIL__SET_BIAS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "car_cpp/srv/detail/set_bias__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace car_cpp
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
  ::car_cpp::srv::SetBias_Request bias(::car_cpp::srv::SetBias_Request::_bias_type arg)
  {
    msg_.bias = std::move(arg);
    return std::move(msg_);
  }

private:
  ::car_cpp::srv::SetBias_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::car_cpp::srv::SetBias_Request>()
{
  return car_cpp::srv::builder::Init_SetBias_Request_bias();
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
auto build<::car_cpp::srv::SetBias_Response>()
{
  return ::car_cpp::srv::SetBias_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace car_cpp

#endif  // CAR_CPP__SRV__DETAIL__SET_BIAS__BUILDER_HPP_
