// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hector_gazebo_plugins:srv/SetBias.idl
// generated code does not contain a copyright notice

#ifndef HECTOR_GAZEBO_PLUGINS__SRV__DETAIL__SET_BIAS__STRUCT_HPP_
#define HECTOR_GAZEBO_PLUGINS__SRV__DETAIL__SET_BIAS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'bias'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__hector_gazebo_plugins__srv__SetBias_Request __attribute__((deprecated))
#else
# define DEPRECATED__hector_gazebo_plugins__srv__SetBias_Request __declspec(deprecated)
#endif

namespace hector_gazebo_plugins
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetBias_Request_
{
  using Type = SetBias_Request_<ContainerAllocator>;

  explicit SetBias_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : bias(_init)
  {
    (void)_init;
  }

  explicit SetBias_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : bias(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _bias_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _bias_type bias;

  // setters for named parameter idiom
  Type & set__bias(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->bias = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hector_gazebo_plugins::srv::SetBias_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const hector_gazebo_plugins::srv::SetBias_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hector_gazebo_plugins::srv::SetBias_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hector_gazebo_plugins::srv::SetBias_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hector_gazebo_plugins::srv::SetBias_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hector_gazebo_plugins::srv::SetBias_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hector_gazebo_plugins::srv::SetBias_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hector_gazebo_plugins::srv::SetBias_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hector_gazebo_plugins::srv::SetBias_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hector_gazebo_plugins::srv::SetBias_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hector_gazebo_plugins__srv__SetBias_Request
    std::shared_ptr<hector_gazebo_plugins::srv::SetBias_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hector_gazebo_plugins__srv__SetBias_Request
    std::shared_ptr<hector_gazebo_plugins::srv::SetBias_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetBias_Request_ & other) const
  {
    if (this->bias != other.bias) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetBias_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetBias_Request_

// alias to use template instance with default allocator
using SetBias_Request =
  hector_gazebo_plugins::srv::SetBias_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace hector_gazebo_plugins


#ifndef _WIN32
# define DEPRECATED__hector_gazebo_plugins__srv__SetBias_Response __attribute__((deprecated))
#else
# define DEPRECATED__hector_gazebo_plugins__srv__SetBias_Response __declspec(deprecated)
#endif

namespace hector_gazebo_plugins
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetBias_Response_
{
  using Type = SetBias_Response_<ContainerAllocator>;

  explicit SetBias_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit SetBias_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    hector_gazebo_plugins::srv::SetBias_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const hector_gazebo_plugins::srv::SetBias_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hector_gazebo_plugins::srv::SetBias_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hector_gazebo_plugins::srv::SetBias_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hector_gazebo_plugins::srv::SetBias_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hector_gazebo_plugins::srv::SetBias_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hector_gazebo_plugins::srv::SetBias_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hector_gazebo_plugins::srv::SetBias_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hector_gazebo_plugins::srv::SetBias_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hector_gazebo_plugins::srv::SetBias_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hector_gazebo_plugins__srv__SetBias_Response
    std::shared_ptr<hector_gazebo_plugins::srv::SetBias_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hector_gazebo_plugins__srv__SetBias_Response
    std::shared_ptr<hector_gazebo_plugins::srv::SetBias_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetBias_Response_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetBias_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetBias_Response_

// alias to use template instance with default allocator
using SetBias_Response =
  hector_gazebo_plugins::srv::SetBias_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace hector_gazebo_plugins

namespace hector_gazebo_plugins
{

namespace srv
{

struct SetBias
{
  using Request = hector_gazebo_plugins::srv::SetBias_Request;
  using Response = hector_gazebo_plugins::srv::SetBias_Response;
};

}  // namespace srv

}  // namespace hector_gazebo_plugins

#endif  // HECTOR_GAZEBO_PLUGINS__SRV__DETAIL__SET_BIAS__STRUCT_HPP_
