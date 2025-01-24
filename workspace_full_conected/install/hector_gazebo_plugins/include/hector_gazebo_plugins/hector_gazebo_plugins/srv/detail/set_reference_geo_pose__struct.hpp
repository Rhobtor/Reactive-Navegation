// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hector_gazebo_plugins:srv/SetReferenceGeoPose.idl
// generated code does not contain a copyright notice

#ifndef HECTOR_GAZEBO_PLUGINS__SRV__DETAIL__SET_REFERENCE_GEO_POSE__STRUCT_HPP_
#define HECTOR_GAZEBO_PLUGINS__SRV__DETAIL__SET_REFERENCE_GEO_POSE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'geo_pose'
#include "geographic_msgs/msg/detail/geo_pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__hector_gazebo_plugins__srv__SetReferenceGeoPose_Request __attribute__((deprecated))
#else
# define DEPRECATED__hector_gazebo_plugins__srv__SetReferenceGeoPose_Request __declspec(deprecated)
#endif

namespace hector_gazebo_plugins
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetReferenceGeoPose_Request_
{
  using Type = SetReferenceGeoPose_Request_<ContainerAllocator>;

  explicit SetReferenceGeoPose_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : geo_pose(_init)
  {
    (void)_init;
  }

  explicit SetReferenceGeoPose_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : geo_pose(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _geo_pose_type =
    geographic_msgs::msg::GeoPose_<ContainerAllocator>;
  _geo_pose_type geo_pose;

  // setters for named parameter idiom
  Type & set__geo_pose(
    const geographic_msgs::msg::GeoPose_<ContainerAllocator> & _arg)
  {
    this->geo_pose = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hector_gazebo_plugins::srv::SetReferenceGeoPose_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const hector_gazebo_plugins::srv::SetReferenceGeoPose_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hector_gazebo_plugins::srv::SetReferenceGeoPose_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hector_gazebo_plugins::srv::SetReferenceGeoPose_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hector_gazebo_plugins::srv::SetReferenceGeoPose_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hector_gazebo_plugins::srv::SetReferenceGeoPose_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hector_gazebo_plugins::srv::SetReferenceGeoPose_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hector_gazebo_plugins::srv::SetReferenceGeoPose_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hector_gazebo_plugins::srv::SetReferenceGeoPose_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hector_gazebo_plugins::srv::SetReferenceGeoPose_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hector_gazebo_plugins__srv__SetReferenceGeoPose_Request
    std::shared_ptr<hector_gazebo_plugins::srv::SetReferenceGeoPose_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hector_gazebo_plugins__srv__SetReferenceGeoPose_Request
    std::shared_ptr<hector_gazebo_plugins::srv::SetReferenceGeoPose_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetReferenceGeoPose_Request_ & other) const
  {
    if (this->geo_pose != other.geo_pose) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetReferenceGeoPose_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetReferenceGeoPose_Request_

// alias to use template instance with default allocator
using SetReferenceGeoPose_Request =
  hector_gazebo_plugins::srv::SetReferenceGeoPose_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace hector_gazebo_plugins


#ifndef _WIN32
# define DEPRECATED__hector_gazebo_plugins__srv__SetReferenceGeoPose_Response __attribute__((deprecated))
#else
# define DEPRECATED__hector_gazebo_plugins__srv__SetReferenceGeoPose_Response __declspec(deprecated)
#endif

namespace hector_gazebo_plugins
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetReferenceGeoPose_Response_
{
  using Type = SetReferenceGeoPose_Response_<ContainerAllocator>;

  explicit SetReferenceGeoPose_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit SetReferenceGeoPose_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    hector_gazebo_plugins::srv::SetReferenceGeoPose_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const hector_gazebo_plugins::srv::SetReferenceGeoPose_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hector_gazebo_plugins::srv::SetReferenceGeoPose_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hector_gazebo_plugins::srv::SetReferenceGeoPose_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hector_gazebo_plugins::srv::SetReferenceGeoPose_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hector_gazebo_plugins::srv::SetReferenceGeoPose_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hector_gazebo_plugins::srv::SetReferenceGeoPose_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hector_gazebo_plugins::srv::SetReferenceGeoPose_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hector_gazebo_plugins::srv::SetReferenceGeoPose_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hector_gazebo_plugins::srv::SetReferenceGeoPose_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hector_gazebo_plugins__srv__SetReferenceGeoPose_Response
    std::shared_ptr<hector_gazebo_plugins::srv::SetReferenceGeoPose_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hector_gazebo_plugins__srv__SetReferenceGeoPose_Response
    std::shared_ptr<hector_gazebo_plugins::srv::SetReferenceGeoPose_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetReferenceGeoPose_Response_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetReferenceGeoPose_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetReferenceGeoPose_Response_

// alias to use template instance with default allocator
using SetReferenceGeoPose_Response =
  hector_gazebo_plugins::srv::SetReferenceGeoPose_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace hector_gazebo_plugins

namespace hector_gazebo_plugins
{

namespace srv
{

struct SetReferenceGeoPose
{
  using Request = hector_gazebo_plugins::srv::SetReferenceGeoPose_Request;
  using Response = hector_gazebo_plugins::srv::SetReferenceGeoPose_Response;
};

}  // namespace srv

}  // namespace hector_gazebo_plugins

#endif  // HECTOR_GAZEBO_PLUGINS__SRV__DETAIL__SET_REFERENCE_GEO_POSE__STRUCT_HPP_
