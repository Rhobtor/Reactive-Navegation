// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from car_interfaces:msg/Graph.idl
// generated code does not contain a copyright notice

#ifndef CAR_INTERFACES__MSG__DETAIL__GRAPH__STRUCT_HPP_
#define CAR_INTERFACES__MSG__DETAIL__GRAPH__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'nodes'
#include "geometry_msgs/msg/detail/pose_array__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__car_interfaces__msg__Graph __attribute__((deprecated))
#else
# define DEPRECATED__car_interfaces__msg__Graph __declspec(deprecated)
#endif

namespace car_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Graph_
{
  using Type = Graph_<ContainerAllocator>;

  explicit Graph_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : nodes(_init)
  {
    (void)_init;
  }

  explicit Graph_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : nodes(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _nodes_type =
    geometry_msgs::msg::PoseArray_<ContainerAllocator>;
  _nodes_type nodes;
  using _edges_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _edges_type edges;

  // setters for named parameter idiom
  Type & set__nodes(
    const geometry_msgs::msg::PoseArray_<ContainerAllocator> & _arg)
  {
    this->nodes = _arg;
    return *this;
  }
  Type & set__edges(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->edges = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    car_interfaces::msg::Graph_<ContainerAllocator> *;
  using ConstRawPtr =
    const car_interfaces::msg::Graph_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<car_interfaces::msg::Graph_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<car_interfaces::msg::Graph_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      car_interfaces::msg::Graph_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<car_interfaces::msg::Graph_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      car_interfaces::msg::Graph_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<car_interfaces::msg::Graph_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<car_interfaces::msg::Graph_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<car_interfaces::msg::Graph_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__car_interfaces__msg__Graph
    std::shared_ptr<car_interfaces::msg::Graph_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__car_interfaces__msg__Graph
    std::shared_ptr<car_interfaces::msg::Graph_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Graph_ & other) const
  {
    if (this->nodes != other.nodes) {
      return false;
    }
    if (this->edges != other.edges) {
      return false;
    }
    return true;
  }
  bool operator!=(const Graph_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Graph_

// alias to use template instance with default allocator
using Graph =
  car_interfaces::msg::Graph_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace car_interfaces

#endif  // CAR_INTERFACES__MSG__DETAIL__GRAPH__STRUCT_HPP_
