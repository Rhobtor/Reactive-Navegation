cmake_minimum_required(VERSION 3.5)
project(car_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
#Library
add_library(control_plugin SHARED src/control_plugin.cpp)
target_include_directories(control_plugin PUBLIC include)
ament_target_dependencies(control_plugin rclcpp gazebo_ros std_msgs geometry_msgs)

install(
  TARGETS control_plugin
  LIBRARY DESTINATION lib
)

ament_package()
