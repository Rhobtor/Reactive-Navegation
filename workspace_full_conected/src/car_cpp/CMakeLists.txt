cmake_minimum_required(VERSION 3.5)
project(car_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Library for control plugin
add_library(control_plugin SHARED src/control_plugin.cpp)
target_include_directories(control_plugin PUBLIC include)
ament_target_dependencies(control_plugin rclcpp gazebo_ros std_msgs geometry_msgs)

# Library for control plugin
add_executable(check_map src/check_map.cpp)
target_include_directories(check_map PUBLIC include)
ament_target_dependencies(check_map rclcpp octomap octomap_msgs visualization_msgs)


# # Executable for OctoMap Node
# add_executable(octree_node src/octree_node.cpp)
# target_include_directories(octree_node PUBLIC include)
# ament_target_dependencies(octree_node
#   rclcpp
#   sensor_msgs
#   octomap
#   octomap_msgs
#   pcl_conversions
#   pcl_msgs
# )
# target_link_libraries(octree_node
#   ${octomap_LIBRARIES}
# )

install(
  TARGETS control_plugin 
  LIBRARY DESTINATION lib
)

install(
  TARGETS check_map
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries(check_map
  ${octomap_LIBRARIES}
)

# install(
#   TARGETS octree_node
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
# )

ament_package()
