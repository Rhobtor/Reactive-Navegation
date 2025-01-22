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
find_package(octomap_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Agregar directorios de encabezados
include_directories(include) 

# Archivos fuente
set(SOURCES
  src/check_map.cpp
  src/visualization.cpp
  src/motion.cpp
  src/control_plugin.cpp
  src/move.cpp
  src/move_ros.cpp
  src/obstacle_detection.cpp
  src/odometria.cpp
  src/path_planner.cpp
  src/utilities.cpp

)


add_executable(car_cpp ${SOURCES})
ament_target_dependencies(car_cpp rclcpp gazebo_ros std_msgs geometry_msgs sensor_msgs octomap octomap_msgs octomap_ros pcl_conversions pcl_msgs visualization_msgs nav_msgs)

install(TARGETS
  car_cpp
  DESTINATION lib/${PROJECT_NAME}
)



# # Library for control plugin
# add_library(control_plugin SHARED src/control_plugin.cpp)
# target_include_directories(control_plugin PUBLIC include)
# ament_target_dependencies(control_plugin rclcpp gazebo_ros std_msgs geometry_msgs)

# # Library for control plugin
# add_executable(check_map src/check_map.cpp)
# target_include_directories(check_map PUBLIC include)
# ament_target_dependencies(check_map rclcpp octomap octomap_msgs visualization_msgs)


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

# install(
#   TARGETS control_plugin 
#   LIBRARY DESTINATION lib
# )

# install(
#   TARGETS check_map
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
# )

# target_link_libraries(check_map
#   ${octomap_LIBRARIES}
# )

# Instalar archivos de encabezados
install(DIRECTORY include/
  DESTINATION include/
)

# install(
#   TARGETS octree_node
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
# )

ament_package()
