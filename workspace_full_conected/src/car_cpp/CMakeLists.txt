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
find_package(std_srvs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ignition-math6 REQUIRED)
find_package(ignition-common3 REQUIRED)
find_package(car_interfaces REQUIRED)



# Agregar directorios de encabezados
include_directories(include) 





# Archivos fuente
set(SOURCES
  src/visualization.cpp
  src/motion.cpp
  src/control_plugin.cpp
  src/move.cpp
  src/move_ros.cpp
  src/obstacle_detection.cpp
  src/odometria.cpp
  src/utilities.cpp
)

set(LIBS
    gazebo_ros
    geographic_msgs 
    geometry_msgs
    sensor_msgs
    rclcpp
    std_msgs
    std_srvs
    tf2
    tf2_ros
    tf2_msgs
)

# ## Generate messages
# rosidl_generate_interfaces(${PROJECT_NAME}
#   srv/SetBias.srv
#   srv/SetReferenceGeoPose.srv
#   DEPENDENCIES geographic_msgs geometry_msgs builtin_interfaces
# )

# add_library(hector_gazebo_ros_imu SHARED src/gazebo_ros_imu.cpp)

# ament_target_dependencies(hector_gazebo_ros_imu
#   ${LIBS}
#   rosidl_runtime_cpp
# )




# add_executable(car_cpp ${SOURCES})
# ament_target_dependencies(car_cpp rclcpp gazebo_ros std_msgs geometry_msgs sensor_msgs octomap octomap_msgs octomap_ros pcl_conversions pcl_msgs visualization_msgs nav_msgs)

# install(TARGETS
#   car_cpp
#   DESTINATION lib/${PROJECT_NAME}
# )


add_executable(navigation_nodes src/navigation_nodes.cpp)
ament_target_dependencies(navigation_nodes rclcpp gazebo_ros std_msgs geometry_msgs sensor_msgs octomap octomap_msgs octomap_ros pcl_conversions pcl_msgs visualization_msgs nav_msgs std_srvs geographic_msgs tf2 tf2_ros tf2_msgs  rmw_implementation_cmake tf2_msgs std_srvs geographic_msgs tf2_ros tf2_msgs tf2_geometry_msgs)  

install(TARGETS
  navigation_nodes
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(navigation_nodes_ground src/navigation_nodes_ground.cpp)
ament_target_dependencies(navigation_nodes_ground rclcpp gazebo_ros std_msgs geometry_msgs sensor_msgs octomap octomap_msgs octomap_ros pcl_conversions pcl_msgs visualization_msgs nav_msgs std_srvs geographic_msgs tf2 tf2_ros tf2_msgs  rmw_implementation_cmake tf2_msgs std_srvs geographic_msgs tf2_ros tf2_msgs tf2_geometry_msgs)  

install(TARGETS
  navigation_nodes_ground
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(frontier_nodes src/frontier_nodes.cpp)
ament_target_dependencies(frontier_nodes rclcpp gazebo_ros std_msgs geometry_msgs sensor_msgs octomap octomap_msgs octomap_ros pcl_conversions pcl_msgs visualization_msgs nav_msgs std_srvs geographic_msgs tf2 tf2_ros tf2_msgs  rmw_implementation_cmake tf2_msgs std_srvs geographic_msgs tf2_ros tf2_msgs tf2_geometry_msgs)  

install(TARGETS
  frontier_nodes
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(frontier_reduction src/frontier_reduction.cpp)
ament_target_dependencies(frontier_reduction rclcpp gazebo_ros std_msgs geometry_msgs sensor_msgs octomap octomap_msgs octomap_ros pcl_conversions pcl_msgs visualization_msgs nav_msgs std_srvs geographic_msgs tf2 tf2_ros tf2_msgs  rmw_implementation_cmake tf2_msgs std_srvs geographic_msgs tf2_ros tf2_msgs tf2_geometry_msgs)  

install(TARGETS
  frontier_reduction
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(ground_remove src/ground_remove.cpp)
ament_target_dependencies(ground_remove rclcpp gazebo_ros std_msgs geometry_msgs sensor_msgs octomap octomap_msgs octomap_ros pcl_conversions pcl_msgs visualization_msgs nav_msgs std_srvs geographic_msgs tf2 tf2_ros tf2_msgs  rmw_implementation_cmake tf2_msgs std_srvs geographic_msgs tf2_ros tf2_msgs tf2_geometry_msgs)  

install(TARGETS
  ground_remove
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(camera_filter src/camera_filter.cpp)
ament_target_dependencies(camera_filter rclcpp gazebo_ros std_msgs geometry_msgs sensor_msgs octomap octomap_msgs octomap_ros pcl_conversions pcl_msgs visualization_msgs nav_msgs std_srvs geographic_msgs tf2 tf2_ros tf2_msgs  rmw_implementation_cmake tf2_msgs std_srvs geographic_msgs tf2_ros tf2_msgs tf2_geometry_msgs)  

install(TARGETS
  camera_filter
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(move_navigation_nodes_frontier src/move_navigation_nodes_frontier.cpp)
ament_target_dependencies(move_navigation_nodes_frontier rclcpp gazebo_ros std_msgs geometry_msgs sensor_msgs octomap octomap_msgs octomap_ros pcl_conversions pcl_msgs visualization_msgs nav_msgs std_srvs geographic_msgs tf2 tf2_ros tf2_msgs  rmw_implementation_cmake tf2_msgs std_srvs geographic_msgs tf2_ros tf2_msgs tf2_geometry_msgs)  

install(TARGETS
  move_navigation_nodes_frontier
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(frontiers_values src/frontiers_values.cpp)
ament_target_dependencies(frontiers_values rclcpp gazebo_ros std_msgs geometry_msgs sensor_msgs octomap octomap_msgs octomap_ros pcl_conversions pcl_msgs visualization_msgs nav_msgs std_srvs geographic_msgs tf2 tf2_ros tf2_msgs  rmw_implementation_cmake tf2_msgs std_srvs geographic_msgs tf2_ros tf2_msgs tf2_geometry_msgs)  

install(TARGETS
  frontiers_values
  DESTINATION lib/${PROJECT_NAME}
)



add_executable(map_measure src/map_measure.cpp)
ament_target_dependencies(map_measure rclcpp gazebo_ros std_msgs geometry_msgs sensor_msgs octomap octomap_msgs octomap_ros pcl_conversions pcl_msgs visualization_msgs nav_msgs std_srvs geographic_msgs tf2 tf2_ros tf2_msgs  rmw_implementation_cmake tf2_msgs std_srvs geographic_msgs tf2_ros tf2_msgs tf2_geometry_msgs)  

install(TARGETS
  map_measure
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(occupied_nodes_near_obstacles src/occupied_nodes_near_obstacles.cpp)
ament_target_dependencies(occupied_nodes_near_obstacles rclcpp gazebo_ros std_msgs geometry_msgs sensor_msgs octomap octomap_msgs octomap_ros pcl_conversions pcl_msgs visualization_msgs nav_msgs std_srvs geographic_msgs tf2 tf2_ros tf2_msgs  rmw_implementation_cmake tf2_msgs std_srvs geographic_msgs tf2_ros tf2_msgs tf2_geometry_msgs)  

install(TARGETS
  occupied_nodes_near_obstacles
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(create_wall src/create_wall.cpp)
ament_target_dependencies(create_wall rclcpp gazebo_ros std_msgs geometry_msgs sensor_msgs octomap octomap_msgs octomap_ros pcl_conversions pcl_msgs visualization_msgs nav_msgs std_srvs geographic_msgs tf2 tf2_ros tf2_msgs  rmw_implementation_cmake tf2_msgs std_srvs geographic_msgs tf2_ros tf2_msgs tf2_geometry_msgs)  

install(TARGETS
  create_wall
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(2D_graph src/2D_graph.cpp)
ament_target_dependencies(2D_graph rclcpp gazebo_ros std_msgs geometry_msgs sensor_msgs octomap octomap_msgs octomap_ros pcl_conversions pcl_msgs visualization_msgs nav_msgs std_srvs geographic_msgs tf2 tf2_ros tf2_msgs  rmw_implementation_cmake tf2_msgs std_srvs geographic_msgs tf2_ros tf2_msgs tf2_geometry_msgs)

install(TARGETS
  2D_graph
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(obstacles_in_2d src/obstacles_in_2d.cpp)
ament_target_dependencies(obstacles_in_2d rclcpp gazebo_ros std_msgs geometry_msgs sensor_msgs octomap octomap_msgs octomap_ros pcl_conversions pcl_msgs visualization_msgs nav_msgs std_srvs geographic_msgs tf2 tf2_ros tf2_msgs  rmw_implementation_cmake tf2_msgs std_srvs geographic_msgs tf2_ros tf2_msgs tf2_geometry_msgs)

install(TARGETS
  obstacles_in_2d
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(check_goal src/check_goal.cpp)
ament_target_dependencies(check_goal rclcpp gazebo_ros std_msgs geometry_msgs sensor_msgs octomap octomap_msgs octomap_ros pcl_conversions pcl_msgs visualization_msgs nav_msgs std_srvs geographic_msgs tf2 tf2_ros tf2_msgs  rmw_implementation_cmake tf2_msgs std_srvs geographic_msgs tf2_ros tf2_msgs tf2_geometry_msgs)

install(TARGETS
  check_goal
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(filtered_navigation_nodes src/filtered_navigation_nodes.cpp)
ament_target_dependencies(filtered_navigation_nodes rclcpp gazebo_ros std_msgs geometry_msgs sensor_msgs octomap octomap_msgs octomap_ros pcl_conversions pcl_msgs visualization_msgs nav_msgs std_srvs geographic_msgs tf2 tf2_ros tf2_msgs  rmw_implementation_cmake tf2_msgs std_srvs geographic_msgs tf2_ros tf2_msgs tf2_geometry_msgs)

install(TARGETS
  filtered_navigation_nodes
  DESTINATION lib/${PROJECT_NAME}
)

#check_map
add_executable(check_map src/check_map.cpp)
ament_target_dependencies(check_map rclcpp gazebo_ros std_msgs geometry_msgs sensor_msgs octomap octomap_msgs octomap_ros pcl_conversions pcl_msgs visualization_msgs nav_msgs)

install(TARGETS
  check_map
  DESTINATION lib/${PROJECT_NAME}
)

#path planner
add_executable(path_planner_node src/path_planner.cpp)
ament_target_dependencies(path_planner_node rclcpp gazebo_ros std_msgs geometry_msgs sensor_msgs octomap octomap_msgs octomap_ros pcl_conversions pcl_msgs visualization_msgs nav_msgs)

install(TARGETS
path_planner_node
  DESTINATION lib/${PROJECT_NAME}
)


#Library for control plugin
add_library(control_plugin SHARED src/control_plugin.cpp)
target_include_directories(control_plugin PUBLIC include)
ament_target_dependencies(control_plugin rclcpp ignition-common3 gazebo_ros std_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_msgs)


#Library for control plugin
add_library(joints_plugin SHARED src/joints.cpp)
target_include_directories(joints_plugin PUBLIC include)
ament_target_dependencies(joints_plugin rclcpp ignition-common3 sensor_msgs gazebo_ros std_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_msgs)

#Library for control plugin
add_library(drive_plugin SHARED src/diffdrive_plugin_multi_wheel.cpp)
target_include_directories(drive_plugin PUBLIC include)
ament_target_dependencies(drive_plugin rclcpp ignition-common3 sensor_msgs gazebo_ros std_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_msgs)

#Library for control plugin
add_library(move_plugin SHARED src/move_object.cpp)
target_include_directories(move_plugin PUBLIC include)
ament_target_dependencies(move_plugin rclcpp ignition-common3 sensor_msgs gazebo_ros std_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_msgs)



#Library for control plugin
add_library(driver_plugin SHARED src/control_plugin.cpp)
target_include_directories(driver_plugin PUBLIC include)
ament_target_dependencies(driver_plugin rclcpp ignition-common3 gazebo_ros std_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_msgs)




install(
  TARGETS control_plugin 
  LIBRARY DESTINATION lib
)


install(
  TARGETS move_plugin 
  LIBRARY DESTINATION lib
)


install(
  TARGETS drive_plugin 
  LIBRARY DESTINATION lib
)

install(
  TARGETS joints_plugin 
  LIBRARY DESTINATION lib
)

install(
  TARGETS driver_plugin 
  LIBRARY DESTINATION lib
)


# # Instalar plugins
# install(TARGETS
#   hector_gazebo_ros_imu
#   hector_gazebo_ros_gps
#   LIBRARY DESTINATION lib
# )

# Instalar encabezados
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
