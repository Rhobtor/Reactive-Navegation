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


add_executable(move_navigation_nodes_frontier src/move_navigation_nodes_frontier.cpp)
ament_target_dependencies(move_navigation_nodes_frontier rclcpp gazebo_ros std_msgs geometry_msgs sensor_msgs octomap octomap_msgs octomap_ros pcl_conversions pcl_msgs visualization_msgs nav_msgs std_srvs geographic_msgs tf2 tf2_ros tf2_msgs  rmw_implementation_cmake tf2_msgs std_srvs geographic_msgs tf2_ros tf2_msgs tf2_geometry_msgs)  

install(TARGETS
  move_navigation_nodes_frontier
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(frontier_values src/frontier_values.cpp)
ament_target_dependencies(frontier_values rclcpp gazebo_ros std_msgs geometry_msgs sensor_msgs octomap octomap_msgs octomap_ros pcl_conversions pcl_msgs visualization_msgs nav_msgs std_srvs geographic_msgs tf2 tf2_ros tf2_msgs  rmw_implementation_cmake tf2_msgs std_srvs geographic_msgs tf2_ros tf2_msgs tf2_geometry_msgs)  

install(TARGETS
  frontier_values
  DESTINATION lib/${PROJECT_NAME}
)




add_executable(occupied_nodes_near_obstacles src/occupied_nodes_near_obstacles.cpp)
ament_target_dependencies(occupied_nodes_near_obstacles rclcpp gazebo_ros std_msgs geometry_msgs sensor_msgs octomap octomap_msgs octomap_ros pcl_conversions pcl_msgs visualization_msgs nav_msgs std_srvs geographic_msgs tf2 tf2_ros tf2_msgs  rmw_implementation_cmake tf2_msgs std_srvs geographic_msgs tf2_ros tf2_msgs tf2_geometry_msgs)  

install(TARGETS
  occupied_nodes_near_obstacles
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




# #Library for control plugin
# add_library(control_plugin SHARED src/control_plugin.cpp)
# target_include_directories(control_plugin PUBLIC include)
# ament_target_dependencies(control_plugin rclcpp ignition-common3 gazebo_ros std_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_msgs)

############################################################# prueba
#Library for control plugin
add_library(joints_plugin SHARED src/joints.cpp)
target_include_directories(joints_plugin PUBLIC include)
ament_target_dependencies(joints_plugin rclcpp ignition-common3 sensor_msgs gazebo_ros std_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_msgs)
###################################################################


#Library for control plugin
add_library(drive_plugin SHARED src/diffdrive_plugin_multi_wheel.cpp)
target_include_directories(drive_plugin PUBLIC include)
ament_target_dependencies(drive_plugin rclcpp ignition-common3 sensor_msgs gazebo_ros std_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_msgs)

#Library for control plugin
add_library(move_plugin SHARED src/move_object.cpp)
target_include_directories(move_plugin PUBLIC include)
ament_target_dependencies(move_plugin rclcpp ignition-common3 sensor_msgs gazebo_ros std_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_msgs)



# #Library for control plugin
# add_library(driver_plugin SHARED src/control_plugin.cpp)
# target_include_directories(driver_plugin PUBLIC include)
# ament_target_dependencies(driver_plugin rclcpp ignition-common3 gazebo_ros std_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_msgs)







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



# Instalar encabezados
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
