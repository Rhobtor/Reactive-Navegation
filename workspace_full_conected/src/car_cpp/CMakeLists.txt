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
find_package(rosidl_default_generators REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)



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




add_executable(car_cpp ${SOURCES})
ament_target_dependencies(car_cpp rclcpp gazebo_ros std_msgs geometry_msgs sensor_msgs octomap octomap_msgs octomap_ros pcl_conversions pcl_msgs visualization_msgs nav_msgs)

install(TARGETS
  car_cpp
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(testo src/testo.cpp)
ament_target_dependencies(testo rclcpp gazebo_ros std_msgs geometry_msgs sensor_msgs octomap octomap_msgs octomap_ros pcl_conversions pcl_msgs visualization_msgs nav_msgs std_srvs geographic_msgs tf2 tf2_ros tf2_msgs rosidl_default_generators rmw_implementation_cmake tf2_msgs std_srvs geographic_msgs tf2_ros tf2_msgs tf2_geometry_msgs)  

install(TARGETS
  testo
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


# Library for control plugin
add_library(control_plugin SHARED src/control_plugin.cpp)
target_include_directories(control_plugin PUBLIC include)
ament_target_dependencies(control_plugin rclcpp gazebo_ros std_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_msgs)


# Library for odometry plugin
add_library(odometry_plugin SHARED src/odometry.cpp)
target_include_directories(odometry_plugin PUBLIC include)
ament_target_dependencies(odometry_plugin rclcpp gazebo_ros std_msgs geometry_msgs nav_msgs tf2 tf2_ros)


# # Plugin IMU
# add_library(hector_gazebo_ros_imu SHARED src/gazebo_ros_imu.cpp)
# target_link_libraries(hector_gazebo_ros_imu ${GAZEBO_LIBRARIES})
# ament_target_dependencies(hector_gazebo_ros_imu ${LIBS})

# # Plugin GPS
# add_library(hector_gazebo_ros_gps SHARED src/gazebo_ros_gps.cpp)
# target_link_libraries(hector_gazebo_ros_gps ${GAZEBO_LIBRARIES})
# ament_target_dependencies(hector_gazebo_ros_gps ${LIBS})



install(
  TARGETS control_plugin 
  LIBRARY DESTINATION lib
)


install(
  TARGETS odometry_plugin 
  LIBRARY DESTINATION lib
)



# Instalar archivos de encabezados
install(DIRECTORY include/
  DESTINATION include/
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
