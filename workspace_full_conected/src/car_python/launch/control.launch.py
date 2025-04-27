#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    package_name = 'car'
    share_dir = get_package_share_directory(package_name)
    
    # Ruta al archivo XACRO
    #xacro_file = os.path.join(share_dir, 'urdf', 'car.urdf.xacro')
    move_object_file = os.path.join(share_dir, 'urdf', 'object_movement.sdf')
    
    # # Procesa el archivo xacro y obtiene el URDF
    # #doc = xacro.process_file(xacro_file)
    # #robot_description = doc.toxml()

    # # Nodo robot_state_publisher
    # rsp_node = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     output='screen',
    #     parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    # )

    # Nodo para spawn del robot en Gazebo (usando el robot_description)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'car', '-topic', 'robot_description'],
        output='screen'
    )

    # Incluir el launch de Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'verbose': 'true', 'pause': 'false'}.items()
    )


    spawn_moving_obstacle = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'moving_obstacle',
            '-file', move_object_file
        ],
        output='screen'
    )


    return LaunchDescription([
        #rsp_node,
        gazebo_launch,
        spawn_entity,
        spawn_moving_obstacle
        
    ])

if __name__ == '__main__':
    generate_launch_description()
