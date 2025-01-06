from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
import xacro


def generate_launch_description():
    share_dir = get_package_share_directory('car')

    # Declare the world argument with a default value
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='mountain.world',  # Default world file
        description='World to load into Gazebo'
    )

    # Get the car URDF
    xacro_file = os.path.join(share_dir, 'urdf', 'car.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_urdf}]
    )

    # Joint State Publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    # Gazebo Server with world argument
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gzserver.launch.py'])
        ]),
        launch_arguments={
            'pause': 'true',
            'world': '/home/workspace/src/car/worlds/mountain.world'
        }.items()
    )

    # Gazebo Client
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gzclient.launch.py'])
        ])
    )

    # Spawn the car entity
    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'car', '-topic', 'robot_description','-x', '0', '-y', '0', '-z', '0.5'],
        output='screen'
    )

    # Return LaunchDescription
    return LaunchDescription([
        world_arg,  # Add the world argument to the launch description
        robot_state_publisher_node,
        joint_state_publisher_node,
        gazebo_server,
        gazebo_client,
        urdf_spawn_node,
    ])
