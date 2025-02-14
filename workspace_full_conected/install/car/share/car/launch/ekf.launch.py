from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import os
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the share directory of your package
    share_dir = get_package_share_directory('car')

    # Instead of using os.path.join to build a string, use PathJoinSubstitution.
    # This creates a substitution that is compatible with the launch system.
    world_arg = PathJoinSubstitution([
        FindPackageShare('car'),
        'worlds',
        'map1_test.world'
    ])

    # Get the car URDF by processing the xacro file
    xacro_file = os.path.join(share_dir, 'urdf', 'car.xacro')
    config_file = os.path.join(share_dir, 'config', 'ekf_params.yaml')






    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[config_file]
        )
    ])

