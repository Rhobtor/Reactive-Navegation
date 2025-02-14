#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            name='rgbd_sync',
            parameters=[{
                'approx_sync': True,         # Sincronización aproximada
                'sync_queue_size': 10,       # Tamaño de la cola de sincronización
                'topic_queue_size': 10,      # Tamaño de la cola para cada tópico
                'depth_scale': 1.0,
                'decimation': 1,
                'compressed_rate': 0.0
            }],
            remappings=[
                # Remapea las entradas a los tópicos publicados por el plugin de Gazebo
                ('rgb/image', '/depth_camera/image_raw'),
                ('depth/image', '/depth_camera/depth/image_raw'),
                ('rgb/camera_info', '/depth_camera/camera_info'),
                # El nodo publicará el mensaje sincronizado en este tópico (lo usará RTAB-Map)
                ('rgbd_image', '/rgbd_image')
            ]
        )
    ])
