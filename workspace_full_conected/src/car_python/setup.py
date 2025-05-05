from setuptools import setup
import os
from glob import glob

package_name = 'car'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        #(os.path.join('share', package_name, 'models'), glob('models/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('lib', package_name), glob('lib/' + package_name + '/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='author',
    maintainer_email='todo@todo.com',
    description='The ' + package_name + ' package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "keyboard_control=car.keyboard_control:main",
            "joystick_control=car.joystick_control:main",
            'filter_points_cloud = car.filter_points_cloud:main',
            'points_goal = car.points_goal:main',
            'colision_zone = car.colision_zone:main',
            'supervisor_node = car.supervisor:main',
            'frontier_centroid = car.frontier_centroid:main',
            'map_navegation= car.map_navegation:main',
            'memory_map= car.memory_map:main',
            'path_ppo_cnn_lstm_training_init_mode= car.path_ppo_cnn_lstm_training_init_mode:main',
            'movement_predictive= car.movement_predictive:main',
            'lidar_detection= car.lidar_obstacle_detection:main',
            'path_ppo_cnn_lstm_training_init_mode_simple= car.path_ppo_cnn_lstm_training_init_mode_simple:main',
            'path_ppo_cnn_lstm_full_model= car.path_ppo_cnn_lstm_full_model:main',
            'inference_training_mode_simple= car.inference_training_mode_simple:main',
            'inference_full_model= car.inference_full_model:main',
        ],
    },
)
