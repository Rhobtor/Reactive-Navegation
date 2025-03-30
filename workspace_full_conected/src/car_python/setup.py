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
            "log_joystick=car.log_joystick:main",
            'octomap_2d_display = car.octomap_2d_display:main',
            'ekf_node = car.ekf_node:main',
            'odometry_node = car.odometry:main',
            'debug_node = car.debugtf:main',
            'camara_time_node = car.camara_time:main',
            'map = car.map:main',
            'scan_to_cloud = car.laser_scan:main',
            'filter_points_cloud = car.filter_points_cloud:main',
            'view_point_and_click = car.view_point_and_click:main',
            'frontier_view = car.frontier:main',
            'graph = car.graph:main',
            'points_goal = car.points_goal:main',
            'train_ppo = car.train_ppo:main',
            'inference_ppo = car.inference_ppo:main',
            'colision_zone = car.colision_zone:main',
            'supervisor_node = car.supervisor:main',
            'dqn_ppo = car.dqn_ppo:main',
            'inference_dqn_ppo = car.inference_dqn_ppo:main',
            'ppo= car.ppo:main',
            'imagine_ppo= car.imagine_ppo:main',
            'imagine_ppo_map= car.imagine_ppo_map:main',
        ],
    },
)
