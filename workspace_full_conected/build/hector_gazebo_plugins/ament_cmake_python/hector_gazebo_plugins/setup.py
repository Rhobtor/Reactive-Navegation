from setuptools import find_packages
from setuptools import setup

setup(
    name='hector_gazebo_plugins',
    version='0.5.4',
    packages=find_packages(
        include=('hector_gazebo_plugins', 'hector_gazebo_plugins.*')),
)
