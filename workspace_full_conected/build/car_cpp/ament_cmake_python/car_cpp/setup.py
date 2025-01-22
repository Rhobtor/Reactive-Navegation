from setuptools import find_packages
from setuptools import setup

setup(
    name='car_cpp',
    version='0.1.0',
    packages=find_packages(
        include=('car_cpp', 'car_cpp.*')),
)
