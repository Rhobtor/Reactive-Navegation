#!/bin/bash
while true; do
    echo "Lanzando entorno..."
    ros2 launch car gazebo_simple_world.launch.py 
    echo "El entorno se detuvo o quedó atascado. Reiniciando en 2 segundos..."
    sleep 2
done
