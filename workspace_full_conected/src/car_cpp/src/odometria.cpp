#include "odometria.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <iostream>

odometria::odometria()
    : x_odom(0), y_odom(0), z_odom(0),
      qx_odom(0), qy_odom(0), qz_odom(0), qw_odom(0),
      linear(0), angular(0) {}

bool odometria::set_puntoOdom(const std::shared_ptr<const nav_msgs::msg::Odometry>& odometry,
                               std::vector<double>& posicion,
                               std::vector<double>& orientacion)
{

    if (odometry) {
        // Extraer posición del mensaje de la Odometría
        x_odom = odometry->pose.pose.position.x;
        y_odom = odometry->pose.pose.position.y;
        z_odom = odometry->pose.pose.position.z;
        qx_odom = odometry->pose.pose.orientation.x;
        qy_odom = odometry->pose.pose.orientation.y;
        qz_odom = odometry->pose.pose.orientation.z;
        qw_odom = odometry->pose.pose.orientation.w;

        posicion = {x_odom, y_odom, z_odom};
        orientacion = {qx_odom, qy_odom, qz_odom, qw_odom};

        return true;
    } else {
        return false;
    }
}

void odometria::set_velOdom(const std::shared_ptr<nav_msgs::msg::Odometry>& odometry,
                             std::vector<double>& vel) 
{
    std::cout << "set_velOdom" << std::endl;
    // Extraer velocidad del mensaje de la Odometría
    linear = odometry->twist.twist.linear.x;
    angular = odometry->twist.twist.angular.z;
    // Declaración de la variable 'velocidad'
    std::vector<double> velocidad;
    velocidad = {linear, angular};
    std::cout << "set_velOdom_2   " << velocidad[1] << "   " << velocidad[0] << std::endl;
}

odometria::~odometria() {}
