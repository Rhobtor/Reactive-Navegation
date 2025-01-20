#include "avoide_obstacles/move_ros.hpp"
#include <rclcpp/rclcpp.hpp>


int main (int argc,char** argv)
{
    rclpp::init(argc,argv,"move");
    rclpp::NodeHandle n;

    move_ros navegation(n);
    navegation.run();
    return 0;
}