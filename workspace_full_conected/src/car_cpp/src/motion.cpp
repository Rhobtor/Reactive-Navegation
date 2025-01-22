#include "motion.hpp"
#include "geometry_msgs/msg/twist.hpp"

motion::motion() {}

geometry_msgs::msg::Twist motion::motionControlAngle(double distance, double angle) {
    geometry_msgs::msg::Twist twist_control;
    twist_control.linear.x = 0.02;

    if (angle > 0) {
        twist_control.angular.z = 0.3;
    } else {
        twist_control.angular.z = -0.3;
    }

    return twist_control;
}

geometry_msgs::msg::Twist motion::motionControlDistance(double distance, double angle) {
    geometry_msgs::msg::Twist twist_control;
    twist_control.linear.x = 0.5;
    twist_control.angular.z = 0;

    return twist_control;
}

geometry_msgs::msg::Twist motion::motionControlSTOP() {
    geometry_msgs::msg::Twist twist_control;
    twist_control.linear.x = 0;
    twist_control.angular.z = 0;

    return twist_control;
}

motion::~motion() {}
