#include "motion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include <algorithm>
#define M_PI 3.14159265358979323846
motion::motion() {}

geometry_msgs::msg::Twist motion::motionControlAngle(double distance, double angle) {
    geometry_msgs::msg::Twist twist_control;
        const double ANGLE_THRESHOLD = 0.1;
        const double MAX_FORWARD_SPEED = 4.0;
        const double MAX_REVERSE_SPEED = 2.0;
        const double MIN_DISTANCE = 0.5;
        const double STEER_GAIN = 1.0;
        const double REVERSE_STEER_GAIN = 0.5;

        if (distance < MIN_DISTANCE) {
            twist_control.linear.x = 0.0;
            twist_control.angular.z = 0.0;
            return twist_control;
        }

        if (std::fabs(angle) <= ANGLE_THRESHOLD) {
            twist_control.linear.x = std::min(distance, MAX_FORWARD_SPEED);
            twist_control.angular.z = STEER_GAIN * angle;
        } else {
            twist_control.linear.x = -MAX_REVERSE_SPEED;
            if (angle > 0.0) {
                twist_control.angular.z = -REVERSE_STEER_GAIN * std::fabs(angle);
            } else {
                twist_control.angular.z = REVERSE_STEER_GAIN * std::fabs(angle);
            }
        }
        
        return twist_control;
    }



geometry_msgs::msg::Twist motion::motionControlDistance(double distance, double angle) {
    geometry_msgs::msg::Twist twist_control;
    twist_control.linear.x = 10;
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
