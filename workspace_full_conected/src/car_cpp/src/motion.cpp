#include "evita_obstaculos/motion.hpp" 
 
motion::motion() 
{ 
}; 
 
geometry_msgs::Twist motion::motionControlAngle (double distance, double angle){ 
    twist_control.linear.x = 0.02; 
    if (angle > 0) 
    { 
        twist_control.angular.z = 0.3;; 
    } 
    else 
    { 
        twist_control.angular.z = -0.3; 
    } 
     
 
    return twist_control; 
} 
 
geometry_msgs::Twist motion::motionControlDistance (double distance, double angle){ 
    twist_control.linear.x = 0.5; 
    twist_control.angular.z = 0; 
 
    return twist_control; 
} 
 
geometry_msgs::Twist motion::motionControlSTOP ( ) 
{ 
    twist_control.linear.x = 0; 
    twist_control.angular.z = 0; 
 
    return twist_control; 
} 
 
motion::~motion(){}