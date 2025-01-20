#include "iostream" 
#include "geometry_msgs/Twist.h" 
 
class motion 
{ 
private: 
 
    geometry_msgs::Twist twist_control; 
 
public: 
    motion();  
 
    geometry_msgs::Twist motionControlAngle (double distance, double angle); 
    geometry_msgs::Twist motionControlDistance (double distance, double angle); 
    geometry_msgs::Twist motionControlSTOP (); 
     
}; 
 