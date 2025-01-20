#include "vector" 
#include "nav_msgs/Odometry.h" 
#include "octomap_msgs/Octomap.h" 
#include "octomap_msgs/conversions.h" 
 
class odometria 
{ 
private: 
 
    double x_odom; 
    double y_odom; 
    double z_odom; 
    double qx_odom; 
    double qy_odom; 
    double qz_odom; 
    double qw_odom; 
 
    double linear; 
    double angular; 
 
public: 
    odometria(); 
    ~ odometria(); 
 
    bool set_puntoOdom(nav_msgs::Odometry::ConstPtr odometry, std::vector<double>& 
posicion, std::vector<double>& orientacion); 
    void set_velOdom(nav_msgs::Odometry::ConstPtr odometry, std::vector<double>& 
velocidad);  
}; 