#include "iostream" 
#include "visualization_msgs/Marker.h" 
#include "visualization_msgs/MarkerArray.h" 
#include "octomap_msgs/Octomap.h" 
#include "octomap_msgs/conversions.h" 
#include "octomap/OcTree.h" 
 
class visualization 
{ 
private: 
    visualization_msgs::MarkerArray vector_visualizacion; 
    visualization_msgs::Marker punto_visualizacion; 
 
public: 
    visualization(); 
    ~visualization(); 
 
    visualization_msgs::Marker PointColor(octomap::point3d point, int iter, float r, 
float g, float b); 
    visualization_msgs::Marker createLineMarker(octomap::point3d start, 
octomap::point3d end,int iteracion); 
    visualization_msgs::Marker deleteAllPoints(); 
}; 
