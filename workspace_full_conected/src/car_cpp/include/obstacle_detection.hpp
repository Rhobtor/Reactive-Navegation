#include "iostream" 
#include "octomap_msgs/Octomap.h" 
#include "octomap_msgs/conversions.h" 
#include "octomap/OcTree.h" 
#include "octomap/AbstractOcTree.h" 
#include "octomap/AbstractOccupancyOcTree.h" 
 
class detecta_obstaculo 
{ 
private: 
    octomap::AbstractOcTree *tree;  //Puntero a una estructura AbstractOcTree para 
almacenar en estructura arbol 
    octomap::OcTree *Octree; //Almacena el mapa de cuadrícula de ocupación 3D en un 
OcTree 
 
    bool octomap_recibido; 
     
public: 
    detecta_obstaculo(); 
    ~ detecta_obstaculo(); 
 
    void set_mapa(octomap::OcTree *Octree_); 
    bool obstaculo_cerca(const octomap::point3d location); 
    octomap::OcTree* get_map(); 
};
