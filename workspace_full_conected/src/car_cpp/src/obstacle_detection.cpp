#include "obstacle_detection.hpp" 
 
obstacle_detection::obstacle_detection() 
{ 
    octomap_recibido = false;  //si hemos recibido mensaje de octomap 
} 
 
void obstacle_detection::set_mapa (octomap::OcTree *Octree_) 
{ 
    octomap_recibido = true; 
    Octree = Octree_; 
} 
 
// VERIFICAR SI HAY UN OBTÁCULO EN UN PUNTO ESPECÍFICO (tipo octomap point3d) 
bool obstacle_detection::obstaculo_cerca(const octomap::point3d location)  
{ 
     
    if (!Octree)  
    { 
    std::cout << "Mapa OctoMap no disponible." << std::endl; 
    return false; 
    } 
 
    octomap::OcTreeNode* result = Octree->search(location); 
 
    if (result && Octree->isNodeOccupied(result)) 
    {  
        //std::cout << "Hay un obstáculo     " << std::endl; 
        return true; 
    }  
    else  
    { 
        //std::cout << "No hay obstáculo    " <<std::endl; 
        return false; 
    } 
} 
 
obstacle_detection::~obstacle_detection(){} 
 
octomap::OcTree* obstacle_detection::get_map(){ 
    return Octree; 
}