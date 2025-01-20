#include "evita_obstaculos/detecta_obstaculo.hpp" 
 
detecta_obstaculo::detecta_obstaculo() 
{ 
    octomap_recibido = false;  //si hemos recibido mensaje de octomap 
} 
 
void detecta_obstaculo::set_mapa (octomap::OcTree *Octree_) 
{ 
    octomap_recibido = true; 
    Octree = Octree_; 
} 
 
// VERIFICAR SI HAY UN OBTÁCULO EN UN PUNTO ESPECÍFICO (tipo octomap point3d) 
bool detecta_obstaculo::obstaculo_cerca(const octomap::point3d location)  
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
 
detecta_obstaculo::~detecta_obstaculo(){} 
 
octomap::OcTree* detecta_obstaculo::get_map(){ 
    return Octree; 
}