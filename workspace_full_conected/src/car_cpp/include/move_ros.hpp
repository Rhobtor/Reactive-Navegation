#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/Twist.h"
#include "octomap_msgs/Octomap.h"
#include "octomap_msgs/conversions.h"
#include "octomap/OcTree.h"
#include "octomap/AbstracOctree.h"
#include "octomap/AbstractOccupancyOctree.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Odometry.h"
#include "fstream"
#include "avoid_obstacles/detecta_obstaculo.hpp" 
#include "avoid_obstacles/odometria.hpp" 
#include "avoid_obstacles/utilities.hpp" 
#include "avoid_obstacles/motion.hpp" 
#include "avoid_obstacles/visualization.hpp" 

class move_ros public rclcpp::Node
{

    private:
    obstacle detected obstacle_detected_p;
    odometry odometry_p;
    motion motion_p;
    visualization visualization_p;

    octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
            "/octomap_binary", 10, std::bind(&CheckMapDisplayNode::octomapCallback, this, std::placeholders::_1));

    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry", 10, std::bind(&CheckMapDisplayNode::odometryCallback, this, std::placeholders::_1));

    vel_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/octomap_markers", 10);
    visualization_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization", 10);

     octomap::AbstractOcTree *tree;  //Puntero a una estructura AbstractOcTree para almacenar en estructura arbol 
    octomap::OcTree *Octree; //Almacena el mapa de cuadrícula de ocupación 3D en un OcTree 
     
    nav_msgs::Odometry::ConstPtr odometry; 
     
    std::vector<double> pos_actual; 
    std::vector<double> orientacion_actual_q; 
    double orientacion_actual_E; 
    double orientacion_prueba;
     double distancia_final; 
    char nueva_posicion_final; 
    char continuar; 
    bool bool_odom; 
    bool octomap_recibido; 
    double distancia_punto_destino; 
    double distancia_anterior_punto_destino; 
    double angulo_punto_destino; 
    double dif_angular; 
    char cont; 
    int interaciones_ejecucion_totales; 
    double radio_circunferencia; 
    double numPuntos_circunferecnia; 
    bool evaluar_recta; 
    bool punto_nuevo; 
    bool no_punto_valido; 
 
    double radio_ext; 
    double radio_int; 
    double num_puntos; 
    double distancia_calculo; 
    double tolerancia_angular; 
    double tolerancia_distancia; 
    double tolerancia_distancia_final; 
    octomap::point3d PuntoFIN3d; 
    double pos_fin_x; 
    double pos_fin_y; 
    double pos_fin_z; 

     std::vector<octomap::point3d> Vector_puntos_aleatorios; 
    std::vector<double> Vector_distancias_pto_final; 
    std::vector<octomap::point3d> vector_intermediatePoints; 
    std::vector<octomap::point3d> vector_circunferencePoints; 
 
    octomap::point3d pos_actual3d; 
    std::vector<double> RandomPoint; 
    octomap::point3d RandomPoint3d; 
    bool occupation; 
    bool ocupacion_punto_destino; 
    bool inter_occupation; 
    bool circunference_occupation; 
    double distancia_pto_final; 
    int indice_min_distancia; 
    octomap::point3d pos_destino3d; 
    geometry_msgs::Twist Comando_velocidad; 
    bool no_avanzar;
    visualization_msgs::Marker borrar_puntos; 
    visualization_msgs::Marker punto_visualizacion; 
    visualization_msgs::MarkerArray array_visualizacion; 
    visualization_msgs::Marker linea_visualizacion; 
 
    void octomapCallback(const octomap_msgs::Octomap::ConstPtr& octomap_msg); 
    void odometryCallback(nav_msgs::Odometry::ConstPtr odometry_); 
     
public: 
 
    move_ros(ros::NodeHandle n); 
    ~ move_ros(); 
 
    void run(); 


}