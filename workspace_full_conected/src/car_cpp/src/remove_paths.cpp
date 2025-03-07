// octomap_navigation_nodes.cpp
// Este nodo suscribe al tópico "octomap" (de tipo octomap_msgs::msg::Octomap),
// procesa el árbol OctoMap para proyectar la información 3D en el suelo, y
// publica "nodos de obstáculo" (como PoseArray y MarkerArray) que indican
// las áreas donde el vehículo no debe pasar, basándose en la diferencia
// de altura respecto al suelo en cada celda.

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

// Incluimos los mensajes de OctoMap y la conversión al árbol
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>

// Incluir la librería OctoMap para acceder a la clase OcTree
#include <octomap/octomap.h>

#include <unordered_map>
#include <cmath>
#include <utility>

// Definición de una función hash para std::pair<int,int> para usarlo en unordered_map
struct PairHash {
  std::size_t operator()(const std::pair<int,int>& p) const {
    auto h1 = std::hash<int>{}(p.first);
    auto h2 = std::hash<int>{}(p.second);
    return h1 ^ (h2 << 1);
  }
};

class OctomapNavigationNodes : public rclcpp::Node {
public:
  OctomapNavigationNodes() : Node("octomap_navigation_nodes")
  {
    // Declarar parámetros configurables:
    // obstacle_height_threshold: diferencia mínima en z para considerar que en una celda hay obstáculo
    // cell_size: tamaño de la celda para agrupar los puntos (en metros)
    this->declare_parameter("obstacle_height_threshold", 0.2);
    this->declare_parameter("cell_size", 0.1);
    obstacle_height_threshold_ = this->get_parameter("obstacle_height_threshold").as_double();
    cell_size_ = this->get_parameter("cell_size").as_double();

    // Suscribirse al tópico del OctoMap
    octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      "octomap_full", 10,
      std::bind(&OctomapNavigationNodes::octomapCallback, this, std::placeholders::_1)
    );

    // Publicador para los nodos de obstáculo (PoseArray)
    obstacle_nodes_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("obstacle_navigation_nodes", 10);

    // Publicador para visualizar los nodos de obstáculo con markers
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("obstacle_nodes_markers", 10);

    RCLCPP_INFO(this->get_logger(), "Nodo OctomapNavigationNodes iniciado. Umbral obstáculo: %.2f, Tamaño de celda: %.2f",
                obstacle_height_threshold_, cell_size_);
  }

private:
  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
  {
    // Convertir el mensaje Octomap a un objeto OcTree
    // La función fullMsgToMap realiza la conversión y devuelve un AbstractOcTree*
    octomap::AbstractOcTree* abstract_tree = octomap_msgs::fullMsgToMap(*msg);
    if (!abstract_tree) {
      RCLCPP_ERROR(this->get_logger(), "Error al convertir el mensaje Octomap al árbol OcTree.");
      return;
    }
    // Hacemos un cast dinámico a OcTree (la implementación común de OctoMap)
    octomap::OcTree* tree = dynamic_cast<octomap::OcTree*>(abstract_tree);
    if (!tree) {
      RCLCPP_ERROR(this->get_logger(), "Error al convertir AbstractOcTree a OcTree.");
      delete abstract_tree;
      return;
    }

    // Utilizaremos dos mapas (hash maps) para:
    // 1. Almacenar el nivel del suelo en cada celda (mínimo valor z).
    // 2. Marcar si en una celda se detecta al menos un punto que supere el umbral (obstáculo).
    std::unordered_map<std::pair<int,int>, double, PairHash> ground_levels;
    std::unordered_map<std::pair<int,int>, bool, PairHash> obstacle_cells;

    // Primera pasada: determinar el nivel del suelo en cada celda.
    // Para cada nodo hoja ocupado, se calcula la celda (x,y) a partir del tamaño de celda.
    for (octomap::OcTree::leaf_iterator it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it) {
      // Consideramos nodos ocupados (umbral típico 0.5 para ocupación)
      if (it->getOccupancy() > 0.5) {
        double x = it.getX();
        double y = it.getY();
        double z = it.getZ();
        int cell_x = static_cast<int>(std::floor(x / cell_size_));
        int cell_y = static_cast<int>(std::floor(y / cell_size_));
        std::pair<int,int> cell(cell_x, cell_y);
        // Actualizamos el valor mínimo de z (nivel del suelo) para la celda
        if (ground_levels.find(cell) == ground_levels.end()) {
          ground_levels[cell] = z;
        } else {
          if (z < ground_levels[cell]) {
            ground_levels[cell] = z;
          }
        }
      }
    }

    // Segunda pasada: identificar en cada celda si existe un obstáculo.
    // Se recorre de nuevo el árbol y, para cada nodo ocupado, se compara su altura con el nivel del suelo
    for (octomap::OcTree::leaf_iterator it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it) {
      if (it->getOccupancy() > 0.5) {
        double x = it.getX();
        double y = it.getY();
        double z = it.getZ();
        int cell_x = static_cast<int>(std::floor(x / cell_size_));
        int cell_y = static_cast<int>(std::floor(y / cell_size_));
        std::pair<int,int> cell(cell_x, cell_y);
        // Si se conoce el nivel del suelo para la celda, se verifica si la diferencia es mayor al umbral
        if (ground_levels.find(cell) != ground_levels.end()) {
          double ground_z = ground_levels[cell];
          if (z - ground_z > obstacle_height_threshold_) {
            obstacle_cells[cell] = true;
          }
        }
      }
    }

    // Crear un PoseArray para publicar los "nodos de obstáculo" proyectados sobre el suelo.
    geometry_msgs::msg::PoseArray obstacle_nodes;
    obstacle_nodes.header = msg->header;

    // Crear un MarkerArray para visualizar estos nodos (por ejemplo, como cubos rojos)
    visualization_msgs::msg::MarkerArray markers;
    int marker_id = 0;

    // Iterar sobre las celdas marcadas como obstáculo
    for (const auto &pair : obstacle_cells) {
      if (pair.second) {
        int cell_x = pair.first.first;
        int cell_y = pair.first.second;
        // Obtener el nivel del suelo en la celda
        double ground_z = ground_levels[pair.first];
        // Calcular el centro de la celda en coordenadas (x, y)
        double center_x = (cell_x + 0.5) * cell_size_;
        double center_y = (cell_y + 0.5) * cell_size_;
        double center_z = ground_z; // Proyectado sobre el suelo

        // Crear un pose para el nodo de obstáculo
        geometry_msgs::msg::Pose pose;
        pose.position.x = center_x;
        pose.position.y = center_y;
        pose.position.z = center_z;
        // Orientación identidad
        pose.orientation.w = 1.0;
        obstacle_nodes.poses.push_back(pose);

        // Crear un marker para visualizar el nodo de obstáculo
        visualization_msgs::msg::Marker marker;
        marker.header = msg->header;
        marker.ns = "obstacle_nodes";
        marker.id = marker_id++;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = pose;
        marker.scale.x = cell_size_;
        marker.scale.y = cell_size_;
        marker.scale.z = 0.1; // Altura visual pequeña
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.lifetime = rclcpp::Duration(0, 0);
        markers.markers.push_back(marker);
      }
    }

    // Publicar los nodos de obstáculo y los markers de visualización
    obstacle_nodes_pub_->publish(obstacle_nodes);
    marker_pub_->publish(markers);

    // Liberar la memoria del árbol
    delete tree;
  }

  // Parámetros configurables
  double obstacle_height_threshold_; // Diferencia mínima de altura respecto al suelo para considerar un obstáculo
  double cell_size_;                 // Tamaño de la celda para agrupar puntos (en metros)

  // Suscripción al tópico Octomap
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;

  // Publicadores para los nodos de obstáculo y visualización
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr obstacle_nodes_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OctomapNavigationNodes>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
