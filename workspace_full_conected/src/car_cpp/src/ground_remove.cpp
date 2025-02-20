#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "octomap_msgs/conversions.h"
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class OctomapGroundFilter : public rclcpp::Node {
public:
  OctomapGroundFilter()
  : Node("octomap_ground_filter"), ground_threshold_(0.3), frame_id_("map")
  {
    // Suscribirse al mapa octomap completo
    subscription_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      "/octomap_full", 10,
      std::bind(&OctomapGroundFilter::octomapCallback, this, std::placeholders::_1));
    // Publicar el mapa de ocupación 2D filtrado
    publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/occupancy_grid_filtered", 10);
    RCLCPP_INFO(this->get_logger(), "Nodo de filtro de suelo iniciado.");
  }

private:
  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
    // Convertir el mensaje Octomap a un objeto OcTree
    std::shared_ptr<octomap::OcTree> tree(dynamic_cast<octomap::OcTree*>(octomap_msgs::fullMsgToMap(*msg)));
    if (!tree) {
      RCLCPP_ERROR(this->get_logger(), "No se pudo convertir el mensaje a OcTree");
      return;
    }

    // Obtener los límites del mapa
    double min_x, min_y, min_z;
    double max_x, max_y, max_z;
    tree->getMetricMin(min_x, min_y, min_z);
    tree->getMetricMax(max_x, max_y, max_z);

    // Configurar los parámetros del OccupancyGrid
    double resolution = tree->getResolution();
    int width = static_cast<int>((max_x - min_x) / resolution);
    int height = static_cast<int>((max_y - min_y) / resolution);

    nav_msgs::msg::OccupancyGrid occupancy_grid;
    occupancy_grid.header.stamp = this->now();
    occupancy_grid.header.frame_id = frame_id_;
    occupancy_grid.info.resolution = resolution;
    occupancy_grid.info.width = width;
    occupancy_grid.info.height = height;
    occupancy_grid.info.origin.position.x = min_x;
    occupancy_grid.info.origin.position.y = min_y;
    occupancy_grid.info.origin.position.z = 0.0;
    occupancy_grid.info.origin.orientation.w = 1.0;
    occupancy_grid.data.resize(width * height, -1); // Inicializar con -1 (desconocido)

    // Rellenar el OccupancyGrid con los datos del OctoMap
    for (octomap::OcTree::leaf_iterator it = tree->begin_leafs(), end = tree->end_leafs();
         it != end; ++it)
    {
      octomap::point3d coord = it.getCoordinate();
      if (coord.z() < ground_threshold_) {
        // Ignorar nodos por debajo del umbral del suelo
        continue;
      }
      if (tree->isNodeOccupied(*it)) {
        int x = static_cast<int>((coord.x() - min_x) / resolution);
        int y = static_cast<int>((coord.y() - min_y) / resolution);
        int index = x + y * width;
        occupancy_grid.data[index] = 100; // Ocupado
      }
    }

    // Publicar el OccupancyGrid filtrado
    publisher_->publish(occupancy_grid);
    RCLCPP_INFO(this->get_logger(), "Publicado OccupancyGrid filtrado");
  }

  double ground_threshold_;
  std::string frame_id_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OctomapGroundFilter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
