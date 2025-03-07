#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <vector>
#include <cmath>

class FrontierBoundaryNode : public rclcpp::Node {
public:
  FrontierBoundaryNode()
  : Node("frontier_boundary_node")
  {
    // Suscribirse al mapa proyectado
    occupancy_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/projected_map", 10,
      std::bind(&FrontierBoundaryNode::occupancyCallback, this, std::placeholders::_1)
    );
    // Publicador para los marcadores que representan la frontera
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("frontier_marker", 10);
    // Publicador para las coordenadas de los puntos frontera (lista de poses)
    frontier_points_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("frontier_points", 10);

    RCLCPP_INFO(this->get_logger(), "Nodo de frontera basado en projected_map iniciado.");
  }

private:
  void occupancyCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    int width = msg->info.width;
    int height = msg->info.height;
    double resolution = msg->info.resolution;
    double origin_x = msg->info.origin.position.x;
    double origin_y = msg->info.origin.position.y;

    std::vector<geometry_msgs::msg::Point> frontier_points;

    // Se recorre la grilla del mapa (se asume que el mapa es 2D)
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        int index = y * width + x;
        int8_t cell_value = msg->data[index];

        // Consideramos una celda como fronteriza si es libre (valor 0)
        // y tiene al menos un vecino desconocido (valor -1)
        if (cell_value == 0) {
          bool is_frontier = false;
          for (int dy = -1; dy <= 1 && !is_frontier; ++dy) {
            for (int dx = -1; dx <= 1 && !is_frontier; ++dx) {
              if (dx == 0 && dy == 0)
                continue;
              int nx = x + dx;
              int ny = y + dy;
              if (nx < 0 || nx >= width || ny < 0 || ny >= height)
                continue;
              int n_index = ny * width + nx;
              if (msg->data[n_index] == -1) {
                is_frontier = true;
              }
            }
          }
          if (is_frontier) {
            // Calcular la posición del centro de la celda en coordenadas del mapa
            geometry_msgs::msg::Point pt;
            pt.x = origin_x + (x + 0.5) * resolution;
            pt.y = origin_y + (y + 0.5) * resolution;
            pt.z = 0.0; // Suponiendo un mapa 2D
            frontier_points.push_back(pt);
          }
        }
      }
    }

    // Crear el marcador de tipo SPHERE_LIST para visualizar la frontera
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = msg->header.frame_id;  // Debe ser el mismo frame del mapa, normalmente "map"
    marker.header.stamp = this->now();
    marker.ns = "frontier_boundary";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    // Ajustamos la escala para cada esfera (puedes modificarla según tus necesidades)
    marker.scale.x = resolution;
    marker.scale.y = resolution;
    marker.scale.z = resolution;
    // Color rojo para la frontera
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    // Duración infinita (si se desea actualización continua, el nodo publicará de nuevo)
    marker.lifetime = rclcpp::Duration(0, 0);

    marker.points = frontier_points;

    marker_pub_->publish(marker);
    RCLCPP_INFO(this->get_logger(), "Puntos de frontera publicados en marcador: %zu", frontier_points.size());

    // Crear y publicar un PoseArray con las coordenadas de los puntos frontera
    geometry_msgs::msg::PoseArray frontier_poses;
    frontier_poses.header = msg->header;  // Usamos el mismo header del mapa (frame y timestamp)
    for (const auto &pt : frontier_points) {
      geometry_msgs::msg::Pose pose;
      pose.position = pt;
      // Orientación nula (identidad) ya que solo interesa la posición
      pose.orientation.w = 1.0;
      pose.orientation.x = 0.0;
      pose.orientation.y = 0.0;
      pose.orientation.z = 0.0;
      frontier_poses.poses.push_back(pose);
    }

    frontier_points_pub_->publish(frontier_poses);
    RCLCPP_INFO(this->get_logger(), "Puntos de frontera publicados en PoseArray: %zu", frontier_poses.poses.size());
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr frontier_points_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FrontierBoundaryNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
