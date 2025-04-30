#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <vector>
#include <cmath>

class FrontierBoundaryNode : public rclcpp::Node {
public:
  FrontierBoundaryNode() : Node("frontier_boundary_node")
  {
    // Suscribirse al mapa proyectado
    occupancy_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/projected_map", 10,
      std::bind(&FrontierBoundaryNode::occupancyCallback, this, std::placeholders::_1)
    );
    // Publicador para los marcadores de la frontera
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("frontier_marker", 10);
    // Publicador para las coordenadas de los puntos frontera (lista de poses)
    frontier_points_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("frontier_points", 10);
    // Publicador para las entropías locales de cada punto frontera
    frontier_entropy_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("frontier_entropies", 10);
    // Publicador para la entropía total del mapa
    total_entropy_pub_ = this->create_publisher<std_msgs::msg::Float64>("total_entropy", 10);

    RCLCPP_INFO(this->get_logger(), "Nodo de frontera basado en projected_map iniciado.");
  }

private:
  // Función para convertir el valor de una celda a probabilidad y calcular su entropía.
  double computeCellEntropy(int8_t cell_value) {
    double p;
    if (cell_value == -1) {
      // Desconocido: asignamos p = 0.5
      p = 0.5;
    } else {
      p = static_cast<double>(cell_value) / 100.0;
    }
    // Evitamos problemas en extremos
    if (p <= 0.0 || p >= 1.0)
      return 0.0;
    return -(p * std::log(p) + (1 - p) * std::log(1 - p));
  }

  void occupancyCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    int width = msg->info.width;
    int height = msg->info.height;
    double resolution = msg->info.resolution;
    double origin_x = msg->info.origin.position.x;
    double origin_y = msg->info.origin.position.y;

    std::vector<geometry_msgs::msg::Point> frontier_points;
    std::vector<double> frontier_local_entropies;

    // Primero, calcular la entropía total (promedio) de todo el mapa.
    double total_entropy_sum = 0.0;
    for (size_t i = 0; i < msg->data.size(); ++i) {
      total_entropy_sum += computeCellEntropy(msg->data[i]);
    }
    double total_entropy = total_entropy_sum / msg->data.size();

    // Se recorre la grilla del mapa (se asume que el mapa es 2D)
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        int index = y * width + x;
        int8_t cell_value = msg->data[index];

        // Consideramos una celda como frontera si es libre (valor 0)
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

            // Calcular la entropía local para esta celda considerando una ventana 3x3
            double local_entropy_sum = 0.0;
            int count = 0;
            for (int dy = -1; dy <= 1; ++dy) {
              for (int dx = -1; dx <= 1; ++dx) {
                int nx = x + dx;
                int ny = y + dy;
                if (nx < 0 || nx >= width || ny < 0 || ny >= height)
                  continue;
                int n_index = ny * width + nx;
                local_entropy_sum += computeCellEntropy(msg->data[n_index]);
                ++count;
              }
            }
            double local_entropy = (count > 0) ? local_entropy_sum / count : 0.0;
            frontier_local_entropies.push_back(local_entropy);
          }
        }
      }
    }

    // Publicar el marcador para visualizar la frontera
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = msg->header.frame_id;  // Normalmente "map"
    marker.header.stamp = this->now();
    marker.ns = "frontier_boundary";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = resolution;
    marker.scale.y = resolution;
    marker.scale.z = resolution;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.lifetime = rclcpp::Duration(0, 0);
    marker.points = frontier_points;
    marker_pub_->publish(marker);
    //RCLCPP_INFO(this->get_logger(), "Marcador de frontera publicado (puntos: %zu)", frontier_points.size());

    // Publicar el PoseArray con las coordenadas de los puntos frontera
    geometry_msgs::msg::PoseArray frontier_poses;
    frontier_poses.header = msg->header;
    for (const auto &pt : frontier_points) {
      geometry_msgs::msg::Pose pose;
      pose.position = pt;
      pose.orientation.w = 1.0;
      frontier_poses.poses.push_back(pose);
    }
    frontier_points_pub_->publish(frontier_poses);
    //RCLCPP_INFO(this->get_logger(), "PoseArray de frontera publicado (puntos: %zu)", frontier_poses.poses.size());

    // Publicar las entropías locales de cada punto frontera
    std_msgs::msg::Float64MultiArray entropy_array_msg;
    entropy_array_msg.data = frontier_local_entropies;
    frontier_entropy_pub_->publish(entropy_array_msg);
    //RCLCPP_INFO(this->get_logger(), "Entropías locales publicadas (puntos: %zu)", frontier_local_entropies.size());

    // Publicar la entropía total del mapa
    std_msgs::msg::Float64 total_entropy_msg;
    total_entropy_msg.data = total_entropy;
    total_entropy_pub_->publish(total_entropy_msg);
    RCLCPP_INFO(this->get_logger(), "Entropía total del mapa: %.3f", total_entropy);
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr frontier_points_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr frontier_entropy_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr total_entropy_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FrontierBoundaryNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
