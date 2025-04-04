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
#include <algorithm>
#include <utility>

using std::placeholders::_1;

// Parámetros ajustables
constexpr double SAFE_DISTANCE_THRESHOLD = 3;      // Distancia mínima permitida entre centroide y obstáculo (m)
constexpr double CLUSTER_DISTANCE_THRESHOLD = 4;     // Distancia máxima para agrupar puntos en un mismo cluster

// Función para calcular la distancia euclidiana entre dos puntos
double euclideanDistance(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2) {
  return std::hypot(p2.x - p1.x, p2.y - p1.y);
}

class FrontierBoundaryNode : public rclcpp::Node {
public:
  FrontierBoundaryNode() : Node("frontier_boundary_node")
  {
    // Suscribirse al mapa proyectado (OccupancyGrid)
    occupancy_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/projected_map", 10,
      std::bind(&FrontierBoundaryNode::occupancyCallback, this, _1)
    );
    // Suscribirse a los nodos ocupados (obstáculos)
    occupied_nodes_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/occupied_rejected_nodes", 10,
      std::bind(&FrontierBoundaryNode::occupiedNodesCallback, this, _1)
    );
    // Publicadores originales
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("frontier_marker", 10);
    frontier_points_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("frontier_points", 10);
    frontier_entropy_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("frontier_entropies", 10);
    total_entropy_pub_ = this->create_publisher<std_msgs::msg::Float64>("total_entropy", 10);
    // Publicadores para safe frontier
    safe_frontier_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("safe_frontier_points", 10);
    safe_frontier_entropy_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("safe_frontier_entropy", 10);
    safe_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("safe_frontier_marker", 10);
    total_safe_entropy_pub_ = this->create_publisher<std_msgs::msg::Float64>("total_safe_entropy", 10);

    RCLCPP_INFO(this->get_logger(), "Nodo de frontera basado en projected_map iniciado.");
  }

private:
  // Suscriptores
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr occupied_nodes_sub_;
  // Publicadores originales
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr frontier_points_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr frontier_entropy_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr total_entropy_pub_;
  // Publicadores para safe frontier
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr safe_frontier_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr safe_frontier_entropy_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr safe_marker_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr total_safe_entropy_pub_;

  // Variables para almacenar datos
  nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_grid_;
  geometry_msgs::msg::PoseArray::SharedPtr occupied_nodes_;

  // Función para calcular la entropía de una celda
  double computeCellEntropy(int8_t cell_value) {
    double p;
    if (cell_value == -1) {
      p = 0.5;
    } else {
      p = static_cast<double>(cell_value) / 100.0;
    }
    if (p <= 0.0 || p >= 1.0)
      return 0.0;
    return -(p * std::log(p) + (1 - p) * std::log(1 - p));
  }

  void occupancyCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    occupancy_grid_ = msg;
    processOccupancyGrid(msg);
  }

  void occupiedNodesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    occupied_nodes_ = msg;
  }

  void processOccupancyGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    int width = msg->info.width;
    int height = msg->info.height;
    double resolution = msg->info.resolution;
    double origin_x = msg->info.origin.position.x;
    double origin_y = msg->info.origin.position.y;

    std::vector<geometry_msgs::msg::Point> frontier_points;
    std::vector<double> frontier_local_entropies;

    // Calcular la entropía total del mapa
    double total_entropy_sum = 0.0;
    for (size_t i = 0; i < msg->data.size(); ++i) {
      total_entropy_sum += computeCellEntropy(msg->data[i]);
    }
    double total_entropy = total_entropy_sum / msg->data.size();

    // Recorrer la grilla para detectar celdas frontera
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        int index = y * width + x;
        int8_t cell_value = msg->data[index];
        if (cell_value == 0) {  // celda libre
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
            geometry_msgs::msg::Point pt;
            pt.x = origin_x + (x + 0.5) * resolution;
            pt.y = origin_y + (y + 0.5) * resolution;
            pt.z = 0.0;
            frontier_points.push_back(pt);

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

    // Publicar los puntos frontera originales (PoseArray)
    geometry_msgs::msg::PoseArray frontier_poses;
    frontier_poses.header = msg->header;
    for (const auto &pt : frontier_points) {
      geometry_msgs::msg::Pose pose;
      pose.position = pt;
      pose.orientation.w = 1.0;
      frontier_poses.poses.push_back(pose);
    }
    frontier_points_pub_->publish(frontier_poses);

    // Publicar marcador para visualizar los puntos frontera originales
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = msg->header.frame_id;
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

    // Publicar entropías locales y total
    std_msgs::msg::Float64MultiArray entropy_array_msg;
    entropy_array_msg.data = frontier_local_entropies;
    frontier_entropy_pub_->publish(entropy_array_msg);

    std_msgs::msg::Float64 total_entropy_msg;
    total_entropy_msg.data = total_entropy;
    total_entropy_pub_->publish(total_entropy_msg);
    RCLCPP_INFO(this->get_logger(), "Entropía total del mapa: %.3f", total_entropy);

    // Combinar posición y entropía en un solo vector
    std::vector<std::pair<geometry_msgs::msg::Point, double>> frontier_points_with_entropy;
    if (frontier_points.size() == frontier_local_entropies.size()) {
      for (size_t i = 0; i < frontier_points.size(); ++i) {
        frontier_points_with_entropy.push_back(std::make_pair(frontier_points[i], frontier_local_entropies[i]));
      }
    }
    // Clustering y publicación de safe frontier points y sus entropías
    clusterAndPublishFrontiers(frontier_points_with_entropy);
  }

  // Función de clustering que trabaja con pares (punto, entropía)
  void clusterAndPublishFrontiers(const std::vector<std::pair<geometry_msgs::msg::Point, double>> &frontier_points_with_entropy) {
    // Agrupar en clusters simples
    std::vector<std::vector<std::pair<geometry_msgs::msg::Point, double>>> clusters;
    for (const auto &pt_pair : frontier_points_with_entropy) {
      const auto &pt = pt_pair.first;
      bool added = false;
      for (auto &cluster : clusters) {
        geometry_msgs::msg::Point centroid = computeCentroidFromPairs(cluster);
        if (euclideanDistance(pt, centroid) < CLUSTER_DISTANCE_THRESHOLD) {
          cluster.push_back(pt_pair);
          added = true;
          break;
        }
      }
      if (!added) {
        clusters.push_back({pt_pair});
      }
    }

    // Calcular centroide y entropía promedio de cada cluster
    std::vector<geometry_msgs::msg::Point> safe_centroids;
    std::vector<double> safe_entropies;
    for (const auto &cluster : clusters) {
      geometry_msgs::msg::Point centroid = computeCentroidFromPairs(cluster);
      double sum_entropy = 0.0;
      for (const auto &pair : cluster) {
        sum_entropy += pair.second;
      }
      double avg_entropy = sum_entropy / cluster.size();
      // Filtrar el centroide si está muy cerca de algún obstáculo
      bool safe = true;
      if (occupied_nodes_ != nullptr) {
        for (const auto &obs_pose : occupied_nodes_->poses) {
          if (euclideanDistance(centroid, obs_pose.position) < SAFE_DISTANCE_THRESHOLD) {
            safe = false;
            break;
          }
        }
      }
      if (safe) {
        safe_centroids.push_back(centroid);
        safe_entropies.push_back(avg_entropy);
      }
    }

    // Publicar safe frontier points como PoseArray
    geometry_msgs::msg::PoseArray safe_frontier_poses;
    safe_frontier_poses.header.stamp = this->now();
    safe_frontier_poses.header.frame_id = "map";
    for (const auto &pt : safe_centroids) {
      geometry_msgs::msg::Pose pose;
      pose.position = pt;
      pose.orientation.w = 1.0;
      safe_frontier_poses.poses.push_back(pose);
    }
    safe_frontier_pub_->publish(safe_frontier_poses);

    // Publicar marcador para safe frontier points (usando un publicador separado)
    visualization_msgs::msg::Marker safe_marker;
    safe_marker.header.frame_id = "map";
    safe_marker.header.stamp = this->now();
    safe_marker.ns = "safe_frontier";
    safe_marker.id = 1;
    safe_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    safe_marker.action = visualization_msgs::msg::Marker::ADD;
    safe_marker.scale.x = 0.2;
    safe_marker.scale.y = 0.2;
    safe_marker.scale.z = 0.2;
    safe_marker.color.r = 0.0;
    safe_marker.color.g = 1.0;
    safe_marker.color.b = 0.0;
    safe_marker.color.a = 1.0;
    for (const auto &pt : safe_centroids) {
      safe_marker.points.push_back(pt);
    }
    safe_marker_pub_->publish(safe_marker);

    // Publicar las entropías de los safe frontier points
    std_msgs::msg::Float64MultiArray safe_entropy_array;
    safe_entropy_array.data = safe_entropies;
    safe_frontier_entropy_pub_->publish(safe_entropy_array);

    // Calcular y publicar la entropía total de los safe frontier points (promedio)
    double total_safe_entropy = 0.0;
    if (!safe_entropies.empty()) {
      for (double e : safe_entropies) {
        total_safe_entropy += e;
      }
      total_safe_entropy /= safe_entropies.size();
    }
    std_msgs::msg::Float64 total_safe_entropy_msg;
    total_safe_entropy_msg.data = total_safe_entropy;
    total_safe_entropy_pub_->publish(total_safe_entropy_msg);

    RCLCPP_INFO(this->get_logger(), "Se publicaron %zu centroides seguros de %zu puntos frontera.",
                safe_centroids.size(), frontier_points_with_entropy.size());
  }

  // Función para calcular el centroide de un vector de pares (punto, entropía)
  geometry_msgs::msg::Point computeCentroidFromPairs(const std::vector<std::pair<geometry_msgs::msg::Point, double>> &pairs) {
    geometry_msgs::msg::Point centroid;
    double sum_x = 0.0, sum_y = 0.0;
    for (const auto &pair : pairs) {
      sum_x += pair.first.x;
      sum_y += pair.first.y;
    }
    centroid.x = sum_x / pairs.size();
    centroid.y = sum_y / pairs.size();
    centroid.z = 0.0;
    return centroid;
  }

  // Función para calcular el centroide de un vector de puntos
  geometry_msgs::msg::Point computeCentroid(const std::vector<geometry_msgs::msg::Point> &points) {
    geometry_msgs::msg::Point centroid;
    double sum_x = 0.0, sum_y = 0.0;
    for (const auto &pt : points) {
      sum_x += pt.x;
      sum_y += pt.y;
    }
    centroid.x = sum_x / points.size();
    centroid.y = sum_y / points.size();
    centroid.z = 0.0;
    return centroid;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FrontierBoundaryNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
