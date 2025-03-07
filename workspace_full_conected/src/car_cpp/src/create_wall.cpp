// persistent_wall_generator.cpp
// Este nodo acumula puntos de "occupied_rejected_nodes" durante un período de tiempo configurable
// para mantener la información del muro virtual de forma persistente. Se realiza clustering sobre
// los puntos acumulados y se calcula la envolvente convexa (convex hull) de cada grupo.
// El resultado se publica como un MarkerArray en el tópico "persistent_wall_markers" para visualizarlo en RViz.

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <algorithm>
#include <vector>
#include <cmath>

// Estructura auxiliar para representar un punto 2D.
struct Point2D {
  double x, y;
};

// Función auxiliar: cálculo del producto cruzado, usado en el algoritmo de convex hull.
double cross(const Point2D &O, const Point2D &A, const Point2D &B) {
  return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
}

// Función para calcular la envolvente convexa (convex hull) de un conjunto de puntos 2D usando el algoritmo de Andrew.
std::vector<geometry_msgs::msg::Point> computeConvexHull(const std::vector<geometry_msgs::msg::Point>& points) {
  std::vector<Point2D> pts;
  for (const auto &p : points) {
    pts.push_back({p.x, p.y});
  }
  std::sort(pts.begin(), pts.end(), [](const Point2D &a, const Point2D &b) {
    return (a.x < b.x) || (a.x == b.x && a.y < b.y);
  });
  std::vector<Point2D> lower;
  for (const auto &p : pts) {
    while (lower.size() >= 2 && cross(lower[lower.size()-2], lower[lower.size()-1], p) <= 0)
      lower.pop_back();
    lower.push_back(p);
  }
  std::vector<Point2D> upper;
  for (int i = pts.size()-1; i >= 0; i--) {
    const auto &p = pts[i];
    while (upper.size() >= 2 && cross(upper[upper.size()-2], upper[upper.size()-1], p) <= 0)
      upper.pop_back();
    upper.push_back(p);
  }
  if (!upper.empty()) upper.pop_back();
  if (!lower.empty()) lower.pop_back();
  std::vector<geometry_msgs::msg::Point> hull;
  for (const auto &p : lower) {
    geometry_msgs::msg::Point pt;
    pt.x = p.x; pt.y = p.y; pt.z = 0.0;
    hull.push_back(pt);
  }
  for (const auto &p : upper) {
    geometry_msgs::msg::Point pt;
    pt.x = p.x; pt.y = p.y; pt.z = 0.0;
    hull.push_back(pt);
  }
  return hull;
}

// Estructura para almacenar un cluster de puntos (poses).
struct Cluster {
  std::vector<geometry_msgs::msg::Pose> poses;
};

class PersistentWallGenerator : public rclcpp::Node {
public:
  PersistentWallGenerator() : Node("persistent_wall_generator")
  {
    // Declarar parámetros configurables.
    this->declare_parameter("cluster_distance_threshold", 0.3);
    cluster_distance_threshold_ = this->get_parameter("cluster_distance_threshold").as_double();
    this->declare_parameter("persistence_duration", 5.0);
    persistence_duration_ = this->get_parameter("persistence_duration").as_double();

    // Suscribirse al tópico "occupied_rejected_nodes".
    occupied_nodes_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "occupied_rejected_nodes", 10,
      std::bind(&PersistentWallGenerator::occupiedNodesCallback, this, std::placeholders::_1)
    );

    // Publicador para visualizar el muro virtual como MarkerArray.
    wall_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("persistent_wall_markers", 10);

    RCLCPP_INFO(this->get_logger(), "PersistentWallGenerator iniciado. Umbral clustering: %.2f, Duración de persistencia: %.2f s",
                cluster_distance_threshold_, persistence_duration_);
  }

private:
  // Usamos std::pair, donde first es la pose y second el timestamp.
  std::vector<std::pair<geometry_msgs::msg::Pose, rclcpp::Time>> accumulated_points_;

  // Callback para acumular puntos y eliminar los antiguos.
  void occupiedNodesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    rclcpp::Time current_time = this->now();

    // Añadir nuevos puntos a la lista acumulada.
    for (const auto &pose : msg->poses) {
      accumulated_points_.push_back({pose, current_time});
    }

    // Eliminar puntos más antiguos que persistence_duration_.
    accumulated_points_.erase(
      std::remove_if(accumulated_points_.begin(), accumulated_points_.end(),
                     [current_time, this](const std::pair<geometry_msgs::msg::Pose, rclcpp::Time>& tp) {
                       return (current_time - tp.second).seconds() > persistence_duration_;
                     }),
      accumulated_points_.end()
    );

    // Extraer las poses acumuladas.
    std::vector<geometry_msgs::msg::Pose> all_poses;
    for (const auto &tp : accumulated_points_) {
      all_poses.push_back(tp.first);  // Usar 'first' en lugar de 'pose'
    }

    // Realizar clustering sobre los puntos acumulados.
    std::vector<Cluster> clusters = performClustering(all_poses);

    // Crear un MarkerArray para visualizar los muros virtuales.
    visualization_msgs::msg::MarkerArray marker_array;
    int marker_id = 0;
    int wall_count = 0;

    // Para cada cluster significativo (al menos 2 puntos), calcular el convex hull.
    for (const auto &cluster : clusters) {
      if (cluster.poses.size() < 2)
        continue; // Ignorar clusters pequeños.

      // Extraer puntos 2D (x,y) de las poses.
      std::vector<geometry_msgs::msg::Point> points;
      for (const auto &pose : cluster.poses) {
        geometry_msgs::msg::Point pt;
        pt.x = pose.position.x;
        pt.y = pose.position.y;
        pt.z = 0.0;
        points.push_back(pt);
      }

      // Calcular la envolvente convexa.
      std::vector<geometry_msgs::msg::Point> hull = computeConvexHull(points);
      if (hull.size() < 3)
        continue;

      // Crear un marker de tipo LINE_STRIP para dibujar el contorno del muro.
      visualization_msgs::msg::Marker marker;
      marker.header = msg->header;
      marker.ns = "persistent_wall";
      marker.id = marker_id++;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = 0.05;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
      marker.lifetime = rclcpp::Duration(0, 0);

      for (const auto &pt : hull) {
        marker.points.push_back(pt);
      }
      // Cerrar el contorno.
      marker.points.push_back(hull.front());

      marker_array.markers.push_back(marker);
      wall_count++;
    }

    wall_markers_pub_->publish(marker_array);
    RCLCPP_INFO(this->get_logger(), "Muros virtuales persistentes: %d clusters procesados (acumulados %zu puntos).",
                wall_count, accumulated_points_.size());
  }

  // Algoritmo de clustering simple: agrupa puntos (poses) que estén a menos de 'cluster_distance_threshold_'
  // en el plano XY.
  std::vector<Cluster> performClustering(const std::vector<geometry_msgs::msg::Pose>& poses) {
    std::vector<Cluster> clusters;
    for (const auto &pose : poses) {
      bool added = false;
      for (auto &cluster : clusters) {
        geometry_msgs::msg::Pose centroid = computeCentroid(cluster.poses);
        double dx = pose.position.x - centroid.position.x;
        double dy = pose.position.y - centroid.position.y;
        double dist = std::sqrt(dx * dx + dy * dy);
        if (dist < cluster_distance_threshold_) {
          cluster.poses.push_back(pose);
          added = true;
          break;
        }
      }
      if (!added) {
        Cluster new_cluster;
        new_cluster.poses.push_back(pose);
        clusters.push_back(new_cluster);
      }
    }
    return clusters;
  }

  // Calcula el centroide de un vector de poses.
  geometry_msgs::msg::Pose computeCentroid(const std::vector<geometry_msgs::msg::Pose>& poses) {
    geometry_msgs::msg::Pose centroid;
    if (poses.empty()) {
      centroid.position.x = 0.0;
      centroid.position.y = 0.0;
      centroid.position.z = 0.0;
      centroid.orientation.w = 1.0;
      return centroid;
    }
    double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    for (const auto &pose : poses) {
      sum_x += pose.position.x;
      sum_y += pose.position.y;
      sum_z += pose.position.z;
    }
    centroid.position.x = sum_x / poses.size();
    centroid.position.y = sum_y / poses.size();
    centroid.position.z = sum_z / poses.size();
    centroid.orientation.w = 1.0;
    return centroid;
  }

  double cluster_distance_threshold_;
  double persistence_duration_;

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr occupied_nodes_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wall_markers_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PersistentWallGenerator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
