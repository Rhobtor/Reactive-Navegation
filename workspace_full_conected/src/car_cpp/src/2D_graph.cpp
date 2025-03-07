#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <vector>
#include <cmath>

// Estructura para almacenar un clúster de puntos (persistente en el mapa)
struct Cluster {
  geometry_msgs::msg::Pose centroid;
  int count;
};

class GroundPointsPublisher : public rclcpp::Node {
public:
  GroundPointsPublisher() : Node("ground_points_publisher")
  {
    // Suscripción a los nodos navegables filtrados (por ejemplo, del topic "ground_navigation_nodes")
    nodes_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "ground_navigation_nodes", 10,
      std::bind(&GroundPointsPublisher::nodesCallback, this, std::placeholders::_1)
    );
    
    // Publicador para los puntos agrupados (PoseArray) que se acumulan permanentemente
    ground_points_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("ground_points", 10);
    
    // Publicador para los markers en RViz
    markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("ground_points_markers", 10);
    
    // Parámetro para definir el umbral de agrupamiento (en metros)
    this->declare_parameter("grouping_threshold", 0.5);
    grouping_threshold_ = this->get_parameter("grouping_threshold").as_double();

    //RCLCPP_INFO(this->get_logger(), "GroundPointsPublisher iniciado con grouping_threshold: %f", grouping_threshold_);
  }

private:
  void nodesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    // No se limpia el vector 'clusters_' para mantener los puntos acumulados en el mapa.
    // Por cada punto recibido se agrupa con los existentes o se agrega como nuevo.
    for (const auto &pose : msg->poses) {
      bool found = false;
      for (auto &cluster : clusters_) {
        double dx = pose.position.x - cluster.centroid.position.x;
        double dy = pose.position.y - cluster.centroid.position.y;
        double dz = pose.position.z - cluster.centroid.position.z;
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (dist < grouping_threshold_) {
          // Actualizar el centroide usando promedio ponderado
          cluster.centroid.position.x = (cluster.centroid.position.x * cluster.count + pose.position.x) / (cluster.count + 1);
          cluster.centroid.position.y = (cluster.centroid.position.y * cluster.count + pose.position.y) / (cluster.count + 1);
          cluster.centroid.position.z = (cluster.centroid.position.z * cluster.count + pose.position.z) / (cluster.count + 1);
          cluster.count++;
          found = true;
          break;
        }
      }
      if (!found) {
        Cluster new_cluster;
        new_cluster.centroid = pose;
        new_cluster.count = 1;
        clusters_.push_back(new_cluster);
      }
    }
    
    // Publicar los puntos agrupados (los centroides) en un PoseArray.
    geometry_msgs::msg::PoseArray aggregated_msg;
    // Usamos un header fijo o el del mensaje recibido, siempre en el frame "map"
    aggregated_msg.header.frame_id = "map";
    aggregated_msg.header.stamp = this->now();
    for (const auto &cluster : clusters_) {
      aggregated_msg.poses.push_back(cluster.centroid);
    }
    ground_points_pub_->publish(aggregated_msg);
    
    // Crear markers para visualizar cada clúster en RViz
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;
    for (const auto &cluster : clusters_) {
      visualization_msgs::msg::Marker marker;
      marker.header = aggregated_msg.header;
      marker.ns = "ground_clusters";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose = cluster.centroid;
      marker.scale.x = 0.2;
      marker.scale.y = 0.2;
      marker.scale.z = 0.2;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
      // Lifetime 0 para que el marker permanezca en RViz
      marker.lifetime = rclcpp::Duration(0, 0);
      marker_array.markers.push_back(marker);
    }
    markers_pub_->publish(marker_array);
    //RCLCPP_INFO(this->get_logger(), "Puntos agrupados acumulados: %zu clusters", clusters_.size());
  }
  
  double grouping_threshold_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr nodes_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr ground_points_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  
  // Variable persistente para acumular los puntos (clusters) del mapa
  std::vector<Cluster> clusters_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GroundPointsPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
