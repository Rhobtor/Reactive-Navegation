#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <vector>
#include <memory>
#include <string>
#include <cmath>

// Función que recorre el octree y extrae las coordenadas de los nodos libres.
std::vector<octomap::point3d> extractFreeNodes(const octomap::OcTree &tree) {
  std::vector<octomap::point3d> nodes;
  for (octomap::OcTree::leaf_iterator it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it) {
    // Consideramos el nodo libre si no está ocupado
    if (!tree.isNodeOccupied(*it)) {
      nodes.push_back(it.getCoordinate());
    }
  }
  return nodes;
}

class NavigationNode : public rclcpp::Node {
public:
  NavigationNode()
  : Node("navigation_node"),
    tf_buffer_(this->get_clock()),
    min_distance_(0.0),
    max_distance_(5.0)
  {
    // Parámetros de filtrado por distancia (respecto a la posición del vehículo)
    this->declare_parameter("min_distance", 0.0);
    this->declare_parameter("max_distance", 5.0);
    min_distance_ = this->get_parameter("min_distance").as_double();
    max_distance_ = this->get_parameter("max_distance").as_double();

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("navigation_nodes", 10);
    octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      "/octomap_full",
      10,
      std::bind(&NavigationNode::octomapCallback, this, std::placeholders::_1)
    );
    // Crear el listener para obtener transformaciones
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_, this);

    RCLCPP_INFO(this->get_logger(), "Nodo de navegación iniciado.");
  }

private:
  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
    // Obtener la posición actual del vehículo en el frame "map" usando el transform de "base_link"
    geometry_msgs::msg::TransformStamped transformStamped;
    try {
      // Se obtiene la transformación de "base_link" al frame "map"
      rclcpp::Time query_time(0, 0, this->get_clock()->get_clock_type());
      transformStamped = tf_buffer_.lookupTransform("map", "base_link", query_time, rclcpp::Duration::from_seconds(0.1));

    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "No se pudo obtener la transformación: %s", ex.what());
      return;
    }

    double robot_x = transformStamped.transform.translation.x;
    double robot_y = transformStamped.transform.translation.y;
    double robot_z = transformStamped.transform.translation.z;

    // Convertir el mensaje Octomap a un OcTree
    octomap::AbstractOcTree* tree_raw = octomap_msgs::fullMsgToMap(*msg);
    if (!tree_raw) {
      RCLCPP_ERROR(this->get_logger(), "Error al convertir el mensaje Octomap a OcTree");
      return;
    }
    // Convertir a OcTree (asegurándose de que el árbol es del tipo OcTree)
    octomap::OcTree* tree_ptr = dynamic_cast<octomap::OcTree*>(tree_raw);
    if (!tree_ptr) {
      RCLCPP_ERROR(this->get_logger(), "El árbol obtenido no es de tipo OcTree");
      delete tree_raw;
      return;
    }
    // Usamos un smart pointer para gestionar el árbol
    std::shared_ptr<octomap::OcTree> tree(tree_ptr);

    // Extraer nodos libres (navegables)
    auto nodes = extractFreeNodes(*tree);
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;
    for (const auto &pt : nodes) {
      // Calcular la distancia entre el punto y la posición actual del vehículo (base_link en el frame map)
      double dx = pt.x() - robot_x;
      double dy = pt.y() - robot_y;
      double dz = pt.z() - robot_z;
      double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (distance < min_distance_ || distance > max_distance_) {
        continue;
      }
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = this->now();
      marker.ns = "navigation_nodes";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = pt.x();
      marker.pose.position.y = pt.y();
      marker.pose.position.z = pt.z();
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
      marker.lifetime = rclcpp::Duration(0, 0);
      marker_array.markers.push_back(marker);
    }
    marker_pub_->publish(marker_array);
    RCLCPP_INFO(this->get_logger(), "Markers publicados: %d", id);
  }

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  double min_distance_;
  double max_distance_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavigationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

