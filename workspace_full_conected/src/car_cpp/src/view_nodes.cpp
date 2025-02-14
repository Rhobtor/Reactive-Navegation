#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <octomap/octomap.h>

// Supongamos que tienes una función que recorre el octree y extrae los centros de celdas libres:
std::vector<octomap::point3d> extractFreeNodes(const octomap::OcTree &tree) {
  std::vector<octomap::point3d> nodes;
  for(octomap::OcTree::leaf_iterator it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it) {
    if(tree.isNodeOccupied(*it) == false) {
      nodes.push_back(it.getCoordinate());
    }
  }
  return nodes;
}

class NavigationNode : public rclcpp::Node {
public:
  NavigationNode() : Node("navigation_node") {
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("navigation_nodes", 10);
    // Aquí cargarías o subscribirías al Octomap
    // Luego extraerías los nodos libres y publicarías markers periódicamente.
    timer_ = this->create_wall_timer(std::chrono::seconds(1),
      std::bind(&NavigationNode::publishMarkers, this));
  }

private:
  void publishMarkers() {
    // Ejemplo: cargar un octree (en la práctica lo obtendrías desde tu mapa)
    octomap::OcTree tree(0.1);
    // ... actualizar el octree con los datos del LIDAR ...
    
    // Extraer nodos libres
    auto nodes = extractFreeNodes(tree);
    
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;
    for (const auto &pt : nodes) {
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
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker_array.markers.push_back(marker);
    }
    marker_pub_->publish(marker_array);
  }

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigationNode>());
  rclcpp::shutdown();
  return 0;
}
