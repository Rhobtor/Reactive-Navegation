#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <vector>
#include <cmath>

class GroundNavigationNodes : public rclcpp::Node {
public:
  GroundNavigationNodes() : Node("ground_navigation_nodes")
  {
    // Declarar parámetros configurables
    this->declare_parameter("ground_z_threshold", 0.3);
    this->declare_parameter("connection_distance_threshold", 1.0);
    ground_z_threshold_ = this->get_parameter("ground_z_threshold").as_double();
    connection_distance_threshold_ = this->get_parameter("connection_distance_threshold").as_double();

    // Suscribirse al topic que contiene los nodos de navegación (PoseArray)
    navigable_nodes_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "navigable_nodes", 10,
      std::bind(&GroundNavigationNodes::nodesCallback, this, std::placeholders::_1)
    );

    // Publicador para los nodos filtrados (cercanos al suelo)
    ground_nodes_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("ground_navigation_nodes", 10);

    // Publicador para visualizar los nodos filtrados con markers
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("ground_nodes_markers", 10);

    // Publicador para visualizar el grafo 2D construido (aristas)
    graph_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("ground_nodes_graph", 10);

    // RCLCPP_INFO(this->get_logger(), "Nodo GroundNavigationNodes iniciado. Umbral en z: %.2f | Distancia de conexión: %.2f",
    //             ground_z_threshold_, connection_distance_threshold_);
  }

private:
  void nodesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    // Filtrar los nodos para quedarse solo con aquellos cuyo valor z es menor o igual al umbral
    geometry_msgs::msg::PoseArray ground_nodes;
    ground_nodes.header = msg->header;

    visualization_msgs::msg::MarkerArray markers;
    int marker_id = 0;

    for (const auto &pose : msg->poses) {
      if (pose.position.z <= ground_z_threshold_) {
        ground_nodes.poses.push_back(pose);

        // Crear marker para visualizar el nodo filtrado
        visualization_msgs::msg::Marker marker;
        marker.header = msg->header;
        marker.ns = "ground_nodes";
        marker.id = marker_id++;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = pose;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;
        marker.lifetime = rclcpp::Duration(0, 0);
        markers.markers.push_back(marker);
      }
    }

    // Publicar los nodos filtrados y sus markers
    ground_nodes_pub_->publish(ground_nodes);
    marker_pub_->publish(markers);
    //RCLCPP_INFO(this->get_logger(), "Nodos filtrados: %zu de %zu", ground_nodes.poses.size(), msg->poses.size());

    // Construir el grafo 2D:
    // Para cada par de nodos filtrados, si la distancia euclidiana (en el plano x,y) es menor al umbral,
    // se añade una arista (se representa con dos puntos en un Marker de tipo LINE_LIST)
    visualization_msgs::msg::Marker graph_marker;
    graph_marker.header = msg->header;
    graph_marker.ns = "ground_graph";
    graph_marker.id = 0;
    graph_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    graph_marker.action = visualization_msgs::msg::Marker::ADD;
    graph_marker.scale.x = 0.05;  // Grosor de la línea
    graph_marker.color.r = 1.0;
    graph_marker.color.g = 1.0;
    graph_marker.color.b = 0.0;
    graph_marker.color.a = 1.0;

    size_t edge_count = 0;
    for (size_t i = 0; i < ground_nodes.poses.size(); ++i) {
      for (size_t j = i + 1; j < ground_nodes.poses.size(); ++j) {
        double dx = ground_nodes.poses[i].position.x - ground_nodes.poses[j].position.x;
        double dy = ground_nodes.poses[i].position.y - ground_nodes.poses[j].position.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        if (distance <= connection_distance_threshold_) {
          // Agregar una arista conectando ambos nodos
          geometry_msgs::msg::Point p1 = ground_nodes.poses[i].position;
          geometry_msgs::msg::Point p2 = ground_nodes.poses[j].position;
          graph_marker.points.push_back(p1);
          graph_marker.points.push_back(p2);
          edge_count++;
        }
      }
    }

    graph_marker_pub_->publish(graph_marker);
    //RCLCPP_INFO(this->get_logger(), "Grafo 2D construido con %zu aristas.", edge_count);

    // Aquí podrías almacenar el grafo en una estructura (por ejemplo, una lista de adyacencia)
    // para que otro nodo lo use en planificación (como A*).
  }

  double ground_z_threshold_;
  double connection_distance_threshold_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr navigable_nodes_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr ground_nodes_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr graph_marker_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GroundNavigationNodes>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
