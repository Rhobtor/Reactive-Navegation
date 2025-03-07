// occupied_rejected_nodes.cpp
// Este nodo suscribe a los tópicos "ground_navigation_nodes" y "obstacle_navigation_nodes".
// Para cada nodo de navegación del suelo, si se encuentra a una distancia menor a 
// "obstacle_projection_radius" respecto a algún obstáculo, se marca como rechazado.
// Los nodos rechazados se publican en el tópico "occupied_rejected_nodes", lo que
// implica que su probabilidad de ocupación es 1, permitiendo que influyan en el cálculo de entropía.

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cmath>

class OccupiedRejectedNodes : public rclcpp::Node
{
public:
  OccupiedRejectedNodes() : Node("occupied_rejected_nodes")
  {
    // Declarar parámetro configurable:
    // obstacle_projection_radius: radio de influencia del obstáculo sobre el suelo.
    // Si un nodo de navegación se encuentra dentro de este radio de algún obstáculo,
    // se considerará ocupado (rechazado).
    this->declare_parameter("obstacle_projection_radius", 0.5);
    obstacle_projection_radius_ = this->get_parameter("obstacle_projection_radius").as_double();

    // Suscribirse al tópico de nodos navegables del suelo.
    ground_nodes_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "ground_navigation_nodes", 10,
      std::bind(&OccupiedRejectedNodes::groundNodesCallback, this, std::placeholders::_1)
    );

    // Suscribirse al tópico de nodos de obstáculo.
    obstacle_nodes_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "obstacle_navigation_nodes", 10,
      std::bind(&OccupiedRejectedNodes::obstacleNodesCallback, this, std::placeholders::_1)
    );

    // Publicador para los nodos rechazados (se considerarán ocupados, es decir, probabilidad 1)
    occupied_nodes_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("occupied_rejected_nodes", 10);

    // Publicador para visualizar los nodos rechazados con markers (por ejemplo, color naranja)
    occupied_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("occupied_nodes_markers", 10);

    RCLCPP_INFO(this->get_logger(), "Nodo OccupiedRejectedNodes iniciado. Radio de proyección: %.2f",
                obstacle_projection_radius_);
  }

private:
  // Callback para recibir nodos de obstáculo.
  // Se almacena el mensaje recibido para utilizarlo al filtrar los nodos de navegación.
  void obstacleNodesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    obstacle_nodes_ = *msg;
  }

  // Callback para recibir los nodos navegables del suelo.
  // Se filtran aquellos nodos que estén muy cerca (dentro del radio definido)
  // de alguno de los nodos de obstáculo, y se publican como "ocupados".
  void groundNodesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    // PoseArray para almacenar los nodos rechazados (ocupados).
    geometry_msgs::msg::PoseArray occupied_nodes;
    occupied_nodes.header = msg->header;

    // MarkerArray para visualización de los nodos ocupados.
    visualization_msgs::msg::MarkerArray markers;
    int marker_id = 0;

    // Iterar sobre cada nodo navegable recibido.
    for (const auto &ground_pose : msg->poses)
    {
      bool rejected = false;
      // Verificar la distancia respecto a cada nodo obstáculo.
      for (const auto &obs_pose : obstacle_nodes_.poses)
      {
        double dx = ground_pose.position.x - obs_pose.position.x;
        double dy = ground_pose.position.y - obs_pose.position.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        // Si la distancia es menor que el radio de proyección, se rechaza el nodo.
        if (distance < obstacle_projection_radius_)
        {
          rejected = true;
          break;
        }
      }
      // Si el nodo se rechaza, se añade a la lista de nodos ocupados.
      if (rejected)
      {
        occupied_nodes.poses.push_back(ground_pose);

        // Crear marker para visualizar este nodo rechazado (por ejemplo, como esfera naranja).
        visualization_msgs::msg::Marker marker;
        marker.header = msg->header;
        marker.ns = "occupied_rejected_nodes";
        marker.id = marker_id++;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = ground_pose;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        // Color naranja: (1.0, 0.5, 0.0)
        marker.color.r = 1.0;
        marker.color.g = 0.5;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.lifetime = rclcpp::Duration(0, 0);
        markers.markers.push_back(marker);
      }
    }

    // Publicar los nodos rechazados; se entiende que cada uno de estos representa una celda
    // que se tratará como ocupada (con probabilidad 1) en el cálculo de entropía y en la selección de metas.
    occupied_nodes_pub_->publish(occupied_nodes);
    occupied_markers_pub_->publish(markers);
    RCLCPP_INFO(this->get_logger(), "Nodos ocupados (rechazados): %zu", occupied_nodes.poses.size());
  }

  // Parámetro: radio de proyección para considerar que un nodo está influenciado por un obstáculo.
  double obstacle_projection_radius_;

  // Variable para almacenar el último mensaje de nodos de obstáculo.
  geometry_msgs::msg::PoseArray obstacle_nodes_;

  // Suscriptores a los tópicos de nodos navegables y de obstáculos.
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr ground_nodes_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr obstacle_nodes_sub_;

  // Publicadores para los nodos rechazados (ocupados) y sus markers de visualización.
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr occupied_nodes_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr occupied_markers_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OccupiedRejectedNodes>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
