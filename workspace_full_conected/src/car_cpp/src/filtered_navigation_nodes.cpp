// // // filtered_navigation_nodes.cpp
// // Este nodo filtra los nodos de navegación del suelo (publicados en "ground_navigation_nodes")
// // eliminando aquellos que estén demasiado cerca de los nodos de obstáculo (publicados en "obstacle_navigation_nodes").
// // El resultado se publica en el tópico "filtered_navigation_nodes" y se visualiza mediante markers.

// // filtered_navigation_nodes.cpp
// // Este nodo realiza lo siguiente:
// // 1. Se suscribe a dos tópicos:
// //    - "ground_navigation_nodes": que contiene los nodos navegables del suelo.
// //    - "obstacle_navigation_nodes": que contiene los nodos (proyectados sobre el suelo)
// //      de los obstáculos detectados a partir del OctoMap.
// // 2. Para cada nodo navegable, verifica si se encuentra a menos de un radio configurable
// //    (obstacle_projection_radius) de algún obstáculo.
// // 3. Si el nodo está dentro de la influencia (huella) de algún obstáculo, se descarta.
// // 4. Publica el conjunto de nodos filtrados en el tópico "filtered_navigation_nodes"
// //    y genera markers para su visualización en RViz.

// #include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/pose_array.hpp>
// #include <geometry_msgs/msg/pose.hpp>
// #include <geometry_msgs/msg/point.hpp>
// #include <visualization_msgs/msg/marker_array.hpp>
// #include <visualization_msgs/msg/marker.hpp>
// #include <cmath>
// #include <vector>

// class FilteredNavigationNodes : public rclcpp::Node
// {
// public:
//   FilteredNavigationNodes() : Node("filtered_navigation_nodes")
//   {
//     // Declarar parámetros configurables:
//     // obstacle_projection_radius: radio de influencia de un obstáculo proyectado sobre el suelo.
//     this->declare_parameter("obstacle_projection_radius", 0.5);
//     obstacle_projection_radius_ = this->get_parameter("obstacle_projection_radius").as_double();

//     // Suscribirse al tópico de nodos navegables del suelo
//     ground_nodes_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
//       "ground_navigation_nodes", 10,
//       std::bind(&FilteredNavigationNodes::groundNodesCallback, this, std::placeholders::_1)
//     );

//     // Suscribirse al tópico de nodos de obstáculo (proyectados al suelo)
//     obstacle_nodes_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
//       "obstacle_navigation_nodes", 10,
//       std::bind(&FilteredNavigationNodes::obstacleNodesCallback, this, std::placeholders::_1)
//     );

//     // Publicador para los nodos filtrados (nodos de navegación que no estén en zonas de obstáculo)
//     filtered_nodes_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("filtered_navigation_nodes", 10);

//     // Publicador para visualizar los nodos filtrados (markers)
//     filtered_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("filtered_nodes_markers", 10);

//     RCLCPP_INFO(this->get_logger(), "Nodo FilteredNavigationNodes iniciado. Radio de proyección de obstáculo: %.2f",
//                 obstacle_projection_radius_);
//   }

// private:
//   // Callback para los nodos de obstáculo.
//   // Se almacena el mensaje recibido en la variable miembro obstacle_nodes_
//   void obstacleNodesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
//   {
//     obstacle_nodes_ = *msg;
//   }

//   // Callback para los nodos navegables del suelo.
//   // Se filtran los nodos descartando aquellos que estén dentro de la huella de algún obstáculo.
//   void groundNodesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
//   {
//     // Creamos un PoseArray para almacenar los nodos filtrados.
//     geometry_msgs::msg::PoseArray filtered_nodes;
//     filtered_nodes.header = msg->header;

//     // Creamos un MarkerArray para la visualización en RViz.
//     visualization_msgs::msg::MarkerArray markers;
//     int marker_id = 0;

//     // Iteramos sobre cada nodo navegable recibido
//     for (const auto &ground_pose : msg->poses)
//     {
//       bool keep_node = true; // Bandera que indica si se conserva este nodo

//       // Verificamos la distancia entre este nodo y cada nodo de obstáculo almacenado
//       for (const auto &obs_pose : obstacle_nodes_.poses)
//       {
//         // Se calcula la distancia en el plano (x, y)
//         double dx = ground_pose.position.x - obs_pose.position.x;
//         double dy = ground_pose.position.y - obs_pose.position.y;
//         double distance = std::sqrt(dx * dx + dy * dy);

//         // Si la distancia es menor que el radio de proyección, se descarta este nodo
//         if (distance < obstacle_projection_radius_)
//         {
//           keep_node = false;
//           break;
//         }
//       }

//       // Si el nodo supera el filtro, se añade al PoseArray filtrado
//       if (keep_node)
//       {
//         filtered_nodes.poses.push_back(ground_pose);

//         // Crear marker para visualizar este nodo filtrado (se muestra como esfera verde)
//         visualization_msgs::msg::Marker marker;
//         marker.header = msg->header;
//         marker.ns = "filtered_nodes";
//         marker.id = marker_id++;
//         marker.type = visualization_msgs::msg::Marker::SPHERE;
//         marker.action = visualization_msgs::msg::Marker::ADD;
//         marker.pose = ground_pose;
//         marker.scale.x = 0.1;
//         marker.scale.y = 0.1;
//         marker.scale.z = 0.1;
//         marker.color.r = 0.0;
//         marker.color.g = 1.0;
//         marker.color.b = 0.0;
//         marker.color.a = 1.0;
//         marker.lifetime = rclcpp::Duration(0, 0);
//         markers.markers.push_back(marker);
//       }
//     }

//     // Publicar los nodos filtrados y los markers
//     filtered_nodes_pub_->publish(filtered_nodes);
//     filtered_markers_pub_->publish(markers);

//     //RCLCPP_INFO(this->get_logger(), "Nodos filtrados: %zu de %zu", filtered_nodes.poses.size(), msg->poses.size());
//   }

//   double obstacle_projection_radius_; // Radio de influencia para descartar nodos cercanos a obstáculos

//   // Variable para almacenar el último mensaje recibido de nodos de obstáculo
//   geometry_msgs::msg::PoseArray obstacle_nodes_;

//   // Suscriptores para los nodos de navegación del suelo y los nodos de obstáculo
//   rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr ground_nodes_sub_;
//   rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr obstacle_nodes_sub_;

//   // Publicadores para los nodos filtrados y los markers
//   rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr filtered_nodes_pub_;
//   rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr filtered_markers_pub_;
// };

// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<FilteredNavigationNodes>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }


#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cmath>
#include <vector>
#include <algorithm>

class FilteredNavigationNodes : public rclcpp::Node
{
public:
  FilteredNavigationNodes() : Node("filtered_navigation_nodes")
  {
    // Declarar parámetros configurables:
    // obstacle_projection_radius: para filtrar nodos muy próximos a los obstáculos.
    // obstacle_ray_threshold: distancia mínima permitida desde el rayo a un obstáculo para considerarlo colisionado.
    this->declare_parameter("obstacle_projection_radius", 0.5);
    this->declare_parameter("obstacle_ray_threshold", 0.3);
    obstacle_projection_radius_ = this->get_parameter("obstacle_projection_radius").as_double();
    obstacle_ray_threshold_ = this->get_parameter("obstacle_ray_threshold").as_double();

    // Suscripción a los nodos navegables del suelo.
    ground_nodes_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "ground_navigation_nodes", 10,
      std::bind(&FilteredNavigationNodes::groundNodesCallback, this, std::placeholders::_1)
    );

    // Suscripción a los nodos de obstáculos (obtenidos de obstacle_navigation_nodes).
    obstacle_nodes_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "obstacle_navigation_nodes", 10,
      std::bind(&FilteredNavigationNodes::obstacleNodesCallback, this, std::placeholders::_1)
    );

    // Suscripción a la odometría para obtener la posición actual del robot.
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      std::bind(&FilteredNavigationNodes::odomCallback, this, std::placeholders::_1)
    );

    // Publicadores para los nodos filtrados y para la visualización (markers) en RViz.
    filtered_nodes_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("filtered_navigation_nodes", 10);
    filtered_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("filtered_nodes_markers", 10);

    RCLCPP_INFO(this->get_logger(), "Nodo FilteredNavigationNodes iniciado. Radio: %.2f, umbral de rayo: %.2f",
                obstacle_projection_radius_, obstacle_ray_threshold_);
  }

private:
  // Callback para recibir los nodos de obstáculos.
  void obstacleNodesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    obstacle_nodes_ = *msg;
  }

  // Callback para la odometría (posición del robot).
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    robot_pose_ = msg->pose.pose;
    robot_pose_received_ = true;
  }

  // Función que calcula la distancia desde un punto p a la línea (segmento) definido por v y w.
  double distancePointToSegment(const geometry_msgs::msg::Point &p,
                                const geometry_msgs::msg::Point &v,
                                const geometry_msgs::msg::Point &w)
  {
    double vx = w.x - v.x;
    double vy = w.y - v.y;
    double lengthSq = vx * vx + vy * vy;
    if (lengthSq == 0)
      return std::hypot(p.x - v.x, p.y - v.y);
    double t = ((p.x - v.x) * vx + (p.y - v.y) * vy) / lengthSq;
    t = std::max(0.0, std::min(1.0, t));
    double projX = v.x + t * vx;
    double projY = v.y + t * vy;
    return std::hypot(p.x - projX, p.y - projY);
  }

  // Función para verificar si el rayo (segmento entre start y end) está libre de colisiones
  // con obstáculos (usando los nodos de obstacle_navigation_nodes).
  bool isRayCollisionFree(const geometry_msgs::msg::Point &start,
                          const geometry_msgs::msg::Point &end)
  {
    // Iterar sobre cada obstáculo disponible.
    for (const auto &obs_pose : obstacle_nodes_.poses)
    {
      double dist = distancePointToSegment(obs_pose.position, start, end);
      if (dist < obstacle_ray_threshold_)
      {
        return false;  // Se detectó un obstáculo lo suficientemente cerca del rayo.
      }
    }
    return true;
  }

  // Callback para los nodos navegables del suelo, integrando ambos filtros.
  void groundNodesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    geometry_msgs::msg::PoseArray filtered_nodes;
    filtered_nodes.header = msg->header;
    visualization_msgs::msg::MarkerArray markers;
    int marker_id = 0;

    if (!robot_pose_received_)
    {
      RCLCPP_WARN(this->get_logger(), "No se ha recibido la odometría del robot; se omite la verificación de colisión en el rayo.");
    }

    // Recorrer cada nodo navegable.
    for (const auto &ground_pose : msg->poses)
    {
      bool keep_node = true;
      
      // Filtro 1: Descarta el nodo si está muy cerca de algún obstáculo.
      for (const auto &obs_pose : obstacle_nodes_.poses)
      {
        double dx = ground_pose.position.x - obs_pose.position.x;
        double dy = ground_pose.position.y - obs_pose.position.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        if (distance < obstacle_projection_radius_)
        {
          keep_node = false;
          break;
        }
      }
      
      // Filtro 2: Comprueba la línea de visión (rayo) desde la posición del robot hasta el nodo.
      if (keep_node && robot_pose_received_)
      {
        if (!isRayCollisionFree(robot_pose_.position, ground_pose.position))
        {
          keep_node = false;
        }
      }
      
      if (keep_node)
      {
        filtered_nodes.poses.push_back(ground_pose);
        
        // Crear marker para visualización en RViz.
        visualization_msgs::msg::Marker marker;
        marker.header = msg->header;
        marker.ns = "filtered_nodes";
        marker.id = marker_id++;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = ground_pose;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.lifetime = rclcpp::Duration(0, 0);
        markers.markers.push_back(marker);
      }
    }
    
    filtered_nodes_pub_->publish(filtered_nodes);
    filtered_markers_pub_->publish(markers);
  }

  // Variables miembro
  double obstacle_projection_radius_;
  double obstacle_ray_threshold_;
  geometry_msgs::msg::PoseArray obstacle_nodes_;
  geometry_msgs::msg::Pose robot_pose_;
  bool robot_pose_received_ = false;
  
  // Suscriptores
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr ground_nodes_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr obstacle_nodes_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  
  // Publicadores
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr filtered_nodes_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr filtered_markers_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FilteredNavigationNodes>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
