// #include <rclcpp/rclcpp.hpp>
// #include <visualization_msgs/msg/marker_array.hpp>
// #include <visualization_msgs/msg/marker.hpp>
// #include <geometry_msgs/msg/pose_array.hpp>
// #include <geometry_msgs/msg/pose.hpp>
// #include <octomap/octomap.h>
// #include <octomap/OcTree.h>
// #include <octomap_msgs/msg/octomap.hpp>
// #include <octomap_msgs/conversions.h>
// #include <tf2_ros/buffer.h>
// #include <tf2_ros/transform_listener.h>
// #include <geometry_msgs/msg/transform_stamped.hpp>
// #include <vector>
// #include <memory>
// #include <string>
// #include <cmath>

// // Función que recorre el octree y extrae las coordenadas de los nodos libres.
// std::vector<octomap::point3d> extractFreeNodes(const octomap::OcTree &tree) {
//   std::vector<octomap::point3d> nodes;
//   for (octomap::OcTree::leaf_iterator it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it) {
//     // Consideramos el nodo libre si no está ocupado
//     if (!tree.isNodeOccupied(*it)) {
//       nodes.push_back(it.getCoordinate());
//     }
//   }
//   return nodes;
// }

// class NavigationNode : public rclcpp::Node {
// public:
//   NavigationNode()
//   : Node("navigation_node"),
//     tf_buffer_(this->get_clock()),
//     min_distance_(0.0),
//     max_distance_(10.0)
//   {
//     // Parámetros de filtrado por distancia (respecto a la posición del vehículo)
//     this->declare_parameter("min_distance", 0.0);
//     this->declare_parameter("max_distance", 10.0);
//     min_distance_ = this->get_parameter("min_distance").as_double();
//     max_distance_ = this->get_parameter("max_distance").as_double();

//     // Publicador para los markers de nodos de navegación
//     marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("navigation_nodes", 10);
//     // Publicador para los puntos de navegación (PoseArray)
//     navigable_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("navigable_nodes", 10);

//     octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
//       "/octomap_full",
//       10,
//       std::bind(&NavigationNode::octomapCallback, this, std::placeholders::_1)
//     );
//     // Crear el listener para obtener transformaciones
//     tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_, this);

//     RCLCPP_INFO(this->get_logger(), "Nodo de navegación iniciado.");
//   }

// private:
//   void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
//     // Obtener la posición actual del vehículo en el frame "map" usando el transform de "base_link"
//     geometry_msgs::msg::TransformStamped transformStamped;
//     try {
//       // Se obtiene la transformación de "base_link" al frame "map"
//       rclcpp::Time query_time(0, 0, this->get_clock()->get_clock_type());
//       transformStamped = tf_buffer_.lookupTransform("map", "base_link", query_time, rclcpp::Duration::from_seconds(0.1));
//     } catch (tf2::TransformException &ex) {
//       RCLCPP_WARN(this->get_logger(), "No se pudo obtener la transformación: %s", ex.what());
//       return;
//     }

//     double robot_x = transformStamped.transform.translation.x;
//     double robot_y = transformStamped.transform.translation.y;
//     double robot_z = transformStamped.transform.translation.z;

//     // Convertir el mensaje Octomap a un OcTree
//     octomap::AbstractOcTree* tree_raw = octomap_msgs::fullMsgToMap(*msg);
//     if (!tree_raw) {
//       RCLCPP_ERROR(this->get_logger(), "Error al convertir el mensaje Octomap a OcTree");
//       return;
//     }
//     // Convertir a OcTree (asegurándose de que el árbol es del tipo OcTree)
//     octomap::OcTree* tree_ptr = dynamic_cast<octomap::OcTree*>(tree_raw);
//     if (!tree_ptr) {
//       RCLCPP_ERROR(this->get_logger(), "El árbol obtenido no es de tipo OcTree");
//       delete tree_raw;
//       return;
//     }
//     // Usamos un smart pointer para gestionar el árbol
//     std::shared_ptr<octomap::OcTree> tree(tree_ptr);

//     // Extraer nodos libres (navegables)
//     auto nodes = extractFreeNodes(*tree);
    
//     visualization_msgs::msg::MarkerArray marker_array;
//     geometry_msgs::msg::PoseArray navigable_points;
//     navigable_points.header.frame_id = "map";
//     navigable_points.header.stamp = this->now();
    
//     int id = 0;
//     for (const auto &pt : nodes) {
//       // Calcular la distancia entre el punto y la posición actual del vehículo (base_link en el frame map)
//       double dx = pt.x() - robot_x;
//       double dy = pt.y() - robot_y;
//       double dz = pt.z() - robot_z;
//       double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
//       if (distance < min_distance_ || distance > max_distance_) {
//         continue;
//       }
      
//       // Crear y configurar el marker
//       visualization_msgs::msg::Marker marker;
//       marker.header.frame_id = "map";
//       marker.header.stamp = this->now();
//       marker.ns = "navigation_nodes";
//       marker.id = id++;
//       marker.type = visualization_msgs::msg::Marker::SPHERE;
//       marker.action = visualization_msgs::msg::Marker::ADD;
//       marker.pose.position.x = pt.x();
//       marker.pose.position.y = pt.y();
//       marker.pose.position.z = pt.z();
//       marker.pose.orientation.w = 1.0;
//       marker.scale.x = 0.1;
//       marker.scale.y = 0.1;
//       marker.scale.z = 0.1;
//       marker.color.r = 0.0;
//       marker.color.g = 1.0;
//       marker.color.b = 0.0;
//       marker.color.a = 1.0;
//       marker.lifetime = rclcpp::Duration(0, 0);
//       marker_array.markers.push_back(marker);

//       // Crear y agregar la Pose correspondiente al punto navegable
//       geometry_msgs::msg::Pose pose;
//       pose.position.x = pt.x();
//       pose.position.y = pt.y();
//       pose.position.z = pt.z();
//       // Orientación por defecto (sin rotación)
//       pose.orientation.x = 0.0;
//       pose.orientation.y = 0.0;
//       pose.orientation.z = 0.0;
//       pose.orientation.w = 1.0;
//       navigable_points.poses.push_back(pose);
//     }
    
//     marker_pub_->publish(marker_array);
//     navigable_pub_->publish(navigable_points);
//     //RCLCPP_INFO(this->get_logger(), "Markers publicados: %d, puntos navegables: %zu", id, navigable_points.poses.size());
//   }

//   rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
//   rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr navigable_pub_;
//   rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
//   tf2_ros::Buffer tf_buffer_;
//   std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
//   double min_distance_;
//   double max_distance_;
// };

// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<NavigationNode>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }


// #include <rclcpp/rclcpp.hpp>
// #include <visualization_msgs/msg/marker_array.hpp>
// #include <visualization_msgs/msg/marker.hpp>
// #include <geometry_msgs/msg/pose_array.hpp>
// #include <geometry_msgs/msg/pose.hpp>
// #include <octomap/octomap.h>
// #include <octomap/OcTree.h>
// #include <octomap_msgs/msg/octomap.hpp>
// #include <octomap_msgs/conversions.h>
// #include <tf2_ros/buffer.h>
// #include <tf2_ros/transform_listener.h>
// #include <geometry_msgs/msg/transform_stamped.hpp>
// #include <vector>
// #include <memory>
// #include <string>
// #include <cmath>

// // Función que recorre el octree y extrae las coordenadas de los nodos libres.
// std::vector<octomap::point3d> extractFreeNodes(const octomap::OcTree &tree) {
//   std::vector<octomap::point3d> nodes;
//   for (octomap::OcTree::leaf_iterator it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it) {
//     // Consideramos el nodo libre si no está ocupado
//     if (!tree.isNodeOccupied(*it)) {
//       nodes.push_back(it.getCoordinate());
//     }
//   }
//   return nodes;
// }

// class NavigationNode : public rclcpp::Node {
// public:
//   NavigationNode()
//   : Node("navigation_node"),
//     tf_buffer_(this->get_clock()),
//     min_distance_(0.0),
//     max_distance_(10.0),
//     node_distance_threshold_(2.0)  // Valor por defecto para downsampling
//   {
//     // Parámetros de filtrado por distancia (respecto a la posición del vehículo)
//     this->declare_parameter("min_distance", 0.0);
//     this->declare_parameter("max_distance", 20.0);
//     this->declare_parameter("node_distance_threshold", 1.0);  // Nuevo parámetro

//     min_distance_ = this->get_parameter("min_distance").as_double();
//     max_distance_ = this->get_parameter("max_distance").as_double();
//     node_distance_threshold_ = this->get_parameter("node_distance_threshold").as_double();

//     // Publicador para los markers de nodos de navegación
//     marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("navigation_nodes", 10);
//     // Publicador para los puntos de navegación (PoseArray)
//     navigable_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("navigable_nodes", 10);

//     octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
//       "/octomap_full",
//       10,
//       std::bind(&NavigationNode::octomapCallback, this, std::placeholders::_1)
//     );
//     // Crear el listener para obtener transformaciones
//     tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_, this);

//     RCLCPP_INFO(this->get_logger(), "Nodo de navegación iniciado.");
//   }

// private:
//   void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
//     // Obtener la posición actual del vehículo en el frame "map" usando el transform de "base_link"
//     geometry_msgs::msg::TransformStamped transformStamped;
//     try {
//       // Se obtiene la transformación de "base_link" al frame "map"
//       rclcpp::Time query_time(0, 0, this->get_clock()->get_clock_type());
//       transformStamped = tf_buffer_.lookupTransform("map", "base_link", query_time, rclcpp::Duration::from_seconds(0.1));
//     } catch (tf2::TransformException &ex) {
//       RCLCPP_WARN(this->get_logger(), "No se pudo obtener la transformación: %s", ex.what());
//       return;
//     }

//     double robot_x = transformStamped.transform.translation.x;
//     double robot_y = transformStamped.transform.translation.y;
//     double robot_z = transformStamped.transform.translation.z;

//     // Convertir el mensaje Octomap a un OcTree
//     octomap::AbstractOcTree* tree_raw = octomap_msgs::fullMsgToMap(*msg);
//     if (!tree_raw) {
//       RCLCPP_ERROR(this->get_logger(), "Error al convertir el mensaje Octomap a OcTree");
//       return;
//     }
//     // Convertir a OcTree (asegurándose de que el árbol es del tipo OcTree)
//     octomap::OcTree* tree_ptr = dynamic_cast<octomap::OcTree*>(tree_raw);
//     if (!tree_ptr) {
//       RCLCPP_ERROR(this->get_logger(), "El árbol obtenido no es de tipo OcTree");
//       delete tree_raw;
//       return;
//     }
//     // Usamos un smart pointer para gestionar el árbol
//     std::shared_ptr<octomap::OcTree> tree(tree_ptr);

//     // Extraer nodos libres (navegables)
//     auto nodes = extractFreeNodes(*tree);

//     // Aquí aplicamos un downsampling: agregamos un nodo solo si está suficientemente lejos
//     std::vector<octomap::point3d> downsampled_nodes;
//     for (const auto &pt : nodes) {
//       // Calcular la distancia entre el punto y la posición actual del vehículo (base_link en el frame map)
//       double dx = pt.x() - robot_x;
//       double dy = pt.y() - robot_y;
//       double dz = pt.z() - robot_z;
//       double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
//       if (distance < min_distance_ || distance > max_distance_) {
//         continue;
//       }
      
//       // Verificar que el punto no esté muy cerca de alguno ya aceptado
//       bool is_close = false;
//       for (const auto &accepted_pt : downsampled_nodes) {
//         double d = std::sqrt(std::pow(pt.x() - accepted_pt.x(), 2) +
//                              std::pow(pt.y() - accepted_pt.y(), 2) +
//                              std::pow(pt.z() - accepted_pt.z(), 2));
//         if (d < node_distance_threshold_) {
//           is_close = true;
//           break;
//         }
//       }
//       if (!is_close) {
//         downsampled_nodes.push_back(pt);
//       }
//     }
    
//     visualization_msgs::msg::MarkerArray marker_array;
//     geometry_msgs::msg::PoseArray navigable_points;
//     navigable_points.header.frame_id = "map";
//     navigable_points.header.stamp = this->now();
    
//     int id = 0;
//     for (const auto &pt : downsampled_nodes) {
//       // Crear y configurar el marker
//       visualization_msgs::msg::Marker marker;
//       marker.header.frame_id = "map";
//       marker.header.stamp = this->now();
//       marker.ns = "navigation_nodes";
//       marker.id = id++;
//       marker.type = visualization_msgs::msg::Marker::SPHERE;
//       marker.action = visualization_msgs::msg::Marker::ADD;
//       marker.pose.position.x = pt.x();
//       marker.pose.position.y = pt.y();
//       marker.pose.position.z = pt.z();
//       marker.pose.orientation.w = 1.0;
//       marker.scale.x = 0.1;
//       marker.scale.y = 0.1;
//       marker.scale.z = 0.1;
//       marker.color.r = 0.0;
//       marker.color.g = 1.0;
//       marker.color.b = 0.0;
//       marker.color.a = 1.0;
//       marker.lifetime = rclcpp::Duration(0, 0);
//       marker_array.markers.push_back(marker);

//       // Crear y agregar la Pose correspondiente al punto navegable
//       geometry_msgs::msg::Pose pose;
//       pose.position.x = pt.x();
//       pose.position.y = pt.y();
//       pose.position.z = pt.z();
//       pose.orientation.x = 0.0;
//       pose.orientation.y = 0.0;
//       pose.orientation.z = 0.0;
//       pose.orientation.w = 1.0;
//       navigable_points.poses.push_back(pose);
//     }
    
//     marker_pub_->publish(marker_array);
//     navigable_pub_->publish(navigable_points);
//     //RCLCPP_INFO(this->get_logger(), "Markers publicados: %d, puntos navegables: %zu", id, navigable_points.poses.size());
//   }

//   rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
//   rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr navigable_pub_;
//   rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
//   tf2_ros::Buffer tf_buffer_;
//   std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
//   double min_distance_;
//   double max_distance_;
//   double node_distance_threshold_;
// };

// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<NavigationNode>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
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

// Incluir nanoflann (asegúrate de tener el archivo nanoflann.hpp en tu include path)
#include "nanoflann.hpp"

using namespace nanoflann;

// --- Estructura para representar puntos 3D (solo usaremos x,y) ---
struct PointCloud2D
{
  std::vector<std::array<double, 2>>  pts;

  // Interface requerida por nanoflann:
  inline size_t kdtree_get_point_count() const { return pts.size(); }
  inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
    return pts[idx][dim];
  }
  template <class BBOX>
  bool kdtree_get_bbox(BBOX&) const { return false; }
};

// Función que recorre el octree y extrae las coordenadas de los nodos libres.
std::vector<octomap::point3d> extractFreeNodes(const octomap::OcTree &tree) {
  std::vector<octomap::point3d> nodes;
  for (octomap::OcTree::leaf_iterator it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it) {
    if (!tree.isNodeOccupied(*it)) {
      nodes.push_back(it.getCoordinate());
    }
  }
  return nodes;
}

// Función para realizar downsampling utilizando nanoflann y un kd-tree.
std::vector<octomap::point3d> downsampleNodesKdTree(const std::vector<octomap::point3d>& nodes,
                                                    double node_distance_threshold) {
  std::vector<octomap::point3d> downsampled;
  if (nodes.empty()) return downsampled;

  // Construir el PointCloud2D a partir de los nodos (solo x,y)
  PointCloud2D cloud;
  for (const auto &pt : nodes) {
    cloud.pts.push_back({pt.x(), pt.y()});
  }
  // Construir el kd-tree: usamos 2 dimensiones y la métrica L2.
  typedef KDTreeSingleIndexAdaptor<
      L2_Simple_Adaptor<double, PointCloud2D>,
      PointCloud2D,
      2 /* dim */
      > my_kd_tree_t;

  my_kd_tree_t   index(2, cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */));
  index.buildIndex();

  // Vector para marcar si un punto ya ha sido aceptado
  std::vector<bool> accepted(cloud.pts.size(), false);
  
  // Iterar sobre los puntos y, si no han sido aceptados, agregarlos y marcar vecinos cercanos
  const double search_radius = node_distance_threshold * node_distance_threshold;
  for (size_t i = 0; i < cloud.pts.size(); ++i) {
    if (accepted[i])
      continue;
    // Aceptar este punto
    accepted[i] = true;
    downsampled.push_back(octomap::point3d(cloud.pts[i][0], cloud.pts[i][1], 0.0)); // z=0, o podrías usar el original

    // Buscar vecinos cercanos a este punto
    std::vector<std::pair<unsigned int, double>> ret_matches;
    nanoflann::SearchParams params;
    const double query_pt[2] = { cloud.pts[i][0], cloud.pts[i][1] };
    const size_t nMatches = index.radiusSearch(&query_pt[0], search_radius, ret_matches, params);
    for (size_t j = 0; j < nMatches; j++) {
      accepted[ret_matches[j].first] = true;
    }
  }
  return downsampled;
}

class NavigationNode : public rclcpp::Node {
public:
  NavigationNode()
  : Node("navigation_node"),
    tf_buffer_(this->get_clock()),
    min_distance_(0.0),
    max_distance_(40.0)  // Ampliado a 20 m, por ejemplo
  {
    // Parámetros
    this->declare_parameter("min_distance", 0.0);
    this->declare_parameter("max_distance", 40.0);
    this->declare_parameter("node_distance_threshold", 1.0);
    min_distance_ = this->get_parameter("min_distance").as_double();
    max_distance_ = this->get_parameter("max_distance").as_double();
    node_distance_threshold_ = this->get_parameter("node_distance_threshold").as_double();

    // Publicadores
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("navigation_nodes", 10);
    navigable_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("navigable_nodes", 10);

    // Subscripción a octomap
    octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      "/octomap_full", 10,
      std::bind(&NavigationNode::octomapCallback, this, std::placeholders::_1)
    );
    // Transform listener
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_, this);

    RCLCPP_INFO(this->get_logger(), "Nodo de navegación iniciado.");
  }

private:
  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
    // Obtener la transformación para la posición del vehículo
    geometry_msgs::msg::TransformStamped transformStamped;
    try {
      rclcpp::Time query_time(0, 0, this->get_clock()->get_clock_type());
      transformStamped = tf_buffer_.lookupTransform("map", "base_link", query_time, rclcpp::Duration::from_seconds(0.1));
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "No se pudo obtener la transformación: %s", ex.what());
      return;
    }

    double robot_x = transformStamped.transform.translation.x;
    double robot_y = transformStamped.transform.translation.y;
    double robot_z = transformStamped.transform.translation.z;

    // Convertir el mensaje Octomap a OcTree
    octomap::AbstractOcTree* tree_raw = octomap_msgs::fullMsgToMap(*msg);
    if (!tree_raw) {
      RCLCPP_ERROR(this->get_logger(), "Error al convertir el mensaje Octomap a OcTree");
      return;
    }
    octomap::OcTree* tree_ptr = dynamic_cast<octomap::OcTree*>(tree_raw);
    if (!tree_ptr) {
      RCLCPP_ERROR(this->get_logger(), "El árbol obtenido no es de tipo OcTree");
      delete tree_raw;
      return;
    }
    std::shared_ptr<octomap::OcTree> tree(tree_ptr);

    // Extraer nodos libres
    auto nodes = extractFreeNodes(*tree);

    // Filtrar los nodos según ROI (min_distance y max_distance)
    std::vector<octomap::point3d> roi_nodes;
    for (const auto &pt : nodes) {
      double dx = pt.x() - robot_x;
      double dy = pt.y() - robot_y;
      double dz = pt.z() - robot_z;
      double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (dist >= min_distance_ && dist <= max_distance_) {
        roi_nodes.push_back(pt);
      }
    }

    // Aplicar downsampling usando kd-tree (nanoflann)
    auto downsampled_nodes = downsampleNodesKdTree(roi_nodes, node_distance_threshold_);

    // Preparar mensajes para markers y PoseArray
    visualization_msgs::msg::MarkerArray marker_array;
    geometry_msgs::msg::PoseArray navigable_points;
    navigable_points.header.frame_id = "map";
    navigable_points.header.stamp = this->now();

    int id = 0;
    for (const auto &pt : downsampled_nodes) {
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

      geometry_msgs::msg::Pose pose;
      pose.position.x = pt.x();
      pose.position.y = pt.y();
      pose.position.z = pt.z();
      pose.orientation.x = 0.0;
      pose.orientation.y = 0.0;
      pose.orientation.z = 0.0;
      pose.orientation.w = 1.0;
      navigable_points.poses.push_back(pose);
    }

    marker_pub_->publish(marker_array);
    navigable_pub_->publish(navigable_points);
    //RCLCPP_INFO(this->get_logger(), "Markers publicados: %d, puntos navegables: %zu", id, navigable_points.poses.size());
  }

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr navigable_pub_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  double min_distance_;
  double max_distance_;
  double node_distance_threshold_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavigationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
