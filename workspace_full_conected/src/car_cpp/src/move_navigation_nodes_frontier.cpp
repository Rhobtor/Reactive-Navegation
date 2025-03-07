#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <limits>

// Incluir cabeceras de OctoMap para acceder al mapa de obstáculos
#include <octomap/octomap.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>

class FrontierNavigator : public rclcpp::Node {
public:
  FrontierNavigator()
  : Node("frontier_navigator"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    num_samples_(20),            // Número de muestras a lo largo de la línea (ajustable para pruebas)
    safety_distance_(0.05),       // Distancia mínima de seguridad (en metros) para considerar la celda segura
    stuck_counter_(0),
    stuck_threshold_(10),         // Número de iteraciones sin progreso para considerar al robot estancado
    avoidance_iterations_(5),     // Número de iteraciones en modo evasión reactiva
    avoidance_mode_(false),
    avoidance_counter_(0),
    last_distance_error_(std::numeric_limits<double>::max())
  {
    // Declarar parámetros para poder modificarlos en tiempo de ejecución
    this->declare_parameter("num_samples", num_samples_);
    this->declare_parameter("safety_distance", safety_distance_);
    this->declare_parameter("stuck_threshold", stuck_threshold_);
    this->declare_parameter("avoidance_iterations", avoidance_iterations_);
    
    navigable_nodes_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "ground_navigation_nodes", 10,
      std::bind(&FrontierNavigator::navigableNodesCallback, this, std::placeholders::_1));
      
    frontier_point_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "goal", 10,
      std::bind(&FrontierNavigator::frontierPointCallback, this, std::placeholders::_1));
      
    octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      "octomap", 10,
      std::bind(&FrontierNavigator::octomapCallback, this, std::placeholders::_1));
      
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    goal_reached_pub_ = this->create_publisher<std_msgs::msg::Bool>("goal_reached", 10);
    
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                     std::bind(&FrontierNavigator::controlLoop, this));
                                     
    RCLCPP_INFO(this->get_logger(), "Nodo de navegación reactiva iniciado.");
  }
  
private:
  // Callback para actualizar la lista de nodos navegables.
  void navigableNodesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    latest_navigable_nodes_ = *msg;
    RCLCPP_INFO(this->get_logger(), "Actualizados nodos navegables: %d poses.", static_cast<int>(msg->poses.size()));
  }
  
  // Callback para actualizar el objetivo (se utiliza el primer pose del PoseArray "goal").
  void frontierPointCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    latest_frontier_point_ = *msg;
    RCLCPP_INFO(this->get_logger(), "Actualizado punto de frontera: %d poses recibidas.", static_cast<int>(msg->poses.size()));
  }
  
  // Callback para actualizar el Octomap.
  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
    // Convertir el mensaje Octomap a un objeto OcTree
    octree_ = std::shared_ptr<octomap::OcTree>(
      dynamic_cast<octomap::OcTree*>(octomap_msgs::fullMsgToMap(*msg))
    );
    if(octree_) {
      RCLCPP_INFO(this->get_logger(), "Octomap actualizado correctamente.");
    } else {
      RCLCPP_WARN(this->get_logger(), "Fallo al actualizar el Octomap.");
    }
  }
  
  // Función refinada para comprobar si la trayectoria entre dos puntos está libre.
  bool isPathFree(const geometry_msgs::msg::Point &start, const geometry_msgs::msg::Point &end) {
    if (!octree_) {
      // Si aún no se ha recibido el Octomap, asumimos que la trayectoria es libre.
      return true;
    }
    
    // Utiliza el parámetro num_samples_ para muestrear la línea.
    for (int i = 0; i <= num_samples_; i++) {
      double t = static_cast<double>(i) / num_samples_;
      double x = start.x + t * (end.x - start.x);
      double y = start.y + t * (end.y - start.y);
      double z = start.z + t * (end.z - start.z);
      
      // Consultar la celda en el octree.
      octomap::OcTreeNode* node = octree_->search(x, y, z);
      if (node && octree_->isNodeOccupied(node)) {
        RCLCPP_DEBUG(this->get_logger(), "Obstáculo detectado en: (%.2f, %.2f, %.2f)", x, y, z);
        return false;
      }
    }
    return true;
  }
  
  // Lazo de control principal que incorpora:
  //  - Reevaluación dinámica de la trayectoria
  //  - Comportamiento reactivo en caso de estancamiento
  void controlLoop() {
    if (latest_navigable_nodes_.poses.empty() || latest_frontier_point_.poses.empty()) {
      RCLCPP_DEBUG(this->get_logger(), "Datos insuficientes (nodos o goal vacíos).");
      return;
    }
    
    // Obtener la posición actual del robot en el frame "map".
    geometry_msgs::msg::TransformStamped transformStamped;
    try {
      transformStamped = tf_buffer_.lookupTransform("map", "base_link", rclcpp::Time(0),
                                                      rclcpp::Duration::from_seconds(0.1));
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Error obteniendo transform: %s", ex.what());
      return;
    }
    
    double robot_x = transformStamped.transform.translation.x;
    double robot_y = transformStamped.transform.translation.y;
    double robot_z = transformStamped.transform.translation.z;
    
    // Extraer la orientación (yaw) del robot.
    tf2::Quaternion q;
    tf2::fromMsg(transformStamped.transform.rotation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    
    // Se utiliza el primer pose del PoseArray "goal" como objetivo.
    geometry_msgs::msg::Pose frontier_pose = latest_frontier_point_.poses[0];
    
    // Selección dinámica del nodo candidato.
    geometry_msgs::msg::Pose target_pose;
    double min_dist = std::numeric_limits<double>::max();
    bool found_candidate = false;
    for (const auto &pose : latest_navigable_nodes_.poses) {
      // Primero, comprobar que la trayectoria entre el goal y el nodo esté libre.
      if (!isPathFree(frontier_pose.position, pose.position)) {
        RCLCPP_DEBUG(this->get_logger(), "Nodo (%.2f, %.2f) descartado (camino goal-nodo no libre).", 
                     pose.position.x, pose.position.y);
        continue;
      }
      // También se comprueba la trayectoria desde la posición actual del robot al nodo.
      geometry_msgs::msg::Point robot_point;
      robot_point.x = robot_x;
      robot_point.y = robot_y;
      robot_point.z = robot_z;
      if (!isPathFree(robot_point, pose.position)) {
        RCLCPP_DEBUG(this->get_logger(), "Nodo (%.2f, %.2f) descartado (camino robot-nodo no libre).", 
                     pose.position.x, pose.position.y);
        continue;
      }
      
      // Seleccionar el nodo cuyo camino al goal sea el más corto.
      double dx = pose.position.x - frontier_pose.position.x;
      double dy = pose.position.y - frontier_pose.position.y;
      double dist = std::sqrt(dx * dx + dy * dy);
      if (dist < min_dist) {
        min_dist = dist;
        target_pose = pose;
        found_candidate = true;
      }
    }
    
    if (!found_candidate) {
      RCLCPP_WARN(this->get_logger(), "No se encontró un nodo candidato con trayectoria libre.");
      geometry_msgs::msg::Twist stop;
      stop.linear.x = 0.0;
      stop.angular.z = 0.0;
      cmd_vel_pub_->publish(stop);
      return;
    }
    
    // Re-evaluación dinámica: si la trayectoria desde el robot al nodo candidato se bloquea, se reselecciona.
    geometry_msgs::msg::Point robot_point;
    robot_point.x = robot_x;
    robot_point.y = robot_y;
    robot_point.z = robot_z;
    if (!isPathFree(robot_point, target_pose.position)) {
      RCLCPP_WARN(this->get_logger(), "Camino a nodo candidato bloqueado. Re-evaluando candidatos...");
      found_candidate = false;
      min_dist = std::numeric_limits<double>::max();
      for (const auto &pose : latest_navigable_nodes_.poses) {
        if (!isPathFree(robot_point, pose.position))
          continue;
        double dx = pose.position.x - frontier_pose.position.x;
        double dy = pose.position.y - frontier_pose.position.y;
        double dist = std::sqrt(dx * dx + dy * dy);
        if (dist < min_dist) {
          min_dist = dist;
          target_pose = pose;
          found_candidate = true;
        }
      }
      if (!found_candidate) {
        RCLCPP_WARN(this->get_logger(), "Re-evaluación: no hay nodo candidato disponible.");
        geometry_msgs::msg::Twist stop;
        stop.linear.x = 0.0;
        stop.angular.z = 0.0;
        cmd_vel_pub_->publish(stop);
        return;
      }
    }
    
    // Calcular el error de posición respecto al nodo candidato.
    double error_x = target_pose.position.x - robot_x;
    double error_y = target_pose.position.y - robot_y;
    double distance_error = std::sqrt(error_x * error_x + error_y * error_y);
    
    // Comportamiento reactivo: si el progreso (reducción del error) es insignificante, se considera estancado.
    double progress = last_distance_error_ - distance_error;
    if (progress < 0.001) {  // Umbral mínimo de progreso
      stuck_counter_++;
      RCLCPP_DEBUG(this->get_logger(), "Progreso insignificante. Contador de estancamiento: %d", stuck_counter_);
    } else {
      stuck_counter_ = 0;  // Se resetea si se detecta progreso
    }
    last_distance_error_ = distance_error;
    
    // Si se supera el umbral de estancamiento, activar el modo de evasión reactiva.
    if (stuck_counter_ >= stuck_threshold_) {
      avoidance_mode_ = true;
      avoidance_counter_ = avoidance_iterations_;
      RCLCPP_WARN(this->get_logger(), "Robot estancado. Activando comportamiento de evasión reactiva.");
      stuck_counter_ = 0; // Reiniciar contador de estancamiento
    }
    
    geometry_msgs::msg::Twist cmd_vel;
    
    // Si se activa el modo de evasión reactiva, se ejecuta un giro en sitio (puedes ajustar la acción).
    if (avoidance_mode_) {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.5;  // Valor de giro para intentar evadir el obstáculo
      avoidance_counter_--;
      RCLCPP_INFO(this->get_logger(), "Modo evasión: giro reactivo activado. Iteraciones restantes: %d", avoidance_counter_);
      if (avoidance_counter_ <= 0) {
        avoidance_mode_ = false;
        RCLCPP_INFO(this->get_logger(), "Finalizando modo evasión, re-evaluando trayectoria.");
      }
      cmd_vel_pub_->publish(cmd_vel);
      return; // Salir del control loop durante la evasión
    }
    
    // Calcular el ángulo hacia el nodo candidato.
    double angle_to_target = std::atan2(error_y, error_x);
    double angle_error = angle_to_target - yaw;
    // Normalizar el error angular a [-pi, pi]
    while (angle_error > M_PI)  angle_error -= 2 * M_PI;
    while (angle_error < -M_PI) angle_error += 2 * M_PI;
    
    // Parámetros de control (valores iniciales para pruebas, ajústalos según sea necesario)
    const double angle_threshold = 0.2;
    const double linear_gain = 1.0;
    const double angular_gain = 1.0;
    const double rotational_only_gain = 0.7;
    
    double linear_speed = 0.0;
    double angular_speed = 0.0;
    
    // Si el error angular es grande, rota en sitio; si no, avanza hacia el nodo.
    if (std::fabs(angle_error) > angle_threshold) {
      linear_speed = 0.0;
      angular_speed = rotational_only_gain * angle_error;
      RCLCPP_INFO(this->get_logger(), "Modo rotación: angular_speed=%.2f", angular_speed);
    } else {
      linear_speed = linear_gain * distance_error;
      angular_speed = angular_gain * angle_error;
      RCLCPP_INFO(this->get_logger(), "Modo avance: linear_speed=%.2f, angular_speed=%.2f", linear_speed, angular_speed);
    }
    
    // Si el error de distancia es muy pequeño, se considera alcanzada la meta.
    bool goal_reached = (distance_error < 0.1);
    if (goal_reached) {
      RCLCPP_INFO(this->get_logger(), "Meta alcanzada: distance_error=%.2f", distance_error);
      linear_speed = 0.0;
      angular_speed = 0.0;
    }
    
    // Publicar el comando de velocidad.
    cmd_vel.linear.x = linear_speed;
    cmd_vel.angular.z = angular_speed;
    cmd_vel_pub_->publish(cmd_vel);
    
    // Publicar el estado de meta alcanzada.
    std_msgs::msg::Bool goal_msg;
    goal_msg.data = goal_reached;
    goal_reached_pub_->publish(goal_msg);
    
    // Para pruebas: observa los logs y ajusta los parámetros según el comportamiento observado.
  }
  
  // Subscripciones, publicadores y timer.
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr navigable_nodes_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr frontier_point_sub_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reached_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Datos actualizados.
  geometry_msgs::msg::PoseArray latest_navigable_nodes_;
  geometry_msgs::msg::PoseArray latest_frontier_point_;
  std::shared_ptr<octomap::OcTree> octree_;
  
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  
  // Variables para la verificación de la trayectoria y comportamiento reactivo.
  int num_samples_;
  double safety_distance_;
  int stuck_counter_;
  int stuck_threshold_;
  int avoidance_iterations_;
  bool avoidance_mode_;
  int avoidance_counter_;
  double last_distance_error_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FrontierNavigator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
