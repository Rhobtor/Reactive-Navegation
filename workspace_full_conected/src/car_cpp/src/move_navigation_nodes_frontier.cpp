#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <octomap/octomap.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <cmath>
#include <limits>
#include <algorithm>

// Función auxiliar: calcula la distancia desde un punto p a la línea (segmento) definida por a y b.
double distanceFromPointToLineSegment(const geometry_msgs::msg::Point &p,
                                      const geometry_msgs::msg::Point &a,
                                      const geometry_msgs::msg::Point &b)
{
  double dx = b.x - a.x;
  double dy = b.y - a.y;
  double dz = b.z - a.z;
  double mag2 = dx * dx + dy * dy + dz * dz;
  if(mag2 == 0)
    return std::sqrt((p.x - a.x) * (p.x - a.x) +
                     (p.y - a.y) * (p.y - a.y) +
                     (p.z - a.z) * (p.z - a.z));
  double t = ((p.x - a.x) * dx + (p.y - a.y) * dy + (p.z - a.z) * dz) / mag2;
  t = std::max(0.0, std::min(1.0, t));
  double proj_x = a.x + t * dx;
  double proj_y = a.y + t * dy;
  double proj_z = a.z + t * dz;
  return std::sqrt((p.x - proj_x) * (p.x - proj_x) +
                   (p.y - proj_y) * (p.y - proj_y) +
                   (p.z - proj_z) * (p.z - proj_z));
}

// Función auxiliar: devuelve la clearance (distancia mínima) desde el punto p a alguno de los nodos ocupados.
double getClearance(const geometry_msgs::msg::PoseArray &occupied_nodes,
                    const geometry_msgs::msg::Point &p)
{
  double min_clearance = std::numeric_limits<double>::max();
  for (const auto &node : occupied_nodes.poses) {
    double d = std::sqrt(std::pow(p.x - node.position.x, 2) +
                         std::pow(p.y - node.position.y, 2) +
                         std::pow(p.z - node.position.z, 2));
    if (d < min_clearance)
      min_clearance = d;
  }
  return min_clearance;
}

// Función auxiliar: determina si la línea entre start y end está obstruida por algún nodo ocupado.
bool isPathObstructedByOccupiedNodes(const geometry_msgs::msg::Point &start,
                                     const geometry_msgs::msg::Point &end,
                                     const geometry_msgs::msg::PoseArray &occupied_nodes,
                                     double threshold)
{
  for (const auto &pose : occupied_nodes.poses) {
    double d = distanceFromPointToLineSegment(pose.position, start, end);
    if (d < threshold)
      return true;
  }
  return false;
}

enum class NavigationState { APPROACH, ALIGN, GOAL_REACHED };

class PID {
public:
  PID(double kp, double ki, double kd, double dt)
  : kp_(kp), ki_(ki), kd_(kd), dt_(dt), prev_error_(0.0), integral_(0.0) {}

  double calculate(double setpoint, double measured) {
    double error = setpoint - measured;
    integral_ += error * dt_;
    double derivative = (error - prev_error_) / dt_;
    double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
    prev_error_ = error;
    return output;
  }

  void reset() {
    prev_error_ = 0.0;
    integral_ = 0.0;
  }

private:
  double kp_, ki_, kd_, dt_;
  double prev_error_, integral_;
};

class EnhancedNavigator : public rclcpp::Node {
public:
  EnhancedNavigator()
  : Node("enhanced_navigator"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    num_samples_(30),
    safety_distance_(0.05),
    stuck_counter_(0),
    stuck_threshold_(10),
    avoidance_iterations_(5),
    avoidance_mode_(false),
    avoidance_counter_(0),
    last_distance_error_(std::numeric_limits<double>::max()),
    goal_radius_large_(5.0),
    goal_radius_small_(1.0),
    goal_stable_counter_(0),
    goal_stable_threshold_(20),
    angular_tolerance_(0.1),
    backup_mode_(false),
    backup_speed_(-0.2),
    safe_distance_threshold_(0.1),
    safe_position_set_(false),
    nav_state_(NavigationState::APPROACH),
    align_start_time_(rclcpp::Time(0))
  {
    // Declarar parámetros ajustables
    this->declare_parameter("num_samples", num_samples_);
    this->declare_parameter("safety_distance", safety_distance_);
    this->declare_parameter("stuck_threshold", stuck_threshold_);
    this->declare_parameter("avoidance_iterations", avoidance_iterations_);
    this->declare_parameter("goal_radius_large", goal_radius_large_);
    this->declare_parameter("goal_radius_small", goal_radius_small_);
    this->declare_parameter("goal_stable_threshold", goal_stable_threshold_);
    this->declare_parameter("angular_tolerance", angular_tolerance_);
    this->declare_parameter("backup_speed", backup_speed_);
    this->declare_parameter("safe_distance_threshold", safe_distance_threshold_);
    this->declare_parameter("dt", 0.1);
    this->declare_parameter("align_timeout", 5.0);
    // Umbral para clearance mínimo con nodos ocupados.
    this->declare_parameter("occupied_node_threshold", 0.3);

    num_samples_ = this->get_parameter("num_samples").as_int();
    safety_distance_ = this->get_parameter("safety_distance").as_double();
    stuck_threshold_ = this->get_parameter("stuck_threshold").as_int();
    avoidance_iterations_ = this->get_parameter("avoidance_iterations").as_int();
    goal_radius_large_ = this->get_parameter("goal_radius_large").as_double();
    goal_radius_small_ = this->get_parameter("goal_radius_small").as_double();
    goal_stable_threshold_ = this->get_parameter("goal_stable_threshold").as_int();
    angular_tolerance_ = this->get_parameter("angular_tolerance").as_double();
    backup_speed_ = this->get_parameter("backup_speed").as_double();
    safe_distance_threshold_ = this->get_parameter("safe_distance_threshold").as_double();
    double dt = this->get_parameter("dt").as_double();
    align_timeout_ = this->get_parameter("align_timeout").as_double();
    occupied_node_threshold_ = this->get_parameter("occupied_node_threshold").as_double();

    // Crear controladores PID
    linear_pid_ = std::make_unique<PID>(1.5, 0.0, 0.3, dt);
    // Aumentar la ganancia del PID angular para giros más agresivos.
    angular_pid_ = std::make_unique<PID>(1.5, 0.0, 0.3, dt);

    // Configurar suscripciones y publicadores
    navigable_nodes_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "filtered_navigation_nodes", 10,
      std::bind(&EnhancedNavigator::navigableNodesCallback, this, std::placeholders::_1));

    frontier_point_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "nav_point", 10,
      std::bind(&EnhancedNavigator::frontierPointCallback, this, std::placeholders::_1));

    octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      "octomap", 10,
      std::bind(&EnhancedNavigator::octomapCallback, this, std::placeholders::_1));

    // Odometría para la posición actual
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      std::bind(&EnhancedNavigator::odomCallback, this, std::placeholders::_1));

    // Suscripción a nodos ocupados
    occupied_rejected_nodes_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "occupied_rejected_nodes", 10,
      std::bind(&EnhancedNavigator::occupiedRejectedNodesCallback, this, std::placeholders::_1));

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    goal_reached_pub_ = this->create_publisher<std_msgs::msg::Bool>("goal_reached", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("navigation_markers", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&EnhancedNavigator::controlLoop, this));

    //RCLCPP_INFO(this->get_logger(), "EnhancedNavigator iniciado.");
  }

private:
  // Callbacks de suscripción
  void navigableNodesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    latest_navigable_nodes_ = *msg;
    //RCLCPP_INFO(this->get_logger(), "Actualizados nodos filtrados: %d poses.", static_cast<int>(msg->poses.size()));
  }

  void frontierPointCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    latest_frontier_point_ = *msg;
    //RCLCPP_INFO(this->get_logger(), "Actualizado punto de meta: %d poses recibidas.", static_cast<int>(msg->poses.size()));
    goal_stable_counter_ = 0;
    nav_state_ = NavigationState::APPROACH;
    align_start_time_ = rclcpp::Time(0);
  }

  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
    octree_ = std::shared_ptr<octomap::OcTree>(
      dynamic_cast<octomap::OcTree*>(octomap_msgs::fullMsgToMap(*msg))
    );
    if(octree_) {
      RCLCPP_INFO(this->get_logger(), "Octomap actualizado correctamente.");
    } else {
      RCLCPP_WARN(this->get_logger(), "Fallo al actualizar el Octomap.");
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pose_ = msg->pose.pose;
  }

  void occupiedRejectedNodesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    occupied_rejected_nodes_ = *msg;
    //RCLCPP_DEBUG(this->get_logger(), "Actualizados occupied_rejected_nodes: %d poses.", static_cast<int>(msg->poses.size()));
  }

  // Publicar markers para visualización en RViz
  void publishMarkers() {
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;
    // Nodos filtrados (verde)
    for (const auto &pose : latest_navigable_nodes_.poses) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = this->now();
      marker.ns = "nodos_filtrados";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose = pose;
      marker.scale.x = 0.15;
      marker.scale.y = 0.15;
      marker.scale.z = 0.15;
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker_array.markers.push_back(marker);
    }
    // Nodos ocupados (rojo)
    for (const auto &pose : occupied_rejected_nodes_.poses) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = this->now();
      marker.ns = "nodos_ocupados";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose = pose;
      marker.scale.x = 0.2;
      marker.scale.y = 0.2;
      marker.scale.z = 0.2;
      marker.color.a = 1.0;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker_array.markers.push_back(marker);
    }
    // Nodo candidato (azul)
    visualization_msgs::msg::Marker candidate_marker;
    candidate_marker.header.frame_id = "map";
    candidate_marker.header.stamp = this->now();
    candidate_marker.ns = "nodo_candidato";
    candidate_marker.id = id++;
    candidate_marker.type = visualization_msgs::msg::Marker::SPHERE;
    candidate_marker.action = visualization_msgs::msg::Marker::ADD;
    candidate_marker.pose = candidate_pose_;
    candidate_marker.scale.x = 0.2;
    candidate_marker.scale.y = 0.2;
    candidate_marker.scale.z = 0.2;
    candidate_marker.color.a = 1.0;
    candidate_marker.color.r = 0.0;
    candidate_marker.color.g = 0.0;
    candidate_marker.color.b = 1.0;
    marker_array.markers.push_back(candidate_marker);
    // Posición segura (amarillo)
    visualization_msgs::msg::Marker safe_marker;
    safe_marker.header.frame_id = "map";
    safe_marker.header.stamp = this->now();
    safe_marker.ns = "posicion_segura";
    safe_marker.id = id++;
    safe_marker.type = visualization_msgs::msg::Marker::CUBE;
    safe_marker.action = visualization_msgs::msg::Marker::ADD;
    safe_marker.pose = safe_position_;
    safe_marker.scale.x = 0.25;
    safe_marker.scale.y = 0.25;
    safe_marker.scale.z = 0.25;
    safe_marker.color.a = 1.0;
    safe_marker.color.r = 1.0;
    safe_marker.color.g = 1.0;
    safe_marker.color.b = 0.0;
    marker_array.markers.push_back(safe_marker);

    marker_pub_->publish(marker_array);
  }

  // Verifica si la trayectoria entre dos puntos está libre según octomap.
  bool isPathFree(const geometry_msgs::msg::Point &start, const geometry_msgs::msg::Point &end) {
    if (!octree_) return true;
    for (int i = 0; i <= num_samples_; i++) {
      double t = static_cast<double>(i) / num_samples_;
      double x = start.x + t * (end.x - start.x);
      double y = start.y + t * (end.y - start.y);
      double z = start.z + t * (end.z - start.z);
      octomap::OcTreeNode* node = octree_->search(x, y, z);
      if (node && octree_->isNodeOccupied(node)) {
        RCLCPP_DEBUG(this->get_logger(), "Obstáculo detectado en: (%.2f, %.2f, %.2f)", x, y, z);
        return false;
      }
    }
    return true;
  }

  // Publica que se alcanzó el goal y detiene el robot.
  void publishGoalReached() {
    geometry_msgs::msg::Twist stop;
    stop.linear.x = 0.0;
    stop.angular.z = 0.0;
    cmd_vel_pub_->publish(stop);
    std_msgs::msg::Bool goal_msg;
    goal_msg.data = true;
    //goal_reached_pub_->publish(goal_msg);
    linear_pid_->reset();
    angular_pid_->reset();
  }

  void controlLoop() {
    if (latest_navigable_nodes_.poses.empty() || latest_frontier_point_.poses.empty()) {
      RCLCPP_DEBUG(this->get_logger(), "Datos insuficientes (nodos filtrados o meta vacíos).");
      return;
    }

    // Publicar markers para RViz.
    publishMarkers();

    // Obtener la posición actual del robot (odometría).
    double robot_x = current_pose_.position.x;
    double robot_y = current_pose_.position.y;
    double robot_z = current_pose_.position.z;

    // Tomar el primer goal del PoseArray.
    geometry_msgs::msg::Pose frontier_pose = latest_frontier_point_.poses[0];

    // Seleccionar un nodo candidato entre los nodos filtrados.
    geometry_msgs::msg::Pose target_pose;
    double best_cost = std::numeric_limits<double>::max();
    bool found_candidate = false;
    geometry_msgs::msg::Point robot_point;
    robot_point.x = robot_x;
    robot_point.y = robot_y;
    robot_point.z = robot_z;

    for (const auto &pose : latest_navigable_nodes_.poses) {
      if (!isPathFree(frontier_pose.position, pose.position))
        continue;
      if (!isPathFree(robot_point, pose.position))
        continue;
      if (isPathObstructedByOccupiedNodes(robot_point, pose.position, occupied_rejected_nodes_, occupied_node_threshold_))
        continue;
      double clearance = getClearance(occupied_rejected_nodes_, pose.position);
      if (clearance < occupied_node_threshold_) {
        RCLCPP_DEBUG(this->get_logger(), "Candidato rechazado por baja clearance: %.2f", clearance);
        continue;
      }
      double dx = pose.position.x - frontier_pose.position.x;
      double dy = pose.position.y - frontier_pose.position.y;
      double cost = std::sqrt(dx * dx + dy * dy);
      if (cost < best_cost) {
        best_cost = cost;
        target_pose = pose;
        found_candidate = true;
      }
    }

    if (!found_candidate) {
      RCLCPP_WARN(this->get_logger(), "No se encontró un candidato libre de obstáculos.");
      geometry_msgs::msg::Twist stop;
      cmd_vel_pub_->publish(stop);
      return;
    }
    candidate_pose_ = target_pose;  // Para visualización.

    // Calcular errores en posición y orientación usando odometría.
    double error_x = target_pose.position.x - robot_x;
    double error_y = target_pose.position.y - robot_y;
    double distance_error = std::sqrt(error_x * error_x + error_y * error_y);
    double angle_to_target = std::atan2(error_y, error_x);
    tf2::Quaternion q;
    tf2::fromMsg(current_pose_.orientation, q);
    double roll, pitch, current_yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, current_yaw);
    double angle_error = angle_to_target - current_yaw;
    while (angle_error > M_PI)  angle_error -= 2 * M_PI;
    while (angle_error < -M_PI) angle_error += 2 * M_PI;

    // Estrategia de conducción: decidir si avanzar o retroceder.
    double cos_error = std::cos(angle_error);
    double v = 0.0;
    double omega = 0.0;
    if (cos_error >= 0) {
      v = k_linear * distance_error;
    } else {
      v = -k_linear * distance_error;
      angle_error = -angle_error;
    }
    double L = distance_error;
    double curvature = (L > 0 ? (2.0 * sin(angle_error)) / L : 0.0);
    omega = curvature * std::fabs(v);
    if (distance_error < 0.2)
      v *= 0.5;
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = v;
    cmd_vel.angular.z = omega;

    // Integrar backup basado en progreso insuficiente.
    double progress = last_distance_error_ - distance_error;
    if (progress > 0.005) {
      safe_position_.position.x = robot_x;
      safe_position_.position.y = robot_y;
      safe_position_.position.z = robot_z;
      safe_position_.orientation = current_pose_.orientation;
      safe_position_set_ = true;
    } else {
      stuck_counter_++;
      //RCLCPP_DEBUG(this->get_logger(), "Progreso insuficiente. Contador de estancamiento: %d", stuck_counter_);
    }
    last_distance_error_ = distance_error;

    // Activar backup si se excede el umbral o se detecta un obstáculo muy cerca.
    if (stuck_counter_ >= stuck_threshold_ && safe_position_set_) {
      backup_mode_ = true;
      //RCLCPP_WARN(this->get_logger(), "Robot estancado. Activando modo backup.");
      stuck_counter_ = 0;
    }
    bool obstacle_near = false;
    for (const auto &pose : occupied_rejected_nodes_.poses) {
      double d = std::sqrt(std::pow(robot_x - pose.position.x, 2) +
                           std::pow(robot_y - pose.position.y, 2) +
                           std::pow(robot_z - pose.position.z, 2));
      if (d < occupied_node_threshold_) {
        obstacle_near = true;
        break;
      }
    }
    if (obstacle_near) {
      //RCLCPP_WARN(this->get_logger(), "Obstáculo muy cerca detectado. Activando modo backup.");
      backup_mode_ = true;
    }

    // Máquina de estados para la navegación.
    switch (nav_state_) {
      case NavigationState::APPROACH: {
        if (distance_error < goal_radius_small_) {
          nav_state_ = NavigationState::ALIGN;
          //RCLCPP_INFO(this->get_logger(), "Cambiando a estado ALIGN (cerca del goal).");
          geometry_msgs::msg::Twist stop;
          cmd_vel_pub_->publish(stop);
          align_start_time_ = this->now();
          return;
        }
        cmd_vel_pub_->publish(cmd_vel);
        break;
      }
      case NavigationState::ALIGN: {
        if (align_start_time_.nanoseconds() == 0) {
          align_start_time_ = this->now();
        }
        auto current_time = this->now();
        if ((current_time - align_start_time_).seconds() > align_timeout_) {
          //RCLCPP_WARN(this->get_logger(), "Timeout en alineación. Forzando finalización.");
          publishGoalReached();
          nav_state_ = NavigationState::GOAL_REACHED;
          align_start_time_ = rclcpp::Time(0);
          break;
        }
        if (std::fabs(angle_error) > angular_tolerance_) {
          geometry_msgs::msg::Twist rotate_cmd;
          rotate_cmd.linear.x = 0.0;
          double angular_correction = angular_pid_->calculate(0.0, angle_error);
          if (angular_correction > 1.0)
            angular_correction = 1.0;
          else if (angular_correction < -1.0)
            angular_correction = -1.0;
          rotate_cmd.angular.z = angular_correction;
          cmd_vel_pub_->publish(rotate_cmd);
        } else {
          //RCLCPP_INFO(this->get_logger(), "Meta alcanzada de forma estable (alineación completa).");
          publishGoalReached();
          nav_state_ = NavigationState::GOAL_REACHED;
          align_start_time_ = rclcpp::Time(0);
        }
        break;
      }
      case NavigationState::GOAL_REACHED: {
        geometry_msgs::msg::Twist stop;
        cmd_vel_pub_->publish(stop);
        break;
      }
    }

    // Modo backup: retroceder hasta alcanzar la posición segura.
    if (backup_mode_) {
      double dx = safe_position_.position.x - robot_x;
      double dy = safe_position_.position.y - robot_y;
      double backup_distance = std::sqrt(dx * dx + dy * dy);
      if (backup_distance > safe_distance_threshold_) {
        geometry_msgs::msg::Twist backup_cmd;
        backup_cmd.linear.x = backup_speed_;
        backup_cmd.angular.z = 0.0;
        cmd_vel_pub_->publish(backup_cmd);
        //RCLCPP_INFO(this->get_logger(), "Modo backup: retrocediendo (distancia: %.2f)", backup_distance);
        return;
      } else {
        backup_mode_ = false;
        //RCLCPP_INFO(this->get_logger(), "Posición segura alcanzada. Reiniciando control normal.");
      }
    }
  }

  // Subscripciones y publicadores
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr navigable_nodes_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr frontier_point_sub_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr occupied_rejected_nodes_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reached_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Datos actualizados
  geometry_msgs::msg::PoseArray latest_navigable_nodes_;
  geometry_msgs::msg::PoseArray latest_frontier_point_;
  geometry_msgs::msg::PoseArray occupied_rejected_nodes_;
  std::shared_ptr<octomap::OcTree> octree_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Variables para control y detección de estancamiento
  int num_samples_;
  double safety_distance_;
  int stuck_counter_;
  int stuck_threshold_;
  int avoidance_iterations_;
  bool avoidance_mode_;
  int avoidance_counter_;
  double last_distance_error_;
  double goal_radius_large_;
  double goal_radius_small_;
  int goal_stable_counter_;
  int goal_stable_threshold_;
  double angular_tolerance_;

  // Variables para backup
  geometry_msgs::msg::Pose safe_position_;
  bool safe_position_set_;
  bool backup_mode_;
  double backup_speed_;
  double safe_distance_threshold_;
  double occupied_node_threshold_;

  // Controladores PID
  std::unique_ptr<PID> linear_pid_;
  std::unique_ptr<PID> angular_pid_;

  // Ganancias para pure pursuit
  const double k_linear = 1.0;
  const double k_angular = 1.0;

  // Estado de navegación
  NavigationState nav_state_;

  // Timer para el estado ALIGN
  double align_timeout_;
  rclcpp::Time align_start_time_;

  // Posición actual obtenida de odom
  geometry_msgs::msg::Pose current_pose_;

  // Nodo candidato seleccionado (para visualización)
  geometry_msgs::msg::Pose candidate_pose_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EnhancedNavigator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

