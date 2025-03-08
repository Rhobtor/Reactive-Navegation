#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <octomap/octomap.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <cmath>
#include <limits>

// Clase PID sencilla para control
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
    num_samples_(20),
    safety_distance_(0.05),
    stuck_counter_(0),
    stuck_threshold_(10),
    avoidance_iterations_(5),
    avoidance_mode_(false),
    avoidance_counter_(0),
    last_distance_error_(std::numeric_limits<double>::max())
  {
    // Declarar parámetros ajustables
    this->declare_parameter("num_samples", num_samples_);
    this->declare_parameter("safety_distance", safety_distance_);
    this->declare_parameter("stuck_threshold", stuck_threshold_);
    this->declare_parameter("avoidance_iterations", avoidance_iterations_);
    this->declare_parameter("linear_kp", 0.5);
    this->declare_parameter("linear_ki", 0.0);
    this->declare_parameter("linear_kd", 0.1);
    this->declare_parameter("angular_kp", 1.0);
    this->declare_parameter("angular_ki", 0.0);
    this->declare_parameter("angular_kd", 0.2);
    this->declare_parameter("dt", 0.1); // periodo del timer en segundos
    // Nuevo parámetro para el radio en el que se considera alcanzado el goal.
    this->declare_parameter("goal_radius", 0.6);
    
    num_samples_ = this->get_parameter("num_samples").as_int();
    safety_distance_ = this->get_parameter("safety_distance").as_double();
    stuck_threshold_ = this->get_parameter("stuck_threshold").as_int();
    avoidance_iterations_ = this->get_parameter("avoidance_iterations").as_int();
    double dt = this->get_parameter("dt").as_double();
    goal_radius_ = this->get_parameter("goal_radius").as_double();

    // Crear los controladores PID
    linear_pid_ = std::make_unique<PID>(
      this->get_parameter("linear_kp").as_double(),
      this->get_parameter("linear_ki").as_double(),
      this->get_parameter("linear_kd").as_double(),
      dt);
    angular_pid_ = std::make_unique<PID>(
      this->get_parameter("angular_kp").as_double(),
      this->get_parameter("angular_ki").as_double(),
      this->get_parameter("angular_kd").as_double(),
      dt);

    // Suscripciones y publicadores
    navigable_nodes_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "filtered_navigation_nodes", 10,
      std::bind(&EnhancedNavigator::navigableNodesCallback, this, std::placeholders::_1));
    
    frontier_point_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "goal", 10,
      std::bind(&EnhancedNavigator::frontierPointCallback, this, std::placeholders::_1));
    
    octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      "octomap", 10,
      std::bind(&EnhancedNavigator::octomapCallback, this, std::placeholders::_1));
    
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    goal_reached_pub_ = this->create_publisher<std_msgs::msg::Bool>("goal_reached", 10);
    
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&EnhancedNavigator::controlLoop, this));
    
    RCLCPP_INFO(this->get_logger(), "EnhancedNavigator iniciado.");
  }

private:
  void navigableNodesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    latest_navigable_nodes_ = *msg;
    RCLCPP_INFO(this->get_logger(), "Actualizados nodos filtrados: %d poses.", static_cast<int>(msg->poses.size()));
  }
  
  void frontierPointCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    latest_frontier_point_ = *msg;
    RCLCPP_INFO(this->get_logger(), "Actualizado punto de meta: %d poses recibidas.", static_cast<int>(msg->poses.size()));
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
  
  // Verifica si el camino entre dos puntos está libre
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
  
  // Lazo de control principal
  void controlLoop() {
    if (latest_navigable_nodes_.poses.empty() || latest_frontier_point_.poses.empty()) {
      RCLCPP_DEBUG(this->get_logger(), "Datos insuficientes (nodos filtrados o meta vacíos).");
      return;
    }
    
    // Obtener la posición actual del robot usando TF
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
    
    tf2::Quaternion q;
    tf2::fromMsg(transformStamped.transform.rotation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    
    // Se toma el primer goal recibido como meta
    geometry_msgs::msg::Pose frontier_pose = latest_frontier_point_.poses[0];
    
    // Selección del nodo candidato de entre los nodos filtrados
    geometry_msgs::msg::Pose target_pose;
    double min_dist = std::numeric_limits<double>::max();
    bool found_candidate = false;
    for (const auto &pose : latest_navigable_nodes_.poses) {
      if (!isPathFree(frontier_pose.position, pose.position))
        continue;
      geometry_msgs::msg::Point robot_point;
      robot_point.x = robot_x;
      robot_point.y = robot_y;
      robot_point.z = robot_z;
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
      RCLCPP_WARN(this->get_logger(), "No se encontró un nodo candidato con trayectoria libre.");
      geometry_msgs::msg::Twist stop;
      stop.linear.x = 0.0;
      stop.angular.z = 0.0;
      cmd_vel_pub_->publish(stop);
      return;
    }
    
    // Re-evaluación de la trayectoria desde el robot al nodo candidato.
    geometry_msgs::msg::Point robot_point;
    robot_point.x = robot_x;
    robot_point.y = robot_y;
    robot_point.z = robot_z;
    if (!isPathFree(robot_point, target_pose.position)) {
      RCLCPP_WARN(this->get_logger(), "Camino a nodo candidato bloqueado. Re-evaluando...");
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
    
    // Calcular errores en posición y orientación
    double error_x = target_pose.position.x - robot_x;
    double error_y = target_pose.position.y - robot_y;
    double distance_error = std::sqrt(error_x * error_x + error_y * error_y);
    double angle_to_target = std::atan2(error_y, error_x);
    double angle_error = angle_to_target - yaw;
    while (angle_error > M_PI)  angle_error -= 2 * M_PI;
    while (angle_error < -M_PI) angle_error += 2 * M_PI;
    
    // Control PID para generar comandos de velocidad
    double linear_cmd = -linear_pid_->calculate(0.0, distance_error);
    double angular_cmd = -angular_pid_->calculate(0.0, angle_error);
    
    // Velocidad adaptativa: si la distancia es muy pequeña, reducir la velocidad lineal
    if (distance_error < 0.2) {
      linear_cmd *= 0.5;
    }
    
    // Manejo de estancamiento: si el progreso es mínimo, incrementar contador y activar modo evasivo si es necesario.
    double progress = last_distance_error_ - distance_error;
    if (progress < 0.001) {
      stuck_counter_++;
      RCLCPP_DEBUG(this->get_logger(), "Progreso insignificante. Contador de estancamiento: %d", stuck_counter_);
    } else {
      stuck_counter_ = 0;
    }
    last_distance_error_ = distance_error;
    
    if (stuck_counter_ >= stuck_threshold_) {
      avoidance_mode_ = true;
      avoidance_counter_ = avoidance_iterations_;
      RCLCPP_WARN(this->get_logger(), "Robot estancado. Activando evasión reactiva.");
      stuck_counter_ = 0;
    }
    
    geometry_msgs::msg::Twist cmd_vel;
    if (avoidance_mode_) {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.5;
      avoidance_counter_--;
      if (avoidance_counter_ <= 0) {
        avoidance_mode_ = false;
        RCLCPP_INFO(this->get_logger(), "Finalizando modo evasión, re-evaluando trayectoria.");
      }
      cmd_vel_pub_->publish(cmd_vel);
      return;
    }
    
    // Nuevo: considerar alcanzado el goal si el robot está dentro de un radio de "goal_radius_"
    bool goal_reached = (distance_error < goal_radius_);
    if (goal_reached) {
      RCLCPP_INFO(this->get_logger(), "Meta alcanzada: error=%.2f", distance_error);
      geometry_msgs::msg::Twist stop;
      stop.linear.x = 0.0;
      stop.angular.z = 0.0;
      cmd_vel_pub_->publish(stop);
      std_msgs::msg::Bool goal_msg;
      goal_msg.data = true;
      goal_reached_pub_->publish(goal_msg);
      // Reiniciar PID para evitar acumulación de integral
      linear_pid_->reset();
      angular_pid_->reset();
    } else {
      cmd_vel.linear.x = linear_cmd;
      cmd_vel.angular.z = angular_cmd;
      cmd_vel_pub_->publish(cmd_vel);
    }
  }
  
  // Subscripciones y publicadores
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr navigable_nodes_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr frontier_point_sub_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reached_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Datos actualizados
  geometry_msgs::msg::PoseArray latest_navigable_nodes_; // Del topic filtered_navigation_nodes
  geometry_msgs::msg::PoseArray latest_frontier_point_;
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
  double goal_radius_;
  
  // Controladores PID
  std::unique_ptr<PID> linear_pid_;
  std::unique_ptr<PID> angular_pid_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EnhancedNavigator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
