#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

class OdomTFPublisher : public rclcpp::Node
{
public:
  OdomTFPublisher()
  : Node("odometry_node"), x_(0.0), y_(0.0), yaw_(0.0),
    linear_vel_(0.0), angular_vel_(0.0)
  {
    // Declaración de parámetros
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("base_frame", "base_footprint");

    odom_frame_ = this->get_parameter("odom_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();

    // Publicador de Odometry
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    // Creación del TF Broadcaster para enviar transformaciones dinámicas
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Suscripción para recibir mensajes de Twist (velocidades)
    twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/wheel_torque_command", 10,
      std::bind(&OdomTFPublisher::twist_callback, this, std::placeholders::_1));

    // Timer para actualizar la odometría a 10 Hz (0.1 segundos)
    timer_ = this->create_wall_timer(
      100ms, std::bind(&OdomTFPublisher::timer_callback, this));

    last_time_ = this->now();
  }

private:
  // Callback del suscriptor de Twist
  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    linear_vel_ = msg->linear.x;
    angular_vel_ = msg->angular.z;
  }

  // Callback del timer: actualiza la odometría y transmite los TF
  void timer_callback()
  {
    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_time_).seconds();

    // Actualiza la posición y orientación usando integración simple
    double delta_x   = linear_vel_ * std::cos(yaw_) * dt;
    double delta_y   = linear_vel_ * std::sin(yaw_) * dt;
    double delta_yaw = angular_vel_ * dt;

    x_   += delta_x;
    y_   += delta_y;
    yaw_ += delta_yaw;

    // Construye y envía la transformación dinámica (odom -> base_footprint)
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = current_time;
    t.header.frame_id = odom_frame_;            // por defecto "odom"
    t.child_frame_id  = "base_link";         // se usa "base_footprint" como en el ejemplo original
    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_);
    q.normalize();
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);

    // Construye y publica el mensaje de odometría
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = "base_link";

    // Posición y orientación
    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    // Velocidades lineal y angular
    odom_msg.twist.twist.linear.x = linear_vel_;
    odom_msg.twist.twist.angular.z = angular_vel_;

    odom_pub_->publish(odom_msg);

    last_time_ = current_time;
  }

  // Variables miembro
  std::string odom_frame_;
  std::string base_frame_;

  double x_, y_, yaw_;
  double linear_vel_, angular_vel_;

  rclcpp::Time last_time_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdomTFPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
