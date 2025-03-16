#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>

class GoalReachedNode : public rclcpp::Node
{
public:
  GoalReachedNode()
  : Node("goal_reached_node"), goal_received_(false), odom_received_(false)
  {
    // Declarar parámetro para el umbral de llegada al goal (en metros)
    this->declare_parameter("goal_threshold", 3.0);
    goal_threshold_ = this->get_parameter("goal_threshold").as_double();

    // Suscripción al topic "goal" (PoseArray)
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "goal", 10,
      std::bind(&GoalReachedNode::goalCallback, this, std::placeholders::_1));

    // Suscripción al topic "odom" (nav_msgs/Odometry)
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      std::bind(&GoalReachedNode::odomCallback, this, std::placeholders::_1));

    // Publicador en el topic "goal_reached" (Bool)
    goal_reached_pub_ = this->create_publisher<std_msgs::msg::Bool>("goal_reached", 10);

    // Timer de control que se ejecuta cada 100ms
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&GoalReachedNode::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "GoalReachedNode iniciado usando odom y PoseArray para el goal.");
  }

private:
  // Callback para recibir el goal (PoseArray)
  void goalCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    if (msg->poses.empty()) {
      RCLCPP_WARN(this->get_logger(), "Goal recibido vacío (PoseArray sin elementos).");
      return;
    }
    // Usamos el primer elemento del PoseArray como goal
    goal_pose_ = msg->poses[0];
    goal_received_ = true;
    RCLCPP_INFO(this->get_logger(), "Nuevo goal recibido: [%.2f, %.2f, %.2f]",
                goal_pose_.position.x,
                goal_pose_.position.y,
                goal_pose_.position.z);
  }

  // Callback para obtener la odometría
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_pose_ = msg->pose.pose;
    odom_received_ = true;
    RCLCPP_DEBUG(this->get_logger(), "Odom recibido: [%.2f, %.2f, %.2f]",
                 odom_pose_.position.x,
                 odom_pose_.position.y,
                 odom_pose_.position.z);
  }

  // Loop de control: compara la posición actual (odom) con el goal
  void controlLoop()
  {
    RCLCPP_DEBUG(this->get_logger(), "Control loop: goal_received: %d, odom_received: %d", goal_received_, odom_received_);
    if (!goal_received_ || !odom_received_) {
      return;
    }

    double robot_x = odom_pose_.position.x;
    double robot_y = odom_pose_.position.y;
    double robot_z = odom_pose_.position.z;

    double goal_x = goal_pose_.position.x;
    double goal_y = goal_pose_.position.y;
    double goal_z = goal_pose_.position.z;

    double dx = goal_x - robot_x;
    double dy = goal_y - robot_y;
    double dz = goal_z - robot_z;
    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    // Logs para verificar los valores calculados
    RCLCPP_INFO(this->get_logger(), "Robot: [%.2f, %.2f, %.2f]", robot_x, robot_y, robot_z);
    RCLCPP_INFO(this->get_logger(), "Goal: [%.2f, %.2f, %.2f]", goal_x, goal_y, goal_z);
    RCLCPP_INFO(this->get_logger(), "dx: %.2f, dy: %.2f, dz: %.2f, distance: %.2f", dx, dy, dz, distance);

    if (distance < goal_threshold_) {
      std_msgs::msg::Bool reached_msg;
      reached_msg.data = true;
      goal_reached_pub_->publish(reached_msg);
      RCLCPP_INFO(this->get_logger(), "Goal alcanzado. Distancia: %.2f m", distance);
      // Reiniciamos la bandera para evitar múltiples publicaciones
      goal_received_ = false;
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reached_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Usamos solo el primer pose del PoseArray como goal
  geometry_msgs::msg::Pose goal_pose_;
  geometry_msgs::msg::Pose odom_pose_;
  bool goal_received_;
  bool odom_received_;
  double goal_threshold_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GoalReachedNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
