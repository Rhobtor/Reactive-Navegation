#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <cmath>
#include <limits>
#include <memory>

class MapEntropyNode : public rclcpp::Node
{
public:
  MapEntropyNode() : Node("map_entropy_node"), prev_entropy_(0.0), first_update_(true)
  {
    octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      "/octomap_full", 10,
      std::bind(&MapEntropyNode::octomapCallback, this, std::placeholders::_1)
    );
    reward_pub_ = this->create_publisher<std_msgs::msg::Float64>("exploration_reward", 10);
    RCLCPP_INFO(this->get_logger(), "Nodo de entropía del mapa iniciado.");
  }

private:
  double compute_map_entropy(const octomap::OcTree &tree)
  {
    double total_entropy = 0.0;
    const double epsilon = 1e-6;
    for (octomap::OcTree::leaf_iterator it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it)
    {
      double p = it->getOccupancy();
      // Limitar p a [epsilon, 1-epsilon] para evitar log(0)
      p = std::min(std::max(p, epsilon), 1 - epsilon);
      double H = -p * std::log(p) - (1 - p) * std::log(1 - p);
      total_entropy += H;
    }
    return total_entropy;
  }

  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
  {
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
    
    double current_entropy = compute_map_entropy(*tree);
    double reward = 0.0;
    if (first_update_) {
      // En la primera actualización no hay ganancia
      first_update_ = false;
      reward = 0.0;
    } else {
      reward = prev_entropy_ - current_entropy;
    }
    prev_entropy_ = current_entropy;

    // Publicar la ganancia de información como recompensa
    std_msgs::msg::Float64 reward_msg;
    reward_msg.data = reward;
    reward_pub_->publish(reward_msg);

    RCLCPP_INFO(this->get_logger(), "Entropía actual: %.2f, ganancia: %.2f", current_entropy, reward);
  }

  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr reward_pub_;

  double prev_entropy_;
  bool first_update_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapEntropyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
