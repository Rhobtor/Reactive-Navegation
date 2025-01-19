#include <rclcpp/rclcpp.hpp>
#include <octomap/octomap.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <visualization_msgs/msg/marker_array.hpp>

class CheckMapDisplayNode : public rclcpp::Node {
public:
    CheckMapDisplayNode() : Node("check_map") {
        octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
            "/octomap_binary", 10, std::bind(&CheckMapDisplayNode::octomapCallback, this, std::placeholders::_1));

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/octomap_markers", 10);

        RCLCPP_INFO(this->get_logger(), "CheckMapDisplayNode initialized!");
    }

private:
    void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
        octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
        auto octree = dynamic_cast<octomap::OcTree*>(tree);
        if (!octree) {
            RCLCPP_ERROR(this->get_logger(), "Failed to cast Octomap to OcTree.");
            return;
        }

        visualization_msgs::msg::MarkerArray marker_array;

        // Iterar por los nodos ocupados del Octree
        for (auto it = octree->begin_leafs(); it != octree->end_leafs(); ++it) {
            if (octree->isNodeOccupied(*it)) {
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = "chasis_1";  // Asegúrate de usar el frame adecuado
                marker.header.stamp = this->now();
                marker.ns = "octomap";
                marker.id = marker_array.markers.size();
                marker.type = visualization_msgs::msg::Marker::CUBE;
                marker.action = visualization_msgs::msg::Marker::ADD;

                // Posición del cubo
                marker.pose.position.x = it.getX();
                marker.pose.position.y = it.getY();
                marker.pose.position.z = it.getZ();
                marker.pose.orientation.w = 1.0;

                // Tamaño del cubo (basado en la resolución del Octree)
                double size = it.getSize();
                marker.scale.x = size;
                marker.scale.y = size;
                marker.scale.z = size;

                // Color del cubo
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
                marker.color.a = 0.8;

                marker_array.markers.push_back(marker);
            }
        }

        // Publicar los marcadores
        marker_pub_->publish(marker_array);
        delete tree;  // Liberar memoria
    }

    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CheckMapDisplayNode>());
    rclcpp::shutdown();
    return 0;
}
