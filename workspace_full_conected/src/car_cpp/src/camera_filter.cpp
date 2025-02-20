#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>

class GroundConnectivityFilterNode : public rclcpp::Node {
public:
  GroundConnectivityFilterNode()
  : Node("ground_connectivity_filter_node"),
    ransac_distance_threshold_(0.05),
    connectivity_threshold_(0.5)  // umbral en metros para considerar conectividad
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/depth_camera/points", 10,
      std::bind(&GroundConnectivityFilterNode::pointCloudCallback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_points", 10);
    RCLCPP_INFO(this->get_logger(), "Nodo de filtrado por conectividad al suelo iniciado.");
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // Convertir el mensaje ROS a una nube de puntos PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud);
    if (cloud->empty()) {
      RCLCPP_WARN(this->get_logger(), "Nube vacía.");
      return;
    }

    // --- Segmentación RANSAC para detectar el suelo ---
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(ransac_distance_threshold_);
    seg.setInputCloud(cloud);

    pcl::PointIndices::Ptr ground_inliers(new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    seg.segment(*ground_inliers, *coefficients);
    if (ground_inliers->indices.empty()) {
      RCLCPP_WARN(this->get_logger(), "No se detectó el plano de suelo.");
      return;
    }

    // Verificar que el plano sea casi horizontal (se espera que la componente z de la normal sea dominante)
    if (std::abs(coefficients->values[2]) < 0.9) {
      RCLCPP_WARN(this->get_logger(), "El plano detectado no es horizontal.");
      // Podrías optar por no filtrar en este caso, o continuar
    }

    // Extraer los puntos que se ajustan al plano (inliers)
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(ground_inliers);
    extract.setNegative(false);
    extract.filter(*ground_cloud);
    RCLCPP_INFO(this->get_logger(), "Puntos de suelo detectados: %zu", ground_cloud->points.size());

    // --- Construir kd-tree de los puntos del suelo ---
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(ground_cloud);

    // --- Filtrado por conectividad: conservar solo puntos conectados al suelo ---
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto & pt : cloud->points) {
      std::vector<int> nearest_indices;
      std::vector<float> nearest_sqr_dists;
      if (kdtree.nearestKSearch(pt, 1, nearest_indices, nearest_sqr_dists) > 0) {
        if (nearest_sqr_dists[0] < connectivity_threshold_ * connectivity_threshold_) {
          // El punto está a menos de 'connectivity_threshold_' metros de algún punto del suelo
          filtered_cloud->points.push_back(pt);
        }
      }
    }
    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = true;
    RCLCPP_INFO(this->get_logger(), "Nube filtrada publicada con %zu puntos de %zu originales.",
                filtered_cloud->points.size(), cloud->points.size());

    // Convertir la nube filtrada a mensaje ROS y publicarla
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*filtered_cloud, output);
    output.header = msg->header;
    publisher_->publish(output);
  }

  double ransac_distance_threshold_;
  double connectivity_threshold_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GroundConnectivityFilterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
