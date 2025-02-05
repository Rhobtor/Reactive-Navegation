#include <gazebo/common/Plugin.hh>
#include <rclcpp/rclcpp.hpp>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <memory>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_ros/transform_broadcaster.h"

#define VEHICLE_LENGTH 2.1
#define VEHICLE_WIDTH 1.7
#define WHEEL_RADIUS 0.7

namespace gazebo_plugins
{
class OdometryPlugin : public gazebo::ModelPlugin
{
public:
    OdometryPlugin() : gazebo::ModelPlugin() {}

    void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
    {
        this->model_ = model;
        this->ros_node_ = gazebo_ros::Node::Get(sdf);

        RCLCPP_INFO(ros_node_->get_logger(), "OdometryPlugin: Model name: %s", model->GetName().c_str());

        this->InitJoints(sdf);

        // Crear el publicador de la odometría
        this->odom_pub_ = this->ros_node_->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        this->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
            std::bind(&OdometryPlugin::OnUpdate, this));
    }

private:
    void InitJoints(sdf::ElementPtr sdf)
    {
        this->left_front_wheel_joint_ = model_->GetJoint("Revolute 17");
        this->right_front_wheel_joint_ = model_->GetJoint("Revolute 16");
        this->left_back_wheel_joint_ = model_->GetJoint("Revolute 5");
        this->right_back_wheel_joint_ = model_->GetJoint("Revolute 6");
        this->left_front_speed_wheel_joint_ = model_->GetJoint("Revolute 8");
        this->right_front_speed_wheel_joint_ = model_->GetJoint("Revolute 7");

        if (!this->left_back_wheel_joint_ || !this->right_back_wheel_joint_)
        {
            RCLCPP_ERROR(ros_node_->get_logger(), "OdometryPlugin: Back Wheels joints not found");
            return;
        }
    }

void OnUpdate()
{
    auto current_time = this->ros_node_->get_clock()->now().seconds();
    double delta_time = current_time - last_update_time_;

    if (last_update_time_ == 0.0) {
        last_update_time_ = current_time;
        return; // Esperar a la siguiente iteración para evitar cálculos iniciales erróneos
    }

    // Obtener velocidades de las ruedas traseras
    double left_rear_velocity = left_back_wheel_joint_->GetVelocity(0) * WHEEL_RADIUS;
    double right_rear_velocity = right_back_wheel_joint_->GetVelocity(0) * WHEEL_RADIUS;

    // Obtener el ángulo de dirección promedio de las ruedas delanteras
    double left_steering_angle = left_front_speed_wheel_joint_->Position(0);
    double right_steering_angle = right_front_speed_wheel_joint_->Position(0);
    double steering_angle = (left_steering_angle + -1*(right_steering_angle)) / 2.0;

    // Calcular velocidad lineal y angular
    double linear_velocity = (left_rear_velocity + -1*(right_rear_velocity)) / 2.0;
    double angular_velocity = linear_velocity * tan(steering_angle) / VEHICLE_LENGTH;

    // Actualizar la posición y orientación solo si el delta_time es válido
    if (delta_time > 0.0) {
        // Actualización de la posición basada en la orientación actual
        x_ += linear_velocity * cos(theta_) * delta_time;
        y_ += linear_velocity * sin(theta_) * delta_time;

        // Actualización de la orientación (theta)
        theta_ += angular_velocity * delta_time;

        // Para asegurar que theta esté en el rango [-pi, pi]
        if (theta_ > M_PI) theta_ -= 2.0 * M_PI;
        else if (theta_ < -M_PI) theta_ += 2.0 * M_PI;
    }

    // Crear y publicar el mensaje de odometría
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = ros_node_->get_clock()->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    // Posición
    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;

    // Orientación
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    // Velocidad lineal y angular
    odom_msg.twist.twist.linear.x = linear_velocity;
    odom_msg.twist.twist.angular.z = angular_velocity;

    this->odom_pub_->publish(odom_msg);



    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = ros_node_->get_clock()->now();
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.translation.z = 0.0;

    tf2::Quaternion t;
    q.setRPY(0, 0, theta_);
    transform.transform.rotation.x = t.x();
    transform.transform.rotation.y = t.y();
    transform.transform.rotation.z = t.z();
    transform.transform.rotation.w = t.w();

    tf_broadcaster_->sendTransform(transform);


    // std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_direction_right_;
    // geometry_msgs::msg::TransformStamped transform_right_direction;
    // transform_right_direction.header.stamp = ros_node_->get_clock()->now();
    // transform_right_direction.header.frame_id = "suspension_front_right_1";
    // transform_right_direction.child_frame_id = "direction_front_right_1";
    // transform_right_direction.transform.translation.x = x_;
    // transform_right_direction.transform.translation.y = y_;
    // transform_right_direction.transform.translation.z = 0.0;

    // tf2::Quaternion right_direction;
    // q.setRPY(0, 0, theta_);
    // transform_right_direction.transform.rotation.x = right_direction.x();
    // transform_right_direction.transform.rotation.y = right_direction.y();
    // transform_right_direction.transform.rotation.z = right_direction.z();
    // transform_right_direction.transform.rotation.w = right_direction.w();

    // tf_broadcaster_direction_right_->sendTransform(transform_right_direction);


    // Actualizar last_update_time para la siguiente iteración
    last_update_time_ = current_time;
}


    gazebo::physics::ModelPtr model_;
    std::shared_ptr<gazebo_ros::Node> ros_node_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    gazebo::physics::JointPtr left_front_wheel_joint_;
    gazebo::physics::JointPtr right_front_wheel_joint_;
    gazebo::physics::JointPtr left_back_wheel_joint_;
    gazebo::physics::JointPtr right_back_wheel_joint_;
    gazebo::physics::JointPtr left_front_speed_wheel_joint_;
    gazebo::physics::JointPtr right_front_speed_wheel_joint_;
    gazebo::event::ConnectionPtr update_connection_;
    
    double x_ = 0.0;
    double y_ = 0.0;
    double theta_ = 0.0;  // Orientación en radianes
    double last_update_time_ = 0.0;
}; 

GZ_REGISTER_MODEL_PLUGIN(OdometryPlugin)
}  // namespace gazebo_plugins
