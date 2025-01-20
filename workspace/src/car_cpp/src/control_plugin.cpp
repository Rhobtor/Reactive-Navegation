#include <gazebo/common/Plugin.hh>
#include <rclcpp/rclcpp.hpp>
#include <gazebo_ros/node.hpp>
#include <ignition/math/Pose3.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <memory>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>

#define VehicleLength 3.5932
#define VehicleWidth 1.966
#define WheelRadius 0.497
#define P 11.7553507260245
#define I 0.473007565420235
#define D 788.228671066606

namespace gazebo_plugins
{
class DrivePlugin : public gazebo::ModelPlugin
{
public:
DrivePlugin() : gazebo::ModelPlugin() {}


void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
{
    this->model_ = model;
    this->ros_node_ = gazebo_ros::Node::Get(sdf);

    RCLCPP_INFO(ros_node_->get_logger(), "DrivePlugin: Model name: %s", model->GetName().c_str());

    this->InitJoints(sdf);

    this->subscriber_ = this->ros_node_->create_subscription<geometry_msgs::msg::Twist>(
        "/wheel_torque_command", 10,
        std::bind(&DrivePlugin::OnTorqueCommand, this, std::placeholders::_1));

    RCLCPP_INFO(ros_node_->get_logger(), "DrivePlugin: Subscribed to /wheel_torque_command");

    this->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&DrivePlugin::OnUpdate, this));
}



private:
void InitJoints(sdf::ElementPtr sdf)
{
this->left_front_wheel_joint_ = model_->GetJoint("Revolute 16");
this->right_front_wheel_joint_ = model_->GetJoint("Revolute 17");
this->left_back_wheel_joint_ = model_->GetJoint("Revolute 5");
this->right_back_wheel_joint_ = model_->GetJoint("Revolute 6");


    if (!this->left_back_wheel_joint_ || !this->right_back_wheel_joint_)
    {
        RCLCPP_ERROR(ros_node_->get_logger(), "DrivePlugin: Back Wheels joints not found");
        return;
    }

    if (!this->left_front_wheel_joint_ || !this->right_front_wheel_joint_)
    {
        RCLCPP_ERROR(ros_node_->get_logger(), "DrivePlugin: Front Wheels joints not found");
        return;
    }

    this->suspension_joints_ = {
        model_->GetJoint("Revolute 1"),
        model_->GetJoint("Revolute 2"),
        model_->GetJoint("Revolute 3"),
        model_->GetJoint("Revolute 4")};

    for (const auto &joint : suspension_joints_)
    {
        if (!joint)
        {
            RCLCPP_ERROR(ros_node_->get_logger(), "DrivePlugin: Suspension joint not found");
            return;
        }
    }

}

void OnUpdate()
{
    apply_steering(Steering_Request);
    apply_efforts_suspension();
    ApplyWheelSpeed(desired_speed_);
}

void ApplyWheelSpeed(double desired_speed)
{
    // Calculate the velocity of the left and right wheels
    double left_target_velocity = desired_speed - (Steering_Request * VehicleWidth / 2.0);
    double right_target_velocity = desired_speed + (Steering_Request * VehicleWidth / 2.0);

    // Get current velocities of the wheels
    double left_current_velocity = left_back_wheel_joint_->GetVelocity(0);
    double right_current_velocity = right_back_wheel_joint_->GetVelocity(0);

    // Compute the effort using PID controllers
    double left_effort = wheel_pid_.Update(left_target_velocity - left_current_velocity,std::chrono::duration<double>(0.001));
    double right_effort = wheel_pid_.Update(right_target_velocity - right_current_velocity,std::chrono::duration<double>(0.001));

    // Apply efforts to the wheel joints
    left_back_wheel_joint_->SetForce(0, left_effort);
    right_back_wheel_joint_->SetForce(0, right_effort);
}

void apply_efforts_suspension()
{
    for (auto &joint : suspension_joints_)
    {
        double position = joint->Position(0);
        double target_position = 0.0;
        suspension_pid_.Init(10.0, 0.01, 0.1, 0.0, 0.0, 100.0, -100.0); // Ejemplo de valores
        double force = suspension_pid_.Update(position - target_position,std::chrono::duration<double>(0.001));
        joint->SetForce(0, force);
    }
}

void apply_steering(double Steering_Request)
{
    double ThetaAckerman = 0.0;
    double ThetaOuter = 0.0;

    if (Steering_Request > 0)
    {
        ThetaAckerman = atan(1 / ((1 / tan(Steering_Request)) + (VehicleWidth / VehicleLength)));
        steer_controller(this->left_front_wheel_joint_, Steering_Request);
        steer_controller(this->right_front_wheel_joint_, ThetaAckerman);
    }
    else if (Steering_Request < 0)
    {
        ThetaAckerman = atan(1 / ((1 / tan(-Steering_Request)) + (VehicleWidth / VehicleLength)));
        steer_controller(this->left_front_wheel_joint_, -ThetaAckerman);
        steer_controller(this->right_front_wheel_joint_, Steering_Request);
    }
    else
    {
        steer_controller(this->left_front_wheel_joint_, 0);
        steer_controller(this->right_front_wheel_joint_, 0);
    }
}

void steer_controller(gazebo::physics::JointPtr steer_joint, double Angle)
{
    double currentWheelAngle = steer_joint->Position(0);
    double steeringOmega = steer_joint->GetVelocity(0);

    if (steer_joint == this->left_front_wheel_joint_)
    {
        DesiredAngle = DesiredAngle + steeringSpeed * deltaSimTime * (Angle - DesiredAngle);
        if (fabs(Angle - DesiredAngle) < 0.01) DesiredAngle = Angle;
        IerL += DesiredAngle - currentWheelAngle;
        double jointforce = P * (DesiredAngle - currentWheelAngle) + I * IerL - D * steeringOmega;
        steer_joint->SetForce(0, jointforce);
    }
    else
    {
        DesiredAngleR = DesiredAngleR + steeringSpeed * deltaSimTime * (Angle - DesiredAngleR);
        if (fabs(Angle - DesiredAngleR) < 0.01) DesiredAngleR = Angle;
        IerR += DesiredAngleR - currentWheelAngle;
        double jointforce = P * (DesiredAngleR - currentWheelAngle) + I * IerR - D * steeringOmega;
        steer_joint->SetForce(0, jointforce);
    }
}

void OnTorqueCommand(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    RCLCPP_INFO(ros_node_->get_logger(), "DrivePlugin: Received torque command: linear.x=%f, angular.z=%f",
                msg->linear.x, msg->angular.z);

    desired_speed_ = msg->linear.x;
    Steering_Request = msg->angular.z;
}

gazebo::physics::ModelPtr model_;
gazebo::physics::JointPtr left_front_wheel_joint_;
gazebo::physics::JointPtr right_front_wheel_joint_;
gazebo::physics::JointPtr left_back_wheel_joint_;
gazebo::physics::JointPtr right_back_wheel_joint_;
std::vector<gazebo::physics::JointPtr> suspension_joints_;
std::shared_ptr<gazebo_ros::Node> ros_node_;
gazebo::event::ConnectionPtr update_connection_;
rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;

ignition::math::PID wheel_pid_;
ignition::math::PID suspension_pid_;
double Steering_Request = 0.0;
double DesiredAngle = 0.0;
double DesiredAngleR = 0.0;
double IerL = 0.0;
double IerR = 0.0;
double steeringSpeed = 1.0;
double deltaSimTime = 0.001;
double left_velocity_ = 0.0;
double right_velocity_ = 0.0;
double desired_speed_ = 0.0; // Initial desired speed



};

GZ_REGISTER_MODEL_PLUGIN(DrivePlugin)
} // namespace gazebo_plugins