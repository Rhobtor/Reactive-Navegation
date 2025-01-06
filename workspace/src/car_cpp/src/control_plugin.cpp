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


namespace gazebo_plugins

{
    class ControlPlugin : public gazebo::ModelPlugin
{
public:
    // Contructor
    ControlPlugin() : gazebo::ModelPlugin() {}

    //Se ejecuta cuando el plugin es cargado
    void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
    {
        //Almacen de modelo
        this->model_ = model;

        // Inicializa el nodo de Ros2
        this->ros_node_ = gazebo_ros::Node::Get(sdf);

        //Informacion de debug
        RCLCPP_INFO(ros_node_->get_logger(), "ControlPlugin: Model name: %s", model->GetName().c_str());

        // Obtiene el nombre del joint
        if (sdf->HasElement("joint_name"))
        {
            this->joint_name_ = sdf->Get<std::string>("joint_name");
        }
        else
        {
            RCLCPP_ERROR(ros_node_->get_logger(), "ControlPlugin: No joint_name provided");
            return;
        }

        // Encuentra la junta (joint) en el modelo
        this->joint_ = model->GetJoint(this->joint_name_);
        if (!this->joint_)
        {
            RCLCPP_ERROR(ros_node_->get_logger(), "ControlPlugin: Joint %s does not exist", this->joint_name_.c_str());
            return;
        }

        this->subscriber_ = this->ros_node_->create_subscription<geometry_msgs::msg::Twist>(
                "/wheel_torque_command", 10, std::bind(&ControlPlugin::OnTorqueCommand, this, std::placeholders::_1));
            
        RCLCPP_INFO(ros_node_->get_logger(), "ControlPlugin: Subscribed to /wheel_torque_command");

        // //Inicializa el temporizador    
        // this->timer_ = this->ros_node_->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ControlPlugin::OnUpdate, this));

    }

private:
    //Metodo de actualizacion periodica
    // void OnUpdate()
    // {
    //     //Obtiene la posicion actual de la junta
    //     double position = this->joint_->Position(0);

    //     //Obtiene la fuerza actual de la junta
    //     double force = this->joint_->GetForce(0);

    //     //Informacion de debug
    //     RCLCPP_INFO(ros_node_->get_logger(), "ControlPlugin: Joint %s position: %f, force: %f", this->joint_name_.c_str(), position, force);

    //     //Aplica una fuerza a la junta
    //     this->joint_->SetForce(0, 5.0);
    //     RCLCPP_INFO(ros_node_->get_logger(), "Aplicando fuerza a la junta [%s]", this->joint_name_.c_str());

    // }

    void OnTorqueCommand(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        //Informacion de debug
        RCLCPP_INFO(ros_node_->get_logger(), "ControlPlugin: Received torque command: linear.x=%f, angular.z=%f", msg->linear.x, msg->angular.z);

        //Aplica la fuerza a la junta
        this->joint_->SetForce(0, msg->linear.x);
        RCLCPP_INFO(ros_node_->get_logger(), "ControlPlugin: Applied force to joint [%s]", this->joint_name_.c_str());
    }

    gazebo::physics::ModelPtr model_;
    gazebo::physics::JointPtr joint_;
    std::string joint_name_;
    std::shared_ptr<gazebo_ros::Node> ros_node_;
    // rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
};

// Macro para registrar el plugin
GZ_REGISTER_MODEL_PLUGIN(ControlPlugin)
} // namespace gazebo_plugins