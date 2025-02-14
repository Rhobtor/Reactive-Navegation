#include <gazebo/common/Plugin.hh>
#include <rclcpp/rclcpp.hpp>
#include <gazebo_ros/node.hpp>
#include <ignition/math/Pose3.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Joint.hh>
#include <memory>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>

// Definiciones geométricas y parámetros
#define VehicleLength 3.5932    // Distancia entre ejes (m)
#define VehicleWidth 1.966      // Ancho del vehículo (m)
#define WheelRadius 0.497       // Radio de la rueda (m)
// Los parámetros PID se mantienen (si los usaras para control de error)
#define P 11.7553507260245
#define I 0.473007565420235
#define D 64.9118618875423
#define N 788.228671066606

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

      // Suscripción al tópico que recibe la velocidad y ángulo de dirección
      this->subscriber_ = this->ros_node_->create_subscription<geometry_msgs::msg::Twist>(
          "/cmd_vel", 10,
          std::bind(&DrivePlugin::OnTorqueCommand, this, std::placeholders::_1));

      RCLCPP_INFO(ros_node_->get_logger(), "DrivePlugin: Subscribed to /cmd_vel");

      // (Adicional) Publicador para mostrar el tiempo simulado
      this->time_pub_ = this->ros_node_->create_publisher<std_msgs::msg::Float64>("/sim_time", 10);

      // Conexión a la actualización del mundo de Gazebo
      this->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
          std::bind(&DrivePlugin::OnUpdate, this));
    }

  private:
    // Se obtienen los joints definidos en el modelo
    void InitJoints(sdf::ElementPtr sdf)
    {
      this->left_front_wheel_joint_ = model_->GetJoint("front_left_wheel_joint");
      this->right_front_wheel_joint_ = model_->GetJoint("front_right_wheel_joint");
      this->left_back_wheel_joint_ = model_->GetJoint("back_left_wheel_joint");
      this->right_back_wheel_joint_ = model_->GetJoint("back_right_wheel_joint");

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
    }

    // Esta función se llama en cada ciclo de simulación
    void OnUpdate()
    {
      // --- Sincronización en la fuente: Obtener el tiempo simulado ---
      // Se obtiene el tiempo simulado del mundo de Gazebo
      gazebo::common::Time sim_time = this->model_->GetWorld()->SimTime();
      // Se convierte a un objeto rclcpp::Time (esto es opcional si solo quieres usarlo para depuración)
      rclcpp::Time ros_sim_time(sim_time.sec, static_cast<uint32_t>(sim_time.nsec));
      // Por ejemplo, se puede publicar el tiempo simulado en un tópico
      std_msgs::msg::Float64 time_msg;
      time_msg.data = static_cast<double>(sim_time.sec) + static_cast<double>(sim_time.nsec) / 1e9;
      this->time_pub_->publish(time_msg);
      // También se puede usar para depuración (nota: esto imprimirá en cada ciclo)
      // RCLCPP_INFO(ros_node_->get_logger(), "Sim Time: %ld.%ld", sim_time.sec, sim_time.nsec);
      // -----------------------------------------------------------------

      // Calcular la velocidad de cada lado para un robot diferencial:
      // Para comando lineal (v) y angular (omega), las velocidades de las ruedas se pueden calcular:
      //   v_left  = v - (L/2)*omega
      //   v_right = v + (L/2)*omega
      double L = 1.2;  // Separación entre ruedas (ajusta según tu robot)
      double v = desired_speed_;
      double omega = Steering_Request;
      double v_left  = v - (L / 2.0) * omega;
      double v_right = v + (L / 2.0) * omega;

      // Convertir velocidades lineales a velocidades angulares (rad/s)
      double wheel_radius = 0.3;
      double left_wheel_omega  = v_left / wheel_radius;
      double right_wheel_omega = v_right / wheel_radius;

      // Aplicar la velocidad a las ruedas motrices
      left_back_wheel_joint_->SetVelocity(0, left_wheel_omega);
      right_back_wheel_joint_->SetVelocity(0, -right_wheel_omega);
      left_front_wheel_joint_->SetVelocity(0, left_wheel_omega);
      right_front_wheel_joint_->SetVelocity(0, -right_wheel_omega);
    }

    // Callback para recibir el mensaje de velocidad y ángulo
    void OnTorqueCommand(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
      // Se puede visualizar:
      // RCLCPP_INFO(ros_node_->get_logger(), "Received command: v=%f, delta=%f", msg->linear.x, msg->angular.z);
      desired_speed_ = msg->linear.x;
      Steering_Request = msg->angular.z;
    }

    // Punteros y variables miembro
    gazebo::physics::ModelPtr model_;
    gazebo::physics::JointPtr left_front_wheel_joint_;
    gazebo::physics::JointPtr right_front_wheel_joint_;
    gazebo::physics::JointPtr left_back_wheel_joint_;
    gazebo::physics::JointPtr right_back_wheel_joint_;

    std::shared_ptr<gazebo_ros::Node> ros_node_;
    gazebo::event::ConnectionPtr update_connection_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;

    // Publicador para el tiempo simulado (opcional para sincronización/depuración)
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr time_pub_;

    // Variables para almacenar los comandos
    double Steering_Request = 0.0;
    double desired_speed_ = 0.0;
    
    // (Opcional) Parámetros PID para control adicional
    ignition::math::PID wheel_pid_;
    ignition::math::PID suspension_pid_;
    double DesiredAngle = 0.0;
    double DesiredAngleR = 0.0;
    double IerL = 0.0;
    double IerR = 0.0;
    double steeringSpeed = 1.0;
    double deltaSimTime = 0.001;
    double left_velocity_ = 0.0;
    double right_velocity_ = 0.0;
    double prevErrorL = 0.0;
    double prevErrorR = 0.0;
    double prev_error = 0.0;
    double integral_error = 0.0;
  };

  // Registro del plugin
  GZ_REGISTER_MODEL_PLUGIN(DrivePlugin)
} // namespace gazebo_plugins
