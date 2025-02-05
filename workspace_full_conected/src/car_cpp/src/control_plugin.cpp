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
          "/wheel_torque_command", 10,
          std::bind(&DrivePlugin::OnTorqueCommand, this, std::placeholders::_1));

      RCLCPP_INFO(ros_node_->get_logger(), "DrivePlugin: Subscribed to /wheel_torque_command");

      // Conexión a la actualización del mundo de Gazebo
      this->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
          std::bind(&DrivePlugin::OnUpdate, this));



      
    }

  private:
    // Se obtienen los joints definidos en el modelo
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
        RCLCPP_ERROR(ros_node_->get_logger(), "DrivePlugin: Back Wheels joints not found");
        return;
      }
      if (!this->left_front_wheel_joint_ || !this->right_front_wheel_joint_)
      {
        RCLCPP_ERROR(ros_node_->get_logger(), "DrivePlugin: Front Wheels joints not found");
        return;
      }
      if (!this->left_front_speed_wheel_joint_ || !this->right_front_speed_wheel_joint_)
      {
        RCLCPP_ERROR(ros_node_->get_logger(), "DrivePlugin: Front Speed Wheels joints not found");
        return;
      }
    }

    // Esta función se llama en cada ciclo de simulación
    void OnUpdate()
    {
      // Variables locales para los comandos
      double delta = Steering_Request;  // ángulo de dirección solicitado (en radianes)
      double v_center = desired_speed_;   // velocidad lineal deseada (m/s)

      double left_steer = 0.0;
      double right_steer = 0.0;
      double left_v = v_center;
      double right_v = v_center;

      // Umbral para considerar que no hay giro
      const double eps = 1e-3;

      if (std::fabs(delta) < eps)
      {
        // Sin giro: ambos ángulos son cero y las velocidades son iguales.
        left_steer = 0.0;
        right_steer = 0.0;
      }
      else
      {
        // Calcular el radio de giro (desde el centro del eje trasero)
        double abs_delta = std::fabs(delta);
        double R = VehicleLength / std::tan(abs_delta);

        // Calcular ángulos de las ruedas delanteras según la dirección:
        if (delta > 0)
        {
          // Giro a la izquierda: la rueda izquierda es la interna.
          left_steer = std::atan(VehicleLength / (R - (VehicleWidth / 2.0)));
          right_steer = std::atan(VehicleLength / (R + (VehicleWidth / 2.0)));

          // Ajuste de velocidades: la rueda interna recorre menos distancia
          double inner_speed = v_center * (R - (VehicleWidth / 2.0)) / R;
          double outer_speed = v_center * (R + (VehicleWidth / 2.0)) / R;
          left_v = inner_speed;
          right_v = outer_speed;
        }
        else
        {
          // Giro a la derecha: la rueda derecha es la interna.
          // Se calcula con el mismo valor absoluto de delta
          left_steer = std::atan(VehicleLength / (R + (VehicleWidth / 2.0)));
          right_steer = std::atan(VehicleLength / (R - (VehicleWidth / 2.0)));
          // Se asignan signos negativos para indicar giro a la derecha.
          left_steer = -left_steer;
          right_steer = -right_steer;

          // Ajuste de velocidades: la interna (derecha) va más despacio
          double inner_speed = v_center * (R - (VehicleWidth / 2.0)) / R;
          double outer_speed = v_center * (R + (VehicleWidth / 2.0)) / R;
          // Para giro a la derecha, la rueda derecha es la interna.
          left_v = outer_speed;
          right_v = inner_speed;
        }
      }

      // Convertir velocidad lineal de la rueda a velocidad angular (rad/s)
      double left_wheel_omega = left_v / WheelRadius;
      double right_wheel_omega = right_v / WheelRadius;

      // Aplicar el comando de dirección a los joints de dirección de las ruedas delanteras.
      // Se asume que left_front_wheel_joint_ y right_front_wheel_joint_ controlan el giro (posición)
      left_front_wheel_joint_->SetPosition(0, left_steer);
      right_front_wheel_joint_->SetPosition(0, right_steer);

      // Aplicar la velocidad a las ruedas motrices.
      // En el ejemplo original se invertía la velocidad de las ruedas derechas (debido a la orientación en el modelo)
      left_back_wheel_joint_->SetVelocity(0, left_wheel_omega);
      right_back_wheel_joint_->SetVelocity(0, -right_wheel_omega);
      left_front_speed_wheel_joint_->SetVelocity(0, left_wheel_omega);
      right_front_speed_wheel_joint_->SetVelocity(0, -right_wheel_omega);

      // Se pueden agregar controles PID o límites adicionales según la dinámica deseada
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
    gazebo::physics::JointPtr left_front_speed_wheel_joint_;
    gazebo::physics::JointPtr right_front_speed_wheel_joint_;
    std::shared_ptr<gazebo_ros::Node> ros_node_;
    gazebo::event::ConnectionPtr update_connection_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;

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


