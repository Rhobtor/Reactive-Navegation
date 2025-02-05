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
          "/cmd_vel", 10,
          std::bind(&DrivePlugin::OnTorqueCommand, this, std::placeholders::_1));

      RCLCPP_INFO(ros_node_->get_logger(), "DrivePlugin: Subscribed to /cmd_vel");

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

            // Calcular la velocidad de cada lado para un robot diferencial:
      // Para comando lineal (v) y angular (delta), las velocidades de las ruedas se pueden calcular:
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

      // // Variables locales para los comandos
      // double delta = Steering_Request;  // ángulo de dirección solicitado (en radianes)
      // double v_center = desired_speed_;   // velocidad lineal deseada (m/s)

      // double left_steer = 0.0;
      // double right_steer = 0.0;
      // double left_v = v_center;
      // double right_v = v_center;

      // // Umbral para considerar que no hay giro
      // const double eps = 1e-3;

      // if (std::fabs(delta) < eps)
      // {
      //   // Sin giro: ambos ángulos son cero y las velocidades son iguales.
      //   left_steer = 0.0;
      //   right_steer = 0.0;
      // }
      // else
      // {
      //   // Calcular el radio de giro (desde el centro del eje trasero)
      //   double abs_delta = std::fabs(delta);
      //   double R = VehicleLength / std::tan(abs_delta);

      //   // Calcular ángulos de las ruedas delanteras según la dirección:
      //   if (delta > 0)
      //   {
      //     // Giro a la izquierda: la rueda izquierda es la interna.
      //     left_steer = std::atan(VehicleLength / (R - (VehicleWidth / 2.0)));
      //     right_steer = std::atan(VehicleLength / (R + (VehicleWidth / 2.0)));

      //     // Ajuste de velocidades: la rueda interna recorre menos distancia
      //     double inner_speed = v_center * (R - (VehicleWidth / 2.0)) / R;
      //     double outer_speed = v_center * (R + (VehicleWidth / 2.0)) / R;
      //     left_v = inner_speed;
      //     right_v = outer_speed;
      //   }
      //   else
      //   {
      //     // Giro a la derecha: la rueda derecha es la interna.
      //     // Se calcula con el mismo valor absoluto de delta
      //     left_steer = std::atan(VehicleLength / (R + (VehicleWidth / 2.0)));
      //     right_steer = std::atan(VehicleLength / (R - (VehicleWidth / 2.0)));
      //     // Se asignan signos negativos para indicar giro a la derecha.
      //     left_steer = -left_steer;
      //     right_steer = -right_steer;

      //     // Ajuste de velocidades: la interna (derecha) va más despacio
      //     double inner_speed = v_center * (R - (VehicleWidth / 2.0)) / R;
      //     double outer_speed = v_center * (R + (VehicleWidth / 2.0)) / R;
      //     // Para giro a la derecha, la rueda derecha es la interna.
      //     left_v = outer_speed;
      //     right_v = inner_speed;
      //   }
      // }

      // // Convertir velocidad lineal de la rueda a velocidad angular (rad/s)
      // double left_wheel_omega = left_v / WheelRadius;
      // double right_wheel_omega = right_v / WheelRadius;

      // // Aplicar el comando de dirección a los joints de dirección de las ruedas delanteras.
      // // Se asume que left_front_wheel_joint_ y right_front_wheel_joint_ controlan el giro (posición)
      // left_front_wheel_joint_->SetPosition(0, left_steer);
      // right_front_wheel_joint_->SetPosition(0, right_steer);

      // // Aplicar la velocidad a las ruedas motrices.
      // // En el ejemplo original se invertía la velocidad de las ruedas derechas (debido a la orientación en el modelo)
      // left_back_wheel_joint_->SetVelocity(0, left_wheel_omega);
      // right_back_wheel_joint_->SetVelocity(0, -right_wheel_omega);
      // left_front_speed_wheel_joint_->SetVelocity(0, left_wheel_omega);
      // right_front_speed_wheel_joint_->SetVelocity(0, -right_wheel_omega);

      // // Se pueden agregar controles PID o límites adicionales según la dinámica deseada
    
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

// #include <gazebo/common/Plugin.hh>
// #include <rclcpp/rclcpp.hpp>
// #include <gazebo_ros/node.hpp>
// #include <gazebo/physics/Model.hh>
// #include <gazebo/physics/World.hh>
// #include <gazebo/physics/Joint.hh>
// #include <geometry_msgs/msg/twist.hpp>
// #include <nav_msgs/msg/odometry.hpp>
// #include <geometry_msgs/msg/transform_stamped.hpp>
// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <cmath>

// namespace gazebo_plugins
// {
//   class DrivePlugin : public gazebo::ModelPlugin
//   {
//   public:
//     DrivePlugin() : ModelPlugin(), x_(0.0), y_(0.0), theta_(0.0), last_update_sec_(0.0),
//                     desired_speed_(0.0), Steering_Request(0.0)
//     {}

//     void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
//     {
//       this->model_ = model;
//       this->ros_node_ = gazebo_ros::Node::Get(sdf);
//       RCLCPP_INFO(ros_node_->get_logger(), "DrivePlugin: Model name: %s", model->GetName().c_str());

//       // Inicialización de joints (reemplaza los nombres por los que tengas en tu modelo)
//       this->InitJoints(sdf);

//       // Suscripción al tópico que recibe velocidad y ángulo de dirección
//       this->subscriber_ = this->ros_node_->create_subscription<geometry_msgs::msg::Twist>(
//           "/wheel_torque_command", 10,
//           std::bind(&DrivePlugin::OnTorqueCommand, this, std::placeholders::_1));
//       RCLCPP_INFO(ros_node_->get_logger(), "DrivePlugin: Subscribed to /wheel_torque_command");

//       // // Publicador de odometría y TF broadcaster
//       // this->odom_pub_ = this->ros_node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
//       // this->tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(ros_node_);

//       // Inicializa el tiempo de actualización (usando la simulación)
//       last_update_sec_ = model_->GetWorld()->SimTime().Double();

//       // Conexión a la actualización del mundo
//       this->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
//           std::bind(&DrivePlugin::OnUpdate, this));
//     }

//   private:
//     void InitJoints(sdf::ElementPtr sdf)
//     {
//       // Se obtienen los joints. Cambia los nombres según tu modelo.
//       this->left_front_wheel_joint_  = model_->GetJoint("front_left_wheel_joint");
//       this->right_front_wheel_joint_ = model_->GetJoint("front_right_wheel_joint");
//       this->left_back_wheel_joint_   = model_->GetJoint("back_left_wheel_joint");
//       this->right_back_wheel_joint_  = model_->GetJoint("back_right_wheel_joint");

//       if (!left_back_wheel_joint_ || !right_back_wheel_joint_)
//       {
//         RCLCPP_ERROR(ros_node_->get_logger(), "DrivePlugin: Back Wheels joints not found");
//         return;
//       }
//       if (!left_front_wheel_joint_ || !right_front_wheel_joint_)
//       {
//         RCLCPP_ERROR(ros_node_->get_logger(), "DrivePlugin: Front Wheels joints not found");
//         return;
//       }
//     }

//     // Callback para recibir comando de velocidad y ángulo (Twist)
//     void OnTorqueCommand(const geometry_msgs::msg::Twist::SharedPtr msg)
//     {
//       desired_speed_ = msg->linear.x;
//       Steering_Request = msg->angular.z;
//     }

//     // Función que se llama en cada ciclo de simulación
//     void OnUpdate()
//     {
//       // Obtener el tiempo actual de simulación como double (en segundos)
//       double current_time = model_->GetWorld()->SimTime().Double();
//       double dt = current_time - last_update_sec_;
//       if (dt <= 0.0)
//         return;

//       // Calcular la velocidad de cada lado para un robot diferencial:
//       // Para comando lineal (v) y angular (delta), las velocidades de las ruedas se pueden calcular:
//       //   v_left  = v - (L/2)*omega
//       //   v_right = v + (L/2)*omega
//       double L = 1.2;  // Separación entre ruedas (ajusta según tu robot)
//       double v = desired_speed_;
//       double omega = Steering_Request;
//       double v_left  = v - (L / 2.0) * omega;
//       double v_right = v + (L / 2.0) * omega;

//       // Convertir velocidades lineales a velocidades angulares (rad/s)
//       double wheel_radius = 0.3;
//       double left_wheel_omega  = v_left / wheel_radius;
//       double right_wheel_omega = v_right / wheel_radius;


//       // Aplicar la velocidad a las ruedas motrices
//       left_back_wheel_joint_->SetVelocity(0, left_wheel_omega);
//       right_back_wheel_joint_->SetVelocity(0, -right_wheel_omega);
//       left_front_wheel_joint_->SetVelocity(0, left_wheel_omega);
//       right_front_wheel_joint_->SetVelocity(0, -right_wheel_omega);

//       // Integrar la odometría (modelo diferencial simple)
//       // Calcula la velocidad media y la velocidad angular a partir de las velocidades de las ruedas.
//       double v_actual = (v_left + v_right) / 2.0;
//       double omega_actual = (v_right - v_left) / L;

//       x_     += v_actual * std::cos(theta_) * dt;
//       y_     += v_actual * std::sin(theta_) * dt;
//       theta_ += omega_actual * dt;

//       // // Publicar la odometría y la transformación TF
//       // PublishOdometry(current_time);

//       last_update_sec_ = current_time;
//     }

//     // void PublishOdometry(double sim_time_sec)
//     // {
//     //   // Convertir el tiempo de simulación a un mensaje ROS (usando rclcpp::Time)
//     //   double sim_time_sec = this->now().seconds();
      
//     //   nav_msgs::msg::Odometry odom_msg;
//     //   odom_msg.header.stamp = rcl_time.to_msg();
//     //   odom_msg.header.frame_id = "odom";
//     //   odom_msg.child_frame_id = "base_link";
//     //   odom_msg.pose.pose.position.x = x_;
//     //   odom_msg.pose.pose.position.y = y_;
//     //   odom_msg.pose.pose.position.z = 0.0;

//     //   double roll = 0.0, pitch = 0.0, yaw = theta_;
//     //     q.setRPY(roll, pitch, yaw);
//     //     odom_msg.pose.pose.orientation.x = q.x();
//     //     odom_msg.pose.pose.orientation.y = q.y();
//     //     odom_msg.pose.pose.orientation.z = q.z();
//     //     odom_msg.pose.pose.orientation.w = q.w();

//     //   // Publicar velocidades (calculadas a partir de la cinemática)
//     //   double L = 1.2;
//     //   double v_left  = desired_speed_ - (L / 2.0) * Steering_Request;
//     //   double v_right = desired_speed_ + (L / 2.0) * Steering_Request;
//     //   double v_actual = (v_left + v_right) / 2.0;
//     //   double omega_actual = (v_right - v_left) / L;
//     //   odom_msg.twist.twist.linear.x = v_actual;
//     //   odom_msg.twist.twist.angular.z = omega_actual;

//     //   odom_pub_->publish(odom_msg);

//     //   // Publicar la transformación TF
//     //   geometry_msgs::msg::TransformStamped tf_msg;
//     //   tf_msg.header.stamp = this->now();
//     //   tf_msg.header.frame_id = "odom";
//     //   tf_msg.child_frame_id = "base_link";
//     //   tf_msg.transform.translation.x = x_;
//     //   tf_msg.transform.translation.y = y_;
//     //   tf_msg.transform.translation.z = 0.0;
//     //   tf_msg.transform.rotation.x = q.x();
//     //   tf_msg.transform.rotation.y = q.y();
//     //   tf_msg.transform.rotation.z = q.z();
//     //   tf_msg.transform.rotation.w = q.w();
//     //   tf_broadcaster_->sendTransform(tf_msg);
//     // }

//     // Miembros del plugin:
//     gazebo::physics::ModelPtr model_;
//     std::shared_ptr<gazebo_ros::Node> ros_node_;
//     gazebo::event::ConnectionPtr update_connection_;

//     // Joints
//     gazebo::physics::JointPtr left_front_wheel_joint_;
//     gazebo::physics::JointPtr right_front_wheel_joint_;
//     gazebo::physics::JointPtr left_back_wheel_joint_;
//     gazebo::physics::JointPtr right_back_wheel_joint_;

//     // Suscripción al comando
//     rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;

//     // Publicador de odometría y TF broadcaster
//     rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
//     std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

//     // Variables para la integración de la odometría
//     double x_ = 0.0;
//     double y_ = 0.0;
//     double theta_ = 0.0;
//     double last_update_sec_ = 0.0;

//     // Variables para el comando recibido
//     double desired_speed_ = 0.0;
//     double Steering_Request = 0.0;
//   };

//   // Registro del plugin
//   GZ_REGISTER_MODEL_PLUGIN(DrivePlugin)
// } // namespace gazebo_plugins
