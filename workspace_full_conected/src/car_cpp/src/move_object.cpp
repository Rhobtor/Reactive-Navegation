#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Pose3.hh>
#include <cmath>

namespace gazebo
{
  class MovingObstaclePlugin : public ModelPlugin
  {
  public:
    // Método que se llama cuando el plugin se carga
    void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
    {
      // Guardamos el puntero al modelo
      this->model = _model;
      // Almacenamos la pose inicial del modelo para utilizarla como referencia
      this->initialPose = this->model->WorldPose();
      // Conectamos al evento de actualización del mundo, para mover el modelo en cada iteración
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&MovingObstaclePlugin::OnUpdate, this));
    }

    // Callback que se llama en cada iteración de la simulación
    void OnUpdate()
    {
      // Obtenemos el tiempo de simulación actual
      common::Time t = this->model->GetWorld()->SimTime();
      const double A = 5.0; // Amplitud de la oscilación
      const double w = 10.0; // Frecuencia angular
      // Calculamos una nueva posición en el eje X, por ejemplo, usando una función seno para generar oscilación
      double newX = this->initialPose.Pos().X() + A * std::sin(w*t.Double());

      // Construimos una nueva pose con la posición actualizada y las rotaciones sin cambios
      ignition::math::Pose3d newPose(
          newX,
          this->initialPose.Pos().Y(),
          this->initialPose.Pos().Z(),
          this->initialPose.Rot().Roll(),
          this->initialPose.Rot().Pitch(),
          this->initialPose.Rot().Yaw());

      // Actualizamos la pose del modelo en el mundo
      this->model->SetWorldPose(newPose);
    }

  private:
    // Puntero al modelo que controlamos
    physics::ModelPtr model;
    // Conexión para la actualización del mundo
    event::ConnectionPtr updateConnection;
    // Pose inicial del modelo
    ignition::math::Pose3d initialPose;
  };

  // Registro del plugin en Gazebo
  GZ_REGISTER_MODEL_PLUGIN(MovingObstaclePlugin)
}
