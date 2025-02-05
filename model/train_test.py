import numpy as np
import logging
from gazeob_test import GazeboTest  # Importa la clase GazeboTest

# Configura el logger
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('TrainingScript')

class GazeboTraining:
    def __init__(self, launch_file, num_episodes=5, max_steps=10):
        """
        Clase para gestionar el entrenamiento en Gazebo usando la clase GazeboTest.

        :param launch_file: Archivo de lanzamiento de Gazebo.
        :param num_episodes: Número total de episodios de entrenamiento.
        :param max_steps: Número máximo de pasos por episodio.
        """
        self.env = GazeboTest(launch_file)
        self.num_episodes = num_episodes
        self.max_steps = max_steps

    def choose_action(self, state):
        """
        Define una política simple para seleccionar acciones.
        En este caso, las acciones se generan aleatoriamente.

        :param state: El estado actual del entorno.
        :return: Una acción en forma de [velocidad_lineal, velocidad_angular].
        """
        linear_velocity = np.random.uniform(0.0, 1.0)  # Velocidad lineal
        angular_velocity = np.random.uniform(-1.0, 1.0)  # Velocidad angular
        return [linear_velocity, angular_velocity]

    def train(self):
        """
        Ejecuta el bucle de entrenamiento en múltiples episodios.
        """
        for episode in range(self.num_episodes):
            logger.info(f"Iniciando episodio {episode + 1}/{self.num_episodes}")

            # Reinicia el entorno
            state = self.env.reset()

            total_reward = 0.0
            done = False

            for step in range(self.max_steps):
                # Selecciona una acción
                action = self.choose_action(state)

                # Ejecuta un paso en el entorno
                next_state, reward, done, target_reached = self.env.step(action)

                total_reward += reward
                state = next_state

                logger.info(f"Episodio {episode + 1}, Paso {step + 1}: Recompensa = {reward}")

                # Si se alcanza el objetivo o hay colisión, termina el episodio
                if done:
                    if target_reached:
                        logger.info("Objetivo alcanzado!")
                    else:
                        logger.info("Colisión detectada.")
                    break

            logger.info(f"Episodio {episode + 1} completado con recompensa total: {total_reward}")

if __name__ == "__main__":
    # Especifica el archivo de lanzamiento de Gazebo
    LAUNCH_FILE = "/home/rhobtor/reactive/Reactive-Navegation/workspace_full_conected/src/car_python/launch/gazebo_no_gui.launch.py"

    # Crea y ejecuta la clase de entrenamiento
    trainer = GazeboTraining(LAUNCH_FILE)
    trainer.train()