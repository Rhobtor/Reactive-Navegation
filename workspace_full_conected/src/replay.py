#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelState
from std_srvs.srv import Empty
import time
import json
import os
from torch.utils.tensorboard import SummaryWriter
import logging

# Parámetros de tiempo y archivo de replay
REPLAY_FILE = "replay_data.json"
TIME_STEP = 0.2  # Tiempo entre nodos al reproducir la trayectoria

# Configuración básica del logger
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('ReplayAndMetrics')

class ReplayVisualizer(Node):
    def __init__(self):
        super().__init__('replay_visualizer')
        # Publicador para visualizar el nodo (o "goal") seleccionado
        self.goal_pub = self.create_publisher(ModelState, '/goal', 10)
        # Publicador para actualizar la posición del marcador en Gazebo (opcional)
        self.set_state = self.create_publisher(ModelState, '/gazebo/set_model_state', 10)
        # Cliente para pausar y reanudar la simulación (si se desea controlar)
        self.unpause_client = self.create_client(Empty, '/unpause_physics')
        self.pause_client = self.create_client(Empty, '/pause_physics')

    def publish_goal(self, goal_coord):
        """Publica un marcador en el nodo seleccionado para visualizarlo."""
        goal_state = ModelState()
        goal_state.model_name = "goal_marker"
        goal_state.pose.position.x = goal_coord['x']
        goal_state.pose.position.y = goal_coord['y']
        # Se puede ajustar la orientación si se requiere
        self.goal_pub.publish(goal_state)
        # También se puede publicar en /gazebo/set_model_state para que quede "fijo" en la simulación
        self.set_state.publish(goal_state)
        self.get_logger().info(f"Publicando nodo: x={goal_coord['x']}, y={goal_coord['y']}")

    def unpause_simulation(self):
        if not self.unpause_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Servicio /unpause_physics no disponible")
            return False
        request = Empty.Request()
        future = self.unpause_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info("Simulación reanudada")
            return True
        else:
            self.get_logger().error("Error al reanudar la simulación")
            return False

    def pause_simulation(self):
        if not self.pause_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Servicio /pause_physics no disponible")
            return False
        request = Empty.Request()
        future = self.pause_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info("Simulación pausada")
            return True
        else:
            self.get_logger().error("Error al pausar la simulación")
            return False

def load_replay_data(filepath):
    """Carga el archivo JSON con los datos de replay."""
    if not os.path.exists(filepath):
        logger.error(f"Archivo de replay no encontrado: {filepath}")
        return None
    with open(filepath, 'r') as f:
        data = json.load(f)
    logger.info(f"Se cargaron {len(data)} episodios de replay.")
    return data

def main():
    rclpy.init()
    node = ReplayVisualizer()
    writer = SummaryWriter("runs/replay_experiment")
    replay_data = load_replay_data(REPLAY_FILE)
    if replay_data is None:
        return

    # Reproduce cada episodio guardado
    for episode in replay_data:
        logger.info(f"Iniciando reproducción del episodio {episode['episode']} con recompensa total {episode['total_reward']}")
        writer.add_scalar("Reward/Total", episode["total_reward"], episode["episode"])

        # Opcional: reanudar simulación
        node.unpause_simulation()
        # Se puede esperar un tiempo antes de iniciar el episodio
        time.sleep(1.0)

        # Reproducir cada paso del episodio
        for step in episode["steps"]:
            node.publish_goal(step["goal"])
            # Registrar métricas: por ejemplo, recompensa de cada paso
            writer.add_scalar("Reward/Step", step["reward"], step["timestamp"])
            # Se visualiza cada nodo por un tiempo
            time.sleep(TIME_STEP)

        # Pausar la simulación al finalizar el episodio (opcional)
        node.pause_simulation()
        # Registrar cantidad de pasos o cualquier otra métrica
        writer.add_scalar("Steps/Episode", len(episode["steps"]), episode["episode"])
        # Pequeña pausa entre episodios
        time.sleep(2.0)

    writer.close()
    node.get_logger().info("Reproducción finalizada. Revise TensorBoard para ver las métricas.")
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        logger.error(f"Error en el script de replay: {e}")
