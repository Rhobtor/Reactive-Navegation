#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import subprocess
import os
import signal

class SupervisorNode(Node):
    def __init__(self):
        super().__init__('supervisor_node')
        # Parámetro para definir la lista de comandos a ejecutar para lanzar el entorno.
        # Ejemplo de lista:
        # ["ros2 launch car gazebo_world1.launch.py",
        #  "ros2 launch car gazebo_world2.launch.py",
        #  "ros2 launch car gazebo_world3.launch.py"]
        self.declare_parameter('launch_cmds', [
            'ros2 launch car gazebo_simple_world.launch.py',
            'ros2 launch car gazebo_simple2.launch.py',
            'ros2 launch car gazebo_simple3.launch.py',
            'ros2 launch car gazebo_mountain.launch.py'
        ])
        self.launch_cmds = self.get_parameter('launch_cmds').value
        self.get_logger().info(f"Comandos de lanzamiento: {self.launch_cmds}")

        self.current_index = 0  # Para seleccionar el comando de forma secuencial

        self.env_process = None
        self.launch_environment()

        # Se suscribe al tópico '/reset_request' para recibir solicitudes de reinicio.
        self.create_subscription(Bool, '/reset_request', self.reset_request_callback, 10)
        
        # Publicador en el tópico "goal_reached"
        self.goal_reached_pub = self.create_publisher(Bool, 'goal_reached', 10)

    def kill_existing_gazebo(self):
        """Intenta matar procesos de Gazebo que estén corriendo (gzserver y gzclient)."""
        try:
            subprocess.call(["pkill", "-9", "gzserver"])
            subprocess.call(["pkill", "-9", "gzclient"])
            self.get_logger().info("Procesos existentes de Gazebo cerrados.")
        except Exception as e:
            self.get_logger().error(f"Error al cerrar procesos de Gazebo: {e}")

    def launch_environment(self):
        """Lanza el entorno completo mediante el comando especificado."""
        # Primero, se intentan cerrar procesos previos de Gazebo
        self.kill_existing_gazebo()
        # Seleccionamos el comando actual de la lista
        cmd_str = self.launch_cmds[self.current_index]
        self.launch_cmd = cmd_str.split()
        self.get_logger().info(f"Iniciando entorno con: {self.launch_cmd}")
        # Se usa preexec_fn=os.setsid para crear un grupo de procesos
        self.env_process = subprocess.Popen(self.launch_cmd, preexec_fn=os.setsid)
        self.get_logger().info(f"Entorno iniciado con PID: {self.env_process.pid}")

    def kill_environment(self):
        """Termina el entorno actual enviando una señal a todo el grupo de procesos."""
        if self.env_process is not None:
            self.get_logger().info("Terminando entorno actual...")
            os.killpg(os.getpgid(self.env_process.pid), signal.SIGTERM)
            self.env_process.wait()
            self.get_logger().info("Entorno terminado.")

    def reset_request_callback(self, msg: Bool):
        """Callback que se ejecuta al recibir una solicitud de reinicio."""
        if msg.data:
            self.get_logger().info("Solicitud de reinicio recibida.")
            # Publicamos en el tópico 'goal_reached' para notificar que se alcanzó la meta
            goal_msg = Bool()
            goal_msg.data = True
            self.goal_reached_pub.publish(goal_msg)
            self.get_logger().info("Publicado goal_reached = True")
            # Reiniciamos el entorno
            self.kill_environment()
            # Actualizamos el índice para seleccionar el siguiente comando de la lista
            self.current_index = (self.current_index + 1) % len(self.launch_cmds)
            self.launch_environment()

    def destroy_node(self):
        """Al destruir el nodo, asegurarse de terminar el entorno."""
        self.kill_environment()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
