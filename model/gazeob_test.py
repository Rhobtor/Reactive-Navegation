import subprocess
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
import time
import logging

# Configura el logger
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('GazeboTest')



class GazeboTest:
    def __init__(self,launch_file):
        
        self.odom_x = 0
        self.odom_y = 0

        self.goal_x = 1
        self.goal_y = 0.0
        
        self.last_odom = None

        #Estatus del robot en cada accion
        self.set_self_state = ModelState()
        self.set_self_state.model_name = "car"
        self.set_self_state.pose.position.x = 0.0
        self.set_self_state.pose.position.y = 0.0
        self.set_self_state.pose.position.z = 0.0
        self.set_self_state.pose.orientation.x = 0.0
        self.set_self_state.pose.orientation.y = 0.0
        self.set_self_state.pose.orientation.z = 0.0
        self.set_self_state.pose.orientation.w = 1.0


        # Inicializa rclpy y el nodo
        rclpy.init()
        self.node = rclpy.create_node('gazebo_control')

        # Lanza Gazebo
        self.launch_gazebo(launch_file)

    def launch_gazebo(self, launch_file):
        port = 11345
        logger.info(f"Launching ROS 2 with port {port}...")
        subprocess.Popen(["ros2", "launch", launch_file, f"port:={port}"])

        # Espera para que Gazebo inicie antes de continuar
        logger.info("Gazebo launched!")
        time.sleep(5)  # Ajusta el tiempo según sea necesario
        self.unpause_simulation()

    def unpause_simulation(self):
        unpause_client = self.node.create_client(Empty, '/gazebo/unpause_physics')
        if not unpause_client.wait_for_service(timeout_sec=5.0):
            logger.error("Service /gazebo/unpause_physics not available")
            return False
        
        request = Empty.Request()
        future = unpause_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is not None:
            logger.info("Simulation unpaused successfully!")
            return True
        else:
            logger.error("Failed to unpause the simulation")
            return False