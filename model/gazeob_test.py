import subprocess
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float64
from car_interfaces.msg import Graph
from std_srvs.srv import Empty
import time
import numpy as np
import logging
import math
from squaternion import Quaternion

GOAL_REACHED_DIST = 0.3
COLLISION_DIST = 0.35
TIME_DELTA = 0.1

# Configura el logger
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('GazeboTest')



class GazeboTest:
    def __init__(self,launch_file):
        

        # Goal local 
        self.goal_x = 0.0  
        self.goal_y = 0.0
        # Posicion del robot
        self.odom_x = 0.0
        self.odom_y = 0.0


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

        # Subscribirse a los topic de velocidad y posiscion
        # self.vel_sub = self.node.create_subscription(Twist, '/car/cmd_vel', self.vel_callback, 10)
        self.odom_sub = self.node.create_subscription(ModelState, '/odom', self.odom_callback, 10)

        # Subscribirse al topic del mapa octree
        # self.map_sub = self.node.create_subscription(OccupancyGrid, '/octomap_binary', self.map_callback, 10)

        # Publicar la velocidad
        self.vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)

        # Publicar la posicion del goal
        self.goal_pub = self.node.create_publisher(ModelState, '/goal', 10)

        # recibe el mapa 2d de los nodos
        self.map_2d = self.node.create_subscription(Graph, '/navigation_graph', self.map_callback, 10)

        # recibe la entropia toal del mapa
        self.total_entropy = self.node.create_subscription(Float64, '/total_entropy', self.total_entropy_callback, 10)

        # recibe la entropia de los ptos de interes
        self.ptos_entropy = self.node.create_subscription(Float64, '/frontier_entropie', self.ptos_entropy_callback, 10)

        #recibe la posicion de los puntos fronteras
        self.frontier = self.node.create_subscription(PoseArray, '/frontier_points', self.frontier_callback, 10)

        # Lanza Gazebo
        self.launch_gazebo(launch_file)


    def map_callback(self, msg):
        self.map_data = msg

    def total_entropy_callback(self, msg):
        self.total_entropy = msg.data
    
    def ptos_entropy_callback(self, msg):
        self.ptos_entropy = msg.data
    
    def frontier_callback(self, msg):
        self.frontier = msg

    # Por ahora solo 2D
    def odom_callback(self, msg):
       
        self.odom_x = msg.pose.position.x
        self.odom_y = msg.pose.position.y
        self.last_odom = msg
    

    # Comunicacion con robot-modelo_IA
    def step(self, action):
        target = False

        # Publish the robot goal


        self.goal = ModelState()
        self.goal.pose.position.x= action[0]
        self.goal.pose.position.y= action[1]

        self.goal_pub.publish(self.goal)

        #publicar la seleccion y hacerla visible en rviz
        self.publish_markers(action)

        # Unpause the simulation to propagate the state
        self.unpause_simulation()

        # propagate state for TIME_DELTA seconds
        time.sleep(TIME_DELTA)

        # Pause the simulation to read the new state
        self.pause_simulation()

        # lee el topic del mapa 2d de los nodos
        self.map_data = self.read_map()

        # Calculate robot heading from odometry data
        self.odom_x = self.last_odom.pose.pose.position.x
        self.odom_y = self.last_odom.pose.pose.position.y
        quaternion = Quaternion(
            self.last_odom.pose.pose.orientation.w,
            self.last_odom.pose.pose.orientation.x,
            self.last_odom.pose.pose.orientation.y,
            self.last_odom.pose.pose.orientation.z,
        )
        euler = quaternion.to_euler(degrees=False)
        angle = round(euler[2], 4)
##### Hay una parte del car_cpp que hace esta parte a la hora de seleccionar un nodo transitable
##### Se puede usar para la recompnesa usando un A* o Dijkstra, ESTUDIAR ESTO    (solo para cuando es frontorea euristica)
        
        # Calculate distance to goal from current position
        distance = np.linalg.norm(
            [self.odom_x - self.goal_x, self.odom_y - self.goal_y]
        )

        # Calculate the relative angle between the robots heading and heading toward the goal
        skew_x = self.goal_x - self.odom_x
        skew_y = self.goal_y - self.odom_y
        dot = skew_x * 1 + skew_y * 0
        mag1 = math.sqrt(math.pow(skew_x, 2) + math.pow(skew_y, 2))
        mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
        beta = math.acos(dot / (mag1 * mag2))
        if skew_y < 0:
            if skew_x < 0:
                beta = -beta
            else:
                beta = 0 - beta
        theta = beta - angle
        if theta > np.pi:
            theta = np.pi - theta
            theta = -np.pi - theta
        if theta < -np.pi:
            theta = -np.pi - theta
            theta = np.pi - theta


        # Detect if the goal has been reached and give a large positive reward
        



        # Si llega al objetivo positiva recompensa
        if distance < GOAL_REACHED_DIST:
            target = True 
            done = True

        robot_state = [distance, theta, action[0], action[1]]
        state = np.append(robot_state)
        reward = self.get_reward(target, action)
        return state, reward, done, target




    def launch_gazebo(self, launch_file):
        port = 11345
        logger.info(f"Launching ROS 2 with port {port}...")
        subprocess.Popen(["ros2", "launch", launch_file, f"port:={port}"])

        # Espera para que Gazebo inicie antes de continuar
        logger.info("Gazebo launched!")
        time.sleep(5)  # Ajusta el tiempo según sea necesario
        self.unpause_simulation()


 

    def unpause_simulation(self):
        unpause_client = self.node.create_client(Empty, '/unpause_physics')
        if not unpause_client.wait_for_service(timeout_sec=5.0):
            logger.error("Service /unpause_physics not available")
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
        
    def pause_simulation(self):
        unpause_client = self.node.create_client(Empty, '/pause_physics')
        if not unpause_client.wait_for_service(timeout_sec=5.0):
            logger.error("Service /pause_physics not available")
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


###Reset del mundo y variuables, por ahora solo utilizar 1 mundo, pero se puede cambiar para tener varios mundos
## el reset debe tener en cuenta el modelo DRL tambien, ver si resetar para hacer varios experimentos
    def reset(self):
        # Reset the state of the robot
        reset_client = self.node.create_client(Empty, '/reset_simulation')
        if not reset_client.wait_for_service(timeout_sec=5.0):
            logger.error("Service /reset_simulation not available")
            
        
        request = Empty.Request()
        future = reset_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is not None:
            logger.info("Simulation reset successfully!")
            
        else: 
            logger.error("Failed to reset the simulation")
            

        angle = np.random.uniform(-np.pi, np.pi)
        quaternion = Quaternion.from_euler(0.0, 0.0, angle)
        object_state = self.set_self_state

        x = 0
        y = 0
        position_ok = False
        while not position_ok:
            x = np.random.uniform(-4.5, 4.5)
            y = np.random.uniform(-4.5, 4.5)
            # position_ok = check_pos(x, y)
        object_state.pose.position.x = x
        object_state.pose.position.y = y
        # object_state.pose.position.z = 0.
        object_state.pose.orientation.x = quaternion.x
        object_state.pose.orientation.y = quaternion.y
        object_state.pose.orientation.z = quaternion.z
        object_state.pose.orientation.w = quaternion.w
        self.set_state.publish(object_state)

        self.odom_x = object_state.pose.position.x
        self.odom_y = object_state.pose.position.y

        # set a random goal in empty space in environment
        self.change_goal()
        # randomly scatter boxes in the environment

        self.unpause_simulation()

        time.sleep(TIME_DELTA)

        self.pause_simulation()

        distance = np.linalg.norm(
            [self.odom_x - self.goal_x, self.odom_y - self.goal_y]
        )

        skew_x = self.goal_x - self.odom_x
        skew_y = self.goal_y - self.odom_y

        dot = skew_x * 1 + skew_y * 0
        mag1 = math.sqrt(math.pow(skew_x, 2) + math.pow(skew_y, 2))
        mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
        beta = math.acos(dot / (mag1 * mag2))

        if skew_y < 0:
            if skew_x < 0:
                beta = -beta
            else:
                beta = 0 - beta
        theta = beta - angle

        if theta > np.pi:
            theta = np.pi - theta
            theta = -np.pi - theta
        if theta < -np.pi:
            theta = -np.pi - theta
            theta = np.pi - theta

        robot_state = [distance, theta, 0.0, 0.0]
        state = np.append(robot_state)
        return state

    def reward(self, target, action):
        reward = 0.0
        if target == "arrive":
            reward = 10.0
        if target == "colision":
            reward = -10.0
        if target == "close_obstacle":
            reward = -0.1
        if target == "close_path":
            reward = -10.0
        else:
            reward = -0.1
        return reward

    # si sale el mapa de nodos seria el costo de moverse de un nodo a otro
    # def cost_move(self, action):
        



    #     return cost

