import subprocess
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist
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

        # Subscribirse a los topic de velocidad y posiscion
        # self.vel_sub = self.node.create_subscription(Twist, '/car/cmd_vel', self.vel_callback, 10)
        self.odom_sub = self.node.create_subscription(ModelState, '/imu/data', self.odom_callback, 10)

        # Subscribirse al topic del mapa octree
        # self.map_sub = self.node.create_subscription(OccupancyGrid, '/octomap_binary', self.map_callback, 10)

        # Publicar la velocidad
        self.vel_pub = self.node.create_publisher(Twist, '/wheel_torque_command', 10)


        # Lanza Gazebo
        self.launch_gazebo(launch_file)


    def odom_callback(self, msg):
        self.odom_x = msg.pose.position.x
        self.odom_y = msg.pose.position.y
        self.last_odom = msg
    
    def step(self, action):
        target = False

        # Publish the robot action
        vel_cmd = Twist()
        vel_cmd.linear.x = action[0]
        vel_cmd.angular.z = action[1]
        self.vel_pub.publish(vel_cmd)

        # No se que es la accion a la que se refiere, mirar el otro script
        self.publish_markers(action)

        # Unpause the simulation to propagate the state
        self.unpause_simulation()

        # propagate state for TIME_DELTA seconds
        time.sleep(TIME_DELTA)

        # Pause the simulation to read the new state
        self.pause_simulation()

        # read the state of the octree map
        self.map_data = self.read_map()

        # done, collision, min_laser = self.observe_collision(self.velodyne_data)
        # v_state = []
        # v_state[:] = self.velodyne_data[:]
        # laser_state = [v_state]

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
        
        # Calculate distance to the goal from the robot
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
        if distance < GOAL_REACHED_DIST:
            target = True
            done = True

        robot_state = [distance, theta, action[0], action[1]]
        state = np.append(laser_state, robot_state)
        reward = self.get_reward(target, collision, action, min_laser)
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