#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray, Pose
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState
from squaternion import Quaternion
from std_srvs.srv import Empty
import numpy as np
import tensorflow as tf
import tensorflow_probability as tfp
from tensorflow.keras import layers, optimizers
import math
import time

# Parámetros de recompensa y control
GOAL_REACHED_DIST = 0.3
OBSTACLE_PENALTY_DIST = 0.5
GOAL_REWARD = 20.0
OBSTACLE_PENALTY = -15.0
STEP_PENALTY = -0.05
MIN_LINEAR_VELOCITY = 0.05  # Velocidad mínima para moverse
MIN_ANGULAR_VELOCITY = 0.1  # Evita valores muy pequeños

class PPOModel(tf.keras.Model):
    def __init__(self, state_dim, action_dim):
        super(PPOModel, self).__init__()
        self.fc1 = layers.Dense(128, activation='relu')
        self.fc2 = layers.Dense(128, activation='relu')
        self.mu = layers.Dense(action_dim, activation='tanh')
        self.log_sigma = tf.Variable(initial_value=-0.5 * np.ones(action_dim, dtype=np.float32), dtype=tf.float32, trainable=True)
        self.value = layers.Dense(1)

    def call(self, inputs):
        x = self.fc1(inputs)
        x = self.fc2(x)
        mu = self.mu(x)
        v = self.value(x)
        return mu, v

class NavigationPPOAgent(Node):
    def __init__(self):
        super().__init__('navigation_ppo_agent')

        # Subscripciones
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseArray, '/goal', self.goal_callback, 10)
        self.create_subscription(PoseArray, '/filtered_navigation_nodes', self.filtered_nodes_callback, 10)
        self.create_subscription(PoseArray, '/occupied_rejected_nodes', self.occupied_nodes_callback, 10)
        
        # Publicadores
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(ModelState, '/gazebo/set_model_state', 10)

        # PPO Model
        self.state_dim = 8
        self.action_dim = 2
        self.model = PPOModel(self.state_dim, self.action_dim)
        self.optimizer = optimizers.Adam(3e-4)

        # Variables de estado
        self.odom = None
        self.goal = None
        self.filtered_nodes = None
        self.occupied_nodes = None

        # PPO parameters
        self.gamma = 0.99
        self.clip_ratio = 0.2
        self.buffer = []

    def odom_callback(self, msg):
        self.odom = msg.pose.pose

    def goal_callback(self, msg):
        if msg.poses:
            self.goal = msg.poses[0]

    def filtered_nodes_callback(self, msg):
        self.filtered_nodes = msg

    def occupied_nodes_callback(self, msg):
        self.occupied_nodes = msg

    def get_state(self):
        if self.odom is None or self.goal is None:
            return np.zeros(self.state_dim)
        odom_x, odom_y = self.odom.position.x, self.odom.position.y
        goal_x, goal_y = self.goal.position.x, self.goal.position.y
        dist_goal = math.hypot(goal_x - odom_x, goal_y - odom_y)
        angle_goal = math.atan2(goal_y - odom_y, goal_x - odom_x)

        min_occ = 10.0
        if self.occupied_nodes and self.occupied_nodes.poses:
            min_occ = min(math.hypot(odom_x - p.position.x, odom_y - p.position.y) 
                          for p in self.occupied_nodes.poses)

        num_filtered = len(self.filtered_nodes.poses) if self.filtered_nodes else 0

        return np.array([odom_x, odom_y, goal_x, goal_y, dist_goal, angle_goal, min_occ, num_filtered])

    def reset_env(self):
        # Reset Gazebo
        reset_client = self.create_client(Empty, '/reset_simulation')
        reset_client.wait_for_service()
        reset_future = reset_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, reset_future)

        # Reset Octomap
        octomap_reset_client = self.create_client(Empty, '/octomap_server/reset')
        octomap_reset_client.wait_for_service()
        octomap_future = octomap_reset_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, octomap_future)

        time.sleep(2)  # Espera para estabilizar

        angle = np.random.uniform(-np.pi, np.pi)
        q = Quaternion.from_euler(0, 0, angle)
        state = ModelState(model_name='car', pose=Pose())
        state.pose.position.x = float(np.random.uniform(-4.5, 4.5))
        state.pose.position.y = float(np.random.uniform(-4.5, 4.5))
        state.pose.orientation.x = float(q.x)
        state.pose.orientation.y = float(q.y)
        state.pose.orientation.z = float(q.z)
        state.pose.orientation.w = float(q.w)
        self.state_pub.publish(state)

        time.sleep(2)

        return self.get_state()

    def select_action(self, state):
        state_tensor = tf.convert_to_tensor([state], dtype=tf.float32)
        mu, value = self.model(state_tensor)
        sigma = tf.exp(self.model.log_sigma)

        # Asegura que mu y sigma sean float32 explícitamente
        mu = tf.cast(mu, tf.float32)
        sigma = tf.cast(sigma, tf.float32)

        dist = tfp.distributions.Normal(mu, sigma)
        action = dist.sample()
        log_prob = tf.reduce_sum(dist.log_prob(action), axis=1)

        return action.numpy()[0].astype(np.float32), log_prob.numpy()[0].astype(np.float32), value.numpy()[0, 0].astype(np.float32)

    def step(self):
        state = self.get_state()
        action, log_prob, value = self.select_action(state)

        # Aplica un umbral mínimo de velocidad
        action[0] = max(MIN_LINEAR_VELOCITY, action[0])  
        action[1] = max(MIN_ANGULAR_VELOCITY, action[1])

        cmd = Twist()
        cmd.linear.x, cmd.angular.z = float(action[0]), float(action[1])
        self.cmd_pub.publish(cmd)

        self.get_logger().info(f"Acción tomada: velocidad lineal {cmd.linear.x:.3f}, angular {cmd.angular.z:.3f}")

        time.sleep(0.1)
        rclpy.spin_once(self, timeout_sec=0.1)

        next_state = self.get_state()
        reward = STEP_PENALTY
        done = False

        if next_state[4] < GOAL_REACHED_DIST:
            reward += GOAL_REWARD
            done = True
            self.get_logger().info("¡Goal alcanzado!")

        self.buffer.append((state, action, log_prob, reward, done, value))
        return done

    def update(self):
        # Implementación de actualización PPO
        pass

def main(args=None):
    rclpy.init(args=args)
    agent = NavigationPPOAgent()
    EPISODE_LENGTH = 200
    episodes = 50

    for episode in range(episodes):
        done = False
        step_count = 0
        state = agent.reset_env()
        
        while not done and step_count < EPISODE_LENGTH:
            done = agent.step()
            step_count += 1
        
        agent.update()
        agent.get_logger().info(f'Episodio {episode+1} finalizado después de {step_count} pasos.')

    agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
