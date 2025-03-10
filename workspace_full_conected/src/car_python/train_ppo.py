#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PoseArray, Pose
from nav_msgs.msg import Odometry
import numpy as np
import tensorflow as tf
from tensorflow.keras import layers, optimizers
import math
import time

# Parámetros de recompensa (ajusta según tus necesidades)
GOAL_REACHED_DIST = 0.3
GOAL_REWARD = 10.0
# También podrías penalizar colisiones u otros comportamientos

# Modelo PPO: define la política y el valor
class PPOModel(tf.keras.Model):
    def __init__(self, state_dim, action_dim):
        super(PPOModel, self).__init__()
        self.fc1 = layers.Dense(64, activation='relu', input_shape=(state_dim,))
        self.fc2 = layers.Dense(64, activation='relu')
        self.mu = layers.Dense(action_dim, activation='tanh')  # Acción: velocidad lineal y angular
        # log_sigma es un parámetro entrenable (compartido para todas las acciones)
        self.log_sigma = tf.Variable(initial_value=-0.5 * np.ones(action_dim, dtype=np.float32), trainable=True)
        self.value = layers.Dense(1, activation=None)
    
    def call(self, inputs):
        x = self.fc1(inputs)
        x = self.fc2(x)
        mu = self.mu(x)
        v = self.value(x)
        return mu, v

# Agente PPO que interactúa con Gazebo a través de ROS2
class NavigationPPOAgent(Node):
    def __init__(self):
        super(NavigationPPOAgent, self).__init__('navigation_ppo_agent')
        # Definir el espacio de estados:
        # [odom_x, odom_y, goal_x, goal_y, dist_goal, angle_goal, min_occupied, num_filtered]
        self.state_dim = 8
        self.action_dim = 2  # [velocidad lineal, velocidad angular]
        
        # Subscripciones:
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, 'goal', self.goal_callback, 10)
        self.filtered_nodes_sub = self.create_subscription(PoseArray, 'filtered_navigation_nodes', self.filtered_nodes_callback, 10)
        self.occupied_nodes_sub = self.create_subscription(PoseArray, 'occupied_rejected_nodes', self.occupied_nodes_callback, 10)
        
        # Publicador para cmd_vel:
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Variables del estado:
        self.odom = None
        self.goal = None
        self.filtered_nodes = None
        self.occupied_nodes = None
        
        # Hiperparámetros PPO:
        self.gamma = 0.99
        self.clip_ratio = 0.2
        self.learning_rate = 3e-4
        
        # Crear el modelo PPO:
        self.model = PPOModel(self.state_dim, self.action_dim)
        self.optimizer = optimizers.Adam(learning_rate=self.learning_rate)
        
        # Buffer de experiencia:
        self.states = []
        self.actions = []
        self.log_probs = []
        self.rewards = []
        self.dones = []
        self.values = []
        
        self.get_logger().info("Navigation PPO Agent iniciado.")
    
    # Callbacks de suscripción:
    def odom_callback(self, msg):
        self.odom = msg.pose.pose
    
    def goal_callback(self, msg):
        self.goal = msg.pose
    
    def filtered_nodes_callback(self, msg):
        self.filtered_nodes = msg
    
    def occupied_nodes_callback(self, msg):
        self.occupied_nodes = msg

    def get_state(self):
        # Define el estado incorporando odometría, goal y datos de nodos
        if self.odom is None or self.goal is None:
            return np.zeros(self.state_dim, dtype=np.float32)
        odom_x = self.odom.position.x
        odom_y = self.odom.position.y
        goal_x = self.goal.position.x
        goal_y = self.goal.position.y
        dist_goal = math.hypot(goal_x - odom_x, goal_y - odom_y)
        angle_goal = math.atan2(goal_y - odom_y, goal_x - odom_x)
        # Clearance mínimo a nodos ocupados:
        min_occ = 10.0  # Valor alto si no hay datos
        if self.occupied_nodes is not None and len(self.occupied_nodes.poses) > 0:
            dists = [math.hypot(odom_x - p.position.x, odom_y - p.position.y) for p in self.occupied_nodes.poses]
            min_occ = min(dists)
        # Número de nodos filtrados:
        num_filtered = 0
        if self.filtered_nodes is not None:
            num_filtered = len(self.filtered_nodes.poses)
        state = np.array([odom_x, odom_y, goal_x, goal_y, dist_goal, angle_goal, min_occ, num_filtered], dtype=np.float32)
        return state

    def select_action(self, state):
        state_tensor = tf.convert_to_tensor([state], dtype=tf.float32)
        mu, value = self.model(state_tensor)
        sigma = tf.exp(self.model.log_sigma)
        # Crear una distribución normal:
        dist = tf.compat.v1.distributions.Normal(mu, sigma)
        action = dist.sample()
        log_prob = tf.reduce_sum(dist.log_prob(action), axis=1)
        return action[0].numpy(), log_prob[0].numpy(), value[0][0].numpy()
    
    def step(self):
        state = self.get_state()
        action, log_prob, value = self.select_action(state)
        
        # Publicar acción en cmd_vel:
        twist = Twist()
        twist.linear.x = float(action[0])
        twist.angular.z = float(action[1])
        self.cmd_vel_pub.publish(twist)
        
        # Esperar un breve tiempo (p. ej. 0.1 s)
        time.sleep(0.1)
        rclpy.spin_once(self, timeout_sec=0.01)
        
        next_state = self.get_state()
        # Ejemplo de función de recompensa: negativa de la distancia al goal
        reward = -next_state[4]
        done = next_state[4] < GOAL_REACHED_DIST
        
        return state, action, log_prob, reward, next_state, done, value
    
    def store_transition(self, state, action, log_prob, reward, done, value):
        self.states.append(state)
        self.actions.append(action)
        self.log_probs.append(log_prob)
        self.rewards.append(reward)
        self.dones.append(done)
        self.values.append(value)
    
    def compute_returns(self, next_value, gamma):
        returns = []
        R = next_value
        for reward, done in zip(reversed(self.rewards), reversed(self.dones)):
            R = reward + gamma * R * (1 - done)
            returns.insert(0, R)
        return returns
    
    def update(self):
        states = tf.convert_to_tensor(np.array(self.states), dtype=tf.float32)
        actions = tf.convert_to_tensor(np.array(self.actions), dtype=tf.float32)
        old_log_probs = tf.convert_to_tensor(np.array(self.log_probs), dtype=tf.float32)
        values = tf.convert_to_tensor(np.array(self.values), dtype=tf.float32)
        
        next_state = self.get_state()
        next_state_tensor = tf.convert_to_tensor([next_state], dtype=tf.float32)
        _, next_value = self.model(next_state_tensor)
        next_value = next_value[0][0].numpy()
        returns = np.array(self.compute_returns(next_value, self.gamma), dtype=np.float32)
        returns_tensor = tf.convert_to_tensor(returns, dtype=tf.float32)
        
        advantages = returns_tensor - values
        
        with tf.GradientTape() as tape:
            mu, value_preds = self.model(states)
            sigma = tf.exp(self.model.log_sigma)
            dist = tf.compat.v1.distributions.Normal(mu, sigma)
            new_log_probs = tf.reduce_sum(dist.log_prob(actions), axis=1)
            ratio = tf.exp(new_log_probs - old_log_probs)
            surrogate1 = ratio * advantages
            surrogate2 = tf.clip_by_value(ratio, 1 - self.clip_ratio, 1 + self.clip_ratio) * advantages
            policy_loss = -tf.reduce_mean(tf.minimum(surrogate1, surrogate2))
            value_loss = tf.reduce_mean(tf.square(returns_tensor - tf.squeeze(value_preds)))
            loss = policy_loss + 0.5 * value_loss
        gradients = tape.gradient(loss, self.model.trainable_variables)
        self.optimizer.apply_gradients(zip(gradients, self.model.trainable_variables))
        
        self.states = []
        self.actions = []
        self.log_probs = []
        self.rewards = []
        self.dones = []
        self.values = []
        
        self.get_logger().info(f"Actualización PPO, pérdida: {loss.numpy():.4f}")
    
def main(args=None):
    rclpy.init(args=args)
    agent = NavigationPPOAgent()
    total_steps = 0
    update_interval = 200  # Actualizar cada 200 pasos
    
    try:
        while rclpy.ok():
            state, action, log_prob, reward, next_state, done, value = agent.step()
            agent.store_transition(state, action, log_prob, reward, done, value)
            total_steps += 1
            if done:
                agent.get_logger().info("Goal alcanzado o episodio terminado.")
                # Aquí podrías reiniciar la posición en Gazebo, etc.
            if total_steps % update_interval == 0:
                agent.update()
    except KeyboardInterrupt:
        agent.get_logger().info("Entrenamiento interrumpido.")
    finally:
        agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
