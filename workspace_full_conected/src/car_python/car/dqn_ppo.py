#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import numpy as np
import tensorflow as tf
import datetime
import time
import random
from collections import deque

from std_srvs.srv import Empty
from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import PoseArray, Pose, Quaternion
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from octomap_msgs.msg import Octomap
from gazebo_msgs.msg import ModelState

# --------------------- UTILIDADES ---------------------
def quaternion_to_yaw(q: Quaternion):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

# --------------------- CONSTANTES ---------------------
GOAL_REACHED_DIST = 2.5       
STEP_PENALTY = -0.05          
GOAL_REWARD = 50.0            
OBSTACLE_PENALTY_DIST = 2.0   
MAX_CANDIDATES = 50            
FEATURE_DIM = 6               
GLOBAL_STATE_DIM = 7          
GAMMA = 0.99
LAMBDA = 0.95
CLIP_EPS = 0.2
TRAIN_EPOCHS = 10
BATCH_SIZE = 32
EXPLORATION_DISTANCE_THRESHOLD = 3.0  
EXPLORATION_BONUS_FACTOR = 70.0         
CLEARANCE_BONUS_FACTOR = 50.0           

# --------------------- COMPONENTE PPO ---------------------
class RecurrentActorNetwork(tf.keras.Model):
    def __init__(self, max_candidates, feature_dim, lstm_units=64, **kwargs):
        super(RecurrentActorNetwork, self).__init__(**kwargs)
        self.max_candidates = max_candidates
        self.feature_dim = feature_dim
        self.td1 = tf.keras.layers.TimeDistributed(tf.keras.layers.Dense(64, activation='relu'))
        self.lstm = tf.keras.layers.LSTM(lstm_units, return_sequences=True, return_state=True)
        self.logits_layer = tf.keras.layers.TimeDistributed(tf.keras.layers.Dense(1))
        
    def call(self, x, mask=None, initial_state=None):
        x = self.td1(x)
        lstm_out, h, c = self.lstm(x, initial_state=initial_state)
        logits = self.logits_layer(lstm_out)
        logits = tf.squeeze(logits, axis=-1)
        if mask is not None:
            logits = tf.where(mask, logits, -1e9 * tf.ones_like(logits))
        return logits, (h, c)
    
    def get_config(self):
        config = super(RecurrentActorNetwork, self).get_config()
        config.update({
            "max_candidates": self.max_candidates,
            "feature_dim": self.feature_dim
        })
        return config

class CriticNetwork(tf.keras.Model):
    def __init__(self, input_dim):
        super(CriticNetwork, self).__init__()
        self.dense1 = tf.keras.layers.Dense(64, activation='relu')
        self.dense2 = tf.keras.layers.Dense(64, activation='relu')
        self.dense3 = tf.keras.layers.Dense(64, activation='relu')
        self.value = tf.keras.layers.Dense(1)
    def call(self, x):
        x = self.dense1(x)
        x = self.dense2(x)
        x = self.dense3(x)
        v = self.value(x)
        return tf.squeeze(v, axis=-1)

# --------------------- COMPONENTE DQN ---------------------
class DQN(tf.keras.Model):
    def __init__(self, state_size, action_size):
        super(DQN, self).__init__()
        self.fc1 = tf.keras.layers.Dense(64, activation='relu')
        self.fc2 = tf.keras.layers.Dense(64, activation='relu')
        self.fc3 = tf.keras.layers.Dense(action_size)

    def call(self, x):
        x = self.fc1(x)
        x = self.fc2(x)
        return self.fc3(x)

class DQNAgent:
    def __init__(self, state_size, action_size):
        self.state_size = state_size
        self.action_size = action_size
        self.memory = deque(maxlen=10000)
        self.gamma = 0.99
        self.epsilon = 1.0
        self.epsilon_decay = 0.995
        self.epsilon_min = 0.01
        self.learning_rate = 0.001
        self.batch_size = 32
        self.model = DQN(state_size, action_size)
        self.optimizer = tf.keras.optimizers.Adam(self.learning_rate)
        self.loss_fn = tf.keras.losses.MeanSquaredError()

    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))

    def act(self, state):
        if np.random.rand() < self.epsilon:
            return random.randrange(self.action_size)
        q_values = self.model(np.array([state], dtype=np.float32))
        return int(np.argmax(q_values.numpy()[0]))

    def replay(self):
        if len(self.memory) < self.batch_size:
            return
        minibatch = random.sample(self.memory, self.batch_size)
        states, actions, rewards, next_states, dones = zip(*minibatch)
        states = np.array(states, dtype=np.float32)
        next_states = np.array(next_states, dtype=np.float32)

        with tf.GradientTape() as tape:
            q_values = self.model(states)
            q_values_taken = tf.reduce_sum(q_values * tf.one_hot(actions, self.action_size), axis=1)
            next_q_values = self.model(next_states)
            targets = np.array(rewards, dtype=np.float32) + self.gamma * np.max(next_q_values, axis=1) * (1 - np.array(dones, dtype=np.float32))
            loss = self.loss_fn(targets, q_values_taken)

        grads = tape.gradient(loss, self.model.trainable_variables)
        self.optimizer.apply_gradients(zip(grads, self.model.trainable_variables))

        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

# --------------------- NODO COMBINADO (PPO + DQN) ---------------------
# Definición del estado para DQN: [odom_x, odom_y, total_entropy] + (2 * MAX_CANDIDATES) coordenadas
STATE_SIZE_DQN = 3 + MAX_CANDIDATES * 2
ACTION_SIZE = MAX_CANDIDATES

class NavigationCombinedTrainer(Node):
    def __init__(self):
        super().__init__('navigation_combined_trainer')
        # Subscripciones
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseArray, '/goal', self.goal_callback, 10)
        self.create_subscription(PoseArray, '/filtered_navigation_nodes', self.filtered_nodes_callback, 10)
        self.create_subscription(PoseArray, '/occupied_rejected_nodes', self.occupied_nodes_callback, 10)
        self.create_subscription(Bool, '/virtual_collision', self.collision_callback, 10)
        self.create_subscription(Bool, '/map_reset', self.map_reset_callback, 10)
        self.create_subscription(Float64, '/total_entropy', self.entropy_callback, 10)
        # Publicadores
        self.reset_request_pub = self.create_publisher(Bool, '/reset_request', 10)
        self.marker_pub = self.create_publisher(Marker, '/selected_candidate_marker', 10)
        self.nav_point_pub = self.create_publisher(PoseArray, '/nav_point', 10)
        self.state_pub = self.create_publisher(ModelState, '/gazebo/set_model_state', 10)

        # Variables del entorno
        self.odom = None
        self.goal = None
        self.filtered_nodes = None
        self.occupied_nodes = None
        self.virtual_collision = False
        self.map_points = None
        self.total_entropy = 0.0

        # Variables PPO
        self.current_candidate = None
        self.last_candidate = None
        self.last_candidate_features = None
        self.last_action_index = None  # Se almacenará el índice elegido
        self.ppo_states = []
        self.ppo_actor_inputs = []
        self.ppo_actions = []
        self.ppo_log_probs = []
        self.ppo_rewards = []
        self.ppo_values = []
        self.ppo_dones = []
        self.ppo_steps = 0
        self.ppo_max_steps = 1000

        # Instanciar agente DQN
        self.dqn_agent = DQNAgent(STATE_SIZE_DQN, ACTION_SIZE)
        
        # Otras variables
        self.collision_counter = 0
        self.collision_threshold = 3
        self.escape_counter = 0
        self.escape_threshold = 5
        self.last_movement_time = None
        self.movement_threshold = 0.01
        self.inactivity_time_threshold = 10.0
        self.last_reset_time = 0.0
        self.reset_cooldown = 20.0
        self.same_candidate_count = 0
        self.same_candidate_threshold = 3
        self.same_candidate_distance_threshold = 0.1
        self.last_heading = None
        self.delta_heading = 0.0

        # TensorBoard y control de episodios para PPO
        self.episode_count = 0
        self.total_episodes = 400
        self.start_time = time.time()
        self.log_dir = "logs/combined/" + datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        self.summary_writer = tf.summary.create_file_writer(self.log_dir)
        self.get_logger().info(f"TensorBoard logs en: {self.log_dir}")

        # Redes y optimizadores PPO
        self.actor = RecurrentActorNetwork(MAX_CANDIDATES, FEATURE_DIM)
        self.actor_state = None
        self.critic = CriticNetwork(GLOBAL_STATE_DIM)
        self.actor_optimizer = tf.keras.optimizers.Adam(3e-4)
        self.critic_optimizer = tf.keras.optimizers.Adam(3e-4)
        
        # Esperar datos iniciales
        while self.odom is None or self.goal is None:
            self.get_logger().warn("Esperando /odom y /goal...")
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("Datos iniciales recibidos, iniciando selección de nodo.")
        self.timer = self.create_timer(0.1, self.step)

    # ----- Callbacks -----
    def odom_callback(self, msg: Odometry):
        current_time = self.get_clock().now().nanoseconds / 1e9
        original_pose = msg.pose.pose
        offset_x = 0.5
        adjusted_pose = Pose()
        adjusted_pose.position.x = original_pose.position.x - offset_x
        adjusted_pose.position.y = original_pose.position.y
        adjusted_pose.position.z = original_pose.position.z
        adjusted_pose.orientation = original_pose.orientation
        self.odom = adjusted_pose

        yaw = quaternion_to_yaw(original_pose.orientation)
        if self.last_heading is None:
            self.last_heading = yaw
            self.delta_heading = 0.0
        else:
            self.delta_heading = abs(yaw - self.last_heading)
            self.last_heading = yaw

        linear = msg.twist.twist.linear
        speed = math.hypot(linear.x, linear.y)
        if speed > self.movement_threshold:
            self.last_movement_time = current_time

    def goal_callback(self, msg: PoseArray):
        if msg.poses:
            self.goal = msg.poses[0]

    def filtered_nodes_callback(self, msg: PoseArray):
        self.filtered_nodes = msg

    def occupied_nodes_callback(self, msg: PoseArray):
        self.occupied_nodes = msg

    def collision_callback(self, msg: Bool):
        self.virtual_collision = msg.data

    def map_reset_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info("Se recibió señal para reiniciar el mapa.")
            self.map_points = None

    def entropy_callback(self, msg: Float64):
        self.total_entropy = msg.data

    # ----- Funciones comunes -----
    def request_environment_reset(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.last_reset_time >= self.reset_cooldown:
            reset_msg = Bool()
            reset_msg.data = True
            self.reset_request_pub.publish(reset_msg)
            self.get_logger().info("Solicitud de reinicio del entorno enviada.")
            self.last_reset_time = current_time
        else:
            self.get_logger().info("Cooldown de reinicio activo; no se solicita reset.")

    def reset_internal_state(self):
        self.actor_state = None
        self.collision_counter = 0
        self.escape_counter = 0
        self.same_candidate_count = 0
        self.last_movement_time = self.get_clock().now().nanoseconds / 1e9

    def compute_obstacle_stats(self):
        if self.occupied_nodes and self.occupied_nodes.poses:
            distances = [math.hypot(self.odom.position.x - occ.position.x,
                                    self.odom.position.y - occ.position.y)
                         for occ in self.occupied_nodes.poses]
            avg_dist = np.mean(distances)
            num_obs = len(distances)
        else:
            avg_dist = 10.0
            num_obs = 0
        num_obs_norm = num_obs / 10.0
        return avg_dist, num_obs_norm

    def compute_candidate_features(self):
        features = []
        valid_nodes = []
        if self.filtered_nodes is None or not self.filtered_nodes.poses:
            return None, None, None
        robot_x = self.odom.position.x
        robot_y = self.odom.position.y
        goal_x = self.goal.position.x
        goal_y = self.goal.position.y
        avg_obs_dist, num_obs_norm = self.compute_obstacle_stats()
        for node in self.filtered_nodes.poses:
            if self.occupied_nodes and self.occupied_nodes.poses:
                clearance = min([math.hypot(node.position.x - occ.position.x,
                                            node.position.y - occ.position.y)
                                 for occ in self.occupied_nodes.poses])
            else:
                clearance = 10.0
            if clearance < OBSTACLE_PENALTY_DIST:
                continue
            dx = node.position.x - robot_x
            dy = node.position.y - robot_y
            dist_robot = math.hypot(dx, dy)
            angle_to_node = math.atan2(dy, dx)
            angle_diff = abs(angle_to_node)  # Simplificación
            dist_to_goal = math.hypot(node.position.x - goal_x, node.position.y - goal_y)
            feature_vector = [dist_robot, angle_diff, clearance, dist_to_goal, avg_obs_dist, num_obs_norm]
            features.append(feature_vector)
            valid_nodes.append(node)
        if len(features) == 0:
            return None, None, None
        features, valid_nodes = zip(*sorted(zip(features, valid_nodes), key=lambda x: x[0][3]))
        features = list(features)
        valid_nodes = list(valid_nodes)
        if len(features) > MAX_CANDIDATES:
            features = features[:MAX_CANDIDATES]
            valid_nodes = valid_nodes[:MAX_CANDIDATES]
        num_valid = len(features)
        while len(features) < MAX_CANDIDATES:
            features.append([0.0]*FEATURE_DIM)
        features = np.array(features, dtype=np.float32)
        mask = np.array([True]*num_valid + [False]*(MAX_CANDIDATES - num_valid))
        return features, valid_nodes, mask

    def get_global_state(self):
        avg_obs_dist, num_obs_norm = self.compute_obstacle_stats()
        return np.array([
            self.odom.position.x,
            self.odom.position.y,
            self.goal.position.x,
            self.goal.position.y,
            avg_obs_dist,
            num_obs_norm,
            self.delta_heading
        ], dtype=np.float32)

    def get_dqn_state(self):
        # Estado para DQN: [odom_x, odom_y, total_entropy] + 2 coords por candidato
        state = [self.odom.position.x, self.odom.position.y, self.total_entropy]
        features, valid_nodes, _ = self.compute_candidate_features()
        if valid_nodes is None:
            state += [0.0] * (MAX_CANDIDATES * 2)
        else:
            for node in valid_nodes[:MAX_CANDIDATES]:
                state.extend([node.position.x, node.position.y])
            state += [0.0] * ((MAX_CANDIDATES - len(valid_nodes)) * 2)
        return np.array(state, dtype=np.float32)

    # def compute_reward(self):
    #     d_current = math.hypot(self.odom.position.x - self.goal.position.x,
    #                            self.odom.position.y - self.goal.position.y)
    #     reward = -d_current * 0.1 + STEP_PENALTY
    #     if d_current < GOAL_REACHED_DIST:
    #         reward += GOAL_REWARD
    #     if self.occupied_nodes and self.occupied_nodes.poses:
    #         for occ in self.occupied_nodes.poses:
    #             d = math.hypot(self.odom.position.x - occ.position.x,
    #                            self.odom.position.y - occ.position.y)
    #             if d < 0.05:
    #                 reward -= 1000
    #                 self.collision_counter += 1
    #                 break
    #             elif d < 0.3:
    #                 reward -= (0.3 - d) * 200
    #     if self.virtual_collision:
    #         reward -= 500.0
    #     features, valid_nodes, _ = self.compute_candidate_features()
    #     if features is not None:
    #         clearance = features[0][2]
    #         if clearance < 3.0:
    #             reward -= 100.0
    #     if self.last_candidate is not None:
    #         d_explore = math.hypot(self.odom.position.x - self.last_candidate.position.x,
    #                                self.odom.position.y - self.last_candidate.position.y)
    #         if d_explore > EXPLORATION_DISTANCE_THRESHOLD:
    #             bonus = EXPLORATION_BONUS_FACTOR * (d_explore - EXPLORATION_DISTANCE_THRESHOLD)
    #             reward += bonus
    #     return reward



    def compute_reward(self):
        # Distancia actual al goal
        d_current = math.hypot(self.odom.position.x - self.goal.position.x,
                            self.odom.position.y - self.goal.position.y)
        # Calcular Δd: mejora en la distancia (si no hay dato previo, se considera 0)
        if hasattr(self, 'prev_goal_distance'):
            delta_d = self.prev_goal_distance - d_current
        else:
            delta_d = 0.0
        self.prev_goal_distance = d_current

        # Obtener clearance del mejor candidato (usando la primera característica del candidato)
        features, valid_nodes, mask = self.compute_candidate_features()
        if features is not None:
            clearance = features[0][2]
        else:
            clearance = 10.0

        # Cambio de dirección (Δθ) obtenido en el callback de odometría (self.delta_heading)
        delta_theta = self.delta_heading

        # Parámetros para la fórmula (pueden ajustarse o declararse como parámetros del nodo)
        alpha = 10.0  # Peso para Δd
        beta = 50.0   # Peso para la penalización basada en clearance
        lam = 10.0    # Controla la rapidez de la penalización del clearance
        delta_param = 5.0  # Peso para Δθ

        # Penalización por clearance: cuando clearance es pequeño, la penalización es alta.
        clearance_penalty = beta * math.exp(-lam * clearance)

        # Fórmula compuesta para la recompensa
        reward = alpha * delta_d - clearance_penalty + delta_param * delta_theta

        # Agregar penalización fija por cada paso
        reward += STEP_PENALTY

        # Recompensa si se alcanza el goal
        if d_current < GOAL_REACHED_DIST:
            reward += GOAL_REWARD

        # Penalizaciones por colisión física
        if self.occupied_nodes and self.occupied_nodes.poses:
            for occ in self.occupied_nodes.poses:
                d = math.hypot(self.odom.position.x - occ.position.x,
                            self.odom.position.y - occ.position.y)
                if d < 0.05:
                    reward -= 1000
                    self.get_logger().warn("¡Colisión física detectada!")
                    self.collision_detected = True
                    break
                elif d < 0.3:
                    reward -= (0.3 - d) * 200

        # Penalización por colisión virtual
        if hasattr(self, 'virtual_collision') and self.virtual_collision:
            reward -= 500
            self.get_logger().warn("¡Colisión virtual detectada!")

        # Bonus de exploración: si el robot se mueve (velocidad > 0.05 m/s) y se aleja del último candidato
        if self.last_candidate is not None and hasattr(self, 'last_speed') and self.last_speed > 0.05:
            d_explore = math.hypot(self.odom.position.x - self.last_candidate.position.x,
                                self.odom.position.y - self.last_candidate.position.y)
            if d_explore > EXPLORATION_DISTANCE_THRESHOLD:
                bonus = EXPLORATION_BONUS_FACTOR * (d_explore - EXPLORATION_DISTANCE_THRESHOLD)
                reward += bonus
                self.get_logger().info(f"Bonus de exploración: {bonus:.2f}")

        return reward

    def publish_candidate_marker(self, candidate: Pose):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "candidate_marker"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = candidate
        marker.scale.x = marker.scale.y = marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.b = 1.0
        self.marker_pub.publish(marker)

    def publish_nav_point(self, candidate: Pose):
        nav_points = PoseArray()
        nav_points.header.stamp = self.get_clock().now().to_msg()
        nav_points.header.frame_id = "map"
        nav_points.poses.append(candidate)
        self.nav_point_pub.publish(nav_points)

    # ----- Función step (llamada periódicamente) -----
    def step(self):
        if self.odom is None or self.goal is None:
            return

        current_time = self.get_clock().now().nanoseconds / 1e9
        if self.last_movement_time is not None and current_time - self.last_movement_time >= self.inactivity_time_threshold:
            self.get_logger().warn("Robot inactivo. Solicitando reinicio...")
            self.request_environment_reset()
            self.reset_internal_state()
            return

        if self.collision_counter >= self.collision_threshold:
            if self.last_candidate is not None:
                self.get_logger().info("Colisión detectada, usando candidato previo.")
                self.current_candidate = self.last_candidate
            self.collision_counter = 0

        # Se elimina la condición de "if self.current_candidate is None:" para actualizar en cada paso
        candidate_features, valid_nodes, mask = self.compute_candidate_features()
        if candidate_features is None:
            self.get_logger().warn("No hay candidatos válidos.")
            self.escape_counter += 1
            if self.escape_counter >= self.escape_threshold:
                self.get_logger().warn("Escape persistente. Solicitando reinicio...")
                self.request_environment_reset()
                self.reset_internal_state()
                self.escape_counter = 0
            return
        else:
            self.escape_counter = 0

        # Acción según PPO
        actor_input = np.expand_dims(candidate_features, axis=0)
        mask_input = np.expand_dims(mask, axis=0)
        logits, self.actor_state = self.actor(actor_input, mask=tf.convert_to_tensor(mask_input), initial_state=self.actor_state)
        probs = tf.nn.softmax(logits, axis=-1).numpy()[0]
        action_index_ppo = int(np.argmax(probs))
        if not mask[action_index_ppo]:
            self.get_logger().warn("Candidato PPO inválido.")
            return
        self.last_action_index = action_index_ppo
        candidate_ppo = valid_nodes[action_index_ppo]

        # Actualizamos el candidato en cada iteración
        self.current_candidate = candidate_ppo
        self.last_candidate = candidate_ppo
        self.last_candidate_features = (candidate_features, mask, action_index_ppo, probs)

        self.publish_candidate_marker(self.current_candidate)
        self.publish_nav_point(self.current_candidate)

        global_state = self.get_global_state()
        value = self.critic(tf.convert_to_tensor([global_state], dtype=tf.float32)).numpy()[0]
        reward = self.compute_reward()

        d_goal = math.hypot(
            self.odom.position.x - self.goal.position.x,
            self.odom.position.y - self.goal.position.y)
        done = 1 if d_goal < GOAL_REACHED_DIST or (self.ppo_steps >= self.ppo_max_steps) else 0

        # Guardar experiencia para PPO
        if self.last_candidate_features is not None:
            stored_features, stored_mask, stored_action_index, stored_probs = self.last_candidate_features
            self.ppo_states.append(global_state)
            self.ppo_actor_inputs.append((stored_features, stored_mask))
            self.ppo_actions.append(stored_action_index)
            self.ppo_log_probs.append(np.log(stored_probs[stored_action_index] + 1e-8))
            self.ppo_values.append(value)
            self.ppo_rewards.append(reward)
            self.ppo_dones.append(done)

        # Guardar experiencia para DQN, usando self.last_action_index
        dqn_state = self.get_dqn_state()
        self.dqn_agent.remember(dqn_state, self.last_action_index, reward, self.get_dqn_state(), done)

        self.ppo_steps += 1

        if done or self.ppo_steps >= self.ppo_max_steps:
            self.get_logger().info(f"Episodio terminado en {self.ppo_steps} pasos.")
            self.update_ppo_model()
            for _ in range(5):  # Actualizar DQN 5 veces por episodio
                self.dqn_agent.replay()
            self.ppo_states = []
            self.ppo_actor_inputs = []
            self.ppo_actions = []
            self.ppo_log_probs = []
            self.ppo_values = []
            self.ppo_rewards = []
            self.ppo_dones = []
            self.ppo_steps = 0
            self.current_candidate = None
            self.episode_count += 1
            self.actor_state = None

            elapsed = time.time() - self.start_time
            avg_ep_time = elapsed / self.episode_count
            remaining = (self.total_episodes - self.episode_count) * avg_ep_time
            self.get_logger().info(f"Tiempo transcurrido: {elapsed:.1f}s, Tiempo estimado restante: {remaining:.1f}s")
            
            # --- Registro de progreso de episodios ---
            progress_bar_length = 1  # Longitud de la barra de progreso
            completed_units = int((self.episode_count / self.total_episodes) * progress_bar_length)
            progress_bar = "[" + "#" * completed_units + "-" * (progress_bar_length - completed_units) + "]"
            self.get_logger().info(f"Episodios: {self.episode_count}/{self.total_episodes} {progress_bar}")

            
            
            with self.summary_writer.as_default():
                tf.summary.scalar('ppo_episode_reward', np.sum(self.ppo_rewards), step=self.episode_count)
                tf.summary.scalar('ppo_episode_length', len(self.ppo_rewards), step=self.episode_count)
            if self.episode_count >= self.total_episodes:
                self.get_logger().info("Se completaron los episodios previstos. Guardando modelos y finalizando nodo.")
                self.save_all_models()
                rclpy.shutdown()


    def update_ppo_model(self):
        advantages = np.zeros_like(self.ppo_rewards, dtype=np.float32)
        last_adv = 0.0
        values = np.array(self.ppo_values, dtype=np.float32)
        for t in reversed(range(len(self.ppo_rewards))):
            next_value = values[t+1] if t+1 < len(values) else 0.0
            delta = self.ppo_rewards[t] + GAMMA * next_value * (1 - self.ppo_dones[t]) - values[t]
            advantages[t] = last_adv = delta + GAMMA * LAMBDA * (1 - self.ppo_dones[t]) * last_adv
        returns = advantages + values
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)
        N = len(self.ppo_states)
        actor_inputs, masks = [], []
        for cand_feat, m in self.ppo_actor_inputs:
            actor_inputs.append(cand_feat)
            masks.append(m)
        actor_inputs = np.array(actor_inputs, dtype=np.float32)
        masks = np.array(masks, dtype=bool)
        dataset = tf.data.Dataset.from_tensor_slices((
            np.array(self.ppo_states, dtype=np.float32),
            actor_inputs, masks,
            np.array(self.ppo_actions, dtype=np.int32),
            np.array(self.ppo_log_probs, dtype=np.float32),
            returns.astype(np.float32),
            advantages.astype(np.float32)
        ))
        dataset = dataset.shuffle(N).batch(BATCH_SIZE)
        total_actor_loss = 0.0
        total_critic_loss = 0.0
        total_loss_value = 0.0
        batch_count = 0
        for epoch in range(TRAIN_EPOCHS):
            for batch in dataset:
                batch_states, batch_actor_inputs, batch_masks, batch_actions, batch_old_log_probs, batch_returns, batch_advantages = batch
                with tf.GradientTape(persistent=True) as tape:
                    logits, _ = self.actor(batch_actor_inputs, mask=batch_masks, initial_state=None)
                    batch_actions = tf.cast(batch_actions, tf.int32)
                    indices = tf.stack([tf.range(tf.shape(logits)[0]), batch_actions], axis=1)
                    new_log_probs = tf.math.log(tf.nn.softmax(logits, axis=1) + 1e-8)
                    new_log_probs = tf.gather_nd(new_log_probs, indices)
                    ratio = tf.exp(new_log_probs - batch_old_log_probs)
                    clipped_ratio = tf.clip_by_value(ratio, 1 - CLIP_EPS, 1 + CLIP_EPS)
                    actor_loss = -tf.reduce_mean(tf.minimum(ratio * batch_advantages, clipped_ratio * batch_advantages))
                    values_pred = self.critic(batch_states)
                    critic_loss = tf.reduce_mean(tf.square(batch_returns - values_pred))
                    total_loss = actor_loss + 0.5 * critic_loss
                actor_grads = tape.gradient(total_loss, self.actor.trainable_variables)
                critic_grads = tape.gradient(total_loss, self.critic.trainable_variables)
                self.actor_optimizer.apply_gradients(zip(actor_grads, self.actor.trainable_variables))
                self.critic_optimizer.apply_gradients(zip(critic_grads, self.critic.trainable_variables))
                total_actor_loss += actor_loss.numpy()
                total_critic_loss += critic_loss.numpy()
                total_loss_value += total_loss.numpy()
                batch_count += 1
        avg_actor_loss = total_actor_loss / batch_count if batch_count > 0 else 0.0
        avg_critic_loss = total_critic_loss / batch_count if batch_count > 0 else 0.0
        avg_total_loss = total_loss_value / batch_count if batch_count > 0 else 0.0
        self.get_logger().info("Modelo PPO actualizado.")
        self.get_logger().info(f"Actor Loss: {avg_actor_loss:.4f} | Critic Loss: {avg_critic_loss:.4f} | Total Loss: {avg_total_loss:.4f}")

    def save_all_models(self):
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        actor_filename = f"actor_model_{timestamp}.keras"
        critic_filename = f"critic_model_{timestamp}.keras"
        dqn_filename = f"dqn_model_{timestamp}.keras"
        self.actor.save(actor_filename)
        self.critic.save(critic_filename)
        self.dqn_agent.model.save(dqn_filename)
        self.get_logger().info(f"Modelos guardados: {actor_filename}, {critic_filename} y {dqn_filename}")

def main(args=None):
    rclpy.init(args=args)
    node = NavigationCombinedTrainer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
