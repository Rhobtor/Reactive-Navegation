#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import numpy as np
import tensorflow as tf
import datetime
import time
import subprocess
import os
import signal

from std_srvs.srv import Empty
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from octomap_msgs.msg import Octomap

# PPO hiperparámetros y parámetros de navegación
GOAL_REACHED_DIST = 3.0       
STEP_PENALTY = -0.05          
GOAL_REWARD = 50.0            
OBSTACLE_PENALTY_DIST = 2.0   

MAX_CANDIDATES = 5            
# Características: [dist_robot, angle_diff, clearance, dist_to_goal, avg_obs_dist, num_obs_norm]
FEATURE_DIM = 6               
# Estado global para el crítico: se añade delta_heading (diferencia de orientación)
GLOBAL_STATE_DIM = 7          

GAMMA = 0.99
LAMBDA = 0.95
CLIP_EPS = 0.2
TRAIN_EPOCHS = 10
BATCH_SIZE = 32

# Parámetros de exploración y bonus
EXPLORATION_DISTANCE_THRESHOLD = 3.0  
EXPLORATION_BONUS_FACTOR = 40.0         
CLEARANCE_BONUS_FACTOR = 30.0           

# --- Función auxiliar: Conversión de cuaternión a yaw ---
def quaternion_to_yaw(q: Quaternion):
    # Fórmula estándar: yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

# --- Actor Recurrente con LSTM para Entrenamiento ---
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

    @classmethod
    def from_config(cls, config):
        config.pop('trainable', None)
        config.pop('dtype', None)
        return cls(**config)

# --- Critic Network ---
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

# --- Nodo de Entrenamiento PPO con Actor Recurrente ---
class NavigationPPOCandidateTrainer(Node):
    def __init__(self):
        super().__init__('navigation_ppo_candidate_trainer')
        # Subscripciones a odometría, goal, nodos filtrados, ocupados y colisión virtual
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseArray, '/goal', self.goal_callback, 10)
        self.create_subscription(PoseArray, '/filtered_navigation_nodes', self.filtered_nodes_callback, 10)
        self.create_subscription(PoseArray, '/occupied_rejected_nodes', self.occupied_nodes_callback, 10)
        self.create_subscription(Bool, '/virtual_collision', self.collision_callback, 10)
        # Subscripción para reinicio del mapa (por supervisor)
        self.create_subscription(Bool, '/map_reset', self.map_reset_callback, 10)
        # Publicador para solicitar reinicio del entorno
        self.reset_request_pub = self.create_publisher(Bool, '/reset_request', 10)
        # Publicadores para visualización
        self.marker_pub = self.create_publisher(Marker, '/selected_candidate_marker', 10)
        self.nav_point = self.create_publisher(PoseArray, '/nav_point', 10)
        
        # Variables de estado
        self.odom = None
        self.goal = None
        self.filtered_nodes = None
        self.occupied_nodes = None
        self.current_candidate = None
        self.last_candidate = None
        self.last_candidate_features = None
        
        # Buffer de experiencia
        self.states = []
        self.actor_inputs = []
        self.actions = []
        self.log_probs = []
        self.rewards = []
        self.values = []
        self.dones = []
        self.steps = 0
        self.max_steps = 1000

        # Contadores y flags
        self.collision_detected = False
        self.collision_counter = 0
        self.collision_threshold = 3
        self.escape_counter = 0
        self.escape_threshold = 5

        # Detección de inactividad (basada en velocidad)
        self.last_movement_time = None
        self.movement_threshold = 0.01  # m/s
        self.inactivity_time_threshold = 10.0  # segundos

        # Cooldown para reinicio del entorno
        self.last_reset_time = 0.0
        self.reset_cooldown = 20.0  # segundos

        # Detección de repetición del mismo candidato
        self.same_candidate_count = 0
        self.same_candidate_threshold = 3
        self.same_candidate_distance_threshold = 0.1  # m

        # Para mejorar la memoria: seguimiento de orientación
        self.last_heading = None
        self.delta_heading = 0.0

        # (Opcional) Suscribirse al mapa de puntos
        self.create_subscription(PoseArray, '/navigation_map_points', self.map_points_callback, 10)
        self.map_points = None

        # TensorBoard y tiempo de entrenamiento
        self.episode_count = 0
        self.total_episodes = 400
        self.start_time = time.time()
        self.log_dir = "logs/fit/" + datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        self.summary_writer = tf.summary.create_file_writer(self.log_dir)
        self.get_logger().info(f"TensorBoard logs en: {self.log_dir}")

        self.models_saved = False

        # Redes y optimizadores
        self.actor = RecurrentActorNetwork(MAX_CANDIDATES, FEATURE_DIM)
        self.actor_state = None
        self.critic = CriticNetwork(GLOBAL_STATE_DIM)
        self.actor_optimizer = tf.keras.optimizers.Adam(3e-4)
        self.critic_optimizer = tf.keras.optimizers.Adam(3e-4)
        self.get_logger().info("Navigation PPO Candidate Trainer iniciado.")
        while self.odom is None or self.goal is None:
            self.get_logger().warn("Esperando /odom y /goal...")
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("Datos iniciales recibidos, iniciando selección de nodo.")
        self.timer = self.create_timer(0.1, self.step)

    # --- Callbacks ---
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

        # Extraer yaw del robot
        yaw = quaternion_to_yaw(original_pose.orientation)
        if self.last_heading is None:
            self.last_heading = yaw
            self.delta_heading = 0.0
        else:
            self.delta_heading = abs(yaw - self.last_heading)
            self.last_heading = yaw

        # Actualizar last_movement_time basándose en la velocidad
        linear = msg.twist.twist.linear
        speed = math.hypot(linear.x, linear.y)
        self.last_speed = speed
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

    def map_points_callback(self, msg: PoseArray):
        self.map_points = msg.poses

    def map_reset_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info("Se recibió señal para reiniciar el mapa (variables locales).")
            self.map_points = None

    # --- Solicitar reinicio del entorno ---
    def request_environment_reset(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.last_reset_time >= self.reset_cooldown:
            reset_msg = Bool()
            reset_msg.data = True
            self.reset_request_pub.publish(reset_msg)
            self.get_logger().info("Solicitud de reinicio del entorno enviada al supervisor.")
            self.last_reset_time = current_time
        else:
            self.get_logger().info("Cooldown de reinicio activo; no se solicita reset.")

    # --- Reiniciar el estado interno ---
    def reset_internal_state(self):
        self.actor_state = None
        self.collision_counter = 0
        self.escape_counter = 0
        self.same_candidate_count = 0
        self.last_movement_time = self.get_clock().now().nanoseconds / 1e9

    # --- Estadísticas de obstáculos ---
    def compute_obstacle_stats(self):
        if self.occupied_nodes and self.occupied_nodes.poses:
            distances = [math.hypot(self.odom.position.x - obs.position.x,
                                    self.odom.position.y - obs.position.y)
                         for obs in self.occupied_nodes.poses]
            avg_dist = np.mean(distances)
            num_obs = len(distances)
        else:
            avg_dist = 10.0
            num_obs = 0
        num_obs_norm = num_obs / 10.0
        return avg_dist, num_obs_norm

    # --- Extraer características para cada candidato ---
    def compute_candidate_features(self):
        features = []
        valid_nodes = []
        if self.filtered_nodes is None or not self.filtered_nodes.poses:
            return None, None, None
        robot_x = self.odom.position.x
        robot_y = self.odom.position.y
        goal_x = self.goal.position.x
        goal_y = self.goal.position.y
        current_yaw = 0.0
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
            angle_diff = abs(angle_to_node - current_yaw)
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

    # --- Estado global para el crítico (se añade delta_heading) ---
    def get_global_state(self):
        avg_obs_dist, num_obs_norm = self.compute_obstacle_stats()
        return np.array([
            self.odom.position.x,
            self.odom.position.y,
            self.goal.position.x,
            self.goal.position.y,
            avg_obs_dist,
            num_obs_norm,
            self.delta_heading  # Diferencia en orientación
        ], dtype=np.float32)

    # --- Función de recompensa ---
    def compute_reward(self):
        # Distancia actual al goal
        d_current = math.hypot(self.odom.position.x - self.goal.position.x,
                               self.odom.position.y - self.goal.position.y)
        # Penaliza acercarse demasiado (se usa la distancia)
        reward = -d_current * 0.1 + STEP_PENALTY
        if d_current < GOAL_REACHED_DIST:
            reward += GOAL_REWARD

        # Penalización por colisión física
        if self.occupied_nodes and self.occupied_nodes.poses:
            for occ in self.occupied_nodes.poses:
                d = math.hypot(self.odom.position.x - occ.position.x,
                               self.odom.position.y - occ.position.y)
                if d < 0.05:
                    reward -= 1000
                    self.get_logger().warn("¡Colisión física detectada!")
                    self.collision_detected = True
                    self.collision_counter += 1
                    break
                elif d < 0.3:
                    reward -= (0.3 - d) * 200

        # Penalización por baja clearance (usando el clearance del primer candidato)
        features, valid_nodes, mask = self.compute_candidate_features()
        if features is not None:
            clearance = features[0][2]
            if clearance < 3.0:
                reward -= 100.0

        # Penalización por colisión virtual
        if hasattr(self, 'virtual_collision') and self.virtual_collision:
            reward -= 500.0
            self.get_logger().warn("¡Colisión virtual detectada!")
        
        # Bonus de exploración: si el robot se mueve (velocidad > 0.05 m/s)
        if self.last_candidate is not None and hasattr(self, 'last_speed') and self.last_speed > 0.05:
            d_explore = math.hypot(self.odom.position.x - self.last_candidate.position.x,
                                   self.odom.position.y - self.last_candidate.position.y)
            if d_explore > EXPLORATION_DISTANCE_THRESHOLD:
                bonus = EXPLORATION_BONUS_FACTOR * (d_explore - EXPLORATION_DISTANCE_THRESHOLD)
                reward += bonus
                self.get_logger().info(f"Bonus de exploración: {bonus:.2f}")
        
        # Bonus por mejora en clearance (si el nuevo candidato mejora la seguridad)
        if self.last_candidate_features is not None:
            last_features, _, last_action_index, _ = self.last_candidate_features
            clearance_last = last_features[last_action_index][2]
            new_features, _, _ = self.compute_candidate_features()
            if new_features is not None:
                clearance_new = new_features[0][2]
                if clearance_new > clearance_last:
                    bonus_clearance = CLEARANCE_BONUS_FACTOR * (clearance_new - clearance_last)
                    reward += bonus_clearance
                    self.get_logger().info(f"Bonus por clearance: {bonus_clearance:.2f}")
        return reward

    # --- Placeholder para planificación local (por ejemplo, A*) ---
    def plan_route(self, candidate, map_points):
        # Aquí se podría implementar un algoritmo de planificación local.
        return candidate

    # --- Función step ---
    def step(self):
        if self.odom is None or self.goal is None:
            return

        current_time = self.get_clock().now().nanoseconds / 1e9

        # Verificar inactividad: 10 segundos sin movimiento
        if self.last_movement_time is not None:
            if current_time - self.last_movement_time >= self.inactivity_time_threshold:
                self.get_logger().warn("Robot inactivo por 10 segundos. Solicitando reinicio del entorno...")
                self.request_environment_reset()
                self.reset_internal_state()
                return

        # Si se detecta colisión física, usar el nodo previo como backup
        if self.collision_detected:
            if self.last_candidate is not None:
                self.get_logger().info("Colisión física detectada, usando nodo previo como backup.")
                self.current_candidate = self.last_candidate
            else:
                self.get_logger().warn("Colisión física detectada pero no hay nodo previo para backup.")
            self.collision_detected = False

        # Si se alcanzó el candidato, reiniciar selección
        if self.current_candidate is not None:
            dist_to_candidate = math.hypot(
                self.odom.position.x - self.current_candidate.position.x,
                self.odom.position.y - self.current_candidate.position.y)
            if dist_to_candidate < 2.5:
                self.get_logger().info("Candidato alcanzado. Seleccionando nuevo candidato.")
                self.current_candidate = None

        # Seleccionar candidato si no hay uno vigente
        if self.current_candidate is None:
            candidate_features, valid_nodes, mask = self.compute_candidate_features()
            if candidate_features is None:
                self.get_logger().warn("No hay candidatos válidos.")
                self.escape_counter += 1
                if self.escape_counter >= self.escape_threshold:
                    self.get_logger().warn("Bloqueo persistente detectado. Forzando backup (nodo previo).")
                    if self.last_candidate is not None:
                        self.current_candidate = self.last_candidate
                    self.escape_counter = 0
                return
            else:
                self.escape_counter = 0
            actor_input = np.expand_dims(candidate_features, axis=0)
            mask_input = np.expand_dims(mask, axis=0)
            logits, self.actor_state = self.actor(actor_input, mask=tf.convert_to_tensor(mask_input), initial_state=self.actor_state)
            probs = tf.nn.softmax(logits, axis=-1).numpy()[0]
            action_index = int(np.argmax(probs))
            if not mask[action_index]:
                self.get_logger().warn("Candidato seleccionado inválido.")
                return
            new_candidate = valid_nodes[action_index]
            if self.map_points is not None:
                new_candidate = self.plan_route(new_candidate, self.map_points)
            if self.last_candidate is not None:
                d_same = math.hypot(new_candidate.position.x - self.last_candidate.position.x,
                                    new_candidate.position.y - self.last_candidate.position.y)
                if d_same < self.same_candidate_distance_threshold:
                    self.same_candidate_count += 1
                else:
                    self.same_candidate_count = 0
            else:
                self.same_candidate_count = 0
            if self.same_candidate_count >= self.same_candidate_threshold:
                self.get_logger().warn("Se ha seleccionado el mismo punto repetidamente. Solicitando reinicio del entorno...")
                self.request_environment_reset()
                self.reset_internal_state()
                self.same_candidate_count = 0
                return
            self.current_candidate = new_candidate
            self.get_logger().info(f"Nodo candidato seleccionado: ({self.current_candidate.position.x:.2f}, {self.current_candidate.position.y:.2f})")
            self.last_candidate = self.current_candidate
            self.last_candidate_features = (candidate_features, mask, action_index, probs)

        # Publicar candidato en RViz
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "nodo_candidato"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = self.current_candidate
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        self.marker_pub.publish(marker)

        nav_points = PoseArray()
        nav_points.header.stamp = self.get_clock().now().to_msg()
        nav_points.header.frame_id = "map"
        nav_points.poses.append(self.current_candidate)
        self.nav_point.publish(nav_points)

        # Actualizar experiencia
        if self.last_candidate_features is not None:
            stored_features, stored_mask, stored_action_index, stored_probs = self.last_candidate_features
            global_state = self.get_global_state()
            value = self.critic(tf.convert_to_tensor([global_state], dtype=tf.float32)).numpy()[0]
            reward = self.compute_reward()
            done = 1 if math.hypot(self.odom.position.x - self.goal.position.x,
                                    self.odom.position.y - self.goal.position.y) < GOAL_REACHED_DIST else 0
            self.states.append(global_state)
            self.actor_inputs.append((stored_features, stored_mask))
            self.actions.append(stored_action_index)
            self.log_probs.append(np.log(stored_probs[stored_action_index] + 1e-8))
            self.values.append(value)
            self.rewards.append(reward)
            self.dones.append(done)
        self.steps += 1

        if self.current_candidate is not None:
            dist_to_candidate = math.hypot(
                self.odom.position.x - self.current_candidate.position.x,
                self.odom.position.y - self.current_candidate.position.y)
            if dist_to_candidate < 2.5:
                self.get_logger().info("Candidato alcanzado. Seleccionando nuevo candidato.")
                self.current_candidate = None

        if self.current_candidate is not None:
            self.last_candidate = self.current_candidate

        if done or self.steps >= self.max_steps:
            self.get_logger().info(f"Episodio terminado en {self.steps} pasos.")
            self.update_model()
            elapsed = time.time() - self.start_time
            avg_ep_time = elapsed / (self.episode_count + 1)
            remaining = (self.total_episodes - (self.episode_count + 1)) * avg_ep_time
            self.get_logger().info(f"Tiempo transcurrido: {elapsed:.1f}s, Tiempo estimado restante: {remaining:.1f}s")
            self.states = []
            self.actor_inputs = []
            self.actions = []
            self.log_probs = []
            self.values = []
            self.rewards = []
            self.dones = []
            self.steps = 0
            self.current_candidate = None
            self.episode_count += 1
            self.actor_state = None

    def compute_advantages(self, rewards, values, dones):
        advantages = np.zeros_like(rewards, dtype=np.float32)
        last_adv = 0.0
        for t in reversed(range(len(rewards))):
            next_value = values[t+1] if t+1 < len(values) else 0.0
            delta = rewards[t] + GAMMA * next_value * (1 - dones[t]) - values[t]
            advantages[t] = last_adv = delta + GAMMA * LAMBDA * (1 - dones[t]) * last_adv
        returns = advantages + np.array(values, dtype=np.float32)
        return advantages, returns

    def save_models(self):
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        actor_model_filename = f"actor_model_{timestamp}.keras"
        critic_model_filename = f"critic_model_{timestamp}.keras"
        self.actor.save(actor_model_filename)
        self.critic.save(critic_model_filename)
        self.get_logger().info(f"Modelos guardados: {actor_model_filename} y {critic_model_filename}.")

    def update_model(self):
        states = np.array(self.states, dtype=np.float32)
        actions = np.array(self.actions, dtype=np.int32)
        log_probs_old = np.array(self.log_probs, dtype=np.float32)
        rewards = np.array(self.rewards, dtype=np.float32)
        dones = np.array(self.dones, dtype=np.float32)
        advantages, returns = self.compute_advantages(rewards, self.values, dones)
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)
        N = len(states)
        actor_inputs = []
        masks = []
        for cand_feat, m in self.actor_inputs:
            actor_inputs.append(cand_feat)
            masks.append(m)
        actor_inputs = np.array(actor_inputs, dtype=np.float32)
        masks = np.array(masks, dtype=bool)
        dataset = tf.data.Dataset.from_tensor_slices((states, actor_inputs, masks, actions, log_probs_old, returns, advantages))
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
        episode_reward = np.sum(self.rewards)
        episode_length = len(self.rewards)
        with self.summary_writer.as_default():
            tf.summary.scalar('actor_loss', avg_actor_loss, step=self.episode_count)
            tf.summary.scalar('critic_loss', avg_critic_loss, step=self.episode_count)
            tf.summary.scalar('total_loss', avg_total_loss, step=self.episode_count)
            tf.summary.scalar('episode_reward', episode_reward, step=self.episode_count)
            tf.summary.scalar('episode_length', episode_length, step=self.episode_count)
        self.get_logger().info(f"Métricas registradas para el episodio {self.episode_count}.")
        if not getattr(self, 'models_saved', False):
            self.save_models()
            self.models_saved = True
        if self.episode_count >= 400:
            self.get_logger().info("Se han completado 400 episodios. Finalizando nodo.")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = NavigationPPOCandidateTrainer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
