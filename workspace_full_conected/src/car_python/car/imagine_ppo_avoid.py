#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import numpy as np
import tensorflow as tf
import datetime
import time
import heapq
import random
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion, Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, DurabilityPolicy

# ----------------------- Constantes y Hiperparámetros -----------------------
GOAL_REACHED_DIST = 2.5       
STEP_PENALTY = -5.0          
GOAL_REWARD = 50.0            
OBSTACLE_PENALTY_DIST = 2.7    

MAX_CANDIDATES = 50            
FEATURE_DIM = 6               
GLOBAL_STATE_DIM = 7  

GAMMA = 0.99
LAMBDA = 0.95
CLIP_EPS = 0.2
TRAIN_EPOCHS = 30
BATCH_SIZE = 256

BONUS_WEIGHT = 0.7

TIMEOUT_THRESHOLD = 10.0  
WAYPOINT_THRESHOLD = 2.5  

SAME_CANDIDATE_THRESHOLD = 3
SAME_CANDIDATE_RESET_THRESHOLD = 4

MEMORY_WINDOW_SIZE = 200

RESET_REWARD = -70.0
VIRTUAL_COLLISION_PENALTY = 50.0  
PENALTY_RESET = -50.0      
PENALTY_COLLISION = -50.0  
PENALTY_NO_GOAL = -30.0    
PENALTY_MAX_STEPS = -30.0  

# Radio para la planificación incremental (ventana de replanificación)
WINDOW_RADIUS = 20.0

# ----------------------- Funciones Auxiliares -----------------------
def distance(p1, p2):
    return math.hypot(p2[0]-p1[0], p2[1]-p1[1])

def dijkstra(graph, start, goal):
    dist = {node: float('inf') for node in graph.keys()}
    prev = {node: None for node in graph.keys()}
    dist[start] = 0
    queue = [(0, start)]
    while queue:
        current_dist, current = heapq.heappop(queue)
        if current == goal:
            break
        if current_dist > dist[current]:
            continue
        for neighbor, weight in graph.get(current, []):
            alt = dist[current] + weight
            if alt < dist[neighbor]:
                dist[neighbor] = alt
                prev[neighbor] = current
                heapq.heappush(queue, (alt, neighbor))
    path = []
    node = goal
    while node is not None:
        path.append(node)
        node = prev[node]
    path.reverse()
    return path

def construir_grafo(nodes, threshold):
    N = len(nodes)
    graph = {i: [] for i in range(N)}
    for i in range(N):
        for j in range(i+1, N):
            d = distance(nodes[i], nodes[j])
            if d <= threshold:
                graph[i].append((j, d))
                graph[j].append((i, d))
    return graph

def quaternion_to_euler(q: Quaternion):
    sinr_cosp = 2.0*(q.w*q.x + q.y*q.z)
    cosr_cosp = 1.0 - 2.0*(q.x*q.x + q.y*q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0*(q.w*q.y - q.z*q.x)
    pitch = math.asin(sinp) if abs(sinp) <= 1 else math.copysign(math.pi/2, sinp)
    siny_cosp = 2.0*(q.w*q.z + q.x*q.y)
    cosy_cosp = 1.0 - 2.0*(q.y*q.y + q.z*q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

# ----------------------- PID Controller -----------------------
class PIDController:
    def __init__(self, kp, ki, kd, dt=0.1):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.prev_error = 0.0
        self.integral = 0.0
    def compute(self, setpoint, measured):
        error = setpoint - measured
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output
    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0

# ----------------------- Módulo de Imagination -----------------------
class ImaginationModule(tf.keras.Model):
    def __init__(self, feature_dim, hidden_units=64, **kwargs):
        super(ImaginationModule, self).__init__(**kwargs)
        self.dense1 = tf.keras.layers.Dense(hidden_units, activation='relu')
        self.dense2 = tf.keras.layers.Dense(hidden_units, activation='relu')
        self.bonus_layer = tf.keras.layers.Dense(1, activation=None)
    def call(self, candidate_features):
        x = self.dense1(candidate_features)
        x = self.dense2(x)
        bonus = self.bonus_layer(x)
        bonus = tf.squeeze(bonus, axis=-1)
        return bonus

# ----------------------- Actor Recurrente con Atención -----------------------
class RecurrentActorWithAttention(tf.keras.Model):
    def __init__(self, max_candidates, feature_dim, lstm_units=256, **kwargs):
        super(RecurrentActorWithAttention, self).__init__(**kwargs)
        self.td1 = tf.keras.layers.TimeDistributed(tf.keras.layers.Dense(64, activation='relu'))
        self.td2 = tf.keras.layers.TimeDistributed(tf.keras.layers.Dense(64, activation='relu'))
        self.td3 = tf.keras.layers.TimeDistributed(tf.keras.layers.Dense(64, activation='relu'))
        self.lstm = tf.keras.layers.LSTM(lstm_units, return_sequences=True, return_state=True)
        self.attention_dense = tf.keras.layers.TimeDistributed(tf.keras.layers.Dense(1))
        self.logits_layer = tf.keras.layers.TimeDistributed(tf.keras.layers.Dense(1))
    def call(self, x, mask=None, initial_state=None):
        x = self.td1(x)
        x = self.td2(x)
        x = self.td3(x)
        lstm_out, h, c = self.lstm(x, initial_state=initial_state)
        scores = self.attention_dense(lstm_out)
        scores = tf.squeeze(scores, axis=-1)
        if mask is not None:
            scores = tf.where(mask, scores, -1e9 * tf.ones_like(scores))
        attention_weights = tf.nn.softmax(scores, axis=1)
        candidate_logits = self.logits_layer(lstm_out)
        candidate_logits = tf.squeeze(candidate_logits, axis=-1)
        combined_logits = candidate_logits * attention_weights
        return combined_logits, (h, c)

# ----------------------- Clase de Entrenamiento Completo -----------------------
class NavigationEndToEndTrainer(Node):
    def __init__(self):
        super().__init__('navigation_end2end_trainer')
        qos_profile = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseArray, '/goal', self.goal_callback, 10)
        self.create_subscription(PoseArray, '/filtered_navigation_nodes', self.filtered_nodes_callback, 10)
        self.create_subscription(PoseArray, '/occupied_rejected_nodes', self.occupied_nodes_callback, 10)
        self.create_subscription(PoseArray, '/obstacle_navigation_nodes', self.obstacle_points_callback, 10)

        self.marker_pub = self.create_publisher(Marker, '/planned_path_marker', 10)
        self.nav_point = self.create_publisher(PoseArray, '/nav_point', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.reset_request_pub = self.create_publisher(Bool, '/reset_request', qos_profile)
        self.goal_reached_pub = self.create_publisher(Bool, 'goal_reached', 10)

        self.odom = None
        self.goal = None
        self.filtered_nodes = None
        self.occupied_nodes = None
        self.obstacle_points = None
        self.virtual_collision = False

        self.last_movement_time = None
        self.reset_cooldown = 20.0
        self.goal_reached_count = 0
        self.goal_reset_threshold = random.randint(2, 5)

        self.actor_inputs = []
        self.states = []
        self.actions = []
        self.log_probs = []
        self.rewards = []
        self.values = []
        self.dones = []
        self.steps = 0
        self.max_steps = 200

        self.episode_count = 0
        self.total_episodes = 1000
        self.start_time = time.time()
        self.log_dir = "logs/fit/" + datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        self.summary_writer = tf.summary.create_file_writer(self.log_dir)
        self.get_logger().info(f"TensorBoard logs en: {self.log_dir}")

        self.models_saved = False

        self.actor = RecurrentActorWithAttention(MAX_CANDIDATES, FEATURE_DIM)
        self.actor_state = None
        self.critic = tf.keras.Sequential([
            tf.keras.layers.InputLayer(input_shape=(GLOBAL_STATE_DIM,)),
            tf.keras.layers.Dense(64, activation='relu'),
            tf.keras.layers.Dense(64, activation='relu'),
            tf.keras.layers.Dense(1)
        ])
        self.actor_optimizer = tf.keras.optimizers.Adam(3e-4)
        self.critic_optimizer = tf.keras.optimizers.Adam(3e-4)
        self.imagination_module = ImaginationModule(FEATURE_DIM)

        self.state = "IDLE"
        self.state_start_time = None
        self.current_waypoint = None
        self.last_candidate = None
        self.same_candidate_count = 0
        self.last_action_index = 0
        self.last_probs = np.zeros(MAX_CANDIDATES)
        self.same_candidate_reset_threshold = SAME_CANDIDATE_RESET_THRESHOLD

        self.progress_buffer = []
        self.progress_window = 10
        self.progress_threshold = 0.1
        self.default_map_factor = 1.5
        self.stuck_map_factor = 2.0
        self.dynamic_map_factor = self.default_map_factor

        # Inicialización para planificación incremental
        self.get_logger().info("Navigation End-to-End Trainer iniciado.")
        while self.odom is None or self.goal is None:
            self.get_logger().warn("Esperando /odom y /goal...")
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("Datos iniciales recibidos, iniciando entrenamiento.")

        self.reactive_timer = self.create_timer(0.1, self.reactive_step)
        self.experience_timer = self.create_timer(0.5, self.experience_step)

    # ----------------------- Métodos de la Clase -----------------------
    def odom_callback(self, msg: Odometry):
        self.odom = msg.pose.pose
        self.last_movement_time = time.time()

    def goal_callback(self, msg: PoseArray):
        if msg.poses:
            self.goal = msg.poses[0]

    def filtered_nodes_callback(self, msg: PoseArray):
        self.filtered_nodes = msg

    def occupied_nodes_callback(self, msg: PoseArray):
        self.occupied_nodes = msg

    def obstacle_points_callback(self, msg: PoseArray):
        self.obstacle_points = msg

    def request_environment_reset(self):
        current_time = time.time()
        if current_time - self.last_movement_time >= self.reset_cooldown:
            reset_msg = Bool()
            reset_msg.data = True
            self.reset_request_pub.publish(reset_msg)
            self.get_logger().info("Solicitud de reinicio enviada.")
            self.last_movement_time = current_time

    # Método para calcular el centroide de k vecinos (planificación incremental)
    def compute_centroid(self, valid_nodes, k=3):
        if len(valid_nodes) < k:
            k = len(valid_nodes)
        sorted_nodes = sorted(valid_nodes, key=lambda node: math.hypot(node.position.x - self.goal.position.x,
                                                                        node.position.y - self.goal.position.y))
        k_neighbors = sorted_nodes[:k]
        sum_x = sum(node.position.x for node in k_neighbors)
        sum_y = sum(node.position.y for node in k_neighbors)
        centroid = Pose()
        centroid.position.x = sum_x / k
        centroid.position.y = sum_y / k
        centroid.orientation = k_neighbors[0].orientation
        return centroid

    def plan_route(self, candidate):
        current_pos = (self.odom.position.x, self.odom.position.y)
        candidate_pos = (candidate.position.x, candidate.position.y)
        nodes = []
        nodes.append((current_pos, 'actual'))
        if self.filtered_nodes is not None and self.filtered_nodes.poses:
            for node in self.filtered_nodes.poses:
                # Filtrar solo los nodos dentro de WINDOW_RADIUS
                if distance((node.position.x, node.position.y), current_pos) < WINDOW_RADIUS:
                    pos = (node.position.x, node.position.y)
                    nodes.append((pos, 'filtrado'))
        nodes.append((candidate_pos, 'candidato'))
        graph = construir_grafo([n[0] for n in nodes], 3.0)
        indices_camino = dijkstra(graph, 0, len(nodes)-1)
        camino_calculado = [nodes[i][0] for i in indices_camino]
        if len(indices_camino) >= 2:
            next_waypoint = camino_calculado[1]
            new_pose = Pose()
            new_pose.position.x = next_waypoint[0]
            new_pose.position.y = next_waypoint[1]
            new_pose.orientation = candidate.orientation
            return new_pose, camino_calculado
        else:
            return candidate, camino_calculado

    def compute_candidate_features_tf(self):
        if self.filtered_nodes is None or not self.filtered_nodes.poses:
            return None, None, None
        # Filtrar nodos dentro de WINDOW_RADIUS para planificación incremental
        filtered_in_window = [node for node in self.filtered_nodes.poses 
                              if distance((node.position.x, node.position.y), (self.odom.position.x, self.odom.position.y)) < WINDOW_RADIUS]
        if not filtered_in_window:
            return None, None, None

        robot_x = tf.constant(self.odom.position.x, dtype=tf.float32)
        robot_y = tf.constant(self.odom.position.y, dtype=tf.float32)
        goal_x = tf.constant(self.goal.position.x, dtype=tf.float32)
        goal_y = tf.constant(self.goal.position.y, dtype=tf.float32)
        candidate_features = []
        valid_nodes = []
        for node in filtered_in_window:
            node_x = tf.constant(node.position.x, dtype=tf.float32)
            node_y = tf.constant(node.position.y, dtype=tf.float32)
            if self.occupied_nodes is not None and self.occupied_nodes.poses:
                occ_distances = []
                for occ in self.occupied_nodes.poses:
                    occ_x = tf.constant(occ.position.x, dtype=tf.float32)
                    occ_y = tf.constant(occ.position.y, dtype=tf.float32)
                    d = tf.norm(tf.stack([node_x - occ_x, node_y - occ_y]))
                    occ_distances.append(d)
                clearance = tf.reduce_min(tf.stack(occ_distances))
            else:
                clearance = tf.constant(10.0, dtype=tf.float32)
            if clearance < OBSTACLE_PENALTY_DIST:
                continue
            dist_robot = tf.norm(tf.stack([robot_x - node_x, robot_y - node_y]))
            dist_to_goal = tf.norm(tf.stack([node_x - goal_x, node_y - goal_y]))
            angle_diff = tf.constant(0.0, dtype=tf.float32)
            feature_vector = tf.stack([dist_robot, angle_diff, clearance, dist_to_goal, 0.0, 0.0])
            candidate_features.append(feature_vector)
            valid_nodes.append(node)
        if len(candidate_features) == 0:
            return None, None, None
        sorted_pairs = sorted(zip(candidate_features, valid_nodes), key=lambda x: x[0][3].numpy())
        candidate_features, valid_nodes = zip(*sorted_pairs)
        candidate_features = list(candidate_features)
        valid_nodes = list(valid_nodes)
        if len(candidate_features) > MAX_CANDIDATES:
            candidate_features = candidate_features[:MAX_CANDIDATES]
            valid_nodes = valid_nodes[:MAX_CANDIDATES]
        num_valid = len(candidate_features)
        while len(candidate_features) < MAX_CANDIDATES:
            candidate_features.append(tf.zeros([FEATURE_DIM], dtype=tf.float32))
        features_tensor = tf.stack(candidate_features)
        mask = tf.concat([tf.ones([num_valid], dtype=tf.bool),
                          tf.zeros([MAX_CANDIDATES - num_valid], dtype=tf.bool)], axis=0)
        features_tensor = tf.expand_dims(features_tensor, axis=0)
        mask = tf.expand_dims(mask, axis=0)
        return features_tensor, list(valid_nodes), mask

    def follow_trajectory(self, waypoint):
        lookahead_dist = 1.0
        EVASION_THRESHOLD = 1.0
        EVASION_WEIGHT = 0.8

        current_pos = np.array([self.odom.position.x, self.odom.position.y])
        target_pos = np.array([waypoint.position.x, waypoint.position.y])
        distance_error = np.linalg.norm(target_pos - current_pos)
        angle_desired = math.atan2(target_pos[1]-current_pos[1], target_pos[0]-current_pos[0])
        _, _, yaw_actual = quaternion_to_euler(self.odom.orientation)
        error_angle = angle_desired - yaw_actual
        error_angle = math.atan2(math.sin(error_angle), math.cos(error_angle))

        # Desacelerar a medida que se acerca al waypoint
        decel_factor = 1.0
        if distance_error < 0.5:
            decel_factor = distance_error / 0.5

        KP_linear, KP_angular = 0.8, 1.2
        linear_output = KP_linear * distance_error * decel_factor
        angular_output = KP_angular * error_angle

        repulsive_force = np.array([0.0, 0.0])
        if self.obstacle_points is not None and self.obstacle_points.poses:
            for obs in self.obstacle_points.poses:
                obs_pos = np.array([obs.position.x, obs.position.y])
                diff = current_pos - obs_pos
                dist_obs = np.linalg.norm(diff)
                if dist_obs < EVASION_THRESHOLD and dist_obs > 0.001:
                    repulsion = (EVASION_THRESHOLD - dist_obs)/EVASION_THRESHOLD * (diff/dist_obs)
                    repulsive_force += repulsion
            if np.linalg.norm(repulsive_force) > 0.001:
                rep_angle = math.atan2(repulsive_force[1], repulsive_force[0])
                error_rep = rep_angle - yaw_actual
                error_rep = math.atan2(math.sin(error_rep), math.cos(error_rep))
                angular_output += EVASION_WEIGHT * error_rep
                self.get_logger().info(f"Evasión: error_rep={error_rep:.2f}")

        if abs(error_angle) > 0.3:
            linear_output *= 0.5

        twist_msg = Twist()
        twist_msg.linear.x = float(linear_output)
        twist_msg.angular.z = float(angular_output)
        self.cmd_vel_pub.publish(twist_msg)
        self.get_logger().info(f"Follow Trajectory: dist_error={distance_error:.2f}, error_angle={error_angle:.2f}, linear={twist_msg.linear.x:.2f}, angular={twist_msg.angular.z:.2f}")

    def compute_reward(self, planned_waypoint):
        current_pos = np.array([self.odom.position.x, self.odom.position.y])
        goal_pos = np.array([self.goal.position.x, self.goal.position.y])
        d_current = np.linalg.norm(current_pos - goal_pos)
        wp = np.array([planned_waypoint.position.x, planned_waypoint.position.y])
        d_final = np.linalg.norm(wp - goal_pos)
        progress_reward = (d_current - d_final) * 20.0
        reward = progress_reward + STEP_PENALTY
        if d_current <= GOAL_REACHED_DIST:
            reward += GOAL_REWARD
        if self.virtual_collision:
            self.get_logger().warn("Colisión virtual detectada. Penalizando.")
            reward += - (VIRTUAL_COLLISION_PENALTY + PENALTY_COLLISION)
            self.virtual_collision = False
        return reward

    def update_model(self):
        if len(self.states) == 0:
            self.get_logger().info("No se recogieron experiencias, omitiendo actualización.")
            return
        states = np.array(self.states, dtype=np.float32)
        actions = np.array(self.actions, dtype=np.int32)
        log_probs_old = np.array(self.log_probs, dtype=np.float32)
        rewards = np.array(self.rewards, dtype=np.float32)
        dones = np.array(self.dones, dtype=np.float32)
        advantages = np.zeros_like(rewards, dtype=np.float32)
        last_adv = 0.0
        for t in reversed(range(len(rewards))):
            next_value = self.values[t+1] if t+1 < len(self.values) else 0.0
            delta = rewards[t] + GAMMA * next_value * (1-dones[t]) - self.values[t]
            advantages[t] = last_adv = delta + GAMMA * LAMBDA * (1-dones[t]) * last_adv
        returns = advantages + np.array(self.values, dtype=np.float32)
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)

        actor_inputs = []
        masks = []
        for feat, m in self.actor_inputs:
            actor_inputs.append(feat)
            masks.append(m)
        actor_inputs = np.array(actor_inputs, dtype=np.float32)
        masks = np.array(masks, dtype=bool)
        actor_inputs = tf.convert_to_tensor(actor_inputs)
        masks = tf.convert_to_tensor(masks)
        actor_inputs = tf.squeeze(actor_inputs, axis=1)
        masks = tf.squeeze(masks, axis=1)

        N = len(states)
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
                    logits, _ = self.actor(batch_actor_inputs, mask=tf.convert_to_tensor(batch_masks), initial_state=None)
                    bonus = self.imagination_module(batch_actor_inputs)
                    combined_logits = logits + BONUS_WEIGHT * bonus
                    batch_actions = tf.cast(batch_actions, tf.int32)
                    indices = tf.stack([tf.range(tf.shape(combined_logits)[0]), batch_actions], axis=1)
                    new_log_probs = tf.math.log(tf.nn.softmax(combined_logits, axis=1) + 1e-8)
                    new_log_probs = tf.gather_nd(new_log_probs, indices)
                    ratio = tf.exp(new_log_probs - batch_old_log_probs)
                    clipped_ratio = tf.clip_by_value(ratio, 1-CLIP_EPS, 1+CLIP_EPS)
                    actor_loss = -tf.reduce_mean(tf.minimum(ratio * batch_advantages, clipped_ratio * batch_advantages))
                    values_pred = self.critic(batch_states)
                    critic_loss = tf.reduce_mean(tf.square(batch_returns - values_pred))
                    total_loss = actor_loss + 0.5 * critic_loss
                actor_imagination_vars = self.actor.trainable_variables + self.imagination_module.trainable_variables
                actor_imagination_grads = tape.gradient(total_loss, actor_imagination_vars)
                self.actor_optimizer.apply_gradients(zip(actor_imagination_grads, actor_imagination_vars))
                critic_grads = tape.gradient(total_loss, self.critic.trainable_variables)
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
        if not getattr(self, 'models_saved', False):
            self.save_models()
            self.models_saved = True

    def save_models(self):
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        actor_model_filename = f"actor_model_{timestamp}.keras"
        critic_model_filename = f"critic_model_{timestamp}.keras"
        imagination_model_filename = f"imagination_model_{timestamp}.keras"
        self.actor.save(actor_model_filename)
        self.critic.save(critic_model_filename)
        self.imagination_module.save(imagination_model_filename)
        self.get_logger().info(f"Modelos guardados: {actor_model_filename}, {critic_model_filename} y {imagination_model_filename}.")

    # ----------------------- Bucle Reactivo -----------------------
    def reactive_step(self):
        if self.odom is None or self.goal is None:
            return
        # Recalcular candidatos en cada ciclo
        candidate_features, valid_nodes, mask = self.compute_candidate_features_tf()
        if candidate_features is None:
            self.get_logger().warn("No hay candidatos válidos.")
            return

        # Seleccionar el nuevo candidato usando el centroide de k vecinos
        new_candidate = self.compute_centroid(valid_nodes, k=3)
        # Actualizar el candidato actual y sus features
        self.last_candidate_features = (candidate_features, mask)
        self.last_candidate = new_candidate
        self.current_candidate = new_candidate
        # (Opcional) Actualizar también el índice y las probabilidades si se usan en experiencia
        self.last_action_index = 0
        self.last_probs = tf.nn.softmax(self.actor(candidate_features, mask=mask, initial_state=self.actor_state)[0], axis=-1).numpy()[0]

        self.state = "MOVING"
        self.state_start_time = self.get_clock().now().nanoseconds / 1e9

        planned_waypoint, computed_path = self.plan_route(self.current_candidate)
        self.current_waypoint = planned_waypoint

        # Publicar waypoint y la trayectoria calculada para visualización
        nav_points = PoseArray()
        nav_points.header.stamp = self.get_clock().now().to_msg()
        nav_points.header.frame_id = "map"
        nav_points.poses.append(planned_waypoint)
        self.nav_point.publish(nav_points)

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "planned_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.points = [Point(x=pt[0], y=pt[1], z=0.0) for pt in computed_path]
        self.marker_pub.publish(marker)

        # Enviar comando de velocidad basado en el nuevo waypoint
        self.follow_trajectory(planned_waypoint)

        # Actualizar buffer de progreso
        current_goal_distance = distance((self.odom.position.x, self.odom.position.y),
                                        (self.goal.position.x, self.goal.position.y))
        self.progress_buffer.append(current_goal_distance)
        if len(self.progress_buffer) > self.progress_window:
            self.progress_buffer.pop(0)
            if (self.progress_buffer[0] - self.progress_buffer[-1]) < self.progress_threshold:
                self.dynamic_map_factor = self.stuck_map_factor
                self.get_logger().warn("Progreso insuficiente, aumentando penalización a nodos del mapa.")
            else:
                self.dynamic_map_factor = self.default_map_factor


    # ----------------------- Bucle de Experiencia -----------------------
    def experience_step(self):
        if self.state == "MOVING":
            if distance((self.odom.position.x, self.odom.position.y),
                        (self.current_waypoint.position.x, self.current_waypoint.position.y)) < WAYPOINT_THRESHOLD:
                self.get_logger().info("Waypoint alcanzado. Registrando experiencia.")
                reward = self.compute_reward(self.current_waypoint)
                if self.same_candidate_count >= SAME_CANDIDATE_THRESHOLD:
                    self.get_logger().warn("Candidato repetido demasiadas veces. Penalizando.")
                    reward -= 20.0
                    self.same_candidate_count = 0
                global_state = np.array([
                    self.odom.position.x,
                    self.odom.position.y,
                    self.goal.position.x,
                    self.goal.position.y,
                    0.0, 0.0, 0.0
                ], dtype=np.float32)
                value = self.critic(tf.convert_to_tensor([global_state], dtype=tf.float32)).numpy()[0]
                # Se considera que se ha alcanzado el goal si la distancia es menor que GOAL_REACHED_DIST
                done = 1 if distance((self.odom.position.x, self.odom.position.y),
                                    (self.goal.position.x, self.goal.position.y)) < GOAL_REACHED_DIST else 0
                self.states.append(global_state)
                self.actions.append(self.last_action_index)
                self.log_probs.append(np.log(self.last_probs[self.last_action_index] + 1e-8))
                self.values.append(value)
                self.rewards.append(reward)
                self.dones.append(done)
                self.actor_inputs.append(self.last_candidate_features)
                self.steps += 1

                if done:
                    # Publicar mensaje de goal alcanzado
                    goal_msg = Bool()
                    goal_msg.data = True
                    self.goal_reached_pub.publish(goal_msg)
                    self.get_logger().info("Goal alcanzado. Deteniendo el robot.")
                    # Enviar comando de detención total
                    stop_twist = Twist()
                    stop_twist.linear.x = 0.0
                    stop_twist.angular.z = 0.0
                    self.cmd_vel_pub.publish(stop_twist)
                    # Aquí se podría esperar a que se complete el entrenamiento (por ejemplo, un sleep o una lógica de bloqueo)
                    time.sleep(2)  # Espera 2 segundos (ajusta según lo necesario)
                    self.goal_reached_count += 1
                    self.get_logger().info(f"Meta alcanzada {self.goal_reached_count} veces (umbral: {self.goal_reset_threshold}).")
                    if self.goal_reached_count >= self.goal_reset_threshold:
                        self.get_logger().info("Número de veces alcanzada la meta supera umbral. Solicitando reinicio.")
                        self.request_environment_reset()
                        self.goal_reached_count = 0
                        self.goal_reset_threshold = random.randint(2, 5)
                else:
                    # Si no se ha alcanzado el goal, actualizamos el waypoint sin detener la inferencia
                    self.reactive_step()
        else:
            current_time = self.get_clock().now().nanoseconds / 1e9
            if current_time - self.state_start_time > TIMEOUT_THRESHOLD:
                self.get_logger().warn("Timeout en MOVING. Penalizando y reiniciando candidato.")
                penalty_reward = -10.0
                global_state = np.array([
                    self.odom.position.x,
                    self.odom.position.y,
                    self.goal.position.x,
                    self.goal.position.y,
                    0.0, 0.0, 0.0
                ], dtype=np.float32)
                value = self.critic(tf.convert_to_tensor([global_state], dtype=tf.float32)).numpy()[0]
                done = 0
                self.states.append(global_state)
                self.actions.append(self.last_action_index)
                self.log_probs.append(np.log(self.last_probs[self.last_action_index] + 1e-8))
                self.values.append(value)
                self.rewards.append(penalty_reward)
                self.dones.append(done)
                self.actor_inputs.append(self.last_candidate_features)
                self.steps += 1
                self.reactive_step()

        if len(self.states) > MEMORY_WINDOW_SIZE:
            self.states.pop(0)
            self.actions.pop(0)
            self.log_probs.pop(0)
            self.values.pop(0)
            self.rewards.pop(0)
            self.dones.pop(0)
            self.actor_inputs.pop(0)

        if self.steps >= self.max_steps or (self.odom and self.goal and 
        distance((self.odom.position.x, self.odom.position.y), (self.goal.position.x, self.goal.position.y)) < GOAL_REACHED_DIST):
            self.get_logger().info(f"Episodio terminado en {self.steps} pasos.")
            current_goal_distance = distance((self.odom.position.x, self.odom.position.y),
                                            (self.goal.position.x, self.goal.position.y))
            if current_goal_distance >= GOAL_REACHED_DIST and len(self.rewards) > 0:
                self.rewards[-1] += PENALTY_NO_GOAL
            if self.steps >= self.max_steps and len(self.rewards) > 0:
                self.rewards[-1] += PENALTY_MAX_STEPS
            self.update_model()
            elapsed = time.time() - self.start_time
            avg_ep_time = elapsed / (self.episode_count + 1)
            remaining = (self.total_episodes - (self.episode_count + 1)) * avg_ep_time
            self.get_logger().info(f"Tiempo transcurrido: {elapsed:.1f}s, restante: {remaining:.1f}s")
            self.states = []
            self.actions = []
            self.log_probs = []
            self.values = []
            self.rewards = []
            self.dones = []
            self.actor_inputs = []
            self.steps = 0
            self.episode_count += 1
            self.actor_state = None
            self.state = "IDLE"
            progress_bar_length = 20
            completed_units = int((self.episode_count / self.total_episodes) * progress_bar_length)
            progress_bar = "[" + "#" * completed_units + "-" * (progress_bar_length - completed_units) + "]"
            self.get_logger().info(f"Episodios: {self.episode_count}/{self.total_episodes} {progress_bar}")

def main(args=None):
    rclpy.init(args=args)
    node = NavigationEndToEndTrainer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
