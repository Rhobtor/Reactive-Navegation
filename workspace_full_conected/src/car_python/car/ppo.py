#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import numpy as np
import tensorflow as tf
import datetime
import time
import heapq
import os
import signal
import random

from std_srvs.srv import Empty
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from octomap_msgs.msg import Octomap
from rclpy.qos import QoSProfile, DurabilityPolicy

# ----------------------- Constantes y Hiperparámetros -----------------------
GOAL_REACHED_DIST = 3.0       
STEP_PENALTY = -5.0          
GOAL_REWARD = 50.0            
OBSTACLE_PENALTY_DIST = 2.7   

MAX_CANDIDATES = 50            
FEATURE_DIM = 6               
GLOBAL_STATE_DIM = 7  

GAMMA = 0.99
LAMBDA = 0.95
CLIP_EPS = 0.2
TRAIN_EPOCHS = 10
BATCH_SIZE = 256

EXPLORATION_DISTANCE_THRESHOLD = 3.0  
EXPLORATION_BONUS_FACTOR = 70.0         
CLEARANCE_BONUS_FACTOR = 60.0           

MEMORY_WINDOW_SIZE = 200  # Máximo número de experiencias a retener

# Parámetros para la máquina de estados
TIMEOUT_THRESHOLD = 10.0  # segundos de timeout en estado MOVING
WAYPOINT_THRESHOLD = 2.5  # distancia en metros para considerar alcanzado el waypoint

# Parámetros para evitar la trampa: si se repite el mismo candidato demasiadas veces
SAME_CANDIDATE_THRESHOLD = 3
SAME_CANDIDATE_RESET_THRESHOLD = 4

# Nuevas constantes para reset y colisión virtual
RESET_REWARD = -70.0
VIRTUAL_COLLISION_PENALTY = 50.0  # Valor positivo; se restará para penalizar

# Penalizaciones adicionales:
PENALTY_RESET = -50.0      # Penalización extra al resetear
PENALTY_COLLISION = -50.0  # Penalización extra si choca
PENALTY_NO_GOAL = -30.0    # Penalización si finaliza el episodio sin alcanzar el goal
PENALTY_MAX_STEPS = -30.0  # Penalización si se alcanza el límite de pasos

# ----------------------- Funciones Auxiliares para el Planificador Local -----------------------
def distance(p1, p2):
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

def dijkstra(grafo, inicio, objetivo):
    dist = {nodo: float('inf') for nodo in grafo.keys()}
    prev = {nodo: None for nodo in grafo.keys()}
    dist[inicio] = 0
    cola = [(0, inicio)]
    while cola:
        costo_actual, actual = heapq.heappop(cola)
        if actual == objetivo:
            break
        if costo_actual > dist[actual]:
            continue
        for vecino, peso in grafo.get(actual, []):
            alt = dist[actual] + peso
            if alt < dist[vecino]:
                dist[vecino] = alt
                prev[vecino] = actual
                heapq.heappush(cola, (alt, vecino))
    camino = []
    nodo = objetivo
    while nodo is not None:
        camino.append(nodo)
        nodo = prev[nodo]
    camino.reverse()
    return camino

def construir_grafo(nodos, umbral_conexion):
    N = len(nodos)
    grafo = {i: [] for i in range(N)}
    for i in range(N):
        for j in range(i+1, N):
            d = distance(nodos[i], nodos[j])
            if d <= umbral_conexion:
                grafo[i].append((j, d))
                grafo[j].append((i, d))
    return grafo

# ----------------------- Conversión de Cuaternión a Yaw y Euler -----------------------
def quaternion_to_yaw(q: Quaternion):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def quaternion_to_euler(q: Quaternion):
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

# ----------------------- Actor Recurrente con Atención -----------------------
class RecurrentActorWithAttention(tf.keras.Model):
    def __init__(self, max_candidates, feature_dim, lstm_units=256, **kwargs):
        super(RecurrentActorWithAttention, self).__init__(**kwargs)
        self.max_candidates = max_candidates
        self.feature_dim = feature_dim
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

# ----------------------- Nodo de Entrenamiento End-to-End -----------------------
class NavigationEndToEndTrainer(Node):
    def __init__(self):
        super().__init__('navigation_end2end_trainer')
        qos_profile = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseArray, '/goal', self.goal_callback, 10)
        self.create_subscription(PoseArray, '/filtered_navigation_nodes', self.filtered_nodes_callback, 10)
        self.create_subscription(PoseArray, '/occupied_rejected_nodes', self.occupied_nodes_callback, 10)
        self.create_subscription(PoseArray, '/obstacle_points', self.obstacle_points_callback, 10)
        self.create_subscription(Bool, '/virtual_collision', self.collision_callback, 10)
        self.create_subscription(Bool, '/reset_confirmation', self.map_reset_callback, qos_profile)
        self.marker_pub = self.create_publisher(Marker, '/planned_path_marker', 10)
        self.nav_point = self.create_publisher(PoseArray, '/nav_point', 10)
        self.reset_request_pub = self.create_publisher(Bool, '/reset_request', qos_profile)
        self.goal_reached_pub = self.create_publisher(Bool, 'goal_reached', 10)
        self.reset_confirmation_received = False
        self.reset_triggered = False
        # Manejo asíncrono de la confirmación de reinicio
        self.reset_request_pending = False
        self.reset_request_start_time = 0.0
        self.reset_timeout = 120.0  # 2 minutos

        self.odom = None
        self.goal = None
        self.filtered_nodes = None
        self.occupied_nodes = None
        self.obstacle_points = None
        self.virtual_collision = False

        self.last_movement_time = None
        self.movement_threshold = 0.01
        self.inactivity_time_threshold = 300.0

        self.last_reset_time = 0.0
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
        self.max_steps = 1000

        self.episode_count = 0
        self.total_episodes = 200
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

        self.state = "IDLE"
        self.state_start_time = None
        self.initial_position = None

        self.TIMEOUT_THRESHOLD = TIMEOUT_THRESHOLD
        self.WAYPOINT_THRESHOLD = WAYPOINT_THRESHOLD

        self.current_waypoint = None

        self.last_candidate = None
        self.same_candidate_count = 0
        self.same_candidate_reset_threshold = SAME_CANDIDATE_RESET_THRESHOLD
        self.same_candidate_threshold = SAME_CANDIDATE_THRESHOLD

        self.get_logger().info("Navigation End-to-End Trainer iniciado.")
        while self.odom is None or self.goal is None:
            self.get_logger().warn("Esperando /odom y /goal...")
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("Datos iniciales recibidos, iniciando entrenamiento.")

        self.reactive_timer = self.create_timer(0.1, self.reactive_step)
        self.experience_timer = self.create_timer(0.5, self.experience_step)
        self.reset_check_timer = self.create_timer(0.1, self.check_reset_confirmation)

    # ----------------------- Callbacks -----------------------
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

        linear = msg.twist.twist.linear
        speed = math.hypot(linear.x, linear.y)
        self.last_speed = speed
        if self.last_movement_time is None or speed > self.movement_threshold:
            self.last_movement_time = current_time

        # Detectar vuelco mediante Euler
        roll, pitch, yaw = quaternion_to_euler(original_pose.orientation)
        threshold = math.pi / 4  # 45° en radianes
        if abs(roll) > threshold or abs(pitch) > threshold:
            self.get_logger().error("Robot volcado detectado (roll: {:.2f}, pitch: {:.2f}). Solicitando reinicio.".format(roll, pitch))
            self.request_environment_reset()

    def goal_callback(self, msg: PoseArray):
        if msg.poses:
            self.goal = msg.poses[0]

    def filtered_nodes_callback(self, msg: PoseArray):
        self.filtered_nodes = msg

    def occupied_nodes_callback(self, msg: PoseArray):
        self.occupied_nodes = msg

    def obstacle_points_callback(self, msg: PoseArray):
        self.obstacle_points = msg

    def collision_callback(self, msg: Bool):
        self.virtual_collision = msg.data

    def map_reset_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info("Se recibió señal de confirmación para reiniciar el mapa.")
            self.reset_confirmation_received = True
            self.reset_triggered = True

    # ----------------------- Solicitar Reinicio (Modo asíncrono) -----------------------
    def request_environment_reset(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.last_reset_time >= self.reset_cooldown:
            reset_msg = Bool()
            reset_msg.data = True
            self.reset_request_pub.publish(reset_msg)
            self.get_logger().info("Solicitud de reinicio enviada. Esperando confirmación de forma asíncrona...")
            self.last_reset_time = current_time
            self.reset_request_pending = True
            self.reset_request_start_time = time.time()
        else:
            self.get_logger().info("Cooldown activo; no se solicita reset.")

    def check_reset_confirmation(self):
        if self.reset_request_pending:
            if self.reset_confirmation_received:
                self.get_logger().info("Confirmación de reinicio recibida.")
                self.reset_request_pending = False
                self.reset_confirmation_received = False
            elif time.time() - self.reset_request_start_time > self.reset_timeout:
                self.get_logger().warn("No se recibió confirmación de reinicio dentro del tiempo límite.")
                self.reset_request_pending = False
                self.reset_confirmation_received = False

    # ----------------------- Función de Recompensa -----------------------
    def compute_reward(self, planned_waypoint):
        current_pos = np.array([self.odom.position.x, self.odom.position.y])
        goal_pos = np.array([self.goal.position.x, self.goal.position.y])
        d_current = np.linalg.norm(current_pos - goal_pos)
        wp = np.array([planned_waypoint.position.x, planned_waypoint.position.y])
        d_final = np.linalg.norm(wp - goal_pos)
        progress_reward = (d_current - d_final) * 20.0
        safety_penalty = 0.0
        if self.obstacle_points is not None and self.obstacle_points.poses:
            d_list = [distance((wp[0], wp[1]), (p.position.x, p.position.y))
                      for p in self.obstacle_points.poses]
            if d_list:
                min_wp = min(d_list)
                if min_wp < 1.0:
                    safety_penalty += (1.0 - min_wp) * 50.0
        collision_penalty = 0.0
        if self.occupied_nodes and self.occupied_nodes.poses:
            for occ in self.occupied_nodes.poses:
                d = distance((self.odom.position.x, self.odom.position.y),
                             (occ.position.x, occ.position.y))
                if d < 0.5:
                    collision_penalty += 1000.0
        reward = progress_reward - safety_penalty - collision_penalty + STEP_PENALTY
        # Si se alcanza el goal, se suma la recompensa
        if d_current < GOAL_REACHED_DIST:
            reward += GOAL_REWARD
        # Penalizar por colisión virtual (se suman ambas penalizaciones)
        if self.virtual_collision:
            self.get_logger().warn("Colisión virtual detectada. Penalizando.")
            reward += - (VIRTUAL_COLLISION_PENALTY + PENALTY_COLLISION)
            self.virtual_collision = False
        return reward

    # ----------------------- Cálculo de Características para Candidatos -----------------------
    def compute_candidate_features(self):
        features = []
        valid_nodes = []
        if self.filtered_nodes is None or not self.filtered_nodes.poses:
            return None, None, None
        robot_x = self.odom.position.x
        robot_y = self.odom.position.y
        goal_x = self.goal.position.x
        goal_y = self.goal.position.y
        for node in self.filtered_nodes.poses:
            if self.occupied_nodes and self.occupied_nodes.poses:
                clearance = min([distance((node.position.x, node.position.y),
                                           (occ.position.x, occ.position.y))
                                 for occ in self.occupied_nodes.poses])
            else:
                clearance = 10.0
            if clearance < OBSTACLE_PENALTY_DIST:
                continue
            dist_robot = distance((robot_x, robot_y), (node.position.x, node.position.y))
            dist_to_goal = distance((node.position.x, node.position.y), (goal_x, goal_y))
            angle_diff = 0.0
            feature_vector = [dist_robot, angle_diff, clearance, dist_to_goal, 0.0, 0.0]
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
            features.append([0.0] * FEATURE_DIM)
        features = np.array(features, dtype=np.float32)
        mask = np.array([True] * num_valid + [False] * (MAX_CANDIDATES - num_valid))
        return features, valid_nodes, mask

    # ----------------------- Planificación Local Basada en filtered_nodes -----------------------
    def plan_route(self, candidate):
        if self.filtered_nodes is None or not self.filtered_nodes.poses:
            candidate_pose = candidate
            camino = [(self.odom.position.x, self.odom.position.y),
                      (candidate.position.x, candidate.position.y)]
            return candidate_pose, camino

        current_pos = (self.odom.position.x, self.odom.position.y)
        candidate_pos = (candidate.position.x, candidate.position.y)
        nodos = [current_pos]
        for node in self.filtered_nodes.poses:
            nodos.append((node.position.x, node.position.y))
        nodos.append(candidate_pos)
        umbral_conexion = 3.0
        grafo = construir_grafo(nodos, umbral_conexion)
        inicio = 0
        objetivo = len(nodos) - 1
        indices_camino = dijkstra(grafo, inicio, objetivo)
        camino_calculado = [nodos[i] for i in indices_camino]
        if len(indices_camino) >= 2:
            next_index = indices_camino[1]
            next_waypoint = nodos[next_index]
            new_pose = Pose()
            new_pose.position.x = next_waypoint[0]
            new_pose.position.y = next_waypoint[1]
            new_pose.orientation = candidate.orientation
            return new_pose, camino_calculado
        else:
            return candidate, camino_calculado

    # ----------------------- Bucle Reactivo: Actualización Continua del Waypoint -----------------------
    def reactive_step(self):
        if self.odom is None or self.goal is None:
            return
        if self.reset_triggered:
            return

        if self.state == "IDLE":
            candidate_features, valid_nodes, mask = self.compute_candidate_features()
            if candidate_features is None:
                self.get_logger().warn("No hay candidatos válidos.")
                return
            self.last_candidate_features = (candidate_features, mask)
            actor_input = np.expand_dims(candidate_features, axis=0)
            mask_input = np.expand_dims(mask, axis=0)
            logits, self.actor_state = self.actor(actor_input, mask=tf.convert_to_tensor(mask_input), initial_state=self.actor_state)
            probs = tf.nn.softmax(logits, axis=-1).numpy()[0]
            action_index = int(np.argmax(probs))
            if not mask[action_index]:
                self.get_logger().warn("Candidato seleccionado inválido.")
                return
            new_candidate = valid_nodes[action_index]
            if self.last_candidate is not None:
                d_same = math.hypot(new_candidate.position.x - self.last_candidate.position.x,
                                    new_candidate.position.y - self.last_candidate.position.y)
                if d_same < 0.1:
                    self.same_candidate_count += 1
                else:
                    self.same_candidate_count = 0
            else:
                self.same_candidate_count = 0
            self.last_candidate = new_candidate
            self.current_candidate = new_candidate
            self.last_action_index = action_index
            self.last_probs = probs
            self.get_logger().info(f"Candidato seleccionado: ({new_candidate.position.x:.2f}, {new_candidate.position.y:.2f}), Repeticiones: {self.same_candidate_count}")
            if self.same_candidate_count >= self.same_candidate_reset_threshold:
                self.get_logger().warn("Se ha seleccionado el mismo candidato demasiadas veces. Solicitando reinicio.")
                self.request_environment_reset()
                self.same_candidate_count = 0
                self.state = "IDLE"
                return
            self.state = "MOVING"
            self.state_start_time = self.get_clock().now().nanoseconds / 1e9
        else:
            if not hasattr(self, 'last_action_index'):
                self.last_action_index = 0
            if not hasattr(self, 'last_probs'):
                self.last_probs = np.zeros(MAX_CANDIDATES)
            if not hasattr(self, 'last_candidate_features'):
                candidate_features, valid_nodes, mask = self.compute_candidate_features()
                self.last_candidate_features = (candidate_features, mask)

        planned_waypoint, computed_path = self.plan_route(self.current_candidate)
        self.current_waypoint = planned_waypoint
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

    # ----------------------- Bucle de Experiencia: Registro cuando se alcanza el waypoint -----------------------
    def experience_step(self):
        if self.reset_triggered:
            self.get_logger().info("Reset solicitado. Terminando episodio y registrando recompensa de reinicio.")
            # Al resetear, se suma una penalización adicional
            reset_reward = RESET_REWARD + PENALTY_RESET
            global_state = np.array([
                self.odom.position.x,
                self.odom.position.y,
                self.goal.position.x,
                self.goal.position.y,
                0.0, 0.0, 0.0
            ], dtype=np.float32)
            value = self.critic(tf.convert_to_tensor([global_state], dtype=tf.float32)).numpy()[0]
            done = 1
            action = self.last_action_index if hasattr(self, 'last_action_index') else 0
            log_prob = np.log(self.last_probs[self.last_action_index] + 1e-8) if hasattr(self, 'last_probs') else 0.0
            self.states.append(global_state)
            self.actions.append(action)
            self.log_probs.append(log_prob)
            self.values.append(value)
            self.rewards.append(reset_reward)
            self.dones.append(done)
            self.actor_inputs.append(self.last_candidate_features if hasattr(self, 'last_candidate_features') else (np.zeros((MAX_CANDIDATES, FEATURE_DIM)), np.array([False]*MAX_CANDIDATES)))
            self.steps += 1
            self.get_logger().info("Episodio terminado por reinicio.")
            self.update_model()
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
            self.reset_triggered = False
            self.reset_confirmation_received = False
            return

        if self.state == "MOVING":
            if distance((self.odom.position.x, self.odom.position.y),
                        (self.current_waypoint.position.x, self.current_waypoint.position.y)) < self.WAYPOINT_THRESHOLD:
                self.get_logger().info("Waypoint alcanzado. Registrando experiencia.")
                reward = self.compute_reward(self.current_waypoint)
                if self.same_candidate_count >= self.same_candidate_threshold:
                    self.get_logger().warn("Se ha seleccionado el mismo candidato demasiadas veces. Penalizando.")
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
                    goal_msg = Bool()
                    goal_msg.data = True
                    self.goal_reached_pub.publish(goal_msg)
                    self.goal_reached_count += 1
                    self.get_logger().info(f"Meta alcanzada {self.goal_reached_count} veces (umbral: {self.goal_reset_threshold}).")
                    if self.goal_reached_count >= self.goal_reset_threshold:
                        self.get_logger().info("Número de veces alcanzada la meta supera umbral. Solicitando reinicio.")
                        self.request_environment_reset()
                        self.goal_reached_count = 0
                        self.goal_reset_threshold = random.randint(2, 5)
                self.current_candidate = None
                self.state = "IDLE"
            else:
                current_time = self.get_clock().now().nanoseconds / 1e9
                if current_time - self.state_start_time > self.TIMEOUT_THRESHOLD:
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
                    self.current_candidate = None
                    self.state = "IDLE"

        if len(self.states) > MEMORY_WINDOW_SIZE:
            self.states.pop(0)
            self.actions.pop(0)
            self.log_probs.pop(0)
            self.values.pop(0)
            self.rewards.pop(0)
            self.dones.pop(0)
            self.actor_inputs.pop(0)

        # Al finalizar el episodio, si no se alcanzó el goal o se llegó al límite de pasos, agregar penalizaciones extra
        if self.steps >= self.max_steps or (self.odom and self.goal and distance((self.odom.position.x, self.odom.position.y),
                                                    (self.goal.position.x, self.goal.position.y)) < GOAL_REACHED_DIST):
            self.get_logger().info(f"Episodio terminado en {self.steps} pasos.")
            current_goal_distance = distance((self.odom.position.x, self.odom.position.y), (self.goal.position.x, self.goal.position.y))
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

    # ----------------------- Cálculo de Ventajas y Actualización del Modelo -----------------------
    def compute_advantages(self, rewards, values, dones):
        advantages = np.zeros_like(rewards, dtype=np.float32)
        last_adv = 0.0
        for t in reversed(range(len(rewards))):
            next_value = values[t+1] if t+1 < len(values) else 0.0
            delta = rewards[t] + GAMMA * next_value * (1 - dones[t]) - values[t]
            advantages[t] = last_adv = delta + GAMMA * LAMBDA * (1 - dones[t]) * last_adv
        returns = advantages + np.array(values, dtype=np.float32)
        return advantages, returns

    def update_model(self):
        if len(self.states) == 0:
            self.get_logger().info("No se recogieron experiencias, omitiendo actualización del modelo.")
            return
        states = np.array(self.states, dtype=np.float32)
        actions = np.array(self.actions, dtype=np.int32)
        log_probs_old = np.array(self.log_probs, dtype=np.float32)
        rewards = np.array(self.rewards, dtype=np.float32)
        dones = np.array(self.dones, dtype=np.float32)
        advantages, returns = self.compute_advantages(rewards, self.values, dones)
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)
        actor_inputs = []
        masks = []
        for cand_feat, m in self.actor_inputs:
            actor_inputs.append(cand_feat)
            masks.append(m)
        actor_inputs = np.array(actor_inputs, dtype=np.float32)
        masks = np.array(masks, dtype=bool)
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
            self.get_logger().info("400 episodios completados. Finalizando nodo.")
            rclpy.shutdown()

    def save_models(self):
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        actor_model_filename = f"actor_model_{timestamp}.keras"
        critic_model_filename = f"critic_model_{timestamp}.keras"
        self.actor.save(actor_model_filename)
        self.critic.save(critic_model_filename)
        self.get_logger().info(f"Modelos guardados: {actor_model_filename} y {critic_model_filename}.")

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
