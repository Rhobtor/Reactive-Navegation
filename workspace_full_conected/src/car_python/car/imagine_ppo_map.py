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

from std_srvs.srv import Empty
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from octomap_msgs.msg import Octomap
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
TRAIN_EPOCHS = 10
BATCH_SIZE = 256

EXPLORATION_DISTANCE_THRESHOLD = 2.5 
EXPLORATION_BONUS_FACTOR = 70.0         
CLEARANCE_BONUS_FACTOR = 60.0           

MEMORY_WINDOW_SIZE = 200  

TIMEOUT_THRESHOLD = 10.0  
WAYPOINT_THRESHOLD = 2.5  

SAME_CANDIDATE_THRESHOLD = 3
SAME_CANDIDATE_RESET_THRESHOLD = 4

RESET_REWARD = -70.0
VIRTUAL_COLLISION_PENALTY = 50.0  

PENALTY_RESET = -50.0      
PENALTY_COLLISION = -50.0  
PENALTY_NO_GOAL = -30.0    
PENALTY_MAX_STEPS = -30.0  

BONUS_WEIGHT = 0.7  # Factor para ponderar el bonus del módulo de imaginación

# ----------------------- Funciones Auxiliares -----------------------
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

def quaternion_to_euler(q: Quaternion):
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

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

# ----------------------- Nodo de Entrenamiento con Fusión y Visualización -----------------------
class NavigationEndToEndTrainer(Node):
    def __init__(self):
        super(NavigationEndToEndTrainer, self).__init__('navigation_end2end_trainer')
        qos_profile = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseArray, '/goal', self.goal_callback, 10)
        self.create_subscription(PoseArray, '/filtered_navigation_nodes', self.filtered_nodes_callback, 10)
        self.create_subscription(PoseArray, '/occupied_rejected_nodes', self.occupied_nodes_callback, 10)
        self.create_subscription(PoseArray, '/obstacle_points', self.obstacle_points_callback, 10)
        # Suscripción al mapa generado (por ejemplo, con Octomap)
        # self.create_subscription(PoseArray, '/navigation_map_points', self.map_points_callback, 10)

        self.marker_pub = self.create_publisher(Marker, '/planned_path_marker', 10)
        self.nav_point = self.create_publisher(PoseArray, '/nav_point', 10)
        # Nuevo: Publicador para los marcadores de candidatos (MarkerArray)
        self.candidate_marker_pub = self.create_publisher(MarkerArray, 'candidate_markers', 10)
        self.reset_request_pub = self.create_publisher(Bool, '/reset_request', qos_profile)
        self.goal_reached_pub = self.create_publisher(Bool, 'goal_reached', 10)
        
        # Variables de sensores y mapa
        self.odom = None
        self.goal = None
        self.filtered_nodes = None
        self.occupied_nodes = None
        self.obstacle_points = None
        # self.map_points = None
        self.virtual_collision = False

        # Variables de entrenamiento y experiencia
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
        self.max_steps = 200

        self.episode_count = 0
        self.total_episodes = 1000
        self.start_time = time.time()
        self.log_dir = "logs/fit/" + datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        self.summary_writer = tf.summary.create_file_writer(self.log_dir)
        self.get_logger().info(f"TensorBoard logs en: {self.log_dir}")

        self.models_saved = False

        # Inicialización de modelos
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
        self.initial_position = None

        self.TIMEOUT_THRESHOLD = TIMEOUT_THRESHOLD
        self.WAYPOINT_THRESHOLD = WAYPOINT_THRESHOLD
        self.current_waypoint = None
        self.last_candidate = None
        self.same_candidate_count = 0
        self.same_candidate_reset_threshold = SAME_CANDIDATE_RESET_THRESHOLD
        self.same_candidate_threshold = SAME_CANDIDATE_THRESHOLD

        # Variables para detección de progreso
        self.progress_buffer = []
        self.progress_window = 10
        self.progress_threshold = 0.1  # Diferencia mínima para considerar progreso
        self.default_map_factor = 1.5
        self.stuck_map_factor = 2.0
        self.dynamic_map_factor = self.default_map_factor

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

        roll, pitch, yaw = quaternion_to_euler(original_pose.orientation)
        threshold = math.pi / 4
        if abs(roll) > threshold or abs(pitch) > threshold:
            self.get_logger().error(f"Robot volcado detectado (roll: {roll:.2f}, pitch: {pitch:.2f}). Solicitando reinicio.")
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

    # def map_points_callback(self, msg: PoseArray):
    #     self.map_points = msg
    #     self.get_logger().info(f"Mapa actualizado: {len(msg.poses)} nodos.")

    def request_environment_reset(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.last_reset_time >= self.reset_cooldown:
            reset_msg = Bool()
            reset_msg.data = True
            self.reset_request_pub.publish(reset_msg)
            self.get_logger().info("Solicitud de reinicio enviada.")
            self.last_reset_time = current_time

    def check_reset_confirmation(self):
        pass

    # ----------------------- Métodos para Publicar Visualización de Candidatos -----------------------
    def publish_candidate_markers(self, candidate_positions, candidate_scores):
        marker_array = MarkerArray()
        for i, (pos, score) in enumerate(zip(candidate_positions, candidate_scores)):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "candidate_markers"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = pos[0]
            marker.pose.position.y = pos[1]
            marker.pose.position.z = 0.0
            # Escalar el marcador según el valor absoluto del score
            scale_factor = 0.1 + 0.2 * abs(score)
            marker.scale.x = scale_factor
            marker.scale.y = scale_factor
            marker.scale.z = scale_factor
            marker.color.a = 1.0
            # Color: verde para score positivo, rojo para negativo
            if score >= 0:
                marker.color.r = 0.0
                marker.color.g = 1.0
            else:
                marker.color.r = 1.0
                marker.color.g = 0.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)
        self.candidate_marker_pub.publish(marker_array)

    def publish_candidate_text_markers(self, candidate_positions, candidate_infos):
        marker_array = MarkerArray()
        for i, (pos, info_text) in enumerate(zip(candidate_positions, candidate_infos)):
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "candidate_text_markers"
            text_marker.id = i + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = pos[0]
            text_marker.pose.position.y = pos[1]
            text_marker.pose.position.z = 0.5  # Se desplaza un poco hacia arriba
            text_marker.scale.z = 0.2
            text_marker.color.a = 1.0
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.text = info_text
            marker_array.markers.append(text_marker)
        self.candidate_marker_pub.publish(marker_array)

    # ----------------------- Fusión de Nodos para la Planificación -----------------------
    def plan_route(self, candidate):
        current_pos = (self.odom.position.x, self.odom.position.y)
        candidate_pos = (candidate.position.x, candidate.position.y)
        nodes = []
        nodes.append((current_pos, 'actual'))
        
        # Contadores para saber cuántos nodos provienen de cada fuente
        filtered_count = 0
        map_count = 0

        # Agregar nodos filtrados (datos inmediatos)
        if self.filtered_nodes is not None and self.filtered_nodes.poses:
            for node in self.filtered_nodes.poses:
                pos = (node.position.x, node.position.y)
                nodes.append((pos, 'filtrado'))
                filtered_count += 1

        # # Agregar nodos del mapa (memoria acumulada)
        # if self.map_points is not None and self.map_points.poses:
        #     for node in self.map_points.poses:
        #         pos = (node.position.x, node.position.y)
        #         nodes.append((pos, 'map'))
        #         map_count += 1

        # Agregar el candidato final
        nodes.append((candidate_pos, 'candidato'))

        # Construir el grafo inicializando todas las claves
        grafo = {i: [] for i in range(len(nodes))}
        for i in range(len(nodes)):
            for j in range(i + 1, len(nodes)):
                d = distance(nodes[i][0], nodes[j][0])
                # Si alguno es del mapa, se aplica la penalización dinámica
                if nodes[i][1] == 'map' or nodes[j][1] == 'map':
                    d *= self.dynamic_map_factor
                grafo[i].append((j, d))
                grafo[j].append((i, d))
        inicio = 0
        objetivo = len(nodes) - 1
        indices_camino = dijkstra(grafo, inicio, objetivo)
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



    # ----------------------- Cálculo de Características para Candidatos -----------------------
    def compute_candidate_features_tf(self):
        if self.filtered_nodes is None or not self.filtered_nodes.poses:
            return None, None, None
        robot_x = tf.constant(self.odom.position.x, dtype=tf.float32)
        robot_y = tf.constant(self.odom.position.y, dtype=tf.float32)
        goal_x = tf.constant(self.goal.position.x, dtype=tf.float32)
        goal_y = tf.constant(self.goal.position.y, dtype=tf.float32)
        candidate_features = []
        valid_nodes = []
        for node in self.filtered_nodes.poses:
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

    # ----------------------- Bucle Reactivo con Evaluación de Progreso y Visualización -----------------------
    def reactive_step(self):
        if self.odom is None or self.goal is None:
            return
        if self.state == "IDLE":
            candidate_features, valid_nodes, mask = self.compute_candidate_features_tf()
            if candidate_features is None:
                self.get_logger().warn("No hay candidatos válidos.")
                return
            self.last_candidate_features = (candidate_features, mask)
            logits, self.actor_state = self.actor(candidate_features, mask=mask, initial_state=self.actor_state)
            bonus = self.imagination_module(candidate_features)
            combined_logits = logits + BONUS_WEIGHT * bonus
            probs = tf.nn.softmax(combined_logits, axis=-1).numpy()[0]
            # Extraer información de candidatos válidos
            valid_count = len(valid_nodes)
            logits_values = logits.numpy()[0][:valid_count]
            bonus_values = bonus.numpy()[0][:valid_count]
            combined_logits_values = combined_logits.numpy()[0][:valid_count]
            probs_values = probs[:valid_count]
            candidate_positions = [(node.position.x, node.position.y) for node in valid_nodes]
            candidate_infos = [f"logit: {l:.2f}, bonus: {b:.2f}, prob: {p:.2f}" 
                                for l, b, p in zip(logits_values, bonus_values, probs_values)]
            # Publicar marcadores y etiquetas en RViz para visualizar los candidatos
            self.publish_candidate_markers(candidate_positions, combined_logits_values)
            self.publish_candidate_text_markers(candidate_positions, candidate_infos)
            # Registrar en TensorBoard
            with self.summary_writer.as_default():
                tf.summary.scalar('max_candidate_logit', float(np.max(logits_values)), step=self.episode_count)
                tf.summary.scalar('max_candidate_bonus', float(np.max(bonus_values)), step=self.episode_count)
                tf.summary.scalar('max_candidate_prob', float(np.max(probs_values)), step=self.episode_count)
            action_index = int(np.argmax(probs))
            if not mask.numpy()[0, action_index]:
                self.get_logger().warn("Candidato seleccionado inválido.")
                return
            new_candidate = valid_nodes[action_index]
            if self.last_candidate is not None:
                d_same = distance((new_candidate.position.x, new_candidate.position.y),
                                  (self.last_candidate.position.x, self.last_candidate.position.y))
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
            self.get_logger().info(f"Candidato seleccionado: ({new_candidate.position.x:.2f}, {new_candidate.position.y:.2f}), repeticiones: {self.same_candidate_count}")
            self.state = "MOVING"
            self.state_start_time = self.get_clock().now().nanoseconds / 1e9
        else:
            if not hasattr(self, 'last_action_index'):
                self.last_action_index = 0
            if not hasattr(self, 'last_probs'):
                self.last_probs = np.zeros(MAX_CANDIDATES)
            if not hasattr(self, 'last_candidate_features'):
                candidate_features, valid_nodes, mask = self.compute_candidate_features_tf()
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

        current_goal_distance = distance((self.odom.position.x, self.odom.position.y),
                                         (self.goal.position.x, self.goal.position.y))
        self.progress_buffer.append(current_goal_distance)
        if len(self.progress_buffer) > self.progress_window:
            self.progress_buffer.pop(0)
            first_half = self.progress_buffer[:self.progress_window // 2]
            second_half = self.progress_buffer[self.progress_window // 2:]
            if (first_half[0] - second_half[-1]) < self.progress_threshold:
                self.dynamic_map_factor = self.stuck_map_factor
                self.get_logger().warn("Progreso insuficiente, aumentando penalización a nodos del mapa.")
            else:
                self.dynamic_map_factor = self.default_map_factor

    # ----------------------- Bucle de Experiencia -----------------------
    def experience_step(self):
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

    def compute_reward(self, planned_waypoint):
        current_pos = np.array([self.odom.position.x, self.odom.position.y])
        goal_pos = np.array([self.goal.position.x, self.goal.position.y])
        d_current = np.linalg.norm(current_pos - goal_pos)
        wp = np.array([planned_waypoint.position.x, planned_waypoint.position.y])
        d_final = np.linalg.norm(wp - goal_pos)
        progress_reward = (d_current - d_final) * 20.0
        safety_penalty = 0.0
        if self.obstacle_points is not None and self.obstacle_points.poses:
            d_list = [distance((planned_waypoint.position.x, planned_waypoint.position.y), (p.position.x, p.position.y))
                      for p in self.obstacle_points.poses]
            if d_list:
                min_wp = min(d_list)
                if min_wp < 1.0:
                    safety_penalty += (1.0 - min_wp) * 50.0
        collision_penalty = 0.0
        if self.occupied_nodes and self.occupied_nodes.poses:
            for occ in self.occupied_nodes.poses:
                d = distance((self.odom.position.x, self.odom.position.y), (occ.position.x, occ.position.y))
                if d < 0.5:
                    collision_penalty += 1000.0
        reward = progress_reward - safety_penalty - collision_penalty + STEP_PENALTY
        if d_current < GOAL_REACHED_DIST:
            reward += GOAL_REWARD
        if self.virtual_collision:
            self.get_logger().warn("Colisión virtual detectada. Penalizando.")
            reward += - (VIRTUAL_COLLISION_PENALTY + PENALTY_COLLISION)
            self.virtual_collision = False
        return reward

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
                    clipped_ratio = tf.clip_by_value(ratio, 1 - CLIP_EPS, 1 + CLIP_EPS)
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
        self.get_logger().info(f"Métricas registradas para el episodio {self.episode_count}.")
        if not getattr(self, 'models_saved', False):
            self.save_models()
            self.models_saved = True
        if self.episode_count >= 1000:
            self.get_logger().info("1000 episodios completados. Finalizando nodo.")
            rclpy.shutdown()

    def save_models(self):
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        actor_model_filename = f"actor_model_{timestamp}.keras"
        critic_model_filename = f"critic_model_{timestamp}.keras"
        imagination_model_filename = f"imagination_model_{timestamp}.keras"
        self.actor.save(actor_model_filename)
        self.critic.save(critic_model_filename)
        self.imagination_module.save(imagination_model_filename)
        self.get_logger().info(f"Modelos guardados: {actor_model_filename}, {critic_model_filename} y {imagination_model_filename}.")

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
