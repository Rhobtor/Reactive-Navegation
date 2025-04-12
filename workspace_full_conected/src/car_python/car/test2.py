# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from rclpy.executors import MultiThreadedExecutor
# import threading
# import math
# import numpy as np
# import tensorflow as tf
# import datetime
# import time
# import heapq
# import random

# from std_srvs.srv import Empty
# from std_msgs.msg import Bool
# from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion, Twist, PoseStamped
# from nav_msgs.msg import Odometry, OccupancyGrid, Path
# from visualization_msgs.msg import Marker, MarkerArray
# from octomap_msgs.msg import Octomap
# from rclpy.10 import 10Profile, DurabilityPolicy

# # ----------------------- Constantes e Hiperparámetros -----------------------
# GOAL_REACHED_DIST = 2.5       
# STEP_PENALTY = -5.0          
# GOAL_REWARD = 50.0            
# OBSTACLE_PENALTY_DIST = 2.7    

# KP_LINEAR = 10.0
# KP_ANGULAR = 10.0
# MAX_LINEAR_SPEED = 2.5
# MAX_ANGULAR_SPEED = 4.0

# TIMEOUT_THRESHOLD = 15.0   
# LOOKAHEAD_DISTANCE = 1.5    # Distancia de lookahead para Pure Pursuit
# PROGRESS_WINDOW = 10
# PROGRESS_THRESHOLD = 0.1

# WAYPOINT_THRESHOLD = 0.5

# GAMMA = 0.99
# LAMBDA = 0.95
# CLIP_EPS = 0.2
# TRAIN_EPOCHS = 30
# BATCH_SIZE = 256
# MEMORY_WINDOW_SIZE = 200
# TOTAL_EPISODES = 1000

# MAX_CANDIDATES = 50            
# FEATURE_DIM = 6               
# GLOBAL_STATE_DIM = 7  

# BONUS_WEIGHT = 0.7  
# STUCK_PENALTY = -50.0   

# # ----------------------- Funciones Auxiliares -----------------------
# def distance(p1, p2):
#     return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

# def dijkstra(grafo, inicio, objetivo):
#     dist = {nodo: float('inf') for nodo in grafo.keys()}
#     prev = {nodo: None for nodo in grafo.keys()}
#     dist[inicio] = 0
#     cola = [(0, inicio)]
#     while cola:
#         costo_actual, actual = heapq.heappop(cola)
#         if actual == objetivo:
#             break
#         if costo_actual > dist[actual]:
#             continue
#         for vecino, peso in grafo.get(actual, []):
#             alt = dist[actual] + peso
#             if alt < dist[vecino]:
#                 dist[vecino] = alt
#                 prev[vecino] = actual
#                 heapq.heappush(cola, (alt, vecino))
#     camino = []
#     nodo = objetivo
#     while nodo is not None:
#         camino.append(nodo)
#         nodo = prev[nodo]
#     camino.reverse()
#     return camino

# def quaternion_to_euler(q: Quaternion):
#     sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
#     cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
#     roll = math.atan2(sinr_cosp, cosr_cosp)
#     sinp = 2.0 * (q.w * q.y - q.z * q.x)
#     pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
#     siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
#     cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
#     yaw = math.atan2(siny_cosp, cosy_cosp)
#     return roll, pitch, yaw

# # ----------------------- Módulo de Imagination -----------------------
# class ImaginationModule(tf.keras.Model):
#     def __init__(self, feature_dim, hidden_units=64, **kwargs):
#         super(ImaginationModule, self).__init__(**kwargs)
#         self.dense1 = tf.keras.layers.Dense(hidden_units, activation='relu')
#         self.dense2 = tf.keras.layers.Dense(hidden_units, activation='relu')
#         self.bonus_layer = tf.keras.layers.Dense(1, activation=None)
    
#     def call(self, candidate_features):
#         x = self.dense1(candidate_features)
#         x = self.dense2(x)
#         bonus = self.bonus_layer(x)
#         bonus = tf.squeeze(bonus, axis=-1)
#         return bonus

# # ----------------------- Actor Recurrente con Atención -----------------------
# class RecurrentActorWithAttention(tf.keras.Model):
#     def __init__(self, max_candidates, feature_dim, lstm_units=256, **kwargs):
#         super(RecurrentActorWithAttention, self).__init__(**kwargs)
#         self.max_candidates = max_candidates
#         self.feature_dim = feature_dim
#         self.td1 = tf.keras.layers.TimeDistributed(tf.keras.layers.Dense(64, activation='relu'))
#         self.td2 = tf.keras.layers.TimeDistributed(tf.keras.layers.Dense(64, activation='relu'))
#         self.td3 = tf.keras.layers.TimeDistributed(tf.keras.layers.Dense(64, activation='relu'))
#         self.lstm = tf.keras.layers.LSTM(lstm_units, return_sequences=True, return_state=True)
#         self.attention_dense = tf.keras.layers.TimeDistributed(tf.keras.layers.Dense(1))
#         self.logits_layer = tf.keras.layers.TimeDistributed(tf.keras.layers.Dense(1))
        
#     def call(self, x, mask=None, initial_state=None):
#         x = self.td1(x)
#         x = self.td2(x)
#         x = self.td3(x)
#         lstm_out, h, c = self.lstm(x, initial_state=initial_state)
#         scores = self.attention_dense(lstm_out)
#         scores = tf.squeeze(scores, axis=-1)
#         if mask is not None:
#             scores = tf.where(mask, scores, -1e9 * tf.ones_like(scores))
#         attention_weights = tf.nn.softmax(scores, axis=1)
#         candidate_logits = self.logits_layer(lstm_out)
#         candidate_logits = tf.squeeze(candidate_logits, axis=-1)
#         combined_logits = candidate_logits * attention_weights
#         return combined_logits, (h, c)

# # ----------------------- Nodo de Entrenamiento y Navegación -----------------------
# class NavigationEndToEndTrainer(Node):
#     def __init__(self):
#         super(NavigationEndToEndTrainer, self).__init__('navigation_end2end_trainer')
#         10_profile = 10Profile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        
#         # Suscriptores
#         self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
#         self.create_subscription(PoseArray, '/goal', self.goal_callback, 10)
#         self.create_subscription(Bool, '/virtual_collision', self.virtual_collision_callback, 10)
#         self.create_subscription(OccupancyGrid, '/occupancy_grid', self.current_grid_callback, 10)
#         self.create_subscription(OccupancyGrid, '/persistent_dynamic_occupancy_grid', self.memory_grid_callback, 10)
        
#         # Publicadores
#         self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.nav_point = self.create_publisher(PoseArray, '/nav_point', 10)
#         self.path_pub = self.create_publisher(Path, '/global_path', 10)
#         self.path_marker_pub = self.create_publisher(Marker, '/global_path_marker', 10)
#         self.reset_request_pub = self.create_publisher(Bool, '/reset_request', 10_profile)
#         self.goal_reached_pub = self.create_publisher(Bool, '/goal_reached', 10)
        
#         # Variable para almacenar el último Twist calculado
#         self.last_twist = Twist()
        
#         # Variables de sensores y estado
#         self.odom = None
#         self.goal = None
#         self.virtual_collision = False
#         self.current_grid_msg = None    
#         self.memory_grid_msg = None     

#         self.last_reset_time = 0.0
#         self.reset_cooldown = 20.0

#         # Estados de navegación
#         self.current_waypoint = None
#         self.state = "IDLE"
#         self.state_start_time = None

#         # Variables para experiencia y entrenamiento
#         self.actor_inputs = []
#         self.states = []
#         self.actions = []
#         self.log_probs = []
#         self.rewards = []
#         self.values = []
#         self.dones = []
#         self.steps = 0
#         self.max_steps = 20

#         self.episode_count = 0
#         self.total_episodes = TOTAL_EPISODES
#         self.start_time = time.time()
#         self.log_dir = "logs/fit/" + datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
#         self.summary_writer = tf.summary.create_file_writer(self.log_dir)
#         self.get_logger().info(f"TensorBoard logs en: {self.log_dir}")
#         self.models_saved = False

#         # Variables para selección de acción
#         self.last_action_index = 0
#         self.last_probs = [1.0 / MAX_CANDIDATES] * MAX_CANDIDATES
#         self.last_candidate_features = (tf.zeros((1, MAX_CANDIDATES, FEATURE_DIM), dtype=tf.float32),
#                                         tf.zeros((1, MAX_CANDIDATES), dtype=tf.bool))
#         self.last_candidate_set = []

#         # Inicialización de modelos
#         self.actor = RecurrentActorWithAttention(MAX_CANDIDATES, FEATURE_DIM)
#         self.actor_state = None
#         self.critic = tf.keras.Sequential([
#             tf.keras.layers.InputLayer(input_shape=(GLOBAL_STATE_DIM,)),
#             tf.keras.layers.Dense(64, activation='relu'),
#             tf.keras.layers.Dense(64, activation='relu'),
#             tf.keras.layers.Dense(1)
#         ])
#         self.actor_optimizer = tf.keras.optimizers.Adam(3e-4)
#         self.critic_optimizer = tf.keras.optimizers.Adam(3e-4)
#         self.imagination_module = ImaginationModule(FEATURE_DIM)

#         # Variables para progreso
#         self.progress_buffer = []
#         self.progress_window = 10
#         self.progress_threshold = 0.1
#         self.default_map_factor = 1.5
#         self.stuck_map_factor = 2.0
#         self.dynamic_map_factor = self.default_map_factor

#         self.get_logger().info("Navigation End-to-End Trainer iniciado. Estado inicial: IDLE")
#         while self.odom is None or self.goal is None:
#             self.get_logger().warn("Esperando /odom y /goal...")
#             rclpy.spin_once(self, timeout_sec=0.1)
#         self.get_logger().info("Datos iniciales recibidos, iniciando entrenamiento.")

#         # Temporizadores
#         self.create_timer(0.1, self.reactive_step)         # Ejecuta reactive_step a 10 Hz
#         self.create_timer(0.5, self.experience_step)
#         self.create_timer(0.1, self.check_reset_confirmation)
        
#         # Inicia un hilo para publicar Twist de forma continua
#         self.twist_publisher_thread = threading.Thread(target=self.publish_twist_continuously, daemon=True)
#         self.twist_publisher_thread.start()

#     # Método que publica Twist continuamente en un hilo
#     def publish_twist_continuously(self):
#         rate = 0.05  # 20 Hz
#         while rclpy.ok():
#             self.cmd_vel_pub.publish(self.last_twist)
#             self.get_logger().info(f"Twist publicado: linear.x = {self.last_twist.linear.x:.2f}, angular.z = {self.last_twist.angular.z:.2f}")
#             time.sleep(rate)

#     # ----------------------- Callbacks -----------------------
#     def odom_callback(self, msg: Odometry):
#         original_pose = msg.pose.pose
#         offset_x = 0.5
#         adjusted_pose = Pose()
#         adjusted_pose.position.x = original_pose.position.x - offset_x
#         adjusted_pose.position.y = original_pose.position.y
#         adjusted_pose.position.z = original_pose.position.z
#         adjusted_pose.orientation = original_pose.orientation
#         self.odom = adjusted_pose
#         self.last_speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
#         roll, pitch, _ = quaternion_to_euler(original_pose.orientation)
#         if abs(roll) > math.pi/4 or abs(pitch) > math.pi/4:
#             self.get_logger().error(f"Robot volcado detectado (roll: {roll:.2f}, pitch: {pitch:.2f}). Solicitando reinicio.")
#             self.request_environment_reset()

#     def goal_callback(self, msg: PoseArray):
#         if msg.poses:
#             self.goal = msg.poses[0]
#             if self.state == "WAIT_FOR_GOAL":
#                 self.get_logger().info("Nuevo goal recibido. Reiniciando navegación.")
#                 self.state = "IDLE"

#     def virtual_collision_callback(self, msg: Bool):
#         if msg.data:
#             self.virtual_collision = True
#             self.get_logger().warn("Se detectó colisión virtual.")

#     def current_grid_callback(self, msg: OccupancyGrid):
#         self.current_grid_msg = msg

#     def memory_grid_callback(self, msg: OccupancyGrid):
#         self.memory_grid_msg = msg

#     def request_environment_reset(self):
#         current_time = self.get_clock().now().nanoseconds / 1e9
#         if current_time - self.last_reset_time >= self.reset_cooldown:
#             reset_msg = Bool()
#             reset_msg.data = True
#             self.reset_request_pub.publish(reset_msg)
#             self.get_logger().info("Solicitud de reinicio enviada.")
#             self.last_reset_time = current_time

#     def check_reset_confirmation(self):
#         pass

#     # ----------------------- Extracción de Nodos desde OccupancyGrid -----------------------
#     def extract_candidate_nodes(self, grid_msg: OccupancyGrid):
#         info = grid_msg.info
#         grid = np.array(grid_msg.data, dtype=np.int8).reshape((info.height, info.width))
#         free_indices = np.argwhere(grid == 0)
#         nodes = []
#         for idx in free_indices:
#             j, i = idx
#             x = info.origin.position.x + (i + 0.5) * info.resolution
#             y = info.origin.position.y + (j + 0.5) * info.resolution
#             nodes.append((x, y))
#         return nodes

#     def build_combined_graph_custom(self):
#         nodes = []
#         if self.current_grid_msg is not None:
#             nodes += self.extract_candidate_nodes(self.current_grid_msg)
#         if self.memory_grid_msg is not None:
#             nodes += self.extract_candidate_nodes(self.memory_grid_msg)
#         return nodes

#     # ----------------------- Plan Global de Trayectoria -----------------------
#     def plan_global_path(self):
#         nodes = self.build_combined_graph_custom()
#         if not nodes:
#             self.get_logger().warn("No se encontraron nodos candidatos en los mapas. Usando path directo.")
#             robot_pos = (self.odom.position.x, self.odom.position.y)
#             goal_pos = (self.goal.position.x, self.goal.position.y)
#             nodes = [robot_pos, goal_pos]
#             graph = {0: [(1, distance(robot_pos, goal_pos))], 1: [(0, distance(robot_pos, goal_pos))]}
#             path_indices = [0, 1]
#             path_points = nodes
#             smooth_path = self.smooth_path(path_points)
#             self.publish_path(smooth_path)
#             self.publish_path_marker(smooth_path)
#             return smooth_path

#         goal_coords = (self.goal.position.x, self.goal.position.y)
#         best_candidate = min(nodes, key=lambda n: distance(n, goal_coords))
#         if distance(best_candidate, goal_coords) > 1.0:
#             fraction = 0.5
#             virtual_candidate = (best_candidate[0] + fraction*(goal_coords[0] - best_candidate[0]),
#                                  best_candidate[1] + fraction*(goal_coords[1] - best_candidate[1]))
#         else:
#             virtual_candidate = best_candidate

#         robot_pos = (self.odom.position.x, self.odom.position.y)
#         nodes.append(robot_pos)
#         nodes.append(virtual_candidate)
#         robot_index = len(nodes) - 2
#         goal_index = len(nodes) - 1
#         connection_threshold = 3.0
#         graph = {i: [] for i in range(len(nodes))}
#         for i in range(len(nodes)):
#             for j in range(i+1, len(nodes)):
#                 d = distance(nodes[i], nodes[j])
#                 if d <= connection_threshold:
#                     graph[i].append((j, d))
#                     graph[j].append((i, d))
#         path_indices = dijkstra(graph, robot_index, goal_index)
#         if not path_indices:
#             self.get_logger().warn("No se pudo planificar un path global.")
#             return None
#         path_points = [nodes[i] for i in path_indices]
#         smooth_path = self.smooth_path(path_points)
#         self.publish_path(smooth_path)
#         self.publish_path_marker(smooth_path)
#         return smooth_path

#     def smooth_path(self, path_points):
#         return path_points

#     def publish_path(self, path_points):
#         path_msg = Path()
#         path_msg.header.stamp = self.get_clock().now().to_msg()
#         path_msg.header.frame_id = "map"
#         for pt in path_points:
#             pose_st = PoseStamped()
#             pose_st.header = path_msg.header
#             pose_st.pose.position.x = pt[0]
#             pose_st.pose.position.y = pt[1]
#             pose_st.pose.position.z = 0.0
#             pose_st.pose.orientation.w = 1.0
#             path_msg.poses.append(pose_st)
#         self.path_pub.publish(path_msg)

#     def publish_path_marker(self, path_points):
#         marker = Marker()
#         marker.header.stamp = self.get_clock().now().to_msg()
#         marker.header.frame_id = "map"
#         marker.ns = "global_path"
#         marker.id = 0
#         marker.type = Marker.LINE_STRIP
#         marker.action = Marker.ADD
#         marker.scale.x = 0.2
#         marker.color.r = 1.0
#         marker.color.g = 0.0
#         marker.color.b = 0.0
#         marker.color.a = 1.0
#         marker.points = []
#         for pt in path_points:
#             p = Point()
#             p.x = pt[0]
#             p.y = pt[1]
#             p.z = 0.0
#             marker.points.append(p)
#         self.path_marker_pub.publish(marker)

#     # ----------------------- Extracción de Candidatos desde OccupancyGrid -----------------------
#     def compute_candidates_from_grid(self, grid_msg: OccupancyGrid):
#         info = grid_msg.info
#         grid = np.array(grid_msg.data, dtype=np.int8).reshape((info.height, info.width))
#         free_indices = np.argwhere(grid == 0)
#         candidates = []
#         for idx in free_indices:
#             j, i = idx
#             cell_x = info.origin.position.x + (i + 0.5) * info.resolution
#             cell_y = info.origin.position.y + (j + 0.5) * info.resolution
#             candidate_pose = Pose()
#             candidate_pose.position.x = cell_x
#             candidate_pose.position.y = cell_y
#             candidate_pose.position.z = 0.0
#             candidate_pose.orientation.w = 1.0
#             dist_robot = math.hypot(cell_x - self.odom.position.x, cell_y - self.odom.position.y)
#             dist_to_goal = math.hypot(cell_x - self.goal.position.x, cell_y - self.goal.position.y)
#             angle_robot = math.atan2(cell_y - self.odom.position.y, cell_x - self.odom.position.x)
#             angle_goal = math.atan2(self.goal.position.y - self.odom.position.y, self.goal.position.x - self.odom.position.x)
#             angular_error = abs(math.atan2(math.sin(angle_robot - angle_goal), math.cos(angle_robot - angle_goal)))
#             clearance = 10.0
#             feature_vector = tf.constant([dist_robot, angular_error, clearance, dist_to_goal, 0.0, 0.0],
#                                           dtype=tf.float32)
#             candidates.append((feature_vector, candidate_pose))
#         return candidates

#     def compute_candidate_features_tf(self):
#         candidates = []
#         if self.current_grid_msg is not None:
#             candidates += self.compute_candidates_from_grid(self.current_grid_msg)
#         if self.memory_grid_msg is not None:
#             candidates += self.compute_candidates_from_grid(self.memory_grid_msg)
#         if not candidates:
#             self.get_logger().warn("No se encontraron candidatos en ninguno de los mapas.")
#             return None, None, None
#         candidates.sort(key=lambda tup: tup[0][3].numpy())
#         if len(candidates) > MAX_CANDIDATES:
#             candidates = candidates[:MAX_CANDIDATES]
#         candidate_features = [tup[0] for tup in candidates]
#         valid_nodes = [tup[1] for tup in candidates]
#         num_valid = len(candidate_features)
#         while len(candidate_features) < MAX_CANDIDATES:
#             candidate_features.append(tf.zeros([FEATURE_DIM], dtype=tf.float32))
#         features_tensor = tf.stack(candidate_features)
#         mask = tf.concat([tf.ones([num_valid], dtype=tf.bool),
#                           tf.zeros([MAX_CANDIDATES - num_valid], dtype=tf.bool)], axis=0)
#         features_tensor = tf.expand_dims(features_tensor, axis=0)
#         mask = tf.expand_dims(mask, axis=0)
#         self.last_candidate_set = valid_nodes
#         self.last_candidate_features = (features_tensor, mask)
#         return features_tensor, valid_nodes, mask

#     # ----------------------- Recuperación en Caso de Atascamiento -----------------------
#     def recover_from_stuck(self):
#         if self.memory_grid_msg is None:
#             self.get_logger().warn("No hay mapa de memoria para recuperación.")
#             return None
#         candidates = self.compute_candidates_from_grid(self.memory_grid_msg)
#         if not candidates:
#             self.get_logger().warn("No se encontraron candidatos en el mapa de memoria para recuperación.")
#             return None
#         candidates.sort(key=lambda tup: tup[0][3].numpy())
#         self.get_logger().info("Recuperación: candidato seleccionado desde el mapa de memoria.")
#         return candidates[0][1]

#     # ----------------------- Selección de Waypoint -----------------------
#     def select_waypoint(self):
#         features, valid_nodes, mask = self.compute_candidate_features_tf()
#         if features is None:
#             return self.recover_from_stuck()
#         logits, self.actor_state = self.actor(features, mask=mask, initial_state=self.actor_state)
#         bonus = self.imagination_module(features)
#         combined_logits = logits + BONUS_WEIGHT * bonus
#         probs = tf.nn.softmax(combined_logits, axis=-1).numpy()[0]
#         valid_count = len(valid_nodes)
#         action_index = int(np.argmax(probs[:valid_count]))
#         if action_index >= valid_count:
#             self.get_logger().warn("Acción inválida de la red; se usa recuperación.")
#             selected_node = self.recover_from_stuck()
#         else:
#             selected_node = valid_nodes[action_index]
#             self.last_action_index = action_index
#             self.last_probs = probs
#         if selected_node is not None:
#             self.last_selected_point = selected_node
#         return selected_node

#     # ----------------------- Planificación de Ruta -----------------------
#     def plan_route(self, candidate: Pose):
#         smooth_path = self.plan_global_path()
#         if smooth_path is None or len(smooth_path) < 2:
#             return candidate, smooth_path
#         robot_pos = (self.odom.position.x, self.odom.position.y)
#         waypoint = None
#         for pt in smooth_path:
#             if distance(robot_pos, pt) > 1.0:
#                 waypoint = pt
#                 break
#         if waypoint is None:
#             waypoint = smooth_path[-1]
#         new_pose = Pose()
#         new_pose.position.x = waypoint[0]
#         new_pose.position.y = waypoint[1]
#         new_pose.position.z = 0.0
#         new_pose.orientation.w = 1.0
#         return new_pose, smooth_path

#     # ----------------------- Control Pure Pursuit -----------------------
#     def pure_pursuit_control(self, path, current_pos, current_yaw, lookahead_distance):
#         # Busca el primer punto del path que esté a una distancia >= lookahead_distance
#         target_point = None
#         for pt in path:
#             if distance(current_pos, pt) >= lookahead_distance:
#                 target_point = pt
#                 break
#         if target_point is None:
#             target_point = path[-1]
#         # Calcular ángulo hacia el target
#         angle_to_target = math.atan2(target_point[1] - current_pos[1],
#                                      target_point[0] - current_pos[0])
#         angle_error = angle_to_target - current_yaw
#         # Normalizar el ángulo
#         angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
#         # Calcular la curvatura
#         curvature = 2 * math.sin(angle_error) / lookahead_distance
#         # Definir velocidad lineal fija (se puede ajustar o hacer dependiente de la distancia)
#         linear_speed = MAX_LINEAR_SPEED
#         # Velocidad angular en función de la curvatura
#         angular_speed = curvature * linear_speed
#         return linear_speed, angular_speed, target_point

#     # ----------------------- Bucle Reactivo con Control Fluido -----------------------
#     def reactive_step(self):
#         if self.odom is None or self.goal is None:
#             return

#         current_time = self.get_clock().now().nanoseconds / 1e9
#         if self.state_start_time is None:
#             self.state_start_time = current_time

#         # Obtiene el path global
#         smooth_path = self.plan_global_path()
#         if smooth_path is None or len(smooth_path) < 2:
#             self.get_logger().warn("No se pudo planificar un path global. Enviando Twist cero.")
#             self.last_twist = Twist()
#             return

#         current_pos = (self.odom.position.x, self.odom.position.y)
#         _, _, current_yaw = quaternion_to_euler(self.odom.orientation)

#         # Usamos Pure Pursuit para calcular el control
#         linear_speed, angular_speed, target_point = self.pure_pursuit_control(
#             smooth_path, current_pos, current_yaw, LOOKAHEAD_DISTANCE)

#         self.get_logger().info("Estado cambiado a MOVING")
#         self.get_logger().info(f"Target lookahead: ({target_point[0]:.2f}, {target_point[1]:.2f})")
#         self.get_logger().info(f"Velocidades calculadas: lineal = {linear_speed:.2f} m/s, angular = {angular_speed:.2f} rad/s")

#         twist_msg = Twist()
#         twist_msg.linear.x = linear_speed
#         twist_msg.angular.z = angular_speed
#         self.last_twist = twist_msg  # Se actualiza para que el hilo de publicación lo envíe continuamente

#         # Publica el waypoint de seguimiento para visualización
#         waypoint_pose = Pose()
#         waypoint_pose.position.x = target_point[0]
#         waypoint_pose.position.y = target_point[1]
#         waypoint_pose.position.z = 0.0
#         waypoint_pose.orientation.w = 1.0
#         self.current_waypoint = waypoint_pose
#         nav_points = PoseArray()
#         nav_points.header.stamp = self.get_clock().now().to_msg()
#         nav_points.header.frame_id = "map"
#         nav_points.poses.append(waypoint_pose)
#         self.nav_point.publish(nav_points)

#         # Registro de experiencia si se alcanza el waypoint (umbral)
#         if distance(current_pos, target_point) < WAYPOINT_THRESHOLD:
#             self.get_logger().info("Waypoint alcanzado. Registrando experiencia.")
#             reward = self.compute_reward(self.current_waypoint)
#             global_state = np.array([
#                 self.odom.position.x,
#                 self.odom.position.y,
#                 self.goal.position.x,
#                 self.goal.position.y,
#                 0.0, 0.0, 0.0
#             ], dtype=np.float32)
#             value = self.critic(tf.convert_to_tensor([global_state], dtype=tf.float32)).numpy()[0]
#             done = 1 if distance(current_pos, (self.goal.position.x, self.goal.position.y)) < GOAL_REACHED_DIST else 0
#             self.states.append(global_state)
#             self.actions.append(self.last_action_index)
#             self.log_probs.append(np.log(self.last_probs[self.last_action_index] + 1e-8))
#             self.values.append(value)
#             self.rewards.append(reward)
#             self.dones.append(done)
#             self.actor_inputs.append(self.last_candidate_features)
#             self.steps += 1
#             self.state_start_time = current_time
#             if done:
#                 self.get_logger().info("Meta alcanzada. Deteniendo y reiniciando.")
#                 self.last_twist = Twist()
#                 goal_msg = Bool()
#                 goal_msg.data = True
#                 self.goal_reached_pub.publish(goal_msg)
#                 self.request_environment_reset()
#                 self.state = "WAIT_FOR_GOAL"
#                 return

#         else:
#             if current_time - self.state_start_time > TIMEOUT_THRESHOLD:
#                 self.get_logger().warn("Timeout en MOVING. Iniciando recuperación.")
#                 recovered = self.recover_from_stuck()
#                 if recovered is not None:
#                     new_pose = Pose()
#                     new_pose.position.x = recovered.position.x
#                     new_pose.position.y = recovered.position.y
#                     new_pose.position.z = recovered.position.z
#                     new_pose.orientation = recovered.orientation
#                     self.current_waypoint = new_pose
#                     self.state = "MOVING"
#                     self.state_start_time = current_time
#                     self.get_logger().info("Recuperación aplicada; nuevo waypoint asignado.")
#                     return
#                 else:
#                     self.get_logger().warn("No se pudo recuperar; reiniciando.")
#                     self.request_environment_reset()
#                     self.last_twist = Twist()
#                     self.state = "WAIT_FOR_GOAL"
#                     return

#             if self.virtual_collision:
#                 self.get_logger().warn("Colisión virtual. Reiniciando.")
#                 self.record_stuck_penalty()
#                 self.request_environment_reset()
#                 self.last_twist = Twist()
#                 self.virtual_collision = False
#                 self.state = "WAIT_FOR_GOAL"
#                 return

#             current_goal_distance = distance(current_pos, (self.goal.position.x, self.goal.position.y))
#             self.progress_buffer.append(current_goal_distance)
#             if len(self.progress_buffer) > self.progress_window:
#                 self.progress_buffer.pop(0)
#                 first_half = self.progress_buffer[:self.progress_window // 2]
#                 second_half = self.progress_buffer[self.progress_window // 2:]
#                 if (first_half[0] - second_half[-1]) < self.progress_threshold:
#                     self.dynamic_map_factor = self.stuck_map_factor
#                     self.get_logger().warn("Progreso insuficiente, penalizando.")
#                 else:
#                     self.dynamic_map_factor = self.default_map_factor

#     def record_stuck_penalty(self):
#         penalty_reward = STUCK_PENALTY
#         global_state = np.array([
#             self.odom.position.x,
#             self.odom.position.y,
#             self.goal.position.x,
#             self.goal.position.y,
#             0.0, 0.0, 0.0
#         ], dtype=np.float32)
#         value = self.critic(tf.convert_to_tensor([global_state], dtype=tf.float32)).numpy()[0]
#         done = 0
#         self.states.append(global_state)
#         self.actions.append(self.last_action_index)
#         self.log_probs.append(np.log(self.last_probs[self.last_action_index] + 1e-8))
#         self.values.append(value)
#         self.rewards.append(penalty_reward)
#         self.dones.append(done)
#         self.actor_inputs.append(self.last_candidate_features)
#         self.steps += 1

#     def experience_step(self):
#         if self.steps >= self.max_steps or (self.odom and self.goal and 
#             distance((self.odom.position.x, self.odom.position.y),
#                      (self.goal.position.x, self.goal.position.y)) < GOAL_REACHED_DIST):
#             self.get_logger().info(f"Episodio terminado en {self.steps} pasos.")
#             if self.steps >= self.max_steps:
#                 self.get_logger().info("Número máximo de pasos alcanzado.")
#             current_goal_distance = distance((self.odom.position.x, self.odom.position.y),
#                                              (self.goal.position.x, self.goal.position.y))
#             if current_goal_distance >= GOAL_REACHED_DIST and len(self.rewards) > 0:
#                 self.rewards[-1] += -30.0
#             if self.steps >= self.max_steps and len(self.rewards) > 0:
#                 self.rewards[-1] += -30.0
#             self.update_model()
#             elapsed = time.time() - self.start_time
#             avg_ep_time = elapsed / (self.episode_count + 1)
#             remaining = (self.total_episodes - (self.episode_count + 1)) * avg_ep_time
#             self.get_logger().info(f"Tiempo transcurrido: {elapsed:.1f}s, restante: {remaining:.1f}s")
#             self.states = []
#             self.actions = []
#             self.log_probs = []
#             self.values = []
#             self.rewards = []
#             self.dones = []
#             self.actor_inputs = []
#             self.steps = 0
#             self.episode_count += 1
#             self.actor_state = None
#             self.state = "IDLE"
#             self.get_logger().info(f"Episodios: {self.episode_count}/{self.total_episodes}")
        
#         if len(self.states) > MEMORY_WINDOW_SIZE:
#             self.states.pop(0)
#             self.actions.pop(0)
#             self.log_probs.pop(0)
#             self.values.pop(0)
#             self.rewards.pop(0)
#             self.dones.pop(0)
#             self.actor_inputs.pop(0)

#     def compute_reward(self, planned_waypoint):
#         current_pos = np.array([self.odom.position.x, self.odom.position.y])
#         goal_pos = np.array([self.goal.position.x, self.goal.position.y])
#         d_current = np.linalg.norm(current_pos - goal_pos)
#         wp = np.array([planned_waypoint.position.x, planned_waypoint.position.y])
#         d_final = np.linalg.norm(wp - goal_pos)
#         progress_reward = (d_current - d_final) * 20.0
#         reward = progress_reward + STEP_PENALTY
#         if d_current < GOAL_REACHED_DIST:
#             reward += GOAL_REWARD
#         if self.virtual_collision:
#             self.get_logger().warn("Colisión virtual detectada. Penalizando.")
#             reward += -50.0
#             self.virtual_collision = False
#         return reward

#     def compute_advantages(self, rewards, values, dones):
#         advantages = np.zeros_like(rewards, dtype=np.float32)
#         last_adv = 0.0
#         for t in reversed(range(len(rewards))):
#             next_value = values[t+1] if t+1 < len(values) else 0.0
#             delta = rewards[t] + GAMMA * next_value * (1 - dones[t]) - values[t]
#             advantages[t] = last_adv = delta + GAMMA * LAMBDA * (1 - dones[t]) * last_adv
#         returns = advantages + np.array(values, dtype=np.float32)
#         return advantages, returns

#     def update_model(self):
#         if len(self.states) == 0:
#             self.get_logger().info("No se recogieron experiencias, omitiendo actualización del modelo.")
#             return
#         states = np.array(self.states, dtype=np.float32)
#         actions = np.array(self.actions, dtype=np.int32)
#         log_probs_old = np.array(self.log_probs, dtype=np.float32)
#         rewards = np.array(self.rewards, dtype=np.float32)
#         dones = np.array(self.dones, dtype=np.float32)
#         advantages, returns = self.compute_advantages(rewards, self.values, dones)
#         advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)
#         actor_inputs = []
#         masks = []
#         for cand_feat, m in self.actor_inputs:
#             actor_inputs.append(cand_feat)
#             masks.append(m)
#         actor_inputs = np.array(actor_inputs, dtype=np.float32)
#         masks = np.array(masks, dtype=bool)
#         actor_inputs = tf.convert_to_tensor(actor_inputs)
#         masks = tf.convert_to_tensor(masks)
#         actor_inputs = tf.squeeze(actor_inputs, axis=1)
#         masks = tf.squeeze(masks, axis=1)
#         N = len(states)
#         dataset = tf.data.Dataset.from_tensor_slices((states, actor_inputs, masks, actions, log_probs_old, returns, advantages))
#         dataset = dataset.shuffle(N).batch(BATCH_SIZE)
#         total_actor_loss = 0.0
#         total_critic_loss = 0.0
#         total_loss_value = 0.0
#         batch_count = 0
#         for epoch in range(TRAIN_EPOCHS):
#             for batch in dataset:
#                 batch_states, batch_actor_inputs, batch_masks, batch_actions, batch_old_log_probs, batch_returns, batch_advantages = batch
#                 with tf.GradientTape(persistent=True) as tape:
#                     logits, _ = self.actor(batch_actor_inputs, mask=tf.convert_to_tensor(batch_masks), initial_state=None)
#                     bonus = self.imagination_module(batch_actor_inputs)
#                     combined_logits = logits + BONUS_WEIGHT * bonus
#                     batch_actions = tf.cast(batch_actions, tf.int32)
#                     indices = tf.stack([tf.range(tf.shape(combined_logits)[0]), batch_actions], axis=1)
#                     new_log_probs = tf.math.log(tf.nn.softmax(combined_logits, axis=1) + 1e-8)
#                     new_log_probs = tf.gather_nd(new_log_probs, indices)
#                     ratio = tf.exp(new_log_probs - batch_old_log_probs)
#                     clipped_ratio = tf.clip_by_value(ratio, 1 - CLIP_EPS, 1 + CLIP_EPS)
#                     actor_loss = -tf.reduce_mean(tf.minimum(ratio * batch_advantages, clipped_ratio * batch_advantages))
#                     values_pred = self.critic(batch_states)
#                     critic_loss = tf.reduce_mean(tf.square(batch_returns - values_pred))
#                     total_loss = actor_loss + 0.5 * critic_loss
#                 actor_imagination_vars = self.actor.trainable_variables + self.imagination_module.trainable_variables
#                 actor_imagination_grads = tape.gradient(total_loss, actor_imagination_vars)
#                 self.actor_optimizer.apply_gradients(zip(actor_imagination_grads, actor_imagination_vars))
#                 critic_grads = tape.gradient(total_loss, self.critic.trainable_variables)
#                 self.critic_optimizer.apply_gradients(zip(critic_grads, self.critic.trainable_variables))
#                 total_actor_loss += actor_loss.numpy()
#                 total_critic_loss += critic_loss.numpy()
#                 total_loss_value += total_loss.numpy()
#                 batch_count += 1
#         avg_actor_loss = total_actor_loss / batch_count if batch_count > 0 else 0.0
#         avg_critic_loss = total_critic_loss / batch_count if batch_count > 0 else 0.0
#         avg_total_loss = total_loss_value / batch_count if batch_count > 0 else 0.0
#         self.get_logger().info("Modelo PPO actualizado.")
#         episode_reward = np.sum(self.rewards)
#         episode_length = len(self.rewards)
#         with self.summary_writer.as_default():
#             tf.summary.scalar('actor_loss', avg_actor_loss, step=self.episode_count)
#             tf.summary.scalar('critic_loss', avg_critic_loss, step=self.episode_count)
#             tf.summary.scalar('total_loss', avg_total_loss, step=self.episode_count)
#             tf.summary.scalar('episode_reward', episode_reward, step=self.episode_count)
#             tf.summary.scalar('episode_length', episode_length, step=self.episode_count)
#         self.get_logger().info(f"Métricas registradas para el episodio {self.episode_count}.")
#         if not getattr(self, 'models_saved', False):
#             self.save_models()
#             self.models_saved = True
#         if self.episode_count >= self.total_episodes:
#             self.get_logger().info("1000 episodios completados. Finalizando nodo.")
#             rclpy.shutdown()

#     def save_models(self):
#         timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
#         actor_model_filename = f"actor_model_{timestamp}.keras"
#         critic_model_filename = f"critic_model_{timestamp}.keras"
#         imagination_model_filename = f"imagination_model_{timestamp}.keras"
#         self.actor.save(actor_model_filename)
#         self.critic.save(critic_model_filename)
#         self.imagination_module.save(imagination_model_filename)
#         self.get_logger().info(f"Modelos guardados: {actor_model_filename}, {critic_model_filename} y {imagination_model_filename}.")

# def main(args=None):
#     rclpy.init(args=args)
#     executor = MultiThreadedExecutor(num_threads=4)
#     node = NavigationEndToEndTrainer()
#     executor.add_node(node)
#     try:
#         executor.spin()
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import math
import numpy as np
import heapq
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, Twist, Point
from nav_msgs.msg import MapMetaData
import threading
import time


# Función auxiliar para calcular distancia Euclidiana
def distance(p1, p2):
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

# Algoritmo de Dijkstra para planificar el camino en un grafo
def dijkstra(graph, start, goal):
    dist = {node: float('inf') for node in graph.keys()}
    prev = {node: None for node in graph.keys()}
    dist[start] = 0
    queue = [(0, start)]
    while queue:
        current_dist, current_node = heapq.heappop(queue)
        if current_node == goal:
            break
        if current_dist > dist[current_node]:
            continue
        for neighbor, weight in graph[current_node]:
            alt = current_dist + weight
            if alt < dist[neighbor]:
                dist[neighbor] = alt
                prev[neighbor] = current_node
                heapq.heappush(queue, (alt, neighbor))
    # Reconstruir el camino
    path = []
    node = goal
    while node is not None:
        path.append(node)
        node = prev[node]
    path.reverse()
    return path

# Función para convertir índices del mapa a coordenadas en el mundo
def index_to_world(i, j, info):
    x = info.origin.position.x + (i + 0.5) * info.resolution
    y = info.origin.position.y + (j + 0.5) * info.resolution
    return (x, y)

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        
        
        # Suscriptores
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseArray, '/goal', self.goal_callback, 10)
        self.create_subscription(OccupancyGrid, '/occupancy_grid', self.current_grid_callback, 10)
        self.create_subscription(OccupancyGrid, '/persistent_dynamic_occupancy_grid', self.memory_grid_callback, 10)
        
        # Publicadores
        self.path_pub = self.create_publisher(Path, '/global_path', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Parámetros para la fusión y planificación
        self.connection_threshold = 3.0   # Umbral para conectar nodos
        self.lookahead_distance = 6.0       # Para el controlador Pure Pursuit
        self.linear_speed = 5.0           # Velocidad lineal fija
        self.k_pursuit = 2.0              # Ganancia angular
        self.last_valid_path = None  # Almacena el último camino aceptado
        self.path_update_threshold = 0.4  # Umbral para actualizar el camino (por ejemplo, 30% de diferencia)
        self.last_valid_path = None
        self.last_replan_time = 0.0
        self.replan_interval = 3.0  # segundos
        self.path_change_threshold = 2.3  # Umbral mínimo para actualizar el camino

        # Variables de estado
        self.odom = None
        self.goal = None
        self.current_grid = None
        self.memory_grid = None

        # Variable para el path global persistente
        self.current_global_path = None  
        # Buffer para registrar datos para entrenamiento
        self.training_buffer = []  
        self.current_path_index = 0  # Índice del siguiente waypoint
        # Parámetros para actualizar el path
        self.waypoint_threshold = 0.5  # Umbral para considerar que se ha alcanzado un waypoint

        
        # Timer para actualizar el path y generar el control (10 Hz)
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Nodo de navegación iniciado.")


        self.last_twist = Twist()
        self.twist_pub_rate = 20  # Publicar a 20 Hz
        self.start_twist_publisher()

    def start_twist_publisher(self):
        # Inicia un hilo que publica continuamente el último comando Twist
        thread = threading.Thread(target=self.publish_twist_continuously, daemon=True)
        thread.start()

    def publish_twist_continuously(self):
        rate = 1.0 / self.twist_pub_rate
        while rclpy.ok():
            self.cmd_vel_pub.publish(self.last_twist)
            time.sleep(rate)


    def odom_callback(self, msg: Odometry):
        self.odom = msg.pose.pose

    def goal_callback(self, msg: PoseArray):
        if msg.poses:
            self.goal = msg.poses[0]
            self.get_logger().info("Goal recibido.")

    def current_grid_callback(self, msg: OccupancyGrid):
        self.current_grid = msg

    def memory_grid_callback(self, msg: OccupancyGrid):
        self.memory_grid = msg


    def compute_dynamic_twist(self, current_pos, current_yaw, new_path, fused_grid, info):
        # Calcular velocidades base usando Pure Pursuit
        linear_speed_base, angular_speed = self.pure_pursuit_control(new_path, current_pos, current_yaw)
        
        # Ejemplo: Calcula el error angular con el siguiente punto del path
        target_point = new_path[min(1, len(new_path)-1)]
        desired_heading = math.atan2(target_point[1] - current_pos[1],
                                     target_point[0] - current_pos[0])
        error_angle = math.atan2(math.sin(desired_heading - current_yaw),
                                 math.cos(desired_heading - current_yaw))
        
        # Ajuste de velocidad: cuanto mayor sea el error angular, menor la velocidad lineal
        reduction_factor = max(0.1, 1 - abs(error_angle))
        linear_speed = linear_speed_base * reduction_factor

        # Comprobar obstáculos locales para determinar si retroceder
        safe_distance = 0.5  # Distancia mínima segura (ejemplo)
        obstacle_detected, obs_distance = self.check_local_obstacles(fused_grid, info, current_pos, current_yaw)
        if obstacle_detected and obs_distance < safe_distance:
            self.get_logger().warn(f"Obstáculo muy cerca ({obs_distance:.2f} m), activando retroceso.")
            linear_speed = -abs(linear_speed_base)
        
        return linear_speed, angular_speed

    # def fuse_maps(self):
    #     """
    #     Fusiona dos OccupancyGrids: si en alguna de las celdas hay obstáculo (valor 100),
    #     la celda se marca como obstáculo; si ambas son libres (0), se marca como libre; si
    #     no se conocen, se deja en -1.
    #     """
    #     if self.current_grid is None and self.memory_grid is None:
    #         return None
    #     # Usamos la información del grid actual como base
    #     info = self.current_grid.info if self.current_grid is not None else self.memory_grid.info
    #     # Convertir datos a arrays numpy
    #     grid1 = np.array(self.current_grid.data, dtype=np.int8) if self.current_grid is not None else -1 * np.ones(info.width * info.height, dtype=np.int8)
    #     grid2 = np.array(self.memory_grid.data, dtype=np.int8) if self.memory_grid is not None else -1 * np.ones(info.width * info.height, dtype=np.int8)
    #     # Fusion: si alguno es 100, la celda es 100; si ambos son 0, la celda es 0; en otro caso, -1
    #     fused = np.where((grid1 == 100) | (grid2 == 100), 100,
    #                      np.where((grid1 == 0) & (grid2 == 0), 0, -1))
    #     fused_grid = fused.reshape((info.height, info.width))
    #     return fused_grid, info


    def fuse_maps_dynamic(self,grid_msg1, grid_msg2):
        """
        Fusiona dos mensajes OccupancyGrid, que pueden tener distintos tamaños y orígenes.
        Devuelve un array 2D fused_grid y la información (MapMetaData) asociada.
        
        La fusión se realiza de forma que:
        - Si alguna celda es obstáculo (100) en alguno de los mapas, se marca 100.
        - Si ambas celdas son libres (0), se marca 0.
        - En otro caso, se marca como desconocido (-1).
        """
        # Usamos la información del primer mapa si está disponible; de lo contrario, el segundo.
        info1 = grid_msg1.info if grid_msg1 is not None else None
        info2 = grid_msg2.info if grid_msg2 is not None else None
        if info1 is None and info2 is None:
            return None, None

        # Obtener límites en coordenadas mundo para cada mapa
        def get_bounds(info):
            ox = info.origin.position.x
            oy = info.origin.position.y
            max_x = ox + info.width * info.resolution
            max_y = oy + info.height * info.resolution
            return ox, oy, max_x, max_y

        bounds = []
        if info1 is not None:
            bounds.append(get_bounds(info1))
        if info2 is not None:
            bounds.append(get_bounds(info2))
        
        # Calcular límites globales
        min_x = min(b[0] for b in bounds)
        min_y = min(b[1] for b in bounds)
        max_x = max(b[2] for b in bounds)
        max_y = max(b[3] for b in bounds)

        # Añadir margen extra (por ejemplo, 5 m)
        margin = 5.0
        min_x -= margin
        min_y -= margin
        max_x += margin
        max_y += margin

        # Calcular dimensiones del nuevo grid
        resolution = info1.resolution if info1 is not None else info2.resolution
        new_width = int(math.ceil((max_x - min_x) / resolution))
        new_height = int(math.ceil((max_y - min_y) / resolution))

        # Crear el grid unificado, inicializado con -1 (desconocido)
        fused_grid = -1 * np.ones((new_height, new_width), dtype=np.int8)

        # Función para reproyectar un mapa en el grid unificado
        def reproject_map(grid_msg, fused_grid, new_origin_x, new_origin_y, resolution):
            info = grid_msg.info
            # Convertir los datos del grid a un array 2D
            grid_array = np.array(grid_msg.data, dtype=np.int8).reshape((info.height, info.width))
            # Para cada celda en el grid original, calcular su posición en el nuevo grid
            for j in range(info.height):
                for i in range(info.width):
                    # Coordenadas en el mundo del centro de la celda
                    x = info.origin.position.x + (i + 0.5) * resolution
                    y = info.origin.position.y + (j + 0.5) * resolution
                    # Indices en el grid unificado
                    new_i = int((x - new_origin_x) / resolution)
                    new_j = int((y - new_origin_y) / resolution)
                    if 0 <= new_i < fused_grid.shape[1] and 0 <= new_j < fused_grid.shape[0]:
                        value = grid_array[j, i]
                        # Actualizar la celda: si ya tiene datos, aplicamos reglas de fusión
                        if value == 100:
                            fused_grid[new_j, new_i] = 100
                        elif value == 0 and fused_grid[new_j, new_i] != 100:
                            fused_grid[new_j, new_i] = 0
            return fused_grid

        # Reproyectar cada mapa en el grid unificado
        if grid_msg1 is not None:
            fused_grid = reproject_map(grid_msg1, fused_grid, min_x, min_y, resolution)
        if grid_msg2 is not None:
            fused_grid = reproject_map(grid_msg2, fused_grid, min_x, min_y, resolution)

        # Crear un MapMetaData para el grid fusionado
        new_info = MapMetaData()
        new_info.resolution = resolution
        new_info.width = new_width
        new_info.height = new_height
        new_info.origin.position.x = min_x
        new_info.origin.position.y = min_y
        new_info.origin.position.z = 0.0
        new_info.origin.orientation.w = 1.0

        return fused_grid, new_info


    def extract_nodes(self, fused_grid, info):
        """Extrae nodos candidatos (celdas libres) del mapa fusionado."""
        nodes = []
        for j in range(fused_grid.shape[0]):
            for i in range(fused_grid.shape[1]):
                if fused_grid[j, i] == 0:
                    nodes.append(index_to_world(i, j, info))
        return nodes

    # def plan_path(self, nodes):
    #     """
    #     Construye un grafo a partir de los nodos y añade la posición actual y el goal.
    #     Luego utiliza Dijkstra para obtener el camino.
    #     """
    #     if self.odom is None or self.goal is None:
    #         return None
    #     # Posición actual y goal
    #     robot_pos = (self.odom.position.x, self.odom.position.y)
    #     goal_pos = (self.goal.position.x, self.goal.position.y)
    #     # Añadimos robot y goal al conjunto de nodos
    #     all_nodes = nodes.copy()
    #     all_nodes.append(robot_pos)
    #     all_nodes.append(goal_pos)
    #     robot_index = len(all_nodes) - 2
    #     goal_index = len(all_nodes) - 1
        
    #     # Construir grafo: conectar nodos si están a menos de connection_threshold metros
    #     graph = {i: [] for i in range(len(all_nodes))}
    #     for i in range(len(all_nodes)):
    #         for j in range(i+1, len(all_nodes)):
    #             d = distance(all_nodes[i], all_nodes[j])
    #             if d <= self.connection_threshold:
    #                 graph[i].append((j, d))
    #                 graph[j].append((i, d))
    #     path_indices = dijkstra(graph, robot_index, goal_index)
    #     if not path_indices or len(path_indices) < 2:
    #         self.get_logger().warn("No se pudo planificar un path global, usando línea directa.")
    #         return [robot_pos, goal_pos]
    #     path = [all_nodes[i] for i in path_indices]
    #     return path
    def project_goal(self, goal_pos, info):
        # Limites del mapa
        ox = info.origin.position.x
        oy = info.origin.position.y
        max_x = ox + info.width * info.resolution
        max_y = oy + info.height * info.resolution
        # Proyectar goal dentro de los límites
        x, y = goal_pos
        x = min(max(x, ox), max_x)
        y = min(max(y, oy), max_y)
        return (x, y)
    def rrt_plan_path(self, start, goal, fused_grid, info, max_iterations=1000, step_size=1.0, goal_threshold=1.0, goal_bias=0.2):
        """
        Planifica un camino usando un RRT simple con sesgo hacia el goal.
        start, goal: tuplas (x, y) en coordenadas del mundo.
        fused_grid, info: mapa fusionado y su metadata.
        max_iterations: número máximo de iteraciones para muestreo.
        step_size: tamaño del paso para extender el árbol.
        goal_threshold: distancia para considerar alcanzada la meta.
        goal_bias: probabilidad de usar el goal en el muestreo.
        Retorna una lista de puntos que representa el camino o None si falla.
        """
        # Inicializa el árbol con el nodo de inicio.
        tree = [{'point': start, 'parent': None}]
        
        for i in range(max_iterations):
            # Definir límites a partir de la información del mapa.
            ox = info.origin.position.x
            oy = info.origin.position.y
            max_x = ox + info.width * info.resolution
            max_y = oy + info.height * info.resolution

            # Con probabilidad goal_bias, usar el goal como punto muestreado
            if np.random.rand() < goal_bias:
                random_point = goal
            else:
                # Muestrear un punto aleatorio dentro de los límites.
                random_point = (np.random.uniform(ox, max_x), np.random.uniform(oy, max_y))
            
            # Buscar el nodo más cercano en el árbol.
            nearest = min(tree, key=lambda node: distance(node['point'], random_point))
            
            # Calcular la dirección (ángulo) hacia el punto muestreado.
            theta = math.atan2(random_point[1] - nearest['point'][1],
                            random_point[0] - nearest['point'][0])
            # Crear un nuevo punto extendido desde el nodo cercano.
            new_point = (nearest['point'][0] + step_size * math.cos(theta),
                        nearest['point'][1] + step_size * math.sin(theta))
            
            # Verificar que la línea desde el nodo cercano hasta el nuevo punto esté libre de obstáculos.
            if not self.is_line_free(fused_grid, info, nearest['point'], new_point):
                continue
            
            # Añadir el nuevo nodo al árbol.
            new_node = {'point': new_point, 'parent': nearest}
            tree.append(new_node)
            
            # Si el nuevo nodo está suficientemente cerca del goal y hay línea libre hacia él, se ha encontrado una solución.
            if distance(new_point, goal) < goal_threshold and self.is_line_free(fused_grid, info, new_point, goal):
                goal_node = {'point': goal, 'parent': new_node}
                tree.append(goal_node)
                # Reconstruir el camino retrocediendo desde el nodo goal.
                path = []
                current = goal_node
                while current is not None:
                    path.append(current['point'])
                    current = current['parent']
                path.reverse()
                return path
        # Si no se encuentra un camino en el número máximo de iteraciones, se retorna None.
        return None


    def is_line_free(self,fused_grid, info, start, end):
        """
        Verifica si la línea entre start y end está libre de obstáculos usando el algoritmo de Bresenham.
        start, end: Tuplas (x, y) en coordenadas del mundo.
        fused_grid: Array 2D del mapa fusionado.
        info: MapMetaData del mapa fusionado.
        Retorna True si la línea es libre, False si se detecta un obstáculo.
        """
        # Convertir coordenadas del mundo a índices del grid
        def world_to_index(point, info):
            i = int((point[0] - info.origin.position.x) / info.resolution)
            j = int((point[1] - info.origin.position.y) / info.resolution)
            return i, j

        x0, y0 = world_to_index(start, info)
        x1, y1 = world_to_index(end, info)

        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = -1 if x0 > x1 else 1
        sy = -1 if y0 > y1 else 1
        if dx > dy:
            err = dx / 2.0
            while x != x1:
                if fused_grid[y, x] == 100:  # Asumiendo que 100 es obstáculo
                    return False
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                if fused_grid[y, x] == 100:
                    return False
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        # Revisar la celda final
        if fused_grid[y, x] == 100:
            return False
        return True

    def plan_path(self, nodes, fused_grid, info):
        """
        Construye un grafo a partir de los nodos y añade la posición actual y el goal.
        Solo conecta nodos que estén dentro del umbral Y tengan línea libre.
        """
        if self.odom is None or self.goal is None:
            return None
        # Posición actual y goal
        robot_pos = (self.odom.position.x, self.odom.position.y)
        goal_pos = (self.goal.position.x, self.goal.position.y)
        
        # Proyectar el goal si es necesario (como se mostró previamente)
        if self.current_grid is not None:
            goal_pos = self.project_goal(goal_pos, self.current_grid.info)
        elif self.memory_grid is not None:
            goal_pos = self.project_goal(goal_pos, self.memory_grid.info)
        
        # Añadir robot y goal al conjunto de nodos
        all_nodes = nodes.copy()
        all_nodes.append(robot_pos)
        all_nodes.append(goal_pos)
        robot_index = len(all_nodes) - 2
        goal_index = len(all_nodes) - 1
        
        # Construir grafo: conectar nodos si están a menos de connection_threshold metros
        # Y si la línea entre ellos es libre de obstáculos.
        graph = {i: [] for i in range(len(all_nodes))}
        for i in range(len(all_nodes)):
            for j in range(i + 1, len(all_nodes)):
                d = distance(all_nodes[i], all_nodes[j])
                if d <= self.connection_threshold:
                    if self.is_line_free(fused_grid, info, all_nodes[i], all_nodes[j]):
                        graph[i].append((j, d))
                        graph[j].append((i, d))
        path_indices = dijkstra(graph, robot_index, goal_index)
        if not path_indices or len(path_indices) < 2:
            self.get_logger().warn("No se pudo planificar un path global, usando línea directa.")
            return [robot_pos, goal_pos]
        path = [all_nodes[i] for i in path_indices]
        return path



    def publish_path(self, path):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        for pt in path:
            pose_st = PoseStamped()
            pose_st.header = path_msg.header
            pose_st.pose.position.x = pt[0]
            pose_st.pose.position.y = pt[1]
            pose_st.pose.position.z = 0.0
            pose_st.pose.orientation.w = 1.0
            path_msg.poses.append(pose_st)
        self.path_pub.publish(path_msg)
        self.get_logger().info(f"Path global publicado con {len(path)} puntos.")

    # def pure_pursuit_control(self, path, current_pos, current_yaw):
    #     """
    #     Calcula el comando de velocidad utilizando Pure Pursuit:
    #       - Busca el primer punto en el path a una distancia >= lookahead_distance.
    #       - Calcula el error de dirección y genera el giro.
    #     """
    #     target_point = None
    #     for pt in path:
    #         if distance(current_pos, pt) >= self.lookahead_distance:
    #             target_point = pt
    #             break
    #     if target_point is None:
    #         target_point = path[-1]
    #     desired_heading = math.atan2(target_point[1] - current_pos[1],
    #                                  target_point[0] - current_pos[0])
    #     error_angle = desired_heading - current_yaw
    #     error_angle = math.atan2(math.sin(error_angle), math.cos(error_angle))
    #     angular_speed = self.k_pursuit * error_angle
    #     return self.linear_speed, angular_speed


    def pure_pursuit_control(self, path, current_pos, current_yaw):
        """
        Calcula el comando de velocidad utilizando Pure Pursuit.
        """
        target_point = None
        # Buscar el primer punto en el path a una distancia >= lookahead_distance
        for pt in path:
            if distance(current_pos, pt) >= self.lookahead_distance:
                target_point = pt
                break
        if target_point is None:
            target_point = path[-1]
        
        # Calcular el ángulo deseado hacia el target
        desired_heading = math.atan2(target_point[1] - current_pos[1],
                                    target_point[0] - current_pos[0])
        error_angle = desired_heading - current_yaw
        error_angle = math.atan2(math.sin(error_angle), math.cos(error_angle))
        
        # Ajustar parámetros (prueba con diferentes valores)
        linear_speed = self.linear_speed  # Considera reducir este valor si es muy alto.
        angular_speed = self.k_pursuit * error_angle  # Aumenta k_pursuit si es necesario.
        return linear_speed, angular_speed


    # def control_loop(self):
    #     if self.odom is None or self.goal is None:
    #         return
    #     # Fusionar mapas
    #     fused_result = self.fuse_maps_dynamic(self.current_grid, self.memory_grid)
    #     if fused_result is None or fused_result[0] is None:
    #         self.get_logger().warn("No se pudo fusionar mapas. Esperando datos válidos...")
    #         return
    #     fused_grid, info = fused_result
    #     # Extraer nodos candidatos
    #     nodes = self.extract_nodes(fused_grid, info)
    #     # Planificar path
    #     path = self.plan_path(nodes, fused_grid, info)
    #     if path is None or len(path) < 2:
    #         self.get_logger().warn("No se pudo calcular el path global.")
    #         return
    #     self.publish_path(path)
    #     # Control Pure Pursuit
    #     current_x = self.odom.position.x
    #     current_y = self.odom.position.y
    #     current_pos = (current_x, current_y)
    #     # Extraer el yaw actual
    #     q = self.odom.orientation
    #     siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    #     cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    #     current_yaw = math.atan2(siny_cosp, cosy_cosp)
    #     linear_speed, angular_speed = self.pure_pursuit_control(path, current_pos, current_yaw)
    #     # Crear y publicar comando Twist
    #     twist = Twist()
    #     twist.linear.x = linear_speed
    #     twist.angular.z = angular_speed
    #     self.cmd_vel_pub.publish(twist)
    #     self.get_logger().info(f"Twist publicado: linear = {linear_speed:.2f} m/s, angular = {angular_speed:.2f} rad/s")


    ## es bueno
    # def control_loop(self):
    #     if self.odom is None or self.goal is None:
    #         return
    #     # Fusionar mapas
    #     fused_result = self.fuse_maps_dynamic(self.current_grid, self.memory_grid)
    #     if fused_result is None or fused_result[0] is None:
    #         self.get_logger().warn("No se pudo fusionar mapas. Esperando datos válidos...")
    #         return
    #     fused_grid, info = fused_result
    #     # Extraer nodos candidatos del mapa fusionado
    #     nodes = self.extract_nodes(fused_grid, info)
        
    #     # Intentar planificar el camino con el método actual
    #     path = self.plan_path(nodes, fused_grid, info)
    #     if path is None or len(path) < 3:
    #         self.get_logger().warn("Planificación convencional fallida, intentando RRT.")
    #         # Usar RRT para planificar el camino
    #         start = (self.odom.position.x, self.odom.position.y)
    #         goal = (self.goal.position.x, self.goal.position.y)
    #         path = self.rrt_plan_path(start, goal, fused_grid, info)
    #         if path is None:
    #             self.get_logger().warn("RRT falló. Usando línea directa.")
    #             path = [start, goal]
    #     self.publish_path(path)
        
    #     # Control Pure Pursuit (como antes)
    #     current_pos = (self.odom.position.x, self.odom.position.y)
    #     q = self.odom.orientation
    #     siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    #     cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    #     current_yaw = math.atan2(siny_cosp, cosy_cosp)
    #     linear_speed, angular_speed = self.pure_pursuit_control(path, current_pos, current_yaw)
    #     twist = Twist()
    #     twist.linear.x = linear_speed
    #     twist.angular.z = angular_speed
    #     self.cmd_vel_pub.publish(twist)
    #     self.get_logger().info(f"Twist publicado: linear = {linear_speed:.2f} m/s, angular = {angular_speed:.2f} rad/s")


# Método para suavizar el camino (ejemplo sencillo)
    def smooth_path(self, path):
        # Aquí se puede implementar un suavizado (por ejemplo, media móvil)
        if len(path) < 3:
            return path
        smoothed = [path[0]]
        for i in range(1, len(path)-1):
            # Promediar con el punto anterior y siguiente
            avg_x = (path[i-1][0] + path[i][0] + path[i+1][0]) / 3.0
            avg_y = (path[i-1][1] + path[i][1] + path[i+1][1]) / 3.0
            smoothed.append((avg_x, avg_y))
        smoothed.append(path[-1])
        return smoothed

    # Función para comparar caminos y decidir si se actualiza
    def path_quality_improved(self, new_path, old_path):
        # Ejemplo: si el nuevo camino tiene al menos un 30% más puntos (o menor desviación respecto al goal)
        if old_path is None:
            return True
        len_new = len(new_path)
        len_old = len(old_path)
        # Si el nuevo camino es mucho más detallado, se puede considerar mejor
        return (len_new - len_old) / (len_old + 1e-6) > self.path_update_threshold

    def check_local_obstacles(self, fused_grid, info, current_pos, current_yaw, lookahead_distance=1.0, cone_angle=0.5):
        """
        Revisa en el mapa fusionado si hay obstáculos en una zona frontal definida por
        lookahead_distance y un ángulo cone_angle (en radianes).
        Retorna (obstacle_detected, min_distance) donde:
        - obstacle_detected: Booleano que indica si se detectó obstáculo.
        - min_distance: La distancia mínima al obstáculo dentro de la zona.
        """
        ox = info.origin.position.x
        oy = info.origin.position.y
        resolution = info.resolution
        grid_height, grid_width = fused_grid.shape

        obstacle_detected = False
        min_distance = float('inf')

        # Recorremos una ventana en el frente del robot
        # Por ejemplo, de 0 a lookahead_distance en dirección de current_yaw
        # y ±cone_angle a cada lado.
        steps = int(lookahead_distance / resolution)
        for step in range(1, steps + 1):
            # Distancia actual en la dirección del robot
            dist = step * resolution
            # Revisar varios ángulos dentro del cono
            num_angles = 5  # por ejemplo, 5 ángulos en el cono
            for a in np.linspace(-cone_angle, cone_angle, num_angles):
                angle = current_yaw + a
                # Coordenadas del punto a revisar
                x = current_pos[0] + dist * math.cos(angle)
                y = current_pos[1] + dist * math.sin(angle)
                # Convertir a índices en el grid
                i = int((x - ox) / resolution)
                j = int((y - oy) / resolution)
                if 0 <= i < grid_width and 0 <= j < grid_height:
                    cell_value = fused_grid[j, i]
                    # Asumimos que 100 es obstáculo y -1 es desconocido (podrías tratar -1 como peligro)
                    if cell_value == 100 or cell_value == -1:
                        obstacle_detected = True
                        if dist < min_distance:
                            min_distance = dist
                else:
                    # Si se sale del mapa, se considera zona desconocida
                    obstacle_detected = True
                    if dist < min_distance:
                        min_distance = dist
        return obstacle_detected, min_distance

###muy bueno
    # def control_loop(self):
    #     if self.odom is None or self.goal is None:
    #         return

    #     # Fusionar mapas y obtener info (como antes)
    #     fused_result = self.fuse_maps_dynamic(self.current_grid, self.memory_grid)
    #     if fused_result is None or fused_result[0] is None:
    #         self.get_logger().warn("No se pudo fusionar mapas. Esperando datos válidos...")
    #         return
    #     fused_grid, info = fused_result

    #     current_pos = (self.odom.position.x, self.odom.position.y)
    #     current_time = self.get_clock().now().nanoseconds / 1e9

    #     # Determinar si se debe replanificar globalmente
    #     force_replan = False
    #     if self.last_valid_path is None:
    #         force_replan = True
    #     elif (current_time - self.last_replan_time) > self.replan_interval:
    #         # Solo forzamos la replanificación si el cambio en la posición es mayor que un umbral
    #         if distance(current_pos, self.last_valid_path[0]) > self.path_change_threshold:
    #             force_replan = True

    #     if force_replan:
    #         nodes = self.extract_nodes(fused_grid, info)
    #         new_path = self.plan_path(nodes, fused_grid, info)
    #         if new_path is None or len(new_path) < 3:
    #             self.get_logger().warn("Planificación convencional fallida, intentando RRT.")
    #             new_path = self.rrt_plan_path(current_pos, (self.goal.position.x, self.goal.position.y), fused_grid, info)
    #             if new_path is None:
    #                 self.get_logger().warn("RRT falló. Usando línea directa.")
    #                 new_path = [current_pos, (self.goal.position.x, self.goal.position.y)]
    #         new_path = self.smooth_path(new_path)
    #         # Actualizamos el camino solo si la diferencia es considerable
    #         if self.last_valid_path is None or self.path_quality_improved(new_path, self.last_valid_path):
    #             self.last_valid_path = new_path
    #             self.last_replan_time = current_time
    #     else:
    #         new_path = self.last_valid_path

    #     self.publish_path(new_path)

    #     # Control local reactivo (verificar obstáculos inmediatos, etc.)
    #     q = self.odom.orientation
    #     siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    #     cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    #     current_yaw = math.atan2(siny_cosp, cosy_cosp)
    #     # Aquí se puede incluir el chequeo local de obstáculos (como en el ejemplo anterior)
    #     obstacle_detected, obs_distance = self.check_local_obstacles(fused_grid, info, current_pos, current_yaw)
    #     if obstacle_detected:
    #         self.get_logger().warn(f"Obstáculo detectado a {obs_distance:.2f} m. Activando evasión local.")
    #         linear_speed = 0.0
    #         angular_speed = 0.5
    #     else:
    #         linear_speed, angular_speed = self.compute_dynamic_twist(current_pos, current_yaw, new_path, fused_grid, info)

    #     twist = Twist()
    #     twist.linear.x = linear_speed
    #     twist.angular.z = angular_speed
    #     alpha = 0.8  # Factor de suavizado (entre 0 y 1)
    #     self.last_twist.linear.x = linear_speed
    #     self.last_twist.angular.z = angular_speed
    #     # self.cmd_vel_pub.publish(twist)
    #     self.get_logger().info(f"Twist publicado: linear = {linear_speed:.2f} m/s, angular = {angular_speed:.2f} rad/s")


    def control_loop(self):
        if self.odom is None or self.goal is None:
            return

        fused_result = self.fuse_maps_dynamic(self.current_grid, self.memory_grid)
        if fused_result is None or fused_result[0] is None:
            self.get_logger().warn("No se pudo fusionar mapas. Esperando datos válidos...")
            return
        fused_grid, info = fused_result

        current_pos = (self.odom.position.x, self.odom.position.y)
        current_time = self.get_clock().now().nanoseconds / 1e9

        # Replanificación global: si no existe path o se fuerza replanteo (según tus criterios)
        if self.current_global_path is None or self.should_replan(current_pos):
            nodes = self.extract_nodes(fused_grid, info)
            new_path = self.plan_path(nodes, fused_grid, info)
            if new_path is None or len(new_path) < 3:
                self.get_logger().warn("Planificación convencional fallida, intentando RRT.")
                new_path = self.rrt_plan_path(current_pos, (self.goal.position.x, self.goal.position.y), fused_grid, info)
                if new_path is None:
                    self.get_logger().warn("RRT falló. Usando línea directa.")
                    new_path = [current_pos, (self.goal.position.x, self.goal.position.y)]
            new_path = self.smooth_path(new_path)
            self.current_global_path = new_path
            self.current_path_index = 0  # Reinicia el índice al planificar un nuevo path
            self.training_buffer.append({
                'time': current_time,
                'global_path': new_path,
                'position': current_pos,
            })

        # Actualizar el índice del path conforme se alcanzan los waypoints
        self.update_path_index(current_pos)

        # Definir el path "restante" a seguir
        effective_path = self.current_global_path[self.current_path_index:] if self.current_global_path else None
        if effective_path is None or len(effective_path) < 2:
            self.get_logger().warn("Path global insuficiente, usando línea directa.")
            effective_path = [current_pos, (self.goal.position.x, self.goal.position.y)]
        
        self.publish_path(effective_path)

        # Calcular velocidades (por ejemplo, con compute_dynamic_twist)
        q = self.odom.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        current_yaw = math.atan2(siny_cosp, cosy_cosp)
        linear_speed, angular_speed = self.compute_dynamic_twist(current_pos, current_yaw, effective_path, fused_grid, info)
        
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        self.cmd_vel_pub.publish(twist)
        self.training_buffer.append({
            'time': current_time,
            'position': current_pos,
            'twist': {'linear': linear_speed, 'angular': angular_speed}
        })
        self.get_logger().info(f"Twist publicado: linear = {linear_speed:.2f} m/s, angular = {angular_speed:.2f} rad/s")
    
    def update_path_index(self, current_pos):
        # Actualiza el índice del waypoint actual mientras se alcancen los puntos
        if self.current_global_path is None or len(self.current_global_path) == 0:
            return
        # Si el robot se encuentra dentro del umbral del waypoint actual, incrementa el índice
        while self.current_path_index < len(self.current_global_path) and \
              distance(current_pos, self.current_global_path[self.current_path_index]) < self.waypoint_threshold:
            self.current_path_index += 1
        # Evitar que se salga del rango; si se alcanzó el último punto, mantenerlo
        if self.current_path_index >= len(self.current_global_path):
            self.current_path_index = len(self.current_global_path) - 1

    def should_replan(self, current_pos):
        # Aquí defines cuándo forzar una nueva planificación global
        # Por ejemplo, si el robot se desvía mucho del path actual
        # O puedes definir un intervalo de tiempo
        # Este ejemplo devuelve False para mantener el path mientras sea válido
        return False

    def publish_path(self, path):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        for pt in path:
            pose_st = PoseStamped()
            pose_st.header = path_msg.header
            pose_st.pose.position.x = pt[0]
            pose_st.pose.position.y = pt[1]
            pose_st.pose.position.z = 0.0
            pose_st.pose.orientation.w = 1.0
            path_msg.poses.append(pose_st)
        self.path_pub.publish(path_msg)
        self.get_logger().info(f"Path global publicado con {len(path)} puntos.")

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
