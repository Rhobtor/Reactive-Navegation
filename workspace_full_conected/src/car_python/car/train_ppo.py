# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# import math
# import numpy as np
# import tensorflow as tf
# import datetime
# import time
# import subprocess
# import os
# import signal

# from std_srvs.srv import Empty
# from std_msgs.msg import Bool
# from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
# from nav_msgs.msg import Odometry
# from visualization_msgs.msg import Marker
# from octomap_msgs.msg import Octomap

# # PPO hiperparámetros y parámetros de navegación
# GOAL_REACHED_DIST = 3.0       
# STEP_PENALTY = -0.05          
# GOAL_REWARD = 50.0            
# OBSTACLE_PENALTY_DIST = 2.0   

# MAX_CANDIDATES = 5            
# # Características: [dist_robot, angle_diff, clearance, dist_to_goal, avg_obs_dist, num_obs_norm]
# FEATURE_DIM = 6               
# # Estado global para el crítico: se añade delta_heading (diferencia de orientación)
# GLOBAL_STATE_DIM = 7          

# GAMMA = 0.99
# LAMBDA = 0.95
# CLIP_EPS = 0.2
# TRAIN_EPOCHS = 10
# BATCH_SIZE = 32

# # Parámetros de exploración y bonus
# EXPLORATION_DISTANCE_THRESHOLD = 3.0  
# EXPLORATION_BONUS_FACTOR = 60.0         
# CLEARANCE_BONUS_FACTOR = 50.0           

# # --- Función auxiliar: Conversión de cuaternión a yaw ---
# def quaternion_to_yaw(q: Quaternion):
#     # Fórmula estándar: yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
#     siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
#     cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
#     return math.atan2(siny_cosp, cosy_cosp)

# # --- Actor Recurrente con LSTM para Entrenamiento ---
# class RecurrentActorNetwork(tf.keras.Model):
#     def __init__(self, max_candidates, feature_dim, lstm_units=64, **kwargs):
#         super(RecurrentActorNetwork, self).__init__(**kwargs)
#         self.max_candidates = max_candidates
#         self.feature_dim = feature_dim
#         self.td1 = tf.keras.layers.TimeDistributed(tf.keras.layers.Dense(64, activation='relu'))
#         self.lstm = tf.keras.layers.LSTM(lstm_units, return_sequences=True, return_state=True)
#         self.logits_layer = tf.keras.layers.TimeDistributed(tf.keras.layers.Dense(1))
        
#     def call(self, x, mask=None, initial_state=None):
#         x = self.td1(x)
#         lstm_out, h, c = self.lstm(x, initial_state=initial_state)
#         logits = self.logits_layer(lstm_out)
#         logits = tf.squeeze(logits, axis=-1)
#         if mask is not None:
#             logits = tf.where(mask, logits, -1e9 * tf.ones_like(logits))
#         return logits, (h, c)
    
#     def get_config(self):
#         config = super(RecurrentActorNetwork, self).get_config()
#         config.update({
#             "max_candidates": self.max_candidates,
#             "feature_dim": self.feature_dim
#         })
#         return config

#     @classmethod
#     def from_config(cls, config):
#         config.pop('trainable', None)
#         config.pop('dtype', None)
#         return cls(**config)

# # --- Critic Network ---
# class CriticNetwork(tf.keras.Model):
#     def __init__(self, input_dim):
#         super(CriticNetwork, self).__init__()
#         self.dense1 = tf.keras.layers.Dense(64, activation='relu')
#         self.dense2 = tf.keras.layers.Dense(64, activation='relu')
#         self.dense3 = tf.keras.layers.Dense(64, activation='relu')
#         self.value = tf.keras.layers.Dense(1)
#     def call(self, x):
#         x = self.dense1(x)
#         x = self.dense2(x)
#         x = self.dense3(x)
#         v = self.value(x)
#         return tf.squeeze(v, axis=-1)

# # --- Nodo de Entrenamiento PPO con Actor Recurrente ---
# class NavigationPPOCandidateTrainer(Node):
#     def __init__(self):
#         super().__init__('navigation_ppo_candidate_trainer')
#         # Subscripciones a odometría, goal, nodos filtrados, ocupados y colisión virtual
#         self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
#         self.create_subscription(PoseArray, '/goal', self.goal_callback, 10)
#         self.create_subscription(PoseArray, '/filtered_navigation_nodes', self.filtered_nodes_callback, 10)
#         self.create_subscription(PoseArray, '/occupied_rejected_nodes', self.occupied_nodes_callback, 10)
#         self.create_subscription(Bool, '/virtual_collision', self.collision_callback, 10)
#         # Subscripción para reinicio del mapa (por supervisor)
#         self.create_subscription(Bool, '/map_reset', self.map_reset_callback, 10)
#         # Publicador para solicitar reinicio del entorno
#         self.reset_request_pub = self.create_publisher(Bool, '/reset_request', 10)
#         # Publicadores para visualización
#         self.marker_pub = self.create_publisher(Marker, '/selected_candidate_marker', 10)
#         self.nav_point = self.create_publisher(PoseArray, '/nav_point', 10)
        
#         # Variables de estado
#         self.odom = None
#         self.goal = None
#         self.filtered_nodes = None
#         self.occupied_nodes = None
#         self.current_candidate = None
#         self.last_candidate = None
#         self.last_candidate_features = None
        
#         # Buffer de experiencia
#         self.states = []
#         self.actor_inputs = []
#         self.actions = []
#         self.log_probs = []
#         self.rewards = []
#         self.values = []
#         self.dones = []
#         self.steps = 0
#         self.max_steps = 1000

#         # Contadores y flags
#         self.collision_detected = False
#         self.collision_counter = 0
#         self.collision_threshold = 3
#         self.escape_counter = 0
#         self.escape_threshold = 5

#         # Detección de inactividad (basada en velocidad)
#         self.last_movement_time = None
#         self.movement_threshold = 0.01  # m/s
#         self.inactivity_time_threshold = 300.0  # segundos

#         # Cooldown para reinicio del entorno
#         self.last_reset_time = 0.0
#         self.reset_cooldown = 20.0  # segundos

#         # Detección de repetición del mismo candidato
#         self.same_candidate_count = 0
#         self.same_candidate_threshold = 3
#         self.same_candidate_distance_threshold = 0.1  # m

#         # Para mejorar la memoria: seguimiento de orientación
#         self.last_heading = None
#         self.delta_heading = 0.0

#         # (Opcional) Suscribirse al mapa de puntos
#         self.create_subscription(PoseArray, '/navigation_map_points', self.map_points_callback, 10)
#         self.map_points = None

#         # TensorBoard y tiempo de entrenamiento
#         self.episode_count = 0
#         self.total_episodes = 1000
#         self.start_time = time.time()
#         self.log_dir = "logs/fit/" + datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
#         self.summary_writer = tf.summary.create_file_writer(self.log_dir)
#         self.get_logger().info(f"TensorBoard logs en: {self.log_dir}")

#         self.models_saved = False

#         # Redes y optimizadores
#         self.actor = RecurrentActorNetwork(MAX_CANDIDATES, FEATURE_DIM)
#         self.actor_state = None
#         self.critic = CriticNetwork(GLOBAL_STATE_DIM)
#         self.actor_optimizer = tf.keras.optimizers.Adam(3e-4)
#         self.critic_optimizer = tf.keras.optimizers.Adam(3e-4)
#         self.get_logger().info("Navigation PPO Candidate Trainer iniciado.")
#         while self.odom is None or self.goal is None:
#             self.get_logger().warn("Esperando /odom y /goal...")
#             rclpy.spin_once(self, timeout_sec=0.1)
#         self.get_logger().info("Datos iniciales recibidos, iniciando selección de nodo.")
#         self.timer = self.create_timer(0.1, self.step)

#     # --- Callbacks ---
#     def odom_callback(self, msg: Odometry):
#         current_time = self.get_clock().now().nanoseconds / 1e9
#         original_pose = msg.pose.pose
#         offset_x = 0.5
#         adjusted_pose = Pose()
#         adjusted_pose.position.x = original_pose.position.x - offset_x
#         adjusted_pose.position.y = original_pose.position.y
#         adjusted_pose.position.z = original_pose.position.z
#         adjusted_pose.orientation = original_pose.orientation
#         self.odom = adjusted_pose

#         # Extraer yaw del robot
#         yaw = quaternion_to_yaw(original_pose.orientation)
#         if self.last_heading is None:
#             self.last_heading = yaw
#             self.delta_heading = 0.0
#         else:
#             self.delta_heading = abs(yaw - self.last_heading)
#             self.last_heading = yaw

#         # Actualizar last_movement_time basándose en la velocidad
#         linear = msg.twist.twist.linear
#         speed = math.hypot(linear.x, linear.y)
#         self.last_speed = speed
#         if speed > self.movement_threshold:
#             self.last_movement_time = current_time

#     def goal_callback(self, msg: PoseArray):
#         if msg.poses:
#             self.goal = msg.poses[0]

#     def filtered_nodes_callback(self, msg: PoseArray):
#         self.filtered_nodes = msg

#     def occupied_nodes_callback(self, msg: PoseArray):
#         self.occupied_nodes = msg

#     def collision_callback(self, msg: Bool):
#         self.virtual_collision = msg.data

#     def map_points_callback(self, msg: PoseArray):
#         self.map_points = msg.poses

#     def map_reset_callback(self, msg: Bool):
#         if msg.data:
#             self.get_logger().info("Se recibió señal para reiniciar el mapa (variables locales).")
#             self.map_points = None

#     # --- Solicitar reinicio del entorno ---
#     def request_environment_reset(self):
#         current_time = self.get_clock().now().nanoseconds / 1e9
#         if current_time - self.last_reset_time >= self.reset_cooldown:
#             reset_msg = Bool()
#             reset_msg.data = True
#             self.reset_request_pub.publish(reset_msg)
#             self.get_logger().info("Solicitud de reinicio del entorno enviada al supervisor.")
#             self.last_reset_time = current_time
#         else:
#             self.get_logger().info("Cooldown de reinicio activo; no se solicita reset.")

#     # --- Reiniciar el estado interno ---
#     def reset_internal_state(self):
#         self.actor_state = None
#         self.collision_counter = 0
#         self.escape_counter = 0
#         self.same_candidate_count = 0
#         self.last_movement_time = self.get_clock().now().nanoseconds / 1e9

#     # --- Estadísticas de obstáculos ---
#     def compute_obstacle_stats(self):
#         if self.occupied_nodes and self.occupied_nodes.poses:
#             distances = [math.hypot(self.odom.position.x - obs.position.x,
#                                     self.odom.position.y - obs.position.y)
#                          for obs in self.occupied_nodes.poses]
#             avg_dist = np.mean(distances)
#             num_obs = len(distances)
#         else:
#             avg_dist = 10.0
#             num_obs = 0
#         num_obs_norm = num_obs / 10.0
#         return avg_dist, num_obs_norm

#     # --- Extraer características para cada candidato ---
#     def compute_candidate_features(self):
#         features = []
#         valid_nodes = []
#         if self.filtered_nodes is None or not self.filtered_nodes.poses:
#             return None, None, None
#         robot_x = self.odom.position.x
#         robot_y = self.odom.position.y
#         goal_x = self.goal.position.x
#         goal_y = self.goal.position.y
#         current_yaw = 0.0
#         avg_obs_dist, num_obs_norm = self.compute_obstacle_stats()
#         for node in self.filtered_nodes.poses:
#             if self.occupied_nodes and self.occupied_nodes.poses:
#                 clearance = min([math.hypot(node.position.x - occ.position.x,
#                                             node.position.y - occ.position.y)
#                                  for occ in self.occupied_nodes.poses])
#             else:
#                 clearance = 10.0
#             if clearance < OBSTACLE_PENALTY_DIST:
#                 continue
#             dx = node.position.x - robot_x
#             dy = node.position.y - robot_y
#             dist_robot = math.hypot(dx, dy)
#             angle_to_node = math.atan2(dy, dx)
#             angle_diff = abs(angle_to_node - current_yaw)
#             dist_to_goal = math.hypot(node.position.x - goal_x, node.position.y - goal_y)
#             feature_vector = [dist_robot, angle_diff, clearance, dist_to_goal, avg_obs_dist, num_obs_norm]
#             features.append(feature_vector)
#             valid_nodes.append(node)
#         if len(features) == 0:
#             return None, None, None
#         features, valid_nodes = zip(*sorted(zip(features, valid_nodes), key=lambda x: x[0][3]))
#         features = list(features)
#         valid_nodes = list(valid_nodes)
#         if len(features) > MAX_CANDIDATES:
#             features = features[:MAX_CANDIDATES]
#             valid_nodes = valid_nodes[:MAX_CANDIDATES]
#         num_valid = len(features)
#         while len(features) < MAX_CANDIDATES:
#             features.append([0.0]*FEATURE_DIM)
#         features = np.array(features, dtype=np.float32)
#         mask = np.array([True]*num_valid + [False]*(MAX_CANDIDATES - num_valid))
#         return features, valid_nodes, mask

#     # --- Estado global para el crítico (se añade delta_heading) ---
#     def get_global_state(self):
#         avg_obs_dist, num_obs_norm = self.compute_obstacle_stats()
#         return np.array([
#             self.odom.position.x,
#             self.odom.position.y,
#             self.goal.position.x,
#             self.goal.position.y,
#             avg_obs_dist,
#             num_obs_norm,
#             self.delta_heading  # Diferencia en orientación
#         ], dtype=np.float32)

#     # # --- Función de recompensa ---
#     # def compute_reward(self):
#     #     # Distancia actual al goal
#     #     d_current = math.hypot(self.odom.position.x - self.goal.position.x,
#     #                            self.odom.position.y - self.goal.position.y)
#     #     # Penaliza acercarse demasiado (se usa la distancia)
#     #     reward = -d_current * 0.1 + STEP_PENALTY
#     #     if d_current < GOAL_REACHED_DIST:
#     #         reward += GOAL_REWARD

#     #     # Penalización por colisión física
#     #     if self.occupied_nodes and self.occupied_nodes.poses:
#     #         for occ in self.occupied_nodes.poses:
#     #             d = math.hypot(self.odom.position.x - occ.position.x,
#     #                            self.odom.position.y - occ.position.y)
#     #             if d < 0.05:
#     #                 reward -= 1000
#     #                 self.get_logger().warn("¡Colisión física detectada!")
#     #                 self.collision_detected = True
#     #                 self.collision_counter += 1
#     #                 break
#     #             elif d < 0.3:
#     #                 reward -= (0.3 - d) * 200

#     #     # Penalización por baja clearance (usando el clearance del primer candidato)
#     #     features, valid_nodes, mask = self.compute_candidate_features()
#     #     if features is not None:
#     #         clearance = features[0][2]
#     #         if clearance < 3.0:
#     #             reward -= 100.0

#     #     # Penalización por colisión virtual
#     #     if hasattr(self, 'virtual_collision') and self.virtual_collision:
#     #         reward -= 500.0
#     #         self.get_logger().warn("¡Colisión virtual detectada!")
        
#     #     # Bonus de exploración: si el robot se mueve (velocidad > 0.05 m/s)
#     #     if self.last_candidate is not None and hasattr(self, 'last_speed') and self.last_speed > 0.05:
#     #         d_explore = math.hypot(self.odom.position.x - self.last_candidate.position.x,
#     #                                self.odom.position.y - self.last_candidate.position.y)
#     #         if d_explore > EXPLORATION_DISTANCE_THRESHOLD:
#     #             bonus = EXPLORATION_BONUS_FACTOR * (d_explore - EXPLORATION_DISTANCE_THRESHOLD)
#     #             reward += bonus
#     #             self.get_logger().info(f"Bonus de exploración: {bonus:.2f}")
        
#     #     # Bonus por mejora en clearance (si el nuevo candidato mejora la seguridad)
#     #     if self.last_candidate_features is not None:
#     #         last_features, _, last_action_index, _ = self.last_candidate_features
#     #         clearance_last = last_features[last_action_index][2]
#     #         new_features, _, _ = self.compute_candidate_features()
#     #         if new_features is not None:
#     #             clearance_new = new_features[0][2]
#     #             if clearance_new > clearance_last:
#     #                 bonus_clearance = CLEARANCE_BONUS_FACTOR * (clearance_new - clearance_last)
#     #                 reward += bonus_clearance
#     #                 self.get_logger().info(f"Bonus por clearance: {bonus_clearance:.2f}")
#     #     return reward
#     def compute_reward(self):
#         # Distancia actual al goal
#         d_current = math.hypot(self.odom.position.x - self.goal.position.x,
#                             self.odom.position.y - self.goal.position.y)
#         # Calcular Δd: mejora en la distancia (si no hay dato previo, se considera 0)
#         if hasattr(self, 'prev_goal_distance'):
#             delta_d = self.prev_goal_distance - d_current
#         else:
#             delta_d = 0.0
#         self.prev_goal_distance = d_current

#         # Obtener clearance del mejor candidato (usando la primera característica del candidato)
#         features, valid_nodes, mask = self.compute_candidate_features()
#         if features is not None:
#             clearance = features[0][2]
#         else:
#             clearance = 10.0

#         # Cambio de dirección (Δθ) obtenido en el callback de odometría (self.delta_heading)
#         delta_theta = self.delta_heading

#         # Parámetros para la fórmula (pueden ajustarse o declararse como parámetros del nodo)
#         alpha = 20.0  # Peso para Δd
#         beta = 60.0   # Peso para la penalización basada en clearance
#         lam = 30.0    # Controla la rapidez de la penalización del clearance
#         delta_param = 15.0  # Peso para Δθ

#         # Penalización por clearance: cuando clearance es pequeño, la penalización es alta.
#         clearance_penalty = beta * math.exp(-lam * clearance)

#         # Fórmula compuesta para la recompensa
#         reward = alpha * delta_d - clearance_penalty + delta_param * delta_theta

#         # Agregar penalización fija por cada paso
#         reward += STEP_PENALTY

#         # Recompensa si se alcanza el goal
#         if d_current < GOAL_REACHED_DIST:
#             reward += GOAL_REWARD

#         # Penalizaciones por colisión física
#         if self.occupied_nodes and self.occupied_nodes.poses:
#             for occ in self.occupied_nodes.poses:
#                 d = math.hypot(self.odom.position.x - occ.position.x,
#                             self.odom.position.y - occ.position.y)
#                 if d < 0.05:
#                     reward -= 1000
#                     self.get_logger().warn("¡Colisión física detectada!")
#                     self.collision_detected = True
#                     break
#                 elif d < 0.3:
#                     reward -= (0.3 - d) * 200

#         # Penalización por colisión virtual
#         if hasattr(self, 'virtual_collision') and self.virtual_collision:
#             reward -= 500
#             self.get_logger().warn("¡Colisión virtual detectada!")

#         # Bonus de exploración: si el robot se mueve (velocidad > 0.05 m/s) y se aleja del último candidato
#         if self.last_candidate is not None and hasattr(self, 'last_speed') and self.last_speed > 0.05:
#             d_explore = math.hypot(self.odom.position.x - self.last_candidate.position.x,
#                                 self.odom.position.y - self.last_candidate.position.y)
#             if d_explore > EXPLORATION_DISTANCE_THRESHOLD:
#                 bonus = EXPLORATION_BONUS_FACTOR * (d_explore - EXPLORATION_DISTANCE_THRESHOLD)
#                 reward += bonus
#                 self.get_logger().info(f"Bonus de exploración: {bonus:.2f}")

#         return reward
#     # --- Placeholder para planificación local (por ejemplo, A*) ---
#     def plan_route(self, candidate, map_points):
#         # Aquí se podría implementar un algoritmo de planificación local.
#         return candidate

#     # --- Función step ---
#     def step(self):
#         if self.odom is None or self.goal is None:
#             return

#         current_time = self.get_clock().now().nanoseconds / 1e9

#         # Verificar inactividad: 10 segundos sin movimiento
#         if self.last_movement_time is not None:
#             if current_time - self.last_movement_time >= self.inactivity_time_threshold:
#                 self.get_logger().warn("Robot inactivo por 10 segundos. Solicitando reinicio del entorno...")
#                 self.request_environment_reset()
#                 self.reset_internal_state()
#                 return

#         # Si se detecta colisión física, usar el nodo previo como backup
#         if self.collision_detected:
#             if self.last_candidate is not None:
#                 self.get_logger().info("Colisión física detectada, usando nodo previo como backup.")
#                 self.current_candidate = self.last_candidate
#             else:
#                 self.get_logger().warn("Colisión física detectada pero no hay nodo previo para backup.")
#             self.collision_detected = False

#         # Si se alcanzó el candidato, reiniciar selección
#         if self.current_candidate is not None:
#             dist_to_candidate = math.hypot(
#                 self.odom.position.x - self.current_candidate.position.x,
#                 self.odom.position.y - self.current_candidate.position.y)
#             if dist_to_candidate < 2.5:
#                 self.get_logger().info("Candidato alcanzado. Seleccionando nuevo candidato.")
#                 self.current_candidate = None

#         # Seleccionar candidato si no hay uno vigente
#         if self.current_candidate is None:
#             candidate_features, valid_nodes, mask = self.compute_candidate_features()
#             if candidate_features is None:
#                 self.get_logger().warn("No hay candidatos válidos.")
#                 self.escape_counter += 1
#                 if self.escape_counter >= self.escape_threshold:
#                     self.get_logger().warn("Bloqueo persistente detectado. Forzando backup (nodo previo).")
#                     if self.last_candidate is not None:
#                         self.current_candidate = self.last_candidate
#                     self.escape_counter = 0
#                 return
#             else:
#                 self.escape_counter = 0
#             actor_input = np.expand_dims(candidate_features, axis=0)
#             mask_input = np.expand_dims(mask, axis=0)
#             logits, self.actor_state = self.actor(actor_input, mask=tf.convert_to_tensor(mask_input), initial_state=self.actor_state)
#             probs = tf.nn.softmax(logits, axis=-1).numpy()[0]
#             action_index = int(np.argmax(probs))
#             if not mask[action_index]:
#                 self.get_logger().warn("Candidato seleccionado inválido.")
#                 return
#             new_candidate = valid_nodes[action_index]
#             if self.map_points is not None:
#                 new_candidate = self.plan_route(new_candidate, self.map_points)
#             if self.last_candidate is not None:
#                 d_same = math.hypot(new_candidate.position.x - self.last_candidate.position.x,
#                                     new_candidate.position.y - self.last_candidate.position.y)
#                 if d_same < self.same_candidate_distance_threshold:
#                     self.same_candidate_count += 1
#                 else:
#                     self.same_candidate_count = 0
#             else:
#                 self.same_candidate_count = 0
#             if self.same_candidate_count >= self.same_candidate_threshold:
#                 self.get_logger().warn("Se ha seleccionado el mismo punto repetidamente. Solicitando reinicio del entorno...")
#                 self.request_environment_reset()
#                 self.reset_internal_state()
#                 self.same_candidate_count = 0
#                 return
#             self.current_candidate = new_candidate
#             self.get_logger().info(f"Nodo candidato seleccionado: ({self.current_candidate.position.x:.2f}, {self.current_candidate.position.y:.2f})")
#             self.last_candidate = self.current_candidate
#             self.last_candidate_features = (candidate_features, mask, action_index, probs)

#         # Publicar candidato en RViz
#         marker = Marker()
#         marker.header.frame_id = "map"
#         marker.header.stamp = self.get_clock().now().to_msg()
#         marker.ns = "nodo_candidato"
#         marker.id = 0
#         marker.type = Marker.SPHERE
#         marker.action = Marker.ADD
#         marker.pose = self.current_candidate
#         marker.scale.x = 0.2
#         marker.scale.y = 0.2
#         marker.scale.z = 0.2
#         marker.color.a = 1.0
#         marker.color.r = 0.0
#         marker.color.g = 0.0
#         marker.color.b = 1.0
#         self.marker_pub.publish(marker)

#         nav_points = PoseArray()
#         nav_points.header.stamp = self.get_clock().now().to_msg()
#         nav_points.header.frame_id = "map"
#         nav_points.poses.append(self.current_candidate)
#         self.nav_point.publish(nav_points)

#         # Actualizar experiencia
#         if self.last_candidate_features is not None:
#             stored_features, stored_mask, stored_action_index, stored_probs = self.last_candidate_features
#             global_state = self.get_global_state()
#             value = self.critic(tf.convert_to_tensor([global_state], dtype=tf.float32)).numpy()[0]
#             reward = self.compute_reward()
#             done = 1 if math.hypot(self.odom.position.x - self.goal.position.x,
#                                     self.odom.position.y - self.goal.position.y) < GOAL_REACHED_DIST else 0
#             self.states.append(global_state)
#             self.actor_inputs.append((stored_features, stored_mask))
#             self.actions.append(stored_action_index)
#             self.log_probs.append(np.log(stored_probs[stored_action_index] + 1e-8))
#             self.values.append(value)
#             self.rewards.append(reward)
#             self.dones.append(done)
#         self.steps += 1

#         if self.current_candidate is not None:
#             dist_to_candidate = math.hypot(
#                 self.odom.position.x - self.current_candidate.position.x,
#                 self.odom.position.y - self.current_candidate.position.y)
#             if dist_to_candidate < 2.5:
#                 self.get_logger().info("Candidato alcanzado. Seleccionando nuevo candidato.")
#                 self.current_candidate = None

#         if self.current_candidate is not None:
#             self.last_candidate = self.current_candidate

#         if done or self.steps >= self.max_steps:
#             self.get_logger().info(f"Episodio terminado en {self.steps} pasos.")
#             self.update_model()
#             elapsed = time.time() - self.start_time
#             avg_ep_time = elapsed / (self.episode_count + 1)
#             remaining = (self.total_episodes - (self.episode_count + 1)) * avg_ep_time
#             self.get_logger().info(f"Tiempo transcurrido: {elapsed:.1f}s, Tiempo estimado restante: {remaining:.1f}s")
#             self.states = []
#             self.actor_inputs = []
#             self.actions = []
#             self.log_probs = []
#             self.values = []
#             self.rewards = []
#             self.dones = []
#             self.steps = 0
#             self.current_candidate = None
#             self.episode_count += 1
#             self.actor_state = None

#             elapsed = time.time() - self.start_time
#             avg_ep_time = elapsed / self.episode_count
#             remaining = (self.total_episodes - self.episode_count) * avg_ep_time
#             self.get_logger().info(f"Tiempo transcurrido: {elapsed:.1f}s, Tiempo estimado restante: {remaining:.1f}s")
            
#             # --- Registro de progreso de episodios ---
#             progress_bar_length = 1  # Longitud de la barra de progreso
#             completed_units = int((self.episode_count / self.total_episodes) * progress_bar_length)
#             progress_bar = "[" + "#" * completed_units + "-" * (progress_bar_length - completed_units) + "]"
#             self.get_logger().info(f"Episodios: {self.episode_count}/{self.total_episodes} {progress_bar}")




#     def compute_advantages(self, rewards, values, dones):
#         advantages = np.zeros_like(rewards, dtype=np.float32)
#         last_adv = 0.0
#         for t in reversed(range(len(rewards))):
#             next_value = values[t+1] if t+1 < len(values) else 0.0
#             delta = rewards[t] + GAMMA * next_value * (1 - dones[t]) - values[t]
#             advantages[t] = last_adv = delta + GAMMA * LAMBDA * (1 - dones[t]) * last_adv
#         returns = advantages + np.array(values, dtype=np.float32)
#         return advantages, returns

#     def save_models(self):
#         timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
#         actor_model_filename = f"actor_model_{timestamp}.keras"
#         critic_model_filename = f"critic_model_{timestamp}.keras"
#         self.actor.save(actor_model_filename)
#         self.critic.save(critic_model_filename)
#         self.get_logger().info(f"Modelos guardados: {actor_model_filename} y {critic_model_filename}.")

#     def update_model(self):
#         states = np.array(self.states, dtype=np.float32)
#         actions = np.array(self.actions, dtype=np.int32)
#         log_probs_old = np.array(self.log_probs, dtype=np.float32)
#         rewards = np.array(self.rewards, dtype=np.float32)
#         dones = np.array(self.dones, dtype=np.float32)
#         advantages, returns = self.compute_advantages(rewards, self.values, dones)
#         advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)
#         N = len(states)
#         actor_inputs = []
#         masks = []
#         for cand_feat, m in self.actor_inputs:
#             actor_inputs.append(cand_feat)
#             masks.append(m)
#         actor_inputs = np.array(actor_inputs, dtype=np.float32)
#         masks = np.array(masks, dtype=bool)
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
#                     logits, _ = self.actor(batch_actor_inputs, mask=batch_masks, initial_state=None)
#                     batch_actions = tf.cast(batch_actions, tf.int32)
#                     indices = tf.stack([tf.range(tf.shape(logits)[0]), batch_actions], axis=1)
#                     new_log_probs = tf.math.log(tf.nn.softmax(logits, axis=1) + 1e-8)
#                     new_log_probs = tf.gather_nd(new_log_probs, indices)
#                     ratio = tf.exp(new_log_probs - batch_old_log_probs)
#                     clipped_ratio = tf.clip_by_value(ratio, 1 - CLIP_EPS, 1 + CLIP_EPS)
#                     actor_loss = -tf.reduce_mean(tf.minimum(ratio * batch_advantages, clipped_ratio * batch_advantages))
#                     values_pred = self.critic(batch_states)
#                     critic_loss = tf.reduce_mean(tf.square(batch_returns - values_pred))
#                     total_loss = actor_loss + 0.5 * critic_loss
#                 actor_grads = tape.gradient(total_loss, self.actor.trainable_variables)
#                 critic_grads = tape.gradient(total_loss, self.critic.trainable_variables)
#                 self.actor_optimizer.apply_gradients(zip(actor_grads, self.actor.trainable_variables))
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
#         if self.episode_count >= 400:
#             self.get_logger().info("Se han completado 400 episodios. Finalizando nodo.")
#             rclpy.shutdown()

# def main(args=None):
#     rclpy.init(args=args)
#     node = NavigationPPOCandidateTrainer()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# import math
# import numpy as np
# import tensorflow as tf
# import datetime
# import time
# import heapq
# import os
# import signal

# from std_srvs.srv import Empty
# from std_msgs.msg import Bool
# from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
# from nav_msgs.msg import Odometry
# from visualization_msgs.msg import Marker
# from octomap_msgs.msg import Octomap

# # ----------------------- Constantes y Hiperparámetros -----------------------
# GOAL_REACHED_DIST = 3.0       
# STEP_PENALTY = -0.05          
# GOAL_REWARD = 50.0            

# # Para el modelo end-to-end
# PATH_LENGTH = 5
# ACTION_DIM = PATH_LENGTH * 2  # 10
# PATH_SCALE = 20.0             # Factor para escalar la salida (rango en metros)
# ACTION_STD = 0.1              # Desviación estándar fija para la política gaussiana

# # Nuevo estado global: [odom_x, odom_y, goal_x, goal_y, avg_obs_dist, num_obs_norm, delta_heading, min_obstacle_dist, obs_count_norm]
# GLOBAL_STATE_DIM = 9          

# GAMMA = 0.99
# LAMBDA = 0.95
# CLIP_EPS = 0.2
# TRAIN_EPOCHS = 10
# BATCH_SIZE = 32

# # Parámetros de exploración adicionales (si se desea)
# EXPLORATION_BONUS_FACTOR = 60.0         

# # ----------------------- Funciones Auxiliares -----------------------
# def distance(p1, p2):
#     """Calcula la distancia euclidiana entre dos puntos (x,y)."""
#     return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

# def quaternion_to_yaw(q: Quaternion):
#     """Conversión de cuaternión a yaw."""
#     siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
#     cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
#     return math.atan2(siny_cosp, cosy_cosp)

# # ----------------------- Redes Neuronales -----------------------
# class EndToEndActor(tf.keras.Model):
#     def __init__(self, action_dim, hidden_units=128, **kwargs):
#         super(EndToEndActor, self).__init__(**kwargs)
#         self.dense1 = tf.keras.layers.Dense(hidden_units, activation='relu')
#         self.dense2 = tf.keras.layers.Dense(hidden_units, activation='relu')
#         # Usamos tanh para limitar la salida a [-1,1]; luego se escala
#         self.output_layer = tf.keras.layers.Dense(action_dim, activation='tanh')
#     def call(self, x):
#         x = self.dense1(x)
#         x = self.dense2(x)
#         return self.output_layer(x)

# class CriticNetwork(tf.keras.Model):
#     def __init__(self, input_dim, hidden_units=128):
#         super(CriticNetwork, self).__init__()
#         self.dense1 = tf.keras.layers.Dense(hidden_units, activation='relu')
#         self.dense2 = tf.keras.layers.Dense(hidden_units, activation='relu')
#         self.value = tf.keras.layers.Dense(1)
#     def call(self, x):
#         x = self.dense1(x)
#         x = self.dense2(x)
#         v = self.value(x)
#         return tf.squeeze(v, axis=-1)

# # ----------------------- Nodo de Entrenamiento End-to-End -----------------------
# class NavigationEndToEndTrainer(Node):
#     def __init__(self):
#         super().__init__('navigation_end2end_trainer')
#         # Subscripciones
#         self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
#         self.create_subscription(PoseArray, '/goal', self.goal_callback, 10)
#         self.create_subscription(PoseArray, '/occupied_rejected_nodes', self.occupied_nodes_callback, 10)
#         self.create_subscription(Bool, '/virtual_collision', self.collision_callback, 10)
#         # Obstacle points (obstáculos móviles/fijos detectados 2D)
#         self.create_subscription(PoseArray, '/obstacle_navigation_nodes', self.obstacle_points_callback, 10)
#         # Reinicio del mapa
#         self.create_subscription(Bool, '/map_reset', self.map_reset_callback, 10)
#         # Publicadores (visualización y envío de path)
#         self.marker_pub = self.create_publisher(Marker, '/planned_path_marker', 10)
#         self.nav_point = self.create_publisher(PoseArray, '/nav_point', 10)
#         self.reset_request_pub = self.create_publisher(Bool, '/reset_request', 10)

#         # Estado
#         self.odom = None
#         self.goal = None
#         self.occupied_nodes = None
#         self.obstacle_points = None  # Obstáculos 2D detectados
#         self.virtual_collision = False

#         # Para detectar inactividad
#         self.last_movement_time = None
#         self.movement_threshold = 0.01  # m/s
#         self.inactivity_time_threshold = 300.0  # segundos

#         # Para reinicios
#         self.last_reset_time = 0.0
#         self.reset_cooldown = 20.0  # segundos

#         # Buffer de experiencia
#         self.states = []
#         self.actions = []
#         self.log_probs = []
#         self.rewards = []
#         self.values = []
#         self.dones = []
#         self.steps = 0
#         self.max_steps = 1000

#         # TensorBoard y control de episodios
#         self.episode_count = 0
#         self.total_episodes = 10
#         self.start_time = time.time()
#         self.log_dir = "logs/fit/" + datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
#         self.summary_writer = tf.summary.create_file_writer(self.log_dir)
#         self.get_logger().info(f"TensorBoard logs en: {self.log_dir}")

#         self.models_saved = False

#         # Redes y optimizadores
#         self.actor = EndToEndActor(ACTION_DIM)
#         self.critic = CriticNetwork(GLOBAL_STATE_DIM)
#         self.actor_optimizer = tf.keras.optimizers.Adam(3e-4)
#         self.critic_optimizer = tf.keras.optimizers.Adam(3e-4)

#         self.get_logger().info("Navigation End-to-End Trainer iniciado.")
#         while self.odom is None or self.goal is None:
#             self.get_logger().warn("Esperando /odom y /goal...")
#             rclpy.spin_once(self, timeout_sec=0.1)
#         self.get_logger().info("Datos iniciales recibidos, iniciando entrenamiento.")
#         self.timer = self.create_timer(0.1, self.step)

#     # ----------------------- Callbacks -----------------------
#     def odom_callback(self, msg: Odometry):
#         current_time = self.get_clock().now().nanoseconds / 1e9
#         original_pose = msg.pose.pose
#         offset_x = 0.5
#         adjusted_pose = Pose()
#         adjusted_pose.position.x = original_pose.position.x - offset_x
#         adjusted_pose.position.y = original_pose.position.y
#         adjusted_pose.position.z = original_pose.position.z
#         adjusted_pose.orientation = original_pose.orientation
#         self.odom = adjusted_pose

#         yaw = quaternion_to_yaw(original_pose.orientation)
#         if self.last_movement_time is None:
#             self.last_movement_time = current_time
#         # Actualizar tiempo de movimiento basado en velocidad
#         linear = msg.twist.twist.linear
#         speed = math.hypot(linear.x, linear.y)
#         self.last_speed = speed
#         if speed > self.movement_threshold:
#             self.last_movement_time = current_time

#     def goal_callback(self, msg: PoseArray):
#         if msg.poses:
#             self.goal = msg.poses[0]

#     def occupied_nodes_callback(self, msg: PoseArray):
#         self.occupied_nodes = msg

#     def collision_callback(self, msg: Bool):
#         self.virtual_collision = msg.data

#     def obstacle_points_callback(self, msg: PoseArray):
#         self.obstacle_points = msg

#     def map_reset_callback(self, msg: Bool):
#         if msg.data:
#             self.get_logger().info("Se recibió señal de reinicio del mapa.")
    
#     # ----------------------- Solicitar reinicio del entorno -----------------------
#     def request_environment_reset(self):
#         current_time = self.get_clock().now().nanoseconds / 1e9
#         if current_time - self.last_reset_time >= self.reset_cooldown:
#             reset_msg = Bool()
#             reset_msg.data = True
#             self.reset_request_pub.publish(reset_msg)
#             self.get_logger().info("Solicitud de reinicio del entorno enviada.")
#             self.last_reset_time = current_time
#         else:
#             self.get_logger().info("Cooldown de reinicio activo; no se solicita reset.")

#     # ----------------------- Función para calcular estadísticas de obstáculos -----------------------
#     def compute_obstacle_stats(self):
#         # A partir de occupied_nodes (colisiones muy próximas)
#         if self.occupied_nodes and self.occupied_nodes.poses:
#             dists = [distance((self.odom.position.x, self.odom.position.y),
#                               (obs.position.x, obs.position.y))
#                      for obs in self.occupied_nodes.poses]
#             avg_dist = np.mean(dists)
#             num_obs = len(dists)
#         else:
#             avg_dist = 10.0
#             num_obs = 0
#         num_obs_norm = num_obs / 10.0
#         return avg_dist, num_obs_norm

#     # ----------------------- Estado Global -----------------------
#     def get_global_state(self):
#         avg_obs_dist, num_obs_norm = self.compute_obstacle_stats()
#         # De obstacle_points, calcular la distancia mínima y el número de obstáculos
#         min_obs_dist = 10.0
#         obs_count = 0
#         if self.obstacle_points is not None and self.obstacle_points.poses:
#             dists = [math.hypot(self.odom.position.x - p.position.x, self.odom.position.y - p.position.y)
#                      for p in self.obstacle_points.poses]
#             min_obs_dist = min(dists)
#             obs_count = len(dists)
#         obs_count_norm = obs_count / 10.0
#         return np.array([
#             self.odom.position.x,
#             self.odom.position.y,
#             self.goal.position.x,
#             self.goal.position.y,
#             avg_obs_dist,
#             num_obs_norm,
#             abs(quaternion_to_yaw(self.odom.orientation)),  # delta_heading aproximado
#             min_obs_dist,
#             obs_count_norm
#         ], dtype=np.float32)

#     # ----------------------- Función que interpreta la acción y genera un path -----------------------
#     def generate_path(self, action, current_pos):
#         """
#         A partir de la salida del actor (vector de dimensión ACTION_DIM),
#         se interpreta como una secuencia de 5 waypoints (cada uno de 2D). Se usan de manera
#         acumulativa (cada waypoint se suma al anterior) y se escala por PATH_SCALE.
#         """
#         path = []
#         wp = np.array(current_pos, dtype=np.float32)
#         for i in range(0, ACTION_DIM, 2):
#             offset = np.array([action[i], action[i+1]]) * PATH_SCALE
#             wp = wp + offset  # acumulativo
#             path.append(wp.copy())
#         return path

#     # ----------------------- Función de recompensa -----------------------
#     def compute_reward(self, planned_path):
#         current_pos = np.array([self.odom.position.x, self.odom.position.y])
#         goal_pos = np.array([self.goal.position.x, self.goal.position.y])
#         d_current = np.linalg.norm(current_pos - goal_pos)
#         d_final = np.linalg.norm(planned_path[-1] - goal_pos)
#         progress_reward = (d_current - d_final) * 20.0  # recompensa por acercarse al goal

#         # Penalización de seguridad: si algún waypoint está muy cerca de un obstáculo (del topic obstacle_points)
#         safety_penalty = 0.0
#         if self.obstacle_points is not None and self.obstacle_points.poses:
#             for wp in planned_path:
#                 d_list = [math.hypot(wp[0] - p.position.x, wp[1] - p.position.y)
#                           for p in self.obstacle_points.poses]
#                 if d_list:
#                     min_wp = min(d_list)
#                     if min_wp < 1.0:
#                         safety_penalty += (1.0 - min_wp) * 50.0

#         # Penalización por colisión física (occupied_nodes)
#         collision_penalty = 0.0
#         if self.occupied_nodes and self.occupied_nodes.poses:
#             for occ in self.occupied_nodes.poses:
#                 d = math.hypot(self.odom.position.x - occ.position.x, self.odom.position.y - occ.position.y)
#                 if d < 0.5:
#                     collision_penalty += 1000.0

#         reward = progress_reward - safety_penalty - collision_penalty + STEP_PENALTY
#         if d_current < GOAL_REACHED_DIST:
#             reward += GOAL_REWARD
#         return reward

#     # ----------------------- Función step -----------------------
#     def step(self):
#         if self.odom is None or self.goal is None:
#             return

#         current_time = self.get_clock().now().nanoseconds / 1e9
#         if self.last_movement_time is not None:
#             if current_time - self.last_movement_time >= self.inactivity_time_threshold:
#                 self.get_logger().warn("Robot inactivo. Solicitando reinicio del entorno...")
#                 self.request_environment_reset()
#                 return

#         # Preparar estado global
#         global_state = self.get_global_state()  # shape (9,)
#         actor_input = np.expand_dims(global_state, axis=0)  # shape (1,9)

#         # Obtener acción: muestreo de la política gaussiana
#         action_mean = self.actor(actor_input).numpy()[0]  # shape (10,)
#         noise = np.random.normal(0, ACTION_STD, size=action_mean.shape)
#         action = action_mean + noise
#         # Calcular log-probabilidad (sumatoria sobre dimensiones)
#         log_prob = -0.5 * np.sum(((action - action_mean)/ACTION_STD)**2 + np.log(2*np.pi*(ACTION_STD**2)))

#         # Interpretar la acción para generar el path planificado
#         current_pos = [self.odom.position.x, self.odom.position.y]
#         planned_path = self.generate_path(action, current_pos)
#         # Para enviar la orden de navegación, publicamos el primer waypoint del path
#         next_wp = planned_path[0]
#         nav_pose = Pose()
#         nav_pose.position.x = next_wp[0]
#         nav_pose.position.y = next_wp[1]
#         nav_pose.orientation = self.goal.orientation  # o bien, mantener la orientación actual
#         nav_points_msg = PoseArray()
#         nav_points_msg.header.stamp = self.get_clock().now().to_msg()
#         nav_points_msg.header.frame_id = "map"
#         nav_points_msg.poses.append(nav_pose)
#         self.nav_point.publish(nav_points_msg)

#         # Opcional: Publicar el path completo en RViz (como markers o como PoseArray)
#         marker = Marker()
#         marker.header.frame_id = "map"
#         marker.header.stamp = self.get_clock().now().to_msg()
#         marker.ns = "planned_path"
#         marker.id = 0
#         marker.type = Marker.LINE_STRIP
#         marker.action = Marker.ADD
#         marker.scale.x = 0.1
#         marker.color.a = 1.0
#         marker.color.r = 0.0
#         marker.color.g = 1.0
#         marker.color.b = 0.0
#         # Convertir cada waypoint a Pose (solo posición)
#         for wp in planned_path:
#             p = Pose()
#             p.position.x = wp[0]
#             p.position.y = wp[1]
#             marker.points.append(Point(x=wp[0], y=wp[1], z=0.0))
#         self.marker_pub.publish(marker)

#         # Calcular recompensa basada en el path planificado
#         reward = self.compute_reward(planned_path)

#         # Evaluar valor con la crítica
#         value = self.critic(tf.convert_to_tensor([global_state], dtype=tf.float32)).numpy()[0]

#         # Determinar si se alcanzó el goal (usando la posición real del robot)
#         done = 1 if math.hypot(self.odom.position.x - self.goal.position.x,
#                                self.odom.position.y - self.goal.position.y) < GOAL_REACHED_DIST else 0

#         # Guardar la experiencia
#         self.states.append(global_state)
#         self.actions.append(action)
#         self.log_probs.append(log_prob)
#         self.values.append(value)
#         self.rewards.append(reward)
#         self.dones.append(done)
#         self.steps += 1

#         if done or self.steps >= self.max_steps:
#             self.get_logger().info(f"Episodio terminado en {self.steps} pasos.")
#             self.update_model()
#             elapsed = time.time() - self.start_time
#             avg_ep_time = elapsed / (self.episode_count + 1)
#             remaining = (self.total_episodes - (self.episode_count + 1)) * avg_ep_time
#             self.get_logger().info(f"Tiempo transcurrido: {elapsed:.1f}s, restante: {remaining:.1f}s")
#             # Reiniciar buffers
#             self.states = []
#             self.actions = []
#             self.log_probs = []
#             self.values = []
#             self.rewards = []
#             self.dones = []
#             self.steps = 0
#             self.episode_count += 1

#             progress_bar_length = 20
#             completed_units = int((self.episode_count / self.total_episodes) * progress_bar_length)
#             progress_bar = "[" + "#" * completed_units + "-" * (progress_bar_length - completed_units) + "]"
#             self.get_logger().info(f"Episodios: {self.episode_count}/{self.total_episodes} {progress_bar}")

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
#         states = np.array(self.states, dtype=np.float32)
#         actions = np.array(self.actions, dtype=np.float32)
#         log_probs_old = np.array(self.log_probs, dtype=np.float32)
#         rewards = np.array(self.rewards, dtype=np.float32)
#         dones = np.array(self.dones, dtype=np.float32)
#         advantages, returns = self.compute_advantages(rewards, self.values, dones)
#         advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)
#         N = len(states)
#         dataset = tf.data.Dataset.from_tensor_slices((states, actions, log_probs_old, returns, advantages))
#         dataset = dataset.shuffle(N).batch(BATCH_SIZE)
#         total_actor_loss = 0.0
#         total_critic_loss = 0.0
#         total_loss_value = 0.0
#         batch_count = 0
#         for epoch in range(TRAIN_EPOCHS):
#             for batch in dataset:
#                 batch_states, batch_actions, batch_old_log_probs, batch_returns, batch_advantages = batch
#                 with tf.GradientTape(persistent=True) as tape:
#                     # Obtener media de la política
#                     action_mean = self.actor(batch_states)
#                     # Calcular log probs de la acción (suponiendo distribución gaussiana independiente)
#                     new_log_probs = -0.5 * tf.reduce_sum(((batch_actions - action_mean) / ACTION_STD)**2 
#                                                          + tf.math.log(2 * np.pi * (ACTION_STD**2)), axis=1)
#                     ratio = tf.exp(new_log_probs - batch_old_log_probs)
#                     clipped_ratio = tf.clip_by_value(ratio, 1 - CLIP_EPS, 1 + CLIP_EPS)
#                     actor_loss = -tf.reduce_mean(tf.minimum(ratio * batch_advantages, clipped_ratio * batch_advantages))
#                     values_pred = self.critic(batch_states)
#                     critic_loss = tf.reduce_mean(tf.square(batch_returns - values_pred))
#                     total_loss = actor_loss + 0.5 * critic_loss
#                 actor_grads = tape.gradient(total_loss, self.actor.trainable_variables)
#                 critic_grads = tape.gradient(total_loss, self.critic.trainable_variables)
#                 self.actor_optimizer.apply_gradients(zip(actor_grads, self.actor.trainable_variables))
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
#         if self.episode_count >= 400:
#             self.get_logger().info("Se han completado 400 episodios. Finalizando nodo.")
#             rclpy.shutdown()

#     def save_models(self):
#         timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
#         actor_model_filename = f"actor_model_{timestamp}.keras"
#         critic_model_filename = f"critic_model_{timestamp}.keras"
#         self.actor.save(actor_model_filename)
#         self.critic.save(critic_model_filename)
#         self.get_logger().info(f"Modelos guardados: {actor_model_filename} y {critic_model_filename}.")

# def main(args=None):
#     rclpy.init(args=args)
#     node = NavigationEndToEndTrainer()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# import math
# import numpy as np
# import tensorflow as tf
# import datetime
# import time
# import heapq
# import os
# import signal

# from std_srvs.srv import Empty
# from std_msgs.msg import Bool
# from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
# from nav_msgs.msg import Odometry
# from visualization_msgs.msg import Marker
# from octomap_msgs.msg import Octomap

# # ----------------------- Constantes y Hiperparámetros -----------------------
# GOAL_REACHED_DIST = 3.0       
# STEP_PENALTY = -0.05          
# GOAL_REWARD = 50.0            
# OBSTACLE_PENALTY_DIST = 2.0   

# MAX_CANDIDATES = 5            
# # Características: [dist_robot, angle_diff, clearance, dist_to_goal, avg_obs_dist, num_obs_norm]
# FEATURE_DIM = 6               
# GLOBAL_STATE_DIM = 7  # (no se usa en el actor de candidatos, pero puede emplearse en la función de recompensa si se desea)

# GAMMA = 0.99
# LAMBDA = 0.95
# CLIP_EPS = 0.2
# TRAIN_EPOCHS = 10
# BATCH_SIZE = 32

# EXPLORATION_DISTANCE_THRESHOLD = 3.0  
# EXPLORATION_BONUS_FACTOR = 60.0         
# CLEARANCE_BONUS_FACTOR = 50.0           
# MEMORY_WINDOW_SIZE = 200  # Número máximo de pasos que se retendrán en la memoria
# # ----------------------- Funciones Auxiliares para el Planificador Local -----------------------
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

# def construir_grafo(nodos, umbral_conexion):
#     # Inicializar grafo con todos los índices, incluso si no tienen vecinos.
#     N = len(nodos)
#     grafo = {i: [] for i in range(N)}
#     for i in range(N):
#         for j in range(i+1, N):
#             d = distance(nodos[i], nodos[j])
#             if d <= umbral_conexion:
#                 grafo[i].append((j, d))
#                 grafo[j].append((i, d))
#     return grafo

# # ----------------------- Conversión de Cuaternión a Yaw -----------------------
# def quaternion_to_yaw(q: Quaternion):
#     siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
#     cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
#     return math.atan2(siny_cosp, cosy_cosp)

# # ----------------------- Actor para Selección de Candidato -----------------------
# # Se reutiliza una arquitectura similar a la RecurrentActorNetwork ya que opera sobre un conjunto fijo de candidatos.
# class RecurrentActorNetwork(tf.keras.Model):
#     def __init__(self, max_candidates, feature_dim, lstm_units=64, **kwargs):
#         super(RecurrentActorNetwork, self).__init__(**kwargs)
#         self.max_candidates = max_candidates
#         self.feature_dim = feature_dim
#         self.td1 = tf.keras.layers.TimeDistributed(tf.keras.layers.Dense(64, activation='relu'))
#         self.lstm = tf.keras.layers.LSTM(lstm_units, return_sequences=True, return_state=True)
#         self.logits_layer = tf.keras.layers.TimeDistributed(tf.keras.layers.Dense(1))
        
#     def call(self, x, mask=None, initial_state=None):
#         # x shape: (batch, max_candidates, feature_dim)
#         x = self.td1(x)
#         lstm_out, h, c = self.lstm(x, initial_state=initial_state)
#         logits = self.logits_layer(lstm_out)
#         logits = tf.squeeze(logits, axis=-1)  # shape: (batch, max_candidates)
#         if mask is not None:
#             logits = tf.where(mask, logits, -1e9 * tf.ones_like(logits))
#         return logits, (h, c)

# # ----------------------- Nodo de Entrenamiento End-to-End -----------------------
# class NavigationEndToEndTrainer(Node):
#     def __init__(self):
#         super().__init__('navigation_end2end_trainer')
#         # Subscripciones: odometría, goal, filtered_points, obstáculos y colisiones virtuales
#         self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
#         self.create_subscription(PoseArray, '/goal', self.goal_callback, 10)
#         self.create_subscription(PoseArray, '/filtered_navigation_nodes', self.filtered_nodes_callback, 10)
#         self.create_subscription(PoseArray, '/occupied_rejected_nodes', self.occupied_nodes_callback, 10)
#         self.create_subscription(PoseArray, '/obstacle_points', self.obstacle_points_callback, 10)
#         self.create_subscription(Bool, '/virtual_collision', self.collision_callback, 10)
#         self.create_subscription(Bool, '/map_reset', self.map_reset_callback, 10)
#         # Publicadores para visualización y envío de ruta
#         self.marker_pub = self.create_publisher(Marker, '/planned_path_marker', 10)
#         self.nav_point = self.create_publisher(PoseArray, '/nav_point', 10)
#         self.reset_request_pub = self.create_publisher(Bool, '/reset_request', 10)

#         # Estado interno
#         self.odom = None
#         self.goal = None
#         self.filtered_nodes = None   # Puntos de navegación filtrados
#         self.occupied_nodes = None
#         self.obstacle_points = None  # Obstáculos 2D detectados
#         self.virtual_collision = False

#         self.last_movement_time = None
#         self.movement_threshold = 0.01
#         self.inactivity_time_threshold = 300.0

#         self.last_reset_time = 0.0
#         self.reset_cooldown = 20.0

#         # Experiencia para entrenamiento
#         self.actor_inputs = []
#         self.states = []
#         self.actions = []  # Acción: índice del candidato seleccionado
#         self.log_probs = []
#         self.rewards = []
#         self.values = []
#         self.dones = []
#         self.steps = 0
#         self.max_steps = 1000

#         self.episode_count = 0
#         self.total_episodes = 1000
#         self.start_time = time.time()
#         self.log_dir = "logs/fit/" + datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
#         self.summary_writer = tf.summary.create_file_writer(self.log_dir)
#         self.get_logger().info(f"TensorBoard logs en: {self.log_dir}")

#         self.models_saved = False

#         # Redes y optimizadores
#         self.actor = RecurrentActorNetwork(MAX_CANDIDATES, FEATURE_DIM)
#         self.actor_state = None
#         self.critic = tf.keras.layers.Dense(1)  # Para simplificar, usamos una capa densa en el crítico
#         self.actor_optimizer = tf.keras.optimizers.Adam(3e-4)
#         self.critic_optimizer = tf.keras.optimizers.Adam(3e-4)

#         self.get_logger().info("Navigation End-to-End Trainer iniciado.")
#         while self.odom is None or self.goal is None:
#             self.get_logger().warn("Esperando /odom y /goal...")
#             rclpy.spin_once(self, timeout_sec=0.1)
#         self.get_logger().info("Datos iniciales recibidos, iniciando entrenamiento.")
#         self.timer = self.create_timer(0.1, self.step)

#     # ----------------------- Callbacks -----------------------
#     def odom_callback(self, msg: Odometry):
#         current_time = self.get_clock().now().nanoseconds / 1e9
#         original_pose = msg.pose.pose
#         offset_x = 0.5
#         adjusted_pose = Pose()
#         adjusted_pose.position.x = original_pose.position.x - offset_x
#         adjusted_pose.position.y = original_pose.position.y
#         adjusted_pose.position.z = original_pose.position.z
#         adjusted_pose.orientation = original_pose.orientation
#         self.odom = adjusted_pose
#         linear = msg.twist.twist.linear
#         speed = math.hypot(linear.x, linear.y)
#         self.last_speed = speed
#         if self.last_movement_time is None or speed > self.movement_threshold:
#             self.last_movement_time = current_time

#     def goal_callback(self, msg: PoseArray):
#         if msg.poses:
#             self.goal = msg.poses[0]

#     def filtered_nodes_callback(self, msg: PoseArray):
#         self.filtered_nodes = msg

#     def occupied_nodes_callback(self, msg: PoseArray):
#         self.occupied_nodes = msg

#     def obstacle_points_callback(self, msg: PoseArray):
#         self.obstacle_points = msg

#     def collision_callback(self, msg: Bool):
#         self.virtual_collision = msg.data

#     def map_reset_callback(self, msg: Bool):
#         if msg.data:
#             self.get_logger().info("Se recibió señal para reiniciar el mapa.")

#     # ----------------------- Solicitar Reinicio -----------------------
#     def request_environment_reset(self):
#         current_time = self.get_clock().now().nanoseconds / 1e9
#         if current_time - self.last_reset_time >= self.reset_cooldown:
#             reset_msg = Bool()
#             reset_msg.data = True
#             self.reset_request_pub.publish(reset_msg)
#             self.get_logger().info("Solicitud de reinicio enviada.")
#             self.last_reset_time = current_time
#         else:
#             self.get_logger().info("Cooldown activo; no se solicita reset.")

#     # ----------------------- Cálculo de Características para Candidatos -----------------------
#     def compute_candidate_features(self):
#         features = []
#         valid_nodes = []
#         if self.filtered_nodes is None or not self.filtered_nodes.poses:
#             return None, None, None
#         robot_x = self.odom.position.x
#         robot_y = self.odom.position.y
#         goal_x = self.goal.position.x
#         goal_y = self.goal.position.y
#         for node in self.filtered_nodes.poses:
#             # Calcular clearance usando occupied_nodes (si hay)
#             if self.occupied_nodes and self.occupied_nodes.poses:
#                 clearance = min([distance((node.position.x, node.position.y),
#                                            (occ.position.x, occ.position.y))
#                                  for occ in self.occupied_nodes.poses])
#             else:
#                 clearance = 10.0
#             if clearance < OBSTACLE_PENALTY_DIST:
#                 continue
#             dist_robot = distance((robot_x, robot_y), (node.position.x, node.position.y))
#             dist_to_goal = distance((node.position.x, node.position.y), (goal_x, goal_y))
#             angle_diff = 0.0  # Puede calcularse si se desea
#             # Para simplificar, dejamos avg_obs_dist y num_obs_norm en 0
#             feature_vector = [dist_robot, angle_diff, clearance, dist_to_goal, 0.0, 0.0]
#             features.append(feature_vector)
#             valid_nodes.append(node)
#         if len(features) == 0:
#             return None, None, None
#         # Ordenar según distancia al goal
#         features, valid_nodes = zip(*sorted(zip(features, valid_nodes), key=lambda x: x[0][3]))
#         features = list(features)
#         valid_nodes = list(valid_nodes)
#         if len(features) > MAX_CANDIDATES:
#             features = features[:MAX_CANDIDATES]
#             valid_nodes = valid_nodes[:MAX_CANDIDATES]
#         num_valid = len(features)
#         while len(features) < MAX_CANDIDATES:
#             features.append([0.0] * FEATURE_DIM)
#         features = np.array(features, dtype=np.float32)
#         mask = np.array([True] * num_valid + [False] * (MAX_CANDIDATES - num_valid))
#         return features, valid_nodes, mask

#     # ----------------------- Planificación Local Basada en filtered_nodes -----------------------
#     def plan_route(self, candidate):
#         """
#         Construye un grafo usando:
#           - Posición actual del robot,
#           - Los puntos en filtered_nodes,
#           - El candidato seleccionado.
#         Se utiliza Dijkstra para calcular la ruta más corta.
#         Retorna un tuple: (waypoint_inmediato, camino_calculado),
#         donde 'waypoint_inmediato' es un objeto Pose con el primer waypoint,
#         y 'camino_calculado' es una lista de tuplas (x,y) que representan el camino.
#         """
#         if self.filtered_nodes is None or not self.filtered_nodes.poses:
#             # Si no hay nodos filtrados, devolvemos directamente el candidato
#             candidate_pose = candidate
#             camino = [(self.odom.position.x, self.odom.position.y),
#                       (candidate.position.x, candidate.position.y)]
#             return candidate_pose, camino

#         current_pos = (self.odom.position.x, self.odom.position.y)
#         candidate_pos = (candidate.position.x, candidate.position.y)
#         # Construir la lista de nodos: posición actual, luego los nodos filtrados y finalmente el candidato.
#         nodos = [current_pos]
#         for node in self.filtered_nodes.poses:
#             nodos.append((node.position.x, node.position.y))
#         nodos.append(candidate_pos)

#         umbral_conexion = 3.0
#         grafo = construir_grafo(nodos, umbral_conexion)
#         inicio = 0
#         objetivo = len(nodos) - 1
#         indices_camino = dijkstra(grafo, inicio, objetivo)
#         # Construir la lista de waypoints (como tuplas) a partir de indices_camino:
#         camino_calculado = [nodos[i] for i in indices_camino]
#         if len(indices_camino) >= 2:
#             next_index = indices_camino[1]
#             next_waypoint = nodos[next_index]
#             new_pose = Pose()
#             new_pose.position.x = next_waypoint[0]
#             new_pose.position.y = next_waypoint[1]
#             new_pose.orientation = candidate.orientation  # Se puede ajustar según convenga
#             return new_pose, camino_calculado
#         else:
#             return candidate, camino_calculado

#     # ----------------------- Función de Recompensa -----------------------
#     def compute_reward(self, planned_waypoint):
#         current_pos = np.array([self.odom.position.x, self.odom.position.y])
#         goal_pos = np.array([self.goal.position.x, self.goal.position.y])
#         d_current = np.linalg.norm(current_pos - goal_pos)
#         # Se recompensa por reducir la distancia entre la posición actual y el goal
#         # y se penaliza si el waypoint planificado (es decir, el path que se sigue) no se acerca o está muy cerca de un obstáculo.
#         wp = np.array([planned_waypoint.position.x, planned_waypoint.position.y])
#         d_final = np.linalg.norm(wp - goal_pos)
#         progress_reward = (d_current - d_final) * 20.0
#         safety_penalty = 0.0
#         if self.obstacle_points is not None and self.obstacle_points.poses:
#             d_list = [distance((wp[0], wp[1]), (p.position.x, p.position.y))
#                       for p in self.obstacle_points.poses]
#             if d_list:
#                 min_wp = min(d_list)
#                 if min_wp < 1.0:
#                     safety_penalty += (1.0 - min_wp) * 50.0
#         collision_penalty = 0.0
#         if self.occupied_nodes and self.occupied_nodes.poses:
#             for occ in self.occupied_nodes.poses:
#                 d = distance((self.odom.position.x, self.odom.position.y),
#                              (occ.position.x, occ.position.y))
#                 if d < 0.5:
#                     collision_penalty += 1000.0
#         reward = progress_reward - safety_penalty - collision_penalty + STEP_PENALTY
#         if d_current < GOAL_REACHED_DIST:
#             reward += GOAL_REWARD
#         return reward

#     # ----------------------- Función step -----------------------


#     def step(self):
#         if self.odom is None or self.goal is None:
#             return
#         current_time = self.get_clock().now().nanoseconds / 1e9
#         if self.last_movement_time is not None and current_time - self.last_movement_time >= self.inactivity_time_threshold:
#             self.get_logger().warn("Robot inactivo. Solicitando reinicio...")
#             self.request_environment_reset()
#             return

#         # Selección de candidato: si no hay candidato actual, se calcula a partir de filtered_nodes.
#         if not hasattr(self, 'current_candidate') or self.current_candidate is None:
#             candidate_features, valid_nodes, mask = self.compute_candidate_features()
#             if candidate_features is None:
#                 self.get_logger().warn("No hay candidatos válidos.")
#                 return
#             # Guardamos las características y la máscara en last_candidate_features
#             self.last_candidate_features = (candidate_features, mask)
#             actor_input = np.expand_dims(candidate_features, axis=0)  # forma: (1, MAX_CANDIDATES, FEATURE_DIM)
#             mask_input = np.expand_dims(mask, axis=0)
#             logits, self.actor_state = self.actor(actor_input, mask=tf.convert_to_tensor(mask_input), initial_state=self.actor_state)
#             probs = tf.nn.softmax(logits, axis=-1).numpy()[0]
#             action_index = int(np.argmax(probs))
#             if not mask[action_index]:
#                 self.get_logger().warn("Candidato seleccionado inválido.")
#                 return
#             new_candidate = valid_nodes[action_index]
#             self.current_candidate = new_candidate
#             self.last_action_index = action_index  # Almacenar el índice seleccionado
#             self.last_probs = probs                # Almacenar la distribución de probabilidades
#             self.get_logger().info(f"Candidato seleccionado: ({new_candidate.position.x:.2f}, {new_candidate.position.y:.2f})")
#         else:
#             # Si ya existe un candidato, se reutilizan los valores previamente calculados.
#             if not hasattr(self, 'last_action_index'):
#                 self.last_action_index = 0
#             if not hasattr(self, 'last_probs'):
#                 self.last_probs = np.zeros(MAX_CANDIDATES)
#             if not hasattr(self, 'last_candidate_features'):
#                 candidate_features, valid_nodes, mask = self.compute_candidate_features()
#                 self.last_candidate_features = (candidate_features, mask)

#         # En cada paso, agregamos a actor_inputs la representación (candidatos y máscara) utilizada.
#         self.actor_inputs.append(self.last_candidate_features)
        
#         # Planificar el camino usando los puntos de filtered_nodes y el candidato seleccionado.
#         planned_waypoint, computed_path = self.plan_route(self.current_candidate)
#         # Publicar el waypoint inmediato para navegación.
#         nav_points = PoseArray()
#         nav_points.header.stamp = self.get_clock().now().to_msg()
#         nav_points.header.frame_id = "map"
#         nav_points.poses.append(planned_waypoint)
#         self.nav_point.publish(nav_points)

#         # Publicar el camino calculado en RViz como un LINE_STRIP.
#         marker = Marker()
#         marker.header.frame_id = "map"
#         marker.header.stamp = self.get_clock().now().to_msg()
#         marker.ns = "planned_path"
#         marker.id = 0
#         marker.type = Marker.LINE_STRIP
#         marker.action = Marker.ADD
#         marker.scale.x = 0.1
#         marker.color.a = 1.0
#         marker.color.r = 0.0
#         marker.color.g = 1.0
#         marker.color.b = 0.0
#         marker.points = []
#         for pt in computed_path:
#             marker.points.append(Point(x=pt[0], y=pt[1], z=0.0))
#         self.marker_pub.publish(marker)

#         # Calcular recompensa en función del waypoint planificado.
#         reward = self.compute_reward(planned_waypoint)
#         global_state = np.array([
#             self.odom.position.x,
#             self.odom.position.y,
#             self.goal.position.x,
#             self.goal.position.y,
#             0.0, 0.0, 0.0  # Placeholders para otras variables
#         ], dtype=np.float32)
#         value = self.critic(tf.convert_to_tensor([global_state], dtype=tf.float32)).numpy()[0]
#         done = 1 if distance((self.odom.position.x, self.odom.position.y),
#                             (self.goal.position.x, self.goal.position.y)) < GOAL_REACHED_DIST else 0

#         # Guardar la experiencia.
#         self.states.append(global_state)
#         self.actions.append(self.last_action_index)
#         self.log_probs.append(np.log(self.last_probs[self.last_action_index] + 1e-8))
#         self.values.append(value)
#         self.rewards.append(reward)
#         self.dones.append(done)
#         self.steps += 1

#         # Limitar la memoria volátil: si el buffer supera MEMORY_WINDOW_SIZE, descartar el más antiguo.
#         if len(self.states) > MEMORY_WINDOW_SIZE:
#             self.states.pop(0)
#             self.actions.pop(0)
#             self.log_probs.pop(0)
#             self.values.pop(0)
#             self.rewards.pop(0)
#             self.dones.pop(0)
#             self.actor_inputs.pop(0)

#         # Reiniciar candidato si se alcanzó el waypoint.
#         if distance((self.odom.position.x, self.odom.position.y),
#                     (planned_waypoint.position.x, planned_waypoint.position.y)) < 2.5:
#             self.get_logger().info("Waypoint alcanzado. Reiniciando candidato.")
#             self.current_candidate = None

#         if done or self.steps >= self.max_steps:
#             self.get_logger().info(f"Episodio terminado en {self.steps} pasos.")
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
#             self.actor_inputs = []  # Reiniciamos el buffer de actor_inputs
#             self.steps = 0
#             self.episode_count += 1
#             self.actor_state = None
#             progress_bar_length = 20
#             completed_units = int((self.episode_count / self.total_episodes) * progress_bar_length)
#             progress_bar = "[" + "#" * completed_units + "-" * (progress_bar_length - completed_units) + "]"
#             self.get_logger().info(f"Episodios: {self.episode_count}/{self.total_episodes} {progress_bar}")




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
#         # Convertir buffers a arrays
#         states = np.array(self.states, dtype=np.float32)  # Para el crítico: forma (N, 7)
#         actions = np.array(self.actions, dtype=np.int32)
#         log_probs_old = np.array(self.log_probs, dtype=np.float32)
#         rewards = np.array(self.rewards, dtype=np.float32)
#         dones = np.array(self.dones, dtype=np.float32)
#         advantages, returns = self.compute_advantages(rewards, self.values, dones)
#         advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)
#         # Convertir el buffer de actor_inputs a arrays
#         actor_inputs = []
#         masks = []
#         for cand_feat, m in self.actor_inputs:
#             actor_inputs.append(cand_feat)
#             masks.append(m)
#         actor_inputs = np.array(actor_inputs, dtype=np.float32)  # forma: (N, MAX_CANDIDATES, FEATURE_DIM)
#         masks = np.array(masks, dtype=bool)
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
#                     # Actualizar el actor usando las características de candidato almacenadas.
#                     logits, _ = self.actor(batch_actor_inputs, mask=tf.convert_to_tensor(batch_masks), initial_state=None)
#                     batch_actions = tf.cast(batch_actions, tf.int32)
#                     # Se asume que la acción es el índice (entero) y se extrae la log_probabilidad correspondiente.
#                     indices = tf.stack([tf.range(tf.shape(logits)[0]), batch_actions], axis=1)
#                     new_log_probs = tf.math.log(tf.nn.softmax(logits, axis=1) + 1e-8)
#                     new_log_probs = tf.gather_nd(new_log_probs, indices)
#                     ratio = tf.exp(new_log_probs - batch_old_log_probs)
#                     clipped_ratio = tf.clip_by_value(ratio, 1 - CLIP_EPS, 1 + CLIP_EPS)
#                     actor_loss = -tf.reduce_mean(tf.minimum(ratio * batch_advantages, clipped_ratio * batch_advantages))
#                     # Actualizar el crítico con el estado global
#                     values_pred = self.critic(batch_states)
#                     critic_loss = tf.reduce_mean(tf.square(batch_returns - values_pred))
#                     total_loss = actor_loss + 0.5 * critic_loss
#                 actor_grads = tape.gradient(total_loss, self.actor.trainable_variables)
#                 critic_grads = tape.gradient(total_loss, self.critic.trainable_variables)
#                 self.actor_optimizer.apply_gradients(zip(actor_grads, self.actor.trainable_variables))
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
#         if self.episode_count >= 400:
#             self.get_logger().info("400 episodios completados. Finalizando nodo.")
#             rclpy.shutdown()


#     def save_models(self):
#         timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
#         actor_model_filename = f"actor_model_{timestamp}.keras"
#         critic_model_filename = f"critic_model_{timestamp}.keras"
#         self.actor.save(actor_model_filename)
#         self.critic.save(critic_model_filename)
#         self.get_logger().info(f"Modelos guardados: {actor_model_filename} y {critic_model_filename}.")

# def main(args=None):
#     rclpy.init(args=args)
#     node = NavigationEndToEndTrainer()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# import math
# import numpy as np
# import tensorflow as tf
# import datetime
# import time
# import heapq
# import os
# import signal

# from std_srvs.srv import Empty
# from std_msgs.msg import Bool
# from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
# from nav_msgs.msg import Odometry
# from visualization_msgs.msg import Marker
# from octomap_msgs.msg import Octomap

# # ----------------------- Constantes y Hiperparámetros -----------------------
# GOAL_REACHED_DIST = 3.0       
# STEP_PENALTY = -0.05          
# GOAL_REWARD = 50.0            
# OBSTACLE_PENALTY_DIST = 2.0   

# MAX_CANDIDATES = 5            
# FEATURE_DIM = 6               
# GLOBAL_STATE_DIM = 7  

# GAMMA = 0.99
# LAMBDA = 0.95
# CLIP_EPS = 0.2
# TRAIN_EPOCHS = 10
# BATCH_SIZE = 32

# EXPLORATION_DISTANCE_THRESHOLD = 3.0  
# EXPLORATION_BONUS_FACTOR = 60.0         
# CLEARANCE_BONUS_FACTOR = 50.0           

# MEMORY_WINDOW_SIZE = 200  # Máximo número de experiencias a retener

# # Parámetros para la máquina de estados
# TIMEOUT_THRESHOLD = 10.0  # segundos de timeout en estado MOVING
# WAYPOINT_THRESHOLD = 2.5  # distancia en metros para considerar alcanzado el waypoint

# # ----------------------- Funciones Auxiliares para el Planificador Local -----------------------
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

# def construir_grafo(nodos, umbral_conexion):
#     N = len(nodos)
#     grafo = {i: [] for i in range(N)}
#     for i in range(N):
#         for j in range(i+1, N):
#             d = distance(nodos[i], nodos[j])
#             if d <= umbral_conexion:
#                 grafo[i].append((j, d))
#                 grafo[j].append((i, d))
#     return grafo

# # ----------------------- Conversión de Cuaternión a Yaw -----------------------
# def quaternion_to_yaw(q: Quaternion):
#     siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
#     cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
#     return math.atan2(siny_cosp, cosy_cosp)

# # ----------------------- Actor para Selección de Candidato -----------------------
# class RecurrentActorNetwork(tf.keras.Model):
#     def __init__(self, max_candidates, feature_dim, lstm_units=64, **kwargs):
#         super(RecurrentActorNetwork, self).__init__(**kwargs)
#         self.max_candidates = max_candidates
#         self.feature_dim = feature_dim
#         self.td1 = tf.keras.layers.TimeDistributed(tf.keras.layers.Dense(64, activation='relu'))
#         self.lstm = tf.keras.layers.LSTM(lstm_units, return_sequences=True, return_state=True)
#         self.logits_layer = tf.keras.layers.TimeDistributed(tf.keras.layers.Dense(1))
        
#     def call(self, x, mask=None, initial_state=None):
#         x = self.td1(x)
#         lstm_out, h, c = self.lstm(x, initial_state=initial_state)
#         logits = self.logits_layer(lstm_out)
#         logits = tf.squeeze(logits, axis=-1)
#         if mask is not None:
#             logits = tf.where(mask, logits, -1e9 * tf.ones_like(logits))
#         return logits, (h, c)

# # ----------------------- Nodo de Entrenamiento End-to-End con Memoria Volátil y Desacoplamiento -----------------------
# class NavigationEndToEndTrainer(Node):
#     def __init__(self):
#         super().__init__('navigation_end2end_trainer')
#         # Subscripciones
#         self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
#         self.create_subscription(PoseArray, '/goal', self.goal_callback, 10)
#         self.create_subscription(PoseArray, '/filtered_navigation_nodes', self.filtered_nodes_callback, 10)
#         self.create_subscription(PoseArray, '/occupied_rejected_nodes', self.occupied_nodes_callback, 10)
#         self.create_subscription(PoseArray, '/obstacle_points', self.obstacle_points_callback, 10)
#         self.create_subscription(Bool, '/virtual_collision', self.collision_callback, 10)
#         self.create_subscription(Bool, '/map_reset', self.map_reset_callback, 10)
#         # Publicadores
#         self.marker_pub = self.create_publisher(Marker, '/planned_path_marker', 10)
#         self.nav_point = self.create_publisher(PoseArray, '/nav_point', 10)
#         self.reset_request_pub = self.create_publisher(Bool, '/reset_request', 10)

#         # Estado interno
#         self.odom = None
#         self.goal = None
#         self.filtered_nodes = None
#         self.occupied_nodes = None
#         self.obstacle_points = None
#         self.virtual_collision = False

#         self.last_movement_time = None
#         self.movement_threshold = 0.01
#         self.inactivity_time_threshold = 300.0

#         self.last_reset_time = 0.0
#         self.reset_cooldown = 20.0

#         # Buffers de experiencia
#         self.actor_inputs = []
#         self.states = []
#         self.actions = []  
#         self.log_probs = []
#         self.rewards = []
#         self.values = []
#         self.dones = []
#         self.steps = 0
#         self.max_steps = 1000

#         self.episode_count = 0
#         self.total_episodes = 1000
#         self.start_time = time.time()
#         self.log_dir = "logs/fit/" + datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
#         self.summary_writer = tf.summary.create_file_writer(self.log_dir)
#         self.get_logger().info(f"TensorBoard logs en: {self.log_dir}")

#         self.models_saved = False

#         # Redes y optimizadores
#         self.actor = RecurrentActorNetwork(MAX_CANDIDATES, FEATURE_DIM)
#         self.actor_state = None
#         self.critic = tf.keras.Sequential([
#             tf.keras.layers.InputLayer(input_shape=(GLOBAL_STATE_DIM,)),
#             tf.keras.layers.Dense(64, activation='relu'),
#             tf.keras.layers.Dense(64, activation='relu'),
#             tf.keras.layers.Dense(1)
#         ])
#         self.actor_optimizer = tf.keras.optimizers.Adam(3e-4)
#         self.critic_optimizer = tf.keras.optimizers.Adam(3e-4)

#         # Máquina de estados
#         self.state = "IDLE"  # Estados: "IDLE" (planificar nuevo candidato) y "MOVING" (en ruta)
#         self.state_start_time = None
#         self.initial_position = None

#         self.TIMEOUT_THRESHOLD = TIMEOUT_THRESHOLD
#         self.WAYPOINT_THRESHOLD = WAYPOINT_THRESHOLD

#         # Variable para almacenar el waypoint calculado
#         self.current_waypoint = None

#         self.get_logger().info("Navigation End-to-End Trainer iniciado.")
#         while self.odom is None or self.goal is None:
#             self.get_logger().warn("Esperando /odom y /goal...")
#             rclpy.spin_once(self, timeout_sec=0.1)
#         self.get_logger().info("Datos iniciales recibidos, iniciando entrenamiento.")

#         # Creamos dos timers: uno para replanificación reactiva y otro para registrar experiencia
#         self.reactive_timer = self.create_timer(0.1, self.reactive_step)
#         self.experience_timer = self.create_timer(0.5, self.experience_step)

#     # ----------------------- Callbacks -----------------------
#     def odom_callback(self, msg: Odometry):
#         current_time = self.get_clock().now().nanoseconds / 1e9
#         original_pose = msg.pose.pose
#         offset_x = 0.5
#         adjusted_pose = Pose()
#         adjusted_pose.position.x = original_pose.position.x - offset_x
#         adjusted_pose.position.y = original_pose.position.y
#         adjusted_pose.position.z = original_pose.position.z
#         adjusted_pose.orientation = original_pose.orientation
#         self.odom = adjusted_pose
#         linear = msg.twist.twist.linear
#         speed = math.hypot(linear.x, linear.y)
#         self.last_speed = speed
#         if self.last_movement_time is None or speed > self.movement_threshold:
#             self.last_movement_time = current_time

#     def goal_callback(self, msg: PoseArray):
#         if msg.poses:
#             self.goal = msg.poses[0]

#     def filtered_nodes_callback(self, msg: PoseArray):
#         self.filtered_nodes = msg

#     def occupied_nodes_callback(self, msg: PoseArray):
#         self.occupied_nodes = msg

#     def obstacle_points_callback(self, msg: PoseArray):
#         self.obstacle_points = msg

#     def collision_callback(self, msg: Bool):
#         self.virtual_collision = msg.data

#     def map_reset_callback(self, msg: Bool):
#         if msg.data:
#             self.get_logger().info("Se recibió señal para reiniciar el mapa.")

#     # ----------------------- Solicitar Reinicio -----------------------
#     def request_environment_reset(self):
#         current_time = self.get_clock().now().nanoseconds / 1e9
#         if current_time - self.last_reset_time >= self.reset_cooldown:
#             reset_msg = Bool()
#             reset_msg.data = True
#             self.reset_request_pub.publish(reset_msg)
#             self.get_logger().info("Solicitud de reinicio enviada.")
#             self.last_reset_time = current_time
#         else:
#             self.get_logger().info("Cooldown activo; no se solicita reset.")
#     # ----------------------- Calcula Recompensas -----------------------
#     def compute_reward(self, planned_waypoint):
#         current_pos = np.array([self.odom.position.x, self.odom.position.y])
#         goal_pos = np.array([self.goal.position.x, self.goal.position.y])
#         d_current = np.linalg.norm(current_pos - goal_pos)
#         wp = np.array([planned_waypoint.position.x, planned_waypoint.position.y])
#         d_final = np.linalg.norm(wp - goal_pos)
#         progress_reward = (d_current - d_final) * 20.0
#         safety_penalty = 0.0
#         if self.obstacle_points is not None and self.obstacle_points.poses:
#             d_list = [distance((wp[0], wp[1]), (p.position.x, p.position.y))
#                       for p in self.obstacle_points.poses]
#             if d_list:
#                 min_wp = min(d_list)
#                 if min_wp < 1.0:
#                     safety_penalty += (1.0 - min_wp) * 50.0
#         collision_penalty = 0.0
#         if self.occupied_nodes and self.occupied_nodes.poses:
#             for occ in self.occupied_nodes.poses:
#                 d = distance((self.odom.position.x, self.odom.position.y),
#                              (occ.position.x, occ.position.y))
#                 if d < 0.5:
#                     collision_penalty += 1000.0
#         reward = progress_reward - safety_penalty - collision_penalty + STEP_PENALTY
#         if d_current < GOAL_REACHED_DIST:
#             reward += GOAL_REWARD
#         return reward



#     # ----------------------- Cálculo de Características para Candidatos -----------------------
#     def compute_candidate_features(self):
#         features = []
#         valid_nodes = []
#         if self.filtered_nodes is None or not self.filtered_nodes.poses:
#             return None, None, None
#         robot_x = self.odom.position.x
#         robot_y = self.odom.position.y
#         goal_x = self.goal.position.x
#         goal_y = self.goal.position.y
#         for node in self.filtered_nodes.poses:
#             if self.occupied_nodes and self.occupied_nodes.poses:
#                 clearance = min([distance((node.position.x, node.position.y),
#                                            (occ.position.x, occ.position.y))
#                                  for occ in self.occupied_nodes.poses])
#             else:
#                 clearance = 10.0
#             if clearance < OBSTACLE_PENALTY_DIST:
#                 continue
#             dist_robot = distance((robot_x, robot_y), (node.position.x, node.position.y))
#             dist_to_goal = distance((node.position.x, node.position.y), (goal_x, goal_y))
#             angle_diff = 0.0
#             feature_vector = [dist_robot, angle_diff, clearance, dist_to_goal, 0.0, 0.0]
#             features.append(feature_vector)
#             valid_nodes.append(node)
#         if len(features) == 0:
#             return None, None, None
#         features, valid_nodes = zip(*sorted(zip(features, valid_nodes), key=lambda x: x[0][3]))
#         features = list(features)
#         valid_nodes = list(valid_nodes)
#         if len(features) > MAX_CANDIDATES:
#             features = features[:MAX_CANDIDATES]
#             valid_nodes = valid_nodes[:MAX_CANDIDATES]
#         num_valid = len(features)
#         while len(features) < MAX_CANDIDATES:
#             features.append([0.0] * FEATURE_DIM)
#         features = np.array(features, dtype=np.float32)
#         mask = np.array([True] * num_valid + [False] * (MAX_CANDIDATES - num_valid))
#         return features, valid_nodes, mask

#     # ----------------------- Planificación Local Basada en filtered_nodes -----------------------
#     def plan_route(self, candidate):
#         if self.filtered_nodes is None or not self.filtered_nodes.poses:
#             candidate_pose = candidate
#             camino = [(self.odom.position.x, self.odom.position.y),
#                       (candidate.position.x, candidate.position.y)]
#             return candidate_pose, camino

#         current_pos = (self.odom.position.x, self.odom.position.y)
#         candidate_pos = (candidate.position.x, candidate.position.y)
#         nodos = [current_pos]
#         for node in self.filtered_nodes.poses:
#             nodos.append((node.position.x, node.position.y))
#         nodos.append(candidate_pos)
#         umbral_conexion = 3.0
#         grafo = construir_grafo(nodos, umbral_conexion)
#         inicio = 0
#         objetivo = len(nodos) - 1
#         indices_camino = dijkstra(grafo, inicio, objetivo)
#         camino_calculado = [nodos[i] for i in indices_camino]
#         if len(indices_camino) >= 2:
#             next_index = indices_camino[1]
#             next_waypoint = nodos[next_index]
#             new_pose = Pose()
#             new_pose.position.x = next_waypoint[0]
#             new_pose.position.y = next_waypoint[1]
#             new_pose.orientation = candidate.orientation
#             return new_pose, camino_calculado
#         else:
#             return candidate, camino_calculado

#     # ----------------------- Bucle Reactivo: Actualización Continua del Waypoint -----------------------
#     def reactive_step(self):
#         # Este callback se ejecuta a alta frecuencia (por ejemplo, cada 0.1 s) para actualizar el waypoint.
#         if self.odom is None or self.goal is None:
#             return
#         # Si no hay candidato, se procede a seleccionarlo (estado IDLE)
#         if self.state == "IDLE":
#             candidate_features, valid_nodes, mask = self.compute_candidate_features()
#             if candidate_features is None:
#                 self.get_logger().warn("No hay candidatos válidos.")
#                 return
#             self.last_candidate_features = (candidate_features, mask)
#             actor_input = np.expand_dims(candidate_features, axis=0)
#             mask_input = np.expand_dims(mask, axis=0)
#             logits, self.actor_state = self.actor(actor_input, mask=tf.convert_to_tensor(mask_input), initial_state=self.actor_state)
#             probs = tf.nn.softmax(logits, axis=-1).numpy()[0]
#             action_index = int(np.argmax(probs))
#             if not mask[action_index]:
#                 self.get_logger().warn("Candidato seleccionado inválido.")
#                 return
#             new_candidate = valid_nodes[action_index]
#             self.current_candidate = new_candidate
#             self.last_action_index = action_index
#             self.last_probs = probs
#             self.get_logger().info(f"Candidato seleccionado: ({new_candidate.position.x:.2f}, {new_candidate.position.y:.2f})")
#             self.state = "MOVING"
#             self.state_start_time = self.get_clock().now().nanoseconds / 1e9
#         else:
#             if not hasattr(self, 'last_action_index'):
#                 self.last_action_index = 0
#             if not hasattr(self, 'last_probs'):
#                 self.last_probs = np.zeros(MAX_CANDIDATES)
#             if not hasattr(self, 'last_candidate_features'):
#                 candidate_features, valid_nodes, mask = self.compute_candidate_features()
#                 self.last_candidate_features = (candidate_features, mask)

#         # Actualizar el waypoint reactivo
#         planned_waypoint, computed_path = self.plan_route(self.current_candidate)
#         self.current_waypoint = planned_waypoint
#         nav_points = PoseArray()
#         nav_points.header.stamp = self.get_clock().now().to_msg()
#         nav_points.header.frame_id = "map"
#         nav_points.poses.append(planned_waypoint)
#         self.nav_point.publish(nav_points)
#         marker = Marker()
#         marker.header.frame_id = "map"
#         marker.header.stamp = self.get_clock().now().to_msg()
#         marker.ns = "planned_path"
#         marker.id = 0
#         marker.type = Marker.LINE_STRIP
#         marker.action = Marker.ADD
#         marker.scale.x = 0.1
#         marker.color.a = 1.0
#         marker.color.r = 0.0
#         marker.color.g = 1.0
#         marker.color.b = 0.0
#         marker.points = [Point(x=pt[0], y=pt[1], z=0.0) for pt in computed_path]
#         self.marker_pub.publish(marker)

#     # ----------------------- Bucle de Experiencia: Registro cuando se alcanza el waypoint -----------------------
#     def experience_step(self):
#         if self.state == "MOVING":
#             # Verificar si se ha alcanzado el waypoint reactivo
#             if distance((self.odom.position.x, self.odom.position.y),
#                         (self.current_waypoint.position.x, self.current_waypoint.position.y)) < self.WAYPOINT_THRESHOLD:
#                 self.get_logger().info("Waypoint alcanzado. Registrando experiencia.")
#                 reward = self.compute_reward(self.current_waypoint)
#                 global_state = np.array([
#                     self.odom.position.x,
#                     self.odom.position.y,
#                     self.goal.position.x,
#                     self.goal.position.y,
#                     0.0, 0.0, 0.0
#                 ], dtype=np.float32)
#                 value = self.critic(tf.convert_to_tensor([global_state], dtype=tf.float32)).numpy()[0]
#                 done = 1 if distance((self.odom.position.x, self.odom.position.y),
#                                       (self.goal.position.x, self.goal.position.y)) < GOAL_REACHED_DIST else 0
#                 self.states.append(global_state)
#                 self.actions.append(self.last_action_index)
#                 self.log_probs.append(np.log(self.last_probs[self.last_action_index] + 1e-8))
#                 self.values.append(value)
#                 self.rewards.append(reward)
#                 self.dones.append(done)
#                 self.actor_inputs.append(self.last_candidate_features)
#                 self.steps += 1
#                 # Reiniciar para seleccionar un nuevo candidato
#                 self.current_candidate = None
#                 self.state = "IDLE"
#             else:
#                 # Si ha pasado demasiado tiempo sin alcanzar el waypoint, se aplica timeout
#                 current_time = self.get_clock().now().nanoseconds / 1e9
#                 if current_time - self.state_start_time > self.TIMEOUT_THRESHOLD:
#                     self.get_logger().warn("Timeout en MOVING. Penalizando y reiniciando candidato.")
#                     penalty_reward = -10.0
#                     global_state = np.array([
#                         self.odom.position.x,
#                         self.odom.position.y,
#                         self.goal.position.x,
#                         self.goal.position.y,
#                         0.0, 0.0, 0.0
#                     ], dtype=np.float32)
#                     value = self.critic(tf.convert_to_tensor([global_state], dtype=tf.float32)).numpy()[0]
#                     done = 0
#                     self.states.append(global_state)
#                     self.actions.append(self.last_action_index)
#                     self.log_probs.append(np.log(self.last_probs[self.last_action_index] + 1e-8))
#                     self.values.append(value)
#                     self.rewards.append(penalty_reward)
#                     self.dones.append(done)
#                     self.actor_inputs.append(self.last_candidate_features)
#                     self.steps += 1
#                     self.current_candidate = None
#                     self.state = "IDLE"

#         # Limitar la memoria volátil
#         if len(self.states) > MEMORY_WINDOW_SIZE:
#             self.states.pop(0)
#             self.actions.pop(0)
#             self.log_probs.pop(0)
#             self.values.pop(0)
#             self.rewards.pop(0)
#             self.dones.pop(0)
#             self.actor_inputs.pop(0)

#         # Finalización del episodio
#         if self.steps >= self.max_steps or (self.odom and self.goal and distance((self.odom.position.x, self.odom.position.y),
#                                                    (self.goal.position.x, self.goal.position.y)) < GOAL_REACHED_DIST):
#             self.get_logger().info(f"Episodio terminado en {self.steps} pasos.")
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
#             progress_bar_length = 20
#             completed_units = int((self.episode_count / self.total_episodes) * progress_bar_length)
#             progress_bar = "[" + "#" * completed_units + "-" * (progress_bar_length - completed_units) + "]"
#             self.get_logger().info(f"Episodios: {self.episode_count}/{self.total_episodes} {progress_bar}")

#     # ----------------------- Cálculo de Ventajas y Actualización del Modelo -----------------------
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
#                     batch_actions = tf.cast(batch_actions, tf.int32)
#                     indices = tf.stack([tf.range(tf.shape(logits)[0]), batch_actions], axis=1)
#                     new_log_probs = tf.math.log(tf.nn.softmax(logits, axis=1) + 1e-8)
#                     new_log_probs = tf.gather_nd(new_log_probs, indices)
#                     ratio = tf.exp(new_log_probs - batch_old_log_probs)
#                     clipped_ratio = tf.clip_by_value(ratio, 1 - CLIP_EPS, 1 + CLIP_EPS)
#                     actor_loss = -tf.reduce_mean(tf.minimum(ratio * batch_advantages, clipped_ratio * batch_advantages))
#                     values_pred = self.critic(batch_states)
#                     critic_loss = tf.reduce_mean(tf.square(batch_returns - values_pred))
#                     total_loss = actor_loss + 0.5 * critic_loss
#                 actor_grads = tape.gradient(total_loss, self.actor.trainable_variables)
#                 critic_grads = tape.gradient(total_loss, self.critic.trainable_variables)
#                 self.actor_optimizer.apply_gradients(zip(actor_grads, self.actor.trainable_variables))
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
#         if self.episode_count >= 400:
#             self.get_logger().info("400 episodios completados. Finalizando nodo.")
#             rclpy.shutdown()

#     def save_models(self):
#         timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
#         actor_model_filename = f"actor_model_{timestamp}.keras"
#         critic_model_filename = f"critic_model_{timestamp}.keras"
#         self.actor.save(actor_model_filename)
#         self.critic.save(critic_model_filename)
#         self.get_logger().info(f"Modelos guardados: {actor_model_filename} y {critic_model_filename}.")

# def main(args=None):
#     rclpy.init(args=args)
#     node = NavigationEndToEndTrainer()
#     try:
#         rclpy.spin(node)
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
import tensorflow as tf
import datetime
import time
import heapq
import os
import signal

from std_srvs.srv import Empty
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from octomap_msgs.msg import Octomap
from rclpy.qos import QoSProfile, DurabilityPolicy

# ----------------------- Constantes y Hiperparámetros -----------------------
GOAL_REACHED_DIST = 3.0       
STEP_PENALTY = -0.05          
GOAL_REWARD = 50.0            
OBSTACLE_PENALTY_DIST = 2.7   

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
CLEARANCE_BONUS_FACTOR = 60.0           

MEMORY_WINDOW_SIZE = 200  # Máximo número de experiencias a retener

# Parámetros para la máquina de estados
TIMEOUT_THRESHOLD = 10.0  # segundos de timeout en estado MOVING
WAYPOINT_THRESHOLD = 2.5  # distancia en metros para considerar alcanzado el waypoint

# Parámetros para evitar la trampa: si se repite el mismo candidato demasiadas veces
SAME_CANDIDATE_THRESHOLD = 3

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

# ----------------------- Conversión de Cuaternión a Yaw -----------------------
def quaternion_to_yaw(q: Quaternion):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

# ----------------------- Actor Recurrente -----------------------
# class RecurrentActorNetwork(tf.keras.Model):
#     def __init__(self, max_candidates, feature_dim, lstm_units=256, **kwargs):
#         super(RecurrentActorNetwork, self).__init__(**kwargs)
#         self.max_candidates = max_candidates
#         self.feature_dim = feature_dim
#         self.td1 = tf.keras.layers.TimeDistributed(tf.keras.layers.Dense(64, activation='relu'))
#         self.lstm = tf.keras.layers.LSTM(lstm_units, return_sequences=True, return_state=True)
#         self.logits_layer = tf.keras.layers.TimeDistributed(tf.keras.layers.Dense(1))
        
#     def call(self, x, mask=None, initial_state=None):
#         x = self.td1(x)
#         lstm_out, h, c = self.lstm(x, initial_state=initial_state)
#         logits = self.logits_layer(lstm_out)
#         logits = tf.squeeze(logits, axis=-1)
#         if mask is not None:
#             logits = tf.where(mask, logits, -1e9 * tf.ones_like(logits))
#         return logits, (h, c)


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
        scores = self.attention_dense(lstm_out)  # (batch, timesteps, 1)
        scores = tf.squeeze(scores, axis=-1)     # (batch, timesteps)
        if mask is not None:
            scores = tf.where(mask, scores, -1e9 * tf.ones_like(scores))
        attention_weights = tf.nn.softmax(scores, axis=1)  # (batch, timesteps)
        candidate_logits = self.logits_layer(lstm_out)      # (batch, timesteps, 1)
        candidate_logits = tf.squeeze(candidate_logits, axis=-1)  # (batch, timesteps)
        combined_logits = candidate_logits * attention_weights    # (batch, timesteps)
        return combined_logits, (h, c)


# ----------------------- Nodo de Entrenamiento End-to-End -----------------------
class NavigationEndToEndTrainer(Node):
    def __init__(self):
        super().__init__('navigation_end2end_trainer')
        # Subscripciones
        qos_profile = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseArray, '/goal', self.goal_callback, 10)
        self.create_subscription(PoseArray, '/filtered_navigation_nodes', self.filtered_nodes_callback, 10)
        self.create_subscription(PoseArray, '/occupied_rejected_nodes', self.occupied_nodes_callback, 10)
        self.create_subscription(PoseArray, '/obstacle_points', self.obstacle_points_callback, 10)
        self.create_subscription(Bool, '/virtual_collision', self.collision_callback, 10)
        self.create_subscription(Bool, '/map_reset', self.map_reset_callback, 10)
        # Publicadores
        self.marker_pub = self.create_publisher(Marker, '/planned_path_marker', 10)
        self.nav_point = self.create_publisher(PoseArray, '/nav_point', 10)
        self.reset_request_pub = self.create_publisher(Bool, '/reset_request', qos_profile)
        self.goal_reached_pub = self.create_publisher(Bool, 'goal_reached', qos_profile)
        self.reset_confirmation_received = False
        self.reset_triggered = False

        # Estado interno
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

        # Buffers de experiencia
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
        self.total_episodes = 1000
        self.start_time = time.time()
        self.log_dir = "logs/fit/" + datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        self.summary_writer = tf.summary.create_file_writer(self.log_dir)
        self.get_logger().info(f"TensorBoard logs en: {self.log_dir}")

        self.models_saved = False

        # Redes y optimizadores
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

        # Máquina de estados
        self.state = "IDLE"  # "IDLE": planificar candidato; "MOVING": en ruta.
        self.state_start_time = None
        self.initial_position = None

        self.TIMEOUT_THRESHOLD = TIMEOUT_THRESHOLD
        self.WAYPOINT_THRESHOLD = WAYPOINT_THRESHOLD

        self.current_waypoint = None

        # Variables para detectar repetición de candidato.
        self.last_candidate = None
        self.same_candidate_count = 0
        self.same_candidate_threshold = SAME_CANDIDATE_THRESHOLD

        self.get_logger().info("Navigation End-to-End Trainer iniciado.")
        while self.odom is None or self.goal is None:
            self.get_logger().warn("Esperando /odom y /goal...")
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("Datos iniciales recibidos, iniciando entrenamiento.")

        # Creamos dos timers: uno para replanificación reactiva y otro para registro de experiencia.
        self.reactive_timer = self.create_timer(0.1, self.reactive_step)
        self.experience_timer = self.create_timer(0.5, self.experience_step)

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
            self.get_logger().info("Se recibió señal para reiniciar el mapa.")

    # ----------------------- Solicitar Reinicio -----------------------
    def request_environment_reset(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.last_reset_time >= self.reset_cooldown:
            reset_msg = Bool()
            reset_msg.data = True
            self.reset_request_pub.publish(reset_msg)
            self.get_logger().info("Solicitud de reinicio enviada.")
            self.last_reset_time = current_time
        else:
            self.get_logger().info("Cooldown activo; no se solicita reset.")

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
        if d_current < GOAL_REACHED_DIST:
            reward += GOAL_REWARD
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
        # Si no hay candidato, se procede a seleccionarlo (estado IDLE)
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
            # Comparar con el candidato anterior
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

        # Actualizar el waypoint reactivo
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
        if self.state == "MOVING":
            # Verificar si se ha alcanzado el waypoint reactivo.
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
                # Reiniciar para seleccionar un nuevo candidato.
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

        # Limitar la memoria volátil.
        if len(self.states) > MEMORY_WINDOW_SIZE:
            self.states.pop(0)
            self.actions.pop(0)
            self.log_probs.pop(0)
            self.values.pop(0)
            self.rewards.pop(0)
            self.dones.pop(0)
            self.actor_inputs.pop(0)

        # Finalización del episodio.
        if self.steps >= self.max_steps or (self.odom and self.goal and distance((self.odom.position.x, self.odom.position.y),
                                                    (self.goal.position.x, self.goal.position.y)) < GOAL_REACHED_DIST):
            self.get_logger().info(f"Episodio terminado en {self.steps} pasos.")
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

# if __name__ == '__main__':
#     main()
# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# import math
# import numpy as np
# import tensorflow as tf
# import datetime
# import time
# import heapq
# import os
# import signal
# import random

# from std_srvs.srv import Empty
# from std_msgs.msg import Bool
# from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
# from nav_msgs.msg import Odometry
# from visualization_msgs.msg import Marker
# from octomap_msgs.msg import Octomap
# from rclpy.qos import QoSProfile, DurabilityPolicy

# # ----------------------- Constantes y Hiperparámetros -----------------------
# GOAL_REACHED_DIST = 2.5       
# STEP_PENALTY = -0.05          
# GOAL_REWARD = 50.0            
# OBSTACLE_PENALTY_DIST = 2.5   

# MAX_CANDIDATES = 100            
# FEATURE_DIM = 6               
# GLOBAL_STATE_DIM = 7  

# GAMMA = 0.99
# LAMBDA = 0.95
# CLIP_EPS = 0.2
# TRAIN_EPOCHS = 10
# BATCH_SIZE = 32

# EXPLORATION_DISTANCE_THRESHOLD = 2.5  
# EXPLORATION_BONUS_FACTOR = 50.0         
# CLEARANCE_BONUS_FACTOR = 40.0           

# MEMORY_WINDOW_SIZE = 200

# TIMEOUT_THRESHOLD = 10.0  
# WAYPOINT_THRESHOLD = 2.5  

# SAME_CANDIDATE_THRESHOLD = 10

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

# def construir_grafo(nodos, umbral_conexion):
#     N = len(nodos)
#     grafo = {i: [] for i in range(N)}
#     for i in range(N):
#         for j in range(i+1, N):
#             d = distance(nodos[i], nodos[j])
#             if d <= umbral_conexion:
#                 grafo[i].append((j, d))
#                 grafo[j].append((i, d))
#     return grafo

# def quaternion_to_yaw(q: Quaternion):
#     siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
#     cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
#     return math.atan2(siny_cosp, cosy_cosp)

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
#         scores = self.attention_dense(lstm_out)  # (batch, timesteps, 1)
#         scores = tf.squeeze(scores, axis=-1)     # (batch, timesteps)
#         if mask is not None:
#             scores = tf.where(mask, scores, -1e9 * tf.ones_like(scores))
#         attention_weights = tf.nn.softmax(scores, axis=1)  # (batch, timesteps)
#         candidate_logits = self.logits_layer(lstm_out)      # (batch, timesteps, 1)
#         candidate_logits = tf.squeeze(candidate_logits, axis=-1)  # (batch, timesteps)
#         combined_logits = candidate_logits * attention_weights    # (batch, timesteps)
#         return combined_logits, (h, c)

# # ----------------------- Nodo de Entrenamiento End-to-End con Reset por Confirmación -----------------------
# class NavigationEndToEndTrainer(Node):
#     def __init__(self):
#         super().__init__('navigation_end2end_trainer')
#         qos_profile = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
#         # Subscripciones
#         self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
#         self.create_subscription(PoseArray, '/goal', self.goal_callback, 10)
#         self.create_subscription(PoseArray, '/filtered_navigation_nodes', self.filtered_nodes_callback, 10)
#         self.create_subscription(PoseArray, '/occupied_rejected_nodes', self.occupied_nodes_callback, 10)
#         self.create_subscription(PoseArray, '/obstacle_points', self.obstacle_points_callback, 10)
#         self.create_subscription(Bool, '/virtual_collision', self.collision_callback, 10)
#         self.create_subscription(Bool, '/map_reset', self.map_reset_callback, 10)
#         # Suscripción para reset_confirmation
#         self.create_subscription(Bool, '/reset_confirmation', self.reset_confirmation_callback, qos_profile)
#         self.reset_confirmation_received = False
#         self.reset_triggered = False

#         # Publicadores
#         self.marker_pub = self.create_publisher(Marker, '/planned_path_marker', 10)
#         self.nav_point = self.create_publisher(PoseArray, '/nav_point', 10)
#         self.reset_request_pub = self.create_publisher(Bool, '/reset_request', qos_profile)
#         self.goal_reached_pub = self.create_publisher(Bool, 'goal_reached', qos_profile)

#         # Estado interno
#         self.odom = None
#         self.goal = None
#         self.filtered_nodes = None
#         self.occupied_nodes = None
#         self.obstacle_points = None
#         self.virtual_collision = False

#         self.last_movement_time = None
#         self.movement_threshold = 0.01
#         self.inactivity_time_threshold = 300.0

#         self.last_reset_time = 0.0
#         self.reset_cooldown = 20.0

#         # Buffers de experiencia
#         self.actor_inputs = []
#         self.states = []
#         self.actions = []  
#         self.log_probs = []
#         self.rewards = []
#         self.values = []
#         self.dones = []
#         self.steps = 0
#         self.max_steps = 1000

#         self.episode_count = 0
#         self.total_episodes = 1000
#         self.start_time = time.time()
#         self.log_dir = "logs/fit/" + datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
#         self.summary_writer = tf.summary.create_file_writer(self.log_dir)
#         self.get_logger().info(f"TensorBoard logs en: {self.log_dir}")

#         self.models_saved = False

#         # Redes y optimizadores
#         self.actor = RecurrentActorWithAttention(MAX_CANDIDATES, FEATURE_DIM)
#         self.actor_state = None
#         self.actor_optimizer = tf.keras.optimizers.Adam(3e-4)
#         self.critic = tf.keras.Sequential([
#             tf.keras.layers.InputLayer(input_shape=(GLOBAL_STATE_DIM,)),
#             tf.keras.layers.Dense(64, activation='relu'),
#             tf.keras.layers.Dense(64, activation='relu'),
#             tf.keras.layers.Dense(64, activation='relu'),
#             tf.keras.layers.Dense(1)
#         ])
#         self.critic_optimizer = tf.keras.optimizers.Adam(3e-4)

#         # Máquina de estados: "IDLE" para planificar candidato; "MOVING" para avanzar.
#         self.state = "IDLE"
#         self.state_start_time = None
#         self.initial_position = None

#         self.TIMEOUT_THRESHOLD = TIMEOUT_THRESHOLD
#         self.WAYPOINT_THRESHOLD = WAYPOINT_THRESHOLD

#         self.current_waypoint = None

#         # Variables para detectar repetición de candidato.
#         self.last_candidate = None
#         self.same_candidate_count = 0
#         self.same_candidate_threshold = SAME_CANDIDATE_THRESHOLD

#         # Reset aleatorio: cada X episodios (número aleatorio entre 2 y 5).
#         self.reset_interval = random.randint(2, 5)

#         self.get_logger().info("Navigation End-to-End Trainer iniciado.")
#         while self.odom is None or self.goal is None:
#             self.get_logger().warn("Esperando /odom y /goal...")
#             rclpy.spin_once(self, timeout_sec=0.1)
#         self.get_logger().info("Datos iniciales recibidos, iniciando entrenamiento.")

#         self.reactive_timer = self.create_timer(0.1, self.reactive_step)
#         self.experience_timer = self.create_timer(0.5, self.experience_step)

#     # ----------------------- Callbacks -----------------------
#     def odom_callback(self, msg: Odometry):
#         current_time = self.get_clock().now().nanoseconds / 1e9
#         original_pose = msg.pose.pose
#         offset_x = 0.5
#         adjusted_pose = Pose()
#         adjusted_pose.position.x = original_pose.position.x - offset_x
#         adjusted_pose.position.y = original_pose.position.y
#         adjusted_pose.position.z = original_pose.position.z
#         adjusted_pose.orientation = original_pose.orientation
#         self.odom = adjusted_pose
#         linear = msg.twist.twist.linear
#         speed = math.hypot(linear.x, linear.y)
#         self.last_speed = speed
#         if self.last_movement_time is None or speed > self.movement_threshold:
#             self.last_movement_time = current_time

#     def goal_callback(self, msg: PoseArray):
#         if msg.poses:
#             self.goal = msg.poses[0]

#     def filtered_nodes_callback(self, msg: PoseArray):
#         self.filtered_nodes = msg

#     def occupied_nodes_callback(self, msg: PoseArray):
#         self.occupied_nodes = msg

#     def obstacle_points_callback(self, msg: PoseArray):
#         self.obstacle_points = msg

#     def collision_callback(self, msg: Bool):
#         self.virtual_collision = msg.data

#     def map_reset_callback(self, msg: Bool):
#         if msg.data:
#             self.get_logger().info("Se recibió señal para reiniciar el mapa.")

#     def reset_confirmation_callback(self, msg: Bool):
#         self.get_logger().info(f"Reset confirmation recibido: {msg.data}")
#         if msg.data:
#             self.reset_confirmation_received = True

#     # ----------------------- Solicitar Reinicio (con espera de confirmación) -----------------------
#     def request_environment_reset(self):
#         current_time = self.get_clock().now().nanoseconds / 1e9
#         if current_time - self.last_reset_time >= self.reset_cooldown:
#             reset_msg = Bool()
#             reset_msg.data = True
#             self.reset_request_pub.publish(reset_msg)
#             self.get_logger().info("Solicitud de reinicio enviada. Esperando confirmación...")
#             timeout = 5.0  # segundos
#             t_start = time.time()
#             while not self.reset_confirmation_received:
#                 rclpy.spin_once(self, timeout_sec=0.1)
#                 if time.time() - t_start > timeout:
#                     self.get_logger().warn("Timeout esperando confirmación de reset.")
#                     break
#             if self.reset_confirmation_received:
#                 self.get_logger().info("Reset confirmado. Finalizando episodio sin alcanzar la meta.")
#                 self.reset_triggered = True
#             else:
#                 self.get_logger().warn("No se recibió confirmación; continuando sin reset latched.")
#             self.reset_confirmation_received = False
#             self.last_reset_time = current_time
#         else:
#             self.get_logger().info("Cooldown activo; no se solicita reset.")

#     # ----------------------- Función de Recompensa -----------------------
#     def compute_reward(self, planned_waypoint):
#         current_pos = np.array([self.odom.position.x, self.odom.position.y])
#         goal_pos = np.array([self.goal.position.x, self.goal.position.y])
#         d_current = np.linalg.norm(current_pos - goal_pos)
#         wp = np.array([planned_waypoint.position.x, planned_waypoint.position.y])
#         d_final = np.linalg.norm(wp - goal_pos)
#         progress_reward = (d_current - d_final) * 20.0
#         safety_penalty = 0.0
#         if self.obstacle_points is not None and self.obstacle_points.poses:
#             d_list = [distance((wp[0], wp[1]), (p.position.x, p.position.y))
#                       for p in self.obstacle_points.poses]
#             if d_list:
#                 min_wp = min(d_list)
#                 if min_wp < 1.0:
#                     safety_penalty += (1.0 - min_wp) * 50.0
#         collision_penalty = 0.0
#         if self.occupied_nodes and self.occupied_nodes.poses:
#             for occ in self.occupied_nodes.poses:
#                 d = distance((self.odom.position.x, self.odom.position.y),
#                              (occ.position.x, occ.position.y))
#                 if d < 0.5:
#                     collision_penalty += 1000.0
#         # Si se detecta colisión virtual, se penaliza
#         if self.virtual_collision:
#             collision_penalty += 500.0
#         repetition_penalty = 0.0
#         if self.same_candidate_count >= self.same_candidate_threshold:
#             repetition_penalty = 20.0
#         reward = progress_reward - safety_penalty - collision_penalty - repetition_penalty + STEP_PENALTY
#         if d_current <= GOAL_REACHED_DIST:
#             reward += GOAL_REWARD
#         return reward

#     # ----------------------- Cálculo de Características para Candidatos -----------------------
#     def compute_candidate_features(self):
#         features = []
#         valid_nodes = []
#         if self.filtered_nodes is None or not self.filtered_nodes.poses:
#             return None, None, None
#         robot_x = self.odom.position.x
#         robot_y = self.odom.position.y
#         goal_x = self.goal.position.x
#         goal_y = self.goal.position.y
#         for node in self.filtered_nodes.poses:
#             if self.occupied_nodes and self.occupied_nodes.poses:
#                 clearance = min([distance((node.position.x, node.position.y),
#                                            (occ.position.x, occ.position.y))
#                                  for occ in self.occupied_nodes.poses])
#             else:
#                 clearance = 10.0
#             if clearance < OBSTACLE_PENALTY_DIST:
#                 continue
#             dist_robot = distance((robot_x, robot_y), (node.position.x, node.position.y))
#             dist_to_goal = distance((node.position.x, node.position.y), (goal_x, goal_y))
#             angle_diff = 0.0
#             feature_vector = [dist_robot, angle_diff, clearance, dist_to_goal, 0.0, 0.0]
#             features.append(feature_vector)
#             valid_nodes.append(node)
#         if len(features) == 0:
#             return None, None, None
#         features, valid_nodes = zip(*sorted(zip(features, valid_nodes), key=lambda x: x[0][3]))
#         features = list(features)
#         valid_nodes = list(valid_nodes)
#         if len(features) > MAX_CANDIDATES:
#             features = features[:MAX_CANDIDATES]
#             valid_nodes = valid_nodes[:MAX_CANDIDATES]
#         num_valid = len(features)
#         while len(features) < MAX_CANDIDATES:
#             features.append([0.0] * FEATURE_DIM)
#         features = np.array(features, dtype=np.float32)
#         mask = np.array([True] * num_valid + [False] * (MAX_CANDIDATES - num_valid))
#         return features, valid_nodes, mask

#     # ----------------------- Planificación Local Basada en filtered_nodes -----------------------
#     def plan_route(self, candidate):
#         if self.filtered_nodes is None or not self.filtered_nodes.poses:
#             candidate_pose = candidate
#             camino = [(self.odom.position.x, self.odom.position.y),
#                       (candidate.position.x, candidate.position.y)]
#             return candidate_pose, camino
#         current_pos = (self.odom.position.x, self.odom.position.y)
#         candidate_pos = (candidate.position.x, candidate.position.y)
#         nodos = [current_pos]
#         for node in self.filtered_nodes.poses:
#             nodos.append((node.position.x, node.position.y))
#         nodos.append(candidate_pos)
#         umbral_conexion = 3.0
#         grafo = construir_grafo(nodos, umbral_conexion)
#         inicio = 0
#         objetivo = len(nodos) - 1
#         indices_camino = dijkstra(grafo, inicio, objetivo)
#         camino_calculado = [nodos[i] for i in indices_camino]
#         if len(indices_camino) >= 2:
#             next_index = indices_camino[1]
#             next_waypoint = nodos[next_index]
#             new_pose = Pose()
#             new_pose.position.x = next_waypoint[0]
#             new_pose.position.y = next_waypoint[1]
#             new_pose.orientation = candidate.orientation
#             return new_pose, camino_calculado
#         else:
#             return candidate, camino_calculado

#     # ----------------------- Bucle Reactivo: Actualización Continua del Waypoint -----------------------
#     def reactive_step(self):
#         if self.odom is None or self.goal is None:
#             return
#         if self.state == "IDLE":
#             candidate_features, valid_nodes, mask = self.compute_candidate_features()
#             if candidate_features is None:
#                 self.get_logger().warn("No hay candidatos válidos.")
#                 return
#             self.last_candidate_features = (candidate_features, mask)
#             actor_input = np.expand_dims(candidate_features, axis=0)
#             mask_input = np.expand_dims(mask, axis=0)
#             logits, self.actor_state = self.actor(actor_input, mask=tf.convert_to_tensor(mask_input), initial_state=self.actor_state)
#             probs = tf.nn.softmax(logits, axis=-1).numpy()[0]
#             if np.ndim(probs) == 0:
#                 probs = np.array([probs])
#             action_index = int(np.argmax(probs))
#             if not mask[action_index]:
#                 self.get_logger().warn("Candidato seleccionado inválido.")
#                 return
#             new_candidate = valid_nodes[action_index]
#             if self.last_candidate is not None:
#                 d_same = math.hypot(new_candidate.position.x - self.last_candidate.position.x,
#                                     new_candidate.position.y - self.last_candidate.position.y)
#                 if d_same < 0.1:
#                     self.same_candidate_count += 1
#                 else:
#                     self.same_candidate_count = 0
#             else:
#                 self.same_candidate_count = 0
#             self.last_candidate = new_candidate
#             self.current_candidate = new_candidate
#             self.last_action_index = action_index
#             self.last_probs = probs
#             self.get_logger().info(f"Candidato seleccionado: ({new_candidate.position.x:.2f}, {new_candidate.position.y:.2f}), Repeticiones: {self.same_candidate_count}")
#             self.state = "MOVING"
#             self.state_start_time = self.get_clock().now().nanoseconds / 1e9
#         else:
#             if not hasattr(self, 'last_action_index'):
#                 self.last_action_index = 0
#             if not hasattr(self, 'last_probs'):
#                 self.last_probs = np.zeros(MAX_CANDIDATES)
#             if not hasattr(self, 'last_candidate_features'):
#                 candidate_features, valid_nodes, mask = self.compute_candidate_features()
#                 self.last_candidate_features = (candidate_features, mask)

#         planned_waypoint, computed_path = self.plan_route(self.current_candidate)
#         self.current_waypoint = planned_waypoint
#         nav_points = PoseArray()
#         nav_points.header.stamp = self.get_clock().now().to_msg()
#         nav_points.header.frame_id = "map"
#         nav_points.poses.append(planned_waypoint)
#         self.nav_point.publish(nav_points)
#         marker = Marker()
#         marker.header.frame_id = "map"
#         marker.header.stamp = self.get_clock().now().to_msg()
#         marker.ns = "planned_path"
#         marker.id = 0
#         marker.type = Marker.LINE_STRIP
#         marker.action = Marker.ADD
#         marker.scale.x = 0.1
#         marker.color.a = 1.0
#         marker.color.r = 0.0
#         marker.color.g = 1.0
#         marker.color.b = 0.0
#         marker.points = [Point(x=pt[0], y=pt[1], z=0.0) for pt in computed_path]
#         self.marker_pub.publish(marker)

#     # ----------------------- Bucle de Experiencia: Registro del Final del Episodio -----------------------
#     def experience_step(self):
#         # Si se ha solicitado reset (confirmado), finalizamos el episodio inmediatamente.
#         if self.reset_triggered:
#             self.get_logger().info("Episodio finalizado por reset.")
#             self.goal_reached_pub.publish(Bool(data=False))  # False: reset sin alcanzar goal.
#             self.update_model()
#             self.reset_triggered = False
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
#             return

#         # Comprobamos si se ha alcanzado el goal usando odom y goal.
#         if self.odom and self.goal and distance((self.odom.position.x, self.odom.position.y),
#                                                  (self.goal.position.x, self.goal.position.y)) < GOAL_REACHED_DIST:
#             self.get_logger().info("Goal alcanzado. Publicando goal_reached = True.")
#             self.goal_reached_pub.publish(Bool(data=True))
#             done = 1
#         else:
#             done = 0

#         # Finalización del episodio si se alcanza el goal o se supera el máximo de steps.
#         if done == 1 or self.steps >= self.max_steps:
#             self.get_logger().info(f"Episodio terminado en {self.steps} pasos.")
#             self.update_model()
#             if self.episode_count > 0 and self.episode_count % self.reset_interval == 0:
#                 self.request_environment_reset()
#                 self.reset_interval = random.randint(2, 5)
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
#             return

#         # Si estamos en MOVING, se registra experiencia cuando se alcanza el waypoint.
#         if self.state == "MOVING":
#             if distance((self.odom.position.x, self.odom.position.y),
#                         (self.current_waypoint.position.x, self.current_waypoint.position.y)) < self.WAYPOINT_THRESHOLD:
#                 self.get_logger().info("Waypoint alcanzado. Registrando experiencia.")
#                 reward = self.compute_reward(self.current_waypoint)
#                 if self.same_candidate_count >= self.same_candidate_threshold:
#                     self.get_logger().warn("Repetición excesiva de candidato. Penalizando.")
#                     reward -= 20.0
#                     self.same_candidate_count = 0
#                 global_state = np.array([
#                     self.odom.position.x,
#                     self.odom.position.y,
#                     self.goal.position.x,
#                     self.goal.position.y,
#                     0.0, 0.0, 0.0
#                 ], dtype=np.float32)
#                 value = self.critic(tf.convert_to_tensor([global_state], dtype=tf.float32)).numpy()[0]
#                 self.states.append(global_state)
#                 self.actions.append(self.last_action_index)
#                 self.log_probs.append(np.log(self.last_probs[self.last_action_index] + 1e-8))
#                 self.values.append(value)
#                 self.rewards.append(reward)
#                 self.dones.append(done)
#                 self.actor_inputs.append(self.last_candidate_features)
#                 self.steps += 1
#                 self.current_candidate = None
#                 self.state = "IDLE"
#             else:
#                 current_time = self.get_clock().now().nanoseconds / 1e9
#                 if current_time - self.state_start_time > self.TIMEOUT_THRESHOLD:
#                     self.get_logger().warn("Timeout en MOVING. Penalizando y reiniciando candidato.")
#                     penalty_reward = -10.0
#                     global_state = np.array([
#                         self.odom.position.x,
#                         self.odom.position.y,
#                         self.goal.position.x,
#                         self.goal.position.y,
#                         0.0, 0.0, 0.0
#                     ], dtype=np.float32)
#                     value = self.critic(tf.convert_to_tensor([global_state], dtype=tf.float32)).numpy()[0]
#                     self.states.append(global_state)
#                     self.actions.append(self.last_action_index)
#                     self.log_probs.append(np.log(self.last_probs[self.last_action_index] + 1e-8))
#                     self.values.append(value)
#                     self.rewards.append(penalty_reward)
#                     self.dones.append(done)
#                     self.actor_inputs.append(self.last_candidate_features)
#                     self.steps += 1
#                     self.request_environment_reset()
#                     self.state = "IDLE"

#         if len(self.states) > MEMORY_WINDOW_SIZE:
#             self.states.pop(0)
#             self.actions.pop(0)
#             self.log_probs.pop(0)
#             self.values.pop(0)
#             self.rewards.pop(0)
#             self.dones.pop(0)
#             self.actor_inputs.pop(0)

#     # ----------------------- Cálculo de Ventajas y Actualización del Modelo -----------------------
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
#                     batch_actions = tf.cast(batch_actions, tf.int32)
#                     indices = tf.stack([tf.range(tf.shape(logits)[0]), batch_actions], axis=1)
#                     new_log_probs = tf.math.log(tf.nn.softmax(logits, axis=-1) + 1e-8)
#                     new_log_probs = tf.gather_nd(new_log_probs, indices)
#                     ratio = tf.exp(new_log_probs - batch_old_log_probs)
#                     clipped_ratio = tf.clip_by_value(ratio, 1 - CLIP_EPS, 1 + CLIP_EPS)
#                     actor_loss = -tf.reduce_mean(tf.minimum(ratio * batch_advantages, clipped_ratio * batch_advantages))
#                     values_pred = self.critic(batch_states)
#                     critic_loss = tf.reduce_mean(tf.square(batch_returns - values_pred))
#                     total_loss = actor_loss + 0.5 * critic_loss
#                 actor_grads = tape.gradient(total_loss, self.actor.trainable_variables)
#                 critic_grads = tape.gradient(total_loss, self.critic.trainable_variables)
#                 self.actor_optimizer.apply_gradients(zip(actor_grads, self.actor.trainable_variables))
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
#         if self.episode_count >= 400:
#             self.get_logger().info("400 episodios completados. Finalizando nodo.")
#             rclpy.shutdown()

#     def save_models(self):
#         timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
#         actor_model_filename = f"actor_model_{timestamp}.keras"
#         critic_model_filename = f"critic_model_{timestamp}.keras"
#         self.actor.save(actor_model_filename)
#         self.critic.save(critic_model_filename)
#         self.get_logger().info(f"Modelos guardados: {actor_model_filename} y {critic_model_filename}.")

# def main(args=None):
#     rclpy.init(args=args)
#     node = NavigationEndToEndTrainer()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
