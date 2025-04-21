# # #!/usr/bin/env python3
# # import rclpy
# # from rclpy.node import Node
# # import math
# # import numpy as np
# # import cv2
# # import time
# # import threading
# # import heapq
# # import tensorflow as tf
# # from tensorflow.keras import layers, models, optimizers

# # # Mensajes de ROS2
# # from nav_msgs.msg import Odometry, OccupancyGrid, Path, MapMetaData
# # from geometry_msgs.msg import PoseArray, Twist, Pose,PoseStamped

# # ###############################
# # # Funciones auxiliares y planning plain
# # ###############################

# # def distance(p1, p2):
# #     return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

# # def dijkstra(graph, start, goal):
# #     dist = {node: float('inf') for node in graph.keys()}
# #     prev = {node: None for node in graph.keys()}
# #     dist[start] = 0
# #     queue = [(0, start)]
# #     while queue:
# #         current_dist, current_node = heapq.heappop(queue)
# #         if current_node == goal:
# #             break
# #         if current_dist > dist[current_node]:
# #             continue
# #         for neighbor, weight in graph[current_node]:
# #             alt = current_dist + weight
# #             if alt < dist[neighbor]:
# #                 dist[neighbor] = alt
# #                 prev[neighbor] = current_node
# #                 heapq.heappush(queue, (alt, neighbor))
# #     path = []
# #     node = goal
# #     while node is not None:
# #         path.append(node)
# #         node = prev[node]
# #     path.reverse()
# #     return path

# # def index_to_world(i, j, info):
# #     x = info.origin.position.x + (i + 0.5) * info.resolution
# #     y = info.origin.position.y + (j + 0.5) * info.resolution
# #     return (x, y)

# # ###############################
# # # Agente DRL (PPO basado con CNN)
# # ###############################

# # class PPOAgent:
# #     def __init__(self, num_waypoints=10, input_shape=(64, 64, 2), pos_dim=2):
# #         self.num_waypoints = num_waypoints
# #         self.actor = self.build_actor(num_waypoints, input_shape, pos_dim)
# #         self.critic = self.build_critic(input_shape, pos_dim)
# #         self.actor_optimizer = optimizers.Adam(learning_rate=1e-4)
# #         self.critic_optimizer = optimizers.Adam(learning_rate=1e-3)
# #         self.gamma = 0.99

# #     def build_actor(self, num_waypoints, input_shape, pos_dim):
# #         grid_input = layers.Input(shape=input_shape, name='grid_input')
# #         x = layers.Conv2D(16, kernel_size=3, padding='same', activation='relu')(grid_input)
# #         x = layers.MaxPooling2D(2)(x)
# #         x = layers.Conv2D(32, kernel_size=3, padding='same', activation='relu')(x)
# #         x = layers.MaxPooling2D(2)(x)
# #         x = layers.Conv2D(64, kernel_size=3, padding='same', activation='relu')(x)
# #         x = layers.MaxPooling2D(2)(x)
# #         x = layers.GlobalAveragePooling2D()(x)
        
# #         goal_input = layers.Input(shape=(pos_dim,), name='goal_input')
# #         goal_features = layers.Dense(32, activation='relu')(goal_input)
        
# #         robot_input = layers.Input(shape=(pos_dim,), name='robot_input')
# #         robot_features = layers.Dense(32, activation='relu')(robot_input)
        
# #         concat = layers.Concatenate()([x, goal_features, robot_features])
# #         fc = layers.Dense(256, activation='relu')(concat)
# #         output = layers.Dense(num_waypoints * 2, activation='sigmoid')(fc)
# #         outputs = layers.Reshape((num_waypoints, 2))(output)
# #         model = models.Model(inputs=[grid_input, goal_input, robot_input], outputs=outputs)
# #         return model

# #     def build_critic(self, input_shape, pos_dim):
# #         grid_input = layers.Input(shape=input_shape, name='grid_input')
# #         x = layers.Conv2D(16, kernel_size=3, padding='same', activation='relu')(grid_input)
# #         x = layers.MaxPooling2D(2)(x)
# #         x = layers.Conv2D(32, kernel_size=3, padding='same', activation='relu')(x)
# #         x = layers.MaxPooling2D(2)(x)
# #         x = layers.Conv2D(64, kernel_size=3, padding='same', activation='relu')(x)
# #         x = layers.MaxPooling2D(2)(x)
# #         x = layers.GlobalAveragePooling2D()(x)
        
# #         goal_input = layers.Input(shape=(pos_dim,), name='goal_input')
# #         goal_features = layers.Dense(32, activation='relu')(goal_input)
        
# #         robot_input = layers.Input(shape=(pos_dim,), name='robot_input')
# #         robot_features = layers.Dense(32, activation='relu')(robot_input)
        
# #         concat = layers.Concatenate()([x, goal_features, robot_features])
# #         fc = layers.Dense(256, activation='relu')(concat)
# #         value = layers.Dense(1)(fc)
# #         model = models.Model(inputs=[grid_input, goal_input, robot_input], outputs=value)
# #         return model

# #     def get_action(self, state):
# #         grid, goal, robot = state
# #         action = self.actor([grid, goal, robot], training=False)
# #         return action

# #     def compute_advantages(self, rewards, values, dones):
# #         advantages = []
# #         gae = 0
# #         next_value = 0
# #         for r, v, done in zip(reversed(rewards), reversed(values), reversed(dones)):
# #             delta = r + self.gamma * next_value * (1 - done) - v
# #             gae = delta + self.gamma * 0.95 * (1 - done) * gae
# #             advantages.insert(0, gae)
# #             next_value = v
# #         return advantages



# #     def ppo_update(self, states, actions, rewards, dones):
# #         grid, goal, robot = states
# #         values = self.critic([grid, goal, robot], training=False).numpy().flatten()
# #         advantages = self.compute_advantages(rewards, values, dones)
# #         advantages = np.array(advantages, dtype=np.float32)
# #         # Expandir para hacer broadcast: (batch, 1, 1)
# #         advantages = tf.convert_to_tensor(advantages.reshape(-1, 1, 1), dtype=tf.float32)
# #         targets = advantages + values
# #         actions = tf.convert_to_tensor(actions, dtype=tf.float32)
        
# #         # Actualización simplificada (sin usar ratios ni clipping para simplificar)
# #         for _ in range(10):
# #             with tf.GradientTape() as tape_actor, tf.GradientTape() as tape_critic:
# #                 new_actions = self.actor([grid, goal, robot], training=True)
# #                 actor_loss = tf.reduce_mean(tf.square(new_actions - actions) * advantages)
# #                 new_values = self.critic([grid, goal, robot], training=True)
# #                 critic_loss = tf.reduce_mean(tf.square(targets - new_values))
# #             grads_actor = tape_actor.gradient(actor_loss, self.actor.trainable_variables)
# #             grads_critic = tape_critic.gradient(critic_loss, self.critic.trainable_variables)
# #             self.actor_optimizer.apply_gradients(zip(grads_actor, self.actor.trainable_variables))
# #             self.critic_optimizer.apply_gradients(zip(grads_critic, self.critic.trainable_variables))

# # ###############################
# # # Nodo de entrenamiento e inferencia
# # ###############################

# # class RLNavigationNode(Node):
# #     def __init__(self):
# #         super().__init__('rl_navigation_node')
# #         # Suscripciones a tópicos
# #         self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
# #         self.create_subscription(PoseArray, '/goal', self.goal_callback, 10)
# #         self.create_subscription(OccupancyGrid, '/occupancy_grid', self.current_grid_callback, 10)
# #         self.create_subscription(OccupancyGrid, '/persistent_dynamic_occupancy_grid', self.memory_grid_callback, 10)
# #         # Publicadores
# #         self.path_pub = self.create_publisher(Path, '/global_path_training', 10)
# #         self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
# #         # Variables de estado
# #         self.odom = None
# #         self.goal = None  # Se espera un Pose en el PoseArray
# #         self.current_grid = None
# #         self.memory_grid = None
# #         self.current_global_path = None  # Path en coordenadas reales
# #         self.training_buffer = []
        
# #         # Parámetros para pure pursuit y reward
# #         self.lookahead_distance = 1.0
# #         self.linear_speed = 0.5
# #         self.k_pursuit = 2.0
# #         self.waypoint_threshold = 2.0
# #         self.current_path_index = 0
# #         self.connection_threshold = 2.0  # Distancia máxima para considerar un waypoint alcanzado


# #         # Inicializar el agente DRL
# #         self.agent = PPOAgent(num_waypoints=10)
        
# #         # Publicación continua de Twist (en un hilo)
# #         self.last_twist = Twist()
# #         self.twist_pub_rate = 20
# #         threading.Thread(target=self.publish_twist_continuously, daemon=True).start()
        
# #         # Timer para control loop
# #         self.create_timer(0.1, self.control_loop)
# #         # Hilo para actualizaciones de DRL
# #         threading.Thread(target=self.training_loop, daemon=True).start()
        
# #         self.get_logger().info("Nodo RL Navigation iniciado.")

# #     # Callbacks de suscripción
# #     def odom_callback(self, msg: Odometry):
# #         self.odom = msg.pose.pose

# #     def goal_callback(self, msg: PoseArray):
# #         if msg.poses:
# #             self.goal = msg.poses[0]
# #             self.get_logger().info("Goal recibido.")

# #     def current_grid_callback(self, msg: OccupancyGrid):
# #         self.current_grid = msg

# #     def memory_grid_callback(self, msg: OccupancyGrid):
# #         self.memory_grid = msg

# #     # Publicar Twist continuamente
# #     def publish_twist_continuously(self):
# #         rate = 1.0 / self.twist_pub_rate
# #         while rclpy.ok():
# #             self.cmd_vel_pub.publish(self.last_twist)
# #             time.sleep(rate)

# #     # Preprocesar el grid: redimensiona a 64x64 y crea 2 canales (analizando el entorno)
# #     def preprocess_grid(self, grid_msg):
# #         info = grid_msg.info
# #         grid = np.array(grid_msg.data, dtype=np.int8).reshape((info.height, info.width))
# #         # Asignar valor 1 para obstáculo confirmado, 0 para libre y 0.5 para desconocido (-1)
# #         grid = np.where(grid == 100, 1, grid)          # obstáculo
# #         grid = np.where(grid == -1, 0.5, grid)           # desconocido
# #         grid = np.where(grid == 0, 0, grid)              # libre
# #         grid_resized = cv2.resize(grid, (64, 64), interpolation=cv2.INTER_NEAREST)
# #         grid_resized = np.expand_dims(grid_resized, axis=-1)
# #         grid_resized = np.repeat(grid_resized, 2, axis=-1)
# #         grid_resized = grid_resized.astype(np.float32)
# #         return np.expand_dims(grid_resized, axis=0)


# #     # Normalizar posición (Pose) usando información del mapa
# #     def get_normalized_position(self, pos, info):
# #         map_width = info.width * info.resolution
# #         map_height = info.height * info.resolution
# #         norm_x = (pos.position.x - info.origin.position.x) / map_width
# #         norm_y = (pos.position.y - info.origin.position.y) / map_height
# #         return np.array([[norm_x, norm_y]], dtype=np.float32)

# #     # Obtener estado para el agente: (grid, goal, robot)
# #     def get_state(self):
# #         if self.current_grid is None or self.odom is None or self.goal is None:
# #             return None
# #         grid_tensor = self.preprocess_grid(self.current_grid)
# #         info = self.current_grid.info
# #         robot_pos = self.get_normalized_position(self.odom, info)
# #         goal_pos = self.get_normalized_position(self.goal, info)
# #         return grid_tensor, goal_pos, robot_pos

# #     # Convertir la salida del agente (waypoints normalizados) a coordenadas reales
# #     def convert_predictions_to_world(self, predictions, info):
# #         predictions = predictions.numpy()[0]  # forma: (num_waypoints, 2)
# #         map_width = info.width * info.resolution
# #         map_height = info.height * info.resolution
# #         origin_x = info.origin.position.x
# #         origin_y = info.origin.position.y
# #         path = []
# #         for wp in predictions:
# #             x = origin_x + wp[0] * map_width
# #             y = origin_y + wp[1] * map_height
# #             path.append((x, y))
# #         return path

# #     # Publicar el path para visualizar en RViz
# #     def publish_path(self, path):
# #         path_msg = Path()
# #         path_msg.header.stamp = self.get_clock().now().to_msg()
# #         path_msg.header.frame_id = "map"
# #         for pt in path:
# #             pose_st = PoseStamped()
# #             pose_st.header = path_msg.header
# #             pose_st.pose.position.x = pt[0]
# #             pose_st.pose.position.y = pt[1]
# #             pose_st.pose.position.z = 0.0
# #             pose_st.pose.orientation.w = 1.0
# #             path_msg.poses.append(pose_st)
# #         self.path_pub.publish(path_msg)
# #         self.get_logger().info(f"Path publicado con {len(path)} puntos.")

# #     # Controlador Pure Pursuit basado en el path
# #     def pure_pursuit_control(self, path, current_pos, current_yaw):
# #         target_point = None
# #         for pt in path:
# #             if distance(current_pos, pt) >= self.lookahead_distance:
# #                 target_point = pt
# #                 break
# #         if target_point is None:
# #             target_point = path[-1]
# #         desired_heading = math.atan2(target_point[1] - current_pos[1],
# #                                      target_point[0] - current_pos[0])
# #         error_angle = desired_heading - current_yaw
# #         error_angle = math.atan2(math.sin(error_angle), math.cos(error_angle))
# #         twist = Twist()
# #         twist.linear.x = self.linear_speed
# #         twist.angular.z = self.k_pursuit * error_angle
# #         return twist

# #     # Función de recompensa: premia alcanzar waypoints y el goal, penaliza paths largos
# #     def compute_reward(self):
# #         if self.odom is None or self.goal is None or self.current_grid is None:
# #             return 0, 0
# #         info = self.current_grid.info
# #         robot_x, robot_y = self.odom.position.x, self.odom.position.y
# #         goal_x, goal_y = self.goal.position.x, self.goal.position.y
# #         dist_to_goal = distance((robot_x, robot_y), (goal_x, goal_y))
# #         reward = -dist_to_goal  # penalización base
# #         done = 0
# #         if self.current_global_path is not None and len(self.current_global_path) > 0:
# #             next_wp = self.current_global_path[0]
# #             d_wp = distance((robot_x, robot_y), next_wp)
# #             if d_wp < self.waypoint_threshold:
# #                 reward += 1.0  # recompensa intermedia
# #                 self.current_global_path.pop(0)
# #         if dist_to_goal < 0.5:
# #             reward += 10.0  # recompensa grande por alcanzar el goal
# #             done = 1
# #         if self.current_global_path is not None and len(self.current_global_path) > 1:
# #             path_length = 0
# #             pts = self.current_global_path
# #             for i in range(len(pts)-1):
# #                 path_length += distance(pts[i], pts[i+1])
# #             direct = distance((robot_x, robot_y), (goal_x, goal_y))
# #             if path_length > 1.5 * direct:
# #                 reward -= 2.0
# #         return reward, done
    
# #     def is_line_free(self, fused_grid, info, start, end):
# #         def world_to_index(point, info):
# #             i = int((point[0] - info.origin.position.x) / info.resolution)
# #             j = int((point[1] - info.origin.position.y) / info.resolution)
# #             return i, j

# #         x0, y0 = world_to_index(start, info)
# #         x1, y1 = world_to_index(end, info)

# #         dx = abs(x1 - x0)
# #         dy = abs(y1 - y0)
# #         x, y = x0, y0
# #         sx = -1 if x0 > x1 else 1
# #         sy = -1 if y0 > y1 else 1

# #         # Función auxiliar para verificar límites
# #         def is_in_bounds(x, y, grid):
# #             return 0 <= x < grid.shape[1] and 0 <= y < grid.shape[0]

# #         if dx > dy:
# #             err = dx / 2.0
# #             while x != x1:
# #                 if not is_in_bounds(x, y, fused_grid) or fused_grid[y, x] == 100:
# #                     return False
# #                 err -= dy
# #                 if err < 0:
# #                     y += sy
# #                     err += dx
# #                 x += sx
# #         else:
# #             err = dy / 2.0
# #             while y != y1:
# #                 if not is_in_bounds(x, y, fused_grid) or fused_grid[y, x] == 100:
# #                     return False
# #                 err -= dx
# #                 if err < 0:
# #                     x += sx
# #                     err += dy
# #                 y += sy
# #         if not is_in_bounds(x, y, fused_grid) or fused_grid[y, x] == 100:
# #             return False
# #         return True

# #     def update_path_index(self, current_pos):
# #         if self.current_global_path is None or len(self.current_global_path) == 0:
# #             return
# #         # Mientras la distancia al waypoint actual sea menor que el umbral, avanzar el índice.
# #         while (self.current_path_index < len(self.current_global_path) and 
# #                distance(current_pos, self.current_global_path[self.current_path_index]) < self.waypoint_threshold):
# #             self.get_logger().info(f"Waypoint {self.current_path_index} alcanzado (distancia: {distance(current_pos, self.current_global_path[self.current_path_index]):.2f} m).")
# #             self.current_path_index += 1
# #         # Si se ha alcanzado el último waypoint, ajusta el índice para no salirse del rango.
# #         if self.current_path_index >= len(self.current_global_path):
# #             self.current_path_index = len(self.current_global_path) - 1

# #     def is_path_safe(self, path, fused_grid, info):
# #         # Revisa cada segmento del path para asegurar que la línea entre ellos esté libre.
# #         for i in range(len(path) - 1):
# #             if not self.is_line_free(fused_grid, info, path[i], path[i+1]):
# #                 self.get_logger().warn(f"Segmento inseguro entre {path[i]} y {path[i+1]}.")
# #                 return False
# #         return True

# #     def should_replan(self, current_pos, fused_grid, info):
# #         # Replanifica solo si:
# #         # 1. No hay path generado, o
# #         # 2. El path actual es inseguro (algún segmento cruza un obstáculo), o
# #         # 3. El robot se ha desviado excesivamente del waypoint actual.
# #         if self.current_global_path is None or len(self.current_global_path) == 0:
# #             return True
# #         if not self.is_path_safe(self.current_global_path, fused_grid, info):
# #             self.get_logger().info("Path inseguro detectado, se requiere replanteo.")
# #             return True
# #         current_wp = self.current_global_path[self.current_path_index]
# #         if distance(current_pos, current_wp) > self.connection_threshold * 2:
# #             self.get_logger().info("Desviación excesiva, se forzará replanteo.")
# #             return True
# #         return False

    


# #         # Ciclo de entrenamiento/inferencia y control
# #     def training_loop(self):
# #         while rclpy.ok():
# #             state = self.get_state()
# #             if state is None:
# #                 time.sleep(0.1)
# #                 continue
# #             # Obtener acción (path) del agente DRL
# #             action = self.agent.get_action(state)
# #             info = self.current_grid.info
# #             path_world = self.convert_predictions_to_world(action, info)
            
# #             # Forzar la conectividad: el primer waypoint es la posición actual del robot
# #             # y el último es la posición del goal.
# #             robot_world = (self.odom.position.x, self.odom.position.y)
# #             goal_world = (self.goal.position.x, self.goal.position.y)
# #             path_world[0] = robot_world
# #             path_world[-1] = goal_world

# #             self.current_global_path = path_world.copy()
# #             self.publish_path(path_world)
# #             # Calcular el control pure pursuit para avanzar
# #             current_pos = (self.odom.position.x, self.odom.position.y)
# #             q = self.odom.orientation
# #             siny = 2.0 * (q.w * q.z + q.x * q.y)
# #             cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
# #             current_yaw = math.atan2(siny, cosy)
# #             twist = self.pure_pursuit_control(path_world, current_pos, current_yaw)
# #             self.last_twist = twist
# #             time.sleep(0.1)
# #             reward, done = self.compute_reward()
# #             next_state = self.get_state()
# #             self.training_buffer.append((state, action.numpy(), reward, next_state, done))
# #             if len(self.training_buffer) >= 32:
# #                 batch = self.training_buffer[:32]
# #                 self.training_buffer = self.training_buffer[32:]
# #                 grids = np.concatenate([b[0][0] for b in batch], axis=0)
# #                 goals = np.concatenate([b[0][1] for b in batch], axis=0)
# #                 robots = np.concatenate([b[0][2] for b in batch], axis=0)
# #                 actions_batch = np.array([b[1] for b in batch])
# #                 rewards = np.array([b[2] for b in batch])
# #                 dones = np.array([b[4] for b in batch])
# #                 self.agent.ppo_update((grids, goals, robots), actions_batch, rewards, dones)
# #             if done:
# #                 self.get_logger().info("Goal alcanzado. Reiniciando episodio.")
# #                 # Aquí podrías reinicializar variables o forzar un nuevo episodio.
# #             time.sleep(0.1)

# #     def fallback_path(self, current_pos, goal_pos, fused_grid, info, step_size=0.2, offset_step=0.3, max_offset=1.0):
# #             """
# #             Genera un path incremental entre current_pos y goal_pos en pasos de step_size.
# #             Si algún segmento no es seguro (is_line_free falla), se intenta desplazar el punto
# #             intermedio en dirección perpendicular hasta encontrar una posición segura.
# #             """
# #             # Generar la línea base (recta) con varios puntos
# #             path = []
# #             total_dist = distance(current_pos, goal_pos)
# #             num_steps = int(total_dist / step_size)
# #             if num_steps < 1:
# #                 num_steps = 1
# #             for i in range(num_steps + 1):
# #                 ratio = i / float(num_steps)
# #                 x = current_pos[0] + ratio * (goal_pos[0] - current_pos[0])
# #                 y = current_pos[1] + ratio * (goal_pos[1] - current_pos[1])
# #                 path.append((x, y))
            
# #             # Ahora ajustar los puntos intermedios si el segmento no es seguro
# #             safe_path = [path[0]]
# #             for i in range(1, len(path)):
# #                 prev_pt = safe_path[-1]
# #                 pt = path[i]
# #                 # Verificar si el segmento es seguro
# #                 if self.is_line_free(fused_grid, info, prev_pt, pt):
# #                     safe_path.append(pt)
# #                 else:
# #                     # Calcular la dirección del segmento y su perpendicular
# #                     dx = pt[0] - prev_pt[0]
# #                     dy = pt[1] - prev_pt[1]
# #                     theta = math.atan2(dy, dx)
# #                     perp_angle = theta + math.pi / 2  # dirección perpendicular
                    
# #                     found_safe = False
# #                     # Probar a desplazar en ambas direcciones
# #                     for sign in [1, -1]:
# #                         offset = offset_step
# #                         while offset <= max_offset:
# #                             new_pt = (pt[0] + sign * offset * math.cos(perp_angle),
# #                                     pt[1] + sign * offset * math.sin(perp_angle))
# #                             # Verificar si el segmento modificado es seguro
# #                             if self.is_line_free(fused_grid, info, prev_pt, new_pt):
# #                                 safe_path.append(new_pt)
# #                                 found_safe = True
# #                                 break
# #                             offset += offset_step
# #                         if found_safe:
# #                             break
# #                     # Si no se encontró ninguna alternativa segura, se añade el punto original
# #                     if not found_safe:
# #                         safe_path.append(pt)
# #             return safe_path
    
# #     def plan_path(self, nodes, fused_grid, info):
# #         if self.odom is None or self.goal is None:
# #             return None
# #         robot_pos = (self.odom.position.x, self.odom.position.y)
# #         goal_pos = (self.goal.position.x, self.goal.position.y)
# #         if self.current_grid is not None:
# #             goal_pos = self.project_goal(goal_pos, self.current_grid.info)
# #         elif self.memory_grid is not None:
# #             goal_pos = self.project_goal(goal_pos, self.memory_grid.info)
        
# #         all_nodes = nodes.copy()
# #         all_nodes.append(robot_pos)
# #         all_nodes.append(goal_pos)
# #         robot_index = len(all_nodes) - 2
# #         goal_index = len(all_nodes) - 1
        
# #         graph = {i: [] for i in range(len(all_nodes))}
# #         for i in range(len(all_nodes)):
# #             for j in range(i + 1, len(all_nodes)):
# #                 d = distance(all_nodes[i], all_nodes[j])
# #                 if d <= self.connection_threshold:
# #                     if self.is_line_free(fused_grid, info, all_nodes[i], all_nodes[j]):
# #                         graph[i].append((j, d))
# #                         graph[j].append((i, d))
# #         path_indices = dijkstra(graph, robot_index, goal_index)
# #         if not path_indices or len(path_indices) < 3:
# #             self.get_logger().warn("Path global insuficiente (solo 2 puntos). Forzando RRT.")
# #             return None  # Se fuerza replan usando otro método
# #         path = [all_nodes[i] for i in path_indices]
        
# #         # Verificar que cada segmento del path sea seguro
# #         for i in range(len(path) - 1):
# #             if not self.is_line_free(fused_grid, info, path[i], path[i+1]):
# #                 self.get_logger().warn(f"Segmento inseguro entre {path[i]} y {path[i+1]}. Forzando replan.")
# #                 return None
# #         return path
    
# #     def project_goal(self, goal_pos, info):
# #         ox = info.origin.position.x
# #         oy = info.origin.position.y
# #         max_x = ox + info.width * info.resolution
# #         max_y = oy + info.height * info.resolution
# #         x, y = goal_pos
# #         x = min(max(x, ox), max_x)
# #         y = min(max(y, oy), max_y)
# #         return (x, y)


# #     def extract_nodes(self, fused_grid, info):
# #         nodes = []
# #         for j in range(fused_grid.shape[0]):
# #             for i in range(fused_grid.shape[1]):
# #                 if fused_grid[j, i] == 0:
# #                     nodes.append(index_to_world(i, j, info))
# #         return nodes
    
# #     def control_loop(self):
# #         if self.odom is None or self.goal is None:
# #             return

# #         fused_result = self.fuse_maps_dynamic(self.current_grid, self.memory_grid)
# #         if fused_result is None or fused_result[0] is None:
# #             self.get_logger().warn("No se pudo fusionar mapas. Esperando datos válidos...")
# #             return
# #         fused_grid, info = fused_result
# #         current_pos = (self.odom.position.x, self.odom.position.y)
# #         current_time = self.get_clock().now().nanoseconds / 1e9

# #         # Replanificación global: si no existe path o se fuerza replanteo
# #         if self.current_global_path is None or self.should_replan(current_pos):
# #             nodes = self.extract_nodes(fused_grid, info)
# #             new_path = self.plan_path(nodes, fused_grid, info)
# #             if new_path is None or len(new_path) < 3:
# #                 self.get_logger().warn("Planificación convencional devolvió un path insuficiente (línea recta). Forzando RRT.")
# #                 new_path = self.rrt_plan_path(current_pos, (self.goal.position.x, self.goal.position.y), fused_grid, info)
# #                 if new_path is None or len(new_path) < 3:
# #                     self.get_logger().warn("RRT falló, usando fallback path incremental.")
# #                     new_path = self.fallback_path(current_pos, (self.goal.position.x, self.goal.position.y), fused_grid, info)
# #                     if new_path is None or len(new_path) < 3:
# #                         self.get_logger().error("Fallback path también falló, abortando.")
# #                         return
# #             new_path = self.smooth_path(new_path)
# #             self.current_global_path = new_path
# #             self.current_path_index = 0
# #             self.training_buffer.append({
# #                 'time': current_time,
# #                 'global_path': new_path,
# #                 'position': current_pos,
# #             })
        
# #         # Actualizar índice del path conforme se alcanzan waypoints
# #         self.update_path_index(current_pos)
# #         effective_path = self.current_global_path[self.current_path_index:] if self.current_global_path else None

# #         # Verificar que el effective_path sea seguro. Si no lo es, forzar replanificación.
# #         if effective_path is None or len(effective_path) < 2 or not self.check_effective_path_safety(effective_path, fused_grid, info):
# #             self.get_logger().warn("Effective path inseguro, forzando replanificación.")
# #             nodes = self.extract_nodes(fused_grid, info)
# #             new_path = self.plan_path(nodes, fused_grid, info)
# #             if new_path is None or len(new_path) < 3:
# #                 self.get_logger().warn("Planificación convencional insuficiente. Forzando RRT.")
# #                 new_path = self.rrt_plan_path(current_pos, (self.goal.position.x, self.goal.position.y), fused_grid, info)
# #                 if new_path is None or len(new_path) < 3:
# #                     self.get_logger().warn("RRT falló, usando fallback path incremental.")
# #                     new_path = self.fallback_path(current_pos, (self.goal.position.x, self.goal.position.y), fused_grid, info)
# #                     if new_path is None or len(new_path) < 3:
# #                         self.get_logger().error("Fallback path también falló, abortando.")
# #                         return
# #             new_path = self.smooth_path(new_path)
# #             self.current_global_path = new_path
# #             self.current_path_index = 0
# #             effective_path = new_path  # Nuevo effective path

# #         self.publish_path(effective_path)

# #         # Calcular velocidades dinámicas
# #         q = self.odom.orientation
# #         siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
# #         cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
# #         current_yaw = math.atan2(siny_cosp, cosy_cosp)
# #         linear_speed, angular_speed = self.compute_dynamic_twist(current_pos, current_yaw, effective_path, fused_grid, info)
        
# #         twist = Twist()
# #         twist.linear.x = linear_speed
# #         twist.angular.z = angular_speed
# #         self.cmd_vel_pub.publish(twist)
# #         self.training_buffer.append({
# #             'time': current_time,
# #             'position': current_pos,
# #             'twist': {'linear': linear_speed, 'angular': angular_speed}
# #         })
# #         self.get_logger().info(f"Twist publicado: linear = {linear_speed:.2f} m/s, angular = {angular_speed:.2f} rad/s")

# #     def check_effective_path_safety(self, path, fused_grid, info):
# #         """
# #         Revisa cada segmento del path para asegurarse de que la línea de vista sea segura.
# #         Retorna True si todos los segmentos son seguros; en caso contrario, False.
# #         """
# #         for i in range(len(path) - 1):
# #             if not self.is_line_free(fused_grid, info, path[i], path[i+1]):
# #                 self.get_logger().warn(f"Segmento inseguro entre {path[i]} y {path[i+1]}.")
# #                 return False
# #         return True



# #     # Método de fusión de mapas (similar al código plano)
# #     def fuse_maps_dynamic(self, grid_msg1, grid_msg2):
# #         info1 = grid_msg1.info if grid_msg1 is not None else None
# #         info2 = grid_msg2.info if grid_msg2 is not None else None
# #         if info1 is None and info2 is None:
# #             return None, None
# #         def get_bounds(info):
# #             ox = info.origin.position.x
# #             oy = info.origin.position.y
# #             max_x = ox + info.width * info.resolution
# #             max_y = oy + info.height * info.resolution
# #             return ox, oy, max_x, max_y
# #         bounds = []
# #         if info1 is not None:
# #             bounds.append(get_bounds(info1))
# #         if info2 is not None:
# #             bounds.append(get_bounds(info2))
# #         min_x = min(b[0] for b in bounds)
# #         min_y = min(b[1] for b in bounds)
# #         max_x = max(b[2] for b in bounds)
# #         max_y = max(b[3] for b in bounds)
# #         margin = 5.0
# #         min_x -= margin; min_y -= margin; max_x += margin; max_y += margin
# #         resolution = info1.resolution if info1 is not None else info2.resolution
# #         new_width = int(np.ceil((max_x - min_x) / resolution))
# #         new_height = int(np.ceil((max_y - min_y) / resolution))
# #         fused_grid = -1 * np.ones((new_height, new_width), dtype=np.int8)
# #         def reproject_map(grid_msg, fused_grid, new_origin_x, new_origin_y, resolution):
# #             info = grid_msg.info
# #             grid_array = np.array(grid_msg.data, dtype=np.int8).reshape((info.height, info.width))
# #             for j in range(info.height):
# #                 for i in range(info.width):
# #                     x = info.origin.position.x + (i + 0.5) * resolution
# #                     y = info.origin.position.y + (j + 0.5) * resolution
# #                     new_i = int((x - new_origin_x) / resolution)
# #                     new_j = int((y - new_origin_y) / resolution)
# #                     if 0 <= new_i < fused_grid.shape[1] and 0 <= new_j < fused_grid.shape[0]:
# #                         value = grid_array[j, i]
# #                         if value == 100:
# #                             fused_grid[new_j, new_i] = 100
# #                         elif value == 0 and fused_grid[new_j, new_i] != 100:
# #                             fused_grid[new_j, new_i] = 0
# #             return fused_grid
# #         if grid_msg1 is not None:
# #             fused_grid = reproject_map(grid_msg1, fused_grid, min_x, min_y, resolution)
# #         if grid_msg2 is not None:
# #             fused_grid = reproject_map(grid_msg2, fused_grid, min_x, min_y, resolution)
# #         new_info = MapMetaData()
# #         new_info.resolution = resolution
# #         new_info.width = new_width
# #         new_info.height = new_height
# #         new_info.origin.position.x = min_x
# #         new_info.origin.position.y = min_y
# #         new_info.origin.position.z = 0.0
# #         new_info.origin.orientation.w = 1.0
# #         return fused_grid, new_info

# # def main(args=None):
# #     rclpy.init(args=args)
# #     node = RLNavigationNode()
# #     try:
# #         rclpy.spin(node)
# #     except KeyboardInterrupt:
# #         pass
# #     node.destroy_node()
# #     rclpy.shutdown()

# # if __name__ == '__main__':
# #     main()

# # #!/usr/bin/env python3
# # import rclpy
# # from rclpy.node import Node
# # import math
# # import numpy as np
# # import cv2
# # import time
# # import threading
# # import heapq
# # import tensorflow as tf
# # from tensorflow.keras import layers, models, optimizers

# # # Mensajes de ROS2
# # from nav_msgs.msg import Odometry, OccupancyGrid, Path, MapMetaData
# # from geometry_msgs.msg import PoseArray, Twist, Pose, PoseStamped

# # ###############################
# # # Funciones auxiliares y planificación clásica
# # ###############################

# # def distance(p1, p2):
# #     return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

# # def dijkstra(graph, start, goal):
# #     dist = {node: float('inf') for node in graph.keys()}
# #     prev = {node: None for node in graph.keys()}
# #     dist[start] = 0
# #     queue = [(0, start)]
# #     while queue:
# #         current_dist, current_node = heapq.heappop(queue)
# #         if current_node == goal:
# #             break
# #         if current_dist > dist[current_node]:
# #             continue
# #         for neighbor, weight in graph[current_node]:
# #             alt = current_dist + weight
# #             if alt < dist[neighbor]:
# #                 dist[neighbor] = alt
# #                 prev[neighbor] = current_node
# #                 heapq.heappush(queue, (alt, neighbor))
# #     path = []
# #     node = goal
# #     while node is not None:
# #         path.append(node)
# #         node = prev[node]
# #     path.reverse()
# #     return path

# # def index_to_world(i, j, info):
# #     x = info.origin.position.x + (i + 0.5) * info.resolution
# #     y = info.origin.position.y + (j + 0.5) * info.resolution
# #     return (x, y)

# # ###############################
# # # Agente DRL: PPO con CNN
# # ###############################

# # class PPOAgent:
# #     def __init__(self, num_waypoints=10, input_shape=(64, 64, 2), pos_dim=2):
# #         self.num_waypoints = num_waypoints
# #         self.actor = self.build_actor(num_waypoints, input_shape, pos_dim)
# #         self.critic = self.build_critic(input_shape, pos_dim)
# #         self.actor_optimizer = optimizers.Adam(learning_rate=1e-4)
# #         self.critic_optimizer = optimizers.Adam(learning_rate=1e-3)
# #         self.gamma = 0.99

# #     def build_actor(self, num_waypoints, input_shape, pos_dim):
# #         grid_input = layers.Input(shape=input_shape, name='grid_input')
# #         x = layers.Conv2D(16, kernel_size=3, padding='same', activation='relu')(grid_input)
# #         x = layers.MaxPooling2D(2)(x)
# #         x = layers.Conv2D(32, kernel_size=3, padding='same', activation='relu')(x)
# #         x = layers.MaxPooling2D(2)(x)
# #         x = layers.Conv2D(64, kernel_size=3, padding='same', activation='relu')(x)
# #         x = layers.MaxPooling2D(2)(x)
# #         x = layers.GlobalAveragePooling2D()(x)
        
# #         goal_input = layers.Input(shape=(pos_dim,), name='goal_input')
# #         goal_features = layers.Dense(32, activation='relu')(goal_input)
        
# #         robot_input = layers.Input(shape=(pos_dim,), name='robot_input')
# #         robot_features = layers.Dense(32, activation='relu')(robot_input)
        
# #         concat = layers.Concatenate()([x, goal_features, robot_features])
# #         fc = layers.Dense(256, activation='relu')(concat)
# #         # La salida es una secuencia de waypoints normalizados en [0,1]
# #         output = layers.Dense(num_waypoints * 2, activation='sigmoid')(fc)
# #         outputs = layers.Reshape((num_waypoints, 2))(output)
# #         model = models.Model(inputs=[grid_input, goal_input, robot_input], outputs=outputs)
# #         return model

# #     def build_critic(self, input_shape, pos_dim):
# #         grid_input = layers.Input(shape=input_shape, name='grid_input')
# #         x = layers.Conv2D(16, kernel_size=3, padding='same', activation='relu')(grid_input)
# #         x = layers.MaxPooling2D(2)(x)
# #         x = layers.Conv2D(32, kernel_size=3, padding='same', activation='relu')(x)
# #         x = layers.MaxPooling2D(2)(x)
# #         x = layers.Conv2D(64, kernel_size=3, padding='same', activation='relu')(x)
# #         x = layers.MaxPooling2D(2)(x)
# #         x = layers.GlobalAveragePooling2D()(x)
        
# #         goal_input = layers.Input(shape=(pos_dim,), name='goal_input')
# #         goal_features = layers.Dense(32, activation='relu')(goal_input)
        
# #         robot_input = layers.Input(shape=(pos_dim,), name='robot_input')
# #         robot_features = layers.Dense(32, activation='relu')(robot_input)
        
# #         concat = layers.Concatenate()([x, goal_features, robot_features])
# #         fc = layers.Dense(256, activation='relu')(concat)
# #         value = layers.Dense(1)(fc)
# #         model = models.Model(inputs=[grid_input, goal_input, robot_input], outputs=value)
# #         return model

# #     def get_action(self, state):
# #         grid, goal, robot = state
# #         action = self.actor([grid, goal, robot], training=False)
# #         return action

# #     def compute_advantages(self, rewards, values, dones):
# #         advantages = []
# #         gae = 0
# #         next_value = 0
# #         for r, v, done in zip(reversed(rewards), reversed(values), reversed(dones)):
# #             delta = r + self.gamma * next_value * (1 - done) - v
# #             gae = delta + self.gamma * 0.95 * (1 - done) * gae
# #             advantages.insert(0, gae)
# #             next_value = v
# #         return advantages

# #     def ppo_update(self, states, actions, rewards, dones):
# #         grid, goal, robot = states
# #         values = self.critic([grid, goal, robot], training=False).numpy().flatten()
# #         advantages = self.compute_advantages(rewards, values, dones)
# #         advantages = np.array(advantages, dtype=np.float32)
# #         advantages = tf.convert_to_tensor(advantages.reshape(-1, 1, 1), dtype=tf.float32)
# #         targets = advantages + values
# #         actions = tf.convert_to_tensor(actions, dtype=tf.float32)
        
# #         for _ in range(10):
# #             with tf.GradientTape() as tape_actor, tf.GradientTape() as tape_critic:
# #                 new_actions = self.actor([grid, goal, robot], training=True)
# #                 actor_loss = tf.reduce_mean(tf.square(new_actions - actions) * advantages)
# #                 new_values = self.critic([grid, goal, robot], training=True)
# #                 critic_loss = tf.reduce_mean(tf.square(targets - new_values))
# #             grads_actor = tape_actor.gradient(actor_loss, self.actor.trainable_variables)
# #             grads_critic = tape_critic.gradient(critic_loss, self.critic.trainable_variables)
# #             self.actor_optimizer.apply_gradients(zip(grads_actor, self.actor.trainable_variables))
# #             self.critic_optimizer.apply_gradients(zip(grads_critic, self.critic.trainable_variables))

# # ###############################
# # # Nodo de entrenamiento (sin seguimiento de pure pursuit)
# # ###############################

# # class RLNavigationNode(Node):
# #     def __init__(self):
# #         super().__init__('rl_navigation_node')
# #         # Suscripciones a tópicos
# #         self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
# #         self.create_subscription(PoseArray, '/goal', self.goal_callback, 10)
# #         self.create_subscription(OccupancyGrid, '/occupancy_grid', self.current_grid_callback, 10)
# #         self.create_subscription(OccupancyGrid, '/persistent_dynamic_occupancy_grid', self.memory_grid_callback, 10)
# #         # Publicadores
# #         self.path_pub = self.create_publisher(Path, '/global_path_training', 10)
# #         # (Aquí se omite la publicación de Twist)
        
# #         # Variables de estado
# #         self.odom = None
# #         self.goal = None
# #         self.current_grid = None
# #         self.memory_grid = None
# #         self.current_global_path = None  # Path en coordenadas reales
# #         self.training_buffer = []
        
# #         # Parámetros del sistema y función de recompensa
# #         self.waypoint_threshold = 2.0
# #         self.connection_threshold = 2.0
        
# #         # Inicializar el agente DRL
# #         self.agent = PPOAgent(num_waypoints=10)
        
# #         # Iniciar el ciclo de entrenamiento y el control
# #         threading.Thread(target=self.training_loop, daemon=True).start()
# #         self.create_timer(0.1, self.control_loop)
# #         self.get_logger().info("Nodo RL Navigation iniciado (aprendizaje por ensayo y error).")

# #     # Callbacks de suscripción
# #     def odom_callback(self, msg: Odometry):
# #         self.odom = msg.pose.pose

# #     def goal_callback(self, msg: PoseArray):
# #         if msg.poses:
# #             self.goal = msg.poses[0]
# #             self.get_logger().info("Goal recibido.")

# #     def current_grid_callback(self, msg: OccupancyGrid):
# #         self.current_grid = msg

# #     def memory_grid_callback(self, msg: OccupancyGrid):
# #         self.memory_grid = msg

# #     # Preprocesar el grid a 64x64 con 2 canales
# #     def preprocess_grid(self, grid_msg):
# #         info = grid_msg.info
# #         grid = np.array(grid_msg.data, dtype=np.int8).reshape((info.height, info.width))
# #         grid = np.where(grid == 100, 1, grid)          # obstáculo
# #         grid = np.where(grid == -1, 0.5, grid)           # desconocido
# #         grid = np.where(grid == 0, 0, grid)              # libre
# #         grid_resized = cv2.resize(grid, (64, 64), interpolation=cv2.INTER_NEAREST)
# #         grid_resized = np.expand_dims(grid_resized, axis=-1)
# #         grid_resized = np.repeat(grid_resized, 2, axis=-1)
# #         grid_resized = grid_resized.astype(np.float32)
# #         return np.expand_dims(grid_resized, axis=0)

# #     def get_normalized_position(self, pos, info):
# #         map_width = info.width * info.resolution
# #         map_height = info.height * info.resolution
# #         norm_x = (pos.position.x - info.origin.position.x) / map_width
# #         norm_y = (pos.position.y - info.origin.position.y) / map_height
# #         return np.array([[norm_x, norm_y]], dtype=np.float32)

# #     def get_state(self):
# #         if self.current_grid is None or self.odom is None or self.goal is None:
# #             return None
# #         grid_tensor = self.preprocess_grid(self.current_grid)
# #         info = self.current_grid.info
# #         robot_pos = self.get_normalized_position(self.odom, info)
# #         goal_pos = self.get_normalized_position(self.goal, info)
# #         return grid_tensor, goal_pos, robot_pos

# #     def convert_predictions_to_world(self, predictions, info):
# #         predictions = predictions.numpy()[0]  # (num_waypoints, 2)
# #         map_width = info.width * info.resolution
# #         map_height = info.height * info.resolution
# #         origin_x = info.origin.position.x
# #         origin_y = info.origin.position.y
# #         path = []
# #         for wp in predictions:
# #             x = origin_x + wp[0] * map_width
# #             y = origin_y + wp[1] * map_height
# #             path.append((x, y))
# #         return path

# #     def publish_path(self, path):
# #         path_msg = Path()
# #         path_msg.header.stamp = self.get_clock().now().to_msg()
# #         path_msg.header.frame_id = "map"
# #         for pt in path:
# #             pose_st = PoseStamped()
# #             pose_st.header = path_msg.header
# #             pose_st.pose.position.x = pt[0]
# #             pose_st.pose.position.y = pt[1]
# #             pose_st.pose.position.z = 0.0
# #             pose_st.pose.orientation.w = 1.0
# #             path_msg.poses.append(pose_st)
# #         self.path_pub.publish(path_msg)
# #         self.get_logger().info(f"Path publicado con {len(path)} puntos.")

# #     # Función de recompensa: se penaliza la distancia al goal, se penaliza fuertemente si un segmento cruza obstáculo,
# #     # y se recompensa al alcanzar el goal.
# #     def compute_reward(self):
# #         if self.odom is None or self.goal is None or self.current_grid is None:
# #             return 0, 0
# #         info = self.current_grid.info
# #         robot = (self.odom.position.x, self.odom.position.y)
# #         goal = (self.goal.position.x, self.goal.position.y)
# #         direct_dist = distance(robot, goal)
# #         reward = -direct_dist  # Penaliza mayor distancia
# #         done = 0
# #         if self.current_global_path is not None and len(self.current_global_path) >= 2:
# #             path_penalty = 0
# #             total_length = 0
# #             for i in range(len(self.current_global_path)-1):
# #                 seg_length = distance(self.current_global_path[i], self.current_global_path[i+1])
# #                 total_length += seg_length
# #                 # Aquí se asume que la función is_line_free detecta correctamente los obstáculos.
# #                 if not self.is_line_free(self.preprocess_grid(self.current_grid)[0], info, self.current_global_path[i], self.current_global_path[i+1]):
# #                     path_penalty += 50  
# #             if total_length > 1.5 * direct_dist:
# #                 reward -= 5.0
# #             reward -= path_penalty
# #         if direct_dist < 0.5:
# #             reward += 20.0
# #             done = 1
# #         return reward, done

# #     # Verifica que cada segmento del path sea seguro
# #     def is_path_safe(self, path, fused_grid, info):
# #         for i in range(len(path)-1):
# #             if not self.is_line_free(fused_grid, info, path[i], path[i+1]):
# #                 self.get_logger().warn(f"Segmento inseguro entre {path[i]} y {path[i+1]}.")
# #                 return False
# #         return True

# #     # Decide si se debe replantear el path (por ejemplo, si cruza obstáculos o la conexión es deficiente)
# #     def should_replan(self, current_pos, fused_grid, info):
# #         if self.current_global_path is None or len(self.current_global_path) == 0:
# #             return True
# #         if not self.is_path_safe(self.current_global_path, fused_grid, info):
# #             self.get_logger().info("Path inseguro detectado, se requiere replanteo.")
# #             return True
# #         if distance(current_pos, self.current_global_path[0]) > self.connection_threshold * 2:
# #             self.get_logger().info("Desviación excesiva, se forzará replanteo.")
# #             return True
# #         return False

# #     # (Opcional) Actualiza el path si el nuevo propuesto mejora significativamente
# #     def update_path_if_improved(self, new_path, current_path, threshold=0.2):
# #         if current_path is None or len(current_path) == 0:
# #             return new_path
# #         def path_length(path):
# #             return sum(distance(path[i], path[i+1]) for i in range(len(path)-1))
# #         len_new = path_length(new_path)
# #         len_current = path_length(current_path)
# #         if (len_current - len_new) / (len_current + 1e-6) > threshold:
# #             return new_path
# #         else:
# #             return current_path

# #     # Proyecta el path sobre un grafo de nodos seguros extraídos del grid
# #     def project_path_onto_graph(self, path, fused_grid, info):
# #         nodes = self.extract_nodes(fused_grid, info)
# #         if not nodes:
# #             return path
# #         projected_path = []
# #         for pt in path:
# #             closest = min(nodes, key=lambda n: distance(n, pt))
# #             projected_path.append(closest)
# #         return projected_path

# #     def training_loop(self):
# #         while rclpy.ok():
# #             state = self.get_state()
# #             if state is None:
# #                 time.sleep(0.1)
# #                 continue
# #             # El agente produce un path basado en el estado actual
# #             action = self.agent.get_action(state)
# #             info = self.current_grid.info
# #             proposed_path = self.convert_predictions_to_world(action, info)
            
# #             # Forzar conectividad: primer waypoint = posición actual y último = goal
# #             robot_world = (self.odom.position.x, self.odom.position.y)
# #             goal_world = (self.goal.position.x, self.goal.position.y)
# #             proposed_path[0] = robot_world
# #             proposed_path[-1] = goal_world

# #             # Proyectar el path sobre el grafo de nodos seguros para "corregir" puntos fuera de zona libre
# #             fused_grid, _ = self.fuse_maps_dynamic(self.current_grid, self.memory_grid)
# #             proposed_path = self.project_path_onto_graph(proposed_path, fused_grid, info)

# #             self.current_global_path = proposed_path.copy()
# #             self.publish_path(proposed_path)
            
# #             reward, done = self.compute_reward()
# #             next_state = self.get_state()
# #             self.training_buffer.append((state, action.numpy(), reward, next_state, done))
            
# #             if len(self.training_buffer) >= 32:
# #                 batch = self.training_buffer[:32]
# #                 self.training_buffer = self.training_buffer[32:]
# #                 grids = np.concatenate([b[0][0] for b in batch], axis=0)
# #                 goals = np.concatenate([b[0][1] for b in batch], axis=0)
# #                 robots = np.concatenate([b[0][2] for b in batch], axis=0)
# #                 actions_batch = np.array([b[1] for b in batch])
# #                 rewards = np.array([b[2] for b in batch])
# #                 dones = np.array([b[4] for b in batch])
# #                 self.agent.ppo_update((grids, goals, robots), actions_batch, rewards, dones)
# #             if done:
# #                 self.get_logger().info("Goal alcanzado. Reiniciando episodio.")
# #                 # Reinicializar variables de episodio si es necesario.
# #             time.sleep(0.1)

# #     # Control loop: aquí se verifica y replantea el path si es necesario
# #     def control_loop(self):
# #         if self.odom is None or self.goal is None or self.current_grid is None:
# #             return
# #         fused_result = self.fuse_maps_dynamic(self.current_grid, self.memory_grid)
# #         if fused_result is None or fused_result[0] is None:
# #             self.get_logger().warn("No se pudo fusionar mapas. Esperando datos válidos...")
# #             return
# #         fused_grid, info = fused_result
# #         current_pos = (self.odom.position.x, self.odom.position.y)
# #         if self.current_global_path is None or self.should_replan(current_pos, fused_grid, info):
# #             self.get_logger().info("Replanificando path mediante fallback.")
# #             new_path = self.fallback_path(current_pos, (self.goal.position.x, self.goal.position.y), fused_grid, info)
# #             if new_path is None or len(new_path) < 3:
# #                 self.get_logger().error("Fallback path falló, abortando replanteo.")
# #                 return
# #             new_path = self.smooth_path(new_path)
# #             self.current_global_path = self.update_path_if_improved(new_path, self.current_global_path)
# #         self.publish_path(self.current_global_path)
# #         self.get_logger().info(f"Path efectivo publicado con {len(self.current_global_path)} puntos.")

# #     def is_line_free(self, fused_grid, info, start, end):
# #             def world_to_index(point, info):
# #                 i = int((point[0] - info.origin.position.x) / info.resolution)
# #                 j = int((point[1] - info.origin.position.y) / info.resolution)
# #                 return i, j

# #             x0, y0 = world_to_index(start, info)
# #             x1, y1 = world_to_index(end, info)

# #             dx = abs(x1 - x0)
# #             dy = abs(y1 - y0)
# #             x, y = x0, y0
# #             sx = -1 if x0 > x1 else 1
# #             sy = -1 if y0 > y1 else 1

# #             # Función auxiliar para verificar límites
# #             def is_in_bounds(x, y, grid):
# #                 return 0 <= x < grid.shape[1] and 0 <= y < grid.shape[0]

# #             if dx > dy:
# #                 err = dx / 2.0
# #                 while x != x1:
# #                     if not is_in_bounds(x, y, fused_grid) or fused_grid[y, x] == 100:
# #                         return False
# #                     err -= dy
# #                     if err < 0:
# #                         y += sy
# #                         err += dx
# #                     x += sx
# #             else:
# #                 err = dy / 2.0
# #                 while y != y1:
# #                     if not is_in_bounds(x, y, fused_grid) or fused_grid[y, x] == 100:
# #                         return False
# #                     err -= dx
# #                     if err < 0:
# #                         x += sx
# #                         err += dy
# #                     y += sy
# #             if not is_in_bounds(x, y, fused_grid) or fused_grid[y, x] == 100:
# #                 return False
# #             return True

# #     def fallback_path(self, current_pos, goal_pos, fused_grid, info, step_size=0.2, offset_step=0.3, max_offset=1.0):
# #         """
# #         Genera un path incremental entre current_pos y goal_pos en pasos de step_size.
# #         Si algún segmento no es seguro (is_line_free falla), se intenta desplazar el punto
# #         intermedio en dirección perpendicular hasta encontrar una posición segura.
# #         """
# #         # Generar la línea base (recta) con varios puntos
# #         path = []
# #         total_dist = distance(current_pos, goal_pos)
# #         num_steps = int(total_dist / step_size)
# #         if num_steps < 1:
# #             num_steps = 1
# #         for i in range(num_steps + 1):
# #             ratio = i / float(num_steps)
# #             x = current_pos[0] + ratio * (goal_pos[0] - current_pos[0])
# #             y = current_pos[1] + ratio * (goal_pos[1] - current_pos[1])
# #             path.append((x, y))
        
# #         # Ahora ajustar los puntos intermedios si el segmento no es seguro
# #         safe_path = [path[0]]
# #         for i in range(1, len(path)):
# #             prev_pt = safe_path[-1]
# #             pt = path[i]
# #             # Verificar si el segmento es seguro
# #             if self.is_line_free(fused_grid, info, prev_pt, pt):
# #                 safe_path.append(pt)
# #             else:
# #                 # Calcular la dirección del segmento y su perpendicular
# #                 dx = pt[0] - prev_pt[0]
# #                 dy = pt[1] - prev_pt[1]
# #                 theta = math.atan2(dy, dx)
# #                 perp_angle = theta + math.pi / 2  # dirección perpendicular
                
# #                 found_safe = False
# #                 # Probar a desplazar en ambas direcciones
# #                 for sign in [1, -1]:
# #                     offset = offset_step
# #                     while offset <= max_offset:
# #                         new_pt = (pt[0] + sign * offset * math.cos(perp_angle),
# #                                 pt[1] + sign * offset * math.sin(perp_angle))
# #                         # Verificar si el segmento modificado es seguro
# #                         if self.is_line_free(fused_grid, info, prev_pt, new_pt):
# #                             safe_path.append(new_pt)
# #                             found_safe = True
# #                             break
# #                         offset += offset_step
# #                     if found_safe:
# #                         break
# #                 # Si no se encontró ninguna alternativa segura, se añade el punto original
# #                 if not found_safe:
# #                     safe_path.append(pt)
# #         return safe_path

# #     def smooth_path(self, path):
# #         if len(path) < 3:
# #             return path
# #         smoothed = [path[0]]
# #         for i in range(1, len(path)-1):
# #             avg_x = (path[i-1][0] + path[i][0] + path[i+1][0]) / 3.0
# #             avg_y = (path[i-1][1] + path[i][1] + path[i+1][1]) / 3.0
# #             smoothed.append((avg_x, avg_y))
# #         smoothed.append(path[-1])
# #         return smoothed

# #     def extract_nodes(self, fused_grid, info):
# #         nodes = []
# #         for j in range(fused_grid.shape[0]):
# #             for i in range(fused_grid.shape[1]):
# #                 if fused_grid[j, i] == 0:
# #                     nodes.append(index_to_world(i, j, info))
# #         return nodes

# #     def fuse_maps_dynamic(self, grid_msg1, grid_msg2):
# #         info1 = grid_msg1.info if grid_msg1 is not None else None
# #         info2 = grid_msg2.info if grid_msg2 is not None else None
# #         if info1 is None and info2 is None:
# #             return None, None
# #         def get_bounds(info):
# #             ox = info.origin.position.x
# #             oy = info.origin.position.y
# #             max_x = ox + info.width * info.resolution
# #             max_y = oy + info.height * info.resolution
# #             return ox, oy, max_x, max_y
# #         bounds = []
# #         if info1 is not None:
# #             bounds.append(get_bounds(info1))
# #         if info2 is not None:
# #             bounds.append(get_bounds(info2))
# #         min_x = min(b[0] for b in bounds)
# #         min_y = min(b[1] for b in bounds)
# #         max_x = max(b[2] for b in bounds)
# #         max_y = max(b[3] for b in bounds)
# #         margin = 5.0
# #         min_x -= margin; min_y -= margin; max_x += margin; max_y += margin
# #         resolution = info1.resolution if info1 is not None else info2.resolution
# #         new_width = int(np.ceil((max_x - min_x) / resolution))
# #         new_height = int(np.ceil((max_y - min_y) / resolution))
# #         fused_grid = -1 * np.ones((new_height, new_width), dtype=np.int8)
# #         def reproject_map(grid_msg, fused_grid, new_origin_x, new_origin_y, resolution):
# #             info = grid_msg.info
# #             grid_array = np.array(grid_msg.data, dtype=np.int8).reshape((info.height, info.width))
# #             for j in range(info.height):
# #                 for i in range(info.width):
# #                     x = info.origin.position.x + (i + 0.5) * resolution
# #                     y = info.origin.position.y + (j + 0.5) * resolution
# #                     new_i = int((x - new_origin_x) / resolution)
# #                     new_j = int((y - new_origin_y) / resolution)
# #                     if 0 <= new_i < fused_grid.shape[1] and 0 <= new_j < fused_grid.shape[0]:
# #                         value = grid_array[j, i]
# #                         if value == 100:
# #                             fused_grid[new_j, new_i] = 100
# #                         elif value == 0 and fused_grid[new_j, new_i] != 100:
# #                             fused_grid[new_j, new_i] = 0
# #             return fused_grid
# #         if grid_msg1 is not None:
# #             fused_grid = reproject_map(grid_msg1, fused_grid, min_x, min_y, resolution)
# #         if grid_msg2 is not None:
# #             fused_grid = reproject_map(grid_msg2, fused_grid, min_x, min_y, resolution)
# #         new_info = MapMetaData()
# #         new_info.resolution = resolution
# #         new_info.width = new_width
# #         new_info.height = new_height
# #         new_info.origin.position.x = min_x
# #         new_info.origin.position.y = min_y
# #         new_info.origin.position.z = 0.0
# #         new_info.origin.orientation.w = 1.0
# #         return fused_grid, new_info

# # def main(args=None):
# #     rclpy.init(args=args)
# #     node = RLNavigationNode()
# #     try:
# #         rclpy.spin(node)
# #     except KeyboardInterrupt:
# #         pass
# #     node.destroy_node()
# #     rclpy.shutdown()

# # if __name__ == '__main__':
# #     main()

# # #!/usr/bin/env python3
# # import rclpy
# # from rclpy.node import Node
# # import math
# # import numpy as np
# # import cv2
# # import time
# # import threading
# # import heapq
# # import tensorflow as tf
# # from tensorflow.keras import layers, models, optimizers

# # # Mensajes de ROS2
# # from nav_msgs.msg import Odometry, OccupancyGrid, Path, MapMetaData
# # from geometry_msgs.msg import PoseArray, Twist, Pose, PoseStamped

# # ###############################
# # # Funciones auxiliares y planificación clásica
# # ###############################

# # def distance(p1, p2):
# #     return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

# # def dijkstra(graph, start, goal):
# #     dist = {node: float('inf') for node in graph.keys()}
# #     prev = {node: None for node in graph.keys()}
# #     dist[start] = 0
# #     queue = [(0, start)]
# #     while queue:
# #         current_dist, current_node = heapq.heappop(queue)
# #         if current_node == goal:
# #             break
# #         if current_dist > dist[current_node]:
# #             continue
# #         for neighbor, weight in graph[current_node]:
# #             alt = current_dist + weight
# #             if alt < dist[neighbor]:
# #                 dist[neighbor] = alt
# #                 prev[neighbor] = current_node
# #                 heapq.heappush(queue, (alt, neighbor))
# #     path = []
# #     node = goal
# #     while node is not None:
# #         path.append(node)
# #         node = prev[node]
# #     path.reverse()
# #     return path

# # def index_to_world(i, j, info):
# #     x = info.origin.position.x + (i + 0.5) * info.resolution
# #     y = info.origin.position.y + (j + 0.5) * info.resolution
# #     return (x, y)

# # ###############################
# # # Agente DRL: PPO con CNN
# # ###############################

# # class PPOAgent:
# #     def __init__(self, num_waypoints=10, input_shape=(64, 64, 2), pos_dim=2):
# #         self.num_waypoints = num_waypoints
# #         self.actor = self.build_actor(num_waypoints, input_shape, pos_dim)
# #         self.critic = self.build_critic(input_shape, pos_dim)
# #         self.actor_optimizer = optimizers.Adam(learning_rate=1e-4)
# #         self.critic_optimizer = optimizers.Adam(learning_rate=1e-3)
# #         self.gamma = 0.99

# #     def build_actor(self, num_waypoints, input_shape, pos_dim):
# #         grid_input = layers.Input(shape=input_shape, name='grid_input')
# #         x = layers.Conv2D(16, kernel_size=3, padding='same', activation='relu')(grid_input)
# #         x = layers.MaxPooling2D(2)(x)
# #         x = layers.Conv2D(32, kernel_size=3, padding='same', activation='relu')(x)
# #         x = layers.MaxPooling2D(2)(x)
# #         x = layers.Conv2D(64, kernel_size=3, padding='same', activation='relu')(x)
# #         x = layers.MaxPooling2D(2)(x)
# #         x = layers.GlobalAveragePooling2D()(x)
        
# #         goal_input = layers.Input(shape=(pos_dim,), name='goal_input')
# #         goal_features = layers.Dense(32, activation='relu')(goal_input)
        
# #         robot_input = layers.Input(shape=(pos_dim,), name='robot_input')
# #         robot_features = layers.Dense(32, activation='relu')(robot_input)
        
# #         concat = layers.Concatenate()([x, goal_features, robot_features])
# #         fc = layers.Dense(256, activation='relu')(concat)
# #         # Salida: secuencia de waypoints normalizados
# #         output = layers.Dense(num_waypoints * 2, activation='sigmoid')(fc)
# #         outputs = layers.Reshape((num_waypoints, 2))(output)
# #         model = models.Model(inputs=[grid_input, goal_input, robot_input], outputs=outputs)
# #         return model

# #     def build_critic(self, input_shape, pos_dim):
# #         grid_input = layers.Input(shape=input_shape, name='grid_input')
# #         x = layers.Conv2D(16, kernel_size=3, padding='same', activation='relu')(grid_input)
# #         x = layers.MaxPooling2D(2)(x)
# #         x = layers.Conv2D(32, kernel_size=3, padding='same', activation='relu')(x)
# #         x = layers.MaxPooling2D(2)(x)
# #         x = layers.Conv2D(64, kernel_size=3, padding='same', activation='relu')(x)
# #         x = layers.MaxPooling2D(2)(x)
# #         x = layers.GlobalAveragePooling2D()(x)
        
# #         goal_input = layers.Input(shape=(pos_dim,), name='goal_input')
# #         goal_features = layers.Dense(32, activation='relu')(goal_input)
        
# #         robot_input = layers.Input(shape=(pos_dim,), name='robot_input')
# #         robot_features = layers.Dense(32, activation='relu')(robot_input)
        
# #         concat = layers.Concatenate()([x, goal_features, robot_features])
# #         fc = layers.Dense(256, activation='relu')(concat)
# #         value = layers.Dense(1)(fc)
# #         model = models.Model(inputs=[grid_input, goal_input, robot_input], outputs=value)
# #         return model

# #     def get_action(self, state):
# #         grid, goal, robot = state
# #         action = self.actor([grid, goal, robot], training=False)
# #         return action

# #     def compute_advantages(self, rewards, values, dones):
# #         advantages = []
# #         gae = 0
# #         next_value = 0
# #         for r, v, done in zip(reversed(rewards), reversed(values), reversed(dones)):
# #             delta = r + self.gamma * next_value * (1 - done) - v
# #             gae = delta + self.gamma * 0.95 * (1 - done) * gae
# #             advantages.insert(0, gae)
# #             next_value = v
# #         return advantages

# #     def ppo_update(self, states, actions, rewards, dones):
# #         grid, goal, robot = states
# #         values = self.critic([grid, goal, robot], training=False).numpy().flatten()
# #         advantages = self.compute_advantages(rewards, values, dones)
# #         advantages = np.array(advantages, dtype=np.float32)
# #         advantages = tf.convert_to_tensor(advantages.reshape(-1, 1, 1), dtype=tf.float32)
# #         targets = advantages + values
# #         actions = tf.convert_to_tensor(actions, dtype=tf.float32)
        
# #         for _ in range(10):
# #             with tf.GradientTape() as tape_actor, tf.GradientTape() as tape_critic:
# #                 new_actions = self.actor([grid, goal, robot], training=True)
# #                 actor_loss = tf.reduce_mean(tf.square(new_actions - actions) * advantages)
# #                 new_values = self.critic([grid, goal, robot], training=True)
# #                 critic_loss = tf.reduce_mean(tf.square(targets - new_values))
# #             grads_actor = tape_actor.gradient(actor_loss, self.actor.trainable_variables)
# #             grads_critic = tape_critic.gradient(critic_loss, self.critic.trainable_variables)
# #             self.actor_optimizer.apply_gradients(zip(grads_actor, self.actor.trainable_variables))
# #             self.critic_optimizer.apply_gradients(zip(grads_critic, self.critic.trainable_variables))

# # ###############################
# # # Nodo de entrenamiento (sin seguimiento de pure pursuit)
# # ###############################

# # class RLNavigationNode(Node):
# #     def __init__(self):
# #         super().__init__('rl_navigation_node')
# #         # Suscripciones a tópicos
# #         self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
# #         self.create_subscription(PoseArray, '/goal', self.goal_callback, 10)
# #         self.create_subscription(OccupancyGrid, '/occupancy_grid', self.current_grid_callback, 10)
# #         #self.create_subscription(OccupancyGrid, '/persistent_dynamic_occupancy_grid', self.memory_grid_callback, 10)
# #         # Publicador
# #         self.path_pub = self.create_publisher(Path, '/global_path_training', 10)
# #         # Variables de estado
# #         self.odom = None
# #         self.goal = None
# #         self.current_grid = None
# #         self.memory_grid = None
# #         self.current_global_path = None
# #         self.training_buffer = []
# #         # Parámetros para evaluar la conexión del path
# #         self.waypoint_threshold = 2.0
# #         self.connection_threshold = 2.0
# #         # Inicializar el agente DRL
# #         self.agent = PPOAgent(num_waypoints=10)
# #         # Iniciar el ciclo de entrenamiento y el control
# #         threading.Thread(target=self.training_loop, daemon=True).start()
# #         self.create_timer(0.1, self.control_loop)
# #         self.get_logger().info("Nodo RL Navigation iniciado (aprendizaje por ensayo y error).")

# #     # Callbacks
# #     def odom_callback(self, msg: Odometry):
# #         self.odom = msg.pose.pose

# #     def goal_callback(self, msg: PoseArray):
# #         if msg.poses:
# #             self.goal = msg.poses[0]
# #             self.get_logger().info("Goal recibido.")

# #     def current_grid_callback(self, msg: OccupancyGrid):
# #         self.current_grid = msg

# #     # def memory_grid_callback(self, msg: OccupancyGrid):
# #     #     self.memory_grid = msg

# #     # Preprocesamiento del grid: redimensiona a 64x64 y crea 2 canales
# #     def preprocess_grid(self, grid_msg):
# #         info = grid_msg.info
# #         grid = np.array(grid_msg.data, dtype=np.int8).reshape((info.height, info.width))
# #         grid = np.where(grid == 100, 1, grid)          # obstáculo confirmado
# #         grid = np.where(grid == -1, 0.5, grid)           # incierto
# #         grid = np.where(grid == 0, 0, grid)              # libre
# #         grid_resized = cv2.resize(grid, (64, 64), interpolation=cv2.INTER_NEAREST)
# #         grid_resized = np.expand_dims(grid_resized, axis=-1)
# #         grid_resized = np.repeat(grid_resized, 2, axis=-1)
# #         grid_resized = grid_resized.astype(np.float32)
# #         return np.expand_dims(grid_resized, axis=0)

# #     def get_normalized_position(self, pos, info):
# #         map_width = info.width * info.resolution
# #         map_height = info.height * info.resolution
# #         norm_x = (pos.position.x - info.origin.position.x) / map_width
# #         norm_y = (pos.position.y - info.origin.position.y) / map_height
# #         return np.array([[norm_x, norm_y]], dtype=np.float32)

# #     def get_state(self):
# #         if self.current_grid is None or self.odom is None or self.goal is None:
# #             return None
# #         grid_tensor = self.preprocess_grid(self.current_grid)
# #         info = self.current_grid.info
# #         robot_pos = self.get_normalized_position(self.odom, info)
# #         goal_pos = self.get_normalized_position(self.goal, info)
# #         return grid_tensor, goal_pos, robot_pos

# #     def convert_predictions_to_world(self, predictions, info):
# #         predictions = predictions.numpy()[0]  # (num_waypoints, 2)
# #         map_width = info.width * info.resolution
# #         map_height = info.height * info.resolution
# #         origin_x = info.origin.position.x
# #         origin_y = info.origin.position.y
# #         path = []
# #         for wp in predictions:
# #             x = origin_x + wp[0] * map_width
# #             y = origin_y + wp[1] * map_height
# #             path.append((x, y))
# #         return path


# #     def is_waypoint_safe(self, fused_grid, info, point, radio):
# #         """
# #         Verifica que no existan celdas ocupadas dentro de un círculo de radio 'radio' alrededor del waypoint.
# #         fused_grid: arreglo del grid (por ejemplo, de tamaño (H, W))
# #         info: información del mapa (MapMetaData)
# #         point: (x, y) en coordenadas del mundo
# #         radio: radio en metros a verificar
# #         Retorna True si el waypoint es seguro (sin obstáculos en ese radio), False en caso contrario.
# #         """
# #         # Convertir el radio en número de celdas (suponiendo que info.resolution es en metros/celda)
# #         cells_radio = int(radio / info.resolution)
# #         origin_x = info.origin.position.x
# #         origin_y = info.origin.position.y
# #         # Convertir las coordenadas del waypoint a índices del grid
# #         i_center = int((point[0] - origin_x) / info.resolution)
# #         j_center = int((point[1] - origin_y) / info.resolution)
# #         # Iterar sobre un cuadrado que cubra el círculo
# #         for i in range(i_center - cells_radio, i_center + cells_radio + 1):
# #             for j in range(j_center - cells_radio, j_center + cells_radio + 1):
# #                 # Verificar que (i, j) esté dentro del círculo
# #                 if math.sqrt((i - i_center)**2 + (j - j_center)**2) <= cells_radio:
# #                     # Comprobar límites
# #                     if 0 <= i < fused_grid.shape[1] and 0 <= j < fused_grid.shape[0]:
# #                         if fused_grid[j, i] == 1:  # 1 indica obstáculo confirmado
# #                             return False
# #         return True



# #     def publish_path(self, path):
# #         path_msg = Path()
# #         path_msg.header.stamp = self.get_clock().now().to_msg()
# #         path_msg.header.frame_id = "map"
# #         for pt in path:
# #             pose_st = PoseStamped()
# #             pose_st.header = path_msg.header
# #             pose_st.pose.position.x = pt[0]
# #             pose_st.pose.position.y = pt[1]
# #             pose_st.pose.position.z = 0.0
# #             pose_st.pose.orientation.w = 1.0
# #             path_msg.poses.append(pose_st)
# #         self.path_pub.publish(path_msg)
# #         self.get_logger().info(f"Path publicado con {len(path)} puntos.")

# #     # Función de recompensa: penaliza la distancia al goal, penaliza fuertemente si algún segmento cruza una celda ocupada.
# #     def compute_reward(self):
# #         if self.odom is None or self.goal is None or self.current_grid is None:
# #             return 0, 0
# #         info = self.current_grid.info
# #         robot = (self.odom.position.x, self.odom.position.y)
# #         goal = (self.goal.position.x, self.goal.position.y)
# #         direct_dist = distance(robot, goal)
# #         reward = -direct_dist  # Incentiva caminos cortos
# #         done = 0
# #         penalty = 0
# #         safe_radio = 0.5  # Radio en metros para verificar seguridad del waypoint
# #         if self.current_global_path is not None and len(self.current_global_path) >= 2:
# #             for pt in self.current_global_path:
# #                 # Se penaliza si el waypoint no es seguro
# #                 if not self.is_waypoint_safe(self.preprocess_grid(self.current_grid)[0], info, pt, safe_radio):
# #                     penalty += 100  # Valor de penalización alto
# #             reward -= penalty
# #         if direct_dist < 0.5:
# #             reward += 20.0
# #             done = 1
# #         return reward, done

# #     # Verifica que cada segmento del path sea seguro (solo permite pasar por celdas libres o inciertas)
# #     def is_line_free(self, fused_grid, info, start, end):
# #         def world_to_index(point, info):
# #             i = int((point[0] - info.origin.position.x) / info.resolution)
# #             j = int((point[1] - info.origin.position.y) / info.resolution)
# #             return i, j
# #         x0, y0 = world_to_index(start, info)
# #         x1, y1 = world_to_index(end, info)
# #         dx = abs(x1 - x0)
# #         dy = abs(y1 - y0)
# #         x, y = x0, y0
# #         sx = -1 if x0 > x1 else 1
# #         sy = -1 if y0 > y1 else 1
# #         def is_in_bounds(x, y, grid):
# #             return 0 <= x < grid.shape[1] and 0 <= y < grid.shape[0]
# #         if dx > dy:
# #             err = dx / 2.0
# #             while x != x1:
# #                 if not is_in_bounds(x, y, fused_grid):
# #                     return False
# #                 # Si la celda es ocupada (1) se rechaza. Las inciertas (0.5) se permiten.
# #                 if fused_grid[y, x] == 1:
# #                     return False
# #                 err -= dy
# #                 if err < 0:
# #                     y += sy
# #                     err += dx
# #                 x += sx
# #         else:
# #             err = dy / 2.0
# #             while y != y1:
# #                 if not is_in_bounds(x, y, fused_grid):
# #                     return False
# #                 if fused_grid[y, x] == 1:
# #                     return False
# #                 err -= dx
# #                 if err < 0:
# #                     x += sx
# #                     err += dy
# #                 y += sy
# #         if not is_in_bounds(x, y, fused_grid) or fused_grid[y, x] == 1:
# #             return False
# #         return True

# #     # Forzar replanteo si la posición actual está fuera del path o si el path no es seguro
# #     def should_replan(self, current_pos, fused_grid, info):
# #         if self.current_global_path is None or len(self.current_global_path) == 0:
# #             return True
# #         if distance(current_pos, self.current_global_path[0]) > self.waypoint_threshold:
# #             self.get_logger().info("Posición del robot fuera del path, forzando replanteo.")
# #             return True
# #         if not self.is_line_free(fused_grid, info, self.current_global_path[0], self.current_global_path[-1]):
# #             self.get_logger().info("El path cruza obstáculos, forzando replanteo.")
# #             return True
# #         return False

# #     # Proyecta el path sobre un grafo de nodos seguros extraídos del grid
# #     def project_path_onto_graph(self, path, fused_grid, info):
# #         nodes = self.extract_nodes(fused_grid, info)
# #         if not nodes:
# #             return path
# #         projected_path = []
# #         for pt in path:
# #             closest = min(nodes, key=lambda n: distance(n, pt))
# #             projected_path.append(closest)
# #         return projected_path

# #     # Actualiza el path si el nuevo propuesto mejora significativamente (opcional)
# #     def update_path_if_improved(self, new_path, current_path, threshold=0.2):
# #         if current_path is None or len(current_path) == 0:
# #             return new_path
# #         def path_length(path):
# #             return sum(distance(path[i], path[i+1]) for i in range(len(path)-1))
# #         len_new = path_length(new_path)
# #         len_current = path_length(current_path)
# #         if (len_current - len_new) / (len_current + 1e-6) > threshold:
# #             return new_path
# #         else:
# #             return current_path

# #     # Método fallback: genera un path incremental bordeando obstáculos

# #     def fallback_path(self, current_pos, goal_pos, fused_grid, info, step_size=0.2, offset_step=0.3, max_offset=1.0, safe_radio=2.0):
# #         """
# #         Genera un path incremental entre current_pos y goal_pos.
# #         Para cada waypoint, se verifica que no haya obstáculo dentro de un radio 'safe_radio'.
# #         Si no es seguro, intenta desplazarlo.
# #         """
# #         path = []
# #         total_dist = distance(current_pos, goal_pos)
# #         num_steps = int(total_dist / step_size)
# #         if num_steps < 1:
# #             num_steps = 1
# #         for i in range(num_steps + 1):
# #             ratio = i / float(num_steps)
# #             x = current_pos[0] + ratio * (goal_pos[0] - current_pos[0])
# #             y = current_pos[1] + ratio * (goal_pos[1] - current_pos[1])
# #             path.append((x, y))
        
# #         safe_path = [path[0]]
# #         for i in range(1, len(path)):
# #             prev_pt = safe_path[-1]
# #             pt = path[i]
# #             # Verificar la seguridad del waypoint
# #             if self.is_waypoint_safe(fused_grid, info, pt, safe_radio) and self.is_line_free(fused_grid, info, prev_pt, pt):
# #                 safe_path.append(pt)
# #             else:
# #                 dx = pt[0] - prev_pt[0]
# #                 dy = pt[1] - prev_pt[1]
# #                 theta = math.atan2(dy, dx)
# #                 perp_angle = theta + math.pi / 2
# #                 found_safe = False
# #                 for sign in [1, -1]:
# #                     offset = offset_step
# #                     while offset <= max_offset:
# #                         new_pt = (pt[0] + sign * offset * math.cos(perp_angle),
# #                                 pt[1] + sign * offset * math.sin(perp_angle))
# #                         if self.is_waypoint_safe(fused_grid, info, new_pt, safe_radio) and self.is_line_free(fused_grid, info, prev_pt, new_pt):
# #                             safe_path.append(new_pt)
# #                             found_safe = True
# #                             break
# #                         offset += offset_step
# #                     if found_safe:
# #                         break
# #                 if not found_safe:
# #                     safe_path.append(pt)
# #         return safe_path

# #     def training_loop(self):
# #         while rclpy.ok():
# #             state = self.get_state()
# #             if state is None:
# #                 time.sleep(0.1)
# #                 continue
# #             action = self.agent.get_action(state)
# #             info = self.current_grid.info
# #             proposed_path = self.convert_predictions_to_world(action, info)
# #             # Forzar conectividad: primer waypoint = posición actual y último = goal.
# #             robot_world = (self.odom.position.x, self.odom.position.y)
# #             goal_world = (self.goal.position.x, self.goal.position.y)
# #             proposed_path[0] = robot_world
# #             proposed_path[-1] = goal_world
# #             fused_grid, _ = self.fuse_maps_dynamic(self.current_grid, self.memory_grid)
# #             proposed_path = self.project_path_onto_graph(proposed_path, fused_grid, info)
# #             self.current_global_path = proposed_path.copy()
# #             self.publish_path(proposed_path)
# #             reward, done = self.compute_reward()
# #             next_state = self.get_state()
# #             self.training_buffer.append((state, action.numpy(), reward, next_state, done))
# #             if len(self.training_buffer) >= 32:
# #                 batch = self.training_buffer[:32]
# #                 self.training_buffer = self.training_buffer[32:]
# #                 grids = np.concatenate([b[0][0] for b in batch], axis=0)
# #                 goals = np.concatenate([b[0][1] for b in batch], axis=0)
# #                 robots = np.concatenate([b[0][2] for b in batch], axis=0)
# #                 actions_batch = np.array([b[1] for b in batch])
# #                 rewards = np.array([b[2] for b in batch])
# #                 dones = np.array([b[4] for b in batch])
# #                 self.agent.ppo_update((grids, goals, robots), actions_batch, rewards, dones)
# #             if done:
# #                 self.get_logger().info("Goal alcanzado. Reiniciando episodio.")
# #             time.sleep(0.1)

# #     def control_loop(self):
# #         if self.odom is None or self.goal is None or self.current_grid is None:
# #             return
# #         fused_result = self.fuse_maps_dynamic(self.current_grid, self.memory_grid)
# #         if fused_result is None or fused_result[0] is None:
# #             self.get_logger().warn("No se pudo fusionar mapas. Esperando datos válidos...")
# #             return
# #         fused_grid, info = fused_result
# #         current_pos = (self.odom.position.x, self.odom.position.y)
# #         if self.current_global_path is None or self.should_replan(current_pos, fused_grid, info):
# #             self.get_logger().info("Replanificando path mediante fallback.")
# #             new_path = self.fallback_path(current_pos, (self.goal.position.x, self.goal.position.y), fused_grid, info)
# #             if new_path is None or len(new_path) < 3:
# #                 self.get_logger().error("Fallback path falló, abortando replanteo.")
# #                 return
# #             new_path = self.smooth_path(new_path)
# #             self.current_global_path = self.update_path_if_improved(new_path, self.current_global_path)
# #         self.publish_path(self.current_global_path)
# #         self.get_logger().info(f"Path efectivo publicado con {len(self.current_global_path)} puntos.")

# #     def smooth_path(self, path):
# #         if len(path) < 3:
# #             return path
# #         smoothed = [path[0]]
# #         for i in range(1, len(path)-1):
# #             avg_x = (path[i-1][0] + path[i][0] + path[i+1][0]) / 3.0
# #             avg_y = (path[i-1][1] + path[i][1] + path[i+1][1]) / 3.0
# #             smoothed.append((avg_x, avg_y))
# #         smoothed.append(path[-1])
# #         return smoothed

# #     def extract_nodes(self, fused_grid, info):
# #         nodes = []
# #         for j in range(fused_grid.shape[0]):
# #             for i in range(fused_grid.shape[1]):
# #                 if fused_grid[j, i] == 0:
# #                     nodes.append(index_to_world(i, j, info))
# #         return nodes

# #     def fuse_maps_dynamic(self, grid_msg1, grid_msg2):
# #         info1 = grid_msg1.info if grid_msg1 is not None else None
# #         info2 = grid_msg2.info if grid_msg2 is not None else None
# #         if info1 is None and info2 is None:
# #             return None, None
# #         def get_bounds(info):
# #             ox = info.origin.position.x
# #             oy = info.origin.position.y
# #             max_x = ox + info.width * info.resolution
# #             max_y = oy + info.height * info.resolution
# #             return ox, oy, max_x, max_y
# #         bounds = []
# #         if info1 is not None:
# #             bounds.append(get_bounds(info1))
# #         if info2 is not None:
# #             bounds.append(get_bounds(info2))
# #         min_x = min(b[0] for b in bounds)
# #         min_y = min(b[1] for b in bounds)
# #         max_x = max(b[2] for b in bounds)
# #         max_y = max(b[3] for b in bounds)
# #         margin = 5.0
# #         min_x -= margin; min_y -= margin; max_x += margin; max_y += margin
# #         resolution = info1.resolution if info1 is not None else info2.resolution
# #         new_width = int(np.ceil((max_x - min_x) / resolution))
# #         new_height = int(np.ceil((max_y - min_y) / resolution))
# #         fused_grid = -1 * np.ones((new_height, new_width), dtype=np.int8)
# #         def reproject_map(grid_msg, fused_grid, new_origin_x, new_origin_y, resolution):
# #             info = grid_msg.info
# #             grid_array = np.array(grid_msg.data, dtype=np.int8).reshape((info.height, info.width))
# #             for j in range(info.height):
# #                 for i in range(info.width):
# #                     x = info.origin.position.x + (i + 0.5) * resolution
# #                     y = info.origin.position.y + (j + 0.5) * resolution
# #                     new_i = int((x - new_origin_x) / resolution)
# #                     new_j = int((y - new_origin_y) / resolution)
# #                     if 0 <= new_i < fused_grid.shape[1] and 0 <= new_j < fused_grid.shape[0]:
# #                         value = grid_array[j, i]
# #                         if value == 100:
# #                             fused_grid[new_j, new_i] = 100
# #                         elif value == 0 and fused_grid[new_j, new_i] != 100:
# #                             fused_grid[new_j, new_i] = 0
# #             return fused_grid
# #         if grid_msg1 is not None:
# #             fused_grid = reproject_map(grid_msg1, fused_grid, min_x, min_y, resolution)
# #         if grid_msg2 is not None:
# #             fused_grid = reproject_map(grid_msg2, fused_grid, min_x, min_y, resolution)
# #         new_info = MapMetaData()
# #         new_info.resolution = resolution
# #         new_info.width = new_width
# #         new_info.height = new_height
# #         new_info.origin.position.x = min_x
# #         new_info.origin.position.y = min_y
# #         new_info.origin.position.z = 0.0
# #         new_info.origin.orientation.w = 1.0
# #         return fused_grid, new_info

# # def main(args=None):
# #     rclpy.init(args=args)
# #     node = RLNavigationNode()
# #     try:
# #         rclpy.spin(node)
# #     except KeyboardInterrupt:
# #         pass
# #     node.destroy_node()
# #     rclpy.shutdown()

# # if __name__ == '__main__':
# #     main()
# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# import math
# import numpy as np
# import heapq
# from nav_msgs.msg import Odometry, OccupancyGrid, Path, MapMetaData
# from geometry_msgs.msg import Pose, PoseStamped, Twist, Point, PoseArray
# from visualization_msgs.msg import Marker
# import threading
# import time

# # Función auxiliar para calcular la distancia Euclidiana
# def distance(p1, p2):
#     return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

# # Función para convertir índices del mapa a coordenadas en el mundo
# def index_to_world(i, j, info):
#     x = info.origin.position.x + (i + 0.5) * info.resolution
#     y = info.origin.position.y + (j + 0.5) * info.resolution
#     return (x, y)

# # Función auxiliar de A* sobre la grilla
# def a_star_planning(costmap, info, start_idx, goal_idx):
#     height, width = costmap.shape
#     open_set = []
#     heapq.heappush(open_set, (0, start_idx))
#     came_from = {}
#     g_score = {start_idx: 0}
#     # Usamos la distancia euclidiana entre las posiciones en el mundo como heurística
#     f_score = {start_idx: distance(index_to_world(start_idx[0], start_idx[1], info),
#                                     index_to_world(goal_idx[0], goal_idx[1], info))}
#     neighbors = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]
#     while open_set:
#         current = heapq.heappop(open_set)[1]
#         if current == goal_idx:
#             path = []
#             while current in came_from:
#                 path.append(current)
#                 current = came_from[current]
#             path.append(start_idx)
#             path.reverse()
#             return path
#         for dx, dy in neighbors:
#             neighbor = (current[0]+dx, current[1]+dy)
#             if neighbor[0] < 0 or neighbor[0] >= width or neighbor[1] < 0 or neighbor[1] >= height:
#                 continue
#             if costmap[neighbor[1], neighbor[0]] == 100:
#                 continue  # Obstáculo
#             tentative_g = g_score[current] + math.hypot(dx, dy)
#             if neighbor not in g_score or tentative_g < g_score[neighbor]:
#                 came_from[neighbor] = current
#                 g_score[neighbor] = tentative_g
#                 neighbor_world = index_to_world(neighbor[0], neighbor[1], info)
#                 h = distance(neighbor_world, index_to_world(goal_idx[0], goal_idx[1], info))
#                 f = tentative_g + h
#                 f_score[neighbor] = f
#                 heapq.heappush(open_set, (f, neighbor))
#     return None

# # Función para extraer fronteras a partir del costmap
# def extract_frontiers(costmap, info):
#     height, width = costmap.shape
#     frontier_cells = []
#     for j in range(height):
#         for i in range(width):
#             if costmap[j, i] == 0:
#                 neighbors = [(di, dj) for di in [-1, 0, 1] for dj in [-1, 0, 1] if not (di == 0 and dj == 0)]
#                 for di, dj in neighbors:
#                     ni, nj = i + di, j + dj
#                     if 0 <= ni < width and 0 <= nj < height:
#                         if costmap[nj, ni] == -1:
#                             frontier_cells.append((i, j))
#                             break
#     clusters = []
#     visited = set()
#     for cell in frontier_cells:
#         if cell in visited:
#             continue
#         cluster = []
#         stack = [cell]
#         while stack:
#             current = stack.pop()
#             if current in visited:
#                 continue
#             visited.add(current)
#             cluster.append(current)
#             ci, cj = current
#             for di in [-1, 0, 1]:
#                 for dj in [-1, 0, 1]:
#                     neighbor = (ci+di, cj+dj)
#                     if neighbor in frontier_cells and neighbor not in visited:
#                         stack.append(neighbor)
#         clusters.append(cluster)
#     centroids = []
#     for cluster in clusters:
#         xs = [i for (i, j) in cluster]
#         ys = [j for (i, j) in cluster]
#         centroid_i = sum(xs) / len(xs)
#         centroid_j = sum(ys) / len(ys)
#         centroids.append(index_to_world(centroid_i, centroid_j, info))
#     return centroids

# class NavigationNode(Node):
#     def __init__(self):
#         super().__init__('navigation_node')
#         # Suscriptores
#         self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
#         self.create_subscription(PoseArray, '/goal', self.goal_callback, 10)
#         self.create_subscription(OccupancyGrid, '/occupancy_grid', self.current_grid_callback, 10)
#         self.create_subscription(OccupancyGrid, '/persistent_dynamic_occupancy_grid', self.memory_grid_callback, 10)
#         self.create_subscription(PoseArray, 'safe_frontier_points_centroid', self.safe_frontier_callback, 10)
  
#         # Publicadores
#         self.path_pub = self.create_publisher(Path, '/global_path_predicted', 10)
#         self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.frontier_marker_pub = self.create_publisher(Marker, '/frontier_markers', 10)
        
        
        
#         # Parámetros de control y planificación
#         self.lookahead_distance = 20.0
#         self.linear_speed = 2.5
#         self.k_pursuit = 3.0
#         # Parámetros del planificador local/global (ya no se usa el muestreo aleatorio)
#         self.goal_threshold = 0.8
#         # Parámetros del costmap
#         self.inflation_radius = 3.0
#         self.cost_weight = 0.1
#         # Variables de estado
#         self.current_global_path = None
#         self.current_path_index = 0
#         self.waypoint_threshold = 2.0
#         self.odom = None
#         self.goal = None
#         self.current_grid = None
#         self.memory_grid = None
#         self.safe_frontier_points=None
#         # Publicación continua del Twist
#         self.last_twist = Twist()
#         self.twist_pub_rate = 20
#         self.start_twist_publisher()
#         # Timer para el control loop
#         self.create_timer(0.1, self.control_loop)
#         self.get_logger().info("Nodo de navegación iniciado con planificador A* global.")

#     def start_twist_publisher(self):
#         thread = threading.Thread(target=self.publish_twist_continuously, daemon=True)
#         thread.start()
#     def publish_twist_continuously(self):
#         rate = 1.0 / self.twist_pub_rate
#         while rclpy.ok():
#             self.cmd_vel_pub.publish(self.last_twist)
#             time.sleep(rate)
#     def odom_callback(self, msg: Odometry):
#         self.odom = msg.pose.pose


#     def safe_frontier_callback(self, msg: PoseArray):
#         """Almacena los centroides de los safe frontier points publicados."""
#         if msg.poses:
#             frontier_points = []
#             for p in msg.poses:
#                 frontier_points.append((p.position.x, p.position.y))
#             self.safe_frontier_points = frontier_points
#             self.get_logger().debug("Safe frontier points actualizados.")


#     def goal_callback(self, msg: PoseArray):
#         if msg.poses:
#             self.goal = msg.poses[0]
#             self.get_logger().info("Goal recibido.")
#     def current_grid_callback(self, msg: OccupancyGrid):
#         self.current_grid = msg
#     def memory_grid_callback(self, msg: OccupancyGrid):
#         self.memory_grid = msg
#     def pure_pursuit_control(self, path, current_pos, current_yaw):
#         target_point = None
#         for pt in path:
#             if distance(current_pos, pt) >= self.lookahead_distance:
#                 target_point = pt
#                 break
#         if target_point is None:
#             target_point = path[-1]
#         desired_heading = math.atan2(target_point[1] - current_pos[1],
#                                      target_point[0] - current_pos[0])
#         error_angle = desired_heading - current_yaw
#         error_angle = math.atan2(math.sin(error_angle), math.cos(error_angle))
#         return self.linear_speed, self.k_pursuit * error_angle
#     def smooth_path(self, path):
#         if len(path) < 3:
#             return path
#         smoothed = [path[0]]
#         for i in range(1, len(path)-1):
#             avg_x = (path[i-1][0] + path[i][0] + path[i+1][0]) / 3.0
#             avg_y = (path[i-1][1] + path[i][1] + path[i+1][1]) / 3.0
#             smoothed.append((avg_x, avg_y))
#         smoothed.append(path[-1])
#         return smoothed
#     def create_costmap_from_fused_grid(self, fused_grid, info):
#         costmap = np.copy(fused_grid)
#         inflation_cells = int(math.ceil(self.inflation_radius / info.resolution))
#         height, width = fused_grid.shape
#         for j in range(height):
#             for i in range(width):
#                 if fused_grid[j, i] == 100:
#                     for dj in range(-inflation_cells, inflation_cells+1):
#                         for di in range(-inflation_cells, inflation_cells+1):
#                             ni = i+di
#                             nj = j+dj
#                             if 0 <= ni < width and 0 <= nj < height:
#                                 if math.hypot(di, dj) <= inflation_cells:
#                                     costmap[nj, ni] = 100
#         return costmap
#     def is_line_free(self, grid, info, start, end):
#         def world_to_index(point, info):
#             i = int((point[0]-info.origin.position.x) / info.resolution)
#             j = int((point[1]-info.origin.position.y) / info.resolution)
#             return i, j
#         x0, y0 = world_to_index(start, info)
#         x1, y1 = world_to_index(end, info)
#         dx = abs(x1-x0)
#         dy = abs(y1-y0)
#         x, y = x0, y0
#         sx = -1 if x0 > x1 else 1
#         sy = -1 if y0 > y1 else 1
#         def is_in_bounds(x, y, grid):
#             return 0 <= x < grid.shape[1] and 0 <= y < grid.shape[0]
#         if dx > dy:
#             err = dx/2.0
#             while x != x1:
#                 if not is_in_bounds(x, y, grid) or grid[y, x] == 100:
#                     return False
#                 err -= dy
#                 if err < 0:
#                     y += sy
#                     err += dx
#                 x += sx
#         else:
#             err = dy/2.0
#             while y != y1:
#                 if not is_in_bounds(x, y, grid) or grid[y, x] == 100:
#                     return False
#                 err -= dx
#                 if err < 0:
#                     x += sx
#                     err += dy
#                 y += sy
#         if not is_in_bounds(x, y, grid) or grid[y, x] == 100:
#             return False
#         return True
#     def world_to_index(self, point, info):
#         i = int((point[0]-info.origin.position.x) / info.resolution)
#         j = int((point[1]-info.origin.position.y) / info.resolution)
#         return (i, j)
#     def global_plan_a_star(self, costmap, info, start, goal):
#         start_idx = self.world_to_index(start, info)
#         goal_idx = self.world_to_index(goal, info)
#         grid_path = a_star_planning(costmap, info, start_idx, goal_idx)
#         if grid_path is None:
#             self.get_logger().error("A* no encontró path.")
#             return None
#         path = [index_to_world(i, j, info) for (i,j) in grid_path]
#         return path
#     def publicar_frontiers(self, centroids, info):
#         marker = Marker()
#         marker.header.frame_id = "map"
#         marker.header.stamp = self.get_clock().now().to_msg()
#         marker.ns = "frontiers"
#         marker.id = 0
#         marker.type = Marker.POINTS
#         marker.action = Marker.ADD
#         marker.scale.x = 0.2
#         marker.scale.y = 0.2
#         marker.color.r = 1.0
#         marker.color.g = 0.0
#         marker.color.b = 0.0
#         marker.color.a = 1.0
#         for pt in centroids:
#             p = Point()
#             p.x, p.y = pt[0], pt[1]
#             p.z = 0.0
#             marker.points.append(p)
#         self.frontier_marker_pub.publish(marker)

#     def publish_path(self, path, publisher, label):
#         path_msg = Path()
#         path_msg.header.stamp = self.get_clock().now().to_msg()
#         path_msg.header.frame_id = "map"
#         for pt in path:
#             pose_st = PoseStamped()
#             pose_st.header = path_msg.header
#             pose_st.pose.position.x = pt[0]
#             pose_st.pose.position.y = pt[1]
#             pose_st.pose.position.z = 0.0
#             pose_st.pose.orientation.w = 1.0
#             path_msg.poses.append(pose_st)
#         publisher.publish(path_msg)
#         self.get_logger().info(f"[{label}] Path publicado con {len(path)} puntos.")
#     def update_path_index(self, current_pos):
#         if self.current_global_path is None or len(self.current_global_path) == 0:
#             return
#         old_index = self.current_path_index
#         while (self.current_path_index < len(self.current_global_path) and 
#                distance(current_pos, self.current_global_path[self.current_path_index]) < self.waypoint_threshold):
#             self.get_logger().info(f"Waypoint alcanzado: índice {self.current_path_index}, distancia {distance(current_pos, self.current_global_path[self.current_path_index]):.2f} m")
#             self.current_path_index += 1
#         if self.current_path_index >= len(self.current_global_path):
#             self.current_path_index = len(self.current_global_path)-1
#         if old_index != self.current_path_index:
#             self.get_logger().info(f"Índice actualizado: {old_index} -> {self.current_path_index}")
#     # def control_loop(self):
#     #     if self.odom is None or self.goal is None:
#     #         return
#     #     # Usar current_grid si existe, de lo contrario memory_grid
#     #     grid_msg = self.current_grid if self.current_grid is not None else self.memory_grid
#     #     if grid_msg is None:
#     #         self.get_logger().warn("No se recibió OccupancyGrid aún.")
#     #         return
#     #     info = grid_msg.info
#     #     fused_grid = np.array(grid_msg.data, dtype=np.int8).reshape((info.height, info.width))
#     #     current_pos = (self.odom.position.x, self.odom.position.y)
#     #     goal_pos = (self.goal.position.x, self.goal.position.y)
#     #     costmap = self.create_costmap_from_fused_grid(fused_grid, info)
#     #     frontier_centroids = extract_frontiers(costmap, info)
#     #     self.publicar_frontiers(frontier_centroids, info)
#     #     global_path = self.global_plan_a_star(costmap, info, current_pos, goal_pos)
#     #     if global_path is None or len(global_path) < 2:
#     #         self.get_logger().error("El planificador A* global no logró generar un path válido.")
#     #         return
#     #     self.current_global_path = self.smooth_path(global_path)
#     #     self.current_path_index = 0
#     #     self.publish_path(self.current_global_path, self.path_pub, "Global")
#     #     self.update_path_index(current_pos)
#     #     effective_path = self.current_global_path[self.current_path_index:] if self.current_global_path else None
#     #     q = self.odom.orientation
#     #     siny_cosp = 2.0 * (q.w*q.z + q.x*q.y)
#     #     cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
#     #     current_yaw = math.atan2(siny_cosp, cosy_cosp)
#     #     linear_speed, angular_speed = self.pure_pursuit_control(effective_path, current_pos, current_yaw)
#     #     twist = Twist()
#     #     twist.linear.x = linear_speed
#     #     twist.angular.z = angular_speed
#     #     self.cmd_vel_pub.publish(twist)
#     #     self.get_logger().info(f"Twist publicado: linear = {linear_speed:.2f} m/s, angular = {angular_speed:.2f} rad/s")


#     def create_costmap_from_fused_grid(self, fused_grid, info):
#         # Ajuste del inflado basado en el robot:
#         # Se multiplica el radio de inflado por un factor que considere la huella del robot.
#         # Por ejemplo, factor_robot = 1.5 o basado en parámetros externos.
#         factor_robot = 1.5  # Modificar según dimensiones del robot
#         inflation_cells = int(math.ceil((self.inflation_radius * factor_robot) / info.resolution))
#         costmap = np.copy(fused_grid)
#         height, width = fused_grid.shape
#         for j in range(height):
#             for i in range(width):
#                 if fused_grid[j, i] == 100:
#                     for dj in range(-inflation_cells, inflation_cells+1):
#                         for di in range(-inflation_cells, inflation_cells+1):
#                             ni = i+di
#                             nj = j+dj
#                             if 0 <= ni < width and 0 <= nj < height:
#                                 if math.hypot(di, dj) <= inflation_cells:
#                                     costmap[nj, ni] = 100
#         return costmap

#     def modify_path_for_robot(self, path, costmap, info):
#         """
#         Ajusta los waypoints del path global obtenido para que consideren:
#           - La posición actual y la dirección del robot.
#           - Las dimensiones o huella del robot (por ejemplo, un círculo de radio robot_radius).
#           - Evitar que el camino se acerque demasiado a obstáculos.
          
#         Estrategia:
#           - Recorrer el path y para cada punto se verifica si un "buffer" alrededor (por ejemplo, de radio robot_radius)
#             está libre de obstáculos utilizando la función is_line_free modificada o una verificación puntual.
#           - Si se detecta cercanía a un obstáculo, el waypoint se desplaza lateralmente (o se elimina) tratando de 
#             conseguir un punto alternativo seguro. Esto se puede hacer de manera iterativa.
#         """
#         robot_radius = 4.0  # Radio efectivo del robot en metros; ajustar según el robot
#         path_mod = [path[0]]  # Siempre se conserva el primer punto (la posición actual)
#         for i in range(1, len(path)):
#             pt = path[i]
#             # Se verifica que en el entorno del waypoint (por ejemplo, en 8 direcciones) no haya obstáculos
#             # utilizando una simple verificación del costmap. Se obtiene la posición en índice:
#             cell = self.world_to_index(pt, info)
#             # Si el entorno (un cuadrado centrado en cell) tiene obstáculos, se puede intentar desplazar el punto.
#             clear = True
#             margin = int(math.ceil(robot_radius / info.resolution))
#             for dj in range(-margin, margin + 1):
#                 for di in range(-margin, margin + 1):
#                     ni = cell[0] + di
#                     nj = cell[1] + dj
#                     # Verifica límites y obstáculo
#                     if 0 <= ni < costmap.shape[1] and 0 <= nj < costmap.shape[0]:
#                         if costmap[nj, ni] == 100:
#                             clear = False
#                             break
#                 if not clear:
#                     break
#             if clear:
#                 path_mod.append(pt)
#             else:
#                 # Si no es claro, se intenta desplazar el punto lateralmente
#                 # La dirección de desplazamiento puede basarse en la dirección del segmento actual
#                 prev_pt = path_mod[-1]
#                 dir_vector = (pt[0]-prev_pt[0], pt[1]-prev_pt[1])
#                 # Obtener perpendicular al segmento
#                 perp_vector = (-dir_vector[1], dir_vector[0])
#                 norm = math.hypot(perp_vector[0], perp_vector[1]) or 1.0
#                 perp_unit = (perp_vector[0] / norm, perp_vector[1] / norm)
#                 offset = robot_radius * 0.5
#                 # Se prueba un offset positivo y negativo
#                 candidate1 = (pt[0] + perp_unit[0]*offset, pt[1] + perp_unit[1]*offset)
#                 candidate2 = (pt[0] - perp_unit[0]*offset, pt[1] - perp_unit[1]*offset)
#                 # Se verifica cual candidato queda libre
#                 candidate = pt  # Por defecto, se queda igual
#                 for cand in [candidate1, candidate2]:
#                     cand_cell = self.world_to_index(cand, info)
#                     buffer_clear = True
#                     for dj in range(-margin, margin + 1):
#                         for di in range(-margin, margin + 1):
#                             ni = cand_cell[0] + di
#                             nj = cand_cell[1] + dj
#                             if 0 <= ni < costmap.shape[1] and 0 <= nj < costmap.shape[0]:
#                                 if costmap[nj, ni] == 100:
#                                     buffer_clear = False
#                                     break
#                         if not buffer_clear:
#                             break
#                     if buffer_clear:
#                         candidate = cand
#                         break
#                 path_mod.append(candidate)
#         return path_mod

#     def is_goal_known(self, goal_pos, costmap, info):
#         """Determina si la posición del goal se encuentra en zona conocida (celda diferente a -1)."""
#         i, j = self.world_to_index(goal_pos, info)
#         if 0 <= i < costmap.shape[1] and 0 <= j < costmap.shape[0]:
#             # Si el valor en costmap es -1, se considera desconocido.
#             return costmap[j, i] != -1
#         return False

#     def select_intermediate_goal(self, current_pos, goal_pos):
#         """
#         Selecciona un safe frontier point intermedio que ayude al robot a acercarse al goal.
#         Por ejemplo, se puede elegir aquel punto que minimice la distancia total:
#             distance(current_pos, frontier) + distance(frontier, goal_pos)
#         """
#         if self.safe_frontier_points is None or len(self.safe_frontier_points) == 0:
#             return None
#         best_point = None
#         best_cost = float('inf')
#         for pt in self.safe_frontier_points:
#             cost = distance(current_pos, pt) + distance(pt, goal_pos)
#             if cost < best_cost:
#                 best_cost = cost
#                 best_point = pt
#         self.get_logger().info(f"Subgoal intermedio seleccionado: {best_point}")
#         return best_point

#     def control_loop(self):
#         if self.odom is None or self.goal is None:
#             return
#         grid_msg = self.current_grid if self.current_grid is not None else self.memory_grid
#         if grid_msg is None:
#             self.get_logger().warn("No se recibió OccupancyGrid aún.")
#             return

#         info = grid_msg.info
#         fused_grid = np.array(grid_msg.data, dtype=np.int8).reshape((info.height, info.width))
#         current_pos = (self.odom.position.x, self.odom.position.y)
#         goal_pos = (self.goal.position.x, self.goal.position.y)
#         costmap = self.create_costmap_from_fused_grid(fused_grid, info)
#         frontier_centroids = extract_frontiers(costmap, info)
#         self.publicar_frontiers(frontier_centroids, info)

#         # Verifica si el goal se encuentra en zona conocida
#         if not self.is_goal_known(goal_pos, costmap, info):
#             # Si el goal es desconocido, buscamos un subgoal intermedio.
#             self.get_logger().warn("Goal fuera del área mapeada. Usando safe frontier como subgoal.")
#             intermediate_goal = self.select_intermediate_goal(current_pos, goal_pos)
#             if intermediate_goal is None:
#                 self.get_logger().error("No se ha podido seleccionar un subgoal intermedio.")
#                 return
#             target_goal = intermediate_goal
#         else:
#             target_goal = goal_pos

#         # Se planifica hacia el goal actual (o subgoal intermedio) utilizando A*
#         if self.current_global_path is None or not self.is_path_valid(self.current_global_path, costmap, info):
#             new_path = self.global_plan_a_star(costmap, info, current_pos, target_goal)
#             if new_path is None or len(new_path) < 2:
#                 self.get_logger().error("El planificador global no logró generar un path válido.")
#                 return
#             new_path = self.smooth_path(new_path)
#             self.current_global_path = self.modify_path_for_robot(new_path, costmap, info)
#             self.current_path_index = 0
#             self.publish_path(self.current_global_path, self.path_pub, "Global")
#         else:
#             self.get_logger().info("Usando el global_path previamente calculado.")

#         self.update_path_index(current_pos)
#         effective_path = self.current_global_path[self.current_path_index:] if self.current_global_path else None
#         if effective_path is None or len(effective_path) == 0:
#             self.get_logger().warn("No se encontró un effective_path válido.")
#             return
#         q = self.odom.orientation
#         siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
#         cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
#         current_yaw = math.atan2(siny_cosp, cosy_cosp)
#         linear_speed, angular_speed = self.pure_pursuit_control(effective_path, current_pos, current_yaw)
#         twist = Twist()
#         twist.linear.x = linear_speed
#         twist.angular.z = angular_speed
#         self.cmd_vel_pub.publish(twist)
#         self.get_logger().info(f"Twist publicado: linear = {linear_speed:.2f} m/s, angular = {angular_speed:.2f} rad/s")


#     def is_path_valid(self, path, costmap, info):
#         """
#         Verifica la validez del path actual comprobando que la línea entre el robot y
#         el siguiente waypoint (o varios puntos representativos) esté libre de obstáculos.
#         Se puede ampliar esta función para revisar segmentos clave del camino.
#         """
#         # Por ejemplo, comprobamos la validez del segmento al siguiente waypoint:
#         if len(path) < self.current_path_index + 1:
#             return False  # No hay waypoints suficientes
#         next_waypoint = path[self.current_path_index]
#         # Se utiliza is_line_free para validar la trayectoria
#         if not self.is_line_free(costmap, info, (self.odom.position.x, self.odom.position.y), next_waypoint):
#             self.get_logger().warn("Segmento del path obstruido, replanificando...")
#             return False
#         # También se puede iterar sobre varios puntos del path (por ejemplo, cada N puntos)
#         # para tener una verificación más robusta.
#         return True




# def main(args=None):
#     rclpy.init(args=args)
#     node = NavigationNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()
# if __name__ == '__main__':
#     main()


##############

#por ahora es el main el de aqui arriba


###########



import rclpy
from rclpy.node import Node
import math
import numpy as np
import heapq
import threading
import time
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import Pose, PoseStamped, Twist, Point, PoseArray
from visualization_msgs.msg import Marker
from std_msgs.msg import Float64, Float64MultiArray, Bool

### FUNCIONES AUXILIARES COMUNES ###

def distance(p1, p2):
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

def index_to_world(i, j, info):
    x = info.origin.position.x + (i + 0.5) * info.resolution
    y = info.origin.position.y + (j + 0.5) * info.resolution
    return (x, y)

def planificador_modificable(costmap, info, start, goal, max_iters=1000):
    """
    Genera un path desde 'start' hasta 'goal' sin usar A*, Dijkstra, RRT, etc., 
    basándose en una búsqueda voraz (greedy) incremental. Permite modificar la
    heurística para mayor flexibilidad.
    
    Parámetros:
      - costmap: array 2D con los costos (por ejemplo, 100 para obstáculo, 0 para libre).
      - info: metadatos del OccupancyGrid (tamaño, resolución, origen).
      - start: posición inicial (x, y) en coordenadas del mundo.
      - goal: posición objetivo (x, y) en coordenadas del mundo.
      - max_iters: cantidad máxima de iteraciones para evitar loops infinitos.
    
    Retorna una lista de waypoints [(x1,y1), (x2,y2), ...] que forman el path.
    """
    # Convertir posiciones inicial y final a índices en la grilla.
    start_idx = (int((start[0]-info.origin.position.x) / info.resolution),
                 int((start[1]-info.origin.position.y) / info.resolution))
    goal_idx = (int((goal[0]-info.origin.position.x) / info.resolution),
                int((goal[1]-info.origin.position.y) / info.resolution))
    
    # Inicialización del path con el start (en coordenadas del mundo).
    path = [start]
    
    # Parámetros de la función heurística:
    alpha = 1.0   # peso para la distancia al goal
    beta = 10.0   # peso para penalizar el costo de la celda (por ejemplo, obstáculo = 100)
    
    # Condición de parada: cuando el robot esté suficientemente cerca del goal.
    threshold = 1.0  # distancia en metros

    current_idx = start_idx
    iters = 0
    
    # Lista de vecinos en 8-direcciones
    vecinos = [(-1, 0), (1, 0), (0, -1), (0, 1),
               (-1, -1), (-1, 1), (1, -1), (1, 1)]
    
    while distance(index_to_world(current_idx[0], current_idx[1], info), goal) > threshold and iters < max_iters:
        iters += 1
        candidatos = []
        current_world = index_to_world(current_idx[0], current_idx[1], info)
        
        # Evaluar vecinos
        for dx, dy in vecinos:
            ni = current_idx[0] + dx
            nj = current_idx[1] + dy
            
            # Verificar límites de la grilla
            if ni < 0 or ni >= info.width or nj < 0 or nj >= info.height:
                continue
            
            # Evitar celdas con obstáculo (valor 100) – se puede permitir celdas con valores bajos
            if costmap[nj, ni] >= 100:
                continue
                
            vecino_world = index_to_world(ni, nj, info)
            # Distancia al goal en coordenadas del mundo
            d_goal = distance(vecino_world, goal)
            # Costo de la celda (modificar si se desea, por ejemplo, multiplicar o agregar restricciones)
            costo_celda = costmap[nj, ni]
            
            # Función heurística: se puede modificar para que la celda resulte menos atractiva
            # Si se quiere más modificable, se puede incluir un término que dependa de otros factores.
            h = alpha * d_goal + beta * costo_celda
            
            candidatos.append(((ni, nj), h, vecino_world))
        
        if not candidatos:
            # Si no hay candidatos, se rompe o se podría replanificar con otra estrategia
            print("No hay vecinos viables, rompiendo la búsqueda")
            break
        
        # Seleccionar el vecino con el mínimo valor heurístico
        candidatos.sort(key=lambda x: x[1])
        next_idx, _, next_world = candidatos[0]
        
        # Evitar bucles (opcional: se puede agregar verificación para no regresar a celdas ya visitadas)
        if next_idx == current_idx:
            break
        
        path.append(next_world)
        current_idx = next_idx

    return path

def a_star_planning(costmap, info, start_idx, goal_idx):
    height, width = costmap.shape
    open_set = []
    heapq.heappush(open_set, (0, start_idx))
    came_from = {}
    g_score = {start_idx: 0}
    f_score = {start_idx: distance(index_to_world(start_idx[0], start_idx[1], info),
                                    index_to_world(goal_idx[0], goal_idx[1], info))}
    neighbors = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]
    while open_set:
        current = heapq.heappop(open_set)[1]
        if current == goal_idx:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start_idx)
            path.reverse()
            return path
        for dx, dy in neighbors:
            neighbor = (current[0] + dx, current[1] + dy)
            if neighbor[0] < 0 or neighbor[0] >= width or neighbor[1] < 0 or neighbor[1] >= height:
                continue
            if costmap[neighbor[1], neighbor[0]] == 100:
                continue  # obstáculo
            tentative_g = g_score[current] + math.hypot(dx, dy)
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                neighbor_world = index_to_world(neighbor[0], neighbor[1], info)
                h = distance(neighbor_world, index_to_world(goal_idx[0], goal_idx[1], info))
                f = tentative_g + h
                f_score[neighbor] = f
                heapq.heappush(open_set, (f, neighbor))
    return None

def extract_frontiers(costmap, info):
    height, width = costmap.shape
    frontier_cells = []
    for j in range(height):
        for i in range(width):
            if costmap[j, i] == 0:
                for di in [-1, 0, 1]:
                    for dj in [-1, 0, 1]:
                        if di == 0 and dj == 0:
                            continue
                        ni, nj = i + di, j + dj
                        if 0 <= ni < width and 0 <= nj < height:
                            if costmap[nj, ni] == -1:
                                frontier_cells.append((i, j))
                                break
    clusters = []
    visited = set()
    for cell in frontier_cells:
        if cell in visited:
            continue
        cluster = []
        stack = [cell]
        while stack:
            current = stack.pop()
            if current in visited:
                continue
            visited.add(current)
            cluster.append(current)
            ci, cj = current
            for di in [-1, 0, 1]:
                for dj in [-1, 0, 1]:
                    neighbor = (ci + di, cj + dj)
                    if neighbor in frontier_cells and neighbor not in visited:
                        stack.append(neighbor)
        clusters.append(cluster)
    centroids = []
    for cluster in clusters:
        xs = [i for i, j in cluster]
        ys = [j for i, j in cluster]
        centroid_i = sum(xs) / len(xs)
        centroid_j = sum(ys) / len(ys)
        centroids.append(index_to_world(centroid_i, centroid_j, info))
    return centroids

### NODO DE NAVEGACIÓN CON ACTUALIZACIÓN DINÁMICA DEL PATH ###

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        # Subscripciones
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseArray, '/goal', self.goal_callback, 10)
        self.create_subscription(OccupancyGrid, '/occupancy_grid', self.current_grid_callback, 10)
        self.create_subscription(OccupancyGrid, '/persistent_dynamic_occupancy_grid', self.memory_grid_callback, 10)
        self.create_subscription(PoseArray, 'safe_frontier_points_centroid', self.safe_frontier_callback, 10)
                
        self.create_subscription(Bool,'/goal_reached',self.goal_reached_callback,10)
        # Publicadores: se utiliza el publicador de Path y un Marker para mostrar el segmento actual
        self.path_pub = self.create_publisher(Path, '/global_path_predicted', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.frontier_marker_pub = self.create_publisher(Marker, '/frontier_markers', 10)
        
        # Parámetros de planificación y control
        self.lookahead_distance = 10.0  # Se reduce para visualizar un segmento más cercano
        self.linear_speed = 2.5
        self.k_pursuit = 3.0
        self.goal_threshold = 2.0
        self.inflation_radius = 0.5
        self.cost_weight = 0.1

        # Variables de estado
        self.current_global_path = []  # Se actualizará de forma incremental
        self.current_path_index = 0
        self.waypoint_threshold = 0.8
        self.last_replan_position = None  # Para evaluar si se ha avanzado lo suficiente
        self.odom = None
        self.goal = None
        self.goal_reached = None
        self.current_grid = None
        self.memory_grid = None
        self.safe_frontier_points = []  # Lista persistente de safe frontier points
        self.current_subgoal = None  # Almacena el subgoal actual (intermedio o final)

        # Parámetros para eliminación de frontier alcanzados
        self.frontier_reached_threshold = 1.5

        self.last_twist = Twist()
        self.twist_pub_rate = 20
        self.start_twist_publisher()
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Nodo de navegación (dinámico) iniciado.")

    # ---------- CALLBACKS ------------
    def odom_callback(self, msg):
        self.odom = msg.pose.pose

    # def goal_callback(self, msg: PoseArray):
    #     if msg.poses:
    #         self.goal = msg.poses[0]
    #         self.get_logger().info("Goal recibido.")

    def current_grid_callback(self, msg: OccupancyGrid):
        self.current_grid = msg

    def memory_grid_callback(self, msg: OccupancyGrid):
        self.memory_grid = msg

    def goal_callback(self, msg: PoseArray):
        if msg.poses:
            self.goal = msg.poses[0]
            self.goal_reached = False  # Resetear la bandera al recibir un nuevo goal
            self.get_logger().info("Goal recibido.")

    # def goal_reached_callback(self, msg: Bool):
    #     if msg.data:
    #         # Solo consideramos que se ha alcanzado el goal si la posición actual está cerca del goal final.
    #         if self.odom is not None and self.goal is not None:
    #             current_pos = (self.odom.position.x, self.odom.position.y)
    #             goal_pos = (self.goal.position.x, self.goal.position.y)
    #             if distance(current_pos, goal_pos) < self.goal_threshold:
    #                 self.get_logger().info("Goal global alcanzado. Reiniciando path y safe frontier points.")
    #                 self.current_global_path = []
    #                 self.current_path_index = 0
    #                 self.safe_frontier_points.clear()
    #                 self.persistent_safe_frontiers.clear()
    #                 self.goal_reached = True
    #             else:
    #                 self.get_logger().info("Mensaje goal_reached recibido, pero el robot aún no está cerca del goal global.")
    #         else:
    #             self.get_logger().warn("No se tiene odometría o goal para comparar en goal_reached_callback.")

    def goal_reached_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info("Goal alcanzado. Reiniciando path y reiniciando safe frontier points.")
            self.current_global_path = []  # Vaciar el path actual
            self.current_path_index = 0
            self.goal_reached = True


    def safe_frontier_callback(self, msg: PoseArray):
        nuevos = [(p.position.x, p.position.y) for p in msg.poses]
        # Se agregan sólo los candidatos que no existan en la lista actual (distancia > 0.5)
        for pt in nuevos:
            if all(distance(pt, cand) > 0.5 for cand in self.safe_frontier_points):
                self.safe_frontier_points.append(pt)
        self.get_logger().debug(f"Safe frontier points: {len(self.safe_frontier_points)}")

    # ---------- PUBLICADORES Y FUNCIONES AUXILIARES -----------
    def world_to_index(self, point, info):
        i = int((point[0] - info.origin.position.x) / info.resolution)
        j = int((point[1] - info.origin.position.y) / info.resolution)
        return (i, j)

    def publish_path(self, path, publisher, label):
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
        publisher.publish(path_msg)
        self.get_logger().info(f"[{label}] Path publicado con {len(path)} puntos.")


    def publish_segment_marker(self, segment):
        """Publica un Marker con la línea (segmento) que sale del robot a medida que avanza."""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "path_segment"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.15  # Grosor de la línea
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        for pt in segment:
            p = Point()
            p.x = pt[0]
            p.y = pt[1]
            p.z = 0.0
            marker.points.append(p)
        self.frontier_marker_pub.publish(marker)

    def start_twist_publisher(self):
        thread = threading.Thread(target=self.publish_twist_continuously, daemon=True)
        thread.start()

    def publish_twist_continuously(self):
        rate = 1.0 / self.twist_pub_rate
        while rclpy.ok():
            self.cmd_vel_pub.publish(self.last_twist)
            time.sleep(rate)

    def create_costmap_from_fused_grid(self, fused_grid, info):
        # Inflado adaptado a la dimensión del robot
        factor_robot = 1.5
        inflation_cells = int(math.ceil((self.inflation_radius * factor_robot) / info.resolution))
        costmap = np.copy(fused_grid)
        height, width = fused_grid.shape
        for j in range(height):
            for i in range(width):
                if fused_grid[j, i] == 100:
                    for dj in range(-inflation_cells, inflation_cells+1):
                        for di in range(-inflation_cells, inflation_cells+1):
                            ni = i + di
                            nj = j + dj
                            if 0 <= ni < width and 0 <= nj < height:
                                if math.hypot(di, dj) <= inflation_cells:
                                    costmap[nj, ni] = 100
        return costmap

    def is_line_free(self, grid, info, start, end):
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
        def is_in_bounds(x, y, grid):
            return 0 <= x < grid.shape[1] and 0 <= y < grid.shape[0]
        if dx > dy:
            err = dx / 2.0
            while x != x1:
                if not is_in_bounds(x, y, grid) or grid[y, x] == 100:
                    return False
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                if not is_in_bounds(x, y, grid) or grid[y, x] == 100:
                    return False
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        if not is_in_bounds(x, y, grid) or grid[y, x] == 100:
            return False
        return True

    def remove_reached_frontiers(self, current_pos):
        nuevos = []
        for pt in self.safe_frontier_points:
            if distance(current_pos, pt) > self.frontier_reached_threshold:
                nuevos.append(pt)
            else:
                self.get_logger().info(f"Frontier alcanzada y removida: {pt}")
        self.safe_frontier_points = nuevos

    def select_intermediate_goal(self, current_pos, goal_pos):
        """Selecciona rápidamente un subgoal de los safe frontier candidates."""
        if not self.safe_frontier_points:
            return None
        # Se usa la suma de distancias como heurística
        best_point = min(self.safe_frontier_points,
                         key=lambda pt: distance(current_pos, pt) + distance(pt, goal_pos))
        self.get_logger().info(f"Subgoal intermedio seleccionado: {best_point}")
        return best_point

    def is_goal_known(self, goal_pos, costmap, info):
        i, j = self.world_to_index(goal_pos, info)
        if 0 <= i < costmap.shape[1] and 0 <= j < costmap.shape[0]:
            return costmap[j, i] != -1
        return False

    def global_plan_a_star(self, costmap, info, start, goal):
        start_idx = self.world_to_index(start, info)
        goal_idx = self.world_to_index(goal, info)
        grid_path = a_star_planning(costmap, info, start_idx, goal_idx)
        if grid_path is None:
            self.get_logger().error("A* no encontró path.")
            return None
        path = [index_to_world(i, j, info) for (i, j) in grid_path]
        return path

    def update_path_with_current_position(self, current_pos):
        # Encontrar el punto del path que sea el más cercano a la posición actual
        min_dist = float('inf')
        index_closest = 0
        for i, pt in enumerate(self.current_global_path):
            d = distance(current_pos, pt)
            if d < min_dist:
                min_dist = d
                index_closest = i
        # Actualizar el indice para que el "nuevo" path comience desde el punto más cercano
        self.current_path_index = index_closest


    def publicar_frontiers(self, centroids, info):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "frontiers"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        for pt in centroids:
            p = Point()
            p.x, p.y = pt[0], pt[1]
            p.z = 0.0
            marker.points.append(p)
        self.frontier_marker_pub.publish(marker)

    # ---------- CICLO DE CONTROL -----------

    # def control_loop(self):
    #     if self.odom is None or self.goal is None:
    #         return

    #     grid_msg = self.current_grid if self.current_grid is not None else self.memory_grid
    #     if grid_msg is None:
    #         self.get_logger().warn("No se recibió OccupancyGrid aún.")
    #         return

    #     info = grid_msg.info
    #     fused_grid = np.array(grid_msg.data, dtype=np.int8).reshape((info.height, info.width))
    #     current_pos = (self.odom.position.x, self.odom.position.y)
    #     goal_pos = (self.goal.position.x, self.goal.position.y)
    #     costmap = self.create_costmap_from_fused_grid(fused_grid, info)

    #     # Actualizar lista de safe frontier points y eliminar alcanzados.
    #     self.remove_reached_frontiers(current_pos)

    #     # Revisamos la bandera goal_reached solo si se alcanzó el goal global.
    #     if self.goal_reached:
    #         self.get_logger().info("Goal ya alcanzado. Deteniendo navegación.")
    #         return

    #     # Determinar el objetivo a planificar: si el goal global no es conocido, usar subgoal.
    #     if not self.is_goal_known(goal_pos, costmap, info):
    #         self.get_logger().warn("Goal fuera del área mapeada. Buscando subgoal.")
    #         subgoal = self.select_intermediate_goal(current_pos, goal_pos)
    #         if subgoal is None:
    #             self.get_logger().error("No se ha podido seleccionar un subgoal intermedio.")
    #             return
    #         self.current_subgoal = subgoal
    #     else:
    #         self.current_subgoal = goal_pos

    #     # ... (resto de la replanificación y seguimiento)
    #     # Por ejemplo, forzar replanificación si el path está vacío:
    #     if not self.current_global_path:
    #         new_path = planificador_modificable(costmap, info, current_pos, self.current_subgoal)
    #         if not new_path or len(new_path) < 2:
    #             self.get_logger().error("El planificador modificable no generó un path válido.")
    #             return
    #         new_path = self.smooth_path(new_path)
    #         self.current_global_path = new_path
    #         self.current_path_index = 0
    #         self.publish_path(self.current_global_path, self.path_pub, "Global")
    #         self.last_replan_position = current_pos

    #     # Actualizar el path para que se “ancle” a la posición actual
    #     self.update_path_with_current_position(current_pos)

    #     # Si se alcanzó el subgoal actual (pero no el goal global), actualizar y planificar otro subgoal.
    #     if distance(current_pos, self.current_subgoal) < self.frontier_reached_threshold:
    #         self.get_logger().info(f"Subgoal alcanzado: {self.current_subgoal}")
    #         try:
    #             self.safe_frontier_points.remove(self.current_subgoal)
    #         except ValueError:
    #             pass
    #         nuevo_subgoal = self.select_intermediate_goal(current_pos, goal_pos)
    #         if nuevo_subgoal is not None:
    #             self.current_subgoal = nuevo_subgoal
    #             self.get_logger().info(f"Nuevo subgoal seleccionado: {nuevo_subgoal}")
    #             new_path = planificador_modificable(costmap, info, current_pos, self.current_subgoal)
    #             if new_path and len(new_path) >= 2:
    #                 new_path = self.smooth_path(new_path)
    #                 self.current_global_path = new_path
    #                 self.current_path_index = 0
    #                 self.publish_path(self.current_global_path, self.path_pub, "Global")
    #                 self.last_replan_position = current_pos

    #     # Publicar el segmento actual para visualización
    #     segment = self.current_global_path[self.current_path_index:] if self.current_global_path else []
    #     self.publish_segment_marker(segment)

    #     # Control de seguimiento (pure pursuit)
    #     if not segment or len(segment) == 0:
    #         self.get_logger().warn("No se encontró un segmento válido de path.")
    #         return

    #     q = self.odom.orientation
    #     siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    #     cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    #     current_yaw = math.atan2(siny_cosp, cosy_cosp)
    #     linear_speed, angular_speed = self.pure_pursuit_control(segment, current_pos, current_yaw)
    #     twist = Twist()
    #     twist.linear.x = linear_speed
    #     twist.angular.z = angular_speed
    #     self.last_twist = twist
    #     self.get_logger().info(f"Twist: {linear_speed:.2f} m/s, {angular_speed:.2f} rad/s")

    def control_loop(self):
        if self.odom is None or self.goal is None:
            return

        grid_msg = self.current_grid if self.current_grid is not None else self.memory_grid
        if grid_msg is None:
            self.get_logger().warn("No se recibió OccupancyGrid aún.")
            return

        info = grid_msg.info
        fused_grid = np.array(grid_msg.data, dtype=np.int8).reshape((info.height, info.width))
        current_pos = (self.odom.position.x, self.odom.position.y)
        goal_pos = (self.goal.position.x, self.goal.position.y)
        costmap = self.create_costmap_from_fused_grid(fused_grid, info)

        # Actualizar lista de safe frontier points y eliminar alcanzados.
        self.remove_reached_frontiers(current_pos)

        # Si se ha alcanzado el goal, y se limpió el path, se evita seguir intentando alcanzar el antiguo goal.
        if self.goal_reached:
            self.get_logger().info("Goal ya alcanzado. Deteniendo navegación.")
            # Puedes decidir reiniciar o esperar a recibir otro goal.
            return



        # Determinar el objetivo (goal o subgoal) de acuerdo a la visibilidad o disponibilidad.
        if not self.is_goal_known(goal_pos, costmap, info):
            self.get_logger().warn("Goal fuera del área mapeada. Buscando subgoal.")
            subgoal = self.select_intermediate_goal(current_pos, goal_pos)
            if subgoal is None:
                self.get_logger().error("No se ha podido seleccionar un subgoal intermedio.")
                return
            self.current_subgoal = subgoal
        else:
            self.current_subgoal = goal_pos

        # Forzar replanificación si se ha desviado mucho o si es la primera vez.
        if not self.current_global_path or (self.last_replan_position and distance(current_pos, self.last_replan_position) > self.lookahead_distance * 0.5):
            #new_path = self.global_plan_a_star(costmap, info, current_pos, self.current_subgoal)
            new_path = planificador_modificable(costmap, info, current_pos, self.current_subgoal)
            if not new_path or len(new_path) < 2:
                self.get_logger().error("El planificador modificable no generó un path válido.")
                return
            new_path = self.smooth_path(new_path)
            self.current_global_path = new_path
            self.current_path_index = 0
            self.publish_path(self.current_global_path, self.path_pub, "Global")
            self.last_replan_position = current_pos

        # Actualizar el path para que "siga" la posición actual del robot.
        self.update_path_with_current_position(current_pos)

        # Si se llegó al subgoal actual, se elimina de la lista y se selecciona otro.
        if distance(current_pos, self.current_subgoal) < self.frontier_reached_threshold:
            self.get_logger().info(f"Subgoal alcanzado: {self.current_subgoal}")
            try:
                self.safe_frontier_points.remove(self.current_subgoal)
            except ValueError:
                pass
            nuevo_subgoal = self.select_intermediate_goal(current_pos, goal_pos)
            if nuevo_subgoal is not None:
                self.current_subgoal = nuevo_subgoal
                self.get_logger().info(f"Nuevo subgoal seleccionado: {nuevo_subgoal}")
                # Forzamos la replanificación en este caso también.
                new_path = planificador_modificable(costmap, info, current_pos, self.current_subgoal)
                if new_path and len(new_path) >= 2:
                    new_path = self.smooth_path(new_path)
                    self.current_global_path = new_path
                    self.current_path_index = 0
                    self.publish_path(self.current_global_path, self.path_pub, "Global")
                    self.last_replan_position = current_pos

        # Publicar el segmento actual del path (opcional, para visualización)
        segment = self.current_global_path[self.current_path_index:] if self.current_global_path else []
        self.publish_segment_marker(segment)

        # Control de seguimiento (por ejemplo, pure pursuit) sobre el segmento actualizado.
        if not segment or len(segment) == 0:
            self.get_logger().warn("No se encontró un segmento válido de path.")
            return

        q = self.odom.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        current_yaw = math.atan2(siny_cosp, cosy_cosp)
        linear_speed, angular_speed = self.pure_pursuit_control(segment, current_pos, current_yaw)
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        self.last_twist = twist
        self.get_logger().info(f"Twist: {linear_speed:.2f} m/s, {angular_speed:.2f} rad/s")

    def pure_pursuit_control(self, path, current_pos, current_yaw):
        # Si el robot ya está cerca del último punto del path, se detiene.
        final_point = path[-1]
        # if distance(current_pos, final_point) < self.goal_threshold:
        #     self.get_logger().info("Robot cerca del último waypoint; se envía comando de detención.")
        #     return 0.0, 0.0

        # Seleccionar el siguiente punto de referencia según la distancia lookahead
        target_point = None
        for pt in path:
            if distance(current_pos, pt) >= self.lookahead_distance:
                target_point = pt
                break
        if target_point is None:
            target_point = path[-1]
        desired_heading = math.atan2(target_point[1] - current_pos[1],
                                    target_point[0] - current_pos[0])
        error_angle = desired_heading - current_yaw
        # Normalizar el ángulo
        error_angle = math.atan2(math.sin(error_angle), math.cos(error_angle))
        return self.linear_speed, self.k_pursuit * error_angle


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
    
    def update_path_index(self, current_pos):
        if not self.current_global_path or self.current_path_index >= len(self.current_global_path):
            return
        old_index = self.current_path_index
        while (self.current_path_index < len(self.current_global_path) and 
               distance(current_pos, self.current_global_path[self.current_path_index]) < self.waypoint_threshold):
            self.current_path_index += 1
        if self.current_path_index >= len(self.current_global_path):
            self.current_path_index = len(self.current_global_path) - 1
        if old_index != self.current_path_index:
            self.get_logger().info(f"Índice actualizado: {old_index} -> {self.current_path_index}")

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




