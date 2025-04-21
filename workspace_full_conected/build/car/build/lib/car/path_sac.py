
# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# import math
# import threading
# import time
# import numpy as np
# import tensorflow as tf

# from nav_msgs.msg import Odometry, OccupancyGrid, Path
# from geometry_msgs.msg import PoseStamped, Pose, Twist, Point, PoseArray

# # Función auxiliar para calcular la distancia euclidiana.
# def distance(p1, p2):
#     return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

# # -----------------------------------------------------------------------
# # MODELO COMBINADO CON TENSORFLOW  
# #
# # Se construye un modelo que recibe dos entradas:
# # 1. "occupancy": el grid map completo (suponiendo que ya está centrado en el robot)
# # 2. "state": un vector global con [current_x, current_y, target_x, target_y]
# #
# # La salida es una secuencia de "num_waypoints" (cada uno de 2 dimensiones)
# # que se usarán para formar la trayectoria.
# # -----------------------------------------------------------------------
# def build_combined_model(num_waypoints=5, grid_shape=(None, None, 1)):
#     # Entrada 1: Grid map con tamaño variable (por ejemplo, de un sensor local)
#     occupancy_input = tf.keras.Input(shape=grid_shape, name='occupancy')
#     # Aplicar algunas capas convolucionales y pooling
#     x = tf.keras.layers.Conv2D(16, (3, 3), activation='relu', padding='same')(occupancy_input)
#     x = tf.keras.layers.MaxPooling2D((2, 2))(x)
#     x = tf.keras.layers.Conv2D(32, (3, 3), activation='relu', padding='same')(x)
#     x = tf.keras.layers.MaxPooling2D((2, 2))(x)
#     # Global Average Pooling para obtener un vector de características de tamaño fijo,
#     # independientemente del tamaño de la entrada.
#     x = tf.keras.layers.GlobalAveragePooling2D()(x)
#     # Forzamos la forma estática (aquí suponemos que el número de filtros es 32)
#     # Esto convierte x a una forma (batch_size, 32)
#     x = tf.keras.layers.Lambda(lambda t: tf.reshape(t, (-1, 32)))(x)
    
#     # Entrada 2: Vector de estado global (por ejemplo, [current_x, current_y, target_x, target_y])
#     state_input = tf.keras.Input(shape=(4,), name='state')
#     y = tf.keras.layers.Dense(32, activation='relu')(state_input)
    
#     # Fusionar ambas ramas
#     combined = tf.keras.layers.concatenate([x, y])
#     z = tf.keras.layers.Dense(64, activation='relu')(combined)
#     # Salida: predecir num_waypoints * 2 valores (cada waypoint: x, y)
#     output = tf.keras.layers.Dense(num_waypoints * 2, name='output')(z)
    
#     model = tf.keras.Model(inputs=[occupancy_input, state_input], outputs=output)
#     return model

# # -----------------------------------------------------------------------
# # NODO DE ENTRENAMIENTO RL CON LA NUEVA ARQUITECTURA
# #
# # Este nodo se suscribe a los tópicos relevantes:
# # - /odom para la odometría.
# # - /goal para el objetivo.
# # - /occupancy_grid para el grid map (se asume que ya viene centrado y con tamaño fijo).
# # - /safe_frontier_points_centroid para los puntos de frontier (opcional).
# #
# # Además, publica el comando de velocidad (/cmd_vel) y la trayectoria global (/global_path_predicted)
# # para visualización en RViz. La política se actualiza con REINFORCE (muy básico en este ejemplo).
# # -----------------------------------------------------------------------
# class RLTrainingNode(Node):
#     def __init__(self):
#         super().__init__('rl_training_node')
        
#         # Subscripciones
#         self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
#         self.create_subscription(PoseArray, '/goal', self.goal_callback, 10)
#         self.create_subscription(OccupancyGrid, '/occupancy_grid', self.grid_callback, 10)
#         self.create_subscription(PoseArray, '/safe_frontier_points_centroid', self.safe_frontier_callback, 10)
        
#         # Publicadores
#         self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.path_pub = self.create_publisher(Path, '/global_path_predicted', 10)
        

        
#         # Estado interno
#         self.current_pose = None
#         self.goal_pose = None
#         self.current_grid = None   # Mensaje completo del grid map
#         self.grid_data = None      # Versión procesada del grid (numpy)
#         self.safe_frontier_points = []  # Lista de puntos (tuplas x,y)
#         self.last_twist = Twist()
        
#         # Parámetros del controlador pure pursuit
#         self.lookahead_distance = 3.0
#         self.linear_speed = 0.5
#         self.k_pursuit = 1.0
        
#         # Parámetros y modelo para RL
#         self.num_waypoints = 5
#         self.planning_model = build_combined_model(num_waypoints=self.num_waypoints, grid_shape=(None, None, 1))
#         self.planning_model.compile(optimizer=tf.keras.optimizers.Adam(learning_rate=1e-3), loss='mse')
#         self.gamma = 0.99  # Factor de descuento para REINFORCE
#         self.optimizer = tf.keras.optimizers.Adam(learning_rate=1e-3)
#         self.episode_buffer = []
#         self.prev_distance = None
#         self.goal_threshold = 0.5
#         self.episode_count = 0
        
#         # Publicador continuo de twist (20 Hz)
#         self.twist_pub_rate = 20
#         self.start_twist_publisher()


#         # Timer del loop de control (10 Hz)
#         self.create_timer(0.1, self.control_loop)
        
#         self.get_logger().info("Nodo de entrenamiento RL iniciado.")
    
#     def start_twist_publisher(self):
#         thread = threading.Thread(target=self.publish_twist_continuously, daemon=True)
#         thread.start()
        
#     def publish_twist_continuously(self):
#         rate = 1.0 / self.twist_pub_rate
#         while rclpy.ok():
#             self.cmd_vel_pub.publish(self.last_twist)
#             time.sleep(rate)
    
#     # -------------------------------------------------------------------
#     # Callbacks de los tópicos
#     # -------------------------------------------------------------------
#     def odom_callback(self, msg):
#         self.current_pose = msg.pose.pose
#         #self.get_logger().info("Odom recibido.")

#     def goal_callback(self, msg: PoseArray):
#         if msg.poses:
#             self.goal_pose = msg.poses[0]
#             self.goal_reached = False  # Se resetea la bandera para el nuevo goal
#             #self.get_logger().info("Goal recibido.")
    
#     def grid_callback(self, msg: OccupancyGrid):
#         self.current_grid = msg
#         try:
#             self.grid_data = np.array(msg.data, dtype=np.int8).reshape(
#                 (msg.info.height, msg.info.width)
#             )
#             #self.get_logger().info("Grid map procesado y almacenado.")
#         except Exception as e:
#             self.get_logger().error(f"Error procesando grid: {e}")
#             self.grid_data = None
    
#     def safe_frontier_callback(self, msg: PoseArray):
#         nuevos = []
#         for p in msg.poses:
#             pt = (p.position.x, p.position.y)
#             if all(distance(pt, cand) > 0.5 for cand in self.safe_frontier_points):
#                 nuevos.append(pt)
#         if nuevos:
#             self.safe_frontier_points = nuevos
#             self.get_logger().info(f"Safe frontier points actualizados: {len(nuevos)}")
    
#     # -------------------------------------------------------------------
#     # Función para procesar el grid map completo (se asume que ya viene centrado)
#     # -------------------------------------------------------------------
#     def extract_complete_grid(self, grid_msg):
#         if grid_msg is None:
#             self.get_logger().warn("Grid map no recibido; usando mapa por defecto.")
#             return np.ones((20,20,1), dtype=np.float32) * -1
#         info = grid_msg.info
#         grid = np.array(grid_msg.data, dtype=np.int8).reshape((info.height, info.width))
#         # Normalización simple: transforma el rango [-1, 100] a [0, 1]
#         grid_normalized = (grid + 1) / 101.0
#         grid_normalized = np.expand_dims(grid_normalized, axis=-1)
#         return grid_normalized
    
#     # -------------------------------------------------------------------
#     # Controlador Pure Pursuit
#     # -------------------------------------------------------------------
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
    
#     def get_yaw_from_pose(self, pose: Pose):
#         q = pose.orientation
#         siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
#         cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
#         return math.atan2(siny_cosp, cosy_cosp)
    
#     # -------------------------------------------------------------------
#     # Suavizado y publicación del path global
#     # -------------------------------------------------------------------
#     def smooth_path(self, path):
#         if len(path) < 3:
#             return path
#         smoothed = [path[0]]
#         for i in range(1, len(path)-1):
#             # Promediar con ponderaciones o usando un filtro de mediana
#             avg_x = (path[i-1][0] + path[i][0] + path[i+1][0]) / 3.0
#             avg_y = (path[i-1][1] + path[i][1] + path[i+1][1]) / 3.0
#             # Si el punto actual está muy cerca del anterior, omitir o ajustar
#             if distance(smoothed[-1], (avg_x, avg_y)) < 0.2:
#                 continue
#             smoothed.append((avg_x, avg_y))
#         # Asegúrate de incluir el último punto
#         if distance(smoothed[-1], path[-1]) > 0.01:
#             smoothed.append(path[-1])
#         return smoothed
    
#     def is_goal_known(self, goal, grid_msg: OccupancyGrid):
#         # Se convierte la posición del goal a índices en la grilla
#         info = grid_msg.info
#         i = int((goal[0] - info.origin.position.x) / info.resolution)
#         j = int((goal[1] - info.origin.position.y) / info.resolution)
#         if i < 0 or i >= info.width or j < 0 or j >= info.height:
#             return False
#         # Por ejemplo, si la celda es -1 (desconocida) se considera no mapeada
#         idx = j * info.width + i
#         return grid_msg.data[idx] != -1

#     def select_subgoal(self, current_pos):
#         if not self.safe_frontier_points:
#             return None
#         # Se escoge, por ejemplo, el frontier más cercano
#         subgoal = min(self.safe_frontier_points, key=lambda pt: distance(current_pos, pt))
#         self.get_logger().info(f"Subgoal seleccionado: {subgoal}")
#         return subgoal



#     def publish_path(self, path):
#         path_msg = Path()
#         path_msg.header.stamp = self.get_clock().now().to_msg()
#         path_msg.header.frame_id = "map"
#         for pt in path:
#             pose_st = PoseStamped()
#             pose_st.header = path_msg.header
#             pose_st.pose.position.x = float(pt[0])
#             pose_st.pose.position.y = float(pt[1])
#             pose_st.pose.position.z = 0.0
#             pose_st.pose.orientation.w = 1.0
#             path_msg.poses.append(pose_st)
#         self.path_pub.publish(path_msg)
#         self.get_logger().info(f"Path publicado con {len(path)} puntos.")
    
#     # -------------------------------------------------------------------
#     # Loop de Control y Entrenamiento (REINFORCE básico)
#     # -------------------------------------------------------------------
#     # def control_loop(self):
#     #     if self.current_pose is None or self.goal_pose is None:
#     #         return
#     #     #self.get_logger().info(f"Posición actual: {self.current_pose.pose.pose.x}, {self.current_pose.pose.pose.y}")
            
#     #     # Se utiliza la grilla actual o la memoria si la actual no está disponible.
#     #     grid_msg = self.current_grid if self.current_grid is not None else self.memory_grid
#     #     if grid_msg is None:
#     #         self.get_logger().warn("No se recibió OccupancyGrid aún.")
#     #         return
        
#     #     current_pos = (self.current_pose.position.x, self.current_pose.position.y)
#     #     goal_pos = (self.goal_pose.position.x, self.goal_pose.position.y)
        
#     #     if self.current_grid is None:
#     #         self.get_logger().warn("Grid map no recibido, esperando...")
#     #         return
        
#     #     # Usar el grid map completo (que ya está centrado) como input
#     #     local_map = self.extract_complete_grid(self.current_grid)
        
#     #     # Aquí, por simplicidad, usamos directamente el goal como target.
#     #     # Se podrían implementar lógicas para seleccionar un subgoal usando frontier points.
#     #     target_pos = goal_pos
        
#     #     if self.prev_distance is None:
#     #         self.prev_distance = distance(current_pos, target_pos)
        
#     #     # Preparar el vector de estado global
#     #     state_vector = np.array([[current_pos[0], current_pos[1], target_pos[0], target_pos[1]]], dtype=np.float32)
        
#     #     # El modelo espera dos entradas: la imagen del grid y el vector de estado.
#     #     model_input = [np.expand_dims(local_map, axis=0), state_vector]
#     #     action_pred = self.planning_model(model_input)
#     #     action_pred = action_pred.numpy().flatten()
        
#     #     # Convertir la predicción en una lista de waypoints
#     #     predicted_waypoints = []
#     #     for i in range(self.num_waypoints):
#     #         x = action_pred[2*i]
#     #         y = action_pred[2*i + 1]
#     #         predicted_waypoints.append((x, y))
        
#     #     # Construir el path global: desde la posición actual, pasando por los waypoints, hasta target.
#     #     global_path = [current_pos] + predicted_waypoints + [target_pos]
#     #     global_path = self.smooth_path(global_path)
#     #     self.publish_path(global_path)
        
#     #     # Control con Pure Pursuit
#     #     current_yaw = self.get_yaw_from_pose(self.current_pose)
#     #     linear, angular = self.pure_pursuit_control(global_path, current_pos, current_yaw)
#     #     twist = Twist()
#     #     twist.linear.x = linear
#     #     twist.angular.z = angular
#     #     self.last_twist = twist
        
#     #     self.get_logger().info(f"Control: linear {linear:.2f} m/s, angular {angular:.2f} rad/s")
        
#     #     # Calcular recompensa basada en el progreso hacia el target
#     #     new_distance = distance(current_pos, target_pos)
#     #     reward = self.prev_distance - new_distance  # Progreso (positivo si se acerca)
#     #     self.prev_distance = new_distance
        
#     #     # Guardar la transición en el buffer del episodio
#     #     # Nota: En esta versión no calculamos log_prob porque la política combinada
#     #     # no emite explícitamente el valor estocástico. En una implementación completa,
#     #     # se debería incluir la parte estocástica y su log_prob.
#     #     self.episode_buffer.append((state_vector, 0.0, reward))
        
#     #     # Si se alcanza el target, se finaliza el episodio y se actualiza la política
#     #     if new_distance < self.goal_threshold:
#     #         reward += 10.0  # Bonus al alcanzar el objetivo.
#     #         self.episode_buffer[-1] = (state_vector, 0.0, reward)
#     #         self.get_logger().info("Episodio terminado. Actualizando política...")
#     #         self.update_policy()
#     #         self.episode_count += 1
#     #         self.get_logger().info(f"Episodios completados: {self.episode_count}")
#     #         self.prev_distance = None
#     #         self.episode_buffer = []



#     def control_loop(self):
#         self.get_logger().info("Entrando a control_loop")
#         if self.current_pose is None:
#             self.get_logger().info("Odom aún no disponible.")
#             return
#         #Comenta temporalmente el if del goal para ver qué hay en self.goal_pose:
#         if self.goal_pose is None:
#             self.get_logger().info("Goal aún no disponible.")
#             return
#         # if self.goal_pose is None:
#         #     self.get_logger().info("Goal aún no disponible, pero se usará el último valor conocido.")
        
#         current_pos = (self.current_pose.position.x, self.current_pose.position.y)
#         goal_pos = (self.goal_pose.position.x, self.goal_pose.position.y) if self.goal_pose is not None else (0,0)
#         self.get_logger().debug(f"Posición actual: {current_pos}, Goal: {goal_pos}")
#         if self.current_grid is None:
#             self.get_logger().warn("Grid map no recibido, esperando...")
#             return

#         # Procesar el grid map completo.
#         local_map = self.extract_complete_grid(self.current_grid)
#         # Log para confirmar la forma y valores (puedes imprimir algunos valores de local_map)
#         self.get_logger().info(f"Local map shape: {local_map.shape}")

#         target_pos = goal_pos
#         if self.prev_distance is None:
#             self.prev_distance = distance(current_pos, target_pos)
        
#         state_vector = np.array([[current_pos[0], current_pos[1], target_pos[0], target_pos[1]]], dtype=np.float32)
#         model_input = [np.expand_dims(local_map, axis=0), state_vector]
        
#         # Predicción del modelo y verificación de los valores:
#         action_pred = self.planning_model(model_input)
#         action_pred = action_pred.numpy().flatten()
#         self.get_logger().info(f"Acción predicha: {action_pred}")

#         predicted_waypoints = []
#         for i in range(self.num_waypoints):
#             x = action_pred[2 * i]
#             y = action_pred[2 * i + 1]
#             predicted_waypoints.append((x, y))
#         self.get_logger().info(f"Waypoints predichos: {predicted_waypoints}")
        
#         global_path = [current_pos] + predicted_waypoints + [target_pos]
#         global_path = self.smooth_path(global_path)
#         self.publish_path(global_path)
#         self.get_logger().info(f"Path global: {global_path}")

#         current_yaw = self.get_yaw_from_pose(self.current_pose)
#         linear, angular = self.pure_pursuit_control(global_path, current_pos, current_yaw)
#         self.get_logger().info(f"Control calculado: linear={linear}, angular={angular}")
#         twist = Twist()
#         twist.linear.x = linear
#         twist.angular.z = angular
#         self.last_twist = twist

#         new_distance = distance(current_pos, target_pos)
#         reward = self.prev_distance - new_distance
#         self.prev_distance = new_distance
#         self.episode_buffer.append((state_vector, 0.0, reward))
        
#         if new_distance < self.goal_threshold:
#             reward += 10.0
#             self.episode_buffer[-1] = (state_vector, 0.0, reward)
#             self.get_logger().info("Episodio terminado. Actualizando política...")
#             self.update_policy()
#             self.episode_count += 1
#             self.get_logger().info(f"Episodios completados: {self.episode_count}")
#             self.prev_distance = None
#             self.episode_buffer = []



    
#     # Actualización de la política mediante REINFORCE (simplificado)
#     def update_policy(self):
#         states, log_probs, rewards = [], [], []
#         for (s, lp, r) in self.episode_buffer:
#             states.append(s)
#             log_probs.append(lp)
#             rewards.append(r)
        
#         returns = []
#         R = 0.0
#         for r in reversed(rewards):
#             R = r + self.gamma * R
#             returns.insert(0, R)
        
#         returns = tf.convert_to_tensor(returns, dtype=tf.float32)
#         log_probs = tf.convert_to_tensor(log_probs, dtype=tf.float32)
        
#         # En un escenario real, se recomputarían log_probs de una política estocástica.
#         loss = -tf.reduce_mean(log_probs * returns)
        
#         with tf.GradientTape() as tape:
#             tape.watch(self.planning_model.trainable_variables)
#             loss_value = -tf.reduce_mean(log_probs * returns)
#         grads = tape.gradient(loss_value, self.planning_model.trainable_variables)
#         self.optimizer.apply_gradients(zip(grads, self.planning_model.trainable_variables))
        
#         self.get_logger().info(f"Política actualizada, pérdida: {loss_value.numpy():.3f}")
        
# # -----------------------------------------------------------------------
# # Función principal que inicializa el nodo y lo hace "spin"
# # -----------------------------------------------------------------------
# def main(args=None):
#     rclpy.init(args=args)
#     node = RLTrainingNode()
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
import threading
import time
import numpy as np
import tensorflow as tf

from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose, Twist, Point, PoseArray
from std_msgs.msg import Bool

# --------------------------
# Función auxiliar para calcular la distancia euclidiana.
# --------------------------
def distance(p1, p2):
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

# -----------------------------------------------------------------------
# MODELO COMBINADO CON TENSORFLOW  
#
# Se construye un modelo que recibe dos entradas:
# 1. "occupancy": el grid map completo (con tamaño variable)
# 2. "state": un vector global con [current_x, current_y, target_x, target_y]
#
# La salida es una secuencia de "num_waypoints" (cada uno de 2 dimensiones)
# que se utilizarán para formar la trayectoria.
# Se usa GlobalAveragePooling2D con una capa Lambda para forzar la forma.
# -----------------------------------------------------------------------
def build_combined_model(num_waypoints=5, grid_shape=(None, None, 1)):
    # Entrada 1: Grid map de tamaño variable.
    occupancy_input = tf.keras.Input(shape=grid_shape, name='occupancy')
    # Capas convolucionales y pooling para extraer características espaciales.
    x = tf.keras.layers.Conv2D(16, (3, 3), activation='relu', padding='same')(occupancy_input)
    x = tf.keras.layers.MaxPooling2D((2, 2))(x)
    x = tf.keras.layers.Conv2D(32, (3, 3), activation='relu', padding='same')(x)
    x = tf.keras.layers.MaxPooling2D((2, 2))(x)
    # GlobalAveragePooling2D produce un vector fijo.
    x = tf.keras.layers.GlobalAveragePooling2D()(x)
    # Forzar la forma estática: suponemos que la salida tiene 32 elementos.
    x = tf.keras.layers.Lambda(lambda t: tf.reshape(t, (-1, 32)))(x)
    
    # Entrada 2: Vector de estado global ([current_x, current_y, target_x, target_y]).
    state_input = tf.keras.Input(shape=(4,), name='state')
    y = tf.keras.layers.Dense(32, activation='relu')(state_input)
    
    # Fusionar ambas ramas.
    combined = tf.keras.layers.concatenate([x, y])
    z = tf.keras.layers.Dense(64, activation='relu')(combined)
    # Salida: predice num_waypoints * 2 valores (cada waypoint: x, y).
    output = tf.keras.layers.Dense(num_waypoints * 2, name='output')(z)
    
    model = tf.keras.Model(inputs=[occupancy_input, state_input], outputs=output)
    return model

# -----------------------------------------------------------------------
# NODO DE ENTRENAMIENTO RL CON MEJORAS
#
# Suscribe a:
#   - /odom (odometría)
#   - /goal (objetivo) – se espera un PoseArray; se toma la primera pose.
#   - /occupancy_grid (grid map)
#   - /safe_frontier_points_centroid (puntos frontera)
#   - /goal_reached (flag, opcional)
#
# Publica:
#   - /cmd_vel (comando de velocidad)
#   - /global_path_predicted (trayectoria para visualización)
# -----------------------------------------------------------------------
class RLTrainingNode(Node):
    def __init__(self):
        super().__init__('rl_training_node')
        
        # Suscriptores
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseArray, '/goal', self.goal_callback, 10)
        self.create_subscription(OccupancyGrid, '/occupancy_grid', self.grid_callback, 10)
        self.create_subscription(PoseArray, '/safe_frontier_points_centroid', self.safe_frontier_callback, 10)
        self.create_subscription(Bool, '/goal_reached', self.goal_reached_callback, 10)
        
        # Publicadores
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/global_path_predicted', 10)
        
        # Estado interno
        self.current_pose = None
        self.goal_pose = None  # Se guarda la primera pose del topic /goal
        self.current_grid = None  # Mensaje completo del grid map
        self.grid_data = None
        self.safe_frontier_points = []  # Lista de puntos frontera (tuplas x,y)
        self.last_twist = Twist()
        self.goal_reached = False
        
        # Parámetros del controlador pure pursuit.
        self.lookahead_distance = 3.0  # Aumentado para mirar más lejos.
        self.linear_speed = 0.5
        self.k_pursuit = 1.0
        
        # Parámetros y modelo para RL.
        self.num_waypoints = 5
        # Aquí usamos la arquitectura combinada para aceptar un grid de tamaño variable.
        self.planning_model = build_combined_model(num_waypoints=self.num_waypoints, grid_shape=(None, None, 1))
        self.planning_model.compile(optimizer=tf.keras.optimizers.Adam(learning_rate=1e-3), loss='mse')
        self.gamma = 0.99  # Factor de descuento para REINFORCE.
        self.optimizer = tf.keras.optimizers.Adam(learning_rate=1e-3)
        self.episode_buffer = []
        self.prev_distance = None
        self.goal_threshold = 0.5  # Umbral para terminar episodio.
        self.episode_count = 0
        
        # Publicador continuo del twist (20 Hz).
        self.twist_pub_rate = 20
        self.start_twist_publisher()
        
        # Timer del loop de control (10 Hz).
        self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("Nodo de entrenamiento RL iniciado.")
    
    # -------------------------------------------------------------------
    # Publicador continuo de Twist.
    # -------------------------------------------------------------------
    def start_twist_publisher(self):
        thread = threading.Thread(target=self.publish_twist_continuously, daemon=True)
        thread.start()
    
    def publish_twist_continuously(self):
        rate = 1.0 / self.twist_pub_rate
        while rclpy.ok():
            self.cmd_vel_pub.publish(self.last_twist)
            time.sleep(rate)
    
    # -------------------------------------------------------------------
    # Callbacks de los tópicos.
    # -------------------------------------------------------------------
    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose
        # Se puede loguear de forma breve:
        # self.get_logger().debug("Odom recibido.")
    
    def goal_callback(self, msg: PoseArray):
        if msg.poses:
            self.goal_pose = msg.poses[0]
            self.goal_reached = False
            self.get_logger().info("Goal recibido.")
    
    def grid_callback(self, msg: OccupancyGrid):
        self.current_grid = msg
        try:
            self.grid_data = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
            # self.get_logger().info("Grid map procesado y almacenado.")
        except Exception as e:
            self.get_logger().error(f"Error procesando grid: {e}")
            self.grid_data = None
    
    def safe_frontier_callback(self, msg: PoseArray):
        nuevos = []
        for p in msg.poses:
            pt = (p.position.x, p.position.y)
            if all(distance(pt, cand) > 0.5 for cand in self.safe_frontier_points):
                nuevos.append(pt)
        if nuevos:
            self.safe_frontier_points = nuevos
            self.get_logger().info(f"Frontier points actualizados: {len(nuevos)}")
    
    def goal_reached_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info("Goal alcanzado. Reiniciando path y safe frontier points.")
            self.goal_reached = True
            # Se podría reiniciar otros parámetros si es necesario.
    
    # -------------------------------------------------------------------
    # Función para procesar el grid map completo (se asume que ya viene centrado).
    # Si el grid varía en tamaño, se usa directamente.
    # -------------------------------------------------------------------
    def extract_complete_grid(self, grid_msg):
        if grid_msg is None:
            self.get_logger().warn("Grid map no recibido; usando mapa por defecto.")
            return np.ones((20,20,1), dtype=np.float32) * -1
        info = grid_msg.info
        grid = np.array(grid_msg.data, dtype=np.int8).reshape((info.height, info.width))
        # Normalización: transforma el rango [-1,100] a [0,1] (ajustar según corresponda).
        grid_normalized = (grid + 1) / 101.0
        grid_normalized = np.expand_dims(grid_normalized, axis=-1)
        return grid_normalized
    
    # -------------------------------------------------------------------
    # Función para determinar si el goal está mapeado.
    # -------------------------------------------------------------------
    def is_goal_known(self, goal, grid_msg: OccupancyGrid):
        info = grid_msg.info
        i = int((goal[0] - info.origin.position.x) / info.resolution)
        j = int((goal[1] - info.origin.position.y) / info.resolution)
        if i < 0 or i >= info.width or j < 0 or j >= info.height:
            return False
        idx = j * info.width + i
        return grid_msg.data[idx] != -1
    
    # -------------------------------------------------------------------
    # Seleccionar un subgoal a partir de los safe frontier points.
    # -------------------------------------------------------------------
    def select_subgoal(self, current_pos):
        if not self.safe_frontier_points:
            return None
        subgoal = min(self.safe_frontier_points, key=lambda pt: distance(current_pos, pt))
        self.get_logger().info(f"Subgoal seleccionado: {subgoal}")
        return subgoal
    
    # -------------------------------------------------------------------
    # Controlador Pure Pursuit.
    # -------------------------------------------------------------------
    def pure_pursuit_control(self, path, current_pos, current_yaw):
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
        error_angle = math.atan2(math.sin(error_angle), math.cos(error_angle))
        return self.linear_speed, self.k_pursuit * error_angle
    
    def get_yaw_from_pose(self, pose: Pose):
        q = pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    # -------------------------------------------------------------------
    # Suavizado y publicación del path global.
    # -------------------------------------------------------------------
    def smooth_path(self, path):
        if len(path) < 3:
            return path
        smoothed = [path[0]]
        for i in range(1, len(path) - 1):
            avg_x = (path[i - 1][0] + path[i][0] + path[i + 1][0]) / 3.0
            avg_y = (path[i - 1][1] + path[i][1] + path[i + 1][1]) / 3.0
            # Evitar puntos muy cercanos al anterior
            if distance(smoothed[-1], (avg_x, avg_y)) < 0.2:
                continue
            smoothed.append((avg_x, avg_y))
        if distance(smoothed[-1], path[-1]) > 0.01:
            smoothed.append(path[-1])
        return smoothed
    
    def publish_path(self, path):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        for pt in path:
            pose_st = PoseStamped()
            pose_st.header = path_msg.header
            pose_st.pose.position.x = float(pt[0])
            pose_st.pose.position.y = float(pt[1])
            pose_st.pose.position.z = 0.0
            pose_st.pose.orientation.w = 1.0
            path_msg.poses.append(pose_st)
        self.path_pub.publish(path_msg)
        self.get_logger().info(f"Path publicado con {len(path)} puntos.")
    
    # -------------------------------------------------------------------
    # Loop de Control y Entrenamiento (REINFORCE básico con mejoras)
    # -------------------------------------------------------------------
    def control_loop(self):
        self.get_logger().info("Entrando a control_loop")
        if self.current_pose is None:
            self.get_logger().info("Odom aún no disponible.")
            return
        if self.goal_pose is None:
            self.get_logger().info("Goal aún no disponible.")
            return
        
        current_pos = (self.current_pose.position.x, self.current_pose.position.y)
        goal_pos = (self.goal_pose.position.x, self.goal_pose.position.y)
        self.get_logger().debug(f"Posición actual: {current_pos}, Goal: {goal_pos}")
        
        if self.current_grid is None:
            self.get_logger().warn("Grid map no recibido, esperando...")
            return

        # Procesar el grid map completo.
        local_map = self.extract_complete_grid(self.current_grid)
        self.get_logger().info(f"Local map shape: {local_map.shape}")
        
        # Selección de target:
        # Si el goal no se encuentra mapeado (es decir, no visible en el grid),
        # se selecciona un subgoal a partir de los frontier points.
        if not self.is_goal_known(goal_pos, self.current_grid):
            self.get_logger().warn("El goal no está mapeado. Se usará un subgoal de frontier.")
            subgoal = self.select_subgoal(current_pos)
            target_pos = subgoal if subgoal is not None else goal_pos
        else:
            target_pos = goal_pos
        
        # Inicializar prev_distance si es la primera iteración del episodio.
        if self.prev_distance is None:
            self.prev_distance = distance(current_pos, target_pos)
        
        state_vector = np.array([[current_pos[0], current_pos[1], target_pos[0], target_pos[1]]], dtype=np.float32)
        model_input = [np.expand_dims(local_map, axis=0), state_vector]
        
        # Obtener la acción predicha por la red (los waypoints).
        # action_pred, log_prob, mean, log_std = self.planning_model(model_input)
        # action_pred = action_pred.numpy().flatten()
        action_pred = self.planning_model(model_input)
        action_pred = action_pred.numpy().flatten()
        # Asigna un valor dummy a log_prob ya que el modelo no lo devuelve.
        log_prob = tf.constant(0.0)
        self.get_logger().info(f"Acción predicha: {action_pred}")
        
        predicted_waypoints = []
        for i in range(self.num_waypoints):
            x = action_pred[2 * i]
            y = action_pred[2 * i + 1]
            predicted_waypoints.append((x, y))
        self.get_logger().info(f"Waypoints predichos: {predicted_waypoints}")
        
        # Construir el path global desde la posición actual, vía waypoints hasta el target.
        global_path = [current_pos] + predicted_waypoints + [target_pos]
        global_path = self.smooth_path(global_path)
        self.publish_path(global_path)
        self.get_logger().info(f"Path global: {global_path}")
        
        # --- Mejoras para evitar ir al centro y colisiones con obstáculos ---
        # Calcular el centro del grid map global:
        info = self.current_grid.info
        center_x = info.origin.position.x + (info.width * info.resolution) / 2.0
        center_y = info.origin.position.y + (info.height * info.resolution) / 2.0
        map_center = (center_x, center_y)
        
        penalty = 0.0
        # Penalizar cada waypoint que esté demasiado cerca del centro.
        for pt in predicted_waypoints:
            d_center = distance(pt, map_center)
            if d_center < 1.0:  # umbral en metros (ajustable)
                penalty += 0.5
        # También, penalizar si algún waypoint cae en una celda de obstáculo.
        for pt in predicted_waypoints:
            i = int((pt[0] - info.origin.position.x) / info.resolution)
            j = int((pt[1] - info.origin.position.y) / info.resolution)
            if i < 0 or i >= info.width or j < 0 or j >= info.height:
                continue
            cell_value = self.current_grid.data[j * info.width + i]
            if cell_value >= 100:
                penalty += 1.0  # penalización por obstáculo.
        
        # ---------------------------------------------------------------------
        # Control con Pure Pursuit
        current_yaw = self.get_yaw_from_pose(self.current_pose)
        linear, angular = self.pure_pursuit_control(global_path, current_pos, current_yaw)
        self.get_logger().info(f"Control calculado: linear={linear:.2f} m/s, angular={angular:.2f} rad/s")
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.last_twist = twist
        
        new_distance = distance(current_pos, target_pos)
        reward_progress = self.prev_distance - new_distance  # Progreso hacia el target.
        reward = reward_progress - penalty
        self.get_logger().info(f"Progreso: {reward_progress:.3f}, Penalización: {penalty:.3f}, Recompensa neta: {reward:.3f}")
        self.prev_distance = new_distance
        
        # Guardar la transición en el buffer del episodio.
        self.episode_buffer.append((state_vector, float(log_prob.numpy()), reward))
        
        # Si se alcanza el target (o el subgoal), finaliza el episodio y actualiza la política.
        if new_distance < self.goal_threshold:
            reward += 10.0  # Bonus por alcanzar el objetivo.
            self.episode_buffer[-1] = (state_vector, log_prob.numpy()[0], reward)
            self.get_logger().info("Episodio terminado. Actualizando política...")
            self.update_policy()
            self.episode_count += 1
            self.get_logger().info(f"Episodios completados: {self.episode_count}")
            self.prev_distance = None
            self.episode_buffer = []
    
    # -------------------------------------------------------------------
    # Actualización de la política mediante REINFORCE (simplificado)
    # -------------------------------------------------------------------
    def update_policy(self):
        states, log_probs, rewards = [], [], []
        for (s, lp, r) in self.episode_buffer:
            states.append(s)
            log_probs.append(lp)
            rewards.append(r)
        
        returns = []
        R = 0.0
        for r in reversed(rewards):
            R = r + self.gamma * R
            returns.insert(0, R)
        
        returns = tf.convert_to_tensor(returns, dtype=tf.float32)
        log_probs = tf.convert_to_tensor(log_probs, dtype=tf.float32)
        loss_value = -tf.reduce_mean(log_probs * returns)
        
        with tf.GradientTape() as tape:
            # No es necesario usar tape.watch para variables entrenables.
            current_loss = -tf.reduce_mean(log_probs * returns)
        grads = tape.gradient(current_loss, self.planning_model.trainable_variables)
        self.optimizer.apply_gradients(zip(grads, self.planning_model.trainable_variables))
        self.get_logger().info(f"Política actualizada, pérdida: {current_loss.numpy():.3f}")
    
# -----------------------------------------------------------------------
# Función principal que inicializa el nodo y lo pone a "spin"
# -----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = RLTrainingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
