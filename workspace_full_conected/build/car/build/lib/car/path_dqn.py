# #!/usr/bin/env python3
# import copy
# import math
# import numpy as np
# import tensorflow as tf
# import rclpy
# from rclpy.node import Node

# from nav_msgs.msg import Odometry, OccupancyGrid, Path
# from geometry_msgs.msg import PoseStamped, Point, PoseArray
# from visualization_msgs.msg import Marker

# def distance(p1, p2):
#     return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

# def build_seq_model(grid_shape=(None, None, 1), hidden_size=128):
#     occ_in = tf.keras.Input(shape=grid_shape, name='occupancy')
#     x = tf.keras.layers.Conv2D(16, 3, padding='same', activation='relu')(occ_in)
#     x = tf.keras.layers.MaxPooling2D()(x)
#     x = tf.keras.layers.Conv2D(32, 3, padding='same', activation='relu')(x)
#     x = tf.keras.layers.MaxPooling2D()(x)
#     feat_vec = tf.keras.layers.GlobalAveragePooling2D()(x)

#     state_in = tf.keras.Input(shape=(4,), name='state')
#     concat = tf.keras.layers.Concatenate()([feat_vec, state_in])
#     init_h = tf.keras.layers.Dense(hidden_size, activation='tanh')(concat)
#     init_c = tf.keras.layers.Dense(hidden_size, activation='tanh')(concat)

#     lstm_cell = tf.keras.layers.LSTMCell(hidden_size)
#     wp_out = tf.keras.layers.Dense(2, name='waypoint')
#     stop_out = tf.keras.layers.Dense(1, activation='sigmoid', name='stop_prob')

#     last_wp_in = tf.keras.Input(shape=(2,), name='last_wp')
#     h_in      = tf.keras.Input(shape=(hidden_size,), name='h_in')
#     c_in      = tf.keras.Input(shape=(hidden_size,), name='c_in')

#     _, [h_out, c_out] = lstm_cell(last_wp_in, [h_in, c_in])
#     next_wp = wp_out(h_out)
#     stop_p  = stop_out(h_out)

#     decoder = tf.keras.Model(
#         inputs=[occ_in, state_in, last_wp_in, h_in, c_in],
#         outputs=[next_wp, stop_p, h_out, c_out]
#     )
#     encoder = tf.keras.Model(inputs=occ_in, outputs=feat_vec)
#     return encoder, decoder

# class DebugSeqNode(Node):
#     def __init__(self):
#         super().__init__('debug_seq_node')
#         self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
#         self.create_subscription(PoseArray, '/goal', self.goal_cb, 10)
#         self.create_subscription(OccupancyGrid, '/occupancy_grid', self.grid_cb, 10)
#         self.create_subscription(PoseArray, '/safe_frontier_points_centroid', self.frontier_cb, 10)

#         self.path_pub      = self.create_publisher(Path,   '/global_path_predicted', 10)
#         self.wp_marker_pub = self.create_publisher(Marker, '/path_waypoints_marker', 10)

#         self.current_pose         = None
#         self.goal_pose            = None
#         self.current_grid         = None
#         self.safe_frontier_points = []
#         self.current_path         = []
#         self.current_target       = None
#         self.last_replan_pos      = None
#         self.last_conflict_idx    = None

#         self.reach_threshold     = 2.0
#         self.replan_move_thr     = 0.5
#         self.max_seg_length      = 1.0
#         self.max_steps           = 50
#         self.deviation_threshold = 1.0
#         self.min_frontier_clearance = 1  # cells around frontier must be free

#         self.encoder, self.decoder = build_seq_model((None,None,1), 128)

#         self.create_timer(0.1, self.control_loop)
#         self.get_logger().info("Nodo CNN+LSTM con frontier dinámico iniciado.")

#     def odom_cb(self, msg: Odometry):
#         self.current_pose = msg.pose.pose

#     def goal_cb(self, msg: PoseArray):
#         if msg.poses:
#             self.goal_pose = msg.poses[0]

#     def grid_cb(self, msg: OccupancyGrid):
#         self.current_grid = msg

#     def frontier_cb(self, msg: PoseArray):
#         self.safe_frontier_points = [(p.position.x, p.position.y) for p in msg.poses]

#     def extract_local_grid(self):
#         info = self.current_grid.info
#         H, W = info.height, info.width
#         arr = np.array(self.current_grid.data, dtype=np.int8).reshape((H, W))

#         cp = (self.current_pose.position.x, self.current_pose.position.y)
#         i0 = int((cp[0] - info.origin.position.x) / info.resolution)
#         j0 = int((cp[1] - info.origin.position.y) / info.resolution)

#         ci, cj = W//2, H//2
#         roll_x = ci - i0
#         roll_y = cj - j0
#         local_arr = np.roll(np.roll(arr, roll_y, axis=0), roll_x, axis=1)

#         info_local = copy.copy(info)
#         info_local.origin.position.x = cp[0] - (ci + 0.5) * info.resolution
#         info_local.origin.position.y = cp[1] - (cj + 0.5) * info.resolution

#         norm = (local_arr + 1) / 101.0
#         return np.expand_dims(norm, -1), info_local

#     def is_line_free(self, grid, info, a, b):
#         def w2i(pt):
#             i = int((pt[0] - info.origin.position.x) / info.resolution)
#             j = int((pt[1] - info.origin.position.y) / info.resolution)
#             return i, j

#         x0, y0 = w2i(a); x1, y1 = w2i(b)
#         dx, dy = abs(x1-x0), abs(y1-y0)
#         sx = 1 if x0 < x1 else -1
#         sy = 1 if y0 < y1 else -1
#         err = dx - dy
#         H, W = grid.shape

#         while True:
#             if not (0 <= y0 < H and 0 <= x0 < W): return False
#             if grid[y0, x0] >= 100:            return False
#             if (x0, y0) == (x1, y1):           return True
#             e2 = 2 * err
#             if e2 > -dy:
#                 err -= dy; x0 += sx
#             if e2 < dx:
#                 err += dx; y0 += sy

#     def select_target(self, cp, grid, info):
#         """
#         Escoge el goal si es visible; si no, el frontier más cercano al goal
#         entre los que estén dentro del grid, sean libres y visibles.
#         """
#         gp = (self.goal_pose.position.x, self.goal_pose.position.y)

#         # 1) Si el goal es directamente visible, lo usamos
#         if self.is_line_free(grid, info, cp, gp):
#             return gp

#         # 2) Filtrar frontiers: dentro del grid, conocidos y libres, y visibles
#         H, W = grid.shape
#         valid = []
#         for f in self.safe_frontier_points:
#             # convertir a índices de grid
#             i = int((f[0] - info.origin.position.x) / info.resolution)
#             j = int((f[1] - info.origin.position.y) / info.resolution)
#             # 2.1) Debe estar dentro de los límites
#             if not (0 <= i < W and 0 <= j < H):
#                 continue
#             # 2.2) Debe ser una celda conocida y libre (-1 = desconocido, 100 = obstáculo)
#             val = self.current_grid.data[j * info.width + i]
#             if val == -1 or val >= 100:
#                 continue
#             # 2.3) Debe verse desde el robot
#             if not self.is_line_free(grid, info, cp, f):
#                 continue
#             valid.append(f)

#         if not valid:
#             self.get_logger().warn("[select_target] No hay frontiers dentro del grid, libres y visibles.")
#             return None

#         # 3) De los válidos, el que minimice distancia a goal
#         chosen = min(valid, key=lambda f: distance(f, gp))
#         self.get_logger().info(f"[select_target] Frontier elegido: {chosen}")
#         return chosen


#     def densify(self, path):
#         out = [path[0]]
#         for a, b in zip(path, path[1:]):
#             d = distance(a, b)
#             if d > self.max_seg_length:
#                 steps = int(math.ceil(d / self.max_seg_length))
#                 for i in range(1, steps):
#                     t = i / steps
#                     out.append((a[0]*(1-t)+b[0]*t, a[1]*(1-t)+b[1]*t))
#             out.append(b)
#         return out

#     def generate_path(self, cp, tp):
#         grid, info = self.extract_local_grid()
#         grid_b = grid[None, ...]
#         feat = self.encoder.predict(grid_b)
#         dx, dy = tp[0] - cp[0], tp[1] - cp[1]
#         state = np.array([[0.0, 0.0, dx, dy]], dtype=np.float32)

#         concat = np.concatenate([feat, state], axis=-1)
#         h = tf.keras.layers.Dense(self.decoder.input_shape[3][1], activation='tanh')(concat)
#         c = tf.keras.layers.Dense(self.decoder.input_shape[3][1], activation='tanh')(concat)

#         wp = np.array([[0.0, 0.0]], dtype=np.float32)
#         path = [cp]
#         for _ in range(self.max_steps):
#             next_wp, stop_p, h, c = self.decoder.predict([grid_b, state, wp, h, c])
#             xr, yr = float(next_wp[0,0]), float(next_wp[0,1])
#             xa, ya = cp[0] + xr, cp[1] + yr
#             path.append((xa, ya))
#             if stop_p[0,0] > 0.5 or distance((xa, ya), tp) < self.reach_threshold:
#                 break
#             wp = np.array([[xr, yr]], dtype=np.float32)
#         path.append(tp)
#         return self.densify(path)

#     def publish_waypoints_marker(self, pts):
#         m = Marker()
#         m.header.frame_id = "map"
#         m.header.stamp = self.get_clock().now().to_msg()
#         m.ns = "waypoints"
#         m.id = 2
#         m.type = Marker.POINTS
#         m.action = Marker.ADD
#         m.scale.x = 0.15; m.scale.y = 0.15
#         m.color.r = 1.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 1.0
#         for x, y in pts[1:-1]:
#             m.points.append(Point(x=x, y=y, z=0.0))
#         self.wp_marker_pub.publish(m)




#     def find_conflict(self, path, grid, info):
#         # Recorre cada segmento
#         for idx in range(len(path)-1):
#             a, b = path[idx], path[idx+1]
#             # 4.1) Si la línea atraviesa un obstáculo
#             if not self.is_line_free(grid, info, a, b):
#                 return idx
#             # 4.2) O si el punto b cae dentro de obstáculo
#             #     (por si generó un waypoint dentro de un obstáculo)
#             i = int((b[0] - info.origin.position.x) / info.resolution)
#             j = int((b[1] - info.origin.position.y) / info.resolution)
#             H, W = grid.shape
#             if 0 <= i < W and 0 <= j < H and grid[j, i] >= 100:
#                 return idx
#         return None

#     def control_loop(self):
#         if self.current_pose is None or self.current_grid is None or self.goal_pose is None:
#             return

#         # Posición actual y goal global
#         cp = (self.current_pose.position.x, self.current_pose.position.y)
#         gp = (self.goal_pose.position.x,   self.goal_pose.position.y)

#         # Extrae grid local recenterizado
#         grid_local, info_local = self.extract_local_grid()

#         # 1) CHEQUEO CONTINUO de visibilidad del goal
#         goal_visible = self.is_line_free(grid_local[:,:,0], info_local, cp, gp)
#         if goal_visible:
#             desired_target = gp
#         else:
#             # si ya no vemos el goal, escogemos frontier de nuevo
#             desired_target = self.select_target(cp, grid_local[:,:,0], info_local)
#             if desired_target is None:
#                 self.get_logger().warn("[control] No veo goal ni frontier válido → abortando loop")
#                 return

#         # Si el “target” ha cambiado (p. ej. porque perdió visibilidad del goal),
#         # necesitamos reiniciar el path
#         if desired_target != self.current_target:
#             self.current_target      = desired_target
#             self.current_path        = []
#             self.last_replan_pos     = None
#             self.last_conflict_idx   = None
#             self.get_logger().info(f"[control] Target actualizado → {self.current_target}")

#         # 2) Eliminar waypoints alcanzados
#         while len(self.current_path) > 1 and distance(cp, self.current_path[1]) < self.reach_threshold:
#             wp = self.current_path.pop(1)
#             self.get_logger().info(f"[control] Eliminado waypoint alcanzado: {wp}")

#         # Asegura primer punto = posición actual
#         if self.current_path:
#             self.current_path[0] = cp

#         # 3) Replan completo si el robot se desvía mucho
#         if self.current_path:
#             dmin = min(distance(cp, pt) for pt in self.current_path)
#         else:
#             dmin = float('inf')
#         if dmin > self.deviation_threshold:
#             self.current_path      = self.generate_path(cp, self.current_target)
#             self.last_replan_pos   = cp
#             self.last_conflict_idx = None
#             self.get_logger().info(f"[control] Desviación ({dmin:.2f} m) > umbral → replan completo")


#         # Genera path inicial si aún no hay
#         if not self.current_path:
#             self.current_path      = self.generate_path(cp, self.current_target)
#             self.last_replan_pos   = cp
#             self.last_conflict_idx = None
#             self.get_logger().info(f"[control] Plan inicial completo, len={len(self.current_path)}")

#         # Ahora repite mientras haya conflictos
#         conflict = self.find_conflict(self.current_path, grid_local[:,:,0], info_local)
#         while conflict is not None:
#                 # Punto seguro: desde aquí no hay conflicto
#                 safe = self.current_path[:conflict+1]
#                 self.get_logger().info(f"[control] Conflicto en tramo {conflict}->{conflict+1}, replan parcial")
#                 # Replan del tramo conflictivo: desde el último punto seguro al target
#                 start_pt = safe[-1]
#                 end_pt   = self.current_target
#                 subpath  = self.generate_path(start_pt, end_pt)
#                 # Inyecta subpath (omitimos duplicado de start_pt)
#                 self.current_path = safe + subpath[1:]
#                 # Para la próxima iteración volvemos a extraer grid y buscar conflictos
#                 self.last_replan_pos   = cp
#                 self.last_conflict_idx = conflict
#                 self.get_logger().info(f"[control] Path corregido, len={len(self.current_path)}")
#                 grid_local, info_local = self.extract_local_grid()
#                 conflict = self.find_conflict(self.current_path, grid_local[:,:,0], info_local)


#         # 5) Publicar Path completo
#         path_msg = Path()
#         path_msg.header.stamp    = self.get_clock().now().to_msg()
#         path_msg.header.frame_id = "map"
#         for x,y in self.current_path:
#             ps = PoseStamped()
#             ps.header           = path_msg.header
#             ps.pose.position.x  = x
#             ps.pose.position.y  = y
#             ps.pose.position.z  = 0.0
#             ps.pose.orientation.w = 1.0
#             path_msg.poses.append(ps)
#         self.path_pub.publish(path_msg)

#         # 6) Publicar sólo waypoints
#         self.publish_waypoints_marker(self.current_path)


# def main(args=None):
#     rclpy.init(args=args)
#     node = DebugSeqNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


#######################################################################
#######################################################################

# #!/usr/bin/env python3
# """
# Nodo ROS2 de planificación reactiva mediante CNN+LSTM y frontier dinámico.
# Incluye replanificación automática al detectar movimiento manual y poda de waypoints.
# """
# import copy
# import math
# import numpy as np
# import tensorflow as tf
# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry, OccupancyGrid, Path
# from geometry_msgs.msg import PoseStamped, Point, PoseArray
# from visualization_msgs.msg import Marker


# def distance(p1, p2):
#     """Calcula la distancia Euclídea entre dos puntos 2D."""
#     return math.hypot(p2[0] - p1[0], p2[1] - p1[1])


# def build_seq_model(grid_shape=(None, None, 1), hidden_size=128):
#     """
#     Construye tres submodelos:
#       1) init_model: calcula h0, c0 para el LSTM a partir del grid y el estado.
#       2) encoder: extrae feature vector del occupancy grid.
#       3) decoder: LSTMCell compilado con tf.function para generar waypoints.
#     Acepta grid de tamaño variable.
#     """
#     # --- Encoder CNN ---
#     occ_in = tf.keras.Input(shape=grid_shape, name='occupancy')
#     x = tf.keras.layers.Conv2D(16, 3, padding='same', activation='relu')(occ_in)
#     x = tf.keras.layers.MaxPooling2D()(x)
#     x = tf.keras.layers.Conv2D(32, 3, padding='same', activation='relu')(x)
#     x = tf.keras.layers.MaxPooling2D()(x)
#     feat_vec = tf.keras.layers.GlobalAveragePooling2D()(x)

#     # --- init_model para LSTM ---
#     state_in = tf.keras.Input(shape=(4,), name='state')  # [vx, vy, dx, dy]
#     concat = tf.keras.layers.Concatenate()([feat_vec, state_in])
#     init_h = tf.keras.layers.Dense(hidden_size, activation='tanh', name='init_h')(concat)
#     init_c = tf.keras.layers.Dense(hidden_size, activation='tanh', name='init_c')(concat)
#     init_model = tf.keras.Model([occ_in, state_in], [init_h, init_c], name='init_lstm_state')

#     # --- Decoder LSTMCell ---
#     lstm_cell = tf.keras.layers.LSTMCell(hidden_size)
#     last_wp = tf.keras.Input(shape=(2,), name='last_wp')
#     h_in = tf.keras.Input(shape=(hidden_size,), name='h_in')
#     c_in = tf.keras.Input(shape=(hidden_size,), name='c_in')
#     _, [h_out, c_out] = lstm_cell(last_wp, [h_in, c_in])
#     next_wp = tf.keras.layers.Dense(2, name='waypoint')(h_out)
#     stop_p = tf.keras.layers.Dense(1, activation='sigmoid', name='stop_prob')(h_out)

#     decoder = tf.keras.Model(
#         inputs=[last_wp, h_in, c_in],
#         outputs=[next_wp, stop_p, h_out, c_out],
#         name='decoder'
#     )
#     # Compilamos con tf.function para acelerar inferencia
#     decoder.predict = tf.function(
#         lambda lw, h, c: decoder([lw, h, c]),
#         input_signature=[
#             tf.TensorSpec((1, 2), tf.float32),
#             tf.TensorSpec((1, hidden_size), tf.float32),
#             tf.TensorSpec((1, hidden_size), tf.float32),
#         ]
#     )

#     # --- Encoder standalone ---
#     encoder = tf.keras.Model(inputs=occ_in, outputs=feat_vec, name='encoder')
#     return init_model, decoder, encoder


# class DebugSeqNode(Node):
#     """
#     Nodo principal: maneja callbacks ROS, extrae mapas locales, genera y replanea caminos.
#     """
#     def __init__(self):
#         super().__init__('debug_seq_node')

#         # Suscripciones
#         self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
#         self.create_subscription(PoseArray, '/goal', self.goal_cb, 10)
#         self.create_subscription(OccupancyGrid, '/occupancy_grid', self.grid_cb, 10)
#         self.create_subscription(PoseArray, '/safe_frontier_points_centroid', self.frontier_cb, 10)

#         # Publicadores
#         self.path_pub = self.create_publisher(Path, '/global_path_predicted', 10)
#         self.wp_marker_pub = self.create_publisher(Marker, '/path_waypoints_marker', 10)

#         # Estado interno
#         self.current_pose = None
#         self.goal_pose = None
#         self.current_grid = None
#         self.safe_frontier_points = []
#         self.current_path = []
#         self.current_target = None
#         self.last_replan_pos = None

#         # Parámetros de thresholds y replanning
#         self.reach_threshold = 2.0       # umbral para considerar waypoint alcanzado
#         self.replan_move_thr = 0.5       # distancia de movimiento manual que fuerza replan
#         self.max_seg_length = 1.0        # longitud máxima entre waypoints antes de densificar
#         self.max_steps = 50              # pasos máximos en LSTM decoder
#         self.deviation_threshold = 1.0   # desviación del path para replan completo

#         # Construcción de submodelos TensorFlow
#         self.init_model, self.decoder, self.encoder = build_seq_model((None, None, 1), 128)

#         # Timer para el ciclo de control a 10 Hz
#         self.create_timer(0.1, self.control_loop)
#         self.get_logger().info("Nodo CNN+LSTM con frontier dinámico iniciado.")

#     # ---------------- Callbacks ----------------
#     def odom_cb(self, msg: Odometry):
#         """Callback de odometría: guarda la pose actual del robot."""
#         self.current_pose = msg.pose.pose

#     def goal_cb(self, msg: PoseArray):
#         """Callback de goal: selecciona el primer waypoint de la meta."""
#         if msg.poses:
#             self.goal_pose = msg.poses[0]

#     def grid_cb(self, msg: OccupancyGrid):
#         """Callback del mapa de ocupación: almacena el grid completo."""
#         self.current_grid = msg

#     def frontier_cb(self, msg: PoseArray):
#         """Callback de frontiers seguros: guarda sus coordenadas 2D."""
#         self.safe_frontier_points = [(p.position.x, p.position.y) for p in msg.poses]

#     # ------------- Funciones internas -------------
#     def extract_local_grid(self):
#         """
#         Recorta y centra el occupancy grid alrededor del robot.
#         Normaliza valores de -1..100 a [0..1] y retorna (grid_3D, info_local).
#         """
#         info = self.current_grid.info
#         H, W = info.height, info.width
#         arr = np.array(self.current_grid.data, dtype=np.int8).reshape((H, W))

#         cp = (self.current_pose.position.x, self.current_pose.position.y)
#         i0 = int((cp[0] - info.origin.position.x) / info.resolution)
#         j0 = int((cp[1] - info.origin.position.y) / info.resolution)

#         ci, cj = W // 2, H // 2
#         arr_centered = np.roll(np.roll(arr, cj - j0, axis=0), ci - i0, axis=1)

#         info_local = copy.copy(info)
#         info_local.origin.position.x = cp[0] - (ci + 0.5) * info.resolution
#         info_local.origin.position.y = cp[1] - (cj + 0.5) * info.resolution

#         norm = (arr_centered + 1) / 101.0
#         return np.expand_dims(norm, -1), info_local

#     def is_line_free(self, grid, info, a, b):
#         """
#         Chequea con algoritmo de Bresenham si la línea entre a y b atraviesa obstáculos.
#         grid: array 2D con valores >=100 indicando ocupación.
#         a, b: tuplas (x,y) en coordenadas del mapa.
#         """
#         def w2i(pt):
#             x_idx = int((pt[0] - info.origin.position.x) / info.resolution)
#             y_idx = int((pt[1] - info.origin.position.y) / info.resolution)
#             return x_idx, y_idx

#         x0, y0 = w2i(a); x1, y1 = w2i(b)
#         dx, dy = abs(x1 - x0), abs(y1 - y0)
#         sx = 1 if x0 < x1 else -1
#         sy = 1 if y0 < y1 else -1
#         err = dx - dy
#         H, W = grid.shape

#         while True:
#             if not (0 <= y0 < H and 0 <= x0 < W):
#                 return False
#             if grid[y0, x0] >= 100:
#                 return False
#             if (x0, y0) == (x1, y1):
#                 return True
#             e2 = 2 * err
#             if e2 > -dy:
#                 err -= dy; x0 += sx
#             if e2 < dx:
#                 err += dx; y0 += sy

#     def select_target(self, cp, grid, info):
#         """
#         Selecciona el objetivo:
#           1) Si el goal global es visible → retorna goal.
#           2) Sino → frontier libre y visible más cercano al goal.
#         """
#         gp = (self.goal_pose.position.x, self.goal_pose.position.y)

#         # 1) Goal visible?
#         if self.is_line_free(grid, info, cp, gp):
#             return gp

#         # 2) Filtrar frontiers
#         H, W = grid.shape
#         valid = []
#         for f in self.safe_frontier_points:
#             i = int((f[0] - info.origin.position.x) / info.resolution)
#             j = int((f[1] - info.origin.position.y) / info.resolution)
#             if not (0 <= i < W and 0 <= j < H):
#                 continue
#             val = self.current_grid.data[j * info.width + i]
#             if val == -1 or val >= 100:
#                 continue
#             if not self.is_line_free(grid, info, cp, f):
#                 continue
#             valid.append(f)

#         if not valid:
#             self.get_logger().warn("[select_target] No hay frontiers válidos.")
#             return None

#         # Elegir frontier más cercano al goal
#         chosen = min(valid, key=lambda f: distance(f, gp))
#         self.get_logger().info(f"[select_target] Frontier elegido: {chosen}")
#         return chosen

#     def densify(self, path):
#         """
#         Interpola entre waypoints para que ningún segmento supere max_seg_length.
#         """
#         out = [path[0]]
#         for a, b in zip(path, path[1:]):
#             d = distance(a, b)
#             if d > self.max_seg_length:
#                 steps = int(math.ceil(d / self.max_seg_length))
#                 for i in range(1, steps):
#                     t = i / steps
#                     out.append((a[0] * (1 - t) + b[0] * t,
#                                 a[1] * (1 - t) + b[1] * t))
#             out.append(b)
#         return out

#     def prune_path(self, path, grid, info):
#         """
#         Elimina waypoints intermedios si se puede ir en línea recta sin chocar.
#         """
#         pruned = [path[0]]
#         for i in range(1, len(path) - 1):
#             if not self.is_line_free(grid, info, pruned[-1], path[i + 1]):
#                 pruned.append(path[i])
#         pruned.append(path[-1])
#         return pruned

#     def generate_path(self, cp, tp):
#         """
#         Genera un path desde cp hasta tp:
#           1) Extrae grid local y features.
#           2) Inicializa estado LSTM (h0,c0).
#           3) Decodifica waypoints con LSTMCell.
#           4) Densifica y poda.
#         """
#         grid_3d, info = self.extract_local_grid()
#         grid = grid_3d[:, :, 0]
#         grid_b = grid_3d[np.newaxis, ...].astype(np.float32)
#         feat = self.encoder(grid_b)
#         dx, dy = tp[0] - cp[0], tp[1] - cp[1]
#         state = np.array([[0., 0., dx, dy]], dtype=np.float32)
#         h0, c0 = self.init_model([grid_b, state])

#         wp = np.array([[0., 0.]], dtype=np.float32)
#         path = [cp]
#         for _ in range(self.max_steps):
#             next_wp, stop_p, h0, c0 = self.decoder.predict(wp, h0, c0)
#             xr, yr = float(next_wp[0, 0]), float(next_wp[0, 1])
#             pt = (cp[0] + xr, cp[1] + yr)
#             if not self.is_line_free(grid, info, path[-1], pt):
#                 break
#             path.append(pt)
#             if stop_p[0, 0] > 0.5 or distance(pt, tp) < self.reach_threshold:
#                 break
#             wp = np.array([[xr, yr]], dtype=np.float32)

#         path.append(tp)
#         return self.prune_path(self.densify(path), grid, info)

#     def find_conflict(self, path, grid, info):
#         """
#         Busca el primer segmento del path que colisione con obstáculos.
#         Devuelve el índice del waypoint anterior al conflicto, o None.
#         """
#         for idx in range(len(path) - 1):
#             a, b = path[idx], path[idx + 1]
#             if not self.is_line_free(grid, info, a, b):
#                 return idx
#             i = int((b[0] - info.origin.position.x) / info.resolution)
#             j = int((b[1] - info.origin.position.y) / info.resolution)
#             H, W = grid.shape
#             if 0 <= i < W and 0 <= j < H and grid[j, i] >= 100:
#                 return idx
#         return None

#     def publish_waypoints_marker(self, pts):
#         """Publica marcadores visuales para los waypoints intermedios."""
#         m = Marker()
#         m.header.frame_id = "map"
#         m.header.stamp = self.get_clock().now().to_msg()
#         m.ns = "waypoints"
#         m.id = 2
#         m.type = Marker.POINTS
#         m.action = Marker.ADD
#         m.scale.x, m.scale.y = 0.15, 0.15
#         m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 1.0, 0.0, 1.0
#         for x, y in pts[1:-1]:
#             m.points.append(Point(x=x, y=y, z=0.0))
#         self.wp_marker_pub.publish(m)

#     def control_loop(self):
#         """Bucle de control principal: replanifica, corrige conflictos y publica path."""
#         if not self.current_pose or not self.current_grid or not self.goal_pose:
#             return

#         cp = (self.current_pose.position.x, self.current_pose.position.y)
#         gp = (self.goal_pose.position.x, self.goal_pose.position.y)

#         # Replan si movimiento manual significativo
#         if self.last_replan_pos is None or distance(cp, self.last_replan_pos) > self.replan_move_thr:
#             self.get_logger().info(
#                 f"Movimiento manual detectado ({distance(cp, self.last_replan_pos or cp):.2f} m) → replanificando"
#             )
#             self.current_target = None
#             self.current_path = []

#         # Extraer grid local
#         grid_3d, info_local = self.extract_local_grid()
#         grid = grid_3d[:, :, 0]

#         # 1) Determinar objetivo
#         if self.is_line_free(grid, info_local, cp, gp):
#             desired_target = gp
#         else:
#             desired_target = self.select_target(cp, grid, info_local)
#             if desired_target is None:
#                 self.get_logger().warn("[control] No hay objetivo válido → abortando ciclo")
#                 return

#         # Reset si target cambió
#         if desired_target != self.current_target:
#             self.current_target = desired_target
#             self.current_path = []
#             self.last_replan_pos = None
#             self.get_logger().info(f"[control] Target actualizado → {self.current_target}")

#         # 2) Eliminar waypoints alcanzados
#         while len(self.current_path) > 1 and distance(cp, self.current_path[1]) < self.reach_threshold:
#             reached = self.current_path.pop(1)
#             self.get_logger().info(f"[control] Waypoint alcanzado: {reached}")

#         if self.current_path:
#             self.current_path[0] = cp

#         # 3) Replan completo si hay gran desviación
#         dmin = min(distance(cp, pt) for pt in self.current_path) if self.current_path else float('inf')
#         if dmin > self.deviation_threshold:
#             self.get_logger().info(f"[control] Desviación {dmin:.2f} > {self.deviation_threshold} → replan completo")
#             self.current_path = self.generate_path(cp, self.current_target)
#             self.last_replan_pos = cp

#         # 4) Plan inicial
#         if not self.current_path:
#             self.current_path = self.generate_path(cp, self.current_target)
#             self.last_replan_pos = cp
#             self.get_logger().info(f"[control] Plan inicial generado, len={len(self.current_path)}")

#         # 5) Conflictos parciales
#         conflict = self.find_conflict(self.current_path, grid, info_local)
#         while conflict is not None:
#             safe = self.current_path[:conflict+1]
#             self.get_logger().info(f"[control] Conflicto en tramo {conflict} → replan parcial")
#             subpath = self.generate_path(safe[-1], self.current_target)
#             self.current_path = safe + subpath[1:]
#             self.last_replan_pos = cp
#             grid_3d, info_local = self.extract_local_grid()
#             grid = grid_3d[:, :, 0]
#             conflict = self.find_conflict(self.current_path, grid, info_local)
#             self.get_logger().info(f"[control] Subpath inyectado, len={len(self.current_path)}")

#         # 6) Publicar Path y waypoints
#         path_msg = Path()
#         path_msg.header.stamp = self.get_clock().now().to_msg()
#         path_msg.header.frame_id = "map"
#         for x, y in self.current_path:
#             ps = PoseStamped()
#             ps.header = path_msg.header
#             ps.pose.position.x = x
#             ps.pose.position.y = y
#             ps.pose.position.z = 0.0
#             ps.pose.orientation.w = 1.0
#             path_msg.poses.append(ps)
#         self.path_pub.publish(path_msg)
#         self.publish_waypoints_marker(self.current_path)


# def main(args=None):
#     rclpy.init(args=args)
#     node = DebugSeqNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()



# ############
# #por ahora este
# ########

# # #!/usr/bin/env python3
# # """
# # Nodo ROS2 de planificación reactiva mediante CNN+LSTM y frontier dinámico.
# # Incluye:
# #   - Replanificación automática al detectar movimiento manual.
# #   - Poda de waypoints redundantes.
# #   - Filtrado de frontiers con clearance mínimo respecto a obstáculos.
# # """
# # import copy
# # import math
# # import numpy as np
# # import tensorflow as tf
# # import rclpy
# # from rclpy.node import Node
# # from nav_msgs.msg import Odometry, OccupancyGrid, Path
# # from geometry_msgs.msg import PoseStamped, Point, PoseArray
# # from visualization_msgs.msg import Marker


# # def distance(p1, p2):
# #     """Calcula la distancia Euclídea entre dos puntos 2D."""
# #     return math.hypot(p2[0] - p1[0], p2[1] - p1[1])


# # def build_seq_model(grid_shape=(None, None, 1), hidden_size=128):
# #     """
# #     Construye tres submodelos:
# #       1) init_model: calcula h0, c0 para el LSTM a partir del grid y el estado.
# #       2) encoder: extrae feature vector del occupancy grid.
# #       3) decoder: LSTMCell compilado con tf.function para generar waypoints.
# #     Acepta grids de tamaño variable.
# #     """
# #     # --- Encoder CNN ---
# #     occ_in = tf.keras.Input(shape=grid_shape, name='occupancy')
# #     x = tf.keras.layers.Conv2D(16, 3, padding='same', activation='relu')(occ_in)
# #     x = tf.keras.layers.MaxPooling2D()(x)
# #     x = tf.keras.layers.Conv2D(32, 3, padding='same', activation='relu')(x)
# #     x = tf.keras.layers.MaxPooling2D()(x)
# #     feat_vec = tf.keras.layers.GlobalAveragePooling2D()(x)

# #     # --- init_model para LSTM ---
# #     state_in = tf.keras.Input(shape=(4,), name='state')  # [vx, vy, dx, dy]
# #     concat = tf.keras.layers.Concatenate()([feat_vec, state_in])
# #     init_h = tf.keras.layers.Dense(hidden_size, activation='tanh', name='init_h')(concat)
# #     init_c = tf.keras.layers.Dense(hidden_size, activation='tanh', name='init_c')(concat)
# #     init_model = tf.keras.Model([occ_in, state_in], [init_h, init_c], name='init_lstm_state')

# #     # --- Decoder LSTMCell ---
# #     lstm_cell = tf.keras.layers.LSTMCell(hidden_size)
# #     last_wp = tf.keras.Input(shape=(2,), name='last_wp')
# #     h_in = tf.keras.Input(shape=(hidden_size,), name='h_in')
# #     c_in = tf.keras.Input(shape=(hidden_size,), name='c_in')
# #     _, [h_out, c_out] = lstm_cell(last_wp, [h_in, c_in])
# #     next_wp = tf.keras.layers.Dense(2, name='waypoint')(h_out)
# #     stop_p = tf.keras.layers.Dense(1, activation='sigmoid', name='stop_prob')(h_out)

# #     decoder = tf.keras.Model(
# #         inputs=[last_wp, h_in, c_in],
# #         outputs=[next_wp, stop_p, h_out, c_out],
# #         name='decoder'
# #     )
# #     # Compilamos con tf.function para acelerar inferencia
# #     decoder.predict = tf.function(
# #         lambda lw, h, c: decoder([lw, h, c]),
# #         input_signature=[
# #             tf.TensorSpec((1, 2), tf.float32),
# #             tf.TensorSpec((1, hidden_size), tf.float32),
# #             tf.TensorSpec((1, hidden_size), tf.float32),
# #         ]
# #     )

# #     # --- Encoder standalone ---
# #     encoder = tf.keras.Model(inputs=occ_in, outputs=feat_vec, name='encoder')

# #     return init_model, decoder, encoder


# # class DebugSeqNode(Node):
# #     """Nodo principal: callbacks ROS, extracción de mapas locales, generación y replanificación de caminos."""
# #     def __init__(self):
# #         super().__init__('debug_seq_node')

# #         # Suscripciones
# #         self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
# #         self.create_subscription(PoseArray, '/goal', self.goal_cb, 10)
# #         self.create_subscription(OccupancyGrid, '/occupancy_grid', self.grid_cb, 10)
# #         self.create_subscription(PoseArray, '/safe_frontier_points_centroid', self.frontier_cb, 10)

# #         # Publicadores
# #         self.path_pub = self.create_publisher(Path, '/global_path_predicted', 10)
# #         self.wp_marker_pub = self.create_publisher(Marker, '/path_waypoints_marker', 10)

# #         # Estado interno
# #         self.current_pose = None
# #         self.goal_pose = None
# #         self.current_grid = None
# #         self.safe_frontier_points = []
# #         self.current_path = []
# #         self.current_target = None
# #         self.last_replan_pos = None

# #         # Parámetros de thresholds y replanning
# #         self.reach_threshold = 2.0           # m para considerar waypoint alcanzado
# #         self.replan_move_thr = 0.5           # m de movimiento manual que forza replan
# #         self.max_seg_length = 1.0            # m longitud máxima antes de interpolar
# #         self.max_steps = 50                  # pasos máximos en el decoder LSTM
# #         self.deviation_threshold = 1.0       # m de desviación que fuerza replan completo
# #         self.min_frontier_clearance = 1      # celdas de clearance alrededor del frontier

# #         # Construcción de submodelos TF (grid de tamaño variable)
# #         self.init_model, self.decoder, self.encoder = build_seq_model((None, None, 1), 128)

# #         # Timer 10 Hz para el control_loop
# #         self.create_timer(0.1, self.control_loop)
# #         self.get_logger().info("Nodo CNN+LSTM con frontier dinámico iniciado.")

# #     # -------------- Callbacks ROS --------------
# #     def odom_cb(self, msg: Odometry):
# #         """Guarda la pose actual del robot."""
# #         self.current_pose = msg.pose.pose

# #     def goal_cb(self, msg: PoseArray):
# #         """Actualiza la meta global (primer pose en el array)."""
# #         if msg.poses:
# #             self.goal_pose = msg.poses[0]

# #     def grid_cb(self, msg: OccupancyGrid):
# #         """Almacena la última OccupancyGrid recibida."""
# #         self.current_grid = msg

# #     def frontier_cb(self, msg: PoseArray):
# #         """Guarda los centroides de frontiers seguros."""
# #         self.safe_frontier_points = [(p.position.x, p.position.y) for p in msg.poses]

# #     # -------- Herramientas internas --------
# #     def extract_local_grid(self):
# #         """
# #         Recorta y centra el occupancy grid en el robot;
# #         normaliza de -1..100 a [0..1], devuelve grid_3D y nueva info.
# #         """
# #         info = self.current_grid.info
# #         H, W = info.height, info.width
# #         arr = np.array(self.current_grid.data, dtype=np.int8).reshape((H, W))
# #         cp_x = self.current_pose.position.x
# #         cp_y = self.current_pose.position.y

# #         # Índices del robot en el grid original
# #         i0 = int((cp_x - info.origin.position.x) / info.resolution)
# #         j0 = int((cp_y - info.origin.position.y) / info.resolution)

# #         # Centrar la matriz
# #         ci, cj = W//2, H//2
# #         arr_centered = np.roll(np.roll(arr, cj - j0, axis=0), ci - i0, axis=1)

# #         # Nuevo origen
# #         info_local = copy.copy(info)
# #         info_local.origin.position.x = cp_x - (ci + 0.5) * info.resolution
# #         info_local.origin.position.y = cp_y - (cj + 0.5) * info.resolution

# #         # Normalización
# #         norm = (arr_centered + 1) / 101.0
# #         return np.expand_dims(norm, -1), info_local

# #     def is_line_free(self, grid, info, a, b):
# #         """
# #         Bresenham en grid 2D: devuelve False si línea a→b choca con celda >=100.
# #         """
# #         def w2i(x, y):
# #             ix = int((x - info.origin.position.x) / info.resolution)
# #             iy = int((y - info.origin.position.y) / info.resolution)
# #             return ix, iy

# #         x0, y0 = w2i(*a); x1, y1 = w2i(*b)
# #         dx, dy = abs(x1-x0), abs(y1-y0)
# #         sx, sy = (1 if x0<x1 else -1), (1 if y0<y1 else -1)
# #         err = dx - dy
# #         H, W = grid.shape

# #         while True:
# #             if not (0 <= y0 < H and 0 <= x0 < W):
# #                 return False
# #             if grid[y0, x0] >= 100:
# #                 return False
# #             if (x0, y0) == (x1, y1):
# #                 return True
# #             e2 = 2*err
# #             if e2 > -dy:
# #                 err -= dy; x0 += sx
# #             if e2 < dx:
# #                 err += dx; y0 += sy

# #     def select_target(self, cp, grid, info):
# #         """
# #         1) Si el goal global es visible → goal.
# #         2) Sino → frontier válido (libre, visible y con clearance) más cercano al goal.
# #         """
# #         gp = (self.goal_pose.position.x, self.goal_pose.position.y)

# #         # 1) Goal visible?
# #         if self.is_line_free(grid, info, cp, gp):
# #             return gp

# #         # 2) Filtrar frontiers
# #         H, W = grid.shape
# #         valid = []
# #         for f in self.safe_frontier_points:
# #             ix = int((f[0] - info.origin.position.x) / info.resolution)
# #             iy = int((f[1] - info.origin.position.y) / info.resolution)
# #             # Dentro de límites
# #             if not (0 <= ix < W and 0 <= iy < H):
# #                 continue
# #             # Conocido y libre
# #             val = self.current_grid.data[iy * info.width + ix]
# #             if val == -1 or val >= 100:
# #                 continue
# #             # Visible desde el robot
# #             if not self.is_line_free(grid, info, cp, f):
# #                 continue
# #             # Clearance mínimo
# #             ok = True
# #             r = self.min_frontier_clearance
# #             for dx in range(-r, r+1):
# #                 for dy in range(-r, r+1):
# #                     nx, ny = ix+dx, iy+dy
# #                     if 0 <= nx < W and 0 <= ny < H and grid[ny, nx] >= 100:
# #                         ok = False
# #                         break
# #                 if not ok:
# #                     break
# #             if not ok:
# #                 continue
# #             valid.append(f)

# #         if not valid:
# #             self.get_logger().warn("[select_target] No hay frontiers válidos tras clearance.")
# #             return None

# #         # 3) Frontier más cercano al goal
# #         chosen = min(valid, key=lambda f: distance(f, gp))
# #         self.get_logger().info(f"[select_target] Frontier elegido: {chosen}")
# #         return chosen

# #     def densify(self, path):
# #         """Interpola para que cada segmento sea ≤ max_seg_length."""
# #         out = [path[0]]
# #         for a, b in zip(path, path[1:]):
# #             d = distance(a, b)
# #             if d > self.max_seg_length:
# #                 steps = int(math.ceil(d / self.max_seg_length))
# #                 for i in range(1, steps):
# #                     t = i / steps
# #                     out.append((a[0]*(1-t)+b[0]*t, a[1]*(1-t)+b[1]*t))
# #             out.append(b)
# #         return out

# #     def prune_path(self, path, grid, info):
# #         """Quita waypoints intermedios si la conexión directa es libre."""
# #         pruned = [path[0]]
# #         for i in range(1, len(path)-1):
# #             if not self.is_line_free(grid, info, pruned[-1], path[i+1]):
# #                 pruned.append(path[i])
# #         pruned.append(path[-1])
# #         return pruned

# #     def generate_path(self, cp, tp):
# #         """
# #         Genera path cp→tp:
# #           1) extrae grid local y features,
# #           2) inicializa LSTM (h0,c0),
# #           3) decodifica waypoints,
# #           4) densifica + poda.
# #         """
# #         grid3d, info = self.extract_local_grid()
# #         grid2d = grid3d[:, :, 0]
# #         batch = grid3d[np.newaxis, ...].astype(np.float32)
# #         feat = self.encoder(batch)
# #         dx, dy = tp[0]-cp[0], tp[1]-cp[1]
# #         state = np.array([[0.,0.,dx,dy]], dtype=np.float32)
# #         h, c = self.init_model([batch, state])

# #         wp = np.zeros((1,2), dtype=np.float32)
# #         path = [cp]
# #         for _ in range(self.max_steps):
# #             nw, sp, h, c = self.decoder.predict(wp, h, c)
# #             xr, yr = float(nw[0,0]), float(nw[0,1])
# #             pt = (cp[0]+xr, cp[1]+yr)
# #             if not self.is_line_free(grid2d, info, path[-1], pt):
# #                 break
# #             path.append(pt)
# #             if sp[0,0] > 0.5 or distance(pt, tp) < self.reach_threshold:
# #                 break
# #             wp = np.array([[xr,yr]], dtype=np.float32)

# #         path.append(tp)
# #         return self.prune_path(self.densify(path), grid2d, info)

# #     def find_conflict(self, path, grid, info):
# #         """Retorna índice del primer segmento que colisiona o None."""
# #         for idx in range(len(path)-1):
# #             if not self.is_line_free(grid, info, path[idx], path[idx+1]):
# #                 return idx
# #             # chequeo extra en el punto de llegada
# #             i = int((path[idx+1][0]-info.origin.position.x)/info.resolution)
# #             j = int((path[idx+1][1]-info.origin.position.y)/info.resolution)
# #             if 0 <= i < grid.shape[1] and 0 <= j < grid.shape[0] and grid[j,i] >= 100:
# #                 return idx
# #         return None

# #     def publish_waypoints_marker(self, pts):
# #         """Publica en RViz los waypoints intermedios."""
# #         m = Marker()
# #         m.header.frame_id = "map"
# #         m.header.stamp = self.get_clock().now().to_msg()
# #         m.ns, m.id = "waypoints", 2
# #         m.type, m.action = Marker.POINTS, Marker.ADD
# #         m.scale.x, m.scale.y = 0.15, 0.15
# #         m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 1.0, 0.0, 1.0
# #         for x, y in pts[1:-1]:
# #             m.points.append(Point(x=x, y=y, z=0.0))
# #         self.wp_marker_pub.publish(m)

# #     def control_loop(self):
# #         """Ciclo de control: replanifica, corrige conflictos y publica path."""
# #         if self.current_pose is None or self.current_grid is None or self.goal_pose is None:
# #             return

# #         cp = (self.current_pose.position.x, self.current_pose.position.y)
# #         gp = (self.goal_pose.position.x, self.goal_pose.position.y)

# #         # Si hay movimiento manual > threshold, forzar replanning
# #         if self.last_replan_pos is None or distance(cp, self.last_replan_pos) > self.replan_move_thr:
# #             self.get_logger().info(
# #                 f"Movimiento manual ({distance(cp, self.last_replan_pos or cp):.2f} m) → replanificando"
# #             )
# #             self.current_target = None
# #             self.current_path = []

# #         # Extraer grid local
# #         grid3d, info_local = self.extract_local_grid()
# #         grid2d = grid3d[:, :, 0]

# #         # 1) Selección del objetivo
# #         if self.is_line_free(grid2d, info_local, cp, gp):
# #             desired = gp
# #         else:
# #             desired = self.select_target(cp, grid2d, info_local)
# #             if desired is None:
# #                 self.get_logger().warn("[control] Sin objetivo válido, abortando")
# #                 return

# #         # Si cambió el target, resetear path
# #         if desired != self.current_target:
# #             self.current_target = desired
# #             self.current_path = []
# #             self.last_replan_pos = None
# #             self.get_logger().info(f"[control] Nuevo target → {self.current_target}")

# #         # 2) Eliminar waypoints alcanzados
# #         while len(self.current_path) > 1 and distance(cp, self.current_path[1]) < self.reach_threshold:
# #             reached = self.current_path.pop(1)
# #             self.get_logger().info(f"[control] Eliminado waypoint alcanzado: {reached}")

# #         if self.current_path:
# #             self.current_path[0] = cp

# #         # 3) Replan completo si la ruta se desvía demasiado
# #         dmin = min(distance(cp, pt) for pt in self.current_path) if self.current_path else float('inf')
# #         if dmin > self.deviation_threshold:
# #             self.get_logger().info(f"[control] Desviación {dmin:.2f}>{self.deviation_threshold} → replan completo")
# #             self.current_path = self.generate_path(cp, self.current_target)
# #             self.last_replan_pos = cp

# #         # 4) Plan inicial si no existe
# #         if not self.current_path:
# #             self.current_path = self.generate_path(cp, self.current_target)
# #             self.last_replan_pos = cp
# #             self.get_logger().info(f"[control] Plan inicial generado (len={len(self.current_path)})")

# #         # 5) Resolver conflictos parciales
# #         conflict = self.find_conflict(self.current_path, grid2d, info_local)
# #         while conflict is not None:
# #             self.get_logger().info(f"[control] Conflicto en segmento {conflict} → replan parcial")
# #             safe = self.current_path[:conflict+1]
# #             sub = self.generate_path(safe[-1], self.current_target)
# #             self.current_path = safe + sub[1:]
# #             self.last_replan_pos = cp
# #             grid3d, info_local = self.extract_local_grid()
# #             grid2d = grid3d[:, :, 0]
# #             conflict = self.find_conflict(self.current_path, grid2d, info_local)
# #             self.get_logger().info(f"[control] Subpath inyectado (len={len(self.current_path)})")

# #         # 6) Publicar Path y waypoints
# #         path_msg = Path()
# #         path_msg.header.frame_id = "map"
# #         path_msg.header.stamp = self.get_clock().now().to_msg()
# #         for x, y in self.current_path:
# #             ps = PoseStamped()
# #             ps.header = path_msg.header
# #             ps.pose.position.x = x
# #             ps.pose.position.y = y
# #             ps.pose.position.z = 0.0
# #             ps.pose.orientation.w = 1.0
# #             path_msg.poses.append(ps)
# #         self.path_pub.publish(path_msg)
# #         self.publish_waypoints_marker(self.current_path)


# # def main(args=None):
# #     rclpy.init(args=args)
# #     node = DebugSeqNode()
# #     try:
# #         rclpy.spin(node)
# #     except KeyboardInterrupt:
# #         pass
# #     node.destroy_node()
# #     rclpy.shutdown()


# # if __name__ == '__main__':
# #     main()


# #####
# #bien##
# ####
# #!/usr/bin/env python3
# import copy
# import math
# import numpy as np
# import tensorflow as tf
# import rclpy
# from scipy.ndimage import binary_dilation, generate_binary_structure
# from rclpy.node import Node

# from nav_msgs.msg import Odometry, OccupancyGrid, Path
# from geometry_msgs.msg import PoseStamped, Point, PoseArray
# from visualization_msgs.msg import Marker

# def distance(p1, p2):
#     return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

# def build_seq_model(grid_shape=(None, None, 2), hidden_size=128):
#     occ_in = tf.keras.Input(shape=grid_shape, name='occupancy')
#     x = tf.keras.layers.Conv2D(16, 3, padding='same', activation='relu')(occ_in)
#     x = tf.keras.layers.MaxPooling2D()(x)
#     x = tf.keras.layers.Conv2D(32, 3, padding='same', activation='relu')(x)
#     x = tf.keras.layers.MaxPooling2D()(x)
#     feat_vec = tf.keras.layers.GlobalAveragePooling2D()(x)

#     state_in = tf.keras.Input(shape=(4,), name='state')
#     concat = tf.keras.layers.Concatenate()([feat_vec, state_in])
#     init_h = tf.keras.layers.Dense(hidden_size, activation='tanh')(concat)
#     init_c = tf.keras.layers.Dense(hidden_size, activation='tanh')(concat)

#     lstm_cell = tf.keras.layers.LSTMCell(hidden_size)
#     wp_out = tf.keras.layers.Dense(2, name='waypoint')
#     stop_out = tf.keras.layers.Dense(1, activation='sigmoid', name='stop_prob')

#     last_wp_in = tf.keras.Input(shape=(2,), name='last_wp')
#     h_in      = tf.keras.Input(shape=(hidden_size,), name='h_in')
#     c_in      = tf.keras.Input(shape=(hidden_size,), name='c_in')

#     _, [h_out, c_out] = lstm_cell(last_wp_in, [h_in, c_in])
#     next_wp = wp_out(h_out)
#     stop_p  = stop_out(h_out)

#     decoder = tf.keras.Model(
#         inputs=[occ_in, state_in, last_wp_in, h_in, c_in],
#         outputs=[next_wp, stop_p, h_out, c_out]
#     )
#     encoder = tf.keras.Model(inputs=occ_in, outputs=feat_vec)
#     return encoder, decoder

# class DebugSeqNode(Node):
#     def __init__(self):
#         super().__init__('debug_seq_node')
#         self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
#         self.create_subscription(PoseArray, '/goal', self.goal_cb, 10)
#         self.create_subscription(OccupancyGrid, '/occupancy_grid', self.grid_cb, 10)
#         self.create_subscription(PoseArray, '/safe_frontier_points_centroid', self.frontier_cb, 10)

#         self.path_pub      = self.create_publisher(Path,   '/global_path_predicted', 10)
#         self.wp_marker_pub = self.create_publisher(Marker, '/path_waypoints_marker', 10)
#         self.valid_frontier_pub = self.create_publisher(Marker, '/valid_frontier_points', 10)


#         self.current_pose         = None
#         self.goal_pose            = None
#         self.current_grid         = None
#         self.safe_frontier_points = []
#         self.current_path         = []
#         self.current_target       = None
#         self.last_replan_pos      = None
#         self.last_conflict_idx    = None

#         self.goal_vis_count = 0
#         self.goal_vis_required   = 5
#         self.robot_radius = 5.0  # metros
#         self.reach_threshold     = 2.0
#         self.replan_move_thr     = 0.5
#         self.max_seg_length      = 1.0
#         self.max_steps           = 50
#         self.deviation_threshold = 1.0
#         self.min_frontier_clearance = 1  # cells around frontier must be free

#         self.encoder, self.decoder = build_seq_model((None,None,2), 128)

#         self.create_timer(0.1, self.control_loop)
#         self.get_logger().info("Nodo CNN+LSTM con frontier dinámico iniciado.")

#     def odom_cb(self, msg: Odometry):
#         self.current_pose = msg.pose.pose

#     def goal_cb(self, msg: PoseArray):
#         if msg.poses:
#             self.goal_pose = msg.poses[0]

#     def grid_cb(self, msg: OccupancyGrid):
#         self.current_grid = msg

#     def frontier_cb(self, msg: PoseArray):
#         self.safe_frontier_points = [(p.position.x, p.position.y) for p in msg.poses]


#     def is_cell_safe(self, pt, grid, info):
#         # convierte a índices
#         i = int((pt[0] - info.origin.position.x) / info.resolution)
#         j = int((pt[1] - info.origin.position.y) / info.resolution)
#         cells = int(self.robot_radius / info.resolution)
#         H, W = grid.shape
#         for dj in range(-cells, cells+1):
#             for di in range(-cells, cells+1):
#                 x, y = i+di, j+dj
#                 if 0 <= x < W and 0 <= y < H:
#                     if grid[y, x] >= 100:
#                         return False
#         return True



#     def extract_local_grid(self):
#         """
#         Extrae un parche centrado en el robot de la OccupancyGrid,
#         infla los obstáculos según el radio del robot, genera un costmap
#         para celdas de probabilidad media, y devuelve un array normalizado
#         de forma [H, W, 2] junto con la info local actualizada.
#         """
#         info = self.current_grid.info
#         H, W = info.height, info.width

#         # 1) Reconstruye array 2D a partir del vector unidimensional
#         arr = np.array(self.current_grid.data, dtype=np.int8).reshape((H, W))

#         # 2) Inflado de obstáculos
#         #    Calcula cuántas celdas corresponden al radio del robot
#         cells = int(self.robot_radius / info.resolution)
#         #    Máscara binaria de obstáculos
#         occ_mask = (arr >= 100)
#         #    Estructura de vecindad 4-connect (o 8-connect si lo prefieres)
#         struct = generate_binary_structure(2, 1)
#         #    Dilatación iterativa para inflar
#         inflated_mask = binary_dilation(occ_mask, structure=struct, iterations=cells)
#         #    Reconstruye un array donde las celdas infladas sean 100
#         arr_inflated = np.where(inflated_mask, 100, arr)

#         # 3) Costmap para celdas con probabilidad media
#         cost = np.zeros_like(arr_inflated, dtype=np.float32)
#         #    Rango escogido: [60, 100)  
#         mask_prob = (arr_inflated >= 60) & (arr_inflated < 100)
#         cost[mask_prob] = 1.0  # penalización sencilla; puedes escalar según necesidad

#         # 4) Normalización del mapa de ocupación
#         occ_norm = (arr_inflated + 1) / 101.0  # ahora en [0,1]

#         # 5) Concatena canales: [ocupación, coste]
#         #    Resultado: shape (H, W, 2)
#         norm = np.stack([occ_norm, cost], axis=-1)

#         # 6) Recentra la grid alrededor del robot (igual que antes)
#         cp = (self.current_pose.position.x, self.current_pose.position.y)
#         i0 = int((cp[0] - info.origin.position.x) / info.resolution)
#         j0 = int((cp[1] - info.origin.position.y) / info.resolution)
#         ci, cj = W // 2, H // 2
#         roll_x, roll_y = ci - i0, cj - j0
#         norm = np.roll(np.roll(norm, roll_y, axis=0), roll_x, axis=1)

#         # 7) Ajusta la info del origen local
#         info_local = copy.copy(info)
#         info_local.origin.position.x = cp[0] - (ci + 0.5) * info.resolution
#         info_local.origin.position.y = cp[1] - (cj + 0.5) * info.resolution

        



#         return norm, info_local

        

#     def is_line_free(self, grid, info, a, b):
#         def w2i(pt):
#             i = int((pt[0] - info.origin.position.x) / info.resolution)
#             j = int((pt[1] - info.origin.position.y) / info.resolution)
#             return i, j

#         x0, y0 = w2i(a); x1, y1 = w2i(b)
#         dx, dy = abs(x1-x0), abs(y1-y0)
#         sx = 1 if x0 < x1 else -1
#         sy = 1 if y0 < y1 else -1
#         err = dx - dy
#         H, W = grid.shape

#         while True:
#             if not (0 <= y0 < H and 0 <= x0 < W):
#                 return False
#             val = grid[y0, x0]
#             # ahora bloquea también si desconocido
#             if val == -1 or val >= 100:
#                 return False
#             if (x0, y0) == (x1, y1):
#                 return True
#             e2 = 2 * err
#             if e2 > -dy:
#                 err -= dy; x0 += sx
#             if e2 < dx:
#                 err += dx; y0 += sy


#     def select_target(self, cp, grid, info):
#         """
#         Escoge el goal si es visible; si no, el frontier más cercano al goal
#         entre los que estén dentro del grid, sean libres y visibles.
#         """
#         gp = (self.goal_pose.position.x, self.goal_pose.position.y)

#         # 1) Si el goal es directamente visible, lo usamos
#         if self.is_line_free(grid, info, cp, gp):
#             return gp

#         # 2) Filtrar frontiers: dentro del grid, conocidos y libres, y visibles
#         H, W = grid.shape
#         valid = []
#         for f in self.safe_frontier_points:
#             # convertir a índices de grid
#             i = int((f[0] - info.origin.position.x) / info.resolution)
#             j = int((f[1] - info.origin.position.y) / info.resolution)
#             # 2.1) Debe estar dentro de los límites
#             if not (0 <= i < W and 0 <= j < H):
#                 continue
#             # 2.2) Debe ser una celda conocida y libre (-1 = desconocido, 100 = obstáculo)
#             val = self.current_grid.data[j * info.width + i]
#             if  val >= 100:
#                 continue
#             # 2.3) Debe verse desde el robot
#             if not self.is_line_free(grid, info, cp, f):
#                 continue
#             # 2.4) Debe tener clearance mínimo
#             if not self.is_cell_safe(f, grid, info):
#                 continue
#             valid.append(f)
#             for i, f in enumerate(valid):
#                 # además imprime el valor bruto de grid y visibilidad
#                 i_idx = int((f[0] - info.origin.position.x) / info.resolution)
#                 j_idx = int((f[1] - info.origin.position.y) / info.resolution)
#                 val = self.current_grid.data[j_idx * info.width + i_idx]
#                 vis = self.is_line_free(grid, info, cp, f)
#                 self.get_logger().debug(f"[select_target]  válido {i}: pt={f}, idx=({i_idx},{j_idx}), val={val}, vis={vis}")

#             marker = Marker()
#             marker.header.frame_id = "map"
#             marker.header.stamp = self.get_clock().now().to_msg()
#             marker.ns = "valid_frontiers"
#             marker.id = 0
#             marker.type = Marker.POINTS
#             marker.action = Marker.ADD
#             marker.scale.x = 2.1
#             marker.scale.y = 2.1
#             marker.color.r = 0.0
#             marker.color.g = 0.0
#             marker.color.b = 1.0
#             marker.color.a = 1.0
#             for x, y in valid:
#                 marker.points.append(Point(x=x, y=y, z=0.0))
#             self.valid_frontier_pub.publish(marker)
#         if not valid:
#             self.get_logger().warn("[select_target] No hay frontiers dentro del grid, libres y visibles.")
#             return None

#         # 3) De los válidos, el que minimice distancia a goal
#         chosen = min(valid, key=lambda f: distance(f, gp))
#         self.get_logger().info(f"[select_target] Frontier elegido: {chosen}")
#         return chosen


#     def densify(self, path):
#         out = [path[0]]
#         for a, b in zip(path, path[1:]):
#             d = distance(a, b)
#             if d > self.max_seg_length:
#                 steps = int(math.ceil(d / self.max_seg_length))
#                 for i in range(1, steps):
#                     t = i / steps
#                     out.append((a[0]*(1-t)+b[0]*t, a[1]*(1-t)+b[1]*t))
#             out.append(b)
#         return out

#     def generate_path(self, cp, tp):
#         grid, info_local = self.extract_local_grid()
#         grid_b = grid[None, ...]
#         feat = self.encoder.predict(grid_b)
#         dx, dy = tp[0] - cp[0], tp[1] - cp[1]
#         state = np.array([[0.0, 0.0, dx, dy]], dtype=np.float32)

#         concat = np.concatenate([feat, state], axis=-1)
#         h = tf.keras.layers.Dense(self.decoder.input_shape[3][1], activation='tanh')(concat)
#         c = tf.keras.layers.Dense(self.decoder.input_shape[3][1], activation='tanh')(concat)

#         wp = np.array([[0.0, 0.0]], dtype=np.float32)
#         path = [cp]
#         for _ in range(self.max_steps):
#             next_wp, stop_p, h, c = self.decoder.predict([grid_b, state, wp, h, c])
#             xr, yr = float(next_wp[0,0]), float(next_wp[0,1])
#             xa, ya = cp[0] + xr, cp[1] + yr
#             i = int((xa - info_local.origin.position.x) / info_local.resolution)
#             j = int((ya - info_local.origin.position.y) / info_local.resolution)
#             idx = j * info_local.width + i
#             if self.current_grid.data[idx] == -1:
#                 # ¡inexplorado! descartamos este wp y reiniciamos LSTM (opcional)
#                 self.get_logger().warn(f"WP inexplorado en {(xa,ya)} → rechazado")
#                 break   # o continue, o reinicializa h,c, según tu estrategia

#             path.append((xa, ya))
#             if stop_p[0,0] > 0.5 or distance((xa, ya), tp) < self.reach_threshold:
#                 break
#             wp = np.array([[xr, yr]], dtype=np.float32)
#         path.append(tp)
#         return self.densify(path)

#     def publish_waypoints_marker(self, pts):
#         m = Marker()
#         m.header.frame_id = "map"
#         m.header.stamp = self.get_clock().now().to_msg()
#         m.ns = "waypoints"
#         m.id = 2
#         m.type = Marker.POINTS
#         m.action = Marker.ADD
#         m.scale.x = 0.15; m.scale.y = 0.15
#         m.color.r = 1.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 1.0
#         for x, y in pts[1:-1]:
#             m.points.append(Point(x=x, y=y, z=0.0))
#         self.wp_marker_pub.publish(m)




#     def find_conflict(self, path, grid, info):
#         # Recorre cada segmento
#         for idx in range(len(path)-1):
#             a, b = path[idx], path[idx+1]
#             # 4.1) Si la línea atraviesa un obstáculo
#             if not self.is_line_free(grid, info, a, b):
#                 return idx
#             # 4.2) O si el punto b cae dentro de obstáculo
#             #     (por si generó un waypoint dentro de un obstáculo)
#             i = int((b[0] - info.origin.position.x) / info.resolution)
#             j = int((b[1] - info.origin.position.y) / info.resolution)
#             H, W = grid.shape
#             if 0 <= i < W and 0 <= j < H and grid[j, i] >= 100:
#                 return idx
#         return None

#     def compute_reward(self, episode):
#         reward = 0.0
#         # 1. Llegada al goal:
#         if episode.reached_goal:
#             reward += 100.0
#         else:
#             reward -= 50.0
#         # 2. Penalización por colisión:
#         reward -= 100.0 * episode.num_collisions
#         # 3. Calidad de waypoints:
#         for wp in episode.waypoints:
#             d_obs = self.min_distance_to_obstacle(wp)
#             reward += 0.1 * d_obs  # cuanto más lejos, mejor
#         # 4. Uso de frontiers:
#         reward += 5.0 * episode.num_frontiers_used
#         # 5. Paso por zonas desconocidas (penalizar):
#         reward -= 1.0 * episode.num_waypoints_in_unknown
#         return reward




#     def control_loop(self):
#         if self.current_pose is None or self.current_grid is None or self.goal_pose is None:
#             return

#         # Posición actual y goal global
#         cp = (self.current_pose.position.x, self.current_pose.position.y)
#         gp = (self.goal_pose.position.x,   self.goal_pose.position.y)

#         # Extrae grid local recenterizado
#         grid_local, info_local = self.extract_local_grid()

#         # 1) CHEQUEO CONTINUO de visibilidad del goal
#         # goal_visible = self.is_line_free(grid_local[:,:,0], info_local, cp, gp)
#         if self.is_line_free(grid_local[:,:,0], info_local, cp, gp):
#             self.goal_vis_count += 1
#         else:
#             self.goal_vis_count  = 0

#         # solo consideramos “verdaderamente visible” si se mantiene varios ciclos
#         goal_visible = (self.goal_vis_count >= self.goal_vis_required)

        
        
        
#         if goal_visible:
#             desired_target = gp
#         else:
#             # si ya no vemos el goal, escogemos frontier de nuevo
#             desired_target = self.select_target(cp, grid_local[:,:,0], info_local)
#             if desired_target is None:
#                 self.get_logger().warn("[control] No veo goal ni frontier válido → abortando loop")
#                 return

#         # Si el “target” ha cambiado (p. ej. porque perdió visibilidad del goal),
#         # necesitamos reiniciar el path
#         if desired_target != self.current_target:
#             self.current_target      = desired_target
#             self.current_path        = []
#             self.last_replan_pos     = None
#             self.last_conflict_idx   = None
#             self.get_logger().info(f"[control] Target actualizado → {self.current_target}")

#         # 2) **Detección continua de conflictos** en el path actual
#         if self.current_path:
#             conflict = self.find_conflict(self.current_path, grid_local[:,:,0], info_local)
#             if conflict is not None:
#                 self.get_logger().info(f"[control] Obstáculo detectado en tramo {conflict}→ replan inmediato")
#                 # Opcional: replante completo o parcial
#                 # Para replan completo:
#                 self.current_path = self.generate_path(cp, self.current_target)
#                 # Reinicia contadores
#                 self.last_replan_pos   = cp
#                 self.last_conflict_idx = None

#         # 3) Si no hay path o lo acabamos de resetear, genera uno
#         if not self.current_path:
#             self.current_path      = self.generate_path(cp, self.current_target)
#             self.last_replan_pos   = cp
#             self.last_conflict_idx = None
#             self.get_logger().info(f"[control] Plan inicial, len={len(self.current_path)}")




#         # 4) Eliminar waypoints alcanzados
#         while len(self.current_path) > 1 and distance(cp, self.current_path[1]) < self.reach_threshold:
#             wp = self.current_path.pop(1)
#             self.get_logger().info(f"[control] Eliminado waypoint alcanzado: {wp}")

#         # Asegura primer punto = posición actual
#         if self.current_path:
#             self.current_path[0] = cp

#         # 5) Replan completo si el robot se desvía mucho
#         if self.current_path:
#             dmin = min(distance(cp, pt) for pt in self.current_path)
#         else:
#             dmin = float('inf')
#         if dmin > self.deviation_threshold:
#             self.current_path      = self.generate_path(cp, self.current_target)
#             self.last_replan_pos   = cp
#             self.last_conflict_idx = None
#             self.get_logger().info(f"[control] Desviación ({dmin:.2f} m) > umbral → replan completo")


#         # Genera path inicial si aún no hay
#         if not self.current_path:
#             self.current_path      = self.generate_path(cp, self.current_target)
#             self.last_replan_pos   = cp
#             self.last_conflict_idx = None
#             self.get_logger().info(f"[control] Plan inicial completo, len={len(self.current_path)}")

#         # Ahora repite mientras haya conflictos
#         conflict = self.find_conflict(self.current_path, grid_local[:,:,0], info_local)
#         while conflict is not None:
#                 # Punto seguro: desde aquí no hay conflicto
#                 safe = self.current_path[:conflict+1]
#                 self.get_logger().info(f"[control] Conflicto en tramo {conflict}->{conflict+1}, replan parcial")
#                 # Replan del tramo conflictivo: desde el último punto seguro al target
#                 start_pt = safe[-1]
#                 end_pt   = self.current_target
#                 subpath  = self.generate_path(start_pt, end_pt)
#                 # Inyecta subpath (omitimos duplicado de start_pt)
#                 self.current_path = safe + subpath[1:]
#                 # Para la próxima iteración volvemos a extraer grid y buscar conflictos
#                 self.last_replan_pos   = cp
#                 self.last_conflict_idx = conflict
#                 self.get_logger().info(f"[control] Path corregido, len={len(self.current_path)}")
#                 grid_local, info_local = self.extract_local_grid()
#                 conflict = self.find_conflict(self.current_path, grid_local[:,:,0], info_local)


#         # 6) Publicar Path completo
#         path_msg = Path()
#         path_msg.header.stamp    = self.get_clock().now().to_msg()
#         path_msg.header.frame_id = "map"
#         for x,y in self.current_path:
#             ps = PoseStamped()
#             ps.header           = path_msg.header
#             ps.pose.position.x  = x
#             ps.pose.position.y  = y
#             ps.pose.position.z  = 0.0
#             ps.pose.orientation.w = 1.0
#             path_msg.poses.append(ps)
#         self.path_pub.publish(path_msg)

#         # ) Publicar sólo waypoints
#         self.publish_waypoints_marker(self.current_path)


# def main(args=None):
#     rclpy.init(args=args)
#     node = DebugSeqNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()

#sin modelo

# #!/usr/bin/env python3
# """
# debug_seq_node_flexible.py
# ──────────────────────────
# • Misma lógica que tu nodo original (selección de target,
#   detección de conflictos, replan parcial…).
# • NUEVO  generate_flexible_path():
#       – genera candidatos en abanico  ±60°  y 3 radios
#       – filtra colisión / clearance
#       – elige el que minimiza  d_target - 0.5·clearance
# """

# import copy, math, numpy as np, rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry, OccupancyGrid, Path
# from geometry_msgs.msg import PoseStamped, Point, PoseArray, Vector3, Twist
# from visualization_msgs.msg import Marker

# # ----------------- utilidades básicas ---------------- #
# def distance(a,b): return math.hypot(b[0]-a[0], b[1]-a[1])

# def bres_line_free(grid, info, a, b):
#     """Bresenham en grid de int8 (valor >=100 = obstáculo)"""
#     def idx(p):
#         return (int((p[0]-info.origin.position.x)/info.resolution),
#                 int((p[1]-info.origin.position.y)/info.resolution))
#     i0,j0 = idx(a); i1,j1 = idx(b)
#     di,dj = abs(i1-i0), abs(j1-j0)
#     si = 1 if i0<i1 else -1
#     sj = 1 if j0<j1 else -1
#     err = di-dj
#     H,W = grid.shape
#     while True:
#         if not (0<=i0<W and 0<=j0<H) or grid[j0,i0] >= 100:
#             return False
#         if (i0,j0) == (i1,j1): return True
#         e2 = 2*err
#         if e2>-dj: err-=dj; i0+=si
#         if e2< di: err+=di; j0+=sj

# # ----------------------------------------------------- #
# class DebugSeqNode(Node):
#     def __init__(self):
#         super().__init__("debug_seq_node_flexible")

#         self.create_subscription(Odometry,      "/odom",             self.cb_odom, 10)
#         self.create_subscription(PoseArray,     "/goal",             self.cb_goal, 10)
#         self.create_subscription(OccupancyGrid, "/occupancy_grid",   self.cb_grid, 10)
#         self.create_subscription(PoseArray,     "/safe_frontier_points_centroid",
#                                   self.cb_frontier, 10)

#         self.path_pub      = self.create_publisher(Path,   "/global_path_predicted", 10)
#         self.way_pub       = self.create_publisher(Marker, "/path_waypoints_marker", 10)

#         # estado
#         self.pose=self.goal=None
#         self.grid_msg=None
#         self.frontiers=[]

#         self.current_target=None
#         self.current_path=[]
#         self.reach_thr=0.4
#         self.max_seg  =0.6
#         self.max_steps=60
#         self.dev_thr  =0.8
#         self.clear_min=0.4     # m
#         self.max_seg_length      = 1.0

#         self.create_timer(0.1, self.control_loop)
#         self.get_logger().info("Nodo flexible iniciado")

#     # ---------- callbacks de ROS ---------- #
#     def cb_odom(self,m): self.pose=m.pose.pose
#     def cb_goal(self,m): self.goal=m.poses[0] if m.poses else None
#     def cb_grid(self,m): self.grid_msg=m
#     def cb_frontier(self,m): self.frontiers=[(p.position.x,p.position.y) for p in m.poses]

#     # ---------- grid helpers ---------- #
#     def extract_local_grid(self):
#         info=self.grid_msg.info
#         H,W=info.height,info.width
#         arr=np.array(self.grid_msg.data,dtype=np.int8).reshape((H,W))
#         cp=(self.pose.position.x,self.pose.position.y)
#         i0=int((cp[0]-info.origin.position.x)/info.resolution)
#         j0=int((cp[1]-info.origin.position.y)/info.resolution)
#         ci,cj=W//2,H//2
#         arr=np.roll(np.roll(arr,cj-j0,axis=0),ci-i0,axis=1)
#         info_local=copy.copy(info)
#         info_local.origin.position.x = cp[0] - (ci+0.5)*info.resolution
#         info_local.origin.position.y = cp[1] - (cj+0.5)*info.resolution
#         return arr, info_local
#     # ---------- ① línea libre: bloquea desconocido  ---------- #
#     def bres_line_free(grid, info, a, b):
#         def idx(p):
#             return (int((p[0]-info.origin.position.x)/info.resolution),
#                     int((p[1]-info.origin.position.y)/info.resolution))
#         i0,j0 = idx(a); i1,j1 = idx(b)
#         di,dj = abs(i1-i0), abs(j1-j0); si = 1 if i0<i1 else -1; sj = 1 if j0<j1 else -1
#         err = di-dj; H,W = grid.shape
#         while True:
#             if not (0<=i0<W and 0<=j0<H) or grid[j0,i0] == -1 or grid[j0,i0] >= 100:
#                 return False
#             if (i0,j0)==(i1,j1): return True
#             e2=2*err
#             if e2>-dj: err-=dj; i0+=si
#             if e2< di: err+=di; j0+=sj

#     # ---------- ② clearance estricta (‑1 cuenta)  ---------- #
#     def clearance(self, grid, info, pt, radius_m):
#         r=int(radius_m/info.resolution)
#         i=int((pt[0]-info.origin.position.x)/info.resolution)
#         j=int((pt[1]-info.origin.position.y)/info.resolution)
#         H,W=grid.shape
#         for dj in range(-r,r+1):
#             for di in range(-r,r+1):
#                 x,y=i+di,j+dj
#                 if 0<=x<W and 0<=y<H and (grid[y,x]==-1 or grid[y,x]>=100):
#                     return False
#         return True
#     # ---------- target (goal / frontier) ---------- #
#     def select_target(self, cp, grid, info):
#         gp=(self.goal.position.x,self.goal.position.y)
#         if bres_line_free(grid,info,cp,gp): return gp
#         cand=[f for f in self.frontiers
#               if bres_line_free(grid,info,cp,f)
#               and self.clearance(grid,info,f,0.25)]
#         if not cand: return None
#         return min(cand,key=lambda f: distance(f,gp))
    

#     def densify(self, path):
#         out = [path[0]]
#         for a, b in zip(path, path[1:]):
#             d = distance(a, b)
#             if d > self.max_seg_length:
#                 steps = int(math.ceil(d / self.max_seg_length))
#                 for i in range(1, steps):
#                     t = i / steps
#                     out.append((a[0]*(1-t)+b[0]*t, a[1]*(1-t)+b[1]*t))
#             out.append(b)
#         return out

#     # ---------- nuevo generador flexible ---------- #
#     def generate_flexible_path(self, start, target, grid, info):
#         path=[start]
#         cp=start
#         step_len=self.max_seg
#         angles=np.linspace(-math.pi/3, math.pi/3, 10)   # ±60°
#         radii=[step_len*0.5, step_len, step_len*1.5]

#         for _ in range(self.max_steps):
#             best=None; best_cost=float("inf")
#             vec_t=(target[0]-cp[0], target[1]-cp[1])
#             ang0=math.atan2(vec_t[1], vec_t[0])

#             for r in radii:
#                 for a_off in angles:
#                     ang=ang0 + a_off
#                     cand=(cp[0]+r*math.cos(ang), cp[1]+r*math.sin(ang))

#                     # ---------- filtro ③  “celda debe ser conocida y libre” ----------
#                     i = int((cand[0] - info.origin.position.x) / info.resolution)
#                     j = int((cand[1] - info.origin.position.y) / info.resolution)
#                     if not (0 <= i < grid.shape[1] and 0 <= j < grid.shape[0]):
#                         continue                             # fuera de mapa
#                     if grid[j, i] == -1 or grid[j, i] >= 100:
#                         continue                             # desconocida u obstáculo
#                     # -----------------------------------------------------------------

#                     if not bres_line_free(grid, info, cp, cand):
#                         continue
#                     if not self.clearance(grid, info, cand, self.clear_min):
#                         continue

#                     cost = distance(cand, target) - 0.5*self.clear_min
#                     if cost < best_cost:
#                         best_cost, best = cost, cand

#             if best is None:
#                 break
#             path.append(best)
#             cp = best
#             if distance(cp, target) < self.reach_thr:
#                 break

#         path.append(target)
#         return self.densify(path)          # si usas la misma densify() de antes


#     # ---------- control loop ---------- #
#     def control_loop(self):
#         if None in (self.pose,self.goal,self.grid_msg): return
#         cp=(self.pose.position.x,self.pose.position.y)
#         grid,info=self.extract_local_grid()

#         tgt=self.select_target(cp,grid,info)
#         if tgt is None: return

#         if not self.current_path or distance(cp,self.current_path[1])>self.dev_thr:
#             self.current_path=self.generate_flexible_path(cp,tgt,grid,info)

#         # quitar wp alcanzados
#         while len(self.current_path)>1 and distance(cp,self.current_path[1])<self.reach_thr:
#             self.current_path.pop(1)

#         # publicar
#         self.publish(self.current_path)

#     # ---------- publish helpers ---------- #
#     def publish(self, pts):
#         hdr=self.get_clock().now().to_msg()
#         path=Path()
#         path.header.frame_id="map"; path.header.stamp=hdr
#         for x,y in pts:
#             ps=PoseStamped()
#             ps.header=path.header
#             ps.pose.position.x=x; ps.pose.position.y=y; ps.pose.orientation.w=1.0
#             path.poses.append(ps)
#         self.path_pub.publish(path)

#         mk=Marker()
#         mk.header=path.header; mk.ns="wps"; mk.id=0
#         mk.type=Marker.POINTS; mk.action=Marker.ADD
#         mk.scale=Vector3(x=0.15,y=0.15,z=0.0)
#         mk.color.r=mk.color.g=1.0; mk.color.a=1.0
#         for x,y in pts[1:]: mk.points.append(Point(x=x,y=y))
#         self.way_pub.publish(mk)


# # ---------------- MAIN ---------------- #
# def main(args=None):
#     rclpy.init(args=args)
#     node=DebugSeqNode()
#     try: rclpy.spin(node)
#     except KeyboardInterrupt: pass
#     node.destroy_node(); rclpy.shutdown()

# if __name__=="__main__":
#     main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
flexible_planner_node.py
────────────────────────
Nodo ROS 2 que conserva el flujo completo de tu planner
pero genera segmentos flexibles con ayuda de una policy
CNN+LSTM.  Sin entrenamiento ya evita obstáculos; con
PPO (función al final) puedes ir afinando.
"""

import math, copy, numpy as np, rclpy, tensorflow as tf
from rclpy.node import Node

# ROS msgs
from nav_msgs.msg  import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseArray, Point, Vector3, Twist
from visualization_msgs.msg import Marker
from std_msgs.msg import Header

# ----------------- Hiper‑parámetros ----------------- #
PATCH      = 128
MAX_STEP   = 0.6      # m, radio más largo del abanico
ANGLES     = np.linspace(-math.pi/3, math.pi/3, 11)   # ±60° en 10 pasos
RADII      = [0.3, 0.6, 0.9]                          # m
CLEAR_MIN  = 0.40     # m, clearance mínima alrededor de un waypoint
GOAL_OK    = 4        # ciclos consecutivos requerido para “goal visible”

# PPO                                                                        #
ROLLOUT, BATCH, EPOCHS = 1024, 256, 4
GAMMA, LAMBDA_GAE      = 0.99, 0.95
CLIP, LR_A, LR_C       = 0.2, 3e-4, 1e-3
STD0, STD_MIN, DECAY   = 0.3, 0.05, 0.995
# --------------------------------------------------------------------------- #

# ------------ funciones de ayuda (grid) ------------- #
def dist(a,b): return math.hypot(b[0]-a[0], b[1]-a[1])

def bres_free(grid, info, a, b):
    """True si la línea A‑B atraviesa SÓLO celdas <100 y !=‑1"""
    res = info.resolution
    i0 = int((a[0]-info.origin.position.x)/res)
    j0 = int((a[1]-info.origin.position.y)/res)
    i1 = int((b[0]-info.origin.position.x)/res)
    j1 = int((b[1]-info.origin.position.y)/res)
    di,dj = abs(i1-i0), abs(j1-j0)
    si = 1 if i0<i1 else -1
    sj = 1 if j0<j1 else -1
    err = di-dj
    H,W = grid.shape
    while True:
        if not (0<=i0<W and 0<=j0<H): return False
        val = grid[j0, i0]
        if val == -1 or val >= 100:   return False
        if (i0,j0)==(i1,j1):          return True
        e2=2*err
        if e2>-dj: err-=dj; i0+=si
        if e2< di: err+=di; j0+=sj

def clearance_ok(grid, info, pt, r_m):
    """Comprueba un anillo cuadrado r_m alrededor (‑1 y ≥100 bloquean)"""
    res = info.resolution
    r   = int(r_m/res)
    i   = int((pt[0]-info.origin.position.x)/res)
    j   = int((pt[1]-info.origin.position.y)/res)
    H,W = grid.shape
    for dj in range(-r, r+1):
        for di in range(-r, r+1):
            x,y = i+di, j+dj
            if 0<=x<W and 0<=y<H and (grid[y,x]==-1 or grid[y,x]>=100):
                return False
    return True

# ------------- pequeña policy CNN+LSTM -------------- #
def build_policy():
    g   = tf.keras.Input(shape=(PATCH,PATCH,1), name="grid")
    st  = tf.keras.Input(shape=(4,),            name="state")
    wp0 = tf.keras.Input(shape=(2,),            name="wp0")

    x=tf.keras.layers.Conv2D(16,3,padding="same",activation="relu")(g)
    x=tf.keras.layers.MaxPooling2D()(x)
    x=tf.keras.layers.Conv2D(32,3,padding="same",activation="relu")(x)
    x=tf.keras.layers.GlobalAveragePooling2D()(x)
    z=tf.keras.layers.Concatenate()([x,st])

    h=tf.keras.layers.Dense(128,activation="tanh")(z)
    c=tf.keras.layers.Dense(128,activation="tanh")(z)
    lstm=tf.keras.layers.LSTMCell(128)
    h,[h,c]=lstm(wp0,[h,c])
    delta=tf.keras.layers.Dense(2,activation="tanh")(h)   # » (‑1,1)
    model=tf.keras.Model([g,st,wp0],delta,name="policy")
    return model

# --------------------- Nodo ------------------------- #
class FlexiblePlanner(Node):
    def __init__(self):
        super().__init__("flexible_planner_node")
        # subs
        self.create_subscription(Odometry,"/odom",self.cb_odom,10)
        self.create_subscription(PoseArray,"/goal",self.cb_goal,10)
        self.create_subscription(OccupancyGrid,"/occupancy_grid",self.cb_grid,10)
        self.create_subscription(PoseArray,"/safe_frontier_points_centroid",
                                 self.cb_frontier,10)
        # pubs
        self.path_pub = self.create_publisher(Path,"/global_path_predicted",10)
        self.wps_pub  = self.create_publisher(Marker,"/path_waypoints_marker",10)
        # estado
        self.pose=self.goal=self.grid_msg=None
        self.frontiers=[]
        self.goal_vis_cnt=0
        self.current_target=None
        self.current_path=[]
        # policy
        self.actor=build_policy()
        # self.actor.load_weights('policy.h5')    # ← carga aquí si lo tienes
        self.log_std=tf.Variable(np.log(STD0*np.ones(2,np.float32)),
                                 trainable=True)
        # timer
        self.create_timer(0.1, self.step)
        self.get_logger().info("Nodo flexible + DL iniciado")

    # ----------- Callbacks ROS ----------- #
    def cb_odom(self,m): self.pose=m.pose.pose
    def cb_goal(self,m): self.goal=m.poses[0] if m.poses else None
    def cb_grid(self,m): self.grid_msg=m
    def cb_frontier(self,m):
        self.frontiers=[(p.position.x,p.position.y) for p in m.poses]

    # ----------- util extract patch ----------- #
    def extract_patch(self):
        info=self.grid_msg.info
        H,W=info.height,info.width
        arr=np.array(self.grid_msg.data,dtype=np.int8).reshape((H,W))
        cp=(self.pose.position.x,self.pose.position.y)
        ci=int((cp[0]-info.origin.position.x)/info.resolution)
        cj=int((cp[1]-info.origin.position.y)/info.resolution)
        i_lo,i_hi=ci-PATCH//2,ci+PATCH//2
        j_lo,j_hi=cj-PATCH//2,cj+PATCH//2
        i0,i1=max(i_lo,0),min(i_hi,W)
        j0,j1=max(j_lo,0),min(j_hi,H)
        patch=arr[j0:j1,i0:i1]
        pad=((j0-j_lo,j_hi-j1),(i0-i_lo,i_hi-i1))
        patch=np.pad(patch,pad,'constant',constant_values=-1)
        norm=((patch+1)/101.0).astype(np.float32)  # [-1,100] →  [0,1]
        return np.expand_dims(norm,-1), arr, info

    # --------------- target robusto --------------- #
    def select_target(self, cp, grid, info):
        gp=(self.goal.position.x,self.goal.position.y)
        if bres_free(grid,info,cp,gp):
            self.goal_vis_cnt += 1
        else:
            self.goal_vis_cnt  = 0

        if self.goal_vis_cnt >= GOAL_OK:
            return gp, "GOAL"

        # elegir frontier
        cand=[f for f in self.frontiers
              if bres_free(grid,info,cp,f) and clearance_ok(grid,info,f,CLEAR_MIN)]
        if not cand:
            return None, "NONE"
        tgt=min(cand,key=lambda f: dist(f,gp))
        return tgt, "FRONTIER"

    # --------------- path flexible --------------- #
    def flexible_segment(self, cp, tgt, grid, info):
        # 1) vector preferido de la policy
        patch,_a,_i = self.extract_patch()
        patch_b = patch[None, ...]                    #        (1,128,128,1)

        state = np.array([0, 0,
                        tgt[0] - cp[0],
                        tgt[1] - cp[1]], np.float32) # (4,)
        state_b = state[None, :]                       # (1,4)

        delta = self.actor([patch_b,
                            state_b,
                            np.zeros((1, 2), np.float32)],
                        training=False)[0].numpy()
        base_ang=math.atan2(delta[1],delta[0])
        if np.linalg.norm(delta)<1e-3:
            base_ang=math.atan2(tgt[1]-cp[1], tgt[0]-cp[0])

        # 2) abanico alrededor de ese ángulo
        best=None; best_cost=float("inf")
        for r in RADII:
            for off in ANGLES:
                ang=base_ang+off
                cand=(cp[0]+r*math.cos(ang), cp[1]+r*math.sin(ang))
                # celda conocida y libre
                i=int((cand[0]-info.origin.position.x)/info.resolution)
                j=int((cand[1]-info.origin.position.y)/info.resolution)
                if not (0<=i<grid.shape[1] and 0<=j<grid.shape[0]): continue
                if grid[j,i]==-1 or grid[j,i]>=100: continue
                if not bres_free(grid,info,cp,cand):                      continue
                if not clearance_ok(grid,info,cand,CLEAR_MIN):           continue
                cost=dist(cand,tgt) - 0.5*CLEAR_MIN         # heurística
                if cost<best_cost: best_cost, best = cost, cand
        return best

    # --------------- ciclo principal --------------- #
    def step(self):
        if None in (self.pose,self.goal,self.grid_msg): return
        cp=(self.pose.position.x,self.pose.position.y)
        patch,grid,info=self.extract_patch()

        tgt,t_mode=self.select_target(cp,grid,info)
        if tgt is None:
            self.get_logger().warn("Sin target visible")
            return
        self.get_logger().info(f"[TARGET] modo={t_mode}  {tgt}")

        # genera nuevo camino si no hay o si se desvió mucho
        if (not self.current_path or
            dist(cp,self.current_path[min(2,len(self.current_path)-1)])>0.8):
            path=[cp]
            for _ in range(60):   # máx. 60 wps
                next_pt=self.flexible_segment(path[-1],tgt,grid,info)
                if next_pt is None: break
                path.append(next_pt)
                if dist(next_pt,tgt)<0.4: break
            path.append(tgt)
            self.current_path=path
            self.get_logger().info(f"[PATH] {self.current_path}")

        # elimina wps alcanzados
        while len(self.current_path)>1 and dist(cp,self.current_path[1])<0.3:
            self.current_path.pop(1)
        # publicar
        self.publish(self.current_path)

    # --------------- publicación RViz --------------- #
    def publish(self,pts):
        hdr=Header()
        hdr.frame_id="map"; hdr.stamp=self.get_clock().now().to_msg()

        path=Path(header=hdr)
        for x,y in pts:
            ps=PoseStamped(header=hdr)
            ps.pose.position.x=x; ps.pose.position.y=y; ps.pose.orientation.w=1.0
            path.poses.append(ps)
        self.path_pub.publish(path)

        mk=Marker(header=hdr,ns="wps",id=0,type=Marker.POINTS,action=Marker.ADD)
        mk.scale=Vector3(x=0.15,y=0.15,z=0.0)
        mk.color.r=mk.color.g=1.0; mk.color.a=1.0
        for x,y in pts[1:]: mk.points.append(Point(x=x,y=y))
        self.wps_pub.publish(mk)

# ----------------------- MAIN ----------------------- #
def main(args=None):
    rclpy.init(args=args)
    node=FlexiblePlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node(); rclpy.shutdown()

if __name__=="__main__":
    main()
