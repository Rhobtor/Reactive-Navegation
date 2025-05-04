# #!/usr/bin/env python3
# # -*- coding: utf-8 -*-
# """
# flexible_planner_infer.py
# ─────────────────────────
# Nodo ROS 2 (rclpy) de **INFERENCIA** que ejecuta la política entrenada
# con *flexible_planner_train.py* para navegar en línea sin continuar
# el aprendizaje.

# • Selección robusta de target (goal visible / frontier válido).
# • Planificación de minipaths flexibles usando la CNN+LSTM entrenada
#   junto a búsqueda en abanico.
# • Seguimiento Pure‑Pursuit 2‑D para generar *cmd_vel*.
# • Publicación de *Path* + *Markers* para RViz.
# • Carga de pesos mediante parámetro ROS `weights_path`.

# Ejemplo de uso (asumiendo que el ejecutable está en tu paquete *flex_nav*):
#     ros2 run flex_nav flexible_planner_infer \
#         --ros-args -p weights_path:=/home/usuario/weights/policy_latest_20250502_101530.weights.h5
# """

# # ============================= IMPORTS ===================================
# import math ,pathlib
# import numpy as np
# import os
# import rclpy
# import tensorflow as tf

# from rclpy.node import Node
# from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# from nav_msgs.msg import Odometry, OccupancyGrid, Path
# from geometry_msgs.msg import PoseArray, PoseStamped, Point, Vector3, Twist
# from visualization_msgs.msg import Marker
# from std_msgs.msg import Header, Bool

# # ============================= CONSTANTES ================================
# PATCH           = 128          # lado del parche (celdas)
# CLEAR_MIN       = 3.4          # holgura mínima (m)
# GOAL_VIS_OK     = 4            # ciclos "goal visible" necesarios
# GOAL_RADIUS     = 3.0          # llegar cuando d<3 m

# RADII  = [2.3, 3.6, 5.9]                      # abanico de radios
# ANGLES = np.linspace(-math.pi/3, math.pi/3, 11)  # ±60° (10 pasos)

# MIN_VEL = 1.0   # m/s
# MAX_VEL = 6.0   # m/s
# DTYPE   = np.float32
# RUN_DIR = pathlib.Path.home() / "/home/rhobtor/reactive/Reactive-Navegation/workspace_full_conected/weights"

# # ============================= UTILIDADES ===============================

# def l2(a, b):
#     """Distancia euclídea 2‑D"""
#     return math.hypot(b[0]-a[0], b[1]-a[1])


# def idx_from_world(info, pt):
#     res = info.resolution
#     return (
#         int((pt[0]-info.origin.position.x)/res),
#         int((pt[1]-info.origin.position.y)/res),
#     )


# def bres_free(grid, info, a, b):
#     """Bresenham entre celdas libres (-1 desconocido, >=100 obstáculo)."""
#     i0, j0 = idx_from_world(info, a)
#     i1, j1 = idx_from_world(info, b)
#     di, dj = abs(i1-i0), abs(j1-j0)
#     si = 1 if i0 < i1 else -1
#     sj = 1 if j0 < j1 else -1
#     err = di - dj
#     H, W = grid.shape
#     while True:
#         if not (0 <= i0 < W and 0 <= j0 < H):
#             return False
#         v = grid[j0, i0]
#         if v == -1 or v >= 100:
#             return False
#         if (i0, j0) == (i1, j1):
#             return True
#         e2 = 2 * err
#         if e2 > -dj:
#             err -= dj
#             i0 += si
#         if e2 < di:
#             err += di
#             j0 += sj


# def clearance_ok(grid, info, pt, r_m):
#     i, j = idx_from_world(info, pt)
#     r = int(r_m / info.resolution)
#     H, W = grid.shape
#     for dj in range(-r, r+1):
#         for di in range(-r, r+1):
#             x, y = i + di, j + dj
#             if 0 <= x < W and 0 <= y < H and (grid[y, x] == -1 or grid[y, x] >= 100):
#                 return False
#     return True

# # ============================= MODELO CNN+LSTM ===========================

# def build_policy():
#     g = tf.keras.Input(shape=(PATCH, PATCH, 1), name="grid")
#     st = tf.keras.Input(shape=(4,), name="state")
#     w0 = tf.keras.Input(shape=(2,), name="wp0")  # dummy para LSTM

#     x = tf.keras.layers.Conv2D(16, 3, padding="same", activation="relu")(g)
#     x = tf.keras.layers.MaxPooling2D()(x)
#     x = tf.keras.layers.Conv2D(32, 3, padding="same", activation="relu")(x)
#     x = tf.keras.layers.GlobalAveragePooling2D()(x)
#     z = tf.keras.layers.Concatenate()([x, st])

#     h0 = tf.keras.layers.Dense(128, activation="tanh")(z)
#     c0 = tf.keras.layers.Dense(128, activation="tanh")(z)
#     lstm = tf.keras.layers.LSTMCell(128)
#     h1, _ = lstm(w0, [h0, c0])

#     delta = tf.keras.layers.Dense(2, activation="tanh")(h1)
#     return tf.keras.Model([g, st, w0], delta, name="policy")

# # ============================= NODO ROS 2 ================================

# class FlexPlannerInference(Node):
#     """Nodo ROS 2 que ejecuta la política entrenada y navega online."""

#     def __init__(self):
#         super().__init__("flexible_inference")

#         # # -- parámetro ruta de pesos --------------------------------
#         # self.declare_parameter("weights_path", "")
#         # weights_path = self.get_parameter("weights_path").get_parameter_value().string_value
#         # if not weights_path:
#         #     self.get_logger().error("Parámetro 'weights_path' no definido.")
#         #     raise RuntimeError("weights_path vacío")

#         # -- modelo --------------------------------------------------
#         self.policy = build_policy()
#         #self.policy.load_weights(weights_path)
#         self.policy.load_weights(str(RUN_DIR/"policy_latest_.weights.h5"))
#         self.get_logger().info(f"Pesos cargados desde {RUN_DIR/'policy_latest.weights.h5'}")

#         # -- subscripciones -----------------------------------------
#         qos = 10
#         self.create_subscription(Odometry, "/odom", self.cb_odom, qos)
#         self.create_subscription(PoseArray, "/goal", self.cb_goal, qos)
#         self.create_subscription(OccupancyGrid, "/occupancy_grid", self.cb_grid, qos)
#         self.create_subscription(
#             PoseArray,
#             "/safe_frontier_points_centroid",
#             self.cb_frontier,
#             qos,
#         )

#         # -- publicadores -------------------------------------------
#         self.path_pub = self.create_publisher(Path, "/global_path_predicted", qos)
#         self.wps_pub = self.create_publisher(Marker, "/path_waypoints_marker", qos)
#         self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", qos)

#         # -- estado --------------------------------------------------
#         self.pose = None
#         self.goal = None
#         self.grid_msg = None
#         self.frontiers = []
#         self.current_path = []
#         self.wp_index = 1
#         self.goal_vis = 0

#         # temporizador principal
#         self.create_timer(0.1, self.step)
#         self.get_logger().info("Nodo de inferencia iniciado ✔️")

#     # -------------------- callbacks --------------------------------
#     def cb_odom(self, m: Odometry):
#         self.pose = m.pose.pose

#     def cb_goal(self, m: PoseArray):
#         self.goal = m.poses[0] if m.poses else None

#     def cb_grid(self, m: OccupancyGrid):
#         self.grid_msg = m

#     def cb_frontier(self, m: PoseArray):
#         self.frontiers = [(p.position.x, p.position.y) for p in m.poses]

#     # ------------------ utilidades parche --------------------------
#     def extract_patch(self):
#         info = self.grid_msg.info
#         H, W = info.height, info.width
#         arr = np.array(self.grid_msg.data, dtype=np.int8).reshape((H, W))

#         cp = (self.pose.position.x, self.pose.position.y)
#         ci = int((cp[0]-info.origin.position.x)/info.resolution)
#         cj = int((cp[1]-info.origin.position.y)/info.resolution)

#         i_lo, i_hi = ci-PATCH//2, ci+PATCH//2
#         j_lo, j_hi = cj-PATCH//2, cj+PATCH//2
#         i0, i1 = max(i_lo, 0), min(i_hi, W)
#         j0, j1 = max(j_lo, 0), min(j_hi, H)

#         patch = arr[j0:j1, i0:i1]
#         pad = ((j0-j_lo, j_hi-j1), (i0-i_lo, i_hi-i1))
#         patch = np.pad(patch, pad, "constant", constant_values=-1)
#         norm = ((patch + 1) / 101.0).astype(np.float32)  # [-1,100]→[0,1]
#         return np.expand_dims(norm, -1), arr, info

#     # --------------- selección de target ---------------------------
#     def choose_target(self, cp, grid, info):
#         if self.goal is None:
#             return None, "NONE"
#         gp = (self.goal.position.x, self.goal.position.y)
#         self.goal_vis = self.goal_vis + 1 if bres_free(grid, info, cp, gp) else 0
#         if self.goal_vis >= GOAL_VIS_OK:
#             return gp, "GOAL"
#         cand = [f for f in self.frontiers if bres_free(grid, info, cp, f) and clearance_ok(grid, info, f, CLEAR_MIN)]
#         if cand:
#             return min(cand, key=lambda f: l2(f, gp)), "FRONTIER"
#         return None, "NONE"

#     # --------------- waypoint via policy ---------------------------
#     def next_waypoint(self, cp, tgt, grid, info, patch):
#         state = np.array([0, 0, tgt[0]-cp[0], tgt[1]-cp[1]], np.float32)
#         delta = self.policy([patch[None, ...], state[None, :], np.zeros((1,2), np.float32)], training=False)[0].numpy()
#         ang0 = math.atan2(delta[1], delta[0]) if np.linalg.norm(delta) > 1e-3 else math.atan2(tgt[1]-cp[1], tgt[0]-cp[0])
#         best, best_cost = None, float("inf")
#         for r in RADII:
#             for off in ANGLES:
#                 ang = ang0 + off
#                 cand = (cp[0] + r*math.cos(ang), cp[1] + r*math.sin(ang))
#                 i, j = idx_from_world(info, cand)
#                 if not (0 <= i < grid.shape[1] and 0 <= j < grid.shape[0]):
#                     continue
#                 if grid[j, i] == -1 or grid[j, i] >= 100:
#                     continue
#                 if not bres_free(grid, info, cp, cand):
#                     continue
#                 if not clearance_ok(grid, info, cand, CLEAR_MIN):
#                     continue
#                 cost = l2(cand, tgt) - 0.5*CLEAR_MIN
#                 if cost < best_cost:
#                     best, best_cost = cand, cost
#         return best

#     # --------------- pure‑pursuit 2‑D -------------------------------------
#     def _yaw_from_quaternion(self, q):
#         siny_cosp = 2.0*(q.w*q.z + q.x*q.y)
#         cosy_cosp = 1.0 - 2.0*(q.y*q.y + q.z*q.z)
#         return math.atan2(siny_cosp, cosy_cosp)

#     def _global_to_local(self, dx, dy, yaw):
#         cos_y, sin_y = math.cos(-yaw), math.sin(-yaw)
#         return dx*cos_y - dy*sin_y, dx*sin_y + dy*cos_y

#     def _next_target_index(self, cp, Ld):
#         idx = self.wp_index
#         while idx + 1 < len(self.current_path) and l2(cp, self.current_path[idx]) < Ld:
#             idx += 1
#         return idx

#     def follow_path(self, cp):
#         if self.wp_index >= len(self.current_path):
#             self.cmd_pub.publish(Twist())
#             return
#         v_nom = MAX_VEL
#         Ld = max(0.25, 0.4*v_nom)
#         self.wp_index = self._next_target_index(cp, Ld)
#         tgt = self.current_path[self.wp_index]
#         dx_g, dy_g = tgt[0]-cp[0], tgt[1]-cp[1]
#         dist = math.hypot(dx_g, dy_g)
#         if dist < 1e-3:
#             self.cmd_pub.publish(Twist())
#             return
#         yaw = self._yaw_from_quaternion(self.pose.orientation)
#         dx, dy = self._global_to_local(dx_g, dy_g, yaw)
#         alpha = math.atan2(dy, dx)
#         kappa = 2.0*math.sin(alpha)/Ld
#         v_lin = max(MIN_VEL, min(MAX_VEL, 1.8*dist))
#         omega = kappa*v_lin
#         cmd = Twist()
#         cmd.linear.x = v_lin
#         cmd.angular.z = omega
#         self.cmd_pub.publish(cmd)

#     # --------------- publicación path & markers ---------------------------
#     def publish_path(self, pts):
#         hdr = Header(frame_id="map", stamp=self.get_clock().now().to_msg())
#         path = Path(header=hdr)
#         for x, y in pts:
#             ps = PoseStamped(header=hdr)
#             ps.pose.position.x = x
#             ps.pose.position.y = y
#             ps.pose.orientation.w = 1.0
#             path.poses.append(ps)
#         self.path_pub.publish(path)
#         mk = Marker(header=hdr, ns="wps", id=0, type=Marker.POINTS, action=Marker.ADD)
#         mk.scale = Vector3(x=0.15, y=0.15, z=0.0)
#         mk.color.r = mk.color.g = 1.0
#         mk.color.a = 1.0
#         mk.points = [Point(x=x, y=y) for x, y in pts[1:]]
#         self.wps_pub.publish(mk)

#     # ----------------- ciclo principal -----------------------------------
#     def step(self):
#         if None in (self.pose, self.goal, self.grid_msg):
#             return
#         cp = (self.pose.position.x, self.pose.position.y)
#         patch, grid, info = self.extract_patch()
#         tgt, mode = self.choose_target(cp, grid, info)
#         if tgt is None:
#             self.get_logger().warn("Sin target válido.")
#             return
#         need_replan = (not self.current_path) or l2(cp, self.current_path[min(2, len(self.current_path)-1)]) > 0.8
#         if need_replan:
#             wp = self.next_waypoint(cp, tgt, grid, info, patch)
#             if wp is None:
#                 self.get_logger().warn("next_waypoint no encontró solución")
#                 return
#             self.current_path = [cp, wp, tgt]
#             self.wp_index = 1
#             self.get_logger().info(f"[PATH] nuevo con {len(self.current_path)} pts")
#         self.follow_path(cp)
#         self.publish_path(self.current_path)
#         if l2(cp, tgt) < GOAL_RADIUS:
#             self.get_logger().info("🎉 Objetivo alcanzado")
#             self.current_path.clear()

# # ============================= main() ====================================

# def main(args=None):
#     rclpy.init(args=args)
#     node = FlexPlannerInference()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()


# # #!/usr/bin/env python3
# # # -*- coding: utf-8 -*-
# # """
# # flexible_planner_infer_lvl0.py
# # ──────────────────────────────
# # Nodo ROS 2: inferencias con el modelo de nivel-0 (sin entrenamiento en línea)
# #   • Selección robusta de target   (goal visible / frontera segura)
# #   • Generación del minipath + abanico clásico de nivel-0
# #   • Seguimiento Pure-Pursuit con velocidad fija
# #   • Publica Path y Markers para RViz
# # """

# # # ------------- Imports ----------------------------------------------------
# # import math, pathlib, time, numpy as np, rclpy, tensorflow as tf
# # from rclpy.node      import Node
# # from rclpy.qos       import QoSProfile, ReliabilityPolicy, DurabilityPolicy
# # from nav_msgs.msg    import Path, Odometry, OccupancyGrid
# # from geometry_msgs.msg import (PoseArray, PoseStamped, Point,
# #                                Vector3, Twist)
# # from visualization_msgs.msg import Marker
# # from std_msgs.msg     import Header, Bool

# # # ------------- Parámetros principales ------------------------------------
# # PATCH   = 128
# # CLEAR   = 3.4                     # m — holgura mínima
# # RADII   = [2.3, 3.6, 5.9]
# # ANGLES  = np.linspace(-math.pi/3, math.pi/3, 11)      # ±60°
# # LOOK_A  = 3.0                     # m  (pure-pursuit)
# # MAX_VEL = 2.0
# # RUN_DIR = pathlib.Path.home() / "reactive/Reactive-Navegation/workspace_full_conected/weights"

# # # ------------- Utilidades geométricas ------------------------------------
# # def l2(a,b): return math.hypot(b[0]-a[0], b[1]-a[1])

# # def idx_from_world(info, pt):
# #     res = info.resolution
# #     return (int((pt[0]-info.origin.position.x)/res),
# #             int((pt[1]-info.origin.position.y)/res))

# # def bres_free(grid, info, a, b):
# #     i0,j0 = idx_from_world(info,a);  i1,j1 = idx_from_world(info,b)
# #     di,dj = abs(i1-i0), abs(j1-j0);  si = 1 if i0<i1 else -1;  sj = 1 if j0<j1 else -1
# #     err = di-dj;   H,W = grid.shape
# #     while True:
# #         if not (0<=i0<W and 0<=j0<H):                    return False
# #         v = grid[j0,i0]
# #         if v == -1 or v >= 100:                          return False
# #         if (i0,j0) == (i1,j1):                           return True
# #         e2 = 2*err
# #         if e2>-dj: err-=dj; i0+=si
# #         if e2< di: err+=di; j0+=sj

# # def clearance_ok(grid, info, pt, r_m):
# #     i,j = idx_from_world(info,pt);  r = int(r_m/info.resolution)
# #     H,W = grid.shape
# #     for dj in range(-r,r+1):
# #         for di in range(-r,r+1):
# #             x,y=i+di,j+dj
# #             if 0<=x<W and 0<=y<H and (grid[y,x]==-1 or grid[y,x]>=100):
# #                 return False
# #     return True

# # # ------------- Red CNN + LSTM exactamente como nivel-0 -------------------
# # def build_policy():
# #     g   = tf.keras.Input(shape=(PATCH,PATCH,1), name="grid")
# #     st  = tf.keras.Input(shape=(4,),            name="state")
# #     w0  = tf.keras.Input(shape=(2,),            name="wp0")
# #     x = tf.keras.layers.Conv2D(16,3,padding="same",activation="relu")(g)
# #     x = tf.keras.layers.MaxPooling2D()(x)
# #     x = tf.keras.layers.Conv2D(32,3,padding="same",activation="relu")(x)
# #     x = tf.keras.layers.GlobalAveragePooling2D()(x)
# #     z = tf.keras.layers.Concatenate()([x,st])
# #     h0 = tf.keras.layers.Dense(128,activation="tanh")(z)
# #     c0 = tf.keras.layers.Dense(128,activation="tanh")(z)
# #     lstm = tf.keras.layers.LSTMCell(128)
# #     h1,_ = lstm(w0,[h0,c0])
# #     delta = tf.keras.layers.Dense(2,activation="tanh")(h1)
# #     return tf.keras.Model([g,st,w0], delta, name="policy")

# # # ------------- Nodo principal --------------------------------------------
# # class FlexPlannerInfer(Node):
# #     def __init__(self):
# #         super().__init__("flexible_planner_lvl0_infer")

# #         qos = 10
# #         self.create_subscription(Odometry,      "/odom",           self.cb_odom,  qos)
# #         self.create_subscription(PoseArray,     "/goal",           self.cb_goal,  qos)
# #         self.create_subscription(OccupancyGrid, "/occupancy_grid", self.cb_grid,  qos)
# #         self.create_subscription(PoseArray,     "/safe_frontier_points_centroid",
# #                                  self.cb_frontiers, qos)

# #         self.path_pub = self.create_publisher(Path,   "/global_path_predicted", qos)
# #         self.mk_pub   = self.create_publisher(Marker, "/path_waypoints_marker", qos)
# #         self.cmd_pub  = self.create_publisher(Twist,  "/cmd_vel", qos)

# #         # ---- estado interno ----------------------------------------
# #         self.pose = None
# #         self.goal = None
# #         self.grid = None
# #         self.frontiers = []
# #         self.current_path = []
# #         self.wp_index = 1     # 0 = pose, 1 = primer waypoint

# #         # ---- política ---------------------------------------------
# #         self.policy = build_policy()
# #         for l in self.policy.layers: l.trainable = False
# #         ckpt = RUN_DIR / "policy_latest_20250503_224102.weights.h5"
# #         self.policy.load_weights(ckpt)
# #         self.get_logger().info(f"Pesos cargados de {ckpt.name}")

# #         self.create_timer(0.1, self.step)

# #     # ---------- Callbacks ROS --------------------------------------
# #     def cb_odom(self,msg): self.pose = msg.pose.pose
# #     def cb_goal(self,msg): self.goal = msg.poses[0] if msg.poses else None
# #     def cb_grid(self,msg): self.grid = msg
# #     def cb_frontiers(self,msg):
# #         self.frontiers = [(p.position.x,p.position.y) for p in msg.poses]

# #     # ---------- Auxiliares de parche --------------------------------
# #     def extract_patch(self):
# #         info = self.grid.info
# #         H,W  = info.height, info.width
# #         arr  = np.array(self.grid.data,dtype=np.int8).reshape((H,W))
# #         cp   = (self.pose.position.x, self.pose.position.y)
# #         ci   = int((cp[0]-info.origin.position.x)/info.resolution)
# #         cj   = int((cp[1]-info.origin.position.y)/info.resolution)
# #         i0,i1 = ci-PATCH//2, ci+PATCH//2
# #         j0,j1 = cj-PATCH//2, cj+PATCH//2
# #         i0c,i1c = max(i0,0), min(i1,W)
# #         j0c,j1c = max(j0,0), min(j1,H)
# #         patch = arr[j0c:j1c, i0c:i1c]
# #         pad   = ((j0c-j0, j1-j1c), (i0c-i0, i1-i1c))
# #         patch = np.pad(patch,pad,'constant',constant_values=-1)
# #         norm  = ((patch+1)/101.0).astype(np.float32)
# #         return np.expand_dims(norm,-1), arr, info

# #     # ---------- Target robusto -------------------------------------
# #     def choose_target(self, cp, grid, info):
# #         gp = (self.goal.position.x, self.goal.position.y)
# #         if bres_free(grid,info,cp,gp): return gp
# #         cands = [f for f in self.frontiers
# #                  if bres_free(grid,info,cp,f) and clearance_ok(grid,info,f,CLEAR)]
# #         return min(cands,key=lambda f:l2(f,gp)) if cands else gp

# #     # ---------- Abanico + CNN/LSTM ----------------------------------
# #     def next_waypoint(self, cp, tgt, grid, info, patch):
# #         state = np.array([0,0,tgt[0]-cp[0], tgt[1]-cp[1]], np.float32)
# #         delta = self.policy([patch[None,...],
# #                              state[None,:],
# #                              np.zeros((1,2),np.float32)], training=False)[0].numpy()
# #         ang0  = math.atan2(delta[1],delta[0]) if np.linalg.norm(delta)>1e-3 \
# #                 else math.atan2(tgt[1]-cp[1], tgt[0]-cp[0])

# #         best=None; best_cost=float("inf")
# #         for r in RADII:
# #             for off in ANGLES:
# #                 ang  = ang0+off
# #                 cand = (cp[0]+r*math.cos(ang), cp[1]+r*math.sin(ang))
# #                 i,j  = idx_from_world(info,cand)
# #                 if not (0<=i<grid.shape[1] and 0<=j<grid.shape[0]): continue
# #                 if grid[j,i]==-1 or grid[j,i]>=100:                 continue
# #                 if not bres_free(grid,info,cp,cand):                continue
# #                 if not clearance_ok(grid,info,cand,CLEAR):          continue
# #                 cost = l2(cand,tgt) - 0.5*CLEAR
# #                 if cost < best_cost: best_cost, best = cost, cand
# #         return best

# #     # ---------- Seguimiento Pure-Pursuit ----------------------------
# #     def _yaw(self,q):
# #         return math.atan2(2*(q.w*q.z+q.x*q.y),
# #                           1-2*(q.y*q.y+q.z*q.z))

# #     def _to_base(self,dx,dy,yaw):
# #         c,s = math.cos(-yaw), math.sin(-yaw)
# #         return dx*c - dy*s, dx*s + dy*c

# #     def follow_path(self,cp):
# #         if self.wp_index >= len(self.current_path):
# #             self.cmd_pub.publish(Twist()); return

# #         v_nom  = MAX_VEL
# #         Ld     = max(0.25, 0.4*v_nom)
# #         while (self.wp_index+1 < len(self.current_path) and
# #                l2(cp, self.current_path[self.wp_index]) < Ld):
# #             self.wp_index += 1
# #         tgt = self.current_path[self.wp_index]

# #         dx_g,dy_g = tgt[0]-cp[0], tgt[1]-cp[1]
# #         dist      = math.hypot(dx_g,dy_g)
# #         if dist < 1e-3: self.cmd_pub.publish(Twist()); return

# #         yaw = self._yaw(self.pose.orientation)
# #         dx,dy = self._to_base(dx_g,dy_g,yaw)
# #         alpha = math.atan2(dy,dx)
# #         kappa = 2*math.sin(alpha)/Ld
# #         v_lin = v_nom
# #         omega = kappa*v_lin

# #         cmd = Twist(); cmd.linear.x=v_lin; cmd.angular.z=omega
# #         self.cmd_pub.publish(cmd)

# #     # ---------- Publicación RViz -----------------------------------
# #     def publish_path(self,pts):
# #         hdr = Header(frame_id="map", stamp=self.get_clock().now().to_msg())
# #         path = Path(header=hdr)
# #         for x,y in pts:
# #             ps = PoseStamped(header=hdr)
# #             ps.pose.position.x = x; ps.pose.position.y = y; ps.pose.orientation.w = 1.0
# #             path.poses.append(ps)
# #         self.path_pub.publish(path)

# #         mk = Marker(header=hdr, ns="wps", id=0,
# #                     type=Marker.POINTS, action=Marker.ADD)
# #         mk.scale = Vector3(x=0.15,y=0.15,z=0.0)
# #         mk.color.r = mk.color.g = 1.0; mk.color.a = 1.0
# #         mk.points  = [Point(x=float(x),y=float(y)) for x,y in pts[1:]]
# #         self.mk_pub.publish(mk)

# #     # ---------- Bucle principal ------------------------------------
# #     def step(self):
# #         if None in (self.pose, self.goal, self.grid): return
# #         cp = (self.pose.position.x, self.pose.position.y)

# #         patch,arr,info = self.extract_patch()
# #         tgt = self.choose_target(cp, arr, info)
# #         if tgt is None: return

# #         # ¿necesitamos nueva ruta?
# #         if not self.current_path or l2(cp,self.current_path[-1])>2.0:
# #             wp = self.next_waypoint(cp,tgt,arr,info,patch)
# #             if wp is None: return
# #             self.current_path = [cp, wp, tgt]     # minipath sencillo
# #             self.wp_index = 1
# #         self.follow_path(cp)
# #         self.publish_path(self.current_path)

# # # ------------- MAIN -------------------------------------------------------
# # def main(args=None):
# #     rclpy.init(args=args)
# #     node = FlexPlannerInfer()
# #     try:
# #         rclpy.spin(node)
# #     except KeyboardInterrupt:
# #         pass
# #     node.destroy_node();  rclpy.shutdown()

# # if __name__ == "__main__":
# #     main()




#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
path_ppo_cnn_lstm_inference.py
──────────────────────────────
  • Usa la política CNN+LSTM de nivel‑0 (Δx, Δy)
  • Selecciona target robusto   (goal visible / frontier)
  • Genera un minipath flexible con abanico de seguridad nivel‑0
  • Publica Path y cmd_vel (Pure‑Pursuit)
  • **Sin** aprendizaje, **sin** reset automático del mundo
"""

# ─────────── Imports ────────────────────────────────────────────────────
import math, pathlib, time, numpy as np, rclpy, tensorflow as tf
from rclpy.node import Node
from rclpy.qos  import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg      import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import PoseArray, PoseStamped, Twist, Point, Vector3
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, Bool

# ─────────── Parámetros globales (idénticos al training) ────────────────
PATCH = 128
CLEAR_MIN   = 3.4
GOAL_VIS_OK = 4
GOAL_RADIUS = 2.5
RADII  = [2.3, 3.6, 5.9]
ANGLES = np.linspace(-math.pi/3, math.pi/3, 11)
MIN_VEL, MAX_VEL = 1.0, 6.0
LOOK_A = 3.0                                  # look‑ahead
RUN_DIR = pathlib.Path.home() / "reactive/Reactive-Navegation/workspace_full_conected/weights"

# ─────────── Utilidades geométricas ─────────────────────────────────────
def l2(a,b): return math.hypot(b[0]-a[0], b[1]-a[1])

def idx_from_world(info, pt):
    res = info.resolution
    return (int((pt[0]-info.origin.position.x)/res),
            int((pt[1]-info.origin.position.y)/res))

def bres_free(grid, info, a, b):
    i0,j0 = idx_from_world(info,a); i1,j1 = idx_from_world(info,b)
    di,dj = abs(i1-i0), abs(j1-j0); si = 1 if i0<i1 else -1; sj = 1 if j0<j1 else -1
    err=di-dj; H,W = grid.shape
    while True:
        if not (0<=i0<W and 0<=j0<H): return False
        v = grid[j0,i0];  # desconocido u obstáculo
        if v == -1 or v >= 100:       return False
        if (i0,j0)==(i1,j1):          return True
        e2=2*err
        if e2>-dj: err-=dj; i0+=si
        if e2< di: err+=di; j0+=sj

def clearance_ok(grid,info,pt,r_m):
    i,j = idx_from_world(info,pt); r = int(r_m/info.resolution)
    H,W = grid.shape
    for dj in range(-r,r+1):
        for di in range(-r,r+1):
            x,y = i+di, j+dj
            if 0<=x<W and 0<=y<H and (grid[y,x]==-1 or grid[y,x]>=100):
                return False
    return True

# ─────────── Policy nivel‑0  (Δx,Δy) ────────────────────────────────────
def build_policy():
    g   = tf.keras.Input(shape=(PATCH,PATCH,1))
    st  = tf.keras.Input(shape=(4,))
    w0  = tf.keras.Input(shape=(2,))
    x = tf.keras.layers.Conv2D(16,3,padding="same",activation="relu")(g)
    x = tf.keras.layers.MaxPooling2D()(x)
    x = tf.keras.layers.Conv2D(32,3,padding="same",activation="relu")(x)
    x = tf.keras.layers.GlobalAveragePooling2D()(x)
    z = tf.keras.layers.Concatenate()([x,st])
    h0 = tf.keras.layers.Dense(128,activation="tanh")(z)
    c0 = tf.keras.layers.Dense(128,activation="tanh")(z)
    delta,_ = tf.keras.layers.LSTMCell(128)(w0,[h0,c0])
    out = tf.keras.layers.Dense(2,activation="tanh")(delta)
    return tf.keras.Model([g,st,w0], out)

# ─────────── Nodo principal ─────────────────────────────────────────────
class FlexPlannerInference(Node):
    def __init__(self):
        super().__init__("flexible_planner_lvl0_inference")

        # Suscripciones -----------------------------------------------------
        qos = 10
        self.create_subscription(Odometry,      "/odom",            self.cb_odom,  qos)
        self.create_subscription(PoseArray,     "/goal",            self.cb_goal,  qos)
        self.create_subscription(OccupancyGrid, "/occupancy_grid",  self.cb_grid,  qos)
        self.create_subscription(PoseArray,     "/safe_frontier_points_centroid",
                                 self.cb_frontier, qos)
        # Publicadores ------------------------------------------------------
        self.path_pub = self.create_publisher(Path,   "/global_path_predicted", 10)
        self.mk_pub   = self.create_publisher(Marker, "/path_waypoints_marker",10)
        self.cmd_pub  = self.create_publisher(Twist,  "/cmd_vel", 10)

        # Estado ------------------------------------------------------------
        self.pose=None; self.twist=None
        self.goal=None; self.grid_msg=None
        self.frontiers=[]
        self.goal_vis = 0
        self.current_path=[]; self.wp_index=1

        # Policy ------------------------------------------------------------
        self.policy = build_policy()
        # Carga pesos más recientes
        try:
            ckpt = sorted(RUN_DIR.glob("policy_latest.weights.h5"))[-1]
            self.policy.load_weights(ckpt)
            self.get_logger().info(f"Pesos cargados: {ckpt.name}")
        except Exception as e:
            self.get_logger().warn(f"No se pudieron cargar pesos: {e}")

        # Timer principal
        self.create_timer(0.1, self.step)
        self.get_logger().info("Nodo de inferencia nivel‑0 listo ✅")

    # ---------------- Callbacks -------------------------------------------
    def cb_odom(self,m): self.pose = m.pose.pose
    def cb_goal(self,m): self.goal = m.poses[0] if m.poses else None
    def cb_grid(self,m): self.grid_msg = m
    def cb_frontier(self,m): self.frontiers = [(p.position.x,p.position.y) for p in m.poses]

    # ---------------- Helpers ---------------------------------------------
    def _yaw(self,q):
        return math.atan2(2*(q.w*q.z + q.x*q.y),
                          1-2*(q.y*q.y + q.z*q.z))

    def _global_to_local(self,dx,dy,yaw):
        c,s = math.cos(-yaw), math.sin(-yaw)
        return dx*c - dy*s, dx*s + dy*c

    def extract_patch(self):
        info=self.grid_msg.info
        H,W=info.height,info.width
        arr=np.array(self.grid_msg.data,dtype=np.int8).reshape((H,W))
        cp=(self.pose.position.x,self.pose.position.y)
        ci=int((cp[0]-info.origin.position.x)/info.resolution)
        cj=int((cp[1]-info.origin.position.y)/info.resolution)
        i_lo,i_hi=ci-PATCH//2,ci+PATCH//2
        j_lo,j_hi=cj-PATCH//2,cj+PATCH//2
        i0,i1=max(i_lo,0),min(i_hi,W); j0,j1=max(j_lo,0),min(j_hi,H)
        patch=arr[j0:j1,i0:i1]
        pad=((j0-j_lo,j_hi-j1),(i0-i_lo,i_hi-i1))
        patch=np.pad(patch,pad,'constant',constant_values=-1)
        norm=((patch+1)/101.0).astype(np.float32)
        return np.expand_dims(norm,-1), arr, info

    # ---------------- Target robusto --------------------------------------
    def choose_target(self,cp,grid,info):
        gp=(self.goal.position.x,self.goal.position.y)
        self.goal_vis = self.goal_vis+1 if bres_free(grid,info,cp,gp) else 0
        if self.goal_vis>=GOAL_VIS_OK: return gp,"GOAL"
        cand=[f for f in self.frontiers
              if bres_free(grid,info,cp,f) and clearance_ok(grid,info,f,CLEAR_MIN)]
        if cand: return min(cand,key=lambda f:l2(f,gp)),"FRONTIER"
        return None,"NONE"

    # ---------------- Siguiente waypoint (policy + abanico) ---------------
    def next_waypoint(self,cp,tgt,grid,info,patch):
        state=np.array([0,0,tgt[0]-cp[0],tgt[1]-cp[1]],np.float32)
        delta=self.policy([patch[None,...], state[None,:],
                           np.zeros((1,2),np.float32)],training=False)[0].numpy()
        ang0=math.atan2(delta[1],delta[0]) if np.linalg.norm(delta)>1e-3 \
             else math.atan2(tgt[1]-cp[1], tgt[0]-cp[0])

        best=None; best_cost=float("inf")
        for r in RADII:
            for off in ANGLES:
                cand=(cp[0]+r*math.cos(ang0+off), cp[1]+r*math.sin(ang0+off))
                i,j=idx_from_world(info,cand)
                if not (0<=i<grid.shape[1] and 0<=j<grid.shape[0]): continue
                if grid[j,i]==-1 or grid[j,i]>=100: continue
                if not bres_free(grid,info,cp,cand): continue
                if not clearance_ok(grid,info,cand,CLEAR_MIN): continue
                cost=l2(cand,tgt) - 0.5*CLEAR_MIN
                if cost<best_cost: best_cost,best=cost,cand
        return best

    # ---------------- Follow‑Path (Pure‑Pursuit) ---------------------------
    def _next_target_idx(self,cp,Ld):
        idx=self.wp_index
        while idx+1<len(self.current_path) and l2(cp,self.current_path[idx])<Ld:
            idx+=1
        return idx

    def follow_path(self,cp):
        if self.wp_index>=len(self.current_path):
            self.cmd_pub.publish(Twist()); return
        v_nom=MAX_VEL;  Ld=max(0.25,0.4*v_nom)
        self.wp_index=self._next_target_idx(cp,Ld)
        tgt=self.current_path[self.wp_index]

        dx_g,dy_g = tgt[0]-cp[0], tgt[1]-cp[1]
        dist = math.hypot(dx_g,dy_g)
        if dist<1e-3: self.cmd_pub.publish(Twist()); return

        yaw=self._yaw(self.pose.orientation)
        dx,dy=self._global_to_local(dx_g,dy_g,yaw)
        alpha=math.atan2(dy,dx)
        kappa=2*math.sin(alpha)/Ld
        v_lin=max(MIN_VEL,min(MAX_VEL,1.8*dist))
        cmd=Twist(); cmd.linear.x=v_lin; cmd.angular.z=kappa*v_lin
        self.cmd_pub.publish(cmd)

    # ---------------- Publicación RViz ------------------------------------
    def publish_path(self,pts):
        hdr=Header(frame_id="map",stamp=self.get_clock().now().to_msg())
        path=Path(header=hdr)
        for x,y in pts:
            ps=PoseStamped(header=hdr); ps.pose.position.x=x; ps.pose.position.y=y
            ps.pose.orientation.w=1.0; path.poses.append(ps)
        self.path_pub.publish(path)

        mk=Marker(header=hdr,ns="wps",id=0,type=Marker.POINTS,action=Marker.ADD)
        mk.scale=Vector3(x=0.15,y=0.15,z=0.0); mk.color.r=mk.color.g=1.0; mk.color.a=1.0
        mk.points=[Point(x=float(x),y=float(y)) for x,y in pts[1:]]
        self.mk_pub.publish(mk)

    # ---------------- Loop principal --------------------------------------
    def step(self):
        if None in (self.pose,self.goal,self.grid_msg): return
        cp = (self.pose.position.x,self.pose.position.y)

        patch,grid,info = self.extract_patch()
        tgt,_ = self.choose_target(cp,grid,info)
        if tgt is None: return

        if not self.current_path or self.wp_index>=len(self.current_path):
            self.current_path=[cp]
        wp = self.next_waypoint(cp,tgt,grid,info,patch)
        if wp is None: return
        self.current_path=[cp,wp,tgt]        # minipath 2‑segmentos
        self.wp_index=1

        self.follow_path(cp)
        self.publish_path(self.current_path)

# ─────────── main ────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node=FlexPlannerInference()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node(); rclpy.shutdown()

if __name__=="__main__":
    main()
