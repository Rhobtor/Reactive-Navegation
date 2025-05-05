
##################################################################
##################################################################
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
flexible_planner_train.py
─────────────────────────
Nodo ROS 2 que entrena online con PPO:
  • Selección robusta del target (goal visible / frontier válido)
  • Génesis de minipaths flexibles (policy CNN + LSTM + búsqueda abanico)
  • Recompensas por progreso, llegada, colisión, seguridad, longitud de ruta
  • Publicación de Path + Markers y topics auxiliares
  • Guardado de pesos por episodio y métricas en TensorBoard
"""

# ==============  IMPORTS  =================================================
import math, random, time, os, pathlib, copy, numpy as np, tensorflow as tf
import rclpy
from rclpy.node import Node
from rclpy.qos  import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg      import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import (PoseArray, PoseStamped,
                               Point, Vector3, Twist)
from visualization_msgs.msg import Marker
from std_msgs.msg       import Header, Bool
from scipy.ndimage      import (binary_dilation,
                                generate_binary_structure,
                                distance_transform_edt)

import datetime

# ==============  PARÁMETROS GLOBALES  =====================================
PATCH           = 128                       # lado del parche (celdas)
CLEAR_MIN       = 3.4                       # m (holgura waypoint)
GOAL_VIS_OK     = 4                         # ciclos “goal visible” → OK
GOAL_RADIUS     = 3.0                       # m para “goal reached”
RADII           = [2.3, 3.6,5.9]           # radios candidatos
ANGLES          = np.linspace(-math.pi/3,
                               math.pi/3, 11)   # ±60° (10 pasos)
MAX_WPS_EP      = 60
# Velocidades límite
MIN_VEL = 1.0          # m/s  (velocidad mínima deseada)
MAX_VEL = 6.0          # m/s  (velocidad máxima permitida)
LOOK_A  = 2.0    # m (aceptación)
# PPO
ROLLOUT_STEPS   = 1024
BATCH_SZ        = 256
EPOCHS          = 500
MAX_UPDATES     = 10
GAMMA           = 0.99
GAE_LAMBDA      = 0.95
CLIP_EPS        = 0.2
LR_ACTOR        = 3e-4
LR_CRITIC       = 1e-3
STD_START       = 0.3
STD_MIN         = 0.05
STD_DECAY       = 0.995
MAX_EPISODES = 300 
MAX_TILT = 0.5
SMOOTH = 0.35                   #   0→sin filtro  1→muy lento
ACC_LIM = 5.0                   # m/s² máx. (Δv por paso / Δt≈0.1 s)


# ---------- Behaviour-Cloning / Teacher --------------------------
LAMBDA_BC_START = 1.0     # peso inicial (clava al teacher)
LAMBDA_BC_END   = 0.0     # peso final   (solo PPO)
BC_DECAY_STEPS  = 2_000   # nº updates para pasar de START a END



DTYPE = np.float32

MIN_FIRST_SEG   = 0.60                     # [m] tramo mínimo inicial
REV_ANGLE_MAX   = math.radians(100)        # ≤100° se considera “frontal”
ALPHA_IIR_VEL   = 0.6                      # filtro primera orden velocidad


RUN_DIR = pathlib.Path.home() / "/home/rhobtor/reactive/Reactive-Navegation/workspace_full_conected/weights"
RUN_DIR.mkdir(exist_ok=True)

# ==============  UTILS GEOMÉTRICOS  =======================================
def l2(a, b):  # distancia euclídea 2-D
    return math.hypot(b[0]-a[0], b[1]-a[1])

def idx_from_world(info, pt):
    res = info.resolution
    return (int((pt[0]-info.origin.position.x)/res),
            int((pt[1]-info.origin.position.y)/res))

def distance(a,b): return math.hypot(b[0]-a[0], b[1]-a[1])

def fuse_dynamic_layer(grid_static, obstacles, info, decay_steps=5):
    grid = grid_static.copy()
    res  = info.resolution
    H, W = grid.shape
    for obs in obstacles:                       # PoseArray con vel.z = radius
        x, y, r = obs.position.x, obs.position.y, obs.orientation.w
        i0 = int((x - info.origin.position.x) / res)
        j0 = int((y - info.origin.position.y) / res)
        rad = int(r / res)
        for dj in range(-rad, rad+1):
            for di in range(-rad, rad+1):
                if di*di+dj*dj <= rad*rad:
                    j, i = j0+dj, i0+di
                    if 0 <= j < H and 0 <= i < W:
                        grid[j, i] = min(100, grid[j, i]+100)  # marca ocupado
    return grid

def safe_shadow(cand, obstacles, τ=1.0):
    cx, cy = cand
    for obs in obstacles:
        ox, oy = obs.position.x, obs.position.y
        vx, vy = obs.orientation.x, obs.orientation.y
        r      = obs.orientation.w
        relx, rely = ox - cx, oy - cy
        tc = -(relx*vx + rely*vy) / (vx*vx + vy*vy + 1e-3)
        tc = max(0.0, tc)
        dx = relx + vx*tc
        dy = rely + vy*tc
        if tc < τ and math.hypot(dx, dy) < r + 0.3:
            return False
    return True



def bres_free(grid, info, a, b):
    """Bresenham + bloqueo: (-1) desconocido ó >=100 obstáculo."""
    i0,j0 = idx_from_world(info,a)
    i1,j1 = idx_from_world(info,b)
    di,dj = abs(i1-i0), abs(j1-j0)
    si = 1 if i0<i1 else -1
    sj = 1 if j0<j1 else -1
    err=di-dj
    H,W = grid.shape
    while True:
        if not (0<=i0<W and 0<=j0<H): return False
        v = grid[j0,i0]
        if v==-1 or v>=100:          return False
        if (i0,j0)==(i1,j1):         return True
        e2=2*err
        if e2>-dj: err-=dj; i0+=si
        if e2< di: err+=di; j0+=sj

def clearance_ok(grid, info, pt, r_m):
    i,j = idx_from_world(info,pt)
    r = int(r_m/info.resolution)
    H,W = grid.shape
    for dj in range(-r, r+1):
        for di in range(-r, r+1):
            x,y = i+di, j+dj
            if 0<=x<W and 0<=y<H and (grid[y,x]==-1 or grid[y,x]>=100):
                return False
    return True





# ==============  RED CNN + LSTM  ==========================================
def build_policy():
    g   = tf.keras.Input(shape=(PATCH,PATCH,1), name="grid")
    st  = tf.keras.Input(shape=(4,),            name="state")
    w0  = tf.keras.Input(shape=(2,),            name="wp0")
    # CNN
    x = tf.keras.layers.Conv2D(16,3,padding="same",activation="relu")(g)
    x = tf.keras.layers.MaxPooling2D()(x)
    x = tf.keras.layers.Conv2D(32,3,padding="same",activation="relu")(x)
    x = tf.keras.layers.GlobalAveragePooling2D()(x)
    z = tf.keras.layers.Concatenate()([x,st])
    # LSTM un paso
    h0 = tf.keras.layers.Dense(128,activation="tanh")(z)
    c0 = tf.keras.layers.Dense(128,activation="tanh")(z)
    lstm = tf.keras.layers.LSTMCell(128)
    h1,_ = lstm(w0,[h0,c0])
    delta = tf.keras.layers.Dense(3,activation="tanh")(h1)
    return tf.keras.Model([g,st,w0], delta, name="policy")

# ==============  NODO PRINCIPAL  ==========================================
class FlexPlanner(Node):
    def __init__(self):
        super().__init__("flexible_trainer")

        # --- Subscripciones
        qos=10
        self.create_subscription(Odometry,      "/odom",            self.cb_odom,     qos)
        self.create_subscription(PoseArray,     "/goal",            self.cb_goal,     qos)
        self.create_subscription(OccupancyGrid, "/occupancy_grid",  self.cb_grid,     qos)
        self.create_subscription(PoseArray,     "/safe_frontier_points_centroid",
                                 self.cb_frontier, qos)
        self.create_subscription(Bool,"/virtual_collision", self.cb_collision, qos)
        self.create_subscription(Bool,"/reset_confirmation",self.cb_reset_conf,qos)
        self.create_subscription(Bool,"/goal_reached",self.cb_goal_reached,qos)
        #self.create_subscription(PoseArray, "/obstacle_navigation_nodes_lidar",self.cb_obstacles, 10)

        # --- Publicadores
        self.path_pub  = self.create_publisher(Path,  "/global_path_predicted", qos)
        self.wps_pub   = self.create_publisher(Marker,"/path_waypoints_marker", qos)
        self.coll_pub  = self.create_publisher(Bool,  "/virtual_collision", qos)
        #self.goal_pub  = self.create_publisher(Bool,  "/goal_reached", qos)
        latched=QoSProfile(depth=1,
                           reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.reset_pub = self.create_publisher(Bool,  "/reset_request", latched)
        self.cmd_pub   = self.create_publisher(Twist, "/cmd_vel", qos)

        # --- Estado ROS
        self.waiting_reset=False
        self.current_path = []     # lista de waypoints activos
        self.wp_index     = 1      # índice del wp que se está siguiendo
        self.pose=self.twist=None
        self.goal=None
        self.grid_msg=None
        self.frontiers=[]
        self.obstacles = []  
        self.collided=False
        self.ready = True 
        self.reset_t0=None
        self.episode_done=False
        self.last_teacher = np.zeros(3, np.float32)  
        self.update_done=0
        self.max_seg  =0.6
        self.max_steps=60
        self.dev_thr  =0.8
        self.clear_min=0.4     # m
        self.max_seg_length      = 1.0
        self.reach_thr=0.4
        self.goal_reached_flag=False
        self.v_prev = 0.0               # ← último comando lineal publicado
        self.bc_weight = tf.Variable(LAMBDA_BC_START,
                                    trainable=False, dtype=tf.float32)

        self.stuck_steps = 0       # ← contador “sin progreso”


        self.ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

        # --- Red y PPO
        self.policy = build_policy()
        self.log_std = tf.Variable(np.log(STD_START*np.ones(3,np.float32)),
                                   trainable=True)
        
        self.opt_actor  = tf.keras.optimizers.Adam(LR_ACTOR)
        self.opt_critic = tf.keras.optimizers.Adam(LR_CRITIC)
        self.value_net  = tf.keras.Sequential([
            tf.keras.layers.Input(shape=(36,)),     # 32 (patch emb.) + 4 (state)
            tf.keras.layers.Dense(128,activation="tanh"),
            tf.keras.layers.Dense(1)
        ])

        # TensorBoard
        self.writer=tf.summary.create_file_writer(str(RUN_DIR/
                                                      time.strftime("run_%Y%m%d_%H%M%S")))
        # Buffers PPO
        self.reset_buffers()
        init_ckpt = RUN_DIR / "policy_lvl1_init.weights.h5"

        if init_ckpt.exists():
            # ── Carga directa ────────────────────────────
            self.policy.load_weights(init_ckpt)
            self.get_logger().info(f"Cargado checkpoint inicial: {init_ckpt.name}")


        else:
                try:
                    self.policy.load_weights(str(RUN_DIR/"policy_latest.weights.h5"),
                                            by_name=True, skip_mismatch=True)
                    self.get_logger().info("Pesos nivel-0 cargados (se ignora la capa de salida)")
                except Exception as e:
                    self.get_logger().warn(f"No se pudieron cargar pesos previos: {e}")

                demo_files = sorted(RUN_DIR.glob("demo_ep*.npz"))
                if demo_files:
                    patches=[]; states=[]; acts=[]
                    for f in demo_files:
                        d = np.load(f)
                        patches.append(d["patches"])
                        states.append(d["states"])
                        acts.append( np.hstack([d["actions"],
                                                d["v_norm"][...,None]]) )   # (Δx,Δy,v)

                    Xg = np.concatenate(patches,0).astype(np.float32)
                    Xs = np.concatenate(states ,0).astype(np.float32)
                    Ya = np.concatenate(acts   ,0).astype(np.float32)
                    w0 = np.zeros((Ya.shape[0],2), np.float32)

                    # congela todo salvo la densa final
                    for layer in self.policy.layers: layer.trainable = False
                    self.policy.layers[-1].trainable = True

                    self.policy.compile(optimizer="adam", loss="mse")
                    self.policy.fit([Xg,Xs,w0], Ya, epochs=1000, batch_size=256, verbose=1)

                    # vuelve a descongelar para PPO
                    for layer in self.policy.layers: layer.trainable = True
                self.policy.save_weights(RUN_DIR/"policy_lvl1_init.weights.h5")


        # Episodios
        self.episode=0
        self.goal_counter=0
        self.goals_in_world=random.randint(5,7)
        self.goal_vis=0

        self.create_timer(0.1, self.step)
        self.get_logger().info("Flexible planner + PPO listo")

    # ---------- Callbacks ROS ----------
    def cb_goal_reached(self, msg: Bool):
        if not msg.data:
            return
        # descarta si acabamos de reiniciar (<2 s) o hay colisión activa
        if (self.reset_t0 and
            (self.get_clock().now() - self.reset_t0).nanoseconds < 2e9):
            return
        if self.collided:
            return
        self.goal_reached_flag = True
    def cb_odom(self,m):
        self.pose=m.pose.pose; self.twist=m.twist.twist
    # def cb_obstacles(self, msg: PoseArray):
    #     self.obstacles = msg.poses

    def cb_goal(self,m):
        self.goal=m.poses[0] if m.poses else None
    # def cb_grid(self,m):
    #     self.grid_msg=m

    def cb_grid(self, m: OccupancyGrid):
        # mezclamos capa estática + móviles
        static_arr = np.array(m.data, dtype=np.int8).reshape((m.info.height,
                                                            m.info.width))
        if self.obstacles:
            dyn_arr = fuse_dynamic_layer(static_arr, self.obstacles, m.info)
            self.grid_dyn = dyn_arr                    # guarda dinámico
        else:
            self.grid_dyn = static_arr
        self.grid_msg = m                              # conserva original

    def cb_frontier(self,m):
        self.frontiers=[(p.position.x,p.position.y) for p in m.poses]
    def cb_collision(self,m):
        self.collided = bool(m.data)
    # def cb_reset_conf(self, msg:Bool):
    #     if msg.data:
    #         self.waiting_reset = False
    #         self.get_logger().info("[Reset] confirmado por el supervisor")
    #         self.goal_counter = 0
    #         self.goals_in_world = random.randint(5,7)
    #         self.collided = False
    #         self.reset_buffers()

    def cb_reset_conf(self, msg: Bool):
        if msg.data:
            self.waiting_reset = False
            self.ready         = False          # ← PAUSA hasta que el mundo cargue
            self.reset_t0      = self.get_clock().now()
            self.goal_counter  = 0
            self.goals_in_world = random.randint(5, 7)
            self.collided      = False
            self.reset_buffers()
            self.get_logger().info("[Reset] confirmado por el supervisor")

    def _behind_robot(self, cand, cp, yaw, max_back=0.1):
        """Devuelve True si cand queda detrás del robot más de max_back metros."""
        dx, dy = self._global_to_local(cand[0]-cp[0], cand[1]-cp[1], yaw)
        return dx < -max_back          # delante -> dx > 0
    
    def _candidate_ok(self, cand, cp, grid, info,
                    first_wp=False) -> bool:
        """Comprueba ocupación + línea + clearance (+restricciones iniciales)."""
        i, j = idx_from_world(info, cand)
        if not (0 <= i < grid.shape[1] and 0 <= j < grid.shape[0]):         return False
        if grid[j, i] == -1 or grid[j, i] >= 100:                           return False
        if not bres_free(grid, info, cp, cand):                             return False
        if not clearance_ok(grid, info, cand, CLEAR_MIN):                   return False
        if not safe_shadow(cand, self.obstacles):                           return False   # móviles

        # ── Sólo para el primer waypoint de la ruta ─────────────────
        if first_wp:
            # evita el “bache” si el tramo es demasiado corto
            if l2(cp, cand) < MIN_FIRST_SEG:
                return False

            # ángulo entre robot->cand y yaw actual
            dx, dy   = cand[0]-cp[0], cand[1]-cp[1]
            yaw      = self._yaw_from_quaternion(self.pose.orientation)
    
            alpha    = abs(math.atan2(dy, dx) - yaw)
            if alpha > math.pi: alpha = 2*math.pi - alpha

            # delante: α≤100°  → no aceptamos retroceso
            # detrás : α>100°  → se permite (la marcha atrás ya la gestionará la vel.)
            if alpha <= REV_ANGLE_MAX:
                return True
            # detrás: sólo lo aceptamos si realmente está en la zona trasera
            return True                    # sí, pero queda marcado como “reverse”

        # resto de waypoints → ya pasó los tests básicos
        return True

    # ════════════════════════════════════════════════════════════════
    #  ↓↓↓  PLANIFICADOR NIVEL-0  (teacher y fallback)  ↓↓↓
    # ════════════════════════════════════════════════════════════════
    def next_waypoint_lvl0(self, cp, tgt, grid, info):
        """
        Replica el abanico original: devuelve (cand, acción_teacher)
            action_teacher = [Δx_norm, Δy_norm, v_ref]   con v_ref ∈ [0,1]
        """
        ang0 = math.atan2(tgt[1]-cp[1], tgt[0]-cp[0])

        for r in RADII:
            for off in ANGLES:
                ang = ang0 + off
                cand = (cp[0] + r*math.cos(ang),
                        cp[1] + r*math.sin(ang))
                if not self._candidate_ok(cand, cp, grid, info, first_wp=False):
                    continue

                vec = np.array(cand) - np.array(cp)
                dist = np.linalg.norm(vec)
                vec  = vec / (dist + 1e-6)          # dirección normalizada
                v_ref = np.clip(dist / MAX_VEL, 0.0, 1.0)   # heurística simple
                return cand, np.hstack([vec, v_ref])

        # no hay waypoint viable
        return None, np.zeros(3, np.float32)
    # ════════════════════════════════════════════════════════════════


    def _yaw_from_quaternion(self, q):
        """Devuelve yaw (rad) desde geometry_msgs.msg.Quaternion."""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _roll_pitch_from_quaternion(self, q):
        """Devuelve (roll, pitch) en radianes."""
        sinr_cosp = 2*(q.w*q.x + q.y*q.z)
        cosr_cosp = 1 - 2*(q.x*q.x + q.y*q.y)
        roll  = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2*(q.w*q.y - q.z*q.x)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))   # clamp numérico
        return roll, pitch



    def _global_to_local(self, dx, dy, yaw):
        """Convierte ΔX,ΔY de frame 'map' a 'base_link'."""
        cos_y, sin_y = math.cos(-yaw), math.sin(-yaw)
        return dx*cos_y - dy*sin_y, dx*sin_y + dy*cos_y



    # ---------- Parche local -----------
    def extract_patch(self):
        info=self.grid_msg.info
        H,W=info.height,info.width
        # arr=np.array(self.grid_msg.data,dtype=np.int8).reshape((H,W))
        arr = self.grid_dyn.copy() # capa dinámica #nueva
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
        norm=((patch+1)/101.0).astype(np.float32)   # [-1,100] → [0,1]
        return np.expand_dims(norm,-1), arr, info

    # ---------- Target robusto ----------
    def choose_target(self, cp, grid, info):
        gp=(self.goal.position.x,self.goal.position.y)
        if bres_free(grid,info,cp,gp): self.goal_vis+=1
        else:                          self.goal_vis=0
        if self.goal_vis>=GOAL_VIS_OK:
            return gp,"GOAL"
        cand=[f for f in self.frontiers
              if bres_free(grid,info,cp,f) and clearance_ok(grid,info,f,CLEAR_MIN)]
        if cand:
            return min(cand,key=lambda f: l2(f,gp)),"FRONTIER"
        return None,"NONE"
    def bres_line_free(self,grid, info, a, b):
        def idx(p):
            return (int((p[0]-info.origin.position.x)/info.resolution),
                    int((p[1]-info.origin.position.y)/info.resolution))
        i0,j0 = idx(a); i1,j1 = idx(b)
        di,dj = abs(i1-i0), abs(j1-j0); si = 1 if i0<i1 else -1; sj = 1 if j0<j1 else -1
        err = di-dj; H,W = grid.shape
        while True:
            if not (0<=i0<W and 0<=j0<H) or grid[j0,i0] == -1 or grid[j0,i0] >= 100:
                return False
            if (i0,j0)==(i1,j1): return True
            e2=2*err
            if e2>-dj: err-=dj; i0+=si
            if e2< di: err+=di; j0+=sj


    def densify(self, path):
        out = [path[0]]
        for a, b in zip(path, path[1:]):
            d = distance(a, b)
            if d > self.max_seg_length:
                steps = int(math.ceil(d / self.max_seg_length))
                for i in range(1, steps):
                    t = i / steps
                    out.append((a[0]*(1-t)+b[0]*t, a[1]*(1-t)+b[1]*t))
            out.append(b)
        return out
    
    def generate_flexible_path(self, start, target, grid, info):
        raw = [start];  cp = start
        step_len = self.max_seg
        angles   = np.linspace(-math.pi/3, math.pi/3, 10)
        radii    = [step_len*0.5, step_len, step_len*1.5]

        for _ in range(self.max_steps):
            best = None; best_cost = float("inf")
            ang0 = math.atan2(target[1]-cp[1], target[0]-cp[0])

            for r in radii:
                for a in angles:
                    cand = (cp[0]+r*math.cos(ang0+a),
                            cp[1]+r*math.sin(ang0+a))
                    if not self._candidate_ok(cand, cp, grid, info, first_wp=(len(raw)==1)):
                        continue
                    cost = distance(cand, target) - 0.5*self.clear_min
                    if cost < best_cost:  best_cost, best = cost, cand

            if best is None:    break
            raw.append(best);   cp = best
            if distance(cp, target) < self.reach_thr: break

        raw.append(target)
        path = self.densify(raw)

        # ── Saca el 2º punto si el 1er tramo < MIN_FIRST_SEG ────────
        if len(path) >= 3 and l2(path[0], path[1]) < MIN_FIRST_SEG:
            path.pop(1)
        return path



####################################################antiguo
    # # ---------- Siguiente waypoint ----------
    # def next_waypoint(self, cp, tgt, grid, info, patch):
    #     # 1) Δ “preferido” de la policy
    #     state=np.array([0,0,tgt[0]-cp[0],tgt[1]-cp[1]],np.float32)
    #     patch_b = patch[None,...]
    #     state_b = state[None,:]
    #     delta=self.policy([patch_b, state_b, np.zeros((1,2),np.float32)],
    #                       training=False)[0].numpy()
    #     ang0=math.atan2(delta[1],delta[0]) if np.linalg.norm(delta)>1e-3 \
    #          else math.atan2(tgt[1]-cp[1], tgt[0]-cp[0])

    #     # 2) abanico de candidatos
    #     best=None; best_cost=float("inf")
    #     for r in RADII:
    #         for off in ANGLES:
    #             ang=ang0+off
    #             cand=(cp[0]+r*math.cos(ang), cp[1]+r*math.sin(ang))
    #             i,j=idx_from_world(info,cand)
    #             if not (0<=i<grid.shape[1] and 0<=j<grid.shape[0]): continue
    #             if grid[j,i]==-1 or grid[j,i]>=100: continue
    #             if not bres_free(grid,info,cp,cand):          continue
    #             if not clearance_ok(grid,info,cand,CLEAR_MIN):continue
    #             if not safe_shadow(cand, self.frontiers):     continue
    #             cost=l2(cand,tgt) - 0.5*CLEAR_MIN
    #             if cost<best_cost: best_cost, best = cost, cand
    #     return best, delta
    ##########################################
    # def next_waypoint(self, cp, tgt, grid, info, patch):
    #     # ── 1 · Paso por la política ─────────────────────────────
    #     state  = np.array([0, 0, tgt[0]-cp[0], tgt[1]-cp[1]], np.float32)
    #     out    = self.policy([patch[None, ...],
    #                         state[None, :],
    #                         np.zeros((1, 2), np.float32)], training=False)[0].numpy()
    #     delta     = out[:2]                        # dirección preferida
    #     vel_norm  = out[2]                         # −1 .. 1 → velocidad

    #     ang0 = math.atan2(delta[1], delta[0]) if np.linalg.norm(delta) > 1e-3 \
    #         else math.atan2(tgt[1]-cp[1], tgt[0]-cp[0])

    #     # ── 2 · Abanico de candidatos ────────────────────────────
    #     for r in RADII:
    #         for off in ANGLES:
    #             ang  = ang0 + off
    #             cand = (cp[0] + r*math.cos(ang), cp[1] + r*math.sin(ang))

    #             # Filtros: dentro de mapa, libre, línea, clearance, shadow
    #             i, j = idx_from_world(info, cand)
    #             if not (0 <= i < grid.shape[1] and 0 <= j < grid.shape[0]): continue
    #             if grid[j, i] == -1 or grid[j, i] >= 100: continue
    #             if not bres_free(grid, info, cp, cand):    continue
    #             if not clearance_ok(grid, info, cand, CLEAR_MIN): continue
    #             if not safe_shadow(cand, self.obstacles):  continue

    #             return cand, out     # ← primer candidato viable

    #     return None, out             # no se encontró waypoint

    def _is_front(self, pt, cp, yaw):
        """True si *pt* cae delante del robot en su frame local (+X)."""
        dx =  pt[0] - cp[0]
        dy =  pt[1] - cp[1]
        x_local, _ = self._global_to_local(dx, dy, yaw)
        return x_local >= 0.0

    def next_waypoint(self, cp, tgt, grid, info, patch):
        """
        Devuelve:
        · wp  – way-point elegido (o None)
        · out – salida completa de la red  (Δx,Δy,v_norm)
        """
        first_wp = (self.wp_index <= 2) 
        # ---------- inferencia de la política ------------------------------
        state = np.array([0, 0, tgt[0]-cp[0], tgt[1]-cp[1]], np.float32)
        out   = self.policy([patch[None,...],
                            state[None,:],
                            np.zeros((1,2), np.float32)], training=False)[0]
        dx, dy, v_norm = out.numpy()

        # -------------------------------------------------------------------
        yaw   = self._yaw_from_quaternion(self.pose.orientation)
        first = (self.wp_index <= 1)               # primer wp después del robot
        vec_t = np.array([tgt[0]-cp[0], tgt[1]-cp[1]], np.float32)
        vec_n = np.array([dx,          dy        ], np.float32)

        # --- 2.a · Corrige si la red apunta >90° lejos del objetivo --------
        if np.dot(vec_n, vec_t) < 0:               # ángulo > 90°
            vec_n = vec_t / (np.linalg.norm(vec_t)+1e-6)
            dx, dy = vec_n                         # forzamos dirección

        # ===================================================================
        # ①  CANDIDATO DIRECTO
        # ===================================================================
        if np.hypot(dx, dy) > 1e-3:
            cand = (cp[0] + RADII[1]*dx,
                    cp[1] + RADII[1]*dy)

            # ---- coherencia frente / retro -------------------------------
            front   = self._is_front(cand, cp, yaw)
            advance = v_norm >= 0.0
            if (front and advance) or (not front and not advance):

                if self._candidate_ok(cand, cp, grid, info, first):
                    return cand, out               # ✅ éxito

        # ===================================================================
        # ②  ABANICO CLÁSICO (seguridad nivel-0)
        # ===================================================================

        ang0 = math.atan2(dy, dx) if np.hypot(dx, dy) > 1e-3 else \
            math.atan2(tgt[1]-cp[1], tgt[0]-cp[0])

        for r in RADII:
            for off in ANGLES:
                cand = (cp[0] + r*math.cos(ang0+off),
                        cp[1] + r*math.sin(ang0+off))
                if self._candidate_ok(cand, cp, grid, info, first_wp):
                    return cand, out            # ← usamos la acción de la red

        # ── ③  Fallback definitivo: teacher nivel-0 ───────────────────
        cand, teacher_act = self.next_waypoint_lvl0(cp, tgt, grid, info)
        if cand is not None:
            # guardamos también la acción teacher para la pérdida BC
            self.last_teacher = teacher_act
            return cand, out                    # waypoint seguro, acción de la red

        self.last_teacher = np.zeros(3, np.float32)
        return None, out                        # sin solución



    # def follow_path(self, cp):
    #     # si no hay ruta o ya terminamos → quieto
    #     if len(self.current_path) <= self.wp_index:
    #         self.cmd_pub.publish(Twist())
    #         return

    #     wp = self.current_path[self.wp_index]

    #     # ¿hemos llegado a este wp?
    #     if l2(cp, wp) < LOOK_A:
    #         self.wp_index += 1
    #         if self.wp_index >= len(self.current_path):
    #             self.cmd_pub.publish(Twist())
    #             return
    #         wp = self.current_path[self.wp_index]

    #     dx, dy = wp[0] - cp[0], wp[1] - cp[1]
    #     d      = math.hypot(dx, dy)
    #     vx     = min(MAX_VEL, 1.5 * d) * dx / d
    #     vy     = min(MAX_VEL, 1.5 * d) * dy / d

    #     cmd = Twist()
    #     cmd.linear.x = vx
    #     cmd.linear.y = vy
    #     self.cmd_pub.publish(cmd)

    #     self.get_logger().info(
    #         f"[FOLLOW] wp {self.wp_index}/{len(self.current_path)-1} "
    #         f"→ v=({vx:.2f},{vy:.2f})")
###nuevo
    def _next_target_index(self, cp, look_ahead):
        """Devuelve el índice del primer waypoint a ≥ look_ahead del robot."""
        idx = self.wp_index
        while (idx + 1 < len(self.current_path)
            and l2(cp, self.current_path[idx]) < look_ahead):
            idx += 1
        return idx
########## antiguo###########################################
    # def follow_path(self, cp):
    #     if self.wp_index >= len(self.current_path):        # ruta terminada
    #         self.cmd_pub.publish(Twist()); return

    #     # 1. look-ahead adaptativo --------------------------------------------
    #     v_nom      = MAX_VEL                      # 3.0 m/s
    #     Ld         = max(0.25, 0.4 * v_nom)       # ~1.2 m
    #     self.wp_index = self._next_target_index(cp, Ld)
    #     tgt = self.current_path[self.wp_index]

    #     # 2. vector en mapa y en base_link ------------------------------------
    #     dx_g, dy_g = tgt[0]-cp[0], tgt[1]-cp[1]
    #     dist       = math.hypot(dx_g, dy_g)
    #     if dist < 1e-3:
    #         self.cmd_pub.publish(Twist()); return      # degenerado

    #     # orientación actual del robot
    #     q   = self.pose.orientation
    #     yaw = self._yaw_from_quaternion(q)
    #     dx, dy = self._global_to_local(dx_g, dy_g, yaw)

    #     # 3. ángulo α y curvatura κ ------------------------------------------
    #     alpha   = math.atan2(dy, dx)               # [-π, π]
    #     kappa   = 2.0 * math.sin(alpha) / Ld       # Pure-Pursuit

    #     # 4. velocidad lineal y giro -----------------------------------------
    #     k_gain  = 1.8
    #     v_lin   = max(MIN_VEL, min(MAX_VEL, k_gain*dist))
    #     omega   = kappa * v_lin                    # ω = κ·v

    #     cmd             = Twist()
    #     cmd.linear.x    =  v_lin
    #     cmd.angular.z   =  omega
    #     self.cmd_pub.publish(cmd)

    #     self.get_logger().info(
    #         f"[FOLLOW] wp={self.wp_index}/{len(self.current_path)-1}  "
    #         f"v={v_lin:.2f} m/s  ω={omega:.2f} rad/s  α={alpha*180/math.pi:+.1f}°")
############################################
    # def follow_path(self, cp, v_cmd):
    #     if self.wp_index >= len(self.current_path):
    #         self.cmd_pub.publish(Twist());  return

    #     # look-ahead proporcional a |v_cmd|
    #     look_h = max(0.25, 0.4 * abs(v_cmd))
    #     self.wp_index = self._next_target_index(cp, look_h)
    #     tgt = self.current_path[self.wp_index]

    #     dx_g, dy_g = tgt[0]-cp[0], tgt[1]-cp[1]
    #     dist = math.hypot(dx_g, dy_g)
    #     if dist < 1e-3:                         # degenerado
    #         self.cmd_pub.publish(Twist());  return

    #     yaw = self._yaw_from_quaternion(self.pose.orientation)
    #     dx, dy = self._global_to_local(dx_g, dy_g, yaw)

    #     alpha = math.atan2(dy, dx)
    #     Ld    = look_h
    #     kappa = 2.0 * math.sin(alpha) / Ld

    #     # velocidad lineal establecida por la red
    #     v_lin = float(np.clip(v_cmd * MAX_VEL, -MAX_VEL, MAX_VEL))
    #     if abs(v_lin) < 0.05:                  # “espera”
    #         self.cmd_pub.publish(Twist());  return

    #     omega = float(kappa * v_lin)

    #     cmd = Twist()
    #     cmd.linear.x  = v_lin
    #     cmd.angular.z = omega
    #     self.cmd_pub.publish(cmd)

    #     self.get_logger().info(
    #         f"[FOLLOW] wp={self.wp_index}/{len(self.current_path)-1}  "
    #         f"v={v_lin:.2f} m/s  ω={omega:.2f} rad/s  α={alpha*180/math.pi:+.1f}°")


    # ───────────────────────────────────────────────────────────────
    # 4 · Seguidor Pure–Pursuit con filtro de velocidad
    # ───────────────────────────────────────────────────────────────
    def follow_path(self, cp, v_norm):
        if self.wp_index >= len(self.current_path):
            self.cmd_pub.publish(Twist());  return

        # look-ahead proporcional a la velocidad deseada
        look_h = max(0.25, 0.4 * MAX_VEL * abs(v_norm))
        self.wp_index = self._next_target_index(cp, look_h)
        tgt = self.current_path[self.wp_index]

        dx_g, dy_g = tgt[0]-cp[0], tgt[1]-cp[1]
        dist       = math.hypot(dx_g, dy_g)
        if dist < 1e-3:  self.cmd_pub.publish(Twist()); return

        yaw = self._yaw_from_quaternion(self.pose.orientation)
        dx, dy = self._global_to_local(dx_g, dy_g, yaw)

        alpha = math.atan2(dy, dx)
        kappa = 2.0 * math.sin(alpha) / look_h

        # ── regla “retro solo cuando estamos de espaldas” ───────────
        if abs(alpha) < REV_ANGLE_MAX:
            v_norm = max(0.0, v_norm)       # sólo avance
        else:
            v_norm = min(0.0, v_norm)       # sólo retroceso

        v_cmd      = np.clip(v_norm, -1, 1) * MAX_VEL
        v_filt     = ALPHA_IIR_VEL*v_cmd + (1-ALPHA_IIR_VEL)*self.v_prev
        self.v_prev = v_filt

        #  clip de seguridad nivel-0
        if 0 < abs(v_filt) < MIN_VEL:             # evita “tirones” muy lentos
            v_filt = math.copysign(MIN_VEL, v_filt)

        cmd            = Twist()
        cmd.linear.x   = float(v_filt)
        cmd.angular.z  = float(kappa * v_filt)
        omega = cmd.angular.z
        self.cmd_pub.publish(cmd)
        self.get_logger().info(
                f"[FOLLOW] wp={self.wp_index}/{len(self.current_path)-1}  "
                f"v={v_filt:.2f} m/s  ω={omega:.2f} rad/s  α={alpha*180/math.pi:+.1f}°")



###nuevo

    def compute_reward(self, old_d, new_d, collided,
                    reached, step_len, overturned, alpha):
        r  = 2.0*(old_d - new_d)          # progreso
        r -= 0.05                         # castigo paso
        r -= 0.10*step_len                # longitud de tramo
        r -= 0.05*abs(alpha)              # ⬅️ nuevo: premiar ir recto
        if reached:     r += 200
        if collided:    r -= 200
        if overturned:  r -= 200
        return r


    # ---------- Buffers PPO -------------
    def reset_buffers(self):
        self.patch_buf=[]; self.state_buf=[]
        self.act_buf=[];   self.logp_buf=[]
        self.rew_buf=[];   self.val_buf=[]
        self.done_buf=[]; self.vel_buf=[]
        self.teacher_buf = []


    # ---------- Ciclo principal ---------
    # def step(self):
    #     if self.waiting_reset:
    #         return

    #     if None in (self.pose,self.goal,self.grid_msg): return
    #     cp=(self.pose.position.x,self.pose.position.y)
    #     patch,grid,info=self.extract_patch()

    #     tgt,mode=self.choose_target(cp,grid,info)
    #     if tgt is None:
    #         self.get_logger().warn("Sin target válido")
    #         return
    #     self.get_logger().info(f"[TARGET] mode={mode} -> {tgt}")

    #     wp, delta = self.next_waypoint(cp,tgt,grid,info,patch)
    #     if wp is None:
    #         self.get_logger().info("Sin waypoint; robot parado")
    #         self.publish_path([cp]); return

    #     step_len=l2(cp,wp)
    #     reached = l2(wp,tgt) < GOAL_RADIUS

    #     # Path & marker
    #     self.publish_path([cp,wp])

    #     # cmd_vel simple
    #     cmd=Twist()
    #     cmd.linear.x=(wp[0]-cp[0])*2.0
    #     cmd.angular.z=(wp[1]-cp[1])*2.0
    #     self.cmd_pub.publish(cmd)

    #     # --- reward & buffers
    #     old_d=l2(cp,tgt); new_d=l2(wp,tgt)
    #     reward=self.compute_reward(old_d,new_d,self.collided,reached,step_len)

    #     # value input (32 primeros px + state)
    #     state=np.array([0,0,tgt[0]-cp[0],tgt[1]-cp[1]],np.float32)
    #     state_vec=np.concatenate([patch.flatten()[:32],state])
    #     v_pred=self.value_net(state_vec[None,...])[0,0]

    #     std=np.exp(self.log_std.numpy())
    #     mu=np.zeros(2)          # placeholder; usamos delta como acción
    #     logp=-0.5*np.sum(((delta-mu)/std)**2 + 2*np.log(std)+np.log(2*np.pi))

    #     # buffers
    #     self.patch_buf.append(patch)
    #     self.state_buf.append(state)
    #     self.act_buf.append(delta)
    #     self.logp_buf.append(logp)
    #     self.rew_buf.append(reward)
    #     self.val_buf.append(v_pred)
    #     self.done_buf.append(reached or self.collided)

    #     # --- Termina episodio
    #     if reached:
    #         self.goal_pub.publish(Bool(data=True))
    #         self.goal_counter+=1
    #     if reached or self.collided:
    #         # resumen
    #         with self.writer.as_default():
    #             tf.summary.scalar("episode_reward",sum(self.rew_buf),step=self.episode)
    #             tf.summary.scalar("collided",int(self.collided),step=self.episode)
    #         # guarda pesos
    #         # fname=RUN_DIR/f"policy_ep{self.episode}.h5"
    #         timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    #         fname=RUN_DIR/f"policy_latest_{timestamp}.weights.h5"
    #         self.policy.save_weights(fname)
    #         self.get_logger().info(f"Pesos guardados en {fname}")
    #         self.episode+=1
    #         self.reset_buffers()

    #     # --- reset mundo si toca
    #     # if self.goal_counter>=self.goals_in_world or self.collided:
    #     #     self.reset_pub.publish(Bool(data=True))
    #     #     self.goal_counter=0
    #     #     self.goals_in_world=random.randint(5,7)
    #     #     self.get_logger().info("[RESET] nuevo mundo solicitado")
    #     if not self.waiting_reset and (self.goal_counter >= self.goals_in_world or self.collided):
    #         self.reset_pub.publish(Bool(data=True))
    #         self.waiting_reset = True
    #         self.get_logger().info("[Reset] peticion enviada, esperando confirmacion")

    #     # --- PPO update
    #     if len(self.act_buf) >= ROLLOUT_STEPS:
    #         self.update_ppo()

    # ────────────────────────────────────────────────────────────────
    #  STEP   (ciclo principal 10 Hz)
    # ────────────────────────────────────────────────────────────────
    def step(self):

        # 0 · Espera a que el mundo nuevo cargue ---------------------
        if not self.ready:
            cond_time = self.reset_t0 and \
                        (self.get_clock().now() - self.reset_t0).nanoseconds > 2e9
            cond_grid = self.grid_msg and \
                        np.any(np.array(self.grid_msg.data) != -1)
            if cond_time and self.pose and cond_grid:
                self.ready = True
                self.get_logger().info("[Reset] mundo cargado; reanudando")
            else:
                return

        # 1 · Datos mínimos -----------------------------------------
        if None in (self.pose, self.goal, self.grid_msg):
            return

        cp = (self.pose.position.x, self.pose.position.y)

        # 1.b · vuelco ----------------------------------------------
        roll, pitch = self._roll_pitch_from_quaternion(self.pose.orientation)
        overturned  = abs(roll) > MAX_TILT or abs(pitch) > MAX_TILT
        if overturned:
            self.collided = True

        # 2 · Patch + target robusto --------------------------------
        patch, grid, info = self.extract_patch()
        tgt, mode = self.choose_target(cp, grid, info)
        if tgt is None:
            self.get_logger().warn("Sin target válido")
            return

        # 3 · ¿Replanificamos? --------------------------------------
        need_replan = (
            not self.current_path or
            self.collided or
            l2(cp, self.current_path[min(2, len(self.current_path)-1)]) > 0.8
        )
        if need_replan:
            self.current_path = self.generate_flexible_path(cp, tgt, grid, info)
            self.wp_index     = 0
            self.get_logger().info(f"[PATH] len={len(self.current_path)} wps")

        # descarta tramo inicial demasiado corto
        MIN_FIRST_WP = 1.00
        while len(self.current_path) > 1 and \
            l2(cp, self.current_path[1]) < MIN_FIRST_WP:
            self.current_path.pop(1)

        # 4 · Selección del siguiente waypoint ----------------------
        wp, out = self.next_waypoint(cp, tgt, grid, info, patch)
        if wp is None:
            self.cmd_pub.publish(Twist())
            return

        dx, dy, vel_norm = out.numpy()
        self.current_path[self.wp_index] = wp       # coherencia

        if len(self.current_path) <= self.wp_index:
            self.cmd_pub.publish(Twist())
            return

        wp = self.current_path[self.wp_index]

        # 5 · ----- DETECTOR DE ESTANCAMIENTO -----------------------
        old_d = l2(cp, tgt)
        new_d = l2(wp, tgt)

        if new_d < old_d - 0.05:        # avanzó ≥ 5 cm
            self.stuck_steps = 0
        else:
            self.stuck_steps += 1

        if self.stuck_steps >= 25:      # ~ 2.5 s sin progreso
            self.current_path.clear()   # fuerza replanning
            self.stuck_steps = 0
            self.get_logger().warn("⛔  Sin progreso, replanteando")
            return                      # esperamos al próximo ciclo

        # 6 · Seguimiento + publicación -----------------------------
        self.follow_path(cp, vel_norm)
        self.publish_path(self.current_path)

        # 7 · Recompensa + buffers ----------------------------------
        reached  = self.goal_reached_flag
        collided = self.collided
        step_len = l2(cp, wp)

        yaw       = self._yaw_from_quaternion(self.pose.orientation)
        dxg, dyg  = wp[0]-cp[0], wp[1]-cp[1]
        dxl, dyl  = self._global_to_local(dxg, dyg, yaw)
        alpha     = math.atan2(dyl, dxl)

        reward = self.compute_reward(old_d, new_d,
                                    collided, reached,
                                    step_len, overturned, alpha)

        # ---- buffers -------------------------------------------------
        self.patch_buf.append(patch.astype(np.float32))
        self.state_buf.append(np.array([0,0,tgt[0]-cp[0],tgt[1]-cp[1]],
                                    np.float32))
        self.act_buf.append(out.numpy().astype(np.float32))
        self.teacher_buf.append(self.last_teacher.astype(np.float32))

        std  = np.exp(self.log_std.numpy())
        mu0  = np.zeros(3, np.float32)
        logp = -0.5*np.sum(((out.numpy()-mu0)/std)**2 + 2*np.log(std)
                        + np.log(2*np.pi))
        self.logp_buf.append(np.float32(logp))
        self.rew_buf.append(reward)
        self.val_buf.append(self.value_net(
            np.concatenate([patch.flatten()[:32],
                            self.state_buf[-1]])[None, ...])[0,0])
        self.done_buf.append(reached or collided)

        # 8 · Fin de episodio ---------------------------------------
        if reached or collided:
            self.cmd_pub.publish(Twist())
            if reached:
                self.get_logger().info("⛳  Goal alcanzado")
            if collided:
                self.get_logger().info("💥  Colisión detectada")
            if overturned:
                self.get_logger().warning("🚨  Robot volcado — reset")

            self.update_ppo()
            self.goal_reached_flag = False
            self.episode += 1
            self.goal_counter += int(reached and mode == "GOAL")

        # 9 · Reset de mundo ----------------------------------------
        if (self.goal_counter >= self.goals_in_world) or collided:
            self.reset_pub.publish(Bool(data=True))
            self.ready = False
            self.waiting_reset = True
            self.reset_t0 = self.get_clock().now()
            self.goal_counter = 0
            self.goals_in_world = random.randint(5,7)
            self.get_logger().info("[RESET] solicitado")

        if self.episode >= MAX_EPISODES:
            self.get_logger().info("Fin de entrenamiento")
            self.cmd_pub.publish(Twist())
            rclpy.shutdown()
    # ────────────────────────────────────────────────────────────────



    # ---------- Publicación RViz ----------
    def publish_path(self, pts):
        hdr = Header(frame_id="map",
                    stamp=self.get_clock().now().to_msg())

        # --- Path (linea) -------------------------------------------------
        path = Path(header=hdr)
        for x, y in pts:
            ps = PoseStamped(header=hdr)
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)

        # --- Waypoint markers  -------------------------------------------
        mk          = Marker(header=hdr, ns="wps", id=0,
                            type=Marker.POINTS, action=Marker.ADD)
        mk.scale    = Vector3(x=0.15, y=0.15, z=0.0)
        mk.color.r  = mk.color.g = 1.0
        mk.color.a  = 1.0
        mk.points   = []
        for x, y in pts[1:]:
            p = Point()
            p.x = float(x)
            p.y = float(y)
            p.z = 0.0
            mk.points.append(p)

        self.wps_pub.publish(mk)

    # ────────────────────────────────────────────────────────────────
    #  UPDATE_PPO   (con Behaviour-Cloning)
    # ────────────────────────────────────────────────────────────────
    def update_ppo(self):

        # 1 · returns & ventajas -------------------------------------
        returns, advs = [], []
        gae = 0.0; next_val = 0.0
        for r, v, d in zip(reversed(self.rew_buf),
                        reversed(self.val_buf),
                        reversed(self.done_buf)):
            delta = r + GAMMA*next_val*(1-d) - v
            gae   = delta + GAMMA*GAE_LAMBDA*(1-d)*gae
            advs.insert(0, gae)
            next_val = v

        returns = np.array(advs) + np.array(self.val_buf)
        advs    = (np.array(advs) - np.mean(advs)) / (np.std(advs)+1e-8)

        # 2 · Dataset -------------------------------------------------
        ds = tf.data.Dataset.from_tensor_slices(
            (np.stack(self.patch_buf),
            np.stack(self.state_buf),
            np.stack(self.act_buf),
            np.stack(self.teacher_buf),           # ← NUEVO
            np.array(self.logp_buf, np.float32),
            advs.astype(np.float32),
            returns.astype(np.float32))
        ).shuffle(4096).batch(BATCH_SZ)

        # 3 · Optimización -------------------------------------------
        for _ in range(EPOCHS):
            for g, st, act, act_tch, lp_old, adv, ret in ds:
                with tf.GradientTape() as tpi, tf.GradientTape() as tpv:
                    bs   = tf.shape(act)[0]
                    w0   = tf.zeros((bs,2), tf.float32)

                    mu   = self.policy([g, st, w0], training=True)
                    std  = tf.exp(self.log_std)

                    lp = -0.5*tf.reduce_sum(((act-mu)/std)**2
                                            + 2*tf.math.log(std)
                                            + tf.math.log(2*np.pi), axis=-1)
                    ratio = tf.exp(lp - lp_old)

                    ppo_loss = -tf.reduce_mean(
                        tf.minimum(ratio*adv,
                                tf.clip_by_value(ratio,
                                                    1-CLIP_EPS,1+CLIP_EPS)*adv))

                    bc_loss = tf.reduce_mean(tf.square(mu - act_tch))
                    actor_loss = ppo_loss + self.bc_weight * bc_loss

                    # crítico
                    x_flat = tf.reshape(g, (-1, PATCH*PATCH))[:, :32]
                    v_pred = self.value_net(tf.concat([x_flat, st], -1),
                                            training=True)[:,0]
                    critic_loss = tf.reduce_mean((ret - v_pred)**2)

                # aplica gradientes
                self.opt_actor.apply_gradients(
                    zip(tpi.gradient(actor_loss,
                        self.policy.trainable_variables + [self.log_std]),
                        self.policy.trainable_variables + [self.log_std]))
                self.opt_critic.apply_gradients(
                    zip(tpv.gradient(critic_loss,
                        self.value_net.trainable_variables),
                        self.value_net.trainable_variables))

        # 4 · annealing σ --------------------------------------------
        new_std = tf.maximum(tf.exp(self.log_std)*STD_DECAY, STD_MIN)
        self.log_std.assign(tf.math.log(new_std))

        # 5 · decay Behaviour-Cloning --------------------------------
        new_bc = max(LAMBDA_BC_END,
                    float(self.bc_weight) -
                    (LAMBDA_BC_START-LAMBDA_BC_END)/BC_DECAY_STEPS)
        self.bc_weight.assign(new_bc)

        # 6 · logs ----------------------------------------------------
        with self.writer.as_default():
            tf.summary.scalar("loss_actor",   actor_loss, step=self.episode)
            tf.summary.scalar("loss_critic",  critic_loss, step=self.episode)
            tf.summary.scalar("policy_std",   float(new_std[0]),
                            step=self.episode)
            tf.summary.scalar("lambda_bc",    float(self.bc_weight),
                            step=self.episode)

        # 7 · guarda checkpoint --------------------------------------
        self.policy.save_weights(RUN_DIR / f"policy_latest_new{self.ts}.weights.h5")

        # 8 · limpia buffers -----------------------------------------
        self.reset_buffers()
        self.get_logger().info(
            f"[PPO] π={actor_loss.numpy():.3f}  V={critic_loss.numpy():.3f}  "
            f"σ={float(new_std[0]):.2f}  λBC={float(self.bc_weight):.2f}")
    # ────────────────────────────────────────────────────────────────

# ==============  MAIN  ====================================================
def main(args=None):
    rclpy.init(args=args)
    node=FlexPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node(); rclpy.shutdown()

if __name__=="__main__":
    main()
