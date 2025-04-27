
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
GOAL_RADIUS     = 1.0                       # m para “goal reached”
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
EPOCHS          = 4
GAMMA           = 0.99
GAE_LAMBDA      = 0.95
CLIP_EPS        = 0.2
LR_ACTOR        = 3e-4
LR_CRITIC       = 1e-3
STD_START       = 0.3
STD_MIN         = 0.05
STD_DECAY       = 0.995

DTYPE = np.float32


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
    delta = tf.keras.layers.Dense(2,activation="tanh")(h1)
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

        # --- Publicadores
        self.path_pub  = self.create_publisher(Path,  "/global_path_predicted", qos)
        self.wps_pub   = self.create_publisher(Marker,"/path_waypoints_marker", qos)
        self.coll_pub  = self.create_publisher(Bool,  "/virtual_collision", qos)
        self.goal_pub  = self.create_publisher(Bool,  "/goal_reached", qos)
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
        self.collided=False
        self.max_seg  =0.6
        self.max_steps=60
        self.dev_thr  =0.8
        self.clear_min=0.4     # m
        self.max_seg_length      = 1.0
        self.reach_thr=0.4

        # --- Red y PPO
        self.policy = build_policy()
        self.log_std = tf.Variable(np.log(STD_START*np.ones(2,np.float32)),
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

        # Episodios
        self.episode=0
        self.goal_counter=0
        self.goals_in_world=random.randint(5,7)
        self.goal_vis=0

        self.create_timer(0.1, self.step)
        self.get_logger().info("Flexible planner + PPO listo")

    # ---------- Callbacks ROS ----------
    def cb_odom(self,m):
        self.pose=m.pose.pose; self.twist=m.twist.twist
    def cb_goal(self,m):
        self.goal=m.poses[0] if m.poses else None
    def cb_grid(self,m):
        self.grid_msg=m
    def cb_frontier(self,m):
        self.frontiers=[(p.position.x,p.position.y) for p in m.poses]
    def cb_collision(self,m):
        self.collided = bool(m.data)
    def cb_reset_conf(self, msg:Bool):
        if msg.data:
            self.waiting_reset = False
            self.get_logger().info("[Reset] confirmado por el supervisor")
            self.goal_counter = 0
            self.goals_in_world = random.randint(5,7)
            self.collided = False
            self.reset_buffers()


    def _yaw_from_quaternion(self, q):
        """Devuelve yaw (rad) desde geometry_msgs.msg.Quaternion."""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _global_to_local(self, dx, dy, yaw):
        """Convierte ΔX,ΔY de frame 'map' a 'base_link'."""
        cos_y, sin_y = math.cos(-yaw), math.sin(-yaw)
        return dx*cos_y - dy*sin_y, dx*sin_y + dy*cos_y



    # ---------- Parche local -----------
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
        path=[start]
        cp=start
        step_len=self.max_seg
        angles=np.linspace(-math.pi/3, math.pi/3, 10)   # ±60°
        radii=[step_len*0.5, step_len, step_len*1.5]

        for _ in range(self.max_steps):
            best=None; best_cost=float("inf")
            vec_t=(target[0]-cp[0], target[1]-cp[1])
            ang0=math.atan2(vec_t[1], vec_t[0])

            for r in radii:
                for a_off in angles:
                    ang=ang0 + a_off
                    cand=(cp[0]+r*math.cos(ang), cp[1]+r*math.sin(ang))

                    # ---------- filtro ③  “celda debe ser conocida y libre” ----------
                    i = int((cand[0] - info.origin.position.x) / info.resolution)
                    j = int((cand[1] - info.origin.position.y) / info.resolution)
                    if not (0 <= i < grid.shape[1] and 0 <= j < grid.shape[0]):
                        continue                             # fuera de mapa
                    if grid[j, i] == -1 or grid[j, i] >= 100:
                        continue                             # desconocida u obstáculo
                    # -----------------------------------------------------------------

                    if not self.bres_line_free(grid, info, cp, cand):
                        continue
                    if not clearance_ok(grid, info, cand, self.clear_min):
                        continue

                    cost = distance(cand, target) - 0.5*self.clear_min
                    if cost < best_cost:
                        best_cost, best = cost, cand

            if best is None:
                break
            path.append(best)
            cp = best
            if distance(cp, target) < self.reach_thr:
                break

        path.append(target)
        return self.densify(path)       




    # ---------- Siguiente waypoint ----------
    def next_waypoint(self, cp, tgt, grid, info, patch):
        # 1) Δ “preferido” de la policy
        state=np.array([0,0,tgt[0]-cp[0],tgt[1]-cp[1]],np.float32)
        patch_b = patch[None,...]
        state_b = state[None,:]
        delta=self.policy([patch_b, state_b, np.zeros((1,2),np.float32)],
                          training=False)[0].numpy()
        ang0=math.atan2(delta[1],delta[0]) if np.linalg.norm(delta)>1e-3 \
             else math.atan2(tgt[1]-cp[1], tgt[0]-cp[0])

        # 2) abanico de candidatos
        best=None; best_cost=float("inf")
        for r in RADII:
            for off in ANGLES:
                ang=ang0+off
                cand=(cp[0]+r*math.cos(ang), cp[1]+r*math.sin(ang))
                i,j=idx_from_world(info,cand)
                if not (0<=i<grid.shape[1] and 0<=j<grid.shape[0]): continue
                if grid[j,i]==-1 or grid[j,i]>=100: continue
                if not bres_free(grid,info,cp,cand):          continue
                if not clearance_ok(grid,info,cand,CLEAR_MIN):continue
                cost=l2(cand,tgt) - 0.5*CLEAR_MIN
                if cost<best_cost: best_cost, best = cost, cand
        return best, delta
    

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

    def follow_path(self, cp):
        if self.wp_index >= len(self.current_path):        # ruta terminada
            self.cmd_pub.publish(Twist()); return

        # 1. look-ahead adaptativo --------------------------------------------
        v_nom      = MAX_VEL                      # 3.0 m/s
        Ld         = max(0.25, 0.4 * v_nom)       # ~1.2 m
        self.wp_index = self._next_target_index(cp, Ld)
        tgt = self.current_path[self.wp_index]

        # 2. vector en mapa y en base_link ------------------------------------
        dx_g, dy_g = tgt[0]-cp[0], tgt[1]-cp[1]
        dist       = math.hypot(dx_g, dy_g)
        if dist < 1e-3:
            self.cmd_pub.publish(Twist()); return      # degenerado

        # orientación actual del robot
        q   = self.pose.orientation
        yaw = self._yaw_from_quaternion(q)
        dx, dy = self._global_to_local(dx_g, dy_g, yaw)

        # 3. ángulo α y curvatura κ ------------------------------------------
        alpha   = math.atan2(dy, dx)               # [-π, π]
        kappa   = 2.0 * math.sin(alpha) / Ld       # Pure-Pursuit

        # 4. velocidad lineal y giro -----------------------------------------
        k_gain  = 1.8
        v_lin   = max(MIN_VEL, min(MAX_VEL, k_gain*dist))
        omega   = kappa * v_lin                    # ω = κ·v

        cmd             = Twist()
        cmd.linear.x    =  v_lin
        cmd.angular.z   =  omega
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f"[FOLLOW] wp={self.wp_index}/{len(self.current_path)-1}  "
            f"v={v_lin:.2f} m/s  ω={omega:.2f} rad/s  α={alpha*180/math.pi:+.1f}°")


###nuevo
    # ---------- Recompensa --------------
    def compute_reward(self, old_d, new_d, collided, reached, step_len):
        r= 2.0*(old_d-new_d)                # progreso
        r-= 0.05                            # castigo paso
        r-= 0.1*step_len                    # castigo ruta larga
        if reached:  r+= 200
        if collided: r-= 200
        return r

    # ---------- Buffers PPO -------------
    def reset_buffers(self):
        self.patch_buf=[]; self.state_buf=[]
        self.act_buf=[];   self.logp_buf=[]
        self.rew_buf=[];   self.val_buf=[]
        self.done_buf=[]

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

    def step(self):
        # ───────── Pausa si estamos esperando un reset ─────────
        if self.waiting_reset:
            return

        # ───────── Validación de entradas ROS ─────────
        if None in (self.pose, self.goal, self.grid_msg):
            return

        cp = (self.pose.position.x, self.pose.position.y)

        # Parche local y grid global
        patch, grid, info = self.extract_patch()

        # ───── 1. SELECCIÓN (robusta) DEL TARGET ─────
        tgt, mode = self.choose_target(cp, grid, info)
        if tgt is None:
            self.get_logger().warn("Sin target válido")
            return
        self.get_logger().info(f"[TARGET] mode={mode}  ->  {tgt}")

        # ───── 2. ¿Necesitamos replanificar? ─────
        need_replan = (
            not self.current_path                                   # no hay ruta
            or self.collided                                        # choque
            or l2(cp, self.current_path[min(2, len(self.current_path)-1)]) > 0.8
        )

        if need_replan:
            # genera tramo completo flexible
            self.current_path = self.generate_flexible_path(cp, tgt, grid, info)
            self.wp_index = 1                                       # primer wp real
            self.get_logger().info(f"[PATH] len={len(self.current_path)} wps")

        # # ───── 3. CONSUMIR WAYPOINTS ALCANZADOS ─────
        # LOOK_A = 0.4  # radio de aceptación
        # while (len(self.current_path) > self.wp_index + 1 and
        #     l2(cp, self.current_path[self.wp_index]) < LOOK_A):
        #     self.wp_index += 1

        # # Si ya no quedan waypoints, detener y salir
        # if len(self.current_path) <= self.wp_index:
        #     self.cmd_pub.publish(Twist())           # stop
        #     return

        # # ───── 4. CONTROLADOR PURE-PURSUIT ─────
        wp = self.current_path[self.wp_index]
        dx, dy = wp[0] - cp[0], wp[1] - cp[1]
        d      = math.hypot(dx, dy)
        # MAX_V  = 4.0  # m/s
        # vx     = min(MAX_V, 1.5 * d) * dx / d
        # vy     = min(MAX_V, 1.5 * d) * dy / d
        # cmd = Twist()
        # cmd.linear.x = -vx
        # cmd.angular.z = -vy
        # self.cmd_pub.publish(cmd)
        # self.get_logger().info(
        #     f"[FOLLOW] wp {self.wp_index}/{len(self.current_path)-1} "
        #     f"→ v=({vx:.2f},{vy:.2f})")
        #nuevo
        # ── AQUÍ ES DONDE LLAMAMOS AL SEGUIDOR ────────────────────────────────
        self.follow_path(cp)    
        # Publica Path y Marker (para RViz)
        self.publish_path(self.current_path)

        # ───── 5. RECOMPENSA Y BUFFERS PPO ─────
        reached = l2(wp, tgt) < GOAL_RADIUS
        step_len = d
        reward = self.compute_reward(
            old_d=l2(cp, tgt),
            new_d=l2(wp, tgt),
            collided=self.collided,
            reached=reached,
            step_len=step_len
        )

        state_vec = np.concatenate([patch.flatten()[:32],
                                    np.array([0, 0, tgt[0]-cp[0], tgt[1]-cp[1]])])
        v_pred = self.value_net(state_vec[None, ...])[0, 0]
        std = np.exp(self.log_std.numpy().astype(DTYPE))
        mu  = np.zeros(2,DTYPE)
        logp = -0.5 * np.sum(
            ((np.zeros(2) - mu) / std) ** 2 + 2 * np.log(std) + np.log(2 * np.pi)
        )

        # guarda en buffers
        self.patch_buf.append(patch)
        self.state_buf.append(state_vec[-4:])
        # self.act_buf.append(np.zeros(2))       # acción ficticia; usamos ruta
        self.act_buf.append(np.zeros(2,DTYPE))
        self.logp_buf.append(np.float32(logp))
        self.rew_buf.append(reward)
        self.val_buf.append(v_pred)
        self.done_buf.append(reached or self.collided)


        # ───── 6. FINAL DE EPISODIO ─────
        if reached or self.collided:
            with self.writer.as_default():
                tf.summary.scalar("episode_reward", sum(self.rew_buf), step=self.episode)
                tf.summary.scalar("collided",       int(self.collided), step=self.episode)
            self.episode += 1

            
            # Solo reinicia contadores de meta:
            if reached:
                self.goal_counter += 1

        # ───── 7. ¿PEDIMOS RESET DE MAPA? ─────
        if (self.goal_counter >= self.goals_in_world) or self.collided:
            # 7.1 ENTRENAR con TODO lo acumulado en este mapa
            # if len(self.act_buf) >= 32:        # opcional seguridad
            self.update_ppo()              # ← actualiza CNN+LSTM+σ
            # else:
            #     self.reset_buffers()           # lote demasiado pequeño

            # 7.2 Guardar pesos actualizados
            ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            self.policy.save_weights(RUN_DIR / f"policy_latest_{ts}.weights.h5")

            # 7.3 Solicitar nuevo mundo
            self.reset_pub.publish(Bool(data=True))
            self.waiting_reset = True
            self.goal_counter  = 0
            self.goals_in_world = random.randint(5, 7)
            self.get_logger().info("[RESET] petición enviada; esperando confirmación")


    # ---------- Publicación RViz ----------
    def publish_path(self,pts):
        hdr=Header(frame_id="map",
                   stamp=self.get_clock().now().to_msg())
        path=Path(header=hdr)
        for x,y in pts:
            ps=PoseStamped(header=hdr)
            ps.pose.position.x=x; ps.pose.position.y=y
            ps.pose.orientation.w=1.0
            path.poses.append(ps)
        self.path_pub.publish(path)

        mk=Marker(header=hdr,ns="wps",id=0,
                  type=Marker.POINTS,action=Marker.ADD)
        mk.scale=Vector3(x=0.15,y=0.15,z=0.0)
        mk.color.r=mk.color.g=1.0; mk.color.a=1.0
        mk.points=[Point(x=x,y=y) for x,y in pts[1:]]
        self.wps_pub.publish(mk)

    # ---------- PPO UPDATE completo ----------
    def update_ppo(self):
        # 1) Returns y ventajas
        returns, advs = [], []
        gae=0.0; next_val=0.0
        for r,v,d in zip(reversed(self.rew_buf),
                         reversed(self.val_buf),
                         reversed(self.done_buf)):
            delta = r + GAMMA*next_val*(1-d) - v
            gae   = delta + GAMMA*GAE_LAMBDA*(1-d)*gae
            advs.insert(0,gae)
            next_val = v
        returns = np.array(advs) + np.array(self.val_buf)
        advs = (np.array(advs)-np.mean(advs))/(np.std(advs)+1e-8)

        # 2) DataSet
        ds=tf.data.Dataset.from_tensor_slices(
            (np.stack(self.patch_buf).astype(np.float32),
             np.stack(self.state_buf).astype(np.float32),
             np.stack(self.act_buf).astype(np.float32),
             np.array(self.logp_buf,np.float32),
             advs.astype(np.float32),
             returns.astype(np.float32))
        ).shuffle(4096).batch(BATCH_SZ)

        # 3) Optimización
        for _ in range(EPOCHS):
            for g,st,act,lp_old,adv,ret in ds:
                with tf.GradientTape() as tpi, tf.GradientTape() as tpv:
                    mu=self.policy([g,st,tf.zeros_like(act)],training=True)
                    std=tf.exp(self.log_std)
                    lp=-0.5*tf.reduce_sum(((act-mu)/std)**2
                                          + 2*tf.math.log(std)
                                          + tf.math.log(2*np.pi),axis=-1)
                    ratio=tf.exp(lp-lp_old)
                    pg_loss=-tf.reduce_mean(
                        tf.minimum(ratio*adv,
                                   tf.clip_by_value(ratio,
                                                    1-CLIP_EPS,1+CLIP_EPS)*adv))

                    state_vec=tf.concat([tf.reshape(g,(-1,PATCH*PATCH))[:,:32],st],axis=-1)
                    v=tf.squeeze(self.value_net(state_vec,training=True),axis=-1)
                    v_loss=tf.reduce_mean((ret-v)**2)

                self.opt_actor.apply_gradients(
                    zip(tpi.gradient(pg_loss,
                        self.policy.trainable_variables+[self.log_std]),
                        self.policy.trainable_variables+[self.log_std]))
                self.opt_critic.apply_gradients(
                    zip(tpv.gradient(v_loss,self.value_net.trainable_variables),
                        self.value_net.trainable_variables))

        # 4) Annealing σ
        new_std=tf.maximum(tf.exp(self.log_std)*STD_DECAY, STD_MIN)
        self.log_std.assign(tf.math.log(new_std))

        # 5) Logs TensorBoard
        with self.writer.as_default():
            tf.summary.scalar("loss_actor",pg_loss,step=self.episode)
            tf.summary.scalar("loss_critic",v_loss,step=self.episode)
            tf.summary.scalar("policy_std",float(new_std[0]),step=self.episode)

        self.reset_buffers()
        self.get_logger().info(
            f"[PPO] update  π={pg_loss.numpy():.3f}  V={v_loss.numpy():.3f}  σ={float(new_std[0]):.2f}")

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
