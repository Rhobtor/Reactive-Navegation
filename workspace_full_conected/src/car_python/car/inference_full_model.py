#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
flexible_planner_inference.py
─────────────────────────────
Nodo ROS2 para inferencia online con la política Nivel‑1 entrenada.
Carga pesos de BC+PPO y publica cmd_vel y path en RViz.
"""
import math
import numpy as np
import tensorflow as tf
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import PoseArray, PoseStamped, Twist, Point, Vector3
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, Bool

# Parámetros idénticos a nivel-1
PATCH = 128
RADII = [2.3, 3.6, 5.9]
ANGLES = np.linspace(-math.pi/3, math.pi/3, 11)
CLEAR_MIN = 3.4       # m
REV_ANGLE_MAX = math.radians(100)
MAX_VEL = 6.0         # m/s
MIN_VEL = 1.0         # m/s
ALPHA_IIR_VEL = 0.6
MAX_TILT = 0.5        # rad

DTYPE = np.float32

# ====================== UTILS GEOMÉTRICOS ======================
def l2(a, b):
    return math.hypot(b[0]-a[0], b[1]-a[1])

def idx_from_world(info, pt):
    res = info.resolution
    return (int((pt[0]-info.origin.position.x)/res),
            int((pt[1]-info.origin.position.y)/res))

def bres_free(grid, info, a, b):
    i0,j0 = idx_from_world(info,a)
    i1,j1 = idx_from_world(info,b)
    di,dj = abs(i1-i0), abs(j1-j0)
    si = 1 if i0<i1 else -1
    sj = 1 if j0<j1 else -1
    err = di-dj
    H,W = grid.shape
    while True:
        if not (0<=i0<W and 0<=j0<H): return False
        v = grid[j0,i0]
        if v==-1 or v>=100:          return False
        if (i0,j0)==(i1,j1):         return True
        e2 = 2*err
        if e2>-dj: err-=dj; i0+=si
        if e2< di: err+=di; j0+=sj

    return False

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

def safe_shadow(cand, obstacles, tau=1.0):
    cx, cy = cand
    for obs in obstacles:
        ox, oy = obs.position.x, obs.position.y
        vx, vy = obs.orientation.x, obs.orientation.y
        r      = obs.orientation.w
        relx, rely = ox - cx, oy - cy
        tc = -(relx*vx + rely*vy) / (vx*vx + vy*vy + 1e-3)
        tc = max(0.0, tc)
        dx, dy = relx + vx*tc, rely + vy*tc
        if tc < tau and math.hypot(dx, dy) < r + 0.3:
            return False
    return True

# ====================== RED CNN + LSTM ==========================
def build_policy():
    g   = tf.keras.Input(shape=(PATCH,PATCH,1), name="grid")
    st  = tf.keras.Input(shape=(4,),            name="state")
    w0  = tf.keras.Input(shape=(2,),            name="wp0")
    x = tf.keras.layers.Conv2D(16,3,padding="same",activation="relu")(g)
    x = tf.keras.layers.MaxPooling2D()(x)
    x = tf.keras.layers.Conv2D(32,3,padding="same",activation="relu")(x)
    x = tf.keras.layers.GlobalAveragePooling2D()(x)
    z = tf.keras.layers.Concatenate()([x,st])
    h0 = tf.keras.layers.Dense(128,activation="tanh")(z)
    c0 = tf.keras.layers.Dense(128,activation="tanh")(z)
    lstm = tf.keras.layers.LSTMCell(128)
    h1,_ = lstm(w0,[h0,c0])
    delta = tf.keras.layers.Dense(3,activation="tanh")(h1)
    return tf.keras.Model([g,st,w0], delta, name="policy_lvl1")

# ====================== NODO DE INFERENCIA ======================
class FlexPlannerInference(Node):
    def __init__(self):
        super().__init__("flexible_planner_inference")
        qos = 10
        self.create_subscription(Odometry, "/odom", self.cb_odom, qos)
        self.create_subscription(OccupancyGrid, "/occupancy_grid", self.cb_grid, qos)
        self.create_subscription(PoseArray, "/goal", self.cb_goal, qos)
        self.create_subscription(PoseArray, "/safe_frontier_points_centroid", self.cb_frontier, qos)
        self.create_subscription(PoseArray, "/obstacle_navigation_nodes_lidar", self.cb_obstacles, qos)

        self.cmd_pub   = self.create_publisher(Twist, "/cmd_vel", qos)
        self.path_pub  = self.create_publisher(Path,  "/global_path_predicted", qos)
        self.wps_pub   = self.create_publisher(Marker, "/path_waypoints_marker", qos)

        # Estado
        self.pose = None
        self.grid_msg = None
        self.goal = None
        self.frontiers = []
        self.obstacles = []

        # Política
        self.policy = build_policy()
        weights_path = "/home/rhobtor/reactive/Reactive-Navegation/workspace_full_conected/weights/policy_latest_full_new.weights.h5"
        self.policy.load_weights(weights_path)
        self.log_std = tf.Variable(np.log(0.3*np.ones(3, np.float32)))

        # Parámetros de seguimiento
        self.current_path = []
        self.wp_index = 0
        self.v_prev = 0.0

        # Timer de ciclo
        self.create_timer(0.1, self.step)
        self.get_logger().info("Nivel-1 Inference listo")

    # -------------- Callbacks ROS ----------------
    def cb_odom(self, msg: Odometry):
        self.pose = msg.pose.pose
    def cb_grid(self, msg: OccupancyGrid):
        arr = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        self.grid_msg = msg; self.grid_dyn = arr
    def cb_goal(self, msg: PoseArray):
        self.goal = msg.poses[0] if msg.poses else None
    def cb_frontier(self, msg: PoseArray):
        self.frontiers = [(p.position.x,p.position.y) for p in msg.poses]
    def cb_obstacles(self, msg: PoseArray):
        self.obstacles = msg.poses

    # -------------- Cinemática y utilidades -------------
    def _yaw_from_quaternion(self, q):
        siny = 2.0*(q.w*q.z + q.x*q.y)
        cosy = 1.0 - 2.0*(q.y*q.y + q.z*q.z)
        return math.atan2(siny, cosy)
    def _global_to_local(self, dx, dy, yaw):
        cos_y, sin_y = math.cos(-yaw), math.sin(-yaw)
        return dx*cos_y - dy*sin_y, dx*sin_y + dy*cos_y
    def _is_front(self, pt, cp, yaw):
        dx, dy = pt[0]-cp[0], pt[1]-cp[1]
        x_local,_ = self._global_to_local(dx, dy, yaw)
        return x_local >= 0.0

    # ------------ Extracción de parche --------------
    def extract_patch(self):
        info = self.grid_msg.info
        arr = self.grid_dyn.copy()
        cp = (self.pose.position.x, self.pose.position.y)
        ci = int((cp[0]-info.origin.position.x)/info.resolution)
        cj = int((cp[1]-info.origin.position.y)/info.resolution)
        half = PATCH//2
        i0,i1 = max(ci-half,0), min(ci+half, info.width)
        j0,j1 = max(cj-half,0), min(cj+half, info.height)
        patch = arr[j0:j1, i0:i1]
        pad = ((j0-(cj-half), (cj+half)-j1), (i0-(ci-half), (ci+half)-i1))
        patch = np.pad(patch, pad, 'constant', constant_values=-1)
        norm = ((patch+1)/101.0).astype(np.float32)
        return norm[...,None], arr, info

    # -------------- Filtro candidato ------------
    def _candidate_ok(self, cand, cp, grid, info, first_wp=False):
        i,j = idx_from_world(info,cand)
        if not (0<=i<grid.shape[1] and 0<=j<grid.shape[0]): return False
        if grid[j,i]==-1 or grid[j,i]>=100: return False
        if not bres_free(grid, info, cp, cand): return False
        if not clearance_ok(grid, info, cand, CLEAR_MIN): return False
        if not safe_shadow(cand, self.obstacles): return False
        if first_wp:
            # evitar tramo inicial demasiado corto
            if l2(cp,cand) < 0.60: return False
            alpha = abs(math.atan2(cand[1]-cp[1],cand[0]-cp[0]) - self._yaw_from_quaternion(self.pose.orientation))
            if alpha>math.pi: alpha=2*math.pi-alpha
            # dejamos pasar
        return True

    # -------------- Inferencia siguiente wp -------
    def next_waypoint(self, cp, tgt, grid, info, patch):
        state = np.array([0,0, tgt[0]-cp[0], tgt[1]-cp[1]], np.float32)
        out = self.policy([patch[None,...], state[None,:], np.zeros((1,2),np.float32)], training=False)[0]
        dx, dy, v_norm = out.numpy()
        # corregir >90°
        vec_t = np.array([tgt[0]-cp[0], tgt[1]-cp[1]], np.float32)
        vec_n = np.array([dx, dy], np.float32)
        if np.dot(vec_n, vec_t) < 0:
            vec_n = vec_t/(np.linalg.norm(vec_t)+1e-6)
            dx,dy = vec_n
        # candidato directo
        if np.hypot(dx,dy)>1e-3:
            cand = (cp[0]+RADII[1]*dx, cp[1]+RADII[1]*dy)
            yaw = self._yaw_from_quaternion(self.pose.orientation)
            front = self._is_front(cand, cp, yaw)
            advance = v_norm>=0
            if ((front and advance) or (not front and not advance)) and self._candidate_ok(cand, cp, grid, info, first_wp=(self.wp_index<=1)):
                return cand, out
        # abanico clásico
        ang0 = math.atan2(dy,dy) if np.hypot(dx,dy)>1e-3 else math.atan2(tgt[1]-cp[1], tgt[0]-cp[0])
        for r in RADII:
            for off in ANGLES:
                cand = (cp[0]+r*math.cos(ang0+off), cp[1]+r*math.sin(ang0+off))
                if self._candidate_ok(cand, cp, grid, info, first_wp=(self.wp_index<=1)):
                    return cand, out
        # fallback 0
        return None, out

    # -------------- Pure pursuit filtrada ----------
    def follow_path(self, cp, v_norm):
        if not self.current_path or self.wp_index>=len(self.current_path): return
        # look-ahead proporcional
        look_h = max(0.25, 0.4*MAX_VEL*abs(v_norm))
        # encontrar idx
        idx = self.wp_index
        while idx+1<len(self.current_path) and l2(cp,self.current_path[idx])<look_h:
            idx+=1
        self.wp_index = idx
        tgt = self.current_path[idx]
        dx_g, dy_g = tgt[0]-cp[0], tgt[1]-cp[1]
        dist = math.hypot(dx_g,dy_g)
        if dist<1e-3: return
        yaw = self._yaw_from_quaternion(self.pose.orientation)
        dx,dy = self._global_to_local(dx_g,dy_g,yaw)
        alpha = math.atan2(dy,dx)
        kappa = 2.0*math.sin(alpha)/look_h
        # regular v_norm según ángulo
        if abs(alpha)<REV_ANGLE_MAX:
            v_norm = max(0.0, v_norm)
        else:
            v_norm = min(0.0, v_norm)
        v_cmd = np.clip(v_norm, -1, 1)*MAX_VEL
        v_filt = ALPHA_IIR_VEL*v_cmd + (1-ALPHA_IIR_VEL)*self.v_prev
        self.v_prev = v_filt
        if 0<abs(v_filt)<MIN_VEL:
            v_filt = math.copysign(MIN_VEL, v_filt)
        cmd = Twist()
        cmd.linear.x  = float(v_filt)
        cmd.angular.z = float(kappa*v_filt)
        self.cmd_pub.publish(cmd)

    # -------------- Publicar path en RViz --------
    def publish_path(self, pts):
        hdr = Header(frame_id="map", stamp=self.get_clock().now().to_msg())
        path = Path(header=hdr)
        for x,y in pts:
            ps = PoseStamped(header=hdr)
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)
        mk = Marker(header=hdr, ns="wps", id=0, type=Marker.POINTS, action=Marker.ADD)
        mk.scale = Vector3(x=0.15,y=0.15,z=0.0)
        mk.color.r=mk.color.g=1.0; mk.color.a=1.0
        mk.points = [Point(x=float(x),y=float(y)) for x,y in pts[1:]]
        self.wps_pub.publish(mk)

    # -------------- Ciclo principal -----------
    def step(self):
        if None in (self.pose, self.goal, self.grid_msg): return
        # evadir vuelco
        roll, pitch = self._yaw_from_quaternion(self.pose.orientation), 0
        if abs(roll)>MAX_TILT or abs(pitch)>MAX_TILT:
            return
        cp = (self.pose.position.x, self.pose.position.y)
        patch, grid, info = self.extract_patch()
        gp = (self.goal.position.x, self.goal.position.y)
        # seleccionar waypoint
        cand, out = self.next_waypoint(cp, gp, grid, info, patch)
        if cand is None: return
        # construir ruta corta
        self.current_path = [cp, cand]
        self.wp_index = 1
        # seguir y publicar
        self.follow_path(cp, out.numpy()[2])
        self.publish_path(self.current_path)


def main(args=None):
    rclpy.init(args=args)
    node = FlexPlannerInference()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
