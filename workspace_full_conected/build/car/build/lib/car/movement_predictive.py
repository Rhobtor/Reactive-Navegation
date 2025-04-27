#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dynamic_tracker_pred.py  · ROS 2 Humble

1. Detecta celdas dinámicas en un OccupancyGrid local (o proyectado).
2. Segmenta, asocia y estima velocidad en WORLD_FRAME (map/odom).
3. Publica:
      • Esfera naranja   → posición actual
      • Puntos celestes  → proyección lineal (t = +0.2 … +2 s)
      • Flecha verde     → si |v| > vel_arrow_min
      • /moving_objects/vel (Twist)

Funciona TODO en Python (NumPy + SciPy), sin PCL.
"""

import math, numpy as np
from scipy.ndimage import label, center_of_mass

import rclpy, tf2_ros
from rclpy.node   import Node
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg      import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from tf2_geometry_msgs  import PointStamped


def now_sec(clock):
    """ROS time → float s."""
    t = clock.now()
    return t.nanoseconds * 1e-9


class Track:
    def __init__(self, tid, xy, t):
        self.id = tid
        self.xy = xy            # np.array([x, y])  [m]
        self.v  = np.zeros(2)   # np.array([vx, vy]) [m/s]
        self.t  = t             # último time stamp (float s)

    def update(self, xy, t):
        dt = t - self.t
        if dt > 1e-4:
            self.v = (xy - self.xy) / dt
        self.xy = xy
        self.t  = t


class DynamicTracker(Node):
    def __init__(self):
        super().__init__('dynamic_tracker_pred')

        # ----------------- parámetros -----------------
        self.declare_parameter('input_grid',      '/occupancy_grid')
        self.declare_parameter('world_frame',     'map')   # o “odom”
        self.declare_parameter('dist_threshold',  0.60)    # [m] asociación NN
        self.declare_parameter('pred_horizon',    2.0)     # [s]
        self.declare_parameter('pred_dt',         0.20)    # [s]
        self.declare_parameter('vel_arrow_min',   0.03)    # [m/s]

        grid_topic   = self.get_parameter('input_grid').value
        self.world   = self.get_parameter('world_frame').value
        self.D       = self.get_parameter('dist_threshold').value
        self.horizon = self.get_parameter('pred_horizon').value
        self.pred_dt = self.get_parameter('pred_dt').value
        self.v_show  = self.get_parameter('vel_arrow_min').value

        # TF2
        self.tfbuf = tf2_ros.Buffer()
        self.tfl   = tf2_ros.TransformListener(self.tfbuf, self)

        # pubs/subs
        self.sub = self.create_subscription(OccupancyGrid, grid_topic,
                                            self.cb_grid, 10)
        self.pub_mk  = self.create_publisher(MarkerArray, '/moving_objects', 10)
        self.pub_vel = self.create_publisher(Twist,        '/moving_objects/vel', 10)

        # estado
        self.static     = None
        self.resolution = 0.05
        self.grid_frame = ''
        self.tracks     = {}
        self.next_id    = 0

        self.get_logger().info(f'📡  Escuchando {grid_topic}')

    # --------------------------------------------------
    def cb_grid(self, msg: OccupancyGrid):
        h, w = msg.info.height, msg.info.width
        grid = np.frombuffer(msg.data, dtype=np.int8).reshape(h, w)

        # (re)inicializa si cambió tamaño
        if self.static is None or self.static.shape != grid.shape:
            self.static     = grid.copy()   # snapshot "estático"
            self.resolution = msg.info.resolution
            self.grid_frame = msg.header.frame_id
            self.get_logger().warn('static_map reinicializado')
            return

        dyn = (grid != self.static) & (grid > 0)
        if not dyn.any():
            return

        lbl, n = label(dyn.astype(np.int8),
                       structure=[[0,1,0],[1,1,1],[0,1,0]])
        if n == 0:
            return

        centers = center_of_mass(dyn, lbl, index=range(1, n+1))  # (row, col)
        t_now   = now_sec(self.get_clock())

        updated = set()

        for r, c in centers:
            # punto en frame del grid
            p_grid = PointStamped()
            p_grid.header.frame_id = self.grid_frame
            p_grid.header.stamp    = msg.header.stamp   # tf necesita sello, aunque sea 0
            p_grid.point.x = (c + 0.5) * self.resolution
            p_grid.point.y = (r + 0.5) * self.resolution
            p_grid.point.z = 0.0

            # transforma a world
            try:
                p_world = self.tfbuf.transform(
                    p_grid, self.world,
                    timeout=rclpy.duration.Duration(seconds=0.05))
            except Exception as e:
                self.get_logger().warn(f'[TF2] {e}')
                continue

            xy = np.array([p_world.point.x, p_world.point.y])

            # -------- asociación Nearest-Neighbour ----------
            tid, best_d = None, self.D
            for id_, tr in self.tracks.items():
                d = np.linalg.norm(tr.xy - xy)
                if d < best_d:
                    tid, best_d = id_, d

            if tid is None:
                tid = self.next_id
                self.next_id += 1
                self.tracks[tid] = Track(tid, xy, t_now)

            self.tracks[tid].update(xy, t_now)
            updated.add(tid)

        # descarta tracks no vistos
        for id_ in list(self.tracks):
            if id_ not in updated:
                self.tracks.pop(id_)

        self.publish_markers()

    # --------------------------------------------------
    def publish_markers(self):
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        for tr in self.tracks.values():
            # ---- esfera actual ----
            m_cur = Marker()
            m_cur.header.frame_id = self.world
            m_cur.header.stamp    = stamp
            m_cur.ns, m_cur.id    = 'current', tr.id
            m_cur.type, m_cur.action = Marker.SPHERE, Marker.ADD
            m_cur.pose.position.x, m_cur.pose.position.y = tr.xy
            m_cur.pose.position.z = 0.15
            m_cur.scale.x = m_cur.scale.y = m_cur.scale.z = 0.25
            m_cur.color.r, m_cur.color.g, m_cur.color.b, m_cur.color.a = 1.0, 0.3, 0.0, 0.9
            ma.markers.append(m_cur)

            # ---- puntos de predicción ----
            m_pred = Marker()
            m_pred.header.frame_id = self.world
            m_pred.header.stamp    = stamp
            m_pred.ns, m_pred.id   = 'pred', tr.id
            m_pred.type, m_pred.action = Marker.POINTS, Marker.ADD
            m_pred.scale.x = m_pred.scale.y = 0.18
            m_pred.color.r, m_pred.color.g, m_pred.color.b, m_pred.color.a = 0.0, 0.6, 1.0, 0.9

            steps = int(self.horizon / self.pred_dt)
            for k in range(1, steps + 1):
                p = Point()
                p.x = float(tr.xy[0] + tr.v[0] * self.pred_dt * k)
                p.y = float(tr.xy[1] + tr.v[1] * self.pred_dt * k)
                p.z = 0.15
                m_pred.points.append(p)
            ma.markers.append(m_pred)

            # ---- flecha de dirección (opcional) ----
            speed = np.linalg.norm(tr.v)
            if speed >= self.v_show:
                m_arr = Marker()
                m_arr.header.frame_id = self.world
                m_arr.header.stamp    = stamp
                m_arr.ns, m_arr.id    = 'vel', tr.id
                m_arr.type, m_arr.action = Marker.ARROW, Marker.ADD
                m_arr.pose.position.x, m_arr.pose.position.y = tr.xy
                m_arr.pose.position.z = 0.2
                yaw = math.atan2(tr.v[1], tr.v[0])
                m_arr.pose.orientation.z = math.sin(yaw/2)
                m_arr.pose.orientation.w = math.cos(yaw/2)
                m_arr.scale.x, m_arr.scale.y, m_arr.scale.z = 0.35, 0.06, 0.06
                m_arr.color.r, m_arr.color.g, m_arr.color.b, m_arr.color.a = 0.0, 1.0, 0.2, 0.8
                ma.markers.append(m_arr)

        self.pub_mk.publish(ma)

        # demo: velocidad del primer track
        if self.tracks:
            tr0 = next(iter(self.tracks.values()))
            tw  = Twist()
            tw.linear.x, tw.linear.y = tr0.v
            self.pub_vel.publish(tw)
            # DEBUG
            self.get_logger().debug(f'ID {tr0.id}  v = {tr0.v} m/s')

# --------------------------------------------------
def main():
    rclpy.init()
    rclpy.spin(DynamicTracker())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
